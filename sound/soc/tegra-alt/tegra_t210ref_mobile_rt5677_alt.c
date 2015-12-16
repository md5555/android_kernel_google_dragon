/*
 * tegra_t210ref_mobile.c - Tegra T210 Machine driver for mobile
 *
 * Copyright (c) 2013-2015 NVIDIA CORPORATION.  All rights reserved.
 * Copyright (c) 2015 Google Inc.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/i2c.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include <linux/switch.h>
#include <linux/pm_runtime.h>
#include <linux/input.h>

#include <soc/tegra/sysedp.h>

#include <sound/core.h>
#include <sound/jack.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include "../codecs/rt5677.h"
#include "../codecs/nau8825.h"

#include "tegra_asoc_pdata.h"
#include "tegra_asoc_utils_alt.h"
#include "tegra_asoc_machine_alt.h"

#define DRV_NAME "tegra-snd-t210ref-mobile-rt5677"

#define GPIO_SPKR_EN    BIT(0)
#define GPIO_HP_MUTE    BIT(1)
#define GPIO_INT_MIC_EN BIT(2)
#define GPIO_EXT_MIC_EN BIT(3)
#define GPIO_HP_DET     BIT(4)
/* RT5677_MCLK is the frequency received by rt5677 MCLK1 pin. */
#define RT5677_MCLK	19200000
/* RT5677_SYSCLK is the frequency of the System clock in rt5677 used for
 * DACs/ADCs/DMICs. PLL1 converts 19200000 to 24576000.
 */
#define RT5677_SYSCLK	24576000

struct tegra_t210ref {
	struct tegra_asoc_platform_data *pdata;
	struct tegra_asoc_audio_clock_info audio_clock;
	unsigned int num_codec_links;
	int gpio_requested;
	bool clock_enabled;
	bool hotword_stream_active;
	struct snd_soc_jack jack;
	struct sysedp_consumer *sysedpc;
	const char *edp_name;
};

static struct snd_soc_pcm_stream tegra_rt5677_stream_params = {
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.channels_min = 2,
	.channels_max = 2,
};

static int tegra_t210ref_dai_init(struct snd_soc_pcm_runtime *rtd,
					int rate,
					int channels,
					u64 formats,
					int width)
{
	struct snd_soc_card *card = rtd->card;
	struct tegra_t210ref *machine = snd_soc_card_get_drvdata(card);
	struct snd_soc_pcm_stream *dai_params;
	unsigned int idx;
	int err, tdm_mask;

	/* A constant frequency is used to avoid switching MCLK while the
	 * rt5677 DSP fw is running for hotwording. The DSP expects a fixed
	 * 16KHz mono stream from a DMIC. This also avoids MCLK clock frequeny
	 * change at suspend/resume since tegra can output only a limited number
	 * of frequencies during suspend. e.g. 19.2MHz is supported, but not
	 * 24.576MHz.
	 *
	 * The MCLK output is configured to 19.2MHz at boot, coming from
	 * the oscillator. It never changes. tegra_alt_asoc_utils_set_rate
	 * won't affect MCLK because the pin is controlled via pmc registers.
	 *
	 * MCLK is asynchronous to BCLK/LRCK. RT5677 ASRC function ensures the
	 * constant MCLK works with various BCLK/LRCK frequencies on I2S1.
	 * Filters on those audio paths connected to I2S1 use Clock_i2s1_asrc
	 * which is derived from I2S1 clock.
	 *
	 * RT5677 I2S2 and I2S3 are in master mode, outputing BCLK/LRCK which
	 * are derived from MCLK. Filters on those audio paths connected to
	 * I2S2 and I2S3 use the Clock_system clock which is derived from MCLK.
	 *
	 * RT5677 PLL1 converts 19.2MHz MCLK to 24.576MHz Clock_system which is
	 * more suitable for 48K and 16K rates.
	 */
	err = tegra_alt_asoc_utils_set_rate(&machine->audio_clock,
				rate, RT5677_SYSCLK, RT5677_SYSCLK);
	if (err < 0) {
		dev_err(card->dev, "Can't configure clocks\n");
		return err;
	}

	idx = tegra_machine_get_codec_dai_link_idx("rt5677-playback");
	/* check if idx has valid number */
	if (idx != -EINVAL) {
		dai_params =
			(struct snd_soc_pcm_stream *)card->rtd[idx].dai_link->params;

		err = snd_soc_dai_set_pll(card->rtd[idx].codec_dai, 0,
			RT5677_PLL1_S_MCLK, RT5677_MCLK, RT5677_SYSCLK);
		if (err < 0) {
			dev_err(card->dev, "codec_dai pll not set\n");
			return err;
		}
		err = snd_soc_dai_set_sysclk(card->rtd[idx].codec_dai,
			RT5677_SCLK_S_PLL1, RT5677_SYSCLK, SND_SOC_CLOCK_IN);
		if (err < 0) {
			dev_err(card->dev, "codec_dai clock not set\n");
			return err;
		}

		/* Configure 5677 filter clocks and ASRC
		 *
		 * Clock_i2s1_asrc is enabled and derived from I2S1.
		 *
		 * Speaker playback path: Tegra -> 5677 I2S1 -> Stereo1 DAC
		 * (Clock_i2s1_asrc) -> Stereo3 ADC (Clock_system) -> 5677 I2S2
		 * -> MAX98357A
		 *
		 * Headphone playback path: Tegra -> 5677 I2S1 -> Stereo1 DAC
		 * (Clock_i2s1_asrc) -> Stereo3 ADC (Clock_system) -> 5677 I2S3
		 * -> NAU8825
		 *
		 * DMIC capture path: 4 DMICS -> Stereo1 ADC (Clock_i2s1_asrc) +
		 * Stereo2 ADC (Clock_i2s1_asrc) -> 5677 I2S1 -> Tegra
		 *
		 * AMIC capture path: AMIC -> NAU8825 -> 5677 I2S3 -> DAC MONO3
		 * DD_MIX1 (Clock_system) -> Stereo1 ADC (Clock_i2s1_asrc) ->
		 * 5677 I2S1 -> Tegra
		 *
		 * DMIC hotword path: 1 DMIC -> Mono ADC L (Clock_system5) ->
		 * VAD ADC -> 5677 DSP (Configured in rt5677-hotword.c)
		 */
		rt5677_sel_asrc_clk_src(card->rtd[idx].codec_dai->codec,
			RT5677_DA_STEREO_FILTER | RT5677_AD_STEREO1_FILTER |
			RT5677_AD_STEREO2_FILTER | RT5677_I2S1_SOURCE,
			RT5677_CLK_SEL_I2S1_ASRC);
		rt5677_sel_asrc_clk_src(card->rtd[idx].codec_dai->codec,
			RT5677_AD_STEREO3_FILTER | RT5677_DA_MONO3_L_FILTER |
			RT5677_DA_MONO3_R_FILTER,
			RT5677_CLK_SEL_SYS);

		/* update link_param to update hw_param for DAPM */
		dai_params->rate_min = rate;
		dai_params->channels_min = channels;
		dai_params->formats = formats;

		err = snd_soc_dai_set_bclk_ratio(card->rtd[idx].cpu_dai,
		tegra_machine_get_bclk_ratio(&card->rtd[idx]));
		if (err < 0) {
			dev_err(card->dev, "Can't set cpu dai bclk ratio\n");
			return err;
		}

		tdm_mask = (1 << channels) - 1;
		snd_soc_dai_set_tdm_slot(card->rtd[idx].codec_dai, tdm_mask, tdm_mask,
			channels, width);
		snd_soc_dai_set_tdm_slot(card->rtd[idx].cpu_dai, tdm_mask, tdm_mask,
			channels, width);
	}

	return 0;
}

static int tegra_t210ref_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	int err;
	int srate = params_rate(params);

	err = tegra_t210ref_dai_init(rtd, srate,
			params_channels(params),
			(1ULL << (params_format(params))),
			params_width(params));
	if (err < 0) {
		dev_err(card->dev, "Failed dai init\n");
		return err;
	}

	/* both dai interfaces use the same bit clock,
	* set rate for both interfaces the same, otherwise driver might not
	* be able to configure clock divider.
	*/
	tegra_rt5677_stream_params.rate_min = srate;
	tegra_rt5677_stream_params.rate_max = srate;

	return 0;
}

static int tegra_t210ref_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_card *card = rtd->card;
	struct tegra_t210ref *machine = snd_soc_card_get_drvdata(card);
	struct snd_soc_pcm_stream *dai_params =
		(struct snd_soc_pcm_stream *)rtd->dai_link->params;
	unsigned int srate;
	int err;

	srate = dai_params->rate_min;

	err = tegra_alt_asoc_utils_set_extern_parent(&machine->audio_clock,
							"pll_a_out0");
	if (err < 0) {
		dev_err(card->dev, "Failed to set extern clk parent\n");
		return err;
	}

	return 0;
}

static int tegra_t210ref_sfc_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	unsigned int in_srate, out_srate;
	int err;

	in_srate = 48000;
	out_srate = 8000;

	err = snd_soc_dai_set_sysclk(codec_dai, 0, out_srate,
					SND_SOC_CLOCK_OUT);
	err = snd_soc_dai_set_sysclk(codec_dai, 0, in_srate,
					SND_SOC_CLOCK_IN);

	return 0;
}


static int tegra_rt5677_event_int_spk(struct snd_soc_dapm_widget *w,
					struct snd_kcontrol *k, int event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct tegra_t210ref *machine = snd_soc_card_get_drvdata(card);
	struct tegra_asoc_platform_data *pdata = machine->pdata;

	if (!(machine->gpio_requested & GPIO_SPKR_EN))
		return 0;

	if (SND_SOC_DAPM_EVENT_ON(event))
		sysedp_set_state(machine->sysedpc, 1);

	gpio_set_value_cansleep(pdata->gpio_spkr_en,
				!!SND_SOC_DAPM_EVENT_ON(event));

	if (!SND_SOC_DAPM_EVENT_ON(event))
		sysedp_set_state(machine->sysedpc, 0);

	return 0;
}

static int tegra_rt5677_event_hp(struct snd_soc_dapm_widget *w,
					struct snd_kcontrol *k, int event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct tegra_t210ref *machine = snd_soc_card_get_drvdata(card);
	struct tegra_asoc_platform_data *pdata = machine->pdata;

	if (!(machine->gpio_requested & GPIO_HP_MUTE))
		return 0;

	gpio_set_value_cansleep(pdata->gpio_hp_mute,
				!SND_SOC_DAPM_EVENT_ON(event));

	return 0;
}

static int tegra_rt5677_event_int_mic(struct snd_soc_dapm_widget *w,
					struct snd_kcontrol *k, int event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct tegra_t210ref *machine = snd_soc_card_get_drvdata(card);
	struct tegra_asoc_platform_data *pdata = machine->pdata;

	if (!(machine->gpio_requested & GPIO_INT_MIC_EN))
		return 0;

	gpio_set_value_cansleep(pdata->gpio_int_mic_en,
				!!SND_SOC_DAPM_EVENT_ON(event));

	return 0;
}

static int tegra_rt5677_event_ext_mic(struct snd_soc_dapm_widget *w,
					struct snd_kcontrol *k, int event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct tegra_t210ref *machine = snd_soc_card_get_drvdata(card);
	struct tegra_asoc_platform_data *pdata = machine->pdata;

	if (!(machine->gpio_requested & GPIO_EXT_MIC_EN))
		return 0;

	gpio_set_value_cansleep(pdata->gpio_ext_mic_en,
				!SND_SOC_DAPM_EVENT_ON(event));

	return 0;
}

static struct snd_soc_ops tegra_t210ref_ops = {
	.hw_params = tegra_t210ref_hw_params,
};

static const struct snd_soc_dapm_widget tegra_t210ref_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", tegra_rt5677_event_hp),
	SND_SOC_DAPM_SPK("Int Spk", tegra_rt5677_event_int_spk),
	SND_SOC_DAPM_MIC("Int Mic", tegra_rt5677_event_int_mic),
	SND_SOC_DAPM_MIC("Mic Jack", tegra_rt5677_event_ext_mic),
};

static int tegra_t210ref_suspend_pre(struct snd_soc_card *card)
{
	unsigned int idx;

	/* DAPM dai link stream work for non pcm links */
	for (idx = 0; idx < card->num_rtd; idx++) {
		if (card->rtd[idx].dai_link->params)
			INIT_DELAYED_WORK(&card->rtd[idx].delayed_work, NULL);
	}

	return 0;
}

static int tegra_t210ref_suspend_post(struct snd_soc_card *card)
{
	struct tegra_t210ref *machine = snd_soc_card_get_drvdata(card);

	if (!machine->hotword_stream_active && machine->clock_enabled) {
		machine->clock_enabled = false;
		tegra_alt_asoc_utils_clk_disable(&machine->audio_clock);
	}

	return 0;
}

static int tegra_t210ref_resume_pre(struct snd_soc_card *card)
{
	struct tegra_t210ref *machine = snd_soc_card_get_drvdata(card);

	if (!machine->clock_enabled) {
		machine->clock_enabled = true;
		tegra_alt_asoc_utils_clk_enable(&machine->audio_clock);
	}

	return 0;
}

static int tegra_rt5677_headset_init(struct snd_soc_pcm_runtime *runtime)
{
	struct snd_soc_card *card = runtime->card;
	struct snd_soc_codec *codec = runtime->codec;
	struct tegra_t210ref *machine = snd_soc_card_get_drvdata(card);
	struct snd_soc_jack *jack;
	int ret;

	snd_soc_codec_set_sysclk(codec, NAU8825_CLK_MCLK, 0, 0, SND_SOC_CLOCK_IN);

	/* Enable Headset and 4 Buttons Jack detection */
	ret = snd_soc_card_jack_new(card, "Headset Jack",
	                            SND_JACK_HEADPHONE | SND_JACK_MICROPHONE |
	                            SND_JACK_BTN_0 | SND_JACK_BTN_1 |
	                            SND_JACK_BTN_2 | SND_JACK_BTN_3,
	                            &machine->jack, NULL, 0);
	if (ret) {
		dev_err(card->dev, "New Headset Jack failed! (%d)\n", ret);
		return ret;
	}

	jack = &machine->jack;

	snd_jack_set_key(jack->jack, SND_JACK_BTN_0, KEY_MEDIA);
	snd_jack_set_key(jack->jack, SND_JACK_BTN_1, KEY_VOICECOMMAND);
	snd_jack_set_key(jack->jack, SND_JACK_BTN_2, KEY_VOLUMEUP);
	snd_jack_set_key(jack->jack, SND_JACK_BTN_3, KEY_VOLUMEDOWN);
	nau8825_enable_jack_detect(codec, jack);

	return 0;
}

static int tegra_rt5677_hotword_init(struct snd_soc_pcm_runtime *runtime)
{
	struct snd_soc_card *card = runtime->card;
	struct snd_soc_codec *codec = runtime->codec;
	int ret;

	ret = snd_soc_dai_set_pll(runtime->codec_dai, 0,
		RT5677_PLL1_S_MCLK, RT5677_MCLK, RT5677_SYSCLK);
	if (ret < 0) {
		dev_err(card->dev, "codec_dai pll not set\n");
		return ret;
	}
	ret = snd_soc_dai_set_sysclk(runtime->codec_dai,
		RT5677_SCLK_S_PLL1, RT5677_SYSCLK, SND_SOC_CLOCK_IN);
	if (ret < 0) {
		dev_err(card->dev, "codec_dai clock not set\n");
		return ret;
	}

	snd_soc_dapm_ignore_suspend(&card->dapm, "Int Mic");
	snd_soc_dapm_ignore_suspend(&codec->dapm, "DSP Buffer");
	return 0;
}

static const struct snd_kcontrol_new tegra_t210ref_controls[] = {
	SOC_DAPM_PIN_SWITCH("Int Spk"),
	SOC_DAPM_PIN_SWITCH("Headphone Jack"),
	SOC_DAPM_PIN_SWITCH("Mic Jack"),
	SOC_DAPM_PIN_SWITCH("Int Mic"),
};

static int tegra_rt5677_hotword_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct tegra_t210ref *machine = snd_soc_card_get_drvdata(card);

	machine->hotword_stream_active = true;
	return 0;
}

static void tegra_rt5677_hotword_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct tegra_t210ref *machine = snd_soc_card_get_drvdata(card);

	machine->hotword_stream_active = false;
}

static struct snd_soc_ops tegra_rt5677_hotword_ops = {
	.startup = tegra_rt5677_hotword_startup,
	.shutdown = tegra_rt5677_hotword_shutdown,
};

static struct snd_soc_card snd_soc_tegra_t210ref = {
	.name = "tegra-t210ref",
	.owner = THIS_MODULE,
	.suspend_post = tegra_t210ref_suspend_post,
	.suspend_pre = tegra_t210ref_suspend_pre,
	.resume_pre = tegra_t210ref_resume_pre,
	.controls = tegra_t210ref_controls,
	.num_controls = ARRAY_SIZE(tegra_t210ref_controls),
	.dapm_widgets = tegra_t210ref_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(tegra_t210ref_dapm_widgets),
	.fully_routed = true,
};

static struct snd_soc_dai_link tegra_rt5677_dai[] = {
	{
		.name = "MAX98357A",
		.stream_name = "Speaker",
		.codec_name = "max98357a",
		.cpu_dai_name = "rt5677-aif2",
		.codec_dai_name = "HiFi",
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS,
		.params = &tegra_rt5677_stream_params,
		.playback_only = true,
	}, {
		.name = "nau8825",
		.stream_name = "Headset",
		.codec_name = "nau8825.5-001a",
		.cpu_dai_name = "rt5677-aif3",
		.codec_dai_name = "nau8825-hifi",
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS,
		.params = &tegra_rt5677_stream_params,
		.init = tegra_rt5677_headset_init,
	},

	/* Hotwording PCM */
	{
		.name = "Codec DSP",
		.stream_name = "Wake on Voice",
		.cpu_name = "spi32766.0",
		.cpu_dai_name = "spi32766.0",
		.codec_name = "rt5677.5-002d",
		.codec_dai_name = "rt5677-dspbuffer",
		.platform_name = "spi32766.0",
		.ignore_suspend = 1,
		.init = tegra_rt5677_hotword_init,
		.ops = &tegra_rt5677_hotword_ops,
	},
};

static int tegra_t210ref_set_mclk(struct tegra_t210ref *machine, struct device *dev)
{
	int err;

	err = tegra_alt_asoc_utils_set_rate(&machine->audio_clock,
			48000, RT5677_SYSCLK, RT5677_SYSCLK);
	if (err < 0) {
		dev_err(dev, "Can't configure clocks\n");
		return err;
	}

	return 0;
}

static int tegra_t210ref_driver_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct snd_soc_card *card = &snd_soc_tegra_t210ref;
	struct tegra_t210ref *machine;
	struct snd_soc_dai_link *tegra_machine_dai_links = NULL;
	struct snd_soc_dai_link *tegra_t210ref_codec_links = NULL;
	struct snd_soc_codec_conf *tegra_machine_codec_conf = NULL;
	struct snd_soc_codec_conf *tegra_t210ref_codec_conf = NULL;
	struct tegra_asoc_platform_data *pdata = NULL;
	struct snd_soc_codec *codec = NULL;
	int ret = 0, i, idx;

	if (!np) {
		dev_err(&pdev->dev, "No device tree node for t210ref driver");
		return -ENODEV;
	}

	machine = devm_kzalloc(&pdev->dev, sizeof(struct tegra_t210ref),
			       GFP_KERNEL);
	if (!machine) {
		dev_err(&pdev->dev, "Can't allocate tegra_t210ref struct\n");
		ret = -ENOMEM;
		goto err;
	}

	card->dev = &pdev->dev;
	platform_set_drvdata(pdev, card);
	snd_soc_card_set_drvdata(card, machine);

	ret = snd_soc_of_parse_card_name(card, "nvidia,model");
	if (ret)
		goto err;

	ret = snd_soc_of_parse_audio_routing(card,
				"nvidia,audio-routing");
	if (ret)
		goto err;

	/* set new codec links and conf */
	tegra_t210ref_codec_links = tegra_machine_new_codec_links(pdev,
		tegra_t210ref_codec_links,
		&machine->num_codec_links);
	if (!tegra_t210ref_codec_links)
		goto err_alloc_dai_link;

	tegra_t210ref_codec_conf = tegra_machine_new_codec_conf(pdev,
		tegra_t210ref_codec_conf,
		&machine->num_codec_links);
	if (!tegra_t210ref_codec_conf)
		goto err_alloc_dai_link;


	/* get the xbar dai link/codec conf structure */
	tegra_machine_dai_links = tegra_machine_get_dai_link();
	if (!tegra_machine_dai_links)
		goto err_alloc_dai_link;
	tegra_machine_codec_conf = tegra_machine_get_codec_conf();
	if (!tegra_machine_codec_conf)
		goto err_alloc_dai_link;

	tegra_t210ref_codec_links[1].init = tegra_t210ref_init;

	/* set ADMAIF dai_ops */
	for (i = TEGRA210_DAI_LINK_ADMAIF1;
		i <= TEGRA210_DAI_LINK_ADMAIF10; i++)
		tegra_machine_set_dai_ops(i, &tegra_t210ref_ops);


	/* set sfc dai_init */
	tegra_machine_set_dai_init(TEGRA210_DAI_LINK_SFC1_RX,
		&tegra_t210ref_sfc_init);

	/* append t210ref specific dai_links */
	card->num_links =
		tegra_machine_append_dai_link(tegra_t210ref_codec_links,
			2 * machine->num_codec_links);
	card->num_links =
		tegra_machine_append_dai_link(tegra_rt5677_dai,
			ARRAY_SIZE(tegra_rt5677_dai));

	tegra_machine_dai_links = tegra_machine_get_dai_link();
	card->dai_link = tegra_machine_dai_links;

	/* append t210ref specific codec_conf */
	card->num_configs =
		tegra_machine_append_codec_conf(tegra_t210ref_codec_conf,
			machine->num_codec_links);
	tegra_machine_codec_conf = tegra_machine_get_codec_conf();
	card->codec_conf = tegra_machine_codec_conf;

	pdata = devm_kzalloc(&pdev->dev,
				sizeof(struct tegra_asoc_platform_data),
				GFP_KERNEL);
	if (!pdata) {
		dev_err(&pdev->dev,
			"Can't allocate tegra_asoc_platform_data struct\n");
		return -ENOMEM;
	}

	pdata->gpio_spkr_en = of_get_named_gpio(np,
		"nvidia,spkr-en-gpios", 0);
	if (pdata->gpio_spkr_en == -EPROBE_DEFER)
		return -EPROBE_DEFER;
	if (gpio_is_valid(pdata->gpio_spkr_en)) {
		ret = devm_gpio_request_one(&pdev->dev,
			pdata->gpio_spkr_en, GPIOF_OUT_INIT_LOW,
			"speaker_en");
		if (ret) {
			dev_err(card->dev, "cannot get speaker_en gpio\n");
			return ret;
		}
		machine->gpio_requested |= GPIO_SPKR_EN;
	}

	pdata->gpio_int_mic_en = of_get_named_gpio(np,
		"nvidia,dmic-clk-en-gpios", 0);
	if (pdata->gpio_int_mic_en == -EPROBE_DEFER)
		return -EPROBE_DEFER;
	if (gpio_is_valid(pdata->gpio_int_mic_en)) {
		ret = devm_gpio_request_one(&pdev->dev,
			pdata->gpio_int_mic_en, GPIOF_OUT_INIT_LOW,
			"dmic_clk_en");
		if (ret) {
			dev_err(card->dev, "cannot get dmic_clk_en gpio\n");
			return ret;
		}
		machine->gpio_requested |= GPIO_INT_MIC_EN;
	}

	pdata->gpio_codec1 = pdata->gpio_codec2 = pdata->gpio_codec3 =
	pdata->gpio_hp_mute = pdata->gpio_ext_mic_en = -1;

	machine->pdata = pdata;

	ret = tegra_alt_asoc_utils_init(&machine->audio_clock,
					&pdev->dev,
					card);
	if (ret)
		goto err_alloc_dai_link;
	machine->clock_enabled = true;

	ret = tegra_t210ref_set_mclk(machine, &pdev->dev);
	if (ret)
		goto err_alloc_dai_link;

	card->dapm.idle_bias_off = 1;

	ret = snd_soc_register_card(card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n",
			ret);
		goto err_alloc_dai_link;
	}

	idx = tegra_machine_get_codec_dai_link_idx("rt5677-playback");
	/* check if idx has valid number */
	if (idx == -EINVAL)
		return idx;

	codec = card->rtd[idx].codec;

	if (of_property_read_string(np, "edp-consumer-name",
				    &machine->edp_name))
		machine->edp_name = "codec+speaker";
	machine->sysedpc = sysedp_create_consumer(np, machine->edp_name);

	return 0;

err_alloc_dai_link:
	tegra_machine_remove_dai_link();
	tegra_machine_remove_codec_conf();
err:
	return ret;
}

static int tegra_t210ref_driver_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct tegra_t210ref *machine = snd_soc_card_get_drvdata(card);

	snd_soc_unregister_card(card);

	tegra_machine_remove_dai_link();
	tegra_machine_remove_codec_conf();
	tegra_alt_asoc_utils_fini(&machine->audio_clock);

	sysedp_free_consumer(machine->sysedpc);
	machine->sysedpc = NULL;

	return 0;
}

static const struct of_device_id tegra_t210ref_of_match[] = {
	{ .compatible = "nvidia,tegra-audio-t210ref-mobile-rt5677", },
	{},
};

static struct platform_driver tegra_t210ref_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
		.of_match_table = tegra_t210ref_of_match,
	},
	.probe = tegra_t210ref_driver_probe,
	.remove = tegra_t210ref_driver_remove,
};
module_platform_driver(tegra_t210ref_driver);

MODULE_AUTHOR("Anatol Pomazau <anatol@google.com>");
MODULE_DESCRIPTION("Tegra+t210ref+rt5677 machine ASoC driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_DEVICE_TABLE(of, tegra_t210ref_of_match);
