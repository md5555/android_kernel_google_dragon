/*
 * rt5677-hotword.c  --  RT5677 ALSA SoC audio codec driver
 *
 * Copyright 2013 Realtek Semiconductor Corp.
 * Author: Oder Chiou <oder_chiou@realtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/regmap.h>
#include <linux/i2c.h>
#include <sound/soc.h>
#include <linux/wakelock.h>

#include "rl6231.h"
#include "rt5677.h"
#include "rt5677-spi.h"
#include "rt5677-hotword.h"

#define RT5677_PR_RANGE_BASE (0xff + 1)
#define RT5677_PR_SPACING 0x100
#define RT5677_PR_BASE (RT5677_PR_RANGE_BASE + (0 * RT5677_PR_SPACING))

#define RT5677_DSP_BOOT_VECTOR		0x1801f090
#define RT5677_MIC_BUF_ADDR		0x60030000
#define RT5677_MODEL_ADDR		0x5FFC9800
#define RT5677_MIC_BUF_BYTES		((u32)(0x20000 - sizeof(u32)))
#define RT5677_MIC_BUF_FIRST_READ_SIZE	0x10000
#define RT5677_HOTWORD_START_DELAY_MS	500
#define RT5677_HOTWORD_STREAM_DELAY_MS	10

enum rt5677_hotword_state {
	RT5677_HOTWORD_IDLE,
	RT5677_HOTWORD_START,
	RT5677_HOTWORD_ARMED,
	RT5677_HOTWORD_FIRED,
	RT5677_HOTWORD_STREAM,
	RT5677_HOTWORD_STOP,
};

struct rt5677_dsp {
	struct device *spi;
	struct rt5677_priv *priv;
	struct delayed_work work;
	struct mutex lock;
	struct snd_pcm_substream *substream;
	size_t dma_offset;	/* zero-based offset into runtime->dma_area */
	size_t avail_bytes;	/* number of new bytes since last period */
	u32 mic_read_offset;	/* zero-based offset into DSP's mic buffer */
	enum rt5677_hotword_state state;
	struct wake_lock wake_lock;
	u8 *model_buf;
	unsigned int model_len;
};

static const struct snd_pcm_hardware rt5677_hotword_pcm_hardware = {
	.info			= SNDRV_PCM_INFO_MMAP |
				  SNDRV_PCM_INFO_MMAP_VALID |
				  SNDRV_PCM_INFO_INTERLEAVED,
	.formats		= SNDRV_PCM_FMTBIT_S16_LE,
	.period_bytes_min	= PAGE_SIZE,
	.period_bytes_max	= 0x20000 / 8,
	.periods_min		= 8,
	.periods_max		= 8,
	.channels_min		= 1,
	.channels_max		= 1,
	.buffer_bytes_max	= 0x20000,
};

static int rt5677_hotword_model_put(struct snd_kcontrol *kcontrol,
		const unsigned int __user *bytes, unsigned int size)
{

	struct snd_soc_platform *platform = snd_soc_kcontrol_platform(kcontrol);
	struct rt5677_dsp *dsp = snd_soc_platform_get_drvdata(platform);
	int ret = 0;

	mutex_lock(&dsp->lock);
	if (!dsp->model_buf || dsp->model_len < size) {
		if (dsp->model_buf)
			devm_kfree(dsp->spi, dsp->model_buf);
		dsp->model_buf = devm_kmalloc(dsp->spi, size, GFP_KERNEL);
		if (!dsp->model_buf) {
			ret = -ENOMEM;
			goto done;
		}
	}

	if (copy_from_user(dsp->model_buf, bytes, size))
		ret = -EFAULT;

done:
	dsp->model_len = (ret ? 0 : size);
	mutex_unlock(&dsp->lock);
	return ret;
}

static const struct snd_kcontrol_new rt5677_hotword_snd_controls[] = {
	/* Size of the model is limited to 0x50000 bytes */
	SND_SOC_BYTES_TLV("Hotword Model", 0x50000,
		NULL, rt5677_hotword_model_put),
};

static int rt5677_hotword_load_fw(struct rt5677_dsp *dsp, const u8 *buf,
		unsigned int len)
{
	Elf32_Ehdr *elf_hdr;
	Elf32_Phdr *pr_hdr;
	Elf32_Half i;
	int ret = 0;
	u32 zero = 0;

	if (!buf || (len < sizeof(Elf32_Ehdr)))
		return -ENOMEM;

	elf_hdr = (Elf32_Ehdr *)buf;
#ifndef EM_XTENSA
#define EM_XTENSA	94
#endif
	if (strncmp(elf_hdr->e_ident, ELFMAG, sizeof(ELFMAG) - 1))
		dev_err(dsp->spi, "Wrong ELF header prefix\n");
	if (elf_hdr->e_ehsize != sizeof(Elf32_Ehdr))
		dev_err(dsp->spi, "Wrong Elf header size\n");
	if (elf_hdr->e_machine != EM_XTENSA)
		dev_err(dsp->spi, "Wrong DSP code file\n");

	if (len < elf_hdr->e_phoff)
		return -ENOMEM;
	pr_hdr = (Elf32_Phdr *)(buf + elf_hdr->e_phoff);
	for (i = 0; i < elf_hdr->e_phnum; i++) {
		/* TODO: handle p_memsz != p_filesz */
		if (pr_hdr->p_paddr && pr_hdr->p_filesz) {
			ret = rt5677_spi_write(pr_hdr->p_paddr,
					buf + pr_hdr->p_offset,
					pr_hdr->p_filesz);
			if (ret)
				dev_err(dsp->spi, "Load firmware failed %d\n",
						ret);
		}
		pr_hdr++;
	}

	/* The dsp firmware doesn't work without this. The 4 bytes
	 * [0x5ffe0374, 0x5ffe0377] near the end of .ResetVector.text section
	 * have to be zeros. Why??
	 */
	rt5677_spi_write(0x5ffe0374, &zero, sizeof(zero));
	return ret;
}

static int rt5677_hotword_load_fw_from_file(struct rt5677_dsp *dsp)
{
	const struct firmware *fwp;
	int ret = 0;

	/* Load dsp firmware from rt5677_elf_vad file */
	ret = request_firmware(&fwp, "rt5677_elf_vad", dsp->spi);
	if (ret) {
		dev_err(dsp->spi, "Request rt5677_elf_vad failed %d\n", ret);
		return ret;
	}
	dev_info(dsp->spi, "Requested rt5677_elf_vad (%zu)\n", fwp->size);

	ret = rt5677_hotword_load_fw(dsp, fwp->data, fwp->size);
	release_firmware(fwp);
	if (ret)
		goto error;

	/* Load hotword language model */
	if (dsp->model_buf && dsp->model_len) {
		ret = rt5677_spi_write(RT5677_MODEL_ADDR, dsp->model_buf,
				       dsp->model_len);
		if (ret) {
			dev_err(dsp->spi, "Model load failed %d\n", ret);
			goto error;
		}
		dev_info(dsp->spi, "Loaded model (%u)\n", dsp->model_len);
	}
error:
	return ret;
}

static void rt5677_hotword_setup_audio_path(struct rt5677_priv *rt5677)
{
	int pre_div;
	struct device *dev = rt5677->codec->dev;
	/* The hotword audio path should be controlled by the ucm config.
	 *
	 * For example, to use a DMIC for hotwording, ucm config should setup:
	 * "DMIC L1" ->
	 * "DMIC1" ->
	 * "Mono DMIC L Mux" ->
	 * "Mono ADC2 L Mux" ->
	 * "Mono ADC MIXL" ->
	 * "VAD ADC Mux" ->
	 * "IB01 Mux" ->
	 * "IB01 Bypass Mux" ->
	 * "DSPTX" ->
	 * "DSP Buffer" ->
	 * "DSP Capture".
	 *
	 * "DSP Capture" is the rt5677-hotword-cpu-dai with a PCM interface.
	 */

	/* Calculate sel_i2s_pre_div5 for clk_sys5, based on MCLK input.
	 * The DSP is expecting 16KHz sample rate.
	 */
	pre_div = rl6231_get_clk_info(rt5677->sysclk, 16000);
	if (pre_div < 0) {
		/* If the MCLK frequency is not known at this time, we use
		 * a pre_div of 6 by default, assuming MCLK is 24567000Hz:
		 * 16000 * 256 * 6 = 24576000
		 */
		dev_info(dev, "Unknown sysclk(%dHz), using default pre_div=6\n",
			rt5677->sysclk);
		pre_div = 4;
	}
	regmap_update_bits(rt5677->regmap, RT5677_CLK_TREE_CTRL2,
		RT5677_I2S_PD5_MASK, pre_div << RT5677_I2S_PD5_SFT);

	/* Clock source for Mono L ADC = clk_sys5 */
	rt5677_sel_asrc_clk_src(rt5677->codec, RT5677_AD_MONO_L_FILTER,
		RT5677_CLK_SEL_SYS5);
}

static void rt5677_hotword_start(struct rt5677_dsp *dsp)
{
	struct rt5677_priv *rt5677 = dsp->priv;
	u32 boot_vector[2] = {0x00000009, 0x00000019};

	rt5677_hotword_setup_audio_path(rt5677);

	/* Clear DSP bus clock setting to default to ensure DSP bus clock >=
	 * SPI clock during fw load. The DSP fw modifies these bus clock
	 * settings to save power in SysInit.
	 * DSP BCLK Auto Mode = disable
	 * DSP BCLK pre div in auto mode = /2
	 * DSP BCLK pre div = /2
	 */
	regmap_update_bits(rt5677->regmap, RT5677_GEN_CTRL2,
		RT5677_DSP_BCLK_AUTO_MODE_MASK |
		RT5677_DSP_BUS_PRE_DIV_AUTO_MASK |
		RT5677_DSP_BUS_PRE_DIV_MASK, 0);

	/* DSP Clock = MCLK1 (bypassed PLL2) */
	regmap_write(rt5677->regmap, RT5677_GLB_CLK2, 0x0080);

	/* Enable Gating Mode with MCLK = enable */
	regmap_update_bits(rt5677->regmap, RT5677_DIG_MISC, 0x1, 0x1);

	regmap_update_bits(rt5677->regmap, RT5677_PR_BASE + RT5677_BIAS_CUR4,
		0x0f00, 0x0100);

	/* LDO2 output = 1.2V
	 * LDO1 output = 1.2V (LDO_IN = 1.8V)
	 */
	regmap_update_bits(rt5677->regmap, RT5677_PWR_ANLG1,
		RT5677_LDO1_SEL_MASK | RT5677_LDO2_SEL_MASK,
		0x0055);

	/* Codec core power =  power on
	 * LDO1 power = power on
	 */
	regmap_update_bits(rt5677->regmap, RT5677_PWR_ANLG2,
		RT5677_PWR_CORE | RT5677_PWR_LDO1,
		RT5677_PWR_CORE | RT5677_PWR_LDO1);

	/* Isolation for DCVDD4 = normal (set during probe)
	 * Isolation for DCVDD2 = normal (set during probe)
	 * Isolation for DSP = normal
	 * Isolation for Band 0~7 = disable
	 * Isolation for InBound 4~10 and OutBound 4~10 = disable
	 */
	regmap_write(rt5677->regmap, RT5677_PWR_DSP2, 0x07ff);

	/* System Band 0~7 = power on
	 * InBound 4~10 and OutBound 4~10 = power on
	 * DSP = power on
	 * DSP CPU = stop (will be set to "run" after firmware loaded)
	 */
	regmap_write(rt5677->regmap, RT5677_PWR_DSP1, 0x07ff);
	rt5677->is_dsp_mode = true;

	/* Boot the firmware from IRAM instead of SRAM0. */
	rt5677_spi_write(RT5677_DSP_BOOT_VECTOR, boot_vector, sizeof(u32));
	rt5677_spi_write(RT5677_DSP_BOOT_VECTOR, boot_vector + 1, sizeof(u32));
	rt5677_spi_write(RT5677_DSP_BOOT_VECTOR, boot_vector, sizeof(u32));

	rt5677_hotword_load_fw_from_file(dsp);

	/* Set DSP CPU to Run */
	regmap_update_bits(rt5677->regmap, RT5677_PWR_DSP1, 0x1, 0x0);
	dsp->state = RT5677_HOTWORD_ARMED;
	wake_unlock(&dsp->wake_lock);
}

static void rt5677_hotword_stop(struct rt5677_dsp *dsp)
{
	struct rt5677_priv *rt5677 = dsp->priv;
	/* Set DSP CPU to Stop */
	regmap_update_bits(rt5677->regmap, RT5677_PWR_DSP1, 0x1, 0x1);
	/* Power off DSP */
	regmap_update_bits(rt5677->regmap, RT5677_PWR_DSP1, 0x2, 0x0);
	rt5677->is_dsp_mode = false;
	regmap_write(rt5677->regmap, RT5677_PWR_DSP1, 0x0001);
	dsp->state = RT5677_HOTWORD_IDLE;
}

static int rt5677_hotword_mic_write_offset(u32 *mic_write_offset)
{
	int ret;
	/* Grab the first 4 bytes that hold the write pointer on the
	 * dsp, and check to make sure that it points somewhere inside the
	 * buffer.
	 */
	*mic_write_offset = 0;
	ret = rt5677_spi_read(RT5677_MIC_BUF_ADDR, mic_write_offset,
			sizeof(u32));
	if (ret)
		return ret;
	/* Adjust the offset so that it's zero-based */
	*mic_write_offset = *mic_write_offset - sizeof(u32);
	return *mic_write_offset < RT5677_MIC_BUF_BYTES ? 0 : -EFAULT;
}

/*
 * Copy one contiguous block of audio samples from the DSP mic buffer to the
 * dma_area of the pcm runtime. The receiving buffer may wrap around.
 * @begin: start offset of the block to copy, in bytes.
 * @end:   offset of the first byte after the block to copy, must be greater
 *         than or equal to begin.
 *
 * Return: Zero if successful, or a negative error code on failure.
 */
static int rt5677_hotword_copy_block(struct rt5677_dsp *dsp, u32 begin, u32 end)
{
	struct snd_pcm_runtime *runtime = dsp->substream->runtime;
	size_t bytes_per_frame = frames_to_bytes(runtime, 1);
	size_t first_chunk_len, second_chunk_len;
	int ret;

	if (begin > end || runtime->dma_bytes < 2 * bytes_per_frame) {
		dev_err(dsp->spi,
			"Invalid copy from (%u, %u), dma_area size %zu\n",
			begin, end, runtime->dma_bytes);
		return -EINVAL;
	}

	/* The block to copy is empty */
	if (begin == end)
		return 0;

	/* If the incoming chunk is too big for the receiving buffer, only the
	 * last "receiving buffer size - one frame" bytes are copied.
	 */
	if (end - begin > runtime->dma_bytes - bytes_per_frame)
		begin = end - (runtime->dma_bytes - bytes_per_frame);

	/* May need to split to two chunks, calculate the size of each */
	first_chunk_len = end - begin;
	second_chunk_len = 0;
	if (dsp->dma_offset + first_chunk_len > runtime->dma_bytes) {
		/* Receiving buffer wrapped around */
		second_chunk_len = first_chunk_len;
		first_chunk_len = runtime->dma_bytes - dsp->dma_offset;
		second_chunk_len -= first_chunk_len;
	}

	/* Copy first chunk */
	ret = rt5677_spi_read(RT5677_MIC_BUF_ADDR + sizeof(u32) + begin,
			runtime->dma_area + dsp->dma_offset,
			first_chunk_len);
	if (ret)
		return ret;
	dsp->dma_offset += first_chunk_len;
	if (dsp->dma_offset == runtime->dma_bytes)
		dsp->dma_offset = 0;

	/* Copy second chunk */
	if (second_chunk_len) {
		ret = rt5677_spi_read(RT5677_MIC_BUF_ADDR + sizeof(u32) +
				begin + first_chunk_len, runtime->dma_area,
				second_chunk_len);
		if (!ret)
			dsp->dma_offset = second_chunk_len;
	}
	return ret;
}

/*
 * Copy a given amount of audio samples from the DSP mic buffer starting at
 * mic_read_offset, to the dma_area of the pcm runtime. The source buffer may
 * wrap around. mic_read_offset is updated after successful copy.
 * @amount: amount of samples to copy, in bytes.
 *
 * Return: Zero if successful, or a negative error code on failure.
 */
static int rt5677_hotword_copy(struct rt5677_dsp *dsp, u32 amount)
{
	int ret = 0;
	u32 target;

	if (amount == 0)
		return ret;

	target = dsp->mic_read_offset + amount;
	/* Copy the first chunk in DSP's mic buffer */
	ret |= rt5677_hotword_copy_block(dsp, dsp->mic_read_offset,
			min(target, RT5677_MIC_BUF_BYTES));

	if (target >= RT5677_MIC_BUF_BYTES) {
		/* Wrap around, copy the second chunk */
		target -= RT5677_MIC_BUF_BYTES;
		ret |= rt5677_hotword_copy_block(dsp, 0, target);
	}

	if (!ret)
		dsp->mic_read_offset = target;
	return ret;
}

/*
 * Streams audio samples from the DSP mic buffer to the dma_area of the pcm
 * runtime via SPI.
 */
static void rt5677_hotword_stream(struct rt5677_dsp *dsp)
{
	struct snd_pcm_runtime *runtime;
	u32 mic_write_offset;
	size_t new_bytes, copy_bytes, period_bytes, hw_avail_bytes;
	int ret = 0;

	/* Ensure runtime->dma_area buffer does not go away while copying. */
	if (!dsp->substream)
		goto done;

	runtime = dsp->substream->runtime;

	if (rt5677_hotword_mic_write_offset(&mic_write_offset)) {
		dev_err(dsp->spi, "No mic_write_offset\n");
		return;
	}

	/* If this is the first time that we've asked for streaming data after
	 * a hotword is fired, we should start reading from the previous 2
	 * seconds of audio from wherever the mic_write_offset is currently.
	 */
	if (dsp->state == RT5677_HOTWORD_FIRED) {
		dsp->state = RT5677_HOTWORD_STREAM;
		/* See if buffer wraparound happens */
		if (mic_write_offset < RT5677_MIC_BUF_FIRST_READ_SIZE)
			dsp->mic_read_offset = RT5677_MIC_BUF_BYTES -
					(RT5677_MIC_BUF_FIRST_READ_SIZE -
					mic_write_offset);
		else
			dsp->mic_read_offset = mic_write_offset -
					RT5677_MIC_BUF_FIRST_READ_SIZE;
	}

	/* Calculate the amount of new samples in bytes */
	if (dsp->mic_read_offset <= mic_write_offset)
		new_bytes = mic_write_offset - dsp->mic_read_offset;
	else
		new_bytes = RT5677_MIC_BUF_BYTES + mic_write_offset
				- dsp->mic_read_offset;

	/* Copy all new samples from DSP mic buffer, one period at a time */
	period_bytes = snd_pcm_lib_period_bytes(dsp->substream);
	while (new_bytes) {
		/* If the space left in the dma_area buffer is less then one
		 * period, stop copying from DSP and wait for userspace to read,
		 * so that we don't overflow. The DSP has a 4sec buffer which is
		 * usually much larger than the dma_area buffer.
		 */
		hw_avail_bytes = frames_to_bytes(runtime,
				snd_pcm_capture_hw_avail(runtime));
		if (hw_avail_bytes < period_bytes)
			goto done;

		copy_bytes = min(new_bytes, period_bytes
				- dsp->avail_bytes);
		ret = rt5677_hotword_copy(dsp, copy_bytes);
		if (ret) {
			dev_err(dsp->spi, "Copy failed %d\n", ret);
			goto done;
		}
		dsp->avail_bytes += copy_bytes;
		if (dsp->avail_bytes >= period_bytes) {
			snd_pcm_period_elapsed(dsp->substream);
			dsp->avail_bytes = 0;
		}
		new_bytes -= copy_bytes;
	}

done:
	/* TODO benzh: use better delay time based on period_bytes */
	queue_delayed_work(system_freezable_wq, &dsp->work,
			msecs_to_jiffies(RT5677_HOTWORD_STREAM_DELAY_MS));
}

/*
 * A delayed_work that configures the codec audio path for hotwording, loads
 * DSP firmware, streams audio samples after a hotword is detected, and stops
 * the DSP when requested.
 */
static void rt5677_hotword_work(struct work_struct *work)
{
	struct rt5677_dsp *dsp = container_of(work, struct rt5677_dsp,
			work.work);

	mutex_lock(&dsp->lock);

	switch (dsp->state) {
	case RT5677_HOTWORD_IDLE:
		break;
	case RT5677_HOTWORD_START:
		rt5677_hotword_start(dsp);
		break;
	case RT5677_HOTWORD_ARMED:
		break;
	case RT5677_HOTWORD_FIRED:
		dev_info(dsp->spi, "Hotword fired, streaming...\n");
		/* Fall through to start streaming */
	case RT5677_HOTWORD_STREAM:
		rt5677_hotword_stream(dsp);
		break;
	case RT5677_HOTWORD_STOP:
		rt5677_hotword_stop(dsp);
		dev_info(dsp->spi, "Hotword stopped\n");
		break;
	}
	mutex_unlock(&dsp->lock);
}

static irqreturn_t rt5677_hotword_irq(int unused, void *data)
{
	struct rt5677_dsp *dsp = data;
	struct rt5677_priv *rt5677 = dsp->priv;
	unsigned int val;
	int ret, irq_status = IRQ_NONE;
	mutex_lock(&dsp->lock);
	dev_info(dsp->spi, "Hotword irq: %d\n", dsp->state);
	if (dsp->state == RT5677_HOTWORD_ARMED) {
		ret = regmap_read(rt5677->regmap, RT5677_GPIO_CTRL2, &val);
		if (ret) {
			dev_err(dsp->spi, "Failed to read GPIO1 %d", ret);
			goto done;
		}
		if ((val & RT5677_GPIO1_OUT_MASK) != RT5677_GPIO1_OUT_HI) {
			dev_err(dsp->spi, "Not hotword irq %u", val);
			goto done;
		}
		irq_status = IRQ_HANDLED;
		/* DSP fw drove GPIO1 high to signal hotword fired.
		 * ACK the irq by driving GPIO1 low.
		 */
		ret = regmap_update_bits(rt5677->regmap, RT5677_GPIO_CTRL2,
			RT5677_GPIO1_OUT_MASK, RT5677_GPIO1_OUT_LO);
		if (ret) {
			dev_err(dsp->spi, "Failed to set GPIO1 low %d", ret);
			goto done;
		}
		dsp->state = RT5677_HOTWORD_FIRED;

		/* Grab the wake lock to prevent suspend while streaming audio
		 * samples from the DSP mic buffer. The lock is released when
		 * the pcm device is closed.
		 */
		wake_lock(&dsp->wake_lock);
		queue_delayed_work(system_freezable_wq, &dsp->work, 0);
	}
done:
	mutex_unlock(&dsp->lock);
	return irq_status;
}

static int rt5677_hotword_init_irq(struct snd_soc_pcm_runtime *rtd)
{
	struct rt5677_dsp *dsp = snd_soc_platform_get_drvdata(rtd->platform);
	struct rt5677_priv *priv = snd_soc_codec_get_drvdata(rtd->codec);
	struct i2c_client *i2c;
	int ret = 0;

	/* Skip if irq is already requested */
	if (dsp->priv)
		return 0;

	i2c = i2c_verify_client(rtd->codec->dev);
	if (!i2c) {
		ret = -ENODEV;
		goto done;
	}

	/* Drive GPIO1 low so we don't get spurious irqs before hotword fires */
	regmap_update_bits(priv->regmap, RT5677_GPIO_CTRL2,
		RT5677_GPIO1_DIR_MASK | RT5677_GPIO1_OUT_MASK,
		RT5677_GPIO1_DIR_OUT | RT5677_GPIO1_OUT_LO);
	regmap_update_bits(priv->regmap, RT5677_GPIO_CTRL1,
		RT5677_GPIO1_PIN_MASK, RT5677_GPIO1_PIN_GPIO1);

	ret = devm_request_threaded_irq(&i2c->dev, i2c->irq, NULL,
			rt5677_hotword_irq, IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
			"rt5677", dsp);
	if (!ret)
		dsp->priv = priv;
done:
	dev_info(&i2c->dev, "Request irq for hotword detection: %d\n", ret);
	return ret;
}

/* PCM for streaming audio from the DSP buffer */
static int rt5677_hotword_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct rt5677_dsp *dsp = snd_soc_platform_get_drvdata(rtd->platform);
	int ret;

	ret = rt5677_hotword_init_irq(rtd);
	if (ret)
		return ret;

	/* Grab the wake lock to prevent suspend during DSP fw load. The lock
	 * is released after the DSP is armed or if the pcm device is closed
	 * before the DSP is armed.
	 */
	wake_lock(&dsp->wake_lock);

	dsp->state = RT5677_HOTWORD_START;
	queue_delayed_work(system_freezable_wq, &dsp->work,
			msecs_to_jiffies(RT5677_HOTWORD_START_DELAY_MS));

	snd_soc_set_runtime_hwparams(substream, &rt5677_hotword_pcm_hardware);
	return 0;
}

static int rt5677_hotword_pcm_close(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct rt5677_dsp *dsp = snd_soc_platform_get_drvdata(rtd->platform);

	mutex_lock(&dsp->lock);
	if (dsp->state < RT5677_HOTWORD_ARMED) {
		dsp->state = RT5677_HOTWORD_IDLE;
		goto done;
	}
	dsp->state = RT5677_HOTWORD_STOP;
	queue_delayed_work(system_freezable_wq, &dsp->work, 0);
done:
	mutex_unlock(&dsp->lock);
	flush_delayed_work(&dsp->work);
	/* If the pcm device is closed after the DSP is armed but before a
	 * hotword is detected, we unlock twice which is still fine.
	 */
	wake_unlock(&dsp->wake_lock);
	return 0;
}

static int rt5677_hotword_hw_params(struct snd_pcm_substream *substream,
			       struct snd_pcm_hw_params *hw_params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct rt5677_dsp *dsp = snd_soc_platform_get_drvdata(rtd->platform);
	int ret;

	mutex_lock(&dsp->lock);
	ret = snd_pcm_lib_alloc_vmalloc_buffer(substream,
			params_buffer_bytes(hw_params));
	dsp->substream = substream;
	mutex_unlock(&dsp->lock);

	return ret;
}

static int rt5677_hotword_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct rt5677_dsp *dsp = snd_soc_platform_get_drvdata(rtd->platform);

	mutex_lock(&dsp->lock);
	dsp->substream = 0;
	mutex_unlock(&dsp->lock);

	return snd_pcm_lib_free_vmalloc_buffer(substream);
}

static int rt5677_hotword_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct rt5677_dsp *rt5677_dsp =
			snd_soc_platform_get_drvdata(rtd->platform);

	rt5677_dsp->dma_offset = 0;
	rt5677_dsp->avail_bytes = 0;
	return 0;
}

static snd_pcm_uframes_t rt5677_hotword_pcm_pointer(
		struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct rt5677_dsp *rt5677_dsp =
		snd_soc_platform_get_drvdata(rtd->platform);

	return bytes_to_frames(runtime, rt5677_dsp->dma_offset);
}

struct page *rt5677_hotword_pcm_page(struct snd_pcm_substream *substream,
		unsigned long offset)
{
	return snd_pcm_lib_get_vmalloc_page(substream, offset);
}

static struct snd_pcm_ops rt5677_hotword_pcm_ops = {
	.open		= rt5677_hotword_pcm_open,
	.close		= rt5677_hotword_pcm_close,
	.hw_params	= rt5677_hotword_hw_params,
	.hw_free	= rt5677_hotword_hw_free,
	.prepare	= rt5677_hotword_prepare,
	.pointer	= rt5677_hotword_pcm_pointer,
	.mmap		= snd_pcm_lib_mmap_vmalloc,
	.page		= rt5677_hotword_pcm_page,
};

static int rt5677_hotword_pcm_probe(struct snd_soc_platform *platform)
{
	struct rt5677_dsp *dsp;

	dsp = devm_kzalloc(platform->dev, sizeof(*dsp), GFP_KERNEL);
	dsp->spi = platform->dev;
	mutex_init(&dsp->lock);
	INIT_DELAYED_WORK(&dsp->work, rt5677_hotword_work);
	wake_lock_init(&dsp->wake_lock, WAKE_LOCK_SUSPEND, "rt5677_hotword");

	snd_soc_platform_set_drvdata(platform, dsp);
	return 0;
}

static int rt5677_hotword_pcm_remove(struct snd_soc_platform *platform)
{
	struct rt5677_dsp *dsp = snd_soc_platform_get_drvdata(platform);

	wake_lock_destroy(&dsp->wake_lock);
	return 0;
}

static struct snd_soc_platform_driver rt5677_hotword_platform = {
	.probe		= rt5677_hotword_pcm_probe,
	.remove		= rt5677_hotword_pcm_remove,
	.ops		= &rt5677_hotword_pcm_ops,
};

static const struct snd_soc_component_driver rt5677_hotword_dai_component = {
	.name		= "rt5677-hotword",
	.controls	= rt5677_hotword_snd_controls,
	.num_controls	= ARRAY_SIZE(rt5677_hotword_snd_controls),
};

static int rt5677_hotword_dai_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	return 0;
}

static struct snd_soc_dai_ops rt5677_hotword_dai_ops = {
	.hw_params = rt5677_hotword_dai_hw_params,
};

static struct snd_soc_dai_driver rt5677_hotword_dai = {
	/*
	 * The DAI name rt5677-hotword-cpu-dai isn't used. The actual DAI name
	 * registered with ASoC is the name of the device, because we only have
	 * one DAI. See snd_soc_register_dais(), legacy_naming.
	 */
	.name		= "rt5677-hotword-cpu-dai",
	.id		= 0,
	.capture	= {
				.stream_name = "DSP Capture",
				.channels_min = 1,
				.channels_max = 1,
				.rates = SNDRV_PCM_RATE_16000,
				.formats = SNDRV_PCM_FMTBIT_S16_LE,
			},
	.ops		= &rt5677_hotword_dai_ops,
};

int rt5677_hotword_init(struct device *dev)
{
	int ret;

	ret = snd_soc_register_platform(dev, &rt5677_hotword_platform);
	if (ret < 0) {
		dev_err(dev, "Failed to register platform.\n");
		goto err_plat;
	}

	ret = snd_soc_register_component(dev, &rt5677_hotword_dai_component,
					 &rt5677_hotword_dai, 1);
	if (ret < 0) {
		dev_err(dev, "Failed to register component.\n");
		goto err_comp;
	}
	return 0;

err_comp:
	snd_soc_unregister_platform(dev);
err_plat:
	return ret;
}

int rt5677_hotword_free(struct device *dev)
{
	snd_soc_unregister_component(dev);
	snd_soc_unregister_platform(dev);
	return 0;
}
