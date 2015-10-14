/*
 * tegra_asoc_utils_alt.c - MCLK and DAP Utility driver
 *
 * Author: Stephen Warren <swarren@nvidia.com>
 * Copyright (c) 2010-2014 NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>

#include <sound/soc.h>

#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/consumer.h>

#include "tegra_asoc_utils_alt.h"

#ifdef CONFIG_SWITCH
static bool is_switch_registered;
#endif

int tegra_alt_asoc_utils_set_rate(struct tegra_asoc_audio_clock_info *data,
				int srate,
				int mclk,
				int clk_out_rate)
{
	int new_baseclock;
	bool clk_change;
	int err;

	switch (srate) {
	case 11025:
	case 22050:
	case 44100:
	case 88200:
		if (data->soc == TEGRA_ASOC_UTILS_SOC_TEGRA20)
			new_baseclock = 56448000;
		else if (data->soc == TEGRA_ASOC_UTILS_SOC_TEGRA30)
			new_baseclock = 564480000;
		else
			new_baseclock = 282240000;
		break;
	case 8000:
	case 16000:
	case 32000:
	case 48000:
	case 64000:
	case 96000:
	case 192000:
		if (data->soc == TEGRA_ASOC_UTILS_SOC_TEGRA20)
			new_baseclock = 73728000;
		else if (data->soc == TEGRA_ASOC_UTILS_SOC_TEGRA30)
			new_baseclock = 552960000;
		else
			new_baseclock = 368640000;
		break;
	default:
		return -EINVAL;
	}

	clk_change = ((new_baseclock != data->set_baseclock) ||
			(mclk != data->set_mclk));
	if (!clk_change)
		return 0;

	/* Don't change rate if already one dai-link is using it */
	if (data->lock_count)
		return -EINVAL;

	data->set_baseclock = 0;
	data->set_mclk = 0;

	err = clk_set_rate(data->clk_pll_a, new_baseclock);
	if (err) {
		dev_err(data->dev, "Can't set pll_a rate: %d\n", err);
		return err;
	}

	err = clk_set_rate(data->clk_pll_a_out0, mclk);
	if (err) {
		dev_err(data->dev, "Can't set clk_pll_a_out0 rate: %d\n", err);
		return err;
	}

	err = clk_set_rate(data->clk_cdev1, clk_out_rate);
	if (err) {
		dev_err(data->dev, "Can't set clk_cdev1 rate: %d\n", err);
		return err;
	}
	err = clk_set_rate(data->clk_cdev2, clk_out_rate);
	if (err) {
		dev_err(data->dev, "Can't set clk_cdev2 rate: %d\n", err);
		return err;
	}

	data->set_baseclock = new_baseclock;
	data->set_mclk = mclk;

	return 0;
}
EXPORT_SYMBOL_GPL(tegra_alt_asoc_utils_set_rate);

void tegra_alt_asoc_utils_lock_clk_rate(struct tegra_asoc_audio_clock_info *data,
				    int lock)
{
	if (lock)
		data->lock_count++;
	else if (data->lock_count)
		data->lock_count--;
}
EXPORT_SYMBOL_GPL(tegra_alt_asoc_utils_lock_clk_rate);

int tegra_alt_asoc_utils_clk_enable(struct tegra_asoc_audio_clock_info *data)
{
	int err;

	err = clk_prepare_enable(data->clk_cdev1);
	if (err) {
		dev_err(data->dev, "Can't enable cdev1: %d\n", err);
		return err;
	}
	data->clk_cdev1_state = 1;

	err = clk_prepare_enable(data->clk_cdev2);
	if (err) {
		dev_err(data->dev, "Can't enable cdev2: %d\n", err);
		return err;
	}
	data->clk_cdev2_state = 1;

	return 0;
}
EXPORT_SYMBOL_GPL(tegra_alt_asoc_utils_clk_enable);

int tegra_alt_asoc_utils_clk_disable(struct tegra_asoc_audio_clock_info *data)
{
	clk_disable_unprepare(data->clk_cdev1);
	data->clk_cdev1_state = 0;

	clk_disable_unprepare(data->clk_cdev2);
	data->clk_cdev2_state = 0;

	return 0;
}
EXPORT_SYMBOL_GPL(tegra_alt_asoc_utils_clk_disable);

int tegra_alt_asoc_utils_init(struct tegra_asoc_audio_clock_info *data,
			  struct device *dev, struct snd_soc_card *card)
{
	int ret;

	data->dev = dev;
	data->card = card;

	if (of_machine_is_compatible("nvidia,tegra20"))
		data->soc = TEGRA_ASOC_UTILS_SOC_TEGRA20;
	else if (of_machine_is_compatible("nvidia,tegra30"))
		data->soc = TEGRA_ASOC_UTILS_SOC_TEGRA30;
	else if (of_machine_is_compatible("nvidia,tegra114"))
		data->soc = TEGRA_ASOC_UTILS_SOC_TEGRA114;
	else if (of_machine_is_compatible("nvidia,tegra148"))
		data->soc = TEGRA_ASOC_UTILS_SOC_TEGRA148;
	else if (of_machine_is_compatible("nvidia,tegra124"))
		data->soc = TEGRA_ASOC_UTILS_SOC_TEGRA124;
	else if (of_machine_is_compatible("nvidia,tegra210"))
		data->soc = TEGRA_ASOC_UTILS_SOC_TEGRA210;
	else if (!dev->of_node) {
		/* non-DT is always Tegra20 */
#if defined(CONFIG_ARCH_TEGRA_2x_SOC)
		data->soc = TEGRA_ASOC_UTILS_SOC_TEGRA20;
#elif defined(CONFIG_ARCH_TEGRA_3x_SOC)
		data->soc = TEGRA_ASOC_UTILS_SOC_TEGRA30;
#elif defined(CONFIG_ARCH_TEGRA_11x_SOC)
		data->soc = TEGRA_ASOC_UTILS_SOC_TEGRA114;
#elif defined(CONFIG_ARCH_TEGRA_14x_SOC)
		data->soc = TEGRA_ASOC_UTILS_SOC_TEGRA148;
#elif defined(CONFIG_ARCH_TEGRA_12x_SOC)
		data->soc = TEGRA_ASOC_UTILS_SOC_TEGRA124;
#endif
	} else
		/* DT boot, but unknown SoC */
		return -EINVAL;

	if (data->soc < TEGRA_ASOC_UTILS_SOC_TEGRA210) {
		data->clk_pll_p_out1 = devm_clk_get(dev, "pll_p_out1");
		if (IS_ERR(data->clk_pll_p_out1)) {
			dev_err(data->dev, "Can't retrieve clk pll_p_out1\n");
			ret = PTR_ERR(data->clk_pll_p_out1);
			goto err;
		}
	}

	data->clk_pll_a = devm_clk_get(dev, "pll_a");
	if (IS_ERR(data->clk_pll_a)) {
		dev_err(data->dev, "Can't retrieve clk pll_a\n");
		ret = PTR_ERR(data->clk_pll_a);
		goto err_put_pll_p_out1;
	}

	data->clk_pll_a_out0 = devm_clk_get(dev, "pll_a_out0");
	if (IS_ERR(data->clk_pll_a_out0)) {
		dev_err(data->dev, "Can't retrieve clk pll_a_out0\n");
		ret = PTR_ERR(data->clk_pll_a_out0);
		goto err_put_pll_a;
	}

	data->clk_m = devm_clk_get(dev, "clk_m");
	if (IS_ERR(data->clk_m)) {
		dev_err(data->dev, "Can't retrieve clk clk_m\n");
		ret = PTR_ERR(data->clk_m);
		goto err;
	}

	if (data->soc == TEGRA_ASOC_UTILS_SOC_TEGRA20)
		data->clk_cdev1 = devm_clk_get(dev, "cdev1");
	else
		data->clk_cdev1 = devm_clk_get(dev, "extern1");

	if (IS_ERR(data->clk_cdev1)) {
		dev_err(data->dev, "Can't retrieve clk cdev1\n");
		ret = PTR_ERR(data->clk_cdev1);
		goto err_put_pll_a_out0;
	}

	if (data->soc == TEGRA_ASOC_UTILS_SOC_TEGRA20)
		data->clk_out1 = ERR_PTR(-ENOENT);
	else {
		data->clk_out1 = devm_clk_get(dev, "clk_out_1");
		if (IS_ERR(data->clk_out1)) {
			dev_err(data->dev, "Can't retrieve clk out1\n");
			ret = PTR_ERR(data->clk_out1);
			goto err_put_cdev1;
		}
	}

	ret = clk_prepare_enable(data->clk_cdev1);
	if (ret) {
		dev_err(data->dev, "Can't enable clk cdev1/extern1");
		goto err_put_out1;
	}
	data->clk_cdev1_state = 1;

	if (!IS_ERR(data->clk_out1)) {
		ret = clk_prepare_enable(data->clk_out1);
		if (ret) {
			dev_err(data->dev, "Can't enable clk out1");
			goto err_put_out1;
		}
	}

	if (data->soc == TEGRA_ASOC_UTILS_SOC_TEGRA20)
		data->clk_cdev2 = devm_clk_get(dev, "cdev2");
	else
		data->clk_cdev2 = devm_clk_get(dev, "extern2");

	if (IS_ERR(data->clk_cdev2)) {
		dev_err(data->dev, "Can't retrieve clk cdev2\n");
		ret = PTR_ERR(data->clk_cdev2);
		goto err_put_out1;
	}

	if (data->soc == TEGRA_ASOC_UTILS_SOC_TEGRA20)
		data->clk_out2 = ERR_PTR(-ENOENT);
	else {
		data->clk_out2 = devm_clk_get(dev, "clk_out_2");
		if (IS_ERR(data->clk_out2)) {
			dev_err(data->dev, "Can't retrieve clk out2\n");
			ret = PTR_ERR(data->clk_out2);
			goto err_put_cdev2;
		}
	}

	ret = clk_prepare_enable(data->clk_cdev2);
	if (ret) {
		dev_err(data->dev, "Can't enable clk cdev2/extern2");
		goto err_put_out2;
	}
	data->clk_cdev2_state = 1;

	if (!IS_ERR(data->clk_out2)) {
		ret = clk_prepare_enable(data->clk_out2);
		if (ret) {
			dev_err(data->dev, "Can't enable clk out2");
			goto err_put_out2;
		}
	}

	if (data->soc < TEGRA_ASOC_UTILS_SOC_TEGRA210) {
		ret = tegra_alt_asoc_utils_set_rate(data, 48000,
					256 * 48000, 256 * 48000);
		if (ret)
			goto err_put_out2;
	}

	return 0;

err_put_out2:
	if (!IS_ERR(data->clk_out2))
		clk_put(data->clk_out2);
err_put_cdev2:
	clk_put(data->clk_cdev2);
err_put_out1:
	if (!IS_ERR(data->clk_out1))
		clk_put(data->clk_out1);
err_put_cdev1:
	clk_put(data->clk_cdev1);
err_put_pll_a_out0:
	clk_put(data->clk_pll_a_out0);
err_put_pll_a:
	clk_put(data->clk_pll_a);
err_put_pll_p_out1:
	clk_put(data->clk_pll_p_out1);
err:
	return ret;
}
EXPORT_SYMBOL_GPL(tegra_alt_asoc_utils_init);

int tegra_alt_asoc_utils_set_parent(struct tegra_asoc_audio_clock_info *data,
			int is_i2s_master)
{
	int ret = -ENODEV;

	if (data->soc == TEGRA_ASOC_UTILS_SOC_TEGRA20)
		return ret;

	if (is_i2s_master) {
		ret = clk_set_parent(data->clk_cdev1, data->clk_pll_a_out0);
		if (ret) {
			dev_err(data->dev, "Can't set clk cdev1/extern1 parent");
			return ret;
		}

		ret = clk_set_parent(data->clk_cdev2, data->clk_pll_a_out0);
		if (ret) {
			dev_err(data->dev, "Can't set clk cdev2/extern2 parent");
			return ret;
		}
	} else {
		ret = clk_set_parent(data->clk_cdev1, data->clk_m);
		if (ret) {
			dev_err(data->dev, "Can't set clk cdev1/extern1 parent");
			return ret;
		}

		ret = clk_set_rate(data->clk_cdev1, 13000000);
		if (ret) {
			dev_err(data->dev, "Can't set clk rate");
			return ret;
		}

		ret = clk_set_parent(data->clk_cdev2, data->clk_m);
		if (ret) {
			dev_err(data->dev, "Can't set clk cdev2/extern2 parent");
			return ret;
		}

		ret = clk_set_rate(data->clk_cdev2, 13000000);
		if (ret) {
			dev_err(data->dev, "Can't set clk rate");
			return ret;
		}
	}

	return 0;
}
EXPORT_SYMBOL_GPL(tegra_alt_asoc_utils_set_parent);

int tegra_alt_asoc_utils_set_extern_parent(
	struct tegra_asoc_audio_clock_info *data, const char *parent)
{
	unsigned long rate;
	int err;

	if (data->soc == TEGRA_ASOC_UTILS_SOC_TEGRA20)
		return -ENODEV;

	rate = clk_get_rate(data->clk_cdev1);
	if (!strcmp(parent, "clk_m")) {
		err = clk_set_parent(data->clk_cdev1, data->clk_m);
		if (err) {
			dev_err(data->dev, "Can't set clk extern1 parent");
			return err;
		}
		err = clk_set_parent(data->clk_cdev2, data->clk_m);
		if (err) {
			dev_err(data->dev, "Can't set clk extern2 parent");
			return err;
		}
	} else if (!strcmp(parent, "pll_a_out0")) {
		err = clk_set_parent(data->clk_cdev1, data->clk_pll_a_out0);
		if (err) {
			dev_err(data->dev, "Can't set clk cdev1/extern1 parent");
			return err;
		}
		err = clk_set_parent(data->clk_cdev2, data->clk_pll_a_out0);
		if (err) {
			dev_err(data->dev, "Can't set clk cdev2/extern2 parent");
			return err;
		}
	}

	err = clk_set_rate(data->clk_cdev1, rate);
	if (err) {
		dev_err(data->dev, "Can't set clk rate");
		return err;
	}
	err = clk_set_rate(data->clk_cdev2, rate);
	if (err) {
		dev_err(data->dev, "Can't set clk rate");
		return err;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(tegra_alt_asoc_utils_set_extern_parent);

void tegra_alt_asoc_utils_fini(struct tegra_asoc_audio_clock_info *data)
{
	if (data->clk_cdev2_state)
		clk_disable(data->clk_cdev2);

	if (!IS_ERR(data->clk_out2))
		clk_put(data->clk_out2);

	if (data->clk_cdev1_state)
		clk_disable(data->clk_cdev1);

	if (!IS_ERR(data->clk_out1))
		clk_put(data->clk_out1);

	if (!IS_ERR(data->clk_pll_a_out0))
		clk_put(data->clk_pll_a_out0);

	if (!IS_ERR(data->clk_pll_a))
		clk_put(data->clk_pll_a);

	if (!IS_ERR(data->clk_pll_p_out1))
		clk_put(data->clk_pll_p_out1);
}
EXPORT_SYMBOL_GPL(tegra_alt_asoc_utils_fini);

#ifdef CONFIG_SWITCH
int tegra_alt_asoc_switch_register(struct switch_dev *sdev)
{
	int ret;

	if (is_switch_registered)
		return -EBUSY;

	ret = switch_dev_register(sdev);

	if (ret >= 0)
		is_switch_registered = true;

	return ret;
}
EXPORT_SYMBOL_GPL(tegra_alt_asoc_switch_register);

void tegra_alt_asoc_switch_unregister(struct switch_dev *sdev)
{
	if (!is_switch_registered)
		return;

	switch_dev_unregister(sdev);
	is_switch_registered = false;
}
EXPORT_SYMBOL_GPL(tegra_alt_asoc_switch_unregister);
#endif

MODULE_AUTHOR("Stephen Warren <swarren@nvidia.com>");
MODULE_DESCRIPTION("Tegra ASoC utility code");
MODULE_LICENSE("GPL");
