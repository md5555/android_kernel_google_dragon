/*
 * Copyright (C) 2013-2015 NVIDIA Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/atomic.h>
#include <linux/debugfs.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/thermal.h>
#include <linux/timer.h>

#include <soc/tegra/tegra_emc.h>

static int (*tegra_emc_set_over_temp_state)(unsigned long);
static unsigned long thermal_state;

static int dram_cdev_max_state(struct thermal_cooling_device *tcd,
				   unsigned long *state)
{
	*state = TEGRA_DRAM_OVER_TEMP_MAX;
	return 0;
}

static int dram_cdev_cur_state(struct thermal_cooling_device *tcd,
				   unsigned long *state)
{
	*state = thermal_state;
	return 0;
}

static int dram_cdev_set_state(struct thermal_cooling_device *tcd,
				   unsigned long state)
{
	if (state > TEGRA_DRAM_OVER_TEMP_MAX)
		return -1;

	tegra_emc_set_over_temp_state(state);
	return 0;
}

static const struct thermal_cooling_device_ops dram_cdev_ops = {
	.get_max_state = dram_cdev_max_state,
	.get_cur_state = dram_cdev_cur_state,
	.set_cur_state = dram_cdev_set_state,
};

static struct of_device_id emc_match[] = {
	{
		.compatible = "nvidia,tegra210-emc",
		.data = &tegra210_emc_set_over_temp_state,
	},
	{ },
};

static int tegra_dram_therm_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct device_node *emc_np;
	void *ret;

	emc_np = of_find_matching_node_and_match(NULL, emc_match, &match);
	if (!emc_np) {
		pr_err("EMC device node not found\n");
		return -ENODEV;
	}

	tegra_emc_set_over_temp_state = match->data;
	ret = thermal_of_cooling_device_register(pdev->dev.of_node,
						 "dram_derate",
						 NULL,
						 &dram_cdev_ops);
	if (IS_ERR(ret))
		return PTR_ERR(ret);

	return 0;
}

static const struct of_device_id tegra_dram_therm_of_match[] = {
	{ .compatible = "nvidia,tegra-dram-therm", },
	{},
};

static struct platform_driver tegra_dram_therm_driver = {
	.driver         = {
		.name   = "tegra-dram-therm",
		.of_match_table = tegra_dram_therm_of_match,
	},
	.probe          = tegra_dram_therm_probe,
};
module_platform_driver(tegra_dram_therm_driver);
