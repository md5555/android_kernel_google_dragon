/*
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author:
 *	Mikko Perttunen <mperttunen@nvidia.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/string.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/thermal.h>
#include <soc/tegra/fuse.h>
#include <soc/tegra/pmc.h>
#include <linux/delay.h>
#include <linux/interrupt.h>

#include <dt-bindings/thermal/tegra124-soctherm.h>

#include "tegra_soctherm.h"

/* bits_per_temp_threshold */
#define TEGRA124_BPTT				8

#define TEGRA124_THERMCTL_LVL0_UP_THRESH_MASK	(0xff << 17)
#define TEGRA124_THERMCTL_LVL0_DN_THRESH_MASK	(0xff << 9)

#define TEGRA124_THERMTRIP_ANY_EN_MASK		(0x1 << 28)
#define TEGRA124_THERMTRIP_MEM_EN_MASK		(0x1 << 27)
#define TEGRA124_THERMTRIP_GPU_EN_MASK		(0x1 << 26)
#define TEGRA124_THERMTRIP_CPU_EN_MASK		(0x1 << 25)
#define TEGRA124_THERMTRIP_TSENSE_EN_MASK	(0x1 << 24)
#define TEGRA124_THERMTRIP_GPUMEM_THRESH_MASK	(0xff << 16)
#define TEGRA124_THERMTRIP_CPU_THRESH_MASK	(0xff << 8)
#define TEGRA124_THERMTRIP_TSENSE_THRESH_MASK	0xff

static const struct tegra_tsensor_configuration tegra124_tsensor_config = {
	.tall = 16300,
	.tsample = 120,
	.tiddq_en = 1,
	.ten_count = 1,
	.tsample_ate = 481,
};

static const struct tegra_tsensor_group tegra124_tsensor_group_cpu = {
	.id				= TEGRA124_SOCTHERM_SENSOR_CPU,
	.name				= "cpu",
	.thermctl_isr_shift		= 8,
	.thermctl_lvl0_offset		= THERMCTL_LEVEL0_GROUP_CPU,
	.thermctl_lvl0_up_thresh_mask	= TEGRA124_THERMCTL_LVL0_UP_THRESH_MASK,
	.thermctl_lvl0_dn_thresh_mask	= TEGRA124_THERMCTL_LVL0_DN_THRESH_MASK,
	.sensor_temp_offset		= SENSOR_TEMP1,
	.sensor_temp_mask		= SENSOR_TEMP1_CPU_TEMP_MASK,
	.pdiv				= 8,
	.pdiv_ate			= 8,
	.pdiv_mask			= SENSOR_PDIV_CPU_MASK,
	.pllx_hotspot_mask		= SENSOR_HOTSPOT_CPU_MASK,
	.pllx_hotspot_diff		= 10000,
	.thermtrip_any_en_mask		= TEGRA124_THERMTRIP_ANY_EN_MASK,
	.thermtrip_enable_mask		= TEGRA124_THERMTRIP_CPU_EN_MASK,
	.thermtrip_threshold_mask	= TEGRA124_THERMTRIP_CPU_THRESH_MASK,
	.bptt				= TEGRA124_BPTT,
};

static const struct tegra_tsensor_group tegra124_tsensor_group_gpu = {
	.id				= TEGRA124_SOCTHERM_SENSOR_GPU,
	.name				= "gpu",
	.thermctl_isr_shift		= 16,
	.thermctl_lvl0_offset		= THERMCTL_LEVEL0_GROUP_GPU,
	.thermctl_lvl0_up_thresh_mask	= TEGRA124_THERMCTL_LVL0_UP_THRESH_MASK,
	.thermctl_lvl0_dn_thresh_mask	= TEGRA124_THERMCTL_LVL0_DN_THRESH_MASK,
	.sensor_temp_offset		= SENSOR_TEMP1,
	.sensor_temp_mask		= SENSOR_TEMP1_GPU_TEMP_MASK,
	.pdiv				= 8,
	.pdiv_ate			= 8,
	.pdiv_mask			= SENSOR_PDIV_GPU_MASK,
	.pllx_hotspot_mask		= SENSOR_HOTSPOT_GPU_MASK,
	.pllx_hotspot_diff		= 5000,
	.thermtrip_any_en_mask		= TEGRA124_THERMTRIP_ANY_EN_MASK,
	.thermtrip_enable_mask		= TEGRA124_THERMTRIP_GPU_EN_MASK,
	.thermtrip_threshold_mask	= TEGRA124_THERMTRIP_GPUMEM_THRESH_MASK,
	.bptt				= TEGRA124_BPTT,
};

static const struct tegra_tsensor_group tegra124_tsensor_group_pll = {
	.id				= TEGRA124_SOCTHERM_SENSOR_PLLX,
	.name				= "pll",
	.thermctl_isr_shift		= 0,
	.thermctl_lvl0_offset		= THERMCTL_LEVEL0_GROUP_TSENSE,
	.thermctl_lvl0_up_thresh_mask	= TEGRA124_THERMCTL_LVL0_UP_THRESH_MASK,
	.thermctl_lvl0_dn_thresh_mask	= TEGRA124_THERMCTL_LVL0_DN_THRESH_MASK,
	.sensor_temp_offset		= SENSOR_TEMP2,
	.sensor_temp_mask		= SENSOR_TEMP2_PLLX_TEMP_MASK,
	.pdiv				= 8,
	.pdiv_ate			= 8,
	.pdiv_mask			= SENSOR_PDIV_PLLX_MASK,
	.thermtrip_any_en_mask		= TEGRA124_THERMTRIP_ANY_EN_MASK,
	.thermtrip_enable_mask		= TEGRA124_THERMTRIP_TSENSE_EN_MASK,
	.thermtrip_threshold_mask	= TEGRA124_THERMTRIP_TSENSE_THRESH_MASK,
	.bptt				= TEGRA124_BPTT,
};

static const struct tegra_tsensor_group tegra124_tsensor_group_mem = {
	.id				= TEGRA124_SOCTHERM_SENSOR_MEM,
	.name				= "mem",
	.thermctl_isr_shift		= 24,
	.thermctl_lvl0_offset		= THERMCTL_LEVEL0_GROUP_MEM,
	.thermctl_lvl0_up_thresh_mask	= TEGRA124_THERMCTL_LVL0_UP_THRESH_MASK,
	.thermctl_lvl0_dn_thresh_mask	= TEGRA124_THERMCTL_LVL0_DN_THRESH_MASK,
	.sensor_temp_offset		= SENSOR_TEMP2,
	.sensor_temp_mask		= SENSOR_TEMP2_MEM_TEMP_MASK,
	.pdiv				= 8,
	.pdiv_ate			= 8,
	.pdiv_mask			= SENSOR_PDIV_MEM_MASK,
	.pllx_hotspot_mask		= SENSOR_HOTSPOT_MEM_MASK,
	.pllx_hotspot_diff		= 0,
	.thermtrip_any_en_mask		= TEGRA124_THERMTRIP_ANY_EN_MASK,
	.thermtrip_enable_mask		= TEGRA124_THERMTRIP_MEM_EN_MASK,
	.thermtrip_threshold_mask	= TEGRA124_THERMTRIP_GPUMEM_THRESH_MASK,
	.bptt				= TEGRA124_BPTT,
};

static const struct tegra_tsensor_group *tegra124_tsensor_groups[] = {
	&tegra124_tsensor_group_cpu,
	&tegra124_tsensor_group_gpu,
	&tegra124_tsensor_group_pll,
	&tegra124_tsensor_group_mem,
	NULL,
};

static struct tegra_tsensor tegra124_tsensors[] = {
	{
		.base = 0xc0,
		.name = "cpu0",
		.config = &tegra124_tsensor_config,
		.calib_fuse_offset = 0x098,
		.fuse_corr_alpha = 1135400,
		.fuse_corr_beta = -6266900,
		.group = &tegra124_tsensor_group_cpu,
	},
	{
		.base = 0xe0,
		.name = "cpu1",
		.config = &tegra124_tsensor_config,
		.calib_fuse_offset = 0x084,
		.fuse_corr_alpha = 1122220,
		.fuse_corr_beta = -5700700,
		.group = &tegra124_tsensor_group_cpu,
	},
	{
		.base = 0x100,
		.name = "cpu2",
		.config = &tegra124_tsensor_config,
		.calib_fuse_offset = 0x088,
		.fuse_corr_alpha = 1127000,
		.fuse_corr_beta = -6768200,
		.group = &tegra124_tsensor_group_cpu,
	},
	{
		.base = 0x120,
		.name = "cpu3",
		.config = &tegra124_tsensor_config,
		.calib_fuse_offset = 0x12c,
		.fuse_corr_alpha = 1110900,
		.fuse_corr_beta = -6232000,
		.group = &tegra124_tsensor_group_cpu,
	},
	{
		.base = 0x140,
		.name = "mem0",
		.config = &tegra124_tsensor_config,
		.calib_fuse_offset = 0x158,
		.fuse_corr_alpha = 1122300,
		.fuse_corr_beta = -5936400,
		.group = &tegra124_tsensor_group_mem,
	},
	{
		.base = 0x160,
		.name = "mem1",
		.config = &tegra124_tsensor_config,
		.calib_fuse_offset = 0x15c,
		.fuse_corr_alpha = 1145700,
		.fuse_corr_beta = -7124600,
		.group = &tegra124_tsensor_group_mem,
	},
	{
		.base = 0x180,
		.name = "gpu",
		.config = &tegra124_tsensor_config,
		.calib_fuse_offset = 0x154,
		.fuse_corr_alpha = 1120100,
		.fuse_corr_beta = -6000500,
		.group = &tegra124_tsensor_group_gpu,
	},
	{
		.base = 0x1a0,
		.name = "pllx",
		.config = &tegra124_tsensor_config,
		.calib_fuse_offset = 0x160,
		.fuse_corr_alpha = 1106500,
		.fuse_corr_beta = -6729300,
		.group = &tegra124_tsensor_group_pll,
	},
	{ .name = NULL },
};

/*
 *
 */

static struct of_device_id tegra124_soctherm_of_match[] = {
	{ .compatible = "nvidia,tegra124-soctherm" },
	{ },
};
MODULE_DEVICE_TABLE(of, tegra_soctherm_of_match);

static int tegra124_soctherm_probe(struct platform_device *pdev)
{
	return tegra_soctherm_probe(pdev,
				    tegra124_tsensors,
				    tegra124_tsensor_groups,
				    CHIPID_TEGRA12X);
}

static struct platform_driver tegra124_soctherm_driver = {
	.probe = tegra124_soctherm_probe,
	.remove = tegra_soctherm_remove,
	.driver = {
		.name = "tegra124_soctherm",
		.of_match_table = tegra124_soctherm_of_match,
	},
};
module_platform_driver(tegra124_soctherm_driver);

MODULE_AUTHOR("NVIDIA");
MODULE_DESCRIPTION("Tegra124 SOCTHERM thermal management driver");
MODULE_LICENSE("GPL v2");
