/*
 * XXX
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

#ifndef __DRIVERS_THERMAL_TEGRA_SOCTHERM_H
#define __DRIVERS_THERMAL_TEGRA_SOCTHERM_H

#include <linux/module.h>

#define SENSOR_CONFIG0				0
#define		SENSOR_CONFIG0_STOP		BIT(0)
#define		SENSOR_CONFIG0_TALL_SHIFT	8
#define		SENSOR_CONFIG0_TCALC_OVER	BIT(4)
#define		SENSOR_CONFIG0_OVER		BIT(3)
#define		SENSOR_CONFIG0_CPTR_OVER	BIT(2)
#define SENSOR_CONFIG1				4
#define		SENSOR_CONFIG1_TSAMPLE_SHIFT	0
#define		SENSOR_CONFIG1_TIDDQ_EN_SHIFT	15
#define		SENSOR_CONFIG1_TEN_COUNT_SHIFT	24
#define		SENSOR_CONFIG1_TEMP_ENABLE	BIT(31)
#define SENSOR_CONFIG2				8
#define		SENSOR_CONFIG2_THERMA_MASK	(0xffff << 16)
#define		SENSOR_CONFIG2_THERMB_MASK	0xffff

#define THERMCTL_LEVEL0_GROUP_CPU		0x0
/* DN_THRESH and UP_THRESH are defined in device file */
#define		THERMCTL_LVL0_CPU0_EN_SHIFT		8
#define		THERMCTL_LVL0_CPU0_EN_MASK		0x1
#define		THERMCTL_LVL0_CPU0_CPU_THROT_SHIFT	5
#define		THERMCTL_LVL0_CPU0_CPU_THROT_MASK	0x3
#define		THERMCTL_LVL0_CPU0_CPU_THROT_LIGHT	0x1
#define		THERMCTL_LVL0_CPU0_CPU_THROT_HEAVY	0x2
#define		THERMCTL_LVL0_CPU0_GPU_THROT_SHIFT	3
#define		THERMCTL_LVL0_CPU0_GPU_THROT_MASK	0x3
#define		THERMCTL_LVL0_CPU0_GPU_THROT_LIGHT	0x1
#define		THERMCTL_LVL0_CPU0_GPU_THROT_HEAVY	0x2
#define		THERMCTL_LVL0_CPU0_MEM_THROT_SHIFT	2
#define		THERMCTL_LVL0_CPU0_MEM_THROT_MASK	0x1
#define		THERMCTL_LVL0_CPU0_STATUS_SHIFT		0
#define		THERMCTL_LVL0_CPU0_STATUS_MASK		0x3

#define THERMCTL_LEVEL0_GROUP_GPU		0x4
#define THERMCTL_LEVEL0_GROUP_MEM		0x8
#define THERMCTL_LEVEL0_GROUP_TSENSE		0xc

#define THERMTRIP				0x80
/* BITs are defined in device file */

#define SENSOR_PDIV				0x1c0
#define		SENSOR_PDIV_CPU_MASK		(0xf << 12)
#define		SENSOR_PDIV_GPU_MASK		(0xf << 8)
#define		SENSOR_PDIV_MEM_MASK		(0xf << 4)
#define		SENSOR_PDIV_PLLX_MASK		(0xf << 0)

#define SENSOR_HOTSPOT_OFF			0x1c4
#define		SENSOR_HOTSPOT_CPU_MASK		(0xff << 16)
#define		SENSOR_HOTSPOT_GPU_MASK		(0xff << 8)
#define		SENSOR_HOTSPOT_MEM_MASK		(0xff << 0)

#define SENSOR_TEMP1				0x1c8
#define		SENSOR_TEMP1_CPU_TEMP_MASK	(0xffff << 16)
#define		SENSOR_TEMP1_GPU_TEMP_MASK	0xffff

#define SENSOR_TEMP2				0x1cc
#define		SENSOR_TEMP2_MEM_TEMP_MASK	(0xffff << 16)
#define		SENSOR_TEMP2_PLLX_TEMP_MASK	0xffff

/*
 * struct tegra_tsensor_group.flags meanings
 */
#define SKIP_THERMAL_FW_REGISTRATION		BIT(0)
#define SKIP_THERMTRIP_REGISTRATION		BIT(1)

enum soctherm_chipid {
	CHIPID_TEGRA12X = 0,
	CHIPID_TEGRA13X,
	CHIPID_TEGRA21X,
	CHIPD_SIZE,
};


/**
 * struct tegra_tsensor_group - SOC_THERM sensor group data
 * @name: short name of the temperature sensor group
 * @id: numeric ID of the temperature sensor group
 * @thermctl_isr_shift: bit shift for interrupt status/enable register
 * @thermctl_lvl0_offset: offset of the THERMCTL_LEVEL0_GROUP_* reg
 * @thermtrip_enable_mask: register mask to enable the THERMTRIP feature
 * @thermtrip_any_en_mask: register mask to enable the any THERMTRIPs
 * @thermtrip_threshold_mask: register mask to program the THERMTRIP threshold
 * @thermctl_lvl0_up_thresh_mask: register mask to program UP threshold
 * @thermctl_lvl0_dn_thresh_mask: register mask to program DN threshold
 * @sensor_temp_offset: offset of the SENSOR_TEMP* register
 * @sensor_temp_mask: bit mask for this sensor group in SENSOR_TEMP* register
 * @pllx_hotspot_diff: hotspot offset from the PLLX sensor
 * @pllx_hotspot_mask: register bitfield mask for the HOTSPOT field
 * @pdiv: the sensor count post-divider to use during runtime
 * @pdiv_ate: the sensor count post-divider used during automated test
 * @pdiv_mask: register bitfield mask for the PDIV field for this sensor
 * @flags: any per-tsensor-group flags
 *
 * @pllx_hotspot_diff must be 0 for the PLLX sensor group.
 * The GPU and MEM sensor groups share a thermtrip temperature threshold.
 */
struct tegra_tsensor_group {
	const char	*name;
	u8		id;
	u8		thermctl_isr_shift;
	u8		pdiv;
	u8		pdiv_ate;
	u8		flags;
	u16		thermctl_lvl0_offset;
	u16		sensor_temp_offset;
	u32		sensor_temp_mask;
	u32		thermtrip_enable_mask;
	u32		thermtrip_any_en_mask;
	u32		thermtrip_threshold_mask;
	u32		thermctl_lvl0_up_thresh_mask;
	u32		thermctl_lvl0_dn_thresh_mask;
	u32		pdiv_mask;
	u32		pllx_hotspot_mask;
	int		pllx_hotspot_diff;
	u32		bptt;
};

struct tegra_tsensor_configuration {
	u32 tall, tsample, tiddq_en, ten_count, tsample_ate;
};

struct tegra_tsensor {
	const char *name;
	u32 base;
	const struct tegra_tsensor_configuration *config;
	u32 calib_fuse_offset;
	s32 fuse_corr_alpha, fuse_corr_beta;
	u32 calib;
	const struct tegra_tsensor_group *group;
};

struct tsensor_shared_calibration {
	u32 base_cp, base_ft;
	s32 actual_temp_cp, actual_temp_ft;
};

int tegra_soctherm_calculate_shared_calibration(
				struct tsensor_shared_calibration *r,
				enum soctherm_chipid chipid);
int tegra_soctherm_calculate_tsensor_calibration(
				struct tegra_tsensor *sensor,
				struct tsensor_shared_calibration *shared);
int tegra_soctherm_probe(
		struct platform_device *pdev,
		struct tegra_tsensor *tsensors,
		const struct tegra_tsensor_group **tegra_tsensor_groups,
		enum soctherm_chipid chipid);
int tegra_soctherm_remove(struct platform_device *pdev);

#ifdef CONFIG_PM_SLEEP
int soctherm_suspend(struct device *dev);
int soctherm_resume(struct device *dev);
#endif

#endif
