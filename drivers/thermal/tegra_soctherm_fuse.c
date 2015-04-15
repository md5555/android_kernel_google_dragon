/*
 * drivers/thermal/tegra_soctherm.c
 *
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

#define NOMINAL_CALIB_FT			105
#define NOMINAL_CALIB_CP			25

#define FUSE_TSENSOR_CALIB_CP_TS_BASE_MASK	0x1fff
#define FUSE_TSENSOR_CALIB_FT_TS_BASE_MASK	(0x1fff << 13)
#define FUSE_TSENSOR_CALIB_FT_TS_BASE_SHIFT	13

#define FUSE_TSENSOR_COMMON			0x180
#define FUSE_SPARE_REALIGNMENT_REG_0		0x1fc

/*
 * T210: Layout of bits in FUSE_TSENSOR_COMMON:
 *    3                   2                   1                   0
 *  1 0 9 8 7 6 5 4 3 2 1 0 9 8 7 6 5 4 3 2 1 0 9 8 7 6 5 4 3 2 1 0
 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * |       BASE_FT       |      BASE_CP      | SHFT_FT | SHIFT_CP  |
 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *
 * In chips prior to T210, this fuse was incorrectly sized as 26 bits,
 * and didn't hold SHIFT_CP in [31:26]. Therefore these missing six bits
 * were obtained via the FUSE_SPARE_REALIGNMENT_REG register [5:0].
 *
 * T12x, T13x, etc: FUSE_TSENSOR_COMMON:
 *    3                   2                   1                   0
 *  1 0 9 8 7 6 5 4 3 2 1 0 9 8 7 6 5 4 3 2 1 0 9 8 7 6 5 4 3 2 1 0
 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * |-----------| SHFT_FT |       BASE_FT       |      BASE_CP      |
 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *
 * FUSE_SPARE_REALIGNMENT_REG:
 *    3                   2                   1                   0
 *  1 0 9 8 7 6 5 4 3 2 1 0 9 8 7 6 5 4 3 2 1 0 9 8 7 6 5 4 3 2 1 0
 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * |---------------------------------------------------| SHIFT_CP  |
 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 */

#define FUSE_BASE_CP_MASK(chipid)	\
		((chipid == CHIPID_TEGRA21X) ? (0x3ff << 11) : 0x3ff)
#define FUSE_BASE_CP_SHIFT(chipid)	\
		((chipid == CHIPID_TEGRA21X) ? 11 : 0)
#define FUSE_BASE_FT_MASK(chipid)	\
		((chipid == CHIPID_TEGRA21X) ? (0x7ff << 21) : (0x7ff << 10))
#define FUSE_BASE_FT_SHIFT(chipid)	\
		((chipid == CHIPID_TEGRA21X) ? 21 : 10)

#define FUSE_SHIFT_FT_MASK(chipid)	\
		((chipid == CHIPID_TEGRA21X) ? (0x1f << 6) : (0x1f << 21))
#define FUSE_SHIFT_FT_SHIFT(chipid)	\
		((chipid == CHIPID_TEGRA21X) ? 6 : 21)

static s64 div64_s64_precise(s64 a, s64 b)
{
	s64 r, al;

	/* Scale up for increased precision division */
	al = a << 16;

	r = div64_s64(al * 2 + 1, 2 * b);
	return r >> 16;
}

int tegra_soctherm_calculate_shared_calibration(
				struct tsensor_shared_calibration *r,
				enum soctherm_chipid chipid)
{
	u32 val;
	s32 shifted_cp, shifted_ft;
	int err;

	err = tegra_fuse_readl(FUSE_TSENSOR_COMMON, &val);
	if (err)
		return err;

	r->base_cp = (val & FUSE_BASE_CP_MASK(chipid))
			>> FUSE_BASE_CP_SHIFT(chipid);
	r->base_ft = (val & FUSE_BASE_FT_MASK(chipid))
			>> FUSE_BASE_FT_SHIFT(chipid);

	shifted_ft = (val & FUSE_SHIFT_FT_MASK(chipid))
			>> FUSE_SHIFT_FT_SHIFT(chipid);
	shifted_ft = sign_extend32(shifted_ft, 4);

	if ((chipid == CHIPID_TEGRA12X) || (chipid == CHIPID_TEGRA13X)) {
		err = tegra_fuse_readl(FUSE_SPARE_REALIGNMENT_REG_0, &val);
		if (err)
			return err;
	}
	shifted_cp = sign_extend32(val, 5);

	r->actual_temp_cp = 2 * NOMINAL_CALIB_CP + shifted_cp;
	r->actual_temp_ft = 2 * NOMINAL_CALIB_FT + shifted_ft;

	return 0;
}

int tegra_soctherm_calculate_tsensor_calibration(
				struct tegra_tsensor *sensor,
				struct tsensor_shared_calibration *shared)
{
	struct tegra_tsensor_group *sensor_group;
	u32 val, calib;
	s32 actual_tsensor_ft, actual_tsensor_cp;
	s32 delta_sens, delta_temp;
	s32 mult, div;
	s16 therma, thermb;
	int err;

	sensor_group = sensor->group;

	err = tegra_fuse_readl(sensor->calib_fuse_offset, &val);
	if (err)
		return err;

	actual_tsensor_cp = (shared->base_cp * 64) + sign_extend32(val, 12);
	val = (val & FUSE_TSENSOR_CALIB_FT_TS_BASE_MASK)
		>> FUSE_TSENSOR_CALIB_FT_TS_BASE_SHIFT;
	actual_tsensor_ft = (shared->base_ft * 32) + sign_extend32(val, 12);

	delta_sens = actual_tsensor_ft - actual_tsensor_cp;
	delta_temp = shared->actual_temp_ft - shared->actual_temp_cp;

	mult = sensor_group->pdiv * sensor->config->tsample_ate;
	div = sensor->config->tsample * sensor_group->pdiv_ate;

	therma = div64_s64_precise((s64)delta_temp * (1LL << 13) * mult,
			(s64)delta_sens * div);
	thermb = div64_s64_precise(
			((s64)actual_tsensor_ft * shared->actual_temp_cp) -
			((s64)actual_tsensor_cp * shared->actual_temp_ft),
			(s64)delta_sens);

	therma = div64_s64_precise((s64)therma * sensor->fuse_corr_alpha,
			(s64)1000000LL);
	thermb = div64_s64_precise((s64)thermb * sensor->fuse_corr_alpha +
			sensor->fuse_corr_beta,
			(s64)1000000LL);
	calib = ((u16)therma << (ffs(SENSOR_CONFIG2_THERMA_MASK) - 1)) |
		 ((u16)thermb << (ffs(SENSOR_CONFIG2_THERMB_MASK) - 1));

	sensor->calib = calib;

	return 0;
}

MODULE_AUTHOR("Mikko Perttunen <mperttunen@nvidia.com>");
MODULE_DESCRIPTION("Tegra SOCTHERM fuse management");
MODULE_LICENSE("GPL v2");
