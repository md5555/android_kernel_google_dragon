/*
* Copyright (c) 2014 MediaTek Inc.
* Author: Chiawen Lee <chiawen.lee@mediatek.com>
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*/
#include "servicesext.h"

#ifndef MT8173_MFGDVFS_H
#define MT8173_MFGDVFS_H

struct mtk_mfg_base;
/*********************
* GPU Frequency List
**********************/
/* GPU_DVFS_FREQ0 Used for fixed freq-volt setting maximum boundary */
#define GPU_DVFS_FREQ0                  (700000) /* KHz */
/* GPU_DVFS_FREQ1-GPU_DVFS_FREQ6 Used for dvfs freq-volt table */
#define GPU_DVFS_FREQ1                  (598000) /* KHz */
#define GPU_DVFS_FREQ2                  (494000) /* KHz */
#define GPU_DVFS_FREQ3                  (455000) /* KHz */
#define GPU_DVFS_FREQ4                  (396500) /* KHz */
#define GPU_DVFS_FREQ5                  (299000) /* KHz */
#define GPU_DVFS_FREQ6                  (253500) /* KHz */

#define GPU_DVFS_VOLT1                  (1130)   /* mV */
#define GPU_DVFS_VOLT2                  (1000)   /* mV */

#define GPU_DVFS_INIT_LVL               (3)
#define GPU_DVFS_PTPOD_DISABLE_VOLT     (GPU_DVFS_VOLT2)

#define GPU_ACT_REF_POWER               (530)    /* mW  */
#define GPU_ACT_REF_FREQ                (455000) /* KHz */
#define GPU_ACT_REF_VOLT                (1000)   /* mV */

/* us, (DA9212 externel buck) I2C command delay 100us, Buck 10mv/us */
#define GPU_DVFS_VOLT_SETTLE_TIME(uv_old, uv_new) \
	(((((uv_old) > (uv_new)) ? \
	((uv_old)-(uv_new)) : ((uv_new)-(uv_old))) / 1000 + 9) / 10 + 100)

#define GPU_VOLT_TO_EXTBUCK_MAXVAL(volt) (volt + 9999)

void mtk_mfg_gpu_get_current_power(struct mtk_mfg_base *mfg_base);
void mtk_mfg_gpu_dvfs_init(struct mtk_mfg_base *mfg_base);
void mtk_mfg_gpu_dvfs_deinit(struct mtk_mfg_base *mfg_base);

#endif /* MT8173_MFGDVFS_H */
