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

#ifndef MT8173_MFGSYS_H
#define MT8173_MFGSYS_H

#include <linux/platform_device.h>

/* unit ms, timeout interval for DVFS detection */
#define MTK_DVFS_SWITCH_INTERVAL  300

#define ENABLE_MTK_MFG_DEBUG 0

#if ENABLE_MTK_MFG_DEBUG
#define mtk_mfg_debug(fmt, args...) pr_info("[MFG]" fmt, ##args)
#else
#define mtk_mfg_debug(fmt, args...) do { } while (0)
#endif

struct mtk_mfg {
	struct device *dev;

	struct clk **top_clk;
	void __iomem *reg_base;

	resource_size_t rgx_start;
	resource_size_t rgx_size;
	int rgx_irq;

	/* mutex protect for set power state */
	struct mutex set_power_state;

	/* for gpu device freq/volt update */
	struct regulator *vgpu;
	struct clk *mmpll;

	u32 curr_freq; /* kHz */
	u32 curr_volt; /* uV  */
};


/* used in mtk_module.c  */
int MTKMFGBaseInit(struct device *dev);
void MTKMFGBaseDeInit(struct device *dev);

int mtk_mfg_enable(struct mtk_mfg *mfg);
void mtk_mfg_disable(struct mtk_mfg *mfg);

void MTKSysSetFreq(struct mtk_mfg *mfg, u32 freq);
void MTKSysSetVolt(struct mtk_mfg *mfg, u32 volt);

#endif /* MT8173_MFGSYS_H*/
