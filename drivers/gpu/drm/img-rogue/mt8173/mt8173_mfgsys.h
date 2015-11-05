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

#include "servicesext.h"
#include "rgxdevice.h"

#include <linux/platform_device.h>

/* control APM is enabled or not  */
#define MTK_PM_SUPPORT 1
#define MTK_ENABLE_HWAPM 1

/*  unit ms, timeout interval for DVFS detection */
#define MTK_DVFS_SWITCH_INTERVAL  300

#define ENABLE_MTK_MFG_DEBUG 0

#if ENABLE_MTK_MFG_DEBUG
#define mtk_mfg_debug(fmt, args...) pr_info("[MFG]" fmt, ##args)
#else
#define mtk_mfg_debug(fmt, args...) do { } while (0)
#endif
/*
 * freq : khz, volt : uV
 */
struct mfgsys_fv_table {
	u32 freq;
	u32 volt;
};

struct mtk_mfg_base {
	struct platform_device *pdev;

	struct clk **top_clk;
	void __iomem *reg_base;

	/* mutex protect for set power state */
	struct mutex set_power_state;
	bool power_on;

	/* for gpu device freq/volt update */
	struct regulator *vgpu;
	struct clk *mmpll;
	struct mfgsys_fv_table *fv_table;
	u32  fv_table_length;

	u32 curr_freq; /* kHz */
	u32 curr_volt; /* uV  */
};


/*used in mtk_module.c  */
int MTKMFGBaseInit(struct platform_device *pdev);
int MTKMFGBaseDeInit(struct platform_device *pdev);

/* below register interface in RGX sysconfig.c */
PVRSRV_ERROR MTKSysDevPrePowerState(PVRSRV_DEV_POWER_STATE eNew,
					   PVRSRV_DEV_POWER_STATE eCurrent,
					   IMG_BOOL bForced);
PVRSRV_ERROR MTKSysDevPostPowerState(PVRSRV_DEV_POWER_STATE eNew,
					    PVRSRV_DEV_POWER_STATE eCurrent,
					    IMG_BOOL bForced);
PVRSRV_ERROR MTKSystemPrePowerState(PVRSRV_SYS_POWER_STATE eNew);
PVRSRV_ERROR MTKSystemPostPowerState(PVRSRV_SYS_POWER_STATE eNew);

void MTKSysSetFreq(struct mtk_mfg_base *base, u32 freq);
void MTKSysSetVolt(struct mtk_mfg_base *base, u32 volt);

#endif /* MT8173_MFGSYS_H*/
