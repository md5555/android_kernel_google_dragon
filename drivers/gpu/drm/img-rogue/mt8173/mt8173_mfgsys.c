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
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/regulator/consumer.h>
#include <linux/version.h>
#include <linux/pm_runtime.h>

#include "mt8173_mfgsys.h"
#include "mt8173_mfgdvfs.h"

static char *top_mfg_clk_name[] = {
#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 11, 0))
	"mfg_mem_in_sel",
	"mfg_axi_in_sel",
	"top_axi",
	"top_mem",
	"top_mfg",
#else
	"MT_CG_MFG_POWER",
	"MT_CG_MFG_AXI",
	"MT_CG_MFG_MEM",
	"MT_CG_MFG_G3D",
	"MT_CG_MFG_26M",
#endif
};

#define MAX_TOP_MFG_CLK ARRAY_SIZE(top_mfg_clk_name)

static struct platform_device *sPVRLDMDev;
#define GET_MTK_MFG_BASE(x) (struct mtk_mfg_base *)(x->dev.platform_data)

#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 11, 0))

#define REG_MFG_AXI BIT(0)
#define REG_MFG_MEM BIT(1)
#define REG_MFG_G3D BIT(2)
#define REG_MFG_26M BIT(3)
#define REG_MFG_ALL (REG_MFG_AXI | REG_MFG_MEM | REG_MFG_G3D | REG_MFG_26M)

#define REG_MFG_CG_STA 0x00
#define REG_MFG_CG_SET 0x04
#define REG_MFG_CG_CLR 0x08

static void mtk_mfg_set_clock_gating(void __iomem *reg)
{
	writel(REG_MFG_ALL, reg + REG_MFG_CG_SET);
}

static void mtk_mfg_clr_clock_gating(void __iomem *reg)
{
	writel(REG_MFG_ALL, reg + REG_MFG_CG_CLR);
}
#else
static void mtk_mfg_set_clock_gating(void __iomem *reg)
{

}

static void mtk_mfg_clr_clock_gating(void __iomem *reg)
{

}
#endif

static int mtk_mfg_prepare_clock(struct mtk_mfg_base *mfg_base)
{
	int i;

	for (i = 0; i < MAX_TOP_MFG_CLK; i++)
		clk_prepare(mfg_base->top_clk[i]);

	return PVRSRV_OK;
}

static int mtk_mfg_unprepare_clock(struct mtk_mfg_base *mfg_base)
{
	int i;

	for (i = MAX_TOP_MFG_CLK - 1; i >= 0; i--)
		clk_unprepare(mfg_base->top_clk[i]);

	return PVRSRV_OK;
}

static int mtk_mfg_enable_clock(struct mtk_mfg_base *mfg_base)
{
	int i;

	pm_runtime_get_sync(&mfg_base->pdev->dev);
	for (i = 0; i < MAX_TOP_MFG_CLK; i++)
		clk_enable(mfg_base->top_clk[i]);
	mtk_mfg_clr_clock_gating(mfg_base->reg_base);

	return PVRSRV_OK;
}

static int mtk_mfg_disable_clock(struct mtk_mfg_base *mfg_base)
{
	int i;

	mtk_mfg_set_clock_gating(mfg_base->reg_base);
	for (i = MAX_TOP_MFG_CLK - 1; i >= 0; i--)
		clk_disable(mfg_base->top_clk[i]);
	pm_runtime_put_sync(&mfg_base->pdev->dev);

	return PVRSRV_OK;
}

#if defined(MTK_ENABLE_HWAPM)
static void mtk_mfg_enable_hw_apm(struct mtk_mfg_base *mfg_base)
{
	writel(0x003c3d4d, mfg_base->reg_base + 0x24);
	writel(0x4d45440b, mfg_base->reg_base + 0x28);
	writel(0x7a710184, mfg_base->reg_base + 0xe0);
	writel(0x835f6856, mfg_base->reg_base + 0xe4);
	writel(0x002b0234, mfg_base->reg_base + 0xe8);
	writel(0x80000000, mfg_base->reg_base + 0xec);
	writel(0x08000000, mfg_base->reg_base + 0xa0);
}

static void mtk_mfg_disable_hw_apm(struct mtk_mfg_base *mfg_base)
{
	writel(0x00, mfg_base->reg_base + 0x24);
	writel(0x00, mfg_base->reg_base + 0x28);
	writel(0x00, mfg_base->reg_base + 0xe0);
	writel(0x00, mfg_base->reg_base + 0xe4);
	writel(0x00, mfg_base->reg_base + 0xe8);
	writel(0x00, mfg_base->reg_base + 0xec);
	writel(0x00, mfg_base->reg_base + 0xa0);
}
#endif

static int mtk_mfg_get_opp_table(struct platform_device *pdev,
				 struct mtk_mfg_base *mfg_base)
{
	const struct property *prop;
	int i, nr;
	const __be32 *val;

	prop = of_find_property(pdev->dev.of_node, "operating-points", NULL);
	if (!prop) {
		dev_err(&pdev->dev, "failed to fail operating-points\n");
		return -ENODEV;
	}

	if (!prop->value) {
		dev_err(&pdev->dev, "failed to get fv array data\n");
		return -ENODATA;
	}

	/*
	 * Each OPP is a set of tuples consisting of frequency and
	 * voltage like <freq-kHz vol-uV>.
	 */
	nr = prop->length / sizeof(u32);
	if (nr % 2) {
		dev_err(&pdev->dev, "Invalid OPP list\n");
		return -EINVAL;
	}

	mfg_base->fv_table_length = nr / 2;
	mfg_base->fv_table = devm_kcalloc(&pdev->dev,
					  mfg_base->fv_table_length,
					  sizeof(*mfg_base->fv_table),
					  GFP_KERNEL);
	if (!mfg_base->fv_table)
		return -ENOMEM;

	val = prop->value;

	for (i = 0; i < mfg_base->fv_table_length; ++i) {
		u32 freq = be32_to_cpup(val++);
		u32 volt = be32_to_cpup(val++);

		mfg_base->fv_table[i].freq = freq;
		mfg_base->fv_table[i].volt = volt;

		dev_info(&pdev->dev, "freq:%d kHz volt:%d uV\n", freq, volt);
	}

	return 0;
}

int mtk_mfg_bind_device_resource(struct platform_device *pdev,
				 struct mtk_mfg_base *mfg_base)
{
	int i, err;
	int len = sizeof(struct clk *) * MAX_TOP_MFG_CLK;

	mfg_base->top_clk = devm_kzalloc(&pdev->dev, len, GFP_KERNEL);
	if (!mfg_base->top_clk)
		return -ENOMEM;

	mfg_base->reg_base = of_iomap(pdev->dev.of_node, 1);
	if (!mfg_base->reg_base) {
		mtk_mfg_debug("Unable to ioremap registers pdev %p\n", pdev);
		return -ENOMEM;
	}

#ifndef MTK_MFG_DVFS
	mfg_base->mmpll = devm_clk_get(&pdev->dev, "mmpll_clk");
	if (IS_ERR(mfg_base->mmpll)) {
		err = PTR_ERR(mfg_base->mmpll);
		dev_err(&pdev->dev, "devm_clk_get mmpll_clk failed !!!\n");
		goto err_iounmap_reg_base;
	}
#endif

	for (i = 0; i < MAX_TOP_MFG_CLK; i++) {
		mfg_base->top_clk[i] = devm_clk_get(&pdev->dev,
						    top_mfg_clk_name[i]);
		if (IS_ERR(mfg_base->top_clk[i])) {
			err = PTR_ERR(mfg_base->top_clk[i]);
			dev_err(&pdev->dev, "devm_clk_get %s failed !!!\n",
				top_mfg_clk_name[i]);
			goto err_iounmap_reg_base;
		}
	}

#ifndef MTK_MFG_DVFS
	mfg_base->vgpu = devm_regulator_get(&pdev->dev, "mfgsys-power");
	if (IS_ERR(mfg_base->vgpu)) {
		err = PTR_ERR(mfg_base->vgpu);
		goto err_iounmap_reg_base;
	}

	err = regulator_enable(mfg_base->vgpu);
	if (err != 0) {
		dev_err(&pdev->dev, "failed to enable regulator vgpu\n");
		goto err_iounmap_reg_base;
	}
	mtk_mfg_get_opp_table(pdev, mfg_base);
#endif

	pm_runtime_enable(&pdev->dev);
	mfg_base->pdev = pdev;

	return 0;

err_iounmap_reg_base:
	iounmap(mfg_base->reg_base);
	return err;
}

int mtk_mfg_unbind_device_resource(struct platform_device *pdev,
				   struct mtk_mfg_base *mfg_base)
{
	pr_info("mtk_mfg_unbind_device_resource start\n");

	iounmap(mfg_base->reg_base);
#ifndef MTK_MFG_DVFS
	regulator_disable(mfg_base->vgpu);
#endif
	pm_runtime_disable(&pdev->dev);
	mfg_base->pdev = NULL;

	pr_info("mtk_mfg_unbind_device_resource end\n");
	return 0;
}

void MTKSysSetInitialPowerState(void)
{
	mtk_mfg_debug("MTKSysSetInitialPowerState ---\n");
}

void MTKSysRestoreInitialPowerState(void)
{
	mtk_mfg_debug("MTKSysRestoreInitialPowerState ---\n");
}

PVRSRV_ERROR MTKSysDevPrePowerState(PVRSRV_DEV_POWER_STATE eNewPowerState,
				    PVRSRV_DEV_POWER_STATE eCurrentPowerState,
				    IMG_BOOL bForced)
{
	struct mtk_mfg_base *mfg_base = GET_MTK_MFG_BASE(sPVRLDMDev);

	mtk_mfg_debug("MTKSysDevPrePowerState (%d->%d), bForced = %d\n",
		      eCurrentPowerState, eNewPowerState, bForced);

	mutex_lock(&mfg_base->set_power_state);

	if ((PVRSRV_DEV_POWER_STATE_OFF == eNewPowerState) &&
	    (PVRSRV_DEV_POWER_STATE_ON == eCurrentPowerState)) {
#if defined(MTK_ENABLE_HWAPM)
		mtk_mfg_disable_hw_apm(mfg_base);
#endif
		mtk_mfg_disable_clock(mfg_base);
		mfg_base->power_on = false;
	}

	mutex_unlock(&mfg_base->set_power_state);
	return PVRSRV_OK;
}

PVRSRV_ERROR MTKSysDevPostPowerState(PVRSRV_DEV_POWER_STATE eNewPowerState,
				     PVRSRV_DEV_POWER_STATE eCurrentPowerState,
				     IMG_BOOL bForced)
{
	struct mtk_mfg_base *mfg_base = GET_MTK_MFG_BASE(sPVRLDMDev);

	mtk_mfg_debug("MTKSysDevPostPowerState (%d->%d)\n",
		      eCurrentPowerState, eNewPowerState);

	mutex_lock(&mfg_base->set_power_state);

	if ((PVRSRV_DEV_POWER_STATE_ON == eNewPowerState) &&
	    (PVRSRV_DEV_POWER_STATE_OFF == eCurrentPowerState)) {
		mtk_mfg_enable_clock(mfg_base);
#if defined(MTK_ENABLE_HWAPM)
		mtk_mfg_enable_hw_apm(mfg_base);
#endif
		mfg_base->power_on = true;
	}

	mutex_unlock(&mfg_base->set_power_state);

	return PVRSRV_OK;
}

PVRSRV_ERROR MTKSystemPrePowerState(PVRSRV_SYS_POWER_STATE eNewPowerState)
{
	int err;
	struct mtk_mfg_base *mfg_base = GET_MTK_MFG_BASE(sPVRLDMDev);

	pr_err("MTKSystemPrePowerState(%d) eNewPowerState %d\n",
		      mfg_base->power_on, eNewPowerState);
#ifndef MTK_MFG_DVFS
	/* turn off regulator for power saving ~30mw */
	if (eNewPowerState == PVRSRV_SYS_POWER_STATE_OFF)
		err = regulator_disable(mfg_base->vgpu);
	else if (eNewPowerState == PVRSRV_SYS_POWER_STATE_ON)
		err = regulator_enable(mfg_base->vgpu);

	if (err != 0) {
		pr_err("failed to %s regulator vgpu\n",
		       ((eNewPowerState == PVRSRV_SYS_POWER_STATE_OFF)
		       ? "disable" : "enable"));
		return PVRSRV_ERROR_INVALID_PARAMS;
	}
#endif
	return PVRSRV_OK;
}

PVRSRV_ERROR MTKSystemPostPowerState(PVRSRV_SYS_POWER_STATE eNewPowerState)
{
	mtk_mfg_debug("MTKSystemPostPowerState eNewPowerState %d\n",
		      eNewPowerState);

	return PVRSRV_OK;
}

int MTKMFGBaseInit(struct platform_device *pdev)
{
	int err;
	struct mtk_mfg_base *mfg_base;

	mtk_mfg_debug("MTKMFGBaseInit Begin\n");

	mfg_base = devm_kzalloc(&pdev->dev, sizeof(*mfg_base), GFP_KERNEL);
	if (!mfg_base)
		return -ENOMEM;

	err = mtk_mfg_bind_device_resource(pdev, mfg_base);
	if (err != 0)
		return err;

	mutex_init(&mfg_base->set_power_state);

	mtk_mfg_prepare_clock(mfg_base);
#if !defined(PVR_DVFS) && !defined(MTK_MFG_DVFS)
	mtk_mfg_gpu_dvfs_init(mfg_base);
#endif

	/* attach mfg_base to pdev->dev.platform_data */
	pdev->dev.platform_data = mfg_base;
	sPVRLDMDev = pdev;

	mtk_mfg_debug("MTKMFGBaseInit End\n");
	return 0;
}

int MTKMFGBaseDeInit(struct platform_device *pdev)
{
	struct mtk_mfg_base *mfg_base = GET_MTK_MFG_BASE(sPVRLDMDev);

	if (pdev != sPVRLDMDev) {
		dev_err(&pdev->dev, "release %p != %p\n", pdev, sPVRLDMDev);
		return 0;
	}

	mtk_mfg_unprepare_clock(mfg_base);
#if !defined(PVR_DVFS) && !defined(MTK_MFG_DVFS)
	mtk_mfg_gpu_dvfs_deinit(mfg_base);
#endif

	mtk_mfg_unbind_device_resource(pdev, mfg_base);
	return 0;
}


int MTKMFGSystemInit(void)
{
	mtk_mfg_debug("MTKMFGSystemInit\n");
	return PVRSRV_OK;
}

int MTKMFGSystemDeInit(void)
{
	mtk_mfg_debug("MTKMFGSystemDeInit\n");
	return PVRSRV_OK;
}

