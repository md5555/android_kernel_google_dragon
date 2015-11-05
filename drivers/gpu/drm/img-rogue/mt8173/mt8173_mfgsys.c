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
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>

#include "mt8173_mfgdvfs.h"
#include "mt8173_mfgsys.h"

static const char * const top_mfg_clk_name[] = {
	"mfg_mem_in_sel",
	"mfg_axi_in_sel",
	"top_axi",
	"top_mem",
	"top_mfg",
};

#define MAX_TOP_MFG_CLK ARRAY_SIZE(top_mfg_clk_name)

static struct platform_device *sPVRLDMDev;
#define GET_MTK_MFG_BASE(x) (struct mtk_mfg_base *)(x->dev.platform_data)

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

static int mtk_mfg_prepare_clock(struct mtk_mfg_base *mfg_base)
{
	int i;
	int ret;

	for (i = 0; i < MAX_TOP_MFG_CLK; i++) {
		ret = clk_prepare(mfg_base->top_clk[i]);
		if (ret)
			goto unwind;
	}

	return 0;
unwind:
	while (i--)
		clk_unprepare(mfg_base->top_clk[i]);

	return ret;
}

static void mtk_mfg_unprepare_clock(struct mtk_mfg_base *mfg_base)
{
	int i;

	for (i = MAX_TOP_MFG_CLK - 1; i >= 0; i--)
		clk_unprepare(mfg_base->top_clk[i]);
}

static int mtk_mfg_enable_clock(struct mtk_mfg_base *mfg_base)
{
	int i;
	int ret;

	pm_runtime_get_sync(&mfg_base->pdev->dev);
	for (i = 0; i < MAX_TOP_MFG_CLK; i++) {
		ret = clk_enable(mfg_base->top_clk[i]);
		if (ret)
			goto unwind;
	}
	mtk_mfg_clr_clock_gating(mfg_base->reg_base);

	return 0;
unwind:
	while (i--)
		clk_disable(mfg_base->top_clk[i]);

	return ret;
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

static int mtk_mfg_bind_device_resource(struct platform_device *pdev,
					struct mtk_mfg_base *mfg_base)
{
	int i, err;
	int len = sizeof(struct clk *) * MAX_TOP_MFG_CLK;
	struct resource *res;

	mfg_base->top_clk = devm_kzalloc(&pdev->dev, len, GFP_KERNEL);
	if (!mfg_base->top_clk)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;
	mfg_base->rgx_start = res->start;
	mfg_base->rgx_size = resource_size(res);

	mfg_base->rgx_irq = platform_get_irq_byname(pdev, "RGX");
	if (mfg_base->rgx_irq < 0)
		return mfg_base->rgx_irq;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	mfg_base->reg_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(mfg_base->reg_base))
		return PTR_ERR(mfg_base->reg_base);

	mfg_base->mmpll = devm_clk_get(&pdev->dev, "mmpll_clk");
	if (IS_ERR(mfg_base->mmpll)) {
		dev_err(&pdev->dev, "devm_clk_get mmpll_clk failed !!!\n");
		return PTR_ERR(mfg_base->mmpll);
	}

	for (i = 0; i < MAX_TOP_MFG_CLK; i++) {
		mfg_base->top_clk[i] = devm_clk_get(&pdev->dev,
						    top_mfg_clk_name[i]);
		if (IS_ERR(mfg_base->top_clk[i])) {
			dev_err(&pdev->dev, "devm_clk_get %s failed !!!\n",
				top_mfg_clk_name[i]);
			return PTR_ERR(mfg_base->top_clk[i]);
		}
	}

	mfg_base->vgpu = devm_regulator_get(&pdev->dev, "mfgsys-power");
	if (IS_ERR(mfg_base->vgpu))
		return PTR_ERR(mfg_base->vgpu);

	err = regulator_enable(mfg_base->vgpu);
	if (err != 0) {
		dev_err(&pdev->dev, "failed to enable regulator vgpu\n");
		return err;
	}

	err = of_init_opp_table(&pdev->dev);
	if (err) {
		dev_err(&pdev->dev, "failed to init opp table, %d\n", err);
		goto err_regulator_disable;
	}

	pm_runtime_enable(&pdev->dev);
	mfg_base->pdev = pdev;

	return 0;

err_regulator_disable:
	regulator_disable(mfg_base->vgpu);

	return err;
}

static void mtk_mfg_unbind_device_resource(struct platform_device *pdev,
				    struct mtk_mfg_base *mfg_base)
{
	pr_info("mtk_mfg_unbind_device_resource start\n");

	mfg_base->pdev = NULL;
	pm_runtime_disable(&pdev->dev);
	of_free_opp_table(&pdev->dev);
	regulator_disable(mfg_base->vgpu);

	pr_info("mtk_mfg_unbind_device_resource end\n");
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
		mtk_mfg_disable_hw_apm(mfg_base);
		mtk_mfg_disable_clock(mfg_base);
	}

	mutex_unlock(&mfg_base->set_power_state);
	return PVRSRV_OK;
}

PVRSRV_ERROR MTKSysDevPostPowerState(PVRSRV_DEV_POWER_STATE eNewPowerState,
				     PVRSRV_DEV_POWER_STATE eCurrentPowerState,
				     IMG_BOOL bForced)
{
	struct mtk_mfg_base *mfg_base = GET_MTK_MFG_BASE(sPVRLDMDev);
	PVRSRV_ERROR ret;

	mtk_mfg_debug("MTKSysDevPostPowerState (%d->%d)\n",
		      eCurrentPowerState, eNewPowerState);

	mutex_lock(&mfg_base->set_power_state);

	if ((PVRSRV_DEV_POWER_STATE_ON == eNewPowerState) &&
	    (PVRSRV_DEV_POWER_STATE_OFF == eCurrentPowerState)) {
		if (mtk_mfg_enable_clock(mfg_base)) {
			ret = PVRSRV_ERROR_DEVICE_POWER_CHANGE_FAILURE;
			goto done;
		}
		mtk_mfg_enable_hw_apm(mfg_base);
	}

	ret = PVRSRV_OK;
done:
	mutex_unlock(&mfg_base->set_power_state);

	return ret;
}

PVRSRV_ERROR MTKSystemPrePowerState(PVRSRV_SYS_POWER_STATE eNewPowerState)
{
	int err;
	struct mtk_mfg_base *mfg_base = GET_MTK_MFG_BASE(sPVRLDMDev);

	pr_err("MTKSystemPrePowerState() eNewPowerState %d\n", eNewPowerState);
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

	err = mtk_mfg_prepare_clock(mfg_base);
	if (err)
		goto err_unbind_resource;

	/* attach mfg_base to pdev->dev.platform_data */
	pdev->dev.platform_data = mfg_base;
	sPVRLDMDev = pdev;

	mtk_mfg_debug("MTKMFGBaseInit End\n");

	return 0;
err_unbind_resource:
	mtk_mfg_unbind_device_resource(pdev, mfg_base);

	return err;
}

void MTKMFGBaseDeInit(struct platform_device *pdev)
{
	struct mtk_mfg_base *mfg_base = GET_MTK_MFG_BASE(sPVRLDMDev);

	mtk_mfg_unprepare_clock(mfg_base);

	mtk_mfg_unbind_device_resource(pdev, mfg_base);
}
