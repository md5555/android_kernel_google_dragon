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

static int mtk_mfg_prepare_clock(struct mtk_mfg *mfg)
{
	int i;
	int ret;

	for (i = 0; i < MAX_TOP_MFG_CLK; i++) {
		ret = clk_prepare(mfg->top_clk[i]);
		if (ret)
			goto unwind;
	}

	return 0;
unwind:
	while (i--)
		clk_unprepare(mfg->top_clk[i]);

	return ret;
}

static void mtk_mfg_unprepare_clock(struct mtk_mfg *mfg)
{
	int i;

	for (i = MAX_TOP_MFG_CLK - 1; i >= 0; i--)
		clk_unprepare(mfg->top_clk[i]);
}

static int mtk_mfg_enable_clock(struct mtk_mfg *mfg)
{
	int i;
	int ret;

	pm_runtime_get_sync(&mfg->pdev->dev);
	for (i = 0; i < MAX_TOP_MFG_CLK; i++) {
		ret = clk_enable(mfg->top_clk[i]);
		if (ret)
			goto unwind;
	}
	mtk_mfg_clr_clock_gating(mfg->reg_base);

	return 0;
unwind:
	while (i--)
		clk_disable(mfg->top_clk[i]);

	return ret;
}

static void mtk_mfg_disable_clock(struct mtk_mfg *mfg)
{
	int i;

	mtk_mfg_set_clock_gating(mfg->reg_base);
	for (i = MAX_TOP_MFG_CLK - 1; i >= 0; i--)
		clk_disable(mfg->top_clk[i]);
	pm_runtime_put_sync(&mfg->pdev->dev);
}

static void mtk_mfg_enable_hw_apm(struct mtk_mfg *mfg)
{
	writel(0x003c3d4d, mfg->reg_base + 0x24);
	writel(0x4d45440b, mfg->reg_base + 0x28);
	writel(0x7a710184, mfg->reg_base + 0xe0);
	writel(0x835f6856, mfg->reg_base + 0xe4);
	writel(0x002b0234, mfg->reg_base + 0xe8);
	writel(0x80000000, mfg->reg_base + 0xec);
	writel(0x08000000, mfg->reg_base + 0xa0);
}

static void mtk_mfg_disable_hw_apm(struct mtk_mfg *mfg)
{
	writel(0x00, mfg->reg_base + 0x24);
	writel(0x00, mfg->reg_base + 0x28);
	writel(0x00, mfg->reg_base + 0xe0);
	writel(0x00, mfg->reg_base + 0xe4);
	writel(0x00, mfg->reg_base + 0xe8);
	writel(0x00, mfg->reg_base + 0xec);
	writel(0x00, mfg->reg_base + 0xa0);
}

int mtk_mfg_enable(struct mtk_mfg *mfg)
{
	int ret;

	ret = mtk_mfg_enable_clock(mfg);
	if (ret)
		return ret;

	mtk_mfg_enable_hw_apm(mfg);

	return 0;
}

void mtk_mfg_disable(struct mtk_mfg *mfg)
{
	mtk_mfg_disable_hw_apm(mfg);
	mtk_mfg_disable_clock(mfg);
}

static int mtk_mfg_bind_device_resource(struct platform_device *pdev,
					struct mtk_mfg *mfg)
{
	int i, err;
	struct resource *res;

	mfg->top_clk = devm_kcalloc(&pdev->dev, MAX_TOP_MFG_CLK,
				    sizeof(*mfg->top_clk), GFP_KERNEL);
	if (!mfg->top_clk)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;
	mfg->rgx_start = res->start;
	mfg->rgx_size = resource_size(res);

	mfg->rgx_irq = platform_get_irq_byname(pdev, "RGX");
	if (mfg->rgx_irq < 0)
		return mfg->rgx_irq;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	mfg->reg_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(mfg->reg_base))
		return PTR_ERR(mfg->reg_base);

	mfg->mmpll = devm_clk_get(&pdev->dev, "mmpll_clk");
	if (IS_ERR(mfg->mmpll)) {
		dev_err(&pdev->dev, "devm_clk_get mmpll_clk failed !!!\n");
		return PTR_ERR(mfg->mmpll);
	}

	for (i = 0; i < MAX_TOP_MFG_CLK; i++) {
		mfg->top_clk[i] = devm_clk_get(&pdev->dev,
						    top_mfg_clk_name[i]);
		if (IS_ERR(mfg->top_clk[i])) {
			dev_err(&pdev->dev, "devm_clk_get %s failed !!!\n",
				top_mfg_clk_name[i]);
			return PTR_ERR(mfg->top_clk[i]);
		}
	}

	mfg->vgpu = devm_regulator_get(&pdev->dev, "mfgsys-power");
	if (IS_ERR(mfg->vgpu))
		return PTR_ERR(mfg->vgpu);

	err = regulator_enable(mfg->vgpu);
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
	mfg->pdev = pdev;

	return 0;

err_regulator_disable:
	regulator_disable(mfg->vgpu);

	return err;
}

static void mtk_mfg_unbind_device_resource(struct platform_device *pdev,
				    struct mtk_mfg *mfg)
{
	pr_info("mtk_mfg_unbind_device_resource start\n");

	mfg->pdev = NULL;
	pm_runtime_disable(&pdev->dev);
	of_free_opp_table(&pdev->dev);
	regulator_disable(mfg->vgpu);

	pr_info("mtk_mfg_unbind_device_resource end\n");
}

int MTKMFGBaseInit(struct platform_device *pdev)
{
	int err;
	struct mtk_mfg *mfg;

	mtk_mfg_debug("MTKMFGBaseInit Begin\n");

	mfg = devm_kzalloc(&pdev->dev, sizeof(*mfg), GFP_KERNEL);
	if (!mfg)
		return -ENOMEM;

	err = mtk_mfg_bind_device_resource(pdev, mfg);
	if (err != 0)
		return err;

	mutex_init(&mfg->set_power_state);

	err = mtk_mfg_prepare_clock(mfg);
	if (err)
		goto err_unbind_resource;

	/* attach mfg to pdev->dev.platform_data */
	pdev->dev.platform_data = mfg;

	mtk_mfg_debug("MTKMFGBaseInit End\n");

	return 0;
err_unbind_resource:
	mtk_mfg_unbind_device_resource(pdev, mfg);

	return err;
}

void MTKMFGBaseDeInit(struct platform_device *pdev)
{
	struct mtk_mfg *mfg = dev_get_platdata(&pdev->dev);

	mtk_mfg_unprepare_clock(mfg);

	mtk_mfg_unbind_device_resource(pdev, mfg);
}
