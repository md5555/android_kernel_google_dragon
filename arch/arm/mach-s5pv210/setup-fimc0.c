/* linux/arch/arm/mach-s5pv210/setup-fimc0.c
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * Base FIMC 0 gpio configuration
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <plat/clock.h>
#include <plat/gpio-cfg.h>
#include <linux/io.h>
#include <plat/map-s5p.h>
#include <mach/map.h>
#include <mach/pd.h>

struct platform_device; /* don't need the contents */

void s3c_fimc0_cfg_gpio(struct platform_device *pdev)
{
	/* CAM MCLK */
	s3c_gpio_cfgpin(S5PV210_GPE1(3), S3C_GPIO_SFN(2));
	s3c_gpio_setpull(S5PV210_GPE1(3), S3C_GPIO_PULL_NONE);
}

int s3c_fimc_clk_on(struct platform_device *pdev, struct clk *clk)
{
	struct clk *sclk_fimc_lclk = NULL;
	struct clk *mout_fimc_lclk = NULL;
	struct clk *mout_mpll = NULL;
	int ret;

	mout_mpll = clk_get(&pdev->dev, "mout_mpll");
	if (IS_ERR(mout_mpll)) {
		dev_err(&pdev->dev, "failed to get mout_mpll\n");
		goto err_clk1;
	}

	mout_fimc_lclk = clk_get(&pdev->dev, "mout_fimc_lclk");
	if (IS_ERR(mout_fimc_lclk)) {
		dev_err(&pdev->dev, "failed to get mout_fimc_lclk\n");
		goto err_clk2;
	}

	sclk_fimc_lclk = clk_get(&pdev->dev, "sclk_fimc_lclk");
	if (IS_ERR(sclk_fimc_lclk)) {
		dev_err(&pdev->dev, "failed to get sclk_fimc_lclk\n");
		goto err_clk3;
	}

	clk_set_parent(mout_fimc_lclk, mout_mpll);
	clk_set_rate(sclk_fimc_lclk, 166750000);

	/* be able to handle clock on/off only with this clock */
	clk = clk_get(&pdev->dev, "fimc");
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "failed to get interface clock\n");
		goto err_clk3;
	}

	clk_put(mout_mpll);
	clk_put(mout_fimc_lclk);

	ret = s5pv210_pd_enable("fimc_pd");
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to enable fimc power domain\n");
		goto err_clk3;
	}

	clk_enable(clk);

	return 0;

err_clk3:
	clk_put(mout_fimc_lclk);

err_clk2:
	clk_put(mout_mpll);

err_clk1:
	return -EINVAL;
}

int s3c_fimc0_clk_off(struct platform_device *pdev, struct clk *clk)
{
	int ret;

	clk_disable(clk);
	clk_put(clk);

	clk = NULL;
	ret = s5pv210_pd_disable("fimc_pd");
	if (ret < 0)
		dev_err(&pdev->dev, "failed to disable fimc power domain\n");

	return 0;
}
