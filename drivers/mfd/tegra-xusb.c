/*
 * NVIDIA Tegra XUSB MFD driver
 *
 * Copyright (C) 2015 Google, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 */

#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mfd/core.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>

static const struct of_device_id tegra_xusb_of_match[] = {
	{ .compatible = "nvidia,tegra124-xusb", },
	{},
};
MODULE_DEVICE_TABLE(of, tegra_xusb_of_match);

static struct regmap_config tegra_fpci_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
};

static int tegra_xusb_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct regmap *fpci_regs;
	void __iomem *fpci_base;
	int ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	fpci_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(fpci_base))
		return PTR_ERR(fpci_base);

	tegra_fpci_regmap_config.max_register = res->end - res->start - 3;
	fpci_regs = devm_regmap_init_mmio(&pdev->dev, fpci_base,
					  &tegra_fpci_regmap_config);
	if (IS_ERR(fpci_regs)) {
		ret = PTR_ERR(fpci_regs);
		dev_err(&pdev->dev, "Failed to init regmap: %d\n", ret);
		return ret;
	}
	platform_set_drvdata(pdev, fpci_regs);

	ret = of_platform_populate(pdev->dev.of_node, NULL, NULL, &pdev->dev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to add sub-devices: %d\n", ret);
		return ret;
	}

	return 0;
}

static struct platform_driver tegra_xusb_driver = {
	.probe = tegra_xusb_probe,
	.driver = {
		.name = "tegra-xusb",
		.of_match_table = tegra_xusb_of_match,
	},
};
module_platform_driver(tegra_xusb_driver);

MODULE_DESCRIPTION("NVIDIA Tegra XUSB MFD");
MODULE_AUTHOR("Andrew Bresticker <abrestic@chromium.org>");
MODULE_LICENSE("GPL v2");
