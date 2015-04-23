/*
 * Copyright (c) 2015, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/reset.h>

#include <soc/tegra/kfuse.h>

#define KFUSE_STATE 0x80
#define  KFUSE_STATE_CRCPASS	(1 << 17)
#define  KFUSE_STATE_DONE	(1 << 16)

#define KFUSE_ERRCOUNT 0x84

#define KFUSE_KEYADDR 0x88
#define  KFUSE_KEYADDR_AUTOINC (1 << 16)
#define  KFUSE_KEYADDR_ADDR(x) (((x) & 0xff) << 0)

#define KFUSE_KEYS 0x8c

static const struct of_device_id tegra_kfuse_match[] = {
	{ .compatible = "nvidia,tegra210-kfuse" },
	{ /* sentinel */ }
};

struct tegra_kfuse {
	struct device *dev;
	void __iomem *base;
	struct clk *clk;
	struct reset_control *rst;

	size_t size;
};

static int tegra_kfuse_wait_for_decode(struct tegra_kfuse *kfuse,
				       unsigned long timeout)
{
	u32 value;

	timeout = jiffies + msecs_to_jiffies(timeout);

	while (time_before(jiffies, timeout)) {
		value = readl(kfuse->base + KFUSE_STATE);
		if (value & KFUSE_STATE_DONE)
			return 0;

		usleep_range(100, 1000);
	}

	return -ETIMEDOUT;
}

static int tegra_kfuse_wait_for_crc(struct tegra_kfuse *kfuse,
				    unsigned long timeout)
{
	u32 value;

	timeout = jiffies + msecs_to_jiffies(timeout);

	while (time_before(jiffies, timeout)) {
		value = readl(kfuse->base + KFUSE_STATE);
		if (value & KFUSE_STATE_CRCPASS)
			return 0;

		usleep_range(100, 1000);
	}

	return -ETIMEDOUT;
}

static int tegra_kfuse_probe(struct platform_device *pdev)
{
	struct tegra_kfuse *kfuse;
	struct resource *regs;
	int err = 0;

	kfuse = devm_kzalloc(&pdev->dev, sizeof(*kfuse), GFP_KERNEL);
	if (!kfuse)
		return -ENOMEM;

	kfuse->dev = &pdev->dev;

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	kfuse->base = devm_ioremap_resource(&pdev->dev, regs);
	if (IS_ERR(kfuse->base))
		return PTR_ERR(kfuse->base);

	kfuse->clk = devm_clk_get(&pdev->dev, "kfuse");
	if (IS_ERR(kfuse->clk)) {
		dev_err(&pdev->dev, "failed to get clock: %ld\n",
			PTR_ERR(kfuse->clk));
		return PTR_ERR(kfuse->clk);
	}

	err = clk_prepare_enable(kfuse->clk);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to enable clock: %d\n", err);
		return err;
	}

	kfuse->rst = devm_reset_control_get(&pdev->dev, "kfuse");
	if (IS_ERR(kfuse->rst)) {
		err = PTR_ERR(kfuse->rst);
		dev_err(&pdev->dev, "failed to get reset control: %d\n", err);
		goto disable;
	}

	err = reset_control_deassert(kfuse->rst);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to deassert reset: %d\n", err);
		goto disable;
	}

	err = tegra_kfuse_wait_for_decode(kfuse, 100);
	if (err < 0) {
		dev_err(&pdev->dev, "error waiting for decode: %d\n", err);
		goto reset;
	}

	err = tegra_kfuse_wait_for_crc(kfuse, 100);
	if (err < 0) {
		dev_err(&pdev->dev, "error waiting for CRC check: %d\n", err);
		goto reset;
	}

	/*
	 * The ECC-decoded keyglob data is 144 32-bit words (576 bytes).
	 */
	kfuse->size = 576;

	platform_set_drvdata(pdev, kfuse);

	return 0;

reset:
	reset_control_assert(kfuse->rst);
disable:
	clk_disable_unprepare(kfuse->clk);
	return err;
}

static int tegra_kfuse_remove(struct platform_device *pdev)
{
	struct tegra_kfuse *kfuse = platform_get_drvdata(pdev);
	int err = 0;

	reset_control_assert(kfuse->rst);
	clk_disable_unprepare(kfuse->clk);

	dev_info(&pdev->dev, "< %s() = %d\n", __func__, err);
	return err;
}

static struct platform_driver tegra_kfuse_driver = {
	.driver = {
		.name = "tegra-kfuse",
		.of_match_table = tegra_kfuse_match,
	},
	.probe = tegra_kfuse_probe,
	.remove = tegra_kfuse_remove,
};
module_platform_driver(tegra_kfuse_driver);

struct tegra_kfuse *tegra_kfuse_find_by_of_node(struct device_node *np)
{
	struct device *dev;

	dev = driver_find_device(&tegra_kfuse_driver.driver, NULL, np,
				 of_device_match);
	if (!dev)
		return NULL;

	return dev_get_drvdata(dev);
}
EXPORT_SYMBOL(tegra_kfuse_find_by_of_node);

ssize_t tegra_kfuse_read(struct tegra_kfuse *kfuse, void *buffer, size_t size)
{
	size_t offset;
	u32 value;

	if (!buffer && size == 0)
		return kfuse->size;

	if (size > kfuse->size)
		size = kfuse->size;

	value = KFUSE_KEYADDR_AUTOINC | KFUSE_KEYADDR_ADDR(0);
	writel(value, kfuse->base + KFUSE_KEYADDR);

	for (offset = 0; offset < size; offset += 4) {
		value = readl(kfuse->base + KFUSE_KEYS);
		memcpy(buffer + offset, &value, 4);
	}

	return offset;
}
EXPORT_SYMBOL(tegra_kfuse_read);

MODULE_DESCRIPTION("NVIDIA Tegra KFUSE driver");
MODULE_AUTHOR("Thierry Reding <treding@nvidia.com>");
MODULE_LICENSE("GPL v2");
