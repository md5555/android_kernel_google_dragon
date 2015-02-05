/*
 * Copyright (C) 2014 NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/of.h>
#include <linux/slab.h>
#include <linux/sys_soc.h>

#include <soc/tegra/common.h>
#include <soc/tegra/fuse.h>

static const struct of_device_id tegra_machine_match[] = {
	{ .compatible = "nvidia,tegra20", },
	{ .compatible = "nvidia,tegra30", },
	{ .compatible = "nvidia,tegra114", },
	{ .compatible = "nvidia,tegra124", },
	{ .compatible = "nvidia,tegra210", },
	{ }
};

bool soc_is_tegra(void)
{
	struct device_node *root;

	root = of_find_node_by_path("/");
	if (!root)
		return false;

	return of_match_node(tegra_machine_match, root) != NULL;
}

static int __init tegra_soc_init(void)
{
	struct soc_device_attribute *soc_dev_attr;
	struct soc_device *soc_dev;

	if (!soc_is_tegra())
		return 0;

	soc_dev_attr = kzalloc(sizeof(*soc_dev_attr), GFP_KERNEL);
	if (!soc_dev_attr)
		return -ENOMEM;

	soc_dev_attr->family = kasprintf(GFP_KERNEL, "Tegra");
	soc_dev_attr->revision = kasprintf(GFP_KERNEL, "%d",
					   tegra_sku_info.revision);
	soc_dev_attr->soc_id = kasprintf(GFP_KERNEL, "%u", tegra_get_chip_id());

	soc_dev = soc_device_register(soc_dev_attr);
	if (IS_ERR(soc_dev)) {
		kfree(soc_dev_attr->family);
		kfree(soc_dev_attr->revision);
		kfree(soc_dev_attr->soc_id);
		kfree(soc_dev_attr);
		return -ENODEV;
	}

	return 0;
}
device_initcall(tegra_soc_init);
