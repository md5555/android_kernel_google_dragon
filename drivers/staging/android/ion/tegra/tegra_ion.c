/*
 * drivers/gpu/tegra/tegra_ion.c
 *
 * Copyright (C) 2011 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/err.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include "../ion.h"
#include "../ion_priv.h"

static struct ion_device *idev;
static int num_heaps;
static struct ion_heap **heaps;

static struct ion_platform_heap tegra210_heaps[] = {
	{
		.id	= ION_HEAP_TYPE_SYSTEM,
		.type	= ION_HEAP_TYPE_SYSTEM,
		.name	= "system",
	},
	{
		.id	= ION_HEAP_TYPE_CARVEOUT,
		.type	= ION_HEAP_TYPE_CARVEOUT,
		.name	= "vpr",
		.flags	= ION_HEAP_FLAG_DEVICE_MEM,
	},
};

static struct ion_platform_data tegra210_ion_pdata = {
	.nr = ARRAY_SIZE(tegra210_heaps),
	.heaps = tegra210_heaps,
};

static const struct of_device_id ion_of_ids[] = {
	{
		.compatible = "nvidia,tegra210-ion-heaps",
		.data = &tegra210_ion_pdata
	},
	{},
};

static ion_phys_addr_t vpr_start;
static size_t vpr_size;

static int __init tegra_vpr_arg(char *p)
{
	vpr_size = memparse(p, &p);
	if (*p == '@')
		vpr_start = memparse(p + 1, NULL);

	pr_info("Found vpr, start=0x%lx size=0x%zx\n", vpr_start, vpr_size);
	return 0;
}
early_param("vpr", tegra_vpr_arg);

static int tegra_ion_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct ion_platform_data *pdata;
	int err;
	int i;

	match = of_match_device(ion_of_ids, &pdev->dev);
	if (!match) {
		dev_err(&pdev->dev, "Error: No device match found\n");
		return -ENODEV;
	}
	pdata = (struct ion_platform_data *)match->data;

	num_heaps = pdata->nr;
	heaps = devm_kzalloc(&pdev->dev,
			     sizeof(struct ion_heap *) * pdata->nr,
			     GFP_KERNEL);
	if (!heaps)
		return -ENOMEM;

	idev = ion_device_create(NULL);
	if (IS_ERR_OR_NULL(idev))
		return PTR_ERR(idev);

	/* create the heaps as specified in the board file */
	for (i = 0; i < num_heaps; i++) {
		struct ion_platform_heap *heap_data = &pdata->heaps[i];

		if (heap_data->type == ION_HEAP_TYPE_CARVEOUT) {
			if (vpr_size) {
				heap_data->base = vpr_start;
				heap_data->size = vpr_size;
			} else {
				continue;
			}
		}

		heaps[i] = ion_heap_create(heap_data);
		if (IS_ERR_OR_NULL(heaps[i])) {
			err = PTR_ERR(heaps[i]);
			goto err;
		}
		ion_device_add_heap(idev, heaps[i]);
	}
	platform_set_drvdata(pdev, idev);
	return 0;
err:
	for (i = 0; i < num_heaps; i++) {
		if (heaps[i])
			ion_heap_destroy(heaps[i]);
	}
	return err;
}

static int tegra_ion_remove(struct platform_device *pdev)
{
	struct ion_device *idev = platform_get_drvdata(pdev);
	int i;

	ion_device_destroy(idev);
	for (i = 0; i < num_heaps; i++)
		ion_heap_destroy(heaps[i]);
	return 0;
}

static struct platform_driver ion_driver = {
	.probe = tegra_ion_probe,
	.remove = tegra_ion_remove,
	.driver = {
		.name = "ion-tegra",
		.owner = THIS_MODULE,
		.of_match_table = ion_of_ids,
	},
};

module_platform_driver(ion_driver);

