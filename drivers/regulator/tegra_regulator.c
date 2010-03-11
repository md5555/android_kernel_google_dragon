/*
 * tegra-regulator.c
 *
 * Regulator driver for setting the vbus charging current for NVIDIA Tegra SoCs
 *
 * Copyright (C) 2010 NVIDIA Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/err.h>
#include <linux/regulator/machine.h>
#include <linux/mutex.h>
#include "mach/nvrm_linux.h"
#include "nvassert.h"
#include "nvrm_pmu.h"
#include "nvrm_power.h"

/*
 * Tegra regulator data structure.
 */
struct tegra_regulator_data {
	struct regulator_init_data vbus_data; /* vbus data for current limit */
	struct regulator_consumer_supply vbus_supply; /* vbus supply info */
	int current_limit_uA; /* current limit in uA */
	struct mutex lock; /* mutex for gaurding current_limit_uA */
	struct work_struct work; /* work thread for setting the current limit */
};

/*
 * Work thread function for setting the charging current limit.
 *
 * This work thread is created to avoid the pre-emption from the ISR context.
 * If any driver calls the tegra_regulator_set_current_limit() api this will
 * schedule the work thread for setting the current limit.
 * NvRmPmuSetChargingCurrentLimit() uses I2C driver, that waits on semaphore
 * during the I2C transaction this will cause the pre-emption if called in ISR.
 */
static void tegra_regulator_set_current_limit_work(struct work_struct* work)
{
	struct tegra_regulator_data *reg_data =
				container_of(work, struct tegra_regulator_data, work);

	mutex_lock(&reg_data->lock);
	NvRmPmuSetChargingCurrentLimit(
		s_hRmGlobal,
		NvRmPmuChargingPath_UsbBus,
		(reg_data->current_limit_uA/1000),
		NvOdmUsbChargerType_SE0);
	mutex_unlock(&reg_data->lock);
}

/*
 * Stores the current limit and starts the work thread for setting the limit
 */
static int tegra_regulator_set_current_limit(struct regulator_dev * rdev,
	int min_uA, int max_uA)
{
	struct tegra_regulator_data *reg_data = rdev_get_drvdata(rdev);

	mutex_lock(&reg_data->lock);
	reg_data->current_limit_uA = max_uA;
	mutex_unlock(&reg_data->lock);
	schedule_work(&reg_data->work);

	return 0;
}

static int tegra_regulator_initialize(void *drv_data)
{
	struct tegra_regulator_data *reg_data =
				(struct tegra_regulator_data *)drv_data;

	mutex_init(&reg_data->lock);
	reg_data->current_limit_uA = 0;
	INIT_WORK(&reg_data->work, tegra_regulator_set_current_limit_work);

	return 0;
}

static struct regulator_ops tegra_regulator_ops = {
	.set_current_limit = tegra_regulator_set_current_limit,
};

static struct regulator_desc tegra_vbus_regulator = {
	.name = "vbus_draw",
	.id = 0,
	.ops = &tegra_regulator_ops,
	.type = REGULATOR_CURRENT,
	.owner = THIS_MODULE,
};

static int __devinit tegra_regulator_probe(struct platform_device *pdev)
{
	struct regulator_dev *rdev;
	struct tegra_regulator_data *reg_data;

	reg_data = kzalloc(sizeof(struct tegra_regulator_data),GFP_KERNEL);
	if (!reg_data)
		return -ENOMEM;

	reg_data->vbus_supply.dev = &pdev->dev;
	reg_data->vbus_supply.supply = "vbus_draw";
	reg_data->vbus_data.constraints.min_uA = 0;
	reg_data->vbus_data.constraints.max_uA = 1800000;
	reg_data->vbus_data.constraints.valid_ops_mask = REGULATOR_CHANGE_CURRENT;
	reg_data->vbus_data.num_consumer_supplies = 1;
	reg_data->vbus_data.consumer_supplies = &reg_data->vbus_supply;
	reg_data->vbus_data.regulator_init = tegra_regulator_initialize;
	pdev->dev.platform_data = &reg_data->vbus_data;

	rdev = regulator_register(&tegra_vbus_regulator, &pdev->dev, reg_data);
	if (IS_ERR(rdev)) {
		dev_err(&pdev->dev, "failed to register %s\n",
				tegra_vbus_regulator.name);
		return PTR_ERR(rdev);
	}

	return 0;
}

static int __devexit tegra_regulator_remove(struct platform_device *pdev)
{
	struct regulator_dev *rdev = platform_get_drvdata(pdev);
	struct tegra_regulator_data *reg_data = rdev_get_drvdata(rdev);

	regulator_unregister(rdev);
	kfree(reg_data);

	return 0;
}

static struct platform_driver tegra_regulator_driver = {
	.driver = {
		.name = "tegra_regulator",
	},
	.probe = tegra_regulator_probe,
	.remove = __devexit_p(tegra_regulator_remove),
};


static int __init tegra_regulator_init(void)
{
	return platform_driver_register(&tegra_regulator_driver);
}
module_init(tegra_regulator_init);

static void __exit tegra_regulator_exit(void)
{
	platform_driver_unregister(&tegra_regulator_driver);
}
module_exit(tegra_regulator_exit);

MODULE_DESCRIPTION("Tegra regulator driver");
MODULE_LICENSE("GPL");
