/*
 * Power off driver for Maxim MAX77620 device.
 *
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author: Chaitanya Bandi <bandik@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any kind,
 * whether express or implied; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 */

#include <linux/errno.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/mfd/max77620.h>

#define MAX77620_PM_RTC_AUTO_CLEAR_MASK	(1 << 1)
#define MAX77620_PM_RTC_WAKE_MASK			(1 << 3)
#define MAX77620_PM_RTC_RB_UPDATE_MASK		(1 << 4)
#define MAX77620_PM_RTC_INT1_MASK			(1 << 1)

struct max77620_poweroff {
	struct device *dev;
	struct max77620_chip *max77620;
	bool use_power_off;
	bool use_power_reset;
	bool need_rtc_power_on;
};

static struct max77620_poweroff *max77620_pm_poweroff;

#if 0
static void max77620_auto_power_on(struct max77620_poweroff *max77620_poweroff)
{
	int ret;
	u8 reg_val;
	u8 val =  MAX77620_PM_RTC_AUTO_CLEAR_MASK | MAX77620_PM_RTC_WAKE_MASK |
			MAX77620_PM_RTC_RB_UPDATE_MASK;

	dev_info(max77620_poweroff->dev, "Resetting through Max77620 RTC\n");

	ret = max77620_reg_write(max77620_poweroff->max77620->dev,
						MAX77620_RTC_SLAVE,
						MAX77620_REG_RTCUPDATE0, val);
	if (ret < 0) {
		dev_err(max77620_poweroff->dev,
			"MAX77620_REG_RTCUPDATE0 write failed\n");
		goto scrub;
	}

	/* Must wait 16ms for buffer update */
	usleep_range(16000, 16000);

	ret = max77620_reg_read(max77620_poweroff->max77620->dev,
			MAX77620_RTC_SLAVE,
			MAX77620_REG_RTCINTM, &reg_val);
	if (ret < 0) {
		dev_err(max77620_poweroff->dev,
			"Failed to read MAX77620_REG_RTCINTM: %d\n", ret);
		goto scrub;
	}

	reg_val = (reg_val & ~MAX77620_PM_RTC_INT1_MASK);
	ret = max77620_reg_writes(max77620_poweroff->max77620->dev,
			MAX77620_RTC_SLAVE, MAX77620_REG_RTCINTM, 1, &reg_val);
	if (ret < 0) {
		dev_err(max77620_poweroff->dev,
			"Failed to write MAX77620_REG_RTCINTM: %d\n", ret);
		goto scrub;
	}

	val =  MAX77620_PM_RTC_AUTO_CLEAR_MASK | MAX77620_PM_RTC_WAKE_MASK |
			MAX77620_PM_RTC_RB_UPDATE_MASK;
	ret = max77620_reg_write(max77620_poweroff->max77620->dev,
						MAX77620_RTC_SLAVE,
						MAX77620_REG_RTCUPDATE0, val);
	if (ret < 0) {
		dev_err(max77620_poweroff->dev,
			"MAX77620_REG_RTCUPDATE0 write failed\n");
		goto scrub;
	}

	/* Must wait 16ms for buffer update */
	usleep_range(16000, 16000);

	ret = max77620_reg_update(max77620_poweroff->max77620->dev,
		MAX77620_PWR_SLAVE, MAX77620_REG_ONOFFCNFG2,
		MAX77620_ONOFFCNFG2_SFT_RST_WK, 0);

	ret = max77620_reg_update(max77620_poweroff->max77620->dev,
		MAX77620_PWR_SLAVE, MAX77620_REG_ONOFFCNFG1,
		MAX77620_ONOFFCNFG1_SFT_RST, MAX77620_ONOFFCNFG1_SFT_RST);
	if (ret < 0)
		dev_err(max77620_poweroff->dev,
			"REG_ONOFFCNFG1 update failed, %d\n", ret);

	 while (1)
		dev_err(max77620_poweroff->dev, "Device should not be here\n");
scrub:
	return;
}
#endif

static void max77620_pm_power_off(void)
{
	int ret;

	if (!max77620_pm_poweroff) {
		pr_err("MAX77620 poweroff is not initialized\n");
		return;
	}

	dev_info(max77620_pm_poweroff->dev, "Powering off system\n");

#if 0
	if (max77620_pm_poweroff->need_rtc_power_on)
		max77620_auto_power_on(max77620_pm_poweroff);
#endif

	ret = max77620_reg_update(max77620_pm_poweroff->max77620->dev,
		MAX77620_PWR_SLAVE, MAX77620_REG_ONOFFCNFG1,
		MAX77620_ONOFFCNFG1_PWR_OFF, MAX77620_ONOFFCNFG1_PWR_OFF);
	if (ret < 0)
		dev_err(max77620_pm_poweroff->dev,
			"REG_ONOFFCNFG1 update failed, %d\n", ret);
}

#if 0
static void max77620_pm_power_reset(void *drv_data)
{
	struct max77620_poweroff *max77620_poweroff = drv_data;
	int ret;

	dev_info(max77620_poweroff->dev, "Power resetting system\n");

	ret = max77620_reg_update(max77620_poweroff->max77620->dev,
		MAX77620_PWR_SLAVE, MAX77620_REG_ONOFFCNFG2,
		MAX77620_ONOFFCNFG2_SFT_RST_WK, MAX77620_ONOFFCNFG2_SFT_RST_WK);

	ret = max77620_reg_update(max77620_poweroff->max77620->dev,
		MAX77620_PWR_SLAVE, MAX77620_REG_ONOFFCNFG1,
		MAX77620_ONOFFCNFG1_SFT_RST, MAX77620_ONOFFCNFG1_SFT_RST);
	if (ret < 0)
		dev_err(max77620_poweroff->dev,
			"REG_ONOFFCNFG1 update failed, %d\n", ret);
}
#endif

static int max77620_poweroff_probe(struct platform_device *pdev)
{
	struct max77620_poweroff *max77620_poweroff;
	struct device_node *np = pdev->dev.parent->of_node;
	struct max77620_chip *max77620 = dev_get_drvdata(pdev->dev.parent);
	bool use_power_off = false;
	bool use_power_reset = false;
	u8 poweroff_event_recorder;
	int ret;

	if (np) {
		bool system_pc;

		use_power_off = of_property_read_bool(np,
				"maxim,system-pmic-power-off");
		use_power_reset = of_property_read_bool(np,
				"maxim,system-pmic-power-reset");
		system_pc = of_property_read_bool(np,
				"maxim,system-power-controller");
		if (system_pc) {
			use_power_off = true;
			use_power_reset = true;
		}
	}

	if (!use_power_off && !use_power_reset) {
		dev_warn(&pdev->dev,
			"power off and reset functionality not selected\n");
		return 0;
	}

	max77620_poweroff = devm_kzalloc(&pdev->dev, sizeof(*max77620_poweroff),
				GFP_KERNEL);
	if (!max77620_poweroff)
		return -ENOMEM;

	max77620_poweroff->max77620 = max77620;
	max77620_poweroff->dev = &pdev->dev;
	max77620_poweroff->use_power_off = use_power_off;
	max77620_poweroff->use_power_reset = use_power_reset;

#if 0
	config.allow_power_off = use_power_off;
	config.allow_power_reset = use_power_reset;
#endif

	max77620_pm_poweroff = max77620_poweroff;
	if (!pm_power_off)
		pm_power_off = max77620_pm_power_off;

	ret = max77620_reg_read(max77620_poweroff->max77620->dev,
		MAX77620_PWR_SLAVE, MAX77620_REG_NVERC,
		&poweroff_event_recorder);
	if (ret < 0) {
		dev_err(max77620_poweroff->dev,
			"REG_NVERC read failed, %d\n", ret);
		return ret;
	} else
		dev_info(&pdev->dev, "Event recorder REG_NVERC : 0x%x\n",
				poweroff_event_recorder);

	return 0;
}

static int max77620_poweroff_remove(struct platform_device *pdev)
{
	if (pm_power_off == max77620_pm_power_off)
		pm_power_off = NULL;
	max77620_pm_poweroff = NULL;

	return 0;
}

static struct platform_driver max77620_poweroff_driver = {
	.driver = {
		.name = "max77620-power-off",
		.owner = THIS_MODULE,
	},
	.probe = max77620_poweroff_probe,
	.remove = max77620_poweroff_remove,
};

module_platform_driver(max77620_poweroff_driver);

MODULE_DESCRIPTION("Power off driver for MAX77620 PMIC Device");
MODULE_ALIAS("platform:max77620-power-off");
MODULE_AUTHOR("Chaitanya Bandi <bandik@nvidia.com>");
MODULE_LICENSE("GPL v2");
