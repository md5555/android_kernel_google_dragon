/*
 * drivers/power/nvec_battery.c
 *
 * Battery driver for batteries connected to NVIDIA NvEc-compliant embedded
 * controller
 *
 * Copyright (c) 2009, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/device.h>
#include <linux/debugfs.h>
#include <linux/power_supply.h>
#include <linux/wakelock.h>
#include <linux/tegra_devices.h>

#include "nvcommon.h"
#include "nvos.h"
#include "nvodm_battery.h"
#include "nvec_device.h"


/* This defines the manufacturer name and model name length */
#define BATTERY_INFO_NAME_LEN 30

#define GET_CHARGER_STATUS 1

#define NVBATTERY_POLLING_INTERVAL 30000 /* 30 Seconds */

static struct timer_list battery_poll_timer;

typedef enum
{
	NvCharger_Type_Battery = 0,
	NvCharger_Type_USB,
	NvCharger_Type_AC,
	NvCharger_Type_Num,
	NvCharger_Type_Force32 = 0x7FFFFFFF
} NvCharger_Type;

typedef enum
{
	NvCharge_Control_Charging_Disable = 0,
	NvCharge_Control_Charging_Enable,
	NvCharge_Control_Num,
	NvCharge_Control_Force32 = 0x7FFFFFFF
} NvCharge_Control;

static enum power_supply_property tegra_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_EMPTY,
	POWER_SUPPLY_PROP_TEMP,
//    POWER_SUPPLY_PROP_MODEL_NAME,
//    POWER_SUPPLY_PROP_MANUFACTURER,

};

static enum power_supply_property tegra_power_properties[] = {
	    POWER_SUPPLY_PROP_ONLINE,
};

static char *supply_list[] = {
        "battery",
};

static int tegra_power_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val);

static int tegra_battery_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val);

static struct power_supply tegra_power_supplies[] = {
	{
		.name = "battery",
		.type = POWER_SUPPLY_TYPE_BATTERY,
		.properties = tegra_battery_properties,
		.num_properties = ARRAY_SIZE(tegra_battery_properties),
		.get_property = tegra_battery_get_property,
	},
	{
		.name = "usb",
		.type = POWER_SUPPLY_TYPE_USB,
		.supplied_to = supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = tegra_power_properties,
		.num_properties = ARRAY_SIZE(tegra_power_properties),
		.get_property = tegra_power_get_property,
	},
	{
		.name = "ac",
		.type = POWER_SUPPLY_TYPE_MAINS,
		.supplied_to = supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = tegra_power_properties,
		.num_properties = ARRAY_SIZE(tegra_power_properties),
		.get_property = tegra_power_get_property,
	},
};

/* This is the Battery context structure */
struct tegra_battery_dev {
	NvOdmBatteryDeviceHandle hOdmBattDev;
	NvU32	batt_id;
	NvU32	voltage;		/* voltage */
	NvU32	temp;			/* Temperature */
	NvU32	current_ma;		/* Battery current */
	NvU32	current_avg;		/* average current */
	NvU32	charging_source;	/* 0: no cable, 1:usb, 2:AC */
	NvU32	charging_enabled;	/* 0: Disable, 1: Enable */
	NvU32	capacity;		/* full capacity of battery (mAh) */
	NvU32	capacity_crit;		/* critical capacity level */
	NvU32	capacity_remain;	/* remaining battery capacity */
	NvU32	percent_remain;		/* percentage of battery remaining */
	NvU32	lifetime;
	NvU32	consumed;
	NvBool	ac_status;
	NvBool	present;
};

static struct tegra_battery_dev *batt_dev;

static void tegra_get_battery_tech(int *val, NvOdmBatteryInstance inst)
{
	NvOdmBatteryChemistry chemistry = NvOdmBatteryChemistry_Num;

	NvOdmBatteryGetBatteryChemistry(batt_dev->hOdmBattDev,
		inst, &chemistry);

	switch(chemistry) {
	case NvOdmBatteryChemistry_NICD:
		*val = POWER_SUPPLY_TECHNOLOGY_NiCd;
		break;

	case NvOdmBatteryChemistry_NIMH:
		*val = POWER_SUPPLY_TECHNOLOGY_NiMH;
		break;

	case NvOdmBatteryChemistry_LION:
		*val = POWER_SUPPLY_TECHNOLOGY_LION;
		break;

	case NvOdmBatteryChemistry_LIPOLY:
		*val = POWER_SUPPLY_TECHNOLOGY_LIPO;
		break;

	case NvOdmBatteryChemistry_XINCAIR:
	case NvOdmBatteryChemistry_Alkaline:
	default:
		*val = POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
		break;
	}
}

static void tegra_battery_convert(NvOdmBatteryData *data)
{
	if (data->BatteryLifePercent == NVODM_BATTERY_DATA_UNKNOWN)
		data->BatteryLifePercent = 0;

	if (data->BatteryLifeTime == NVODM_BATTERY_DATA_UNKNOWN)
		data->BatteryLifeTime = 0;

	if (data->BatteryVoltage == NVODM_BATTERY_DATA_UNKNOWN)
		data->BatteryVoltage = 0;

	if (data->BatteryCurrent == NVODM_BATTERY_DATA_UNKNOWN)
		data->BatteryCurrent = 0;

	if (data->BatteryMahConsumed == NVODM_BATTERY_DATA_UNKNOWN)
		data->BatteryMahConsumed = 0;

	if (data->BatteryTemperature == NVODM_BATTERY_DATA_UNKNOWN)
		data->BatteryTemperature = 0;

	if (data->BatteryAverageCurrent == NVODM_BATTERY_DATA_UNKNOWN)
		data->BatteryAverageCurrent = 0;

	if (data->BatteryLastChargeFullCapacity == NVODM_BATTERY_DATA_UNKNOWN)
		data->BatteryLastChargeFullCapacity = 0;

	if (data->BatteryCriticalCapacity == NVODM_BATTERY_DATA_UNKNOWN)
		data->BatteryCriticalCapacity = 0;

	if (data->BatteryRemainingCapacity == NVODM_BATTERY_DATA_UNKNOWN)
		data->BatteryRemainingCapacity = 0;
}

static NvBool tegra_battery_data(NvOdmBatteryInstance NvBatteryInst)
{
	NvOdmBatteryData data = {0};

	if (!NvOdmBatteryGetBatteryData(batt_dev->hOdmBattDev,
		NvBatteryInst, &data))
		return NV_FALSE;

	tegra_battery_convert(&data);

	if (NvBatteryInst == NvOdmBatteryInst_Main) {
		batt_dev->voltage = data.BatteryVoltage;
		batt_dev->current_ma = data.BatteryCurrent;
		batt_dev->current_avg  = data.BatteryAverageCurrent;
		batt_dev->temp = data.BatteryTemperature;
		batt_dev->percent_remain = data.BatteryLifePercent;
		batt_dev->lifetime = data.BatteryLifeTime;
		batt_dev->consumed = data.BatteryMahConsumed;
		batt_dev->capacity = data.BatteryLastChargeFullCapacity;
		batt_dev->capacity_crit = data.BatteryCriticalCapacity;
		batt_dev->capacity_remain = data.BatteryRemainingCapacity;
	}

	return NV_TRUE;
}

static int tegra_power_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	NvCharger_Type charger = 0;
	NvOdmBatteryAcLineStatus status = NvOdmBatteryAcLine_Offline;

	/* Need to find out the way which tell the charger source */

	switch (psp) {

	case POWER_SUPPLY_PROP_ONLINE:
		if (!NvOdmBatteryGetAcLineStatus(batt_dev->hOdmBattDev,
			&status))
			return -ENODEV;

		if (status == NvOdmBatteryAcLine_Offline) {
			batt_dev->ac_status = NV_FALSE;
		}
		else if (status == NvOdmBatteryAcLine_Online) {
			batt_dev->ac_status = NV_TRUE;
			charger = NvCharger_Type_AC;
		}
		else
			batt_dev->ac_status = NV_FALSE;

		if (psy->type == POWER_SUPPLY_TYPE_MAINS)
			val->intval = (charger == NvCharger_Type_AC);
		else if (psy->type == POWER_SUPPLY_TYPE_USB)
			val->intval = (charger == NvCharger_Type_USB);
		else
			val->intval = 0;

		break;

	default:
		return -EINVAL;
	}
	return 0;
}

static int tegra_battery_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	NvU8 name[BATTERY_INFO_NAME_LEN] = {0};
	int technology = 0;
	NvU8 state = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
			/* TODO:Get Charger status here */
#if GET_CHARGER_STATUS
		if (!NvOdmBatteryGetBatteryStatus(batt_dev->hOdmBattDev,
			NvOdmBatteryInst_Main, &state))
			return -ENODEV;

		if (state == NVODM_BATTERY_STATUS_UNKNOWN) {
			batt_dev->present = NV_FALSE;
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		} else if (state == NVODM_BATTERY_STATUS_NO_BATTERY) {
			batt_dev->present = NV_FALSE;
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		} else if (state & NVODM_BATTERY_STATUS_CHARGING) {
			batt_dev->present = NV_TRUE;
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
			/* TODO:Get Charger status here */
		} else if (state & NVODM_BATTERY_STATUS_HIGH) {
			batt_dev->present = NV_TRUE;
			val->intval = POWER_SUPPLY_STATUS_FULL;
			/* TODO:Get Charger status here */
		} else
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
#endif
		/* Getting the battery info once here so for the other property
		 * requests there will not be lot of ec req */
		if (!tegra_battery_data(NvOdmBatteryInst_Main)) {
			; /* FIXME: return error? */
		}
		break;

	case POWER_SUPPLY_PROP_HEALTH:
		if (batt_dev->present)
			val->intval = POWER_SUPPLY_HEALTH_GOOD;
		else
			val->intval = POWER_SUPPLY_HEALTH_UNKNOWN;

		break;

	case POWER_SUPPLY_PROP_PRESENT:
		if (!NvOdmBatteryGetBatteryStatus(batt_dev->hOdmBattDev,
			NvOdmBatteryInst_Main, &state))
			return -EINVAL;

		if (state == NVODM_BATTERY_STATUS_UNKNOWN) {
			batt_dev->present = NV_FALSE;
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		} else {
			if (state == NVODM_BATTERY_STATUS_NO_BATTERY) {
				 batt_dev->present = NV_FALSE;
				 val->intval = NV_FALSE;
			}
			if (state & (NVODM_BATTERY_STATUS_HIGH |
				NVODM_BATTERY_STATUS_LOW |
				NVODM_BATTERY_STATUS_CRITICAL |
				NVODM_BATTERY_STATUS_CHARGING)) {
				batt_dev->present = NV_TRUE;
				val->intval = NV_TRUE;
			}
		}
		break;

	case POWER_SUPPLY_PROP_TECHNOLOGY:
		tegra_get_battery_tech(&technology, NvOdmBatteryInst_Main);
		val->intval = technology;
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = batt_dev->percent_remain;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = batt_dev->voltage*1000;
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = batt_dev->current_ma;
		break;

	case POWER_SUPPLY_PROP_CURRENT_AVG:
		val->intval = batt_dev->current_avg;
		break;

	case POWER_SUPPLY_PROP_CHARGE_NOW:
		val->intval = batt_dev->capacity_remain;
		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = batt_dev->capacity;
		break;

	case POWER_SUPPLY_PROP_CHARGE_EMPTY:
		val->intval = batt_dev->capacity_crit;
		break;

	case POWER_SUPPLY_PROP_TEMP:
		/* returned value is degrees C * 10 */
		val->intval = batt_dev->temp/10;
		break;

	case POWER_SUPPLY_PROP_MODEL_NAME:
		if (!NvOdmBatteryGetModel(batt_dev->hOdmBattDev,
			NvOdmBatteryInst_Main, name))
			return -EINVAL;

		strncpy((char *)val->strval, name, strlen(name));
		break;

	case POWER_SUPPLY_PROP_MANUFACTURER:
		if (!NvOdmBatteryGetManufacturer(batt_dev->hOdmBattDev,
			NvOdmBatteryInst_Main, name))
			return -EINVAL;

		strncpy((char *)val->strval, name, strlen(name));
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static void tegra_battery_poll_timer_func(unsigned long unused)
{
	power_supply_changed(&tegra_power_supplies[NvCharger_Type_Battery]);
	power_supply_changed(&tegra_power_supplies[NvCharger_Type_USB]);
	power_supply_changed(&tegra_power_supplies[NvCharger_Type_AC]);

	mod_timer(&battery_poll_timer,
		jiffies + msecs_to_jiffies(NVBATTERY_POLLING_INTERVAL));
}

static int nvec_battery_probe(struct nvec_device *pdev)
{
	int i;
	NvBool result = NV_FALSE;

	batt_dev = kzalloc(sizeof(struct tegra_battery_dev), GFP_KERNEL);
	if (!batt_dev) {
		return -ENOMEM;
	}

	result = NvOdmBatteryDeviceOpen(&(batt_dev->hOdmBattDev), NULL);
	if (!result || !batt_dev->hOdmBattDev) {
		kfree(batt_dev);
		batt_dev = NULL;
		return -1;
	}

	for (i = 0; i < ARRAY_SIZE(tegra_power_supplies); i++) {
		if (power_supply_register(&pdev->dev, &tegra_power_supplies[i]))
			printk(KERN_ERR "Failed to register power supply\n");
	}

	setup_timer(&battery_poll_timer, tegra_battery_poll_timer_func, 0);
	mod_timer(&battery_poll_timer,
		jiffies + msecs_to_jiffies(NVBATTERY_POLLING_INTERVAL));

	return 0;
}

static int nvec_battery_remove(struct nvec_device *pdev)
{
	unsigned int i = 0;

	if (batt_dev) {
		if (batt_dev->hOdmBattDev) {
			NvOdmBatteryDeviceClose(batt_dev->hOdmBattDev);
			batt_dev->hOdmBattDev = NULL;

			for (i = 0; i < ARRAY_SIZE(tegra_power_supplies); i++) {
				power_supply_unregister(&tegra_power_supplies[i]);
			}
		}

		kfree(batt_dev);
		batt_dev = NULL;

	}

	del_timer_sync(&battery_poll_timer);

	return 0;
}

static int nvec_battery_suspend(struct nvec_device *dev,
	pm_message_t state)
{
	/* Kill the Battery Polling timer */
	del_timer_sync(&battery_poll_timer);
	return 0;
}

static int nvec_battery_resume(struct nvec_device *dev)
{
	/*Create Battery Polling timer */
	setup_timer(&battery_poll_timer, tegra_battery_poll_timer_func, 0);
	mod_timer(&battery_poll_timer,
		jiffies + msecs_to_jiffies(NVBATTERY_POLLING_INTERVAL));
	return 0;
}

static struct nvec_driver nvec_battery_driver = {
	.name		= "nvec_battery",
	.probe		= nvec_battery_probe,
	.remove		= nvec_battery_remove,
	.suspend	= nvec_battery_suspend,
	.resume		= nvec_battery_resume,
};

static struct nvec_device nvec_battery_device = {
	.name	= "nvec_battery",
	.driver	= &nvec_battery_driver,
};

static int __init nvec_battery_init(void)
{
	int err;

	err = nvec_register_driver(&nvec_battery_driver);
	if (err)
	{
		pr_err("**nvec_battery_init: nvec_register_driver: fail\n");
		return err;
	}

	err = nvec_register_device(&nvec_battery_device);
	if (err)
	{
		pr_err("**nvec_battery_init: nvec_device_add: fail\n");
		nvec_unregister_driver(&nvec_battery_driver);
		return err;
	}

	return 0;
}

static void __exit nvec_battery_exit(void)
{
	nvec_unregister_device(&nvec_battery_device);
	nvec_unregister_driver(&nvec_battery_driver);
}

module_init(nvec_battery_init);
module_exit(nvec_battery_exit);
MODULE_DESCRIPTION("TEGRA EC based Battery Driver");

