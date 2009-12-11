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
/*
 * Copyright (c) 2009 NVIDIA Corporation.  All rights reserved.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/power_supply.h>
#include <linux/wakelock.h>

#include "nvcommon.h"
#include "nvos.h"
#include "nvodm_battery.h"

/* This defines the manufacturer name and model name length */
#define BATTERY_INFO_NAME_LEN 30

#define GET_CHARGER_STATUS 0

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
	NvU32	current;		/* Battery current */
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
		batt_dev->current = data.BatteryCurrent;
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
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		} else if (state & NVODM_BATTERY_STATUS_CHARGING) {
			batt_dev->present = NV_TRUE;
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
			/* TODO:Get Charger status here */
		} else
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
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
		val->intval = batt_dev->current;
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

static int tegra_battery_probe(struct platform_device *pdev)
{
	int i, rc;
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

	return 0;
}

static int tegra_battery_remove(struct platform_device *pdev)
{
	if (batt_dev) {
		if (batt_dev->hOdmBattDev) {
			NvOdmBatteryDeviceClose(batt_dev->hOdmBattDev);
			batt_dev->hOdmBattDev = NULL;
		}

		kfree(batt_dev);
		batt_dev = NULL;

	}
	return 0;
}

static struct platform_driver tegra_battery_driver = {
	.probe  = tegra_battery_probe,
	.remove = tegra_battery_remove,
	.driver = {
		.name   = "tegra_battery",
		.owner  = THIS_MODULE,
	},
};

static int __init tegra_battery_init(void)
{
	platform_driver_register(&tegra_battery_driver);
	return 0;
}

static void __exit tegra_battery_exit(void)
{
	platform_driver_unregister(&tegra_battery_driver);
}

module_init(tegra_battery_init);
module_exit(tegra_battery_exit);
MODULE_DESCRIPTION("TEGRA EC based Battery Driver");

