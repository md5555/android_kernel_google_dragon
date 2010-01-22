/*
 * arch/arm/mach-tegra/tegra_rfkill_odm.c
 *
 * RF kill device using NVIDIA Tegra ODM kit
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

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/rfkill.h>

#include "mach/nvrm_linux.h"
#include "nvodm_query.h"
#include "nvrm_gpio.h"
#include "nvodm_services.h"
#include "nvodm_query_discovery.h"

#define DRIVER_NAME    "tegra_rfkill"
#define DRIVER_DESC    "Nvidia Tegra rfkill"

static NvRmGpioHandle hGpio = NULL;
static NvRmGpioPinHandle  hBlueToothResetPin = 0;
static NvU32 blueToothPowerRailId = 0xff;
static struct rfkill *bt = NULL;

void rfkill_switch_all(enum rfkill_type type, enum rfkill_state state);

static int bluetooth_set_power(void *data, enum rfkill_state state)
{
	NvOdmServicesPmuHandle hPmu;
	NvOdmServicesPmuVddRailCapabilities vddrailcap;
	NvU32 settletime = 0;
	NvU32 GpioLevel;

	if (blueToothPowerRailId == 0xff || !hBlueToothResetPin || !hGpio)
		return -ENXIO;

	hPmu = NvOdmServicesPmuOpen();
	if (!hPmu)
		return -ENXIO;

	switch (state) {
	case RFKILL_STATE_UNBLOCKED:
		NvOdmServicesPmuGetCapabilities(hPmu, blueToothPowerRailId,
			&vddrailcap);
		NvOdmServicesPmuSetVoltage(hPmu, blueToothPowerRailId,
			vddrailcap.requestMilliVolts, &settletime);
		if (settletime)
			NvOdmOsWaitUS(settletime);

		/* Pulse a reset */
		GpioLevel = 0;
		NvRmGpioWritePins(hGpio, &hBlueToothResetPin, &GpioLevel, 1);

		/* Configure as output */
		NvRmGpioConfigPins(hGpio, &hBlueToothResetPin, 1,
			NvRmGpioPinMode_Output);

		/* Give 5 milli seconds for the reset pulse */
		NvOdmOsSleepMS(5);

		GpioLevel = 1;
		NvRmGpioWritePins(hGpio, &hBlueToothResetPin, &GpioLevel, 1);

		printk(KERN_INFO "Bluetooth power ON\n");
		break;

	case RFKILL_STATE_SOFT_BLOCKED:
		/* Disable power */
		GpioLevel = 0;
		NvRmGpioWritePins(hGpio, &hBlueToothResetPin, &GpioLevel, 1);

		/* Configure as output */
		NvRmGpioConfigPins(hGpio, &hBlueToothResetPin, 1,
			NvRmGpioPinMode_Output);

		NvOdmServicesPmuSetVoltage( hPmu, blueToothPowerRailId,
			NVODM_VOLTAGE_OFF, &settletime);
		if (settletime)
			NvOdmOsWaitUS(settletime);

		printk(KERN_INFO "Bluetooth power OFF\n");
		break;

	default:
		printk(KERN_ERR "Bad bluetooth rfkill state %d\n", state);
	}
	NvOdmServicesPmuClose(hPmu);

	return 0;
}

static int __init tegra_rfkill_probe(struct platform_device *pdev)
{
	int rc;
	const NvOdmPeripheralConnectivity *conn = NULL;
	NvU32 port = 0xffff, pin = 0xffff;
	NvError err;
	NvU32 i;
	NvU64 bluetooth = NV_ODM_GUID('b','l','u','t','o','o','t','h');

	/* conn will be null if bluetooth is not present. */
	conn =
		NvOdmPeripheralGetGuid(bluetooth);
	if (!conn)
		return -ENXIO;

	for (i = 0; i < conn->NumAddress; i++) {
		if (conn->AddressList[i].Interface == NvOdmIoModule_Gpio) {
			port  = conn->AddressList[i].Instance;
			pin = conn->AddressList[i].Address;
		}
		if (conn->AddressList[i].Interface == NvOdmIoModule_Vdd) {
			blueToothPowerRailId = conn->AddressList[i].Address;
		}
	}

	if (port == 0xffff || pin == 0xffff || blueToothPowerRailId == 0xff) {
		printk(KERN_ERR "bluetooth information is invalid\n");
		return -ENXIO;
	}

	err = NvRmGpioOpen(s_hRmGlobal, &hGpio);
	if (err)  {
		printk(KERN_ERR "NvRmGpioOpen failed\n");
		return -ENXIO;
	}
	err = NvRmGpioAcquirePinHandle(hGpio, port, pin, &hBlueToothResetPin);
	if (err) {
		printk(KERN_ERR "NvRmGpioAcquirePinHandle failed\n");
		NvRmGpioClose(hGpio);
		return -ENXIO;
	}

	rfkill_switch_all(RFKILL_TYPE_BLUETOOTH, RFKILL_STATE_SOFT_BLOCKED);
	bluetooth_set_power(NULL, RFKILL_STATE_SOFT_BLOCKED);

	bt = rfkill_allocate(&pdev->dev, RFKILL_TYPE_BLUETOOTH);
	if (!bt) {
		rc = -ENOMEM;
		goto fail;
	}

	bt->name = "unknown"; /* FIXME how do we get this name? */
	bt->state = RFKILL_STATE_SOFT_BLOCKED;
	bt->user_claim = 0;
	bt->user_claim_unsupported = 1;
	bt->data = NULL;
	bt->toggle_radio = bluetooth_set_power;

	rc = rfkill_register(bt);
	if (rc)
		goto fail;

	return rc;

fail:
	if (hBlueToothResetPin)
		NvRmGpioReleasePinHandles(hGpio, &hBlueToothResetPin, 1);

	if (hGpio)
		NvRmGpioClose(hGpio);

	if (bt)
		rfkill_free(bt);
	return rc;
}

static int __init tegra_rfkill_remove(struct platform_device *pdev)
{
	rfkill_switch_all(RFKILL_TYPE_BLUETOOTH, RFKILL_STATE_SOFT_BLOCKED);
	bluetooth_set_power(NULL, RFKILL_STATE_SOFT_BLOCKED);

	if (hBlueToothResetPin)
		NvRmGpioReleasePinHandles(hGpio, &hBlueToothResetPin, 1);

	if (hGpio)
		NvRmGpioClose(hGpio);

	if (bt) {
		rfkill_unregister(bt);
		rfkill_free(bt);
	}

	return 0;
}

static struct platform_driver tegra_rfkill_driver = {
	.probe	= tegra_rfkill_probe,
	.remove	= tegra_rfkill_remove,
	.driver	= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init tegra_rfkill_init(void)
{
	return platform_driver_register(&tegra_rfkill_driver);
}

module_init(tegra_rfkill_init);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
