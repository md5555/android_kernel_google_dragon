/*
 * include/linux/tegra_devices.h
 *
 * Definitions for platform devices and related flags NVIDIA Tegra SoCs
 *
 * Copyright (C) 2009 NVIDIA Corporation
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

#ifndef _LINUX_DEVICE_TEGRA_H
#define _LINUX_DEVICE_TEGRA_H

#include "nvcommon.h"
#include "nvrm_gpio.h"
#include "nvddk_usbphy.h"
#include "nvodm_query.h"

struct tegra_hcd_platform_data {
	NvU32			instance;
	NvRmGpioPinHandle	hGpioIDpin;
	const NvOdmUsbProperty	*pUsbProperty;
	NvU32			powerClientId;
	NvU32			vBusPowerRail;
	/* USB PHY power rail. Tegra has integrated UTMI (USB transciver
	 * macrocell interface) PHY on USB controllers 0 and 2. These 2 PHYs
	 * have its own rails.
	 */
	NvU32			phyPowerRail; 
	NvDdkUsbPhyHandle	hUsbPhy;
};

#endif
