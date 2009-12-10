/*
 * drivers/usb/gadget/tegra_udc.c
 *
 * USB device controller clock and PHY programming for Tegra SoCs
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

#include <linux/kernel.h>
#include <linux/platform_device.h>

#include "nvddk_usbphy.h"
#include "mach/nvrm_linux.h"
#include "nvassert.h"

NvDdkUsbPhyHandle s_hUsbPhy;

int tegra_udc_clk_init(struct platform_device *pdev)
{
	NvError nverr;
	NvDdkUsbPhyIoctl_VBusStatusOutputArgs Status;

	nverr = NvDdkUsbPhyOpen(s_hRmGlobal, pdev->id, &s_hUsbPhy);
	if (nverr != NvSuccess)
		return -ENODEV;

	NV_ASSERT_SUCCESS(
		NvDdkUsbPhyIoctl(s_hUsbPhy,
			NvDdkUsbPhyIoctlType_VBusStatus,
			NULL, &Status));

	if (Status.VBusDetected) {
		NV_ASSERT_SUCCESS(NvDdkUsbPhyPowerUp(s_hUsbPhy, 0));
	} else {
		NV_ASSERT_SUCCESS(NvDdkUsbPhyPowerDown(s_hUsbPhy, 0));
	}
	printk("tegra_udc_clk_init called\n");

	return 0;
}

void tegra_udc_clk_finalize(struct platform_device *pdev)
{
	printk("tegra_udc_clk_finalize called\n");
}

void tegra_udc_clk_release(void)
{
	printk("tegra_udc_clk_release called\n");
}

void tegra_udc_clk_suspend(void)
{
	NV_ASSERT_SUCCESS(NvDdkUsbPhyPowerDown(s_hUsbPhy, 0));
}

void tegra_udc_clk_resume(void)
{
	NV_ASSERT_SUCCESS(NvDdkUsbPhyPowerUp(s_hUsbPhy, 0));
}
