/*
 * ehci-tegra.c
 *
 * EHCI-compliant USB host controller driver for NVIDIA Tegra SoCs
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

#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/tegra_devices.h>

#include "mach/nvrm_linux.h"
#include "nvodm_query.h"
#include "nvodm_query_gpio.h"
#include "nvodm_query_discovery.h"
#include "nvrm_pmu.h"
#include "nvrm_analog.h"
#include "nvassert.h"
#include "nvrm_interrupt.h"
#include "nvrm_power.h"
#include "nvodm_pmu.h"
#include "nvrm_hardware_access.h"
#include "nvddk_usbphy.h"

/* FIXME: Power Management is un-ported so temporarily disable it */
#undef CONFIG_PM

#define TEGRA_USB_ID_INT_ENABLE		(1 << 24)
#define TEGRA_USB_ID_INT_STATUS		(1 << 16)
#define TEGRA_USB_ID_PIN_STATUS		(1 << 8)
#define TEGRA_USB_OTG_REG_OFFSET	(0x1a4)

static irqreturn_t tegra_ehci_irq (struct usb_hcd *hcd)
{
	struct ehci_hcd *ehci = hcd_to_ehci (hcd);
	struct tegra_hcd_platform_data *pdata;
	u32 status;

	pdata = hcd->self.controller->platform_data;

	spin_lock (&ehci->lock);

	if (pdata->pUsbProperty->IdPinDetectionType ==
		NvOdmUsbIdPinType_CableId) {
		/* read otgsc register for ID pin status change */
		status = readl(hcd->regs + TEGRA_USB_OTG_REG_OFFSET);
		writel(status, (hcd->regs + TEGRA_USB_OTG_REG_OFFSET));

		/* Check if there is any ID pin interrupt */
		if (status & TEGRA_USB_ID_INT_STATUS) {
			/* Check pin status and enable/disable the power */
			if (status & TEGRA_USB_ID_PIN_STATUS) {
				NvDdkUsbPhyPowerDown(pdata->hUsbPhy, 0);
			} else {
				NvDdkUsbPhyPowerUp(pdata->hUsbPhy, 0);
			}
		}
	}

	spin_unlock (&ehci->lock);

	return ehci_irq(hcd);
}


static int tegra_ehci_reinit(struct usb_hcd *hcd)
{
	struct tegra_hcd_platform_data *pdata;
	NvError e;

	pdata = hcd->self.controller->platform_data;

	NV_CHECK_ERROR_CLEANUP(NvDdkUsbPhyOpen(s_hRmGlobal,
		pdata->instance, &pdata->hUsbPhy));

	return 0;

fail:
	printk( KERN_INFO " tegra_ehci_reinit failed\n");
	return -EINVAL;
}

static int tegra_ehci_setup(struct usb_hcd *hcd)
{
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	int retval;

	/* EHCI registers start at offset 0x100 */
	ehci->caps = hcd->regs + 0x100;
	ehci->regs = hcd->regs + 0x100 +
		HC_LENGTH(readl(&ehci->caps->hc_capbase));

	dbg_hcs_params(ehci, "reset");
	dbg_hcc_params(ehci, "reset");

	/* cache this readonly data; minimize chip reads */
	ehci->hcs_params = readl(&ehci->caps->hcs_params);

	retval = ehci_halt(ehci);
	if (retval) {
		return retval;
	}

	/* data structure init */
	retval = ehci_init(hcd);
	if (retval) {
		return retval;
	}

	hcd->has_tt = 1;
	ehci->sbrn = 0x20;

	ehci_reset(ehci);

	/* Resetting the controller has the side effect of resetting the PHY.
	 * So, never reset the controller after the calling
	 * tegra_ehci_reinit API. */
	ehci->controller_resets_phy = 1;

	ehci_port_power(ehci, 0);
	return retval;
}

static const struct hc_driver tegra_ehci_hc_driver = {
	.description		= hcd_name,
	.product_desc		= "Tegra Ehci host controller",
	.hcd_priv_size		= sizeof(struct ehci_hcd),

	.flags			= HCD_USB2,
	/* lifecycle management */
	.reset			= tegra_ehci_setup,
	.irq			= tegra_ehci_irq,

	.start			= ehci_run,
	.stop			= ehci_stop,
	.shutdown		= ehci_shutdown,
	.urb_enqueue		= ehci_urb_enqueue,
	.urb_dequeue		= ehci_urb_dequeue,
	.endpoint_disable	= ehci_endpoint_disable,
	.get_frame_number	= ehci_get_frame,
	.hub_status_data	= ehci_hub_status_data,
	.hub_control		= ehci_hub_control,
	.bus_suspend		= ehci_bus_suspend,
	.bus_resume		= ehci_bus_resume,
	.relinquish_port	= ehci_relinquish_port,
	.port_handed_over	= ehci_port_handed_over,
};


static int tegra_ehci_probe(struct platform_device *pdev)
{
	int				instance = pdev->id;
	NvRmPhysAddr			addr;
	NvU32				size;
	struct tegra_hcd_platform_data	*pdata;
	struct usb_hcd			*hcd;
	int				e = 0;
	int				irq;
	unsigned int			temp;
	void				*vaddr;
	static u64			dummy_mask = DMA_32BIT_MASK;

	pdata = (struct tegra_hcd_platform_data *)pdev->dev.platform_data;
	if (!pdata) {
		dev_err(&pdev->dev, "Cannot run without platform data\n");
	}

	/* FIXME These are needed to enable DMA mode! Don't know why? */
	pdev->dev.coherent_dma_mask = ~0;
	pdev->dev.dma_mask = &dummy_mask;

	hcd = usb_create_hcd(&tegra_ehci_hc_driver, &pdev->dev,
		dev_name(&pdev->dev));
	if (!hcd) {
		e = -ENOMEM;
		goto fail;
	}
	pdata->instance = instance;

	pdata->pUsbProperty = NvOdmQueryGetUsbProperty(NvOdmIoModule_Usb, 
		instance);

	if (pdata->pUsbProperty->IdPinDetectionType == NvOdmUsbIdPinType_Gpio
		&& (instance == 0 || instance == 1)) {

		const NvOdmGpioPinInfo *pGpioPinInfo;
		NvU32 GpioPinCount = 0;

		pGpioPinInfo = NvOdmQueryGpioPinMap(NvOdmGpioPinGroup_Usb, 
			instance, &GpioPinCount);
		/* If the ODM says ID type is GPIO, they better provide the
                 * GPIO port and pin for that */
		BUG_ON(GpioPinCount == 0);

		NV_ASSERT_SUCCESS(NvRmGpioAcquirePinHandle(s_hGpioGlobal, 
			pGpioPinInfo[NvOdmGpioPin_UsbCableId].Port, 
			pGpioPinInfo[NvOdmGpioPin_UsbCableId].Pin, 
			&pdata->hGpioIDpin));

		// Configure Cable ID Pin as Input Pin 
		NV_ASSERT_SUCCESS(NvRmGpioConfigPins(s_hGpioGlobal, 
			&pdata->hGpioIDpin, 1, NvRmGpioPinMode_InputData));
	} else {

	}

	/* Init the tegra USB phy */
	e = tegra_ehci_reinit(hcd);
	if (e) {
		printk("%s: failed with error (0x%x)\n", __func__, e);
		goto fail;
	}

	NvRmModuleGetBaseAddress(s_hRmGlobal, 
		NVRM_MODULE_ID(NvRmModuleID_Usb2Otg, instance), &addr, &size);
	if (addr == 0x0 || size == 0) {
		e = -ENODEV;
		goto fail;
	}
	NvRmPhysicalMemMap(addr, size, NVOS_MEM_READ_WRITE, 
		 NvOsMemAttribute_Uncached, (void **)&vaddr);
	if (vaddr == NULL) {
		e = -ENOMEM;
		goto fail;
	}
	hcd->rsrc_start = addr;
	hcd->rsrc_len = size;
	hcd->regs = vaddr;

	/* Set to Host mode by setting bit 0-1 of USB device mode register */
	temp = readl(hcd->regs + 0x1a8);
	writel((temp | 0x3), (hcd->regs + 0x1a8));

	irq = NvRmGetIrqForLogicalInterrupt(s_hRmGlobal, 
		NVRM_MODULE_ID(NvRmModuleID_Usb2Otg, instance), 0);
	if (irq == 0xffff)
		goto fail;
	set_irq_flags(irq, IRQF_VALID);

	e  = usb_add_hcd(hcd, irq, IRQF_DISABLED | IRQF_SHARED);
	if (e != 0)
		goto fail;
	platform_set_drvdata(pdev, hcd);

	if (pdata->pUsbProperty->IdPinDetectionType ==
		NvOdmUsbIdPinType_CableId) {
		/* enable the cable ID interrupt */
		temp = readl(hcd->regs + TEGRA_USB_OTG_REG_OFFSET);
		writel((temp | TEGRA_USB_ID_INT_ENABLE),
			(hcd->regs + TEGRA_USB_OTG_REG_OFFSET));

		/* Check if we detect any device connected */
		if (temp & TEGRA_USB_ID_PIN_STATUS) {
			NvDdkUsbPhyPowerDown(pdata->hUsbPhy, 0);
		} else {
			NvDdkUsbPhyPowerUp(pdata->hUsbPhy, 0);
		}
	}

	return e;
fail:
	return e;
}

static int tegra_ehci_remove(struct platform_device *pdev)
{
	struct tegra_hcd_platform_data *pdata = pdev->dev.platform_data;
	struct usb_hcd *hcd = platform_get_drvdata(pdev);

	if (pdata == NULL || hcd == NULL)
		return -EINVAL;

	NvDdkUsbPhyClose(pdata->hUsbPhy);
	
	iounmap(hcd->regs);

	usb_remove_hcd(hcd);
	usb_put_hcd(hcd);

	if (pdata->hGpioIDpin)
		NvRmGpioReleasePinHandles(s_hGpioGlobal, &pdata->hGpioIDpin, 1);

	return 0;
}

#ifdef CONFIG_PM
static int tegra_ehci_suspend(struct platform_device *pdev,
	pm_message_t message)
{
	return -ENXIO;
}
static int tegra_ehci_resume(struct platform_device *pdev)
{
	return -ENXIO;
}
#else
#define tegra_ehci_resume NULL
#define tegra_ehci_suspend NULL
#endif

static struct platform_driver tegra_ehci_driver =
{
	.probe		= tegra_ehci_probe,
	.remove		= tegra_ehci_remove,
	.shutdown	= usb_hcd_platform_shutdown,
	.suspend	= tegra_ehci_suspend,
	.resume		= tegra_ehci_resume,
	.driver		= {
		.name	= "tegra-ehci",
	}
};


