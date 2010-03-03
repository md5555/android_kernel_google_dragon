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

#define TEGRA_USB_ID_INT_ENABLE			(1 << 0)
#define TEGRA_USB_ID_INT_STATUS			(1 << 1)
#define TEGRA_USB_ID_PIN_STATUS			(1 << 2)
#define TEGRA_USB_ID_PIN_WAKEUP_ENABLE		(1 << 6)
#define TEGRA_USB_PHY_WAKEUP_REG_OFFSET		(0x408)
#define TEGRA_USB_USBMODE_REG_OFFSET		(0x1a8)
#define TEGRA_USB_USBMODE_HOST			(3)

static void tegra_ehci_power_up(struct usb_hcd *hcd)
{
	struct ehci_hcd	*ehci = hcd_to_ehci(hcd);
	struct tegra_hcd_platform_data *pdata;

	pdata = hcd->self.controller->platform_data;

	NV_ASSERT_SUCCESS(NvDdkUsbPhyPowerUp(pdata->hUsbPhy, NV_TRUE, 0));
	ehci->host_resumed = 1;
}

static void tegra_ehci_power_down(struct usb_hcd *hcd)
{
	struct ehci_hcd	*ehci = hcd_to_ehci(hcd);
	struct tegra_hcd_platform_data *pdata;

	pdata = hcd->self.controller->platform_data;

	NV_ASSERT_SUCCESS(NvDdkUsbPhyPowerDown(pdata->hUsbPhy, NV_TRUE, 0));
	ehci->host_resumed = 0;
}

static int tegra_ehci_hub_control (
	struct usb_hcd	*hcd,
	u16		typeReq,
	u16		wValue,
	u16		wIndex,
	char		*buf,
	u16		wLength
) {
	struct ehci_hcd	*ehci = hcd_to_ehci (hcd);
	u32 __iomem	*status_reg = &ehci->regs->port_status[
				(wIndex & 0xff) - 1];
	u32		temp;
	struct tegra_hcd_platform_data *pdata;
	unsigned long	flags;
	int		retval = 0;

	/* initialize the platform data pointer */
	pdata = hcd->self.controller->platform_data;

	/* if hardware is not accessable then don't read the registers */
	if (!test_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags)) {
		if (buf)
			memset (buf, 0, wLength);
		return retval;
	}

	/* In ehci_hub_control() for USB_PORT_FEAT_ENABLE clears the other bits
	 * that are write on clear, by wrting back the register read value, so
	 * USB_PORT_FEAT_ENABLE is handled here by masking the set on clear bits */
	if ((typeReq == ClearPortFeature) && (wValue == USB_PORT_FEAT_ENABLE)) {
		spin_lock_irqsave (&ehci->lock, flags);
		temp = ehci_readl(ehci, status_reg);
		ehci_writel(ehci, (temp & ~PORT_RWC_BITS) & ~PORT_PE, status_reg);
		spin_unlock_irqrestore (&ehci->lock, flags);
		return retval;
	}

	/* Handle the hub control events here */
	retval = ehci_hub_control(hcd, typeReq, wValue, wIndex, buf, wLength);

	/* Power down the USB phy when there is no port connection and all
	 * HUB events are cleared by checking the lower four bits
	 * (PORT_CONNECT | PORT_CSC | PORT_PE | PORT_PEC) */
#ifdef CONFIG_USB_OTG_UTILS
	if ((pdata->pUsbProperty->UsbMode == NvOdmUsbModeType_OTG)
		&& ehci->transceiver) {
		if (ehci->transceiver->state == OTG_STATE_A_SUSPEND) {
			temp = ehci_readl(ehci, status_reg);
			if (!(temp & (PORT_CONNECT | PORT_CSC | PORT_PE | PORT_PEC))
				&& ehci->host_reinited) {
				tegra_ehci_power_down(hcd);
				ehci->transceiver->state = OTG_STATE_UNDEFINED;
				ehci->host_reinited = 0;
				/* indicate hcd flags, that hardware is not accessable now */
				clear_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
			}
		}
	}
#endif

	return retval;
}

#ifdef CONFIG_USB_OTG_UTILS
static void tegra_ehci_restart (struct usb_hcd *hcd)
{
	unsigned int temp;
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);

	/* Set to Host mode by setting bit 0-1 of USB device mode register */
	temp = readl(hcd->regs + TEGRA_USB_USBMODE_REG_OFFSET);
	writel((temp | TEGRA_USB_USBMODE_HOST),
		(hcd->regs + TEGRA_USB_USBMODE_REG_OFFSET));

	/* reset the ehci controller */
	ehci->controller_resets_phy = 0;
	ehci_reset(ehci);
	ehci->controller_resets_phy = 1;
	/* setup the frame list and Async q heads */
	ehci_writel(ehci, ehci->periodic_dma, &ehci->regs->frame_list);
	ehci_writel(ehci, (u32)ehci->async->qh_dma, &ehci->regs->async_next);
	/* setup the command register and set the controller in RUN mode */
	ehci->command &= ~(CMD_LRESET|CMD_IAAD|CMD_PSE|CMD_ASE|CMD_RESET);
	ehci->command |= CMD_RUN;
	ehci_writel(ehci, ehci->command, &ehci->regs->command);

	down_write(&ehci_cf_port_reset_rwsem);
	hcd->state = HC_STATE_RUNNING;
	/* unblock posted writes */
	ehci_writel(ehci, FLAG_CF, &ehci->regs->configured_flag);
	ehci_readl(ehci, &ehci->regs->command);
	up_write(&ehci_cf_port_reset_rwsem);

	/* Turn On Interrupts */
	ehci_writel(ehci, INTR_MASK, &ehci->regs->intr_enable);
}
#endif

static void tegra_ehci_shutdown (struct usb_hcd *hcd)
{
	/* ehci_shutdown touches the USB controller registers, make sure
	 * controller has clocks to it */
	tegra_ehci_power_up(hcd);
	/* call ehci shut down */
	ehci_shutdown(hcd);
	/* we are ready to shut down, powerdown the phy */
	tegra_ehci_power_down(hcd);
}

static irqreturn_t tegra_ehci_irq (struct usb_hcd *hcd)
{
	struct ehci_hcd *ehci = hcd_to_ehci (hcd);
	struct tegra_hcd_platform_data *pdata;
	u32 status;

	pdata = hcd->self.controller->platform_data;

	spin_lock (&ehci->lock);

#ifdef CONFIG_USB_OTG_UTILS
	if ((pdata->pUsbProperty->UsbMode == NvOdmUsbModeType_OTG)
		&& ehci->transceiver) {
		if (ehci->transceiver->state == OTG_STATE_A_HOST) {
			if (!ehci->host_reinited) {
				ehci->host_reinited = 1;
				tegra_ehci_power_up(hcd);
				tegra_ehci_restart(hcd);
			}
		} else if (ehci->transceiver->state == OTG_STATE_A_SUSPEND) {
			if (!ehci->host_reinited) {
				spin_unlock (&ehci->lock);
				return IRQ_HANDLED;
			}
		} else {
			spin_unlock (&ehci->lock);
			return IRQ_HANDLED;
		}
	} else
#endif
	{
		if (pdata->pUsbProperty->IdPinDetectionType ==
			NvOdmUsbIdPinType_CableId) {
			/* read otgsc register for ID pin status change */
			status = readl(hcd->regs + TEGRA_USB_PHY_WAKEUP_REG_OFFSET);
			writel(status, (hcd->regs + TEGRA_USB_PHY_WAKEUP_REG_OFFSET));

			/* Check if there is any ID pin interrupt */
			if (status & TEGRA_USB_ID_INT_STATUS) {
				/* Check pin status and enable/disable the power */
				if (status & TEGRA_USB_ID_PIN_STATUS) {
					tegra_ehci_power_down(hcd);
				} else {
					tegra_ehci_power_up(hcd);
				}
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
	tegra_ehci_power_up(hcd);

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

static int tegra_ehci_bus_suspend(struct usb_hcd *hcd)
{
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	int err_status = 0;

#ifdef CONFIG_USB_OTG_UTILS
	struct tegra_hcd_platform_data *pdata;
	/* initialize the platform data pointer */
	pdata = hcd->self.controller->platform_data;
	if ((pdata->pUsbProperty->UsbMode == NvOdmUsbModeType_OTG)
		&& ehci->transceiver) {
		if (ehci->transceiver->state != OTG_STATE_A_HOST) {
			/* we are not in host mode, return */
			return err_status;
		}
	}
#endif

	if (ehci->host_resumed) {
		err_status = ehci_bus_suspend(hcd);
		if (!err_status)
			tegra_ehci_power_down(hcd);
	}
	return err_status;
}

static int tegra_ehci_bus_resume(struct usb_hcd *hcd)
{
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	int err_status = 0;
	u32 status;

	struct tegra_hcd_platform_data *pdata;
	/* initialize the platform data pointer */
	pdata = hcd->self.controller->platform_data;

#ifdef CONFIG_USB_OTG_UTILS
	if ((pdata->pUsbProperty->UsbMode == NvOdmUsbModeType_OTG)
		&& ehci->transceiver) {
		if (ehci->transceiver->state != OTG_STATE_A_HOST) {
			/* we are not in host mode, return */
			return err_status;
		}
	}
#endif

	if (!ehci->host_resumed) {
		tegra_ehci_power_up(hcd);
		err_status = ehci_bus_resume(hcd);
	}

	if ((pdata->pUsbProperty->UsbMode != NvOdmUsbModeType_OTG)
		&& (pdata->pUsbProperty->IdPinDetectionType ==
			NvOdmUsbIdPinType_CableId)) {
		/* read otgsc register for ID pin status */
		status = readl(hcd->regs + TEGRA_USB_PHY_WAKEUP_REG_OFFSET);
		writel(status, (hcd->regs + TEGRA_USB_PHY_WAKEUP_REG_OFFSET));
		/* If no Id pin then disable the power */
		if (status & TEGRA_USB_ID_PIN_STATUS) {
			tegra_ehci_power_down(hcd);
		}
        }
	return err_status;
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
	.shutdown		= tegra_ehci_shutdown,
	.urb_enqueue		= ehci_urb_enqueue,
	.urb_dequeue		= ehci_urb_dequeue,
	.endpoint_disable	= ehci_endpoint_disable,
	.get_frame_number	= ehci_get_frame,
	.hub_status_data	= ehci_hub_status_data,
	.hub_control		= tegra_ehci_hub_control,
	.bus_suspend		= tegra_ehci_bus_suspend,
	.bus_resume 		= tegra_ehci_bus_resume,
	.relinquish_port	= ehci_relinquish_port,
	.port_handed_over	= ehci_port_handed_over,
};
static int tegra_ehci_probe(struct platform_device *pdev)
{
	int instance = pdev->id;
	struct resource *res;
	struct tegra_hcd_platform_data *pdata;
	struct usb_hcd *hcd;
	int e = 0;
	int irq;
	unsigned int temp;
	static u64 dummy_mask = DMA_32BIT_MASK;
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

		/* Configure Cable ID Pin as Input Pin */
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
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		e = -ENXIO;
		goto fail;
	}
	hcd->rsrc_start = res->start;
	hcd->rsrc_len = resource_size(res);
	hcd->regs = ioremap(res->start, resource_size(res));
	if (!hcd->regs) {
		e = -ENOMEM;
		goto fail;
	}

	/* Set to Host mode by setting bit 0-1 of USB device mode register */
	temp = readl(hcd->regs + TEGRA_USB_USBMODE_REG_OFFSET);
	writel((temp | TEGRA_USB_USBMODE_HOST),
			(hcd->regs + TEGRA_USB_USBMODE_REG_OFFSET));
	irq = platform_get_irq(pdev, 0);
	if (!irq) {
		e = -ENODEV;
		goto fail;
	}

	set_irq_flags(irq, IRQF_VALID);

	e  = usb_add_hcd(hcd, irq, IRQF_DISABLED | IRQF_SHARED);
	if (e != 0)
		goto fail;
	platform_set_drvdata(pdev, hcd);

#ifdef CONFIG_USB_OTG_UTILS
	if (pdata->pUsbProperty->UsbMode == NvOdmUsbModeType_OTG) {
		struct ehci_hcd *ehci = hcd_to_ehci(hcd);
		ehci->transceiver = otg_get_transceiver();
		if (ehci->transceiver) {
			otg_set_host(ehci->transceiver, (struct usb_bus *)hcd);
			/* Stop the controller and power down the phy, OTG will
			 * start the host driver based on the ID pin
			 * detection */
			ehci_halt(ehci);
			/* reset the host and put the controller in idle mode */
			temp = ehci_readl(ehci, &ehci->regs->command);
			temp |= CMD_RESET;
			ehci_writel(ehci, temp, &ehci->regs->command);
			temp = readl(hcd->regs + TEGRA_USB_USBMODE_REG_OFFSET);
			writel((temp & ~TEGRA_USB_USBMODE_HOST),
				(hcd->regs + TEGRA_USB_USBMODE_REG_OFFSET));
			tegra_ehci_power_down(hcd);
			ehci->host_reinited = 0;
			/* indicate hcd flags, that hardware is not accessable now in host mode*/
			clear_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
		} else {
			dev_err(&pdev->dev, "Cannot get OTG transceiver\n");
			e = -ENODEV;
			goto fail;
		}
	} else
#endif
	{
		if (pdata->pUsbProperty->IdPinDetectionType ==
			NvOdmUsbIdPinType_CableId) {
			/* enable the cable ID interrupt */
			temp = readl(hcd->regs + TEGRA_USB_PHY_WAKEUP_REG_OFFSET);
			temp |= TEGRA_USB_ID_INT_ENABLE;
			temp |= TEGRA_USB_ID_PIN_WAKEUP_ENABLE;
			writel(temp, (hcd->regs + TEGRA_USB_PHY_WAKEUP_REG_OFFSET));

			/* Check if we detect any device connected */
			if (temp & TEGRA_USB_ID_PIN_STATUS) {
				tegra_ehci_power_down(hcd);
			} else {
				tegra_ehci_power_up(hcd);
			}
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

static struct platform_driver tegra_ehci_driver =
{
	.probe		= tegra_ehci_probe,
	.remove		= tegra_ehci_remove,
	.shutdown	= usb_hcd_platform_shutdown,
	.driver		= {
		.name	= "tegra-ehci",
	}
};
