/*
 * drivers/mmc/host/sdhci-tegra.c
 *
 * SDHCI-compatible driver for NVIDIA Tegra SoCs
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

#include <linux/mmc/host.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/tegra_devices.h>

#include "sdhci.h"
#include "mach/nvrm_linux.h"
#include "mach/io.h"
#include "nvodm_sdio.h"
#include "nvrm_module.h"
#include "nvrm_power.h"
#include "nvrm_interrupt.h"
#include "nvrm_pmu.h"
#include "nvos.h"
#include "nvodm_query_gpio.h"

#define DRIVER_DESC "Nvidia Tegra SDHCI compliant driver"
#define DRIVER_NAME "tegra_sdhci"

struct tegra_sdhci
{
	struct sdhci_host	*sdhost;
	struct platform_device	*pdev;
	NvOdmSdioHandle		hSdioHandle;
	NvU32			ModId;
	NvRmGpioPinHandle	wp_pin;
	NvU32			wp_polarity;
	NvRmGpioPinState	cd_pin;
	NvU32			cd_polarity;
	NvRmGpioInterruptHandle	cd_int;
	NvU32			MaxClock;
#ifdef CONFIG_EMBEDDED_MMC_START_OFFSET
	NvU32			StartOffset;
#endif
};

static int tegra_sdhci_enable_dma(struct sdhci_host *host)
{
	return 0;
}

/*
 *  Return 0 if the card is removed.
 *  -ve if the feature is not supported.
 *  +ve if the card is inserted.
 */
static int tegra_sdhci_detect(struct tegra_sdhci *t_sdhci)
{
	NvRmGpioPinState val;

	if (t_sdhci->cd_pin) {
		NvRmGpioReadPins(s_hGpioGlobal, &(t_sdhci->cd_pin), &val, 1);
		return val == t_sdhci->cd_polarity;
	}

	return -1;
}

static void do_handle_cardetect(void *args)
{
	struct sdhci_host *sdhost = (struct sdhci_host *)args;
	struct tegra_sdhci *t_sdhci = sdhci_priv(sdhost);

	sdhost->card_present = tegra_sdhci_detect(t_sdhci);
	sdhci_card_detect_callback(sdhost);
	NvRmGpioInterruptDone(t_sdhci->cd_int);
}

/*
 * Return 
 * -ve value if not supported.
 * 0 if not write protected.
 * +ve value if write protected.
 */
static int tegra_sdhci_get_ro(struct sdhci_host *host)
{
	struct tegra_sdhci *t_sdhci;
	NvRmGpioPinState val = NvRmGpioPinState_Low;

	t_sdhci = sdhci_priv(host);

	if (tegra_sdhci_detect(t_sdhci) && t_sdhci->wp_pin) {
		NvRmGpioReadPins(s_hGpioGlobal, &(t_sdhci->wp_pin), &val, 1);
		return val == t_sdhci->wp_polarity;
	}

	return -1;
}

#ifdef CONFIG_EMBEDDED_MMC_START_OFFSET
static unsigned int tegra_sdhci_get_StartOffset(struct sdhci_host *host)
{
	struct tegra_sdhci *t_sdhci;

	t_sdhci = sdhci_priv(host);

	return t_sdhci->StartOffset;
}
#endif

static unsigned int tegra_sdhci_get_maxclock(struct sdhci_host *host)
{
	struct tegra_sdhci *t_sdhci;

	t_sdhci = sdhci_priv(host);

	return t_sdhci->MaxClock / 1000;
}

static int tegra_sdhci_set_clock(struct sdhci_host *host,
	unsigned int clock)
{
	struct tegra_sdhci *t_sdhci;

	t_sdhci = sdhci_priv(host);
	if (clock) {
		NvError err;
		NvU32 min, max, target, actual_freq;
		min = 400;
		max = clock / 1000;
		target = clock / 1000;

		if (max < min) max  = min;
		if (target < min) target  = min;

		err = NvRmPowerModuleClockConfig(s_hRmGlobal,
			t_sdhci->ModId, 0, min, max, &target, 1,
			&actual_freq, 0);
		if (err==NvSuccess)
			return 1;
	} else {
		/* TODO Do we need to shutdown the clock?  */
	}
	return 0;
}

static struct sdhci_ops tegra_sdhci_ops = {
	.enable_dma		= tegra_sdhci_enable_dma,
	.get_ro			= tegra_sdhci_get_ro,
	.get_maxclock		= tegra_sdhci_get_maxclock,
	.set_clock		= tegra_sdhci_set_clock,
#ifdef CONFIG_EMBEDDED_MMC_START_OFFSET
	.get_startoffset	= tegra_sdhci_get_StartOffset,
#endif
};

struct sdioCaps {
	NvBool EnableDmaSupport;
};

int __init tegra_sdhci_probe(struct platform_device *pdev)
{
	struct sdhci_host *sdhost;
	struct tegra_sdhci *host;
	NvError err1, err2, err3;
	NvU32 irq = 0xffff;
	NvU32 addr;
	NvU32 size;
	NvU32 min, max, target;
	NvU32 ModId;
	int ret = -ENODEV;
#ifdef CONFIG_EMBEDDED_MMC_START_OFFSET
	struct tegra_sdio_platform_data *pdata = pdev->dev.platform_data;
#endif
	NvRmModuleSdmmcInterfaceCaps SdioInterfaceCaps;

	struct sdioCaps sdioCapsArray[2];
	struct sdioCaps *pSdioCaps = NULL;
	NvRmModuleCapability sdioAllChipCaps[] = {
		{1, 0, 0, &sdioCapsArray[0]},
		{2, 0, 0, &sdioCapsArray[1]},
	};
	const NvOdmGpioPinInfo *gp_info;
	NvU32 PinCount;

	/* Only enable DMA support on AP2x chips */
	sdioCapsArray[0].EnableDmaSupport = NV_FALSE;
	sdioCapsArray[1].EnableDmaSupport = NV_TRUE;

	if (pdev->id == -1)
		return -ENODEV;
	ModId = NVRM_MODULE_ID(NvRmModuleID_Sdio, pdev->id);

	sdhost  = sdhci_alloc_host(&pdev->dev, sizeof(struct tegra_sdhci));
	if (!sdhost) {
		return -ENOMEM;
	}

	host = sdhci_priv(sdhost);

	host->pdev = pdev;
	host->sdhost = sdhost;
	host->ModId = ModId;
#ifdef CONFIG_EMBEDDED_MMC_START_OFFSET
	if (pdata)
		host->StartOffset = pdata->StartOffset;
#endif

	host->hSdioHandle = NvOdmSdioOpen(pdev->id);
	if (!host->hSdioHandle) {
		goto fail;
	}

	/* Get the GPIOs for the card detect and the write protect */
	gp_info = NvOdmQueryGpioPinMap(NvOdmGpioPinGroup_Sdio,
		pdev->id, &PinCount);

	if (gp_info) {
		switch (PinCount) {
		/* Both write proect pin and card insert pin is specified */
		case 2:
			NvRmGpioAcquirePinHandle(s_hGpioGlobal, gp_info[1].Port,
				gp_info[1].Pin, &host->wp_pin);
			host->wp_polarity = gp_info[1].activeState;
			NvRmGpioConfigPins(s_hGpioGlobal, &host->wp_pin, 1,
				NvRmGpioPinMode_InputData);
				/*fall through*/
			/* only card detect pin is specified */
		case 1:
			NvRmGpioAcquirePinHandle(s_hGpioGlobal, gp_info[0].Port,
				gp_info[0].Pin, &host->cd_pin);
			host->cd_polarity = gp_info[0].activeState;
			NvRmGpioConfigPins(s_hGpioGlobal, &host->cd_pin, 1,
				NvRmGpioPinMode_InputData);
			break;
		default:
			dev_info(&pdev->dev, "Invalid pin count(%d) for SDIO "
				"instance %d\n", PinCount, pdev->id);
		}
	} else {
		dev_info(&pdev->dev, "No GPIO map for SDIO instance %d\n",
			pdev->id);
	}

	irq = NvRmGetIrqForLogicalInterrupt(s_hRmGlobal, ModId, 0);
	NvRmModuleGetBaseAddress(s_hRmGlobal, ModId, &addr, &size);
	if (irq == 0xffff || addr == 0x0 || size == 0) {
		goto fail;
	}

	min = 400;
	max = NvRmFreqMaximum;
	target = NvRmFreqMaximum;
	err1 = NvRmSetModuleTristate(s_hRmGlobal, ModId, NV_FALSE);
	err2 = NvRmPowerModuleClockConfig(s_hRmGlobal, ModId, 0,
		min, max, &target, 1, &host->MaxClock, 0);

	err3 = NvRmPowerModuleClockControl(s_hRmGlobal, ModId, 0, NV_TRUE);
	if (err1 || err2 || err3) {
		goto fail;
	}
	NvRmModuleReset(s_hRmGlobal, ModId);
	NvOdmSdioResume(host->hSdioHandle);

	NvRmModuleGetCapabilities(s_hRmGlobal,
		NVRM_MODULE_ID(NvRmModuleID_Sdio, pdev->id),
		sdioAllChipCaps, NV_ARRAY_SIZE(sdioAllChipCaps),
		(void**)&pSdioCaps);

	sdhost->hw_name = "tegra";
	sdhost->ops = &tegra_sdhci_ops;
	sdhost->irq = irq;

	/* Set the irq flags to irq valid, which is the default linux behaviour.
	 * For irqs used by Nv* APIs, IRQF_NOAUTOEN is also set */
	set_irq_flags(irq, IRQF_VALID);
	sdhost->ioaddr = IO_ADDRESS(addr);

	sdhost->quirks =
		SDHCI_QUIRK_BROKEN_TIMEOUT_VAL |
		SDHCI_QUIRK_SINGLE_POWER_WRITE |
	SDHCI_QUIRK_ENABLE_INTERRUPT_AT_BLOCK_GAP |
	SDHCI_QUIRK_BROKEN_WRITE_PROTECT |
	SDHCI_QUIRK_PRESENT_STATE_REGISTER_INVALID |
	SDHCI_QUIRK_BROKEN_CTRL_HISPD;

	if (!pSdioCaps->EnableDmaSupport)
		sdhost->quirks |= SDHCI_QUIRK_BROKEN_DMA;
	else {
		sdhost->quirks |= SDHCI_QUIRK_BROKEN_SPEC_VERSION;
		sdhost->quirks |= SDHCI_QUIRK_32KB_MAX_ADMA_SIZE;
	}

	err1 = NvRmGetModuleInterfaceCapabilities(s_hRmGlobal,
		NVRM_MODULE_ID(NvRmModuleID_Sdio, pdev->id),
		sizeof(NvRmModuleSdmmcInterfaceCaps), &SdioInterfaceCaps);
	if (err1 == NvSuccess && SdioInterfaceCaps.MmcInterfaceWidth >= 8)
		sdhost->data_width = 8;

	ret = sdhci_add_host(sdhost);
	if (ret)
		goto fail;

	platform_set_drvdata(pdev, host);

	if (host->cd_pin) {
		/* Register interrupt handler for */
		err1 = NvRmGpioInterruptRegister(s_hGpioGlobal, s_hRmGlobal,
			host->cd_pin, do_handle_cardetect,
			NvRmGpioPinMode_InputInterruptAny, sdhost,
			&host->cd_int, 5);
		if (err1 != NvSuccess)
			goto fail;
		err1 = NvRmGpioInterruptEnable(host->cd_int);
		if (err1 != NvSuccess)
			goto fail;
	}

	sdhost->card_present = tegra_sdhci_detect(host);

	dev_info(&pdev->dev, "tegra_sdhci_probe: irq(%d) io(0x%x) phys(0x%x)",
		irq, (NvU32)sdhost->ioaddr, addr);

	return  0;

fail:
	dev_err(&pdev->dev, "tegra_sdhci_probe failed\n");

	if (host->hSdioHandle)
		NvOdmSdioClose(host->hSdioHandle);

	if (sdhost)
		sdhci_free_host(sdhost);

	return ret;
}


static int __devexit tegra_sdhci_remove(struct platform_device *pdev)
{
	struct tegra_sdhci *host = dev_get_drvdata(&pdev->dev);

	if (host->hSdioHandle)
		NvOdmSdioClose(host->hSdioHandle);
	(void)NvRmSetModuleTristate(s_hRmGlobal, host->ModId, NV_TRUE);
	(void)NvRmPowerModuleClockControl(s_hRmGlobal,
		host->ModId, 0, NV_FALSE);

	if (host->wp_pin)
		NvRmGpioReleasePinHandles(s_hGpioGlobal, &host->wp_pin, 1);

	if (host->cd_int)
		NvRmGpioInterruptUnregister(s_hGpioGlobal, s_hRmGlobal,
			host->cd_int);

	sdhci_free_host(host->sdhost);

	return 0;
}

struct platform_driver tegra_sdhci_driver = {
	.probe		= tegra_sdhci_probe,
	.remove		= __devexit_p(tegra_sdhci_remove),
	.suspend	= NULL,
	.resume		= NULL,
	.driver		= {
		.name	= "tegra-sdhci",
		.owner	= THIS_MODULE,
	},
};

static int __init tegra_sdhci_init(void)
{
	return platform_driver_register(&tegra_sdhci_driver);
}

static void __exit tegra_sdhci_exit(void)
{
	platform_driver_unregister(&tegra_sdhci_driver);
}

module_init(tegra_sdhci_init);
module_exit(tegra_sdhci_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_DESC);

