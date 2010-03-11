/*
 * arch/arm/mach-tegra/board-nvodm.c
 *
 * Board registration for ODM-kit generic Tegra boards
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
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/synaptics_i2c_rmi.h>
#include <linux/i2c.h>
#include <linux/pm.h>
#include <linux/spi/spi.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <mach/board.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>

#include <linux/io.h>
#include <linux/serial_8250.h>

#include <linux/delay.h>

#include "nvcommon.h"
#include "nvrm_init.h"
#include "nvrm_module.h"
#include "nvrm_interrupt.h"
#include "nvrm_pinmux.h"
#include "nvrm_power.h"
#include "nvrm_pmu.h"
#include "nvodm_query.h"
#include "nvodm_services.h"
#include "nvodm_sdio.h"
#include "nvodm_pmu.h"
#include "nvrm_gpio.h"
#include "mach/nvrm_linux.h"
#include "nvassert.h"
#include "nvodm_query_discovery.h"

#include <../../../drivers/staging/android/timed_output.h>

#if defined(CONFIG_USB_ANDROID) || defined(CONFIG_USB_ANDROID_MODULE)
#include <linux/usb/android.h>
#define SERIAL_NUMBER_STRING_LEN 16
#endif

extern struct sys_timer tegra_timer;
extern const char* tegra_boot_device;
extern void __init tegra_init_irq(void);
extern void __init tegra_map_common_io(void);

static struct platform_device nvrm_device =
{
    .name = "nvrm"
};

#if defined(CONFIG_USB_ANDROID) || defined(CONFIG_USB_ANDROID_MODULE)
static struct android_usb_platform_data android_usb_plat =
{
    .vendor_id = 0x0955,
    .product_id = 0x7000,
    .adb_product_id = 0x7100,
    .product_name = "ADB Composite Device",
    .manufacturer_name = "NVIDIA Corporation",
    .serial_number = "0000000000000000",
    .nluns = 1,
    .bulk_size = 16384,
};

static struct platform_device android_usb_device =
{
    .name = "android_usb",
    .dev =
    {
        .platform_data = &android_usb_plat,
    },
};
#endif

#ifdef CONFIG_TEGRA_NVEC_USER
static struct platform_device tegra_nvec =
{
    .name = "nvec",
    .id   = -1,
};
#endif

#if defined(CONFIG_USB_GADGETFS) || defined(CONFIG_USB_GADGETFS_MODULE)
static struct platform_device android_gadgetfs_device =
{
    .name = "gadgetfs",
    .id   = -1,
};
#endif

#if defined(CONFIG_BATTERY_TEGRA_ODM) || defined(CONFIG_TEGRA_BATTERY_NVEC)
static struct platform_device tegra_battery_device =
{
    .name = "tegra_battery",
    .id   = -1,
};
#endif

#ifdef CONFIG_MTD_NAND_TEGRA
static struct platform_device tegra_nand_device =
{
    .name = "tegra_nand",
    .id   = -1,
};
#endif


#ifdef CONFIG_RTC_DRV_TEGRA_ODM
static struct platform_device tegra_rtc_device =
{
    .name = "tegra_rtc",
    .id   = -1,
};
#endif

#ifdef CONFIG_TOUCHSCREEN_TEGRA_ODM
static struct platform_device tegra_touch_device =
{
    .name = "tegra_touch",
    .id   = -1,
};
#endif

#ifdef CONFIG_INPUT_TEGRA_ODM_ACCEL
static struct platform_device tegra_accelerometer_device =
{
    .name = "tegra_accelerometer",
    .id   = -1,
};
#endif

#ifdef CONFIG_KEYBOARD_TEGRA
static struct platform_device tegra_kbc_device =
{
    .name = "tegra_kbc",
    .id   = -1,
};
#endif

#ifdef CONFIG_INPUT_TEGRA_ODM_SCROLL
static struct platform_device tegra_scrollwheel_device =
{
    .name = "tegra_scrollwheel",
    .id   = -1,
};
#endif

#ifdef CONFIG_TEGRA_ODM_VIBRATE
static struct timed_output_dev tegra_vibrator_device = {
    .name = "tegra_vibrator",
};
#endif

#ifdef CONFIG_TEGRA_ODM_RFKILL
static struct platform_device tegra_rfkill =
{
    .name = "tegra_rfkill",
    .id   = -1,
};
#endif


/*  FIXME:  NvBl uses 57.6k for FPGAs.  16x multiplier for UART baud rate
 *  should be chip-dependent. */
#define TARGET_BAUD_RATE 115200
static void __init NvGetUartClockRange(NvU32 BaudBitRate, NvU32 *pTargetClkKHz,
    NvU32 *pMinClkKHz, NvU32 *pMaxClkKHz)
{
    NvU32 TargetClkKHz = (BaudBitRate * 16) / 1000;
    //  DDK uses +- 5% tolerance.  We use +- 6.25% to remove the extra divides
    *pMinClkKHz = TargetClkKHz - (TargetClkKHz>>4);
    *pMaxClkKHz = TargetClkKHz + (TargetClkKHz>>4);
    *pTargetClkKHz = TargetClkKHz;
}

static void __init NvConfigDebugConsole(
    NvRmDeviceHandle hRm)
{
    NvOdmDebugConsole Console = NvOdmQueryDebugConsole();
    struct plat_serial8250_port *pUartDebugPort = NULL;
    struct platform_device *pDebugConsole = NULL;
    NvU32 ModId = 0;

    /*  The debug console uses the standard serial 8250 driver,
     *  rather than the power-managed Tegra UART driver */
    if (((NvU32)Console >= (NvU32)NvOdmDebugConsole_UartA) &&
        ((NvU32)Console <= (NvU32)NvOdmDebugConsole_UartE))
    {
        NvU32 Port = (NvU32)(Console - NvOdmDebugConsole_UartA);
        NvU32 Clk, TargetClk, MinClk, MaxClk, Temp;

        ModId = NVRM_MODULE_ID(NvRmModuleID_Uart, Port);

        if (NvRmSetModuleTristate(hRm, ModId, NV_FALSE)!=NvSuccess)
            goto exit_fail;

        NvGetUartClockRange(TARGET_BAUD_RATE, &TargetClk, &MinClk, &MaxClk);

        if (NvRmPowerModuleClockConfig(hRm, ModId, 0, MinClk, MaxClk,
                &TargetClk, 1, &Clk, 0)!=NvSuccess)
            goto cleanup_ts;

        if (NvRmPowerModuleClockControl(hRm, ModId, 0, NV_TRUE)!=NvSuccess)
            goto cleanup_ts;

        NvRmModuleReset(hRm, ModId);
        pDebugConsole = (struct platform_device *)
            NvOsAlloc(sizeof(struct platform_device));
        if (!pDebugConsole)
            goto cleanup;
        pUartDebugPort = (struct plat_serial8250_port *)
            NvOsAlloc(2*sizeof(struct plat_serial8250_port));
        if (!pUartDebugPort)
            goto cleanup;

        NvOsMemset(pUartDebugPort, 0, 2*sizeof(struct plat_serial8250_port));
        NvOsMemset(pDebugConsole, 0, sizeof(struct platform_device));
        pDebugConsole->name = "serial8250";
        pDebugConsole->id = PLAT8250_DEV_PLATFORM;

        NvRmModuleGetBaseAddress(hRm, ModId, &pUartDebugPort->mapbase, &Temp);
        pUartDebugPort->membase = IO_ADDRESS(pUartDebugPort->mapbase);
        pUartDebugPort->uartclk = Clk * 1000;
        pUartDebugPort->irq = NvRmGetIrqForLogicalInterrupt(hRm, ModId, 0);
        pUartDebugPort->iotype = UPIO_MEM32;
        pUartDebugPort->regshift = 2;
        pUartDebugPort->flags = UPF_BOOT_AUTOCONF;
        pDebugConsole->dev.platform_data = pUartDebugPort;
        if (platform_device_register(pDebugConsole))
        {
            NV_ASSERT(!"Unable to register debug console device");
        }
    }

    return;

 cleanup:
    if (pDebugConsole)
        NvOsFree(pDebugConsole);
    if (pUartDebugPort)
        NvOsFree(pUartDebugPort);
    (void) NvRmPowerModuleClockControl(hRm, ModId, 0, NV_FALSE);
 cleanup_ts:
    (void) NvRmSetModuleTristate(hRm, ModId, NV_TRUE);
  exit_fail:
    return;
}

void tegra_set_voltage(NvU64 guid, int on)
{
	u32 settling_time;
	const NvOdmPeripheralConnectivity *con = NULL;
	int i;

	con = NvOdmPeripheralGetGuid(guid);
	if (con == NULL)
		return;

	for (i = 0; i < con->NumAddress; i++) {
		if (con->AddressList[i].Interface != NvOdmIoModule_Vdd)
			continue;
		if (on) {
			NvRmPmuVddRailCapabilities rail;
			NvRmPmuGetCapabilities(s_hRmGlobal,
				con->AddressList[i].Address, &rail);
			NvRmPmuSetVoltage(s_hRmGlobal,
				con->AddressList[i].Address,
				rail.requestMilliVolts, &settling_time);
		} else
			NvRmPmuSetVoltage(s_hRmGlobal,
				con->AddressList[i].Address, NVODM_VOLTAGE_OFF,
				&settling_time);
		udelay(settling_time);
	}
}

extern void __init tegra_common_init(void);
extern void __init tegra_clk_init(void);
#ifdef CONFIG_TEGRA_DPRAM
extern void __init tegra_init_snor_controller(void);
#endif


static void tegra_system_power_off(void)
{
	tegra_set_voltage(NV_VDD_SoC_ODM_ID, 0);
}

#if !(defined(CONFIG_ENC28J60) && defined(CONFIG_SPI_TEGRA))
#define register_enc28j60() do {} while (0)
#else
static struct spi_board_info tegra_spi_devices[] __initdata = {
    {
        .modalias = "enc28j60",
        .bus_num = 1,
        .chip_select = 0,
        .mode = SPI_MODE_0,
        .max_speed_hz = 18000000,
        .platform_data = NULL,
        .irq = 0,
    },
};

static void __init register_enc28j60(void)
{
    NvError err;
    NvRmGpioPinHandle hPin;
    NvU32 irq;
    NvU32 instance = 0xFFFF;
    NvU32 cs = 0xFFF;
    NvU32 pin = 0xFFFF, port = 0xFFFF;
    const NvOdmPeripheralConnectivity *pConnectivity = NULL;
    int i;
    const NvOdmQuerySpiDeviceInfo *pSpiDeviceInfo;

    pConnectivity =
        NvOdmPeripheralGetGuid(NV_ODM_GUID('e','n','c','2','8','j','6','0'));
    if (!pConnectivity)
        return;

    for (i = 0; i < pConnectivity->NumAddress; i++)
    {
        switch (pConnectivity->AddressList[i].Interface)
        {
            case NvOdmIoModule_Spi:
                instance = pConnectivity->AddressList[i].Instance;
                cs = pConnectivity->AddressList[i].Address;
                break;
            case NvOdmIoModule_Gpio:
                port = pConnectivity->AddressList[i].Instance;
                pin = pConnectivity->AddressList[i].Address;
                break;
            default:
                break;
        }
    }

    /* SPI ethernet driver needs one SPI info and a gpio for interrupt */
    if (instance == 0xffff || cs == 0xffff || port == 0xFFFF || pin == 0xFFFF)
        return;

    /* Check if the SPI is configured as a master for this instance
     * If it it not, don't register the device.
     * */
    pSpiDeviceInfo = NvOdmQuerySpiGetDeviceInfo(NvOdmIoModule_Spi, instance,
	cs);
    if (pSpiDeviceInfo && pSpiDeviceInfo->IsSlave)
        return;

    err = NvRmGpioAcquirePinHandle(s_hGpioGlobal, port, pin, &hPin);
    if (err)
    {
        return;
    }
    NvRmGpioConfigPins(s_hGpioGlobal, &hPin, 1,
        NvRmGpioPinMode_InputInterruptFallingEdge);
    NvRmGpioGetIrqs(s_hRmGlobal, &hPin, &irq, 1);

    printk("Enabled SPI driver\n");

    tegra_spi_devices[0].irq = irq;
    /* FIXME, instance need not be same as bus number. */
    tegra_spi_devices[0].bus_num = instance;
    tegra_spi_devices[0].chip_select = cs;
    spi_register_board_info(tegra_spi_devices, ARRAY_SIZE(tegra_spi_devices));
}
#endif

static void __init tegra_machine_init(void)
{
#if defined(CONFIG_USB_ANDROID) || defined(CONFIG_USB_ANDROID_MODULE)
    NvU32 serial_number[2] = {0};
#endif

    (void) platform_device_register(&nvrm_device);

    tegra_common_init();
    tegra_clk_init();
    NvConfigDebugConsole(s_hRmGlobal);

#ifdef CONFIG_TEGRA_DPRAM
    tegra_init_snor_controller();
#endif

#ifdef CONFIG_TEGRA_NVEC_USER
    (void) platform_device_register(&tegra_nvec);
#endif

#ifdef CONFIG_TOUCHSCREEN_TEGRA_ODM
    (void) platform_device_register(&tegra_touch_device);
#endif

#ifdef CONFIG_INPUT_TEGRA_ODM_ACCEL
    (void) platform_device_register(&tegra_accelerometer_device);
#endif

#ifdef CONFIG_TEGRA_ODM_VIBRATE
    (void) timed_output_dev_register(&tegra_vibrator_device);
#endif

    register_enc28j60();

    /* register the devices */
#ifdef CONFIG_MTD_NAND_TEGRA
    if (tegra_boot_device && !NvOsMemcmp(tegra_boot_device, "nand", 4))
        (void) platform_device_register(&tegra_nand_device);
#endif

#ifdef CONFIG_RTC_DRV_TEGRA_ODM
    (void) platform_device_register(&tegra_rtc_device);
#endif

#ifdef CONFIG_KEYBOARD_TEGRA
    (void) platform_device_register(&tegra_kbc_device);
#endif

#if defined(CONFIG_BATTERY_TEGRA_ODM) || defined(CONFIG_TEGRA_BATTERY_NVEC)
    (void) platform_device_register(&tegra_battery_device);
#endif

#if defined(CONFIG_USB_ANDROID) || defined(CONFIG_USB_ANDROID_MODULE)
    // get the board specific unique ID
    (void) NvRmQueryChipUniqueId(
                s_hRmGlobal,
                sizeof(NvU32) * 2,
                (void *)serial_number);

    snprintf(android_usb_plat.serial_number,
             SERIAL_NUMBER_STRING_LEN,
             "%08X%08X",
             serial_number[1],
             serial_number[0]);
    android_usb_device.dev.platform_data = &android_usb_plat;
    (void) platform_device_register(&android_usb_device);
#endif
#if defined(CONFIG_USB_GADGETFS) || defined(CONFIG_USB_GADGETFS_MODULE)
    (void) platform_device_register(&android_gadgetfs_device);
#endif

#ifdef CONFIG_INPUT_TEGRA_ODM_SCROLL
    (void) platform_device_register(&tegra_scrollwheel_device);
#endif


#ifdef CONFIG_TEGRA_ODM_RFKILL
    (void) platform_device_register(&tegra_rfkill);
#endif

#ifdef CONFIG_TEGRA_PCI
	tegra_set_voltage( NV_VDD_PEX_CLK_ODM_ID, 1);
#else
	tegra_set_voltage( NV_VDD_PEX_CLK_ODM_ID, 0);
#endif

	pm_power_off = tegra_system_power_off;
}

MACHINE_START(TEGRA_GENERIC, "Tegra generic")

    .boot_params  = 0x00000100,
    .map_io       = tegra_map_common_io,
    .init_irq     = tegra_init_irq,
    .init_machine = tegra_machine_init,
    .timer        = &tegra_timer,

MACHINE_END
