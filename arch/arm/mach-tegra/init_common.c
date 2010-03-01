/*
 * arch/arm/mach-tegra/init_common.c
 *
 * Miscellaneous driver registration routines for Tegra
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
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/dma-mapping.h>
#include <linux/tegra_devices.h>
#include "nvcommon.h"
#include "nvrm_init.h"
#include "nvrm_drf.h"
#include "mach/nvrm_linux.h"
#include "mach/platform.h"
#include "nvos.h"
#include "nvutil.h"
#include "nvassert.h"
#include "nvrm_hardware_access.h"
#include "ap15/arapb_misc.h"
#include "nvrm_module.h"
#include "nvodm_modules.h"
#include "nvodm_query.h"
#include "nvrm_arm_cp.h"
#include "nvrm_interrupt.h"
#include "ap20/arusb.h"

const char *tegra_partition_list = NULL;
char *tegra_boot_device = NULL;
NvRmGpioHandle s_hGpioGlobal = NULL;

#ifdef CONFIG_PM
#ifdef MACH_TEGRA_GENERIC_DEBUG
extern int console_suspend_enabled;
#endif
extern void tegra_set_suspend_ops(void);
#endif

/*
 * The format for the partition list command line parameter is
 * tagrapart=<linux_name>:<start_sector>:<length_in_sectors>:<sector_size>,...
 */
static int __init tegrapart_setup(char *options)
{
    if (options && *options && !tegra_partition_list)
        tegra_partition_list = options;

    return 0;
}

__setup("tegrapart=", tegrapart_setup);
static int __init tegraboot_setup(char *options)
{
    tegra_boot_device = options;
    return 0;
    
}
__setup("tegraboot=", tegraboot_setup);
int tegra_was_boot_device(const char *boot)
{
    if (!tegra_boot_device)
        return 0;

    if (!NvOsMemcmp(boot, tegra_boot_device, NvOsStrlen(boot)))
        return 1;

    return 0;
}

int tegra_get_partition_info_by_num(
    int    PartitionNum,
    char **pName,
    NvU64 *pSectorStart,
    NvU64 *pSectorLength,
    NvU32 *pSectorSize)
{
    const char *Ptr = tegra_partition_list;
    int Cnt = 0;
    int Len = 0;

    if (!Ptr)
        return -1;

    while (Cnt<PartitionNum && *Ptr && *Ptr!=' ')
    {
        if (*Ptr==',')
            Cnt++;
        Ptr++;
    }

    if (pName)
        *pName = NULL;

    if (Cnt==PartitionNum && *Ptr!=' ' && *Ptr)
    {
        char *End;
        NvU64 Temp;
        for (Len=0; Ptr[Len] && Ptr[Len]!=' ' && Ptr[Len]!=':'; Len++) { }
        if (pName)
        {
            *pName = NvOsAlloc(Len+1);
            if (!*pName)
                return -ENOMEM;
            (*pName)[Len] = 0;
            NvOsMemcpy(*pName, Ptr, Len);           
        }
        Ptr += Len+1;
        Temp = NvUStrtoull(Ptr, &End, 16);
        if (*End!=':')
            goto fail;
        if (pSectorStart)
            *pSectorStart = Temp;
        Ptr = End+1;
        Temp = NvUStrtoull(Ptr, &End, 16);
        if (*End!=':')
            goto fail;
        if (pSectorLength)
            *pSectorLength = Temp;
        Ptr = End+1;
        Temp = NvUStrtoull(Ptr, &End, 16);
        if (*End!=',' && *End!=' ' && *End)
            goto fail;
        if (pSectorSize)
            *pSectorSize = (NvU32)Temp;

        return 0;
    }

 fail:
    if (pName && *pName)
        NvOsFree(*pName);

    return -1;
}

int tegra_get_partition_info_by_name(
    const char *PartName,
    NvU64      *pSectorStart,
    NvU64      *pSectorLength,
    NvU32      *pSectorSize)
{
    int Len = NvOsStrlen(PartName);
    const char *Ptr = tegra_partition_list;
    char *End;

    if (!Ptr)
        return -1;

    while (*Ptr && *Ptr!=' ')
    {
        if (!NvOsStrncmp(Ptr, PartName, Len) && Ptr[Len]==':')
        {
            Ptr += Len + 1;
            *pSectorStart = NvUStrtoull(Ptr, &End, 16);
            if (*End!=':')
                return -1;
            Ptr = End+1;
            *pSectorLength = NvUStrtoull(Ptr, &End, 16);
            if (*End!=':')
                return -1;
            Ptr = End+1;
            *pSectorSize = NvUStrtoul(Ptr, &End, 16);
            if (*End!=',' && *End!=' ' && *End)
                return -1;
            return 0;
        }
        else
        {
            while (*Ptr != ',' && *Ptr)
                Ptr++;
            if (!*Ptr)
                return -1;
            Ptr++;
        }
    }
    return -1;
}


#if !defined(CONFIG_SPI_TEGRA)
#define tegra_register_spi() do {} while (0)
#else
static void __init tegra_register_spi(void)
{
    NvU32 Num, Cnt;
    const NvU32 *pPinMuxes;
    NvU32 NumPinMuxes;
    struct platform_device *pDev;
    const NvRmModuleID Modules[] = { NvRmModuleID_Slink, NvRmModuleID_Spi};
    const NvOdmIoModule OdmModules[] = 
        { NvOdmIoModule_Spi, NvOdmIoModule_Sflash };
    const NvOdmQuerySpiDeviceInfo *pSpiDeviceInfo;
    unsigned int i, j, k, ToAdd;
    struct tegra_spi_platform_data SpiData;

    for (i=0, Cnt=0; i<NV_ARRAY_SIZE(Modules); i++)
    {
        Num = NvRmModuleGetNumInstances(s_hRmGlobal, Modules[i]);
        NvOdmQueryPinMux(OdmModules[i], &pPinMuxes, &NumPinMuxes);
        for (j=0; j<Num && j<NumPinMuxes; j++)
        {
            if (!pPinMuxes[j])
                continue;

            pSpiDeviceInfo = NvOdmQuerySpiGetDeviceInfo(OdmModules[i], j, 0);
            if (pSpiDeviceInfo && pSpiDeviceInfo->IsSlave)
                continue;

            if (pPinMuxes[j]==NvOdmSpiPinMap_Multiplexed)
                ToAdd = (unsigned int)NvOdmSpiPinMap_Config6;
            else
                ToAdd = 1;

            for (k=0; k<ToAdd; k++)
            {
                pDev = platform_device_alloc("tegra_spi", Cnt);
                if (!pDev)
                    goto fail;
                Cnt++;
                
                SpiData.IoModuleID = OdmModules[i];
                SpiData.Instance = j;
                SpiData.PinMuxConfig = 
                    (ToAdd==1) ? 0 : (NvOdmSpiPinMap_Config1 + k);
                if (platform_device_add_data(pDev, &SpiData, sizeof(SpiData)))
                    goto fail;

                if (platform_device_add(pDev))
                    goto fail;
            }
        }
    }

    return;

 fail:
    if (pDev)
        platform_device_del(pDev);
    //  just stop adding new devices
}
#endif

#if !defined(CONFIG_W1_MASTER_TEGRA)
#define tegra_register_w1() do { } while (0)
#else
static void __init tegra_register_w1(void)
{
    const NvU32 *pPinMuxes;
    NvU32 NumPinMuxes, NumModules;
    struct platform_device *pDev;
    struct tegra_w1_platform_data W1Data;
    NvU32 i;

    NumModules = NvRmModuleGetNumInstances(s_hRmGlobal, NvOdmIoModule_OneWire);
    NvOdmQueryPinMux(NvOdmIoModule_OneWire, &pPinMuxes, &NumPinMuxes);

    for (i=0; i < NumModules && i < NumPinMuxes; i++)
    {
	if (!pPinMuxes[i])
	    continue;

	pDev = platform_device_alloc("tegra_w1", i);
	if (!pDev)
	    goto fail;
	W1Data.Instance = i;
	W1Data.PinMuxConfig = pPinMuxes[i];
	if (platform_device_add_data(pDev, &W1Data, sizeof(W1Data)))
	    goto fail;
	if (platform_device_add(pDev))
	    goto fail;
    }
    return;
fail:
    if (pDev)
	platform_device_del(pDev);
}
#endif

#if !defined(CONFIG_I2C_TEGRA)
#define tegra_register_i2c() do { } while (0)
#else
static int __init tegra_map_i2c_odm_to_bus(NvOdmIoModule Module,
    NvU32 Instance, NvU32 PinMuxConfig)
{
    NvOdmIoModule SearchModule;
    NvU32 SearchInstance;
    int cnt = 0;

    const NvU32 *pPinMuxes;
    NvU32 NumPinMuxes;

    //  FIXME:  Pinmux multiplexing is an ugly hack..  we instantiate
    //  "N" buses (hard-coded to the maximum number of I2C instance 2
    //  configurations), and assign peripherals to the bus ID which
    //  matches the requested pin mux.  Too much chip-specific stuff
    //  here, but pin-muxing has always been ugly...
    if (PinMuxConfig && (Module!=NvOdmIoModule_I2c || Instance!= 2 ||
                         PinMuxConfig > NvOdmI2cPinMap_Config4))
        return -1;

    for (SearchModule = NvOdmIoModule_I2c_Pmu;
         SearchModule >= NvOdmIoModule_I2c;
         SearchModule--)
    {
        NvOdmQueryPinMux(SearchModule, &pPinMuxes, &NumPinMuxes);
        for (SearchInstance = 0; SearchInstance < NumPinMuxes;
             SearchInstance++)
        {
            if (!pPinMuxes[SearchInstance])
                continue;

#ifdef CONFIG_NVEC
            /* nvec uses instance 0 as I2C slave */
            if (SearchModule == NvOdmIoModule_I2c &&
                    SearchInstance == 0)
            {
                continue;
            }
#endif
            if (SearchModule==Module && SearchInstance==Instance)
            {
                if (PinMuxConfig &&
                    pPinMuxes[Instance]!=NvOdmI2cPinMap_Multiplexed)
                    return -1;

                if (pPinMuxes[Instance]==NvOdmI2cPinMap_Multiplexed &&
                    !PinMuxConfig)
                    return -1;

                if (PinMuxConfig)
                    return cnt + (PinMuxConfig - NvOdmI2cPinMap_Config1);

                return cnt;
            }

            if (pPinMuxes[SearchInstance]==NvOdmI2cPinMap_Multiplexed)
                cnt += (int)NvOdmI2cPinMap_Config4;
            else
                cnt++;
        }
        
    }
    return -1;
}

static void __init tegra_register_i2c(void)
{
    NvU32 Num, Cnt;
    const NvU32 *pPinMuxes;
    NvU32 NumPinMuxes;
    struct platform_device *pDev;
    const NvRmModuleID Modules[] = { NvRmModuleID_Dvc, NvRmModuleID_I2c };
    const NvOdmIoModule OdmModules[] = { NvOdmIoModule_I2c_Pmu,
                                         NvOdmIoModule_I2c };
    unsigned int i, j, k, ToAdd;
    struct tegra_i2c_platform_data I2cData;
    

    for (i=0, Cnt=0; i<NV_ARRAY_SIZE(Modules); i++)
    {
        Num = NvRmModuleGetNumInstances(s_hRmGlobal, Modules[i]);
        NvOdmQueryPinMux(OdmModules[i], &pPinMuxes, &NumPinMuxes);
        /* nvec uses i2c instance 0 as I2C slave */
        for (j=0; j<Num && j<NumPinMuxes; j++)
        {
            if (!pPinMuxes[j])
                continue;

            if (pPinMuxes[j]==NvOdmI2cPinMap_Multiplexed)
                ToAdd = (unsigned int)NvOdmI2cPinMap_Config4;
            else
                ToAdd = 1;

            for (k=0; k<ToAdd; k++)
            {
                pDev = platform_device_alloc("tegra_i2c", Cnt);
                if (!pDev)
                    goto fail;
                Cnt++;
                
                I2cData.IoModuleID = OdmModules[i];
                I2cData.Instance = j;
                I2cData.PinMuxConfig = 
                    (ToAdd==1) ? 0 : (NvOdmI2cPinMap_Config1 + k);
                //  FIXME:  Always defaulting to 100KHz for now.
                I2cData.ClockInKHz = 100;
                if (platform_device_add_data(pDev, &I2cData, sizeof(I2cData)))
                    goto fail;

                if (platform_device_add(pDev))
                    goto fail;
            }
            
        }
    }

    return;

 fail:
    if (pDev)
        platform_device_del(pDev);
    //  just stop adding new devices
}
#endif

#if !(defined(CONFIG_SERIAL_TEGRA_DDK) || defined(CONFIG_SERIAL_TEGRA))
#define tegra_register_uart() do {} while (0)
#else

static u64 tegra_uart_dma_mask = DMA_32BIT_MASK;

void __init tegra_register_uart(void)
{
    struct platform_device *pDev = NULL;
    NvU32 NumberOfUarts;
    NvU32 i;
    NvOdmDebugConsole Console;
    NvU32 Port = ~0;
    const NvU32 *pPinMuxes;
    NvU32 NumPinMuxes;
    NvU32 Cnt = 0;
    
    Console = NvOdmQueryDebugConsole();
    NumberOfUarts = NvRmModuleGetNumInstances(s_hRmGlobal, 
        NvRmModuleID_Uart);

    NvOdmQueryPinMux(NvOdmIoModule_Uart, &pPinMuxes, &NumPinMuxes);

    /* Skip the UART port used as a debug console */
    if (((NvU32)Console >= (NvU32)NvOdmDebugConsole_UartA) &&
        ((NvU32)Console <= (NvU32)NvOdmDebugConsole_UartE))
    {
        Port = (NvU32)(Console - NvOdmDebugConsole_UartA);
    }

    for (i=0; i<NumberOfUarts && i<NumPinMuxes; i++)
    {
        if (i==Port || !pPinMuxes[i])
            continue;

        pDev = platform_device_alloc("tegra_uart", Cnt++);
        if (!pDev)
            goto fail;
        if (platform_device_add(pDev))
            goto fail;

        pDev->dev.coherent_dma_mask = ~0;
        pDev->dev.dma_mask = &tegra_uart_dma_mask;
    }
fail:
    if (pDev)
        return;
}
#endif

/* FIXME: Needed for filling the USB gadget device platform data structure */
#if !defined(CONFIG_USB_TEGRA)
#define tegra_register_usb_gadget() do {} while (0)
#define tegra_is_udc(p) 0
#else

#ifdef CONFIG_ARCH_TEGRA_1x_SOC
static inline unsigned long tegra_get_phy_base(void)
{
    NvRmPhysAddr pa;
    NvU32 len;

    NvU32 mod = NVRM_MODULE_ID(NvRmModuleID_Misc, 0);
    NvRmModuleGetBaseAddress(s_hRmGlobal, mod, &pa, &len);
    return (unsigned long)pa;
}
#else
#define tegra_get_phy_base() ~0UL
#endif

static u64 tegra_udc_dma_mask = DMA_32BIT_MASK;
static NvU32 tegra_udc_module_id = 0;

static void __init tegra_register_usb_gadget(void)
{
    struct fsl_usb2_platform_data pdata;
    struct platform_device *platdev = NULL;
    NvU32 port_count, i;

    port_count = NvRmModuleGetNumInstances(s_hRmGlobal, NvRmModuleID_Usb2Otg);

    for (i=0; i<port_count && !platdev; i++) {
        const NvOdmUsbProperty *p;
        struct resource *res;
        NvU32 mod = NVRM_MODULE_ID(NvRmModuleID_Usb2Otg, i);
        unsigned long phy_addr = tegra_get_phy_base();
        NvRmPhysAddr cont_addr;
        NvU32 len;
        NvU16 irq;

        /* fixme: add ulpi here? */
        p = NvOdmQueryGetUsbProperty(NvOdmIoModule_Usb, i);
        if (!p || !((p->UsbMode & NvOdmUsbModeType_Device) ||
            (p->UsbMode & NvOdmUsbModeType_OTG)))
            continue;

        irq = NvRmGetIrqForLogicalInterrupt(s_hRmGlobal, mod, 0);
        NvRmModuleGetBaseAddress(s_hRmGlobal, mod, &cont_addr, &len);
        if (irq==0xffff || !(~cont_addr) || !len)
            continue;

        platdev = platform_device_alloc("tegra-udc", i);
        if (!platdev) {
            pr_err("unable to allocate platform device for tegra-udc\n");
            goto fail;
        }

        res = kzalloc(sizeof(struct resource)*3, GFP_KERNEL);
        res[0].flags = IORESOURCE_MEM;
        res[0].start = cont_addr;
        res[0].end = cont_addr + len - 1;
        res[2].flags = IORESOURCE_IRQ;
        res[2].start = res[2].end = irq;
        if (phy_addr != ~0UL) {
            res[1].flags = IORESOURCE_MEM;
            res[1].start = phy_addr + APB_MISC_PP_USB_PHY_SELF_TEST_0;
            res[1].end = phy_addr + APB_MISC_PP_USB_PHY_ALT_VBUS_STS_0 + 4 - 1;
        }
        if (platform_device_add_resources(platdev, res, 3)) {
            pr_err("unable to add resources to tegra-udc device\n");
            goto fail;
        }
        memset(&pdata, 0, sizeof(pdata));
        /* FIXME: add support for ULPI and HSIC here */
        pdata.phy_mode = FSL_USB2_PHY_UTMI;
        pdata.operating_mode = FSL_USB2_DR_DEVICE;
        if (platform_device_add_data(platdev, &pdata, sizeof(pdata))) {
            pr_err("unable to add data to tegra-udc device\n");
            goto fail;
        }

        if (platform_device_add(platdev)) {
            pr_err("unable to add tegra-udc device\n");
            goto fail;
        }

        tegra_udc_module_id = mod;
        platdev->dev.coherent_dma_mask = ~0;
        platdev->dev.dma_mask = &tegra_udc_dma_mask;
    }
fail:
    ;
}
/* when both device & host are enabled w/o OTG, the first USB port specified
 * with device capabilities is registered as tegra-udc, the rest (if Host is also'
 * specified for the ports) are registered as tegra-ehci
 */
static inline int tegra_is_udc(NvU32 mod)
{
    return (mod==tegra_udc_module_id);
}
#endif  /* CONFIG_USB_TEGRA */

#if !defined(CONFIG_USB_TEGRA_HCD)
#define tegra_register_usb_host() do {} while (0)
#else
static void __init tegra_register_usb_host(void)
{
    NvU32 port_count, i;
    port_count = NvRmModuleGetNumInstances(s_hRmGlobal, NvRmModuleID_Usb2Otg);

    for (i=0; i<port_count; i++) {
        const NvOdmUsbProperty *p;
        struct resource *res;
        struct tegra_hcd_platform_data pdata;
        struct platform_device *platdev = NULL;

        p = NvOdmQueryGetUsbProperty(NvOdmIoModule_Usb, i);
        if (!p || (p->UsbMode & NvOdmUsbModeType_Device))
            continue;

        platdev = platform_device_alloc("tegra-ehci", i);
        if (!platdev) {
            pr_err("unable to allocate device for tegra-ehci\n");
            goto fail;
        }

        res = kzalloc(sizeof(struct resource)*2, GFP_KERNEL);
        if (!res) {
            pr_err("unable to allocate resource for tegra-otg\n");
            goto fail;
        }

        res[0].flags = IORESOURCE_MEM;
        res[0].start = tegra_get_module_inst_base("usbotg", i);
        res[0].end = res[0].start + tegra_get_module_inst_size("usbotg", i) - 1;
        res[1].flags = IORESOURCE_IRQ;
        res[1].start = res[1].end = tegra_get_module_inst_irq("usbotg", i, 0);

        if (platform_device_add_resources(platdev, res, 2)) {
            pr_err("unable to add resources to tegra-ehci device\n");
            goto fail;
        }

        memset(&pdata, 0, sizeof(pdata));
        pdata.instance = i;
        pdata.pUsbProperty = p;

        if (platform_device_add_data(platdev, &pdata, sizeof(pdata))) {
            pr_err("unable to add data to tegra-ehci device\n");
            goto fail;
        }

        if (platform_device_add(platdev)) {
            pr_err("unable to add tegra-ehci device\n");
            goto fail;
        }
    }
fail:
     ;
}
#endif

#if !defined(CONFIG_USB_TEGRA_OTG)
#define tegra_register_usb_otg() do {} while (0)
#else

static void __init tegra_register_usb_otg(void)
{
    NvU32 port_count, i;

    port_count = NvRmModuleGetNumInstances(s_hRmGlobal, NvRmModuleID_Usb2Otg);

    for (i=0; i<port_count; i++) {
        const NvOdmUsbProperty *p;
        struct platform_device *platdev = NULL;
        struct resource *res;
        struct tegra_otg_platform_data pdata;

        p = NvOdmQueryGetUsbProperty(NvOdmIoModule_Usb, i);
        if (!p || !(p->UsbMode & NvOdmUsbModeType_OTG))
            continue;

        platdev = platform_device_alloc("tegra-otg", i);
        if (!platdev) {
            pr_err("unable to allocate device for tegra-otg\n");
            goto fail;
        }

        res = kzalloc(sizeof(struct resource)*2, GFP_KERNEL);
        if (!res) {
            pr_err("unable to allocate resource for tegra-otg\n");
            goto fail;
        }

        res[0].flags = IORESOURCE_MEM;
        res[0].start = tegra_get_module_inst_base("usbotg", i);
        res[0].end = res[0].start + tegra_get_module_inst_size("usbotg", i) - 1;
        res[1].flags = IORESOURCE_IRQ;
        res[1].start = res[1].end = tegra_get_module_inst_irq("usbotg", i, 0);

        if (platform_device_add_resources(platdev, res, 2)) {
            pr_err("unable to add resources to tegra-otg device\n");
            goto fail;
        }

        memset(&pdata, 0, sizeof(pdata));
        pdata.instance = i;
        pdata.usb_property = p;

        if (platform_device_add_data(platdev, &pdata, sizeof(pdata))) {
            pr_err("unable to add platform data to tegra-otg device\n");
            goto fail;
        }

        if (platform_device_add(platdev)) {
            pr_err("unable to add tegra-otg device\n");
            goto fail;
        }
    }
fail:
    ;
}
#endif

void __init tegra_register_usb(void)
{
    tegra_register_usb_otg();
    tegra_register_usb_gadget();
    tegra_register_usb_host();
}

#define MAX_SDIO_INSTANCES  8

#if !(defined(CONFIG_MMC_SDHCI_TEGRA) || defined(CONFIG_MMC_TEGRA_SDIO))
#define tegra_register_sdio() do {} while (0)
#else
#ifdef CONFIG_MMC_SDHCI_TEGRA
#define tegra_register_sdio() tegra_register_sdio_int("tegra-sdhci")
#else
#define tegra_register_sdio() tegra_register_sdio_int("tegra-sdio")
#endif

#define register_sdio(_drv, _cnt, _boot, _comparison, _fail)            \
    do {                                                                \
        NvU32 i;                                                        \
        for (i=0; i<(_cnt); i++) {                                      \
            const NvOdmQuerySdioInterfaceProperty *prop;                \
            struct platform_device *platdev;                            \
            struct tegra_sdio_platform_data pdata;                      \
            if (i==(_boot))                                             \
                continue;                                               \
            prop = NvOdmQueryGetSdioInterfaceProperty(i);               \
            if (!prop || !_comparison(prop))                            \
                continue;                                               \
            platdev = platform_device_alloc((_drv), i);                 \
            pdata.StartOffset = 0;                                      \
            if (!platdev) {                                             \
                pr_err("error allocating SDIO devices\n");              \
                _fail;                                                  \
            }                                                           \
            if (platform_device_add_data(platdev, &pdata, sizeof(pdata))) { \
                pr_err("error adding platform data to SDIO devices\n"); \
                _fail;                                                  \
            }                                                           \
            if (platform_device_add(platdev)) {                         \
                pr_err("error adding SDIO platform devices\n");         \
                _fail;                                                  \
            }                                                           \
        }                                                               \
    } while (0)
            

#define is_media_slot(port) ((port)->usage == NvOdmQuerySdioSlotUsage_Media)

#define is_used_slot(port) ((port)->usage != NvOdmQuerySdioSlotUsage_unused)

#define is_sdio_slot(port) (!is_media_slot(port) && is_used_slot(port))

static void __init tegra_register_sdio_int(const char *driver_name)
{
    NvU32 port_count;
    NvU32 boot_id = (NvU32)-1;

    port_count = NvRmModuleGetNumInstances(s_hRmGlobal, 
        NvRmModuleID_Sdio);
    BUG_ON(port_count > MAX_SDIO_INSTANCES);

    /* register the boot device first, so that it always registers as
     * mmcblk0
     */
    if (tegra_boot_device && !memcmp(tegra_boot_device, "sdmmc", 5)) {
        struct platform_device *platdev;
        struct tegra_sdio_platform_data pdata;
        NvU64 start, length;
        NvU32 sector_size;
        boot_id = 3;
        tegra_get_partition_info_by_name("mbr", &start, &length, &sector_size);
        pdata.StartOffset = start * (NvU64)sector_size;
        platdev = platform_device_alloc(driver_name, boot_id);
        if (!platdev) {
            pr_err("unable to allocate device memory for SDIO boot device\n");
            goto fail;
        }
        if (platform_device_add_data(platdev, &pdata, sizeof(pdata))) {
            pr_err("failed to add data to SDIO boot device\n");
            goto fail;
        }
        if (platform_device_add(platdev)) {
            pr_err("failed to add SDIO boot device\n");
            goto fail;
        }
    }

    register_sdio(driver_name, port_count, boot_id, is_media_slot, goto fail);
    register_sdio(driver_name, port_count, boot_id, is_sdio_slot, goto fail);

fail:
    ;
}
#endif

#if !(defined(CONFIG_CACHE_PL3X0) || defined(CONFIG_CACHE_L2X0))
#define tegra_pl310_init() do {} while (0)
#else
#include <asm/hardware/cache-l2x0.h>
#include "ap20/arpl310.h"
static void __init tegra_pl310_init(void)
{
    NvRmPhysAddr CachePa;
    NvU32 Len;
    volatile NvU8 *pCache = NULL;
    NvU32 AuxValue =
        NV_DRF_NUM(PL310, AUXILIARY_CONTROL, FULL_LINE_OF_ZERO, 1) |
        NV_DRF_NUM(PL310, AUXILIARY_CONTROL, SO_DEV_HIGH_PRIORITY, 0) |
        /* FIXME:  Read performance tests show ~8-10% performance loss (uniprocessor
         * config) when L1/L2 exclusive operation is enabled for L1 miss / L2 hit heavy
         * tests, and up to 30% performance loss in some extreme cases.  Need to
         * understand this better before enabling this bit */
        NV_DRF_NUM(PL310, AUXILIARY_CONTROL, EXCLUSIVE, 0) |
        NV_DRF_DEF(PL310, AUXILIARY_CONTROL, WAY_SIZE, WAY_128KB) |
        NV_DRF_DEF(PL310, AUXILIARY_CONTROL, ASSOCIATIVITY, ASSOC_8) |
        NV_DRF_NUM(PL310, AUXILIARY_CONTROL, PARITY, 0) |
        NV_DRF_NUM(PL310, AUXILIARY_CONTROL, EVENT_MONITOR_BUS, 0) |
        NV_DRF_NUM(PL310, AUXILIARY_CONTROL, NON_SECURE_LOCKDOWN_WR, 1) |
        NV_DRF_NUM(PL310, AUXILIARY_CONTROL, NON_SECURE_INTERRUPT_ACCESS, 1) |
        NV_DRF_NUM(PL310, AUXILIARY_CONTROL, DATA_PREFETCH, 0) |
        NV_DRF_NUM(PL310, AUXILIARY_CONTROL, INSTRUCTION_PREFETCH, 1) | 
        NV_DRF_DEF(PL310, AUXILIARY_CONTROL, FORCE_WRITE_ALLOCATE, DISABLED) |
        NV_DRF_NUM(PL310, AUXILIARY_CONTROL, SHARED_ATTRIBUTE_OVERRIDE, 0) |
        NV_DRF_NUM(PL310, AUXILIARY_CONTROL, EARLY_BRESP, 1);
  
    if (!NvRmModuleGetNumInstances(s_hRmGlobal, NvRmPrivModuleID_Pl310))
        return;

    NvRmModuleGetBaseAddress(s_hRmGlobal,
        NVRM_MODULE_ID(NvRmPrivModuleID_Pl310, 0), &CachePa, &Len);

    if (NvRmPhysicalMemMap(CachePa, Len, NVOS_MEM_READ_WRITE,
            NvOsMemAttribute_Uncached, (void **)&pCache)!=NvSuccess)
    {
        printk(__FILE__ ":%d failed to map PL310\n", __LINE__);
        return;
    }

    NV_WRITE32(pCache + PL310_TAG_RAM_LATENCY_0,
        NV_DRF_DEF(PL310, TAG_RAM_LATENCY, SETUP, SW_DEFAULT) |
        NV_DRF_DEF(PL310, TAG_RAM_LATENCY, READ, SW_DEFAULT) |
        NV_DRF_DEF(PL310, TAG_RAM_LATENCY, WRITE, SW_DEFAULT));
    NV_WRITE32(pCache + PL310_DATA_RAM_LATENCY_0,
        NV_DRF_DEF(PL310, DATA_RAM_LATENCY, SETUP, SW_DEFAULT) |
        NV_DRF_DEF(PL310, DATA_RAM_LATENCY, READ, SW_DEFAULT) |
        NV_DRF_DEF(PL310, DATA_RAM_LATENCY, WRITE, SW_DEFAULT));

    if (NV_DRF_VAL(PL310, AUXILIARY_CONTROL, EXCLUSIVE, AuxValue) ||
        NV_DRF_VAL(PL310, AUXILIARY_CONTROL, FULL_LINE_OF_ZERO, AuxValue))
    {
        unsigned long flags;
        NvU32 Reg;
        local_irq_save(flags);
        MRC(p15, 0, Reg, c1, c0, 1);
        if (NV_DRF_VAL(PL310, AUXILIARY_CONTROL, EXCLUSIVE, AuxValue))
            Reg |= (1<<7);
        else
            Reg &= ~(1<<7);

        if (NV_DRF_VAL(PL310, AUXILIARY_CONTROL, FULL_LINE_OF_ZERO, AuxValue))
            Reg |= (1<<3);
        else
            Reg &= ~(1<<3);

        MCR(p15, 0, Reg, c1, c0, 1);
        local_irq_restore(flags);
    }
    l2x0_init((void __iomem *)pCache, AuxValue, 0x8200c3fe);    
}
#endif


#if defined(CONFIG_ARCH_TEGRA_1x_SOC)
extern void __init NvAp15InitFlowController(void);
#if defined(CONFIG_CACHE_TEGRA_CMC)
extern void __init tegra_cmc_enable(void);
#else
#define tegra_cmc_enable() do {} while (0)
#endif

static void __init tegra_init_cpu(void)
{
    tegra_cmc_enable();
    NvAp15InitFlowController();
}

#elif defined(CONFIG_ARCH_TEGRA_2x_SOC)
extern void __init NvAp20InitFlowController(void);

static void __init tegra_init_cpu(void)
{
    tegra_pl310_init();
    NvAp20InitFlowController();
}

#else
#error "Unrecognized Tegra SoC family"
#endif

#ifdef CONFIG_TEGRA_SYSTEM_DMA
extern int __init tegra_dma_init(void);
#else
#define tegra_dma_init() do {} while (0)
#endif

void __init tegra_common_init(void)
{
    NV_ASSERT_SUCCESS(NvRmOpen(&s_hRmGlobal,0));
    NV_ASSERT_SUCCESS(NvRmGpioOpen(s_hRmGlobal, &s_hGpioGlobal));

    tegra_init_cpu();
    tegra_dma_init();
    tegra_register_i2c();
    tegra_register_spi();
    tegra_register_uart();
    tegra_register_sdio();
    tegra_register_usb();
    tegra_register_w1();
#ifdef CONFIG_PM
#ifdef MACH_TEGRA_GENERIC_DEBUG
	/* This is needed to get prints on UART
	 * during suspend/resume */
	console_suspend_enabled = 0;
#endif
	tegra_set_suspend_ops();
#endif
}

