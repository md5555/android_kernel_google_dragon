/*
 * Copyright (c) 2008-2009 NVIDIA Corporation.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of the NVIDIA Corporation nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "nvos.h"
#include "nvrm_pcie.h"
#include "nvrm_module_private.h"
#include "ap20rm_pcie_private.h"
#include "nvrm_hardware_access.h"
#include "nvrm_hwintf.h"
#include "nvassert.h"
#include "nvrm_clocks.h"
#include "nvrm_interrupt.h"
#include "nvrm_processor.h"
#include "nvrm_memmgr.h"
#include "nvrm_memctrl.h"
#include "nvodm_pmu.h"
#include "nvodm_query_discovery.h"
#include "nvrm_pinmux.h"

/* PCIE driver state. 
 * FIXME move to a structure if needed.
 * */
/* static variables holding the configuration space of the 2 root port bridges */
static NvU32 s_pciConfig0[NV_PROJ__PCIE2_RP_ERPTCAP/4];   // 256 bytes
static NvU32 s_pciConfig1[NV_PROJ__PCIE2_RP_ERPTCAP/4];   // 256 bytes
static NvBool s_PcieRootPortPresent = NV_FALSE;
static NvBool s_PcieRootPort0Present = NV_FALSE;
static NvBool s_PcieRootPort1Present = NV_FALSE;

static NvU8 *s_pcieBase = NULL;
static NvRmMemHandle s_pcieMsiMemoryHandle = NULL; 
static NvU32 s_pcieSize = 0;
static NvU32 s_pcieMsiVectorCount = 0;
static NvRmPhysAddr s_pciePhysical = 0;
static NvRmPciDevice pciDevices[MAX_PCI_DEVICES];
static NvRmPciDevice *s_rp0 = NULL;
static NvU32 s_NumPciDevices = 0;
static NvOsInterruptHandle s_pcieInterruptHandle = NULL;
static NvOsInterruptHandle s_pcieMSIHandle = NULL;
static NvU32 s_PowerClientId;
#define ROOTPORT_0_BUS       0x00
#define ROOTPORT_0_SUBBUS    0x1f
#define ROOTPORT_1_BUS       (ROOTPORT_0_SUBBUS + 1)
#define ROOTPORT_1_SUBBUS    (ROOTPORT_1_BUS + ROOTPORT_0_SUBBUS - ROOTPORT_0_BUS) //same size as RP0

//#define SINGLE_PORT

//return the matching "device" entry.  
//assumes they've been loaded already
static NvRmPciDevice *
pcie_GetDevice(NvU32 function_device_bus);

static void 
pcie_ConfigureDeviceBAR(NvRmDeviceHandle rm, NvRmPciDevice *dev);

static
NvRmPciDevice *pcie_allocDevice(NvRmDeviceHandle rm);

static void
pcie_ConfigureMSI(NvRmDeviceHandle rm, NvRmPciDevice *device);

static 
void pcie_scanbus(NvRmDeviceHandle rm, NvRmPciDevice *dev_parent);

//interrupt handler for MSI interrupts
void NvRmPrivHandlePcieMSI(void *arg);

static
void pcie_businit(NvRmDeviceHandle rm);

static
void pcie_businitx(NvRmDeviceHandle rm, int rpindex);

static
void pcie_buswalk(NvRmDeviceHandle rm, NvRmPciDevice *dev);

static 
void pcie_readmultiple(NvU8 *dest, NvU8 *src, NvU32 len);

static
void NvRmPrivHandlePcieInterrupt(void *arg);

static
void pcie_ReadRPConfig(NvRmDeviceHandle hRm, NvU8 *data, NvU32 len, NvU32 offset, NvU32 controller_number);

static
void pcie_WriteRPConfig(NvRmDeviceHandle hRm, NvU8 *data, NvU32 len, NvU32 offset, NvU32 controller_number);
typedef NvError (*NvRmMSIHandler)(NvU8, void *);

#define MAX_MSI_HANDLERS 256
#define MSI_MEMORY_ALIGNMENT 16
#define MAX_LEGACY_HANDLERS 16

struct {
    NvOsSemaphoreHandle sem;
} static MSIHandlers[MAX_MSI_HANDLERS];

struct {
    NvOsSemaphoreHandle sem;
} static LegacyHandlers[MAX_LEGACY_HANDLERS];

static void pcie_readmultiple(NvU8 *dest, NvU8 *src, NvU32 len)
{
    NvU32 temp;

    while (((NvU32)src & 0x3) && len)
    {
        *dest++ = NV_READ8(src);
        src++;
        len--;
    }

    while (len/4)
    {
        NvU8 *tempPtr = (NvU8 *)&temp;

        temp = NV_READ32(src);
        /* Dest unaligned? */
        if (!((NvU32)dest & 0x3))
        {
            *(NvU32 *)dest = temp;
            dest += 4;
        } else
        {
            *dest++ = *tempPtr++;
            *dest++ = *tempPtr++;
            *dest++ = *tempPtr++;
            *dest++ = *tempPtr++;
        }
        src += 4;
        len -= 4;
    }

    while (len)
    {
        *dest++ = NV_READ8(src);
        src++;
        len--;
    }
}

static void pcie_writemultiple(NvU8 *dest, NvU8 *src, NvU32 len);
static void pcie_writemultiple(NvU8 *dest, NvU8 *src, NvU32 len)
{
    NvU32 temp;

    while (((NvU32)dest & 0x3) && len)
    {
        NV_WRITE08(dest, *src);
        src++;
        dest++;
        len--;
    }

    while (len/4)
    {
        NvU8 *tempPtr = (NvU8 *)&temp;
        /* Source unaligned? */
        if (!((NvU32)src & 0x3))
        {
            temp = *(NvU32 *)src;
            src += 4;
        } else
        {
            *tempPtr++ = *src++;
            *tempPtr++ = *src++;
            *tempPtr++ = *src++;
            *tempPtr++ = *src++;
        }
        NV_WRITE32(dest, temp);
        dest += 4;
        len -= 4;
    }

    while (len)
    {
        NV_WRITE08(dest, *src);
        dest++;
        src++;
        len--;
    }
}



static void 
pcie_regw(NvRmDeviceHandle hRm, PcieRegType type, NvU32 offset, NvU32 data);

static void 
pcie_regw(NvRmDeviceHandle hRm, PcieRegType type, NvU32 offset, NvU32 data)
{
    switch (type)
    {
        case PcieRegType_AFI:
            offset += NV_PCIE_AXI_AFI_REGS_OFSET;
            break;
        case PcieRegType_CFG0:
            offset += NV_PCIE_AXI_RP_T0C0_OFFSET;
            break;
        case PcieRegType_CFG1:
            offset += NV_PCIE_AXI_RP_T0C1_OFFSET;
            break;
        case PcieRegType_PADS:
            offset += NV_PCIE_AXI_PADS_OFSET;
            break;
        case PcieRegType_Force32:
        default:
            NV_ASSERT(0);
            return;
    }

    NV_ASSERT(s_pcieBase != NULL);
    NV_WRITE32(s_pcieBase + offset, data);
    return;
}

static NvU32 
pcie_regr(NvRmDeviceHandle hRm, PcieRegType type, NvU32 offset);

static NvU32 pcie_regr(NvRmDeviceHandle hRm, PcieRegType type, NvU32 offset)
{
    switch (type)
    {
        case PcieRegType_AFI:
            offset += NV_PCIE_AXI_AFI_REGS_OFSET;
            break;
        case PcieRegType_CFG0:
            offset += NV_PCIE_AXI_RP_T0C0_OFFSET;
            break;
        case PcieRegType_CFG1:
            offset += NV_PCIE_AXI_RP_T0C1_OFFSET;
            break;
        case PcieRegType_PADS:
            offset += NV_PCIE_AXI_PADS_OFSET;
            break;
        case PcieRegType_Force32:
        default:
            NV_ASSERT(0);
            return 0;
    }
    NV_ASSERT(s_pcieBase != NULL);
    return NV_READ32(s_pcieBase + offset);
}

NvError NvRmReadWriteConfigSpace( 
    NvRmDeviceHandle hDeviceHandle,
    NvU32 function_device_bus,
    NvRmPcieAccessType type,
    NvU32 offset,
    NvU8 *Data,
    NvU32 DataLen )
{
    NvBool extendedConfig;
    NvU32 addr;
    NvU32 bus;
    NvU32 device;
    NvU32 function;

    bus = function_device_bus & 0xff;
    device = (function_device_bus >> 8) & 0x1f;
    function = (function_device_bus >> 16) & 0x7;

    if (DataLen == 0)
        return NvSuccess;

    if (Data == NULL)
        return NvError_BadParameter;

    /* No device attached, so bailout */
    if (!s_PcieRootPortPresent)
        return NvError_DeviceNotFound;

    /* We don't support more than 256 devices*/
    NV_ASSERT((bus) < NVRM_PCIE_MAX_DEVICES);
    //if someone tries to access a bus outside of the range, they get an abort-crash
    //handle that more gracefully
    if(! (   (s_PcieRootPort0Present && (bus>ROOTPORT_0_BUS) && (bus <=ROOTPORT_0_SUBBUS)) 
          || (s_PcieRootPort1Present && (bus>ROOTPORT_1_BUS) && (bus <=ROOTPORT_1_SUBBUS)) ))
    {
        //handle accessing the root ports more gracefully
        if((bus==ROOTPORT_0_BUS) && s_PcieRootPort0Present && (device==0) && (function==0))
        {
            if(type==NvRmPcieAccessType_Read)
                pcie_ReadRPConfig(hDeviceHandle, Data, DataLen, offset, 0); 
            else
                pcie_WriteRPConfig(hDeviceHandle, Data, DataLen, offset, 0); 

            return NvSuccess;
        }
        if((bus==ROOTPORT_1_BUS) && s_PcieRootPort1Present && (device==0) && (function==0)) 
        {
            if(type==NvRmPcieAccessType_Read)
                pcie_ReadRPConfig(hDeviceHandle, Data, DataLen, offset, 1); 
            else
                pcie_WriteRPConfig(hDeviceHandle, Data, DataLen, offset, 1); 
            return NvSuccess;
        }

        return NvError_BadParameter;
    }   

    if (offset + DataLen < 256)
    {
        extendedConfig = NV_FALSE;
    } else if (offset + DataLen < 4096 )
    {
        extendedConfig = NV_TRUE;
    } else
    {
        NV_ASSERT(!"Illegal config access\n");
        return NvError_BadParameter;
    }

    /* Cannot straddle between basic config and extended config */
    if (offset < 256 && offset + DataLen > 256)
    {
        return NvError_BadParameter;
    }

    addr = (NvU32)s_pcieBase; 
    if (extendedConfig)
    {
        addr += NVRM_PCIE_EXTENDED_CONFIG_OFFSET;
    } else
    {
        addr += NVRM_PCIE_CONFIG_OFFSET;
    }

    /* 16:24 bits are interpreted as bus number. 
     * 11:15 bits are interpreted as device number
     * 8:11 bits are interpreted as function number */
    addr += bus << 16;
    addr += device << 11;
    addr += function << 8;
    addr += offset;

    if (type == NvRmPcieAccessType_Read)
    {
        pcie_readmultiple(Data, (NvU8 *)addr, DataLen);
    } else
    {
        NV_ASSERT(type == NvRmPcieAccessType_Write);
        pcie_writemultiple((NvU8 *)addr, Data, DataLen);
    }
    return NvSuccess;
}

static
void pcie_ReadRPConfig(NvRmDeviceHandle hRm, NvU8 *data, NvU32 len, NvU32 offset, NvU32 controller_numer)
{
    NvU8 *addr;

    addr = s_pcieBase;
    if (controller_numer == 0)
    {
        addr += NV_PCIE_AXI_RP_T0C0_OFFSET;
    } else if (controller_numer == 1)
    {
        addr += NV_PCIE_AXI_RP_T0C1_OFFSET;
    } else
    {
        NV_ASSERT(!"Only 2 controllers in AP20");
        return;
    }

    addr += offset;

    pcie_readmultiple(data, addr, len);
    return;
}

static
void pcie_WriteRPConfig(NvRmDeviceHandle hRm, NvU8 *data, NvU32 len, NvU32 offset, NvU32 controller_numer)
{
    NvU8 *addr;

    addr = s_pcieBase;
    if (controller_numer == 0)
    {
        addr += NV_PCIE_AXI_RP_T0C0_OFFSET;
    } else if (controller_numer == 1)
    {
        addr += NV_PCIE_AXI_RP_T0C1_OFFSET;
    } else
    {
        NV_ASSERT(!"Only 2 controllers in AP20");
        return;
    }

    addr += offset;

    pcie_writemultiple(addr, data, len);
    return;
}

static
void pcie_setupAfiAddressTranslations(NvRmDeviceHandle hRm);

static
void pcie_setupAfiAddressTranslations(NvRmDeviceHandle hRm)
{
    NvU32 fpci_bar;
    NvU32 size;
    NvU32 axi_address;
    NvU32 bar;
    NvU32 msi_base;

    /* Downstream address translations */

    /* Config Bar */
    bar = 0;
    fpci_bar = ((NvU32)0xfdff << 16);
    size = NVRM_PCIE_CONFIG_SIZE;
    axi_address = s_pciePhysical + NVRM_PCIE_CONFIG_SIZE;
    pcie_regw(hRm, PcieRegType_AFI, AFI_AXI_BAR0_START_0 + bar * 4, axi_address);
    pcie_regw(hRm, PcieRegType_AFI, AFI_AXI_BAR0_SZ_0 + bar * 4, size >> 12);
    pcie_regw(hRm, PcieRegType_AFI, AFI_FPCI_BAR0_0 + bar * 4, fpci_bar);

    /* Extended config Bar */
    bar = 1;
    fpci_bar = ((NvU32)0xfe1 << 20);
    size = NVRM_PCIE_EXTENDED_CONFIG_SIZE;
    axi_address = s_pciePhysical + NVRM_PCIE_EXTENDED_CONFIG_OFFSET;
    pcie_regw(hRm, PcieRegType_AFI, AFI_AXI_BAR0_START_0 + bar * 4, axi_address);
    pcie_regw(hRm, PcieRegType_AFI, AFI_AXI_BAR0_SZ_0 + bar * 4, size >> 12);
    pcie_regw(hRm, PcieRegType_AFI, AFI_FPCI_BAR0_0 + bar * 4, fpci_bar);

    /* Downstream IO bar */
    bar = 2;
    fpci_bar = ((NvU32)0xfdfc << 16);
    size = NVRM_PCIE_DOWNSTREAM_IO_SIZE;
    axi_address = s_pciePhysical + NVRM_PCIE_DOWNSTREAM_IO_OFFSET;
    pcie_regw(hRm, PcieRegType_AFI, AFI_AXI_BAR0_START_0 + bar * 4, axi_address);
    pcie_regw(hRm, PcieRegType_AFI, AFI_AXI_BAR0_SZ_0 + bar * 4, size >> 12);
    pcie_regw(hRm, PcieRegType_AFI, AFI_FPCI_BAR0_0 + bar * 4, fpci_bar);

    /* Pre-fetchable memory BAR */
    bar = 3;
    /* Bits 39:12 of 40 bit FPCI address goes to bits 31:4 */
    fpci_bar = (((FPCI_PREFETCH_MEMORY_OFFSET >> 12) & 0x0FFFFFFF) << 4);
    fpci_bar |= 0x1;
    size = NVRM_PCIE_PREFETCH_MEMORY_SIZE;
    axi_address = s_pciePhysical + NVRM_PCIE_PREFETCH_MEMORY_OFFSET;
    pcie_regw(hRm, PcieRegType_AFI, AFI_AXI_BAR0_START_0 + bar * 4, axi_address);
    pcie_regw(hRm, PcieRegType_AFI, AFI_AXI_BAR0_SZ_0 + bar * 4, size >> 12);
    pcie_regw(hRm, PcieRegType_AFI, AFI_FPCI_BAR0_0 + bar * 4, fpci_bar);

    /* Non pre-fetchable memory BAR */
    bar = 4;
    /* Bits 39:12 of 40 bit FPCI address goes to bits 31:4 */
    fpci_bar = (((FPCI_NON_PREFETCH_MEMORY_OFFSET >> 12) & 0x0FFFFFFF) << 4);
    fpci_bar |= 0x1;
    size = NVRM_PCIE_NON_PREFETCH_MEMORY_SIZE;
    axi_address = s_pciePhysical + NVRM_PCIE_NON_PREFETCH_MEMORY_OFFSET;
    pcie_regw(hRm, PcieRegType_AFI, AFI_AXI_BAR0_START_0 + bar * 4, axi_address);
    pcie_regw(hRm, PcieRegType_AFI, AFI_AXI_BAR0_SZ_0 + bar * 4, size >> 12);
    pcie_regw(hRm, PcieRegType_AFI, AFI_FPCI_BAR0_0 + bar * 4, fpci_bar);

    /* NULL out the remaining BAR as it is not used */
    fpci_bar = 0;
    size = 0;
    axi_address = 0;

    bar = 5;
    pcie_regw(hRm, PcieRegType_AFI, AFI_AXI_BAR0_START_0 + bar * 4, axi_address);
    pcie_regw(hRm, PcieRegType_AFI, AFI_AXI_BAR0_SZ_0 + bar * 4, size >> 12);
    pcie_regw(hRm, PcieRegType_AFI, AFI_FPCI_BAR0_0 + bar * 4, fpci_bar);

    /* Upstream address translations. Map the entire system memory as cached */
    pcie_regw(hRm, PcieRegType_AFI, AFI_CACHE_BAR0_ST_0, FPCI_SYSTEM_MEMORY_OFFSET);
    pcie_regw(hRm, PcieRegType_AFI, AFI_CACHE_BAR0_SZ_0, FPCI_SYSTEM_MEMORY_SIZE >> 12);

    /* Second cache bar is not used */
    pcie_regw(hRm, PcieRegType_AFI, AFI_CACHE_BAR1_ST_0, 0);
    pcie_regw(hRm, PcieRegType_AFI, AFI_CACHE_BAR1_SZ_0, 0);

    /* Map MSI bar */
    pcie_regw(hRm, PcieRegType_AFI, AFI_MSI_FPCI_BAR_ST_0, 0);
    pcie_regw(hRm, PcieRegType_AFI, AFI_MSI_BAR_SZ_0, 0);
    pcie_regw(hRm, PcieRegType_AFI, AFI_MSI_AXI_BAR_ST_0, 0);
    pcie_regw(hRm, PcieRegType_AFI, AFI_MSI_BAR_SZ_0, 0x00010000);
    msi_base = NvRmMemPin(s_pcieMsiMemoryHandle);
    pcie_regw(hRm, PcieRegType_AFI, AFI_MSI_AXI_BAR_ST_0, msi_base);
    pcie_regw(hRm, PcieRegType_AFI, AFI_MSI_FPCI_BAR_ST_0, msi_base);

    //enable all of the MSIs 
    pcie_regw(hRm, PcieRegType_AFI, AFI_MSI_EN_VEC0_0, 0xffffffff);
    pcie_regw(hRm, PcieRegType_AFI, AFI_MSI_EN_VEC1_0, 0xffffffff);
    pcie_regw(hRm, PcieRegType_AFI, AFI_MSI_EN_VEC2_0, 0xffffffff);
    pcie_regw(hRm, PcieRegType_AFI, AFI_MSI_EN_VEC3_0, 0xffffffff);
    pcie_regw(hRm, PcieRegType_AFI, AFI_MSI_EN_VEC4_0, 0xffffffff);
    pcie_regw(hRm, PcieRegType_AFI, AFI_MSI_EN_VEC5_0, 0xffffffff);
    pcie_regw(hRm, PcieRegType_AFI, AFI_MSI_EN_VEC6_0, 0xffffffff);
    pcie_regw(hRm, PcieRegType_AFI, AFI_MSI_EN_VEC7_0, 0xffffffff);

    return;
}

static void
NvRmPrivPciePowerControl(NvRmDeviceHandle hRm, NvBool bEnable)
{
    NvOdmServicesPmuHandle hPmu;
    NvU32 SettlingTime;
    NvOdmServicesPmuVddRailCapabilities RailCaps;
    NvU32 i;
    const NvOdmPeripheralConnectivity *pConnectivity = NULL;

    pConnectivity = NvOdmPeripheralGetGuid(NV_VDD_PEX_CLK_ODM_ID);
    if (pConnectivity == NULL)
        return;

    hPmu = NvOdmServicesPmuOpen();
    for (i = 0; i < pConnectivity->NumAddress; i++)
    {
        if (pConnectivity->AddressList[i].Interface != NvOdmIoModule_Vdd)
            continue;
        NvOdmServicesPmuGetCapabilities(hPmu, 
                pConnectivity->AddressList[i].Address, &RailCaps);

        if (bEnable)
        {
            NvOdmServicesPmuSetVoltage(hPmu, 
                    pConnectivity->AddressList[i].Address, 
                    RailCaps.requestMilliVolts, &SettlingTime);
        } else
        {
            NvOdmServicesPmuSetVoltage(hPmu, 
                    pConnectivity->AddressList[i].Address, 
                    NVODM_VOLTAGE_OFF, &SettlingTime);
        }
        if (SettlingTime)
        {
            NvOdmOsWaitUS(SettlingTime);
        }
    }
    NvOdmServicesPmuClose(hPmu);
}


static
NvError CheckPcieRPx(NvRmDeviceHandle hRm, int index)
{
    NvError retval=NvError_DeviceNotFound;
    NvU32 max_timeout=250;  //.25 seconds - it's either fast or it doesn't work at all
    NvU32 max_retries=2;
    NvU32 retry_count;
    NvU32 timeout;
    NvU32 data0, data1;

    //some local constants, depending on which RP this is
    const NvU32 pcieregtype_cfgx=    (index) ? PcieRegType_CFG1      : PcieRegType_CFG0;
    const NvU32 afi_pexx_ctrl_0 =    (index) ? AFI_PEX1_CTRL_0       : AFI_PEX0_CTRL_0;


    switch(index) {
        case 0:
        case 1:
            break;
        default:
            NV_ASSERT(!"bad index\n");
            retval=NvError_BadParameter;
            goto FAIL;
    }

    for(retry_count=0; retry_count<max_retries; retry_count++) {
        timeout=max_timeout;

        //if this isn't the first time through, we'll try resetting the port 
        //and hope it wakes up this time.
        if(retry_count > 0) {
            NvOsDebugPrintf("initialzing RP%d failed - resetting\n", index);
            //trigger the reset
            data0 = pcie_regr(hRm, PcieRegType_AFI, afi_pexx_ctrl_0);
            switch(index) {    
                case 0: data0 = NV_FLD_SET_DRF_NUM(AFI, PEX0_CTRL, PEX0_RST_L, 1, data0); break;
                case 1: data0 = NV_FLD_SET_DRF_NUM(AFI, PEX1_CTRL, PEX1_RST_L, 1, data0); break;
                default: NV_ASSERT(!"bad index"); goto FAIL;
            }
            pcie_regw(hRm, PcieRegType_AFI, afi_pexx_ctrl_0, data0);
            NvOsSleepMS(100);  
            switch(index) {    
                case 0: data0 = NV_FLD_SET_DRF_NUM(AFI, PEX0_CTRL, PEX0_RST_L, 0, data0); break;
                case 1: data0 = NV_FLD_SET_DRF_NUM(AFI, PEX1_CTRL, PEX1_RST_L, 0, data0); break;
                default: NV_ASSERT(!"bad index"); goto FAIL;
            }
            pcie_regw(hRm, PcieRegType_AFI, afi_pexx_ctrl_0, data0);

            //reenable pcie
            data0 = pcie_regr(hRm, PcieRegType_AFI, AFI_CONFIGURATION_0);
            data0 = data0 | AFI_CONFIGURATION_0_EN_FPCI_DEFAULT_MASK;
            pcie_regw(hRm, PcieRegType_AFI, AFI_CONFIGURATION_0, data0);
        }
                    
        data1 = NVPCIE_DRF_NUM(RP, VEND_XP, DL_UP, 1);
        data0 = pcie_regr(hRm, pcieregtype_cfgx, NV_PROJ__PCIE2_RP_VEND_XP);
        while (((data0 & data1) != data1) && timeout)
        {
            NvOsSleepMS(1);
            data0 = pcie_regr(hRm, pcieregtype_cfgx, NV_PROJ__PCIE2_RP_VEND_XP);
            timeout--;
        }
    
        if(!timeout) {
            NvOsDebugPrintf("PCIe RP %d DL timed out after %dms\n", index, max_timeout);
            continue;
        } 
    
        NvOsDebugPrintf("PCIe RP %d DL  took %d ms\n", index, (max_timeout - timeout));
    
        timeout=max_timeout;
        data1 = NVPCIE_DRF_NUM(RP, LINK_CONTROL_STATUS, LINKSTAT, 0x2000);
        data0 = pcie_regr(hRm, pcieregtype_cfgx, NV_PROJ__PCIE2_RP_LINK_CONTROL_STATUS);
        while (((data0 & data1) != data1) && timeout)
        {
            NvOsSleepMS(1);
            data0 = pcie_regr(hRm, pcieregtype_cfgx, NV_PROJ__PCIE2_RP_LINK_CONTROL_STATUS);
            timeout--;
            NvOsDebugPrintf("Link status not up...retrying..\n");
        }
        if(!timeout) {
            NvOsDebugPrintf("PCIe RP %d Link Control timed out after %dms\n", index, max_timeout);
            continue;
        } 
        
        NvOsDebugPrintf("PCIe RP %d Link Control took %dms\n", index, max_timeout-timeout);
    
        retval=NvSuccess;
        break;  //if we're here, the root port is up 
    }

FAIL:
    return retval;
}


/* should be called only once and by the RmOpen */
NvError NvRmPrivPcieOpen(NvRmDeviceHandle hRm)
{
    NvU32 data0, data1;
    NvError err = NvSuccess;
    ExecPlatform exec;
    void *pCaps;
    NvRmModuleCapability ModuleCaps[1];
    NvOsInterruptHandler hInt = NvRmPrivHandlePcieInterrupt;
    NvU32 irq;
    NvU32 i;

    exec = NvRmPrivGetExecPlatform(hRm);
    if ((exec != ExecPlatform_Soc) && (exec != ExecPlatform_Fpga))
    {
        /* PCIE driver is not supported on other platforms */
        return NvError_ModuleNotPresent;
    }

    NvRmPrivPciePowerControl(hRm, NV_TRUE);
    NV_ASSERT_SUCCESS( NvRmSetModuleTristate(hRm, 
                NVRM_MODULE_ID(NvRmPrivModuleID_Pcie, 0), NV_FALSE));

    ModuleCaps[0].MajorVersion = 1; // AP20
    ModuleCaps[0].MinorVersion = 0;
    ModuleCaps[0].EcoLevel = 0;
    ModuleCaps[0].Capability = NULL; 
    // for now using null caps as this call is made to find the presence of the module.

    err = NvRmModuleGetCapabilities(hRm, NVRM_MODULE_ID(NvRmPrivModuleID_Pcie, 0), 
            ModuleCaps, 1, (void **)&pCaps);
    if (err != NvSuccess)
        return err;

    NvRmModuleGetBaseAddress(hRm, NvRmPrivModuleID_Pcie, &s_pciePhysical, &s_pcieSize);
    if (s_pciePhysical == 0)
        return NvError_ModuleNotPresent; 

    s_pcieMsiVectorCount=0;

    for(i=0;i<MAX_MSI_HANDLERS;i++) {
        MSIHandlers[i].sem=NULL;
    }

    /* Only map the 3 sub-apertures instead of the entire 1GB aperture. */
    s_pcieSize = NVRM_PCIE_EXTENDED_CONFIG_SIZE + NVRM_PCIE_CONFIG_SIZE + NVRM_PCIE_REGISTER_APERTURE_SIZE;

    err = NvRmPhysicalMemMap(s_pciePhysical,
        s_pcieSize,
        NVOS_MEM_READ_WRITE,
        NvOsMemAttribute_Uncached,
        (void** )&s_pcieBase); 
    if (err != NvSuccess)
    {
        goto fail;
    }

    /* Start PCIE refclock (enable PLLE) */
    if (exec == ExecPlatform_Soc)
    {
        if (NvRmPowerRegister(hRm, 0, &s_PowerClientId) != NvSuccess)
            goto fail;
        if (NvRmPowerModuleClockControl(hRm, NvRmPrivModuleID_Pcie, 
                    s_PowerClientId, NV_TRUE) != NvSuccess)
        {
            goto fail;
        }
    }

    /* Pulse the reset to AFI and PCIe and keep the PCIXCLK in reset untill the
     * AFI and PCIE are configured */
    //NvRmModuleReset(hRm, NvRmPrivModuleID_Afi);
    //NvRmModuleReset(hRm, NvRmPrivModuleID_Pcie);
    //NvRmPrivModuleReset(hRm, NvRmPrivModuleID_PcieXclk, NV_TRUE);
    NvRmModuleResetWithHold(hRm, NvRmPrivModuleID_Afi, NV_FALSE);
    NvRmModuleResetWithHold(hRm, NvRmPrivModuleID_Pcie, NV_FALSE);
    NvRmModuleResetWithHold(hRm, NvRmPrivModuleID_PcieXclk, NV_TRUE);


    /* Enable slot clock and pulse external reset signal */
    data0 = pcie_regr(hRm, PcieRegType_AFI, AFI_PEX0_CTRL_0);
    data0 = NV_FLD_SET_DRF_NUM(AFI, PEX0_CTRL, PEX0_REFCLK_EN, 1, data0);
    pcie_regw(hRm, PcieRegType_AFI, AFI_PEX0_CTRL_0, data0);

    data1 = pcie_regr(hRm, PcieRegType_AFI, AFI_PEX1_CTRL_0);
    data1 = NV_FLD_SET_DRF_NUM(AFI, PEX1_CTRL, PEX1_REFCLK_EN, 1, data1);
    pcie_regw(hRm, PcieRegType_AFI, AFI_PEX1_CTRL_0, data1);
    data0 = NV_FLD_SET_DRF_NUM(AFI, PEX0_CTRL, PEX0_RST_L, 0, data0);
    data1 = NV_FLD_SET_DRF_NUM(AFI, PEX1_CTRL, PEX1_RST_L, 0, data1);
    pcie_regw(hRm, PcieRegType_AFI, AFI_PEX0_CTRL_0, data0);
    pcie_regw(hRm, PcieRegType_AFI, AFI_PEX1_CTRL_0, data1);
    /* FIXME it seems that the PCIe devices need this much delay! */
    NvOsSleepMS(100);   

    data0 = pcie_regr(hRm, PcieRegType_AFI, AFI_PEX0_CTRL_0);
    data0 = NV_FLD_SET_DRF_NUM(AFI, PEX0_CTRL, PEX0_RST_L, 1, data0);
    pcie_regw(hRm, PcieRegType_AFI, AFI_PEX0_CTRL_0, data0);
#ifndef SINGLE_PORT
    data1 = pcie_regr(hRm, PcieRegType_AFI, AFI_PEX1_CTRL_0);
    data1 = NV_FLD_SET_DRF_NUM(AFI, PEX0_CTRL, PEX0_RST_L, 1, data1);
    pcie_regw(hRm, PcieRegType_AFI, AFI_PEX1_CTRL_0, data1);
#endif

    // Verify the controller DEVIDs
    data0 = pcie_regr(hRm, PcieRegType_CFG0, NV_PROJ__PCIE2_RP_DEV_ID);
    if ((NVPCIE_DRF_VAL(RP, DEV_ID, VENDOR_ID, data0)) != NV_PROJ__PCIE2_RP_DEV_ID_VENDOR_ID_NVIDIA)
    {
        NV_ASSERT(!"Something broken cannot read PCIE root port 0 DevID register, check the clcoks to the module");
    }
#ifndef SINGLE_PORT

    data0 = pcie_regr(hRm, PcieRegType_CFG1, NV_PROJ__PCIE2_RP_DEV_ID);
    if ((NVPCIE_DRF_VAL(RP, DEV_ID, VENDOR_ID, data0)) != NV_PROJ__PCIE2_RP_DEV_ID_VENDOR_ID_NVIDIA)
    {
        NV_ASSERT(!"Something broken cannot read PCIE root port 1 DevID register, check the clcoks to the module");
    }
#endif

    /* Enable dual controller and both ports*/
    data0 = pcie_regr(hRm, PcieRegType_AFI, AFI_PCIE_CONFIG_0);
    data0 = NV_FLD_SET_DRF_NUM(AFI, PCIE_CONFIG, PCIEC0_DISABLE_DEVICE, 0, data0); 
#ifndef SINGLE_PORT 
    data0 = NV_FLD_SET_DRF_NUM(AFI, PCIE_CONFIG, SM2TMS0_XBAR_CONFIG, 1, data0);
    data0 = NV_FLD_SET_DRF_NUM(AFI, PCIE_CONFIG, PCIEC1_DISABLE_DEVICE, 0, data0); 
#endif
    pcie_regw(hRm, PcieRegType_AFI, AFI_PCIE_CONFIG_0, data0);

    /* This is for FPGA only, as it has seperate PHY */
    if (exec == ExecPlatform_Fpga)
    {
        //FIXME - should be different for single port
        pcie_regw(hRm, PcieRegType_AFI, AFI_WR_SCRATCH_0, 0x2020);
        pcie_regw(hRm, PcieRegType_AFI, AFI_WR_SCRATCH_0, 0x0);
        NvOsSleepMS(100);
        pcie_regw(hRm, PcieRegType_AFI, AFI_WR_SCRATCH_0, 0x2020);
        // CHECK PE0 PRESENT. Bit 3 will set to 1 when the the cable is present.
        do 
        {
            data0 = pcie_regr(hRm, PcieRegType_AFI, AFI_RD_SCRATCH_0);
        } while ((data0 & 0x8) != 0x8);
    } else 
    {
        /* Initialze AP20 internal PHY */
        //ENABLE up to 16 PCIE lanes
        pcie_regw(hRm, PcieRegType_PADS, NV_PROJ__PCIE2_PADS_CTL_SEL_1, 0x0);

        //override IDDQ to 1 on all 4 lanes
        data0 = pcie_regr(hRm, PcieRegType_PADS, NV_PROJ__PCIE2_PADS_CTL_1);
        data0 = NVPCIE_FLD_SET_DRF_NUM(PADS, CTL_1, IDDQ_1L, 1, data0);
        pcie_regw(hRm, PcieRegType_PADS, NV_PROJ__PCIE2_PADS_CTL_1, data0); 

        //set up PHY PLL inputs select PLLE output as refclock
        data0 = pcie_regr(hRm, PcieRegType_PADS, NV_PROJ__PCIE2_PADS_PLL_CTL1);
        data0 = NVPCIE_FLD_SET_DRF_NUM(PADS, PLL_CTL1, PLL_REFCLK_SEL, 
                NV_PROJ__PCIE2_PADS_PLL_CTL1_PLL_REFCLK_SEL_INTERNAL_CML, data0);

        //set TX ref sel to div10 (not div5)
        data0 = NVPCIE_FLD_SET_DRF_NUM(PADS, PLL_CTL1, PLL_TXCLKREF_SEL, 
                NV_PROJ__PCIE2_PADS_PLL_CTL1_PLL_TXCLKREF_SEL_DIV10, data0);
        pcie_regw(hRm, PcieRegType_PADS, NV_PROJ__PCIE2_PADS_PLL_CTL1, data0); 

        //take PLL out of reset 
        data0 = pcie_regr(hRm, PcieRegType_PADS, NV_PROJ__PCIE2_PADS_PLL_CTL1);
        data0 = NVPCIE_FLD_SET_DRF_NUM(PADS, PLL_CTL1, PLL_RST_B4SM, 
                NV_PROJ__PCIE2_PADS_PLL_CTL1_PLL_RST_B4SM_DEASSERT, data0);
        pcie_regw(hRm, PcieRegType_PADS, NV_PROJ__PCIE2_PADS_PLL_CTL1, data0); 

        //set the clock voltage to MCP default
        //this register isn't in documentation!
        //but it works and is necessary
        data0=0xFA5CFA5C;
        pcie_regw(hRm,PcieRegType_PADS, 0xc8, data0);

        //check PLL is locked 
        data1 = 0;
        data1 = NVPCIE_FLD_SET_DRF_NUM(PADS, PLL_CTL1, PLL_LOCKDET, 
                NV_PROJ__PCIE2_PADS_PLL_CTL1_PLL_LOCKDET_LOCKED, data1);
        data0 = pcie_regr(hRm, PcieRegType_PADS, NV_PROJ__PCIE2_PADS_PLL_CTL1);
        while ((data0 & data1) != data1) 
        {
            //wait for PLL to lock
            data0 = pcie_regr(hRm, PcieRegType_PADS, NV_PROJ__PCIE2_PADS_PLL_CTL1);
        }  

        //turn off IDDQ override
        data0 = pcie_regr(hRm, PcieRegType_PADS, NV_PROJ__PCIE2_PADS_CTL_1);
        data0 = NVPCIE_FLD_SET_DRF_NUM(PADS, CTL_1, IDDQ_1L, 0, data0);
        pcie_regw(hRm, PcieRegType_PADS, NV_PROJ__PCIE2_PADS_CTL_1, data0); 

        //ENABLE TX/RX data
        data0 = pcie_regr(hRm, PcieRegType_PADS, NV_PROJ__PCIE2_PADS_CTL_1);
        data0 = NVPCIE_FLD_SET_DRF_NUM(PADS, CTL_1, TX_DATA_EN_1L, 
                NV_PROJ__PCIE2_PADS_CTL_1_TX_DATA_EN_1L_ENABLE, data0);
        data0 = NVPCIE_FLD_SET_DRF_NUM(PADS, CTL_1, RX_DATA_EN_1L, 
                NV_PROJ__PCIE2_PADS_CTL_1_RX_DATA_EN_1L_ENABLE, data0);
        pcie_regw(hRm, PcieRegType_PADS, NV_PROJ__PCIE2_PADS_CTL_1, data0); 
    } 

    irq = NvRmGetIrqForLogicalInterrupt(hRm, NVRM_MODULE_ID(NvRmPrivModuleID_Pcie, 0), 0);
    err = NvRmInterruptRegister( hRm, 1, &irq, &hInt, hRm,
            &s_pcieInterruptHandle, NV_TRUE );
    if (err != NvSuccess)
    {
        goto fail;
    }

    irq = NvRmGetIrqForLogicalInterrupt(hRm, NVRM_MODULE_ID(NvRmPrivModuleID_Pcie, 0), 1);
    hInt= NvRmPrivHandlePcieMSI;

    err = NvRmMemHandleCreate( hRm, &s_pcieMsiMemoryHandle, MAX_MSI_HANDLERS * sizeof(NvU32));
    if( err != NvSuccess )
    {
        goto fail;
    }
    err = NvRmMemAlloc(s_pcieMsiMemoryHandle, NULL, 0, MSI_MEMORY_ALIGNMENT, 
            NvOsMemAttribute_Uncached);
    if( err != NvSuccess )
    {
        goto fail;
    }

    err = NvRmInterruptRegister( hRm, 1, &irq, &hInt, hRm,
            &s_pcieMSIHandle, NV_TRUE );
    if (err != NvSuccess)
    {
        goto fail;
    }

    /* setup the AFI address translations */
    pcie_setupAfiAddressTranslations(hRm);

    /* Take the PCIe interface module out of reset to start the PCIe training
     * sequence */
    NvRmModuleResetWithHold(hRm, NvRmPrivModuleID_PcieXclk, NV_FALSE);

    /* Enable PCIE */
    data0 = pcie_regr(hRm, PcieRegType_AFI, AFI_CONFIGURATION_0);
    data0 = data0 | AFI_CONFIGURATION_0_EN_FPCI_DEFAULT_MASK;
    pcie_regw(hRm, PcieRegType_AFI, AFI_CONFIGURATION_0, data0);

    /* Assume no device */
    s_PcieRootPort0Present = NV_FALSE;
    s_PcieRootPort1Present = NV_FALSE;
    s_PcieRootPortPresent = NV_FALSE;

    if(CheckPcieRPx(hRm, 0)==NvSuccess)
        s_PcieRootPort0Present=NV_TRUE;
#ifndef SINGLE_PORT
    if(CheckPcieRPx(hRm, 1)==NvSuccess)
        s_PcieRootPort1Present=NV_TRUE;
#endif

    if(s_PcieRootPort0Present || s_PcieRootPort1Present)
        s_PcieRootPortPresent=NV_TRUE;
    else  {
        NvOsDebugPrintf("PCIe link failure on both root ports!\n");
        err = NvError_DeviceNotFound;
        goto fail;
    }

#ifdef SINGLE_PORT
    data0=pcie_regr(hRm, PcieRegType_CFG0, NV_PROJ__PCIE2_RP_MISC0);
    data0=NVPCIE_FLD_SET_DRF_NUM(RP, MISC0, ENABLE_CLUMPING, 1, data0); 
    pcie_regw(hRm, PcieRegType_CFG0, NV_PROJ__PCIE2_RP_MISC0, data0);

#else
    //if both ports are up, set up SLI mode too
    //if(s_PcieRootPort0Present && s_PcieRootPort1Present) {
    if(0){
        //RP0
        data0 = pcie_regr(hRm, PcieRegType_CFG0, NV_PROJ__PCIE2_RP_MISC0);
        data0 = NVPCIE_FLD_SET_DRF_NUM(RP, MISC0, NATIVE_P2P_ENABLE, 1, data0);
        pcie_regw(hRm, PcieRegType_CFG0, NV_PROJ__PCIE2_RP_MISC0, data0);
        //RP1
        data0 = pcie_regr(hRm, PcieRegType_CFG1, NV_PROJ__PCIE2_RP_MISC0);
        data0 = NVPCIE_FLD_SET_DRF_NUM(RP, MISC0, NATIVE_P2P_ENABLE, 1, data0);
        pcie_regw(hRm, PcieRegType_CFG1, NV_PROJ__PCIE2_RP_MISC0, data0);

        data0 = pcie_regr(hRm, PcieRegType_AFI, AFI_FUSE_0);
        data0 = NV_FLD_SET_DRF_NUM(AFI, FUSE, FUSE_PCIE_SLI_DIS, 0, data0);
        pcie_regw(hRm, PcieRegType_AFI, AFI_FUSE_0, data0);
    }
#endif
    /* Enable PCIe interrupts */
    data0 = 0;
    data0 |= NV_DRF_NUM(AFI, AFI_INTR_ENABLE, EN_INI_SLVERR, 1);
    data0 |= NV_DRF_NUM(AFI, AFI_INTR_ENABLE, EN_INI_DECERR, 1);
    data0 |= NV_DRF_NUM(AFI, AFI_INTR_ENABLE, EN_TGT_SLVERR, 1);
    data0 |= NV_DRF_NUM(AFI, AFI_INTR_ENABLE, EN_TGT_DECERR, 1);
    data0 |= NV_DRF_NUM(AFI, AFI_INTR_ENABLE, EN_TGT_WRERR, 1);
    data0 |= NV_DRF_NUM(AFI, AFI_INTR_ENABLE, EN_DFPCI_DECERR, 1);
    pcie_regw(hRm, PcieRegType_AFI, AFI_AFI_INTR_ENABLE_0, data0);
    pcie_regw(hRm, PcieRegType_AFI, AFI_SM_INTR_ENABLE_0, 0xffffffff);
    data0 = 0;
    data0 |= NV_DRF_NUM(AFI, INTR_MASK, INT_MASK, 1);
    data0 |= NV_DRF_NUM(AFI, INTR_MASK, MSI_MASK, 1);
    pcie_regw(hRm, PcieRegType_AFI, AFI_INTR_MASK_0, data0);



    /* set the PCI to generate secure upstream transactions */
    data0 = 0;
    /* Bit 0 = I/D access bit
     * Bit 1 = NS bit
     * Bit 2 = Normal/Privileged bit
     * */
    data0 |= NV_DRF_NUM(AFI, APROT_OVERRIDE, APROT_OVERRIDE_VAL, 0x0);
    data0 |= NV_DRF_NUM(AFI, APROT_OVERRIDE, APROT_OVERRIDE_EN, 0x1);
    //pcie_regw(hRm, PcieRegType_AFI, AFI_APROT_OVERRIDE_0, data0);

    pcie_businit(hRm);
    NvOsDebugPrintf("PCIe bus: Found %d devices/bridges on the pci bus\n", s_NumPciDevices);
    if (NV_DEBUG)
    {
        pcie_buswalk(hRm, s_rp0);
    }

    /* Disable masksing off the aborts as the enumeration is done. From now on,
     * any illegal access will cause data abort.
     * 
     * Illegal access can be invalid bus segment, invalid pcie aperture
     */
    data0 = 0;
    data0 |= NV_DRF_NUM(AFI, FPCI_ERROR_MASKS, MASK_FPCI_TARGET_ABORT, 1);
    data0 |= NV_DRF_NUM(AFI, FPCI_ERROR_MASKS, MASK_FPCI_DATA_ERROR, 1);
    data0 |= NV_DRF_NUM(AFI, FPCI_ERROR_MASKS, MASK_FPCI_MASTER_ABORT, 1);
    pcie_regw(hRm, PcieRegType_AFI, AFI_FPCI_ERROR_MASKS_0, data0);

    return NvSuccess;

fail:
    if (s_pcieInterruptHandle != NULL)
    {
        NvRmInterruptUnregister(hRm, s_pcieInterruptHandle);
    }
    NvRmPhysicalMemUnmap(s_pcieBase, s_pcieSize);
    if (s_pcieMsiMemoryHandle)
    {
        NvRmMemUnpin(s_pcieMsiMemoryHandle);
        NvRmMemHandleFree(s_pcieMsiMemoryHandle);
        s_pcieMsiMemoryHandle = NULL;
    }
    NvRmPrivPciePowerControl(hRm, NV_FALSE);
    NV_ASSERT_SUCCESS( NvRmSetModuleTristate(hRm, 
                NVRM_MODULE_ID(NvRmPrivModuleID_Pcie, 0), NV_TRUE));
    return err;
}

void NvRmPrivPcieClose(NvRmDeviceHandle hDeviceHandle)
{
    if (s_pcieInterruptHandle != NULL)
    {
        NvRmInterruptUnregister(hDeviceHandle, s_pcieInterruptHandle);
    }
    NvRmPowerModuleClockControl(hDeviceHandle, NvRmPrivModuleID_Pcie, 
            s_PowerClientId, NV_FALSE);
    NvRmPowerUnRegister(hDeviceHandle, s_PowerClientId);
    NvRmPhysicalMemUnmap(s_pcieBase, s_pcieSize);
    s_pcieSize = 0;
    s_pcieBase = 0;
    s_pciePhysical = 0;
    if (s_pcieMsiMemoryHandle)
    {
        NvRmMemUnpin(s_pcieMsiMemoryHandle);
        NvRmMemHandleFree(s_pcieMsiMemoryHandle);
        s_pcieMsiMemoryHandle = NULL;
    }
    NvRmPrivPciePowerControl(hDeviceHandle, NV_FALSE);
    NV_ASSERT_SUCCESS( NvRmSetModuleTristate(hDeviceHandle, 
                NVRM_MODULE_ID(NvRmPrivModuleID_Pcie, 0), NV_TRUE));
    return;
}


static
NvRmPciDevice *pcie_allocDevice(NvRmDeviceHandle rm)
{
    static NvU32 index = 0;
    NvRmPciDevice *dev;

    if (index == 0)
    {
        NvOsMemset(pciDevices, 0, sizeof(pciDevices));
    }

    dev = &pciDevices[index];
    index++;
    s_NumPciDevices = index;
    return dev;
}


static NvRmPciDevice *pcie_RecursiveSearch(NvRmPciDevice *device, NvU32 function_device_bus);

static NvRmPciDevice *
pcie_RecursiveSearch(NvRmPciDevice *root, NvU32 function_device_bus)
{
    NvRmPciDevice *device = 0;

    if (root) 
    { 
        if (root->bus == function_device_bus)  
        {
            return root;
        }  
        device = pcie_RecursiveSearch(root->next, function_device_bus);
        if (device)
            return device;
    
        return pcie_RecursiveSearch(root->child, function_device_bus);
    }
    return device;
}


static NvRmPciDevice *
pcie_GetDevice(NvU32 function_device_bus)
{
    return pcie_RecursiveSearch(s_rp0, function_device_bus);
}

static void 
pcie_ConfigureDeviceBAR(NvRmDeviceHandle rm, NvRmPciDevice *dev)
{
    NvU32 size;
    NvU8 flags;
    NvRmPciResource r;
    NvU32 bar_index;
    NvU32 addr;
    NvU32 control;
    
    NvU32 io_base = dev->io_base;
    NvU32 mem_base = dev->mem_base;
    NvU32 prefetch_base = dev->prefetch_base;


    for (bar_index = 0x0; bar_index  < 6; bar_index ++)
    {
        NvBool valid = NV_FALSE;
        size = 0xFFFFFFFF;
        (void)NvRmReadWriteConfigSpace(rm, dev->bus, NvRmPcieAccessType_Write, 
                bar_index * 4+ 0x10, (NvU8 *)&size, 4);
        (void)NvRmReadWriteConfigSpace(rm, dev->bus, NvRmPcieAccessType_Read, 
                bar_index * 4 + 0x10, (NvU8 *)&size, 4);

        if (size == 0xffffffff) continue;
        if (size == 0) continue;  //some devices are broken
        flags = (size & 0x000f);

        /* Size align the addr and write that BAR offset */
        if (flags & 0x1)
        {
            size &= ~0xF;   // Ignore the last 4 bits
            size |= 0xffff0000; //some devices hardwire the high bits of IO bars to 0
            size = ~size;   // Do the 1's complement
            size += 1;      // Add 1 to get the final size of the BAR.

            r = NvRmPciResource_Io;
            //IO spaces are at most 256 bytes, and sometimes
            //  they hardwire the upper 16 bits in the bar to 0 
            addr = io_base;
            addr += (size-1);
            addr &= ~(size-1);
            if((addr + size)> NVRM_PCIE_DOWNSTREAM_IO_SIZE) 
            {
                valid = NV_FALSE ;
                NvOsDebugPrintf("Warning: could not allocate I/O mem for device %02x bar %d\n", dev->bus, bar_index);
            }
            else 
            {
                io_base = addr + size;
                valid = NV_TRUE; 
            }

            /* FIXME assert if there is more space needed */

        } else
        {
            size &= ~0xF;   // Ignore the last 4 bits
            size = ~size;   // Do the 1's complement
            size += 1;      // Add 1 to get the final size of the BAR.

            if (flags & 0x08)
            {
                r = NvRmPciResource_PrefetchMemory;
                addr = prefetch_base;
                addr += (size-1);
                addr &= ~(size-1);

                // make sure we have memory in the prefetchable memory space available for this 
                // we hit this case with 2xGT218 (they need 256MB for bar 1, and another ~32MB for bar 3)
                if((addr + size) <= (dev->prefetch_max)) {
                    valid = NV_TRUE;
                    prefetch_base = addr + size;
                } else {
                    NvOsDebugPrintf("Warning: could not allocate prefetchable memory for device %02x bar %d\n", dev->bus, bar_index);
                    valid = NV_FALSE;
                } 

            } else
            {
                r = NvRmPciResource_NonPrefetchMemory;
                addr = mem_base;
                addr += (size-1);
                addr &= ~(size-1);

                if((addr+size) <= (dev->mem_max)) {
                    valid = NV_TRUE;
                    mem_base = addr + size;
                } else {
                    NvOsDebugPrintf("Warning: could not allocate non-prefetchable memory for device %02x bar %d\n", dev->bus, bar_index);
                    valid = NV_FALSE;
                } 
            }
        }

        if (valid==NV_TRUE) 
        {
            NvRmReadWriteConfigSpace(rm, dev->bus, 
                NvRmPcieAccessType_Write, bar_index * 4 + 0x10, 
                (NvU8 *)&addr, 4);
        }

        dev->bar_base[bar_index] = addr;
        dev->bar_size[bar_index] = size;
        dev->bar_type[bar_index] = r;

        /* handle 64 bit addresses differently */
        if ((flags == 0x0c) || (flags==0x04)) 
        {
            //we're just locating it in 32bit space, so the 64 bit extension should be 0
            NvU32 upper_addr_bits = 0;

            //I hope the last bar doesn't claim to be 64 bit!!!
            bar_index++;
            NV_ASSERT(bar_index < 6);

            (void)NvRmReadWriteConfigSpace(rm, dev->bus, 
                    NvRmPcieAccessType_Write, bar_index * 4 + 0x10, (NvU8 *)&upper_addr_bits, 4);
        }
    }

    dev->io_limit = io_base;
    dev->mem_limit = mem_base;
    dev->prefetch_limit = prefetch_base;

    /* Update the control register to enable memory/io/bus-mastering */
    NvRmReadWriteConfigSpace(rm, dev->bus, NvRmPcieAccessType_Read, 
            NV_PROJ__PCIE2_RP_DEV_CTRL, (NvU8 *)&control, 2);
    control = NVPCIE_FLD_SET_DRF_DEF(RP, DEV_CTRL, IO_SPACE, ENABLED, control);
    control = NVPCIE_FLD_SET_DRF_DEF(RP, DEV_CTRL, MEMORY_SPACE, ENABLED, control);
    control = NVPCIE_FLD_SET_DRF_DEF(RP, DEV_CTRL, BUS_MASTER, ENABLED, control);
    control = NVPCIE_FLD_SET_DRF_DEF(RP, DEV_CTRL, INTR_DISABLE, NO, control);
    control = NVPCIE_FLD_SET_DRF_DEF(RP, DEV_CTRL, SERR, ENABLED, control);
    NvRmReadWriteConfigSpace(rm, dev->bus, NvRmPcieAccessType_Write, 
            NV_PROJ__PCIE2_RP_DEV_CTRL, (NvU8 *)&control, 2);
}

static NvU8
pcie_MsiGetOffset(NvRmDeviceHandle rm, NvRmPciDevice *device);

//0 if it didn't find it.  (offset can't be 0, that's the PCIE devid)
static NvU8
pcie_MsiGetOffset(NvRmDeviceHandle rm, NvRmPciDevice *device)
{
    NvU8 offset=0;
    NvU8 src;
    NvError e;

    NV_CHECK_ERROR_CLEANUP(NvRmReadWriteConfigSpace(rm, 
                device->bus, NvRmPcieAccessType_Read, 
                NV_PROJ__PCIE2_RP_CAP_PTR, (NvU8 *)&src, 1));

    offset=src;
    offset&=0xfc;

    while (offset) 
    {
        NvU8 id;
        NvU8 next_offset;
        NvU16 configarea;

        e = NvRmReadWriteConfigSpace(rm, device->bus, 
                NvRmPcieAccessType_Read, 
                offset, (NvU8*)&configarea, 2);
        if (e!=NvSuccess) 
        {
            offset=0;
            goto fail;
        }

        id=(configarea & 0x00ff);
        //high byte is the address of the next cap
        next_offset=((configarea & 0xff00) >> 8);
        next_offset &= 0xfc; //mask off the bottom two bits again

        if (id==NV_PROJ__PCIE2_RP_MSI_CTRL_CAP_ID_MSI) 
            break;
        offset=next_offset;
    }

fail:
    return offset;
}

static NvBool pcie_Is64BitMsi(NvRmDeviceHandle rm, NvRmPciDevice *device, NvU8 offset);
static NvBool
pcie_Is64BitMsi(NvRmDeviceHandle rm, NvRmPciDevice *device, NvU8 offset)
{
    NvU16 message_control;

    if (offset) 
    {
        NvRmReadWriteConfigSpace(rm, device->bus, NvRmPcieAccessType_Read, 
                offset+2, (NvU8 *)&message_control, 2);
        return  (message_control & 0x0080) ? NV_TRUE :  NV_FALSE;
    }
    return NV_FALSE;
}


static void pcie_SetMsiMultipleMessageEnable(NvRmDeviceHandle rm, NvRmPciDevice *device, NvU8 offset, NvU8 count);

static void
pcie_SetMsiMultipleMessageEnable(NvRmDeviceHandle rm, NvRmPciDevice *device, NvU8 offset, NvU8 count)
{    
    NvU16 message_control=0;

    if(offset) 
    {
        NvU8 val;

        switch (count) {
            case  1: val=0x00; break;
            case  2: val=0x01; break;
            case  4: val=0x02; break;
            case  8: val=0x03; break;
            case 16: val=0x04; break;
            case 32: val=0x05; break;
            default:
                NV_ASSERT(!"bad count");
                return;
        }


        NvRmReadWriteConfigSpace(rm, device->bus, NvRmPcieAccessType_Read,
                                           offset+2, (NvU8 *)&message_control, 2);

        message_control &= 0x008f;
        message_control |= (val<<4); 

        NvRmReadWriteConfigSpace(rm, device->bus, NvRmPcieAccessType_Write,
                                           offset+2, (NvU8 *)&message_control, 2);

    }
}

static void pcie_MsiSetAddr(NvRmDeviceHandle rm, NvRmPciDevice *device, NvU8 offset, NvU64 addr);

static void
pcie_MsiSetAddr(NvRmDeviceHandle rm, NvRmPciDevice *device, NvU8 offset, NvU64 addr)
{
    if(offset) 
    {
        NvU32 low32;
        NvU32 high32;

        low32 = (NvU32)(addr);
        high32 = (NvU32)(addr>>32);

        if(!pcie_Is64BitMsi(rm, device, offset)) 
        {
            if(addr > 0x0000000100000000ull) 
            {
                NvOsDebugPrintf("64 bit address given, but only 32 supported for MSI");
                return;
            }
        } 
        else 
        {
            //write the high bits of the 64 bit register
            NvRmReadWriteConfigSpace(rm, device->bus, NvRmPcieAccessType_Write, offset+8, (NvU8 *)&high32, 4);
        }

        //the low 32 bits of 32 and 64 bit MSI are at the same spot
        NvRmReadWriteConfigSpace(rm, device->bus, NvRmPcieAccessType_Write, offset+4, (NvU8 *)&low32, 4);
    } 
}

static void pcie_MsiSetData(NvRmDeviceHandle rm, NvRmPciDevice *device, NvU8 offset, NvU16 data);

static void
pcie_MsiSetData(NvRmDeviceHandle rm, NvRmPciDevice *device, NvU8 offset, NvU16 data)
{
    if (offset) 
    {
        //data is in a different spot based on the size of the MSI
        if (pcie_Is64BitMsi(rm, device, offset))
            NvRmReadWriteConfigSpace(rm, device->bus, NvRmPcieAccessType_Write, offset+12, (NvU8 *)&data, 2);
        else
            NvRmReadWriteConfigSpace(rm, device->bus, NvRmPcieAccessType_Write, offset+8, (NvU8 *)&data, 2);
    } 
}

static void
pcie_ConfigureMSI(NvRmDeviceHandle rm, NvRmPciDevice *device) 
{
    //iterate through the extended config looking for the MSI record
    NvU8 offset;
    
    //search for the MSI pointer
    offset = pcie_MsiGetOffset(rm, device);
    if (offset) 
    {
        NvU64 addr;
        NvU16 new_vector_count;
        NvU8 num_vectors;

        if (!pcie_Is64BitMsi(rm, device, offset)) 
        {
            //the address we use is a 64 bit one - would need to change all this 
            NV_ASSERT(!"32 bit MSI device unsupported\n");
            return;
        } 

        addr = pcie_regr(rm, PcieRegType_AFI, AFI_MSI_FPCI_BAR_ST_0);
        //FIXME - what is this supposed to be - need to test if this needs to be shifted
        //addr=addr

        NvRmReadWriteConfigSpace(rm, device->bus, NvRmPcieAccessType_Read, 
                offset+2, (NvU8 *)&num_vectors, 2);
        num_vectors = (num_vectors & 0x000e) >> 1;
        num_vectors = 1 << num_vectors;
        NV_ASSERT(num_vectors <= 32);

        //new vector base has to start at a multiple of num_vectors. 
        //i.e. if num_vectors is 8 then start has to be xxxxx000
        //     if num_vectors is 2 then start has to be xxxxxxx0
        new_vector_count=((s_pcieMsiVectorCount + (num_vectors - 1)) & ~(num_vectors-1));
        NV_ASSERT(new_vector_count < 256);        

        pcie_MsiSetAddr(rm, device, offset, addr + new_vector_count);
        pcie_MsiSetData(rm, device, offset, new_vector_count);

        pcie_SetMsiMultipleMessageEnable(rm, device, offset, num_vectors);
        s_pcieMsiVectorCount=new_vector_count;
    }
}


    

static void 
pcie_scanbus(NvRmDeviceHandle rm, NvRmPciDevice *dev_parent)
{
    NvU32 subordinate_bus;
    NvU32 next_bus_number;
    NvU32 device = 0;
    NvU32 data;
    NvU32 id;
    NvError err;
    NvRmPciDevice *dev;

    next_bus_number = dev_parent->sec_bus;
    while (1)
    {
        NvU32 retry_count = 6;
        if (device == 0x20)
        {
            dev_parent->sub_bus = next_bus_number;
            if (!dev_parent->isRootPort)
            {
                /* Change the subordinate bus-number to the actual value of all
                 * buses on the hierarcy.
                 *
                 * Do this execpt for the root port.
                 */
                data = next_bus_number;
                NvRmReadWriteConfigSpace(rm, (dev_parent->bus), 
                        NvRmPcieAccessType_Write, NV_PROJ__PCIE2_RP_BN_LT + 0x2, 
                        (NvU8 *)&data, 1);
            }
            return;
        }

        if (dev_parent->isRootPort && device != 0)
        {
            /* Sepcial Exit condition for root ports, as AP20 root port seems to
             * connect to only one device */
            return;
        }

        while (--retry_count)
        {
            err = NvRmReadWriteConfigSpace(rm, (dev_parent->sec_bus | device << 8), 
                    NvRmPcieAccessType_Read, 0x0, (NvU8 *)&id, 4);
            if (err != NvSuccess)
            {
                // NvOsDebugPrintf("PCIe link is not up\n");
                return;
            }
            if (id != 0xFFFFFFFF)
            {
                /* Found a valid device, break. Otherwise, retry a couple of
                 * times. It is possible that the bridges can take some time to
                 * settle and it will take couple of transcations to find the
                 * devcies behind the bridge.
                 */
                //this is still necessary, to detect the g98 behind the br04
                NvOsSleepMS(100);   /* FIXME it seems that the PCIe devices need this much delay! */
                break;
            }
        }
        if (id == 0xFFFFFFFF)
        {
            /* Invalid device. Skip that one and look for next device */
            device++;
            continue;
        }

        dev = pcie_allocDevice(rm);
        /* Fill the device information */
        dev->parent = dev_parent;
        dev->id = id;
        dev->bus = dev_parent->sec_bus | device << 8;
        if (dev_parent->child == NULL)
        {
            dev_parent->child = dev;
            dev->prev = NULL;
        } else
        {
            /* Add dev to the list of devices on the same bus */
            NvRmPciDevice *temp;

            temp = dev_parent->child;
            NV_ASSERT(temp != NULL);
            while (temp->next != NULL)
                temp = temp->next;
            temp->next = dev;
            dev->prev = temp;
        }

        (void)NvRmReadWriteConfigSpace(rm, dev->bus, NvRmPcieAccessType_Read, 
                NV_PROJ__PCIE2_RP_MISC_1, (NvU8 *)&data, 4);
        data = (data >> 16);
        if ((data & 0x7f) == 0x1)
        {
            /* Bridge device */

            /* Temporarily assign 0xff for the subordinate bus-number, as we don't
             * know how many devices are preset in the system */
            subordinate_bus = 0xff;
            dev->sec_bus = next_bus_number + 1;
            data = (subordinate_bus << 16) | (dev->sec_bus << 8) | (dev_parent->sec_bus);
            NvRmReadWriteConfigSpace(rm, dev->bus, NvRmPcieAccessType_Write, 
                    NV_PROJ__PCIE2_RP_BN_LT, (NvU8 *)&data, 3);
            
            /* Scan all the buses behind this bridge */
            pcie_scanbus(rm, dev);

            next_bus_number = dev->sub_bus;
        } else if ((data & 0x7f) == 0x0) 
        {
            //NvOsDebugPrintf("PCI endpoint (0x%x) is on bus = %d, device = %d\n",
            //        id, dev_parent->sec_bus, device);
            /* PCI endpoint - Can be single function or multie function */

        } else if ((data & 0x7f) == 0x2)
        {
            /* PC card device */
        } else
        {
            NV_ASSERT(!"invalid or malfunctional PCIe device \n");
        }
        device ++;
    }
}

static
void pcie_allocateResoruces(NvRmDeviceHandle rm, NvRmPciDevice *dev)
{
    NvU32 data;

    if (dev == NULL)
        return;
 
    if (!dev->isRootPort)
    {
        dev->mem_base = dev->parent->mem_base;
        dev->io_base = dev->parent->io_base;
        dev->prefetch_base = dev->parent->prefetch_base;

        dev->mem_limit = dev->parent->mem_limit;
        dev->io_limit = dev->parent->io_limit;
        dev->prefetch_limit = dev->parent->prefetch_limit;

        dev->mem_max = dev->parent->mem_max;
        dev->prefetch_max = dev->parent->prefetch_max;
    }

    /* Employing a depth first search algorithm for resource allocation. */
    if (dev->child != NULL)
    {
        pcie_allocateResoruces(rm, dev->child);
    }

    if (dev->next != NULL)
    {
        pcie_allocateResoruces(rm, dev->next);
    }

    /* A PCI device */
    if (dev->sub_bus == 0)
    {
        NvRmPciDevice *nextDev;

        /* If this is first device on the bus, get the "base" from the parent,
         * if not get the base from the next node on the chain which is not
         * disabled. Disabled node is node that is either malfunctioning or a
         * bridge with no devices beneath that device. */
        nextDev = dev->next;
        while (nextDev != NULL)
        {
            if (!nextDev->IsDisabled)
                break;
            nextDev = nextDev->next;
        }
        if (nextDev)
        {
            dev->mem_base = nextDev->mem_limit;
            dev->io_base = nextDev->io_limit;
            dev->prefetch_base = nextDev->prefetch_limit;
        }
        pcie_ConfigureDeviceBAR(rm, dev);

        pcie_ConfigureMSI(rm, dev);
    } else
    {
        NvRmPciDevice *child_dev;
        NvBool is64bit = NV_FALSE;

        /* First enabled child will have the max values of the address
         * regions for all the devices on that bus */
        child_dev = dev->child;
        while (child_dev != NULL)
        {
            if (!child_dev->IsDisabled)
                break;
            child_dev = child_dev->next;
        }
        if (child_dev == NULL)
        {
            dev->IsDisabled = NV_TRUE;
            dev->mem_limit = 0;
            dev->io_limit = 0;
            dev->prefetch_limit = 0;
            dev->mem_base = 0;
            dev->io_base = 0;
            dev->prefetch_base = 0;
            /* No valid child dev behind this bridge. Just return */
            return;
        }

        dev->mem_limit = child_dev->mem_limit;
        dev->io_limit = child_dev->io_limit;
        dev->prefetch_limit = child_dev->prefetch_limit;

        /* Now program the bridge address filtering registers */

        if (dev->isRootPort)
        {
            /* Root port is handled differently just return */
            return;
        }

        /* Program the non-prefetchable memory base and limit for the bridge */
        data = 0;
        data = NVPCIE_FLD_SET_DRF_NUM(RP, MEM_BL, MEM_BASE, dev->mem_base >> 20, data);
        data = NVPCIE_FLD_SET_DRF_NUM(RP, MEM_BL, MEM_LIMIT, dev->mem_limit >> 20, data);
        NvRmReadWriteConfigSpace(rm, dev->bus, NvRmPcieAccessType_Write, 
                NV_PROJ__PCIE2_RP_MEM_BL, (NvU8 *)&data, 4);
        
        /* Program the prefetchable memory base and limit for the bridge. */
        NvRmReadWriteConfigSpace(rm, dev->bus, NvRmPcieAccessType_Read, 
                NV_PROJ__PCIE2_RP_PRE_BL, (NvU8 *)&data, 4);
        if ((NVPCIE_DRF_VAL(RP, PRE_BL, B64BIT, data)) == 1)
        {
            is64bit = NV_TRUE;
        }

        data = 0;
        data = NVPCIE_FLD_SET_DRF_NUM(RP, PRE_BL, PREFETCH_MEM_BASE, dev->prefetch_base >> 20, data);
        data = NVPCIE_FLD_SET_DRF_NUM(RP, PRE_BL, PREFETCH_MEM_LIMIT, dev->prefetch_limit >> 20, data);
        if (is64bit)
        {
            data = NVPCIE_FLD_SET_DRF_DEF(RP, PRE_BL, L64BIT, YES, data);
            data = NVPCIE_FLD_SET_DRF_DEF(RP, PRE_BL, B64BIT, YES, data);
        }
        NvRmReadWriteConfigSpace(rm, dev->bus, NvRmPcieAccessType_Write, 
                NV_PROJ__PCIE2_RP_PRE_BL, (NvU8 *)&data, 4);
        if (is64bit)
        {
            data = 0;
            NvRmReadWriteConfigSpace(rm, dev->bus, NvRmPcieAccessType_Write, 
                    NV_PROJ__PCIE2_RP_PRE_LU32, (NvU8 *)&data, 4);
            NvRmReadWriteConfigSpace(rm, dev->bus, NvRmPcieAccessType_Write, 
                    NV_PROJ__PCIE2_RP_PRE_BU32, (NvU8 *)&data, 4);
        }

        /* Update the control register to enable memory/io/bus-mastering */
        NvRmReadWriteConfigSpace(rm, dev->bus, NvRmPcieAccessType_Read, 
                NV_PROJ__PCIE2_RP_DEV_CTRL, (NvU8 *)&data, 2);
        data = NVPCIE_FLD_SET_DRF_DEF(RP, DEV_CTRL, IO_SPACE, ENABLED, data);
        data = NVPCIE_FLD_SET_DRF_DEF(RP, DEV_CTRL, MEMORY_SPACE, ENABLED, data);
        data = NVPCIE_FLD_SET_DRF_DEF(RP, DEV_CTRL, BUS_MASTER, ENABLED, data);
        data = NVPCIE_FLD_SET_DRF_DEF(RP, DEV_CTRL, INTR_DISABLE, NO, data);
        data = NVPCIE_FLD_SET_DRF_DEF(RP, DEV_CTRL, SERR, ENABLED, data);
        NvRmReadWriteConfigSpace(rm, dev->bus, NvRmPcieAccessType_Write, 
                NV_PROJ__PCIE2_RP_DEV_CTRL, (NvU8 *)&data, 2);
    }
    return;
}

static
void pcie_buswalk(NvRmDeviceHandle rm, NvRmPciDevice *dev)
{
    NvU32 i;
    if (dev == NULL)
        return;
    
    NvOsDebugPrintf("PCIe device/bridge\n");
    NvOsDebugPrintf("   Id = 0x%x bus = 0x%x\n", dev->id, dev->bus);
    if (dev->IsDisabled)
    {
        NvOsDebugPrintf("   Device/bridge disabled \n");
    } else
    {
        NvOsDebugPrintf("   mem_base = 0x%x mem_limit = 0x%x\n", 
                dev->mem_base, dev->mem_limit);
        NvOsDebugPrintf("   prefetch_base = 0x%x prefetch_limit = 0x%x\n", 
                dev->prefetch_base, dev->prefetch_limit);
        NvOsDebugPrintf("   io_base = 0x%x io_limit = 0x%x\n", 
                dev->io_base, dev->io_limit);
        for (i=0; i< 6; i++)
        {
            //skip printing the empty ones
            if((dev->bar_base[i]) || (dev->bar_size[i]) || (dev->bar_type[i]))
                NvOsDebugPrintf("   bar(%d) base = 0x%x size = 0x%x type = %d\n", 
                        i, dev->bar_base[i], dev->bar_size[i], dev->bar_type[i]);
        }
    }

    if (dev->child != NULL)
        pcie_buswalk(rm, dev->child);

    if (dev->next != NULL)
        pcie_buswalk(rm, dev->next);
}


void pcie_businit(NvRmDeviceHandle hRm)
{
    pcie_businitx(hRm, 0);
    pcie_businitx(hRm, 1);
}

void pcie_businitx(NvRmDeviceHandle rm, int rpindex) 
{
    //note, this function is not re-entrant.
    NvRmPciDevice *rp=NULL;
    NvU32 data0;
    //some constants, depending on which bus we're on
    const NvU32 pcieregtype_cfgx=    (rpindex) ? PcieRegType_CFG1      : PcieRegType_CFG0;
    const NvU32 rootport_x_bus=      (rpindex) ? ROOTPORT_1_BUS        : ROOTPORT_0_BUS;
    const NvU32 rootport_x_subbus=   (rpindex) ? ROOTPORT_1_SUBBUS     : ROOTPORT_0_SUBBUS;
    NvU32 *pciconfigx=               (rpindex) ? s_pciConfig1          : s_pciConfig0; 

    switch(rpindex) {
        case 0:
        case 1:
            break;
        default:
            //if there's more than two root ports, various
            //parts of this function will need to be fixed
            NV_ASSERT(!"Bad root port index");
            goto fail;
    }

    //is this root port present?
    if((rpindex==0) && (s_PcieRootPort0Present == NV_FALSE)) goto fail;
    if((rpindex==1) && (s_PcieRootPort1Present == NV_FALSE)) goto fail;
            
    rp=pcie_allocDevice(rm);

    //if there's already a root port allocated, attach to that one's "next"
    //there shouldn't be anything attached to it already
    //otherwise, this is the first root port
    if(s_rp0)
        if(s_rp0->next)
            NV_ASSERT(!("next for s_rp0\n")); //shouldn't be possible
        else
            s_rp0->next=rp;  
    else
        s_rp0 = rp;

    rp->isRootPort = NV_TRUE;
    rp->sec_bus = rootport_x_bus + 1;
    rp->sub_bus = rootport_x_subbus;
    rp->bus = rootport_x_bus;
    rp->id = pcie_regr(rm, pcieregtype_cfgx, NV_PROJ__PCIE2_RP_DEV_ID);

    /* Read the config space into the SW shadow, configure the config and then
     * write back */
    pcie_ReadRPConfig(rm, (NvU8 *)pciconfigx, sizeof(s_pciConfig0), 0, rpindex);

    /* Set the root port bus numbers to maximum. */
    data0 = pciconfigx[NV_PROJ__PCIE2_RP_BN_LT/4];
    data0 = NVPCIE_FLD_SET_DRF_NUM(RP, BN_LT, PRI_BUS_NUMBER, rp->bus, data0);
    data0 = NVPCIE_FLD_SET_DRF_NUM(RP, BN_LT, SEC_BUS_NUMBER, rp->sec_bus, data0);
    data0 = NVPCIE_FLD_SET_DRF_NUM(RP, BN_LT, SUB_BUS_NUMBER, rp->sub_bus, data0);
    pciconfigx[NV_PROJ__PCIE2_RP_BN_LT/4] = data0;
    pcie_WriteRPConfig(rm, (NvU8 *)&data0, 4, NV_PROJ__PCIE2_RP_BN_LT, rpindex);

    /* Scan the bus and assign the bus numbers. */
    pcie_scanbus(rm, rp);

    /* fix the sub-ordinate bus number for the root port */
    data0 = pciconfigx[NV_PROJ__PCIE2_RP_BN_LT/4];
    data0 = NVPCIE_FLD_SET_DRF_NUM(RP, BN_LT, SUB_BUS_NUMBER, rp->sub_bus, data0);
    pciconfigx[NV_PROJ__PCIE2_RP_BN_LT/4] = data0;
    pcie_WriteRPConfig(rm, (NvU8 *)&data0, 4, NV_PROJ__PCIE2_RP_BN_LT, rpindex);

    //each port gets half the range
    if(s_rp0 == rp) {
        rp->mem_base = FPCI_NON_PREFETCH_MEMORY_OFFSET;
        rp->mem_max = FPCI_NON_PREFETCH_MEMORY_OFFSET + NVRM_PCIE_NON_PREFETCH_MEMORY_SIZE/2;
        rp->prefetch_base = FPCI_PREFETCH_MEMORY_OFFSET;
        rp->prefetch_max = FPCI_PREFETCH_MEMORY_OFFSET + NVRM_PCIE_PREFETCH_MEMORY_SIZE/2;
        rp->io_base = 0x16; //not starting at 0 - some drivers have issue with 0
    } else {
        rp->mem_base = FPCI_NON_PREFETCH_MEMORY_OFFSET + (NVRM_PCIE_NON_PREFETCH_MEMORY_SIZE/2);
        rp->mem_max = FPCI_NON_PREFETCH_MEMORY_OFFSET + NVRM_PCIE_NON_PREFETCH_MEMORY_SIZE;
        rp->prefetch_base = FPCI_PREFETCH_MEMORY_OFFSET + (NVRM_PCIE_PREFETCH_MEMORY_SIZE/2);
        rp->prefetch_max = FPCI_PREFETCH_MEMORY_OFFSET + NVRM_PCIE_PREFETCH_MEMORY_SIZE;
        rp->io_base = s_rp0->io_limit;
    }

    pcie_allocateResoruces(rm, rp);

    /* Memory/IO/prefetch address filtering */
    data0 = pciconfigx[NV_PROJ__PCIE2_RP_IO_BL_SS/4];
    data0 = NVPCIE_FLD_SET_DRF_DEF(RP, IO_BL_SS, IO_BASE_SUPPORT, 32, data0);
    data0 = NVPCIE_FLD_SET_DRF_DEF(RP, IO_BL_SS, IO_BASE, ADDRESS_0, data0);
    data0 = NVPCIE_FLD_SET_DRF_DEF(RP, IO_BL_SS, IO_LIMIT_SUPPORT, 32, data0);
    data0 = NVPCIE_FLD_SET_DRF_DEF(RP, IO_BL_SS, IO_LIMIT, ADDRESS_256, data0);
    pciconfigx[NV_PROJ__PCIE2_RP_IO_BL_SS/4] = data0;
    pciconfigx[NV_PROJ__PCIE2_RP_IO_BL_U16/4] = 0;
    data0 = 0;
    data0 = NVPCIE_FLD_SET_DRF_NUM(RP, MEM_BL, MEM_BASE, rp->mem_base >> 20, data0);
    data0 = NVPCIE_FLD_SET_DRF_NUM(RP, MEM_BL, MEM_LIMIT, (rp->mem_limit-1) >> 20, data0);
    pciconfigx[NV_PROJ__PCIE2_RP_MEM_BL/4] = data0;
    data0 = 0;
    data0 = NVPCIE_FLD_SET_DRF_DEF(RP, PRE_BL, B64BIT, YES, data0);
    data0 = NVPCIE_FLD_SET_DRF_NUM(RP, PRE_BL, PREFETCH_MEM_BASE, rp->prefetch_base >> 20, data0);
    data0 = NVPCIE_FLD_SET_DRF_DEF(RP, PRE_BL, L64BIT, YES, data0);
    data0 = NVPCIE_FLD_SET_DRF_NUM(RP, PRE_BL, PREFETCH_MEM_LIMIT, (rp->prefetch_limit-1) >> 20, data0);
    pciconfigx[NV_PROJ__PCIE2_RP_PRE_BL/4] = data0;
    pciconfigx[NV_PROJ__PCIE2_RP_PRE_BU32/4] = 0x0;
    pciconfigx[NV_PROJ__PCIE2_RP_PRE_LU32/4] = 0x0;

    /* command register */
    data0 = 0;
    data0 = NVPCIE_FLD_SET_DRF_DEF(RP, DEV_CTRL, IO_SPACE, ENABLED, data0);
    data0 = NVPCIE_FLD_SET_DRF_DEF(RP, DEV_CTRL, MEMORY_SPACE, ENABLED, data0);
    data0 = NVPCIE_FLD_SET_DRF_DEF(RP, DEV_CTRL, BUS_MASTER, ENABLED, data0);
    data0 = NVPCIE_FLD_SET_DRF_DEF(RP, DEV_CTRL, INTR_DISABLE, NO, data0);
    data0 = NVPCIE_FLD_SET_DRF_DEF(RP, DEV_CTRL, SERR, ENABLED, data0);
    pciconfigx[NV_PROJ__PCIE2_RP_DEV_CTRL/4] = data0;

    pcie_WriteRPConfig(rm, (NvU8 *)pciconfigx, sizeof(s_pciConfig0),0, rpindex);

fail:
    return;
}


/* For now implementing a static address translation. One can do some complicated
 * dynamic address mapping, but AP20 use case, doesn't need one. 
 *
 *  It would be non-trivial to support dynamic memory mapping.
 *
 * */
NvRmPhysAddr NvRmMapPciMemory( 
    NvRmDeviceHandle hDeviceHandle,
    NvRmPciPhysAddr mem,
    NvU32 size )
{
    NvRmPciPhysAddr prefetch_base;
    NvRmPciPhysAddr prefetch_limit;
    NvRmPciPhysAddr mem_base;
    NvRmPciPhysAddr mem_limit;

    prefetch_base = FPCI_PREFETCH_MEMORY_OFFSET;
    mem_base = FPCI_NON_PREFETCH_MEMORY_OFFSET;

    prefetch_limit = FPCI_PREFETCH_MEMORY_OFFSET + NVRM_PCIE_PREFETCH_MEMORY_SIZE;
    mem_limit = FPCI_NON_PREFETCH_MEMORY_OFFSET + NVRM_PCIE_NON_PREFETCH_MEMORY_SIZE;

    if (mem >= mem_base && ((mem + size) <= mem_limit))
    {
        return (s_pciePhysical + (NvRmPhysAddr)(((mem - mem_base) + NVRM_PCIE_NON_PREFETCH_MEMORY_OFFSET)));
    }
    if (mem >= prefetch_base && ((mem + size) <= prefetch_limit))
    {
        return (s_pciePhysical + (NvRmPhysAddr)(((mem - prefetch_base) + NVRM_PCIE_PREFETCH_MEMORY_OFFSET)));
    }
    return 0;
}

void NvRmUnmapPciMemory( 
    NvRmDeviceHandle hDeviceHandle,
    NvRmPhysAddr mem,
    NvU32 size )
{
    return;
}

NvError NvRmRegisterPcieLegacyHandler(
    NvRmDeviceHandle rm, 
    NvU32 function_device_bus, 
    NvOsSemaphoreHandle sem,
    NvBool InterruptEnable)
{
    NvError retval=NvSuccess;
    NvRmPciDevice *device;  
    NvU32 index;
    static NvBool initialized=NV_FALSE;
    
    device = pcie_GetDevice(function_device_bus);
    if (device == NULL) 
    {
        retval=NvError_DeviceNotFound;
        goto fail;
    }

    if(!initialized) 
    {
        for(index=0; index<=MAX_LEGACY_HANDLERS; index++) 
        {
            LegacyHandlers[index].sem=0;
        }
        initialized=NV_TRUE;
    }

    for(index=0; index<MAX_LEGACY_HANDLERS; index++) 
    {
        if(LegacyHandlers[index].sem==0) break;
    }

    if(index>=MAX_LEGACY_HANDLERS)
    {
        NvOsDebugPrintf("ran out of legacy interrupt handles!\n");
        goto fail;
    }
    LegacyHandlers[index].sem = sem;

    if (InterruptEnable) 
    {
        //do something here?
    }

fail:
    return retval;
}

/* max of 256 possible MSI handlers, but normally much less*/
NvError NvRmRegisterPcieMSIHandler(
    NvRmDeviceHandle rm, 
    NvU32 function_device_bus, 
    NvU32 index,
    NvOsSemaphoreHandle sem,
    NvBool InterruptEnable)
{
    //#1 - how to uniquely identify the device?
    //And which MSI to handle?
    //does this actually need the rm handle?  There can't be multiple instances.

    //device is function_device_bus
    NvError retval=NvSuccess;

    NvU8 offset;
    NvU16 num_messages;
    NvU16 baseIndex;
    NvRmPciDevice *device;  
    
    device = pcie_GetDevice(function_device_bus);
    if (device == NULL) 
    {
        retval=NvError_DeviceNotFound;
        goto fail;
    }

    offset = pcie_MsiGetOffset(rm, device);
    //first make sure there's msi's for this device.
    if (!offset) {
        retval = NvError_DeviceNotFound;
        goto fail;
    }

    NvRmReadWriteConfigSpace(rm, device->bus, NvRmPcieAccessType_Read, 
            offset+2, (NvU8 *)&num_messages, 2);

    num_messages = (num_messages & 0x0070) >> 4;
    num_messages = 1 << num_messages;

    /* MSI interrupts are allocated during enumeration. Find the vector that is
     * mapped to this device */
    if (pcie_Is64BitMsi(rm, device, offset)) 
        NvRmReadWriteConfigSpace(rm, device->bus, 
                NvRmPcieAccessType_Read, offset+12, (NvU8 *)&baseIndex, 2);
    else
        NvRmReadWriteConfigSpace(rm, device->bus, 
                NvRmPcieAccessType_Read, offset+8, (NvU8 *)&baseIndex, 2);

    index += baseIndex;
    MSIHandlers[index].sem = sem;

    if (InterruptEnable) {
        NvU16 message_control; 

        NvRmReadWriteConfigSpace(rm, device->bus, NvRmPcieAccessType_Read, 
                offset+2, (NvU8 *)&message_control, 2);

        message_control &= 0x00fe; 
        message_control |= 1;

        NvRmReadWriteConfigSpace(rm, device->bus, NvRmPcieAccessType_Write, 
                offset+2, (NvU8 *)&message_control, 2);
    }

fail:
    return retval;
}

static
void NvRmPrivHandlePcieInterrupt(void *arg)
{
    NvU32 intr_info, intr_extended_info;
    NvRmDeviceHandle rm = (NvRmDeviceHandle)arg;

    intr_info = pcie_regr(rm, PcieRegType_AFI, AFI_INTR_CODE_0);
    intr_info = NV_DRF_VAL(AFI, INTR_CODE, INT_CODE, intr_info);
    intr_extended_info = pcie_regr(rm, PcieRegType_AFI, AFI_INTR_SIGNATURE_0);
    switch (intr_info)
    {
        /* Interrupt code */
        case 6:
            {
                NvBool match=NV_FALSE;
                NvU32 i;
                for(i=0; i<MAX_LEGACY_HANDLERS; i++) {
                    if(LegacyHandlers[i].sem) {
                        NvOsSemaphoreSignal(LegacyHandlers[i].sem);
                        match=NV_TRUE;
                    }
                }
                if(match==NV_FALSE) 
                    NvOsDebugPrintf("Got an unhandled sideband message interrupt 0x%x\n", intr_extended_info);
            }
            break;
        case 1:
            /* SLVERR */
            NvOsDebugPrintf("AXI Slave error interrupt\n");
            break;
        case 2:
            /* DECERR */
            NvOsDebugPrintf("AXI decode error interrupt\n");
            break;
        case 3:
            /* PCIE target abort */
            NvOsDebugPrintf("PCIE target abort interrupt\n");
            break;
        case 4:
            /* PCIE master abort */
            // Don't print this, as this error is a common error during
            // enumeration.
            // NvOsDebugPrintf("PCIE master abort interrupt\n");
            break;
        case 5:
            /* Bufferable write to non-posted write */
            NvOsDebugPrintf("Invalid write interrupt: Bufferable write to non-posted region\n");
            break;
        case 7:
            /* Response address mapping error */
            NvOsDebugPrintf("PCIE response decoding error interrupt\n");
            break;
        case 8:
            /* Response address mapping error */
            NvOsDebugPrintf("AXI response decoding error interrupt\n");
            break;
        case 9:
            /* PCIE timeout */
            NvOsDebugPrintf("PCIE transcation timeout\n");
            break;
        default:
            break;
    }
    /* Clear the interrupt code register to sample the next interrupt */
    pcie_regw(rm, PcieRegType_AFI, AFI_INTR_CODE_0, 0);

    NvRmInterruptDone(s_pcieInterruptHandle);
    return;
}


void NvRmPrivHandlePcieMSI(void *arg)
{
    NvU32 vecs[8];
    int i;
    NvRmDeviceHandle hRm=arg;

    //figure out which MSI we got, and then call the handler

    vecs[0]=pcie_regr(hRm, PcieRegType_AFI, AFI_MSI_VEC0_0);
    vecs[1]=pcie_regr(hRm, PcieRegType_AFI, AFI_MSI_VEC1_0);
    vecs[2]=pcie_regr(hRm, PcieRegType_AFI, AFI_MSI_VEC2_0);
    vecs[3]=pcie_regr(hRm, PcieRegType_AFI, AFI_MSI_VEC3_0);
    vecs[4]=pcie_regr(hRm, PcieRegType_AFI, AFI_MSI_VEC4_0);
    vecs[5]=pcie_regr(hRm, PcieRegType_AFI, AFI_MSI_VEC5_0);
    vecs[6]=pcie_regr(hRm, PcieRegType_AFI, AFI_MSI_VEC6_0);
    vecs[7]=pcie_regr(hRm, PcieRegType_AFI, AFI_MSI_VEC7_0);

    for (i=0; i<8; i++) 
    {
        while (vecs[i])
        {
            NvU32 bit;
            NvU32 index;

            bit = (31 - CountLeadingZeros(vecs[i]));
            index = (i * 32) + bit;

            // clear the MSI bit in the register and in the shadow
            vecs[i] &= ~(1<<bit);
            pcie_regw(hRm, PcieRegType_AFI, AFI_MSI_VEC0_0 + i * 4,  0x01 << bit);

            if (MSIHandlers[index].sem)
            {
                NvOsSemaphoreSignal(MSIHandlers[i].sem);
            } else 
            {
                NvOsDebugPrintf("unhandled MSI %08x %08x\n", i,bit);
            }
        }
    }
    NvRmInterruptDone(s_pcieMSIHandle);
}

