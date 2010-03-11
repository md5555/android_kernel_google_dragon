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

#ifndef INCLUDED_AP20RM_PCIE_PRIVATE_H
#define INCLUDED_AP20RM_PCIE_PRIVATE_H

#include "nvcommon.h"
#include "ap20/dev_ap_pcie2_root_port.h"
#include "ap20/dev_ap_pcie2_pads.h"
#include "ap20/arafi.h"
#include "nvrm_drf.h"


/*
 * AXI address map for the PCIe aperture.  AP20, defines 1GB in the AXI address
 * map for PCIe. 
 *
 *  That address space is split into different regions, with sizes and offsets
 *  as follows.
 *  
 * 0x8000_0000 to 0x80ff_ffff - Register space          16MB.
 * 0x8100_0000 to 0x81ff_ffff - Config space            16MB.
 * 0x8200_0000 to 0x82ff_ffff - Extended config space   16MB.
 * 0x8300_0000 to 0x83ff_ffff - Downstream IO space 
 * ...          Will be filled with other BARS like MSI/upstream IO etc.
 * 0x9000_0000 to 0x9fff_ffff - non-prefetchable memory aperture 
 * 0xa000_0000 to 0xbfff_ffff - Prefetchable memory aperture 
 *
 * Config and Extended config sizes are choosen to support maximum of 256 devices, 
 * which is good enough for all the AP20 use cases.
 *
 * */
#define NVRM_PCIE_REGISTER_APERTURE_SIZE    0x1000000UL
#define NVRM_PCIE_CONFIG_OFFSET             NVRM_PCIE_REGISTER_APERTURE_SIZE
#define NVRM_PCIE_CONFIG_SIZE               0x1000000UL
#define NVRM_PCIE_EXTENDED_CONFIG_OFFSET    NVRM_PCIE_CONFIG_SIZE + NVRM_PCIE_CONFIG_OFFSET
#define NVRM_PCIE_EXTENDED_CONFIG_SIZE      0x1000000UL
#define NVRM_PCIE_DOWNSTREAM_IO_OFFSET      NVRM_PCIE_EXTENDED_CONFIG_SIZE + NVRM_PCIE_EXTENDED_CONFIG_OFFSET
#define NVRM_PCIE_DOWNSTREAM_IO_SIZE        0x100000UL
/*... some room for the other BARs */
#define NVRM_PCIE_NON_PREFETCH_MEMORY_OFFSET    0x10000000UL
#define NVRM_PCIE_NON_PREFETCH_MEMORY_SIZE      0x10000000UL
#define NVRM_PCIE_PREFETCH_MEMORY_OFFSET         NVRM_PCIE_NON_PREFETCH_MEMORY_OFFSET + NVRM_PCIE_NON_PREFETCH_MEMORY_SIZE
#define NVRM_PCIE_PREFETCH_MEMORY_SIZE           0x20000000UL

/*
 *  PCI address map for memory mapped devices. Still using 32-bit aperture.
 *
 *  1GB for the system memory. 
 *  1GB for the non pre-fetchable memory
 *  1GB for the pre-fetchable memory
 *
 *  Though all the PCI devices gets mapped to this address ranges, ARM cannot
 *  see the entire non-prefectable/prefectable memory as the AXI address map
 *  only has 768MB for the total non-prefectable/prefetchable memory. See the
 *  above defines for that address map.
 */
#define FPCI_SYSTEM_MEMORY_OFFSET           0x0UL
#define FPCI_SYSTEM_MEMORY_SIZE             0x40000000UL
#define FPCI_NON_PREFETCH_MEMORY_OFFSET     (FPCI_SYSTEM_MEMORY_OFFSET + FPCI_SYSTEM_MEMORY_SIZE)
#define FPCI_NON_PREFETCH_MEMORY_SIZE       0x40000000UL
#define FPCI_PREFETCH_MEMORY_OFFSET         (FPCI_NON_PREFETCH_MEMORY_OFFSET + FPCI_NON_PREFETCH_MEMORY_SIZE)
#define FPCI_PREFETCH_MEMORY_SIZE           0x40000000UL

/* Lower 16K of the PCIE apperture has root port registers. There are 4 groups
 * of root port registers.
 *
 * 1. AFI registers - AFI is a wrapper between PCIE and ARM AXI bus. These
 * registers define the address translation registers, interrupt registers and
 * some configuration (a.k.a CYA) registers.
 * 2. PAD registers - PAD control registers which are inside the PCIE CORE.
 * 3. Configuration 0 and Configuration 1 registers - These registers are PCIe
 * configuration registers of Root port 0 and root port 1.
 *
 * Check the PcieRegType enumeration for the list of Registers banks inside the
 * PCIE aperture.
 *
 * */
#define NV_PCIE_AXI_AFI_REGS_OFSET		    0x3800
#define NV_PCIE_AXI_PADS_OFSET	            0x3000
#define NV_PCIE_AXI_RP_T0C0_OFFSET		    0x0000
#define NV_PCIE_AXI_RP_T0C1_OFFSET	        0x1000

#define NVRM_PCIE_MAX_DEVICES               256

/* PCIE DRF macros to read and write PRI registers */

/** NVPCIE_DRF_DEF - define a new register value.

    @param d register domain (hardware block)
    @param r register name
    @param f register field
    @param c defined value for the field
 */
#define NVPCIE_DRF_DEF(d,r,f,c) \
    ((NV_PROJ__PCIE2_##d##_##r##_##f##_##c) << NV_FIELD_SHIFT(NV_PROJ__PCIE2_##d##_##r##_##f))

/** NVPCIE_DRF_NUM - define a new register value.

    @param d register domain (hardware block)
    @param r register name
    @param f register field
    @param n numeric value for the field
 */
#define NVPCIE_DRF_NUM(d,r,f,n) \
    (((n)& NV_FIELD_MASK(NV_PROJ__PCIE2_##d##_##r##_##f)) << \
        NV_FIELD_SHIFT(NV_PROJ__PCIE2_##d##_##r##_##f))

/** NVPCIE_DRF_VAL - read a field from a register.

    @param d register domain (hardware block)
    @param r register name
    @param f register field
    @param v register value
 */
#define NVPCIE_DRF_VAL(d,r,f,v) \
    (((v)>> NV_FIELD_SHIFT(NV_PROJ__PCIE2_##d##_##r##_##f)) & \
        NV_FIELD_MASK(NV_PROJ__PCIE2_##d##_##r##_##f))

/** NVPCIE_FLD_SET_DRF_NUM - modify a register field.

    @param d register domain (hardware block)
    @param r register name
    @param f register field
    @param n numeric field value
    @param v register value
 */
#define NVPCIE_FLD_SET_DRF_NUM(d,r,f,n,v) \
    ((v & ~NV_FIELD_SHIFTMASK(NV_PROJ__PCIE2_##d##_##r##_##f)) | NVPCIE_DRF_NUM(d,r,f,n))

/** NVPCIE_FLD_SET_DRF_DEF - modify a register field.

    @param d register domain (hardware block)
    @param r register name
    @param f register field
    @param c defined field value
    @param v register value
 */
#define NVPCIE_FLD_SET_DRF_DEF(d,r,f,c,v) \
    (((v) & ~NV_FIELD_SHIFTMASK(NV_PROJ__PCIE2_##d##_##r##_##f)) | \
        NVPCIE_DRF_DEF(d,r,f,c))

/** NVPCIE_RESETVAL - get the reset value for a register.

    @param d register domain (hardware block)
    @param r register name
 */
#define NVPCIE_RESETVAL(d,r)    (d##_##r##_0_RESET_VAL)

/* Major PCIE register banks */
typedef enum {
    PcieRegType_AFI,
    PcieRegType_CFG0, 
    PcieRegType_CFG1, 
    PcieRegType_PADS, 
    PcieRegType_Force32 = 0x7FFFFFFF,
} PcieRegType; 

typedef enum 
{
    NvRmPciResource_Io,
    NvRmPciResource_PrefetchMemory,
    NvRmPciResource_NonPrefetchMemory,
    NvRmPciResource_Force32 = 0x7FFFFFFF,

} NvRmPciResource;


typedef struct NvRmPciDeviceRec
{
    /// Bus and device number encoding of the PCEIe device/bridge
    NvU32 bus;
    /// Secondary bus nummber. Non-zero only for bridge devices. 
    NvU32 sec_bus;
    /// Subordinate bus number. Non-zero only for the bridge devices.
    NvU32 sub_bus;
    /// Device ID/vendor ID of the PCI device/bridge. upper 16 bits are device
    /// ID and lower 16 bits are vendor ID.
    NvU32 id;

    /// Base address registers of PCIe devices.
    NvU32 bar_base[6];
    NvU32 bar_size[6];
    NvRmPciResource bar_type[6];

    NvU32 io_base;
    NvU32 io_limit;

    NvU32 mem_base;
    NvU32 mem_limit;
    NvU32 mem_max; //highest address available on this's rootport

    NvU32 prefetch_base;
    NvU32 prefetch_limit;
    NvU32 prefetch_max;  //highest address available on this's rootport 

    NvBool IsDisabled;

    struct NvRmPciDeviceRec *parent;
    struct NvRmPciDeviceRec *next;
    struct NvRmPciDeviceRec *prev;
    struct NvRmPciDeviceRec *child;
    NvBool isRootPort;
}  NvRmPciDevice;


#define MAX_PCI_DEVICES     64


NvError NvRmPrivPcieOpen(NvRmDeviceHandle hDeviceHandle);
void NvRmPrivPcieClose(NvRmDeviceHandle hDeviceHandle);



#endif // INCLUDED_AP20RM_PCIE_PRIVATE_H

