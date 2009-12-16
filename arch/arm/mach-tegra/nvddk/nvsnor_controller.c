/*
 * arch/arm/mach-tegra/nvddk/nvsnor_controller.c
 *
 * DDK-like routines for accessing the SNOR controller
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

#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>

#include "nvsnor_controller.h"

#define SNOR_CS_FOR_DPRAM 4
#define SNOR_CS_FOR_ONENAND 2

NvSnorInformation *g_pSnorInfo = NULL;

struct mutex snor_operation_lock;
struct mutex snor_hal_lock;

            
static void SnorISR (void *pArgs)
{
    NvSnorHandle hSnor_isr = (NvSnorHandle)pArgs;
    NvU32 StatusReg;

    StatusReg = SNOR_READ32(hSnor_isr->pSnorControllerVirtBaseAdd, DMA_CFG);
    if (NV_DRF_VAL(SNOR, DMA_CFG, IS_DMA_DONE, StatusReg))
    {
        SNOR_WRITE32(hSnor_isr->pSnorControllerVirtBaseAdd, DMA_CFG, StatusReg);
        NvOsSemaphoreSignal(hSnor_isr->hCommandCompleteSema);
    }

    NvRmInterruptDone(hSnor_isr->hIntr);
}

static NvError RegisterSnorInterrupt(NvSnorHandle hSnor)
{
    NvU32 IrqList;
    NvOsInterruptHandler IntHandle = SnorISR;

    IrqList = NvRmGetIrqForLogicalInterrupt(hSnor->hRmDevice,
                        NVRM_MODULE_ID(NvRmModuleID_SyncNor, 0),
                        0);
    return NvRmInterruptRegister(hSnor->hRmDevice, 1, &IrqList, &IntHandle,
                                    hSnor, &hSnor->hIntr, NV_TRUE);
}
/**
 * Create the dma buffer memory handle.
 */
static NvError
CreateDmaBufferMemoryHandle(
    NvRmDeviceHandle hDevice,
    NvRmMemHandle *phNewMemHandle,
    NvRmPhysAddr *pNewMemAddr,
    NvU32 BufferSize)
{
    NvError Error = NvSuccess;
    NvRmMemHandle hNewMemHandle = NULL;
    static const NvRmHeap HeapProperty[] =
    {
        NvRmHeap_ExternalCarveOut,
        NvRmHeap_External,
        NvRmHeap_GART,
    };

    // Initialize the memory handle with NULL
    *phNewMemHandle = NULL;

    /// Create memory handle
    Error = NvRmMemHandleCreate(hDevice, &hNewMemHandle, BufferSize);

    // Allocates the memory from the sdram
    if (!Error)
        Error = NvRmMemAlloc(hNewMemHandle, HeapProperty,
                        NV_ARRAY_SIZE(HeapProperty), 4, NvOsMemAttribute_Uncached);

    // Pin the memory allocation so that it should not move by memory manager.
    if (!Error)
        *pNewMemAddr = NvRmMemPin(hNewMemHandle);

    // If error then free the memory allocation and memory handle.
    if (Error)
    {
        NvRmMemHandleFree(hNewMemHandle);
        hNewMemHandle = NULL;
    }

    *phNewMemHandle = hNewMemHandle;
    return Error;
}

 /**
  * Destroy the dma buffer memory handle.
  */
static void DestroyDmaBufferMemoryHandle(NvRmMemHandle hMemHandle)
{
    // Can accept the null parameter. If it is not null then only destroy.
    if (hMemHandle)
    {
        // Unpin the memory allocation.
        NvRmMemUnpin(hMemHandle);

        // Free the memory handle.
        NvRmMemHandleFree(hMemHandle);
    }
}

static NvError
CreateDmaTransferBuffer(
    NvRmDeviceHandle hRmDevice,
    NvRmMemHandle *phRmMemory,
    NvRmPhysAddr *pBuffPhysAddr,
    void **pBuffPtr,
    NvU32 BufferSize)
{
    NvError Error = NvSuccess;
    NvRmMemHandle hRmMemory = NULL;
    NvRmPhysAddr BuffPhysAddr;

    // Reset all the members realted to the dma buffer.
    BuffPhysAddr = 0;

    *phRmMemory = NULL;
    *pBuffPtr = (void *)NULL;
    *pBuffPhysAddr = 0;

    // Create the dma buffer memory for receive and transmit.
    // It will be double of the OneBufferSize
    Error = CreateDmaBufferMemoryHandle(hRmDevice, &hRmMemory, 
                                                &BuffPhysAddr, BufferSize);
    if (!Error)
    {
        // 0 to OneBufferSize-1 is buffer 1 and OneBufferSize to 2*OneBufferSize
        // is second buffer.
        Error = NvRmMemMap(hRmMemory, 0, BufferSize, 
                                                NVOS_MEM_READ_WRITE, pBuffPtr);
        // If error then free the allocation and reset all changed value.
        if (Error)
        {
            DestroyDmaBufferMemoryHandle(hRmMemory);
            hRmMemory = NULL;
            *pBuffPtr = (void *)NULL;
            return Error;
        }
        *phRmMemory = hRmMemory;
        *pBuffPhysAddr = BuffPhysAddr;
    }
    return Error;
}

/**
 * Destroy the dma transfer buffer.
 */
static void
DestroyDmaTransferBuffer(
    NvRmMemHandle hRmMemory,
    void *pBuffPtr,
    NvU32 BufferSize)
{
    if (hRmMemory)
    {
        if (pBuffPtr)
            NvRmMemUnmap(hRmMemory, pBuffPtr, BufferSize);
        DestroyDmaBufferMemoryHandle(hRmMemory);
    }
}

static NvError SetPowerControl(NvSnorHandle hSnor, NvBool IsEnable)
{
    NvError Error = NvError_Success;
    NvRmModuleID ModuleId;

    ModuleId = NVRM_MODULE_ID(NvRmModuleID_SyncNor, 0);
    if (IsEnable)
    {
        // Enable power for Snor module
        if (!Error)
            Error = NvRmPowerVoltageControl(hSnor->hRmDevice, ModuleId,
                            hSnor->RmPowerClientId,
                            NvRmVoltsUnspecified, NvRmVoltsUnspecified,
                            NULL, 0, NULL);

        // Enable the clock.
        if (!Error)
            Error = NvRmPowerModuleClockControl(hSnor->hRmDevice, ModuleId, 
                        hSnor->RmPowerClientId, NV_TRUE);
    }
    else
    {
        // Disable the clocks.
        Error = NvRmPowerModuleClockControl(
                                    hSnor->hRmDevice,
                                    ModuleId, hSnor->RmPowerClientId,
                                    NV_FALSE);
        
        // Disable the power to the controller.
        if (!Error)
            Error = NvRmPowerVoltageControl(
                                    hSnor->hRmDevice,
                                    ModuleId,
                                    hSnor->RmPowerClientId,
                                    NvRmVoltsOff,
                                    NvRmVoltsOff,
                                    NULL,
                                    0,
                                    NULL);
        NV_ASSERT(Error == NvError_Success);
    }
    return Error;
}                

NvError InitSnorInformation(void)
{
    NvSnorInformation *pSnorInfo;
    
    if(!g_pSnorInfo)
    {
        pSnorInfo = NvOsAlloc(sizeof(*pSnorInfo));
        if (!pSnorInfo)
        {
            //printk("InitSnorInformation InsufficientMemory!!\n");
            return NvError_InsufficientMemory;
        }
        NvOsMemset(pSnorInfo, 0, sizeof(*pSnorInfo));
    
        pSnorInfo->hRmDevice = s_hRmGlobal;
        NV_ASSERT(pSnorInfo->hRmDevice);

        pSnorInfo->hSnor = NULL;

        
        if (NvOsAtomicCompareExchange32((NvS32*)&g_pSnorInfo, 0, (NvS32)pSnorInfo)!=0)
        {
            NvOsFree(pSnorInfo);
            return NvError_BadParameter;
        }
    }
    mutex_init(&snor_operation_lock);
    mutex_init(&snor_hal_lock);
    return NvError_Success;
}

void __init tegra_init_snor_controller(void)
{
    NvError Error = NvError_Success;
    Error = InitSnorInformation();
    printk("tegra_init_snor_controller returnes %x!!\n", Error);
    NV_ASSERT(Error==NvError_Success);
    
}

#if 0
void DeinitSnorInformation(void)
{
    if(g_pSnorInfo)
    {
        NvOsFree(g_pSnorInfo);
    }
}
#endif

void InitSnorController(NvSnorHandle hSnor, NvU32 DevTypeSNOREn, SnorControllerTimingRegVals TimingRegVals)
{
//#define NOR_CONTROLLER_TIMING_SET 1 //TODO Need to check with HW engineer to define timing value of DPRAM device.

    NvU32 SnorConfig = hSnor->SnorRegs.Config;
#ifdef NOR_CONTROLLER_TIMING_SET
    NvU32 SnorTiming0 = hSnor->SnorRegs.Timing0;
    NvU32 SnorTiming1 = hSnor->SnorRegs.Timing1;
#endif
     
    // Set device type to NOR. With this type setting, can access DPRAM, too.
if (DevTypeSNOREn==NV_TRUE)
    SnorConfig = NV_FLD_SET_DRF_DEF(SNOR, CONFIG, NOR_DEVICE_TYPE, SNOR, SnorConfig);
else
    SnorConfig = NV_FLD_SET_DRF_DEF(SNOR, CONFIG, NOR_DEVICE_TYPE, MUXONENAND, SnorConfig);

    SnorConfig = NV_FLD_SET_DRF_DEF(SNOR, CONFIG, WORDWIDE_GMI, NOR16BIT, SnorConfig); //TODO Add configuring the 16/32 bit mode of snor controller. 
    SnorConfig = NV_FLD_SET_DRF_DEF(SNOR, CONFIG, MUXMODE_GMI, AD_MUX, SnorConfig); //TODO Add configuring the mux/nonmux mode.
    
    SnorConfig = NV_FLD_SET_DRF_DEF(SNOR, CONFIG, DEVICE_MODE, ASYNC, SnorConfig); 


    //TODO Timing Register Setting. Will be better to query timing from ODM interface.
    //Need to control SNOR Timing0 Register and SNOR Timing1 Register.

#ifdef NOR_CONTROLLER_TIMING_SET
    SnorTiming0 = NV_FLD_SET_DRF_NUM(SNOR, TIMING0, MUXED_WIDTH, TimingRegVals.Muxed_Width, SnorTiming0); 
    SnorTiming0 = NV_FLD_SET_DRF_NUM(SNOR, TIMING0, HOLD_WIDTH, TimingRegVals.Hold_Width, SnorTiming0); 
    SnorTiming0 = NV_FLD_SET_DRF_NUM(SNOR, TIMING0, ADV_WIDTH, TimingRegVals.ADV_dWidth, SnorTiming0); 

    SnorTiming1 = NV_FLD_SET_DRF_NUM(SNOR, TIMING1, WE_WIDTH, TimingRegVals.WE_Width, SnorTiming1); 
    SnorTiming1 = NV_FLD_SET_DRF_NUM(SNOR, TIMING1, OE_WIDTH, TimingRegVals.OE_Width, SnorTiming1); 
    SnorTiming1 = NV_FLD_SET_DRF_NUM(SNOR, TIMING1, WAIT_WIDTH, TimingRegVals.Wait_Width, SnorTiming1); 
#endif
    
    hSnor->SnorRegs.Config = SnorConfig;
    SNOR_WRITE32(hSnor->pSnorControllerVirtBaseAdd, CONFIG, SnorConfig);

#ifdef NOR_CONTROLLER_TIMING_SET
    hSnor->SnorRegs.Timing0 = SnorTiming0;
    SNOR_WRITE32(hSnor->pSnorControllerVirtBaseAdd, TIMING0, SnorTiming0);
    hSnor->SnorRegs.Timing1 = SnorTiming1;
    SNOR_WRITE32(hSnor->pSnorControllerVirtBaseAdd, TIMING1, SnorTiming1);
#endif
    //TODO ... In nvodm_query_pinmux.c, right pinmux for SyncNor need to be defined in NvOdmQueryPinMux. 
}

void SetChipSelect(NvSnorHandle hSnor, NvU32 ChipSelId)
{
    NvU32 SnorConfig;

    NV_ASSERT(ChipSelId<SNOR_CONTROLLER_CHIPSELECT_MAX);
    
    SnorConfig = hSnor->SnorRegs.Config;
    SnorConfig = NV_FLD_SET_DRF_NUM(SNOR, CONFIG, SNOR_SEL, ChipSelId, SnorConfig);

    hSnor->SnorRegs.Config = SnorConfig;
    SNOR_WRITE32(hSnor->pSnorControllerVirtBaseAdd, CONFIG, SnorConfig);
}

NvError CreateSnorHandle(
        NvRmDeviceHandle hRmDevice,
        NvSnorHandle *phSnor)
{
    NvError Error = NvError_Success;
    NvSnorHandle HandleSnor;
    SnorControllerTimingRegVals TimingRegValues = {0x1, 0x1, 0x1, 0x3, 0x3, 0x8};  

    HandleSnor = NvOsAlloc(sizeof(HandleSnor));
    if (!HandleSnor)
    {
        return NvError_InsufficientMemory;
    }
    
    NV_ASSERT(hRmDevice);
    
    HandleSnor->hRmDevice = hRmDevice;
    HandleSnor->OpenCount = 0;
    HandleSnor->SnorControllerBaseAdd = 0;
    HandleSnor->pSnorControllerVirtBaseAdd = NULL;

    HandleSnor->SnorRegMapSize=0;
    
    HandleSnor->hRmPowerEventSema = NULL;
    HandleSnor->RmPowerClientId = 0;
    HandleSnor->hCommandCompleteSema = NULL;
    HandleSnor->hIntr = NULL;
    
    HandleSnor->ConnectedDevReg.DeviceBaseAddress = 0;
    HandleSnor->ConnectedDevReg.DeviceAddressSize = 0;
    HandleSnor->ConnectedDevReg.pDeviceBaseVirtAddress = NULL;
    HandleSnor->ConnectedDevReg.DevicePureAddress = 0;
    
    HandleSnor->SnorRegs.Config = NV_RESETVAL(SNOR, CONFIG);
    HandleSnor->SnorRegs.Status = NV_RESETVAL(SNOR, STA);
    HandleSnor->SnorRegs.NorAddressPtr = NV_RESETVAL(SNOR, NOR_ADDR_PTR);
    HandleSnor->SnorRegs.AhbAddrPtr = NV_RESETVAL(SNOR, AHB_ADDR_PTR);
    HandleSnor->SnorRegs.Timing0 = NV_RESETVAL(SNOR, TIMING0);
    HandleSnor->SnorRegs.Timing1 = NV_RESETVAL(SNOR, TIMING1);
    HandleSnor->SnorRegs.MioCfg = NV_RESETVAL(SNOR, MIO_CFG);
    HandleSnor->SnorRegs.MioTiming = NV_RESETVAL(SNOR, MIO_TIMING0);
    HandleSnor->SnorRegs.DmaConfig = NV_RESETVAL(SNOR, DMA_CFG);
    HandleSnor->SnorRegs.ChipSelectMuxConfig = NV_RESETVAL(SNOR, CS_MUX_CFG);

    HandleSnor->Snor_DmaBufSize = SNOR_DMA_BUFFER_SIZE_BYTE;  

    // Initialize the base addresses of SNOR controller
    NvRmModuleGetBaseAddress(hRmDevice, NVRM_MODULE_ID(NvRmModuleID_SyncNor, 0),
                            &HandleSnor->SnorControllerBaseAdd,
                            &HandleSnor->SnorRegMapSize);
    
    Error = NvRmPhysicalMemMap(HandleSnor->SnorControllerBaseAdd,
                                HandleSnor->SnorRegMapSize,
                                NVOS_MEM_READ_WRITE, NvOsMemAttribute_Uncached,
                                (void **)&(HandleSnor->pSnorControllerVirtBaseAdd));
    
    if (Error)
    {
        printk("SNOR Controller register mapping fails!!\n");
        goto ErrorEnd;
    }


    
    // Initialize the base addresses of device which is connected to SNOR controller. 
    // For example, NOR device, Dual-port RAM.
    // Then memory read/write from/to pDeviceBaseVirtAddress will be done through GMI_ADxx lines.
    NvRmModuleGetBaseAddress(hRmDevice, 
                            NVRM_MODULE_ID(NvRmModuleID_Nor, 0),
                            &HandleSnor->ConnectedDevReg.DeviceBaseAddress,
                            &HandleSnor->ConnectedDevReg.DeviceAddressSize);
    
    printk("During Nor Base Register mapping...the size of DeviceAddressSize: %d (0x%x) !!\n", 
            HandleSnor->ConnectedDevReg.DeviceAddressSize, HandleSnor->ConnectedDevReg.DeviceAddressSize);

    
    
    Error = NvRmPhysicalMemMap(HandleSnor->ConnectedDevReg.DeviceBaseAddress,
                                HandleSnor->ConnectedDevReg.DeviceAddressSize,
                                NVOS_MEM_READ_WRITE, NvOsMemAttribute_Uncached,
                                (void **)&(HandleSnor->ConnectedDevReg.pDeviceBaseVirtAddress));
    
    if (Error)
    {
        printk("Device Base address mapping fails!!\n");
        goto ErrorEnd;
    }
    

    // Register as the Rm power client
    Error = NvOsSemaphoreCreate(&HandleSnor->hRmPowerEventSema, 0);
    if (!Error)
        Error = NvRmPowerRegister(HandleSnor->hRmDevice, 
                        HandleSnor->hRmPowerEventSema,
                        &HandleSnor->RmPowerClientId);

    if (!Error)
        Error =  SetPowerControl(HandleSnor, NV_TRUE);

    if (!Error)
    {
        InitSnorController(HandleSnor, NV_TRUE, TimingRegValues);
        SetChipSelect(HandleSnor, SNOR_CS_FOR_DPRAM);   
    }
    // Reset the snor controller.
    if (!Error)
        NvRmModuleReset(hRmDevice, NVRM_MODULE_ID(NvRmModuleID_SyncNor, 0));


    if (!Error)
        Error = RegisterSnorInterrupt(HandleSnor);


    if (!Error)
        Error = NvOsSemaphoreCreate(&HandleSnor->hCommandCompleteSema, 0);

    if (!Error)
        Error = CreateDmaTransferBuffer(HandleSnor->hRmDevice, &HandleSnor->hRmMemory, 
            &HandleSnor->DmaBuffPhysAdd, (void **)&HandleSnor->pAhbDmaBuffer,
            HandleSnor->Snor_DmaBufSize );

    ErrorEnd:       
    // If error then destroy all the allocation done here.
    if (Error)
    {
        DestroySnorHandle(HandleSnor);
        HandleSnor = NULL;
    }
    *phSnor = HandleSnor;
    return Error;
  
}

void DestroySnorHandle(NvSnorHandle hSnor)
{
    if (!hSnor) 
    {
        return;
    }

    if (hSnor->RmPowerClientId)
    {
        SetPowerControl(hSnor, NV_FALSE);
        // Unregister for the power manager.
        NvRmPowerUnRegister(hSnor->hRmDevice, hSnor->RmPowerClientId);
    }
    if (hSnor->hRmPowerEventSema) NvOsSemaphoreDestroy(hSnor->hRmPowerEventSema);

    if (hSnor->hIntr)
    {
        NvRmInterruptUnregister(hSnor->hRmDevice, hSnor->hIntr);
        hSnor->hIntr = NULL;
    }

    // Unmap the virtual mapping of the snor controller.
    if (hSnor->pSnorControllerVirtBaseAdd)
        NvRmPhysicalMemUnmap(hSnor->pSnorControllerVirtBaseAdd, hSnor->SnorRegMapSize);

    // Unmap the virtual mapping of the Nor interfacing register.
    if (hSnor->ConnectedDevReg.pDeviceBaseVirtAddress)
        NvRmPhysicalMemUnmap(hSnor->ConnectedDevReg.pDeviceBaseVirtAddress, 
                                    hSnor->ConnectedDevReg.DeviceAddressSize);
    if (hSnor->hCommandCompleteSema) NvOsSemaphoreDestroy(hSnor->hCommandCompleteSema);

    if (hSnor->pAhbDmaBuffer) DestroyDmaTransferBuffer(hSnor->hRmMemory, hSnor->pAhbDmaBuffer,
                hSnor->Snor_DmaBufSize);
    
    if (hSnor) NvOsFree(hSnor);

    if (g_pSnorInfo->hSnor->OpenCount) g_pSnorInfo->hSnor->OpenCount=0;
    if (g_pSnorInfo->hSnor) g_pSnorInfo->hSnor=NULL;
    
    
}


void NvReadViaSNORControllerDMA (NvSnorHandle hSnor, void* SnorAddr, NvU32 word32bit_count)
{
    mutex_lock(&snor_hal_lock);
    //SetChipSelect(hSnor, SNOR_CS_FOR_DPRAM);  --> Move to CreateHandle part.
    
    //DMA READ operation
    //GO_NOR field of configuration reg is already set in InitSnorController.
    
    NvU32 NorAddressPtr = hSnor->SnorRegs.NorAddressPtr;
    NvU32 AhbAddrPtr = hSnor->SnorRegs.AhbAddrPtr;
    NvU32 DmaConfig = hSnor->SnorRegs.DmaConfig;
    NvU32 SnorConfig = hSnor->SnorRegs.Config;
    NvU32 StatusReg = 0;
    
    NorAddressPtr = NV_FLD_SET_DRF_NUM(SNOR, NOR_ADDR_PTR, SNOR_NOR_ADDR_PTR, (NvU32)SnorAddr, NorAddressPtr); 
    AhbAddrPtr = NV_FLD_SET_DRF_NUM(SNOR, AHB_ADDR_PTR, SNOR_AHB_ADDR_PTR, (NvU32)(hSnor->DmaBuffPhysAdd), AhbAddrPtr); 

    SnorConfig = NV_FLD_SET_DRF_DEF(SNOR, CONFIG, GO_NOR, ENABLE, SnorConfig);
    SnorConfig = NV_FLD_SET_DRF_DEF(SNOR, CONFIG, MST_ENB, ENABLE, SnorConfig);

    hSnor->SnorRegs.NorAddressPtr = NorAddressPtr;
    SNOR_WRITE32(hSnor->pSnorControllerVirtBaseAdd, NOR_ADDR_PTR, NorAddressPtr);

    hSnor->SnorRegs.AhbAddrPtr = AhbAddrPtr;
    SNOR_WRITE32(hSnor->pSnorControllerVirtBaseAdd, AHB_ADDR_PTR, AhbAddrPtr);

    DmaConfig = NV_FLD_SET_DRF_DEF(SNOR, DMA_CFG, DMA_GO, ENABLE, DmaConfig);
    DmaConfig = NV_FLD_SET_DRF_DEF(SNOR, DMA_CFG, DIR, NOR2AHB, DmaConfig);

    DmaConfig = NV_FLD_SET_DRF_DEF(SNOR, DMA_CFG, IE_DMA_DONE, ENABLE, DmaConfig);
    if ((word32bit_count&0x0007)==0) //This means word counts is 8, 16, 24, ...
    {
        DmaConfig = NV_FLD_SET_DRF_DEF(SNOR, DMA_CFG, BURST_SIZE, BS8WORD, DmaConfig);
    }
    else if ((word32bit_count&0x0003)==0) //This means word counts is 4, 12, 20, ...
    {
        DmaConfig = NV_FLD_SET_DRF_DEF(SNOR, DMA_CFG, BURST_SIZE, BS4WORD, DmaConfig);
    }
     
    else DmaConfig = NV_FLD_SET_DRF_DEF(SNOR, DMA_CFG, BURST_SIZE, BS1WORD, DmaConfig);  
    
    DmaConfig = NV_FLD_SET_DRF_NUM(SNOR, DMA_CFG, WORD_COUNT, word32bit_count-1, DmaConfig); //Controller transfer total N+1 words.

    hSnor->SnorRegs.Config = SnorConfig;
    hSnor->SnorRegs.DmaConfig = DmaConfig;

    SNOR_WRITE32(hSnor->pSnorControllerVirtBaseAdd, CONFIG, SnorConfig);
    SNOR_WRITE32(hSnor->pSnorControllerVirtBaseAdd, DMA_CFG, DmaConfig);
    
    NvOsSemaphoreWait(hSnor->hCommandCompleteSema);
    
    mutex_unlock(&snor_hal_lock);
}

void NvWriteViaSNORControllerDMA (NvSnorHandle hSnor, void* SnorAddr, NvU32 word32bit_count)
{
    mutex_lock(&snor_hal_lock);
    //SetChipSelect(hSnor, SNOR_CS_FOR_DPRAM);  --> Move to CreateHandle part.
    //
    //DMA WRITE operation
    //GO_NOR field of configuration reg is already set in InitSnorController.
    
    NvU32 NorAddressPtr = hSnor->SnorRegs.NorAddressPtr;
    NvU32 AhbAddrPtr = hSnor->SnorRegs.AhbAddrPtr;
    NvU32 DmaConfig = hSnor->SnorRegs.DmaConfig;
    NvU32 SnorConfig = hSnor->SnorRegs.Config;
    NvU32 StatusReg = 0;

    
    NorAddressPtr = NV_FLD_SET_DRF_NUM(SNOR, NOR_ADDR_PTR, SNOR_NOR_ADDR_PTR, (NvU32)SnorAddr, NorAddressPtr); 
    AhbAddrPtr = NV_FLD_SET_DRF_NUM(SNOR, AHB_ADDR_PTR, SNOR_AHB_ADDR_PTR, (NvU32)(hSnor->DmaBuffPhysAdd), AhbAddrPtr); 

    
    SnorConfig = NV_FLD_SET_DRF_DEF(SNOR, CONFIG, GO_NOR, ENABLE, SnorConfig);
    SnorConfig = NV_FLD_SET_DRF_DEF(SNOR, CONFIG, MST_ENB, ENABLE, SnorConfig);

    hSnor->SnorRegs.NorAddressPtr = NorAddressPtr;
    SNOR_WRITE32(hSnor->pSnorControllerVirtBaseAdd, NOR_ADDR_PTR, NorAddressPtr);

    hSnor->SnorRegs.AhbAddrPtr = AhbAddrPtr;
    SNOR_WRITE32(hSnor->pSnorControllerVirtBaseAdd, AHB_ADDR_PTR, AhbAddrPtr);

    
    DmaConfig = NV_FLD_SET_DRF_DEF(SNOR, DMA_CFG, DMA_GO, ENABLE, DmaConfig);
    DmaConfig = NV_FLD_SET_DRF_DEF(SNOR, DMA_CFG, DIR, AHB2NOR, DmaConfig);
    DmaConfig = NV_FLD_SET_DRF_DEF(SNOR, DMA_CFG, IE_DMA_DONE, ENABLE, DmaConfig);
    if ((word32bit_count&0x0007)==0) //This means word counts is 8, 16, 24, ...
    {
        DmaConfig = NV_FLD_SET_DRF_DEF(SNOR, DMA_CFG, BURST_SIZE, BS8WORD, DmaConfig);
    }
    else if ((word32bit_count&0x0003)==0) //This means word counts is 4, 12, 20, ...
    {
        DmaConfig = NV_FLD_SET_DRF_DEF(SNOR, DMA_CFG, BURST_SIZE, BS4WORD, DmaConfig);
    }
    else DmaConfig = NV_FLD_SET_DRF_DEF(SNOR, DMA_CFG, BURST_SIZE, BS1WORD, DmaConfig);  
    DmaConfig = NV_FLD_SET_DRF_NUM(SNOR, DMA_CFG, WORD_COUNT, word32bit_count-1, DmaConfig); //Controller transfer total N+1 words.
    
    hSnor->SnorRegs.Config = SnorConfig;
    hSnor->SnorRegs.DmaConfig = DmaConfig;
    
    SNOR_WRITE32(hSnor->pSnorControllerVirtBaseAdd, CONFIG, SnorConfig);
    SNOR_WRITE32(hSnor->pSnorControllerVirtBaseAdd, DMA_CFG, DmaConfig);
    
    NvOsSemaphoreWait(hSnor->hCommandCompleteSema);

    mutex_unlock(&snor_hal_lock);
}
