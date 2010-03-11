/*
 * Copyright (c) 2007-2009 NVIDIA Corporation.
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

/**
 * @file
 * @brief <b>NVIDIA Driver Development Kit:
 *                  Uart ddk Driver implementation</b>
 *
 * @b Description: Implementation of the NvDdk UART API.
 *
 * Some constrains and designs details of the uart driver:
 *
 * We use the continuous double buffering mechanims for the uart receive. 
 * In this method, the apb dma is programmed for the receive in contunous mode* 
 * and generte the interrupt after half of the buffer completion.
 * Currently the buffer size is 64KB  and so dma generate the interrupt after
 * every 32 K transfer. So the uart ddk clinet need to copy this dma buffer
 * to local buffer before next 32K bytes received. For 4.5Mbps rate, the 32 K 
 * transfer takes around 70ms and we have the only latency of 70ms to copy this 
 * buffer. For lower baudrate, the latency is more.
 *
 * In the receive path, there are three possible errors:
 * - FIFO overrun error: If there is no space in the Rx FIFO (16 bytes in uart 
 *                       controller) and if data arrives then we get this error. 
 *                       As we are using the apb dma in double continuous mode 
 *                       with the trigger level of 4 bytes so whenever there 
 *                       is 4 bytes in the fifo, the apb dma transfers this to 
 *                       memory. This is done by hw and sw does not get involved. 
 *                       This transfer is continuous. As hw is transferring the 
 *                       data from the fifo to memory, there is very less 
 *                       chances of getting the overrun error. We may get only 
 *                       when system bus is too much loaded, or 
 *                       ahb/apb/cpu/memory frequencies are too low or any 
 *                       system bandwidth issue. We have verified 4.5Mbps baud 
 *                       rate maximum and not seen any such issue.

 * - Dma buffer overwrite: There is 64KB of dma buffer on which data is filled 
 *                      from fifo. This is continuous mode filling and dma 
 *                      generates the interrupt after every 32Kbyte of transfers. 
 *                      So we have to respond this interrupt till next 32K data 
 *                      receive and copy this data to local buffer so that dma 
 *                      can fill again on this dma buffer. if we don't copy the 
 *                      data in a given time, the dma will overwrite this data 
 *                      with the new arriving data and so lose the old data. So 
 *                      responding the interrupt in a given time line is necessary
 *                      and it defines the maximum interrupt latency for the 
 *                      uart communication. If system does not respond in the 
 *                      given time, the system will not perform as expected. 

 * - Local buffer full: As ddk copies the incoming data from dma buffer to 
 *                      local buffer so that it can be supplied to client when 
 *                      client calls the read. If data are arriving continuously 
 *                      and data is not being read then buffer will get filled. 
 *                      If we have the hw flow control then sw deassert the RTS
 *                      so that no more data can be send from other end. It will 
 *                      again assert when the client calls the read and there 
 *                      is some threshold level of the space. Currently in ddk 
 *                      it is set to 48K minimum so if there is data available 
 *                      less than 48K in the local buffer then only RTS will be 
 *                      enabled otherwise it will be deasserted. This RTS control
 *                      is done through sw (driver) and so system should 
 *                      response the IST/high priority thread in a given time. 
 *
 * Moreover in the cuttent design, the DMA IST/UART IST priorities are high on 
 * the system as these are ISR. The Rx thread priority in the uart shim driver 
 * should be the IST priority level. 
 *
 * If the system responds the threads in a given time then we will not see any 
 * data loss/overrun type of error. 
 *
 * What happen if operating system is unable to schedule the Nvidia UART driver 
 * routines in a timely fashion or is mishandling hardware handshaking. 
 * Specifically, it is necessary for the driver to de-assert RTS when it is in 
 * danger of a FIFO overrun, 16 bytes max, to inform the sender that it should 
 * hold off on further transmissions. 
 * 
 * Answer:
 * If OS is unable to respond the IST/higher priority thread on a given time 
 * line then system is too loaded and need to redesign the application/system
 * because it is not meeting the real time requirements for the device (even it
 * is soft real time system). 
 * We don't have the RTS control enable from the hw and so sw control the RTS 
 * based on the driver buffer state/internal state. For this the thread should 
 * be scheduled properly so that proper flow control can be controlled. 
 * Even in double buffering mechanism, the hw control of RTS is also not useful 
 * as the hw does not know your buffer overwrite issue. 
 * We are setting the RTS activation/deactivation control to the size of half 
 * local buffer (minimum local buffer size is 96K). So if there is data in the 
 * local buffer more than half the local buffer, the RTS will be deactivated 
 * and if it's less, then only it's activated. This water level mechanism saves 
 * the 'out of local buffer issue'. 
 */

#include "nvddk_uart.h"
#include "ddkuart_hw_private.h"
#include "nvrm_power.h"
#include "nvrm_memmgr.h"
#include "nvrm_dma.h"
#include "nvodm_query.h"
#include "nvrm_interrupt.h"
#include "nvassert.h"
#include "nvrm_hardware_access.h"
#include "nvodm_query_pinmux.h"
#include "nvrm_pinmux.h"
#include "nvodm_uart.h"

// Some important debug print to debug the flow
#define DEBUG_WRITE_TIMEOUT 0
#define DEBUG_READ_RXERROR 0
#define ENABLE_WRITE_PROFILE 0
#define ENABLE_READ_PROFILE 0
// Constants used to size arrays.  Must be >= the max of all chips.
enum {MAX_UART_CONTROLLERS = 5};

// Assuming 50 ms data size for the 4.5Mbps
// Assuming the 50 ms will be the time taken to copy the recv data into the
// Passed buffer.
enum {UART_RX_DMA_PING_BUFFER_SIZE = 0x8000};

// Minimum size of the local buffering of the data to avoid any data loss.
enum {UART_RX_LOCAL_BUFFER_SIZE_MIN = UART_RX_DMA_PING_BUFFER_SIZE*3};

// Local buffer threshold for disabling the rts line.
// This is the 2/3rd of the local buffer size.
enum {UART_RX_LOCAL_BUFFER_THRESOLD_MIN = UART_RX_DMA_PING_BUFFER_SIZE << 1};

// maximum frequency boost list
enum {MAX_FREQ_REQ_LIST = 3};

// Time from last operation on read/write complete on uart port.
// If the time is more than the limit then uart will request for the
// frequency down.
enum {HIGH_FREQUENCY_HOLD_TIMEOUT = 3000}; // 3 seconds

// Mutex definition which are used for the locking the read/write/channel 
// register access.
#define DeclareReadFlowLock NvOsIntrMutexHandle hReadMutex
#define CreateReadFlowLock(hUart) NvOsIntrMutexCreate(&hUart->hReadMutex)
#define LockReadFlow(hUart) NvOsIntrMutexLock(hUart->hReadMutex)
#define UnlockReadFlow(hUart) NvOsIntrMutexUnlock(hUart->hReadMutex)
#define DestroyReadFlowLock(hUart) NvOsIntrMutexDestroy(hUart->hReadMutex)

#define DeclareWriteFlowLock NvOsIntrMutexHandle hWriteMutex
#define CreateWriteFlowLock(hUart) NvOsIntrMutexCreate(&hUart->hWriteMutex)
#define LockWriteFlow(hUart) NvOsIntrMutexLock(hUart->hWriteMutex)
#define UnlockWriteFlow(hUart) NvOsIntrMutexUnlock(hUart->hWriteMutex)
#define DestroyWriteFlowLock(hUart) NvOsIntrMutexDestroy(hUart->hWriteMutex)

#define DeclareRegisterAccessLock NvOsIntrMutexHandle hRegisterAccessMutex
#define CreateRegisterAccessLock(hUart) NvOsIntrMutexCreate(&hUart->hRegisterAccessMutex)
#define LockRegisterAccess(hUart) NvOsIntrMutexLock(hUart->hRegisterAccessMutex)
#define UnlockRegisterAccess(hUart) NvOsIntrMutexUnlock(hUart->hRegisterAccessMutex)
#define DestroyRegisterAccessLock(hUart) NvOsIntrMutexDestroy(hUart->hRegisterAccessMutex)

// Way to reset the semaphore count.
#define ResetSemaphoreCount(hSema) \
            while(NvOsSemaphoreWaitTimeout(hSema, 0) != NvError_Timeout)

#if DEBUG_READ_RXERROR
#define DEBUG_PRINT_RX_ERR(Expr, Format) \
                    do { \
                        if (Expr) \
                        {   \
                            NvOsDebugPrintf Format; \
                        }   \
                    } while(0)
#else
#define DEBUG_PRINT_RX_ERR(Expr, Format) 
#endif

#if DEBUG_WRITE_TIMEOUT
#define DEBUG_PRINT_TX_ERR(Expr, Format) \
                    do { \
                        if (Expr) \
                        {   \
                            NvOsDebugPrintf Format; \
                        }   \
                    } while(0)
#else
#define DEBUG_PRINT_TX_ERR(Expr, Format) 
#endif

/**
 * Combines the SOC uart capability structure.
 */
typedef struct SocUartCapabilityRec
{
    // Tells whether the end of data interrupt is supported or not.
    NvBool IsEndOfDataIntSupported;

    // Tells whether the RTS hw flow control is supported or not.
    NvBool IsRtsHwFlowControlSupported;

    // Fifo depth of the uart controller.
    NvU32 FifoDepth;
} SocUartCapability;

/**
 * Combines the uart channel information.
 */
typedef struct 
{
    // Nv Rm device handles.
    NvRmDeviceHandle hRmDevice;

    // List of the uart handle exist in the ddk.
    NvDdkUartHandle hUartList[MAX_UART_CONTROLLERS];

    // Mutex for uart channel information.
    NvOsMutexHandle hUartOpenMutex;
} DdkUartChannelInfo;

/**
 * Combines the Uart transfer information for receive and transmit for the given
 * uart channel.
 */
typedef struct 
{
    // Tells whether transfer is going on or not?
    NvBool IsBusy;

    NvU8 *pReqBuffer;

    // Pointer to the current transfer.
    NvU8 *pCurrTransBuf;

    NvU32 BytesRequested;

    NvU32 BytesTransferred;

    // Bytes remaining to transfer.
    NvU32 BytesRemaining;

    // Current transfer status.
    NvError TransferStatus;

    // On complete semaphore Id which need to be signal after transfer
    // completion or abort on the transfer.
    NvOsSemaphoreHandle hOnCompleteSema;

    NvBool IsStopReq;
} UartTransferReqInfo;

/**
 * Uart local buffer information.
 */
typedef struct 
{
    // Pointer to the local buffer.
    NvU8 *pBuffPtr;

    // Read Index
    NvU32 ReadIndex;

    // Write index.
    NvU32 WriteIndex;

    // Total size of the buffer.
    NvU32 TotalBufferSize;

    // Higher buffer level from where the RTS is deactivated to stop the sender
    // as ddk is not able to receive more data.
    NvU32 BufferLevelHigh;

    // Total data available in the buffer.
    NvU32 DataAvailable;
    
    // Semaphore which will be signalled once the data is available in the
    // fifo/dma buffer. Client passed this semaphore Id.
    NvOsSemaphoreHandle hDataAvailableSema;

    // Semaphore which will be signalled when there is any error or break receive
    // in  the Rx line. Client passed this semaphore Id.
    NvOsSemaphoreHandle hLineStatusSema;
} UartLocalBuffer;

/**
 * Uart handle which combines all the information related to the given uart
 * channels
 */
typedef struct NvDdkUartRec
{
    // Nv Rm device handles.
    NvRmDeviceHandle hRmDevice;

    // Instance Id of the uart channels.
    NvU32 InstanceId;

    // Number of open count.
    NvU32 OpenCount;

    // The uart capability for this uart channel only.
    SocUartCapability SocUartCaps;

    // Number of uart interface lines physically available on board.
    NvU32 NumberOfInterfaceLine;
    
    // Uart configuration parameter.
    NvDdkUartConfiguarations UartConfig;

    // Uart hw register information.
    UartHwRegisters UartHwRegs;

    // Uart write transfer request information.
    UartTransferReqInfo WriteReq;

    // Read transfer request information.
    UartTransferReqInfo ReadReq;

    // Local buffer information which is used for receive.
    UartLocalBuffer LocalRxBuff;

    // Tells whether the dma support is available for this channel.
    // This can be false if the dma allocation or the dma buffer creation fails.
    NvBool IsDmaSupport;

    // Is receive is dma based transfer or not.
    NvBool IsRxApbDmaBased;

    // Is read started or not.
    NvBool IsDmaReadStarted;

    // Rx dma handles.
    NvRmDmaHandle hRxRmDma;

    // Tx dma handles.
    NvRmDmaHandle hTxRmDma;

    // Memory handle to create the uncache memory for dma transfer.
    NvRmMemHandle hRmMemory;

    // Rx Dma buffer address.
    NvRmPhysAddr RxDmaBuffPhysAdd;

    // Tx Dma buffer address.
    NvRmPhysAddr TxDmaBuffPhysAdd;

    NvU32 RxDmaBufferSize;
    NvU32 HalfRxDmaBufferSize;

    NvU32 TxDmaBufferSize;

    // Virtual pointer to the Rx dma buffer.
    NvU8 *pRxDmaBuffer;

    // Virtual pointer to the Tx dma buffer.
    NvU8 *pTxDmaBuffer;

    // Write dma request
    NvRmDmaClientBuffer WDmaReq;
    
    // Read dma request
    NvRmDmaClientBuffer RDmaReq;

    // The last read index from dma buffer to the local buffer.
    NvU32 LastReadIndex;

    // Is current write is polling
    NvBool IsWritePolling;

    // Maximum transmit bytes which can be send by polling.
    NvU32 MaxTxBytesPolling;

    // 4 character time in microseconds, the write will wait for this much of 
    // time when it founds that Tx fifo is full.
    NvU32 Char4TimeUs;
    
    // Receive Synchronous semaphore Id which need to be signalled.
    NvOsSemaphoreHandle hRxSynchSema;

    // Transmit Synchronous semaphore Id which need to be signalled.
    NvOsSemaphoreHandle hTxSynchSema;

    // Mutex to access the read flow.
    DeclareReadFlowLock;

    // Mutex to access the write flow.
    DeclareWriteFlowLock;

    // Mutex to access the register which are common for read/write/configuration.
    DeclareRegisterAccessLock;

    // Callback for signal change notification.
    NvDdkUartSignalChangeCallback hModemSigChngCallback;

    // Arguments to the signal change callback function.
    void *ModemSigChngHandlerArgs;

    // Clinet wants the notification on the signal change.
    NvU32 ModemSignalName;
    
    NvBool IsRtsFlowEnable;

    NvBool IsCtsFlowEnable;
    
    // Pointer to the uart information.
    DdkUartChannelInfo *pUartInfo;

    // Semaphore  for registering the client with the power manager.
    NvOsSemaphoreHandle hRmPowerEventSema;

    // Power client Id.
    NvU32 RmPowerClientId;

    // Interrupt handle
    NvOsInterruptHandle InterruptHandle;

    // Odm uart handle
    NvOdmUartHandle hOdmUart;

    // Busy hints for setting the required frequency of the system/cpu.
    NvRmDfsBusyHint UpFreq[MAX_FREQ_REQ_LIST];
    NvRmDfsBusyHint DownFreq[MAX_FREQ_REQ_LIST];

    // Thread to keep the power/frequency requirement on the channel.
    NvOsThreadHandle hUartPowerMgrThread;

    NvOsSemaphoreHandle hUartPowerMgrSema;

    NvBool IsStopPowerThread;

    NvBool IsFreqBoosted;

    NvBool IsSuspendedState;

    NvBool OldRtsState;

    NvBool OldDtrState;
} NvDdkUart;

static DdkUartChannelInfo *s_pUartInfo = NULL;

/**
 * Get the uart soc capability.
 *
 */
static NvError
UartGetSocCapabilities(
    NvRmDeviceHandle hRmDevice,
    NvU32 UartInstanceId,
    SocUartCapability *pUartSocCaps)
{
    NvRmModuleID ModuleId;
    static SocUartCapability s_SocUartCapsList[3];
    NvRmModuleCapability UartCapsList[] =
    {
        { 1, 0, 0, &s_SocUartCapsList[0] }, // Major.Minor = 1.0
        { 1, 1, 0, &s_SocUartCapsList[1] }, // Major.Minor = 1.1
        { 1, 2, 0, &s_SocUartCapsList[2] }, // Major.Minor = 1.2
    };
    SocUartCapability *pUartCaps = NULL;

    ModuleId = NVRM_MODULE_ID(NvRmModuleID_Uart, UartInstanceId);

    s_SocUartCapsList[0].IsEndOfDataIntSupported = NV_FALSE;
    s_SocUartCapsList[0].IsRtsHwFlowControlSupported = NV_FALSE;
    s_SocUartCapsList[0].FifoDepth = 16;

    s_SocUartCapsList[1].IsEndOfDataIntSupported = NV_TRUE;
    s_SocUartCapsList[1].FifoDepth = 16;

    s_SocUartCapsList[2].IsEndOfDataIntSupported = NV_TRUE;
    s_SocUartCapsList[2].FifoDepth = 32;

    // FIXEME!! HW Bug: The RTS hw flow control is not working when enabling
    // EORD interrupt.
    // As the EORD is more important feature then the Rts hw flow control,
    // disabling this feature.
    s_SocUartCapsList[1].IsRtsHwFlowControlSupported = NV_FALSE;
    s_SocUartCapsList[2].IsRtsHwFlowControlSupported = NV_FALSE;

    // Get the capability from modules files.
    NV_ASSERT_SUCCESS(NvRmModuleGetCapabilities(hRmDevice, ModuleId,
                           UartCapsList, NV_ARRAY_SIZE(UartCapsList),
                                    (void **)&pUartCaps));
    pUartSocCaps->IsEndOfDataIntSupported = pUartCaps->IsEndOfDataIntSupported;
    pUartSocCaps->IsRtsHwFlowControlSupported = pUartCaps->IsRtsHwFlowControlSupported;
    pUartSocCaps->FifoDepth = pUartCaps->FifoDepth;
    return NvSuccess;
}

/**
 * Initialize the uart information.
 * Thread safety: Caller responsibility.
 */
static NvError InitUartInformation(NvRmDeviceHandle hRmDevice)
{
    NvError Error = NvSuccess;
    DdkUartChannelInfo *pUartInfo = NULL;

    NV_ASSERT(NvRmModuleGetNumInstances(hRmDevice, NvRmModuleID_Uart) <= MAX_UART_CONTROLLERS);
    if (!s_pUartInfo)
    {
        // Allocate the memory for the uart information.
        pUartInfo = NvOsAlloc(sizeof(*pUartInfo));
        if (!pUartInfo)
            return NvError_InsufficientMemory;
        NvOsMemset(pUartInfo, 0, sizeof(*pUartInfo));

        // Initialize all the parameters.
        pUartInfo->hRmDevice = hRmDevice;

        // Create the mutex to accss the uart information.
        Error = NvOsMutexCreate(&pUartInfo->hUartOpenMutex);
        // If error exit till then destroy all allocations.
        if (Error)
        {
            NvOsFree(pUartInfo);
            pUartInfo = NULL;
            return Error;
        }    

        if (NvOsAtomicCompareExchange32((NvS32*)&s_pUartInfo, 0, (NvS32)pUartInfo)!=0)
        {
            NvOsMutexDestroy(pUartInfo->hUartOpenMutex);
            NvOsFree(pUartInfo);
        }
    }
    return NvSuccess;
}

/**
 * Create the dma buffer memory handle.
 */
static NvError
CreateDmaBufferMemoryHandle(
    NvRmDeviceHandle hRmDevice,
    NvRmMemHandle *phMemHandle,
    NvRmPhysAddr *pMemPhyAddr,
    NvU32 BufferSize)
{
    NvError Error = NvSuccess;
    NvRmMemHandle hNewMemHandle = NULL;

    // Initialize the memory handle with NULL
    *phMemHandle = NULL;

    /// Create memory handle
    Error = NvRmMemHandleCreate(hRmDevice, &hNewMemHandle, BufferSize);

    // Allocates the memory from the sdram
    if (!Error)
        Error = NvRmMemAlloc(hNewMemHandle, NULL, 0, 0x4, NvOsMemAttribute_Uncached);

    // Pin the memory allocation so that it should not move by memory manager.
    // Also get the physical address.
    if (!Error)
        *pMemPhyAddr = NvRmMemPin(hNewMemHandle);

    // If error then free the memory allocation and memory handle.
    if (Error)
    {
        NvRmMemHandleFree(hNewMemHandle);
        hNewMemHandle = NULL;
    }

    *phMemHandle = hNewMemHandle;
    return Error;
}

 /**
  * Destroy the dma buffer memory handle.
  * Thread safety: Caller responsibility.
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

/**
 * Create the dma transfer buffer for the given handles.
 * Thread safety: Caller responsibility.
 */
static NvError
CreateDmaTransferBuffer(
    NvRmDeviceHandle hRmDevice,
    NvRmMemHandle *phRmMemory,
    NvRmPhysAddr *pBuff1PhysAddr,
    void **pBuff1Ptr,
    NvRmPhysAddr *pBuff2PhysAddr,
    void **pBuff2Ptr,
    NvU32 OneBufferSize)
{
    NvError Error = NvSuccess;
    NvRmMemHandle hRmMemory = NULL;
    NvRmPhysAddr BuffPhysAddr;

    // Reset all the members related to the dma buffer.
    BuffPhysAddr = 0;

    *phRmMemory = NULL;
    *pBuff1Ptr = (void *)NULL;
    *pBuff1PhysAddr = 0;
    *pBuff2Ptr = (void *)NULL;
    *pBuff2PhysAddr = 0;

    // Create the dma buffer memory for receive and transmit.
    // It will be double of the OneBufferSize
    Error = CreateDmaBufferMemoryHandle(hRmDevice, &hRmMemory, &BuffPhysAddr, 
                                                            (OneBufferSize << 1));
    if (!Error)
    {
        // 0 to OneBufferSize-1 is buffer 1 and OneBufferSize to 2*OneBufferSize
        // is second buffer.
        Error = NvRmMemMap(hRmMemory, 0, OneBufferSize, 
                                        NVOS_MEM_READ_WRITE, pBuff1Ptr);
        if (!Error)
        {
            Error = NvRmMemMap(hRmMemory, OneBufferSize, OneBufferSize, 
                                        NVOS_MEM_READ_WRITE, pBuff2Ptr);
            if (Error)
                NvRmMemUnmap(hRmMemory, pBuff1Ptr, OneBufferSize);
        }            
        // If error then free the allocation and reset all changed value.
        if (Error)
        {
            DestroyDmaBufferMemoryHandle(hRmMemory);
            hRmMemory = NULL;
            *pBuff1Ptr = (void *)NULL;
            *pBuff2Ptr = (void *)NULL;
            return Error;
        }
        *phRmMemory = hRmMemory;
        *pBuff1PhysAddr = BuffPhysAddr;
        *pBuff2PhysAddr = BuffPhysAddr + OneBufferSize;
    }
    return Error;
}

/**
 * Destroy the dma transfer buffer.
 * Thread safety: Caller responsibility.
 */
static void
DestroyDmaTransferBuffer(
    NvRmMemHandle hRmMemory,
    void *pBuff1Ptr,
    void *pBuff2Ptr,
    NvU32 OneBufferSize)
{
    NvRmMemUnmap(hRmMemory, pBuff1Ptr, OneBufferSize);
    NvRmMemUnmap(hRmMemory, pBuff2Ptr, OneBufferSize);
    DestroyDmaBufferMemoryHandle(hRmMemory);
}

/**
 * Initialize the transfer information for the given handles.
 * Thread safety: Caller /responsibility.
 */
static void
InitTransferInfo(
    NvRmDeviceHandle hRmDevice,
    UartTransferReqInfo *pTransferInfo)
{
    NvOsMemset(pTransferInfo, 0, sizeof(UartTransferReqInfo));
    pTransferInfo->IsBusy = NV_FALSE;
    pTransferInfo->pReqBuffer = NULL;
    pTransferInfo->BytesRequested = 0;
    pTransferInfo->BytesTransferred = 0;
    pTransferInfo->pCurrTransBuf = NULL;
    pTransferInfo->BytesRemaining = 0;
    pTransferInfo->TransferStatus = NvSuccess;
    pTransferInfo->hOnCompleteSema = NULL;
}


/**
 * Create the local buffer.
 * Thread safety: Caller responsibility.
 */
static NvError CreateLocalRxBuff(NvDdkUartHandle hUart, NvU32 BufferSize)
{
    NvError Error = NvSuccess;

    hUart->LocalRxBuff.pBuffPtr = NULL;
    hUart->LocalRxBuff.ReadIndex = 0;
    hUart->LocalRxBuff.WriteIndex = 0;
    hUart->LocalRxBuff.DataAvailable = 0;
    hUart->LocalRxBuff.hDataAvailableSema = NULL;
    hUart->LocalRxBuff.TotalBufferSize = BufferSize;
    // Setting the RTS activation/deactivation thresold level to the half of the
    // buffer size.
    hUart->LocalRxBuff.BufferLevelHigh = (BufferSize >> 1);
    if (BufferSize)
    {
        hUart->LocalRxBuff.pBuffPtr = NvOsAlloc(BufferSize);
        if (!hUart->LocalRxBuff.pBuffPtr)
            Error = NvError_InsufficientMemory;
    }
    return Error;
}

/**
 * Destroy the local buffer.
 * Thread safety: Caller responsibility.
 */
static void DestroyLocalRxBuff(NvDdkUartHandle hUart)
{
    NvOsFree(hUart->LocalRxBuff.pBuffPtr);
    hUart->LocalRxBuff.pBuffPtr = NULL;
    hUart->LocalRxBuff.ReadIndex = 0;
    hUart->LocalRxBuff.DataAvailable = 0;
    hUart->LocalRxBuff.WriteIndex = 0;
    hUart->LocalRxBuff.hDataAvailableSema = NULL;
    hUart->LocalRxBuff.hLineStatusSema = NULL;
    hUart->LocalRxBuff.TotalBufferSize = 0;
    hUart->LocalRxBuff.BufferLevelHigh = 0;
}

/**
 * Handle the uart receive interrupt.
 * Thread safety: Caller responsibility.
 */
static void
HandleUartRxInterrupt(
    NvDdkUartHandle hUart,
    UartHwInterruptSource InterruptReason)
{
    NvError Error = NvSuccess;
    NvU32 SpaceFromWriteIndex;
    NvU32 SpaceFromStartIndex;
    NvU32 BytesRead;
    NvU32 IsNextRead = NV_TRUE;
    

    if (hUart->IsRxApbDmaBased)
    {
        LockRegisterAccess(hUart);
        NvDdkPrivUartHwSetReceiveInterrupt(&hUart->UartHwRegs, NV_FALSE, NV_TRUE);
        UnlockRegisterAccess(hUart);
        NvOsSemaphoreSignal(hUart->LocalRxBuff.hDataAvailableSema);
    }
    else
    {
        LockReadFlow(hUart);
        if (hUart->LocalRxBuff.ReadIndex > hUart->LocalRxBuff.WriteIndex)
        {
            // 0.....w....r...(n-1)
            SpaceFromWriteIndex = hUart->LocalRxBuff.ReadIndex -
                                        hUart->LocalRxBuff.WriteIndex;
            SpaceFromStartIndex = 0;
        }
        else
        {
            // 0.....r....w...(n-1)
            SpaceFromWriteIndex = hUart->LocalRxBuff.TotalBufferSize -
                                        hUart->LocalRxBuff.WriteIndex;
            SpaceFromStartIndex = hUart->LocalRxBuff.ReadIndex;
        }

        if (SpaceFromWriteIndex)
        {
            Error = NvDdkPrivUartHwReadFromReceiveFifo(&hUart->UartHwRegs,
                &hUart->LocalRxBuff.pBuffPtr[hUart->LocalRxBuff.WriteIndex], 
                SpaceFromWriteIndex, &BytesRead);
            if (BytesRead)
            {
                hUart->LocalRxBuff.WriteIndex += BytesRead;
                if (hUart->LocalRxBuff.WriteIndex >= hUart->LocalRxBuff.TotalBufferSize)
                    hUart->LocalRxBuff.WriteIndex = 0;
                hUart->LocalRxBuff.DataAvailable += BytesRead;
            }
            if (BytesRead < SpaceFromWriteIndex)
                IsNextRead = NV_FALSE;
        }
        if ((!Error) && (IsNextRead) && (SpaceFromStartIndex))
        {
            Error = NvDdkPrivUartHwReadFromReceiveFifo(&hUart->UartHwRegs,
                &hUart->LocalRxBuff.pBuffPtr[hUart->LocalRxBuff.WriteIndex], 
                SpaceFromStartIndex, &BytesRead);
            if (BytesRead)
            {
                hUart->LocalRxBuff.WriteIndex += BytesRead;
                if (hUart->LocalRxBuff.WriteIndex >= hUart->LocalRxBuff.TotalBufferSize)
                    hUart->LocalRxBuff.WriteIndex = 0;
                hUart->LocalRxBuff.DataAvailable += BytesRead;
            }
        }

        // If flow control is enabled and data available is crossed the thresold
        // level then disable the rts so there will not be any incoming data.
        if (hUart->LocalRxBuff.DataAvailable >= hUart->LocalRxBuff.BufferLevelHigh)
        {
            if (hUart->IsRtsFlowEnable)
            {
                LockRegisterAccess(hUart);
                NvDdkPrivUartHwSetModemSignal(&hUart->UartHwRegs, 
                                        NvDdkUartSignalName_Rts, NV_FALSE);
                UnlockRegisterAccess(hUart);
            }                                        
        }
        UnlockReadFlow(hUart);
        
        // Update the transfer status.
        hUart->ReadReq.TransferStatus = Error;
        NvOsSemaphoreSignal(hUart->LocalRxBuff.hDataAvailableSema);
    }
}

/**
 * Handle the uart transmit interrupt.
 * Thread safety: Caller responsibility.
 */
static void HandleUartTxInterrupt(NvDdkUartHandle hUart)
{
    NvU32 BytesWritten;

    if (hUart->IsWritePolling)
    {
        LockRegisterAccess(hUart);
        NvDdkPrivUartHwSetTransmitInterrupt(&hUart->UartHwRegs, NV_FALSE);
        UnlockRegisterAccess(hUart);

        if (hUart->WriteReq.hOnCompleteSema)
            NvOsSemaphoreSignal(hUart->WriteReq.hOnCompleteSema);
        hUart->WriteReq.hOnCompleteSema = NULL;
        return;    
    }
    // Write into the transmit fifo.
    BytesWritten = NvDdkPrivUartHwWriteInTransmitFifo(&hUart->UartHwRegs,
                    hUart->WriteReq.pCurrTransBuf, hUart->WriteReq.BytesRemaining);
    if (BytesWritten)
    {
        hUart->WriteReq.BytesRemaining -= BytesWritten;
        hUart->WriteReq.BytesTransferred += BytesWritten;
        hUart->WriteReq.pCurrTransBuf += BytesWritten;
    }
    
    // If there is error or the bytes remaining to write is 0 then inform the
    // client.
    if (!hUart->WriteReq.BytesRemaining)
    {
        LockRegisterAccess(hUart);
        NvDdkPrivUartHwSetTransmitInterrupt(&hUart->UartHwRegs, NV_FALSE);
        UnlockRegisterAccess(hUart);
        NvOsSemaphoreSignal(hUart->WriteReq.hOnCompleteSema);
    }
}

/**
 * Handle the uart modem interrupt.
 * Thread safety: Caller responsibility.
 */
static void HandleUartModemInterrupt(NvDdkUartHandle hUart)
{
    NvU32 OldState = (hUart->UartHwRegs.ModemSignalSatus & hUart->ModemSignalName);
    NvU32 NewState;
    
    // Update the modem signal status.
    LockRegisterAccess(hUart);
    NvDdkPrivUartHwUpdateModemSignal(&hUart->UartHwRegs);
    NewState = (hUart->UartHwRegs.ModemSignalSatus & hUart->ModemSignalName);
    UnlockRegisterAccess(hUart);

    if (OldState != NewState)
    {
        // Inform client that there is changes in the modem control signal.
        if (hUart->hModemSigChngCallback)
            (*(hUart->hModemSigChngCallback))
                (hUart->ModemSigChngHandlerArgs);
    }
}

/**
 * Handle all types of the uart interrupts.
 * Thread safety: Caller responsibility.
 */
static void HandleUartInterrupt(void *args)
{
    UartHwInterruptSource InterruptReason;
    NvDdkUartHandle hUart;

    hUart = (NvDdkUartHandle)args;

    // Some action on this channel, so boost frequency
    if (!hUart->IsFreqBoosted)
        NvOsSemaphoreSignal(hUart->hUartPowerMgrSema);

    // The interrupt status is provided from high priority till no pending
    // interrupt so do in the loop to handle all interrupts.
    while (1)
    {
        // Get the interrupt reason for this uart channel.
        InterruptReason = NvDdkPrivUartHwGetInterruptReason(&hUart->UartHwRegs);

        // No pending interrupt or error then break the loop and exit the error.
        if (UartHwInterruptSource_None == InterruptReason)
            break;

        // there is a valid interrupt reason so call appropriate function.
        switch (InterruptReason)
        {
            case UartHwInterruptSource_ReceiveError:
                NvDdkPrivUartHwSetReceiveInterrupt(&hUart->UartHwRegs, NV_FALSE, hUart->IsRxApbDmaBased);
                NvOsSemaphoreSignal(hUart->LocalRxBuff.hLineStatusSema);
                DEBUG_PRINT_RX_ERR(1, ("HandleUartInterrupt(): RX Error interrupt occured\n")); 
                break;
            
            case UartHwInterruptSource_Receive:
            case UartHwInterruptSource_RxTimeout:
            case UartHwInterruptSource_EndOfData:
                HandleUartRxInterrupt(hUart, InterruptReason);
                break;

            case UartHwInterruptSource_Transmit:
                HandleUartTxInterrupt(hUart);
                break;

            case UartHwInterruptSource_ModemControl:
                HandleUartModemInterrupt(hUart);
                break;

            default:
                break;
        }
    }

    NvRmInterruptDone(hUart->InterruptHandle);
    return;
}

static NvError
RegisterUartInterrupt(
    NvRmDeviceHandle hRmDevice,
    NvDdkUartHandle hDdkUart)
{
    NvU32 IrqList;
    NvOsInterruptHandler IntHandlers;
    if (hDdkUart->InterruptHandle)
    {
        return NvSuccess;
    }
    IrqList = NvRmGetIrqForLogicalInterrupt(hRmDevice,
                    NVRM_MODULE_ID(NvRmModuleID_Uart, hDdkUart->InstanceId), 0);
    IntHandlers = HandleUartInterrupt;
    return(NvRmInterruptRegister(hRmDevice, 1, &IrqList, &IntHandlers, hDdkUart,
            &hDdkUart->InterruptHandle, NV_TRUE));
}

/**
 * Uart power manager thread which manages the frequency requirement for the
 * channel. If there is no transaction in the channel for some time then it 
 * reduces the frequency request otherwise it will keep the request for
 * higher frequency.
 */
static void UartPowerManagerThread(void *args)
{
    NvDdkUartHandle hUart = (NvDdkUartHandle)args;
    NvError Error = NvSuccess;
    NvU32 TimeOut = NV_WAIT_INFINITE;
    NvU32 TimeoutErrCount = 0;
    NvBool IsFreqDown = NV_TRUE;
    for (;;)
    {
        // Wait till any event occurs for the transmit thread to take any action.
        Error = NvOsSemaphoreWaitTimeout(hUart->hUartPowerMgrSema, TimeOut);
        if (!Error) // Got the signal from client.
        {
            // If need to stop the thread the return
            if (hUart->IsStopPowerThread)
            {
                // Bringdown the frequency before returning
                if (!IsFreqDown)
                    NvRmPowerBusyHintMulti(hUart->hRmDevice, hUart->RmPowerClientId, 
                                                hUart->DownFreq, MAX_FREQ_REQ_LIST,
                                                NvRmDfsBusyHintSyncMode_Async);
                hUart->IsFreqBoosted = NV_FALSE;
                break;
            }
            
            TimeoutErrCount = 0;

            // If the new state is suspended state then bringdown the freq
            if (hUart->IsSuspendedState)
            {
                if (!IsFreqDown)
                    NvRmPowerBusyHintMulti(hUart->hRmDevice, hUart->RmPowerClientId, 
                                                hUart->DownFreq, MAX_FREQ_REQ_LIST,
                                                NvRmDfsBusyHintSyncMode_Async);
                hUart->IsFreqBoosted = NV_FALSE;
                IsFreqDown = NV_TRUE;
                // Now boost the freq only when resume is called.
                TimeOut = NV_WAIT_INFINITE;
                continue;
            }
            
            // If frequency is already boosted then no need to increase the 
            // frequency
            if (IsFreqDown)
                NvRmPowerBusyHintMulti(hUart->hRmDevice, hUart->RmPowerClientId, 
                                            hUart->UpFreq, MAX_FREQ_REQ_LIST,
                                            NvRmDfsBusyHintSyncMode_Async);
            hUart->IsFreqBoosted = NV_TRUE;
            IsFreqDown = NV_FALSE;
            // Wait for the high freq hold timeout.
            TimeOut = HIGH_FREQUENCY_HOLD_TIMEOUT;
            continue;
        }

        // Dont down the freq in first timeout, do in next timeout.
        if (!TimeoutErrCount)
        {
            // Assume the freq is reduced but actually reduce in next timeout.
            hUart->IsFreqBoosted = NV_FALSE;
            TimeoutErrCount++;
            TimeOut = HIGH_FREQUENCY_HOLD_TIMEOUT;
        }
        else
        {
            // No signal from client and second time, timeout happen and so it 
            // may bringdown the frequency
            if (!IsFreqDown)
                NvRmPowerBusyHintMulti(hUart->hRmDevice, hUart->RmPowerClientId, 
                                            hUart->DownFreq, MAX_FREQ_REQ_LIST,
                                            NvRmDfsBusyHintSyncMode_Async);
            hUart->IsFreqBoosted = NV_FALSE;
            IsFreqDown = NV_TRUE;
            
            // Now wait for the signal for freq boosting request.
            TimeOut = NV_WAIT_INFINITE;
        }
    }
}

static void 
SetBaudRateDependentParameter(
    NvDdkUartHandle hUart, 
    NvU32 BaudRate)
{
    NvU32 CpuFreqReqd;
    CpuFreqReqd  = (BaudRate >= 900000)? 450000: ((BaudRate >= 250000)? 400000: 350000);  

    hUart->UpFreq[0].ClockId = NvRmDfsClockId_Cpu;
    hUart->UpFreq[0].BoostDurationMs = NV_WAIT_INFINITE;
    hUart->UpFreq[0].BoostKHz = CpuFreqReqd;
    hUart->UpFreq[0].BusyAttribute = NV_TRUE;

    hUart->UpFreq[1].ClockId = NvRmDfsClockId_Ahb;
    hUart->UpFreq[1].BoostDurationMs = NV_WAIT_INFINITE;
    hUart->UpFreq[1].BoostKHz = 144000;
    hUart->UpFreq[1].BusyAttribute = NV_TRUE;

    hUart->UpFreq[2].ClockId = NvRmDfsClockId_Emc;
    hUart->UpFreq[2].BoostDurationMs = NV_WAIT_INFINITE;
    hUart->UpFreq[2].BoostKHz = 166000;
    hUart->UpFreq[2].BusyAttribute = NV_TRUE;

    hUart->DownFreq[0].ClockId = NvRmDfsClockId_Cpu;
    hUart->DownFreq[0].BoostDurationMs = NV_WAIT_INFINITE;
    hUart->DownFreq[0].BoostKHz = 0;
    hUart->DownFreq[0].BusyAttribute = NV_TRUE;

    hUart->DownFreq[1].ClockId = NvRmDfsClockId_Ahb;
    hUart->DownFreq[1].BoostDurationMs = NV_WAIT_INFINITE;
    hUart->DownFreq[1].BoostKHz = 0;
    hUart->DownFreq[1].BusyAttribute = NV_TRUE;

    hUart->DownFreq[2].ClockId = NvRmDfsClockId_Emc;
    hUart->DownFreq[2].BoostDurationMs = NV_WAIT_INFINITE;
    hUart->DownFreq[2].BoostKHz = 0;
    hUart->DownFreq[2].BusyAttribute = NV_TRUE;

    // Selected 400 microsecond for polling
    hUart->MaxTxBytesPolling = (BaudRate >= 2000000)? 0x40 : ((BaudRate >= 900000)? 0x30: 0x10); 
    hUart->Char4TimeUs = (40000000 + BaudRate - 1)/(BaudRate); // 4 char*1000000us*10bitperchar
}

/**
 * Destroy the handle of uart channel and free all the allocation done for it.
 * Thread safety: Caller responsibility.
 */
static void DestroyUartChannelHandle(NvDdkUartHandle hUart)
{
    NvRmModuleID ModuleId;

    // If null pointer for uart handle then return error.
    if (!hUart)
        return;

    // Close the uart odm handle
    if (hUart->hOdmUart)
        NvOdmUartClose(hUart->hOdmUart);

    ModuleId = NVRM_MODULE_ID(NvRmModuleID_Uart, hUart->InstanceId);

    if (hUart->RmPowerClientId)
    {
        // Disable the clocks.
        NV_ASSERT_SUCCESS(NvRmPowerModuleClockControl(
                                    hUart->hRmDevice,
                                    ModuleId, hUart->RmPowerClientId,
                                    NV_FALSE));

        // Disable the power to the controller.
        NV_ASSERT_SUCCESS(NvRmPowerVoltageControl(
                                    hUart->hRmDevice,
                                    ModuleId,
                                    hUart->RmPowerClientId,
                                    NvRmVoltsOff,
                                    NvRmVoltsOff,
                                    NULL,
                                    0,
                                    NULL));

        // Unregister for the power manager.
        NvRmPowerUnRegister(hUart->hRmDevice, hUart->RmPowerClientId);
        NvOsSemaphoreDestroy(hUart->hRmPowerEventSema);
    }
    NvDdkPrivUartHwClearAllInt(&hUart->UartHwRegs);

    if (hUart->InterruptHandle)
    {
        NvRmInterruptUnregister(hUart->hRmDevice, hUart->InterruptHandle);
        hUart->InterruptHandle = NULL;
    }

    // Kill the power manager thread.
    if (hUart->hUartPowerMgrThread)
    {
        hUart->IsStopPowerThread = NV_TRUE;
        NvOsSemaphoreSignal(hUart->hUartPowerMgrSema);
        NvOsThreadJoin(hUart->hUartPowerMgrThread);
    }

    NvOsSemaphoreDestroy(hUart->hUartPowerMgrSema);

    // Unmap the virtual mapping of the uart hw register.
    NvRmPhysicalMemUnmap(hUart->UartHwRegs.pRegsVirtBaseAdd,
                         hUart->UartHwRegs.BankSize);

    DestroyLocalRxBuff(hUart);

    if (hUart->IsDmaSupport)
    {
        // Free the dma handles if it is allocated.
        NvRmDmaAbort(hUart->hRxRmDma);
        NvRmDmaFree(hUart->hRxRmDma);

        NvRmDmaAbort(hUart->hTxRmDma);
        NvRmDmaFree(hUart->hTxRmDma);

        // Destroy Dma Buffers
        DestroyDmaTransferBuffer(hUart->hRmMemory, hUart->pRxDmaBuffer,
                    hUart->pTxDmaBuffer, UART_RX_DMA_PING_BUFFER_SIZE << 1);
    }

    // Tri-State the pin-mux pins
    NV_ASSERT_SUCCESS(NvRmSetModuleTristate(hUart->hRmDevice, ModuleId, NV_TRUE));

    // Destroy the mutex allocated for the uart read/write/register/channel access.
    DestroyReadFlowLock(hUart);
    DestroyWriteFlowLock(hUart);
    DestroyRegisterAccessLock(hUart);


    // Destroy the sync sempahores.
    NvOsSemaphoreDestroy(hUart->hRxSynchSema);
    NvOsSemaphoreDestroy(hUart->hTxSynchSema);

    // Free the memory of the uart handles.
    NvOsFree(hUart);
}

/**
 * Create the handle for the uart channel.
 * Thread safety: Caller responsibility.
 */
static NvError CreateUartChannelHandle(
    NvRmDeviceHandle hRmDevice,
    NvU32 ChannelId,
    NvDdkUartHandle *phUart)
{
    NvError Error = NvSuccess;
    NvDdkUartHandle hNewUart = NULL;
    NvRmModuleID ModuleId;
    NvRmModuleUartInterfaceCaps InterfaceCap;
    
    ModuleId = NVRM_MODULE_ID(NvRmModuleID_Uart, ChannelId);

    *phUart = NULL;

    if (NvRmSetModuleTristate(hRmDevice,ModuleId,NV_FALSE)!=NvSuccess)
        return NvError_NotSupported;

    Error = NvRmGetModuleInterfaceCapabilities(hRmDevice, ModuleId,
                        sizeof(InterfaceCap), &InterfaceCap);
    if (Error)
        return Error;

    // If the number of lines available in platform is 0 then return error
    // as the uart is not supported on the given platform.
    if (InterfaceCap.NumberOfInterfaceLines == 0)
        return NvError_NotSupported;
        
    // Allocoate the memory for the uart handle.
    hNewUart = NvOsAlloc(sizeof(NvDdkUart));
    if (!hNewUart)
    {
        return NvError_InsufficientMemory;
    }

    // Reset the memory allocated for the uart handle.
    NvOsMemset(hNewUart, 0, sizeof(*hNewUart));

    // Set the uart handle parameters.
    hNewUart->hRmDevice = hRmDevice;
    hNewUart->InstanceId = ChannelId;
    hNewUart->OpenCount = 0;
    hNewUart->NumberOfInterfaceLine = InterfaceCap.NumberOfInterfaceLines;

    hNewUart->WriteReq.hOnCompleteSema = NULL;
    hNewUart->WriteReq.pCurrTransBuf = NULL;

    hNewUart->ReadReq.hOnCompleteSema = NULL;
    hNewUart->ReadReq.pCurrTransBuf = NULL;

    hNewUart->hRxSynchSema = NULL;
    hNewUart->hTxSynchSema = NULL;

    hNewUart->IsRtsFlowEnable = NV_FALSE;
    hNewUart->IsCtsFlowEnable = NV_FALSE;

    hNewUart->hRmMemory = NULL;
    hNewUart->RxDmaBuffPhysAdd = 0;
    hNewUart->pRxDmaBuffer = NULL;
    hNewUart->hRxRmDma = NULL;

    hNewUart->TxDmaBuffPhysAdd = 0;
    hNewUart->pTxDmaBuffer = NULL;
    hNewUart->hTxRmDma = NULL;

    hNewUart->hWriteMutex = NULL;
    hNewUart->hReadMutex = NULL;
    hNewUart->hRegisterAccessMutex = NULL;

    hNewUart->hModemSigChngCallback = NULL;
    hNewUart->pUartInfo = s_pUartInfo;
    hNewUart->hRmPowerEventSema = NULL;
    hNewUart->RmPowerClientId = 0;
    hNewUart->hOdmUart = NULL;
    
    hNewUart->IsRxApbDmaBased = NV_FALSE;
    hNewUart->IsDmaReadStarted = NV_FALSE;

    hNewUart->IsStopPowerThread = NV_FALSE;
    hNewUart->IsFreqBoosted = NV_FALSE;
    hNewUart->IsSuspendedState = NV_FALSE;

    hNewUart->hUartPowerMgrThread = NULL;
    hNewUart->hUartPowerMgrSema = NULL;

    // Initialize the uart configuration parameters.
    hNewUart->UartConfig.UartBaudRate = 0;
    hNewUart->UartConfig.UartDataLength = 5;
    hNewUart->UartConfig.UartParityBit = NvDdkUartParity_None;
    hNewUart->UartConfig.UartStopBit = NvDdkUartStopBit_1;
    hNewUart->UartConfig.IsEnableIrdaModulation = NV_FALSE;
    
    NvDdkPrivUartHwRegisterInitialize(ChannelId, &hNewUart->UartHwRegs);
    hNewUart->OldRtsState = hNewUart->UartHwRegs.IsRtsActive;
    hNewUart->OldDtrState = hNewUart->UartHwRegs.IsDtrActive;

    // Initialize the baudrate dependent parameter.
    SetBaudRateDependentParameter(hNewUart, 9600);
    
    // Get the soc capabilities and if error return here now.
    Error = UartGetSocCapabilities(hRmDevice, ChannelId, &hNewUart->SocUartCaps);
    if (!Error)
    {
        hNewUart->UartHwRegs.IsEndOfDataIntSupport =
                                hNewUart->SocUartCaps.IsEndOfDataIntSupported;
        hNewUart->UartHwRegs.IsRtsHwFlowSupported =
                                hNewUart->SocUartCaps.IsRtsHwFlowControlSupported;
    }

    // If error the return now.
    if (Error)
    {
        NvOsFree(hNewUart);
        return Error;
    }

   // Create the odm uart handle
   hNewUart->hOdmUart = NvOdmUartOpen(ChannelId);

    // Create the local buffer information.
    Error = CreateLocalRxBuff(hNewUart, UART_RX_LOCAL_BUFFER_SIZE_MIN);

    // Create all mutex required for the channel.
    if (!Error)
        Error = CreateReadFlowLock(hNewUart);
    if (!Error)
        Error = CreateWriteFlowLock(hNewUart);
    if (!Error)
        Error = CreateRegisterAccessLock(hNewUart);

    // Create the synchronous semaphores.
    if (!Error)
        Error = NvOsSemaphoreCreate(&hNewUart->hRxSynchSema, 0);

    if (!Error)
        Error = NvOsSemaphoreCreate(&hNewUart->hTxSynchSema, 0);

    // Create the read/write transfer information.
    if (!Error)
    {
        InitTransferInfo(hNewUart->hRmDevice, &hNewUart->WriteReq);
        InitTransferInfo(hNewUart->hRmDevice, &hNewUart->ReadReq);

        NvRmModuleGetBaseAddress(hRmDevice, ModuleId,
            &hNewUart->UartHwRegs.RegsPhyBaseAdd, &hNewUart->UartHwRegs.BankSize);

        Error = NvRmPhysicalMemMap(hNewUart->UartHwRegs.RegsPhyBaseAdd,
                    hNewUart->UartHwRegs.BankSize, NVOS_MEM_READ_WRITE,
                    NvOsMemAttribute_Uncached,
                    (void **)&hNewUart->UartHwRegs.pRegsVirtBaseAdd);
    }

    // Create the dma transfer buffer, allocate the dma and initialize the
    // request structure.
    if (!Error)
    {
        Error = CreateDmaTransferBuffer(hNewUart->hRmDevice, &hNewUart->hRmMemory, 
                    &hNewUart->RxDmaBuffPhysAdd, (void **)&hNewUart->pRxDmaBuffer, 
                    &hNewUart->TxDmaBuffPhysAdd, (void **)&hNewUart->pTxDmaBuffer, 
                    UART_RX_DMA_PING_BUFFER_SIZE << 1);

        // Allocate the two dma (one for Rx and One for Tx) with high priority
        if (!Error)
            Error = NvRmDmaAllocate(hNewUart->hRmDevice, &hNewUart->hRxRmDma,
                             NV_FALSE, NvRmDmaPriority_High, NvRmDmaModuleID_Uart,
                             hNewUart->InstanceId);
        if (!Error)
        {
            Error = NvRmDmaAllocate(hNewUart->hRmDevice, &hNewUart->hTxRmDma,
                         NV_FALSE, NvRmDmaPriority_High, NvRmDmaModuleID_Uart,
                         hNewUart->InstanceId);
        }
        if (Error)
        {
            hNewUart->IsDmaSupport = NV_FALSE;
            DestroyDmaTransferBuffer(hNewUart->hRmMemory, hNewUart->pRxDmaBuffer,
                            hNewUart->pTxDmaBuffer, UART_RX_DMA_PING_BUFFER_SIZE << 1);
                            
            NvRmDmaFree(hNewUart->hRxRmDma);
            NvRmDmaFree(hNewUart->hTxRmDma);
            hNewUart->hRmMemory = NULL;
            hNewUart->pRxDmaBuffer = NULL;
            hNewUart->pTxDmaBuffer = NULL;
            hNewUart->hRxRmDma = NULL;
            hNewUart->hTxRmDma = NULL;
            Error = NvSuccess;

        }
        else
        {
            hNewUart->IsDmaSupport = NV_TRUE;
            hNewUart->WDmaReq.SourceBufferPhyAddress = hNewUart->TxDmaBuffPhysAdd; 
            hNewUart->WDmaReq.DestinationBufferPhyAddress = hNewUart->UartHwRegs.RegsPhyBaseAdd; 
            hNewUart->WDmaReq.SourceAddressWrapSize = 0; 
            hNewUart->WDmaReq.DestinationAddressWrapSize = 4; 
            hNewUart->TxDmaBufferSize = UART_RX_DMA_PING_BUFFER_SIZE << 1;

            hNewUart->RDmaReq.SourceBufferPhyAddress = hNewUart->UartHwRegs.RegsPhyBaseAdd; 
            hNewUart->RDmaReq.DestinationBufferPhyAddress = hNewUart->RxDmaBuffPhysAdd; 
            hNewUart->RDmaReq.SourceAddressWrapSize = 4; 
            hNewUart->RDmaReq.DestinationAddressWrapSize = 0; 
            hNewUart->RxDmaBufferSize = UART_RX_DMA_PING_BUFFER_SIZE << 1;
            hNewUart->HalfRxDmaBufferSize = UART_RX_DMA_PING_BUFFER_SIZE;
     
        }
    }

    // Register as the Rm power client
    if (!Error)
    {
        Error = NvOsSemaphoreCreate(&hNewUart->hRmPowerEventSema, 0);
        if (!Error)
            Error = NvRmPowerRegister(hNewUart->hRmDevice, hNewUart->hRmPowerEventSema,
                                &hNewUart->RmPowerClientId);
    }

    // Enable power for uart module
    if (!Error)
        Error = NvRmPowerVoltageControl(hNewUart->hRmDevice, ModuleId,
                        hNewUart->RmPowerClientId,
                        NvRmVoltsUnspecified, NvRmVoltsUnspecified,
                        NULL, 0, NULL);

    // Enable the clock.
    if (!Error)
        Error = NvRmPowerModuleClockControl(hRmDevice, ModuleId, hNewUart->RmPowerClientId,
                        NV_TRUE);

    // Create the thread which keep the power/frequency requirement of the
    // uart channel.
    if (!Error)
    {
        Error = NvOsSemaphoreCreate(&hNewUart->hUartPowerMgrSema, 0);
        if (!Error)
            Error = NvOsThreadCreate(UartPowerManagerThread, hNewUart, &hNewUart->hUartPowerMgrThread);
    }

    // Reset the uart controller.
    if (!Error)
        NvRmModuleReset(hRmDevice, ModuleId);

    if (!Error)
        Error = RegisterUartInterrupt(hRmDevice, hNewUart);

    // Configure the uart handle for the non dma mode transfer as basic transfer.
    if (!Error)
    {
        // Enable the fifo mode of the uart and reset the fifo
        NvDdkPrivUartHwInitFifo(&hNewUart->UartHwRegs);
        NvDdkPrivUartHwSetDmaMode(&hNewUart->UartHwRegs, hNewUart->IsDmaSupport);
    }
    // If error then destroy all the allocation done here.
    if (Error)
    {
        DestroyUartChannelHandle(hNewUart);
        hNewUart = NULL;
    }
    *phUart = hNewUart;
    return Error;
}

/**
 * Read from the local buffer.
 * Thread safety: Caller responsibility.
 */
static NvU32 ReadFromLocalBuffer(
    NvU8 *pReceiveBuffer,
    NvU32 BytesRequested,
    UartLocalBuffer *pRxLocalBuffer)
{
    NvU32 DataToBeRead;
    NvU32 DataRead = 0;

    // Get the current write index and then read based on this index.
    DataToBeRead = NV_MIN(BytesRequested, pRxLocalBuffer->DataAvailable);
    while (DataToBeRead)
    {
        *pReceiveBuffer++  = pRxLocalBuffer->pBuffPtr[pRxLocalBuffer->ReadIndex++];
        if (pRxLocalBuffer->ReadIndex >= pRxLocalBuffer->TotalBufferSize)
            pRxLocalBuffer->ReadIndex = 0;

        DataToBeRead--;
        DataRead++;
    }
    pRxLocalBuffer->DataAvailable = pRxLocalBuffer->DataAvailable - DataRead;
    return DataRead;
}


/**
 * Stop the write operation.
 * Thread protection: By caller
 */
static void UartStopWrite(NvDdkUartHandle hUart, NvBool IsResetTxFifoIfNotBusy)
{
    // If write transfer state is not busy then invalid state error.
    if (hUart->WriteReq.IsBusy)
    {
        hUart->WriteReq.IsStopReq = NV_TRUE;
        NvOsSemaphoreSignal(hUart->WriteReq.hOnCompleteSema);
        if (!hUart->IsDmaSupport)
        {
            LockRegisterAccess(hUart);
            NvDdkPrivUartHwSetTransmitInterrupt(&hUart->UartHwRegs, NV_FALSE);
            UnlockRegisterAccess(hUart);
        }    
    }
    else
    {
        if (IsResetTxFifoIfNotBusy)
            NvDdkPrivUartHwResetFifo(&hUart->UartHwRegs, 
                                                    UartDataDirection_Transmit);
    }
}


/**
 * Wait till tx fifo become empty. This is required when either we change the 
 * baudrate or we go for suspend.
 * Thread protection: By caller
 */
static void WaitTillTxFifoEmpty(NvDdkUartHandle hUart)
{
    NvDdkUartConfiguarations *pUartCurrConfig = NULL;

    NvBool IsTxFifoEmpty;
    NvU32 CharacterWaitTimeInUs;
    NvU32 WaitTimeInMs;
    NvU32 RemainingWaitTimeoutUs;
    
    // Get the uart configuration pointers.
    pUartCurrConfig = &hUart->UartConfig;
    
    // Wait for the Tx fifo to be empty if the current baudrate is non zero.
    // Wait time is maximum of (fifo depth + 4) character time, and if it 
    // does not get flushed then reset the fifo.
    // Here + 4 is for the safer side.
    if (pUartCurrConfig->UartBaudRate)
    {
        // 1 char time in microsecond: 1 char*1000000us*10 bitperchar
        CharacterWaitTimeInUs = (10000000 + pUartCurrConfig->UartBaudRate - 1)/
                                (pUartCurrConfig->UartBaudRate); 
        RemainingWaitTimeoutUs = (hUart->SocUartCaps.FifoDepth + 4)*CharacterWaitTimeInUs;
        WaitTimeInMs = (CharacterWaitTimeInUs + 999)/1000;
        while(1)
        {
            IsTxFifoEmpty = NvDdkPrivUartHwIsTransmitFifoEmpty(&hUart->UartHwRegs);
            if (IsTxFifoEmpty)
                break;
            NvOsSleepMS(WaitTimeInMs);
            if (RemainingWaitTimeoutUs > (WaitTimeInMs * 1000))
            {
                RemainingWaitTimeoutUs -= (WaitTimeInMs * 1000);
                continue;
            }    
            break;
        }
    
        // If the tx fifo is not empty then reset the fifo forcefully.
        if (!IsTxFifoEmpty)
        {
            LockRegisterAccess(hUart);
            NvDdkPrivUartHwResetFifo(&hUart->UartHwRegs, UartDataDirection_Transmit);
            UnlockRegisterAccess(hUart);
        }    
    }

}
/**
 * Open the uart handle.
 */
NvError
NvDdkUartOpen(
    NvRmDeviceHandle hRmDevice,
    NvU32 ChannelId,
    NvDdkUartHandle *phUart)
{
    NvError Error = NvSuccess;
    DdkUartChannelInfo *pUartInfo = NULL;
    NvDdkUart *pUartChannel = NULL;
    NvU32 InstanceAvailable;

    NV_ASSERT(phUart);
    NV_ASSERT(hRmDevice);

    *phUart = NULL;

    InstanceAvailable = NvRmModuleGetNumInstances(hRmDevice, NvRmModuleID_Uart);
    if (ChannelId >= InstanceAvailable)
        return NvError_NotSupported;

    Error = InitUartInformation(hRmDevice);
    if (Error)
        return Error;
    pUartInfo = s_pUartInfo;

    // Lock the uart info mutex access.
    NvOsMutexLock(pUartInfo->hUartOpenMutex);

    // Check for the open uart handle to find out whether same instance port
    // name exit or not.
    pUartChannel = pUartInfo->hUartList[ChannelId];

    // If the uart handle does not exist then create it.
    if (!pUartChannel)
    {
        Error = CreateUartChannelHandle(hRmDevice, ChannelId, &pUartChannel);
        if (!Error)
        {
            pUartInfo->hUartList[ChannelId] = pUartChannel;
            pUartChannel->OpenCount++;
        }
    }

    // Unlock the uart info access.
    NvOsMutexUnlock(pUartInfo->hUartOpenMutex);

    // If no error then update the passed pointers.
    if (!Error)
    {
        *phUart = pUartChannel;
    }
    return Error;
}

/**
 * Close the uart handle.
 */
void NvDdkUartClose(NvDdkUartHandle hUart)
{
    DdkUartChannelInfo *pUartInformation = NULL;

    // if null parameter then do nothing.
    if (!hUart)
        return;

    // Get the uart information pointer.
    pUartInformation = hUart->pUartInfo;

    // Lock the uart information access.
    NvOsMutexLock(pUartInformation->hUartOpenMutex);

    // decrement the open count and it becomes 0 then release all the allocation
    // done for this handle.
    hUart->OpenCount--;

    // If the open count become zero then remove from the list of handles and
    // free..
    if (hUart->OpenCount == 0)
    {
        // Read/write should not be on this channel.
        NV_ASSERT((!hUart->ReadReq.IsBusy) || (!hUart->WriteReq.IsBusy)); 
        
        pUartInformation->hUartList[hUart->InstanceId] = NULL;
        // Now destroy the handles.
        DestroyUartChannelHandle(hUart);
    }

    // Unlock the uart information access.
    NvOsMutexUnlock(pUartInformation->hUartOpenMutex);
}

/**
 * Set the uart configuration which is configured currently.
 */
NvError
NvDdkUartSetConfiguration(
    NvDdkUartHandle hUart,
    const NvDdkUartConfiguarations* const pUartNewConfig)
{
    NvError Error = NvSuccess;
    NvDdkUartConfiguarations *pUartCurrConfig = NULL;
    NvU32 ConfiguredClockFreq = 0;
    NvRmModuleID ModuleId;
    NvU32 MinClockFreqReqd;
    NvU32 MaxClockFreqReqd;
    NvU32 ClockFreqReqd = 0;

    NV_ASSERT(hUart);
    NV_ASSERT(pUartNewConfig);
    if (hUart->IsSuspendedState)
        return NvError_InvalidState;

    // Stop bit 1.5 is supported only with the data length of 5.
    if ((pUartNewConfig->UartStopBit == NvDdkUartStopBit_1_5) &&
                            ((pUartNewConfig->UartDataLength != 5)))
        return NvError_NotSupported;

    // Stop bit 2 is not supported with the data length of 5.
    if ((pUartNewConfig->UartStopBit == NvDdkUartStopBit_2) &&
                            ((pUartNewConfig->UartDataLength == 5)))
        return NvError_NotSupported;

    // Get the uart configuration pointers.
    pUartCurrConfig = &hUart->UartConfig;

    // Wait for the Tx fifo to be empty.
    WaitTillTxFifoEmpty(hUart);

    LockRegisterAccess(hUart);
    // Set the required baud rate if there is any change.
    if (pUartNewConfig->UartBaudRate != pUartCurrConfig->UartBaudRate)
    {
        // Get the clock frequency related to the baud rate.
        NvDdkPrivUartHwGetReqdClockSourceFreq(pUartNewConfig->UartBaudRate,
            &MinClockFreqReqd, &ClockFreqReqd, &MaxClockFreqReqd);

        ModuleId = NVRM_MODULE_ID(NvRmModuleID_Uart, hUart->InstanceId);
        Error = NvRmPowerModuleClockConfig(hUart->hRmDevice,
                    ModuleId, 0, MinClockFreqReqd, MaxClockFreqReqd,
                    &ClockFreqReqd, 1, &ConfiguredClockFreq, 0);

        // If not error then check for configured clock frequency and verify.
        if (!Error)
        {
            // Set the baudrate and get the required clock frequency.
            Error = NvDdkPrivUartHwSetBaudRate(&hUart->UartHwRegs,
                            pUartNewConfig->UartBaudRate, ConfiguredClockFreq);
            if (!Error)
                pUartCurrConfig->UartBaudRate = pUartNewConfig->UartBaudRate;
        }

        // If error then return here only.
        if (Error)
            goto ErrorExit;

        SetBaudRateDependentParameter(hUart, pUartCurrConfig->UartBaudRate);
    }

    // Configured for parity bit.
    if (pUartNewConfig->UartParityBit != pUartCurrConfig->UartParityBit)
    {
        Error = NvDdkPrivUartHwSetParityBit(&hUart->UartHwRegs,
                                    pUartNewConfig->UartParityBit);
        if (Error)
            goto ErrorExit;
        pUartCurrConfig->UartParityBit = pUartNewConfig->UartParityBit;
    }

    // Configured for the data length.
    if (pUartNewConfig->UartDataLength != pUartCurrConfig->UartDataLength)
    {
        Error = NvDdkPrivUartHwSetDataLength(&hUart->UartHwRegs,
                    pUartNewConfig->UartDataLength);
        if (Error)
            goto ErrorExit;
        pUartCurrConfig->UartDataLength = pUartNewConfig->UartDataLength;
    }

    // Configured for stop bit length.
    if (pUartNewConfig->UartStopBit != pUartCurrConfig->UartStopBit)
    {
        Error = NvDdkPrivUartHwSetStopBit(&hUart->UartHwRegs,
                                pUartNewConfig->UartStopBit);
        if (Error)
            goto ErrorExit;
        pUartCurrConfig->UartStopBit = pUartNewConfig->UartStopBit;
    }

    // Configured for the Irda modulation.
    if (pUartNewConfig->IsEnableIrdaModulation != pUartCurrConfig->IsEnableIrdaModulation)
    {
        NvDdkPrivUartHwSetIrdaCoding(&hUart->UartHwRegs, pUartNewConfig->IsEnableIrdaModulation);
        pUartCurrConfig->IsEnableIrdaModulation = pUartNewConfig->IsEnableIrdaModulation;
    }
ErrorExit:
    UnlockRegisterAccess(hUart);

    // Boost frequency now.
    if ((!Error) && (!hUart->IsFreqBoosted))
        NvOsSemaphoreSignal(hUart->hUartPowerMgrSema);

    return Error;
}

/**
 * Get the uart configuartion which is configured currently.
 */
NvError
NvDdkUartGetConfiguration(
    NvDdkUartHandle hUart,
    NvDdkUartConfiguarations* const pUartDriverConfiguration)
{
    NV_ASSERT(hUart);
    NV_ASSERT(pUartDriverConfiguration);
    if (hUart->IsSuspendedState)
        return NvError_InvalidState;

    LockRegisterAccess(hUart);
    // Copy the current configured parameter for the uart communication.
    NvOsMemcpy(pUartDriverConfiguration, &hUart->UartConfig,
                                            sizeof(*pUartDriverConfiguration));
    UnlockRegisterAccess(hUart);
    return NvSuccess;
}

NvError
NvDdkUartStartReadOnBuffer(
    NvDdkUartHandle hUart,
    NvOsSemaphoreHandle hRxEventSema,
    NvU32 BufferSize)
{
    NvError Error = NvSuccess;
    NvU32 NewBufferSize;
    NvU8 *pNewBuff = NULL;
    
    NV_ASSERT(hUart);
    NV_ASSERT(BufferSize);
    if (hUart->IsSuspendedState)
        return NvError_InvalidState;

    // If reading is going on then it is not possible to change the size of the
    // local buffering
    if (hUart->ReadReq.IsBusy)
        return NvError_InvalidState;

    // Create the local buffer first.
    LockReadFlow(hUart);
    if (hUart->ReadReq.IsBusy)
    {
        UnlockReadFlow(hUart);
        return NvError_InvalidState;
    }
    
    // Deactivate the RTS line to stop the sender to data send as ddk is not 
    // able to read the data.
    LockRegisterAccess(hUart);
    if (hUart->IsRtsFlowEnable)
        NvDdkPrivUartHwSetModemSignal(&hUart->UartHwRegs, NvDdkUartSignalName_Rts, NV_FALSE);

    // Disable receive interrupt.
    NvDdkPrivUartHwSetReceiveInterrupt(&hUart->UartHwRegs, NV_TRUE, hUart->IsRxApbDmaBased);
    UnlockRegisterAccess(hUart);

    // Abort the dma if it is started.
    if ((hUart->IsDmaReadStarted) && (hUart->IsRxApbDmaBased))
        NvRmDmaAbort(hUart->hRxRmDma);
        
    hUart->IsRxApbDmaBased = NV_FALSE;
    hUart->IsDmaReadStarted = NV_FALSE;

    // Reset the Rx fifo.
    NvDdkPrivUartHwResetFifo(&hUart->UartHwRegs, UartDataDirection_Receive);

    // Check whether it is same size or not. If not then reallocate it.
    NewBufferSize = NV_MAX(UART_RX_LOCAL_BUFFER_SIZE_MIN, BufferSize);
    if (NewBufferSize != hUart->LocalRxBuff.TotalBufferSize)
    {
        pNewBuff = NvOsAlloc(NewBufferSize);

        // If allocated successfully then use this memory for local buffering.
        if(pNewBuff)
        {
            // Free the existing buffer.
            if (hUart->LocalRxBuff.pBuffPtr)
                NvOsFree(hUart->LocalRxBuff.pBuffPtr);
            hUart->LocalRxBuff.pBuffPtr = pNewBuff;
        }
        else
        {
            NewBufferSize = hUart->LocalRxBuff.TotalBufferSize;
        }
    }
    hUart->LocalRxBuff.ReadIndex = 0;
    hUart->LocalRxBuff.WriteIndex = 0;
    hUart->LocalRxBuff.DataAvailable = 0;
    hUart->LocalRxBuff.TotalBufferSize = NewBufferSize;
    hUart->LocalRxBuff.BufferLevelHigh = (NewBufferSize >> 1);
    hUart->LocalRxBuff.hDataAvailableSema = hRxEventSema;
    hUart->LocalRxBuff.hLineStatusSema = hRxEventSema;

    // Start the dma to start receiving the data.
    if (hUart->IsDmaSupport)
    {
        hUart->RDmaReq.TransferSize = hUart->RxDmaBufferSize;
        Error = NvRmDmaStartDmaTransfer(hUart->hRxRmDma, &hUart->RDmaReq,
                NvRmDmaDirection_Forward, 0, hUart->LocalRxBuff.hDataAvailableSema);
        if (!Error)
        {
            hUart->IsRxApbDmaBased = NV_TRUE;
            hUart->IsDmaReadStarted = NV_TRUE;
            hUart->LastReadIndex = 0;
        }
        Error = NvSuccess;
    }

    LockRegisterAccess(hUart);
    // Enable the receive interrupt.
    NvDdkPrivUartHwSetReceiveInterrupt(&hUart->UartHwRegs, NV_TRUE, hUart->IsRxApbDmaBased);

    // Activate the RTS line as ddk is able to receive more data as receive  
    // buffer is empty.     
    if (hUart->IsRtsFlowEnable)
        NvDdkPrivUartHwSetModemSignal(&hUart->UartHwRegs, NvDdkUartSignalName_Rts, 
                                                            NV_TRUE);

    UnlockRegisterAccess(hUart);
    UnlockReadFlow(hUart);
    return Error;
}
    
NvError NvDdkUartClearReceiveBuffer(NvDdkUartHandle hUart)
{
    NV_ASSERT(hUart);
    if (hUart->IsSuspendedState)
        return NvError_InvalidState;
    
    return NvDdkUartStartReadOnBuffer(hUart, hUart->LocalRxBuff.hDataAvailableSema,
                            hUart->LocalRxBuff.TotalBufferSize);
}

NvError 
NvDdkUartUpdateReceiveBuffer(
    NvDdkUartHandle hUart, 
    NvU32 *pAvailableBytes)
{
    NvU32 SpaceAvailable;
    NvError Error = NvSuccess;
    NvU32 TransferdCount;
    NvU32 DataToBeCopied;
    NvU8  CpuReadBuffer[30];
    NvU32 TempTransCount;
    NvU32 Index;
    NvBool IsDmaIndexTobeReset = NV_FALSE;
    NvU32 BytesRead;
    NvBool IsBreakDetectd;
    
    NV_ASSERT(hUart);
    NV_ASSERT(pAvailableBytes);
    if (hUart->IsSuspendedState)
        return NvError_InvalidState;

    // Lock the transfer status mutex.
    LockReadFlow(hUart);
    
    BytesRead = 0;
    TransferdCount = 0;
    IsBreakDetectd = NvDdkPrivUartHwIsBreakSignalDetected(&hUart->UartHwRegs);
    if (hUart->IsRxApbDmaBased)
    {
        if (NvDdkPrivUartHwIsDataAvailableInFifo(&hUart->UartHwRegs))
        {
            // Disable the rts line as cpu is going to read the fifo.
            if (hUart->IsRtsFlowEnable)
                NvDdkPrivUartHwSetModemSignal(&hUart->UartHwRegs, 
                                            NvDdkUartSignalName_Rts, NV_FALSE);

            // Get the transfer Count transferred by the dma. Also stop the apb 
            // dma so that apb dma will not be able to transfer the data.
            Error = NvRmDmaGetTransferredCount(hUart->hRxRmDma, &TransferdCount, NV_TRUE);
            NV_ASSERT(Error == NvSuccess);
            if (Error)
                TransferdCount = 0;

            // Now Read the fifo by cpu.
            hUart->ReadReq.TransferStatus =
                    NvDdkPrivUartHwReadFromReceiveFifo(&hUart->UartHwRegs,
                                                CpuReadBuffer, 25, &BytesRead);
            DEBUG_PRINT_RX_ERR(hUart->ReadReq.TransferStatus,
                ("NvDdkUartUpdateReceiveBuffer(): LSR Status shows error Error occured 0x%08x\n", hUart->ReadReq.TransferStatus)); 

            // Now start the dma so that receive data can be transferred again.
            Error = NvRmDmaGetTransferredCount(hUart->hRxRmDma, &TempTransCount, NV_FALSE);
            NV_ASSERT(Error == NvSuccess);
            if (Error)
                TransferdCount = 0;
            IsDmaIndexTobeReset = NV_TRUE;    
        }
        else
        {
            // Get the transfer Count transferred by the dma without stopping the dma.
            Error = NvRmDmaGetTransferredCount(hUart->hRxRmDma, &TransferdCount, NV_FALSE);
            NV_ASSERT(Error == NvSuccess);
            if (Error)
                TransferdCount = 0;
            hUart->ReadReq.TransferStatus = NvSuccess;
                
        }
        if (TransferdCount)
        {
            //NvOsDebugPrintf("UartUpdate old TransCount 0x%08x and old Last ReadIndex 0x%08x\n ",
                        //TransferdCount, hUart->LastReadIndex);
            if (hUart->LastReadIndex < hUart->HalfRxDmaBufferSize)
                TransferdCount = TransferdCount - hUart->LastReadIndex;    
            else
                TransferdCount = TransferdCount - (hUart->LastReadIndex - hUart->HalfRxDmaBufferSize);
        }

        // Copy in the circular buffer
        if (TransferdCount)
        {
            SpaceAvailable = hUart->LocalRxBuff.TotalBufferSize - hUart->LocalRxBuff.DataAvailable;
            DataToBeCopied = NV_MIN(SpaceAvailable, TransferdCount);

            // For debugging the DataToBeCopied should never be less than
            // the TransferdCount
            NV_ASSERT(DataToBeCopied == TransferdCount);
            for (Index = 0; Index < DataToBeCopied; ++Index)
            {
                hUart->LocalRxBuff.pBuffPtr[hUart->LocalRxBuff.WriteIndex++] =
                    hUart->pRxDmaBuffer[hUart->LastReadIndex++];
                if (hUart->LocalRxBuff.WriteIndex >= hUart->LocalRxBuff.TotalBufferSize)
                    hUart->LocalRxBuff.WriteIndex = 0;
                if (hUart->LastReadIndex >= hUart->RxDmaBufferSize)
                    hUart->LastReadIndex = 0;
                hUart->LocalRxBuff.DataAvailable++;
            }
//            NvOsDebugPrintf("uartUpdate New TransCount 0x%08x and new LastReadIndex 0x%08x\n",
//                                TransferdCount, hUart->LastReadIndex);
        }
        if (BytesRead)
        {
            // Get the lcoal buffer pointer.
            SpaceAvailable = hUart->LocalRxBuff.TotalBufferSize - hUart->LocalRxBuff.DataAvailable;
            DataToBeCopied = NV_MIN(BytesRead, SpaceAvailable);

            // FixME!! For debugging the DataToBeCopied should never be less than
            // the BytesRead
            NV_ASSERT(DataToBeCopied == BytesRead);

            for (Index = 0; Index < DataToBeCopied; ++Index)
            {
                hUart->LocalRxBuff.pBuffPtr[hUart->LocalRxBuff.WriteIndex++] =
                                        CpuReadBuffer[Index];
                if (hUart->LocalRxBuff.WriteIndex >= hUart->LocalRxBuff.TotalBufferSize)
                    hUart->LocalRxBuff.WriteIndex = 0;
                hUart->LocalRxBuff.DataAvailable++;
            }
        }

        if (IsDmaIndexTobeReset)
            hUart->LastReadIndex = 0;

        NvDdkPrivUartHwSetReceiveInterrupt(&hUart->UartHwRegs, NV_TRUE, NV_TRUE);
    }  

    // See whether the readcall is waiting for the data or not. If yes then
    // copy on client buffer.
    if (hUart->ReadReq.IsBusy)
    {
        // If transfer status is success then read form local buffer
        // and upate the client information and inform if it
        // request has been completed.
        if ((hUart->ReadReq.TransferStatus == NvSuccess) && (!IsBreakDetectd))
        {
            BytesRead = ReadFromLocalBuffer(hUart->ReadReq.pCurrTransBuf,
                            hUart->ReadReq.BytesRemaining, &hUart->LocalRxBuff);

            // If bytes read then update the read parameters.
            if (BytesRead)
            {
                hUart->ReadReq.pCurrTransBuf += BytesRead;
                hUart->ReadReq.BytesRemaining -= BytesRead;
                hUart->ReadReq.BytesTransferred += BytesRead;
            }
        }

        // If the transfer status is error or bytes remaining for
        // client is  0 then inform the client.
        if ((hUart->ReadReq.TransferStatus) || (!hUart->ReadReq.BytesRemaining) 
                            || IsBreakDetectd)
        {
            // Break signal status is higher priority than other status.
            if (IsBreakDetectd)
                hUart->ReadReq.TransferStatus = NvError_UartBreakReceived;

            // Update the current state.
            hUart->ReadReq.IsBusy = NV_FALSE;

            // Inform the client that transfer is complete.
            NvOsSemaphoreSignal(hUart->ReadReq.hOnCompleteSema);
        }
    }
    else
    {
        // Break signal status is higher priority than other status.
        if (IsBreakDetectd)
            hUart->ReadReq.TransferStatus = NvError_UartBreakReceived;
    }

    // If flow control is enabled and if the data available is more then the
    // higher water level of the local buffer then disable the rts line.
    if (hUart->IsRtsFlowEnable)
    {
        if (hUart->LocalRxBuff.DataAvailable < hUart->LocalRxBuff.BufferLevelHigh)
            NvDdkPrivUartHwSetModemSignal(&hUart->UartHwRegs, 
                                            NvDdkUartSignalName_Rts, NV_TRUE);
        else
            NvDdkPrivUartHwSetModemSignal(&hUart->UartHwRegs, 
                                            NvDdkUartSignalName_Rts, NV_FALSE);
    }
    UnlockReadFlow(hUart);

    *pAvailableBytes = hUart->LocalRxBuff.DataAvailable;
    return hUart->ReadReq.TransferStatus;    
}

/**
 * Read the data from com channel.
 * Thread safety: Provided in the function.
 */
NvError
NvDdkUartRead(
    NvDdkUartHandle hUart,
    NvU8 *pReceiveBuffer,
    NvU32 BytesRequested,
    NvU32 *pBytesRead,
    NvU32 WaitTimeoutMs)
{
    NvError Error = NvSuccess;
    NvU32 BytesRead;
#if ENABLE_READ_PROFILE
    NvU32 EntryTime = NvOsGetTimeMS();
#endif

    NV_ASSERT(hUart);
    NV_ASSERT(pReceiveBuffer);
    NV_ASSERT(pBytesRead);
    if (hUart->IsSuspendedState)
        return NvError_InvalidState;

    // Check for the status and if it is in busy state then return error.
    if (hUart->ReadReq.IsBusy)
    {
        DEBUG_PRINT_RX_ERR(1, ("NvDdkUartRead(): Error occured Invalid state\n")); 
        return NvError_InvalidState;
    }

    LockReadFlow(hUart);
    if (hUart->ReadReq.IsBusy)
    {
        UnlockReadFlow(hUart);
        DEBUG_PRINT_RX_ERR(1, ("NvDdkUartRead(): Error occured Invalid state\n")); 
        return NvError_InvalidState;
    }

    // Get the data from the local buffer and if it has the requested number of 
    // bytes then comeout.
    BytesRead = ReadFromLocalBuffer(pReceiveBuffer, BytesRequested, &hUart->LocalRxBuff);
    // If read completely or if wait timeout is 0 (read whatever available 
    // in the fifo) or any error then return.
    if ((BytesRead == BytesRequested) || (!WaitTimeoutMs) || 
                        (hUart->ReadReq.TransferStatus))
    {
        Error = hUart->ReadReq.TransferStatus;
        hUart->ReadReq.TransferStatus = NvSuccess;

        // If flow control is enabled and if the data available is less then the
        // higher water level of the local buffer then enable the rts line.
        if (hUart->IsRtsFlowEnable)
        {
            if (hUart->LocalRxBuff.DataAvailable < hUart->LocalRxBuff.BufferLevelHigh)
                NvDdkPrivUartHwSetModemSignal(&hUart->UartHwRegs, 
                                                NvDdkUartSignalName_Rts, NV_TRUE);
        }

        UnlockReadFlow(hUart);
        *pBytesRead = BytesRead;
        DEBUG_PRINT_RX_ERR(Error, ("NvDdkUartRead(): Error occured 0x%08x\n",Error)); 
#if ENABLE_READ_PROFILE
        if (!Error)
            NvOsDebugPrintf("NvDdkUartRead(): EntryTime 0x%08x and ExitTime 0x%08x\n",
                                EntryTime, NvOsGetTimeMS());
#endif
        return Error;
     }

    hUart->ReadReq.pReqBuffer = pReceiveBuffer;
    hUart->ReadReq.pCurrTransBuf = pReceiveBuffer + BytesRead;
    hUart->ReadReq.BytesRequested = BytesRequested;
    hUart->ReadReq.BytesRemaining = BytesRequested - BytesRead;
    hUart->ReadReq.BytesTransferred = BytesRead;
    hUart->ReadReq.TransferStatus = NvSuccess;
    hUart->ReadReq.hOnCompleteSema = hUart->hRxSynchSema;
    hUart->ReadReq.IsStopReq = NV_FALSE;
    hUart->ReadReq.IsBusy = NV_TRUE;

    // Reset the semaphore count
    ResetSemaphoreCount(hUart->hRxSynchSema);
    UnlockReadFlow(hUart);
    *pBytesRead = 0;
    Error = NvOsSemaphoreWaitTimeout(hUart->ReadReq.hOnCompleteSema, WaitTimeoutMs);

    // Check the status
    LockReadFlow(hUart);
    if ((hUart->ReadReq.TransferStatus) || (hUart->ReadReq.IsStopReq) || 
                                                    (Error == NvError_Timeout))
    {
        hUart->ReadReq.IsBusy = NV_FALSE;
        if (!hUart->ReadReq.BytesRemaining)
            Error = NvSuccess;
        else
            Error = (hUart->ReadReq.TransferStatus)? hUart->ReadReq.TransferStatus: NvError_Timeout;
    }
    UnlockReadFlow(hUart);
    
    DEBUG_PRINT_RX_ERR(((Error) && (Error != NvError_Timeout)),
                    ("NvDdkUartRead(): Error occured 0x%08x\n",Error)); 
    *pBytesRead = hUart->ReadReq.BytesTransferred;

#if ENABLE_READ_PROFILE
    if (!Error)
        NvOsDebugPrintf("NvDdkUartRead(): EntryTime 0x%08x and ExitTime 0x%08x\n",
                            EntryTime, NvOsGetTimeMS());
#endif
    return Error;
}

/**
 * Stop the write opeartion.
 */
void NvDdkUartStopRead(NvDdkUartHandle hUart)
{
    NV_ASSERT(hUart);
    if (hUart->IsSuspendedState)
        return;

    // Update the current transfer state and current transfer parameters.
    LockReadFlow(hUart);
    // If read transfer state is not busy then invalid state error.
    if (hUart->ReadReq.IsBusy)
    {
        NvOsSemaphoreSignal(hUart->ReadReq.hOnCompleteSema);
        hUart->ReadReq.IsBusy = NV_FALSE;
        hUart->ReadReq.IsStopReq = NV_TRUE;
    }
    UnlockReadFlow(hUart);
}

/**
 * Transmit the data from com channel.
 * Thread safety: Provided in the function.
 */
NvError 
NvDdkUartWrite(
    NvDdkUartHandle hUart,   
    NvU8 *pTransmitBuffer,
    NvU32 BytesRequested,
    NvU32 *pBytesWritten,
    NvU32 WaitTimeoutMs)
{
    NvError Error = NvSuccess;
    NvU32 BytesWritten;
    NvU32 StartTime;
    NvU32 CurrentTime;
    NvU32 TimeElapsed;
    NvU32 DmaBytesRemaining;
    NvU32 TransferSize;
    NvU32 DmaTransferedCount;
    NvU32 Timeout;
    NvBool IsCtsActive;
#if DEBUG_WRITE_TIMEOUT
    NvBool IsCtsPrevActive = NV_FALSE;
#endif
#if ENABLE_WRITE_PROFILE
    NvU32 EntryTime = NvOsGetTimeMS();
#endif

    NV_ASSERT(hUart);
    NV_ASSERT(pTransmitBuffer);
    NV_ASSERT(pBytesWritten);

    // We dont support the timeout to be 0. Lets assert for debug.
    NV_ASSERT(WaitTimeoutMs);
    if (!WaitTimeoutMs)
        return NvError_BadParameter;
        
    if (hUart->IsSuspendedState)
        return NvError_InvalidState;

#if DEBUG_WRITE_TIMEOUT
        if (hUart->IsCtsFlowEnable)
            IsCtsPrevActive = NvDdkPrivUartHwIsCtsActive(&hUart->UartHwRegs);
        if (!IsCtsPrevActive)
            NvOsDebugPrintf("The CTS is inactive %d\n", IsCtsPrevActive);
#endif

    if (!hUart->IsFreqBoosted)
        NvOsSemaphoreSignal(hUart->hUartPowerMgrSema);
    
    LockWriteFlow(hUart);        
    // If current write transfer state is busy then return error.
    if (hUart->WriteReq.IsBusy)
    {
        UnlockWriteFlow(hUart);        
        return NvError_InvalidState;
    }
    hUart->WriteReq.IsBusy = NV_TRUE;
    hUart->WriteReq.pReqBuffer = pTransmitBuffer;
    hUart->WriteReq.pCurrTransBuf = pTransmitBuffer;
    hUart->WriteReq.BytesRequested = BytesRequested;
    hUart->WriteReq.BytesTransferred = 0;
    hUart->WriteReq.BytesRemaining = BytesRequested;
    hUart->WriteReq.IsStopReq = NV_FALSE;
    hUart->IsWritePolling = NV_FALSE;
    UnlockWriteFlow(hUart);        

    StartTime = NvOsGetTimeMS();
    // Use interrupt/dma based transfer if the number of bytes requested is more 
    // than polling limit.
    if (hUart->WriteReq.BytesRemaining > hUart->MaxTxBytesPolling)
    {
        if (hUart->IsDmaSupport)
        {   
            // Apb dma based transfer
            hUart->WriteReq.hOnCompleteSema = hUart->hTxSynchSema;

            // Reset the semaphore if it is already signalled.
            ResetSemaphoreCount(hUart->WriteReq.hOnCompleteSema);
            
            // APB dma accept multiple of 4 bytes only so transfer the data using the apb dma
            // to nearest of 4 in lower side.
            DmaBytesRemaining = ((hUart->WriteReq.BytesRequested >> 2) << 2);
            while (DmaBytesRemaining)
            {
                TransferSize = NV_MIN(hUart->TxDmaBufferSize, DmaBytesRemaining);
                NvOsMemcpy(hUart->pTxDmaBuffer, hUart->WriteReq.pCurrTransBuf, TransferSize);
                hUart->WDmaReq.TransferSize = TransferSize;
                Error = NvRmDmaStartDmaTransfer(hUart->hTxRmDma, &hUart->WDmaReq,
                                NvRmDmaDirection_Forward, 0, hUart->WriteReq.hOnCompleteSema);
                if (Error)
                    break;

                Error = NvOsSemaphoreWaitTimeout(hUart->WriteReq.hOnCompleteSema, WaitTimeoutMs);
                // The semaphore can be signalled by stopping the transfer or transfer
                // complets.
                if ((hUart->WriteReq.IsStopReq) || (Error == NvError_Timeout))
                {
                    Error = NvRmDmaGetTransferredCount(hUart->hTxRmDma, &DmaTransferedCount, NV_TRUE);

                    // Aborting the dma request if the dma current transaction incomplete
                    NvRmDmaAbort(hUart->hTxRmDma);
                    if (Error)
                    {
                        DmaTransferedCount = 0;
                    }
                    else
                    {
                        hUart->WriteReq.BytesTransferred += DmaTransferedCount;
                        hUart->WriteReq.pCurrTransBuf += DmaTransferedCount;
                        DmaBytesRemaining -= DmaTransferedCount;
                        // If transfer is not completed the return timeout error
                        if (!Error)
                            Error = (DmaBytesRemaining)? NvError_Timeout: NvSuccess;
                    }
                    break;
                }
                hUart->WriteReq.BytesTransferred += TransferSize;
                hUart->WriteReq.pCurrTransBuf += TransferSize;
                DmaBytesRemaining -= TransferSize;
            }

            hUart->WriteReq.BytesRemaining = hUart->WriteReq.BytesRequested -
                                    hUart->WriteReq.BytesTransferred;
        }
        else
        {   
            // Interrupt based transfer.
            hUart->WriteReq.hOnCompleteSema = hUart->hTxSynchSema;
            LockRegisterAccess(hUart);
            NvDdkPrivUartHwSetFifoTriggerLevel(&hUart->UartHwRegs, UartDataDirection_Transmit, 8);
            NvDdkPrivUartHwSetTransmitInterrupt(&hUart->UartHwRegs, NV_TRUE);
            UnlockRegisterAccess(hUart);
            Error = NvOsSemaphoreWaitTimeout(hUart->WriteReq.hOnCompleteSema, WaitTimeoutMs);
            if ((hUart->WriteReq.IsStopReq) || (Error == NvError_Timeout))
            {
                LockRegisterAccess(hUart);
                NvDdkPrivUartHwSetTransmitInterrupt(&hUart->UartHwRegs, NV_FALSE);
                UnlockRegisterAccess(hUart);
            }    
        }
    }

    if ((!Error) && (hUart->WriteReq.BytesRemaining) && (!hUart->WriteReq.IsStopReq))
    {
        hUart->IsWritePolling = NV_TRUE;
        
        // Do by polling
        while (hUart->WriteReq.BytesRemaining)
        {
            BytesWritten = NvDdkPrivUartHwPollingWriteInTransmitFifo(&hUart->UartHwRegs,
                                hUart->WriteReq.pCurrTransBuf, hUart->WriteReq.BytesRemaining);
            if (BytesWritten)
            {
                hUart->WriteReq.pCurrTransBuf += BytesWritten;
                hUart->WriteReq.BytesRemaining -= BytesWritten;
                hUart->WriteReq.BytesTransferred += BytesWritten;
                continue;
            }
            // Calculate the timeouts.
            if (WaitTimeoutMs == NV_WAIT_INFINITE)
                Timeout = NV_WAIT_INFINITE;
            else
            {
                CurrentTime = NvOsGetTimeMS();    
                TimeElapsed = (CurrentTime >= StartTime)? (CurrentTime - StartTime):
                                    0xFFFFFFFF-StartTime + CurrentTime;
                if (WaitTimeoutMs <= TimeElapsed)
                    break;
                Timeout = (WaitTimeoutMs - TimeElapsed);    
            }

            // CTS Hw flow disable: As CTS hw flow is disable we will get the 
            // fifo empty after transmitting the bytes as there is no stop 
            // for transmit data
            // CTS Hw flow enable: If CTS hw flow is enable and If CTS line 
            // is inactive then we dont know how much time it will take to
            // get the fifo empty and in this case better to wait for semaphore
            // with the trigger level interrupt.
            if (hUart->IsCtsFlowEnable)
            {
                IsCtsActive = NvDdkPrivUartHwIsCtsActive(&hUart->UartHwRegs);
                if (!IsCtsActive)
                {
                    // Wait for fifo get empty                        
                    hUart->WriteReq.hOnCompleteSema = hUart->hTxSynchSema;
                    LockRegisterAccess(hUart);
                    NvDdkPrivUartHwSetFifoTriggerLevel(&hUart->UartHwRegs, UartDataDirection_Transmit, 4);
                    NvDdkPrivUartHwSetTransmitInterrupt(&hUart->UartHwRegs, NV_TRUE);
                    UnlockRegisterAccess(hUart);
                    Error = NvOsSemaphoreWaitTimeout(hUart->WriteReq.hOnCompleteSema, Timeout);
                    if ((hUart->WriteReq.IsStopReq) || (Error == NvError_Timeout))
                    {
                        LockRegisterAccess(hUart);
                        NvDdkPrivUartHwSetTransmitInterrupt(&hUart->UartHwRegs, NV_FALSE);
                        UnlockRegisterAccess(hUart);
                        break;
                    }
                    continue;
                }
            }
            
            // Wait for the 4 character time in efficient way.
            // If more than 500 microsecond then wait with the NvOsSleep
            if (hUart->Char4TimeUs > 500)
                NvOsSleepMS((hUart->Char4TimeUs + 999)/1000);
            else
                NvOsWaitUS(hUart->Char4TimeUs);
        }        
    }

    DEBUG_PRINT_TX_ERR((Error == NvError_Timeout),
                ("NvDdkUartWrite(): BytesRequested 0x%08x and Timeout 0x%08x \n",
                                                BytesRequested, WaitTimeoutMs));
    DEBUG_PRINT_TX_ERR(((Error == NvError_Timeout) &&(hUart->IsCtsFlowEnable)),
        ("NvDdkUartWrite(): CTS State starting %d and end %d",IsCtsPrevActive, NvDdkPrivUartHwIsCtsActive(&hUart->UartHwRegs)));    

    // If the write is stopped the reset the Tx fifo.
    // Fixme!! Do we really need to reset the fifo?
    if (hUart->WriteReq.IsStopReq)
        NvDdkPrivUartHwResetFifo(&hUart->UartHwRegs, UartDataDirection_Transmit);
    
    *pBytesWritten = hUart->WriteReq.BytesTransferred;
    hUart->WriteReq.IsBusy = NV_FALSE;
    
#if ENABLE_WRITE_PROFILE
    if (!Error)
        NvOsDebugPrintf("NvDdkUartWrite(): EntryTime 0x%08x and ExitTime 0x%08x\n",
                            EntryTime, NvOsGetTimeMS());
#endif
    return Error;
}

/**
 * Stop the write opeartion.
 */
void NvDdkUartStopWrite(NvDdkUartHandle hUart)
{
    NV_ASSERT(hUart);
    if (hUart->IsSuspendedState)
        return;

    LockWriteFlow(hUart);  
    UartStopWrite(hUart, NV_TRUE);
    UnlockWriteFlow(hUart);
}

void 
NvDdkUartGetTransferStatus(
    NvDdkUartHandle hUart,
    NvU32 *pTxBytesToRemain,
    NvError *pTxStatus,
    NvU32 *pRxBytesAvailable,
    NvError *pRxStatus)
{
    NvU32 TxCount = 0;
    NvError TxStatus = NvSuccess;
    NvU32 RxBytesAvailable;
    NvError RxStatus = NvSuccess;
    
    NvError Err = NvSuccess;
    NV_ASSERT(hUart);
    if (hUart->IsSuspendedState)
        return;
    
    if (hUart->WriteReq.IsBusy)
    {
        if ((hUart->WriteReq.BytesRequested > hUart->MaxTxBytesPolling) &&
                        (hUart->IsDmaSupport))
        {
            Err = NvRmDmaGetTransferredCount(hUart->hTxRmDma,&TxCount, NV_FALSE);
            if (!Err)
                TxCount += hUart->WriteReq.BytesTransferred;
            else
                TxCount = hUart->WriteReq.BytesTransferred;
        }
        else
            TxCount = hUart->WriteReq.BytesTransferred;
        TxStatus = hUart->WriteReq.TransferStatus;
    }

    RxBytesAvailable = hUart->LocalRxBuff.DataAvailable;
    RxStatus = hUart->ReadReq.TransferStatus;

    *pTxBytesToRemain = TxCount;
    *pTxStatus = TxStatus;
    *pRxBytesAvailable = RxBytesAvailable;
    *pRxStatus =  RxStatus;
}

/**
 * Start/Stop sending the break signal.
 */
NvError NvDdkUartSetBreakSignal(NvDdkUartHandle hUart, NvBool IsStart)
{
    NV_ASSERT(hUart);
    if (hUart->IsSuspendedState)
        return NvError_InvalidState;

    // Start sending the break signal.
    LockRegisterAccess(hUart);
    if (IsStart)
        NvDdkPrivUartHwSetBreakControlSignal(&hUart->UartHwRegs, NV_TRUE);
    else
        NvDdkPrivUartHwSetBreakControlSignal(&hUart->UartHwRegs, NV_FALSE);
    UnlockRegisterAccess(hUart);
    return NvSuccess;
}

/**
 * Set the flow control signal level.
 */
NvError 
NvDdkUartSetFlowControlSignal(
    NvDdkUartHandle hUart, 
    NvDdkUartSignalName SignalName, 
    NvDdkUartFlowControl FlowControl)
{
    NvError Error = NvSuccess;
    NvU32 IsHigh;
    NV_ASSERT(hUart);
    if (hUart->IsSuspendedState)
        return NvError_InvalidState;

    LockRegisterAccess(hUart);
    if (FlowControl == NvDdkUartFlowControl_Handshake)
    {
        // For RTS/CTS handshake flow control, the number of line should be 4 
        // or more.
        if (hUart->NumberOfInterfaceLine < 4)
        {
            Error = NvError_NotSupported;
            goto Exit;
        }    
        
        if (SignalName & NvDdkUartSignalName_Rts)
            hUart->IsRtsFlowEnable = NV_TRUE;
            
        if (SignalName & NvDdkUartSignalName_Cts)
        {
            NvDdkPrivUartHwSetHWFlowControl(&hUart->UartHwRegs, NV_TRUE);
            hUart->IsCtsFlowEnable = NV_TRUE;
        }
        Error = NvSuccess;
        goto Exit;
    }

    IsHigh = (FlowControl == NvDdkUartFlowControl_Enable)? NV_TRUE: NV_FALSE;
    if (SignalName & NvDdkUartSignalName_Rts)
        hUart->IsRtsFlowEnable = NV_FALSE;
        
    if (SignalName & NvDdkUartSignalName_Cts)
    {
        if (FlowControl == NvDdkUartFlowControl_Enable)
        {
            NvDdkPrivUartHwSetHWFlowControl(&hUart->UartHwRegs, NV_TRUE);
            hUart->IsCtsFlowEnable = NV_TRUE;
        }    
        else
        {
            NvDdkPrivUartHwSetHWFlowControl(&hUart->UartHwRegs, NV_FALSE);
            hUart->IsCtsFlowEnable = NV_FALSE;
        }    
    }
    NvDdkPrivUartHwSetModemSignal(&hUart->UartHwRegs, SignalName, IsHigh);

Exit:    
    UnlockRegisterAccess(hUart);
    return Error;
}

/**
 * Get the input modem signal level status.
 */
NvError 
NvDdkUartGetFlowControlSignalLevel(
    NvDdkUartHandle hUart, 
    NvDdkUartSignalName SignalName,
    NvU32 *pSignalState)

{
    NvError Error = NvSuccess;
    NvU32 SignalState = 0;
    
    NV_ASSERT(hUart);
    NV_ASSERT(pSignalState);
    if (hUart->IsSuspendedState)
        return NvError_InvalidState;

    LockRegisterAccess(hUart);
    SignalState = (hUart->UartHwRegs.ModemSignalSatus & SignalName);
    UnlockRegisterAccess(hUart);
    *pSignalState = SignalState;
    return Error;
}

/**
 * Set semaphore for the modem signal status change. Any signal change in modem
 * line will signal this semaphore.
 */
NvError 
NvDdkUartRegisterModemSignalChange(
    NvDdkUartHandle hUart, 
    NvDdkUartSignalName SignalName,
    NvDdkUartSignalChangeCallback Callback,
    void *args)
{
    NV_ASSERT(hUart);
     if (hUart->IsSuspendedState)
        return NvError_InvalidState;

    LockRegisterAccess(hUart);
    // Update the semaphore for the modem signal status change.
    if (Callback != NULL)
    {
        hUart->hModemSigChngCallback = Callback;
        hUart->ModemSignalName = SignalName;
        hUart->ModemSigChngHandlerArgs = args;
        // Enable the modem control interrupt only when the number of line are
        // 8 i.e. full modem interface is available.
        if (hUart->NumberOfInterfaceLine == 8)
            NvDdkPrivUartHwSetModemControlInterrupt(&hUart->UartHwRegs, NV_TRUE);
    }
    else
    {
        hUart->hModemSigChngCallback = NULL;
        hUart->ModemSignalName = 0;
        hUart->ModemSigChngHandlerArgs = NULL;
        NvDdkPrivUartHwSetModemControlInterrupt(&hUart->UartHwRegs, NV_FALSE);
    }
    UnlockRegisterAccess(hUart);
    return NvSuccess;
}

NvError NvDdkUartSuspend(NvDdkUartHandle hUart)
{
    NvError Error;
    NvRmPowerState PowerState;
    NvRmModuleID ModuleId;
    NvU32 TimeCount = 50; // 100 ms waiting

    NV_ASSERT(hUart);
     if (hUart->IsSuspendedState)
        return NvSuccess;
    
    ModuleId = NVRM_MODULE_ID(NvRmModuleID_Uart, hUart->InstanceId);

    // First Declared as suspended state.
    hUart->IsSuspendedState = NV_TRUE;

    // Abort write if it is going
    LockWriteFlow(hUart);  
    UartStopWrite(hUart, NV_FALSE);
    UnlockWriteFlow(hUart);      

    // Wait for the write busy to come to non-busy, maximum 50 ms.
    while((hUart->WriteReq.IsBusy) && TimeCount)
    {
        NvOsSleepMS(2);
        TimeCount--;
    }
    // Still write busy then something wrong, so dont enter into suspend state.
    if (hUart->WriteReq.IsBusy)
    {
        hUart->IsSuspendedState = NV_FALSE;
        return NvError_InvalidState;
    }    

    // Wait for the Tx fifo to be empty.
    WaitTillTxFifoEmpty(hUart);

    // Now abort the read flow
    LockReadFlow(hUart);
    LockRegisterAccess(hUart);

    hUart->OldRtsState = hUart->UartHwRegs.IsRtsActive;
    hUart->OldDtrState = hUart->UartHwRegs.IsDtrActive;
    if (hUart->UartHwRegs.IsRtsActive)
        NvDdkPrivUartHwSetModemSignal(&hUart->UartHwRegs, NvDdkUartSignalName_Rts, NV_FALSE);

    if (hUart->UartHwRegs.IsDtrActive)
        NvDdkPrivUartHwSetModemSignal(&hUart->UartHwRegs, NvDdkUartSignalName_Dtr, NV_FALSE);

    // Abort the dma if it is started.
    if ((hUart->IsDmaReadStarted) && (hUart->IsRxApbDmaBased))
        NvRmDmaAbort(hUart->hRxRmDma);
        
    hUart->IsRxApbDmaBased = NV_FALSE;
    hUart->IsDmaReadStarted = NV_FALSE;
    hUart->LastReadIndex = 0;

    // Reset the uart controller to clear all status and interrupt request 
    // if there is any. This is require because after clock disabling, if we 
    // enter into the ISR, we may not be able to clear that.
    NvRmModuleReset(hUart->hRmDevice, ModuleId);

    // Disable the clock
    Error = NvRmPowerModuleClockControl(hUart->hRmDevice, ModuleId,
                                    hUart->RmPowerClientId, NV_FALSE);

    // Bring down the freq requirements.
    NvOsSemaphoreSignal(hUart->hUartPowerMgrSema);

    // Disable power
    if (!Error)
        Error = NvRmPowerVoltageControl(hUart->hRmDevice, ModuleId,
                    hUart->RmPowerClientId, NvRmVoltsOff, NvRmVoltsOff,
                    NULL, 0, NULL);

    // Tri-State the pin-mux pins
    NV_ASSERT_SUCCESS(NvRmSetModuleTristate(hUart->hRmDevice, ModuleId, NV_TRUE));

    // Check whether the power state is idle
    if (!Error)
    {
        Error = NvRmPowerGetState(hUart->hRmDevice, &PowerState);
        // if (!Error)
            // NV_ASSERT(PowerState != NvRmPowerState_Active);
    }
    UnlockRegisterAccess(hUart);
    UnlockReadFlow(hUart);
    return Error;
}

NvError NvDdkUartResume(NvDdkUartHandle hUart)
{
    NvError Error;
    NvRmPowerState PowerState;
    NvRmModuleID ModuleId;

    NV_ASSERT(hUart);
    if (!hUart->IsSuspendedState)
        return NvSuccess;

    ModuleId = NVRM_MODULE_ID(NvRmModuleID_Uart, hUart->InstanceId);

    LockReadFlow(hUart);
    LockRegisterAccess(hUart);

    // Reenable the pin nmux
    Error = NvRmSetModuleTristate(hUart->hRmDevice, ModuleId, NV_FALSE);
    
    // Enable power for uart module
    if (!Error)
        Error = NvRmPowerVoltageControl(hUart->hRmDevice, ModuleId,
                    hUart->RmPowerClientId,
                    NvRmVoltsUnspecified, NvRmVoltsUnspecified,
                    NULL, 0, NULL);

    // Enable the clock.
    if (!Error)
        Error = NvRmPowerModuleClockControl(hUart->hRmDevice, ModuleId,
                    hUart->RmPowerClientId, NV_TRUE);

    // Reset the module.
    if (!Error)
        NvRmModuleReset(hUart->hRmDevice, ModuleId);

    // Check whether the power state is active
    if (!Error)
    {
        Error = NvRmPowerGetState(hUart->hRmDevice, &PowerState);
        if (!Error)
            NV_ASSERT(PowerState == NvRmPowerState_Active);
    }

    // Restore the registers.
    if (!Error)
    {
        NvDdkPrivUartHwReconfigureRegisters(&hUart->UartHwRegs);

        // Restart the apb dma.
        if (hUart->IsDmaSupport)
        {
            hUart->RDmaReq.TransferSize = hUart->RxDmaBufferSize;
            Error = NvRmDmaStartDmaTransfer(hUart->hRxRmDma, &hUart->RDmaReq,
                    NvRmDmaDirection_Forward, 0, hUart->LocalRxBuff.hDataAvailableSema);
            if (!Error)
            {
                hUart->IsRxApbDmaBased = NV_TRUE;
                hUart->IsDmaReadStarted = NV_TRUE;
                hUart->LastReadIndex = 0;
            }
            Error = NvSuccess;
        }
        
        // Enable the receive interrupt.
        NvDdkPrivUartHwSetReceiveInterrupt(&hUart->UartHwRegs, NV_TRUE, hUart->IsRxApbDmaBased);
        if (hUart->OldRtsState)
            NvDdkPrivUartHwSetModemSignal(&hUart->UartHwRegs, NvDdkUartSignalName_Rts, NV_TRUE);
        if (hUart->OldDtrState)
            NvDdkPrivUartHwSetModemSignal(&hUart->UartHwRegs, NvDdkUartSignalName_Dtr, NV_TRUE);
    }
    
    hUart->IsSuspendedState = NV_FALSE;
    NvOsSemaphoreSignal(hUart->hUartPowerMgrSema);

    UnlockRegisterAccess(hUart);
    UnlockReadFlow(hUart);
    return Error;
}
