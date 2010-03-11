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
 *                 NvDDK SDIO Driver Implementation</b>
 *
 * @b Description: Implementation of the NvDDK SDIO API.
 *
 */

#include "nvddk_sdio.h"
#include "ap20/arsdmmc.h"
#include "ap20/arclk_rst.h"
#include "nvrm_drf.h"
#include "nvrm_hardware_access.h"
#include "nvodm_query_pinmux.h"
#include "nvrm_interrupt.h"
#include "nvassert.h"
#include "nvrm_memmgr.h"
#include "nvrm_pinmux.h"
#include "nvos.h"
#include "nvodm_pmu.h"
#include "nvodm_query_discovery.h"
#include "nvodm_query_gpio.h"
#include "nvrm_pmu.h"
#include "nvodm_sdio.h"
#include "nvodm_query.h"

#define ENABLE_DEBUG_PRINTS 0

#if ENABLE_DEBUG_PRINTS
#define SD_PRINT(x) NvOsDebugPrintf x
#else
#define SD_PRINT(x)
#endif

// Macro to get expression for modulo value that is power of 2
// Expression: DIVIDEND % (pow(2, Log2X))
#define MACRO_MOD_LOG2NUM(DIVIDEND, Log2X) \
    ((DIVIDEND) & ((1 << (Log2X)) - 1))

// Macro to get expression for multiply by number which is power of 2
// Expression: VAL * (1 << Log2Num)
#define MACRO_POW2_LOG2NUM(Log2Num) \
    (1 << (Log2Num))

// Macro to get expression for multiply by number which is power of 2
// Expression: VAL * (1 << Log2Num)
#define MACRO_MULT_POW2_LOG2NUM(VAL, Log2Num) \
    ((VAL) << (Log2Num))

// Macro to get expression for div by number that is power of 2
// Expression: VAL / (1 << Log2Num)
#define MACRO_DIV_POW2_LOG2NUM(VAL, Log2Num) \
    ((VAL) >> (Log2Num))

// The sdio controller needs a delay of 500 AHB clock cycles after
// the soft reset. Assuming the minimum AHB clock frequency as 10MHz,
// 500 AHB clock cycles will be equivalent to 50 USec. (refer Bug ID:  334872)
#define SDMMC_SOFT_RESET_DELAY_USEC 50

// Semaphore timeout for the sdio abort
#define NVDDK_SDMMC_ABORT_TIMEOUT_MSEC 10

// Delay required while doing sdio abort as per HW bug #371685.
#define SDMMC_ABORT_DELAY_USEC 2000

#define SDMMC_DMA_BUFFER_SIZE    \
    SDMMC_BLOCK_SIZE_BLOCK_COUNT_0_HOST_DMA_BUFFER_SIZE_DMA16K
#define SDMMC_DMA_TRANSFER_SIZE      (1 << (12 + SDMMC_DMA_BUFFER_SIZE))

// Size of the memory buffer size
#define SDMMC_MEMORY_BUFFER_SIZE ((SDMMC_DMA_TRANSFER_SIZE) * (2))

// Maximum sdio transfer size using polling method
#define SDMMC_MAX_POLLING_SIZE   4096

// Maximum polling time for completing the sdio transfer
#define SDMMC_MIN_POLLING_TIME_USEC 1500000

// Mmc erase command and timeout
#define MMC_ERASE_COMMAND 38
#define MMC_ERASE_COMMAND_MIN_POLLING_TIME_USEC 10000000


// Polling size(1MB) for timeout calculation
#define SDMMC_TIMEOUT_CALCULATION_SIZE (1024*1024)
// Delay between polling the interrupt status register
#define SDMMC_POLLING_DELAY_USEC  50

#define SDMMC_ERROR_STATUS_VALUE     0xFFFF0000
#define SDMMC_DEBOUNCE_TIME_MS 5

/* Minimum system frequency of 100MHz used in case of busy hints */
#define SDMMC_HW_MIN_SYSTEM_FREQ_KH   100000
#define SDMMC_EMC_MIN_SYSTEM_FREQ_KH 166000

// Frequency of the sdio controller when sdio controller is suspended
#define SDMMC_LOW_POWER_FREQ_KHZ 100

// Enable the following flag to enable the read busy hints.
#define ENABLE_READ_BUSY_HINTS  0

// Delay after issuing the abort
#define NVDDK_SDMMC_DELAY_AFTER_ABORT_USEC   40
// sdio interrupts used by the driver
#define SDIO_INTERRUPTS     0x7F000F
// sdio error interrupts
#define SDIO_ERROR_INTERRUPTS   0x7F0000


//Sdio command errors
#define SDIO_CMD_ERROR_INTERRUPTS  0xF0000
#define SDMMC_MAX_NUMBER_OF_BLOCKS   65536

enum
{
    SdioNormalModeMinFreq = 100,       // 100Khz
    SdioNormalModeMaxFreq = 25000,     // 25Mhz
    SdioHighSpeedModeMaxFreq = 52000
};

enum { SDMMC_INTERNAL_CLOCK_TIMEOUT_STEP_USEC = 10 };
enum { SDMMC_INTERNAL_CLOCK_TIMEOUT_USEC = 100 };

enum { SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL_STEP_USEC = 10 };
enum { SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL_USEC = 100 };

enum { SW_CONTROLLER_BUSY_TIMEOUT_STEP_USEC = 10 };
enum { SW_CONTROLLER_BUSY_TIMEOUT_USEC = 1000 };

enum
{
    SDMMC_MAX_BLOCK_SIZE_512 = 512,
    SDMMC_MAX_BLOCK_SIZE_1024 = 1024,
    SDMMC_MAX_BLOCK_SIZE_2048 = 2048
};

// Defines various MMC card specific command types

#define SDMMC_REGR(pSdioHwRegsVirtBaseAdd, reg) \
        NV_READ32((pSdioHwRegsVirtBaseAdd) + ((SDMMC_##reg##_0)/4))

#define SDMMC_REGW(pSdioHwRegsVirtBaseAdd, reg, val) \
    do\
    {\
        NV_WRITE32((((pSdioHwRegsVirtBaseAdd) + ((SDMMC_##reg##_0)/4))), (val));\
    }while (0)
    #define DIFF_FREQ(x, y) \
    (x>y)?(x-y):(y-x)

enum { SDMMC_ENABLE_ALL_INTERRUPTS = 0x000000FF };

#define NVDDK_SDMMC_COMMAND_TIMEOUT_MSEC 1000
#define NVDDK_SDMMC_DATA_TIMEOUT_MSEC 1000

/**
 * @brief Contains the sdio instance details . This information is shared
 * between the thread and the isr
 */
typedef struct NvDdkSdioInfoRec
{
    // Nvrm device handle
    NvRmDeviceHandle hRm;
    // Instance of the SDMMC module
    NvU32 Instance;
    // SDMMC configuration pin-map.
    NvOdmSdioPinMap PinMap;
    // Physical base address of the specific sdio instance
    NvRmPhysAddr SdioPhysicalAddress;
    // Virtual base address of the specific sdio instance
    NvU32* pSdioVirtualAddress;
    // size of the sdio register map
    NvU32 SdioBankSize;
   /**
    * @brief Semaphore to signal any notification to the sdio client modules.
    * The notifications include error conditions, Command complete,
    * read/write transfer complete, card related events etc.
    * The client is supposed to call the NvDdkSdioGetAsyncStatus API after
    * receiving a notification, to get the details of a particular event.
    */
    NvOsSemaphoreHandle NotificationSema;
    /** Card insertion/removal semaphore handle received from the client */
    NvOsSemaphoreHandle CardEventsSema;
    NvRmGpioHandle hGpio;
    // Bus width
    NvU32 BusWidth;
    // Bus voltage
    NvU32 BusVoltage;
    // High speed mode
    NvU32 IsHighSpeedEnabled;
    // Clock frequency
    NvRmFreqKHz ConfiguredFrequency;
    // flag to indicate sd memory or sdio mode
    NvBool IsAcceptCardEvents;
    // Indicates whether it is a read or a write transaction
    NvBool IsRead;
    // Request Type
    SdioRequestType RequestType;
    // Semaphore handle used for internal handshaking between the calling thread
    // and the isr
    NvOsSemaphoreHandle PrivSdioSema;
    // Rmmemory Handle of the buffer allocated
    NvRmMemHandle hRmMemHandle;
    // Physical Buffer Address of the memory
    NvU32 pPhysBuffer;
    // Virtual Buffer Address
    void* pVirtBuffer;
    // Interrupt handle
    NvRmGpioInterruptHandle GpioIntrHandle;
    NvDdkSdioStatus* ControllerStatus;
    NvRmGpioPinHandle WriteProtectPin;
    NvRmGpioPinHandle CardDetectPin;
    NvOsSemaphoreHandle SdioPowerMgtSema;
    NvU32 SdioRmPowerClientId;
    const NvOdmGpioPinInfo *GpioPinInfo;
    // Maximum block size supported by the controller
    NvU32 MaxBlockLength;
    NvOdmSdioHandle SdioOdmHandle;
    NvOsInterruptHandle InterruptHandle;
    NvBool ISControllerSuspended;
    NvBool IsSdControllerVersion2;
    NvOsIntrMutexHandle SdioThreadSafetyMutex;
}NvDdkSdioInfo;

typedef enum
{
    // data time out frequency = TMCLK/8K
    SdioDataTimeout_COUNTER_8K = 0,
    // data time out frequency = TMCLK/16K
    SdioDataTimeout_COUNTER_16K,
    // data time out frequency = TMCLK/32K
    SdioDataTimeout_COUNTER_32K,
    // data time out frequency = TMCLK/64K
    SdioDataTimeout_COUNTER_64K,
    // data time out frequency = TMCLK/128K
    SdioDataTimeout_COUNTER_128K,
    // data time out frequency = TMCLK/256K
    SdioDataTimeout_COUNTER_256K,
    // data time out frequency = TMCLK/512K
    SdioDataTimeout_COUNTER_512K,
    // data time out frequency = TMCLK/1M
    SdioDataTimeout_COUNTER_1M,
    // data time out frequency = TMCLK/2M
    SdioDataTimeout_COUNTER_2M,
    // data time out frequency = TMCLK/4M
    SdioDataTimeout_COUNTER_4M,
    // data time out frequency = TMCLK/8M
    SdioDataTimeout_COUNTER_8M,
    // data time out frequency = TMCLK/16M
    SdioDataTimeout_COUNTER_16M,
    // data time out frequency = TMCLK/32M
    SdioDataTimeout_COUNTER_32M,
    // data time out frequency = TMCLK/64M
    SdioDataTimeout_COUNTER_64M,
    // data time out frequency = TMCLK/128M
    SdioDataTimeout_COUNTER_128M,
    // data time out frequency = TMCLK/256M
    SdioDataTimeout_COUNTER_256M
}SdioDataTimeout;

// This is the default block size set whenever the nvddk sdio driver is open
enum { SDMMC_DEFAULT_BLOCK_SIZE = 512 };

static NvBool SdioEnableInternalClock(NvDdkSdioDeviceHandle hSdio);
static NvBool SdioIsReset(NvDdkSdioDeviceHandle hSdio);
static NvError SdioEnableBusPower(NvDdkSdioDeviceHandle hSdio);
static void ConfigureInterrupts(NvDdkSdioDeviceHandle hSdio, NvU32 IntrEnableMask, NvU32 IntrDisableMask, NvU32 IntrStatusEnableMask);
static void PrivSdioAbort(NvDdkSdioDeviceHandle hSdio);
void PrivSdioErrorRecovery(NvDdkSdioDeviceHandle hSdio);
NvError SdioEnableCardClock(NvDdkSdioDeviceHandle hSdio, NvBool IsEnable);

static NvError
SdioSetSlotClockRate(
    NvDdkSdioDeviceHandle hSdio,
    NvDdkSdioClkDivider Divider);

static NvError
    SdioSetDataTimeout(
        NvDdkSdioDeviceHandle hSdio,
        SdioDataTimeout SdioDataToCounter);

static NvU32
SdioIsControllerBusy(
    NvDdkSdioDeviceHandle hSdio,
    NvBool IsDataCommand);
static NvError
SdioRegisterInterrupts(
    NvRmDeviceHandle hRm,
    NvDdkSdioDeviceHandle hSdio);

static void SdioIsr(void* args);

static void GpioInterruptHandler(void *arg);

NvError SdioConfigureCardClock(NvDdkSdioDeviceHandle hSdio, NvBool IsEnable);

NvError
    SdioGetPhysAdd(
        NvRmDeviceHandle hRmDevice,
       NvRmMemHandle* hRmMemHandle,
        void** pVirtBuffer,
        NvU32 size,
        NvU32* pPhysBuffer);

static NvError SdioBlockTransfer(
    NvDdkSdioDeviceHandle hSdio,
    NvU32 NumOfRWBytes,
    NvU32 pReadBuffer,
    NvDdkSdioCommand *pRWRequest,
    NvBool HWAutoCMD12Enable,
    NvBool IsRead);

static NvError
PrivSdioPollingRead(
    NvDdkSdioDeviceHandle hSdio,
    NvU32 NumOfBytesToRead,
    void  *pReadBuffer,
    NvDdkSdioCommand *pRWCommand,
    NvBool HWAutoCMD12Enable,
    NvU32* SdioStatus);

static NvError
PrivSdioPollingWrite(
    NvDdkSdioDeviceHandle hSdio,
    NvU32 NumOfBytesToWrite,
    void *pWriteBuffer,
    NvDdkSdioCommand *pRWCommand,
    NvBool HWAutoCMD12Enable,
    NvU32* SdioStatus);

void PrivSdioReset(NvDdkSdioDeviceHandle hSdio);

NvError
PrivSdioSendCommandPolling(
        NvDdkSdioDeviceHandle hSdio,
        NvDdkSdioCommand *pCommand,
        NvU32* SdioStatus);

void
PrivSdioGetCaps(
    NvRmDeviceHandle hDevice,
    NvDdkSdioHostCapabilities *pHostCap,
    NvU32 instance);

// function to check power of 2
static NvBool
UtilCheckPowerOf2(NvU32 Num)
{
    // A power of 2 satisfies condition (N & (N - 1)) == (2 * N - 1)
    if ((Num & (Num - 1)) == 0)
        return NV_TRUE;
    else
        return NV_FALSE;
}

// Simple function to get log2, assumed value power of 2, else return 
// returns log2 of immediately smaller number
NvU8
SdUtilGetLog2(NvU32 Val)
{
    NvU8 Log2Val = 0;
    NvU32 i;
    // Value should be non-zero
    NV_ASSERT(Val > 0);
    if (UtilCheckPowerOf2(Val) == NV_FALSE)
    {
        NvOsDebugPrintf("\nCalling simple log2 with value which is "
            "not power of 2 ");
        // In case of values that are not power of 2 we return the 
        // integer part of the result of log2
    }
    // Value is power of 2
    if (Val > 0)
    {
        // Assumed that Val is NvU32
        for (i = 0; i < 32; i++)
        {
            // divide by 2
            Val = MACRO_DIV_POW2_LOG2NUM(Val, 1);
            if (Val == 0)
            {
                // Return 0 when Val is 1
                break;
            }
            Log2Val++;
        }
    }
    return Log2Val;
}



NvError NvDdkSdioOpen(
    NvRmDeviceHandle hDevice,
    NvRmGpioHandle hGpio,
    NvDdkSdioDeviceHandle *phSdio,
    NvOsSemaphoreHandle *pNotificationSema,
    NvOsSemaphoreHandle *pCardEventsSema,
    NvU8 Instance)
{

    NvError e = NvSuccess;
    NvBool IsReset = NV_TRUE;
    NvBool IsClkStable = NV_FALSE;
    NvDdkSdioDeviceHandle hSdio = NULL;
    NvRmFreqKHz pConfiguredFrequencyKHz = 0;
    NvDdkSdioStatus* ControllerStatus = NULL;
    NvU32 val = 0;
    NvU32 capabilities = 0;
    NvU32 MaxInstances = 0;
    NvBool IsCardInserted = NV_TRUE;
    NvDdkSdioHostCapabilities HostCap;

#if !NV_OAL
    NvU32 PinCount = 0;
    NvOsInterruptHandler IntrHandler = (NvOsInterruptHandler)GpioInterruptHandler;

    NV_ASSERT(hGpio);
#endif

    NV_ASSERT(pNotificationSema);
    NV_ASSERT(pCardEventsSema);

    MaxInstances = NvRmModuleGetNumInstances(hDevice, NvRmModuleID_Sdio);
    // Validate the instance number
    if (Instance >= MaxInstances)
    {
        return NvError_SdioInstanceTaken;
    }

    if (NvRmSetModuleTristate(hDevice,
        NVRM_MODULE_ID(NvRmModuleID_Sdio,Instance), NV_FALSE)!=NvSuccess)
        return NvError_NotSupported;

    hSdio = NvOsAlloc(sizeof(NvDdkSdioInfo));

    if (!hSdio)
    {
        return NvError_InsufficientMemory;
    }

    NvOsMemset(hSdio, 0, sizeof(NvDdkSdioInfo));

    // initialise the members of the NvDdkSdioDeviceHandle struct
    hSdio->hRm = hDevice;
    hSdio->Instance = Instance;
    hSdio->NotificationSema = *pNotificationSema;
    hSdio->ConfiguredFrequency = SdioNormalModeMinFreq;
    hSdio->CardEventsSema = *pCardEventsSema;
    hSdio->IsSdControllerVersion2 = NV_FALSE;
    hSdio->ISControllerSuspended = NV_FALSE;

    NvRmModuleGetBaseAddress(
        hDevice,
        NVRM_MODULE_ID(NvRmModuleID_Sdio, Instance),
        &hSdio->SdioPhysicalAddress,
        &hSdio->SdioBankSize);

    NV_CHECK_ERROR_CLEANUP(NvRmPhysicalMemMap(
        hSdio->SdioPhysicalAddress,
        hSdio->SdioBankSize, NVOS_MEM_READ_WRITE,
        NvOsMemAttribute_Uncached,
        (void **)&hSdio->pSdioVirtualAddress));

    ControllerStatus = NvOsAlloc(sizeof(NvDdkSdioStatus));
    if (NULL == ControllerStatus)
    {
        e = NvError_InsufficientMemory;
        goto fail;
    }

    e = NvOsIntrMutexCreate(&hSdio->SdioThreadSafetyMutex);
    if (e != NvError_Success)
    {
        goto fail;
    }

    ControllerStatus->SDControllerStatus = NvDdkSdioCommandStatus_None;
    ControllerStatus->SDErrorStatus = NvDdkSdioError_None;
    hSdio->ControllerStatus = ControllerStatus;

     // Event sema to register with the rm_power module
    NV_CHECK_ERROR_CLEANUP(NvOsSemaphoreCreate(&hSdio->PrivSdioSema, 0));

     // Event sema to register with the rm_power module
    NV_CHECK_ERROR_CLEANUP(NvOsSemaphoreCreate(&hSdio->SdioPowerMgtSema, 0));

    // register with the rm_power manager
    hSdio->SdioRmPowerClientId = NVRM_POWER_CLIENT_TAG('S','D','I','O');
    NV_CHECK_ERROR_CLEANUP(NvRmPowerRegister(hSdio->hRm,
                                             hSdio->SdioPowerMgtSema,
                                             &hSdio->SdioRmPowerClientId));

    hSdio->SdioOdmHandle = NvOdmSdioOpen(Instance);
    if (!hSdio->SdioOdmHandle)
    {
        e = NvError_NotSupported;
        goto fail;
    }

    // enable power
    NV_CHECK_ERROR_CLEANUP(NvRmPowerVoltageControl(hSdio->hRm,
                                                   NVRM_MODULE_ID(NvRmModuleID_Sdio, Instance),
                                                   hSdio->SdioRmPowerClientId,
                                                   NvRmVoltsUnspecified,
                                                   NvRmVoltsUnspecified,
                                                   NULL,
                                                   0,
                                                   NULL));

    // now enable clock to sdio controller
    NV_CHECK_ERROR_CLEANUP(NvRmPowerModuleClockControl(hSdio->hRm,
                                                       NVRM_MODULE_ID(NvRmModuleID_Sdio, Instance),
                                                       hSdio->SdioRmPowerClientId,
                                                       NV_TRUE));



    // reset controller
    NvRmModuleReset(hSdio->hRm, NVRM_MODULE_ID(NvRmModuleID_Sdio, Instance));

    IsReset = SdioIsReset(hSdio);
    if (IsReset != NV_TRUE)
    {
        goto fail;
    }

    // enable internal clock to sdio
    IsClkStable = SdioEnableInternalClock(hSdio);
    if (IsClkStable != NV_TRUE)
    {
        goto fail;
    }

    // Configure the clock frequency
    NV_CHECK_ERROR_CLEANUP(NvDdkSdioSetClockFrequency(hSdio,
                                                      hSdio->ConfiguredFrequency,
                                                      &pConfiguredFrequencyKHz));

    PrivSdioGetCaps(hDevice, &HostCap, Instance);
    hSdio->IsSdControllerVersion2 = HostCap.IsSdControllerVersion2;

    capabilities = SDMMC_REGR(hSdio->pSdioVirtualAddress, CAPABILITIES);
    val = NV_DRF_VAL(SDMMC, CAPABILITIES, MAX_BLOCK_LENGTH, capabilities);
    switch (val)
    {
      case SDMMC_CAPABILITIES_0_MAX_BLOCK_LENGTH_BYTE512:
           hSdio->MaxBlockLength = SDMMC_MAX_BLOCK_SIZE_512;
           break;

      case SDMMC_CAPABILITIES_0_MAX_BLOCK_LENGTH_BYTE1024:
           hSdio->MaxBlockLength = SDMMC_MAX_BLOCK_SIZE_1024;
           break;

      case SDMMC_CAPABILITIES_0_MAX_BLOCK_LENGTH_BYTE2048:
           hSdio->MaxBlockLength = SDMMC_MAX_BLOCK_SIZE_2048;
           break;

     default:
           hSdio->MaxBlockLength = SDMMC_DEFAULT_BLOCK_SIZE;
           break;
    }
    HostCap.MaxBlockLength = hSdio->MaxBlockLength;
    
    // Set the block size
    NvDdkSdioSetBlocksize(hSdio, SDMMC_DEFAULT_BLOCK_SIZE);

    val = NV_DRF_VAL(SDMMC, CAPABILITIES, VOLTAGE_SUPPORT_3_3_V, capabilities);
    if (val)
    {
        HostCap.BusVoltage = NvDdkSdioSDBusVoltage_3_3;
    }
    else
    {
        val = NV_DRF_VAL(SDMMC, CAPABILITIES, VOLTAGE_SUPPORT_3_0_V, capabilities);
        if (val)
        {
                HostCap.BusVoltage = NvDdkSdioSDBusVoltage_3_0;
        }
        else
        {
            val = NV_DRF_VAL(SDMMC, CAPABILITIES, VOLTAGE_SUPPORT_1_8_V, capabilities);
            if (val)
            {
                HostCap.BusVoltage = NvDdkSdioSDBusVoltage_1_8;
            }
            else
            {
                // Invalid bus voltage
                NV_ASSERT(0);
            }
        }
    }
    
    // set sd bus voltage
    NvDdkSdioSetSDBusVoltage(hSdio, HostCap.BusVoltage);


    // enable sd bus power
    SdioEnableBusPower(hSdio);

    // set data timeout counter value
    SdioSetDataTimeout(hSdio, SdioDataTimeout_COUNTER_128M);

    // register interrupt handler
    NV_CHECK_ERROR_CLEANUP(SdioRegisterInterrupts(hDevice, hSdio));

#if !NV_OAL
    hSdio->hGpio = hGpio;

    hSdio->GpioPinInfo = NvOdmQueryGpioPinMap(NvOdmGpioPinGroup_Sdio,
                                              hSdio->Instance, &PinCount);

    if (hSdio->GpioPinInfo != NULL)
    {
        // Check whether the write protect gpio pin is supported or not
        if (PinCount == 1)
        {
            NvRmGpioAcquirePinHandle(hGpio, hSdio->GpioPinInfo[0].Port,
                    hSdio->GpioPinInfo[0].Pin, &hSdio->CardDetectPin);
        }
        else if (PinCount == 2)
        {
            NvRmGpioAcquirePinHandle(hGpio, hSdio->GpioPinInfo[0].Port,
                    hSdio->GpioPinInfo[0].Pin, &hSdio->CardDetectPin);

            NvRmGpioAcquirePinHandle(hGpio, hSdio->GpioPinInfo[1].Port,
                    hSdio->GpioPinInfo[1].Pin, &hSdio->WriteProtectPin);

            NV_CHECK_ERROR_CLEANUP(NvRmGpioConfigPins(hGpio, &hSdio->WriteProtectPin, 1,
                NvRmGpioPinMode_InputData));
        }

        e = NvRmGpioInterruptRegister(hGpio, hDevice, hSdio->CardDetectPin,
                IntrHandler, NvRmGpioPinMode_InputInterruptAny,
                hSdio, &hSdio->GpioIntrHandle, SDMMC_DEBOUNCE_TIME_MS);
        if (e != NvError_Success)
        {
            goto fail;
        }

        e = NvRmGpioInterruptEnable(hSdio->GpioIntrHandle);
        if (e != NvError_Success)
        {
            goto fail;
        }
    }
#endif

    // Enable ALL important Normal interrupts
    ConfigureInterrupts(hSdio, SDIO_INTERRUPTS, ~SDIO_INTERRUPTS, 0);


    *phSdio = hSdio;
    SdioConfigureCardClock(hSdio, NV_FALSE);

    // Allocate memory for sdio data transfers
    NV_CHECK_ERROR_CLEANUP(SdioGetPhysAdd(hSdio->hRm, &hSdio->hRmMemHandle,
                        &hSdio->pVirtBuffer, SDMMC_MEMORY_BUFFER_SIZE, &hSdio->pPhysBuffer));

    // Disable power to the slot if the card is not inserted
    e = NvDdkSdioIsCardInserted(hSdio, &IsCardInserted);
    if (e == NvSuccess)
    {
        if (!IsCardInserted)
        {
            NvOdmSdioSuspend(hSdio->SdioOdmHandle);
        }
    }

    return NvSuccess;

fail:
    NvOsIntrMutexDestroy(hSdio->SdioThreadSafetyMutex);
    hSdio->SdioThreadSafetyMutex = NULL;
    if (hSdio->SdioOdmHandle)
    {
        NvOdmSdioClose(hSdio->SdioOdmHandle);
    }

    if (hSdio->hRmMemHandle != NULL)
    {
#if !NV_OAL
        NvRmMemUnmap(hSdio->hRmMemHandle, hSdio->pVirtBuffer, SDMMC_MEMORY_BUFFER_SIZE);
#endif
        NvRmMemUnpin(hSdio->hRmMemHandle);
        NvRmMemHandleFree(hSdio->hRmMemHandle);
    }

     // disable power
    NV_ASSERT_SUCCESS(NvRmPowerVoltageControl(hSdio->hRm,
                            NVRM_MODULE_ID(NvRmModuleID_Sdio, Instance),
                            hSdio->SdioRmPowerClientId,
                            NvRmVoltsOff,
                            NvRmVoltsOff,
                            NULL,
                            0,
                            NULL));

     // unregister with the power manager
    NvRmPowerUnRegister(hSdio->hRm, hSdio->SdioRmPowerClientId);

    // unregister/disable interrupt handler
    NvRmInterruptUnregister(hDevice, hSdio->InterruptHandle);
    hSdio->InterruptHandle = NULL;

    if (hSdio != NULL)
    {
        // destory the internal sdio abort semaphore
        NvOsSemaphoreDestroy(hSdio->PrivSdioSema);
        hSdio->PrivSdioSema = NULL;

        NvRmPhysicalMemUnmap(hSdio->pSdioVirtualAddress, hSdio->SdioBankSize);
    }

    NvOsFree(ControllerStatus);
    ControllerStatus = NULL;
    hSdio->ControllerStatus = NULL;
    NvOsSemaphoreDestroy(hSdio->SdioPowerMgtSema);
    hSdio->SdioPowerMgtSema = NULL;
    NvOsFree(hSdio);
    hSdio = NULL;
    *phSdio = NULL;
    NV_ASSERT_SUCCESS(NvRmSetModuleTristate(hDevice,
        NVRM_MODULE_ID(NvRmModuleID_Sdio,Instance), NV_TRUE));
    return e;
}

NvError
NvDdkSdioSendCommand(
        NvDdkSdioDeviceHandle hSdio,
        NvDdkSdioCommand *pCommand,
        NvU32* SdioStatus)
{
    NvU32 val = 0;
    NvBool IsCrcCheckEnable = NV_FALSE;
    NvBool IsIndexCheckEnable = NV_FALSE;
    NvU32 IsControllerBusy = 0;
    NvError status;
    NvDdkSdioStatus* ControllerStatus = NULL;

    NV_ASSERT(hSdio);
    NV_ASSERT(pCommand);

    ControllerStatus = hSdio->ControllerStatus;

    // clear the previous status if any
    ControllerStatus->SDControllerStatus = NvDdkSdioCommandStatus_None;
    ControllerStatus->SDErrorStatus = NvDdkSdioError_None;
    hSdio->RequestType = NVDDK_SDIO_NORMAL_REQUEST;

#if NV_OAL
    status = PrivSdioSendCommandPolling(hSdio, pCommand, SdioStatus);
    return status;
#endif

    // Commands with response type as R1b will generate transfer complete.
    // Check if the command response is R1b. If the response is R1b we will get
    // transfer complete interrupt.
    if (pCommand->ResponseType == NvDdkSdioRespType_R1b && pCommand->CommandCode != 12)
    {
        // Disable command complete and enable transfer complete
        ConfigureInterrupts(hSdio, SDIO_INTERRUPTS, NvDdkSdioCommandStatus_CommandComplete, 0);
    }
    else
    {
        // Enable command complete and disable transfer complete
        ConfigureInterrupts(hSdio, SDIO_INTERRUPTS, NvDdkSdioCommandStatus_TransferComplete, 0);
    }

    val = SDMMC_REGR(hSdio->pSdioVirtualAddress, CMD_XFER_MODE);

    // set the command number
    val = NV_FLD_SET_DRF_NUM(SDMMC, CMD_XFER_MODE, COMMAND_INDEX,
                             pCommand->CommandCode, val);

    // check if any data transfer is involved in the command
    val = NV_FLD_SET_DRF_NUM(SDMMC, CMD_XFER_MODE, DATA_PRESENT_SELECT,
                             pCommand->IsDataCommand, val);
    val = NV_FLD_SET_DRF_NUM(SDMMC, CMD_XFER_MODE, COMMAND_TYPE,
                                pCommand->CommandType, val);
    /* set the response type */
    switch (pCommand->ResponseType)
    {
        case NvDdkSdioRespType_NoResp:
             IsCrcCheckEnable = NV_FALSE;
             IsIndexCheckEnable = NV_FALSE;
             val = NV_FLD_SET_DRF_DEF(SDMMC, CMD_XFER_MODE, RESP_TYPE_SELECT,
                                      NO_RESPONSE, val);
            break;

        case NvDdkSdioRespType_R1:
        case NvDdkSdioRespType_R5:
        case NvDdkSdioRespType_R6:
             IsCrcCheckEnable = NV_TRUE;
             IsIndexCheckEnable = NV_TRUE;
             val = NV_FLD_SET_DRF_DEF(SDMMC, CMD_XFER_MODE, RESP_TYPE_SELECT,
                                      RESP_LENGTH_48, val);
            break;

        case NvDdkSdioRespType_R2:
             IsCrcCheckEnable = NV_TRUE;
             IsIndexCheckEnable = NV_FALSE;
             val = NV_FLD_SET_DRF_DEF(SDMMC, CMD_XFER_MODE, RESP_TYPE_SELECT,
                                      RESP_LENGTH_136, val);
            break;

        case NvDdkSdioRespType_R3:
        case NvDdkSdioRespType_R4:
             IsCrcCheckEnable = NV_FALSE;
             IsIndexCheckEnable = NV_FALSE;
             val = NV_FLD_SET_DRF_DEF(SDMMC, CMD_XFER_MODE, RESP_TYPE_SELECT,
                                      RESP_LENGTH_48, val);
            break;

        case NvDdkSdioRespType_R1b:
             IsCrcCheckEnable = NV_TRUE;
             IsIndexCheckEnable = NV_TRUE;
             val = NV_FLD_SET_DRF_DEF(SDMMC, CMD_XFER_MODE, RESP_TYPE_SELECT,
                                      RESP_LENGTH_48BUSY, val);
            break;

        case NvDdkSdioRespType_R7:
             IsCrcCheckEnable = NV_TRUE;
             IsIndexCheckEnable = NV_FALSE;
             val = NV_FLD_SET_DRF_DEF(SDMMC, CMD_XFER_MODE, RESP_TYPE_SELECT,
                                      RESP_LENGTH_48, val);
            break;

        default:
            NV_ASSERT(0);
    }

    // set the is indexed
    val = NV_FLD_SET_DRF_NUM(SDMMC, CMD_XFER_MODE, CMD_INDEX_CHECK_EN,
                             IsIndexCheckEnable, val);

    // set the is crc
    val = NV_FLD_SET_DRF_NUM(SDMMC, CMD_XFER_MODE, CMD_CRC_CHECK_EN,
                             IsCrcCheckEnable, val);

    IsControllerBusy = SdioIsControllerBusy(hSdio, pCommand->IsDataCommand);
    if (IsControllerBusy)
    {
        return NvError_SdioControllerBusy;
    }

    // now write to the command argument register
    SDMMC_REGW(hSdio->pSdioVirtualAddress, ARGUMENT, pCommand->CmdArgument);

    SdioConfigureCardClock(hSdio, NV_TRUE);

    // now write to the command xfer register
    SDMMC_REGW(hSdio->pSdioVirtualAddress, CMD_XFER_MODE, val);

    status = NvOsSemaphoreWaitTimeout(hSdio->PrivSdioSema, NVDDK_SDMMC_COMMAND_TIMEOUT_MSEC);

    if (status == NvSuccess)
    {
        // if there is a Command timeout because of the previous command, reset CMD line
        if (ControllerStatus->SDErrorStatus & NvDdkSdioError_CommandTimeout)
        {
            NvOsWaitUS(SDMMC_SOFT_RESET_DELAY_USEC);
            val = SDMMC_REGR(hSdio->pSdioVirtualAddress,
                                    SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL);
            val = NV_DRF_VAL(SDMMC, SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL,
                                    SW_RESET_FOR_CMD_LINE, val);
            NV_ASSERT(val == 0);
        }

        // if there is a data timeout because of the previous command, reset DAT line
        if (ControllerStatus->SDErrorStatus & NvDdkSdioError_DataTimeout)
        {
            NvOsWaitUS(SDMMC_SOFT_RESET_DELAY_USEC);
            val = SDMMC_REGR(hSdio->pSdioVirtualAddress,
                                    SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL);
            val = NV_DRF_VAL(SDMMC, SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL,
                                    SW_RESET_FOR_DAT_LINE, val);
            NV_ASSERT(val == 0);
        }
    }

    SdioConfigureCardClock(hSdio, NV_FALSE);

    *SdioStatus = ControllerStatus->SDErrorStatus;
    return status;
}

NvError
PrivSdioSendCommandPolling(
        NvDdkSdioDeviceHandle hSdio,
        NvDdkSdioCommand *pCommand,
        NvU32* SdioStatus)
{
    NvU32 val = 0;
    NvBool IsCrcCheckEnable = NV_FALSE;
    NvBool IsIndexCheckEnable = NV_FALSE;
    NvU32 IsControllerBusy = 0;
    NvError status = NvSuccess;
    NvDdkSdioStatus* ControllerStatus = NULL;
    NvU32 PollTime = 0;
    NvU32 TotalPollingTime = SDMMC_MIN_POLLING_TIME_USEC;

    if (pCommand->CommandCode == MMC_ERASE_COMMAND)
    {
        TotalPollingTime = MMC_ERASE_COMMAND_MIN_POLLING_TIME_USEC;
    }

    NV_ASSERT(hSdio);
    NV_ASSERT(pCommand);

    ControllerStatus = hSdio->ControllerStatus;

    // clear the previous status if any
    ControllerStatus->SDControllerStatus = NvDdkSdioCommandStatus_None;
    ControllerStatus->SDErrorStatus = NvDdkSdioError_None;
    hSdio->RequestType = NVDDK_SDIO_NORMAL_REQUEST;

    // Commands with response type as R1b will generate transfer complete.
    // Check if the command response is R1b. If the response is R1b we will get
    // transfer complete interrupt.
    // Commands with response type as R1b will generate transfer complete.
    // Check if the command response is R1b. If the response is R1b we will get
    // transfer complete interrupt.
    if (pCommand->ResponseType == NvDdkSdioRespType_R1b && pCommand->CommandCode != 12)
    {
        // Disable all interrupts
        ConfigureInterrupts(hSdio, 0, SDIO_INTERRUPTS,
                    (SDIO_ERROR_INTERRUPTS|NvDdkSdioCommandStatus_TransferComplete));
    }
    else
    {
        // Enable command complete and disable transfer complete
        ConfigureInterrupts(hSdio, 0, SDIO_INTERRUPTS,
                (SDIO_ERROR_INTERRUPTS | NvDdkSdioCommandStatus_CommandComplete));
    }

    // set the command number
    val |= NV_DRF_NUM(SDMMC, CMD_XFER_MODE, COMMAND_INDEX,
                             pCommand->CommandCode);
    // check if any data transfer is involved in the command
    val |= NV_DRF_NUM(SDMMC, CMD_XFER_MODE, DATA_PRESENT_SELECT,
                             pCommand->IsDataCommand);
    val |= NV_DRF_NUM(SDMMC, CMD_XFER_MODE, COMMAND_TYPE,
                             pCommand->CommandType);
    /* set the response type */
    switch (pCommand->ResponseType)
    {
        case NvDdkSdioRespType_NoResp:
             IsCrcCheckEnable = NV_FALSE;
             IsIndexCheckEnable = NV_FALSE;
             val = NV_FLD_SET_DRF_DEF(SDMMC, CMD_XFER_MODE, RESP_TYPE_SELECT,
                                      NO_RESPONSE, val);
            break;

        case NvDdkSdioRespType_R1:
        case NvDdkSdioRespType_R5:
        case NvDdkSdioRespType_R6:
             IsCrcCheckEnable = NV_TRUE;
             IsIndexCheckEnable = NV_TRUE;
             val |= NV_DRF_DEF(SDMMC, CMD_XFER_MODE, RESP_TYPE_SELECT,
                                      RESP_LENGTH_48);
            break;

        case NvDdkSdioRespType_R2:
             IsCrcCheckEnable = NV_TRUE;
             IsIndexCheckEnable = NV_FALSE;
             val |= NV_DRF_DEF(SDMMC, CMD_XFER_MODE, RESP_TYPE_SELECT,
                                      RESP_LENGTH_136);
            break;

        case NvDdkSdioRespType_R3:
        case NvDdkSdioRespType_R4:
             IsCrcCheckEnable = NV_FALSE;
             IsIndexCheckEnable = NV_FALSE;
             val |= NV_DRF_DEF(SDMMC, CMD_XFER_MODE, RESP_TYPE_SELECT,
                                      RESP_LENGTH_48);
            break;

        case NvDdkSdioRespType_R1b:
             IsCrcCheckEnable = NV_TRUE;
             IsIndexCheckEnable = NV_TRUE;
             val |= NV_DRF_DEF(SDMMC, CMD_XFER_MODE, RESP_TYPE_SELECT,
                                      RESP_LENGTH_48BUSY);
            break;

        case NvDdkSdioRespType_R7:
             IsCrcCheckEnable = NV_TRUE;
             IsIndexCheckEnable = NV_FALSE;
             val |= NV_DRF_DEF(SDMMC, CMD_XFER_MODE, RESP_TYPE_SELECT,
                                      RESP_LENGTH_48);
            break;

        default:
            NV_ASSERT(0);
    }

    // set the is indexed
    val |= NV_DRF_NUM(SDMMC, CMD_XFER_MODE, CMD_INDEX_CHECK_EN,
                             IsIndexCheckEnable);

    // set the is crc
    val |= NV_DRF_NUM(SDMMC, CMD_XFER_MODE, CMD_CRC_CHECK_EN,
                             IsCrcCheckEnable);

    IsControllerBusy = SdioIsControllerBusy(hSdio, pCommand->IsDataCommand);
    if (IsControllerBusy)
    {
        return NvError_SdioControllerBusy;
    }

    *SdioStatus = NvDdkSdioError_None;

    // now write to the command argument register
    SDMMC_REGW(hSdio->pSdioVirtualAddress, ARGUMENT, pCommand->CmdArgument);

    SdioConfigureCardClock(hSdio, NV_TRUE);

    // now write to the command xfer register
    SDMMC_REGW(hSdio->pSdioVirtualAddress, CMD_XFER_MODE, val);

    // poll for the transfer or command complete interrupt
    while (PollTime < TotalPollingTime)
    {
        val = SDMMC_REGR(hSdio->pSdioVirtualAddress, INTERRUPT_STATUS);
        if (val & SDMMC_ERROR_STATUS_VALUE)
        {
            val &= SDMMC_ERROR_STATUS_VALUE;
            SDMMC_REGW(hSdio->pSdioVirtualAddress, INTERRUPT_STATUS, val);
            *SdioStatus = val;
            PrivSdioReset(hSdio);
            status = NvSuccess;
            break;
        }
        else if (NV_DRF_VAL(SDMMC, INTERRUPT_STATUS, XFER_COMPLETE, val))
        {
            val = NV_DRF_DEF(SDMMC, INTERRUPT_STATUS, XFER_COMPLETE, GEN_INT);
            SDMMC_REGW(hSdio->pSdioVirtualAddress, INTERRUPT_STATUS, val);
            status = NvSuccess;
            break;
        }
        else if (NV_DRF_VAL(SDMMC, INTERRUPT_STATUS, CMD_COMPLETE, val))
        {
            val = NV_DRF_DEF(SDMMC, INTERRUPT_STATUS, CMD_COMPLETE, GEN_INT);
            SDMMC_REGW(hSdio->pSdioVirtualAddress, INTERRUPT_STATUS, val);
            status = NvSuccess;
            break;
        }
        NvOsWaitUS(SDMMC_POLLING_DELAY_USEC);
        PollTime += SDMMC_POLLING_DELAY_USEC;
    }
    if (PollTime >= TotalPollingTime)
    {
        PrivSdioReset(hSdio);
        status = NvError_Timeout;
    }

    SdioConfigureCardClock(hSdio, NV_FALSE);
    return status;
}

NvError
NvDdkSdioGetCommandResponse(
    NvDdkSdioDeviceHandle hSdio,
    NvU32 CommandNumber,
    NvDdkSdioRespType ResponseType,
    NvU32 *pResponse)
{
    NvU32 *pTemp;

    NV_ASSERT(hSdio);
    NV_ASSERT(pResponse);

    pTemp = pResponse;
    /* set the response type */
    switch (ResponseType)
    {
        case NvDdkSdioRespType_NoResp:
             *pTemp = 0;
             return NvSuccess;

        // SDMMC_RESP_LENGTH_48
        case NvDdkSdioRespType_R1:
        case NvDdkSdioRespType_R1b:
        case NvDdkSdioRespType_R3:
        case NvDdkSdioRespType_R4:
        case NvDdkSdioRespType_R5:
        case NvDdkSdioRespType_R6:
        case NvDdkSdioRespType_R7:
             *pTemp = SDMMC_REGR(hSdio->pSdioVirtualAddress, RESPONSE_R0_R1);
             *(++pTemp) = SDMMC_REGR(hSdio->pSdioVirtualAddress, RESPONSE_R2_R3);
             break;

        // SDMMC_RESP_LENGTH_136
        case NvDdkSdioRespType_R2:
             *pTemp = SDMMC_REGR(hSdio->pSdioVirtualAddress, RESPONSE_R0_R1);
             *(++pTemp) = SDMMC_REGR(hSdio->pSdioVirtualAddress, RESPONSE_R2_R3);
             *(++pTemp) = SDMMC_REGR(hSdio->pSdioVirtualAddress, RESPONSE_R4_R5);
             *(++pTemp) = SDMMC_REGR(hSdio->pSdioVirtualAddress, RESPONSE_R6_R7);
             break;

        default:
             NV_ASSERT(0);
    }

    return NvSuccess;
}

static NvError
PrivSdioPollingRead(
    NvDdkSdioDeviceHandle hSdio,
    NvU32 NumOfBytesToRead,
    void  *pReadBuffer,
    NvDdkSdioCommand *pRWCommand,
    NvBool HWAutoCMD12Enable,
    NvU32* SdioStatus)
{
    NvU32 val = 0;
    NvU32 PollTime = 0;
    NvU8* VirtAddr = (NvU8*)hSdio->pVirtBuffer;
    NvError Error = NvError_SdioReadFailed;
    NvU32 BytesReceived = 0;
    NvU32 BytesToBeCopied = 0;
    NvU32 BytesToBeReceived = 0;
    NvU8* ReadPtr = (NvU8*)pReadBuffer;
    NvU32 PhysAddr = hSdio->pPhysBuffer;
    NvBool IsTransferCompleted = NV_FALSE;
    NvU32 IntrStatus = 0;
    NvU32 PollingTimeOut = 0;

    BytesToBeReceived = (NumOfBytesToRead > SDMMC_DMA_TRANSFER_SIZE) ?
                  SDMMC_DMA_TRANSFER_SIZE : NumOfBytesToRead;

    PollingTimeOut = (NumOfBytesToRead > SDMMC_TIMEOUT_CALCULATION_SIZE) ?
                  (SDMMC_MIN_POLLING_TIME_USEC * (NumOfBytesToRead/
                  SDMMC_TIMEOUT_CALCULATION_SIZE)) : SDMMC_MIN_POLLING_TIME_USEC;
    if (NumOfBytesToRead > SDMMC_DMA_TRANSFER_SIZE)
    {
        ConfigureInterrupts(hSdio, 0, SDIO_INTERRUPTS,
                    (SDIO_ERROR_INTERRUPTS|NvDdkSdioCommandStatus_TransferComplete|
                                                               NvDdkSdioCommandStatus_DMA));
    }
    else
    {
        ConfigureInterrupts(hSdio, 0, SDIO_INTERRUPTS, (SDIO_ERROR_INTERRUPTS|
                                                                NvDdkSdioCommandStatus_TransferComplete));
    }
    /* this turns on the sdio clock */
    Error = SdioBlockTransfer(hSdio,
                    NumOfBytesToRead,
                    hSdio->pPhysBuffer,
                    pRWCommand,
                    HWAutoCMD12Enable,
                    NV_TRUE);
    if (Error == NvSuccess)
    {
        // poll for the transfer complete interrupt
        while (PollTime < PollingTimeOut)
        {
            IntrStatus = SDMMC_REGR(hSdio->pSdioVirtualAddress, INTERRUPT_STATUS);
            if (IntrStatus & SDMMC_ERROR_STATUS_VALUE)
            {
                // Disable the error interrupts
                ConfigureInterrupts(hSdio, 0, SDIO_ERROR_INTERRUPTS, 0);

                // check if there are any command errors
                if (IntrStatus & SDIO_CMD_ERROR_INTERRUPTS)
                {
                    // Rest the cmd line if there are any command related errors
                    val = SDMMC_REGR(hSdio->pSdioVirtualAddress,
                                        SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL);
                    val = NV_FLD_SET_DRF_DEF(SDMMC, SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL,
                                        SW_RESET_FOR_CMD_LINE, RESETED, val);
                    SDMMC_REGW(hSdio->pSdioVirtualAddress,
                                        SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL, val);
                }
                else
                {
                    // reset the dat line if there are any data transfer errors
                   val = SDMMC_REGR(hSdio->pSdioVirtualAddress,
                                    SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL);
                    val = NV_FLD_SET_DRF_DEF(SDMMC, SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL,
                                    SW_RESET_FOR_DAT_LINE, RESETED, val);
                    SDMMC_REGW(hSdio->pSdioVirtualAddress,
                                    SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL, val);
                }

                // clear the error interrupts
                IntrStatus &= SDMMC_ERROR_STATUS_VALUE;
                SDMMC_REGW(hSdio->pSdioVirtualAddress, INTERRUPT_STATUS, IntrStatus);
                *SdioStatus = IntrStatus;

                // Do the error recovery
                PrivSdioErrorRecovery(hSdio);
                Error = NvError_SdioReadFailed;
                SD_PRINT(("SDIO_DDK polling read failed error[0x%x] \
                        instance[%d]\n", *SdioStatus, hSdio->Instance));
                break;
            }
            else if (NV_DRF_VAL(SDMMC, INTERRUPT_STATUS, DMA_INTERRUPT, IntrStatus))
            {
                val = NV_DRF_DEF(SDMMC, INTERRUPT_STATUS, DMA_INTERRUPT, GEN_INT);
                // clearing the DMA interrupt
                SDMMC_REGW(hSdio->pSdioVirtualAddress, INTERRUPT_STATUS, val);
            }
            else if (NV_DRF_VAL(SDMMC, INTERRUPT_STATUS, XFER_COMPLETE, IntrStatus))
            {
                val = NV_DRF_DEF(SDMMC, INTERRUPT_STATUS, XFER_COMPLETE, GEN_INT);
                SDMMC_REGW(hSdio->pSdioVirtualAddress, INTERRUPT_STATUS, val);
                IsTransferCompleted = NV_TRUE;
            }
            else
            {
                NvOsWaitUS(SDMMC_POLLING_DELAY_USEC);
                PollTime += SDMMC_POLLING_DELAY_USEC;
                continue;
            }

            if (BytesToBeReceived)
            {
                BytesToBeCopied = BytesToBeReceived;
                BytesReceived += BytesToBeReceived;
                BytesToBeReceived = NumOfBytesToRead - BytesReceived;
            }
            if (BytesToBeReceived == 0)
            {
                if (IsTransferCompleted)
                {
                    NvOsMemcpy(ReadPtr, VirtAddr, BytesToBeCopied);
                    break;
                }
                else
                {
                    NvOsWaitUS(SDMMC_POLLING_DELAY_USEC);
                    PollTime += SDMMC_POLLING_DELAY_USEC;
                    continue;
                }
            }
            else if (BytesToBeReceived > SDMMC_DMA_TRANSFER_SIZE)
            {
                BytesToBeReceived = SDMMC_DMA_TRANSFER_SIZE;
            }

            if (PhysAddr == hSdio->pPhysBuffer)
            {
                PhysAddr += SDMMC_DMA_TRANSFER_SIZE;
                // program the system start address
                SDMMC_REGW(hSdio->pSdioVirtualAddress, SYSTEM_ADDRESS, PhysAddr);
                NvOsMemcpy(ReadPtr, VirtAddr, BytesToBeCopied);
                VirtAddr += SDMMC_DMA_TRANSFER_SIZE;
            }
            else
            {
                PhysAddr = hSdio->pPhysBuffer;
                // program the system start address
                SDMMC_REGW(hSdio->pSdioVirtualAddress, SYSTEM_ADDRESS, hSdio->pPhysBuffer);
                NvOsMemcpy(ReadPtr, VirtAddr, BytesToBeCopied);
                VirtAddr = hSdio->pVirtBuffer;
            }
            ReadPtr += BytesToBeCopied;
            NvOsWaitUS(SDMMC_POLLING_DELAY_USEC);
            PollTime += SDMMC_POLLING_DELAY_USEC;
        }
        if (PollTime >= PollingTimeOut)
        {
            PrivSdioReset(hSdio);
            Error = NvError_SdioReadFailed;
           SD_PRINT(("SDIO_DDK polling read timeout Instance[%d]\n",
                            hSdio->Instance));
        }
        SdioConfigureCardClock(hSdio, NV_FALSE);
    }
    return Error;
}

NvError
NvDdkSdioRead(
    NvDdkSdioDeviceHandle hSdio,
    NvU32 NumOfBytesToRead,
    void  *pReadBuffer,
    NvDdkSdioCommand *pRWCommand,
    NvBool HWAutoCMD12Enable,
    NvU32* SdioStatus)
{
    NvError Error = NvError_SdioReadFailed;
    NvDdkSdioStatus* ControllerStatus = NULL;
    NvU32 BytesToBeReceived = 0;
    NvU32 BytesReceived = 0;
    NvU8* ReadPtr = (NvU8*)pReadBuffer;
    NvU32 BytesToBeCopied = 0;
    NvU8* VirtAddr = (NvU8*)hSdio->pVirtBuffer;
    NvU32 PhysAddr = hSdio->pPhysBuffer;

#if !NV_OAL
#if ENABLE_READ_BUSY_HINTS
    // Enable pulse mode
    NvRmDfsBusyHint pMultiHintOff[3] = {
        {NvRmDfsClockId_Emc, 0, 0, NV_TRUE},
        {NvRmDfsClockId_System, 0, 0, NV_TRUE},
        {NvRmDfsClockId_Cpu, 0, 0, NV_TRUE} };

    NvRmDfsBusyHint pMultiHintOn[3] = {
        {NvRmDfsClockId_Emc, NV_WAIT_INFINITE, 80000, NV_TRUE},
        {NvRmDfsClockId_System, NV_WAIT_INFINITE, 80000, NV_TRUE},
        {NvRmDfsClockId_Cpu, NV_WAIT_INFINITE, 240000, NV_TRUE} };
#endif
#endif

    NV_ASSERT(hSdio);
    NV_ASSERT(pReadBuffer);
    NV_ASSERT(pRWCommand);

    *SdioStatus = NvDdkSdioError_None;
#if NV_OAL
    Error = PrivSdioPollingRead(hSdio, NumOfBytesToRead, pReadBuffer,
                        pRWCommand, HWAutoCMD12Enable, SdioStatus);
    return Error;
#endif

    // To get better performance, use polling (to eliminate interrupt latency) for lower transfer sizes.
    if (NumOfBytesToRead <= SDMMC_MAX_POLLING_SIZE)
    {
        Error = PrivSdioPollingRead(hSdio, NumOfBytesToRead, pReadBuffer,
                            pRWCommand, HWAutoCMD12Enable, SdioStatus);
        return Error;
    }

#if !NV_OAL
#if ENABLE_READ_BUSY_HINTS
    // Enable busy hints
    NV_ASSERT_SUCCESS(
    NvRmPowerBusyHintMulti(hSdio->hRm,
         hSdio->SdioRmPowerClientId,
         pMultiHintOn,
         3,
         NvRmDfsBusyHintSyncMode_Async));
#endif
#endif

    ControllerStatus = hSdio->ControllerStatus;

    BytesToBeReceived = (NumOfBytesToRead > SDMMC_DMA_TRANSFER_SIZE) ?
                          SDMMC_DMA_TRANSFER_SIZE : NumOfBytesToRead;
    if (NumOfBytesToRead == SDMMC_DMA_TRANSFER_SIZE)
    {
        ConfigureInterrupts(hSdio, SDIO_INTERRUPTS, (NvDdkSdioCommandStatus_CommandComplete|NvDdkSdioCommandStatus_DMA), 0);
    }
    else
    {
        ConfigureInterrupts(hSdio, SDIO_INTERRUPTS, NvDdkSdioCommandStatus_CommandComplete, 0);
    }

    /* this function turns on the sdio clock */
    Error = SdioBlockTransfer(hSdio,
                              NumOfBytesToRead,
                              PhysAddr,
                              pRWCommand,
                              HWAutoCMD12Enable,
                              NV_TRUE);
    if (Error == NvSuccess)
    {
        // loop till we receive all the requested bytes
        while (BytesReceived != NumOfBytesToRead)
        {
            Error = NvOsSemaphoreWaitTimeout(hSdio->PrivSdioSema, NVDDK_SDMMC_DATA_TIMEOUT_MSEC);
            if (Error == NvSuccess)
            {
                if (ControllerStatus->SDErrorStatus != NvDdkSdioError_None)
                {
#if !NV_OAL
#if ENABLE_READ_BUSY_HINTS
                    // Disable busy hints
                    NV_ASSERT_SUCCESS(
                        NvRmPowerBusyHintMulti(hSdio->hRm,
                         hSdio->SdioRmPowerClientId,
                         pMultiHintOff,
                         3,
                         NvRmDfsBusyHintSyncMode_Async));
#endif
#endif
                    *SdioStatus = ControllerStatus->SDErrorStatus;
                    if (ControllerStatus->SDErrorStatus & NvDdkSdioError_DataCRC)
                    {
                        NV_DEBUG_PRINTF(("NvDdkSdio Read Error Status: Data CRC error occured \n"));
                        PrivSdioErrorRecovery(hSdio);
                    }
                    else
                    {
                        PrivSdioReset(hSdio);
                    }
                    SdioConfigureCardClock(hSdio, NV_FALSE);
                    return Error;
                }
                else
                {
                    BytesToBeCopied = BytesToBeReceived;
                    BytesReceived += BytesToBeReceived;
                    BytesToBeReceived = NumOfBytesToRead - BytesReceived;
                    if (BytesToBeReceived == 0)
                    {
                        NvOsMemcpy(ReadPtr, VirtAddr, BytesToBeCopied);
                        break;
                    }
                    else if (BytesToBeReceived == SDMMC_DMA_TRANSFER_SIZE)
                    {
                        ConfigureInterrupts(hSdio, SDIO_INTERRUPTS, (NvDdkSdioCommandStatus_DMA|NvDdkSdioCommandStatus_CommandComplete), 0);
                    }
                    else if (BytesToBeReceived > SDMMC_DMA_TRANSFER_SIZE)
                    {
                        BytesToBeReceived = SDMMC_DMA_TRANSFER_SIZE;
                    }

                    if (PhysAddr == hSdio->pPhysBuffer)
                    {
                        PhysAddr += SDMMC_DMA_TRANSFER_SIZE;
                        // program the system start address
                        SDMMC_REGW(hSdio->pSdioVirtualAddress, SYSTEM_ADDRESS, PhysAddr);
                        NvOsMemcpy(ReadPtr, VirtAddr, BytesToBeCopied);
                        VirtAddr += SDMMC_DMA_TRANSFER_SIZE;
                    }
                    else
                    {
                        PhysAddr = hSdio->pPhysBuffer;
                        // program the system start address
                        SDMMC_REGW(hSdio->pSdioVirtualAddress, SYSTEM_ADDRESS, hSdio->pPhysBuffer);
                        NvOsMemcpy(ReadPtr, VirtAddr, BytesToBeCopied);
                        VirtAddr = hSdio->pVirtBuffer;
                    }
                    ReadPtr += BytesToBeCopied;
                }
            }
            else
            {
#if !NV_OAL
#if ENABLE_READ_BUSY_HINTS
                 // Disable busy hints
                NV_ASSERT_SUCCESS(
                    NvRmPowerBusyHintMulti(hSdio->hRm,
                     hSdio->SdioRmPowerClientId,
                     pMultiHintOff,
                     3,
                     NvRmDfsBusyHintSyncMode_Async));
#endif
#endif
                // break if there is any timeout
                *SdioStatus = ControllerStatus->SDErrorStatus;
                // !!! JN: don't leave the clock togging in case of an error.
                //         probably should do a reset or something here, not sure?
                PrivSdioReset(hSdio);
                SdioConfigureCardClock(hSdio, NV_FALSE);
                SD_PRINT(("SDIO_DDK normal read timeout Instance[%d]\n",
                                    hSdio->Instance));
                return Error;
            }
        }
    }

#if !NV_OAL
#if ENABLE_READ_BUSY_HINTS
    // Disable Multihint
    NV_ASSERT_SUCCESS(
        NvRmPowerBusyHintMulti(hSdio->hRm,
             hSdio->SdioRmPowerClientId,
             pMultiHintOff,
             3,
             NvRmDfsBusyHintSyncMode_Async));
#endif
#endif

    SdioConfigureCardClock(hSdio, NV_FALSE);

    *SdioStatus = ControllerStatus->SDErrorStatus;
    return Error;
}

static NvError
PrivSdioPollingWrite(
    NvDdkSdioDeviceHandle hSdio,
    NvU32 NumOfBytesToWrite,
    void *pWriteBuffer,
    NvDdkSdioCommand *pRWCommand,
    NvBool HWAutoCMD12Enable,
    NvU32* SdioStatus)
{
    NvU32 val = 0;
    NvU32 PollTime = 0;
    NvU8* VirtAddr = (NvU8*)hSdio->pVirtBuffer;
    NvError Error = NvError_SdioWriteFailed;
    NvU32 BytesToBeSent = 0;
    NvU32 BytesSent = 0;
    NvU8* WritePtr = (NvU8*)pWriteBuffer;
    NvU32 PhysAddr = hSdio->pPhysBuffer;
    NvBool IsTransferCompleted = NV_FALSE;
    NvU32 IntrStatus = 0;
    NvBool IsDmaStarted = NV_FALSE;
    NvU32 PollingTimeOut = 0;

    BytesToBeSent = (NumOfBytesToWrite > SDMMC_DMA_TRANSFER_SIZE) ?
                          SDMMC_DMA_TRANSFER_SIZE : NumOfBytesToWrite;

    PollingTimeOut = (NumOfBytesToWrite > SDMMC_TIMEOUT_CALCULATION_SIZE) ?
                  (SDMMC_MIN_POLLING_TIME_USEC * (NumOfBytesToWrite/
                  SDMMC_TIMEOUT_CALCULATION_SIZE)) : SDMMC_MIN_POLLING_TIME_USEC;
    if (NumOfBytesToWrite > SDMMC_DMA_TRANSFER_SIZE)
    {
        ConfigureInterrupts(hSdio, 0, SDIO_INTERRUPTS,
                    (SDIO_ERROR_INTERRUPTS|NvDdkSdioCommandStatus_TransferComplete|
                                                               NvDdkSdioCommandStatus_DMA));
    }
    else
    {
        ConfigureInterrupts(hSdio, 0, SDIO_INTERRUPTS, (SDIO_ERROR_INTERRUPTS|
                                                                NvDdkSdioCommandStatus_TransferComplete));
    }

    NvOsMemcpy(VirtAddr, pWriteBuffer, BytesToBeSent);

    /* this function enables the sdio clock */
    Error = SdioBlockTransfer(hSdio,
                    NumOfBytesToWrite,
                    hSdio->pPhysBuffer,
                    pRWCommand,
                    HWAutoCMD12Enable,
                    NV_FALSE);
    if (Error == NvSuccess)
    {
        IsDmaStarted = NV_TRUE;
        // poll for the transfer complete interrupt
        while (PollTime < PollingTimeOut)
        {
            if (IsDmaStarted)
            {
                WritePtr += BytesToBeSent;
                BytesSent += BytesToBeSent;
                BytesToBeSent = NumOfBytesToWrite - BytesSent;

                BytesToBeSent =
                    (BytesToBeSent > SDMMC_DMA_TRANSFER_SIZE) ? SDMMC_DMA_TRANSFER_SIZE : BytesToBeSent;

                if (BytesToBeSent)
                {
                    PhysAddr =
                        (PhysAddr == hSdio->pPhysBuffer) ? (PhysAddr + SDMMC_DMA_TRANSFER_SIZE) : hSdio->pPhysBuffer;

                    VirtAddr =
                        (VirtAddr == hSdio->pVirtBuffer) ? (VirtAddr + SDMMC_DMA_TRANSFER_SIZE) : hSdio->pVirtBuffer;

                    NvOsMemcpy(VirtAddr, WritePtr, BytesToBeSent);
                }
                IsDmaStarted = NV_FALSE;
            }
            IntrStatus = SDMMC_REGR(hSdio->pSdioVirtualAddress, INTERRUPT_STATUS);
            if (IntrStatus & SDMMC_ERROR_STATUS_VALUE)
            {
                // Disable the error interrupts
                ConfigureInterrupts(hSdio, 0, SDIO_ERROR_INTERRUPTS, 0);

                // check if there are any command errors
                if (IntrStatus & SDIO_CMD_ERROR_INTERRUPTS)
                {
                    // Rest the cmd line if there are any command related errors
                    val = SDMMC_REGR(hSdio->pSdioVirtualAddress,
                                        SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL);
                    val = NV_FLD_SET_DRF_DEF(SDMMC, SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL,
                                        SW_RESET_FOR_CMD_LINE, RESETED, val);
                    SDMMC_REGW(hSdio->pSdioVirtualAddress,
                                        SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL, val);
                }
                else
                {
                    // reset the dat line if there are any data transfer errors
                   val = SDMMC_REGR(hSdio->pSdioVirtualAddress,
                                    SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL);
                    val = NV_FLD_SET_DRF_DEF(SDMMC, SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL,
                                    SW_RESET_FOR_DAT_LINE, RESETED, val);
                    SDMMC_REGW(hSdio->pSdioVirtualAddress,
                                    SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL, val);
                }

                // Clear the error interrupts
                IntrStatus &= SDMMC_ERROR_STATUS_VALUE;
                SDMMC_REGW(hSdio->pSdioVirtualAddress, INTERRUPT_STATUS, IntrStatus);
                *SdioStatus = IntrStatus;

                // Do the error recovery
                PrivSdioErrorRecovery(hSdio);
                Error = NvError_SdioWriteFailed;
                SD_PRINT(("SDIO_DDK polling write failed error[0x%x] \
                            Instance[%d]\n", *SdioStatus, hSdio->Instance));
                break;
            }
        else if (NV_DRF_VAL(SDMMC, INTERRUPT_STATUS, DMA_INTERRUPT, IntrStatus))
        {
            val = NV_DRF_DEF(SDMMC, INTERRUPT_STATUS, DMA_INTERRUPT, GEN_INT);
            // clearing the DMA interrupt
            SDMMC_REGW(hSdio->pSdioVirtualAddress, INTERRUPT_STATUS, val);
        }
        else if (NV_DRF_VAL(SDMMC, INTERRUPT_STATUS, XFER_COMPLETE, IntrStatus))
        {
            val = NV_DRF_DEF(SDMMC, INTERRUPT_STATUS, XFER_COMPLETE, GEN_INT);
            SDMMC_REGW(hSdio->pSdioVirtualAddress, INTERRUPT_STATUS, val);
            IsTransferCompleted = NV_TRUE;
        }
        else
        {
            NvOsWaitUS(SDMMC_POLLING_DELAY_USEC);
            PollTime += SDMMC_POLLING_DELAY_USEC;
            continue;
        }

        if (BytesToBeSent)
        {
           SDMMC_REGW(hSdio->pSdioVirtualAddress, SYSTEM_ADDRESS, PhysAddr);
           IsDmaStarted = NV_TRUE;
        }
        else if (IsTransferCompleted)
        {
            break;
        }
        NvOsWaitUS(SDMMC_POLLING_DELAY_USEC);
        PollTime += SDMMC_POLLING_DELAY_USEC;
        }
        if (PollTime >= PollingTimeOut)
        {
            PrivSdioReset(hSdio);
            Error = NvError_SdioWriteFailed;
            SD_PRINT(("SDIO_DDK polling write timeout Instance[%d]\n",
                            hSdio->Instance));
        }
    }
    /* turn off the sdio clock */
    SdioConfigureCardClock(hSdio, NV_FALSE);
    return Error;
}

NvError
NvDdkSdioWrite(
    NvDdkSdioDeviceHandle hSdio,
    NvU32 NumOfBytesToWrite,
    void *pWriteBuffer,
    NvDdkSdioCommand *pRWCommand,
    NvBool HWAutoCMD12Enable,
    NvU32* SdioStatus)
{
    NvError Error = NvError_SdioWriteFailed;
    NvDdkSdioStatus* ControllerStatus = NULL;
    NvU32 BytesToBeSent = 0;
    NvU32 BytesSent = 0;
    NvU8* WritePtr = (NvU8*)pWriteBuffer;
    NvU8* VirtAddr = (NvU8*)hSdio->pVirtBuffer;
    NvU32 PhysAddr = hSdio->pPhysBuffer;
    NvBool IsDisableDmaInterrupt = NV_FALSE;

    // Disable pulse mode, since performance is hit if pulse mode is used in Write
    NvRmDfsBusyHint pMultiHintOff[3] = {
        {NvRmDfsClockId_Emc, 0, 0, NV_FALSE},
        {NvRmDfsClockId_System, 0, 0, NV_FALSE},
        {NvRmDfsClockId_Cpu, 0, 0, NV_FALSE} };

    NvRmDfsBusyHint pMultiHintOn[3] = {
        {NvRmDfsClockId_Emc, NV_WAIT_INFINITE, 80000, NV_FALSE},
        {NvRmDfsClockId_System, NV_WAIT_INFINITE, 80000, NV_FALSE},
        {NvRmDfsClockId_Cpu, NV_WAIT_INFINITE, 450000, NV_FALSE} };

    NV_ASSERT(hSdio);
    NV_ASSERT(pWriteBuffer);
    NV_ASSERT(pRWCommand);

    *SdioStatus = NvDdkSdioError_None;
#if NV_OAL
        Error = PrivSdioPollingWrite(hSdio, NumOfBytesToWrite, pWriteBuffer,
                            pRWCommand, HWAutoCMD12Enable, SdioStatus);
        return Error;
#endif

    // To get better performance, use polling (to eliminate interrupt latency) for lower transfer sizes.
    if (NumOfBytesToWrite <= SDMMC_MAX_POLLING_SIZE)
    {
        Error = PrivSdioPollingWrite(hSdio, NumOfBytesToWrite, pWriteBuffer,
                            pRWCommand, HWAutoCMD12Enable, SdioStatus);
        return Error;
    }

     // Enable busy hints
    NV_ASSERT_SUCCESS(
    NvRmPowerBusyHintMulti(hSdio->hRm,
         hSdio->SdioRmPowerClientId,
         pMultiHintOn,
         3,
         NvRmDfsBusyHintSyncMode_Async));

    ControllerStatus = hSdio->ControllerStatus;
    BytesToBeSent = (NumOfBytesToWrite > SDMMC_DMA_TRANSFER_SIZE) ?
                          SDMMC_DMA_TRANSFER_SIZE : NumOfBytesToWrite;

    if (NumOfBytesToWrite == SDMMC_DMA_TRANSFER_SIZE)
    {
        ConfigureInterrupts(hSdio, SDIO_INTERRUPTS, (NvDdkSdioCommandStatus_CommandComplete|NvDdkSdioCommandStatus_DMA), 0);
    }
    else
    {
        ConfigureInterrupts(hSdio, SDIO_INTERRUPTS, NvDdkSdioCommandStatus_CommandComplete, 0);
    }

    NvOsMemcpy(VirtAddr, WritePtr, BytesToBeSent);

    /* this enables the sdio clock */
    Error = SdioBlockTransfer(hSdio,
                              NumOfBytesToWrite,
                              PhysAddr,
                              pRWCommand,
                              HWAutoCMD12Enable,
                              NV_FALSE);
    if (Error == NvSuccess)
    {
        // loop till we receive all the requested bytes
        while (BytesSent != NumOfBytesToWrite)
        {
            WritePtr += BytesToBeSent;
            BytesSent += BytesToBeSent;
            BytesToBeSent = NumOfBytesToWrite - BytesSent;
            if (BytesToBeSent == 0)
            {
                Error = NvOsSemaphoreWaitTimeout(hSdio->PrivSdioSema,
                                                                    NVDDK_SDMMC_DATA_TIMEOUT_MSEC);
                break;
            }
            else if (BytesToBeSent == SDMMC_DMA_TRANSFER_SIZE)
            {
                IsDisableDmaInterrupt = NV_TRUE;
            }
            else if (BytesToBeSent > SDMMC_DMA_TRANSFER_SIZE)
            {
                BytesToBeSent = SDMMC_DMA_TRANSFER_SIZE;
            }

            if (PhysAddr == hSdio->pPhysBuffer)
            {
                PhysAddr += SDMMC_DMA_TRANSFER_SIZE;
                VirtAddr += SDMMC_DMA_TRANSFER_SIZE;
            }
            else
            {
                 PhysAddr = hSdio->pPhysBuffer;
                 VirtAddr = hSdio->pVirtBuffer;
            }
            NvOsMemcpy(VirtAddr, WritePtr, BytesToBeSent);
            Error = NvOsSemaphoreWaitTimeout(hSdio->PrivSdioSema,
                                                                    NVDDK_SDMMC_DATA_TIMEOUT_MSEC);

            // issue abort incase of error
            if (ControllerStatus->SDErrorStatus != NvDdkSdioError_None)
            {
                 // Disable busy hints
                NV_ASSERT_SUCCESS(
                    NvRmPowerBusyHintMulti(hSdio->hRm,
                     hSdio->SdioRmPowerClientId,
                     pMultiHintOff,
                     3,
                     NvRmDfsBusyHintSyncMode_Async));
                *SdioStatus = ControllerStatus->SDErrorStatus;
                if (ControllerStatus->SDErrorStatus & NvDdkSdioError_DataCRC)
                {
                    NV_DEBUG_PRINTF(("NvDdkSdio Write Error Status: Data CRC error occured \n"));
                    PrivSdioErrorRecovery(hSdio);
                }
                else
                {
                    PrivSdioReset(hSdio);
                }
                // !!! JN: don't leave the clock toggling if there is an error
                SdioConfigureCardClock(hSdio, NV_FALSE);
                SD_PRINT(("SDIO_DDK normal write failed error[0x%x] \
                            Instance[%d]\n", *SdioStatus, hSdio->Instance));
                return Error;
            }
            else
            {
                if (IsDisableDmaInterrupt)
                {
                    ConfigureInterrupts(hSdio, SDIO_INTERRUPTS, (NvDdkSdioCommandStatus_DMA|NvDdkSdioCommandStatus_CommandComplete), 0);
                }
               SDMMC_REGW(hSdio->pSdioVirtualAddress, SYSTEM_ADDRESS, PhysAddr);
            }
        }
    }

    // Disable busy hints
    NV_ASSERT_SUCCESS(
        NvRmPowerBusyHintMulti(hSdio->hRm,
         hSdio->SdioRmPowerClientId,
         pMultiHintOff,
         3,
         NvRmDfsBusyHintSyncMode_Async));
    SdioConfigureCardClock(hSdio, NV_FALSE);
    *SdioStatus = ControllerStatus->SDErrorStatus;
    return Error;
}

void PrivSdioErrorRecovery(NvDdkSdioDeviceHandle hSdio)
{

    NvU32 val = 0;

    // Issue soft reset
    PrivSdioReset(hSdio);

    // For read commands disable AutoCMD12
    if (hSdio->IsRead && (!hSdio->IsSdControllerVersion2))
    {
        val = SDMMC_REGR(hSdio->pSdioVirtualAddress, CMD_XFER_MODE);
        val = NV_FLD_SET_DRF_DEF(SDMMC, CMD_XFER_MODE, AUTO_CMD12_EN, DISABLE, val);
        SDMMC_REGW(hSdio->pSdioVirtualAddress, CMD_XFER_MODE, val);
    }

    // Issue abort
    PrivSdioAbort(hSdio);

    if (!hSdio->IsSdControllerVersion2)
    {
        NvOsWaitUS(NVDDK_SDMMC_DELAY_AFTER_ABORT_USEC);

        // Issue soft reset
        PrivSdioReset(hSdio);
    }

}

NvError
SdioGetPhysAdd(
    NvRmDeviceHandle hRmDevice,
    NvRmMemHandle* hRmMemHandle,
    void** pVirtBuffer,
    NvU32 size,
    NvU32* pPhysBuffer)
{
    NvU32 SDMMC_ALIGNMENT_SIZE = SDMMC_DMA_TRANSFER_SIZE;
    NvError Error = NvError_InvalidAddress;

    // Initialise the handle to NULL
    *pPhysBuffer = 0;
    *hRmMemHandle = NULL;

    // Create the Memory Handle
    Error = NvRmMemHandleCreate(hRmDevice, hRmMemHandle, size);
    if (Error != NvSuccess)
    {
        return Error;
    }

    // Allocate the memory
    Error = NvRmMemAlloc(*hRmMemHandle, NULL, 0,
                         SDMMC_ALIGNMENT_SIZE, NvOsMemAttribute_Uncached);
    if (Error != NvSuccess)
    {
        NvRmMemHandleFree(*hRmMemHandle);
        return Error;
    }

    // Pin the memory and Get Physical Address
    *pPhysBuffer = NvRmMemPin(*hRmMemHandle);

    Error = NvRmMemMap(*hRmMemHandle, 0, size, NVOS_MEM_READ_WRITE, pVirtBuffer);
    if (Error != NvSuccess)
    {
        NvRmMemUnpin(*hRmMemHandle);
        NvRmMemHandleFree(*hRmMemHandle);
        return Error;
    }

    return NvSuccess;
}

NvError
SdioBlockTransfer(
    NvDdkSdioDeviceHandle hSdio,
    NvU32 NumOfRWBytes,
    NvU32 pReadBuffer,
    NvDdkSdioCommand *pRWRequest,
    NvBool HWAutoCMD12Enable,
    NvBool IsRead)
{
    NvBool IsCrcCheckEnable = NV_FALSE;
    NvBool IsIndexCheckEnable = NV_FALSE;
    NvU32 IsControllerBusy = 0;
    NvBool IsDataCommand = NV_TRUE;
    NvU32 NumOfBlocks = 0;
    NvU32 CmdXferReg = 0;
    NvU32 BlkSizeCountReg = 0;
    NvU32 SdioBlockSize = 512;
    NvDdkSdioStatus* ControllerStatus = NULL;

    if (pRWRequest->BlockSize > hSdio->MaxBlockLength)
    {
         return NvError_SdioBadBlockSize;
    }

    ControllerStatus = hSdio->ControllerStatus;
    // clear the previous status if any
    ControllerStatus->SDControllerStatus = NvDdkSdioCommandStatus_None;
    ControllerStatus->SDErrorStatus = NvDdkSdioError_None;
    hSdio->RequestType = NVDDK_SDIO_NORMAL_REQUEST;

    hSdio->IsRead = IsRead;
    SdioBlockSize = pRWRequest->BlockSize;
    NumOfBlocks = NumOfRWBytes/SdioBlockSize;

    if (NumOfBlocks > SDMMC_MAX_NUMBER_OF_BLOCKS)
        return NvError_InvalidSize;
    // set DMA (system) address
    SDMMC_REGW(hSdio->pSdioVirtualAddress, SYSTEM_ADDRESS, pReadBuffer);

    if (NumOfBlocks > 1)
    {
        // Enable multi block in Cmd xfer register
        CmdXferReg |= NV_DRF_DEF(SDMMC, CMD_XFER_MODE, MULTI_BLOCK_SELECT, ENABLE);

        // Enable Block Count (used in case of multi-block transfers)
        CmdXferReg |= NV_DRF_DEF(SDMMC, CMD_XFER_MODE, BLOCK_COUNT_EN, ENABLE);

        // set Auto CMD12 (stop transmission)
        CmdXferReg |=
            NV_DRF_NUM(SDMMC, CMD_XFER_MODE, AUTO_CMD12_EN, HWAutoCMD12Enable);
    }

    if ((SdioBlockSize >= 1) && (SdioBlockSize < 4096))
    {
        BlkSizeCountReg |=
            NV_DRF_NUM(SDMMC, BLOCK_SIZE_BLOCK_COUNT, XFER_BLOCK_SIZE_12,
                                                    NV_FALSE);
    }
    else
    {
        NV_ASSERT(SdioBlockSize && (SdioBlockSize < 8192));
        // Transfer Block Size 12th bit. This bit is added to support 4Kb Data block transfer
        BlkSizeCountReg |=
            NV_DRF_NUM(SDMMC, BLOCK_SIZE_BLOCK_COUNT, XFER_BLOCK_SIZE_12,
                                                    NV_TRUE);
    }

    BlkSizeCountReg |= NV_DRF_NUM(SDMMC, BLOCK_SIZE_BLOCK_COUNT, XFER_BLOCK_SIZE_11_0,
                                                   SdioBlockSize);

    // set number of blocks to be read/written (block count)
    BlkSizeCountReg |=
        NV_DRF_NUM(SDMMC, BLOCK_SIZE_BLOCK_COUNT, BLOCKS_COUNT, NumOfBlocks);

    BlkSizeCountReg |= NV_DRF_NUM(SDMMC, BLOCK_SIZE_BLOCK_COUNT,
                                        HOST_DMA_BUFFER_SIZE, SDMMC_DMA_BUFFER_SIZE);

    SDMMC_REGW(hSdio->pSdioVirtualAddress, BLOCK_SIZE_BLOCK_COUNT, BlkSizeCountReg);

    // set the command number
    CmdXferReg |= NV_DRF_NUM(SDMMC, CMD_XFER_MODE, COMMAND_INDEX,
                             pRWRequest->CommandCode);

    // set the data command bit
    CmdXferReg |= NV_DRF_NUM(SDMMC, CMD_XFER_MODE, DATA_PRESENT_SELECT,
                             IsDataCommand);

    // set the transfer direction
    CmdXferReg |= NV_DRF_NUM(SDMMC, CMD_XFER_MODE, DATA_XFER_DIR_SEL, IsRead);

    // enable DMA
    CmdXferReg |= NV_DRF_DEF(SDMMC, CMD_XFER_MODE, DMA_EN, ENABLE);

    // set the cmd type
    CmdXferReg |=
        NV_DRF_NUM(SDMMC, CMD_XFER_MODE, COMMAND_TYPE, pRWRequest->CommandType);

    /* set the response type */
    switch (pRWRequest->ResponseType)
    {
        case NvDdkSdioRespType_NoResp:
             IsCrcCheckEnable = NV_FALSE;
             IsIndexCheckEnable = NV_FALSE;
             CmdXferReg |= NV_DRF_DEF(SDMMC, CMD_XFER_MODE, RESP_TYPE_SELECT,
                                      NO_RESPONSE);
            break;

        case NvDdkSdioRespType_R1:
        case NvDdkSdioRespType_R5:
        case NvDdkSdioRespType_R6:
             IsCrcCheckEnable = NV_TRUE;
             IsIndexCheckEnable = NV_TRUE;
             CmdXferReg |= NV_DRF_DEF(SDMMC, CMD_XFER_MODE, RESP_TYPE_SELECT,
                                      RESP_LENGTH_48);
             break;

        case NvDdkSdioRespType_R2:
             IsCrcCheckEnable = NV_TRUE;
             IsIndexCheckEnable = NV_FALSE;
             CmdXferReg |= NV_DRF_DEF(SDMMC, CMD_XFER_MODE, RESP_TYPE_SELECT,
                                      RESP_LENGTH_136);
             break;

        case NvDdkSdioRespType_R3:
        case NvDdkSdioRespType_R4:
             IsCrcCheckEnable = NV_FALSE;
             IsIndexCheckEnable = NV_FALSE;
             CmdXferReg |= NV_DRF_DEF(SDMMC, CMD_XFER_MODE, RESP_TYPE_SELECT,
                                      RESP_LENGTH_48);
             break;

        case NvDdkSdioRespType_R1b:
             IsCrcCheckEnable = NV_TRUE;
             IsIndexCheckEnable = NV_TRUE;
             CmdXferReg |= NV_DRF_DEF(SDMMC, CMD_XFER_MODE, RESP_TYPE_SELECT,
                                      RESP_LENGTH_48BUSY);
             break;

        case NvDdkSdioRespType_R7:
             IsCrcCheckEnable = NV_TRUE;
             IsIndexCheckEnable = NV_FALSE;
             CmdXferReg |= NV_DRF_DEF(SDMMC, CMD_XFER_MODE, RESP_TYPE_SELECT,
                                      RESP_LENGTH_48);
             break;

        default:
             NV_ASSERT(0);
    }

    // set the is indexed
    CmdXferReg |= NV_DRF_NUM(SDMMC, CMD_XFER_MODE, CMD_INDEX_CHECK_EN,
                             IsIndexCheckEnable);

    // set the is crc
    CmdXferReg |= NV_DRF_NUM(SDMMC, CMD_XFER_MODE, CMD_CRC_CHECK_EN,
                             IsCrcCheckEnable);

    IsControllerBusy = SdioIsControllerBusy(hSdio, IsDataCommand);
    if (IsControllerBusy)
    {
        NV_DEBUG_PRINTF(("controller is busy\n"));
        return NvError_SdioControllerBusy;
    }

    ControllerStatus->SDControllerStatus = NvDdkSdioCommandStatus_None;
    ControllerStatus->SDErrorStatus = NvDdkSdioError_None;

    /* now write to the command argument register */
    SDMMC_REGW(hSdio->pSdioVirtualAddress, ARGUMENT, pRWRequest->CmdArgument);

    SdioConfigureCardClock(hSdio, NV_TRUE);

    /* now write to the command xfer register */
    SDMMC_REGW(hSdio->pSdioVirtualAddress, CMD_XFER_MODE, CmdXferReg);

    return NvSuccess;
}

void
PrivSdioGetCaps(
    NvRmDeviceHandle hDevice,
    NvDdkSdioHostCapabilities *pHostCap,
    NvU32 instance)
{
    static NvDdkSdioHostCapabilities s_SdioCap[2];
    static NvDdkSdioHostCapabilities* s_pSdioCap = NULL;

    static NvRmModuleCapability s_SdioCaps[] =
        {
            {1, 0, 0, &s_SdioCap[0]},
            {2, 0, 0, &s_SdioCap[1]},
        };
    NV_ASSERT(hDevice);
    NV_ASSERT(pHostCap);

    s_SdioCap[0].MaxInstances = NvRmModuleGetNumInstances(hDevice, NvRmModuleID_Sdio);
    s_SdioCap[0].IsAutoCMD12Supported = NV_TRUE;
    s_SdioCap[0].IsSdControllerVersion2 = NV_FALSE;
    s_SdioCap[1].MaxInstances = NvRmModuleGetNumInstances(hDevice, NvRmModuleID_Sdio);
    s_SdioCap[1].IsAutoCMD12Supported = NV_TRUE;
    s_SdioCap[1].IsSdControllerVersion2 = NV_TRUE;

    NV_ASSERT_SUCCESS(NvRmModuleGetCapabilities(hDevice,
                      NVRM_MODULE_ID(NvRmModuleID_Sdio, instance),
                      s_SdioCaps, NV_ARRAY_SIZE(s_SdioCaps), (void**)&s_pSdioCap));

    // Fill the client capabilities structure.
    NvOsMemcpy(pHostCap, s_pSdioCap, sizeof(NvDdkSdioHostCapabilities));
}

NvError
NvDdkSdioGetCapabilities(
    NvRmDeviceHandle hDevice,
    NvDdkSdioHostCapabilities *pHostCap,
    NvDdkSdioInterfaceCapabilities *pInterfaceCap,
    NvU32 instance)
{
    NvRmModuleSdmmcInterfaceCaps SdioCaps;
    NvError Error = NvSuccess;
    const NvOdmQuerySdioInterfaceProperty* pSdioInterfaceCaps = NULL;

    PrivSdioGetCaps(hDevice, pHostCap, instance);

    Error = NvRmGetModuleInterfaceCapabilities(
                                                hDevice,
                                                NVRM_MODULE_ID(NvRmModuleID_Sdio, instance),
                                                sizeof(NvRmModuleSdmmcInterfaceCaps),
                                                &SdioCaps);

    if (Error != NvSuccess)
        return Error;

    pInterfaceCap->SDIOCardSettlingDelayMSec = 0;
    pSdioInterfaceCaps =  NvOdmQueryGetSdioInterfaceProperty(instance);
    if (pSdioInterfaceCaps)
    {
        pInterfaceCap->SDIOCardSettlingDelayMSec =  pSdioInterfaceCaps->SDIOCardSettlingDelayMSec;
        pInterfaceCap->MmcInterfaceWidth = SdioCaps.MmcInterfaceWidth;
        pHostCap->AlwaysON = pSdioInterfaceCaps->AlwaysON;
    }

    return NvSuccess;
}

NvError
NvDdkSdioSetHostBusWidth(
    NvDdkSdioDeviceHandle hSdio,
    NvDdkSdioDataWidth CardDataWidth)
{
    NvU32 DataWidthReg = 0;
    NvRmModuleSdmmcInterfaceCaps SdioCaps;

    NV_ASSERT(hSdio);

    // set the buswidth
    if (CardDataWidth != NvDdkSdioDataWidth_8Bit)
    {
        DataWidthReg = SDMMC_REGR(hSdio->pSdioVirtualAddress, POWER_CONTROL_HOST);
        DataWidthReg = NV_FLD_SET_DRF_NUM(SDMMC, POWER_CONTROL_HOST, DATA_XFER_WIDTH,
                                 CardDataWidth, DataWidthReg);
        DataWidthReg = NV_FLD_SET_DRF_DEF(SDMMC, POWER_CONTROL_HOST,
                            EXTENDED_DATA_TRANSFER_WIDTH, NOBIT_8, DataWidthReg);
    }
    else
    {
        NV_ASSERT_SUCCESS(NvRmGetModuleInterfaceCapabilities(
                hSdio->hRm,
                NVRM_MODULE_ID(NvRmModuleID_Sdio, hSdio->Instance),
                sizeof(NvRmModuleSdmmcInterfaceCaps),
                &SdioCaps));
        // check if 8bit mode is supported
        if (SdioCaps.MmcInterfaceWidth != 8)
        {
            return NvError_NotSupported;
        }

        DataWidthReg = SDMMC_REGR(hSdio->pSdioVirtualAddress, POWER_CONTROL_HOST);
        DataWidthReg = NV_FLD_SET_DRF_NUM(SDMMC, POWER_CONTROL_HOST, DATA_XFER_WIDTH,
                                 0, DataWidthReg);
        DataWidthReg = NV_FLD_SET_DRF_DEF(SDMMC, POWER_CONTROL_HOST,
                                EXTENDED_DATA_TRANSFER_WIDTH, BIT_8, DataWidthReg);
    }


    // now write to the power control host register
    SDMMC_REGW(hSdio->pSdioVirtualAddress, POWER_CONTROL_HOST, DataWidthReg);

    hSdio->BusWidth = CardDataWidth;

    return NvSuccess;
}

NvError NvDdkSdioHighSpeedEnable(NvDdkSdioDeviceHandle hSdio)
{
    NvU32 val = 0;

    NV_ASSERT(hSdio);

    val = SDMMC_REGR(hSdio->pSdioVirtualAddress, CAPABILITIES);
    val = NV_DRF_VAL(SDMMC, CAPABILITIES, HIGH_SPEED_SUPPORT, val);
    if (val)
    {
        // set the high speed enable
        val = SDMMC_REGR(hSdio->pSdioVirtualAddress, POWER_CONTROL_HOST);
        val = NV_FLD_SET_DRF_DEF(SDMMC, POWER_CONTROL_HOST, HIGH_SPEED_EN,
                                    HIGH_SPEED, val);
        SDMMC_REGW(hSdio->pSdioVirtualAddress, POWER_CONTROL_HOST, val);
    }

    return NvSuccess;
}

NvError NvDdkSdioHighSpeedDisable(NvDdkSdioDeviceHandle hSdio)
{
    NvU32 val = 0;

    NV_ASSERT(hSdio);

    // disable high speed mode
    val = SDMMC_REGR(hSdio->pSdioVirtualAddress, POWER_CONTROL_HOST);
    val = NV_FLD_SET_DRF_DEF(SDMMC, POWER_CONTROL_HOST, HIGH_SPEED_EN,
                            NORMAL_SPEED, val);
    SDMMC_REGW(hSdio->pSdioVirtualAddress, POWER_CONTROL_HOST, val);

    return NvSuccess;
}

void NvDdkSdioResetController(NvDdkSdioDeviceHandle hSdio)
{
    NvU32 val = 0;

    NV_ASSERT(hSdio);

    // reset the controller write (1) to the reset_all field.
    val = SDMMC_REGR(hSdio->pSdioVirtualAddress, SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL);
    val = NV_FLD_SET_DRF_DEF(SDMMC, SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL,
                             SW_RESET_FOR_ALL, RESETED, val);
    SDMMC_REGW(hSdio->pSdioVirtualAddress, SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL, val);
}

void NvDdkSdioClose(NvDdkSdioDeviceHandle hSdio)
{
    if (NULL != hSdio)
    {
        NV_ASSERT_SUCCESS(NvRmSetModuleTristate(hSdio->hRm,
            NVRM_MODULE_ID(NvRmModuleID_Sdio, hSdio->Instance), NV_TRUE));

        /* disable power */
        NV_ASSERT_SUCCESS(NvRmPowerVoltageControl(hSdio->hRm,
            NVRM_MODULE_ID(NvRmModuleID_Sdio, hSdio->Instance),
            hSdio->SdioRmPowerClientId,
            NvRmVoltsOff,
            NvRmVoltsOff,
            NULL,
            0,
            NULL));
#if !NV_OAL
        NvRmMemUnmap(hSdio->hRmMemHandle, hSdio->pVirtBuffer, SDMMC_MEMORY_BUFFER_SIZE);
#endif
        NvRmMemUnpin(hSdio->hRmMemHandle);
        NvRmMemHandleFree(hSdio->hRmMemHandle);

        // disable clock to sdio controller
        NvRmPowerModuleClockControl(hSdio->hRm,
            NVRM_MODULE_ID(NvRmModuleID_Sdio, hSdio->Instance),
            hSdio->SdioRmPowerClientId,
            NV_FALSE);

        // unregister with the power manager
        NvRmPowerUnRegister(hSdio->hRm, hSdio->SdioRmPowerClientId);

        // unregister/disable interrupt handler
        NvRmInterruptUnregister(hSdio->hRm, hSdio->InterruptHandle);
        hSdio->InterruptHandle = NULL;

#if !NV_OAL
        // unregister the gpio interrupt handler
        NvRmGpioInterruptUnregister(hSdio->hGpio, hSdio->hRm, hSdio->GpioIntrHandle);
#endif
        // destory the internal sdio abort semaphore
        NvOsSemaphoreDestroy(hSdio->PrivSdioSema);
        hSdio->PrivSdioSema = NULL;

        // Unmap the sdio register virtual address space
        NvRmPhysicalMemUnmap(hSdio->pSdioVirtualAddress,
                             hSdio->SdioBankSize);
#if !NV_OAL
        NvRmGpioConfigPins(hSdio->hGpio, &hSdio->CardDetectPin, 1,
                           NvRmGpioPinMode_Inactive);
#endif
        if (hSdio->SdioOdmHandle)
        {
            NvOdmSdioClose(hSdio->SdioOdmHandle);
        }
        NvOsIntrMutexDestroy(hSdio->SdioThreadSafetyMutex);
        hSdio->SdioThreadSafetyMutex = NULL;
        NvOsFree(hSdio->ControllerStatus);
        hSdio->ControllerStatus = NULL;
        NvOsSemaphoreDestroy(hSdio->SdioPowerMgtSema);
        hSdio->SdioPowerMgtSema = NULL;
        NvOsFree(hSdio);
        hSdio = NULL;
  }
}

NvError
NvDdkSdioIsCardInserted(
        NvDdkSdioDeviceHandle hSdio,
        NvBool *IscardInserted)
{
#if NV_OAL
    *IscardInserted = NV_TRUE;
    return NvError_SdioCardAlwaysPresent;
#else
    NvRmGpioPinState val = NvRmGpioPinState_Low;
    NvU32 PinCount = 0;
    const NvOdmGpioPinInfo *GpioPinInfo;
    const NvOdmQuerySdioInterfaceProperty* hOdmSdioInterface = NULL;

    GpioPinInfo = NvOdmQueryGpioPinMap(NvOdmGpioPinGroup_Sdio, hSdio->Instance, &PinCount);
    if (GpioPinInfo)
    {
        NvRmGpioReadPins(hSdio->hGpio, &(hSdio->CardDetectPin), &val, 1);
        if (val == GpioPinInfo[0].activeState)
            *IscardInserted = NV_TRUE;
        else
            *IscardInserted = NV_FALSE;
        return NvSuccess;
    }
    else
    {
        *IscardInserted = NV_FALSE;
        hOdmSdioInterface = NvOdmQueryGetSdioInterfaceProperty(hSdio->Instance);
        if (hOdmSdioInterface)
        {
            if (!hOdmSdioInterface->IsCardRemovable)
            {
                *IscardInserted = NV_TRUE;
                return NvError_SdioCardAlwaysPresent;
            }
        }
        return NvError_SdioAutoDetectCard;
    }
#endif
}

NvError
NvDdkSdioIsWriteProtected(
        NvDdkSdioDeviceHandle hSdio,
        NvRmGpioHandle hGpio,
        NvBool *IsWriteprotected)
{
#if !NV_OAL
    NvRmGpioPinState val = NvRmGpioPinState_Low;
    NvU32 PinCount = 0;
    const NvOdmGpioPinInfo *GpioPinInfo;
    NvBool IsCardInserted = NV_FALSE;

    NV_ASSERT(hGpio);
    NV_ASSERT(IsWriteprotected);

    GpioPinInfo = NvOdmQueryGpioPinMap(NvOdmGpioPinGroup_Sdio, hSdio->Instance, &PinCount);
    if (GpioPinInfo)
    {
        // Check whether the write protect gpio pin is supported or not
        if (PinCount >= 2)
        {
            if (NvDdkSdioIsCardInserted(hSdio, &IsCardInserted))
            {
                return NvError_SdioCardNotPresent;
            }
            NvRmGpioReadPins(hGpio, &(hSdio->WriteProtectPin), &val, 1);
            if (val == GpioPinInfo[1].activeState)
                *IsWriteprotected = NV_TRUE;
            else
                *IsWriteprotected = NV_FALSE;
            return NvSuccess;
        }
    }
#endif
        return NvError_NotSupported;
}

NvError
SdioRegisterInterrupts(
    NvRmDeviceHandle hRm,
    NvDdkSdioDeviceHandle hSdio)
{
    NvU32 IrqList;
    NvOsInterruptHandler IntHandlers;
    if (hSdio->InterruptHandle)
    {
        return NvSuccess;
    }
    IrqList = NvRmGetIrqForLogicalInterrupt(
        hRm, NVRM_MODULE_ID(NvRmModuleID_Sdio, hSdio->Instance), 0);
    IntHandlers = SdioIsr;
    return NvRmInterruptRegister(hRm, 1, &IrqList, &IntHandlers, hSdio,
            &hSdio->InterruptHandle, NV_TRUE);
}

static void SdioIsr(void* args)
{
    NvDdkSdioDeviceHandle hSdio;
    volatile NvU32 InterruptStatus = 0;
    volatile NvU32 val = 0;
    NvDdkSdioStatus* ControllerStatus = NULL;
    NvBool IsClkStable = NV_FALSE;

    hSdio = args;

    if (hSdio->ISControllerSuspended && hSdio->IsSdControllerVersion2)
    {
        // enable clock to sdio controller
        NvRmPowerModuleClockControl(hSdio->hRm,
            NVRM_MODULE_ID(NvRmModuleID_Sdio, hSdio->Instance),
            hSdio->SdioRmPowerClientId,
            NV_TRUE);

        // enable internal clock to sdio
        IsClkStable = SdioEnableInternalClock(hSdio);
        if (!IsClkStable)
        {
            NV_ASSERT(!"Sdio controller internal clock not stable");
            SD_PRINT(("Sdio controller internal clock not stable\n"));
        }

        // Enable SDMMC_CLK bit in VENDOR_CLOCK_CONTROL register
        val = SDMMC_REGR(hSdio->pSdioVirtualAddress, VENDOR_CLOCK_CNTRL);
        val = NV_FLD_SET_DRF_DEF(SDMMC, VENDOR_CLOCK_CNTRL,
                                    SDMMC_CLK, ENABLE, val);
        SDMMC_REGW(hSdio->pSdioVirtualAddress, VENDOR_CLOCK_CNTRL, val);
    }

    // update the Controller status
    InterruptStatus = SDMMC_REGR(hSdio->pSdioVirtualAddress, INTERRUPT_STATUS);

    ControllerStatus = hSdio->ControllerStatus;
    if (NV_DRF_VAL(SDMMC, INTERRUPT_STATUS, CARD_INTERRUPT, InterruptStatus))
    {
        if (hSdio->IsAcceptCardEvents == NV_TRUE)
        {
             // Disable the card interrupt
            NvDdkSdioEnableIoMode(hSdio, NV_FALSE);
            ControllerStatus->SDControllerStatus |= NvDdkSdioCommandStatus_Card;
            NvOsSemaphoreSignal(hSdio->NotificationSema);
        }
        else
        {
             // Disable the card interrupt
            NvDdkSdioEnableIoMode(hSdio, NV_FALSE);
            ControllerStatus->SDControllerStatus |= NvDdkSdioCommandStatus_Card;
        }
    }
    else if (NV_DRF_VAL(SDMMC, INTERRUPT_STATUS, BLOCK_GAP_EVENT, InterruptStatus))
    {
        // read transaction
        if (hSdio->IsRead == NV_TRUE)
        {
            val = NV_DRF_DEF(SDMMC, INTERRUPT_STATUS, XFER_COMPLETE, GEN_INT);
            val |= NV_DRF_DEF(SDMMC, INTERRUPT_STATUS, BLOCK_GAP_EVENT, GEN_INT);

             // Disable card clock
            SdioConfigureCardClock(hSdio, NV_FALSE);
            NvOsWaitUS(SDMMC_ABORT_DELAY_USEC);
            // Clearblock gap event and error interrupts if any
            SDMMC_REGW(hSdio->pSdioVirtualAddress, INTERRUPT_STATUS,
                                ((InterruptStatus & SDMMC_ERROR_STATUS_VALUE) | val));
        }
        // write transaction
        else if (NV_DRF_VAL(SDMMC, INTERRUPT_STATUS, XFER_COMPLETE, InterruptStatus))
        {
            // transfer complete interrupt
            val = NV_DRF_DEF(SDMMC, INTERRUPT_STATUS, XFER_COMPLETE, GEN_INT);
            val |= NV_DRF_DEF(SDMMC, INTERRUPT_STATUS, BLOCK_GAP_EVENT, GEN_INT);

            // Disable card clock
            SdioConfigureCardClock(hSdio, NV_FALSE);
            // Clear interrupts
            SDMMC_REGW(hSdio->pSdioVirtualAddress, INTERRUPT_STATUS, val);
        }
        NvOsSemaphoreSignal(hSdio->PrivSdioSema);
    }
    else  if (NV_DRF_VAL(SDMMC, INTERRUPT_STATUS, ERR_INTERRUPT, InterruptStatus))
    {
        // Disable the error interrupts
        ConfigureInterrupts(hSdio, 0, SDIO_ERROR_INTERRUPTS, 0);
        ControllerStatus->SDErrorStatus = InterruptStatus & SDIO_ERROR_INTERRUPTS;

        if (ControllerStatus->SDErrorStatus & (NvDdkSdioError_CommandTimeout |
                                               NvDdkSdioError_CommandCRC |
                                               NvDdkSdioError_CommandEndBit |
                                               NvDdkSdioError_CommandIndex))
        {
            val = SDMMC_REGR(hSdio->pSdioVirtualAddress,
                                SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL);
            val = NV_FLD_SET_DRF_DEF(SDMMC, SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL,
                                SW_RESET_FOR_CMD_LINE, RESETED, val);
            SDMMC_REGW(hSdio->pSdioVirtualAddress,
                                SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL, val);
        }
        if (ControllerStatus->SDErrorStatus & (NvDdkSdioError_DataTimeout |
                                               NvDdkSdioError_DataCRC |
                                               NvDdkSdioError_DataEndBit))
        {
           val = SDMMC_REGR(hSdio->pSdioVirtualAddress,
                            SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL);
            val = NV_FLD_SET_DRF_DEF(SDMMC, SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL,
                            SW_RESET_FOR_DAT_LINE, RESETED, val);
            SDMMC_REGW(hSdio->pSdioVirtualAddress,
                            SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL, val);
        }

        // Clear the error interrupt
        SDMMC_REGW(hSdio->pSdioVirtualAddress, INTERRUPT_STATUS,
                                (InterruptStatus & SDMMC_ERROR_STATUS_VALUE));
        NvOsSemaphoreSignal(hSdio->PrivSdioSema);
    }
    else
    {
        val = 0;
        if (NV_DRF_VAL(SDMMC, INTERRUPT_STATUS, CMD_COMPLETE, InterruptStatus))
        {
             SdioConfigureCardClock(hSdio, NV_FALSE);
             // command complete interrupt
            val = NV_DRF_DEF(SDMMC, INTERRUPT_STATUS, CMD_COMPLETE, GEN_INT);
            ControllerStatus->SDControllerStatus |=
                                NvDdkSdioCommandStatus_CommandComplete;

            // Clear the source of the interrupt
            SDMMC_REGW(hSdio->pSdioVirtualAddress, INTERRUPT_STATUS, val);
            NvOsSemaphoreSignal(hSdio->PrivSdioSema);
        }
        else if (NV_DRF_VAL(SDMMC, INTERRUPT_STATUS, DMA_INTERRUPT, InterruptStatus))
        {
            val = NV_DRF_DEF(SDMMC, INTERRUPT_STATUS, DMA_INTERRUPT, GEN_INT);
            ControllerStatus->SDControllerStatus |= NvDdkSdioCommandStatus_DMA;
            // clearing the DMA interrupt
            SDMMC_REGW(hSdio->pSdioVirtualAddress, INTERRUPT_STATUS, val);
            NvOsSemaphoreSignal(hSdio->PrivSdioSema);
        }
        else if (NV_DRF_VAL(SDMMC, INTERRUPT_STATUS, XFER_COMPLETE, InterruptStatus))
        {
            SdioConfigureCardClock(hSdio, NV_FALSE);
             // transfer complete interrupt
            val = NV_DRF_DEF(SDMMC, INTERRUPT_STATUS, XFER_COMPLETE, GEN_INT);
            ControllerStatus->SDControllerStatus |=
                                NvDdkSdioCommandStatus_TransferComplete;
            // Clear the source of the interrupt
            SDMMC_REGW(hSdio->pSdioVirtualAddress, INTERRUPT_STATUS, val);
            NvOsSemaphoreSignal(hSdio->PrivSdioSema);
        }
        else
        {
            SD_PRINT(("Interrupt Status 0x%x Instance[%d]\n",
                    InterruptStatus, hSdio->Instance));
            //NV_ASSERT(!"SDIO_DDK Invalid Interrupt");
        }
    }

    NvRmInterruptDone(hSdio->InterruptHandle);
}

NvError
NvDdkSdioSetSDBusVoltage(
    NvDdkSdioDeviceHandle hSdio,
    NvDdkSdioSDBusVoltage Voltage)
{
    NvU32 val = 0;

    NV_ASSERT(hSdio);

    val = SDMMC_REGR(hSdio->pSdioVirtualAddress, POWER_CONTROL_HOST);
    val |= NV_DRF_NUM(SDMMC, POWER_CONTROL_HOST, SD_BUS_VOLTAGE_SELECT, Voltage);
    SDMMC_REGW(hSdio->pSdioVirtualAddress, POWER_CONTROL_HOST, val);
    hSdio->BusVoltage = Voltage;
    return NvSuccess;
}

static NvError
SdioSetSlotClockRate(
    NvDdkSdioDeviceHandle hSdio,
    NvDdkSdioClkDivider Divider)
{
    NvU32 val = 0;

    // disable clk
    val = SDMMC_REGR(hSdio->pSdioVirtualAddress, SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL);
    val = NV_FLD_SET_DRF_DEF(SDMMC, SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL,
                             SD_CLOCK_EN, DISABLE, val);
    SDMMC_REGW(hSdio->pSdioVirtualAddress,
              SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL, val);

    val = SDMMC_REGR(hSdio->pSdioVirtualAddress, SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL);
    val = NV_FLD_SET_DRF_NUM(SDMMC, SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL,
                                      SDCLK_FREQUENCYSELECT, Divider, val);
    val = NV_FLD_SET_DRF_DEF(SDMMC, SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL,
                             SD_CLOCK_EN, ENABLE, val);
    SDMMC_REGW(hSdio->pSdioVirtualAddress, SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL, val);

    return NvSuccess;
}

NvError
NvDdkSdioSetClockFrequency(
    NvDdkSdioDeviceHandle hSdio,
    NvRmFreqKHz FrequencyKHz,
    NvRmFreqKHz* pConfiguredFrequencyKHz)
{
    NvError Error = NvSuccess;
    NvRmFreqKHz PrefFreqList[1];
    NvRmFreqKHz CurrentFreq;
    NvRmFreqKHz NewFreq;
    NvU32 i, Divider = 0;
    NvU32 Difference = 0xFFFFFFFF;
    NV_ASSERT(hSdio);

    PrefFreqList[0] = FrequencyKHz;

    // first disable the clk
    Error = NvRmPowerModuleClockControl(hSdio->hRm,
                                        NVRM_MODULE_ID(NvRmModuleID_Sdio, hSdio->Instance),
                                        hSdio->SdioRmPowerClientId,
                                        NV_FALSE);
    if (Error)
    {
        return Error;
    }

    // now enable clock to sdio controller
    Error = NvRmPowerModuleClockControl(hSdio->hRm,
                                        NVRM_MODULE_ID(NvRmModuleID_Sdio, hSdio->Instance),
                                        hSdio->SdioRmPowerClientId,
                                        NV_TRUE);
    if (Error)
    {
        return Error;
    }

    // request for clk
    Error = NvRmPowerModuleClockConfig(hSdio->hRm,
                                       NVRM_MODULE_ID(NvRmModuleID_Sdio, hSdio->Instance),
                                       hSdio->SdioRmPowerClientId,
                                       SdioNormalModeMinFreq,
                                       ((PrefFreqList[0] < SdioNormalModeMaxFreq) ? SdioNormalModeMaxFreq : 
                                                    SdioHighSpeedModeMaxFreq),
                                       PrefFreqList,
                                       1,
                                       &CurrentFreq, NvRmClockConfig_QuietOverClock);
    if (Error)
    {
        return Error;
    }
    
    if (PrefFreqList[0] < CurrentFreq)
    {
        for (i = 0; i < 9; i++)
        {
            NewFreq = (CurrentFreq >> i);
            if ((DIFF_FREQ(PrefFreqList[0], NewFreq)) < Difference)
            {
                Difference = DIFF_FREQ(PrefFreqList[0], NewFreq);
                Divider = i;
            }
        }
    }

    if (pConfiguredFrequencyKHz != NULL)
    {
        *pConfiguredFrequencyKHz = (CurrentFreq >> Divider);
    }

    // set the clk divider to zero
    if (Divider == 0)
    {
        Error = SdioSetSlotClockRate(hSdio, NvDdkSdioClkDivider_DIV_BASE);
    }
    else
    {
        Error = SdioSetSlotClockRate(hSdio, (NvDdkSdioClkDivider)(1 <<(Divider - 1)));
    }

    hSdio->ConfiguredFrequency = CurrentFreq;
    return Error;
}

NvError NvDdkSdioSetBlocksize(NvDdkSdioDeviceHandle hSdio, NvU32 Blocksize)
{
    NvU32 val = 0;

    NV_ASSERT(hSdio);

    if (Blocksize > hSdio->MaxBlockLength)
    {
         return NvError_SdioBadBlockSize;
    }

    val = SDMMC_REGR(hSdio->pSdioVirtualAddress, BLOCK_SIZE_BLOCK_COUNT);
    if ((Blocksize >= 1) && (Blocksize < 4096))
    {
        val =
            NV_FLD_SET_DRF_NUM(SDMMC, BLOCK_SIZE_BLOCK_COUNT, XFER_BLOCK_SIZE_12,
                                                    NV_FALSE, val);
    }
    else
    {
        NV_ASSERT(Blocksize && (Blocksize < 8192));
        // Transfer Block Size 12th bit. This bit is added to support 4Kb Data block transfer
        val =
            NV_FLD_SET_DRF_NUM(SDMMC, BLOCK_SIZE_BLOCK_COUNT, XFER_BLOCK_SIZE_12,
                                                    NV_TRUE, val);
    }
    val = NV_FLD_SET_DRF_NUM(SDMMC, BLOCK_SIZE_BLOCK_COUNT, XFER_BLOCK_SIZE_11_0,
                                                   Blocksize, val);
    SDMMC_REGW(hSdio->pSdioVirtualAddress, BLOCK_SIZE_BLOCK_COUNT, val);

    // read back the block size
    val = SDMMC_REGR(hSdio->pSdioVirtualAddress, BLOCK_SIZE_BLOCK_COUNT);
    val = NV_DRF_VAL(SDMMC, BLOCK_SIZE_BLOCK_COUNT, XFER_BLOCK_SIZE_11_0, val);
    if (val != Blocksize)
    {
        return NvError_SdioBadBlockSize;
    }

    return NvSuccess;
}

NvBool SdioIsReset(NvDdkSdioDeviceHandle hSdio)
{
    NvU32 val = 0;
    NvU32 Timeout = 0;

    while ((!val) && (Timeout != SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL_USEC))
    {
        val = SDMMC_REGR(hSdio->pSdioVirtualAddress,
                        SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL);
        val = NV_DRF_VAL(SDMMC, SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL,
                         SW_RESET_FOR_ALL, val);
        NvOsWaitUS(SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL_STEP_USEC);
        Timeout += SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL_STEP_USEC;
    }

    return (!val);
}

NvBool SdioEnableInternalClock(NvDdkSdioDeviceHandle hSdio)
{
    NvU32 val = 0;
    NvU32 Timeout = 0;

    val = SDMMC_REGR(hSdio->pSdioVirtualAddress,
                    SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL);
    val = NV_FLD_SET_DRF_DEF(SDMMC, SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL,
                             INTERNAL_CLOCK_EN, OSCILLATE, val);
    SDMMC_REGW(hSdio->pSdioVirtualAddress, SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL, val);

    while (Timeout != SDMMC_INTERNAL_CLOCK_TIMEOUT_USEC)
    {
        val = SDMMC_REGR(hSdio->pSdioVirtualAddress,
                        SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL);
        val = NV_DRF_VAL(SDMMC, SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL,
                         INTERNAL_CLOCK_STABLE, val);
        if (val)
        {
            break;
        }
        NvOsWaitUS(SDMMC_INTERNAL_CLOCK_TIMEOUT_STEP_USEC);
        Timeout += SDMMC_INTERNAL_CLOCK_TIMEOUT_STEP_USEC;
    }

    return (val);
}

NvError SdioEnableCardClock(NvDdkSdioDeviceHandle hSdio, NvBool IsEnable)
{
    NvU32 val = 0;
    NvU32 Timeout = 0;
    NvU32 IsControllerBusy = 0;

    NV_ASSERT(hSdio);

    NvOsIntrMutexLock(hSdio->SdioThreadSafetyMutex);
    if (IsEnable)
    {
        val = SDMMC_REGR(hSdio->pSdioVirtualAddress, SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL);
        val = NV_FLD_SET_DRF_DEF(SDMMC, SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL,
                                 SD_CLOCK_EN, ENABLE, val);
        if (!hSdio->IsSdControllerVersion2)
        {
            NvOsWaitUS(10);
        }
        SDMMC_REGW(hSdio->pSdioVirtualAddress,
                  SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL, val);

        while (Timeout != SDMMC_INTERNAL_CLOCK_TIMEOUT_USEC)
        {
            val = SDMMC_REGR(hSdio->pSdioVirtualAddress,
                                        SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL);
            val = NV_DRF_VAL(SDMMC, SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL,
                                                        INTERNAL_CLOCK_STABLE, val);
            if (val)
            {
                break;
            }
            NvOsWaitUS(SDMMC_INTERNAL_CLOCK_TIMEOUT_STEP_USEC);
            Timeout += SDMMC_INTERNAL_CLOCK_TIMEOUT_STEP_USEC;
        }
    }
    else
    {
         // check if controller is bsuy
        IsControllerBusy = SdioIsControllerBusy(hSdio, NV_TRUE);
        if (IsControllerBusy)
        {
            NvOsIntrMutexUnlock(hSdio->SdioThreadSafetyMutex);
            return NvError_SdioControllerBusy;
        }

        val = SDMMC_REGR(hSdio->pSdioVirtualAddress,
                        SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL);
        val = NV_FLD_SET_DRF_DEF(SDMMC, SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL, SD_CLOCK_EN,
                                 DISABLE, val);
        if (!hSdio->IsSdControllerVersion2)
        {
            NvOsWaitUS(10);
        }
        SDMMC_REGW(hSdio->pSdioVirtualAddress, SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL, val);
    }
    NvOsIntrMutexUnlock(hSdio->SdioThreadSafetyMutex);
    return NvSuccess;
}

NvError SdioConfigureCardClock(NvDdkSdioDeviceHandle hSdio, NvBool IsEnable)
{
    NV_ASSERT(hSdio);

    if (!hSdio->IsSdControllerVersion2)
    {
        SdioEnableCardClock(hSdio, IsEnable);
    }
    return NvSuccess;
}

NvError SdioEnableBusPower(NvDdkSdioDeviceHandle hSdio)
{
    NvU32 val = 0;

    val = SDMMC_REGR(hSdio->pSdioVirtualAddress, POWER_CONTROL_HOST);
    val = NV_FLD_SET_DRF_DEF(SDMMC, POWER_CONTROL_HOST, SD_BUS_POWER, POWER_ON, val);
    SDMMC_REGW(hSdio->pSdioVirtualAddress, POWER_CONTROL_HOST, val);

    return NvSuccess;
}

NvError
    SdioSetDataTimeout(
        NvDdkSdioDeviceHandle hSdio,
        SdioDataTimeout SdioDataToCounter)
{
    NvU32 val = 0;

    val = SDMMC_REGR(hSdio->pSdioVirtualAddress, SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL);
    val |= NV_DRF_NUM(SDMMC, SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL,
                      DATA_TIMEOUT_COUNTER_VALUE, SdioDataToCounter);
    SDMMC_REGW(hSdio->pSdioVirtualAddress, SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL, val);

    return NvSuccess;
}

static NvU32
SdioIsControllerBusy(
    NvDdkSdioDeviceHandle hSdio,
    NvBool IsDataCommand)
{
    volatile NvU32 val = 0;
    NvU32 IsControllerBusy = 0;
    NvU32 timeout = 0;


    while (timeout <= SW_CONTROLLER_BUSY_TIMEOUT_USEC)
    {
        val = SDMMC_REGR(hSdio->pSdioVirtualAddress, PRESENT_STATE);

        // check if CMD line is busy
        IsControllerBusy = NV_DRF_VAL(SDMMC, PRESENT_STATE, CMD_INHIBIT_CMD, val);
        if (!IsControllerBusy)
        {
            if (IsDataCommand)
            {
                 // check if DAT line is busy
                IsControllerBusy = (NV_DRF_VAL(SDMMC, PRESENT_STATE, CMD_INHIBIT_DAT, val));
                if (!IsControllerBusy)
                {
                    // check if DAT line is busy
                    IsControllerBusy = NV_DRF_VAL(SDMMC, PRESENT_STATE, DAT_LINE_ACTIVE, val);
                    if (!IsControllerBusy)
                    {
                        break;
                    }
                }
            }
            else
            {
                break;
            }
        }

        NvOsWaitUS(SW_CONTROLLER_BUSY_TIMEOUT_STEP_USEC);
        timeout += SW_CONTROLLER_BUSY_TIMEOUT_STEP_USEC;
    }
    return IsControllerBusy;
}

void
    NvDdkSdioAbort(
    NvDdkSdioDeviceHandle hSdio,
    SdioRequestType RequestType,
    NvU32 FunctionNumber)
{
    NvU32 val;

     hSdio->RequestType = RequestType;

    // enable the stop at block gap
    val = SDMMC_REGR(hSdio->pSdioVirtualAddress, POWER_CONTROL_HOST);
    val = NV_FLD_SET_DRF_DEF(SDMMC, POWER_CONTROL_HOST,
                                STOP_AT_BLOCK_GAP_REQUEST, TRANSFER, val);
    SDMMC_REGW(hSdio->pSdioVirtualAddress, POWER_CONTROL_HOST, val);
}

static void PrivSdioAbort(NvDdkSdioDeviceHandle hSdio)
{
    NvDdkSdioCommand pCommand;
    NvError Error = NvSuccess;
    NvU32 status = 0;

    // Issue CMD12

    // For memory cards send command 12 for abort and for i/o cards send command 52 for abort
    if (hSdio->RequestType == NVDDK_SDIO_MEMORY_ABORT_REQUEST)
    {
        pCommand.CmdArgument = 0;
        pCommand.CommandCode = 12;
        pCommand.ResponseType = NvDdkSdioRespType_R1b;
    }
    else
    {
        pCommand.CmdArgument = 0x80000C01;
        pCommand.CommandCode = 52;
        pCommand.ResponseType = NvDdkSdioRespType_R5;
    }

    pCommand.CommandType = NvDdkSdioCommandType_Abort;
    pCommand.IsDataCommand = NV_FALSE;

    Error = NvDdkSdioSendCommand(hSdio, &pCommand, &status);
    if (Error != NvSuccess)
    {
        NV_DEBUG_PRINTF(("PrivSdioAbort: Failed to send command abort command \n"));
    }
    else
    {
        Error = NvOsSemaphoreWaitTimeout(hSdio->PrivSdioSema,
                                                            NVDDK_SDMMC_ABORT_TIMEOUT_MSEC);
        if (Error != NvSuccess)
        {
            NV_DEBUG_PRINTF(("PrivSdioAbort: abort command timeout, Non Recoverable error \n"));
            //Reset the controller
            NvRmModuleReset(hSdio->hRm, NVRM_MODULE_ID(NvRmModuleID_Sdio, hSdio->Instance));
        }
    }

}

void PrivSdioReset(NvDdkSdioDeviceHandle hSdio)
{
    NvU32 val = 0;
    NvU32 Timeout = 0;

    // issue soft reset for both command and dat lines
   val = SDMMC_REGR(hSdio->pSdioVirtualAddress,
                    SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL);
    val = NV_FLD_SET_DRF_DEF(SDMMC, SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL,
                    SW_RESET_FOR_CMD_LINE, RESETED, val);
    val = NV_FLD_SET_DRF_DEF(SDMMC, SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL,
                        SW_RESET_FOR_DAT_LINE, RESETED, val);
    SDMMC_REGW(hSdio->pSdioVirtualAddress,
                      SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL, val);

    // For AP15 we need to wait for sometime for the information to synchronize
    if (!hSdio->IsSdControllerVersion2)
    {
        NvOsWaitUS(SDMMC_SOFT_RESET_DELAY_USEC);
    }
    else
    {
        while ((!val) && (Timeout != SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL_USEC))
        {
            val = SDMMC_REGR(hSdio->pSdioVirtualAddress,
                            SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL);
            val = NV_DRF_VAL(SDMMC, SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL,
                     SW_RESET_FOR_CMD_LINE, val);
            if (val)
            {
                val = NV_DRF_VAL(SDMMC, SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL,
                     SW_RESET_FOR_DAT_LINE, val);
            }
            NvOsWaitUS(SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL_STEP_USEC);
            Timeout += SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL_STEP_USEC;
        }
        if (Timeout == SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL_USEC)
        {
            SD_PRINT(("SDIO_DDK COMD and DAT line reset failed\n"));
        }
    }
}

void ConfigureInterrupts(NvDdkSdioDeviceHandle hSdio, NvU32 IntrEnableMask, NvU32 IntrDisableMask, NvU32 IntrStatusEnableMask)
{
    NvU32 val = 0;

    NvOsIntrMutexLock(hSdio->SdioThreadSafetyMutex);

    val = SDMMC_REGR(hSdio->pSdioVirtualAddress, INTERRUPT_STATUS_ENABLE);
    val |= IntrEnableMask;
    val &= ~IntrDisableMask;

    SDMMC_REGW(hSdio->pSdioVirtualAddress, INTERRUPT_STATUS_ENABLE, (val |IntrStatusEnableMask));

    #if !NV_OAL
    val = SDMMC_REGR(hSdio->pSdioVirtualAddress, INTERRUPT_SIGNAL_ENABLE);
    val |= IntrEnableMask;
    val &= ~IntrDisableMask;

    SDMMC_REGW(hSdio->pSdioVirtualAddress, INTERRUPT_SIGNAL_ENABLE, val);
    #endif

    NvOsIntrMutexUnlock(hSdio->SdioThreadSafetyMutex);

}

void
    NvDdkSdioEnableIoMode(
        NvDdkSdioDeviceHandle hSdio,
        NvBool IsAcceptCardEvents)
{
    NvU32 val = 0;

    NV_ASSERT(hSdio);
    if (IsAcceptCardEvents == NV_TRUE)
    {
        ConfigureInterrupts(hSdio, NvDdkSdioCommandStatus_Card, 0, 0);
        if (hSdio->IsSdControllerVersion2)
        {
            // enable the interrupt at block gap
            val = SDMMC_REGR(hSdio->pSdioVirtualAddress, POWER_CONTROL_HOST);
            val = NV_FLD_SET_DRF_DEF(SDMMC, POWER_CONTROL_HOST,
                                        INTERRUPT_AT_BLOCK_GAP, ENABLE, val);
            SDMMC_REGW(hSdio->pSdioVirtualAddress, POWER_CONTROL_HOST, val);
        }
    }
    else
    {
        ConfigureInterrupts(hSdio, 0, NvDdkSdioCommandStatus_Card, 0);
        if (hSdio->IsSdControllerVersion2)
        {
            // disable the interrupt at block gap
            val = SDMMC_REGR(hSdio->pSdioVirtualAddress, POWER_CONTROL_HOST);
            val = NV_FLD_SET_DRF_DEF(SDMMC, POWER_CONTROL_HOST,
                                        INTERRUPT_AT_BLOCK_GAP, DISABLE, val);
            SDMMC_REGW(hSdio->pSdioVirtualAddress, POWER_CONTROL_HOST, val);
        }
    }
    hSdio->IsAcceptCardEvents = IsAcceptCardEvents;
}

void NvDdkSdioSuspend(NvDdkSdioDeviceHandle hSdio, NvBool SwitchOffSDDevice)
{
    NvRmFreqKHz pConfiguredFrequencyKHz = 0;
    NvRmFreqKHz  SdioCurrentFreqKHz = 0;
    NvU32 val = 0;


    // switch off the voltage to the sd device
    if (SwitchOffSDDevice == NV_TRUE)
    {
        NvOdmSdioSuspend(hSdio->SdioOdmHandle);
    }

    if (!hSdio->ISControllerSuspended)
    {
        if (hSdio->IsSdControllerVersion2)
        {
                // remove the card clock
            SdioEnableCardClock(hSdio, NV_FALSE);

            // Disable SDMMC_CLK bit in VENDOR_CLOCK_CONTROL register
            val = SDMMC_REGR(hSdio->pSdioVirtualAddress, VENDOR_CLOCK_CNTRL);
            val = NV_FLD_SET_DRF_DEF(SDMMC, VENDOR_CLOCK_CNTRL,
                                        SDMMC_CLK, DISABLE, val);
            SDMMC_REGW(hSdio->pSdioVirtualAddress, VENDOR_CLOCK_CNTRL, val);

            // disable clock to sdio controller
            NvRmPowerModuleClockControl(hSdio->hRm,
                NVRM_MODULE_ID(NvRmModuleID_Sdio, hSdio->Instance),
                hSdio->SdioRmPowerClientId,
                NV_FALSE);
        }
        else
        {
            SdioCurrentFreqKHz = hSdio->ConfiguredFrequency;
            // Reduce clock to the lowest frequency, to support IO card interrupt detection
            // even when the sdio driver is suspended( provided system is not in LP0)
            NV_ASSERT_SUCCESS(NvDdkSdioSetClockFrequency(
                                        hSdio,
                                        SDMMC_LOW_POWER_FREQ_KHZ,
                                        &pConfiguredFrequencyKHz));
            hSdio->ConfiguredFrequency = SdioCurrentFreqKHz;
        }

            /* Report RM to disable power */
            NV_ASSERT_SUCCESS(NvRmPowerVoltageControl(hSdio->hRm,
                    NVRM_MODULE_ID(NvRmModuleID_Sdio, hSdio->Instance),
                    hSdio->SdioRmPowerClientId,
                    NvRmVoltsOff,
                    NvRmVoltsOff,
                    NULL,
                    0,
                    NULL));
        hSdio->ISControllerSuspended = NV_TRUE;
    }
}

NvError
NvDdkSdioResume(
    NvDdkSdioDeviceHandle hSdio,
    NvBool IsCardInserted,
    NvBool SwitchOnSDDevice)
{
    NvError e = NvSuccess;
    NvBool IsClkStable = NV_FALSE;
    NvRmFreqKHz pConfiguredFrequencyKHz = 0;
    NvRmPowerEvent Event = NvRmPowerEvent_NoEvent;
    NvBool IsReset = NV_FALSE;
    NvU32 val = 0;

    // switch on the voltage to the sd device
    if (SwitchOnSDDevice == NV_TRUE)
    {
        if (NV_TRUE != NvOdmSdioResume(hSdio->SdioOdmHandle))
        {
            NV_ASSERT(!"SdioOdm Resume Failed");
        }
    }

     /* enable power */
    NV_CHECK_ERROR_CLEANUP(NvRmPowerVoltageControl(hSdio->hRm,
                                                   NVRM_MODULE_ID(NvRmModuleID_Sdio, hSdio->Instance),
                                                   hSdio->SdioRmPowerClientId,
                                                   NvRmVoltsUnspecified,
                                                   NvRmVoltsUnspecified,
                                                   NULL,
                                                   0,
                                                   NULL));

    // now enable clock to sdio controller
    NV_CHECK_ERROR_CLEANUP(NvRmPowerModuleClockControl(
                                            hSdio->hRm,
                                            NVRM_MODULE_ID(NvRmModuleID_Sdio, hSdio->Instance),
                                            hSdio->SdioRmPowerClientId,
                                            NV_TRUE));
#if !NV_OAL
    NV_CHECK_ERROR_CLEANUP(NvRmPowerGetEvent(
        hSdio->hRm,
        hSdio->SdioRmPowerClientId,
        &Event ));
#endif
    if (IsCardInserted == NV_TRUE)
    {
        NvRmModuleReset(hSdio->hRm,
                    NVRM_MODULE_ID(NvRmModuleID_Sdio, hSdio->Instance));
        IsReset = SdioIsReset(hSdio);
        if (IsReset == NV_FALSE)
        {
            goto fail;
        }
        hSdio->BusWidth = NvDdkSdioDataWidth_1Bit;
    }

    // enable internal clock to sdio
    IsClkStable = SdioEnableInternalClock(hSdio);
    if (IsClkStable == NV_FALSE)
    {
        goto fail;
    }

    if (hSdio->IsSdControllerVersion2)
    {
        // enable the card clock
        NV_CHECK_ERROR_CLEANUP(SdioEnableCardClock(hSdio, NV_TRUE));

        // Enable SDMMC_CLK bit in VENDOR_CLOCK_CONTROL register
        val = SDMMC_REGR(hSdio->pSdioVirtualAddress, VENDOR_CLOCK_CNTRL);
        val = NV_FLD_SET_DRF_DEF(SDMMC, VENDOR_CLOCK_CNTRL,
                                    SDMMC_CLK, ENABLE, val);
        SDMMC_REGW(hSdio->pSdioVirtualAddress, VENDOR_CLOCK_CNTRL, val);
    }

    if ((Event == NvRmPowerEvent_WakeLP0) || (IsCardInserted == NV_TRUE))
    {
        // After LP0 need to reinitialize following settings

        // Restore current voltage setting
        NV_CHECK_ERROR_CLEANUP(NvDdkSdioSetSDBusVoltage(hSdio, hSdio->BusVoltage));

        // enable sd bus power
        NV_CHECK_ERROR_CLEANUP(SdioEnableBusPower(hSdio));

        // Initialize the block size again
        NV_CHECK_ERROR_CLEANUP(NvDdkSdioSetBlocksize(hSdio, SDMMC_DEFAULT_BLOCK_SIZE));

        // set data timeout counter value
        NV_CHECK_ERROR_CLEANUP(SdioSetDataTimeout(hSdio, SdioDataTimeout_COUNTER_128M));


        ConfigureInterrupts(hSdio, SDIO_INTERRUPTS, ~SDIO_INTERRUPTS, 0);

        NvDdkSdioEnableIoMode(hSdio, hSdio->IsAcceptCardEvents);

        // Restore current data width
        NV_ASSERT_SUCCESS(NvDdkSdioSetHostBusWidth(hSdio, hSdio->BusWidth));
    }

    NV_CHECK_ERROR_CLEANUP(NvDdkSdioSetClockFrequency(
                                                    hSdio,
                                                    hSdio->ConfiguredFrequency,
                                                    &pConfiguredFrequencyKHz));

    NV_CHECK_ERROR_CLEANUP(SdioConfigureCardClock(hSdio, NV_FALSE));

    hSdio->ISControllerSuspended = NV_FALSE;
    return NvSuccess;


fail:
    NV_DEBUG_PRINTF(("resume failed [%x]\n", e));
    NV_ASSERT(!"Sdio DDK Resume Failed");

    return e;
}

static void
GpioInterruptHandler(void *arg)
{
#if !NV_OAL
    NvBool IsCardInserted = NV_TRUE;
    NvError e = NvSuccess;
    NvDdkSdioDeviceHandle hSdio = (NvDdkSdioDeviceHandle)arg;

    if (hSdio->CardEventsSema)
    {
        // Disable power to the slot if the card is not inserted
        e = NvDdkSdioIsCardInserted(hSdio, &IsCardInserted);
        if (e == NvSuccess)
        {
            if (IsCardInserted)
            {
                NvOdmSdioResume(hSdio->SdioOdmHandle);
            }
            else
            {
               NvOdmSdioSuspend(hSdio->SdioOdmHandle);
            }
        }
        NvOsSemaphoreSignal(hSdio->CardEventsSema);
    }
    NvRmGpioInterruptDone(hSdio->GpioIntrHandle);
#endif
}

