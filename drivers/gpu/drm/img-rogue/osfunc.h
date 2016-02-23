/**************************************************************************/ /*!
@File
@Title          OS functions header
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    OS specific API definitions
@License        Dual MIT/GPLv2

The contents of this file are subject to the MIT license as set out below.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

Alternatively, the contents of this file may be used under the terms of
the GNU General Public License Version 2 ("GPL") in which case the provisions
of GPL are applicable instead of those above.

If you wish to allow use of your version of this file only under the terms of
GPL, and not to allow others to use your version of this file under the terms
of the MIT license, indicate your decision by deleting the provisions above
and replace them with the notice and other provisions required by GPL as set
out in the file called "GPL-COPYING" included in this distribution. If you do
not delete the provisions above, a recipient may use your version of this file
under the terms of either the MIT license or GPL.

This License is also included in this distribution in the file called
"MIT-COPYING".

EXCEPT AS OTHERWISE STATED IN A NEGOTIATED AGREEMENT: (A) THE SOFTWARE IS
PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT; AND (B) IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/ /***************************************************************************/

#ifdef DEBUG_RELEASE_BUILD
#pragma optimize( "", off )
#define DEBUG		1
#endif

#ifndef __OSFUNC_H__
#define __OSFUNC_H__


#if defined(__KERNEL__) && defined(ANDROID) && !defined(__GENKSYMS__)
#define __pvrsrv_defined_struct_enum__
#include <services_kernel_client.h>
#endif

#if defined(__QNXNTO__)
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#endif

#include "img_types.h"
#include "pvrsrv_device.h"
#include "device.h"

/******************************************************************************
 * Static defines
 *****************************************************************************/
#define KERNEL_ID			0xffffffffL
#define ISR_ID				0xfffffffdL

IMG_UINT64 OSClockns64(void);
IMG_UINT64 OSClockus64(void);
IMG_UINT32 OSClockus(void);
IMG_UINT32 OSClockms(void);

size_t OSGetPageSize(void);
size_t OSGetPageShift(void);
size_t OSGetPageMask(void);
size_t OSGetOrder(size_t uSize);

typedef void (*PFN_THREAD)(void *pvData);

PVRSRV_ERROR OSChangeSparseMemCPUAddrMap(void **psPageArray,
										IMG_UINT64 sCpuVAddrBase,
										uintptr_t sCpuPAHeapBase,
										IMG_UINT32 ui32AllocPageCount,
										IMG_UINT32 *pai32AllocIndices,
										IMG_UINT32 ui32FreePageCount,
										IMG_UINT32 *pai32FreeIndices,
										IMG_UINT32	*pui32Status,
										IMG_BOOL bIsLMA);

PVRSRV_ERROR OSInstallDeviceLISR(PVRSRV_DEVICE_CONFIG *psDevConfig,
								 IMG_HANDLE *hLISRData,
								 PFN_LISR pfnLISR,
								 void *hData);
PVRSRV_ERROR OSUninstallDeviceLISR(IMG_HANDLE hLISRData);

PVRSRV_ERROR OSInstallMISR(IMG_HANDLE *hMISRData,
						   PFN_MISR pfnMISR,
						   void *hData);
PVRSRV_ERROR OSUninstallMISR(IMG_HANDLE hMISRData);
PVRSRV_ERROR OSScheduleMISR(IMG_HANDLE hMISRData);




/*************************************************************************/ /*!
@Function       OSThreadCreate
@Description    Creates a kernel thread and starts it running. The caller
                is responsible for informing the thread that it must finish
                and return from the pfnThread function. It is not possible
                to kill or terminate it.The new thread runs with the default
                priority provided by the Operating System.
@Output         phThread       Returned handle to the thread.
@Input          pszThreadName  Name to assign to the thread.
@Input          pfnThread      Thread entry point function.
@Input          hData          Thread specific data pointer for pfnThread().
@Return         Standard PVRSRV_ERROR error code.
*/ /**************************************************************************/

PVRSRV_ERROR OSThreadCreate(IMG_HANDLE *phThread,
							IMG_CHAR *pszThreadName,
							PFN_THREAD pfnThread,
							void *hData);

/*! Available priority levels for the creation of a new Kernel Thread. */
typedef enum priority_levels
{
	OS_THREAD_HIGHEST_PRIORITY = 0,
	OS_THREAD_HIGH_PRIORITY,
	OS_THREAD_NORMAL_PRIORITY,
	OS_THREAD_LOW_PRIORITY,
	OS_THREAD_LOWEST_PRIORITY,
	OS_THREAD_NOSET_PRIORITY,   /* With this option the priority level is is the default for the given OS */
	OS_THREAD_LAST_PRIORITY     /* This must be always the last entry */
} OS_THREAD_LEVEL;

/*************************************************************************/ /*!
@Function       OSThreadCreatePriority
@Description    As OSThreadCreate, this function creates a kernel thread and
                starts it running. The difference is that with this function
                is possible to specify the priority used to schedule the new
                thread.

@Output         phThread        Returned handle to the thread.
@Input          pszThreadName   Name to assign to the thread.
@Input          pfnThread       Thread entry point function.
@Input          hData           Thread specific data pointer for pfnThread().
@Input          eThreadPriority Priority level to assign to the new thread.
@Return         Standard PVRSRV_ERROR error code.
*/ /**************************************************************************/
PVRSRV_ERROR OSThreadCreatePriority(IMG_HANDLE *phThread,
									IMG_CHAR *pszThreadName,
									PFN_THREAD pfnThread,
									void *hData,
									OS_THREAD_LEVEL eThreadPriority);

/*************************************************************************/ /*!
@Function       OSThreadDestroy
@Description    Waits for the thread to end and then destroys the thread
                handle memory. This function will block and wait for the
                thread to finish successfully, thereby providing a sync point
                for the thread completing its work. No attempt is made to kill
                or otherwise terminate the thread.
@Input          phThread  The thread handle returned by OSThreadCreate().
@Return         Standard PVRSRV_ERROR error code.
*/ /**************************************************************************/
PVRSRV_ERROR OSThreadDestroy(IMG_HANDLE hThread);

void PVRSRVDeviceMemSet(void *pvDest, IMG_UINT8 ui8Value, size_t ui32Size);
void PVRSRVDeviceMemCopy(void *pvDst, const void *pvSrc, size_t ui32Size);

#if defined(__arm64__) || defined(__aarch64__) || defined (PVRSRV_DEVMEM_SAFE_MEMSETCPY)
#define OSDeviceMemSet(a,b,c) PVRSRVDeviceMemSet((a), (b), (c))
#define OSDeviceMemCopy(a,b,c) PVRSRVDeviceMemCopy((a), (b), (c))
#define OSMemSet(a,b,c)  PVRSRVDeviceMemSet((a), (b), (c))
#define OSMemCopy(a,b,c)  PVRSRVDeviceMemCopy((a), (b), (c))
#else
#define OSDeviceMemSet(a,b,c) memset((a), (b), (c))
#define OSDeviceMemCopy(a,b,c) memcpy((a), (b), (c))
#define OSMemSet(a,b,c)  memset((a), (b), (c))
#define OSMemCopy(a,b,c)  memcpy((a), (b), (c))
#endif

#define OSCachedMemSet(a,b,c) memset((a), (b), (c))
#define OSCachedMemCopy(a,b,c) memcpy((a), (b), (c))

void *OSMapPhysToLin(IMG_CPU_PHYADDR BasePAddr, size_t ui32Bytes, IMG_UINT32 ui32Flags);
IMG_BOOL OSUnMapPhysToLin(void *pvLinAddr, size_t ui32Bytes, IMG_UINT32 ui32Flags);


PVRSRV_ERROR OSCPUOperation(PVRSRV_CACHE_OP eCacheOp);

void OSFlushCPUCacheRangeKM(void *pvVirtStart,
							void *pvVirtEnd,
							IMG_CPU_PHYADDR sCPUPhysStart,
							IMG_CPU_PHYADDR sCPUPhysEnd);


void OSCleanCPUCacheRangeKM(void *pvVirtStart,
							void *pvVirtEnd,
							IMG_CPU_PHYADDR sCPUPhysStart,
							IMG_CPU_PHYADDR sCPUPhysEnd);

void OSInvalidateCPUCacheRangeKM(void *pvVirtStart,
								 void *pvVirtEnd,
								 IMG_CPU_PHYADDR sCPUPhysStart,
								 IMG_CPU_PHYADDR sCPUPhysEnd);

typedef enum _IMG_DCACHE_ATTRIBUTE_
{
	PVR_DCACHE_LINE_SIZE = 0,
	PVR_DCACHE_ATTRIBUTE_COUNT
} IMG_DCACHE_ATTRIBUTE;

IMG_UINT32 OSCPUCacheAttributeSize(IMG_DCACHE_ATTRIBUTE eCacheAttribute);

IMG_PID OSGetCurrentProcessID(void);
IMG_CHAR *OSGetCurrentProcessName(void);
uintptr_t OSGetCurrentThreadID(void);

IMG_PID OSGetCurrentClientProcessIDKM(void);
IMG_CHAR *OSGetCurrentClientProcessNameKM(void);
uintptr_t OSGetCurrentClientThreadIDKM(void);




IMG_INT OSMemCmp(void *pvBufA, void *pvBufB, size_t uiLen);

PVRSRV_ERROR OSPhyContigPagesAlloc(PVRSRV_DEVICE_NODE *psDevNode, size_t uiSize,
							PG_HANDLE *psMemHandle, IMG_DEV_PHYADDR *psDevPAddr);

void OSPhyContigPagesFree(PVRSRV_DEVICE_NODE *psDevNode, PG_HANDLE *psMemHandle);

PVRSRV_ERROR OSPhyContigPagesMap(PVRSRV_DEVICE_NODE *psDevNode, PG_HANDLE *psMemHandle,
						size_t uiSize, IMG_DEV_PHYADDR *psDevPAddr,
						void **pvPtr);

void OSPhyContigPagesUnmap(PVRSRV_DEVICE_NODE *psDevNode, PG_HANDLE *psMemHandle, void *pvPtr);


PVRSRV_ERROR OSInitEnvData(void);
void OSDeInitEnvData(void);

IMG_CHAR* OSStringNCopy(IMG_CHAR *pszDest, const IMG_CHAR *pszSrc, size_t uSize);
IMG_INT32 OSSNPrintf(IMG_CHAR *pStr, size_t ui32Size, const IMG_CHAR *pszFormat, ...) IMG_FORMAT_PRINTF(3, 4);
size_t OSStringLength(const IMG_CHAR *pStr);
size_t OSStringNLength(const IMG_CHAR *pStr, size_t uiCount);
IMG_INT32 OSStringCompare(const IMG_CHAR *pStr1, const IMG_CHAR *pStr2);

PVRSRV_ERROR OSEventObjectCreate(const IMG_CHAR *pszName,
								 IMG_HANDLE *EventObject);
PVRSRV_ERROR OSEventObjectDestroy(IMG_HANDLE hEventObject);
PVRSRV_ERROR OSEventObjectSignal(IMG_HANDLE hEventObject);
PVRSRV_ERROR OSEventObjectWait(IMG_HANDLE hOSEventKM);
PVRSRV_ERROR OSEventObjectWaitTimeout(IMG_HANDLE hOSEventKM, IMG_UINT32 uiTimeoutMs);
PVRSRV_ERROR OSEventObjectWaitAndHoldBridgeLock(IMG_HANDLE hOSEventKM);
PVRSRV_ERROR OSEventObjectWaitTimeoutAndHoldBridgeLock(IMG_HANDLE hOSEventKM, IMG_UINT32 uiTimeoutMs);
PVRSRV_ERROR OSEventObjectOpen(IMG_HANDLE hEventObject,
											IMG_HANDLE *phOSEvent);
PVRSRV_ERROR OSEventObjectClose(IMG_HANDLE hOSEventKM);

/* Avoid macros so we don't evaluate pslzSrc twice */
static INLINE IMG_CHAR *OSStringCopy(IMG_CHAR *pszDest, const IMG_CHAR *pszSrc)
{
	return OSStringNCopy(pszDest, pszSrc, OSStringLength(pszSrc) + 1);
}

/*!
******************************************************************************

 @Function OSWaitus
 
 @Description 
    This function implements a busy wait of the specified microseconds
    This function does NOT release thread quanta
 
 @Input ui32Timeus - (us)

 @Return void

******************************************************************************/ 
void OSWaitus(IMG_UINT32 ui32Timeus);


/*!
******************************************************************************

 @Function OSSleepms
 
 @Description 
    This function implements a sleep of the specified milliseconds
    This function may allow pre-emption if implemented
 
 @Input ui32Timems - (ms)

 @Return void

******************************************************************************/ 
void OSSleepms(IMG_UINT32 ui32Timems);

void OSReleaseThreadQuanta(void);

IMG_UINT8 OSReadHWReg8(void *pvLinRegBaseAddr, IMG_UINT32 ui32Offset);
IMG_UINT16 OSReadHWReg16(void *pvLinRegBaseAddr, IMG_UINT32 ui32Offset);
IMG_UINT32 OSReadHWReg32(void *pvLinRegBaseAddr, IMG_UINT32 ui32Offset);
IMG_UINT64 OSReadHWReg64(void *pvLinRegBaseAddr, IMG_UINT32 ui32Offset);
IMG_UINT64 OSReadHWRegBank(void *pvLinRegBaseAddr, IMG_UINT32 ui32Offset, IMG_UINT8 *pui8DstBuf, IMG_UINT64 ui64DstBufLen);

void OSWriteHWReg8(void *pvLinRegBaseAddr, IMG_UINT32 ui32Offset, IMG_UINT8 ui32Value);
void OSWriteHWReg16(void *pvLinRegBaseAddr, IMG_UINT32 ui32Offset, IMG_UINT16 ui32Value);
void OSWriteHWReg32(void *pvLinRegBaseAddr, IMG_UINT32 ui32Offset, IMG_UINT32 ui32Value);
void OSWriteHWReg64(void *pvLinRegBaseAddr, IMG_UINT32 ui32Offset, IMG_UINT64 ui64Value);
IMG_UINT64 OSWriteHWRegBank(void *pvLinRegBaseAddr, IMG_UINT32 ui32Offset, IMG_UINT8 *pui8SrcBuf, IMG_UINT64 ui64rcBufLen);

typedef void (*PFN_TIMER_FUNC)(void*);
IMG_HANDLE OSAddTimer(PFN_TIMER_FUNC pfnTimerFunc, void *pvData, IMG_UINT32 ui32MsTimeout);
PVRSRV_ERROR OSRemoveTimer(IMG_HANDLE hTimer);
PVRSRV_ERROR OSEnableTimer(IMG_HANDLE hTimer);
PVRSRV_ERROR OSDisableTimer(IMG_HANDLE hTimer);


/******************************************************************************

 @Function		OSPanic

 @Description	Take action in response to an unrecoverable driver error

 @Input    void

 @Return   void

******************************************************************************/
void OSPanic(void);

IMG_BOOL OSProcHasPrivSrvInit(void);

typedef enum _img_verify_test
{
	PVR_VERIFY_WRITE = 0,
	PVR_VERIFY_READ
} IMG_VERIFY_TEST;

IMG_BOOL OSAccessOK(IMG_VERIFY_TEST eVerification, void *pvUserPtr, size_t ui32Bytes);

PVRSRV_ERROR OSCopyToUser(void *pvProcess, void *pvDest, const void *pvSrc, size_t ui32Bytes);
PVRSRV_ERROR OSCopyFromUser(void *pvProcess, void *pvDest, const void *pvSrc, size_t ui32Bytes);

#if defined (__linux__) || defined (WINDOWS_WDF)
#define OSBridgeCopyFromUser OSCopyFromUser
#define OSBridgeCopyToUser OSCopyToUser
#else
PVRSRV_ERROR OSBridgeCopyFromUser (void *pvProcess,
						void *pvDest,
						const void *pvSrc,
						size_t ui32Bytes);
PVRSRV_ERROR OSBridgeCopyToUser (void *pvProcess,
						void *pvDest,
						const void *pvSrc,
						size_t ui32Bytes);
#endif

/* Fairly arbitrary sizes - hopefully enough for all bridge calls */
#define PVRSRV_MAX_BRIDGE_IN_SIZE      0x2000
#define PVRSRV_MAX_BRIDGE_OUT_SIZE     0x1000

PVRSRV_ERROR OSGetGlobalBridgeBuffers (void **ppvBridgeInBuffer,
							IMG_UINT32 *pui32BridgeInBufferSize,
							void **ppvBridgeOutBuffer,
							IMG_UINT32 *pui32BridgeOutBufferSize);

IMG_BOOL OSSetDriverSuspended(void);
IMG_BOOL OSClearDriverSuspended(void);
IMG_BOOL OSGetDriverSuspended(void);

void OSWriteMemoryBarrier(void);
void OSMemoryBarrier(void);

#if defined(LINUX) && defined(__KERNEL__)

/* Provide LockDep friendly definitions for Services RW locks */
#include <linux/mutex.h>
#include <linux/slab.h>
#include "allocmem.h"

typedef struct rw_semaphore *POSWR_LOCK;

#define OSWRLockCreate(ppsLock) ({ \
	PVRSRV_ERROR e = PVRSRV_ERROR_OUT_OF_MEMORY; \
	*(ppsLock) = OSAllocMem(sizeof(struct rw_semaphore)); \
	if (*(ppsLock)) { init_rwsem(*(ppsLock)); e = PVRSRV_OK; }; \
	e;})
#define OSWRLockDestroy(psLock) ({OSFreeMem(psLock); PVRSRV_OK;})

#define OSWRLockAcquireRead(psLock) ({down_read(psLock); PVRSRV_OK;})
#define OSWRLockReleaseRead(psLock) ({up_read(psLock); PVRSRV_OK;})
#define OSWRLockAcquireWrite(psLock) ({down_write(psLock); PVRSRV_OK;})
#define OSWRLockReleaseWrite(psLock) ({up_write(psLock); PVRSRV_OK;})

#elif defined(LINUX) || defined(__QNXNTO__)
/* User-mode unit tests use these definitions on Linux */

typedef struct _OSWR_LOCK_ *POSWR_LOCK;

PVRSRV_ERROR OSWRLockCreate(POSWR_LOCK *ppsLock);
void OSWRLockDestroy(POSWR_LOCK psLock);
void OSWRLockAcquireRead(POSWR_LOCK psLock);
void OSWRLockReleaseRead(POSWR_LOCK psLock);
void OSWRLockAcquireWrite(POSWR_LOCK psLock);
void OSWRLockReleaseWrite(POSWR_LOCK psLock);

#else
struct _OSWR_LOCK_ {
	IMG_UINT32 ui32Dummy;
};
#if defined(WINDOWS_WDF)
	typedef struct _OSWR_LOCK_ *POSWR_LOCK;
#endif

static INLINE PVRSRV_ERROR OSWRLockCreate(POSWR_LOCK *ppsLock)
{
	PVR_UNREFERENCED_PARAMETER(ppsLock);
	return PVRSRV_OK;
}

static INLINE void OSWRLockDestroy(POSWR_LOCK psLock)
{
	PVR_UNREFERENCED_PARAMETER(psLock);
}

static INLINE void OSWRLockAcquireRead(POSWR_LOCK psLock)
{
	PVR_UNREFERENCED_PARAMETER(psLock);
}

static INLINE void OSWRLockReleaseRead(POSWR_LOCK psLock)
{
	PVR_UNREFERENCED_PARAMETER(psLock);
}

static INLINE void OSWRLockAcquireWrite(POSWR_LOCK psLock)
{
	PVR_UNREFERENCED_PARAMETER(psLock);
}

static INLINE void OSWRLockReleaseWrite(POSWR_LOCK psLock)
{
	PVR_UNREFERENCED_PARAMETER(psLock);
}
#endif

IMG_UINT64 OSDivide64r64(IMG_UINT64 ui64Divident, IMG_UINT32 ui32Divisor, IMG_UINT32 *pui32Remainder);
IMG_UINT32 OSDivide64(IMG_UINT64 ui64Divident, IMG_UINT32 ui32Divisor, IMG_UINT32 *pui32Remainder);

void OSDumpStack(void);

void OSAcquireBridgeLock(void);
void OSReleaseBridgeLock(void);
#if defined(LINUX)
void PMRLock(void);
void PMRUnlock(void);
IMG_BOOL PMRIsLocked(void);
IMG_BOOL PMRIsLockedByMe(void);
#else
#define PMRLock()
#define PMRUnlock()
#define PMRIsLocked() IMG_FALSE
#define PMRIsLockedByMe() IMG_FALSE
#endif


/*
 *  Functions for providing support for PID statistics.
 */
typedef void (OS_STATS_PRINTF_FUNC)(void *pvFilePtr, const IMG_CHAR *pszFormat, ...);
 
typedef void (OS_STATS_PRINT_FUNC)(void *pvFilePtr,
								   void *pvStatPtr,
								   OS_STATS_PRINTF_FUNC* pfnOSGetStatsPrintf);

typedef IMG_UINT32 (OS_INC_STATS_MEM_REFCOUNT_FUNC)(void *pvStatPtr);
typedef IMG_UINT32 (OS_DEC_STATS_MEM_REFCOUNT_FUNC)(void *pvStatPtr);

void *OSCreateStatisticEntry(IMG_CHAR* pszName, void *pvFolder,
							 OS_STATS_PRINT_FUNC* pfnStatsPrint,
							 OS_INC_STATS_MEM_REFCOUNT_FUNC* pfnIncMemRefCt,
							 OS_DEC_STATS_MEM_REFCOUNT_FUNC* pfnDecMemRefCt,
							 void *pvData);
void OSRemoveStatisticEntry(void *pvEntry);
void *OSCreateStatisticFolder(IMG_CHAR *pszName, void *pvFolder);
void OSRemoveStatisticFolder(void **ppvFolder);

void OSUserModeAccessToPerfCountersEn(void);

#endif /* __OSFUNC_H__ */

/******************************************************************************
 End of file (osfunc.h)
******************************************************************************/

