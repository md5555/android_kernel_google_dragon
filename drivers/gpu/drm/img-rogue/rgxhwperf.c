/*************************************************************************/ /*!
@File
@Title          RGX HW Performance implementation
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    RGX HW Performance implementation
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
*/ /**************************************************************************/

//#define PVR_DPF_FUNCTION_TRACE_ON 1
#undef PVR_DPF_FUNCTION_TRACE_ON

#include "pvr_debug.h"
#include "pvr_hwperf.h"
#include "osfunc.h"
#include "allocmem.h"

#include "pvrsrv.h"
#include "tlclient.h"
#include "tlstream.h"

#include "rgx_hwperf_km.h"
#include "rgxhwperf.h"
#include "rgxapi_km.h"
#include "rgxfwutils.h"
#include "devicemem_pdump.h"
#include "pdump_km.h"

#if defined(SUPPORT_GPUTRACE_EVENTS)
#include "pvr_gputrace.h"
#endif

#define HWPERF_TL_STREAM_NAME  "hwperf"
#define HWPERF_HOST_TL_STREAM_NAME "hwperf_host"
#define HWPERF_HOST_TL_STREAM_SIZE 0x10000 /* 64KB */

/* Defined to ensure HWPerf packets are not delayed */
#define SUPPORT_TL_PROODUCER_CALLBACK 1


/******************************************************************************
 *
 *****************************************************************************/


/*
	RGXHWPerfCopyDataL1toL2
*/
static IMG_UINT32 RGXHWPerfCopyDataL1toL2(IMG_HANDLE hHWPerfStream,
										  IMG_BYTE   *pbFwBuffer, 
										  IMG_UINT32 ui32BytesExp)
{
  	IMG_BYTE 	 *pbL2Buffer;
	IMG_UINT32   ui32L2BufFree;
	IMG_UINT32   ui32BytesCopied = 0;
	IMG_UINT32   ui32BytesExpMin = RGX_HWPERF_GET_SIZE(RGX_HWPERF_GET_PACKET(pbFwBuffer));
	PVRSRV_ERROR eError;

/* HWPERF_MISR_FUNC_DEBUG enables debug code for investigating HWPerf issues */
#ifdef HWPERF_MISR_FUNC_DEBUG
	static IMG_UINT32 gui32Ordinal = IMG_UINT32_MAX;
#endif

	PVR_DPF_ENTERED;

#ifdef HWPERF_MISR_FUNC_DEBUG
	PVR_DPF((PVR_DBG_VERBOSE, "EVENTS to copy from 0x%p length:%05d",
							  pbFwBuffer, ui32BytesExp));
#endif

#ifdef HWPERF_MISR_FUNC_DEBUG
	{
		/* Check the incoming buffer of data has not lost any packets */
 	 	IMG_BYTE *pbFwBufferIter = pbFwBuffer;
 	 	IMG_BYTE *pbFwBufferEnd = pbFwBuffer+ui32BytesExp;
	 	do
		{
			RGX_HWPERF_V2_PACKET_HDR *asCurPos = RGX_HWPERF_GET_PACKET(pbFwBufferIter);
			IMG_UINT32 ui32CurOrdinal = asCurPos->ui32Ordinal;
			if (gui32Ordinal != IMG_UINT32_MAX)
			{
				if ((gui32Ordinal+1) != ui32CurOrdinal)
				{
					if (gui32Ordinal < ui32CurOrdinal)
					{
						PVR_DPF((PVR_DBG_WARNING,
								 "HWPerf [%p] packets lost (%u packets) between ordinal %u...%u",
								 pbFwBufferIter,
								 ui32CurOrdinal - gui32Ordinal - 1,
								 gui32Ordinal,
								 ui32CurOrdinal));
					}
					else
					{
						PVR_DPF((PVR_DBG_WARNING,
								 "HWPerf [%p] packet ordinal out of sequence last: %u, current: %u",
								  pbFwBufferIter,
								  gui32Ordinal,
								  ui32CurOrdinal));
					}
				}
			}
			gui32Ordinal = asCurPos->ui32Ordinal;
			pbFwBufferIter += RGX_HWPERF_GET_SIZE(asCurPos);
		} while( pbFwBufferIter < pbFwBufferEnd );
	}
#endif

	/* Try submitting all data in one TL packet. */
	eError = TLStreamReserve2( hHWPerfStream, 
							   &pbL2Buffer, 
							   (size_t)ui32BytesExp, ui32BytesExpMin,
							   &ui32L2BufFree);
	if ( eError == PVRSRV_OK )
	{
		OSMemCopy( pbL2Buffer, pbFwBuffer, (size_t)ui32BytesExp );
		eError = TLStreamCommit(hHWPerfStream, (size_t)ui32BytesExp);
		if ( eError != PVRSRV_OK )
		{
			PVR_DPF((PVR_DBG_ERROR,
					 "TLStreamCommit() failed (%d) in %s(), unable to copy packet from L1 to L2 buffer",
					 eError, __func__));
			goto e0;
		}
		/* Data were successfully written */
		ui32BytesCopied = ui32BytesExp;
	}
	else if (eError == PVRSRV_ERROR_STREAM_RESERVE_TOO_BIG)
	{
		/* There was not enough space for all data, copy as much as possible */
		IMG_UINT32                sizeSum  = 0;
		RGX_PHWPERF_V2_PACKET_HDR psCurPkt = RGX_HWPERF_GET_PACKET(pbFwBuffer);

		PVR_DPF((PVR_DBG_MESSAGE, "Unable to reserve space (%d) in host buffer on first attempt, remaining free space: %d", ui32BytesExp, ui32L2BufFree));

		/* Traverse the array to find how many packets will fit in the available space. */
		while ( sizeSum < ui32BytesExp  &&
				sizeSum + RGX_HWPERF_GET_SIZE(psCurPkt) < ui32L2BufFree )
		{
			sizeSum += RGX_HWPERF_GET_SIZE(psCurPkt);
			psCurPkt = RGX_HWPERF_GET_NEXT_PACKET(psCurPkt);
		}

		if ( 0 != sizeSum )
		{
			eError = TLStreamReserve( hHWPerfStream, &pbL2Buffer, (size_t)sizeSum);

			if ( eError == PVRSRV_OK )
			{
				OSMemCopy( pbL2Buffer, pbFwBuffer, (size_t)sizeSum );
				eError = TLStreamCommit(hHWPerfStream, (size_t)sizeSum);
				if ( eError != PVRSRV_OK )
				{
					PVR_DPF((PVR_DBG_ERROR,
							 "TLStreamCommit() failed (%d) in %s(), unable to copy packet from L1 to L2 buffer",
							 eError, __func__));
					goto e0;
				}
				/* sizeSum bytes of hwperf packets have been successfully written */
				ui32BytesCopied = sizeSum;
			}
			else if ( PVRSRV_ERROR_STREAM_RESERVE_TOO_BIG == eError )
			{
				PVR_DPF((PVR_DBG_WARNING, "Can not write HWPerf packet into host buffer, check data in case of packet loss, remaining free space: %d", ui32L2BufFree));
			}
		}
		else
		{
			PVR_DPF((PVR_DBG_MESSAGE, "Can not find space in host buffer, check data in case of packet loss, remaining free space: %d", ui32L2BufFree));
		}
	}
	if ( PVRSRV_OK != eError && /*  Some other error occurred */
	     PVRSRV_ERROR_STREAM_RESERVE_TOO_BIG != eError ) /* Full error handled by caller, we returning the copied bytes count to caller*/
	{
		PVR_DPF((PVR_DBG_ERROR,
				 "HWPerf enabled: Unexpected Error ( %d ) while copying FW buffer to TL buffer.",
				 eError));
	}

e0:
	/* Return the remaining packets left to be transported. */
	PVR_DPF_RETURN_VAL(ui32BytesCopied);
}


static INLINE IMG_UINT32 RGXHWPerfAdvanceRIdx(
		const IMG_UINT32 ui32BufSize,
		const IMG_UINT32 ui32Pos,
		const IMG_UINT32 ui32Size)
{
	return (  ui32Pos + ui32Size < ui32BufSize ? ui32Pos + ui32Size : 0 );
}


/*
	RGXHWPerfDataStore
*/
static IMG_UINT32 RGXHWPerfDataStore(PVRSRV_RGXDEV_INFO	*psDevInfo)
{
	RGXFWIF_TRACEBUF	*psRGXFWIfTraceBufCtl = psDevInfo->psRGXFWIfTraceBuf;
	IMG_BYTE*			psHwPerfInfo = psDevInfo->psRGXFWIfHWPerfBuf;
	IMG_UINT32			ui32SrcRIdx, ui32SrcWIdx, ui32SrcWrapCount;
	IMG_UINT32			ui32BytesExp = 0, ui32BytesCopied = 0, ui32BytesCopiedSum = 0;
#ifdef HWPERF_MISR_FUNC_DEBUG
	IMG_UINT32			ui32BytesExpSum = 0;
#endif
	
	PVR_DPF_ENTERED;

	/* Caller should check this member is valid before calling */
	PVR_ASSERT(psDevInfo->hHWPerfStream);
	
 	/* Get a copy of the current
	 *   read (first packet to read) 
	 *   write (empty location for the next write to be inserted) 
	 *   WrapCount (size in bytes of the buffer at or past end)
	 * indexes of the FW buffer */
	ui32SrcRIdx = psRGXFWIfTraceBufCtl->ui32HWPerfRIdx;
	ui32SrcWIdx = psRGXFWIfTraceBufCtl->ui32HWPerfWIdx;
	OSMemoryBarrier();
	ui32SrcWrapCount = psRGXFWIfTraceBufCtl->ui32HWPerfWrapCount;

	/* Is there any data in the buffer not yet retrieved? */
	if ( ui32SrcRIdx != ui32SrcWIdx )
	{
		PVR_DPF((PVR_DBG_MESSAGE, "RGXHWPerfDataStore EVENTS found srcRIdx:%d srcWIdx: %d ", ui32SrcRIdx, ui32SrcWIdx));

		/* Is the write position higher than the read position? */
		if ( ui32SrcWIdx > ui32SrcRIdx )
		{
			/* Yes, buffer has not wrapped */
			ui32BytesExp  = ui32SrcWIdx - ui32SrcRIdx;
#ifdef HWPERF_MISR_FUNC_DEBUG
			ui32BytesExpSum += ui32BytesExp;
#endif
			ui32BytesCopied = RGXHWPerfCopyDataL1toL2(psDevInfo->hHWPerfStream,
													  psHwPerfInfo + ui32SrcRIdx,
													  ui32BytesExp);
			ui32BytesCopiedSum += ui32BytesCopied;

			/* Advance the read index and the free bytes counter by the number
			 * of bytes transported. Items will be left in buffer if not all data
			 * could be transported. Exit to allow buffer to drain. */
			psRGXFWIfTraceBufCtl->ui32HWPerfRIdx = RGXHWPerfAdvanceRIdx(
					psDevInfo->ui32RGXFWIfHWPerfBufSize, ui32SrcRIdx,
					ui32BytesCopied);
		}
		/* No, buffer has wrapped and write position is behind read position */
		else
		{
			/* Byte count equal to 
			 *     number of bytes from read position to the end of the buffer, 
			 *   + data in the extra space in the end of the buffer. */
			ui32BytesExp = ui32SrcWrapCount - ui32SrcRIdx;

#ifdef HWPERF_MISR_FUNC_DEBUG
			ui32BytesExpSum += ui32BytesExp;
#endif
			/* Attempt to transfer the packets to the TL stream buffer */
			ui32BytesCopied = RGXHWPerfCopyDataL1toL2(psDevInfo->hHWPerfStream,
													  psHwPerfInfo + ui32SrcRIdx,
													  ui32BytesExp);
			ui32BytesCopiedSum += ui32BytesCopied;

			/* Advance read index as before and Update the local copy of the
			 * read index as it might be used in the last if branch*/
			ui32SrcRIdx = RGXHWPerfAdvanceRIdx(
					psDevInfo->ui32RGXFWIfHWPerfBufSize, ui32SrcRIdx,
					ui32BytesCopied);

			/* Update Wrap Count */
			if ( ui32SrcRIdx == 0)
			{
				psRGXFWIfTraceBufCtl->ui32HWPerfWrapCount = psDevInfo->ui32RGXFWIfHWPerfBufSize;
			}
			psRGXFWIfTraceBufCtl->ui32HWPerfRIdx = ui32SrcRIdx;
			
			/* If all the data in the end of the array was copied, try copying
			 * wrapped data in the beginning of the array, assuming there is
			 * any and the RIdx was wrapped. */
			if (   (ui32BytesCopied == ui32BytesExp)
			    && (ui32SrcWIdx > 0) 
				&& (ui32SrcRIdx == 0) )
			{
				ui32BytesExp = ui32SrcWIdx;
#ifdef HWPERF_MISR_FUNC_DEBUG
				ui32BytesExpSum += ui32BytesExp;
#endif
				ui32BytesCopied = RGXHWPerfCopyDataL1toL2(psDevInfo->hHWPerfStream,
														  psHwPerfInfo,
														  ui32BytesExp);
				ui32BytesCopiedSum += ui32BytesCopied;
				/* Advance the FW buffer read position. */
				psRGXFWIfTraceBufCtl->ui32HWPerfRIdx = RGXHWPerfAdvanceRIdx(
						psDevInfo->ui32RGXFWIfHWPerfBufSize, ui32SrcRIdx,
						ui32BytesCopied);
			}
		}
#ifdef HWPERF_MISR_FUNC_DEBUG
		if (ui32BytesCopiedSum != ui32BytesExpSum)
		{
			PVR_DPF((PVR_DBG_WARNING, "RGXHWPerfDataStore: FW L1 RIdx:%u. Not all bytes copied to L2: %u bytes out of %u expected", psRGXFWIfTraceBufCtl->ui32HWPerfRIdx, ui32BytesCopiedSum, ui32BytesExpSum));
		}
#endif

	}
	else
	{
		PVR_DPF((PVR_DBG_VERBOSE, "RGXHWPerfDataStore NO EVENTS to transport"));
	}

	PVR_DPF_RETURN_VAL(ui32BytesCopiedSum);
}


PVRSRV_ERROR RGXHWPerfDataStoreCB(PVRSRV_DEVICE_NODE *psDevInfo)
{
	PVRSRV_ERROR		eError = PVRSRV_OK;
	PVRSRV_RGXDEV_INFO* psRgxDevInfo;
	IMG_UINT32          ui32BytesCopied;


	PVR_DPF_ENTERED;

	PVR_ASSERT(psDevInfo);
	psRgxDevInfo = psDevInfo->pvDevice;

	if (psRgxDevInfo->hHWPerfStream != 0)
	{
		OSLockAcquire(psRgxDevInfo->hLockHWPerfModule);
		ui32BytesCopied = RGXHWPerfDataStore(psRgxDevInfo);
		OSLockRelease(psRgxDevInfo->hLockHWPerfModule);

		if ( ui32BytesCopied )
		{	/* Signal consumers that packets may be available to read when
			 * running from a HW kick, not when called by client APP thread
			 * via the transport layer CB as this can lead to stream
			 * corruption.*/
			eError = TLStreamSync(psRgxDevInfo->hHWPerfStream);
			PVR_ASSERT(eError == PVRSRV_OK);
		}
		else
		{
			PVR_DPF((PVR_DBG_VERBOSE, "RGXHWPerfDataStoreCB: Zero bytes copied from FW L1 to L2."));
		}
	}

	PVR_DPF_RETURN_OK;
}


/* Not currently supported by default */
#if defined(SUPPORT_TL_PROODUCER_CALLBACK)
static PVRSRV_ERROR RGXHWPerfTLCB(IMG_HANDLE hStream,
		IMG_UINT32 ui32ReqOp, IMG_UINT32* ui32Resp, void* pvUser)
{
	PVRSRV_ERROR eError = PVRSRV_OK;
	PVRSRV_RGXDEV_INFO* psRgxDevInfo = (PVRSRV_RGXDEV_INFO*)pvUser;

	PVR_UNREFERENCED_PARAMETER(hStream);
	PVR_UNREFERENCED_PARAMETER(ui32Resp);

	PVR_ASSERT(psRgxDevInfo);

	switch (ui32ReqOp)
	{
	case TL_SOURCECB_OP_CLIENT_EOS:
		if (psRgxDevInfo->hHWPerfStream != 0)
		{
			OSLockAcquire(psRgxDevInfo->hLockHWPerfModule);
			(void) RGXHWPerfDataStore(psRgxDevInfo);
			OSLockRelease(psRgxDevInfo->hLockHWPerfModule);
		}
		break;

	default:
		break;
	}

	return eError;
}
#endif


/* References to key objects to allow kernel-side behaviour to function
 * e.g. FTrace and KM interface to HWPerf.
 */
static PVRSRV_DEVICE_NODE* gpsRgxDevNode = NULL;
static PVRSRV_RGXDEV_INFO* gpsRgxDevInfo = NULL;

static void RGXHWPerfL1BufferDeinit(void)
{
	if (gpsRgxDevInfo && gpsRgxDevInfo->psRGXFWIfHWPerfBufMemDesc)
	{
		if (gpsRgxDevInfo->psRGXFWIfHWPerfBuf != NULL)
		{
			DevmemReleaseCpuVirtAddr(gpsRgxDevInfo->psRGXFWIfHWPerfBufMemDesc);
			gpsRgxDevInfo->psRGXFWIfHWPerfBuf = NULL;
		}
		DevmemFwFree(gpsRgxDevInfo->psRGXFWIfHWPerfBufMemDesc);
		gpsRgxDevInfo->psRGXFWIfHWPerfBufMemDesc = NULL;
	}
}

/*************************************************************************/ /*!
@Function       RGXHWPerfInit

@Description    Called during driver init for initialization of HWPerf module
		in the Rogue device driver. This function keeps allocated
		only the minimal necessary resources, which are required for
		functioning of HWPerf server module.

@Input          psRgxDevInfo	RGX Device Node

@Return		PVRSRV_ERROR
*/ /**************************************************************************/
PVRSRV_ERROR RGXHWPerfInit(PVRSRV_DEVICE_NODE *psRgxDevNode)
{
	PVRSRV_ERROR eError;

	PVR_DPF_ENTERED;

	/* expecting a valid device node */
	PVR_ASSERT(psRgxDevNode);

	/* Keep RGX device's reference for later use as this parameter is
	 * optional on later calls to HWPerf server module */
	gpsRgxDevNode = psRgxDevNode;
	gpsRgxDevInfo = psRgxDevNode->pvDevice;

	/* Create a lock for HWPerf server module used for serializing, L1 to L2
	 * copy calls (e.g. in case of TL producer callback) and L1, L2 resource
	 * allocation */
	eError = OSLockCreate(&gpsRgxDevInfo->hLockHWPerfModule, LOCK_TYPE_PASSIVE);
	PVR_LOGR_IF_ERROR(eError, "OSLockCreate");

	/* avoid uninitialised data */
	gpsRgxDevInfo->hHWPerfStream = NULL;
	gpsRgxDevInfo->psRGXFWIfHWPerfBufMemDesc = NULL;

	PVR_DPF_RETURN_OK;
}

/*************************************************************************/ /*!
@Function       RGXHWPerfIsInitRequired

@Description    Returns true if the HWperf firmware buffer (L1 buffer) and host
		driver TL buffer (L2 buffer) are not already allocated. Caller
		should possess hLockHWPerfModule lock  before calling this
		function.

@Return		IMG_BOOL	Whether initialization (allocation) is required
*/ /**************************************************************************/
static INLINE IMG_BOOL RGXHWPerfIsInitRequired(void)
{
#if !defined (NO_HARDWARE)
	/* Both L1 and L2 buffers are required (for HWPerf functioning) on driver
	 * built for actual hardware (TC, EMU, etc.) */
	if (gpsRgxDevInfo->hHWPerfStream == NULL)
	{
		/* The allocation API (RGXHWPerfInitOnDemandResources) allocates
		 * device memory for both L1 and L2 without any checks. Hence,
		 * either both should be allocated or both be NULL.
		 *
		 * In-case this changes in future (for e.g. a situation where one
		 * of the 2 buffers is already allocated and other is required),
		 * add required checks before allocation calls to avoid memory leaks.
		 */
		PVR_ASSERT(gpsRgxDevInfo->psRGXFWIfHWPerfBufMemDesc == NULL);
		return IMG_TRUE;
	}
	PVR_ASSERT(gpsRgxDevInfo->psRGXFWIfHWPerfBufMemDesc != NULL);
#else
	/* On a NO-HW driver L2 is not allocated. So, no point in checking its
	 * allocation */
	if (gpsRgxDevInfo->psRGXFWIfHWPerfBufMemDesc == NULL)
	{
		return IMG_TRUE;
	}
#endif
	return IMG_FALSE;
}

/*************************************************************************/ /*!
@Function       RGXHWPerfInitOnDemandResources

@Description    This function allocates the HWperf firmware buffer (L1 buffer)
		and host driver TL buffer (L2 buffer) if HWPerf is enabled at
		driver load time. Otherwise, these buffers are allocated
		on-demand as and when required.

@Return		PVRSRV_ERROR
*/ /**************************************************************************/
PVRSRV_ERROR RGXHWPerfInitOnDemandResources(void)
{
	PVRSRV_ERROR eError;
	IMG_UINT32 ui32L2BufferSize;
	DEVMEM_FLAGS_T uiMemAllocFlags;

	PVR_DPF_ENTERED;

	/* Create the L1 HWPerf buffer on demand */
	uiMemAllocFlags = PVRSRV_MEMALLOCFLAG_DEVICE_FLAG(PMMETA_PROTECT)
				| PVRSRV_MEMALLOCFLAG_GPU_READABLE
				| PVRSRV_MEMALLOCFLAG_GPU_WRITEABLE
				| PVRSRV_MEMALLOCFLAG_CPU_READABLE
				| PVRSRV_MEMALLOCFLAG_KERNEL_CPU_MAPPABLE
				| PVRSRV_MEMALLOCFLAG_UNCACHED
				#if defined(PDUMP)
				| PVRSRV_MEMALLOCFLAG_ZERO_ON_ALLOC
				#endif
				;

	PMRLock();
	/* Allocate HWPerf FW L1 buffer */
	eError = DevmemFwAllocate(gpsRgxDevInfo,
							  gpsRgxDevInfo->ui32RGXFWIfHWPerfBufSize+RGXFW_HWPERF_L1_PADDING_DEFAULT,
							  uiMemAllocFlags,
							  "FwHWPerfBuffer",
							  &gpsRgxDevInfo->psRGXFWIfHWPerfBufMemDesc);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Failed to allocate kernel fw hwperf buffer (%u)",
					__FUNCTION__, eError));
		goto e0;
	}

	/* Expecting the RuntimeCfg structure is mapped into CPU virtual memory.
	 * Also, make sure the FW address is not already set */
	PVR_ASSERT(gpsRgxDevInfo->psRGXFWIfRuntimeCfg && gpsRgxDevInfo->psRGXFWIfRuntimeCfg->pui8HWPerfBuf.ui32Addr == 0x0);

	/* Meta cached flag removed from this allocation as it was found
	 * FW performance was better without it. */
	RGXSetFirmwareAddress(&gpsRgxDevInfo->psRGXFWIfRuntimeCfg->pui8HWPerfBuf,
						  gpsRgxDevInfo->psRGXFWIfHWPerfBufMemDesc,
						  0, RFW_FWADDR_NOREF_FLAG);

	eError = DevmemAcquireCpuVirtAddr(gpsRgxDevInfo->psRGXFWIfHWPerfBufMemDesc,
									  (void**)&gpsRgxDevInfo->psRGXFWIfHWPerfBuf);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Failed to acquire kernel hwperf buffer (%u)",
					 __FUNCTION__, eError));
		goto e0;
	}
	
	/* PDump the memory allocated */
	PDUMPCOMMENT("Dump rgxfw HW Perf Info structure");
	DevmemPDumpLoadMem (gpsRgxDevInfo->psRGXFWIfHWPerfBufMemDesc,
						0,
						gpsRgxDevInfo->ui32RGXFWIfHWPerfBufSize,
						PDUMP_FLAGS_CONTINUOUS);

	/* On NO-HW driver, there is no MISR installed to copy data from L1 to L2. Hence,
	 * L2 buffer is not allocated */
#if !defined(NO_HARDWARE)
	/* Host L2 HWPERF buffer size in bytes must be bigger than the L1 buffer
	 * accessed by the FW. The MISR may try to write one packet the size of the L1
	 * buffer in some scenarios. When logging is enabled in the MISR, it can be seen
	 * if the L2 buffer hits a full condition. The closer in size the L2 and L1 buffers
	 * are the more chance of this happening.
	 * Size chosen to allow MISR to write an L1 sized packet and for the client
	 * application/daemon to drain a L1 sized packet e.g. ~ 2xL1+64 working space.
	 */
	ui32L2BufferSize = gpsRgxDevInfo->ui32RGXFWIfHWPerfBufSize<<1;
	eError = TLStreamCreate(&gpsRgxDevInfo->hHWPerfStream, HWPERF_TL_STREAM_NAME,
					ui32L2BufferSize+RGXFW_HWPERF_L1_PADDING_DEFAULT,
					TL_FLAG_RESERVE_DROP_NEWER | TL_FLAG_NO_SIGNAL_ON_COMMIT,
#if !defined(SUPPORT_TL_PROODUCER_CALLBACK)
					NULL, NULL
#else
					/* Not enabled  by default */
					RGXHWPerfTLCB, gpsRgxDevInfo
#endif
					);
	PVR_LOGG_IF_ERROR(eError, "TLStreamCreate", e1);
#else /* defined (NO_HARDWARE) */
	PVR_UNREFERENCED_PARAMETER(ui32L2BufferSize);
	PVR_UNREFERENCED_PARAMETER(RGXHWPerfTLCB);
#endif 

	PMRUnlock();

	PVR_DPF_RETURN_OK;

#if !defined(NO_HARDWARE)
e1: /* L2 buffer initialisation failures */
	gpsRgxDevInfo->hHWPerfStream = NULL;
#endif
e0: /* L1 buffer initialisation failures */
	RGXHWPerfL1BufferDeinit();
	
	PMRUnlock();
	PVR_DPF_RETURN_RC(eError);
}


void RGXHWPerfDeinit(void)
{
	PVR_DPF_ENTERED;

	/* Clean up the L2 buffer stream object if allocated */
	if (gpsRgxDevInfo && gpsRgxDevInfo->hHWPerfStream)
	{
		TLStreamClose(gpsRgxDevInfo->hHWPerfStream);
		gpsRgxDevInfo->hHWPerfStream = NULL;
	}

	/* Cleanup L1 buffer resources */
	RGXHWPerfL1BufferDeinit();

	/* Cleanup the HWPerf server module lock resource */
	if (gpsRgxDevInfo && gpsRgxDevInfo->hLockHWPerfModule)
	{
		OSLockDestroy(gpsRgxDevInfo->hLockHWPerfModule);
		gpsRgxDevInfo->hLockHWPerfModule = NULL;
	}

	/* Clear global RGX device reference */
	gpsRgxDevInfo = NULL;
	gpsRgxDevNode = NULL;

	PVR_DPF_RETURN;
}


/******************************************************************************
 * RGX HW Performance Profiling Server API(s)
 *****************************************************************************/

static PVRSRV_ERROR RGXHWPerfCtrlFwBuffer(PVRSRV_DEVICE_NODE *psDeviceNode,
                                          IMG_BOOL bToggle,
                                          IMG_UINT64 ui64Mask)
{
	PVRSRV_ERROR eError = PVRSRV_OK;
	PVRSRV_RGXDEV_INFO* psDevice = psDeviceNode->pvDevice;
	RGXFWIF_KCCB_CMD sKccbCmd;

	OSLockAcquire(psDevice->hLockHWPerfModule);
	/* If this method is being used whether to enable or disable
	 * then the hwperf buffers (host and FW) are likely to be needed
	 * eventually so create them, also helps unit testing. Buffers
	 * allocated on demand to reduce RAM foot print on systems not
	 * needing HWPerf resources. */
	if (RGXHWPerfIsInitRequired())
	{
		eError = RGXHWPerfInitOnDemandResources();
		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR, "%s: Initialisation of on-demand HWPerfGpu "
			        "resources failed", __func__));
			goto unlock_and_return;
		}
	}

	/* Return if the filter is the same */
	if (!bToggle && gpsRgxDevInfo->ui64HWPerfFilter == ui64Mask)
		goto unlock_and_return;

	/* Prepare command parameters ... */
	sKccbCmd.eCmdType = RGXFWIF_KCCB_CMD_HWPERF_UPDATE_CONFIG;
	sKccbCmd.uCmdData.sHWPerfCtrl.bToggle = bToggle;
	sKccbCmd.uCmdData.sHWPerfCtrl.ui64Mask = ui64Mask;

	/* Ask the FW to carry out the HWPerf configuration command */
	eError = RGXScheduleCommand(psDeviceNode->pvDevice,	RGXFWIF_DM_GP, 
								&sKccbCmd, sizeof(sKccbCmd), IMG_TRUE);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Failed to set new HWPerfGpu filter in "
				"firmware (error = %d)", __func__, eError));
		goto unlock_and_return;
	}

	gpsRgxDevInfo->ui64HWPerfFilter = bToggle ?
	        gpsRgxDevInfo->ui64HWPerfFilter ^ ui64Mask : ui64Mask;

	OSLockRelease(psDevice->hLockHWPerfModule);

	/* Wait for FW to complete */
	eError = RGXWaitForFWOp(psDeviceNode->pvDevice, RGXFWIF_DM_GP,
	                        psDeviceNode->psSyncPrim, IMG_TRUE);
	PVR_LOGG_IF_ERROR(eError, "RGXWaitForFWOp", return_);

#if defined(DEBUG)
	if (bToggle)
	{
		PVR_DPF((PVR_DBG_WARNING, "HWPerfGpu events (%llx) have been TOGGLED",
		        ui64Mask));
	}
	else
	{
		PVR_DPF((PVR_DBG_WARNING, "HWPerfGpu mask has been SET to (%llx)",
		        ui64Mask));
	}
#endif

	return PVRSRV_OK;

unlock_and_return:
	OSLockRelease(psDevice->hLockHWPerfModule);

return_:
	return eError;
}

static PVRSRV_ERROR RGXHWPerfCtrlHostBuffer(PVRSRV_DEVICE_NODE *psDeviceNode,
                                            IMG_BOOL bToggle,
                                            IMG_UINT32 ui32Mask)
{
	PVRSRV_ERROR eError = PVRSRV_OK;
	PVRSRV_RGXDEV_INFO* psDevice = psDeviceNode->pvDevice;

	OSLockAcquire(psDevice->hLockHWPerfHostStream);
	if (psDevice->hHWPerfHostStream == NULL)
	{
		eError = RGXHWPerfHostInitOnDemandResources();
		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR, "%s: Initialization of on-demand HWPerfHost"
			        " resources failed", __FUNCTION__));
			OSLockRelease(psDevice->hLockHWPerfHostStream);
			return eError;
		}
	}

	psDevice->ui32HWPerfHostFilter = bToggle ?
	        psDevice->ui32HWPerfHostFilter ^ ui32Mask : ui32Mask;
	OSLockRelease(psDevice->hLockHWPerfHostStream);

#if defined(DEBUG)
	if (bToggle)
	{
		PVR_DPF((PVR_DBG_WARNING, "HWPerfHost events (%x) have been TOGGLED",
		        ui32Mask));
	}
	else
	{
		PVR_DPF((PVR_DBG_WARNING, "HWPerfHost mask has been SET to (%x)",
		        ui32Mask));
	}
#endif

	return PVRSRV_OK;
}

/*
	PVRSRVRGXCtrlHWPerfKM
*/
PVRSRV_ERROR PVRSRVRGXCtrlHWPerfKM(
	CONNECTION_DATA         *psConnection,
	PVRSRV_DEVICE_NODE      *psDeviceNode,
	RGX_HWPERF_STREAM_ID     eStreamId,
	IMG_BOOL                 bToggle,
	IMG_UINT64               ui64Mask)
{
	PVR_UNREFERENCED_PARAMETER(psConnection);

	PVR_DPF_ENTERED;
	PVR_ASSERT(psDeviceNode);

	if (eStreamId == RGX_HWPERF_STREAM_ID0_FW)
	{
		return RGXHWPerfCtrlFwBuffer(psDeviceNode, bToggle, ui64Mask);
	}
	else if (eStreamId == RGX_HWPERF_STREAM_ID1_HOST)
	{
		return RGXHWPerfCtrlHostBuffer(psDeviceNode, bToggle, (IMG_UINT32) ui64Mask);
	}
	else
	{
		PVR_DPF((PVR_DBG_ERROR, "PVRSRVRGXCtrlHWPerfKM: Unknown stream id."));
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	PVR_DPF_RETURN_OK;
}


/*
	PVRSRVRGXEnableHWPerfCountersKM
*/
PVRSRV_ERROR PVRSRVRGXConfigEnableHWPerfCountersKM(
	CONNECTION_DATA          * psConnection,
	PVRSRV_DEVICE_NODE       * psDeviceNode,
	IMG_UINT32                 ui32ArrayLen,
	RGX_HWPERF_CONFIG_CNTBLK * psBlockConfigs)
{
	PVRSRV_ERROR 		eError = PVRSRV_OK;
	RGXFWIF_KCCB_CMD 	sKccbCmd;
	DEVMEM_MEMDESC*		psFwBlkConfigsMemDesc;
	RGX_HWPERF_CONFIG_CNTBLK* psFwArray;

	PVR_UNREFERENCED_PARAMETER(psConnection);

	PVR_DPF_ENTERED;

	PVR_ASSERT(psDeviceNode);
	PVR_ASSERT(ui32ArrayLen>0);
	PVR_ASSERT(psBlockConfigs);

	/* Fill in the command structure with the parameters needed
	 */
	sKccbCmd.eCmdType = RGXFWIF_KCCB_CMD_HWPERF_CONFIG_ENABLE_BLKS;
	sKccbCmd.uCmdData.sHWPerfCfgEnableBlks.ui32NumBlocks = ui32ArrayLen;

	eError = DevmemFwAllocate(psDeviceNode->pvDevice,
			sizeof(RGX_HWPERF_CONFIG_CNTBLK)*ui32ArrayLen, 
			PVRSRV_MEMALLOCFLAG_DEVICE_FLAG(PMMETA_PROTECT) |
									  PVRSRV_MEMALLOCFLAG_GPU_READABLE | 
									  PVRSRV_MEMALLOCFLAG_GPU_WRITEABLE |
									  PVRSRV_MEMALLOCFLAG_CPU_READABLE |
									  PVRSRV_MEMALLOCFLAG_KERNEL_CPU_MAPPABLE | 
									  PVRSRV_MEMALLOCFLAG_UNCACHED |
									  PVRSRV_MEMALLOCFLAG_ZERO_ON_ALLOC,
			"FwHWPerfCountersConfigBlock",
			&psFwBlkConfigsMemDesc);
	if (eError != PVRSRV_OK)
		PVR_LOGR_IF_ERROR(eError, "DevmemFwAllocate");

	RGXSetFirmwareAddress(&sKccbCmd.uCmdData.sHWPerfCfgEnableBlks.pasBlockConfigs,
			psFwBlkConfigsMemDesc, 0, 0);

	eError = DevmemAcquireCpuVirtAddr(psFwBlkConfigsMemDesc, (void **)&psFwArray);
	if (eError != PVRSRV_OK)
	{
		PVR_LOGG_IF_ERROR(eError, "DevmemAcquireCpuVirtAddr", fail1);
	}

	OSMemCopy(psFwArray, psBlockConfigs, sizeof(RGX_HWPERF_CONFIG_CNTBLK)*ui32ArrayLen);
	DevmemPDumpLoadMem(psFwBlkConfigsMemDesc,
						0,
						sizeof(RGX_HWPERF_CONFIG_CNTBLK)*ui32ArrayLen,
						0);

	/* PVR_DPF((PVR_DBG_VERBOSE, "PVRSRVRGXConfigEnableHWPerfCountersKM parameters set, calling FW")); */

	/* Ask the FW to carry out the HWPerf configuration command
	 */
	eError = RGXScheduleCommand(psDeviceNode->pvDevice,
			RGXFWIF_DM_GP, &sKccbCmd, sizeof(sKccbCmd), IMG_TRUE);
	if (eError != PVRSRV_OK)
	{
		PVR_LOGG_IF_ERROR(eError, "RGXScheduleCommand", fail2);
	}

	/* PVR_DPF((PVR_DBG_VERBOSE, "PVRSRVRGXConfigEnableHWPerfCountersKM command scheduled for FW")); */

	/* Wait for FW to complete */
	eError = RGXWaitForFWOp(psDeviceNode->pvDevice, RGXFWIF_DM_GP, psDeviceNode->psSyncPrim, IMG_TRUE);
	if (eError != PVRSRV_OK)
	{
		PVR_LOGG_IF_ERROR(eError, "RGXWaitForFWOp", fail2);
	}

	/* Release temporary memory used for block configuration
	 */
	RGXUnsetFirmwareAddress(psFwBlkConfigsMemDesc);
	DevmemReleaseCpuVirtAddr(psFwBlkConfigsMemDesc);
	DevmemFwFree(psFwBlkConfigsMemDesc);

	/* PVR_DPF((PVR_DBG_VERBOSE, "PVRSRVRGXConfigEnableHWPerfCountersKM firmware completed")); */

	PVR_DPF((PVR_DBG_WARNING, "HWPerf %d counter blocks configured and ENABLED",  ui32ArrayLen));

	PVR_DPF_RETURN_OK;

fail2:
	DevmemReleaseCpuVirtAddr(psFwBlkConfigsMemDesc);
fail1:
	RGXUnsetFirmwareAddress(psFwBlkConfigsMemDesc);
	DevmemFwFree(psFwBlkConfigsMemDesc);

	PVR_DPF_RETURN_RC(eError);
}


/*
	PVRSRVRGXConfigCustomCountersReadingHWPerfKM
 */
PVRSRV_ERROR PVRSRVRGXConfigCustomCountersKM(
	CONNECTION_DATA             * psConnection,
	PVRSRV_DEVICE_NODE          * psDeviceNode,
	IMG_UINT16                    ui16CustomBlockID,
	IMG_UINT16                    ui16NumCustomCounters,
	IMG_UINT32                  * pui32CustomCounterIDs)
{
	PVRSRV_ERROR		eError = PVRSRV_OK;
	RGXFWIF_KCCB_CMD	sKccbCmd;
	DEVMEM_MEMDESC*		psFwSelectCntrsMemDesc = NULL;
	IMG_UINT32*			psFwArray;

	PVR_UNREFERENCED_PARAMETER(psConnection);

	PVR_DPF_ENTERED;

	PVR_ASSERT(psDeviceNode);

	PVR_DPF((PVR_DBG_MESSAGE, "PVRSRVRGXSelectCustomCountersKM: configure block %u to read %u counters", ui16CustomBlockID, ui16NumCustomCounters));

	/* Fill in the command structure with the parameters needed */
	sKccbCmd.eCmdType = RGXFWIF_KCCB_CMD_HWPERF_SELECT_CUSTOM_CNTRS;
	sKccbCmd.uCmdData.sHWPerfSelectCstmCntrs.ui16NumCounters = ui16NumCustomCounters;
	sKccbCmd.uCmdData.sHWPerfSelectCstmCntrs.ui16CustomBlock = ui16CustomBlockID;

	if (ui16NumCustomCounters > 0)
	{
		PVR_ASSERT(pui32CustomCounterIDs);

		eError = DevmemFwAllocate(psDeviceNode->pvDevice,
				sizeof(IMG_UINT32) * ui16NumCustomCounters,
				PVRSRV_MEMALLOCFLAG_DEVICE_FLAG(PMMETA_PROTECT) |
				PVRSRV_MEMALLOCFLAG_GPU_READABLE |
				PVRSRV_MEMALLOCFLAG_GPU_WRITEABLE |
				PVRSRV_MEMALLOCFLAG_CPU_READABLE |
				PVRSRV_MEMALLOCFLAG_KERNEL_CPU_MAPPABLE |
				PVRSRV_MEMALLOCFLAG_UNCACHED |
				PVRSRV_MEMALLOCFLAG_ZERO_ON_ALLOC,
				"FwHWPerfConfigCustomCounters",
				&psFwSelectCntrsMemDesc);
		if (eError != PVRSRV_OK)
			PVR_LOGR_IF_ERROR(eError, "DevmemFwAllocate");

		RGXSetFirmwareAddress(&sKccbCmd.uCmdData.sHWPerfSelectCstmCntrs.pui32CustomCounterIDs,
				psFwSelectCntrsMemDesc, 0, 0);

		eError = DevmemAcquireCpuVirtAddr(psFwSelectCntrsMemDesc, (void **)&psFwArray);
		if (eError != PVRSRV_OK)
		{
			PVR_LOGG_IF_ERROR(eError, "DevmemAcquireCpuVirtAddr", fail1);
		}

		OSMemCopy(psFwArray, pui32CustomCounterIDs, sizeof(IMG_UINT32) * ui16NumCustomCounters);
		DevmemPDumpLoadMem(psFwSelectCntrsMemDesc,
				0,
				sizeof(IMG_UINT32) * ui16NumCustomCounters,
				0);
	}

	/* Push in the KCCB the command to configure the custom counters block */
	eError = RGXScheduleCommand(psDeviceNode->pvDevice,
			RGXFWIF_DM_GP, &sKccbCmd, sizeof(sKccbCmd), IMG_TRUE);
	if (eError != PVRSRV_OK)
	{
		PVR_LOGG_IF_ERROR(eError, "RGXScheduleCommand", fail2);
	}
	PVR_DPF((PVR_DBG_VERBOSE, "PVRSRVRGXSelectCustomCountersKM: Command scheduled"));

	/* Wait for FW to complete */
	eError = RGXWaitForFWOp(psDeviceNode->pvDevice, RGXFWIF_DM_GP, psDeviceNode->psSyncPrim, IMG_TRUE);
	if (eError != PVRSRV_OK)
	{
		PVR_LOGG_IF_ERROR(eError, "RGXWaitForFWOp", fail2);
	}
	PVR_DPF((PVR_DBG_VERBOSE, "PVRSRVRGXSelectCustomCountersKM: FW operation completed"));

	if (ui16NumCustomCounters > 0)
	{
		/* Release temporary memory used for block configuration */
		RGXUnsetFirmwareAddress(psFwSelectCntrsMemDesc);
		DevmemReleaseCpuVirtAddr(psFwSelectCntrsMemDesc);
		DevmemFwFree(psFwSelectCntrsMemDesc);
	}

	PVR_DPF((PVR_DBG_MESSAGE, "HWPerf custom counters %u reading will be sent with the next HW events", ui16NumCustomCounters));

	PVR_DPF_RETURN_OK;

	fail2:
	if (psFwSelectCntrsMemDesc) DevmemReleaseCpuVirtAddr(psFwSelectCntrsMemDesc);

	fail1:
	if (psFwSelectCntrsMemDesc) 
	{
		RGXUnsetFirmwareAddress(psFwSelectCntrsMemDesc);
		DevmemFwFree(psFwSelectCntrsMemDesc);
	}
	
	PVR_DPF_RETURN_RC(eError);
}
/*
	PVRSRVRGXDisableHWPerfcountersKM
*/
PVRSRV_ERROR PVRSRVRGXCtrlHWPerfCountersKM(
	CONNECTION_DATA             * psConnection,
	PVRSRV_DEVICE_NODE          * psDeviceNode,
	IMG_BOOL                      bEnable,
	IMG_UINT32                    ui32ArrayLen,
	IMG_UINT16                  * psBlockIDs)
{
	PVRSRV_ERROR 		eError = PVRSRV_OK;
	RGXFWIF_KCCB_CMD 	sKccbCmd;

	PVR_UNREFERENCED_PARAMETER(psConnection);

	PVR_DPF_ENTERED;

	PVR_ASSERT(psDeviceNode);
	PVR_ASSERT(ui32ArrayLen>0);
	PVR_ASSERT(ui32ArrayLen<=RGXFWIF_HWPERF_CTRL_BLKS_MAX);
	PVR_ASSERT(psBlockIDs);

	/* Fill in the command structure with the parameters needed
	 */
	sKccbCmd.eCmdType = RGXFWIF_KCCB_CMD_HWPERF_CTRL_BLKS;
	sKccbCmd.uCmdData.sHWPerfCtrlBlks.bEnable = bEnable;
	sKccbCmd.uCmdData.sHWPerfCtrlBlks.ui32NumBlocks = ui32ArrayLen;
	OSMemCopy(sKccbCmd.uCmdData.sHWPerfCtrlBlks.aeBlockIDs, psBlockIDs, sizeof(IMG_UINT16)*ui32ArrayLen);

	/* PVR_DPF((PVR_DBG_VERBOSE, "PVRSRVRGXCtrlHWPerfCountersKM parameters set, calling FW")); */

	/* Ask the FW to carry out the HWPerf configuration command
	 */
	eError = RGXScheduleCommand(psDeviceNode->pvDevice,
			RGXFWIF_DM_GP, &sKccbCmd, sizeof(sKccbCmd), IMG_TRUE);
	if (eError != PVRSRV_OK)
		PVR_LOGR_IF_ERROR(eError, "RGXScheduleCommand");

	/* PVR_DPF((PVR_DBG_VERBOSE, "PVRSRVRGXCtrlHWPerfCountersKM command scheduled for FW")); */

	/* Wait for FW to complete */
	eError = RGXWaitForFWOp(psDeviceNode->pvDevice, RGXFWIF_DM_GP, psDeviceNode->psSyncPrim, IMG_TRUE);
	if (eError != PVRSRV_OK)
		PVR_LOGR_IF_ERROR(eError, "RGXWaitForFWOp");

	/* PVR_DPF((PVR_DBG_VERBOSE, "PVRSRVRGXCtrlHWPerfCountersKM firmware completed")); */

#if defined(DEBUG)
	if (bEnable)
		PVR_DPF((PVR_DBG_WARNING, "HWPerf %d counter blocks have been ENABLED",  ui32ArrayLen));
	else
		PVR_DPF((PVR_DBG_WARNING, "HWPerf %d counter blocks have been DISABLED",  ui32ArrayLen));
#endif

	PVR_DPF_RETURN_OK;
}

/******************************************************************************
 * RGX HW Performance Host Stream API
 *****************************************************************************/

/*************************************************************************/ /*!
@Function       RGXHWPerfHostInit

@Description    Called during driver init for initialisation of HWPerfHost
                stream in the Rogue device driver. This function keeps allocated
                only the minimal necessary resources, which are required for
                functioning of HWPerf server module.

@Return         PVRSRV_ERROR
*/ /**************************************************************************/
PVRSRV_ERROR RGXHWPerfHostInit(void)
{
	PVRSRV_ERROR eError;
	PVR_ASSERT(gpsRgxDevInfo != NULL);
	
	eError = OSLockCreate(&gpsRgxDevInfo->hLockHWPerfHostStream, LOCK_TYPE_PASSIVE);
	PVR_LOGG_IF_ERROR(eError, "OSLockCreate", error);

	gpsRgxDevInfo->hHWPerfHostStream = NULL;
	gpsRgxDevInfo->bHWPerfHostEnabled = IMG_FALSE;
	gpsRgxDevInfo->ui32HWPerfHostFilter = 0; /* disable all events */
	gpsRgxDevInfo->ui32HWPerfHostNextOrdinal = 0;

error:
	return eError;
}

/*************************************************************************/ /*!
@Function       RGXHWPerfHostInitOnDemandResources

@Description    This function allocates the HWPerfHost buffer if HWPerf is
                enabled at driver load time. Otherwise, these buffers are
                allocated on-demand as and when required.

@Return         PVRSRV_ERROR
*/ /**************************************************************************/
PVRSRV_ERROR RGXHWPerfHostInitOnDemandResources(void)
{
	PVRSRV_ERROR eError;

	eError = TLStreamCreate(&gpsRgxDevInfo->hHWPerfHostStream,
	        HWPERF_HOST_TL_STREAM_NAME, HWPERF_HOST_TL_STREAM_SIZE,
	        TL_FLAG_RESERVE_DROP_NEWER, NULL, NULL);
	PVR_LOGG_IF_ERROR(eError, "TLStreamCreate", error_stream_create);

	gpsRgxDevInfo->bHWPerfHostEnabled = IMG_TRUE;

	return PVRSRV_OK;

error_stream_create:
	OSLockDestroy(gpsRgxDevInfo->hLockHWPerfHostStream);
	gpsRgxDevInfo->hLockHWPerfHostStream = NULL;

	return eError;
}

void RGXHWPerfHostDeInit(void)
{
	if (gpsRgxDevInfo && gpsRgxDevInfo->hHWPerfHostStream)
	{
		TLStreamClose(gpsRgxDevInfo->hHWPerfHostStream);
		gpsRgxDevInfo->hHWPerfHostStream = NULL;
	}

	if (gpsRgxDevInfo && gpsRgxDevInfo->hLockHWPerfHostStream)
	{
		OSLockDestroy(gpsRgxDevInfo->hLockHWPerfHostStream);
		gpsRgxDevInfo->hLockHWPerfHostStream = NULL;
	}

	if (gpsRgxDevInfo)
	{
		gpsRgxDevInfo->bHWPerfHostEnabled = IMG_FALSE;
	}
}

void RGXHWPerfHostSetEventFilter(IMG_UINT32 ui32Filter)
{
	gpsRgxDevInfo->ui32HWPerfHostFilter = ui32Filter;
}

void RGXHWPerfHostPostEnqEvent(const RGX_HWPERF_KICK_TYPE ui32EnqType,
                               const IMG_UINT32 ui32Pid,
                               const IMG_UINT32 ui32FWCtx,
                               const IMG_UINT32 ui32ExtJobRef,
                               const IMG_UINT32 ui32IntJobRef)
{
	RGX_HWPERF_V2_PACKET_HDR *psHeader;
	RGX_HWPERF_HOST_ENQ_DATA *psData;
	const size_t uiSize = RGX_HWPERF_MAKE_SIZE_FIXED(RGX_HWPERF_HOST_ENQ_DATA);
	IMG_UINT8 *pui8Dest;
	PVRSRV_ERROR eError;

	PVR_ASSERT(gpsRgxDevInfo->hLockHWPerfHostStream != NULL);
	PVR_ASSERT(gpsRgxDevInfo->hHWPerfHostStream != NULL);

	OSLockAcquire(gpsRgxDevInfo->hLockHWPerfHostStream);

	/* In case we drop packet we increment ordinal beforehand. */
	gpsRgxDevInfo->ui32HWPerfHostNextOrdinal++;

	eError = TLStreamReserve(gpsRgxDevInfo->hHWPerfHostStream, &pui8Dest, uiSize);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_MESSAGE, "%s: Could not reserve space in %s buffer"
		        " (%d). Dropping packet.",
		        __func__, HWPERF_HOST_TL_STREAM_NAME, eError));
		goto cleanup;
	}
	PVR_ASSERT(pui8Dest != NULL);

	psHeader = (RGX_HWPERF_V2_PACKET_HDR *) pui8Dest;
	psHeader->ui32Ordinal = gpsRgxDevInfo->ui32HWPerfHostNextOrdinal;
	psHeader->ui64Timestamp = OSClockus64();
	psHeader->ui32Sig = HWPERF_PACKET_V2B_SIG;
	psHeader->eTypeId = RGX_HWPERF_MAKE_TYPEID(RGX_HWPERF_STREAM_ID1_HOST,
	        RGX_HWPERF_HOST_ENQ, 0);
	psHeader->ui32Size = RGX_HWPERF_MAKE_SIZE_FIXED(RGX_HWPERF_HOST_ENQ_DATA);

	psData = (RGX_HWPERF_HOST_ENQ_DATA *) (psHeader + 1);
	psData->ui32EnqType = ui32EnqType;
	psData->ui32PID = ui32Pid;
	psData->ui32FWCtx = ui32FWCtx;
	psData->ui32ExtJobRef = ui32ExtJobRef;
	psData->ui32IntJobRef = ui32IntJobRef;

	eError = TLStreamCommit(gpsRgxDevInfo->hHWPerfHostStream, uiSize);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_MESSAGE, "%s: Could not commit data to %s"
	            " (%d)", __func__, HWPERF_HOST_TL_STREAM_NAME, eError));
	}

cleanup:
	OSLockRelease(gpsRgxDevInfo->hLockHWPerfHostStream);
}

/******************************************************************************
 * SUPPORT_GPUTRACE_EVENTS
 *
 * Currently only implemented on Linux and Android. Feature can be enabled on
 * Android builds but can also be enabled on Linux builds for testing
 * but requires the gpu.h FTrace event header file to be present.
 *****************************************************************************/
#if defined(SUPPORT_GPUTRACE_EVENTS)


static POS_LOCK g_hFTraceLock;

/* This lock ensures that the reference counting operation on the FTrace UFO
 * events and enable/disable operation on firmware event are performed as
 * one atomic operation. This should ensure that there are no race conditions
 * between reference counting and firmware event state change.
 * See below comment for g_uiUfoEventRef.
 */
static POS_LOCK g_hLockFTraceEventLock;

/* Multiple FTrace UFO events are reflected in the firmware as only one event. When
 * we enable FTrace UFO event we want to also at the same time enable it in
 * the firmware. Since there is a multiple-to-one relation between those events
 * we count how many FTrace UFO events is enabled. If at least one event is
 * enabled we enabled the firmware event. When all FTrace UFO events are disabled
 * we disable firmware event. */
static IMG_INT g_uiUfoEventRef = 0;

static void RGXHWPerfFTraceCmdCompleteNotify(PVRSRV_CMDCOMP_HANDLE);

static PVRSRV_ERROR RGXHWPerfFTraceGPUEnable(void)
{
	PVRSRV_ERROR eError = PVRSRV_OK;
	IMG_UINT64 ui64UFOFilter = RGX_HWPERF_EVENT_MASK_VALUE(RGX_HWPERF_UFO) &
	                           gpsRgxDevInfo->ui64HWPerfFilter;

	PVR_DPF_ENTERED;

	PVR_ASSERT(gpsRgxDevNode && gpsRgxDevInfo);

	/* In the case where the AppHint has not been set we need to
	 * initialise the host driver HWPerf resources here. Allocated on
	 * demand to reduce RAM foot print on systems not needing HWPerf.
	 * Signal FW to enable event generation.
	 */
	if (gpsRgxDevNode->psSyncPrim)
	{
		eError = PVRSRVRGXCtrlHWPerfKM(NULL, gpsRgxDevNode, IMG_FALSE,
		                               RGX_HWPERF_STREAM_ID0_FW,
		                               RGX_HWPERF_EVENT_MASK_HW_KICKFINISH |
		                               ui64UFOFilter);
		PVR_LOGG_IF_ERROR(eError, "PVRSRVRGXCtrlHWPerfKM", err_out);
	}

	/* Open the TL Stream for HWPerf data consumption */
	eError = TLClientOpenStream(DIRECT_BRIDGE_HANDLE,
								HWPERF_TL_STREAM_NAME,
								PVRSRV_STREAM_FLAG_ACQUIRE_NONBLOCKING,
								&gpsRgxDevInfo->hGPUTraceTLStream);
	PVR_LOGG_IF_ERROR(eError, "TLClientOpenStream", err_out);

	/* Register a notifier to collect HWPerf data whenever the HW completes
	 * an operation.
	 */
	eError = PVRSRVRegisterCmdCompleteNotify(
		&gpsRgxDevInfo->hGPUTraceCmdCompleteHandle,
		&RGXHWPerfFTraceCmdCompleteNotify,
		gpsRgxDevInfo);
	PVR_LOGG_IF_ERROR(eError, "PVRSRVRegisterCmdCompleteNotify", err_close_stream);

	/* Reset the OS timestamp coming from the timer correlation data
	 * associated with the latest HWPerf event we processed.
	 */
	gpsRgxDevInfo->ui64LastSampledTimeCorrOSTimeStamp = 0;

	gpsRgxDevInfo->bFTraceGPUEventsEnabled = IMG_TRUE;

err_out:
	PVR_DPF_RETURN_RC(eError);

err_close_stream:
	TLClientCloseStream(DIRECT_BRIDGE_HANDLE,
						gpsRgxDevInfo->hGPUTraceTLStream);
	goto err_out;
}

static PVRSRV_ERROR RGXHWPerfFTraceGPUDisable(IMG_BOOL bDeInit)
{
	PVRSRV_ERROR eError = PVRSRV_OK;

	PVR_DPF_ENTERED;

	PVR_ASSERT(gpsRgxDevNode && gpsRgxDevInfo);

	OSLockAcquire(g_hFTraceLock);

	if (!bDeInit)
	{
		eError = PVRSRVRGXCtrlHWPerfKM(NULL, gpsRgxDevNode, RGX_HWPERF_STREAM_ID0_FW, IMG_FALSE, (RGX_HWPERF_EVENT_MASK_NONE));
		PVR_LOG_IF_ERROR(eError, "PVRSRVRGXCtrlHWPerfKM");
	}


	if (gpsRgxDevInfo->hGPUTraceCmdCompleteHandle)
	{
		/* Tracing is being turned off. Unregister the notifier. */
		eError = PVRSRVUnregisterCmdCompleteNotify(
				gpsRgxDevInfo->hGPUTraceCmdCompleteHandle);
		PVR_LOG_IF_ERROR(eError, "PVRSRVUnregisterCmdCompleteNotify");
		gpsRgxDevInfo->hGPUTraceCmdCompleteHandle = NULL;
	}

	if (gpsRgxDevInfo->hGPUTraceTLStream)
	{
		IMG_PBYTE pbTmp = NULL;
		IMG_UINT32 ui32Tmp = 0;

		/* We have to flush both the L1 (FW) and L2 (Host) buffers in case there
		 * are some events left unprocessed in this FTrace/systrace "session"
		 * (note that even if we have just disabled HWPerf on the FW some packets
		 * could have been generated and already copied to L2 by the MISR handler).
		 *
		 * With the following calls we will both copy new data to the Host buffer
		 * (done by the producer callback in TLClientAcquireData) and advance
		 * the read offset in the buffer to catch up with the latest events.
		 */
		eError = TLClientAcquireData(DIRECT_BRIDGE_HANDLE,
		                             gpsRgxDevInfo->hGPUTraceTLStream,
		                             &pbTmp, &ui32Tmp);
		PVR_LOG_IF_ERROR(eError, "TLClientCloseStream");

		/* Let close stream perform the release data on the outstanding acquired data */
		eError = TLClientCloseStream(DIRECT_BRIDGE_HANDLE,
		                             gpsRgxDevInfo->hGPUTraceTLStream);
		PVR_LOG_IF_ERROR(eError, "TLClientCloseStream");

		gpsRgxDevInfo->hGPUTraceTLStream = NULL;
	}

	gpsRgxDevInfo->bFTraceGPUEventsEnabled = IMG_FALSE;

	OSLockRelease(g_hFTraceLock);

	PVR_DPF_RETURN_RC(eError);
}

PVRSRV_ERROR RGXHWPerfFTraceGPUEventsEnabledSet(IMG_BOOL bNewValue)
{
	IMG_BOOL bOldValue;
	PVRSRV_ERROR eError = PVRSRV_OK;

	PVR_DPF_ENTERED;

	if (!gpsRgxDevInfo)
	{
		/* RGXHWPerfFTraceGPUInit hasn't been called yet -- it's too early
		 * to enable tracing.
		 */
		eError = PVRSRV_ERROR_NO_DEVICEDATA_FOUND;
		PVR_DPF_RETURN_RC(eError);
	}

	bOldValue = gpsRgxDevInfo->bFTraceGPUEventsEnabled;

	if (bOldValue != bNewValue)
	{
		if (bNewValue)
		{
			eError = RGXHWPerfFTraceGPUEnable();
		}
		else
		{
			eError = RGXHWPerfFTraceGPUDisable(IMG_FALSE);
		}
	}

	PVR_DPF_RETURN_RC(eError);
}

PVRSRV_ERROR PVRGpuTraceEnabledSet(IMG_BOOL bNewValue)
{
	PVRSRV_ERROR eError = PVRSRV_OK;

	/* Lock down because we need to protect
	 * RGXHWPerfFTraceGPUDisable()/RGXHWPerfFTraceGPUEnable()
	 */
	OSAcquireBridgeLock();
    eError = RGXHWPerfFTraceGPUEventsEnabledSet(bNewValue);
	OSReleaseBridgeLock();

	PVR_DPF_RETURN_RC(eError);
}

IMG_BOOL RGXHWPerfFTraceGPUEventsEnabled(void)
{
	return(gpsRgxDevInfo->bFTraceGPUEventsEnabled);
}

IMG_BOOL PVRGpuTraceEnabled(void)
{
	return (RGXHWPerfFTraceGPUEventsEnabled());
}

void RGXHWPerfFTraceGPUEnqueueEvent(PVRSRV_RGXDEV_INFO *psDevInfo,
		IMG_UINT32 ui32ExternalJobRef, IMG_UINT32 ui32InternalJobRef,
		RGX_HWPERF_KICK_TYPE eKickType)
{
	PVR_DPF_ENTERED;

	PVR_DPF((PVR_DBG_VERBOSE, "RGXHWPerfFTraceGPUEnqueueEvent: ExtJobRef %u, "
	        "IntJobRef %u", ui32ExternalJobRef, ui32InternalJobRef));

	PVRGpuTraceClientWork(ui32ExternalJobRef, ui32InternalJobRef,
	                      RGXHWPerfKickTypeToStr(eKickType));

	PVR_DPF_RETURN;
}


static void RGXHWPerfFTraceGPUSwitchEvent(PVRSRV_RGXDEV_INFO *psDevInfo,
		RGX_HWPERF_V2_PACKET_HDR* psHWPerfPkt, const IMG_CHAR* pszWorkName,
		PVR_GPUTRACE_SWITCH_TYPE eSwType)
{
	IMG_UINT64 ui64Timestamp;
	RGX_HWPERF_HW_DATA_FIELDS* psHWPerfPktData;
	RGXFWIF_GPU_UTIL_FWCB *psGpuUtilFWCB = psDevInfo->psRGXFWIfGpuUtilFWCb;
	RGXFWIF_TIME_CORR *psTimeCorr;
	IMG_UINT32 ui32CRDeltaToOSDeltaKNs;
	IMG_UINT64 ui64CRTimeStamp;
	IMG_UINT64 ui64OSTimeStamp;

	PVR_DPF_ENTERED;

	PVR_ASSERT(psHWPerfPkt);
	PVR_ASSERT(pszWorkName);

	psHWPerfPktData = (RGX_HWPERF_HW_DATA_FIELDS*) RGX_HWPERF_GET_PACKET_DATA_BYTES(psHWPerfPkt);

	/* Filter out 3DFINISH events for 3DTQKICKs which have already been
	 * filtered by ValidAndEmitFTraceEvent() */

	/* Calculate the OS timestamp given an RGX timestamp in the HWPerf event */
	psTimeCorr              = &psGpuUtilFWCB->sTimeCorr[psHWPerfPktData->ui32TimeCorrIndex];
	ui64CRTimeStamp         = psTimeCorr->ui64CRTimeStamp;
	ui64OSTimeStamp         = psTimeCorr->ui64OSTimeStamp;
	ui32CRDeltaToOSDeltaKNs = psTimeCorr->ui32CRDeltaToOSDeltaKNs;

	if(psDevInfo->ui64LastSampledTimeCorrOSTimeStamp > ui64OSTimeStamp)
	{
		/* The previous packet had a time reference (time correlation data) more recent
		 * than the one in the current packet, it means the timer correlation array wrapped
		 * too quickly (buffer too small) and in the previous call to RGXHWPerfFTraceGPUSwitchEvent
		 * we read one of the newest timer correlations rather than one of the oldest ones.
		 */
		PVR_DPF((PVR_DBG_ERROR, "RGXHWPerfFTraceGPUSwitchEvent: The timestamps computed so far could be wrong! "
		                        "The time correlation array size should be increased to avoid this."));
	}
	psDevInfo->ui64LastSampledTimeCorrOSTimeStamp = ui64OSTimeStamp;

	{
		IMG_UINT64 deltaRgxTimer = psHWPerfPkt->ui64Timestamp - ui64CRTimeStamp;  /* RGX CR timer ticks delta */
		IMG_UINT64 delta_nS =
		    RGXFWIF_GET_DELTA_OSTIME_NS(deltaRgxTimer, ui32CRDeltaToOSDeltaKNs); /* RGX time delta in nS */
		ui64Timestamp = ui64OSTimeStamp + delta_nS;                              /* Calculate OS time of HWPerf event */

		PVR_DPF((PVR_DBG_VERBOSE, "RGXHWPerfFTraceGPUSwitchEvent: psCurrentDvfs RGX %llu, OS %llu, DVFSCLK %u",
		         ui64CRTimeStamp, ui64OSTimeStamp, psTimeCorr->ui32CoreClockSpeed ));
	}

	PVR_DPF((PVR_DBG_VERBOSE, "RGXHWPerfFTraceGPUSwitchEvent: %s ui32ExtJobRef=%d, ui32IntJobRef=%d, eSwType=%d",
			pszWorkName, psHWPerfPktData->ui32DMContext, psHWPerfPktData->ui32IntJobRef, eSwType));

	PVRGpuTraceWorkSwitch(ui64Timestamp, psHWPerfPktData->ui32DMContext,
	                      psHWPerfPktData->ui32IntJobRef, pszWorkName, eSwType);

	PVR_DPF_RETURN;
}

static void RGXHWPerfFTraceGPUUfoEvent(PVRSRV_RGXDEV_INFO *psDevInfo,
                                       RGX_HWPERF_V2_PACKET_HDR* psHWPerfPkt)
{
	IMG_UINT64 ui64Timestamp;
	RGX_HWPERF_UFO_DATA *psHWPerfPktData;
	RGXFWIF_GPU_UTIL_FWCB *psGpuUtilFWCB = psDevInfo->psRGXFWIfGpuUtilFWCb;
	RGXFWIF_TIME_CORR *psTimeCorr;
	IMG_UINT32 ui32CRDeltaToOSDeltaKNs;
	IMG_UINT64 ui64CRTimeStamp;
	IMG_UINT64 ui64OSTimeStamp;
	IMG_UINT32 ui32UFOCount;
	RGX_HWPERF_UFO_DATA_ELEMENT *puData;

	psHWPerfPktData = (RGX_HWPERF_UFO_DATA *)
	        RGX_HWPERF_GET_PACKET_DATA_BYTES(psHWPerfPkt);

	ui32UFOCount = RGX_HWPERF_GET_UFO_STREAMSIZE(psHWPerfPktData->ui32StreamInfo);
	puData = (RGX_HWPERF_UFO_DATA_ELEMENT *) (((IMG_BYTE *) psHWPerfPktData)
	        + RGX_HWPERF_GET_UFO_STREAMOFFSET(psHWPerfPktData->ui32StreamInfo));

	/* Calculate the OS timestamp given an RGX timestamp in the HWPerf event */
	psTimeCorr = &psGpuUtilFWCB->sTimeCorr[psHWPerfPktData->ui32TimeCorrIndex];
	ui64CRTimeStamp = psTimeCorr->ui64CRTimeStamp;
	ui64OSTimeStamp = psTimeCorr->ui64OSTimeStamp;
	ui32CRDeltaToOSDeltaKNs = psTimeCorr->ui32CRDeltaToOSDeltaKNs;

	if(psDevInfo->ui64LastSampledTimeCorrOSTimeStamp > ui64OSTimeStamp)
	{
		/* The previous packet had a time reference (time correlation data) more
		 * recent than the one in the current packet, it means the timer
		 * correlation array wrapped too quickly (buffer too small) and in the
		 * previous call to RGXHWPerfFTraceGPUUfoEvent we read one of the
		 * newest timer correlations rather than one of the oldest ones.
		 */
		PVR_DPF((PVR_DBG_ERROR, "RGXHWPerfFTraceGPUUfoEvent: The timestamps "
		        "computed so far could be wrong! The time correlation array "
		        "size should be increased to avoid this."));
	}
	psDevInfo->ui64LastSampledTimeCorrOSTimeStamp = ui64OSTimeStamp;

	{
		/* RGX CR timer ticks delta */
		IMG_UINT64 deltaRgxTimer = psHWPerfPkt->ui64Timestamp - ui64CRTimeStamp;
		/* RGX time delta in nS */
		IMG_UINT64 delta_ns =
		        RGXFWIF_GET_DELTA_OSTIME_NS(deltaRgxTimer, ui32CRDeltaToOSDeltaKNs);
		/* Calculate OS time of HWPerf event */
		ui64Timestamp = ui64OSTimeStamp + delta_ns;

		PVR_DPF((PVR_DBG_VERBOSE, "RGXHWPerfFTraceGPUUfoEvent: psCurrentDvfs "
		        "RGX %llu, OS %llu, DVFSCLK %u", ui64CRTimeStamp,
		        ui64OSTimeStamp, psTimeCorr->ui32CoreClockSpeed));
	}

	PVR_DPF((PVR_DBG_VERBOSE, "RGXHWPerfFTraceGPUUfoEvent: ui32ExtJobRef=%d, "
	        "ui32IntJobRef=%d", psHWPerfPktData->ui32ExtJobRef,
	        psHWPerfPktData->ui32IntJobRef));

	PVRGpuTraceUfo(ui64Timestamp, psHWPerfPktData->eEvType,
	        psHWPerfPktData->ui32ExtJobRef, psHWPerfPktData->ui32FWCtx,
	        psHWPerfPktData->ui32IntJobRef, ui32UFOCount, puData);
}

static IMG_BOOL ValidAndEmitFTraceEvent(PVRSRV_RGXDEV_INFO *psDevInfo,
		RGX_HWPERF_V2_PACKET_HDR* psHWPerfPkt)
{
	RGX_HWPERF_EVENT_TYPE eType;
	const IMG_CHAR* pszWorkName;
	PVR_GPUTRACE_SWITCH_TYPE eSwType;
	static const struct {
		IMG_CHAR* pszName;
		PVR_GPUTRACE_SWITCH_TYPE eSwType;
	} aszHwEventTypeMap[] = {
			{ /* RGX_HWPERF_HW_TAKICK */       "TA",     PVR_GPUTRACE_SWITCH_TYPE_BEGIN },
			{ /* RGX_HWPERF_HW_TAFINISHED */   "TA",     PVR_GPUTRACE_SWITCH_TYPE_END },
			{ /* RGX_HWPERF_HW_3DTQKICK */     "TQ3D",   PVR_GPUTRACE_SWITCH_TYPE_BEGIN },
			{ /* RGX_HWPERF_HW_3DKICK */       "3D",     PVR_GPUTRACE_SWITCH_TYPE_BEGIN },
			{ /* RGX_HWPERF_HW_3DFINISHED */   "3D",     PVR_GPUTRACE_SWITCH_TYPE_END },
			{ /* RGX_HWPERF_HW_CDMKICK */      "CDM",    PVR_GPUTRACE_SWITCH_TYPE_BEGIN },
			{ /* RGX_HWPERF_HW_CDMFINISHED */  "CDM",    PVR_GPUTRACE_SWITCH_TYPE_END },
			{ /* RGX_HWPERF_HW_TLAKICK */      "TQ2D",   PVR_GPUTRACE_SWITCH_TYPE_BEGIN },
			{ /* RGX_HWPERF_HW_TLAFINISHED */  "TQ2D",   PVR_GPUTRACE_SWITCH_TYPE_END },
			{ /* RGX_HWPERF_HW_3DSPMKICK */    "3DSPM",  PVR_GPUTRACE_SWITCH_TYPE_BEGIN },
			{ /* RGX_HWPERF_HW_PERIODIC */     NULL, 0 }, /* PERIODIC not supported */
			{ /* RGX_HWPERF_HW_RTUKICK */      "RTU",    PVR_GPUTRACE_SWITCH_TYPE_BEGIN },
			{ /* RGX_HWPERF_HW_RTUFINISHED */  "RTU",    PVR_GPUTRACE_SWITCH_TYPE_END },
			{ /* RGX_HWPERF_HW_SHGKICK */      "SHG",    PVR_GPUTRACE_SWITCH_TYPE_BEGIN },
			{ /* RGX_HWPERF_HW_SHGFINISHED */  "SHG",    PVR_GPUTRACE_SWITCH_TYPE_END },
			{ /* RGX_HWPERF_HW_3DTQFINISHED */ "TQ3D",   PVR_GPUTRACE_SWITCH_TYPE_END },
			{ /* RGX_HWPERF_HW_3DSPMFINISHED */ "3DSPM", PVR_GPUTRACE_SWITCH_TYPE_END },
	};

	PVR_ASSERT(psHWPerfPkt);
	eType = RGX_HWPERF_GET_TYPE(psHWPerfPkt);

	if (psDevInfo->ui32FTraceLastOrdinal != psHWPerfPkt->ui32Ordinal - 1)
	{
		RGX_HWPERF_STREAM_ID eStreamId = RGX_HWPERF_GET_STREAM_ID(psHWPerfPkt);
		PVRGpuTraceEventsLost(eStreamId,
		                      psDevInfo->ui32FTraceLastOrdinal,
		                      psHWPerfPkt->ui32Ordinal);
		PVR_DPF((PVR_DBG_ERROR, "FTrace events lost (stream_id = %u, ordinal: last = %u, current = %u)",
		         eStreamId, psDevInfo->ui32FTraceLastOrdinal, psHWPerfPkt->ui32Ordinal));
	}

	psDevInfo->ui32FTraceLastOrdinal = psHWPerfPkt->ui32Ordinal;

	/* Process UFO packets */
	if (eType == RGX_HWPERF_UFO)
	{
		RGXHWPerfFTraceGPUUfoEvent(psDevInfo, psHWPerfPkt);
		return IMG_TRUE;
	}

	/* Process HW packets */
	if (((eType < RGX_HWPERF_HW_TAKICK) || (eType > RGX_HWPERF_HW_3DSPMFINISHED)))
	{
		/* No map entry, ignore event */
		PVR_DPF((PVR_DBG_VERBOSE, "ValidAndEmitFTraceEvent: Unsupported event type %d %02d",
			eType, eType+RGX_HWPERF_HW_TAKICK)); 
		return IMG_FALSE;
	}
	eType-=RGX_HWPERF_HW_TAKICK;

	if (aszHwEventTypeMap[eType].pszName == NULL)
	{
		/* Not supported map entry, ignore event */
		PVR_DPF((PVR_DBG_VERBOSE, "ValidAndEmitFTraceEventl: Unsupported event type %d %02d",
			eType, eType+RGX_HWPERF_HW_TAKICK)); 
		return IMG_FALSE;
	}

	pszWorkName = aszHwEventTypeMap[eType].pszName;
	eSwType = aszHwEventTypeMap[eType].eSwType;

	RGXHWPerfFTraceGPUSwitchEvent(psDevInfo, psHWPerfPkt, pszWorkName, eSwType);

	return IMG_TRUE;
}


static void RGXHWPerfFTraceGPUProcessPackets(PVRSRV_RGXDEV_INFO *psDevInfo,
		IMG_PBYTE pBuffer, IMG_UINT32 ui32ReadLen)
{
	IMG_UINT32			ui32TlPackets = 0;
	IMG_UINT32			ui32HWPerfPackets = 0;
	IMG_UINT32			ui32HWPerfPacketsSent = 0;
	IMG_PBYTE			pBufferEnd;
	PVRSRVTL_PPACKETHDR psHDRptr;
	PVRSRVTL_PACKETTYPE ui16TlType;

	PVR_DPF_ENTERED;

	PVR_ASSERT(psDevInfo);
	PVR_ASSERT(pBuffer);
	PVR_ASSERT(ui32ReadLen);

	/* Process the TL Packets
	 */
	pBufferEnd = pBuffer+ui32ReadLen;
	psHDRptr = GET_PACKET_HDR(pBuffer);
	while ( psHDRptr < (PVRSRVTL_PPACKETHDR)pBufferEnd )
	{
		ui16TlType = GET_PACKET_TYPE(psHDRptr);
		if (ui16TlType == PVRSRVTL_PACKETTYPE_DATA)
		{
			IMG_UINT16 ui16DataLen = GET_PACKET_DATA_LEN(psHDRptr);
			if (0 == ui16DataLen)
			{
				PVR_DPF((PVR_DBG_ERROR, "RGXHWPerfFTraceGPUProcessPackets: ZERO Data in TL data packet: %p", psHDRptr));
			}
			else
			{
				RGX_HWPERF_V2_PACKET_HDR* psHWPerfPkt;
				RGX_HWPERF_V2_PACKET_HDR* psHWPerfEnd;

				/* Check for lost hwperf data packets */
				psHWPerfEnd = RGX_HWPERF_GET_PACKET(GET_PACKET_DATA_PTR(psHDRptr)+ui16DataLen);
				psHWPerfPkt = RGX_HWPERF_GET_PACKET(GET_PACKET_DATA_PTR(psHDRptr));
				do
				{
					if (ValidAndEmitFTraceEvent(psDevInfo, psHWPerfPkt))
					{
						ui32HWPerfPacketsSent++;
					}
					ui32HWPerfPackets++;
					psHWPerfPkt = RGX_HWPERF_GET_NEXT_PACKET(psHWPerfPkt);
				}
				while (psHWPerfPkt < psHWPerfEnd);
			}
		}
		else if (ui16TlType == PVRSRVTL_PACKETTYPE_MOST_RECENT_WRITE_FAILED)
		{
			PVR_DPF((PVR_DBG_MESSAGE, "RGXHWPerfFTraceGPUProcessPackets: Indication that the transport buffer was full"));
		}
		else
		{
			/* else Ignore padding packet type and others */
			PVR_DPF((PVR_DBG_MESSAGE, "RGXHWPerfFTraceGPUProcessPackets: Ignoring TL packet, type %d", ui16TlType ));
		}

		psHDRptr = GET_NEXT_PACKET_ADDR(psHDRptr);
		ui32TlPackets++;
	}

	PVR_DPF((PVR_DBG_VERBOSE, "RGXHWPerfFTraceGPUProcessPackets: TL "
	 		"Packets processed %03d, HWPerf packets %03d, sent %03d",
	 		ui32TlPackets, ui32HWPerfPackets, ui32HWPerfPacketsSent));

	PVR_DPF_RETURN;
}


static
void RGXHWPerfFTraceCmdCompleteNotify(PVRSRV_CMDCOMP_HANDLE hCmdCompHandle)
{
	PVRSRV_DATA*		psPVRSRVData = PVRSRVGetPVRSRVData();
	PVRSRV_RGXDEV_INFO* psDeviceInfo = hCmdCompHandle;
	PVRSRV_ERROR		eError;
	IMG_HANDLE			hStream;
	IMG_PBYTE			pBuffer;
	IMG_UINT32			ui32ReadLen;

	PVR_DPF_ENTERED;

	/* Command-complete notifiers can run concurrently. If this is
	 * happening, just bail out and let the previous call finish.
	 * This is ok because we can process the queued packets on the next call.
	 */
	if (!(OSTryLockAcquire(g_hFTraceLock)))
	{
		PVR_DPF_RETURN;
	}

	/* Exit if no HWPerf enabled device exits */
	PVR_ASSERT(psDeviceInfo != NULL &&
			   psPVRSRVData != NULL &&
			   gpsRgxDevInfo != NULL);

	hStream = psDeviceInfo->hGPUTraceTLStream;

	if (hStream)
	{
		/* If we have a valid stream attempt to acquire some data */
		eError = TLClientAcquireData(DIRECT_BRIDGE_HANDLE, hStream, &pBuffer, &ui32ReadLen);
		if (eError == PVRSRV_OK)
		{
			/* Process the HWPerf packets and release the data */
			if (ui32ReadLen > 0)
			{
				PVR_DPF((PVR_DBG_VERBOSE, "RGXHWPerfFTraceGPUThread: DATA AVAILABLE offset=%p, length=%d", pBuffer, ui32ReadLen));

				/* Process the transport layer data for HWPerf packets... */
				RGXHWPerfFTraceGPUProcessPackets(psDeviceInfo, pBuffer, ui32ReadLen);

				eError = TLClientReleaseData(DIRECT_BRIDGE_HANDLE, hStream);
				if (eError != PVRSRV_OK)
				{
					PVR_LOG_ERROR(eError, "TLClientReleaseData");

					/* Serious error, disable FTrace GPU events */

					/* Release TraceLock so we always have the locking
					 * order BridgeLock->TraceLock to prevent AB-BA deadlocks*/
					OSLockRelease(g_hFTraceLock);
					OSAcquireBridgeLock();
					RGXHWPerfFTraceGPUDisable(IMG_FALSE);
					OSReleaseBridgeLock();
					goto out;

				}
			} /* else no data, ignore */
		}
		else if (eError != PVRSRV_ERROR_TIMEOUT)
		{
			PVR_LOG_ERROR(eError, "TLClientAcquireData");
		}
	}

	OSLockRelease(g_hFTraceLock);
out:
	PVR_DPF_RETURN;
}


PVRSRV_ERROR RGXHWPerfFTraceGPUInit(PVRSRV_RGXDEV_INFO *psDevInfo)
{
	PVRSRV_ERROR eError = PVRSRV_OK;

	PVR_DPF_ENTERED;

    /* Must be setup already by the general HWPerf module initialisation.
	 * DevInfo object needed by FTrace event generation code */
	PVR_ASSERT(gpsRgxDevInfo);
	gpsRgxDevInfo->bFTraceGPUEventsEnabled = IMG_FALSE;
	/* We initialise it only once because we want to track if any
	 * packets were dropped. */
	gpsRgxDevInfo->ui32FTraceLastOrdinal = IMG_UINT32_MAX - 1;

	eError = OSLockCreate(&g_hFTraceLock, LOCK_TYPE_DISPATCH);
	eError = OSLockCreate(&g_hLockFTraceEventLock, LOCK_TYPE_PASSIVE);

	PVR_DPF_RETURN_RC(eError);
}


void RGXHWPerfFTraceGPUDeInit(PVRSRV_RGXDEV_INFO *psDevInfo)
{
	PVR_DPF_ENTERED;

	if (gpsRgxDevInfo->bFTraceGPUEventsEnabled)
	{
		RGXHWPerfFTraceGPUDisable(IMG_TRUE);
		gpsRgxDevInfo->bFTraceGPUEventsEnabled = IMG_FALSE;
	}

	OSLockDestroy(g_hLockFTraceEventLock);
	OSLockDestroy(g_hFTraceLock);

	PVR_DPF_RETURN;
}

void PVRGpuTraceEnableUfoCallback(void)
{
    OSLockAcquire(g_hLockFTraceEventLock);

	if (g_uiUfoEventRef++ == 0)
	{
		IMG_UINT64 ui64Filter = RGX_HWPERF_EVENT_MASK_VALUE(RGX_HWPERF_UFO) |
		        gpsRgxDevInfo->ui64HWPerfFilter;
		/* Small chance exists that ui64HWPerfFilter can be changed here and
		 * the newest filter value will be changed to the old one + UFO event.
		 * This is not a critical problem. */
		if (PVRSRVRGXCtrlHWPerfKM(NULL, gpsRgxDevNode, RGX_HWPERF_STREAM_ID0_FW,
		                          IMG_FALSE, ui64Filter) != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR, "Could not enable UFO HWPerf event."));
		}
	}

    OSLockRelease(g_hLockFTraceEventLock);
}

void PVRGpuTraceDisableUfoCallback(void)
{
    OSLockAcquire(g_hLockFTraceEventLock);

    if (--g_uiUfoEventRef == 0)
	{
		IMG_UINT64 ui64Filter = ~(RGX_HWPERF_EVENT_MASK_VALUE(RGX_HWPERF_UFO)) &
		        gpsRgxDevInfo->ui64HWPerfFilter;
		/* Small chance exists that ui64HWPerfFilter can be changed here and
		 * the newest filter value will be changed to the old one + UFO event.
		 * This is not a critical problem. */
		if (PVRSRVRGXCtrlHWPerfKM(NULL, gpsRgxDevNode, RGX_HWPERF_STREAM_ID0_FW,
		                          IMG_FALSE, ui64Filter) != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR, "Could not enable UFO HWPerf event."));
		}
	}

    OSLockRelease(g_hLockFTraceEventLock);
}

#endif /* SUPPORT_GPUTRACE_EVENTS */

/******************************************************************************
 * Currently only implemented on Linux. Feature can be enabled to provide
 * an interface to 3rd-party kernel modules that wish to access the
 * HWPerf data. The API is documented in the rgxapi_km.h header and
 * the rgx_hwperf* headers.
 *****************************************************************************/

/* Internal HWPerf kernel connection/device data object to track the state
 * of a client session.
 */
typedef struct
{
	PVRSRV_DEVICE_NODE* psRgxDevNode;
	PVRSRV_RGXDEV_INFO* psRgxDevInfo;

	/* TL Open/close state */
	IMG_HANDLE          hSD[RGX_HWPERF_STREAM_ID_LAST];

	/* TL Acquire/release state */
	IMG_PBYTE			pHwpBuf[RGX_HWPERF_STREAM_ID_LAST];
	IMG_UINT32			ui32HwpBufLen[RGX_HWPERF_STREAM_ID_LAST];

} RGX_KM_HWPERF_DEVDATA;


PVRSRV_ERROR RGXHWPerfLazyConnect(
		IMG_HANDLE* phDevData)
{
	RGX_KM_HWPERF_DEVDATA* psDevData;

	/* Valid input argument values supplied by the caller */
	if (!phDevData)
	{
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	/* Clear the handle to aid error checking by caller */
	*phDevData = NULL;

	/* Check the HWPerf module is initialised before we allow a connection */
	if (!gpsRgxDevNode || !gpsRgxDevInfo)
	{
		return PVRSRV_ERROR_INVALID_DEVICE;
	}

	/* Allocation the session object for this connection */
	psDevData = OSAllocZMem(sizeof(*psDevData));
	if (psDevData == NULL)
	{
		return PVRSRV_ERROR_OUT_OF_MEMORY;
	}
	psDevData->psRgxDevNode = gpsRgxDevNode;
	psDevData->psRgxDevInfo = gpsRgxDevInfo;

	*phDevData = psDevData;

	return PVRSRV_OK;
}

PVRSRV_ERROR RGXHWPerfOpen(
		IMG_HANDLE hDevData)
{
	PVRSRV_ERROR eError;
	RGX_KM_HWPERF_DEVDATA* psDevData = (RGX_KM_HWPERF_DEVDATA*) hDevData;

	/* Valid input argument values supplied by the caller */
	if (!psDevData)
	{
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	/* Check the HWPerf module is initialised before we allow a connection */
	if (!psDevData->psRgxDevNode || !psDevData->psRgxDevInfo)
	{
		return PVRSRV_ERROR_INVALID_DEVICE;
	}

	/* In the case where the AppHint has not been set we need to
	 * initialise the HWPerf resources here. Allocated on-demand
	 * to reduce RAM foot print on systems not needing HWPerf.
	 */
	OSLockAcquire(gpsRgxDevInfo->hLockHWPerfModule);
	if (RGXHWPerfIsInitRequired())
	{
		eError = RGXHWPerfInitOnDemandResources();
		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR, "%s: Initialization of on-demand HWPerfGpu"
			        " resources failed", __FUNCTION__));
			OSLockRelease(gpsRgxDevInfo->hLockHWPerfModule);
			goto e0;
		}
	}
	OSLockRelease(gpsRgxDevInfo->hLockHWPerfModule);

	OSLockAcquire(gpsRgxDevInfo->hLockHWPerfHostStream);
	if (gpsRgxDevInfo->hHWPerfHostStream == NULL)
	{
		eError = RGXHWPerfHostInitOnDemandResources();
		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR, "%s: Initialization of on-demand HWPerfHost"
			        " resources failed", __FUNCTION__));
			OSLockRelease(gpsRgxDevInfo->hLockHWPerfHostStream);
			goto e0;
		}
	}
	OSLockRelease(gpsRgxDevInfo->hLockHWPerfHostStream);

	/* Open the 'hwperf_gpu' TL stream for reading in this session */
	eError = TLClientOpenStream(DIRECT_BRIDGE_HANDLE,
								HWPERF_TL_STREAM_NAME,
								PVRSRV_STREAM_FLAG_ACQUIRE_NONBLOCKING,
								&psDevData->hSD[RGX_HWPERF_STREAM_ID0_FW]);
	if (eError != PVRSRV_OK)
	{
		goto e1;
	}

	/* Open the 'hwperf_host' TL stream for reading in this session */
	eError = TLClientOpenStream(DIRECT_BRIDGE_HANDLE,
								HWPERF_HOST_TL_STREAM_NAME,
								PVRSRV_STREAM_FLAG_ACQUIRE_NONBLOCKING,
								&psDevData->hSD[RGX_HWPERF_STREAM_ID1_HOST]);
	if (eError != PVRSRV_OK)
	{
		goto e1;
	}

	return PVRSRV_OK;

e1:
	RGXHWPerfHostDeInit();
e0:
	RGXHWPerfDeinit();

	return eError;
}


PVRSRV_ERROR RGXHWPerfConnect(
		IMG_HANDLE* phDevData)
{
	PVRSRV_ERROR eError;

	eError = RGXHWPerfLazyConnect(phDevData);
	PVR_LOGG_IF_ERROR(eError, "RGXHWPerfLazyConnect", e0);

	eError = RGXHWPerfOpen(*phDevData);
	PVR_LOGG_IF_ERROR(eError, "RGXHWPerfOpen", e1);

	return PVRSRV_OK;

e1:
	RGXHWPerfFreeConnection(phDevData);
e0:
	*phDevData = NULL;
	return eError;
}


PVRSRV_ERROR RGXHWPerfControl(
		IMG_HANDLE           hDevData,
		RGX_HWPERF_STREAM_ID eStreamId,
		IMG_BOOL             bToggle,
		IMG_UINT64           ui64Mask)
{
	PVRSRV_ERROR           eError;
	RGX_KM_HWPERF_DEVDATA* psDevData = (RGX_KM_HWPERF_DEVDATA*)hDevData;

	/* Valid input argument values supplied by the caller */
	if (!psDevData)
	{
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	/* Ensure we are initialised and have a valid device node */
	if (!psDevData->psRgxDevNode)
	{
		return PVRSRV_ERROR_INVALID_DEVICE;
	}

	/* Call the internal server API */
	eError = PVRSRVRGXCtrlHWPerfKM(NULL, psDevData->psRgxDevNode, eStreamId, bToggle, ui64Mask);
	return eError;
}


PVRSRV_ERROR RGXHWPerfConfigureAndEnableCounters(
		IMG_HANDLE					hDevData,
		IMG_UINT32					ui32NumBlocks,
		RGX_HWPERF_CONFIG_CNTBLK*	asBlockConfigs)
{
	PVRSRV_ERROR           eError;
	RGX_KM_HWPERF_DEVDATA* psDevData = (RGX_KM_HWPERF_DEVDATA*)hDevData;

	/* Valid input argument values supplied by the caller */
	if (!psDevData || ui32NumBlocks==0 || !asBlockConfigs)
	{
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	if (ui32NumBlocks > RGXFWIF_HWPERF_CTRL_BLKS_MAX)
	{
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	/* Ensure we are initialised and have a valid device node */
	if (!psDevData->psRgxDevNode)
	{
		return PVRSRV_ERROR_INVALID_DEVICE;
	}

	/* Call the internal server API */
	eError = PVRSRVRGXConfigEnableHWPerfCountersKM(NULL,
			psDevData->psRgxDevNode, ui32NumBlocks, asBlockConfigs);
	return eError;
}


PVRSRV_ERROR RGXHWPerfDisableCounters(
		IMG_HANDLE   hDevData,
		IMG_UINT32   ui32NumBlocks,
		IMG_UINT16*   aeBlockIDs)
{
	PVRSRV_ERROR           eError;
	RGX_KM_HWPERF_DEVDATA* psDevData = (RGX_KM_HWPERF_DEVDATA*)hDevData;

	/* Valid input argument values supplied by the caller */
	if (!psDevData || ui32NumBlocks==0 || !aeBlockIDs)
	{
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	if (ui32NumBlocks > RGXFWIF_HWPERF_CTRL_BLKS_MAX)
    {
        return PVRSRV_ERROR_INVALID_PARAMS;
    }

	/* Ensure we are initialised and have a valid device node */
	if (!psDevData->psRgxDevNode)
	{
		return PVRSRV_ERROR_INVALID_DEVICE;
	}

	/* Call the internal server API */
	eError = PVRSRVRGXCtrlHWPerfCountersKM(NULL,
			psDevData->psRgxDevNode, IMG_FALSE, ui32NumBlocks, aeBlockIDs);
	return eError;
}


PVRSRV_ERROR RGXHWPerfAcquireData(
		IMG_HANDLE  hDevData,
		RGX_HWPERF_STREAM_ID eStreamId,
		IMG_PBYTE*  ppBuf,
		IMG_UINT32* pui32BufLen)
{
	PVRSRV_ERROR			eError;
	RGX_KM_HWPERF_DEVDATA*	psDevData = (RGX_KM_HWPERF_DEVDATA*)hDevData;
	IMG_PBYTE				pTlBuf = NULL;
	IMG_UINT32				ui32TlBufLen = 0;
	IMG_PBYTE				pDataDest;
	IMG_UINT32			ui32TlPackets = 0;
	IMG_PBYTE			pBufferEnd;
	PVRSRVTL_PPACKETHDR psHDRptr;
	PVRSRVTL_PACKETTYPE ui16TlType;

	/* Reset the output arguments in case we discover an error */
	*ppBuf = NULL;
	*pui32BufLen = 0;

	/* Valid input argument values supplied by the caller */
	if (!psDevData || eStreamId >= RGX_HWPERF_STREAM_ID_LAST)
	{
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	/* Acquire some data to read from the HWPerf TL stream */
	eError = TLClientAcquireData(DIRECT_BRIDGE_HANDLE,
								 psDevData->hSD,
								 &pTlBuf,
								 &ui32TlBufLen);
	PVR_LOGR_IF_ERROR(eError, "TLClientAcquireData");

	/* TL indicates no data exists so return OK and zero. */
	if ((pTlBuf == NULL) || (ui32TlBufLen == 0))
	{
		return PVRSRV_OK;
	}

	/* Is the client buffer allocated and too small? */
	if (psDevData->pHwpBuf[eStreamId] && (psDevData->ui32HwpBufLen[eStreamId] < ui32TlBufLen))
	{
		OSFREEMEM(psDevData->pHwpBuf[eStreamId]);
	}

	/* Do we need to allocate a new client buffer? */
	if (!psDevData->pHwpBuf[eStreamId])
	{
		psDevData->pHwpBuf[eStreamId] = OSAllocMem(ui32TlBufLen);
		if (psDevData->pHwpBuf[eStreamId]  == NULL)
		{
			(void) TLClientReleaseData(DIRECT_BRIDGE_HANDLE, psDevData->hSD[eStreamId]);
			return PVRSRV_ERROR_OUT_OF_MEMORY;
		}
		psDevData->ui32HwpBufLen[eStreamId] = ui32TlBufLen;
	}

	/* Process each TL packet in the data buffer we have acquired */
	pBufferEnd = pTlBuf+ui32TlBufLen;
	pDataDest = psDevData->pHwpBuf[eStreamId];
	psHDRptr = GET_PACKET_HDR(pTlBuf);
	while ( psHDRptr < (PVRSRVTL_PPACKETHDR)pBufferEnd )
	{
		ui16TlType = GET_PACKET_TYPE(psHDRptr);
		if (ui16TlType == PVRSRVTL_PACKETTYPE_DATA)
		{
			IMG_UINT16 ui16DataLen = GET_PACKET_DATA_LEN(psHDRptr);
			if (0 == ui16DataLen)
			{
				PVR_DPF((PVR_DBG_ERROR, "RGXHWPerfAcquireData: ZERO Data in TL data packet: %p", psHDRptr));
			}
			else
			{
				/* For valid data copy it into the client buffer and move
				 * the write position on */
				OSMemCopy(pDataDest, GET_PACKET_DATA_PTR(psHDRptr), ui16DataLen);
				pDataDest += ui16DataLen;
			}
		}
		else if (ui16TlType == PVRSRVTL_PACKETTYPE_MOST_RECENT_WRITE_FAILED)
		{
			PVR_DPF((PVR_DBG_MESSAGE, "RGXHWPerfAcquireData: Indication that the transport buffer was full"));
		}
		else
		{
			/* else Ignore padding packet type and others */
			PVR_DPF((PVR_DBG_MESSAGE, "RGXHWPerfAcquireData: Ignoring TL packet, type %d", ui16TlType ));
		}

		/* Update loop variable to the next packet and increment counts */
		psHDRptr = GET_NEXT_PACKET_ADDR(psHDRptr);
		ui32TlPackets++;
	}

	PVR_DPF((PVR_DBG_VERBOSE, "RGXHWPerfAcquireData: TL Packets processed %03d", ui32TlPackets));

	/* Update output arguments with client buffer details and true length */
	*ppBuf = psDevData->pHwpBuf[eStreamId];
	*pui32BufLen = pDataDest - psDevData->pHwpBuf[eStreamId];

	return PVRSRV_OK;
}


PVRSRV_ERROR RGXHWPerfReleaseData(
		IMG_HANDLE hDevData,
		RGX_HWPERF_STREAM_ID eStreamId)
{
	PVRSRV_ERROR			eError;
	RGX_KM_HWPERF_DEVDATA*	psDevData = (RGX_KM_HWPERF_DEVDATA*)hDevData;

	/* Valid input argument values supplied by the caller */
	if (!psDevData || eStreamId >= RGX_HWPERF_STREAM_ID_LAST)
	{
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	/* Free the client buffer if allocated and reset length */
	if (psDevData->pHwpBuf[eStreamId])
	{
		OSFREEMEM(psDevData->pHwpBuf[eStreamId]);
	}
	psDevData->ui32HwpBufLen[eStreamId] = 0;

	/* Inform the TL that we are done with reading the data. Could perform this
	 * in the acquire call but felt it worth keeping it symmetrical */
	eError = TLClientReleaseData(DIRECT_BRIDGE_HANDLE, psDevData->hSD);
	return eError;
}


PVRSRV_ERROR RGXHWPerfGetFilter(
		IMG_HANDLE  hDevData,
		RGX_HWPERF_STREAM_ID eStreamId,
		IMG_UINT64 *ui64Filter)
{
#if !defined(PVRSRV_GPUVIRT_GUESTDRV)
	PVRSRV_RGXDEV_INFO* psRgxDevInfo =
			hDevData ? ((RGX_KM_HWPERF_DEVDATA*) hDevData)->psRgxDevInfo : NULL;

	/* Valid input argument values supplied by the caller */
	if (!psRgxDevInfo)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Invalid pointer to the RGX device",
		        __func__));
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	switch (eStreamId) {
		case RGX_HWPERF_STREAM_ID0_FW:
			OSLockAcquire(psRgxDevInfo->hLockHWPerfModule);
			*ui64Filter = psRgxDevInfo->ui64HWPerfFilter;
			OSLockRelease(psRgxDevInfo->hLockHWPerfModule);
			break;
		case RGX_HWPERF_STREAM_ID1_HOST:
			OSLockAcquire(psRgxDevInfo->hLockHWPerfHostStream);
			*ui64Filter = psRgxDevInfo->ui32HWPerfHostFilter;
			OSLockRelease(psRgxDevInfo->hLockHWPerfHostStream);
			break;
		default:
			PVR_DPF((PVR_DBG_ERROR, "%s: Invalid stream ID",
			        __func__));
			return PVRSRV_ERROR_INVALID_PARAMS;
	}
#endif

	return PVRSRV_OK;
}


PVRSRV_ERROR RGXHWPerfFreeConnection(
		IMG_HANDLE hDevData)
{
	RGX_KM_HWPERF_DEVDATA* psDevData = (RGX_KM_HWPERF_DEVDATA*) hDevData;

	/* Check session handle is not zero */
	if (!psDevData)
	{
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	/* Free the session memory */
	psDevData->psRgxDevNode = NULL;
	psDevData->psRgxDevInfo = NULL;
	OSFREEMEM(psDevData);

	return PVRSRV_OK;
}


PVRSRV_ERROR RGXHWPerfClose(
		IMG_HANDLE hDevData)
{
	RGX_KM_HWPERF_DEVDATA* psDevData = (RGX_KM_HWPERF_DEVDATA*) hDevData;
	IMG_UINT uiStreamId;
	PVRSRV_ERROR eError;

	/* Check session handle is not zero */
	if (!psDevData)
	{
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	for (uiStreamId = 0; uiStreamId < RGX_HWPERF_STREAM_ID_LAST; uiStreamId++)
	{
		/* If the client buffer exists they have not called ReleaseData
		 * before disconnecting so clean it up */
		if (psDevData->pHwpBuf[uiStreamId])
		{
			/* RGXHWPerfReleaseData call will null out the buffer fields
			 * and length */
			eError = RGXHWPerfReleaseData(hDevData, uiStreamId);
			PVR_LOG_ERROR(eError, "RGXHWPerfReleaseData");
		}

		/* Close the TL stream, ignore the error if it occurs as we
		 * are disconnecting */
		if (psDevData->hSD[uiStreamId])
		{
			eError = TLClientCloseStream(DIRECT_BRIDGE_HANDLE,
										 psDevData->hSD[uiStreamId]);
			PVR_LOG_ERROR(eError, "TLClientCloseStream");
			psDevData->hSD[uiStreamId] = NULL;
		}
	}

	return PVRSRV_OK;
}


PVRSRV_ERROR RGXHWPerfDisconnect(
		IMG_HANDLE hDevData)
{
	PVRSRV_ERROR eError = PVRSRV_OK;

	eError = RGXHWPerfClose(hDevData);
	PVR_LOG_ERROR(eError, "RGXHWPerfClose");

	eError = RGXHWPerfFreeConnection(hDevData);
	PVR_LOG_ERROR(eError, "RGXHWPerfFreeConnection");

	return eError;
}


const IMG_CHAR *RGXHWPerfKickTypeToStr(RGX_HWPERF_KICK_TYPE eKickType)
{
	static const IMG_CHAR *aszKickType[RGX_HWPERF_KICK_TYPE_LAST+1] = {
		"TA3D", "TQ2D", "TQ3D", "CDM", "RS", "VRDM", "LAST"
	};

	/* cast in case of negative value */
	if (((IMG_UINT32) eKickType) >= RGX_HWPERF_KICK_TYPE_LAST)
	{
		return "<UNKNOWN>";
	}

	return aszKickType[eKickType];
}

/******************************************************************************
 End of file (rgxdebug.c)
******************************************************************************/
