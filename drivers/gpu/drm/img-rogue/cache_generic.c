/*************************************************************************/ /*!
@File
@Title          CPU generic cache management
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    Implements server side code for CPU cache management in a
                CPU agnostic manner.
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
#include "cache_generic.h"
#include "cache_internal.h"
#include "device.h"
#include "pvr_debug.h"
#include "pvrsrv.h"
#include "osfunc.h"
#include "pmr.h"

PVRSRV_ERROR CacheOpQueue(PMR *psPMR,
						  IMG_DEVMEM_OFFSET_T uiOffset,
						  IMG_DEVMEM_SIZE_T uiSize,
						  PVRSRV_CACHE_OP uiCacheOp)
{
	PVRSRV_ERROR eError;

#if defined(SUPPORT_RANGEBASED_CACHEFLUSH)
	IMG_HANDLE hPrivOut;
	IMG_BYTE *pbCpuVirtAddr;
	IMG_CPU_PHYADDR sCpuPhyAddr;
	IMG_DEVMEM_SIZE_T uiOutSize;
	IMG_DEVMEM_SIZE_T uiCLAlignedSize;
	IMG_DEVMEM_SIZE_T uiPgAlignedSize;
	IMG_DEVMEM_OFFSET_T uiCLAlingedEndOffset;
	IMG_DEVMEM_OFFSET_T uiPgAlignedEndOffset;
	IMG_DEVMEM_OFFSET_T uiCLAlignedStartOffset;
	IMG_DEVMEM_OFFSET_T uiPgAlignedStartOffset;
	static IMG_UINT32 uiCacheLineSize = 0;

	if (uiCacheOp == PVRSRV_CACHE_OP_NONE)
	{
		return PVRSRV_OK;
	}

	/* Carry out full dcache operation if size qualifies */
	if ((uiSize >> PAGE_SHIFT) >= PVR_DIRTY_PAGECOUNT_FLUSH_THRESHOLD)
	{
		if (OSCPUOperation(uiCacheOp) == PVRSRV_OK)
		{
			return PVRSRV_OK;
		}
	}

	/* Need to track both cache-aligned and page-aligned quantities */
	uiPgAlignedSize = uiCLAlignedSize = uiSize;	
	if (! uiCacheLineSize)
	{
		uiCacheLineSize = OSCPUCacheAttributeSize(PVR_DCACHE_LINE_SIZE);
		PVR_ASSERT(uiCacheLineSize != 0);
	}

	/* Round the incoming offset down to the nearest cache-line-aligned address, adjusts size respectively */
	uiCLAlignedStartOffset = (IMG_DEVMEM_OFFSET_T) ((IMG_DEVMEM_OFFSET_T)uiOffset & ~((IMG_DEVMEM_OFFSET_T)uiCacheLineSize-1));
	uiCLAlignedSize += uiOffset - uiCLAlignedStartOffset;
	uiCLAlignedSize = (IMG_DEVMEM_SIZE_T) PVR_ALIGN((IMG_DEVMEM_SIZE_T)uiCLAlignedSize, (IMG_DEVMEM_SIZE_T)uiCacheLineSize);
	uiCLAlingedEndOffset = uiCLAlignedStartOffset + uiCLAlignedSize;

	/* Similar to the above as we need to track page-aligned quantities here to lookup the correct PFNs */
	uiPgAlignedStartOffset = (IMG_DEVMEM_OFFSET_T) ((IMG_DEVMEM_OFFSET_T)uiOffset & ~((IMG_DEVMEM_OFFSET_T)PAGE_SIZE-1));
	uiPgAlignedSize += uiOffset - uiPgAlignedStartOffset;
	uiPgAlignedSize = (IMG_DEVMEM_SIZE_T) PVR_ALIGN((IMG_DEVMEM_SIZE_T)uiPgAlignedSize, (IMG_DEVMEM_SIZE_T)PAGE_SIZE);
	uiPgAlignedEndOffset = uiPgAlignedStartOffset + uiPgAlignedSize;

	/* Acquire total virtual address range using initial page-aligned size/offfset */
	if (PMR_IsSparse(psPMR))
	{
		eError =
		PMRAcquireSparseKernelMappingData(psPMR, uiPgAlignedStartOffset, uiPgAlignedSize,
										  (void **)&pbCpuVirtAddr, (size_t*)&uiOutSize, &hPrivOut);
	}
	else
	{
		eError = PMRAcquireKernelMappingData(psPMR, uiPgAlignedStartOffset, uiPgAlignedSize,
											 (void **)&pbCpuVirtAddr, (size_t*)&uiOutSize, &hPrivOut);
	}
	if (eError != PVRSRV_OK)
	{
		goto e0;
	}

	/* For each virtual page of device memory, lookup the corresponding physical page
	   and flush from the CPU dcache successive cache-line worth of bytes upto said 
	   physical page boundary (or in-page cache-line aligned boundary) */
	for ( ; uiPgAlignedStartOffset < uiPgAlignedEndOffset 
		  ; uiPgAlignedStartOffset += PAGE_SIZE, pbCpuVirtAddr += PAGE_SIZE)
	{
		IMG_BOOL bValid;
		IMG_DEVMEM_SIZE_T uiRelFlushSize;
		IMG_DEVMEM_OFFSET_T uiRelFlushOffset;
		IMG_BYTE *pbCpuVirtAddrStart, *pbCpuVirtAddrEnd;
		IMG_CPU_PHYADDR sCpuPhyAddrStart, sCpuPhyAddrEnd;

		/* Never cross page boundary without looking up the corresponding virtual page PFNs */
		eError = PMR_CpuPhysAddr(psPMR, PAGE_SHIFT, 1, uiPgAlignedStartOffset, &sCpuPhyAddr, &bValid);
		if (eError != PVRSRV_OK)
		{
			goto e1;
		}
		else
		{
			if (! bValid)
			{
				continue;
			}
			else
			{
				/* These quantities allows us to perform cache operations at cache-line
				   granulatrity thereby ensuring we do not perfom more than is neccessary */
				if (uiPgAlignedStartOffset < uiCLAlingedEndOffset)
				{
					IMG_DEVMEM_SIZE_T uiNextPgAlignedOffset = uiPgAlignedStartOffset + PAGE_SIZE;
					uiRelFlushSize = PAGE_SIZE;
					uiRelFlushOffset = 0;

					if (uiCLAlignedStartOffset > uiPgAlignedStartOffset)
					{
						/* Zero unless initially starting at an in-page offset */
						uiRelFlushOffset = uiCLAlignedStartOffset - uiPgAlignedStartOffset;
					}

					if (uiNextPgAlignedOffset > uiCLAlingedEndOffset)
					{
						/* PAGE_SIZE unless current outstanding size is smaller */
						uiRelFlushSize = uiRelFlushOffset ?	
									uiCLAlingedEndOffset - uiCLAlignedStartOffset :
									uiCLAlingedEndOffset - uiPgAlignedStartOffset;
					}
				}
				else
				{
					uiRelFlushOffset = 0;
					uiRelFlushSize = uiCLAlingedEndOffset - (uiPgAlignedStartOffset - PAGE_SIZE);
				}
			}
		}

		if (uiRelFlushSize > 0)
		{
			/* More efficient to request operation for full relative range */
			pbCpuVirtAddrStart = pbCpuVirtAddr + uiRelFlushOffset;
			pbCpuVirtAddrEnd = pbCpuVirtAddrStart + uiRelFlushSize;
			sCpuPhyAddrStart.uiAddr = sCpuPhyAddr.uiAddr + uiRelFlushOffset;
			sCpuPhyAddrEnd.uiAddr = sCpuPhyAddrStart.uiAddr + uiRelFlushSize;

			switch (uiCacheOp)
			{
				case PVRSRV_CACHE_OP_CLEAN:
					OSCleanCPUCacheRangeKM(pbCpuVirtAddrStart, pbCpuVirtAddrEnd,
											sCpuPhyAddrStart, sCpuPhyAddrEnd);
					break;
				case PVRSRV_CACHE_OP_INVALIDATE:
					OSInvalidateCPUCacheRangeKM(pbCpuVirtAddrStart, pbCpuVirtAddrEnd,
											sCpuPhyAddrStart, sCpuPhyAddrEnd);
					break;
				case PVRSRV_CACHE_OP_FLUSH:
					OSFlushCPUCacheRangeKM(pbCpuVirtAddrStart, pbCpuVirtAddrEnd,
											sCpuPhyAddrStart, sCpuPhyAddrEnd);
					break;
				default:
					PVR_DPF((PVR_DBG_ERROR,	"%s: Invalid cache operation type %d",
							__FUNCTION__, uiCacheOp));
					goto e1;
			}
		}
		else
		{
			/* Sanity check */
			PVR_ASSERT(0);
		}
	}

e1:
	eError = PMRReleaseKernelMappingData(psPMR, hPrivOut);
e0:
#else
	PVRSRV_DATA *psData = PVRSRVGetPVRSRVData();
	psData->uiCacheOp = SetCacheOp(psData->uiCacheOp, uiCacheOp);
	eError = PVRSRV_OK;
#endif

	return eError;
}
