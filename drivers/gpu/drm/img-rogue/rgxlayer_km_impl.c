/*************************************************************************/ /*!
@File
@Title          DDK implementation of the Services abstraction layer
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    DDK implementation of the Services abstraction layer
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

#include "rgxlayer_km_impl.h"
#include "pdump_km.h"
#include "devicemem_utils.h"
#include "pvrsrv.h"
#include "rgxdevice.h"


void RGXWriteReg32(const void *hPrivate, IMG_UINT32 ui32RegAddr, IMG_UINT32 ui32RegValue)
{
	PVRSRV_RGXDEV_INFO *psDevInfo;
	void *pvRegsBase;

	PVR_ASSERT(hPrivate != NULL);
	psDevInfo = ((RGX_POWER_LAYER_PARAMS*)hPrivate)->psDevInfo;
	pvRegsBase = psDevInfo->pvRegsBaseKM;

	OSWriteHWReg32(pvRegsBase, ui32RegAddr, ui32RegValue);
	PDUMPREG32(RGX_PDUMPREG_NAME, ui32RegAddr, ui32RegValue, PDUMP_FLAGS_CONTINUOUS);
}

void RGXWriteReg64(const void *hPrivate, IMG_UINT32 ui32RegAddr, IMG_UINT64 ui64RegValue)
{
	PVRSRV_RGXDEV_INFO *psDevInfo;
	void *pvRegsBase;

	PVR_ASSERT(hPrivate != NULL);
	psDevInfo = ((RGX_POWER_LAYER_PARAMS*)hPrivate)->psDevInfo;
	pvRegsBase = psDevInfo->pvRegsBaseKM;

	OSWriteHWReg64(pvRegsBase, ui32RegAddr, ui64RegValue);
	PDUMPREG64(RGX_PDUMPREG_NAME, ui32RegAddr, ui64RegValue, PDUMP_FLAGS_CONTINUOUS);
}

IMG_UINT32 RGXReadReg32(const void *hPrivate, IMG_UINT32 ui32RegAddr)
{
	PVRSRV_RGXDEV_INFO *psDevInfo;
	void *pvRegsBase;
	IMG_UINT32 ui32RegValue;

	PVR_ASSERT(hPrivate != NULL);
	psDevInfo = ((RGX_POWER_LAYER_PARAMS*)hPrivate)->psDevInfo;
	pvRegsBase = psDevInfo->pvRegsBaseKM;

	ui32RegValue = OSReadHWReg32(pvRegsBase, ui32RegAddr);
	PDUMPREGREAD32(RGX_PDUMPREG_NAME, ui32RegAddr, PDUMP_FLAGS_CONTINUOUS);

	return ui32RegValue;
}

IMG_UINT64 RGXReadReg64(const void *hPrivate, IMG_UINT32 ui32RegAddr)
{
	PVRSRV_RGXDEV_INFO *psDevInfo;
	void *pvRegsBase;
	IMG_UINT64 ui64RegValue;

	PVR_ASSERT(hPrivate != NULL);
	psDevInfo = ((RGX_POWER_LAYER_PARAMS*)hPrivate)->psDevInfo;
	pvRegsBase = psDevInfo->pvRegsBaseKM;

	ui64RegValue = OSReadHWReg64(pvRegsBase, ui32RegAddr);
	PDUMPREGREAD64(RGX_PDUMPREG_NAME, ui32RegAddr, PDUMP_FLAGS_CONTINUOUS);

	return ui64RegValue;
}

PVRSRV_ERROR RGXPollReg32(const void *hPrivate,
                          IMG_UINT32 ui32RegAddr,
                          IMG_UINT32 ui32RegValue,
                          IMG_UINT32 ui32RegMask)
{
	PVRSRV_RGXDEV_INFO *psDevInfo;
	void *pvRegsBase;

	PVR_ASSERT(hPrivate != NULL);
	psDevInfo = ((RGX_POWER_LAYER_PARAMS*)hPrivate)->psDevInfo;
	pvRegsBase = psDevInfo->pvRegsBaseKM;

	if (PVRSRVPollForValueKM((IMG_UINT32 *)((IMG_UINT8*)pvRegsBase + ui32RegAddr),
	                         ui32RegValue,
	                         ui32RegMask) != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "RGXPollReg32: Poll for Reg (0x%x) failed", ui32RegAddr));
		return PVRSRV_ERROR_TIMEOUT;
	}

	PDUMPREGPOL(RGX_PDUMPREG_NAME,
	            ui32RegAddr,
	            ui32RegValue,
	            ui32RegMask,
	            PDUMP_FLAGS_CONTINUOUS,
	            PDUMP_POLL_OPERATOR_EQUAL);

	return PVRSRV_OK;
}

PVRSRV_ERROR RGXPollReg64(const void *hPrivate,
                          IMG_UINT32 ui32RegAddr,
                          IMG_UINT64 ui64RegValue,
                          IMG_UINT64 ui64RegMask)
{
	PVRSRV_RGXDEV_INFO *psDevInfo;
	void *pvRegsBase;

	/* Split lower and upper words */
	IMG_UINT32 ui32UpperValue = (IMG_UINT32) (ui64RegValue >> 32);
	IMG_UINT32 ui32LowerValue = (IMG_UINT32) (ui64RegValue);
	IMG_UINT32 ui32UpperMask = (IMG_UINT32) (ui64RegMask >> 32);
	IMG_UINT32 ui32LowerMask = (IMG_UINT32) (ui64RegMask);

	PVR_ASSERT(hPrivate != NULL);
	psDevInfo = ((RGX_POWER_LAYER_PARAMS*)hPrivate)->psDevInfo;
	pvRegsBase = psDevInfo->pvRegsBaseKM;

	if (PVRSRVPollForValueKM((IMG_UINT32 *)((IMG_UINT8*)pvRegsBase + ui32RegAddr + 4),
	                         ui32UpperValue,
	                         ui32UpperMask) != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "RGXPollReg64: Poll for upper part of Reg (0x%x) failed", ui32RegAddr));
		return PVRSRV_ERROR_TIMEOUT;
	}

	PDUMPREGPOL(RGX_PDUMPREG_NAME,
	            ui32RegAddr + 4,
	            ui32UpperValue,
	            ui32UpperMask,
	            PDUMP_FLAGS_CONTINUOUS,
	            PDUMP_POLL_OPERATOR_EQUAL);

	if (PVRSRVPollForValueKM((IMG_UINT32 *)((IMG_UINT8*)pvRegsBase + ui32RegAddr),
	                         ui32LowerValue,
	                         ui32LowerMask) != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "RGXPollReg64: Poll for upper part of Reg (0x%x) failed", ui32RegAddr));
		return PVRSRV_ERROR_TIMEOUT;
	}

	PDUMPREGPOL(RGX_PDUMPREG_NAME,
	            ui32RegAddr,
	            ui32LowerValue,
	            ui32LowerMask,
	            PDUMP_FLAGS_CONTINUOUS,
	            PDUMP_POLL_OPERATOR_EQUAL);

	return PVRSRV_OK;
}

void RGXWaitCycles(const void *hPrivate, IMG_UINT32 ui32Cycles, IMG_UINT32 ui32TimeUs)
{
	PVR_UNREFERENCED_PARAMETER(hPrivate);
	OSWaitus(ui32TimeUs);
	PDUMPIDLWITHFLAGS(ui32Cycles, PDUMP_FLAGS_CONTINUOUS);
}

void RGXCommentLogPower(const void *hPrivate, IMG_CHAR *pszString, ...)
{
#if defined(PDUMP)
	IMG_CHAR szBuffer[PVRSRV_PDUMP_MAX_COMMENT_SIZE];
	va_list argList;

	va_start(argList, pszString);
	vsnprintf(szBuffer, sizeof(szBuffer), pszString, argList);
	va_end(argList);

	PDUMPCOMMENTWITHFLAGS(PDUMP_FLAGS_CONTINUOUS, szBuffer);
	PVR_UNREFERENCED_PARAMETER(hPrivate);
#else
	PVR_UNREFERENCED_PARAMETER(pszString);
	PVR_UNREFERENCED_PARAMETER(hPrivate);
#endif
}

void RGXAcquireKernelMMUPC(const void *hPrivate, IMG_DEV_PHYADDR *psPCAddr)
{
	PVR_ASSERT(hPrivate != NULL);
	*psPCAddr = ((RGX_POWER_LAYER_PARAMS*)hPrivate)->sPCAddr;
}

#if defined(PDUMP)
#if !defined(RGX_FEATURE_SLC_VIVT)
void RGXWriteKernelMMUPC64(const void *hPrivate,
                           IMG_UINT32 ui32PCReg,
                           IMG_UINT32 ui32PCRegAlignShift,
                           IMG_UINT32 ui32PCRegShift,
                           IMG_UINT64 ui64PCVal)
{
	PVRSRV_RGXDEV_INFO *psDevInfo;

	PVR_ASSERT(hPrivate != NULL);
	psDevInfo = ((RGX_POWER_LAYER_PARAMS*)hPrivate)->psDevInfo;

	/* Write the cat-base address */
	OSWriteHWReg64(psDevInfo->pvRegsBaseKM, ui32PCReg, ui64PCVal);

	/* Pdump catbase address */
	MMU_PDumpWritePageCatBase(psDevInfo->psKernelMMUCtx,
	                          RGX_PDUMPREG_NAME,
	                          ui32PCReg,
	                          8,
	                          ui32PCRegAlignShift,
	                          ui32PCRegShift,
	                          PDUMP_FLAGS_CONTINUOUS);
}
#else
void RGXWriteKernelMMUPC32(const void *hPrivate,
                           IMG_UINT32 ui32PCReg,
                           IMG_UINT32 ui32PCRegAlignShift,
                           IMG_UINT32 ui32PCRegShift,
                           IMG_UINT32 ui32PCVal)
{
	PVRSRV_RGXDEV_INFO *psDevInfo;

	PVR_ASSERT(hPrivate != NULL);
	psDevInfo = ((RGX_POWER_LAYER_PARAMS*)hPrivate)->psDevInfo;

	/* Write the cat-base address */
	OSWriteHWReg32(psDevInfo->pvRegsBaseKM, ui32PCReg, ui32PCVal);

	/* Pdump catbase address */
	MMU_PDumpWritePageCatBase(psDevInfo->psKernelMMUCtx,
	                          RGX_PDUMPREG_NAME,
	                          ui32PCReg,
	                          4,
	                          ui32PCRegAlignShift,
	                          ui32PCRegShift,
	                          PDUMP_FLAGS_CONTINUOUS);
}
#endif
#endif /* defined(PDUMP) */

