/*************************************************************************/ /*!
@File
@Title          PVR Bridge Module (kernel side)
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    Receives calls from the user portion of services and
                despatches them to functions in the kernel portion.
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

#include <linux/mm_types.h>

#include "img_defs.h"
#include "pvr_bridge.h"
#include "connection_server.h"
#include "syscommon.h"
#include "pvr_debug.h"
#include "pvr_debugfs.h"
#include "private_data.h"
#include "linkage.h"
#include "pmr.h"

#if defined(SUPPORT_DRM)
#include <drm/drmP.h>
#include "pvr_drm.h"
#endif /* defined(SUPPORT_DRM) */

/* RGX: */
#if defined(SUPPORT_RGX)
#include "rgx_bridge.h"
#endif

#include "srvcore.h"
#include "common_srvcore_bridge.h"

#if defined(MODULE_TEST)
/************************************************************************/
// additional includes for services testing
/************************************************************************/
#include "pvr_test_bridge.h"
#include "kern_test.h"
/************************************************************************/
// end of additional includes
/************************************************************************/
#endif

/* WARNING!
 * The mmap code has its own mutex, to prevent a possible deadlock,
 * when using gPVRSRVLock.
 * The Linux kernel takes the mm->mmap_sem before calling the mmap
 * entry points (PVRMMap, MMapVOpen, MMapVClose), but the ioctl
 * entry point may take mm->mmap_sem during fault handling, or 
 * before calling get_user_pages.  If gPVRSRVLock was used in the
 * mmap entry points, a deadlock could result, due to the ioctl
 * and mmap code taking the two locks in different orders.
 * As a corollary to this, the mmap entry points must not call
 * any driver code that relies on gPVRSRVLock is held.
 */
static DEFINE_MUTEX(g_sMMapMutex);

#if defined(DEBUG_BRIDGE_KM)
static PVR_DEBUGFS_ENTRY_DATA *gpsPVRDebugFSBridgeStatsEntry = NULL;
static struct seq_operations gsBridgeStatsReadOps;
#endif

/* These will go when full bridge gen comes in */
#if defined(PDUMP)
PVRSRV_ERROR InitPDUMPCTRLBridge(void);
PVRSRV_ERROR DeinitPDUMPCTRLBridge(void);
PVRSRV_ERROR InitPDUMPBridge(void);
PVRSRV_ERROR DeinitPDUMPBridge(void);
PVRSRV_ERROR InitRGXPDUMPBridge(void);
PVRSRV_ERROR DeinitRGXPDUMPBridge(void);
#endif
#if defined(SUPPORT_DISPLAY_CLASS)
PVRSRV_ERROR InitDCBridge(void);
PVRSRV_ERROR DeinitDCBridge(void);
#endif
PVRSRV_ERROR InitMMBridge(void);
PVRSRV_ERROR DeinitMMBridge(void);
PVRSRV_ERROR InitCMMBridge(void);
PVRSRV_ERROR DeinitCMMBridge(void);
PVRSRV_ERROR InitPDUMPMMBridge(void);
PVRSRV_ERROR DeinitPDUMPMMBridge(void);
PVRSRV_ERROR InitSRVCOREBridge(void);
PVRSRV_ERROR DeinitSRVCOREBridge(void);
PVRSRV_ERROR InitSYNCBridge(void);
PVRSRV_ERROR DeinitSYNCBridge(void);
#if defined(SUPPORT_SERVER_SYNC)
#if defined(SUPPORT_INSECURE_EXPORT)
PVRSRV_ERROR InitSYNCEXPORTBridge(void);
PVRSRV_ERROR DeinitSYNCEXPORTBridge(void);
#endif
#if defined(SUPPORT_SECURE_EXPORT)
PVRSRV_ERROR InitSYNCSEXPORTBridge(void);
PVRSRV_ERROR DeinitSYNCSEXPORTBridge(void);
#endif
#endif /* defined(SUPPORT_SERVER_SYNC) */
#if defined (SUPPORT_RGX)
PVRSRV_ERROR InitRGXINITBridge(void);
PVRSRV_ERROR DeinitRGXINITBridge(void);
PVRSRV_ERROR InitRGXTA3DBridge(void);
PVRSRV_ERROR DeinitRGXTA3DBridge(void);
PVRSRV_ERROR InitRGXTQBridge(void);
PVRSRV_ERROR DeinitRGXTQBridge(void);
PVRSRV_ERROR InitRGXCMPBridge(void);
PVRSRV_ERROR DeinitRGXCMPBridge(void);
PVRSRV_ERROR InitBREAKPOINTBridge(void);
PVRSRV_ERROR DeinitBREAKPOINTBridge(void);
PVRSRV_ERROR InitDEBUGMISCBridge(void);
PVRSRV_ERROR DeinitDEBUGMISCBridge(void);
PVRSRV_ERROR InitRGXHWPERFBridge(void);
PVRSRV_ERROR DeinitRGXHWPERFBridge(void);
#if defined(RGX_FEATURE_RAY_TRACING)
PVRSRV_ERROR InitRGXRAYBridge(void);
PVRSRV_ERROR DeinitRGXRAYBridge(void);
#endif /* RGX_FEATURE_RAY_TRACING */
PVRSRV_ERROR InitREGCONFIGBridge(void);
PVRSRV_ERROR DeinitREGCONFIGBridge(void);
PVRSRV_ERROR InitTIMERQUERYBridge(void);
PVRSRV_ERROR DeinitTIMERQUERYBridge(void);
PVRSRV_ERROR InitRGXKICKSYNCBridge(void);
PVRSRV_ERROR DeinitRGXKICKSYNCBridge(void);
#endif /* SUPPORT_RGX */
PVRSRV_ERROR InitCACHEGENERICBridge(void);
PVRSRV_ERROR DeinitCACHEGENERICBridge(void);
#if defined(SUPPORT_SECURE_EXPORT)
PVRSRV_ERROR InitSMMBridge(void);
PVRSRV_ERROR DeinitSMMBridge(void);
#endif
PVRSRV_ERROR InitHTBUFFERBridge(void);
PVRSRV_ERROR DeinitHTBUFFERBridge(void);
PVRSRV_ERROR InitPVRTLBridge(void);
PVRSRV_ERROR DeinitPVRTLBridge(void);
#if defined(PVR_RI_DEBUG)
PVRSRV_ERROR InitRIBridge(void);
PVRSRV_ERROR DeinitRIBridge(void);
#endif
#if defined(SUPPORT_PAGE_FAULT_DEBUG)
PVRSRV_ERROR InitDEVICEMEMHISTORYBridge(void);
PVRSRV_ERROR DeinitDEVICEMEMHISTORYBridge(void);
#endif
PVRSRV_ERROR InitDMABUFBridge(void);
PVRSRV_ERROR DeinitDMABUFBridge(void);
#if defined(SUPPORT_VALIDATION)
PVRSRV_ERROR InitVALIDATIONBridge(void);
#endif
#if defined(PVR_TESTING_UTILS)
PVRSRV_ERROR InitTUTILSBridge(void);
PVRSRV_ERROR DeinitTUTILSBridge(void);
#endif

PVRSRV_ERROR
LinuxBridgeInit(void)
{
	PVRSRV_ERROR eError;
#if defined(DEBUG_BRIDGE_KM)
	IMG_INT iResult;

	iResult = PVRDebugFSCreateEntry("bridge_stats",
					NULL,
					&gsBridgeStatsReadOps,
					NULL,
					&g_BridgeDispatchTable[0],
					&gpsPVRDebugFSBridgeStatsEntry);
	if (iResult != 0)
	{
		return PVRSRV_ERROR_OUT_OF_MEMORY;
	}
#endif

	BridgeDispatchTableStartOffsetsInit();

	eError = InitSRVCOREBridge();
	if (eError != PVRSRV_OK)
	{
		return eError;
	}

	eError = InitSYNCBridge();
	if (eError != PVRSRV_OK)
	{
		return eError;
	}

#if defined(SUPPORT_SERVER_SYNC)
#if defined(SUPPORT_INSECURE_EXPORT)
	eError = InitSYNCEXPORTBridge();
	if (eError != PVRSRV_OK)
	{
		return eError;
	}
#endif
#if defined(SUPPORT_SECURE_EXPORT)
	eError = InitSYNCSEXPORTBridge();
	if (eError != PVRSRV_OK)
	{
		return eError;
	}
#endif
#endif /* defined(SUPPORT_SERVER_SYNC) */

#if defined(PDUMP)
	eError = InitPDUMPCTRLBridge();
	if (eError != PVRSRV_OK)
	{
		return eError;
	}
#endif

	eError = InitMMBridge();
	if (eError != PVRSRV_OK)
	{
		return eError;
	}
	eError = InitCMMBridge();
	if (eError != PVRSRV_OK)
	{
		return eError;
	}

#if defined(PDUMP)
	eError = InitPDUMPMMBridge();
	if (eError != PVRSRV_OK)
	{
		return eError;
	}
	eError = InitPDUMPBridge();
	if (eError != PVRSRV_OK)
	{
		return eError;
	}
#endif

	eError = InitDMABUFBridge();
	if (eError != PVRSRV_OK)
	{
		return eError;
	}

#if defined(SUPPORT_DISPLAY_CLASS)
	eError = InitDCBridge();
	if (eError != PVRSRV_OK)
	{
		return eError;
	}
#endif

	eError = InitCACHEGENERICBridge();
	if (eError != PVRSRV_OK)
	{
		return eError;
	}

#if defined(SUPPORT_SECURE_EXPORT)
	eError = InitSMMBridge();
	if (eError != PVRSRV_OK)
	{
		return eError;
	}
#endif

	eError = InitHTBUFFERBridge();
	if (eError != PVRSRV_OK)
	{
		return eError;
	}

	eError = InitPVRTLBridge();
	if (eError != PVRSRV_OK)
	{
		return eError;
	}

	#if defined(PVR_RI_DEBUG)
	eError = InitRIBridge();
	if (eError != PVRSRV_OK)
	{
		return eError;
	}
	#endif

#if defined(SUPPORT_VALIDATION)
	eError = InitVALIDATIONBridge();
	if (eError != PVRSRV_OK)
	{
		return eError;
	}
#endif

#if defined(PVR_TESTING_UTILS)
	eError = InitTUTILSBridge();
	if (eError != PVRSRV_OK)
	{
		return eError;
	}
#endif

#if defined(SUPPORT_PAGE_FAULT_DEBUG)
	eError = InitDEVICEMEMHISTORYBridge();
	if (eError != PVRSRV_OK)
	{
		return eError;
	}
#endif

	#if defined (SUPPORT_RGX)
	eError = InitRGXTQBridge();
	if (eError != PVRSRV_OK)
	{
		return eError;
	}

#if defined(RGX_FEATURE_COMPUTE)
	eError = InitRGXCMPBridge();
	if (eError != PVRSRV_OK)
	{
		return eError;
	}
#endif

	eError = InitRGXINITBridge();
	if (eError != PVRSRV_OK)
	{
		return eError;
	}

	eError = InitRGXTA3DBridge();
	if (eError != PVRSRV_OK)
	{
		return eError;
	}

	eError = InitBREAKPOINTBridge();
	if (eError != PVRSRV_OK)
	{
		return eError;
	}

	eError = InitDEBUGMISCBridge();
	if (eError != PVRSRV_OK)
	{
		return eError;
	}

#if defined(PDUMP)
	eError = InitRGXPDUMPBridge();
	if (eError != PVRSRV_OK)
	{
		return eError;
	}
#endif

	eError = InitRGXHWPERFBridge();
	if (eError != PVRSRV_OK)
	{
		return eError;
	}

#if defined(RGX_FEATURE_RAY_TRACING)
	eError = InitRGXRAYBridge();
	if (eError != PVRSRV_OK)
	{
		return eError;
	}
#endif /* RGX_FEATURE_RAY_TRACING */

	eError = InitREGCONFIGBridge();
	if (eError != PVRSRV_OK)
	{
		return eError;
	}

	eError = InitTIMERQUERYBridge();
	if (eError != PVRSRV_OK)
	{
		return eError;
	}

	eError = InitRGXKICKSYNCBridge();
	if (eError != PVRSRV_OK)
	{
		return eError;
	}


#endif /* SUPPORT_RGX */

	return eError;
}

PVRSRV_ERROR
LinuxBridgeDeInit(void)
{
	PVRSRV_ERROR eError;
#if defined(DEBUG_BRIDGE_KM)
	if (gpsPVRDebugFSBridgeStatsEntry != NULL)
	{
		PVRDebugFSRemoveEntry(&gpsPVRDebugFSBridgeStatsEntry);
	}
#endif

	eError = DeinitSRVCOREBridge();
	if (eError != PVRSRV_OK)
	{
		return eError;
	}

	eError = DeinitSYNCBridge();
	if (eError != PVRSRV_OK)
	{
		return eError;
	}

#if defined(SUPPORT_SERVER_SYNC)
#if defined(SUPPORT_INSECURE_EXPORT)
	eError = DeinitSYNCEXPORTBridge();
	if (eError != PVRSRV_OK)
	{
		return eError;
	}
#endif
#if defined(SUPPORT_SECURE_EXPORT)
	eError = DeinitSYNCSEXPORTBridge();
	if (eError != PVRSRV_OK)
	{
		return eError;
	}
#endif
#endif /* defined(SUPPORT_SERVER_SYNC) */

#if defined(PDUMP)
	eError = DeinitPDUMPCTRLBridge();
	if (eError != PVRSRV_OK)
	{
		return eError;
	}
#endif

	eError = DeinitMMBridge();
	if (eError != PVRSRV_OK)
	{
		return eError;
	}
	eError = DeinitCMMBridge();
	if (eError != PVRSRV_OK)
	{
		return eError;
	}

#if defined(PDUMP)
	eError = DeinitPDUMPMMBridge();
	if (eError != PVRSRV_OK)
	{
		return eError;
	}
	eError = DeinitPDUMPBridge();
	if (eError != PVRSRV_OK)
	{
		return eError;
	}
#endif

	eError = DeinitDMABUFBridge();
	if (eError != PVRSRV_OK)
	{
		return eError;
	}

#if defined(PVR_TESTING_UTILS)
	eError = DeinitTUTILSBridge();
	if (eError != PVRSRV_OK)
	{
		return eError;
	}
#endif

#if defined(SUPPORT_DISPLAY_CLASS)
	eError = DeinitDCBridge();
	if (eError != PVRSRV_OK)
	{
		return eError;
	}
#endif

	eError = DeinitCACHEGENERICBridge();
	if (eError != PVRSRV_OK)
	{
		return eError;
	}

#if defined(SUPPORT_SECURE_EXPORT)
	eError = DeinitSMMBridge();
	if (eError != PVRSRV_OK)
	{
		return eError;
	}
#endif

	eError = DeinitHTBUFFERBridge();
	if (eError != PVRSRV_OK)
	{
		return eError;
	}

	eError = DeinitPVRTLBridge();
	if (eError != PVRSRV_OK)
	{
		return eError;
	}

	#if defined(PVR_RI_DEBUG)
	eError = DeinitRIBridge();
	if (eError != PVRSRV_OK)
	{
		return eError;
	}
	#endif

#if defined(SUPPORT_PAGE_FAULT_DEBUG)
	eError = DeinitDEVICEMEMHISTORYBridge();
	if (eError != PVRSRV_OK)
	{
		return eError;
	}
#endif

	#if defined (SUPPORT_RGX)
	eError = DeinitRGXTQBridge();
	if (eError != PVRSRV_OK)
	{
		return eError;
	}

#if defined(RGX_FEATURE_COMPUTE)
	eError = DeinitRGXCMPBridge();
	if (eError != PVRSRV_OK)
	{
		return eError;
	}
#endif

	eError = DeinitRGXINITBridge();
	if (eError != PVRSRV_OK)
	{
		return eError;
	}

	eError = DeinitRGXTA3DBridge();
	if (eError != PVRSRV_OK)
	{
		return eError;
	}

	eError = DeinitBREAKPOINTBridge();
	if (eError != PVRSRV_OK)
	{
		return eError;
	}

	eError = DeinitDEBUGMISCBridge();
	if (eError != PVRSRV_OK)
	{
		return eError;
	}

#if defined(PDUMP)
	eError = DeinitRGXPDUMPBridge();
	if (eError != PVRSRV_OK)
	{
		return eError;
	}
#endif

	eError = DeinitRGXHWPERFBridge();
	if (eError != PVRSRV_OK)
	{
		return eError;
	}

#if defined(RGX_FEATURE_RAY_TRACING)
	eError = DeinitRGXRAYBridge();
	if (eError != PVRSRV_OK)
	{
		return eError;
	}
#endif /* RGX_FEATURE_RAY_TRACING */

	eError = DeinitREGCONFIGBridge();
	if (eError != PVRSRV_OK)
	{
		return eError;
	}

	eError = DeinitTIMERQUERYBridge();
	if (eError != PVRSRV_OK)
	{
		return eError;
	}

	eError = DeinitRGXKICKSYNCBridge();
	if (eError != PVRSRV_OK)
	{
		return eError;
	}


#endif /* SUPPORT_RGX */

	return eError;
}

#if defined(DEBUG_BRIDGE_KM)
static void *BridgeStatsSeqStart(struct seq_file *psSeqFile, loff_t *puiPosition)
{
	PVRSRV_BRIDGE_DISPATCH_TABLE_ENTRY *psDispatchTable = (PVRSRV_BRIDGE_DISPATCH_TABLE_ENTRY *)psSeqFile->private;

	OSAcquireBridgeLock();

	if (psDispatchTable == NULL || (*puiPosition) > BRIDGE_DISPATCH_TABLE_ENTRY_COUNT)
	{
		return NULL;
	}

	if ((*puiPosition) == 0) 
	{
		return SEQ_START_TOKEN;
	}

	return &(psDispatchTable[(*puiPosition) - 1]);
}

static void BridgeStatsSeqStop(struct seq_file *psSeqFile, void *pvData)
{
	PVR_UNREFERENCED_PARAMETER(psSeqFile);
	PVR_UNREFERENCED_PARAMETER(pvData);

	OSReleaseBridgeLock();
}

static void *BridgeStatsSeqNext(struct seq_file *psSeqFile,
			       void *pvData,
			       loff_t *puiPosition)
{
	PVRSRV_BRIDGE_DISPATCH_TABLE_ENTRY *psDispatchTable = (PVRSRV_BRIDGE_DISPATCH_TABLE_ENTRY *)psSeqFile->private;
	loff_t uiItemAskedFor = *puiPosition; /* puiPosition on entry is the index to return */

	PVR_UNREFERENCED_PARAMETER(pvData);

	/* Is the item asked for (starts at 0) a valid table index? */
	if (uiItemAskedFor < BRIDGE_DISPATCH_TABLE_ENTRY_COUNT)
	{
		(*puiPosition)++; /* on exit it is the next seq index to ask for */
		return &(psDispatchTable[uiItemAskedFor]);
	}

	/* Now passed the end of the table to indicate stop */
	return NULL;
}

static int BridgeStatsSeqShow(struct seq_file *psSeqFile, void *pvData)
{
	if (pvData == SEQ_START_TOKEN)
	{
		seq_printf(psSeqFile,
			   "Total ioctl call count = %u\n"
			   "Total number of bytes copied via copy_from_user = %u\n"
			   "Total number of bytes copied via copy_to_user = %u\n"
			   "Total number of bytes copied via copy_*_user = %u\n\n"
			   "%-60s | %-48s | %10s | %20s | %10s\n",
			   g_BridgeGlobalStats.ui32IOCTLCount,
			   g_BridgeGlobalStats.ui32TotalCopyFromUserBytes,
			   g_BridgeGlobalStats.ui32TotalCopyToUserBytes,
			   g_BridgeGlobalStats.ui32TotalCopyFromUserBytes + g_BridgeGlobalStats.ui32TotalCopyToUserBytes,
			   "  #: Bridge Name",
			   "Wrapper Function",
			   "Call Count",
			   "copy_from_user Bytes",
			   "copy_to_user Bytes");
	}
	else if (pvData != NULL)
	{
		PVRSRV_BRIDGE_DISPATCH_TABLE_ENTRY *psEntry = (	PVRSRV_BRIDGE_DISPATCH_TABLE_ENTRY *)pvData;

		seq_printf(psSeqFile,
			   "%3d: %-60s   %-48s   %-10u   %-20u   %-10u\n",
			   (IMG_UINT32)(((size_t)psEntry-(size_t)g_BridgeDispatchTable)/sizeof(PVRSRV_BRIDGE_DISPATCH_TABLE_ENTRY)),
			   psEntry->pszIOCName,
			   psEntry->pszFunctionName,
			   psEntry->ui32CallCount,
			   psEntry->ui32CopyFromUserTotalBytes,
			   psEntry->ui32CopyToUserTotalBytes);
	}

	return 0;
}

static struct seq_operations gsBridgeStatsReadOps =
{
	.start = BridgeStatsSeqStart,
	.stop = BridgeStatsSeqStop,
	.next = BridgeStatsSeqNext,
	.show = BridgeStatsSeqShow,
};
#endif /* defined(DEBUG_BRIDGE_KM) */


#if defined(SUPPORT_DRM)
int
PVRSRV_BridgeDispatchKM(struct drm_device unref__ *dev, void *arg, struct drm_file *pDRMFile)
#else
long
PVRSRV_BridgeDispatchKM(struct file *pFile, unsigned int ioctlCmd, unsigned long arg)
#endif
{
#if defined(SUPPORT_DRM)
	struct file *pFile = PVR_FILE_FROM_DRM_FILE(pDRMFile);
#else
	PVRSRV_BRIDGE_PACKAGE *psBridgePackageUM = (PVRSRV_BRIDGE_PACKAGE *)arg;
	PVRSRV_BRIDGE_PACKAGE sBridgePackageKM;
#endif
	PVRSRV_BRIDGE_PACKAGE *psBridgePackageKM;
	CONNECTION_DATA *psConnection = LinuxConnectionFromFile(pFile);

	if(psConnection == NULL)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Connection is closed", __FUNCTION__));
		return -EFAULT;
	}

	if(OSGetDriverSuspended())
	{
		return -EINTR;
	}

#if defined(SUPPORT_DRM)
	psBridgePackageKM = (PVRSRV_BRIDGE_PACKAGE *)arg;
	PVR_ASSERT(psBridgePackageKM != NULL);
#else

	psBridgePackageKM = &sBridgePackageKM;

	if (!OSAccessOK(PVR_VERIFY_WRITE,
				   psBridgePackageUM,
				   sizeof(PVRSRV_BRIDGE_PACKAGE)))
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Received invalid pointer to function arguments",
				 __FUNCTION__));

		return -EFAULT;
	}
	
	if (OSCopyFromUser(NULL,
					  psBridgePackageKM,
					  psBridgePackageUM,
					  sizeof(PVRSRV_BRIDGE_PACKAGE))
	  != PVRSRV_OK)
	{
		return -EFAULT;
	}

	if (PVRSRV_GET_BRIDGE_ID(ioctlCmd) != psBridgePackageKM->ui32BridgeID ||
	    psBridgePackageKM->ui32Size != sizeof(PVRSRV_BRIDGE_PACKAGE))
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Inconsistent data passed from user space",
				__FUNCTION__));
		return PVRSRV_ERROR_INVALID_PARAMS;
	}
#endif

#if defined(DEBUG_BRIDGE_CALLS)
	{
		IMG_UINT32 mangledID;
		mangledID = psBridgePackageKM->ui32BridgeID;

		psBridgePackageKM->ui32BridgeID = PVRSRV_GET_BRIDGE_ID(psBridgePackageKM->ui32BridgeID);

		PVR_DPF((PVR_DBG_WARNING, "%s: Bridge ID (x%8x) %8u (mangled: x%8x) ", __FUNCTION__, psBridgePackageKM->ui32BridgeID, psBridgePackageKM->ui32BridgeID, mangledID));
	}
#else
		psBridgePackageKM->ui32BridgeID = PVRSRV_GET_BRIDGE_ID(psBridgePackageKM->ui32BridgeID);
#endif

	return BridgedDispatchKM(psConnection, psBridgePackageKM);
}


#if defined(CONFIG_COMPAT)
#if defined(SUPPORT_DRM)
int
#else
long
#endif
PVRSRV_BridgeCompatDispatchKM(struct file *pFile,
                              unsigned int ioctlCmd,
                              unsigned long arg)
{
	struct bridge_package_from_32
	{
		IMG_UINT32				bridge_id;			/*!< ioctl bridge group */
		IMG_UINT32				function_id;        /*!< ioctl function index */
		IMG_UINT32				size;				/*!< size of structure */
		IMG_UINT32				addr_param_in;		/*!< input data buffer */ 
		IMG_UINT32				in_buffer_size;		/*!< size of input data buffer */
		IMG_UINT32				addr_param_out;		/*!< output data buffer */
		IMG_UINT32				out_buffer_size;	/*!< size of output data buffer */
	};

	PVRSRV_BRIDGE_PACKAGE params_for_64;
	struct bridge_package_from_32 params;
 	struct bridge_package_from_32 * const params_addr = &params;
	CONNECTION_DATA *psConnection = LinuxConnectionFromFile(pFile);

	if(psConnection == NULL)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Connection is closed", __FUNCTION__));
		return -EFAULT;
	}

	if(OSGetDriverSuspended())
	{
		return -EINTR;
	}

	/* make sure there is no padding inserted by compiler */
	BUILD_BUG_ON(sizeof(struct bridge_package_from_32) != 7 * sizeof(IMG_UINT32));

	if(!OSAccessOK(PVR_VERIFY_READ, (void *) arg,
				   sizeof(struct bridge_package_from_32)))
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Received invalid pointer to function arguments",
				 __FUNCTION__));

		return -EFAULT;
	}
	
	if(OSCopyFromUser(NULL, params_addr, (void*) arg,
					  sizeof(struct bridge_package_from_32))
	   != PVRSRV_OK)
	{
		return -EFAULT;
	}

#if defined(SUPPORT_DRM)
	if (params_addr->size != sizeof(struct bridge_package_from_32))
#else
	if (PVRSRV_GET_BRIDGE_ID(ioctlCmd) != PVRSRV_GET_BRIDGE_ID(params_addr->bridge_id) ||
	    params_addr->size != sizeof(struct bridge_package_from_32))
#endif
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Inconsistent data passed from user space",
		        __FUNCTION__));
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	params_for_64.ui32BridgeID = PVRSRV_GET_BRIDGE_ID(params_addr->bridge_id);
	params_for_64.ui32FunctionID = params_addr->function_id;
	params_for_64.ui32Size = sizeof(params_for_64);
	params_for_64.pvParamIn = (void*) ((size_t) params_addr->addr_param_in);
	params_for_64.pvParamOut = (void*) ((size_t) params_addr->addr_param_out);
	params_for_64.ui32InBufferSize = params_addr->in_buffer_size;
	params_for_64.ui32OutBufferSize = params_addr->out_buffer_size;

	return BridgedDispatchKM(psConnection, &params_for_64);
}
#endif /* defined(CONFIG_COMPAT) */

int
PVRSRV_MMap(struct file *pFile, struct vm_area_struct *ps_vma)
{
	CONNECTION_DATA *psConnection = LinuxConnectionFromFile(pFile);
	IMG_HANDLE hSecurePMRHandle = (IMG_HANDLE)((uintptr_t)ps_vma->vm_pgoff);
	PMR *psPMR;
	PVRSRV_ERROR eError;

	if(psConnection == NULL)
	{
		PVR_DPF((PVR_DBG_ERROR, "Invalid connection data"));
		return -ENOENT;
	}

	/*
	 * The bridge lock used here to protect PVRSRVLookupHandle is replaced
	 * by a specific lock considering that the handle functions have now
	 * their own lock. This change was necessary to solve the lockdep issues
	 * related with the PVRSRV_MMap.
	 */
	mutex_lock(&g_sMMapMutex);
	PMRLock();

	eError = PVRSRVLookupHandle(psConnection->psHandleBase,
								(void **)&psPMR,
								hSecurePMRHandle,
								PVRSRV_HANDLE_TYPE_PHYSMEM_PMR);
	if (eError != PVRSRV_OK)
	{
		goto e0;
	}

	/* Note: PMRMMapPMR will take a reference on the PMR */
	eError = PMRMMapPMR(psPMR, ps_vma);
	if (eError != PVRSRV_OK)
	{
		goto e1;
	}

	PMRUnlock();
	mutex_unlock(&g_sMMapMutex);

	return 0;

e1:
	PMRUnrefPMR(psPMR);
	goto em1;
e0:
	PVR_DPF((PVR_DBG_ERROR, "Error in mmap critical section"));
	PMRUnlock();
em1:
	mutex_unlock(&g_sMMapMutex);

	PVR_DPF((PVR_DBG_ERROR, "Unable to translate error %d", eError));
	PVR_ASSERT(eError != PVRSRV_OK);

	return -ENOENT; // -EAGAIN // or what?
}
