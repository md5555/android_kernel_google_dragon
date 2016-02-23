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

#include "pvr_hwperf.h"
#include "pvr_debugfs.h"
#include "pvr_uaccess.h"

#include "rgxhwperf.h"
#include "rgx_hwperf_km.h"
#include "rgxapi_km.h"

struct
{
	PVR_DEBUGFS_DIR_DATA *psRootDir;
	PVR_DEBUGFS_ENTRY_DATA *psGpuFilterEntry;
	PVR_DEBUGFS_ENTRY_DATA *psHostFilterEntry;
} g_sHWPerfDebugFs = {0};

static void *HWPerfFilterSeqStart(struct seq_file *psSeqFile,
                                  loff_t *puiPosition)
{
	if (*puiPosition == 0)
	{
		/* We want only one entry in the sequence, one call to show() */
		return (void *) 1;
	}

	PVR_UNREFERENCED_PARAMETER(psSeqFile);

	return NULL;
}

static void HWPerfFilterSeqStop(struct seq_file *psSeqFile, void *pvData)
{
	PVR_UNREFERENCED_PARAMETER(psSeqFile);
	PVR_UNREFERENCED_PARAMETER(pvData);
}

static void *HWPerfFilterSeqNext(struct seq_file *psSeqFile, void *pvData,
                                 loff_t *puiPosition)
{
	PVR_UNREFERENCED_PARAMETER(psSeqFile);
	PVR_UNREFERENCED_PARAMETER(pvData);
	PVR_UNREFERENCED_PARAMETER(puiPosition);
	return NULL;
}

static int HWPerfFilterSeqShow(struct seq_file *psSeqFile, void *pvData)
{
	IMG_CHAR buff[sizeof("0x0123456789abcdef") + 1] = {'\0'};
	RGX_HWPERF_STREAM_ID eStreamId = (RGX_HWPERF_STREAM_ID) psSeqFile->private;
	IMG_UINT64 ui64Filter = 0;
	IMG_HANDLE hHWPerf = NULL;
	PVRSRV_ERROR eError;

	eError = RGXHWPerfLazyConnect(&hHWPerf);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "Could not connect to HWPerf. Most likely"
				" not initialised yet."));
		return -EFAULT;
	}

	eError = RGXHWPerfGetFilter(hHWPerf, eStreamId, &ui64Filter);

	(void) RGXHWPerfFreeConnection(hHWPerf); /* ignore return status */

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Failed to get HWPerf filter value of"
		        " stream %d.", __func__, eStreamId));
		return -EFAULT;
	}

	switch (eStreamId)
	{
		case RGX_HWPERF_STREAM_ID0_FW:
			if (OSSNPrintf(buff, sizeof(buff), "0x%llx\n", ui64Filter) < 0)
			{
				PVR_DPF((PVR_DBG_ERROR, "%s: OSSNPrintf failed to write"
						" filter value of stream %d", __func__, eStreamId));
				return -EFAULT;
			}
			break;
		case RGX_HWPERF_STREAM_ID1_HOST:
			if (OSSNPrintf(buff, sizeof(buff), "0x%x\n",
			        (IMG_UINT32) ui64Filter) < 0)
			{
				PVR_DPF((PVR_DBG_ERROR, "%s: OSSNPrintf failed to write"
						" filter value of stream %d", __func__, eStreamId));
				return -EFAULT;
			}
			break;
		default:
			break;
	}

	seq_puts(psSeqFile, buff);

	PVR_UNREFERENCED_PARAMETER(pvData);
	return 0;
}

static struct seq_operations g_sSeqOps = {
	.start = HWPerfFilterSeqStart,
	.stop  = HWPerfFilterSeqStop,
	.next  = HWPerfFilterSeqNext,
	.show  = HWPerfFilterSeqShow
};

static ssize_t HWPerfFilterSet(const char __user *buffer, size_t count,
                               loff_t uiPosition, void *data)
{
	IMG_CHAR buff[sizeof("0x0123456789abcdef")];
	RGX_HWPERF_STREAM_ID eStreamId = (RGX_HWPERF_STREAM_ID) data;
	IMG_UINT64 ui64Filter = 0;
	IMG_UINT uiBuffLen = 0;
	PVRSRV_ERROR eError;
	IMG_HANDLE hHWPerf;

	switch (eStreamId)
	{
		case RGX_HWPERF_STREAM_ID0_FW:
			uiBuffLen = sizeof("0x") + sizeof(IMG_UINT64) * 2;
			break;
		case RGX_HWPERF_STREAM_ID1_HOST:
			uiBuffLen = sizeof("0x") + sizeof(IMG_UINT32) * 2;
			break;
		default:
			return count;
	}

	if (count > uiBuffLen)
	{
		PVR_DPF((PVR_DBG_WARNING, "HWPerfFilterSet: String too long (%zd)",
		         count));
		return -EINVAL;
	}

	if (pvr_copy_from_user(buff, buffer, count))
	{
		PVR_DPF((PVR_DBG_WARNING, "HWPerfFilterSet: Copy of user data failed"));
		return -EFAULT;
	}
	buff[count] = '\0';

	if (count > uiBuffLen - 2 && !(buff[0] == '0' && buff[1] == 'x'))
	{
		PVR_DPF((PVR_DBG_WARNING, "HWPerfFilterSet: String too long (%zd)",
		         count));
		return -EINVAL;
	}

	if (sscanf(buff, "%llx", &ui64Filter) != 1)
	{
		PVR_DPF((PVR_DBG_WARNING, "HWPerfFilterSet: Invalid input data"));
		return -EINVAL;
	}

	if (RGXHWPerfLazyConnect(&hHWPerf) != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "Could not connect to HWPerf. Most likely"
				" not initialised yet."));
		return -EFAULT;
	}

	eError = RGXHWPerfControl(hHWPerf, eStreamId, IMG_FALSE, ui64Filter);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_WARNING, "HWPerfFilterSet: Failed to set filter in"
		         "for the GPU"));
		return -EFAULT;
	}

	(void) RGXHWPerfFreeConnection(hHWPerf); /* ignore return status */

	PVR_UNREFERENCED_PARAMETER(uiPosition);
	return count;
}

PVRSRV_ERROR PVRSRVHWperfCreateDebugFs(void)
{
	int iError;

	iError = PVRDebugFSCreateEntryDir("hwperf", NULL,
	                                  &g_sHWPerfDebugFs.psRootDir);
	if (iError)
	{
		PVR_DPF((PVR_DBG_WARNING, "Failed to create \"hwperf\" DebugFS"
				 "directory."));
		return iError;
	}

	iError = PVRDebugFSCreateEntry("gpu_filter", g_sHWPerfDebugFs.psRootDir,
	                               &g_sSeqOps, HWPerfFilterSet,
	                               (void *) RGX_HWPERF_STREAM_ID0_FW,
	                               &g_sHWPerfDebugFs.psGpuFilterEntry);
	if (iError)
	{
		PVR_DPF((PVR_DBG_WARNING, "Failed to create \"gpu_filter\" DebugFS"
				 " entry."));
	}

	iError = PVRDebugFSCreateEntry("host_filter", g_sHWPerfDebugFs.psRootDir,
	                               &g_sSeqOps, HWPerfFilterSet,
	                               (void *) RGX_HWPERF_STREAM_ID1_HOST,
	                               &g_sHWPerfDebugFs.psHostFilterEntry);
	if (iError)
	{
		PVR_DPF((PVR_DBG_WARNING, "Failed to create \"host_filter\""
				" DebugFS entry."));
	}

	return iError == 0 ? PVRSRV_OK : iError; /* return ok or last error */
}

void PVRSRVHWperfDestroyDebugFs(void)
{
	if (g_sHWPerfDebugFs.psGpuFilterEntry)
	{
		PVRDebugFSRemoveEntry(&g_sHWPerfDebugFs.psGpuFilterEntry);
	}

	if (g_sHWPerfDebugFs.psHostFilterEntry)
	{
		PVRDebugFSRemoveEntry(&g_sHWPerfDebugFs.psHostFilterEntry);
	}

	if (g_sHWPerfDebugFs.psRootDir)
	{
		PVRDebugFSRemoveEntryDir(&g_sHWPerfDebugFs.psRootDir);
	}
}
