/*************************************************************************/ /*!
@File
@Title          PowerVR devfreq device implementation
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    Linux module setup
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

#include <linux/devfreq.h>
#include <linux/version.h>

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,13,0))
#include <linux/pm_opp.h>
#define OPP_GET_FREQ dev_pm_opp_get_freq
#define OPP_GET_VOLTAGE dev_pm_opp_get_voltage
#define OPP_ADD dev_pm_opp_add
#define OPP_FIND_FREQ_FLOOR dev_pm_opp_find_freq_floor
#define OPP_STRUCT dev_pm_opp
#else
#include <linux/opp.h>
#define OPP_GET_FREQ opp_get_freq
#define OPP_GET_VOLTAGE opp_get_voltage
#define OPP_ADD opp_add
#define OPP_FIND_FREQ_FLOOR opp_find_freq_floor
#define OPP_STRUCT opp
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0))
#define DEVFREQ_GOVERNOR "img_governor"
#else
#define DEVFREQ_GOVERNOR &devfreq_img_governor
#endif

#include "pvr_dvfs_governor.h"

#include "pvrsrv_device.h"
#include "syscommon.h"
#include "rgxdevice.h"
#include "rgxinit.h"
#include "pvr_dvfs_device.h"
#include "power.h"

#include <linux/device.h>
#if defined(LDM_PLATFORM)
#include <linux/platform_device.h>
#define DEVICE_STRUCT platform_device
#elif defined(LDM_PCI)
#define DEVICE_STRUCT pci_dev
#include <linux/pci.h>
#endif

static PVRSRV_DEVICE_NODE* gpsDeviceNode = NULL;

static IMG_INT32 devfreq_target(struct device *dev, long unsigned *requested_freq, IMG_UINT32 flags)
{
	RGX_DATA		*psRGXData = (RGX_DATA*) gpsDeviceNode->psDevConfig->hDevData;
	IMG_DVFS_DEVICE		*psDVFSDevice = &gpsDeviceNode->psDevConfig->sDVFS.sDVFSDevice;
	IMG_DVFS_DEVICE_CFG	*psDVFSDeviceCfg = &gpsDeviceNode->psDevConfig->sDVFS.sDVFSDeviceCfg;
	RGX_TIMING_INFORMATION	*psRGXTimingInfo = psRGXData->psRGXTimingInfo;
	IMG_UINT32		ui32Freq, ui32CurFreq, ui32Volt;
	struct OPP_STRUCT	*opp;

	rcu_read_lock();
	opp = devfreq_recommended_opp(dev, requested_freq, flags);
	if (IS_ERR(opp)) {
		rcu_read_unlock();
		PVR_DPF((PVR_DBG_ERROR, "Invalid OPP"));
		return PTR_ERR(opp);
	}

	ui32Freq = OPP_GET_FREQ(opp);
	ui32Volt = OPP_GET_VOLTAGE(opp);
	rcu_read_unlock();

	ui32CurFreq = psRGXTimingInfo->ui32CoreClockSpeed;

	if (ui32CurFreq == ui32Freq)
	{
		return 0;
	}

	if (!psDVFSDevice->bEnabled)
	{
		return 0;
	}

	PVRSRVDevicePreClockSpeedChange(0, psDVFSDeviceCfg->bIdleReq, NULL);

	/* Increasing frequency, change voltage first */
	if (ui32Freq > ui32CurFreq)
	{
		psDVFSDeviceCfg->pfnSetVoltage(ui32Volt);
	}

	psDVFSDeviceCfg->pfnSetFrequency(ui32Freq);

	/* Decreasing frequency, change frequency first */
	if (ui32Freq < ui32CurFreq)
	{
		psDVFSDeviceCfg->pfnSetVoltage(ui32Volt);
	}

	psRGXTimingInfo->ui32CoreClockSpeed = ui32Freq;

	PVRSRVDevicePostClockSpeedChange(0, psDVFSDeviceCfg->bIdleReq, NULL);

	return 0;
}

static IMG_INT32 devfreq_get_dev_status(struct device *dev, struct devfreq_dev_status *stat)
{
	PVRSRV_RGXDEV_INFO*	psDevInfo = gpsDeviceNode->pvDevice;
	IMG_DVFS_DEVICE		*psDVFSDevice = &gpsDeviceNode->psDevConfig->sDVFS.sDVFSDevice;
	RGX_DATA		*psRGXData = (RGX_DATA*) gpsDeviceNode->psDevConfig->hDevData;
	RGX_TIMING_INFORMATION	*psRGXTimingInfo = psRGXData->psRGXTimingInfo;
	RGXFWIF_GPU_UTIL_STATS	sGpuUtilStats;
	PVRSRV_ERROR eError = PVRSRV_OK;

	stat->current_frequency = psRGXTimingInfo->ui32CoreClockSpeed;

	if (psDevInfo->pfnGetGpuUtilStats == NULL)
	{
		return PVRSRV_ERROR_INIT_FAILURE;
	}

	eError = psDevInfo->pfnGetGpuUtilStats(psDevInfo->psDeviceNode,
						psDVFSDevice->hGpuUtilUserDVFS,
						&sGpuUtilStats);

	if (eError != PVRSRV_OK)
	{
		return eError;
	}

	stat->busy_time = sGpuUtilStats.ui64GpuStatActiveHigh;
	stat->total_time = sGpuUtilStats.ui64GpuStatCumulative;

	return 0;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0))
static IMG_INT32 devfreq_cur_freq(struct device *dev, unsigned long *freq)
{
	RGX_DATA *psRGXData = (RGX_DATA*) gpsDeviceNode->psDevConfig->hDevData;

	*freq = psRGXData->psRGXTimingInfo->ui32CoreClockSpeed;

	return 0;
}
#endif

static struct devfreq_dev_profile img_devfreq_dev_profile = {
	.target			= devfreq_target,
	.get_dev_status		= devfreq_get_dev_status,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0))
	.get_cur_freq		= devfreq_cur_freq,
#endif
};

static PVRSRV_ERROR InitializeOPPTable(struct device *psDev)
{
	IMG_DVFS_DEVICE_CFG	*psDVFSDeviceCfg = &gpsDeviceNode->psDevConfig->sDVFS.sDVFSDeviceCfg;
	IMG_UINT32		i;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0))
	img_devfreq_dev_profile.freq_table = OSAllocMem(psDVFSDeviceCfg->ui32OPPTableSize * sizeof(img_devfreq_dev_profile.freq_table[0]));
	img_devfreq_dev_profile.max_state = psDVFSDeviceCfg->ui32OPPTableSize;
#endif
	for (i = 0; i < psDVFSDeviceCfg->ui32OPPTableSize; i++)
	{
		IMG_UINT32	ui32Error, ui32Freq, ui32Volt;

		ui32Freq = psDVFSDeviceCfg->pasOPPTable[i].ui32Freq;
		ui32Volt = psDVFSDeviceCfg->pasOPPTable[i].ui32Volt;

		ui32Error = OPP_ADD(psDev, ui32Freq, ui32Volt);
		if (ui32Error)
		{
			PVR_DPF((PVR_DBG_ERROR, "Failed to add OPP"));
			return PVRSRV_ERROR_INIT_FAILURE;
		}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0))
		img_devfreq_dev_profile.freq_table[i] = ui32Freq;
#endif
	}

	return PVRSRV_OK;
}

PVRSRV_ERROR InitDVFS(PVRSRV_DATA *psPVRSRVData, void *hDevice)
{
	RGX_TIMING_INFORMATION	*psRGXTimingInfo = NULL;
	IMG_DVFS_DEVICE		*psDVFSDevice = NULL;
	IMG_DVFS_DEVICE_CFG	*psDVFSDeviceCfg = NULL;
	IMG_UINT32		ui32InitialFreq, ui32InitialVolt, ui32Error, i;
	PVRSRV_ERROR		eError;
	struct OPP_STRUCT	*opp;
	struct DEVICE_STRUCT	*psLDMDev = (struct DEVICE_STRUCT *) hDevice;
	struct device		*psDev = &psLDMDev->dev;
	unsigned long		freq;

	for (i = 0; i < psPVRSRVData->ui32RegisteredDevices; i++)
	{
		PVRSRV_DEVICE_NODE* psDeviceNode = psPVRSRVData->apsRegisteredDevNodes[i];
		if (psDeviceNode && psDeviceNode->psDevConfig &&
				psDeviceNode->psDevConfig->eDeviceType == PVRSRV_DEVICE_TYPE_RGX)
		{
			RGX_DATA *psRGXData = (RGX_DATA*) psDeviceNode->psDevConfig->hDevData;

			gpsDeviceNode = psDeviceNode;
			psDVFSDevice = &gpsDeviceNode->psDevConfig->sDVFS.sDVFSDevice;
			psDVFSDeviceCfg = &gpsDeviceNode->psDevConfig->sDVFS.sDVFSDeviceCfg;
			psRGXTimingInfo = psRGXData->psRGXTimingInfo;
			freq = psDVFSDeviceCfg->ui32FreqMin;
		}
	}
	if (gpsDeviceNode == NULL)
	{
		PVR_DPF((PVR_DBG_ERROR, "Failed to find RGX device"));
		return PVRSRV_ERROR_INIT_FAILURE;
	}

	eError = RGXRegisterGpuUtilStats(&psDVFSDevice->hGpuUtilUserDVFS);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "Failed to register to the GPU utilisation stats"));
		return eError;
	}

	eError = InitializeOPPTable(psDev);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "Failed to init OPP table"));
		return eError;
	}

	rcu_read_lock();
	opp = OPP_FIND_FREQ_FLOOR(psDev, &freq);

	if (IS_ERR(opp))
	{
		rcu_read_unlock();
		PVR_DPF((PVR_DBG_ERROR, "Invalid initial frequency %u MHz", psDVFSDeviceCfg->ui32FreqMin));
		return PVRSRV_ERROR_INIT_FAILURE;
	}

	ui32InitialFreq = OPP_GET_FREQ(opp);
	ui32InitialVolt = OPP_GET_VOLTAGE(opp);
	rcu_read_unlock();

	img_devfreq_dev_profile.initial_freq = ui32InitialFreq;
	img_devfreq_dev_profile.polling_ms = psDVFSDeviceCfg->ui32PollMs;

	psDVFSDeviceCfg->pfnSetFrequency(ui32InitialFreq);

	psRGXTimingInfo->ui32CoreClockSpeed = ui32InitialFreq;

	psDVFSDeviceCfg->pfnSetVoltage(ui32InitialVolt);

	InitGovernor(psPVRSRVData);

	psDVFSDevice->psDevFreq = devfreq_add_device(psDev, &img_devfreq_dev_profile, DEVFREQ_GOVERNOR, NULL);
	if (IS_ERR(psDVFSDevice->psDevFreq))
	{
		PVR_DPF((PVR_DBG_ERROR, "Failed to add as devfreq device %p", psDVFSDevice->psDevFreq));
		return PVRSRV_ERROR_INIT_FAILURE;
	}

	eError = SuspendDVFS();
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVInit: Failed to suspend DVFS"));
		return eError;
	}

	psDVFSDevice->psDevFreq->min_freq = psDVFSDeviceCfg->ui32FreqMin;
	psDVFSDevice->psDevFreq->max_freq = psDVFSDeviceCfg->ui32FreqMax;

	ui32Error = devfreq_register_opp_notifier(psDev, psDVFSDevice->psDevFreq);
	if (ui32Error < 0) {
		PVR_DPF((PVR_DBG_ERROR, "Failed to register opp notifier"));
		return PVRSRV_ERROR_INIT_FAILURE;
	}

	PVR_TRACE(("PVR DVFS activated: %u-%uMhz, Period: %ums", psDVFSDeviceCfg->ui32FreqMin, \
			psDVFSDeviceCfg->ui32FreqMax, psDVFSDeviceCfg->ui32PollMs));

	return PVRSRV_OK;
}

void DeinitDVFS(PVRSRV_DATA *psPVRSRVData, void *hDevice)
{
	IMG_DVFS_DEVICE		*psDVFSDevice = &gpsDeviceNode->psDevConfig->sDVFS.sDVFSDevice;
	struct DEVICE_STRUCT	*psPlatDev = (struct DEVICE_STRUCT *) hDevice;
	struct device		*psDev = &psPlatDev->dev;
	IMG_INT32		i32Error;

	if (psDVFSDevice->psDevFreq)
	{
		i32Error = devfreq_unregister_opp_notifier(psDev, psDVFSDevice->psDevFreq);
		if (i32Error < 0)
		{
			PVR_DPF((PVR_DBG_ERROR, "Failed to unregister OPP notifier"));
		}

		devfreq_remove_device(psDVFSDevice->psDevFreq);

		psDVFSDevice->psDevFreq = NULL;
	}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0))
	if (img_devfreq_dev_profile.freq_table)
	{
		OSFreeMem(img_devfreq_dev_profile.freq_table);
	}
#endif

	RGXUnregisterGpuUtilStats(psDVFSDevice->hGpuUtilUserDVFS);

	gpsDeviceNode = NULL;

	DeinitGovernor();
}

PVRSRV_ERROR SuspendDVFS(void)
{
	IMG_DVFS_DEVICE		*psDVFSDevice = &gpsDeviceNode->psDevConfig->sDVFS.sDVFSDevice;

	psDVFSDevice->bEnabled = IMG_FALSE;

	return PVRSRV_OK;
}

PVRSRV_ERROR ResumeDVFS(void)
{
	IMG_DVFS_DEVICE		*psDVFSDevice = &gpsDeviceNode->psDevConfig->sDVFS.sDVFSDevice;

	psDVFSDevice->bEnabled = IMG_TRUE;

	return PVRSRV_OK;
}

