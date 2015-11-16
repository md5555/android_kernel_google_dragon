/*************************************************************************/ /*!
@File
@Title          PowerVR devfreq governor implementation
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

#include <linux/errno.h>
#include <linux/module.h>
#include <linux/devfreq.h>
#include <linux/math64.h>
#include <linux/clk.h>
#include <linux/thermal.h>
#include <linux/version.h>

#include "power.h"
#include "pvr_dvfs_governor.h"
#include "pvrsrv_device.h"

/* This is ignorant overflow detection for whatever numbers the device gives the governor.
 * It's used to ensure the math to determine the ideal frequency doesn't cause wrapping.
 * Currently we provide a percentage of busy & total time as that's what the GPU utilisation
 * stats are based on. At present we can assert they will never be above DVFS_MAX_TIME.
 * However this may change to raw values (e.g. microseconds) in the future. */
#define DVFS_MAX_TIME (1 << 24)
#define DVFS_OVERFLOW_SHIFT (7)

#define DVFS_POWACT_TEMP_DEFAULT 30

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,13,0))
#include <linux/pm_opp.h>
#define OPP_GET_FREQ dev_pm_opp_get_freq
#define OPP_GET_VOLTAGE dev_pm_opp_get_voltage
#define OPP_ADD dev_pm_opp_add
#define OPP_FIND_FREQ_CEIL dev_pm_opp_find_freq_ceil
#define OPP_STRUCT dev_pm_opp
#else
#include <linux/opp.h>
#define OPP_GET_FREQ opp_get_freq
#define OPP_GET_VOLTAGE opp_get_voltage
#define OPP_ADD opp_add
#define OPP_FIND_FREQ_CEIL opp_find_freq_ceil
#define OPP_STRUCT opp
#endif

static PVRSRV_DEVICE_NODE* gpsDeviceNode = NULL;

static IMG_INT32 GetTargetFreq(struct devfreq *df, unsigned long *freq)
{
	IMG_DVFS_GOVERNOR_CFG		*psDVFSGovernorCfg = &gpsDeviceNode->psDevConfig->sDVFS.sDVFSGovernorCfg;
	IMG_DVFS_DEVICE			*psDVFSDevice = &gpsDeviceNode->psDevConfig->sDVFS.sDVFSDevice;
	IMG_INT32			err;
	IMG_UINT64			ui64a, ui64b, ui64max = df->max_freq;
	IMG_UINT32			ui32Remainder, dfso_upthreshold = psDVFSGovernorCfg->ui32UpThreshold;
	IMG_UINT32			dfso_downdifferential = psDVFSGovernorCfg->ui32DownDifferential;
	struct devfreq_dev_status	stat;

	err = psDVFSDevice->psDevFreq->profile->get_dev_status(psDVFSDevice->psDevFreq->dev.parent, &stat);
	if (err)
	{
		return 0;
	}

	/* Assume MAX if it is going to be divided by zero */
	if (stat.total_time == 0) {
		*freq = ui64max;
		goto EnvelopeCheck;
	}

	/* Prevent overflow */
	if (stat.busy_time >= (DVFS_MAX_TIME) || stat.total_time >= (DVFS_MAX_TIME)) {
		stat.busy_time >>= DVFS_OVERFLOW_SHIFT;
		stat.total_time >>= DVFS_OVERFLOW_SHIFT;
	}

	/* Set MAX if it's busy enough */
	if (stat.busy_time * 100 > stat.total_time * dfso_upthreshold)
	{
		*freq = ui64max;
		goto EnvelopeCheck;
	}

	/* Keep the current frequency */
	if (stat.busy_time * 100 > stat.total_time * (dfso_upthreshold - dfso_downdifferential))
	{
		*freq = stat.current_frequency;
		goto EnvelopeCheck;
	}

	/* Set the desired frequency based on the load */
	ui64a = stat.busy_time;
	ui64a *= stat.current_frequency;
	ui64b = OSDivide64(ui64a, stat.total_time, &ui32Remainder);
	ui64b *= 100;
	ui64b = OSDivide64(ui64b, (dfso_upthreshold - dfso_downdifferential / 2), &ui32Remainder);
	*freq = (unsigned long) ui64b;

EnvelopeCheck:

	/* Envelope check within specified min/max */
	if (*freq < df->min_freq)
	{
		*freq = df->min_freq;
	}
	if (*freq > ui64max)
	{
		*freq = ui64max;
	}

	return 0;
}
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0))
static IMG_INT32 EventHandler(struct devfreq *devfreq, unsigned int event, void *data)
{
	switch (event)
	{
	case DEVFREQ_GOV_START:
		devfreq_monitor_start(devfreq);
		break;

	case DEVFREQ_GOV_STOP:
		devfreq_monitor_stop(devfreq);
		break;

	case DEVFREQ_GOV_INTERVAL:
		devfreq_interval_update(devfreq, (unsigned int *)data);
		break;

	case DEVFREQ_GOV_SUSPEND:
		devfreq_monitor_suspend(devfreq);
		break;

	case DEVFREQ_GOV_RESUME:
		devfreq_monitor_resume(devfreq);
		break;

	default:
		break;
	}

	return 0;
}
#endif

struct devfreq_governor devfreq_img_governor = {
	.name = "img_governor",
	.get_target_freq = GetTargetFreq,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0))
	.event_handler = EventHandler,
#endif
};

PVRSRV_ERROR InitGovernor(PVRSRV_DATA *psPVRSRVData)
{
	IMG_DVFS_GOVERNOR	*psDVFSGovernor = NULL;
	IMG_DVFS_GOVERNOR_CFG	*psDVFSGovernorCfg = NULL;
	IMG_UINT32		i;

	for (i = 0; i < psPVRSRVData->ui32RegisteredDevices; i++)
	{
		PVRSRV_DEVICE_NODE* psDeviceNode = psPVRSRVData->apsRegisteredDevNodes[i];
		if (psDeviceNode && psDeviceNode->psDevConfig &&
				psDeviceNode->psDevConfig->eDeviceType == PVRSRV_DEVICE_TYPE_RGX)
		{
			gpsDeviceNode = psDeviceNode;
			psDVFSGovernor = &psDeviceNode->psDevConfig->sDVFS.sDVFSGovernor;
			psDVFSGovernorCfg = &psDeviceNode->psDevConfig->sDVFS.sDVFSGovernorCfg;
		}
	}
	if (gpsDeviceNode == NULL)
	{
		PVR_DPF((PVR_DBG_ERROR, "Failed to find RGX device"));
		return PVRSRV_ERROR_INIT_FAILURE;
	}

	/* UpThreshold must be greater than DownDifferential */
	PVR_ASSERT(psDVFSGovernorCfg->ui32UpThreshold > psDVFSGovernorCfg->ui32DownDifferential);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0))
	devfreq_add_governor(&devfreq_img_governor);
#endif

	psDVFSGovernor->bEnabled = IMG_TRUE;

	return PVRSRV_OK;
}

PVRSRV_ERROR DeinitGovernor(void)
{
	IMG_DVFS_GOVERNOR	*psDVFSGovernor = &gpsDeviceNode->psDevConfig->sDVFS.sDVFSGovernor;
	IMG_INT32		i32Error;

	if (psDVFSGovernor->bEnabled == IMG_TRUE)
	{
		i32Error = devfreq_remove_governor(&devfreq_img_governor);
		if (i32Error)
		{
			PVR_DPF((PVR_DBG_ERROR, "Failed to remove IMG governor"));
			return PVRSRV_ERROR_INVALID_PARAMS;
		}
	}

	psDVFSGovernor->bEnabled = IMG_FALSE;
	gpsDeviceNode = NULL;

	return PVRSRV_OK;
}
