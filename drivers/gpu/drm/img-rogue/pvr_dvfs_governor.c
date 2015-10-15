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

#if defined(PVR_POWER_ACTOR)
#include <linux/power_actor.h>

static IMG_UINT32  GetStaticPower(IMG_UINT32 ui32Volt, IMG_INT32 i32T)
{
	IMG_DVFS_PA_CFG	*psDVFSPACfg = &gpsDeviceNode->psDevConfig->sDVFS.sDVFSPACfg;
	IMG_UINT32	ui32Remainder;
	IMG_INT32	i32T2 = i32T * i32T;
	IMG_INT32	i32T3 = i32T2 * i32T;
	IMG_UINT32	ui32V3 = (ui32Volt * ui32Volt * ui32Volt) / (1000 * 1000); /* mV */
	IMG_UINT64	ui64Power;

	ui64Power = (psDVFSPACfg->i32Ta * i32T3) + (psDVFSPACfg->i32Tb * i32T2) + (psDVFSPACfg->i32Tc * i32T) + psDVFSPACfg->i32Td;
	ui64Power = OSDivide64(ui64Power, 100000, &ui32Remainder); /* mW */
	ui64Power *= ui32V3; /* Normalise for voltage */
	ui64Power = OSDivide64(ui64Power, 1000, &ui32Remainder); /* mW */
	ui64Power += psDVFSPACfg->ui32Other;

	return ui64Power; /* mW */
}

static IMG_UINT32 GetDynamicPower(unsigned int uiEnergy, IMG_UINT32 ui32Volt)
{
	IMG_DVFS_PA	*psDVFSPA = &gpsDeviceNode->psDevConfig->sDVFS.sDVFSPA;
	IMG_UINT32	ui32V2 = (ui32Volt * ui32Volt) / 1000; /* mV */
	IMG_UINT64	ui64TimeNow = OSClockus();
	IMG_UINT64	ui64Energy, ui64Power;
	IMG_UINT32	ui32Remainder, ui32Power, ui32EnergyDiff;

	ui32EnergyDiff = (uiEnergy >= psDVFSPA->ui32Energy) ? uiEnergy - psDVFSPA->ui32Energy : uiEnergy + (UINT_MAX - psDVFSPA->ui32Energy);

	if (ui32EnergyDiff)
	{
		ui64Energy = OSDivide64(((IMG_UINT64) ui32EnergyDiff << 32), 100000, &ui32Remainder); /* mJ * 1000 */
		ui64Power = OSDivide64(ui64Energy, (ui64TimeNow - psDVFSPA->ui64StartTime), &ui32Remainder); /* mW */
	}
	else
	{
		ui64Power = 0;
	}

	ui64Power *= ui32V2; /* Normalise for voltage */
	ui32Power = ((IMG_UINT32) ui64Power) / (1000); /* mW */

	psDVFSPA->ui32Energy = uiEnergy;
	psDVFSPA->ui64StartTime = ui64TimeNow;

	return ui32Power; /* mW */
};

static IMG_UINT32 GetTotalPower(IMG_UINT32 *ui32StaticPower, IMG_UINT32 *ui32DynamicPower, struct devfreq_dev_status *hstat, IMG_BOOL bReset)
{
	IMG_DVFS_PA			*psDVFSPA = &gpsDeviceNode->psDevConfig->sDVFS.sDVFSPA;
	IMG_DVFS_DEVICE			*psDVFSDevice = &gpsDeviceNode->psDevConfig->sDVFS.sDVFSDevice;
	IMG_UINT32			ui32TotalPower, ui32Energy;
	struct devfreq_dev_status	stat;

	/* Callers of GetTotalPower can provide an already populated stat structure */
	if (hstat == NULL)
	{
		IMG_INT32	err;

		/* Filled by devfreq device */
		stat.private_data = &ui32Energy;

		err = psDVFSDevice->psDevFreq->profile->get_dev_status(psDVFSDevice->psDevFreq->dev.parent, &stat);
		if (err)
		{
			return 0;
		}

		hstat = &stat;
	}
	else
	{
		ui32Energy = *(IMG_UINT32*) hstat->private_data;
	}

	*ui32StaticPower = (GetStaticPower(psDVFSPA->sOPPCurrent.ui32Volt, psDVFSPA->i32Temp) * hstat->busy_time) / hstat->total_time;
	OSLockAcquire(psDVFSPA->hDVFSLock);
	*ui32DynamicPower = GetDynamicPower(ui32Energy, psDVFSPA->sOPPCurrent.ui32Volt);
	psDVFSPA->sPowerAvg.ui32Power += *ui32StaticPower + *ui32DynamicPower;
	psDVFSPA->sPowerAvg.ui32Samples++;
	ui32TotalPower = psDVFSPA->sPowerAvg.ui32Power / psDVFSPA->sPowerAvg.ui32Samples;

#if defined (PVR_POWER_ACTOR_DEBUG)
	PVR_DPF((PVR_DBG_ERROR,"GetTotalPower: %d, Static: %d, Dynamic %d, Temp %d", *ui32StaticPower + *ui32DynamicPower, \
							*ui32StaticPower, *ui32DynamicPower, psDVFSPA->i32Temp));
#endif

	if (bReset)
	{
		psDVFSPA->sPowerAvg.ui32Power = 0;
		psDVFSPA->sPowerAvg.ui32Samples = 0;
	}

	OSLockRelease(psDVFSPA->hDVFSLock);

	return ui32TotalPower;
}

static IMG_UINT32 GetCurrentPower(struct power_actor *actor, struct thermal_zone_device *zone)
{
	IMG_DVFS_PA			*psDVFSPA = &gpsDeviceNode->psDevConfig->sDVFS.sDVFSPA;
	IMG_UINT32			ui32StaticPower, ui32DynamicPower, ui32TotalPower;

	psDVFSPA->i32Temp = zone->temperature;

	ui32TotalPower = GetTotalPower(&ui32StaticPower, &ui32DynamicPower, NULL, IMG_TRUE);

#if defined (PVR_POWER_ACTOR_DEBUG)
	PVR_DPF((PVR_DBG_ERROR,"GetCurrentPower: %d", ui32TotalPower));
#endif

	return ui32TotalPower;
}

static IMG_UINT32 GetMaxPower (struct power_actor *actor, struct thermal_zone_device *zone)
{
	IMG_DVFS_DEVICE_CFG		*psDVFSDeviceCfg = &gpsDeviceNode->psDevConfig->sDVFS.sDVFSDeviceCfg;
	IMG_DVFS_PA			*psDVFSPA = &gpsDeviceNode->psDevConfig->sDVFS.sDVFSPA;
	IMG_UINT32			ui32OPPLast = psDVFSDeviceCfg->ui32OPPTableSize - 1;
	IMG_UINT32			ui32DynamicPower, ui32StaticPower, ui32Ratio, i;

	psDVFSPA->i32Temp = zone->temperature;

	GetTotalPower(&ui32StaticPower, &ui32DynamicPower, NULL, IMG_FALSE);

	/* Avoid divide by zero */
	if (ui32DynamicPower)
	{
		/* Look up OPP index we are currently set to */
		for (i = 0; i < ui32OPPLast; i++)
		{
			if (psDVFSDeviceCfg->pasOPPTable[i].ui32Freq >= psDVFSPA->sOPPCurrent.ui32Freq)
			{
				break;
			}
		}

		/* Calculate ratio based on estimated current power */
		ui32Ratio = psDVFSPA->aui32ConversionTable[i] / ui32DynamicPower;

		/* Calculate dynamic power at maximum OPP */
		ui32DynamicPower = psDVFSPA->aui32ConversionTable[ui32OPPLast] / ui32Ratio;
	}

	ui32StaticPower = GetStaticPower(psDVFSDeviceCfg->pasOPPTable[ui32OPPLast].ui32Volt, psDVFSPA->i32Temp);

#if defined (PVR_POWER_ACTOR_DEBUG)
	PVR_DPF((PVR_DBG_ERROR,"GetMaxPower: %d", ui32StaticPower + ui32DynamicPower));
#endif

	return ui32StaticPower + ui32DynamicPower;
}

static IMG_INT32 SetPower(struct power_actor *actor, struct thermal_zone_device *zone, u32 power)
{
	IMG_DVFS_PA	*psDVFSPA = &gpsDeviceNode->psDevConfig->sDVFS.sDVFSPA;

	psDVFSPA->i32Temp = zone->temperature;
	psDVFSPA->ui32AllocatedPower = power;

#if defined (PVR_POWER_ACTOR_DEBUG)
	PVR_DPF((PVR_DBG_ERROR,"SetPower: %d", power));
#endif

	return 0;
}

static struct power_actor_ops img_power_actor_ops = {
	.get_req_power = GetCurrentPower,
	.get_max_power = GetMaxPower,
	.set_power = SetPower,
};

static IMG_UINT32 DynamicPowerToFreq (IMG_UINT32 ui32PowerRemain, IMG_UINT32 ui32DynamicPower, IMG_UINT32 ui32Freq)
{
	IMG_DVFS_DEVICE_CFG	*psDVFSDeviceCfg = &gpsDeviceNode->psDevConfig->sDVFS.sDVFSDeviceCfg;
	IMG_DVFS_PA		*psDVFSPA = &gpsDeviceNode->psDevConfig->sDVFS.sDVFSPA;
	IMG_UINT32		ui32Ratio, i, ui32OPPLast = psDVFSDeviceCfg->ui32OPPTableSize - 1;

	/* Look up OPP index we are currently set to */
	for (i = 0; i < ui32OPPLast; i++)
	{
		if (ui32Freq == psDVFSDeviceCfg->pasOPPTable[i].ui32Freq)
		{
			break;
		}
	}

	/* Prevent divide by zero */
	if (ui32DynamicPower)
	{
		/* Calculate ratio based on estimated current power */
		ui32Ratio = psDVFSPA->aui32ConversionTable[i] / ui32DynamicPower;

		/* Look up ideal OPP for PowerRemain */
		for (i = 0; i < ui32OPPLast; i++)
		{
			IMG_UINT32 uiEstimatedPower = psDVFSPA->aui32ConversionTable[i] / ui32Ratio;
			if (uiEstimatedPower > ui32PowerRemain)
			{
				break;
			}
		}
	}

	/* Return frequency for ideal OPP */
	return psDVFSDeviceCfg->pasOPPTable[i].ui32Freq;
}

static PVRSRV_ERROR IPAEnvelopeCheck(struct devfreq *df, struct devfreq_dev_status *stat, unsigned long *freq)
{
	IMG_DVFS_PA		*psDVFSPA = &gpsDeviceNode->psDevConfig->sDVFS.sDVFSPA;
	IMG_UINT32		ui32StaticPower, ui32DynamicPower;
	IMG_UINT32		ui32AllocatedPower = psDVFSPA->ui32AllocatedPower;

	GetTotalPower(&ui32StaticPower, &ui32DynamicPower, stat, IMG_FALSE);

	if ((ui32StaticPower + ui32DynamicPower) > ui32AllocatedPower)
	{
		IMG_UINT32 ui32DynamicPowerRemain = (ui32StaticPower > ui32AllocatedPower) ? 0 : (ui32AllocatedPower - ui32StaticPower);
		*freq = DynamicPowerToFreq(ui32DynamicPowerRemain, ui32DynamicPower, psDVFSPA->sOPPCurrent.ui32Freq);
	}

	return PVRSRV_OK;
}

static PVRSRV_ERROR UpdateCurrentOPP(struct devfreq *df, unsigned long *freq)
{
	IMG_DVFS_PA		*psDVFSPA = &gpsDeviceNode->psDevConfig->sDVFS.sDVFSPA;
	struct OPP_STRUCT	*opp;

	rcu_read_lock();
	opp = OPP_FIND_FREQ_CEIL(df->dev.parent, freq);
	if (IS_ERR(opp))
	{
		PVR_DPF((PVR_DBG_ERROR, "Failed to find OPP"));
		rcu_read_unlock();
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	psDVFSPA->sOPPCurrent.ui32Freq = *freq;
	psDVFSPA->sOPPCurrent.ui32Volt = OPP_GET_VOLTAGE(opp);
	rcu_read_unlock();

	return PVRSRV_OK;
}

PVRSRV_ERROR InitPowerActor(void)
{
	IMG_DVFS_PA		*psDVFSPA = &gpsDeviceNode->psDevConfig->sDVFS.sDVFSPA;
	IMG_DVFS_DEVICE_CFG	*psDVFSDeviceCfg = &gpsDeviceNode->psDevConfig->sDVFS.sDVFSDeviceCfg;
	IMG_DVFS_PA_CFG		*psDVFSPACfg = &gpsDeviceNode->psDevConfig->sDVFS.sDVFSPACfg;
	PVRSRV_ERROR		eError;
	IMG_UINT32		i;

	psDVFSPA->aui32ConversionTable = OSAllocMem(psDVFSDeviceCfg->ui32OPPTableSize * sizeof(psDVFSPA->aui32ConversionTable[0]));

	for (i = 0; i < psDVFSDeviceCfg->ui32OPPTableSize; i++)
	{
		psDVFSPA->aui32ConversionTable[i] = psDVFSDeviceCfg->pasOPPTable[i].ui32Volt * \
			psDVFSDeviceCfg->pasOPPTable[i].ui32Volt * psDVFSDeviceCfg->pasOPPTable[i].ui32Freq;
	}

	psDVFSPA->ui32AllocatedPower = UINT_MAX;
	psDVFSPA->i32Temp = DVFS_POWACT_TEMP_DEFAULT;
	psDVFSPA->ui64StartTime = OSClockus();
	psDVFSPA->sPowerAvg.ui32Power = 0;
	psDVFSPA->sPowerAvg.ui32Samples = 0;

	eError = OSLockCreate(&psDVFSPA->hDVFSLock, LOCK_TYPE_PASSIVE);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "Failed to create lock"));
		return PVRSRV_ERROR_INIT_FAILURE;
	}

	psDVFSPA->psPowerActor = power_actor_register(psDVFSPACfg->ui32Weight, &img_power_actor_ops, NULL);
	if (IS_ERR_OR_NULL(psDVFSPA->psPowerActor))
	{
		return PVRSRV_ERROR_INIT_FAILURE;
	}

#if defined (PVR_POWER_ACTOR_DEBUG)
	PVR_DPF((PVR_DBG_ERROR,"InitPowerActor: Init power actor with weight %d", psDVFSPACfg->ui32Weight));
#endif

	return PVRSRV_OK;
}

PVRSRV_ERROR DeinitPowerActor(void)
{
	IMG_DVFS_PA	*psDVFSPA = &gpsDeviceNode->psDevConfig->sDVFS.sDVFSPA;

#if defined (PVR_POWER_ACTOR_DEBUG)
	PVR_DPF((PVR_DBG_ERROR,"DeinitPowerActor: Deinit power actor"));
#endif

	if (psDVFSPA->psPowerActor)
	{
		power_actor_unregister(psDVFSPA->psPowerActor);
	}

	if (psDVFSPA->aui32ConversionTable)
	{
		OSFreeMem(psDVFSPA->aui32ConversionTable);
	}

	OSLockDestroy(psDVFSPA->hDVFSLock);

	return PVRSRV_OK;
}

#endif /* defined(PVR_POWER_ACTOR) */

static IMG_INT32 GetTargetFreq(struct devfreq *df, unsigned long *freq)
{
	IMG_DVFS_GOVERNOR_CFG		*psDVFSGovernorCfg = &gpsDeviceNode->psDevConfig->sDVFS.sDVFSGovernorCfg;
	IMG_DVFS_DEVICE			*psDVFSDevice = &gpsDeviceNode->psDevConfig->sDVFS.sDVFSDevice;
	IMG_INT32			err;
	IMG_UINT64			ui64a, ui64b, ui64max = df->max_freq;
	IMG_UINT32			ui32Remainder, dfso_upthreshold = psDVFSGovernorCfg->ui32UpThreshold;
	IMG_UINT32			dfso_downdifferential = psDVFSGovernorCfg->ui32DownDifferential;
	struct devfreq_dev_status	stat;
#if defined(PVR_POWER_ACTOR)
	IMG_UINT32			ui32Energy;
	PVRSRV_ERROR			eError;
	/* Filled by devfreq device */
	stat.private_data = &ui32Energy;
#endif

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
#if defined(PVR_POWER_ACTOR)
	eError = IPAEnvelopeCheck(df, &stat, freq);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"GetTargetFreq: Failed to perform power envelope check"));
		return 0;
	}
#endif

	/* Envelope check within specified min/max */
	if (*freq < df->min_freq)
	{
		*freq = df->min_freq;
	}
	if (*freq > ui64max)
	{
		*freq = ui64max;
	}

#if defined(PVR_POWER_ACTOR)
	eError = UpdateCurrentOPP(df, freq);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"GetTargetFreq: Failed to update current OPP"));
		return 0;
	}
#endif

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
