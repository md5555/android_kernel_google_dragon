/*************************************************************************/ /*!
@File
@Title          System Configuration
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    System Configuration functions
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

#include "pvrsrv_device.h"
#include "syscommon.h"
#include "mt8173_sysconfig.h"
#include "physheap.h"
#if defined(SUPPORT_ION)
#include "ion_support.h"
#endif
#include "mt8173_mfgsys.h"
#if defined (LINUX)
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#endif

#define RGX_CR_ISP_GRIDOFFSET                             (0x0FA0U)

static RGX_TIMING_INFORMATION	gsRGXTimingInfo;
static RGX_DATA					gsRGXData;
static PVRSRV_DEVICE_CONFIG 	gsDevices[1];
static PVRSRV_SYSTEM_CONFIG 	gsSysConfig;

static PHYS_HEAP_FUNCTIONS	gsPhysHeapFuncs;
static PHYS_HEAP_CONFIG		gsPhysHeapConfig;

/*
	CPU to Device physcial address translation
*/
static
void UMAPhysHeapCpuPAddrToDevPAddr(IMG_HANDLE hPrivData,
								   IMG_UINT32 ui32NumOfAddr,
								   IMG_DEV_PHYADDR *psDevPAddr,
								   IMG_CPU_PHYADDR *psCpuPAddr)
{
	PVR_UNREFERENCED_PARAMETER(hPrivData);
	
	/* Optimise common case */
	psDevPAddr[0].uiAddr = psCpuPAddr[0].uiAddr;
	if (ui32NumOfAddr > 1)
	{
		IMG_UINT32 ui32Idx;
		for (ui32Idx = 1; ui32Idx < ui32NumOfAddr; ++ui32Idx)
		{
			psDevPAddr[ui32Idx].uiAddr = psCpuPAddr[ui32Idx].uiAddr;
		}
	}
}

/*
	Device to CPU physcial address translation
*/
static
void UMAPhysHeapDevPAddrToCpuPAddr(IMG_HANDLE hPrivData,
								   IMG_UINT32 ui32NumOfAddr,
								   IMG_CPU_PHYADDR *psCpuPAddr,
								   IMG_DEV_PHYADDR *psDevPAddr)
{
	PVR_UNREFERENCED_PARAMETER(hPrivData);

	/* Optimise common case */
	psCpuPAddr[0].uiAddr = psDevPAddr[0].uiAddr;
	if (ui32NumOfAddr > 1)
	{
		IMG_UINT32 ui32Idx;
		for (ui32Idx = 1; ui32Idx < ui32NumOfAddr; ++ui32Idx)
		{
			psCpuPAddr[ui32Idx].uiAddr = psDevPAddr[ui32Idx].uiAddr;
		}
	}
}

#if defined(PVR_DVFS)
void SetFrequency(IMG_UINT32 ui64Freq)
{
	if (gsDevices[0].hSysData)
		MTKSysSetFreq((struct mtk_mfg_base *)gsDevices[0].hSysData,
			      ui64Freq);
}

void SetVoltage(IMG_UINT32 ui64Volt)
{
	if (gsDevices[0].hSysData)
		MTKSysSetVolt((struct mtk_mfg_base *)gsDevices[0].hSysData,
			      ui64Volt);
}

void SetupDVFSInfo(void *hDevice, PVRSRV_DVFS *hDVFS)
{
	struct platform_device *pDevice = (struct platform_device *)hDevice;
	struct mtk_mfg_base *mfg_base;
	IMG_OPP *opp_table;
	IMG_DVFS_DEVICE_CFG *img_dvfs_cfg;
	int i;

	mfg_base = (struct mtk_mfg_base *)pDevice->dev.platform_data;

	if (!mfg_base || !mfg_base->fv_table)
		return;

	opp_table = devm_kcalloc(&pDevice->dev,
				 mfg_base->fv_table_length,
				 sizeof(*opp_table),
				 GFP_KERNEL);

	if (!opp_table)
		return;

	img_dvfs_cfg = &hDVFS->sDVFSDeviceCfg;

	for (i = 0; i < mfg_base->fv_table_length; ++i) {
		opp_table[i].ui32Freq = mfg_base->fv_table[i].freq;
		opp_table[i].ui32Volt = mfg_base->fv_table[i].volt;
	}

	img_dvfs_cfg->bIdleReq = IMG_FALSE;
	img_dvfs_cfg->pasOPPTable = (IMG_OPP_TABLE)opp_table;
	img_dvfs_cfg->ui32OPPTableSize = mfg_base->fv_table_length;
	img_dvfs_cfg->ui32FreqMin = opp_table[0].ui32Freq;
	img_dvfs_cfg->ui32FreqMax = opp_table[mfg_base->fv_table_length - 1].ui32Freq;
	img_dvfs_cfg->pfnSetFrequency = SetFrequency;
	img_dvfs_cfg->pfnSetVoltage = SetVoltage;
	img_dvfs_cfg->ui32PollMs = MTK_DVFS_SWITCH_INTERVAL;

	hDVFS->sDVFSGovernorCfg.ui32UpThreshold = 90;
	hDVFS->sDVFSGovernorCfg.ui32DownDifferential = 10;
	gsDevices[0].hSysData = mfg_base;
}
#endif

#if defined(MTK_POWER_ACTOR)
IMG_UINT32 GetStaticPower(IMG_UINT32 voltage, IMG_INT32 temperature)
{
	#define	NUM_RANGE 5
	int t_range[NUM_RANGE] = {25, 45, 65, 85, 105};
	IMG_UINT32 lookup_table_0p_9v[NUM_RANGE] = {14540, 35490, 60420, 120690, 230000};
	IMG_UINT32 lookup_table_1p_0v[NUM_RANGE] = {21570, 41910, 82380, 159140, 298620};
	IMG_UINT32 lookup_table_1p_1v[NUM_RANGE] = {32320, 72950,111320, 209290, 382700};
	IMG_UINT32 *lookup = &lookup_table_1p_0v[0];
	IMG_UINT32 power;
	int low_idx = 0, high_idx = NUM_RANGE - 1;
	int i;

	if (voltage < 1000000)
		lookup = &lookup_table_0p_9v[0];
	else if (voltage > 1100000)
		lookup = &lookup_table_1p_1v[0];

	for (i = 0; i < NUM_RANGE; i++)	{
		if (temperature <= t_range[NUM_RANGE - 1 - i])
			high_idx = NUM_RANGE - 1 - i;

		if (temperature >= t_range[i])
			low_idx = i;
	}

	if (low_idx == high_idx) {
		power = lookup[low_idx];
	} else {
		IMG_UINT32 t_interval = t_range[high_idx] - t_range[low_idx];
		IMG_UINT32 p_interval = lookup[high_idx] - lookup[low_idx];

		power = p_interval * (temperature - t_range[low_idx]) / t_interval;
		power += lookup[low_idx];
	}

	PVR_TRACE(("voltage = %d, T = %d, power = %d [%d, %d]\n",
		  voltage, temperature, power, low_idx, high_idx));
	return power;
}
#endif

/*
	SysCreateConfigData
*/
PVRSRV_ERROR SysCreateConfigData(PVRSRV_SYSTEM_CONFIG **ppsSysConfig, void *hDevice)
{
	struct platform_device *pDevice = (struct platform_device *)hDevice;
	struct resource *irq_res;
	struct resource *reg_res;

	if (!pDevice) {
		PVR_DPF((PVR_DBG_ERROR, "pDevice = NULL!"));
		return PVRSRV_ERROR_INIT_FAILURE;
	}

	/*
	 * Setup information about physaical memory heap(s) we have
	 */
	gsPhysHeapFuncs.pfnCpuPAddrToDevPAddr = UMAPhysHeapCpuPAddrToDevPAddr;
	gsPhysHeapFuncs.pfnDevPAddrToCpuPAddr = UMAPhysHeapDevPAddrToCpuPAddr;

	gsPhysHeapConfig.ui32PhysHeapID = 0;
	gsPhysHeapConfig.pszPDumpMemspaceName = "SYSMEM";
	gsPhysHeapConfig.eType = PHYS_HEAP_TYPE_UMA;
	gsPhysHeapConfig.psMemFuncs = &gsPhysHeapFuncs;
	gsPhysHeapConfig.hPrivData = (IMG_HANDLE)&gsSysConfig;
	#if 1
	gsPhysHeapConfig.sStartAddr.uiAddr = 0;
	gsPhysHeapConfig.uiSize = 0;
	#endif

	gsSysConfig.pasPhysHeaps = &gsPhysHeapConfig;
	gsSysConfig.ui32PhysHeapCount = sizeof(gsPhysHeapConfig) / sizeof(PHYS_HEAP_CONFIG);
/*
add for new DDK 1.1.2550513
*/
	gsSysConfig.pui32BIFTilingHeapConfigs = gauiBIFTilingHeapXStrides;
	gsSysConfig.ui32BIFTilingHeapCount = IMG_ARR_NUM_ELEMS(gauiBIFTilingHeapXStrides);

	/*
	 * Setup RGX specific timing data
	 */
	gsRGXTimingInfo.ui32CoreClockSpeed        = RGX_HW_CORE_CLOCK_SPEED;
	
	#if MTK_PM_SUPPORT
	gsRGXTimingInfo.bEnableActivePM           = IMG_TRUE;
	gsRGXTimingInfo.ui32ActivePMLatencyms       = SYS_RGX_ACTIVE_POWER_LATENCY_MS;
	#else
	gsRGXTimingInfo.bEnableActivePM           = IMG_FALSE;
	#endif
	
	gsRGXTimingInfo.bEnableRDPowIsland        = IMG_FALSE;

	/* for HWAPM enable */
	#if MTK_ENABLE_HWAPM
	gsRGXTimingInfo.bEnableRDPowIsland    = IMG_TRUE;
	#else
	gsRGXTimingInfo.bEnableRDPowIsland    = IMG_FALSE;
	#endif
	/*
	 *Setup RGX specific data
	 */
	gsRGXData.psRGXTimingInfo = &gsRGXTimingInfo;

	/*
	 * Setup RGX device
	 */
	gsDevices[0].eDeviceType            = PVRSRV_DEVICE_TYPE_RGX;
	gsDevices[0].pszName                = "RGX";

	irq_res = platform_get_resource(pDevice, IORESOURCE_IRQ, 0);
	if (irq_res) {
		gsDevices[0].ui32IRQ = irq_res->start;
		gsDevices[0].bIRQIsShared    = IMG_FALSE;
		gsDevices[0].eIRQActiveLevel = PVRSRV_DEVICE_IRQ_ACTIVE_LOW;
	} else {
		PVR_DPF((PVR_DBG_ERROR, "irq_res = NULL!"));
		return PVRSRV_ERROR_INIT_FAILURE;
	}

	reg_res = platform_get_resource(pDevice, IORESOURCE_MEM, 0);
	if (reg_res) {
		gsDevices[0].sRegsCpuPBase.uiAddr = reg_res->start;
		gsDevices[0].ui32RegsSize	  = resource_size(reg_res);
	} else {
		PVR_DPF((PVR_DBG_ERROR, "reg_res = NULL!"));
		return PVRSRV_ERROR_INIT_FAILURE;
	}

#if defined(PVR_DVFS)
	SetupDVFSInfo(hDevice , (PVRSRV_DVFS *)(&gsDevices[0].sDVFS));
#endif

#if defined(PVR_POWER_ACTOR)
	/* I=0.000497135 J=-0.048369531 K=2.65455599 L=-22.33068359 *10000 */
	gsDevices[0].sDVFS.sDVFSPACfg.i32Ta = 49;
	gsDevices[0].sDVFS.sDVFSPACfg.i32Tb = -4836;
	gsDevices[0].sDVFS.sDVFSPACfg.i32Tc = 265455;
	gsDevices[0].sDVFS.sDVFSPACfg.i32Td = -2233068;
	gsDevices[0].sDVFS.sDVFSPACfg.ui32Other = 0;
	gsDevices[0].sDVFS.sDVFSPACfg.ui32Weight = 0;
#endif

	/* Device's physical heap IDs */
	gsDevices[0].aui32PhysHeapID[PVRSRV_DEVICE_PHYS_HEAP_GPU_LOCAL] = 0;
	gsDevices[0].aui32PhysHeapID[PVRSRV_DEVICE_PHYS_HEAP_CPU_LOCAL] = 0;
	gsDevices[0].aui32PhysHeapID[PVRSRV_DEVICE_PHYS_HEAP_FW_LOCAL] = 0;

	/*  power management on  HW system */
	#if MTK_PM_SUPPORT
	gsDevices[0].pfnPrePowerState       = MTKSysDevPrePowerState;
	gsDevices[0].pfnPostPowerState      = MTKSysDevPostPowerState;
	#else
	gsDevices[0].pfnPrePowerState       = NULL;
	gsDevices[0].pfnPostPowerState      = NULL;
	#endif

	/*  clock frequency  */
	gsDevices[0].pfnClockFreqGet        = NULL;

	/*  interrupt handled  */
	gsDevices[0].pfnInterruptHandled    = NULL;

	gsDevices[0].hDevData               = &gsRGXData;

	#if 0
	gsDevices[0].bBPSet = IMG_FALSE;
	gsDevices[0].eBPDM = RGXFWIF_DM_TA; //need to check
	gsDevices[0].hSysData = NULL;
	gsDevices[0].ui32PhysHeapID = 0;
	gsDevices[0].uiFlags = 0; // currently unused
	#endif

	

	/*
	 * Setup system config
	 */
	gsSysConfig.pszSystemName = RGX_HW_SYSTEM_NAME;
	gsSysConfig.uiDeviceCount = sizeof(gsDevices)/sizeof(gsDevices[0]);
	gsSysConfig.pasDevices = &gsDevices[0];

	/*  power management on  HW system */
	#if MTK_PM_SUPPORT
	gsSysConfig.pfnSysPrePowerState = MTKSystemPrePowerState;
	gsSysConfig.pfnSysPostPowerState = MTKSystemPostPowerState;
	#else
	gsSysConfig.pfnSysPrePowerState = NULL;
	gsSysConfig.pfnSysPostPowerState =NULL;
	#endif

	/*  cache snooping */
	//gsSysConfig.bHasCacheSnooping = IMG_FALSE; // new DDK has new variable
    gsSysConfig.eCacheSnoopingMode = 0;

	#if 1 //chenzhu add 
	gsSysConfig.uiSysFlags = 0;
	#endif

	/* Setup other system specific stuff */
#if defined(SUPPORT_ION)
	IonInit(NULL);
#endif

	*ppsSysConfig = &gsSysConfig;

#if 0
	PVR_TRACE(("SysCreateConfigData: start to OSMapPhysToLin "));
	
	void *pvRegsBaseKM = OSMapPhysToLin(gsDevices[0].sRegsCpuPBase, gsDevices[0].ui32RegsSize , 0);
	IMG_UINT32 ui32Value;
		
    PVR_TRACE(("PVRCore_Init:pvRegsBaseKM = %p ",pvRegsBaseKM));
		

	ui32Value = OSReadHWReg32(pvRegsBaseKM, 0x20);
	PVR_TRACE(("PVRCore_Init:ui32Value = 0x%X ",ui32Value));
	ui32Value = OSReadHWReg32(pvRegsBaseKM, 0x28);
	PVR_TRACE(("PVRCore_Init:ui32Value = 0x%X ",ui32Value));

	OSWriteHWReg32(pvRegsBaseKM, RGX_CR_ISP_GRIDOFFSET, 0x55555555);
	ui32Value = OSReadHWReg32(pvRegsBaseKM, RGX_CR_ISP_GRIDOFFSET);
	PVR_TRACE(("PVRCore_Init:ui32Value = 0x%X ",ui32Value));

	OSWriteHWReg32(pvRegsBaseKM, RGX_CR_ISP_GRIDOFFSET, 0xAAAAAAAA);
	ui32Value = OSReadHWReg32(pvRegsBaseKM, RGX_CR_ISP_GRIDOFFSET);
	PVR_TRACE(("PVRCore_Init:ui32Value = 0x%X ",ui32Value));
#endif		

	MTKSysSetInitialPowerState();

	return PVRSRV_OK;
}


/*
	SysDestroyConfigData
*/
void SysDestroyConfigData(PVRSRV_SYSTEM_CONFIG *psSysConfig)
{
	PVR_UNREFERENCED_PARAMETER(psSysConfig);

#if defined(SUPPORT_ION)
	IonDeinit();
#endif

	MTKSysRestoreInitialPowerState();
}

PVRSRV_ERROR SysDebugInfo(PVRSRV_SYSTEM_CONFIG *psSysConfig,
						  DUMPDEBUG_PRINTF_FUNC *pfnDumpDebugPrintf)
{
	PVR_UNREFERENCED_PARAMETER(psSysConfig);
	PVR_UNREFERENCED_PARAMETER(pfnDumpDebugPrintf);

	return PVRSRV_OK;
}

SYS_PHYS_ADDRESS_MASK SysDevicePhysAddressMask(void)
{
	return SYS_PHYS_ADDRESS_64_BIT;
}

/******************************************************************************
 End of file (sysconfig.c)
******************************************************************************/
