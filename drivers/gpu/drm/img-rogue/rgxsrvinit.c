/*************************************************************************/ /*!
@File
@Title          Services initialisation routines
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    Device specific functions
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

#include "img_defs.h"
#include "srvinit.h"
#include "pvr_debug.h"
#include "osfunc.h"

#include "devicemem.h"
#include "devicemem_pdump.h"

#include "client_rgxinit_bridge.h"
#include "rgx_fwif_km.h"
#include "rgx_fwif_client.h"
#include "rgx_fwif_alignchecks.h"
#include "rgx_fwif_sig_km.h"
#if !defined(SUPPORT_KERNEL_SRVINIT) && defined(PDUMP)
#include "rgx_fwif_sig.h"
#endif

#include "rgx_compat_bvnc.h"

#include "srvinit_osfunc.h"
#if defined(SUPPORT_GPUVIRT_VALIDATION)
#include "virt_validation_defs.h"
#endif

#include "srvinit_param.h"
#include "srvinit_pdump.h"

#include "rgx_fwif_hwperf.h"
#include "rgx_hwperf_table.h"

#include "rgxsrvinit_script.h"

#include "rgxfwload.h"
#include "rgxlayer_impl.h"
#include "rgxfwimageutils.h"

#include "rgx_hwperf_km.h"
#if !defined(SUPPORT_KERNEL_SRVINIT)
#include "rgx_hwperf.h"
/* Check server assumptions of certain HWPerf types which can not be used 
 * directly in the KM server. */
static_assert(RGXFW_HWPERF_L1_PADDING_DEFAULT>=RGX_HWPERF_V2_MAX_PACKET_SIZE,
			  "RGXFW_HWPERF_L1_PADDING_DEFAULT overflows max HWPerf packet size");
#endif

#if defined(SUPPORT_TRUSTED_DEVICE)
#if !defined(SUPPORT_KERNEL_SRVINIT)
#error "SUPPORT_KERNEL_SRVINIT is required by SUPPORT_TRUSTED_DEVICE!"
#endif
#include "rgxdevice.h"
#include "pvrsrv_device.h"
#endif

static RGX_INIT_COMMAND asDbgCommands[RGX_MAX_DEBUG_COMMANDS];
static RGX_INIT_COMMAND asDbgBusCommands[RGX_MAX_DBGBUS_COMMANDS];
static RGX_INIT_COMMAND asDeinitCommands[RGX_MAX_DEINIT_COMMANDS];


#define	HW_PERF_FILTER_DEFAULT         0x00000000 /* Default to no HWPerf */
#define HW_PERF_FILTER_DEFAULT_ALL_ON  0xFFFFFFFF /* All events */

/* Services initialisation parameters */
SrvInitParamInitBOOL(EnableSignatureChecks, RGXFW_SIG_CHECKS_ENABLED_DEFAULT);
SrvInitParamInitUINT32(SignatureChecksBufSize, RGXFW_SIG_BUFFER_SIZE_DEFAULT);
SrvInitParamInitBOOL(Enable2ndThread, IMG_FALSE);
SrvInitParamInitUINT32(EnableFWContextSwitch, RGXFWIF_INICFG_CTXSWITCH_DM_ALL);
SrvInitParamInitUINT32(FWContextSwitchProfile, 2); /* Default to maximum coverage */
SrvInitParamInitUINT32(FirmwarePerf, FW_PERF_CONF_NONE);
SrvInitParamInitBOOL(EnableHWPerf, IMG_FALSE);
SrvInitParamInitUINT32(HWPerfFWBufSizeInKB, 0);	/* Default to server default */
SrvInitParamInitUINT32(HWPerfFilter0, HW_PERF_FILTER_DEFAULT);
SrvInitParamInitUINT32(HWPerfFilter1, HW_PERF_FILTER_DEFAULT);
SrvInitParamInitBOOL(EnableHWPerfHost, IMG_FALSE);
SrvInitParamInitUINT32(HWPerfHostFilter, HW_PERF_FILTER_DEFAULT);
SrvInitParamInitBOOL(HWPerfDisableCustomCounterFilter, IMG_FALSE);
SrvInitParamInitUINT32(EnableAPM, RGX_ACTIVEPM_DEFAULT);
SrvInitParamInitUINT32(EnableRDPowerIsland, RGX_RD_POWER_ISLAND_DEFAULT);
SrvInitParamInitUINT32(UseMETAT1, RGX_META_T1_OFF); /* Default to not using thread 1 */
SrvInitParamInitBOOL(DustRequestInject, IMG_FALSE);

#if defined(HWR_DEFAULT_ENABLED)
SrvInitParamInitBOOL(EnableHWR, IMG_TRUE);
#else
SrvInitParamInitBOOL(EnableHWR, IMG_FALSE);
#endif

#if defined(DEBUG)
SrvInitParamInitUINT32(HWRDebugDumpLimit, RGXFWIF_HWR_DEBUG_DUMP_ALL);
SrvInitParamInitBOOL(CheckMList, IMG_TRUE);
#else
SrvInitParamInitUINT32(HWRDebugDumpLimit, 1); /* dump only first HWR */
SrvInitParamInitBOOL(CheckMList, IMG_FALSE);
#endif
SrvInitParamInitBOOL(ZeroFreelist, IMG_FALSE);
SrvInitParamInitBOOL(DisableClockGating, IMG_FALSE);
SrvInitParamInitBOOL(DisablePDP, IMG_FALSE);
#if defined(SUPPORT_GPUTRACE_EVENTS)
SrvInitParamInitBOOL(EnableFTraceGPU, IMG_FALSE);
#endif
#if defined(HW_ERN_41805) || defined(HW_ERN_42606)
SrvInitParamInitUINT32(TruncateMode, 0);
#endif
#if defined(HW_ERN_42290) && defined(RGX_FEATURE_TPU_FILTERING_MODE_CONTROL)
SrvInitParamInitBOOL(NewFilteringMode, IMG_TRUE);
#endif
#if defined(RGX_FEATURE_S7_TOP_INFRASTRUCTURE)
SrvInitParamInitUINT32(JonesDisableMask, 0);
#endif
SrvInitParamInitBOOL(DisableFEDLogging, IMG_FALSE);
SrvInitParamInitBOOL(EnableRTUBypass, IMG_FALSE);
#if defined(DEBUG)
SrvInitParamInitBOOL(AssertOutOfMemory, IMG_FALSE);
#endif
SrvInitParamInitBOOL(EnableCDMKillingRandMode, IMG_FALSE);
SrvInitParamInitBOOL(DisableDMOverlap, IMG_FALSE);

static SRV_INIT_PARAM_UINT32_LOOKUP asLogTypeTable[] = {
	{ "trace", 2},
	{ "tbi", 1},
	{ "none", 0}
};
SrvInitParamInitUINT32List(FirmwareLogType, 0, asLogTypeTable); 

static SRV_INIT_PARAM_UINT32_LOOKUP asLogGroupTable[] = { RGXFWIF_LOG_GROUP_NAME_VALUE_MAP };
SrvInitParamInitUINT32BitField(EnableLogGroup, 0, asLogGroupTable);

#if	defined(PVRSRV_GPUVIRT_GUESTDRV)
	/* These functionality is n/a to guest drivers */
#else
/************************************************************************
* Private functions
************************************************************************/
/*!
*******************************************************************************

 @Function		PrepareDebugScript

 @Description	Generates a script to dump debug info

 @Input			psScript

 @Return		IMG_BOOL True if it runs out of cmds when building the script

******************************************************************************/
static IMG_BOOL PrepareDebugScript(RGX_SCRIPT_BUILD* psDbgInitScript, IMG_BOOL bFirmwarePerf)
{
#define DBG_READ(T, R, S)		if (!ScriptDBGReadRGXReg(psDbgInitScript, T, R, S)) return IMG_FALSE;
#define DBG_MSP_READ(R, S)		if (!ScriptDBGReadMetaRegThroughSP(psDbgInitScript, R, S)) return IMG_FALSE;
#define DBG_MCR_READ(R, S)		if (!ScriptDBGReadMetaCoreReg(psDbgInitScript, R, S)) return IMG_FALSE;
#define DBG_CALC(R, S, T, U, V)	if (!ScriptDBGCalc(psDbgInitScript, R, S, T, U, V)) return IMG_FALSE;
#if defined(FIX_HW_BRN_44871)
#define DBG_STRING(S)			if (!ScriptDBGString(psDbgInitScript, S)) return IMG_FALSE;
#endif
#define DBG_READ32(R, S)				DBG_READ(RGX_INIT_OP_DBG_READ32_HW_REG, R, S)
#define DBG_READ64(R, S)				DBG_READ(RGX_INIT_OP_DBG_READ64_HW_REG, R, S)
#define DBG_CALC_TA_AND_3D(R, S, T, U)	DBG_CALC(RGX_INIT_OP_DBG_CALC, R, S, T, U)

#if defined(RGX_FEATURE_MIPS)
	PVR_UNREFERENCED_PARAMETER(bFirmwarePerf);
#endif

	DBG_READ32(RGX_CR_CORE_ID,							"CORE_ID                         ");
	DBG_READ32(RGX_CR_CORE_REVISION,					"CORE_REVISION                   ");
	DBG_READ32(RGX_CR_DESIGNER_REV_FIELD1,				"DESIGNER_REV_FIELD1             ");
	DBG_READ32(RGX_CR_DESIGNER_REV_FIELD2,				"DESIGNER_REV_FIELD2             ");
	DBG_READ64(RGX_CR_CHANGESET_NUMBER,					"CHANGESET_NUMBER                ");
#if defined(RGX_FEATURE_META)
	DBG_READ32(RGX_CR_META_SP_MSLVIRQSTATUS,			"META_SP_MSLVIRQSTATUS           ");
#endif
	DBG_READ64(RGX_CR_CLK_CTRL,							"CLK_CTRL                        ");
	DBG_READ64(RGX_CR_CLK_STATUS,						"CLK_STATUS                      ");
	DBG_READ64(RGX_CR_CLK_CTRL2,						"CLK_CTRL2                       ");
	DBG_READ64(RGX_CR_CLK_STATUS2,						"CLK_STATUS2                     ");
#if defined(RGX_FEATURE_S7_TOP_INFRASTRUCTURE)
	DBG_READ64(RGX_CR_CLK_XTPLUS_CTRL,					"CLK_XTPLUS_CTRL                 ");
	DBG_READ64(RGX_CR_CLK_XTPLUS_STATUS,				"CLK_XTPLUS_STATUS               ");
#endif /* RGX_FEATURE_S7_TOP_INFRASTRUCTURE */
	DBG_READ32(RGX_CR_EVENT_STATUS,						"EVENT_STATUS                    ");
#if defined(RGX_FEATURE_S7_TOP_INFRASTRUCTURE)
	DBG_READ64(RGX_CR_MMU_FAULT_STATUS,					"MMU_FAULT_STATUS                ");
	DBG_READ64(RGX_CR_MMU_FAULT_STATUS_META,			"MMU_FAULT_STATUS_META           ");
#else
	DBG_READ32(RGX_CR_BIF_FAULT_BANK0_MMU_STATUS,		"BIF_FAULT_BANK0_MMU_STATUS      ");
	DBG_READ64(RGX_CR_BIF_FAULT_BANK0_REQ_STATUS,		"BIF_FAULT_BANK0_REQ_STATUS      ");
	DBG_READ32(RGX_CR_BIF_FAULT_BANK1_MMU_STATUS,		"BIF_FAULT_BANK1_MMU_STATUS      ");
	DBG_READ64(RGX_CR_BIF_FAULT_BANK1_REQ_STATUS,		"BIF_FAULT_BANK1_REQ_STATUS      ");
#endif	
	DBG_READ32(RGX_CR_BIF_MMU_STATUS,					"BIF_MMU_STATUS                  ");
	DBG_READ32(RGX_CR_BIF_MMU_ENTRY,					"BIF_MMU_ENTRY                   ");
	DBG_READ64(RGX_CR_BIF_MMU_ENTRY_STATUS,				"BIF_MMU_ENTRY_STATUS            ");
#if defined(RGX_FEATURE_S7_TOP_INFRASTRUCTURE)
	DBG_READ32(RGX_CR_BIF_JONES_OUTSTANDING_READ,		"BIF_JONES_OUTSTANDING_READ      ");
	DBG_READ32(RGX_CR_BIF_BLACKPEARL_OUTSTANDING_READ,	"BIF_BLACKPEARL_OUTSTANDING_READ ");
	DBG_READ32(RGX_CR_BIF_DUST_OUTSTANDING_READ,		"BIF_DUST_OUTSTANDING_READ       ");
#else
#if !defined(RGX_FEATURE_XT_TOP_INFRASTRUCTURE)
	DBG_READ32(RGX_CR_BIF_STATUS_MMU,					"BIF_STATUS_MMU                  ");
	DBG_READ32(RGX_CR_BIF_READS_EXT_STATUS,				"BIF_READS_EXT_STATUS            ");
	DBG_READ32(RGX_CR_BIF_READS_INT_STATUS,				"BIF_READS_INT_STATUS            ");
#endif
	DBG_READ32(RGX_CR_BIFPM_STATUS_MMU,					"BIFPM_STATUS_MMU                ");
	DBG_READ32(RGX_CR_BIFPM_READS_EXT_STATUS,			"BIFPM_READS_EXT_STATUS          ");
	DBG_READ32(RGX_CR_BIFPM_READS_INT_STATUS,			"BIFPM_READS_INT_STATUS          ");
#endif
#if defined(FIX_HW_BRN_44871)
	DBG_STRING("Warning: BRN44871 is present");
#endif
	DBG_READ32(RGX_CR_SLC_STATUS0,						"SLC_STATUS0                     ");
	DBG_READ64(RGX_CR_SLC_STATUS1,						"SLC_STATUS1                     ");
	DBG_READ64(RGX_CR_SLC_STATUS2,						"SLC_STATUS2                     ");

#if defined(RGX_FEATURE_SLC_VIVT)
	DBG_READ64(RGX_CR_CONTEXT_MAPPING0,					"CONTEXT_MAPPING0                ");
	DBG_READ64(RGX_CR_CONTEXT_MAPPING1,					"CONTEXT_MAPPING1                ");
	DBG_READ64(RGX_CR_CONTEXT_MAPPING2,					"CONTEXT_MAPPING2                ");
	DBG_READ64(RGX_CR_CONTEXT_MAPPING3,					"CONTEXT_MAPPING3                ");
	DBG_READ64(RGX_CR_CONTEXT_MAPPING4,					"CONTEXT_MAPPING4                ");
#else
	DBG_READ64(RGX_CR_BIF_CAT_BASE_INDEX,				"BIF_CAT_BASE_INDEX              ");
	DBG_READ64(RGX_CR_BIF_CAT_BASE0,					"BIF_CAT_BASE0                   ");
	DBG_READ64(RGX_CR_BIF_CAT_BASE1,					"BIF_CAT_BASE1                   ");
	DBG_READ64(RGX_CR_BIF_CAT_BASE2,					"BIF_CAT_BASE2                   ");
	DBG_READ64(RGX_CR_BIF_CAT_BASE3,					"BIF_CAT_BASE3                   ");
	DBG_READ64(RGX_CR_BIF_CAT_BASE4,					"BIF_CAT_BASE4                   ");
	DBG_READ64(RGX_CR_BIF_CAT_BASE5,					"BIF_CAT_BASE5                   ");
	DBG_READ64(RGX_CR_BIF_CAT_BASE6,					"BIF_CAT_BASE6                   ");
	DBG_READ64(RGX_CR_BIF_CAT_BASE7,					"BIF_CAT_BASE7                   ");
#endif

	DBG_READ32(RGX_CR_BIF_CTRL_INVAL,					"BIF_CTRL_INVAL                  ");
	DBG_READ32(RGX_CR_BIF_CTRL,							"BIF_CTRL                        ");

	DBG_READ64(RGX_CR_BIF_PM_CAT_BASE_VCE0,				"BIF_PM_CAT_BASE_VCE0            ");
	DBG_READ64(RGX_CR_BIF_PM_CAT_BASE_TE0,				"BIF_PM_CAT_BASE_TE0             ");
	DBG_READ64(RGX_CR_BIF_PM_CAT_BASE_ALIST0,			"BIF_PM_CAT_BASE_ALIST0          ");
	DBG_READ64(RGX_CR_BIF_PM_CAT_BASE_VCE1,				"BIF_PM_CAT_BASE_VCE1            ");
	DBG_READ64(RGX_CR_BIF_PM_CAT_BASE_TE1,				"BIF_PM_CAT_BASE_TE1             ");
	DBG_READ64(RGX_CR_BIF_PM_CAT_BASE_ALIST1,			"BIF_PM_CAT_BASE_ALIST1          ");
	
	DBG_READ32(RGX_CR_PERF_TA_PHASE,					"PERF_TA_PHASE                   ");
	DBG_READ32(RGX_CR_PERF_TA_CYCLE,					"PERF_TA_CYCLE                   ");
	DBG_READ32(RGX_CR_PERF_3D_PHASE,					"PERF_3D_PHASE                   ");
	DBG_READ32(RGX_CR_PERF_3D_CYCLE,					"PERF_3D_CYCLE                   ");

	DBG_READ32(RGX_CR_PERF_TA_OR_3D_CYCLE,				"PERF_TA_OR_3D_CYCLE             ");
	DBG_CALC_TA_AND_3D(RGX_CR_PERF_TA_CYCLE, RGX_CR_PERF_3D_CYCLE, RGX_CR_PERF_TA_OR_3D_CYCLE,
														"PERF_TA_AND_3D_CYCLE            ");

	DBG_READ32(RGX_CR_PERF_COMPUTE_PHASE,				"PERF_COMPUTE_PHASE              ");
	DBG_READ32(RGX_CR_PERF_COMPUTE_CYCLE,				"PERF_COMPUTE_CYCLE              ");

	DBG_READ32(RGX_CR_PM_PARTIAL_RENDER_ENABLE,			"PARTIAL_RENDER_ENABLE           ");

	DBG_READ32(RGX_CR_ISP_RENDER,						"ISP_RENDER                      ");
	DBG_READ64(RGX_CR_TLA_STATUS,						"TLA_STATUS                      ");
	DBG_READ64(RGX_CR_MCU_FENCE,						"MCU_FENCE                       ");

	DBG_READ32(RGX_CR_VDM_CONTEXT_STORE_STATUS,			"VDM_CONTEXT_STORE_STATUS        ");
	DBG_READ64(RGX_CR_VDM_CONTEXT_STORE_TASK0,			"VDM_CONTEXT_STORE_TASK0         ");
	DBG_READ64(RGX_CR_VDM_CONTEXT_STORE_TASK1,			"VDM_CONTEXT_STORE_TASK1         ");
	DBG_READ64(RGX_CR_VDM_CONTEXT_STORE_TASK2,			"VDM_CONTEXT_STORE_TASK2         ");
	DBG_READ64(RGX_CR_VDM_CONTEXT_RESUME_TASK0,			"VDM_CONTEXT_RESUME_TASK0        ");
	DBG_READ64(RGX_CR_VDM_CONTEXT_RESUME_TASK1,			"VDM_CONTEXT_RESUME_TASK1        ");
	DBG_READ64(RGX_CR_VDM_CONTEXT_RESUME_TASK2,			"VDM_CONTEXT_RESUME_TASK2        ");

	DBG_READ32(RGX_CR_ISP_CTL,							"ISP_CTL                         ");
	DBG_READ32(RGX_CR_ISP_STATUS,						"ISP_STATUS                      ");
#if !defined(RGX_FEATURE_S7_TOP_INFRASTRUCTURE)
	DBG_READ32(RGX_CR_ISP_STORE0,						"ISP_STORE0                      ");
	DBG_READ32(RGX_CR_ISP_STORE1,						"ISP_STORE1                      ");
	DBG_READ32(RGX_CR_ISP_STORE2,						"ISP_STORE2                      ");
	#if (RGX_FEATURE_NUM_ISP_IPP_PIPES > 3)
	DBG_READ32(RGX_CR_ISP_STORE3,						"ISP_STORE3                      ");
	DBG_READ32(RGX_CR_ISP_STORE4,						"ISP_STORE4                      ");
	DBG_READ32(RGX_CR_ISP_STORE5,						"ISP_STORE5                      ");
	#endif
	#if (RGX_FEATURE_NUM_ISP_IPP_PIPES > 6)
	DBG_READ32(RGX_CR_ISP_STORE6,						"ISP_STORE6                      ");
	DBG_READ32(RGX_CR_ISP_STORE7,						"ISP_STORE7                      ");
	#endif
	DBG_READ32(RGX_CR_ISP_RESUME0,						"ISP_RESUME0                     ");
	DBG_READ32(RGX_CR_ISP_RESUME1,						"ISP_RESUME1                     ");
	DBG_READ32(RGX_CR_ISP_RESUME2,						"ISP_RESUME2                     ");
	#if (RGX_FEATURE_NUM_ISP_IPP_PIPES > 3)
	DBG_READ32(RGX_CR_ISP_RESUME3,						"ISP_RESUME3                     ");
	DBG_READ32(RGX_CR_ISP_RESUME4,						"ISP_RESUME4                     ");
	DBG_READ32(RGX_CR_ISP_RESUME5,						"ISP_RESUME5                     ");
	#endif
	#if (RGX_FEATURE_NUM_ISP_IPP_PIPES > 6)
	DBG_READ32(RGX_CR_ISP_RESUME6,						"ISP_RESUME6                     ");
	DBG_READ32(RGX_CR_ISP_RESUME7,						"ISP_RESUME7                     ");
	#endif
#else
	{
		IMG_UINT	uiPipe;
		IMG_CHAR	pszLabel[64]; /* larger than necessary in case alignment changes */
		for (uiPipe = 0; uiPipe < RGX_FEATURE_NUM_ISP_IPP_PIPES; uiPipe++)
		{
			OSSNPrintf(pszLabel, sizeof(pszLabel),		"ISP_XTP_STORE%02d                 ", uiPipe);
			DBG_READ32(RGX_CR_ISP_XTP_STORE0 + 0x8*uiPipe, pszLabel);
		}
		for (uiPipe = 0; uiPipe < RGX_FEATURE_NUM_ISP_IPP_PIPES; uiPipe++)
		{
			OSSNPrintf(pszLabel, sizeof(pszLabel),		"ISP_XTP_RESUME%02d                ", uiPipe);
			DBG_READ32(RGX_CR_ISP_XTP_RESUME0 + 0x8*uiPipe, pszLabel);
		}
	}
#endif /* RGX_FEATURE_S7_TOP_INFRASTRUCTURE */
	DBG_READ32(RGX_CR_MTS_INTCTX,						"MTS_INTCTX                      ");
	DBG_READ32(RGX_CR_MTS_BGCTX,						"MTS_BGCTX                       ");
	DBG_READ32(RGX_CR_MTS_BGCTX_COUNTED_SCHEDULE,		"MTS_BGCTX_COUNTED_SCHEDULE      ");
	DBG_READ32(RGX_CR_MTS_SCHEDULE,						"MTS_SCHEDULE                    ");
	DBG_READ32(RGX_CR_MTS_GPU_INT_STATUS,				"MTS_GPU_INT_STATUS              ");

	DBG_READ32(RGX_CR_CDM_CONTEXT_STORE_STATUS,			"CDM_CONTEXT_STORE_STATUS        ");
	DBG_READ64(RGX_CR_CDM_CONTEXT_PDS0,					"CDM_CONTEXT_PDS0                ");
	DBG_READ64(RGX_CR_CDM_CONTEXT_PDS1,					"CDM_CONTEXT_PDS1                ");
	DBG_READ64(RGX_CR_CDM_TERMINATE_PDS,				"CDM_TERMINATE_PDS               ");
	DBG_READ64(RGX_CR_CDM_TERMINATE_PDS1,				"CDM_TERMINATE_PDS1              ");
#if defined(HW_ERN_47025)
	DBG_READ64(RGX_CR_CDM_CONTEXT_LOAD_PDS0,			"CDM_CONTEXT_LOAD_PDS0           ");
	DBG_READ64(RGX_CR_CDM_CONTEXT_LOAD_PDS1,			"CDM_CONTEXT_LOAD_PDS1           ");
#endif
	
#if defined(RGX_FEATURE_RAY_TRACING)
	DBG_READ32(DPX_CR_BIF_MMU_STATUS,					"DPX_CR_BIF_MMU_STATUS           ");
	DBG_READ64(DPX_CR_BIF_FAULT_BANK_MMU_STATUS,		"DPX_CR_BIF_FAULT_BANK_MMU_STATUS");
	DBG_READ64(DPX_CR_BIF_FAULT_BANK_REQ_STATUS,		"DPX_CR_BIF_FAULT_BANK_REQ_STATUS");

	DBG_READ64(RGX_CR_RPM_SHF_FPL,						"RGX_CR_RPM_SHF_FPL	             ");
	DBG_READ32(RGX_CR_RPM_SHF_FPL_READ,					"RGX_CR_RPM_SHF_FPL_READ         ");
	DBG_READ32(RGX_CR_RPM_SHF_FPL_WRITE,				"RGX_CR_RPM_SHF_FPL_WRITE        ");
	DBG_READ64(RGX_CR_RPM_SHG_FPL,   					"RGX_CR_RPM_SHG_FPL	             ");
	DBG_READ32(RGX_CR_RPM_SHG_FPL_READ,					"RGX_CR_RPM_SHG_FPL_READ         ");
	DBG_READ32(RGX_CR_RPM_SHG_FPL_WRITE,				"RGX_CR_RPM_SHG_FPL_WRITE        ");
#endif
#if defined(RGX_FEATURE_S7_TOP_INFRASTRUCTURE)
	DBG_READ32(RGX_CR_JONES_IDLE,						"JONES_IDLE                      ");
#endif
	DBG_READ32(RGX_CR_SIDEKICK_IDLE,					"SIDEKICK_IDLE                   ");
#if !defined(RGX_FEATURE_S7_TOP_INFRASTRUCTURE)
	DBG_READ32(RGX_CR_SLC_IDLE,							"SLC_IDLE                        ");
#else
	DBG_READ32(RGX_CR_SLC3_IDLE,						"SLC3_IDLE                       ");
	DBG_READ64(RGX_CR_SLC3_STATUS,						"SLC3_STATUS                     ");
	DBG_READ32(RGX_CR_SLC3_FAULT_STOP_STATUS,			"SLC3_FAULT_STOP_STATUS          ");
#endif
#if defined(RGX_FEATURE_META)
	DBG_MSP_READ(META_CR_T0ENABLE_OFFSET,				"T0 TXENABLE                     ");
	DBG_MSP_READ(META_CR_T0STATUS_OFFSET,				"T0 TXSTATUS                     ");
	DBG_MSP_READ(META_CR_T0DEFR_OFFSET,					"T0 TXDEFR                       ");
	DBG_MCR_READ(META_CR_THR0_PC,						"T0 PC                           ");
	DBG_MCR_READ(META_CR_THR0_PCX,						"T0 PCX                          ");
	DBG_MCR_READ(META_CR_THR0_SP,						"T0 SP                           ");

#if (RGX_FEATURE_META == MTP218) || (RGX_FEATURE_META == MTP219)
	DBG_MSP_READ(META_CR_T1ENABLE_OFFSET,				"T1 TXENABLE                     ");
	DBG_MSP_READ(META_CR_T1STATUS_OFFSET,				"T1 TXSTATUS                     ");
	DBG_MSP_READ(META_CR_T1DEFR_OFFSET,					"T1 TXDEFR                       ");
	DBG_MCR_READ(META_CR_THR1_PC,						"T1 PC                           ");
	DBG_MCR_READ(META_CR_THR1_PCX,						"T1 PCX                          ");
	DBG_MCR_READ(META_CR_THR1_SP,						"T1 SP                           ");
#endif

	if (bFirmwarePerf)
	{
		DBG_MSP_READ(META_CR_PERF_COUNT0,				"PERF_COUNT0                     ");
		DBG_MSP_READ(META_CR_PERF_COUNT1,				"PERF_COUNT1                     ");
	}
#endif
#if defined(RGX_FEATURE_MIPS)
	DBG_READ32(RGX_CR_MIPS_EXCEPTION_STATUS,            "MIPS_EXCEPTION_STATUS           ");
#endif

	return IMG_TRUE;
}

/*!
*******************************************************************************

 @Function		PrepareDebugScript

 @Description	Generates a script to dump debug info

 @Input			psScript

 @Return		IMG_BOOL True if it runs out of cmds when building the script

******************************************************************************/
static IMG_BOOL PrepareDebugBusScript(RGX_SCRIPT_BUILD* psDbgBusScript)
{
#if defined(RGX_FEATURE_PERFBUS) && defined(SUPPORT_DEBUG_BUS_DUMP)
	IMG_UINT32 i;
#define DUMP_DEBUG_BUS(N,R,C)	if (!ScriptDBGBusReadBlock(psDbgBusScript, N, R, C)) return IMG_FALSE;
#define DBG_WRITE(R,V)			if (!ScriptWriteRGXReg(psDbgBusScript, R, V)) return IMG_FALSE;
									  
	DUMP_DEBUG_BUS("TA group", RGX_CR_TA_PERF, 9);
	DUMP_DEBUG_BUS("RAST group", RGX_CR_RASTERISATION_PERF, 10);
	DUMP_DEBUG_BUS("HUB group", RGX_CR_HUB_BIFPMCACHE_PERF, 17);

	for (i = 0; i < RGX_HWPERF_PHANTOM_INDIRECT_BY_DUST; i++)
	{
		IMG_CHAR aszGroupName[100];
		DBG_WRITE(RGX_CR_TPU_MCU_L0_PERF_INDIRECT, i);
		snprintf(aszGroupName, sizeof(aszGroupName), "TPU group %d", i);
		DUMP_DEBUG_BUS(aszGroupName, RGX_CR_TPU_MCU_L0_PERF, 6);
	}

	for (i = 0; i < RGX_HWPERF_PHANTOM_INDIRECT_BY_CLUSTER; i++)
	{
		IMG_CHAR aszGroupName[100];
		DBG_WRITE(RGX_CR_USC_PERF_INDIRECT, i);
		snprintf(aszGroupName, sizeof(aszGroupName), "USC group %d", i);
		DUMP_DEBUG_BUS(aszGroupName, RGX_CR_USC_PERF, 11);
	}
#undef DUMP_DEBUG_BUS
#undef DBG_WRITE
#else
	PVR_UNREFERENCED_PARAMETER(psDbgBusScript);
#endif /* RGX_FEATURE_PERFBUS && SUPPORT_DEBUG_BUS_DUMP*/

	return IMG_TRUE;
}

/*!
*******************************************************************************

 @Function		PrepareDeinitScript

 @Description	Generates a script to suspend the hw threads

 @Input			psScript

 @Return		IMG_BOOL True if it runs out of cmds when building the script

******************************************************************************/
static IMG_BOOL PrepareDeinitScript(RGX_SCRIPT_BUILD* psScript)
{
	/* Wait for Sidekick to signal IDLE except for the Garten Wrapper */
#if !defined(RGX_FEATURE_S7_TOP_INFRASTRUCTURE)
	if(!ScriptPollRGXReg(
			psScript, 
			RGX_CR_SIDEKICK_IDLE,
			RGX_CR_SIDEKICK_IDLE_MASKFULL^(RGX_CR_SIDEKICK_IDLE_GARTEN_EN|RGX_CR_SIDEKICK_IDLE_SOCIF_EN|RGX_CR_SIDEKICK_IDLE_HOSTIF_EN),
			RGX_CR_SIDEKICK_IDLE_MASKFULL^(RGX_CR_SIDEKICK_IDLE_GARTEN_EN|RGX_CR_SIDEKICK_IDLE_SOCIF_EN|RGX_CR_SIDEKICK_IDLE_HOSTIF_EN))) return IMG_FALSE;
#else
	if(!ScriptPollRGXReg(
			psScript, 
			RGX_CR_JONES_IDLE,
			RGX_CR_JONES_IDLE_MASKFULL^(RGX_CR_JONES_IDLE_GARTEN_EN|RGX_CR_JONES_IDLE_SOCIF_EN|RGX_CR_JONES_IDLE_HOSTIF_EN),
			RGX_CR_JONES_IDLE_MASKFULL^(RGX_CR_JONES_IDLE_GARTEN_EN|RGX_CR_JONES_IDLE_SOCIF_EN|RGX_CR_JONES_IDLE_HOSTIF_EN))) return IMG_FALSE;
#endif

#if !defined(SUPPORT_SHARED_SLC)
	/* Wait for SLC to signal IDLE */
#if !defined(RGX_FEATURE_S7_TOP_INFRASTRUCTURE)
	if(!ScriptPollRGXReg(
			psScript, 
			RGX_CR_SLC_IDLE,
			RGX_CR_SLC_IDLE_MASKFULL,
			RGX_CR_SLC_IDLE_MASKFULL)) return IMG_FALSE;
#else
	if(!ScriptPollRGXReg(
			psScript, 
			RGX_CR_SLC3_IDLE,
			RGX_CR_SLC3_IDLE_MASKFULL,
			RGX_CR_SLC3_IDLE_MASKFULL)) return IMG_FALSE;
#endif /* RGX_FEATURE_S7_TOP_INFRASTRUCTURE */
#endif /* SUPPORT_SHARED_SLC */

	/* unset MTS DM association with threads */
	if(!ScriptWriteRGXReg(
			psScript, 
			RGX_CR_MTS_INTCTX_THREAD0_DM_ASSOC,
			RGX_CR_MTS_INTCTX_THREAD0_DM_ASSOC_DM_ASSOC_CLRMSK & RGX_CR_MTS_INTCTX_THREAD0_DM_ASSOC_MASKFULL)) return IMG_FALSE;
	if(!ScriptWriteRGXReg(
			psScript, 
			RGX_CR_MTS_BGCTX_THREAD0_DM_ASSOC,
			RGX_CR_MTS_BGCTX_THREAD0_DM_ASSOC_DM_ASSOC_CLRMSK & RGX_CR_MTS_BGCTX_THREAD0_DM_ASSOC_MASKFULL)) return IMG_FALSE;

	if(!ScriptWriteRGXReg(
			psScript, 
			RGX_CR_MTS_INTCTX_THREAD1_DM_ASSOC,
			RGX_CR_MTS_INTCTX_THREAD1_DM_ASSOC_DM_ASSOC_CLRMSK & RGX_CR_MTS_INTCTX_THREAD1_DM_ASSOC_MASKFULL)) return IMG_FALSE;
	if(!ScriptWriteRGXReg(
			psScript, 
			RGX_CR_MTS_BGCTX_THREAD1_DM_ASSOC,
			RGX_CR_MTS_BGCTX_THREAD1_DM_ASSOC_DM_ASSOC_CLRMSK & RGX_CR_MTS_BGCTX_THREAD1_DM_ASSOC_MASKFULL)) return IMG_FALSE;

#if defined(RGX_FEATURE_META)
	/* Disabling threads is only required for pdumps to stop the fw gracefully */
	/* Disable thread 0 */
	if(!ScriptWriteMetaRegThroughSP(psScript, 
						 META_CR_T0ENABLE_OFFSET,
						 ~META_CR_TXENABLE_ENABLE_BIT)) return IMG_FALSE;

	/* Disable thread 1 */
	if(!ScriptWriteMetaRegThroughSP(psScript, 
						 META_CR_T1ENABLE_OFFSET,
						 ~META_CR_TXENABLE_ENABLE_BIT)) return IMG_FALSE;

	/* Only in PDUMPS: Clear down any irq raised by META 
	   (done after disabling the FW threads to avoid a race condition) */
	if(!ScriptWriteRGXRegPDUMPOnly(
			psScript, 
			RGX_CR_META_SP_MSLVIRQSTATUS,
			0x0)) return IMG_FALSE;

	/* Wait for the Slave Port to finish all the transactions */
	if(!ScriptPollRGXReg(
			psScript, 
			RGX_CR_META_SP_MSLVCTRL1, 
			RGX_CR_META_SP_MSLVCTRL1_READY_EN|RGX_CR_META_SP_MSLVCTRL1_GBLPORT_IDLE_EN, 
			RGX_CR_META_SP_MSLVCTRL1_READY_EN|RGX_CR_META_SP_MSLVCTRL1_GBLPORT_IDLE_EN)
	  ) return IMG_FALSE;
#endif
	/* Extra Idle checks */
	if(!ScriptPollRGXReg(
			psScript, 
			RGX_CR_BIF_STATUS_MMU, 
			0,
			RGX_CR_BIF_STATUS_MMU_MASKFULL)
	  ) return IMG_FALSE;
	if(!ScriptPollRGXReg(
			psScript, 
			RGX_CR_BIFPM_STATUS_MMU, 
			0,
			RGX_CR_BIFPM_STATUS_MMU_MASKFULL)
	  ) return IMG_FALSE;
#if !defined(RGX_FEATURE_XT_TOP_INFRASTRUCTURE) && !defined(RGX_FEATURE_S7_TOP_INFRASTRUCTURE)
	if(!ScriptPollRGXReg(
			psScript, 
			RGX_CR_BIF_READS_EXT_STATUS, 
			0,
			RGX_CR_BIF_READS_EXT_STATUS_MASKFULL)
	  ) return IMG_FALSE;
#endif
	if(!ScriptPollRGXReg(
			psScript, 
			RGX_CR_BIFPM_READS_EXT_STATUS, 
			0,
			RGX_CR_BIFPM_READS_EXT_STATUS_MASKFULL)
	  ) return IMG_FALSE;
	if(!ScriptPoll64RGXReg
		(
			psScript, 
			RGX_CR_SLC_STATUS1, 
			0,
#if defined(FIX_HW_BRN_43276)
			RGX_CR_SLC_STATUS1_MASKFULL & RGX_CR_SLC_STATUS1_READS1_EXT_CLRMSK & RGX_CR_SLC_STATUS1_READS0_EXT_CLRMSK)
#else
			RGX_CR_SLC_STATUS1_MASKFULL)
#endif 
	  ) return IMG_FALSE;

	if(!ScriptPoll64RGXReg(
			psScript, 
			RGX_CR_SLC_STATUS2, 
			0,
			RGX_CR_SLC_STATUS2_MASKFULL)
	  ) return IMG_FALSE;

#if !defined(SUPPORT_SHARED_SLC)
	/* Wait for SLC to signal IDLE */
#if !defined(RGX_FEATURE_S7_TOP_INFRASTRUCTURE)
	if(!ScriptPollRGXReg(
			psScript, 
			RGX_CR_SLC_IDLE,
			RGX_CR_SLC_IDLE_MASKFULL,
			RGX_CR_SLC_IDLE_MASKFULL)) return IMG_FALSE;
#else
	if(!ScriptPollRGXReg(
			psScript, 
			RGX_CR_SLC3_IDLE,
			RGX_CR_SLC3_IDLE_MASKFULL,
			RGX_CR_SLC3_IDLE_MASKFULL)) return IMG_FALSE;
#endif /* RGX_FEATURE_S7_TOP_INFRASTRUCTURE */
#endif /* SUPPORT_SHARED_SLC */

#if !defined(RGX_FEATURE_S7_TOP_INFRASTRUCTURE)
	if(!ScriptPollRGXReg(
			psScript, 
			RGX_CR_SIDEKICK_IDLE,
			RGX_CR_SIDEKICK_IDLE_MASKFULL^(RGX_CR_SIDEKICK_IDLE_GARTEN_EN|RGX_CR_SIDEKICK_IDLE_SOCIF_EN|RGX_CR_SIDEKICK_IDLE_HOSTIF_EN),
			RGX_CR_SIDEKICK_IDLE_MASKFULL^(RGX_CR_SIDEKICK_IDLE_GARTEN_EN|RGX_CR_SIDEKICK_IDLE_SOCIF_EN|RGX_CR_SIDEKICK_IDLE_HOSTIF_EN))) return IMG_FALSE;
#else
	if(!ScriptPollRGXReg(
			psScript, 
			RGX_CR_JONES_IDLE,
			RGX_CR_JONES_IDLE_MASKFULL^(RGX_CR_JONES_IDLE_GARTEN_EN|RGX_CR_JONES_IDLE_SOCIF_EN|RGX_CR_JONES_IDLE_HOSTIF_EN),
			RGX_CR_JONES_IDLE_MASKFULL^(RGX_CR_JONES_IDLE_GARTEN_EN|RGX_CR_JONES_IDLE_SOCIF_EN|RGX_CR_JONES_IDLE_HOSTIF_EN))) return IMG_FALSE;
#endif

#if defined(RGX_FEATURE_META)
#if !defined(RGX_FEATURE_S7_TOP_INFRASTRUCTURE)
	/* Wait for Sidekick to signal IDLE including the Garten Wrapper if
	   there is no debugger attached (TxVECINT_BHALT = 0x0) */
	if(!ScriptMetaRegCondPollRGXReg(
			psScript, 
			META_CR_TxVECINT_BHALT,
			0x0,
			0xFFFFFFFFU,
			RGX_CR_SIDEKICK_IDLE,
			RGX_CR_SIDEKICK_IDLE_GARTEN_EN,
			RGX_CR_SIDEKICK_IDLE_GARTEN_EN)) return IMG_FALSE;
#else
	/* Wait for Jone to signal IDLE including the Garten Wrapper if
	   there is no debugger attached (TxVECINT_BHALT = 0x0) */
	if(!ScriptMetaRegCondPollRGXReg(
			psScript, 
			META_CR_TxVECINT_BHALT,
			0x0,
			0xFFFFFFFFU,
			RGX_CR_JONES_IDLE,
			RGX_CR_JONES_IDLE_GARTEN_EN,
			RGX_CR_JONES_IDLE_GARTEN_EN)) return IMG_FALSE;
#endif /* RGX_FEATURE_S7_TOP_INFRASTRUCTURE */
#else
	if(!ScriptPollRGXReg(
			psScript,
			RGX_CR_SIDEKICK_IDLE,
			RGX_CR_SIDEKICK_IDLE_GARTEN_EN,
			RGX_CR_SIDEKICK_IDLE_GARTEN_EN)) return IMG_FALSE;

#endif

	return IMG_TRUE;
}

#if defined(PDUMP)
/*!
*******************************************************************************

 @Function	InitialiseHWPerfCounters

 @Description

 Initialisation of hardware performance counters and dumping them out to pdump, so that they can be modified at a later point.

 @Input hServices

 @Input psHWPerfDataMemDesc

 @Input psHWPerfInitDataInt

 @Return  void

******************************************************************************/

static void InitialiseHWPerfCounters(SHARED_DEV_CONNECTION hServices, DEVMEM_MEMDESC *psHWPerfDataMemDesc, RGXFWIF_HWPERF_CTL *psHWPerfInitDataInt)
{
	RGXFWIF_HWPERF_CTL_BLK *psHWPerfInitBlkData;
	const RGXFW_HWPERF_CNTBLK_TYPE_MODEL* psBlkTypeDesc;
	IMG_UINT32 gasCntBlkModelLen;
	const RGXFW_HWPERF_CNTBLK_TYPE_MODEL *gasCntBlkTypeModel;
	IMG_UINT32 ui32BlockID, ui32BlkCfgIdx, ui32CounterIdx ;

	gasCntBlkModelLen = RGXGetHWPerfBlockConfig(&gasCntBlkTypeModel);
	for(ui32BlkCfgIdx = 0; ui32BlkCfgIdx < gasCntBlkModelLen; ui32BlkCfgIdx++)
	{
		/* Exit early if this core does not have any of these counter blocks
		 * due to core type/BVNC.... */
		psBlkTypeDesc = &gasCntBlkTypeModel[ui32BlkCfgIdx];
		if (psBlkTypeDesc == NULL || psBlkTypeDesc->uiNumUnits == 0)
		{
			continue;
		}
		/* Program all counters in one block so those already on may
		 * be configured off and vice-a-versa. */
		for (ui32BlockID = psBlkTypeDesc->uiCntBlkIdBase;
					 ui32BlockID < psBlkTypeDesc->uiCntBlkIdBase+psBlkTypeDesc->uiNumUnits;
					 ui32BlockID++)
		{

			SRVINITPDumpComment(hServices, "Unit %d Block : %s", ui32BlockID-psBlkTypeDesc->uiCntBlkIdBase, psBlkTypeDesc->pszBlockNameComment);
			/* Get the block configure store to update from the global store of
			 * block configuration. This is used to remember the configuration
			 * between configurations and core power on in APM */
			psHWPerfInitBlkData = rgxfw_hwperf_get_block_ctl(ui32BlockID, psHWPerfInitDataInt);
			/* Assert to check for HWPerf block mis-configuration */
			PVR_ASSERT(psHWPerfInitBlkData);

			psHWPerfInitBlkData->bValid = IMG_TRUE;	
			SRVINITPDumpComment(hServices, "bValid: This specifies if the layout block is valid for the given BVNC.");
			DevmemPDumpLoadMemValue32(psHWPerfDataMemDesc,
							(size_t)&(psHWPerfInitBlkData->bValid) - (size_t)(psHWPerfInitDataInt),
							psHWPerfInitBlkData->bValid,
							PDUMP_FLAGS_CONTINUOUS);

			psHWPerfInitBlkData->bEnabled = IMG_FALSE;
			SRVINITPDumpComment(hServices, "bEnabled: Set to 0x1 if the block needs to be enabled during playback. ");
			DevmemPDumpLoadMemValue32(psHWPerfDataMemDesc,
							(size_t)&(psHWPerfInitBlkData->bEnabled) - (size_t)(psHWPerfInitDataInt),
							psHWPerfInitBlkData->bEnabled,
							PDUMP_FLAGS_CONTINUOUS);

			psHWPerfInitBlkData->eBlockID = ui32BlockID;
			SRVINITPDumpComment(hServices, "eBlockID: The Block ID for the layout block. See RGX_HWPERF_CNTBLK_ID for further information.");
			DevmemPDumpLoadMemValue32(psHWPerfDataMemDesc,
							(size_t)&(psHWPerfInitBlkData->eBlockID) - (size_t)(psHWPerfInitDataInt),
							psHWPerfInitBlkData->eBlockID,
							PDUMP_FLAGS_CONTINUOUS);

			psHWPerfInitBlkData->uiCounterMask = 0x00;
			SRVINITPDumpComment(hServices, "uiCounterMask: Bitmask for selecting the counters that need to be configured.(Bit 0 - counter0, bit 1 - counter1 and so on. ");
			DevmemPDumpLoadMemValue32(psHWPerfDataMemDesc,
							(size_t)&(psHWPerfInitBlkData->uiCounterMask) - (size_t)(psHWPerfInitDataInt),
							psHWPerfInitBlkData->uiCounterMask,
							PDUMP_FLAGS_CONTINUOUS);

			for(ui32CounterIdx = RGX_CNTBLK_COUNTER0_ID; ui32CounterIdx < psBlkTypeDesc->uiNumCounters; ui32CounterIdx++)
			{
				psHWPerfInitBlkData->aui64CounterCfg[ui32CounterIdx] = IMG_UINT64_C(0x0000000000000000);

				SRVINITPDumpComment(hServices, "%s_COUNTER_%d", psBlkTypeDesc->pszBlockNameComment,ui32CounterIdx);
				DevmemPDumpLoadMemValue64(psHWPerfDataMemDesc,
							(size_t)&(psHWPerfInitBlkData->aui64CounterCfg[ui32CounterIdx]) - (size_t)(psHWPerfInitDataInt),
							psHWPerfInitBlkData->aui64CounterCfg[ui32CounterIdx],
							PDUMP_FLAGS_CONTINUOUS);

			}
		}
	}


}
/*!
*******************************************************************************

 @Function	InitialiseCustomCounters

 @Description

 Initialisation of custom counters and dumping them out to pdump, so that they can be modified at a later point.

 @Input hServices

 @Input psHWPerfDataMemDesc

 @Return  void

******************************************************************************/

static void InitialiseCustomCounters(SHARED_DEV_CONNECTION hServices, DEVMEM_MEMDESC *psHWPerfDataMemDesc)
{
	IMG_UINT32 ui32CustomBlock, ui32CounterID;

	SRVINITPDumpComment(hServices, "ui32SelectedCountersBlockMask - The Bitmask of the custom counters that are to be selected");
	DevmemPDumpLoadMemValue32(psHWPerfDataMemDesc,
						offsetof(RGXFWIF_HWPERF_CTL, ui32SelectedCountersBlockMask),
						0,
						PDUMP_FLAGS_CONTINUOUS);

	for( ui32CustomBlock = 0; ui32CustomBlock < RGX_HWPERF_MAX_CUSTOM_BLKS; ui32CustomBlock++ )
	{
			SRVINITPDumpComment(hServices, "ui32NumSelectedCounters - The Number of counters selected for this Custom Block: %d",ui32CustomBlock );
			DevmemPDumpLoadMemValue32(psHWPerfDataMemDesc,
						offsetof(RGXFWIF_HWPERF_CTL, SelCntr[ui32CustomBlock].ui32NumSelectedCounters),
						0,
						PDUMP_FLAGS_CONTINUOUS);

		for(ui32CounterID = 0; ui32CounterID < RGX_HWPERF_MAX_CUSTOM_CNTRS; ui32CounterID++ )
		{
			SRVINITPDumpComment(hServices, "CUSTOMBLK_%d_COUNTERID_%d",ui32CustomBlock, ui32CounterID);
			DevmemPDumpLoadMemValue32(psHWPerfDataMemDesc,
					offsetof(RGXFWIF_HWPERF_CTL, SelCntr[ui32CustomBlock].aui32SelectedCountersIDs[ui32CounterID]),
					0,
					PDUMP_FLAGS_CONTINUOUS);
		}
	}
}
#endif

#if defined(SUPPORT_TRUSTED_DEVICE) && !defined(NO_HARDWARE)
/*************************************************************************/ /*!
 @Function       RGXTDProcessFWImage

 @Description    Fetch and send data used by the trusted device to complete
                 the FW image setup

 @Input          psDeviceNode - Device node
 @Input          psRGXFW      - Firmware blob

 @Return         PVRSRV_ERROR
*/ /**************************************************************************/
static PVRSRV_ERROR RGXTDProcessFWImage(PVRSRV_DEVICE_NODE *psDeviceNode,
                                        struct RGXFW *psRGXFW)
{
	PVRSRV_DEVICE_CONFIG *psDevConfig = psDeviceNode->psDevConfig;
	PVRSRV_RGXDEV_INFO *psDevInfo = psDeviceNode->pvDevice;
	PVRSRV_TD_FW_PARAMS sTDFWParams;
	PVRSRV_ERROR eError;

	if (psDevConfig->pfnTDSendFWImage == NULL)
	{
		PVR_DPF((PVR_DBG_ERROR, "RGXTDProcessFWImage: TDProcessFWImage not implemented!"));
		return PVRSRV_ERROR_NOT_IMPLEMENTED;
	}

	sTDFWParams.pvFirmware = RGXFirmwareData(psRGXFW);
	sTDFWParams.ui32FirmwareSize = RGXFirmwareSize(psRGXFW);
	sTDFWParams.sFWCodeDevVAddrBase = psDevInfo->sFWCodeDevVAddrBase;
	sTDFWParams.sFWDataDevVAddrBase = psDevInfo->sFWDataDevVAddrBase;
	sTDFWParams.sFWCorememCodeFWAddr = psDevInfo->sFWCorememCodeFWAddr;
	sTDFWParams.sFWInitFWAddr = psDevInfo->sFWInitFWAddr;

	eError = psDevConfig->pfnTDSendFWImage(&sTDFWParams);

	return eError;
}
#endif

#endif /* defined(PVRSRV_GPUVIRT_GUESTDRV) */


/*!
*******************************************************************************

 @Function	RGXInit

 @Description

 RGX Initialisation

 @Input hServices

 @Return   PVRSRV_ERROR

******************************************************************************/
IMG_INTERNAL
PVRSRV_ERROR RGXInit(SHARED_DEV_CONNECTION hServices)
{
	PVRSRV_ERROR				eError;
	RGXFWIF_COMPCHECKS_BVNC_DECLARE_AND_INIT(sBVNC);

	IMG_UINT32			aui32RGXFWAlignChecks[] = {RGXFW_ALIGN_CHECKS_INIT};

	/* Server scripts */
	RGX_SCRIPT_BUILD	sDbgInitScript    = {RGX_MAX_DEBUG_COMMANDS,  0, IMG_FALSE, asDbgCommands};
	RGX_SCRIPT_BUILD	sDbgBusInitScript = {RGX_MAX_DBGBUS_COMMANDS, 0, IMG_FALSE, asDbgBusCommands};
	RGX_SCRIPT_BUILD	sDeinitScript     = {RGX_MAX_DEINIT_COMMANDS, 0, IMG_FALSE, asDeinitCommands};

	/* FW code memory */
	IMG_DEVMEM_SIZE_T			uiFWCodeAllocSize = 0;	
	void						*pvFWCodeHostAddr;

	/* FW data memory */
	IMG_DEVMEM_SIZE_T			uiFWDataAllocSize = 0;	
	IMG_DEV_VIRTADDR			sFWDataDevVAddrBase;

	/* FW Coremem buffer */
	IMG_DEVMEM_SIZE_T			uiFWCorememCodeAllocSize = 0;
#if defined(RGX_META_COREMEM_CODE) || defined(RGX_META_COREMEM_DATA)
	IMG_HANDLE 					hFWCorememImportHandle;
	IMG_DEVMEM_SIZE_T			uiFWCorememImportSize;
	DEVMEM_MEMDESC				*psFWCorememHostMemDesc;
#endif

	IMG_DEV_VIRTADDR			sFWCorememDevVAddrBase;
	IMG_HANDLE					hFWCorememPMR;

	RGXFWIF_DEV_VIRTADDR		sFWCorememFWAddr;
	/* FW code memory */
	IMG_DEV_VIRTADDR			sFWCodeDevVAddrBase;
	IMG_HANDLE					hFWCodePMR;
	/* FW data memory */
	IMG_HANDLE					hFWDataPMR;
	RGXFWIF_DEV_VIRTADDR 		sRGXFwInit;

	/* Services initialisation parameters*/
	void				*pvParamState = NULL;
	IMG_UINT32			ui32ParamTemp;
	IMG_BOOL			bEnableSignatureChecks;
	IMG_UINT32			ui32SignatureChecksBufSize;
	IMG_UINT32			ui32DeviceFlags = 0;
	IMG_UINT32			ui32FWConfigFlags = 0;
	IMG_UINT32			ui32HWPerfFilter0, ui32HWPerfFilter1;
	IMG_UINT32			ui32HWPerfHostFilter;
	IMG_UINT32			ui32HWPerfFWBufSize;
	IMG_UINT32			ui32HWRDebugDumpLimit;
	RGX_RD_POWER_ISLAND_CONF	eRGXRDPowerIslandConf;
	RGX_META_T1_CONF    eUseMETAT1;
	RGX_ACTIVEPM_CONF		eRGXActivePMConf;
	IMG_BOOL			bDustRequestInject;

	IMG_UINT32			ui32LogType;
	IMG_UINT32			ui32FilterFlags = 0;
	IMG_UINT32			ui32JonesDisableMask = 0;	/*!< Bitfield to disable Jones ECO fixes */
	IMG_BOOL			bFirmwareLogTypeConfigured;
	IMG_UINT32			ui32LogTypeTemp = 0;
	IMG_BOOL			bAnyLogGroupConfigured = IMG_FALSE;	

	FW_PERF_CONF		eFirmwarePerf;
#if	defined(PVRSRV_GPUVIRT_GUESTDRV)
	/* These variables are n/a in guest drivers */
#else
	/* FW code memory */
	DEVMEM_MEMDESC		*psFWCodeHostMemDesc;
	IMG_HANDLE 			hFWCodeImportHandle;
	IMG_DEVMEM_SIZE_T	uiFWCodeImportSize;
	/* FW data memory */
	DEVMEM_MEMDESC		*psFWDataHostMemDesc;
	IMG_HANDLE 			hFWDataImportHandle;
	IMG_DEVMEM_SIZE_T	uiFWDataImportSize;
	void				*pvFWDataHostAddr;
	void                *pvFWCorememHostAddr;
	struct RGXFW 		*psRGXFW;
	const IMG_BYTE 		*pbRGXFirmware;
	
	IMG_UINT32			ui32FWContextSwitchProfile;	
	IMG_UINT32			ui32TruncateMode = 0;
	IMG_BOOL			bFilteringMode = IMG_FALSE;
	IMG_BOOL			bEnable2Thrds;	
	IMG_BOOL            bHWPerfDisableCustomCounterFilter;
	IMG_BOOL			bEnableHWR;
	IMG_BOOL			bEnableHWPerf;
	IMG_BOOL			bEnableHWPerfHost;
	IMG_UINT32			ui32EnableFWContextSwitch;
	IMG_BOOL			bZeroFreelist;
	IMG_BOOL			bCheckMlist;
#if defined(DEBUG)
	IMG_BOOL			bAssertOnOutOfMem = IMG_FALSE;
#endif
	IMG_BOOL			bEnableCDMKillRand = IMG_FALSE;
	IMG_BOOL			bDisableDMOverlap = IMG_FALSE;
	IMG_BOOL			bDisableClockGating;
	IMG_BOOL			bDisablePDP;
	IMG_BOOL			bEnableRTUBypass;
#if defined(SUPPORT_GPUTRACE_EVENTS)
	IMG_BOOL			bEnableFTrace;
#endif
	IMG_BOOL			bDisableFEDLogging;

	RGX_INIT_LAYER_PARAMS sInitParams;

#if defined(SUPPORT_GPUVIRT_VALIDATION)
	IMG_UINT32			aui32OSidMin[GPUVIRT_VALIDATION_NUM_OS][GPUVIRT_VALIDATION_NUM_REGIONS];
	IMG_UINT32			aui32OSidMax[GPUVIRT_VALIDATION_NUM_OS][GPUVIRT_VALIDATION_NUM_REGIONS];
#endif
#if defined(PDUMP)
	RGXFWIF_HWPERF_CTL *psHWPerfInitData;
	DEVMEM_MEMDESC				*psHWPerfDataMemDesc;
	IMG_HANDLE 				hHWPerfDataImportHandle;
	IMG_DEVMEM_SIZE_T			uiHWPerfDataImportSize;
#endif
#endif
	IMG_HANDLE					hHWPerfDataPMR;

#if	defined(PVRSRV_GPUVIRT_GUESTDRV)
	/* Guest drivers do not perform firmware loading */
	PVR_UNREFERENCED_PARAMETER(pvFWCodeHostAddr);
#else
	psRGXFW = RGXLoadFirmware();
	if (psRGXFW == NULL)
	{
		PVR_DPF((PVR_DBG_ERROR, "RGXInit: Firmware load failed"));
		eError = PVRSRV_ERROR_INIT_FAILURE;
		goto cleanup;
	}
	pbRGXFirmware = RGXFirmwareData(psRGXFW);
#endif

	/* Services initialisation parameters */
	pvParamState = SrvInitParamOpen();

	SrvInitParamGetUINT32(pvParamState, FirmwarePerf, ui32ParamTemp);
	eFirmwarePerf = ui32ParamTemp;

	SrvInitParamGetUINT32(pvParamState, HWRDebugDumpLimit, ui32HWRDebugDumpLimit);
	SrvInitParamGetUINT32(pvParamState, HWPerfFWBufSizeInKB, ui32HWPerfFWBufSize);
	SrvInitParamGetUINT32(pvParamState, SignatureChecksBufSize, ui32SignatureChecksBufSize);
	SrvInitParamGetUINT32(pvParamState, HWPerfFilter0, ui32HWPerfFilter0);
	SrvInitParamGetUINT32(pvParamState, HWPerfFilter1, ui32HWPerfFilter1);
	SrvInitParamGetUINT32(pvParamState, HWPerfHostFilter, ui32HWPerfHostFilter);
	SrvInitParamGetBOOL(pvParamState, EnableSignatureChecks, bEnableSignatureChecks);

	SrvInitParamGetUINT32(pvParamState, EnableAPM, ui32ParamTemp);
	eRGXActivePMConf = ui32ParamTemp;

	SrvInitParamGetUINT32(pvParamState, EnableRDPowerIsland, ui32ParamTemp);
	eRGXRDPowerIslandConf = ui32ParamTemp;

	SrvInitParamGetUINT32(pvParamState, UseMETAT1, ui32ParamTemp);
	eUseMETAT1 = ui32ParamTemp & RGXFWIF_INICFG_METAT1_MASK;

	SrvInitParamGetBOOL(pvParamState, DustRequestInject, bDustRequestInject);

#if	defined(PVRSRV_GPUVIRT_GUESTDRV)
	/* These parameter is n/a in guest drivers */
#else
	SrvInitParamGetBOOL(pvParamState, Enable2ndThread, bEnable2Thrds);
	SrvInitParamGetUINT32(pvParamState, EnableFWContextSwitch, ui32EnableFWContextSwitch);
	SrvInitParamGetUINT32(pvParamState, FWContextSwitchProfile, ui32FWContextSwitchProfile);

	SrvInitParamGetBOOL(pvParamState, EnableHWPerf,	bEnableHWPerf);
	SrvInitParamGetBOOL(pvParamState, EnableHWPerfHost, bEnableHWPerfHost);
	SrvInitParamGetBOOL(pvParamState, HWPerfDisableCustomCounterFilter, bHWPerfDisableCustomCounterFilter);

	SrvInitParamGetBOOL(pvParamState, EnableHWR, bEnableHWR);
	SrvInitParamGetBOOL(pvParamState, CheckMList, bCheckMlist);
	SrvInitParamGetBOOL(pvParamState, ZeroFreelist,	bZeroFreelist);
	SrvInitParamGetBOOL(pvParamState, DisableClockGating, bDisableClockGating);
	SrvInitParamGetBOOL(pvParamState, DisablePDP, bDisablePDP);
#if defined(SUPPORT_GPUTRACE_EVENTS)
	SrvInitParamGetBOOL(pvParamState, EnableFTraceGPU, bEnableFTrace);
#endif
#if defined(HW_ERN_41805) || defined(HW_ERN_42606)
	SrvInitParamGetUINT32(pvParamState, TruncateMode, ui32TruncateMode);
#endif
#if defined(HW_ERN_42290) && defined(RGX_FEATURE_TPU_FILTERING_MODE_CONTROL)
	SrvInitParamGetBOOL(pvParamState, NewFilteringMode, bFilteringMode);
#endif
	SrvInitParamGetBOOL(pvParamState, DisableFEDLogging, bDisableFEDLogging);
	SrvInitParamGetBOOL(pvParamState, EnableRTUBypass, bEnableRTUBypass);

#if defined(DEBUG)
	SrvInitParamGetBOOL(pvParamState, AssertOutOfMemory, bAssertOnOutOfMem);
#endif
	SrvInitParamGetBOOL(pvParamState, EnableCDMKillingRandMode, bEnableCDMKillRand);
	SrvInitParamGetBOOL(pvParamState, DisableDMOverlap, bDisableDMOverlap);
#if defined(RGX_FEATURE_S7_TOP_INFRASTRUCTURE)
	SrvInitParamGetUINT32(pvParamState, JonesDisableMask, ui32JonesDisableMask);
	ui32JonesDisableMask = ui32JonesDisableMask & ~RGX_CR_JONES_FIX_DISABLE_CLRMSK;
#endif

#if defined(SUPPORT_GPUVIRT_VALIDATION)
{
	IMG_UINT	uiCounter, uiRegion;
	void		*pvHintState = NULL;

	PVR_DPF((PVR_DBG_MESSAGE,"\n[GPU Virtualization Validation]: Reading OSid limits\n"));

	PVRSRVCreateAppHintState(IMG_SRVCLIENT, 0, &pvHintState);

	for (uiRegion = 0; uiRegion < GPUVIRT_VALIDATION_NUM_REGIONS; uiRegion++)
	{
		for (uiCounter = 0; uiCounter < GPUVIRT_VALIDATION_NUM_OS; uiCounter++)
		{
			IMG_CHAR	pszHintString[GPUVIRT_VALIDATION_MAX_STRING_LENGTH];
			IMG_UINT32	ui32Default = 0;

			snprintf(pszHintString, GPUVIRT_VALIDATION_MAX_STRING_LENGTH, "OSidRegion%dMin%d", uiRegion, uiCounter );

			PVRSRVGetAppHint(pvHintState, pszHintString, IMG_UINT_TYPE, &ui32Default, &(aui32OSidMin[uiCounter][uiRegion]));

			snprintf(pszHintString, GPUVIRT_VALIDATION_MAX_STRING_LENGTH, "OSidRegion%dMax%d", uiRegion, uiCounter );

			PVRSRVGetAppHint(pvHintState, pszHintString, IMG_UINT_TYPE, &ui32Default, &(aui32OSidMax[uiCounter][uiRegion]));
		}
	}

	for (uiCounter = 0; uiCounter < GPUVIRT_VALIDATION_NUM_OS; uiCounter++)
	{
		for (uiRegion = 0; uiRegion < GPUVIRT_VALIDATION_NUM_REGIONS; uiRegion++)
		{
			PVR_DPF((PVR_DBG_MESSAGE,"\n[GPU Virtualization Validation]: Region:%d, OSid:%d, Min:%u, Max:%u\n",uiRegion, uiCounter, aui32OSidMin[uiCounter][uiRegion], aui32OSidMax[uiCounter][uiRegion]));
		}
	}

	PVRSRVFreeAppHintState(IMG_SRVCLIENT, pvHintState);
}
#endif

	ui32HWRDebugDumpLimit = MIN(ui32HWRDebugDumpLimit, RGXFWIF_HWR_DEBUG_DUMP_ALL);
#endif

	bFirmwareLogTypeConfigured = SrvInitParamGetUINT32List(pvParamState, FirmwareLogType, ui32LogTypeTemp);
	bAnyLogGroupConfigured = SrvInitParamGetUINT32BitField(pvParamState, EnableLogGroup, ui32LogType);

	if (bFirmwareLogTypeConfigured)
	{
		if (ui32LogTypeTemp == 2 /* TRACE */)
		{
			if (!bAnyLogGroupConfigured)
			{
				/* no groups configured - defaulting to MAIN group */
				ui32LogType |= RGXFWIF_LOG_TYPE_GROUP_MAIN;
			}
			ui32LogType |= RGXFWIF_LOG_TYPE_TRACE;
		}
		else if (ui32LogTypeTemp == 1 /* TBI */)
		{
			if (!bAnyLogGroupConfigured)
			{
				/* no groups configured - defaulting to MAIN group */
				ui32LogType |= RGXFWIF_LOG_TYPE_GROUP_MAIN;
			}
			ui32LogType &= ~RGXFWIF_LOG_TYPE_TRACE;
		}
		else if (ui32LogTypeTemp == 0 /* NONE */)
		{
			ui32LogType = RGXFWIF_LOG_TYPE_NONE;
		}
	}
	else
	{
		/* no log type configured - defaulting to TRACE */
		ui32LogType |= RGXFWIF_LOG_TYPE_TRACE;
	}

#if	defined(PVRSRV_GPUVIRT_GUESTDRV)
	/* These functionality is n/a to guest drivers */
#else
	if (bEnable2Thrds)
	{
		eError = PVRSRV_ERROR_NOT_SUPPORTED;
		PVR_DPF((PVR_DBG_ERROR, "RGXInit: Enable2ndThread parameter not supported (%d)", eError));
#if defined(FIX_HW_BRN_40298)
		/* If the second thread needs to be enabled, then we need a proper WA for this BRN */
		PVR_DPF((PVR_DBG_ERROR, "RGXInit: 2nd Thread Enabled but WA not implemented (%d)", eError));
#endif
		goto cleanup;
	}
	ui32FWConfigFlags |= (ui32EnableFWContextSwitch & ~RGXFWIF_INICFG_CTXSWITCH_CLRMSK);
	ui32FWConfigFlags |= ((ui32FWContextSwitchProfile << RGXFWIF_INICFG_CTXSWITCH_PROFILE_SHIFT) & RGXFWIF_INICFG_CTXSWITCH_PROFILE_MASK);
	ui32FWConfigFlags |= bEnableHWPerf? RGXFWIF_INICFG_HWPERF_EN : 0;
	ui32FWConfigFlags |= bHWPerfDisableCustomCounterFilter ? RGXFWIF_INICFG_HWP_DISABLE_FILTER : 0;
	ui32FWConfigFlags |= bCheckMlist? RGXFWIF_INICFG_CHECK_MLIST_EN : 0;
#if !defined(NO_HARDWARE)
	ui32FWConfigFlags |= bEnableHWR? RGXFWIF_INICFG_HWR_EN : 0;
#endif
	ui32FWConfigFlags |= bDisableClockGating? RGXFWIF_INICFG_DISABLE_CLKGATING_EN : 0;

	ui32FWConfigFlags |= (eFirmwarePerf==FW_PERF_CONF_POLLS)? RGXFWIF_INICFG_POLL_COUNTERS_EN : 0;
	ui32FWConfigFlags |= (eFirmwarePerf==FW_PERF_CONF_CUSTOM_TIMER)? RGXFWIF_INICFG_CUSTOM_PERF_TIMER_EN : 0;

	ui32FWConfigFlags |= bDisablePDP? RGXFWIF_SRVCFG_DISABLE_PDP_EN : 0;
	ui32FWConfigFlags |= bEnableCDMKillRand? RGXFWIF_INICFG_CDM_KILL_MODE_RAND_EN : 0;
#if defined(DEBUG)
	ui32FWConfigFlags |= bAssertOnOutOfMem? RGXFWIF_INICFG_ASSERT_ON_OUTOFMEMORY : 0;
#endif
	ui32FWConfigFlags |= bDisableDMOverlap? RGXFWIF_INICFG_DISABLE_DM_OVERLAP : 0;

	ui32DeviceFlags |= bZeroFreelist ? RGXKMIF_DEVICE_STATE_ZERO_FREELIST : 0;
	ui32DeviceFlags |= bDisableFEDLogging ? RGXKMIF_DEVICE_STATE_DISABLE_DW_LOGGING_EN : 0;
	ui32DeviceFlags |= bDustRequestInject? RGXKMIF_DEVICE_STATE_DUST_REQUEST_INJECT_EN : 0;

	ui32DeviceFlags |= bEnableHWPerfHost ? RGXKMIF_DEVICE_STATE_HWPERF_HOST_EN : 0;
	if (bEnableHWPerf && (ui32HWPerfFilter0 == 0 && ui32HWPerfFilter1 == 0))
	{
		ui32HWPerfFilter0 = HW_PERF_FILTER_DEFAULT_ALL_ON;
		ui32HWPerfFilter1 = HW_PERF_FILTER_DEFAULT_ALL_ON;
	}
#if defined(SUPPORT_GPUTRACE_EVENTS)
	if (bEnableFTrace)
	{
		ui32DeviceFlags |= RGXKMIF_DEVICE_STATE_FTRACE_EN;
		/* Since FTrace GPU events depends on HWPerf, ensure it is enabled here */
		ui32FWConfigFlags |= RGXFWIF_INICFG_HWPERF_EN;

		/* In case we have not set EnableHWPerf AppHint just request creation
		 * of certain events we need for the FTrace.
		 */
		if (!bEnableHWPerf)
		{
			ui32HWPerfFilter0 = 0x007BFF00;
			ui32HWPerfFilter1 = 0x0;
		}
		else
		{
			ui32HWPerfFilter0 = HW_PERF_FILTER_DEFAULT_ALL_ON;
			ui32HWPerfFilter1 = HW_PERF_FILTER_DEFAULT_ALL_ON;
		}

	}
#endif

#if defined(RGX_FEATURE_RAY_TRACING)
	ui32FWConfigFlags |= bEnableRTUBypass ? RGXFWIF_INICFG_RTU_BYPASS_EN : 0;
#endif

	ui32FWConfigFlags |= (eUseMETAT1 << RGXFWIF_INICFG_METAT1_SHIFT);

	ui32FilterFlags |= bFilteringMode ? RGXFWIF_FILTCFG_NEW_FILTER_MODE : 0;
	if(ui32TruncateMode == 2)
	{
		ui32FilterFlags |= RGXFWIF_FILTCFG_TRUNCATE_INT;
	}
	else if(ui32TruncateMode == 3)
	{
		ui32FilterFlags |= RGXFWIF_FILTCFG_TRUNCATE_HALF;
	}
#endif /* defined(PVRSRV_GPUVIRT_GUESTDRV) */
	
	/* Require a minimum amount of memory for the signature buffers */
	if (ui32SignatureChecksBufSize < RGXFW_SIG_BUFFER_SIZE_DEFAULT)
	{
		ui32SignatureChecksBufSize = RGXFW_SIG_BUFFER_SIZE_DEFAULT;
	}

#if defined(SUPPORT_KERNEL_SRVINIT)
	rgx_bvnc_packed(&sBVNC.ui32BNC, sBVNC.aszV, sBVNC.ui32VLenMax, RGX_BVNC_KM_B, RGX_BVNC_KM_V_ST, RGX_BVNC_KM_N, RGX_BVNC_KM_C); 
#else
	rgx_bvnc_packed(&sBVNC.ui32BNC, sBVNC.aszV, sBVNC.ui32VLenMax, RGX_BVNC_B, RGX_BVNC_V_ST, RGX_BVNC_N, RGX_BVNC_C); 
#endif


#if defined(SUPPORT_GPUVIRT_VALIDATION)
{
	IMG_UINT    uiOS, uiRegion;
	IMG_UINT32  aui32Buffer[GPUVIRT_VALIDATION_NUM_OS * GPUVIRT_VALIDATION_NUM_REGIONS * 2]; /* The final 2 is 1 for Min and 1 for Max */
	IMG_UINT32  ui32Counter = 0;

	for (uiOS = 0; uiOS < GPUVIRT_VALIDATION_NUM_OS; uiOS++)
	{
		for (uiRegion = 0; uiRegion < GPUVIRT_VALIDATION_NUM_REGIONS; uiRegion++)
		{
			aui32Buffer[ui32Counter++] = aui32OSidMin[uiOS][uiRegion];
			aui32Buffer[ui32Counter++] = aui32OSidMax[uiOS][uiRegion];
		}
	}

	BridgeGPUVIRTPopulateLMASubArenas(hServices, ui32Counter, aui32Buffer);
}
#endif


	/*
	 * FW Memory allocation
	 */
	RGXGetFWImageAllocSize(&uiFWCodeAllocSize,
	                       &uiFWDataAllocSize,
	                       &uiFWCorememCodeAllocSize);

	/* Do firmware memory allocations that back its code and data sections */
	eError = BridgeRGXInitAllocFWImgMem(hServices,
	                                    uiFWCodeAllocSize,
	                                    uiFWDataAllocSize,
	                                    uiFWCorememCodeAllocSize,
	                                    &hFWCodePMR,
	                                    &sFWCodeDevVAddrBase,
	                                    &hFWDataPMR,
	                                    &sFWDataDevVAddrBase,
	                                    &hFWCorememPMR,
	                                    &sFWCorememDevVAddrBase,
	                                    &sFWCorememFWAddr);
	if(eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "RGXInit: PVRSRVRGXInitAllocFWImgMemfailed (%d)", eError));
		goto cleanup;
	}

	/* Initialize firmware configuration */
	eError = BridgeRGXInitFirmware(hServices,
	                               &sRGXFwInit,
	                               bEnableSignatureChecks,
	                               ui32SignatureChecksBufSize,
	                               ui32HWPerfFWBufSize,
	                               (IMG_UINT64)ui32HWPerfFilter0|((IMG_UINT64)ui32HWPerfFilter1<<32),
	                               sizeof(aui32RGXFWAlignChecks),
	                               aui32RGXFWAlignChecks,
	                               ui32FWConfigFlags,
	                               ui32LogType,
	                               ui32FilterFlags,
	                               ui32JonesDisableMask,
	                               ui32HWRDebugDumpLimit,
	                               &sBVNC,
	                               sizeof(RGXFWIF_HWPERF_CTL),
	                               &hHWPerfDataPMR,
	                               eRGXRDPowerIslandConf,
	                               eFirmwarePerf);

	if(eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "RGXInit: PVRSRVRGXInitFirmware failed (%d)", eError));
		goto cleanup;
	}

#if defined(SUPPORT_KERNEL_SRVINIT)
	PMRLock();
#endif

#if	defined(PVRSRV_GPUVIRT_GUESTDRV)
	/* These functionality is n/a to guest drivers */
#else
#if !defined(SUPPORT_TRUSTED_DEVICE)
	DevmemMakeLocalImportHandle(hServices,
	                            hFWCodePMR,
	                            &hFWCodeImportHandle);

	DevmemLocalImport(hServices,
	                  hFWCodeImportHandle,
	                  PVRSRV_MEMALLOCFLAG_CPU_READABLE |
	                  PVRSRV_MEMALLOCFLAG_CPU_WRITEABLE |
	                  PVRSRV_MEMALLOCFLAG_GPU_CACHE_INCOHERENT |
	                  PVRSRV_MEMALLOCFLAG_CPU_WRITE_COMBINE,
	                  &psFWCodeHostMemDesc,
	                  &uiFWCodeImportSize);

	DevmemAcquireCpuVirtAddr(psFWCodeHostMemDesc,
	                         &pvFWCodeHostAddr);
#else
	PVR_UNREFERENCED_PARAMETER(hFWCodeImportHandle);
	PVR_UNREFERENCED_PARAMETER(psFWCodeHostMemDesc);
	PVR_UNREFERENCED_PARAMETER(uiFWCodeImportSize);

	/* We can't get a pointer to a secure FW allocation from within the DDK */
	pvFWCodeHostAddr = NULL;
#endif

	DevmemMakeLocalImportHandle(hServices,
	                            hFWDataPMR,
	                            &hFWDataImportHandle);

	DevmemLocalImport(hServices,
	                  hFWDataImportHandle,
	                  PVRSRV_MEMALLOCFLAG_CPU_READABLE |
	                  PVRSRV_MEMALLOCFLAG_CPU_WRITEABLE |
	                  PVRSRV_MEMALLOCFLAG_GPU_CACHE_INCOHERENT |
	                  PVRSRV_MEMALLOCFLAG_CPU_WRITE_COMBINE,
	                  &psFWDataHostMemDesc,
	                  &uiFWDataImportSize);

	DevmemAcquireCpuVirtAddr(psFWDataHostMemDesc,
	                         &pvFWDataHostAddr);


	/* Map FW coremem buffer to copy the coremem contents on it */
#if defined(RGX_META_COREMEM_CODE) || defined(RGX_META_COREMEM_DATA)
	DevmemMakeLocalImportHandle(hServices,
	                            hFWCorememPMR,
	                            &hFWCorememImportHandle);

	DevmemLocalImport(hServices,
	                  hFWCorememImportHandle,
	                  PVRSRV_MEMALLOCFLAG_CPU_READABLE |
	                  PVRSRV_MEMALLOCFLAG_CPU_WRITEABLE |
	                  PVRSRV_MEMALLOCFLAG_GPU_CACHE_INCOHERENT |
	                  PVRSRV_MEMALLOCFLAG_CPU_WRITE_COMBINE,
	                  &psFWCorememHostMemDesc,
	                  &uiFWCorememImportSize);

	DevmemAcquireCpuVirtAddr(psFWCorememHostMemDesc,
	                         &pvFWCorememHostAddr);
#else
	pvFWCorememHostAddr = NULL;
#endif

#if defined(SUPPORT_KERNEL_SRVINIT)
	PMRUnlock();
#endif

	/*
	 * Process the FW image and setup code and data segments.
	 *
	 * When the trusted device is enabled and the FW image lives
	 * in secure memory we will only setup the data segments here,
	 * while the code segments will be loaded to secure memory
	 * by the trusted device.
	 */
	sInitParams.hServices = hServices;

	eError = RGXProcessFWImage(&sInitParams,
	                           pbRGXFirmware,
	                           pvFWCodeHostAddr,
	                           pvFWDataHostAddr,
	                           pvFWCorememHostAddr,
	                           &sFWCodeDevVAddrBase,
	                           &sFWDataDevVAddrBase,
	                           &sFWCorememDevVAddrBase,
	                           &sFWCorememFWAddr,
	                           &sRGXFwInit,
	                           eUseMETAT1 == RGX_META_T1_OFF ? 1 : 2,
	                           eUseMETAT1 == RGX_META_T1_MAIN ? 1 : 0);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "RGXInit: RGXProcessFWImage failed (%d)", eError));
		goto cleanup;
	}

#if defined(SUPPORT_TRUSTED_DEVICE) && !defined(NO_HARDWARE)
	RGXTDProcessFWImage(hServices, psRGXFW);
#endif

	/*
	 * FW image processing complete, perform final steps
	 * (if any) on the kernel before pdumping the FW code
	 */
	eError = BridgeRGXInitFinaliseFWImage(hServices,
	                                      hFWCodePMR,
	                                      uiFWCodeAllocSize);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "RGXInit: RGXInitFinaliseFWImage failed (%d)", eError));
		goto cleanup;
	}

#if defined(SUPPORT_KERNEL_SRVINIT)
	PMRLock();
#endif

#if !defined(SUPPORT_TRUSTED_DEVICE)
	/* dump the fw code */
	SRVINITPDumpComment(hServices, "Dump firmware code image");
	DevmemPDumpLoadMem(psFWCodeHostMemDesc,
							 0,
							 uiFWCodeAllocSize,
							 PDUMP_FLAGS_CONTINUOUS);
#endif
	SRVINITPDumpComment(hServices, "Dump firmware data image");
	DevmemPDumpLoadMem(psFWDataHostMemDesc,
							 0,
							 uiFWDataAllocSize,
							 PDUMP_FLAGS_CONTINUOUS);


#if defined(RGX_META_COREMEM_CODE) && !defined(SUPPORT_TRUSTED_DEVICE)
	SRVINITPDumpComment(hServices, "Dump firmware coremem image");
	DevmemPDumpLoadMem(psFWCorememHostMemDesc,
							 0,
							 RGX_META_COREMEM_CODE_SIZE,
							 PDUMP_FLAGS_CONTINUOUS);

	/* clean the mappings that are not required anymore */
	DevmemReleaseCpuVirtAddr(psFWCorememHostMemDesc);
	DevmemFree(psFWCorememHostMemDesc);
	DevmemUnmakeLocalImportHandle(hServices,
	                              hFWCorememImportHandle);
#endif
	
	/* clean the mappings that are not required anymore */
	DevmemReleaseCpuVirtAddr(psFWDataHostMemDesc);
	DevmemFree(psFWDataHostMemDesc);
	DevmemUnmakeLocalImportHandle(hServices,
	                              hFWDataImportHandle);

#if !defined(SUPPORT_TRUSTED_DEVICE)
	DevmemReleaseCpuVirtAddr(psFWCodeHostMemDesc);
	DevmemFree(psFWCodeHostMemDesc);
	DevmemUnmakeLocalImportHandle(hServices,
	                              hFWCodeImportHandle);
#endif

#if defined(SUPPORT_KERNEL_SRVINIT)
	PMRUnlock();
#endif

	/*
	 * Build Debug info script
	 */
	sDbgInitScript.psCommands = asDbgCommands;

	if(!PrepareDebugScript(&sDbgInitScript, eFirmwarePerf != FW_PERF_CONF_NONE))
	{
		PVR_DPF((PVR_DBG_ERROR, "RGXInit: Run out of mem for the dbg commands"));
	}


	/* finish the script */
	if(!ScriptHalt(&sDbgInitScript))
	{
		PVR_DPF((PVR_DBG_ERROR, "RGXInit: Run out of mem for the terminating dbg script"));
	}

	/*
	 * Build Debugbus script
	 */
	sDbgBusInitScript.psCommands = asDbgBusCommands;

	if(!PrepareDebugBusScript(&sDbgBusInitScript))
	{
		PVR_DPF((PVR_DBG_ERROR, "RGXInit: Run out of mem for the dbg commands"));
	}

	/* finish the script */
	if(!ScriptHalt(&sDbgBusInitScript))
	{
		PVR_DPF((PVR_DBG_ERROR, "RGXInit: Run out of mem for the terminating dbg script"));
	}

	/*
	 * Build deinit script
	 */
	sDeinitScript.psCommands = asDeinitCommands;

	if(!PrepareDeinitScript(&sDeinitScript))
	{
		PVR_DPF((PVR_DBG_ERROR, "RGXInit: Run out of mem for the deinit commands"));
	}

	/* finish the script */
	if(!ScriptHalt(&sDeinitScript))
	{
		PVR_DPF((PVR_DBG_ERROR, "RGXInit: Run out of mem for the terminating deinit script"));
	}
#if defined(PDUMP)
#if defined(SUPPORT_KERNEL_SRVINIT)
	PMRLock();
#endif
	/* Acquire HWPerf data mem handle to be able to fill it later */
	DevmemMakeLocalImportHandle(hServices,
	                            hHWPerfDataPMR,
	                            &hHWPerfDataImportHandle);

	DevmemLocalImport(hServices,
	                        hHWPerfDataImportHandle,
	                        PVRSRV_MEMALLOCFLAG_CPU_READABLE |
	                        PVRSRV_MEMALLOCFLAG_CPU_WRITEABLE |
	                        PVRSRV_MEMALLOCFLAG_GPU_CACHE_INCOHERENT |
	                        PVRSRV_MEMALLOCFLAG_CPU_WRITE_COMBINE,
	                        &psHWPerfDataMemDesc,
	                        &uiHWPerfDataImportSize);
	eError = DevmemAcquireCpuVirtAddr(psHWPerfDataMemDesc, (void **)&psHWPerfInitData);
	if (eError != PVRSRV_OK)
	{
		PVR_LOGG_IF_ERROR(eError, "DevmemAcquireCpuVirtAddr", failHWPerfCountersMemDescAqCpuVirt);
	}

	InitialiseHWPerfCounters(hServices, psHWPerfDataMemDesc, psHWPerfInitData);
	InitialiseCustomCounters(hServices, psHWPerfDataMemDesc);

#if defined(SUPPORT_KERNEL_SRVINIT)
	PMRUnlock();
#endif
#endif
#endif /* defined(PVRSRV_GPUVIRT_GUESTDRV) */

	/*
	 * Perform second stage of RGX initialisation
	 */
	eError = BridgeRGXInitDevPart2(hServices,
	                               sDbgInitScript.psCommands,
	                               sDbgBusInitScript.psCommands,
	                               sDeinitScript.psCommands,
	                               ui32DeviceFlags,
	                               ui32HWPerfHostFilter,
	                               eRGXActivePMConf,
	                               hFWCodePMR,
	                               hFWDataPMR,
	                               hFWCorememPMR,
	                               hHWPerfDataPMR);

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "RGXInit: BridgeRGXInitDevPart2 failed (%d)", eError));
		goto cleanup;
	}

#if	defined(PVRSRV_GPUVIRT_GUESTDRV)
	/* These functionality is n/a to guest drivers */
#else
#if !defined(SUPPORT_KERNEL_SRVINIT) && defined(PDUMP)
	/*
	 * Dump the list of signature registers
	 */
	{
		IMG_UINT32 i;

		SRVINITPDumpComment(hServices, "Signature TA registers: ");
		for (i = 0; i < sizeof(asTASigRegList)/sizeof(RGXFW_REGISTER_LIST); i++)
		{
			if (asTASigRegList[i].ui16IndirectRegNum != 0)
			{
				SRVINITPDumpComment(hServices, " * 0x%8.8X (indirect via 0x%8.8X %d to %d)",
				              asTASigRegList[i].ui16RegNum, asTASigRegList[i].ui16IndirectRegNum,
				              asTASigRegList[i].ui16IndirectStartVal, asTASigRegList[i].ui16IndirectEndVal);
			}
			else
			{
				SRVINITPDumpComment(hServices, " * 0x%8.8X", asTASigRegList[i].ui16RegNum);
			}
		}

		SRVINITPDumpComment(hServices, "Signature 3D registers: ");
		for (i = 0; i < sizeof(as3DSigRegList)/sizeof(RGXFW_REGISTER_LIST); i++)
		{
			if (as3DSigRegList[i].ui16IndirectRegNum != 0)
			{
				SRVINITPDumpComment(hServices, " * 0x%8.8X (indirect via 0x%8.8X %d to %d)",
				              as3DSigRegList[i].ui16RegNum, as3DSigRegList[i].ui16IndirectRegNum,
				              as3DSigRegList[i].ui16IndirectStartVal, as3DSigRegList[i].ui16IndirectEndVal);
			}
			else
			{
				SRVINITPDumpComment(hServices, " * 0x%8.8X", as3DSigRegList[i].ui16RegNum);
			}
		}

#if defined(RGX_FEATURE_RAY_TRACING)
		SRVINITPDumpComment(hServices, "Signature RTU registers: ");
		for (i = 0; i < sizeof(asRTUSigRegList)/sizeof(RGXFW_REGISTER_LIST); i++)
		{
			if (asRTUSigRegList[i].ui16IndirectRegNum != 0)
			{
				SRVINITPDumpComment(hServices, " * 0x%8.8X (indirect via 0x%8.8X %d to %d)",
				              asRTUSigRegList[i].ui16RegNum, asRTUSigRegList[i].ui16IndirectRegNum,
				              asRTUSigRegList[i].ui16IndirectStartVal, asRTUSigRegList[i].ui16IndirectEndVal);
			}
			else
			{
				SRVINITPDumpComment(hServices, " * 0x%8.8X", asRTUSigRegList[i].ui16RegNum);
			}
		}

		SRVINITPDumpComment(hServices, "Signature SHG registers: ");
		for (i = 0; i < sizeof(asSHGSigRegList)/sizeof(RGXFW_REGISTER_LIST); i++)
		{
			if (asSHGSigRegList[i].ui16IndirectRegNum != 0)
			{
				SRVINITPDumpComment(hServices, " * 0x%8.8X (indirect via 0x%8.8X %d to %d)",
				              asSHGSigRegList[i].ui16RegNum, asSHGSigRegList[i].ui16IndirectRegNum,
				              asSHGSigRegList[i].ui16IndirectStartVal, asSHGSigRegList[i].ui16IndirectEndVal);
			}
			else
			{
				SRVINITPDumpComment(hServices, " * 0x%8.8X", asSHGSigRegList[i].ui16RegNum);
			}
		}
#endif
	}
#endif	/* !defined(SUPPORT_KERNEL_SRVINIT) && defined(PDUMP) */
#endif /*   defined(PVRSRV_GPUVIRT_GUESTDRV) */

	eError = PVRSRV_OK;
#if defined(PDUMP)
#if defined(SUPPORT_KERNEL_SRVINIT)
	PMRLock();
#endif

	DevmemReleaseCpuVirtAddr(psHWPerfDataMemDesc);
	DevmemFree(psHWPerfDataMemDesc);
	DevmemUnmakeLocalImportHandle(hServices,
	                              hHWPerfDataImportHandle);
#if defined(SUPPORT_KERNEL_SRVINIT)
	PMRUnlock();
#endif
failHWPerfCountersMemDescAqCpuVirt:
#endif
cleanup:
	SrvInitParamClose(pvParamState);

#if	defined(PVRSRV_GPUVIRT_GUESTDRV)
	/* Guest driver do not unload firmware */
#else
	if (psRGXFW != NULL)
	{
		RGXUnloadFirmware(psRGXFW);
	}
#endif
	return eError;
}

/******************************************************************************
 End of file (rgxsrvinit.c)
******************************************************************************/
