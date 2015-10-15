/*************************************************************************/ /*!
@File
@Title          RGX firmware interface structures
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    RGX firmware interface structures used by srvclient
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
#if !defined (__RGX_FWIF_CLIENT_H__)
#define __RGX_FWIF_CLIENT_H__

#include "img_types.h"
#include "rgxdefs_km.h"

#include "rgx_fwif_alignchecks.h"
#include "rgx_fwif_shared.h"
#include "rgx_fwif_defines.h"


/* Indicates the number of RTDATAs per RTDATASET */
#define RGXMKIF_NUM_RTDATAS			2


#define RGXMKIF_RENDERFLAGS_NORENDER			0x00000001UL	/*!< Only run the TA phase on the hardware, on terminate
																	 free the parameters and dummy process 3D phase */
#define RGXMKIF_RENDERFLAGS_ABORT				0x00000002UL	/*!< The scene has been aborted free the parameters and dummy
																	 process to completion */
#define RGXMKIF_RENDERFLAGS_RENDERMIDSCENE		0x00000004UL	/*!< Indicates this is a midscene render and the render resources
																	 should not be released */
#define RGXMKIF_RENDERFLAGS_NODEALLOC			0x00000008UL	/*!< Do not free the parameter memory when rendering. Typically	
																	 used when doing z-only renders */
#define RGXMKIF_RENDERFLAGS_ZONLYRENDER			0x00000010UL	/*!< This render is only being used to generate z buffer data
																	 so some registers need to be tweaked (e.g. TE_PSG, ISP_ZLSCTL).
																     Setting this flag will cause the TA to terminate the control streams,
																	 but keep the TPC entries. This is so more primitives can be sent if needed and
																	 added to the control streams ready for any more renders of this scene. */
#define RGXMKIF_RENDERFLAGS_GETVISRESULTS		0x00000020UL	/*!< This render has visibility result associated with it.
																	 Setting this flag will cause the firmware to collect the visibility results */
#define RGXMKIF_RENDERFLAGS_ACCUMVISRESULTS		0x00000040UL	/*!< Setting this flag will cause the firmware to accumulate (sum)
																	 the visibility results */
#define RGXMKIF_RENDERFLAGS_DEPTHBUFFER			0x00000080UL	/*!< Indicates whether a depth buffer is present */
#define RGXMKIF_RENDERFLAGS_STENCILBUFFER		0x00000100UL	/*!< Indicates whether a stencil buffer is present */ 
#define RGXMKIF_RENDERFLAGS_TA_DEPENDENCY		0x00000200UL	/*!< Update 3D dependancy sync object after the render has completed */
#define RGXMKIF_RENDERFLAGS_RENDERSCENE			0x00000400UL	/*!< Update HWRTDataset completed ops after the render has completed */
#define	RGXMKIF_RENDERFLAGS_FLUSH_SLC			0x00000800UL	/*!< Flush the system cache after the render has completed */
#define RGXMKIF_RENDERFLAGS_BBOX_RENDER			0x00001000UL	/*!< This is a bounding box render */
#define RGXMKIF_RENDERFLAGS_SECURE				0x00002000UL	/*!< This render needs DRM Security */
#define RGXMKIF_RENDERFLAGS_ABORT_NOFREE		0x00004000UL	/*!< This flags goes in hand with ABORT and excplicity ensure no mem free is issued in case of first TA kick */
#define RGXMKIF_RENDERFLAGS_DISABLE_PIXELMERGE	0x00008000UL	/*!< Force disabling of pixel merging */

/*
	The host must indicate if this is the first and/or last command to
	be issued for the specified task
*/
#define RGXMKIF_TAFLAGS_FIRSTKICK				0x00000001UL
#define RGXMKIF_TAFLAGS_LASTKICK				0x00000002UL
/*
	When a mid-scene render has been kicked this flag tells the pcore code
	to wait for it to complete before kicking TA
*/
#define RGXMKIF_TAFLAGS_RESUMEMIDSCENE			0x00000004UL

/*
	Tells the primary core that we are sending another ta command after a
	mid-scene render which freed the associated parameters so a ta context
	reset is required.
*/
#define RGXMKIF_TAFLAGS_RESUMEDAFTERFREE		0x00000008UL

/*
	Invalidate the USE cache before this TA kick.
*/
#define RGXMKIF_TAFLAGS_INVALIDATEUSECACHE		0x00000010UL

/*
	Invalidate the PDS cache before this TA kick.
*/
#define RGXMKIF_TAFLAGS_INVALIDATEPDSCACHE		0x00000020UL

/*
	Flush the data cache before this ta kick.
*/
#define RGXMKIF_TAFLAGS_INVALIDATEDATACACHE		0x00000040UL

/*
	Indicates that complex geometry is used during the TA stream
*/
#define RGXMKIF_TAFLAGS_CMPLX_GEOMETRY_PRESENT	0x00000080UL

/*
 * 	Indicates the particular TA needs to be aborted
 */
#define RGXMKIF_TAFLAGS_TA_ABORT			0x00000100UL
/*
	Flags to check 3D dependency sync object
*/
#define RGXMKIF_TAFLAGS_DEPENDENT_TA			0x00000400UL

#if defined(RGX_FEATURE_SPM_CONTEXT_SWITCH)
#define RGXMKIF_TAFLAGS_TA_SPM_DRAINED			0x00001000UL
#endif

/* Flags to indicate this cmd has an update to the PB */
#define RGXMKIF_TAFLAGS_PB_THRESHOLD_UPDATE		0x00010000UL
#if !defined(DISABLE_RGX_PB_GROW_SHRINK) && defined(SUPPORT_PERCONTEXT_FREELIST)
#define RGXMKIF_TAFLAGS_PB_GROW					0x00020000UL
#define RGXMKIF_TAFLAGS_PB_SHRINK				0x00040000UL
#define RGXMKIF_TAFLAGS_PB_UPDATE_MASK			(RGXMKIF_TAFLAGS_PB_GROW | \
												 RGXMKIF_TAFLAGS_PB_SHRINK | \
												 RGXMKIF_TAFLAGS_PB_THRESHOLD_UPDATE)
#else
#define RGXMKIF_TAFLAGS_PB_UPDATE_MASK			(RGXMKIF_TAFLAGS_PB_THRESHOLD_UPDATE)
#endif

#define RGXMKIF_TAFLAGS_SECURE					0x00080000UL

#if defined(FIX_HW_BRN_37918) || (defined(FIX_HW_BRN_38059) && !defined(HW_ERN_36400))
#define FIX_HW_VCE_PAUSE
/*
	Indicates that the VCE needs to be halted and restarted as per the WA
*/
#define RGXMKIF_TAFLAGS_APPLY_VCE_PAUSE			0x00100000UL
#endif

/* flags for transfer queue commands */
#define RGXMKIF_CMDTRANSFER_FLAG_SECURE			0x00000001UL

/* flags for 2D commands */
#define RGXMKIF_CMD2D_FLAG_SECURE				0x00000001
#define RGXMKIF_CMD2D_FLAG_SRC_FBCDC			0x00000002
#define RGXMKIF_CMD2D_FLAG_DST_FBCDC			0x00000004

/* flags for compute commands */
#define RGXMKIF_COMPUTE_FLAG_SECURE				0x00000001

/* flags for ray commands */
#define RGXFWIF_RAY_FLAG_SECURE					0x00000001
#define RGXFWIF_RAY_NULL_CMDRTU					0x00000001
/*
	The host must indicate if this is the first and/or last command to
	be issued for the specified task
*/
#define RGXMKIF_SHGFLAGS_FIRSTKICK				0x00000001UL
#define RGXMKIF_SHGFLAGS_LASTKICK				0x00000002UL

/*****************************************************************************
 Parameter/HWRTData control structures.
*****************************************************************************/

/*!
	Configuration registers which need to be loaded by the firmware before a TA
	kick can be started.
*/
typedef struct _RGXFWIF_TAREGISTERS_
{
	IMG_UINT64	uTAReg_PM_MTILE_ARRAY;
	IMG_UINT64	uTAReg_PPP_CTRL;
	IMG_UINT64	uTAReg_PPP_WCLAMP;
	IMG_UINT64	uTAReg_TE_PSG;
	IMG_UINT64	uTAReg_TE_PSGREGION_ADDR;
	IMG_UINT64	uTAReg_VDM_CTRL_STREAM_BASE;
	IMG_UINT64	uTAReg_VDM_CTRL_STREAM_TERMINATE;
	IMG_UINT64	uTAReg_VDM_SYNC_PDS_DATA_BASE;
	IMG_UINT64	uTAReg_TA_RTC_ADDR_BASE;
	IMG_UINT64	uTAReg_TPU_BORDER_COLOUR_TABLE;
	IMG_UINT64	uTAReg_TPU_YUV_CSC_COEFFICIENTS;
#if defined(RGX_FEATURE_TPU_CEM_DATAMASTER_GLOBAL_REGISTERS)
	IMG_UINT64  uTAReg_TPU_CEM_VDM;
#else
    IMG_UINT64  uTAReg_TPU;
#endif
	IMG_UINT64  uTAReg_TPU_ARRAYS_VDM;

#if defined(RGX_FEATURE_TESSELLATION)
	IMG_UINT64	uTAReg_PDS_COEFF_FREE_PROG;
#endif
	
#if defined(RGX_FEATURE_VDM_DRAWINDIRECT)
	IMG_UINT64	uTAReg_VDM_DRAW_INDIRECT0;
	IMG_UINT32	uTAReg_VDM_DRAW_INDIRECT1;
#endif

// 	IMG_UINT32	ui32USELDRedirect;/*!< USE LD redirect register value */
// 	IMG_UINT32	ui32USESTRange;/*!< USE ST Range control register value */
} RGXFWIF_TAREGISTERS, *PRGXFWIF_TAREGISTERS;

typedef struct _RGXFWIF_TAREGISTERS_CSWITCH_
{
	IMG_UINT64	uTAReg_VDM_CONTEXT_STATE_BASE_ADDR;
#if defined(FIX_HW_VCE_PAUSE)
	IMG_UINT64	uTAReg_VDM_CONTEXT_STATE_RESUME_ADDR;
#endif
	IMG_UINT64	uTAReg_TA_CONTEXT_STATE_BASE_ADDR;

	struct
	{
		IMG_UINT64	uTAReg_VDM_CONTEXT_STORE_TASK0;
		IMG_UINT64	uTAReg_VDM_CONTEXT_STORE_TASK1;
		IMG_UINT64	uTAReg_VDM_CONTEXT_STORE_TASK2;
		
		/* VDM resume state update controls */
		IMG_UINT64	uTAReg_VDM_CONTEXT_RESUME_TASK0;
		IMG_UINT64	uTAReg_VDM_CONTEXT_RESUME_TASK1;
		IMG_UINT64	uTAReg_VDM_CONTEXT_RESUME_TASK2;
		
	#if defined(RGX_FEATURE_VDM_OBJECT_LEVEL_LLS)
		IMG_UINT64	uTAReg_VDM_CONTEXT_STORE_TASK3;
		IMG_UINT64	uTAReg_VDM_CONTEXT_STORE_TASK4;

		IMG_UINT64	uTAReg_VDM_CONTEXT_RESUME_TASK3;
		IMG_UINT64	uTAReg_VDM_CONTEXT_RESUME_TASK4;
	#endif
	} asTAState[2];

} RGXFWIF_TAREGISTERS_CSWITCH, *PRGXFWIF_TAREGISTERS_CSWITCH;

/*!
	Configuration registers which need to be loaded by the firmware before the
	dummy region header is issued.
*/
typedef struct _RGXFWIF_BRN44455_TAREGISTERS_
{
	IMG_UINT64	uTAReg_TE_PSGREGION_ADDR;
} RGXFWIF_BRN44455_TAREGISTERS, *PRGXFWIF_BRN44455_TAREGISTERS;

/*!
	Configuration registers which need to be loaded by the firmware before any
	VDM kick is issued on cores affected by this BRN.
*/
typedef struct _RGXFWIF_BRN52563_TAREGISTERS_
{
	IMG_UINT64	uTAReg_VDM_CONTEXT_SYNC_BASE_ADDR;			/*!< Base address for the VDM PDS context store sync */
	IMG_UINT64	uTAReg_DDM_CONTEXT_SYNC_BASE_ADDR;			/*!< Base address for the DDM PDS context store sync */
	IMG_UINT64	uTAReg_DDM_CONTEXT_STORE_SYNC_BASE_ADDR;	/*!< Base address for the DDM USC context store sync */
	IMG_UINT32	ui32BufferSize;
} RGXFWIF_BRN52563_TAREGISTERS, *PRGXFWIF_BRN52563_TAREGISTERS;

/*!
	TA command. The RGX TA can be used to tile a whole scene's objects
	as per TA behaviour on RGX.
*/
typedef struct _RGXFWIF_CMDTA_
{
	RGXFWIF_TAREGISTERS			RGXFW_ALIGN sTARegs;/*!< TA registers */
	RGXFWIF_TAREGISTERS_CSWITCH	RGXFW_ALIGN sTARegsCSwitch;/*!< TA registers for ctx switch */
	IMG_UINT32					ui32Flags; /*!< command control flags */
	IMG_UINT32					ui32FrameNum;/*!< associated frame number */
	RGXFWIF_DEV_VIRTADDR		sHWRTData; /* RTData associated with this command, this is used for context selection and for storing out HW-context, when TA is switched out for continuing later */
	RGXFWIF_DEV_VIRTADDR		sZBuffer;	/* Z-Buffer */
	RGXFWIF_DEV_VIRTADDR		sSBuffer;	/* S-Buffer */
	RGXFWIF_UFO					sPartialRenderTA3DFence; /* Holds the TA/3D fence value to allow the 3D partial render command to go through */
	RGXFWIF_UFO					sPartialRenderHardwareSyncFence; /* Holds the HardwareSync fence value to allow the 3D partial render command to go through */
	IMG_UINT32					ui32MaxDeadlineMS;/*!< Max HWR deadline limit in ms */
#if defined(FIX_HW_BRN_44455)
	RGXFWIF_BRN44455_TAREGISTERS	RGXFW_ALIGN sBRN44455TARegs;/*!< TA registers for BRN44455 workaround */
#endif
#if defined(FIX_HW_BRN_52563)
	RGXFWIF_BRN52563_TAREGISTERS	RGXFW_ALIGN	sBRN52563TARegs;/*!< TA registers for BRN52563 workaround */
#endif

} RGXFWIF_CMDTA,*PRGXFWIF_CMDTA;


/*!
	Configuration registers common to the RTU and 3D REFS
 */
typedef struct _RGXFWIF_COMMON_RTU_REGISTERS_
{
	IMG_UINT64 uDPXReg_RQ_RAY_SETUP;
	IMG_UINT64 uDPXReg_RS_CTRL0;
	IMG_UINT64 uDPXReg_RS_CTRL1;
	IMG_UINT64 uDPXReg_RS_CTRL4;
	IMG_UINT64 uDPXReg_RS_CTRL16;

	IMG_UINT64 uDPXReg_RS_CTRL_DUMP_ADDR;
	IMG_UINT64 uDPXReg_RS_CTRL_DUMP_SIZE;

	IMG_UINT64 uDPXReg_BF_CTRL;

	IMG_UINT64 uDPXReg_PRIM_TABLE_SIZE;
	IMG_UINT64 uDPXReg_BIF_PRIMITIVE_BASE_ADDRESS;
	IMG_UINT64 uDPXReg_FBA_MAIN_CTRL;
	IMG_UINT64 uDPXReg_FILTER_CTRL;
	IMG_UINT64 uDPXReg_FILTER_CTRL1;
	IMG_UINT64 uDPXReg_FBA_AP_BASE[DPX_MAX_FBA_AP];
	IMG_UINT64 uDPXReg_FBA_AP_CTRL[DPX_MAX_FBA_AP];
	IMG_UINT64 uDPXReg_FBA_AP_MAP;
	IMG_UINT64 uDPXReg_BIF_ROOTID_BASE;
	IMG_UINT32 auDPXReg_FBA_FILTER_RAM[DPX_MAX_FBA_FILTER_WIDTH * DPX_MAX_FBA_FILTER_WIDTH];
} RGXFWIF_COMMON_RTU_REGISTERS, * PRGXFWIF_COMMON_RTU_REGISTERS;


typedef struct _RGXFWIF_COMMON_RTU_SETUP_
{
	IMG_UINT32            ui32FBAFilterTableSize; /*!< Controls number of valid entries in uDPXReg_FBA_FILTER_RAM */
	IMG_BOOL			  bInvalidateRootBIDCache; /*!< Decide whether to force the RQ to stop sending rays to the RU and invalidate any ROOTBID lookup entries */
} RGXFWIF_COMMON_RTU_SETUP, * PRGXFWIF_COMMON_RTU_SETUP;


/*!
	Configuration registers which need to be loaded by the firmware before ISP
	can be started.
*/
typedef struct _RGXFWIF_3DREGISTERS_
{
	IMG_UINT64	u3DReg_USC_PIXEL_OUTPUT_CTRL;
	IMG_UINT64  u3DReg_USC_CLEAR_REGISTER0;
	IMG_UINT64  u3DReg_USC_CLEAR_REGISTER1;
	IMG_UINT64  u3DReg_USC_CLEAR_REGISTER2;
	IMG_UINT64  u3DReg_USC_CLEAR_REGISTER3;
	IMG_UINT64  u3DReg_USC_CLEAR_REGISTER4;
	IMG_UINT64  u3DReg_USC_CLEAR_REGISTER5;
	IMG_UINT64  u3DReg_USC_CLEAR_REGISTER6;
	IMG_UINT64  u3DReg_USC_CLEAR_REGISTER7;
	IMG_UINT64	u3DReg_ISP_MTILE_BASE;
	IMG_UINT64	u3DReg_ISP_MTILE_SIZE;
	IMG_UINT64	u3DReg_ISP_GRIDOFFSET;
	IMG_UINT64	u3DReg_ISP_MULTISAMPLECTL;
	IMG_UINT64	u3DReg_ISP_SCISSOR_BASE;
	IMG_UINT64	u3DReg_ISP_DBIAS_BASE;
	IMG_UINT64	u3DReg_ISP_BGOBJDEPTH;
	IMG_UINT64	u3DReg_ISP_BGOBJVALS;
    IMG_UINT64  u3DReg_ISP_AA;
    IMG_UINT64  u3DReg_ISP_OCLQRY_BASE;
    IMG_UINT64  u3DReg_ISP_ZLSCTL;
    IMG_UINT64  u3DReg_ISP_MASK_LOAD_BASE;
    IMG_UINT64  u3DReg_ISP_MASK_STORE_BASE;
    IMG_UINT64  u3DReg_ISP_ZLOAD_BASE;
    IMG_UINT64  u3DReg_ISP_ZSTORE_BASE;
    IMG_UINT64  u3DReg_ISP_STENCIL_LOAD_BASE;
    IMG_UINT64  u3DReg_ISP_STENCIL_STORE_BASE;
	IMG_UINT64  u3DReg_ISP_ZLS_PIXELS;
    IMG_UINT64	u3DReg_ISP_RGN;
    IMG_UINT64  u3DReg_ISP_CTL;
#if defined(RGX_FEATURE_S7_TOP_INFRASTRUCTURE)
    IMG_UINT64  u3DReg_ISP_XTP_PIPE_ENABLE;
#endif
    IMG_UINT64  u3DReg_FB_CDC_ZLS;
	IMG_UINT64	u3DReg_PM_3D_FSTACK_BASE;
	IMG_UINT64	u3DReg_PM_3D_FSTACK;
	IMG_UINT64	u3DReg_EVENT_PIXEL_PDS_CODE;
	IMG_UINT64	u3DReg_EVENT_PIXEL_PDS_DATA;
	IMG_UINT64	u3DReg_EVENT_PIXEL_PDS_INFO;
	IMG_UINT64	u3DReg_PBE_WORD0_MRT0;
	IMG_UINT64	u3DReg_PBE_WORD1_MRT0;
	IMG_UINT64	u3DReg_PBE_WORD2_MRT0;
	IMG_UINT64	u3DReg_PBE_WORD0_MRT1;
	IMG_UINT64	u3DReg_PBE_WORD1_MRT1;
	IMG_UINT64	u3DReg_PBE_WORD2_MRT1;
	IMG_UINT64	u3DReg_PBE_WORD0_MRT2;
	IMG_UINT64	u3DReg_PBE_WORD1_MRT2;
	IMG_UINT64	u3DReg_PBE_WORD2_MRT2;
	IMG_UINT64	u3DReg_PBE_WORD0_MRT3;
	IMG_UINT64	u3DReg_PBE_WORD1_MRT3;
	IMG_UINT64	u3DReg_PBE_WORD2_MRT3;
	IMG_UINT64	u3DReg_PBE_WORD0_MRT4;
	IMG_UINT64	u3DReg_PBE_WORD1_MRT4;
	IMG_UINT64	u3DReg_PBE_WORD2_MRT4;
	IMG_UINT64	u3DReg_PBE_WORD0_MRT5;
	IMG_UINT64	u3DReg_PBE_WORD1_MRT5;
	IMG_UINT64	u3DReg_PBE_WORD2_MRT5;
	IMG_UINT64	u3DReg_PBE_WORD0_MRT6;
	IMG_UINT64	u3DReg_PBE_WORD1_MRT6;
	IMG_UINT64	u3DReg_PBE_WORD2_MRT6;
	IMG_UINT64	u3DReg_PBE_WORD0_MRT7;
	IMG_UINT64	u3DReg_PBE_WORD1_MRT7;
	IMG_UINT64	u3DReg_PBE_WORD2_MRT7;
	IMG_UINT64  u3DReg_PBE;
	IMG_UINT64  u3DReg_TPU_BORDER_COLOUR_TABLE;
	IMG_UINT64	u3DReg_TPU_YUV_CSC_COEFFICIENTS;
	IMG_UINT64  u3DReg_TPU;
	IMG_UINT64  u3DReg_TPU_ARRAYS;
	IMG_UINT64  u3DReg_TPUMADDCtrl;
	IMG_UINT64	u3DReg_PDS_BGND0_BASE;
	IMG_UINT64	u3DReg_PDS_BGND1_BASE;
	IMG_UINT64	u3DReg_PDS_BGND2_BASE;
	IMG_UINT64	u3DReg_PDS_BGND3_SIZEINFO;
	IMG_UINT64	u3DReg_PDS_PR_BGND0_BASE;
	IMG_UINT64	u3DReg_PDS_PR_BGND1_BASE;
	IMG_UINT64	u3DReg_PDS_PR_BGND2_BASE;
	IMG_UINT64	u3DReg_PDS_PR_BGND3_SIZEINFO;
	IMG_UINT64	u3DReg_RGX_CR_BLACKPEARL_FIX;

} RGXFWIF_3DREGISTERS, *PRGXFWIF_3DREGISTERS;

typedef struct _RGXFWIF_CMD3D_
{
	RGXFWIF_3DREGISTERS		RGXFW_ALIGN s3DRegs;		/*!< 3D registers */
	IMG_UINT32				ui32Flags; /*!< command control flags */
	IMG_UINT32				ui32FrameNum;/*!< associated frame number */
	RGXFWIF_DEV_VIRTADDR	sHWRTData; /* RTData associated with this command, this is used for context selection and for storing out HW-context, when TA is switched out for continuing later */
	RGXFWIF_DEV_VIRTADDR	sZBuffer;	/* Z-Buffer */
	RGXFWIF_DEV_VIRTADDR	sSBuffer;	/* S-Buffer */
	IMG_UINT32				ui32ZLSStride; /* Stride IN BYTES for Z-Buffer in case of RTAs */
	IMG_UINT32				ui32SLSStride; /* Stride IN BYTES for S-Buffer in case of RTAs */
	IMG_UINT32				ui32MaxDeadlineMS;/*!< Max HWR deadline limit in ms */
} RGXFWIF_CMD3D,*PRGXFWIF_CMD3D;

typedef struct _RGXFWIF_TRANSFERREGISTERS_
{
	IMG_UINT64 uTransReg_ISP_BGOBJVALS;
	IMG_UINT64 uTransReg_ISP_BGOBJDEPTH;

	IMG_UINT64 uTransReg_USC_PIXEL_OUTPUT_CTRL;
	IMG_UINT64 uTransReg_USC_CLEAR_REGISTER0;

	IMG_UINT64 uTransReg_PDS_BGND0_BASE;
	IMG_UINT64 uTransReg_PDS_BGND1_BASE;
	IMG_UINT64 uTransReg_PDS_BGND2_BASE;
	IMG_UINT64 uTransReg_PDS_BGND3_SIZEINFO;

	IMG_UINT64 uTransReg_TPU_BORDER_COLOUR_TABLE;
	IMG_UINT64 uTransReg_TPU_YUV_CSC_COEFFICIENTS;

	IMG_UINT64 uTransReg_ISP_RENDER;
	IMG_UINT64 uTransReg_ISP_MTILE_SIZE;
	IMG_UINT64 uTransReg_ISP_MTILE_BASE;
	IMG_UINT64 uTransReg_ISP_PIXEL_BASE;
	IMG_UINT64 uTransReg_ISP_RENDER_ORIGIN;
	IMG_UINT64 uTransReg_ISP_CTL;
#if defined(RGX_FEATURE_S7_TOP_INFRASTRUCTURE)
    IMG_UINT64  uTransReg_ISP_XTP_PIPE_ENABLE;
#endif
	IMG_UINT64 uTransReg_ISP_AA;

	IMG_UINT64 uTransReg_EVENT_PIXEL_PDS_INFO;
	IMG_UINT64 uTransReg_EVENT_PIXEL_PDS_CODE;
	IMG_UINT64 uTransReg_EVENT_PIXEL_PDS_DATA;

	IMG_UINT64 uTransReg_PBE_WORDX_MRTY[9]; /* TQ_MAX_RENDER_TARGETS * PBE_STATE_SIZE */

} RGXFWIF_TRANSFERREGISTERS, *PRGXFWIF_TRANSFERREGISTERS;



typedef struct _RGXFWIF_CMDTRANSFER_
{
	RGXFWIF_TRANSFERREGISTERS			RGXFW_ALIGN sTransRegs;

	IMG_UINT32				ui32FrameNum;
	IMG_UINT32				ui32Flags;/*!< flags */

} RGXFWIF_CMDTRANSFER, *PRGXFWIF_CMDTRANSFER;


typedef struct _RGXFWIF_TLAREGISTERS_
{ 
	IMG_UINT64	u2DReg_TLA_CMD_STREAM;
	IMG_UINT64	u2DReg_TLA_FBC_MEM_SRC_REGION;
	IMG_UINT64	u2DReg_TLA_FBC_MEM_SRC_CTRL;
	IMG_UINT64	u2DReg_TLA_FBC_MEM_DST_REGION;
	IMG_UINT64	u2DReg_TLA_FBC_MEM_DST_CTRL;
#if defined(FIX_HW_BRN_57193)
	IMG_UINT64	u2DReg_BRN57193_TLA_CMD_STREAM;
#endif
} RGXFWIF_2DREGISTERS, *PRGXFWIF_2DREGISTERS;

typedef struct _RGXFWIF_CMD2D_
{
	RGXFWIF_2DREGISTERS		RGXFW_ALIGN s2DRegs;

	IMG_UINT32 				ui32FrameNum;
	IMG_UINT32				ui32Flags;/*!< flags */

} RGXFWIF_CMD2D, *PRGXFWIF_CMD2D;

/*****************************************************************************
 Host interface structures.
*****************************************************************************/

#define		RGXMKIF_HWCFLAGS_NEWCONTEXT			0x00000001
#define 	RGXMKIF_HWCFLAGS_DUMMYTRANSFER		0x00000002
#define 	RGXMKIF_HWCFLAGS_DUMMY2D			0x00000004
#if defined(RGX_FEATURE_SPM_CONTEXT_SWITCH)
#define 	RGXMKIF_HWCFLAGS_DRAIN_USED			0x00000008
#endif

/*!
	Configuration registers which need to be loaded by the firmware before CDM
	can be started.
*/
typedef struct _RGXFWIF_CDM_REGISTERS_
{
	IMG_UINT64	uCDMReg_CDM_CTRL_STREAM_BASE;
	IMG_UINT64	uCDMReg_TPU_BORDER_COLOUR_TABLE;
	IMG_UINT64	uCDMReg_COMPUTE_THREAD_BARRIER;


	IMG_UINT64  uCDMReg_PDS_CTRL;

	IMG_UINT64  uCDMReg_TPU_YUV_CSC_COEFFICIENTS;

#if defined(RGX_FEATURE_COMPUTE_MORTON_CAPABLE)
	IMG_UINT64	uCDMReg_CDM_ITEM;
#endif
#if defined(RGX_FEATURE_TPU_CEM_DATAMASTER_GLOBAL_REGISTERS)
	IMG_UINT64  uCDMReg_TPU_CEM_CDM;
#else
	IMG_UINT64  uCDMReg_TPU;
#endif
	IMG_UINT64  uCDMReg_TPU_ARRAYS_CDM;
#if defined(RGX_FEATURE_CLUSTER_GROUPING)
	IMG_UINT64  uCDMReg_COMPUTE_CLUSTER;
#endif

#if defined(RGX_FEATURE_TPU_DM_GLOBAL_REGISTERS)
	IMG_UINT64 uCDMReg_TPU_TAG_CDM_CTRL;
#endif
} RGXFWIF_CDM_REGISTERS;

typedef struct _RGXFWIF_CDM_REGISTERS_CSWITCH_
{
	IMG_UINT64	uCDMReg_CDM_CONTEXT_STATE_BASE_ADDR;
	IMG_UINT64	uCDMReg_CDM_CONTEXT_PDS0;
	IMG_UINT64	uCDMReg_CDM_CONTEXT_PDS1;
	IMG_UINT64  uCDMReg_CDM_TERMINATE_PDS;
	IMG_UINT64  uCDMReg_CDM_TERMINATE_PDS1;
#if defined(RGX_SW_COMPUTE_PDS_BARRIER) && (RGX_NUM_PHANTOMS == 2)
	IMG_UINT64  uCDMReg_CDM_TERMINATE_PDS_PH2;	/* PDS task variant which synchronises 2 phantoms. Not used in FW */
	IMG_UINT64  uCDMReg_CDM_TERMINATE_PDS1_PH2;
#endif

	/* CDM resume controls */
	IMG_UINT64	uCDMReg_CDM_RESUME_PDS0;
	IMG_UINT64	uCDMReg_CDM_RESUME_PDS1;
#if defined(RGX_FEATURE_COMPUTE_MORTON_CAPABLE)
	IMG_UINT64	uCDMReg_CDM_CONTEXT_PDS0_B;
	IMG_UINT64	uCDMReg_CDM_RESUME_PDS0_B;
#endif

} RGXFWIF_CDM_REGISTERS_CSWITCH;

#if defined(FIX_HW_BRN_54441)
typedef struct _RGXFWIF_CDM_54441_REGISTERS_
{
	IMG_UINT64	uCDMReg_CDM_CTRL_STREAM_TERMINATE;
} RGXFWIF_CDM_54441_REGISTERS;
#endif

/*!
	RGX Compute command.
*/
typedef struct _RGXFWIF_CMD_COMPUTE_
{
	RGXFWIF_CDM_REGISTERS			RGXFW_ALIGN sCDMRegs;	/*!< CDM registers */
	RGXFWIF_CDM_REGISTERS_CSWITCH	RGXFW_ALIGN sCDMRegsCSwitch;/*!< CDM registers for ctx switch */
#if defined(FIX_HW_BRN_54441)
	RGXFWIF_CDM_54441_REGISTERS		RGXFW_ALIGN	sBRN54441CDMRegs;
#endif

	IMG_UINT32				ui32Flags;	/*!< Control flags */
	IMG_UINT32				ui32MaxDeadlineMS; /*!< Max HWR deadline limit in ms */
} RGXFWIF_CMD_COMPUTE;




/*!
	Configuration registers which need to be loaded by the firmware
	before a VRDM kick can be started.
*/
typedef struct _RGXFWIF_VRDMREGISTERS_
{
	IMG_UINT64  uVRDMReg_SHF_MIN_X_EXTENTS;
	IMG_UINT64  uVRDMReg_SHF_MIN_Y_EXTENTS;
	IMG_UINT64  uVRDMReg_SHF_MIN_Z_EXTENTS;
	IMG_UINT64  uVRDMReg_SHF_MAX_X_EXTENTS;
	IMG_UINT64  uVRDMReg_SHF_MAX_Y_EXTENTS;
	IMG_UINT64  uVRDMReg_SHF_MAX_Z_EXTENTS;

	IMG_UINT64  uVRDMReg_SHG_CTRL0;
	IMG_UINT64  uVRDMReg_SHG_CTRL1;
	
	IMG_UINT64  uVRDMReg_SHF_CTRL;
	
	IMG_UINT64  uVRDMReg_BF_CTRL;
	
	IMG_UINT64  uVRDMReg_VRM_CTRL_STREAM_BASE;
	IMG_UINT64  uVRDMReg_VRM_SYNC_PDS_DATA_BASE;
	IMG_UINT64  uVRDMReg_TPU_BORDER_COLOUR_TABLE;

	IMG_UINT64  uVRDMReg_PRIM_TABLE_BASE; /*!< primitive table base address */
	IMG_UINT64	uVRDMReg_PRIM_TABLE_SIZE; /*!< primitive table size (bytes) */

	IMG_UINT64  uVRDMReg_RPM_PAGE_TABLE_BASE;
	
	IMG_UINT64	uVRDMReg_RPM_SHF_FPL;
	IMG_UINT64	uVRDMReg_RPM_SHG_FPL;

} RGXFWIF_VRDMREGISTERS, *PRGXFWIF_VRDMREGISTERS;


/*!
	RGX VRDM command.
*/
typedef struct _RGXFWIF_CMD_VRDM_
{
	RGXFWIF_VRDMREGISTERS RGXFW_ALIGN sVRDMRegs;	/*!< VRDM registers */
	IMG_UINT32            ui32Flags;				/*!< Control flags */
	IMG_UINT32            ui32FrameNum;/*!< associated frame number */
#if defined(RGX_FEATURE_RAY_TRACING)
	RGXFWIF_DEV_VIRTADDR  asFWRPMFreeList[RGXFW_MAX_RPM_FREELISTS];	/*!< FW address for SHF/SHG free list */
	RGXFWIF_DEV_VIRTADDR  sHWFrameData; /* Frame Data associated with this command, this is used for RPM abort handling */
#endif
} RGXFWIF_CMDVRDM;


#if !(defined(__linux__) && defined(__KERNEL__))
/*!
	Configuration registers which need to be loaded by the firmware before a RTU
	kick can be started. Add REFS related registers to
	RGXFWIF_COMMON_RTU_REGISTERS instead.
*/
typedef struct _RGXFWIF_RTU_REGISTERS_
{
	IMG_UINT64 uDPXReg_RS_CTRL_FRAME_ENGINE_ADDR;
	IMG_UINT64 uDPXReg_RS_CTRL_CANCEL_TASK;
	IMG_UINT64 uDPXReg_FRAME_ENGINE_FREE_THRESH;
} RGXFWIF_RTU_REGISTERS, *PRGXFWIF_RTU_REGISTERS;

/*!
	RGX DPX command.
*/
typedef struct _RGXFWIF_CMD_RTU_
{
	RGXFWIF_RTU_REGISTERS        RGXFW_ALIGN sRTURegs;
	RGXFWIF_COMMON_RTU_REGISTERS RGXFW_ALIGN sRTUCommonRegs;

	RGXFWIF_COMMON_RTU_SETUP sRTUCommonSetup; /* bits shared with REFS */
	
	IMG_UINT32            ui32Flags;	/*!< Control flags */
	IMG_UINT32            ui32FrameNum;/*!< associated frame number */
	RGXFWIF_DEV_VIRTADDR  sHWFrameData; /* Frame Data associated with this command, this is used for RPM abort handling */
} RGXFWIF_CMDRTU;

/*!
	RGX FC command.
*/
typedef struct _RGXFWIF_CMD_RTU_FC_
{
	IMG_UINT32				ui32Woff;
	IMG_UINT32				ui32FrameContextID;
	IMG_UINT32            	ui32Flags;	/*!< Control flags */
} RGXFWIF_CMDRTU_FC;
#endif	/* !(defined(__linux__) && defined(__KERNEL__)) */

#endif /*  __RGX_FWIF_CLIENT_H__ */

/******************************************************************************
 End of file (rgx_fwif_client.h)
******************************************************************************/
