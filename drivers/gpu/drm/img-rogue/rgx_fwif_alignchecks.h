/*************************************************************************/ /*!
@File
@Title          RGX fw interface alignment checks
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    Checks to avoid disalignment in RGX fw data structures 
                shared with the host
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

#if !defined (__RGX_FWIF_ALIGNCHECKS_H__)
#define __RGX_FWIF_ALIGNCHECKS_H__

/* for the offsetof macro */
#include <stddef.h> 

/*!
 ******************************************************************************
 * Alignment checks array
 *****************************************************************************/

#define RGXFW_ALIGN_CHECKS_INIT0						\
		sizeof(RGXFWIF_TRACEBUF),						\
		offsetof(RGXFWIF_TRACEBUF, ui32LogType),		\
		offsetof(RGXFWIF_TRACEBUF, sTraceBuf),			\
		offsetof(RGXFWIF_TRACEBUF, aui16HwrDmLockedUpCount),	\
		offsetof(RGXFWIF_TRACEBUF, aui16HwrDmOverranCount),	\
		offsetof(RGXFWIF_TRACEBUF, aui16HwrDmRecoveredCount),	\
		offsetof(RGXFWIF_TRACEBUF, aui16HwrDmFalseDetectCount),	\
														\
		/* RGXFWIF_CMDTA checks */						\
		sizeof(RGXFWIF_CMDTA),							\
		offsetof(RGXFWIF_CMDTA, sTARegs),				\
														\
		/* RGXFWIF_CMD3D checks */						\
		sizeof(RGXFWIF_CMD3D),							\
		offsetof(RGXFWIF_CMD3D, s3DRegs),				\
														\
		/* RGXFWIF_CMDTRANSFER checks */				\
		sizeof(RGXFWIF_CMDTRANSFER),					\
		offsetof(RGXFWIF_CMDTRANSFER, sTransRegs),		\
														\
		/* RGXFWIF_CMD2D checks */						\
		sizeof(RGXFWIF_CMD2D),							\
		offsetof(RGXFWIF_CMD2D, s2DRegs),				\
														\
		/* RGXFWIF_CMD_COMPUTE checks */				\
		sizeof(RGXFWIF_CMD_COMPUTE),					\
		offsetof(RGXFWIF_CMD_COMPUTE, sCDMRegs),		\
									\
		sizeof(RGXFWIF_FREELIST), \
		offsetof(RGXFWIF_FREELIST, psFreeListDevVAddr),\
		offsetof(RGXFWIF_FREELIST, ui32MaxPages),\
		offsetof(RGXFWIF_FREELIST, ui32CurrentPages),\
		offsetof(RGXFWIF_FREELIST, ui32HWRCounter),\
									\
		sizeof(RGXFWIF_RENDER_TARGET),\
		offsetof(RGXFWIF_RENDER_TARGET, psVHeapTableDevVAddr), \
							\
		sizeof(RGXFWIF_HWRTDATA), \
		offsetof(RGXFWIF_HWRTDATA, psPMMListDevVAddr), \
		offsetof(RGXFWIF_HWRTDATA, apsFreeLists),\
		offsetof(RGXFWIF_HWRTDATA, ui64VCECatBase), \
		offsetof(RGXFWIF_HWRTDATA, psParentRenderTarget), \
		offsetof(RGXFWIF_HWRTDATA, eState), \
		offsetof(RGXFWIF_HWRTDATA, ui32NumPartialRenders), \
							\
		sizeof(RGXFWIF_HWPERF_CTL_BLK), \
		offsetof(RGXFWIF_HWPERF_CTL_BLK, aui64CounterCfg), \
							\
		sizeof(RGXFWIF_REGISTER_GUESTOS_OFFSETS), \
		offsetof(RGXFWIF_REGISTER_GUESTOS_OFFSETS, ui32OSid), \
		offsetof(RGXFWIF_REGISTER_GUESTOS_OFFSETS, sKCCBCtl), \
		offsetof(RGXFWIF_REGISTER_GUESTOS_OFFSETS, sKCCB), \
		offsetof(RGXFWIF_REGISTER_GUESTOS_OFFSETS, sFirmwareCCBCtl), \
		offsetof(RGXFWIF_REGISTER_GUESTOS_OFFSETS, sFirmwareCCB), \
\
		sizeof(RGXFWIF_HWPERF_CTL), \
		offsetof(RGXFWIF_HWPERF_CTL, SelCntr)


#if defined(RGX_FEATURE_RAY_TRACING)
#define RGXFW_ALIGN_CHECKS_INIT							\
		RGXFW_ALIGN_CHECKS_INIT0,						\
		sizeof(RGXFWIF_RPM_FREELIST), 					\
		offsetof(RGXFWIF_RPM_FREELIST, sFreeListDevVAddr),\
		offsetof(RGXFWIF_RPM_FREELIST, sRPMPageListDevVAddr),\
		offsetof(RGXFWIF_RPM_FREELIST, ui32MaxPages),\
		offsetof(RGXFWIF_RPM_FREELIST, ui32CurrentPages),\
		offsetof(RGXFWIF_RPM_FREELIST, ui32GrowPages)

#else
#define RGXFW_ALIGN_CHECKS_INIT		RGXFW_ALIGN_CHECKS_INIT0

#endif /* RGX_FEATURE_RAY_TRACING */

#endif /*  __RGX_FWIF_ALIGNCHECKS_H__ */

/******************************************************************************
 End of file (rgx_fwif_alignchecks.h)
******************************************************************************/


