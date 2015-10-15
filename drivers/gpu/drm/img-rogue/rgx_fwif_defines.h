/*************************************************************************/ /*!
@File
@Title          RGX firmware interface defines
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    RGX firmware interface defines used by srvclient and ogl
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
#if !defined (__RGX_FWIF_DEFINES_H__)
#define __RGX_FWIF_DEFINES_H__

/******************************************************************************
 kernel/client functions:
******************************************************************************/

/* 
 * Minimum PB size supported by RGX and Services.
 * 
 * This is based on the fact that the reissue FIFO is 9 pages deep.
 * 
 * Minimum PB = 272K + (NUM_TE_PIPES-1)*16K + (NUM_VCE_PIPES-1)*64K + IF_PM_PREALLOC(NUM_TE_PIPES*16K + NUM_VCE_PIPES*16K)
 */
#if defined(RGX_FEATURE_SCALABLE_TE_ARCH)
#define		RGX_PM_FLPAGES_FOR_SCALABLE_TE		((RGX_FEATURE_SCALABLE_TE_ARCH-1)*4)
#else
#define		RGX_PM_FLPAGES_FOR_SCALABLE_TE		(0*4)
#endif

#if defined(RGX_FEATURE_SCALABLE_VCE)
#define		RGX_PM_FLPAGES_FOR_SCALABLE_VCE		((RGX_FEATURE_SCALABLE_VCE-1)*16)
#else
#define		RGX_PM_FLPAGES_FOR_SCALABLE_VCE		(0*16)
#endif

#if defined(HW_ERN_46066)
/* With PM Pre-Alloc each requester can take an additional page, for VCE2 + TE2 config, it is 64KB. */
#if defined(RGX_FEATURE_SCALABLE_TE_ARCH)
#define		RGX_PM_FLPAGES_FOR_TE_PREALLOC		(RGX_FEATURE_SCALABLE_TE_ARCH*4)
#else
#define		RGX_PM_FLPAGES_FOR_TE_PREALLOC		(1*4)
#endif
#if defined(RGX_FEATURE_SCALABLE_VCE)
#define		RGX_PM_FLPAGES_FOR_VCE_PREALLOC		(RGX_FEATURE_SCALABLE_VCE*4)
#else
#define		RGX_PM_FLPAGES_FOR_VCE_PREALLOC		(1*4)
#endif
#else
#define		RGX_PM_FLPAGES_FOR_TE_PREALLOC		(0*4)
#define		RGX_PM_FLPAGES_FOR_VCE_PREALLOC		(0*4)
#endif

#if defined(SUPPORT_MMU_FREELIST)
#define		RGX_PM_MIN_FLSIZE	   				((RGX_PM_FLPAGES_FOR_SCALABLE_TE  +    \
                                                  RGX_PM_FLPAGES_FOR_SCALABLE_VCE +    \
                                                  RGX_PM_FLPAGES_FOR_TE_PREALLOC  +    \
                                                  RGX_PM_FLPAGES_FOR_VCE_PREALLOC +    \
                                                  53) * RGX_BIF_PM_PHYSICAL_PAGE_SIZE)
#define		RGX_PM_MMU_MIN_FLSIZE				(16 * RGX_BIF_PM_PHYSICAL_PAGE_SIZE)
#else
#define		RGX_PM_MIN_FLSIZE					((RGX_PM_FLPAGES_FOR_SCALABLE_TE  +    \
                                                  RGX_PM_FLPAGES_FOR_SCALABLE_VCE +    \
                                                  RGX_PM_FLPAGES_FOR_TE_PREALLOC  +    \
                                                  RGX_PM_FLPAGES_FOR_VCE_PREALLOC +    \
                                                  68) * RGX_BIF_PM_PHYSICAL_PAGE_SIZE)
#endif

#endif /*  __RGX_FWIF_DEFINES_H__ */

/******************************************************************************
 End of file (rgx_fwif_defines.h)
******************************************************************************/


