/* -*- mode: c; indent-tabs-mode: t; c-basic-offset: 8; tab-width: 8 -*- */
/* vi: set ts=8 sw=8 sts=8: */
/*************************************************************************/ /*!
@File
@Title          PowerVR drm driver
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    drm module
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

#if !defined(__PVR_DRM_H__)
#define __PVR_DRM_H__

#include <linux/version.h>
#include <drm/drmP.h>

#include "module_common.h"
#include "connection_server.h"
#include "sync_server.h"
#include "pmr.h"

#if defined(SUPPORT_BUFFER_SYNC)
#include "pvr_fence.h"
#endif

#if defined(PDUMP)
#include "linuxsrv.h"
#endif

#if defined(SUPPORT_DRM)
#if (!defined(LDM_PLATFORM) && !defined(LDM_PCI)) || \
	(defined(LDM_PLATFORM) && defined(LDM_PCI))
	#error "LDM_PLATFORM or LDM_PCI must be defined"
#endif

#define	PVR_DRM_FILE_FROM_FILE(pFile)		((struct drm_file *)((pFile)->private_data))
#define	PVR_FILE_FROM_DRM_FILE(pDRMFile)	((pDRMFile)->filp)

extern struct drm_driver sPVRDRMDriver;

int PVRSRVSystemInit(struct drm_device *pDrmDevice);
void PVRSRVSystemDeInit(LDM_DEV *pDevice);

int PVRSRVOpen(struct drm_device *dev, struct drm_file *file);
void PVRSRVRelease(struct drm_device *dev, struct drm_file *file);

#if defined(PDUMP)
int dbgdrv_init(void);
void dbgdrv_cleanup(void);
int dbgdrv_ioctl(struct drm_device *dev, void *arg, struct drm_file *file);
int dbgdrv_ioctl_compat(struct drm_device *dev, void *arg, struct drm_file *file);
#endif

int PVRSRV_BridgeDispatchKM(struct drm_device *dev, void *arg, struct drm_file *file);

#if defined(CONFIG_COMPAT)
int PVRSRV_BridgeCompatDispatchKM(struct file *file, unsigned int cmd, unsigned long arg);
#endif

#endif	/* defined(SUPPORT_DRM) */
#endif /* !defined(__PVR_DRM_H__) */
