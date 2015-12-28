/*************************************************************************/ /*!
@File
@Title          MT8173 DRM module setup
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
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

#include <drm/drmP.h>
#include <linux/dma-mapping.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/version.h>

#include "module_common.h"
#include "pvr_debug.h"
#include "pvr_drm.h"
#include "pvrmodule.h"
#include "linkage.h"
#include "srvkm.h"
#include "syscommon.h"
#include "sysinfo.h"
#include "mt8173/mt8173_mfgsys.h"

/*
 * DRVNAME is the name we use to register our driver.
 * DEVNAME is the name we use to register actual device nodes.
 */
#define DRVNAME  PVR_LDM_DRIVER_REGISTRATION_NAME
#define DEVNAME  PVRSRV_MODNAME

/*
 * This is all module configuration stuff required by the linux kernel.
 */
MODULE_SUPPORTED_DEVICE(DEVNAME);

static int PVRSRVDriverRemove(struct platform_device *device);
static int PVRSRVDriverProbe(struct platform_device *device);

static const struct dev_pm_ops powervr_dev_pm_ops = {
	.suspend	= PVRSRVDriverSuspend,
	.resume		= PVRSRVDriverResume,
};

static const struct of_device_id mt_powervr_of_match[] = {
	{ .compatible = "mediatek,mt8173-gpu", },
	{ .compatible = "mediatek,HAN", },
	{},
};

MODULE_DEVICE_TABLE(of, mt_powervr_of_match);

static struct platform_driver powervr_driver = {
	.driver = {
		.name	= DRVNAME,
		.of_match_table = of_match_ptr(mt_powervr_of_match),
		.pm	= &powervr_dev_pm_ops,
	},
	.probe		= PVRSRVDriverProbe,
	.remove		= PVRSRVDriverRemove,
	.shutdown	= PVRSRVDriverShutdown,
};

/*!
******************************************************************************

 @Function		PVRSRVSystemInit

 @Description

 Wrapper for PVRSRVInit.

 @input pDevice - the device for which a probe is requested

 @Return 0 for success or <0 for an error.

*****************************************************************************/
int PVRSRVSystemInit(struct drm_device *pDrmDevice)
{
	struct platform_device *pDevice = pDrmDevice->platformdev;
	PVRSRV_ERROR err;

	PVR_TRACE(("%s (pDevice=%p)", __func__, pDevice));

	gpsPVRLDMDev = pDevice;

	err = PVRSRVInit(pDevice);
	if (err) {
		if (err == PVRSRV_ERROR_PROBE_DEFER)
			return -EPROBE_DEFER;
		else
			return -ENODEV;
	}

	return 0;
}

/*!
******************************************************************************

 @Function		PVRSRVSystemDeInit

 @Description

 Wrapper for PVRSRVDeInit.

 @input pDevice - the device for which a probe is requested
 @Return nothing.

*****************************************************************************/
void PVRSRVSystemDeInit(struct platform_device *pDevice)
{
	PVR_TRACE(("%s (pDevice=%p)", __func__, pDevice));
	PVRSRVDeInit(pDevice);
}

/*!
******************************************************************************

 @Function		PVRSRVDriverProbe

 @Description

 See whether a given device is really one we can drive.

 @input pDevice - the device for which a probe is requested

 @Return 0 for success or <0 for an error.

*****************************************************************************/
static int PVRSRVDriverProbe(struct platform_device *pDevice)
{
	int result;

	PVR_TRACE(("%s (pDevice=%p)", __func__, pDevice));

	if (OSStringCompare(pDevice->name, DEVNAME) != 0) {
		result = MTKMFGBaseInit(&pDevice->dev);
		if (result != 0)
			return result;
	}

	result = drm_platform_init(&sPVRDRMDriver, pDevice);

	dma_set_mask(&pDevice->dev, DMA_BIT_MASK(33));

	if (result == 0)
		PVRSRVDeviceInit();

	return result;
}


/*!
******************************************************************************

 @Function		PVRSRVDriverRemove

 @Description

 This call is the opposite of the probe call; it is called when the device is
 being removed from the driver's control.

 @input pDevice - the device for which driver detachment is happening

 @Return 0, or no return value at all, depending on the device type.

*****************************************************************************/
static int PVRSRVDriverRemove(struct platform_device *pDevice)
{
	PVR_TRACE(("%s (pDevice=%p)", __func__, pDevice));

	PVRSRVDeviceDeinit();
	MTKMFGBaseDeInit(&pDevice->dev);
	drm_put_dev(platform_get_drvdata(pDevice));

	return 0;
}

/*!
******************************************************************************

 @Function		PVRSRVOpen

 @Description

 Open the PVR services node.

 @input pInode - the inode for the file being opened.
 @input dev    - the DRM device corresponding to this driver.

 @input pFile - the file handle data for the actual file being opened

 @Return 0 for success or <0 for an error.

*****************************************************************************/
int PVRSRVOpen(struct drm_device unref__ *dev, struct drm_file *pDRMFile)
{
	struct file *pFile = PVR_FILE_FROM_DRM_FILE(pDRMFile);
	int err;

	if (!try_module_get(THIS_MODULE)) {
		PVR_DPF((PVR_DBG_ERROR, "Failed to get module"));
		return -ENOENT;
	}

	err = PVRSRVCommonOpen(pFile);
	if (err != 0)
		module_put(THIS_MODULE);

	return err;
}

/*!
******************************************************************************

 @Function		PVRSRVRelease

 @Description

 Release access the PVR services node - called when a file is closed, whether
 at exit or using close(2) system call.

 @input pInode - the inode for the file being released
 @input pvPrivData - driver private data

 @input pFile - the file handle data for the actual file being released

 @Return 0 for success or <0 for an error.

*****************************************************************************/
void PVRSRVRelease(struct drm_device unref__ *dev, struct drm_file *pDRMFile)
{
	struct file *pFile = PVR_FILE_FROM_DRM_FILE(pDRMFile);

	PVRSRVCommonRelease(pFile);

	module_put(THIS_MODULE);
}

/*!
******************************************************************************

 @Function		PVRCore_Init

 @Description

 Insert the driver into the kernel.

 Readable and/or writable debugfs entries under /sys/kernel/debug/pvr are
 created with PVRDebugFSCreateEntry().  These can be read at runtime to get
 information about the device (eg. 'cat /sys/kernel/debug/pvr/nodes')

 __init places the function in a special memory section that the kernel frees
 once the function has been run.  Refer also to module_init() macro call below.

 @input none

 @Return none

*****************************************************************************/
static int __init PVRCore_Init(void)
{
	int error = 0;

	PVR_TRACE(("%s", __func__));

#if defined(PDUMP)
	error = dbgdrv_init();
	if (error != 0)
		return error;
#endif

	error = PVRSRVDriverInit();
	if (error != 0)
		return error;

	error = platform_driver_register(&powervr_driver);
	if (error != 0) {
		PVR_DPF((PVR_DBG_ERROR,
		         "%s: unable to register platform driver (err=%d)",
		         __func__, error));
		return error;
	}

	return 0;
}

/*!
*****************************************************************************

 @Function		PVRCore_Cleanup

 @Description

 Remove the driver from the kernel.

 There's no way we can get out of being unloaded other than panicking; we
 just do everything and plough on regardless of error.

 __exit places the function in a special memory section that the kernel frees
 once the function has been run.  Refer also to module_exit() macro call below.

 @input none

 @Return none

*****************************************************************************/
static void __exit PVRCore_Cleanup(void)
{
	PVR_TRACE(("%s", __func__));

	platform_driver_unregister(&powervr_driver);

	PVRSRVDriverDeinit();

#if defined(PDUMP)
	dbgdrv_cleanup();
#endif
	PVR_TRACE(("%s: unloaded", __func__));
}

/*
 * These macro calls define the initialisation and removal functions of the
 * driver.  Although they are prefixed `module_', they apply when compiling
 * statically as well; in both cases they define the function the kernel will
 * run to start/stop the driver.
 */
module_init(PVRCore_Init);
module_exit(PVRCore_Cleanup);
