/*
 * arch/arm/mach-tegra/nvec_user.c
 *
 * User-land access to NvEc embedded controller features
 *
 * Copyright (c) 2008-2009, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/miscdevice.h>
#include "linux/nvos_ioctl.h"
#include "nvec.h"
#include "linux/nvec_ioctls.h"
#include "nvreftrack.h"
#include "nvassert.h"

static NvRtHandle s_RtHandle = NULL;

NvError NvECPackage_Dispatch(void *InBuffer, NvU32 InSize, void *OutBuffer,
	NvU32 OutSize, NvDispatchCtx* Ctx);

static int nvec_open(struct inode *inode, struct file *file);
static int nvec_close(struct inode *inode, struct file *file);
static long nvec_unlocked_ioctl(struct file *file,
	unsigned int cmd, unsigned long arg);

int nvec_open(struct inode *inode, struct file *file)
{
	NvRtClientHandle Client;

	if (NvRtRegisterClient(s_RtHandle, &Client) != NvSuccess)
		return -ENOMEM;

	file->private_data = (void*)Client;
	return 0;
}
 
int nvec_close(struct inode *inode, struct file *file)
{
	NvRtClientHandle client = (NvRtClientHandle)file->private_data;

	if (NvRtUnregisterClient(s_RtHandle, client)) {
		NvDispatchCtx dctx;

		dctx.Rt = s_RtHandle;
		dctx.Client = client;
		dctx.PackageIdx = 0;

		// TODO: Enable this code for freeing up leaked handles
		#if 0
		for (;;)
		{
			void* ptr = NvRtFreeObjRef(&dctx,
				NvRtObjType_NvEc_NvEcHandle, NULL);
			if (!ptr) break;
			NVRT_LEAK("NvEc", "NvEcHandle", ptr);
			NvEcClose(ptr);
		}
		#endif

		NvRtUnregisterClient(s_RtHandle, client);
	}
	return 0;
}
 
long nvec_unlocked_ioctl(struct file *file,
	unsigned int cmd, unsigned long arg)
{
	NvError err;
	NvOsIoctlParams p;
	NvU32 size;
	NvU32 small_buf[8];
	void *ptr = 0;
	long e;
	NvBool bAlloc = NV_FALSE;

	switch( cmd ) {
	case NvECKernelIoctls_Generic:
	{
		NvDispatchCtx dctx;

		dctx.Rt = s_RtHandle;
		dctx.Client = (NvRtClientHandle)file->private_data;
		dctx.PackageIdx = 0;

		err = NvOsCopyIn(&p, (void *)arg, sizeof(p));
		if (err != NvSuccess) {
			printk("NvECKernelIoctls_Generic: copy in failed\n");
			goto fail;
		}

		size = p.InBufferSize + p.InOutBufferSize + p.OutBufferSize;
		if (size <= sizeof(small_buf)) {
			ptr = small_buf;
		} else {
			ptr = NvOsAlloc(size);
			if (!ptr) {
				printk("NvECKernelIoctls_Generic: alloc err\n");
				goto fail;
			}

			bAlloc = NV_TRUE;
		}

		err = NvOsCopyIn(ptr, p.pBuffer, p.InBufferSize +
			p.InOutBufferSize);
		if (err != NvSuccess) {
			printk("NvECKernelIoctls_Generic: copy in failure\n");
			goto fail;
		}

		err = NvECPackage_Dispatch(ptr,
			p.InBufferSize + p.InOutBufferSize,
			((NvU8 *)ptr) + p.InBufferSize, p.InOutBufferSize +
			p.OutBufferSize, &dctx);
		if (err != NvSuccess) {
			printk("NvECKernelIoctls_Generic: dispatch failure\n");
			goto fail;
		}

		if (p.InOutBufferSize || p.OutBufferSize) {
			err = NvOsCopyOut(
				((NvU8 *)((NvOsIoctlParams *)arg)->pBuffer) +
					p.InBufferSize,
				((NvU8 *)ptr) + p.InBufferSize,
				p.InOutBufferSize + p.OutBufferSize);
			if (err != NvSuccess) {
				printk("NvECKernelIoctls_Generic: copyout err\n");
				goto fail;
			}
		}

		break;
	}
	default:
		printk("unknown ioctl code\n");
		goto fail;
	}
	e = 0;
	goto clean;

fail:
	e = -EINVAL;

clean:
	if (bAlloc)
		NvOsFree(ptr);

	return e;
}

#define DEVICE_NAME "nvec"

static const struct file_operations nvec_fops =
{
	.owner		= THIS_MODULE,
	.open		= nvec_open,
	.release	= nvec_close,
	.unlocked_ioctl	= nvec_unlocked_ioctl,
};

static struct miscdevice nvec_dev =
{
	.name	= DEVICE_NAME,
	.fops	= &nvec_fops,
	.minor	= MISC_DYNAMIC_MINOR,
};

static NvEcHandle s_NvEcHandle = NULL;

static int __init nvec_init( void )
{
	int e = 0;
	NvError status;
	NvU32 NumTypes = 1; // TODO: must have NvRtObjType_NvEc_Num instead;

	NV_ASSERT(s_RtHandle == NULL);

	if (NvRtCreate(1, &NumTypes, &s_RtHandle) != NvSuccess) {
		printk("nvec NvRtCreate returned error\n");
		return -ENOMEM;
	}

	status = NvEcOpen(&s_NvEcHandle, 0);
	if (status != NvError_Success) {
		printk("nvec NvEcOpen returned 0x%x\n", status);
		return -EINVAL;
	}

	e = misc_register(&nvec_dev);
	if (e < 0) {
		if (s_RtHandle) {
			NvRtDestroy(s_RtHandle);
			s_RtHandle = NULL;
		}
		printk("nvec failed to open\n");
	}
	return e;
}

static void __exit nvec_deinit( void )
{
	NvEcClose(s_NvEcHandle);
	misc_deregister(&nvec_dev);
	NvRtDestroy(s_RtHandle);
	s_RtHandle = NULL;
}

module_init(nvec_init);
module_exit(nvec_deinit);

