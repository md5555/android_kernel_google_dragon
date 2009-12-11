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
#include "nvec.h"
#include "nvreftrack.h"

NvError NvEc_Dispatch(void *InBuffer, NvU32 InSize, void *OutBuffer,
	NvU32 OutSize, NvDispatchCtx* Ctx);

static int nvec_open(struct inode *inode, struct file *file);
static int nvec_close(struct inode *inode, struct file *file);
static long nvec_unlocked_ioctl(struct file *file,
	unsigned int cmd, unsigned long arg);

int nvec_open(struct inode *inode, struct file *filp)
{
	return 0;
}
 
int nvec_close(struct inode *inode, struct file *filp)
{
	return 0;
}
 
long nvec_unlocked_ioctl(struct file *file,
	unsigned int cmd, unsigned long arg)
{
	return 0;
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
	printk("nvec init\n");

	status = NvEcOpen(&s_NvEcHandle, 0);
	if (status != NvError_Success) {
		return -EINVAL;
	}
	e = misc_register( &nvec_dev );
	return e;
}

static void __exit nvec_deinit( void )
{
	NvEcClose(s_NvEcHandle);
	misc_deregister( &nvec_dev );
}

module_init(nvec_init);
module_exit(nvec_deinit);

