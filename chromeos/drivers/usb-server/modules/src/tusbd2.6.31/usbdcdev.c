/*
 *  tusbd/usbdcdev.c
 *
 *  Copyright (C) 2007-2008 IncentivesPro
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include "usbd.h"

#define MAX_MINORS 255

static dev_t s_devnum = MKDEV(0,0);

static void *s_pminors[MAX_MINORS];

ssize_t cdev_read(struct file *, char __user *, size_t, loff_t *);
ssize_t cdev_write(struct file *, const char __user *, size_t, loff_t *);
int cdev_ioctl(struct inode *, struct file *, unsigned int, unsigned long);
int cdev_open(struct inode *, struct file *);
int cdev_release(struct inode *, struct file *);
unsigned int cdev_poll(struct file *filp, poll_table *wait);
int cdev_mmap(struct file *filp, struct vm_area_struct *vma);

static struct file_operations g_cdev_fops = {
    .owner =    THIS_MODULE,
    .read =     cdev_read,
    .write =    cdev_write,
    .poll =  	cdev_poll,
    .ioctl =    cdev_ioctl,
    .mmap =    	cdev_mmap,
    .open =     cdev_open,
    .release =  cdev_release,
};

int usbd_cdev_init_module(int *pmajor)
{
	int result = 0;

	TRACE("init_cdev_module: ++\n");

	if( s_devnum == MKDEV(0,0) )
	{
		result = alloc_chrdev_region(&s_devnum, 0, 255, "usbd_cdev");

		if( result != 0 )
			TRACE("init_cdev_module: alloc_chrdev_region failed. Error number %d\n", result);

		TRACE("init_cdev_module: allocated devnum region: Major %d Minor %d\n", MAJOR(s_devnum), MINOR(s_devnum));

		memset(s_pminors, 0, sizeof(s_pminors));

		*pmajor = MAJOR(s_devnum);

	} else
	{
		TRACE("init_cdev_module: attempt to reinit initialized cdev module\n");
	}

	TRACE("init_cdev_module: -- result=%d\n", result);

	return result;
}

int usbd_cdev_deinit_module(void)
{
	if( s_devnum != MKDEV(0,0) )
	{
		unregister_chrdev_region(s_devnum, 255);
		s_devnum = MKDEV(0,0);
	} else
	{
		TRACE("deinit_cdev_module: attempt to deinit uninitialized cdev module\n");
	}

	return 0;
}


// pcdev - on return contains initialized cdev structure
int usbd_cdev_create(struct cdev *pcdev)
{
	int result;
	dev_t devnum;


	TRACE("create_cdev: ++\n");

	cdev_init(pcdev, &g_cdev_fops);

	devnum = MKDEV(MAJOR(s_devnum), 0);
		
	result = cdev_add(pcdev, devnum, MAX_MINORS);

	if( result != 0 )
		TRACE("create_cdev: cdev_add failed. Error number %d\n", result);

	TRACE("create_cdev: -- result=%d\n", result);

	return result;
}

int usbd_cdev_destroy(struct cdev *pcdev)
{
	TRACE("destroy_cdev\n");
	cdev_del(pcdev);
	return 0;
}

int usbd_cdev_alloc_minor(struct minor_descriptor *pdescriptor)
{
	int i;

	for(i=0;i<MAX_MINORS;i++)
	{
		if( s_pminors[i] == NULL )
		{
			s_pminors[i] = pdescriptor;
			break;
		}
	}

	if( i == MAX_MINORS ) return (-1);

	pdescriptor->minor = i;

	return 0;
}

dev_t usbd_cdev_get_minor_devnum(int minor)
{
	return MKDEV(MAJOR(s_devnum), minor);
}

void *usbd_cdev_get_minor_context(int minor)
{
	if( minor < MAX_MINORS )
		return s_pminors[minor];

	return NULL;
}


void usbd_cdev_free_minor(int minor)
{
	if( minor < MAX_MINORS && s_pminors[minor] != NULL )
	{
		s_pminors[minor] = NULL;
	}
}

/******************************************************************************/
// Character device operations

ssize_t cdev_read(
	struct file *filp, 
	char __user *buff, 
	size_t count, 
	loff_t *offp)
{
//#if LINUX_VERSION_CODE == KERNEL_VERSION(2,6,21)
//	u32	m = iminor(filp->f_path.dentry->d_inode);
//#elif LINUX_VERSION_CODE == KERNEL_VERSION(2,6,17)
	u32	m = iminor(filp->f_dentry->d_inode);
//#endif
	struct minor_descriptor *pminor;
//	TRACE("cdev_read(device %08X, buffer %08X, count %d)\n", m, (u32)buff, count);

	pminor = s_pminors[m];

	if( pminor )
	{
		if( pminor->ops.read )
			return pminor->ops.read(pminor->context, buff, count);
		else
			return 0;
	}
	return -ENODEV;
}

ssize_t cdev_write(
	struct file *filp, 
	const char __user *buff, 
	size_t count, 
	loff_t *offp)
{
//#if LINUX_VERSION_CODE == KERNEL_VERSION(2,6,21)
//	u32	m = iminor(filp->f_path.dentry->d_inode);
//#elif LINUX_VERSION_CODE == KERNEL_VERSION(2,6,17)
	u32	m = iminor(filp->f_dentry->d_inode);
//#endif
	struct minor_descriptor *pminor;

//	TRACE("cdev_write(device %08X, buffer %08X, count %d)\n", m, (u32)buff, count);

	pminor = s_pminors[m];

	if( pminor )
	{
		if( pminor->ops.write )
			return pminor->ops.write(pminor->context, buff, count);
		else
			return 0;
	}
	return -ENODEV;
}

unsigned int cdev_poll(
	struct file *filp, 
	poll_table *wait)
{
//#if LINUX_VERSION_CODE == KERNEL_VERSION(2,6,21)
//	u32	m = iminor(filp->f_path.dentry->d_inode);
//#elif LINUX_VERSION_CODE == KERNEL_VERSION(2,6,17)
	u32	m = iminor(filp->f_dentry->d_inode);
//#endif
	struct minor_descriptor *pminor;

//	TRACE("cdev_poll(device %08X)\n", m/*filp->private_data*/);

	pminor = s_pminors[m];

	if( pminor )
	{
		if( pminor->ops.poll )
			return pminor->ops.poll(pminor->context, filp, wait);
		else
			return 0;
	}
	return -ENODEV;
}

int cdev_ioctl(
	struct inode *inode, 
	struct file *filp, 
	unsigned int cmd, 
	unsigned long arg)
{
	u32 m = iminor(inode);
	struct minor_descriptor *pminor;

//	TRACE("cdev_ioctl(device %08X, cmd %08X, arg %08X)\n", m, cmd, (u32)arg);

	pminor = s_pminors[m];

	if( pminor )
	{
		if( pminor->ops.ioctl )
			return pminor->ops.ioctl(pminor->context, cmd, arg);
		else
			return 0;
	}
	return -ENODEV;
}

int cdev_mmap(
	struct file *filp, 
	struct vm_area_struct *vma)
{
	u32	m = iminor(filp->f_dentry->d_inode);
	struct minor_descriptor *pminor;

	pminor = s_pminors[m];

	if( pminor && pminor->ops.mmap )
	{
		if( pminor->ops.mmap )
			return pminor->ops.mmap(pminor->context, vma);
		else
			return -EINVAL;
	}
	return -ENODEV;
}


int cdev_open(
	struct inode *inode, 
	struct file *filp)
{
	u32 m = iminor(inode);
	struct minor_descriptor *pminor;

//	TRACE("cdev_open(device %08X)\n", m);

	filp->private_data = inode->i_cdev;

	pminor = s_pminors[m];

	if( pminor )
	{
		if( pminor->ops.open )
			return pminor->ops.open(pminor->context);
		else
			return 0;
	}
	return -ENODEV;
}	

int cdev_release(
	struct inode *inode, 
	struct file *filp)
{
	u32 m = iminor(inode);
	struct minor_descriptor *pminor;

//	TRACE("cdev_close(device %08X)\n", m);

	pminor = s_pminors[m];

	if( pminor )
	{
		if( pminor->ops.release )
			return pminor->ops.release(pminor->context);
		else
			return 0;
	}
	return -ENODEV;
}

