/*
 *  tusbd/usbdcdev.h
 *
 *  Copyright (C) 2007-2008 IncentivesPro
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef _USBCDEV_H
#define _USBCDEV_H

struct minor_descriptor
{
	struct
	{
		ssize_t (*read)(void *, char __user *, size_t);
		ssize_t (*write)(void *, const char __user *, size_t);
		int (*ioctl)(void *, unsigned int, unsigned long);
		int (*open)(void *);
		int (*release)(void *);
		unsigned int (*poll)(void *, struct file *filp, poll_table *);
		int (*mmap)(void *, struct vm_area_struct *vma);
	} ops;

	int minor;
	void *context;
	struct fifo_descriptor *fifo;
};

int usbd_cdev_init_module(int *pmajor);
int usbd_cdev_deinit_module(void);

int usbd_cdev_create(struct cdev *pcdev);
int usbd_cdev_destroy(struct cdev *pcdev);

int usbd_cdev_alloc_minor(struct minor_descriptor *pdescriptor);
void usbd_cdev_free_minor(int minor);
void *usbd_cdev_get_minor_context(int minor);
dev_t usbd_cdev_get_minor_devnum(int minor);


#endif // _USBCDEV_H
