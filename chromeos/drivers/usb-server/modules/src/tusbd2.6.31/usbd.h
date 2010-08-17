/*
 *  tusbd/usbd.h
 *
 *  Copyright (C) 2007-2008 IncentivesPro
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/usb.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/poll.h>
#include <linux/spinlock.h>
#include <linux/list.h>
#include <linux/version.h>
#include <linux/kref.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include <linux/vmalloc.h>
#include "../public/pubstt2.h"
#include "../public/nettypes.h"
#include "../public/pubuniprotocol.h"
#include "waitable_queue.h"
#include "driver.h"
#include "fifo.h"
#include "usbdcdev.h"
#include "usb_device.h"
#include "vhci_device.h"
#include "minor_control.h"
#include "minor_device.h"
#include "minor_vhci.h"
#include "utils.h"
#include "bind_unbind.h"
#include "urb_chain.h"

#define IO_BUFFER_SIZE (1024*1024)

#define USBD_STUB_DRIVER_NAME "usb-stub-driver"
#define USBD_VHCI_DRIVER_NAME "usb-vhci-driver"

#define TRACE_CRITICAL(fmt, args...) printk( KERN_DEBUG "usbd: " fmt, ## args)

#ifdef _USBD_DEBUG_BUILD_
#define TRACE(fmt, args...) printk( KERN_DEBUG "usbd [%09u]: " fmt, jiffies_to_msecs(jiffies), ## args)
#else
#define TRACE(fmt, args...)
#endif

#define MIN_OF(a,b) ((a)<(b)?(a):(b))

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24)
/*TODO: 64bit beware!*/
typedef unsigned long uintptr_t;
#endif
