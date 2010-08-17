/*
 *  tusbd/utils.h
 *
 *  Copyright (C) 2007-2008 IncentivesPro
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef _UTILS_H_
#define _UTILS_H_

#include "urb_chain.h"
#include "waitable_queue.h"

struct usbd_usb_request
{
	struct urb	*purb; // used for solid transfers
	void 		*buffer;

	struct urb_chain urbchain; // used for partitioned transfers
	struct buf_chain bufchain;

	struct usbdevice_descriptor *pusbdevice;
	uint8 		endpoint;
};

#ifdef _USBD_ENABLE_STUB_
size_t usbd_copy_n_free_unrb(void __user *buf, size_t count, struct waitable_queue_entry *pqentry);
struct waitable_queue_entry *usbd_alloc_n_copy_unrb(void __user *buf, size_t count, struct usbdevice_descriptor* pusbdevice, struct usb_device *pudev);
#endif

void dump_urb(struct urb *purb, uint64 unique_id);
void dump_unrb(PUNRB punrb);

#ifdef _USBD_ENABLE_VHCI_
int usbd_pack_urb(struct urb *purb, uint64 unique_id, void *buf);
int usbd_unpack_urb(PUNRB punrb, struct urb *purb);
#endif

int usbd_map_vmem(void *vmem, struct vm_area_struct *vma, unsigned long len);

uint64 usbd_get_unique_id(void);

size_t usbd_pack_iso_buffer(struct usb_iso_packet_descriptor *packets, int count, void *dst, void *src, int use_actual_length);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,30)
#define get_busid(dev) dev_name(dev)
#else
#define get_busid(dev) (dev)->bus_id
#endif

#endif //_UTILS_H_
