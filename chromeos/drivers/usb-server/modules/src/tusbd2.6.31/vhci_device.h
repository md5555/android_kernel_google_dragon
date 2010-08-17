/*
 *  tusbd/vhci_device.h
 *
 *  Copyright (C) 2007 IncentivesPro
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef _VHCI_DEVICE_H
#define _VHCI_DEVICE_H

#ifdef _USBD_ENABLE_VHCI_

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
#define VHCI_USB_MAX_CHILDREN	15
#else
#define VHCI_USB_MAX_CHILDREN	USB_MAXCHILDREN
#endif

// number of bytes required to contain a bitmap of all hub ports
#define VHCI_USB_MAX_CHILDREN_BYTES ((VHCI_USB_MAX_CHILDREN / 8) + 1)

struct vhci_device_descriptor
{
    struct list_head list;
    struct minor_descriptor *pminor;
	struct kref reference;

	int port;
	int address;
	int speed;
	int reset_in_progress;

	u32 hserver;
	u32 hdevice;

	struct usb_hcd *parent;

	struct waitable_queue queue_pending_transfer_unrb; // queue of UNRBs waiting for transfer
	struct waitable_queue queue_pending_completion_unrb; // queue of UNRBs waiting for completion

	void *iobuf; // i/o buffer
	void *readbuf; // buffer for outgoing data
	void *writebuf; // buffer for incoming data
};

struct vhci_queue_data
{
	uint64 unique_id;
	struct urb* purb;
	PUNRB punrb;
};


struct vhci_hcd_descriptor
{
	spinlock_t lock;
	struct vhci_device_descriptor *port_devices[VHCI_USB_MAX_CHILDREN];
	unsigned long port_status[VHCI_USB_MAX_CHILDREN];
	int pending_connection; // port number

	unsigned resuming:1;
	unsigned suspended:1;
	unsigned long re_timeout;
};


int usbd_vhci_device_init_module(void);
void usbd_vhci_device_deinit_module(void);

struct vhci_device_descriptor *usbd_vhci_device_create(void);
void usbd_vhci_device_destroy(struct vhci_device_descriptor *pdesc);

void usbd_vhci_device_connect(struct vhci_device_descriptor *pdesc);
void usbd_vhci_device_disconnect(struct vhci_device_descriptor *pdesc);

struct vhci_device_descriptor *usbd_vhci_device_find1(unsigned long hserver, unsigned long hdevice);
struct vhci_device_descriptor *usbd_vhci_device_find2(struct usb_hcd *hcd, int address);

struct waitable_queue_entry *usbd_vhci_device_dequeue_urb(struct vhci_device_descriptor *pdev, struct urb* purb);
void usbd_vhci_device_complete_urb(struct urb* purb, int status);
void usbd_vhci_device_cancel_all_urbs(struct vhci_device_descriptor *pdev);

#define usbd_vhci_device_reference(pdesc) kref_get(&(pdesc)->reference)
#define usbd_vhci_device_dereference(pdesc) kref_put(&(pdesc)->reference, usbd_vhci_device_free)
void usbd_vhci_device_free(struct kref* pkref);

#endif // _USBD_ENABLE_VHCI_

#endif // _VHCI_DEVICE_H
