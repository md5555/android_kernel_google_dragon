/*
 *  tusbd/usbdevice.h
 *
 *  Copyright (C) 2007-2008 IncentivesPro
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef _USBDEVICE_H
#define _USBDEVICE_H

#ifdef _USBD_ENABLE_STUB_
#ifndef BUS_ID_SIZE
#define BUS_ID_SIZE 20
#endif


struct usbdevice_descriptor
{
	struct list_head list;
	struct minor_descriptor *pminor;
	struct kref reference;
	u32 handle;
	u16 vid;
	u16 pid;
	char bus_id[BUS_ID_SIZE];
	eDeviceState state;
	struct usb_device* pudev;
	spinlock_t pudev_lock;
	int is_ehci;
	int total_interfaces;
	int init_interfaces; // number of attached and initialized interfaces
	int config_changed;
	int frame_delta_in[16];
	int frame_delta_out[16];
	int iso_counter_in[16];
	int iso_counter_out[16];
	struct waitable_queue queue_completed_unrb; // queue of completed UNRBs
	struct waitable_queue queue_pending_unrb; // queue of completed UNRBs

	wait_queue_head_t config_change_wait;
};

struct usbd_exclusion
{
	u32 handle;
	u16 vid;
	u16 pid;
	char bus_id[BUS_ID_SIZE];
};

int usbd_usbdevice_init_module(void);
void usbd_usbdevice_deinit_module(void);

struct usbdevice_descriptor *usbd_usbdevice_create(void);
void usbd_usbdevice_destroy(struct usbdevice_descriptor *pdesc);

struct usbdevice_descriptor *usbd_usbdevice_find1(unsigned short vid, unsigned short pid, char *bus_id);
struct usbdevice_descriptor *usbd_usbdevice_find2(unsigned long handle);
struct usbdevice_descriptor *usbd_usbdevice_find3(int minor);

int usbd_usbdevice_refresh_state(struct usbdevice_descriptor *pdesc);
int usbd_usbdevice_change_state(struct usbdevice_descriptor *pdesc, eDeviceState newstate);
void usbd_usbdevice_reset(struct usbdevice_descriptor *pdesc, struct usb_device *pudev);
int usbd_usbdevice_change_configuration(struct usbdevice_descriptor *pdesc, struct usb_device *pudev, int configuration, int timeoutms);

int usbd_usbdevice_inc_iso_counter(struct usbdevice_descriptor *pdesc, int pipe);
int usbd_usbdevice_dec_iso_counter(struct usbdevice_descriptor *pdesc, int pipe);
void usbd_usbdevice_reset_iso_counter(struct usbdevice_descriptor *pdesc, int pipe);
int usbd_usbdevice_get_frame_delta(struct usbdevice_descriptor *pdesc, int pipe);
void usbd_usbdevice_set_frame_delta(struct usbdevice_descriptor *pdesc, int pipe, int frame_delta);

#define usbd_usbdevice_reference(pdesc) {if((pdesc)) kref_get(&(pdesc)->reference);}
#define usbd_usbdevice_dereference(pdesc) {if((pdesc)) kref_put(&(pdesc)->reference, usbd_usbdevice_free);}
void usbd_usbdevice_free(struct kref* pkref);

struct usb_device *usbd_usbdevice_pudev_get(struct usbdevice_descriptor *pdesc);
void usbd_usbdevice_pudev_put(struct usbdevice_descriptor *pdesc, struct usb_device *pudev);

#endif // #ifdef _USBD_ENABLE_STUB_

#endif // _USBDEVICE_H
