/*
 *  tusbd/bind_unbind.h
 *
 *  Copyright (C) 2007-2008 IncentivesPro
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef _BIND_UNBIND_H
#define _BIND_UNBIND_H

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,13)

struct usb_device *usbd_find_usb_device_by_busid(const char *busid, struct usb_device *root_dev);
int usbd_bind_device_to_driver(struct usbdevice_descriptor *device, struct usb_driver *driver);

#endif // LINUX_VERSION_CODE < KERNEL_VERSION(2,6,13)

#endif // _BIND_UNBIND_H
