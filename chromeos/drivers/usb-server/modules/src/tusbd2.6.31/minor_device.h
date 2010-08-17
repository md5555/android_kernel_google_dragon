/*
 *  tusbd/minor_device.h
 *
 *  Copyright (C) 2007-2008 IncentivesPro
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef _MINOR_DEVICE_H
#define _MINOR_DEVICE_H

#ifdef _USBD_ENABLE_STUB_

#include "usbdcdev.h"

struct minor_descriptor *usbd_md_alloc_descriptor(struct usbdevice_descriptor *pusbdevice);
void usbd_md_free_descriptor(struct minor_descriptor *pdesc);

#endif // #ifdef _USBD_ENABLE_STUB_

#endif // _MINOR_DEVICE_H
