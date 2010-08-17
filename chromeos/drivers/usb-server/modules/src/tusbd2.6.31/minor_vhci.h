/*
 *  tusbd/minor_vhci.h
 *
 *  Copyright (C) 2007 IncentivesPro
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef _MINOR_VHCI_H
#define _MINOR_VHCI_H

#ifdef _USBD_ENABLE_VHCI_

#include "usbdcdev.h"

struct minor_descriptor *usbd_mv_alloc_descriptor(struct vhci_device_descriptor *pvhci_device);
void usbd_mv_free_descriptor(struct minor_descriptor *pdesc);

#endif // _USBD_ENABLE_VHCI_

#endif // _MINOR_VHCI_H
