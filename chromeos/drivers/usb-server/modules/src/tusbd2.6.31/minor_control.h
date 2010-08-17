/*
 *  tusbd/minor_control.h
 *
 *  Copyright (C) 2007-2008 IncentivesPro
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef _MINOR_CONTROL_H
#define _MINOR_CONTROL_H

#include "usbdcdev.h"

struct minor_descriptor *usbd_mc_alloc_descriptor(void);
void usbd_mc_free_descriptor(struct minor_descriptor *pdesc);

int usbd_mc_add_event(struct minor_descriptor *pdesc, const char *event);

#endif // _MINOR_CONTROL_H
