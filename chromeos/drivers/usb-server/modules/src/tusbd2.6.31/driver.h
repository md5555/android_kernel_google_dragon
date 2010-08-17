/*
 *  tusbd/driver.h
 *
 *  Copyright (C) 2007-2008 IncentivesPro
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef _USBDRIVER_H
#define _USBDRIVER_H

extern struct minor_descriptor *g_pminorcontrol;
extern int g_major;
extern struct usb_driver usbd_driver;
extern int g_autosharing;
extern struct waitable_queue g_exclusions;

#endif // _USBDRIVER_H
