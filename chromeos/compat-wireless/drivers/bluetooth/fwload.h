/*
 *
 *  Generic Bluetooth USB DFU driver to download firmware to target RAM
 *
 *  Copyright (c) 2009-2010 Atheros Communications Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#ifndef _FWLOAD_H_
#define _FWLOAD_H_

/* callbacks to load firmware to BT device RAM
 * when it is inserted */
struct fw_cb_config {
	const char *fwfile;
	void * (*fwload)(struct usb_interface *intf, const char *fwfile,
			 bool *bsuspend);
	void (*fwunload)(void *, bool);
	const struct usb_device_id *usb_id_table;
	void *data;
	bool bsuspend;
};
void *ath_fw_load(struct usb_interface *intf, const char *, bool *);
void ath_fw_unload(void *pdata, bool bsuspend);

#endif /* _FWLOAD_H_ */
