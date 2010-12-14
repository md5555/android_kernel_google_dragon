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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/usb.h>
#include <net/bluetooth/bluetooth.h>

#define USB_REQ_DFU_DNLOAD	1
#define USB_REQ_GET_STATE	5
#define USB_FIRMWARE_RAM_MODE 11
#define USB_FIRMWARE_FLASH_MODE 12
#define BULK_SIZE		4096
#define VERSION "1.0"

struct firmware_data {
	struct usb_device *udev;
	u8 *fw_data;
	u32 fw_size;
	u32 fw_sent;
};

static int load_firmware(struct firmware_data *data,
				unsigned char *firmware,
				int count)
{
	u8 *send_buf;
	int err, pipe, len, size, sent = 0;
	char ucFirmware = 0;

	BT_DBG("ath3k %p udev %p", data, data->udev);

	if ((usb_control_msg(data->udev, usb_rcvctrlpipe(data->udev, 0),
				USB_REQ_GET_STATE,
				USB_TYPE_VENDOR | USB_DIR_IN, 0, 0,
				&ucFirmware, 1, USB_CTRL_SET_TIMEOUT)) < 0) {
		BT_ERR("Can't change to loading configuration err");
		return -EBUSY;
	}

	if (ucFirmware == USB_FIRMWARE_RAM_MODE) {
		/* RAM based firmware is available in the target.
		 * No need to load the firmware to RAM */
		BT_DBG("RAM based firmware is available");
		return 0;
	}

	pipe = usb_sndctrlpipe(data->udev, 0);
	if ((usb_control_msg(data->udev, pipe,
				USB_REQ_DFU_DNLOAD,
				USB_TYPE_VENDOR, 0, 0,
				firmware, 20, USB_CTRL_SET_TIMEOUT)) < 0) {
		BT_ERR("Can't change to loading configuration err");
		return -EBUSY;
	}
	sent += 20;
	count -= 20;

	send_buf = kmalloc(BULK_SIZE, GFP_ATOMIC);
	if (!send_buf) {
		BT_ERR("Can't allocate memory chunk for firmware");
		return -ENOMEM;
	}

	while (count) {
		size = min_t(uint, count, BULK_SIZE);
		pipe = usb_sndbulkpipe(data->udev, 0x02);
		memcpy(send_buf, firmware + sent, size);

		err = usb_bulk_msg(data->udev, pipe, send_buf, size,
					&len, 3000);

		if (err || (len != size)) {
			BT_ERR("Error in firmware loading err = %d,"
				"len = %d, size = %d", err, len, size);
			goto error;
		}

		sent  += size;
		count -= size;
	}

	kfree(send_buf);
	return 0;

error:
	kfree(send_buf);
	return err;
}

void *ath_fw_load(struct usb_interface *intf,
			const char *fwfile, bool *suspend)
{
	const struct firmware *firmware;
	struct usb_device *udev = interface_to_usbdev(intf);
	static struct firmware_data *data;
	int size;

	BT_DBG("\nintf %p suspend %d\n", intf, *suspend);

	if (*suspend) {
		load_firmware(data, data->fw_data, data->fw_size);
		*suspend = 0;
		return data;
	}

	if (intf->cur_altsetting->desc.bInterfaceNumber != 0)
		return NULL;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return NULL;
	data->udev = udev;

	if (request_firmware(&firmware, fwfile, &udev->dev) < 0) {
		kfree(data);
		return NULL;
	}

	size = max_t(uint, firmware->size, 4096);
	data->fw_data = kmalloc(size, GFP_KERNEL);
	if (!data->fw_data) {
		release_firmware(firmware);
		kfree(data);
		return NULL;
	}

	memcpy(data->fw_data, firmware->data, firmware->size);
	data->fw_size = firmware->size;
	data->fw_sent = 0;
	release_firmware(firmware);

	if (load_firmware(data, data->fw_data, data->fw_size)) {
		kfree(data->fw_data);
		kfree(data);
		return NULL;
	}
	return data;
}
EXPORT_SYMBOL(ath_fw_load);

void ath_fw_unload(void *pdata, bool bsuspend)
{
	struct firmware_data *data = (struct firmware_data *)pdata;

	if (data == NULL)
		return;

	/* do not free the data on suspend as we will
	 * use it on resume */
	if (!bsuspend) {
		kfree(data->fw_data);
		kfree(data);
	}
}
EXPORT_SYMBOL(ath_fw_unload);

static int __init fwload_init(void)
{
	BT_INFO("Firmware load driver init. Version:%s", VERSION);
	return 0;
}

static void __exit fwload_deinit(void)
{
	BT_INFO("Firmware load driver deinit");
}

module_init(fwload_init);
module_exit(fwload_deinit);

MODULE_AUTHOR("Atheros Communications");
MODULE_DESCRIPTION("Firmware load driver");
MODULE_VERSION(VERSION);
MODULE_LICENSE("GPL");
