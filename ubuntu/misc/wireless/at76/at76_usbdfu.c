/* -*- linux-c -*- */
/*
 * USB Device Firmware Upgrade (DFU) handler
 *
 * Copyright (c) 2003 Oliver Kurth
 * Copyright (c) 2004 Jörg Albert
 *
 * This file is part of the driver for WLAN USB devices based on the Atmel
 * AT76C503A/505/505A. See at76c503.h for details.
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation; either version 2 of
 *	the License, or (at your option) any later version.
 *
 * 2003_01_19 0.1:
 * - initial release
 *
 * TODO:
 * (someday)
 * - make a way for drivers to feed firmware data at download time (instead of
 *   providing it all at once during register)
 * - procfs support for userland firmware downloaders
 * - Firmware upload (device-to-host) support
 */

#include <linux/slab.h>
#include <linux/usb.h>
#include "at76c503.h"

/* DFU states */

#define STATE_IDLE  			0x00
#define STATE_DETACH			0x01
#define STATE_DFU_IDLE			0x02
#define STATE_DFU_DOWNLOAD_SYNC		0x03
#define STATE_DFU_DOWNLOAD_BUSY		0x04
#define STATE_DFU_DOWNLOAD_IDLE		0x05
#define STATE_DFU_MANIFEST_SYNC		0x06
#define STATE_DFU_MANIFEST		0x07
#define STATE_DFU_MANIFEST_WAIT_RESET	0x08
#define STATE_DFU_UPLOAD_IDLE		0x09
#define STATE_DFU_ERROR			0x0a

/* DFU commands */
#define DFU_DETACH			0
#define DFU_DNLOAD			1
#define DFU_UPLOAD			2
#define DFU_GETSTATUS			3
#define DFU_CLRSTATUS			4
#define DFU_GETSTATE			5
#define DFU_ABORT			6

struct dfu_status {
	unsigned char bStatus;
	unsigned char bwPollTimeout[3];
	unsigned char bState;
	unsigned char iString;
} __attribute__ ((packed));


/* driver independent download context */
struct dfu_ctx {
	struct usb_device *udev;
	u8 dfu_state;
	struct dfu_status dfu_status;
	u8 *buf;
};

#define USB_SUCCESS(a) ((a) >= 0)

#define DFU_PACKETSIZE 1024

static
int dfu_download_block(struct dfu_ctx *ctx, u8 *buffer,
		       int bytes, int block)
{
	int result;
	u8 *tmpbuf = ctx->buf;
	struct usb_device *udev = ctx->udev;

	dbg(DBG_DFU, "dfu_download_block(): buffer=%p, bytes=%d, block=%d", buffer, bytes, block);

	if(tmpbuf == NULL)
		return -ENOMEM;

	memcpy(tmpbuf, buffer, bytes);

	result = usb_control_msg(udev, usb_sndctrlpipe(udev,0),
				 DFU_DNLOAD,
				 USB_TYPE_CLASS | USB_DIR_OUT | USB_RECIP_INTERFACE,
				 block,	/* Value */
				 0,	/* Index */
				 tmpbuf,	/* Buffer */
				 bytes,	/* Size */
				 HZ);
	return result;
}

static
int dfu_get_status(struct dfu_ctx *ctx, struct dfu_status *status)
{
	int result;
	struct usb_device *udev = ctx->udev;

//	dbg(DBG_DFU, "dfu_get_status()");

	result = usb_control_msg(udev, usb_rcvctrlpipe(udev, 0),
				 DFU_GETSTATUS,
				 USB_TYPE_CLASS | USB_DIR_IN | USB_RECIP_INTERFACE,
				 0,	/* Value */
				 0,	/* Index */
				 status,	/* Buffer */
				 sizeof(struct dfu_status),	/* Size */
				 HZ);

	return result;
}

static
u8 dfu_get_state(struct usb_device *udev, u8 *state)
{
	int result;

//	dbg(DBG_DFU, "dfu_get_state()");

	result = usb_control_msg(udev, usb_rcvctrlpipe(udev, 0),
				 DFU_GETSTATE,	/* Request */
				 USB_TYPE_CLASS | USB_DIR_IN | USB_RECIP_INTERFACE,
				 0,	/* Value */
				 0,	/* Index */
				 state,	/* Buffer */
				 1,	/* Size */
				 HZ);

	return result;
}

static inline
u32 __get_timeout(struct dfu_status *s)
{
	unsigned long ret = 0;

	ret  = (unsigned long) (s->bwPollTimeout[2] << 16);
	ret |= (unsigned long) (s->bwPollTimeout[1] << 8);
	ret |= (unsigned long) (s->bwPollTimeout[0]);

	return ret;
}

static
struct dfu_ctx *dfu_alloc_ctx(struct usb_device *udev)
{
	struct dfu_ctx *ctx;

	ctx = kmalloc(sizeof(struct dfu_ctx) + DFU_PACKETSIZE, GFP_KERNEL|GFP_DMA);
	if(ctx){
		ctx->udev = udev;
		ctx->buf = (u8 *)&(ctx[1]);
	}
	return ctx;
}

/* == PROC usbdfu_download ==
   if manifest_sync_timeout > 0 use this timeout (in msec) instead of the
   one reported by the device in state MANIFEST_SYNC */
int usbdfu_download(struct usb_device *udev, u8 *dfu_buffer, u32 dfu_len, 
		    int manifest_sync_timeout)
{
	struct dfu_ctx *ctx;
	struct dfu_status *dfu_stat_buf;
	int status = 0;
	int need_dfu_state = 1;
	int is_done = 0;
	u8 dfu_state = 0;
	u32 dfu_timeout = 0;
	int dfu_block_bytes = 0, dfu_bytes_left = dfu_len, dfu_buffer_offset = 0;
	int dfu_block_cnt = 0;

	dbg(DBG_DFU, "%s( %p, %u, %d)", __FUNCTION__, dfu_buffer, 
	    dfu_len, manifest_sync_timeout);

	if (dfu_len == 0) {
		err("FW Buffer length invalid!");
		return -EINVAL;
	}

	ctx = dfu_alloc_ctx(udev);
	if(ctx == NULL)
		return -ENOMEM;

	dfu_stat_buf = &ctx->dfu_status;

	do {
		if (need_dfu_state) {
			status = dfu_get_state(ctx->udev, &ctx->dfu_state);
			if (!USB_SUCCESS(status)) {
				err("DFU: Failed to get DFU state: %d", status);
				goto exit;
			}
			dfu_state = ctx->dfu_state;
			need_dfu_state = 0;
		}

		switch (dfu_state) {
		case STATE_DFU_DOWNLOAD_SYNC:
			dbg(DBG_DFU, "STATE_DFU_DOWNLOAD_SYNC");
			status = dfu_get_status(ctx, dfu_stat_buf);
			if (USB_SUCCESS(status)) {
				dfu_state = dfu_stat_buf->bState;
				dfu_timeout = __get_timeout(dfu_stat_buf);
				need_dfu_state = 0;
			} else
				err("dfu_get_status failed with %d", status);
			break;

		case STATE_DFU_DOWNLOAD_BUSY:
			dbg(DBG_DFU, "STATE_DFU_DOWNLOAD_BUSY");
			need_dfu_state = 1;

			if (dfu_timeout >= 0){
				dbg(DBG_DFU, "DFU: Resetting device");
				set_current_state( TASK_INTERRUPTIBLE );
				schedule_timeout(1+dfu_timeout*HZ/1000);
			}else
				dbg(DBG_DFU, "DFU: In progress");

			break;

		case STATE_DFU_DOWNLOAD_IDLE:
			dbg(DBG_DFU, "DOWNLOAD...");
			/* fall through */
		case STATE_DFU_IDLE:
			dbg(DBG_DFU, "DFU IDLE");

			if (dfu_bytes_left <= DFU_PACKETSIZE)
				dfu_block_bytes = dfu_bytes_left;
			else
				dfu_block_bytes = DFU_PACKETSIZE;

			dfu_bytes_left -= dfu_block_bytes;
			status = dfu_download_block(ctx,
					      dfu_buffer +
					      dfu_buffer_offset,
					      dfu_block_bytes,
					      dfu_block_cnt);
			dfu_buffer_offset += dfu_block_bytes;
			dfu_block_cnt++;

			if (!USB_SUCCESS(status))
				err("dfu_download_block failed with %d", status);
			need_dfu_state = 1;
			break;

		case STATE_DFU_MANIFEST_SYNC:
			dbg(DBG_DFU, "STATE_DFU_MANIFEST_SYNC");

			status = dfu_get_status(ctx, dfu_stat_buf);

			if (USB_SUCCESS(status)) {
				dfu_state = dfu_stat_buf->bState;
				dfu_timeout = __get_timeout(dfu_stat_buf);
				need_dfu_state = 0;

				/* override the timeout from the status response,
				   needed for AT76C505A */
				if (manifest_sync_timeout > 0)
					dfu_timeout = manifest_sync_timeout;

				if (dfu_timeout >= 0){
					dbg(DBG_DFU, "DFU: Waiting for manifest phase");

					set_current_state( TASK_INTERRUPTIBLE );
					schedule_timeout((dfu_timeout*HZ+999)/1000);
				}else
					dbg(DBG_DFU, "DFU: In progress");
			}
			break;

		case STATE_DFU_MANIFEST:
			dbg(DBG_DFU, "STATE_DFU_MANIFEST");
			is_done = 1;
			break;

		case STATE_DFU_MANIFEST_WAIT_RESET:
			dbg(DBG_DFU, "STATE_DFU_MANIFEST_WAIT_RESET");
//			usb_reset_device(udev);
			is_done = 1;
			break;

		case STATE_DFU_UPLOAD_IDLE:
			dbg(DBG_DFU, "STATE_DFU_UPLOAD_IDLE");
			break;

		case STATE_DFU_ERROR:
			dbg(DBG_DFU, "STATE_DFU_ERROR");
//			usb_reset_device(udev);
			status = -EPIPE;
			break;

		default:
			dbg(DBG_DFU, "DFU UNKNOWN STATE (%d)", dfu_state);
			status = -EINVAL;
			break;
		}
	} while (!is_done && USB_SUCCESS(status));

 exit:
	kfree(ctx);
	if (status < 0)
		return status;
	else
		return 0;
}

