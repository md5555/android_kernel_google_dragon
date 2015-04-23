/*
 * Copyright (c) 2015 NVIDIA Corporation. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sub license,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial portions
 * of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <linux/i2c.h>
#include <linux/slab.h>

#include <drm/drm_hdcp_helper.h>

/**
 * DOC: hdcp helpers
 *
 * These functions contain common logic to deal with HDCP authentication.
 *
 * Helpers are provided for a number of standard accesses to HDCP receivers.

 */

/**
 * drm_hdcp_register - register a DHCP helper
 * @hdcp: DRM HDCP helper
 * @ddc: DDC I2C adapter
 *
 * Returns: 0 on success or a negative error code on failure.
 */
int drm_hdcp_register(struct drm_hdcp *hdcp, struct i2c_adapter *ddc)
{
	const struct i2c_board_info info = {
		.type = "hdcp",
		.addr = 0x3a,
	};

	hdcp->client = i2c_new_device(ddc, &info);
	if (!hdcp->client)
		return -ENODEV;

	return 0;
}
EXPORT_SYMBOL(drm_hdcp_register);

/**
 * drm_hdcp_unregister - unregister a DHCP helper
 * @hdcp: DRM HDCP helper
 */
void drm_hdcp_unregister(struct drm_hdcp *hdcp)
{
	i2c_unregister_device(hdcp->client);
}
EXPORT_SYMBOL(drm_hdcp_unregister);

static ssize_t drm_hdcp_read(struct drm_hdcp *hdcp, u8 offset, void *buffer,
			     size_t size)
{
	return i2c_smbus_read_i2c_block_data(hdcp->client, offset, size,
					     buffer);
}

static ssize_t drm_hdcp_write(struct drm_hdcp *hdcp, u8 offset,
			      const void *buffer, size_t size)
{
	return i2c_smbus_write_i2c_block_data(hdcp->client, offset, size,
					      buffer);
}

/**
 * drm_hdcp_read_caps - read an HDCP receiver's capabilities
 * @hdcp: DRM HDCP helper
 * @caps: return location for the capabilities
 *
 * Returns: The number of bytes read from the HDCP receiver on success or a
 * negative error code on failure.
 */
int drm_hdcp_read_caps(struct drm_hdcp *hdcp, u8 *caps)
{
	return drm_hdcp_read(hdcp, HDCP_CAPS, caps, 1);
}
EXPORT_SYMBOL(drm_hdcp_read_caps);

/**
 * drm_hdcp_write_info - write the HDCP transmitter info to the receiver
 * @hdcp: DRM HDCP helper
 * @info: the HDCP transmitter information
 *
 * Returns: The number of bytes written to the HDCP receiver on success or a
 * negative error code on failure.
 */
int drm_hdcp_write_info(struct drm_hdcp *hdcp, u8 info)
{
	return drm_hdcp_write(hdcp, HDCP_INFO, &info, 1);
}
EXPORT_SYMBOL(drm_hdcp_write_info);

/**
 * drm_hdcp_read_bksv - read the Bksv from the HDCP receiver
 * @hdcp: DRM HDCP helper
 * @bksv: return location for the HDCP receiver's Bksv
 *
 * Returns: The number of bytes read from the HDCP receiver on success or a
 * negative error code on failure.
 */
int drm_hdcp_read_bksv(struct drm_hdcp *hdcp, u64 *bksv)
{
	return drm_hdcp_read(hdcp, HDCP_BKSV, bksv, 5);
}
EXPORT_SYMBOL(drm_hdcp_read_bksv);

/**
 * drm_hdcp_write_an - write the HDCP session random number to the receiver
 * @hdcp: DRM HDCP helper
 * @an: the HDCP session random number
 *
 * Returns: The number of bytes written to the HDCP receiver on success or a
 * negative error code on failure.
 */
int drm_hdcp_write_an(struct drm_hdcp *hdcp, u64 an)
{
	return drm_hdcp_write(hdcp, HDCP_AN, &an, 8);
}
EXPORT_SYMBOL(drm_hdcp_write_an);

/**
 * drm_hdcp_write_aksv - write the HDCP transmitter's Aksv to the receiver
 * @hdcp: DRM HDCP helper
 * @aksv: the HDCP transmitter's Aksv
 *
 * Returns: The number of bytes written to the HDCP receiver on success or a
 * negative error code on failure.
 */
int drm_hdcp_write_aksv(struct drm_hdcp *hdcp, u64 aksv)
{
	return drm_hdcp_write(hdcp, HDCP_AKSV, &aksv, 5);
}
EXPORT_SYMBOL(drm_hdcp_write_aksv);

/**
 * drm_hdcp_read_ri - read the link verification response (Ri) from the
 *     receiver
 * @hdcp: DRM HDCP helper
 * @ri: return location for the HDCP receiver's link verification response
 *
 * Returns: The number of bytes read from the HDCP receiver on success or a
 * negative error code on failure.
 */
int drm_hdcp_read_ri(struct drm_hdcp *hdcp, u16 *ri)
{
	return drm_hdcp_read(hdcp, HDCP_RI, ri, 2);
}
EXPORT_SYMBOL(drm_hdcp_read_ri);
