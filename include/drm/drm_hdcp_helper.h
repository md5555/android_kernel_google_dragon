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

#ifndef DRM_HDCP_HELPER_H
#define DRM_HDCP_HELPER_H

#define HDCP_BKSV 0x00
#define HDCP_RI 0x08
#define HDCP_AKSV 0x10
#define HDCP_INFO 0x15
#define  HDCP_INFO_1_1 (1 << 1)
#define HDCP_AN 0x18
#define HDCP_CAPS 0x40
#define  HDCP_CAPS_1_1 (1 << 1)

/**
 * struct drm_hdcp - HDCP helper
 * @client: I2C client representing the HDCP port (slave address 0x3a)
 */
struct drm_hdcp {
	struct i2c_client *client;
};

int drm_hdcp_register(struct drm_hdcp *hdcp, struct i2c_adapter *ddc);
void drm_hdcp_unregister(struct drm_hdcp *hdcp);

int drm_hdcp_read_caps(struct drm_hdcp *hdcp, u8 *caps);
int drm_hdcp_write_info(struct drm_hdcp *hdcp, u8 info);
int drm_hdcp_read_bksv(struct drm_hdcp *hdcp, u64 *bksv);
int drm_hdcp_write_an(struct drm_hdcp *hdcp, u64 an);
int drm_hdcp_write_aksv(struct drm_hdcp *hdcp, u64 aksv);
int drm_hdcp_read_ri(struct drm_hdcp *hdcp, u16 *ri);

#endif
