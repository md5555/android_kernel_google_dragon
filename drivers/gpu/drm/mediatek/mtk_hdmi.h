/*
 * Copyright (c) 2014 MediaTek Inc.
 * Author: Jie Qiu <jie.qiu@mediatek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef _MTK_HDMI_CTRL_H
#define _MTK_HDMI_CTRL_H

#include <drm/drm_crtc.h>
#include <linux/hdmi.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/types.h>

struct clk;
struct device;
struct device_node;
struct i2c_adapter;
struct platform_device;
struct phy;
struct regmap;

enum mtk_hdmi_clk_id {
	MTK_HDMI_CLK_HDMI_PIXEL,
	MTK_HDMI_CLK_HDMI_PLL,
	MTK_HDMI_CLK_AUD_BCLK,
	MTK_HDMI_CLK_AUD_SPDIF,
	MTK_HDMI_CLK_COUNT
};

enum hdmi_aud_input_type {
	HDMI_AUD_INPUT_I2S = 0,
	HDMI_AUD_INPUT_SPDIF,
};

enum hdmi_aud_i2s_fmt {
	HDMI_I2S_MODE_RJT_24BIT = 0,
	HDMI_I2S_MODE_RJT_16BIT,
	HDMI_I2S_MODE_LJT_24BIT,
	HDMI_I2S_MODE_LJT_16BIT,
	HDMI_I2S_MODE_I2S_24BIT,
	HDMI_I2S_MODE_I2S_16BIT
};

enum hdmi_aud_mclk {
	HDMI_AUD_MCLK_128FS,
	HDMI_AUD_MCLK_192FS,
	HDMI_AUD_MCLK_256FS,
	HDMI_AUD_MCLK_384FS,
	HDMI_AUD_MCLK_512FS,
	HDMI_AUD_MCLK_768FS,
	HDMI_AUD_MCLK_1152FS,
};

enum hdmi_aud_iec_frame_rate {
	HDMI_IEC_32K = 0,
	HDMI_IEC_96K,
	HDMI_IEC_192K,
	HDMI_IEC_768K,
	HDMI_IEC_44K,
	HDMI_IEC_88K,
	HDMI_IEC_176K,
	HDMI_IEC_705K,
	HDMI_IEC_16K,
	HDMI_IEC_22K,
	HDMI_IEC_24K,
	HDMI_IEC_48K,
};

enum hdmi_aud_channel_type {
	HDMI_AUD_CHAN_TYPE_1_0 = 0,
	HDMI_AUD_CHAN_TYPE_1_1,
	HDMI_AUD_CHAN_TYPE_2_0,
	HDMI_AUD_CHAN_TYPE_2_1,
	HDMI_AUD_CHAN_TYPE_3_0,
	HDMI_AUD_CHAN_TYPE_3_1,
	HDMI_AUD_CHAN_TYPE_4_0,
	HDMI_AUD_CHAN_TYPE_4_1,
	HDMI_AUD_CHAN_TYPE_5_0,
	HDMI_AUD_CHAN_TYPE_5_1,
	HDMI_AUD_CHAN_TYPE_6_0,
	HDMI_AUD_CHAN_TYPE_6_1,
	HDMI_AUD_CHAN_TYPE_7_0,
	HDMI_AUD_CHAN_TYPE_7_1,
	HDMI_AUD_CHAN_TYPE_3_0_LRS,
	HDMI_AUD_CHAN_TYPE_3_1_LRS,
	HDMI_AUD_CHAN_TYPE_4_0_CLRS,
	HDMI_AUD_CHAN_TYPE_4_1_CLRS,
	HDMI_AUD_CHAN_TYPE_6_1_CS,
	HDMI_AUD_CHAN_TYPE_6_1_CH,
	HDMI_AUD_CHAN_TYPE_6_1_OH,
	HDMI_AUD_CHAN_TYPE_6_1_CHR,
	HDMI_AUD_CHAN_TYPE_7_1_LH_RH,
	HDMI_AUD_CHAN_TYPE_7_1_LSR_RSR,
	HDMI_AUD_CHAN_TYPE_7_1_LC_RC,
	HDMI_AUD_CHAN_TYPE_7_1_LW_RW,
	HDMI_AUD_CHAN_TYPE_7_1_LSD_RSD,
	HDMI_AUD_CHAN_TYPE_7_1_LSS_RSS,
	HDMI_AUD_CHAN_TYPE_7_1_LHS_RHS,
	HDMI_AUD_CHAN_TYPE_7_1_CS_CH,
	HDMI_AUD_CHAN_TYPE_7_1_CS_OH,
	HDMI_AUD_CHAN_TYPE_7_1_CS_CHR,
	HDMI_AUD_CHAN_TYPE_7_1_CH_OH,
	HDMI_AUD_CHAN_TYPE_7_1_CH_CHR,
	HDMI_AUD_CHAN_TYPE_7_1_OH_CHR,
	HDMI_AUD_CHAN_TYPE_7_1_LSS_RSS_LSR_RSR,
	HDMI_AUD_CHAN_TYPE_6_0_CS,
	HDMI_AUD_CHAN_TYPE_6_0_CH,
	HDMI_AUD_CHAN_TYPE_6_0_OH,
	HDMI_AUD_CHAN_TYPE_6_0_CHR,
	HDMI_AUD_CHAN_TYPE_7_0_LH_RH,
	HDMI_AUD_CHAN_TYPE_7_0_LSR_RSR,
	HDMI_AUD_CHAN_TYPE_7_0_LC_RC,
	HDMI_AUD_CHAN_TYPE_7_0_LW_RW,
	HDMI_AUD_CHAN_TYPE_7_0_LSD_RSD,
	HDMI_AUD_CHAN_TYPE_7_0_LSS_RSS,
	HDMI_AUD_CHAN_TYPE_7_0_LHS_RHS,
	HDMI_AUD_CHAN_TYPE_7_0_CS_CH,
	HDMI_AUD_CHAN_TYPE_7_0_CS_OH,
	HDMI_AUD_CHAN_TYPE_7_0_CS_CHR,
	HDMI_AUD_CHAN_TYPE_7_0_CH_OH,
	HDMI_AUD_CHAN_TYPE_7_0_CH_CHR,
	HDMI_AUD_CHAN_TYPE_7_0_OH_CHR,
	HDMI_AUD_CHAN_TYPE_7_0_LSS_RSS_LSR_RSR,
	HDMI_AUD_CHAN_TYPE_8_0_LH_RH_CS,
	HDMI_AUD_CHAN_TYPE_UNKNOWN = 0xFF
};

enum hdmi_aud_channel_swap_type {
	HDMI_AUD_SWAP_LR,
	HDMI_AUD_SWAP_LFE_CC,
	HDMI_AUD_SWAP_LSRS,
	HDMI_AUD_SWAP_RLS_RRS,
	HDMI_AUD_SWAP_LR_STATUS,
};

struct hdmi_audio_param {
	enum hdmi_audio_coding_type aud_codec;
	enum hdmi_audio_sample_frequency aud_hdmi_fs;
	enum hdmi_audio_sample_size aud_sampe_size;
	enum hdmi_aud_input_type aud_input_type;
	enum hdmi_aud_i2s_fmt aud_i2s_fmt;
	enum hdmi_aud_mclk aud_mclk;
	enum hdmi_aud_iec_frame_rate iec_frame_fs;
	enum hdmi_aud_channel_type aud_input_chan_type;
	u8 hdmi_l_channel_state[6];
	u8 hdmi_r_channel_state[6];
};

struct mtk_hdmi {
	struct drm_bridge bridge;
	struct drm_connector conn;
	struct device *dev;
	struct phy *phy;
	struct device *cec_dev;
	struct i2c_adapter *ddc_adpt;
	struct clk *clk[MTK_HDMI_CLK_COUNT];
#if defined(CONFIG_DEBUG_FS)
	struct dentry *debugfs;
#endif
	struct platform_device *audio_pdev;
	struct drm_display_mode mode;
	bool dvi_mode;
	u32 min_clock;
	u32 max_clock;
	u32 max_hdisplay;
	u32 max_vdisplay;
	u32 ibias;
	u32 ibias_up;
	struct regmap *sys_regmap;
	unsigned int sys_offset;
	void __iomem *regs;
	bool init;
	enum hdmi_colorspace csp;
	bool audio_enable;
	bool output;
	struct hdmi_audio_param aud_param;
};

static inline struct mtk_hdmi *hdmi_ctx_from_bridge(struct drm_bridge *b)
{
	return container_of(b, struct mtk_hdmi, bridge);
}

static inline struct mtk_hdmi *hdmi_ctx_from_conn(struct drm_connector *c)
{
	return container_of(c, struct mtk_hdmi, conn);
}

int mtk_hdmi_output_init(struct mtk_hdmi *hdmi);
int mtk_hdmi_hpd_high(struct mtk_hdmi *hdmi);
int mtk_hdmi_output_set_display_mode(struct mtk_hdmi *hdmi,
				     struct drm_display_mode *mode);
void mtk_hdmi_power_on(struct mtk_hdmi *hdmi);
void mtk_hdmi_power_off(struct mtk_hdmi *hdmi);
#if defined(CONFIG_DEBUG_FS)
int mtk_drm_hdmi_debugfs_init(struct mtk_hdmi *hdmi);
void mtk_drm_hdmi_debugfs_exit(struct mtk_hdmi *hdmi);
#else
int mtk_drm_hdmi_debugfs_init(struct mtk_hdmi *hdmi)
{
	return 0;
}

void mtk_drm_hdmi_debugfs_exit(struct mtk_hdmi *hdmi)
{
}
#endif /* CONFIG_DEBUG_FS */

extern struct platform_driver mtk_cec_driver;
extern struct platform_driver mtk_hdmi_ddc_driver;
extern struct platform_driver mtk_hdmi_phy_driver;
#endif /* _MTK_HDMI_CTRL_H */
