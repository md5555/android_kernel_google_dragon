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
#include "mtk_hdmi_hw.h"
#include "mtk_hdmi_regs.h"
#include "mtk_hdmi.h"

#include <linux/arm-smccc.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/hdmi.h>
#include <linux/io.h>
#include <linux/regmap.h>

static u32 mtk_hdmi_read(struct mtk_hdmi *hdmi, u32 offset)
{
	return readl(hdmi->regs + offset);
}

static void mtk_hdmi_write(struct mtk_hdmi *hdmi, u32 offset, u32 val)
{
	writel(val, hdmi->regs + offset);
}

static void mtk_hdmi_mask(struct mtk_hdmi *hdmi, u32 offset, u32 val, u32 mask)
{
	u32 tmp = mtk_hdmi_read(hdmi, offset) & ~mask;

	tmp |= (val & mask);
	mtk_hdmi_write(hdmi, offset, tmp);
}

#define NCTS_BYTES          0x07

void mtk_hdmi_hw_vid_black(struct mtk_hdmi *hdmi,
			   bool black)
{
	mtk_hdmi_mask(hdmi, VIDEO_CFG_4, black ? GEN_RGB : NORMAL_PATH,
		      VIDEO_SOURCE_SEL);
}

void mtk_hdmi_hw_make_reg_writable(struct mtk_hdmi *hdmi, bool enable)
{
	struct arm_smccc_res res;

	/*
	 * MT8173 HDMI hardware has an output control bit to enable/disable HDMI
	 * output. This bit can only be controlled in ARM supervisor mode.
	 * The ARM trusted firmware provides an API for the HDMI driver to set
	 * this control bit to enable HDMI output in supervisor mode.
	 */
	arm_smccc_smc(MTK_SIP_SET_AUTHORIZED_SECURE_REG, 0x14000904, 0x80000000,
		      0, 0, 0, 0, 0, &res);

	regmap_update_bits(hdmi->sys_regmap, hdmi->sys_offset + HDMI_SYS_CFG20,
			   HDMI_PCLK_FREE_RUN, enable ? HDMI_PCLK_FREE_RUN : 0);
	regmap_update_bits(hdmi->sys_regmap, hdmi->sys_offset + HDMI_SYS_CFG1C,
			   HDMI_ON | ANLG_ON, enable ? (HDMI_ON | ANLG_ON) : 0);
}

void mtk_hdmi_hw_1p4_version_enable(struct mtk_hdmi *hdmi, bool enable)
{
	regmap_update_bits(hdmi->sys_regmap, hdmi->sys_offset + HDMI_SYS_CFG20,
			   HDMI2P0_EN, enable ? 0 : HDMI2P0_EN);
}

void mtk_hdmi_hw_aud_mute(struct mtk_hdmi *hdmi, bool mute)
{
	mtk_hdmi_mask(hdmi, GRL_AUDIO_CFG, mute ? AUDIO_ZERO : 0, AUDIO_ZERO);
}

void mtk_hdmi_hw_reset(struct mtk_hdmi *hdmi)
{
	regmap_update_bits(hdmi->sys_regmap, hdmi->sys_offset + HDMI_SYS_CFG1C,
			   HDMI_RST, HDMI_RST);
	regmap_update_bits(hdmi->sys_regmap, hdmi->sys_offset + HDMI_SYS_CFG1C,
			   HDMI_RST, 0);
	mtk_hdmi_mask(hdmi, GRL_CFG3, 0, CFG3_CONTROL_PACKET_DELAY);
	regmap_update_bits(hdmi->sys_regmap, hdmi->sys_offset + HDMI_SYS_CFG1C,
			   ANLG_ON, ANLG_ON);
}

void mtk_hdmi_hw_enable_notice(struct mtk_hdmi *hdmi, bool enable_notice)
{
	mtk_hdmi_mask(hdmi, GRL_CFG2, enable_notice ? CFG2_NOTICE_EN : 0,
		      CFG2_NOTICE_EN);
}

void mtk_hdmi_hw_write_int_mask(struct mtk_hdmi *hdmi, u32 int_mask)
{
	mtk_hdmi_write(hdmi, GRL_INT_MASK, int_mask);
}

void mtk_hdmi_hw_enable_dvi_mode(struct mtk_hdmi *hdmi, bool enable)
{
	mtk_hdmi_mask(hdmi, GRL_CFG1, enable ? CFG1_DVI : 0, CFG1_DVI);
}

void mtk_hdmi_hw_send_info_frame(struct mtk_hdmi *hdmi, u8 *buffer, u8 len)
{
	u32 ctrl_reg = GRL_CTRL;
	int i;
	u8 *frame_data;
	u8 frame_type;
	u8 frame_ver;
	u8 frame_len;
	u8 checksum;
	int ctrl_frame_en = 0;

	frame_type = *buffer;
	buffer += 1;
	frame_ver = *buffer;
	buffer += 1;
	frame_len = *buffer;
	buffer += 1;
	checksum = *buffer;
	buffer += 1;
	frame_data = buffer;

	dev_dbg(hdmi->dev,
		"frame_type:0x%x,frame_ver:0x%x,frame_len:0x%x,checksum:0x%x\n",
		frame_type, frame_ver, frame_len, checksum);

	switch (frame_type) {
	case HDMI_INFOFRAME_TYPE_AVI:
		ctrl_frame_en = CTRL_AVI_EN;
		ctrl_reg = GRL_CTRL;
		break;
	case HDMI_INFOFRAME_TYPE_SPD:
		ctrl_frame_en = CTRL_SPD_EN;
		ctrl_reg = GRL_CTRL;
		break;
	case HDMI_INFOFRAME_TYPE_AUDIO:
		ctrl_frame_en = CTRL_AUDIO_EN;
		ctrl_reg = GRL_CTRL;
		break;
	case HDMI_INFOFRAME_TYPE_VENDOR:
		ctrl_frame_en = VS_EN;
		ctrl_reg = GRL_ACP_ISRC_CTRL;
		break;
	default:
		break;
	}
	mtk_hdmi_mask(hdmi, ctrl_reg, 0, ctrl_frame_en);
	mtk_hdmi_write(hdmi, GRL_INFOFRM_TYPE, frame_type);
	mtk_hdmi_write(hdmi, GRL_INFOFRM_VER, frame_ver);
	mtk_hdmi_write(hdmi, GRL_INFOFRM_LNG, frame_len);

	mtk_hdmi_write(hdmi, GRL_IFM_PORT, checksum);
	for (i = 0; i < frame_len; i++)
		mtk_hdmi_write(hdmi, GRL_IFM_PORT, frame_data[i]);

	mtk_hdmi_mask(hdmi, ctrl_reg, ctrl_frame_en, ctrl_frame_en);
}

void mtk_hdmi_hw_send_aud_packet(struct mtk_hdmi *hdmi, bool enable)
{
	mtk_hdmi_mask(hdmi, GRL_SHIFT_R2, enable ? 0 : AUDIO_PACKET_OFF,
		      AUDIO_PACKET_OFF);
}

void mtk_hdmi_hw_config_sys(struct mtk_hdmi *hdmi)
{
	regmap_update_bits(hdmi->sys_regmap, hdmi->sys_offset + HDMI_SYS_CFG20,
			   HDMI_OUT_FIFO_EN | MHL_MODE_ON, 0);
	mdelay(2);
	regmap_update_bits(hdmi->sys_regmap, hdmi->sys_offset + HDMI_SYS_CFG20,
			   HDMI_OUT_FIFO_EN | MHL_MODE_ON, HDMI_OUT_FIFO_EN);
}

void mtk_hdmi_hw_set_deep_color_mode(struct mtk_hdmi *hdmi)
{
	regmap_update_bits(hdmi->sys_regmap, hdmi->sys_offset + HDMI_SYS_CFG20,
			   DEEP_COLOR_MODE_MASK | DEEP_COLOR_EN, COLOR_8BIT_MODE);
}

void mtk_hdmi_hw_send_av_mute(struct mtk_hdmi *hdmi)
{
	mtk_hdmi_mask(hdmi, GRL_CFG4, 0, CTRL_AVMUTE);
	mdelay(2);
	mtk_hdmi_mask(hdmi, GRL_CFG4, CTRL_AVMUTE, CTRL_AVMUTE);
}

void mtk_hdmi_hw_send_av_unmute(struct mtk_hdmi *hdmi)
{
	mtk_hdmi_mask(hdmi, GRL_CFG4, CFG4_AV_UNMUTE_EN,
		      CFG4_AV_UNMUTE_EN | CFG4_AV_UNMUTE_SET);
	mdelay(2);
	mtk_hdmi_mask(hdmi, GRL_CFG4, CFG4_AV_UNMUTE_SET,
		      CFG4_AV_UNMUTE_EN | CFG4_AV_UNMUTE_SET);
}

void mtk_hdmi_hw_ncts_enable(struct mtk_hdmi *hdmi, bool on)
{
	mtk_hdmi_mask(hdmi, GRL_CTS_CTRL, on ? 0 : CTS_CTRL_SOFT,
		      CTS_CTRL_SOFT);
}

void mtk_hdmi_hw_ncts_auto_write_enable(struct mtk_hdmi *hdmi, bool enable)
{
	mtk_hdmi_mask(hdmi, GRL_CTS_CTRL, enable ? NCTS_WRI_ANYTIME : 0,
		      NCTS_WRI_ANYTIME);
}

void mtk_hdmi_hw_msic_setting(struct mtk_hdmi *hdmi,
			      struct drm_display_mode *mode)
{
	mtk_hdmi_mask(hdmi, GRL_CFG4, 0, CFG_MHL_MODE);

	if (mode->flags & DRM_MODE_FLAG_INTERLACE &&
	    mode->clock == 74250 &&
	    mode->vdisplay == 1080)
		mtk_hdmi_mask(hdmi, GRL_CFG2, 0, MHL_DE_SEL);
	else
		mtk_hdmi_mask(hdmi, GRL_CFG2, MHL_DE_SEL, MHL_DE_SEL);
}

void mtk_hdmi_hw_aud_set_channel_swap(struct mtk_hdmi *hdmi,
				      enum hdmi_aud_channel_swap_type swap)
{
	u8 swap_bit;

	switch (swap) {
	case HDMI_AUD_SWAP_LR:
		swap_bit = LR_SWAP;
		break;
	case HDMI_AUD_SWAP_LFE_CC:
		swap_bit = LFE_CC_SWAP;
		break;
	case HDMI_AUD_SWAP_LSRS:
		swap_bit = LSRS_SWAP;
		break;
	case HDMI_AUD_SWAP_RLS_RRS:
		swap_bit = RLS_RRS_SWAP;
		break;
	case HDMI_AUD_SWAP_LR_STATUS:
		swap_bit = LR_STATUS_SWAP;
		break;
	default:
		swap_bit = LFE_CC_SWAP;
		break;
	}
	mtk_hdmi_mask(hdmi, GRL_CH_SWAP, swap_bit, 0xff);
}

void mtk_hdmi_hw_aud_raw_data_enable(struct mtk_hdmi *hdmi, bool enable)
{
	mtk_hdmi_mask(hdmi, GRL_MIX_CTRL, enable ? MIX_CTRL_FLAT : 0,
		      MIX_CTRL_FLAT);
}

void mtk_hdmi_hw_aud_set_bit_num(struct mtk_hdmi *hdmi,
				 enum hdmi_audio_sample_size bit_num)
{
	u32 val = 0;

	if (bit_num == HDMI_AUDIO_SAMPLE_SIZE_16)
		val = AOUT_16BIT;
	else if (bit_num == HDMI_AUDIO_SAMPLE_SIZE_20)
		val = AOUT_20BIT;
	else if (bit_num == HDMI_AUDIO_SAMPLE_SIZE_24)
		val = AOUT_24BIT;

	mtk_hdmi_mask(hdmi, GRL_AOUT_BNUM_SEL, val, 0x03);
}

void mtk_hdmi_hw_aud_set_i2s_fmt(struct mtk_hdmi *hdmi,
				 enum hdmi_aud_i2s_fmt i2s_fmt)
{
	u32 val = 0;

	val = mtk_hdmi_read(hdmi, GRL_CFG0);
	val &= ~0x33;

	switch (i2s_fmt) {
	case HDMI_I2S_MODE_RJT_24BIT:
		val |= (CFG0_I2S_MODE_RTJ | CFG0_I2S_MODE_24BIT);
		break;
	case HDMI_I2S_MODE_RJT_16BIT:
		val |= (CFG0_I2S_MODE_RTJ | CFG0_I2S_MODE_16BIT);
		break;
	case HDMI_I2S_MODE_LJT_24BIT:
		val |= (CFG0_I2S_MODE_LTJ | CFG0_I2S_MODE_24BIT);
		break;
	case HDMI_I2S_MODE_LJT_16BIT:
		val |= (CFG0_I2S_MODE_LTJ | CFG0_I2S_MODE_16BIT);
		break;
	case HDMI_I2S_MODE_I2S_24BIT:
		val |= (CFG0_I2S_MODE_I2S | CFG0_I2S_MODE_24BIT);
		break;
	case HDMI_I2S_MODE_I2S_16BIT:
		val |= (CFG0_I2S_MODE_I2S | CFG0_I2S_MODE_16BIT);
		break;
	default:
		break;
	}
	mtk_hdmi_write(hdmi, GRL_CFG0, val);
}

void mtk_hdmi_hw_aud_set_high_bitrate(struct mtk_hdmi *hdmi, bool enable)
{
	mtk_hdmi_mask(hdmi, GRL_AOUT_BNUM_SEL,
		      enable ? HIGH_BIT_RATE_PACKET_ALIGN : 0,
		      HIGH_BIT_RATE_PACKET_ALIGN);
	mtk_hdmi_mask(hdmi, GRL_AUDIO_CFG, enable ? HIGH_BIT_RATE : 0,
		      HIGH_BIT_RATE);
}

void mtk_hdmi_phy_aud_dst_normal_double_enable(struct mtk_hdmi *hdmi,
					       bool enable)
{
	mtk_hdmi_mask(hdmi, GRL_AUDIO_CFG, enable ? DST_NORMAL_DOUBLE : 0,
		      DST_NORMAL_DOUBLE);
}

void mtk_hdmi_hw_aud_dst_enable(struct mtk_hdmi *hdmi, bool enable)
{
	mtk_hdmi_mask(hdmi, GRL_AUDIO_CFG, enable ? SACD_DST : 0, SACD_DST);
}

void mtk_hdmi_hw_aud_dsd_enable(struct mtk_hdmi *hdmi, bool enable)
{
	mtk_hdmi_mask(hdmi, GRL_AUDIO_CFG, enable ? SACD_SEL : 0, SACD_SEL);
}

void mtk_hdmi_hw_aud_set_i2s_chan_num(struct mtk_hdmi *hdmi,
				      enum hdmi_aud_channel_type channel_type,
				      u8 channel_count)
{
	u8 val_1, val_2, val_3, val_4;

	if (channel_count == 2) {
		val_1 = 0x04;
		val_2 = 0x50;
	} else if (channel_count == 3 || channel_count == 4) {
		if (channel_count == 4 &&
		    (channel_type == HDMI_AUD_CHAN_TYPE_3_0_LRS ||
		    channel_type == HDMI_AUD_CHAN_TYPE_4_0)) {
			val_1 = 0x14;
		} else {
			val_1 = 0x0c;
		}
		val_2 = 0x50;
	} else if (channel_count == 6 || channel_count == 5) {
		if (channel_count == 6 &&
		    channel_type != HDMI_AUD_CHAN_TYPE_5_1 &&
		    channel_type != HDMI_AUD_CHAN_TYPE_4_1_CLRS) {
			val_1 = 0x3c;
			val_2 = 0x50;
		} else {
			val_1 = 0x1c;
			val_2 = 0x50;
		}
	} else if (channel_count == 8 || channel_count == 7) {
		val_1 = 0x3c;
		val_2 = 0x50;
	} else {
		val_1 = 0x04;
		val_2 = 0x50;
	}

	val_3 = 0xc6;
	val_4 = 0xfa;

	mtk_hdmi_write(hdmi, GRL_CH_SW0, val_2);
	mtk_hdmi_write(hdmi, GRL_CH_SW1, val_3);
	mtk_hdmi_write(hdmi, GRL_CH_SW2, val_4);
	mtk_hdmi_write(hdmi, GRL_I2S_UV, val_1);
}

void mtk_hdmi_hw_aud_set_input_type(struct mtk_hdmi *hdmi,
				    enum hdmi_aud_input_type input_type)
{
	u32 val = 0;

	val = mtk_hdmi_read(hdmi, GRL_CFG1);
	if (input_type == HDMI_AUD_INPUT_I2S &&
	    (val & CFG1_SPDIF) == CFG1_SPDIF) {
		val &= ~CFG1_SPDIF;
	} else if (input_type == HDMI_AUD_INPUT_SPDIF &&
		(val & CFG1_SPDIF) == 0) {
		val |= CFG1_SPDIF;
	}
	mtk_hdmi_write(hdmi, GRL_CFG1, val);
}

void mtk_hdmi_hw_aud_set_channel_status(struct mtk_hdmi *hdmi,
					u8 *l_chan_status, u8 *r_chan_status,
					enum hdmi_audio_sample_frequency
					aud_hdmi_fs)
{
	u8 l_status[5];
	u8 r_status[5];
	u8 val = 0;

	l_status[0] = l_chan_status[0];
	l_status[1] = l_chan_status[1];
	l_status[2] = l_chan_status[2];
	r_status[0] = r_chan_status[0];
	r_status[1] = r_chan_status[1];
	r_status[2] = r_chan_status[2];

	l_status[0] &= ~0x02;
	r_status[0] &= ~0x02;

	val = l_chan_status[3] & 0xf0;
	switch (aud_hdmi_fs) {
	case HDMI_AUDIO_SAMPLE_FREQUENCY_32000:
		val |= 0x03;
		break;
	case HDMI_AUDIO_SAMPLE_FREQUENCY_44100:
		break;
	case HDMI_AUDIO_SAMPLE_FREQUENCY_88200:
		val |= 0x08;
		break;
	case HDMI_AUDIO_SAMPLE_FREQUENCY_96000:
		val |= 0x0a;
		break;
	case HDMI_AUDIO_SAMPLE_FREQUENCY_48000:
		val |= 0x02;
		break;
	default:
		val |= 0x02;
		break;
	}
	l_status[3] = val;
	r_status[3] = val;

	val = l_chan_status[4];
	val |= ((~(l_status[3] & 0x0f)) << 4);
	l_status[4] = val;
	r_status[4] = val;

	val = l_status[0];
	mtk_hdmi_write(hdmi, GRL_I2S_C_STA0, val);
	mtk_hdmi_write(hdmi, GRL_L_STATUS_0, val);

	val = r_status[0];
	mtk_hdmi_write(hdmi, GRL_R_STATUS_0, val);

	val = l_status[1];
	mtk_hdmi_write(hdmi, GRL_I2S_C_STA1, val);
	mtk_hdmi_write(hdmi, GRL_L_STATUS_1, val);

	val = r_status[1];
	mtk_hdmi_write(hdmi, GRL_R_STATUS_1, val);

	val = l_status[2];
	mtk_hdmi_write(hdmi, GRL_I2S_C_STA2, val);
	mtk_hdmi_write(hdmi, GRL_L_STATUS_2, val);

	val = r_status[2];
	mtk_hdmi_write(hdmi, GRL_R_STATUS_2, val);

	val = l_status[3];
	mtk_hdmi_write(hdmi, GRL_I2S_C_STA3, val);
	mtk_hdmi_write(hdmi, GRL_L_STATUS_3, val);

	val = r_status[3];
	mtk_hdmi_write(hdmi, GRL_R_STATUS_3, val);

	val = l_status[4];
	mtk_hdmi_write(hdmi, GRL_I2S_C_STA4, val);
	mtk_hdmi_write(hdmi, GRL_L_STATUS_4, val);

	val = r_status[4];
	mtk_hdmi_write(hdmi, GRL_R_STATUS_4, val);

	for (val = 0; val < 19; val++) {
		mtk_hdmi_write(hdmi, GRL_L_STATUS_5 + val * 4, 0);
		mtk_hdmi_write(hdmi, GRL_R_STATUS_5 + val * 4, 0);
	}
}

void mtk_hdmi_hw_aud_src_reenable(struct mtk_hdmi *hdmi)
{
	u32 val;

	val = mtk_hdmi_read(hdmi, GRL_MIX_CTRL);
	if (val & MIX_CTRL_SRC_EN) {
		val &= ~MIX_CTRL_SRC_EN;
		mtk_hdmi_write(hdmi, GRL_MIX_CTRL, val);
		usleep_range(255, 512);
		val |= MIX_CTRL_SRC_EN;
		mtk_hdmi_write(hdmi, GRL_MIX_CTRL, val);
	}
}

void mtk_hdmi_hw_aud_src_off(struct mtk_hdmi *hdmi)
{
	u32 val;

	val = mtk_hdmi_read(hdmi, GRL_MIX_CTRL);
	val &= ~MIX_CTRL_SRC_EN;
	mtk_hdmi_write(hdmi, GRL_MIX_CTRL, val);
	mtk_hdmi_write(hdmi, GRL_SHIFT_L1, 0x00);
}

void mtk_hdmi_hw_aud_set_mclk(struct mtk_hdmi *hdmi, enum hdmi_aud_mclk mclk)
{
	u32 val;

	val = mtk_hdmi_read(hdmi, GRL_CFG5);
	val &= CFG5_CD_RATIO_MASK;

	switch (mclk) {
	case HDMI_AUD_MCLK_128FS:
		val |= CFG5_FS128;
		break;
	case HDMI_AUD_MCLK_256FS:
		val |= CFG5_FS256;
		break;
	case HDMI_AUD_MCLK_384FS:
		val |= CFG5_FS384;
		break;
	case HDMI_AUD_MCLK_512FS:
		val |= CFG5_FS512;
		break;
	case HDMI_AUD_MCLK_768FS:
		val |= CFG5_FS768;
		break;
	default:
		val |= CFG5_FS256;
		break;
	}
	mtk_hdmi_write(hdmi, GRL_CFG5, val);
}

void mtk_hdmi_hw_aud_aclk_inv_enable(struct mtk_hdmi *hdmi, bool enable)
{
	u32 val;

	val = mtk_hdmi_read(hdmi, GRL_CFG2);
	if (enable)
		val |= 0x80;
	else
		val &= ~0x80;
	mtk_hdmi_write(hdmi, GRL_CFG2, val);
}

struct hdmi_acr_n {
	unsigned int clock;
	unsigned int n[3];
};

/* Recommended N values from HDMI specification, tables 7-1 to 7-3 */
static const struct hdmi_acr_n hdmi_rec_n_table[] = {
	/* Clock, N: 32kHz 44.1kHz 48kHz */
	{  25175, {  4576,  7007,  6864 } },
	{  74176, { 11648, 17836, 11648 } },
	{ 148352, { 11648,  8918,  5824 } },
	{ 296703, {  5824,  4459,  5824 } },
	{ 297000, {  3072,  4704,  5120 } },
	{      0, {  4096,  6272,  6144 } }, /* all other TMDS clocks */
};

/**
 * hdmi_recommended_n() - Return N value recommended by HDMI specification
 * @freq: audio sample rate in Hz
 * @clock: rounded TMDS clock in kHz
 */
static unsigned int hdmi_recommended_n(unsigned int freq, unsigned int clock)
{
	const struct hdmi_acr_n *recommended;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(hdmi_rec_n_table) - 1; i++) {
		if (clock == hdmi_rec_n_table[i].clock)
			break;
	}
	recommended = hdmi_rec_n_table + i;

	switch (freq) {
	case 32000:
		return recommended->n[0];
	case 44100:
		return recommended->n[1];
	case 48000:
		return recommended->n[2];
	case 88200:
		return recommended->n[1] * 2;
	case 96000:
		return recommended->n[2] * 2;
	case 176400:
		return recommended->n[1] * 4;
	case 192000:
		return recommended->n[2] * 4;
	default:
		return (128 * freq) / 1000;
	}
}

static unsigned int hdmi_mode_clock_to_hz(unsigned int clock)
{
	switch (clock) {
	case 25175:
		return 25174825;	/* 25.2/1.001 MHz */
	case 74176:
		return 74175824;	/* 74.25/1.001 MHz */
	case 148352:
		return 148351648;	/* 148.5/1.001 MHz */
	case 296703:
		return 296703297;	/* 297/1.001 MHz */
	default:
		return clock * 1000;
	}
}

static unsigned int hdmi_expected_cts(unsigned int audio_sample_rate,
				      unsigned int tmds_clock, unsigned int n)
{
	return DIV_ROUND_CLOSEST_ULL((u64)hdmi_mode_clock_to_hz(tmds_clock) * n,
				     128 * audio_sample_rate);
}

static void do_hdmi_hw_aud_set_ncts(struct mtk_hdmi *hdmi, unsigned int n,
				    unsigned int cts)
{
	unsigned char val[NCTS_BYTES];
	int i;

	mtk_hdmi_write(hdmi, GRL_NCTS, 0);
	mtk_hdmi_write(hdmi, GRL_NCTS, 0);
	mtk_hdmi_write(hdmi, GRL_NCTS, 0);
	memset(val, 0, sizeof(val));

	val[0] = (cts >> 24) & 0xff;
	val[1] = (cts >> 16) & 0xff;
	val[2] = (cts >> 8) & 0xff;
	val[3] = cts & 0xff;

	val[4] = (n >> 16) & 0xff;
	val[5] = (n >> 8) & 0xff;
	val[6] = n & 0xff;

	for (i = 0; i < NCTS_BYTES; i++)
		mtk_hdmi_write(hdmi, GRL_NCTS, val[i]);
}

void mtk_hdmi_hw_aud_set_ncts(struct mtk_hdmi *hdmi, unsigned int sample_rate,
			      unsigned int clock)
{
	unsigned int n, cts;

	n = hdmi_recommended_n(sample_rate, clock);
	cts = hdmi_expected_cts(sample_rate, clock, n);

	dev_dbg(hdmi->dev, "%s: sample_rate=%u, clock=%d, cts=%u, n=%u\n",
		__func__, sample_rate, clock, n, cts);

	mtk_hdmi_mask(hdmi, DUMMY_304, AUDIO_I2S_NCTS_SEL_64,
		      AUDIO_I2S_NCTS_SEL);
	do_hdmi_hw_aud_set_ncts(hdmi, n, cts);
}
