/*
 * Copyright (C) 2013 NVIDIA Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>

#include <soc/tegra/kfuse.h>
#include <soc/tegra/pmc.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_dp_helper.h>
#include <drm/drm_hdcp_helper.h>
#include <drm/drm_panel.h>

#include "dc.h"
#include "drm.h"
#include "sor.h"

#define SOR_REKEY 0x38

struct tegra_sor_hdmi_settings {
	unsigned long frequency;

	u8 vcocap;
	u8 ichpmp;
	u8 loadadj;
	u8 termadj;
	u8 tx_pu;
	u8 bg_vref;

	u8 drive_current[4];
	u8 preemphasis[4];
};

static const struct tegra_sor_hdmi_settings tegra210_sor_hdmi_defaults[] = {
	{
		.frequency = 54000000,
		.vcocap = 0x0,
		.ichpmp = 0x1,
		.loadadj = 0x3,
		.termadj = 0x9,
		.tx_pu = 0x10,
		.bg_vref = 0x8,
		.drive_current = { 0x33, 0x3a, 0x3a, 0x3a },
		.preemphasis = { 0x00, 0x00, 0x00, 0x00 },
	}, {
		.frequency = 75000000,
		.vcocap = 0x3,
		.ichpmp = 0x1,
		.loadadj = 0x3,
		.termadj = 0x9,
		.tx_pu = 0x40,
		.bg_vref = 0x8,
		.drive_current = { 0x33, 0x3a, 0x3a, 0x3a },
		.preemphasis = { 0x00, 0x00, 0x00, 0x00 },
	}, {
		.frequency = 150000000,
		.vcocap = 0x3,
		.ichpmp = 0x1,
		.loadadj = 0x3,
		.termadj = 0x9,
		.tx_pu = 0x66,
		.bg_vref = 0x8,
		.drive_current = { 0x33, 0x3a, 0x3a, 0x3a },
		.preemphasis = { 0x00, 0x00, 0x00, 0x00 },
	}, {
		.frequency = 300000000,
		.vcocap = 0x3,
		.ichpmp = 0x1,
		.loadadj = 0x3,
		.termadj = 0x9,
		.tx_pu = 0x66,
		.bg_vref = 0xa,
		.drive_current = { 0x33, 0x3f, 0x3f, 0x3f },
		.preemphasis = { 0x00, 0x17, 0x17, 0x17 },
	}, {
		.frequency = 600000000,
		.vcocap = 0x3,
		.ichpmp = 0x1,
		.loadadj = 0x3,
		.termadj = 0x9,
		.tx_pu = 0x66,
		.bg_vref = 0x8,
		.drive_current = { 0x33, 0x3f, 0x3f, 0x3f },
		.preemphasis = { 0x00, 0x00, 0x00, 0x00 },
	},
};

struct tegra_sor_soc {
	bool supports_edp;
	bool supports_lvds;
	bool supports_hdmi;
	bool supports_dp;

	const struct tegra_sor_hdmi_settings *settings;
	unsigned int num_settings;

	const u8 *xbar_cfg;
	const u8 *lane_map;

	const u8 (*voltage_swing)[4][4];
	const u8 (*pre_emphasis)[4][4];
	const u8 (*post_cursor)[4][4];
	const u8 (*tx_pu)[4][4];
};

struct tegra_sor;

struct tegra_sor_ops {
	const char *name;
	int (*probe)(struct tegra_sor *sor);
	int (*remove)(struct tegra_sor *sor);
};

struct tegra_sor {
	struct host1x_client client;
	struct tegra_output output;
	struct device *dev;

	const struct tegra_sor_soc *soc;
	void __iomem *regs;
	unsigned int irq;

	struct reset_control *rst;
	struct clk *clk_parent;
	struct clk *clk_safe;
	struct clk *clk_dp;
	struct clk *clk;

	struct drm_dp_aux *aux;
	struct drm_dp_link link;

	struct drm_info_list *debugfs_files;
	struct drm_minor *minor;
	struct dentry *debugfs;

	const struct tegra_sor_ops *ops;

	/* for HDMI 2.0 */
	struct tegra_sor_hdmi_settings *settings;
	unsigned int num_settings;

	struct regulator *avdd_io_supply;
	struct regulator *vdd_pll_supply;
	struct regulator *hdmi_supply;

	struct tegra_kfuse *kfuse;
	struct drm_hdcp hdcp;
};

struct tegra_sor_state {
	struct drm_connector_state base;

	unsigned int bpc;
};

static inline struct tegra_sor_state *
to_sor_state(struct drm_connector_state *state)
{
	return container_of(state, struct tegra_sor_state, base);
}

struct tegra_sor_config {
	u32 bits_per_pixel;

	u32 active_polarity;
	u32 active_count;
	u32 tu_size;
	u32 active_frac;
	u32 watermark;

	u32 hblank_symbols;
	u32 vblank_symbols;
};

static inline struct tegra_sor *
host1x_client_to_sor(struct host1x_client *client)
{
	return container_of(client, struct tegra_sor, client);
}

static inline struct tegra_sor *to_sor(struct tegra_output *output)
{
	return container_of(output, struct tegra_sor, output);
}

static inline u32 tegra_sor_readl(struct tegra_sor *sor, unsigned long offset)
{
	return readl(sor->regs + (offset << 2));
}

static inline void tegra_sor_writel(struct tegra_sor *sor, u32 value,
				    unsigned long offset)
{
	writel(value, sor->regs + (offset << 2));
}

static int tegra_sor_set_parent_clock(struct tegra_sor *sor, struct clk *parent)
{
	int err;

	clk_disable_unprepare(sor->clk);

	err = clk_set_parent(sor->clk, parent);
	if (err < 0)
		return err;

	err = clk_prepare_enable(sor->clk);
	if (err < 0)
		return err;

	return 0;
}

static int tegra_sor_power_up_lanes(struct tegra_sor *sor, unsigned int lanes)
{
	unsigned long timeout;
	u32 value;

	/*
	 * Clear or set the PD_TXD bit corresponding to each lane, depending
	 * on whether it is used or not.
	 */
	value = tegra_sor_readl(sor, SOR_DP_PADCTL0);

	if (sor->link.lanes <= 2)
		value &= ~(SOR_DP_PADCTL_PD_TXD(sor->soc->lane_map[3]) |
			   SOR_DP_PADCTL_PD_TXD(sor->soc->lane_map[2]));
	else
		value |= SOR_DP_PADCTL_PD_TXD(sor->soc->lane_map[3]) |
			 SOR_DP_PADCTL_PD_TXD(sor->soc->lane_map[2]);

	if (sor->link.lanes <= 1)
		value &= ~SOR_DP_PADCTL_PD_TXD(sor->soc->lane_map[1]);
	else
		value |= SOR_DP_PADCTL_PD_TXD(sor->soc->lane_map[1]);

	if (sor->link.lanes == 0)
		value &= ~SOR_DP_PADCTL_PD_TXD(sor->soc->lane_map[0]);
	else
		value |= SOR_DP_PADCTL_PD_TXD(sor->soc->lane_map[0]);

	tegra_sor_writel(sor, value, SOR_DP_PADCTL0);

	/* start lane sequencer */
	value = SOR_LANE_SEQ_CTL_TRIGGER | SOR_LANE_SEQ_CTL_SEQUENCE_DOWN |
		SOR_LANE_SEQ_CTL_POWER_STATE_UP;
	tegra_sor_writel(sor, value, SOR_LANE_SEQ_CTL);

	timeout = jiffies + msecs_to_jiffies(250);

	while (time_before(jiffies, timeout)) {
		value = tegra_sor_readl(sor, SOR_LANE_SEQ_CTL);
		if ((value & SOR_LANE_SEQ_CTL_TRIGGER) == 0)
			break;

		usleep_range(250, 1000);
	}

	if ((value & SOR_LANE_SEQ_CTL_TRIGGER) != 0)
		return -ETIMEDOUT;

	return 0;
}

static int tegra_sor_power_down_lanes(struct tegra_sor *sor)
{
	unsigned long timeout;
	u32 value;

	/* power down all lanes */
	value = tegra_sor_readl(sor, SOR_DP_PADCTL0);
	value &= ~(SOR_DP_PADCTL_PD_TXD_3 | SOR_DP_PADCTL_PD_TXD_0 |
		   SOR_DP_PADCTL_PD_TXD_1 | SOR_DP_PADCTL_PD_TXD_2);
	tegra_sor_writel(sor, value, SOR_DP_PADCTL0);

	/* start lane sequencer */
	value = SOR_LANE_SEQ_CTL_TRIGGER | SOR_LANE_SEQ_CTL_SEQUENCE_UP |
		SOR_LANE_SEQ_CTL_POWER_STATE_DOWN;
	tegra_sor_writel(sor, value, SOR_LANE_SEQ_CTL);

	timeout = jiffies + msecs_to_jiffies(250);

	while (time_before(jiffies, timeout)) {
		value = tegra_sor_readl(sor, SOR_LANE_SEQ_CTL);
		if ((value & SOR_LANE_SEQ_CTL_TRIGGER) == 0)
			break;

		usleep_range(25, 100);
	}

	if ((value & SOR_LANE_SEQ_CTL_TRIGGER) != 0)
		return -ETIMEDOUT;

	return 0;
}

static void tegra_sor_dp_precharge(struct tegra_sor *sor, unsigned int lanes)
{
	u32 value;

	/* pre-charge all used lanes */
	value = tegra_sor_readl(sor, SOR_DP_PADCTL0);

	if (lanes <= 2)
		value &= ~(SOR_DP_PADCTL_CM_TXD(sor->soc->lane_map[3]) |
			   SOR_DP_PADCTL_CM_TXD(sor->soc->lane_map[2]));
	else
		value |= SOR_DP_PADCTL_CM_TXD(sor->soc->lane_map[3]) |
			 SOR_DP_PADCTL_CM_TXD(sor->soc->lane_map[2]);

	if (lanes <= 1)
		value &= ~SOR_DP_PADCTL_CM_TXD(sor->soc->lane_map[1]);
	else
		value |= SOR_DP_PADCTL_CM_TXD(sor->soc->lane_map[1]);

	if (lanes == 0)
		value &= ~SOR_DP_PADCTL_CM_TXD(sor->soc->lane_map[0]);
	else
		value |= SOR_DP_PADCTL_CM_TXD(sor->soc->lane_map[0]);

	tegra_sor_writel(sor, value, SOR_DP_PADCTL0);

	usleep_range(15, 100);

	value = tegra_sor_readl(sor, SOR_DP_PADCTL0);
	value &= ~(SOR_DP_PADCTL_CM_TXD_3 | SOR_DP_PADCTL_CM_TXD_2 |
		   SOR_DP_PADCTL_CM_TXD_1 | SOR_DP_PADCTL_CM_TXD_0);
	tegra_sor_writel(sor, value, SOR_DP_PADCTL0);
}

static void tegra_sor_dp_term_calibrate(struct tegra_sor *sor)
{
	u32 mask = 0x08, adj = 0, value;

	/* enable pad calibration logic */
	value = tegra_sor_readl(sor, SOR_DP_PADCTL0);
	value &= ~SOR_DP_PADCTL_PAD_CAL_PD;
	tegra_sor_writel(sor, value, SOR_DP_PADCTL0);

	value = tegra_sor_readl(sor, SOR_PLL1);
	value |= SOR_PLL1_TMDS_TERM;
	tegra_sor_writel(sor, value, SOR_PLL1);

	while (mask) {
		adj |= mask;

		value = tegra_sor_readl(sor, SOR_PLL1);
		value &= ~SOR_PLL1_TMDS_TERMADJ_MASK;
		value |= SOR_PLL1_TMDS_TERMADJ(adj);
		tegra_sor_writel(sor, value, SOR_PLL1);

		usleep_range(100, 200);

		value = tegra_sor_readl(sor, SOR_PLL1);
		if (value & SOR_PLL1_TERM_COMPOUT)
			adj &= ~mask;

		mask >>= 1;
	}

	value = tegra_sor_readl(sor, SOR_PLL1);
	value &= ~SOR_PLL1_TMDS_TERMADJ_MASK;
	value |= SOR_PLL1_TMDS_TERMADJ(adj);
	tegra_sor_writel(sor, value, SOR_PLL1);

	/* disable pad calibration logic */
	value = tegra_sor_readl(sor, SOR_DP_PADCTL0);
	value |= SOR_DP_PADCTL_PAD_CAL_PD;
	tegra_sor_writel(sor, value, SOR_DP_PADCTL0);
}

static int tegra_sor_dp_link_apply_training(struct drm_dp_link *link)
{
	struct tegra_sor *sor = container_of(link, struct tegra_sor, link);
	u32 voltage_swing = 0, pre_emphasis = 0, post_cursor = 0;
	const struct tegra_sor_soc *soc = sor->soc;
	u32 pattern = 0, tx_pu = 0, value;
	unsigned int i;

	for (value = 0, i = 0; i < link->lanes; i++) {
		u8 vs = link->train.request.voltage_swing[i];
		u8 pe = link->train.request.pre_emphasis[i];
		u8 pc = link->train.request.post_cursor[i];
		u8 shift = sor->soc->lane_map[i] << 3;

		voltage_swing |= soc->voltage_swing[pc][vs][pe] << shift;
		pre_emphasis |= soc->pre_emphasis[pc][vs][pe] << shift;
		post_cursor |= soc->post_cursor[pc][vs][pe] << shift;

		if (sor->soc->tx_pu[pc][vs][pe] > tx_pu)
			tx_pu = sor->soc->tx_pu[pc][vs][pe];

		switch (link->train.pattern) {
		case DP_TRAINING_PATTERN_DISABLE:
			value = SOR_DP_TPG_SCRAMBLER_GALIOS |
				SOR_DP_TPG_PATTERN_NONE;
			break;

		case DP_TRAINING_PATTERN_1:
			value = SOR_DP_TPG_SCRAMBLER_NONE |
				SOR_DP_TPG_PATTERN_TRAIN1;
			break;

		case DP_TRAINING_PATTERN_2:
			value = SOR_DP_TPG_SCRAMBLER_NONE |
				SOR_DP_TPG_PATTERN_TRAIN2;
			break;

		case DP_TRAINING_PATTERN_3:
			value = SOR_DP_TPG_SCRAMBLER_NONE |
				SOR_DP_TPG_PATTERN_TRAIN3;
			break;

		default:
			return -EINVAL;
		}

		if (link->capabilities & DP_LINK_CAP_ANSI_8B10B)
			value |= SOR_DP_TPG_CHANNEL_CODING;

		pattern = pattern << 8 | value;
	}

	tegra_sor_writel(sor, voltage_swing, SOR_LANE_DRIVE_CURRENT0);
	tegra_sor_writel(sor, pre_emphasis, SOR_LANE_PREEMPHASIS0);

	if (link->capabilities & DP_LINK_CAP_TPS3)
		tegra_sor_writel(sor, post_cursor, SOR_LANE_POSTCURSOR0);

	tegra_sor_writel(sor, pattern, SOR_DP_TPG);

	value = tegra_sor_readl(sor, SOR_DP_PADCTL0);
	value &= ~SOR_DP_PADCTL_TX_PU_MASK;
	value |= SOR_DP_PADCTL_TX_PU_ENABLE;
	value |= SOR_DP_PADCTL_TX_PU(tx_pu);
	tegra_sor_writel(sor, value, SOR_DP_PADCTL0);

	usleep_range(20, 100);

	return 0;
}

static int tegra_sor_dp_link_configure(struct drm_dp_link *link)
{
	struct tegra_sor *sor = container_of(link, struct tegra_sor, link);
	unsigned int rate, lanes;
	u32 value;
	int err;

	rate = drm_dp_link_rate_to_bw_code(link->rate);
	lanes = link->lanes;

	/* configure link speed and lane count */
	value = tegra_sor_readl(sor, SOR_CLK_CNTRL);
	value &= ~SOR_CLK_CNTRL_DP_LINK_SPEED_MASK;
	value |= SOR_CLK_CNTRL_DP_LINK_SPEED(rate);
	tegra_sor_writel(sor, value, SOR_CLK_CNTRL);

	value = tegra_sor_readl(sor, SOR_DP_LINKCTL0);
	value &= ~SOR_DP_LINKCTL_LANE_COUNT_MASK;
	value |= SOR_DP_LINKCTL_LANE_COUNT(lanes);

	if (link->capabilities & DP_LINK_CAP_ENHANCED_FRAMING)
		value |= SOR_DP_LINKCTL_ENHANCED_FRAME;

	tegra_sor_writel(sor, value, SOR_DP_LINKCTL0);

	usleep_range(400, 1000);

	/* configure load pulse position adjustment */
	value = tegra_sor_readl(sor, SOR_PLL1);
	value &= ~SOR_PLL1_LOADADJ_MASK;

	switch (rate) {
	case DP_LINK_BW_1_62:
		value |= SOR_PLL1_LOADADJ(0x3);
		break;

	case DP_LINK_BW_2_7:
		value |= SOR_PLL1_LOADADJ(0x4);
		break;

	case DP_LINK_BW_5_4:
		value |= SOR_PLL1_LOADADJ(0x6);
		break;
	}

	tegra_sor_writel(sor, value, SOR_PLL1);

	/* use alternate scrambler reset for eDP */
	value = tegra_sor_readl(sor, SOR_DP_SPARE0);

	if (link->edp == 0)
		value &= ~SOR_DP_SPARE_PANEL_INTERNAL;
	else
		value |= SOR_DP_SPARE_PANEL_INTERNAL;

	tegra_sor_writel(sor, value, SOR_DP_SPARE0);

	err = tegra_sor_power_down_lanes(sor);
	if (err < 0) {
		dev_err(sor->dev, "failed to power down lanes: %d\n", err);
		return err;
	}

	/* power up and pre-charge lanes */
	err = tegra_sor_power_up_lanes(sor, lanes);
	if (err < 0) {
		dev_err(sor->dev, "failed to power up %u lane%s: %d\n",
			lanes, (lanes != 1) ? "s" : "", err);
		return err;
	}

	tegra_sor_dp_precharge(sor, lanes);

	return 0;
}

static const struct drm_dp_link_ops tegra_sor_dp_link_ops = {
	.apply_training = tegra_sor_dp_link_apply_training,
	.configure = tegra_sor_dp_link_configure,
};

static void tegra_sor_super_update(struct tegra_sor *sor)
{
	tegra_sor_writel(sor, 0, SOR_SUPER_STATE0);
	tegra_sor_writel(sor, 1, SOR_SUPER_STATE0);
	tegra_sor_writel(sor, 0, SOR_SUPER_STATE0);
}

static void tegra_sor_update(struct tegra_sor *sor)
{
	tegra_sor_writel(sor, 0, SOR_STATE0);
	tegra_sor_writel(sor, 1, SOR_STATE0);
	tegra_sor_writel(sor, 0, SOR_STATE0);
}

static int tegra_sor_setup_pwm(struct tegra_sor *sor, unsigned long timeout)
{
	u32 value;

	value = tegra_sor_readl(sor, SOR_PWM_DIV);
	value &= ~SOR_PWM_DIV_MASK;
	value |= 0x400; /* period */
	tegra_sor_writel(sor, value, SOR_PWM_DIV);

	value = tegra_sor_readl(sor, SOR_PWM_CTL);
	value &= ~SOR_PWM_CTL_DUTY_CYCLE_MASK;
	value |= 0x400; /* duty cycle */
	value &= ~SOR_PWM_CTL_CLK_SEL; /* clock source: PCLK */
	value |= SOR_PWM_CTL_TRIGGER;
	tegra_sor_writel(sor, value, SOR_PWM_CTL);

	return wait_for_interval(
		!(tegra_sor_readl(sor, SOR_PWM_CTL) & SOR_PWM_CTL_TRIGGER),
		timeout, 25);
}

static int tegra_sor_attach(struct tegra_sor *sor)
{
	unsigned long value;

	/* wake up in normal mode */
	value = tegra_sor_readl(sor, SOR_SUPER_STATE1);
	value |= SOR_SUPER_STATE_HEAD_MODE_AWAKE;
	value |= SOR_SUPER_STATE_MODE_NORMAL;
	tegra_sor_writel(sor, value, SOR_SUPER_STATE1);
	tegra_sor_super_update(sor);

	/* attach */
	value = tegra_sor_readl(sor, SOR_SUPER_STATE1);
	value |= SOR_SUPER_STATE_ATTACHED;
	tegra_sor_writel(sor, value, SOR_SUPER_STATE1);
	tegra_sor_super_update(sor);

	return wait_for_interval(
		tegra_sor_readl(sor, SOR_TEST) & SOR_TEST_ATTACHED, 250, 25);
}

static int tegra_sor_wakeup(struct tegra_sor *sor)
{
	return wait_for_interval(
		(tegra_sor_readl(sor, SOR_TEST) & SOR_TEST_HEAD_MODE_MASK) ==
			SOR_TEST_HEAD_MODE_AWAKE,
		250, 25);
}

static int tegra_sor_power_up(struct tegra_sor *sor, unsigned long timeout)
{
	u32 value;

	value = tegra_sor_readl(sor, SOR_PWR);
	value |= SOR_PWR_TRIGGER | SOR_PWR_NORMAL_STATE_PU;
	tegra_sor_writel(sor, value, SOR_PWR);

	return wait_for_interval(
		!(tegra_sor_readl(sor, SOR_PWR) & SOR_PWR_TRIGGER), timeout,
		25);
}

struct tegra_sor_params {
	/* number of link clocks per line */
	unsigned int num_clocks;
	/* ratio between input and output */
	u64 ratio;
	/* precision factor */
	u64 precision;

	unsigned int active_polarity;
	unsigned int active_count;
	unsigned int active_frac;
	unsigned int tu_size;
	unsigned int error;
};

static int tegra_sor_compute_params(struct tegra_sor *sor,
				    struct tegra_sor_params *params,
				    unsigned int tu_size)
{
	u64 active_sym, active_count, frac, approx;
	u32 active_polarity, active_frac = 0;
	const u64 f = params->precision;
	s64 error;

	active_sym = params->ratio * tu_size;
	active_count = div_u64(active_sym, f) * f;
	frac = active_sym - active_count;

	/* fraction < 0.5 */
	if (frac >= (f / 2)) {
		active_polarity = 1;
		frac = f - frac;
	} else {
		active_polarity = 0;
	}

	if (frac != 0) {
		frac = div_u64(f * f,  frac); /* 1/fraction */
		if (frac <= (15 * f)) {
			active_frac = div_u64(frac, f);

			/* round up */
			if (active_polarity)
				active_frac++;
		} else {
			active_frac = active_polarity ? 1 : 15;
		}
	}

	if (active_frac == 1)
		active_polarity = 0;

	if (active_polarity == 1) {
		if (active_frac) {
			approx = active_count + (active_frac * (f - 1)) * f;
			approx = div_u64(approx, active_frac * f);
		} else {
			approx = active_count + f;
		}
	} else {
		if (active_frac)
			approx = active_count + div_u64(f, active_frac);
		else
			approx = active_count;
	}

	error = div_s64(active_sym - approx, tu_size);
	error *= params->num_clocks;

	if (error <= 0 && abs64(error) < params->error) {
		params->active_count = div_u64(active_count, f);
		params->active_polarity = active_polarity;
		params->active_frac = active_frac;
		params->error = abs64(error);
		params->tu_size = tu_size;

		if (error == 0)
			return true;
	}

	return false;
}

static int tegra_sor_compute_config(struct tegra_sor *sor,
				    const struct drm_display_mode *mode,
				    struct tegra_sor_config *config,
				    struct drm_dp_link *link)
{
	const u64 f = 100000, link_rate = link->rate * 1000;
	const u64 pclk = mode->clock * 1000;
	u64 input, output, watermark, num;
	struct tegra_sor_params params;
	u32 num_syms_per_line;
	unsigned int i;

	if (!link_rate || !link->lanes || !pclk || !config->bits_per_pixel)
		return -EINVAL;

	input = pclk * config->bits_per_pixel;
	output = link_rate * 8 * link->lanes;

	if (input >= output)
		return -ERANGE;

	memset(&params, 0, sizeof(params));
	params.ratio = div64_u64(input * f, output);
	params.num_clocks = div_u64(link_rate * mode->hdisplay, pclk);
	params.precision = f;
	params.error = 64 * f;
	params.tu_size = 64;

	for (i = params.tu_size; i >= 32; i--)
		if (tegra_sor_compute_params(sor, &params, i))
			break;

	if (params.active_frac == 0) {
		config->active_polarity = 0;
		config->active_count = params.active_count;

		if (!params.active_polarity)
			config->active_count--;

		config->tu_size = params.tu_size;
		config->active_frac = 1;
	} else {
		config->active_polarity = params.active_polarity;
		config->active_count = params.active_count;
		config->active_frac = params.active_frac;
		config->tu_size = params.tu_size;
	}

	dev_dbg(sor->dev,
		"polarity: %d active count: %d tu size: %d active frac: %d\n",
		config->active_polarity, config->active_count,
		config->tu_size, config->active_frac);

	watermark = params.ratio * config->tu_size * (f - params.ratio);
	watermark = div_u64(watermark, f);

	watermark = div_u64(watermark + params.error, f);
	config->watermark = watermark + (config->bits_per_pixel / 8) + 2;
	num_syms_per_line = (mode->hdisplay * config->bits_per_pixel) *
			    (link->lanes * 8);

	if (config->watermark > 30) {
		config->watermark = 30;
		dev_err(sor->dev,
			"unable to compute TU size, forcing watermark to %u\n",
			config->watermark);
	} else if (config->watermark > num_syms_per_line) {
		config->watermark = num_syms_per_line;
		dev_err(sor->dev, "watermark too high, forcing to %u\n",
			config->watermark);
	}

	/* compute the number of symbols per horizontal blanking interval */
	num = ((mode->htotal - mode->hdisplay) - 7) * link_rate;
	config->hblank_symbols = div_u64(num, pclk);

	if (link->capabilities & DP_LINK_CAP_ENHANCED_FRAMING)
		config->hblank_symbols -= 3;

	config->hblank_symbols -= 12 / link->lanes;

	/* compute the number of symbols per vertical blanking interval */
	num = (mode->hdisplay - 25) * link_rate;
	config->vblank_symbols = div_u64(num, pclk);
	config->vblank_symbols -= 36 / link->lanes + 4;

	dev_dbg(sor->dev, "blank symbols: H:%u V:%u\n", config->hblank_symbols,
		config->vblank_symbols);

	return 0;
}

static void tegra_sor_apply_config(struct tegra_sor *sor,
				   const struct tegra_sor_config *config)
{
	u32 value;

	/* set linkctl */
	value = tegra_sor_readl(sor, SOR_DP_LINKCTL0);
	value &= ~SOR_DP_LINKCTL_TU_SIZE_MASK;
	value |= SOR_DP_LINKCTL_TU_SIZE(config->tu_size);
	tegra_sor_writel(sor, value, SOR_DP_LINKCTL0);

	value = tegra_sor_readl(sor, SOR_DP_CONFIG0);
	value &= ~SOR_DP_CONFIG_WATERMARK_MASK;
	value |= SOR_DP_CONFIG_WATERMARK(config->watermark);

	value &= ~SOR_DP_CONFIG_ACTIVE_SYM_COUNT_MASK;
	value |= SOR_DP_CONFIG_ACTIVE_SYM_COUNT(config->active_count);

	value &= ~SOR_DP_CONFIG_ACTIVE_SYM_FRAC_MASK;
	value |= SOR_DP_CONFIG_ACTIVE_SYM_FRAC(config->active_frac);

	if (config->active_polarity)
		value |= SOR_DP_CONFIG_ACTIVE_SYM_POLARITY;
	else
		value &= ~SOR_DP_CONFIG_ACTIVE_SYM_POLARITY;

	value |= SOR_DP_CONFIG_ACTIVE_SYM_ENABLE;
	value |= SOR_DP_CONFIG_DISPARITY_NEGATIVE;
	tegra_sor_writel(sor, value, SOR_DP_CONFIG0);

	value = tegra_sor_readl(sor, SOR_DP_AUDIO_HBLANK_SYMBOLS);
	value &= ~SOR_DP_AUDIO_HBLANK_SYMBOLS_MASK;
	value |= config->hblank_symbols & 0xffff;
	tegra_sor_writel(sor, value, SOR_DP_AUDIO_HBLANK_SYMBOLS);

	value = tegra_sor_readl(sor, SOR_DP_AUDIO_VBLANK_SYMBOLS);
	value &= ~SOR_DP_AUDIO_VBLANK_SYMBOLS_MASK;
	value |= config->vblank_symbols & 0xffff;
	tegra_sor_writel(sor, value, SOR_DP_AUDIO_VBLANK_SYMBOLS);
}

static void tegra_sor_mode_set(struct tegra_sor *sor,
			       const struct drm_display_mode *mode,
			       struct tegra_sor_state *state)
{
	struct tegra_dc *dc = to_tegra_dc(sor->output.encoder.crtc);
	unsigned int vbe, vse, hbe, hse, vbs, hbs;
	u32 value;

	value = tegra_sor_readl(sor, SOR_STATE1);
	value &= ~SOR_STATE_ASY_PIXELDEPTH_MASK;
	value &= ~SOR_STATE_ASY_CRC_MODE_MASK;
	value &= ~SOR_STATE_ASY_OWNER_MASK;

	value |= SOR_STATE_ASY_CRC_MODE_COMPLETE |
		 SOR_STATE_ASY_OWNER(dc->pipe + 1);

	if (mode->flags & DRM_MODE_FLAG_PHSYNC)
		value &= ~SOR_STATE_ASY_HSYNCPOL;

	if (mode->flags & DRM_MODE_FLAG_NHSYNC)
		value |= SOR_STATE_ASY_HSYNCPOL;

	if (mode->flags & DRM_MODE_FLAG_PVSYNC)
		value &= ~SOR_STATE_ASY_VSYNCPOL;

	if (mode->flags & DRM_MODE_FLAG_NVSYNC)
		value |= SOR_STATE_ASY_VSYNCPOL;

	switch (state->bpc) {
	case 16:
		value |= SOR_STATE_ASY_PIXELDEPTH_BPP_48_444;
		break;

	case 12:
		value |= SOR_STATE_ASY_PIXELDEPTH_BPP_36_444;
		break;

	case 10:
		value |= SOR_STATE_ASY_PIXELDEPTH_BPP_30_444;
		break;

	case 8:
		value |= SOR_STATE_ASY_PIXELDEPTH_BPP_24_444;
		break;

	case 6:
		value |= SOR_STATE_ASY_PIXELDEPTH_BPP_18_444;
		break;

	default:
		value |= SOR_STATE_ASY_PIXELDEPTH_BPP_24_444;
		break;
	}

	tegra_sor_writel(sor, value, SOR_STATE1);

	/*
	 * TODO: The video timing programming below doesn't seem to match the
	 * register definitions.
	 */

	value = ((mode->vtotal & 0x7fff) << 16) | (mode->htotal & 0x7fff);
	tegra_sor_writel(sor, value, SOR_HEAD_STATE1(dc->pipe));

	/* sync end = sync width - 1 */
	vse = mode->vsync_end - mode->vsync_start - 1;
	hse = mode->hsync_end - mode->hsync_start - 1;

	value = ((vse & 0x7fff) << 16) | (hse & 0x7fff);
	tegra_sor_writel(sor, value, SOR_HEAD_STATE2(dc->pipe));

	/* blank end = sync end + back porch */
	vbe = vse + (mode->vtotal - mode->vsync_end);
	hbe = hse + (mode->htotal - mode->hsync_end);

	value = ((vbe & 0x7fff) << 16) | (hbe & 0x7fff);
	tegra_sor_writel(sor, value, SOR_HEAD_STATE3(dc->pipe));

	/* blank start = blank end + active */
	vbs = vbe + mode->vdisplay;
	hbs = hbe + mode->hdisplay;

	value = ((vbs & 0x7fff) << 16) | (hbs & 0x7fff);
	tegra_sor_writel(sor, value, SOR_HEAD_STATE4(dc->pipe));

	/* XXX interlacing support */
	tegra_sor_writel(sor, 0x001, SOR_HEAD_STATE5(dc->pipe));
}

static int tegra_sor_detach(struct tegra_sor *sor)
{
	int err;
	unsigned long value;

	/* switch to safe mode */
	value = tegra_sor_readl(sor, SOR_SUPER_STATE1);
	value &= ~SOR_SUPER_STATE_MODE_NORMAL;
	tegra_sor_writel(sor, value, SOR_SUPER_STATE1);
	tegra_sor_super_update(sor);

	err = wait_for_atomic(tegra_sor_readl(sor, SOR_PWR) & SOR_PWR_MODE_SAFE,
				250);
	if (err)
		return err;

	/* go to sleep */
	value = tegra_sor_readl(sor, SOR_SUPER_STATE1);
	value &= ~SOR_SUPER_STATE_HEAD_MODE_MASK;
	tegra_sor_writel(sor, value, SOR_SUPER_STATE1);
	tegra_sor_super_update(sor);

	/* detach */
	value = tegra_sor_readl(sor, SOR_SUPER_STATE1);
	value &= ~SOR_SUPER_STATE_ATTACHED;
	tegra_sor_writel(sor, value, SOR_SUPER_STATE1);
	tegra_sor_super_update(sor);

	return wait_for_interval(
		!(tegra_sor_readl(sor, SOR_TEST) & SOR_TEST_ATTACHED), 250, 25);
}

static int tegra_sor_power_down(struct tegra_sor *sor)
{
	unsigned long value;
	int err;

	value = tegra_sor_readl(sor, SOR_PWR);
	value &= ~SOR_PWR_NORMAL_STATE_PU;
	value |= SOR_PWR_TRIGGER;
	tegra_sor_writel(sor, value, SOR_PWR);

	err = wait_for_interval(
		!(tegra_sor_readl(sor, SOR_PWR) & SOR_PWR_TRIGGER), 250, 25);
	if (err)
		return err;

	/* switch to safe parent clock */
	err = tegra_sor_set_parent_clock(sor, sor->clk_safe);
	if (err < 0)
		dev_err(sor->dev, "failed to set safe parent clock: %d\n", err);

	value = tegra_sor_readl(sor, SOR_PLL2);
	value |= SOR_PLL2_PORT_POWERDOWN;
	tegra_sor_writel(sor, value, SOR_PLL2);

	usleep_range(20, 100);

	value = tegra_sor_readl(sor, SOR_PLL0);
	value |= SOR_PLL0_VCOPD | SOR_PLL0_PWR;
	tegra_sor_writel(sor, value, SOR_PLL0);

	value = tegra_sor_readl(sor, SOR_PLL2);
	value |= SOR_PLL2_SEQ_PLLCAPPD;
	value |= SOR_PLL2_SEQ_PLLCAPPD_ENFORCE;
	tegra_sor_writel(sor, value, SOR_PLL2);

	usleep_range(20, 100);

	return 0;
}

static int tegra_sor_crc_wait(struct tegra_sor *sor, unsigned long timeout)
{
	return wait_for_interval(
		tegra_sor_readl(sor, SOR_CRCA) & SOR_CRCA_VALID, timeout,
		100);
}

static int tegra_sor_show_crc(struct seq_file *s, void *data)
{
	struct drm_info_node *node = s->private;
	struct tegra_sor *sor = node->info_ent->data;
	struct drm_crtc *crtc = sor->output.encoder.crtc;
	struct drm_device *drm = node->minor->dev;
	int err = 0;
	u32 value;

	drm_modeset_lock_all(drm);

	if (!crtc || !crtc->state->active) {
		err = -EBUSY;
		goto unlock;
	}

	value = tegra_sor_readl(sor, SOR_STATE1);
	value &= ~SOR_STATE_ASY_CRC_MODE_MASK;
	tegra_sor_writel(sor, value, SOR_STATE1);

	value = tegra_sor_readl(sor, SOR_CRC_CNTRL);
	value |= SOR_CRC_CNTRL_ENABLE;
	tegra_sor_writel(sor, value, SOR_CRC_CNTRL);

	value = tegra_sor_readl(sor, SOR_TEST);
	value &= ~SOR_TEST_CRC_POST_SERIALIZE;
	tegra_sor_writel(sor, value, SOR_TEST);

	err = tegra_sor_crc_wait(sor, 100);
	if (err < 0)
		goto unlock;

	tegra_sor_writel(sor, SOR_CRCA_RESET, SOR_CRCA);
	value = tegra_sor_readl(sor, SOR_CRCB);

	seq_printf(s, "%08x\n", value);

unlock:
	drm_modeset_unlock_all(drm);
	return err;
}

static int tegra_sor_show_regs(struct seq_file *s, void *data)
{
	struct drm_info_node *node = s->private;
	struct tegra_sor *sor = node->info_ent->data;
	struct drm_crtc *crtc = sor->output.encoder.crtc;
	struct drm_device *drm = node->minor->dev;
	int err = 0;

	drm_modeset_lock_all(drm);

	if (!crtc || !crtc->state->active) {
		err = -EBUSY;
		goto unlock;
	}

#define DUMP_REG(name)						\
	seq_printf(s, "%-38s %#05x %08x\n", #name, name,	\
		   tegra_sor_readl(sor, name))

	DUMP_REG(SOR_CTXSW);
	DUMP_REG(SOR_SUPER_STATE0);
	DUMP_REG(SOR_SUPER_STATE1);
	DUMP_REG(SOR_STATE0);
	DUMP_REG(SOR_STATE1);
	DUMP_REG(SOR_HEAD_STATE0(0));
	DUMP_REG(SOR_HEAD_STATE0(1));
	DUMP_REG(SOR_HEAD_STATE1(0));
	DUMP_REG(SOR_HEAD_STATE1(1));
	DUMP_REG(SOR_HEAD_STATE2(0));
	DUMP_REG(SOR_HEAD_STATE2(1));
	DUMP_REG(SOR_HEAD_STATE3(0));
	DUMP_REG(SOR_HEAD_STATE3(1));
	DUMP_REG(SOR_HEAD_STATE4(0));
	DUMP_REG(SOR_HEAD_STATE4(1));
	DUMP_REG(SOR_HEAD_STATE5(0));
	DUMP_REG(SOR_HEAD_STATE5(1));
	DUMP_REG(SOR_CRC_CNTRL);
	DUMP_REG(SOR_DP_DEBUG_MVID);
	DUMP_REG(SOR_CLK_CNTRL);
	DUMP_REG(SOR_CAP);
	DUMP_REG(SOR_PWR);
	DUMP_REG(SOR_TEST);
	DUMP_REG(SOR_PLL0);
	DUMP_REG(SOR_PLL1);
	DUMP_REG(SOR_PLL2);
	DUMP_REG(SOR_PLL3);
	DUMP_REG(SOR_CSTM);
	DUMP_REG(SOR_LVDS);
	DUMP_REG(SOR_CRCA);
	DUMP_REG(SOR_CRCB);
	DUMP_REG(SOR_BLANK);
	DUMP_REG(SOR_SEQ_CTL);
	DUMP_REG(SOR_LANE_SEQ_CTL);
	DUMP_REG(SOR_SEQ_INST(0));
	DUMP_REG(SOR_SEQ_INST(1));
	DUMP_REG(SOR_SEQ_INST(2));
	DUMP_REG(SOR_SEQ_INST(3));
	DUMP_REG(SOR_SEQ_INST(4));
	DUMP_REG(SOR_SEQ_INST(5));
	DUMP_REG(SOR_SEQ_INST(6));
	DUMP_REG(SOR_SEQ_INST(7));
	DUMP_REG(SOR_SEQ_INST(8));
	DUMP_REG(SOR_SEQ_INST(9));
	DUMP_REG(SOR_SEQ_INST(10));
	DUMP_REG(SOR_SEQ_INST(11));
	DUMP_REG(SOR_SEQ_INST(12));
	DUMP_REG(SOR_SEQ_INST(13));
	DUMP_REG(SOR_SEQ_INST(14));
	DUMP_REG(SOR_SEQ_INST(15));
	DUMP_REG(SOR_PWM_DIV);
	DUMP_REG(SOR_PWM_CTL);
	DUMP_REG(SOR_VCRC_A0);
	DUMP_REG(SOR_VCRC_A1);
	DUMP_REG(SOR_VCRC_B0);
	DUMP_REG(SOR_VCRC_B1);
	DUMP_REG(SOR_CCRC_A0);
	DUMP_REG(SOR_CCRC_A1);
	DUMP_REG(SOR_CCRC_B0);
	DUMP_REG(SOR_CCRC_B1);
	DUMP_REG(SOR_EDATA_A0);
	DUMP_REG(SOR_EDATA_A1);
	DUMP_REG(SOR_EDATA_B0);
	DUMP_REG(SOR_EDATA_B1);
	DUMP_REG(SOR_COUNT_A0);
	DUMP_REG(SOR_COUNT_A1);
	DUMP_REG(SOR_COUNT_B0);
	DUMP_REG(SOR_COUNT_B1);
	DUMP_REG(SOR_DEBUG_A0);
	DUMP_REG(SOR_DEBUG_A1);
	DUMP_REG(SOR_DEBUG_B0);
	DUMP_REG(SOR_DEBUG_B1);
	DUMP_REG(SOR_TRIG);
	DUMP_REG(SOR_MSCHECK);
	DUMP_REG(SOR_XBAR_CTRL);
	DUMP_REG(SOR_XBAR_POL);
	DUMP_REG(SOR_DP_LINKCTL0);
	DUMP_REG(SOR_DP_LINKCTL1);
	DUMP_REG(SOR_LANE_DRIVE_CURRENT0);
	DUMP_REG(SOR_LANE_DRIVE_CURRENT1);
	DUMP_REG(SOR_LANE4_DRIVE_CURRENT0);
	DUMP_REG(SOR_LANE4_DRIVE_CURRENT1);
	DUMP_REG(SOR_LANE_PREEMPHASIS0);
	DUMP_REG(SOR_LANE_PREEMPHASIS1);
	DUMP_REG(SOR_LANE4_PREEMPHASIS0);
	DUMP_REG(SOR_LANE4_PREEMPHASIS1);
	DUMP_REG(SOR_LANE_POSTCURSOR0);
	DUMP_REG(SOR_LANE_POSTCURSOR1);
	DUMP_REG(SOR_DP_CONFIG0);
	DUMP_REG(SOR_DP_CONFIG1);
	DUMP_REG(SOR_DP_MN0);
	DUMP_REG(SOR_DP_MN1);
	DUMP_REG(SOR_DP_PADCTL0);
	DUMP_REG(SOR_DP_PADCTL1);
	DUMP_REG(SOR_DP_DEBUG0);
	DUMP_REG(SOR_DP_DEBUG1);
	DUMP_REG(SOR_DP_SPARE0);
	DUMP_REG(SOR_DP_SPARE1);
	DUMP_REG(SOR_DP_AUDIO_CTRL);
	DUMP_REG(SOR_DP_AUDIO_HBLANK_SYMBOLS);
	DUMP_REG(SOR_DP_AUDIO_VBLANK_SYMBOLS);
	DUMP_REG(SOR_DP_GENERIC_INFOFRAME_HEADER);
	DUMP_REG(SOR_DP_GENERIC_INFOFRAME_SUBPACK0);
	DUMP_REG(SOR_DP_GENERIC_INFOFRAME_SUBPACK1);
	DUMP_REG(SOR_DP_GENERIC_INFOFRAME_SUBPACK2);
	DUMP_REG(SOR_DP_GENERIC_INFOFRAME_SUBPACK3);
	DUMP_REG(SOR_DP_GENERIC_INFOFRAME_SUBPACK4);
	DUMP_REG(SOR_DP_GENERIC_INFOFRAME_SUBPACK5);
	DUMP_REG(SOR_DP_GENERIC_INFOFRAME_SUBPACK6);
	DUMP_REG(SOR_DP_TPG);
	DUMP_REG(SOR_DP_TPG_CONFIG);
	DUMP_REG(SOR_DP_LQ_CSTM0);
	DUMP_REG(SOR_DP_LQ_CSTM1);
	DUMP_REG(SOR_DP_LQ_CSTM2);

#undef DUMP_REG

unlock:
	drm_modeset_unlock_all(drm);
	return err;
}

static const struct drm_info_list debugfs_files[] = {
	{ "crc", tegra_sor_show_crc, 0, NULL },
	{ "regs", tegra_sor_show_regs, 0, NULL },
};

static int tegra_sor_debugfs_init(struct tegra_sor *sor,
				  struct drm_minor *minor)
{
	const char *name = sor->soc->supports_dp ? "sor1" : "sor";
	unsigned int i;
	int err;

	sor->debugfs = debugfs_create_dir(name, minor->debugfs_root);
	if (!sor->debugfs)
		return -ENOMEM;

	sor->debugfs_files = kmemdup(debugfs_files, sizeof(debugfs_files),
				     GFP_KERNEL);
	if (!sor->debugfs_files) {
		err = -ENOMEM;
		goto remove;
	}

	for (i = 0; i < ARRAY_SIZE(debugfs_files); i++)
		sor->debugfs_files[i].data = sor;

	err = drm_debugfs_create_files(sor->debugfs_files,
				       ARRAY_SIZE(debugfs_files),
				       sor->debugfs, minor);
	if (err < 0)
		goto free;

	sor->minor = minor;

	return 0;

free:
	kfree(sor->debugfs_files);
	sor->debugfs_files = NULL;
remove:
	debugfs_remove_recursive(sor->debugfs);
	sor->debugfs = NULL;
	return err;
}

static void tegra_sor_debugfs_exit(struct tegra_sor *sor)
{
	drm_debugfs_remove_files(sor->debugfs_files, ARRAY_SIZE(debugfs_files),
				 sor->minor);
	sor->minor = NULL;

	kfree(sor->debugfs_files);
	sor->debugfs_files = NULL;

	debugfs_remove_recursive(sor->debugfs);
	sor->debugfs = NULL;
}

static void tegra_sor_connector_reset(struct drm_connector *connector)
{
	struct tegra_sor_state *state;

	kfree(connector->state);
	connector->state = NULL;

	state = kzalloc(sizeof(*state), GFP_KERNEL);
	if (state)
		connector->state = &state->base;
}

static enum drm_connector_status
tegra_sor_connector_detect(struct drm_connector *connector, bool force)
{
	struct tegra_output *output = connector_to_output(connector);
	struct tegra_sor *sor = to_sor(output);

	if (sor->aux)
		return drm_dp_aux_detect(sor->aux);

	return tegra_output_connector_detect(connector, force);
}

static struct drm_connector_state *
tegra_sor_connector_duplicate_state(struct drm_connector *connector)
{
	struct tegra_sor_state *state = to_sor_state(connector->state);
	struct tegra_sor_state *copy;

	copy = kmemdup(state, sizeof(*state), GFP_KERNEL);
	if (!copy)
		return NULL;

	return &copy->base;
}

static const struct drm_connector_funcs tegra_sor_connector_funcs = {
	.dpms = drm_atomic_helper_connector_dpms,
	.reset = tegra_sor_connector_reset,
	.detect = tegra_sor_connector_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = tegra_output_connector_destroy,
	.atomic_duplicate_state = tegra_sor_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static int tegra_sor_connector_get_modes(struct drm_connector *connector)
{
	struct tegra_output *output = connector_to_output(connector);
	struct tegra_sor *sor = to_sor(output);
	int err;

	if (sor->aux)
		drm_dp_aux_enable(sor->aux);

	err = tegra_output_connector_get_modes(connector);

	if (sor->aux)
		drm_dp_aux_disable(sor->aux);

	return err;
}

static enum drm_mode_status
tegra_sor_connector_mode_valid(struct drm_connector *connector,
			       struct drm_display_mode *mode)
{
	return MODE_OK;
}

static const struct drm_connector_helper_funcs tegra_sor_connector_helper_funcs = {
	.get_modes = tegra_sor_connector_get_modes,
	.mode_valid = tegra_sor_connector_mode_valid,
	.best_encoder = tegra_output_connector_best_encoder,
};

static const struct drm_encoder_funcs tegra_sor_encoder_funcs = {
	.destroy = tegra_output_encoder_destroy,
};

static int tegra_sor_hdmi_hdcp(struct tegra_sor *sor)
{
	unsigned long weight;
	u64 aksv, bksv, an;
	size_t size, i, j;
	u8 caps, info = 0;
	ssize_t err;
	void *keys;
	u32 value;
	u16 ri;

	err = drm_hdcp_read_caps(&sor->hdcp, &caps);
	if (err < 0) {
		dev_err(sor->dev, "failed to read HDCP capabilities: %zd\n",
			err);
		return err;
	}

	if (caps & HDCP_CAPS_1_1)
		info = HDCP_INFO_1_1;

	err = drm_hdcp_write_info(&sor->hdcp, info);
	if (err < 0) {
		dev_err(sor->dev, "failed to write info: %zd\n", err);
		return err;
	}

	err = tegra_kfuse_read(sor->kfuse, NULL, 0);
	if (err < 0) {
		dev_err(sor->dev, "failed to query KFUSE size: %zd\n",
			err);
		return err;
	}

	size = err;

	keys = kmalloc(size, GFP_KERNEL);
	if (!keys)
		return -ENOMEM;

	err = tegra_kfuse_read(sor->kfuse, keys, size);
	if (err < 0) {
		dev_err(sor->dev, "failed to read HDMI keys: %zd\n",
			err);
		kfree(keys);
		return err;
	}

	if (err != size) {
		dev_err(sor->dev, "not enough data available from KFUSE: got %zd, requested %zu\n", err, size);
		kfree(keys);
		return err;
	}

	dev_info(sor->dev, "  %zu bytes loaded from HDMI key store\n",
		 size);

	print_hex_dump(KERN_INFO, "", DUMP_PREFIX_OFFSET, 16, 1, keys, size,
		       true);

	value = tegra_sor_readl(sor, SOR_KEY_CTRL);
	value &= ~SOR_KEY_CTRL_LOAD_ADDRESS_MASK;
	value |= SOR_KEY_CTRL_LOCAL_KEYS;
	tegra_sor_writel(sor, value, SOR_KEY_CTRL);

	value = tegra_sor_readl(sor, SOR_KEY_CTRL);
	value |= SOR_KEY_CTRL_PKEY_REQUEST_RELOAD;
	tegra_sor_writel(sor, value, SOR_KEY_CTRL);

	while (true) {
		value = tegra_sor_readl(sor, SOR_KEY_CTRL);
		if (value & SOR_KEY_CTRL_PKEY_LOADED)
			break;

		usleep_range(100, 1000);
	}

	dev_info(sor->dev, "  private key loaded\n");

	/*
	 * XXX introduce enable/disable API for KFUSE so that it can
	 * be disabled here since it's no longer used?
	 */

	for (i = 0; i < size / 4; i += 4) {
		for (j = 0; j < 4; j++) {
			memcpy(&value, keys + (i + j) * 4, 4);
			tegra_sor_writel(sor, value, SOR_KEY_HDCP_KEY_0 + j);
		}

		value = tegra_sor_readl(sor, SOR_KEY_HDCP_KEY_TRIG);
		value |= SOR_KEY_HDCP_KEY_TRIG_LOAD;
		tegra_sor_writel(sor, value, SOR_KEY_HDCP_KEY_TRIG);

		value = tegra_sor_readl(sor, SOR_KEY_CTRL);

		if (i == 0)
			value &= ~SOR_KEY_CTRL_AUTOINC;
		else
			value |= SOR_KEY_CTRL_AUTOINC;

		value |= SOR_KEY_CTRL_WRITE16;

		tegra_sor_writel(sor, value, SOR_KEY_CTRL);

		while (true) {
			value = tegra_sor_readl(sor, SOR_KEY_CTRL);
			if ((value & SOR_KEY_CTRL_WRITE16) == 0)
				break;

			usleep_range(100, 1000);
		}
	}

	dev_info(sor->dev, "  keys loaded to HDMI SRAM\n");
	kfree(keys);

	tegra_sor_writel(sor, 0, SOR_KEY_SKEY_INDEX);

	dev_info(sor->dev, "  key 0 selected\n");

	value = tegra_sor_readl(sor, SOR_TMDS_HDCP_CTRL);
	value |= SOR_TMDS_HDCP_CTRL_RUN;
	tegra_sor_writel(sor, value, SOR_TMDS_HDCP_CTRL);

	while (true) {
		value = tegra_sor_readl(sor, SOR_TMDS_HDCP_CTRL);
		if (value & SOR_TMDS_HDCP_CTRL_AN)
			break;

		usleep_range(100, 1000);
	}

	value = tegra_sor_readl(sor, SOR_TMDS_HDCP_AKSV_MSB);
	aksv = (u64)value << 32;

	value = tegra_sor_readl(sor, SOR_TMDS_HDCP_AKSV_LSB);
	aksv |= value;

	dev_info(sor->dev, "  AN ready: aksv: %llx\n", aksv);
	weight = hweight64(aksv);

	if (weight == 20)
		dev_info(sor->dev, "    valid\n");
	else
		dev_info(sor->dev, "    invalid: weight: %lu\n", weight);

	value = tegra_sor_readl(sor, SOR_TMDS_HDCP_AN_MSB);
	an = (u64)value << 32;

	value = tegra_sor_readl(sor, SOR_TMDS_HDCP_AN_LSB);
	an |= value;

	dev_info(sor->dev, "AN: %llx\n", an);

	err = drm_hdcp_write_an(&sor->hdcp, an);
	if (err < 0) {
		dev_err(sor->dev, "failed to write AN to HDCP: %zd\n", err);
	}

	err = drm_hdcp_write_aksv(&sor->hdcp, aksv);
	if (err < 0) {
		dev_err(sor->dev, "failed to write AKSV to HDCP: %zd\n", err);
	}

	err = drm_hdcp_read_bksv(&sor->hdcp, &bksv);
	if (err < 0) {
		dev_err(sor->dev, "failed to read BKSV: %zd\n", err);
		return err;
	}

	dev_info(sor->dev, "  BKSV: %llx\n", bksv);

	weight = hweight64(bksv);

	if (weight == 20)
		dev_info(sor->dev, "    valid\n");
	else
		dev_info(sor->dev, "    invalid: weight: %lu\n", weight);

	/* LSB must be written first */
	value = bksv & 0xffffffff;
	tegra_sor_writel(sor, value, SOR_TMDS_HDCP_BKSV_LSB);

	value = (bksv >> 32) & 0xff;
	tegra_sor_writel(sor, value, SOR_TMDS_HDCP_BKSV_MSB);

	while (true) {
		value = tegra_sor_readl(sor, SOR_TMDS_HDCP_CTRL);
		if (value & SOR_TMDS_HDCP_CTRL_R0)
			break;

		usleep_range(100, 1000);
	}

	value = tegra_sor_readl(sor, SOR_TMDS_HDCP_RI);
	dev_info(sor->dev, "  RI: %04x\n", value);

	/* can't read Ri until 100 ms after Aksv was written */
	msleep(100);

	err = drm_hdcp_read_ri(&sor->hdcp, &ri);
	if (err < 0) {
		dev_err(sor->dev, "failed to read RI from HDCP: %zd\n", err);
	}

	dev_info(sor->dev, "  RI: %04x\n", ri);

	if (ri != value) {
		dev_err(sor->dev, "Ri doesn't match: %04x != %04x\n", ri, value);
		return -EIO;
	}

	value = tegra_sor_readl(sor, SOR_TMDS_HDCP_CTRL);

	if (caps & HDCP_CAPS_1_1)
		value |= SOR_TMDS_HDCP_CTRL_ONE_ONE;

	value |= SOR_TMDS_HDCP_CTRL_CRYPT;
	tegra_sor_writel(sor, value, SOR_TMDS_HDCP_CTRL);

	return 0;
}

static void tegra_sor_edp_disable(struct drm_encoder *encoder)
{
	struct tegra_output *output = encoder_to_output(encoder);
	struct tegra_dc *dc = to_tegra_dc(encoder->crtc);
	struct tegra_sor *sor = to_sor(output);
	u32 value;
	int err;

	if (output->panel)
		drm_panel_disable(output->panel);

	err = tegra_sor_detach(sor);
	if (err < 0)
		dev_err(sor->dev, "failed to detach SOR: %d\n", err);

	tegra_sor_writel(sor, 0, SOR_STATE1);
	tegra_sor_update(sor);

	value = tegra_dc_readl(dc, DC_DISP_DISP_WIN_OPTIONS);
	value &= ~SOR_ENABLE;
	tegra_dc_writel(dc, value, DC_DISP_DISP_WIN_OPTIONS);

	tegra_dc_commit(dc);

	err = tegra_sor_power_down(sor);
	if (err < 0)
		dev_err(sor->dev, "failed to power down SOR: %d\n", err);

	err = drm_dp_aux_disable(sor->aux);
	if (err < 0)
		dev_err(sor->dev, "failed to disable DP: %d\n", err);

	err = tegra_io_rail_power_off(TEGRA_IO_RAIL_LVDS);
	if (err < 0)
		dev_err(sor->dev, "failed to power off I/O rail: %d\n", err);

	if (output->panel)
		drm_panel_unprepare(output->panel);

	reset_control_assert(sor->rst);
	usleep_range(1000, 2000);
	clk_disable_unprepare(sor->clk);
}

#if 0
static int calc_h_ref_to_sync(const struct drm_display_mode *mode,
			      unsigned int *value)
{
	unsigned int hfp, hsw, hbp, a = 0, b;

	hfp = mode->hsync_start - mode->hdisplay;
	hsw = mode->hsync_end - mode->hsync_start;
	hbp = mode->htotal - mode->hsync_end;

	pr_info("hfp: %u, hsw: %u, hbp: %u\n", hfp, hsw, hbp);

	b = hfp - 1;

	pr_info("a: %u, b: %u\n", a, b);
	pr_info("a + hsw + hbp = %u\n", a + hsw + hbp);

	if (a + hsw + hbp <= 11) {
		a = 1 + 11 - hsw - hbp;
		pr_info("a: %u\n", a);
	}

	if (a > b)
		return -EINVAL;

	if (hsw < 1)
		return -EINVAL;

	if (mode->hdisplay < 16)
		return -EINVAL;

	if (value) {
		if (b > a && a % 2)
			*value = a + 1;
		else
			*value = a;
	}

	return 0;
}
#endif

static void tegra_sor_edp_enable(struct drm_encoder *encoder)
{
	struct tegra_output *output = encoder_to_output(encoder);
	struct tegra_dc *dc = to_tegra_dc(encoder->crtc);
	struct tegra_sor *sor = to_sor(output);
	struct tegra_sor_config config;
	struct tegra_sor_state *state;
	struct drm_display_mode *mode;
	struct drm_display_info *info;
	unsigned int i;
	u32 value;
	int err;

	state = to_sor_state(output->connector.state);
	mode = &encoder->crtc->state->adjusted_mode;
	info = &output->connector.display_info;

	err = clk_prepare_enable(sor->clk);
	if (err < 0)
		dev_err(sor->dev, "failed to enable clock: %d\n", err);

	usleep_range(1000, 2000);

	reset_control_deassert(sor->rst);

	/* switch to safe parent clock */
	err = tegra_sor_set_parent_clock(sor, sor->clk_safe);
	if (err < 0)
		dev_err(sor->dev, "failed to set safe parent clock: %d\n", err);

	err = tegra_io_rail_power_on(TEGRA_IO_RAIL_LVDS);
	if (err < 0)
		dev_err(sor->dev, "failed to power on LVDS rail: %d\n", err);

	usleep_range(20, 100);

	drm_dp_aux_enable(sor->aux);

	err = drm_dp_link_probe(sor->aux, &sor->link);
	if (err < 0) {
		dev_err(sor->dev, "failed to probe eDP link: %d\n", err);
		goto out;
	}

	err = drm_dp_link_choose(&sor->link, mode, info);
	if (err < 0) {
		dev_err(sor->dev, "failed to choose link: %d\n", err);
		goto out;
	}

	if (output->panel)
		drm_panel_prepare(output->panel);

	value = tegra_sor_readl(sor, SOR_PLL2);
	value &= ~SOR_PLL2_BANDGAP_POWERDOWN;
	tegra_sor_writel(sor, value, SOR_PLL2);

	usleep_range(20, 40);

	value = tegra_sor_readl(sor, SOR_PLL3);
	value |= SOR_PLL3_PLL_VDD_MODE_3V3;
	tegra_sor_writel(sor, value, SOR_PLL3);

	value = tegra_sor_readl(sor, SOR_PLL0);
	value &= ~(SOR_PLL0_VCOPD | SOR_PLL0_PWR);
	tegra_sor_writel(sor, value, SOR_PLL0);

	value = tegra_sor_readl(sor, SOR_PLL2);
	value &= ~SOR_PLL2_SEQ_PLLCAPPD_ENFORCE;
	value |= SOR_PLL2_SEQ_PLLCAPPD;
	tegra_sor_writel(sor, value, SOR_PLL2);

	usleep_range(200, 400);

	value = tegra_sor_readl(sor, SOR_PLL2);
	value &= ~SOR_PLL2_POWERDOWN_OVERRIDE;
	value &= ~SOR_PLL2_PORT_POWERDOWN;
	tegra_sor_writel(sor, value, SOR_PLL2);

	value = tegra_sor_readl(sor, SOR_CLK_CNTRL);
	value &= ~SOR_CLK_CNTRL_DP_CLK_SEL_MASK;
	value |= SOR_CLK_CNTRL_DP_CLK_SEL_SINGLE_DPCLK;
	tegra_sor_writel(sor, value, SOR_CLK_CNTRL);

	value = tegra_sor_readl(sor, SOR_DP_SPARE0);
	/* XXX not in TRM */
	value |= SOR_DP_SPARE_PANEL_INTERNAL;
	value |= SOR_DP_SPARE_SEQ_ENABLE;
	tegra_sor_writel(sor, value, SOR_DP_SPARE0);

	/* XXX not in TRM */
	tegra_sor_writel(sor, 0, SOR_LVDS);

	value = tegra_sor_readl(sor, SOR_PLL0);
	value &= ~SOR_PLL0_ICHPMP_MASK;
	value &= ~SOR_PLL0_VCOCAP_MASK;
	value |= SOR_PLL0_ICHPMP(0x1);
	value |= SOR_PLL0_VCOCAP(0x3);
	value |= SOR_PLL0_RESISTOR_EXT;
	tegra_sor_writel(sor, value, SOR_PLL0);

	/* XXX not in TRM */
	for (value = 0, i = 0; i < 5; i++)
		value |= SOR_XBAR_CTRL_LINK0_XSEL(i, sor->soc->xbar_cfg[i]) |
			 SOR_XBAR_CTRL_LINK1_XSEL(i, i);

	tegra_sor_writel(sor, 0x00000000, SOR_XBAR_POL);
	tegra_sor_writel(sor, value, SOR_XBAR_CTRL);

	/* switch to DP parent clock */
	err = tegra_sor_set_parent_clock(sor, sor->clk_dp);
	if (err < 0) {
		dev_err(sor->dev, "failed to set parent clock: %d\n", err);
		goto out;
	}

	/* use DP-A protocol */
	value = tegra_sor_readl(sor, SOR_STATE1);
	value &= ~SOR_STATE_ASY_PROTOCOL_MASK;
	value |= SOR_STATE_ASY_PROTOCOL_DP_A;
	tegra_sor_writel(sor, value, SOR_STATE1);

	/* enable port */
	value = tegra_sor_readl(sor, SOR_DP_LINKCTL0);
	value |= SOR_DP_LINKCTL_ENABLE;
	tegra_sor_writel(sor, value, SOR_DP_LINKCTL0);

	tegra_sor_dp_term_calibrate(sor);

	err = drm_dp_link_train(&sor->link);
	if (err < 0) {
		dev_err(sor->dev, "link training failed: %d\n", err);
		goto out;
	}

	dev_dbg(sor->dev, "link training succeeded\n");

	err = drm_dp_link_power_up(sor->aux, &sor->link);
	if (err < 0) {
		dev_err(sor->dev, "failed to power up eDP link: %d\n", err);
		goto out;
	}

	/* compute configuration */
	memset(&config, 0, sizeof(config));
	config.bits_per_pixel = state->bpc * 3;

	err = tegra_sor_compute_config(sor, mode, &config, &sor->link);
	if (err < 0) {
		dev_err(sor->dev, "failed to compute configuration: %d\n", err);
		goto out;
	}

	tegra_sor_apply_config(sor, &config);
	tegra_sor_mode_set(sor, mode, state);

	/* PWM setup */
	err = tegra_sor_setup_pwm(sor, 250);
	if (err < 0)
		dev_err(sor->dev, "failed to setup PWM: %d\n", err);

	tegra_sor_update(sor);

	err = tegra_sor_power_up(sor, 250);
	if (err < 0)
		dev_err(sor->dev, "failed to power up SOR: %d\n", err);

	/* attach and wake up */
	err = tegra_sor_attach(sor);
	if (err < 0)
		dev_err(sor->dev, "failed to attach SOR: %d\n", err);

	value = tegra_dc_readl(dc, DC_DISP_DISP_WIN_OPTIONS);
	value |= SOR_ENABLE;
	tegra_dc_writel(dc, value, DC_DISP_DISP_WIN_OPTIONS);

	tegra_dc_commit(dc);

	err = tegra_sor_wakeup(sor);
	if (err < 0)
		dev_err(sor->dev, "failed to wakeup SOR: %d\n", err);

	if (output->panel)
		drm_panel_enable(output->panel);

out:
	drm_dp_aux_disable(sor->aux);
}

static int
tegra_sor_encoder_atomic_check(struct drm_encoder *encoder,
			       struct drm_crtc_state *crtc_state,
			       struct drm_connector_state *conn_state)
{
	struct tegra_output *output = encoder_to_output(encoder);
	struct tegra_sor_state *state = to_sor_state(conn_state);
	struct tegra_dc *dc = to_tegra_dc(conn_state->crtc);
	unsigned long pclk = crtc_state->mode.clock * 1000;
	struct tegra_sor *sor = to_sor(output);
	struct drm_display_info *info;
	int err;

	info = &output->connector.display_info;

	err = tegra_dc_state_setup_clock(dc, crtc_state, sor->clk_parent,
					 pclk, 0);
	if (err < 0) {
		dev_err(output->dev, "failed to setup CRTC state: %d\n", err);
		return err;
	}

	switch (info->bpc) {
	case 8:
	case 6:
		state->bpc = info->bpc;
		break;

	default:
		DRM_DEBUG_KMS("%u bits-per-color not supported\n", info->bpc);
		state->bpc = 8;
		break;
	}

	return 0;
}

static const struct drm_encoder_helper_funcs tegra_sor_edp_helpers = {
	.disable = tegra_sor_edp_disable,
	.enable = tegra_sor_edp_enable,
	.atomic_check = tegra_sor_encoder_atomic_check,
};

static inline u32 tegra_sor_hdmi_subpack(const u8 *ptr, size_t size)
{
	u32 value = 0;
	size_t i;

	for (i = size; i > 0; i--)
		value = (value << 8) | ptr[i - 1];

	return value;
}

static void tegra_sor_hdmi_write_infopack(struct tegra_sor *sor,
					  const void *data, size_t size)
{
	const u8 *ptr = data;
	unsigned long offset;
	size_t i, j;
	u32 value;

	switch (ptr[0]) {
	case HDMI_INFOFRAME_TYPE_AVI:
		offset = SOR_HDMI_AVI_INFOFRAME_HEADER;
		break;

	case HDMI_INFOFRAME_TYPE_AUDIO:
		offset = SOR_HDMI_AUDIO_INFOFRAME_HEADER;
		break;

	case HDMI_INFOFRAME_TYPE_VENDOR:
		offset = SOR_HDMI_VSI_INFOFRAME_HEADER;
		break;

	default:
		dev_err(sor->dev, "unsupported infoframe type: %02x\n",
			ptr[0]);
		return;
	}

	value = INFOFRAME_HEADER_TYPE(ptr[0]) |
		INFOFRAME_HEADER_VERSION(ptr[1]) |
		INFOFRAME_HEADER_LEN(ptr[2]);
	tegra_sor_writel(sor, value, offset);
	offset++;

	/*
	 * Each subpack contains 7 bytes, divided into:
	 * - subpack_low: bytes 0 - 3
	 * - subpack_high: bytes 4 - 6 (with byte 7 padded to 0x00)
	 */
	for (i = 3, j = 0; i < size; i += 7, j += 8) {
		size_t rem = size - i, num = min_t(size_t, rem, 4);

		value = tegra_sor_hdmi_subpack(&ptr[i], num);
		tegra_sor_writel(sor, value, offset++);

		num = min_t(size_t, rem - num, 3);

		value = tegra_sor_hdmi_subpack(&ptr[i + 4], num);
		tegra_sor_writel(sor, value, offset++);
	}
}

static int
tegra_sor_hdmi_setup_avi_infoframe(struct tegra_sor *sor,
				   const struct drm_display_mode *mode)
{
	u8 buffer[HDMI_INFOFRAME_SIZE(AVI)];
	struct hdmi_avi_infoframe frame;
	u32 value;
	int err;

	/* disable AVI infoframe */
	value = tegra_sor_readl(sor, SOR_HDMI_AVI_INFOFRAME_CTRL);
	value &= ~INFOFRAME_CTRL_SINGLE;
	value &= ~INFOFRAME_CTRL_OTHER;
	value &= ~INFOFRAME_CTRL_ENABLE;
	tegra_sor_writel(sor, value, SOR_HDMI_AVI_INFOFRAME_CTRL);

	err = drm_hdmi_avi_infoframe_from_display_mode(&frame, mode);
	if (err < 0) {
		dev_err(sor->dev, "failed to setup AVI infoframe: %d\n", err);
		return err;
	}

	err = hdmi_avi_infoframe_pack(&frame, buffer, sizeof(buffer));
	if (err < 0) {
		dev_err(sor->dev, "failed to pack AVI infoframe: %d\n", err);
		return err;
	}

	tegra_sor_hdmi_write_infopack(sor, buffer, err);

	/* enable AVI infoframe */
	value = tegra_sor_readl(sor, SOR_HDMI_AVI_INFOFRAME_CTRL);
	value |= INFOFRAME_CTRL_CHECKSUM_ENABLE;
	value |= INFOFRAME_CTRL_ENABLE;
	tegra_sor_writel(sor, value, SOR_HDMI_AVI_INFOFRAME_CTRL);

	return 0;
}

static void tegra_sor_hdmi_disable_audio_infoframe(struct tegra_sor *sor)
{
	u32 value;

	value = tegra_sor_readl(sor, SOR_HDMI_AUDIO_INFOFRAME_CTRL);
	value &= ~INFOFRAME_CTRL_ENABLE;
	tegra_sor_writel(sor, value, SOR_HDMI_AUDIO_INFOFRAME_CTRL);
}

static struct tegra_sor_hdmi_settings *
tegra_sor_hdmi_find_settings(struct tegra_sor *sor, unsigned long frequency)
{
	unsigned int i;

	for (i = 0; i < sor->num_settings; i++)
		if (frequency <= sor->settings[i].frequency)
			return &sor->settings[i];

	return NULL;
}

static void tegra_sor_hdmi_disable(struct drm_encoder *encoder)
{
	struct tegra_output *output = encoder_to_output(encoder);
	struct tegra_dc *dc = to_tegra_dc(encoder->crtc);
	struct tegra_sor *sor = to_sor(output);
	u32 value;
	int err;

	err = tegra_sor_detach(sor);
	if (err < 0)
		dev_err(sor->dev, "failed to detach SOR: %d\n", err);

	tegra_sor_writel(sor, 0, SOR_STATE1);
	tegra_sor_update(sor);

	/* disable display to SOR clock */
	value = tegra_dc_readl(dc, DC_DISP_DISP_WIN_OPTIONS);
	value &= ~SOR1_TIMING_CYA;
	value &= ~SOR1_ENABLE;
	tegra_dc_writel(dc, value, DC_DISP_DISP_WIN_OPTIONS);

	tegra_dc_commit(dc);

	err = tegra_sor_power_down(sor);
	if (err < 0)
		dev_err(sor->dev, "failed to power down SOR: %d\n", err);

	err = tegra_io_rail_power_off(TEGRA_IO_RAIL_HDMI);
	if (err < 0)
		dev_err(sor->dev, "failed to power off HDMI rail: %d\n", err);

	reset_control_assert(sor->rst);
	usleep_range(1000, 2000);
	clk_disable_unprepare(sor->clk);
}

static void tegra_sor_hdmi_enable(struct drm_encoder *encoder)
{
	struct tegra_output *output = encoder_to_output(encoder);
	unsigned int h_ref_to_sync = 1, pulse_start, max_ac;
	struct tegra_dc *dc = to_tegra_dc(encoder->crtc);
	struct tegra_sor_hdmi_settings *settings;
	struct tegra_sor *sor = to_sor(output);
	struct tegra_sor_state *state;
	struct drm_display_mode *mode;
	unsigned int div, i;
	u32 value;
	int err;

	state = to_sor_state(output->connector.state);
	mode = &encoder->crtc->state->adjusted_mode;

	err = clk_prepare_enable(sor->clk);
	if (err < 0)
		dev_err(sor->dev, "failed to enable clock: %d\n", err);

	usleep_range(1000, 2000);

	reset_control_deassert(sor->rst);

	/* switch to safe parent clock */
	err = tegra_sor_set_parent_clock(sor, sor->clk_safe);
	if (err < 0)
		dev_err(sor->dev, "failed to set safe parent clock: %d\n", err);

	div = clk_get_rate(sor->clk) / 1000000 * 4;

	err = tegra_io_rail_power_on(TEGRA_IO_RAIL_HDMI);
	if (err < 0)
		dev_err(sor->dev, "failed to power on HDMI rail: %d\n", err);

	usleep_range(20, 100);

	value = tegra_sor_readl(sor, SOR_PLL2);
	value &= ~SOR_PLL2_BANDGAP_POWERDOWN;
	tegra_sor_writel(sor, value, SOR_PLL2);

	usleep_range(20, 100);

	value = tegra_sor_readl(sor, SOR_PLL3);
	value &= ~SOR_PLL3_PLL_VDD_MODE_3V3;
	tegra_sor_writel(sor, value, SOR_PLL3);

	value = tegra_sor_readl(sor, SOR_PLL0);
	value &= ~SOR_PLL0_VCOPD;
	value &= ~SOR_PLL0_PWR;
	tegra_sor_writel(sor, value, SOR_PLL0);

	value = tegra_sor_readl(sor, SOR_PLL2);
	value &= ~SOR_PLL2_SEQ_PLLCAPPD_ENFORCE;
	tegra_sor_writel(sor, value, SOR_PLL2);

	usleep_range(200, 400);

	value = tegra_sor_readl(sor, SOR_PLL2);
	value &= ~SOR_PLL2_POWERDOWN_OVERRIDE;
	value &= ~SOR_PLL2_PORT_POWERDOWN;
	tegra_sor_writel(sor, value, SOR_PLL2);

	usleep_range(20, 100);

	value = tegra_sor_readl(sor, SOR_DP_PADCTL0);
	value |= SOR_DP_PADCTL_PD_TXD_3 | SOR_DP_PADCTL_PD_TXD_0 |
		 SOR_DP_PADCTL_PD_TXD_1 | SOR_DP_PADCTL_PD_TXD_2;
	tegra_sor_writel(sor, value, SOR_DP_PADCTL0);

	while (true) {
		value = tegra_sor_readl(sor, SOR_LANE_SEQ_CTL);
		if ((value & SOR_LANE_SEQ_CTL_STATE_BUSY) == 0)
			break;

		usleep_range(250, 1000);
	}

	value = SOR_LANE_SEQ_CTL_TRIGGER | SOR_LANE_SEQ_CTL_SEQUENCE_DOWN |
		SOR_LANE_SEQ_CTL_POWER_STATE_UP | SOR_LANE_SEQ_CTL_DELAY(5);
	tegra_sor_writel(sor, value, SOR_LANE_SEQ_CTL);

	while (true) {
		value = tegra_sor_readl(sor, SOR_LANE_SEQ_CTL);
		if ((value & SOR_LANE_SEQ_CTL_TRIGGER) == 0)
			break;

		usleep_range(250, 1000);
	}

	value = tegra_sor_readl(sor, SOR_CLK_CNTRL);
	value &= ~SOR_CLK_CNTRL_DP_LINK_SPEED_MASK;
	value &= ~SOR_CLK_CNTRL_DP_CLK_SEL_MASK;

	if (mode->clock < 340000)
		value |= SOR_CLK_CNTRL_DP_LINK_SPEED_G2_70;
	else
		value |= SOR_CLK_CNTRL_DP_LINK_SPEED_G5_40;

	value |= SOR_CLK_CNTRL_DP_CLK_SEL_SINGLE_PCLK;
	tegra_sor_writel(sor, value, SOR_CLK_CNTRL);

	value = tegra_sor_readl(sor, SOR_DP_SPARE0);
	value |= SOR_DP_SPARE_DISP_VIDEO_PREAMBLE;
	value &= ~SOR_DP_SPARE_PANEL_INTERNAL;
	value |= SOR_DP_SPARE_SEQ_ENABLE;
	tegra_sor_writel(sor, value, SOR_DP_SPARE0);

	value = SOR_SEQ_CTL_PU_PC(0) | SOR_SEQ_CTL_PU_PC_ALT(0) |
		SOR_SEQ_CTL_PD_PC(8) | SOR_SEQ_CTL_PD_PC_ALT(8);
	tegra_sor_writel(sor, value, SOR_SEQ_CTL);

	value = SOR_SEQ_INST_DRIVE_PWM_OUT_LO | SOR_SEQ_INST_HALT |
		SOR_SEQ_INST_WAIT_VSYNC | SOR_SEQ_INST_WAIT(1);
	tegra_sor_writel(sor, value, SOR_SEQ_INST(0));
	tegra_sor_writel(sor, value, SOR_SEQ_INST(8));

	/* program the reference clock */
	value = SOR_REFCLK_DIV_INT(div) | SOR_REFCLK_DIV_FRAC(div);
	tegra_sor_writel(sor, value, SOR_REFCLK);

	/* XXX not in TRM */
	for (value = 0, i = 0; i < 5; i++)
		value |= SOR_XBAR_CTRL_LINK0_XSEL(i, sor->soc->xbar_cfg[i]) |
			 SOR_XBAR_CTRL_LINK1_XSEL(i, i);

	tegra_sor_writel(sor, 0x00000000, SOR_XBAR_POL);
	tegra_sor_writel(sor, value, SOR_XBAR_CTRL);

	/* switch to parent clock */
	err = tegra_sor_set_parent_clock(sor, sor->clk_parent);
	if (err < 0)
		dev_err(sor->dev, "failed to set parent clock: %d\n", err);

	value = SOR_INPUT_CONTROL_HDMI_SRC_SELECT(dc->pipe);

	/* XXX is this the proper check? */
	if (mode->clock < 75000)
		value |= SOR_INPUT_CONTROL_ARM_VIDEO_RANGE_LIMITED;

	tegra_sor_writel(sor, value, SOR_INPUT_CONTROL);

	max_ac = ((mode->htotal - mode->hdisplay) - SOR_REKEY - 18) / 32;

	value = SOR_HDMI_CTRL_ENABLE | SOR_HDMI_CTRL_MAX_AC_PACKET(max_ac) |
		SOR_HDMI_CTRL_AUDIO_LAYOUT | SOR_HDMI_CTRL_REKEY(SOR_REKEY);
	tegra_sor_writel(sor, value, SOR_HDMI_CTRL);

	/* H_PULSE2 setup */
	pulse_start = h_ref_to_sync + (mode->hsync_end - mode->hsync_start) +
		      (mode->htotal - mode->hsync_end) - 10;

	value = PULSE_LAST_END_A | PULSE_QUAL_VACTIVE |
		PULSE_POLARITY_HIGH | PULSE_MODE_NORMAL;
	tegra_dc_writel(dc, value, DC_DISP_H_PULSE2_CONTROL);

	value = PULSE_END(pulse_start + 8) | PULSE_START(pulse_start);
	tegra_dc_writel(dc, value, DC_DISP_H_PULSE2_POSITION_A);

	value = tegra_dc_readl(dc, DC_DISP_DISP_SIGNAL_OPTIONS0);
	value |= H_PULSE2_ENABLE;
	tegra_dc_writel(dc, value, DC_DISP_DISP_SIGNAL_OPTIONS0);

	/* infoframe setup */
	err = tegra_sor_hdmi_setup_avi_infoframe(sor, mode);
	if (err < 0)
		dev_err(sor->dev, "failed to setup AVI infoframe: %d\n", err);

	/* XXX HDMI audio support not implemented yet */
	tegra_sor_hdmi_disable_audio_infoframe(sor);

	/* use single TMDS protocol */
	value = tegra_sor_readl(sor, SOR_STATE1);
	value &= ~SOR_STATE_ASY_PROTOCOL_MASK;
	value |= SOR_STATE_ASY_PROTOCOL_SINGLE_TMDS_A;
	tegra_sor_writel(sor, value, SOR_STATE1);

	/* power up pad calibration */
	value = tegra_sor_readl(sor, SOR_DP_PADCTL0);
	value &= ~SOR_DP_PADCTL_PAD_CAL_PD;
	tegra_sor_writel(sor, value, SOR_DP_PADCTL0);

	/* production settings */
	settings = tegra_sor_hdmi_find_settings(sor, mode->clock * 1000);
	if (IS_ERR(settings)) {
		dev_err(sor->dev, "no settings for pixel clock %d Hz: %ld\n",
			mode->clock * 1000, PTR_ERR(settings));
		return;
	}

	value = tegra_sor_readl(sor, SOR_PLL0);
	value &= ~SOR_PLL0_ICHPMP_MASK;
	value &= ~SOR_PLL0_VCOCAP_MASK;
	value |= SOR_PLL0_ICHPMP(settings->ichpmp);
	value |= SOR_PLL0_VCOCAP(settings->vcocap);
	tegra_sor_writel(sor, value, SOR_PLL0);

	tegra_sor_dp_term_calibrate(sor);

	value = tegra_sor_readl(sor, SOR_PLL1);
	value &= ~SOR_PLL1_LOADADJ_MASK;
	value |= SOR_PLL1_LOADADJ(settings->loadadj);
	tegra_sor_writel(sor, value, SOR_PLL1);

	value = tegra_sor_readl(sor, SOR_PLL3);
	value &= ~SOR_PLL3_BG_VREF_LEVEL_MASK;
	value |= SOR_PLL3_BG_VREF_LEVEL(settings->bg_vref);
	tegra_sor_writel(sor, value, SOR_PLL3);

	value = settings->drive_current[0] << 24 |
		settings->drive_current[1] << 16 |
		settings->drive_current[2] <<  8 |
		settings->drive_current[3] <<  0;
	tegra_sor_writel(sor, value, SOR_LANE_DRIVE_CURRENT0);

	value = settings->preemphasis[0] << 24 |
		settings->preemphasis[1] << 16 |
		settings->preemphasis[2] <<  8 |
		settings->preemphasis[3] <<  0;
	tegra_sor_writel(sor, value, SOR_LANE_PREEMPHASIS0);

	value = tegra_sor_readl(sor, SOR_DP_PADCTL0);
	value &= ~SOR_DP_PADCTL_TX_PU_MASK;
	value |= SOR_DP_PADCTL_TX_PU_ENABLE;
	value |= SOR_DP_PADCTL_TX_PU(settings->tx_pu);
	tegra_sor_writel(sor, value, SOR_DP_PADCTL0);

	/* power down pad calibration */
	value = tegra_sor_readl(sor, SOR_DP_PADCTL0);
	value |= SOR_DP_PADCTL_PAD_CAL_PD;
	tegra_sor_writel(sor, value, SOR_DP_PADCTL0);

	/* miscellaneous display controller settings */
	value = VSYNC_H_POSITION(1);
	tegra_dc_writel(dc, value, DC_DISP_DISP_TIMING_OPTIONS);

	value = tegra_dc_readl(dc, DC_DISP_DISP_COLOR_CONTROL);
	value &= ~DITHER_CONTROL_MASK;
	value &= ~BASE_COLOR_SIZE_MASK;

	switch (state->bpc) {
	case 6:
		value |= BASE_COLOR_SIZE_666;
		break;

	case 8:
		value |= BASE_COLOR_SIZE_888;
		break;

	default:
		WARN(1, "%u bits-per-color not supported\n", state->bpc);
		value |= BASE_COLOR_SIZE_888;
		break;
	}

	tegra_dc_writel(dc, value, DC_DISP_DISP_COLOR_CONTROL);

	err = tegra_sor_power_up(sor, 250);
	if (err < 0)
		dev_err(sor->dev, "failed to power up SOR: %d\n", err);

	/* configure dynamic range of output */
	value = tegra_sor_readl(sor, SOR_HEAD_STATE0(dc->pipe));
	value &= ~SOR_HEAD_STATE_RANGECOMPRESS_MASK;
	value &= ~SOR_HEAD_STATE_DYNRANGE_MASK;
	tegra_sor_writel(sor, value, SOR_HEAD_STATE0(dc->pipe));

	/* configure colorspace */
	value = tegra_sor_readl(sor, SOR_HEAD_STATE0(dc->pipe));
	value &= ~SOR_HEAD_STATE_COLORSPACE_MASK;
	value |= SOR_HEAD_STATE_COLORSPACE_RGB;
	tegra_sor_writel(sor, value, SOR_HEAD_STATE0(dc->pipe));

	tegra_sor_mode_set(sor, mode, state);

	tegra_sor_update(sor);

	err = tegra_sor_attach(sor);
	if (err < 0)
		dev_err(sor->dev, "failed to attach SOR: %d\n", err);

	/* enable display to SOR clock and generate HDMI preamble */
	value = tegra_dc_readl(dc, DC_DISP_DISP_WIN_OPTIONS);
	value |= SOR1_ENABLE | SOR1_TIMING_CYA;
	tegra_dc_writel(dc, value, DC_DISP_DISP_WIN_OPTIONS);

	tegra_dc_commit(dc);

	err = tegra_sor_wakeup(sor);
	if (err < 0)
		dev_err(sor->dev, "failed to wakeup SOR: %d\n", err);

	if (sor->kfuse)
		tegra_sor_hdmi_hdcp(sor);
}

static const struct drm_encoder_helper_funcs tegra_sor_hdmi_helpers = {
	.disable = tegra_sor_hdmi_disable,
	.enable = tegra_sor_hdmi_enable,
	.atomic_check = tegra_sor_encoder_atomic_check,
};

static void tegra_sor_dp_disable(struct drm_encoder *encoder)
{
	struct tegra_output *output = encoder_to_output(encoder);
	struct tegra_dc *dc = to_tegra_dc(encoder->crtc);
	struct tegra_sor *sor = to_sor(output);
	u32 value;
	int err;

	drm_dp_aux_enable(sor->aux);

	err = drm_dp_link_power_down(sor->aux, &sor->link);
	if (err < 0)
		dev_err(sor->dev, "failed to power down link: %d\n", err);

	drm_dp_aux_disable(sor->aux);

	err = tegra_sor_detach(sor);
	if (err < 0)
		dev_err(sor->dev, "failed to detach SOR: %d\n", err);

	value = tegra_dc_readl(dc, DC_DISP_DISP_WIN_OPTIONS);
	value &= ~SOR1_ENABLE;
	tegra_dc_writel(dc, value, DC_DISP_DISP_WIN_OPTIONS);
	tegra_dc_commit(dc);

	value = tegra_sor_readl(sor, SOR_STATE1);
	value &= ~SOR_STATE_ASY_PROTOCOL_MASK;
	value &= ~SOR_STATE_ASY_SUBOWNER_MASK;
	value &= ~SOR_STATE_ASY_OWNER_MASK;
	tegra_sor_writel(sor, value, SOR_STATE1);
	tegra_sor_update(sor);

	/* switch to safe parent clock */
	err = tegra_sor_set_parent_clock(sor, sor->clk_safe);
	if (err < 0)
		dev_err(sor->dev, "failed to set safe clock: %d\n", err);

	err = tegra_sor_power_down(sor);
	if (err < 0)
		dev_err(sor->dev, "failed to power down SOR: %d\n", err);

	usleep_range(25, 100);

	err = tegra_io_rail_power_off(TEGRA_IO_RAIL_LVDS);
	if (err < 0)
		dev_err(sor->dev, "failed to power off LVDS rail: %d\n", err);

	usleep_range(25, 100);

	reset_control_assert(sor->rst);
	usleep_range(1000, 2000);
	clk_disable_unprepare(sor->clk);
}

static void tegra_sor_dp_enable(struct drm_encoder *encoder)
{
	struct tegra_output *output = encoder_to_output(encoder);
	struct tegra_dc *dc = to_tegra_dc(encoder->crtc);
	struct tegra_sor *sor = to_sor(output);
	struct tegra_sor_config config;
	struct tegra_sor_state *state;
	struct drm_display_mode *mode;
	struct drm_display_info *info;
	unsigned int i;
	u32 value;
	int err;

	state = to_sor_state(output->connector.state);
	mode = &encoder->crtc->state->adjusted_mode;
	info = &output->connector.display_info;

	err = clk_prepare_enable(sor->clk);
	if (err < 0)
		dev_err(sor->dev, "failed to enable clock: %d\n", err);

	usleep_range(1000, 2000);

	reset_control_deassert(sor->rst);

	/* switch to safe parent clock */
	err = tegra_sor_set_parent_clock(sor, sor->clk_safe);
	if (err < 0)
		dev_err(sor->dev, "failed to set safe parent clock: %d\n", err);

	err = tegra_io_rail_power_on(TEGRA_IO_RAIL_LVDS);
	if (err < 0)
		dev_err(sor->dev, "failed to power on LVDS rail: %d\n", err);

	usleep_range(20, 100);

	drm_dp_aux_enable(sor->aux);

	err = drm_dp_link_probe(sor->aux, &sor->link);
	if (err < 0) {
		dev_err(sor->dev, "failed to probe DP link: %d\n", err);
		goto out;
	}

	err = drm_dp_link_choose(&sor->link, mode, info);
	if (err < 0) {
		dev_err(sor->dev, "failed to choose link: %d\n", err);
		goto out;
	}

	value = tegra_sor_readl(sor, SOR_PLL2);
	value &= ~SOR_PLL2_BANDGAP_POWERDOWN;
	tegra_sor_writel(sor, value, SOR_PLL2);

	usleep_range(20, 40);

	value = tegra_sor_readl(sor, SOR_PLL3);
	value |= SOR_PLL3_PLL_VDD_MODE_3V3;
	tegra_sor_writel(sor, value, SOR_PLL3);

	value = tegra_sor_readl(sor, SOR_PLL0);
	value &= ~(SOR_PLL0_VCOPD | SOR_PLL0_PWR);
	tegra_sor_writel(sor, value, SOR_PLL0);

	value = tegra_sor_readl(sor, SOR_PLL2);
	value &= ~SOR_PLL2_SEQ_PLLCAPPD_ENFORCE;
	value |= SOR_PLL2_SEQ_PLLCAPPD;
	tegra_sor_writel(sor, value, SOR_PLL2);

	usleep_range(200, 400);

	value = tegra_sor_readl(sor, SOR_PLL2);
	value &= ~SOR_PLL2_POWERDOWN_OVERRIDE;
	value &= ~SOR_PLL2_PORT_POWERDOWN;
	tegra_sor_writel(sor, value, SOR_PLL2);

	value = tegra_sor_readl(sor, SOR_CLK_CNTRL);
	value &= ~SOR_CLK_CNTRL_DP_CLK_SEL_MASK;
	value |= SOR_CLK_CNTRL_DP_CLK_SEL_SINGLE_DPCLK;
	tegra_sor_writel(sor, value, SOR_CLK_CNTRL);

	value = tegra_sor_readl(sor, SOR_DP_SPARE0);
	/* XXX not in TRM */
	value &= ~SOR_DP_SPARE_PANEL_INTERNAL;
	value |= SOR_DP_SPARE_SEQ_ENABLE;
	tegra_sor_writel(sor, value, SOR_DP_SPARE0);

	/* XXX not in TRM */
	tegra_sor_writel(sor, 0, SOR_LVDS);

	value = tegra_sor_readl(sor, SOR_PLL0);
	value &= ~SOR_PLL0_ICHPMP_MASK;
	value &= ~SOR_PLL0_VCOCAP_MASK;
	value |= SOR_PLL0_ICHPMP(0x1);
	value |= SOR_PLL0_VCOCAP(0x3);
	value |= SOR_PLL0_RESISTOR_EXT;
	tegra_sor_writel(sor, value, SOR_PLL0);

	/* XXX not in TRM */
	for (value = 0, i = 0; i < 5; i++)
		value |= SOR_XBAR_CTRL_LINK0_XSEL(i, sor->soc->xbar_cfg[i]) |
			 SOR_XBAR_CTRL_LINK1_XSEL(i, i);

	tegra_sor_writel(sor, 0x00000000, SOR_XBAR_POL);
	tegra_sor_writel(sor, value, SOR_XBAR_CTRL);

	/* switch to DP parent clock */
	err = tegra_sor_set_parent_clock(sor, sor->clk_dp);
	if (err < 0) {
		dev_err(sor->dev, "failed to set parent clock: %d\n", err);
		goto out;
	}

	/* use DP-A protocol */
	value = tegra_sor_readl(sor, SOR_STATE1);
	value &= ~SOR_STATE_ASY_PROTOCOL_MASK;
	value |= SOR_STATE_ASY_PROTOCOL_DP_A;
	tegra_sor_writel(sor, value, SOR_STATE1);

	/* enable port */
	value = tegra_sor_readl(sor, SOR_DP_LINKCTL0);
	value |= SOR_DP_LINKCTL_ENABLE;
	tegra_sor_writel(sor, value, SOR_DP_LINKCTL0);

	tegra_sor_dp_term_calibrate(sor);

	err = drm_dp_link_train(&sor->link);
	if (err < 0) {
		dev_err(sor->dev, "link training failed: %d\n", err);
		goto out;
	}

	dev_dbg(sor->dev, "link training succeeded\n");

	err = drm_dp_link_power_up(sor->aux, &sor->link);
	if (err < 0) {
		dev_err(sor->dev, "failed to power up DP link: %d\n", err);
		goto out;
	}

	/* compute configuration */
	memset(&config, 0, sizeof(config));
	config.bits_per_pixel = state->bpc * 3;

	err = tegra_sor_compute_config(sor, mode, &config, &sor->link);
	if (err < 0) {
		dev_err(sor->dev, "failed to compute configuration: %d\n", err);
		goto out;
	}

	tegra_sor_apply_config(sor, &config);
	tegra_sor_mode_set(sor, mode, state);
	tegra_sor_update(sor);

	err = tegra_sor_power_up(sor, 250);
	if (err < 0)
		dev_err(sor->dev, "failed to power up SOR: %d\n", err);

	/* attach and wake up */
	err = tegra_sor_attach(sor);
	if (err < 0)
		dev_err(sor->dev, "failed to attach SOR: %d\n", err);

	value = tegra_dc_readl(dc, DC_DISP_DISP_WIN_OPTIONS);
	value |= SOR1_ENABLE;
	tegra_dc_writel(dc, value, DC_DISP_DISP_WIN_OPTIONS);

	tegra_dc_commit(dc);

	err = tegra_sor_wakeup(sor);
	if (err < 0)
		dev_err(sor->dev, "failed to wakeup SOR: %d\n", err);

out:
	drm_dp_aux_disable(sor->aux);
}

static const struct drm_encoder_helper_funcs tegra_sor_dp_helpers = {
	.disable = tegra_sor_dp_disable,
	.enable = tegra_sor_dp_enable,
	.atomic_check = tegra_sor_encoder_atomic_check,
};

static int tegra_sor_init(struct host1x_client *client)
{
	struct drm_device *drm = dev_get_drvdata(client->parent);
	const struct drm_encoder_helper_funcs *helpers = NULL;
	struct tegra_sor *sor = host1x_client_to_sor(client);
	int connector = DRM_MODE_CONNECTOR_Unknown;
	int encoder = DRM_MODE_ENCODER_NONE;
	int err;

	if (!sor->aux) {
		if (sor->soc->supports_hdmi) {
			connector = DRM_MODE_CONNECTOR_HDMIA;
			encoder = DRM_MODE_ENCODER_TMDS;
			helpers = &tegra_sor_hdmi_helpers;
		} else if (sor->soc->supports_lvds) {
			connector = DRM_MODE_CONNECTOR_LVDS;
			encoder = DRM_MODE_ENCODER_LVDS;
		}
	} else {
		if (sor->soc->supports_edp) {
			connector = DRM_MODE_CONNECTOR_eDP;
			encoder = DRM_MODE_ENCODER_TMDS;
			helpers = &tegra_sor_edp_helpers;
		} else if (sor->soc->supports_dp) {
			connector = DRM_MODE_CONNECTOR_DisplayPort;
			encoder = DRM_MODE_ENCODER_TMDS;
			helpers = &tegra_sor_dp_helpers;
		}

		drm_dp_link_train_init(&sor->link.train);
		sor->link.ops = &tegra_sor_dp_link_ops;
		sor->link.aux = sor->aux;
	}

	sor->output.dev = sor->dev;

	drm_connector_init(drm, &sor->output.connector,
			   &tegra_sor_connector_funcs,
			   connector);
	drm_connector_helper_add(&sor->output.connector,
				 &tegra_sor_connector_helper_funcs);
	sor->output.connector.dpms = DRM_MODE_DPMS_OFF;

	drm_encoder_init(drm, &sor->output.encoder, &tegra_sor_encoder_funcs,
			 encoder);
	drm_encoder_helper_add(&sor->output.encoder, helpers);

	drm_mode_connector_attach_encoder(&sor->output.connector,
					  &sor->output.encoder);
	drm_connector_register(&sor->output.connector);

	err = tegra_output_init(drm, &sor->output);
	if (err < 0) {
		dev_err(client->dev, "failed to initialize output: %d\n", err);
		return err;
	}

	sor->output.encoder.possible_crtcs = 0x3;

	if (IS_ENABLED(CONFIG_DEBUG_FS)) {
		err = tegra_sor_debugfs_init(sor, drm->primary);
		if (err < 0)
			dev_err(sor->dev, "debugfs setup failed: %d\n", err);
	}

	if (sor->aux) {
		err = drm_dp_aux_attach(sor->aux, &sor->output);
		if (err < 0) {
			dev_err(sor->dev, "failed to attach DP: %d\n", err);
			return err;
		}
	}

	/*
	 * XXX: Remove this reset once proper hand-over from firmware to
	 * kernel is possible.
	 */
	err = reset_control_assert(sor->rst);
	if (err < 0) {
		dev_err(sor->dev, "failed to assert SOR reset: %d\n", err);
		return err;
	}

	err = clk_prepare_enable(sor->clk);
	if (err < 0) {
		dev_err(sor->dev, "failed to enable clock: %d\n", err);
		return err;
	}

	usleep_range(1000, 3000);

	err = reset_control_deassert(sor->rst);
	if (err < 0) {
		dev_err(sor->dev, "failed to deassert SOR reset: %d\n", err);
		return err;
	}

	err = clk_prepare_enable(sor->clk_safe);
	if (err < 0)
		return err;

	err = clk_prepare_enable(sor->clk_dp);
	if (err < 0)
		return err;

	return 0;
}

static int tegra_sor_exit(struct host1x_client *client)
{
	struct tegra_sor *sor = host1x_client_to_sor(client);
	int err;

	tegra_output_exit(&sor->output);

	if (sor->aux) {
		err = drm_dp_aux_detach(sor->aux);
		if (err < 0) {
			dev_err(sor->dev, "failed to detach DP: %d\n", err);
			return err;
		}
	}

	clk_disable_unprepare(sor->clk_safe);
	clk_disable_unprepare(sor->clk_dp);
	clk_disable_unprepare(sor->clk);

	if (IS_ENABLED(CONFIG_DEBUG_FS))
		tegra_sor_debugfs_exit(sor);

	return 0;
}

static const struct host1x_client_ops sor_client_ops = {
	.init = tegra_sor_init,
	.exit = tegra_sor_exit,
};

static const struct tegra_sor_ops tegra_sor_edp_ops = {
	.name = "eDP",
};

static int tegra_sor_hdmi_probe(struct tegra_sor *sor)
{
	int err;

	sor->avdd_io_supply = devm_regulator_get(sor->dev, "avdd-io");
	if (IS_ERR(sor->avdd_io_supply)) {
		dev_err(sor->dev, "cannot get AVDD I/O supply: %ld\n",
			PTR_ERR(sor->avdd_io_supply));
		return PTR_ERR(sor->avdd_io_supply);
	}

	err = regulator_enable(sor->avdd_io_supply);
	if (err < 0) {
		dev_err(sor->dev, "failed to enable AVDD I/O supply: %d\n",
			err);
		return err;
	}

	sor->vdd_pll_supply = devm_regulator_get(sor->dev, "vdd-pll");
	if (IS_ERR(sor->vdd_pll_supply)) {
		dev_err(sor->dev, "cannot get VDD PLL supply: %ld\n",
			PTR_ERR(sor->vdd_pll_supply));
		return PTR_ERR(sor->vdd_pll_supply);
	}

	err = regulator_enable(sor->vdd_pll_supply);
	if (err < 0) {
		dev_err(sor->dev, "failed to enable VDD PLL supply: %d\n",
			err);
		return err;
	}

	sor->hdmi_supply = devm_regulator_get(sor->dev, "hdmi");
	if (IS_ERR(sor->hdmi_supply)) {
		dev_err(sor->dev, "cannot get HDMI supply: %ld\n",
			PTR_ERR(sor->hdmi_supply));
		return PTR_ERR(sor->hdmi_supply);
	}

	err = regulator_enable(sor->hdmi_supply);
	if (err < 0) {
		dev_err(sor->dev, "failed to enable HDMI supply: %d\n", err);
		return err;
	}

	return 0;
}

static int tegra_sor_hdmi_remove(struct tegra_sor *sor)
{
	regulator_disable(sor->hdmi_supply);
	regulator_disable(sor->vdd_pll_supply);
	regulator_disable(sor->avdd_io_supply);

	return 0;
}

static const struct tegra_sor_ops tegra_sor_hdmi_ops = {
	.name = "HDMI",
	.probe = tegra_sor_hdmi_probe,
	.remove = tegra_sor_hdmi_remove,
};

static int tegra_sor_dp_probe(struct tegra_sor *sor)
{
	int err;

	sor->avdd_io_supply = devm_regulator_get(sor->dev, "avdd-io-hdmi-dp");
	if (IS_ERR(sor->avdd_io_supply))
		return PTR_ERR(sor->avdd_io_supply);

	err = regulator_enable(sor->avdd_io_supply);
	if (err < 0)
		return err;

	sor->vdd_pll_supply = devm_regulator_get(sor->dev, "vdd-hdmi-dp-pll");
	if (IS_ERR(sor->vdd_pll_supply))
		return PTR_ERR(sor->vdd_pll_supply);

	err = regulator_enable(sor->vdd_pll_supply);
	if (err < 0)
		return err;

	return 0;
}

static int tegra_sor_dp_remove(struct tegra_sor *sor)
{
	return 0;
}

static const struct tegra_sor_ops tegra_sor_dp_ops = {
	.name = "DP",
	.probe = tegra_sor_dp_probe,
	.remove = tegra_sor_dp_remove,
};

static irqreturn_t tegra_sor_irq(int irq, void *data)
{
	struct tegra_sor *sor = data;

	dev_info(sor->dev, "> %s(irq=%d, data=%p)\n", __func__, irq, data);
	dev_info(sor->dev, "< %s()\n", __func__);

	return IRQ_HANDLED;
}

/* Tegra124 and Tegra132 have lanes 0 and 2 swapped. */
static const u8 tegra124_sor_lane_map[4] = {
	2, 1, 0, 3,
};

static const u8 tegra124_sor_xbar_cfg[5] = {
	0, 1, 2, 3, 4
};

static const u8 tegra124_sor_voltage_swing[4][4][4] = {
	{
		{ 0x13, 0x19, 0x1e, 0x28 },
		{ 0x1e, 0x25, 0x2d, },
		{ 0x28, 0x32, },
		{ 0x3c, },
	}, {
		{ 0x12, 0x17, 0x1b, 0x25 },
		{ 0x1c, 0x23, 0x2a, },
		{ 0x25, 0x2f, },
		{ 0x39, }
	}, {
		{ 0x12, 0x16, 0x1a, 0x22 },
		{ 0x1b, 0x20, 0x27, },
		{ 0x24, 0x2d, },
		{ 0x36, },
	}, {
		{ 0x11, 0x14, 0x17, 0x1f },
		{ 0x19, 0x1e, 0x24, },
		{ 0x22, 0x2a, },
		{ 0x32, },
	},
};

static const u8 tegra124_sor_pre_emphasis[4][4][4] = {
	{
		{ 0x00, 0x09, 0x13, 0x25 },
		{ 0x00, 0x0f, 0x1e, },
		{ 0x00, 0x14, },
		{ 0x00, },
	}, {
		{ 0x00, 0x0a, 0x14, 0x28 },
		{ 0x00, 0x0f, 0x1e, },
		{ 0x00, 0x14, },
		{ 0x00 },
	}, {
		{ 0x00, 0x0a, 0x14, 0x28 },
		{ 0x00, 0x0f, 0x1e, },
		{ 0x00, 0x14, },
		{ 0x00, },
	}, {
		{ 0x00, 0x0a, 0x14, 0x28 },
		{ 0x00, 0x0f, 0x1e, },
		{ 0x00, 0x14, },
		{ 0x00, },
	},
};

static const u8 tegra124_sor_post_cursor[4][4][4] = {
	{
		{ 0x00, 0x00, 0x00, 0x00 },
		{ 0x00, 0x00, 0x00, },
		{ 0x00, 0x00, },
		{ 0x00, },
	}, {
		{ 0x02, 0x02, 0x04, 0x05 },
		{ 0x02, 0x04, 0x05, },
		{ 0x04, 0x05, },
		{ 0x05, },
	}, {
		{ 0x04, 0x05, 0x08, 0x0b },
		{ 0x05, 0x09, 0x0b, },
		{ 0x08, 0x0a, },
		{ 0x0b, },
	}, {
		{ 0x05, 0x09, 0x0b, 0x12 },
		{ 0x09, 0x0d, 0x12, },
		{ 0x0b, 0x0f, },
		{ 0x12, },
	},
};

static const u8 tegra124_sor_tx_pu[4][4][4] = {
	{
		{ 0x20, 0x30, 0x40, 0x60 },
		{ 0x30, 0x40, 0x60, },
		{ 0x40, 0x60, },
		{ 0x60, },
	}, {
		{ 0x20, 0x20, 0x30, 0x50 },
		{ 0x30, 0x40, 0x50, },
		{ 0x40, 0x50, },
		{ 0x60, },
	}, {
		{ 0x20, 0x20, 0x30, 0x40, },
		{ 0x30, 0x30, 0x40, },
		{ 0x40, 0x50, },
		{ 0x60, },
	}, {
		{ 0x20, 0x20, 0x20, 0x40, },
		{ 0x30, 0x30, 0x40, },
		{ 0x40, 0x40, },
		{ 0x60, },
	},
};

static const struct tegra_sor_soc tegra124_sor = {
	.supports_edp = true,
	.supports_lvds = true,
	.supports_hdmi = false,
	.supports_dp = false,
	.xbar_cfg = tegra124_sor_xbar_cfg,
	.lane_map = tegra124_sor_lane_map,
	.voltage_swing = tegra124_sor_voltage_swing,
	.pre_emphasis = tegra124_sor_pre_emphasis,
	.post_cursor = tegra124_sor_post_cursor,
	.tx_pu = tegra124_sor_tx_pu,
};

static const u8 tegra132_sor_pre_emphasis[4][4][4] = {
	{
		{ 0x00, 0x08, 0x12, 0x24 },
		{ 0x01, 0x0e, 0x1d, },
		{ 0x01, 0x13, },
		{ 0x00, },
	}, {
		{ 0x00, 0x08, 0x12, 0x24 },
		{ 0x00, 0x0e, 0x1d, },
		{ 0x00, 0x13, },
		{ 0x00 },
	}, {
		{ 0x00, 0x08, 0x12, 0x24 },
		{ 0x00, 0x0e, 0x1d, },
		{ 0x00, 0x13, },
		{ 0x00, },
	}, {
		{ 0x00, 0x08, 0x12, 0x24 },
		{ 0x00, 0x0e, 0x1d, },
		{ 0x00, 0x13, },
		{ 0x00, },
	},
};

static const struct tegra_sor_soc tegra132_sor = {
	.supports_edp = true,
	.supports_lvds = true,
	.supports_hdmi = false,
	.supports_dp = false,
	.lane_map = tegra124_sor_lane_map,
	.xbar_cfg = tegra124_sor_xbar_cfg,
	.voltage_swing = tegra124_sor_voltage_swing,
	.pre_emphasis = tegra132_sor_pre_emphasis,
	.post_cursor = tegra124_sor_post_cursor,
	.tx_pu = tegra124_sor_tx_pu,
};

static const u8 tegra210_sor_lane_map[4] = {
	0, 1, 2, 3,
};

static const u8 tegra210_sor_xbar_cfg[5] = {
	2, 1, 0, 3, 4
};

static const struct tegra_sor_soc tegra210_sor = {
	.supports_edp = true,
	.supports_lvds = false,
	.supports_hdmi = false,
	.supports_dp = false,
	.xbar_cfg = tegra210_sor_xbar_cfg,
	.lane_map = tegra210_sor_lane_map,
	.voltage_swing = tegra124_sor_voltage_swing,
	.pre_emphasis = tegra124_sor_pre_emphasis,
	.post_cursor = tegra124_sor_post_cursor,
	.tx_pu = tegra124_sor_tx_pu,
};

static const struct tegra_sor_soc tegra210_sor1 = {
	.supports_edp = false,
	.supports_lvds = false,
	.supports_hdmi = true,
	.supports_dp = true,
	.num_settings = ARRAY_SIZE(tegra210_sor_hdmi_defaults),
	.settings = tegra210_sor_hdmi_defaults,
	.xbar_cfg = tegra210_sor_xbar_cfg,
	.lane_map = tegra210_sor_lane_map,
	.voltage_swing = tegra124_sor_voltage_swing,
	.pre_emphasis = tegra124_sor_pre_emphasis,
	.post_cursor = tegra124_sor_post_cursor,
	.tx_pu = tegra124_sor_tx_pu,
};

static const struct of_device_id tegra_sor_of_match[] = {
	{ .compatible = "nvidia,tegra210-sor1", .data = &tegra210_sor1 },
	{ .compatible = "nvidia,tegra210-sor", .data = &tegra210_sor },
	{ .compatible = "nvidia,tegra132-sor", .data = &tegra132_sor },
	{ .compatible = "nvidia,tegra124-sor", .data = &tegra124_sor },
	{ },
};
MODULE_DEVICE_TABLE(of, tegra_sor_of_match);

static int tegra_sor_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct device_node *np;
	struct tegra_sor *sor;
	struct resource *regs;
	int err;

	match = of_match_device(tegra_sor_of_match, &pdev->dev);

	sor = devm_kzalloc(&pdev->dev, sizeof(*sor), GFP_KERNEL);
	if (!sor)
		return -ENOMEM;

	sor->output.dev = sor->dev = &pdev->dev;
	sor->soc = match->data;

	sor->settings = devm_kmemdup(&pdev->dev, sor->soc->settings,
				     sor->soc->num_settings *
					sizeof(*sor->settings),
				     GFP_KERNEL);
	if (!sor->settings)
		return -ENOMEM;

	sor->num_settings = sor->soc->num_settings;

	np = of_parse_phandle(pdev->dev.of_node, "nvidia,dpaux", 0);
	if (np) {
		sor->aux = drm_dp_aux_find_by_of_node(np);
		of_node_put(np);

		if (!sor->aux)
			return -EPROBE_DEFER;

		sor->output.ddc = &sor->aux->ddc;
	}

	np = of_parse_phandle(pdev->dev.of_node, "nvidia,kfuse", 0);
	if (np) {
		sor->kfuse = tegra_kfuse_find_by_of_node(np);
		of_node_put(np);

		if (!sor->kfuse)
			return -EPROBE_DEFER;
	}

	if (!sor->aux) {
		if (sor->soc->supports_hdmi) {
			sor->ops = &tegra_sor_hdmi_ops;
		} else if (sor->soc->supports_lvds) {
			dev_err(&pdev->dev, "LVDS not supported yet\n");
			return -ENODEV;
		} else {
			dev_err(&pdev->dev, "unknown (non-DP) support\n");
			return -ENODEV;
		}
	} else {
		if (sor->soc->supports_edp) {
			sor->ops = &tegra_sor_edp_ops;
		} else if (sor->soc->supports_dp) {
			sor->ops = &tegra_sor_dp_ops;
		} else {
			dev_err(&pdev->dev, "unknown (DP) support\n");
			return -ENODEV;
		}
	}

	err = tegra_output_probe(&sor->output);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to probe output: %d\n", err);
		return err;
	}

	if (sor->ops && sor->ops->probe) {
		err = sor->ops->probe(sor);
		if (err < 0) {
			dev_err(&pdev->dev, "failed to probe %s: %d\n",
				sor->ops->name, err);
			goto output;
		}
	}

	if (sor->kfuse) {
		err = drm_hdcp_register(&sor->hdcp, sor->output.ddc);
		if (err < 0) {
			dev_err(&pdev->dev,
				"failed to register HDCP helper: %d\n",
				err);
			goto remove;
		}
	}

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	sor->regs = devm_ioremap_resource(&pdev->dev, regs);
	if (IS_ERR(sor->regs)) {
		err = PTR_ERR(sor->regs);
		goto hdcp;
	}

	err = platform_get_irq(pdev, 0);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to get IRQ: %d\n", err);
		return err;
	}

	sor->irq = err;

	sor->rst = devm_reset_control_get(&pdev->dev, "sor");
	if (IS_ERR(sor->rst)) {
		err = PTR_ERR(sor->rst);
		dev_err(&pdev->dev, "failed to get reset control: %d\n", err);
		goto hdcp;
	}

	sor->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(sor->clk)) {
		err = PTR_ERR(sor->clk);
		dev_err(&pdev->dev, "failed to get module clock: %d\n", err);
		goto hdcp;
	}

	sor->clk_parent = devm_clk_get(&pdev->dev, "parent");
	if (IS_ERR(sor->clk_parent)) {
		err = PTR_ERR(sor->clk_parent);
		dev_err(&pdev->dev, "failed to get parent clock: %d\n", err);
		goto hdcp;
	}

	sor->clk_safe = devm_clk_get(&pdev->dev, "safe");
	if (IS_ERR(sor->clk_safe)) {
		err = PTR_ERR(sor->clk_safe);
		dev_err(&pdev->dev, "failed to get safe clock: %d\n", err);
		goto hdcp;
	}

	sor->clk_dp = devm_clk_get(&pdev->dev, "dp");
	if (IS_ERR(sor->clk_dp)) {
		err = PTR_ERR(sor->clk_dp);
		dev_err(&pdev->dev, "failed to get DP clock: %d\n", err);
		goto hdcp;
	}

	err = devm_request_irq(&pdev->dev, sor->irq, tegra_sor_irq, 0,
			       dev_name(&pdev->dev), sor);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to request IRQ#%u: %d\n",
			sor->irq, err);
		return err;
	}

	INIT_LIST_HEAD(&sor->client.list);
	sor->client.ops = &sor_client_ops;
	sor->client.dev = &pdev->dev;

	err = host1x_client_register(&sor->client);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to register host1x client: %d\n",
			err);
		goto hdcp;
	}

	platform_set_drvdata(pdev, sor);

	return 0;

hdcp:
	if (sor->kfuse)
		drm_hdcp_unregister(&sor->hdcp);
remove:
	if (sor->ops && sor->ops->remove)
		sor->ops->remove(sor);
output:
	tegra_output_remove(&sor->output);
	return err;
}

static int tegra_sor_remove(struct platform_device *pdev)
{
	struct tegra_sor *sor = platform_get_drvdata(pdev);
	int err;

	err = host1x_client_unregister(&sor->client);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to unregister host1x client: %d\n",
			err);
		return err;
	}

	if (sor->kfuse)
		drm_hdcp_unregister(&sor->hdcp);

	if (sor->ops && sor->ops->remove) {
		err = sor->ops->remove(sor);
		if (err < 0)
			dev_err(&pdev->dev, "failed to remove SOR: %d\n", err);
	}

	tegra_output_remove(&sor->output);

	return 0;
}

struct platform_driver tegra_sor_driver = {
	.driver = {
		.name = "tegra-sor",
		.of_match_table = tegra_sor_of_match,
	},
	.probe = tegra_sor_probe,
	.remove = tegra_sor_remove,
};
