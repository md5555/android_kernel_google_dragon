/*
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>
#include <linux/slab.h>

#include <soc/tegra/fuse.h>
#include <soc/tegra/xusb.h>

#include <dt-bindings/pinctrl/pinctrl-tegra-xusb.h>

#include "core.h"
#include "pinctrl-tegra-xusb.h"
#include "pinctrl-utils.h"

#define FUSE_SKU_CALIB_HS_CURR_LEVEL_PADX_SHIFT(x) ((x) ? 15 : 0)
#define FUSE_SKU_CALIB_HS_CURR_LEVEL_PAD_MASK 0x3f
#define FUSE_SKU_CALIB_HS_IREF_CAP_SHIFT 13
#define FUSE_SKU_CALIB_HS_IREF_CAP_MASK 0x3
#define FUSE_SKU_CALIB_HS_SQUELCH_LEVEL_SHIFT 11
#define FUSE_SKU_CALIB_HS_SQUELCH_LEVEL_MASK 0x3
#define FUSE_SKU_CALIB_HS_TERM_RANGE_ADJ_SHIFT 7
#define FUSE_SKU_CALIB_HS_TERM_RANGE_ADJ_MASK 0xf

#define XUSB_PADCTL_USB2_PORT_CAP 0x008
#define XUSB_PADCTL_USB2_PORT_CAP_PORTX_CAP_SHIFT(x) ((x) * 4)
#define XUSB_PADCTL_USB2_PORT_CAP_PORT_CAP_MASK 0x3
#define XUSB_PADCTL_USB2_PORT_CAP_DISABLED 0x0
#define XUSB_PADCTL_USB2_PORT_CAP_HOST 0x1
#define XUSB_PADCTL_USB2_PORT_CAP_DEVICE 0x2
#define XUSB_PADCTL_USB2_PORT_CAP_OTG 0x3

#define XUSB_PADCTL_SS_PORT_MAP 0x014
#define XUSB_PADCTL_SS_PORT_MAP_PORTX_SHIFT(x) ((x) * 4)
#define XUSB_PADCTL_SS_PORT_MAP_PORT_MASK 0x7

#define XUSB_PADCTL_ELPG_PROGRAM 0x01c
#define XUSB_PADCTL_ELPG_PROGRAM_AUX_MUX_LP0_VCORE_DOWN (1 << 26)
#define XUSB_PADCTL_ELPG_PROGRAM_AUX_MUX_LP0_CLAMP_EN_EARLY (1 << 25)
#define XUSB_PADCTL_ELPG_PROGRAM_AUX_MUX_LP0_CLAMP_EN (1 << 24)
#define XUSB_PADCTL_ELPG_PROGRAM_SSPX_ELPG_VCORE_DOWN(x) (1 << (18 + (x) * 4))
#define XUSB_PADCTL_ELPG_PROGRAM_SSPX_ELPG_CLAMP_EN_EARLY(x) \
							(1 << (17 + (x) * 4))
#define XUSB_PADCTL_ELPG_PROGRAM_SSPX_ELPG_CLAMP_EN(x) (1 << (16 + (x) * 4))

#define XUSB_PADCTL_IOPHY_PLL_P0_CTL1 0x040
#define XUSB_PADCTL_IOPHY_PLL_P0_CTL1_PLL0_LOCKDET (1 << 19)
#define XUSB_PADCTL_IOPHY_PLL_P0_CTL1_REFCLK_SEL_MASK (0xf << 12)
#define XUSB_PADCTL_IOPHY_PLL_P0_CTL1_PLL_RST (1 << 1)

#define XUSB_PADCTL_IOPHY_PLL_P0_CTL2 0x044
#define XUSB_PADCTL_IOPHY_PLL_P0_CTL2_REFCLKBUF_EN (1 << 6)
#define XUSB_PADCTL_IOPHY_PLL_P0_CTL2_TXCLKREF_EN (1 << 5)
#define XUSB_PADCTL_IOPHY_PLL_P0_CTL2_TXCLKREF_SEL (1 << 4)

#define XUSB_PADCTL_IOPHY_USB3_PADX_CTL2(x) (0x058 + (x) * 4)
#define XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_CDR_CNTL_SHIFT 24
#define XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_CDR_CNTL_MASK 0xff
#define XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_CDR_CNTL_VAL 0x24
#define XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_RX_EQ_Z_SHIFT 16
#define XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_RX_EQ_Z_MASK 0x3f
#define XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_RX_EQ_G_SHIFT 8
#define XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_RX_EQ_G_MASK 0x3f
#define XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_RX_EQ_SHIFT 8
#define XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_RX_EQ_MASK 0xffff
#define XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_RX_EQ_VAL 0xf070
#define XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_RX_WANDER_SHIFT 4
#define XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_RX_WANDER_MASK 0xf
#define XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_RX_WANDER_VAL 0xf

#define XUSB_PADCTL_IOPHY_USB3_PADX_CTL4(x) (0x068 + (x) * 4)
#define XUSB_PADCTL_IOPHY_USB3_PAD_CTL4_DFE_CNTL_TAP_SHIFT 24
#define XUSB_PADCTL_IOPHY_USB3_PAD_CTL4_DFE_CNTL_TAP_MASK 0x1f
#define XUSB_PADCTL_IOPHY_USB3_PAD_CTL4_DFE_CNTL_AMP_SHIFT 16
#define XUSB_PADCTL_IOPHY_USB3_PAD_CTL4_DFE_CNTL_AMP_MASK 0x7f
#define XUSB_PADCTL_IOPHY_USB3_PAD_CTL4_DFE_CNTL_VAL 0x002008ee

#define XUSB_PADCTL_IOPHY_MISC_PAD_PX_CTL2(x) ((x) < 2 ? 0x078 + (x) * 4 : \
					       0x0f8 + (x) * 4)
#define XUSB_PADCTL_IOPHY_MISC_PAD_CTL2_SPARE_IN_SHIFT 28
#define XUSB_PADCTL_IOPHY_MISC_PAD_CTL2_SPARE_IN_MASK 0x3
#define XUSB_PADCTL_IOPHY_MISC_PAD_CTL2_SPARE_IN_VAL 0x1

#define XUSB_PADCTL_IOPHY_MISC_PAD_PX_CTL5(x) ((x) < 2 ? 0x090 + (x) * 4 : \
					       0x11c + (x) * 4)
#define XUSB_PADCTL_IOPHY_MISC_PAD_CTL5_RX_QEYE_EN (1 << 8)

#define XUSB_PADCTL_IOPHY_MISC_PAD_PX_CTL6(x) ((x) < 2 ? 0x098 + (x) * 4 : \
					       0x128 + (x) * 4)
#define XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SHIFT 24
#define XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_G_Z_MASK 0x3f
#define XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_TAP_MASK 0x1f
#define XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_AMP_MASK 0x7f
#define XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SEL_SHIFT 16
#define XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SEL_MASK 0xff
#define XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SEL_G_Z 0x21
#define XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SEL_TAP 0x32
#define XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SEL_AMP 0x33
#define XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SEL_CTLE_Z 0x48
#define XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SEL_LATCH_G_Z 0xa1

#define XUSB_PADCTL_USB2_OTG_PADX_CTL0(x) (0x0a0 + (x) * 4)
#define XUSB_PADCTL_USB2_OTG_PAD_CTL0_PD_ZI (1 << 21)
#define XUSB_PADCTL_USB2_OTG_PAD_CTL0_PD2 (1 << 20)
#define XUSB_PADCTL_USB2_OTG_PAD_CTL0_PD (1 << 19)
#define XUSB_PADCTL_USB2_OTG_PAD_CTL0_LS_RSLEW_SHIFT 14
#define XUSB_PADCTL_USB2_OTG_PAD_CTL0_LS_RSLEW_MASK 0x3
#define XUSB_PADCTL_USB2_OTG_PAD_CTL0_LS_RSLEW_VAL(x) ((x) ? 0x0 : 0x3)
#define XUSB_PADCTL_USB2_OTG_PAD_CTL0_HS_SLEW_SHIFT 6
#define XUSB_PADCTL_USB2_OTG_PAD_CTL0_HS_SLEW_MASK 0x3f
#define XUSB_PADCTL_USB2_OTG_PAD_CTL0_HS_SLEW_VAL 0x0e
#define XUSB_PADCTL_USB2_OTG_PAD_CTL0_HS_CURR_LEVEL_SHIFT 0
#define XUSB_PADCTL_USB2_OTG_PAD_CTL0_HS_CURR_LEVEL_MASK 0x3f

#define XUSB_PADCTL_USB2_OTG_PADX_CTL1(x) (0x0ac + (x) * 4)
#define XUSB_PADCTL_USB2_OTG_PAD_CTL1_HS_IREF_CAP_SHIFT 9
#define XUSB_PADCTL_USB2_OTG_PAD_CTL1_HS_IREF_CAP_MASK 0x3
#define XUSB_PADCTL_USB2_OTG_PAD_CTL1_TERM_RANGE_ADJ_SHIFT 3
#define XUSB_PADCTL_USB2_OTG_PAD_CTL1_TERM_RANGE_ADJ_MASK 0x7
#define XUSB_PADCTL_USB2_OTG_PAD_CTL1_PD_DR (1 << 2)
#define XUSB_PADCTL_USB2_OTG_PAD_CTL1_PD_DISC_FORCE_POWERUP (1 << 1)
#define XUSB_PADCTL_USB2_OTG_PAD_CTL1_PD_CHRP_FORCE_POWERUP (1 << 0)

#define XUSB_PADCTL_USB2_BIAS_PAD_CTL0 0x0b8
#define XUSB_PADCTL_USB2_BIAS_PAD_CTL0_PD (1 << 12)
#define XUSB_PADCTL_USB2_BIAS_PAD_CTL0_HS_DISCON_LEVEL_SHIFT 2
#define XUSB_PADCTL_USB2_BIAS_PAD_CTL0_HS_DISCON_LEVEL_MASK 0x7
#define XUSB_PADCTL_USB2_BIAS_PAD_CTL0_HS_DISCON_LEVEL_VAL 0x5
#define XUSB_PADCTL_USB2_BIAS_PAD_CTL0_HS_SQUELCH_LEVEL_SHIFT 0
#define XUSB_PADCTL_USB2_BIAS_PAD_CTL0_HS_SQUELCH_LEVEL_MASK 0x3

#define XUSB_PADCTL_HSIC_PADX_CTL0(x) (0x0c0 + (x) * 4)
#define XUSB_PADCTL_HSIC_PAD_CTL0_TX_RSLEWN_SHIFT 12
#define XUSB_PADCTL_HSIC_PAD_CTL0_TX_RSLEWN_MASK 0x7
#define XUSB_PADCTL_HSIC_PAD_CTL0_TX_RSLEWP_SHIFT 8
#define XUSB_PADCTL_HSIC_PAD_CTL0_TX_RSLEWP_MASK 0x7
#define XUSB_PADCTL_HSIC_PAD_CTL0_TX_RTUNEN_SHIFT 4
#define XUSB_PADCTL_HSIC_PAD_CTL0_TX_RTUNEN_MASK 0x7
#define XUSB_PADCTL_HSIC_PAD_CTL0_TX_RTUNEP_SHIFT 0
#define XUSB_PADCTL_HSIC_PAD_CTL0_TX_RTUNEP_MASK 0x7

#define XUSB_PADCTL_HSIC_PADX_CTL1(x) (0x0c8 + (x) * 4)
#define XUSB_PADCTL_HSIC_PAD_CTL1_RPU_STROBE (1 << 10)
#define XUSB_PADCTL_HSIC_PAD_CTL1_RPU_DATA (1 << 9)
#define XUSB_PADCTL_HSIC_PAD_CTL1_RPD_STROBE (1 << 8)
#define XUSB_PADCTL_HSIC_PAD_CTL1_RPD_DATA (1 << 7)
#define XUSB_PADCTL_HSIC_PAD_CTL1_PD_ZI (1 << 5)
#define XUSB_PADCTL_HSIC_PAD_CTL1_PD_RX (1 << 4)
#define XUSB_PADCTL_HSIC_PAD_CTL1_PD_TRX (1 << 3)
#define XUSB_PADCTL_HSIC_PAD_CTL1_PD_TX (1 << 2)
#define XUSB_PADCTL_HSIC_PAD_CTL1_AUTO_TERM_EN (1 << 0)

#define XUSB_PADCTL_HSIC_PADX_CTL2(x) (0x0d0 + (x) * 4)
#define XUSB_PADCTL_HSIC_PAD_CTL2_RX_STROBE_TRIM_SHIFT 4
#define XUSB_PADCTL_HSIC_PAD_CTL2_RX_STROBE_TRIM_MASK 0x7
#define XUSB_PADCTL_HSIC_PAD_CTL2_RX_DATA_TRIM_SHIFT 0
#define XUSB_PADCTL_HSIC_PAD_CTL2_RX_DATA_TRIM_MASK 0x7

#define XUSB_PADCTL_HSIC_STRB_TRIM_CONTROL 0x0e0
#define XUSB_PADCTL_HSIC_STRB_TRIM_CONTROL_STRB_TRIM_MASK 0x1f

#define XUSB_PADCTL_IOPHY_PLL_S0_CTL1 0x138
#define XUSB_PADCTL_IOPHY_PLL_S0_CTL1_PLL1_LOCKDET (1 << 27)
#define XUSB_PADCTL_IOPHY_PLL_S0_CTL1_PLL1_MODE (1 << 24)
#define XUSB_PADCTL_IOPHY_PLL_S0_CTL1_PLL0_REFCLK_NDIV_SHIFT 20
#define XUSB_PADCTL_IOPHY_PLL_S0_CTL1_PLL0_REFCLK_NDIV_MASK 0x3
#define XUSB_PADCTL_IOPHY_PLL_S0_CTL1_PLL_PWR_OVRD (1 << 3)
#define XUSB_PADCTL_IOPHY_PLL_S0_CTL1_PLL_RST (1 << 1)
#define XUSB_PADCTL_IOPHY_PLL_S0_CTL1_PLL_IDDQ (1 << 0)

#define XUSB_PADCTL_IOPHY_PLL_S0_CTL2 0x13c
#define XUSB_PADCTL_IOPHY_PLL_S0_CTL2_PLL1_CP_CNTL_SHIFT 20
#define XUSB_PADCTL_IOPHY_PLL_S0_CTL2_PLL1_CP_CNTL_MASK 0xf
#define XUSB_PADCTL_IOPHY_PLL_S0_CTL2_PLL0_CP_CNTL_SHIFT 16
#define XUSB_PADCTL_IOPHY_PLL_S0_CTL2_PLL0_CP_CNTL_MASK 0xf
#define XUSB_PADCTL_IOPHY_PLL_S0_CTL2_TCLKOUT_EN (1 << 12)
#define XUSB_PADCTL_IOPHY_PLL_S0_CTL2_TXCLKREF_SEL (1 << 4)
#define XUSB_PADCTL_IOPHY_PLL_S0_CTL2_XDIGCLK_SEL_SHIFT 0
#define XUSB_PADCTL_IOPHY_PLL_S0_CTL2_XDIGCLK_SEL_MASK 0x7

#define XUSB_PADCTL_IOPHY_PLL_S0_CTL3 0x140
#define XUSB_PADCTL_IOPHY_PLL_S0_CTL3_RCAL_BYPASS (1 << 7)

#define XUSB_PADCTL_IOPHY_MISC_PAD_S0_CTL1 0x148
#define XUSB_PADCTL_IOPHY_MISC_PAD_S0_CTL1_IDDQ_OVRD (1 << 1)
#define XUSB_PADCTL_IOPHY_MISC_PAD_S0_CTL1_IDDQ (1 << 0)

#define XUSB_PADCTL_IOPHY_MISC_PAD_S0_CTL2 0x14c

#define XUSB_PADCTL_IOPHY_MISC_PAD_S0_CTL5 0x158

#define XUSB_PADCTL_IOPHY_MISC_PAD_S0_CTL6 0x15c

struct tegra124_xusb_fuse_calibration {
	u32 hs_curr_level[3];
	u32 hs_iref_cap;
	u32 hs_term_range_adj;
	u32 hs_squelch_level;
};

struct tegra124_xusb_padctl {
	struct tegra124_xusb_fuse_calibration fuse;
};

static int tegra124_xusb_padctl_enable(struct tegra_xusb_padctl *padctl)
{
	u32 value;

	mutex_lock(&padctl->lock);

	if (padctl->enable++ > 0)
		goto out;

	value = padctl_readl(padctl, XUSB_PADCTL_ELPG_PROGRAM);
	value &= ~XUSB_PADCTL_ELPG_PROGRAM_AUX_MUX_LP0_CLAMP_EN;
	padctl_writel(padctl, value, XUSB_PADCTL_ELPG_PROGRAM);

	usleep_range(100, 200);

	value = padctl_readl(padctl, XUSB_PADCTL_ELPG_PROGRAM);
	value &= ~XUSB_PADCTL_ELPG_PROGRAM_AUX_MUX_LP0_CLAMP_EN_EARLY;
	padctl_writel(padctl, value, XUSB_PADCTL_ELPG_PROGRAM);

	usleep_range(100, 200);

	value = padctl_readl(padctl, XUSB_PADCTL_ELPG_PROGRAM);
	value &= ~XUSB_PADCTL_ELPG_PROGRAM_AUX_MUX_LP0_VCORE_DOWN;
	padctl_writel(padctl, value, XUSB_PADCTL_ELPG_PROGRAM);

out:
	mutex_unlock(&padctl->lock);
	return 0;
}

static int tegra124_xusb_padctl_disable(struct tegra_xusb_padctl *padctl)
{
	u32 value;

	mutex_lock(&padctl->lock);

	if (WARN_ON(padctl->enable == 0))
		goto out;

	if (--padctl->enable > 0)
		goto out;

	value = padctl_readl(padctl, XUSB_PADCTL_ELPG_PROGRAM);
	value |= XUSB_PADCTL_ELPG_PROGRAM_AUX_MUX_LP0_VCORE_DOWN;
	padctl_writel(padctl, value, XUSB_PADCTL_ELPG_PROGRAM);

	usleep_range(100, 200);

	value = padctl_readl(padctl, XUSB_PADCTL_ELPG_PROGRAM);
	value |= XUSB_PADCTL_ELPG_PROGRAM_AUX_MUX_LP0_CLAMP_EN_EARLY;
	padctl_writel(padctl, value, XUSB_PADCTL_ELPG_PROGRAM);

	usleep_range(100, 200);

	value = padctl_readl(padctl, XUSB_PADCTL_ELPG_PROGRAM);
	value |= XUSB_PADCTL_ELPG_PROGRAM_AUX_MUX_LP0_CLAMP_EN;
	padctl_writel(padctl, value, XUSB_PADCTL_ELPG_PROGRAM);

out:
	mutex_unlock(&padctl->lock);
	return 0;
}

static int tegra124_xusb_phy_init(struct phy *phy)
{
	struct tegra_xusb_padctl *padctl = phy_get_drvdata(phy);

	return tegra124_xusb_padctl_enable(padctl);
}

static int tegra124_xusb_phy_exit(struct phy *phy)
{
	struct tegra_xusb_padctl *padctl = phy_get_drvdata(phy);

	return tegra124_xusb_padctl_disable(padctl);
}

static int tegra124_pcie_phy_power_on(struct phy *phy)
{
	struct tegra_xusb_padctl *padctl = phy_get_drvdata(phy);
	unsigned long timeout;
	int err = -ETIMEDOUT;
	u32 value;

	value = padctl_readl(padctl, XUSB_PADCTL_IOPHY_PLL_P0_CTL1);
	value &= ~XUSB_PADCTL_IOPHY_PLL_P0_CTL1_REFCLK_SEL_MASK;
	padctl_writel(padctl, value, XUSB_PADCTL_IOPHY_PLL_P0_CTL1);

	value = padctl_readl(padctl, XUSB_PADCTL_IOPHY_PLL_P0_CTL2);
	value |= XUSB_PADCTL_IOPHY_PLL_P0_CTL2_REFCLKBUF_EN |
		 XUSB_PADCTL_IOPHY_PLL_P0_CTL2_TXCLKREF_EN |
		 XUSB_PADCTL_IOPHY_PLL_P0_CTL2_TXCLKREF_SEL;
	padctl_writel(padctl, value, XUSB_PADCTL_IOPHY_PLL_P0_CTL2);

	value = padctl_readl(padctl, XUSB_PADCTL_IOPHY_PLL_P0_CTL1);
	value |= XUSB_PADCTL_IOPHY_PLL_P0_CTL1_PLL_RST;
	padctl_writel(padctl, value, XUSB_PADCTL_IOPHY_PLL_P0_CTL1);

	timeout = jiffies + msecs_to_jiffies(50);

	while (time_before(jiffies, timeout)) {
		value = padctl_readl(padctl, XUSB_PADCTL_IOPHY_PLL_P0_CTL1);
		if (value & XUSB_PADCTL_IOPHY_PLL_P0_CTL1_PLL0_LOCKDET) {
			err = 0;
			break;
		}

		usleep_range(100, 200);
	}

	return err;
}

static int tegra124_pcie_phy_power_off(struct phy *phy)
{
	struct tegra_xusb_padctl *padctl = phy_get_drvdata(phy);
	u32 value;

	value = padctl_readl(padctl, XUSB_PADCTL_IOPHY_PLL_P0_CTL1);
	value &= ~XUSB_PADCTL_IOPHY_PLL_P0_CTL1_PLL_RST;
	padctl_writel(padctl, value, XUSB_PADCTL_IOPHY_PLL_P0_CTL1);

	return 0;
}

static const struct phy_ops tegra124_pcie_phy_ops = {
	.init = tegra124_xusb_phy_init,
	.exit = tegra124_xusb_phy_exit,
	.power_on = tegra124_pcie_phy_power_on,
	.power_off = tegra124_pcie_phy_power_off,
	.owner = THIS_MODULE,
};

static int tegra124_sata_phy_power_on(struct phy *phy)
{
	struct tegra_xusb_padctl *padctl = phy_get_drvdata(phy);
	unsigned long timeout;
	int err = -ETIMEDOUT;
	u32 value;

	value = padctl_readl(padctl, XUSB_PADCTL_IOPHY_MISC_PAD_S0_CTL1);
	value &= ~XUSB_PADCTL_IOPHY_MISC_PAD_S0_CTL1_IDDQ_OVRD;
	value &= ~XUSB_PADCTL_IOPHY_MISC_PAD_S0_CTL1_IDDQ;
	padctl_writel(padctl, value, XUSB_PADCTL_IOPHY_MISC_PAD_S0_CTL1);

	value = padctl_readl(padctl, XUSB_PADCTL_IOPHY_PLL_S0_CTL1);
	value &= ~XUSB_PADCTL_IOPHY_PLL_S0_CTL1_PLL_PWR_OVRD;
	value &= ~XUSB_PADCTL_IOPHY_PLL_S0_CTL1_PLL_IDDQ;
	padctl_writel(padctl, value, XUSB_PADCTL_IOPHY_PLL_S0_CTL1);

	value = padctl_readl(padctl, XUSB_PADCTL_IOPHY_PLL_S0_CTL1);
	value |= XUSB_PADCTL_IOPHY_PLL_S0_CTL1_PLL1_MODE;
	padctl_writel(padctl, value, XUSB_PADCTL_IOPHY_PLL_S0_CTL1);

	value = padctl_readl(padctl, XUSB_PADCTL_IOPHY_PLL_S0_CTL1);
	value |= XUSB_PADCTL_IOPHY_PLL_S0_CTL1_PLL_RST;
	padctl_writel(padctl, value, XUSB_PADCTL_IOPHY_PLL_S0_CTL1);

	timeout = jiffies + msecs_to_jiffies(50);

	while (time_before(jiffies, timeout)) {
		value = padctl_readl(padctl, XUSB_PADCTL_IOPHY_PLL_S0_CTL1);
		if (value & XUSB_PADCTL_IOPHY_PLL_S0_CTL1_PLL1_LOCKDET) {
			err = 0;
			break;
		}

		usleep_range(100, 200);
	}

	return err;
}

static int tegra124_sata_phy_power_off(struct phy *phy)
{
	struct tegra_xusb_padctl *padctl = phy_get_drvdata(phy);
	u32 value;

	value = padctl_readl(padctl, XUSB_PADCTL_IOPHY_PLL_S0_CTL1);
	value &= ~XUSB_PADCTL_IOPHY_PLL_S0_CTL1_PLL_RST;
	padctl_writel(padctl, value, XUSB_PADCTL_IOPHY_PLL_S0_CTL1);

	value = padctl_readl(padctl, XUSB_PADCTL_IOPHY_PLL_S0_CTL1);
	value &= ~XUSB_PADCTL_IOPHY_PLL_S0_CTL1_PLL1_MODE;
	padctl_writel(padctl, value, XUSB_PADCTL_IOPHY_PLL_S0_CTL1);

	value = padctl_readl(padctl, XUSB_PADCTL_IOPHY_PLL_S0_CTL1);
	value |= XUSB_PADCTL_IOPHY_PLL_S0_CTL1_PLL_PWR_OVRD;
	value |= XUSB_PADCTL_IOPHY_PLL_S0_CTL1_PLL_IDDQ;
	padctl_writel(padctl, value, XUSB_PADCTL_IOPHY_PLL_S0_CTL1);

	value = padctl_readl(padctl, XUSB_PADCTL_IOPHY_MISC_PAD_S0_CTL1);
	value |= ~XUSB_PADCTL_IOPHY_MISC_PAD_S0_CTL1_IDDQ_OVRD;
	value |= ~XUSB_PADCTL_IOPHY_MISC_PAD_S0_CTL1_IDDQ;
	padctl_writel(padctl, value, XUSB_PADCTL_IOPHY_MISC_PAD_S0_CTL1);

	return 0;
}

static const struct phy_ops tegra124_sata_phy_ops = {
	.init = tegra124_xusb_phy_init,
	.exit = tegra124_xusb_phy_exit,
	.power_on = tegra124_sata_phy_power_on,
	.power_off = tegra124_sata_phy_power_off,
	.owner = THIS_MODULE,
};

static int tegra124_usb3_phy_init(struct phy *phy)
{
	struct tegra_xusb_usb3_phy *usb = phy_get_drvdata(phy);
	struct tegra_xusb_padctl *padctl = usb->padctl;
	u32 value;
	int err;

	err = tegra124_xusb_padctl_enable(padctl);
	if (err < 0)
		return err;

	value = padctl_readl(padctl, XUSB_PADCTL_SS_PORT_MAP);
	value &= ~(XUSB_PADCTL_SS_PORT_MAP_PORT_MASK <<
		   XUSB_PADCTL_SS_PORT_MAP_PORTX_SHIFT(usb->index));
	value |= usb->port <<
		 XUSB_PADCTL_SS_PORT_MAP_PORTX_SHIFT(usb->index);
	padctl_writel(padctl, value, XUSB_PADCTL_SS_PORT_MAP);

	return 0;
}

static int tegra124_usb3_phy_exit(struct phy *phy)
{
	struct tegra_xusb_usb3_phy *usb = phy_get_drvdata(phy);
	struct tegra_xusb_padctl *padctl = usb->padctl;
	u32 value;

	value = padctl_readl(padctl, XUSB_PADCTL_SS_PORT_MAP);
	value |= 0x7 << XUSB_PADCTL_SS_PORT_MAP_PORTX_SHIFT(usb->index);
	padctl_writel(padctl, value, XUSB_PADCTL_SS_PORT_MAP);

	return tegra124_xusb_padctl_disable(padctl);
}

static int tegra124_usb3_phy_power_on(struct phy *phy)
{
	struct tegra_xusb_usb3_phy *usb = phy_get_drvdata(phy);
	struct tegra_xusb_padctl *padctl = usb->padctl;
	unsigned int port = usb->index;
	u32 value, offset;

	value = padctl_readl(padctl, XUSB_PADCTL_IOPHY_USB3_PADX_CTL2(port));
	value &= ~((XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_RX_WANDER_MASK <<
		    XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_RX_WANDER_SHIFT) |
		   (XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_RX_EQ_MASK <<
		    XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_RX_EQ_SHIFT) |
		   (XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_CDR_CNTL_MASK <<
		    XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_CDR_CNTL_SHIFT));
	value |= (XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_RX_WANDER_VAL <<
		  XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_RX_WANDER_SHIFT) |
		 (XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_CDR_CNTL_VAL <<
		  XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_CDR_CNTL_SHIFT) |
		 (XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_RX_EQ_VAL <<
		  XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_RX_EQ_SHIFT);
	if (usb->context_saved) {
		value &= ~((XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_RX_EQ_G_MASK <<
			    XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_RX_EQ_G_SHIFT) |
			   (XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_RX_EQ_Z_MASK <<
			    XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_RX_EQ_Z_SHIFT));
		value |= (usb->ctle_g <<
			  XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_RX_EQ_G_SHIFT) |
			 (usb->ctle_z <<
			  XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_RX_EQ_Z_SHIFT);
	}
	padctl_writel(padctl, value, XUSB_PADCTL_IOPHY_USB3_PADX_CTL2(port));

	value = XUSB_PADCTL_IOPHY_USB3_PAD_CTL4_DFE_CNTL_VAL;
	if (usb->context_saved) {
		value &= ~((XUSB_PADCTL_IOPHY_USB3_PAD_CTL4_DFE_CNTL_TAP_MASK <<
			    XUSB_PADCTL_IOPHY_USB3_PAD_CTL4_DFE_CNTL_TAP_SHIFT) |
			   (XUSB_PADCTL_IOPHY_USB3_PAD_CTL4_DFE_CNTL_AMP_MASK <<
			    XUSB_PADCTL_IOPHY_USB3_PAD_CTL4_DFE_CNTL_AMP_SHIFT));
		value |= (usb->tap1 <<
			  XUSB_PADCTL_IOPHY_USB3_PAD_CTL4_DFE_CNTL_TAP_SHIFT) |
			 (usb->amp <<
			  XUSB_PADCTL_IOPHY_USB3_PAD_CTL4_DFE_CNTL_AMP_SHIFT);
	}
	padctl_writel(padctl, value, XUSB_PADCTL_IOPHY_USB3_PADX_CTL4(port));

	offset = (usb->lane == PIN_SATA_0) ?
		XUSB_PADCTL_IOPHY_MISC_PAD_S0_CTL2 :
		XUSB_PADCTL_IOPHY_MISC_PAD_PX_CTL2(usb->lane - PIN_PCIE_0);
	value = padctl_readl(padctl, offset);
	value &= ~(XUSB_PADCTL_IOPHY_MISC_PAD_CTL2_SPARE_IN_MASK <<
		   XUSB_PADCTL_IOPHY_MISC_PAD_CTL2_SPARE_IN_SHIFT);
	value |= XUSB_PADCTL_IOPHY_MISC_PAD_CTL2_SPARE_IN_VAL <<
		XUSB_PADCTL_IOPHY_MISC_PAD_CTL2_SPARE_IN_SHIFT;
	padctl_writel(padctl, value, offset);

	offset = (usb->lane == PIN_SATA_0) ?
		XUSB_PADCTL_IOPHY_MISC_PAD_S0_CTL5 :
		XUSB_PADCTL_IOPHY_MISC_PAD_PX_CTL5(usb->lane - PIN_PCIE_0);
	value = padctl_readl(padctl, offset);
	value |= XUSB_PADCTL_IOPHY_MISC_PAD_CTL5_RX_QEYE_EN;
	padctl_writel(padctl, value, offset);

	/* Enable SATA PHY when SATA lane is used */
	if (usb->lane == PIN_SATA_0) {
		value = padctl_readl(padctl, XUSB_PADCTL_IOPHY_PLL_S0_CTL1);
		value &= ~(XUSB_PADCTL_IOPHY_PLL_S0_CTL1_PLL0_REFCLK_NDIV_MASK <<
			   XUSB_PADCTL_IOPHY_PLL_S0_CTL1_PLL0_REFCLK_NDIV_SHIFT);
		value |= 0x2 <<
			XUSB_PADCTL_IOPHY_PLL_S0_CTL1_PLL0_REFCLK_NDIV_SHIFT;
		padctl_writel(padctl, value, XUSB_PADCTL_IOPHY_PLL_S0_CTL1);

		value = padctl_readl(padctl, XUSB_PADCTL_IOPHY_PLL_S0_CTL2);
		value &= ~((XUSB_PADCTL_IOPHY_PLL_S0_CTL2_XDIGCLK_SEL_MASK <<
			    XUSB_PADCTL_IOPHY_PLL_S0_CTL2_XDIGCLK_SEL_SHIFT) |
			   (XUSB_PADCTL_IOPHY_PLL_S0_CTL2_PLL1_CP_CNTL_MASK <<
			    XUSB_PADCTL_IOPHY_PLL_S0_CTL2_PLL1_CP_CNTL_SHIFT) |
			   (XUSB_PADCTL_IOPHY_PLL_S0_CTL2_PLL0_CP_CNTL_MASK <<
			    XUSB_PADCTL_IOPHY_PLL_S0_CTL2_PLL0_CP_CNTL_SHIFT) |
			   XUSB_PADCTL_IOPHY_PLL_S0_CTL2_TCLKOUT_EN);
		value |= (0x7 <<
			  XUSB_PADCTL_IOPHY_PLL_S0_CTL2_XDIGCLK_SEL_SHIFT) |
			 (0x8 <<
			  XUSB_PADCTL_IOPHY_PLL_S0_CTL2_PLL1_CP_CNTL_SHIFT) |
			 (0x8 <<
			  XUSB_PADCTL_IOPHY_PLL_S0_CTL2_PLL0_CP_CNTL_SHIFT) |
			 XUSB_PADCTL_IOPHY_PLL_S0_CTL2_TXCLKREF_SEL;
		padctl_writel(padctl, value, XUSB_PADCTL_IOPHY_PLL_S0_CTL2);

		value = padctl_readl(padctl, XUSB_PADCTL_IOPHY_PLL_S0_CTL3);
		value &= ~XUSB_PADCTL_IOPHY_PLL_S0_CTL3_RCAL_BYPASS;
		padctl_writel(padctl, value, XUSB_PADCTL_IOPHY_PLL_S0_CTL3);
	}

	value = padctl_readl(padctl, XUSB_PADCTL_ELPG_PROGRAM);
	value &= ~XUSB_PADCTL_ELPG_PROGRAM_SSPX_ELPG_VCORE_DOWN(port);
	padctl_writel(padctl, value, XUSB_PADCTL_ELPG_PROGRAM);

	usleep_range(100, 200);

	value = padctl_readl(padctl, XUSB_PADCTL_ELPG_PROGRAM);
	value &= ~XUSB_PADCTL_ELPG_PROGRAM_SSPX_ELPG_CLAMP_EN_EARLY(port);
	padctl_writel(padctl, value, XUSB_PADCTL_ELPG_PROGRAM);

	usleep_range(100, 200);

	value = padctl_readl(padctl, XUSB_PADCTL_ELPG_PROGRAM);
	value &= ~XUSB_PADCTL_ELPG_PROGRAM_SSPX_ELPG_CLAMP_EN(port);
	padctl_writel(padctl, value, XUSB_PADCTL_ELPG_PROGRAM);

	return 0;
}

static int tegra124_usb3_phy_power_off(struct phy *phy)
{
	struct tegra_xusb_usb3_phy *usb = phy_get_drvdata(phy);
	struct tegra_xusb_padctl *padctl = usb->padctl;
	unsigned int port = usb->index;
	u32 value;

	value = padctl_readl(padctl, XUSB_PADCTL_ELPG_PROGRAM);
	value |= XUSB_PADCTL_ELPG_PROGRAM_SSPX_ELPG_CLAMP_EN_EARLY(port);
	padctl_writel(padctl, value, XUSB_PADCTL_ELPG_PROGRAM);

	usleep_range(100, 200);

	value = padctl_readl(padctl, XUSB_PADCTL_ELPG_PROGRAM);
	value |= XUSB_PADCTL_ELPG_PROGRAM_SSPX_ELPG_CLAMP_EN(port);
	padctl_writel(padctl, value, XUSB_PADCTL_ELPG_PROGRAM);

	usleep_range(250, 350);

	value = padctl_readl(padctl, XUSB_PADCTL_ELPG_PROGRAM);
	value |= XUSB_PADCTL_ELPG_PROGRAM_SSPX_ELPG_VCORE_DOWN(port);
	padctl_writel(padctl, value, XUSB_PADCTL_ELPG_PROGRAM);

	return 0;
}

static int tegra124_usb3_phy_save_context(struct tegra_xusb_padctl *padctl,
					  unsigned int port)
{
	struct tegra_xusb_usb3_phy *usb;
	u32 value, offset;

	if (port >= padctl->soc->num_usb3_phys)
		return -EINVAL;

	usb = &padctl->usb3_phys[port];
	usb->context_saved = true;

	offset = (usb->lane == PIN_SATA_0) ?
		XUSB_PADCTL_IOPHY_MISC_PAD_S0_CTL6 :
		XUSB_PADCTL_IOPHY_MISC_PAD_PX_CTL6(usb->lane - PIN_PCIE_0);

	value = padctl_readl(padctl, offset);
	value &= ~(XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SEL_MASK <<
		   XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SEL_SHIFT);
	value |= XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SEL_TAP <<
		XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SEL_SHIFT;
	padctl_writel(padctl, value, offset);

	value = padctl_readl(padctl, offset) >>
		XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SHIFT;
	usb->tap1 = value & XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_TAP_MASK;

	value = padctl_readl(padctl, offset);
	value &= ~(XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SEL_MASK <<
		   XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SEL_SHIFT);
	value |= XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SEL_AMP <<
		XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SEL_SHIFT;
	padctl_writel(padctl, value, offset);

	value = padctl_readl(padctl, offset) >>
		XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SHIFT;
	usb->amp = value & XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_AMP_MASK;

	value = padctl_readl(padctl, XUSB_PADCTL_IOPHY_USB3_PADX_CTL4(port));
	value &= ~((XUSB_PADCTL_IOPHY_USB3_PAD_CTL4_DFE_CNTL_TAP_MASK <<
		    XUSB_PADCTL_IOPHY_USB3_PAD_CTL4_DFE_CNTL_TAP_SHIFT) |
		   (XUSB_PADCTL_IOPHY_USB3_PAD_CTL4_DFE_CNTL_AMP_MASK <<
		    XUSB_PADCTL_IOPHY_USB3_PAD_CTL4_DFE_CNTL_AMP_SHIFT));
	value |= (usb->tap1 <<
		  XUSB_PADCTL_IOPHY_USB3_PAD_CTL4_DFE_CNTL_TAP_SHIFT) |
		 (usb->amp <<
		  XUSB_PADCTL_IOPHY_USB3_PAD_CTL4_DFE_CNTL_AMP_SHIFT);
	padctl_writel(padctl, value, XUSB_PADCTL_IOPHY_USB3_PADX_CTL4(port));

	value = padctl_readl(padctl, offset);
	value &= ~(XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SEL_MASK <<
		   XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SEL_SHIFT);
	value |= XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SEL_LATCH_G_Z <<
		XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SEL_SHIFT;
	padctl_writel(padctl, value, offset);

	value = padctl_readl(padctl, offset);
	value &= ~(XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SEL_MASK <<
		   XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SEL_SHIFT);
	value |= XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SEL_G_Z <<
		XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SEL_SHIFT;
	padctl_writel(padctl, value, offset);

	value = padctl_readl(padctl, offset) >>
		XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SHIFT;
	usb->ctle_g = value &
		XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_G_Z_MASK;

	value = padctl_readl(padctl, offset);
	value &= ~(XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SEL_MASK <<
		   XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SEL_SHIFT);
	value |= XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SEL_CTLE_Z <<
		XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SEL_SHIFT;
	padctl_writel(padctl, value, offset);

	value = padctl_readl(padctl, offset) >>
		XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_SHIFT;
	usb->ctle_z = value &
		XUSB_PADCTL_IOPHY_MISC_PAD_CTL6_MISC_OUT_G_Z_MASK;

	value = padctl_readl(padctl, XUSB_PADCTL_IOPHY_USB3_PADX_CTL2(port));
	value &= ~((XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_RX_EQ_G_MASK <<
		    XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_RX_EQ_G_SHIFT) |
		   (XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_RX_EQ_Z_MASK <<
		    XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_RX_EQ_Z_SHIFT));
	value |= (usb->ctle_g <<
		  XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_RX_EQ_G_SHIFT) |
		 (usb->ctle_z <<
		  XUSB_PADCTL_IOPHY_USB3_PAD_CTL2_RX_EQ_Z_SHIFT);
	padctl_writel(padctl, value, XUSB_PADCTL_IOPHY_USB3_PADX_CTL2(port));

	return 0;
}

static const struct phy_ops tegra124_usb3_phy_ops = {
	.init = tegra124_usb3_phy_init,
	.exit = tegra124_usb3_phy_exit,
	.power_on = tegra124_usb3_phy_power_on,
	.power_off = tegra124_usb3_phy_power_off,
	.owner = THIS_MODULE,
};

static int tegra124_utmi_phy_init(struct phy *phy)
{
	struct tegra_xusb_utmi_phy *utmi = phy_get_drvdata(phy);

	return tegra124_xusb_padctl_enable(utmi->padctl);
}

static int tegra124_utmi_phy_exit(struct phy *phy)
{
	struct tegra_xusb_utmi_phy *utmi = phy_get_drvdata(phy);

	return tegra124_xusb_padctl_disable(utmi->padctl);
}

static int tegra124_utmi_phy_power_on(struct phy *phy)
{
	struct tegra_xusb_utmi_phy *utmi = phy_get_drvdata(phy);
	struct tegra_xusb_padctl *padctl = utmi->padctl;
	struct tegra124_xusb_padctl *priv = padctl->priv;
	unsigned int port = utmi->index;
	u32 value;
	int err;

	value = padctl_readl(padctl, XUSB_PADCTL_USB2_BIAS_PAD_CTL0);
	value &= ~((XUSB_PADCTL_USB2_BIAS_PAD_CTL0_HS_SQUELCH_LEVEL_MASK <<
		    XUSB_PADCTL_USB2_BIAS_PAD_CTL0_HS_SQUELCH_LEVEL_SHIFT) |
		   (XUSB_PADCTL_USB2_BIAS_PAD_CTL0_HS_DISCON_LEVEL_MASK <<
		    XUSB_PADCTL_USB2_BIAS_PAD_CTL0_HS_DISCON_LEVEL_SHIFT));
	value |= (priv->fuse.hs_squelch_level <<
		  XUSB_PADCTL_USB2_BIAS_PAD_CTL0_HS_SQUELCH_LEVEL_SHIFT) |
		 (XUSB_PADCTL_USB2_BIAS_PAD_CTL0_HS_DISCON_LEVEL_VAL <<
		  XUSB_PADCTL_USB2_BIAS_PAD_CTL0_HS_DISCON_LEVEL_SHIFT);
	padctl_writel(padctl, value, XUSB_PADCTL_USB2_BIAS_PAD_CTL0);

	value = padctl_readl(padctl, XUSB_PADCTL_USB2_PORT_CAP);
	value &= ~(XUSB_PADCTL_USB2_PORT_CAP_PORT_CAP_MASK <<
		   XUSB_PADCTL_USB2_PORT_CAP_PORTX_CAP_SHIFT(port));
	value |= XUSB_PADCTL_USB2_PORT_CAP_HOST <<
		XUSB_PADCTL_USB2_PORT_CAP_PORTX_CAP_SHIFT(port);
	padctl_writel(padctl, value, XUSB_PADCTL_USB2_PORT_CAP);

	value = padctl_readl(padctl, XUSB_PADCTL_USB2_OTG_PADX_CTL0(port));
	value &= ~((XUSB_PADCTL_USB2_OTG_PAD_CTL0_HS_CURR_LEVEL_MASK <<
		    XUSB_PADCTL_USB2_OTG_PAD_CTL0_HS_CURR_LEVEL_SHIFT) |
		   (XUSB_PADCTL_USB2_OTG_PAD_CTL0_HS_SLEW_MASK <<
		    XUSB_PADCTL_USB2_OTG_PAD_CTL0_HS_SLEW_SHIFT) |
		   (XUSB_PADCTL_USB2_OTG_PAD_CTL0_LS_RSLEW_MASK <<
		    XUSB_PADCTL_USB2_OTG_PAD_CTL0_LS_RSLEW_SHIFT) |
		   XUSB_PADCTL_USB2_OTG_PAD_CTL0_PD |
		   XUSB_PADCTL_USB2_OTG_PAD_CTL0_PD2 |
		   XUSB_PADCTL_USB2_OTG_PAD_CTL0_PD_ZI);
	value |= (priv->fuse.hs_curr_level[port] +
		  utmi->hs_curr_level_offset) <<
		XUSB_PADCTL_USB2_OTG_PAD_CTL0_HS_CURR_LEVEL_SHIFT;
	value |= XUSB_PADCTL_USB2_OTG_PAD_CTL0_HS_SLEW_VAL <<
		XUSB_PADCTL_USB2_OTG_PAD_CTL0_HS_SLEW_SHIFT;
	value |= XUSB_PADCTL_USB2_OTG_PAD_CTL0_LS_RSLEW_VAL(port) <<
		XUSB_PADCTL_USB2_OTG_PAD_CTL0_LS_RSLEW_SHIFT;
	padctl_writel(padctl, value, XUSB_PADCTL_USB2_OTG_PADX_CTL0(port));

	value = padctl_readl(padctl, XUSB_PADCTL_USB2_OTG_PADX_CTL1(port));
	value &= ~((XUSB_PADCTL_USB2_OTG_PAD_CTL1_TERM_RANGE_ADJ_MASK <<
		    XUSB_PADCTL_USB2_OTG_PAD_CTL1_TERM_RANGE_ADJ_SHIFT) |
		   (XUSB_PADCTL_USB2_OTG_PAD_CTL1_HS_IREF_CAP_MASK <<
		    XUSB_PADCTL_USB2_OTG_PAD_CTL1_HS_IREF_CAP_SHIFT) |
		   XUSB_PADCTL_USB2_OTG_PAD_CTL1_PD_DR |
		   XUSB_PADCTL_USB2_OTG_PAD_CTL1_PD_CHRP_FORCE_POWERUP |
		   XUSB_PADCTL_USB2_OTG_PAD_CTL1_PD_DISC_FORCE_POWERUP);
	value |= (priv->fuse.hs_term_range_adj <<
		  XUSB_PADCTL_USB2_OTG_PAD_CTL1_TERM_RANGE_ADJ_SHIFT) |
		 (priv->fuse.hs_iref_cap <<
		  XUSB_PADCTL_USB2_OTG_PAD_CTL1_HS_IREF_CAP_SHIFT);
	padctl_writel(padctl, value, XUSB_PADCTL_USB2_OTG_PADX_CTL1(port));

	err = regulator_enable(utmi->supply);
	if (err)
		return err;

	mutex_lock(&padctl->lock);

	if (padctl->utmi_enable++ > 0)
		goto out;

	value = padctl_readl(padctl, XUSB_PADCTL_USB2_BIAS_PAD_CTL0);
	value &= ~XUSB_PADCTL_USB2_BIAS_PAD_CTL0_PD;
	padctl_writel(padctl, value, XUSB_PADCTL_USB2_BIAS_PAD_CTL0);

out:
	mutex_unlock(&padctl->lock);
	return 0;
}

static int tegra124_utmi_phy_power_off(struct phy *phy)
{
	struct tegra_xusb_utmi_phy *utmi = phy_get_drvdata(phy);
	struct tegra_xusb_padctl *padctl = utmi->padctl;
	u32 value;

	mutex_lock(&padctl->lock);

	if (WARN_ON(padctl->utmi_enable == 0))
		goto out;

	if (--padctl->utmi_enable > 0)
		goto out;

	value = padctl_readl(padctl, XUSB_PADCTL_USB2_BIAS_PAD_CTL0);
	value |= XUSB_PADCTL_USB2_BIAS_PAD_CTL0_PD;
	padctl_writel(padctl, value, XUSB_PADCTL_USB2_BIAS_PAD_CTL0);

out:
	regulator_disable(utmi->supply);
	mutex_unlock(&padctl->lock);
	return 0;
}

static const struct phy_ops tegra124_utmi_phy_ops = {
	.init = tegra124_utmi_phy_init,
	.exit = tegra124_utmi_phy_exit,
	.power_on = tegra124_utmi_phy_power_on,
	.power_off = tegra124_utmi_phy_power_off,
	.owner = THIS_MODULE,
};

static int tegra124_hsic_phy_init(struct phy *phy)
{
	struct tegra_xusb_hsic_phy *hsic = phy_get_drvdata(phy);

	return tegra124_xusb_padctl_enable(hsic->padctl);
}

static int tegra124_hsic_phy_exit(struct phy *phy)
{
	struct tegra_xusb_hsic_phy *hsic = phy_get_drvdata(phy);

	return tegra124_xusb_padctl_disable(hsic->padctl);
}

static int tegra124_hsic_phy_power_on(struct phy *phy)
{
	struct tegra_xusb_hsic_phy *hsic = phy_get_drvdata(phy);
	struct tegra_xusb_padctl *padctl = hsic->padctl;
	unsigned int port = hsic->index;
	u32 value;
	int err;

	err = regulator_enable(padctl->vddio_hsic);
	if (err)
		return err;

	padctl_writel(padctl, hsic->strobe_trim,
		      XUSB_PADCTL_HSIC_STRB_TRIM_CONTROL);

	value = padctl_readl(padctl, XUSB_PADCTL_HSIC_PADX_CTL1(port));
	if (hsic->auto_term)
		value |= XUSB_PADCTL_HSIC_PAD_CTL1_AUTO_TERM_EN;
	else
		value &= ~XUSB_PADCTL_HSIC_PAD_CTL1_AUTO_TERM_EN;
	padctl_writel(padctl, value, XUSB_PADCTL_HSIC_PADX_CTL1(port));

	value = padctl_readl(padctl, XUSB_PADCTL_HSIC_PADX_CTL0(port));
	value &= ~((XUSB_PADCTL_HSIC_PAD_CTL0_TX_RTUNEN_MASK <<
		    XUSB_PADCTL_HSIC_PAD_CTL0_TX_RTUNEN_SHIFT) |
		   (XUSB_PADCTL_HSIC_PAD_CTL0_TX_RTUNEP_MASK <<
		    XUSB_PADCTL_HSIC_PAD_CTL0_TX_RTUNEP_SHIFT) |
		   (XUSB_PADCTL_HSIC_PAD_CTL0_TX_RSLEWN_MASK <<
		    XUSB_PADCTL_HSIC_PAD_CTL0_TX_RSLEWN_SHIFT) |
		   (XUSB_PADCTL_HSIC_PAD_CTL0_TX_RSLEWP_MASK <<
		    XUSB_PADCTL_HSIC_PAD_CTL0_TX_RSLEWP_SHIFT));
	value |= (hsic->tx_rtune_n <<
		  XUSB_PADCTL_HSIC_PAD_CTL0_TX_RTUNEN_SHIFT) |
		(hsic->tx_rtune_p <<
		  XUSB_PADCTL_HSIC_PAD_CTL0_TX_RTUNEP_SHIFT) |
		(hsic->tx_rslew_n <<
		 XUSB_PADCTL_HSIC_PAD_CTL0_TX_RSLEWN_SHIFT) |
		(hsic->tx_rslew_p <<
		 XUSB_PADCTL_HSIC_PAD_CTL0_TX_RSLEWP_SHIFT);
	padctl_writel(padctl, value, XUSB_PADCTL_HSIC_PADX_CTL0(port));

	value = padctl_readl(padctl, XUSB_PADCTL_HSIC_PADX_CTL2(port));
	value &= ~((XUSB_PADCTL_HSIC_PAD_CTL2_RX_STROBE_TRIM_MASK <<
		    XUSB_PADCTL_HSIC_PAD_CTL2_RX_STROBE_TRIM_SHIFT) |
		   (XUSB_PADCTL_HSIC_PAD_CTL2_RX_DATA_TRIM_MASK <<
		    XUSB_PADCTL_HSIC_PAD_CTL2_RX_DATA_TRIM_SHIFT));
	value |= (hsic->rx_strobe_trim <<
		  XUSB_PADCTL_HSIC_PAD_CTL2_RX_STROBE_TRIM_SHIFT) |
		(hsic->rx_data_trim <<
		 XUSB_PADCTL_HSIC_PAD_CTL2_RX_DATA_TRIM_SHIFT);
	padctl_writel(padctl, value, XUSB_PADCTL_HSIC_PADX_CTL2(port));

	value = padctl_readl(padctl, XUSB_PADCTL_HSIC_PADX_CTL1(port));
	value &= ~(XUSB_PADCTL_HSIC_PAD_CTL1_RPD_STROBE |
		   XUSB_PADCTL_HSIC_PAD_CTL1_RPU_DATA |
		   XUSB_PADCTL_HSIC_PAD_CTL1_PD_RX |
		   XUSB_PADCTL_HSIC_PAD_CTL1_PD_ZI |
		   XUSB_PADCTL_HSIC_PAD_CTL1_PD_TRX |
		   XUSB_PADCTL_HSIC_PAD_CTL1_PD_TX);
	value |= XUSB_PADCTL_HSIC_PAD_CTL1_RPD_DATA |
		 XUSB_PADCTL_HSIC_PAD_CTL1_RPU_STROBE;
	padctl_writel(padctl, value, XUSB_PADCTL_HSIC_PADX_CTL1(port));

	return 0;
}

static int tegra124_hsic_phy_power_off(struct phy *phy)
{
	struct tegra_xusb_hsic_phy *hsic = phy_get_drvdata(phy);
	struct tegra_xusb_padctl *padctl = hsic->padctl;
	unsigned int port = hsic->index;
	u32 value;

	value = padctl_readl(padctl, XUSB_PADCTL_HSIC_PADX_CTL1(port));
	value |= XUSB_PADCTL_HSIC_PAD_CTL1_PD_RX |
		 XUSB_PADCTL_HSIC_PAD_CTL1_PD_ZI |
		 XUSB_PADCTL_HSIC_PAD_CTL1_PD_TRX |
		 XUSB_PADCTL_HSIC_PAD_CTL1_PD_TX;
	padctl_writel(padctl, value, XUSB_PADCTL_HSIC_PADX_CTL1(port));

	regulator_disable(padctl->vddio_hsic);

	return 0;
}

static void tegra124_hsic_phy_set_idle(struct tegra_xusb_padctl *padctl,
				       unsigned int port, bool idle)
{
	u32 value;

	value = padctl_readl(padctl, XUSB_PADCTL_HSIC_PADX_CTL1(port));
	if (idle)
		value |= XUSB_PADCTL_HSIC_PAD_CTL1_RPD_DATA |
			 XUSB_PADCTL_HSIC_PAD_CTL1_RPU_STROBE;
	else
		value &= ~(XUSB_PADCTL_HSIC_PAD_CTL1_RPD_DATA |
			   XUSB_PADCTL_HSIC_PAD_CTL1_RPU_STROBE);
	padctl_writel(padctl, value, XUSB_PADCTL_HSIC_PADX_CTL1(port));
}

static const struct phy_ops tegra124_hsic_phy_ops = {
	.init = tegra124_hsic_phy_init,
	.exit = tegra124_hsic_phy_exit,
	.power_on = tegra124_hsic_phy_power_on,
	.power_off = tegra124_hsic_phy_power_off,
	.owner = THIS_MODULE,
};

static void tegra124_handle_message(struct tegra_xusb_padctl *padctl,
				    struct tegra_xusb_mbox_msg *msg)
{
	struct tegra_xusb_mbox_msg resp;
	unsigned int i;
	u32 ports;

	resp.cmd = 0;
	switch (msg->cmd) {
	case MBOX_CMD_SAVE_DFE_CTLE_CTX:
		resp.data = msg->data;
		if (tegra124_usb3_phy_save_context(padctl, msg->data) < 0)
			resp.cmd = MBOX_CMD_NAK;
		else
			resp.cmd = MBOX_CMD_ACK;
		break;
	case MBOX_CMD_START_HSIC_IDLE:
	case MBOX_CMD_STOP_HSIC_IDLE:
		ports = msg->data >> (padctl->soc->hsic_port_offset + 1);
		resp.data = msg->data;
		resp.cmd = MBOX_CMD_ACK;
		for (i = 0; i < padctl->soc->num_hsic_phys; i++) {
			if (!(ports & BIT(i)))
				continue;
			if (msg->cmd == MBOX_CMD_START_HSIC_IDLE)
				tegra124_hsic_phy_set_idle(padctl, i, true);
			else
				tegra124_hsic_phy_set_idle(padctl, i, false);
		}
		break;
	default:
		break;
	}

	if (resp.cmd)
		mbox_send_message(padctl->mbox_chan, &resp);
}

static bool tegra124_is_phy_message(struct tegra_xusb_mbox_msg *msg)
{
	switch (msg->cmd) {
	case MBOX_CMD_SAVE_DFE_CTLE_CTX:
	case MBOX_CMD_START_HSIC_IDLE:
	case MBOX_CMD_STOP_HSIC_IDLE:
		return true;
	default:
		return false;
	}
}

static const struct pinctrl_pin_desc tegra124_pins[] = {
	PINCTRL_PIN(PIN_OTG_0,  "otg-0"),
	PINCTRL_PIN(PIN_OTG_1,  "otg-1"),
	PINCTRL_PIN(PIN_OTG_2,  "otg-2"),
	PINCTRL_PIN(PIN_ULPI_0, "ulpi-0"),
	PINCTRL_PIN(PIN_HSIC_0, "hsic-0"),
	PINCTRL_PIN(PIN_HSIC_1, "hsic-1"),
	PINCTRL_PIN(PIN_PCIE_0, "pcie-0"),
	PINCTRL_PIN(PIN_PCIE_1, "pcie-1"),
	PINCTRL_PIN(PIN_PCIE_2, "pcie-2"),
	PINCTRL_PIN(PIN_PCIE_3, "pcie-3"),
	PINCTRL_PIN(PIN_PCIE_4, "pcie-4"),
	PINCTRL_PIN(PIN_SATA_0, "sata-0"),
};

static const char * const tegra124_snps_groups[] = {
	"otg-0",
	"otg-1",
	"otg-2",
	"ulpi-0",
	"hsic-0",
	"hsic-1",
};

static const char * const tegra124_xusb_groups[] = {
	"otg-0",
	"otg-1",
	"otg-2",
	"ulpi-0",
	"hsic-0",
	"hsic-1",
};

static const char * const tegra124_uart_groups[] = {
	"otg-0",
	"otg-1",
	"otg-2",
};

static const char * const tegra124_pcie_groups[] = {
	"pcie-0",
	"pcie-1",
	"pcie-2",
	"pcie-3",
	"pcie-4",
	"sata-0",
};

static const char * const tegra124_usb3_groups[] = {
	"pcie-0",
	"pcie-1",
	"pcie-2",
	"pcie-3",
	"pcie-4",
	"sata-0",
};

static const char * const tegra124_sata_groups[] = {
	"pcie-0",
	"pcie-1",
	"pcie-2",
	"pcie-3",
	"pcie-4",
	"sata-0",
};

static const char * const tegra124_rsvd_groups[] = {
	"otg-0",
	"otg-1",
	"otg-2",
	"pcie-0",
	"pcie-1",
	"pcie-2",
	"pcie-3",
	"pcie-4",
	"sata-0",
};

#define TEGRA124_FUNCTION(_name)					\
	{								\
		.name = #_name,						\
		.num_groups = ARRAY_SIZE(tegra124_##_name##_groups),	\
		.groups = tegra124_##_name##_groups,			\
	}

static struct tegra_xusb_padctl_function tegra124_functions[] = {
	TEGRA124_FUNCTION(snps),
	TEGRA124_FUNCTION(xusb),
	TEGRA124_FUNCTION(uart),
	TEGRA124_FUNCTION(pcie),
	TEGRA124_FUNCTION(usb3),
	TEGRA124_FUNCTION(sata),
	TEGRA124_FUNCTION(rsvd),
};

enum tegra124_function {
	TEGRA124_FUNC_SNPS,
	TEGRA124_FUNC_XUSB,
	TEGRA124_FUNC_UART,
	TEGRA124_FUNC_PCIE,
	TEGRA124_FUNC_USB3,
	TEGRA124_FUNC_SATA,
	TEGRA124_FUNC_RSVD,
};

static const unsigned int tegra124_otg_functions[] = {
	TEGRA124_FUNC_SNPS,
	TEGRA124_FUNC_XUSB,
	TEGRA124_FUNC_UART,
	TEGRA124_FUNC_RSVD,
};

static const unsigned int tegra124_usb_functions[] = {
	TEGRA124_FUNC_SNPS,
	TEGRA124_FUNC_XUSB,
};

static const unsigned int tegra124_pci_functions[] = {
	TEGRA124_FUNC_PCIE,
	TEGRA124_FUNC_USB3,
	TEGRA124_FUNC_SATA,
	TEGRA124_FUNC_RSVD,
};

#define TEGRA124_LANE(_name, _offset, _shift, _mask, _iddq, _funcs)	\
	{								\
		.name = _name,						\
		.offset = _offset,					\
		.shift = _shift,					\
		.mask = _mask,						\
		.iddq = _iddq,						\
		.num_funcs = ARRAY_SIZE(tegra124_##_funcs##_functions),	\
		.funcs = tegra124_##_funcs##_functions,			\
	}

static const struct tegra_xusb_padctl_lane tegra124_lanes[] = {
	TEGRA124_LANE("otg-0",  0x004,  0, 0x3, 0, otg),
	TEGRA124_LANE("otg-1",  0x004,  2, 0x3, 0, otg),
	TEGRA124_LANE("otg-2",  0x004,  4, 0x3, 0, otg),
	TEGRA124_LANE("ulpi-0", 0x004, 12, 0x1, 0, usb),
	TEGRA124_LANE("hsic-0", 0x004, 14, 0x1, 0, usb),
	TEGRA124_LANE("hsic-1", 0x004, 15, 0x1, 0, usb),
	TEGRA124_LANE("pcie-0", 0x134, 16, 0x3, 1, pci),
	TEGRA124_LANE("pcie-1", 0x134, 18, 0x3, 2, pci),
	TEGRA124_LANE("pcie-2", 0x134, 20, 0x3, 3, pci),
	TEGRA124_LANE("pcie-3", 0x134, 22, 0x3, 4, pci),
	TEGRA124_LANE("pcie-4", 0x134, 24, 0x3, 5, pci),
	TEGRA124_LANE("sata-0", 0x134, 26, 0x3, 6, pci),
};

static int
tegra124_xusb_read_fuse_calibration(struct tegra124_xusb_fuse_calibration *fuse)
{
	unsigned int i;
	int err;
	u32 value;

	err = tegra_fuse_readl(TEGRA_FUSE_SKU_CALIB_0, &value);
	if (err < 0)
		return err;

	for (i = 0; i < ARRAY_SIZE(fuse->hs_curr_level); i++) {
		fuse->hs_curr_level[i] =
			(value >> FUSE_SKU_CALIB_HS_CURR_LEVEL_PADX_SHIFT(i)) &
			FUSE_SKU_CALIB_HS_CURR_LEVEL_PAD_MASK;
	}
	fuse->hs_iref_cap =
		(value >> FUSE_SKU_CALIB_HS_IREF_CAP_SHIFT) &
		FUSE_SKU_CALIB_HS_IREF_CAP_MASK;
	fuse->hs_term_range_adj =
		(value >> FUSE_SKU_CALIB_HS_TERM_RANGE_ADJ_SHIFT) &
		FUSE_SKU_CALIB_HS_TERM_RANGE_ADJ_MASK;
	fuse->hs_squelch_level =
		(value >> FUSE_SKU_CALIB_HS_SQUELCH_LEVEL_SHIFT) &
		FUSE_SKU_CALIB_HS_SQUELCH_LEVEL_MASK;

	return 0;
}

static int tegra124_xusb_padctl_probe(struct tegra_xusb_padctl *padctl)
{
	struct tegra124_xusb_padctl *priv;

	priv = devm_kzalloc(padctl->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	padctl->priv = priv;

	return tegra124_xusb_read_fuse_calibration(&priv->fuse);
}

static void tegra124_xusb_padctl_remove(struct tegra_xusb_padctl *padctl)
{
}

const struct tegra_xusb_padctl_soc tegra124_xusb_padctl_soc = {
	.num_pins = ARRAY_SIZE(tegra124_pins),
	.pins = tegra124_pins,
	.num_functions = ARRAY_SIZE(tegra124_functions),
	.functions = tegra124_functions,
	.num_lanes = ARRAY_SIZE(tegra124_lanes),
	.lanes = tegra124_lanes,
	.pcie_phy_ops = &tegra124_pcie_phy_ops,
	.sata_phy_ops = &tegra124_sata_phy_ops,
	.num_usb3_phys = 2,
	.usb3_phy_ops = &tegra124_usb3_phy_ops,
	.num_utmi_phys = 3,
	.utmi_phy_ops = &tegra124_utmi_phy_ops,
	.num_hsic_phys = 2,
	.hsic_phy_ops = &tegra124_hsic_phy_ops,
	.hsic_port_offset = 6,
	.is_phy_message = tegra124_is_phy_message,
	.handle_message = tegra124_handle_message,
	.probe = tegra124_xusb_padctl_probe,
	.remove = tegra124_xusb_padctl_remove,
};
EXPORT_SYMBOL_GPL(tegra124_xusb_padctl_soc);

MODULE_AUTHOR("Thierry Reding <treding@nvidia.com>");
MODULE_DESCRIPTION("Tegra 124 XUSB Pad Control driver");
MODULE_LICENSE("GPL v2");
