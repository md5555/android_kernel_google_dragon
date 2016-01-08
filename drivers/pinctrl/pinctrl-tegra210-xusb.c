/*
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
 * Copyright (C) 2015 Google, Inc.
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

#include <linux/clk.h>
#include <linux/clk/tegra.h>
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
#include <linux/usb/of.h>

#include <soc/tegra/fuse.h>
#include <soc/tegra/xusb.h>

#include <dt-bindings/pinctrl/pinctrl-tegra-xusb.h>

#include "core.h"
#include "pinctrl-tegra-xusb.h"
#include "pinctrl-utils.h"

#define FUSE_SKU_CALIB_HS_CURR_LEVEL_PADX_SHIFT(x) \
					((x) ? (11 + ((x) - 1) * 6) : 0)
#define FUSE_SKU_CALIB_HS_CURR_LEVEL_PAD_MASK 0x3f
#define FUSE_SKU_CALIB_HS_TERM_RANGE_ADJ_SHIFT 7
#define FUSE_SKU_CALIB_HS_TERM_RANGE_ADJ_MASK 0xf

#define FUSE_USB_CALIB_EXT_RPD_CTRL_SHIFT 0
#define FUSE_USB_CALIB_EXT_RPD_CTRL_MASK 0x1f

#define XUSB_PADCTL_USB2_PAD_MUX 0x004
#define XUSB_PADCTL_USB2_PAD_MUX_HSIC_PAD_TRK_SHIFT 16
#define XUSB_PADCTL_USB2_PAD_MUX_HSIC_PAD_TRK_MASK 0x3
#define XUSB_PADCTL_USB2_PAD_MUX_HSIC_PAD_TRK_XUSB 0x1
#define XUSB_PADCTL_USB2_PAD_MUX_USB2_BIAS_PAD_SHIFT 18
#define XUSB_PADCTL_USB2_PAD_MUX_USB2_BIAS_PAD_MASK 0x3
#define XUSB_PADCTL_USB2_PAD_MUX_USB2_BIAS_PAD_XUSB 0x1

#define XUSB_PADCTL_USB2_PORT_CAP 0x008
#define XUSB_PADCTL_USB2_PORT_CAP_PORTX_CAP_SHIFT(x) ((x) * 4)
#define XUSB_PADCTL_USB2_PORT_CAP_PORT_CAP_MASK 0x3
#define XUSB_PADCTL_USB2_PORT_CAP_DISABLED 0x0
#define XUSB_PADCTL_USB2_PORT_CAP_HOST 0x1
#define XUSB_PADCTL_USB2_PORT_CAP_DEVICE 0x2
#define XUSB_PADCTL_USB2_PORT_CAP_OTG 0x3

#define XUSB_PADCTL_SS_PORT_MAP 0x014
#define XUSB_PADCTL_SS_PORT_MAP_PORTX_SHIFT(x) ((x) * 5)
#define XUSB_PADCTL_SS_PORT_MAP_PORT_MASK 0x7
#define XUSB_PADCTL_SS_PORT_MAP_PORT_DISABLED 0x7

#define XUSB_PADCTL_ELPG_PROGRAM0 0x020
#define XUSB_PADCTL_ELPG_PROGRAM0_HSIC_PORTX_WAKEUP_EVENT(x) (1 << (30 + (x)))
#define XUSB_PADCTL_ELPG_PROGRAM0_HSIC_PORTX_WAKE_ENABLE(x) (1 << (28 + (x)))
#define XUSB_PADCTL_ELPG_PROGRAM0_SS_PORTX_WAKEUP_EVENT(x) (1 << (21 + (x)))
#define XUSB_PADCTL_ELPG_PROGRAM0_SS_PORTX_WAKE_ENABLE(x) (1 << (14 + (x)))
#define XUSB_PADCTL_ELPG_PROGRAM0_USB2_PORTX_WAKEUP_EVENT(x) (1 << (7 + (x)))
#define XUSB_PADCTL_ELPG_PROGRAM0_USB2_PORTX_WAKE_ENABLE(x) (1 << (x))

#define XUSB_PADCTL_ELPG_PROGRAM1 0x024
#define XUSB_PADCTL_ELPG_PROGRAM1_AUX_MUX_LP0_VCORE_DOWN (1 << 31)
#define XUSB_PADCTL_ELPG_PROGRAM1_AUX_MUX_LP0_CLAMP_EN_EARLY (1 << 30)
#define XUSB_PADCTL_ELPG_PROGRAM1_AUX_MUX_LP0_CLAMP_EN (1 << 29)
#define XUSB_PADCTL_ELPG_PROGRAM1_SSPX_ELPG_VCORE_DOWN(x) (1 << (2 + (x) * 3))
#define XUSB_PADCTL_ELPG_PROGRAM1_SSPX_ELPG_CLAMP_EN_EARLY(x) \
							(1 << (1 + (x) * 3))
#define XUSB_PADCTL_ELPG_PROGRAM1_SSPX_ELPG_CLAMP_EN(x) (1 << ((x) * 3))

#define XUSB_PADCTL_USB3_PAD_MUX 0x028

#define XUSB_PADCTL_USB2_BATTERY_CHRG_OTGPADX_CTL1(x) (0x084 + (x) * 0x40)
#define XUSB_PADCTL_USB2_BATTERY_CHRG_OTGPAD_CTL1_VREG_LEV_SHIFT 7
#define XUSB_PADCTL_USB2_BATTERY_CHRG_OTGPAD_CTL1_VREG_LEV_MASK 0x3
#define XUSB_PADCTL_USB2_BATTERY_CHRG_OTGPAD_CTL1_VREG_LEV_EN 0x1
#define XUSB_PADCTL_USB2_BATTERY_CHRG_OTGPAD_CTL1_VREG_FIX18 (1 << 6)

#define XUSB_PADCTL_USB2_OTG_PADX_CTL0(x) (0x088 + (x) * 0x40)
#define XUSB_PADCTL_USB2_OTG_PAD_CTL0_PD_ZI (1 << 29)
#define XUSB_PADCTL_USB2_OTG_PAD_CTL0_PD2 (1 << 27)
#define XUSB_PADCTL_USB2_OTG_PAD_CTL0_PD (1 << 26)
#define XUSB_PADCTL_USB2_OTG_PAD_CTL0_HS_CURR_LEVEL_SHIFT 0
#define XUSB_PADCTL_USB2_OTG_PAD_CTL0_HS_CURR_LEVEL_MASK 0x3f

#define XUSB_PADCTL_USB2_OTG_PADX_CTL1(x) (0x08c + (x) * 0x40)
#define XUSB_PADCTL_USB2_OTG_PAD_CTL1_RPD_CTRL_SHIFT 26
#define XUSB_PADCTL_USB2_OTG_PAD_CTL1_RPD_CTRL_MASK 0x1f
#define XUSB_PADCTL_USB2_OTG_PAD_CTL1_TERM_RANGE_ADJ_SHIFT 3
#define XUSB_PADCTL_USB2_OTG_PAD_CTL1_TERM_RANGE_ADJ_MASK 0xf
#define XUSB_PADCTL_USB2_OTG_PAD_CTL1_PD_DR (1 << 2)
#define XUSB_PADCTL_USB2_OTG_PAD_CTL1_PD_DISC_OVRD (1 << 1)
#define XUSB_PADCTL_USB2_OTG_PAD_CTL1_PD_CHRP_OVRD (1 << 0)

#define XUSB_PADCTL_USB2_BIAS_PAD_CTL0 0x284
#define XUSB_PADCTL_USB2_BIAS_PAD_CTL0_PD (1 << 11)
#define XUSB_PADCTL_USB2_BIAS_PAD_CTL0_HS_DISCON_LEVEL_SHIFT 3
#define XUSB_PADCTL_USB2_BIAS_PAD_CTL0_HS_DISCON_LEVEL_MASK 0x7
#define XUSB_PADCTL_USB2_BIAS_PAD_CTL0_HS_DISCON_LEVEL_VAL 0x7
#define XUSB_PADCTL_USB2_BIAS_PAD_CTL0_HS_SQUELCH_LEVEL_SHIFT 0
#define XUSB_PADCTL_USB2_BIAS_PAD_CTL0_HS_SQUELCH_LEVEL_MASK 0x7
#define XUSB_PADCTL_USB2_BIAS_PAD_CTL0_HS_SQUELCH_LEVEL_VAL 0x2

#define XUSB_PADCTL_USB2_BIAS_PAD_CTL1 0x288
#define XUSB_PADCTL_USB2_BIAS_PAD_CTL1_PD_TRK (1 << 26)
#define XUSB_PADCTL_USB2_BIAS_PAD_CTL1_TRK_DONE_RESET_TIMER_SHIFT 19
#define XUSB_PADCTL_USB2_BIAS_PAD_CTL1_TRK_DONE_RESET_TIMER_MASK 0x7f
#define XUSB_PADCTL_USB2_BIAS_PAD_CTL1_TRK_DONE_RESET_TIMER_VAL 0x0a
#define XUSB_PADCTL_USB2_BIAS_PAD_CTL1_TRK_START_TIMER_SHIFT 12
#define XUSB_PADCTL_USB2_BIAS_PAD_CTL1_TRK_START_TIMER_MASK 0x7f
#define XUSB_PADCTL_USB2_BIAS_PAD_CTL1_TRK_START_TIMER_VAL 0x1e
#define XUSB_PADCTL_USB2_BIAS_PAD_CTL1_PCTRL_MASK 0x3f
#define XUSB_PADCTL_USB2_BIAS_PAD_CTL1_PCTRL_SHIFT 6
#define XUSB_PADCTL_USB2_BIAS_PAD_CTL1_TCTRL_MASK 0x3f
#define XUSB_PADCTL_USB2_BIAS_PAD_CTL1_TCTRL_SHIFT 0

#define XUSB_PADCTL_HSIC_PADX_CTL0(x) (0x300 + (x) * 0x20)
#define XUSB_PADCTL_HSIC_PAD_CTL0_RPU_STROBE (1 << 18)
#define XUSB_PADCTL_HSIC_PAD_CTL0_RPU_DATA1 (1 << 17)
#define XUSB_PADCTL_HSIC_PAD_CTL0_RPU_DATA0 (1 << 16)
#define XUSB_PADCTL_HSIC_PAD_CTL0_RPD_STROBE (1 << 15)
#define XUSB_PADCTL_HSIC_PAD_CTL0_RPD_DATA1 (1 << 14)
#define XUSB_PADCTL_HSIC_PAD_CTL0_RPD_DATA0 (1 << 13)
#define XUSB_PADCTL_HSIC_PAD_CTL0_PD_ZI_STROBE (1 << 9)
#define XUSB_PADCTL_HSIC_PAD_CTL0_PD_ZI_DATA1 (1 << 8)
#define XUSB_PADCTL_HSIC_PAD_CTL0_PD_ZI_DATA0 (1 << 7)
#define XUSB_PADCTL_HSIC_PAD_CTL0_PD_RX_STROBE (1 << 6)
#define XUSB_PADCTL_HSIC_PAD_CTL0_PD_RX_DATA1 (1 << 5)
#define XUSB_PADCTL_HSIC_PAD_CTL0_PD_RX_DATA0 (1 << 4)
#define XUSB_PADCTL_HSIC_PAD_CTL0_PD_TX_STROBE (1 << 3)
#define XUSB_PADCTL_HSIC_PAD_CTL0_PD_TX_DATA1 (1 << 2)
#define XUSB_PADCTL_HSIC_PAD_CTL0_PD_TX_DATA0 (1 << 1)

#define XUSB_PADCTL_HSIC_PADX_CTL1(x) (0x304 + (x) * 0x20)
#define XUSB_PADCTL_HSIC_PAD_CTL1_TX_RTUNEP_SHIFT 0
#define XUSB_PADCTL_HSIC_PAD_CTL1_TX_RTUNEP_MASK 0xf

#define XUSB_PADCTL_HSIC_PADX_CTL2(x) (0x308 + (x) * 0x20)
#define XUSB_PADCTL_HSIC_PAD_CTL2_RX_STROBE_TRIM_SHIFT 8
#define XUSB_PADCTL_HSIC_PAD_CTL2_RX_STROBE_TRIM_MASK 0xf
#define XUSB_PADCTL_HSIC_PAD_CTL2_RX_DATA_TRIM_SHIFT 0
#define XUSB_PADCTL_HSIC_PAD_CTL2_RX_DATA_TRIM_MASK 0xff

#define XUSB_PADCTL_HSIC_PAD_TRK_CTL 0x340
#define XUSB_PADCTL_HSIC_PAD_TRK_CTL_PD_TRK (1 << 19)
#define XUSB_PADCTL_HSIC_PAD_TRK_CTL_TRK_DONE_RESET_TIMER_SHIFT 12
#define XUSB_PADCTL_HSIC_PAD_TRK_CTL_TRK_DONE_RESET_TIMER_MASK 0x7f
#define XUSB_PADCTL_HSIC_PAD_TRK_CTL_TRK_DONE_RESET_TIMER_VAL 0x0a
#define XUSB_PADCTL_HSIC_PAD_TRK_CTL_TRK_START_TIMER_SHIFT 5
#define XUSB_PADCTL_HSIC_PAD_TRK_CTL_TRK_START_TIMER_MASK 0x7f
#define XUSB_PADCTL_HSIC_PAD_TRK_CTL_TRK_START_TIMER_VAL 0x1e

#define XUSB_PADCTL_HSIC_STRB_TRIM_CONTROL 0x344

#define XUSB_PADCTL_UPHY_PLL_P0_CTL1 0x360
#define XUSB_PADCTL_UPHY_PLL_CTL1_FREQ_NDIV_SHIFT 20
#define XUSB_PADCTL_UPHY_PLL_CTL1_FREQ_NDIV_MASK 0xff
#define XUSB_PADCTL_UPHY_PLL_CTL1_FREQ_NDIV_USB_VAL 0x19
#define XUSB_PADCTL_UPHY_PLL_CTL1_FREQ_NDIV_SATA_VAL 0x1e
#define XUSB_PADCTL_UPHY_PLL_CTL1_FREQ_MDIV_SHIFT 16
#define XUSB_PADCTL_UPHY_PLL_CTL1_FREQ_MDIV_MASK 0x3
#define XUSB_PADCTL_UPHY_PLL_CTL1_LOCKDET_STATUS (1 << 15)
#define XUSB_PADCTL_UPHY_PLL_CTL1_PWR_OVRD (1 << 4)
#define XUSB_PADCTL_UPHY_PLL_CTL1_ENABLE (1 << 3)
#define XUSB_PADCTL_UPHY_PLL_CTL1_SLEEP_SHIFT 1
#define XUSB_PADCTL_UPHY_PLL_CTL1_SLEEP_MASK 0x3
#define XUSB_PADCTL_UPHY_PLL_CTL1_IDDQ (1 << 0)

#define XUSB_PADCTL_UPHY_PLL_P0_CTL2 0x364
#define XUSB_PADCTL_UPHY_PLL_CTL2_CAL_CTRL_SHIFT 4
#define XUSB_PADCTL_UPHY_PLL_CTL2_CAL_CTRL_MASK 0xffffff
#define XUSB_PADCTL_UPHY_PLL_CTL2_CAL_CTRL_VAL 0x136
#define XUSB_PADCTL_UPHY_PLL_CTL2_CAL_OVRD (1 << 2)
#define XUSB_PADCTL_UPHY_PLL_CTL2_CAL_DONE (1 << 1)
#define XUSB_PADCTL_UPHY_PLL_CTL2_CAL_EN (1 << 0)

#define XUSB_PADCTL_UPHY_PLL_P0_CTL4 0x36c
#define XUSB_PADCTL_UPHY_PLL_CTL4_TXCLKREF_EN (1 << 15)
#define XUSB_PADCTL_UPHY_PLL_CTL4_TXCLKREF_SEL_SHIFT 12
#define XUSB_PADCTL_UPHY_PLL_CTL4_TXCLKREF_SEL_MASK 0x3
#define XUSB_PADCTL_UPHY_PLL_CTL4_TXCLKREF_SEL_USB_VAL 0x2
#define XUSB_PADCTL_UPHY_PLL_CTL4_TXCLKREF_SEL_SATA_VAL 0x0
#define XUSB_PADCTL_UPHY_PLL_CTL4_REFCLKBUF_EN (1 << 8)
#define XUSB_PADCTL_UPHY_PLL_CTL4_REFCLK_SEL_SHIFT 4
#define XUSB_PADCTL_UPHY_PLL_CTL4_REFCLK_SEL_MASK 0xf

#define XUSB_PADCTL_UPHY_PLL_P0_CTL5 0x370
#define XUSB_PADCTL_UPHY_PLL_CTL5_DCO_CTRL_SHIFT 16
#define XUSB_PADCTL_UPHY_PLL_CTL5_DCO_CTRL_MASK 0xff
#define XUSB_PADCTL_UPHY_PLL_CTL5_DCO_CTRL_VAL 0x2a

#define XUSB_PADCTL_UPHY_PLL_P0_CTL8 0x37c
#define XUSB_PADCTL_UPHY_PLL_CTL8_RCAL_DONE (1 << 31)
#define XUSB_PADCTL_UPHY_PLL_CTL8_RCAL_OVRD (1 << 15)
#define XUSB_PADCTL_UPHY_PLL_CTL8_RCAL_CLK_EN (1 << 13)
#define XUSB_PADCTL_UPHY_PLL_CTL8_RCAL_EN (1 << 12)

#define XUSB_PADCTL_UPHY_MISC_PAD_PX_CTL1(x) (0x460 + (x) * 0x40)
#define XUSB_PADCTL_UPHY_MISC_PAD_CTL1_AUX_RX_IDLE_MODE_SHIFT 20
#define XUSB_PADCTL_UPHY_MISC_PAD_CTL1_AUX_RX_IDLE_MODE_MASK 0x3
#define XUSB_PADCTL_UPHY_MISC_PAD_CTL1_AUX_RX_IDLE_MODE_VAL 0x1
#define XUSB_PADCTL_UPHY_MISC_PAD_CTL1_AUX_RX_TERM_EN BIT(18)
#define XUSB_PADCTL_UPHY_MISC_PAD_CTL1_AUX_RX_MODE_OVRD BIT(13)

#define XUSB_PADCTL_UPHY_PLL_S0_CTL1 0x860

#define XUSB_PADCTL_UPHY_PLL_S0_CTL2 0x864

#define XUSB_PADCTL_UPHY_PLL_S0_CTL4 0x86c

#define XUSB_PADCTL_UPHY_PLL_S0_CTL5 0x870

#define XUSB_PADCTL_UPHY_PLL_S0_CTL8 0x87c

#define XUSB_PADCTL_UPHY_MISC_PAD_S0_CTL1 0x960

#define XUSB_PADCTL_UPHY_USB3_PADX_ECTL1(x) (0xa60 + (x) * 0x40)
#define XUSB_PADCTL_UPHY_USB3_PAD_ECTL1_TX_TERM_CTRL_SHIFT 16
#define XUSB_PADCTL_UPHY_USB3_PAD_ECTL1_TX_TERM_CTRL_MASK 0x3
#define XUSB_PADCTL_UPHY_USB3_PAD_ECTL1_TX_TERM_CTRL_VAL 0x2

#define XUSB_PADCTL_UPHY_USB3_PADX_ECTL2(x) (0xa64 + (x) * 0x40)
#define XUSB_PADCTL_UPHY_USB3_PAD_ECTL2_RX_CTLE_SHIFT 0
#define XUSB_PADCTL_UPHY_USB3_PAD_ECTL2_RX_CTLE_MASK 0xffff
#define XUSB_PADCTL_UPHY_USB3_PAD_ECTL2_RX_CTLE_VAL 0x00fc

#define XUSB_PADCTL_UPHY_USB3_PADX_ECTL3(x) (0xa68 + (x) * 0x40)
#define XUSB_PADCTL_UPHY_USB3_PAD_ECTL3_RX_DFE_VAL 0xc0077f1f

#define XUSB_PADCTL_UPHY_USB3_PADX_ECTL4(x) (0xa6c + (x) * 0x40)
#define XUSB_PADCTL_UPHY_USB3_PAD_ECTL4_RX_CDR_CTRL_SHIFT 16
#define XUSB_PADCTL_UPHY_USB3_PAD_ECTL4_RX_CDR_CTRL_MASK 0xffff
#define XUSB_PADCTL_UPHY_USB3_PAD_ECTL4_RX_CDR_CTRL_VAL 0x01c7

#define XUSB_PADCTL_UPHY_USB3_PADX_ECTL6(x) (0xa74 + (x) * 0x40)
#define XUSB_PADCTL_UPHY_USB3_PAD_ECTL6_RX_EQ_CTRL_H_VAL 0xfcf01368

#define XUSB_PADCTL_USB2_VBUS_ID 0xc60
#define XUSB_PADCTL_USB2_VBUS_ID_ID_OVERRIDE_SHIFT 18
#define XUSB_PADCTL_USB2_VBUS_ID_ID_OVERRIDE_MASK 0xf
#define XUSB_PADCTL_USB2_VBUS_ID_ID_OVERRIDE_FLOAT 0x8
#define XUSB_PADCTL_USB2_VBUS_ID_ID_OVERRIDE_GND 0x0
#define XUSB_PADCTL_USB2_VBUS_ID_VBUS_OVERRIDE (1 << 14)

struct tegra210_xusb_fuse_calibration {
	u32 hs_curr_level[4];
	u32 hs_term_range_adj;
	u32 rpd_ctrl;
};

struct tegra210_xusb_padctl_saved_regs {
	u32 usb2_pad_mux;
	u32 usb3_pad_mux;
};

struct tegra210_xusb_padctl {
	struct tegra210_xusb_fuse_calibration fuse;

	struct reset_control *pex_rst;
	struct reset_control *sata_rst;

	struct clk *usb2_trk;
	struct clk *hsic_trk;
	struct clk *pll_e;

	unsigned int pex_enable;
	unsigned int sata_enable;

	struct tegra210_xusb_padctl_saved_regs saved_regs;
};

static int tegra210_pex_uphy_enable(struct tegra_xusb_padctl *padctl)
{
	struct tegra210_xusb_padctl *priv = padctl->priv;
	unsigned long timeout;
	u32 value;
	int err;

	mutex_lock(&padctl->lock);

	if (priv->pex_enable > 0) {
		priv->pex_enable++;
		mutex_unlock(&padctl->lock);
		return 0;
	}

	err = clk_prepare_enable(priv->pll_e);
	if (err < 0)
		goto unlock;

	reset_control_deassert(priv->pex_rst);

	value = padctl_readl(padctl, XUSB_PADCTL_UPHY_PLL_P0_CTL2);
	value &= ~(XUSB_PADCTL_UPHY_PLL_CTL2_CAL_CTRL_MASK <<
		   XUSB_PADCTL_UPHY_PLL_CTL2_CAL_CTRL_SHIFT);
	value |= XUSB_PADCTL_UPHY_PLL_CTL2_CAL_CTRL_VAL <<
		 XUSB_PADCTL_UPHY_PLL_CTL2_CAL_CTRL_SHIFT;
	padctl_writel(padctl, value, XUSB_PADCTL_UPHY_PLL_P0_CTL2);

	value = padctl_readl(padctl, XUSB_PADCTL_UPHY_PLL_P0_CTL5);
	value &= ~(XUSB_PADCTL_UPHY_PLL_CTL5_DCO_CTRL_MASK <<
		   XUSB_PADCTL_UPHY_PLL_CTL5_DCO_CTRL_SHIFT);
	value |= XUSB_PADCTL_UPHY_PLL_CTL5_DCO_CTRL_VAL <<
		 XUSB_PADCTL_UPHY_PLL_CTL5_DCO_CTRL_SHIFT;
	padctl_writel(padctl, value, XUSB_PADCTL_UPHY_PLL_P0_CTL5);

	value = padctl_readl(padctl, XUSB_PADCTL_UPHY_PLL_P0_CTL1);
	value |= XUSB_PADCTL_UPHY_PLL_CTL1_PWR_OVRD;
	padctl_writel(padctl, value, XUSB_PADCTL_UPHY_PLL_P0_CTL1);

	value = padctl_readl(padctl, XUSB_PADCTL_UPHY_PLL_P0_CTL2);
	value |= XUSB_PADCTL_UPHY_PLL_CTL2_CAL_OVRD;
	padctl_writel(padctl, value, XUSB_PADCTL_UPHY_PLL_P0_CTL2);

	value = padctl_readl(padctl, XUSB_PADCTL_UPHY_PLL_P0_CTL8);
	value |= XUSB_PADCTL_UPHY_PLL_CTL8_RCAL_OVRD;
	padctl_writel(padctl, value, XUSB_PADCTL_UPHY_PLL_P0_CTL8);

	value = padctl_readl(padctl, XUSB_PADCTL_UPHY_PLL_P0_CTL4);
	value &= ~((XUSB_PADCTL_UPHY_PLL_CTL4_TXCLKREF_SEL_MASK <<
		    XUSB_PADCTL_UPHY_PLL_CTL4_TXCLKREF_SEL_SHIFT) |
		   (XUSB_PADCTL_UPHY_PLL_CTL4_REFCLK_SEL_MASK <<
		    XUSB_PADCTL_UPHY_PLL_CTL4_REFCLK_SEL_SHIFT));
	value |= (XUSB_PADCTL_UPHY_PLL_CTL4_TXCLKREF_SEL_USB_VAL <<
		  XUSB_PADCTL_UPHY_PLL_CTL4_TXCLKREF_SEL_SHIFT) |
		 XUSB_PADCTL_UPHY_PLL_CTL4_TXCLKREF_EN;
	padctl_writel(padctl, value, XUSB_PADCTL_UPHY_PLL_P0_CTL4);

	value = padctl_readl(padctl, XUSB_PADCTL_UPHY_PLL_P0_CTL1);
	value &= ~((XUSB_PADCTL_UPHY_PLL_CTL1_FREQ_MDIV_MASK <<
		    XUSB_PADCTL_UPHY_PLL_CTL1_FREQ_MDIV_SHIFT) |
		   (XUSB_PADCTL_UPHY_PLL_CTL1_FREQ_NDIV_MASK <<
		    XUSB_PADCTL_UPHY_PLL_CTL1_FREQ_NDIV_SHIFT));
	value |= XUSB_PADCTL_UPHY_PLL_CTL1_FREQ_NDIV_USB_VAL <<
		 XUSB_PADCTL_UPHY_PLL_CTL1_FREQ_NDIV_SHIFT;
	padctl_writel(padctl, value, XUSB_PADCTL_UPHY_PLL_P0_CTL1);

	value = padctl_readl(padctl, XUSB_PADCTL_UPHY_PLL_P0_CTL1);
	value &= ~XUSB_PADCTL_UPHY_PLL_CTL1_IDDQ;
	padctl_writel(padctl, value, XUSB_PADCTL_UPHY_PLL_P0_CTL1);

	value = padctl_readl(padctl, XUSB_PADCTL_UPHY_PLL_P0_CTL1);
	value &= ~(XUSB_PADCTL_UPHY_PLL_CTL1_SLEEP_MASK <<
		   XUSB_PADCTL_UPHY_PLL_CTL1_SLEEP_SHIFT);
	padctl_writel(padctl, value, XUSB_PADCTL_UPHY_PLL_P0_CTL1);

	usleep_range(10, 20);

	value = padctl_readl(padctl, XUSB_PADCTL_UPHY_PLL_P0_CTL4);
	value |= XUSB_PADCTL_UPHY_PLL_CTL4_REFCLKBUF_EN;
	padctl_writel(padctl, value, XUSB_PADCTL_UPHY_PLL_P0_CTL4);

	value = padctl_readl(padctl, XUSB_PADCTL_UPHY_PLL_P0_CTL2);
	value |= XUSB_PADCTL_UPHY_PLL_CTL2_CAL_EN;
	padctl_writel(padctl, value, XUSB_PADCTL_UPHY_PLL_P0_CTL2);

	timeout = jiffies + msecs_to_jiffies(100);

	while (time_before(jiffies, timeout)) {
		value = padctl_readl(padctl, XUSB_PADCTL_UPHY_PLL_P0_CTL2);
		if (value & XUSB_PADCTL_UPHY_PLL_CTL2_CAL_DONE)
			break;

		usleep_range(10, 20);
	}

	value = padctl_readl(padctl, XUSB_PADCTL_UPHY_PLL_P0_CTL2);
	if (!(value & XUSB_PADCTL_UPHY_PLL_CTL2_CAL_DONE)) {
		err = -ETIMEDOUT;
		goto reset;
	}

	value = padctl_readl(padctl, XUSB_PADCTL_UPHY_PLL_P0_CTL2);
	value &= ~XUSB_PADCTL_UPHY_PLL_CTL2_CAL_EN;
	padctl_writel(padctl, value, XUSB_PADCTL_UPHY_PLL_P0_CTL2);

	timeout = jiffies + msecs_to_jiffies(100);

	while (time_before(jiffies, timeout)) {
		value = padctl_readl(padctl, XUSB_PADCTL_UPHY_PLL_P0_CTL2);
		if (!(value & XUSB_PADCTL_UPHY_PLL_CTL2_CAL_DONE))
			break;

		usleep_range(10, 20);
	}

	value = padctl_readl(padctl, XUSB_PADCTL_UPHY_PLL_P0_CTL2);
	if (value & XUSB_PADCTL_UPHY_PLL_CTL2_CAL_DONE) {
		err = -ETIMEDOUT;
		goto reset;
	}

	value = padctl_readl(padctl, XUSB_PADCTL_UPHY_PLL_P0_CTL1);
	value |= XUSB_PADCTL_UPHY_PLL_CTL1_ENABLE;
	padctl_writel(padctl, value, XUSB_PADCTL_UPHY_PLL_P0_CTL1);

	timeout = jiffies + msecs_to_jiffies(100);

	while (time_before(jiffies, timeout)) {
		value = padctl_readl(padctl, XUSB_PADCTL_UPHY_PLL_P0_CTL1);
		if (value & XUSB_PADCTL_UPHY_PLL_CTL1_LOCKDET_STATUS)
			break;

		usleep_range(10, 20);
	}

	value = padctl_readl(padctl, XUSB_PADCTL_UPHY_PLL_P0_CTL1);
	if (!(value & XUSB_PADCTL_UPHY_PLL_CTL1_LOCKDET_STATUS)) {
		err = -ETIMEDOUT;
		goto reset;
	}

	value = padctl_readl(padctl, XUSB_PADCTL_UPHY_PLL_P0_CTL8);
	value |= XUSB_PADCTL_UPHY_PLL_CTL8_RCAL_EN |
		 XUSB_PADCTL_UPHY_PLL_CTL8_RCAL_CLK_EN;
	padctl_writel(padctl, value, XUSB_PADCTL_UPHY_PLL_P0_CTL8);

	timeout = jiffies + msecs_to_jiffies(100);

	while (time_before(jiffies, timeout)) {
		value = padctl_readl(padctl, XUSB_PADCTL_UPHY_PLL_P0_CTL8);
		if (value & XUSB_PADCTL_UPHY_PLL_CTL8_RCAL_DONE)
			break;

		usleep_range(10, 20);
	}

	value = padctl_readl(padctl, XUSB_PADCTL_UPHY_PLL_P0_CTL8);
	if (!(value & XUSB_PADCTL_UPHY_PLL_CTL8_RCAL_DONE)) {
		err = -ETIMEDOUT;
		goto reset;
	}

	value = padctl_readl(padctl, XUSB_PADCTL_UPHY_PLL_P0_CTL8);
	value &= ~XUSB_PADCTL_UPHY_PLL_CTL8_RCAL_EN;
	padctl_writel(padctl, value, XUSB_PADCTL_UPHY_PLL_P0_CTL8);

	timeout = jiffies + msecs_to_jiffies(100);

	while (time_before(jiffies, timeout)) {
		value = padctl_readl(padctl, XUSB_PADCTL_UPHY_PLL_P0_CTL8);
		if (!(value & XUSB_PADCTL_UPHY_PLL_CTL8_RCAL_DONE))
			break;

		usleep_range(10, 20);
	}

	value = padctl_readl(padctl, XUSB_PADCTL_UPHY_PLL_P0_CTL8);
	if (value & XUSB_PADCTL_UPHY_PLL_CTL8_RCAL_DONE) {
		err = -ETIMEDOUT;
		goto reset;
	}

	value = padctl_readl(padctl, XUSB_PADCTL_UPHY_PLL_P0_CTL8);
	value &= ~XUSB_PADCTL_UPHY_PLL_CTL8_RCAL_CLK_EN;
	padctl_writel(padctl, value, XUSB_PADCTL_UPHY_PLL_P0_CTL8);

	tegra210_xusb_pll_hw_control_enable();

	value = padctl_readl(padctl, XUSB_PADCTL_UPHY_PLL_P0_CTL1);
	value &= ~XUSB_PADCTL_UPHY_PLL_CTL1_PWR_OVRD;
	padctl_writel(padctl, value, XUSB_PADCTL_UPHY_PLL_P0_CTL1);

	value = padctl_readl(padctl, XUSB_PADCTL_UPHY_PLL_P0_CTL2);
	value &= ~XUSB_PADCTL_UPHY_PLL_CTL2_CAL_OVRD;
	padctl_writel(padctl, value, XUSB_PADCTL_UPHY_PLL_P0_CTL2);

	value = padctl_readl(padctl, XUSB_PADCTL_UPHY_PLL_P0_CTL8);
	value &= ~XUSB_PADCTL_UPHY_PLL_CTL8_RCAL_OVRD;
	padctl_writel(padctl, value, XUSB_PADCTL_UPHY_PLL_P0_CTL8);

	usleep_range(10, 20);

	tegra210_xusb_pll_hw_sequence_start();

	priv->pex_enable++;
	mutex_unlock(&padctl->lock);

	return 0;
reset:
	reset_control_assert(priv->pex_rst);
	clk_disable_unprepare(priv->pll_e);
unlock:
	mutex_unlock(&padctl->lock);
	return err;
}

static void tegra210_pex_uphy_disable(struct tegra_xusb_padctl *padctl)
{
	struct tegra210_xusb_padctl *priv = padctl->priv;

	mutex_lock(&padctl->lock);

	if (WARN_ON(priv->pex_enable == 0))
		goto out;

	if (--priv->pex_enable > 0)
		goto out;

	reset_control_assert(priv->pex_rst);
	clk_disable_unprepare(priv->pll_e);
out:
	mutex_unlock(&padctl->lock);
}

static int tegra210_sata_uphy_enable(struct tegra_xusb_padctl *padctl, bool usb)
{
	struct tegra210_xusb_padctl *priv = padctl->priv;
	unsigned long timeout;
	u32 value;
	int err;

	mutex_lock(&padctl->lock);

	if (priv->sata_enable > 0) {
		priv->sata_enable++;
		mutex_unlock(&padctl->lock);
		return 0;
	}

	err = clk_prepare_enable(priv->pll_e);
	if (err < 0)
		goto unlock;

	reset_control_deassert(priv->sata_rst);

	value = padctl_readl(padctl, XUSB_PADCTL_UPHY_PLL_S0_CTL2);
	value &= ~(XUSB_PADCTL_UPHY_PLL_CTL2_CAL_CTRL_MASK <<
		   XUSB_PADCTL_UPHY_PLL_CTL2_CAL_CTRL_SHIFT);
	value |= XUSB_PADCTL_UPHY_PLL_CTL2_CAL_CTRL_VAL <<
		 XUSB_PADCTL_UPHY_PLL_CTL2_CAL_CTRL_SHIFT;
	padctl_writel(padctl, value, XUSB_PADCTL_UPHY_PLL_S0_CTL2);

	value = padctl_readl(padctl, XUSB_PADCTL_UPHY_PLL_S0_CTL5);
	value &= ~(XUSB_PADCTL_UPHY_PLL_CTL5_DCO_CTRL_MASK <<
		   XUSB_PADCTL_UPHY_PLL_CTL5_DCO_CTRL_SHIFT);
	value |= XUSB_PADCTL_UPHY_PLL_CTL5_DCO_CTRL_VAL <<
		 XUSB_PADCTL_UPHY_PLL_CTL5_DCO_CTRL_SHIFT;
	padctl_writel(padctl, value, XUSB_PADCTL_UPHY_PLL_S0_CTL5);

	value = padctl_readl(padctl, XUSB_PADCTL_UPHY_PLL_S0_CTL1);
	value |= XUSB_PADCTL_UPHY_PLL_CTL1_PWR_OVRD;
	padctl_writel(padctl, value, XUSB_PADCTL_UPHY_PLL_S0_CTL1);

	value = padctl_readl(padctl, XUSB_PADCTL_UPHY_PLL_S0_CTL2);
	value |= XUSB_PADCTL_UPHY_PLL_CTL2_CAL_OVRD;
	padctl_writel(padctl, value, XUSB_PADCTL_UPHY_PLL_S0_CTL2);

	value = padctl_readl(padctl, XUSB_PADCTL_UPHY_PLL_S0_CTL8);
	value |= XUSB_PADCTL_UPHY_PLL_CTL8_RCAL_OVRD;
	padctl_writel(padctl, value, XUSB_PADCTL_UPHY_PLL_S0_CTL8);

	value = padctl_readl(padctl, XUSB_PADCTL_UPHY_PLL_S0_CTL4);
	value &= ~((XUSB_PADCTL_UPHY_PLL_CTL4_TXCLKREF_SEL_MASK <<
		    XUSB_PADCTL_UPHY_PLL_CTL4_TXCLKREF_SEL_SHIFT) |
		   (XUSB_PADCTL_UPHY_PLL_CTL4_REFCLK_SEL_MASK <<
		    XUSB_PADCTL_UPHY_PLL_CTL4_REFCLK_SEL_SHIFT));
	value |= XUSB_PADCTL_UPHY_PLL_CTL4_TXCLKREF_EN;
	if (usb) {
		value |= (XUSB_PADCTL_UPHY_PLL_CTL4_TXCLKREF_SEL_USB_VAL <<
			  XUSB_PADCTL_UPHY_PLL_CTL4_TXCLKREF_SEL_SHIFT);
	} else {
		value |= (XUSB_PADCTL_UPHY_PLL_CTL4_TXCLKREF_SEL_SATA_VAL <<
			  XUSB_PADCTL_UPHY_PLL_CTL4_TXCLKREF_SEL_SHIFT);
	}
	padctl_writel(padctl, value, XUSB_PADCTL_UPHY_PLL_S0_CTL4);

	value = padctl_readl(padctl, XUSB_PADCTL_UPHY_PLL_S0_CTL1);
	value &= ~((XUSB_PADCTL_UPHY_PLL_CTL1_FREQ_MDIV_MASK <<
		    XUSB_PADCTL_UPHY_PLL_CTL1_FREQ_MDIV_SHIFT) |
		   (XUSB_PADCTL_UPHY_PLL_CTL1_FREQ_NDIV_MASK <<
		    XUSB_PADCTL_UPHY_PLL_CTL1_FREQ_NDIV_SHIFT));
	if (usb) {
		value |= XUSB_PADCTL_UPHY_PLL_CTL1_FREQ_NDIV_USB_VAL <<
			 XUSB_PADCTL_UPHY_PLL_CTL1_FREQ_NDIV_SHIFT;
	} else {
		value |= XUSB_PADCTL_UPHY_PLL_CTL1_FREQ_NDIV_SATA_VAL <<
			 XUSB_PADCTL_UPHY_PLL_CTL1_FREQ_NDIV_SHIFT;
	}
	padctl_writel(padctl, value, XUSB_PADCTL_UPHY_PLL_S0_CTL1);

	value = padctl_readl(padctl, XUSB_PADCTL_UPHY_PLL_S0_CTL1);
	value &= ~XUSB_PADCTL_UPHY_PLL_CTL1_IDDQ;
	padctl_writel(padctl, value, XUSB_PADCTL_UPHY_PLL_S0_CTL1);

	value = padctl_readl(padctl, XUSB_PADCTL_UPHY_PLL_S0_CTL1);
	value &= ~(XUSB_PADCTL_UPHY_PLL_CTL1_SLEEP_MASK <<
		   XUSB_PADCTL_UPHY_PLL_CTL1_SLEEP_SHIFT);
	padctl_writel(padctl, value, XUSB_PADCTL_UPHY_PLL_S0_CTL1);

	usleep_range(10, 20);

	value = padctl_readl(padctl, XUSB_PADCTL_UPHY_PLL_S0_CTL4);
	value |= XUSB_PADCTL_UPHY_PLL_CTL4_REFCLKBUF_EN;
	padctl_writel(padctl, value, XUSB_PADCTL_UPHY_PLL_S0_CTL4);

	value = padctl_readl(padctl, XUSB_PADCTL_UPHY_PLL_S0_CTL2);
	value |= XUSB_PADCTL_UPHY_PLL_CTL2_CAL_EN;
	padctl_writel(padctl, value, XUSB_PADCTL_UPHY_PLL_S0_CTL2);

	timeout = jiffies + msecs_to_jiffies(100);

	while (time_before(jiffies, timeout)) {
		value = padctl_readl(padctl, XUSB_PADCTL_UPHY_PLL_S0_CTL2);
		if (value & XUSB_PADCTL_UPHY_PLL_CTL2_CAL_DONE)
			break;

		usleep_range(10, 20);
	}

	value = padctl_readl(padctl, XUSB_PADCTL_UPHY_PLL_S0_CTL2);
	if (!(value & XUSB_PADCTL_UPHY_PLL_CTL2_CAL_DONE)) {
		err = -ETIMEDOUT;
		goto reset;
	}

	value = padctl_readl(padctl, XUSB_PADCTL_UPHY_PLL_S0_CTL2);
	value &= ~XUSB_PADCTL_UPHY_PLL_CTL2_CAL_EN;
	padctl_writel(padctl, value, XUSB_PADCTL_UPHY_PLL_S0_CTL2);

	timeout = jiffies + msecs_to_jiffies(100);

	while (time_before(jiffies, timeout)) {
		value = padctl_readl(padctl, XUSB_PADCTL_UPHY_PLL_S0_CTL2);
		if (!(value & XUSB_PADCTL_UPHY_PLL_CTL2_CAL_DONE))
			break;

		usleep_range(10, 20);
	}

	value = padctl_readl(padctl, XUSB_PADCTL_UPHY_PLL_S0_CTL2);
	if (value & XUSB_PADCTL_UPHY_PLL_CTL2_CAL_DONE) {
		err = -ETIMEDOUT;
		goto reset;
	}

	value = padctl_readl(padctl, XUSB_PADCTL_UPHY_PLL_S0_CTL1);
	value |= XUSB_PADCTL_UPHY_PLL_CTL1_ENABLE;
	padctl_writel(padctl, value, XUSB_PADCTL_UPHY_PLL_S0_CTL1);

	timeout = jiffies + msecs_to_jiffies(100);

	while (time_before(jiffies, timeout)) {
		value = padctl_readl(padctl, XUSB_PADCTL_UPHY_PLL_S0_CTL1);
		if (value & XUSB_PADCTL_UPHY_PLL_CTL1_LOCKDET_STATUS)
			break;

		usleep_range(10, 20);
	}

	value = padctl_readl(padctl, XUSB_PADCTL_UPHY_PLL_S0_CTL1);
	if (!(value & XUSB_PADCTL_UPHY_PLL_CTL1_LOCKDET_STATUS)) {
		err = -ETIMEDOUT;
		goto reset;
	}

	value = padctl_readl(padctl, XUSB_PADCTL_UPHY_PLL_S0_CTL8);
	value |= XUSB_PADCTL_UPHY_PLL_CTL8_RCAL_EN |
		 XUSB_PADCTL_UPHY_PLL_CTL8_RCAL_CLK_EN;
	padctl_writel(padctl, value, XUSB_PADCTL_UPHY_PLL_S0_CTL8);

	timeout = jiffies + msecs_to_jiffies(100);

	while (time_before(jiffies, timeout)) {
		value = padctl_readl(padctl, XUSB_PADCTL_UPHY_PLL_S0_CTL8);
		if (value & XUSB_PADCTL_UPHY_PLL_CTL8_RCAL_DONE)
			break;

		usleep_range(10, 20);
	}

	value = padctl_readl(padctl, XUSB_PADCTL_UPHY_PLL_S0_CTL8);
	if (!(value & XUSB_PADCTL_UPHY_PLL_CTL8_RCAL_DONE)) {
		err = -ETIMEDOUT;
		goto reset;
	}

	value = padctl_readl(padctl, XUSB_PADCTL_UPHY_PLL_S0_CTL8);
	value &= ~XUSB_PADCTL_UPHY_PLL_CTL8_RCAL_EN;
	padctl_writel(padctl, value, XUSB_PADCTL_UPHY_PLL_S0_CTL8);

	timeout = jiffies + msecs_to_jiffies(100);

	while (time_before(jiffies, timeout)) {
		value = padctl_readl(padctl, XUSB_PADCTL_UPHY_PLL_S0_CTL8);
		if (!(value & XUSB_PADCTL_UPHY_PLL_CTL8_RCAL_DONE))
			break;

		usleep_range(10, 20);
	}

	value = padctl_readl(padctl, XUSB_PADCTL_UPHY_PLL_S0_CTL8);
	if (value & XUSB_PADCTL_UPHY_PLL_CTL8_RCAL_DONE) {
		err = -ETIMEDOUT;
		goto reset;
	}

	value = padctl_readl(padctl, XUSB_PADCTL_UPHY_PLL_S0_CTL8);
	value &= ~XUSB_PADCTL_UPHY_PLL_CTL8_RCAL_CLK_EN;
	padctl_writel(padctl, value, XUSB_PADCTL_UPHY_PLL_S0_CTL8);

	tegra210_sata_pll_hw_control_enable();

	value = padctl_readl(padctl, XUSB_PADCTL_UPHY_PLL_S0_CTL1);
	value &= ~XUSB_PADCTL_UPHY_PLL_CTL1_PWR_OVRD;
	padctl_writel(padctl, value, XUSB_PADCTL_UPHY_PLL_S0_CTL1);

	value = padctl_readl(padctl, XUSB_PADCTL_UPHY_PLL_S0_CTL2);
	value &= ~XUSB_PADCTL_UPHY_PLL_CTL2_CAL_OVRD;
	padctl_writel(padctl, value, XUSB_PADCTL_UPHY_PLL_S0_CTL2);

	value = padctl_readl(padctl, XUSB_PADCTL_UPHY_PLL_S0_CTL8);
	value &= ~XUSB_PADCTL_UPHY_PLL_CTL8_RCAL_OVRD;
	padctl_writel(padctl, value, XUSB_PADCTL_UPHY_PLL_S0_CTL8);

	usleep_range(10, 20);

	tegra210_sata_pll_hw_sequence_start();

	priv->sata_enable++;
	mutex_unlock(&padctl->lock);

	return 0;
reset:
	reset_control_assert(priv->sata_rst);
	clk_disable_unprepare(priv->pll_e);
unlock:
	mutex_unlock(&padctl->lock);
	return err;
}

static void tegra210_sata_uphy_disable(struct tegra_xusb_padctl *padctl)
{
	struct tegra210_xusb_padctl *priv = padctl->priv;

	mutex_lock(&padctl->lock);

	if (WARN_ON(priv->sata_enable == 0))
		goto out;

	if (--priv->sata_enable > 0)
		goto out;

	reset_control_assert(priv->sata_rst);
out:
	mutex_unlock(&padctl->lock);
}

static int tegra210_xusb_padctl_enable(struct tegra_xusb_padctl *padctl)
{
	u32 value;

	mutex_lock(&padctl->lock);

	if (padctl->enable++ > 0)
		goto out;

	value = padctl_readl(padctl, XUSB_PADCTL_ELPG_PROGRAM1);
	value &= ~XUSB_PADCTL_ELPG_PROGRAM1_AUX_MUX_LP0_CLAMP_EN;
	padctl_writel(padctl, value, XUSB_PADCTL_ELPG_PROGRAM1);

	usleep_range(100, 200);

	value = padctl_readl(padctl, XUSB_PADCTL_ELPG_PROGRAM1);
	value &= ~XUSB_PADCTL_ELPG_PROGRAM1_AUX_MUX_LP0_CLAMP_EN_EARLY;
	padctl_writel(padctl, value, XUSB_PADCTL_ELPG_PROGRAM1);

	usleep_range(100, 200);

	value = padctl_readl(padctl, XUSB_PADCTL_ELPG_PROGRAM1);
	value &= ~XUSB_PADCTL_ELPG_PROGRAM1_AUX_MUX_LP0_VCORE_DOWN;
	padctl_writel(padctl, value, XUSB_PADCTL_ELPG_PROGRAM1);

out:
	mutex_unlock(&padctl->lock);
	return 0;
}

static int tegra210_xusb_padctl_disable(struct tegra_xusb_padctl *padctl)
{
	u32 value;

	mutex_lock(&padctl->lock);

	if (WARN_ON(padctl->enable == 0))
		goto out;

	if (--padctl->enable > 0)
		goto out;

	value = padctl_readl(padctl, XUSB_PADCTL_ELPG_PROGRAM1);
	value |= XUSB_PADCTL_ELPG_PROGRAM1_AUX_MUX_LP0_VCORE_DOWN;
	padctl_writel(padctl, value, XUSB_PADCTL_ELPG_PROGRAM1);

	usleep_range(100, 200);

	value = padctl_readl(padctl, XUSB_PADCTL_ELPG_PROGRAM1);
	value |= XUSB_PADCTL_ELPG_PROGRAM1_AUX_MUX_LP0_CLAMP_EN_EARLY;
	padctl_writel(padctl, value, XUSB_PADCTL_ELPG_PROGRAM1);

	usleep_range(100, 200);

	value = padctl_readl(padctl, XUSB_PADCTL_ELPG_PROGRAM1);
	value |= XUSB_PADCTL_ELPG_PROGRAM1_AUX_MUX_LP0_CLAMP_EN;
	padctl_writel(padctl, value, XUSB_PADCTL_ELPG_PROGRAM1);

out:
	mutex_unlock(&padctl->lock);
	return 0;
}

static int tegra210_xusb_phy_init(struct phy *phy)
{
	struct tegra_xusb_padctl *padctl = phy_get_drvdata(phy);

	return tegra210_xusb_padctl_enable(padctl);
}

static int tegra210_xusb_phy_exit(struct phy *phy)
{
	struct tegra_xusb_padctl *padctl = phy_get_drvdata(phy);

	return tegra210_xusb_padctl_disable(padctl);
}

static int tegra210_pcie_phy_power_on(struct phy *phy)
{
	struct tegra_xusb_padctl *padctl = phy_get_drvdata(phy);

	return tegra210_pex_uphy_enable(padctl);
}

static int tegra210_pcie_phy_power_off(struct phy *phy)
{
	struct tegra_xusb_padctl *padctl = phy_get_drvdata(phy);

	tegra210_pex_uphy_disable(padctl);

	return 0;
}

static const struct phy_ops tegra210_pcie_phy_ops = {
	.init = tegra210_xusb_phy_init,
	.exit = tegra210_xusb_phy_exit,
	.power_on = tegra210_pcie_phy_power_on,
	.power_off = tegra210_pcie_phy_power_off,
	.owner = THIS_MODULE,
};

static int tegra210_sata_phy_power_on(struct phy *phy)
{
	struct tegra_xusb_padctl *padctl = phy_get_drvdata(phy);

	return tegra210_sata_uphy_enable(padctl, false);
}

static int tegra210_sata_phy_power_off(struct phy *phy)
{
	struct tegra_xusb_padctl *padctl = phy_get_drvdata(phy);

	tegra210_sata_uphy_disable(padctl);

	return 0;
}

static const struct phy_ops tegra210_sata_phy_ops = {
	.init = tegra210_xusb_phy_init,
	.exit = tegra210_xusb_phy_exit,
	.power_on = tegra210_sata_phy_power_on,
	.power_off = tegra210_sata_phy_power_off,
	.owner = THIS_MODULE,
};

static int tegra210_usb3_phy_init(struct phy *phy)
{
	struct tegra_xusb_usb3_phy *usb = phy_get_drvdata(phy);
	struct tegra_xusb_padctl *padctl = usb->padctl;
	u32 value;
	int err;

	err = tegra210_xusb_padctl_enable(padctl);
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

static int tegra210_usb3_phy_exit(struct phy *phy)
{
	struct tegra_xusb_usb3_phy *usb = phy_get_drvdata(phy);
	struct tegra_xusb_padctl *padctl = usb->padctl;
	u32 value;

	value = padctl_readl(padctl, XUSB_PADCTL_SS_PORT_MAP);
	value |= XUSB_PADCTL_SS_PORT_MAP_PORT_DISABLED <<
		 XUSB_PADCTL_SS_PORT_MAP_PORTX_SHIFT(usb->index);
	padctl_writel(padctl, value, XUSB_PADCTL_SS_PORT_MAP);

	return tegra210_xusb_padctl_disable(padctl);
}

static int tegra210_usb3_phy_power_on(struct phy *phy)
{
	struct tegra_xusb_usb3_phy *usb = phy_get_drvdata(phy);
	struct tegra_xusb_padctl *padctl = usb->padctl;
	unsigned int port = usb->index;
	u32 value;
	int err;

	value = padctl_readl(padctl, XUSB_PADCTL_UPHY_USB3_PADX_ECTL1(port));
	value &= ~(XUSB_PADCTL_UPHY_USB3_PAD_ECTL1_TX_TERM_CTRL_MASK <<
		   XUSB_PADCTL_UPHY_USB3_PAD_ECTL1_TX_TERM_CTRL_SHIFT);
	value |= XUSB_PADCTL_UPHY_USB3_PAD_ECTL1_TX_TERM_CTRL_VAL <<
		 XUSB_PADCTL_UPHY_USB3_PAD_ECTL1_TX_TERM_CTRL_SHIFT;
	padctl_writel(padctl, value, XUSB_PADCTL_UPHY_USB3_PADX_ECTL1(port));

	value = padctl_readl(padctl, XUSB_PADCTL_UPHY_USB3_PADX_ECTL2(port));
	value &= ~(XUSB_PADCTL_UPHY_USB3_PAD_ECTL2_RX_CTLE_MASK <<
		   XUSB_PADCTL_UPHY_USB3_PAD_ECTL2_RX_CTLE_SHIFT);
	value |= XUSB_PADCTL_UPHY_USB3_PAD_ECTL2_RX_CTLE_VAL <<
		 XUSB_PADCTL_UPHY_USB3_PAD_ECTL2_RX_CTLE_SHIFT;
	padctl_writel(padctl, value, XUSB_PADCTL_UPHY_USB3_PADX_ECTL2(port));

	padctl_writel(padctl, XUSB_PADCTL_UPHY_USB3_PAD_ECTL3_RX_DFE_VAL,
		      XUSB_PADCTL_UPHY_USB3_PADX_ECTL3(port));

	value = padctl_readl(padctl, XUSB_PADCTL_UPHY_USB3_PADX_ECTL4(port));
	value &= ~(XUSB_PADCTL_UPHY_USB3_PAD_ECTL4_RX_CDR_CTRL_MASK <<
		   XUSB_PADCTL_UPHY_USB3_PAD_ECTL4_RX_CDR_CTRL_SHIFT);
	value |= XUSB_PADCTL_UPHY_USB3_PAD_ECTL4_RX_CDR_CTRL_VAL <<
		 XUSB_PADCTL_UPHY_USB3_PAD_ECTL4_RX_CDR_CTRL_SHIFT;
	padctl_writel(padctl, value, XUSB_PADCTL_UPHY_USB3_PADX_ECTL4(port));

	padctl_writel(padctl, XUSB_PADCTL_UPHY_USB3_PAD_ECTL6_RX_EQ_CTRL_H_VAL,
		      XUSB_PADCTL_UPHY_USB3_PADX_ECTL6(port));

	if (usb->lane == PIN_SATA_0)
		err = tegra210_sata_uphy_enable(padctl, true);
	else
		err = tegra210_pex_uphy_enable(padctl);
	if (err) {
		dev_err(&phy->dev, "%s: uphy enabling failed: %d\n", __func__,
			err);
		return err;
	}

	value = padctl_readl(padctl, XUSB_PADCTL_ELPG_PROGRAM1);
	value &= ~XUSB_PADCTL_ELPG_PROGRAM1_SSPX_ELPG_VCORE_DOWN(port);
	padctl_writel(padctl, value, XUSB_PADCTL_ELPG_PROGRAM1);

	usleep_range(100, 200);

	value = padctl_readl(padctl, XUSB_PADCTL_ELPG_PROGRAM1);
	value &= ~XUSB_PADCTL_ELPG_PROGRAM1_SSPX_ELPG_CLAMP_EN_EARLY(port);
	padctl_writel(padctl, value, XUSB_PADCTL_ELPG_PROGRAM1);

	usleep_range(100, 200);

	value = padctl_readl(padctl, XUSB_PADCTL_ELPG_PROGRAM1);
	value &= ~XUSB_PADCTL_ELPG_PROGRAM1_SSPX_ELPG_CLAMP_EN(port);
	padctl_writel(padctl, value, XUSB_PADCTL_ELPG_PROGRAM1);

	return 0;
}

static int tegra210_usb3_phy_power_off(struct phy *phy)
{
	struct tegra_xusb_usb3_phy *usb = phy_get_drvdata(phy);
	struct tegra_xusb_padctl *padctl = usb->padctl;
	unsigned int port = usb->index;
	u32 value;

	value = padctl_readl(padctl, XUSB_PADCTL_ELPG_PROGRAM1);
	value |= XUSB_PADCTL_ELPG_PROGRAM1_SSPX_ELPG_CLAMP_EN_EARLY(port);
	padctl_writel(padctl, value, XUSB_PADCTL_ELPG_PROGRAM1);

	usleep_range(100, 200);

	value = padctl_readl(padctl, XUSB_PADCTL_ELPG_PROGRAM1);
	value |= XUSB_PADCTL_ELPG_PROGRAM1_SSPX_ELPG_CLAMP_EN(port);
	padctl_writel(padctl, value, XUSB_PADCTL_ELPG_PROGRAM1);

	usleep_range(250, 350);

	value = padctl_readl(padctl, XUSB_PADCTL_ELPG_PROGRAM1);
	value |= XUSB_PADCTL_ELPG_PROGRAM1_SSPX_ELPG_VCORE_DOWN(port);
	padctl_writel(padctl, value, XUSB_PADCTL_ELPG_PROGRAM1);

	if (usb->lane == PIN_SATA_0)
		tegra210_sata_uphy_disable(padctl);
	else
		tegra210_pex_uphy_disable(padctl);

	return 0;
}

static void
tegra210_usb3_phy_set_lfps_detector(struct tegra_xusb_padctl *padctl,
				    unsigned int port, bool enable)
{
	struct tegra_xusb_usb3_phy *usb;
	u32 value, offset;

	usb = &padctl->usb3_phys[port];
	if (!lane_is_pcie_or_sata(usb->lane))
		return;

	if (usb->lane == PIN_SATA_0)
		offset = XUSB_PADCTL_UPHY_MISC_PAD_S0_CTL1;
	else
		offset = XUSB_PADCTL_UPHY_MISC_PAD_PX_CTL1(usb->lane -
							   PIN_PCIE_0);

	value = padctl_readl(padctl, offset);
	value &= ~((XUSB_PADCTL_UPHY_MISC_PAD_CTL1_AUX_RX_IDLE_MODE_MASK <<
		    XUSB_PADCTL_UPHY_MISC_PAD_CTL1_AUX_RX_IDLE_MODE_SHIFT) |
		   XUSB_PADCTL_UPHY_MISC_PAD_CTL1_AUX_RX_TERM_EN |
		   XUSB_PADCTL_UPHY_MISC_PAD_CTL1_AUX_RX_MODE_OVRD);
	if (!enable) {
		value |= (XUSB_PADCTL_UPHY_MISC_PAD_CTL1_AUX_RX_IDLE_MODE_VAL <<
			  XUSB_PADCTL_UPHY_MISC_PAD_CTL1_AUX_RX_IDLE_MODE_SHIFT) |
			 XUSB_PADCTL_UPHY_MISC_PAD_CTL1_AUX_RX_TERM_EN |
			 XUSB_PADCTL_UPHY_MISC_PAD_CTL1_AUX_RX_MODE_OVRD;
	}
	padctl_writel(padctl, value, offset);
}

static void tegra210_usb3_phy_set_wake(struct tegra_xusb_usb3_phy *usb3,
				       bool enable)
{
	struct tegra_xusb_padctl *padctl = usb3->padctl;
	unsigned int index = usb3->index;
	u32 value;

	value = padctl_readl(padctl, XUSB_PADCTL_ELPG_PROGRAM0);
	value |= XUSB_PADCTL_ELPG_PROGRAM0_SS_PORTX_WAKEUP_EVENT(index);
	padctl_writel(padctl, value, XUSB_PADCTL_ELPG_PROGRAM0);

	usleep_range(10, 20);

	value = padctl_readl(padctl, XUSB_PADCTL_ELPG_PROGRAM0);
	if (enable)
		value |= XUSB_PADCTL_ELPG_PROGRAM0_SS_PORTX_WAKE_ENABLE(index);
	else
		value &= ~XUSB_PADCTL_ELPG_PROGRAM0_SS_PORTX_WAKE_ENABLE(index);
	padctl_writel(padctl, value, XUSB_PADCTL_ELPG_PROGRAM0);
}

static const struct phy_ops tegra210_usb3_phy_ops = {
	.init = tegra210_usb3_phy_init,
	.exit = tegra210_usb3_phy_exit,
	.power_on = tegra210_usb3_phy_power_on,
	.power_off = tegra210_usb3_phy_power_off,
	.owner = THIS_MODULE,
};

static int tegra210_utmi_phy_init(struct phy *phy)
{
	struct tegra_xusb_utmi_phy *utmi = phy_get_drvdata(phy);
	struct tegra_xusb_padctl *padctl = utmi->padctl;
	u32 value;

	value = padctl_readl(padctl, XUSB_PADCTL_USB2_PAD_MUX);
	value &= ~(XUSB_PADCTL_USB2_PAD_MUX_USB2_BIAS_PAD_MASK <<
		   XUSB_PADCTL_USB2_PAD_MUX_USB2_BIAS_PAD_SHIFT);
	value |= XUSB_PADCTL_USB2_PAD_MUX_USB2_BIAS_PAD_XUSB <<
		 XUSB_PADCTL_USB2_PAD_MUX_USB2_BIAS_PAD_SHIFT;
	padctl_writel(padctl, value, XUSB_PADCTL_USB2_PAD_MUX);

	return tegra210_xusb_padctl_enable(utmi->padctl);
}

static int tegra210_utmi_phy_exit(struct phy *phy)
{
	struct tegra_xusb_utmi_phy *utmi = phy_get_drvdata(phy);

	return tegra210_xusb_padctl_disable(utmi->padctl);
}

static int tegra210_utmi_phy_power_on(struct phy *phy)
{
	struct tegra_xusb_utmi_phy *utmi = phy_get_drvdata(phy);
	struct tegra_xusb_padctl *padctl = utmi->padctl;
	struct tegra210_xusb_padctl *priv = padctl->priv;
	unsigned int port = utmi->index;
	u32 value;
	int err;

	value = padctl_readl(padctl, XUSB_PADCTL_USB2_BIAS_PAD_CTL0);
	value &= ~((XUSB_PADCTL_USB2_BIAS_PAD_CTL0_HS_SQUELCH_LEVEL_MASK <<
		    XUSB_PADCTL_USB2_BIAS_PAD_CTL0_HS_SQUELCH_LEVEL_SHIFT) |
		   (XUSB_PADCTL_USB2_BIAS_PAD_CTL0_HS_DISCON_LEVEL_MASK <<
		    XUSB_PADCTL_USB2_BIAS_PAD_CTL0_HS_DISCON_LEVEL_SHIFT));
	value |= (XUSB_PADCTL_USB2_BIAS_PAD_CTL0_HS_DISCON_LEVEL_VAL <<
		  XUSB_PADCTL_USB2_BIAS_PAD_CTL0_HS_DISCON_LEVEL_SHIFT);
	if (tegra_sku_info.revision < TEGRA_REVISION_A02)
		value |=
			(XUSB_PADCTL_USB2_BIAS_PAD_CTL0_HS_SQUELCH_LEVEL_VAL <<
			XUSB_PADCTL_USB2_BIAS_PAD_CTL0_HS_SQUELCH_LEVEL_SHIFT);
	padctl_writel(padctl, value, XUSB_PADCTL_USB2_BIAS_PAD_CTL0);

	value = padctl_readl(padctl, XUSB_PADCTL_USB2_PORT_CAP);
	value &= ~(XUSB_PADCTL_USB2_PORT_CAP_PORT_CAP_MASK <<
		   XUSB_PADCTL_USB2_PORT_CAP_PORTX_CAP_SHIFT(port));
	switch (utmi->dr_mode) {
	case USB_DR_MODE_PERIPHERAL:
		value |= XUSB_PADCTL_USB2_PORT_CAP_DEVICE <<
			 XUSB_PADCTL_USB2_PORT_CAP_PORTX_CAP_SHIFT(port);
		break;
	case USB_DR_MODE_OTG:
		value |= XUSB_PADCTL_USB2_PORT_CAP_OTG <<
			 XUSB_PADCTL_USB2_PORT_CAP_PORTX_CAP_SHIFT(port);
		break;
	case USB_DR_MODE_HOST:
	default:
		value |= XUSB_PADCTL_USB2_PORT_CAP_HOST <<
			 XUSB_PADCTL_USB2_PORT_CAP_PORTX_CAP_SHIFT(port);
		break;
	}
	padctl_writel(padctl, value, XUSB_PADCTL_USB2_PORT_CAP);

	value = padctl_readl(padctl, XUSB_PADCTL_USB2_OTG_PADX_CTL0(port));
	value &= ~((XUSB_PADCTL_USB2_OTG_PAD_CTL0_HS_CURR_LEVEL_MASK <<
		    XUSB_PADCTL_USB2_OTG_PAD_CTL0_HS_CURR_LEVEL_SHIFT) |
		   XUSB_PADCTL_USB2_OTG_PAD_CTL0_PD |
		   XUSB_PADCTL_USB2_OTG_PAD_CTL0_PD2 |
		   XUSB_PADCTL_USB2_OTG_PAD_CTL0_PD_ZI);
	value |= (((u32) clamp_val((s32) priv->fuse.hs_curr_level[port] +
		   utmi->hs_curr_level_offset, 0, 0x3f)) <<
		  XUSB_PADCTL_USB2_OTG_PAD_CTL0_HS_CURR_LEVEL_SHIFT);
	padctl_writel(padctl, value, XUSB_PADCTL_USB2_OTG_PADX_CTL0(port));

	value = padctl_readl(padctl, XUSB_PADCTL_USB2_OTG_PADX_CTL1(port));
	value &= ~((XUSB_PADCTL_USB2_OTG_PAD_CTL1_TERM_RANGE_ADJ_MASK <<
		    XUSB_PADCTL_USB2_OTG_PAD_CTL1_TERM_RANGE_ADJ_SHIFT) |
		   (XUSB_PADCTL_USB2_OTG_PAD_CTL1_RPD_CTRL_MASK <<
		    XUSB_PADCTL_USB2_OTG_PAD_CTL1_RPD_CTRL_SHIFT) |
		   XUSB_PADCTL_USB2_OTG_PAD_CTL1_PD_DR |
		   XUSB_PADCTL_USB2_OTG_PAD_CTL1_PD_CHRP_OVRD |
		   XUSB_PADCTL_USB2_OTG_PAD_CTL1_PD_DISC_OVRD);
	value |= (priv->fuse.hs_term_range_adj <<
		  XUSB_PADCTL_USB2_OTG_PAD_CTL1_TERM_RANGE_ADJ_SHIFT) |
		 (priv->fuse.rpd_ctrl <<
		  XUSB_PADCTL_USB2_OTG_PAD_CTL1_RPD_CTRL_SHIFT);
	padctl_writel(padctl, value, XUSB_PADCTL_USB2_OTG_PADX_CTL1(port));

	value = padctl_readl(padctl,
			     XUSB_PADCTL_USB2_BATTERY_CHRG_OTGPADX_CTL1(port));
	value &= ~(XUSB_PADCTL_USB2_BATTERY_CHRG_OTGPAD_CTL1_VREG_LEV_MASK <<
		   XUSB_PADCTL_USB2_BATTERY_CHRG_OTGPAD_CTL1_VREG_LEV_SHIFT);
	value |= XUSB_PADCTL_USB2_BATTERY_CHRG_OTGPAD_CTL1_VREG_FIX18;
	padctl_writel(padctl, value,
		      XUSB_PADCTL_USB2_BATTERY_CHRG_OTGPADX_CTL1(port));

	err = regulator_enable(utmi->supply);
	if (err)
		return err;

	mutex_lock(&padctl->lock);

	if (padctl->utmi_enable > 0) {
		padctl->utmi_enable++;
		mutex_unlock(&padctl->lock);
		return 0;
	}

	err = clk_prepare_enable(priv->usb2_trk);
	if (err)
		goto disable_regulator;

	value = padctl_readl(padctl, XUSB_PADCTL_USB2_BIAS_PAD_CTL1);
	value &= ~((XUSB_PADCTL_USB2_BIAS_PAD_CTL1_TRK_START_TIMER_MASK <<
		    XUSB_PADCTL_USB2_BIAS_PAD_CTL1_TRK_START_TIMER_SHIFT) |
		   (XUSB_PADCTL_USB2_BIAS_PAD_CTL1_TRK_DONE_RESET_TIMER_MASK <<
		    XUSB_PADCTL_USB2_BIAS_PAD_CTL1_TRK_DONE_RESET_TIMER_SHIFT));
	value |= (XUSB_PADCTL_USB2_BIAS_PAD_CTL1_TRK_START_TIMER_VAL <<
		  XUSB_PADCTL_USB2_BIAS_PAD_CTL1_TRK_START_TIMER_SHIFT) |
		 (XUSB_PADCTL_USB2_BIAS_PAD_CTL1_TRK_DONE_RESET_TIMER_VAL <<
		  XUSB_PADCTL_USB2_BIAS_PAD_CTL1_TRK_DONE_RESET_TIMER_SHIFT);
	padctl_writel(padctl, value, XUSB_PADCTL_USB2_BIAS_PAD_CTL1);

	value = padctl_readl(padctl, XUSB_PADCTL_USB2_BIAS_PAD_CTL0);
	value &= ~XUSB_PADCTL_USB2_BIAS_PAD_CTL0_PD;
	padctl_writel(padctl, value, XUSB_PADCTL_USB2_BIAS_PAD_CTL0);

	udelay(1);

	value = padctl_readl(padctl, XUSB_PADCTL_USB2_BIAS_PAD_CTL1);
	value &= ~XUSB_PADCTL_USB2_BIAS_PAD_CTL1_PD_TRK;
	padctl_writel(padctl, value, XUSB_PADCTL_USB2_BIAS_PAD_CTL1);

	udelay(50);

	clk_disable_unprepare(priv->usb2_trk);

	padctl->utmi_enable++;
	mutex_unlock(&padctl->lock);

	return 0;
disable_regulator:
	regulator_disable(utmi->supply);
	mutex_unlock(&padctl->lock);
	return err;
}

static int tegra210_utmi_phy_power_off(struct phy *phy)
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

static void tegra210_utmi_phy_set_wake(struct tegra_xusb_utmi_phy *utmi,
				       bool enable)
{
	struct tegra_xusb_padctl *padctl = utmi->padctl;
	unsigned int index = utmi->index;
	u32 value;

	value = padctl_readl(padctl, XUSB_PADCTL_ELPG_PROGRAM0);
	value |= XUSB_PADCTL_ELPG_PROGRAM0_USB2_PORTX_WAKEUP_EVENT(index);
	padctl_writel(padctl, value, XUSB_PADCTL_ELPG_PROGRAM0);

	usleep_range(10, 20);

	value = padctl_readl(padctl, XUSB_PADCTL_ELPG_PROGRAM0);
	if (enable)
		value |= XUSB_PADCTL_ELPG_PROGRAM0_USB2_PORTX_WAKE_ENABLE(index);
	else
		value &= ~XUSB_PADCTL_ELPG_PROGRAM0_USB2_PORTX_WAKE_ENABLE(index);
	padctl_writel(padctl, value, XUSB_PADCTL_ELPG_PROGRAM0);
}

static void
tegra210_utmi_phy_get_pad_config(struct tegra_xusb_utmi_phy *utmi,
				 struct tegra_utmi_pad_config *config)
{
	struct tegra_xusb_padctl *padctl = utmi->padctl;
	unsigned int index = utmi->index;
	u32 value;

	value = padctl_readl(padctl, XUSB_PADCTL_USB2_BIAS_PAD_CTL1);
	config->tctrl = (value & XUSB_PADCTL_USB2_BIAS_PAD_CTL1_TCTRL_MASK) >>
		XUSB_PADCTL_USB2_BIAS_PAD_CTL1_TCTRL_SHIFT;
	config->pctrl = (value & XUSB_PADCTL_USB2_BIAS_PAD_CTL1_PCTRL_MASK) >>
		XUSB_PADCTL_USB2_BIAS_PAD_CTL1_PCTRL_SHIFT;
	value = padctl_readl(padctl, XUSB_PADCTL_USB2_OTG_PADX_CTL1(index));
	config->rpd_ctrl = (value & XUSB_PADCTL_USB2_OTG_PAD_CTL1_RPD_CTRL_MASK) >>
		XUSB_PADCTL_USB2_OTG_PAD_CTL1_RPD_CTRL_SHIFT;
}

static void tegra210_utmi_pad_protect(struct tegra_xusb_utmi_phy *utmi,
				      bool enable)
{
	struct tegra_xusb_padctl *padctl = utmi->padctl;
	unsigned int port = utmi->index;
	u32 value;

	value = padctl_readl(padctl,
			     XUSB_PADCTL_USB2_BATTERY_CHRG_OTGPADX_CTL1(port));
	value &= ~(XUSB_PADCTL_USB2_BATTERY_CHRG_OTGPAD_CTL1_VREG_FIX18 |
		   (XUSB_PADCTL_USB2_BATTERY_CHRG_OTGPAD_CTL1_VREG_LEV_MASK <<
		    XUSB_PADCTL_USB2_BATTERY_CHRG_OTGPAD_CTL1_VREG_LEV_SHIFT));
	if (enable) {
		value |= XUSB_PADCTL_USB2_BATTERY_CHRG_OTGPAD_CTL1_VREG_LEV_EN <<
			 XUSB_PADCTL_USB2_BATTERY_CHRG_OTGPAD_CTL1_VREG_LEV_SHIFT;
	} else {
		value |= XUSB_PADCTL_USB2_BATTERY_CHRG_OTGPAD_CTL1_VREG_FIX18;
	}
	padctl_writel(padctl, value,
		      XUSB_PADCTL_USB2_BATTERY_CHRG_OTGPADX_CTL1(port));
}

static void tegra210_utmi_vbus_override(struct tegra_xusb_utmi_phy *utmi,
					bool enable)
{
	struct tegra_xusb_padctl *padctl = utmi->padctl;
	u32 value;

	value = padctl_readl(padctl, XUSB_PADCTL_USB2_VBUS_ID);
	value &= ~(XUSB_PADCTL_USB2_VBUS_ID_ID_OVERRIDE_MASK <<
		   XUSB_PADCTL_USB2_VBUS_ID_ID_OVERRIDE_SHIFT);
	if (enable) {
		/* Enable VBUS override  set ID override to floating. */
		value |= (XUSB_PADCTL_USB2_VBUS_ID_ID_OVERRIDE_FLOAT <<
			  XUSB_PADCTL_USB2_VBUS_ID_ID_OVERRIDE_SHIFT) |
			 XUSB_PADCTL_USB2_VBUS_ID_VBUS_OVERRIDE;

		tegra210_utmi_pad_protect(utmi, true);
	} else {
		/* Disable VBUS override, set ID override to grounded. */
		value &= ~XUSB_PADCTL_USB2_VBUS_ID_VBUS_OVERRIDE;
		value |= XUSB_PADCTL_USB2_VBUS_ID_ID_OVERRIDE_GND <<
			 XUSB_PADCTL_USB2_VBUS_ID_ID_OVERRIDE_SHIFT;

		tegra210_utmi_pad_protect(utmi, false);
	}
	padctl_writel(padctl, value, XUSB_PADCTL_USB2_VBUS_ID);
}

static const struct phy_ops tegra210_utmi_phy_ops = {
	.init = tegra210_utmi_phy_init,
	.exit = tegra210_utmi_phy_exit,
	.power_on = tegra210_utmi_phy_power_on,
	.power_off = tegra210_utmi_phy_power_off,
	.owner = THIS_MODULE,
};

static int tegra210_hsic_phy_init(struct phy *phy)
{
	struct tegra_xusb_hsic_phy *hsic = phy_get_drvdata(phy);
	struct tegra_xusb_padctl *padctl = hsic->padctl;
	u32 value;

	value = padctl_readl(padctl, XUSB_PADCTL_USB2_PAD_MUX);
	value &= ~(XUSB_PADCTL_USB2_PAD_MUX_HSIC_PAD_TRK_MASK <<
		   XUSB_PADCTL_USB2_PAD_MUX_HSIC_PAD_TRK_SHIFT);
	value |= XUSB_PADCTL_USB2_PAD_MUX_HSIC_PAD_TRK_XUSB <<
		 XUSB_PADCTL_USB2_PAD_MUX_HSIC_PAD_TRK_SHIFT;
	padctl_writel(padctl, value, XUSB_PADCTL_USB2_PAD_MUX);

	return tegra210_xusb_padctl_enable(hsic->padctl);
}

static int tegra210_hsic_phy_exit(struct phy *phy)
{
	struct tegra_xusb_hsic_phy *hsic = phy_get_drvdata(phy);

	return tegra210_xusb_padctl_disable(hsic->padctl);
}

static int tegra210_hsic_phy_power_on(struct phy *phy)
{
	struct tegra_xusb_hsic_phy *hsic = phy_get_drvdata(phy);
	struct tegra_xusb_padctl *padctl = hsic->padctl;
	struct tegra210_xusb_padctl *priv = padctl->priv;
	unsigned int port = hsic->index;
	u32 value;
	int err;

	err = regulator_enable(padctl->vddio_hsic);
	if (err)
		return err;

	padctl_writel(padctl, hsic->strobe_trim,
		      XUSB_PADCTL_HSIC_STRB_TRIM_CONTROL);

	value = padctl_readl(padctl, XUSB_PADCTL_HSIC_PADX_CTL1(port));
	value &= ~(XUSB_PADCTL_HSIC_PAD_CTL1_TX_RTUNEP_MASK <<
		   XUSB_PADCTL_HSIC_PAD_CTL1_TX_RTUNEP_SHIFT);
	value |= (hsic->tx_rtune_p <<
		  XUSB_PADCTL_HSIC_PAD_CTL1_TX_RTUNEP_SHIFT);
	padctl_writel(padctl, value, XUSB_PADCTL_HSIC_PADX_CTL1(port));

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

	value = padctl_readl(padctl, XUSB_PADCTL_HSIC_PADX_CTL0(port));
	value &= ~(XUSB_PADCTL_HSIC_PAD_CTL0_RPU_DATA0 |
		   XUSB_PADCTL_HSIC_PAD_CTL0_RPU_DATA1 |
		   XUSB_PADCTL_HSIC_PAD_CTL0_RPU_STROBE |
		   XUSB_PADCTL_HSIC_PAD_CTL0_PD_RX_DATA0 |
		   XUSB_PADCTL_HSIC_PAD_CTL0_PD_RX_DATA1 |
		   XUSB_PADCTL_HSIC_PAD_CTL0_PD_RX_STROBE |
		   XUSB_PADCTL_HSIC_PAD_CTL0_PD_ZI_DATA0 |
		   XUSB_PADCTL_HSIC_PAD_CTL0_PD_ZI_DATA1 |
		   XUSB_PADCTL_HSIC_PAD_CTL0_PD_ZI_STROBE |
		   XUSB_PADCTL_HSIC_PAD_CTL0_PD_TX_DATA0 |
		   XUSB_PADCTL_HSIC_PAD_CTL0_PD_TX_DATA1 |
		   XUSB_PADCTL_HSIC_PAD_CTL0_PD_TX_STROBE);
	value |= XUSB_PADCTL_HSIC_PAD_CTL0_RPD_DATA0 |
		 XUSB_PADCTL_HSIC_PAD_CTL0_RPD_DATA1 |
		 XUSB_PADCTL_HSIC_PAD_CTL0_RPD_STROBE;
	padctl_writel(padctl, value, XUSB_PADCTL_HSIC_PADX_CTL0(port));

	err = clk_prepare_enable(priv->hsic_trk);
	if (err)
		goto disable_regulator;

	value = padctl_readl(padctl, XUSB_PADCTL_HSIC_PAD_TRK_CTL);
	value &= ~((XUSB_PADCTL_HSIC_PAD_TRK_CTL_TRK_START_TIMER_MASK <<
		    XUSB_PADCTL_HSIC_PAD_TRK_CTL_TRK_START_TIMER_SHIFT) |
		   (XUSB_PADCTL_HSIC_PAD_TRK_CTL_TRK_DONE_RESET_TIMER_MASK <<
		    XUSB_PADCTL_HSIC_PAD_TRK_CTL_TRK_DONE_RESET_TIMER_SHIFT));
	value |= (XUSB_PADCTL_HSIC_PAD_TRK_CTL_TRK_START_TIMER_VAL <<
		  XUSB_PADCTL_HSIC_PAD_TRK_CTL_TRK_START_TIMER_SHIFT) |
		 (XUSB_PADCTL_HSIC_PAD_TRK_CTL_TRK_DONE_RESET_TIMER_VAL <<
		  XUSB_PADCTL_HSIC_PAD_TRK_CTL_TRK_DONE_RESET_TIMER_SHIFT);
	padctl_writel(padctl, value, XUSB_PADCTL_HSIC_PAD_TRK_CTL);

	udelay(1);

	value = padctl_readl(padctl, XUSB_PADCTL_HSIC_PAD_TRK_CTL);
	value &= ~XUSB_PADCTL_HSIC_PAD_TRK_CTL_PD_TRK;
	padctl_writel(padctl, value, XUSB_PADCTL_HSIC_PAD_TRK_CTL);

	udelay(50);

	clk_disable_unprepare(priv->hsic_trk);

	return 0;
disable_regulator:
	regulator_disable(padctl->vddio_hsic);
	return err;
}

static int tegra210_hsic_phy_power_off(struct phy *phy)
{
	struct tegra_xusb_hsic_phy *hsic = phy_get_drvdata(phy);
	struct tegra_xusb_padctl *padctl = hsic->padctl;
	unsigned int port = hsic->index;
	u32 value;

	value = padctl_readl(padctl, XUSB_PADCTL_HSIC_PADX_CTL0(port));
	value |= XUSB_PADCTL_HSIC_PAD_CTL0_PD_RX_DATA0 |
		 XUSB_PADCTL_HSIC_PAD_CTL0_PD_RX_DATA1 |
		 XUSB_PADCTL_HSIC_PAD_CTL0_PD_RX_STROBE |
		 XUSB_PADCTL_HSIC_PAD_CTL0_PD_ZI_DATA0 |
		 XUSB_PADCTL_HSIC_PAD_CTL0_PD_ZI_DATA1 |
		 XUSB_PADCTL_HSIC_PAD_CTL0_PD_ZI_STROBE |
		 XUSB_PADCTL_HSIC_PAD_CTL0_PD_TX_DATA0 |
		 XUSB_PADCTL_HSIC_PAD_CTL0_PD_TX_DATA1 |
		 XUSB_PADCTL_HSIC_PAD_CTL0_PD_TX_STROBE;
	padctl_writel(padctl, value, XUSB_PADCTL_HSIC_PADX_CTL1(port));

	regulator_disable(padctl->vddio_hsic);

	return 0;
}

static void tegra210_hsic_phy_set_idle(struct tegra_xusb_padctl *padctl,
				       unsigned int port, bool idle)
{
	u32 value;

	value = padctl_readl(padctl, XUSB_PADCTL_HSIC_PADX_CTL0(port));
	value &= ~(XUSB_PADCTL_HSIC_PAD_CTL0_RPU_DATA0 |
		   XUSB_PADCTL_HSIC_PAD_CTL0_RPU_DATA1 |
		   XUSB_PADCTL_HSIC_PAD_CTL0_RPD_STROBE);
	if (idle)
		value |= XUSB_PADCTL_HSIC_PAD_CTL0_RPD_DATA0 |
			 XUSB_PADCTL_HSIC_PAD_CTL0_RPD_DATA1 |
			 XUSB_PADCTL_HSIC_PAD_CTL0_RPU_STROBE;
	else
		value &= ~(XUSB_PADCTL_HSIC_PAD_CTL0_RPD_DATA0 |
			   XUSB_PADCTL_HSIC_PAD_CTL0_RPD_DATA1 |
			   XUSB_PADCTL_HSIC_PAD_CTL0_RPU_STROBE);
	padctl_writel(padctl, value, XUSB_PADCTL_HSIC_PADX_CTL0(port));
}

static void tegra210_hsic_phy_set_wake(struct tegra_xusb_hsic_phy *hsic,
				       bool enable)
{
	struct tegra_xusb_padctl *padctl = hsic->padctl;
	unsigned int index = hsic->index;
	u32 value;

	value = padctl_readl(padctl, XUSB_PADCTL_ELPG_PROGRAM0);
	value |= XUSB_PADCTL_ELPG_PROGRAM0_HSIC_PORTX_WAKEUP_EVENT(index);
	padctl_writel(padctl, value, XUSB_PADCTL_ELPG_PROGRAM0);

	usleep_range(10, 20);

	value = padctl_readl(padctl, XUSB_PADCTL_ELPG_PROGRAM0);
	if (enable)
		value |= XUSB_PADCTL_ELPG_PROGRAM0_HSIC_PORTX_WAKE_ENABLE(index);
	else
		value &= ~XUSB_PADCTL_ELPG_PROGRAM0_HSIC_PORTX_WAKE_ENABLE(index);
	padctl_writel(padctl, value, XUSB_PADCTL_ELPG_PROGRAM0);
}

static const struct phy_ops tegra210_hsic_phy_ops = {
	.init = tegra210_hsic_phy_init,
	.exit = tegra210_hsic_phy_exit,
	.power_on = tegra210_hsic_phy_power_on,
	.power_off = tegra210_hsic_phy_power_off,
	.owner = THIS_MODULE,
};

static void tegra210_handle_message(struct tegra_xusb_padctl *padctl,
				    struct tegra_xusb_mbox_msg *msg)
{
	struct tegra_xusb_mbox_msg resp;
	unsigned int i;
	u32 ports;

	resp.cmd = 0;
	switch (msg->cmd) {
	case MBOX_CMD_START_HSIC_IDLE:
	case MBOX_CMD_STOP_HSIC_IDLE:
		ports = msg->data >> (padctl->soc->hsic_port_offset + 1);
		resp.data = msg->data;
		resp.cmd = MBOX_CMD_ACK;
		for (i = 0; i < padctl->soc->num_hsic_phys; i++) {
			if (!(ports & BIT(i)))
				continue;
			if (msg->cmd == MBOX_CMD_START_HSIC_IDLE)
				tegra210_hsic_phy_set_idle(padctl, i, true);
			else
				tegra210_hsic_phy_set_idle(padctl, i, false);
		}
		break;
	case MBOX_CMD_DISABLE_SS_LFPS_DETECTION:
	case MBOX_CMD_ENABLE_SS_LFPS_DETECTION:
		ports = msg->data >> 1;
		resp.data = msg->data;
		resp.cmd = MBOX_CMD_ACK;
		for (i = 0; i < padctl->soc->num_usb3_phys; i++) {
			if (!(ports & BIT(i)))
				continue;
			if (msg->cmd == MBOX_CMD_ENABLE_SS_LFPS_DETECTION)
				tegra210_usb3_phy_set_lfps_detector(padctl, i,
								    true);
			else
				tegra210_usb3_phy_set_lfps_detector(padctl, i,
								    false);
		}
		break;
	default:
		break;
	}

	if (resp.cmd)
		mbox_send_message(padctl->mbox_chan, &resp);
}

static bool tegra210_is_phy_message(struct tegra_xusb_mbox_msg *msg)
{
	switch (msg->cmd) {
	case MBOX_CMD_START_HSIC_IDLE:
	case MBOX_CMD_STOP_HSIC_IDLE:
	case MBOX_CMD_DISABLE_SS_LFPS_DETECTION:
	case MBOX_CMD_ENABLE_SS_LFPS_DETECTION:
		return true;
	default:
		return false;
	}
}

static const struct pinctrl_pin_desc tegra210_pins[] = {
	PINCTRL_PIN(PIN_OTG_0,  "otg-0"),
	PINCTRL_PIN(PIN_OTG_1,  "otg-1"),
	PINCTRL_PIN(PIN_OTG_2,  "otg-2"),
	PINCTRL_PIN(PIN_OTG_3,  "otg-3"),
	PINCTRL_PIN(PIN_HSIC_0, "hsic-0"),
	PINCTRL_PIN(PIN_HSIC_1, "hsic-1"),
	PINCTRL_PIN(PIN_PCIE_0, "pcie-0"),
	PINCTRL_PIN(PIN_PCIE_1, "pcie-1"),
	PINCTRL_PIN(PIN_PCIE_2, "pcie-2"),
	PINCTRL_PIN(PIN_PCIE_3, "pcie-3"),
	PINCTRL_PIN(PIN_PCIE_4, "pcie-4"),
	PINCTRL_PIN(PIN_PCIE_5, "pcie-5"),
	PINCTRL_PIN(PIN_PCIE_6, "pcie-6"),
	PINCTRL_PIN(PIN_SATA_0, "sata-0"),
};

static const char * const tegra210_snps_groups[] = {
	"otg-0",
	"otg-1",
	"otg-2",
	"otg-3",
	"hsic-0",
	"hsic-1",
};

static const char * const tegra210_xusb_groups[] = {
	"otg-0",
	"otg-1",
	"otg-2",
	"otg-3",
	"hsic-0",
	"hsic-1",
};

static const char * const tegra210_uart_groups[] = {
	"otg-0",
	"otg-1",
	"otg-2",
	"otg-3",
};

static const char * const tegra210_pciex1_groups[] = {
	"pcie-0",
	"pcie-1",
	"pcie-2",
	"pcie-3",
	"pcie-4",
	"pcie-5",
	"pcie-6",
	"sata-0",
};

static const char * const tegra210_pciex4_groups[] = {
	"pcie-0",
	"pcie-1",
	"pcie-2",
	"pcie-3",
	"pcie-4",
	"pcie-5",
	"pcie-6",
	"sata-0",
};

static const char * const tegra210_usb3_groups[] = {
	"pcie-0",
	"pcie-1",
	"pcie-2",
	"pcie-3",
	"pcie-4",
	"pcie-5",
	"pcie-6",
	"sata-0",
};

static const char * const tegra210_sata_groups[] = {
	"pcie-0",
	"pcie-1",
	"pcie-2",
	"pcie-3",
	"pcie-4",
	"pcie-5",
	"pcie-6",
	"sata-0",
};

static const char * const tegra210_rsvd_groups[] = {
	"otg-0",
	"otg-1",
	"otg-2",
	"otg-3",
	"hsic-0",
	"hsic-1",
};

#define TEGRA210_FUNCTION(_name)					\
	{								\
		.name = #_name,						\
		.num_groups = ARRAY_SIZE(tegra210_##_name##_groups),	\
		.groups = tegra210_##_name##_groups,			\
	}

static struct tegra_xusb_padctl_function tegra210_functions[] = {
	TEGRA210_FUNCTION(snps),
	TEGRA210_FUNCTION(xusb),
	TEGRA210_FUNCTION(uart),
	TEGRA210_FUNCTION(pciex1),
	TEGRA210_FUNCTION(pciex4),
	TEGRA210_FUNCTION(usb3),
	TEGRA210_FUNCTION(sata),
	TEGRA210_FUNCTION(rsvd),
};

enum tegra210_function {
	TEGRA210_FUNC_SNPS,
	TEGRA210_FUNC_XUSB,
	TEGRA210_FUNC_UART,
	TEGRA210_FUNC_PCIE_X1,
	TEGRA210_FUNC_PCIE_X4,
	TEGRA210_FUNC_USB3,
	TEGRA210_FUNC_SATA,
	TEGRA210_FUNC_RSVD,
};

static const unsigned int tegra210_otg_functions[] = {
	TEGRA210_FUNC_SNPS,
	TEGRA210_FUNC_XUSB,
	TEGRA210_FUNC_UART,
	TEGRA210_FUNC_RSVD,
};

static const unsigned int tegra210_usb_functions[] = {
	TEGRA210_FUNC_SNPS,
	TEGRA210_FUNC_XUSB,
};

static const unsigned int tegra210_pci_functions[] = {
	TEGRA210_FUNC_PCIE_X1,
	TEGRA210_FUNC_USB3,
	TEGRA210_FUNC_SATA,
	TEGRA210_FUNC_PCIE_X4,
};

#define TEGRA210_LANE(_name, _offset, _shift, _mask, _iddq, _funcs)	\
	{								\
		.name = _name,						\
		.offset = _offset,					\
		.shift = _shift,					\
		.mask = _mask,						\
		.iddq = _iddq,						\
		.num_funcs = ARRAY_SIZE(tegra210_##_funcs##_functions),	\
		.funcs = tegra210_##_funcs##_functions,			\
	}

static const struct tegra_xusb_padctl_lane tegra210_lanes[] = {
	TEGRA210_LANE("otg-0",  0x004,  0, 0x3, 0, otg),
	TEGRA210_LANE("otg-1",  0x004,  2, 0x3, 0, otg),
	TEGRA210_LANE("otg-2",  0x004,  4, 0x3, 0, otg),
	TEGRA210_LANE("otg-3",  0x004,  6, 0x3, 0, otg),
	TEGRA210_LANE("hsic-0", 0x004, 14, 0x1, 0, usb),
	TEGRA210_LANE("hsic-1", 0x004, 15, 0x1, 0, usb),
	TEGRA210_LANE("pcie-0", 0x028, 12, 0x3, 1, pci),
	TEGRA210_LANE("pcie-1", 0x028, 14, 0x3, 2, pci),
	TEGRA210_LANE("pcie-2", 0x028, 16, 0x3, 3, pci),
	TEGRA210_LANE("pcie-3", 0x028, 18, 0x3, 4, pci),
	TEGRA210_LANE("pcie-4", 0x028, 20, 0x3, 5, pci),
	TEGRA210_LANE("pcie-5", 0x028, 22, 0x3, 6, pci),
	TEGRA210_LANE("pcie-6", 0x028, 24, 0x3, 7, pci),
	TEGRA210_LANE("sata-0", 0x028, 30, 0x3, 8, pci),
};

static int
tegra210_xusb_read_fuse_calibration(struct tegra210_xusb_fuse_calibration *fuse)
{
	unsigned int i;
	u32 value;
	int err;

	err = tegra_fuse_readl(TEGRA_FUSE_SKU_CALIB_0, &value);
	if (err < 0)
		return err;

	for (i = 0; i < ARRAY_SIZE(fuse->hs_curr_level); i++) {
		fuse->hs_curr_level[i] =
			(value >> FUSE_SKU_CALIB_HS_CURR_LEVEL_PADX_SHIFT(i)) &
			FUSE_SKU_CALIB_HS_CURR_LEVEL_PAD_MASK;
	}
	fuse->hs_term_range_adj =
		(value >> FUSE_SKU_CALIB_HS_TERM_RANGE_ADJ_SHIFT) &
		FUSE_SKU_CALIB_HS_TERM_RANGE_ADJ_MASK;

	err = tegra_fuse_readl(TEGRA_FUSE_USB_CALIB_EXT_0, &value);
	if (err < 0)
		return err;

	fuse->rpd_ctrl =
		(value >> FUSE_USB_CALIB_EXT_RPD_CTRL_SHIFT) &
		FUSE_USB_CALIB_EXT_RPD_CTRL_MASK;

	return 0;
}

static int tegra210_xusb_padctl_probe(struct tegra_xusb_padctl *padctl)
{
	struct tegra210_xusb_padctl *priv;
	int err;

	priv = devm_kzalloc(padctl->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	padctl->priv = priv;

	err = tegra210_xusb_read_fuse_calibration(&priv->fuse);
	if (err < 0)
		return err;

	priv->pex_rst = devm_reset_control_get(padctl->dev, "pex_uphy");
	if (IS_ERR(priv->pex_rst))
		return PTR_ERR(priv->pex_rst);
	priv->sata_rst = devm_reset_control_get(padctl->dev, "sata_uphy");
	if (IS_ERR(priv->sata_rst))
		return PTR_ERR(priv->sata_rst);

	priv->usb2_trk = devm_clk_get(padctl->dev, "usb2_trk");
	if (IS_ERR(priv->usb2_trk))
		return PTR_ERR(priv->usb2_trk);
	priv->hsic_trk = devm_clk_get(padctl->dev, "hsic_trk");
	if (IS_ERR(priv->hsic_trk))
		return PTR_ERR(priv->hsic_trk);
	priv->pll_e = devm_clk_get(padctl->dev, "pll_e");
	if (IS_ERR(priv->pll_e))
		return PTR_ERR(priv->pll_e);

	return 0;
}

static void tegra210_xusb_padctl_remove(struct tegra_xusb_padctl *padctl)
{
}

static int tegra210_xusb_padctl_suspend(struct tegra_xusb_padctl *padctl)
{
	struct tegra210_xusb_padctl *priv = padctl->priv;
	struct tegra210_xusb_padctl_saved_regs *regs = &priv->saved_regs;

	regs->usb2_pad_mux = padctl_readl(padctl, XUSB_PADCTL_USB2_PAD_MUX);
	regs->usb3_pad_mux = padctl_readl(padctl, XUSB_PADCTL_USB3_PAD_MUX);

	return 0;
}

static int tegra210_xusb_padctl_resume(struct tegra_xusb_padctl *padctl)
{
	struct tegra210_xusb_padctl *priv = padctl->priv;
	struct tegra210_xusb_padctl_saved_regs *regs = &priv->saved_regs;

	padctl_writel(padctl, regs->usb2_pad_mux, XUSB_PADCTL_USB2_PAD_MUX);
	padctl_writel(padctl, regs->usb3_pad_mux, XUSB_PADCTL_USB3_PAD_MUX);

	return 0;
}

const struct tegra_xusb_padctl_soc tegra210_xusb_padctl_soc = {
	.num_pins = ARRAY_SIZE(tegra210_pins),
	.pins = tegra210_pins,
	.num_functions = ARRAY_SIZE(tegra210_functions),
	.functions = tegra210_functions,
	.num_lanes = ARRAY_SIZE(tegra210_lanes),
	.lanes = tegra210_lanes,
	.pcie_phy_ops = &tegra210_pcie_phy_ops,
	.sata_phy_ops = &tegra210_sata_phy_ops,
	.num_usb3_phys = 4,
	.usb3_phy_ops = &tegra210_usb3_phy_ops,
	.usb3_set_wake = tegra210_usb3_phy_set_wake,
	.num_utmi_phys = 4,
	.utmi_phy_ops = &tegra210_utmi_phy_ops,
	.utmi_set_wake = tegra210_utmi_phy_set_wake,
	.utmi_get_pad_config = tegra210_utmi_phy_get_pad_config,
	.utmi_vbus_override = tegra210_utmi_vbus_override,
	.num_hsic_phys = 2,
	.hsic_phy_ops = &tegra210_hsic_phy_ops,
	.hsic_set_wake = tegra210_hsic_phy_set_wake,
	.hsic_port_offset = 8,
	.is_phy_message = tegra210_is_phy_message,
	.handle_message = tegra210_handle_message,
	.probe = tegra210_xusb_padctl_probe,
	.remove = tegra210_xusb_padctl_remove,
	.suspend = tegra210_xusb_padctl_suspend,
	.resume = tegra210_xusb_padctl_resume,
};
EXPORT_SYMBOL_GPL(tegra210_xusb_padctl_soc);

MODULE_AUTHOR("Andrew Bresticker <abrestic@chromium.org>");
MODULE_DESCRIPTION("Tegra 210 XUSB Pad Control driver");
MODULE_LICENSE("GPL v2");
