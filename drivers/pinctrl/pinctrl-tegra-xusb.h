/*
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
 * Copyright (c) 2015, Google Inc.
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

#ifndef __PINCTRL_TEGRA_XUSB_H
#define __PINCTRL_TEGRA_XUSB_H

#include <linux/io.h>
#include <linux/mailbox_client.h>
#include <linux/mutex.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/workqueue.h>

#include <soc/tegra/xusb.h>

#include <dt-bindings/pinctrl/pinctrl-tegra-xusb.h>

struct phy;
struct phy_provider;
struct platform_device;
struct regulator;
struct tegra_xusb_padctl;

struct tegra_xusb_padctl_function {
	const char *name;
	const char * const *groups;
	unsigned int num_groups;
};

struct tegra_xusb_padctl_group {
	const unsigned int *funcs;
	unsigned int num_funcs;
};

struct tegra_xusb_padctl_soc {
	const struct pinctrl_pin_desc *pins;
	unsigned int num_pins;

	const struct tegra_xusb_padctl_function *functions;
	unsigned int num_functions;

	const struct tegra_xusb_padctl_lane *lanes;
	unsigned int num_lanes;

	const struct phy_ops *pcie_phy_ops;
	const struct phy_ops *sata_phy_ops;

	const struct phy_ops *usb3_phy_ops;
	unsigned int num_usb3_phys;

	const struct phy_ops *utmi_phy_ops;
	unsigned int num_utmi_phys;

	const struct phy_ops *hsic_phy_ops;
	unsigned int num_hsic_phys;
	unsigned int hsic_port_offset;

	bool (*is_phy_message)(struct tegra_xusb_mbox_msg *msg);
	void (*handle_message)(struct tegra_xusb_padctl *padctl,
			       struct tegra_xusb_mbox_msg *msg);

	int (*probe)(struct tegra_xusb_padctl *padctl);
	void (*remove)(struct tegra_xusb_padctl *padctl);
};

struct tegra_xusb_padctl_lane {
	const char *name;

	unsigned int offset;
	unsigned int shift;
	unsigned int mask;
	unsigned int iddq;

	const unsigned int *funcs;
	unsigned int num_funcs;
};

struct tegra_xusb_usb3_phy {
	struct tegra_xusb_padctl *padctl;
	bool context_saved;
	unsigned int index;
	unsigned int lane;
	unsigned int port;

	u32 tap1;
	u32 amp;
	u32 ctle_z;
	u32 ctle_g;
};

struct tegra_xusb_utmi_phy {
	struct tegra_xusb_padctl *padctl;
	unsigned int index;

	u32 hs_curr_level_offset;
	struct regulator *supply;
};

struct tegra_xusb_hsic_phy {
	struct tegra_xusb_padctl *padctl;
	unsigned int index;

	u32 strobe_trim;
	u32 rx_strobe_trim;
	u32 rx_data_trim;
	u32 tx_rtune_n;
	u32 tx_rtune_p;
	u32 tx_rslew_n;
	u32 tx_rslew_p;
	bool auto_term;
};

struct tegra_xusb_padctl {
	struct device *dev;
	void __iomem *regs;
	struct mutex lock;
	struct reset_control *rst;

	const struct tegra_xusb_padctl_soc *soc;
	struct pinctrl_dev *pinctrl;
	struct pinctrl_desc desc;

	struct phy_provider *provider;
	struct phy *phys[TEGRA_XUSB_PADCTL_MAX_PHYS];

	struct tegra_xusb_hsic_phy *hsic_phys;
	struct tegra_xusb_utmi_phy *utmi_phys;
	struct tegra_xusb_usb3_phy *usb3_phys;

	unsigned int enable;

	struct work_struct mbox_req_work;
	struct tegra_xusb_mbox_msg mbox_req;
	struct mbox_client mbox_client;
	struct mbox_chan *mbox_chan;

	unsigned int utmi_enable;
	struct regulator *vddio_hsic;

	void *priv;
};

static inline void padctl_writel(struct tegra_xusb_padctl *padctl, u32 value,
				 unsigned long offset)
{
	writel(value, padctl->regs + offset);
}

static inline u32 padctl_readl(struct tegra_xusb_padctl *padctl,
			       unsigned long offset)
{
	return readl(padctl->regs + offset);
}

#define PIN_OTG_0   0
#define PIN_OTG_1   1
#define PIN_OTG_2   2
#define PIN_OTG_3   3 /* on Tegra210 */
#define PIN_ULPI_0  4 /* on Tegra124 */
#define PIN_HSIC_0  5
#define PIN_HSIC_1  6
#define PIN_PCIE_0  7
#define PIN_PCIE_1  8
#define PIN_PCIE_2  9
#define PIN_PCIE_3 10
#define PIN_PCIE_4 11
#define PIN_PCIE_5 12 /* on Tegra210 */
#define PIN_PCIE_6 13 /* on Tegra210 */
#define PIN_SATA_0 14

static inline bool lane_is_otg(unsigned int lane)
{
	return lane >= PIN_OTG_0 && lane <= PIN_OTG_3;
}

static inline bool lane_is_hsic(unsigned int lane)
{
	return lane >= PIN_HSIC_0 && lane <= PIN_HSIC_1;
}

static inline bool lane_is_pcie_or_sata(unsigned int lane)
{
	return lane >= PIN_PCIE_0 && lane <= PIN_SATA_0;
}

static inline struct tegra_xusb_utmi_phy *
lane_to_utmi_phy(struct tegra_xusb_padctl *padctl, unsigned int lane)
{
	if (!lane_is_otg(lane))
		return NULL;
	return &padctl->utmi_phys[lane - PIN_OTG_0];
}

static inline struct tegra_xusb_hsic_phy *
lane_to_hsic_phy(struct tegra_xusb_padctl *padctl, unsigned int lane)
{
	if (!lane_is_hsic(lane))
		return NULL;
	return &padctl->hsic_phys[lane - PIN_HSIC_0];
}

#ifdef CONFIG_PINCTRL_TEGRA124_XUSB
extern const struct tegra_xusb_padctl_soc tegra124_xusb_padctl_soc;
#endif
#ifdef CONFIG_PINCTRL_TEGRA210_XUSB
extern const struct tegra_xusb_padctl_soc tegra210_xusb_padctl_soc;
#endif

#endif /* __PINCTRL_TEGRA_XUSB_H */
