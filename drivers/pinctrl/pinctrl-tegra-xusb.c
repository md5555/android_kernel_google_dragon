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
#include <linux/workqueue.h>

#include <soc/tegra/fuse.h>
#include <soc/tegra/xusb.h>

#include <dt-bindings/pinctrl/pinctrl-tegra-xusb.h>

#include "core.h"
#include "pinctrl-tegra-xusb.h"
#include "pinctrl-utils.h"

static int tegra_xusb_padctl_get_groups_count(struct pinctrl_dev *pinctrl)
{
	struct tegra_xusb_padctl *padctl = pinctrl_dev_get_drvdata(pinctrl);

	return padctl->soc->num_pins;
}

static const char *tegra_xusb_padctl_get_group_name(struct pinctrl_dev *pinctrl,
						    unsigned int group)
{
	struct tegra_xusb_padctl *padctl = pinctrl_dev_get_drvdata(pinctrl);

	return padctl->soc->pins[group].name;
}

enum tegra_xusb_padctl_param {
	TEGRA_XUSB_PADCTL_IDDQ,
	TEGRA_XUSB_PADCTL_HSIC_STROBE_TRIM,
	TEGRA_XUSB_PADCTL_HSIC_RX_STROBE_TRIM,
	TEGRA_XUSB_PADCTL_HSIC_RX_DATA_TRIM,
	TEGRA_XUSB_PADCTL_HSIC_TX_RTUNEN,
	TEGRA_XUSB_PADCTL_HSIC_TX_RTUNEP,
	TEGRA_XUSB_PADCTL_HSIC_TX_RSLEWN,
	TEGRA_XUSB_PADCTL_HSIC_TX_RSLEWP,
	TEGRA_XUSB_PADCTL_HSIC_AUTO_TERM,
	TEGRA_XUSB_PADCTL_OTG_HS_CURR_LEVEL_OFFSET,
};

static const struct tegra_xusb_padctl_property {
	const char *name;
	enum tegra_xusb_padctl_param param;
} properties[] = {
	{ "nvidia,iddq", TEGRA_XUSB_PADCTL_IDDQ },
	{ "nvidia,hsic-strobe-trim", TEGRA_XUSB_PADCTL_HSIC_STROBE_TRIM },
	{ "nvidia,hsic-rx-strobe-trim", TEGRA_XUSB_PADCTL_HSIC_RX_STROBE_TRIM },
	{ "nvidia,hsic-rx-data-trim", TEGRA_XUSB_PADCTL_HSIC_RX_DATA_TRIM },
	{ "nvidia,hsic-tx-rtune-n", TEGRA_XUSB_PADCTL_HSIC_TX_RTUNEN },
	{ "nvidia,hsic-tx-rtune-p", TEGRA_XUSB_PADCTL_HSIC_TX_RTUNEP },
	{ "nvidia,hsic-tx-rslew-n", TEGRA_XUSB_PADCTL_HSIC_TX_RSLEWN },
	{ "nvidia,hsic-tx-rslew-p", TEGRA_XUSB_PADCTL_HSIC_TX_RSLEWP },
	{ "nvidia,hsic-auto-term", TEGRA_XUSB_PADCTL_HSIC_AUTO_TERM },
	{ "nvidia,otg-hs-curr-level-offset",
	  TEGRA_XUSB_PADCTL_OTG_HS_CURR_LEVEL_OFFSET },
};

#define TEGRA_XUSB_PADCTL_PACK(param, value) ((param) << 16 | (value))
#define TEGRA_XUSB_PADCTL_UNPACK_PARAM(config) ((config) >> 16)
#define TEGRA_XUSB_PADCTL_UNPACK_VALUE(config) ((config) & 0xffff)

static int tegra_xusb_padctl_parse_subnode(struct tegra_xusb_padctl *padctl,
					   struct device_node *np,
					   struct pinctrl_map **maps,
					   unsigned int *reserved_maps,
					   unsigned int *num_maps)
{
	unsigned int i, reserve = 0, num_configs = 0;
	unsigned long config, *configs = NULL;
	const char *function, *group;
	struct property *prop;
	int err = 0;
	u32 value;

	err = of_property_read_string(np, "nvidia,function", &function);
	if (err < 0) {
		if (err != -EINVAL)
			return err;

		function = NULL;
	}

	for (i = 0; i < ARRAY_SIZE(properties); i++) {
		err = of_property_read_u32(np, properties[i].name, &value);
		if (err < 0) {
			if (err == -EINVAL)
				continue;

			goto out;
		}

		config = TEGRA_XUSB_PADCTL_PACK(properties[i].param, value);

		err = pinctrl_utils_add_config(padctl->pinctrl, &configs,
					       &num_configs, config);
		if (err < 0)
			goto out;
	}

	if (function)
		reserve++;

	if (num_configs)
		reserve++;

	err = of_property_count_strings(np, "nvidia,lanes");
	if (err < 0)
		goto out;

	reserve *= err;

	err = pinctrl_utils_reserve_map(padctl->pinctrl, maps, reserved_maps,
					num_maps, reserve);
	if (err < 0)
		goto out;

	of_property_for_each_string(np, "nvidia,lanes", prop, group) {
		if (function) {
			err = pinctrl_utils_add_map_mux(padctl->pinctrl, maps,
					reserved_maps, num_maps, group,
					function);
			if (err < 0)
				goto out;
		}

		if (num_configs) {
			err = pinctrl_utils_add_map_configs(padctl->pinctrl,
					maps, reserved_maps, num_maps, group,
					configs, num_configs,
					PIN_MAP_TYPE_CONFIGS_GROUP);
			if (err < 0)
				goto out;
		}
	}

	err = 0;

out:
	kfree(configs);
	return err;
}

static int tegra_xusb_padctl_dt_node_to_map(struct pinctrl_dev *pinctrl,
					    struct device_node *parent,
					    struct pinctrl_map **maps,
					    unsigned int *num_maps)
{
	struct tegra_xusb_padctl *padctl = pinctrl_dev_get_drvdata(pinctrl);
	unsigned int reserved_maps = 0;
	struct device_node *np;
	int err;

	*num_maps = 0;
	*maps = NULL;

	for_each_child_of_node(parent, np) {
		err = tegra_xusb_padctl_parse_subnode(padctl, np, maps,
						      &reserved_maps,
						      num_maps);
		if (err < 0)
			return err;
	}

	return 0;
}

static const struct pinctrl_ops tegra_xusb_padctl_pinctrl_ops = {
	.get_groups_count = tegra_xusb_padctl_get_groups_count,
	.get_group_name = tegra_xusb_padctl_get_group_name,
	.dt_node_to_map = tegra_xusb_padctl_dt_node_to_map,
	.dt_free_map = pinctrl_utils_dt_free_map,
};

static int tegra_xusb_padctl_get_functions_count(struct pinctrl_dev *pinctrl)
{
	struct tegra_xusb_padctl *padctl = pinctrl_dev_get_drvdata(pinctrl);

	return padctl->soc->num_functions;
}

static const char *
tegra_xusb_padctl_get_function_name(struct pinctrl_dev *pinctrl,
				    unsigned int function)
{
	struct tegra_xusb_padctl *padctl = pinctrl_dev_get_drvdata(pinctrl);

	return padctl->soc->functions[function].name;
}

static int tegra_xusb_padctl_get_function_groups(struct pinctrl_dev *pinctrl,
						 unsigned int function,
						 const char * const **groups,
						 unsigned * const num_groups)
{
	struct tegra_xusb_padctl *padctl = pinctrl_dev_get_drvdata(pinctrl);

	*num_groups = padctl->soc->functions[function].num_groups;
	*groups = padctl->soc->functions[function].groups;

	return 0;
}

static int tegra_xusb_padctl_pinmux_set(struct pinctrl_dev *pinctrl,
					unsigned int function,
					unsigned int group)
{
	struct tegra_xusb_padctl *padctl = pinctrl_dev_get_drvdata(pinctrl);
	const struct tegra_xusb_padctl_lane *lane;
	unsigned int i;
	u32 value;

	lane = &padctl->soc->lanes[group];

	for (i = 0; i < lane->num_funcs; i++)
		if (lane->funcs[i] == function)
			break;

	if (i >= lane->num_funcs)
		return -EINVAL;

	value = padctl_readl(padctl, lane->offset);
	value &= ~(lane->mask << lane->shift);
	value |= i << lane->shift;
	padctl_writel(padctl, value, lane->offset);

	return 0;
}

static const struct pinmux_ops tegra_xusb_padctl_pinmux_ops = {
	.get_functions_count = tegra_xusb_padctl_get_functions_count,
	.get_function_name = tegra_xusb_padctl_get_function_name,
	.get_function_groups = tegra_xusb_padctl_get_function_groups,
	.set_mux = tegra_xusb_padctl_pinmux_set,
};

static int tegra_xusb_padctl_pinconf_group_get(struct pinctrl_dev *pinctrl,
					       unsigned int group,
					       unsigned long *config)
{
	struct tegra_xusb_padctl *padctl = pinctrl_dev_get_drvdata(pinctrl);
	const struct tegra_xusb_padctl_lane *lane;
	enum tegra_xusb_padctl_param param;
	struct tegra_xusb_hsic_phy *hsic;
	struct tegra_xusb_utmi_phy *utmi;
	u32 value;

	param = TEGRA_XUSB_PADCTL_UNPACK_PARAM(*config);
	lane = &padctl->soc->lanes[group];

	switch (param) {
	case TEGRA_XUSB_PADCTL_IDDQ:
		/* lanes with iddq == 0 don't support this parameter */
		if (lane->iddq == 0)
			return -EINVAL;

		value = padctl_readl(padctl, lane->offset);

		if (value & BIT(lane->iddq))
			value = 0;
		else
			value = 1;
		break;

	case TEGRA_XUSB_PADCTL_HSIC_STROBE_TRIM:
		hsic = lane_to_hsic_phy(padctl, group);
		if (!hsic) {
			dev_err(padctl->dev, "Pin %d not an HSIC\n", group);
			return -EINVAL;
		}

		value = hsic->strobe_trim;
		break;

	case TEGRA_XUSB_PADCTL_HSIC_RX_STROBE_TRIM:
		hsic = lane_to_hsic_phy(padctl, group);
		if (!hsic) {
			dev_err(padctl->dev, "Pin %d not an HSIC\n", group);
			return -EINVAL;
		}

		value = hsic->rx_strobe_trim;
		break;

	case TEGRA_XUSB_PADCTL_HSIC_RX_DATA_TRIM:
		hsic = lane_to_hsic_phy(padctl, group);
		if (!hsic) {
			dev_err(padctl->dev, "Pin %d not an HSIC\n", group);
			return -EINVAL;
		}

		value = hsic->rx_data_trim;
		break;

	case TEGRA_XUSB_PADCTL_HSIC_TX_RTUNEN:
		hsic = lane_to_hsic_phy(padctl, group);
		if (!hsic) {
			dev_err(padctl->dev, "Pin %d not an HSIC\n", group);
			return -EINVAL;
		}

		value = hsic->tx_rtune_n;
		break;

	case TEGRA_XUSB_PADCTL_HSIC_TX_RTUNEP:
		hsic = lane_to_hsic_phy(padctl, group);
		if (!hsic) {
			dev_err(padctl->dev, "Pin %d not an HSIC\n", group);
			return -EINVAL;
		}

		value = hsic->tx_rtune_p;
		break;

	case TEGRA_XUSB_PADCTL_HSIC_TX_RSLEWN:
		hsic = lane_to_hsic_phy(padctl, group);
		if (!hsic) {
			dev_err(padctl->dev, "Pin %d not an HSIC\n", group);
			return -EINVAL;
		}

		value = hsic->tx_rslew_n;
		break;

	case TEGRA_XUSB_PADCTL_HSIC_TX_RSLEWP:
		hsic = lane_to_hsic_phy(padctl, group);
		if (!hsic) {
			dev_err(padctl->dev, "Pin %d not an HSIC\n", group);
			return -EINVAL;
		}

		value = hsic->tx_rslew_p;
		break;

	case TEGRA_XUSB_PADCTL_HSIC_AUTO_TERM:
		hsic = lane_to_hsic_phy(padctl, group);
		if (!hsic) {
			dev_err(padctl->dev, "Pin %d not an HSIC\n", group);
			return -EINVAL;
		}

		value = hsic->auto_term;
		break;

	case TEGRA_XUSB_PADCTL_OTG_HS_CURR_LEVEL_OFFSET:
		utmi = lane_to_utmi_phy(padctl, group);
		if (!utmi) {
			dev_err(padctl->dev, "Pin %d is not an OTG pad\n",
				group);
			return -EINVAL;
		}

		value = utmi->hs_curr_level_offset;
		break;

	default:
		dev_err(padctl->dev, "invalid configuration parameter: %04x\n",
			param);
		return -ENOTSUPP;
	}

	*config = TEGRA_XUSB_PADCTL_PACK(param, value);
	return 0;
}

static int tegra_xusb_padctl_pinconf_group_set(struct pinctrl_dev *pinctrl,
					       unsigned int group,
					       unsigned long *configs,
					       unsigned int num_configs)
{
	struct tegra_xusb_padctl *padctl = pinctrl_dev_get_drvdata(pinctrl);
	const struct tegra_xusb_padctl_lane *lane;
	enum tegra_xusb_padctl_param param;
	struct tegra_xusb_hsic_phy *hsic;
	struct tegra_xusb_utmi_phy *utmi;
	unsigned long value;
	unsigned int i;
	u32 regval;

	lane = &padctl->soc->lanes[group];

	for (i = 0; i < num_configs; i++) {
		param = TEGRA_XUSB_PADCTL_UNPACK_PARAM(configs[i]);
		value = TEGRA_XUSB_PADCTL_UNPACK_VALUE(configs[i]);

		switch (param) {
		case TEGRA_XUSB_PADCTL_IDDQ:
			/* lanes with iddq == 0 don't support this parameter */
			if (lane->iddq == 0)
				return -EINVAL;

			regval = padctl_readl(padctl, lane->offset);

			if (value)
				regval &= ~BIT(lane->iddq);
			else
				regval |= BIT(lane->iddq);

			padctl_writel(padctl, regval, lane->offset);
			break;

		case TEGRA_XUSB_PADCTL_HSIC_STROBE_TRIM:
			hsic = lane_to_hsic_phy(padctl, group);
			if (!hsic) {
				dev_err(padctl->dev, "Pin %d not an HSIC\n",
					group);
				return -EINVAL;
			}

			hsic->strobe_trim = value;
			break;

		case TEGRA_XUSB_PADCTL_HSIC_RX_STROBE_TRIM:
			hsic = lane_to_hsic_phy(padctl, group);
			if (!hsic) {
				dev_err(padctl->dev, "Pin %d not an HSIC\n",
					group);
				return -EINVAL;
			}

			hsic->rx_strobe_trim = value;
			break;

		case TEGRA_XUSB_PADCTL_HSIC_RX_DATA_TRIM:
			hsic = lane_to_hsic_phy(padctl, group);
			if (!hsic) {
				dev_err(padctl->dev, "Pin %d not an HSIC\n",
					group);
				return -EINVAL;
			}

			hsic->rx_data_trim = value;
			break;

		case TEGRA_XUSB_PADCTL_HSIC_TX_RTUNEN:
			hsic = lane_to_hsic_phy(padctl, group);
			if (!hsic) {
				dev_err(padctl->dev, "Pin %d not an HSIC\n",
					group);
				return -EINVAL;
			}

			hsic->tx_rtune_n = value;
			break;

		case TEGRA_XUSB_PADCTL_HSIC_TX_RTUNEP:
			hsic = lane_to_hsic_phy(padctl, group);
			if (!hsic) {
				dev_err(padctl->dev, "Pin %d not an HSIC\n",
					group);
				return -EINVAL;
			}

			hsic->tx_rtune_p = value;
			break;

		case TEGRA_XUSB_PADCTL_HSIC_TX_RSLEWN:
			hsic = lane_to_hsic_phy(padctl, group);
			if (!hsic) {
				dev_err(padctl->dev, "Pin %d not an HSIC\n",
					group);
				return -EINVAL;
			}

			hsic->tx_rslew_n = value;
			break;

		case TEGRA_XUSB_PADCTL_HSIC_TX_RSLEWP:
			hsic = lane_to_hsic_phy(padctl, group);
			if (!hsic) {
				dev_err(padctl->dev, "Pin %d not an HSIC\n",
					group);
				return -EINVAL;
			}

			hsic->tx_rslew_n = value;
			break;

		case TEGRA_XUSB_PADCTL_HSIC_AUTO_TERM:
			hsic = lane_to_hsic_phy(padctl, group);
			if (!hsic) {
				dev_err(padctl->dev, "Pin %d not an HSIC\n",
					group);
				return -EINVAL;
			}

			hsic->auto_term = !!value;
			break;

		case TEGRA_XUSB_PADCTL_OTG_HS_CURR_LEVEL_OFFSET:
			utmi = lane_to_utmi_phy(padctl, group);
			if (!utmi) {
				dev_err(padctl->dev,
					"Pin %d is not an OTG pad\n", group);
				return -EINVAL;
			}

			utmi->hs_curr_level_offset = value;
			break;

		default:
			dev_err(padctl->dev,
				"invalid configuration parameter: %04x\n",
				param);
			return -ENOTSUPP;
		}
	}

	return 0;
}

#ifdef CONFIG_DEBUG_FS
static const char *strip_prefix(const char *s)
{
	const char *comma = strchr(s, ',');
	if (!comma)
		return s;

	return comma + 1;
}

static void
tegra_xusb_padctl_pinconf_group_dbg_show(struct pinctrl_dev *pinctrl,
					 struct seq_file *s,
					 unsigned int group)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(properties); i++) {
		unsigned long config, value;
		int err;

		config = TEGRA_XUSB_PADCTL_PACK(properties[i].param, 0);

		err = tegra_xusb_padctl_pinconf_group_get(pinctrl, group,
							  &config);
		if (err < 0)
			continue;

		value = TEGRA_XUSB_PADCTL_UNPACK_VALUE(config);

		seq_printf(s, "\n\t%s=%lu\n", strip_prefix(properties[i].name),
			   value);
	}
}

static void
tegra_xusb_padctl_pinconf_config_dbg_show(struct pinctrl_dev *pinctrl,
					  struct seq_file *s,
					  unsigned long config)
{
	enum tegra_xusb_padctl_param param;
	const char *name = "unknown";
	unsigned long value;
	unsigned int i;

	param = TEGRA_XUSB_PADCTL_UNPACK_PARAM(config);
	value = TEGRA_XUSB_PADCTL_UNPACK_VALUE(config);

	for (i = 0; i < ARRAY_SIZE(properties); i++) {
		if (properties[i].param == param) {
			name = properties[i].name;
			break;
		}
	}

	seq_printf(s, "%s=%lu", strip_prefix(name), value);
}
#endif

static const struct pinconf_ops tegra_xusb_padctl_pinconf_ops = {
	.pin_config_group_get = tegra_xusb_padctl_pinconf_group_get,
	.pin_config_group_set = tegra_xusb_padctl_pinconf_group_set,
#ifdef CONFIG_DEBUG_FS
	.pin_config_group_dbg_show = tegra_xusb_padctl_pinconf_group_dbg_show,
	.pin_config_config_dbg_show = tegra_xusb_padctl_pinconf_config_dbg_show,
#endif
};

static inline struct tegra_xusb_padctl *
mbox_work_to_padctl(struct work_struct *work)
{
	return container_of(work, struct tegra_xusb_padctl, mbox_req_work);
}

static void tegra_xusb_phy_mbox_work(struct work_struct *work)
{
	struct tegra_xusb_padctl *padctl = mbox_work_to_padctl(work);
	struct tegra_xusb_mbox_msg *msg = &padctl->mbox_req;

	padctl->soc->handle_message(padctl, msg);
}

static void tegra_xusb_phy_mbox_rx(struct mbox_client *cl, void *data)
{
	struct tegra_xusb_padctl *padctl = dev_get_drvdata(cl->dev);
	struct tegra_xusb_mbox_msg *msg = data;

	if (padctl->soc->is_phy_message(msg)) {
		padctl->mbox_req = *msg;
		schedule_work(&padctl->mbox_req_work);
	}
}

static struct phy *tegra_xusb_padctl_xlate(struct device *dev,
					   struct of_phandle_args *args)
{
	struct tegra_xusb_padctl *padctl = dev_get_drvdata(dev);
	unsigned int index = args->args[0];

	if (args->args_count <= 0)
		return ERR_PTR(-EINVAL);

	if (index >= ARRAY_SIZE(padctl->phys))
		return ERR_PTR(-EINVAL);

	if (!padctl->phys[index])
		return ERR_PTR(-EINVAL);

	return padctl->phys[index];
}

static const struct of_device_id tegra_xusb_padctl_of_match[] = {
#ifdef CONFIG_PINCTRL_TEGRA124_XUSB
	{
		.compatible = "nvidia,tegra124-xusb-padctl",
		.data = &tegra124_xusb_padctl_soc
	},
#endif
	{ }
};
MODULE_DEVICE_TABLE(of, tegra_xusb_padctl_of_match);

static int tegra_xusb_padctl_find_pin_by_name(struct tegra_xusb_padctl *padctl,
					      const char *name)
{
	unsigned int i;

	for (i = 0; i < padctl->soc->num_pins; i++) {
		const struct pinctrl_pin_desc *pin = &padctl->soc->pins[i];

		if (strcmp(pin->name, name) == 0)
			return pin->number;
	}

	return -ENODEV;
}

static struct device_node *
tegra_xusb_padctl_find_phy_node(struct tegra_xusb_padctl *padctl,
				const char *type, unsigned int index)
{
	struct device_node *np;

	np = of_find_node_by_name(padctl->dev->of_node, "phys");
	if (np) {
		struct device_node *of_node;
		char *name;

		name = kasprintf(GFP_KERNEL, "%s-%u", type, index);
		of_node = of_find_node_by_name(np, name);
		kfree(name);

		of_node_put(np);
		np = of_node;
	}

	return np;
}

static int tegra_xusb_usb3_phy_parse_dt(struct phy *phy)
{
	struct tegra_xusb_usb3_phy *usb = phy_get_drvdata(phy);
	struct device_node *np = phy->dev.of_node;
	const char *lane = NULL;
	u32 value;
	int err;

	if (!np)
		return 0;

	/* only a single lane can be mapped to each USB3 port */
	err = of_property_count_strings(np, "nvidia,lanes");
	if (err < 0 && err != -EINVAL) {
		dev_err(&phy->dev, "failed to get number of lanes: %d\n", err);
		return err;
	}

	if (err > 1)
		dev_warn(&phy->dev, "found %d lanes, expected 1\n", err);

	err = of_property_read_string(np, "nvidia,lanes", &lane);
	if (err < 0) {
		dev_err(&phy->dev, "failed to read lanes: %d\n", err);
		return err;
	}

	if (lane) {
		err = tegra_xusb_padctl_find_pin_by_name(usb->padctl, lane);
		if (err < 0) {
			dev_err(&phy->dev, "unknown pin: %s\n", lane);
			return err;
		}

		if (!lane_is_pcie_or_sata(err)) {
			dev_err(&phy->dev,
				"USB3 PHY %u mapped to invalid lane %s\n",
				usb->index, lane);
			return -EINVAL;
		}

		usb->lane = err;
	}

	err = of_property_read_u32_index(np, "nvidia,port", 0, &value);
	if (err < 0) {
		dev_err(&phy->dev, "failed to read port: %d\n", err);
		return err;
	}

	usb->port = value;

	return 0;
}

static struct phy *tegra_xusb_usb3_phy_create(struct tegra_xusb_padctl *padctl,
					      unsigned int index)
{
	struct tegra_xusb_usb3_phy *usb;
	struct device_node *np;
	struct phy *phy;
	int err;

	/*
	 * If there is no supplemental configuration in the device tree the
	 * PHY is unusable. But it is valid to configure only a single PHY,
	 * hence return NULL instead of an error to mark the PHY as not in
	 * use. Similarly if the PHY is marked as disabled, don't register
	 * it.
	 */
	np = tegra_xusb_padctl_find_phy_node(padctl, "usb3", index);
	if (!np || !of_device_is_available(np))
		return NULL;

	phy = devm_phy_create(padctl->dev, np, padctl->soc->usb3_phy_ops, NULL);
	if (IS_ERR(phy))
		return phy;

	usb = &padctl->usb3_phys[index];
	phy_set_drvdata(phy, usb);
	usb->padctl = padctl;
	usb->index = index;

	err = tegra_xusb_usb3_phy_parse_dt(phy);
	if (err < 0)
		return ERR_PTR(err);

	return phy;
}

static struct phy *tegra_xusb_utmi_phy_create(struct tegra_xusb_padctl *padctl,
					      unsigned int index)
{
	struct tegra_xusb_utmi_phy *utmi;
	struct device_node *np;
	struct phy *phy;

	/*
	 * UTMI PHYs don't require additional properties, but if the PHY is
	 * marked as disabled there is no reason to register it.
	 */
	np = tegra_xusb_padctl_find_phy_node(padctl, "utmi", index);
	if (np && !of_device_is_available(np))
		return NULL;

	phy = devm_phy_create(padctl->dev, np, padctl->soc->utmi_phy_ops, NULL);
	if (IS_ERR(phy))
		return ERR_CAST(phy);

	utmi = &padctl->utmi_phys[index];
	phy_set_drvdata(phy, utmi);
	utmi->padctl = padctl;
	utmi->index = index;

	utmi->supply = devm_regulator_get(&phy->dev, "vbus");
	if (IS_ERR(utmi->supply))
		return ERR_CAST(utmi->supply);

	return phy;
}

static struct phy *tegra_xusb_hsic_phy_create(struct tegra_xusb_padctl *padctl,
					      unsigned int index)
{
	struct tegra_xusb_hsic_phy *hsic;
	struct device_node *np;
	struct phy *phy;

	/*
	 * HSIC PHYs don't require additional properties, but if the PHY is
	 * marked as disabled there is no reason to register it.
	 */
	np = tegra_xusb_padctl_find_phy_node(padctl, "hsic", index);
	if (np && !of_device_is_available(np))
		return NULL;

	phy = devm_phy_create(padctl->dev, np, padctl->soc->hsic_phy_ops, NULL);
	if (IS_ERR(phy))
		return ERR_CAST(phy);

	hsic = &padctl->hsic_phys[index];
	phy_set_drvdata(phy, hsic);
	hsic->padctl = padctl;
	hsic->index = index;

	return phy;
}

static int tegra_xusb_setup_usb(struct tegra_xusb_padctl *padctl)
{
	struct phy *phy;
	unsigned int i;

	for (i = 0; i < padctl->soc->num_usb3_phys; i++) {
		phy = tegra_xusb_usb3_phy_create(padctl, i);
		if (IS_ERR(phy))
			return PTR_ERR(phy);

		padctl->phys[TEGRA_XUSB_PADCTL_USB3_P0 + i] = phy;
	}

	for (i = 0; i < padctl->soc->num_utmi_phys; i++) {
		phy = tegra_xusb_utmi_phy_create(padctl, i);
		if (IS_ERR(phy))
			return PTR_ERR(phy);

		padctl->phys[TEGRA_XUSB_PADCTL_UTMI_P0 + i] = phy;
	}

	padctl->vddio_hsic = devm_regulator_get(padctl->dev, "vddio-hsic");
	if (IS_ERR(padctl->vddio_hsic))
		return PTR_ERR(padctl->vddio_hsic);

	for (i = 0; i < padctl->soc->num_hsic_phys; i++) {
		phy = tegra_xusb_hsic_phy_create(padctl, i);
		if (IS_ERR(phy))
			return PTR_ERR(phy);

		padctl->phys[TEGRA_XUSB_PADCTL_HSIC_P0 + i] = phy;
	}

	return 0;
}

static int tegra_xusb_padctl_probe(struct platform_device *pdev)
{
	struct tegra_xusb_padctl *padctl;
	const struct of_device_id *match;
	struct resource *res;
	struct phy *phy;
	int err;

	padctl = devm_kzalloc(&pdev->dev, sizeof(*padctl), GFP_KERNEL);
	if (!padctl)
		return -ENOMEM;

	platform_set_drvdata(pdev, padctl);
	mutex_init(&padctl->lock);
	padctl->dev = &pdev->dev;

	match = of_match_node(tegra_xusb_padctl_of_match, pdev->dev.of_node);
	padctl->soc = match->data;

	padctl->usb3_phys = devm_kcalloc(&pdev->dev, padctl->soc->num_usb3_phys,
					 sizeof(*padctl->usb3_phys),
					 GFP_KERNEL);
	if (!padctl->usb3_phys)
		return -ENOMEM;
	padctl->utmi_phys = devm_kcalloc(&pdev->dev, padctl->soc->num_utmi_phys,
					 sizeof(*padctl->utmi_phys),
					 GFP_KERNEL);
	if (!padctl->utmi_phys)
		return -ENOMEM;
	padctl->hsic_phys = devm_kcalloc(&pdev->dev, padctl->soc->num_hsic_phys,
					 sizeof(*padctl->hsic_phys),
					 GFP_KERNEL);
	if (!padctl->hsic_phys)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	padctl->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(padctl->regs))
		return PTR_ERR(padctl->regs);

	padctl->rst = devm_reset_control_get(&pdev->dev, NULL);
	if (IS_ERR(padctl->rst))
		return PTR_ERR(padctl->rst);

	err = reset_control_deassert(padctl->rst);
	if (err < 0)
		return err;

	err = padctl->soc->probe(padctl);
	if (err < 0)
		goto reset;

	memset(&padctl->desc, 0, sizeof(padctl->desc));
	padctl->desc.name = dev_name(padctl->dev);
	padctl->desc.pctlops = &tegra_xusb_padctl_pinctrl_ops;
	padctl->desc.pmxops = &tegra_xusb_padctl_pinmux_ops;
	padctl->desc.confops = &tegra_xusb_padctl_pinconf_ops;
	padctl->desc.owner = THIS_MODULE;

	padctl->pinctrl = pinctrl_register(&padctl->desc, &pdev->dev, padctl);
	if (!padctl->pinctrl) {
		dev_err(&pdev->dev, "failed to register pincontrol\n");
		err = -ENODEV;
		goto soc_remove;
	}

	phy = devm_phy_create(&pdev->dev, NULL, padctl->soc->pcie_phy_ops,
			      NULL);
	if (IS_ERR(phy)) {
		err = PTR_ERR(phy);
		goto unregister;
	}

	padctl->phys[TEGRA_XUSB_PADCTL_PCIE] = phy;
	phy_set_drvdata(phy, padctl);

	phy = devm_phy_create(&pdev->dev, NULL, padctl->soc->sata_phy_ops,
			      NULL);
	if (IS_ERR(phy)) {
		err = PTR_ERR(phy);
		goto unregister;
	}

	padctl->phys[TEGRA_XUSB_PADCTL_SATA] = phy;
	phy_set_drvdata(phy, padctl);

	INIT_WORK(&padctl->mbox_req_work, tegra_xusb_phy_mbox_work);
	padctl->mbox_client.dev = &pdev->dev;
	padctl->mbox_client.tx_block = true;
	padctl->mbox_client.tx_tout = 0;
	padctl->mbox_client.rx_callback = tegra_xusb_phy_mbox_rx;
	padctl->mbox_chan = mbox_request_channel(&padctl->mbox_client, 0);
	if (IS_ERR(padctl->mbox_chan)) {
		err = PTR_ERR(padctl->mbox_chan);
		if (err == -EPROBE_DEFER) {
			goto unregister;
		} else {
			dev_warn(&pdev->dev,
				 "failed to get mailbox, USB support disabled");
		}
	} else {
		err = tegra_xusb_setup_usb(padctl);
		if (err)
			goto unregister;
	}

	padctl->provider = devm_of_phy_provider_register(&pdev->dev,
							 tegra_xusb_padctl_xlate);
	if (IS_ERR(padctl->provider)) {
		err = PTR_ERR(padctl->provider);
		dev_err(&pdev->dev, "failed to register PHYs: %d\n", err);
		goto unregister;
	}

	return 0;

unregister:
	pinctrl_unregister(padctl->pinctrl);
soc_remove:
	padctl->soc->remove(padctl);
reset:
	reset_control_assert(padctl->rst);
	return err;
}

static int tegra_xusb_padctl_remove(struct platform_device *pdev)
{
	struct tegra_xusb_padctl *padctl = platform_get_drvdata(pdev);
	int err;

	if (!IS_ERR(padctl->mbox_chan)) {
		cancel_work_sync(&padctl->mbox_req_work);
		mbox_free_channel(padctl->mbox_chan);
	}

	pinctrl_unregister(padctl->pinctrl);

	padctl->soc->remove(padctl);

	err = reset_control_assert(padctl->rst);
	if (err < 0)
		dev_err(&pdev->dev, "failed to assert reset: %d\n", err);

	return err;
}

static struct platform_driver tegra_xusb_padctl_driver = {
	.driver = {
		.name = "tegra-xusb-padctl",
		.of_match_table = tegra_xusb_padctl_of_match,
	},
	.probe = tegra_xusb_padctl_probe,
	.remove = tegra_xusb_padctl_remove,
};
module_platform_driver(tegra_xusb_padctl_driver);

MODULE_AUTHOR("Thierry Reding <treding@nvidia.com>");
MODULE_DESCRIPTION("Tegra XUSB Pad Control driver");
MODULE_LICENSE("GPL v2");
