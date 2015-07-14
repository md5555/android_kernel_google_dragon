/*
 * Copyright (C) 2014 NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/sort.h>

#include <soc/tegra/fuse.h>

#include "mc.h"

#define MC_INTSTATUS 0x000
#define  MC_INT_DECERR_MTS (1 << 16)
#define  MC_INT_SECERR_SEC (1 << 13)
#define  MC_INT_DECERR_VPR (1 << 12)
#define  MC_INT_INVALID_APB_ASID_UPDATE (1 << 11)
#define  MC_INT_INVALID_SMMU_PAGE (1 << 10)
#define  MC_INT_ARBITRATION_EMEM (1 << 9)
#define  MC_INT_SECURITY_VIOLATION (1 << 8)
#define  MC_INT_DECERR_EMEM (1 << 6)

#define MC_INTMASK 0x004

#define MC_ERR_STATUS 0x08
#define  MC_ERR_STATUS_TYPE_SHIFT 28
#define  MC_ERR_STATUS_TYPE_INVALID_SMMU_PAGE (6 << MC_ERR_STATUS_TYPE_SHIFT)
#define  MC_ERR_STATUS_TYPE_MASK (0x7 << MC_ERR_STATUS_TYPE_SHIFT)
#define  MC_ERR_STATUS_READABLE (1 << 27)
#define  MC_ERR_STATUS_WRITABLE (1 << 26)
#define  MC_ERR_STATUS_NONSECURE (1 << 25)
#define  MC_ERR_STATUS_ADR_HI_SHIFT 20
#define  MC_ERR_STATUS_ADR_HI_MASK 0x3
#define  MC_ERR_STATUS_SECURITY (1 << 17)
#define  MC_ERR_STATUS_RW (1 << 16)
#ifdef CONFIG_ARCH_TEGRA_210_SOC
#define  MC_ERR_STATUS_CLIENT_MASK 0xff
#else
#define  MC_ERR_STATUS_CLIENT_MASK 0x7f
#endif

#define MC_ERR_ADR 0x0c

#define MC_EMEM_ARB_CFG 0x90
#define  MC_EMEM_ARB_CFG_CYCLES_PER_UPDATE(x)	(((x) & 0x1ff) << 0)
#define  MC_EMEM_ARB_CFG_CYCLES_PER_UPDATE_MASK	0x1ff
#define MC_EMEM_ARB_MISC0 0xd8

#define MC_EMEM_ADR_CFG 0x54
#define MC_EMEM_ADR_CFG_EMEM_NUMDEV BIT(0)

#define MC_EMEM_ARB_TIMING_W2R					0xc4
#define MC_EMEM_ARB_DA_TURNS					0xd0
#define MC_EMEM_ARB_MISC1					0xdc
#define MC_EMEM_ARB_RING3_THROTTLE				0xe4
#define MC_EMEM_ARB_OVERRIDE					0xe8
#define MC_RESERVED_RSV                                         0x3fc
#define MC_TIMING_CONTROL					0xfc

#define MC_SECURITY_CARVEOUT2_CFG0                              0xc58
#define MC_SECURITY_CARVEOUT2_BOM                               0xc5c
#define MC_SECURITY_CARVEOUT3_CFG0                              0xca8
#define MC_SECURITY_CARVEOUT3_BOM                               0xcac

#define MC_TIMING_REG_NUM\
	(((MC_EMEM_ARB_TIMING_W2R - MC_EMEM_ARB_CFG) / 4 + 1) + \
	((MC_EMEM_ARB_MISC1 - MC_EMEM_ARB_DA_TURNS) / 4 + 1))

static const struct of_device_id tegra_mc_of_match[] = {
#ifdef CONFIG_ARCH_TEGRA_3x_SOC
	{ .compatible = "nvidia,tegra30-mc", .data = &tegra30_mc_soc },
#endif
#ifdef CONFIG_ARCH_TEGRA_114_SOC
	{ .compatible = "nvidia,tegra114-mc", .data = &tegra114_mc_soc },
#endif
#ifdef CONFIG_ARCH_TEGRA_124_SOC
	{ .compatible = "nvidia,tegra124-mc", .data = &tegra124_mc_soc },
#endif
#ifdef CONFIG_ARCH_TEGRA_132_SOC
	{ .compatible = "nvidia,tegra132-mc", .data = &tegra132_mc_soc },
#endif
#ifdef CONFIG_ARCH_TEGRA_210_SOC
	{ .compatible = "nvidia,tegra210-mc", .data = &tegra210_mc_soc },
#endif
	{ }
};
MODULE_DEVICE_TABLE(of, tegra_mc_of_match);


#ifdef CONFIG_PM_SLEEP
static void tegra_mc_save(struct tegra_mc *mc)
{
	u32 off;
	u32 *buf = mc->reg_buf;
	u32 i;

	for (off = MC_EMEM_ARB_CFG; off <= MC_EMEM_ARB_TIMING_W2R; off += 4)
		*buf++ = mc_readl(mc, off);

	for (off = MC_EMEM_ARB_DA_TURNS; off <= MC_EMEM_ARB_MISC1; off += 4)
		*buf++ = mc_readl(mc, off);

	*buf++ = mc_readl(mc, MC_EMEM_ARB_RING3_THROTTLE);
	*buf++ = mc_readl(mc, MC_EMEM_ARB_OVERRIDE);
	*buf++ = mc_readl(mc, MC_RESERVED_RSV);

	for (i = 0; i < mc->soc->num_clients; i++) {
		const struct tegra_mc_la *la = &mc->soc->clients[i].la;
		*buf++ = mc_readl(mc, la->reg);
	}

	*buf++ = mc_readl(mc, MC_INTMASK);

	/* Save WPR registers for GPU */
	*buf++ = mc_readl(mc, MC_SECURITY_CARVEOUT2_BOM);
	*buf++ = mc_readl(mc, MC_SECURITY_CARVEOUT3_BOM);
	*buf++ = mc_readl(mc, MC_SECURITY_CARVEOUT2_CFG0);
	*buf++ = mc_readl(mc, MC_SECURITY_CARVEOUT3_CFG0);
}

static void tegra_mc_restore(struct tegra_mc *mc)
{
	u32 off;
	u32 *buf = mc->reg_buf;
	u32 i;

	for (off = MC_EMEM_ARB_CFG; off <= MC_EMEM_ARB_TIMING_W2R; off += 4)
		mc_writel(mc, *buf++, off);

	for (off = MC_EMEM_ARB_DA_TURNS; off <= MC_EMEM_ARB_MISC1; off += 4)
		mc_writel(mc, *buf++, off);

	mc_writel(mc, *buf++, MC_EMEM_ARB_RING3_THROTTLE);
	mc_writel(mc, *buf++, MC_EMEM_ARB_OVERRIDE);
	mc_writel(mc, *buf++, MC_RESERVED_RSV);

	for (i = 0; i < mc->soc->num_clients; i++) {
		const struct tegra_mc_la *la = &mc->soc->clients[i].la;
		mc_writel(mc, *buf++, la->reg);
	}

	mc_writel(mc, *buf++, MC_INTMASK);
	off = mc_readl(mc, MC_INTMASK);

	/* Restore WPR registers for GPU */
	mc_writel(mc, *buf++, MC_SECURITY_CARVEOUT2_BOM);
	mc_writel(mc, *buf++, MC_SECURITY_CARVEOUT3_BOM);
	mc_writel(mc, *buf++, MC_SECURITY_CARVEOUT2_CFG0);
	mc_writel(mc, *buf++, MC_SECURITY_CARVEOUT3_CFG0);

	mc_writel(mc, 0x1, MC_TIMING_CONTROL);
	off = mc_readl(mc, MC_TIMING_CONTROL);
}
#endif

const struct tegra_mc_flush *tegra_mc_flush_get(struct tegra_mc *mc,
						unsigned int swgroup)
{
	const struct tegra_mc_flush *flush = NULL;
	int i;

	mutex_lock(&mc->lock);

	for (i = 0; i < mc->soc->num_flushes; i++) {
		if (mc->soc->flushes[i].swgroup == swgroup) {
			if (mc->flush_reserved[i] == false) {
				mc->flush_reserved[i] = true;
				flush = &mc->soc->flushes[i];
			}
			break;
		}
	}

	mutex_unlock(&mc->lock);

	return flush;
}
EXPORT_SYMBOL(tegra_mc_flush_get);

void tegra_mc_flush_put(struct tegra_mc *mc, unsigned int swgroup)
{
	int i;

	mutex_lock(&mc->lock);

	for (i = 0; i < mc->soc->num_flushes; i++) {
		if (mc->soc->flushes[i].swgroup == swgroup) {
			mc->flush_reserved[i] = false;
			break;
		}
	}

	mutex_unlock(&mc->lock);
}
EXPORT_SYMBOL(tegra_mc_flush_put);

/*
 * Must be called with mc->lock held
 */
static bool tegra_mc_flush_done(struct tegra_mc *mc,
				const struct tegra_mc_flush *flush,
				bool unstable)
{
	u32 val;
	int i;

	WARN_ON(!mutex_is_locked(&mc->lock));

	/*
	 * There might be a glitch seen with the status register if we program
	 * the control register and then read the status register in a short
	 * window (on the order of 5 cycles) due to a HW bug. So here we poll
	 * for a stable status read.
	 */
	val = mc_readl(mc, flush->status);

	if (unstable) {
		for (i = 0; i < 5; i++) {
			if (mc_readl(mc, flush->status) != val)
				return false;
		}
	}

	return val & BIT(flush->bit);
}

int tegra_mc_flush(struct tegra_mc *mc, const struct tegra_mc_flush *flush,
		   bool enable)
{
	u32 val;

	if (!mc || !flush)
		return -EINVAL;

	mutex_lock(&mc->lock);

	val = mc_readl(mc, flush->ctrl);

	if (enable)
		val |= BIT(flush->bit);
	else
		val &= ~BIT(flush->bit);

	mc_writel(mc, val, flush->ctrl);
	mc_readl(mc, flush->ctrl);

	/*
	 * If activating the flush, poll the
	 * status register until the flush is done.
	 */
	if (enable) {
		do {
			udelay(10);
		} while (!tegra_mc_flush_done(mc, flush,
					     mc->soc->flush_unstable));
	}

	mutex_unlock(&mc->lock);

	dev_dbg(mc->dev, "%s bit %d\n", __func__, flush->bit);

	return 0;
}
EXPORT_SYMBOL(tegra_mc_flush);

void la_writel(struct tegra_mc *mc, u32 val, struct tegra_mc_la *la)
{
	u32 reg_value;

	BUG_ON(val > MC_LA_MAX_VALUE);

	reg_value = mc_readl(mc, la->reg);

	reg_value &= ~(la->mask << la->shift);
	reg_value |= (val & la->mask) << la->shift;

	mc_writel(mc, reg_value, la->reg);

	la->la_set = val;
}

int tegra_la_set_disp_la(struct tegra_mc *mc,
			 int id,
			 unsigned long emc_freq_hz,
			 unsigned int bw_kbps,
			 struct tegra_dc_to_la_params disp_params)
{
	struct tegra_la_soc *la_soc = mc->soc->la_soc;

	return la_soc->set_disp_la(mc, id, emc_freq_hz, bw_kbps, disp_params);
}
EXPORT_SYMBOL(tegra_la_set_disp_la);

int tegra_la_set_camera_ptsa(struct tegra_mc *mc,
			     int id,
			     unsigned int bw_kbps,
			     int is_hiso)
{
	struct tegra_la_soc *la_soc = mc->soc->la_soc;

	return la_soc->update_camera_ptsa_rate(mc, id, bw_kbps, is_hiso);
}
EXPORT_SYMBOL(tegra_la_set_camera_ptsa);

struct tegra_disp_client *tegra_la_get_disp_client_info(struct tegra_mc *mc,
							int id)
{
	struct tegra_la_soc *la_soc = mc->soc->la_soc;
	int i;

	for (i = 0; i < la_soc->num_clients; i++) {
		if (la_soc->clients[i].id == id)
			break;
	}

	if (i == la_soc->num_clients)
		return ERR_PTR(-EINVAL);

	return &la_soc->clients[i].la.disp_client;
}
EXPORT_SYMBOL(tegra_la_get_disp_client_info);

static int tegra_mc_setup_latency_allowance(struct tegra_mc *mc)
{
	struct tegra_la_soc *la_soc = mc->soc->la_soc;
	unsigned long long tick;
	unsigned int i;
	u32 value;

	if (la_soc->init_la)
		la_soc->init_la(mc);

	/* compute the number of MC clock cycles per tick */
	tick = mc->tick * clk_get_rate(mc->clk);
	do_div(tick, NSEC_PER_SEC);

	value = mc_readl(mc, MC_EMEM_ARB_CFG);
	value &= ~MC_EMEM_ARB_CFG_CYCLES_PER_UPDATE_MASK;
	value |= MC_EMEM_ARB_CFG_CYCLES_PER_UPDATE(tick);
	mc_writel(mc, value, MC_EMEM_ARB_CFG);

	/* write latency allowance defaults */
	for (i = 0; i < mc->soc->num_clients; i++) {
		struct tegra_mc_la *la = &mc->soc->la_soc->clients[i].la;
		unsigned int emc_mhz;

		if (!la->reg)
			continue;

		if (la->la_ref_clk_mhz) {
			emc_mhz = clk_get_rate(mc->emc_clk) / 1000000;
			if (la->la_ref_clk_mhz > emc_mhz) {
				value = min_t(unsigned int,
					      la->def *
					      la->la_ref_clk_mhz /
					      emc_mhz,
					      MC_LA_MAX_VALUE);
			} else {
				value = min_t(unsigned int,
					      la->def,
					      MC_LA_MAX_VALUE);
			}
		} else {
			value = la->def;
		}

		la_writel(mc, value, la);
	}

	if (la_soc->init_ptsa)
		la_soc->init_ptsa(mc);

	return 0;
}

void tegra_mc_write_emem_configuration(struct tegra_mc *mc, unsigned long rate)
{
	unsigned int i;
	struct tegra_mc_timing *timing = NULL;

	for (i = 0; i < mc->num_timings; i++) {
		if (mc->timings[i].rate == rate) {
			timing = &mc->timings[i];
			break;
		}
	}

	if (!timing) {
		dev_err(mc->dev, "no memory timing registered for rate %lu\n",
			rate);
		return;
	}

	for (i = 0; i < mc->soc->num_emem_regs; ++i)
		mc_writel(mc, timing->emem_data[i], mc->soc->emem_regs[i]);
}

unsigned int tegra_mc_get_emem_device_count(struct tegra_mc *mc)
{
	u8 dram_count;

	dram_count = mc_readl(mc, MC_EMEM_ADR_CFG);
	dram_count &= MC_EMEM_ADR_CFG_EMEM_NUMDEV;
	dram_count++;

	return dram_count;
}

static int load_one_timing(struct tegra_mc *mc,
			   struct tegra_mc_timing *timing,
			   struct device_node *node)
{
	int err;
	u32 tmp;

	err = of_property_read_u32(node, "clock-frequency", &tmp);
	if (err) {
		dev_err(mc->dev,
			"timing %s: failed to read rate\n", node->name);
		return err;
	}

	timing->rate = tmp;
	timing->emem_data = devm_kcalloc(mc->dev, mc->soc->num_emem_regs,
					 sizeof(u32), GFP_KERNEL);
	if (!timing->emem_data)
		return -ENOMEM;

	err = of_property_read_u32_array(node, "nvidia,emem-configuration",
					 timing->emem_data,
					 mc->soc->num_emem_regs);
	if (err) {
		dev_err(mc->dev,
			"timing %s: failed to read EMEM configuration\n",
			node->name);
		return err;
	}

	return 0;
}

static int load_timings(struct tegra_mc *mc, struct device_node *node)
{
	struct device_node *child;
	struct tegra_mc_timing *timing;
	int child_count = of_get_child_count(node);
	int i = 0, err;

	mc->timings = devm_kcalloc(mc->dev, child_count, sizeof(*timing),
				   GFP_KERNEL);
	if (!mc->timings)
		return -ENOMEM;

	mc->num_timings = child_count;

	for_each_child_of_node(node, child) {
		timing = &mc->timings[i++];

		err = load_one_timing(mc, timing, child);
		if (err)
			return err;
	}

	return 0;
}

static int tegra_mc_setup_timings(struct tegra_mc *mc)
{
	struct device_node *node;
	u32 ram_code, node_ram_code;
	int err;

	ram_code = tegra_read_ram_code();

	mc->num_timings = 0;

	for_each_child_of_node(mc->dev->of_node, node) {
		err = of_property_read_u32(node, "nvidia,ram-code",
					   &node_ram_code);
		if (err || (node_ram_code != ram_code)) {
			of_node_put(node);
			continue;
		}

		err = load_timings(mc, node);
		if (err)
			return err;
		of_node_put(node);
		break;
	}

	if (mc->num_timings == 0)
		dev_warn(mc->dev,
			 "no memory timings for RAM code %u registered\n",
			 ram_code);

	return 0;
}

static const char *const status_names[32] = {
	[ 1] = "External interrupt",
	[ 6] = "EMEM address decode error",
	[ 8] = "Security violation",
	[ 9] = "EMEM arbitration error",
	[10] = "Page fault",
	[11] = "Invalid APB ASID update",
	[12] = "VPR violation",
	[13] = "Secure carveout violation",
	[16] = "MTS carveout violation",
};

static const char *const error_names[8] = {
	[2] = "EMEM decode error",
	[3] = "TrustZone violation",
	[4] = "Carveout violation",
	[6] = "SMMU translation error",
};

static irqreturn_t tegra_mc_irq(int irq, void *data)
{
	struct tegra_mc *mc = data;
	unsigned long status, mask;
	unsigned int bit;

	/* mask all interrupts to avoid flooding */
	status = mc_readl(mc, MC_INTSTATUS);
	mask = mc_readl(mc, MC_INTMASK);

	for_each_set_bit(bit, &status, 32) {
		const char *error = status_names[bit] ?: "unknown";
		const char *client = "unknown", *desc;
		const char *direction, *secure;
		phys_addr_t addr = 0;
		unsigned int i;
		char perm[7];
		u8 id, type;
		u32 value;

		value = mc_readl(mc, MC_ERR_STATUS);

#ifdef CONFIG_PHYS_ADDR_T_64BIT
		if (mc->soc->num_address_bits > 32) {
			addr = ((value >> MC_ERR_STATUS_ADR_HI_SHIFT) &
				MC_ERR_STATUS_ADR_HI_MASK);
			addr <<= 32;
		}
#endif

		if (value & MC_ERR_STATUS_RW)
			direction = "write";
		else
			direction = "read";

		if (value & MC_ERR_STATUS_SECURITY)
			secure = "secure ";
		else
			secure = "";

		id = value & MC_ERR_STATUS_CLIENT_MASK;

		for (i = 0; i < mc->soc->num_clients; i++) {
			if (mc->soc->clients[i].id == id) {
				client = mc->soc->clients[i].name;
				break;
			}
		}

		type = (value & MC_ERR_STATUS_TYPE_MASK) >>
		       MC_ERR_STATUS_TYPE_SHIFT;
		desc = error_names[type];

		switch (value & MC_ERR_STATUS_TYPE_MASK) {
		case MC_ERR_STATUS_TYPE_INVALID_SMMU_PAGE:
			perm[0] = ' ';
			perm[1] = '[';

			if (value & MC_ERR_STATUS_READABLE)
				perm[2] = 'R';
			else
				perm[2] = '-';

			if (value & MC_ERR_STATUS_WRITABLE)
				perm[3] = 'W';
			else
				perm[3] = '-';

			if (value & MC_ERR_STATUS_NONSECURE)
				perm[4] = '-';
			else
				perm[4] = 'S';

			perm[5] = ']';
			perm[6] = '\0';
			break;

		default:
			perm[0] = '\0';
			break;
		}

		value = mc_readl(mc, MC_ERR_ADR);
		addr |= value;

		dev_err_ratelimited(mc->dev, "%s: %s%s @%pa: %s (%s%s)\n",
				    client, secure, direction, &addr, error,
				    desc, perm);
	}

	/* clear interrupts */
	mc_writel(mc, status, MC_INTSTATUS);

	return IRQ_HANDLED;
}

static int tegra_mc_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct resource *res;
	struct tegra_mc *mc;
	u32 value;
	int err;

	match = of_match_node(tegra_mc_of_match, pdev->dev.of_node);
	if (!match)
		return -ENODEV;

	mc = devm_kzalloc(&pdev->dev, sizeof(*mc), GFP_KERNEL);
	if (!mc)
		return -ENOMEM;

	platform_set_drvdata(pdev, mc);
	mc->soc = match->data;
	mc->dev = &pdev->dev;

	mc->flush_reserved = devm_kcalloc(&pdev->dev, mc->soc->num_flushes,
					  sizeof(mc->flush_reserved),
					  GFP_KERNEL);
	if (!mc->flush_reserved)
		return -ENOMEM;

	/* length of MC tick in nanoseconds */
	mc->tick = 30;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	mc->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(mc->regs))
		return PTR_ERR(mc->regs);

	mc->clk = devm_clk_get(&pdev->dev, "mc");
	if (IS_ERR(mc->clk)) {
		dev_err(&pdev->dev, "failed to get MC clock: %ld\n",
			PTR_ERR(mc->clk));
		return PTR_ERR(mc->clk);
	}

	mc->emc_clk = devm_clk_get(&pdev->dev, "emc");
	if (IS_ERR(mc->emc_clk)) {
		dev_err(&pdev->dev, "failed to get EMC clock: %ld\n",
			PTR_ERR(mc->emc_clk));
		return PTR_ERR(mc->emc_clk);
	}

	err = tegra_mc_setup_latency_allowance(mc);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to setup latency allowance: %d\n",
			err);
		return err;
	}

	err = tegra_mc_setup_timings(mc);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to setup timings: %d\n", err);
		return err;
	}

	if (IS_ENABLED(CONFIG_TEGRA_IOMMU_SMMU)) {
		mc->smmu = tegra_smmu_probe(&pdev->dev, mc->soc->smmu, mc);
		if (IS_ERR(mc->smmu)) {
			dev_err(&pdev->dev, "failed to probe SMMU: %ld\n",
				PTR_ERR(mc->smmu));
			return PTR_ERR(mc->smmu);
		}
	}

	mc->irq = platform_get_irq(pdev, 0);
	if (mc->irq < 0) {
		dev_err(&pdev->dev, "interrupt not specified\n");
		return mc->irq;
	}

	err = devm_request_irq(&pdev->dev, mc->irq, tegra_mc_irq, IRQF_SHARED,
			       dev_name(&pdev->dev), mc);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to request IRQ#%u: %d\n", mc->irq,
			err);
		return err;
	}

	mutex_init(&mc->lock);

	value = MC_INT_DECERR_MTS | MC_INT_SECERR_SEC | MC_INT_DECERR_VPR |
		MC_INT_INVALID_APB_ASID_UPDATE | MC_INT_INVALID_SMMU_PAGE |
		MC_INT_SECURITY_VIOLATION | MC_INT_DECERR_EMEM;

	mc_writel(mc, value, MC_INTMASK);

#ifdef CONFIG_PM_SLEEP
	/* Allocate the memory for saving registers */
	mc->reg_buf = devm_kcalloc(mc->dev,
		mc->soc->num_clients + MC_TIMING_REG_NUM + 4,
		sizeof(u32), GFP_KERNEL);
	if (!mc->reg_buf)
		return -ENOMEM;
#endif
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int
tegra_mc_resume_early(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct tegra_mc *mc = platform_get_drvdata(pdev);

	tegra_mc_restore(mc);

	if (IS_ENABLED(CONFIG_TEGRA_IOMMU_SMMU))
		tegra_smmu_resume(mc->smmu);

	mc->soc->la_soc->la_resume(mc);

	return 0;
}

static int
tegra_mc_suspend_late(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct tegra_mc *mc = platform_get_drvdata(pdev);

	mc->soc->la_soc->la_suspend(mc);

	if (IS_ENABLED(CONFIG_TEGRA_IOMMU_SMMU))
		tegra_smmu_suspend(mc->smmu);

	tegra_mc_save(mc);

	return 0;
}

static const struct dev_pm_ops tegra_mc_pm_ops = {
	.resume_early = tegra_mc_resume_early,
	.suspend_late = tegra_mc_suspend_late,
};
#endif

static struct platform_driver tegra_mc_driver = {
	.driver = {
		.name = "tegra-mc",
		.of_match_table = tegra_mc_of_match,
		.suppress_bind_attrs = true,
	},
	.prevent_deferred_probe = true,
	.probe = tegra_mc_probe,
#ifdef CONFIG_PM_SLEEP
	.driver.pm = &tegra_mc_pm_ops,
#endif
};

static int tegra_mc_init(void)
{
	return platform_driver_register(&tegra_mc_driver);
}
arch_initcall(tegra_mc_init);

MODULE_AUTHOR("Thierry Reding <treding@nvidia.com>");
MODULE_DESCRIPTION("NVIDIA Tegra Memory Controller driver");
MODULE_LICENSE("GPL v2");
