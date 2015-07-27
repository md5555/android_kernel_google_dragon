/*
 * Copyright (C) 2014 NVIDIA Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __SOC_TEGRA_MC_H__
#define __SOC_TEGRA_MC_H__

#include <linux/types.h>

struct clk;
struct device;
struct page;

struct tegra_smmu_enable {
	unsigned int reg;
	unsigned int bit;
};

struct tegra_mc_timing {
	unsigned long rate;

	u32 *emem_data;
};

/* latency allowance */
struct tegra_mc_la {
	unsigned int reg;
	unsigned int shift;
	unsigned int mask;
	unsigned int def;
};

struct tegra_mc_client {
	unsigned int id;
	const char *name;
	unsigned int swgroup;

	unsigned int fifo_size;

	struct tegra_smmu_enable smmu;
	struct tegra_mc_la la;
};

struct tegra_mc_flush {
	unsigned int swgroup;
	unsigned int ctrl;
	unsigned int status;
	unsigned int bit;
};

struct tegra_smmu_swgroup {
	const char *name;
	unsigned int swgroup;
	unsigned int reg;
};

struct tegra_smmu_soc {
	const struct tegra_mc_client *clients;
	unsigned int num_clients;

	const struct tegra_smmu_swgroup *swgroups;
	unsigned int num_swgroups;

	bool supports_round_robin_arbitration;
	bool supports_request_limit;

	unsigned int num_asids;
};

struct tegra_mc;
struct tegra_smmu;

#ifdef CONFIG_TEGRA_IOMMU_SMMU
struct tegra_smmu *tegra_smmu_probe(struct device *dev,
				    const struct tegra_smmu_soc *soc,
				    struct tegra_mc *mc);
void tegra_smmu_remove(struct tegra_smmu *smmu);
void tegra_smmu_suspend(struct tegra_smmu *smmu);
void tegra_smmu_resume(struct tegra_smmu *smmu);
#else
static inline struct tegra_smmu *
tegra_smmu_probe(struct device *dev, const struct tegra_smmu_soc *soc,
		 struct tegra_mc *mc)
{
	return NULL;
}

static inline void tegra_smmu_remove(struct tegra_smmu *smmu)
{
}
static inline void tegra_smmu_suspend(struct tegra_smmu *smmu)
{
}
static inline void tegra_smmu_resume(struct tegra_smmu *smmu)
{
}
#endif

struct tegra_mc_soc {
	const struct tegra_mc_client *clients;
	unsigned int num_clients;

	const bool flush_unstable;
	const struct tegra_mc_flush *flushes;
	unsigned int num_flushes;

	const unsigned long *emem_regs;
	unsigned int num_emem_regs;

	unsigned int num_address_bits;
	unsigned int atom_size;

	const struct tegra_smmu_soc *smmu;
};

struct tegra_mc {
	struct device *dev;
	struct tegra_smmu *smmu;
	void __iomem *regs;
	struct clk *clk;
	int irq;

	const struct tegra_mc_soc *soc;
	unsigned long tick;

	struct tegra_mc_timing *timings;
	unsigned int num_timings;

	bool *flush_reserved;

	struct mutex lock;

	u32 *reg_buf;
};

void tegra_mc_write_emem_configuration(struct tegra_mc *mc, unsigned long rate);
unsigned int tegra_mc_get_emem_device_count(struct tegra_mc *mc);
const struct tegra_mc_flush *tegra_mc_flush_get(struct tegra_mc *mc,
						  unsigned int swgroup);
void tegra_mc_flush_put(struct tegra_mc *mc, unsigned int swgroup);
int tegra_mc_flush(struct tegra_mc *mc, const struct tegra_mc_flush *s,
	bool enable);

#endif /* __SOC_TEGRA_MC_H__ */
