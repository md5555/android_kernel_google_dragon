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

#define MC_LA_MAX_VALUE					255

#define LA_ST_LA_MINUS_SNAP_ARB_TO_ROW_SRT_EMCCLKS_FP	70000
#define LA_DRAM_WIDTH_BITS				64
#define LA_DISP_CATCHUP_FACTOR_FP			1100

#define LA_BW_DISRUPTION_TIME_EMCCLKS_FP		1342000
#define LA_STATIC_LA_SNAP_ARB_TO_ROW_SRT_EMCCLKS_FP	54000
#define LA_CONS_MEM_EFFICIENCY_FP			500
#define LA_ROW_SRT_SZ_BYTES		       (64 * (LA_MC_EMEM_NUM_SLOTS + 1))
#define LA_MC_EMEM_NUM_SLOTS				63
#define LA_MAX_DRAIN_TIME_USEC				10

#define LA_FP_FACTOR					1000
#define LA_REAL_TO_FP(val)				((val) * LA_FP_FACTOR)
#define LA_FP_TO_REAL(val)				((val) / LA_FP_FACTOR)

struct tegra_smmu_enable {
	unsigned int reg;
	unsigned int bit;
};

struct tegra_mc_timing {
	unsigned long rate;

	u32 *emem_data;
};

enum tegra_la_client_type {
	TEGRA_LA_CLIENT_TYPE_NORMAL,
	TEGRA_LA_CLIENT_TYPE_DISPLAY,
	TEGRA_LA_CLIENT_TYPE_BBC,
	TEGRA_LA_CLIENT_TYPE_CAMERA,
	TEGRA_LA_CLIENT_TYPE_NUM,
};

enum tegra_disp_win_type {
	TEGRA_LA_DISP_WIN_TYPE_FULL,
	TEGRA_LA_DISP_WIN_TYPE_FULLA,
	TEGRA_LA_DISP_WIN_TYPE_FULLB,
	TEGRA_LA_DISP_WIN_TYPE_SIMPLE,
	TEGRA_LA_DISP_WIN_TYPE_CURSOR,
	TEGRA_LA_DISP_WIN_TYPE_NUM_TYPES
};

struct tegra_disp_client {
	enum tegra_disp_win_type win_type;
	unsigned int mccif_size_bytes;
	unsigned int line_buf_sz_bytes;
};

enum tegra_agg_camera_client_id {
	TEGRA_LA_AGG_CAMERA_VE = 0,
	TEGRA_LA_AGG_CAMERA_VE2,
	TEGRA_LA_AGG_CAMERA_ISP,
	TEGRA_LA_AGG_CAMERA_NUM_CLIENTS
};

struct tegra_agg_camera_client {
	unsigned int bw_fp;
	unsigned int frac_fp;
	unsigned int ptsa_min;
	unsigned int ptsa_max;
	bool is_hiso;
};

struct tegra_dc_to_la_params {
	unsigned int thresh_lwm_bytes;
	unsigned int spool_up_buffering_adj_bytes;
	unsigned int drain_time_usec_fp;
	unsigned int total_dc0_bw;
	unsigned int total_dc1_bw;
};

/* latency allowance */
struct tegra_mc_la {
	unsigned int reg;
	unsigned int shift;
	unsigned int mask;
	unsigned int def;

	unsigned int type;

	unsigned int fifo_size_in_atoms;
	unsigned int expiration_in_ns;
	bool scaling_support;
	unsigned int la_set;
	unsigned int la_ref_clk_mhz;

	unsigned int disp_bw;
	struct tegra_disp_client disp_client;
	unsigned int bbc_bw;
	unsigned int camera_bw;
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

	unsigned int num_tlb_lines;
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

struct tegra_ptsa_info {
	unsigned int dis_ptsa_rate;
	unsigned int dis_ptsa_min;
	unsigned int dis_ptsa_max;
	unsigned int disb_ptsa_rate;
	unsigned int disb_ptsa_min;
	unsigned int disb_ptsa_max;
	unsigned int ve_ptsa_rate;
	unsigned int ve_ptsa_min;
	unsigned int ve_ptsa_max;
	unsigned int ve2_ptsa_rate;
	unsigned int ve2_ptsa_min;
	unsigned int ve2_ptsa_max;
	unsigned int ring2_ptsa_rate;
	unsigned int ring2_ptsa_min;
	unsigned int ring2_ptsa_max;
	unsigned int bbc_ptsa_rate;
	unsigned int bbc_ptsa_min;
	unsigned int bbc_ptsa_max;
	unsigned int mpcorer_ptsa_rate;
	unsigned int mpcorer_ptsa_min;
	unsigned int mpcorer_ptsa_max;
	unsigned int ftop_ptsa_min;
	unsigned int ftop_ptsa_max;
	unsigned int ftop_ptsa_rate;
	unsigned int smmu_ptsa_rate;
	unsigned int smmu_ptsa_min;
	unsigned int smmu_ptsa_max;
	unsigned int ring1_ptsa_rate;
	unsigned int ring1_ptsa_min;
	unsigned int ring1_ptsa_max;

	unsigned int ptsa_grant_dec;
	unsigned int bbcll_earb_cfg;

	unsigned int isp_ptsa_rate;
	unsigned int isp_ptsa_min;
	unsigned int isp_ptsa_max;
	unsigned int a9avppc_ptsa_min;
	unsigned int a9avppc_ptsa_max;
	unsigned int avp_ptsa_min;
	unsigned int avp_ptsa_max;
	unsigned int mse_ptsa_min;
	unsigned int mse_ptsa_max;
	unsigned int gk_ptsa_min;
	unsigned int gk_ptsa_max;
	unsigned int vicpc_ptsa_min;
	unsigned int vicpc_ptsa_max;
	unsigned int apb_ptsa_min;
	unsigned int apb_ptsa_max;
	unsigned int pcx_ptsa_min;
	unsigned int pcx_ptsa_max;
	unsigned int host_ptsa_min;
	unsigned int host_ptsa_max;
	unsigned int ahb_ptsa_min;
	unsigned int ahb_ptsa_max;
	unsigned int sax_ptsa_min;
	unsigned int sax_ptsa_max;
	unsigned int aud_ptsa_min;
	unsigned int aud_ptsa_max;
	unsigned int sd_ptsa_min;
	unsigned int sd_ptsa_max;
	unsigned int usbx_ptsa_min;
	unsigned int usbx_ptsa_max;
	unsigned int usbd_ptsa_min;
	unsigned int usbd_ptsa_max;

	unsigned int r0_dis_ptsa_min;
	unsigned int r0_dis_ptsa_max;
	unsigned int r0_disb_ptsa_min;
	unsigned int r0_disb_ptsa_max;
	unsigned int vd_ptsa_min;
	unsigned int vd_ptsa_max;

	unsigned int jpg_ptsa_min;
	unsigned int jpg_ptsa_max;
	unsigned int gk2_ptsa_min;
	unsigned int gk2_ptsa_max;
};

struct tegra_la_soc {
	struct tegra_mc_client *clients;
	unsigned int num_clients;

	unsigned int ns_per_tick;
	unsigned int la_max_value;
	unsigned int total_dc0_bw;
	unsigned int total_dc1_bw;
	unsigned int emc_min_freq_mhz_fp;
	unsigned int emc_min_freq_mhz;
	unsigned int emc_max_freq_mhz;
	unsigned int hi_gd_fp;
	unsigned int lo_gd_fp;
	unsigned int hi_gd_fpa;
	unsigned int lo_gd_fpa;
	unsigned int low_freq_bw;
	unsigned int dda_div;
	struct tegra_agg_camera_client agg_camera_client[TEGRA_LA_AGG_CAMERA_NUM_CLIENTS];
	struct tegra_ptsa_info *ptsa_info;
	bool disable_la;
	bool disable_ptsa;
	bool disable_disp_ptsa;
	bool disable_bbc_ptsa;
	int (*init_la)(struct tegra_mc *mc);
	void (*init_ptsa)(struct tegra_mc *mc);
	int (*update_camera_ptsa_rate)(struct tegra_mc *mc, int id,
				       unsigned int bw_kbps,
				       int is_hiso);
	int (*set_disp_la)(struct tegra_mc *mc, int id,
			   unsigned long emc_freq_hz,
			   unsigned int bw_kbps,
			   struct tegra_dc_to_la_params disp_params);
	void (*la_suspend)(struct tegra_mc *mc);
	void (*la_resume)(struct tegra_mc *mc);

	struct mutex la_ptsa_lock;
};

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
	struct tegra_la_soc *la_soc;
};

struct tegra_mc {
	struct device *dev;
	struct tegra_smmu *smmu;
	void __iomem *regs;
	void __iomem *emc_base;
	struct clk *clk;
	struct clk *emc_clk;
	int irq;
	int dram_type;

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

int tegra_la_set_disp_la(struct tegra_mc *mc, int id,
			 unsigned long emc_freq_hz,
			 unsigned int bw_kbps,
			 struct tegra_dc_to_la_params disp_params);
int tegra_la_set_camera_ptsa(struct tegra_mc *mc, int id, unsigned int bw_kbps,
			     int is_hiso);
struct tegra_disp_client *tegra_la_get_disp_client_info(struct tegra_mc *mc,
							int id);

#endif /* __SOC_TEGRA_MC_H__ */
