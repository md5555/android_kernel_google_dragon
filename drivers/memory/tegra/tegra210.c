/*
 * Copyright (C) 2015 NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/mm.h>

#include <dt-bindings/memory/tegra210-mc.h>

#include <soc/tegra/tegra_emc.h>

#include "mc.h"
#include "tegra210-mc-reg.h"

#define EMC_FBIO_CFG5					0x104
#define EMC_FBIO_CFG5_DRAM_TYPE_SHIFT			0x0
#define EMC_FBIO_CFG5_DRAM_TYPE_MASK			0x3
#define MC_EMEM_ARB_MISC0_MC_EMC_SAME_FREQ_BIT		(1 << 27)

#define MC_PTSA_MIN_DEFAULT_MASK			0x3f
#define MC_PTSA_MAX_DEFAULT_MASK			0x3f
#define MC_PTSA_RATE_DEFAULT_MASK			0xfff

#define MC_MAX_FREQ_MHZ					533
#define MAX_GRANT_DEC					511

#define EXP_TIME_EMCCLKS_FP				88000
#define MAX_LA_NSEC					7650
#define DDA_BW_MARGIN_FP				1100
#define EMEM_PTSA_MINMAX_WIDTH				5
#define ONE_DDA_FRAC_FP					10
#define EMEM_PTSA_RATE_WIDTH				12
#define MAX_DDA_RATE					0xfff
#define CPU_RD_BW_PERC					9
#define CPU_WR_BW_PERC					1

#define LA_USEC_TO_NSEC_FACTOR		1000
#define LA_HZ_TO_MHZ_FACTOR		1000000
#define LA_ADDITIONAL_FP_FACTOR		10
#define LA_FP_TO_FPA(val)		((val) * LA_ADDITIONAL_FP_FACTOR)
#define LA_FPA_TO_FP(val)		((val) / LA_ADDITIONAL_FP_FACTOR)
#define LA_FPA_TO_REAL(val)		((val) / LA_FP_FACTOR /		\
					 LA_ADDITIONAL_FP_FACTOR)
#define LA_REAL_TO_FPA(val)		((val) * LA_FP_FACTOR *	\
					 LA_ADDITIONAL_FP_FACTOR)

enum {
	TEGRA_CLIENT_PTCR_IDX,
	TEGRA_CLIENT_DISPLAY0A_IDX,
	TEGRA_CLIENT_DISPLAY0AB_IDX,
	TEGRA_CLIENT_DISPLAY0B_IDX,
	TEGRA_CLIENT_DISPLAY0BB_IDX,
	TEGRA_CLIENT_DISPLAY0C_IDX,
	TEGRA_CLIENT_DISPLAY0CB_IDX,
	TEGRA_CLIENT_AFIR_IDX,
	TEGRA_CLIENT_AVPCARM7R_IDX,
	TEGRA_CLIENT_DISPLAYHC_IDX,
	TEGRA_CLIENT_DISPLAYHCB_IDX,
	TEGRA_CLIENT_HDAR_IDX,
	TEGRA_CLIENT_HOST1XDMAR_IDX,
	TEGRA_CLIENT_HOST1XR_IDX,
	TEGRA_CLIENT_NVENCSRD_IDX,
	TEGRA_CLIENT_PPCSAHBDMAR_IDX,
	TEGRA_CLIENT_PPCSAHBSLVR_IDX,
	TEGRA_CLIENT_SATAR_IDX,
	TEGRA_CLIENT_MPCORER_IDX,
	TEGRA_CLIENT_NVENCSWR_IDX,
	TEGRA_CLIENT_AFIW_IDX,
	TEGRA_CLIENT_AVPCARM7W_IDX,
	TEGRA_CLIENT_HDAW_IDX,
	TEGRA_CLIENT_HOST1XW_IDX,
	TEGRA_CLIENT_MPCOREW_IDX,
	TEGRA_CLIENT_PPCSAHBDMAW_IDX,
	TEGRA_CLIENT_PPCSAHBSLVW_IDX,
	TEGRA_CLIENT_SATAW_IDX,
	TEGRA_CLIENT_ISPRA_IDX,
	TEGRA_CLIENT_ISPWA_IDX,
	TEGRA_CLIENT_ISPWB_IDX,
	TEGRA_CLIENT_XUSB_HOSTR_IDX,
	TEGRA_CLIENT_XUSB_HOSTW_IDX,
	TEGRA_CLIENT_XUSB_DEVR_IDX,
	TEGRA_CLIENT_XUSB_DEVW_IDX,
	TEGRA_CLIENT_ISPRAB_IDX,
	TEGRA_CLIENT_ISPWAB_IDX,
	TEGRA_CLIENT_ISPWBB_IDX,
	TEGRA_CLIENT_TSECSRD_IDX,
	TEGRA_CLIENT_TSECSWR_IDX,
	TEGRA_CLIENT_A9AVPSCR_IDX,
	TEGRA_CLIENT_A9AVPSCW_IDX,
	TEGRA_CLIENT_GPUSRD_IDX,
	TEGRA_CLIENT_GPUSWR_IDX,
	TEGRA_CLIENT_DISPLAYT_IDX,
	TEGRA_CLIENT_SDMMCRA_IDX,
	TEGRA_CLIENT_SDMMCRAA_IDX,
	TEGRA_CLIENT_SDMMCR_IDX,
	TEGRA_CLIENT_SDMMCRAB_IDX,
	TEGRA_CLIENT_SDMMCWA_IDX,
	TEGRA_CLIENT_SDMMCWAA_IDX,
	TEGRA_CLIENT_SDMMCW_IDX,
	TEGRA_CLIENT_SDMMCWAB_IDX,
	TEGRA_CLIENT_VICSRD_IDX,
	TEGRA_CLIENT_VICSWR_IDX,
	TEGRA_CLIENT_VIW_IDX,
	TEGRA_CLIENT_DISPLAYD_IDX,
	TEGRA_CLIENT_NVDECSRD_IDX,
	TEGRA_CLIENT_NVDECSWR_IDX,
	TEGRA_CLIENT_APER_IDX,
	TEGRA_CLIENT_APEW_IDX,
	TEGRA_CLIENT_NVJPGSRD_IDX,
	TEGRA_CLIENT_NVJPGSWR_IDX,
	TEGRA_CLIENT_SESRD_IDX,
	TEGRA_CLIENT_SESWR_IDX,
	TEGRA_CLIENT_AXIAPR_IDX,
	TEGRA_CLIENT_AXIAPW_IDX,
	TEGRA_CLIENT_ETRR_IDX,
	TEGRA_CLIENT_ETRW_IDX,
	TEGRA_CLIENT_TSECSRDB_IDX,
	TEGRA_CLIENT_TSECSWRB_IDX,
	TEGRA_CLIENT_GPUSRD2_IDX,
	TEGRA_CLIENT_GPUSWR2_IDX,
};

enum {
	DRAM_TYPE_DDR3   = 0,
	DRAM_TYPE_LPDDR4 = 1,
	DRAM_TYPE_LPDDR2 = 2,
	DRAM_TYPE_DDR2 = 3,
};

static struct tegra_ptsa_info tegra210_ptsa_info = {
	.dis_ptsa_min = -5 & MC_PTSA_MIN_DEFAULT_MASK,
	.dis_ptsa_max = 31 & MC_PTSA_MAX_DEFAULT_MASK,
	.disb_ptsa_min = -5 & MC_PTSA_MIN_DEFAULT_MASK,
	.disb_ptsa_max = 31 & MC_PTSA_MAX_DEFAULT_MASK,
	.ve_ptsa_min = -5 & MC_PTSA_MIN_DEFAULT_MASK,
	.ve_ptsa_max = 31 & MC_PTSA_MAX_DEFAULT_MASK,
	.ve2_ptsa_min = -5 & MC_PTSA_MIN_DEFAULT_MASK,
	.ve2_ptsa_max = 31 & MC_PTSA_MAX_DEFAULT_MASK,
	.ring2_ptsa_rate = 12 & MC_PTSA_RATE_DEFAULT_MASK,
	.ring2_ptsa_min = -2 & MC_PTSA_MIN_DEFAULT_MASK,
	.ring2_ptsa_max = 0,
	.mpcorer_ptsa_min = -4 & MC_PTSA_MIN_DEFAULT_MASK,
	.mpcorer_ptsa_max = 4 & MC_PTSA_MAX_DEFAULT_MASK,
	.ftop_ptsa_min = -2 & MC_PTSA_MIN_DEFAULT_MASK,
	.ftop_ptsa_max = 0,
	.smmu_ptsa_min = 1 & MC_PTSA_MIN_DEFAULT_MASK,
	.smmu_ptsa_max = 1 & MC_PTSA_MAX_DEFAULT_MASK,
	.ring1_ptsa_min = -5 & MC_PTSA_MIN_DEFAULT_MASK,
	.ring1_ptsa_max = 31 & MC_PTSA_MAX_DEFAULT_MASK,
	.isp_ptsa_min = -5 & MC_PTSA_MIN_DEFAULT_MASK,
	.isp_ptsa_max = 31 & MC_PTSA_MAX_DEFAULT_MASK,
	.a9avppc_ptsa_min = -5 & MC_PTSA_MIN_DEFAULT_MASK,
	.a9avppc_ptsa_max = 16 & MC_PTSA_MAX_DEFAULT_MASK,
	.avp_ptsa_min = -2 & MC_PTSA_MIN_DEFAULT_MASK,
	.avp_ptsa_max = 0,
	.mse_ptsa_min = -2 & MC_PTSA_MIN_DEFAULT_MASK,
	.mse_ptsa_max = 0,
	.gk_ptsa_min = -2 & MC_PTSA_MIN_DEFAULT_MASK,
	.gk_ptsa_max = 0,
	.vicpc_ptsa_min = -2 & MC_PTSA_MIN_DEFAULT_MASK,
	.vicpc_ptsa_max = 0,
	.apb_ptsa_min = -2 & MC_PTSA_MIN_DEFAULT_MASK,
	.apb_ptsa_max = 0,
	.pcx_ptsa_min = -2 & MC_PTSA_MIN_DEFAULT_MASK,
	.pcx_ptsa_max = 0,
	.host_ptsa_min = -2 & MC_PTSA_MIN_DEFAULT_MASK,
	.host_ptsa_max = 0,
	.ahb_ptsa_min = -2 & MC_PTSA_MIN_DEFAULT_MASK,
	.ahb_ptsa_max = 0,
	.sax_ptsa_min = -2 & MC_PTSA_MIN_DEFAULT_MASK,
	.sax_ptsa_max = 0,
	.aud_ptsa_min = -5 & MC_PTSA_MIN_DEFAULT_MASK,
	.aud_ptsa_max = 31 & MC_PTSA_MAX_DEFAULT_MASK,
	.sd_ptsa_min = -2 & MC_PTSA_MIN_DEFAULT_MASK,
	.sd_ptsa_max = 0,
	.usbx_ptsa_min = -2 & MC_PTSA_MIN_DEFAULT_MASK,
	.usbx_ptsa_max = 0,
	.usbd_ptsa_min = -2 & MC_PTSA_MIN_DEFAULT_MASK,
	.usbd_ptsa_max = 0,
	.jpg_ptsa_min = -2 & MC_PTSA_MIN_DEFAULT_MASK,
	.jpg_ptsa_max = 0,
	.gk2_ptsa_min = -2 & MC_PTSA_MIN_DEFAULT_MASK,
	.gk2_ptsa_max = 0,
};

static struct tegra_mc_client tegra210_mc_clients[] = {
	{
		.id = 0x00,
		.name = "ptcr",
		.swgroup = TEGRA_SWGROUP_PTC,
		.la = {
			.reg = 0x34c,
			.shift = 0,
			.mask = 0xff,
		},
	}, {
		.id = 0x01,
		.name = "display0a",
		.swgroup = TEGRA_SWGROUP_DC,
		.smmu = {
			.reg = 0x228,
			.bit = 1,
		},
		.la = {
			.reg = 0x2e8,
			.shift = 0,
			.mask = 0xff,
			.def = 0x50,
			.scaling_support = true,
			.type = TEGRA_LA_CLIENT_TYPE_DISPLAY,
			.disp_client = {
				.win_type = TEGRA_LA_DISP_WIN_TYPE_FULL,
				.mccif_size_bytes = 6144,
				.line_buf_sz_bytes = 151552,
			},
		},
	}, {
		.id = 0x02,
		.name = "display0ab",
		.swgroup = TEGRA_SWGROUP_DCB,
		.smmu = {
			.reg = 0x228,
			.bit = 2,
		},
		.la = {
			.reg = 0x2f4,
			.shift = 0,
			.mask = 0xff,
			.def = 0x50,
			.scaling_support = true,
			.type = TEGRA_LA_CLIENT_TYPE_DISPLAY,
			.disp_client = {
				.win_type = TEGRA_LA_DISP_WIN_TYPE_FULLB,
				.mccif_size_bytes = 11520,
				.line_buf_sz_bytes = 112640,
			},
		},
	}, {
		.id = 0x03,
		.name = "display0b",
		.swgroup = TEGRA_SWGROUP_DC,
		.smmu = {
			.reg = 0x228,
			.bit = 3,
		},
		.la = {
			.reg = 0x2e8,
			.shift = 16,
			.mask = 0xff,
			.def = 0x50,
			.scaling_support = true,
			.type = TEGRA_LA_CLIENT_TYPE_DISPLAY,
			.disp_client = {
				.win_type = TEGRA_LA_DISP_WIN_TYPE_FULLA,
				.mccif_size_bytes = 6144,
				.line_buf_sz_bytes = 112640,
			},
		},
	}, {
		.id = 0x04,
		.name = "display0bb",
		.swgroup = TEGRA_SWGROUP_DCB,
		.smmu = {
			.reg = 0x228,
			.bit = 4,
		},
		.la = {
			.reg = 0x2f4,
			.shift = 16,
			.mask = 0xff,
			.def = 0x50,
			.scaling_support = true,
			.type = TEGRA_LA_CLIENT_TYPE_DISPLAY,
			.disp_client = {
				.win_type = TEGRA_LA_DISP_WIN_TYPE_FULLB,
				.mccif_size_bytes = 6144,
				.line_buf_sz_bytes = 112640,
			},
		},
	}, {
		.id = 0x05,
		.name = "display0c",
		.swgroup = TEGRA_SWGROUP_DC,
		.smmu = {
			.reg = 0x228,
			.bit = 5,
		},
		.la = {
			.reg = 0x2ec,
			.shift = 0,
			.mask = 0xff,
			.def = 0x50,
			.scaling_support = true,
			.type = TEGRA_LA_CLIENT_TYPE_DISPLAY,
			.disp_client = {
				.win_type = TEGRA_LA_DISP_WIN_TYPE_FULLA,
				.mccif_size_bytes = 11520,
				.line_buf_sz_bytes = 112640,
			},
		},
	}, {
		.id = 0x06,
		.name = "display0cb",
		.swgroup = TEGRA_SWGROUP_DCB,
		.smmu = {
			.reg = 0x228,
			.bit = 6,
		},
		.la = {
			.reg = 0x2f8,
			.shift = 0,
			.mask = 0xff,
			.def = 0x50,
			.scaling_support = true,
			.type = TEGRA_LA_CLIENT_TYPE_DISPLAY,
			.disp_client = {
				.win_type = TEGRA_LA_DISP_WIN_TYPE_FULLB,
				.mccif_size_bytes = 6144,
				.line_buf_sz_bytes = 112640,
			},
		},
	}, {
		.id = 0x0e,
		.name = "afir",
		.swgroup = TEGRA_SWGROUP_AFI,
		.smmu = {
			.reg = 0x228,
			.bit = 14,
		},
		.la = {
			.reg = 0x2e0,
			.shift = 0,
			.mask = 0xff,
			.def = 0x3B,
			.la_ref_clk_mhz = 400,
		},
	}, {
		.id = 0x0f,
		.name = "avpcarm7r",
		.swgroup = TEGRA_SWGROUP_AVPC,
		.smmu = {
			.reg = 0x228,
			.bit = 15,
		},
		.la = {
			.reg = 0x2e4,
			.shift = 0,
			.mask = 0xff,
			.def = 0x04,
		},
	}, {
		.id = 0x10,
		.name = "displayhc",
		.swgroup = TEGRA_SWGROUP_DC,
		.smmu = {
			.reg = 0x228,
			.bit = 16,
		},
		.la = {
			.reg = 0x2f0,
			.shift = 0,
			.mask = 0xff,
			.def = 0x50,
			.type = TEGRA_LA_CLIENT_TYPE_DISPLAY,
			.disp_client = {
				.win_type = TEGRA_LA_DISP_WIN_TYPE_CURSOR,
				.mccif_size_bytes = 4992,
				.line_buf_sz_bytes = 320,
			},
		},
	}, {
		.id = 0x11,
		.name = "displayhcb",
		.swgroup = TEGRA_SWGROUP_DCB,
		.smmu = {
			.reg = 0x228,
			.bit = 17,
		},
		.la = {
			.reg = 0x2fc,
			.shift = 0,
			.mask = 0xff,
			.def = 0x50,
			.type = TEGRA_LA_CLIENT_TYPE_DISPLAY,
			.disp_client = {
				.win_type = TEGRA_LA_DISP_WIN_TYPE_CURSOR,
				.mccif_size_bytes = 4992,
				.line_buf_sz_bytes = 320,
			},
		},
	}, {
		.id = 0x15,
		.name = "hdar",
		.swgroup = TEGRA_SWGROUP_HDA,
		.smmu = {
			.reg = 0x228,
			.bit = 21,
		},
		.la = {
			.reg = 0x318,
			.shift = 0,
			.mask = 0xff,
			.def = 0x24,
		},
	}, {
		.id = 0x16,
		.name = "host1xdmar",
		.swgroup = TEGRA_SWGROUP_HC,
		.smmu = {
			.reg = 0x228,
			.bit = 22,
		},
		.la = {
			.reg = 0x310,
			.shift = 0,
			.mask = 0xff,
			.def = 0x16,
			.la_ref_clk_mhz = 800,
		},
	}, {
		.id = 0x17,
		.name = "host1xr",
		.swgroup = TEGRA_SWGROUP_HC,
		.smmu = {
			.reg = 0x228,
			.bit = 23,
		},
		.la = {
			.reg = 0x310,
			.shift = 16,
			.mask = 0xff,
			.def = 0x50,
		},
	}, {
		.id = 0x1c,
		.name = "nvencsrd",
		.swgroup = TEGRA_SWGROUP_NVENC,
		.smmu = {
			.reg = 0x228,
			.bit = 28,
		},
		.la = {
			.reg = 0x328,
			.shift = 0,
			.mask = 0xff,
			.def = 0x74,
			.la_ref_clk_mhz = 209,
		},
	}, {
		.id = 0x1d,
		.name = "ppcsahbdmar",
		.swgroup = TEGRA_SWGROUP_PPCS,
		.smmu = {
			.reg = 0x228,
			.bit = 29,
		},
		.la = {
			.reg = 0x344,
			.shift = 0,
			.mask = 0xff,
			.def = 0x49,
		},
	}, {
		.id = 0x1e,
		.name = "ppcsahbslvr",
		.swgroup = TEGRA_SWGROUP_PPCS,
		.smmu = {
			.reg = 0x228,
			.bit = 30,
		},
		.la = {
			.reg = 0x344,
			.shift = 16,
			.mask = 0xff,
			.def = 0x59,
			.la_ref_clk_mhz = 248,
		},
	}, {
		.id = 0x1f,
		.name = "satar",
		.swgroup = TEGRA_SWGROUP_SATA,
		.smmu = {
			.reg = 0x228,
			.bit = 31,
		},
		.la = {
			.reg = 0x350,
			.shift = 0,
			.mask = 0xff,
			.def = 0x68,
			.la_ref_clk_mhz = 400,
		},
	}, {
		.id = 0x27,
		.name = "mpcorer",
		.swgroup = TEGRA_SWGROUP_MPCORE,
		.la = {
			.reg = 0x320,
			.shift = 0,
			.mask = 0xff,
			.def = 0x04,
		},
	}, {
		.id = 0x2b,
		.name = "nvencswr",
		.swgroup = TEGRA_SWGROUP_NVENC,
		.smmu = {
			.reg = 0x22c,
			.bit = 11,
		},
		.la = {
			.reg = 0x328,
			.shift = 16,
			.mask = 0xff,
			.def = 0x80,
			.la_ref_clk_mhz = 800,
		},
	}, {
		.id = 0x31,
		.name = "afiw",
		.swgroup = TEGRA_SWGROUP_AFI,
		.smmu = {
			.reg = 0x22c,
			.bit = 17,
		},
		.la = {
			.reg = 0x2e0,
			.shift = 16,
			.mask = 0xff,
			.def = 0x80,
			.la_ref_clk_mhz = 800,
		},
	}, {
		.id = 0x32,
		.name = "avpcarm7w",
		.swgroup = TEGRA_SWGROUP_AVPC,
		.smmu = {
			.reg = 0x22c,
			.bit = 18,
		},
		.la = {
			.reg = 0x2e4,
			.shift = 16,
			.mask = 0xff,
			.def = 0x80,
			.la_ref_clk_mhz = 800,
		},
	}, {
		.id = 0x35,
		.name = "hdaw",
		.swgroup = TEGRA_SWGROUP_HDA,
		.smmu = {
			.reg = 0x22c,
			.bit = 21,
		},
		.la = {
			.reg = 0x318,
			.shift = 16,
			.mask = 0xff,
			.def = 0x80,
			.la_ref_clk_mhz = 800,
		},
	}, {
		.id = 0x36,
		.name = "host1xw",
		.swgroup = TEGRA_SWGROUP_HC,
		.smmu = {
			.reg = 0x22c,
			.bit = 22,
		},
		.la = {
			.reg = 0x314,
			.shift = 0,
			.mask = 0xff,
			.def = 0x80,
			.la_ref_clk_mhz = 800,
		},
	}, {
		.id = 0x39,
		.name = "mpcorew",
		.swgroup = TEGRA_SWGROUP_MPCORE,
		.la = {
			.reg = 0x320,
			.shift = 16,
			.mask = 0xff,
			.def = 0x80,
			.la_ref_clk_mhz = 800,
		},
	}, {
		.id = 0x3b,
		.name = "ppcsahbdmaw",
		.swgroup = TEGRA_SWGROUP_PPCS,
		.smmu = {
			.reg = 0x22c,
			.bit = 27,
		},
		.la = {
			.reg = 0x348,
			.shift = 0,
			.mask = 0xff,
			.def = 0x80,
			.la_ref_clk_mhz = 800,
		},
	}, {
		.id = 0x3c,
		.name = "ppcsahbslvw",
		.swgroup = TEGRA_SWGROUP_PPCS,
		.smmu = {
			.reg = 0x22c,
			.bit = 28,
		},
		.la = {
			.reg = 0x348,
			.shift = 16,
			.mask = 0xff,
			.def = 0x80,
			.la_ref_clk_mhz = 800,
		},
	}, {
		.id = 0x3d,
		.name = "sataw",
		.swgroup = TEGRA_SWGROUP_SATA,
		.smmu = {
			.reg = 0x22c,
			.bit = 29,
		},
		.la = {
			.reg = 0x350,
			.shift = 16,
			.mask = 0xff,
			.def = 0x80,
			.la_ref_clk_mhz = 800,
		},
	}, {
		.id = 0x44,
		.name = "ispra",
		.swgroup = TEGRA_SWGROUP_ISP2,
		.smmu = {
			.reg = 0x230,
			.bit = 4,
		},
		.la = {
			.reg = 0x370,
			.shift = 0,
			.mask = 0xff,
			.def = 0x33,
			.la_ref_clk_mhz = 412,
			.type = TEGRA_LA_CLIENT_TYPE_CAMERA,
		},
	}, {
		.id = 0x46,
		.name = "ispwa",
		.swgroup = TEGRA_SWGROUP_ISP2,
		.smmu = {
			.reg = 0x230,
			.bit = 6,
		},
		.la = {
			.reg = 0x374,
			.shift = 0,
			.mask = 0xff,
			.def = 0x80,
			.la_ref_clk_mhz = 800,
			.type = TEGRA_LA_CLIENT_TYPE_CAMERA,
		},
	}, {
		.id = 0x47,
		.name = "ispwb",
		.swgroup = TEGRA_SWGROUP_ISP2,
		.smmu = {
			.reg = 0x230,
			.bit = 7,
		},
		.la = {
			.reg = 0x374,
			.shift = 16,
			.mask = 0xff,
			.def = 0x80,
			.la_ref_clk_mhz = 800,
			.type = TEGRA_LA_CLIENT_TYPE_CAMERA,
		},
	}, {
		.id = 0x4a,
		.name = "xusb_hostr",
		.swgroup = TEGRA_SWGROUP_XUSB_HOST,
		.smmu = {
			.reg = 0x230,
			.bit = 10,
		},
		.la = {
			.reg = 0x37c,
			.shift = 0,
			.mask = 0xff,
			.def = 0x42,
			.la_ref_clk_mhz = 300,
		},
	}, {
		.id = 0x4b,
		.name = "xusb_hostw",
		.swgroup = TEGRA_SWGROUP_XUSB_HOST,
		.smmu = {
			.reg = 0x230,
			.bit = 11,
		},
		.la = {
			.reg = 0x37c,
			.shift = 16,
			.mask = 0xff,
			.def = 0x80,
			.la_ref_clk_mhz = 800,
		},
	}, {
		.id = 0x4c,
		.name = "xusb_devr",
		.swgroup = TEGRA_SWGROUP_XUSB_DEV,
		.smmu = {
			.reg = 0x230,
			.bit = 12,
		},
		.la = {
			.reg = 0x380,
			.shift = 0,
			.mask = 0xff,
			.def = 0x3b,
			.la_ref_clk_mhz = 400,
		},
	}, {
		.id = 0x4d,
		.name = "xusb_devw",
		.swgroup = TEGRA_SWGROUP_XUSB_DEV,
		.smmu = {
			.reg = 0x230,
			.bit = 13,
		},
		.la = {
			.reg = 0x380,
			.shift = 16,
			.mask = 0xff,
			.def = 0x80,
			.la_ref_clk_mhz = 800,
		},
	}, {
		.id = 0x4e,
		.name = "isprab",
		.swgroup = TEGRA_SWGROUP_ISP2B,
		.smmu = {
			.reg = 0x230,
			.bit = 14,
		},
		.la = {
			.reg = 0x384,
			.shift = 0,
			.mask = 0xff,
			.def = 0x33,
			.la_ref_clk_mhz = 412,
			.type = TEGRA_LA_CLIENT_TYPE_CAMERA,
		},
	}, {
		.id = 0x50,
		.name = "ispwab",
		.swgroup = TEGRA_SWGROUP_ISP2B,
		.smmu = {
			.reg = 0x230,
			.bit = 16,
		},
		.la = {
			.reg = 0x388,
			.shift = 0,
			.mask = 0xff,
			.def = 0x80,
			.la_ref_clk_mhz = 800,
			.type = TEGRA_LA_CLIENT_TYPE_CAMERA,
		},
	}, {
		.id = 0x51,
		.name = "ispwbb",
		.swgroup = TEGRA_SWGROUP_ISP2B,
		.smmu = {
			.reg = 0x230,
			.bit = 17,
		},
		.la = {
			.reg = 0x388,
			.shift = 16,
			.mask = 0xff,
			.def = 0x80,
			.la_ref_clk_mhz = 800,
			.type = TEGRA_LA_CLIENT_TYPE_CAMERA,
		},
	}, {
		.id = 0x54,
		.name = "tsecsrd",
		.swgroup = TEGRA_SWGROUP_TSEC,
		.smmu = {
			.reg = 0x230,
			.bit = 20,
		},
		.la = {
			.reg = 0x390,
			.shift = 0,
			.mask = 0xff,
			.def = 0x9d,
			.la_ref_clk_mhz = 248,
		},
	}, {
		.id = 0x55,
		.name = "tsecswr",
		.swgroup = TEGRA_SWGROUP_TSEC,
		.smmu = {
			.reg = 0x230,
			.bit = 21,
		},
		.la = {
			.reg = 0x390,
			.shift = 16,
			.mask = 0xff,
			.def = 0x80,
			.la_ref_clk_mhz = 800,
		},
	}, {
		.id = 0x56,
		.name = "a9avpscr",
		.swgroup = TEGRA_SWGROUP_A9AVP,
		.smmu = {
			.reg = 0x230,
			.bit = 22,
		},
		.la = {
			.reg = 0x3a4,
			.shift = 0,
			.mask = 0xff,
			.def = 0x04,
		},
	}, {
		.id = 0x57,
		.name = "a9avpscw",
		.swgroup = TEGRA_SWGROUP_A9AVP,
		.smmu = {
			.reg = 0x230,
			.bit = 23,
		},
		.la = {
			.reg = 0x3a4,
			.shift = 16,
			.mask = 0xff,
			.def = 0x80,
		},
	}, {
		.id = 0x58,
		.name = "gpusrd",
		.swgroup = TEGRA_SWGROUP_GPU,
		.smmu = {
			/* read-only */
			.reg = 0x230,
			.bit = 24,
		},
		.la = {
			.reg = 0x3ac,
			.shift = 0,
			.mask = 0xff,
			.def = 0x19,
			.la_ref_clk_mhz = 800,
		},
	}, {
		.id = 0x59,
		.name = "gpuswr",
		.swgroup = TEGRA_SWGROUP_GPU,
		.smmu = {
			/* read-only */
			.reg = 0x230,
			.bit = 25,
		},
		.la = {
			.reg = 0x3ac,
			.shift = 16,
			.mask = 0xff,
			.def = 0x80,
			.la_ref_clk_mhz = 800,
		},
	}, {
		.id = 0x5a,
		.name = "displayt",
		.swgroup = TEGRA_SWGROUP_DC,
		.smmu = {
			.reg = 0x230,
			.bit = 26,
		},
		.la = {
			.reg = 0x2f0,
			.shift = 16,
			.mask = 0xff,
			.def = 0x50,
			.type = TEGRA_LA_CLIENT_TYPE_DISPLAY,
			.disp_client = {
				.win_type = TEGRA_LA_DISP_WIN_TYPE_SIMPLE,
				.mccif_size_bytes = 4672,
				.line_buf_sz_bytes = 18432,
			},
		},
	}, {
		.id = 0x60,
		.name = "sdmmcra",
		.swgroup = TEGRA_SWGROUP_SDMMC1A,
		.smmu = {
			.reg = 0x234,
			.bit = 0,
		},
		.la = {
			.reg = 0x3b8,
			.shift = 0,
			.mask = 0xff,
			.def = 0x96,
			.la_ref_clk_mhz = 248,
		},
	}, {
		.id = 0x61,
		.name = "sdmmcraa",
		.swgroup = TEGRA_SWGROUP_SDMMC2A,
		.smmu = {
			.reg = 0x234,
			.bit = 1,
		},
		.la = {
			.reg = 0x3bc,
			.shift = 0,
			.mask = 0xff,
			.def = 0x58,
			.la_ref_clk_mhz = 248,
		},
	}, {
		.id = 0x62,
		.name = "sdmmcr",
		.swgroup = TEGRA_SWGROUP_SDMMC3A,
		.smmu = {
			.reg = 0x234,
			.bit = 2,
		},
		.la = {
			.reg = 0x3c0,
			.shift = 0,
			.mask = 0xff,
			.def = 0x96,
			.la_ref_clk_mhz = 248,
		},
	}, {
		.id = 0x63,
		.swgroup = TEGRA_SWGROUP_SDMMC4A,
		.name = "sdmmcrab",
		.smmu = {
			.reg = 0x234,
			.bit = 3,
		},
		.la = {
			.reg = 0x3c4,
			.shift = 0,
			.mask = 0xff,
			.def = 0x58,
			.la_ref_clk_mhz = 248,
		},
	}, {
		.id = 0x64,
		.name = "sdmmcwa",
		.swgroup = TEGRA_SWGROUP_SDMMC1A,
		.smmu = {
			.reg = 0x234,
			.bit = 4,
		},
		.la = {
			.reg = 0x3b8,
			.shift = 16,
			.mask = 0xff,
			.def = 0x80,
			.la_ref_clk_mhz = 800,
		},
	}, {
		.id = 0x65,
		.name = "sdmmcwaa",
		.swgroup = TEGRA_SWGROUP_SDMMC2A,
		.smmu = {
			.reg = 0x234,
			.bit = 5,
		},
		.la = {
			.reg = 0x3bc,
			.shift = 16,
			.mask = 0xff,
			.def = 0x80,
			.la_ref_clk_mhz = 800,
		},
	}, {
		.id = 0x66,
		.name = "sdmmcw",
		.swgroup = TEGRA_SWGROUP_SDMMC3A,
		.smmu = {
			.reg = 0x234,
			.bit = 6,
		},
		.la = {
			.reg = 0x3c0,
			.shift = 16,
			.mask = 0xff,
			.def = 0x80,
			.la_ref_clk_mhz = 800,
		},
	}, {
		.id = 0x67,
		.name = "sdmmcwab",
		.swgroup = TEGRA_SWGROUP_SDMMC4A,
		.smmu = {
			.reg = 0x234,
			.bit = 7,
		},
		.la = {
			.reg = 0x3c4,
			.shift = 16,
			.mask = 0xff,
			.def = 0x80,
			.la_ref_clk_mhz = 800,
		},
	}, {
		.id = 0x6c,
		.name = "vicsrd",
		.swgroup = TEGRA_SWGROUP_VIC,
		.smmu = {
			.reg = 0x234,
			.bit = 12,
		},
		.la = {
			.reg = 0x394,
			.shift = 0,
			.mask = 0xff,
			.def = 0x1d,
			.la_ref_clk_mhz = 800,
		},
	}, {
		.id = 0x6d,
		.name = "vicswr",
		.swgroup = TEGRA_SWGROUP_VIC,
		.smmu = {
			.reg = 0x234,
			.bit = 13,
		},
		.la = {
			.reg = 0x394,
			.shift = 16,
			.mask = 0xff,
			.def = 0x80,
			.la_ref_clk_mhz = 800,
		},
	}, {
		.id = 0x72,
		.name = "viw",
		.swgroup = TEGRA_SWGROUP_VI,
		.smmu = {
			.reg = 0x234,
			.bit = 18,
		},
		.la = {
			.reg = 0x398,
			.shift = 0,
			.mask = 0xff,
			.def = 0x80,
			.la_ref_clk_mhz = 800,
			.type = TEGRA_LA_CLIENT_TYPE_CAMERA,
		},
	}, {
		.id = 0x73,
		.name = "displayd",
		.swgroup = TEGRA_SWGROUP_DC,
		.smmu = {
			.reg = 0x234,
			.bit = 19,
		},
		.la = {
			.reg = 0x3c8,
			.shift = 0,
			.mask = 0xff,
			.def = 0x50,
			.type = TEGRA_LA_CLIENT_TYPE_DISPLAY,
			.disp_client = {
				.win_type = TEGRA_LA_DISP_WIN_TYPE_SIMPLE,
				.mccif_size_bytes = 4672,
				.line_buf_sz_bytes = 18432,
			},
		},
	}, {
		.id = 0x78,
		.name = "nvdecsrd",
		.swgroup = TEGRA_SWGROUP_NVDEC,
		.smmu = {
			.reg = 0x234,
			.bit = 24,
		},
		.la = {
			.reg = 0x3d8,
			.shift = 0,
			.mask = 0xff,
			.def = 0x1e,
			.la_ref_clk_mhz = 352,
		},
	}, {
		.id = 0x79,
		.name = "nvdecswr",
		.swgroup = TEGRA_SWGROUP_NVDEC,
		.smmu = {
			.reg = 0x234,
			.bit = 25,
		},
		.la = {
			.reg = 0x3d8,
			.shift = 16,
			.mask = 0xff,
			.def = 0x80,
			.la_ref_clk_mhz = 800,
		},
	}, {
		.id = 0x7a,
		.name = "aper",
		.swgroup = TEGRA_SWGROUP_APE,
		.smmu = {
			.reg = 0x234,
			.bit = 26,
		},
		.la = {
			.reg = 0x3dc,
			.shift = 0,
			.mask = 0xff,
			.def = 0xff,
		},
	}, {
		.id = 0x7b,
		.name = "apew",
		.swgroup = TEGRA_SWGROUP_APE,
		.smmu = {
			.reg = 0x234,
			.bit = 27,
		},
		.la = {
			.reg = 0x3dc,
			.shift = 0,
			.mask = 0xff,
			.def = 0x80,
			.la_ref_clk_mhz = 800,
		},
	}, {
		.id = 0x7e,
		.name = "nvjpgsrd",
		.swgroup = TEGRA_SWGROUP_NVJPG,
		.smmu = {
			.reg = 0x234,
			.bit = 30,
		},
		.la = {
			.reg = 0x3e4,
			.shift = 0,
			.mask = 0xff,
			.def = 0x7a,
			.la_ref_clk_mhz = 200,
		},
	}, {
		.id = 0x7f,
		.name = "nvjpgswr",
		.swgroup = TEGRA_SWGROUP_NVJPG,
		.smmu = {
			.reg = 0x234,
			.bit = 31,
		},
		.la = {
			.reg = 0x3e4,
			.shift = 16,
			.mask = 0xff,
			.def = 0x80,
			.la_ref_clk_mhz = 800,
		},
	}, {
		.id = 0x80,
		.name = "sesrd",
		.swgroup = TEGRA_SWGROUP_SE,
		.smmu = {
			.reg = 0xb98,
			.bit = 0,
		},
		.la = {
			.reg = 0x3e0,
			.shift = 0,
			.mask = 0xff,
			.def = 0x58,
			.la_ref_clk_mhz = 248,
		},
	}, {
		.id = 0x81,
		.name = "seswr",
		.swgroup = TEGRA_SWGROUP_SE,
		.smmu = {
			.reg = 0xb98,
			.bit = 1,
		},
		.la = {
			.reg = 0x3e0,
			.shift = 16,
			.mask = 0xff,
			.def = 0x80,
			.la_ref_clk_mhz = 800,
		},
	}, {
		.id = 0x82,
		.name = "axiapr",
		.swgroup = TEGRA_SWGROUP_AXIAP,
		.smmu = {
			.reg = 0xb98,
			.bit = 2,
		},
		.la = {
			.reg = 0x3a0,
			.shift = 0,
			.mask = 0xff,
			.def = 0x50,
		},
	}, {
		.id = 0x83,
		.name = "axiapw",
		.swgroup = TEGRA_SWGROUP_AXIAP,
		.smmu = {
			.reg = 0xb98,
			.bit = 3,
		},
		.la = {
			.reg = 0x3a0,
			.shift = 16,
			.mask = 0xff,
			.def = 0x80,
			.la_ref_clk_mhz = 800,
		},
	}, {
		.id = 0x84,
		.name = "etrr",
		.swgroup = TEGRA_SWGROUP_ETR,
		.smmu = {
			.reg = 0xb98,
			.bit = 4,
		},
		.la = {
			.reg = 0x3ec,
			.shift = 0,
			.mask = 0xff,
			.def = 0x50,
		},
	}, {
		.id = 0x85,
		.name = "etrw",
		.swgroup = TEGRA_SWGROUP_ETR,
		.smmu = {
			.reg = 0xb98,
			.bit = 5,
		},
		.la = {
			.reg = 0x3ec,
			.shift = 16,
			.mask = 0xff,
			.def = 0x80,
			.la_ref_clk_mhz = 800,
		},
	}, {
		.id = 0x86,
		.name = "tsecsrdb",
		.swgroup = TEGRA_SWGROUP_TSECB,
		.smmu = {
			.reg = 0xb98,
			.bit = 6,
		},
		.la = {
			.reg = 0x3f0,
			.shift = 0,
			.mask = 0xff,
			.def = 0x9d,
			.la_ref_clk_mhz = 248,
		},
	}, {
		.id = 0x87,
		.name = "tsecswrb",
		.swgroup = TEGRA_SWGROUP_TSECB,
		.smmu = {
			.reg = 0xb98,
			.bit = 7,
		},
		.la = {
			.reg = 0x3f0,
			.shift = 16,
			.mask = 0xff,
			.def = 0x80,
			.la_ref_clk_mhz = 800,
		},
	}, {
		.id = 0x88,
		.name = "gpusrd2",
		.swgroup = TEGRA_SWGROUP_GPU,
		.smmu = {
			/* read-only */
			.reg = 0xb98,
			.bit = 8,
		},
		.la = {
			.reg = 0x3e8,
			.shift = 0,
			.mask = 0xff,
			.def = 0x19,
			.la_ref_clk_mhz = 800,
		},
	}, {
		.id = 0x89,
		.name = "gpuswr2",
		.swgroup = TEGRA_SWGROUP_GPU,
		.smmu = {
			/* read-only */
			.reg = 0xb98,
			.bit = 9,
		},
		.la = {
			.reg = 0x3e8,
			.shift = 16,
			.mask = 0xff,
			.def = 0x80,
			.la_ref_clk_mhz = 800,
		},
	},
};

static const struct of_device_id emc_match[] = {
	{ .compatible = "nvidia,tegra210-emc" },
	{},
};

static int id_lookup(struct tegra_mc *mc, unsigned int id)
{
	int i;

	for (i = 0; i < mc->soc->num_clients; i++) {
		if (mc->soc->clients[i].id == id)
			return i;
	}

	return -EINVAL;
}

static int tegra210_init_la(struct tegra_mc *mc)
{
	struct device_node *node;
	struct tegra_la_soc *la_soc = mc->soc->la_soc;

	node = of_find_matching_node(NULL, emc_match);
	if (!node)
		return -EINVAL;

	mc->emc_base = of_iomap(node, 0);
	if (!mc->emc_base)
		return -EINVAL;

	mc->dram_type = (readl(mc->emc_base + EMC_FBIO_CFG5) &
			EMC_FBIO_CFG5_DRAM_TYPE_MASK) >>
			EMC_FBIO_CFG5_DRAM_TYPE_SHIFT;

	if (mc->dram_type == DRAM_TYPE_LPDDR4) {
		la_soc->emc_min_freq_mhz_fp = 25000;
		la_soc->emc_min_freq_mhz = 25;
		la_soc->emc_max_freq_mhz = 2132;
		la_soc->hi_gd_fp = 1500;
		la_soc->lo_gd_fp = 18;
		la_soc->hi_gd_fpa = 14998;
		la_soc->lo_gd_fpa = 176;
		la_soc->dda_div = 1;
	} else {
		la_soc->emc_min_freq_mhz_fp = 12500;
		la_soc->emc_min_freq_mhz = 12;
		la_soc->emc_max_freq_mhz = 1200;
		la_soc->hi_gd_fp = 2000;
		la_soc->lo_gd_fp = 21;
		la_soc->hi_gd_fpa = 19998;
		la_soc->lo_gd_fpa = 208;
		la_soc->dda_div = 2;
	}

	la_soc->low_freq_bw = la_soc->emc_min_freq_mhz_fp * 2 *
			  LA_DRAM_WIDTH_BITS / 8 / 1000;

	mutex_init(&la_soc->la_ptsa_lock);
	return 0;
}

static unsigned int __fraction2dda_fp(unsigned int fraction_fpa,
				      unsigned int div,
				      unsigned int mask)
{
	unsigned int dda, r;
	int i;

	fraction_fpa /= div;

	for (i = 0; i < EMEM_PTSA_RATE_WIDTH; i++) {
		fraction_fpa *= 2;
		r = LA_FPA_TO_REAL(fraction_fpa);
		dda = (dda << 1) | (unsigned int)(r);
		fraction_fpa -= LA_REAL_TO_FPA(r);
	}
	if (fraction_fpa > 0) {
		if (dda != mask)
			dda++;
	}

	return min_t(unsigned int, dda, MAX_DDA_RATE);
}

static inline unsigned int fraction2dda_fp(unsigned int fraction_fp,
					   unsigned int div,
					   unsigned int mask)
{
	unsigned int fraction_fpa = LA_FP_TO_FPA(fraction_fp);

	return __fraction2dda_fp(fraction_fpa, div, mask);
}

static void calc_disp_and_camera_ptsa(struct tegra_mc *mc)
{
	struct tegra_la_soc *la_soc = mc->soc->la_soc;
	struct tegra_ptsa_info *p = la_soc->ptsa_info;
	unsigned int ve_bw_fp, ve2_bw_fp, isp_bw_fp;
	unsigned int total_dc0_bw_fp, total_dc1_bw_fp;
	unsigned int low_freq_bw_fp;
	unsigned int dis_frac_fp, disb_frac_fp;
	unsigned int total_iso_bw_fp;
	int max_max = (1 << EMEM_PTSA_MINMAX_WIDTH) - 1;
	int i;

	total_dc0_bw_fp = la_soc->total_dc0_bw * DDA_BW_MARGIN_FP;
	total_dc1_bw_fp = la_soc->total_dc1_bw * DDA_BW_MARGIN_FP;
	low_freq_bw_fp = LA_REAL_TO_FP(la_soc->low_freq_bw);
	dis_frac_fp = LA_FPA_TO_FP(la_soc->lo_gd_fpa * total_dc0_bw_fp /
				   low_freq_bw_fp);
	disb_frac_fp = LA_FPA_TO_FP(la_soc->lo_gd_fpa * total_dc1_bw_fp /
				    low_freq_bw_fp);
	total_iso_bw_fp = total_dc0_bw_fp + total_dc1_bw_fp;

	ve_bw_fp = la_soc->clients[TEGRA_CLIENT_VIW_IDX].la.camera_bw *
		   DDA_BW_MARGIN_FP;

	if (la_soc->agg_camera_client[TEGRA_LA_AGG_CAMERA_VE2].is_hiso) {
		ve2_bw_fp = DDA_BW_MARGIN_FP * (
			la_soc->clients[TEGRA_CLIENT_ISPRAB_IDX].la.camera_bw +
			la_soc->clients[TEGRA_CLIENT_ISPWAB_IDX].la.camera_bw +
			la_soc->clients[TEGRA_CLIENT_ISPWBB_IDX].la.camera_bw);
	} else {
		ve2_bw_fp = LA_REAL_TO_FP(
			la_soc->clients[TEGRA_CLIENT_ISPRAB_IDX].la.camera_bw +
			la_soc->clients[TEGRA_CLIENT_ISPWAB_IDX].la.camera_bw +
			la_soc->clients[TEGRA_CLIENT_ISPWBB_IDX].la.camera_bw);
	}

	if (la_soc->agg_camera_client[TEGRA_LA_AGG_CAMERA_ISP].is_hiso) {
		isp_bw_fp = DDA_BW_MARGIN_FP * (
			 la_soc->clients[TEGRA_CLIENT_ISPRA_IDX].la.camera_bw +
			 la_soc->clients[TEGRA_CLIENT_ISPWA_IDX].la.camera_bw +
			 la_soc->clients[TEGRA_CLIENT_ISPWB_IDX].la.camera_bw);
	} else {
		isp_bw_fp = LA_REAL_TO_FP(
			 la_soc->clients[TEGRA_CLIENT_ISPRA_IDX].la.camera_bw +
			 la_soc->clients[TEGRA_CLIENT_ISPWA_IDX].la.camera_bw +
			 la_soc->clients[TEGRA_CLIENT_ISPWB_IDX].la.camera_bw);
	}

	la_soc->agg_camera_client[TEGRA_LA_AGG_CAMERA_VE].bw_fp = ve_bw_fp;
	la_soc->agg_camera_client[TEGRA_LA_AGG_CAMERA_VE2].bw_fp = ve2_bw_fp;
	la_soc->agg_camera_client[TEGRA_LA_AGG_CAMERA_ISP].bw_fp = isp_bw_fp;

	for (i = 0; i < TEGRA_LA_AGG_CAMERA_NUM_CLIENTS; i++) {
		struct tegra_agg_camera_client *agg_client =
						&la_soc->agg_camera_client[i];

		if (agg_client->is_hiso) {
			agg_client->frac_fp = LA_FPA_TO_FP(la_soc->lo_gd_fpa *
							   agg_client->bw_fp /
							   low_freq_bw_fp);
			agg_client->ptsa_min = (unsigned int)(-5) &
						MC_PTSA_MIN_DEFAULT_MASK;
			agg_client->ptsa_max = (unsigned int)(max_max) &
						MC_PTSA_MAX_DEFAULT_MASK;

			total_iso_bw_fp += agg_client->bw_fp;
		} else {
			agg_client->frac_fp = ONE_DDA_FRAC_FP;
			agg_client->ptsa_min = (unsigned int)(-2) &
						MC_PTSA_MIN_DEFAULT_MASK;
			agg_client->ptsa_max = (unsigned int)(0) &
						MC_PTSA_MAX_DEFAULT_MASK;
		}
	}

	p->dis_ptsa_min = -5 & MC_PTSA_MIN_DEFAULT_MASK;
	p->dis_ptsa_max = max_max & MC_PTSA_MAX_DEFAULT_MASK;
	p->dis_ptsa_rate = fraction2dda_fp(dis_frac_fp, la_soc->dda_div,
					   MC_PTSA_RATE_DEFAULT_MASK) &
			   MC_PTSA_RATE_DEFAULT_MASK;

	p->disb_ptsa_min = -5 & MC_PTSA_MIN_DEFAULT_MASK;
	p->disb_ptsa_max = max_max & MC_PTSA_MAX_DEFAULT_MASK;
	p->disb_ptsa_rate = fraction2dda_fp(disb_frac_fp, la_soc->dda_div,
					    MC_PTSA_RATE_DEFAULT_MASK) &
			    MC_PTSA_RATE_DEFAULT_MASK;

	p->ve_ptsa_min =
		   la_soc->agg_camera_client[TEGRA_LA_AGG_CAMERA_VE].ptsa_min &
		   MC_PTSA_MIN_DEFAULT_MASK;
	p->ve_ptsa_max =
		   la_soc->agg_camera_client[TEGRA_LA_AGG_CAMERA_VE].ptsa_max &
		   MC_PTSA_MAX_DEFAULT_MASK;
	p->ve_ptsa_rate = fraction2dda_fp(
		     la_soc->agg_camera_client[TEGRA_LA_AGG_CAMERA_VE].frac_fp,
		     la_soc->dda_div, MC_PTSA_RATE_DEFAULT_MASK) &
		     MC_PTSA_RATE_DEFAULT_MASK;

	p->ve2_ptsa_min =
		  la_soc->agg_camera_client[TEGRA_LA_AGG_CAMERA_VE2].ptsa_min &
		  MC_PTSA_MIN_DEFAULT_MASK;
	p->ve2_ptsa_max =
		  la_soc->agg_camera_client[TEGRA_LA_AGG_CAMERA_VE2].ptsa_max &
		  MC_PTSA_MAX_DEFAULT_MASK;
	p->ve2_ptsa_rate = fraction2dda_fp(
		    la_soc->agg_camera_client[TEGRA_LA_AGG_CAMERA_VE2].frac_fp,
		    la_soc->dda_div, MC_PTSA_RATE_DEFAULT_MASK) &
		    MC_PTSA_RATE_DEFAULT_MASK;

	p->isp_ptsa_min =
		  la_soc->agg_camera_client[TEGRA_LA_AGG_CAMERA_ISP].ptsa_min &
		  MC_PTSA_MIN_DEFAULT_MASK;
	p->isp_ptsa_max =
		  la_soc->agg_camera_client[TEGRA_LA_AGG_CAMERA_ISP].ptsa_max &
		  MC_PTSA_MAX_DEFAULT_MASK;
	p->isp_ptsa_rate = fraction2dda_fp(
		    la_soc->agg_camera_client[TEGRA_LA_AGG_CAMERA_ISP].frac_fp,
		    la_soc->dda_div, MC_PTSA_RATE_DEFAULT_MASK) &
		    MC_PTSA_RATE_DEFAULT_MASK;

	p->ring1_ptsa_min = -5 & MC_PTSA_MIN_DEFAULT_MASK;
	p->ring1_ptsa_max = max_max & MC_PTSA_MAX_DEFAULT_MASK;
	p->ring1_ptsa_rate = p->dis_ptsa_rate +
			     p->disb_ptsa_rate +
			     p->ve_ptsa_rate;
	p->ring1_ptsa_rate +=
		   la_soc->agg_camera_client[TEGRA_LA_AGG_CAMERA_VE2].is_hiso ?
		   p->ve2_ptsa_rate : 0;
	p->ring1_ptsa_rate +=
		   la_soc->agg_camera_client[TEGRA_LA_AGG_CAMERA_ISP].is_hiso ?
		   p->isp_ptsa_rate : 0;

	if (mc->dram_type == DRAM_TYPE_LPDDR4)
		p->ring1_ptsa_rate /= 2;

	p->ring1_ptsa_rate += mc_readl(mc, MC_MLL_MPCORER_PTSA_RATE) +
			      mc_readl(mc, MC_FTOP_PTSA_RATE);

	if (!p->ring1_ptsa_rate)
		p->ring1_ptsa_rate = 1;
}

static void tegra210_update_display_ptsa_rate(struct tegra_mc *mc,
				       struct tegra_dc_to_la_params disp_params)
{
	struct tegra_ptsa_info *p = mc->soc->la_soc->ptsa_info;

	mutex_lock(&mc->soc->la_soc->la_ptsa_lock);
	mc->soc->la_soc->total_dc0_bw = DIV_ROUND_UP(disp_params.total_dc0_bw,
						     1000);
	mc->soc->la_soc->total_dc1_bw = DIV_ROUND_UP(disp_params.total_dc1_bw,
						     1000);

	calc_disp_and_camera_ptsa(mc);

	mc_writel(mc, p->ring1_ptsa_min, MC_RING1_PTSA_MIN);
	mc_writel(mc, p->ring1_ptsa_max, MC_RING1_PTSA_MAX);
	mc_writel(mc, p->ring1_ptsa_rate, MC_RING1_PTSA_RATE);

	mc_writel(mc, p->dis_ptsa_min, MC_DIS_PTSA_MIN);
	mc_writel(mc, p->dis_ptsa_max, MC_DIS_PTSA_MAX);
	mc_writel(mc, p->dis_ptsa_rate, MC_DIS_PTSA_RATE);

	mc_writel(mc, p->disb_ptsa_min, MC_DISB_PTSA_MIN);
	mc_writel(mc, p->disb_ptsa_max, MC_DISB_PTSA_MAX);
	mc_writel(mc, p->disb_ptsa_rate, MC_DISB_PTSA_RATE);

	mutex_unlock(&mc->soc->la_soc->la_ptsa_lock);
}

static int tegra210_update_camera_ptsa_rate(struct tegra_mc *mc,
					    int id,
					    unsigned int bw_kbps,
					    int is_hiso)
{
	struct tegra_la_soc *la_soc = mc->soc->la_soc;
	struct tegra_ptsa_info *p = la_soc->ptsa_info;
	int index = id_lookup(mc, id);

	if (la_soc->clients[index].la.type != TEGRA_LA_CLIENT_TYPE_CAMERA) {
		pr_err("%s: Ignoring request from a non-camera client.\n",
			__func__);
		return -EINVAL;
	}

	mutex_lock(&la_soc->la_ptsa_lock);
	if (id == TEGRA_CLIENT_VIW) {
		is_hiso = 1;
		la_soc->agg_camera_client[TEGRA_LA_AGG_CAMERA_VE].is_hiso = 1;
	} else if ((id == TEGRA_CLIENT_ISPRAB) ||
		   (id == TEGRA_CLIENT_ISPWAB) ||
		   (id == TEGRA_CLIENT_ISPWBB)) {
		la_soc->agg_camera_client[TEGRA_LA_AGG_CAMERA_VE2].is_hiso =
									is_hiso;
	} else {
		la_soc->agg_camera_client[TEGRA_LA_AGG_CAMERA_ISP].is_hiso =
									is_hiso;
	}

	la_soc->clients[index].la.camera_bw = DIV_ROUND_UP(bw_kbps, 1000);

	calc_disp_and_camera_ptsa(mc);

	mc_writel(mc, p->ring1_ptsa_min, MC_RING1_PTSA_MIN);
	mc_writel(mc, p->ring1_ptsa_max, MC_RING1_PTSA_MAX);
	mc_writel(mc, p->ring1_ptsa_rate, MC_RING1_PTSA_RATE);

	mc_writel(mc, p->ve_ptsa_min, MC_VE_PTSA_MIN);
	mc_writel(mc, p->ve_ptsa_max, MC_VE_PTSA_MAX);
	mc_writel(mc, p->ve_ptsa_rate, MC_VE_PTSA_RATE);

	mc_writel(mc, p->ve2_ptsa_min, MC_VE2_PTSA_MIN);
	mc_writel(mc, p->ve2_ptsa_max, MC_VE2_PTSA_MAX);
	mc_writel(mc, p->ve2_ptsa_rate, MC_VE2_PTSA_RATE);

	mc_writel(mc, p->isp_ptsa_min, MC_ISP_PTSA_MIN);
	mc_writel(mc, p->isp_ptsa_max, MC_ISP_PTSA_MAX);
	mc_writel(mc, p->isp_ptsa_rate, MC_ISP_PTSA_RATE);

	mutex_unlock(&la_soc->la_ptsa_lock);
	return 0;
}

static void tegra210_program_la(struct tegra_mc *mc, int la_client_idx,
				unsigned int value)
{
	struct tegra_mc_client *cli = &mc->soc->la_soc->clients[la_client_idx];
	unsigned int scale_val;

	la_writel(mc, value, &cli->la);

	if (cli->id == TEGRA_CLIENT_DISPLAY0A) {
		scale_val = ((value <<
			    MC_SCALED_LATENCY_ALLOWANCE_DISPLAY0A_LOW_SHIFT) &
			    MC_SCALED_LATENCY_ALLOWANCE_DISPLAY0A_LOW_MASK) |
			    ((value <<
			    MC_SCALED_LATENCY_ALLOWANCE_DISPLAY0A_HIGH_SHIFT) &
			    MC_SCALED_LATENCY_ALLOWANCE_DISPLAY0A_HIGH_MASK);
		mc_writel(mc, scale_val, MC_SCALED_LATENCY_ALLOWANCE_DISPLAY0A);
	} else if (cli->id == TEGRA_CLIENT_DISPLAY0AB) {
		scale_val = ((value <<
			    MC_SCALED_LATENCY_ALLOWANCE_DISPLAY0AB_LOW_SHIFT) &
			    MC_SCALED_LATENCY_ALLOWANCE_DISPLAY0AB_LOW_MASK) |
			    ((value <<
			    MC_SCALED_LATENCY_ALLOWANCE_DISPLAY0AB_HIGH_SHIFT)
			    & MC_SCALED_LATENCY_ALLOWANCE_DISPLAY0AB_HIGH_MASK);
		mc_writel(mc, scale_val,
			  MC_SCALED_LATENCY_ALLOWANCE_DISPLAY0AB);
	} else if (cli->id == TEGRA_CLIENT_DISPLAY0B) {
		scale_val = ((value <<
			    MC_SCALED_LATENCY_ALLOWANCE_DISPLAY0B_LOW_SHIFT) &
			    MC_SCALED_LATENCY_ALLOWANCE_DISPLAY0B_LOW_MASK) |
			    ((value <<
			    MC_SCALED_LATENCY_ALLOWANCE_DISPLAY0B_HIGH_SHIFT) &
			    MC_SCALED_LATENCY_ALLOWANCE_DISPLAY0B_HIGH_MASK);
		mc_writel(mc, scale_val, MC_SCALED_LATENCY_ALLOWANCE_DISPLAY0B);
	} else if (cli->id == TEGRA_CLIENT_DISPLAY0BB) {
		scale_val = ((value <<
			    MC_SCALED_LATENCY_ALLOWANCE_DISPLAY0BB_LOW_SHIFT) &
			    MC_SCALED_LATENCY_ALLOWANCE_DISPLAY0BB_LOW_MASK) |
			    ((value <<
			    MC_SCALED_LATENCY_ALLOWANCE_DISPLAY0BB_HIGH_SHIFT) &
			    MC_SCALED_LATENCY_ALLOWANCE_DISPLAY0BB_HIGH_MASK);
		mc_writel(mc, scale_val,
			  MC_SCALED_LATENCY_ALLOWANCE_DISPLAY0BB);
	} else if (cli->id == TEGRA_CLIENT_DISPLAY0C) {
		scale_val = ((value <<
			    MC_SCALED_LATENCY_ALLOWANCE_DISPLAY0C_LOW_SHIFT) &
			    MC_SCALED_LATENCY_ALLOWANCE_DISPLAY0C_LOW_MASK) |
			    ((value <<
			    MC_SCALED_LATENCY_ALLOWANCE_DISPLAY0C_HIGH_SHIFT) &
			    MC_SCALED_LATENCY_ALLOWANCE_DISPLAY0C_HIGH_MASK);
		mc_writel(mc, scale_val, MC_SCALED_LATENCY_ALLOWANCE_DISPLAY0C);
	} else if (cli->id == TEGRA_CLIENT_DISPLAY0CB) {
		scale_val = ((value <<
			    MC_SCALED_LATENCY_ALLOWANCE_DISPLAY0CB_LOW_SHIFT) &
			    MC_SCALED_LATENCY_ALLOWANCE_DISPLAY0CB_LOW_MASK) |
			    ((value <<
			    MC_SCALED_LATENCY_ALLOWANCE_DISPLAY0CB_HIGH_SHIFT) &
			    MC_SCALED_LATENCY_ALLOWANCE_DISPLAY0CB_HIGH_MASK);
		mc_writel(mc, scale_val,
			  MC_SCALED_LATENCY_ALLOWANCE_DISPLAY0CB);
	}
}

static unsigned int min_la(struct tegra_mc *mc,
			   struct tegra_dc_to_la_params *disp_params)
{
	unsigned int min_la_fp = disp_params->drain_time_usec_fp *
				1000 /
				mc->soc->la_soc->ns_per_tick;

	if (min_la_fp % LA_FP_FACTOR != 0)
		min_la_fp += LA_FP_FACTOR;

	return LA_FP_TO_REAL(min_la_fp);
}

static int calc_disp_la(struct tegra_mc *mc,
			int index,
			unsigned long emc_freq_hz,
			unsigned int bw_mbps,
			struct tegra_dc_to_la_params disp_params,
			unsigned int *ret_value)
{
	struct tegra_la_soc *la_soc = mc->soc->la_soc;
	unsigned int dvfs_time_nsec;
	unsigned int dvfs_buffering_reqd_bytes;
	unsigned int thresh_dvfs_bytes;
	unsigned int total_buf_sz_bytes;
	int effective_mccif_buf_sz;
	long long la_bw_upper_bound_nsec_fp;
	long long la_bw_upper_bound_nsec;
	long long la_nsec;
	struct tegra_mc_la *la = &la_soc->clients[index].la;
	long long value;

	if (la->type != TEGRA_LA_CLIENT_TYPE_DISPLAY) {
		pr_err("%s: Ignoring request from a non-display client.\n",
		       __func__);
		return -EINVAL;
	}

	tegra210_update_display_ptsa_rate(mc, disp_params);

	dvfs_time_nsec = tegra210_emc_get_clk_latency(emc_freq_hz);
	dvfs_buffering_reqd_bytes = bw_mbps * dvfs_time_nsec /
				    LA_USEC_TO_NSEC_FACTOR;

	thresh_dvfs_bytes = disp_params.thresh_lwm_bytes +
			    dvfs_buffering_reqd_bytes +
			    disp_params.spool_up_buffering_adj_bytes;

	total_buf_sz_bytes = la->disp_client.line_buf_sz_bytes +
			     la->disp_client.mccif_size_bytes;

	effective_mccif_buf_sz =
		      (la->disp_client.line_buf_sz_bytes > thresh_dvfs_bytes) ?
		      la->disp_client.mccif_size_bytes :
		      total_buf_sz_bytes - thresh_dvfs_bytes;

	if (effective_mccif_buf_sz < 0)
		return -EINVAL;

	la_bw_upper_bound_nsec_fp = effective_mccif_buf_sz *
				    LA_FP_FACTOR /
				    bw_mbps;
	la_bw_upper_bound_nsec_fp = la_bw_upper_bound_nsec_fp *
				    LA_FP_FACTOR /
				    LA_DISP_CATCHUP_FACTOR_FP;
	la_bw_upper_bound_nsec_fp = la_bw_upper_bound_nsec_fp -
				(LA_ST_LA_MINUS_SNAP_ARB_TO_ROW_SRT_EMCCLKS_FP +
				EXP_TIME_EMCCLKS_FP) /
				(emc_freq_hz / LA_HZ_TO_MHZ_FACTOR);
	la_bw_upper_bound_nsec_fp *= LA_USEC_TO_NSEC_FACTOR;
	la_bw_upper_bound_nsec = LA_FP_TO_REAL(la_bw_upper_bound_nsec_fp);

	la_nsec = min_t(long long, la_bw_upper_bound_nsec, MAX_LA_NSEC);

	value = min_t(long long, la_nsec/la_soc->ns_per_tick, MC_LA_MAX_VALUE);

	if ((value < min_la(mc, &disp_params)) || (value > 255))
		return -EINVAL;

	*ret_value = (unsigned int)value;
	return 0;
}

static int tegra210_set_disp_la(struct tegra_mc *mc,
				int id,
				unsigned long emc_freq_hz,
				unsigned int bw_kbps,
				struct tegra_dc_to_la_params disp_params)
{
	int idx = id_lookup(mc, id);
	int ret;
	unsigned int bw_mbps = DIV_ROUND_UP(bw_kbps, 1000);
	unsigned int value;

	ret = calc_disp_la(mc, idx, emc_freq_hz, bw_mbps, disp_params, &value);
	if (IS_ERR_VALUE(ret))
		return ret;

	tegra210_program_la(mc, idx, value);

	return 0;
}

static void tegra210_save_ptsa(struct tegra_mc *mc)
{
	struct tegra_ptsa_info *p = mc->soc->la_soc->ptsa_info;

	p->ptsa_grant_dec = mc_readl(mc, MC_PTSA_GRANT_DECREMENT);

	p->dis_ptsa_min = mc_readl(mc, MC_DIS_PTSA_MIN);
	p->dis_ptsa_max = mc_readl(mc, MC_DIS_PTSA_MAX);
	p->dis_ptsa_rate = mc_readl(mc, MC_DIS_PTSA_RATE);
	p->disb_ptsa_min = mc_readl(mc, MC_DISB_PTSA_MIN);
	p->disb_ptsa_max = mc_readl(mc, MC_DISB_PTSA_MAX);
	p->disb_ptsa_rate = mc_readl(mc, MC_DISB_PTSA_RATE);
	p->ve_ptsa_min = mc_readl(mc, MC_VE_PTSA_MIN);
	p->ve_ptsa_max = mc_readl(mc, MC_VE_PTSA_MAX);
	p->ve_ptsa_rate = mc_readl(mc, MC_VE_PTSA_RATE);
	p->ve2_ptsa_min = mc_readl(mc, MC_VE2_PTSA_MIN);
	p->ve2_ptsa_max = mc_readl(mc, MC_VE2_PTSA_MAX);
	p->ve2_ptsa_rate = mc_readl(mc, MC_VE2_PTSA_RATE);
	p->ring2_ptsa_min = mc_readl(mc, MC_RING2_PTSA_MIN);
	p->ring2_ptsa_max = mc_readl(mc, MC_RING2_PTSA_MAX);
	p->ring2_ptsa_rate = mc_readl(mc, MC_RING2_PTSA_RATE);
	p->mpcorer_ptsa_min = mc_readl(mc, MC_MLL_MPCORER_PTSA_MIN);
	p->mpcorer_ptsa_max = mc_readl(mc, MC_MLL_MPCORER_PTSA_MAX);
	p->mpcorer_ptsa_rate = mc_readl(mc, MC_MLL_MPCORER_PTSA_RATE);
	p->smmu_ptsa_min = mc_readl(mc, MC_SMMU_SMMU_PTSA_MIN);
	p->smmu_ptsa_max = mc_readl(mc, MC_SMMU_SMMU_PTSA_MAX);
	p->smmu_ptsa_rate = mc_readl(mc, MC_SMMU_SMMU_PTSA_RATE);
	p->ring1_ptsa_min = mc_readl(mc, MC_RING1_PTSA_MIN);
	p->ring1_ptsa_max = mc_readl(mc, MC_RING1_PTSA_MAX);
	p->ring1_ptsa_rate = mc_readl(mc, MC_RING1_PTSA_RATE);
	p->isp_ptsa_min = mc_readl(mc, MC_ISP_PTSA_MIN);
	p->isp_ptsa_max = mc_readl(mc, MC_ISP_PTSA_MAX);
	p->isp_ptsa_rate = mc_readl(mc, MC_ISP_PTSA_RATE);
	p->a9avppc_ptsa_min = mc_readl(mc, MC_A9AVPPC_PTSA_MIN);
	p->a9avppc_ptsa_max = mc_readl(mc, MC_A9AVPPC_PTSA_MAX);
	p->avp_ptsa_min = mc_readl(mc, MC_AVP_PTSA_MIN);
	p->avp_ptsa_max = mc_readl(mc, MC_AVP_PTSA_MAX);
	p->mse_ptsa_min = mc_readl(mc, MC_MSE_PTSA_MIN);
	p->mse_ptsa_max = mc_readl(mc, MC_MSE_PTSA_MAX);
	p->gk_ptsa_min = mc_readl(mc, MC_GK_PTSA_MIN);
	p->gk_ptsa_max = mc_readl(mc, MC_GK_PTSA_MAX);
	p->vicpc_ptsa_min = mc_readl(mc, MC_VICPC_PTSA_MIN);
	p->vicpc_ptsa_max = mc_readl(mc, MC_VICPC_PTSA_MAX);
	p->apb_ptsa_min = mc_readl(mc, MC_APB_PTSA_MIN);
	p->apb_ptsa_max = mc_readl(mc, MC_APB_PTSA_MAX);
	p->pcx_ptsa_min = mc_readl(mc, MC_PCX_PTSA_MIN);
	p->pcx_ptsa_max = mc_readl(mc, MC_PCX_PTSA_MAX);
	p->host_ptsa_min = mc_readl(mc, MC_HOST_PTSA_MIN);
	p->host_ptsa_max = mc_readl(mc, MC_HOST_PTSA_MAX);
	p->ahb_ptsa_min = mc_readl(mc, MC_AHB_PTSA_MIN);
	p->ahb_ptsa_max = mc_readl(mc, MC_AHB_PTSA_MAX);
	p->sax_ptsa_min = mc_readl(mc, MC_SAX_PTSA_MIN);
	p->sax_ptsa_max = mc_readl(mc, MC_SAX_PTSA_MAX);
	p->aud_ptsa_min = mc_readl(mc, MC_AUD_PTSA_MIN);
	p->aud_ptsa_max = mc_readl(mc, MC_AUD_PTSA_MAX);
	p->sd_ptsa_min = mc_readl(mc, MC_SD_PTSA_MIN);
	p->sd_ptsa_max = mc_readl(mc, MC_SD_PTSA_MAX);
	p->usbx_ptsa_min = mc_readl(mc, MC_USBX_PTSA_MIN);
	p->usbx_ptsa_max = mc_readl(mc, MC_USBX_PTSA_MAX);
	p->usbd_ptsa_min = mc_readl(mc, MC_USBD_PTSA_MIN);
	p->usbd_ptsa_max = mc_readl(mc, MC_USBD_PTSA_MAX);
	p->ftop_ptsa_min = mc_readl(mc, MC_FTOP_PTSA_MIN);
	p->ftop_ptsa_max = mc_readl(mc, MC_FTOP_PTSA_MAX);
}

static void tegra210_program_ptsa(struct tegra_mc *mc)
{
	struct tegra_ptsa_info *p = mc->soc->la_soc->ptsa_info;

	mc_writel(mc, p->ptsa_grant_dec, MC_PTSA_GRANT_DECREMENT);
	mc_writel(mc, 1, MC_TIMING_CONTROL);

	mc_writel(mc, p->dis_ptsa_min, MC_DIS_PTSA_MIN);
	mc_writel(mc, p->dis_ptsa_max, MC_DIS_PTSA_MAX);
	mc_writel(mc, p->dis_ptsa_rate, MC_DIS_PTSA_RATE);
	mc_writel(mc, p->disb_ptsa_min, MC_DISB_PTSA_MIN);
	mc_writel(mc, p->disb_ptsa_max, MC_DISB_PTSA_MAX);
	mc_writel(mc, p->disb_ptsa_rate, MC_DISB_PTSA_RATE);
	mc_writel(mc, p->ve_ptsa_min, MC_VE_PTSA_MIN);
	mc_writel(mc, p->ve_ptsa_max, MC_VE_PTSA_MAX);
	mc_writel(mc, p->ve_ptsa_rate, MC_VE_PTSA_RATE);
	mc_writel(mc, p->ve2_ptsa_min, MC_VE2_PTSA_MIN);
	mc_writel(mc, p->ve2_ptsa_max, MC_VE2_PTSA_MAX);
	mc_writel(mc, p->ve2_ptsa_rate, MC_VE2_PTSA_RATE);
	mc_writel(mc, p->ring2_ptsa_min, MC_RING2_PTSA_MIN);
	mc_writel(mc, p->ring2_ptsa_max, MC_RING2_PTSA_MAX);
	mc_writel(mc, p->ring2_ptsa_rate, MC_RING2_PTSA_RATE);
	mc_writel(mc, p->mpcorer_ptsa_min, MC_MLL_MPCORER_PTSA_MIN);
	mc_writel(mc, p->mpcorer_ptsa_max, MC_MLL_MPCORER_PTSA_MAX);
	mc_writel(mc, p->mpcorer_ptsa_rate, MC_MLL_MPCORER_PTSA_RATE);
	mc_writel(mc, p->smmu_ptsa_min, MC_SMMU_SMMU_PTSA_MIN);
	mc_writel(mc, p->smmu_ptsa_max, MC_SMMU_SMMU_PTSA_MAX);
	mc_writel(mc, p->smmu_ptsa_rate, MC_SMMU_SMMU_PTSA_RATE);
	mc_writel(mc, p->ring1_ptsa_min, MC_RING1_PTSA_MIN);
	mc_writel(mc, p->ring1_ptsa_max, MC_RING1_PTSA_MAX);
	mc_writel(mc, p->ring1_ptsa_rate, MC_RING1_PTSA_RATE);
	mc_writel(mc, p->isp_ptsa_min, MC_ISP_PTSA_MIN);
	mc_writel(mc, p->isp_ptsa_max, MC_ISP_PTSA_MAX);
	mc_writel(mc, p->isp_ptsa_rate, MC_ISP_PTSA_RATE);
	mc_writel(mc, p->a9avppc_ptsa_min, MC_A9AVPPC_PTSA_MIN);
	mc_writel(mc, p->a9avppc_ptsa_max, MC_A9AVPPC_PTSA_MAX);
	mc_writel(mc, p->avp_ptsa_min, MC_AVP_PTSA_MIN);
	mc_writel(mc, p->avp_ptsa_max, MC_AVP_PTSA_MAX);
	mc_writel(mc, p->mse_ptsa_min, MC_MSE_PTSA_MIN);
	mc_writel(mc, p->mse_ptsa_max, MC_MSE_PTSA_MAX);
	mc_writel(mc, p->gk_ptsa_min, MC_GK_PTSA_MIN);
	mc_writel(mc, p->gk_ptsa_max, MC_GK_PTSA_MAX);
	mc_writel(mc, p->vicpc_ptsa_min, MC_VICPC_PTSA_MIN);
	mc_writel(mc, p->vicpc_ptsa_max, MC_VICPC_PTSA_MAX);
	mc_writel(mc, p->apb_ptsa_min, MC_APB_PTSA_MIN);
	mc_writel(mc, p->apb_ptsa_max, MC_APB_PTSA_MAX);
	mc_writel(mc, p->pcx_ptsa_min, MC_PCX_PTSA_MIN);
	mc_writel(mc, p->pcx_ptsa_max, MC_PCX_PTSA_MAX);
	mc_writel(mc, p->host_ptsa_min, MC_HOST_PTSA_MIN);
	mc_writel(mc, p->host_ptsa_max, MC_HOST_PTSA_MAX);
	mc_writel(mc, p->ahb_ptsa_min, MC_AHB_PTSA_MIN);
	mc_writel(mc, p->ahb_ptsa_max, MC_AHB_PTSA_MAX);
	mc_writel(mc, p->sax_ptsa_min, MC_SAX_PTSA_MIN);
	mc_writel(mc, p->sax_ptsa_max, MC_SAX_PTSA_MAX);
	mc_writel(mc, p->aud_ptsa_min, MC_AUD_PTSA_MIN);
	mc_writel(mc, p->aud_ptsa_max, MC_AUD_PTSA_MAX);
	mc_writel(mc, p->sd_ptsa_min, MC_SD_PTSA_MIN);
	mc_writel(mc, p->sd_ptsa_max, MC_SD_PTSA_MAX);
	mc_writel(mc, p->usbx_ptsa_min, MC_USBX_PTSA_MIN);
	mc_writel(mc, p->usbx_ptsa_max, MC_USBX_PTSA_MAX);
	mc_writel(mc, p->usbd_ptsa_min, MC_USBD_PTSA_MIN);
	mc_writel(mc, p->usbd_ptsa_max, MC_USBD_PTSA_MAX);
	mc_writel(mc, p->ftop_ptsa_min, MC_FTOP_PTSA_MIN);
	mc_writel(mc, p->ftop_ptsa_max, MC_FTOP_PTSA_MAX);
}

static void tegra210_init_ptsa(struct tegra_mc *mc)
{
	struct tegra_la_soc *la_soc = mc->soc->la_soc;
	struct tegra_ptsa_info *p = la_soc->ptsa_info;
	unsigned int emc_freq_mhz;
	unsigned int mc_freq_mhz;
	unsigned int same_freq;
	unsigned int cpu_rd_bw, cpu_wr_bw;
	unsigned int gd_int, gd_frac_fp;
	int gd_fpa;

	emc_freq_mhz = clk_get_rate(mc->emc_clk) / LA_HZ_TO_MHZ_FACTOR;

	same_freq = mc_readl(mc, MC_EMEM_ARB_MISC0) &&
		MC_EMEM_ARB_MISC0_MC_EMC_SAME_FREQ_BIT;
	mc_freq_mhz = same_freq ? emc_freq_mhz : emc_freq_mhz / 2;

	gd_fpa = (LA_FP_TO_FPA(la_soc->lo_gd_fp) * emc_freq_mhz) /
		 la_soc->emc_min_freq_mhz;
	if (gd_fpa >= LA_REAL_TO_FPA(1)) {
		gd_int = 1;
		gd_fpa -= LA_REAL_TO_FPA(1);
	} else {
		gd_int = 0;
	}

	gd_frac_fp = __fraction2dda_fp(gd_fpa, 1, MC_PTSA_RATE_DEFAULT_MASK);
	p->ptsa_grant_dec = (gd_int << 12) | gd_frac_fp;

	cpu_rd_bw = (emc_freq_mhz * 16 * CPU_RD_BW_PERC) / 100;
	if (mc->dram_type == DRAM_TYPE_LPDDR4)
		cpu_rd_bw /= 2;

	p->mpcorer_ptsa_rate = __fraction2dda_fp(la_soc->lo_gd_fpa * cpu_rd_bw /
						 la_soc->low_freq_bw,
						 la_soc->dda_div,
						 MC_PTSA_RATE_DEFAULT_MASK);

	cpu_wr_bw = (emc_freq_mhz * 16 * CPU_WR_BW_PERC) / 100;
	if (mc->dram_type == DRAM_TYPE_LPDDR4)
		cpu_rd_bw /= 2;

	p->ftop_ptsa_rate = __fraction2dda_fp(la_soc->lo_gd_fpa * cpu_wr_bw /
					      la_soc->low_freq_bw,
					      la_soc->dda_div,
					      MC_PTSA_RATE_DEFAULT_MASK);

	p->ring1_ptsa_rate =  p->mpcorer_ptsa_rate + p->ftop_ptsa_rate;
	p->ring1_ptsa_rate += p->dis_ptsa_rate + p->disb_ptsa_rate;
	p->ring1_ptsa_rate += p->ve_ptsa_rate + p->ring2_ptsa_rate;

	tegra210_program_ptsa(mc);
}

static void tegra210_la_suspend(struct tegra_mc *mc)
{
	int i;
	struct tegra_la_soc *la_soc = mc->soc->la_soc;
	struct tegra_mc_la *la;

	for (i = 0; i < la_soc->num_clients; i++) {
		la = &la_soc->clients[i].la;
		la->la_set = (mc_readl(mc, la->reg) >> la->shift) & la->mask;
	}

	tegra210_save_ptsa(mc);
}

static void tegra210_la_resume(struct tegra_mc *mc)
{
	int i;
	struct tegra_la_soc *la_soc = mc->soc->la_soc;
	struct tegra_mc_la *la;

	for (i = 0; i < la_soc->num_clients; i++) {
		la = &la_soc->clients[i].la;
		if (la->la_set)
			tegra210_program_la(mc, i, la->la_set);
	}

	tegra210_program_ptsa(mc);
}

static struct tegra_la_soc tegra210_la_soc = {
	.clients = tegra210_mc_clients,
	.num_clients = ARRAY_SIZE(tegra210_mc_clients),
	.ns_per_tick = 30,
	.la_max_value = MC_LA_MAX_VALUE,
	.ptsa_info = &tegra210_ptsa_info,
	.init_la = &tegra210_init_la,
	.init_ptsa = &tegra210_init_ptsa,
	.update_camera_ptsa_rate = tegra210_update_camera_ptsa_rate,
	.set_disp_la = tegra210_set_disp_la,
	.la_suspend = tegra210_la_suspend,
	.la_resume = tegra210_la_resume,
};

static const struct tegra_smmu_swgroup tegra210_swgroups[] = {
	{ .name = "dc",        .swgroup = TEGRA_SWGROUP_DC,        .reg = 0x240 },
	{ .name = "dcb",       .swgroup = TEGRA_SWGROUP_DCB,       .reg = 0x244 },
	{ .name = "afi",       .swgroup = TEGRA_SWGROUP_AFI,       .reg = 0x238 },
	{ .name = "avpc",      .swgroup = TEGRA_SWGROUP_AVPC,      .reg = 0x23c },
	{ .name = "hda",       .swgroup = TEGRA_SWGROUP_HDA,       .reg = 0x254 },
	{ .name = "hc",        .swgroup = TEGRA_SWGROUP_HC,        .reg = 0x250 },
	{ .name = "nvenc",     .swgroup = TEGRA_SWGROUP_NVENC,     .reg = 0x264 },
	{ .name = "ppcs",      .swgroup = TEGRA_SWGROUP_PPCS,      .reg = 0x270 },
	{ .name = "sata",      .swgroup = TEGRA_SWGROUP_SATA,      .reg = 0x274 },
	{ .name = "isp2",      .swgroup = TEGRA_SWGROUP_ISP2,      .reg = 0x258 },
	{ .name = "xusb_host", .swgroup = TEGRA_SWGROUP_XUSB_HOST, .reg = 0x288 },
	{ .name = "xusb_dev",  .swgroup = TEGRA_SWGROUP_XUSB_DEV,  .reg = 0x28c },
	{ .name = "isp2b",     .swgroup = TEGRA_SWGROUP_ISP2B,     .reg = 0xaa4 },
	{ .name = "tsec",      .swgroup = TEGRA_SWGROUP_TSEC,      .reg = 0x294 },
	{ .name = "a9avp",     .swgroup = TEGRA_SWGROUP_A9AVP,     .reg = 0x290 },
	{ .name = "gpu",       .swgroup = TEGRA_SWGROUP_GPU,       .reg = 0xaac },
	{ .name = "sdmmc1a",   .swgroup = TEGRA_SWGROUP_SDMMC1A,   .reg = 0xa94 },
	{ .name = "sdmmc2a",   .swgroup = TEGRA_SWGROUP_SDMMC2A,   .reg = 0xa98 },
	{ .name = "sdmmc3a",   .swgroup = TEGRA_SWGROUP_SDMMC3A,   .reg = 0xa9c },
	{ .name = "sdmmc4a",   .swgroup = TEGRA_SWGROUP_SDMMC4A,   .reg = 0xaa0 },
	{ .name = "vic",       .swgroup = TEGRA_SWGROUP_VIC,       .reg = 0x284 },
	{ .name = "vi",        .swgroup = TEGRA_SWGROUP_VI,        .reg = 0x280 },
	{ .name = "nvdec",     .swgroup = TEGRA_SWGROUP_NVDEC,     .reg = 0xab4 },
	{ .name = "ape",       .swgroup = TEGRA_SWGROUP_APE,       .reg = 0xab8 },
	{ .name = "nvjpg",     .swgroup = TEGRA_SWGROUP_NVJPG,     .reg = 0xac0 },
	{ .name = "se",        .swgroup = TEGRA_SWGROUP_SE,        .reg = 0xabc },
	{ .name = "axiap",     .swgroup = TEGRA_SWGROUP_AXIAP,     .reg = 0xacc },
	{ .name = "etr",       .swgroup = TEGRA_SWGROUP_ETR,       .reg = 0xad0 },
	{ .name = "tsecb",     .swgroup = TEGRA_SWGROUP_TSECB,     .reg = 0xad4 },
};

static const struct tegra_mc_flush tegra210_mc_flush[] = {
	{TEGRA_SWGROUP_AFI,        0x200, 0x204,  0},
	{TEGRA_SWGROUP_AVPC,       0x200, 0x204,  1},
	{TEGRA_SWGROUP_DC,         0x200, 0x204,  2},
	{TEGRA_SWGROUP_DCB,        0x200, 0x204,  3},
	{TEGRA_SWGROUP_HC,         0x200, 0x204,  6},
	{TEGRA_SWGROUP_HDA,        0x200, 0x204,  7},
	{TEGRA_SWGROUP_ISP2,       0x200, 0x204,  8},
	{TEGRA_SWGROUP_MPCORE,     0x200, 0x204,  9},
	{TEGRA_SWGROUP_NVENC,      0x200, 0x204, 11},
	{TEGRA_SWGROUP_PPCS,       0x200, 0x204, 14},
	{TEGRA_SWGROUP_SATA,       0x200, 0x204, 15},
	{TEGRA_SWGROUP_VI,         0x200, 0x204, 17},
	{TEGRA_SWGROUP_VIC,        0x200, 0x204, 18},
	{TEGRA_SWGROUP_XUSB_HOST,  0x200, 0x204, 19},
	{TEGRA_SWGROUP_XUSB_DEV,   0x200, 0x204, 20},
	{TEGRA_SWGROUP_A9AVP,      0x200, 0x204, 21},
	{TEGRA_SWGROUP_TSEC,       0x200, 0x204, 22},
	{TEGRA_SWGROUP_SDMMC1A,    0x200, 0x204, 29},
	{TEGRA_SWGROUP_SDMMC2A,    0x200, 0x204, 30},
	{TEGRA_SWGROUP_SDMMC3A,    0x200, 0x204, 31},
	{TEGRA_SWGROUP_SDMMC4A,    0x970, 0x974,  0},
	{TEGRA_SWGROUP_ISP2B,      0x970, 0x974,  1},
	{TEGRA_SWGROUP_GPU,        0x970, 0x974,  2},
	{TEGRA_SWGROUP_NVDEC,      0x970, 0x974,  5},
	{TEGRA_SWGROUP_APE,        0x970, 0x974,  6},
	{TEGRA_SWGROUP_SE,         0x970, 0x974,  7},
	{TEGRA_SWGROUP_NVJPG,      0x970, 0x974,  8},
	{TEGRA_SWGROUP_TSECB,      0x970, 0x974, 13},
};

static const struct tegra_smmu_soc tegra210_smmu_soc = {
	.clients = tegra210_mc_clients,
	.num_clients = ARRAY_SIZE(tegra210_mc_clients),
	.swgroups = tegra210_swgroups,
	.num_swgroups = ARRAY_SIZE(tegra210_swgroups),
	.supports_round_robin_arbitration = true,
	.supports_request_limit = true,
	.num_tlb_lines = 48,
	.num_asids = 128,
};

const struct tegra_mc_soc tegra210_mc_soc = {
	.clients = tegra210_mc_clients,
	.num_clients = ARRAY_SIZE(tegra210_mc_clients),
	.num_address_bits = 34,
	.atom_size = 64,
	.smmu = &tegra210_smmu_soc,
	.flush_unstable = true,
	.flushes = tegra210_mc_flush,
	.num_flushes = ARRAY_SIZE(tegra210_mc_flush),
	.la_soc = &tegra210_la_soc,
};
