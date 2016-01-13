/*
 * Copyright (c) 2015, NVIDIA CORPORATION. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */
/* ACRNONYMS USED:
 * LSF - Low Secure Falcon (i.e. PMU and FECS)
 * LSFM- Low Secure Falcon Manager
 * ACR - Access Controlled Region
 * WPR - Write Protected Region
 * LSB Header - Low Secure Boot Header(For LS falocons i.e PMU and FECS)
 * BL  - Bootloader
 * HS  - High Secure
 *
 * ABOUT THIS FILE (Nouveau Secure Boot)
 * LS falcons in below description refers to PMU and FECS.
 *
 * For secure boot feature,there is a pre-requisite that, we need a ucode blob,
 * in which we copy each LS falon's ucode, BL and signatures to a particular
 * location and provide this location address and size to the HS (High Secure)
 * falcon ucode.
 * This High Secure(HS) ucode, called the ACR ucode,copies the entire ucode blob
 * to a pre-defined WPR and verifies the ucodes with their signatures for each
 * LS falcon.
 * If verification for a particular falcon goes well, ACR ucode loads memory of
 * that particular falcon with corresponding BL and other BL params.
 * */
#include "priv.h"
#include "gk20a.h"
#include <core/gpuobj.h>
#include <core/device.h>
#include <subdev/fb.h>
#include <subdev/mc.h>
#include <linux/firmware.h>
#include <subdev/timer.h>

struct gm20b_ctxsw_bootloader_desc {
	u32 start_offset;
	u32 size;
	u32 imem_offset;
	u32 entry_point;
};

/*Defines*/
#define TEGRA_MC_BASE                           0x70019000
#define MC_SECURITY_CARVEOUT2_BOM_0		0xc5c
#define MC_SECURITY_CARVEOUT3_BOM_0		0xcac
#define MC_ERR_GENERALIZED_CARVEOUT_STATUS_0	0xc00

/*chip specific defines*/
#define MAX_SUPPORTED_LSFM 2 /*PMU & FECS*/
#define LSF_UCODE_DATA_ALIGNMENT 4096

/*Firmware Names*/
#define GM20B_PMU_UCODE_IMAGE "gpmu_ucode_image.bin"
#define GM20B_PMU_UCODE_DESC "gpmu_ucode_desc.bin"
#define GM20B_PMU_UCODE_SIG "pmu_sig.bin"
#define GM20B_FECS_UCODE_SIG "fecs_sig.bin"
#define GK20A_FECS_UCODE_IMAGE  "fecs.bin"
#define GM20B_HSBIN_PMU_UCODE_IMAGE "acr_ucode.bin"
#define GM20B_HSBIN_PMU_BL_UCODE_IMAGE "pmu_bl.bin"

#define PMU_SECURE_MODE (0x1)
#define PMU_LSFM_MANAGED (0x2)
/* defined by pmu hw spec */
#define GK20A_PMU_VA_SIZE		(512 * 1024 * 1024)
#define GK20A_PMU_UCODE_SIZE_MAX	(256 * 1024)
/* idle timeout */
#define GK20A_IDLE_CHECK_DEFAULT	10000 /* usec */
#define GK20A_IDLE_CHECK_MAX		5000 /* usec */

#define GK20A_PMU_DMEM_BLKSIZE2		    8
#define GK20A_PMU_UCODE_NB_MAX_OVERLAY	    32
#define GK20A_PMU_UCODE_NB_MAX_DATE_LENGTH  64

#define T210_FLCN_ACR_MAX_REGIONS                  (2)
#define LSF_BOOTSTRAP_OWNER_RESERVED_DMEM_SIZE   (0x200)
/*
 * Falcon Id Defines
 * Defines a common Light Secure Falcon identifier.
 */
#define LSF_FALCON_ID_PMU       (0)
#define LSF_FALCON_ID_FECS      (2)
#define LSF_FALCON_ID_INVALID   (0xFFFFFFFF)

/*Bootstrap Owner Defines*/
#define LSF_BOOTSTRAP_OWNER_DEFAULT (LSF_FALCON_ID_PMU)

/*Image Status Defines*/
#define LSF_IMAGE_STATUS_NONE                           (0)
#define LSF_IMAGE_STATUS_COPY                           (1)
#define LSF_IMAGE_STATUS_VALIDATION_CODE_FAILED         (2)
#define LSF_IMAGE_STATUS_VALIDATION_DATA_FAILED         (3)
#define LSF_IMAGE_STATUS_VALIDATION_DONE                (4)
#define LSF_IMAGE_STATUS_VALIDATION_SKIPPED             (5)
#define LSF_IMAGE_STATUS_BOOTSTRAP_READY                (6)

/*LSB header related defines*/
#define NV_FLCN_ACR_LSF_FLAG_LOAD_CODE_AT_0_FALSE       0
#define NV_FLCN_ACR_LSF_FLAG_LOAD_CODE_AT_0_TRUE        1
#define NV_FLCN_ACR_LSF_FLAG_DMACTL_REQ_CTX_FALSE       0
#define NV_FLCN_ACR_LSF_FLAG_DMACTL_REQ_CTX_TRUE        4

/*Light Secure WPR Content Alignments*/
#define LSF_LSB_HEADER_ALIGNMENT    256
#define LSF_BL_DATA_ALIGNMENT       256
#define LSF_BL_DATA_SIZE_ALIGNMENT  256
#define LSF_BL_CODE_SIZE_ALIGNMENT  256

/*Falcon UCODE header index.*/
#define FLCN_NL_UCODE_HDR_OS_CODE_OFF_IND              (0)
#define FLCN_NL_UCODE_HDR_OS_CODE_SIZE_IND             (1)
#define FLCN_NL_UCODE_HDR_OS_DATA_OFF_IND              (2)
#define FLCN_NL_UCODE_HDR_OS_DATA_SIZE_IND             (3)
#define FLCN_NL_UCODE_HDR_NUM_APPS_IND                 (4)
/*
 * There are total N number of Apps with code and offset defined in UCODE header
 * This macro provides the CODE and DATA offset and size of Ath application.
 */
#define FLCN_NL_UCODE_HDR_APP_CODE_START_IND           (5)
#define FLCN_NL_UCODE_HDR_APP_CODE_OFF_IND(N, A) \
	(FLCN_NL_UCODE_HDR_APP_CODE_START_IND + (A*2))
#define FLCN_NL_UCODE_HDR_APP_CODE_SIZE_IND(N, A) \
	(FLCN_NL_UCODE_HDR_APP_CODE_START_IND + (A*2) + 1)
#define FLCN_NL_UCODE_HDR_APP_CODE_END_IND(N) \
	(FLCN_NL_UCODE_HDR_APP_CODE_START_IND + (N*2) - 1)

#define FLCN_NL_UCODE_HDR_APP_DATA_START_IND(N) \
	(FLCN_NL_UCODE_HDR_APP_CODE_END_IND(N) + 1)
#define FLCN_NL_UCODE_HDR_APP_DATA_OFF_IND(N, A) \
	(FLCN_NL_UCODE_HDR_APP_DATA_START_IND(N) + (A*2))
#define FLCN_NL_UCODE_HDR_APP_DATA_SIZE_IND(N, A) \
	(FLCN_NL_UCODE_HDR_APP_DATA_START_IND(N) + (A*2) + 1)
#define FLCN_NL_UCODE_HDR_APP_DATA_END_IND(N) \
	(FLCN_NL_UCODE_HDR_APP_DATA_START_IND(N) + (N*2) - 1)

#define FLCN_NL_UCODE_HDR_OS_OVL_OFF_IND(N) \
	(FLCN_NL_UCODE_HDR_APP_DATA_END_IND(N) + 1)
#define FLCN_NL_UCODE_HDR_OS_OVL_SIZE_IND(N) \
	(FLCN_NL_UCODE_HDR_APP_DATA_END_IND(N) + 2)

struct lsf_ucode_desc {
	u8  prd_keys[2][16];
	u8  dbg_keys[2][16];
	u32 b_prd_present;
	u32 b_dbg_present;
	u32 falcon_id;
};
/*
 * Light Secure WPR Header
 * Defines state allowing Light Secure Falcon bootstrapping.
 *
 * falcon_id       - LS falcon ID
 * lsb_offset      - Offset into WPR region holding LSB header
 * bootstrap_owner - Bootstrap OWNER
 * lazy_bootstrap  - Skip bootstrapping by ACR
 * status          - Bootstrapping status
 */
struct lsf_wpr_header {
	u32  falcon_id;
	u32  lsb_offset;
	u32  bootstrap_owner;
	u32  lazy_bootstrap;
	u32  status;
};

struct pmu_mem_v1 {
	u32 dma_base;
	u8  dma_offset;
	u8  dma_idx;
	u16 fb_size;
};

struct pmu_cmdline_args_v1 {
	u32 reserved;
	u32 cpu_freq_hz;		/* Frequency of the clock driving PMU */
	u32 falc_trace_size;		/* falctrace buffer size (bytes) */
	u32 falc_trace_dma_base;	/* 256-byte block address */
	u32 falc_trace_dma_idx;		/* dmaIdx for DMA operations */
	u8 secure_mode;
	u8 raise_priv_sec;
	struct pmu_mem_v1 gc6_ctx;		/* dmem offset of gc6 context */
};

struct lsf_lsb_header {
	struct lsf_ucode_desc signature;
	u32 ucode_off;
	u32 ucode_size;
	u32 data_size;
	u32 bl_code_size;
	u32 bl_imem_off;
	u32 bl_data_off;
	u32 bl_data_size;
	u32 app_code_off;
	u32 app_code_size;
	u32 app_data_off;
	u32 app_data_size;
	u32 flags;
};

static void __iomem *mc;

/*Structure used by the bootloader*/
struct loader_config {
	u32 dma_idx;
	u32 code_dma_base;     /*upper 32-bits of 40-bit dma address*/
	u32 code_size_total;
	u32 code_size_to_load;
	u32 code_entry_point;
	u32 data_dma_base;    /*upper 32-bits of 40-bit dma address*/
	u32 data_size;        /*initialized data of the application */
	u32 overlay_dma_base; /*upper 32-bits of the 40-bit dma address*/
	u32 argc;
	u32 argv;
};

struct flcn_ucode_img {
	u32 *header;
	u32 *data;
	struct pmu_ucode_desc *desc;
	u32 data_size;
	void *fw_ver;
	u8 load_entire_os_data;
	struct lsf_ucode_desc *lsf_desc;
	u8 free_res_allocs;
	u32 flcn_inst;
};


/*ACR related structs*/
/*!
 * start_addr     - Starting address of region
 * end_addr       - Ending address of region
 * region_id      - Region ID
 * read_mask      - Read Mask
 * write_mask     - WriteMask
 * client_mask    - Bit map of all clients currently using this region
 */
struct flcn_acr_region_prop {
	u32   start_addr;
	u32   end_addr;
	u32   region_id;
	u32   read_mask;
	u32   write_mask;
	u32   client_mask;
};

/*!
 * no_regions   - Number of regions used.
 * region_props   - Region properties
 */
struct flcn_acr_regions {
	u32                     no_regions;
	struct flcn_acr_region_prop   region_props[T210_FLCN_ACR_MAX_REGIONS];
};

/*!
 * reserved_dmem-When the bootstrap owner has done bootstrapping other falcons,
 *                and need to switch into LS mode, it needs to have its own
 *                actual DMEM image copied into DMEM as part of LS setup. If
 *                ACR desc is at location 0, it will definitely get overwritten
 *                causing data corruption. Hence we are reserving 0x200 bytes
 *                to give room for any loading data. NOTE: This has to be the
 *                first member always
 * signature    - Signature of ACR ucode.
 * wpr_region_id - Region ID holding the WPR header and its details
 * wpr_offset    - Offset from the WPR region holding the wpr header
 * regions       - Region descriptors
 * nonwpr_ucode_blob_start -stores non-WPR start where kernel stores ucode blob
 * nonwpr_ucode_blob_end   -stores non-WPR end where kernel stores ucode blob
 */
struct flcn_acr_desc {
	union {
		u32 reserved_dmem[(LSF_BOOTSTRAP_OWNER_RESERVED_DMEM_SIZE/4)];
		u32 signatures[4];
	} ucode_reserved_space;
	/*Always 1st*/
	u32 wpr_region_id;
	u32 wpr_offset;
	u32 mmu_mem_range;
	struct flcn_acr_regions regions;
	u32 nonwpr_ucode_blob_size;
	u64 nonwpr_ucode_blob_start;
};
#define BLK_SIZE 256
/* Union of all supported structures used by bootloaders*/
union flcn_bl_generic_desc {
	struct flcn_bl_dmem_desc bl_dmem_desc;
	struct loader_config loader_cfg;
};

/*
 * LSFM Managed Ucode Image
 * next             : Next image in the list, NULL if last.
 * wpr_header         : WPR header for this ucode image
 * lsb_header         : LSB header for this ucode image
 * bl_gen_desc     : Bootloader generic desc structure for this ucode image
 * bl_gen_desc_size : Sizeof bootloader desc structure for this ucode image
 * full_ucode_size  : Surface size required for final ucode image
 * ucode_img        : Ucode image info
 */

struct lsfm_managed_ucode_img {
	struct lsfm_managed_ucode_img *next;
	struct lsf_wpr_header wpr_header;
	struct lsf_lsb_header lsb_header;
	union flcn_bl_generic_desc bl_gen_desc;
	u32 bl_gen_desc_size;
	u32 full_ucode_size;
	struct flcn_ucode_img ucode_img;
};

/*Structure to manage Low secure falcons*/
struct ls_flcn_mgr {
	u16 managed_flcn_cnt;
	u32 wpr_size;
	u32 disable_mask;
/*Pointer to linked list of managed ucode images*/
	struct lsfm_managed_ucode_img *ucode_img_list;
};

/*
 * Structure/object which single register write need to be done during PG init
 * sequence to set PROD values.
 */
struct pg_init_sequence_list {
	u32 regaddr;
	u32 writeval;
};

/* PROD settings for ELPG sequencing registers*/
static struct pg_init_sequence_list pg_init_seq_gm20b[] = {
	{ 0x0010ab10, 0x00008180 },
	{ 0x0010e118, 0x83828180 },
	{ 0x0010e068, 0x00000000 },
	{ 0x0010e06c, 0x00000080 },
	{ 0x0010e06c, 0x00000081 },
	{ 0x0010e06c, 0x00000082 },
	{ 0x0010e06c, 0x00000083 },
	{ 0x0010e06c, 0x00000084 },
	{ 0x0010e06c, 0x00000085 },
	{ 0x0010e06c, 0x00000086 },
	{ 0x0010e06c, 0x00000087 },
	{ 0x0010e06c, 0x00000088 },
	{ 0x0010e06c, 0x00000089 },
	{ 0x0010e06c, 0x0000008a },
	{ 0x0010e06c, 0x0000008b },
	{ 0x0010e06c, 0x0000008c },
	{ 0x0010e06c, 0x0000008d },
	{ 0x0010e06c, 0x0000008e },
	{ 0x0010e06c, 0x0000008f },
	{ 0x0010e06c, 0x00000090 },
	{ 0x0010e06c, 0x00000091 },
	{ 0x0010e06c, 0x00000092 },
	{ 0x0010e06c, 0x00000093 },
	{ 0x0010e06c, 0x00000094 },
	{ 0x0010e06c, 0x00000095 },
	{ 0x0010e06c, 0x00000096 },
	{ 0x0010e06c, 0x00000097 },
	{ 0x0010e06c, 0x00000098 },
	{ 0x0010e06c, 0x00000099 },
	{ 0x0010e06c, 0x0000009a },
	{ 0x0010e06c, 0x0000009b },
	{ 0x0010e06c, 0x00000000 },
	{ 0x0010e06c, 0x00000000 },
	{ 0x0010e06c, 0x00000000 },
	{ 0x0010e06c, 0x00000000 },
	{ 0x0010e06c, 0x00000000 },
	{ 0x0010e06c, 0x00000000 },
	{ 0x0010e06c, 0x00000000 },
	{ 0x0010e06c, 0x00000000 },
	{ 0x0010e06c, 0x00000000 },
	{ 0x0010e06c, 0x00000000 },
	{ 0x0010e06c, 0x00000000 },
	{ 0x0010e06c, 0x00000000 },
	{ 0x0010e06c, 0x00000000 },
	{ 0x0010e06c, 0x00000000 },
	{ 0x0010e06c, 0x00000000 },
	{ 0x0010e06c, 0x00000000 },
	{ 0x0010e06c, 0x00000000 },
	{ 0x0010e06c, 0x00000000 },
	{ 0x0010e06c, 0x00000000 },
	{ 0x0010e06c, 0x00000000 },
	{ 0x0010e06c, 0x00000000 },
	{ 0x0010e06c, 0x00000000 },
	{ 0x0010e06c, 0x00000000 },
	{ 0x0010e06c, 0x00000000 },
	{ 0x0010e06c, 0x00000000 },
	{ 0x0010e06c, 0x00000000 },
	{ 0x0010e06c, 0x00000000 },
	{ 0x0010e06c, 0x00000000 },
	{ 0x0010e06c, 0x00000000 },
	{ 0x0010e06c, 0x00000000 },
	{ 0x0010e06c, 0x00000000 },
	{ 0x0010e06c, 0x00000000 },
	{ 0x0010e06c, 0x00000000 },
	{ 0x0010e06c, 0x00000000 },
	{ 0x0010e06c, 0x00000000 },
	{ 0x0010e06c, 0x00000000 },
	{ 0x0010e06c, 0x00000000 },
	{ 0x0010e06c, 0x00000000 },
	{ 0x0010ab14, 0x00000000 },
	{ 0x0010ab18, 0x00000000 },
	{ 0x0010e024, 0x00000000 },
	{ 0x0010e028, 0x00000000 },
	{ 0x0010e11c, 0x00000000 },
	{ 0x0010e120, 0x00000000 },
	{ 0x0010ab1c, 0x02010155 },
	{ 0x0010e020, 0x001b1b55 },
	{ 0x0010e124, 0x01030355 },
	{ 0x0010ab20, 0x89abcdef },
	{ 0x0010ab24, 0x00000000 },
	{ 0x0010e02c, 0x89abcdef },
	{ 0x0010e030, 0x00000000 },
	{ 0x0010e128, 0x89abcdef },
	{ 0x0010e12c, 0x00000000 },
	{ 0x0010ab28, 0x74444444 },
	{ 0x0010ab2c, 0x70000000 },
	{ 0x0010e034, 0x74444444 },
	{ 0x0010e038, 0x70000000 },
	{ 0x0010e130, 0x74444444 },
	{ 0x0010e134, 0x70000000 },
	{ 0x0010ab30, 0x00000000 },
	{ 0x0010ab34, 0x00000001 },
	{ 0x00020004, 0x00000000 },
	{ 0x0010e138, 0x00000000 },
	{ 0x0010e040, 0x00000000 },
};

static int
gm20b_pmu_setup_elpg(struct nvkm_pmu *pmu)
{
	int ret = 0;
	u32 reg_writes;
	u32 index;

	reg_writes = ARRAY_SIZE(pg_init_seq_gm20b);
	/* Initialize registers with production values*/
	for (index = 0; index < reg_writes; index++)
		nv_wr32(pmu, pg_init_seq_gm20b[index].regaddr,
					pg_init_seq_gm20b[index].writeval);

	return ret;
}

/* slcg bus */
static const
struct gating_desc gm20b_slcg_bus[] = {
	{.addr = 0x00001c04, .prod = 0x00000000, .disable = 0x000003fe},
};

/* slcg ce2 */
static const
struct gating_desc gm20b_slcg_ce2[] = {
	{.addr = 0x00106f28, .prod = 0x00000000, .disable = 0x000007fe},
};

/* slcg chiplet */
static const
struct gating_desc gm20b_slcg_chiplet[] = {
	{.addr = 0x0010c07c, .prod = 0x00000000, .disable = 0x00000007},
	{.addr = 0x0010e07c, .prod = 0x00000000, .disable = 0x00000007},
	{.addr = 0x0010d07c, .prod = 0x00000000, .disable = 0x00000007},
	{.addr = 0x0010e17c, .prod = 0x00000000, .disable = 0x00000007},
};

/* slcg fb */
static const
struct gating_desc gm20b_slcg_fb[] = {
	{.addr = 0x00100d14, .prod = 0x00000000, .disable = 0xfffffffe},
	{.addr = 0x00100c9c, .prod = 0x00000000, .disable = 0x000001fe},
};

/* slcg fifo */
static const
struct gating_desc gm20b_slcg_fifo[] = {
	{.addr = 0x000026ac, .prod = 0x00000100, .disable = 0x0001fffe},
};

/* slcg gr */
static const
struct gating_desc gm20b_slcg_gr[] = {
	{.addr = 0x004041f4, .prod = 0x00000000, .disable = 0x03fffffe},
	{.addr = 0x0040917c, .prod = 0x00020008, .disable = 0x0003fffe},
	{.addr = 0x00409894, .prod = 0x00000040, .disable = 0x0003fffe},
	{.addr = 0x004078c4, .prod = 0x00000000, .disable = 0x000001fe},
	{.addr = 0x00406004, .prod = 0x00000000, .disable = 0x0001fffe},
	{.addr = 0x00405864, .prod = 0x00000000, .disable = 0x000001fe},
	{.addr = 0x00405910, .prod = 0xfffffff0, .disable = 0xfffffffe},
	{.addr = 0x00408044, .prod = 0x00000000, .disable = 0x000007fe},
	{.addr = 0x00407004, .prod = 0x00000000, .disable = 0x0000007e},
	{.addr = 0x0041a17c, .prod = 0x00020008, .disable = 0x0003fffe},
	{.addr = 0x0041a894, .prod = 0x00000040, .disable = 0x0003fffe},
	{.addr = 0x00418504, .prod = 0x00000000, .disable = 0x0007fffe},
	{.addr = 0x0041860c, .prod = 0x00000000, .disable = 0x000001fe},
	{.addr = 0x0041868c, .prod = 0x00000000, .disable = 0x0000001e},
	{.addr = 0x0041871c, .prod = 0x00000000, .disable = 0x0000003e},
	{.addr = 0x00418388, .prod = 0x00000000, .disable = 0x00000001},
	{.addr = 0x0041882c, .prod = 0x00000000, .disable = 0x0001fffe},
	{.addr = 0x00418bc0, .prod = 0x00000000, .disable = 0x000001fe},
	{.addr = 0x00418974, .prod = 0x00000000, .disable = 0x0001fffe},
	{.addr = 0x00418c74, .prod = 0xffffffc0, .disable = 0xfffffffe},
	{.addr = 0x00418cf4, .prod = 0xfffffffc, .disable = 0xfffffffe},
	{.addr = 0x00418d74, .prod = 0xffffffe0, .disable = 0xfffffffe},
	{.addr = 0x00418f10, .prod = 0xffffffe0, .disable = 0xfffffffe},
	{.addr = 0x00418e10, .prod = 0xfffffffe, .disable = 0xfffffffe},
	{.addr = 0x00419024, .prod = 0x000001fe, .disable = 0x000001fe},
	{.addr = 0x0041889c, .prod = 0x00000000, .disable = 0x000001fe},
	{.addr = 0x00419d64, .prod = 0x00000000, .disable = 0x000001ff},
	{.addr = 0x00419a44, .prod = 0x00000000, .disable = 0x0000000e},
	{.addr = 0x00419a4c, .prod = 0x00000000, .disable = 0x000001fe},
	{.addr = 0x00419a54, .prod = 0x00000000, .disable = 0x0000003e},
	{.addr = 0x00419a5c, .prod = 0x00000000, .disable = 0x0000000e},
	{.addr = 0x00419a64, .prod = 0x00000000, .disable = 0x000001fe},
	{.addr = 0x00419a6c, .prod = 0x00000000, .disable = 0x0000000e},
	{.addr = 0x00419a74, .prod = 0x00000000, .disable = 0x0000000e},
	{.addr = 0x00419a7c, .prod = 0x00000000, .disable = 0x0000003e},
	{.addr = 0x00419a84, .prod = 0x00000000, .disable = 0x0000000e},
	{.addr = 0x0041986c, .prod = 0x00000104, .disable = 0x00fffffe},
	{.addr = 0x00419cd8, .prod = 0x00000000, .disable = 0x001ffffe},
	{.addr = 0x00419ce0, .prod = 0x00000000, .disable = 0x001ffffe},
	{.addr = 0x00419c74, .prod = 0x0000001e, .disable = 0x0000001e},
	{.addr = 0x00419fd4, .prod = 0x00000000, .disable = 0x0003fffe},
	{.addr = 0x00419fdc, .prod = 0xffedff00, .disable = 0xfffffffe},
	{.addr = 0x00419fe4, .prod = 0x00001b00, .disable = 0x00001ffe},
	{.addr = 0x00419ff4, .prod = 0x00000000, .disable = 0x00003ffe},
	{.addr = 0x00419ffc, .prod = 0x00000000, .disable = 0x0001fffe},
	{.addr = 0x0041be2c, .prod = 0x04115fc0, .disable = 0xfffffffe},
	{.addr = 0x0041bfec, .prod = 0xfffffff0, .disable = 0xfffffffe},
	{.addr = 0x0041bed4, .prod = 0xfffffff6, .disable = 0xfffffffe},
	{.addr = 0x00408814, .prod = 0x00000000, .disable = 0x0001fffe},
	{.addr = 0x0040881c, .prod = 0x00000000, .disable = 0x0001fffe},
	{.addr = 0x00408a84, .prod = 0x00000000, .disable = 0x0001fffe},
	{.addr = 0x00408a8c, .prod = 0x00000000, .disable = 0x0001fffe},
	{.addr = 0x00408a94, .prod = 0x00000000, .disable = 0x0001fffe},
	{.addr = 0x00408a9c, .prod = 0x00000000, .disable = 0x0001fffe},
	{.addr = 0x00408aa4, .prod = 0x00000000, .disable = 0x0001fffe},
	{.addr = 0x00408aac, .prod = 0x00000000, .disable = 0x0001fffe},
	{.addr = 0x004089ac, .prod = 0x00000000, .disable = 0x0001fffe},
	{.addr = 0x00408a24, .prod = 0x00000000, .disable = 0x000001ff},
};

/* slcg ltc */
static const
struct gating_desc gm20b_slcg_ltc[] = {
	{.addr = 0x0017e050, .prod = 0x00000000, .disable = 0xfffffffe},
	{.addr = 0x0017e35c, .prod = 0x00000000, .disable = 0xfffffffe},
};

/* slcg perf */
static const
struct gating_desc gm20b_slcg_perf[] = {
	{.addr = 0x001be018, .prod = 0x000001ff, .disable = 0x00000000},
	{.addr = 0x001bc018, .prod = 0x000001ff, .disable = 0x00000000},
	{.addr = 0x001b8018, .prod = 0x000001ff, .disable = 0x00000000},
	{.addr = 0x001b4124, .prod = 0x00000001, .disable = 0x00000000},
};

/* slcg PriRing */
static const
struct gating_desc gm20b_slcg_priring[] = {
	{.addr = 0x001200a8, .prod = 0x00000000, .disable = 0x00000001},
};

/* slcg pmu */
static const
struct gating_desc gm20b_slcg_pmu[] = {
	{.addr = 0x0010a17c, .prod = 0x00020008, .disable = 0x0003fffe},
	{.addr = 0x0010aa74, .prod = 0x00000000, .disable = 0x00007ffe},
	{.addr = 0x0010ae74, .prod = 0x00000000, .disable = 0x0000000f},
};

/* slcg Xbar */
static const
struct gating_desc gm20b_slcg_xbar[] = {
	{.addr = 0x0013cbe4, .prod = 0x00000000, .disable = 0x1ffffffe},
	{.addr = 0x0013cc04, .prod = 0x00000000, .disable = 0x1ffffffe},
};

/* blcg bus */
static const
struct gating_desc gm20b_blcg_bus[] = {
	{.addr = 0x00001c00, .prod = 0x00000042, .disable = 0x00000000},
};

/* blcg fb */
static const
struct gating_desc gm20b_blcg_fb[] = {
	{.addr = 0x00100d10, .prod = 0x0000c242, .disable = 0x00000000},
	{.addr = 0x00100d30, .prod = 0x0000c242, .disable = 0x00000000},
	{.addr = 0x00100d3c, .prod = 0x00000242, .disable = 0x00000000},
	{.addr = 0x00100d48, .prod = 0x0000c242, .disable = 0x00000000},
	{.addr = 0x00100d1c, .prod = 0x00000042, .disable = 0x00000000},
	{.addr = 0x00100c98, .prod = 0x00000242, .disable = 0x00000000},
};

/* blcg fifo */
static const
struct gating_desc gm20b_blcg_fifo[] = {
	{.addr = 0x000026a4, .prod = 0x0000c242, .disable = 0x00000000},
};

/* blcg gr */
static const
struct gating_desc gm20b_blcg_gr[] = {
	{.addr = 0x004041f0, .prod = 0x00004046, .disable = 0x00000000},
	{.addr = 0x00409890, .prod = 0x0000007f, .disable = 0x00000000},
	{.addr = 0x004098b0, .prod = 0x0000007f, .disable = 0x00000000},
	{.addr = 0x004078c0, .prod = 0x00000042, .disable = 0x00000000},
	{.addr = 0x00406000, .prod = 0x00004044, .disable = 0x00000000},
	{.addr = 0x00405860, .prod = 0x00004042, .disable = 0x00000000},
	{.addr = 0x0040590c, .prod = 0x00004044, .disable = 0x00000000},
	{.addr = 0x00408040, .prod = 0x00004044, .disable = 0x00000000},
	{.addr = 0x00407000, .prod = 0x00004041, .disable = 0x00000000},
	{.addr = 0x00405bf0, .prod = 0x00004044, .disable = 0x00000000},
	{.addr = 0x0041a890, .prod = 0x0000007f, .disable = 0x00000000},
	{.addr = 0x0041a8b0, .prod = 0x0000007f, .disable = 0x00000000},
	{.addr = 0x00418500, .prod = 0x00004044, .disable = 0x00000000},
	{.addr = 0x00418608, .prod = 0x00004042, .disable = 0x00000000},
	{.addr = 0x00418688, .prod = 0x00004042, .disable = 0x00000000},
	{.addr = 0x00418718, .prod = 0x00000042, .disable = 0x00000000},
	{.addr = 0x00418828, .prod = 0x00000044, .disable = 0x00000000},
	{.addr = 0x00418bbc, .prod = 0x00004042, .disable = 0x00000000},
	{.addr = 0x00418970, .prod = 0x00004042, .disable = 0x00000000},
	{.addr = 0x00418c70, .prod = 0x00004044, .disable = 0x00000000},
	{.addr = 0x00418cf0, .prod = 0x00004044, .disable = 0x00000000},
	{.addr = 0x00418d70, .prod = 0x00004044, .disable = 0x00000000},
	{.addr = 0x00418f0c, .prod = 0x00004044, .disable = 0x00000000},
	{.addr = 0x00418e0c, .prod = 0x00004044, .disable = 0x00000000},
	{.addr = 0x00419020, .prod = 0x00004042, .disable = 0x00000000},
	{.addr = 0x00419038, .prod = 0x00000042, .disable = 0x00000000},
	{.addr = 0x00418898, .prod = 0x00000042, .disable = 0x00000000},
	{.addr = 0x00419a40, .prod = 0x00000042, .disable = 0x00000000},
	{.addr = 0x00419a48, .prod = 0x00004042, .disable = 0x00000000},
	{.addr = 0x00419a50, .prod = 0x00004042, .disable = 0x00000000},
	{.addr = 0x00419a58, .prod = 0x00004042, .disable = 0x00000000},
	{.addr = 0x00419a60, .prod = 0x00004042, .disable = 0x00000000},
	{.addr = 0x00419a68, .prod = 0x00004042, .disable = 0x00000000},
	{.addr = 0x00419a70, .prod = 0x00004042, .disable = 0x00000000},
	{.addr = 0x00419a78, .prod = 0x00004042, .disable = 0x00000000},
	{.addr = 0x00419a80, .prod = 0x00004042, .disable = 0x00000000},
	{.addr = 0x00419868, .prod = 0x00000042, .disable = 0x00000000},
	{.addr = 0x00419cd4, .prod = 0x00000002, .disable = 0x00000000},
	{.addr = 0x00419cdc, .prod = 0x00000002, .disable = 0x00000000},
	{.addr = 0x00419c70, .prod = 0x00004044, .disable = 0x00000000},
	{.addr = 0x00419fd0, .prod = 0x00004044, .disable = 0x00000000},
	{.addr = 0x00419fd8, .prod = 0x00004046, .disable = 0x00000000},
	{.addr = 0x00419fe0, .prod = 0x00004044, .disable = 0x00000000},
	{.addr = 0x00419fe8, .prod = 0x00000042, .disable = 0x00000000},
	{.addr = 0x00419ff0, .prod = 0x00004045, .disable = 0x00000000},
	{.addr = 0x00419ff8, .prod = 0x00000002, .disable = 0x00000000},
	{.addr = 0x00419f90, .prod = 0x00000002, .disable = 0x00000000},
	{.addr = 0x0041be28, .prod = 0x00000042, .disable = 0x00000000},
	{.addr = 0x0041bfe8, .prod = 0x00004044, .disable = 0x00000000},
	{.addr = 0x0041bed0, .prod = 0x00004044, .disable = 0x00000000},
	{.addr = 0x00408810, .prod = 0x00004042, .disable = 0x00000000},
	{.addr = 0x00408818, .prod = 0x00004042, .disable = 0x00000000},
	{.addr = 0x00408a80, .prod = 0x00004042, .disable = 0x00000000},
	{.addr = 0x00408a88, .prod = 0x00004042, .disable = 0x00000000},
	{.addr = 0x00408a90, .prod = 0x00004042, .disable = 0x00000000},
	{.addr = 0x00408a98, .prod = 0x00004042, .disable = 0x00000000},
	{.addr = 0x00408aa0, .prod = 0x00004042, .disable = 0x00000000},
	{.addr = 0x00408aa8, .prod = 0x00004042, .disable = 0x00000000},
	{.addr = 0x004089a8, .prod = 0x00004042, .disable = 0x00000000},
	{.addr = 0x004089b0, .prod = 0x00000042, .disable = 0x00000000},
	{.addr = 0x004089b8, .prod = 0x00004042, .disable = 0x00000000},
};

/* blcg ltc */
static const
struct gating_desc gm20b_blcg_ltc[] = {
	{.addr = 0x0017e030, .prod = 0x00000044, .disable = 0x00000000},
	{.addr = 0x0017e040, .prod = 0x00000044, .disable = 0x00000000},
	{.addr = 0x0017e3e0, .prod = 0x00000044, .disable = 0x00000000},
	{.addr = 0x0017e3c8, .prod = 0x00000044, .disable = 0x00000000},
};

/* blcg pmu */
static const
struct gating_desc gm20b_blcg_pmu[] = {
	{.addr = 0x0010aa70, .prod = 0x00000045, .disable = 0x00000000},
};

static int
gm20b_pmu_disable_clk_gating(struct nvkm_pmu *pmu)
{
	struct gk20a_pmu_priv *priv = to_gk20a_priv(pmu);
	int ret;

	mutex_lock(&priv->clk_gating_mutex);

	if (priv->clk_gating_disable_depth++ > 0) {
		ret = 0;
		goto do_nothing;
	}

	gk20a_init_elcg_mode(pmu, ELCG_RUN, ENGINE_CE2_GK20A);
	gk20a_init_elcg_mode(pmu, ELCG_RUN, ENGINE_CE2_GK20A);

	gk20a_disable_load_gating_prod(pmu, gm20b_slcg_bus,
					ARRAY_SIZE(gm20b_slcg_bus));
	gk20a_disable_load_gating_prod(pmu, gm20b_slcg_ce2,
					ARRAY_SIZE(gm20b_slcg_ce2));
	gk20a_disable_load_gating_prod(pmu, gm20b_slcg_chiplet,
					ARRAY_SIZE(gm20b_slcg_chiplet));
	gk20a_disable_load_gating_prod(pmu, gm20b_slcg_fb,
					ARRAY_SIZE(gm20b_slcg_fb));
	gk20a_disable_load_gating_prod(pmu, gm20b_slcg_fifo,
					ARRAY_SIZE(gm20b_slcg_fifo));
	gk20a_disable_load_gating_prod(pmu, gm20b_slcg_gr,
					ARRAY_SIZE(gm20b_slcg_gr));
	gk20a_disable_load_gating_prod(pmu, gm20b_slcg_ltc,
					ARRAY_SIZE(gm20b_slcg_ltc));
	gk20a_disable_load_gating_prod(pmu, gm20b_slcg_perf,
					ARRAY_SIZE(gm20b_slcg_perf));
	gk20a_disable_load_gating_prod(pmu, gm20b_slcg_priring,
					ARRAY_SIZE(gm20b_slcg_priring));
	gk20a_disable_load_gating_prod(pmu, gm20b_slcg_pmu,
					ARRAY_SIZE(gm20b_slcg_pmu));
	gk20a_disable_load_gating_prod(pmu, gm20b_slcg_xbar,
					ARRAY_SIZE(gm20b_slcg_xbar));

	gk20a_disable_load_gating_prod(pmu, gm20b_blcg_bus,
					ARRAY_SIZE(gm20b_blcg_bus));
	gk20a_disable_load_gating_prod(pmu, gm20b_blcg_fb,
					ARRAY_SIZE(gm20b_blcg_fb));
	gk20a_disable_load_gating_prod(pmu, gm20b_blcg_fifo,
					ARRAY_SIZE(gm20b_blcg_fifo));
	gk20a_disable_load_gating_prod(pmu, gm20b_blcg_gr,
					ARRAY_SIZE(gm20b_blcg_gr));
	gk20a_disable_load_gating_prod(pmu, gm20b_blcg_ltc,
					ARRAY_SIZE(gm20b_blcg_ltc));
	gk20a_disable_load_gating_prod(pmu, gm20b_blcg_pmu,
					ARRAY_SIZE(gm20b_blcg_pmu));

do_nothing:
	mutex_unlock(&priv->clk_gating_mutex);

	return 0;
}

static int
gm20b_pmu_enable_clk_gating(struct nvkm_pmu *pmu)
{
	struct gk20a_pmu_priv *priv = to_gk20a_priv(pmu);
	int ret;

	mutex_lock(&priv->clk_gating_mutex);

	if (--priv->clk_gating_disable_depth > 0) {
		ret = 0;
		goto do_nothing;
	}

	if (pmu->elcg_enabled) {
		gk20a_init_elcg_mode(pmu, ELCG_AUTO, ENGINE_CE2_GK20A);
		gk20a_init_elcg_mode(pmu, ELCG_AUTO, ENGINE_GR_GK20A);
	}

	if (pmu->slcg_enabled) {
		gk20a_enable_load_gating_prod(pmu, gm20b_slcg_bus,
						ARRAY_SIZE(gm20b_slcg_bus));
		gk20a_enable_load_gating_prod(pmu, gm20b_slcg_ce2,
						ARRAY_SIZE(gm20b_slcg_ce2));
		gk20a_enable_load_gating_prod(pmu, gm20b_slcg_chiplet,
						ARRAY_SIZE(gm20b_slcg_chiplet));
		gk20a_enable_load_gating_prod(pmu, gm20b_slcg_fb,
						ARRAY_SIZE(gm20b_slcg_fb));
		gk20a_enable_load_gating_prod(pmu, gm20b_slcg_fifo,
						ARRAY_SIZE(gm20b_slcg_fifo));
		gk20a_enable_load_gating_prod(pmu, gm20b_slcg_gr,
						ARRAY_SIZE(gm20b_slcg_gr));
		gk20a_enable_load_gating_prod(pmu, gm20b_slcg_ltc,
						ARRAY_SIZE(gm20b_slcg_ltc));
		gk20a_enable_load_gating_prod(pmu, gm20b_slcg_perf,
						ARRAY_SIZE(gm20b_slcg_perf));
		gk20a_enable_load_gating_prod(pmu, gm20b_slcg_priring,
						ARRAY_SIZE(gm20b_slcg_priring));
		gk20a_enable_load_gating_prod(pmu, gm20b_slcg_pmu,
						ARRAY_SIZE(gm20b_slcg_pmu));
		gk20a_enable_load_gating_prod(pmu, gm20b_slcg_xbar,
					ARRAY_SIZE(gm20b_slcg_xbar));
	}

	if (pmu->blcg_enabled) {
		gk20a_enable_load_gating_prod(pmu, gm20b_blcg_bus,
						ARRAY_SIZE(gm20b_blcg_bus));
		gk20a_enable_load_gating_prod(pmu, gm20b_blcg_fb,
						ARRAY_SIZE(gm20b_blcg_fb));
		gk20a_enable_load_gating_prod(pmu, gm20b_blcg_fifo,
						ARRAY_SIZE(gm20b_blcg_fifo));
		gk20a_enable_load_gating_prod(pmu, gm20b_blcg_gr,
						ARRAY_SIZE(gm20b_blcg_gr));
		gk20a_enable_load_gating_prod(pmu, gm20b_blcg_ltc,
						ARRAY_SIZE(gm20b_blcg_ltc));
		gk20a_enable_load_gating_prod(pmu, gm20b_blcg_pmu,
						ARRAY_SIZE(gm20b_blcg_pmu));
	}

do_nothing:
	mutex_unlock(&priv->clk_gating_mutex);

	return 0;
}

int
pmu_reset(struct nvkm_pmu *ppmu, struct nvkm_mc *pmc)
{
	int err;
	struct gk20a_pmu_priv *pmu = to_gk20a_priv(ppmu);

	err = gk20a_pmu_idle(pmu);
	if (err)
		return err;

	/* TBD: release pmu hw mutex */

	err = gk20a_pmu_enable(pmu, pmc, false);
	if (err)
		return err;

	err = gk20a_pmu_enable(pmu, pmc, true);
	if (err)
		return err;

	return 0;
}

static int
gm20b_pmu_init_vm(struct nvkm_pmu *ppmu)
{
	int ret = 0;
	struct gk20a_pmu_priv *pmu = to_gk20a_priv(ppmu);
	struct nvkm_pmu_priv_vm *pmuvm = &pmu->pmuvm;
	struct nvkm_device *device = nv_device(&ppmu->base);
	struct nvkm_vm *vm;
	struct nvkm_mmu *mmu = nvkm_mmu(pmu);
	const u64 pmu_area_len = 600*1024;

	/* allocate inst blk */
	ret = nvkm_gpuobj_new(nv_object(ppmu), NULL, 0x1000, 0, 0, &pmuvm->mem);
	if (ret)
		return ret;

	/* allocate pgd and initialize inst blk */
	ret = mmu->create_pgd(mmu, nv_object(ppmu), pmuvm->mem,
				pmu_area_len, &pmuvm->pgd);
	if (ret)
		goto err_pgd;

	/* allocate virtual memory range */
	ret = nvkm_vm_new(device, 0, pmu_area_len, 0, &vm);
	if (ret)
		goto err_vm;

	atomic_inc(&vm->engref[NVDEV_SUBDEV_PMU]);

	/* update VM with pgd */
	ret = nvkm_vm_ref(vm, &pmuvm->vm, pmuvm->pgd);
	if (ret)
		goto err_ref;

	ppmu->pmu_vm = pmuvm;
	return 0;

err_ref:
	nvkm_vm_ref(NULL, &vm, NULL);
err_vm:
	nvkm_gpuobj_destroy(pmuvm->pgd);
	pmuvm->pgd = NULL;
err_pgd:
	nvkm_gpuobj_destroy(pmuvm->mem);
	pmuvm->mem = NULL;
	return ret;
}

typedef int (*get_ucode_details)(struct nvkm_pmu *ppmu,
						  struct flcn_ucode_img *udata);

static void
gm20b_copy_ctxsw_ucode_segments(
	u8 *buf, struct gm20b_ctxsw_ucode_segments *segments,
	u32 *bootimage, u32 *code, u32 *data)
{
	memcpy(buf + segments->boot.offset, bootimage, segments->boot.size);
	memcpy(buf + segments->code.offset, code,      segments->code.size);
	memcpy(buf + segments->data.offset, data,      segments->data.size);
}

static void
gm20b_init_ctxsw_ucode_segment(
	struct gm20b_ctxsw_ucode_segment *p_seg, u32 *offset, u32 size)
{
	p_seg->offset = *offset;
	p_seg->size = size;
	*offset = ALIGN(*offset + size, BLK_SIZE);
}

static u8
gm20b_lsfm_falcon_disabled(struct nvkm_pmu *ppmu, struct ls_flcn_mgr *plsfm,
								  u32 falcon_id)
{
	return (plsfm->disable_mask >> falcon_id) & 0x1;
}

/* Free any ucode image structure resources*/
static void
gm20b_lsfm_free_ucode_img_res(struct flcn_ucode_img *p_img)
{
	if (p_img->lsf_desc != NULL) {
		kfree(p_img->lsf_desc);
		p_img->lsf_desc = NULL;
	}
}

/* Free any ucode image structure resources*/
static void
gm20b_lsfm_free_nonpmu_ucode_img_res(struct flcn_ucode_img *p_img)
{
	if (p_img->lsf_desc != NULL) {
		kfree(p_img->lsf_desc);
		p_img->lsf_desc = NULL;
	}
	if (p_img->desc != NULL) {
		kfree(p_img->desc);
		p_img->desc = NULL;
	}
	if (p_img->data != NULL) {
		kfree(p_img->data);
		p_img->data = NULL;
		p_img->data_size = 0;
	}
}

/*
 * Calculates PHY and VIRT addresses for various portions of the ucode image.
 * like: application code, application data, and bootloader code.
 * Return if ucode image is header based.
 * BL desc will be used by HS bin to boot corresponding LS(Low secure) falcon.
 */
static int
gm20b_flcn_populate_bl_dmem_desc(struct nvkm_pmu *ppmu,
	struct lsfm_managed_ucode_img *lsfm,
	union flcn_bl_generic_desc *p_bl_gen_desc, u32 *p_bl_gen_desc_size)
{

	struct flcn_ucode_img *p_img = &(lsfm->ucode_img);
	struct flcn_bl_dmem_desc *ldr_cfg =
		(struct flcn_bl_dmem_desc *)(&p_bl_gen_desc->bl_dmem_desc);
	u64 addr_base;
	struct pmu_ucode_desc *desc;
	u64 addr_code, addr_data;

	if (p_img->desc == NULL)
		return -EINVAL;
	desc = p_img->desc;

	addr_base = lsfm->lsb_header.ucode_off;
	addr_base += ioread32_native(mc + MC_SECURITY_CARVEOUT2_BOM_0);
	nv_debug(ppmu, "gen loader cfg %x u32 addrbase %x ID\n", (u32)addr_base,
		lsfm->wpr_header.falcon_id);
	addr_code = lower_32_bits((addr_base +
				desc->app_start_offset +
				desc->app_resident_code_offset) >> 8);
	addr_data = lower_32_bits((addr_base +
				desc->app_start_offset +
				desc->app_resident_data_offset) >> 8);

	nv_debug(ppmu, "gen cfg %x u32 addrcode %x & data %x load offst %xID\n",
		(u32)addr_code, (u32)addr_data, desc->bootloader_start_offset,
		lsfm->wpr_header.falcon_id);

	memset((void *) ldr_cfg, 0, sizeof(struct flcn_bl_dmem_desc));
	ldr_cfg->ctx_dma = GK20A_PMU_DMAIDX_UCODE;
	ldr_cfg->code_dma_base = addr_code;
	ldr_cfg->non_sec_code_size = desc->app_resident_code_size;
	ldr_cfg->data_dma_base = addr_data;
	ldr_cfg->data_size = desc->app_resident_data_size;
	ldr_cfg->code_entry_point = desc->app_imem_entry;
	*p_bl_gen_desc_size = sizeof(p_bl_gen_desc->bl_dmem_desc);
	return 0;
}

static u8
pmu_is_debug_mode_en(struct nvkm_pmu *pmu)
{
	u32 ctl_stat = nv_rd32(pmu, 0x0010ac08);

	return (ctl_stat >> 20) & 0x1;
}

/*
 * @brief Patch signatures into ucode image
 */
static int
acr_ucode_patch_sig(struct nvkm_pmu *pmu,
		unsigned int *p_img,
		unsigned int *p_prod_sig,
		unsigned int *p_dbg_sig,
		unsigned int *p_patch_loc,
		unsigned int *p_patch_ind)
{
	int i, *p_sig;

	if (!pmu_is_debug_mode_en(pmu)) {
		p_sig = p_prod_sig;
		nv_debug(pmu, "PRODUCTION MODE\n");
	} else {
		p_sig = p_dbg_sig;
		nv_debug(pmu, "DEBUG MODE\n");
	}

	/*Patching logic:*/
	for (i = 0; i < sizeof(*p_patch_loc)>>2; i++) {
		p_img[(p_patch_loc[i]>>2)] = p_sig[(p_patch_ind[i]<<2)];
		p_img[(p_patch_loc[i]>>2)+1] = p_sig[(p_patch_ind[i]<<2)+1];
		p_img[(p_patch_loc[i]>>2)+2] = p_sig[(p_patch_ind[i]<<2)+2];
		p_img[(p_patch_loc[i]>>2)+3] = p_sig[(p_patch_ind[i]<<2)+3];
	}
	return 0;
}

/*Once is LS mode, cpuctl_alias is only accessible*/
static void start_gm20b_pmu(struct nvkm_pmu *ppmu)
{
	struct nvkm_mc *pmc = nvkm_mc(ppmu);
	struct gk20a_pmu_priv *pmu = to_gk20a_priv(ppmu);

	mutex_lock(&pmu->isr_mutex);
	gk20a_pmu_enable_irq(pmu, pmc, true);
	pmu->isr_enabled = true;
	mutex_unlock(&pmu->isr_mutex);
	nv_wr32(ppmu, 0x0010a130, 0x2);
}

/*Parses UCODE header of falcon to fill respective LSB header*/
static int
gm20b_lsfm_parse_no_loader_ucode(u32 *p_ucodehdr,
	struct lsf_lsb_header *lsb_hdr)
{

	u32 code_size = 0;
	u32 data_size = 0;
	u32 i = 0;
	u32 total_apps = p_ucodehdr[FLCN_NL_UCODE_HDR_NUM_APPS_IND];

	/* Lets calculate code size*/
	code_size += p_ucodehdr[FLCN_NL_UCODE_HDR_OS_CODE_SIZE_IND];
	for (i = 0; i < total_apps; i++) {
		code_size += p_ucodehdr[FLCN_NL_UCODE_HDR_APP_CODE_SIZE_IND
			(total_apps, i)];
	}
	code_size += p_ucodehdr[FLCN_NL_UCODE_HDR_OS_OVL_SIZE_IND(total_apps)];

	/* Calculate data size*/
	data_size += p_ucodehdr[FLCN_NL_UCODE_HDR_OS_DATA_SIZE_IND];
	for (i = 0; i < total_apps; i++) {
		data_size += p_ucodehdr[FLCN_NL_UCODE_HDR_APP_DATA_SIZE_IND
			(total_apps, i)];
	}

	lsb_hdr->ucode_size = code_size;
	lsb_hdr->data_size = data_size;
	lsb_hdr->bl_code_size = p_ucodehdr[FLCN_NL_UCODE_HDR_OS_CODE_SIZE_IND];
	lsb_hdr->bl_imem_off = 0;
	lsb_hdr->bl_data_off = p_ucodehdr[FLCN_NL_UCODE_HDR_OS_DATA_OFF_IND];
	lsb_hdr->bl_data_size = p_ucodehdr[FLCN_NL_UCODE_HDR_OS_DATA_SIZE_IND];
	return 0;
}

static void
gm20b_free_acr_resources(struct nvkm_pmu *ppmu, struct ls_flcn_mgr *plsfm)
{
	u32 cnt = plsfm->managed_flcn_cnt;
	struct lsfm_managed_ucode_img *mg_ucode_img;

	while (cnt) {
		mg_ucode_img = plsfm->ucode_img_list;
		if (mg_ucode_img->ucode_img.lsf_desc->falcon_id ==
				LSF_FALCON_ID_PMU)
			gm20b_lsfm_free_ucode_img_res(&mg_ucode_img->ucode_img);
		else
			gm20b_lsfm_free_nonpmu_ucode_img_res(
				&mg_ucode_img->ucode_img);
		plsfm->ucode_img_list = mg_ucode_img->next;
		kfree(mg_ucode_img);
		cnt--;
	}
}

/*
 * Calculates PHY and VIRT addresses for various portions of the PMU ucode.
 * e.g. application code, application data, and bootloader code.
 * Return -EINVAL if ucode image is header based.
 * HS bin will use BL desc to boot PMU LS(Low secure) falcon.
 */
static int
gm20b_pmu_populate_loader_cfg(struct nvkm_pmu *ppmu,
	struct lsfm_managed_ucode_img *lsfm,
	union flcn_bl_generic_desc *p_bl_gen_desc, u32 *p_bl_gen_desc_size)
{
	struct gk20a_pmu_priv *priv = to_gk20a_priv(ppmu);
	struct gm20b_acr *acr = &priv->acr;
	struct flcn_ucode_img *p_img = &(lsfm->ucode_img);
	struct loader_config *ldr_cfg =
		(struct loader_config *)(&p_bl_gen_desc->loader_cfg);
	u64 addr_base;
	struct pmu_ucode_desc *desc;
	u64 addr_code, addr_data;
	u32 addr_args;
	struct pmu_cmdline_args_v1 cmdline_args;

	if (p_img->desc == NULL)
		return -EINVAL;
	desc = p_img->desc;
	addr_base = lsfm->lsb_header.ucode_off;
	addr_base += ioread32_native(mc + MC_SECURITY_CARVEOUT2_BOM_0);
	nv_debug(ppmu, "pmu loader cfg u32 addrbase %x\n", (u32)addr_base);
	addr_code = lower_32_bits((addr_base +
				desc->app_start_offset +
				desc->app_resident_code_offset) >> 8);
	nv_debug(ppmu, "app start %d app res code off %d\n",
		desc->app_start_offset, desc->app_resident_code_offset);
	addr_data = lower_32_bits((addr_base +
				desc->app_start_offset +
				desc->app_resident_data_offset) >> 8);
	nv_debug(ppmu, "app res data offset%d\n",
		desc->app_resident_data_offset);
	nv_debug(ppmu, "bl start off %d\n", desc->bootloader_start_offset);

	addr_args = ((nv_rd32(ppmu, 0x0010a108) >> 9) & 0x1ff)
						<< GK20A_PMU_DMEM_BLKSIZE2;
	addr_args -= sizeof(cmdline_args);

	nv_debug(ppmu, "addr_args %x\n", addr_args);

	ldr_cfg->dma_idx = GK20A_PMU_DMAIDX_UCODE;
	ldr_cfg->code_dma_base = addr_code;
	ldr_cfg->code_size_total = desc->app_size;
	ldr_cfg->code_size_to_load = desc->app_resident_code_size;
	ldr_cfg->code_entry_point = desc->app_imem_entry;
	ldr_cfg->data_dma_base = addr_data;
	ldr_cfg->data_size = desc->app_resident_data_size;
	ldr_cfg->overlay_dma_base = addr_code;

	ldr_cfg->argc = 1;
	ldr_cfg->argv = addr_args;

	*p_bl_gen_desc_size = sizeof(p_bl_gen_desc->loader_cfg);
	acr->pmu_args = addr_args;

	return 0;
}

static void
gm20b_init_ctxsw_ucode_segments(
	struct gm20b_ctxsw_ucode_segments *segments, u32 *offset,
	struct gm20b_ctxsw_bootloader_desc *bootdesc,
	u32 code_size, u32 data_size)
{
	u32 boot_size = ALIGN(bootdesc->size, sizeof(u32));

	segments->boot_entry = bootdesc->entry_point;
	segments->boot_imem_offset = bootdesc->imem_offset;
	gm20b_init_ctxsw_ucode_segment(&segments->boot, offset, boot_size);
	gm20b_init_ctxsw_ucode_segment(&segments->code, offset, code_size);
	gm20b_init_ctxsw_ucode_segment(&segments->data, offset, data_size);
}

/*Get PMU ucode details & fill flcn_ucode_img struct with these details*/
static int
gm20b_pmu_ucode_details(struct nvkm_pmu *ppmu, struct flcn_ucode_img *p_img)
{
	const struct firmware *pmu_fw, *pmu_desc, *pmu_sig;
	struct gk20a_pmu_priv *pmu = to_gk20a_priv(ppmu);
	struct gm20b_acr *acr = &pmu->acr;
	struct lsf_ucode_desc *lsf_desc;
	int err;

	nv_debug(ppmu, "requesting PMU ucode in GM20B\n");
	err = gk20a_load_firmware(ppmu, &pmu_fw, GM20B_PMU_UCODE_IMAGE);
	if (err) {
		nv_error(ppmu, "failed to load pmu ucode!!\n");
		return -ENOENT;
	}
	acr->pmu_fw = pmu_fw;
	nv_debug(ppmu, "Loaded PMU ucode in for blob preparation\n");

	nv_debug(ppmu, "requesting PMU ucode desc in GM20B\n");
	err = gk20a_load_firmware(ppmu, &pmu_desc, GM20B_PMU_UCODE_DESC);
	if (err) {
		nv_error(ppmu, "failed to load pmu ucode desc!!\n");
		err = -ENOENT;
		goto release_img_fw;
	}
	nv_debug(ppmu, "requesting PMU ucode signature in GM20B\n");
	err = gk20a_load_firmware(ppmu, &pmu_sig, GM20B_PMU_UCODE_SIG);
	if (err) {
		nv_error(ppmu, "failed to load pmu sig!!\n");
		err = -ENOENT;
		goto release_desc;
	}

	pmu->desc = (struct pmu_ucode_desc *)pmu_desc->data;
	pmu->ucode_image = (u32 *)pmu_fw->data;
	acr->pmu_desc = pmu_desc;
	lsf_desc = kzalloc(sizeof(struct lsf_ucode_desc), GFP_KERNEL);
	if (!lsf_desc) {
		err = -ENOMEM;
		nv_error(ppmu, "lsf_desc alloc failed\n");
		goto release_sig;
	}

	memcpy(lsf_desc, (void *)pmu_sig->data, sizeof(struct lsf_ucode_desc));
	lsf_desc->falcon_id = LSF_FALCON_ID_PMU;

	p_img->desc = pmu->desc;
	p_img->data = pmu->ucode_image;
	p_img->data_size = pmu->desc->image_size;
	nv_debug(ppmu, "pmu ucode img loc %p & size %d\n", p_img->data,
							      p_img->data_size);
	p_img->fw_ver = NULL;
	p_img->header = NULL;
	p_img->lsf_desc = (struct lsf_ucode_desc *)lsf_desc;
	gk20a_release_firmware(ppmu, pmu_sig);

	return 0;
release_sig:
	gk20a_release_firmware(ppmu, pmu_sig);
release_desc:
	gk20a_release_firmware(ppmu, pmu_desc);
release_img_fw:
	gk20a_release_firmware(ppmu, pmu_fw);
	return err;
}

/*Populate static LSB header information using the provided ucode image*/
static void
gm20b_lsfm_fill_static_lsb_hdr(struct nvkm_pmu *ppmu,
			   u32 falcon_id, struct lsfm_managed_ucode_img *pnode)
{
	struct gk20a_pmu_priv *pmu = to_gk20a_priv(ppmu);
	u32 full_app_size = 0;
	u32 data = 0;

	if (pnode->ucode_img.lsf_desc)
		memcpy(&pnode->lsb_header.signature, pnode->ucode_img.lsf_desc,
			sizeof(struct lsf_ucode_desc));
	pnode->lsb_header.ucode_size = pnode->ucode_img.data_size;

	/* The remainder of the LSB depends on the loader usage */
	if (pnode->ucode_img.header) {
		/* Does not use a loader */
		pnode->lsb_header.data_size = 0;
		pnode->lsb_header.bl_code_size = 0;
		pnode->lsb_header.bl_data_off = 0;
		pnode->lsb_header.bl_data_size = 0;

		gm20b_lsfm_parse_no_loader_ucode(pnode->ucode_img.header,
			&(pnode->lsb_header));

		/* Load the first 256 bytes of IMEM. */
		/* Set LOAD_CODE_AT_0 and DMACTL_REQ_CTX.
		True for all method based falcons */
		data = NV_FLCN_ACR_LSF_FLAG_LOAD_CODE_AT_0_TRUE |
			NV_FLCN_ACR_LSF_FLAG_DMACTL_REQ_CTX_TRUE;
		pnode->lsb_header.flags = data;
	} else {
		/* Uses a loader. that is has a desc */
		pnode->lsb_header.data_size = 0;

		/* The loader code size is already aligned (padded) such that
		the code following it is aligned, but the size in the image
		desc is not, bloat it up to be on a 256 byte alignment. */
		pnode->lsb_header.bl_code_size = ALIGN(
			pnode->ucode_img.desc->bootloader_size,
			LSF_BL_CODE_SIZE_ALIGNMENT);
		full_app_size = ALIGN(pnode->ucode_img.desc->app_size,
			LSF_BL_CODE_SIZE_ALIGNMENT) +
			pnode->lsb_header.bl_code_size;
		pnode->lsb_header.ucode_size = ALIGN(
			pnode->ucode_img.desc->app_resident_data_offset,
			LSF_BL_CODE_SIZE_ALIGNMENT) +
			pnode->lsb_header.bl_code_size;
		pnode->lsb_header.data_size = full_app_size -
			pnode->lsb_header.ucode_size;
		/* Though the BL is located at 0th offset of the image, the VA
		is different to make sure that it doesnt collide the actual OS
		VA range */
		pnode->lsb_header.bl_imem_off =
			pnode->ucode_img.desc->bootloader_imem_offset;
		pnode->lsb_header.app_code_off =
			pnode->ucode_img.desc->app_start_offset +
			pnode->ucode_img.desc->app_resident_code_offset;
		pnode->lsb_header.app_code_size =
			pnode->ucode_img.desc->app_resident_code_size;
		pnode->lsb_header.app_data_off =
			pnode->ucode_img.desc->app_start_offset +
			pnode->ucode_img.desc->app_resident_data_offset;
		pnode->lsb_header.app_data_size =
			pnode->ucode_img.desc->app_resident_data_size;

		pnode->lsb_header.flags = 0;

		if (falcon_id == pmu->falcon_id) {
			data = NV_FLCN_ACR_LSF_FLAG_DMACTL_REQ_CTX_TRUE;
			pnode->lsb_header.flags = data;
		}
	}
}

/*Adds a ucode image to the list of managed ucode images managed*/
static int
gm20b_lsfm_add_ucode_img(struct nvkm_pmu *ppmu, struct ls_flcn_mgr *plsfm,
			     struct flcn_ucode_img *ucode_image, u32 falcon_id)
{

	struct lsfm_managed_ucode_img *pnode;

	pnode = kzalloc(sizeof(struct lsfm_managed_ucode_img), GFP_KERNEL);
	if (pnode == NULL)
		return -ENOMEM;
	nv_debug(ppmu, "created pnode\n");
	/*Keep a copy of the ucode image info locally*/
	memcpy(&pnode->ucode_img, ucode_image, sizeof(struct flcn_ucode_img));

	/*Fill in static WPR header info*/
	pnode->wpr_header.falcon_id = falcon_id;
	pnode->wpr_header.bootstrap_owner = LSF_BOOTSTRAP_OWNER_DEFAULT;
	pnode->wpr_header.status = LSF_IMAGE_STATUS_COPY;

	/*Fill in static LSB header info elsewhere*/
	gm20b_lsfm_fill_static_lsb_hdr(ppmu, falcon_id, pnode);
	nv_debug(ppmu, "filled lsb header\n");
	pnode->next = plsfm->ucode_img_list;
	plsfm->ucode_img_list = pnode;

	return 0;
}

/*Get FECS ucode details & fill flcn_ucode_img struct with these details*/
static int
gm20b_fecs_ucode_details(struct nvkm_pmu *ppmu, struct flcn_ucode_img *p_img)
{
	struct lsf_ucode_desc *lsf_desc;
	const struct firmware *fecs_sig, *fecs_blfw, *fecs_cfw, *fecs_dfw;
	int err;
	u32 ucode_size;
	u8 *buf;
	struct gk20a_pmu_priv *pmu = to_gk20a_priv(ppmu);
	struct gm20b_ctxsw_bootloader_desc *fecs_boot_desc;
	struct gm20b_ctxsw_ucode_info *ucode_info = &pmu->ucode_info;
	u32 *fecs_boot_image;

	err = gk20a_load_firmware(ppmu, &fecs_blfw, GK20A_FECS_UCODE_IMAGE);
	if (err) {
		nv_error(ppmu, "failed to load fecs ucode image\n");
		return -ENOENT;
	}

	err = gk20a_load_firmware(ppmu, &fecs_cfw, "nv12b_fuc409c");
	if (err) {
		nv_error(ppmu, "failed to load fecs ucode image\n");
		goto rel_fecs_blfw;
	}

	err = gk20a_load_firmware(ppmu, &fecs_dfw, "nv12b_fuc409d");
	if (err) {
		nv_error(ppmu, "failed to load fecs data image\n");
		goto rel_fecs_cfw;
	}

	err = gk20a_load_firmware(ppmu, &fecs_sig, GM20B_FECS_UCODE_SIG);
	if (err) {
		nv_error(ppmu, "failed to load fecs ucode sig\n");
		goto rel_fecs_dfw;
	}

	lsf_desc = kzalloc(sizeof(struct lsf_ucode_desc), GFP_KERNEL);
	if (!lsf_desc) {
		err = -ENOMEM;
		goto rel_fecs_sig;
	}

	nv_debug(ppmu, "fecs related fws loaded\n");
	memcpy(lsf_desc, (void *)fecs_sig->data, sizeof(struct lsf_ucode_desc));
	lsf_desc->falcon_id = LSF_FALCON_ID_FECS;
	fecs_boot_desc = (void *)fecs_blfw->data;
	fecs_boot_image = (void *)(fecs_blfw->data +
				sizeof(struct gm20b_ctxsw_bootloader_desc));
	ucode_size = 0;
	nv_debug(ppmu, "fecs related size calculation\n");
	gm20b_init_ctxsw_ucode_segments(&ucode_info->fecs, &ucode_size,
				fecs_boot_desc, fecs_cfw->size, fecs_dfw->size);

	buf = kzalloc(ucode_size, GFP_KERNEL);
	if (buf == NULL) {
		err = -ENOMEM;
		goto free_lsf_desc;
	}

	p_img->desc = kzalloc(sizeof(struct pmu_ucode_desc), GFP_KERNEL);
	if (p_img->desc == NULL) {
		err = -ENOMEM;
		goto free_ucode_buf;
	}

	gm20b_copy_ctxsw_ucode_segments(buf, &ucode_info->fecs,
		fecs_boot_image, (u32 *)fecs_cfw->data, (u32 *)fecs_dfw->data);

	nv_debug(ppmu, "fecs related p_img allocated\n");
	p_img->desc->bootloader_start_offset =
		pmu->ucode_info.fecs.boot.offset;
	p_img->desc->bootloader_size =
		ALIGN(pmu->ucode_info.fecs.boot.size, 256);
	p_img->desc->bootloader_imem_offset =
		pmu->ucode_info.fecs.boot_imem_offset;
	p_img->desc->bootloader_entry_point =
		pmu->ucode_info.fecs.boot_entry;

	p_img->desc->image_size =
		ALIGN(pmu->ucode_info.fecs.boot.size, 256) +
		ALIGN(pmu->ucode_info.fecs.code.size, 256) +
		ALIGN(pmu->ucode_info.fecs.data.size, 256);
	p_img->desc->app_size = ALIGN(pmu->ucode_info.fecs.code.size, 256) +
		ALIGN(pmu->ucode_info.fecs.data.size, 256);
	p_img->desc->app_start_offset = pmu->ucode_info.fecs.code.offset;
	p_img->desc->app_imem_offset = 0;
	p_img->desc->app_imem_entry = 0;
	p_img->desc->app_dmem_offset = 0;
	p_img->desc->app_resident_code_offset = 0;
	p_img->desc->app_resident_code_size =
		pmu->ucode_info.fecs.code.size;
	p_img->desc->app_resident_data_offset =
		pmu->ucode_info.fecs.data.offset -
		pmu->ucode_info.fecs.code.offset;
	p_img->desc->app_resident_data_size =
		pmu->ucode_info.fecs.data.size;
	p_img->data = (u32 *)buf;
	p_img->data_size = p_img->desc->image_size;
	nv_debug(ppmu, "fecs ucode image location %p and size %d\n",
						 p_img->data, p_img->data_size);
	p_img->fw_ver = NULL;
	p_img->header = NULL;
	p_img->lsf_desc = (struct lsf_ucode_desc *)lsf_desc;
	nv_debug(ppmu, "fecs fw loaded\n");
	gk20a_release_firmware(ppmu, fecs_sig);
	gk20a_release_firmware(ppmu, fecs_blfw);
	gk20a_release_firmware(ppmu, fecs_cfw);
	gk20a_release_firmware(ppmu, fecs_dfw);

	return 0;
free_ucode_buf:
	kfree(buf);
free_lsf_desc:
	kfree(lsf_desc);
rel_fecs_sig:
	gk20a_release_firmware(ppmu, fecs_sig);
rel_fecs_dfw:
	gk20a_release_firmware(ppmu, fecs_dfw);
rel_fecs_cfw:
	gk20a_release_firmware(ppmu, fecs_cfw);
rel_fecs_blfw:
	gk20a_release_firmware(ppmu, fecs_blfw);

	return err;
}

/* Populate falcon boot loader generic desc.*/
static int
gm20b_lsfm_fill_flcn_bl_gen_desc(struct nvkm_pmu *ppmu,
		struct lsfm_managed_ucode_img *pnode)
{
	struct gk20a_pmu_priv *pmu = to_gk20a_priv(ppmu);

	if (pnode->wpr_header.falcon_id != pmu->falcon_id) {
		nv_debug(ppmu, "non pmu. write flcn bl gen desc\n");
		gm20b_flcn_populate_bl_dmem_desc(ppmu, pnode,
				&pnode->bl_gen_desc, &pnode->bl_gen_desc_size);
		return 0;
	}

	if (pmu->pmu_mode & PMU_LSFM_MANAGED) {
		nv_debug(ppmu, "pmu write flcn bl gen desc\n");
		if (pnode->wpr_header.falcon_id == pmu->falcon_id)
			return gm20b_pmu_populate_loader_cfg(ppmu, pnode,
				&pnode->bl_gen_desc, &pnode->bl_gen_desc_size);
	}

	/* Failed to find the falcon requested. */
	return -ENOENT;
}

/*writes memory allocated to nvgpu object*/
void
gpu_obj_memrd(struct nvkm_gpuobj *ucodeobj, int offset, void *read, int size)
{
	int temp = size;
	u32 *source32;
	u16 *source16;
	u8 *source8;
	u32 red;
	int four_bytes_cnt, two_bytes_cnt, one_bytes_cnt;

	four_bytes_cnt = temp / 4;
	temp = temp % 4;
	two_bytes_cnt = temp / 2;
	temp = temp % 2;
	one_bytes_cnt = temp;
	source32 = (u32 *)read;
	for (temp = 0; temp < four_bytes_cnt; temp++) {
		source32 = (u32 *)read + temp;
		red = nv_ro32(ucodeobj, offset);
		memcpy(source32, &red, sizeof(u32));
		offset += 4;
	}
	source16 = (u16 *)source32;
	for (temp = 0; temp < two_bytes_cnt; temp++) {
		source16 = (u16 *)source32 + temp;
		*source16 = nv_ro16(ucodeobj, offset);
		offset += 2;
	}
	source8 = (u8 *)source16;
	for (temp = 0; temp < one_bytes_cnt; temp++) {
		source8 = (u8 *)source16 + temp;
		*source8 = nv_ro08(ucodeobj, offset);
		offset += 1;
	}
}
/*
* Initialize ucodeblob/WPR contents i.e.
* Walk the managed falcons, copy WPR and LSB headers to ucodeblob created.
* flush any bl args to the storage area relative to the
* ucode image (appended on the end as a DMEM area).
*/
static int
gm20b_lsfm_init_wpr_contents(struct nvkm_pmu *ppmu, struct ls_flcn_mgr *plsfm,
						struct nvkm_gpuobj *ucodebufobj)
{

	int status = 0;

	if (!ucodebufobj) {
		nv_error(ppmu, "ucode blob gpuboj is NULL\n");
		status = -EINVAL;
	} else {
		struct lsfm_managed_ucode_img *pnode = plsfm->ucode_img_list;
		struct lsf_wpr_header wpr_hdr;
		struct lsf_lsb_header lsb_hdr;
		u32 i, offset;

		/* The WPR array is at the base of the ucode blob (WPR)*/
		pnode = plsfm->ucode_img_list;
		i = 0;

		nv_debug(ppmu, "iterate all managed falcons\n");
		while (pnode) {
			nv_debug(ppmu, "falconid :%d\n",
				pnode->wpr_header.falcon_id);
			nv_debug(ppmu, "lsb_offset :%x\n",
				pnode->wpr_header.lsb_offset);
			nv_debug(ppmu, "bootstrap_owner : %d\n",
				pnode->wpr_header.bootstrap_owner);
			gpu_obj_memwr(ucodebufobj,
				0 + (i * sizeof(struct lsf_wpr_header)),
				&pnode->wpr_header,
				sizeof(struct lsf_wpr_header));
			/*Copying of WPR header*/
			gpu_obj_memrd(ucodebufobj,
				0 + i * sizeof(struct lsf_wpr_header),
				&wpr_hdr, sizeof(struct lsf_wpr_header));
			nv_debug(ppmu, "wpr header as in memory and pnode\n");
			nv_debug(ppmu, "falconid :%d %d\n",
					pnode->wpr_header.falcon_id,
					wpr_hdr.falcon_id);
			nv_debug(ppmu, "lsb_offset :%x %x\n",
					pnode->wpr_header.lsb_offset,
					wpr_hdr.lsb_offset);
			nv_debug(ppmu, "bootstrap_owner :%d %d\n",
					pnode->wpr_header.bootstrap_owner,
					wpr_hdr.bootstrap_owner);
			nv_debug(ppmu, "lazy_bootstrap :%d %d\n",
					pnode->wpr_header.lazy_bootstrap,
					wpr_hdr.lazy_bootstrap);
			nv_debug(ppmu, "status :%d %d\n",
				pnode->wpr_header.status, wpr_hdr.status);

			offset = pnode->wpr_header.lsb_offset;
			nv_debug(ppmu, "writing lsb header @ offset = %d\n",
									offset);
			/*Copying of LSB header*/
			gpu_obj_memwr(ucodebufobj, offset,
					&pnode->lsb_header,
					sizeof(struct lsf_lsb_header));
			gpu_obj_memrd(ucodebufobj, offset,
					&lsb_hdr,
					sizeof(struct lsf_lsb_header));
			nv_debug(ppmu, "lsb header as in memory and pnode\n");
			nv_debug(ppmu, "ucode_off :%x %x\n",
					pnode->lsb_header.ucode_off,
					lsb_hdr.ucode_off);
			nv_debug(ppmu, "ucode_size :%x %x\n",
					pnode->lsb_header.ucode_size,
					lsb_hdr.ucode_size);
			nv_debug(ppmu, "data_size :%x %x\n",
					pnode->lsb_header.data_size,
					lsb_hdr.data_size);
			nv_debug(ppmu, "bl_code_size :%x %x\n",
					pnode->lsb_header.bl_code_size,
					lsb_hdr.bl_code_size);
			nv_debug(ppmu, "bl_imem_off :%x %x\n",
					pnode->lsb_header.bl_imem_off,
					lsb_hdr.bl_imem_off);
			nv_debug(ppmu, "bl_data_off :%x %x\n",
					pnode->lsb_header.bl_data_off,
					lsb_hdr.bl_data_off);
			nv_debug(ppmu, "bl_data_size :%x %x\n",
					pnode->lsb_header.bl_data_size,
					lsb_hdr.bl_data_size);
			nv_debug(ppmu, "flags :%x %x\n",
					pnode->lsb_header.flags, lsb_hdr.flags);

			if (!pnode->ucode_img.header) {
				offset = pnode->lsb_header.bl_data_off;
				/*Copying of generic bootloader*/
				gm20b_lsfm_fill_flcn_bl_gen_desc(ppmu, pnode);
				nv_debug(ppmu,
				   "writing bl header @ offset = %d\n", offset);
				gpu_obj_memwr(ucodebufobj, offset,
					&pnode->bl_gen_desc,
					pnode->bl_gen_desc_size);
			}
			offset = (pnode->lsb_header.ucode_off);
			nv_debug(ppmu, "writing ucode @ offset = %d\n", offset);
			/*Copying of ucode*/
			nv_debug(ppmu, "ucode start from loc %p & size = %d\n",
			     pnode->ucode_img.data, pnode->ucode_img.data_size);
			gpu_obj_memwr(ucodebufobj, offset,
						pnode->ucode_img.data,
						pnode->ucode_img.data_size);
			pnode = pnode->next;
			i++;
		}
		nv_wo32(ucodebufobj, 0 + i * sizeof(struct lsf_wpr_header),
							 LSF_FALCON_ID_INVALID);
	}
	return status;
}

static get_ucode_details pmu_acr_supp_ucode_list[] = {
	gm20b_pmu_ucode_details,
	gm20b_fecs_ucode_details,
};

/* Discover all managed falcon ucode images */
static int
gm20b_lsfm_discover_ucode_images(struct nvkm_pmu *ppmu,
						     struct ls_flcn_mgr *plsfm)
{
	struct gk20a_pmu_priv *pmu = to_gk20a_priv(ppmu);
	struct flcn_ucode_img ucode_img;
	u32 falcon_id;
	u32 i;
	int status;

	/* LSFM requires a secure PMU, discover it first.*/
	/* Obtain the PMU ucode image and add it to the list if required*/
	memset(&ucode_img, 0, sizeof(ucode_img));
	status = gm20b_pmu_ucode_details(ppmu, &ucode_img);
	if (status == 0) {
		if (ucode_img.lsf_desc != NULL) {
			/* The falon_id is formed by grabbing the static base
			 * falon_id from the image and adding the
			 * engine-designated falcon instance.*/
			pmu->pmu_mode |= PMU_SECURE_MODE;
			falcon_id = ucode_img.lsf_desc->falcon_id +
				ucode_img.flcn_inst;

			if (!gm20b_lsfm_falcon_disabled(ppmu, plsfm,
								falcon_id)) {
				pmu->falcon_id = falcon_id;
				if (gm20b_lsfm_add_ucode_img(ppmu,
					plsfm, &ucode_img, pmu->falcon_id) == 0)
					pmu->pmu_mode |= PMU_LSFM_MANAGED;

				plsfm->managed_flcn_cnt++;
			} else {
				nv_debug(ppmu, "id not managed %d\n",
					ucode_img.lsf_desc->falcon_id);
			}
		}

		/*Free any ucode image resources if not managing this falcon*/
		if (!(pmu->pmu_mode & PMU_LSFM_MANAGED)) {
			nv_debug(ppmu, "pmu is not LSFM managed\n");
			gm20b_lsfm_free_ucode_img_res(&ucode_img);
		}
	}
	/* Enumerate all constructed falcon objects,
	 as we need the ucode image info and total falcon count.*/

	/*0th index is always PMU which is already handled in earlier
	if condition*/
	for (i = 1; i < (MAX_SUPPORTED_LSFM); i++) {
		memset(&ucode_img, 0, sizeof(ucode_img));
		if (pmu_acr_supp_ucode_list[i](ppmu, &ucode_img) == 0) {
			if (ucode_img.lsf_desc != NULL) {
				/* We have engine sigs, ensure that this falcon
				is aware of the secure mode expectations
				(ACR status)*/

				nv_debug(ppmu, "iterated fecs ucode details\n");
				/* falon_id is formed by grabbing the static
				base falonId from the image and adding the
				engine-designated falcon instance. */
				falcon_id = ucode_img.lsf_desc->falcon_id +
					ucode_img.flcn_inst;

				if (!gm20b_lsfm_falcon_disabled(ppmu, plsfm,
					falcon_id)) {
					/* Do not manage non-FB ucode*/
					if (gm20b_lsfm_add_ucode_img(ppmu,
						plsfm, &ucode_img, falcon_id)
						== 0)
						plsfm->managed_flcn_cnt++;
				} else {
					nv_debug(ppmu, "not managed %d\n",
						ucode_img.lsf_desc->falcon_id);
					gm20b_lsfm_free_nonpmu_ucode_img_res(
						&ucode_img);
				}
			}
		} else {
			/* Consumed all available falcon objects */
			nv_debug(ppmu, "Done checking for ucodes %d\n", i);
			break;
		}
	}
	return 0;
}

/*Calculate size required for UCODE BLOB*/
static int
gm20b_lsf_gen_wpr_requirements(struct nvkm_pmu *ppmu, struct ls_flcn_mgr *plsfm)
{
	struct lsfm_managed_ucode_img *pnode = plsfm->ucode_img_list;
	u32 wpr_offset;

	/* Calculate WPR size required */

	/* Start with an array of WPR headers at the base of the WPR.
	 The expectation here is that the secure falcon will do a single DMA
	 read of this array and cache it internally so it's OK to pack these.
	 Also, we add 1 to the falcon count to indicate the end of the array.*/
	wpr_offset = sizeof(struct lsf_wpr_header) *
		(plsfm->managed_flcn_cnt+1);

	/* Walk the managed falcons, accounting for the LSB structs
	as well as the ucode images. */
	while (pnode) {
		/* Align, save off, and include an LSB header size */
		wpr_offset = ALIGN(wpr_offset,
			LSF_LSB_HEADER_ALIGNMENT);
		nv_debug(ppmu, "LSB header offset %d\n", wpr_offset);
		pnode->wpr_header.lsb_offset = wpr_offset;
		wpr_offset += sizeof(struct lsf_lsb_header);

		/* Align, save off, and include the original (static)
		ucode image size */
		wpr_offset = ALIGN(wpr_offset, LSF_UCODE_DATA_ALIGNMENT);
		pnode->lsb_header.ucode_off = wpr_offset;
		nv_debug(ppmu, "UCODE offset %d and size %d\n",
					wpr_offset, pnode->ucode_img.data_size);
		wpr_offset += pnode->ucode_img.data_size;

		/* For falcons that use a boot loader (BL), we append a loader
		desc structure on the end of the ucode image and consider this
		the boot loader data. The host will then copy the loader desc
		args to this space within the WPR region (before locking down)
		and the HS bin will then copy them to DMEM 0 for the loader. */
		if (!pnode->ucode_img.header) {
			/* Track the size for LSB details filled in later
			 Note that at this point we don't know what kind of i
			boot loader desc, so we just take the size of the
			generic one, which is the largest it will will ever be.
			*/
			/* Align (size bloat) and save off generic
			descriptor size*/
			pnode->lsb_header.bl_data_size = ALIGN(
				sizeof(pnode->bl_gen_desc),
				LSF_BL_DATA_SIZE_ALIGNMENT);

			/*Align, save off, and include the additional BL data*/
			wpr_offset = ALIGN(wpr_offset,
				LSF_BL_DATA_ALIGNMENT);
			pnode->lsb_header.bl_data_off = wpr_offset;
			nv_debug(ppmu, "BL desc offset %d\n", wpr_offset);
			wpr_offset += pnode->lsb_header.bl_data_size;
		} else {
			/* bl_data_off is already assigned in static
			information. But that is from start of the image */
			pnode->lsb_header.bl_data_off +=
				(wpr_offset - pnode->ucode_img.data_size);
		}

		/* Finally, update ucode surface size to include updates */
		pnode->full_ucode_size = wpr_offset -
			pnode->lsb_header.ucode_off;
		pnode = pnode->next;
	}

	plsfm->wpr_size = wpr_offset;
	return 0;
}

static int
gm20b_prepare_ucode_blob(struct nvkm_pmu *ppmu)
{
	u32 status;
	struct ls_flcn_mgr lsfm_l, *plsfm;
	int ret;
	struct gk20a_pmu_priv *priv = to_gk20a_priv(ppmu);
	struct gm20b_acr *acr = &priv->acr;

	if (priv->pmuvm.vm)
		return 0;

	plsfm = &lsfm_l;
	memset((void *)plsfm, 0, sizeof(struct ls_flcn_mgr));
	status = gm20b_lsfm_discover_ucode_images(ppmu, plsfm);
	if (status != 0)
		return status;

	nv_debug(ppmu, "All ucode images discovered\n");
	nv_debug(ppmu, " Managed Falcon cnt %d\n", plsfm->managed_flcn_cnt);
	if (plsfm->managed_flcn_cnt) {
		/* Generate WPR requirements*/
		status = gm20b_lsf_gen_wpr_requirements(ppmu, plsfm);
		if (status != 0)
			return status;

		ret = gm20b_pmu_init_vm(ppmu);
		if (ret) {
			nv_error(ppmu, "gm20b_init_vm failed\n");
			return ret;
		}

		ret = nvkm_gpuobj_new(nv_object(ppmu), NULL,
						plsfm->wpr_size, 0x1000, 0,
						&acr->ucode_blob.obj);
		if (ret) {
			nv_error(ppmu, "alloc for ucode blob failed\n");
			return ret;
		}

		nv_debug(ppmu, "managed LS falcon %d, WPR size %d bytes.\n",
				     plsfm->managed_flcn_cnt, plsfm->wpr_size);
		gm20b_lsfm_init_wpr_contents(ppmu, plsfm, acr->ucode_blob.obj);
		nv_debug(ppmu, "base reg carveout 2:%x\n",
		ioread32_native(mc + MC_SECURITY_CARVEOUT2_BOM_0));
		nv_debug(ppmu, "base reg carveout 3:%x\n",
		ioread32_native(mc + MC_SECURITY_CARVEOUT3_BOM_0));
		nv_debug(ppmu, "wpr init success\n");

		ret = nvkm_gpuobj_map_vm(nv_gpuobj(acr->ucode_blob.obj),
					priv->pmuvm.vm,
					NV_MEM_ACCESS_RW,
					&acr->ucode_blob.vma);
		if (ret) {
			nv_error(ppmu, "mapping of ucode blob failed\n");
			return ret;
		}
		nv_debug(ppmu, "acr->ucode_blob.vma.offset is: 0x%llx\n",
			acr->ucode_blob.vma.offset);
		acr->ucode_blob_start = acr->ucode_blob.obj->addr;
		acr->ucode_blob_size = plsfm->wpr_size;
	} else {
		nv_error(ppmu, "LSFM is managing no falcons.\n");
	}

	nv_debug(ppmu, "prepare ucode blob success\n");
	gm20b_free_acr_resources(ppmu, plsfm);
	gk20a_release_firmware(ppmu, priv->acr.pmu_fw);
	gk20a_release_firmware(ppmu, priv->acr.pmu_desc);

	return 0;
}
/*!
*	Wait for PMU halt interrupt status to be cleared
*	@param[in]	g		GPU object pointer
*	@param[in]	timeout_us	Timeout in Us for PMU to halt
*	@return '0' if PMU halt irq status is clear
*/
int clear_halt_interrupt_status(struct nvkm_pmu *pmu, unsigned int timeout)
{
	u32 data = 0;

	while (timeout != 0) {
		nv_mask(pmu, 0x0010a004, 0x10, 0x10);
		data = nv_rd32(pmu, 0x0010a008);
		if ((data & 0x10) != 0x10)
			break;
		timeout--;
		udelay(1);
	}

	if (timeout == 0)
		return -EBUSY;
	return 0;
}

/*!
*	Wait for PMU to halt
*	@param[in]	g		GPU object pointer
*	@param[in]	timeout_us	Timeout in Us for PMU to halt
*	@return '0' if PMU halts
*/
int pmu_wait_for_halt(struct nvkm_pmu *pmu, unsigned int timeout)
{
	u32 data = 0;

	udelay(10);
	data = nv_rd32(pmu, 0x0010a100);
	nv_debug(pmu, "cpuctl %x, timeout %d\n", data, timeout);
	while (timeout != 0) {
		data = nv_rd32(pmu, 0x0010a100);
		if (data & (0x1 << 4))
			break;
		timeout--;
		udelay(1);
	}

	if (timeout == 0) {
		nv_error(pmu, "Timeout happens in pmu_wait_for_halt.\n");
		return -EBUSY;
	}

	data = nv_rd32(pmu, 0x0010a040);
	if (data) {
		nv_error(pmu, "ACR boot failed, err %x\n", data);
		return -EAGAIN;
	}

	return 0;
}

static int bl_bootstrap(struct nvkm_pmu *ppmu,
	struct flcn_bl_dmem_desc *pbl_desc, u32 bl_sz)
{
	struct gk20a_pmu_priv *pmu = to_gk20a_priv(ppmu);
	struct gm20b_acr *acr = &pmu->acr;
	u32 imem_dst_blk = 0;
	u32 virt_addr = 0;
	u32 tag = 0;
	u32 index = 0;
	struct hsflcn_bl_desc *pmu_bl_gm10x_desc = &acr->pmu_hsbl_desc;

	nv_mask(ppmu, 0x0010a048, 0x1, 0x1);
	nv_wr32(ppmu, 0x0010a480, (0xfffffff &
			pmu->pmuvm.mem->addr >> 12) |
			((0x1) << 30) | 0x20000000);

	/*copy bootloader interface structure to dmem*/
	nv_wr32(ppmu, 0x0010a1c0, (0 | (0x1 << 24)));
	gk20a_pmu_copy_to_dmem(pmu, 0, (u8 *)pbl_desc,
		sizeof(struct flcn_bl_dmem_desc), 0);

	/* Now copy bootloader to TOP of IMEM */
	imem_dst_blk = (nv_rd32(ppmu, 0x0010a108) & 0x1ff) - bl_sz/256;

	/* Set Auto-Increment on write */
	nv_wr32(ppmu, 0x0010a180,
			((imem_dst_blk & 0xff) << 8) | (0x1 << 24));
	virt_addr = pmu_bl_gm10x_desc->bl_start_tag << 8;
	tag = virt_addr >> 8; /* tag is always 256B aligned */
	for (index = 0; index < bl_sz/4; index++) {
		if ((index % 64) == 0) {
			nv_wr32(ppmu, 0x0010a188, (tag & 0xffff) << 0);
			tag++;
		}
		nv_wr32(ppmu, 0x0010a184,
				nv_ro32(acr->hsbl_ucode.obj, index * 4));
	}

	nv_wr32(ppmu, 0x0010a188, (0 & 0xffff) << 0);
	nv_debug(ppmu, "Before starting falcon with BL\n");
	nv_wr32(ppmu, 0x0010a104, (virt_addr & 0xffffffff));
	nv_wr32(ppmu, 0x0010a100, 0x2);

	return 0;
}

static int gm20b_init_pmu_setup_hw1(struct nvkm_pmu *ppmu,
		struct flcn_bl_dmem_desc *desc, u32 bl_sz)
{

	struct nvkm_mc *pmc = nvkm_mc(ppmu);
	struct gk20a_pmu_priv *pmu = to_gk20a_priv(ppmu);
	struct gm20b_acr *acr = &pmu->acr;
	int err;
	struct pmu_cmdline_args_v1 args;

	/* TODO remove this when perfmon is used for pstate/dvfs scaling */
	pmu->perfmon_sampling_enabled = false;

	if (!pmu->trace_buf.size) {
		err = nvkm_gpuobj_new(nv_object(ppmu), NULL,
					GK20A_PMU_TRACE_BUFSIZE, 0x1000, 0,
					&pmu->trace_buf.obj);
		if (err) {
			nv_error(ppmu, "alloc for trace buf failed\n");
			return err;
		}

		err = nvkm_gpuobj_map_vm(nv_gpuobj(pmu->trace_buf.obj),
					pmu->pmuvm.vm,
					NV_MEM_ACCESS_RW,
					&pmu->trace_buf.vma);
		if (err) {
			nv_error(ppmu, "mapping of trace buf failed\n");
			return err;
		}

		pmu->trace_buf.size = GK20A_PMU_TRACE_BUFSIZE;
	}

	if (!pmu->seq_buf.size) {
		err = nvkm_gpuobj_new(nv_object(ppmu), NULL,
					GK20A_PMU_SEQ_BUFSIZE, 0x1000, 0,
					&pmu->seq_buf.obj);
		if (err) {
			nv_error(ppmu, "alloc for seq buf failed\n");
			return err;
		}
		err = nvkm_gpuobj_map_vm(nv_gpuobj(pmu->seq_buf.obj),
						pmu->pmuvm.vm,
						NV_MEM_ACCESS_RW,
						&pmu->seq_buf.vma);
		if (err) {
			nv_error(ppmu, "mapping of seq buf failed\n");
			return err;
		}

		pmu->seq_buf.size = GK20A_PMU_SEQ_BUFSIZE;
	}

	memset(&args, 0x00, sizeof(struct pmu_cmdline_args_v1));

	gk20a_pmu_seq_init(pmu);
	INIT_WORK(&pmu->pg_init, gk20a_pmu_setup_hw);
	reinit_completion(&pmu->lspmu_completion);
	reinit_completion(&ppmu->gr_init);
	mutex_lock(&pmu->isr_mutex);
	pmu_reset(ppmu, pmc);
	pmu->isr_enabled = true;
	mutex_unlock(&pmu->isr_mutex);
	INIT_WORK(&pmu->base.recv.work, gk20a_pmu_process_message);

	/* setup apertures - virtual */
	nv_wr32(ppmu, 0x0010ae00 + 4 * (GK20A_PMU_DMAIDX_UCODE),
			0x4 | 0x0);
	nv_wr32(ppmu, 0x0010ae00 + 4 * (GK20A_PMU_DMAIDX_VIRT),
		0x0);
	/* setup apertures - physical */
	nv_wr32(ppmu, 0x0010ae00 + 4 * (GK20A_PMU_DMAIDX_PHYS_VID),
		0x4);
	nv_wr32(ppmu, 0x0010ae00 + 4 * (GK20A_PMU_DMAIDX_PHYS_SYS_COH),
			0x4 | 0x1);
	nv_wr32(ppmu, 0x0010ae00 + 4 * (GK20A_PMU_DMAIDX_PHYS_SYS_NCOH),
			0x4 | 0x2);

	args.cpu_freq_hz = 0;
	args.secure_mode = 1;
	args.falc_trace_size = GK20A_PMU_TRACE_BUFSIZE;
	args.falc_trace_dma_base = ((u32)pmu->trace_buf.vma.offset) >> 8;
	args.falc_trace_dma_idx = GK20A_PMU_DMAIDX_VIRT;
	gk20a_pmu_copy_to_dmem(pmu, acr->pmu_args,
			(u8 *)(&args), sizeof(args), 0);
	/*disable irqs for hs falcon booting as we will poll for halt*/
	mutex_lock(&pmu->isr_mutex);
	gk20a_pmu_enable_irq(pmu, pmc, false);
	pmu->isr_enabled = false;
	mutex_unlock(&pmu->isr_mutex);

	return bl_bootstrap(ppmu, desc, bl_sz);
}

/*
* Executes a generic bootloader and wait for PMU to halt.
* This BL will be used for those binaries that are loaded
* and executed at times other than RM PMU Binary execution.
*
* @param[in] g			gk20a pointer
* @param[in] desc		Bootloader descriptor
* @param[in] dma_idx		DMA Index
* @param[in] b_wait_for_halt	Wait for PMU to HALT
*/
int pmu_exec_gen_bl(struct nvkm_pmu *pmu, void *desc, u8 b_wait_for_halt)
{
	int i, err = 0;
	struct gk20a_pmu_priv *priv = to_gk20a_priv(pmu);
	struct gm20b_acr *acr = &priv->acr;
	u32 bl_sz;
	const struct firmware *hsbl_fw = acr->hsbl_fw;
	struct hsflcn_bl_desc *pmu_bl_desc = &acr->pmu_hsbl_desc;
	struct hsflcn_bl_desc *pmu_hsbl_desc;
	u32 *pmu_bl = NULL;

	if (!acr->hsbl_ucode.size) {
		err = gk20a_load_firmware(pmu, &hsbl_fw,
			GM20B_HSBIN_PMU_BL_UCODE_IMAGE);
		if (err) {
			nv_error(pmu, "HSbin BL ucode load failed\n");
			return -ENOENT;
		}
		acr->hsbl_fw = hsbl_fw;
		acr->bl_bin_hdr = (struct bin_hdr *)hsbl_fw->data;
		pmu_hsbl_desc = (struct hsflcn_bl_desc *)(hsbl_fw->data +
				acr->bl_bin_hdr->header_offset);

		*pmu_bl_desc = *pmu_hsbl_desc;
		pmu_bl = (u32 *)(hsbl_fw->data +
			acr->bl_bin_hdr->data_offset);
		bl_sz = ALIGN(pmu_bl_desc->bl_img_hdr.bl_code_size,
				256);
		acr->hsbl_ucode.size = bl_sz;
		nv_debug(pmu, "Executing Generic Bootloader\n");

		err = nvkm_gpuobj_new(nv_object(pmu), NULL, bl_sz, 0x1000, 0,
				&acr->hsbl_ucode.obj);
		if (err) {
			nv_error(pmu, "acr bl ucode alloc failed\n");
			gk20a_release_firmware(pmu, hsbl_fw);
			acr->hsbl_fw = NULL;
			return -ENOMEM;
		}

		err = nvkm_gpuobj_map_vm(nv_gpuobj(acr->hsbl_ucode.obj),
					priv->pmuvm.vm, NV_MEM_ACCESS_RW,
					&acr->hsbl_ucode.vma);
		if (err) {
			nv_error(pmu, "acr bl ucode mapping to vm failed\n");
			gk20a_release_firmware(pmu, hsbl_fw);
			acr->hsbl_fw = NULL;
			goto err_free_gpuobj;
		}

		nv_debug(pmu, "acr->hsbl_ucode.vma.offset is: 0x%llx\n",
			acr->hsbl_ucode.vma.offset);

		for (i = 0; i < (bl_sz) >> 2; i++)
			nv_wo32(acr->hsbl_ucode.obj, i * 4, pmu_bl[i]);

		nv_debug(pmu, "Copied HSBL ucode to iova\n");
		gk20a_release_firmware(pmu, hsbl_fw);
		acr->hsbl_fw = NULL;
	}
	/*
	 * Disable interrupts to avoid kernel hitting breakpoint due
	 * to PMU halt
	 */

	if (clear_halt_interrupt_status(pmu, GK20A_IDLE_CHECK_DEFAULT))
		goto err_unmap_bl;

	nv_debug(pmu, "err reg :%x\n", ioread32_native(mc +
			MC_ERR_GENERALIZED_CARVEOUT_STATUS_0));
	nv_debug(pmu, "phys sec reg %x\n", nv_rd32(pmu, 0x00100ce4));
	nv_debug(pmu, "before ACR exec sctl %x\n", nv_rd32(pmu, 0x0010a240));

	err = gm20b_init_pmu_setup_hw1(pmu, desc, acr->hsbl_ucode.size);
	if (err) {
		nv_error(pmu, "gm20b_init_pmu_setup_hw1 failed\n");
		goto err_unmap_bl;
	}

	/* Poll for HALT */
	if (b_wait_for_halt) {
		err = pmu_wait_for_halt(pmu, GK20A_IDLE_CHECK_DEFAULT);
		if (err == 0) {
			/* Clear the HALT interrupt */
		  if (clear_halt_interrupt_status(pmu,
						GK20A_IDLE_CHECK_DEFAULT))
			goto err_unmap_bl;
		} else
			goto err_unmap_bl;
	}

	nv_debug(pmu, "after waiting for halt, err %x\n", err);
	nv_debug(pmu, "MC_ERR_GENERALIZED_CARVEOUT_STATUS_0: 0x%X\n",
		ioread32_native(mc + MC_ERR_GENERALIZED_CARVEOUT_STATUS_0));
	nv_debug(pmu, "MC_ERR_GENERALIZED_CARVEOUT_ADR_0: 0x%X\n",
		ioread32_native(mc + 0xc04));

	nv_debug(pmu, "phys sec reg %x\n", nv_rd32(pmu,
		0x00100ce4));
	nv_debug(pmu, "sctl reg %x\n", nv_rd32(pmu, 0x0010a240));

	priv->pmu_state = PMU_STATE_STARTING;
	start_gm20b_pmu(pmu);

	return 0;
err_unmap_bl:
	nvkm_gpuobj_unmap(&priv->acr.hsbl_ucode.vma);
err_free_gpuobj:
	nvkm_gpuobj_ref(NULL, &priv->acr.hsbl_ucode.obj);

	return err;
}


/*Loads ACR bin to FB mem and bootstraps PMU with bootloader code
 * start and end are addresses of ucode blob in non-WPR region*/

int gm20b_bootstrap_hs_flcn(struct nvkm_pmu *ppmu)
{
	int i, err = 0, offset = 0;
	u64 *acr_dmem;
	u32 img_size_in_bytes = 0;
	u32 size;
	u64 start;
	struct gk20a_pmu_priv *pmu = to_gk20a_priv(ppmu);
	struct gm20b_acr *acr = &pmu->acr;
	const struct firmware *acr_fw = acr->acr_fw;
	struct flcn_bl_dmem_desc *bl_dmem_desc = &acr->bl_dmem_desc;
	u32 *acr_ucode_header_t210_load;
	u32 *acr_ucode_data_t210_load;

	start = acr->ucode_blob_start;
	size = acr->ucode_blob_size;

	if (!acr->acr_ucode.size) {
		/*First time init case*/
		err = gk20a_load_firmware(ppmu, &acr_fw,
						GM20B_HSBIN_PMU_UCODE_IMAGE);
		if (err) {
			nv_error(ppmu, "pmu ucode get fail\n");
			return -ENOENT;
		}
		acr->acr_fw = acr_fw;
		acr->hsbin_hdr = (struct bin_hdr *)acr_fw->data;
		acr->fw_hdr = (struct acr_fw_header *)(acr_fw->data +
				acr->hsbin_hdr->header_offset);
		acr_ucode_data_t210_load = (u32 *)(acr_fw->data +
				acr->hsbin_hdr->data_offset);
		acr_ucode_header_t210_load = (u32 *)(acr_fw->data +
				acr->fw_hdr->hdr_offset);
		img_size_in_bytes = ALIGN((acr->hsbin_hdr->data_size), 256);

		/*Lets patch the signatures first..*/
		if (acr_ucode_patch_sig(ppmu, acr_ucode_data_t210_load,
					(u32 *)(acr_fw->data +
						acr->fw_hdr->sig_prod_offset),
					(u32 *)(acr_fw->data +
						acr->fw_hdr->sig_dbg_offset),
					(u32 *)(acr_fw->data +
						acr->fw_hdr->patch_loc),
					(u32 *)(acr_fw->data +
						acr->fw_hdr->patch_sig)) < 0) {
			nv_error(ppmu, "patch signatures fail\n");
			err = -1;
			gk20a_release_firmware(ppmu, acr_fw);
			acr->acr_fw = NULL;
			return err;
		}

		err = nvkm_gpuobj_new(nv_object(ppmu), NULL,
				img_size_in_bytes, 0x1000, 0,
				&acr->acr_ucode.obj);
		if (err) {
			nv_error(ppmu, "alloc for acr ucode failed\n");
			gk20a_release_firmware(ppmu, acr_fw);
			acr->acr_fw = NULL;
			return -ENOMEM;
		}
		err = nvkm_gpuobj_map_vm(nv_gpuobj(acr->acr_ucode.obj),
						pmu->pmuvm.vm,
						NV_MEM_ACCESS_RW,
						&acr->acr_ucode.vma);
		if (err) {
			nv_error(ppmu, "alloc for acr ucode failed\n");
			gk20a_release_firmware(ppmu, acr_fw);
			acr->acr_fw = NULL;
			goto err_map_acr_gpuobj;
		}
		nv_debug(ppmu, "acr_ucode.vma.offset is: 0x%llX\n",
			acr->acr_ucode.vma.offset);

		acr_dmem = (u64 *)
			&(((u8 *)acr_ucode_data_t210_load)[
					acr_ucode_header_t210_load[2]]);
		((struct flcn_acr_desc *)acr_dmem)->nonwpr_ucode_blob_start =
			start;
		((struct flcn_acr_desc *)acr_dmem)->nonwpr_ucode_blob_size =
			size;
		((struct flcn_acr_desc *)acr_dmem)->regions.no_regions = 2;
		((struct flcn_acr_desc *)acr_dmem)->wpr_offset = 0;

		for (i = 0; i < (img_size_in_bytes/4); i++) {
			nv_wo32(acr->acr_ucode.obj,
				offset , acr_ucode_data_t210_load[i]);
			offset += 4;
		}

		acr->acr_ucode.size = img_size_in_bytes;

		/* * In order to execute this binary, we will be using
		   * a bootloader which will load this image into PMU IMEM/DMEM.
		   * Fill up the bootloader descriptor for PMU HAL to use..*/


		bl_dmem_desc->signature[0] = 0;
		bl_dmem_desc->signature[1] = 0;
		bl_dmem_desc->signature[2] = 0;
		bl_dmem_desc->signature[3] = 0;
		bl_dmem_desc->ctx_dma = GK20A_PMU_DMAIDX_VIRT;
		bl_dmem_desc->code_dma_base =
			lower_32_bits(((u64)acr->acr_ucode.vma.offset >> 8));
		bl_dmem_desc->non_sec_code_off  = acr_ucode_header_t210_load[0];
		bl_dmem_desc->non_sec_code_size = acr_ucode_header_t210_load[1];
		bl_dmem_desc->sec_code_off = acr_ucode_header_t210_load[5];
		bl_dmem_desc->sec_code_size = acr_ucode_header_t210_load[6];
		bl_dmem_desc->code_entry_point = 0;
		bl_dmem_desc->data_dma_base =
			bl_dmem_desc->code_dma_base +
			((acr_ucode_header_t210_load[2]) >> 8);
		bl_dmem_desc->data_size = acr_ucode_header_t210_load[3];
		gk20a_release_firmware(ppmu, acr_fw);
		acr->acr_fw = NULL;
	}
	err = pmu_exec_gen_bl(ppmu, bl_dmem_desc, 1);
	if (err)
		goto err_acr_exec;

	return 0;

err_acr_exec:
	nvkm_gpuobj_unmap(&pmu->acr.acr_ucode.vma);
err_map_acr_gpuobj:
	nvkm_gpuobj_ref(NULL, &pmu->acr.acr_ucode.obj);

	return err;
}

static void
pmu_handle_acr_init_wpr_msg(struct nvkm_pmu *pmu, struct pmu_msg *msg,
			void *param, u32 handle, u32 status)
{
	struct gk20a_pmu_priv *priv = param;

	if (msg->msg.acr.acrmsg.error_code == PMU_ACR_SUCCESS) {
		nv_debug(pmu, "reply PMU_ACR_CMD_ID_INIT_WPR_REGION done\n");
		priv->lspmu_wpr_init_done = true;
		complete(&priv->lspmu_completion);
	}
}

int
gm20b_pmu_init_acr(struct nvkm_pmu *pmu)
{
	struct gk20a_pmu_priv *priv = to_gk20a_priv(pmu);
	struct pmu_cmd cmd;
	u32 seq;

	/* init ACR */
	memset(&cmd, 0, sizeof(cmd));

	cmd.hdr.unit_id = PMU_UNIT_ACR;
	cmd.hdr.size = PMU_CMD_HDR_SIZE +
				sizeof(struct pmu_acr_cmd_init_wpr_details);
	cmd.cmd.acr.init_wpr.cmd_type = PMU_ACR_CMD_ID_INIT_WPR_REGION;
	cmd.cmd.acr.init_wpr.region_id = 0x01;
	cmd.cmd.acr.init_wpr.wpr_offset = 0x00;

	nv_debug(pmu, "cmd post PMU_ACR_CMD_ID_INIT_WPR_REGION\n");

	gk20a_pmu_cmd_post(pmu, &cmd, NULL, NULL, PMU_COMMAND_QUEUE_HPQ,
			pmu_handle_acr_init_wpr_msg, priv, &seq, ~0);

	return 0;
}

static void
pmu_handle_fecs_boot_acr_msg(struct nvkm_pmu *pmu,
		struct pmu_msg *msg, void *param, u32 handle, u32 status)
{
	struct gk20a_pmu_priv *priv = param;

	if (msg->msg.acr.acrmsg.falcon_id == LSF_FALCON_ID_FECS) {
		nv_debug(pmu, "reply PMU_ACR_CMD_ID_BOOTSTRAP_FALCON\n");
		priv->recovery_in_progress = false;
		complete(&priv->lspmu_completion);
	}

	nv_debug(pmu, "response code = %x\n", msg->msg.acr.acrmsg.falcon_id);
}

int
gm20b_pmu_load_lsf(struct nvkm_pmu *pmu, u8 falcon_id)
{
	struct gk20a_pmu_priv *priv = to_gk20a_priv(pmu);
	struct pmu_cmd cmd;
	u32 seq;

	nv_debug(pmu, "wprinit status = %x\n", priv->lspmu_wpr_init_done);

	if (!priv->lspmu_wpr_init_done) {
		nv_error(pmu, "lspmu WPR init not done\n");
		return -EAGAIN;
	}

	/* send message to load FECS falcon */
	memset(&cmd, 0, sizeof(struct pmu_cmd));

	cmd.hdr.unit_id = PMU_UNIT_ACR;
	cmd.hdr.size = PMU_CMD_HDR_SIZE +
			sizeof(struct pmu_acr_cmd_bootstrap_falcon);
	cmd.cmd.acr.bootstrap_falcon.cmd_type =
			PMU_ACR_CMD_ID_BOOTSTRAP_FALCON;
	cmd.cmd.acr.bootstrap_falcon.flags =
			PMU_ACR_CMD_BOOTSTRAP_FALCON_FLAGS_RESET_YES;
	cmd.cmd.acr.bootstrap_falcon.falcon_id = falcon_id;

	nv_debug(pmu, "cmd post PMU_ACR_CMD_ID_BOOTSTRAP_FALCON\n");

	gk20a_pmu_cmd_post(pmu, &cmd, NULL, NULL, PMU_COMMAND_QUEUE_HPQ,
			pmu_handle_fecs_boot_acr_msg, priv, &seq, ~0);

	return 0;
}

int
gm20b_boot_fecs(struct nvkm_pmu *pmu)
{
	struct gk20a_pmu_priv *priv = to_gk20a_priv(pmu);
	int ret;

	priv->recovery_in_progress = true;

	wait_for_completion_timeout(&priv->lspmu_completion,
							  usecs_to_jiffies(50));
	if (!priv->lspmu_wpr_init_done) {
		nv_error(pmu, "lspmu wpr init not completed, timeout\n");
		return -ETIMEDOUT;
	}

	reinit_completion(&priv->lspmu_completion);

	ret = gm20b_pmu_load_lsf(pmu, LSF_FALCON_ID_FECS);
	if (ret) {
		nv_error(pmu, "loading/booting LSF falcon failed\n");
		return ret;
	}

	wait_for_completion_timeout(&priv->lspmu_completion,
							  usecs_to_jiffies(50));

	if (priv->recovery_in_progress) {
		nv_error(pmu, "recovery not completed, timeout\n");
		return -ETIMEDOUT;
	}

	return 0;
}

int gm20b_boot_secure(struct nvkm_pmu *ppmu)
{
	struct gk20a_pmu_priv *priv = to_gk20a_priv(ppmu);
	int ret;

	ret = gm20b_bootstrap_hs_flcn(ppmu);
	if (ret) {
		nv_error(ppmu, "%s failed\n", __func__);
		goto error;
	}

	ppmu->cold_boot = false;
	/* Have to re-init the idle count ctrl after secure boot */
	gk20a_pmu_dvfs_init(priv);

	return ret;

error:
	nvkm_gpuobj_unmap(&priv->seq_buf.vma);
	nvkm_gpuobj_ref(NULL, &priv->seq_buf.obj);
	nvkm_gpuobj_unmap(&priv->trace_buf.vma);
	nvkm_gpuobj_ref(NULL, &priv->trace_buf.obj);
	nvkm_gpuobj_unmap(&priv->acr.acr_ucode.vma);
	nvkm_gpuobj_ref(NULL, &priv->acr.acr_ucode.obj);
	nvkm_gpuobj_unmap(&priv->acr.hsbl_ucode.vma);
	nvkm_gpuobj_ref(NULL, &priv->acr.hsbl_ucode.obj);

	return ret;
}

extern struct gk20a_pmu_dvfs_data gk20a_dvfs_data;

static int
gm20b_pmu_ctor(struct nvkm_object *parent, struct nvkm_object *engine,
	       struct nvkm_oclass *oclass, void *data, u32 size,
	       struct nvkm_object **pobject)
{
	struct gk20a_pmu_priv *priv;
	int ret, i;
	struct nvkm_pmu *ppmu;

	ret = nvkm_pmu_create(parent, engine, oclass, &priv);
	*pobject = nv_object(priv);
	if (ret)
		return ret;

	priv->pmu_chip_data = kzalloc(sizeof(struct pmu_gk20a_data),
			GFP_KERNEL);
	if (!priv->pmu_chip_data) {
		nv_error(priv, "failed to allocate mem for chip data\n");
		return -ENOMEM;
	}

	priv->mutex_cnt = MUTEX_CNT;
	priv->mutex = kzalloc(priv->mutex_cnt *
				sizeof(struct pmu_mutex), GFP_KERNEL);
	if (!priv->mutex) {
		nv_error(priv, "alloc for pmu_mutexes failed\n");
		return -ENOMEM;
	}

	for (i = 0; i < priv->mutex_cnt; i++)
		priv->mutex[i].index = i;

	priv->seq = kzalloc(PMU_MAX_NUM_SEQUENCES *
		sizeof(struct pmu_sequence), GFP_KERNEL);
	if (!priv->seq) {
		nv_error(priv, "alloc for sequences failed\n");
		kfree(priv->mutex);
		return -ENOMEM;
	}

	ppmu = &priv->base;
	mc = ioremap(TEGRA_MC_BASE, 0x00000d00);
	ppmu->cold_boot = true;
	priv->allow_elpg = true;

	nv_subdev(ppmu)->intr = gk20a_pmu_intr;

	priv->data = &gk20a_dvfs_data;
	nvkm_alarm_init(&priv->alarm, gk20a_pmu_dvfs_work);
	mutex_init(&priv->isr_mutex);
	mutex_init(&priv->allow_elpg_mutex);
	init_completion(&priv->lspmu_completion);
	init_completion(&priv->elpg_off_completion);
	init_completion(&priv->elpg_on_completion);
	init_completion(&ppmu->gr_init);

	ret = gm20b_prepare_ucode_blob(ppmu);
	if (ret)
		nv_error(ppmu, "prepare ucode failed\n");

	ret = gk20a_pmu_debugfs_register(priv);
	if (ret)
		nv_error(ppmu, "debugfs register failed\n");

	return ret;
}

static int
gm20b_pmu_fini(struct nvkm_object *object, bool suspend)
{
	struct gk20a_pmu_priv *priv = (void *)object;
	struct nvkm_mc *pmc = nvkm_mc(object);
	struct nvkm_pmu *pmu = &priv->base;

	nvkm_timer_alarm_cancel(priv, &priv->alarm);

	if (suspend) {
		cancel_work_sync(&priv->base.recv.work);
		cancel_work_sync(&priv->pg_init);
		mutex_lock(&priv->isr_mutex);
		gk20a_pmu_enable(priv, pmc, false);
		priv->isr_enabled = false;
		mutex_unlock(&priv->isr_mutex);
		priv->pmu_state = PMU_STATE_OFF;
		mutex_lock(&priv->clk_gating_mutex);
		priv->clk_gating_disable_depth = 0;
		mutex_unlock(&priv->clk_gating_mutex);
		pmu->elcg_enabled = false;
		pmu->slcg_enabled = false;
		pmu->blcg_enabled = false;
		priv->pmu_ready = false;
		priv->out_of_reset = false;
		pmu->cold_boot = true;
		priv->lspmu_wpr_init_done = false;
		nv_wr32(priv, 0x10a014, 0x00000060);
	} else {
		mutex_lock(&priv->isr_mutex);
		gk20a_pmu_enable(priv, pmc, true);
		pmu_reset(pmu, pmc);
		mutex_unlock(&priv->isr_mutex);
	}

	return nvkm_subdev_fini(&priv->base.base, suspend);
}

static void
gm20b_pmu_dtor(struct nvkm_object *object)
{
	struct nvkm_pmu *ppmu = (void *)object;
	struct gk20a_pmu_priv *pmu = to_gk20a_priv(ppmu);

	nvkm_gpuobj_unmap(&pmu->acr.ucode_blob.vma);
	nvkm_gpuobj_ref(NULL, &pmu->acr.ucode_blob.obj);
	nvkm_vm_ref(NULL, &pmu->pmuvm.vm, pmu->pmuvm.pgd);
	nvkm_gpuobj_ref(NULL, &pmu->pmuvm.pgd);
	nvkm_gpuobj_ref(NULL, &pmu->pmuvm.mem);
	nvkm_gpuobj_unmap(&pmu->acr.acr_ucode.vma);
	nvkm_gpuobj_ref(NULL, &pmu->acr.acr_ucode.obj);
	nvkm_gpuobj_unmap(&pmu->acr.hsbl_ucode.vma);
	nvkm_gpuobj_ref(NULL, &pmu->acr.hsbl_ucode.obj);
	ppmu->cold_boot = true;
	gk20a_pmu_allocator_destroy(&pmu->dmem);
	gk20a_pmu_debugfs_unregister(pmu);
	mutex_destroy(&pmu->allow_elpg_mutex);
	kfree(pmu->mutex);
	kfree(pmu->seq);
	kfree(pmu->pmu_chip_data);
}

static int
gm20b_pmu_init(struct nvkm_object *object) {
	struct nvkm_pmu *ppmu = (void *)object;
	struct gk20a_pmu_priv *priv = to_gk20a_priv(ppmu);
	int ret;

	ret = nvkm_subdev_init(&ppmu->base);
	if (ret) {
		nv_error(ppmu, "subdev init failed\n");
		return ret;
	}

	mutex_init(&priv->pmu_copy_lock);
	mutex_init(&priv->pmu_seq_lock);
	mutex_init(&priv->elpg_mutex);
	mutex_init(&priv->clk_gating_mutex);
	ppmu->secure_bootstrap = gm20b_boot_secure;
	ppmu->boot_fecs = gm20b_boot_fecs;
	ppmu->enable_elpg = gk20a_pmu_enable_elpg;
	ppmu->disable_elpg = gk20a_pmu_disable_elpg;
	ppmu->enable_clk_gating = gm20b_pmu_enable_clk_gating;
	ppmu->disable_clk_gating = gm20b_pmu_disable_clk_gating;
	ppmu->fecs_secure_boot = true;
	ppmu->gpccs_secure_boot = false;
	ppmu->elcg_enabled = true;
	ppmu->slcg_enabled = true;
	ppmu->blcg_enabled = true;
	priv->out_of_reset = true;

	priv->pmu_setup_elpg = gm20b_pmu_setup_elpg;

	mutex_lock(&priv->elpg_mutex);
	/*
	 * ELPG will be enabled when PMU finishes booting, so setting the
	 * counter to 1 initialy.
	 */
	priv->elpg_disable_depth = 1;
	mutex_unlock(&priv->elpg_mutex);

	mutex_lock(&priv->clk_gating_mutex);
	priv->clk_gating_disable_depth = 0;
	mutex_unlock(&priv->clk_gating_mutex);

	gk20a_pmu_dvfs_init(priv);

	nvkm_timer_alarm(priv, PMU_DVFS_INTERVAL, &priv->alarm);

	return ret;
}

struct nvkm_oclass *
gm20b_pmu_oclass = &(struct nvkm_pmu_impl) {
	.base.handle = NV_SUBDEV(PMU, 0x12b),
	.base.ofuncs = &(struct nvkm_ofuncs) {
		.ctor = gm20b_pmu_ctor,
		.dtor = gm20b_pmu_dtor,
		.init = gm20b_pmu_init,
		.fini = gm20b_pmu_fini,
	},
	.base.handle = NV_SUBDEV(PMU, 0x12b),
	.acquire_mutex = gk20a_pmu_mutex_acquire,
	.release_mutex = gk20a_pmu_mutex_release,
} .base;
