/*
 *
 * Copyright (c) 2015, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef _gk20a_h_
#define _gk20a_h_

#include <subdev/mmu.h>
#include <subdev/timer.h>
#include <subdev/mc.h>
#include <linux/debugfs.h>

#define BUSY_SLOT	0
#define CLK_SLOT	7

/* Used by PERFMON (performance monitoring) task running on PMU*/
#define PERFMON_BUSY_SLOT	3
#define PERFMON_CLK_SLOT	6

/* Writen by sw, read by Pmu, protected by sw mutex lock High Prio Q. */
#define PMU_COMMAND_QUEUE_HPQ		0
/* Writen by sw, read by Pmu, protected by sw mutex lock Low Prio Q. */
#define PMU_COMMAND_QUEUE_LPQ		1

/* To/Can be used by Nouveau to get the raw counter values */
#define NV_CLK_SLOT	1
#define NV_BUSY_SLOT	2

#define GK20A_PMU_UCODE_NB_MAX_OVERLAY	    32
#define GK20A_PMU_UCODE_NB_MAX_DATE_LENGTH  64
#define GK20A_PMU_TRACE_BUFSIZE             0x4000
#define GK20A_PMU_SEQ_BUFSIZE               0x1000

#define PMU_QUEUE_COUNT			5

#define PMU_MAX_NUM_SEQUENCES		256
#define PMU_SEQ_TBL_SIZE		BITS_TO_LONGS(PMU_MAX_NUM_SEQUENCES)

#define MUTEX_CNT			16

#define PMU_CMD_HDR_SIZE	sizeof(struct pmu_hdr)

#define PMU_UNIT_ACR			(0x0A)

/* Choices for pmu_state */
enum {
	PMU_STATE_OFF,             /* 0  PMU is off */
	PMU_STATE_STARTING,        /* 1  PMU is on, but not booted */
	PMU_STATE_INIT_RECEIVED,   /* 2  PMU init message received */
	PMU_STATE_ELPG_BOOTING,	   /* 3  PMU is booting */
	PMU_STATE_ELPG_BOOTED,	   /* 4  ELPG is initialized */
	PMU_STATE_LOADING_PG_BUF,  /* 5  Loading PG buf */
	PMU_STATE_LOADING_ZBC,	   /* 6  Loading ZBC buf */
	PMU_STATE_STARTED          /* 7  Fully unitialized */
};

struct pmu_mutex {
	u32 index;
	u32 ref_cnt;
};

struct pmu_payload {
	struct {
		void *buf;
		u32 offset;
		u16 size;
	} in, out;
};

struct pmu_mem_gk20a {
	u32 dma_base;
	u8  dma_offset;
	u8  dma_idx;
	u16 fb_size;
};

/* Struct defining a chunk of PMU dmem. */
struct pmu_dmem {
	u16 size;
	u32 offset;
};

struct pmu_allocation_gk20a {
	struct {
		struct pmu_dmem dmem;
		struct pmu_mem_gk20a fb;
	} alloc;
};

/* PERFMON */
#define PMU_DOMAIN_GROUP_PSTATE		0
#define PMU_DOMAIN_GROUP_GPC2CLK	1
#define PMU_DOMAIN_GROUP_NUM		2

enum pmu_perfmon_cmd_start_fields {
	COUNTER_ALLOC
};

#define PMU_PERFMON_PCT_TO_INC		58
#define PMU_PERFMON_PCT_TO_DEC		23

struct pmu_perfmon_counter_gk20a {
	u8 index;
	u8 flags;
	u8 group_id;
	u8 valid;
	u16 upper_threshold; /* units of 0.01% */
	u16 lower_threshold; /* units of 0.01% */
};

struct pmu_gk20a_data {
	struct pmu_perfmon_counter_gk20a perfmon_counter_gk20a;
	u32 perfmon_state_id[PMU_DOMAIN_GROUP_NUM];
};

#define PMU_PERFMON_FLAG_ENABLE_INCREASE	(0x00000001)
#define PMU_PERFMON_FLAG_ENABLE_DECREASE	(0x00000002)
#define PMU_PERFMON_FLAG_CLEAR_PREV		(0x00000004)

/* PERFMON CMD */
enum {
	PMU_PERFMON_CMD_ID_START = 0,
	PMU_PERFMON_CMD_ID_STOP  = 1,
	PMU_PERFMON_CMD_ID_INIT  = 2
};

/* PERFMON MSG */
enum {
	PMU_PERFMON_MSG_ID_INCREASE_EVENT = 0,
	PMU_PERFMON_MSG_ID_DECREASE_EVENT = 1,
	PMU_PERFMON_MSG_ID_INIT_EVENT     = 2,
	PMU_PERFMON_MSG_ID_ACK            = 3
};

struct gm20b_ctxsw_ucode_segment {
	u32 offset;
	u32 size;
};

struct gm20b_ctxsw_ucode_segments {
	u32 boot_entry;
	u32 boot_imem_offset;
	u32 boot_signature;
	struct gm20b_ctxsw_ucode_segment boot;
	struct gm20b_ctxsw_ucode_segment code;
	struct gm20b_ctxsw_ucode_segment data;
};

struct gm20b_ctxsw_ucode_info {
	struct gm20b_ctxsw_ucode_segments fecs;
	struct gm20b_ctxsw_ucode_segments gpccs;
};

struct pmu_cmdline_args_gk20a {
	u32 cpu_freq_hz;		/* Frequency of the clock driving PMU */
	u32 falc_trace_size;		/* falctrace buffer size (bytes) */
	u32 falc_trace_dma_base;	/* 256-byte block address */
	u32 falc_trace_dma_idx;		/* dmaIdx for DMA operations */
	u8 secure_mode;
	struct pmu_mem_gk20a gc6_ctx;		/* dmem offset of gc6 context */
};

struct pmu_ucode_desc {
	u32 descriptor_size;
	u32 image_size;
	u32 tools_version;
	u32 app_version;
	char date[GK20A_PMU_UCODE_NB_MAX_DATE_LENGTH];
	u32 bootloader_start_offset;
	u32 bootloader_size;
	u32 bootloader_imem_offset;
	u32 bootloader_entry_point;
	u32 app_start_offset;
	u32 app_size;
	u32 app_imem_offset;
	u32 app_imem_entry;
	u32 app_dmem_offset;
	u32 app_resident_code_offset;
	u32 app_resident_code_size;
	u32 app_resident_data_offset;
	u32 app_resident_data_size;
	u32 nb_overlays;
	struct {u32 start; u32 size; } load_ovl[GK20A_PMU_UCODE_NB_MAX_OVERLAY];
	u32 compressed;
};

struct pmu_rc_msg_unhandled_cmd {
	u8 msg_type;
	u8 unit_id;
};

struct pmu_rc_msg {
	u8 msg_type;
	struct pmu_rc_msg_unhandled_cmd unhandled_cmd;
};

/**
 * Structure pmu_hdr  - Header structure for pmu cmd(that we send) or pmu msg(that we receive).
 * unit_id            - Comp in PMU to/from which cmd sent or msg received.
 * size               - Total size of pmu cmd or pmu msg.
 * ctrl_flags         - Flag to indicate type of msg/cmd.
 * seq_id             - Sequence id to match a pmu msg to pmu cmd.
 */
struct pmu_hdr {
	u8 unit_id;
	u8 size;
	u8 ctrl_flags;
	u8 seq_id;
};

enum {
	PMU_PG_CMD_ID_ELPG_CMD = 0,
	PMU_PG_CMD_ID_ENG_BUF_LOAD,
	PMU_PG_CMD_ID_ENG_BUF_UNLOAD,
	PMU_PG_CMD_ID_PG_STAT,
	PMU_PG_CMD_ID_PG_LOG_INIT,
	PMU_PG_CMD_ID_PG_LOG_FLUSH,
	PMU_PG_CMD_ID_PG_PARAM,
	PMU_PG_CMD_ID_ELPG_INIT,
	PMU_PG_CMD_ID_ELPG_POLL_CTXSAVE,
	PMU_PG_CMD_ID_ELPG_ABORT_POLL,
	PMU_PG_CMD_ID_ELPG_PMU_UP,
	PMU_PG_CMD_ID_ELPG_DISALLOW,
	PMU_PG_CMD_ID_ELPG_ALLOW,
	PMU_PG_CMD_ID_AP,
	PMU_PG_CMD_ID_PSI,
	PMU_PG_CMD_ID_CG,
	PMU_PG_CMD_ID_ZBC_TABLE_UPDATE,
	PMU_PG_CMD_ID_PMU_RAIL_GATE_DISABLE = 0x20,
	PMU_PG_CMD_ID_PMU_RAIL_GATE_ENABLE,
	PMU_PG_CMD_ID_PMU_RAIL_SMU_MSG_DISABLE
};

enum {
	PMU_PG_ELPG_CMD_INIT,
	PMU_PG_ELPG_CMD_DISALLOW,
	PMU_PG_ELPG_CMD_ALLOW,
	PMU_PG_ELPG_CMD_FREEZE,
	PMU_PG_ELPG_CMD_UNFREEZE,
};

struct pmu_pg_cmd_elpg_cmd {
	u8 cmd_type;
	u8 engine_id;
	u16 cmd;
};

struct pmu_pg_cmd_eng_buf_load {
	u8 cmd_type;
	u8 engine_id;
	u8 buf_idx;
	u8 pad;
	u16 buf_size;
	u32 dma_base;
	u8 dma_offset;
	u8 dma_idx;
};

enum {
	PMU_PG_STAT_CMD_ALLOC_DMEM = 0,
};

struct pmu_pg_cmd_stat {
	u8 cmd_type;
	u8 engine_id;
	u16 sub_cmd_id;
	u32 data;
};

struct pmu_pg_cmd {
	union {
		u8 cmd_type;
		struct pmu_pg_cmd_elpg_cmd elpg_cmd;
		struct pmu_pg_cmd_eng_buf_load eng_buf_load;
		struct pmu_pg_cmd_stat stat;
	};
};

struct pmu_perfmon_cmd_start_gk20a {
	u8 cmd_type;
	u8 group_id;
	u8 state_id;
	u8 flags;
	struct pmu_allocation_gk20a counter_alloc;
};

struct pmu_perfmon_cmd_stop {
	u8 cmd_type;
};

struct pmu_perfmon_cmd_init_gk20a {
	u8 cmd_type;
	u8 to_decrease_count;
	u8 base_counter_id;
	u32 sample_period_us;
	struct pmu_allocation_gk20a counter_alloc;
	u8 num_counters;
	u8 samples_in_moving_avg;
	u16 sample_buffer;
};

struct pmu_perfmon_cmd {
	union {
		u8 cmd_type;
		struct pmu_perfmon_cmd_start_gk20a start_gk20a;
		struct pmu_perfmon_cmd_stop stop;
		struct pmu_perfmon_cmd_init_gk20a init_gk20a;
	};
};

struct pmu_zbc_cmd {
	u8 cmd_type;
	u8 pad;
	u16 entry_mask;
};

/*
 * Struct to contain PMU init msg format.
 * This is the structure used by PMU firmware
 * to communicate INIT msg from PMU firmware
 * to Nouveau.
 *
 * NOTE: This structure should not be changed unless
 * we do the same change @ PMU firmware side.
 */
struct pmu_init_msg_pmu_gk20a {
	u8 msg_type;
	u8 pad;
	u16  os_debug_entry_point;

	struct {
		u16 size;
		u16 offset;
		u8 index;
		u8 pad;
	} queue_info[PMU_QUEUE_COUNT];

	u16 sw_managed_area_offset;
	u16 sw_managed_area_size;
};

/*
 * ELPG(Engine Level Clock gating) message structure.
 * msg_type     -  ELPG message type.
 * engine_id    -  Engine ID.
 * msg          -  ELPG msg.
 *
 * NOTE: This structure should not be changed unless
 * we do the same change @ PMU firmware side.
 */
struct pmu_pg_msg_elpg_msg {
	u8 msg_type;
	u8 engine_id;
	u16 msg;
};

enum {
	PMU_PG_STAT_MSG_RESP_DMEM_OFFSET = 0,
};

struct pmu_pg_msg_stat {
	u8 msg_type;
	u8 engine_id;
	u16 sub_msg_id;
	u32 data;
};

enum {
	PMU_PG_MSG_ENG_BUF_LOADED,
	PMU_PG_MSG_ENG_BUF_UNLOADED,
	PMU_PG_MSG_ENG_BUF_FAILED,
};

struct pmu_pg_msg_eng_buf_stat {
	u8 msg_type;
	u8 engine_id;
	u8 buf_idx;
	u8 status;
};

/*
 * Generic PG message structure
 * elpg_msg     -  ELPG message struct.
 * stat         -  ELPG stats msg struct.
 * eng_buf_stat -  ELPG eng buff stat msg struct.
 */
struct pmu_pg_msg {
	union {
		u8 msg_type;
		struct pmu_pg_msg_elpg_msg elpg_msg;
		struct pmu_pg_msg_stat stat;
		struct pmu_pg_msg_eng_buf_stat eng_buf_stat;
	};
};

struct pmu_perfmon_msg_generic {
	u8 msg_type;
	u8 state_id;
	u8 group_id;
	u8 data;
};

struct pmu_perfmon_msg {
	union {
		u8 msg_type;
		struct pmu_perfmon_msg_generic gen;
	};
};
/*
 * Bootstrap ACR message structure
 * error_code -  error code if error occurred.
 * falcon_id  -  falcon_id specifying falcon i.e booted.
 *
 * NOTE: This structure should not be changed unless
 * we do the same change @ PMU firmware side.
 */
struct pmu_acr_msg_bootstrap_falcon {
	u8 msg_type;
	union {
		u32 error_code;
		u32 falcon_id;
	};
};


/*
 * PMU ACR message structure
 * msg_type  - type of message.
 * acrmsg    - bootstrap ACR message.
 *
 * NOTE: This structure should not be changed unless
 * we do the same change @ PMU firmware side.
 */
struct pmu_acr_msg {
	union {
		u8 msg_type;
		struct pmu_acr_msg_bootstrap_falcon acrmsg;
	};
};

/* ACR Commands/Message structures */
enum {
	PMU_ACR_CMD_ID_INIT_WPR_REGION = 0x0          ,
	PMU_ACR_CMD_ID_BOOTSTRAP_FALCON,
};

/*
 * ACR WPR init command structure
 * cmd_type   - type of command.
 * regionid   - specifying region ID in WPR.
 * wpr_offset - wpr offset in WPR region.
 *
 * NOTE: This structure should not be changed unless
 * we do the same change @ PMU firmware side.
 */
struct pmu_acr_cmd_init_wpr_details {
	u8  cmd_type;
	u32 region_id;
	u32 wpr_offset;

};

/*
 * ACR bootstrap falcon command structure
 * cmd_type   - type of command.
 * flags      - Flag specifying RESET or no RESET.
 * falcon id  - Falcon id specifying falcon to bootstrap.
 *
 * NOTE: This structure should not be changed unless
 * we do the same change @ PMU firmware side.
 */
struct pmu_acr_cmd_bootstrap_falcon {
	u8 cmd_type;
	u32 flags;
	u32 falcon_id;
};

#define PMU_ACR_CMD_BOOTSTRAP_FALCON_FLAGS_RESET_NO  1
#define PMU_ACR_CMD_BOOTSTRAP_FALCON_FLAGS_RESET_YES 0

/*
 * Generic acr command structure
 * cmd_type     -   type of ACR command.
 * bootstrap_falcon  - ACR command to bootstrap FALCON.
 * init_wpr          - ACR command to initialise WPR details.
 *
 * NOTE:
 * More type of commands (structs) can be added to this generic struct.
 * The command(struct) added should have same format in PMU fw as well.
 * i.e same struct should be present on PMU fw and Nouveau.
 */
struct pmu_acr_cmd {
	union {
		u8 cmd_type;
		struct pmu_acr_cmd_bootstrap_falcon bootstrap_falcon;
		struct pmu_acr_cmd_init_wpr_details init_wpr;
	};
};

/* ACR messages */

/*
 * Returns the WPR region init information.
 */
#define PMU_ACR_MSG_ID_INIT_WPR_REGION   0
/*
 * Returns the Bootstrapped falcon ID to Nouveau
 */
#define PMU_ACR_MSG_ID_BOOTSTRAP_FALCON  1

/*
 * Returns the WPR init status
 */
#define PMU_ACR_SUCCESS                  0
#define PMU_ACR_ERROR                    1

/*
 * Struct to contain PMU cmd format.
 * hdr     - PMU hdr format.
 * perfmon - Perfmon cmd i.e It's a cmd used to start/init Perfmon TASK in PMU.
 * acr     - ACR command type
 * pg      - Powergating command type
 * zbc     - ZBC command type
 *
 * NOTE:
 * More type of commands (structs) can be added to this generic struct.
 * The command(struct) added should have same format in PMU fw as well.
 * i.e same struct should be present on PMU fw and Nouveau.
 */
struct pmu_cmd {
	struct pmu_hdr hdr;
	union {
		struct pmu_perfmon_cmd perfmon;
		struct pmu_acr_cmd acr;
		struct pmu_pg_cmd pg;
		struct pmu_zbc_cmd zbc;
	} cmd;
};

struct pmu_init_msg {
	union {
		u8 msg_type;
		struct pmu_init_msg_pmu_gk20a pmu_init_gk20a;
	};
};

/*
 * Struct to contain PMU generic msg format.
 * hdr          -    header structure for all types of msgs.
 * union:
 *	init    -    PMU init related PMU msg
 *	rc      -    pmu_rc related PMU msg
 *	perfmon -    Perfmon related PMU msg
 *	acr     -    ACR related PMU msg
 *	pg      -    PG(Power gating) related PMU msg
 */
struct pmu_msg {
	struct pmu_hdr hdr;
	union {
		struct pmu_init_msg init;
		struct pmu_rc_msg rc;
		struct pmu_perfmon_msg perfmon;
		struct pmu_acr_msg acr;
		struct pmu_pg_msg pg;
	} msg;
};

typedef void (*pmu_callback)(struct nvkm_pmu *, struct pmu_msg *,
			     void *, u32, u32);
struct pmu_sequence {
	u8 id;
	u32 state;
	u32 desc;
	struct pmu_msg *msg;
	struct pmu_allocation_gk20a in_gk20a;
	struct pmu_allocation_gk20a out_gk20a;
	u8 *out_payload;
	pmu_callback callback;
	void *cb_params;
};

/*Choices for DMA to use*/
enum {
	GK20A_PMU_DMAIDX_UCODE		= 0,
	GK20A_PMU_DMAIDX_VIRT		= 1,
	GK20A_PMU_DMAIDX_PHYS_VID	= 2,
	GK20A_PMU_DMAIDX_PHYS_SYS_COH	= 3,
	GK20A_PMU_DMAIDX_PHYS_SYS_NCOH	= 4,
};
/*
 * bl_code_off        - Offset of code section in the image
 * bl_code_size          - Size of code section in the image
 * bl_data_off        - Offset of data section in the image
 * bl_data_size          - Size of data section in the image
 */
struct hsflcn_bl_img_hdr {
	u32 bl_code_off;
	u32 bl_code_size;
	u32 bl_data_off;
	u32 bl_data_size;
};

/*
 * Structure used by the boot-loader to load the rest of the code. This has
 * to be filled by host and copied into DMEM at offset provided in the
 * hsflcn_bl_desc.bl_desc_dmem_load_off.
 *
 * signature         - 16B signature for secure code. 0s if no secure code
 * ctx_dma           - CtxDma to be used by BL while loading code/data
 * code_dma_base     - 256B aligned Physical FB Address where code is located
 * non_sec_code_off  - Offset from code_dma_base where the nonSecure code is
 *                     located. The offset must be multiple of 256 to help perf
 * non_sec_code_size - The size of the nonSecure code part.
 * sec_code_size     - Offset from code_dma_base where the secure code is
 *                     located. The offset must be multiple of 256 to help perf
 * code_entry_point  - Code entry point which will be invoked by BL after
 *			code is loaded.
 * data_dma_base     - 256B aligned Physical FB Address where data is located.
 * data_size         - Size of data block. Should be multiple of 256B
 */
struct flcn_bl_dmem_desc {
	u32    reserved[4];
	u32    signature[4];
	u32    ctx_dma;
	u32    code_dma_base;
	u32    non_sec_code_off;
	u32    non_sec_code_size;
	u32    sec_code_off;
	u32    sec_code_size;
	u32    code_entry_point;
	u32    data_dma_base;
	u32    data_size;
};
/*The descriptor used to figure out the requirements of boot loader.
 * bl_start_tag          - Starting tag of bootloader
 * bl_desc_dmem_load_off - Dmem offset where _def_rm_flcn_bl_dmem_desc
   to be loaded
 * bl_img_hdr            - Description of the image
 */
struct hsflcn_bl_desc {
	u32 bl_start_tag;
	u32 bl_desc_dmem_load_off;
	struct hsflcn_bl_img_hdr bl_img_hdr;
};
struct bin_hdr {
	u32 bin_magic;      /* 0x10de */
	u32 bin_ver;          /* versioning of bin format */
	u32 bin_size;         /* entire image size including this header */
	u32 header_offset; /* Header offset of executable binary metadata,
				start @ offset- 0x100 */
	u32 data_offset; /* Start of executable binary data, start @
				offset- 0x200 */
	u32 data_size; /* Size ofexecutable binary */
};

struct acr_fw_header {
	u32 sig_dbg_offset;
	u32 sig_dbg_size;
	u32 sig_prod_offset;
	u32 sig_prod_size;
	u32 patch_loc;
	u32 patch_sig;
	u32 hdr_offset; /*this header points to acr_ucode_header_t210_load*/
	u32 hdr_size; /*size of above header*/
};

/*Structure describing ACR*/
struct gm20b_acr {
	u64 ucode_blob_start;/*Start address of ucode blob*/
	u32 ucode_blob_size;/*ucode blob size*/
	struct bin_hdr *bl_bin_hdr;
	struct hsflcn_bl_desc pmu_hsbl_desc;
	struct bin_hdr *hsbin_hdr;
	struct acr_fw_header *fw_hdr;
	u32 pmu_args;
	const struct firmware *acr_fw;
	struct pmu_buf_desc acr_ucode;
	const struct firmware *hsbl_fw;
	struct pmu_buf_desc hsbl_ucode;
	struct pmu_buf_desc ucode_blob;/*structure for containing ucode blob*/
	struct flcn_bl_dmem_desc bl_dmem_desc;
	const struct firmware *pmu_fw;
	const struct firmware *pmu_desc;
};

/**
 * Struct pmu allocator
 * base            - min value of this linear space
 * size            - total size
 * bitmap          - bitmap
 */
struct nvkm_pmu_allocator {
	u32 base;
	u32 size;
	unsigned long *bitmap;
};

/**
 * Structure pmu_queue
 * mutex id           - used by hw, for BIOS/SMI queue
 * mutex_lock         - used by sw, for LPQ/HPQ queue
 * position           - current write position
 * offset             - physical dmem offset where this queue begins
 * id                 - logical queue identifier
 * index              - physical queue index
 * size               - in bytes
 * oflag              - flag to indentify R/W
 * opened             - opened implied locked
 */
struct pmu_queue {
	u32 mutex_id;
	u32 mutex_lock;
	struct mutex mutex;
	u32 position;
	u32 offset;
	u32 id;
	u32 index;
	u32 size;
	u32 oflag;
	bool opened;
};

struct gk20a_pmu_priv {
	struct nvkm_pmu base;
	struct nvkm_alarm alarm;
	struct gm20b_acr acr;
	struct gk20a_pmu_dvfs_data *data;
	struct pmu_ucode_desc *desc;
	struct pmu_buf_desc ucode;
	u32 *ucode_image;
	struct pmu_buf_desc trace_buf;
	struct pmu_buf_desc seq_buf;
	struct mutex pmu_copy_lock;
	struct mutex pmu_seq_lock;
	struct work_struct pg_init;
	bool pmu_ready;
	bool allow_elpg;
	struct mutex allow_elpg_mutex;
	bool recovery_in_progress;
	bool lspmu_wpr_init_done;
	struct completion lspmu_completion;
	struct completion elpg_off_completion;
	struct completion elpg_on_completion;
	int pmu_state;
	int elpg_disable_depth;
	int clk_gating_disable_depth;
	struct nvkm_pmu_priv_vm pmuvm;
	struct gm20b_ctxsw_ucode_info ucode_info;
	struct mutex isr_mutex;
	struct mutex elpg_mutex;
	struct mutex clk_gating_mutex;
	bool isr_enabled;
	u32 stat_dmem_offset;
	u8 pmu_mode;
	u32 falcon_id;
	u32 elpg_stat;
	struct nvkm_pmu_allocator dmem;
	struct pmu_queue queue[PMU_QUEUE_COUNT];
	u32 next_seq_desc;
	struct pmu_sequence *seq;
	unsigned long pmu_seq_tbl[PMU_SEQ_TBL_SIZE];
	struct pmu_mutex *mutex;
	u32 mutex_cnt;
	unsigned long perfmon_events_cnt;
	bool perfmon_sampling_enabled;
	bool sw_ready;
	bool perfmon_ready;
	bool out_of_reset;
	u32 sample_buffer;
	bool buf_loaded;
	void *pmu_chip_data;
	int (*pmu_setup_elpg)(struct nvkm_pmu *pmu);
	struct dentry *dbgfs_dir;
};

struct gating_desc {
	u32 addr;
	u32 prod;
	u32 disable;
};

enum {
	ENGINE_GR_GK20A		= 0,
	ENGINE_CE2_GK20A	= 1,
	ENGINE_INVAL_GK20A
};

enum {
	ELCG_RUN,  /* clk always run, i.e. disable elcg */
	ELCG_STOP, /* clk is stopped */
	ELCG_AUTO  /* clk will run when non-idle, standard elcg mode */
};

void
gpu_obj_memwr(struct nvkm_gpuobj *ucodeobj, int offset, void *src, int size);

void
gpu_obj_memrd(struct nvkm_gpuobj *ucodeobj, int offset, void *read, int size);

int
gk20a_load_firmware(struct nvkm_pmu *ppmu,
			const struct firmware **pfw, const char *fw_name);

int
gk20a_pmu_enable_hw(struct gk20a_pmu_priv *priv,
					struct nvkm_mc *pmc, bool enable);

void
gk20a_pmu_enable_irq(struct gk20a_pmu_priv *priv,
					struct nvkm_mc *pmc, bool enable);

void
gk20a_release_firmware(struct nvkm_pmu *ppmu, const struct firmware *pfw);

int
gk20a_pmu_idle(struct gk20a_pmu_priv *priv);

int
gk20a_pmu_enable(struct gk20a_pmu_priv *priv, struct nvkm_mc *pmc, bool enable);

void
gk20a_pmu_copy_to_dmem(struct gk20a_pmu_priv *priv, u32 dst,
					u8 *src, u32 size, u8 port);

void
gk20a_pmu_dvfs_init(struct gk20a_pmu_priv *priv);

void
gk20a_pmu_dvfs_work(struct nvkm_alarm *alarm);

void
gk20a_pmu_intr(struct nvkm_subdev *subdev);

void
gk20a_pmu_process_message(struct work_struct *work);

void
gk20a_pmu_allocator_destroy(struct nvkm_pmu_allocator *allocator);

int
gk20a_pmu_cmd_post(struct nvkm_pmu *pmu, struct pmu_cmd *cmd,
		struct pmu_msg *msg, struct pmu_payload *payload,
		u32 queue_id, pmu_callback callback, void *cb_param,
		u32 *seq_desc, unsigned long timeout);

int
gm20b_pmu_init_acr(struct nvkm_pmu *pmu);

void
gk20a_pmu_seq_init(struct gk20a_pmu_priv *priv);

int
gk20a_pmu_enable_elpg(struct nvkm_pmu *pmu);

int
gk20a_pmu_disable_elpg(struct nvkm_pmu *pmu);

void
gk20a_pmu_setup_hw(struct work_struct *work);

void
gk20a_init_elcg_mode(struct nvkm_pmu *ppmu, u32 mode, u32 engine);

void
gk20a_enable_load_gating_prod(struct nvkm_pmu *pmu,
			const struct gating_desc *desc, int size);

void
gk20a_disable_load_gating_prod(struct nvkm_pmu *pmu,
			const struct gating_desc *desc, int size);
int
gk20a_pmu_debugfs_register(struct gk20a_pmu_priv *priv);

void
gk20a_pmu_debugfs_unregister(struct gk20a_pmu_priv *priv);
#define to_gk20a_priv(ptr) container_of(ptr, struct gk20a_pmu_priv, base)

int
gk20a_pmu_mutex_acquire(struct nvkm_pmu *pmu, u32 id, u32 *token);

int
gk20a_pmu_mutex_release(struct nvkm_pmu *pmu, u32 id, u32 *token);

#endif
