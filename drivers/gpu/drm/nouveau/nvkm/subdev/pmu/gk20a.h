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

#define BUSY_SLOT	0
#define CLK_SLOT	7

#define GK20A_PMU_UCODE_NB_MAX_OVERLAY	    32
#define GK20A_PMU_UCODE_NB_MAX_DATE_LENGTH  64
#define GK20A_PMU_TRACE_BUFSIZE             0x4000

#define PMU_QUEUE_COUNT			5

#define PMU_MAX_NUM_SEQUENCES		256
#define PMU_SEQ_TBL_SIZE		BITS_TO_LONGS(PMU_MAX_NUM_SEQUENCES)

#define MUTEX_CNT			16

/* Choices for pmu_state */
enum {
	PMU_STATE_OFF,             /* 0  PMU is off */
	PMU_STATE_STARTING,        /* 1  PMU is on, but not booted */
	PMU_STATE_INIT_RECEIVED    /* 2  PMU init message received */
};

struct pmu_mutex {
	u32 index;
	u32 ref_cnt;
};

struct pmu_buf_desc {
	struct nvkm_gpuobj *obj;
	struct nvkm_vma vma;
	size_t size;
};

struct nvkm_pmu_priv_vm {
	struct nvkm_gpuobj *mem;
	struct nvkm_gpuobj *pgd;
	struct nvkm_vm *vm;
};

struct pmu_mem_gk20a {
	u32 dma_base;
	u8  dma_offset;
	u8  dma_idx;
	u16 fb_size;
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

struct pmu_init_msg {
	union {
		u8 msg_type;
		struct pmu_init_msg_pmu_gk20a pmu_init_gk20a;
	};
};

/*
 * Struct to contain PMU generic msg format.
 * hdr -  PMU msg header
 * msg -  union of various msg types
 */
struct pmu_msg {
	struct pmu_hdr hdr;
	union {
		struct pmu_init_msg init;
		struct pmu_rc_msg rc;
	} msg;
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
	struct hsflcn_bl_desc *pmu_hsbl_desc;
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
	struct mutex pmu_copy_lock;
	struct mutex pmu_seq_lock;
	bool pmu_ready;
	int pmu_state;
	struct nvkm_pmu_priv_vm pmuvm;
	struct gm20b_ctxsw_ucode_info ucode_info;
	struct mutex isr_mutex;
	bool isr_enabled;
	u8 pmu_mode;
	u32 falcon_id;
	struct nvkm_pmu_allocator dmem;
	struct pmu_queue queue[PMU_QUEUE_COUNT];
	u32 next_seq_desc;
	struct pmu_sequence *seq;
	unsigned long pmu_seq_tbl[PMU_SEQ_TBL_SIZE];
	struct pmu_mutex *mutex;
	u32 mutex_cnt;
	bool sw_ready;
	bool initialized;
	u32 sample_buffer;
	bool buf_loaded;
	void *pmu_chip_data;
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

#define to_gk20a_priv(ptr) container_of(ptr, struct gk20a_pmu_priv, base)

#endif
