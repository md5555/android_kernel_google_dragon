#ifndef __NVKM_PMU_H__
#define __NVKM_PMU_H__
#include <core/subdev.h>
#include <subdev/mmu.h>

/*
 * Copied from nvgpu driver and should be compatible with NVIDIA proprietary
 * drivers for dGPU.
 */
enum pmu_mutex_id {
	PMU_MUTEX_ID_GPUSER     = 1,
	PMU_MUTEX_ID_QUEUE_BIOS = 2,
	PMU_MUTEX_ID_QUEUE_SMI  = 3,
	PMU_MUTEX_ID_GPMUTEX    = 4,
	PMU_MUTEX_ID_I2C        = 5,
	PMU_MUTEX_ID_RMLOCK     = 6,
	PMU_MUTEX_ID_MSGBOX     = 7,
	PMU_MUTEX_ID_FIFO       = 8,
	PMU_MUTEX_ID_PG         = 9,
	PMU_MUTEX_ID_GR         = 10,
	PMU_MUTEX_ID_CLK        = 11,
	PMU_MUTEX_ID_INVALID    = 16,
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

struct nvkm_pmu {
	struct nvkm_subdev base;

	struct {
		u32 base;
		u32 size;
	} send;

	struct {
		u32 base;
		u32 size;

		struct work_struct work;
		wait_queue_head_t wait;
		u32 process;
		u32 message;
		u32 data[2];
	} recv;

	struct completion gr_init;
	bool fecs_secure_boot;
	bool cold_boot;
	bool gpccs_secure_boot;
	bool elcg_enabled;
	bool slcg_enabled;
	bool blcg_enabled;
	struct pmu_buf_desc pg_buf;
	struct nvkm_pmu_priv_vm *pmu_vm;
	int (*secure_bootstrap)(struct nvkm_pmu *);
	int (*enable_elpg)(struct nvkm_pmu *);
	int (*disable_elpg)(struct nvkm_pmu *);
	int (*enable_clk_gating)(struct nvkm_pmu *);
	int (*disable_clk_gating)(struct nvkm_pmu *);
	int (*boot_fecs)(struct nvkm_pmu *);
	int (*message)(struct nvkm_pmu *, u32[2], u32, u32, u32, u32);
	void (*pgob)(struct nvkm_pmu *, bool);
	int (*acquire_mutex)(struct nvkm_pmu *, u32, u32 *);
	int (*release_mutex)(struct nvkm_pmu *, u32, u32 *);
};

static inline struct nvkm_pmu *
nvkm_pmu(void *obj)
{
	return (void *)nvkm_subdev(obj, NVDEV_SUBDEV_PMU);
}

extern struct nvkm_oclass *gt215_pmu_oclass;
extern struct nvkm_oclass *gf100_pmu_oclass;
extern struct nvkm_oclass *gf110_pmu_oclass;
extern struct nvkm_oclass *gk104_pmu_oclass;
extern struct nvkm_oclass *gk110_pmu_oclass;
extern struct nvkm_oclass *gk208_pmu_oclass;
extern struct nvkm_oclass *gk20a_pmu_oclass;
extern struct nvkm_oclass *gm20b_pmu_oclass;

/* interface to MEMX process running on PMU */
struct nvkm_memx;
int  nvkm_memx_init(struct nvkm_pmu *, struct nvkm_memx **);
int  nvkm_memx_fini(struct nvkm_memx **, bool exec);
void nvkm_memx_wr32(struct nvkm_memx *, u32 addr, u32 data);
void nvkm_memx_wait(struct nvkm_memx *, u32 addr, u32 mask, u32 data, u32 nsec);
void nvkm_memx_nsec(struct nvkm_memx *, u32 nsec);
void nvkm_memx_wait_vblank(struct nvkm_memx *);
void nvkm_memx_train(struct nvkm_memx *);
int  nvkm_memx_train_result(struct nvkm_pmu *, u32 *, int);
void nvkm_memx_block(struct nvkm_memx *);
void nvkm_memx_unblock(struct nvkm_memx *);
#endif
