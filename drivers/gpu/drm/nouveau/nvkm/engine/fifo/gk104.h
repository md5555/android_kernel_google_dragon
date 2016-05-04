#ifndef __NVKM_FIFO_NVE0_H__
#define __NVKM_FIFO_NVE0_H__
#include <engine/fifo.h>
#include <subdev/fb.h>

struct fifo_engine {
	u64 subdev;
	u64 mask;
};

#define _(a,b) { (a), ((1ULL << (a)) | (b)) }
static const struct fifo_engine gk104_fifo_engines[] = {
	_(NVDEV_ENGINE_GR      , (1ULL << NVDEV_ENGINE_SW) |
				 (1ULL << NVDEV_ENGINE_CE2)),
	_(NVDEV_ENGINE_MSPDEC  , 0),
	_(NVDEV_ENGINE_MSPPP   , 0),
	_(NVDEV_ENGINE_MSVLD   , 0),
	_(NVDEV_ENGINE_CE0     , 0),
	_(NVDEV_ENGINE_CE1     , 0),
	_(NVDEV_ENGINE_MSENC   , 0),
};

static const struct fifo_engine gk20a_fifo_engines[] = {
	_(NVDEV_ENGINE_GR      , (1ULL << NVDEV_ENGINE_SW) |
				 (1ULL << NVDEV_ENGINE_CE2)),
	_(NVDEV_ENGINE_CE2  , 0),
};
#undef _

struct gk104_fifo_engn {
	struct nvkm_gpuobj *runlist[2];
	int cur_runlist;
	wait_queue_head_t wait;
};

struct gk104_fifo_priv {
	struct nvkm_fifo base;

	struct work_struct fault;
	u64 mask;

	struct gk104_fifo_engn *engine;
	struct {
		struct nvkm_gpuobj *mem;
		struct nvkm_vma bar;
	} user;
	int spoon_nr;

	/* MMU fault revoery */
	struct gk104_fifo_chan *fault_chan;
	struct work_struct mmu_fault;
	u32 fault_unit;
};

int  gk104_fifo_ctor(struct nvkm_object *, struct nvkm_object *,
		    struct nvkm_oclass *, void *, u32,
		    struct nvkm_object **);
void gk104_fifo_dtor(struct nvkm_object *);
int  gk104_fifo_init(struct nvkm_object *);
int  gk104_fifo_fini(struct nvkm_object *, bool);

struct gk104_fifo_impl {
	struct nvkm_oclass base;
	u32 channels;
	const struct fifo_engine *engine;
	u32 num_engine;
};

extern struct nvkm_ofuncs gk104_fifo_chan_ofuncs;

int  gm204_fifo_ctor(struct nvkm_object *, struct nvkm_object *,
		    struct nvkm_oclass *, void *, u32,
		    struct nvkm_object **);
#endif
