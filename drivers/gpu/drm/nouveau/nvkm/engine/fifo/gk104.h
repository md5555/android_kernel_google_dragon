#ifndef __NVKM_FIFO_NVE0_H__
#define __NVKM_FIFO_NVE0_H__
#include <engine/fifo.h>
#include <subdev/fb.h>

#define _(a,b) { (a), ((1ULL << (a)) | (b)) }
static const struct {
	u64 subdev;
	u64 mask;
} fifo_engine[] = {
	_(NVDEV_ENGINE_GR      , (1ULL << NVDEV_ENGINE_SW) |
				 (1ULL << NVDEV_ENGINE_CE2)),
	_(NVDEV_ENGINE_MSPDEC  , 0),
	_(NVDEV_ENGINE_MSPPP   , 0),
	_(NVDEV_ENGINE_MSVLD   , 0),
	_(NVDEV_ENGINE_CE0     , 0),
	_(NVDEV_ENGINE_CE1     , 0),
	_(NVDEV_ENGINE_MSENC   , 0),
};
#undef _

#define FIFO_ENGINE_NR ARRAY_SIZE(fifo_engine)

struct gk104_fifo_engn {
	struct nvkm_gpuobj *runlist[2];
	int cur_runlist;
	wait_queue_head_t wait;
};

struct gk104_fifo_priv {
	struct nvkm_fifo base;

	struct work_struct fault;
	u64 mask;

	struct gk104_fifo_engn engine[FIFO_ENGINE_NR];
	struct {
		struct nvkm_gpuobj *mem;
		struct nvkm_vma bar;
	} user;
	int spoon_nr;
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
};

extern struct nvkm_ofuncs gk104_fifo_chan_ofuncs;

int  gm204_fifo_ctor(struct nvkm_object *, struct nvkm_object *,
		    struct nvkm_oclass *, void *, u32,
		    struct nvkm_object **);
#endif
