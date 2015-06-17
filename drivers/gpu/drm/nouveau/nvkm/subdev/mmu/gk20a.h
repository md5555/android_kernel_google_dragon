#ifndef __GK20A_MMU_PRIV__
#define __GK20A_MMU_PRIV__

#include <subdev/fb.h>
#include <subdev/mmu.h>

#include "gf100.h"

int gk20a_mmu_ctor(struct nvkm_object *parent, struct nvkm_object *engine,
	   struct nvkm_oclass *oclass, void *data, u32 size,
	   struct nvkm_object **pobject);

#endif
