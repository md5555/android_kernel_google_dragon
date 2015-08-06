#ifndef __GK20A_MMU_PRIV__
#define __GK20A_MMU_PRIV__

#include <subdev/fb.h>
#include <subdev/mmu.h>

#include "gf100.h"

void gk20a_instobj_map_sg(struct nvkm_vma *vma, struct nvkm_object *object,
			  struct nvkm_mem *mem, u32 pte, u32 cnt,
			  dma_addr_t *list);

void gk20a_instobj_unmap_sg(struct nvkm_object *object, u32 pte, u32 cnt);

int gk20a_mmu_ctor(struct nvkm_object *parent, struct nvkm_object *engine,
		   struct nvkm_oclass *oclass, void *data, u32 size,
		   struct nvkm_object **pobject);

#endif
