#ifndef __GK20A_MMU_PRIV__
#define __GK20A_MMU_PRIV__

#include <subdev/fb.h>
#include <subdev/mmu.h>

#include "gf100.h"

struct gk20a_mmu_priv {
	struct nvkm_mmu base;

	/*
	 * Data structures for big page management
	 *
	 * This tree is intended to be used to track all the buffers from all
	 * the clients. One buffer might be shared between clients, and the
	 * clients would map the buffer on their own virtual memory address.
	 * To prevent assigning different IOMMU addresses to the same buffer
	 * which is imported by different clients, we return the existing IOMMU
	 * entries which is maintained by this tree.
	 */
	struct radix_tree_root mapping_tree;
	struct mutex mapping_lock;
};

void gk20a_instobj_map(struct nvkm_vma *vma, struct nvkm_object *object,
		       struct nvkm_mem *mem, u32 pte, u32 cnt, u64 phys,
		       u64 delta);

void
gk20a_instobj_map_pgt(struct nvkm_object *object, u32 index,
			struct nvkm_gpuobj *pgt[2]);

void gk20a_instobj_map_sg(struct nvkm_vma *vma, struct nvkm_object *object,
			  struct nvkm_mem *mem, u32 pte, u32 cnt,
			  dma_addr_t *list, u64 delta);

void gk20a_instobj_unmap_sg(struct nvkm_object *object, u32 pte, u32 cnt);

void gk20a_instobj_set_sparse(struct nvkm_object *object, u32 pte, u32 cnt);

int gk20a_mmu_ctor(struct nvkm_object *parent, struct nvkm_object *engine,
		   struct nvkm_oclass *oclass, void *data, u32 size,
		   struct nvkm_object **pobject);
#endif
