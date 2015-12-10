#ifndef __GF100_MMU_PRIV__
#define __GF100_MMU_PRIV__

#include <subdev/fb.h>
#include <subdev/mmu.h>

struct gf100_mmu_priv {
	struct nvkm_mmu base;
};

int gf100_vm_create(struct nvkm_mmu *mmu, u64 offset, u64 length,
		u64 mm_offset, struct nvkm_vm **pvm);

int gf100_vm_create_pgd(struct nvkm_mmu *mmu, struct nvkm_object *parent,
		  void *inst_blk, u64 length, struct nvkm_gpuobj **ppgd);

void gf100_vm_map_pgt(struct nvkm_gpuobj *pgd, u32 index,
		struct nvkm_gpuobj *pgt[2]);

void gf100_vm_map(struct nvkm_vma *vma, struct nvkm_gpuobj *pgt,
	     struct nvkm_mem *mem, u32 pte, u32 cnt, u64 phys, u64 delta);

void gf100_vm_map_sg(struct nvkm_vma *vma, struct nvkm_gpuobj *pgt,
		struct nvkm_mem *mem, u32 pte, u32 cnt, dma_addr_t *list,
		u64 delta);

void gf100_vm_unmap(struct nvkm_gpuobj *pgt, u32 pte, u32 cnt);

void gf100_vm_flush(struct nvkm_vm *vm);

int gf100_mmu_ctor(struct nvkm_object *parent, struct nvkm_object *engine,
	   struct nvkm_oclass *oclass, void *data, u32 size,
	   struct nvkm_object **pobject);

static inline u64
gf100_vm_addr(struct nvkm_vma *vma, u64 phys, u32 memtype, u32 target)
{
	phys >>= 8;

	phys |= 0x00000001; /* present */
	if (vma->access & NV_MEM_ACCESS_SYS)
		phys |= 0x00000002;

	phys |= ((u64)target  << 32);
	phys |= ((u64)memtype << 36);
	return phys;
}

#endif
