#ifndef __NOUVEAU_BO_H__
#define __NOUVEAU_BO_H__

#include <drm/drm_gem.h>

struct nouveau_channel;
struct nouveau_fence;
struct nvkm_vma;

struct nouveau_bo {
	struct ttm_buffer_object bo;
	struct ttm_placement placement;
	u32 valid_domains;
	struct ttm_place placements[3];
	struct ttm_place busy_placements[3];
	bool force_coherent;
	bool gpu_cacheable;
	struct ttm_bo_kmap_obj kmap;
	struct list_head head;

	/* protected by ttm_bo_reserve() */
	struct drm_file *reserved_by;
	struct list_head entry;
	int pbbo_index;
	bool validate_mapped;

	struct list_head vma_list;
	struct mutex vma_list_lock;
	bool vma_immutable;
	unsigned page_shift;

	u32 tile_mode;
	u32 tile_flags;
	struct nouveau_drm_tile *tile;

	/* Only valid if allocated via nouveau_gem_new() and iff you hold a
	 * gem reference to it! For debugging, use gem.filp != NULL to test
	 * whether it is valid. */
	struct drm_gem_object gem;

	/* protect by the ttm reservation lock */
	int pin_refcnt;

	struct ttm_bo_kmap_obj dma_buf_vmap;
};

static inline struct nouveau_bo *
nouveau_bo(struct ttm_buffer_object *bo)
{
	return container_of(bo, struct nouveau_bo, bo);
}

static inline int
nouveau_bo_ref(struct nouveau_bo *ref, struct nouveau_bo **pnvbo)
{
	struct nouveau_bo *prev;

	if (!pnvbo)
		return -EINVAL;
	prev = *pnvbo;

	*pnvbo = ref ? nouveau_bo(ttm_bo_reference(&ref->bo)) : NULL;
	if (prev) {
		struct ttm_buffer_object *bo = &prev->bo;

		ttm_bo_unref(&bo);
	}

	return 0;
}

extern struct ttm_bo_driver nouveau_bo_driver;

void nouveau_bo_move_init(struct nouveau_drm *);
int  nouveau_bo_new(struct drm_device *, int size, int align, u32 flags,
		    u32 tile_mode, u32 tile_flags, struct sg_table *sg,
		    struct reservation_object *robj,
		    struct nouveau_bo **, bool unchanged_vma_list);
int  nouveau_bo_pin(struct nouveau_bo *, u32 flags, bool contig);
int  nouveau_bo_unpin(struct nouveau_bo *);
int  nouveau_bo_map(struct nouveau_bo *);
void nouveau_bo_unmap(struct nouveau_bo *);
void nouveau_bo_placement_set(struct nouveau_bo *, u32 type, u32 busy);
void nouveau_bo_wr16(struct nouveau_bo *, unsigned index, u16 val);
u32  nouveau_bo_rd32(struct nouveau_bo *, unsigned index);
void nouveau_bo_wr32(struct nouveau_bo *, unsigned index, u32 val);
void nouveau_bo_fence(struct nouveau_bo *, struct nouveau_fence *, bool exclusive);
int  nouveau_bo_validate(struct nouveau_bo *, bool interruptible,
			 bool no_wait_gpu);
void nouveau_bo_sync_for_device(struct nouveau_bo *nvbo);
void nouveau_bo_sync_for_cpu(struct nouveau_bo *nvbo);
int  nouveau_bo_sync(struct nouveau_bo *, struct nouveau_channel *,
		     bool exclusive, bool intr);

void nouveau_bo_l2_invalidate(struct nouveau_bo *nvbo);
void nouveau_bo_l2_flush(struct nouveau_bo *nvbo);

struct nvkm_vma *
nouveau_bo_vma_find(struct nouveau_bo *, struct nvkm_vm *);

struct nvkm_vma *
nouveau_bo_subvma_find(struct nouveau_bo *, struct nvkm_vm *, u64, u64, u64);

struct nvkm_vma *
nouveau_bo_subvma_find_offset(struct nouveau_bo *, struct nvkm_vm *, u64);

int  nouveau_bo_vma_add(struct nouveau_bo *, struct nvkm_vm *,
			struct nvkm_vma *, bool lazy);
int  nouveau_bo_vma_add_offset(struct nouveau_bo *, struct nvkm_vm *,
			       struct nvkm_vma *, u64 offset, bool lazy);
void nouveau_bo_vma_del(struct nouveau_bo *, struct nvkm_vma *);
int  nouveau_bo_subvma_add(struct nouveau_bo *, struct nvkm_vm *,
			   struct nvkm_vma *, u64 offset, u64 delta, u64 length,
			   u32 memtype, bool lazy);
void nouveau_bo_subvma_del(struct nouveau_bo *, struct nvkm_vma *);
void nouveau_defer_vm_map(struct nvkm_vma *vma, struct nouveau_bo *nvbo);
void nouveau_cancel_defer_vm_map(struct nvkm_vma *vma, struct nouveau_bo *nvbo);
void nouveau_bo_vma_list_lock(struct nouveau_bo *);
void nouveau_bo_vma_list_unlock(struct nouveau_bo *);

/* TODO: submit equivalent to TTM generic API upstream? */
static inline void __iomem *
nvbo_kmap_obj_iovirtual(struct nouveau_bo *nvbo)
{
	bool is_iomem;
	void __iomem *ioptr = (void __force __iomem *)ttm_kmap_obj_virtual(
						&nvbo->kmap, &is_iomem);
	WARN_ON_ONCE(ioptr && !is_iomem);
	return ioptr;
}

#endif
