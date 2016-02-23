/*
 * Copyright (c) 2015, NVIDIA CORPORATION. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <core/gpuobj.h>

#include <subdev/fb.h>
#include <subdev/ltc.h>
#include <subdev/mmu.h>
#include <subdev/timer.h>

#ifdef __KERNEL__
#include <linux/iommu.h>
#include <nouveau_platform.h>
#endif

#include "gk20a.h"

struct gk20a_mapping {
	struct sg_table *sg;
	u64 iova;
	u64 phys;
	u64 length;
	struct kref ref;
	struct nvkm_mmu *mmu;
	struct nvkm_mm_node *node;
};

extern void
gk20a_instobj_map_sg(struct nvkm_vma *vma, struct nvkm_object *object,
		struct nvkm_mem *mem, u32 pte, u32 cnt, dma_addr_t *list,
		u64 delta);
extern void
gk20a_instobj_unmap_sg(struct nvkm_object *object, u32 pte, u32 cnt);

static struct gk20a_mapping*
gk20a_mapping_get(struct nvkm_mmu *mmu, dma_addr_t addr, u64 length)
{
	struct gk20a_mmu_priv *priv = (void *)mmu;
	struct gk20a_mapping *mapping;

	mutex_lock(&priv->mapping_lock);

	mapping = radix_tree_lookup(&priv->mapping_tree, addr);
	if (mapping) {
		struct nouveau_platform_device *plat =
			nv_device_to_platform(nv_device(&mmu->base));
		u64 addr = iommu_iova_to_phys(plat->gpu->iommu.domain,
				mapping->iova);

		WARN(addr != mapping->phys,
				"IOVA=0x%016llx, SG=0x%016llx, PHYS=0x%016llx\n",
				mapping->iova, mapping->phys, addr);
		if (mapping->length != length) {
			nv_error(mmu, "IOMMU mapping length mismatch\n");
			mapping = ERR_PTR(-EINVAL);
			goto lookup_err;
		}

		kref_get(&mapping->ref);
	}

lookup_err:
	mutex_unlock(&priv->mapping_lock);

	return mapping;
}

/* Must be called with mapping_lock locked */
static void
gk20a_mapping_release(struct kref *ref)
{
	struct gk20a_mapping *mapping = container_of(ref,
			struct gk20a_mapping, ref);
	struct gk20a_mapping *tmp;
	struct nvkm_mmu *mmu = mapping->mmu;
	struct gk20a_mmu_priv *priv = (void *)mmu;
	struct nouveau_platform_device *plat;
	int ret;

	plat = nv_device_to_platform(nv_device(&mmu->base));

	WARN_ON(!mutex_is_locked(&priv->mapping_lock));

	ret = iommu_unmap(plat->gpu->iommu.domain, mapping->iova,
			mapping->length);
	if (ret != mapping->length)
		nv_error(mmu, "failed to unmap IOMMU address 0x%16llx\n",
				mapping->iova);

	mutex_lock(&plat->gpu->iommu.mutex);
	nvkm_mm_free(plat->gpu->iommu.mm, &mapping->node);
	mutex_unlock(&plat->gpu->iommu.mutex);

	tmp = radix_tree_delete(&priv->mapping_tree, mapping->phys);
	if (tmp != mapping)
		nv_error(priv, "invalid item in big pages!\n");

	nv_trace(priv, "%s() putting mapping for IOVA 0x%16llx\n", __func__,
			mapping->iova);
	kfree(mapping);

	mutex_unlock(&priv->mapping_lock);
}

static int
gk20a_mapping_insert(struct nvkm_mmu *mmu, struct gk20a_mapping *mapping,
		struct gk20a_mapping **mapped)
{
	struct gk20a_mmu_priv *priv = (void *)mmu;
	int ret = 0;

	mutex_lock(&priv->mapping_lock);
	*mapped = radix_tree_lookup(&priv->mapping_tree, mapping->phys);
	if (!*mapped)
		ret = radix_tree_insert(&priv->mapping_tree, mapping->phys, mapping);
	else
		kref_get(&(*mapped)->ref);
	mutex_unlock(&priv->mapping_lock);

	return ret;
}

static void
gk20a_vm_map(struct nvkm_vma *vma, struct nvkm_gpuobj *pgt,
	     struct nvkm_mem *mem, u32 pte, u32 cnt, u64 phys, u64 delta)
{
	gk20a_instobj_map(vma, pgt->parent, mem, pte, cnt, phys, delta);
}

static void
gk20a_vm_map_sg(struct nvkm_vma *vma, struct nvkm_gpuobj *pgt,
		struct nvkm_mem *mem, u32 pte, u32 cnt, dma_addr_t *list,
		u64 delta)
{
	gk20a_instobj_map_sg(vma, pgt->parent, mem, pte, cnt, list, delta);
}

static void
gk20a_vm_unmap(struct nvkm_gpuobj *pgt, u32 pte, u32 cnt)
{
	gk20a_instobj_unmap_sg(pgt->parent, pte, cnt);
}

/*
 * gk20a_vm_map_sg_iommu: map SG table on platform IOMMU
 *
 * @mmu: mmu subdev
 * @sg: SG table for imported buffer
 * @length: buffer length
 * @iova: IOMMU VA (output)
 */
static void *
gk20a_vm_map_sg_iommu(struct nvkm_mmu *mmu, struct sg_table *sg, u64 length,
		u64 *iova)
{
	struct nouveau_platform_device *plat;
	struct nvkm_mm_node *node;
	struct gk20a_mapping *mapping, *mapped = NULL;
	int npages = length >> PAGE_SHIFT;
	size_t mapped_len;
	int ret = 0;

	if (length % (1 << mmu->lpg_shift) != 0) {
		nv_error(mmu, "invalid length for iommu mapping\n");
		return ERR_PTR(-EINVAL);
	}

	plat = nv_device_to_platform(nv_device(&mmu->base));

	/*
	 * Check if the buffer is already mapped into IOMMU by other VM. If yes,
	 * return the mapped IOVA, otherwise create a new mapping.
	 */
	mapping = gk20a_mapping_get(mmu, sg_phys(sg->sgl), length);
	if (IS_ERR(mapping)) {
		return mapping;
	} else if (mapping) {
		*iova = mapping->iova;
		return mapping;
	}

	mapping = kzalloc(sizeof(struct gk20a_mapping), GFP_KERNEL);
	if (!mapping) {
		nv_error(mmu, "failed to allocate mapping\n");
		return ERR_PTR(-ENOMEM);
	}

	mutex_lock(&plat->gpu->iommu.mutex);
	ret = nvkm_mm_head(plat->gpu->iommu.mm, 0, 1, npages, npages,
			(1 << mmu->lpg_shift) >> 12, &node);
	mutex_unlock(&plat->gpu->iommu.mutex);
	if (ret) {
		nv_error(mmu, "failed to allocate iommu space\n");
		goto iommu_va_err;
	}

	mapped_len = iommu_map_sg(plat->gpu->iommu.domain,
			node->offset << PAGE_SHIFT,
			sg->sgl, sg->nents, IOMMU_READ | IOMMU_WRITE);

	if (mapped_len != length) {
		nv_error(mmu, "failed to map iommu\n");
		ret = -EINVAL;
		goto iommu_map_err;
	}

	mapping->iova = (u64)node->offset << PAGE_SHIFT;
	mapping->iova |= BIT_ULL(plat->gpu->iommu.translation_enable_bit);
	mapping->phys = sg_phys(sg->sgl);
	mapping->length = length;
	mapping->mmu = mmu;
	mapping->node = node;
	kref_init(&mapping->ref);
	ret = gk20a_mapping_insert(mmu, mapping, &mapped);
	if (mapped) {
		*iova = mapped->iova;
		goto remove_dup;
	} else if (ret) {
		nv_error(mmu, "failed to insert address 0x%016llx\n",
				mapping->phys);
		goto iommu_map_err;
	}

	nv_trace(mmu, "%s(): IOVA=0x%016llx, PHYS=0x%016llx\n", __func__,
			mapping->iova, mapping->phys);

	*iova = mapping->iova;

	return mapping;

remove_dup:
	iommu_unmap(plat->gpu->iommu.domain, mapping->iova, mapping->length);
iommu_map_err:
	mutex_lock(&plat->gpu->iommu.mutex);
	nvkm_mm_free(plat->gpu->iommu.mm, &node);
	mutex_unlock(&plat->gpu->iommu.mutex);
iommu_va_err:
	kfree(mapping);

	if (mapped)
		return mapped;
	else
		return ERR_PTR(ret);
}

static void
gk20a_vm_unmap_iommu(struct nvkm_vma *vma, void *priv)
{
	struct gk20a_mmu_priv *mmu = (void *)vma->vm->mmu;
	struct gk20a_mapping *mapping= priv;

	nv_trace(mmu, "%s() unmapping IOVA 0x%16llx\n", __func__,
			mapping->iova);
	kref_put_mutex(&mapping->ref, gk20a_mapping_release,
				&mmu->mapping_lock);
}

extern const u8 gf100_pte_storage_type_map[256];

static u32
gk20a_vm_uc_type(struct nvkm_mmu *mmu, u32 type)
{
	return mmu->storage_type_map[(u8)(type & 0xff)];
}

int
gk20a_mmu_ctor(struct nvkm_object *parent, struct nvkm_object *engine,
	       struct nvkm_oclass *oclass, void *data, u32 size,
	       struct nvkm_object **pobject)
{
	struct gk20a_mmu_priv *priv;
	struct nouveau_platform_device *plat;
	int ret;

	ret = nvkm_mmu_create(parent, engine, oclass, "VM", "vm", &priv);
	*pobject = nv_object(priv);
	if (ret)
		return ret;

	plat = nv_device_to_platform(nv_device(parent));
	if (plat->gpu->iommu.domain)
		priv->base.iommu_capable = true;

	priv->base.limit = 1ULL << 40;
	priv->base.dma_bits = 40;
	priv->base.pgt_bits  = 27 - 12;
	priv->base.spg_shift = 12;
	priv->base.lpg_shift = 17;
	priv->base.create = gf100_vm_create;
	priv->base.create_pgd = gf100_vm_create_pgd;
	priv->base.map_pgt = gf100_vm_map_pgt;
	priv->base.map = gk20a_vm_map;
	priv->base.map_sg = gk20a_vm_map_sg;
	priv->base.map_sg_iommu = gk20a_vm_map_sg_iommu;
	priv->base.unmap = gk20a_vm_unmap;
	priv->base.unmap_iommu = gk20a_vm_unmap_iommu;
	priv->base.flush = gf100_vm_flush;
	priv->base.uc_type = gk20a_vm_uc_type;
	priv->base.storage_type_map = gf100_pte_storage_type_map;
	INIT_RADIX_TREE(&priv->mapping_tree, GFP_KERNEL);
	mutex_init(&priv->mapping_lock);

	return 0;
}

struct nvkm_oclass
gk20a_mmu_oclass = {
	.handle = NV_SUBDEV(MMU, 0xea),
	.ofuncs = &(struct nvkm_ofuncs) {
		.ctor = gk20a_mmu_ctor,
		.dtor = _nvkm_mmu_dtor,
		.init = _nvkm_mmu_init,
		.fini = _nvkm_mmu_fini,
	},
};
