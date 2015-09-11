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

/*
 * GK20A does not have dedicated video memory, and to accurately represent this
 * fact Nouveau will not create a RAM device for it. Therefore its instmem
 * implementation must be done directly on top of system memory, while providing
 * coherent read and write operations.
 *
 * Instmem can be allocated through two means:
 * 1) If an IOMMU mapping has been probed, the IOMMU API is used to make memory
 *    pages contiguous to the GPU. This is the preferred way.
 * 2) If no IOMMU mapping is probed, the DMA API is used to allocate physically
 *    contiguous memory.
 *
 * In both cases CPU read and writes are performed using PRAMIN (i.e. using the
 * GPU path) to ensure these operations are coherent for the GPU. This allows us
 * to use more "relaxed" allocation parameters when using the DMA API, since we
 * never need a kernel mapping.
 */

#include <subdev/fb.h>
#include <core/mm.h>
#include <core/device.h>

#ifdef __KERNEL__
#include <linux/dma-attrs.h>
#include <linux/iommu.h>
#include <nouveau_platform.h>
#endif

#include "priv.h"

struct gk20a_instobj_priv {
	struct nvkm_instobj base;
	/* Must be second member here - see nouveau_gpuobj_map_vm() */
	struct nvkm_mem *mem;
	/* Pointed by mem */
	struct nvkm_mem _mem;
};

/*
 * Used for objects allocated using the DMA API
 */
struct gk20a_instobj_dma {
	struct gk20a_instobj_priv base;

	void *cpuaddr;
	dma_addr_t handle;
	struct nvkm_mm_node r;
};

/*
 * Used for objects flattened using the IOMMU API
 */
struct gk20a_instobj_iommu {
	struct gk20a_instobj_priv base;

	/* array of base.mem->size pages */
	struct page **pages;
	dma_addr_t *page_addrs;
};

struct gk20a_instmem_priv {
	struct nvkm_instmem base;
	spinlock_t lock;
	u64 addr;

	/* Only used if IOMMU if present */
	struct mutex *mm_mutex;
	struct nvkm_mm *mm;
	struct iommu_domain *domain;
	unsigned long iommu_pgshift;
	unsigned long iommu_addr_bit;

	/* Only used by DMA API */
	struct dma_attrs attrs;
};

/*
 * Use PRAMIN to read/write data and avoid coherency issues.
 * PRAMIN uses the GPU path and ensures data will always be coherent.
 *
 * A dynamic mapping based solution would be desirable in the future, but
 * the issue remains of how to maintain coherency efficiently. On ARM it is
 * not easy (if possible at all?) to create uncached temporary mappings.
 */

static u32
gk20a_instobj_rd32(struct nvkm_object *object, u64 offset)
{
	struct gk20a_instmem_priv *priv = (void *)nvkm_instmem(object);
	struct gk20a_instobj_priv *node = (void *)object;
	unsigned long flags;
	u64 base = (node->mem->offset + offset) & 0xffffff00000ULL;
	u64 addr = (node->mem->offset + offset) & 0x000000fffffULL;
	u32 data;

	spin_lock_irqsave(&priv->lock, flags);
	if (unlikely(priv->addr != base)) {
		nv_wr32(priv, 0x001700, base >> 16);
		priv->addr = base;
	}
	data = nv_rd32(priv, 0x700000 + addr);
	spin_unlock_irqrestore(&priv->lock, flags);
	return data;
}

void
gk20a_instobj_map_sg(struct nvkm_vma *vma, struct nvkm_object *object,
		     struct nvkm_mem *mem, u32 pte, u32 cnt, dma_addr_t *list,
		     u64 delta)
{
	struct gk20a_instmem_priv *priv = (void *)nvkm_instmem(object);
	struct nvkm_mmu *mmu = nvkm_mmu(priv);
	unsigned long flags;
	u32 target = (vma->access & NV_MEM_ACCESS_NOSNOOP) ? 6 : 4;
	u32 memtype = mem->memtype & 0xff;
	u32 *ramin_ptr;
	u32 tag = 0;

	WARN_ON(delta & ((1 << mmu->lpg_shift) - 1));

	if (!mem->cached)
		target |= 1;

	if (!priv->domain) {
		struct gk20a_instobj_dma *node_dma = (void*)object;
		ramin_ptr = node_dma->cpuaddr;
	} else {
		struct gk20a_instobj_iommu *node_iommu = (void*)object;
		int num_ramin_pages, first_ramin_page, last_ramin_page;

		first_ramin_page = pte / 512;
		last_ramin_page = (cnt + pte - 1) / 512;
		num_ramin_pages = last_ramin_page - first_ramin_page + 1;

		ramin_ptr = vmap(&node_iommu->pages[first_ramin_page],
				 num_ramin_pages, VM_MAP,
				 pgprot_writecombine(PAGE_KERNEL));
		pte -= first_ramin_page * 512;
	}

	spin_lock_irqsave(&priv->lock, flags);

	/* Write posting -- ensure that previous mmio has completed */
	nv_rd32(priv, 0x700000);

	if (mem->tag)
		tag = mem->tag->offset + (delta >> mmu->lpg_shift);

	pte <<= 1;
	while (cnt--) {
		ramin_ptr[pte] = (*list >> 8) | 0x1 /* present */;
		ramin_ptr[pte + 1] = target | (memtype << 4) | (tag << 12);
		list ++;
		pte += 2;
		if (mem->tag)
			tag++;
	}

	spin_unlock_irqrestore(&priv->lock, flags);

	if (priv->domain)
		vunmap(ramin_ptr);
}

void
gk20a_instobj_unmap_sg(struct nvkm_object *object, u32 pte, u32 cnt)
{
	struct gk20a_instmem_priv *priv = (void *)nvkm_instmem(object);
	unsigned long flags;
	u32 *ramin_ptr;

	if (!priv->domain) {
		struct gk20a_instobj_dma *node_dma = (void*)object;
		ramin_ptr = node_dma->cpuaddr;
	} else {
		struct gk20a_instobj_iommu *node_iommu = (void*)object;
		int num_ramin_pages, first_ramin_page, last_ramin_page;

		first_ramin_page = pte / 512;
		last_ramin_page = (cnt + pte - 1) / 512;
		num_ramin_pages = last_ramin_page - first_ramin_page + 1;

		ramin_ptr = vmap(&node_iommu->pages[first_ramin_page],
				 num_ramin_pages, VM_MAP,
				 pgprot_writecombine(PAGE_KERNEL));
		pte -= first_ramin_page * 512;
	}

	spin_lock_irqsave(&priv->lock, flags);

	/* Write posting -- ensure that previous mmio has completed */
	nv_rd32(priv, 0x700000);

	pte <<= 1;
	while (cnt--) {
		ramin_ptr[pte] = 0x00000000;
		ramin_ptr[pte + 1] = 0x00000000;
		pte += 2;
	}

	spin_unlock_irqrestore(&priv->lock, flags);

	if (priv->domain)
		vunmap(ramin_ptr);
}

static void
gk20a_instobj_wr32(struct nvkm_object *object, u64 offset, u32 data)
{
	struct gk20a_instmem_priv *priv = (void *)nvkm_instmem(object);
	struct gk20a_instobj_priv *node = (void *)object;
	unsigned long flags;
	u64 base = (node->mem->offset + offset) & 0xffffff00000ULL;
	u64 addr = (node->mem->offset + offset) & 0x000000fffffULL;

	spin_lock_irqsave(&priv->lock, flags);
	if (unlikely(priv->addr != base)) {
		nv_wr32(priv, 0x001700, base >> 16);
		priv->addr = base;
	}
	nv_wr32(priv, 0x700000 + addr, data);
	spin_unlock_irqrestore(&priv->lock, flags);
}

static void
gk20a_instobj_dtor_dma(struct gk20a_instobj_priv *_node)
{
	struct gk20a_instobj_dma *node = (void *)_node;
	struct gk20a_instmem_priv *priv = (void *)nvkm_instmem(node);
	struct device *dev = nv_device_base(nv_device(priv));

	if (unlikely(!node->cpuaddr))
		return;

	dma_free_attrs(dev, _node->mem->size << PAGE_SHIFT, node->cpuaddr,
		       node->handle, &priv->attrs);
}

static void
gk20a_instobj_dtor_iommu(struct gk20a_instobj_priv *_node)
{
	struct gk20a_instobj_iommu *node = (void *)_node;
	struct gk20a_instmem_priv *priv = (void *)nvkm_instmem(node);
	struct device *dev = nv_device_base(nv_device(priv));
	struct nvkm_mm_node *r;
	int i;

	if (unlikely(list_empty(&_node->mem->regions)))
		return;

	r = list_first_entry(&_node->mem->regions, struct nvkm_mm_node,
			     rl_entry);

	/* clear IOMMU translation bit to unmap pages */
	r->offset &= ~BIT(priv->iommu_addr_bit - priv->iommu_pgshift);

	/* Unmap pages from GPU address space and free them */
	for (i = 0; i < _node->mem->size; i++) {
		iommu_unmap(priv->domain,
			    (r->offset + i) << priv->iommu_pgshift, PAGE_SIZE);
		dma_unmap_page(dev, node->page_addrs[i], PAGE_SIZE,
				DMA_BIDIRECTIONAL);
		__free_page(node->pages[i]);
	}

	/* Release area from GPU address space */
	mutex_lock(priv->mm_mutex);
	nvkm_mm_free(priv->mm, &r);
	mutex_unlock(priv->mm_mutex);
}

static void
gk20a_instobj_dtor(struct nvkm_object *object)
{
	struct gk20a_instobj_priv *node = (void *)object;
	struct gk20a_instmem_priv *priv = (void *)nvkm_instmem(node);

	if (priv->domain)
		gk20a_instobj_dtor_iommu(node);
	else
		gk20a_instobj_dtor_dma(node);

	nvkm_instobj_destroy(&node->base);
}

static int
gk20a_instobj_ctor_dma(struct nvkm_object *parent, struct nvkm_object *engine,
		       struct nvkm_oclass *oclass, u32 npages, u32 align,
		       struct gk20a_instobj_priv **_node)
{
	struct gk20a_instobj_dma *node;
	struct gk20a_instmem_priv *priv = (void *)nvkm_instmem(parent);
	struct device *dev = nv_device_base(nv_device(parent));
	int ret;

	ret = nvkm_instobj_create_(parent, engine, oclass, sizeof(*node),
				   (void **)&node);
	*_node = &node->base;
	if (ret)
		return ret;

	node->cpuaddr = dma_alloc_attrs(dev, npages << PAGE_SHIFT,
					&node->handle, GFP_KERNEL,
					&priv->attrs);
	if (!node->cpuaddr) {
		nv_error(priv, "cannot allocate DMA memory\n");
		return -ENOMEM;
	}

	/* alignment check */
	if (unlikely(node->handle & (align - 1)))
		nv_warn(priv, "memory not aligned as requested: %pad (0x%x)\n",
			&node->handle, align);

	/* present memory for being mapped using small pages */
	node->r.type = 12;
	node->r.offset = node->handle >> 12;
	node->r.length = (npages << PAGE_SHIFT) >> 12;

	node->base._mem.offset = node->handle;

	INIT_LIST_HEAD(&node->base._mem.regions);
	list_add_tail(&node->r.rl_entry, &node->base._mem.regions);

	return 0;
}

static int
gk20a_instobj_ctor_iommu(struct nvkm_object *parent, struct nvkm_object *engine,
			 struct nvkm_oclass *oclass, u32 npages, u32 align,
			 struct gk20a_instobj_priv **_node)
{
	struct gk20a_instobj_iommu *node;
	struct gk20a_instmem_priv *priv = (void *)nvkm_instmem(parent);
	struct device *dev = nv_device_base(nv_device(parent));
	struct nvkm_mm_node *r;
	int ret;
	int i;

	ret = nvkm_instobj_create_(parent, engine, oclass, sizeof(*node)
				+ sizeof(node->pages[0]) * npages
				+ sizeof(node->page_addrs[0]) * npages,
				(void **)&node);
	*_node = &node->base;
	if (ret)
		return ret;

	node->pages = (struct page **)&node[1];
	node->page_addrs = (dma_addr_t *)&node->pages[npages];

	/* Allocate backing memory */
	for (i = 0; i < npages; i++) {
		struct page *p = alloc_page(GFP_KERNEL);
		dma_addr_t addr;

		if (p == NULL) {
			ret = -ENOMEM;
			goto free_pages;
		}

		addr = dma_map_page(dev, p, 0, PAGE_SIZE, DMA_BIDIRECTIONAL);
		if (dma_mapping_error(dev, addr)) {
			__free_page(p);
			goto free_pages;
		}

		node->pages[i] = p;
		node->page_addrs[i] = addr;
	}

	mutex_lock(priv->mm_mutex);
	/* Reserve area from GPU address space */
	ret = nvkm_mm_head(priv->mm, 0, 1, npages, npages,
			   align >> priv->iommu_pgshift, &r);
	mutex_unlock(priv->mm_mutex);
	if (ret) {
		nv_error(priv, "virtual space is full!\n");
		goto free_pages;
	}

	/* Map into GPU address space */
	for (i = 0; i < npages; i++) {
		u32 offset = (r->offset + i) << priv->iommu_pgshift;

		ret = iommu_map(priv->domain, offset, node->page_addrs[i],
				PAGE_SIZE, IOMMU_READ | IOMMU_WRITE);
		if (ret < 0) {
			nv_error(priv, "IOMMU mapping failure: %d\n", ret);

			while (i-- > 0) {
				offset -= PAGE_SIZE;
				iommu_unmap(priv->domain, offset, PAGE_SIZE);
			}
			goto release_area;
		}
	}

	/*
	 * The iommu_addr_bit tells that an address is to be resolved through
	 * the IOMMU
	 */
	r->offset |= BIT(priv->iommu_addr_bit - priv->iommu_pgshift);

	node->base._mem.offset = ((u64)r->offset) << priv->iommu_pgshift;

	INIT_LIST_HEAD(&node->base._mem.regions);
	list_add_tail(&r->rl_entry, &node->base._mem.regions);

	return 0;

release_area:
	mutex_lock(priv->mm_mutex);
	nvkm_mm_free(priv->mm, &r);
	mutex_unlock(priv->mm_mutex);

free_pages:
	for (i = 0; i < npages && node->pages[i] != NULL; i++) {
		dma_unmap_page(dev, node->page_addrs[i], PAGE_SIZE,
				DMA_BIDIRECTIONAL);
		__free_page(node->pages[i]);
	}

	return ret;
}

static int
gk20a_instobj_ctor(struct nvkm_object *parent, struct nvkm_object *engine,
		   struct nvkm_oclass *oclass, void *data, u32 _size,
		   struct nvkm_object **pobject)
{
	struct nvkm_instobj_args *args = data;
	struct gk20a_instmem_priv *priv = (void *)nvkm_instmem(parent);
	struct gk20a_instobj_priv *node;
	u32 size, align;
	int ret;

	nv_debug(parent, "%s (%s): size: %x align: %x\n", __func__,
		 priv->domain ? "IOMMU" : "DMA", args->size, args->align);

	/* Round size and align to page bounds */
	size = max(roundup(args->size, PAGE_SIZE), PAGE_SIZE);
	align = max(roundup(args->align, PAGE_SIZE), PAGE_SIZE);

	if (priv->domain)
		ret = gk20a_instobj_ctor_iommu(parent, engine, oclass,
					      size >> PAGE_SHIFT, align, &node);
	else
		ret = gk20a_instobj_ctor_dma(parent, engine, oclass,
					     size >> PAGE_SHIFT, align, &node);
	*pobject = nv_object(node);
	if (ret)
		return ret;

	node->mem = &node->_mem;

	/* present memory for being mapped using small pages */
	node->mem->size = size >> 12;
	node->mem->memtype = 0;
	node->mem->page_shift = 12;

	node->base.addr = node->mem->offset;
	node->base.size = size;

	nv_debug(parent, "alloc size: 0x%x, align: 0x%x, gaddr: 0x%llx\n",
		 size, align, node->mem->offset);

	return 0;
}

static struct nvkm_instobj_impl
gk20a_instobj_oclass = {
	.base.ofuncs = &(struct nvkm_ofuncs) {
		.ctor = gk20a_instobj_ctor,
		.dtor = gk20a_instobj_dtor,
		.init = _nvkm_instobj_init,
		.fini = _nvkm_instobj_fini,
		.rd32 = gk20a_instobj_rd32,
		.wr32 = gk20a_instobj_wr32,
	},
};

static int
gk20a_instmem_init(struct nvkm_object *object)
{
	struct gk20a_instmem_priv *priv = (void *)object;
	return nvkm_subdev_init(&priv->base.base);
}


static int
gk20a_instmem_fini(struct nvkm_object *object, bool suspend)
{
	struct gk20a_instmem_priv *priv = (void *)object;
	priv->addr = ~0ULL;
	return nvkm_subdev_fini(&priv->base.base, suspend);
}

static int
gk20a_instmem_ctor(struct nvkm_object *parent, struct nvkm_object *engine,
		   struct nvkm_oclass *oclass, void *data, u32 size,
		   struct nvkm_object **pobject)
{
	struct gk20a_instmem_priv *priv;
	struct nouveau_platform_device *plat;
	int ret;

	ret = nvkm_instmem_create(parent, engine, oclass, &priv);
	*pobject = nv_object(priv);
	if (ret)
		return ret;

	spin_lock_init(&priv->lock);

	plat = nv_device_to_platform(nv_device(parent));
	if (plat->gpu->iommu.domain) {
		priv->domain = plat->gpu->iommu.domain;
		priv->mm = plat->gpu->iommu.mm;
		priv->iommu_pgshift = plat->gpu->iommu.pgshift;
		priv->iommu_addr_bit = plat->gpu->iommu.translation_enable_bit;
		priv->mm_mutex = &plat->gpu->iommu.mutex;

		nv_info(priv, "using IOMMU\n");
	} else {
		init_dma_attrs(&priv->attrs);
		/*
		 * We will access instmem through PRAMIN and thus do not need a
		 * consistent CPU pointer or kernel mapping
		 */
		dma_set_attr(DMA_ATTR_NON_CONSISTENT, &priv->attrs);
		dma_set_attr(DMA_ATTR_WEAK_ORDERING, &priv->attrs);
		dma_set_attr(DMA_ATTR_WRITE_COMBINE, &priv->attrs);

		nv_info(priv, "using DMA API\n");
	}

	return 0;
}

struct nvkm_oclass *
gk20a_instmem_oclass = &(struct nvkm_instmem_impl) {
	.base.handle = NV_SUBDEV(INSTMEM, 0xea),
	.base.ofuncs = &(struct nvkm_ofuncs) {
		.ctor = gk20a_instmem_ctor,
		.dtor = _nvkm_instmem_dtor,
		.init = gk20a_instmem_init,
		.fini = gk20a_instmem_fini,
	},
	.instobj = &gk20a_instobj_oclass.base,
}.base;
