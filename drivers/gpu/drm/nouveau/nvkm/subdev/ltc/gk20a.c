/*
 * Copyright (C) 2015 Google, Inc.
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
 * THE COPYRIGHT HOLDER(S) OR AUTHOR(S) BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#include <subdev/timer.h>

#ifdef __KERNEL__
#include <linux/iommu.h>
#include <nouveau_platform.h>
#endif

#include "priv.h"

struct gk20a_ltc_priv {
	struct nvkm_ltc_priv priv;
	struct page **pages;
	dma_addr_t *page_addrs;
	int npages;
};

/* Algorithm from NVIDIA nvgpu driver */
static int
gk20a_ltc_calc_tags_size(struct gk20a_ltc_priv *priv,
		u32 *compbit_backing_size, u32 *max_comptag_lines)
{
	u64 max_size;
	u32 hw_max_comptag_lines = nv_rd32(priv, 0x17e8d0) & 0x1ffff;
	u32 num_fbps = nv_rd32(priv, 0x120074) & 0x1f;
	u32 cbc_param = nv_rd32(priv, 0x17e8dc);
	u32 comptags_per_cacheline = cbc_param & 0xffff;
	u32 slices_per_fbp = (cbc_param >> 28) & 0xf;
	u32 cacheline_size = 512 << ((cbc_param >> 24) & 0xf);

	max_size = totalram_pages << PAGE_SHIFT;
	/* one tag line covers 128KB */
	*max_comptag_lines = max_size >> 17;

	if (*max_comptag_lines == 0)
		return -EINVAL;

	if (*max_comptag_lines > hw_max_comptag_lines)
		*max_comptag_lines = hw_max_comptag_lines;

	/* no hyrid fb */
	*compbit_backing_size =
		DIV_ROUND_UP(*max_comptag_lines, comptags_per_cacheline) *
		cacheline_size * slices_per_fbp * num_fbps;

	/* aligned to 2KB * num_fbps */
	*compbit_backing_size += num_fbps << 11;

	/* must be a multiple of 64KB */
	*compbit_backing_size = roundup(*compbit_backing_size, 64 * 1024);

	*max_comptag_lines =
		(*compbit_backing_size * comptags_per_cacheline) /
		cacheline_size * slices_per_fbp * num_fbps;

	if (*max_comptag_lines > hw_max_comptag_lines)
		*max_comptag_lines = hw_max_comptag_lines;

	return 0;
}

static int
gk20a_ltc_init_tag_mem(struct gk20a_ltc_priv *priv)
{
	struct nouveau_platform_device *plat;
	struct nvkm_ltc_priv *ltc_priv = &priv->priv;
	struct device *dev = nv_device_base(nv_device(priv));
	u32 compbit_backing_size, max_comptag_lines;
	int i, ret, npages;

	plat = nv_device_to_platform(nv_device(&ltc_priv->base));
	if (!plat->gpu->iommu.domain) {
		nv_warn(priv, "can't init comptags due to lack of IOMMU support\n");
		return 0;
	}

	ret = gk20a_ltc_calc_tags_size(priv, &compbit_backing_size,
						&max_comptag_lines);
	if (ret)
		return 0;

	npages = DIV_ROUND_UP(compbit_backing_size, PAGE_SIZE);
	mutex_lock(&plat->gpu->iommu.mutex);
	ret = nvkm_mm_head(plat->gpu->iommu.mm, 1, 1,
						npages, npages, 1, &ltc_priv->tag_ram);
	mutex_unlock(&plat->gpu->iommu.mutex);
	if (ret) {
		nv_error(priv, "failed to get comptags VM from IOMMU space\n");
		return ret;
	}

	ret = nvkm_mm_init(&ltc_priv->tags, 0, max_comptag_lines, 1);
	if (ret) {
		nv_error(priv, "failed to init comptags virtual memory space\n");
		goto err_comptags_vm;
	}

	priv->pages = kzalloc(sizeof(struct page *) * npages, GFP_KERNEL);
	if (!priv->pages) {
		nv_error(priv, "failed to allocate for page pointers\n");
		ret = -ENOMEM;
		goto err_pages_alloc1;
	}

	priv->page_addrs = kzalloc(sizeof(dma_addr_t) * npages, GFP_KERNEL);
	if (!priv->page_addrs) {
		nv_error(priv, "failed to allocate for page addresses\n");
		ret = -ENOMEM;
		goto err_pages_alloc2;
	}

	for (i = 0; i < npages; i++) {
		struct page *p;
		dma_addr_t addr;

		p = alloc_page(GFP_KERNEL);
		if (!p) {
			nv_error(priv, "failed to allocate pages for comptags\n");
			ret = -ENOMEM;
			goto err_pages_alloc3;
		}

		addr = dma_map_page(dev, p, 0, PAGE_SIZE, DMA_BIDIRECTIONAL);
		if (dma_mapping_error(dev, addr)) {
			__free_page(p);
			goto err_pages_alloc3;
		}

		priv->pages[i] = p;
		priv->page_addrs[i] = addr;
	}
	priv->npages = npages;

	mutex_lock(&plat->gpu->iommu.mutex);
	for (i = 0; i < npages; i++) {
		struct page *p = priv->pages[i];
		unsigned long iova = (ltc_priv->tag_ram->offset + i) << PAGE_SHIFT;

		ret = iommu_map(plat->gpu->iommu.domain,
					iova,
					page_to_phys(p),
					PAGE_SIZE,
					IOMMU_READ | IOMMU_WRITE);
		if (ret) {
			nv_error(priv, "failed to map pages to IOMMU\n");

			while (i-- > 0) {
				iova -= PAGE_SIZE;
				iommu_unmap(plat->gpu->iommu.domain, iova, PAGE_SIZE);
			}
			mutex_unlock(&plat->gpu->iommu.mutex);
			goto err_pages_alloc3;
		}
	}
	mutex_unlock(&plat->gpu->iommu.mutex);

	ltc_priv->num_tags = max_comptag_lines;
	ltc_priv->tag_base = (ltc_priv->tag_ram->offset |
		BIT_ULL(plat->gpu->iommu.translation_enable_bit - 12)) <<(12 - 11);

	nv_info(priv, "compbit backing store size: %u\n", npages << PAGE_SHIFT);
	nv_info(priv, "tags number: %u\n", ltc_priv->num_tags);
	nv_info(priv, "tags base: 0x%08x\n", ltc_priv->tag_base);

	return 0;

err_pages_alloc3:
	for (i = 0; i < npages && priv->pages[i] != NULL; i++) {
		dma_unmap_page(dev, priv->page_addrs[i], PAGE_SIZE,
				DMA_BIDIRECTIONAL);
		__free_page(priv->pages[i]);
	}
	kfree(priv->page_addrs);
err_pages_alloc2:
	kfree(priv->pages);
err_pages_alloc1:
	nvkm_mm_fini(&ltc_priv->tags);

err_comptags_vm:
	mutex_lock(&plat->gpu->iommu.mutex);
	nvkm_mm_free(plat->gpu->iommu.mm, &ltc_priv->tag_ram);
	mutex_unlock(&plat->gpu->iommu.mutex);

	return ret;
}

static void
gk20a_ltc_fini_tag_mem(struct gk20a_ltc_priv *priv)
{
	struct nouveau_platform_device *plat;
	struct device *dev = nv_device_base(nv_device(priv));
	int i;

	plat = nv_device_to_platform(nv_device(&priv->priv.base));
	if (!plat->gpu->iommu.domain)
		return;

	iommu_unmap(plat->gpu->iommu.domain,
				priv->priv.tag_ram->offset << PAGE_SHIFT,
				PAGE_SIZE * priv->npages);

	for (i = 0; i < priv->npages && priv->pages[i] != NULL; i++) {
		dma_unmap_page(dev, priv->page_addrs[i], PAGE_SIZE,
				DMA_BIDIRECTIONAL);
		__free_page(priv->pages[i]);
	}

	nvkm_mm_fini(&priv->priv.tags);

	mutex_lock(&plat->gpu->iommu.mutex);
	nvkm_mm_free(plat->gpu->iommu.mm, &priv->priv.tag_ram);
	mutex_unlock(&plat->gpu->iommu.mutex);

	kfree(priv->page_addrs);
	kfree(priv->pages);
}

void
gk20a_ltc_invalidate(struct nvkm_ltc_priv *ltc)
{
	spin_lock(&ltc->maint_op_lock);

	nv_wr32(ltc, 0x70004, 0x00000001);
	if (!nv_wait(ltc, 0x70004, 0x00000003, 0x00000000))
		nv_warn(ltc, "L2 invalidate timeout\n");

	spin_unlock(&ltc->maint_op_lock);
}

void
gk20a_ltc_flush(struct nvkm_ltc_priv *ltc)
{
	spin_lock(&ltc->maint_op_lock);

	nv_wr32(ltc, 0x70010, 0x00000001);
	if (!nv_wait(ltc, 0x70010, 0x00000003, 0x00000000))
		nv_warn(ltc, "L2 flush timeout\n");

	spin_unlock(&ltc->maint_op_lock);
}

static void
gk20a_ltc_init_cbc(struct nvkm_ltc_priv *priv)
{
	int c, s;

	/* invalidate all compbits */
	nv_wr32(priv, 0x17e8c8, 0x2);

	for (c = 0; c < priv->ltc_nr; c++) {
		for (s = 0; s < priv->lts_nr; s++)
			if (!nv_wait(priv, 0x1410c8 + c * 0x2000 + s * 0x400, 0x2, 0))
				nv_warn(priv, "CBC invalidate timeout, ltc=%d, lts=%d\n",
						c, s);
	}
}

static int
gk20a_ltc_init(struct nvkm_object *object)
{
	struct nvkm_ltc_priv *priv = (void *)object;
	int ret;

	ret = gk104_ltc_init(object);
	if (ret)
		return ret;

	gk20a_ltc_init_cbc(priv);

	return 0;
}

static int
gk20a_ltc_fini(struct nvkm_object *object, bool suspend)
{
	struct nvkm_ltc_priv *priv = (void *)object;

	/* flush compression bit */
	nv_wr32(priv, 0x17e828, 0x00000001);

	if (!nv_wait(priv, 0x140828, 0x1, 0x0))
		nv_warn(priv, "compression bit flush timeout\n");

	return nvkm_ltc_fini(priv, suspend);
}

static int
gk20a_ltc_ctor(struct nvkm_object *parent, struct nvkm_object *engine,
	       struct nvkm_oclass *oclass, void *data, u32 size,
	       struct nvkm_object **pobject)
{
	struct gk20a_ltc_priv *priv;
	struct nvkm_ltc_priv *ltc_priv;
	int ret;

	ret = nvkm_ltc_create(parent, engine, oclass, &priv);
	*pobject = nv_object(priv);
	if (ret)
		return ret;

	ltc_priv = &priv->priv;
	ltc_priv->ltc_nr = 1;
	ltc_priv->lts_nr = nv_rd32(priv, 0x17e8dc) >> 28;

	ret = gk20a_ltc_init_tag_mem(priv);
	if (ret)
		return ret;

	return 0;
}

static void
gk20a_ltc_dtor(struct nvkm_object *object)
{
	struct gk20a_ltc_priv *priv = (void *)object;

	gk20a_ltc_fini_tag_mem(priv);
	nvkm_ltc_destroy(&priv->priv);
}

struct nvkm_oclass *
gk20a_ltc_oclass = &(struct nvkm_ltc_impl) {
	.base.handle = NV_SUBDEV(LTC, 0xea),
	.base.ofuncs = &(struct nvkm_ofuncs) {
		.ctor = gk20a_ltc_ctor,
		.dtor = gk20a_ltc_dtor,
		.init = gk20a_ltc_init,
		.fini = gk20a_ltc_fini,
	},
	.intr = gf100_ltc_intr,
	.cbc_clear = gf100_ltc_cbc_clear,
	.cbc_wait = gf100_ltc_cbc_wait,
	.zbc = 16,
	.zbc_clear_color = gf100_ltc_zbc_clear_color,
	.zbc_clear_depth = gf100_ltc_zbc_clear_depth,
	.invalidate = gk20a_ltc_invalidate,
	.flush = gk20a_ltc_flush,
}.base;
