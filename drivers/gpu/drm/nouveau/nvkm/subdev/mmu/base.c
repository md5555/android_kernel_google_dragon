/*
 * Copyright 2010 Red Hat Inc.
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
 *
 * Authors: Ben Skeggs
 */

#include <subdev/bar.h>
#include <subdev/ltc.h>
#include <subdev/mmu.h>
#include <subdev/fb.h>

#include <core/gpuobj.h>

#ifdef CONFIG_SYNC
#include <sync.h>
#endif

enum page_size_index {
	SMALL_PAGE_INDEX = 0,
	BIG_PAGE_INDEX = 1
};

static void
nvkm_vm_map_at(struct nvkm_vma *vma, u64 delta, struct nvkm_mem *node)
{
	struct nvkm_vm *vm = vma->vm;
	struct nvkm_mmu *mmu = vm->mmu;
	struct nvkm_ltc *ltc = nvkm_ltc(mmu);
	struct nvkm_mm_node *r;
	int big = vma->node->type != mmu->spg_shift;
	u32 offset = vma->node->offset;
	u32 bits = vma->node->type - 12;
	u32 pde  = (offset >> mmu->pgt_bits) - vm->fpde;
	u32 pte  = (offset & ((1 << mmu->pgt_bits) - 1)) >> bits;
	u32 max  = 1 << (mmu->pgt_bits - bits);
	u32 end, len;

	list_for_each_entry(r, &node->regions, rl_entry) {
		u64 phys = (u64)r->offset << 12;
		u32 num  = r->length >> bits;

		while (num) {
			struct nvkm_gpuobj *pgt = vm->pgt[pde].obj[big];

			end = (pte + num);
			if (unlikely(end >= max))
				end = max;
			len = end - pte;

			mmu->map(vma, pgt, node, pte, len, phys, delta);

			num -= len;
			pte += len;
			if (unlikely(end >= max)) {
				phys += len << (bits + 12);
				pde++;
				pte = 0;
			}

			delta += (u64)len << vma->node->type;
		}
	}

	ltc->invalidate(ltc);
	mmu->flush(vm);
}

static void
nvkm_vm_map_sg_table(struct nvkm_vma *vma, u64 delta, u64 length,
		     struct nvkm_mem *mem)
{
	struct nvkm_vm *vm = vma->vm;
	struct nvkm_mmu *mmu = vm->mmu;
	struct nvkm_ltc *ltc = nvkm_ltc(mmu);
	int big = vma->node->type != mmu->spg_shift;
	u32 offset = vma->node->offset;
	u32 bits = vma->node->type - 12;
	u32 num  = length >> vma->node->type;
	u32 pde  = (offset >> mmu->pgt_bits) - vm->fpde;
	u32 pte  = (offset & ((1 << mmu->pgt_bits) - 1)) >> bits;
	u32 max  = 1 << (mmu->pgt_bits - bits);
	unsigned m;
	u32 end, len;
	struct scatterlist *sg;
	u64 skip;

	/*
	 * Skip "delta" bytes into the sgl. "skip" is left with the offset
	 * into the first sg element we want to start with.
	 */
	sg = mem->sg->sgl;
	skip = delta;
	while (skip >= sg_dma_len(sg)) {
		skip -= sg_dma_len(sg);
		sg = sg_next(sg);
	}

	while (sg) {
		dma_addr_t *addr_list;
		struct nvkm_gpuobj *pgt = vm->pgt[pde].obj[big];
		dma_addr_t sgdma = sg_dma_address(sg) + skip;
		unsigned sglen = (sg_dma_len(sg) - skip) >> PAGE_SHIFT;

		skip = 0;

next_pde:
		end = pte + sglen;
		if (unlikely(end >= max))
			end = max;
		len = end - pte;
		if (unlikely(len > num))
			len = num;

		addr_list = kmalloc(sizeof(dma_addr_t) * len, GFP_KERNEL);
		if (WARN_ON(!addr_list))
			return;

		for (m = 0; m < len; m++)
			addr_list[m] = sgdma + (m << PAGE_SHIFT);

		mmu->map_sg(vma, pgt, mem, pte, len, addr_list, delta);
		kfree(addr_list);
		pte += m;
		num -= m;

		if (num == 0)
			goto finish;

		if (unlikely(end >= max)) {
			pde++;
			pte = 0;
		}

		delta += (u64)len << vma->node->type;

		if (m < sglen) {
			sglen -= m;
			sgdma += m << PAGE_SHIFT;
			pgt = vm->pgt[pde].obj[big];
			goto next_pde;
		}

		sg = sg_next(sg);
	}
finish:
	ltc->invalidate(ltc);
	mmu->flush(vm);
}

static void
nvkm_vm_map_sg_table_with_iommu(struct nvkm_vma *vma, u64 delta, u64 length,
		     struct nvkm_mem *mem)
{
	struct nvkm_vm *vm = vma->vm;
	struct nvkm_mmu *mmu = vm->mmu;
	struct nvkm_ltc *ltc = nvkm_ltc(mmu);
	u32 offset = vma->node->offset;
	u32 bits = vma->node->type - 12;
	u32 num  = length >> vma->node->type;
	u32 max  = 1 << (mmu->pgt_bits - bits);
	u32 lpidx;
	u64 iova;
	dma_addr_t *addr_list, *list;

	if (likely(!vma->iommu_mapping)) {
		vma->iommu_mapping = mmu->map_sg_iommu(mmu, mem->sg,
						       mem->size << 12, &iova);
		if (IS_ERR(vma->iommu_mapping)) {
			vma->iommu_mapping = NULL;
			return;
		}
		vma->iommu_iova = iova;
	} else {
		iova = vma->iommu_iova;
	}

	addr_list = vmalloc(sizeof(dma_addr_t) * num);
	if (!addr_list) {
		mmu->unmap_iommu(vma, vma->iommu_mapping);
		nv_error(mmu, "no memory for dma list\n");
		return;
	}

	for (lpidx = 0; lpidx < num; lpidx++)
		addr_list[lpidx] = iova + (lpidx << mmu->lpg_shift) + delta;

	lpidx = 0;
	list = addr_list;
	while (num) {
		struct nvkm_gpuobj *pgt;
		u32 lpoff, pde, pte, end, len;

		lpoff = offset + lpidx * (1 << (mmu->lpg_shift - mmu->spg_shift));
		pde = (lpoff >> (mmu->pgt_bits)) - vm->fpde;
		pte = (lpoff & ((1 << mmu->pgt_bits) - 1)) >> bits;
		pgt = vm->pgt[pde].obj[BIG_PAGE_INDEX];

		end = (pte + num);
		if (unlikely(end > max))
			end = max;
		len = end - pte;

		mmu->map_sg(vma, pgt, mem, pte, len, list, delta);

		num  -= len;
		list += len;
		lpidx += len;
		delta += len << mmu->lpg_shift;
	}

	vfree(addr_list);

	ltc->invalidate(ltc);
	mmu->flush(vm);
}

static void
nvkm_vm_map_sg(struct nvkm_vma *vma, u64 delta, u64 length,
	       struct nvkm_mem *mem)
{
	struct nvkm_vm *vm = vma->vm;
	struct nvkm_mmu *mmu = vm->mmu;
	dma_addr_t *list = mem->pages;
	struct nvkm_ltc *ltc = nvkm_ltc(mmu);
	int big = vma->node->type != mmu->spg_shift;
	u32 offset = vma->node->offset;
	u32 bits = vma->node->type - 12;
	u32 num  = length >> vma->node->type;
	u32 pde  = (offset >> mmu->pgt_bits) - vm->fpde;
	u32 pte  = (offset & ((1 << mmu->pgt_bits) - 1)) >> bits;
	u32 max  = 1 << (mmu->pgt_bits - bits);
	u32 end, len;

	while (num) {
		struct nvkm_gpuobj *pgt = vm->pgt[pde].obj[big];

		end = (pte + num);
		if (unlikely(end >= max))
			end = max;
		len = end - pte;

		mmu->map_sg(vma, pgt, mem, pte, len, list, delta);

		num  -= len;
		pte  += len;
		list += len;
		if (unlikely(end >= max)) {
			pde++;
			pte = 0;
		}

		delta += (u64)len << vma->node->type;
	}

	ltc->invalidate(ltc);
	mmu->flush(vm);
}

void
nvkm_vm_map(struct nvkm_vma *vma, struct nvkm_mem *node)
{
	struct nvkm_vm *vm = vma->vm;
	struct nvkm_mmu *mmu = vm->mmu;

	if (node->sg) {
		if (mmu->iommu_capable && vma->node->type == mmu->lpg_shift)
			nvkm_vm_map_sg_table_with_iommu(vma, vma->delta,
							vma->length, node);
		else
			nvkm_vm_map_sg_table(vma, vma->delta, vma->length,
					     node);
	} else if (node->pages) {
			nvkm_vm_map_sg(vma, vma->delta, vma->length, node);
	} else {
		nvkm_vm_map_at(vma, vma->delta, node);
	}

	vma->mapped = true;
}

static void
nvkm_vm_unmap_at(struct nvkm_vma *vma, u64 length)
{
	struct nvkm_vm *vm = vma->vm;
	struct nvkm_mmu *mmu = vm->mmu;
	struct nvkm_ltc *ltc = nvkm_ltc(mmu);
	int big = vma->node->type != mmu->spg_shift;
	u32 offset = vma->node->offset;
	u32 bits = vma->node->type - 12;
	u32 num  = length >> vma->node->type;
	u32 pde  = (offset >> mmu->pgt_bits) - vm->fpde;
	u32 pte  = (offset & ((1 << mmu->pgt_bits) - 1)) >> bits;
	u32 max  = 1 << (mmu->pgt_bits - bits);
	u32 end, len;

	while (num) {
		struct nvkm_gpuobj *pgt = vm->pgt[pde].obj[big];

		end = (pte + num);
		if (unlikely(end >= max))
			end = max;
		len = end - pte;

		if (unlikely(vma->as && vma->as->sparse))
			mmu->sparse(pgt, pte, len);
		else
			mmu->unmap(pgt, pte, len);

		num -= len;
		pte += len;
		if (unlikely(end >= max)) {
			pde++;
			pte = 0;
		}
	}

	/*
	 * Make sure that modified PTEs are not in GPU cache, so that
	 * after GMMU TLB flush, new values can be loaded into TLB.
	 */
	ltc->invalidate(ltc);
	mmu->flush(vm);
	/*
	 * Make sure that GPU cache does not contain any dirty lines
	 * related to the buffer being unmapped. Otherwise we could
	 * end up with write-back triggered already after freeing the
	 * memory to the system (or unmapping from IOMMU if it is used).
	 */
	ltc->flush(ltc);
}

static void
nvkm_vm_unmap_iommu(struct nvkm_vma *vma)
{
	struct nvkm_mmu *mmu = vma->vm->mmu;

	mmu->unmap_iommu(vma, vma->iommu_mapping);
}

void
nvkm_vm_unmap(struct nvkm_vma *vma)
{
	nvkm_vm_unmap_at(vma, (u64)vma->node->length << 12);

	if (vma->iommu_mapping)
		nvkm_vm_unmap_iommu(vma);

	vma->mapped = false;
}

static void
nvkm_vm_unmap_pgt(struct nvkm_vm *vm, int big, u32 fpde, u32 lpde)
{
	unsigned int num_unref_pgts = 0;
	struct nvkm_gpuobj **unref_pgts;
	struct nvkm_mmu *mmu = vm->mmu;
	struct nvkm_ltc *ltc = nvkm_ltc(mmu);
	struct nvkm_vm_pgd *vpgd;
	struct nvkm_vm_pgt *vpgt;
	struct nvkm_gpuobj *pgt;
	u32 pde;

	unref_pgts = kcalloc(lpde - fpde + 1, sizeof(*unref_pgts),
				GFP_KERNEL | __GFP_NOFAIL);

	for (pde = fpde; pde <= lpde; pde++) {
		vpgt = &vm->pgt[pde - vm->fpde];
		if (--vpgt->refcount[big])
			continue;

		pgt = vpgt->obj[big];
		vpgt->obj[big] = NULL;

		list_for_each_entry(vpgd, &vm->pgd_list, head) {
			mmu->map_pgt(vpgd->obj, pde, vpgt->obj);
		}

		unref_pgts[num_unref_pgts++] = pgt;
	}

	if (num_unref_pgts) {
		int i;

		mutex_unlock(&nv_subdev(mmu)->mutex);

		ltc->invalidate(ltc);
		mmu->flush(vm);

		for (i = 0; i < num_unref_pgts; ++i)
			nvkm_gpuobj_ref(NULL, &unref_pgts[i]);

		mutex_lock(&nv_subdev(mmu)->mutex);
	}

	kfree(unref_pgts);
}

static int
nvkm_vm_map_pgt(struct nvkm_vm *vm, u32 pde, u32 type)
{
	struct nvkm_mmu *mmu = vm->mmu;
	struct nvkm_ltc *ltc = nvkm_ltc(mmu);
	struct nvkm_vm_pgt *vpgt = &vm->pgt[pde - vm->fpde];
	struct nvkm_vm_pgd *vpgd;
	struct nvkm_gpuobj *pgt;
	int big = (type != mmu->spg_shift);
	u32 pgt_size;
	int ret;

	pgt_size  = (1 << (mmu->pgt_bits + 12)) >> type;
	pgt_size *= 8;

	mutex_unlock(&nv_subdev(mmu)->mutex);
	ret = nvkm_gpuobj_new(nv_object(vm->mmu), NULL, pgt_size, 0x1000,
			      NVOBJ_FLAG_ZERO_ALLOC, &pgt);
	mutex_lock(&nv_subdev(mmu)->mutex);
	if (unlikely(ret))
		return ret;

	/* someone beat us to filling the PDE while we didn't have the lock */
	if (unlikely(vpgt->refcount[big]++)) {
		mutex_unlock(&nv_subdev(mmu)->mutex);
		nvkm_gpuobj_ref(NULL, &pgt);
		mutex_lock(&nv_subdev(mmu)->mutex);
		return 0;
	}

	vpgt->obj[big] = pgt;
	list_for_each_entry(vpgd, &vm->pgd_list, head) {
		mmu->map_pgt(vpgd->obj, pde, vpgt->obj);
	}

	mutex_unlock(&nv_subdev(mmu)->mutex);
	ltc->invalidate(ltc);
	mmu->flush(vm);
	mutex_lock(&nv_subdev(mmu)->mutex);

	return 0;
}

struct nvkm_as *
nvkm_vm_find_as(struct nvkm_vm *vm, u64 offset)
{
	struct nvkm_as *as;

	list_for_each_entry(as, &vm->as_list, head)
		if ((offset >= as->offset) &&
		    (offset < as->offset + as->length))
			return as;

	return NULL;
}

int
nvkm_vm_get_offset(struct nvkm_vm *vm, u64 delta, u64 size, u32 page_shift,
		   u32 access, struct nvkm_vma *vma, u64 offset)
{
	struct nvkm_mmu *mmu = vm->mmu;
	u32 align = (1 << page_shift) >> 12;
	u32 msize = size >> 12;
	u32 fpde, lpde, pde;
	int ret;

	mutex_lock(&nv_subdev(mmu)->mutex);

	/**
	 * If an offset is supplied, we try to find an address space that
	 * encompasses it.  If we find one, that means someone has previously
	 * allocated some address space, and so we use the allocator for that
	 * address space.  Otherwise, fall back to the main allocator.
	 */
	if (offset) {
		struct nvkm_mm *mm = &vm->mm;
		struct nvkm_as *as;
		u64 moffset = offset >> 12;

		as = nvkm_vm_find_as(vm, offset);
		if (as) {
			mm = &as->mm;
			vma->as = as;
		}

		ret = nvkm_mm_head_offset(mm, 0, page_shift, msize, msize,
					  align, &vma->node, moffset);

	} else {
		ret = nvkm_mm_head(&vm->mm, 0, page_shift, msize, msize, align,
				   &vma->node);
	}
	if (unlikely(ret != 0)) {
		mutex_unlock(&nv_subdev(mmu)->mutex);
		return ret;
	}

	fpde = (vma->node->offset >> mmu->pgt_bits);
	lpde = (vma->node->offset + vma->node->length - 1) >> mmu->pgt_bits;

	for (pde = fpde; pde <= lpde; pde++) {
		struct nvkm_vm_pgt *vpgt = &vm->pgt[pde - vm->fpde];
		int big = (vma->node->type != mmu->spg_shift);

		if (likely(vpgt->refcount[big])) {
			vpgt->refcount[big]++;
			continue;
		}

		ret = nvkm_vm_map_pgt(vm, pde, vma->node->type);
		if (ret) {
			if (pde != fpde)
				nvkm_vm_unmap_pgt(vm, big, fpde, pde - 1);
			nvkm_mm_free(&vm->mm, &vma->node);
			mutex_unlock(&nv_subdev(mmu)->mutex);
			return ret;
		}
	}
	mutex_unlock(&nv_subdev(mmu)->mutex);

	vma->vm = NULL;
	nvkm_vm_ref(vm, &vma->vm, NULL);
	vma->offset = (u64)vma->node->offset << 12;
	vma->access = access;
	vma->delta = delta;
	vma->length = size;
	return 0;
}

int
nvkm_vm_get(struct nvkm_vm *vm, u64 delta, u64 size, u32 page_shift,
	    u32 access, struct nvkm_vma *vma)
{
	return nvkm_vm_get_offset(vm, delta, size, page_shift, access, vma, 0);
}

void
nvkm_vm_put(struct nvkm_vma *vma)
{
	struct nvkm_vm *vm = vma->vm;
	struct nvkm_mmu *mmu = vm->mmu;
	u32 fpde, lpde;

	if (unlikely(vma->node == NULL))
		return;
	fpde = (vma->node->offset >> mmu->pgt_bits);
	lpde = (vma->node->offset + vma->node->length - 1) >> mmu->pgt_bits;

	mutex_lock(&nv_subdev(mmu)->mutex);
	nvkm_vm_unmap_pgt(vm, vma->node->type != mmu->spg_shift, fpde, lpde);
	if (vma->as)
		nvkm_mm_free(&vma->as->mm, &vma->node);
	else
		nvkm_mm_free(&vm->mm, &vma->node);
	mutex_unlock(&nv_subdev(mmu)->mutex);

	nvkm_vm_ref(NULL, &vma->vm, NULL);
}

int
nvkm_vm_create(struct nvkm_mmu *mmu, u64 offset, u64 length, u64 mm_offset,
	       u32 block, struct nvkm_vm **pvm)
{
	struct nvkm_vm *vm;
	u64 mm_length = (offset + length) - mm_offset;
	int ret;

	vm = kzalloc(sizeof(*vm), GFP_KERNEL);
	if (!vm)
		return -ENOMEM;

	INIT_LIST_HEAD(&vm->pgd_list);
	mutex_init(&vm->fence_lock);

	INIT_LIST_HEAD(&vm->dirty_vma_list);
	mutex_init(&vm->dirty_vma_lock);

	INIT_LIST_HEAD(&vm->as_list);

	vm->mmu = mmu;
	kref_init(&vm->refcount);
	vm->fpde = offset >> (mmu->pgt_bits + 12);
	vm->lpde = (offset + length - 1) >> (mmu->pgt_bits + 12);

	vm->pgt  = vzalloc((vm->lpde - vm->fpde + 1) * sizeof(*vm->pgt));
	if (!vm->pgt) {
		kfree(vm);
		return -ENOMEM;
	}

	ret = nvkm_mm_init(&vm->mm, mm_offset >> 12, mm_length >> 12,
			   block >> 12);
	if (ret) {
		vfree(vm->pgt);
		kfree(vm);
		return ret;
	}

	*pvm = vm;

	return 0;
}

int
nvkm_vm_new(struct nvkm_device *device, u64 offset, u64 length, u64 mm_offset,
	    struct nvkm_vm **pvm)
{
	struct nvkm_mmu *mmu = nvkm_mmu(device);
	return mmu->create(mmu, offset, length, mm_offset, pvm);
}

static int
nvkm_vm_link(struct nvkm_vm *vm, struct nvkm_gpuobj *pgd)
{
	struct nvkm_mmu *mmu = vm->mmu;
	struct nvkm_ltc *ltc = nvkm_ltc(mmu);
	struct nvkm_bar *bar = nvkm_bar(mmu);
	struct nvkm_vm_pgd *vpgd;
	int i;

	if (!pgd)
		return 0;

	vpgd = kzalloc(sizeof(*vpgd), GFP_KERNEL);
	if (!vpgd)
		return -ENOMEM;

	nvkm_gpuobj_ref(pgd, &vpgd->obj);

	mutex_lock(&nv_subdev(mmu)->mutex);
	for (i = vm->fpde; i <= vm->lpde; i++)
		mmu->map_pgt(pgd, i, vm->pgt[i - vm->fpde].obj);
	list_add(&vpgd->head, &vm->pgd_list);
	mutex_unlock(&nv_subdev(mmu)->mutex);

	ltc->invalidate(ltc);
	/* bar might yet be initialized when this is called */
	if (bar)
		mmu->flush(vm);

	return 0;
}

static void
nvkm_vm_unlink(struct nvkm_vm *vm, struct nvkm_gpuobj *mpgd)
{
	struct nvkm_mmu *mmu = vm->mmu;
	struct nvkm_vm_pgd *vpgd, *tmp;
	struct nvkm_gpuobj *pgd = NULL;

	if (!mpgd)
		return;

	mutex_lock(&nv_subdev(mmu)->mutex);
	list_for_each_entry_safe(vpgd, tmp, &vm->pgd_list, head) {
		if (vpgd->obj == mpgd) {
			pgd = vpgd->obj;
			list_del(&vpgd->head);
			kfree(vpgd);
			break;
		}
	}
	mutex_unlock(&nv_subdev(mmu)->mutex);

	nvkm_gpuobj_ref(NULL, &pgd);
}

static void
nvkm_vm_del(struct kref *kref)
{
	struct nvkm_vm *vm = container_of(kref, typeof(*vm), refcount);
	struct nvkm_vm_pgd *vpgd, *tmp_vpgd;
	struct nvkm_as *as, *tmp_as;

	list_for_each_entry_safe(vpgd, tmp_vpgd, &vm->pgd_list, head) {
		nvkm_vm_unlink(vm, vpgd->obj);
	}

#ifdef CONFIG_SYNC
	if (vm->fence)
		sync_fence_put(vm->fence);
#endif

	list_for_each_entry_safe(as, tmp_as, &vm->as_list, head) {
		nv_error(vm->mmu, "Not clean! Freeing as at "
				  "offset 0x%016llx, length 0x%016llx\n",
			 as->offset, as->length);
		list_del(&as->head);
		nvkm_mm_fini(&as->mm);
		nvkm_mm_free(&vm->mm, &as->node);
	}

	nvkm_mm_fini(&vm->mm);
	vfree(vm->pgt);
	kfree(vm);
}

int
nvkm_vm_ref(struct nvkm_vm *ref, struct nvkm_vm **ptr, struct nvkm_gpuobj *pgd)
{
	if (ref) {
		int ret = nvkm_vm_link(ref, pgd);
		if (ret)
			return ret;

		kref_get(&ref->refcount);
	}

	if (*ptr) {
		nvkm_vm_unlink(*ptr, pgd);
		kref_put(&(*ptr)->refcount, nvkm_vm_del);
	}

	*ptr = ref;
	return 0;
}

int nvkm_vm_fence(struct nvkm_vm *vm, struct fence *fence)
{
#ifdef CONFIG_SYNC
	int ret = 0;
	struct sync_fence *f;

	f = sync_fence_create("nv-pushbuf", fence_get(fence));
	if (!f)
		return -ENOMEM;

	mutex_lock(&vm->fence_lock);
	if (!vm->fence)
		vm->fence = f;
	else {
		struct sync_fence *tmp = sync_fence_merge("vm-last-op", vm->fence, f);
		sync_fence_put(f);
		if (tmp) {
			sync_fence_put(vm->fence);
			vm->fence = tmp;
		} else
			ret = -ENOMEM;
	}

	mutex_unlock(&vm->fence_lock);
	return ret;
#else
	return -ENODEV;
#endif
}

int nvkm_vm_wait(struct nvkm_vm *vm)
{
#ifdef CONFIG_SYNC
	int ret;
	struct sync_fence *fence;

	if (!vm->fence)
		return 0;

	mutex_lock(&vm->fence_lock);
	if ((fence = vm->fence))
		sync_fence_get(fence);
	mutex_unlock(&vm->fence_lock);

	if (!fence)
		return 0;

	ret = sync_fence_wait(fence, -1);
	sync_fence_put(fence);
	return ret;
#else
	if (!vm->fence)
		return 0;
	return -ENODEV;
#endif
}

int nvkm_vm_as_alloc(struct nvkm_vm *vm, u64 align, u64 length, u32 page_shift,
		     u64 *address, bool sparse)
{
	struct nvkm_mmu *mmu = vm->mmu;
	struct nvkm_as *as;
	u32 malign = align >> 12;
	u64 msize = length >> 12;
	u64 moffset = *address >> 12;
	int ret;

	if (sparse && !mmu->sparse)
		return -ENOSYS;

	as = kzalloc(sizeof(*as), GFP_KERNEL);
	if (!as)
		return -ENOMEM;

	as->sparse = sparse;

	/*
	 * Figure out the requested alignment for mappings done in this address
	 * space allocation.  We want to either use big or small page mappings.
	 */
	if ((ffs(align) - 1) >= mmu->lpg_shift)
		as->align_shift = mmu->lpg_shift;
	else if ((ffs(align) - 1) >= mmu->spg_shift)
		as->align_shift = mmu->spg_shift;
	else {
		ret = -EINVAL;
		goto error;
	}

	mutex_lock(&nv_subdev(mmu)->mutex);

	/* Try to allocate the address space from the main address allocator */
	ret = nvkm_mm_head_offset(&vm->mm, 0, page_shift, msize, msize, malign,
				  &as->node, moffset);
	if (ret != 0)
		goto error_unlock;

	/* Initialize our sub-address-allocator */
	ret = nvkm_mm_init(&as->mm, as->node->offset, as->node->length, malign);
	if (ret != 0)
		goto error_mm_free;

	as->offset = as->node->offset << 12;
	as->length = as->node->length << 12;

	list_add(&as->head, &vm->as_list);

	/*
	 * PDE/PTEs for sparse address space must be initialized immediately.
	 *
	 * This mimic the primitive implementation of downstream driver where
	 * PTEs are always allocated for sparse page even if the whole PDE is
	 * sparse.  Progressive approach should be considerred for replacement
	 * in the future.
	 */
	if (sparse) {
		struct nvkm_ltc *ltc = nvkm_ltc(mmu);
		u32 fpde, lpde, pde, pte, num, max, bits, end, cnt;
		int pgsz;

		/* fill PDEs */
		fpde = as->node->offset >> mmu->pgt_bits;
		lpde = (as->node->offset + as->node->length - 1)
			>> mmu->pgt_bits;
		pgsz = as->align_shift == mmu->lpg_shift ?
			BIG_PAGE_INDEX : SMALL_PAGE_INDEX;

		for (pde = fpde; pde <= lpde; pde++) {
			struct nvkm_vm_pgt *vpgt = &vm->pgt[pde - vm->fpde];

			if (likely(vpgt->refcount[pgsz])) {
				vpgt->refcount[pgsz]++;
				continue;
			}

			ret = nvkm_vm_map_pgt(vm, pde, as->node->type);
			if (ret) {
				if (pde != fpde)
					nvkm_vm_unmap_pgt(vm, pgsz, fpde, pde - 1);
				goto error_mm_free;
			}
		}
		mutex_unlock(&nv_subdev(mmu)->mutex);

		bits = as->node->type - 12;
		num = as->node->length >> bits;	/* total num of PTEs to fill */
		max  = 1 << (mmu->pgt_bits - bits);	/* PTEs per table */
		pte  = (as->node->offset & ((1 << mmu->pgt_bits) - 1)) >> bits;
		pde = fpde;

		while (num) {
			struct nvkm_gpuobj *pgt = vm->pgt[pde].obj[pgsz];

			end = (pte + num);
			if (unlikely(end > max))
				end = max;
			cnt = end - pte;

			mmu->sparse(pgt, pte, cnt);

			num -= cnt;
			pde++;
			pte = 0;
		}

		ltc->invalidate(ltc);
		mmu->flush(vm);
	} else
		mutex_unlock(&nv_subdev(mmu)->mutex);

	*address = as->offset;

	return 0;

error_mm_free:
	nvkm_mm_free(&vm->mm, &as->node);
error_unlock:
	mutex_unlock(&nv_subdev(mmu)->mutex);
error:
	kfree(as);
	return ret;
}

int nvkm_vm_as_free(struct nvkm_vm *vm, u64 offset)
{
	struct nvkm_mmu *mmu = vm->mmu;
	struct nvkm_as *as;
	int ret;

	mutex_lock(&nv_subdev(mmu)->mutex);

	as = nvkm_vm_find_as(vm, offset);
	if (!as)
		return -ENOENT;

	/*
	 * This can fail if there are still outstanding allocations in this
	 * address space, in which case, we fail the address space free too.
	 */
	ret = nvkm_mm_fini(&as->mm);
	if (ret < 0) {
		mutex_unlock(&nv_subdev(mmu)->mutex);
		return ret;
	}
	if (as->sparse) {
		struct nvkm_ltc *ltc = nvkm_ltc(mmu);
		u32 fpde, lpde, pde, pte, num, max, bits, end, cnt;
		int pgsz;

		/* release PDEs */
		fpde = as->node->offset >> mmu->pgt_bits;
		lpde = (as->node->offset + as->node->length - 1)
			>> mmu->pgt_bits;
		pgsz = as->align_shift == mmu->lpg_shift ?
			BIG_PAGE_INDEX : SMALL_PAGE_INDEX;

		bits = as->node->type - 12;
		num = as->node->length >> bits;	/* total num of PTEs to fill */
		max  = 1 << (mmu->pgt_bits - bits);	/* PTEs per table */
		pte  = (as->node->offset & ((1 << mmu->pgt_bits) - 1)) >> bits;
		pde = fpde;

		/* Unlock the mutex since we'll flush mmu next */
		mutex_unlock(&nv_subdev(mmu)->mutex);
		/* Clear VOL bit */
		while (num) {
			struct nvkm_gpuobj *pgt = vm->pgt[pde].obj[pgsz];

			end = (pte + num);
			if (unlikely(end > max))
				end = max;
			cnt = end - pte;

			mmu->unmap(pgt, pte, cnt);

			num -= cnt;
			pde++;
			pte = 0;
		}
		ltc->invalidate(ltc);
		mmu->flush(vm);

		mutex_lock(&nv_subdev(mmu)->mutex);
		nvkm_vm_unmap_pgt(vm, pgsz, fpde, lpde);
	}

	nvkm_mm_free(&vm->mm, &as->node);

	list_del(&as->head);

	mutex_unlock(&nv_subdev(mmu)->mutex);

	kfree(as);

	return 0;
}
