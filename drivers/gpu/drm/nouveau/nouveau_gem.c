/*
 * Copyright (C) 2008 Ben Skeggs.
 * All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial
 * portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE COPYRIGHT OWNER(S) AND/OR ITS SUPPLIERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include <linux/dma-buf.h>
#include <linux/freezer.h>
#include <linux/kthread.h>
#include <linux/wait.h>
#include <sync.h>

#include "nouveau_drm.h"
#include "nouveau_dma.h"
#include "nouveau_fence.h"
#include "nouveau_abi16.h"

#include "nouveau_ttm.h"
#include "nouveau_gem.h"

void
nouveau_gem_object_del(struct drm_gem_object *gem)
{
	struct nouveau_bo *nvbo = nouveau_gem_object(gem);
	struct nouveau_drm *drm = nouveau_bdev(nvbo->bo.bdev);
	struct ttm_buffer_object *bo = &nvbo->bo;
	struct device *dev = drm->dev->dev;
	int ret;

	ret = pm_runtime_get_sync(dev);
	if (WARN_ON(ret < 0 && ret != -EACCES))
		return;

	if (gem->import_attach) {
		ttm_bo_reserve(bo, false, false, false, NULL);
		reservation_object_init(&bo->ttm_resv);
		ttm_bo_unreserve(bo);
		bo->resv = &bo->ttm_resv;
		drm_prime_gem_destroy(gem, nvbo->bo.sg);
	}

	drm_gem_object_release(gem);

	/* reset filp so nouveau_bo_del_ttm() can test for it */
	gem->filp = NULL;
	ttm_bo_unref(&bo);

	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);
}

int
nouveau_gem_object_open(struct drm_gem_object *gem, struct drm_file *file_priv)
{
	struct nouveau_cli *cli = nouveau_cli(file_priv);
	struct nouveau_bo *nvbo = nouveau_gem_object(gem);
	struct nouveau_drm *drm = nouveau_bdev(nvbo->bo.bdev);
	struct nvkm_vma *vma;
	struct device *dev = drm->dev->dev;
	int ret;

	if (!cli->vm)
		return 0;

	ret = ttm_bo_reserve(&nvbo->bo, false, false, false, NULL);
	if (ret)
		return ret;

	vma = nouveau_bo_vma_find(nvbo, cli->vm);
	if (!vma) {
		vma = kzalloc(sizeof(*vma), GFP_KERNEL);
		if (!vma) {
			ret = -ENOMEM;
			goto out;
		}

		ret = pm_runtime_get_sync(dev);
		if (ret < 0 && ret != -EACCES)
			goto out;

		ret = nouveau_bo_vma_add(nvbo, cli->vm, vma, true);
		if (ret)
			kfree(vma);

		pm_runtime_mark_last_busy(dev);
		pm_runtime_put_autosuspend(dev);
	} else {
		vma->refcount++;
	}

out:
	ttm_bo_unreserve(&nvbo->bo);
	return ret;
}

static void
nouveau_gem_object_delete(void *data)
{
	struct nvkm_vma *vma = data;
	nvkm_vm_unmap(vma);
	nvkm_vm_put(vma);
	kfree(vma);
}

struct nouveau_vma_delete_work
{
	struct work_struct work;
	struct nvkm_vma *vma;
	struct nouveau_bo *nvbo;
};

static void
nouveau_gem_free_tags(void *_priv)
{
	struct nouveau_gem_dma_buf_priv *priv = _priv;
	struct nvkm_ltc *ltc;

	if (!_priv)
		return;

	ltc = nvxx_ltc(&priv->drm->device);

	if (priv->tags) {
		struct nvkm_mm_node **pnode = (struct nvkm_mm_node **)&priv->tags;
		mutex_lock(&ltc->tags_lock);
		ltc->tags_free(ltc, pnode);
		mutex_unlock(&ltc->tags_lock);
	}

	kfree(priv);
}

static int
nouveau_gem_alloc_tags(struct nouveau_drm *drm, struct nouveau_bo *nvbo,
		int size, struct nvkm_mem *node)
{
	struct nvkm_ltc *ltc = nvxx_ltc(&drm->device);
	struct drm_gem_object *gem = &nvbo->gem;
	struct dma_buf *dma_buf = gem->import_attach->dmabuf;
	struct nouveau_gem_dma_buf_priv *priv;
	struct device *dev = drm->dev->dev;
	int err = 0;

	mutex_lock(&ltc->tags_lock);
	priv = dma_buf_get_drvdata(dma_buf, dev);
	if (!IS_ERR_OR_NULL(priv)) {
		node->tag = priv->tags;
		mutex_unlock(&ltc->tags_lock);
		return 0;
	}

	priv = kzalloc(sizeof (*priv), GFP_KERNEL);
	if (!priv) {
		err = -ENOMEM;
		goto alloc_err;
	}

	ltc->tags_alloc(ltc, size, &node->tag);
	if (unlikely(!node->tag)) {
		err = -ENOMEM;
		goto tags_alloc_err;
	}

	priv->drm = drm;
	priv->tags = node->tag;

	dma_buf_set_drvdata(dma_buf, dev, priv, nouveau_gem_free_tags);

	ltc->tags_clear(ltc, node->tag->offset, size);

	mutex_unlock(&ltc->tags_lock);

	return 0;

tags_alloc_err:
	kfree(priv);
alloc_err:
	mutex_unlock(&ltc->tags_lock);
	return err;
}

static int
nouveau_gem_init_tags(struct nouveau_drm *drm, struct nouveau_bo *nvbo)
{
	struct nvkm_mmu *mmu = nvxx_mmu(&drm->device);
	struct ttm_mem_reg *mem = &(nvbo->bo.mem);
	struct nvkm_mem *node = mem->mm_node;
	u32 type = (node->memtype & 0x0ff);
	int ret = -EINVAL;

	if (!mmu->uc_type)
		return 0;

	if (mmu->uc_type(mmu, type) != type) {

		/* compression only works with lpages */
		if (mem->page_alignment ==
				(1 << (mmu->lpg_shift - mmu->spg_shift))) {
			int n = mem->num_pages >> (mmu->lpg_shift - mmu->spg_shift);
			ret = nouveau_gem_alloc_tags(drm, nvbo, n, node);
		}

		if (unlikely(ret))
			type = mmu->uc_type(mmu, type);
	}
	node->memtype = type;

	return 0;
}

static void gem_unmap_work(struct work_struct *__work)
{
	struct nouveau_vma_delete_work *del_work =
		container_of(__work, struct nouveau_vma_delete_work, work);
	struct nvkm_vma *vma = del_work->vma;
	struct nouveau_bo *nvbo = del_work->nvbo;
	struct drm_device *dev = nvbo->gem.dev;
	struct reservation_object *resv = nvbo->bo.resv;
	struct reservation_object_list *fobj;
	struct fence *fence = NULL;
	int ret;

	ret = pm_runtime_get_sync(dev->dev);
	WARN_ON(ret < 0 && ret != -EACCES);

	if (vma->mapped)
		WARN_ON(nvkm_vm_wait(vma->vm));

	fobj = reservation_object_get_list(resv);

	list_del(&vma->head);

	if (fobj && fobj->shared_count > 1)
		ttm_bo_wait(&nvbo->bo, true, false, false);
	else if (fobj && fobj->shared_count == 1)
		fence = rcu_dereference_protected(fobj->shared[0],
						reservation_object_held(resv));
	else
		fence = reservation_object_get_excl(nvbo->bo.resv);

	if (fence && vma->mapped) {
		nouveau_fence_work(fence, nouveau_gem_object_delete, vma);
	} else {
		if (vma->mapped)
			nvkm_vm_unmap(vma);
		nvkm_vm_put(vma);
		kfree(vma);
	}

	mutex_lock(&dev->struct_mutex);
	drm_gem_object_unreference(&nvbo->gem);
	mutex_unlock(&dev->struct_mutex);

	kfree(del_work);

	if (ret >= 0) {
		pm_runtime_mark_last_busy(dev->dev);
		pm_runtime_put_autosuspend(dev->dev);
	}
}

/*
 * - Grab a reference to the object so we can access from the worker.
 * - Since the handle is synchronously destroyed by the caller, further
 *   references are impossible, so we don't need to protect against that.
 * - Once the worker is done, it can drop the extra reference.
 */
static void
nouveau_gem_object_unmap(struct nouveau_bo *nvbo, struct nvkm_vma *vma)
{
	struct nouveau_drm *drm = nouveau_bdev(nvbo->bo.bdev);
	struct nouveau_vma_delete_work *del_work;

	/*
	 * If the vma mapping is still deferred, and we want to free the
	 * vma (i.e. we did not submit a pushbuffer that uses this buffer
	 * object, which is normally how mapping deferrals get flushed),
	 * we need to pull the vma off of the deferred mapping list so that
	 * the mapping deferral flush that will happen on the next pushbuffer
	 * submit doesn't try to access our just-freed vma.
	 */
	if (!vma->mapped)
	        nouveau_cancel_defer_vm_map(vma, nvbo);

	del_work = (struct nouveau_vma_delete_work*) kzalloc(sizeof(*del_work), GFP_KERNEL);
	if (WARN_ON(!del_work))
		return;

	drm_gem_object_reference(&nvbo->gem);

	INIT_WORK(&del_work->work, gem_unmap_work);
	del_work->vma = vma;
	del_work->nvbo = nvbo;

	queue_work(drm->gem_unmap_wq, &del_work->work);
}

void
nouveau_gem_object_close(struct drm_gem_object *gem, struct drm_file *file_priv)
{
	struct nouveau_cli *cli = nouveau_cli(file_priv);
	struct nouveau_bo *nvbo = nouveau_gem_object(gem);
	struct nouveau_drm *drm = nouveau_bdev(nvbo->bo.bdev);
	struct device *dev = drm->dev->dev;
	struct nvkm_vma *vma;
	int ret;

	if (!cli->vm)
		return;

	ret = ttm_bo_reserve(&nvbo->bo, false, false, false, NULL);
	if (ret)
		return;

	vma = nouveau_bo_vma_find(nvbo, cli->vm);
	if (vma) {
		if (--vma->refcount == 0) {
			ret = pm_runtime_get_sync(dev);
			if (!WARN_ON(ret < 0 && ret != -EACCES)) {
				nouveau_gem_object_unmap(nvbo, vma);
				pm_runtime_mark_last_busy(dev);
				pm_runtime_put_autosuspend(dev);
			}
		}
	}
	ttm_bo_unreserve(&nvbo->bo);
}

int
nouveau_gem_ioctl_set_tiling(struct drm_device *dev, void *data,
			     struct drm_file *file_priv)
{
	struct nouveau_drm *drm = nouveau_drm(dev);
	struct nouveau_cli *cli = nouveau_cli(file_priv);
	struct nvkm_fb *pfb = nvxx_fb(&drm->device);
	struct drm_nouveau_gem_set_tiling *req = data;
	struct drm_gem_object *gem;
	struct nouveau_bo *nvbo;
	struct nvkm_mem *mem;
	struct nvkm_vma *vma;
	int ret = 0;

	if (!pfb->memtype_valid(pfb, req->tile_flags)) {
		NV_PRINTK(error, cli, "bad memtype: 0x%08x\n", req->tile_flags);
		return -EINVAL;
	}

	gem = drm_gem_object_lookup(dev, file_priv, req->handle);
	if (!gem)
		return -ENOENT;

	nvbo = nouveau_gem_object(gem);

	if (nvbo->tile_mode != req->tile_mode ||
	    nvbo->tile_flags != req->tile_flags) {

		ret = ttm_bo_reserve(&nvbo->bo, false, false, false, NULL);
		if (ret)
			goto out;

		vma = nouveau_bo_vma_find(nvbo, cli->vm);
		if (!vma) {
			ret = -ENOENT;
			goto unreserve;
		}

		mem = nvbo->bo.mem.mm_node;
		nvbo->tile_mode = req->tile_mode;
		nvbo->tile_flags = req->tile_flags;

		/* Need to rewrite page tables */
		mem->memtype = (nvbo->tile_flags >> 8) & 0xff;
		if (gem->import_attach) {
			ret = nouveau_gem_init_tags(drm, nvbo);
			if (ret)
				NV_PRINTK(error, cli, "failed to allocate tagline\n");
		}

		nouveau_defer_vm_map(vma, nvbo);

unreserve:
		ttm_bo_unreserve(&nvbo->bo);
	}

out:
	drm_gem_object_unreference_unlocked(gem);
	return ret;
}

int
nouveau_gem_new(struct drm_device *dev, int size, int align, uint32_t domain,
		uint32_t tile_mode, uint32_t tile_flags,
		struct nouveau_bo **pnvbo)
{
	struct nouveau_drm *drm = nouveau_drm(dev);
	struct nouveau_bo *nvbo;
	u32 flags = 0;
	int ret;

	if (domain & NOUVEAU_GEM_DOMAIN_VRAM)
		flags |= TTM_PL_FLAG_VRAM;
	if (domain & NOUVEAU_GEM_DOMAIN_GART)
		flags |= TTM_PL_FLAG_TT;
	if (!flags || domain & NOUVEAU_GEM_DOMAIN_CPU)
		flags |= TTM_PL_FLAG_SYSTEM;

	if (domain & NOUVEAU_GEM_DOMAIN_COHERENT)
		flags |= TTM_PL_FLAG_UNCACHED;

	ret = nouveau_bo_new(dev, size, align, flags, tile_mode,
			     tile_flags, NULL, NULL, pnvbo);
	if (ret)
		return ret;
	nvbo = *pnvbo;

	/* we restrict allowed domains on nv50+ to only the types
	 * that were requested at creation time.  not possibly on
	 * earlier chips without busting the ABI.
	 */
	nvbo->valid_domains = NOUVEAU_GEM_DOMAIN_VRAM |
			      NOUVEAU_GEM_DOMAIN_GART;
	if (drm->device.info.family >= NV_DEVICE_INFO_V0_TESLA)
		nvbo->valid_domains &= domain;

	/* Initialize the embedded gem-object. We return a single gem-reference
	 * to the caller, instead of a normal nouveau_bo ttm reference. */
	ret = drm_gem_object_init(dev, &nvbo->gem, nvbo->bo.mem.size);
	if (ret) {
		nouveau_bo_ref(NULL, pnvbo);
		return -ENOMEM;
	}

	nvbo->bo.persistent_swap_storage = nvbo->gem.filp;
	return 0;
}

static int
nouveau_gem_info(struct drm_file *file_priv, struct drm_gem_object *gem,
		 struct drm_nouveau_gem_info *rep)
{
	struct nouveau_cli *cli = nouveau_cli(file_priv);
	struct nouveau_bo *nvbo = nouveau_gem_object(gem);
	struct nvkm_vma *vma;

	if (nvbo->bo.mem.mem_type == TTM_PL_TT)
		rep->domain = NOUVEAU_GEM_DOMAIN_GART;
	else
		rep->domain = NOUVEAU_GEM_DOMAIN_VRAM;

	if (!nvbo->gpu_cacheable)
		rep->domain |= NOUVEAU_GEM_DOMAIN_COHERENT;

	rep->offset = nvbo->bo.offset;
	if (cli->vm) {
		vma = nouveau_bo_vma_find(nvbo, cli->vm);
		if (!vma)
			return -EINVAL;

		rep->offset = vma->offset;
	}

	rep->size = nvbo->bo.mem.num_pages << PAGE_SHIFT;
	rep->map_handle = drm_vma_node_offset_addr(&nvbo->bo.vma_node);
	rep->tile_mode = nvbo->tile_mode;
	rep->tile_flags = nvbo->tile_flags;
	return 0;
}

static int nouveau_gem_remap(struct nouveau_drm *drm, struct nvkm_vm *vm,
			     struct nvkm_vma *vma, struct nouveau_bo *nvbo,
			     u64 offset, struct nvkm_vma **new_vma)
{
	struct nvkm_as *as;
	unsigned old_page_shift = nvbo->page_shift;
	int ret;

	/* Unmap the old vma. */
	nouveau_cancel_defer_vm_map(vma, nvbo);
	nouveau_bo_vma_del(nvbo, vma);

	/*
	 * If this offset falls within an address space allocation, then honor
	 * the address space allocation's alignment request (as->align_shift).
	 * This may result in changing the nvbo to map with small page size
	 * when previously it was mapped with large page size.
	 *
	 * Note that if the nvbo was originally mapped with small page size
	 * (if, for example, its size was not a multiple of large page size),
	 * we cannot just promote to large page size.
	 */
	as = nvkm_vm_find_as(vm, offset);
	if (as)
		nvbo->page_shift = min(nvbo->page_shift, as->align_shift);

	*new_vma = kzalloc(sizeof(*vma), GFP_KERNEL);
	if (!*new_vma) {
		ret = -ENOMEM;
		goto error;
	}

	/*
	 * Try mapping the new vma.  If this succeeds, we're done, so delete
	 * the old vma and return the new vma.
	 */
	ret = nouveau_bo_vma_add_offset(nvbo, vm, *new_vma, offset, true);
	if (!ret)
		return 0;

	/*
	 * We could have failed if there is an unmap pending for a given
	 * address, and we ask for that address again, so flush the pending
	 * unmaps and try again.
	 */
	flush_workqueue(drm->gem_unmap_wq);
	ret = nouveau_bo_vma_add_offset(nvbo, vm, *new_vma, offset, true);
	if (!ret)
		return 0;

	/*
	 * We failed, so delete the new vma, and restore the old one.  We just
	 * removed the old one...so we should be able to re-add it without
	 * error.
	 */
	kfree(*new_vma);
	*new_vma = NULL;

error:
	nvbo->page_shift = old_page_shift;
	nouveau_bo_vma_add_offset(nvbo, vm, vma, vma->offset, true);
	return ret;
}

int
nouveau_gem_ioctl_set_info(struct drm_device *dev, void *data,
			   struct drm_file *file_priv)
{
	struct nouveau_drm *drm = nouveau_drm(dev);
	struct nouveau_cli *cli = nouveau_cli(file_priv);
	struct nvkm_fb *pfb = nvxx_fb(&drm->device);
	struct drm_nouveau_gem_info *req = data;
	bool gpu_cacheable = !(req->domain & NOUVEAU_GEM_DOMAIN_COHERENT);
	struct drm_gem_object *gem;
	struct nouveau_bo *nvbo;
	struct nvkm_mem *mem;
	struct nvkm_vma *vma;
	int ret = 0;

	if (!pfb->memtype_valid(pfb, req->tile_flags)) {
		NV_PRINTK(error, cli, "bad memtype: 0x%08x\n", req->tile_flags);
		return -EINVAL;
	}

	gem = drm_gem_object_lookup(dev, file_priv, req->handle);
	if (!gem)
		return -ENOENT;

	nvbo = nouveau_gem_object(gem);

	if (nvbo->tile_mode != req->tile_mode ||
	    nvbo->tile_flags != req->tile_flags ||
	    nvbo->gpu_cacheable != gpu_cacheable ||
	    req->offset) {

		ret = ttm_bo_reserve(&nvbo->bo, false, false, false, NULL);
		if (ret)
			goto out;

		vma = nouveau_bo_vma_find(nvbo, cli->vm);
		if (!vma) {
			ret = -ENOENT;
			goto unreserve;
		}

		/*
		 * If nonzero offset, remap with the new offset (this results
		 * in a new vma).
		 */
		if (req->offset) {
			struct nvkm_vma *new_vma;
			ret = nouveau_gem_remap(drm, cli->vm, vma, nvbo,
						req->offset, &new_vma);
			if (ret)
				goto unreserve;

			kfree(vma);
			vma = new_vma;
		}

		mem = nvbo->bo.mem.mm_node;
		nvbo->tile_mode = req->tile_mode;
		nvbo->tile_flags = req->tile_flags;
		nvbo->gpu_cacheable = gpu_cacheable;

		/* Need to rewrite page tables */
		mem->memtype = (nvbo->tile_flags >> 8) & 0xff;
		mem->cached = nvbo->gpu_cacheable;
		if (gem->import_attach) {
			ret = nouveau_gem_init_tags(drm, nvbo);
			if (ret)
				NV_PRINTK(error, cli, "failed to allocate tagline\n");
		}
		nouveau_defer_vm_map(vma, nvbo);

unreserve:
		ttm_bo_unreserve(&nvbo->bo);
	}

out:
	if (!ret)
		ret = nouveau_gem_info(file_priv, gem, req);

	drm_gem_object_unreference_unlocked(gem);
	return ret;
}

int
nouveau_gem_ioctl_new(struct drm_device *dev, void *data,
		      struct drm_file *file_priv)
{
	struct nouveau_drm *drm = nouveau_drm(dev);
	struct nouveau_cli *cli = nouveau_cli(file_priv);
	struct nvkm_fb *pfb = nvxx_fb(&drm->device);
	struct drm_nouveau_gem_new *req = data;
	struct nouveau_bo *nvbo = NULL;
	int ret = 0;

	if (!pfb->memtype_valid(pfb, req->info.tile_flags)) {
		NV_PRINTK(error, cli, "bad page flags: 0x%08x\n", req->info.tile_flags);
		return -EINVAL;
	}

	ret = nouveau_gem_new(dev, req->info.size, req->align,
			      req->info.domain, req->info.tile_mode,
			      req->info.tile_flags, &nvbo);
	if (ret)
		return ret;

	ret = drm_gem_handle_create(file_priv, &nvbo->gem, &req->info.handle);
	if (ret == 0) {
		ret = nouveau_gem_info(file_priv, &nvbo->gem, &req->info);
		if (ret)
			drm_gem_handle_delete(file_priv, req->info.handle);
	}

	/* drop reference from allocate - handle holds it now */
	drm_gem_object_unreference_unlocked(&nvbo->gem);
	return ret;
}

static int
nouveau_gem_set_domain(struct drm_gem_object *gem, uint32_t read_domains,
		       uint32_t write_domains, uint32_t valid_domains)
{
	struct nouveau_bo *nvbo = nouveau_gem_object(gem);
	struct ttm_buffer_object *bo = &nvbo->bo;
	uint32_t domains = valid_domains & nvbo->valid_domains &
		(write_domains ? write_domains : read_domains);
	uint32_t pref_flags = 0, valid_flags = 0;

	if (!domains)
		return -EINVAL;

	if (valid_domains & NOUVEAU_GEM_DOMAIN_VRAM)
		valid_flags |= TTM_PL_FLAG_VRAM;

	if (valid_domains & NOUVEAU_GEM_DOMAIN_GART)
		valid_flags |= TTM_PL_FLAG_TT;

	if ((domains & NOUVEAU_GEM_DOMAIN_VRAM) &&
	    bo->mem.mem_type == TTM_PL_VRAM)
		pref_flags |= TTM_PL_FLAG_VRAM;

	else if ((domains & NOUVEAU_GEM_DOMAIN_GART) &&
		 bo->mem.mem_type == TTM_PL_TT)
		pref_flags |= TTM_PL_FLAG_TT;

	else if (domains & NOUVEAU_GEM_DOMAIN_VRAM)
		pref_flags |= TTM_PL_FLAG_VRAM;

	else
		pref_flags |= TTM_PL_FLAG_TT;

	nouveau_bo_placement_set(nvbo, pref_flags, valid_flags);

	return 0;
}

struct validate_op {
	struct list_head list;
	struct ww_acquire_ctx ticket;
};

static void
validate_fini_no_ticket(struct validate_op *op, struct nouveau_fence *fence,
			struct drm_nouveau_gem_pushbuf_bo *pbbo)
{
	struct nouveau_bo *nvbo;
	struct drm_nouveau_gem_pushbuf_bo *b;

	while (!list_empty(&op->list)) {
		nvbo = list_entry(op->list.next, struct nouveau_bo, entry);
		b = &pbbo[nvbo->pbbo_index];

		if (likely(fence))
			nouveau_bo_fence(nvbo, fence, !!b->write_domains);

		if (unlikely(nvbo->validate_mapped)) {
			ttm_bo_kunmap(&nvbo->kmap);
			nvbo->validate_mapped = false;
		}

		list_del(&nvbo->entry);
		nvbo->reserved_by = NULL;
		ttm_bo_unreserve_ticket(&nvbo->bo, &op->ticket);
		drm_gem_object_unreference_unlocked(&nvbo->gem);
	}
}

static void
validate_fini(struct validate_op *op, struct nouveau_fence *fence,
	      struct drm_nouveau_gem_pushbuf_bo *pbbo)
{
	validate_fini_no_ticket(op, fence, pbbo);
	ww_acquire_fini(&op->ticket);
}

static int
validate_init(struct nouveau_channel *chan, struct drm_file *file_priv,
	      struct drm_nouveau_gem_pushbuf_bo *pbbo,
	      int nr_buffers, struct validate_op *op)
{
	struct nouveau_cli *cli = nouveau_cli(file_priv);
	struct drm_device *dev = chan->drm->dev;
	int trycnt = 0;
	int ret, i;
	struct nouveau_bo *res_bo = NULL;
	LIST_HEAD(gart_list);
	LIST_HEAD(vram_list);
	LIST_HEAD(both_list);

	ww_acquire_init(&op->ticket, &reservation_ww_class);
retry:
	if (++trycnt > 100000) {
		NV_PRINTK(error, cli, "%s failed and gave up.\n", __func__);
		return -EINVAL;
	}

	for (i = 0; i < nr_buffers; i++) {
		struct drm_nouveau_gem_pushbuf_bo *b = &pbbo[i];
		struct drm_gem_object *gem;
		struct nouveau_bo *nvbo;

		gem = drm_gem_object_lookup(dev, file_priv, b->handle);
		if (!gem) {
			NV_PRINTK(error, cli, "Unknown handle 0x%08x\n", b->handle);
			ret = -ENOENT;
			break;
		}
		nvbo = nouveau_gem_object(gem);
		if (nvbo == res_bo) {
			res_bo = NULL;
			drm_gem_object_unreference_unlocked(gem);
			continue;
		}

		if (nvbo->reserved_by && nvbo->reserved_by == file_priv) {
			NV_PRINTK(error, cli, "multiple instances of buffer %d on "
				      "validation list\n", b->handle);
			drm_gem_object_unreference_unlocked(gem);
			ret = -EINVAL;
			break;
		}

		ret = ttm_bo_reserve(&nvbo->bo, true, false, true, &op->ticket);
		if (ret) {
			list_splice_tail_init(&vram_list, &op->list);
			list_splice_tail_init(&gart_list, &op->list);
			list_splice_tail_init(&both_list, &op->list);
			validate_fini_no_ticket(op, NULL, NULL);
			if (unlikely(ret == -EDEADLK)) {
				ret = ttm_bo_reserve_slowpath(&nvbo->bo, true,
							      &op->ticket);
				if (!ret)
					res_bo = nvbo;
			}
			if (unlikely(ret)) {
				if (ret != -ERESTARTSYS)
					NV_PRINTK(error, cli, "fail reserve\n");
				break;
			}
		}

		b->user_priv = (uint64_t)(unsigned long)nvbo;
		nvbo->reserved_by = file_priv;
		nvbo->pbbo_index = i;
		if ((b->valid_domains & NOUVEAU_GEM_DOMAIN_VRAM) &&
		    (b->valid_domains & NOUVEAU_GEM_DOMAIN_GART))
			list_add_tail(&nvbo->entry, &both_list);
		else
		if (b->valid_domains & NOUVEAU_GEM_DOMAIN_VRAM)
			list_add_tail(&nvbo->entry, &vram_list);
		else
		if (b->valid_domains & NOUVEAU_GEM_DOMAIN_GART)
			list_add_tail(&nvbo->entry, &gart_list);
		else {
			NV_PRINTK(error, cli, "invalid valid domains: 0x%08x\n",
				 b->valid_domains);
			list_add_tail(&nvbo->entry, &both_list);
			ret = -EINVAL;
			break;
		}
		if (nvbo == res_bo)
			goto retry;
	}

	ww_acquire_done(&op->ticket);
	list_splice_tail(&vram_list, &op->list);
	list_splice_tail(&gart_list, &op->list);
	list_splice_tail(&both_list, &op->list);
	if (ret)
		validate_fini(op, NULL, NULL);
	return ret;

}

static int
validate_list(struct nouveau_channel *chan, struct nouveau_cli *cli,
	      struct list_head *list, struct drm_nouveau_gem_pushbuf_bo *pbbo,
	      uint64_t user_pbbo_ptr)
{
	struct nouveau_drm *drm = chan->drm;
	struct drm_nouveau_gem_pushbuf_bo __user *upbbo =
				(void __force __user *)(uintptr_t)user_pbbo_ptr;
	struct nouveau_bo *nvbo;
	int ret, relocs = 0;

	list_for_each_entry(nvbo, list, entry) {
		struct drm_nouveau_gem_pushbuf_bo *b = &pbbo[nvbo->pbbo_index];

		ret = nouveau_gem_set_domain(&nvbo->gem, b->read_domains,
					     b->write_domains,
					     b->valid_domains);
		if (unlikely(ret)) {
			NV_PRINTK(error, cli, "fail set_domain\n");
			return ret;
		}

		ret = nouveau_bo_validate(nvbo, true, false);
		if (unlikely(ret)) {
			if (ret != -ERESTARTSYS)
				NV_PRINTK(error, cli, "fail ttm_validate\n");
			return ret;
		}

		ret = nouveau_bo_sync(nvbo, chan, !!b->write_domains, true);
		if (unlikely(ret)) {
			if (ret != -ERESTARTSYS)
				NV_PRINTK(error, cli, "fail post-validate sync\n");
			return ret;
		}

		if (drm->device.info.family < NV_DEVICE_INFO_V0_TESLA) {
			if (nvbo->bo.offset == b->presumed.offset &&
			    ((nvbo->bo.mem.mem_type == TTM_PL_VRAM &&
			      b->presumed.domain & NOUVEAU_GEM_DOMAIN_VRAM) ||
			     (nvbo->bo.mem.mem_type == TTM_PL_TT &&
			      b->presumed.domain & NOUVEAU_GEM_DOMAIN_GART)))
				continue;

			if (nvbo->bo.mem.mem_type == TTM_PL_TT)
				b->presumed.domain = NOUVEAU_GEM_DOMAIN_GART;
			else
				b->presumed.domain = NOUVEAU_GEM_DOMAIN_VRAM;
			b->presumed.offset = nvbo->bo.offset;
			b->presumed.valid = 0;
			relocs++;

			if (copy_to_user(&upbbo[nvbo->pbbo_index].presumed,
					     &b->presumed, sizeof(b->presumed)))
				return -EFAULT;
		}
	}

	return relocs;
}

static int
nouveau_gem_pushbuf_validate(struct nouveau_channel *chan,
			     struct drm_file *file_priv,
			     struct drm_nouveau_gem_pushbuf_bo *pbbo,
			     uint64_t user_buffers, int nr_buffers,
			     struct validate_op *op, int *apply_relocs)
{
	struct nouveau_cli *cli = nouveau_cli(file_priv);
	int ret;

	INIT_LIST_HEAD(&op->list);

	if (nr_buffers == 0)
		return 0;

	ret = validate_init(chan, file_priv, pbbo, nr_buffers, op);
	if (unlikely(ret)) {
		if (ret != -ERESTARTSYS)
			NV_PRINTK(error, cli, "validate_init\n");
		return ret;
	}

	ret = validate_list(chan, cli, &op->list, pbbo, user_buffers);
	if (unlikely(ret < 0)) {
		if (ret != -ERESTARTSYS)
			NV_PRINTK(error, cli, "validating bo list\n");
		validate_fini(op, NULL, NULL);
		return ret;
	}

	if (apply_relocs)
		*apply_relocs = ret;
	else
		BUG_ON(ret > 0);

	return 0;
}

static inline void
u_free(void *addr)
{
	if (!is_vmalloc_addr(addr))
		kfree(addr);
	else
		vfree(addr);
}

static inline void *
u_memcpya(uint64_t user, unsigned nmemb, unsigned size)
{
	void *mem;
	void __user *userptr = (void __force __user *)(uintptr_t)user;

	size *= nmemb;

	mem = kmalloc(size, GFP_KERNEL | __GFP_NOWARN);
	if (!mem)
		mem = vmalloc(size);
	if (!mem)
		return ERR_PTR(-ENOMEM);

	if (copy_from_user(mem, userptr, size)) {
		u_free(mem);
		return ERR_PTR(-EFAULT);
	}

	return mem;
}

static int
nouveau_gem_pushbuf_reloc_apply(struct nouveau_cli *cli,
				struct drm_nouveau_gem_pushbuf *req,
				struct drm_nouveau_gem_pushbuf_bo *bo)
{
	struct drm_nouveau_gem_pushbuf_reloc *reloc = NULL;
	int ret = 0;
	unsigned i;

	reloc = u_memcpya(req->relocs, req->nr_relocs, sizeof(*reloc));
	if (IS_ERR(reloc))
		return PTR_ERR(reloc);

	for (i = 0; i < req->nr_relocs; i++) {
		struct drm_nouveau_gem_pushbuf_reloc *r = &reloc[i];
		struct drm_nouveau_gem_pushbuf_bo *b;
		struct nouveau_bo *nvbo;
		uint32_t data;

		if (unlikely(r->bo_index > req->nr_buffers)) {
			NV_PRINTK(error, cli, "reloc bo index invalid\n");
			ret = -EINVAL;
			break;
		}

		b = &bo[r->bo_index];
		if (b->presumed.valid)
			continue;

		if (unlikely(r->reloc_bo_index > req->nr_buffers)) {
			NV_PRINTK(error, cli, "reloc container bo index invalid\n");
			ret = -EINVAL;
			break;
		}
		nvbo = (void *)(unsigned long)bo[r->reloc_bo_index].user_priv;

		if (unlikely(r->reloc_bo_offset + 4 >
			     nvbo->bo.mem.num_pages << PAGE_SHIFT)) {
			NV_PRINTK(error, cli, "reloc outside of bo\n");
			ret = -EINVAL;
			break;
		}

		if (!nvbo->kmap.virtual) {
			ret = ttm_bo_kmap(&nvbo->bo, 0, nvbo->bo.mem.num_pages,
					  &nvbo->kmap);
			if (ret) {
				NV_PRINTK(error, cli, "failed kmap for reloc\n");
				break;
			}
			nvbo->validate_mapped = true;
		}

		if (r->flags & NOUVEAU_GEM_RELOC_LOW)
			data = b->presumed.offset + r->data;
		else
		if (r->flags & NOUVEAU_GEM_RELOC_HIGH)
			data = (b->presumed.offset + r->data) >> 32;
		else
			data = r->data;

		if (r->flags & NOUVEAU_GEM_RELOC_OR) {
			if (b->presumed.domain == NOUVEAU_GEM_DOMAIN_GART)
				data |= r->tor;
			else
				data |= r->vor;
		}

		ret = ttm_bo_wait(&nvbo->bo, true, false, false);
		if (ret) {
			NV_PRINTK(error, cli, "reloc wait_idle failed: %d\n", ret);
			break;
		}

		nouveau_bo_wr32(nvbo, r->reloc_bo_offset >> 2, data);
	}

	u_free(reloc);
	return ret;
}

struct nouveau_chan_waiter {
	struct sync_fence_waiter base;
	struct nouveau_channel *chan;
};

struct nouveau_pushbuf_data {
	struct drm_device *dev;
	struct drm_file *file_priv;
	struct nouveau_channel *chan;
	struct sync_fence *input_fence;
	struct nouveau_fence *fence;
	uint32_t nr_push;
	uint32_t nr_buffers;
	uint32_t *push;
	struct drm_nouveau_gem_pushbuf_bo *bo;
	struct validate_op op;
	struct list_head queue;
};

static
void nouveau_free_pushbuf_data(struct nouveau_pushbuf_data *pb_data)
{
	if (pb_data->input_fence)
		sync_fence_put(pb_data->input_fence);

	if (pb_data->nr_buffers)
		validate_fini(&pb_data->op, pb_data->fence, pb_data->bo);

	nouveau_fence_unref(&pb_data->fence);
	u_free(pb_data->push);
	u_free(pb_data->bo);
	kfree(pb_data);
}

static bool
nouveau_vm_map_deferred(struct nvkm_vm *vm)
{
	bool need_flush = false;

	mutex_lock(&vm->dirty_vma_lock);
	while (!list_empty(&vm->dirty_vma_list)) {
		struct nvkm_dirty_vma *dirty_vma = list_entry(vm->dirty_vma_list.next, struct nvkm_dirty_vma, entry);
		struct nouveau_bo *nvbo = dirty_vma->bo;
		struct ttm_buffer_object *ttm_bo = &nvbo->bo;

		nvkm_vm_map(dirty_vma->vma, nvbo->bo.mem.mm_node);
		ttm_bo_unref(&ttm_bo);
		list_del(&dirty_vma->entry);
		kfree(dirty_vma);
		need_flush = true;
	}
	mutex_unlock(&vm->dirty_vma_lock);

	return need_flush;
}

static int
nouveau_gem_do_pushbuf(struct nouveau_pushbuf_data *pb_data)
{
	struct nouveau_cli *cli = nouveau_cli(pb_data->file_priv);
	struct nouveau_channel *chan = pb_data->chan;
	uint32_t *push = pb_data->push;
	int i, ret;

	nouveau_vm_map_deferred(cli->vm);

	mutex_lock(&chan->fifo_lock);
	ret = nouveau_dma_wait(chan, pb_data->nr_push + 1, 16);
	if (ret) {
		NV_PRINTK(error, cli, "nv50cal_space: %d\n", ret);
		goto out;
	}

	for (i = 0; i < pb_data->nr_push * 2; i+=2)
		nv50_dma_push(chan, push[i], push[i+1]);

	ret = nouveau_fence_emit_initted(pb_data->fence, pb_data->chan);
	if (ret) {
		NV_PRINTK(error, cli, "error fencing pushbuf: %d\n", ret);
		WIND_RING(chan);
	}

out:
	mutex_unlock(&chan->fifo_lock);
	return ret;
}

static struct nouveau_pushbuf_data *
nouveau_gem_pushbuf_queue_head(struct nouveau_channel *chan)
{
	struct nouveau_pushbuf_data *pb_data = NULL;

	spin_lock(&chan->pushbuf_lock);
	if (!list_empty(&chan->pushbuf_queue))
		pb_data = list_first_entry(&chan->pushbuf_queue,
					   struct nouveau_pushbuf_data, queue);
	spin_unlock(&chan->pushbuf_lock);

	return pb_data;
}

int
nouveau_gem_pushbuf_queue_kthread_fn(void *data)
{
	struct nouveau_channel *chan = (struct nouveau_channel *)data;
	struct device *dev = chan->drm->dev->dev;
	struct nouveau_pushbuf_data *pb_data = NULL;
	struct sync_fence *fence;
	int ret = 0;

	NV_DEBUG(chan->drm, "PB thread started on channel %s\n",
		nvxx_client(chan)->name);

	set_freezable();

	while (1) {
		ret = wait_event_freezable(chan->pushbuf_waitqueue,
			(pb_data = nouveau_gem_pushbuf_queue_head(chan))
			|| kthread_should_stop());
		if (ret) {
			NV_ERROR(chan->drm,
				 "PB thread interrupted on channel %s\n",
				 nvxx_client(chan)->name);
			break;
		}

		/*
		 * We can break out of the wait_event() above with !pb_data
		 * only if kthread_should_stop() is true. Otherwise we need
		 * to loop until there are no more pushbuffers in the queue.
		 */
		if (unlikely(!pb_data))
			break;

		fence = pb_data->input_fence;
		if (fence) {
			int i;
			for (i = 0; i < fence->num_fences; ++i) {
				struct fence *pt = fence->cbs[i].sync_pt;
				ret = nouveau_fence_sync(pt, chan, true);
				if (ret)
					NV_ERROR(chan->drm,
						 "fence %s [%p] timeout on channel %s\n",
						 fence->name, fence,
						 nvxx_client(chan)->name);
			}
		}

		ret = pm_runtime_get_sync(dev);
		if (unlikely(ret < 0 && ret != -EACCES)) {
			NV_ERROR(chan->drm, "failed to get device %d\n", ret);
			cond_resched();
			continue;
		}

		ret = nouveau_gem_do_pushbuf(pb_data);
		if (ret)
			NV_ERROR(chan->drm, "do_pushbuf() err=%d\n", ret);

		pm_runtime_mark_last_busy(dev);
		pm_runtime_put_autosuspend(dev);

		spin_lock(&chan->pushbuf_lock);
		list_del(&pb_data->queue);
		spin_unlock(&chan->pushbuf_lock);
		nouveau_free_pushbuf_data(pb_data);
	}

	NV_DEBUG(chan->drm, "PB thread exiting on channel %s\n",
		nvxx_client(chan)->name);

	return 0;
}

int
nouveau_gem_ioctl_pushbuf_2(struct drm_device *dev, void *data,
                            struct drm_file *file_priv)
{
	struct nouveau_abi16 *abi16 = nouveau_abi16_get(file_priv, dev);
	struct nouveau_cli *cli = nouveau_cli(file_priv);
	struct nouveau_abi16_chan *temp;
	struct nouveau_drm *drm = nouveau_drm(dev);
	struct drm_nouveau_gem_pushbuf_2 *req = data;
	struct drm_nouveau_gem_pushbuf_bo *bo = NULL;
	struct nouveau_channel *chan = NULL;
	struct sync_fence *input_fence = NULL;
	struct nouveau_fence *fence = NULL;
	uint32_t *push = NULL;
	struct nouveau_pushbuf_data *pb_data = NULL;
	int ret = 0;

	if (unlikely(!abi16))
		return -ENOMEM;

	list_for_each_entry(temp, &abi16->channels, head) {
		if (temp->chan->object->handle == (NVDRM_CHAN | req->channel)) {
			chan = temp->chan;
			break;
		}
	}

	if (!chan)
		return nouveau_abi16_put(abi16, -ENOENT);

	if (!chan->dma.ib_max)
		return nouveau_abi16_put(abi16, -ENODEV);

	req->vram_available = drm->gem.vram_available;
	req->gart_available = drm->gem.gart_available;

	if (unlikely(req->nr_push > NOUVEAU_GEM_MAX_PUSH)) {
		NV_PRINTK(error, cli, "pushbuf push count exceeds limit: %d max %d\n",
		           req->nr_push, NOUVEAU_GEM_MAX_PUSH);
		return nouveau_abi16_put(abi16, -EINVAL);
	}

	if (unlikely(req->nr_buffers > NOUVEAU_GEM_MAX_BUFFERS)) {
		NV_PRINTK(error, cli, "pushbuf bo count exceeds limit: %d max %d\n",
		           req->nr_buffers, NOUVEAU_GEM_MAX_BUFFERS);
		return nouveau_abi16_put(abi16, -EINVAL);
	}

	if (req->nr_push) {
		push = u_memcpya(req->push, req->nr_push, 8);
		if (IS_ERR(push))
			return nouveau_abi16_put(abi16, PTR_ERR(push));
	}

	if (req->nr_buffers) {
		bo = u_memcpya(req->buffers, req->nr_buffers, sizeof(*bo));
		if (IS_ERR(bo)) {
			u_free(push);
			return nouveau_abi16_put(abi16, PTR_ERR(bo));
		}
	}

	pb_data = kzalloc(sizeof(*pb_data), GFP_KERNEL);
	if (!pb_data) {
		ret = -ENOMEM;
		goto out_push_bo;
	}

	/* Validate buffer list */
	ret = nouveau_gem_pushbuf_validate(chan, file_priv, bo, req->buffers,
					   req->nr_buffers, &pb_data->op, NULL);
	if (ret) {
		if (ret != -ERESTARTSYS)
			NV_PRINTK(error, cli, "validate: %d\n", ret);

		goto out_pb_data;
	}

	if (req->flags & NOUVEAU_GEM_PUSHBUF_2_FENCE_WAIT) {
		input_fence = sync_fence_fdget(req->fence);
		if (!input_fence) {
			ret = -EINVAL;
			goto out_validate;
		}
	}

	fence = kzalloc(sizeof(*fence), GFP_KERNEL);
	if (!fence) {
		ret = -ENOMEM;
		goto out_input_fence;
	}

	nouveau_fence_init(fence, chan);

	if (req->flags & NOUVEAU_GEM_PUSHBUF_2_FENCE_EMIT) {
		struct fence *f = fence_get(&fence->base);
		ret = nouveau_fence_install(f, "nv-pushbuf", &req->fence);

		if (ret) {
			NV_PRINTK(error, cli, "fence install: %d\n", ret);
			fence_put(f);
			goto out_fence;
		}
	}
	WARN_ON(nvkm_vm_fence(cli->vm, &fence->base));

	pb_data->dev = dev;
	pb_data->file_priv = file_priv;
	pb_data->chan = chan;
	pb_data->input_fence = input_fence;
	pb_data->fence = fence;
	pb_data->nr_push = req->nr_push;
	pb_data->nr_buffers = req->nr_buffers;
	pb_data->push = push;
	pb_data->bo = bo;

	spin_lock(&chan->pushbuf_lock);
	list_add_tail(&pb_data->queue, &chan->pushbuf_queue);
	spin_unlock(&chan->pushbuf_lock);
	ret = nouveau_abi16_put(abi16, ret);

	wake_up(&chan->pushbuf_waitqueue);

	return ret;

out_fence:
	nouveau_fence_unref(&fence);
out_input_fence:
	if (input_fence)
		sync_fence_put(input_fence);
out_validate:
	if (req->nr_buffers)
		validate_fini(&pb_data->op, fence, bo);
out_pb_data:
	kfree(pb_data);
out_push_bo:
	u_free(push);
	u_free(bo);

	return nouveau_abi16_put(abi16, ret);
}

int
nouveau_gem_ioctl_pushbuf(struct drm_device *dev, void *data,
			  struct drm_file *file_priv)
{
	struct nouveau_abi16 *abi16 = nouveau_abi16_get(file_priv, dev);
	struct nouveau_cli *cli = nouveau_cli(file_priv);
	struct nouveau_abi16_chan *temp;
	struct nouveau_drm *drm = nouveau_drm(dev);
	struct drm_nouveau_gem_pushbuf *req = data;
	struct drm_nouveau_gem_pushbuf_push *push;
	struct drm_nouveau_gem_pushbuf_bo *bo;
	struct nouveau_channel *chan = NULL;
	struct validate_op op;
	struct nouveau_fence *fence = NULL;
	int i, j, ret = 0, do_reloc = 0;

	if (unlikely(!abi16))
		return -ENOMEM;

	nouveau_vm_map_deferred(cli->vm);

	list_for_each_entry(temp, &abi16->channels, head) {
		if (temp->chan->object->handle == (NVDRM_CHAN | req->channel)) {
			chan = temp->chan;
			break;
		}
	}

	if (!chan)
		return nouveau_abi16_put(abi16, -ENOENT);

	req->vram_available = drm->gem.vram_available;
	req->gart_available = drm->gem.gart_available;
	if (unlikely(req->nr_push == 0))
		goto out_next;

	if (unlikely(req->nr_push > NOUVEAU_GEM_MAX_PUSH)) {
		NV_PRINTK(error, cli, "pushbuf push count exceeds limit: %d max %d\n",
			 req->nr_push, NOUVEAU_GEM_MAX_PUSH);
		return nouveau_abi16_put(abi16, -EINVAL);
	}

	if (unlikely(req->nr_buffers > NOUVEAU_GEM_MAX_BUFFERS)) {
		NV_PRINTK(error, cli, "pushbuf bo count exceeds limit: %d max %d\n",
			 req->nr_buffers, NOUVEAU_GEM_MAX_BUFFERS);
		return nouveau_abi16_put(abi16, -EINVAL);
	}

	if (unlikely(req->nr_relocs > NOUVEAU_GEM_MAX_RELOCS)) {
		NV_PRINTK(error, cli, "pushbuf reloc count exceeds limit: %d max %d\n",
			 req->nr_relocs, NOUVEAU_GEM_MAX_RELOCS);
		return nouveau_abi16_put(abi16, -EINVAL);
	}

	push = u_memcpya(req->push, req->nr_push, sizeof(*push));
	if (IS_ERR(push))
		return nouveau_abi16_put(abi16, PTR_ERR(push));

	bo = u_memcpya(req->buffers, req->nr_buffers, sizeof(*bo));
	if (IS_ERR(bo)) {
		u_free(push);
		return nouveau_abi16_put(abi16, PTR_ERR(bo));
	}

	/* Ensure all push buffers are on validate list */
	for (i = 0; i < req->nr_push; i++) {
		if (push[i].bo_index >= req->nr_buffers) {
			NV_PRINTK(error, cli, "push %d buffer not in list\n", i);
			ret = -EINVAL;
			goto out_prevalid;
		}
	}

	/* Validate buffer list */
	ret = nouveau_gem_pushbuf_validate(chan, file_priv, bo, req->buffers,
					   req->nr_buffers, &op, &do_reloc);
	if (ret) {
		if (ret != -ERESTARTSYS)
			NV_PRINTK(error, cli, "validate: %d\n", ret);
		goto out_prevalid;
	}

	/* Apply any relocations that are required */
	if (do_reloc) {
		ret = nouveau_gem_pushbuf_reloc_apply(cli, req, bo);
		if (ret) {
			NV_PRINTK(error, cli, "reloc apply: %d\n", ret);
			goto out_no_lock;
		}
	}

	mutex_lock(&chan->fifo_lock);
	if (chan->dma.ib_max) {
		ret = nouveau_dma_wait(chan, req->nr_push + 1, 16);
		if (ret) {
			NV_PRINTK(error, cli, "nv50cal_space: %d\n", ret);
			goto out;
		}

		for (i = 0; i < req->nr_push; i++) {
			struct nouveau_bo *nvbo = (void *)(unsigned long)
				bo[push[i].bo_index].user_priv;

			nv50_dma_push_bo(chan, nvbo, push[i].offset,
				         push[i].length);
		}
	} else
	if (drm->device.info.chipset >= 0x25) {
		ret = RING_SPACE(chan, req->nr_push * 2);
		if (ret) {
			NV_PRINTK(error, cli, "cal_space: %d\n", ret);
			goto out;
		}

		for (i = 0; i < req->nr_push; i++) {
			struct nouveau_bo *nvbo = (void *)(unsigned long)
				bo[push[i].bo_index].user_priv;

			OUT_RING(chan, (nvbo->bo.offset + push[i].offset) | 2);
			OUT_RING(chan, 0);
		}
	} else {
		ret = RING_SPACE(chan, req->nr_push * (2 + NOUVEAU_DMA_SKIPS));
		if (ret) {
			NV_PRINTK(error, cli, "jmp_space: %d\n", ret);
			goto out;
		}

		for (i = 0; i < req->nr_push; i++) {
			struct nouveau_bo *nvbo = (void *)(unsigned long)
				bo[push[i].bo_index].user_priv;
			uint32_t cmd;

			cmd = chan->push.vma.offset + ((chan->dma.cur + 2) << 2);
			cmd |= 0x20000000;
			if (unlikely(cmd != req->suffix0)) {
				if (!nvbo->kmap.virtual) {
					ret = ttm_bo_kmap(&nvbo->bo, 0,
							  nvbo->bo.mem.
							  num_pages,
							  &nvbo->kmap);
					if (ret) {
						WIND_RING(chan);
						goto out;
					}
					nvbo->validate_mapped = true;
				}

				nouveau_bo_wr32(nvbo, (push[i].offset +
						push[i].length - 8) / 4, cmd);
			}

			OUT_RING(chan, 0x20000000 |
				      (nvbo->bo.offset + push[i].offset));
			OUT_RING(chan, 0);
			for (j = 0; j < NOUVEAU_DMA_SKIPS; j++)
				OUT_RING(chan, 0);
		}
	}

	ret = nouveau_fence_new(chan, false, &fence);
	if (ret) {
		NV_PRINTK(error, cli, "error fencing pushbuf: %d\n", ret);
		WIND_RING(chan);
	}

out:
	mutex_unlock(&chan->fifo_lock);

out_no_lock:
	validate_fini(&op, fence, bo);
	nouveau_fence_unref(&fence);

out_prevalid:
	u_free(bo);
	u_free(push);

out_next:
	if (chan->dma.ib_max) {
		req->suffix0 = 0x00000000;
		req->suffix1 = 0x00000000;
	} else
	if (drm->device.info.chipset >= 0x25) {
		req->suffix0 = 0x00020000;
		req->suffix1 = 0x00000000;
	} else {
		req->suffix0 = 0x20000000 |
			      (chan->push.vma.offset + ((chan->dma.cur + 2) << 2));
		req->suffix1 = 0x00000000;
	}

	return nouveau_abi16_put(abi16, ret);
}

int
nouveau_gem_ioctl_set_error_notifier(struct drm_device *dev, void *data,
                            struct drm_file *file_priv)
{
	struct nouveau_abi16 *abi16 = nouveau_abi16_get(file_priv, dev);
	struct drm_nouveau_gem_set_error_notifier *req = data;
	struct nouveau_abi16_chan *abi16_ch;
	struct nouveau_channel *chan = NULL;
	int ret = 0;

	if (unlikely(!abi16))
		return -ENOMEM;

	list_for_each_entry(abi16_ch, &abi16->channels, head) {
		if (abi16_ch->chan->object->handle
				== (NVDRM_CHAN | req->channel)) {
			chan = abi16_ch->chan;
			break;
		}
	}
	if (!chan)
		return nouveau_abi16_put(abi16, -ENOENT);

	ret = nouveau_channel_init_error_notifier(chan, file_priv,
			req->buffer, req->offset);

	return nouveau_abi16_put(abi16, ret);
}

int
nouveau_gem_ioctl_cpu_prep(struct drm_device *dev, void *data,
			   struct drm_file *file_priv)
{
	struct drm_nouveau_gem_cpu_prep *req = data;
	struct drm_gem_object *gem;
	struct nouveau_bo *nvbo;
	bool no_wait = !!(req->flags & NOUVEAU_GEM_CPU_PREP_NOWAIT);
	bool write = !!(req->flags & NOUVEAU_GEM_CPU_PREP_WRITE);
	int ret;

	gem = drm_gem_object_lookup(dev, file_priv, req->handle);
	if (!gem)
		return -ENOENT;
	nvbo = nouveau_gem_object(gem);

	if (no_wait)
		ret = reservation_object_test_signaled_rcu(nvbo->bo.resv, write) ? 0 : -EBUSY;
	else {
		long lret;

		lret = reservation_object_wait_timeout_rcu(nvbo->bo.resv, write, true, 30 * HZ);
		if (!lret)
			ret = -EBUSY;
		else if (lret > 0)
			ret = 0;
		else
			ret = lret;
	}

	nouveau_bo_l2_flush(nvbo);
	nouveau_bo_sync_for_cpu(nvbo);
	drm_gem_object_unreference_unlocked(gem);

	return ret;
}

int
nouveau_gem_ioctl_cpu_fini(struct drm_device *dev, void *data,
			   struct drm_file *file_priv)
{
	struct drm_nouveau_gem_cpu_fini *req = data;
	struct drm_gem_object *gem;
	struct nouveau_bo *nvbo;

	gem = drm_gem_object_lookup(dev, file_priv, req->handle);
	if (!gem)
		return -ENOENT;
	nvbo = nouveau_gem_object(gem);

	nouveau_bo_sync_for_device(nvbo);
	nouveau_bo_l2_invalidate(nvbo);
	drm_gem_object_unreference_unlocked(gem);
	return 0;
}

int
nouveau_gem_ioctl_info(struct drm_device *dev, void *data,
		       struct drm_file *file_priv)
{
	struct drm_nouveau_gem_info *req = data;
	struct drm_gem_object *gem;
	int ret;

	gem = drm_gem_object_lookup(dev, file_priv, req->handle);
	if (!gem)
		return -ENOENT;

	ret = nouveau_gem_info(file_priv, gem, req);
	drm_gem_object_unreference_unlocked(gem);
	return ret;
}

int
nouveau_gem_ioctl_as_alloc(struct drm_device *dev, void *data,
			   struct drm_file *file_priv)
{
	struct nouveau_abi16 *abi16 = nouveau_abi16_get(file_priv, dev);
	struct nouveau_cli *cli = nouveau_cli(file_priv);
	struct drm_nouveau_gem_as_alloc *req = data;
	u32 page_shift;
	u64 align;
	int ret = 0;

	if (unlikely(!abi16))
		return -ENOMEM;

	/* Check for NPOT page size. */
	page_shift = ffs(req->page_size) - 1;
	if (req->page_size != (1 << page_shift))
		return nouveau_abi16_put(abi16, -EINVAL);

	/*
	 * Make sure requested address meets alignment requirements.
	 * Also handles the "don't care" address of 0.
	 */
	align = max_t(uint64_t, 1 << page_shift, req->align);
	if (!IS_ALIGNED(req->address, align))
		return nouveau_abi16_put(abi16, -EINVAL);

	ret = nvkm_vm_as_alloc(cli->vm, align, req->pages * req->page_size,
			       page_shift, &req->address);

	return nouveau_abi16_put(abi16, ret);
}

int
nouveau_gem_ioctl_as_free(struct drm_device *dev, void *data,
			  struct drm_file *file_priv)
{
	struct nouveau_abi16 *abi16 = nouveau_abi16_get(file_priv, dev);
	struct nouveau_cli *cli = nouveau_cli(file_priv);
	struct nouveau_drm *drm = nouveau_drm(dev);
	struct drm_nouveau_gem_as_free *req = data;
	int ret = 0;

	if (unlikely(!abi16))
		return -ENOMEM;

	flush_workqueue(drm->gem_unmap_wq);

	ret = nvkm_vm_as_free(cli->vm, req->address);

	return nouveau_abi16_put(abi16, ret);
}
