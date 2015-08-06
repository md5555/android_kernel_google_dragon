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

#include "gk20a.h"

static void
gk20a_vm_map_sg(struct nvkm_vma *vma, struct nvkm_gpuobj *pgt,
		struct nvkm_mem *mem, u32 pte, u32 cnt, dma_addr_t *list)
{
	gk20a_instobj_map_sg(vma, pgt->parent, mem, pte, cnt, list);
}

static void
gk20a_vm_unmap(struct nvkm_gpuobj *pgt, u32 pte, u32 cnt)
{
	gk20a_instobj_unmap_sg(pgt->parent, pte, cnt);
}

int
gk20a_mmu_ctor(struct nvkm_object *parent, struct nvkm_object *engine,
	       struct nvkm_oclass *oclass, void *data, u32 size,
	       struct nvkm_object **pobject)
{
	struct gf100_mmu_priv *priv;
	int ret;

	ret = gf100_mmu_ctor(parent, engine, oclass, data, size, pobject);
	if (ret)
		return ret;

	priv = (void *)*pobject;
	priv->base.map_sg = gk20a_vm_map_sg;
	priv->base.unmap = gk20a_vm_unmap;

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
