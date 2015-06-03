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

#ifdef __KERNEL__
#include <linux/iommu.h>
#include <nouveau_platform.h>
#endif

#include "priv.h"

#include <core/enum.h>
#include <subdev/timer.h>

/* Algorithm from NVIDIA nvgpu driver */
static int
gm20b_ltc_calc_tags_size(struct gk20a_ltc_priv *priv,
		u32 *compbit_backing_size, u32 *max_comptag_lines)
{
	struct nvkm_ltc_priv *ltc_priv = &priv->priv;
	u64 max_size;
	u32 hw_max_comptag_lines = 0x1ffff;
	u32 cbc_param = nv_rd32(priv, 0x17e280);
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
		cacheline_size * slices_per_fbp * ltc_priv->ltc_nr;

	/* aligned to 2KB * ltc_nr */
	*compbit_backing_size += ltc_priv->ltc_nr << 11;

	/* must be a multiple of 64KB */
	*compbit_backing_size = roundup(*compbit_backing_size, 64 * 1024);

	*max_comptag_lines =
		(*compbit_backing_size * comptags_per_cacheline) /
		(cacheline_size * slices_per_fbp * ltc_priv->ltc_nr);

	if (*max_comptag_lines > hw_max_comptag_lines)
		*max_comptag_lines = hw_max_comptag_lines;

	return 0;
}

static const struct nvkm_bitfield
gm20b_ltc_lts_intr_name[] = {
	{ 0x00000001, "IDLE_ERROR_IQ" },
	{ 0x00000002, "IDLE_ERROR_CBC" },
	{ 0x00000004, "IDLE_ERROR_TSTG" },
	{ 0x00000008, "IDLE_ERROR_DSTG" },
	{ 0x00000010, "EVICTED_CB" },
	{ 0x00000020, "ILLEGAL_COMPSTAT" },
	{ 0x00000040, "BLOCKLINEAR_CB" },
	{ 0x00000100, "ECC_SEC_ERROR" },
	{ 0x00000200, "ECC_DED_ERROR" },
	{ 0x00000400, "DEBUG" },
	{ 0x00000800, "ATOMIC_TO_Z" },
	{ 0x00001000, "ILLEGAL_ATOMIC" },
	{ 0x00002000, "BLKACTIVITY_ERR" },
	{ 0x00004000, "ILLEGAL_COMPSTAT_ACCESS" },
	{ 0x00008000, "ILLEGAL_ROP_ACCESS" },
	{}
};

static void
gm20b_ltc_lts_isr(struct nvkm_ltc_priv *priv, int ltc, int lts)
{
	u32 base = 0x140400 + (ltc * 0x2000) + (lts * 0x200);
	u32 stat = nv_rd32(priv, base + 0x00c) & 0xffff;

	if (stat) {
		nv_error(priv, "LTC%d_LTS%d:", ltc, lts);
		nvkm_bitfield_print(gm20b_ltc_lts_intr_name, stat);
		pr_cont("\n");
		nv_wr32(priv, base + 0x00c, stat);
	}
}

static void
gm20b_ltc_intr(struct nvkm_subdev *subdev)
{
	struct nvkm_ltc_priv *priv = (void *)subdev;
	u32 mask;

	mask = nv_rd32(priv, 0x00017c);
	while (mask) {
		u32 lts, ltc = __ffs(mask);
		for (lts = 0; lts < priv->lts_nr; lts++)
			gm20b_ltc_lts_isr(priv, ltc, lts);
		mask &= ~(1 << ltc);
	}
}

static void
gm20b_ltc_init_cbc(struct nvkm_ltc_priv *priv)
{
	int c, s;

	/* invalidate all compbits */
	nv_wr32(priv, 0x17e26c, 0x2);

	for (c = 0; c < priv->ltc_nr; c++) {
		for (s = 0; s < priv->lts_nr; s++)
			if (!nv_wait(priv, 0x14046c + c * 0x2000 + s * 0x200, 0x2, 0))
				nv_warn(priv, "CBC invalidate timeout, ltc=%d, lts=%d\n",
						c, s);
	}
}

static int
gm20b_ltc_init(struct nvkm_object *object)
{
	struct nvkm_ltc_priv *priv = (void *)object;
	u32 lpg128 = !(nv_rd32(priv, 0x100c80) & 0x00000001);
	int ret;
	u32 intr, val;

	ret = nvkm_ltc_init(priv);
	if (ret)
		return ret;

	nv_mask(priv, 0x17e000, 0x0000001f, priv->ltc_nr);
	nv_mask(priv, 0x17e27c, 0x0000001f, priv->ltc_nr);
	nv_mask(priv, 0x17e264, 0x00000002, lpg128 ? 0x00000002 : 0x00000000);

	val = nv_rd32(priv, 0x140518);
	nv_wr32(priv, 0x17e318, val | 0x8000);

	/* Disable LTC interrups */
	intr = nv_rd32(priv, 0x17e20c);
	intr &= ~(BIT(20) | BIT(30));
	nv_wr32(priv, 0x17e20c, intr);

	nv_wr32(priv, 0x17e278, priv->tag_base);
	gm20b_ltc_init_cbc(priv);

	return 0;
}

static int
gm20b_ltc_fini(struct nvkm_object *object, bool suspend)
{
	struct nvkm_ltc_priv *priv = (void *)object;
	int c;

	/* flush compression bit */
	nv_wr32(priv, 0x17e214, 0x00000001);

	for (c = 0; c < priv->ltc_nr; c++) {
		if (!nv_wait(priv, 0x140214 + c * 0x2000, 0x1, 0x0))
			nv_warn(priv, "compression bit flush timeout, ltc=%d\n", c);
	}

	return nvkm_ltc_fini(priv, suspend);
}

static int
gm20b_ltc_ctor(struct nvkm_object *parent, struct nvkm_object *engine,
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

	priv->calc_tags_size = gm20b_ltc_calc_tags_size;

	ltc_priv = &priv->priv;
	ltc_priv->ltc_nr = nv_rd32(priv, 0x12006c) & 0x1f;
	ltc_priv->lts_nr = nv_rd32(priv, 0x17e280) >> 28;
	nv_debug(priv, "LTC Num = %d, LTS Num = %d\n",
			ltc_priv->ltc_nr, ltc_priv->lts_nr);

	ret = gk20a_ltc_init_tag_mem(priv);
	if (ret)
		return ret;

	return 0;
}

struct nvkm_oclass *
gm20b_ltc_oclass = &(struct nvkm_ltc_impl) {
	.base.handle = NV_SUBDEV(LTC, 0x2b),
	.base.ofuncs = &(struct nvkm_ofuncs) {
		.ctor = gm20b_ltc_ctor,
		.dtor = gk20a_ltc_dtor,
		.init = gm20b_ltc_init,
		.fini = gm20b_ltc_fini,
	},
	.intr = gm20b_ltc_intr,
	.cbc_clear = gm107_ltc_cbc_clear,
	.cbc_wait = gm107_ltc_cbc_wait,
	.zbc = 16,
	.zbc_clear_color = gm107_ltc_zbc_clear_color,
	.zbc_clear_depth = gm107_ltc_zbc_clear_depth,
	.invalidate = gk20a_ltc_invalidate,
	.flush = gk20a_ltc_flush,
}.base;
