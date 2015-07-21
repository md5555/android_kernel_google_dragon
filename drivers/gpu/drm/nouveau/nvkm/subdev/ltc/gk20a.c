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

#include "priv.h"

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

struct nvkm_oclass *
gk20a_ltc_oclass = &(struct nvkm_ltc_impl) {
	.base.handle = NV_SUBDEV(LTC, 0xea),
	.base.ofuncs = &(struct nvkm_ofuncs) {
		.ctor = gf100_ltc_ctor,
		.dtor = gf100_ltc_dtor,
		.init = gk104_ltc_init,
		.fini = _nvkm_ltc_fini,
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
