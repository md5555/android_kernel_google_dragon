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

#ifndef __GK20A_GR_H__
#define __GK20A_GR_H__

#include "gf100.h"

struct fecs_method_op_gk20a {
	struct {
		u32 addr;
		u32 data;
	} method;

	struct {
		u32 id;
		u32 data;
		u32 clr;
		u32 *ret;
		u32 ok;
		u32 fail;
	} mailbox;

	struct {
		u32 ok;
		u32 fail;
	} cond;
};

enum {
	WAIT_UCODE_LOOP,
	WAIT_UCODE_TIMEOUT,
	WAIT_UCODE_ERROR,
	WAIT_UCODE_OK
};

enum {
	GR_IS_UCODE_OP_EQUAL,
	GR_IS_UCODE_OP_NOT_EQUAL,
	GR_IS_UCODE_OP_AND,
	GR_IS_UCODE_OP_LESSER,
	GR_IS_UCODE_OP_LESSER_EQUAL,
	GR_IS_UCODE_OP_SKIP
};

enum {
	UCODE_HANDSHAKE_INIT_COMPLETE = 1,
	UCODE_HANDSHAKE_METHOD_FINISHED
};

enum {
	FECS_CTX_MAILBOX_0,
	FECS_CTX_MAILBOX_1,
	FECS_CTX_MAILBOX_2,
	FECS_CTX_MAILBOX_3,
	FECS_CTX_MAILBOX_4
};

struct gk20a_gr_oclass {
	struct gf100_gr_oclass gf100;

	void (*init_gpc_mmu)(struct gf100_gr_priv *);
	void (*set_hww_esr_report_mask)(struct gf100_gr_priv *);
};

#endif
