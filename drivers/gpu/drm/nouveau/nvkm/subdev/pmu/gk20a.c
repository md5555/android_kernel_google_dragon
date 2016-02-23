/*
 * Copyright (c) 2014-2015, NVIDIA CORPORATION. All rights reserved.
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

#include "priv.h"
#include "gk20a.h"
#include <core/client.h>
#include <core/gpuobj.h>
#include <engine/gr.h>
#include <subdev/bar.h>
#include <subdev/fb.h>
#include <subdev/mc.h>
#include <subdev/timer.h>
#include <subdev/mmu.h>
#include <subdev/pmu.h>
#include <core/object.h>
#include <core/device.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <subdev/clk.h>
#include <subdev/ltc.h>
#include <subdev/timer.h>
#include <subdev/volt.h>

#define APP_VERSION_GK20A  17997577
#define GK20A_PMU_UCODE_SIZE_MAX  (256 * 1024)

#define GK20A_PMU_DMEM_BLKSIZE2		    8

#define PMU_PGENG_GR_BUFFER_IDX_INIT	(0)
#define PMU_PGENG_GR_BUFFER_IDX_ZBC	(1)
#define PMU_PGENG_GR_BUFFER_IDX_FECS	(2)

#define PMU_UNIT_REWIND		(0x00)
#define PMU_UNIT_PG		(0x03)
#define PMU_UNIT_INIT		(0x07)
#define PMU_UNIT_PERFMON	(0x12)
#define PMU_UNIT_THERM		(0x1B)
#define PMU_UNIT_RC		(0x1F)
#define PMU_UNIT_NULL		(0x20)
#define PMU_UNIT_END		(0x23)
#define PMU_UNIT_TEST_START	(0xFE)
#define PMU_UNIT_END_SIM	(0xFF)
#define PMU_UNIT_TEST_END	(0xFF)

#define PMU_UNIT_ID_IS_VALID(id)		\
		(((id) < PMU_UNIT_END) || ((id) >= PMU_UNIT_TEST_START))
#define PMU_DMEM_ALIGNMENT		(4)
#define PMU_DMEM_ALLOC_ALIGNMENT	(32)

#define ZBC_MASK(i)			(~(~(0) << ((i) + 1)) & 0xfffe)

#define GK20A_PMU_UCODE_IMAGE	"gpmu_ucode.bin"

#define PMU_PG_IDLE_THRESHOLD                   32000
#define PMU_PG_POST_POWERUP_IDLE_THRESHOLD      1000000

#define PMU_CMD_FLAGS_PMU_MASK		(0xF0)
#define PMU_CMD_FLAGS_STATUS		BIT(0)
#define PMU_CMD_FLAGS_INTR		BIT(1)
#define PMU_CMD_FLAGS_EVENT		BIT(2)
#define PMU_CMD_FLAGS_WATERMARK		BIT(3)

/*
 * Worst case wait will be 40*40us i.e 1.6 ms,
 * (see its usage) which is acceptable and sufficient for all
 * busy tasks to finish
 */
#define MAX_RETRIES			40
enum {
	OFLAG_READ = 0,
	OFLAG_WRITE
};

#define PMU_MSG_HDR_SIZE	sizeof(struct pmu_hdr)

#define PMU_INIT_MSG_TYPE_PMU_INIT 0

#define PMU_RC_MSG_TYPE_UNHANDLED_CMD	0

/* slcg gr */
static const
struct gating_desc gk20a_slcg_gr[] = {
	{.addr = 0x004041f4, .prod = 0x00000000, .disable = 0x03fffffe},
	{.addr = 0x00409894, .prod = 0x00000040, .disable = 0x0003fffe},
	{.addr = 0x004078c4, .prod = 0x00000000, .disable = 0x000001fe},
	{.addr = 0x00406004, .prod = 0x00000000, .disable = 0x0001fffe},
	{.addr = 0x00405864, .prod = 0x00000000, .disable = 0x000001fe},
	{.addr = 0x00405910, .prod = 0x00000000, .disable = 0xfffffffe},
	{.addr = 0x00408044, .prod = 0x00000000, .disable = 0x000007fe},
	{.addr = 0x00407004, .prod = 0x00000000, .disable = 0x0000001e},
	{.addr = 0x0041a894, .prod = 0x00000000, .disable = 0x0003fffe},
	{.addr = 0x00418504, .prod = 0x00000000, .disable = 0x0001fffe},
	{.addr = 0x0041860c, .prod = 0x00000000, .disable = 0x000001fe},
	{.addr = 0x0041868c, .prod = 0x00000000, .disable = 0x0000001e},
	{.addr = 0x0041871c, .prod = 0x00000000, .disable = 0x0000003e},
	{.addr = 0x00418388, .prod = 0x00000000, .disable = 0x00000001},
	{.addr = 0x0041882c, .prod = 0x00000000, .disable = 0x0001fffe},
	{.addr = 0x00418bc0, .prod = 0x00000000, .disable = 0x000001fe},
	{.addr = 0x00418974, .prod = 0x00000000, .disable = 0x0001fffe},
	{.addr = 0x00418c74, .prod = 0x00000000, .disable = 0xfffffffe},
	{.addr = 0x00418cf4, .prod = 0x00000000, .disable = 0xfffffffe},
	{.addr = 0x00418d74, .prod = 0x00000000, .disable = 0xfffffffe},
	{.addr = 0x00418f10, .prod = 0x00000000, .disable = 0xfffffffe},
	{.addr = 0x00418e10, .prod = 0x00000000, .disable = 0xfffffffe},
	{.addr = 0x00419024, .prod = 0x00000000, .disable = 0x000001fe},
	{.addr = 0x00419a44, .prod = 0x00000000, .disable = 0x0000000e},
	{.addr = 0x00419a4c, .prod = 0x00000000, .disable = 0x000001fe},
	{.addr = 0x00419a54, .prod = 0x00000000, .disable = 0x0000003e},
	{.addr = 0x00419a5c, .prod = 0x00000000, .disable = 0x0000000e},
	{.addr = 0x00419a64, .prod = 0x00000000, .disable = 0x000001fe},
	{.addr = 0x00419a6c, .prod = 0x00000000, .disable = 0x0000000e},
	{.addr = 0x00419a74, .prod = 0x00000000, .disable = 0x0000000e},
	{.addr = 0x00419a7c, .prod = 0x00000000, .disable = 0x0000003e},
	{.addr = 0x00419a84, .prod = 0x00000000, .disable = 0x0000000e},
	{.addr = 0x00419ad0, .prod = 0x00000000, .disable = 0x0000000e},
	{.addr = 0x0041986c, .prod = 0x0000dfc0, .disable = 0x00fffffe},
	{.addr = 0x00419cd8, .prod = 0x00000000, .disable = 0x001ffffe},
	{.addr = 0x00419ce0, .prod = 0x00000000, .disable = 0x001ffffe},
	{.addr = 0x00419c74, .prod = 0x00000000, .disable = 0x0000001e},
	{.addr = 0x00419fd4, .prod = 0x00000000, .disable = 0x0003fffe},
	{.addr = 0x00419fdc, .prod = 0x00000000, .disable = 0xfffffffe},
	{.addr = 0x00419fe4, .prod = 0x00000000, .disable = 0x0000000e},
	{.addr = 0x00419ff4, .prod = 0x00000000, .disable = 0x00003ffe},
	{.addr = 0x00419ffc, .prod = 0x00000000, .disable = 0x0001fffe},
	{.addr = 0x0041be2c, .prod = 0x020bbfc0, .disable = 0xfffffffe},
	{.addr = 0x0041bfec, .prod = 0x00000000, .disable = 0xfffffffe},
	{.addr = 0x0041bed4, .prod = 0x00000000, .disable = 0xfffffffe},
	{.addr = 0x00408814, .prod = 0x00000000, .disable = 0x0001fffe},
	{.addr = 0x0040881c, .prod = 0x00000000, .disable = 0x0001fffe},
	{.addr = 0x00408a84, .prod = 0x00000000, .disable = 0x0001fffe},
	{.addr = 0x00408a8c, .prod = 0x00000000, .disable = 0x0001fffe},
	{.addr = 0x00408a94, .prod = 0x00000000, .disable = 0x0001fffe},
	{.addr = 0x00408a9c, .prod = 0x00000000, .disable = 0x0001fffe},
	{.addr = 0x00408aa4, .prod = 0x00000000, .disable = 0x0001fffe},
	{.addr = 0x00408aac, .prod = 0x00000000, .disable = 0x0001fffe},
	{.addr = 0x004089ac, .prod = 0x00000000, .disable = 0x0001fffe},
	{.addr = 0x00408a24, .prod = 0x00000000, .disable = 0x000001ff},
	{.addr = 0x0017e050, .prod = 0x00000000, .disable = 0x00fffffe},
	{.addr = 0x001200a8, .prod = 0x00000000, .disable = 0x00000001},
	{.addr = 0x0010e48c, .prod = 0x00000000, .disable = 0x0000003e},
	{.addr = 0x00001c04, .prod = 0x00000000, .disable = 0x000000fe},
	{.addr = 0x00106f28, .prod = 0x00000040, .disable = 0x000007fe},
	{.addr = 0x000206b8, .prod = 0x00000000, .disable = 0x0000000f},
	{.addr = 0x0017ea98, .prod = 0x00000000, .disable = 0xfffffffe},
	{.addr = 0x00106f28, .prod = 0x00000040, .disable = 0x000007fe},
	{.addr = 0x00120048, .prod = 0x00000000, .disable = 0x00000049},
};

/* slcg perf */
static const
struct gating_desc gk20a_slcg_perf[] = {
	{.addr = 0x001be018, .prod = 0x000001ff, .disable = 0x00000000},
	{.addr = 0x001bc018, .prod = 0x000001ff, .disable = 0x00000000},
	{.addr = 0x001b8018, .prod = 0x000001ff, .disable = 0x00000000},
	{.addr = 0x001b4124, .prod = 0x00000001, .disable = 0x00000000},
};

/* therm gr */
static const
struct gating_desc gk20a_slcg_therm[] = {
	{.addr = 0x000206b8, .prod = 0x00000000, .disable = 0x0000000f},
};

/* blcg gr */
static const
struct gating_desc gk20a_blcg_gr[] = {
	{.addr = 0x004041f0, .prod = 0x00004046, .disable = 0x00000000},
	{.addr = 0x00409890, .prod = 0x0000007f, .disable = 0x00000000},
	{.addr = 0x004098b0, .prod = 0x0000007f, .disable = 0x00000000},
	{.addr = 0x004078c0, .prod = 0x00000042, .disable = 0x00000000},
	{.addr = 0x00406000, .prod = 0x00004044, .disable = 0x00000000},
	{.addr = 0x00405860, .prod = 0x00004042, .disable = 0x00000000},
	{.addr = 0x0040590c, .prod = 0x00004044, .disable = 0x00000000},
	{.addr = 0x00408040, .prod = 0x00004044, .disable = 0x00000000},
	{.addr = 0x00407000, .prod = 0x00004041, .disable = 0x00000000},
	{.addr = 0x00405bf0, .prod = 0x00004044, .disable = 0x00000000},
	{.addr = 0x0041a890, .prod = 0x0000007f, .disable = 0x00000000},
	{.addr = 0x0041a8b0, .prod = 0x0000007f, .disable = 0x00000000},
	{.addr = 0x00418500, .prod = 0x00004044, .disable = 0x00000000},
	{.addr = 0x00418608, .prod = 0x00004042, .disable = 0x00000000},
	{.addr = 0x00418688, .prod = 0x00004042, .disable = 0x00000000},
	{.addr = 0x00418718, .prod = 0x00000042, .disable = 0x00000000},
	{.addr = 0x00418828, .prod = 0x00000044, .disable = 0x00000000},
	{.addr = 0x00418bbc, .prod = 0x00004042, .disable = 0x00000000},
	{.addr = 0x00418970, .prod = 0x00004042, .disable = 0x00000000},
	{.addr = 0x00418c70, .prod = 0x00004044, .disable = 0x00000000},
	{.addr = 0x00418cf0, .prod = 0x00004044, .disable = 0x00000000},
	{.addr = 0x00418d70, .prod = 0x00004044, .disable = 0x00000000},
	{.addr = 0x00418f0c, .prod = 0x00004044, .disable = 0x00000000},
	{.addr = 0x00418e0c, .prod = 0x00004044, .disable = 0x00000000},
	{.addr = 0x00419020, .prod = 0x00004042, .disable = 0x00000000},
	{.addr = 0x00419038, .prod = 0x00000042, .disable = 0x00000000},
	{.addr = 0x00419a40, .prod = 0x00004042, .disable = 0x00000000},
	{.addr = 0x00419a48, .prod = 0x00004042, .disable = 0x00000000},
	{.addr = 0x00419a50, .prod = 0x00004042, .disable = 0x00000000},
	{.addr = 0x00419a58, .prod = 0x00004042, .disable = 0x00000000},
	{.addr = 0x00419a60, .prod = 0x00004042, .disable = 0x00000000},
	{.addr = 0x00419a68, .prod = 0x00004042, .disable = 0x00000000},
	{.addr = 0x00419a70, .prod = 0x00004042, .disable = 0x00000000},
	{.addr = 0x00419a78, .prod = 0x00004042, .disable = 0x00000000},
	{.addr = 0x00419a80, .prod = 0x00004042, .disable = 0x00000000},
	{.addr = 0x00419acc, .prod = 0x00004047, .disable = 0x00000000},
	{.addr = 0x00419868, .prod = 0x00000043, .disable = 0x00000000},
	{.addr = 0x00419cd4, .prod = 0x00004042, .disable = 0x00000000},
	{.addr = 0x00419cdc, .prod = 0x00004042, .disable = 0x00000000},
	{.addr = 0x00419c70, .prod = 0x00004045, .disable = 0x00000000},
	{.addr = 0x00419fd0, .prod = 0x00004043, .disable = 0x00000000},
	{.addr = 0x00419fd8, .prod = 0x00004045, .disable = 0x00000000},
	{.addr = 0x00419fe0, .prod = 0x00004042, .disable = 0x00000000},
	{.addr = 0x00419fe8, .prod = 0x00004042, .disable = 0x00000000},
	{.addr = 0x00419ff0, .prod = 0x00004044, .disable = 0x00000000},
	{.addr = 0x00419ff8, .prod = 0x00004042, .disable = 0x00000000},
	{.addr = 0x00419f90, .prod = 0x00004042, .disable = 0x00000000},
	{.addr = 0x0041be28, .prod = 0x00000042, .disable = 0x00000000},
	{.addr = 0x0041bfe8, .prod = 0x00004044, .disable = 0x00000000},
	{.addr = 0x0041bed0, .prod = 0x00004044, .disable = 0x00000000},
	{.addr = 0x00408810, .prod = 0x00004042, .disable = 0x00000000},
	{.addr = 0x00408818, .prod = 0x00004042, .disable = 0x00000000},
	{.addr = 0x00408a80, .prod = 0x00004042, .disable = 0x00000000},
	{.addr = 0x00408a88, .prod = 0x00004042, .disable = 0x00000000},
	{.addr = 0x00408a90, .prod = 0x00004042, .disable = 0x00000000},
	{.addr = 0x00408a98, .prod = 0x00004042, .disable = 0x00000000},
	{.addr = 0x00408aa0, .prod = 0x00004042, .disable = 0x00000000},
	{.addr = 0x00408aa8, .prod = 0x00004042, .disable = 0x00000000},
	{.addr = 0x004089a8, .prod = 0x00004042, .disable = 0x00000000},
	{.addr = 0x004089b0, .prod = 0x00000042, .disable = 0x00000000},
	{.addr = 0x004089b8, .prod = 0x00004042, .disable = 0x00000000},
	{.addr = 0x0017ea60, .prod = 0x00000044, .disable = 0x00000000},
	{.addr = 0x0017ea68, .prod = 0x00000044, .disable = 0x00000000},
	{.addr = 0x00100d30, .prod = 0x0000c242, .disable = 0x00000000},
	{.addr = 0x00100d48, .prod = 0x0000c242, .disable = 0x00000000},
	{.addr = 0x00100d3c, .prod = 0x00000242, .disable = 0x00000000},
	{.addr = 0x0017ea78, .prod = 0x00000044, .disable = 0x00000000},
	{.addr = 0x0017e040, .prod = 0x00000044, .disable = 0x00000000},
	{.addr = 0x00100d1c, .prod = 0x00000042, .disable = 0x00000000},
	{.addr = 0x00106f24, .prod = 0x0000c242, .disable = 0x00000000},
	{.addr = 0x0041be00, .prod = 0x00000004, .disable = 0x00000007},
	{.addr = 0x00100d10, .prod = 0x0000c242, .disable = 0x00000000},
	{.addr = 0x0017ea70, .prod = 0x00000044, .disable = 0x00000000},
	{.addr = 0x00001c00, .prod = 0x00000042, .disable = 0x00000000},
	{.addr = 0x00100c98, .prod = 0x00000242, .disable = 0x00000000},
	{.addr = 0x0017e030, .prod = 0x00000044, .disable = 0x00000000},
};

/* pg gr */
static const
struct gating_desc gk20a_pg_gr[] = {
	{.addr = 0x004041f8, .prod = 0x10940000, .disable = 0x00000000},
	{.addr = 0x004041fc, .prod = 0xff00a725, .disable = 0x00000000},
	{.addr = 0x00409898, .prod = 0x10140000, .disable = 0x00000000},
	{.addr = 0x0040989c, .prod = 0xff00000a, .disable = 0x00000000},
	{.addr = 0x004078c8, .prod = 0x10940000, .disable = 0x00000000},
	{.addr = 0x004078cc, .prod = 0xff00a725, .disable = 0x00000000},
	{.addr = 0x00406008, .prod = 0x10940000, .disable = 0x00000000},
	{.addr = 0x0040600c, .prod = 0xff00a725, .disable = 0x00000000},
	{.addr = 0x00405868, .prod = 0x10940000, .disable = 0x00000000},
	{.addr = 0x0040586c, .prod = 0xff00a725, .disable = 0x00000000},
	{.addr = 0x00405914, .prod = 0x10940000, .disable = 0x00000000},
	{.addr = 0x00405924, .prod = 0xff00a725, .disable = 0x00000000},
	{.addr = 0x00408048, .prod = 0x10940000, .disable = 0x00000000},
	{.addr = 0x0040804c, .prod = 0xff00a725, .disable = 0x00000000},
	{.addr = 0x00407008, .prod = 0x10140000, .disable = 0x00000000},
	{.addr = 0x0040700c, .prod = 0xff00000a, .disable = 0x00000000},
	{.addr = 0x00405bf8, .prod = 0x10940000, .disable = 0x00000000},
	{.addr = 0x00405bfc, .prod = 0xff00a725, .disable = 0x00000000},
	{.addr = 0x0041a898, .prod = 0x10140000, .disable = 0x00000000},
	{.addr = 0x0041a89c, .prod = 0xff00000a, .disable = 0x00000000},
	{.addr = 0x00418510, .prod = 0x10940000, .disable = 0x00000000},
	{.addr = 0x00418514, .prod = 0xff00a725, .disable = 0x00000000},
	{.addr = 0x00418610, .prod = 0x10940000, .disable = 0x00000000},
	{.addr = 0x00418614, .prod = 0xff00a725, .disable = 0x00000000},
	{.addr = 0x00418690, .prod = 0x10940000, .disable = 0x00000000},
	{.addr = 0x00418694, .prod = 0xff00a725, .disable = 0x00000000},
	{.addr = 0x00418720, .prod = 0x10940000, .disable = 0x00000000},
	{.addr = 0x00418724, .prod = 0xff00a725, .disable = 0x00000000},
	{.addr = 0x00418840, .prod = 0x10940000, .disable = 0x00000000},
	{.addr = 0x00418844, .prod = 0xff00a725, .disable = 0x00000000},
	{.addr = 0x00418bc4, .prod = 0x10940000, .disable = 0x00000000},
	{.addr = 0x00418bc8, .prod = 0xff00a725, .disable = 0x00000000},
	{.addr = 0x00418978, .prod = 0x10940000, .disable = 0x00000000},
	{.addr = 0x0041897c, .prod = 0xff00a725, .disable = 0x00000000},
	{.addr = 0x00418c78, .prod = 0x10940000, .disable = 0x00000000},
	{.addr = 0x00418c7c, .prod = 0xff00a725, .disable = 0x00000000},
	{.addr = 0x00418cf8, .prod = 0x10940000, .disable = 0x00000000},
	{.addr = 0x00418cfc, .prod = 0xff00a725, .disable = 0x00000000},
	{.addr = 0x00418d78, .prod = 0x10940000, .disable = 0x00000000},
	{.addr = 0x00418d7c, .prod = 0xff00a725, .disable = 0x00000000},
	{.addr = 0x00418f14, .prod = 0x10940000, .disable = 0x00000000},
	{.addr = 0x00418f18, .prod = 0xff00a725, .disable = 0x00000000},
	{.addr = 0x00418e14, .prod = 0x10940000, .disable = 0x00000000},
	{.addr = 0x00418e18, .prod = 0xff00a725, .disable = 0x00000000},
	{.addr = 0x00419030, .prod = 0x10940000, .disable = 0x00000000},
	{.addr = 0x00419050, .prod = 0xff00a725, .disable = 0x00000000},
	{.addr = 0x00419a88, .prod = 0x10940000, .disable = 0x00000000},
	{.addr = 0x00419a8c, .prod = 0xff00a725, .disable = 0x00000000},
	{.addr = 0x00419a90, .prod = 0x10940000, .disable = 0x00000000},
	{.addr = 0x00419a94, .prod = 0xff00a725, .disable = 0x00000000},
	{.addr = 0x00419a98, .prod = 0x10940000, .disable = 0x00000000},
	{.addr = 0x00419a9c, .prod = 0xff00a725, .disable = 0x00000000},
	{.addr = 0x00419aa0, .prod = 0x10940000, .disable = 0x00000000},
	{.addr = 0x00419aa4, .prod = 0xff00a725, .disable = 0x00000000},
	{.addr = 0x00419ad4, .prod = 0x10940000, .disable = 0x00000000},
	{.addr = 0x00419ad8, .prod = 0xff00a725, .disable = 0x00000000},
	{.addr = 0x00419870, .prod = 0x10940000, .disable = 0x00000000},
	{.addr = 0x00419874, .prod = 0xff00a725, .disable = 0x00000000},
	{.addr = 0x00419ce4, .prod = 0x10940000, .disable = 0x00000000},
	{.addr = 0x00419cf0, .prod = 0xff00a725, .disable = 0x00000000},
	{.addr = 0x00419c78, .prod = 0x10940000, .disable = 0x00000000},
	{.addr = 0x00419c7c, .prod = 0xff00a725, .disable = 0x00000000},
	{.addr = 0x00419fa0, .prod = 0x10940000, .disable = 0x00000000},
	{.addr = 0x00419fa4, .prod = 0xff00a725, .disable = 0x00000000},
	{.addr = 0x00419fa8, .prod = 0x10940000, .disable = 0x00000000},
	{.addr = 0x00419fac, .prod = 0xff00a725, .disable = 0x00000000},
	{.addr = 0x00419fb0, .prod = 0x10940000, .disable = 0x00000000},
	{.addr = 0x00419fb4, .prod = 0xff00a725, .disable = 0x00000000},
	{.addr = 0x00419fb8, .prod = 0x10940000, .disable = 0x00000000},
	{.addr = 0x00419fbc, .prod = 0xff00a725, .disable = 0x00000000},
	{.addr = 0x00419fc0, .prod = 0x10940000, .disable = 0x00000000},
	{.addr = 0x00419fc4, .prod = 0xff00a725, .disable = 0x00000000},
	{.addr = 0x00419fc8, .prod = 0x10940000, .disable = 0x00000000},
	{.addr = 0x00419fcc, .prod = 0xff00a725, .disable = 0x00000000},
	{.addr = 0x0041be30, .prod = 0x10940000, .disable = 0x00000000},
	{.addr = 0x0041be34, .prod = 0xff00a725, .disable = 0x00000000},
	{.addr = 0x0041bff0, .prod = 0x10747c00, .disable = 0x00000000},
	{.addr = 0x0041bff4, .prod = 0xff00000a, .disable = 0x00000000},
	{.addr = 0x0041bed8, .prod = 0x10240a00, .disable = 0x00000000},
	{.addr = 0x0041bee0, .prod = 0xff00000a, .disable = 0x00000000},
	{.addr = 0x00408820, .prod = 0x10940000, .disable = 0x00000000},
	{.addr = 0x00408824, .prod = 0xff00a725, .disable = 0x00000000},
	{.addr = 0x00408828, .prod = 0x10940000, .disable = 0x00000000},
	{.addr = 0x0040882c, .prod = 0xff00a725, .disable = 0x00000000},
	{.addr = 0x00408ac0, .prod = 0x10940000, .disable = 0x00000000},
	{.addr = 0x00408ac4, .prod = 0xff00a725, .disable = 0x00000000},
	{.addr = 0x00408ac8, .prod = 0x10940000, .disable = 0x00000000},
	{.addr = 0x00408acc, .prod = 0xff00a725, .disable = 0x00000000},
	{.addr = 0x00408ad0, .prod = 0x10940000, .disable = 0x00000000},
	{.addr = 0x00408ad4, .prod = 0xff00a725, .disable = 0x00000000},
	{.addr = 0x00408ad8, .prod = 0x10940000, .disable = 0x00000000},
	{.addr = 0x00408adc, .prod = 0xff00a725, .disable = 0x00000000},
	{.addr = 0x00408ae0, .prod = 0x10940000, .disable = 0x00000000},
	{.addr = 0x00408ae4, .prod = 0xff00a725, .disable = 0x00000000},
	{.addr = 0x00408ae8, .prod = 0x10940000, .disable = 0x00000000},
	{.addr = 0x00408aec, .prod = 0xff00a725, .disable = 0x00000000},
	{.addr = 0x004089c0, .prod = 0x10940000, .disable = 0x00000000},
	{.addr = 0x004089c4, .prod = 0xff00a725, .disable = 0x00000000},
	{.addr = 0x004089c8, .prod = 0x10940000, .disable = 0x00000000},
	{.addr = 0x004089cc, .prod = 0xff00a725, .disable = 0x00000000},
	{.addr = 0x004089d0, .prod = 0x10940000, .disable = 0x00000000},
	{.addr = 0x004089d4, .prod = 0xff00a725, .disable = 0x00000000},
};

struct pmu_pg_stats {
	u64 pg_entry_start_timestamp;
	u64 pg_ingating_start_timestamp;
	u64 pg_exit_start_timestamp;
	u64 pg_ungating_start_timestamp;
	u32 pg_avg_entry_time_us;
	u32 pg_ingating_cnt;
	u32 pg_ingating_time_us;
	u32 pg_avg_exit_time_us;
	u32 pg_ungating_count;
	u32 pg_ungating_time_us;
	u32 pg_gating_cnt;
	u32 pg_gating_deny_cnt;
};

void
gk20a_enable_load_gating_prod(struct nvkm_pmu *pmu,
			  const struct gating_desc *desc, int size)
{
	unsigned int i;

	for (i = 0; i < size; i++)
		nv_wr32(pmu, desc[i].addr, desc[i].prod);
}

void
gk20a_disable_load_gating_prod(struct nvkm_pmu *pmu,
			  const struct gating_desc *desc, int size)
{
	unsigned int i;

	for (i = 0; i < size; i++)
		nv_wr32(pmu, desc[i].addr, desc[i].disable);
}

/* Writen by pmu, read by Sw, accessed by interrupt handler, no lock. */
#define PMU_MESSAGE_QUEUE		4

#define PMU_IS_COMMAND_QUEUE(id)	\
		((id)  < PMU_MESSAGE_QUEUE)

#define PMU_IS_SW_COMMAND_QUEUE(id)	\
		(((id) == PMU_COMMAND_QUEUE_HPQ) || \
		 ((id) == PMU_COMMAND_QUEUE_LPQ))

#define  PMU_IS_MESSAGE_QUEUE(id)	\
		((id) == PMU_MESSAGE_QUEUE)

#define QUEUE_ALIGNMENT			(4)

#define PMU_INVALID_MUTEX_OWNER_ID	(0)

#define PMU_INVALID_SEQ_DESC		(~0)

enum {
	PMU_SEQ_STATE_FREE = 0,
	PMU_SEQ_STATE_PENDING,
	PMU_SEQ_STATE_USED,
	PMU_SEQ_STATE_CANCELLED
};

enum {
	PMU_ELPG_STAT_OFF = 0,           /* 0 elpg is off */
	PMU_ELPG_STAT_ON,                /* 1 elpg is on */
	/* elpg is off, ALLOW cmd has been sent, wait for ack 2 */
	PMU_ELPG_STAT_ON_PENDING,
	/* elpg is on, DISALLOW cmd has been sent, wait for ack 3 */
	PMU_ELPG_STAT_OFF_PENDING,
};

enum {
	PMU_PG_ELPG_MSG_INIT_ACK,
	PMU_PG_ELPG_MSG_DISALLOW_ACK,
	PMU_PG_ELPG_MSG_ALLOW_ACK
};

struct gk20a_pmu_dvfs_data {
	int p_load_target;
	int p_load_max;
	int p_smooth;
	unsigned int avg_load;
};

struct gk20a_pmu_dvfs_dev_status {
	unsigned long total;
	unsigned long busy;
	int cur_state;
};


void
gk20a_release_firmware(struct nvkm_pmu *ppmu, const struct firmware *pfw)
{
	if (WARN_ON(!pfw))
		return;

	nv_debug(ppmu, "firmware released\n");
	release_firmware(pfw);
}

int
gk20a_load_firmware(struct nvkm_pmu *ppmu, const struct firmware **pfw, const
char *fw_name)
{
	struct nvkm_device *dev;
	char name[72];
	int ret;

	dev = nv_device(ppmu);
	snprintf(name, sizeof(name), "nouveau/%s", fw_name);
	ret = request_firmware(pfw, name, nv_device_base(dev));
	return ret;
}

/*reads memory allocated to nvgpu object*/
void
gpu_obj_memwr(struct nvkm_gpuobj *ucodeobj, int offset, void *src, int size)
{
	int temp = size;
	u32 *source32;
	u16 *source16;
	u8 *source8;
	int four_bytes_cnt, two_bytes_cnt, one_bytes_cnt;

	four_bytes_cnt = temp / 4;
	temp = temp % 4;
	two_bytes_cnt = temp / 2;
	temp = temp % 2;
	one_bytes_cnt = temp;
	source32 = (u32 *)src;
	for (temp = 0; temp < four_bytes_cnt; temp++) {
		source32 = (u32 *)src + temp;
		nv_wo32(ucodeobj, offset, *source32);
		offset += 4;
	}
	source16 = (u16 *)source32;
	for (temp = 0; temp < two_bytes_cnt; temp++) {
		source16 = (u16 *)source32 + temp;
		nv_wo16(ucodeobj, offset, *source16);
		offset += 2;
	}
	source8 = (u8 *)source16;
	for (temp = 0; temp < one_bytes_cnt; temp++) {
		source8 = (u8 *)source16 + temp;
		nv_wo08(ucodeobj, offset, *source8);
		offset += 1;
	}

}

static void
gk20a_pmu_dump_firmware_info(struct nvkm_pmu *pmu, const struct firmware *fw)
{
	struct pmu_ucode_desc *desc = (struct pmu_ucode_desc *)fw->data;

	nv_debug(pmu, "GK20A PMU firmware information\n");
	nv_debug(pmu, "descriptor size = %u\n", desc->descriptor_size);
	nv_debug(pmu, "image size  = %u\n", desc->image_size);
	nv_debug(pmu, "app_version = 0x%08x\n", desc->app_version);
	nv_debug(pmu, "date = %s\n", desc->date);
	nv_debug(pmu, "bootloader_start_offset = 0x%08x\n",
				desc->bootloader_start_offset);
	nv_debug(pmu, "bootloader_size = 0x%08x\n", desc->bootloader_size);
	nv_debug(pmu, "bootloader_imem_offset = 0x%08x\n",
				desc->bootloader_imem_offset);
	nv_debug(pmu, "bootloader_entry_point = 0x%08x\n",
				desc->bootloader_entry_point);
	nv_debug(pmu, "app_start_offset = 0x%08x\n", desc->app_start_offset);
	nv_debug(pmu, "app_size = 0x%08x\n", desc->app_size);
	nv_debug(pmu, "app_imem_offset = 0x%08x\n", desc->app_imem_offset);
	nv_debug(pmu, "app_imem_entry = 0x%08x\n", desc->app_imem_entry);
	nv_debug(pmu, "app_dmem_offset = 0x%08x\n", desc->app_dmem_offset);
	nv_debug(pmu, "app_resident_code_offset = 0x%08x\n",
			desc->app_resident_code_offset);
	nv_debug(pmu, "app_resident_code_size = 0x%08x\n",
			desc->app_resident_code_size);
	nv_debug(pmu, "app_resident_data_offset = 0x%08x\n",
			desc->app_resident_data_offset);
	nv_debug(pmu, "app_resident_data_size = 0x%08x\n",
			desc->app_resident_data_size);
	nv_debug(pmu, "nb_overlays = %d\n", desc->nb_overlays);

	nv_debug(pmu, "compressed = %u\n", desc->compressed);
}

static int
gk20a_pmu_dvfs_target(struct gk20a_pmu_priv *priv, int *state)
{
	struct nvkm_clk *clk = nvkm_clk(priv);

	return nvkm_clk_astate(clk, *state, 0, false);
}

static int
gk20a_pmu_dvfs_get_cur_state(struct gk20a_pmu_priv *priv, int *state)
{
	struct nvkm_clk *clk = nvkm_clk(priv);

	*state = clk->pstate;
	return 0;
}

static int
gk20a_pmu_dvfs_get_target_state(struct gk20a_pmu_priv *priv,
				int *state, int avg_load, int load)
{
	struct gk20a_pmu_dvfs_data *data = priv->data;
	struct nvkm_clk *clk = nvkm_clk(priv);
	int cur_level, level;
	static int inhibit = 0;

	/* For GK20A, the performance level is directly mapped to pstate */
	level = cur_level = clk->pstate;

	if (inhibit > 0)
		inhibit --;

	if (load > data->p_load_max) {
		level = min(clk->state_nr - 1, level + (clk->state_nr / 3));
		inhibit = data->p_smooth;
	} else if (inhibit <= 0) {
		level += ((avg_load - data->p_load_target) * 10 /
				data->p_load_target) / 2;
		level = max(0, level);
		level = min(clk->state_nr - 1, level);
	}

	nv_trace(priv, "cur level = %d, new level = %d\n", cur_level, level);

	*state = level;

	if (level == cur_level)
		return 0;
	else
		return 1;
}

static int
gk20a_pmu_dvfs_get_dev_status(struct gk20a_pmu_priv *priv,
			      struct gk20a_pmu_dvfs_dev_status *status)
{
	status->busy = nv_rd32(priv, 0x10a508 + (BUSY_SLOT * 0x10));
	status->total= nv_rd32(priv, 0x10a508 + (CLK_SLOT * 0x10));
	return 0;
}

static void
gk20a_pmu_dvfs_reset_dev_status(struct gk20a_pmu_priv *priv)
{
	nv_wr32(priv, 0x10a508 + (BUSY_SLOT * 0x10), 0x80000000);
	nv_wr32(priv, 0x10a508 + (CLK_SLOT * 0x10), 0x80000000);
}

void
gk20a_pmu_dvfs_init(struct gk20a_pmu_priv *priv)
{
	nv_wr32(priv, 0x10a504 + (BUSY_SLOT * 0x10), 0x00200001);
	nv_wr32(priv, 0x10a50c + (BUSY_SLOT * 0x10), 0x00000002);
	nv_wr32(priv, 0x10a50c + (CLK_SLOT * 0x10), 0x00000003);
}

void
gk20a_pmu_dvfs_work(struct nvkm_alarm *alarm)
{
	struct gk20a_pmu_priv *priv =
		container_of(alarm, struct gk20a_pmu_priv, alarm);
	struct gk20a_pmu_dvfs_data *data = priv->data;
	struct gk20a_pmu_dvfs_dev_status status;
	struct nvkm_clk *clk = nvkm_clk(priv);
	struct nvkm_volt *volt = nvkm_volt(priv);
	u32 utilization = 0;
	int state, ret;

	/*
	 * The PMU is initialized before CLK and VOLT, so we have to make sure the
	 * CLK and VOLT are ready here.
	 */
	if (!clk || !volt)
		goto resched;

	ret = gk20a_pmu_dvfs_get_dev_status(priv, &status);
	if (ret) {
		nv_warn(priv, "failed to get device status\n");
		goto resched;
	}

	if (status.total)
		utilization = div_u64((u64)status.busy * 100, status.total);

	data->avg_load = (data->p_smooth * data->avg_load) + utilization;
	data->avg_load /= data->p_smooth + 1;
	nv_trace(priv, "utilization = %d %%, avg_load = %d %%\n",
			utilization, data->avg_load);

	ret = gk20a_pmu_dvfs_get_cur_state(priv, &state);
	if (ret) {
		nv_warn(priv, "failed to get current state\n");
		goto resched;
	}

	if (gk20a_pmu_dvfs_get_target_state(priv, &state, data->avg_load, utilization)) {
		nv_trace(priv, "set new state to %d\n", state);
		gk20a_pmu_dvfs_target(priv, &state);
	}

resched:
	gk20a_pmu_dvfs_reset_dev_status(priv);
	nvkm_timer_alarm(priv, PMU_DVFS_INTERVAL, alarm);
}

int
gk20a_pmu_enable_hw(struct gk20a_pmu_priv *priv, struct nvkm_mc *pmc, bool enable)
{
	if (enable) {
		nv_mask(pmc, 0x000200, 0x00002000, 0x00002000);
		nv_rd32(pmc, 0x00000200);
		if (nv_wait(priv, 0x0010a10c, 0x00000006, 0x00000000))
			return 0;
		nv_mask(pmc, 0x00000200, 0x2000, 0x00000000);
		nv_error(priv, "Falcon mem scrubbing timeout\n");
		return -ETIMEDOUT;
	} else {
		nv_mask(pmc, 0x00000200, 0x2000, 0x00000000);
		return 0;
	}
}

void
gk20a_pmu_enable_irq(struct gk20a_pmu_priv *priv, struct nvkm_mc *pmc, bool enable)
{
	if (enable) {
		nv_debug(priv, "enable pmu irq\n");
		nv_wr32(priv, 0x0010a010, 0xff);
		nv_mask(pmc, 0x00000640, 0x1000000, 0x1000000);
		nv_mask(pmc, 0x00000644, 0x1000000, 0x1000000);
	} else {
		nv_debug(priv, "disable pmu irq\n");
		nv_mask(pmc, 0x00000640, 0x1000000, 0x00000000);
		nv_mask(pmc, 0x00000644, 0x1000000, 0x00000000);
		nv_wr32(priv, 0x0010a014, 0xff);
	}

}

int
gk20a_pmu_idle(struct gk20a_pmu_priv *priv)
{
	if (!nv_wait(priv, 0x0010a04c, 0x0000ffff, 0x00000000)) {
		nv_error(priv, "timeout waiting pmu idle\n");
		return -EBUSY;
	}

	return 0;
}

int
gk20a_pmu_enable(struct gk20a_pmu_priv *priv, struct nvkm_mc *pmc, bool enable)
{
	u32 pmc_enable;
	int err;

	if (enable) {
		err = gk20a_pmu_enable_hw(priv, pmc, true);
		if (err)
			return err;

		err = gk20a_pmu_idle(priv);
		if (err)
			return err;

		gk20a_pmu_enable_irq(priv, pmc, true);
	} else {
		pmc_enable = nv_rd32(pmc, 0x200);
		if ((pmc_enable & 0x2000) != 0x0) {
			gk20a_pmu_enable_irq(priv, pmc, false);
			gk20a_pmu_enable_hw(priv, pmc, false);
		}
	}

	return 0;
}

void
gk20a_pmu_copy_to_dmem(struct gk20a_pmu_priv *priv, u32 dst, u8 *src, u32 size,
		       u8 port)
{
	u32 i, words, bytes;
	u32 data, addr_mask;
	u32 *src_u32 = (u32 *)src;

	if (size == 0) {
		nv_error(priv, "size is zero\n");
		goto out;
	}

	if (dst & 0x3) {
		nv_error(priv, "dst (0x%08x) not 4-byte aligned\n", dst);
		goto out;
	}

	mutex_lock(&priv->pmu_copy_lock);
	words = size >> 2;
	bytes = size & 0x3;
	addr_mask = 0xfffc;
	dst &= addr_mask;

	nv_wr32(priv, (0x10a1c0 + (port * 8)), (dst | (0x1 << 24)));

	for (i = 0; i < words; i++) {
		nv_wr32(priv, (0x10a1c4 + (port * 8)), src_u32[i]);
		nv_debug(priv, "0x%08x\n", src_u32[i]);
	}

	if (bytes > 0) {
		data = 0;
		for (i = 0; i < bytes; i++)
			((u8 *)&data)[i] = src[(words << 2) + i];
		nv_wr32(priv, (0x10a1c4 + (port * 8)), data);
		nv_debug(priv, "0x%08x\n", data);
	}

	data = nv_rd32(priv, (0x10a1c0 + (port * 8))) & addr_mask;
	size = ALIGN(size, 4);
	if (data != dst + size) {
		nv_error(priv, "copy failed.... bytes written %d, expected %d\n",
							      data - dst, size);
	}
	mutex_unlock(&priv->pmu_copy_lock);
out:
	nv_debug(priv, "exit %s\n", __func__);
}

static void
gk20a_copy_from_dmem(struct gk20a_pmu_priv *priv, u32 src, u8 *dst, u32 size,
		     u8 port)
{
	u32 i, words, bytes;
	u32 data, addr_mask;
	u32 *dst_u32 = (u32 *)dst;

	if (size == 0) {
		nv_error(priv, "size is zero\n");
		goto out;
	}

	if (src & 0x3) {
		nv_error(priv, "src (0x%08x) not 4-byte aligned\n", src);
		goto out;
	}

	mutex_lock(&priv->pmu_copy_lock);

	words = size >> 2;
	bytes = size & 0x3;

	addr_mask = 0xfffc;

	src &= addr_mask;

	nv_wr32(priv, (0x10a1c0 + (port * 8)), (src | (0x1 << 25)));

	for (i = 0; i < words; i++) {
		dst_u32[i] = nv_rd32(priv, (0x0010a1c4 + port * 8));
		nv_debug(priv, "0x%08x\n", dst_u32[i]);
	}
	if (bytes > 0) {
		data = nv_rd32(priv, (0x0010a1c4 + port * 8));
		nv_debug(priv, "0x%08x\n", data);

		for (i = 0; i < bytes; i++)
			dst[(words << 2) + i] = ((u8 *)&data)[i];
	}
	mutex_unlock(&priv->pmu_copy_lock);
out:
	nv_debug(priv, "exit %s\n", __func__);
}

void
gk20a_pmu_seq_init(struct gk20a_pmu_priv *pmu)
{
	u32 i;

	memset(pmu->seq, 0,
		sizeof(struct pmu_sequence) * PMU_MAX_NUM_SEQUENCES);
	memset(pmu->pmu_seq_tbl, 0,
		sizeof(pmu->pmu_seq_tbl));

	for (i = 0; i < PMU_MAX_NUM_SEQUENCES; i++)
		pmu->seq[i].id = i;
}

static int
gk20a_pmu_seq_acquire(struct gk20a_pmu_priv *priv,
			struct pmu_sequence **pseq)
{
	struct nvkm_pmu *pmu = &priv->base;
	struct pmu_sequence *seq;
	u32 index;

	mutex_lock(&priv->pmu_seq_lock);
	index = find_first_zero_bit(priv->pmu_seq_tbl,
				PMU_MAX_NUM_SEQUENCES);
	if (index >= PMU_MAX_NUM_SEQUENCES) {
		nv_error(pmu,
			"no free sequence available\n");
		mutex_unlock(&priv->pmu_seq_lock);
		return -EAGAIN;
	}
	set_bit(index, priv->pmu_seq_tbl);
	mutex_unlock(&priv->pmu_seq_lock);
	seq = &priv->seq[index];
	seq->state = PMU_SEQ_STATE_PENDING;
	nv_debug(pmu, "seq id acquired is = %d index = %d\n", seq->id, index);

	*pseq = seq;
	return 0;
}

static void
gk20a_pmu_seq_release(struct gk20a_pmu_priv *pmu,
			struct pmu_sequence *seq)
{
	seq->state = PMU_SEQ_STATE_FREE;
	seq->desc = PMU_INVALID_SEQ_DESC;
	seq->callback = NULL;
	seq->cb_params = NULL;
	seq->msg = NULL;
	seq->out_payload = NULL;
	seq->in_gk20a.alloc.dmem.size = 0;
	seq->out_gk20a.alloc.dmem.size = 0;
	nv_debug(&pmu->base, "seq released %d\n", seq->id);
	clear_bit(seq->id, pmu->pmu_seq_tbl);
}

static int
gk20a_pmu_queue_init(struct gk20a_pmu_priv *priv,
		u32 id, struct pmu_init_msg_pmu_gk20a *init)
{
	struct nvkm_pmu *pmu = &priv->base;
	struct pmu_queue *queue = &priv->queue[id];

	queue->id = id;
	queue->index = init->queue_info[id].index;
	queue->offset = init->queue_info[id].offset;
	queue->size = init->queue_info[id].size;
	queue->mutex_id = id;
	mutex_init(&queue->mutex);

	nv_debug(pmu, "queue %d: index %d, offset 0x%08x, size 0x%08x\n",
		id, queue->index, queue->offset, queue->size);

	return 0;
}

static u32
gk20a_pmu_queue_head_get(struct gk20a_pmu_priv *priv, struct pmu_queue *queue)
{
	struct nvkm_pmu *pmu = &priv->base;

	if (PMU_IS_COMMAND_QUEUE(queue->id)) {
		/*
		 * Return some safe value to avoid overwriting some other
		 * data if write is attempted to such incorrect queue.
		 *
		 * The queue will be always seen as empty.
		 */
		if (WARN_ON(queue->index > 3))
			return queue->offset;

		return nv_rd32(pmu, 0x0010a4a0 + 4 * (queue->index & 3));
	}

	return nv_rd32(pmu, 0x0010a4c8);
}

static void
gk20a_pmu_queue_head_set(struct gk20a_pmu_priv *priv, struct pmu_queue *queue,
			 u32 head)
{
	struct nvkm_pmu *pmu = &priv->base;

	if (PMU_IS_COMMAND_QUEUE(queue->id)) {
		if (WARN_ON(queue->index > 3))
			return;

		nv_wr32(pmu, 0x0010a4a0 + 4 * (queue->index & 3), head);
		return;
	}

	nv_wr32(pmu, 0x0010a4c8, head);
}

static u32
gk20a_pmu_queue_tail_get(struct gk20a_pmu_priv *priv, struct pmu_queue *queue)
{
	struct nvkm_pmu *pmu = &priv->base;

	if (PMU_IS_COMMAND_QUEUE(queue->id)) {
		/*
		 * Return some safe value to avoid overwriting some other
		 * data if write is attempted to such incorrect queue.
		 *
		 * The queue will be always seen as empty.
		 */
		if (WARN_ON(queue->index > 3))
			return queue->offset;

		return nv_rd32(pmu, 0x0010a4b0 + 4 * (queue->index & 3));
	}

	return nv_rd32(pmu, 0x0010a4cc);
}

static void
gk20a_pmu_queue_tail_set(struct gk20a_pmu_priv *priv, struct pmu_queue *queue,
			 u32 tail)
{
	struct nvkm_pmu *pmu = &priv->base;

	if (PMU_IS_COMMAND_QUEUE(queue->id)) {
		if (WARN_ON(queue->index > 3))
			return;

		nv_wr32(pmu, 0x0010a4b0 + 4 * (queue->index & 3), tail);
	}

	nv_wr32(pmu, 0x0010a4cc, tail);
}

static inline void
gk20a_pmu_queue_read(struct gk20a_pmu_priv *priv,
			u32 offset, u8 *dst, u32 size)
{
	gk20a_copy_from_dmem(priv, offset, dst, size, 0);
}

static inline void
gk20a_pmu_queue_write(struct gk20a_pmu_priv *priv,
			u32 offset, u8 *src, u32 size)
{
	gk20a_pmu_copy_to_dmem(priv, offset, src, size, 0);
}

int
gk20a_pmu_mutex_acquire(struct nvkm_pmu *pmu, u32 id, u32 *token)
{
	struct gk20a_pmu_priv *priv = to_gk20a_priv(pmu);
	struct pmu_mutex *mutex;
	u32 data, owner, max_retry;

	if (WARN_ON(!priv->out_of_reset))
		return -EINVAL;

	if (WARN_ON(!token))
		return -EINVAL;

	if (WARN_ON(id > priv->mutex_cnt))
		return -EINVAL;

	mutex = &priv->mutex[id];

	owner = nv_rd32(pmu, 0x0010a580 + (mutex->index * 4)) & 0xff;

	if (*token != PMU_INVALID_MUTEX_OWNER_ID && *token == owner) {
		if (WARN_ON(mutex->ref_cnt == 0))
			return -EINVAL;

		nv_debug(pmu, "already acquired by owner : 0x%08x\n", *token);
		mutex->ref_cnt++;
		return 0;
	}

	/*
	 * Worst case wait will be 40*40us i.e 1.6 ms,
	 * (see its usage) which is acceptable and sufficient for all
	 * busy tasks to finish.
	 */
	max_retry = MAX_RETRIES;
	do {
		data = nv_rd32(pmu, 0x0010a488) & 0xff;
		if (data == 0 || data == 0xff) {
			nv_warn(pmu,
				"fail to generate mutex token: val 0x%08x\n",
				owner);
			usleep_range(20, 40);
			continue;
		}

		owner = data;
		nv_wr32(pmu, (0x0010a580 + mutex->index * 4),
			owner & 0xff);

		data = nv_rd32(pmu, 0x0010a580 + mutex->index * 4);

		if (owner == data) {
			mutex->ref_cnt = 1;
			nv_debug(pmu, "mutex acquired: id=%d, token=0x%x\n",
							mutex->index, *token);
			*token = owner;
			return 0;
		}
		/*
		 * This can happen if the same mutex is used by some other task
		 * in PMU. This time is sufficient/affordable for a task to
		 * release acquired mutex.
		 */
		nv_debug(pmu, "fail to acquire mutex idx=0x%08x\n",
							mutex->index);
		nv_mask(pmu, 0x0010a48c, 0xff, owner & 0xff);
		usleep_range(20, 40);
		continue;

	} while (max_retry-- > 0);

	return -EBUSY;
}

int
gk20a_pmu_mutex_release(struct nvkm_pmu *pmu, u32 id, u32 *token)
{
	struct gk20a_pmu_priv *priv = to_gk20a_priv(pmu);
	struct pmu_mutex *mutex;
	u32 owner;

	if (WARN_ON(!priv->out_of_reset))
		return -EINVAL;

	if (WARN_ON(!token))
		return -EINVAL;

	if (WARN_ON(id > priv->mutex_cnt))
		return -EINVAL;

	mutex = &priv->mutex[id];

	owner = nv_rd32(pmu, 0x0010a580 + (mutex->index * 4)) & 0xff;

	if (*token != owner) {
		nv_error(pmu,
			"requester 0x%08x NOT match owner 0x%08x\n",
			*token, owner);
		return -EINVAL;
	}

	if (--mutex->ref_cnt > 0)
		return 0;

	nv_wr32(pmu, 0x0010a580 + (mutex->index * 4), 0x00);

	nv_mask(pmu, 0x0010a48c, 0xff, owner & 0xff);

	nv_debug(pmu, "mutex released: id=%d, token=0x%x\n",
							  mutex->index, *token);

	return 0;
}

static int
gk20a_pmu_queue_lock(struct gk20a_pmu_priv *priv,
			struct pmu_queue *queue)
{
	struct nvkm_pmu *pmu = &priv->base;

	if (PMU_IS_MESSAGE_QUEUE(queue->id))
		return 0;

	if (PMU_IS_SW_COMMAND_QUEUE(queue->id)) {
		mutex_lock(&queue->mutex);
		return 0;
	}

	return gk20a_pmu_mutex_acquire(pmu, queue->mutex_id,
						&queue->mutex_lock);

}

static int
gk20a_pmu_queue_unlock(struct gk20a_pmu_priv *priv,
			struct pmu_queue *queue)
{
	struct nvkm_pmu *pmu = &priv->base;

	if (PMU_IS_MESSAGE_QUEUE(queue->id))
		return 0;

	if (PMU_IS_SW_COMMAND_QUEUE(queue->id)) {
		mutex_unlock(&queue->mutex);
		return 0;
	}

	return gk20a_pmu_mutex_release(pmu, queue->mutex_id,
						&queue->mutex_lock);
}

/* called by gk20a_pmu_read_message, no lock */
static bool
gk20a_pmu_queue_is_empty(struct gk20a_pmu_priv *priv,
			struct pmu_queue *queue)
{
	u32 head, tail;

	head = gk20a_pmu_queue_head_get(priv, queue);
	if (queue->opened && queue->oflag == OFLAG_READ)
		tail = queue->position;
	else
		tail = gk20a_pmu_queue_tail_get(priv, queue);

	return head == tail;
}

static bool
gk20a_pmu_queue_has_room(struct gk20a_pmu_priv *priv,
			struct pmu_queue *queue, u32 size, bool *need_rewind)
{
	u32 head, tail, free;
	bool rewind = false;

	size = ALIGN(size, QUEUE_ALIGNMENT);

	head = gk20a_pmu_queue_head_get(priv, queue);
	tail = gk20a_pmu_queue_tail_get(priv, queue);

	if (head >= tail) {
		free = queue->offset + queue->size - head;
		free -= PMU_CMD_HDR_SIZE;

		if (size > free) {
			rewind = true;
			head = queue->offset;
		}
	}

	if (head < tail)
		free = tail - head - 1;

	if (need_rewind)
		*need_rewind = rewind;

	return size <= free;
}

static int
gk20a_pmu_queue_push(struct gk20a_pmu_priv *priv,
			struct pmu_queue *queue, void *data, u32 size)
{

	struct nvkm_pmu *pmu = &priv->base;

	if (!queue->opened && queue->oflag == OFLAG_WRITE) {
		nv_error(pmu, "queue not opened for write\n");
		return -EINVAL;
	}

	gk20a_pmu_queue_write(priv, queue->position, data, size);
	queue->position += ALIGN(size, QUEUE_ALIGNMENT);
	return 0;
}

static int
gk20a_pmu_queue_pop(struct gk20a_pmu_priv *priv,
			struct pmu_queue *queue, void *data, u32 size,
			u32 *bytes_read)
{
	u32 head, tail, used;
	struct nvkm_pmu *pmu = &priv->base;

	*bytes_read = 0;

	if (!queue->opened && queue->oflag == OFLAG_READ) {
		nv_error(pmu, "queue not opened for read\n");
		return -EINVAL;
	}

	head = gk20a_pmu_queue_head_get(priv, queue);
	tail = queue->position;

	if (head == tail) {
		*bytes_read = 0;
		return 0;
	}

	if (head > tail)
		used = head - tail;
	else
		used = queue->offset + queue->size - tail;

	if (size > used) {
		nv_warn(pmu, "queue size smaller than request read\n");
		size = used;
	}

	gk20a_pmu_queue_read(priv, tail, data, size);
	queue->position += ALIGN(size, QUEUE_ALIGNMENT);
	*bytes_read = size;
	return 0;
}

static void
gk20a_pmu_queue_rewind(struct gk20a_pmu_priv *priv,
			struct pmu_queue *queue)
{
	struct pmu_cmd cmd;
	struct nvkm_pmu *pmu = &priv->base;
	int err;

	if (!queue->opened) {
		nv_error(pmu, "queue not opened\n");
		return;
	}

	if (queue->oflag == OFLAG_WRITE) {
		cmd.hdr.unit_id = PMU_UNIT_REWIND;
		cmd.hdr.size = PMU_CMD_HDR_SIZE;
		err = gk20a_pmu_queue_push(priv, queue, &cmd, cmd.hdr.size);
		if (err)
			nv_error(pmu, "gk20a_pmu_queue_push failed\n");
		nv_debug(pmu, "queue %d rewinded\n", queue->id);
	}

	queue->position = queue->offset;
	nv_debug(pmu, "exit %s\n", __func__);
}

/* Open for read and lock the queue */
static int
gk20a_pmu_queue_open_read(struct gk20a_pmu_priv *priv,
			struct pmu_queue *queue)
{
	int err;

	err = gk20a_pmu_queue_lock(priv, queue);
	if (err)
		return err;

	if (WARN_ON(queue->opened))
		return -EBUSY;

	queue->position = gk20a_pmu_queue_tail_get(priv, queue);
	queue->oflag = OFLAG_READ;
	queue->opened = true;

	return 0;
}

/*
 * open for write and lock the queue
 * make sure there's enough free space for the write
 */
static int
gk20a_pmu_queue_open_write(struct gk20a_pmu_priv *priv,
			struct pmu_queue *queue, u32 size)
{
	struct nvkm_pmu *pmu = &priv->base;
	bool rewind = false;
	int err;

	err = gk20a_pmu_queue_lock(priv, queue);
	if (err)
		return err;

	if (WARN_ON(queue->opened))
		return -EBUSY;

	if (!gk20a_pmu_queue_has_room(priv, queue, size, &rewind)) {
		nv_error(pmu, "queue full\n");
		gk20a_pmu_queue_unlock(priv, queue);
		return -EAGAIN;
	}

	queue->position = gk20a_pmu_queue_head_get(priv, queue);
	queue->oflag = OFLAG_WRITE;
	queue->opened = true;

	if (rewind)
		gk20a_pmu_queue_rewind(priv, queue);

	return 0;
}

/* close and unlock the queue */
static int
gk20a_pmu_queue_close(struct gk20a_pmu_priv *priv,
			struct pmu_queue *queue, bool commit)
{
	struct nvkm_pmu *pmu = &priv->base;

	if (WARN_ON(!queue->opened)) {
		nv_warn(pmu, "queue already closed\n");
		return 0;
	}

	if (commit) {
		if (queue->oflag == OFLAG_READ) {
			gk20a_pmu_queue_tail_set(priv, queue,
				queue->position);
		} else {
			gk20a_pmu_queue_head_set(priv, queue,
				queue->position);
		}
	}

	queue->opened = false;
	gk20a_pmu_queue_unlock(priv, queue);

	return 0;
}

static bool
gk20a_check_cmd_params(struct gk20a_pmu_priv *priv, struct pmu_cmd *cmd,
			struct pmu_msg *msg, struct pmu_payload *payload,
			u32 queue_id)
{
	struct nvkm_pmu *pmu = &priv->base;
	struct pmu_queue *queue;
	u32 in_size, out_size;

	nv_debug(pmu, "check cmd params\n");

	if (!PMU_IS_SW_COMMAND_QUEUE(queue_id))
		return false;

	queue = &priv->queue[queue_id];
	if (cmd->hdr.size < PMU_CMD_HDR_SIZE)
		return false;

	if (cmd->hdr.size > (queue->size >> 1))
		return false;

	if (msg != NULL && msg->hdr.size < PMU_MSG_HDR_SIZE)
		return false;

	if (!PMU_UNIT_ID_IS_VALID(cmd->hdr.unit_id))
		return false;

	if (payload == NULL)
		return true;

	if (payload->in.buf == NULL && payload->out.buf == NULL)
		return false;

	if ((payload->in.buf != NULL && payload->in.size == 0) ||
	    (payload->out.buf != NULL && payload->out.size == 0))
		return false;

	in_size = PMU_CMD_HDR_SIZE;
	if (payload->in.buf) {
		in_size += payload->in.offset;
		in_size += sizeof(struct pmu_allocation_gk20a);
	}

	out_size = PMU_CMD_HDR_SIZE;
	if (payload->out.buf) {
		out_size += payload->out.offset;
		out_size += sizeof(struct pmu_allocation_gk20a);
	}

	if (in_size > cmd->hdr.size || out_size > cmd->hdr.size)
		return false;


	if ((payload->in.offset != 0 && payload->in.buf == NULL) ||
	    (payload->out.offset != 0 && payload->out.buf == NULL))
		return false;

	return true;
}

/*
 * PMU DMEM allocator
 * *addr != 0 for fixed address allocation, if *addr = 0,
 * base addr is returned to caller in *addr.
 * Contigous allocation, which allocates one block of
 * contiguous address.
 */
static int
gk20a_pmu_allocator_block_alloc(struct nvkm_pmu_allocator *allocator,
		u32 *addr, u32 len, u32 align)
{
	unsigned long _addr;

	len = ALIGN(len, align);
	if (!len)
		return -ENOMEM;

	_addr = bitmap_find_next_zero_area(allocator->bitmap,
			allocator->size,
			*addr ? (*addr - allocator->base) : 0,
			len,
			align - 1);
	if ((_addr > allocator->size) ||
	    (*addr && *addr != (_addr + allocator->base))) {
		return -ENOMEM;
	}

	bitmap_set(allocator->bitmap, _addr, len);
	*addr = allocator->base + _addr;

	return 0;
}

/* Free all blocks between start and end. */
static int
gk20a_pmu_allocator_block_free(struct nvkm_pmu_allocator *allocator,
		u32 addr, u32 len, u32 align)
{
	len = ALIGN(len, align);
	if (!len)
		return -EINVAL;

	bitmap_clear(allocator->bitmap, addr - allocator->base, len);

	return 0;
}

static int
gk20a_pmu_allocator_init(struct nvkm_pmu_allocator *allocator,
		const char *name, u32 start, u32 len)
{
	/* kfree is safe even bitmap is NULL */
	kfree(allocator->bitmap);

	allocator->base = start;
	allocator->size = len;

	allocator->bitmap = kcalloc(BITS_TO_LONGS(len), sizeof(long),
			GFP_KERNEL);
	if (!allocator->bitmap)
		return -ENOMEM;

	return 0;
}

/* destroy allocator, free all remaining blocks if any */
void
gk20a_pmu_allocator_destroy(struct nvkm_pmu_allocator *allocator)
{
	kfree(allocator->bitmap);
}

static bool
gk20a_pmu_validate_cmd(struct gk20a_pmu_priv *priv, struct pmu_cmd *cmd,
			struct pmu_msg *msg, struct pmu_payload *payload,
			u32 queue_id)
{
	bool params_valid;
	struct nvkm_pmu *pmu = &priv->base;

	params_valid = gk20a_check_cmd_params(priv, cmd, msg,
							payload, queue_id);
	nv_debug(pmu, "pmu validate cmd\n");

	if (!params_valid)
		nv_error(pmu, "invalid pmu cmd :\n"
			"queue_id=%d,\n"
			"cmd_size=%d, cmd_unit_id=%d, msg=%p, msg_size=%d,\n"
			"payload in=%p, in_size=%d, in_offset=%d,\n"
			"payload out=%p, out_size=%d, out_offset=%d\n",
			queue_id, cmd->hdr.size, cmd->hdr.unit_id,
			msg, msg ? msg->hdr.unit_id : ~0,
			&payload->in, payload->in.size, payload->in.offset,
			&payload->out, payload->out.size, payload->out.offset);

	return params_valid;
}

static int
gk20a_pmu_write_cmd(struct gk20a_pmu_priv *priv, struct pmu_cmd *cmd,
			u32 queue_id, unsigned long timeout)
{
	struct nvkm_pmu *pmu = &priv->base;
	struct pmu_queue *queue;
	unsigned long end_jiffies = jiffies +
		msecs_to_jiffies(timeout);
	int err;

	nv_debug(pmu, "pmu write cmd\n");

	queue = &priv->queue[queue_id];

	do {
		err = gk20a_pmu_queue_open_write(priv, queue, cmd->hdr.size);
		if (err != -EAGAIN || time_after(jiffies, end_jiffies))
			break;
		usleep_range(1000, 2000);

	} while (1);

	if (err) {
		nv_error(pmu, "pmu_queue_open_write failed\n");
		return err;
	}

	err = gk20a_pmu_queue_push(priv, queue, cmd, cmd->hdr.size);
	if (err) {
		nv_error(pmu, "pmu_queue_push failed\n");
		goto clean_up;
	}

	err = gk20a_pmu_queue_close(priv, queue, true);
	if (err)
		nv_error(pmu, "fail to close the queue %d\n", queue_id);

	nv_debug(pmu, "cmd writing done\n");

	return 0;

clean_up:
	nv_error(pmu, "%s failed\n", __func__);

	err = gk20a_pmu_queue_close(priv, queue, true);
	if (err)
		nv_error(pmu, "fail to close the queue %d\n", queue_id);

	return err;
}

int
gk20a_pmu_cmd_post(struct nvkm_pmu *pmu, struct pmu_cmd *cmd,
		struct pmu_msg *msg, struct pmu_payload *payload,
		u32 queue_id, pmu_callback callback, void *cb_param,
		u32 *seq_desc, unsigned long timeout)
{
	struct gk20a_pmu_priv *priv = to_gk20a_priv(pmu);
	struct pmu_sequence *seq;
	struct pmu_allocation_gk20a *in = NULL, *out = NULL;
	int err;

	if (WARN_ON(!cmd))
		return -EINVAL;
	if (WARN_ON(!seq_desc))
		return -EINVAL;
	if (WARN_ON(!priv->pmu_ready))
		return -EINVAL;

	if (!gk20a_pmu_validate_cmd(priv, cmd, msg, payload, queue_id))
		return -EINVAL;

	err = gk20a_pmu_seq_acquire(priv, &seq);
	if (err)
		return err;

	cmd->hdr.seq_id = seq->id;

	cmd->hdr.ctrl_flags = 0;
	cmd->hdr.ctrl_flags |= PMU_CMD_FLAGS_STATUS;
	cmd->hdr.ctrl_flags |= PMU_CMD_FLAGS_INTR;

	seq->callback = callback;
	seq->cb_params = cb_param;
	seq->msg = msg;
	seq->out_payload = NULL;
	seq->desc = priv->next_seq_desc++;

	if (payload)
		seq->out_payload = payload->out.buf;

	*seq_desc = seq->desc;

	if (payload && payload->in.offset != 0) {
		in = (struct pmu_allocation_gk20a *)((u8 *)&cmd->cmd +
			payload->in.offset);

		in->alloc.dmem.size = payload->in.size;

		err = gk20a_pmu_allocator_block_alloc(&priv->dmem,
						&in->alloc.dmem.offset,
						in->alloc.dmem.size,
						PMU_DMEM_ALLOC_ALIGNMENT);
		if (err) {
			nv_error(pmu, "gk20a_pmu_allocator alloc failed\n");
			goto clean_up;
		}

		gk20a_pmu_copy_to_dmem(priv, in->alloc.dmem.offset,
			payload->in.buf, payload->in.size, 0);
		seq->in_gk20a.alloc.dmem.size = in->alloc.dmem.size;
		seq->in_gk20a.alloc.dmem.offset = in->alloc.dmem.offset;
	}

	if (payload && payload->out.offset != 0) {
		out = (struct pmu_allocation_gk20a *)((u8 *)&cmd->cmd +
			payload->out.offset);

		out->alloc.dmem.size = payload->out.size;

		err = gk20a_pmu_allocator_block_alloc(&priv->dmem,
					&out->alloc.dmem.offset,
					out->alloc.dmem.size,
					PMU_DMEM_ALLOC_ALIGNMENT);
		if (err) {
			nv_error(pmu, "gk20a_pmu_allocator alloc failed\n");
			goto clean_up;
		}

		seq->out_gk20a.alloc.dmem.size = out->alloc.dmem.size;
		seq->out_gk20a.alloc.dmem.offset = out->alloc.dmem.offset;
	}

	seq->state = PMU_SEQ_STATE_USED;
	err = gk20a_pmu_write_cmd(priv, cmd, queue_id, timeout);
	if (err)
		seq->state = PMU_SEQ_STATE_PENDING;

	return 0;

clean_up:

	nv_error(pmu, "cmd post failed\n");
	if (in)
		gk20a_pmu_allocator_block_free(&priv->dmem,
					in->alloc.dmem.offset,
					in->alloc.dmem.size,
					PMU_DMEM_ALLOC_ALIGNMENT);
	if (out)
		gk20a_pmu_allocator_block_free(&priv->dmem,
					out->alloc.dmem.offset,
					out->alloc.dmem.size,
					PMU_DMEM_ALLOC_ALIGNMENT);

	gk20a_pmu_seq_release(priv, seq);

	return err;
}

static bool
gk20a_pmu_read_message(struct gk20a_pmu_priv *priv, struct pmu_queue *queue,
			struct pmu_msg *msg, int *status)
{
	struct nvkm_pmu *pmu = &priv->base;
	u32 read_size, bytes_read;
	int err;

	*status = 0;

	if (gk20a_pmu_queue_is_empty(priv, queue))
		return false;

	err = gk20a_pmu_queue_open_read(priv, queue);
	if (err) {
		nv_error(pmu,
			"fail to open queue %d for read\n", queue->id);
		*status = err;
		return false;
	}

	err = gk20a_pmu_queue_pop(priv, queue, &msg->hdr,
			PMU_MSG_HDR_SIZE, &bytes_read);
	if (err || bytes_read != PMU_MSG_HDR_SIZE) {
		nv_error(pmu,
			"fail to read msg from queue %d\n", queue->id);
		*status = err | -EINVAL;
		goto clean_up;
	}

	if (msg->hdr.unit_id == PMU_UNIT_REWIND) {
		gk20a_pmu_queue_rewind(priv, queue);
		/* read again after rewind */
		err = gk20a_pmu_queue_pop(priv, queue, &msg->hdr,
				PMU_MSG_HDR_SIZE, &bytes_read);
		if (err || bytes_read != PMU_MSG_HDR_SIZE) {
			nv_error(pmu,
				"fail to read msg from queue %d\n", queue->id);
			*status = err | -EINVAL;
			goto clean_up;
		}
	}

	if (!PMU_UNIT_ID_IS_VALID(msg->hdr.unit_id)) {
		nv_error(pmu,
			"read invalid unit_id %d from queue %d\n",
			msg->hdr.unit_id, queue->id);
			*status = -EINVAL;
			goto clean_up;
	}

	if (msg->hdr.size > PMU_MSG_HDR_SIZE) {
		read_size = msg->hdr.size - PMU_MSG_HDR_SIZE;
		err = gk20a_pmu_queue_pop(priv, queue, &msg->msg,
			read_size, &bytes_read);
		if (err || bytes_read != read_size) {
			nv_error(pmu,
				"fail to read msg from queue %d\n", queue->id);
			*status = err;
			goto clean_up;
		}
	}

	err = gk20a_pmu_queue_close(priv, queue, true);
	if (err) {
		nv_error(pmu,
			"fail to close queue %d\n", queue->id);
		*status = err;
		return false;
	}

	return true;

clean_up:
	err = gk20a_pmu_queue_close(priv, queue, false);
	if (err)
		nv_error(pmu,
			"fail to close queue %d\n", queue->id);
	return false;

}

static int
gk20a_pmu_response_handle(struct gk20a_pmu_priv *priv, struct pmu_msg *msg)
{
	struct nvkm_pmu *pmu = &priv->base;
	struct pmu_sequence *seq;
	int ret = 0;

	nv_debug(pmu, "handling pmu response\n");
	seq = &priv->seq[msg->hdr.seq_id];
	if (seq->state != PMU_SEQ_STATE_USED &&
	    seq->state != PMU_SEQ_STATE_CANCELLED) {
		nv_error(pmu, "msg for an unknown sequence %d\n", seq->id);
		return -EINVAL;
	}

	if (msg->hdr.unit_id == PMU_UNIT_RC &&
	    msg->msg.rc.msg_type == PMU_RC_MSG_TYPE_UNHANDLED_CMD) {
		nv_error(pmu, "unhandled cmd: seq %d\n", seq->id);
	} else if (seq->state != PMU_SEQ_STATE_CANCELLED) {
		if (seq->msg) {
			if (seq->msg->hdr.size >= msg->hdr.size) {
				memcpy(seq->msg, msg, msg->hdr.size);
				if (seq->out_gk20a.alloc.dmem.size != 0) {
					gk20a_copy_from_dmem(priv,
					seq->out_gk20a.alloc.dmem.offset,
					seq->out_payload,
					seq->out_gk20a.alloc.dmem.size, 0);
				}
			} else {
				nv_error(pmu,
					"sequence %d msg buffer too small\n",
					seq->id);
			}
		}
	} else {
		seq->callback = NULL;
	}

	if (seq->in_gk20a.alloc.dmem.size != 0)
		gk20a_pmu_allocator_block_free(&priv->dmem,
			seq->in_gk20a.alloc.dmem.offset,
			seq->in_gk20a.alloc.dmem.size,
			PMU_DMEM_ALLOC_ALIGNMENT);
	if (seq->out_gk20a.alloc.dmem.size != 0)
		gk20a_pmu_allocator_block_free(&priv->dmem,
			seq->out_gk20a.alloc.dmem.offset,
			seq->out_gk20a.alloc.dmem.size,
			PMU_DMEM_ALLOC_ALIGNMENT);

	if (seq->callback)
		seq->callback(pmu, msg, seq->cb_params, seq->desc, ret);

	gk20a_pmu_seq_release(priv, seq);

	/* TBD: notify client waiting for available dmem */
	nv_debug(pmu, "pmu response processed\n");

	return 0;
}

void
gk20a_init_elcg_mode(struct nvkm_pmu *ppmu, u32 mode, u32 engine)
{
	u32 gate_ctrl;

	gate_ctrl = nv_rd32(ppmu, 0x00020200 + engine * 4);
	gate_ctrl = (gate_ctrl & ~0x3);

	switch (mode) {
	case ELCG_RUN:
		/* set elpg to auto to meet hw expectation */
		gate_ctrl = ((gate_ctrl & ~(0x3 << 4)) | 0x10);
		break;
	case ELCG_STOP:
		gate_ctrl |= 0x2;
		break;
	case ELCG_AUTO:
		gate_ctrl |= 0x1;
		break;
	default:
		nv_error(ppmu, "invalid elcg mode %d\n", mode);
	}

	gate_ctrl &= ~(0x1f << 8);
	gate_ctrl |= 9 << 8;
	gate_ctrl &= ~(0x7 << 13);
	gate_ctrl |= 2 << 13;

	nv_wr32(ppmu, 0x00020200 + engine * 4, gate_ctrl);

	/* default fecs_idle_filter to 0 */
	nv_wr32(ppmu, 0x00020288, 0);
	/* default hubmmu_idle_filter to 0 */
	nv_wr32(ppmu, 0x0002028c, 0);
}

static void
gk20a_pmu_handle_pg_stat_msg(struct nvkm_pmu *pmu, struct pmu_msg *msg,
					void *param, u32 handle, u32 status)
{
	struct gk20a_pmu_priv *priv = param;

	if (status != 0) {
		nv_error(pmu, "ELPG cmd aborted\n");
		/* TBD: disable ELPG */
		return;
	}

	switch (msg->msg.pg.stat.sub_msg_id) {
	case PMU_PG_STAT_MSG_RESP_DMEM_OFFSET:
		nv_trace(pmu, "ALLOC DMEM OFFSET %x\n", msg->msg.pg.stat.data);
		priv->stat_dmem_offset = msg->msg.pg.stat.data;
		break;
	default:
		break;
	}
}

static void
gk20a_pmu_handle_pg_elpg_msg(struct nvkm_pmu *pmu, struct pmu_msg *msg,
			void *param, u32 handle, u32 status)
{
	struct gk20a_pmu_priv *priv = param;
	struct pmu_pg_msg_elpg_msg *elpg_msg = &msg->msg.pg.elpg_msg;

	if (status != 0) {
		nv_error(pmu, "ELPG cmd aborted\n");
		/* TBD: disable ELPG */
		return;
	}

	switch (elpg_msg->msg) {
	case PMU_PG_ELPG_MSG_INIT_ACK:
		nv_trace(pmu, "INIT_ELPG cmd ACK\n");
		break;
	case PMU_PG_ELPG_MSG_ALLOW_ACK:
		nv_trace(pmu, "ELPG_ALLOW cmd ACK\n");
		priv->elpg_stat = PMU_ELPG_STAT_ON;
		complete_all(&priv->elpg_on_completion);
		break;
	case PMU_PG_ELPG_MSG_DISALLOW_ACK:
		nv_trace(pmu, "ELPG_DISALLOW cmd ACK\n");
		priv->elpg_stat = PMU_ELPG_STAT_OFF;
		complete_all(&priv->elpg_off_completion);
		if (priv->pmu_state == PMU_STATE_ELPG_BOOTING) {
			priv->pmu_state = PMU_STATE_ELPG_BOOTED;
			schedule_work(&priv->pg_init);
		}
		break;
	default:
		nv_error(pmu, "wrong ELPG message : 0x%04x\n", elpg_msg->msg);
	}
}

static void
gk20a_pmu_handle_pg_buf_config_msg(struct nvkm_pmu *pmu,
			struct pmu_msg *msg, void *param, u32 handle,
			u32 status)
{
	struct gk20a_pmu_priv *priv = param;
	struct pmu_pg_msg_eng_buf_stat *buf_stat = &msg->msg.pg.eng_buf_stat;

	nv_debug(pmu, "PMU_PG_CMD_ID_ENG_BUF_LOADED cmd ACK\n");
	if (status != 0) {
		nv_error(pmu, "PGENG cmd aborted\n");
		return;
	}

	priv->buf_loaded = (buf_stat->status == PMU_PG_MSG_ENG_BUF_LOADED);

	if ((!priv->buf_loaded) &&
				  (priv->pmu_state == PMU_STATE_LOADING_PG_BUF))
		nv_error(pmu, "failed to load PGENG buffer\n");
	else
		schedule_work(&priv->pg_init);
}

static void
gk20a_pmu_handle_zbc_msg(struct nvkm_pmu *pmu, struct pmu_msg *msg,
					    void *param, u32 handle, u32 status)
{
	struct gk20a_pmu_priv *priv = param;

	nv_debug(pmu, "reply ZBC_TABLE_UPDATE\n");
	schedule_work(&priv->pg_init);
}

static void
gk20a_pmu_save_zbc(struct nvkm_pmu *pmu, u32 entries)
{
	struct gk20a_pmu_priv *priv = to_gk20a_priv(pmu);
	struct pmu_cmd cmd;
	u32 seq;

	if (!priv->pmu_ready || !entries)
		return;

	memset(&cmd, 0, sizeof(struct pmu_cmd));
	cmd.hdr.unit_id = PMU_UNIT_PG;
	cmd.hdr.size = PMU_CMD_HDR_SIZE + sizeof(struct pmu_zbc_cmd);
	cmd.cmd.zbc.cmd_type = PMU_PG_CMD_ID_ZBC_TABLE_UPDATE;
	cmd.cmd.zbc.entry_mask = ZBC_MASK(entries);

	nv_debug(pmu, "cmd post ZBC_TABLE_UPDATE enteries = %d\n", entries);
	gk20a_pmu_cmd_post(pmu, &cmd, NULL, NULL, PMU_COMMAND_QUEUE_HPQ,
			   gk20a_pmu_handle_zbc_msg, priv, &seq, ~0);
}

static int
gk20a_init_pmu_bind_fecs(struct nvkm_pmu *pmu)
{
	struct gk20a_pmu_priv *priv = to_gk20a_priv(pmu);
	struct pmu_cmd cmd;
	u32 desc;

	memset(&cmd, 0, sizeof(struct pmu_cmd));
	cmd.hdr.unit_id = PMU_UNIT_PG;
	cmd.hdr.size = PMU_CMD_HDR_SIZE +
					 sizeof(struct pmu_pg_cmd_eng_buf_load);
	cmd.cmd.pg.eng_buf_load.cmd_type = PMU_PG_CMD_ID_ENG_BUF_LOAD;
	cmd.cmd.pg.eng_buf_load.engine_id = ENGINE_GR_GK20A;
	cmd.cmd.pg.eng_buf_load.buf_idx = PMU_PGENG_GR_BUFFER_IDX_FECS;
	cmd.cmd.pg.eng_buf_load.buf_size = pmu->pg_buf.size;
	cmd.cmd.pg.eng_buf_load.dma_base =
				lower_32_bits(pmu->pg_buf.vma.offset >> 8);
	cmd.cmd.pg.eng_buf_load.dma_offset =
				(u8)(pmu->pg_buf.vma.offset & 0xff);
	cmd.cmd.pg.eng_buf_load.dma_idx = GK20A_PMU_DMAIDX_VIRT;

	priv->buf_loaded = false;
	nv_debug(pmu, "cmd post Load buffer -> PMU_PGENG_GR_BUFFER_IDX_FECS\n");
	gk20a_pmu_cmd_post(pmu, &cmd, NULL, NULL, PMU_COMMAND_QUEUE_LPQ,
			gk20a_pmu_handle_pg_buf_config_msg, priv, &desc, ~0);
	priv->pmu_state = PMU_STATE_LOADING_PG_BUF;

	return 0;
}

static void
gk20a_pmu_setup_hw_load_zbc(struct nvkm_pmu *pmu)
{
	struct gk20a_pmu_priv *priv = to_gk20a_priv(pmu);
	struct pmu_cmd cmd;
	u32 desc;

	memset(&cmd, 0, sizeof(struct pmu_cmd));
	cmd.hdr.unit_id = PMU_UNIT_PG;
	cmd.hdr.size = PMU_CMD_HDR_SIZE +
					 sizeof(struct pmu_pg_cmd_eng_buf_load);
	cmd.cmd.pg.eng_buf_load.cmd_type = PMU_PG_CMD_ID_ENG_BUF_LOAD;
	cmd.cmd.pg.eng_buf_load.engine_id = ENGINE_GR_GK20A;
	cmd.cmd.pg.eng_buf_load.buf_idx = PMU_PGENG_GR_BUFFER_IDX_ZBC;
	cmd.cmd.pg.eng_buf_load.buf_size = GK20A_PMU_SEQ_BUFSIZE;
	cmd.cmd.pg.eng_buf_load.dma_base =
				lower_32_bits(priv->seq_buf.vma.offset >> 8);
	cmd.cmd.pg.eng_buf_load.dma_offset =
				(u8)(priv->seq_buf.vma.offset & 0xff);
	cmd.cmd.pg.eng_buf_load.dma_idx = GK20A_PMU_DMAIDX_VIRT;

	priv->buf_loaded = false;
	nv_debug(pmu, "cmd post Load buffer -> PMU_PGENG_GR_BUFFER_IDX_ZBC\n");
	gk20a_pmu_cmd_post(pmu, &cmd, NULL, NULL, PMU_COMMAND_QUEUE_LPQ,
			gk20a_pmu_handle_pg_buf_config_msg, priv, &desc, ~0);
	priv->pmu_state = PMU_STATE_LOADING_ZBC;
}
static int
gk20a_pmu_elpg_ctrl_locked(struct nvkm_pmu *pmu, bool enable)
{
	struct gk20a_pmu_priv *priv = to_gk20a_priv(pmu);
	struct pmu_cmd cmd;
	u32 seq;

	memset(&cmd, 0, sizeof(struct pmu_cmd));
	cmd.hdr.unit_id = PMU_UNIT_PG;
	cmd.hdr.size = PMU_CMD_HDR_SIZE + sizeof(struct pmu_pg_cmd_elpg_cmd);
	cmd.cmd.pg.elpg_cmd.cmd_type = PMU_PG_CMD_ID_ELPG_CMD;
	cmd.cmd.pg.elpg_cmd.engine_id = ENGINE_GR_GK20A;

	if (enable) {
		reinit_completion(&priv->elpg_on_completion);
		cmd.cmd.pg.elpg_cmd.cmd = PMU_PG_ELPG_CMD_ALLOW;
		priv->elpg_stat = PMU_ELPG_STAT_ON_PENDING;
	} else {
		reinit_completion(&priv->elpg_off_completion);
		cmd.cmd.pg.elpg_cmd.cmd = PMU_PG_ELPG_CMD_DISALLOW;
		priv->elpg_stat = PMU_ELPG_STAT_OFF_PENDING;
	}

	 return gk20a_pmu_cmd_post(pmu, &cmd, NULL, NULL, PMU_COMMAND_QUEUE_HPQ,
			gk20a_pmu_handle_pg_elpg_msg, pmu, &seq, ~0);
}

/**
 * Return: <0 if error, 1 if ELPG is already enabled, 0 for
 * success
*/
int gk20a_pmu_enable_elpg(struct nvkm_pmu *pmu)
{
	struct gk20a_pmu_priv *priv = to_gk20a_priv(pmu);
	int ret;

	mutex_lock(&priv->elpg_mutex);

	if (!priv->elpg_disable_depth) {
		WARN(1, "Trying to enable ELPG without disable.\n");
		ret = -EFAULT;
		goto exit_unlock;
	}

	if (--priv->elpg_disable_depth > 0) {
		ret = 0;
		goto exit_unlock;
	}

	/*
	 * Don't enable elpg until golden ctx is created,
	 * which is related with the ctx that ELPG save and restore.
	 */

	/* return if ELPG is already on or on_pending */
	if (priv->elpg_stat == PMU_ELPG_STAT_ON) {
		ret = 1;
		goto exit_unlock;
	}

	if (priv->elpg_stat == PMU_ELPG_STAT_ON_PENDING)
		goto exit_unlock;

	if (priv->elpg_stat == PMU_ELPG_STAT_OFF_PENDING) {
		ret = wait_for_completion_timeout(&priv->elpg_off_completion,
						msecs_to_jiffies(2000));
		if (!ret)
			nv_error(pmu, "ELPG off timeout\n");

		if (priv->elpg_stat != PMU_ELPG_STAT_OFF) {
			nv_error(pmu, "ELPG_DISALLOW failed\n");
			ret = -EFAULT;
			goto exit_unlock;
		}
	}

	ret = gk20a_pmu_elpg_ctrl_locked(pmu, true);

	if (ret) {
		nv_error(pmu, "ELPG enable failed\n");
		goto exit_unlock;
	}

	ret = wait_for_completion_timeout(&priv->elpg_on_completion,
						msecs_to_jiffies(2000));
	if (!ret) {
		nv_error(pmu, "ELPG on timeout\n");
		ret = -EFAULT;
	} else {
		ret = 0;
	}

exit_unlock:
	mutex_unlock(&priv->elpg_mutex);
	return ret;
}

/**
 * Return: <0 if error, 1 if ELPG is already disabled, 0 for
 * success
*/
int gk20a_pmu_disable_elpg(struct nvkm_pmu *pmu)
{
	struct gk20a_pmu_priv *priv = to_gk20a_priv(pmu);
	int ret;

	mutex_lock(&priv->elpg_mutex);

	if (priv->elpg_disable_depth++ > 0) {
		ret = 0;
		goto exit_unlock;
	}

	if (priv->elpg_stat == PMU_ELPG_STAT_OFF) {
		ret = 1;
		goto exit_unlock;
	}

	if (priv->elpg_stat == PMU_ELPG_STAT_OFF_PENDING)
		goto exit_unlock;

	if (priv->elpg_stat == PMU_ELPG_STAT_ON_PENDING) {
		ret = wait_for_completion_timeout(&priv->elpg_on_completion,
						msecs_to_jiffies(2000));
		if (!ret)
			nv_error(pmu, "ELPG on timeout\n");

		if (priv->elpg_stat != PMU_ELPG_STAT_ON) {
			nv_error(pmu, "ELPG_ALLOW failed, elpg_stat = %d\n",
						priv->elpg_stat);
			ret = -EFAULT;
			goto exit_unlock;
		}
	}

	ret = gk20a_pmu_elpg_ctrl_locked(pmu, false);

	if (ret) {
		nv_error(pmu, "PMU ELPG disable failed\n");
		goto exit_unlock;
	}

	ret = wait_for_completion_timeout(&priv->elpg_off_completion,
							msecs_to_jiffies(2000));
	if (!ret) {
		nv_error(pmu, "ELPG off timeout\n");
		ret = -EFAULT;
	} else {
		ret = 0;
	}

exit_unlock:
	mutex_unlock(&priv->elpg_mutex);
	return ret;
}

static void
gk20a_pmu_setup_hw_enable_elpg(struct nvkm_pmu *pmu)
{
	struct gk20a_pmu_priv *priv = to_gk20a_priv(pmu);
	struct nvkm_gr *gr = nvkm_gr((void *)pmu);
	struct nvkm_ltc *ltc = nvkm_ltc(priv);
	int ret;

	priv->pmu_state = PMU_STATE_STARTED;

	if (gr->wait_idle) {
		ret = gr->wait_idle(gr);
		if (ret)
			nv_error(pmu, "gr_wait_idle failed\n");
	}

	/* Save zbc table after PMU is initialized */
	gk20a_pmu_save_zbc(pmu, ltc->zbc_max);
}

static int
gk20a_pmu_enable_clk_gating(struct nvkm_pmu *pmu)
{
	struct gk20a_pmu_priv *priv = to_gk20a_priv(pmu);
	int ret;

	mutex_lock(&priv->clk_gating_mutex);

	if (--priv->clk_gating_disable_depth > 0) {
		ret = 0;
		goto do_nothing;
	}

	if (pmu->elcg_enabled) {
		gk20a_init_elcg_mode(pmu, ELCG_AUTO, ENGINE_CE2_GK20A);
		gk20a_init_elcg_mode(pmu, ELCG_AUTO, ENGINE_GR_GK20A);
	}

	if (pmu->slcg_enabled) {
		gk20a_enable_load_gating_prod(pmu, gk20a_slcg_gr,
					  ARRAY_SIZE(gk20a_slcg_gr));
		gk20a_enable_load_gating_prod(pmu, gk20a_slcg_therm,
					  ARRAY_SIZE(gk20a_slcg_therm));
		gk20a_enable_load_gating_prod(pmu, gk20a_slcg_perf,
					  ARRAY_SIZE(gk20a_slcg_perf));
	}

	if (pmu->blcg_enabled) {
		gk20a_enable_load_gating_prod(pmu, gk20a_blcg_gr,
					  ARRAY_SIZE(gk20a_blcg_gr));
		gk20a_enable_load_gating_prod(pmu, gk20a_pg_gr,
					  ARRAY_SIZE(gk20a_pg_gr));
	}

do_nothing:
	mutex_unlock(&priv->clk_gating_mutex);
	return 0;
}

static int
gk20a_pmu_disable_clk_gating(struct nvkm_pmu *pmu)
{
	struct gk20a_pmu_priv *priv = to_gk20a_priv(pmu);
	int ret;

	mutex_lock(&priv->clk_gating_mutex);

	if (priv->clk_gating_disable_depth++ > 0) {
		ret = 0;
		goto do_nothing;
	}

	gk20a_init_elcg_mode(pmu, ELCG_RUN, ENGINE_CE2_GK20A);
	gk20a_init_elcg_mode(pmu, ELCG_RUN, ENGINE_GR_GK20A);

	gk20a_disable_load_gating_prod(pmu, gk20a_slcg_gr,
					  ARRAY_SIZE(gk20a_slcg_gr));
	gk20a_disable_load_gating_prod(pmu, gk20a_slcg_therm,
					  ARRAY_SIZE(gk20a_slcg_therm));
	gk20a_disable_load_gating_prod(pmu, gk20a_slcg_perf,
					  ARRAY_SIZE(gk20a_slcg_perf));

	gk20a_disable_load_gating_prod(pmu, gk20a_blcg_gr,
					  ARRAY_SIZE(gk20a_blcg_gr));
	gk20a_disable_load_gating_prod(pmu, gk20a_pg_gr,
					  ARRAY_SIZE(gk20a_pg_gr));

do_nothing:
	mutex_unlock(&priv->clk_gating_mutex);

	return 0;
}

static int
gk20a_pmu_init_powergating(struct nvkm_pmu *pmu)
{
	struct gk20a_pmu_priv *priv = to_gk20a_priv(pmu);
	struct pmu_cmd cmd;
	u32 seq;
	int ret;

	nv_wr32(pmu, 0x0010a6c0 + ENGINE_GR_GK20A * 4,
			PMU_PG_IDLE_THRESHOLD);
	nv_wr32(pmu, 0x0010a6e8 + ENGINE_GR_GK20A * 4,
			PMU_PG_POST_POWERUP_IDLE_THRESHOLD);

	ret = wait_for_completion_timeout(&pmu->gr_init,
						msecs_to_jiffies(2000));
	if (!ret) {
		nv_error(pmu, "Gr init timeout\n");
		return ret;
	}

	nv_debug(pmu, "gr_init done, initialize powergating\n");

	/* Initialize ELCG with disabled mode */
	gk20a_init_elcg_mode(pmu, ELCG_RUN, ENGINE_GR_GK20A);
	gk20a_init_elcg_mode(pmu, ELCG_RUN, ENGINE_CE2_GK20A);

	/* init ELPG */
	memset(&cmd, 0, sizeof(struct pmu_cmd));
	cmd.hdr.unit_id = PMU_UNIT_PG;
	cmd.hdr.size = PMU_CMD_HDR_SIZE + sizeof(struct pmu_pg_cmd_elpg_cmd);
	cmd.cmd.pg.elpg_cmd.cmd_type = PMU_PG_CMD_ID_ELPG_CMD;
	cmd.cmd.pg.elpg_cmd.engine_id = ENGINE_GR_GK20A;
	cmd.cmd.pg.elpg_cmd.cmd = PMU_PG_ELPG_CMD_INIT;

	nv_debug(pmu, "cmd post PMU_PG_ELPG_CMD_INIT\n");
	gk20a_pmu_cmd_post(pmu, &cmd, NULL, NULL, PMU_COMMAND_QUEUE_HPQ,
			gk20a_pmu_handle_pg_elpg_msg, priv, &seq, ~0);
	/* alloc dmem for powergating state log */
	priv->stat_dmem_offset = 0;
	memset(&cmd, 0, sizeof(struct pmu_cmd));
	cmd.hdr.unit_id = PMU_UNIT_PG;
	cmd.hdr.size = PMU_CMD_HDR_SIZE + sizeof(struct pmu_pg_cmd_stat);
	cmd.cmd.pg.stat.cmd_type = PMU_PG_CMD_ID_PG_STAT;
	cmd.cmd.pg.stat.engine_id = ENGINE_GR_GK20A;
	cmd.cmd.pg.stat.sub_cmd_id = PMU_PG_STAT_CMD_ALLOC_DMEM;
	cmd.cmd.pg.stat.data = 0;

	if (priv->pmu_state == PMU_STATE_INIT_RECEIVED)
		priv->pmu_state = PMU_STATE_ELPG_BOOTING;

	priv->elpg_stat = PMU_ELPG_STAT_ON;

	nv_debug(pmu, "cmd post PMU_PG_STAT_CMD_ALLOC_DMEM\n");
	gk20a_pmu_cmd_post(pmu, &cmd, NULL, NULL, PMU_COMMAND_QUEUE_LPQ,
			gk20a_pmu_handle_pg_stat_msg, priv, &seq, ~0);

	/*
	 * Disallow ELPG initially
	 * PMU ucode requires a disallow cmd before allow cmd.
	 */
	gk20a_pmu_disable_elpg(pmu);


	if (priv->pmu_setup_elpg)
		priv->pmu_setup_elpg(pmu);

	return 0;
}

void
gk20a_pmu_setup_hw(struct work_struct *work)
{
	struct gk20a_pmu_priv *priv = container_of(work,
						struct gk20a_pmu_priv, pg_init);
	struct nvkm_pmu *pmu = &priv->base;
	int err;

	switch (priv->pmu_state) {
	case PMU_STATE_INIT_RECEIVED:
		nv_debug(pmu, "PMU INIT received\n");
		err = gk20a_pmu_init_powergating(pmu);
		if (err) {
			nv_error(pmu, "init powergating failed\n");
			return;
		}
		break;
	case PMU_STATE_ELPG_BOOTED:
		nv_debug(pmu, "elpg booted, bind fecs\n");
		gk20a_init_pmu_bind_fecs(pmu);
		break;
	case PMU_STATE_LOADING_PG_BUF:
		nv_debug(pmu, "loaded pg buf\n");
		gk20a_pmu_setup_hw_load_zbc(pmu);
		break;
	case PMU_STATE_LOADING_ZBC:
		gk20a_pmu_setup_hw_enable_elpg(pmu);
		nv_debug(pmu, "loaded zbc\n");
		break;
	case PMU_STATE_STARTED:
		gk20a_pmu_enable_elpg(pmu);
		nv_debug(pmu, "PMU booted\n");
		break;
	default:
		nv_error(pmu, "invalid state\n");
		break;
	}
}

static int
gk20a_pmu_perfmon_start_sampling(struct gk20a_pmu_priv *priv)
{
	struct nvkm_pmu *pmu = &priv->base;
	struct pmu_cmd cmd;
	struct pmu_payload payload;
	u32 seq;
	struct pmu_gk20a_data *pmugk20adata =
				(struct pmu_gk20a_data *)priv->pmu_chip_data;
	struct pmu_perfmon_counter_gk20a *perfcntrgk20a;

	perfcntrgk20a = &pmugk20adata->perfmon_counter_gk20a;

	/* PERFMON Start */
	memset(&cmd, 0, sizeof(cmd));

	cmd.hdr.unit_id = PMU_UNIT_PERFMON;
	cmd.hdr.size = PMU_CMD_HDR_SIZE +
		sizeof(struct pmu_perfmon_cmd_start_gk20a);
	cmd.cmd.perfmon.start_gk20a.cmd_type = PMU_PERFMON_CMD_ID_START;
	cmd.cmd.perfmon.start_gk20a.group_id = PMU_DOMAIN_GROUP_PSTATE;
	cmd.cmd.perfmon.start_gk20a.state_id =
		pmugk20adata->perfmon_state_id[PMU_DOMAIN_GROUP_PSTATE];
	cmd.cmd.perfmon.start_gk20a.flags =
		PMU_PERFMON_FLAG_ENABLE_INCREASE |
		PMU_PERFMON_FLAG_ENABLE_DECREASE |
		PMU_PERFMON_FLAG_CLEAR_PREV;

	memset(&payload, 0, sizeof(struct pmu_payload));

	perfcntrgk20a->upper_threshold = 3000;
	perfcntrgk20a->lower_threshold = 1000;

	perfcntrgk20a->valid = TRUE;
	payload.in.buf = perfcntrgk20a;
	payload.in.size = sizeof(struct pmu_perfmon_counter_gk20a);
	payload.in.offset = offsetof(struct pmu_perfmon_cmd_start_gk20a,
		counter_alloc);

	nv_debug(pmu, "cmd post PMU_PERFMON_CMD_ID_START\n");
	gk20a_pmu_cmd_post(pmu, &cmd, NULL, &payload, PMU_COMMAND_QUEUE_LPQ,
			NULL, NULL, &seq, ~0);

	return 0;
}

static int
gk20a_pmu_handle_perfmon_event(struct gk20a_pmu_priv *priv,
			struct pmu_perfmon_msg *msg)
{

	struct nvkm_pmu *pmu = &priv->base;

	switch (msg->msg_type) {
	case PMU_PERFMON_MSG_ID_INCREASE_EVENT:
		nv_debug(pmu, "perfmon increase event:\n"
			"state_id %d, ground_id %d, pct %d\n",
			msg->gen.state_id, msg->gen.group_id, msg->gen.data);
		priv->perfmon_events_cnt++;
		return 0;
	case PMU_PERFMON_MSG_ID_DECREASE_EVENT:
		nv_debug(pmu, "perfmon decrease event:\n"
			"state_id %d, ground_id %d, pct %d\n",
			msg->gen.state_id, msg->gen.group_id, msg->gen.data);
		priv->perfmon_events_cnt++;
		return 0;
	case PMU_PERFMON_MSG_ID_INIT_EVENT:
		priv->perfmon_ready = 1;
		nv_debug(pmu, "perfmon init event\n");
		break;
	default:
		break;
	}

	/* restart sampling */
	if (priv->perfmon_sampling_enabled)
		return gk20a_pmu_perfmon_start_sampling(priv);

	return 0;
}

static int
gk20a_pmu_handle_event(struct gk20a_pmu_priv *priv, struct pmu_msg *msg)
{
	int err = 0;
	struct nvkm_pmu *pmu = &priv->base;

	switch (msg->hdr.unit_id) {
	case PMU_UNIT_PERFMON:
		err = gk20a_pmu_handle_perfmon_event(priv, &msg->msg.perfmon);
		nv_debug(pmu, "init perfmon event generated\n");
		break;
	default:
		nv_debug(pmu, "default event generated\n");
		break;
	}

	return err;
}

static int
gk20a_pmu_process_init_msg(struct gk20a_pmu_priv *priv, struct pmu_msg *msg)
{
	struct pmu_init_msg_pmu_gk20a *init;
	u32 tail, i;
	struct pmu_gk20a_data *gk20adata = priv->pmu_chip_data;

	tail = nv_rd32(priv, 0x0010a4cc);
	gk20a_copy_from_dmem(priv, tail,
				(u8 *)&msg->hdr, PMU_MSG_HDR_SIZE, 0);

	if (msg->hdr.unit_id != PMU_UNIT_INIT) {
		nv_error(priv, "expecting init msg\n");
		return -EINVAL;
	}

	gk20a_copy_from_dmem(priv, tail + PMU_MSG_HDR_SIZE,
		(u8 *)&msg->msg, msg->hdr.size - PMU_MSG_HDR_SIZE, 0);

	if (msg->msg.init.msg_type != PMU_INIT_MSG_TYPE_PMU_INIT) {
		nv_error(priv, "expecting init msg\n");
		return -EINVAL;
	}

	tail += ALIGN(msg->hdr.size, PMU_DMEM_ALIGNMENT);
	nv_wr32(priv, 0x0010a4cc, tail);
	init = &msg->msg.init.pmu_init_gk20a;
	priv->pmu_ready = true;

	for (i = 0; i < PMU_QUEUE_COUNT; i++)
		gk20a_pmu_queue_init(priv, i, init);

	gk20a_pmu_allocator_init(&priv->dmem, "gk20a_pmu_dmem",
					init->sw_managed_area_offset,
					init->sw_managed_area_size);

	priv->pmu_state = PMU_STATE_INIT_RECEIVED;
	nv_debug(priv, "init msg processed\n");

	gk20adata->perfmon_counter_gk20a.index = 3;
	gk20adata->perfmon_counter_gk20a.group_id = PMU_DOMAIN_GROUP_PSTATE;
	schedule_work(&priv->pg_init);

	return 0;
}

static int
gk20a_pmu_init_perfmon(struct gk20a_pmu_priv *priv)
{
	struct nvkm_pmu *pmu = &priv->base;
	struct pmu_cmd cmd;
	struct pmu_payload payload;
	u32 seq;
	int err = 0;
	struct pmu_gk20a_data *pmugk20adata = (struct pmu_gk20a_data *)
			priv->pmu_chip_data;
	struct pmu_perfmon_counter_gk20a *perfcntrgk20a;

	/* TODO remove this when perfmon is used for pstate/dvfs scaling */
	return 0;

	perfcntrgk20a = &pmugk20adata->perfmon_counter_gk20a;

	if (!priv->sample_buffer)
		gk20a_pmu_allocator_block_alloc(&priv->dmem,
				      &priv->sample_buffer, 2 * sizeof(u16),
				      PMU_DMEM_ALLOC_ALIGNMENT);
	if (err) {
		nv_error(pmu,
			"failed to allocate perfmon sample buffer\n");
		return -ENOMEM;
	}

	priv->perfmon_ready = 0;
	/* use counter #3 for GR && CE2 busy cycles */
	nv_wr32(pmu, 0x0010a504 + PERFMON_BUSY_SLOT * 0x10, 0x200001);

	/* disable idle filtering for counters 3 and 6 */
	nv_mask(pmu, 0x0010a50c + PERFMON_BUSY_SLOT * 0x10, 0x07, 0x02);

	/* use counter #6 for total cycles */
	nv_mask(pmu, 0x0010a50c + PERFMON_CLK_SLOT * 0x10, 0x07, 0x03);

	/*
	 * We don't want to disturb counters #3 and #6, which are used by
	 * perfmon task running in PMU, so we add wiring also
	 * to counters #1 and #2 for exposing raw counter readings.
	 */
	nv_wr32(pmu, 0x0010a504 + NV_CLK_SLOT * 0x10, 0x200001);

	nv_mask(pmu, 0x0010a50c + NV_CLK_SLOT * 0x10, 0x07, 0x02);

	nv_mask(pmu, 0x0010a50c + NV_BUSY_SLOT * 0x10, 0x07, 0x03);

	/* init PERFMON */
	memset(&cmd, 0, sizeof(cmd));

	cmd.hdr.unit_id = PMU_UNIT_PERFMON;
	cmd.hdr.size = PMU_CMD_HDR_SIZE +
		sizeof(struct pmu_perfmon_cmd_init_gk20a);
	cmd.cmd.perfmon.cmd_type = PMU_PERFMON_CMD_ID_INIT;
	/* buffer to save counter values for pmu perfmon */
	cmd.cmd.perfmon.init_gk20a.sample_buffer = (u16)priv->sample_buffer;
	/* number of sample periods below lower threshold
	   before pmu triggers perfmon decrease event */
	cmd.cmd.perfmon.init_gk20a.to_decrease_count = 15;
	/* index of base counter, aka. always ticking counter */
	cmd.cmd.perfmon.init_gk20a.base_counter_id = 6;
	/* microseconds interval between pmu polls perf counters */
	cmd.cmd.perfmon.init_gk20a.sample_period_us = 16700;
	/* number of perfmon counters
	   counter #3 (GR and CE2) for gk20a */
	cmd.cmd.perfmon.init_gk20a.num_counters = 1;
	/* moving average window for sample periods
	   sample_period_us = 17 */
	cmd.cmd.perfmon.init_gk20a.samples_in_moving_avg = 17;

	memset(&payload, 0, sizeof(struct pmu_payload));

	payload.in.buf = perfcntrgk20a;
	payload.in.size = sizeof(struct pmu_perfmon_counter_gk20a);
	payload.in.offset = offsetof(struct pmu_perfmon_cmd_start_gk20a,
		counter_alloc);

	nv_debug(pmu,
		"payload in=%p, in_size=%d, in_offset=%d,\n"
		"payload out=%p, out_size=%d, out_offset=%d\n",
		&payload.in, payload.in.size, payload.in.offset,
		&payload.out, payload.out.size, payload.out.offset);
	nv_debug(pmu, "cmd post PMU_PERFMON_CMD_ID_INIT\n");
	gk20a_pmu_cmd_post(pmu, &cmd, NULL, &payload, PMU_COMMAND_QUEUE_LPQ,
			NULL, NULL, &seq, ~0);

	return 0;
}

void
gk20a_pmu_process_message(struct work_struct *work)
{
	struct nvkm_pmu *pmu = container_of(work, struct nvkm_pmu, recv.work);
	struct gk20a_pmu_priv *priv = to_gk20a_priv(pmu);
	struct pmu_msg msg;
	int status;
	struct nvkm_mc *pmc = nvkm_mc(pmu);

	mutex_lock(&priv->isr_mutex);
	if (unlikely(!priv->pmu_ready)) {
		nv_debug(pmu, "processing init msg\n");
		gk20a_pmu_process_init_msg(priv, &msg);
		gk20a_pmu_init_perfmon(priv);
		if (pmu->fecs_secure_boot)
			gm20b_pmu_init_acr(pmu);

		goto out;
	}
	while (gk20a_pmu_read_message(priv,
		&priv->queue[PMU_MESSAGE_QUEUE], &msg, &status)) {

		nv_debug(pmu, "read msg hdr:\n"
				"unit_id = 0x%08x, size = 0x%08x,\n"
				"ctrl_flags = 0x%08x, seq_id = 0x%08x\n",
				msg.hdr.unit_id, msg.hdr.size,
				msg.hdr.ctrl_flags, msg.hdr.seq_id);

		msg.hdr.ctrl_flags &= ~PMU_CMD_FLAGS_PMU_MASK;

		if (msg.hdr.ctrl_flags == PMU_CMD_FLAGS_EVENT)
			gk20a_pmu_handle_event(priv, &msg);
		else
			gk20a_pmu_response_handle(priv, &msg);
	}
out:
	mutex_unlock(&priv->isr_mutex);
	gk20a_pmu_enable_irq(priv, pmc, true);
	nv_debug(pmu, "exit %s\n", __func__);
}

static int
gk20a_pmu_init_vm(struct gk20a_pmu_priv *priv, const struct firmware *fw)
{
	int ret = 0;
	u32 *ucode_image;
	struct pmu_ucode_desc *desc = (struct pmu_ucode_desc *)fw->data;
	struct nvkm_pmu_priv_vm *pmuvm = &priv->pmuvm;
	struct nvkm_device *device = nv_device(&priv->base);
	struct nvkm_vm *vm;
	struct nvkm_pmu *pmu = &priv->base;
	struct nvkm_mmu *mmu = nvkm_mmu(priv);
	const u64 pmu_area_len = 300*1024;

	/* allocate inst blk */
	ret = nvkm_gpuobj_new(nv_object(priv), NULL, 0x1000, 0, 0, &pmuvm->mem);
	if (ret)
		return ret;

	/* allocate pgd and initialize inst blk */
	ret = mmu->create_pgd(mmu, nv_object(priv), pmuvm->mem,
				pmu_area_len, &pmuvm->pgd);
	if (ret)
		goto err_pgd;

	/* allocate virtual memory range */
	ret = nvkm_vm_new(device, 0, pmu_area_len, 0, &vm);
	if (ret)
		goto err_vm;

	atomic_inc(&vm->engref[NVDEV_SUBDEV_PMU]);

	/* update VM with pgd */
	ret = nvkm_vm_ref(vm, &pmuvm->vm, pmuvm->pgd);
	if (ret)
		goto err_ref;

	/* allocate memory for pmu fw to be copied to*/
	ret = nvkm_gpuobj_new(nv_object(priv), NULL, GK20A_PMU_UCODE_SIZE_MAX,
			      0x1000, 0, &priv->ucode.obj);
	if (ret)
		goto err_fw;

	ucode_image = (u32 *)((u8 *)desc + desc->descriptor_size);
	gpu_obj_memwr(priv->ucode.obj, 0,
			ucode_image, desc->app_start_offset + desc->app_size);

	/* map allocated memory into GMMU */
	ret = nvkm_gpuobj_map_vm(priv->ucode.obj, vm, NV_MEM_ACCESS_RW,
				 &priv->ucode.vma);
	if (ret)
		goto err_map;

	pmu->pmu_vm = pmuvm;
	return 0;

err_map:
	nvkm_gpuobj_destroy(priv->ucode.obj);
	priv->ucode.obj = NULL;
err_fw:
	nvkm_vm_ref(NULL, &vm, pmuvm->pgd);
err_ref:
	nvkm_vm_ref(NULL, &vm, NULL);
err_vm:
	nvkm_gpuobj_destroy(pmuvm->pgd);
	pmuvm->pgd = NULL;
err_pgd:
	nvkm_gpuobj_destroy(pmuvm->mem);
	pmuvm->mem = NULL;
	return ret;
}

static int
gk20a_init_pmu_setup_sw(struct gk20a_pmu_priv *priv)
{
	struct nvkm_pmu_priv_vm *pmuvm = &priv->pmuvm;
	struct nvkm_pmu *pmu = &priv->base;
	int ret = 0, i;

	INIT_WORK(&priv->base.recv.work, gk20a_pmu_process_message);
	priv->mutex_cnt = MUTEX_CNT;
	priv->mutex = kzalloc(priv->mutex_cnt *
		sizeof(struct pmu_mutex), GFP_KERNEL);

	if (!priv->mutex) {
		nv_error(pmu, "not enough space ENOMEM\n");
		return -ENOMEM;
	}

	for (i = 0; i < priv->mutex_cnt; i++)
		priv->mutex[i].index = i;

	priv->seq = kzalloc(PMU_MAX_NUM_SEQUENCES *
		sizeof(struct pmu_sequence), GFP_KERNEL);

	if (!priv->seq) {
		nv_error(pmu, "not enough space ENOMEM\n");
		kfree(priv->mutex);
		return -ENOMEM;
	}

	gk20a_pmu_seq_init(priv);
	INIT_WORK(&priv->pg_init, gk20a_pmu_setup_hw);

	/* TODO remove this when perfmon is used for pstate/dvfs scaling */
	priv->perfmon_sampling_enabled = false;

	ret = nvkm_gpuobj_new(nv_object(priv), NULL, GK20A_PMU_TRACE_BUFSIZE,
					    0, 0, &priv->trace_buf.obj);
	if (ret)
		goto error;

	ret = nvkm_gpuobj_map_vm(nv_gpuobj(priv->trace_buf.obj), pmuvm->vm,
					NV_MEM_ACCESS_RW, &priv->trace_buf.vma);
	if (ret)
		goto error;

	ret = nvkm_gpuobj_new(nv_object(priv), NULL, GK20A_PMU_SEQ_BUFSIZE,
					    0, 0, &priv->seq_buf.obj);
	if (ret)
		goto error;

	ret = nvkm_gpuobj_map_vm(nv_gpuobj(priv->seq_buf.obj), pmuvm->vm,
					NV_MEM_ACCESS_RW, &priv->seq_buf.vma);
	if (ret)
		goto error;

	return 0;
error:
	kfree(priv->mutex);
	kfree(priv->seq);

	return ret;
}

static int
gk20a_pmu_bootstrap(struct gk20a_pmu_priv *priv)
{
	struct pmu_ucode_desc *desc = priv->desc;
	u32 addr_code, addr_data, addr_load;
	u32 i, blocks, addr_args;
	struct pmu_cmdline_args_gk20a cmdline_args;
	struct nvkm_pmu_priv_vm *pmuvm = &priv->pmuvm;

	nv_mask(priv, 0x0010a048, 0x01, 0x01);
	/*bind the address*/
	nv_wr32(priv, 0x0010a480,
		pmuvm->mem->addr >> 12 |
		0x1 << 30 |
		0x20000000);

	/* TBD: load all other surfaces */
	cmdline_args.falc_trace_size = GK20A_PMU_TRACE_BUFSIZE;
	cmdline_args.falc_trace_dma_base =
			    lower_32_bits(priv->trace_buf.vma.offset >> 8);
	cmdline_args.falc_trace_dma_idx = GK20A_PMU_DMAIDX_VIRT;
	cmdline_args.cpu_freq_hz = 204;
	cmdline_args.secure_mode = 0;

	addr_args = (nv_rd32(priv, 0x0010a108) >> 9) & 0x1ff;
	addr_args = addr_args << GK20A_PMU_DMEM_BLKSIZE2;
	addr_args -= sizeof(struct pmu_cmdline_args_gk20a);
	nv_debug(priv, "initiating copy to dmem\n");
	gk20a_pmu_copy_to_dmem(priv, addr_args,
			(u8 *)&cmdline_args,
			sizeof(struct pmu_cmdline_args_gk20a), 0);

	nv_wr32(priv, 0x0010a1c0, 0x1 << 24);

	addr_code = lower_32_bits((priv->ucode.vma.offset +
			desc->app_start_offset +
			desc->app_resident_code_offset) >> 8);

	addr_data = lower_32_bits((priv->ucode.vma.offset +
			desc->app_start_offset +
			desc->app_resident_data_offset) >> 8);

	addr_load = lower_32_bits((priv->ucode.vma.offset +
			desc->bootloader_start_offset) >> 8);

	nv_wr32(priv, 0x0010a1c4, GK20A_PMU_DMAIDX_UCODE);
	nv_debug(priv, "0x%08x\n", GK20A_PMU_DMAIDX_UCODE);
	nv_wr32(priv, 0x0010a1c4, (addr_code));
	nv_debug(priv, "0x%08x\n", (addr_code));
	nv_wr32(priv, 0x0010a1c4, desc->app_size);
	nv_debug(priv, "0x%08x\n", desc->app_size);
	nv_wr32(priv, 0x0010a1c4, desc->app_resident_code_size);
	nv_debug(priv, "0x%08x\n", desc->app_resident_code_size);
	nv_wr32(priv, 0x0010a1c4, desc->app_imem_entry);
	nv_debug(priv, "0x%08x\n", desc->app_imem_entry);
	nv_wr32(priv, 0x0010a1c4,  (addr_data));
	nv_debug(priv, "0x%08x\n", (addr_data));
	nv_wr32(priv, 0x0010a1c4, desc->app_resident_data_size);
	nv_debug(priv, "0x%08x\n", desc->app_resident_data_size);
	nv_wr32(priv, 0x0010a1c4, (addr_code));
	nv_debug(priv, "0x%08x\n", (addr_code));
	nv_wr32(priv, 0x0010a1c4, 0x1);
	nv_debug(priv, "0x%08x\n", 1);
	nv_wr32(priv, 0x0010a1c4, addr_args);
	nv_debug(priv, "0x%08x\n", addr_args);

	nv_wr32(priv, 0x0010a110,
		(addr_load) - (desc->bootloader_imem_offset >> 8));

	blocks = ((desc->bootloader_size + 0xFF) & ~0xFF) >> 8;

	for (i = 0; i < blocks; i++) {
		nv_wr32(priv, 0x0010a114,
			desc->bootloader_imem_offset + (i << 8));
		nv_wr32(priv, 0x0010a11c,
			desc->bootloader_imem_offset + (i << 8));
		nv_wr32(priv, 0x0010a118,
			0x01 << 4  |
			0x06 << 8  |
			((GK20A_PMU_DMAIDX_UCODE & 0x07) << 12));
	}

	nv_wr32(priv, 0x0010a104, (desc->bootloader_entry_point));
	nv_wr32(priv, 0x0010a100, 0x1 << 1);
	nv_wr32(priv, 0x0010a080, desc->app_version);

	return 0;
}

static int
gk20a_init_pmu_setup_hw1(struct gk20a_pmu_priv *priv, struct nvkm_mc *pmc)
{
	int err;

	mutex_lock(&priv->isr_mutex);
	err = gk20a_pmu_enable(priv, pmc, true);
	priv->isr_enabled = (err == 0);
	mutex_unlock(&priv->isr_mutex);
	if (err)
		return err;

	/* setup apertures - virtual */
	nv_wr32(priv, 0x10a600 + 0 * 4, 0x0);
	nv_wr32(priv, 0x10a600 + 1 * 4, 0x0);
	/* setup apertures - physical */
	nv_wr32(priv, 0x10a600 + 2 * 4, 0x4 | 0x0);
	nv_wr32(priv, 0x10a600 + 3 * 4, 0x4 | 0x1);
	nv_wr32(priv, 0x10a600 + 4 * 4, 0x4 | 0x2);

	/* TBD: load pmu ucode */
	err = gk20a_pmu_bootstrap(priv);
	if (err)
		return err;

	return 0;
}

void
gk20a_pmu_intr(struct nvkm_subdev *subdev)
{
	struct gk20a_pmu_priv *priv = to_gk20a_priv(nvkm_pmu(subdev));
	struct nvkm_mc *pmc = nvkm_mc(priv);
	u32 intr, mask;

	if (!priv->isr_enabled)
		return;

	mask = nv_rd32(priv, 0x0010a018) & nv_rd32(priv, 0x0010a01c);

	intr = nv_rd32(priv, 0x0010a008) & mask;

	nv_debug(priv, "received falcon interrupt: 0x%08x\n", intr);
	gk20a_pmu_enable_irq(priv, pmc, false);

	if (!intr || priv->pmu_state == PMU_STATE_OFF) {
		nv_wr32(priv, 0x0010a004, intr);
		nv_error(priv, "pmu state off\n");
		gk20a_pmu_enable_irq(priv, pmc, true);
	}

	if (intr & 0x10)
		nv_error(priv, "pmu halt intr not implemented\n");

	if (intr & 0x20) {
		nv_error(priv, "exterr interrupt  not impl..Clear interrupt\n");
		nv_mask(priv, 0x0010a16c, (0x1 << 31), 0x00000000);
	}

	if (intr & 0x40) {
		nv_debug(priv, "scheduling work\n");
		schedule_work(&priv->base.recv.work);
		gk20a_pmu_enable_irq(priv, pmc, true);
	}

	nv_wr32(priv, 0x0010a004, intr);
	nv_debug(priv, "irq handled\n");
}

static void
gk20a_pmu_pgob(struct nvkm_pmu *pmu, bool enable)
{
}

static int elpg_stats_show(struct seq_file *s, void *data)
{
	struct gk20a_pmu_priv *priv = s->private;
	struct nvkm_pmu *pmu = &priv->base;
	struct device *dev = nv_device_base(nv_device(priv));
	struct pmu_pg_stats stats;
	int ret;

	if (!priv->pmu_ready) {
		nv_error(pmu, "PMU not ready\n");
		return -EAGAIN;
	}

	ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		nv_error(pmu, "power up gpu failed: %d\n", ret);
		return ret;
	}

	gk20a_copy_from_dmem(priv, priv->stat_dmem_offset,
		(u8 *)&stats, sizeof(struct pmu_pg_stats), 0);

	seq_printf(s, "pg_entry_start_timestamp : 0x%016llx\n",
		stats.pg_entry_start_timestamp);
	seq_printf(s, "pg_exit_start_timestamp : 0x%016llx\n",
		stats.pg_exit_start_timestamp);
	seq_printf(s, "pg_ingating_start_timestamp : 0x%016llx\n",
		stats.pg_ingating_start_timestamp);
	seq_printf(s, "pg_ungating_start_timestamp : 0x%016llx\n",
		stats.pg_ungating_start_timestamp);
	seq_printf(s, "pg_avg_entry_time_us : 0x%08x\n",
		stats.pg_avg_entry_time_us);
	seq_printf(s, "pg_avg_exit_time_us : 0x%08x\n",
		stats.pg_avg_exit_time_us);
	seq_printf(s, "pg_ingating_cnt : 0x%08x\n",
		stats.pg_ingating_cnt);
	seq_printf(s, "pg_ingating_time_us : 0x%08x\n",
		stats.pg_ingating_time_us);
	seq_printf(s, "pg_ungating_count : 0x%08x\n",
		stats.pg_ungating_count);
	seq_printf(s, "pg_ungating_time_us 0x%08x:\n",
		stats.pg_ungating_time_us);
	seq_printf(s, "pg_gating_cnt : 0x%08x\n",
		stats.pg_gating_cnt);
	seq_printf(s, "pg_gating_deny_cnt : 0x%08x\n",
		stats.pg_gating_deny_cnt);

	seq_printf(s, "pmu_pmu_idle_mask_supp_r(3): 0x%08x\n",
		nv_rd32(pmu, 0x0010a9f0 + 3 * 8));
	seq_printf(s, "pmu_pmu_idle_mask_1_supp_r(3): 0x%08x\n",
		nv_rd32(pmu, 0x0010a9f4 + 3 * 8));
	seq_printf(s, "pmu_pmu_idle_ctrl_supp_r(3): 0x%08x\n",
		nv_rd32(pmu, 0x0010aa30 + 3*8));
	seq_printf(s, "pmu_pmu_pg_idle_cnt_r(0): 0x%08x\n",
		nv_rd32(pmu, 0x0010a710));
	seq_printf(s, "pmu_pmu_pg_intren_r(0): 0x%08x\n",
		nv_rd32(pmu, 0x0010a760));

	seq_printf(s, "pmu_pmu_idle_count_r(3): 0x%08x\n",
		nv_rd32(pmu, 0x0010a508 + 3 * 16));
	seq_printf(s, "pmu_pmu_idle_count_r(4): 0x%08x\n",
		nv_rd32(pmu, 0x0010a508 + 4 * 16));
	seq_printf(s, "pmu_pmu_idle_count_r(7): 0x%08x\n",
		nv_rd32(pmu, 0x0010a508 + 7 * 16));
	seq_printf(s, "PG threshold value: 0x%08x\n\n\n",
		nv_rd32(pmu, 0x0010a6c0 + ENGINE_GR_GK20A * 4));

	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);
	return 0;
}

static int elpg_stats_open(struct inode *inode, struct file *file)
{
	return single_open(file, elpg_stats_show, inode->i_private);
}

static const struct file_operations elpg_stats_fops = {
	.open		= elpg_stats_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int allow_elpg_show(struct seq_file *s, void *data)
{
	struct gk20a_pmu_priv *priv = s->private;

	seq_printf(s, "%d\n", priv->allow_elpg);
	return 0;
}

static int allow_elpg_open(struct inode *inode, struct file *file)
{
	return single_open(file, allow_elpg_show, inode->i_private);
}

static ssize_t allow_elpg_write(struct file *file,
	const char __user *userbuf, size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct gk20a_pmu_priv *priv = s->private;
	struct nvkm_pmu *pmu = &priv->base;
	struct device *dev = nv_device_base(nv_device(priv));
	unsigned long val = 0;
	int ret;

	if (kstrtoul_from_user(userbuf, count, 0, &val) < 0)
		return -EINVAL;

	ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		nv_error(pmu, "power up gpu failed: %d\n", ret);
		return ret;
	}

	/* Don't turn on gk20a unnecessarily */
	mutex_lock(&priv->allow_elpg_mutex);
	if (val && !priv->allow_elpg)
		gk20a_pmu_enable_elpg(pmu);
	else if (!val && priv->allow_elpg)
		gk20a_pmu_disable_elpg(pmu);
	priv->allow_elpg = val;
	mutex_unlock(&priv->allow_elpg_mutex);

	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);
	return count;
}

static const struct file_operations allow_elpg_fops = {
	.open		= allow_elpg_open,
	.read		= seq_read,
	.write		= allow_elpg_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

void
gk20a_pmu_debugfs_unregister(struct gk20a_pmu_priv *priv)
{
	debugfs_remove_recursive(priv->dbgfs_dir);
}

int
gk20a_pmu_debugfs_register(struct gk20a_pmu_priv *priv)
{
	struct dentry *d;

	priv->dbgfs_dir = debugfs_create_dir("PMU", NULL);
	if (!priv->dbgfs_dir) {
		pr_err("%s: Failed to make debugfs node\n", __func__);
		return -ENOMEM;
	}

	d = debugfs_create_file(
		"elpg_stats", 0644, priv->dbgfs_dir, priv, &elpg_stats_fops);
	if (!d) {
		pr_err("elpg_stats register failed\n");
		gk20a_pmu_debugfs_unregister(priv);
		return -ENODEV;
	}

	d = debugfs_create_file(
		"allow_elpg", S_IRUGO | S_IWUSR, priv->dbgfs_dir, priv,
				&allow_elpg_fops);
	if (!d) {
		pr_err("allow_elpg register failed\n");
		gk20a_pmu_debugfs_unregister(priv);
		return -ENODEV;
	}

	return 0;
}

static int
gk20a_pmu_init(struct nvkm_object *object)
{
	struct gk20a_pmu_priv *priv = (void *)object;
	struct nvkm_mc *pmc = nvkm_mc(object);
	struct nvkm_pmu *pmu = &priv->base;
	int ret;

	ret = nvkm_subdev_init(&priv->base.base);
	if (ret)
		return ret;

	reinit_completion(&pmu->gr_init);
	priv->pmu_state = PMU_STATE_STARTING;
	ret = gk20a_init_pmu_setup_hw1(priv, pmc);
	if (ret)
		return ret;

	priv->out_of_reset = true;

	gk20a_pmu_dvfs_init(priv);
	pmu->fecs_secure_boot = false;
	pmu->elcg_enabled = true;
	pmu->slcg_enabled = true;
	pmu->blcg_enabled = true;

	priv->pmu_setup_elpg = NULL;

	pmu->enable_clk_gating = gk20a_pmu_enable_clk_gating;
	pmu->disable_clk_gating = gk20a_pmu_disable_clk_gating;

	nvkm_timer_alarm(priv, PMU_DVFS_INTERVAL, &priv->alarm);

	mutex_lock(&priv->elpg_mutex);
	/*
	 * ELPG will be enabled when PMU finishes booting, so setting the
	 * counter to 1 initialy.
	 */
	priv->elpg_disable_depth = 1;
	mutex_unlock(&priv->elpg_mutex);

	mutex_lock(&priv->clk_gating_mutex);
	priv->clk_gating_disable_depth = 0;
	mutex_unlock(&priv->clk_gating_mutex);

	return ret;
}

static int
gk20a_pmu_fini(struct nvkm_object *object, bool suspend)
{
	struct gk20a_pmu_priv *priv = (void *)object;
	struct nvkm_pmu *pmu = (void *)object;
	struct nvkm_mc *pmc = nvkm_mc(object);

	nvkm_timer_alarm_cancel(priv, &priv->alarm);

	cancel_work_sync(&priv->base.recv.work);
	cancel_work_sync(&priv->pg_init);

	mutex_lock(&priv->clk_gating_mutex);
	priv->clk_gating_disable_depth = 0;
	mutex_unlock(&priv->clk_gating_mutex);

	mutex_lock(&priv->isr_mutex);
	gk20a_pmu_enable(priv, pmc, false);
	priv->isr_enabled = false;
	mutex_unlock(&priv->isr_mutex);

	priv->pmu_state = PMU_STATE_OFF;
	priv->pmu_ready = false;
	priv->out_of_reset = false;

	pmu->elcg_enabled = false;
	pmu->slcg_enabled = false;
	pmu->blcg_enabled = false;

	nv_wr32(priv, 0x10a014, 0x00000060);

	return nvkm_subdev_fini(&priv->base.base, suspend);
}

static void
gk20a_pmu_dtor(struct nvkm_object *object)
{
	struct gk20a_pmu_priv *priv = (void *)object;

	nvkm_gpuobj_unmap(&priv->seq_buf.vma);
	nvkm_gpuobj_ref(NULL, &priv->seq_buf.obj);
	nvkm_gpuobj_unmap(&priv->trace_buf.vma);
	nvkm_gpuobj_ref(NULL, &priv->trace_buf.obj);

	nvkm_gpuobj_unmap(&priv->ucode.vma);
	nvkm_gpuobj_ref(NULL, &priv->ucode.obj);
	nvkm_vm_ref(NULL, &priv->pmuvm.vm, priv->pmuvm.pgd);
	nvkm_gpuobj_ref(NULL, &priv->pmuvm.pgd);
	nvkm_gpuobj_ref(NULL, &priv->pmuvm.mem);
	gk20a_pmu_allocator_destroy(&priv->dmem);
	kfree(priv->pmu_chip_data);
	gk20a_pmu_debugfs_unregister(priv);
	mutex_destroy(&priv->allow_elpg_mutex);
}

struct gk20a_pmu_dvfs_data gk20a_dvfs_data = {
	.p_load_target = 40,
	.p_load_max = 78,
	.p_smooth = 5,
};

static int
gk20a_pmu_ctor(struct nvkm_object *parent, struct nvkm_object *engine,
	       struct nvkm_oclass *oclass, void *data, u32 size,
	       struct nvkm_object **pobject)
{
	struct gk20a_pmu_priv *priv;
	struct nvkm_pmu *pmu;
	struct nvkm_mc *pmc;
	const struct firmware *pmufw = NULL;
	int ret;

	ret = nvkm_pmu_create(parent, engine, oclass, &priv);
	*pobject = nv_object(priv);
	if (ret)
		return ret;

	mutex_init(&priv->isr_mutex);
	mutex_init(&priv->pmu_copy_lock);
	mutex_init(&priv->pmu_seq_lock);
	mutex_init(&priv->allow_elpg_mutex);
	mutex_init(&priv->elpg_mutex);
	mutex_init(&priv->clk_gating_mutex);
	priv->data = &gk20a_dvfs_data;
	pmu = &priv->base;
	init_completion(&pmu->gr_init);
	pmc = nvkm_mc(pmu);
	nv_subdev(pmu)->intr = gk20a_pmu_intr;
	pmu->enable_elpg = gk20a_pmu_enable_elpg;
	pmu->disable_elpg = gk20a_pmu_disable_elpg;
	priv->allow_elpg = true;
	init_completion(&priv->elpg_off_completion);
	init_completion(&priv->elpg_on_completion);

	priv->pmu_chip_data = kzalloc(sizeof(struct pmu_gk20a_data),
			GFP_KERNEL);
	if (!priv->pmu_chip_data) {
		nv_error(priv, "failed to allocate mem for chip data\n");
		return -ENOMEM;
	}

	ret = gk20a_load_firmware(pmu, &pmufw, GK20A_PMU_UCODE_IMAGE);
	if (ret < 0) {
		nv_error(priv, "failed to load pmu fimware\n");
		return ret;
	}

	ret = gk20a_pmu_init_vm(priv, pmufw);
	if (ret < 0) {
		nv_error(priv, "failed to map pmu fw to va space\n");
		goto err;
	}

	priv->desc = (struct pmu_ucode_desc *)pmufw->data;
	gk20a_pmu_dump_firmware_info(pmu, pmufw);

	if (priv->desc->app_version != APP_VERSION_GK20A) {
		nv_error(priv, "PMU version unsupported: %d\n",
						       priv->desc->app_version);
		ret = -EINVAL;
		goto err;
	}

	ret = gk20a_init_pmu_setup_sw(priv);
	if (ret)
		goto err;

	ret = gk20a_pmu_debugfs_register(priv);
	if (ret)
		goto err;

	pmu->pgob = nvkm_pmu_pgob;
	nvkm_alarm_init(&priv->alarm, gk20a_pmu_dvfs_work);

	return 0;

err:
	gk20a_release_firmware(pmu, pmufw);
	return ret;
}

struct nvkm_oclass *
gk20a_pmu_oclass = &(struct nvkm_pmu_impl) {
	.base.handle = NV_SUBDEV(PMU, 0xea),
	.base.ofuncs = &(struct nvkm_ofuncs) {
		.ctor = gk20a_pmu_ctor,
		.dtor = gk20a_pmu_dtor,
		.init = gk20a_pmu_init,
		.fini = gk20a_pmu_fini,
	},
	.pgob = gk20a_pmu_pgob,
	.acquire_mutex = gk20a_pmu_mutex_acquire,
	.release_mutex = gk20a_pmu_mutex_release,
}.base;

