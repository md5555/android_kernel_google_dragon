/*
 * Copyright (C) 2014 Sergey Senozhatsky.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/lz4.h>

#include "zcomp_lz4.h"

static void *zcomp_lz4_create(void)
{
	void *buf;
	size_t size = LZ4_MEM_COMPRESS;

	/* fallback to vmalloc allocation */
	buf = kzalloc(size, GFP_KERNEL | __GFP_NORETRY | __GFP_NOWARN);
	if (!buf && size > PAGE_SIZE) {
		buf = vmalloc(size);
		memset(buf, 0, size);
	}
	return buf;
}

static void zcomp_lz4_destroy(void *private)
{
	kvfree(private);
}

static int zcomp_lz4_compress(const unsigned char *src, unsigned char *dst,
		size_t *dst_len, void *private)
{
	/* return  : Success if return 0 */
	return lz4_compress(src, PAGE_SIZE, dst, dst_len, private);
}

static int zcomp_lz4_decompress(const unsigned char *src, size_t src_len,
		unsigned char *dst)
{
	size_t dst_len = PAGE_SIZE;
	/* return  : Success if return 0 */
	return lz4_decompress_unknownoutputsize(src, src_len, dst, &dst_len);
}

struct zcomp_backend zcomp_lz4 = {
	.compress = zcomp_lz4_compress,
	.decompress = zcomp_lz4_decompress,
	.create = zcomp_lz4_create,
	.destroy = zcomp_lz4_destroy,
	.name = "lz4",
};
