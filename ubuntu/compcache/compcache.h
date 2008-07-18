/*
 * Compressed RAM based swap device
 *
 * (C) 2008 Nitin Gupta <nitingupta910@gmail.com>
 *
 * This RAM based block device acts as swap disk.
 * Pages swapped to this device are compressed and
 * stored in memory.
 *
 * Released under the terms of the GNU General Public
 * License (version 2). See linux/COPYING for more information.
 *
 * Project home: http://code.google.com/p/compcache
 */

#ifndef _COMPCACHE_H_
#define _COMPCACHE_H_

#define SECTOR_SHIFT		9
#define SECTOR_SIZE		(1 << SECTOR_SHIFT)
#define SECTORS_PER_PAGE_SHIFT	(PAGE_SHIFT - SECTOR_SHIFT)
#define SECTORS_PER_PAGE	(1 << SECTORS_PER_PAGE_SHIFT)

/*-- Configurable parameters */
/* Default compcache size: 25% of total RAM */
#define DEFAULT_COMPCACHE_PERCENT	25
#define INIT_SIZE_BYTES			(16 * 1024)
#define GROW_SIZE_BYTES			INIT_SIZE_BYTES
/*-- */

/* Message prefix */
#define C "compcache: "

/* Debugging and Stats */
#define NOP	do { } while(0)

#if defined(CONFIG_BLK_DEV_COMPCACHE_DEBUG)
#define DEBUG
#endif

#if defined(CONFIG_BLK_DEV_COMPCACHE_STATS)
#define STATS
#endif

#if defined(STATS)
#define stat_inc(stat)			(stat++)
#define stat_dec(stat)			(stat--)
#define stat_set(stat, val)		(stat = val)
#define stat_setmax(stat, curr)		(stat = (curr) > stat ? (curr) : stat) 
#define stat_inc_if_less(stat, val1, val2) \
					(stat += ((val1) < (val2) ? 1 : 0))
#else	/* STATS */
#define stat_inc(x)			NOP
#define stat_dec(x)			NOP
#define stat_set(x, v)			NOP
#define stat_setmax(x, v)		NOP
#define stat_inc_if_less(x, v1, v2)	NOP
#endif	/* STATS */

/*-- Data structures */
/* Indexed by page no. */
struct table {
	void *addr;
	unsigned long len;
};

struct compcache {
	void *mem_pool;
	void *compress_workmem;
	void *compress_buffer;
	struct table *table;
	struct mutex lock;
	struct gendisk *disk;
	size_t size;            /* In sectors */
};

#if defined(STATS)
struct compcache_stats {
	u32 num_reads;		/* failed + successful */
	u32 num_writes;		/* --do-- */
	u32 failed_reads;	/* can happen when memory is tooo low */
	u32 failed_writes;	/* should NEVER! happen */
	u32 invalid_io;		/* non-swap I/O requests */
	u32 good_compress;	/* no. of pages with compression
				 * ratio <= 50%. TODO: export full
				 * compressed page size histogram */
	u32 pages_expand;	/* no. of incompressible pages */
	u32 notify_free;	/* count of swap entry free notifications */
	size_t curr_pages;	/* current no. of compressed pages */
	size_t curr_mem;	/* current total size of compressed pages */
	size_t peak_mem;
};
#endif
/*-- */

#endif
