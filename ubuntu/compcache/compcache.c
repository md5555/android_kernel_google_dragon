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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/blkdev.h>
#include <linux/buffer_head.h>
#include <linux/device.h>
#include <linux/genhd.h>
#include <linux/highmem.h>
#include <linux/lzo.h>
#include <linux/mutex.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <linux/swap.h>
#include <linux/swapops.h>
#include <linux/tlsf.h>
#include <linux/vmalloc.h>

#include "compcache.h"

static struct block_device_operations compcache_devops = {
	.owner = THIS_MODULE,
};

static unsigned int init_done;
static struct compcache compcache;
static unsigned long compcache_size_kbytes;
#if defined(STATS)
static struct compcache_stats stats;
#endif

#if defined(STATS)
static struct proc_dir_entry *proc;

static int proc_compcache_read(char *page, char **start, off_t off,
				int count, int *eof, void *data)
{
	int len;
#if defined(STATS)
	size_t succ_writes;
	unsigned int good_compress_perc = 0, no_compress_perc = 0;
#endif

	if (off > 0) {
		*eof = 1;
		return 0;
	}

	len = sprintf(page,
		"DiskSize:	%8zu kB\n",
		compcache.size >> (10 - SECTOR_SHIFT));
#if defined(STATS)
	succ_writes = stats.num_writes - stats.failed_writes;
	if (succ_writes) {
		good_compress_perc = stats.good_compress * 100 / succ_writes;
		no_compress_perc = stats.pages_expand * 100 / succ_writes;
	}

#define K(x)	((x) >> 10)
	len += sprintf(page + len,
		"NumReads:	%8u\n"
		"NumWrites:	%8u\n"
		"FailedReads:	%8u\n"
		"FailedWrites:	%8u\n"
		"InvalidIO:	%8u\n"
		"GoodCompress:	%8u %%\n"
		"NoCompress:	%8u %%\n"
		"NotifyFree:	%8u\n"
		"CurrentPages:	%8zu\n"
		"CurrentMem:	%8zu kB\n"
		"PeakMem:	%8zu kB\n",
		stats.num_reads,
		stats.num_writes,
		stats.failed_reads,
		stats.failed_writes,
		stats.invalid_io,
		good_compress_perc,
		no_compress_perc,
		stats.notify_free,
		stats.curr_pages,
		K(stats.curr_mem),
		K(stats.peak_mem));
#endif
	return len;
}
#endif	/* STATS */

/*
 * callback function called when swap_map[offset] == 0
 * i.e page at this swap offset is no longer used
 */
static void notify_swap_entry_free(unsigned long offset)
{
	stat_inc(stats.notify_free);

	/*
	 *
	 * This callback happened due to some page being
	 * freed from swap-cache. This page was not written
	 * to swap disk.
	 */
	if (compcache.table[offset].addr == NULL)
		return;

	tlsf_free(compcache.table[offset].addr, compcache.mem_pool);
	stat_dec(stats.curr_pages);
	stat_set(stats.curr_mem, stats.curr_mem	-
			compcache.table[offset].len);
	compcache.table[offset].addr = NULL;
	compcache.table[offset].len = 0;
}

/* Check if request is within bounds and page aligned */
static inline int valid_swap_request(struct bio *bio)
{
	if (unlikely((bio->bi_sector >= compcache.size) ||
			(bio->bi_sector & (SECTORS_PER_PAGE - 1)) ||
			(bio->bi_vcnt != 1) ||
			(bio->bi_size != PAGE_SIZE) ||
			(bio->bi_io_vec[0].bv_offset != 0)))
		return 0;
	return 1;
}

static int compcache_make_request(struct request_queue *queue, struct bio *bio)
{
	int ret;
	size_t clen, page_no;
	void *user_mem;
	struct page *page;

	if (!valid_swap_request(bio)) {
		stat_inc(stats.invalid_io);
		goto out_nomap;
	}

	page = bio->bi_io_vec[0].bv_page;
	page_no = bio->bi_sector >> SECTORS_PER_PAGE_SHIFT;

	if (unlikely(!init_done) && PageSwapCache(page)) {
		swp_entry_t entry = { .val = page_private(page) };
		set_notify_swap_entry_free(swp_type(entry),
					notify_swap_entry_free);
		init_done = 1;
	}

	user_mem = kmap(page);

	if (bio_data_dir(bio) == READ) {
		stat_inc(stats.num_reads);
		/*
		 * This is attempt to read before any previous write
		 * to this location. This happens due to readahead when
		 * swap device is read from user-space (e.g. during swapon)
		 */
		if (unlikely(compcache.table[page_no].addr == NULL)) {
			pr_debug("Read before write on swap device: "
				"sector=%lu, size=%u, offset=%u\n",
				(ulong)(bio->bi_sector),
				bio->bi_size,
				bio->bi_io_vec[0].bv_offset);
			memset(user_mem, 0, PAGE_SIZE);
			kunmap(page);
			set_bit(BIO_UPTODATE, &bio->bi_flags);
			bio_endio(bio, 0);
			return 0;
		}

		/* Page is stored uncompressed since its incompressible */
		if (unlikely(compcache.table[page_no].len == PAGE_SIZE)) {
			memcpy(user_mem, compcache.table[page_no].addr,
							PAGE_SIZE);
			kunmap(page);
			set_bit(BIO_UPTODATE, &bio->bi_flags);
			bio_endio(bio, 0);
			return 0;
		}

		clen = PAGE_SIZE;
		ret = lzo1x_decompress_safe(
			compcache.table[page_no].addr,
			compcache.table[page_no].len,
			user_mem,
			&clen);

		/* should NEVER happen */
		if (unlikely(ret != LZO_E_OK)) {
			pr_err(C "Decompression failed! "
				"err=%d, page=%zu, len=%lu\n", ret, page_no,
				compcache.table[page_no].len);
			stat_inc(stats.failed_reads);
			goto out;
		}

		kunmap(page);
		set_bit(BIO_UPTODATE, &bio->bi_flags);
		bio_endio(bio, 0);
		return 0;
	} else {	/* Write */
		unsigned char *src = compcache.compress_buffer;
		stat_inc(stats.num_writes);
		/*
		 * System swaps to same sector again when the stored page
		 * is no longer referenced by any process. So, its now safe
		 * to free the memory that was allocated for this page.
		 */
		if (compcache.table[page_no].addr) {
			tlsf_free(compcache.table[page_no].addr,
				compcache.mem_pool);
			stat_dec(stats.curr_pages);
			stat_set(stats.curr_mem, stats.curr_mem	-
					compcache.table[page_no].len);
			compcache.table[page_no].addr = NULL;
			compcache.table[page_no].len = 0;
		}

		mutex_lock(&compcache.lock);
		ret = lzo1x_1_compress(user_mem, PAGE_SIZE,
			src, &clen, compcache.compress_workmem);
		if (unlikely(ret != LZO_E_OK)) {
			mutex_unlock(&compcache.lock);
			pr_err(C "Compression failed! err=%d\n", ret);
			compcache.table[page_no].addr = NULL;
			compcache.table[page_no].len = 0;
			stat_inc(stats.failed_writes);
			goto out;
		}

		/* Page is incompressible - store it as is */
		if (clen >= PAGE_SIZE) {
			pr_debug("Page expand on compression: "
				"page=%zu, size=%zu\n", page_no, clen);
			clen = PAGE_SIZE;
			src = user_mem;
		}

		if ((compcache.table[page_no].addr = tlsf_malloc(clen,
					compcache.mem_pool)) == NULL) {
			mutex_unlock(&compcache.lock);
			pr_err(C "Error allocating memory for compressed "
				"page: %zu, size=%zu \n", page_no, clen);
			compcache.table[page_no].len = 0;
			stat_inc(stats.failed_writes);
			goto out;
		}
		
		memcpy(compcache.table[page_no].addr, src, clen);

		/* Update stats */
		stat_inc(stats.curr_pages);
		stat_set(stats.curr_mem, stats.curr_mem + clen);
		stat_setmax(stats.peak_mem, stats.curr_mem);
		stat_inc_if_less(stats.pages_expand, PAGE_SIZE - 1, clen);
		stat_inc_if_less(stats.good_compress, clen,
						PAGE_SIZE / 2 + 1);
		mutex_unlock(&compcache.lock);
		
		compcache.table[page_no].len = clen;

		kunmap(page);
		set_bit(BIO_UPTODATE, &bio->bi_flags);
		bio_endio(bio, 0);
		return 0;
	}
out:
	kunmap(page);
out_nomap:
	bio_io_error(bio);
	return 0;
}

static void setup_swap_header(union swap_header *s)
{
	s->info.version = 1;
	s->info.last_page = compcache.size >> SECTORS_PER_PAGE_SHIFT;
	s->info.nr_badpages = 0;
	memcpy(s->magic.magic, "SWAPSPACE2", 10);
}

static void *get_mem(size_t size)
{
	return __vmalloc(size, GFP_NOIO, PAGE_KERNEL);
}

static void put_mem(void *ptr)
{
	vfree(ptr);
}

static int __init compcache_init(void)
{
	int ret;
	size_t num_pages;
	struct sysinfo i;

	mutex_init(&compcache.lock);

	if (compcache_size_kbytes == 0) {
		pr_info(C "compcache size not provided."
			" Using default: (%u%% of Total RAM).\n"
			"Use compcache_size_kbytes module param to specify"
			" custom size\n", DEFAULT_COMPCACHE_PERCENT);
		si_meminfo(&i);
		compcache_size_kbytes = ((DEFAULT_COMPCACHE_PERCENT *
				i.totalram) / 100) << (PAGE_SHIFT - 10);
	}
	
	compcache.size = compcache_size_kbytes << 10;
	compcache.size = (compcache.size + PAGE_SIZE - 1) & PAGE_MASK;
	pr_info(C "Compressed swap size set to: %zu KB\n", compcache.size >> 10);
	compcache.size >>= SECTOR_SHIFT;

	compcache.compress_workmem = kmalloc(LZO1X_MEM_COMPRESS, GFP_KERNEL);
	if (compcache.compress_workmem == NULL) {
		pr_err(C "Error allocating compressor working memory\n");
		ret = -ENOMEM;
		goto fail;
	}

	compcache.compress_buffer = kmalloc(2 * PAGE_SIZE, GFP_KERNEL);
	if (compcache.compress_buffer == NULL) {
		pr_err(C "Error allocating compressor buffer space\n");
		ret = -ENOMEM;
		goto fail;
	}

	num_pages = compcache.size >> SECTORS_PER_PAGE_SHIFT;
        compcache.table = vmalloc(num_pages * sizeof(*compcache.table));
        if (compcache.table == NULL) {
                pr_err(C "Error allocating compcache address table\n");
                ret = -ENOMEM;
                goto fail;
        }
        memset(compcache.table, 0, num_pages * sizeof(*compcache.table));

	compcache.table[0].addr = (void *)get_zeroed_page(GFP_KERNEL);
	if (compcache.table[0].addr == NULL) {
		pr_err(C "Error allocating swap header page\n");
		ret = -ENOMEM;
		goto fail;
	}
	compcache.table[0].len = PAGE_SIZE;
	setup_swap_header((union swap_header *)(compcache.table[0].addr));

	compcache.disk = alloc_disk(1);
	if (compcache.disk == NULL) {
		pr_err(C "Error allocating disk structure\n");
		ret = -ENOMEM;
		goto fail;
	}

	compcache.disk->first_minor = 0;
	compcache.disk->fops = &compcache_devops;
	/*
	 * It is named like this to prevent distro installers
	 * from offering compcache as installation target. They
	 * seem to ignore all devices beginning with 'ram'
	 */
	strcpy(compcache.disk->disk_name, "ramzswap0");

	compcache.disk->major = register_blkdev(0, compcache.disk->disk_name);
	if (compcache.disk->major < 0) {
		pr_err(C "Cannot register block device\n");
		ret = -EFAULT;
		goto fail;
	}

	compcache.disk->queue = blk_alloc_queue(GFP_KERNEL);
	if (compcache.disk->queue == NULL) {
		pr_err(C "Cannot register disk queue\n");
		ret = -EFAULT;
		goto fail;
	}

	set_capacity(compcache.disk, compcache.size);
	blk_queue_make_request(compcache.disk->queue, compcache_make_request);
	blk_queue_hardsect_size(compcache.disk->queue, PAGE_SIZE);
	add_disk(compcache.disk);

	compcache.mem_pool = tlsf_create_memory_pool("compcache",
				get_mem, put_mem,
				INIT_SIZE_BYTES, 0, GROW_SIZE_BYTES);
	if (compcache.mem_pool == NULL) {
		pr_err(C "Error creating memory pool\n");
		ret = -ENOMEM;
		goto fail;
	}

#if defined(STATS)
	proc = create_proc_entry("compcache", S_IRUGO, NULL);
	if (proc)
		proc->read_proc = &proc_compcache_read;
	else {
		ret = -ENOMEM;
		pr_warning(C "Error creating proc entry\n");
		goto fail;
	}
#endif

	pr_debug(C "Initialization done!\n");
	return 0;

fail:
	if (compcache.disk != NULL) {
		if (compcache.disk->major > 0)
			unregister_blkdev(compcache.disk->major,
					compcache.disk->disk_name);
		del_gendisk(compcache.disk);
	}

	free_page((unsigned long)compcache.table[0].addr);
	kfree(compcache.compress_workmem);
	kfree(compcache.compress_buffer);
	vfree(compcache.table);
	tlsf_destroy_memory_pool(compcache.mem_pool);
#if defined(STATS)
	if (proc)
		remove_proc_entry("compcache", proc->parent);
#endif
	pr_err(C "Initialization failed: err=%d\n", ret);
	return ret;
}

static void __exit compcache_exit(void)
{
	size_t i, num_pages;
	num_pages = compcache.size >> SECTORS_PER_PAGE_SHIFT;

	unregister_blkdev(compcache.disk->major, compcache.disk->disk_name);
	del_gendisk(compcache.disk);
	free_page((unsigned long)compcache.table[0].addr);
	kfree(compcache.compress_workmem);
	kfree(compcache.compress_buffer);

	/* Free all pages that are still in compcache */
	for (i = 1; i < num_pages; i++)
		if (compcache.table[i].addr)
			tlsf_free(compcache.table[i].addr, compcache.mem_pool);
	vfree(compcache.table);
	tlsf_destroy_memory_pool(compcache.mem_pool);

#if defined(STATS)
	remove_proc_entry("compcache", proc->parent);
#endif
	pr_debug("cleanup done!\n");
}

#ifndef MODULE
static int __init compcache_size_setup(char *str)
{
	if (str)
		compcache_size_kbytes = strtoul(str, NULL, 10);
	return 1;
}

__setup("compcache_size_kbytes=", compcache_size_setup);
#endif

module_param(compcache_size_kbytes, ulong, 0);
MODULE_PARM_DESC(compcache_size_kbytes, "compcache device size (in KB)");

module_init(compcache_init);
module_exit(compcache_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nitin Gupta <nitingupta910@gmail.com>");
MODULE_DESCRIPTION("Compressed RAM Based Swap Device");
