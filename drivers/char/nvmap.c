/*
 * drivers/char/nvmap.c
 *
 * Memory mapping driver for Tegra anonymous memory handles
 *
 * Copyright (c) 2009, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/module.h>
#include <linux/bitmap.h>
#include <linux/wait.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/mm.h>
#include <linux/mman.h>
#include <linux/uaccess.h>
#include <linux/backing-dev.h>
#include <linux/device.h>
#include <linux/highmem.h>
#include <linux/smp_lock.h>
#include <linux/pagemap.h>
#include <linux/sched.h>
#include <linux/io.h>
#include <linux/tegra_devices.h>
#include <asm/tlbflush.h>
#include "linux/nvmem_ioctl.h"
#include "nvcommon.h"
#include "nvrm_memmgr.h"
#include "nvrm_memmgr_private.h"

static void nvmap_vma_open(struct vm_area_struct *vma);

static void nvmap_vma_close(struct vm_area_struct *vma);

static int nvmap_vma_fault(struct vm_area_struct *vma, struct vm_fault *vmf);

static int nvmap_open(struct inode *inode, struct file *filp);

static int nvmap_release(struct inode *inode, struct file *file);

static int nvmap_mmap(struct file *filp, struct vm_area_struct *vma);

static long nvmap_ioctl(struct file *filp,
	unsigned int cmd, unsigned long arg);

static int nvmap_cache_maint(struct file *filp, void __user *arg);

static int nvmap_map_into_caller_ptr(struct file *filp, void __user *arg);

static int nvmap_rw_handle(struct file *filp, int is_read,
	void __user* arg);

extern void NvRmPrivMemIncrRef(NvRmMemHandle hmem);

static struct backing_dev_info nvmap_bdi = {
	.ra_pages	= 0,
	.capabilities	= (BDI_CAP_NO_ACCT_AND_WRITEBACK |
			   BDI_CAP_READ_MAP | BDI_CAP_WRITE_MAP),
};


#define NVMAP_PTE_OFFSET(x) (((unsigned long)(x) - NVMAP_BASE) >> PAGE_SHIFT)
#define NVMAP_PTE_INDEX(x) (((unsigned long)(x) - NVMAP_BASE)>>PGDIR_SHIFT)
#define NUM_NVMAP_PTES (NVMAP_SIZE >> PGDIR_SHIFT)
#define NVMAP_END (NVMAP_BASE + NVMAP_SIZE)
#define NVMAP_PAGES (NVMAP_SIZE >> PAGE_SHIFT)

static pte_t *nvmap_pte[NUM_NVMAP_PTES];
static unsigned long nvmap_ptebits[NVMAP_PAGES/BITS_PER_LONG];
static DEFINE_SPINLOCK(nvmap_ptelock);
static DECLARE_WAIT_QUEUE_HEAD(nvmap_ptefull);

static struct vm_operations_struct nvmap_vma_ops = {
	.open	= nvmap_vma_open,
	.close	= nvmap_vma_close,
	.fault	= nvmap_vma_fault,
};

static struct file_operations nvmap_file_ops = {
	.owner		= THIS_MODULE,
	.open		= nvmap_open,
	.release	= nvmap_release,
	.unlocked_ioctl = nvmap_ioctl,
	.mmap		= nvmap_mmap
};

struct nvmap_vma_priv {
	NvRmMemHandle	hmem;
	size_t		offs;
	atomic_t	ref;
};

enum {
	NVMAP_DEV_DRAM = 0,
	NVMAP_DEV_IRAM,
	NVMAP_DEV_COUNT,
};

static struct {
	struct miscdevice	dev;
	umode_t			mode;
} nvmap_device = {

	.dev = {
		.name	= "nvmap",
		.fops	= &nvmap_file_ops,
		.minor	= MISC_DYNAMIC_MINOR,
	},
	.mode = S_IRUGO | S_IWUGO,
};

static int _nvmap_map_pte(unsigned long pfn, pgprot_t prot, void **vaddr)
{
	static unsigned int last_bit = 0;
	unsigned long bit;
	pte_t *pte;
	unsigned long addr;
	unsigned long flags;
        u32 off;
        int idx;

	spin_lock_irqsave(&nvmap_ptelock, flags);

	bit = find_next_zero_bit(nvmap_ptebits, NVMAP_PAGES, last_bit);
	if (bit==NVMAP_PAGES) {
		bit = find_first_zero_bit(nvmap_ptebits, last_bit);
		if (bit == last_bit) bit = NVMAP_PAGES;
	}

	if (bit==NVMAP_PAGES) {
		spin_unlock_irqrestore(&nvmap_ptelock, flags);
		return -ENOMEM;
	}

	last_bit = bit;
	set_bit(bit, nvmap_ptebits);
	spin_unlock_irqrestore(&nvmap_ptelock, flags);

	addr = NVMAP_BASE + bit*PAGE_SIZE;

	idx = NVMAP_PTE_INDEX(addr);
	off = NVMAP_PTE_OFFSET(addr) & (PTRS_PER_PTE-1);

	pte = nvmap_pte[idx] + off;
	set_pte_ext(pte, pfn_pte(pfn, prot), 0);
	flush_tlb_kernel_page(addr);
	*vaddr = (void *)addr;
	return 0;
}

static int nvmap_map_pte(unsigned long pfn, pgprot_t prot, void **addr)
{
	int ret;
	ret = wait_event_interruptible(nvmap_ptefull,
		!_nvmap_map_pte(pfn, prot, addr));

	if (ret==-ERESTARTSYS) return -EINTR;
	return ret;
}

static void nvmap_unmap_pte(void *addr)
{
	unsigned long bit = NVMAP_PTE_OFFSET(addr);
	unsigned long flags;

	/* the ptes aren't cleared in this function, since the address isn't
	 * re-used until it is allocated again by nvmap_map_pte. */
	BUG_ON(bit >= NVMAP_PAGES);
	spin_lock_irqsave(&nvmap_ptelock, flags);
	clear_bit(bit, nvmap_ptebits);
	spin_unlock_irqrestore(&nvmap_ptelock, flags);
	wake_up(&nvmap_ptefull);
}

/* to ensure that the backing store for the VMA isn't freed while a fork'd
 * reference still exists, nvmap_vma_open increments the reference count on
 * the handle, and nvmap_vma_close decrements it. alternatively, we could
 * disallow copying of the vma, or behave like pmem and zap the pages. FIXME.
*/
static void nvmap_vma_open(struct vm_area_struct *vma)
{
	struct nvmap_vma_priv *priv;

	priv = vma->vm_private_data;

	BUG_ON(!priv);

	atomic_inc(&priv->ref);
}

static void nvmap_vma_close(struct vm_area_struct *vma) {
	struct nvmap_vma_priv *priv = vma->vm_private_data;

	if (priv && !atomic_dec_return(&priv->ref)) {
		NvRmMemHandle hmem = priv->hmem;
		if (hmem) {
			if (hmem->coherency==NvOsMemAttribute_WriteBack)
				dmac_clean_all();
			NvRmMemHandleFree(priv->hmem);
		}
		kfree(priv);
	}
	vma->vm_private_data = NULL;
}

extern struct page *NvOsPageGetPage(NvOsPageAllocHandle, size_t);
#define nvmap_range(x,y) (x), (x)+(y)

#define is_same_page(a, b)  \
	((unsigned long)(a)>>PAGE_SHIFT == (unsigned long)(b)>>PAGE_SHIFT)

static int nvmap_vma_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
	struct nvmap_vma_priv *priv;
	struct page *page;
	unsigned long pfn;
	unsigned long offs;

	offs = (unsigned long)(vmf->virtual_address - vma->vm_start);
	priv = vma->vm_private_data;
	if (!priv || !priv->hmem)
		return VM_FAULT_SIGBUS;

	offs += priv->offs;
	/* if the VMA was split for some reason, vm_pgoff will be the VMA's
	 * offset from the original VMA */
	offs += (vma->vm_pgoff << PAGE_SHIFT);

	if (offs >= priv->hmem->size)
		return VM_FAULT_SIGBUS;

	switch (priv->hmem->heap) {
	case NvRmHeap_ExternalCarveOut:
	case NvRmHeap_IRam:
		pfn = ((priv->hmem->PhysicalAddress+offs) >> PAGE_SHIFT);
		vm_insert_pfn(vma, (unsigned long)vmf->virtual_address, pfn);
		return VM_FAULT_NOPAGE;

	case NvRmHeap_GART:
	case NvRmHeap_External:
		if (!priv->hmem->hPageHandle)
			return VM_FAULT_SIGBUS;
		page = NvOsPageGetPage(priv->hmem->hPageHandle, offs);
		if (page) get_page(page);
		vmf->page = page;
		return (page) ? 0 : VM_FAULT_SIGBUS;

	default:
		return VM_FAULT_SIGBUS;
	}
}

static long nvmap_ioctl(struct file *filp,
	unsigned int cmd, unsigned long arg)
{
	int err = 0;
	void __user *uarg = (void __user *)arg;

	if (_IOC_TYPE(cmd) != NVMEM_IOC_MAGIC)
		return -ENOTTY;

	if (_IOC_NR(cmd) > NVMEM_IOC_MAXNR)
		return -ENOTTY;

	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, uarg, _IOC_SIZE(cmd));
	if (_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, uarg, _IOC_SIZE(cmd));

	if (err)
		return -EFAULT;

	switch (cmd) {
	case NVMEM_IOC_MMAP:
		err = nvmap_map_into_caller_ptr(filp, uarg);
		break;

	case NVMEM_IOC_WRITE:
	case NVMEM_IOC_READ:
		err = nvmap_rw_handle(filp, cmd==NVMEM_IOC_READ, uarg);
		break;

	case NVMEM_IOC_CACHE:
		err = nvmap_cache_maint(filp, uarg);
		break;

	default:
		return -ENOTTY;
	}
	return err;
}

static int nvmap_release(struct inode *inode, struct file *file)
{
	return 0;
}

static int nvmap_open(struct inode *inode, struct file *filp)
{
	/* eliminate read, write and llseek support on this node */
	int ret;

	ret = nonseekable_open(inode, filp);
	if (unlikely(ret))
		return ret;

	filp->f_mapping->backing_dev_info = &nvmap_bdi;

	filp->private_data = NULL;

	return 0;
}

static int nvmap_map_into_caller_ptr(struct file *filp, void __user *arg)
{
	struct nvmem_map_caller op;
	struct nvmap_vma_priv *priv;
	struct vm_area_struct *vma;
	NvRmMemHandle hmem;
	int err = 0;

	err = copy_from_user(&op, arg, sizeof(op));
	if (err)
		return err;

	hmem = (NvRmMemHandle)op.handle;
	if (!hmem)
		return -EINVAL;

	down_read(&current->mm->mmap_sem);

	vma = find_vma(current->mm, (unsigned long)op.addr);
	if (!vma || !vma->vm_private_data) {
		err = -ENOMEM;
		goto out;
	}

	if (op.offset & ~PAGE_MASK) {
		err = -EFAULT;
		goto out;
	}

	if ((op.offset + op.length) >
		((hmem->size+PAGE_SIZE-1)&PAGE_MASK)) {
		err = -EADDRNOTAVAIL;
		goto out;
	}

	priv = vma->vm_private_data;
	BUG_ON(!priv);

	/* the VMA must exactly match the requested mapping operation, and the
	 * VMA that is targetted must have been created originally by /dev/nvmap
	 */
	if (((void*)vma->vm_start != op.addr) ||
		(vma->vm_ops != &nvmap_vma_ops) ||
		(vma->vm_end-vma->vm_start != op.length)) {
		err = -EPERM;
		goto out;
	}

	/* verify that each mmap() system call creates a unique VMA */

	if (priv->hmem && hmem==priv->hmem)
		goto out;
	else if (priv->hmem) {
		err = -EADDRNOTAVAIL;
		goto out;
	}

	if (hmem->alignment & ~PAGE_MASK) {
		err = -EFAULT;
		goto out;
	}

	priv->hmem = hmem;
	priv->offs = op.offset;

	/* if the hmem is not writeback-cacheable, drop back to a page mapping
	 * which will guarantee DMA coherency
	 */
	if (hmem->coherency == NvOsMemAttribute_WriteBack)
		vma->vm_page_prot = pgprot_inner_writeback(vma->vm_page_prot);
	else
		vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

	NvRmPrivMemIncrRef(hmem);

 out:
	up_read(&current->mm->mmap_sem);
	return err;
}
/* Initially, the nvmap mmap system call is used to allocate an inaccessible
 * region of virtual-address space in the client.  A subsequent
 * NVMAP_IOC_MMAP ioctl will associate each
 */
static int nvmap_mmap(struct file *filp, struct vm_area_struct *vma)
{
	/* FIXME: drivers which do not support cow seem to be split down the
	 * middle whether to force the VM_SHARED flag, or to return an error
	 * when this flag isn't already set (i.e., MAP_PRIVATE).
	 */
	struct nvmap_vma_priv *priv;

	vma->vm_private_data = NULL;

	priv = kzalloc(sizeof(*priv),GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->offs = 0;
	priv->hmem = NULL;
	atomic_set(&priv->ref, 1);

	vma->vm_flags |= VM_SHARED;
	vma->vm_flags |= (VM_IO | VM_DONTEXPAND | VM_MIXEDMAP | VM_RESERVED);
	vma->vm_ops = &nvmap_vma_ops;
	vma->vm_private_data = priv;

	return 0;
}

static int nvmap_cache_maint(struct file *filp, void __user *arg)
{
	struct nvmem_cache_op	op;
	int			err = 0;
	struct vm_area_struct	*vma;
	struct nvmap_vma_priv	*priv;
	size_t			offs;
	size_t			end;
	unsigned long		count;
	pgprot_t		prot = pgprot_inner_writeback(pgprot_kernel);

	err = copy_from_user(&op, arg, sizeof(op));
	if (err) return err;

	if (!op.handle || !op.addr || op.op<NVMEM_CACHE_OP_WB ||
		op.op>NVMEM_CACHE_OP_WB_INV)
		return -EINVAL;

	vma = find_vma(current->active_mm, (unsigned long)op.addr);
	if (!vma || vma->vm_ops!=&nvmap_vma_ops ||
		(unsigned long)op.addr + op.len > vma->vm_end) {
		err = -EADDRNOTAVAIL;
		goto out;
	}

	priv = (struct nvmap_vma_priv *)vma->vm_private_data;

	if (priv->hmem != (NvRmMemHandle)op.handle) {
		err = -EFAULT;
		goto out;
	}

	/* don't waste time on cache maintenance if the handle isn't cached */
	if (priv->hmem->coherency != NvOsMemAttribute_WriteBack)
		goto out;

	offs = (unsigned long)op.addr - vma->vm_start;
	end  = offs + op.len;

	/* for any write-back operation, it is safe to writeback the entire
	 * cache rather than one line at a time.  for large regions, it
	 * is faster to do this than to iterate over every line. */
	if (end-offs >= PAGE_SIZE*3 && op.op != NVMEM_CACHE_OP_INV) {
		if (op.op==NVMEM_CACHE_OP_WB) dmac_clean_all();
		else dmac_flush_all();
		goto out;
	}

	while (offs < end) {
		struct page *page = NULL;
		void *addr = NULL, *src;

		if (priv->hmem->hPageHandle) {
			struct page *page;
			page = NvOsPageGetPage(priv->hmem->hPageHandle, offs);
			get_page(page);
			err = nvmap_map_pte(page_to_pfn(page), prot, &addr);
		} else {
			unsigned long phys = priv->hmem->PhysicalAddress + offs;
			err = nvmap_map_pte(phys>>PAGE_SHIFT, prot, &addr);
		}

		if (err) {
			if (page) put_page(page);
			break;
		}

		src = addr + (offs & ~PAGE_MASK);
		count = min_t(size_t, end-offs, PAGE_SIZE-(offs & ~PAGE_MASK));

		switch (op.op) {
		case NVMEM_CACHE_OP_WB:
			smp_dma_clean_range(src, src+count);
			break;
		case NVMEM_CACHE_OP_INV:
			smp_dma_inv_range(src, src+count);
			break;
		case NVMEM_CACHE_OP_WB_INV:
			smp_dma_flush_range(src, src+count);
			break;
		}
		offs += count;
		nvmap_unmap_pte(addr);
		if (page) put_page(page);
	}

 out:
	return err;
}

static int nvmap_do_rw_handle(NvRmMemHandle hmem, int is_read,
	unsigned long l_hmem_offs, unsigned long l_user_addr,
	unsigned long bytes, pgprot_t prot)
{
	void *addr = NULL, *dest;
	struct page *page = NULL;
	int ret = 0;

        if (hmem->hPageHandle) {
		page = NvOsPageGetPage(hmem->hPageHandle, l_hmem_offs);
		get_page(page);
		ret = nvmap_map_pte(page_to_pfn(page), prot, &addr);
	} else {
		unsigned long phys = hmem->PhysicalAddress + l_hmem_offs;
		ret = nvmap_map_pte(phys>>PAGE_SHIFT, prot, &addr);
	}

	dest = addr + (l_hmem_offs & ~PAGE_MASK);

	if (is_read && !access_ok(VERIFY_WRITE, (void *)l_user_addr, bytes))
		ret = -EPERM;

	if (!is_read && !access_ok(VERIFY_READ, (void *)l_user_addr, bytes))
		ret = -EPERM;

	if (!ret) {
		if (is_read) copy_to_user((void *)l_user_addr, dest, bytes);
		else copy_from_user(dest, (void*)l_user_addr, bytes);
	}

	if (addr) nvmap_unmap_pte(addr);
	if (page) put_page(page);
	return ret;
}

static int nvmap_rw_handle(struct file *filp, int is_read,
	void __user* arg)
{
	struct nvmem_rw_handle op;
	NvRmMemHandle hmem;
	uintptr_t user_addr, hmem_offs;
	int err = 0;
	pgprot_t prot;

	err = copy_from_user(&op, arg, sizeof(op));
	if (err)
		return err;

	if (!op.handle || !op.addr || !op.count || !op.elem_size)
		return -EINVAL;

	hmem = (NvRmMemHandle)op.handle;

	if (op.elem_size == op.hmem_stride &&
		op.elem_size == op.user_stride) {
		op.elem_size *= op.count;
		op.hmem_stride = op.elem_size;
		op.user_stride = op.elem_size;
		op.count = 1;
	}

	user_addr = (uintptr_t)op.addr;
	hmem_offs = (uintptr_t)op.offset;

	if (hmem->coherency==NvOsMemAttribute_WriteBack)
		prot = pgprot_inner_writeback(pgprot_kernel);
	else if (hmem->coherency==NvOsMemAttribute_WriteCombined)
		prot = pgprot_writecombine(pgprot_kernel);
	else
		prot = pgprot_noncached(pgprot_kernel);

	while (op.count--) {
		unsigned long remain;
		unsigned long l_hmem_offs = hmem_offs;
		unsigned long l_user_addr = user_addr;

		remain = op.elem_size;

		while (remain && !err) {
			unsigned long bytes;

			bytes = min(remain, PAGE_SIZE-(l_hmem_offs&~PAGE_MASK));
			bytes = min(bytes, PAGE_SIZE-(l_user_addr&~PAGE_MASK));

			err = nvmap_do_rw_handle(hmem, is_read, l_hmem_offs,
				l_user_addr, bytes, prot);

			if (!err) {
				remain -= bytes;
				l_hmem_offs += bytes;
				l_user_addr += bytes;
			}
		}

		user_addr += op.user_stride;
		hmem_offs += op.hmem_stride;
	}
	dmb();

	return err;
}

static int nvmap_probe(struct platform_device *pdev)
{
	int e = 0;
	static int id_count = 0;

	/* only one nvmap device can be probed today */
	if (id_count)
		return -EINVAL;

	e = misc_register(&nvmap_device.dev);

	if (e<0)
		nvmap_device.dev.minor = MISC_DYNAMIC_MINOR;
	else
		id_count++;

	return e;
}

static int nvmap_remove(struct platform_device *pdev)
{
	if (nvmap_device.dev.minor != MISC_DYNAMIC_MINOR)
		misc_deregister(&nvmap_device.dev);
	nvmap_device.dev.minor = MISC_DYNAMIC_MINOR;

	return 0;
}

static struct platform_driver nvmap_driver = {
	.probe = nvmap_probe,
	.remove = nvmap_remove,
	.driver = { .name = "nvmap_drv" }
};

static int __init nvmap_pte_init(void)
{
	u32 base = NVMAP_BASE;
	pgd_t *pgd;
	pmd_t *pmd;
	pte_t *pte;
	int i = 0;
	do {
		pgd = pgd_offset(&init_mm, base);
		pmd = pmd_alloc(&init_mm, pgd, base);
		if (!pmd) {
			pr_err("%s: no pmd tables\n", __func__);
			return -ENOMEM;
		}
		pte = pte_alloc_kernel(pmd, base);
		if (!pte) {
			pr_err("%s: no pte tables\n", __func__);
			return -ENOMEM;
		}
		nvmap_pte[i++] = pte;
		base += (1<<PGDIR_SHIFT);
	} while (base < NVMAP_END);

	return 0;
}
core_initcall(nvmap_pte_init);

static int __init nvmap_init(void)
{
	int err = bdi_init(&nvmap_bdi);

	if (err) return err;

	bitmap_zero(nvmap_ptebits, NVMAP_PAGES);

	return platform_driver_register(&nvmap_driver);
}

static void __exit nvmap_deinit(void) {
	platform_driver_unregister(&nvmap_driver);
}

module_init(nvmap_init);
module_exit(nvmap_deinit);
