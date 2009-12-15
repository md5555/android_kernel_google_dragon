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
#include "../nvrm/nvrmkernel/core/ap15/ap15rm_private.h"
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

static void nvmap_clean_handle(NvRmMemHandle hmem, size_t offs, size_t len);

static int nvmap_cache_maint(struct file *filp, void __user *arg);

static int nvmap_map_into_caller_ptr(struct file *filp, void __user *arg);

static int nvmap_rw_handle(struct file *filp, int is_read,
    void __user* arg);

static struct backing_dev_info nvmap_bdi = {
    .ra_pages     = 0,
    .capabilities = (BDI_CAP_NO_ACCT_AND_WRITEBACK |
                     BDI_CAP_READ_MAP | BDI_CAP_WRITE_MAP),
};

static struct vm_operations_struct nvmap_vma_ops = {
    .open = nvmap_vma_open,
    .close = nvmap_vma_close,
    .fault = nvmap_vma_fault,
};

static struct file_operations nvmap_file_ops = {
    .owner          = THIS_MODULE,
    .open           = nvmap_open,
    .release        = nvmap_release,
    .unlocked_ioctl = nvmap_ioctl,
    .mmap           = nvmap_mmap
};

struct nvmap_vma_priv {
    NvRmMemHandle hmem;
    size_t        offs;
    atomic_t      ref;
};

enum {
    NVMAP_DEV_DRAM = 0,
    NVMAP_DEV_IRAM,
    NVMAP_DEV_COUNT,
};

static struct {
    struct miscdevice dev;
    umode_t           mode;
} nvmap_device = {

    .dev = {
        .name  = "nvmap",
        .fops  = &nvmap_file_ops,
        .minor = MISC_DYNAMIC_MINOR,
    },
    .mode = S_IRUGO | S_IWUGO,
};

/* to ensure that the backing store for the VMA isn't freed while a fork'd
 * reference still exists, nvmap_vma_open increments the reference count on
 * the handle, and nvmap_vma_close decrements it. alternatively, we could
 * disallow copying of the vma, or behave like pmem and zap the pages. FIXME.
*/
static void nvmap_vma_open(struct vm_area_struct *vma)
{
    struct nvmap_vma_priv *priv;

    priv = (struct nvmap_vma_priv *)vma->vm_private_data;

    BUG_ON(!priv);

    atomic_inc(&priv->ref);
}

static void nvmap_vma_close(struct vm_area_struct *vma) {
    struct nvmap_vma_priv *priv = (struct nvmap_vma_priv *)vma->vm_private_data;

    if (priv && !atomic_dec_return(&priv->ref)) {
        if (priv->hmem) {
            size_t offs = (vma->vm_pgoff << PAGE_SHIFT) + priv->offs;
            size_t len = vma->vm_end - vma->vm_start;
            nvmap_clean_handle(priv->hmem, offs, len);
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

static inline void nvmap_flush(void* kva, unsigned long phys, size_t len)
{
    BUG_ON(!is_same_page(kva, kva+len-1));
    smp_dma_flush_range(nvmap_range(kva, len));
    outer_flush_range(nvmap_range(phys, len));
}

static void nvmap_clean_handle(NvRmMemHandle hmem, size_t start, size_t len)
{
    if (hmem->coherency != NvOsMemAttribute_WriteBack)
        return;

    if (hmem->hPageHandle) {
        size_t offs;
        size_t end = start + len;
        for (offs=start; offs<end; offs+=PAGE_SIZE) {
            size_t bytes = min((size_t)end-offs, (size_t)PAGE_SIZE);
            struct page *page = NvOsPageGetPage(hmem->hPageHandle, offs);
            void        *ptr = page_address(page) + (offs & ~PAGE_MASK);

            smp_dma_clean_range(nvmap_range(ptr, bytes));
            outer_clean_range(nvmap_range(__pa(ptr), bytes));
        }
    }
    else if (hmem->VirtualAddress && hmem->PhysicalAddress) {
        smp_dma_clean_range(nvmap_range(hmem->VirtualAddress, hmem->size));
        outer_clean_range(nvmap_range(hmem->PhysicalAddress, hmem->size));
    }
}

static int nvmap_vma_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
    struct nvmap_vma_priv *priv;
    unsigned long pfn;
    unsigned long offs = (unsigned long)(vmf->virtual_address - vma->vm_start);

    priv = (struct nvmap_vma_priv *)vma->vm_private_data;
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
        break;

    case NvRmHeap_GART:
    case NvRmHeap_External:
        if (!priv->hmem->hPageHandle)
            return VM_FAULT_SIGBUS;
        pfn = NvOsPageAddress(priv->hmem->hPageHandle, offs) >> PAGE_SHIFT;
        break;

    default:
        return VM_FAULT_SIGBUS;
    }
    vm_insert_pfn(vma, (unsigned long)vmf->virtual_address, pfn);
    return VM_FAULT_NOPAGE;
}

static long nvmap_ioctl(struct file *filp,
    unsigned int cmd, unsigned long arg)
{
    int err = 0;

    if (_IOC_TYPE(cmd) != NVMEM_IOC_MAGIC)
        return -ENOTTY;

    if (_IOC_NR(cmd) > NVMEM_IOC_MAXNR)
        return -ENOTTY;

    if (_IOC_DIR(cmd) & _IOC_READ)
        err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
    if (_IOC_DIR(cmd) & _IOC_WRITE)
        err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));

    if (err)
        return -EFAULT;

    switch (cmd) {
    case NVMEM_IOC_MMAP:
        err = nvmap_map_into_caller_ptr(filp, (void __user *)arg);
        break;

    case NVMEM_IOC_WRITE:
    case NVMEM_IOC_READ:
        err = nvmap_rw_handle(filp, cmd==NVMEM_IOC_READ, (void __user *)arg);
        break;

    case NVMEM_IOC_CACHE:
        err = nvmap_cache_maint(filp, (void __user *)arg);
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
    struct nvmap_vma_priv  *priv;
    struct vm_area_struct  *vma;
    NvRmMemHandle           hmem;
    int                     err = 0;

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

    priv = (struct nvmap_vma_priv *)vma->vm_private_data;
    BUG_ON(!priv);

    /* the VMA must exactly match the requested mapping operation, and the
     * VMA that is targetted must have been created originally by /dev/nvmap
     */
    if (((void*)vma->vm_start != op.addr) || (vma->vm_ops != &nvmap_vma_ops) ||
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
    if (hmem->coherency != NvOsMemAttribute_WriteBack)
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
    vma->vm_flags |= (VM_IO | VM_DONTEXPAND | VM_PFNMAP | VM_RESERVED);
    vma->vm_ops = &nvmap_vma_ops;
    vma->vm_private_data = priv;

    return 0;
}

static int nvmap_cache_maint(struct file *filp, void __user *arg)
{
    struct nvmem_cache_op  op;
    int                    err = 0;
    struct vm_area_struct *vma;
    struct nvmap_vma_priv *priv;
    size_t                 offs;
    size_t                 end;

    err = copy_from_user(&op, arg, sizeof(op));
    if (err)
        return err;

    if (!op.handle || !op.addr ||
        op.op<NVMEM_CACHE_OP_WB ||
        op.op>NVMEM_CACHE_OP_WB_INV) {
        return -EINVAL;
    }

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

    while (offs < end) {
        unsigned long inner_addr = 0;
        unsigned long outer_addr;
        size_t        count;
        int           clean = 0;

        /* if kernel mappings exist already for the memory handle, use
         * the kernel's mapping for cache maintenance */
        if (priv->hmem->VirtualAddress) {
            inner_addr = (unsigned long)priv->hmem->VirtualAddress + offs;
            clean = 1;
        }
        else {
            /* fixme: this is overly-conservative; not all pages in the
             * range might have be mapped, since pages are faulted on-demand.
             * the fault handler could be extended to mark pages as accessed /
             * dirty so that maintenance ops are only performed on pages which
             * need it
             */
            if (priv->hmem->hPageHandle) {
                struct page *page;
                page = NvOsPageGetPage(priv->hmem->hPageHandle, offs);
                inner_addr = (unsigned long)page_address(page);
                inner_addr += (offs & ~PAGE_MASK);
                clean = 1;
            }
            if (!inner_addr) {
                /* this case only triggers for rm-managed memory apertures
                 * (carveout, iram) for handles which do not have a mirror
                 * mapping in the kernel.
                 *
                 * use follow_page, to ensure that we only try to clean
                 * the cache using addresses that have been mapped */
                inner_addr = vma->vm_start + offs;
                if (follow_page(vma, inner_addr, FOLL_WRITE))
                    clean = 1;
            }
        }

        switch (priv->hmem->heap) {
        case NvRmHeap_ExternalCarveOut:
        case NvRmHeap_IRam:
            outer_addr = priv->hmem->PhysicalAddress+offs;
            break;

        case NvRmHeap_GART:
        case NvRmHeap_External:
            BUG_ON(!priv->hmem->hPageHandle);
            outer_addr = NvOsPageAddress(priv->hmem->hPageHandle, offs);
            break;
        default:
            BUG();
        }

        count = PAGE_SIZE - (inner_addr & ~PAGE_MASK);
        if (clean) {
            switch (op.op) {
            case NVMEM_CACHE_OP_WB:
                smp_dma_clean_range((void*)inner_addr,(void*) inner_addr+count);
                outer_clean_range(outer_addr, outer_addr+count);
                break;
            case NVMEM_CACHE_OP_INV:
                smp_dma_inv_range((void*)inner_addr,(void*) inner_addr+count);
                outer_inv_range(outer_addr, outer_addr+count);
                break;
            case NVMEM_CACHE_OP_WB_INV:
                smp_dma_flush_range((void*)inner_addr, (void*)inner_addr+count);
                outer_flush_range(outer_addr, outer_addr+count);
                break;
            }
        }
        offs += count;
    }

 out:
    return err;
}

static int nvmap_rw_handle(struct file *filp, int is_read,
    void __user* arg)
{
    struct nvmem_rw_handle op;
    NvRmMemHandle          hmem;
    uintptr_t              user_addr, hmem_offs;
    int                    err = 0;

    err = copy_from_user(&op, arg, sizeof(op));
    if (err)
        return err;

    if (!op.handle || !op.addr || !op.count || !op.elem_size)
        return -EINVAL;

    hmem = (NvRmMemHandle)op.handle;

    if (op.elem_size == op.hmem_stride &&
        op.elem_size == op.user_stride)
    {
        op.elem_size *= op.count;
        op.hmem_stride = op.elem_size;
        op.user_stride = op.elem_size;
        op.count = 1;
    }

    user_addr = (uintptr_t)op.addr;
    hmem_offs = (uintptr_t)op.offset;

    while (op.count--)
    {
        unsigned long remain;
        unsigned long l_hmem_offs = hmem_offs;
        unsigned long l_user_addr = user_addr;

        remain = op.elem_size;

        while (remain) {
            void         *hmemp;
            unsigned long hmem_phys;
            int           needs_unmap = 1;
            unsigned long bytes;

            bytes = min(remain, PAGE_SIZE-(l_hmem_offs & ~PAGE_MASK));
            bytes = min(bytes, PAGE_SIZE-(l_user_addr & ~PAGE_MASK));

            if (hmem->VirtualAddress) {
                hmemp = (void*)((uintptr_t)hmem->VirtualAddress + l_hmem_offs);
                needs_unmap = 0;
            }
            else if (hmem->hPageHandle) {
                unsigned long addr = l_hmem_offs & PAGE_MASK;

                if (hmem->coherency == NvOsMemAttribute_WriteBack) {
                    struct page *os_page;
                    os_page = NvOsPageGetPage(hmem->hPageHandle, addr);
                    hmemp = page_address(os_page);
                    needs_unmap = 0;
                }
                else {
                    addr = NvOsPageAddress(hmem->hPageHandle, addr);
                    hmemp = ioremap_wc(addr, PAGE_SIZE);
                }
                hmemp += (l_hmem_offs & ~PAGE_MASK);
                hmem_phys = NvOsPageAddress(hmem->hPageHandle, l_hmem_offs);
            }
            else {
                uintptr_t offs = hmem->PhysicalAddress + l_hmem_offs;
                uintptr_t base = offs & PAGE_MASK;

                if (hmem->coherency == NvOsMemAttribute_WriteBack)
                    hmemp = ioremap_cached(base, PAGE_SIZE);
                else
                    hmemp = ioremap_wc(base, PAGE_SIZE);

                hmemp += (offs & ~PAGE_MASK);
                hmem_phys = offs;
            }

            if (is_read) {
                BUG_ON(!access_ok(VERIFY_WRITE, (void *)l_user_addr, bytes));
                copy_to_user((void *)l_user_addr, hmemp, bytes);
            }
            else {
                BUG_ON(!access_ok(VERIFY_READ, (void*)l_user_addr, bytes));
                copy_from_user(hmemp, (void*)l_user_addr, bytes);
            }

            if (hmem->coherency == NvOsMemAttribute_WriteBack)
                nvmap_flush(hmemp, hmem_phys, bytes);

            if (needs_unmap) {
                iounmap((void *)((uintptr_t)hmemp & PAGE_MASK));
            }

            remain -= bytes;
            l_hmem_offs += bytes;
            l_user_addr += bytes;
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

static int __init nvmap_init(void)
{
    int err = bdi_init(&nvmap_bdi);
    if (err)
        return err;

    return platform_driver_register(&nvmap_driver);
}

static void __exit nvmap_deinit(void) {
    platform_driver_unregister(&nvmap_driver);
}

module_init(nvmap_init);
module_exit(nvmap_deinit);
