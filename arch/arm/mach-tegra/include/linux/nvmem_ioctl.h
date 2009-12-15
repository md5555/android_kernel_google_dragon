/*
 * arch/arm/mach-tegra/include/linux/nvmem_ioctl.h
 *
 * structure declarations for nvmem and nvmap user-space ioctls
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

#include <linux/ioctl.h>

#if !defined(__KERNEL__)
#define __user
#endif

#ifndef _MACH_TEGRA_NVMEM_IOCTL_H_
#define _MACH_TEGRA_NVMEM_IOCTL_H_

struct nvmem_create_handle {
    union {
        unsigned long key;    // stores the Key for ClaimPreservedHandle
        unsigned long id;     // stores the ID for FromId
        unsigned long size;   // stores the size for CreateHandle
    };
    uintptr_t     handle;
};

#define NVMEM_ALLOC_HANDLE_FLAG(_COH, _ALIGN) (((_COH)<<16) | ((_ALIGN)&0xffff))
struct nvmem_alloc_handle {
    uintptr_t     handle;     // hmem
    unsigned int  heap_mask;  // bitmask of legal heaps, or 0 for default heaps
    unsigned int  flags;      // munging of coherency and alignment into 1 word
};

struct nvmem_map_caller {
    uintptr_t     handle;     // hmem
    void __user  *addr;       // user pointer
    size_t        offset;     // offset into hmem; should be page-aligned
    size_t        length;     // number of bytes to map; should be page-aligned
    unsigned int  flags;
};

struct nvmem_rw_handle {
    uintptr_t     handle;      // hmem
    void __user  *addr;        // user pointer
    size_t        offset;      // offset into hmem
    size_t        elem_size;   // individual atom size
    size_t        hmem_stride; // delta in bytes between atoms in hmem
    size_t        user_stride; // delta in bytes between atoms in user
    size_t        count;       // number of atoms to copy
};

struct nvmem_pin_handle {
    unsigned long __user *handles;  // array of handles to pin/unpin
    unsigned long __user *addr;     // array of addresses to return   
    unsigned long         count;    // number of entries in handles
};

enum {
    NVMEM_CACHE_OP_WB = 0,
    NVMEM_CACHE_OP_INV,
    NVMEM_CACHE_OP_WB_INV,
};

struct nvmem_cache_op {
    uintptr_t    handle;
    void __user *addr;
    size_t       len;
    int          op;
};

#define NVMEM_IOC_MAGIC 'N'

/* Creates a new memory handle. On input, the argument is the size of the new
 * handle; on return, the argument is the name of the new handle 
 */
#define NVMEM_IOC_CREATE   _IOWR(NVMEM_IOC_MAGIC, 0, struct nvmem_create_handle)
#define NVMEM_IOC_CLAIM    _IOWR(NVMEM_IOC_MAGIC, 1, struct nvmem_create_handle)
#define NVMEM_IOC_FROM_ID  _IOWR(NVMEM_IOC_MAGIC, 2, struct nvmem_create_handle)


/* Actually allocates memory for the specified handle */
#define NVMEM_IOC_ALLOC    _IOW (NVMEM_IOC_MAGIC, 3, struct nvmem_alloc_handle)

/* Frees a memory handle, unpinning any pinned pages and unmapping any mappings
 */
#define NVMEM_IOC_FREE       _IOW (NVMEM_IOC_MAGIC, 4, uintptr_t)

/* Maps the region of the specified handle into a user-provided virtual address
 * that was previously created via an mmap syscall on this fd */
#define NVMEM_IOC_MMAP       _IOWR(NVMEM_IOC_MAGIC, 5, struct nvmem_map_caller)

/* Reads/writes data (possibly strided) from a user-provided buffer into the
 * hmem at the specified offset */
#define NVMEM_IOC_WRITE      _IOW (NVMEM_IOC_MAGIC, 6, struct nvmem_rw_handle)
#define NVMEM_IOC_READ       _IOW (NVMEM_IOC_MAGIC, 7, struct nvmem_rw_handle)

/* Pins a single memory handle and ensures a contiguous mapping exists in
 * either phsyical memory or the GART. If the memory handle is backed by a
 * swap device, all pages will be resident in memory before this ioctl returns.
 * Pin operations may be performed recursively on memory handles.
 */
#define NVMEM_IOC_PIN        _IOWR(NVMEM_IOC_MAGIC, 8, uintptr_t)

/* Unpins a single memory handle. If the memory handle is backed by a swap
 * device, unpinning a memory handle may result in the handle being decommitted
 */
#define NVMEM_IOC_UNPIN      _IOW (NVMEM_IOC_MAGIC, 9, uintptr_t)

/* Like IOC_PIN and IOC_UNPIN, but operates on a list of memory handles */
#define NVMEM_IOC_PIN_MULT   _IOWR(NVMEM_IOC_MAGIC, 10, struct nvmem_pin_handle)
#define NVMEM_IOC_UNPIN_MULT _IOW (NVMEM_IOC_MAGIC, 11, struct nvmem_pin_handle)

#define NVMEM_IOC_CACHE      _IOW (NVMEM_IOC_MAGIC, 12, struct nvmem_cache_op)

#define NVMEM_IOC_MAXNR (_IOC_NR(NVMEM_IOC_CACHE))
#endif
