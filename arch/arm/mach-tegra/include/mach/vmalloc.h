/*
 * arch/arm/mach-tegra/include/mach/vmalloc.h
 *
 * Copyright (c) 2009, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __MACH_TEGRA_VMALLOC_H
#define __MACH_TEGRA_VMALLOC_H

/************************************************************************
 *                    Kernel memory map for 512MB DRAM                  *
 *                                                                      *
 *                                                                      *
 *  FF00:0000     FFFF:FFFF    DMA mapping, reset vectors, etc. (16MB)  *
 *  FE00:0000     FEFF:FFFF    Static register apertures (16MB)         *
 *  F800:0000     FDFF:FFFF    -- Open -- (96MB)                        *
 *  E000:0000     F7FF:FFFF    VMalloc region (384MB)                   *
 *  C000:0000     DFFF:FFFF    Kernel direct-mapped memory region (.5GB)*
 *  BF00:0000     BFFF:FFFF    Kernel modules (16MB)                    *
 *  0000:0000     BEFF:FFFF    Task area (3056MB)                       *
 ************************************************************************/

#define VMALLOC_END        ((PAGE_OFFSET) + 512*1024*1024 + 384*1024*1024)

#endif

#endif
