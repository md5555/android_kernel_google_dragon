/*
 * arch/arm/mach-tegra/include/mach/vmalloc.h
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

#ifndef __MACH_TEGRA_VMALLOC_H
#define __MACH_TEGRA_VMALLOC_H

/************************************************************************
 *  FF00:0000	FFFF:FFF	DMA mapping, reset vectors, etc. (16MB)	*
 *  FE00:0000	FEFF:FFFF	Static register apertures (16MB)	*
 *  F840:0000	FDFF:FFFF	-- Open -- (92MB)			*
 *  F820:0000	F83F:FFFF	nvmap remapping area (2MB)		*
 *  F800:0000	FDFF:FFFF	-- empty -- (2MB)			*
 *  high_memory	F7FF:FFFF	VMalloc region				*
 ************************************************************************/

#define VMALLOC_END		((PAGE_OFFSET) + 512*1024*1024 + 384*1024*1024)

#ifdef CONFIG_DEVNVMAP
#define NVMAP_BASE		(VMALLOC_END + SZ_2M)
#define NVMAP_SIZE		SZ_2M
#endif

#endif
