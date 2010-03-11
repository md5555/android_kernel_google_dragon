/*
 * arch/arm/mach-tegra/include/mach/memory.h
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

#ifndef __MACH_TEGRA_MEMORY_H
#define __MACH_TEGRA_MEMORY_H

/* physical offset of RAM */
#if defined(CONFIG_ARCH_TEGRA_1x_SOC) || defined(CONFIG_ARCH_TEGRA_2x_SOC)
#define PHYS_OFFSET		UL(0)
#else
#error "Invalid Tegra SoC family selection"
#endif

/*
 * Unaligned DMA causes tegra dma to place data on 4-byte boundary after
 * expected address. Call to skb_reserve(skb, NET_IP_ALIGN) was causing skb
 * buffers in usbnet.c to become unaligned.
 */
#define NET_IP_ALIGN	0
#define NET_SKB_PAD	L1_CACHE_BYTES


#define __arch_page_to_dma(dev, page)	((dma_addr_t)__virt_to_phys(page_address(page)))

#define __arch_dma_to_virt(dev, addr)	((void *) __phys_to_virt(addr))

#define __arch_virt_to_dma(dev, addr)	((dma_addr_t) __virt_to_phys((unsigned long)(addr)))

#define __arch_dma_to_page(dev, addr)	(phys_to_page(addr))

#endif

