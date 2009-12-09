/*
 * arch/arm/mach-tegra/include/mach/io.h
 *
 * Copyright (c) 2008-2009, NVIDIA Corporation.
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

#ifndef __MACH_TEGRA_IO_H
#define __MACH_TEGRA_IO_H

#define IO_SPACE_LIMIT 0xffffffff

/* All of the register apertures for kernel-used peripherals exist within
 * the first 1MB of 6 apertures on AP15 and AP20.  These will be statically
 * mapped into kernel VAs at startup, so that the RM can be bootstrap with
 * the machine init.
 */

#if defined(CONFIG_ARCH_TEGRA_1x_SOC) || defined(CONFIG_ARCH_TEGRA_2x_SOC)
#define tegra_apertures(_aperture)			\
	_aperture(IRAM,		0x40000000, SZ_1M)	\
	_aperture(HOST1X,	0x50000000, SZ_1M)	\
	_aperture(PPSB,		0x60000000, SZ_1M)	\
	_aperture(APB,		0x70000000, SZ_1M)	\
	_aperture(USB,		0xC5000000, SZ_1M)	\
	_aperture(SDIO,		0xC8000000, SZ_1M)

/* remaps USB to 0xFE9xxxxx, SDIO to 0xFECxxxxx, and everything else to
 * 0xFEnxxxxx, where n is the most significant nybble */
#define tegra_munge_pa(_pa)						\
	(((((_pa)&0x70000000UL)>>8) + (((_pa)&0x0F000000UL)>>4)) |	\
	 ((_pa)&0xFFFFFUL) | 0xFE000000UL )
#else
#error "Invalid Tegra SoC family selection"
#endif

#ifndef __ASSEMBLER__

#define IO_ADDRESS(n) ((void __iomem *) tegra_munge_pa((n)))

static inline void __iomem *__io(unsigned long addr)
{
	return (void __iomem *)addr;
}
#define __io(a)         __io(a)
#define __mem_pci(a)    (a)

#endif

#endif
