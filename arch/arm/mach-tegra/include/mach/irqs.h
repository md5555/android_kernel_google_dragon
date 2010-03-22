/*
 * arch/arm/mach-tegra/include/mach/irqs.h
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

#ifndef __MACH_TEGRA_IRQS_H
#define __MACH_TEGRA_IRQS_H

#if defined(CONFIG_ARCH_TEGRA_2x_SOC)
#define NR_IRQS 512

#define INT_PRI_BASE		32
#define INT_RTC			(INT_PRI_BASE + 2)
#define INT_USB			(INT_PRI_BASE + 20)
#define INT_USB2		(INT_PRI_BASE + 21)
#define INT_APB_DMA		(INT_PRI_BASE + 26)

#define INT_SEC_BASE		(INT_PRI_BASE + 32)
#define INT_GPIO1		(INT_SEC_BASE + 0)
#define INT_GPIO2		(INT_SEC_BASE + 1)
#define INT_GPIO3		(INT_SEC_BASE + 2)
#define INT_GPIO4		(INT_SEC_BASE + 3)
#define INT_SYS_STATS_MON	(INT_SEC_BASE + 22)
#define INT_GPIO5		(INT_SEC_BASE + 23)

#define INT_TRI_BASE		(INT_SEC_BASE + 32)
#define INT_KBC			(INT_TRI_BASE + 21)
#define INT_EXTERNAL_PMU	(INT_TRI_BASE + 22)
#define INT_GPIO6		(INT_TRI_BASE + 23)
#define INT_GPIO7		(INT_TRI_BASE + 25)

#define INT_QUAD_BASE		(INT_TRI_BASE + 32)
#define INT_USB3		(INT_QUAD_BASE + 1)

#define INT_GPIO_BASE		(INT_QUAD_BASE + 32)
#define INT_GPIO_NR		(28*8)

#define INT_APBDMA_BASE	 	(INT_GPIO_BASE + INT_GPIO_NR)
#define INT_APBDMA_NR		(16)

#else
#error "Invalid Tegra SoC family selection"
#endif

#define INT_SYS_NR	(INT_GPIO_BASE - INT_PRI_BASE)
#define INT_SYS_SZ	(INT_SEC_BASE - INT_PRI_BASE)

#endif
