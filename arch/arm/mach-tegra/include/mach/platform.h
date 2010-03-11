/*
 * arch/arm/mach-tegra/include/mach/platform.h
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

#ifndef __MACH_TEGRA_PLATFORM_H

extern unsigned long	tegra_get_module_inst_base(const char *, int);
extern unsigned long	tegra_get_module_inst_size(const char *, int);
extern unsigned int	tegra_get_module_inst_irq(const char *, int, int);

#define TEGRA_PL310_BASE	(tegra_get_module_inst_base("pl310", 0))
#define TEGRA_PL310_IRQ		(tegra_get_module_inst_irq("pl310", 0, 0))

#define TEGRA_SCU_BASE		(tegra_get_module_inst_base("scu", 0))
#define TEGRA_PCIE_BASE		(tegra_get_module_inst_base("pcie", 0))

#define TEGRA_SCU0_IRQ	NO_IRQ
#define TEGRA_SCU1_IRQ	NO_IRQ
#define TEGRA_SCU2_IRQ	NO_IRQ
#define TEGRA_SCU3_IRQ	NO_IRQ
#define TEGRA_SCU4_IRQ	NO_IRQ
#define TEGRA_SCU5_IRQ	NO_IRQ
#define TEGRA_SCU6_IRQ	NO_IRQ
#define TEGRA_SCU7_IRQ	NO_IRQ

#if defined(CONFIG_ARCH_TEGRA_1x_SOC) || defined(CONFIG_ARCH_TEGRA_2x_SOC)
/* fixme: are the performance monitor interrupts in the relocation table?
 * if so, which module ID?
 */
#define TEGRA_CPU0_PERFMON_IRQ	(32+56)
#define TEGRA_CPU1_PERFMON_IRQ	(32+57)
#define TEGRA_CPU2_PERFMON_IRQ	NO_IRQ
#define TEGRA_CPU3_PERFMON_IRQ	NO_IRQ
#else
#error "Performance monitor IRQs need to be defined for oprofile compilation"
#endif

#endif
