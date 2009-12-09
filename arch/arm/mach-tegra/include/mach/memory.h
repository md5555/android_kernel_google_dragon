/*
 * arch/arm/mach-tegra/include/mach/memory.h
 *
 * Copyright (c) 2009, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __MACH_TEGRA_MEMORY_H
#define __MACH_TEGRA_MEMORY_H

/* physical offset of RAM */
#if defined(CONFIG_ARCH_TEGRA_1x_SOC) || defined(CONFIG_ARCH_TEGRA_2x_SOC)
#define PHYS_OFFSET		UL(0)
#else
#error "Invalid Tegra SoC family selection"
#endif

/* bus address and physical addresses are identical */
#define __virt_to_bus(x)	__virt_to_phys(x)
#define __bus_to_virt(x)	__phys_to_virt(x)

#endif

