/*
 * arch/arm/mach-tegra/include/mach/irqs.h
 *
 * Copyright (c) 2009, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __MACH_TEGRA_IRQS_H
#define __MACH_TEGRA_IRQS_H


#if defined(CONFIG_ARCH_TEGRA_1x_SOC)
#define NR_IRQS 512
#elif defined(CONFIG_ARCH_TEGRA_2x_SOC)
#define NR_IRQS 512
#else
#error "Invalid Tegra SoC family selection"
#endif

#endif
