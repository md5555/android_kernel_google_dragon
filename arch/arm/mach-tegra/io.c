/*
 * arch/arm/mach-tegra/io.c
 *
 * static I/O aperture definition for NVIDIA Tegra SoCs
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

#include <linux/kernel.h>
#include <linux/init.h>

#include <mach/hardware.h>
#include <asm/io.h>
#include <asm/page.h>
#include <asm/mach/map.h>
#include "nvrm_init.h"
#include "nvrm_module.h"
#include "nvrm_hardware_access.h"
#include "nvassert.h"
#include "nvos.h"
#include "mach/nvrm_linux.h"
#include "ap15/arapbpm.h"

extern void __init tegra_nvos_kernel_init(void);
NvRmDeviceHandle s_hRmGlobal = NULL;

#define declare_aperture(_name, _pa, _size)		\
	{						\
		.virtual = tegra_munge_pa((_pa)),	\
		.pfn = __phys_to_pfn((_pa)),		\
		.length = _size,			\
		.type = MT_DEVICE,			\
	},

static struct map_desc static_apertures[] __initdata = 
{
	tegra_apertures(declare_aperture)
};


void __init tegra_map_common_io(void)
{
	iotable_init(static_apertures, ARRAY_SIZE(static_apertures));
	tegra_nvos_kernel_init();
	NvRmInit(&s_hRmGlobal);
}
