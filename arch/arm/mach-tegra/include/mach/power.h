/*
 * arch/arm/mach-tegra/power.h
 *
 * Header for tegra power
 *
 * Copyright (c) 2010, NVIDIA Corporation.
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

#ifndef	_MACH_TEGRA_POWER_H_
#define	_MACH_TEGRA_POWER_H_

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include "nvos.h"
#include "nvrm_init.h"
#include "nvrm_drf.h"
#include "nvrm_module.h"
#include "nvrm_hardware_access.h"
#include "nvrm_interrupt.h"
#include "nvrm_power.h"
#include "nvrm_power_private.h"

#if defined(CONFIG_ARCH_TEGRA_2x_SOC)
#include "power-t2.h"
#else
#error "Unsupported Tegra SOC architecture"
#endif

extern NvRmDeviceHandle s_hRmGlobal;

#define NV_CAR_REGR(pBase, reg)\
		NV_READ32( (((NvUPtr)(pBase)) + CLK_RST_CONTROLLER_##reg##_0))
#define NV_CAR_REGW(pBase, reg, val)\
		NV_WRITE32( (((NvUPtr)(pBase)) + CLK_RST_CONTROLLER_##reg##_0), (val))
#define NV_CAR_REGR_OFFSET(pBase, off)\
		NV_READ32( (((NvUPtr)(pBase)) + off))
#define NV_CAR_REGW_OFFSET(pBase, off, val)\
		NV_WRITE32( (((NvUPtr)(pBase)) + off), (val))

#define NV_ICTLR_REGR(pBase, reg)\
		NV_READ32( (((NvUPtr)(pBase)) + ICTLR_##reg##_0))
#define NV_ICTLR_REGW(pBase, reg, val)\
		NV_WRITE32( (((NvUPtr)(pBase)) + ICTLR_##reg##_0), (val))

#define NV_MISC_REGR(pBase, reg)\
		NV_READ32( (((NvUPtr)(pBase)) + APB_MISC_##reg##_0))
#define NV_MISC_REGW(pBase, reg, val)\
		NV_WRITE32( (((NvUPtr)(pBase)) + APB_MISC_##reg##_0), (val))

#define NV_GPIO_REGR(pBase, reg)\
		NV_READ32( (((NvUPtr)(pBase)) + GPIO_##reg))
#define NV_GPIO_REGW(pBase, reg, val)\
		NV_WRITE32( (((NvUPtr)(pBase)) + GPIO_##reg), (val))

#define NV_APBDMA_REGR(pBase, reg)\
		NV_READ32( (((NvUPtr)(pBase)) + APBDMA_##reg##_0))
#define NV_APBDMA_REGW(pBase, reg, val)\
		NV_WRITE32( (((NvUPtr)(pBase)) + APBDMA_##reg##_0), (val))

#define NV_APBDMACH_REGR(pBase, reg)\
		NV_READ32( (((NvUPtr)(pBase)) + APBDMACHAN_CHANNEL_0_##reg##_0))
#define NV_APBDMACH_REGW(pBase, reg, val)\
		NV_WRITE32( (((NvUPtr)(pBase)) + APBDMACHAN_CHANNEL_0_##reg##_0),(val))

#define NV_MC_REGR(pBase, reg)\
		NV_READ32( (((NvUPtr)(pBase)) + MC_##reg##_0))
#define NV_MC_REGW(pBase, reg, val)\
		NV_WRITE32( (((NvUPtr)(pBase)) + MC_##reg##_0), (val))

#define NV_PMC_REGR(pBase, reg)\
		NV_READ32( (((NvUPtr)(pBase)) + APBDEV_PMC_##reg##_0))
#define NV_PMC_REGW(pBase, reg, val)\
		NV_WRITE32( (((NvUPtr)(pBase)) + APBDEV_PMC_##reg##_0), (val))

#define NV_DR_REGR(d,r)\
		NV_READ32( ((NvUPtr)(g_p##d)) + d##_##r##_0)

typedef struct
{
	NvU32 *pBase;
	NvU32 *pContext;
}  power_module_context;

struct power_context
{
	NvU32 context_size_words;
	NvU32 *first_context_location;
	power_module_context interrupt;
	power_module_context misc;
	power_module_context clock_reset;
	power_module_context apb_dma;
	power_module_context apb_dma_chan;
	power_module_context gpio;
	power_module_context vde;
	power_module_context mc;
};

struct wakeup_source
{
	NvU32 Module;
	NvU32 Index;
};

typedef enum
{
	PowerModuleContext_Init,
	PowerModuleContext_Save,
	PowerModuleContext_Restore,
	PowerModuleContext_DisableInterrupt,
	PowerModuleContext_SaveLP1,
	PowerModuleContext_RestoreLP1,
	PowerModuleContext_Force32 = 0x7fffffff
} PowerModuleContext;

typedef enum
{
	PowerPllM = 0x1,  // Memory
	PowerPllC = 0x2,  // CPU
	PowerPllP = 0x4,  // Peripherals
	PowerPllA = 0x8,  // Audio
	PowerPllX = 0x10, // CPU Complex
	PowerPll_Force32 = 0x7fffffff
} PowerPll;

typedef enum
{
	POWER_STATE_LP0,
	POWER_STATE_LP1,
	POWER_STATE_LP2,
} PowerState;

typedef NvU16 NvIrqNumber;

#endif /* _MACH_TEGRA_POWER_H_ */

