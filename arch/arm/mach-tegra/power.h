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

#include "nvos.h"
#include "nvrm_init.h"
#include "nvrm_drf.h"
#include "ap20/arapbpm.h"
#include "nvrm_module.h"
#include "ap20/arflow_ctlr.h"
#include "ap20/arclk_rst.h"
#include "ap20/arapb_misc.h"
#include "ap20/arapbdma.h"
#include "ap20/arapbdmachan.h"
#include "ap20/armc.h"
#include "ap15/arictlr.h"
#include "ap15/argpio.h"
#include "nvrm_hardware_access.h"
#include "nvrm_interrupt.h"
#include "nvrm_power.h"
#include "nvrm_power_private.h"
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include "ap20/nvboot_pmc_scratch_map.h"

extern NvRmDeviceHandle s_hRmGlobal;

#define NUM_LOCAL_TIMER_REGISTERS 3
#define WAKE_PAD_MIN_LATCH_TIME_US 130
#define WAKE_PAD_MIN_SAMPLE_TIME_US 70

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

#define CAR_CLK_SOURCES_OFFSET_START	CLK_RST_CONTROLLER_CLK_SOURCE_I2S1_0
#define CAR_CLK_SOURCES_OFFSET_END		CLK_RST_CONTROLLER_CLK_SOURCE_OSC_0
#define CAR_CLK_SOURCES_REGISTER_COUNT\
	((CAR_CLK_SOURCES_OFFSET_END - CAR_CLK_SOURCES_OFFSET_START +\
		sizeof(NvU32)) / sizeof(NvU32))

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
	PowerPllA = 0,
	PowerPllC,
	PowerPllM,
	PowerPllP,
	PowerPllX
} PowerPll;

typedef enum
{
	POWER_STATE_LP2,
	POWER_STATE_LP1,
	POWER_STATE_LP0,
} PowerState;
