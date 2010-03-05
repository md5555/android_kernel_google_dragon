/*
 * arch/arm/mach-tegra/power-context-t2.c
 *
 * Module save/restore routines for Tegra 2 SoCs
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

#include "power.h"

extern NvRmDeviceHandle s_hRmGlobal;
extern uintptr_t g_resume, g_contextSavePA;

static void update_registers_for_lp0(void);
static void wake_pad_configure(NvRmDeviceHandle hRmDeviceHandle);
static void setup_wakeup_events(void);
static NvU32* save_clockreset_context(PowerModuleContext Context,
				struct power_context *pAnchor, NvU32 *pCM);
static NvU32* save_intc_context(PowerModuleContext Context,
				struct power_context *pAnchor, NvU32 *pCM);
static NvU32* save_misc_context(PowerModuleContext Context,
				struct power_context *pAnchor, NvU32 *pCM);
static NvU32* save_gpio_context(PowerModuleContext Context,
				struct power_context *pAnchor, NvU32 *pCM);
static NvU32* save_apbdma_context(PowerModuleContext Context,
				struct power_context *pAnchor, NvU32 *pCM);
static NvU32* save_apbdmachan_context(PowerModuleContext Context,
				struct power_context *pAnchor, NvU32 *pCM);
static NvU32* save_mc_context(PowerModuleContext Context,
				struct power_context *pAnchor, NvU32 *pCM);
NvU32* perform_context_operation(PowerModuleContext Context);
void prepare_for_wb0(void);

struct power_context *s_pModulesContextAnchor = NULL;
extern NvU32 g_AvpWarmbootEntry;

#define MODULE_CONTEXT_SAVE_AREA_SIZE 4096

static void update_registers_for_lp0(void)
{
	NvU32 Reg;

	//SCRATCH0 : Warmbootflag
	Reg = NV_REGR(s_hRmGlobal, NvRmModuleID_Pmif, 0,
				APBDEV_PMC_SCRATCH0_0);
	Reg = NV_FLD_SET_DRF_NUM (APBDEV_PMC, SCRATCH0, WARM_BOOT0_FLAG, 1, Reg);
	NV_REGW(s_hRmGlobal, NvRmModuleID_Pmif, 0,
				APBDEV_PMC_SCRATCH0_0, Reg);

	//SCRATCH1 : AVP-side recovery physical address.
	NV_REGW(s_hRmGlobal, NvRmModuleID_Pmif, 0,
			APBDEV_PMC_SCRATCH1_0, g_AvpWarmbootEntry);

	//SCRATCH41 : CPU-side recovery physical address.
	//LP0 needs the resume address in SCRATCH41
	NV_REGW(s_hRmGlobal, NvRmModuleID_Pmif, 0,
			APBDEV_PMC_SCRATCH41_0, g_resume);
}

static void wake_pad_configure(NvRmDeviceHandle hRmDeviceHandle)
{
	NvU32 WakePadCount;
	const NvOdmWakeupPadInfo* pWakePadInfo =
		NvOdmQueryGetWakeupPadTable(&WakePadCount);
	NvU32 WakeMask, WakeLevel, WakeStatus, WakeCntrl, i;

	if ((WakePadCount == 0) || (pWakePadInfo == NULL))
		return;

	//Clear wake status register
	NV_REGW(s_hRmGlobal, NvRmModuleID_Pmif, 0,
		APBDEV_PMC_SW_WAKE_STATUS_0, 0);
	NvOsWaitUS(WAKE_PAD_MIN_LATCH_TIME_US);

	//Sample current wake pad status (on 1=>0 transition of latch
	//enable bit in PMC control register)
	WakeCntrl = NV_REGR( s_hRmGlobal, NvRmModuleID_Pmif, 0,
					APBDEV_PMC_CNTRL_0);
	WakeCntrl = NV_FLD_SET_DRF_DEF(
		APBDEV_PMC, CNTRL, LATCHWAKE_EN, ENABLE, WakeCntrl);
	NV_REGW(s_hRmGlobal, NvRmModuleID_Pmif, 0,
		APBDEV_PMC_CNTRL_0, WakeCntrl);
	NvOsWaitUS(WAKE_PAD_MIN_SAMPLE_TIME_US);
	WakeCntrl = NV_FLD_SET_DRF_DEF(
		APBDEV_PMC, CNTRL, LATCHWAKE_EN, DISABLE, WakeCntrl);
	NV_REGW(s_hRmGlobal, NvRmModuleID_Pmif, 0,
		APBDEV_PMC_CNTRL_0, WakeCntrl);
	NvOsWaitUS(WAKE_PAD_MIN_LATCH_TIME_US);

	//Read current wake mask, level, and latched status
	WakeMask = NV_REGR( s_hRmGlobal, NvRmModuleID_Pmif, 0,
				APBDEV_PMC_WAKE_MASK_0);
	WakeLevel = NV_REGR( s_hRmGlobal, NvRmModuleID_Pmif, 0,
					APBDEV_PMC_WAKE_LVL_0);
	WakeStatus = NV_REGR( s_hRmGlobal, NvRmModuleID_Pmif, 0,
		APBDEV_PMC_SW_WAKE_STATUS_0);
	/*
	 * Enable/disable wake pad, and configure polarity for enabled pads.
	 * For "any edge" wake pads toggle level if pad status has been sampled
	 * as active; it is possible that wake input has changed during sampling
	 * and configured polarity may be incorrect. However, this change would
	 * activate the respective driver via interrupt controller. Hence, this
	 * configuration routine will be re-entered after driver becomes idle
	 * again, before actual suspend happens.
	 */
	for (i = 0; i < WakePadCount; i++)
	{
		NvU32 mask = 0x1 << pWakePadInfo[i].WakeupPadNumber;
		if (pWakePadInfo[i].enable)
		{
			WakeMask |= mask;
			switch (pWakePadInfo[i].Polarity) {
			case NvOdmWakeupPadPolarity_Low:
				WakeLevel &= (~mask);
				break;
			case NvOdmWakeupPadPolarity_High:
				WakeLevel |= mask;
				break;
			case NvOdmWakeupPadPolarity_AnyEdge:
				if (WakeStatus & mask)
					WakeLevel ^= mask;
				break;
			default:
				break;
			}
		}
		else
		{
			WakeMask &= (~mask);
		}
	}
	//Write back updated wake mask, and level
	NV_REGW(s_hRmGlobal, NvRmModuleID_Pmif, 0,
		APBDEV_PMC_WAKE_MASK_0, WakeMask);
	NV_REGW(s_hRmGlobal, NvRmModuleID_Pmif, 0,
		APBDEV_PMC_WAKE_LVL_0, WakeLevel);
}

static void setup_wakeup_events(void)
{
	NvU32 Reg;

	//Configure wake pads, per their current status.
	wake_pad_configure(s_hRmGlobal);

	//Enable RTC wake event.
	Reg = NV_REGR(s_hRmGlobal, NvRmModuleID_Pmif, 0,
			APBDEV_PMC_WAKE_MASK_0);
	Reg = NV_FLD_SET_DRF_DEF(APBDEV_PMC, WAKE_MASK, RTC, ENABLE, Reg);
	NV_REGW(s_hRmGlobal, NvRmModuleID_Pmif, 0,
			APBDEV_PMC_WAKE_MASK_0, Reg);

	//Set RTC wake level.
	Reg = NV_REGR(s_hRmGlobal, NvRmModuleID_Pmif, 0,
				APBDEV_PMC_WAKE_LVL_0);
	Reg = NV_FLD_SET_DRF_DEF(APBDEV_PMC, WAKE_LVL, RTC, ACTIVE_HIGH, Reg);
	NV_REGW(s_hRmGlobal, NvRmModuleID_Pmif, 0,
				APBDEV_PMC_WAKE_LVL_0, Reg);
}

static NvU32* save_clockreset_context(
		PowerModuleContext Context,
		struct power_context *pAnchor,
		NvU32 *pCM)
{
	NvU32			Aperture;		// Register base address
	NvU32			ApertureSize;	// Register set size
	NvRmModuleID	ModuleId;		// Composite module id & instance
	NvU32			Offset;			// Register offset
	NvU32			OscCtrl1;		// Scratch register
	NvU32			OscCtrl2;		// Scratch register
	NvU32			BitVal;			// Scratch register
	NvU32*			pBase;			// Pointer to register base addresses

	//Get a pointer to the base address for the registers.
	pBase = pAnchor->clock_reset.pBase;
	switch (Context) {
	case PowerModuleContext_Init:
		//Already initialized?
		if (pBase == NULL)
		{
			//Generate the composite module id and instance.
			ModuleId = NVRM_MODULE_ID(NvRmPrivModuleID_ClockAndReset, 0);

			//Retrieve the register base PA
			NvRmModuleGetBaseAddress(s_hRmGlobal, ModuleId,
					&Aperture, &ApertureSize);

			//Get the corresponding VA
			if (NvOsPhysicalMemMap(Aperture,
					ApertureSize,
					NvOsMemAttribute_Uncached,
					NVOS_MEM_READ_WRITE,
					(void*)&(pAnchor->clock_reset.pBase)))
			{
				printk("Failed to map the context save area!\n");
				goto fail;
			}
		}
		break;
	case PowerModuleContext_Save:
		//Register base address must have been set
		//by PowerModuleContext_Init.
		if (pBase == NULL)
			goto fail;

		//Anchor the starting point for this controller's context.
		pAnchor->clock_reset.pContext = pCM;

		//Save clock sources for individual modules
		Offset = CAR_CLK_SOURCES_OFFSET_START;
		do
		{
			//Skip saving MC and EMC CAR values.
			if (Offset != CLK_RST_CONTROLLER_CLK_SOURCE_EMC_0)
			{
				*pCM++ = NV_CAR_REGR_OFFSET(pBase, Offset);
			}
			Offset += sizeof(NvU32);
		} while (Offset <= CAR_CLK_SOURCES_OFFSET_END);

		//Save oscillator control register
		*pCM++ = NV_CAR_REGR(pBase, OSC_CTRL);

		//Save module reset state
		*pCM++ = NV_CAR_REGR(pBase, RST_DEVICES_L);
		*pCM++ = NV_CAR_REGR(pBase, RST_DEVICES_H);
		*pCM++ = NV_CAR_REGR(pBase, RST_DEVICES_U);
		//Save module clock enable state
		*pCM++ = NV_CAR_REGR(pBase, CLK_OUT_ENB_L);
		*pCM++ = NV_CAR_REGR(pBase, CLK_OUT_ENB_H);
		*pCM++ = NV_CAR_REGR(pBase, CLK_OUT_ENB_U);
		//Save clock mask register
		*pCM++ = NV_CAR_REGR(pBase, CLK_MASK_ARM);
		break;
	case PowerModuleContext_Restore:
		//Register base address must have been set
		//by PowerModuleContext_Init.
		if (pBase == NULL)
			goto fail;

		//We should be at the same place in the context
		//as when we saved things.
		if (pCM != pAnchor->clock_reset.pContext)
			goto fail;

		//Retrieve the anchored starting point for this controller's context.
		pCM = pAnchor->clock_reset.pContext;

		if (pCM == NULL)
			goto fail;

		//Force PLLA in bypass mode, until Rm gets a chance to
		//initialize it back again. This is to ensure all
		//downstream modules depending on PLLA get a known clock
		//when they are taken out of reset.
		NV_CAR_REGW(pBase, PLLA_BASE,
			NV_DRF_NUM(CLK_RST_CONTROLLER, PLLA_BASE, PLLA_BYPASS, 0x1));

		//Force PLLP in bypass mode [bug 455955], until Rm gets a
		//chance to initialize it back again. This is to ensure
		//that all downstream modules depending on PLLP get a known
		//clock when they are taken out of reset.
		NV_CAR_REGW(pBase, PLLP_BASE,
			NV_DRF_NUM(CLK_RST_CONTROLLER, PLLP_BASE, PLLP_BYPASS, 0x1));

		//The restore sequence below follows a h/w recommended
		//way of bringing up the chip. Remember, in our case the
		//bootrom would have run on a LP0 exit. Hopefully, it does the
		//correct thing.

		//Enable clock for all modules
		NV_CAR_REGW(pBase, CLK_OUT_ENB_L,
			CLK_RST_CONTROLLER_CLK_OUT_ENB_L_0_RESET_MASK);
		NV_CAR_REGW(pBase, CLK_OUT_ENB_H,
			CLK_RST_CONTROLLER_CLK_OUT_ENB_H_0_RESET_MASK);
		NV_CAR_REGW(pBase, CLK_OUT_ENB_U,
			CLK_RST_CONTROLLER_CLK_OUT_ENB_U_0_RESET_MASK);
		//Restore clock sources for individual modules
		Offset = CAR_CLK_SOURCES_OFFSET_START;
		do
		{
			//Skip MC and EMC restoration. Keep bootrom settings until
			//DVFS gets a chance to change it.
			if (Offset != CLK_RST_CONTROLLER_CLK_SOURCE_EMC_0)
			{
				NV_CAR_REGW_OFFSET(pBase, Offset, *pCM++);
			}
			Offset += sizeof(NvU32);
		} while (Offset <= CAR_CLK_SOURCES_OFFSET_END);

		//Restore oscillator control register-drive strength+osc bypass only.
		OscCtrl1 = *pCM++;
		OscCtrl2 = NV_CAR_REGR(pBase, OSC_CTRL);
		BitVal   = NV_DRF_VAL(CLK_RST_CONTROLLER,
						OSC_CTRL, XOFS, OscCtrl1);
		OscCtrl2 = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
						OSC_CTRL, XOFS, BitVal, OscCtrl2);
		BitVal   = NV_DRF_VAL(CLK_RST_CONTROLLER,
						OSC_CTRL, XOBP, OscCtrl1);
		OscCtrl2 = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
						OSC_CTRL, XOBP, BitVal, OscCtrl2);
		NV_CAR_REGW(pBase, OSC_CTRL, OscCtrl2);

		//Restore module reset state
		NV_CAR_REGW(pBase, RST_DEVICES_L, *pCM++);
		NV_CAR_REGW(pBase, RST_DEVICES_H, *pCM++);
		NV_CAR_REGW(pBase, RST_DEVICES_U, *pCM++);
		//Restore module clock enable state
		NV_CAR_REGW(pBase, CLK_OUT_ENB_L, *pCM++);
		NV_CAR_REGW(pBase, CLK_OUT_ENB_H, *pCM++);
		NV_CAR_REGW(pBase, CLK_OUT_ENB_U, *pCM++);
		//Restore clock mask register
		NV_CAR_REGW(pBase, CLK_MASK_ARM, *pCM++);
		break;

	case PowerModuleContext_SaveLP1:
	case PowerModuleContext_RestoreLP1:
	case PowerModuleContext_DisableInterrupt:
		//Do nothing.
		break;

	default:
		break;
	}

	return pCM;

fail:
	return NULL;
}

static NvU32* save_intc_context(
		PowerModuleContext Context,
		struct power_context *pAnchor,
		NvU32 *pCM)
{
	NvU32			Aperture;		//Register base address
	NvU32			ApertureSize;	//Register set size
	NvRmModuleID	ModuleId;		//Composite module id & instance
	NvU32			Instance;		//Interrupt Controller Instance
	NvU32			Instances;		//Total number of ictlr instances
	NvU32*			pBase;			//Ptr to list of register base addresses

	//NOTE: This controller has multiple instances. Therefore,
	//pAnchor->interrupt.pBase is a pointer to a list of
	//controller instance base addresses and not the controller's
	//base address itself.

	//Get the number of interrupt controller instances.
	Instances = NvRmModuleGetNumInstances(s_hRmGlobal,
					NvRmPrivModuleID_Interrupt);

	//Get a pointer to the base address list.
	pBase = pAnchor->interrupt.pBase;

	switch (Context) {
	case PowerModuleContext_Init:
		//Already initialized?
		if (pBase == NULL)
		{
			//Anchor the starting point for the list of
			//instance base addresses.
			pAnchor->interrupt.pBase = pCM;

			//For each instance...
			for (Instance = 0; Instance < Instances; ++Instance)
			{
				//Generate the composite module id and instance.
				ModuleId = NVRM_MODULE_ID(NvRmPrivModuleID_Interrupt,
								Instance);

				//Retrieve the register base PA
				NvRmModuleGetBaseAddress(s_hRmGlobal, ModuleId,
						&Aperture, &ApertureSize);

				//Get the corresponding VA
				if (NvOsPhysicalMemMap(Aperture,
						ApertureSize,
						NvOsMemAttribute_Uncached,
						NVOS_MEM_READ_WRITE,
						(void*)pCM))
				{
					printk("Failed to map the context save area!\n");
					goto fail;
				}
				pCM++;
			}
		}
		break;

	case PowerModuleContext_Save:
	case PowerModuleContext_SaveLP1:
		//Register base address list must have been
		//set by PowerModuleContext_Init.
		if (pBase == NULL)
			goto fail;

		//Anchor the starting point for this controller's context.
		pAnchor->interrupt.pContext = pCM;

		//For each instance...
		for (Instance = 0; Instance < Instances; ++Instance, ++pBase)
		{
			//Register base address must have been
			//set by PowerModuleContext_Init.
			if (*pBase == 0)
				goto fail;

			//Save module registers...
			*pCM++ = NV_ICTLR_REGR(*pBase, CPU_IER);
			*pCM++ = NV_ICTLR_REGR(*pBase, CPU_IEP_CLASS);
			*pCM++ = NV_ICTLR_REGR(*pBase, COP_IER);
			*pCM++ = NV_ICTLR_REGR(*pBase, COP_IEP_CLASS);
		}
		break;

		case PowerModuleContext_Restore:
		case PowerModuleContext_RestoreLP1:
		//Register base address list must have been
		//set by PowerModuleContext_Init.
		if (pBase == NULL)
			goto fail;

		//We should be at the same place in the context
		//as when we saved things.
		if (pCM != pAnchor->interrupt.pContext)
			goto fail;

		//Retrieve the anchored starting point for this controller's context.
		pCM = pAnchor->interrupt.pContext;

		if (pCM == NULL)
			goto fail;

		//For each instance...
		for (Instance = 0; Instance < Instances; ++Instance, ++pBase)
		{
			//Register base address must have been
			//set by PowerModuleContext_Init.
			if (*pBase == 0)
				goto fail;

			//Restore module registers...
			NV_ICTLR_REGW(*pBase, CPU_IER_SET,   *pCM++);
			NV_ICTLR_REGW(*pBase, CPU_IEP_CLASS, *pCM++);
			NV_ICTLR_REGW(*pBase, COP_IER_SET,   *pCM++);
			NV_ICTLR_REGW(*pBase, COP_IEP_CLASS, *pCM++);
		}
		break;

	case PowerModuleContext_DisableInterrupt:
		//For each instance...
		for (Instance = 0; Instance < Instances; ++Instance, ++pBase)
		{
			if (*pBase == 0)
				goto fail;

			//Disable module interrupts.
			NV_ICTLR_REGW(*pBase, CPU_IER_CLR, ~0);
			NV_ICTLR_REGW(*pBase, COP_IER_CLR, ~0);
		}
		break;

	default:
		break;
	}

	return pCM;

fail:
	return NULL;
}

static NvU32* save_misc_context(
		PowerModuleContext Context,
		struct power_context *pAnchor,
		NvU32 *pCM)
{
	NvU32*			pBase;			//Pointer to register base addresses
	NvU32			Aperture;		// Register base address
	NvU32			ApertureSize;	// Register set size
	NvRmModuleID	ModuleId;		// Composite module id & instance

	//Get a pointer to the base address for the registers.
	pBase = pAnchor->misc.pBase;

	switch (Context) {
	case PowerModuleContext_Init:
		//Already initialized?
		if (pBase == NULL)
		{
			//Generate the composite module id and instance.
			ModuleId = NVRM_MODULE_ID(NvRmModuleID_Misc, 0);

			//Retrieve the register base PA
			NvRmModuleGetBaseAddress(s_hRmGlobal, ModuleId,
					&Aperture, &ApertureSize);

			//Get the corresponding VA
			if (NvOsPhysicalMemMap(Aperture,
					ApertureSize,
					NvOsMemAttribute_Uncached,
					NVOS_MEM_READ_WRITE,
					(void*)&(pAnchor->misc.pBase)))
			{
				printk("Failed to map the context save area!\n");
				goto fail;
			}
		}
		break;
	case PowerModuleContext_Save:
		//Register base address must have been set by PowerModuleContext_Init.
		if (pBase == NULL)
			goto fail;

		//Anchor the starting point for this controller's context.
		pAnchor->misc.pContext = pCM;

		*pCM++ = NV_MISC_REGR(pBase, PP_TRISTATE_REG_A);
		*pCM++ = NV_MISC_REGR(pBase, PP_TRISTATE_REG_B);
		*pCM++ = NV_MISC_REGR(pBase, PP_TRISTATE_REG_C);
		*pCM++ = NV_MISC_REGR(pBase, PP_TRISTATE_REG_D);

		*pCM++ = NV_MISC_REGR(pBase, PP_PIN_MUX_CTL_A);
		*pCM++ = NV_MISC_REGR(pBase, PP_PIN_MUX_CTL_B);
		*pCM++ = NV_MISC_REGR(pBase, PP_PIN_MUX_CTL_C);
		*pCM++ = NV_MISC_REGR(pBase, PP_PIN_MUX_CTL_D);
		*pCM++ = NV_MISC_REGR(pBase, PP_PIN_MUX_CTL_E);
		*pCM++ = NV_MISC_REGR(pBase, PP_PIN_MUX_CTL_F);
		*pCM++ = NV_MISC_REGR(pBase, PP_PIN_MUX_CTL_G);
		*pCM++ = NV_MISC_REGR(pBase, PP_PIN_MUX_CTL_H);

		*pCM++ = NV_MISC_REGR(pBase, PP_PULLUPDOWN_REG_A);
		*pCM++ = NV_MISC_REGR(pBase, PP_PULLUPDOWN_REG_B);
		*pCM++ = NV_MISC_REGR(pBase, PP_PULLUPDOWN_REG_C);
		*pCM++ = NV_MISC_REGR(pBase, PP_PULLUPDOWN_REG_D);
		*pCM++ = NV_MISC_REGR(pBase, PP_PULLUPDOWN_REG_E);
		break;
	case PowerModuleContext_Restore:
		//Register base address must have been set by PowerModuleContext_Init.
		if (pBase == NULL)
			goto fail;

		//We should be at the same place in the context
		//as when we saved things.
		if (pCM != pAnchor->misc.pContext)
			goto fail;

		//Retrieve the anchored starting point for this controller's context.
		pCM = pAnchor->misc.pContext;
		if (pCM == NULL)
			goto fail;

		NV_MISC_REGW(pBase, PP_TRISTATE_REG_A, *pCM++);
		NV_MISC_REGW(pBase, PP_TRISTATE_REG_B, *pCM++);
		NV_MISC_REGW(pBase, PP_TRISTATE_REG_C, *pCM++);
		NV_MISC_REGW(pBase, PP_TRISTATE_REG_D, *pCM++);

		NV_MISC_REGW(pBase, PP_PIN_MUX_CTL_A, *pCM++);
		NV_MISC_REGW(pBase, PP_PIN_MUX_CTL_B, *pCM++);
		NV_MISC_REGW(pBase, PP_PIN_MUX_CTL_C, *pCM++);
		NV_MISC_REGW(pBase, PP_PIN_MUX_CTL_D, *pCM++);
		NV_MISC_REGW(pBase, PP_PIN_MUX_CTL_E, *pCM++);
		NV_MISC_REGW(pBase, PP_PIN_MUX_CTL_F, *pCM++);
		NV_MISC_REGW(pBase, PP_PIN_MUX_CTL_G, *pCM++);
		NV_MISC_REGW(pBase, PP_PIN_MUX_CTL_H, *pCM++);

		NV_MISC_REGW(pBase, PP_PULLUPDOWN_REG_A, *pCM++);
		NV_MISC_REGW(pBase, PP_PULLUPDOWN_REG_B, *pCM++);
		NV_MISC_REGW(pBase, PP_PULLUPDOWN_REG_C, *pCM++);
		NV_MISC_REGW(pBase, PP_PULLUPDOWN_REG_D, *pCM++);
		NV_MISC_REGW(pBase, PP_PULLUPDOWN_REG_E, *pCM++);
		break;
	case PowerModuleContext_SaveLP1:
	case PowerModuleContext_RestoreLP1:
	case PowerModuleContext_DisableInterrupt:
		//Do nothing.
		break;
	default:
		break;
	}

	return pCM;
fail:
	return NULL;
}

static NvU32* save_gpio_context(
		PowerModuleContext Context,
		struct power_context *pAnchor,
		NvU32 *pCM)
{
	NvU32			Aperture;		// Register base address
	NvU32			ApertureSize;	// Register set size
	NvRmModuleID	ModuleId;		// Composite module id & instance
	NvU8			Instance;		// GPIO Instance
	NvU32			Instances;		// Total number of gpio instances
	NvU32*			pBase;			// Ptr to list of register base addresses

	//NOTE: This controller has multiple instances.
	//Therefore, pAnchor->Gpio.pBase is a pointer to a list
	//of controller instance base addresses and not the
	//controller's base address itself.

	//Get the number of GPIO controller instances.
	Instances = NvRmModuleGetNumInstances(s_hRmGlobal, NvRmPrivModuleID_Gpio);

	//Get a pointer to the base address list.
	pBase = pAnchor->gpio.pBase;

	switch (Context) {
	case PowerModuleContext_Init:
		//Already initialized?
		if (pBase == NULL)
		{
			//Anchor the starting point for the list
			//of instance base addresses.
			pAnchor->gpio.pBase = pCM;

			//For each instance...
			for (Instance = 0; Instance < Instances; ++Instance)
			{
				//Generate the composite module id and instance.
				ModuleId = NVRM_MODULE_ID(NvRmPrivModuleID_Gpio, Instance);

				//Retrieve the register base PA
				NvRmModuleGetBaseAddress(s_hRmGlobal, ModuleId,
						&Aperture, &ApertureSize);

				//Get the corresponding VA
				if (NvOsPhysicalMemMap(Aperture,
						ApertureSize,
						NvOsMemAttribute_Uncached,
						NVOS_MEM_READ_WRITE,
						(void*)pCM))
			   {
					printk("Failed to map the context save area!\n");
					goto fail;
			   }
				pCM++;
			}
		}
		break;
	case PowerModuleContext_Save:
		//Register base address list must have been
		//set by PowerModuleContext_Init.
		if (pBase == NULL)
			goto fail;

		//Anchor the starting point for this controller's context.
		pAnchor->gpio.pContext = pCM;

		//For each instance...
		for (Instance = 0; Instance < Instances; ++Instance, ++pBase)
		{
			//Register base address must have been
			//set by PowerModuleContext_Init.
			if (*pBase == 0)
				goto fail;

			//Save the GPIO configuration.
			*pCM++ = NV_GPIO_REGR(*pBase, CNF_0);
			*pCM++ = NV_GPIO_REGR(*pBase, CNF_1);
			*pCM++ = NV_GPIO_REGR(*pBase, CNF_2);
			*pCM++ = NV_GPIO_REGR(*pBase, CNF_3);

			//Save the GPIO output settings.
			*pCM++ = NV_GPIO_REGR(*pBase, OUT_0);
			*pCM++ = NV_GPIO_REGR(*pBase, OUT_1);
			*pCM++ = NV_GPIO_REGR(*pBase, OUT_2);
			*pCM++ = NV_GPIO_REGR(*pBase, OUT_3);

			//Save the GPIO output enable settings.
			*pCM++ = NV_GPIO_REGR(*pBase, OE_0);
			*pCM++ = NV_GPIO_REGR(*pBase, OE_1);
			*pCM++ = NV_GPIO_REGR(*pBase, OE_2);
			*pCM++ = NV_GPIO_REGR(*pBase, OE_3);

			//Save the GPIO level enables.
			*pCM++ = NV_GPIO_REGR(*pBase, INT_ENB_0);
			*pCM++ = NV_GPIO_REGR(*pBase, INT_ENB_1);
			*pCM++ = NV_GPIO_REGR(*pBase, INT_ENB_2);
			*pCM++ = NV_GPIO_REGR(*pBase, INT_ENB_3);

			//Save the GPIO
			*pCM++ = NV_GPIO_REGR(*pBase, INT_LVL_0);
			*pCM++ = NV_GPIO_REGR(*pBase, INT_LVL_1);
			*pCM++ = NV_GPIO_REGR(*pBase, INT_LVL_2);
			*pCM++ = NV_GPIO_REGR(*pBase, INT_LVL_3);
		}
		break;
	case PowerModuleContext_Restore:
		//Register base address list must have been
		//set by PowerModuleContext_Init.
		if (pBase == NULL)
			goto fail;

		//We should be at the same place in the context
		//as when we saved things.
		if (pCM != pAnchor->gpio.pContext)
			goto fail;

		//Retrieve the anchored starting point for this controller's context.
		pCM = pAnchor->gpio.pContext;
		if (pCM == NULL)
			goto fail;

		//For each instance...
		for (Instance = 0; Instance < Instances; ++Instance, ++pBase)
		{
			//Register base address must have been
			//set by PowerModuleContext_Init.
			if (*pBase == 0)
				goto fail;

			//Restore the GPIO configuration.
			NV_GPIO_REGW(*pBase, CNF_0, *pCM++);
			NV_GPIO_REGW(*pBase, CNF_1, *pCM++);
			NV_GPIO_REGW(*pBase, CNF_2, *pCM++);
			NV_GPIO_REGW(*pBase, CNF_3, *pCM++);

			//Restore the GPIO output settings.
			NV_GPIO_REGW(*pBase, OUT_0, *pCM++);
			NV_GPIO_REGW(*pBase, OUT_1, *pCM++);
			NV_GPIO_REGW(*pBase, OUT_2, *pCM++);
			NV_GPIO_REGW(*pBase, OUT_3, *pCM++);

			//Restore the GPIO output enable settings.
			NV_GPIO_REGW(*pBase, OE_0, *pCM++);
			NV_GPIO_REGW(*pBase, OE_1, *pCM++);
			NV_GPIO_REGW(*pBase, OE_2, *pCM++);
			NV_GPIO_REGW(*pBase, OE_3, *pCM++);

			//Restore the GPIO level enables.
			NV_GPIO_REGW(*pBase, INT_ENB_0, *pCM++);
			NV_GPIO_REGW(*pBase, INT_ENB_1, *pCM++);
			NV_GPIO_REGW(*pBase, INT_ENB_2, *pCM++);
			NV_GPIO_REGW(*pBase, INT_ENB_3, *pCM++);

			//Restore the shadowed GPIO level settings.
			NV_GPIO_REGW(*pBase, INT_LVL_0, *pCM++);
			NV_GPIO_REGW(*pBase, INT_LVL_1, *pCM++);
			NV_GPIO_REGW(*pBase, INT_LVL_2, *pCM++);
			NV_GPIO_REGW(*pBase, INT_LVL_3, *pCM++);
		}
		break;
	case PowerModuleContext_SaveLP1:
		//Register base address list must have been
		//set by PowerModuleContext_Init.
		if (pBase == NULL)
			goto fail;

		//Anchor the starting point for this controller's context.
		pAnchor->gpio.pContext = pCM;

		//For each instance...
		for (Instance = 0; Instance < Instances; ++Instance, ++pBase)
		{
			//Register base address must have been
			//set by PowerModuleContext_Init.
			if (*pBase == 0)
				goto fail;

			//Save the GPIO enables.
			*pCM++ = NV_GPIO_REGR(*pBase, INT_ENB_0);
			*pCM++ = NV_GPIO_REGR(*pBase, INT_ENB_1);
			*pCM++ = NV_GPIO_REGR(*pBase, INT_ENB_2);
			*pCM++ = NV_GPIO_REGR(*pBase, INT_ENB_3);
		}
		break;
	case PowerModuleContext_RestoreLP1:
		//Register base address list must have been
		//set by PowerModuleContext_Init.
		if (pBase == NULL)
			goto fail;

		//We should be at the same place in the context
		//as when we saved things.
		if (pCM != pAnchor->gpio.pContext)
			goto fail;

		//For each instance...
		for (Instance = 0; Instance < Instances; ++Instance, ++pBase)
		{
			//Register base address must have been
			//set by PowerModuleContext_Init.
			if (*pBase == 0)
				goto fail;

			// Restore the GPIO enables.
			NV_GPIO_REGW(*pBase, INT_ENB_0, *pCM++);
			NV_GPIO_REGW(*pBase, INT_ENB_1, *pCM++);
			NV_GPIO_REGW(*pBase, INT_ENB_2, *pCM++);
			NV_GPIO_REGW(*pBase, INT_ENB_3, *pCM++);
		}
		break;
	case PowerModuleContext_DisableInterrupt:
		//Register base address list must have been
		//set by PowerModuleContext_Init.
		if (pBase == NULL)
			goto fail;

		//For each instance...
		for (Instance = 0; Instance < Instances; ++Instance, ++pBase)
		{
			//Register base address must have been
			//set by PowerModuleContext_Init.
			if (*pBase == 0)
				goto fail;

			//Disable module interrupts.
			NV_GPIO_REGW(*pBase, INT_ENB_0, 0);
			NV_GPIO_REGW(*pBase, INT_ENB_1, 0);
			NV_GPIO_REGW(*pBase, INT_ENB_2, 0);
			NV_GPIO_REGW(*pBase, INT_ENB_3, 0);
		}
		break;
	default:
		break;
	}

	return pCM;
fail:
	return NULL;
}

static NvU32* save_apbdma_context(
		PowerModuleContext Context,
		struct power_context *pAnchor,
		NvU32 *pCM)
{
	NvU32			Aperture;		//Register base address
	NvU32			ApertureSize;	//Register set size
	NvRmModuleID	ModuleId;		//Composite module id & instance
	NvU32*			pBase;			//Pointer to register base addresses

	//Get a pointer to the base address for the registers.
	pBase = pAnchor->apb_dma.pBase;

	switch (Context) {
	case PowerModuleContext_Init:
		//Already initialized?
		if (pBase == NULL)
		{
			//Generate the composite module id and instance.
			ModuleId = NVRM_MODULE_ID(NvRmPrivModuleID_ApbDma, 0);

			//Retrieve the register base PA
			NvRmModuleGetBaseAddress(s_hRmGlobal, ModuleId,
					&Aperture, &ApertureSize);

			//Get the corresponding VA
			if (NvOsPhysicalMemMap(Aperture,
					ApertureSize,
					NvOsMemAttribute_Uncached,
					NVOS_MEM_READ_WRITE,
					(void*)&(pAnchor->apb_dma.pBase)))
			{
				printk("Failed to map the context save area!\n");
				goto fail;
			}
		}
		break;
	case PowerModuleContext_Save:
		//Register base address must have been set by PowerModuleContext_Init.
		if (pBase == NULL)
			goto fail;

		//Anchor the starting point for this controller's context.
		pAnchor->apb_dma.pContext = pCM;

		*pCM++ = NV_APBDMA_REGR(pBase, COMMAND);
		*pCM++ = NV_APBDMA_REGR(pBase, CNTRL_REG);
		*pCM++ = NV_APBDMA_REGR(pBase, IRQ_MASK);
		break;
	case PowerModuleContext_Restore:
		//Register base address must have been set by PowerModuleContext_Init.
		if (pBase == NULL)
			goto fail;

		//We should be at the same place in the context
		//as when we saved things.
		if (pCM != pAnchor->apb_dma.pContext || pCM == NULL)
			goto fail;

		NV_APBDMA_REGW(pBase, COMMAND, *pCM++);
		NV_APBDMA_REGW(pBase, CNTRL_REG, *pCM++);
		NV_APBDMA_REGW(pBase, IRQ_MASK_SET, *pCM++);
		break;
	case PowerModuleContext_SaveLP1:
	case PowerModuleContext_RestoreLP1:
	case PowerModuleContext_DisableInterrupt:
		break;
	default:
		break;
	}

	return pCM;
fail:
	return NULL;
}

static NvU32* save_apbdmachan_context(
		PowerModuleContext Context,
		struct power_context *pAnchor,
		NvU32 *pCM)
{
	NvU32			Aperture;		//Register base address
	NvU32			ApertureSize;	//Register set size
	NvRmModuleID	ModuleId;		//Composite module id & instance
	NvU32			Instance;		//DMA Channel Instances
	NvU32			Instances;		//Total number of dma instances
	NvU32*			pBase;			//Ptr to list of register base addresses

	//NOTE: This controller has multiple instances.
	//Therefore, pAnchor->ApbDmaChannel.pBase is a pointer
	//to a list of controller instance base addresses and not the
	//controller's base address itself.

	//Get the number of APB DMA channels.
	Instances = NvRmModuleGetNumInstances(s_hRmGlobal,
					NvRmPrivModuleID_ApbDmaChannel);

	// Get a pointer to the base address list.
	pBase = pAnchor->apb_dma_chan.pBase;

	switch (Context) {
	case PowerModuleContext_Init:
		//Already initialized?
		if (pBase == NULL)
		{
			//Anchor the starting point for the list
			//of instance base addresses.
			pAnchor->apb_dma_chan.pBase = pCM;

			//For each instance...
			for (Instance = 0; Instance < Instances; ++Instance)
			{
				//Generate the composite module id and instance.
				ModuleId = NVRM_MODULE_ID(NvRmPrivModuleID_ApbDmaChannel,
								Instance);

				//Retrieve the register base PA
				NvRmModuleGetBaseAddress(s_hRmGlobal, ModuleId,
						&Aperture, &ApertureSize);

				//Get the corresponding VA
				if (NvOsPhysicalMemMap(Aperture,
						ApertureSize,
						NvOsMemAttribute_Uncached,
						NVOS_MEM_READ_WRITE,
						(void*)pCM))
				{
					printk("Failed to map the context save area!\n");
					goto fail;
				}
				pCM++;
			}
		}
		break;
	case PowerModuleContext_Save:
		//Register base address list must have been
		//set by PowerModuleContext_Init.
		if (pBase == NULL)
			goto fail;

		//Anchor the starting point for this controller's context.
		pAnchor->apb_dma_chan.pContext = pCM;

		//For each instance...
		for (Instance = 0; Instance < Instances; ++Instance, ++pBase)
		{
			//Register base address must have been
			//set by PowerModuleContext_Init.
			if (*pBase == 0)
				goto fail;

			*pCM++ = NV_APBDMACH_REGR(*pBase, CSR);
			*pCM++ = NV_APBDMACH_REGR(*pBase, STA);
			*pCM++ = NV_APBDMACH_REGR(*pBase, AHB_PTR);
			*pCM++ = NV_APBDMACH_REGR(*pBase, AHB_SEQ);
			*pCM++ = NV_APBDMACH_REGR(*pBase, APB_PTR);
			*pCM++ = NV_APBDMACH_REGR(*pBase, APB_SEQ);
		}
		break;
	case PowerModuleContext_Restore:
		//Register base address list must have been
		//set by PowerModuleContext_Init.
		if (pBase == NULL)
			goto fail;

		//We should be at the same place in the context
		//as when we saved things.
		if (pCM != pAnchor->apb_dma_chan.pContext || pCM == NULL)
			goto fail;

		//For each instance...
		for (Instance = 0; Instance < Instances; ++Instance, ++pBase)
		{
			//Register base address must have been
			//set by PowerModuleContext_Init.
			if (*pBase == 0)
				goto fail;

			NV_APBDMACH_REGW(*pBase, CSR, *pCM++);
			NV_APBDMACH_REGW(*pBase, STA, *pCM++);
			NV_APBDMACH_REGW(*pBase, AHB_PTR, *pCM++);
			NV_APBDMACH_REGW(*pBase, AHB_SEQ, *pCM++);
			NV_APBDMACH_REGW(*pBase, APB_PTR, *pCM++);
			NV_APBDMACH_REGW(*pBase, APB_SEQ, *pCM++);
		}
		break;
	case PowerModuleContext_SaveLP1:
	case PowerModuleContext_RestoreLP1:
	case PowerModuleContext_DisableInterrupt:
		break;
	default:
		break;
	}

	return pCM;
fail:
	return NULL;
}

static NvU32* save_mc_context(
		PowerModuleContext Context,
		struct power_context *pAnchor,
		NvU32 *pCM)
{
	NvU32*			pBase;			//Pointer to register base addresses
	NvU32			Aperture;		//Register base address
	NvU32			ApertureSize;	//Register set size
	NvRmModuleID	ModuleId;		//Composite module id & instance

	//Get a pointer to the base address for the registers.
	pBase = pAnchor->mc.pBase;

	switch (Context) {
	case PowerModuleContext_Init:
		//Already initialized?
		if (pBase == NULL)
		{
			//Generate the composite module id and instance.
			ModuleId = NVRM_MODULE_ID(NvRmPrivModuleID_MemoryController, 0);

			//Retrieve the register base PA
			NvRmModuleGetBaseAddress(s_hRmGlobal, ModuleId,
					&Aperture, &ApertureSize);

			//Get the corresponding VA
			if (NvOsPhysicalMemMap(Aperture,
					ApertureSize,
					NvOsMemAttribute_Uncached,
					NVOS_MEM_READ_WRITE,
					(void*)&(pAnchor->mc.pBase)))
			{
				printk("Failed to map the context save area!\n");
				goto fail;
			}
		}
		break;
	case PowerModuleContext_Save:
		//Register base address must have been set by PowerModuleContext_Init.
		if (pBase == NULL)
			goto fail;

		//Anchor the starting point for this controller's context.
		pAnchor->mc.pContext = pCM;

		*pCM++ = NV_MC_REGR(pBase, SECURITY_CFG0);
		*pCM++ = NV_MC_REGR(pBase, SECURITY_CFG1);
		break;
	case PowerModuleContext_Restore:
		//Register base address must have been set by PowerModuleContext_Init.
		if (pBase == NULL)
			goto fail;

		//We should be at the same place in the context
		//as when we saved things.
		if (pCM != pAnchor->mc.pContext || pCM == NULL)
			goto fail;

		NV_MC_REGW(pBase, SECURITY_CFG0, *pCM++);
		NV_MC_REGW(pBase, SECURITY_CFG1, *pCM++);
		break;
	case PowerModuleContext_SaveLP1:
	case PowerModuleContext_RestoreLP1:
	case PowerModuleContext_DisableInterrupt:
		break;
	default:
		break;
	}

	return pCM;
fail:
	return NULL;
}

NvU32* perform_context_operation(PowerModuleContext Context)
{
	NvU32*  pCM;	// Pointer to next available save area location

	//First save area location available for module use is located just
	//beyond the anchor structure.
	if (s_pModulesContextAnchor == NULL)
		return NULL;

	//Initializing?
	if (Context == PowerModuleContext_Init)
	{
		//The context pointer start immediately following the anchor structure.
		pCM = (NvU32*)(((NvU8*)s_pModulesContextAnchor) +
					sizeof(*s_pModulesContextAnchor));
	}
	else
	{
		//Use the starting point for module contexts.
		pCM = s_pModulesContextAnchor->first_context_location;
	}

	pCM = save_clockreset_context(Context, s_pModulesContextAnchor, pCM);
	pCM = save_intc_context(Context, s_pModulesContextAnchor, pCM);
	pCM = save_misc_context(Context, s_pModulesContextAnchor, pCM);
	pCM = save_gpio_context(Context, s_pModulesContextAnchor, pCM);
	pCM = save_apbdma_context(Context, s_pModulesContextAnchor, pCM);
	pCM = save_apbdmachan_context(Context, s_pModulesContextAnchor, pCM);
	pCM = save_mc_context(Context, s_pModulesContextAnchor, pCM);

	//Check if we went past the context area limits.
	if (pCM >= ((NvU32*)s_pModulesContextAnchor +
			s_pModulesContextAnchor->context_size_words))
		return NULL;

	return pCM;
}

void module_context_init(void)
{
	struct power_context * pAnchor;

	//Get the address of the module context save area. The anchor structure
	//will be placed here.
	pAnchor = kmalloc(MODULE_CONTEXT_SAVE_AREA_SIZE, GFP_ATOMIC);

	//Clear the context anchor.
	NvOsMemset(pAnchor, 0, sizeof(*pAnchor));

	//Initialize the anchor structure.
	pAnchor->context_size_words = MODULE_CONTEXT_SAVE_AREA_SIZE/sizeof(NvU32);

	//Save the global anchor pointer.
	s_pModulesContextAnchor = pAnchor;

	//Initialize module contexts.
	pAnchor->first_context_location =
			perform_context_operation(PowerModuleContext_Init);

	if (pAnchor->first_context_location == NULL)
		printk("module_context_init failed!\n");
}

void prepare_for_wb0(void)
{
	update_registers_for_lp0();

	//Save pointer to context in the scratch register reserved
	//for that purpose.
	NV_REGW(s_hRmGlobal, NvRmModuleID_Pmif, 0,
				APBDEV_PMC_SCRATCH37_0, g_contextSavePA);

	//Setup our wakeup events. RM can set something on it's own too,
	//let's not destroy those.
	setup_wakeup_events();

	//Save module context that should be restored after LP0
	//Interrupt, gpio, pin mux, clock management etc
	perform_context_operation(PowerModuleContext_Save);
}

void prepare_for_wb1(void)
{
	perform_context_operation(PowerModuleContext_SaveLP1);
	perform_context_operation(PowerModuleContext_DisableInterrupt);
}
