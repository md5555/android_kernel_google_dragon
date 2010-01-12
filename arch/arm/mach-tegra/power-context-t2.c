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
NvU32* perform_context_operation(PowerModuleContext Context);
void prepare_for_wb0(void);

struct power_context *s_pModulesContextAnchor = NULL;

static void update_registers_for_lp0(void)
{
	NvU32 Reg;

	//SCRATCH0 : Warmbootflag
	Reg = NV_REGR(s_hRmGlobal, NvRmModuleID_Pmif, 0,
				APBDEV_PMC_SCRATCH0_0);
	Reg = NV_FLD_SET_DRF_NUM (APBDEV_PMC, SCRATCH0, WARM_BOOT0_FLAG, 0, Reg);
	NV_REGW(s_hRmGlobal, NvRmModuleID_Pmif, 0,
				APBDEV_PMC_SCRATCH0_0, Reg);

	//SCRATCH1 : AVP-side recovery physical address.
	//FIXME: Need to get the AVP restore address from the bootloader

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
		if (pBase == NULL);
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

		//Save module clock enable state
		*pCM++ = NV_CAR_REGR(pBase, CLK_OUT_ENB_L);
		*pCM++ = NV_CAR_REGR(pBase, CLK_OUT_ENB_H);

		//Save clock mask register
		*pCM++ = NV_CAR_REGR(pBase, CLK_MASK_ARM);
		break;
	case PowerModuleContext_Restore:
		//Register base address must have been set
		//by PowerModuleContext_Init.
		if (pBase == NULL);
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

		//Restore module clock enable state
		NV_CAR_REGW(pBase, CLK_OUT_ENB_L, *pCM++);
		NV_CAR_REGW(pBase, CLK_OUT_ENB_H, *pCM++);

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

NvU32* perform_context_operation(PowerModuleContext Context)
{
	NvU32*  pCM;	// Pointer to next available save area location

	//First save area location available for module use is located just
	//beyond the anchor structure.
	//NV_ASSERT(s_pModulesContextAnchor != NULL);

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
	//TODO: Save rest of modules here

	//Check if we went past the context area limits.
	if (pCM >= ((NvU32*)s_pModulesContextAnchor +
			s_pModulesContextAnchor->context_size_words))
		return NULL;

	return pCM;
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
