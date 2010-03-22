/*
 * arch/arm/mach-tegra/power-t2.c
 *
 * CPU shutdown routines for Tegra 2 SoCs
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
#include "linux/interrupt.h"

extern int enter_power_state(PowerState state, unsigned int proc_id);
extern void prepare_for_wb0(void);
extern void prepare_for_wb1(void);
extern NvU32* perform_context_operation(PowerModuleContext Context);

void cpu_ap20_do_lp2(void);
void resume(unsigned int state);
static NvU32 select_wakeup_pll(void);
void enable_pll(PowerPll pll, NvBool enable);
void enable_plls(NvBool enable);
void do_suspend_prep(void);
void reset_cpu(unsigned int cpu, unsigned int reset);
static void init_lp0_scratch_registers(void);
void shadow_runstate_scratch_regs(void);
void shadow_lp0_scratch_regs(void);


extern NvRmDeviceHandle s_hRmGlobal;

uintptr_t g_resume = 0, g_contextSavePA = 0, g_contextSaveVA = 0;
uintptr_t g_iramContextSaveVA = 0;
NvU32 g_modifiedPlls;
NvU32 g_wakeupCcbp = 0, g_NumActiveCPUs, g_Sync = 0, g_ArmPerif = 0;
NvU32 g_enterLP2PA = 0;
NvU32 g_localTimerLoadRegister, g_localTimerCntrlRegister;
NvU32 g_coreSightClock, g_currentCcbp;
NvU32 g_lp1CpuPwrGoodCnt, g_currentCpuPwrGoodCnt;
volatile void *g_pPMC, *g_pAHB, *g_pCLK_RST_CONTROLLER;
volatile void *g_pEMC, *g_pMC, *g_pAPB_MISC, *g_pTimerus;
volatile void *g_pIRAM;

#define WAKEUP_SOURCE_INT_RTC   16
#define INVALID_IRQ				(0xFFFF)
#define AP20_BASE_PA_BOOT_INFO	0x40000000
#define MAX_IRQ_CONTROLLERS	4
#define MAX_IRQ	(32*(MAX_IRQ_CONTROLLERS+1))

void cpu_ap20_do_lp0(void)
{
	//NOTE: Once we enter this routine, there is no way to avert a LP0.
	//If there is a potential interrupt waiting, we will enter LP0
	//and exit immediately as soon the PMC samples it and the
	//power good timer expires.
	NvU32   Reg;
	NvOdmPmuProperty	PmuProperty;
	NvBool HasPmuProperty = NvOdmQueryGetPmuProperty(&PmuProperty);

	//Inform RM about entry to LP0 state
	NvRmPrivPowerSetState(s_hRmGlobal, NvRmPowerState_LP0);

	if (HasPmuProperty && PmuProperty.CombinedPowerReq)
	{
		//Enable core power request, and tristate CPU power request outputs
		//(both requests are active, so there are no glitches as long as
		//proper external pu/pd are set)
		Reg = NV_REGR(s_hRmGlobal, NvRmModuleID_Pmif, 0,
				APBDEV_PMC_CNTRL_0);
		Reg = NV_FLD_SET_DRF_DEF(APBDEV_PMC, CNTRL, PWRREQ_OE, ENABLE, Reg);
		NV_REGW(s_hRmGlobal, NvRmModuleID_Pmif, 0,
				APBDEV_PMC_CNTRL_0, Reg);
		// FIXME: do we need an explicit delay?
		Reg = NV_REGR(s_hRmGlobal, NvRmModuleID_Pmif, 0,
			APBDEV_PMC_CNTRL_0);
		Reg = NV_FLD_SET_DRF_DEF(APBDEV_PMC, CNTRL,
			CPUPWRREQ_OE, DISABLE, Reg);
	}

	//Enter low power LP0 mode
	prepare_for_wb0();
	shadow_lp0_scratch_regs();
	printk("LP0: Entering...\n");
	enter_power_state(POWER_STATE_LP0, 0);
	printk("LP0: Exited...\n");
	shadow_runstate_scratch_regs();

	if (HasPmuProperty && PmuProperty.CombinedPowerReq)
	{
		//Enable CPU power request and tristate core power request outputs
		//(both requests are active, so there are no glitches as long as
		//proper external pu/pd are set)
		Reg = NV_REGR(s_hRmGlobal, NvRmModuleID_Pmif, 0,
				APBDEV_PMC_CNTRL_0);
		Reg = NV_FLD_SET_DRF_DEF(APBDEV_PMC, CNTRL, CPUPWRREQ_OE, ENABLE, Reg);
		NV_REGW(s_hRmGlobal, NvRmModuleID_Pmif, 0,
				APBDEV_PMC_CNTRL_0, Reg);
		//FIXME: do we need an explicit delay ?
		Reg = NV_REGR(s_hRmGlobal, NvRmModuleID_Pmif, 0,
				APBDEV_PMC_CNTRL_0);
		Reg = NV_FLD_SET_DRF_DEF(APBDEV_PMC, CNTRL, PWRREQ_OE, DISABLE, Reg);
		NV_REGW(s_hRmGlobal, NvRmModuleID_Pmif, 0,
				APBDEV_PMC_CNTRL_0, Reg);
	}

	//Restore the saved module(s) context
	//Interrupt, gpio, pin mux, clock management etc
	perform_context_operation(PowerModuleContext_Restore);
}

void cpu_ap20_do_lp1(void)
{
	NvU32 irq, moduleId;
	unsigned int proc_id = smp_processor_id();

	prepare_for_wb1();

	//Inform RM about entry to LP1 state
	NvRmPrivPowerSetState(s_hRmGlobal, NvRmPowerState_LP1);

	NvOsMemcpy((void*)g_iramContextSaveVA, (void*)g_pIRAM,
		AVP_CONTEXT_SAVE_AREA_SIZE);
	moduleId = NVRM_MODULE_ID(NvRmModuleID_SysStatMonitor, 0);
	irq = NvRmGetIrqForLogicalInterrupt(s_hRmGlobal, moduleId, 0);

	//Save our context ptrs to scratch regs
	NV_REGW(s_hRmGlobal, NvRmModuleID_Pmif, 0,
			APBDEV_PMC_SCRATCH1_0, g_resume);
	NV_REGW(s_hRmGlobal, NvRmModuleID_Pmif, 0,
			APBDEV_PMC_SCRATCH37_0, g_contextSavePA);

	//Only CPU0 must execute the actual suspend operations
	//CPU1 must be in the reset state before we continue LP1
	if (!proc_id)
	{
		//Disable the Statistics interrupt
		disable_irq(INT_SYS_STATS_MON);
		do_suspend_prep();

		// Set/save CPU power good count
		g_currentCpuPwrGoodCnt = NV_REGR(s_hRmGlobal,
			NvRmModuleID_Pmif, 0, APBDEV_PMC_CPUPWRGOOD_TIMER_0);
		NV_REGW(s_hRmGlobal, NvRmModuleID_Pmif, 0,
			APBDEV_PMC_CPUPWRGOOD_TIMER_0, g_lp1CpuPwrGoodCnt);
	}

	printk("entering lp1\n");
	//Do LP1
	enter_power_state(POWER_STATE_LP1, proc_id);
	printk("exiting lp1\n");

	if (!proc_id)
	{
		//We're back
		enable_irq(INT_SYS_STATS_MON);

		g_NumActiveCPUs = num_online_cpus();
                // Assembly LP1 code explicitly turn on PLLX,PLLM and PLLP so no need to enable it
		if (g_modifiedPlls & PowerPllC) {
			enable_pll(PowerPllC, NV_TRUE);
			NvOsWaitUS(300);
		}
		//Restore burst policy
		NV_REGW(s_hRmGlobal, NvRmPrivModuleID_ClockAndReset, 0,
				CLK_RST_CONTROLLER_CCLK_BURST_POLICY_0, g_currentCcbp);

		//Restore the CoreSight clock source.
		NV_REGW(s_hRmGlobal, NvRmPrivModuleID_ClockAndReset, 0,
				CLK_RST_CONTROLLER_CLK_SOURCE_CSITE_0, g_coreSightClock);

		// Restore CPU power good count
		NV_REGW(s_hRmGlobal, NvRmModuleID_Pmif, 0,
			APBDEV_PMC_CPUPWRGOOD_TIMER_0, g_currentCpuPwrGoodCnt);
	}

	NvOsMemcpy((void*)g_pIRAM, (void*)g_iramContextSaveVA,
		AVP_CONTEXT_SAVE_AREA_SIZE);

	perform_context_operation(PowerModuleContext_RestoreLP1);
}

void cpu_ap20_do_lp2(void)
{
    unsigned int proc_id = smp_processor_id();

    //Save our context ptrs to scratch regs
    NV_REGW(s_hRmGlobal, NvRmModuleID_Pmif, 0,
            APBDEV_PMC_SCRATCH33_0, g_resume);
    NV_REGW(s_hRmGlobal, NvRmModuleID_Pmif, 0,
            APBDEV_PMC_SCRATCH37_0, g_contextSavePA);

    //Only CPU0 must execute the actual suspend operations
    //CPU1 must be in the reset state before we continue LP2        
    if (!proc_id)
    {
        //Disable the Statistics interrupt
        disable_irq(INT_SYS_STATS_MON);
        do_suspend_prep();
    }

    //Do LP2
    enter_power_state(POWER_STATE_LP2, proc_id);
    
    if (!proc_id)
    {
        //We're back
        enable_irq(INT_SYS_STATS_MON);
    
        g_NumActiveCPUs = num_online_cpus();
        //Delay if needed
    
        if (g_modifiedPlls & PowerPllC)    
            enable_pll(PowerPllC, NV_TRUE);
        if (g_modifiedPlls & PowerPllM)    
            enable_pll(PowerPllM, NV_TRUE);
        if (g_modifiedPlls & PowerPllP)    
            enable_pll(PowerPllP, NV_TRUE);

        NvOsWaitUS(300);
        //Restore burst policy
        NV_REGW(s_hRmGlobal, NvRmPrivModuleID_ClockAndReset, 0, 
                CLK_RST_CONTROLLER_CCLK_BURST_POLICY_0, g_currentCcbp);
        
        //Restore the CoreSight clock source.
        NV_REGW(s_hRmGlobal, NvRmPrivModuleID_ClockAndReset, 0, 
                CLK_RST_CONTROLLER_CLK_SOURCE_CSITE_0, g_coreSightClock);
    }
}

void power_lp0_init(void)
{
	NvU32				Reg;
	NvOdmPmuProperty	PmuProperty;
	NvBool HasPmuProperty = NvOdmQueryGetPmuProperty(&PmuProperty);
	const NvOdmSocPowerStateInfo	*LPStateInfo;

	LPStateInfo = NvOdmQueryLowestSocPowerState();

	//Enable CPU power request. Leave it enabled to be ready for LP2/LP1.
	Reg = NV_PMC_REGR(g_pPMC, CNTRL);
	Reg = NV_FLD_SET_DRF_DEF(APBDEV_PMC, CNTRL, CPUPWRREQ_OE, ENABLE, Reg);
	NV_PMC_REGW(g_pPMC, CNTRL, Reg);

	//If the system supports deep sleep (LP0), initialize PMC accordingly.
	if (LPStateInfo->LowestPowerState == NvOdmSocPowerState_DeepSleep)
	{
		//Initialize the scratch registers  with the current system info.
		init_lp0_scratch_registers();

		//Get the core_power_request and sys_clock_request signal polarities.
		if (HasPmuProperty)
		{
			Reg = NV_PMC_REGR(g_pPMC, CNTRL);

			//Configure CORE power request signal polarity (the output is
			//still tristated)
			if (PmuProperty.CorePowerReqPolarity == NvOdmCorePowerReqPolarity_Low)
			{
				Reg = NV_FLD_SET_DRF_DEF(APBDEV_PMC, CNTRL,
					PWRREQ_POLARITY, INVERT, Reg);
			}

			//Enable clock request signal if supported and it's polarity.
			Reg = NV_FLD_SET_DRF_DEF(APBDEV_PMC, CNTRL,
				SYSCLK_OE, ENABLE, Reg);
			if (PmuProperty.SysClockReqPolarity == NvOdmSysClockReqPolarity_Low)
			{
				Reg = NV_FLD_SET_DRF_DEF(APBDEV_PMC, CNTRL,
					SYSCLK_POLARITY, INVERT, Reg);
			}

			NV_PMC_REGW(g_pPMC,CNTRL,Reg);

			//Enable CORE power request output if it is connected separately
			//to PMU; keep it tristated if it is combined with CPU request -
			//it will be dynamically enabled/disabled on entry/exit to LP0
			if (!PmuProperty.CombinedPowerReq)
			{
				Reg = NV_PMC_REGR(g_pPMC, CNTRL); // make sure polarity is set
				Reg = NV_FLD_SET_DRF_DEF(APBDEV_PMC, CNTRL,
					PWRREQ_OE, ENABLE, Reg);
				NV_PMC_REGW(g_pPMC,CNTRL,Reg);
			}
		}

		//Program the core power good timer
		NV_PMC_REGW(g_pPMC,PWRGOOD_TIMER,PmuProperty.PowerGoodCount);
	}

	// Get ready CPU power good count in 32.768 kHz clocks (don't set timer
	// until actual LP1 entry; at run time it is on clock, scaled by DVFS)
	// Calculate count as 32.768 * TimeUS / 1000 = 4096 * TimeUS / 125000
	// and round it up.
	g_lp1CpuPwrGoodCnt = HasPmuProperty ?
		 PmuProperty.CpuPowerGoodUs : 2000;	// Use 2ms by default
	g_lp1CpuPwrGoodCnt =
		(4096 * PmuProperty.CpuPowerGoodUs + 124999) / 125000;

}

//Generate definitions of local variables to hold scratch register values.
#define SCRATCH_REG(s) static NvU32 s = 0;
SCRATCH_REGS()
#undef SCRATCH_REG

//Generate definitions of local variables to hold shadow
//scratch register values.
#define SHADOW_REG(s) static NvU32 SHADOW_##s = 0;
SHADOW_REGS()
#undef SHADOW_REG

static void init_lp0_scratch_registers(void)
{
	NvU32					Reg;		//Module register contents
	NvU32					Val;		//Register field contents

	//NOTE: The SDRAM registers have already been saved
	//by the bootloader

	//Generate reads to the shadowed PMC scratch registers to copy the current
	//values while we populate the register with their LP0 values.
	#define SHADOW_REG(s) SHADOW_##s = NV_PMC_REGR(g_pPMC, s);
	SHADOW_REGS()
	#undef SHADOW_REG

	//Define the transformation macro that will read and pull apart the
	// module register values and pack them into PMC scratch regsiters.

	//REG(s,d,r,f)
	//s = destination Scratch register
	//d = Device name
	//r = Register name
	//f = register Field
	#define REG(s,d,r,f)  \
		Reg = NV_DR_REGR(d,r); \
		Val = NV_DRF_VAL(d,r,f,Reg); \
		s = NV_FLD_SET_SDRF_NUM(s,d,r,f,Val);

	//Instantiate all of the register transformations.
	REGS()
	#undef REG
	#undef RAM
	#undef CONSTANT

	//Generate writes to the PMC scratch registers to copy the local
	//variables to the actual registers.
	#define SCRATCH_REG(s) NV_PMC_REGW(g_pPMC, s, s);
	SCRATCH_REGS()
	#undef SCRATCH_REG

	//Generate writes to the shadowed PMC scratch registers to restore the
	//"normal" values expected by RM.
	#define SHADOW_REG(s) NV_PMC_REGW(g_pPMC, s, SHADOW_##s);
	SHADOW_REGS()
	#undef SHADOW_REG

	return;
}

void shadow_runstate_scratch_regs(void)
{
	//Generate writes to the shadowed PMC scratch registers to restore the
	//"normal" running mode values expected by RM.
	#define SHADOW_REG(s) NV_PMC_REGW(g_pPMC, s, SHADOW_##s);
	SHADOW_REGS()
	#undef SHADOW_REG
}

void shadow_lp0_scratch_regs(void)
{
	//Generate reads of the shadowed PMC scratch registers to copy the current
	//values while we populate the register with their LP0 values.
	#define SHADOW_REG(s) SHADOW_##s = NV_PMC_REGR(g_pPMC, s);
	SHADOW_REGS()
	#undef SHADOW_REG

	// Generate writes to the shadowed PMC scratch registers
	//for the LP0 values.
	#define SHADOW_REG(s) NV_PMC_REGW(g_pPMC, s, s);
	SHADOW_REGS()
	#undef SHADOW_REG
}

static NvU32 select_wakeup_pll(void)
{
    NvU32   Reg = 0;            // Scratch register
    NvU32   CpuState;       // CPU run state
    NvU32   Ccbp;           // CPU clock burst policy
    NvU32   CurrentCcbp;    // CPU clock burst policy
    NvU32   PllMask;        // Mask of possible PLLs
    NvU32   Pll;            // Wakeup PLL

    //Get the current CPU burst policy.
    CurrentCcbp = NV_REGR(s_hRmGlobal, NvRmPrivModuleID_ClockAndReset, 0, 
                    CLK_RST_CONTROLLER_CCLK_BURST_POLICY_0);
    Ccbp = CurrentCcbp;

    //Assume wakeup will be same as current.
    g_wakeupCcbp = Ccbp;

    //Extract the current burst policy system state.
    CpuState = NV_DRF_VAL(CLK_RST_CONTROLLER, CCLK_BURST_POLICY,
                    CPU_STATE, Ccbp);
    switch (CpuState)
    {
        case CLK_RST_CONTROLLER_CCLK_BURST_POLICY_0_CPU_STATE_STDBY:
            // Running on the 32KHz oscillator.
            return CurrentCcbp;

        case CLK_RST_CONTROLLER_CCLK_BURST_POLICY_0_CPU_STATE_IDLE:
            // Extract the 'idle' state clock source.
            Reg = NV_DRF_VAL(CLK_RST_CONTROLLER, CCLK_BURST_POLICY, 
                    CWAKEUP_IDLE_SOURCE, Ccbp);
            break;

        case CLK_RST_CONTROLLER_CCLK_BURST_POLICY_0_CPU_STATE_RUN:
            // Extract the 'run' state clock source.
            Reg = NV_DRF_VAL(CLK_RST_CONTROLLER, CCLK_BURST_POLICY, 
                    CWAKEUP_RUN_SOURCE, Ccbp);
            break;

        case CLK_RST_CONTROLLER_CCLK_BURST_POLICY_0_CPU_STATE_IRQ:
            // Extract the 'IRQ' state clock source.
            Reg = NV_DRF_VAL(CLK_RST_CONTROLLER, CCLK_BURST_POLICY, 
                    CWAKEUP_IRQ_SOURCE, Ccbp);
            break;

        case CLK_RST_CONTROLLER_CCLK_BURST_POLICY_0_CPU_STATE_FIQ:
            // Extract the 'FIQ' state clock source.
            Reg = NV_DRF_VAL(CLK_RST_CONTROLLER, CCLK_BURST_POLICY, 
                    CWAKEUP_FIQ_SOURCE, Ccbp);
            break;
        default:
            break;
    }

    // Are we running on PLL-X right now?
    if (Reg != CLK_RST_CONTROLLER_CCLK_BURST_POLICY_0_CWAKEUP_IRQ_SOURCE_PLLX_OUT0)
    {
        // No. Whatever PLL we're running on now will be there when we wake up.
        return CurrentCcbp;
    }

    // We're running on PLL-X now. When the CPU complex is powered down, PLL-X
    // will be reset. Add PLL-X to the list of PLLs that need to be restarted.
    g_modifiedPlls |= PowerPllX;

    // Get the possible PLL clock sources for the CPU complex. Don't consider
    // any PLLs that were turned off and of course PLL-X is not a possibility.
    PllMask = (PowerPllC | PowerPllP | PowerPllM) & ~g_modifiedPlls;

    // Pick a PLL source for the CPU complex so that we don't have to
    // run on the oscillator which is soooo slooooow.
    Pll = CLK_RST_CONTROLLER_CCLK_BURST_POLICY_0_CWAKEUP_RUN_SOURCE_CLKM;

    // Will there be a running PLL for the CPU when we wake up?
    if (PllMask)
    {
        // First choice is to run on PLL-M? Will it be available?
        if (PllMask & PowerPllM)
        {
            // Is it running now?
            Reg = NV_REGR(s_hRmGlobal, NvRmPrivModuleID_ClockAndReset, 0, 
                    CLK_RST_CONTROLLER_PLLM_BASE_0);
            if (NV_DRF_VAL(CLK_RST_CONTROLLER, PLLM_BASE, PLLM_ENABLE, Reg))
            {
                // Yes. Switch to PLL-M now so we're be on it when we get back.
                Pll = CLK_RST_CONTROLLER_CCLK_BURST_POLICY_0_CWAKEUP_RUN_SOURCE_PLLM_OUT0;
                goto SetWakeupSource;
            }
        }

        // Next choice is to run on PLL-P? Will it be available?
        if (PllMask & PowerPllP)
        {
            // Is it running now?
            Reg = NV_REGR(s_hRmGlobal, NvRmPrivModuleID_ClockAndReset, 
                    0, CLK_RST_CONTROLLER_PLLP_BASE_0);
            if (NV_DRF_VAL(CLK_RST_CONTROLLER, PLLP_BASE, PLLP_ENABLE, Reg))
            {
                Pll = CLK_RST_CONTROLLER_CCLK_BURST_POLICY_0_CWAKEUP_RUN_SOURCE_PLLP_OUT0;
                goto SetWakeupSource;
            }
        }

        // Next choice is to run on PLL-C? Will it be available?
        if (PllMask & PowerPllC)
        {
            // Is it running now?
            Reg = NV_REGR(s_hRmGlobal, NvRmPrivModuleID_ClockAndReset, 0,
                     CLK_RST_CONTROLLER_PLLC_BASE_0);
            if (NV_DRF_VAL(CLK_RST_CONTROLLER, PLLC_BASE, PLLC_ENABLE, Reg))
            {
                Pll = CLK_RST_CONTROLLER_CCLK_BURST_POLICY_0_CWAKEUP_RUN_SOURCE_PLLC_OUT0;
                goto SetWakeupSource;
            }
        }
    }

SetWakeupSource:

    switch (CpuState)
    {
        case CLK_RST_CONTROLLER_CCLK_BURST_POLICY_0_CPU_STATE_IDLE:
            // Extract the 'idle' state clock source.
            Ccbp = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, CCLK_BURST_POLICY, 
                    CWAKEUP_IDLE_SOURCE, Pll, Ccbp);
            break;

        case CLK_RST_CONTROLLER_CCLK_BURST_POLICY_0_CPU_STATE_RUN:
            // Extract the 'run' state clock source.
            Ccbp = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, CCLK_BURST_POLICY, 
                    CWAKEUP_RUN_SOURCE, Pll, Ccbp);
            break;

        case CLK_RST_CONTROLLER_CCLK_BURST_POLICY_0_CPU_STATE_IRQ:
            // Extract the 'IRQ' state clock source.
            Ccbp = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, CCLK_BURST_POLICY, 
                    CWAKEUP_IRQ_SOURCE, Pll, Ccbp);
            break;

        case CLK_RST_CONTROLLER_CCLK_BURST_POLICY_0_CPU_STATE_FIQ:
            // Extract the 'FIQ' state clock source.
            Ccbp = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, CCLK_BURST_POLICY, 
                    CWAKEUP_FIQ_SOURCE, Pll, Ccbp);
            break;

        default:
            break;
    }

    // Return the wakeup source.
    g_wakeupCcbp = Ccbp;

    return CurrentCcbp;
}

void enable_pll(PowerPll pll, NvBool enable)
{
    NvU32 reg;
    
    switch(pll)
    {
        case PowerPllM:
            reg = NV_REGR(s_hRmGlobal, NvRmPrivModuleID_ClockAndReset, 
                    0, CLK_RST_CONTROLLER_PLLM_BASE_0);
            reg = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLM_BASE, 
                    PLLM_ENABLE, (enable ? 1 : 0), reg);
            NV_REGW(s_hRmGlobal, NvRmPrivModuleID_ClockAndReset, 
                    0, CLK_RST_CONTROLLER_PLLM_BASE_0, reg);
            break;

        case PowerPllC:
            reg = NV_REGR(s_hRmGlobal, NvRmPrivModuleID_ClockAndReset, 
                    0, CLK_RST_CONTROLLER_PLLC_BASE_0);
            reg = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLC_BASE, 
                    PLLC_ENABLE, (enable ? 1 : 0), reg);
            NV_REGW(s_hRmGlobal, NvRmPrivModuleID_ClockAndReset, 
                    0, CLK_RST_CONTROLLER_PLLC_BASE_0, reg);
            break;

        case PowerPllP:
            reg = NV_REGR(s_hRmGlobal, NvRmPrivModuleID_ClockAndReset, 
                    0, CLK_RST_CONTROLLER_PLLP_BASE_0);
            reg = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLP_BASE, 
                    PLLP_ENABLE, (enable ? 1 : 0), reg);
            NV_REGW(s_hRmGlobal, NvRmPrivModuleID_ClockAndReset, 0, 
                    CLK_RST_CONTROLLER_PLLP_BASE_0, reg);
            break;

        case PowerPllA:
            reg = NV_REGR(s_hRmGlobal, NvRmPrivModuleID_ClockAndReset, 
                    0, CLK_RST_CONTROLLER_PLLA_BASE_0);
            reg = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLA_BASE, 
                    PLLA_ENABLE, (enable ? 1 : 0), reg);
            NV_REGW(s_hRmGlobal, NvRmPrivModuleID_ClockAndReset, 
                    0, CLK_RST_CONTROLLER_PLLA_BASE_0, reg);
            break;

        default:
            break;
    }
}

void enable_plls(NvBool enable)
{
    NvU32 dfsPllFlags = NvRmPrivGetDfsFlags(s_hRmGlobal);
    
    g_modifiedPlls = 0;
    
    //DFS tells us which PLLs are safe to turn off
    if (dfsPllFlags & NvRmDfsStatusFlags_StopPllC0)
    {
        enable_pll(PowerPllC, NV_FALSE);
        g_modifiedPlls |= PowerPllC;
    }

    //pllp
    if ((dfsPllFlags & NvRmDfsStatusFlags_StopPllP0) && 
        (dfsPllFlags & NvRmDfsStatusFlags_StopPllA0))
    {
        enable_pll(PowerPllP, NV_FALSE);
        g_modifiedPlls |= PowerPllP;
    }

    //pllm
    if (dfsPllFlags & NvRmDfsStatusFlags_StopPllM0)
    {
        enable_pll(PowerPllM, NV_FALSE);
        g_modifiedPlls |= PowerPllM;
    }
}

void do_suspend_prep(void)
{
    NvU32 reg;
    
    //Enable CPU power request output
    reg = NV_REGR(s_hRmGlobal, NvRmModuleID_Pmif, 0, APBDEV_PMC_CNTRL_0);
    reg = NV_FLD_SET_DRF_DEF(APBDEV_PMC, CNTRL, CPUPWRREQ_OE, ENABLE, reg);
    NV_REGW(s_hRmGlobal, NvRmModuleID_Pmif, 0, APBDEV_PMC_CNTRL_0, reg);
    
    //Save the CoreSight stuff
    g_coreSightClock = NV_REGR(s_hRmGlobal, NvRmPrivModuleID_ClockAndReset, 
            0, CLK_RST_CONTROLLER_CLK_SOURCE_CSITE_0);
    reg = NV_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_CSITE, 
            CSITE_CLK_SRC, CLK_M);
    NV_REGW(s_hRmGlobal, NvRmPrivModuleID_ClockAndReset, 0, 
            CLK_RST_CONTROLLER_CLK_SOURCE_CSITE_0, reg);
    //Turn off necessary PLLs
    enable_plls(NV_FALSE);
    //Select the wakeup pll
    g_currentCcbp = select_wakeup_pll();

}

void reset_cpu(unsigned int cpu, unsigned int reset)
{
    NvU32 reg;
    
    if (reset)
    {
        reg = NV_DRF_DEF(FLOW_CTLR, HALT_CPU1_EVENTS, MODE,
                FLOW_MODE_WAITEVENT);
                    
        if (cpu)
        {                    
            NV_REGW(s_hRmGlobal, NvRmModuleID_FlowCtrl, 0, 
                    FLOW_CTLR_HALT_CPU1_EVENTS_0, reg);
        }
        else
        {
            NV_REGW(s_hRmGlobal, NvRmModuleID_FlowCtrl, 0, 
                    FLOW_CTLR_HALT_CPU_EVENTS_0, reg);            
        }
        // Place CPUn into reset.
        reg = NV_DRF_NUM(CLK_RST_CONTROLLER, RST_CPU_CMPLX_SET, SET_CPURESET0, 1)
            | NV_DRF_NUM(CLK_RST_CONTROLLER, RST_CPU_CMPLX_SET, SET_DBGRESET0, 1)
            | NV_DRF_NUM(CLK_RST_CONTROLLER, RST_CPU_CMPLX_SET, SET_DERESET0,  1);
        reg <<= cpu;
        NV_REGW(s_hRmGlobal, NvRmPrivModuleID_ClockAndReset, 0, 
            CLK_RST_CONTROLLER_RST_CPU_CMPLX_SET_0, reg);    
    }
    else
    {
        reg = NV_DRF_DEF(FLOW_CTLR, HALT_CPU1_EVENTS, MODE, 
                    FLOW_MODE_NONE);           
        if (cpu)
        {                    
            NV_REGW(s_hRmGlobal, NvRmModuleID_FlowCtrl, 0, 
                    FLOW_CTLR_HALT_CPU1_EVENTS_0, reg);
        }
        else
        {
            NV_REGW(s_hRmGlobal, NvRmModuleID_FlowCtrl, 0, 
                    FLOW_CTLR_HALT_CPU_EVENTS_0, reg);            
        }        
        // Take CPUn out of reset.
        reg = NV_DRF_NUM(CLK_RST_CONTROLLER, RST_CPU_CMPLX_CLR, CLR_CPURESET0, 1)
            | NV_DRF_NUM(CLK_RST_CONTROLLER, RST_CPU_CMPLX_CLR, CLR_DBGRESET0, 1)
            | NV_DRF_NUM(CLK_RST_CONTROLLER, RST_CPU_CMPLX_CLR, CLR_DERESET0,  1);
        reg <<= cpu;
        NV_REGW(s_hRmGlobal, NvRmPrivModuleID_ClockAndReset, 0, 
            CLK_RST_CONTROLLER_RST_CPU_CMPLX_CLR_0, reg);
    }
}

unsigned int check_for_cpu1_reset(void)
{
    volatile NvU32 reg;

    reg = NV_REGR(s_hRmGlobal, NvRmPrivModuleID_ClockAndReset, 0, 
            CLK_RST_CONTROLLER_RST_CPU_CMPLX_SET_0);
            
    reg = reg & 0x2;                
    
    return reg;
}

void save_local_timers(void)
{
    volatile NvU32 *reg = (volatile NvU32 *)(g_ArmPerif+0x600);
    volatile NvU32 spin = 0;

    while (spin);

    g_localTimerLoadRegister = reg[0];
    g_localTimerCntrlRegister = reg[2];    
}

void restore_local_timers(void)
{
    volatile NvU32 *reg = (volatile NvU32 *)(g_ArmPerif+0x600);
    volatile NvU32 spin = 0;

    while (spin);
    
    reg[0] = g_localTimerLoadRegister;
    reg[2] = g_localTimerCntrlRegister;
}

