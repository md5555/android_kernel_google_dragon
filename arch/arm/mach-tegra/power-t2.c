/*
 * arch/arm/mach-tegra/power-t2.c
 *
 * CPU shutdown routines for Tegra 2 SoCs
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

#include "nvos.h"
#include "nvrm_init.h"
#include "nvrm_drf.h"
#include "ap20/arapbpm.h"
#include "nvrm_module.h"
#include "ap20/arflow_ctlr.h"
#include "ap20/arclk_rst.h"
#include "nvrm_hardware_access.h"
#include "nvrm_interrupt.h"
#include "nvrm_power.h"
#include "nvrm_power_private.h"
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>

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

extern void NvPrivAp20MaskIrq(unsigned int irq);
extern void NvPrivAp20UnmaskIrq(unsigned int irq);
extern int enter_power_state(PowerState state, unsigned int proc_id);
void cpu_ap20_do_lp2(void);
void resume(unsigned int state);
static NvU32 select_wakeup_pll(void);
void enable_pll(PowerPll pll, NvBool enable);
void enable_plls(NvBool enable);
void do_suspend_prep(void);
void reset_cpu(unsigned int cpu, unsigned int reset);
extern NvRmDeviceHandle s_hRmGlobal;

#define NUM_LOCAL_TIMER_REGISTERS 3

uintptr_t g_resume = 0, g_contextSavePA = 0, g_contextSaveVA = 0;
NvU32 g_modifiedPlls;
NvU32 g_wakeupCcbp = 0, g_NumActiveCPUs, g_Sync = 0, g_ArmPerif = 0;
NvU32 g_enterLP2PA = 0;
NvU32 g_localTimerLoadRegister, g_localTimerCntrlRegister;
NvU32 g_coreSightClock, g_currentCcbp;

void cpu_ap20_do_lp2(void)
{
    NvU32 irq, moduleId, reg;
    unsigned int proc_id = smp_processor_id();
    
    moduleId = NVRM_MODULE_ID(NvRmModuleID_SysStatMonitor, 0);
    irq = NvRmGetIrqForLogicalInterrupt(s_hRmGlobal, moduleId, 0);

    //Save our context ptrs to scratch regs
    NV_REGW(s_hRmGlobal, NvRmModuleID_Pmif, 0, 
            APBDEV_PMC_SCRATCH1_0, g_resume);
    NV_REGW(s_hRmGlobal, NvRmModuleID_Pmif, 0, 
            APBDEV_PMC_SCRATCH37_0, g_contextSavePA);

    //Only CPU0 must execute the actual suspend operations
    //CPU1 must be in the reset state before we continue LP2        
    if (!proc_id)
    {
        //Disable the Statistics interrupt
        NvPrivAp20MaskIrq(irq);
        
        do_suspend_prep();
    }    

    //Do LP2
    enter_power_state(POWER_STATE_LP2, proc_id);
    
    if (!proc_id)
    {
        //We're back
        NvPrivAp20UnmaskIrq(irq);
    
        g_NumActiveCPUs = num_online_cpus();
        //We're back
        //Delay if needed
    
        if (g_modifiedPlls & PowerPllC)    
            enable_pll(PowerPllC, NV_TRUE);
        if (g_modifiedPlls & PowerPllM)    
            enable_pll(PowerPllM, NV_TRUE);
        if (g_modifiedPlls & PowerPllP)    
            enable_pll(PowerPllP, NV_TRUE);
        if (g_modifiedPlls & PowerPllX)    
            enable_pll(PowerPllX, NV_TRUE);        
        
        NvOsWaitUS(300);
            
        //Restore burst policy
        NV_REGW(s_hRmGlobal, NvRmPrivModuleID_ClockAndReset, 0, 
                CLK_RST_CONTROLLER_CCLK_BURST_POLICY_0, g_currentCcbp);
        
        //Restore the CoreSight clock source.
        NV_REGW(s_hRmGlobal, NvRmPrivModuleID_ClockAndReset, 0, 
                CLK_RST_CONTROLLER_CLK_SOURCE_CSITE_0, g_coreSightClock);

    }
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

    //pllx - DFS currently doesn't signal this, but it doesn't matter
    //We're going to turn it off anyway
    if (dfsPllFlags & NvRmDfsStatusFlags_StopPllX0)
    {        
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

