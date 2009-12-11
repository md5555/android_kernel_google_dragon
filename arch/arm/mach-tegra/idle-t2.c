/*
 * arch/arm/mach-tegra/idle-t2.c
 *
 * CPU hotplug support for Tegra SoCs
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
#include "nvrm_hardware_access.h"
#include "nvrm_power_private.h"
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>

extern NvRmDeviceHandle s_hRmGlobal;
extern void cpu_ap20_do_lp2(void);
extern void resume(unsigned int state);
extern uintptr_t g_resume, g_contextSavePA, g_contextSaveVA;
extern NvU32 g_NumActiveCPUs, g_ArmPerif;
extern NvU32 g_enterLP2PA;

#ifdef CONFIG_WAKELOCK
extern struct wake_lock main_wake_lock;
#endif

// Debug-only: set to zero to disable use of flow-controller based idling
#define CPU_CONTEXT_SAVE_AREA_SIZE 4096
#define TEMP_SAVE_AREA_SIZE 16
#define ENABLE_LP2 1
#define NV_POWER_LP2_IDLE_THRESHOLD_MS 700
#define NV_POWER_IDLE_WINDOW_SIZE 100
#define MAX_LP2_TIME_US 1000000
#define NV_POWER_COMPLETE_IDLE_MS 1000

// When non-zero, collects and prints aggregate statistics about idle times
static volatile NvU8 *s_pFlowCtrl = NULL;

void cpu_ap20_do_idle(void);
void mach_tegra_reset(void);
void mach_tegra_idle(void);
extern void enter_lp2(NvU32, NvU32);
extern void exit_power_state(void);

NvU32 lp2count = 0, lp3count = 0, lp2safe = 0;

void __init NvAp20InitFlowController(void);

void __init NvAp20InitFlowController(void)
{
    NvU32 len;
    NvRmPhysAddr pa;
    volatile NvU8 *pTempFc, *pTempArmPerif;
    
    NvRmModuleGetBaseAddress(s_hRmGlobal,
        NVRM_MODULE_ID(NvRmModuleID_FlowCtrl, 0), &pa, &len);
            
    if (NvRmPhysicalMemMap(pa, len, NVOS_MEM_READ_WRITE,
            NvOsMemAttribute_Uncached, (void**)&pTempFc)!=NvSuccess)
    {
        printk(KERN_INFO "failed to map flow controller; DVFS will not function"
               " correctly as a result\n");
        return;
    }

    NvRmModuleGetBaseAddress(s_hRmGlobal,
        NVRM_MODULE_ID(NvRmPrivModuleID_ArmPerif, 0), &pa, &len);

    if (NvRmPhysicalMemMap(pa, len, NVOS_MEM_READ_WRITE,
            NvOsMemAttribute_Uncached, (void**)&pTempArmPerif)!=NvSuccess)
    {
        printk(KERN_INFO "failed to map Arm Perif; DVFS will not function"
               " correctly as a result\n");
        return;
    }

    s_pFlowCtrl = pTempFc;
    g_ArmPerif = (NvU32)pTempArmPerif;

    /* Get physical and virtual addresses for a bunch of things
     * 1) The resume address physical - we have to come back here with MMU off
     * 2) The context save physical address - We need to save a bunch of
     * stuff here
     */
    g_resume = virt_to_phys((void*)exit_power_state);
    g_contextSaveVA =
        (uintptr_t)kmalloc(CPU_CONTEXT_SAVE_AREA_SIZE, GFP_ATOMIC);
    g_contextSavePA = virt_to_phys((void*)g_contextSaveVA);
    g_NumActiveCPUs = num_online_cpus();
    g_enterLP2PA = virt_to_phys((void*)enter_lp2);    
}

/*
 *  cpu_ap20_do_idle()
 *
 *  Idle the processor (eg, wait for interrupt).
 *
 *  IRQs are already disabled.
 */
void cpu_ap20_do_idle(void)
{
    unsigned int tmp = 0;
    volatile uint32_t *addr = 0;
    
    dsb();

    if (likely(s_pFlowCtrl))
    {
        /* 
         *  Trigger the "stats monitor" to count the CPU idle cycles.
         */
        if (smp_processor_id())
        {
            addr = (volatile uint32_t*)(s_pFlowCtrl + FLOW_CTLR_HALT_CPU1_EVENTS_0);
        } else
        {
            addr = (volatile uint32_t*)(s_pFlowCtrl + FLOW_CTLR_HALT_CPU_EVENTS_0);
        }

        tmp = NV_DRF_DEF(FLOW_CTLR, HALT_CPU1_EVENTS, MODE, FLOW_MODE_WAITEVENT) 
            | NV_DRF_NUM(FLOW_CTLR, HALT_CPU1_EVENTS, JTAG, 1);

        NV_WRITE32(addr, tmp);
        tmp = NV_READ32(addr);
    }

    // Wait for any interrupt
    __asm__ volatile ("wfi");

    if (likely(s_pFlowCtrl))
    {
        /*
         * Signal "stats monitor" to stop counting the idle cycles.
         */
        tmp = NV_DRF_DEF(FLOW_CTLR, HALT_CPU1_EVENTS, MODE, FLOW_MODE_NONE); 
        NV_WRITE32(addr, tmp);
        tmp = NV_READ32(addr);
    }
}

void mach_tegra_idle(void)
{ 
    static NvU32 flag = 0, lp2_time;
    static NvU64 cur_jiffies = 0, old_jiffies = 0;
    NvU64 delta_jif = 0;
    NvU32 msec, delta;
    NvRmDfsClockUsage clock_usage;

#ifdef CONFIG_WAKELOCK
    //The wake lock api is ready if the main lock is ready 
    if (main_wake_lock.flags)
    {
        //Check if there are any IDLE locks pending
        //If there are, then we do LP3. Else we do LP2
        if (!has_wake_lock(WAKE_LOCK_IDLE) && flag)
        {
            flag = 0;
#if ENABLE_LP2            
            //Only attempt LP2 if the flow controller is ready
            if (s_pFlowCtrl)            
            {
                if (num_online_cpus() > 1)
                    lp2safe = 1;

                if (num_online_cpus() == 1 && lp2safe)
                {
                    lp2count++;
                    NvSpareTimerTrigger(lp2_time);
                    cpu_ap20_do_lp2();
                    NvRmPrivSetLp2TimeUS(s_hRmGlobal, MAX_LP2_TIME_US);
                    return;
                }
            }
#endif
        }        
    }
#endif // CONFIG_WAKELOCK
    lp3count++;
    if (lp3count % NV_POWER_IDLE_WINDOW_SIZE == 0)
    {
        old_jiffies = cur_jiffies;
        cur_jiffies = get_jiffies_64();
		delta_jif = cur_jiffies - old_jiffies;
		msec = jiffies_to_msecs(delta_jif);
		
		//In a truly idle system, time spent in the idle loop
		//will be very high due to the wfi. In a non-idle system
		//we will enter and exit the idle loop quickly. As a result,
		//we can say that the longer an IDLE_WINDOW, the more
		//idle the system is.
		if (msec > NV_POWER_LP2_IDLE_THRESHOLD_MS)
		{    	
	        flag = 1;

	        //The IDLE_WINDOW addresses the first axis - Performance.
	        //The second axis - Power savings, still needs to be addressed.
	        //The greater our LP2 duty cycle, the greater our power savings.
	        //We can say that the longer the IDLE_WINDOW, the more idle the
	        //system and, consequently, the longer we can stay in LP2.
	        if (msec > NV_POWER_COMPLETE_IDLE_MS)
	        	msec = NV_POWER_COMPLETE_IDLE_MS;

			delta = NV_POWER_COMPLETE_IDLE_MS - msec;

			if (delta >= 300)
			{			
	        	lp2_time = MAX_LP2_TIME_US / 8;
			}
			else if (delta >= 200)
			{
				lp2_time = MAX_LP2_TIME_US / 5;
			}
			else if (delta >= 100)
			{
				lp2_time = MAX_LP2_TIME_US / 3;
			}
			else
			{
				lp2_time = MAX_LP2_TIME_US;
			}	        
		}
    }
    cpu_ap20_do_idle();    
}

void mach_tegra_reset(void)
{
    NvU32 reg;

    reg = NV_DRF_DEF(APBDEV_PMC, CNTRL, MAIN_RST, ENABLE);
    NV_REGW(s_hRmGlobal, NvRmModuleID_Pmif, 0, APBDEV_PMC_CNTRL_0, reg);
}
