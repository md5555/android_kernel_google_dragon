/*
 * arch/arm/mach-tegra/platsmp.c
 *
 * SMP management routines for SMP Tegra SoCs
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

#include <linux/init.h>
#include <linux/smp.h>
#include <asm/cacheflush.h>
#include <asm/localtimer.h>

#include "mach/nvrm_linux.h"
#include "nvrm_module.h"
#include "nvrm_init.h"
#include "nvrm_drf.h"
#include "nvrm_hardware_access.h"
#include "nvcommon.h"
#include "ap20/arscu.h"
#include "ap20/arevp.h"
#include "ap20/arclk_rst.h"
#include "ap20/arfic_proc_if.h"
#include "ap20/arflow_ctlr.h"

static DEFINE_SPINLOCK(boot_lock);

extern void exit_lp2(void);

static inline volatile NvU8 *TegraScuAddress(void)
{
    NvRmPhysAddr Pa;
    NvU32 Len;
    volatile NvU8 *pScu = NULL;
    NvRmModuleGetBaseAddress(s_hRmGlobal,
        NVRM_MODULE_ID(NvRmPrivModuleID_ArmPerif, 0), &Pa, &Len);

    if (Pa==0xffffffffUL || !Len) {
        printk("TegraSMP: No SCU present\n");
        return NULL;
    }

    NvRmPhysicalMemMap(Pa, Len, NVOS_MEM_READ_WRITE, 
        NvOsMemAttribute_Uncached, (void**)&pScu);
    return pScu;
}

#if 0
#define TegraCoreCount() 1
#else
static unsigned int __init TegraCoreCount(void)
{
    volatile NvU8 *pScu = TegraScuAddress();
    unsigned int Cores = 1;
    if (pScu) {
        Cores = NV_READ32(pScu + SCU_CONFIG_0);
        Cores = NV_DRF_VAL(SCU, CONFIG, CPU_NUM, Cores) + 1;
    }

    if (Cores>NR_CPUS) {
        printk("TegraSMP: Kernel configured for fewer NR_CPUS than hardware\n");
        Cores = NR_CPUS;
    }
    return Cores;
}
#endif

void platform_secondary_init(unsigned int cpu)
{
    NvRmPhysAddr Pa;
    NvU32 Len;
    volatile NvU8* pArm = NULL;
    static unsigned int first_init = 1;
    
    if (first_init == 0)
        return;

    trace_hardirqs_off();

    spin_lock(&boot_lock);
    spin_unlock(&boot_lock);

    NvRmModuleGetBaseAddress(s_hRmGlobal,
        NVRM_MODULE_ID(NvRmPrivModuleID_ArmPerif,0), &Pa, &Len);
    BUG_ON(Pa==-1UL || !Len);
    NvRmPhysicalMemMap(Pa, Len, NVOS_MEM_READ_WRITE, 
        NvOsMemAttribute_Uncached, (void**)&pArm);
    BUG_ON(!pArm);

    gic_cpu_init(0, (void __iomem*)pArm + FIC_PROC_IF_CONTROL_0);
    
    first_init = 0;
}

void __init smp_init_cpus(void)
{
    unsigned int i;
    unsigned int Cores = TegraCoreCount();

    for (i=0; i<Cores; i++)
        cpu_set(i, cpu_possible_map);
}

void __init smp_prepare_cpus(unsigned int Max)
{
    unsigned int Cores = TegraCoreCount();
    unsigned int Cpu = smp_processor_id();
    volatile NvU8 *pScu = NULL;
    unsigned int i;

    smp_store_cpu_info(Cpu);


    pScu = TegraScuAddress();
    if (!pScu)
        Cores = 1;

    Max = NV_MIN(Max, Cores);

    for (i=0; i<Max; i++)
        cpu_set(i, cpu_present_map);

    if (Max > 1)
    {
        /*
         * Enable the local timer or broadcast device for the
         * boot CPU, but only if we have more than one CPU.
         */
        percpu_timer_setup();

        NvU32 v = NV_READ32(pScu + SCU_CONTROL_0);
        v = NV_FLD_SET_DRF_NUM(SCU, CONTROL, SCU_ENABLE, 1, v);
        NV_WRITE32(pScu + SCU_CONTROL_0, v);
    }
}

#define CHECK_ADDR(P,L,N)                                   \
    do {                                                    \
        if ((P)==-1UL || !(L))                              \
        {                                                   \
            printk("TegraSMP: No %s module present\n", #N); \
            return -ENOSYS;                                 \
        }                                                   \
    } while (0);

int __cpuinit boot_secondary(unsigned int cpu, struct task_struct *idle)
{
    volatile NvU8 *pEvp = NULL;
    volatile NvU8 *pFlowCtrl = NULL;
    volatile NvU8 *pClkRst = NULL;
    NvUPtr BootFunc;
    NvRmPhysAddr Pa;
    NvU32 Len;
    NvU32 HaltAddr;
    NvU32 ResetVal;
    NvU32 ClkEnable;
    NvU32 OldReset;
    NvU32 v, Msg;
    extern void tegra_secondary_startup(void);
#ifdef CONFIG_HOTPLUG_CPU
    extern void TegraHotplugStartup(void);
    static NvU32 EnabledCores = 0;
#endif
    unsigned long timeout;

    spin_lock(&boot_lock);

    /*  Map exception vector, flow controller and clock & reset module */
    NvRmModuleGetBaseAddress(s_hRmGlobal,
        NVRM_MODULE_ID(NvRmModuleID_ExceptionVector, 0), &Pa, &Len);
    CHECK_ADDR(Pa, Len, EVP);
    NvRmPhysicalMemMap(Pa, Len, NVOS_MEM_READ_WRITE, 
        NvOsMemAttribute_Uncached, (void**)&pEvp);
    NvRmModuleGetBaseAddress(s_hRmGlobal,
        NVRM_MODULE_ID(NvRmModuleID_FlowCtrl, 0), &Pa, &Len);
    CHECK_ADDR(Pa, Len, FlowCtrl);
    NvRmPhysicalMemMap(Pa, Len, NVOS_MEM_READ_WRITE,
        NvOsMemAttribute_Uncached, (void**)&pFlowCtrl);
    NvRmModuleGetBaseAddress(s_hRmGlobal,
        NVRM_MODULE_ID(NvRmPrivModuleID_ClockAndReset,0), &Pa, &Len);
    CHECK_ADDR(Pa, Len, ClockReset);
    NvRmPhysicalMemMap(Pa, Len, NVOS_MEM_READ_WRITE,
        NvOsMemAttribute_Uncached, (void**)&pClkRst);

    if (!pClkRst || !pFlowCtrl || !pEvp)
    {
        printk("TegraSMP: Unable to map necessary modules for SMP start-up\n");
        return -ENOSYS;
    }

    ResetVal =
        NV_DRF_NUM(CLK_RST_CONTROLLER, RST_CPU_CMPLX_CLR, CLR_CPURESET0, 1)|
        NV_DRF_NUM(CLK_RST_CONTROLLER, RST_CPU_CMPLX_CLR, CLR_DBGRESET0, 1)|
        NV_DRF_NUM(CLK_RST_CONTROLLER, RST_CPU_CMPLX_CLR, CLR_DERESET0, 1);
    ResetVal <<= cpu;

    switch (cpu) {
    case 0:
        //  Kernel should never call this, since master CPU will always be up
        HaltAddr = FLOW_CTLR_HALT_CPU_EVENTS_0;
        ClkEnable = 
            ~NV_DRF_NUM(CLK_RST_CONTROLLER, CLK_CPU_CMPLX, CPU0_CLK_STP, 1);
        break;
    case 1:
        HaltAddr = FLOW_CTLR_HALT_CPU1_EVENTS_0;
        ClkEnable = 
            ~NV_DRF_NUM(CLK_RST_CONTROLLER, CLK_CPU_CMPLX, CPU1_CLK_STP, 1);
        break;
    default:
        panic("Unsupported cpu ID: %u\n", cpu);
    }

#ifdef CONFIG_HOTPLUG_CPU
    if (EnabledCores & (1<<cpu)) {
        BootFunc = virt_to_phys((void*)exit_lp2);
    }
    else
#endif
    {
        BootFunc = virt_to_phys((void*)tegra_secondary_startup);
    }
    OldReset = NV_READ32(pEvp + EVP_CPU_RESET_VECTOR_0);
    smp_wmb();
    flush_cache_all();
    
    NV_WRITE32(pEvp + EVP_CPU_RESET_VECTOR_0, BootFunc);
    dsb();
    isb();
    NV_WRITE32(pFlowCtrl + HaltAddr,
        NV_DRF_DEF(FLOW_CTLR, HALT_CPU_EVENTS, MODE, FLOW_MODE_NONE));
    v = NV_READ32(pClkRst + CLK_RST_CONTROLLER_CLK_CPU_CMPLX_0);
    dsb();
    isb();
    v &= ClkEnable;
    NV_WRITE32(pClkRst + CLK_RST_CONTROLLER_CLK_CPU_CMPLX_0, v);
    dsb();
    isb();
    NV_WRITE32(pClkRst + CLK_RST_CONTROLLER_RST_CPU_CMPLX_CLR_0, ResetVal);
    dsb();
    isb();

    timeout = jiffies + (10 * HZ);
    do {
        /* The slave CPU will put its ID into the reset vector register after
         * it initializes its cache */
        Msg = NV_READ32(pEvp + EVP_CPU_RESET_VECTOR_0);
        if (Msg != BootFunc)
            break;
    } while (time_before(jiffies, timeout));

    /* Restore the original reset vector, after either the slave processor
     * wakes up, or we timeout waiting for it */
    NV_WRITE32(pEvp + EVP_CPU_RESET_VECTOR_0, OldReset);

    spin_unlock(&boot_lock);
    if (Msg == BootFunc) {
        printk(KERN_INFO "TegraSMP: Failed to init CPU %u\n", cpu);
        return -ENOSYS;
    }

    printk(KERN_INFO "TegraSMP: CPU %u responded with \"0x%08x\"\n", cpu, Msg);

#ifdef CONFIG_HOTPLUG_CPU
    EnabledCores |= (1<<cpu);
#endif

    return 0;
}
