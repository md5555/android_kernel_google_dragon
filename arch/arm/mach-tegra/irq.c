/*
 * arch/arm/mach-tegra/irq.c
 *
 * IRQ chip driver for Tegra SoCs
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
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <asm/hardware/gic.h>

#include "nvcommon.h"
#include "nvrm_init.h"
#include "nvrm_drf.h"
#include "nvrm_hardware_access.h"
#include "nvrm_module.h"
#include "nvrm_interrupt.h"
#include "mach/nvrm_linux.h"
#include "ap15/arictlr.h"
#include "ap15/arapb_misc.h"
#include "ap20/arfic_proc_if.h"
#include "ap20/arfic_dist.h"
#include "mach/board.h"

#ifdef CONFIG_TEGRA_SYSTEM_DMA
extern void __init tegra_init_dma(void);  /* irq_dma.c */
#endif
extern void __init tegra_init_gpio(void); /* irq_gpio.c */

/* Exported symbol shared with NvOs defining the number of non-GPIO interrupts
 * present on the system */
NvU32 g_NvNumSocIrqs = 0;

/* Causes the interrupt decoder to use the legacy portal player decoder */
NvU32 g_NvUsePpiDecoder = 0;
void __iomem *g_NvIctlrBase = NULL;

#ifdef CONFIG_CPU_AP15
#define NV_MAX_IRQ_INSTANCES 3
static volatile NvU8 *s_Controllers[NV_MAX_IRQ_INSTANCES] = {NULL};

static struct irq_chip s_NvIrqDispatch = {
	.name = "tegra",
};

static void NvPrivAckIrq(unsigned int irq)
{
	/* nothing to do */
}

static void NvPrivAp15MaskIrq(unsigned int irq)
{
	NV_WRITE32(s_Controllers[irq>>5] + ICTLR_CPU_IER_CLR_0,
			   1 << (irq & 31));
}

static void NvPrivAp15UnmaskIrq(unsigned int irq)
{
	NV_WRITE32(s_Controllers[irq>>5] + ICTLR_CPU_IER_SET_0,
			   1 << (irq & 31));
}

static struct irq_chip* __init NvPrivAp15InitIrq(void)
{
	NvU32 i;
	NvRmPhysAddr Phys;
	NvU32 Len;
	NvU32 Num;

	Num = NvRmModuleGetNumInstances(s_hRmGlobal,NvRmPrivModuleID_Interrupt);
	if (Num > NV_MAX_IRQ_INSTANCES) {
		printk("More interrupt controllers than static array size\n");
		while (1) { }
	}
	g_NvNumSocIrqs = Num*32;
	
	for (i=0; i<Num; i++) {
		NvRmModuleGetBaseAddress(s_hRmGlobal,
			NVRM_MODULE_ID(NvRmPrivModuleID_Interrupt,i),
			&Phys, &Len);
		if (NvRmPhysicalMemMap(Phys, Len, NVOS_MEM_READ_WRITE,
			NvOsMemAttribute_Uncached,
			(void **)&s_Controllers[i])!=NvSuccess) {
			printk("Failed to get IRQ controller base address\n");
			while (1) { }
		}
		NV_WRITE32(s_Controllers[i] + ICTLR_CPU_IER_CLR_0, ~0UL);
		NV_WRITE32(s_Controllers[i] + ICTLR_CPU_IEP_CLASS_0, 0);
	}

	s_NvIrqDispatch.mask = NvPrivAp15MaskIrq;
	s_NvIrqDispatch.unmask = NvPrivAp15UnmaskIrq;
	s_NvIrqDispatch.ack = NvPrivAckIrq;
	s_NvIrqDispatch.set_type = NULL;
	g_NvIctlrBase = (void __iomem*)s_Controllers[0];
	g_NvUsePpiDecoder = 1;
	return &s_NvIrqDispatch;
}
#endif

#ifdef CONFIG_ARM_GIC
#define NV_MAX_IRQ_INSTANCES 4
static volatile NvU8 *s_Controllers[NV_MAX_IRQ_INSTANCES] = {NULL};
extern void gic_mask_irq(unsigned int);
extern void gic_unmask_irq(unsigned int);
extern void gic_ack_irq(unsigned int);
extern void gic_set_cpu(unsigned int, const struct cpumask*);

static struct irq_chip s_NvIrqDispatch = 
{
	.name = "tegra2",
};

void NvPrivAp20MaskIrq(unsigned int irq)
{
	gic_mask_irq(irq);
	
	irq -= 32;
	
	NV_WRITE32(s_Controllers[irq>>5] + ICTLR_CPU_IER_CLR_0,
		1 << (irq & 31));
			   
}

void NvPrivAp20UnmaskIrq(unsigned int irq)
{
	gic_unmask_irq(irq);
	
	irq -= 32;
	
	NV_WRITE32(s_Controllers[irq>>5] + ICTLR_CPU_IER_SET_0,
		1 << (irq & 31));
}

static void NvPrivAp20AckIrq(unsigned int irq)
{
	gic_ack_irq(irq);
}

#ifdef CONFIG_SMP
static void NvPrivAp20SetCpu(unsigned int irq, const struct cpumask *mask_val)
{
	gic_set_cpu(irq, mask_val);
}
#endif

static void tegra_irq_register_module(NvRmModuleID Module,
	struct irq_chip* pIrq)
{
	NvU32 i, Num;
	Num = NvRmModuleGetNumInstances(s_hRmGlobal, Module);
	for (i=0; i<Num; i++) {
		NvU32 Ints;
		Ints = NvRmGetIrqCountForLogicalInterrupt(s_hRmGlobal,
			NVRM_MODULE_ID(Module,i));
		while (Ints) {
			NvU32 Irq;
			--Ints;
			if (Module == NvRmPrivModuleID_Gpio) {
			/* GPIOs are handled differently. Here we set the
			 * handler for the entire GPIO controller. Each GPIO
			 * controller has a set of GPIO ports which can
			 * programmed by the GPIO APIs. Check the NvRmGpio*
			 * APIs for more information
			 *
			 * To get the main IRQ line connected to the GPIO line,
			 * use an index value of 0xff.
			 */
				Irq = NvRmGetIrqForLogicalInterrupt(s_hRmGlobal,
					NVRM_MODULE_ID(Module, i), 0xff);
			} else {
				Irq = NvRmGetIrqForLogicalInterrupt(s_hRmGlobal,
					NVRM_MODULE_ID(Module, i), Ints);
			}

			if (pIrq) {
				if (set_irq_chip(Irq, pIrq)) {
					panic("set_irq_chip %d failed\n", Irq);
					}
				set_irq_handler(Irq, handle_level_irq);
			}

			set_irq_flags(Irq, IRQF_VALID);
		}
	}
}

static struct irq_chip* __init NvPrivGicInitIrq(void)
{
	NvRmPhysAddr Phys;
	NvU32 Len;
	NvU32 Num;
	NvU32 i;
	volatile NvU8 *pArm = NULL;

	Num = NvRmModuleGetNumInstances(s_hRmGlobal,
		NvRmPrivModuleID_Interrupt);
	g_NvNumSocIrqs = (Num+1)*32;

	NvRmModuleGetBaseAddress(s_hRmGlobal,
		NVRM_MODULE_ID(NvRmPrivModuleID_ArmPerif,0), &Phys, &Len);

	if (NvRmPhysicalMemMap(Phys, Len, NVOS_MEM_READ_WRITE,
		NvOsMemAttribute_Uncached, (void **)&pArm)!=NvSuccess) {
		panic("Unable to map interrupt controller aperture\n");
	}

	gic_dist_init(0,
		(void __iomem*)(pArm + FIC_DIST_DISTRIBUTOR_ENABLE_0), 29);
	gic_cpu_init(0, (void __iomem*)(pArm + FIC_PROC_IF_CONTROL_0));

	g_NvIctlrBase = (void __iomem*)(pArm + FIC_PROC_IF_CONTROL_0);

	/* gic_dist_init sets the IRQF_PROBE and IRQF_VALID flags for the entire
	 * range of distributor IRQs.  The Tegra code undoes this, and then
	 * respecifies the desired flags for only valid IRQ numbers */
	for (i=32; i<g_NvNumSocIrqs; i++)
		set_irq_flags(i, 0);

	/* Set up the legacy controller as well. Needed for flow controller */
	for (i=0; i<Num; i++) {
		NvRmModuleGetBaseAddress(s_hRmGlobal,
			NVRM_MODULE_ID(NvRmPrivModuleID_Interrupt,i),
			&Phys, &Len);
		if (NvRmPhysicalMemMap(Phys, Len, NVOS_MEM_READ_WRITE,
			NvOsMemAttribute_Uncached,
			(void **)&s_Controllers[i])!=NvSuccess) {
			printk("failed to get IRQ controller base address\n");
			while (1) { }
		}
		NV_WRITE32(s_Controllers[i] + ICTLR_CPU_IER_CLR_0, ~0UL);
		NV_WRITE32(s_Controllers[i] + ICTLR_CPU_IEP_CLASS_0, 0);
	}

	//Set up the irq_chip for AP20
	s_NvIrqDispatch.mask = NvPrivAp20MaskIrq;
	s_NvIrqDispatch.unmask = NvPrivAp20UnmaskIrq;
	s_NvIrqDispatch.ack = NvPrivAp20AckIrq;
#ifdef CONFIG_SMP
	s_NvIrqDispatch.set_affinity = NvPrivAp20SetCpu;
#endif

	return &s_NvIrqDispatch;
}

#endif

void __init tegra_init_irq(void)
{
	NvRmModuleID Module;
	NvU32 ChipId;
	volatile NvU8 *MiscRegionVirtual;
	NvRmPhysAddr Phys;
	NvU32 Len;
	struct irq_chip* pIrq = NULL;

	NvRmModuleGetBaseAddress(s_hRmGlobal,
		NVRM_MODULE_ID(NvRmModuleID_Misc,0), 
		&Phys, &Len);

	if (NvRmPhysicalMemMap(Phys, Len, NVOS_MEM_READ_WRITE, 
		NvOsMemAttribute_Uncached,
		(void **)&MiscRegionVirtual)!=NvSuccess) {
		printk("ERROR: Failed to get Misc controller base address\n");
		while (1) { };
	}

	ChipId = NV_READ32(MiscRegionVirtual + APB_MISC_GP_HIDREV_0);
	ChipId = NV_DRF_VAL(APB_MISC_GP, HIDREV, CHIPID, ChipId);

	if (ChipId==0x15 || ChipId==0x16) {
#ifdef CONFIG_CPU_AP15
		pIrq = NvPrivAp15InitIrq();
#else
		panic("Kernel built without APX 2600 support\n");
#endif
	}
	else if (ChipId==0x20) {
#ifdef CONFIG_ARM_GIC
		pIrq = NvPrivGicInitIrq();
#else
		panic("Kernel built without AP2x support\n");
#endif
	}
	else {
		panic("Unsupported chip ID: 0x%x\n", ChipId);
	}

	for (Module = NvRmModuleID_Cpu; Module<NvRmPrivModuleID_Num; Module++) {

		/* Skip interrupt registration for interrupt controllers  */
		if ((Module == NvRmPrivModuleID_Interrupt)
			|| (Module == NvRmPrivModuleID_ArmInterruptctrl)) {
			continue;
		}

                tegra_irq_register_module(Module, pIrq);
	}
 
	tegra_init_gpio();

#ifdef CONFIG_TEGRA_SYSTEM_DMA
	tegra_init_dma();
#endif
}
