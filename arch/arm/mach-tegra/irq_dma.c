/*
 * arch/arm/mach-tegra/irq_dma.c
 *
 * Second-level IRQ decoder for system DMA driver
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
#include "ap15/arapbdma.h"
#include <asm/io.h>

#include <mach/nvrm_linux.h>
#include "nvrm_interrupt.h"

static int dma_irq_start;
static int dma_main_irq;
static void __iomem *apb_dma_regs;
static NvOsPhysAddr dma_phys;
static unsigned int dma_size;

static void dma_irq_mask(unsigned int irq)
{
    unsigned long reg;

    reg = 1 << (irq - dma_irq_start);
    writel(reg, apb_dma_regs + APBDMA_IRQ_MASK_CLR_0);
}

static void dma_irq_unmask(unsigned int irq)
{
    unsigned long reg;

    reg = 1 << (irq - dma_irq_start);
    writel(reg, apb_dma_regs + APBDMA_IRQ_MASK_SET_0);
}

static void dma_irq_ack(unsigned int irq)
{

}

static struct irq_chip dma_irq_chip = {
    .name           = "APBDMA",
    .ack            = dma_irq_ack,
    .mask           = dma_irq_mask,
    .unmask         = dma_irq_unmask,
};


static void dma_irq_handler(unsigned int irqMain, struct irq_desc *desc)
{
	struct irq_chip *chip = get_irq_chip(irqMain);
    unsigned int channel;
    unsigned int reg;

    chip->ack(irqMain);

    reg = readl(apb_dma_regs + APBDMA_IRQ_STA_CPU_0);
    if (reg) {
        __asm__ __volatile__ (      \
            "clz %0, %1 \r\t"       \
            :"=r"(channel)          \
            :"r"(reg));
        channel = 31 - channel;

        reg = writel(1 << channel, apb_dma_regs + APBDMA_IRQ_STA_CPU_0);
        generic_handle_irq(channel + dma_irq_start);
    }
    chip->unmask(irqMain);
}

void __init tegra_init_dma(void)
{
    int num_channels;
    int channel;
    int irq;

    NvRmModuleGetBaseAddress(s_hRmGlobal, 
        NVRM_MODULE_ID(NvRmPrivModuleID_ApbDma, 0), 
        &dma_phys, &dma_size);

    apb_dma_regs = IO_ADDRESS(dma_phys);
    if (!apb_dma_regs)
        return;

    num_channels = NvRmModuleGetNumInstances(s_hRmGlobal, NvRmPrivModuleID_ApbDmaChannel);
    if (num_channels == 0)
        return;

    dma_main_irq = NvRmGetIrqForLogicalInterrupt(s_hRmGlobal, 
        NVRM_MODULE_ID(NvRmPrivModuleID_ApbDma, 0), 0xff);

    dma_irq_start = NvRmGetIrqForLogicalInterrupt(s_hRmGlobal, 
        NVRM_MODULE_ID(NvRmPrivModuleID_ApbDma, 0), 0);

    channel = num_channels;
    while (channel--)
    {
        irq = NvRmGetIrqForLogicalInterrupt(s_hRmGlobal, 
            NVRM_MODULE_ID(NvRmPrivModuleID_ApbDma, 0), channel);

        set_irq_chip(irq, &dma_irq_chip);
        set_irq_handler(irq, handle_level_irq);
        set_irq_flags(irq, IRQF_VALID);
    }
    set_irq_chained_handler(dma_main_irq, dma_irq_handler);
}

