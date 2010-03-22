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
#include <linux/kernel.h>
#include <linux/irq.h>
#include <linux/smp.h>
#include <linux/io.h>
#include <linux/interrupt.h>

#include <asm/irq.h>
#include <asm/bitops.h>
#include <mach/platform.h>

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
#define INT_PPI_ADDRESS(_inst) (0x60004000ul + 0x100*(_inst))
#define INT_APBDMA_ADDRESS 0x6000a000ul
#else
#error "interrupt controller addresses not defined"
#endif

#ifdef CONFIG_ARM_GIC
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
#define ARM_PERIF_BASE 0x50040000ul
#define GIC_PROC_IF_BASE (ARM_PERIF_BASE + 0x100ul)
#define GIC_DIST_BASE (ARM_PERIF_BASE + 0x1000ul)
#else
#error "interrupt distributor address not defined"
#endif
#endif

#define ICTLR_CPU_IER_0       0x20
#define ICTLR_CPU_IER_SET_0   0x24
#define ICTLR_CPU_IER_CLR_0   0x28
#define ICTLR_CPU_IEP_CLASS_0 0x2c
#define ICTLR_COP_IER_0       0x30
#define ICTLR_COP_IER_SET_0   0x34
#define ICTLR_COP_IER_CLR_0   0x38
#define ICTLR_COP_IEP_CLASS_0 0x3c

#define APBDMA_IRQ_STA_CPU_0  0x14
#define APBDMA_IRQ_MASK_SET_0 0x20
#define APBDMA_IRQ_MASK_CLR_0 0x24

#ifdef CONFIG_ARM_GIC
extern void gic_mask_irq(unsigned int);
extern void gic_unmask_irq(unsigned int);
extern void gic_ack_irq(unsigned int);
extern void gic_set_cpu(unsigned int, const struct cpumask*);
extern void gic_dist_init(unsigned int, void __iomem *, unsigned int);
extern void gic_cpu_init(unsigned int, void __iomem *);
#else
#define gic_mask_irq(i) do { } while (0)
#define gic_unmask_irq(i) do { } while (0)
#define gic_ack_irq(i) do { } while (0)
#define gic_set_cpu NULL
#define gic_dist_init(i, p, s) do { } while (0)
#define gic_cpu_init(i, p) do { } while (0)
#endif

struct tegra_irq_chip {
	unsigned int irq_start;
	void __iomem *mmio;
	/* context save/restore data for interrupts */
#ifdef CONFIG_PM
	u32 cpu_ier;
	u32 cop_ier;
#endif
};

static struct tegra_irq_chip tegra_chip[(INT_SYS_NR+INT_SYS_SZ-1)/INT_SYS_SZ];

static void tegra_mask(unsigned int irq)
{
	struct tegra_irq_chip *chip;
	gic_mask_irq(irq);
	irq -= INT_PRI_BASE;
	chip = &tegra_chip[irq/INT_SYS_SZ];
	writel(1<<(irq&31), chip->mmio + ICTLR_CPU_IER_CLR_0);
}

static void tegra_unmask(unsigned int irq)
{
	struct tegra_irq_chip *chip;
	gic_unmask_irq(irq);
	irq -= INT_PRI_BASE;
	chip = &tegra_chip[irq/INT_SYS_SZ];
	writel(1<<(irq&31), chip->mmio + ICTLR_CPU_IER_SET_0);
}

static void tegra_ack(unsigned int irq)
{
	gic_ack_irq(irq);
}

#ifdef CONFIG_PM

static int tegra_set_wake(unsigned int irq, unsigned int on)
{
	return 0;
}

void tegra_irq_resume(void)
{
	unsigned long flags;
	int i;

        local_irq_save(flags);
	for (i=0; i<ARRAY_SIZE(tegra_chip); i++) {
		struct tegra_irq_chip *chip = &tegra_chip[i];
		writel(chip->cpu_ier, chip->mmio + ICTLR_CPU_IER_SET_0);
		writel(0, chip->mmio + ICTLR_CPU_IEP_CLASS_0);
		writel(chip->cop_ier, chip->mmio + ICTLR_COP_IER_SET_0);
		writel(0, chip->mmio + ICTLR_COP_IEP_CLASS_0);
	}
	local_irq_restore(flags);

	for (i=INT_PRI_BASE; i<INT_GPIO_BASE; i++) {
		struct irq_desc *desc = irq_to_desc(i);
		if (!desc || (desc->status & IRQ_WAKEUP)) continue;
		enable_irq(i);
	}

	for (i=INT_APBDMA_BASE; i<INT_APBDMA_BASE+INT_APBDMA_NR; i++)
		enable_irq(i);
}

void tegra_irq_suspend(void)
{
	unsigned long flags;
	int i;

	for (i=INT_APBDMA_BASE; i<INT_APBDMA_BASE+INT_APBDMA_NR; i++)
		disable_irq(i);

	for (i=INT_PRI_BASE; i<INT_GPIO_BASE; i++) {
		struct irq_desc *desc= irq_to_desc(i);
		if (!desc) continue;
		if (desc->status & IRQ_WAKEUP) {
			pr_debug("irq %d is wakeup\n", i);
			continue;
		}
		disable_irq(i);
	}

	local_irq_save(flags);
	for (i=0; i<ARRAY_SIZE(tegra_chip); i++) {
		struct tegra_irq_chip *chip = &tegra_chip[i];
		chip->cpu_ier = readl(chip->mmio + ICTLR_CPU_IER_0);
		chip->cop_ier = readl(chip->mmio + ICTLR_COP_IER_0);
		writel(~0ul, chip->mmio + ICTLR_COP_IER_CLR_0);
	}
	local_irq_restore(flags);
}

#endif


#ifdef CONFIG_TEGRA_SYSTEM_DMA
struct apbdma_irq_chip {
	unsigned int irq_start;
	void __iomem *mmio;
	spinlock_t lock;
};

static struct apbdma_irq_chip apbdma_chip;

static void apbdma_ack(unsigned int irq) { }

static void apbdma_mask(unsigned int irq)
{
	struct apbdma_irq_chip *chip = get_irq_chip_data(irq);
	irq -= chip->irq_start;
	writel(1<<irq, chip->mmio + APBDMA_IRQ_MASK_CLR_0);
}

static void apbdma_unmask(unsigned int irq)
{
	struct apbdma_irq_chip *chip = get_irq_chip_data(irq);
	irq -= chip->irq_start;
	writel(1<<irq, chip->mmio + APBDMA_IRQ_MASK_SET_0);
}

static void apbdma_cascade(unsigned int irq, struct irq_desc *desc)
{
	struct irq_chip *pri = get_irq_chip(irq);
	struct apbdma_irq_chip *chip = get_irq_data(irq);
	u32 reg, ch=0;

	pri->ack(irq);
	spin_lock(&chip->lock);
	reg = readl(chip->mmio + APBDMA_IRQ_STA_CPU_0);
	if (reg) {
		reg = __fls(reg);
		writel(1<<reg, chip->mmio + APBDMA_IRQ_STA_CPU_0);
		ch = chip->irq_start + reg;
	}
	spin_unlock(&chip->lock);
	if (ch)	generic_handle_irq(ch);
	pri->unmask(irq);
}

static struct irq_chip apbdma_irq = {
	.name	= "APBDMA",
	.ack	= apbdma_ack,
	.mask	= apbdma_mask,
	.unmask	= apbdma_unmask,
};
#endif

static struct irq_chip tegra_irq = {
	.name		= "PPI",
	.mask		= tegra_mask,
	.unmask		= tegra_unmask,
	.ack		= tegra_ack,
#ifdef CONFIG_PM
	.set_wake	= tegra_set_wake,
#endif
#ifdef CONFIG_SMP
	.set_affinity	= gic_set_cpu,
#endif

};

void __init tegra_init_irq(void)
{
	unsigned int i;

	for (i=0; i<ARRAY_SIZE(tegra_chip); i++) {
		tegra_chip[i].irq_start = INT_PRI_BASE + INT_SYS_SZ*i;
		tegra_chip[i].mmio = IO_ADDRESS(INT_PPI_ADDRESS(i));
		writel(~0ul, tegra_chip[i].mmio + ICTLR_CPU_IER_CLR_0);
		writel(0, tegra_chip[i].mmio + ICTLR_CPU_IEP_CLASS_0);
	}

	gic_dist_init(0, IO_ADDRESS(GIC_DIST_BASE), 29);
	gic_cpu_init(0, IO_ADDRESS(GIC_PROC_IF_BASE));

	for (i=INT_PRI_BASE; i<INT_GPIO_BASE; i++) {
		set_irq_chip(i, &tegra_irq);
		set_irq_handler(i, handle_level_irq);
		set_irq_flags(i, IRQF_VALID);
	}

#ifdef CONFIG_TEGRA_SYSTEM_DMA
	apbdma_chip.mmio = IO_ADDRESS(INT_APBDMA_ADDRESS);
	spin_lock_init(&apbdma_chip.lock);
	apbdma_chip.irq_start = INT_APBDMA_BASE;

	for (i=INT_APBDMA_BASE; i<INT_APBDMA_NR+INT_APBDMA_BASE; i++) {
		set_irq_chip(i, &apbdma_irq);
		set_irq_chip_data(i, &apbdma_chip);
		set_irq_handler(i, handle_level_irq);
		set_irq_flags(i, IRQF_VALID);
	}
	if (set_irq_data(INT_APB_DMA, &apbdma_chip))
		BUG();
	set_irq_chained_handler(INT_APB_DMA, apbdma_cascade);
#endif
}
