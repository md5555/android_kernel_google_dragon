/*
 * arch/arm/mach-tegra/include/mach/entry-macro.S
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

#if defined(CONFIG_ARM_GIC)
#include <asm/hardware/gic.h>
#endif

	.macro	disable_fiq
	.endm

	.macro	get_irqnr_preamble, base, tmp
	@ enable imprecise aborts
	cpsie	a
	@ load interrupt controller base address
#if defined(CONFIG_ARCH_TEGRA_1x_SOC)
	ldr	\base, =0xfe604000
#elif defined(CONFIG_ARCH_TEGRA_2x_SOC)
	ldr	\base, =0xfe540100
#else
#error "Invalid Tegra SoC family selection"
#endif
	.endm

	.macro	arch_ret_to_user, tmp1, tmp2
	.endm

	@ on return:
	@ Z=1 no irq pending
	@ Z=0 irq is pending, irqnr is valid
	.macro	get_irqnr_and_base, irqnr, irqstat, base, tmp
#ifdef CONFIG_ARCH_TEGRA_1x_SOC
	@ check ictlr1
	ldr	\irqstat, [\base, #0x000]

	@ check for any pending timer interrupt first
	tst	\irqstat, #1
	movne	\irqnr, #0
	bne     1001f

	cmp	\irqstat, #0
	clzne	\irqnr, \irqstat
	movne	\tmp, #31
	subne	\irqnr, \tmp, \irqnr
	bne	1001f

	@ check ictlr2
	ldr	\irqstat, [\base, #0x100]
	cmp	\irqstat, #0
	clzne	\irqnr, \irqstat
	movne	\tmp, #63
	subne	\irqnr, \tmp, \irqnr
	bne	1001f

	@ check ictlr3
	ldr	\irqstat, [\base, #0x200]
	cmp	\irqstat, #0
	clzne	\irqnr, \irqstat
	movne	\tmp, #95
	subne	\irqnr, \tmp, \irqnr
#elif defined(CONFIG_ARM_GIC)
	ldr	\irqstat, [\base, #GIC_CPU_INTACK]
	ldr	\tmp, =1021

	bic	\irqnr, \irqstat, #0x1c00

	/*
	 *  Interrupts 0-15 are IPI
	 *  16-28 are reserved.
	 *  29-31 are local. 29 is local timer and 30 is local watchdog.
	 *  32-1020 are the interrupts from the rest od the hardware blocks in
	 *  the system.
	 */

	@ Z=1 if the IRQ number == 31
	cmp	\irqnr, #29
	@ Z|=1 if IRQ number < 31
	cmpcc	\irqnr, \irqnr
	@ Z|=1 if IRQ number == 1021
	cmpne	\irqnr, \tmp
	@ Z|=1 if IRQ number > 1021
	cmpcs	\irqnr, \irqnr
#else
#error "No interrupt decoder defined for selected Tegra family"
#endif
1001:
	.endm


#ifdef CONFIG_SMP
	.macro test_for_ipi, irqnr, irqstat, base, tmp
	bic	\irqnr, \irqstat, #0x1c00
	cmp	\irqnr, #16
	strcc	\irqstat, [\base, #GIC_CPU_EOI]
	cmpcs	\irqnr, \irqnr
	.endm


	.macro test_for_ltirq, irqnr, irqstat, base, tmp
	bic	\irqnr, \irqstat, #0x1c00
	mov 	\tmp, #0
	cmp	\irqnr, #29
	moveq	\tmp, #1
	streq	\irqstat, [\base, #GIC_CPU_EOI]
	cmp	\tmp, #0
	.endm
#endif
