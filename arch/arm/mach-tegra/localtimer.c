/*
 * arch/arm/mach-tegra/localtimer.c
 *
 * CPU local timer support for SMP Tegra SoCs
 *
 * Copyright (c) 2008-2009, NVIDIA Corporation.
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

/*
 *  For SMP, linux needs one local timer per CPU. This is in addition to the
 *  global timner that the linux requires. These local timers can be either a
 *  seperate timer hardware one per CPU or can be key'ed off the global timer
 *  using the IPI timer broadcast mechanism.
 *
 */

#include <linux/time.h>
#include <linux/smp.h>
#include <linux/percpu.h>
#include <linux/clockchips.h>
#include <asm/smp_twd.h>

#if defined(CONFIG_ARCH_TEGRA_2x_SOC)
#define LOCAL_TIMER_ADDR 0x50040600UL
#define LOCAL_TIMER_IRQ  29
#else
#error "Unsupported Tegra SoC family"
#endif

#if defined(CONFIG_LOCAL_TIMERS) && defined(CONFIG_HAVE_ARM_TWD)

void __cpuinit local_timer_setup(struct clock_event_device *clk)
{
	twd_base = IO_ADDRESS(LOCAL_TIMER_ADDR);
	clk->irq = LOCAL_TIMER_IRQ;
	twd_timer_setup(clk);
}

#endif

