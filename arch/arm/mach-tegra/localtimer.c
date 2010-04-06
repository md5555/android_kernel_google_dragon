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
#include "mach/timex.h"

#if defined(CONFIG_ARCH_TEGRA_2x_SOC)
#define LOCAL_TIMER_ADDR 0x50040600UL
#define LOCAL_TIMER_IRQ  29
#define CPU_FREQ_SCALE_SHIFT	24
#define CPU_FREQ_SCALE_DIVIDER	(0x1 << CPU_FREQ_SCALE_SHIFT)
#define CPU_FREQ_SCALE_INIT	125
#else
#error "Unsupported Tegra SoC family"
#endif

#if defined(CONFIG_LOCAL_TIMERS) && defined(CONFIG_HAVE_ARM_TWD)

#if defined(CONFIG_USE_ARM_TWD_PRESCALER)
/*
 * Find new prescaler value for cpu frequency changes, so that local timer
 * input frequency is kept at calibration level. Save new value in shadow
 * variable - do not update h/w.
 */
void local_timer_rescale(unsigned long cpu_freq_khz)
{
	static unsigned long cpu_freq_scale_mult = 0;
	unsigned long scale;

	/* 1st call at boot/calibration frequency */
	if (cpu_freq_scale_mult == 0) {
		cpu_freq_scale_mult = ((timer_prescaler + 1) <<
			CPU_FREQ_SCALE_SHIFT) / cpu_freq_khz;
		printk("Local timer scaling factor %d, shift %d\n",
			cpu_freq_scale_mult, CPU_FREQ_SCALE_SHIFT);
		return;
	}

	scale = (unsigned long)((((uint64_t)cpu_freq_khz * cpu_freq_scale_mult)
			+ CPU_FREQ_SCALE_DIVIDER -1) >> CPU_FREQ_SCALE_SHIFT);
	timer_prescaler = scale - 1;
}
#endif

void __cpuinit local_timer_setup(struct clock_event_device *clk)
{
#if defined(CONFIG_USE_ARM_TWD_PRESCALER)
	if (timer_prescaler == 0)
		timer_prescaler = CPU_FREQ_SCALE_INIT - 1;
#endif
	twd_base = IO_ADDRESS(LOCAL_TIMER_ADDR);
	clk->irq = LOCAL_TIMER_IRQ;
	twd_timer_setup(clk);
}

#endif

