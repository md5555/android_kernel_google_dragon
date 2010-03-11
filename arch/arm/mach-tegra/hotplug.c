/*
 * arch/arm/mach-tegra/hotplug.c
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

#include <linux/init.h>
#include <linux/smp.h>
#include <linux/completion.h>
#include <asm/cacheflush.h>


static DECLARE_COMPLETION(cpu_killed);

extern void cpu_ap20_do_lp2(void);

int platform_cpu_kill(unsigned int cpu)
{
	return wait_for_completion_timeout(&cpu_killed, 5000);
}

void platform_cpu_die(unsigned int cpu)
{
	flush_cache_all();
	preempt_enable_no_resched();
	complete(&cpu_killed);
	cpu_ap20_do_lp2();
}

int mach_cpu_disable(unsigned int cpu)
{
	return (cpu==0) ? -EPERM : 0;
}
