/*
 * arch/arm/mach-tegra/platsmp.c
 *
 * SMP management routines for SMP Tegra SoCs
 *
 * Copyright (c) 2010, NVIDIA Corporation.
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
#include <linux/io.h>
#include <mach/iomap.h>

static DEFINE_SPINLOCK(boot_lock);
extern void exit_lp2(void);
extern void tegra_secondary_startup(void);

#define SCU_CONTROL_0 0x0
#define SCU_CONFIG_0 0x4

#define EVP_CPU_RESET_VECTOR_0 0x100

/* takes cpu out of reset */
#define CLK_RST_CONTROLLER_RST_CPU_CMPLX_CLR_0 0x344
#define CPU_RESET(cpu)    (0x1011ul<<(cpu))

/* used as mask to enable clock to cpu */
#define CLK_RST_CONTROLLER_CLK_CPU_CMPLX_0 0x4c
#define CPU_CLK_STOP(cpu) (0x1<<(8+cpu))

/* write 0 to take cpu out of flow controlled state */
#define FLOW_CTRL_HALT_CPUx_EVENTS(cpu) ((cpu)?((cpu-1)*0x8 + 0x14) : 0x0)

static DECLARE_BITMAP(cpu_init_bits, CONFIG_NR_CPUS) __read_mostly;
const struct cpumask *const cpu_init_mask = to_cpumask(cpu_init_bits);
#define cpu_init_map (*(cpumask_t *)cpu_init_mask)

static u32 orig_reset;

void platform_secondary_init(unsigned int cpu)
{
	if (cpumask_test_cpu(cpu, cpu_init_mask))
		return;

	trace_hardirqs_off();
	spin_lock(&boot_lock);
	cpu_set(cpu, cpu_init_map);
	spin_unlock(&boot_lock);

	gic_cpu_init(0, IO_ADDRESS(TEGRA_GIC_PROC_IF_BASE));
}

void __init smp_init_cpus()
{
	unsigned int cfg;
	unsigned int cpus;
	void __iomem *evp = IO_ADDRESS(TEGRA_EXCEPTION_VECTORS_BASE);

	cfg = __raw_readl(IO_ADDRESS(TEGRA_SCU_BASE) + SCU_CONFIG_0);

	cpus = min_t(unsigned int, NR_CPUS, (cfg & 3) + 1);

	while (cpus--)
		cpu_set(cpus, cpu_possible_map);

	orig_reset = __raw_readl(evp + EVP_CPU_RESET_VECTOR_0);
}

void __init smp_prepare_cpus(unsigned int max)
{
	unsigned int cpu;

	smp_store_cpu_info(smp_processor_id());
	for_each_possible_cpu(cpu)
		cpu_set(cpu, cpu_present_map);

	if (num_present_cpus()>1) {
		u32 ctrl;

		percpu_timer_setup();
		ctrl = __raw_readl(IO_ADDRESS(TEGRA_SCU_BASE) + SCU_CONTROL_0);
		ctrl |= 1;
		__raw_writel(ctrl, IO_ADDRESS(TEGRA_SCU_BASE) + SCU_CONTROL_0);
	}
}

static inline void bwritel(unsigned long v, void __iomem *a)
{
	__raw_writel(v, a);
	dsb();
	isb();
}

int __cpuinit boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	void __iomem *clk = IO_ADDRESS(TEGRA_CLK_RESET_BASE);
	void __iomem *flow = IO_ADDRESS(TEGRA_FLOW_CTRL_BASE);
	void __iomem *evp = IO_ADDRESS(TEGRA_EXCEPTION_VECTORS_BASE);
	unsigned long boot;
	unsigned long timeout;
	u32 r;

	spin_lock(&boot_lock);

	if (likely(cpumask_test_cpu(cpu, cpu_init_mask)))
		boot = virt_to_phys((void *)exit_lp2);
	else
		boot = virt_to_phys((void *)tegra_secondary_startup);

	flush_cache_all();
	smp_wmb();

	bwritel(boot, evp + EVP_CPU_RESET_VECTOR_0);

	bwritel(0, flow + FLOW_CTRL_HALT_CPUx_EVENTS(cpu));

	r = __raw_readl(clk + CLK_RST_CONTROLLER_CLK_CPU_CMPLX_0);
	r &= ~CPU_CLK_STOP(cpu);
	bwritel(r, clk + CLK_RST_CONTROLLER_CLK_CPU_CMPLX_0);

	bwritel(CPU_RESET(cpu), clk + CLK_RST_CONTROLLER_RST_CPU_CMPLX_CLR_0);

	timeout = jiffies + 10*HZ;

	do {
		r = __raw_readl(evp + EVP_CPU_RESET_VECTOR_0);
		if (r!=boot)
			break;
		cpu_relax();
	} while (time_before(jiffies, timeout));

	__raw_writel(orig_reset, evp + EVP_CPU_RESET_VECTOR_0);
	spin_unlock(&boot_lock);

	if (r==boot) {
		pr_err("failed to initialize CPU %u\n", cpu);
		return -EIO;
	}

	return 0;
}

#ifdef CONFIG_HOTPLUG_CPU

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
	WARN_ON(!cpu);
	if (!cpu)
		return -EPERM;

	return 0;
}


#endif
