/*
 * arch/arm/mach-tegra/cpufreq.c
 *
 * cpufreq driver for NVIDIA Tegra SoCs
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

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/cpufreq.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <asm/system.h>

#include <mach/hardware.h>
#include <mach/nvrm_linux.h>
#include <nvrm_power.h>
#include <nvos.h>


NvU32 s_PowerClientId = 0;

static int tegra_verify_speed(struct cpufreq_policy *policy)
{
	cpufreq_verify_within_limits(policy, policy->cpuinfo.min_freq,
		policy->cpuinfo.max_freq);
	return 0;
}

static unsigned int tegra_get_speed(unsigned int cpu)
{
	NvError Error;
	NvRmDfsClockUsage ClocksUsage;

	Error = NvRmDfsGetClockUtilization(s_hRmGlobal,
		NvRmDfsClockId_Cpu, &ClocksUsage);
	if (Error) {
		pr_err("%s: NvRmDfsGetClockUtilization() fail 0x%08x \n",
			__func__, Error);
		return -EINVAL;
	}
	return ClocksUsage.CurrentKHz;
}

static int tegra_set_policy(struct cpufreq_policy *policy)
{
	NvError Error;

	pr_debug("%s()++ Cpu %d Req Min:0x%08x Max: 0x%08x\n",
		__func__, policy->cpu, policy->min,policy->max);
	Error = NvRmDfsSetCpuEnvelope(s_hRmGlobal, policy->min, policy->max);
	if (Error) {
		pr_err("%s: NvRmDfsSetCpuEnvelope() fail 0x%08x \n",
			__func__, Error);
		return -EINVAL;
	}

	pr_debug("%s()-- \n",__func__);
	return 0;
}

static int tegra_cpufreq_driver_init(struct cpufreq_policy *policy)
{
	NvError Error;
	int ret = 0;
	NvRmDfsClockUsage ClocksUsage;

	pr_debug("tegra_cpufreq_driver_init()++ Cpu %d \n", policy->cpu);

	Error = NvRmDfsGetClockUtilization(s_hRmGlobal,
		NvRmDfsClockId_Cpu, &ClocksUsage);
	if (Error) {
		pr_err("%s(): NvRmDfsGetClockUtilization() Error 0x%08x\n",
			__func__, Error);
		ret = -EINVAL;
		goto end;
	}

	pr_debug("%s: The Min:Max:Curr Freq = 0x%08x:0x%08x:0x%08x\n",
		__func__, ClocksUsage.MinKHz, ClocksUsage.MaxKHz,
		ClocksUsage.CurrentKHz);

	policy->min = ClocksUsage.LowCornerKHz;
	policy->max = ClocksUsage.HighCornerKHz;
	policy->cur = ClocksUsage.CurrentKHz;

	policy->cpuinfo.min_freq = ClocksUsage.MinKHz;
	policy->cpuinfo.max_freq = ClocksUsage.MaxKHz;
	policy->cpuinfo.transition_latency = 0;

end:
	return ret;
}

static struct cpufreq_driver s_tegra_cpufreq_driver = {
	.flags		= CPUFREQ_CONST_LOOPS,
	.verify		= tegra_verify_speed,
	.setpolicy	= tegra_set_policy,
	.get		= tegra_get_speed,
	.init		= tegra_cpufreq_driver_init,
	.name		= "tegra_cpufreq",
	.owner		= THIS_MODULE,

};

static int __init tegra_cpufreq_init(void)
{
	return cpufreq_register_driver(&s_tegra_cpufreq_driver);
}

static void __exit tegra_cpufreq_exit(void)
{
	cpufreq_unregister_driver(&s_tegra_cpufreq_driver);
}

MODULE_DESCRIPTION("CPU frequency driver for the Tegra SOC");
MODULE_LICENSE("GPL");
module_init(tegra_cpufreq_init);
module_exit(tegra_cpufreq_exit);
