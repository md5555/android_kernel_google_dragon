#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/smp.h>
#include <linux/sched.h>
#include <linux/cpuidle.h>
#include <linux/hrtimer.h>
#include <linux/cpu.h>
#include <linux/io.h>
#include <linux/tick.h>
#include <linux/interrupt.h>
#include <mach/iomap.h>

static unsigned int latency_factor __read_mostly = 2;
module_param(latency_factor, uint, 0644);

struct cpuidle_driver tegra_idle = {
	.name = "tegra_idle",
	.owner = THIS_MODULE,
};

static DEFINE_PER_CPU(struct cpuidle_device *, idle_devices);

#define FLOW_CTRL_WAITEVENT   (2<<29)
#define FLOW_CTRL_JTAG_RESUME (1<<28)
#define FLOW_CTRL_HALT_CPUx_EVENTS(cpu) ((cpu)?((cpu-1)*0x8 + 0x14) : 0x0)

#define PMC_SCRATCH_38 0x134
#define PMC_SCRATCH_39 0x138

static int tegra_idle_enter_lp3(struct cpuidle_device *dev,
	struct cpuidle_state *state)
{
	void __iomem *flow_ctrl = IO_ADDRESS(TEGRA_FLOW_CTRL_BASE);
	ktime_t enter, exit;
	s64 us;
	u32 reg = FLOW_CTRL_WAITEVENT | FLOW_CTRL_JTAG_RESUME;

	flow_ctrl = flow_ctrl + FLOW_CTRL_HALT_CPUx_EVENTS(dev->cpu);
	local_irq_disable();
	enter = ktime_get();
	if (!need_resched()) {
		dsb();
		__raw_writel(reg, flow_ctrl);
		reg = __raw_readl(flow_ctrl);
		__asm__ volatile ("wfi");
		__raw_writel(0, flow_ctrl);
		reg = __raw_readl(flow_ctrl);
	}
	exit = ktime_get();
	enter = ktime_sub(exit, enter);
	us = ktime_to_us(enter);
	local_irq_enable();
	return (int)us;
}

extern void cpu_ap20_do_lp2(void);

typedef struct NvRmDeviceRec *NvRmDeviceHandle;
extern NvRmDeviceHandle s_hRmGlobal;

extern void NvRmPrivSetLp2TimeUS(NvRmDeviceHandle, u32);
extern void tegra_lp2_set_trigger(unsigned long);

static int tegra_idle_enter_lp2(struct cpuidle_device *dev,
	struct cpuidle_state *state)
{
	ktime_t enter, exit;
	void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
	u32 idle_time;
	s64 us;

	if (num_online_cpus()>1 || rcu_needs_cpu(dev->cpu)) {
		dev->last_state = &dev->states[0];
		return tegra_idle_enter_lp3(dev, &dev->states[0]);
	}

	local_irq_disable();
	us = ktime_to_us(tick_nohz_get_sleep_length());
	if (us <= state->target_residency) {
		local_irq_enable();
		dev->last_state = &dev->states[0];
		return tegra_idle_enter_lp3(dev, &dev->states[0]);
	}
	enter = ktime_get();
	tegra_lp2_set_trigger((unsigned long)(us-state->exit_latency));
	cpu_ap20_do_lp2();
	exit = ktime_get();
	enter = ktime_sub(exit, enter);
	us = ktime_to_us(enter);
	idle_time = __raw_readl(pmc + PMC_SCRATCH_39);
	idle_time -= __raw_readl(pmc + PMC_SCRATCH_38);
	local_irq_enable();
	NvRmPrivSetLp2TimeUS(s_hRmGlobal, idle_time);
	return (int)us;
}

static int tegra_idle_enter(unsigned int cpu)
{
	struct cpuidle_device *dev;
	struct cpuidle_state *state;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->state_count = 0;
	dev->cpu = cpu;

	state = &dev->states[0];
	snprintf(state->name, CPUIDLE_NAME_LEN, "LP3");
	snprintf(state->desc, CPUIDLE_DESC_LEN, "CPU flow-controlled");
	state->exit_latency = 10;
	state->target_residency = 10;
	state->power_usage = 600;
	state->flags = CPUIDLE_FLAG_SHALLOW | CPUIDLE_FLAG_TIME_VALID;
	state->enter = tegra_idle_enter_lp3;
	dev->safe_state = state;
	dev->state_count++;

	if (cpu == 0) {
		state = &dev->states[1];
		snprintf(state->name, CPUIDLE_NAME_LEN, "LP2");
		snprintf(state->desc, CPUIDLE_DESC_LEN, "CPU power-gate");
		state->exit_latency = 4000;
		state->target_residency = state->exit_latency * latency_factor;
		state->power_usage = 0;
		state->flags = CPUIDLE_FLAG_BALANCED | CPUIDLE_FLAG_TIME_VALID;
		state->enter = tegra_idle_enter_lp2;
		dev->safe_state = state;
		dev->state_count++;
	}

	if (cpuidle_register_device(dev)) {
		pr_err("CPU%u failed to register idle device\n", cpu);
		kfree(dev);
		return -EIO;
	}
	per_cpu(idle_devices, cpu) = dev;
	return 0;
}

static int __init tegra_cpuidle_init(void)
{
	unsigned int cpu = smp_processor_id();
	int ret;

	ret = cpuidle_register_driver(&tegra_idle);

	if (ret)
		return ret;

	for_each_possible_cpu(cpu) {
		if (tegra_idle_enter(cpu))
			pr_err("error initializing idle loop for processor %u\n", cpu);
	}
	return 0;
}

static void __exit tegra_cpuidle_exit(void)
{
	cpuidle_unregister_driver(&tegra_idle);
}


module_init(tegra_cpuidle_init);
module_exit(tegra_cpuidle_exit);
