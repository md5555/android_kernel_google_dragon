/*
 * ARM/ARM64 generic CPU idle driver.
 *
 * Copyright (C) 2014 ARM Ltd.
 * Author: Lorenzo Pieralisi <lorenzo.pieralisi@arm.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt) "CPUidle arm: " fmt

#include <linux/cpuidle.h>
#include <linux/cpumask.h>
#include <linux/cpu_pm.h>
#include <linux/of_device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/slab.h>

#include <asm/cpuidle.h>

#include "dt_idle_states.h"

/*
 * arm_enter_idle_state - Programs CPU to enter the specified state
 *
 * dev: cpuidle device
 * drv: cpuidle driver
 * idx: state index
 *
 * Called from the CPUidle framework to program the device to the
 * specified target state selected by the governor.
 */
static int arm_enter_idle_state(struct cpuidle_device *dev,
				struct cpuidle_driver *drv, int idx)
{
	int ret;

	if (!idx) {
		cpu_do_idle();
		return idx;
	}

	ret = cpu_pm_enter();
	if (!ret) {
		/*
		 * Pass idle state index to cpu_suspend which in turn will
		 * call the CPU ops suspend protocol with idle index as a
		 * parameter.
		 */
		arm_cpuidle_suspend(idx);

		cpu_pm_exit();
	}

	return ret ? -1 : idx;
}

#define ARM_CPUIDLE_MAX_DRIVERS	2

static struct cpuidle_driver arm_idle_drivers[ARM_CPUIDLE_MAX_DRIVERS] = {
	[0 ... ARM_CPUIDLE_MAX_DRIVERS - 1] = {
		.name = "arm_idle",
		.owner = THIS_MODULE,
		/*
		 * State at index 0 is standby wfi and considered standard
		 * on all ARM platforms. If in some platforms simple wfi
		 * can't be used as "state 0", DT bindings must be implemented
		 * to work around this issue and allow installing a special
		 * handler for idle state index 0.
		 */
		.states[0] = {
			.enter                  = arm_enter_idle_state,
			.exit_latency           = 1,
			.target_residency       = 1,
			.power_usage		= UINT_MAX,
			.name                   = "WFI",
			.desc                   = "ARM WFI",
		}
	}
};

static const struct of_device_id arm_idle_state_match[] __initconst = {
	{ .compatible = "arm,idle-state",
	  .data = arm_enter_idle_state },
	{ },
};

/*
 * Compare idle states phandle properties
 *
 * Return true if properties are valid and equal, false otherwise
 */
static bool __init idle_states_cmp(struct property *states1,
				   struct property *states2)
{
	/*
	 * NB: Implemented through code from drivers/of/unittest.c
	 *     Function is generic and can be moved to generic OF code
	 *     if needed
	 */
	return states1 && states2 &&
	       (states1->length == states2->length) &&
	       states1->value && states2->value &&
	       !memcmp(states1->value, states2->value, states1->length);
}

static int __init arm_idle_init_driver(struct cpuidle_driver *drv)
{
	int ret, cpu;
	struct cpuidle_device *dev;
	struct property *curr_idle_states, *idle_states = NULL;
	struct device_node *cpu_node;

	for_each_cpu(cpu, drv->cpumask) {
		cpu_node = of_cpu_device_node_get(cpu);
		curr_idle_states = of_find_property(cpu_node,
						    "cpu-idle-states", NULL);
		of_node_put(cpu_node);

		/*
		 * Stash the first valid idle states phandle in the cpumask.
		 * If curr_idle_states is NULL assigning it to idle_states
		 * is harmless and it is managed by idle states comparison
		 * code. Keep track of first valid phandle so that
		 * subsequent cpus can compare against it.
		 */
		if (!idle_states)
			idle_states = curr_idle_states;

		/*
		 * If idle states phandles are not equal, remove the
		 * cpu from the driver mask since a CPUidle driver
		 * is only capable of managing uniform idle states.
		 *
		 * Comparison works also when idle_states and
		 * curr_idle_states are the same property, since
		 * they can be == NULL so the cpu must be removed from
		 * the driver mask in that case too (ie cpu has no idle
		 * states).
		 */
		if (!idle_states_cmp(idle_states, curr_idle_states))
			cpumask_clear_cpu(cpu, drv->cpumask);
	}

	/*
	 *  If there are no valid states for this driver we rely on arch
	 *  default idle behaviour, bail out
	 */
	if (!idle_states)
		return -ENODEV;

	/*
	 * Initialize idle states data, starting at index 1.
	 * This driver is DT only, if no DT idle states are detected (ret == 0)
	 * let the driver initialization fail accordingly since there is no
	 * reason to initialize the idle driver if only wfi is supported.
	 */
	ret = dt_init_idle_driver(drv, arm_idle_state_match, 1);
	if (ret <= 0)
		return ret ? : -ENODEV;

	ret = cpuidle_register_driver(drv);
	if (ret) {
		pr_err("Failed to register cpuidle driver\n");
		return ret;
	}

	/*
	 * Call arch CPU operations in order to initialize
	 * idle states suspend back-end specific data
	 */
	for_each_cpu(cpu, drv->cpumask) {
		ret = arm_cpuidle_init(cpu);

		/*
		 * Skip the cpuidle device initialization if the reported
		 * failure is a HW misconfiguration/breakage (-ENXIO).
		 */
		if (ret == -ENXIO)
			continue;

		if (ret) {
			pr_err("CPU %d failed to init idle CPU ops\n", cpu);
			goto out_fail;
		}

		dev = kzalloc(sizeof(*dev), GFP_KERNEL);
		if (!dev) {
			pr_err("Failed to allocate cpuidle device\n");
			goto out_fail;
		}
		dev->cpu = cpu;

		ret = cpuidle_register_device(dev);
		if (ret) {
			pr_err("Failed to register cpuidle device for CPU %d\n",
			       cpu);
			kfree(dev);
			goto out_fail;
		}
	}

	return 0;
out_fail:
	while (--cpu >= 0) {
		dev = per_cpu(cpuidle_devices, cpu);
		cpuidle_unregister_device(dev);
		kfree(dev);
	}

	cpuidle_unregister_driver(drv);
	return ret;
}

/*
 * arm_idle_init
 *
 * Registers the arm specific cpuidle driver(s) with the cpuidle
 * framework. It relies on core code to parse the idle states
 * and initialize them using driver data structures accordingly.
 */
static int __init arm_idle_init(void)
{
	int i, ret = -ENODEV;
	struct cpuidle_driver *drv;
	cpumask_var_t tmpmask;

	/*
	 * These drivers require DT idle states to be present.
	 * If no idle states are detected there is no reason to
	 * proceed any further hence we return early.
	 */
	if (!of_find_node_by_name(NULL, "idle-states"))
		return -ENODEV;

	if (!alloc_cpumask_var(&tmpmask, GFP_KERNEL))
		return -ENOMEM;

	/*
	 * We need to vet idle states to create CPUidle drivers
	 * that share a common set of them. Create a tmp mask
	 * that we use to keep track of initialized cpus.
	 * Start off by initializing the mask with all possible
	 * cpus, we clear it as we go, till either all cpus
	 * have a CPUidle driver initialized or there are some
	 * CPUs that have no idle states or a parsing error
	 * occurs.
	 */
	cpumask_copy(tmpmask, cpu_possible_mask);

	for (i = 0; !cpumask_empty(tmpmask); i++) {
		if (i == ARM_CPUIDLE_MAX_DRIVERS) {
			pr_warn("number of drivers exceeding static allocation\n");
			break;
		}

		drv = &arm_idle_drivers[i];
		drv->cpumask = kzalloc(cpumask_size(), GFP_KERNEL);
		if (!drv->cpumask) {
			ret = -ENOMEM;
			break;
		}
		/*
		 * Force driver mask, arm_idle_init_driver()
		 * will tweak it by vetting idle states.
		 */
		cpumask_copy(drv->cpumask, tmpmask);

		ret = arm_idle_init_driver(drv);
		if (ret) {
			kfree(drv->cpumask);
			break;
		}
		/*
		 * Remove the cpus that were part of the registered
		 * driver from the mask of cpus to be initialized
		 * and restart.
		 */
		cpumask_andnot(tmpmask, tmpmask, drv->cpumask);
	}

	free_cpumask_var(tmpmask);
	return ret;
}
device_initcall(arm_idle_init);
