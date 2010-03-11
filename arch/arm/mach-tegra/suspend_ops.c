/*
 * arch/arm/mach-tegra/suspend_ops.c
 *
 * Suspend Operation API implementation
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

#include <linux/suspend.h>
#include "nvcommon.h"
#include "nvodm_query.h"
#include <linux/wakelock.h>
#include <linux/cpu.h>

extern void cpu_ap20_do_lp0(void);
extern void cpu_ap20_do_lp1(void);

#if defined(CONFIG_TEGRA_ODM_HARMONY)
static struct wake_lock suspend_ops_wake_lock;
static bool wake_lock_initialized = false;
#endif

int tegra_state_valid(suspend_state_t state)
{
	if (state > PM_SUSPEND_ON && state <= PM_SUSPEND_MAX)
		return 1;
	return 0;
}

int tegra_state_enter(suspend_state_t state)
{
	const NvOdmSocPowerStateInfo *LPStateInfo;

	LPStateInfo = NvOdmQueryLowestSocPowerState();
	if (LPStateInfo->LowestPowerState == NvOdmSocPowerState_DeepSleep) {
#if defined(CONFIG_ARCH_TEGRA_2x_SOC)
		/* do AP20 LP0 */
		cpu_ap20_do_lp0();
#endif
	}
	else if (LPStateInfo->LowestPowerState == NvOdmSocPowerState_Suspend) {
#if defined(CONFIG_ARCH_TEGRA_2x_SOC)
		/* do AP20 LP1 */
		cpu_ap20_do_lp1();
#endif
	}
	return 0;
}

void tegra_state_end(void)
{
#if defined(CONFIG_TEGRA_ODM_HARMONY)
	if(!wake_lock_initialized) {
		wake_lock_init(&suspend_ops_wake_lock, WAKE_LOCK_SUSPEND, "tegra_suspend_ops");
		wake_lock_initialized = true;
	}
	wake_lock_timeout(&suspend_ops_wake_lock, 1000);
#endif
}

static struct platform_suspend_ops tegra_suspend_ops =
{
	.valid   = tegra_state_valid,
	.enter   = tegra_state_enter,
	.end     = tegra_state_end
};

void tegra_set_suspend_ops(void)
{
	suspend_set_ops(&tegra_suspend_ops);
}
