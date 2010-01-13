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

int tegra_state_valid(suspend_state_t state)
{
	printk("%s CALLED\n", __func__);
	if (state > PM_SUSPEND_ON && state <= PM_SUSPEND_MAX)
		return 1;
	return 0;
}

int tegra_state_begin(suspend_state_t state)
{
	printk("%s CALLED with state = %d\n", __func__, state);
	return 0;
}

int tegra_state_prepare(void)
{
	printk("%s CALLED \n", __func__);
	return 0;
}

int tegra_state_enter(suspend_state_t state)
{
	printk("%s CALLED with state = %d\n", __func__, state);
	return 0;
}

void tegra_state_finish(void)
{
	printk("%s CALLED \n", __func__);
}

void tegra_state_end(void)
{
	printk("%s CALLED \n", __func__);
}

void tegra_state_recover(void)
{
	printk("%s CALLED \n", __func__);
}

static struct platform_suspend_ops tegra_suspend_ops =
{
	.valid   = tegra_state_valid,
	.begin   = tegra_state_begin,
	.prepare = tegra_state_prepare,
	.enter   = tegra_state_enter,
	.finish  = tegra_state_finish,
	.end     = tegra_state_end,
	.recover = tegra_state_recover
};

void tegra_set_suspend_ops()
{
	suspend_set_ops(&tegra_suspend_ops);
}
