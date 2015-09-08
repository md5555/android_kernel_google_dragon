/*
 * Copyright (c) 2015, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <linux/pm_runtime.h>

int host1x_module_busy(struct device *dev)
{
	int err;

	err = pm_runtime_get_sync(dev);
	if (err < 0)
		return err;

	return 0;
}

void host1x_module_idle(struct device *dev)
{
	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);
}

void host1x_module_idle_mult(struct device *dev, int refs)
{
	while (refs--)
		host1x_module_idle(dev);
}
