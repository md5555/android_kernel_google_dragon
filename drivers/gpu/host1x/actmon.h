/*
 * Copyright (c) 2015, NVIDIA Corporation
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

#ifndef HOST1X_ACTMON_H
#define HOST1X_ACTMON_H

#include <linux/host1x.h>

struct host1x_actmon;

enum host1x_actmon_state {
	ACTMON_OFF = 0,
	ACTMON_READY = 1,
	ACTMON_SLEEP = 2
};

struct host1x_actmon_worker {
	int type;
	struct host1x_actmon *actmon;
	struct work_struct work;
};

struct host1x_actmon {
	enum host1x_actmon_state state;

	struct host1x *host;
	void __iomem *regs;
	struct clk *clk_actmon;

	/* Store actmon period. clks_per_sample can be used even when host1x is
	 * not active. */
	long usecs_per_sample;
	long clks_per_sample;

	int k;
	int divider;
	struct device *dev;
	struct host1x_actmon_worker above_wmark_worker;
	struct host1x_actmon_worker below_wmark_worker;
};

void host1x_actmon_notify_busy(struct host1x_client *client);
void host1x_actmon_notify_idle(struct host1x_client *client);

void host1x_actmon_irq(struct device *dev, int type);

#endif
