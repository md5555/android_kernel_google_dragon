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
#include <linux/clk.h>
#include <linux/host1x.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <soc/tegra/tegra_emc.h>

#include "actmon.h"

int host1x_module_update_rate(struct host1x_client *client,
			      enum host1x_clock_index index)
{
	struct host1x_client_clock *clock = NULL;
	struct host1x_user *user;
	unsigned long rate = 0;

	if (index >= ARRAY_SIZE(client->clocks))
		return -EINVAL;

	clock = &client->clocks[index];
	mutex_lock(&client->user_list_lock);

	/* aggregate client constraints */
	list_for_each_entry(user, &client->user_list, node) {
		unsigned long user_rate = user->constraint[index].rate;
		int type = user->constraint[index].type;
		int pixels_per_cycle = client->clocks[index].pixels_per_cycle;

		if (type == HOST1X_USER_CONSTRAINT_TYPE_UNINITIALIZED)
			continue;

		WARN_ON(!(clock->valid_constraint_types & BIT(type)));

		switch (type) {
		case HOST1X_USER_CONSTRAINT_TYPE_HZ:
			rate = max(rate, user_rate);
			break;
		case HOST1X_USER_CONSTRAINT_TYPE_BW_KBPS:
			rate = max(rate, rate +
					 tegra_emc_bw_to_freq_req(user_rate));
			break;
		case HOST1X_USER_CONSTRAINT_TYPE_PIXELRATE:
			if (!pixels_per_cycle)
				return -EINVAL;
			rate = max(rate, rate + user_rate / pixels_per_cycle);
			break;
		default:
			WARN_ON(1);
		}
	}

	mutex_unlock(&client->user_list_lock);

	if (!rate)
		rate = clock->default_rate;

	return clk_set_rate(clock->clk, rate);
}

int host1x_module_busy(struct host1x_client *client)
{
	int err;

	err = pm_runtime_get_sync(client->dev);
	if (err < 0)
		return err;

	host1x_actmon_notify_busy(client);

	return 0;
}

void host1x_module_idle(struct host1x_client *client)
{
	host1x_actmon_notify_idle(client);

	pm_runtime_mark_last_busy(client->dev);
	pm_runtime_put_autosuspend(client->dev);
}

void host1x_module_idle_mult(struct host1x_client *client, int refs)
{
	while (refs--)
		host1x_module_idle(client);
}

int host1x_module_get_clocks(struct host1x_client *client)
{
	int k, err;

	for (k = 0; k < ARRAY_SIZE(client->clocks); k++) {
		if (!client->clocks[k].clk_name ||
		    strcmp(client->clocks[k].clk_name, "") == 0)
			continue;

		client->clocks[k].clk = devm_clk_get(client->dev,
						client->clocks[k].clk_name);
		if (!client->clocks[k].clk) {
			err = PTR_ERR(client->clocks[k].clk);
			goto error;
		}
	}

	return 0;

error:
	for (k = k - 1; k >= 0; k--)
		client->clocks[k].clk = NULL;

	return err;
}

void host1x_module_put_clocks(struct host1x_client *client)
{
	int k;

	for (k = ARRAY_SIZE(client->clocks) - 1; k >= 0; k--) {
		if (!client->clocks[k].clk)
			continue;

		client->clocks[k].clk = NULL;
	}
}

int host1x_module_enable_clocks(struct host1x_client *client)
{
	int k, err;

	for (k = 0; k < ARRAY_SIZE(client->clocks); k++) {
		if (!client->clocks[k].clk)
			continue;

		err = clk_prepare_enable(client->clocks[k].clk);
		if (err)
			goto error;
	}

	return 0;

error:
	for (k = k - 1; k >= 0; k--)
		clk_disable_unprepare(client->clocks[k].clk);

	return err;
}

void host1x_module_disable_clocks(struct host1x_client *client)
{
	int k;

	for (k = ARRAY_SIZE(client->clocks) - 1; k >= 0; k--) {
		if (!client->clocks[k].clk)
			continue;

		clk_disable_unprepare(client->clocks[k].clk);
	}
}

int host1x_module_add_user(struct host1x_client *client,
			   struct host1x_user *user)
{
	int k;

	INIT_LIST_HEAD(&user->node);

	/* This client did not advertise any clocks, so don't bother */
	if (!client->clocks)
		return 0;

	for (k = 0; k < ARRAY_SIZE(client->clocks); k++) {
		if (!client->clocks[k].clk)
			continue;

		user->constraint[k].rate = client->clocks[k].default_rate;
	}


	mutex_lock(&client->user_list_lock);
	list_add_tail(&user->node, &client->user_list);
	mutex_unlock(&client->user_list_lock);

	for (k = 0; k < ARRAY_SIZE(client->clocks); k++) {
		if (!client->clocks[k].clk_name)
			continue;

		host1x_module_update_rate(client, k);
	}

	return 0;
}

void host1x_module_remove_user(struct host1x_client *client,
			       struct host1x_user *user)
{
	int k;

	mutex_lock(&client->user_list_lock);
	list_del(&user->node);
	mutex_unlock(&client->user_list_lock);

	/* This client did not advertise any clocks, so don't bother */
	if (!client->clocks)
		return;

	for (k = 0; k < ARRAY_SIZE(client->clocks); k++) {
		if (!client->clocks[k].clk)
			continue;

		host1x_module_update_rate(client, k);
	}
}

int host1x_module_get_rate(struct host1x_client *client,
			   struct host1x_user *user,
			   enum host1x_clock_index index,
			   unsigned long *rate,
			   enum host1x_user_constraint_type type)
{
	unsigned long clk_rate;
	int err;

	if (index >= ARRAY_SIZE(client->clocks))
		return -EINVAL;

	err = host1x_module_busy(client);
	if (err)
		return err;

	clk_rate = clk_get_rate(client->clocks[index].clk);

	host1x_module_idle(client);

	switch (type) {
	case HOST1X_USER_CONSTRAINT_TYPE_HZ:
		*rate = clk_rate;
		break;
	case HOST1X_USER_CONSTRAINT_TYPE_BW_KBPS:
		*rate = tegra_emc_freq_req_to_bw(clk_rate);
		break;
	case HOST1X_USER_CONSTRAINT_TYPE_PIXELRATE:
		if (!client->clocks[index].pixels_per_cycle)
			return -EINVAL;
		*rate = clk_rate * client->clocks[index].pixels_per_cycle;
		break;
	default:
		return -EINVAL;
	}

	return err;
}

int host1x_module_set_rate(struct host1x_client *client,
			   struct host1x_user *user,
			   enum host1x_clock_index index,
			   unsigned long rate,
			   enum host1x_user_constraint_type type)
{
	int err;

	if (index >= ARRAY_SIZE(client->clocks))
		return -EINVAL;

	if (!(client->clocks[index].valid_constraint_types & BIT(type)))
		return -EINVAL;

	err = host1x_module_busy(client);
	if (err)
		return err;

	user->constraint[index].rate = rate;
	user->constraint[index].type = type;

	err = host1x_module_update_rate(client, index);

	host1x_module_idle(client);

	return err;
}
