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

#include <linux/devfreq.h>
#include <linux/debugfs.h>
#include <linux/types.h>
#include <linux/clk.h>
#include <linux/export.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/clk/tegra.h>
#include <soc/tegra/tegra-dvfs.h>

#include <governor.h>

#include "dev.h"
#include "debug.h"
#include "actmon.h"
#include "acm.h"

static ssize_t host1x_actmon_load_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct host1x *host = dev_get_drvdata(dev->parent);
	struct host1x_client *client = dev_get_drvdata(dev);
	struct host1x_actmon_data *actmon_data = client->actmon_data;
	u32 busy_time;
	ssize_t res;

	busy_time = host->actmon_op->read_avg_norm(actmon_data->actmon);
	res = snprintf(buf, PAGE_SIZE, "%u\n", busy_time);

	return res;
}

static DEVICE_ATTR(load, S_IRUGO, host1x_actmon_load_show, NULL);

/*
 * Initialize the frequency table for the given clk that we are scaling
 * with actmon.
 */
static int host1x_actmon_make_freq_table(struct host1x_actmon_data *actmon_data)
{
	unsigned long *dvfs_freqs;
	unsigned long *freqs;
	int dvfs_num_freqs, num_freqs, err;
	unsigned long max_freq =  clk_round_rate(actmon_data->clk, UINT_MAX);
	unsigned long min_freq =  clk_round_rate(actmon_data->clk, 0);
	int k;

	err = tegra_dvfs_get_freqs(clk_get_parent(actmon_data->clk),
				   &dvfs_freqs, &dvfs_num_freqs);
	if (err)
		return -EPROBE_DEFER;

	/*
	 * Allocate max size for the frequency array.  Our final frequency
	 * array may end up being smaller, because we are going to cull
	 * duplicates, and frequencies outside of our min,max range, so we
	 * might waste a little bit of space.
	 */
	freqs = kcalloc(dvfs_num_freqs, sizeof(unsigned long), GFP_KERNEL);
	if (!freqs)
		return -ENOMEM;

	num_freqs = 0;
	for (k = 0; k < dvfs_num_freqs; k++) {
		/* Out of range. */
		if ((dvfs_freqs[k] < min_freq) || (dvfs_freqs[k] > max_freq))
			continue;

		/* Duplicate. */
		if (num_freqs && (dvfs_freqs[k] == freqs[num_freqs - 1]))
			continue;

		/* Weird, non-increasing. */
		if (num_freqs && (dvfs_freqs[k] < freqs[num_freqs - 1])) {
			dev_warn(actmon_data->actmon->dev,
				 "dvfs freqs not in increasing order: %ld\n",
				 dvfs_freqs[k]);
			continue;
		}

		freqs[num_freqs++] = dvfs_freqs[k];
	}

	if (!num_freqs)
		dev_warn(actmon_data->actmon->dev,
			 "dvfs table had no applicable frequencies!\n");

	actmon_data->devfreq_profile.freq_table = freqs;
	actmon_data->devfreq_profile.max_state = num_freqs;

	return 0;
}

/* This function scales the clock. */
static int host1x_actmon_target(struct device *dev, unsigned long *freq,
				u32 flags)
{
	struct host1x_client *client = dev_get_drvdata(dev);
	struct host1x_actmon_data *actmon_data = client->actmon_data;

	*freq = clk_round_rate(clk_get_parent(actmon_data->clk), *freq);
	if (clk_get_rate(actmon_data->clk) == *freq)
		return 0;

	/* Update actmon user's rate constraint, and update aggregate rates */
	actmon_data->user.constraint[HOST1X_CLOCK_INDEX_CBUS].rate = *freq;
	host1x_module_update_rate(client, HOST1X_CLOCK_INDEX_CBUS);
	*freq = clk_get_rate(actmon_data->clk);

	return 0;
}

/*
 * Update load estimate using hardware actmon. The actmon value is normalized
 * based on the time it was asked last time.
 */
static void host1x_update_load_estimate_actmon(
	struct host1x_actmon_data *actmon_data)
{
	struct host1x *host = dev_get_drvdata(actmon_data->actmon->dev->parent);
	ktime_t t;
	unsigned long dt;
	u32 busy_time;

	t = ktime_get();
	dt = ktime_us_delta(t, actmon_data->last_event_time);

	actmon_data->dev_stat.total_time = dt;
	actmon_data->last_event_time = t;
	busy_time = host->actmon_op->read_avg_norm(actmon_data->actmon);
	actmon_data->dev_stat.busy_time = (busy_time * dt) / 1000;
}

/* Queries the current device status. */
static int host1x_actmon_get_dev_status(struct device *dev,
					struct devfreq_dev_status *stat)
{
	struct host1x_client *client = dev_get_drvdata(dev);
	struct host1x_actmon_data *actmon_data = client->actmon_data;

	/* Make sure there are correct values for the current frequency */
	actmon_data->dev_stat.current_frequency =
		clk_get_rate(actmon_data->clk);

	if (actmon_data->actmon)
		host1x_update_load_estimate_actmon(actmon_data);

	/* Copy the contents of the current device status */
	*stat = actmon_data->dev_stat;

	/* Finally, clear out the local values */
	actmon_data->dev_stat.total_time = 0;
	actmon_data->dev_stat.busy_time = 0;

	return 0;
}

/* Set the low watermark threshold. */
static int host1x_actmon_set_low_wmark(struct device *dev,
				      unsigned int threshold)
{
	struct host1x *host = dev_get_drvdata(dev->parent);
	struct host1x_client *client = dev_get_drvdata(dev);
	struct host1x_actmon_data *actmon_data = client->actmon_data;

	host->actmon_op->set_low_wmark(actmon_data->actmon, threshold);

	return 0;
}

/* Set the high watermark threshold. */
static int host1x_actmon_set_high_wmark(struct device *dev,
				       unsigned int threshold)
{
	struct host1x *host = dev_get_drvdata(dev->parent);
	struct host1x_client *client = dev_get_drvdata(dev);
	struct host1x_actmon_data *actmon_data = client->actmon_data;

	host->actmon_op->set_high_wmark(actmon_data->actmon, threshold);

	return 0;
}

/*
 * Calling this function informs that the device is idling (..or busy). This
 * data is used to estimate the current load.
 */
static void host1x_actmon_notify(struct host1x_client *client, bool busy)
{
	struct host1x_actmon_data *actmon_data = client->actmon_data;
	struct devfreq *devfreq;

	if (!actmon_data)
		return;

	/* If devfreq is disabled, set the freq to max or min */
	devfreq = actmon_data->power_manager;
	if (!devfreq) {
		unsigned long freq = busy ? UINT_MAX : 0;

		host1x_actmon_target(client->dev, &freq, 0);
		return;
	}

	mutex_lock(&devfreq->lock);
	update_devfreq(devfreq);
	mutex_unlock(&devfreq->lock);
}

void host1x_actmon_notify_idle(struct host1x_client *client)
{
	host1x_actmon_notify(client, false);
}

void host1x_actmon_notify_busy(struct host1x_client *client)
{
	host1x_actmon_notify(client, true);
}

void host1x_actmon_irq(struct device *dev, int type)
{
	struct host1x_client *client = dev_get_drvdata(dev);
	struct host1x_actmon_data *actmon_data = client->actmon_data;
	struct devfreq *df = actmon_data->power_manager;

	devfreq_watermark_event(df, type);
}

/*
 * Initialize actmon for a particular host1x client, given the input
 * properties in actmon_data.  To enable/disable actmon, call
 * host1x_actmon_enable() and host1x_actmon_disable() from the host1x
 * client driver respectively.
 */
int host1x_actmon_init(struct host1x_client *client,
		       struct host1x_actmon_data *actmon_data)
{
	struct host1x *host = dev_get_drvdata(client->dev->parent);
	int err;

	if (!actmon_data)
		return -EINVAL;

	actmon_data->actmon = kzalloc(sizeof(struct host1x_actmon),
				  GFP_KERNEL);
	if (!actmon_data->actmon)
		return -ENOMEM;

	client->actmon_data = actmon_data;

	actmon_data->actmon->dev = client->dev;

	/* Create frequency table */
	err = host1x_actmon_make_freq_table(actmon_data);
	if (err || !actmon_data->devfreq_profile.max_state)
		goto err_free_actmon;

	/* Turn on actmon by enabling the actmon clock */
	err = clk_prepare_enable(host->clk_actmon);
	if (err < 0)
		goto err_free_actmon;

	err = device_create_file(client->dev, &dev_attr_load);
	if (err)
		goto err_disable_unprepare;

	/* Add user to the client, to aggregate actmon's clock constraint */
	actmon_data->user.constraint[HOST1X_CLOCK_INDEX_CBUS].type =
		HOST1X_USER_CONSTRAINT_TYPE_HZ;
	err = host1x_module_add_user(client, &actmon_data->user);
	if (err)
		goto err_remove_file;

	actmon_data->actmon->host = host;
	actmon_data->actmon->dev = client->dev;

	actmon_data->actmon->regs =
		host->actmon_op->get_actmon_regs(actmon_data->actmon);

	host->actmon_op->init(actmon_data->actmon);
	host->actmon_op->debug_init(actmon_data->actmon, actmon_data->debugfs);
	host->actmon_op->deinit(actmon_data->actmon);

	/* Set up devfreq governonr */
	actmon_data->devfreq_profile.initial_freq =
		actmon_data->devfreq_profile.freq_table[0];
	actmon_data->devfreq_profile.target = host1x_actmon_target;
	actmon_data->devfreq_profile.get_dev_status =
		host1x_actmon_get_dev_status;
	actmon_data->devfreq_profile.set_low_wmark =
		host1x_actmon_set_low_wmark;
	actmon_data->devfreq_profile.set_high_wmark =
		host1x_actmon_set_high_wmark;

	actmon_data->power_manager = devfreq_add_device(client->dev,
				&actmon_data->devfreq_profile,
				actmon_data->devfreq_governor, NULL);

	if (!actmon_data->power_manager) {
		err = -EINVAL;
		goto err_remove_user;
	}

	return 0;

err_remove_user:
	host1x_module_remove_user(client, &actmon_data->user);
err_remove_file:
	device_remove_file(client->dev, &dev_attr_load);
err_disable_unprepare:
	clk_disable_unprepare(host->clk_actmon);
err_free_actmon:
	kfree(actmon_data->actmon);
	client->actmon_data = NULL;
	return err;
}

/*
 * Uninitialize actmon.  host1x_actmon_enable() and host1x_actmon_disable()
 * should not be called after this.
 */
void host1x_actmon_deinit(struct host1x_client *client)
{
	struct host1x *host = dev_get_drvdata(client->dev->parent);
	struct host1x_actmon_data *actmon_data = client->actmon_data;

	if (!actmon_data)
		return;

	if (actmon_data->power_manager)
		devfreq_remove_device(actmon_data->power_manager);

	host1x_module_remove_user(client, &actmon_data->user);

	device_remove_file(client->dev, &dev_attr_load);

	clk_disable_unprepare(host->clk_actmon);

	kfree(actmon_data->devfreq_profile.freq_table);
	kfree(actmon_data->actmon);
	client->actmon_data = NULL;
}

int host1x_actmon_enable(struct host1x_client *client)
{
	struct host1x *host = dev_get_drvdata(client->dev->parent);
	struct host1x_actmon_data *actmon_data = client->actmon_data;

	if (!actmon_data)
		return 0;

	host->actmon_op->init(actmon_data->actmon);

	devfreq_resume_device(actmon_data->power_manager);

	return 0;
}

void host1x_actmon_disable(struct host1x_client *client)
{
	struct host1x *host = dev_get_drvdata(client->dev->parent);
	struct host1x_actmon_data *actmon_data = client->actmon_data;

	if (!actmon_data)
		return;

	devfreq_suspend_device(actmon_data->power_manager);

	host->actmon_op->deinit(actmon_data->actmon);
}
