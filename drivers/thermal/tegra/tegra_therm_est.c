/*
 * Copyright (c) 2010-2013, NVIDIA CORPORATION.  All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/debugfs.h>
#include <linux/module.h>
#include <linux/hwmon-sysfs.h>
#include <linux/suspend.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/workqueue.h>
#include <linux/thermal.h>
#include <linux/platform_device.h>

#include "../thermal_core.h"

#define HIST_LEN (20)
#define DEFAULT_TSKIN (25000) /* default tskin in mC */

struct thermal_trip_info {
	long trip_temp;
	enum thermal_trip_type trip_type;
	unsigned long upper;
	unsigned long lower;
	long hysteresis;
	bool tripped;
	bool bound;
	char *cdev_type;
};

struct therm_est_subdevice {
	struct thermal_zone_device *th;
	s32 coeffs[HIST_LEN];
	long hist[HIST_LEN];
};

struct therm_estimator {
	struct thermal_zone_device *thz;
	int num_trips;
	struct thermal_trip_info *trips;
	struct thermal_zone_params *tzp;

	struct workqueue_struct *workqueue;
	struct delayed_work therm_est_work;
	long cur_temp;
	long low_limit;
	long high_limit;
	int ntemp;
	u32 toffset;
	u32 polling_period;
	int ndevs;
	struct therm_est_subdevice *devs;

#ifdef CONFIG_PM
	struct notifier_block pm_nb;
#endif
};

static int therm_est_update_tripped_state(struct therm_estimator *est,
					  int trip, unsigned long *temp)
{
	struct thermal_trip_info *trip_state = &est->trips[trip];
	unsigned long trip_temp, zone_temp, hyst;
	struct thermal_instance *instance;

	est->thz->ops->get_trip_temp(est->thz, trip, &trip_temp);
	zone_temp = est->thz->temperature;
	est->thz->ops->get_trip_hyst(est->thz, trip, &hyst);

	/*
	 * Check the instance has been created, if so update the
	 * trip_temp and trip_state, and break to avoid going through
	 * the rest of the list.
	 */
	list_for_each_entry(instance, &est->thz->thermal_instances, tz_node) {
		if (instance->trip != trip)
			continue;
		if (zone_temp >= trip_temp) {
			trip_temp -= hyst;
			trip_state->tripped = true;
		} else if (trip_state->tripped) {
			trip_temp -= hyst;
			if (zone_temp < trip_temp)
				trip_state->tripped = false;
		}

		break;
	}

	*temp = trip_temp;

	return 0;
}

static void therm_est_update_limits(struct therm_estimator *est)
{
	const int MAX_HIGH_TEMP = 128000;
	long low_temp = 0, high_temp = MAX_HIGH_TEMP;
	long trip_temp, passive_low_temp = MAX_HIGH_TEMP;
	enum thermal_trip_type trip_type;
	struct thermal_trip_info *trip_state;
	int i;

	for (i = 0; i < est->thz->trips; i++) {
		trip_state = &est->trips[i];
		therm_est_update_tripped_state(est, i, &trip_temp);
		est->thz->ops->get_trip_type(est->thz, i, &trip_type);

		if (!trip_state->tripped) { /* not tripped? update high */
			if (trip_temp < high_temp)
				high_temp = trip_temp;
		} else { /* tripped? update low */
			if (trip_type != THERMAL_TRIP_PASSIVE) {
				/* get highest ACTIVE */
				if (trip_temp > low_temp)
					low_temp = trip_temp;
			} else {
				/* get lowest PASSIVE */
				if (trip_temp < passive_low_temp)
					passive_low_temp = trip_temp;
			}
		}
	}

	if (passive_low_temp != MAX_HIGH_TEMP)
		low_temp = max(low_temp, passive_low_temp);

	est->low_limit = low_temp;
	est->high_limit = high_temp;
}

static void therm_est_work_func(struct work_struct *work)
{
	int i, j, sum = 0;
	struct delayed_work *dwork = container_of(work,
					struct delayed_work, work);
	struct therm_estimator *est = container_of(dwork,
					struct therm_estimator,
					therm_est_work);

	for (i = 0; i < est->ndevs; i++) {
		unsigned long temp;
		struct thermal_zone_device *thz = est->devs[i].th;

		if (!thz || thermal_zone_get_temp(thz, &temp))
			temp = DEFAULT_TSKIN;

		est->devs[i].hist[est->ntemp] = temp;
	}

	for (i = 0; i < est->ndevs; i++)
		for (j = 0; j < HIST_LEN; j++) {
			int index;

			index = (est->ntemp - j + HIST_LEN) % HIST_LEN;
			sum += est->devs[i].hist[index] *
				est->devs[i].coeffs[j];
		}

	est->cur_temp = sum / 100 + est->toffset;

	est->ntemp++;
	est->ntemp = est->ntemp % HIST_LEN;

	if (est->thz &&
	    ((est->cur_temp < est->low_limit) ||
	    (est->cur_temp >= est->high_limit))) {
		thermal_zone_device_update(est->thz);
		therm_est_update_limits(est);
	}

	queue_delayed_work(est->workqueue, &est->therm_est_work,
				msecs_to_jiffies(est->polling_period));
}

static int therm_est_get_temp(void *dev, long *temp)
{
	struct therm_estimator *est = dev;

	*temp = est->cur_temp;
	return 0;
}

static int therm_est_get_trend(void *dev, long *trend)
{
	struct therm_estimator *est = dev;
	struct thermal_zone_device *thz = est->thz;

	if (thz->temperature > thz->last_temperature + 100)
		*trend = THERMAL_TREND_RAISING;
	else if (thz->temperature < thz->last_temperature - 100)
		*trend = THERMAL_TREND_DROPPING;
	else
		*trend = THERMAL_TREND_STABLE;

	return 0;
}

#ifdef CONFIG_DEBUG_FS
static int show_coeff(struct seq_file *s, void *data)
{
	struct device *dev = s->private;
	struct therm_estimator *est = dev_get_drvdata(dev);
	int i, j;

	for (i = 0; i < est->ndevs; i++) {
		seq_printf(s, "[%d]", i);
		for (j = 0; j < HIST_LEN; j++)
			seq_printf(s, " %d", est->devs[i].coeffs[j]);
		seq_puts(s, "\n");
	}
	return 0;
}

static ssize_t set_coeff(struct file *f, const char __user *buf,
			 size_t sz, loff_t *off)
{
	struct device *dev = f->f_inode->i_private;
	struct therm_estimator *est = dev_get_drvdata(dev);
	int devid, scount;
	s32 coeff[20];

	if (HIST_LEN > 20)
		return -EINVAL;

	scount = sscanf(buf,
			"[%d] %d %d %d %d %d %d %d %d %d %d "
			"%d %d %d %d %d %d %d %d %d %d",
			&devid,
			&coeff[0],
			&coeff[1],
			&coeff[2],
			&coeff[3],
			&coeff[4],
			&coeff[5],
			&coeff[6],
			&coeff[7],
			&coeff[8],
			&coeff[9],
			&coeff[10],
			&coeff[11],
			&coeff[12],
			&coeff[13],
			&coeff[14],
			&coeff[15],
			&coeff[16],
			&coeff[17],
			&coeff[18],
			&coeff[19]);

	if (scount != HIST_LEN + 1)
		return -EINVAL;

	if (devid < 0 || devid >= est->ndevs)
		return -EINVAL;

	/* This has obvious locking issues but don't worry about it */
	memcpy(est->devs[devid].coeffs, coeff, sizeof(coeff[0]) * HIST_LEN);

	return sz;
}

static int coeff_open(struct inode *inode, struct file *file)
{
	return single_open(file, show_coeff, inode->i_private);
}

static const struct file_operations coeffs_fops = {
	.open = coeff_open,
	.read = seq_read,
	.write = set_coeff,
	.release = single_release,
};

static int show_temps(struct seq_file *s, void *data)
{
	struct device *dev = s->private;
	struct therm_estimator *est = dev_get_drvdata(dev);
	int i, j;

	/* This has obvious locking issues but don't worry about it */
	for (i = 0; i < est->ndevs; i++) {
		seq_printf(s, "[%d]", i);
		for (j = 0; j < HIST_LEN; j++) {
			int index;

			index = (est->ntemp - j + HIST_LEN) % HIST_LEN;
			seq_printf(s, " %ld", est->devs[i].hist[index]);
		}
		seq_puts(s, "\n");
	}
	return 0;
}

static int temps_open(struct inode *inode, struct file *file)
{
	return single_open(file, show_temps, inode->i_private);
}

static const struct file_operations temps_fops = {
	.open = temps_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
#endif /* CONFIG_DEBUG_FS */

static int therm_est_init_history(struct therm_estimator *est)
{
	int i;

	for (i = 0; i < est->ndevs; i++) {
		struct thermal_zone_device *thz;
		unsigned long temp;
		int j;

		thz = est->devs[i].th;

		if (!thz || thermal_zone_get_temp(thz, &temp))
			temp = DEFAULT_TSKIN;

		for (j = 0; j < HIST_LEN; j++)
			est->devs[i].hist[j] = temp;
	}

	return 0;
}

#ifdef CONFIG_PM
static int therm_est_pm_notify(struct notifier_block *nb,
				unsigned long event, void *data)
{
	struct therm_estimator *est = container_of(
					nb,
					struct therm_estimator,
					pm_nb);

	switch (event) {
	case PM_SUSPEND_PREPARE:
		cancel_delayed_work_sync(&est->therm_est_work);
		break;
	case PM_POST_SUSPEND:
		est->low_limit = 0;
		est->high_limit = 0;
		therm_est_init_history(est);
		queue_delayed_work(est->workqueue,
				&est->therm_est_work,
				msecs_to_jiffies(est->polling_period));
		break;
	default:
		return NOTIFY_DONE;
	}

	return NOTIFY_OK;
}
#endif

static int __parse_dt_subdev(struct device *dev,
			     struct therm_estimator *est)
{
	struct device_node *np = dev->of_node;
	struct device_node *ch, *gch;
	int ndevs, i;

	ch = of_get_child_by_name(np, "sub-devs");
	if (!ch)
		return -ENODATA;

	ndevs = of_get_child_count(ch);
	if (ndevs == 0)
		return -ENODATA;

	est->ndevs = ndevs;

	if (!est->devs) {
		est->devs = devm_kzalloc(dev, sizeof(*est->devs) * ndevs,
					 GFP_KERNEL);
		if (!est->devs)
			return -ENOMEM;
	}

	i = 0;
	for_each_child_of_node(ch, gch) {
		struct device_node *subdev_node;
		struct thermal_zone_device *th;
		int ret;

		subdev_node = of_parse_phandle(gch, "dev", 0);
		if (IS_ERR(subdev_node))
			return -ENODATA;
		th = thermal_zone_get_zone_by_node(subdev_node);
		if (IS_ERR(th))
			return -EPROBE_DEFER;

		ret = of_property_count_u32_elems(gch, "coeffs");
		if (ret != HIST_LEN)
			return -EINVAL;
		of_property_read_u32_array(gch, "coeffs",
					   (u32 *)&est->devs[i].coeffs, ret);

		est->devs[i].th = th;
		i++;
	}

	return 0;
}

static struct therm_estimator *therm_est_get_pdata(struct device *dev)
{
	struct therm_estimator *est;
	struct device_node *np = dev->of_node;
	u32 val;
	int ret;

	est = devm_kzalloc(dev, sizeof(*est), GFP_KERNEL);
	if (!est)
		return ERR_PTR(-ENOMEM);

	ret = of_property_read_u32(np, "toffset", &val);
	if (ret < 0)
		return ERR_PTR(ret);
	est->toffset = val;

	ret = of_property_read_u32(np, "polling-period", &val);
	if (ret < 0)
		return ERR_PTR(ret);
	est->polling_period = val;

	ret = __parse_dt_subdev(dev, est);
	if (ret < 0)
		goto error;

	return est;

error:
	devm_kfree(dev, est);
	return ERR_PTR(ret);
}

static const struct thermal_zone_of_device_ops therm_est_ops = {
	.get_temp = therm_est_get_temp,
	.get_trend = therm_est_get_trend,
};

static int therm_est_probe(struct platform_device *pdev)
{
	struct therm_estimator *est;
	int ret;
#ifdef CONFIG_DEBUG_FS
	struct dentry *parent;
#endif

	est = therm_est_get_pdata(&pdev->dev);
	if (IS_ERR(est))
		return PTR_ERR(est);

	platform_set_drvdata(pdev, est);

	/* initialize history */
	therm_est_init_history(est);

	est->thz = thermal_zone_of_sensor_register(&pdev->dev,
						   0,
						   est,
						   &therm_est_ops);
	if (IS_ERR(est->thz))
		return PTR_ERR(est->thz);

	est->num_trips = est->thz->trips;

	est->trips = devm_kzalloc(&pdev->dev,
				  sizeof(*est->trips) * est->num_trips,
				  GFP_KERNEL);
	if (!(est->trips)) {
		ret = -ENOMEM;
		goto err;
	}

	est->workqueue = alloc_workqueue(dev_name(&pdev->dev),
				    WQ_HIGHPRI | WQ_UNBOUND, 1);
	if (!(est->workqueue)) {
		ret = -EINVAL;
		goto err;
	}

	INIT_DELAYED_WORK(&est->therm_est_work, therm_est_work_func);

	queue_delayed_work(est->workqueue,
				&est->therm_est_work,
				msecs_to_jiffies(est->polling_period));

#ifdef CONFIG_PM
	est->pm_nb.notifier_call = therm_est_pm_notify,
	register_pm_notifier(&est->pm_nb);
#endif

#ifdef CONFIG_DEBUG_FS
	parent = debugfs_create_dir("tegra_therm_est", NULL);
	debugfs_create_file("coeffs", S_IRUSR | S_IWUSR, parent,
			    &pdev->dev, &coeffs_fops);
	debugfs_create_file("temps", S_IRUSR, parent,
			    &pdev->dev, &temps_fops);
	debugfs_create_u32("offset", S_IRUSR | S_IWUSR, parent, &est->toffset);
#endif

	return 0;

err:
	thermal_zone_of_sensor_unregister(&pdev->dev, est->thz);
	return ret;
}

static int therm_est_remove(struct platform_device *pdev)
{
	struct therm_estimator *est = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&est->therm_est_work);
	destroy_workqueue(est->workqueue);

	thermal_zone_of_sensor_unregister(&pdev->dev, est->thz);

#ifdef CONFIG_PM
	unregister_pm_notifier(&est->pm_nb);
#endif

	return 0;
}

static void therm_est_shutdown(struct platform_device *pdev)
{
	struct therm_estimator *est = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&est->therm_est_work);
}

static const struct of_device_id therm_est_match[] = {
	{ .compatible = "nvidia,tegra124-therm-est", },
	{},
};

static struct platform_driver therm_est_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name  = "therm_est",
		.of_match_table = therm_est_match,
	},
	.probe  = therm_est_probe,
	.remove = therm_est_remove,
	.shutdown = therm_est_shutdown,
};

static int __init therm_est_driver_init(void)
{
	return platform_driver_register(&therm_est_driver);
}
module_init(therm_est_driver_init);
