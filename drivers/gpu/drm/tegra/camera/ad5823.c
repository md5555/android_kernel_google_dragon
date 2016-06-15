/*
 * Copyright (c) 2013-2015, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include <uapi/linux/tegra/ad5823.h>

/* address */
#define AD5823_RESET			0x1
#define AD5823_MODE			0x2
#define AD5823_VCM_MOVE_TIME		0x3
#define AD5823_VCM_CODE_MSB		0x4
#define AD5823_VCM_CODE_LSB		0x5
#define AD5823_VCM_THRESHOLD_MSB	0x6
#define AD5823_VCM_THRESHOLD_LSB	0x7
#define AD5823_RING_CTRL		(1 << 2)

#define AD5823_ACTUATOR_RANGE		1023
#define AD5823_POS_LOW_DEFAULT		0
#define AD5823_POS_HIGH_DEFAULT		1023
#define AD5823_FOCUS_MACRO		568
#define AD5823_FOCUS_INFINITY		146

#define SETTLETIME_MS			15
/* define FOCAL_LENGTH/MAX_APERTURE/FNUMBER as integer with granularity of
 * 10000 instead of previous float type -- bug 1519258
 */
#define FOCAL_LENGTH			29500
/* in APEX unit = 2 * log2(fnumber) */
#define MAX_APERTURE			25261
#define FNUMBER				24000
#define AD5823_MOVE_TIME_VALUE		0x43

#define NV_FOCUSER_SET_MAX              10
#define NV_FOCUSER_SET_DISTANCE_PAIR    16

struct nv_focuser_set_dist_pairs {
	__s32 fdn;
	__s32 distance;
} __packed;

struct nv_focuser_set {
	__s32 posture;
	__s32 macro;
	__s32 hyper;
	__s32 inf;
	__s32 hysteresis;
	__u32 settle_time;
	__s32 macro_offset;
	__s32 inf_offset;
	__u32 num_dist_pairs;
	struct nv_focuser_set_dist_pairs
			dist_pair[NV_FOCUSER_SET_DISTANCE_PAIR];
} __packed;

struct nv_focuser_config {
	__u32 focal_length;
	__u32 fnumber;
	__u32 max_aperture;
	__u32 range_ends_reversed;
	__s32 pos_working_low;
	__s32 pos_working_high;
	__u32 pos_actual_low;
	__u32 pos_actual_high;
	__u32 slew_rate;
	__u32 circle_of_confusion;
	__u32 num_focuser_sets;
	struct nv_focuser_set focuser_set[NV_FOCUSER_SET_MAX];
} __packed;

struct ad5823_config {
	__u32 settle_time;
	__u32 actuator_range;
	__u32 pos_low;
	__u32 pos_high;
	float focal_length;
	float fnumber;
	float max_aperture;
};

struct ad5823_cal_data {
	__u32 pos_low;
	__u32 pos_high;
};

struct ad5823_info {
	struct i2c_client *i2c_client;
	struct regulator *regulator;
	struct nv_focuser_config config;
	struct miscdevice miscdev;
	struct regmap *regmap;
	int shutdown_gpio;
	u32 active_features;
	u32 supported_features;
	atomic_t in_use;
};

static int ad5823_set_position(struct ad5823_info *info, u32 position)
{
	int ret = 0;
	u32 position_clamped = 0;
	int retry = 5;

	position_clamped = clamp(position, info->config.pos_actual_low,
				info->config.pos_actual_high);

	if (position != position_clamped) {
		dev_err(&info->i2c_client->dev,
			"%s: position(%d) out of bound([%d, %d])\n",
			__func__, position, info->config.pos_actual_low,
			info->config.pos_actual_high);
	}

	while (retry) {
		ret = regmap_write(info->regmap, AD5823_VCM_MOVE_TIME,
					AD5823_MOVE_TIME_VALUE);
		if (!ret)
			break;

		dev_err(&info->i2c_client->dev,
			"regmap write failed, retry countdown = %d.\n", retry);

		msleep(1);
		retry--;
	}
	if (!retry)
		return ret;

	ret |= regmap_write(info->regmap, AD5823_MODE, 0);
	ret |= regmap_write(info->regmap, AD5823_VCM_CODE_MSB,
		((position >> 8) & 0x3) | (1 << 2));
	ret |= regmap_write(info->regmap, AD5823_VCM_CODE_LSB,
		position & 0xFF);

	return ret;
}

static long ad5823_ioctl(struct file *file,
			unsigned int cmd, unsigned long arg)
{
	struct ad5823_info *info = file->private_data;
	struct ad5823_cal_data cal;
	int err;

	switch (_IOC_NR(cmd)) {
	case _IOC_NR(AD5823_IOCTL_GET_CONFIG):
		if (copy_to_user((void __user *) arg,
				 &info->config,
				 sizeof(info->config)))
			return -EFAULT;

	case _IOC_NR(AD5823_IOCTL_SET_FEATURES):
		info->active_features = arg;
		break;

	case _IOC_NR(AD5823_IOCTL_GET_FEATURES):
		if (copy_to_user((void __user *) arg,
				 &info->supported_features,
				 sizeof(info->supported_features)))
			return -EFAULT;

	case _IOC_NR(AD5823_IOCTL_SET_POSITION):
		err = ad5823_set_position(info, (u32) arg);
		return err;

	case _IOC_NR(AD5823_IOCTL_SET_CAL_DATA):
		if (copy_from_user(&cal, (const void __user *)arg,
					sizeof(cal))) {
			dev_err(&info->i2c_client->dev,
				"%s:Failed to get mode from user.\n",
				__func__);
			return -EFAULT;
		}
		info->config.pos_working_low = cal.pos_low;
		info->config.pos_working_high = cal.pos_high;
		break;

	case _IOC_NR(AD5823_IOCTL_SET_CONFIG):

		cal.pos_low = 0;
		cal.pos_high = 0;
		if (info->config.pos_working_low != 0)
			cal.pos_low = info->config.pos_working_low;
		if (info->config.pos_working_high != 0)
			cal.pos_high = info->config.pos_working_high;

		if (copy_from_user(&info->config, (const void __user *)arg,
			sizeof(info->config))) {
			dev_err(&info->i2c_client->dev,
				"%s:Failed to get config from user.\n",
				__func__);
			return -EFAULT;
		}

		if (cal.pos_low != 0) {
			info->config.pos_working_low = cal.pos_low;
			info->config.focuser_set[0].inf = cal.pos_low;
		}
		if (cal.pos_high != 0) {
			info->config.pos_working_high = cal.pos_high;
			info->config.focuser_set[0].macro = cal.pos_high;
		}
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int ad5823_power_on(struct ad5823_info *info)
{
	int ret;

	ret = regulator_enable(info->regulator);
	if (ret)
		return ret;

	/* Wait for voltages to stabilize. */
	usleep_range(100, 200);

	gpio_set_value(info->shutdown_gpio, 1);

	/* Wait for the focuser to power up (typ. 100 uS). */
	usleep_range(200, 300);

	return 0;
}

static int ad5823_power_off(struct ad5823_info *info)
{
	gpio_set_value(info->shutdown_gpio, 0);

	return regulator_disable(info->regulator);
}

static int ad5823_open(struct inode *inode, struct file *file)
{
	int err = 0;
	struct miscdevice *miscdev = file->private_data;
	struct ad5823_info *info = dev_get_drvdata(miscdev->parent);

	if (atomic_xchg(&info->in_use, 1))
		return -EBUSY;

	err = ad5823_power_on(info);
	file->private_data = info;

	return err;
}

static int ad5823_release(struct inode *inode, struct file *file)
{
	struct ad5823_info *info = file->private_data;
	int ret = ad5823_power_off(info);

	atomic_xchg(&info->in_use, 0);

	return ret;
}

static const struct file_operations ad5823_fileops = {
	.owner = THIS_MODULE,
	.open = ad5823_open,
	.unlocked_ioctl = ad5823_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = ad5823_ioctl,
#endif
	.release = ad5823_release,
};

static const struct miscdevice ad5823_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "focuser",
	.fops = &ad5823_fileops,
};

static struct of_device_id ad5823_of_match[] = {
	{ .compatible = "adi,ad5823", },
	{ },
};

MODULE_DEVICE_TABLE(of, ad5823_of_match);

static int ad5823_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int err;
	struct ad5823_info *info;
	struct device_node *np = client->dev.of_node;
	static struct regmap_config ad5823_regmap_config = {
		.reg_bits = 8,
		.val_bits = 8,
	};

	dev_info(&client->dev, "[AD5823]: probing focuser.\n");

	info = devm_kzalloc(&client->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	if (!np)
		return -EFAULT;

	atomic_set(&info->in_use, 0);

	info->shutdown_gpio = of_get_named_gpio(np, "shutdown-gpio", 0);
	err = devm_gpio_request_one(&client->dev, info->shutdown_gpio,
					GPIOF_OUT_INIT_LOW, "cam-af");
	if (err) {
		dev_err(&client->dev, "[AD5823]: unable to request shutdown-gpio\n");
		return err;
	}

	info->regulator = devm_regulator_get(&client->dev, "vdd");
	if (IS_ERR(info->regulator)) {
		dev_err(&client->dev, "%s vdd ERR: %p\n",
			__func__, info->regulator);
		return PTR_ERR(info->regulator);
	}

	info->regmap = devm_regmap_init_i2c(client, &ad5823_regmap_config);
	if (IS_ERR(info->regmap)) {
		dev_err(&client->dev,
			"Failed to allocate register map: %d\n", err);
		return PTR_ERR(info->regmap);
	}

	info->i2c_client = client;
	info->miscdev = ad5823_device;
	info->miscdev.parent = &client->dev;

	info->config.focal_length = FOCAL_LENGTH;
	info->config.fnumber = FNUMBER;
	info->config.max_aperture = MAX_APERTURE;
	info->config.range_ends_reversed = 0;

	info->config.pos_working_low = AD5823_FOCUS_INFINITY;
	info->config.pos_working_high = AD5823_FOCUS_MACRO;
	info->config.pos_actual_low = AD5823_POS_LOW_DEFAULT;
	info->config.pos_actual_high = AD5823_POS_HIGH_DEFAULT;

	info->config.num_focuser_sets = 1;
	info->config.focuser_set[0].macro = AD5823_FOCUS_MACRO;
	info->config.focuser_set[0].hyper = AD5823_FOCUS_INFINITY;
	info->config.focuser_set[0].inf = AD5823_FOCUS_INFINITY;
	info->config.focuser_set[0].settle_time = SETTLETIME_MS;

	info->active_features = 0;
	info->supported_features = 0;

	i2c_set_clientdata(client, info);

	err = misc_register(&info->miscdev);
	if (err) {
		dev_err(&client->dev, "Unable to register misc device!\n");
		return err;
	}

	return err;
}

static int ad5823_remove(struct i2c_client *client)
{
	struct ad5823_info *info = i2c_get_clientdata(client);

	misc_deregister(&info->miscdev);
	return 0;
}

static const struct i2c_device_id ad5823_id[] = {
	{ "ad5823", 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, ad5823_id);

static struct i2c_driver ad5823_i2c_driver = {
	.driver = {
		.name = "ad5823",
		.owner = THIS_MODULE,
		.of_match_table = ad5823_of_match,
	},
	.probe = ad5823_probe,
	.remove = ad5823_remove,
	.id_table = ad5823_id,
};

module_i2c_driver(ad5823_i2c_driver);
MODULE_LICENSE("GPL v2");
