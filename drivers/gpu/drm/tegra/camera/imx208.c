/*
 * imx208.c - imx208 sensor driver
 *
 * Copyright (c) 2014-2015, NVIDIA CORPORATION, All Rights Reserved.
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
#include <linux/clk.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/regulator/consumer.h>
#include <linux/regmap.h>
#include <uapi/linux/tegra/imx208.h>
#include <linux/gpio.h>
#include <linux/module.h>

#include <linux/kernel.h>
#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include <soc/tegra/sysedp.h>

#define IMX208_TABLE_WAIT_MS		0
#define IMX208_TABLE_END		1
#define IMX208_MAX_RETRIES		3
#define IMX208_WAIT_MS			3

#define MAX_BUFFER_SIZE			32
#define IMX208_FRAME_LENGTH_ADDR_MSB	0x0340
#define IMX208_FRAME_LENGTH_ADDR_LSB	0x0341
#define IMX208_COARSE_TIME_ADDR_MSB	0x0202
#define IMX208_COARSE_TIME_ADDR_LSB	0x0203
#define IMX208_GAIN_ADDR		0x0205

#define MCLK_INIT_RATE			24000000

struct imx208_reg {
	u16 addr;
	u8 val;
};
static struct imx208_reg mode_1920x1080[] = {
	/* PLL Setting */
	{0x0305, 0x04},
	{0x0307, 0x87},
	{0x303C, 0x4B},
	{0x30A4, 0x02},
	/* Mode setting */
	{0x0112, 0x0A},
	{0x0113, 0x0A},
	{0x0340, 0x04},
	{0x0341, 0xB0},
	{0x0342, 0x08},
	{0x0343, 0xC8},
	{0x0344, 0x00},
	{0x0345, 0x08},
	{0x0346, 0x00},
	{0x0347, 0x08},
	{0x0348, 0x07},
	{0x0349, 0x87},
	{0x034A, 0x04},
	{0x034B, 0x3F},
	{0x034C, 0x07},
	{0x034D, 0x80},
	{0x034E, 0x04},
	{0x034F, 0x38},
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x3048, 0x00},
	{0x304E, 0x0A},
	{0x3050, 0x02},
	{0x309B, 0x00},
	{0x30D5, 0x00},
	{0x3301, 0x01},
	{0x3318, 0x61},
	/* Shutter Gain Setting */
	{0x0202, 0x01},
	{0x0203, 0x90},
	{0x0205, 0x00},
	{0x0100, 0x01},
	{IMX208_TABLE_WAIT_MS, IMX208_WAIT_MS},
	{IMX208_TABLE_END, 0x00}
};

enum {
	IMX208_MODE_1920X1080,
};

static struct imx208_reg *mode_table[] = {
	[IMX208_MODE_1920X1080] = mode_1920x1080,
};

struct imx208_power_rail {
	struct regulator *dvdd;
	struct regulator *avdd;
	struct regulator *iovdd;
};

struct imx208_info {
	struct miscdevice		miscdev_info;
	unsigned int			cam2_gpio;
	struct imx208_power_rail	power;
	struct imx208_sensordata	sensor_data;
	struct imx208_flash_control	flash_cap;
	struct i2c_client		*i2c_client;
	struct clk			*mclk;
	struct regmap			*regmap;
	atomic_t			in_use;
	bool				powered_on;
	struct sysedp_consumer		*sysedpc;
};

static const struct regmap_config sensor_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
};

/* Return number of regs */
static inline int
imx208_get_frame_length_regs(struct imx208_reg *regs, u32 frame_length)
{
	regs->addr = IMX208_FRAME_LENGTH_ADDR_MSB;
	regs->val = (frame_length >> 8) & 0xff;
	regs[1].addr = IMX208_FRAME_LENGTH_ADDR_LSB;
	regs[1].val = (frame_length) & 0xff;

	return 2;
}

/* Return number of regs */
static inline int
imx208_get_coarse_time_regs(struct imx208_reg *regs, u32 coarse_time)
{
	regs->addr = IMX208_COARSE_TIME_ADDR_MSB;
	regs->val = (coarse_time >> 8) & 0xff;
	regs[1].addr = IMX208_COARSE_TIME_ADDR_LSB;
	regs[1].val = (coarse_time) & 0xff;

	return 2;
}

static inline int imx208_get_gain_reg(struct imx208_reg *regs, u16 gain)
{
	regs->addr = IMX208_GAIN_ADDR;
	regs->val = gain;

	return 1;
}

static inline int imx208_read_reg(struct imx208_info *info, u16 addr, u8 *val)
{
	return regmap_read(info->regmap, addr, (unsigned int *) val);
}

static int imx208_write_reg(struct imx208_info *info, u16 addr, u8 val)
{
	int err;

	err = regmap_write(info->regmap, addr, val);

	if (err)
		dev_err(&info->i2c_client->dev, "%s:i2c write failed, %x = %x\n",
			__func__, addr, val);
	return err;
}

static int imx208_write_table(struct imx208_info *info,
				const struct imx208_reg table[],
				const struct imx208_reg override_list[],
				int num_override_regs)
{
	int err;
	const struct imx208_reg *next;
	int i;
	u16 val;

	for (next = table; next->addr != IMX208_TABLE_END; next++) {
		if (next->addr == IMX208_TABLE_WAIT_MS) {
			usleep_range(next->val * 1000, next->val * 1000 + 500);
			continue;
		}

		val = next->val;

		/* When an override list is passed in, replace the reg */
		/* value to write if the reg is in the list            */
		if (override_list) {
			for (i = 0; i < num_override_regs; i++) {
				if (next->addr == override_list[i].addr) {
					val = override_list[i].val;
					break;
				}
			}
		}

		err = imx208_write_reg(info, next->addr, val);
		if (err) {
			dev_err(&info->i2c_client->dev,
				"%s:imx208_write_table:%d", __func__, err);
			return err;
		}
	}
	return 0;
}

static int imx208_get_flash_cap(struct imx208_info *info)
{
	struct imx208_flash_control *fctl;

	if (info) {
		fctl = &info->flash_cap;
		dev_dbg(&info->i2c_client->dev,
			"edg: %x, st: %x, rpt: %x, dl: %x\n",
			fctl->edge_trig_en,
			fctl->start_edge,
			fctl->repeat,
			fctl->delay_frm);

		if (fctl->enable)
			return 0;
	}
	return -ENODEV;
}

static inline int imx208_set_flash_control(
	struct imx208_info *info, struct imx208_flash_control *fc)
{
	dev_dbg(&info->i2c_client->dev, "%s\n", __func__);
	return imx208_write_reg(info, 0x0802, 0x01);
}

static int imx208_set_mode(struct imx208_info *info, struct imx208_mode *mode)
{
	int sensor_mode;
	int err;
	int offset;
	struct imx208_reg reg_list[8];

	dev_info(&info->i2c_client->dev,
			"%s: xres %u yres %u framelength %u coarsetime %u gain %u\n",
			__func__, mode->xres, mode->yres, mode->frame_length,
			mode->coarse_time, mode->gain);

	if (mode->xres == 1920 && mode->yres == 1080) {
		sensor_mode = IMX208_MODE_1920X1080;
	} else {
		dev_err(&info->i2c_client->dev,
			"%s: invalid resolution supplied to set mode %d %d\n",
			__func__, mode->xres, mode->yres);
		return -EINVAL;
	}

	/* get a list of override regs for the asking frame length, */
	/* coarse integration time, and gain.                       */
	offset = imx208_get_frame_length_regs(reg_list, mode->frame_length);
	offset += imx208_get_coarse_time_regs(reg_list + offset,
						mode->coarse_time);
	offset += imx208_get_gain_reg(reg_list + offset, mode->gain);

	err = imx208_write_table(info,
				mode_table[sensor_mode],
				reg_list, offset);
	if (err)
		return err;
	dev_info(&info->i2c_client->dev, "[IMX208]: stream on.\n");
	return 0;
}

static int imx208_set_frame_length(struct imx208_info *info, u32 frame_length,
					bool group_hold)
{
	struct imx208_reg reg_list[2];
	int i = 0;
	int ret;
	int num_regs;

	num_regs = imx208_get_frame_length_regs(reg_list, frame_length);

	if (group_hold) {
		ret = imx208_write_reg(info, 0x0104, 0x01);
		if (ret)
			return ret;
	}

	for (i = 0; i < num_regs; i++) {
		ret = imx208_write_reg(info, reg_list[i].addr,
			 reg_list[i].val);
		if (ret)
			return ret;
	}

	if (group_hold) {
		ret = imx208_write_reg(info, 0x0104, 0x0);
		if (ret)
			return ret;
	}

	return 0;
}

static int imx208_set_coarse_time(struct imx208_info *info, u32 coarse_time,
					bool group_hold)
{
	int ret;
	int num_regs;
	struct imx208_reg reg_list[2];
	int i = 0;

	num_regs = imx208_get_coarse_time_regs(reg_list, coarse_time);

	if (group_hold) {
		ret = imx208_write_reg(info, 0x104, 0x01);
		if (ret)
			return ret;
	}

	for (i = 0; i < num_regs; i++) {
		ret = imx208_write_reg(info, reg_list[i].addr,
			 reg_list[i].val);
		if (ret)
			return ret;
	}

	if (group_hold) {
		ret = imx208_write_reg(info, 0x104, 0x0);
		if (ret)
			return ret;
	}
	return 0;
}

static int
imx208_set_gain(struct imx208_info *info, u16 gain, bool group_hold)
{
	int ret;
	struct imx208_reg reg_list;

	imx208_get_gain_reg(&reg_list, gain);

	if (group_hold) {
		ret = imx208_write_reg(info, 0x104, 0x1);
		if (ret)
			return ret;
	}

	ret = imx208_write_reg(info, reg_list.addr, reg_list.val);
	if (ret)
		return ret;

	if (group_hold) {
		ret = imx208_write_reg(info, 0x104, 0x0);
		if (ret)
			return ret;
	}
	return 0;
}

static int
imx208_set_group_hold(struct imx208_info *info, struct imx208_ae *ae)
{
	int ret;
	int count = 0;
	bool group_hold_enabled = false;

	if (ae->gain_enable)
		count++;
	if (ae->coarse_time_enable)
		count++;
	if (ae->frame_length_enable)
		count++;
	if (count >= 2)
		group_hold_enabled = true;

	if (group_hold_enabled) {
		ret = imx208_write_reg(info, 0x104, 0x1);
		if (ret)
			return ret;
	}

	if (ae->gain_enable)
		imx208_set_gain(info, ae->gain, false);
	if (ae->coarse_time_enable)
		imx208_set_coarse_time(info, ae->coarse_time, true);
	if (ae->frame_length_enable)
		imx208_set_frame_length(info, ae->frame_length, false);

	if (group_hold_enabled) {
		ret = imx208_write_reg(info, 0x104, 0x0);
		if (ret)
			return ret;
	}

	return 0;
}

static int imx208_power_on(struct imx208_power_rail *pw)
{
	int err;
	struct imx208_info *info = container_of(pw, struct imx208_info, power);
	struct device *dev = &info->i2c_client->dev;

	if (info->powered_on) {
		dev_err(&info->i2c_client->dev, "Already on\n");
		return 0;
	}

	gpio_set_value(info->cam2_gpio, 0);
	usleep_range(10, 20);

	err = regulator_enable(pw->avdd);
	if (err) {
		dev_err(dev, "%s: Fail to enable avdd\n", __func__);
		goto imx208_avdd_fail;
	}

	err = regulator_enable(pw->dvdd);
	if (err) {
		dev_err(dev, "%s: Fail to enable dvdd\n", __func__);
		goto imx208_dvdd_fail;
	}

	err = regulator_enable(pw->iovdd);
	if (err) {
		dev_err(dev, "%s: Fail to enable iovdd\n", __func__);
		goto imx208_iovdd_fail;
	}

	/* Wait for voltages to stabilize. */
	usleep_range(100, 200);

	gpio_set_value(info->cam2_gpio, 1);

	err = clk_prepare_enable(info->mclk);
	if (err) {
		dev_err(dev, "%s: Cannot enable MClk\n", __func__);
		return err;
	}

	/* Wait for communication start (min. 300 uS). */
	usleep_range(301, 310);

	info->powered_on = true;
	return 0;


imx208_iovdd_fail:
	regulator_disable(pw->dvdd);

imx208_dvdd_fail:
	regulator_disable(pw->avdd);

imx208_avdd_fail:
	clk_disable_unprepare(info->mclk);
	return err;
}

static int imx208_power_off(struct imx208_power_rail *pw)
{
	struct imx208_info *info = container_of(pw, struct imx208_info, power);

	if (!info->powered_on) {
		dev_err(&info->i2c_client->dev, "Already off\n");
		return 0;
	}

	/* Frame output end -> XCLR falling/INCK end (min. 128 INCK cycles). */
	usleep_range(1, 2);

	clk_disable_unprepare(info->mclk);
	gpio_set_value(info->cam2_gpio, 0);

	/* XCLR falling -> power supply down (min 500 nS). */
	usleep_range(1, 2);

	regulator_disable(pw->iovdd);
	regulator_disable(pw->dvdd);
	regulator_disable(pw->avdd);

	info->powered_on = false;
	return 0;
}

static long imx208_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	struct imx208_info *info = file->private_data;

	switch (cmd) {
	case IMX208_IOCTL_SET_POWER:
		if (arg) {
			if (info->sysedpc)
				sysedp_set_state(info->sysedpc, 1);
			err = imx208_power_on(&info->power);
		} else {
			err = imx208_power_off(&info->power);
			if (info->sysedpc)
				sysedp_set_state(info->sysedpc, 0);
		}
		break;
	case IMX208_IOCTL_SET_MODE:
	{
		struct imx208_mode mode;

		if (copy_from_user(&mode, (const void __user *)arg,
			sizeof(mode))) {
			dev_err(&info->i2c_client->dev,
				"%s:Failed to get mode from user.\n", __func__);
			return -EFAULT;
		}
		return imx208_set_mode(info, &mode);
	}
	case IMX208_IOCTL_SET_FRAME_LENGTH:
		return imx208_set_frame_length(info, (u32)arg, true);
	case IMX208_IOCTL_SET_COARSE_TIME:
		return imx208_set_coarse_time(info, (u32)arg, true);
	case IMX208_IOCTL_SET_GAIN:
		return imx208_set_gain(info, (u16)arg, true);
	case IMX208_IOCTL_GET_STATUS:
	{
		u8 status = 0;

		if (copy_to_user((void __user *)arg, &status, sizeof(status))) {
			dev_err(&info->i2c_client->dev,
				"%s:Failed to copy status to user\n", __func__);
			return -EFAULT;
		}
		return 0;
	}
	case IMX208_IOCTL_GET_SENSORDATA:
	{
		if (copy_to_user((void __user *)arg, &info->sensor_data,
				sizeof(info->sensor_data))) {
			dev_info(&info->i2c_client->dev,
				"%s:Failed to copy fuse id to user space\n",
				__func__);
			return -EFAULT;
		}
		return 0;
	}
	case IMX208_IOCTL_SET_GROUP_HOLD:
	{
		struct imx208_ae ae;

		if (copy_from_user(&ae, (const void __user *)arg, sizeof(ae))) {
			dev_err(&info->i2c_client->dev,
				"%s:fail group hold\n", __func__);
			return -EFAULT;
		}
		return imx208_set_group_hold(info, &ae);
	}
	case IMX208_IOCTL_SET_FLASH_MODE:
	{
		struct imx208_flash_control values;

		dev_dbg(&info->i2c_client->dev,
			"IMX208_IOCTL_SET_FLASH_MODE\n");
		if (copy_from_user(&values,
			(const void __user *)arg, sizeof(values))) {
			err = -EFAULT;
			break;
		}
		err = imx208_set_flash_control(info, &values);
		break;
	}
	case IMX208_IOCTL_GET_FLASH_CAP:
		err = imx208_get_flash_cap(info);
		break;
	default:
		dev_err(&info->i2c_client->dev, "%s:unknown cmd.\n", __func__);
		err = -EINVAL;
	}

	return err;
}

static int imx208_open(struct inode *inode, struct file *file)
{
	struct miscdevice *miscdev = file->private_data;
	struct imx208_info *info;

	info = container_of(miscdev, struct imx208_info, miscdev_info);
	/* check if the device is in use */
	if (atomic_xchg(&info->in_use, 1)) {
		dev_err(&info->i2c_client->dev, "%s:BUSY!\n", __func__);
		return -EBUSY;
	}

	file->private_data = info;

	return 0;
}

static int imx208_release(struct inode *inode, struct file *file)
{
	struct imx208_info *info = file->private_data;

	if (info->powered_on)
		imx208_power_off(&info->power);
	atomic_xchg(&info->in_use, 0);
	return 0;
}

static const struct file_operations imx208_fileops = {
	.owner = THIS_MODULE,
	.open = imx208_open,
	.unlocked_ioctl = imx208_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = imx208_ioctl,
#endif
	.release = imx208_release,
};

static struct miscdevice imx208_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "imx208",
	.fops = &imx208_fileops,
};

static int imx208_check_module(struct imx208_info *info)
{
	u8 module_id[2];
	int err;
	struct imx208_power_rail *pw = &info->power;

	err = imx208_power_on(pw);
	if (err)
		return err;

	err = imx208_read_reg(info, 0x0000, &module_id[0]);
	if (err)
		goto check_fail;

	err = imx208_read_reg(info, 0x0001, &module_id[1]);
	if (err)
		goto check_fail;

	if (module_id[0] != 0x02 || module_id[1] != 0x08)
		err = -ENODEV;

check_fail:
	imx208_power_off(pw);
	return err;
}

static int
imx208_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct imx208_info *info;
	struct imx208_power_rail *pw;
	int err;

	dev_info(&client->dev, "[IMX208]: probing sensor.\n");

	if (client->dev.of_node == NULL) {
		dev_err(&client->dev,
			"[IMX208]:%s:Unable to get DT data\n", __func__);
		return -EFAULT;
	}

	info = devm_kzalloc(&client->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->i2c_client = client;

	info->regmap = devm_regmap_init_i2c(client, &sensor_regmap_config);
	if (IS_ERR(info->regmap)) {
		dev_err(&client->dev, "regmap init failed\n");
		return PTR_ERR(info->regmap);
	}

	pw = &info->power;
	pw->avdd = devm_regulator_get(&info->i2c_client->dev, "vana");
	if (IS_ERR(pw->avdd)) {
		dev_err(&info->i2c_client->dev, "Fail to get vana");
		return PTR_ERR(pw->avdd);
	}
	pw->iovdd = devm_regulator_get(&info->i2c_client->dev, "vif");
	if (IS_ERR(pw->iovdd)) {
		dev_err(&info->i2c_client->dev, "Fail to get vif");
		return PTR_ERR(pw->iovdd);
	}
	pw->dvdd = devm_regulator_get(&info->i2c_client->dev, "vdig");
	if (IS_ERR(pw->dvdd)) {
		dev_err(&info->i2c_client->dev, "Fail to get vdig");
		return PTR_ERR(pw->dvdd);
	}
	info->cam2_gpio = of_get_named_gpio(client->dev.of_node,
						"cam2-gpios", 0);
	err = devm_gpio_request_one(&client->dev, info->cam2_gpio,
					GPIOF_OUT_INIT_LOW, "cam2-reset");
	if (err) {
		dev_err(&info->i2c_client->dev, "Fail to request GPIO");
		return err;
	}

	atomic_set(&info->in_use, 0);

	info->mclk = devm_clk_get(&client->dev, "clk_out_3");
	if (IS_ERR(info->mclk)) {
		dev_err(&client->dev, "%s: unable to get mclk\n",
			__func__);
		return PTR_ERR(info->mclk);
	}

	i2c_set_clientdata(client, info);
	info->powered_on = false;

	err = imx208_check_module(info);
	if (err) {
		dev_err(&client->dev, "%s: Module check failed", __func__);
		return err;
	}

	info->miscdev_info = imx208_device;
	err = misc_register(&info->miscdev_info);
	if (err) {
		dev_err(&client->dev,
			"%s:Unable to register misc device!\n", __func__);
		return err;
	}

	info->sysedpc = sysedp_create_consumer(client->dev.of_node,
						client->dev.of_node->name);
	if (IS_ERR_OR_NULL(info->sysedpc))
		dev_warn(&info->i2c_client->dev,
			 "%s: Create sysedp consumer failed!\n", __func__);

	dev_info(&info->i2c_client->dev, "[IMX208]: end of probing sensor.\n");
	return 0;

}

static int imx208_remove(struct i2c_client *client)
{
	struct imx208_info *info = i2c_get_clientdata(client);

	sysedp_free_consumer(info->sysedpc);
	misc_deregister(&info->miscdev_info);
	return 0;
}

static const struct i2c_device_id imx208_id[] = {
	{ "imx208", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, imx208_id);

static struct i2c_driver imx208_i2c_driver = {
	.driver = {
		.name = "imx208",
		.owner = THIS_MODULE,
	},
	.probe = imx208_probe,
	.remove = imx208_remove,
	.id_table = imx208_id,
};

module_i2c_driver(imx208_i2c_driver);
