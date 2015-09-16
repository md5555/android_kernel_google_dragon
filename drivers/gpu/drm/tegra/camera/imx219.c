/*
 * imx219.c - imx219 sensor driver
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
#include <uapi/linux/tegra/imx219.h>
#include <linux/gpio.h>
#include <linux/module.h>

#include <linux/kernel.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include <soc/tegra/sysedp.h>

#include "imx219_tables.h"

#define MCLK_INIT_RATE 24000000

struct imx219_power_rail {
	struct regulator *dvdd;
	struct regulator *avdd;
	struct regulator *iovdd;
	struct regulator *vdd_af;
	struct clk *mclk;
	unsigned int reset_gpio;
};

struct imx219_info {
	struct miscdevice		miscdev_info;
	struct imx219_power_rail power;
	struct imx219_fuseid		fuse_id;
	struct i2c_client		*i2c_client;
	atomic_t			in_use;
	bool				powered_on;
	struct sysedp_consumer		*sysedpc;
#ifdef CONFIG_DEBUG_FS
	struct dentry			*debugfs_root;
	u32				debug_i2c_offset;
#endif
	/* AF data */
	u8 afdat[4];
	bool afdat_read;
};

/* Return number of registers */
static inline u8
imx219_get_frame_length_regs(struct imx219_reg *regs, u32 frame_length)
{
	regs->addr = 0x0160;
	regs->val = (frame_length >> 8) & 0xff;
	regs[1].addr = 0x0161;
	regs[1].val = (frame_length) & 0xff;
	return 2;
}

/* Return number of registers */
static inline u8
imx219_get_coarse_time_regs(struct imx219_reg *regs, u32 coarse_time)
{
	regs->addr = 0x15a;
	regs->val = (coarse_time >> 8) & 0xff;
	regs[1].addr = 0x15b;
	regs[1].val = (coarse_time) & 0xff;
	return 2;
}

/* Return number of registers */
static inline u8
imx219_get_gain_reg(struct imx219_reg *regs, struct imx219_gain *gain)
{
	regs->addr = 0x157;
	regs->val = gain->again;
	regs[1].addr = 0x158;
	regs[1].val = gain->dgain_upper;
	regs[2].addr = 0x159;
	regs[2].val = gain->dgain_lower;
	return 3;
}

static int
imx219_read_reg(struct i2c_client *client, u16 addr, u8 *val)
{
	int err;
	struct i2c_msg msg[2];
	unsigned char data[3];

	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = ARRAY_SIZE(msg);
	msg[0].buf = data;

	/* high byte goes out first */
	data[0] = addr >> 8;
	data[1] = addr & 0xff;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = data + 2;

	err = i2c_transfer(client->adapter, msg, 2);
	if (err == 2) {
		*val = data[2];
		return 0;
	}

	dev_err(&client->dev, "%s:i2c read failed, addr %x, err %d\n",
			__func__, addr, err);

	return err;
}

static int
imx219_write_reg(struct i2c_client *client, u16 addr, u8 val)
{
	int err;
	struct i2c_msg msg;
	unsigned char data[3];

	data[0] = addr >> 8;
	data[1] = addr & 0xff;
	data[2] = val & 0xff;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = ARRAY_SIZE(data);
	msg.buf = data;

	err = i2c_transfer(client->adapter, &msg, 1);
	if (err == 1)
		return 0;

	dev_err(&client->dev, "%s:i2c write failed, addr %x, val %x, err %d\n",
			__func__, addr, val, err);

	return err;
}

static int
imx219_write_table(struct i2c_client *client,
				 const struct imx219_reg *table,
				 const struct imx219_reg *override_list,
				 int num_override_regs)
{
	int err;
	const struct imx219_reg *next;
	int i;
	u16 val;

	for (next = table; next->addr != IMX219_TABLE_END; next++) {
		if (next->addr == IMX219_TABLE_WAIT_MS) {
			usleep_range(1000 * next->val, 1000 * next->val + 500);
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

		err = imx219_write_reg(client, next->addr, val);
		if (err) {
			dev_err(&client->dev,
				"%s:imx219_write_table:%d", __func__, err);
			return err;
		}
	}
	return 0;
}

static int
imx219_set_mode(struct imx219_info *info, struct imx219_mode *mode)
{
	int err, i, offset;
	struct imx219_reg reg_list[8];
	struct imx219_reg *mode_table_entry = NULL;

	dev_info(&info->i2c_client->dev, "%s:xres %u yres %u framelength %u coarsetime %u again %u dgain %u%u\n"
			 , __func__, mode->xres, mode->yres, mode->frame_length,
			 mode->coarse_time, mode->gain.again,
			 mode->gain.dgain_upper, mode->gain.dgain_lower);

	for (i = 0; i < ARRAY_SIZE(mode_list); i++) {
		if (mode->xres == mode_list[i].xres &&
			mode->yres == mode_list[i].yres)
			mode_table_entry = mode_list[i].mode_table;
	}

	if (mode_table_entry == NULL) {
		dev_err(&info->i2c_client->dev, "%s: invalid resolution supplied to set mode %d %d\n",
			 __func__, mode->xres, mode->yres);
		return -EINVAL;
	}

	/* get a list of override regs for the asking frame length, */
	/* coarse integration time, and gain.                       */
	offset = imx219_get_frame_length_regs(reg_list, mode->frame_length);
	offset += imx219_get_coarse_time_regs(reg_list + offset,
						mode->coarse_time);
	offset += imx219_get_gain_reg(reg_list + offset, &mode->gain);

	err = imx219_write_table(info->i2c_client,
				mode_table_entry,
				reg_list, offset);

	dev_info(&info->i2c_client->dev, "[IMX219]: stream on.\n");
	return 0;
}

static int
imx219_set_frame_length(struct imx219_info *info, u32 frame_length)
{
	struct imx219_reg reg_list[2];
	int i = 0;
	int ret, offset;

	offset = imx219_get_frame_length_regs(reg_list, frame_length);

	for (i = 0; i < offset; i++) {
		ret = imx219_write_reg(info->i2c_client, reg_list[i].addr,
			 reg_list[i].val);
		if (ret)
			return ret;
	}

	return 0;
}

static int
imx219_set_coarse_time(struct imx219_info *info, u32 coarse_time)
{
	int ret, offset;
	struct imx219_reg reg_list[2];
	int i = 0;

	offset = imx219_get_coarse_time_regs(reg_list, coarse_time);

	for (i = 0; i < offset; i++) {
		ret = imx219_write_reg(info->i2c_client, reg_list[i].addr,
			 reg_list[i].val);
		if (ret)
			return ret;
	}

	return 0;
}

static int
imx219_set_gain(struct imx219_info *info, struct imx219_gain *gain)
{
	int i;
	int ret, offset;
	struct imx219_reg reg_list[3];

	offset = imx219_get_gain_reg(reg_list, gain);
	for (i = 0; i < offset; ++i) {
		ret = imx219_write_reg(info->i2c_client,
					reg_list[i].addr,
					reg_list[i].val);
		if (ret) {
			dev_err(&info->i2c_client->dev, "%s: unable to write register: %d",
						__func__, ret);
			return ret;
		}
	}

	return ret;
}

static int
imx219_set_group_hold(struct imx219_info *info, struct imx219_ae *ae)
{
	int ret = 0;
	struct imx219_reg reg_list[8];
	int offset = 0;

	if (ae->frame_length_enable)
		offset = imx219_get_frame_length_regs(reg_list + offset,
							ae->frame_length);
	if (ae->coarse_time_enable)
		offset += imx219_get_coarse_time_regs(reg_list + offset,
							ae->coarse_time);
	if (ae->gain_enable)
		offset += imx219_get_gain_reg(reg_list + offset, &ae->gain);

	reg_list[offset].addr = IMX219_TABLE_END;

	ret = imx219_write_table(info->i2c_client,
				reg_list, NULL, 0);
	return ret;
}



static int
imx219_get_af_data(struct imx219_info *info)
{
	int ret = 0;
	int i;

	if (info->afdat_read)
		return 0;

	imx219_write_reg(info->i2c_client, 0x0100, 0); /* SW-Stanby */
	usleep_range(1000 * 33, 1000 * 33 + 500); /* wait one frame */

	imx219_write_table(info->i2c_client, imx219_read_prep, NULL, 0);

	for (i = 0; i < ARRAY_SIZE(info->afdat); i++)
		ret |= imx219_read_reg(info->i2c_client, 0x3204 + i,
					&info->afdat[3-i]);
	info->afdat_read = true;
	return ret;
}

static int
imx219_power_on(struct imx219_info *info)
{
	int err;
	struct imx219_power_rail *pw = &info->power;
	struct device *dev =  &info->i2c_client->dev;

	if (info->powered_on) {
		dev_err(&info->i2c_client->dev, "%s:Already on\n", __func__);
		return 0;
	}

	gpio_set_value(pw->reset_gpio, 0);

	err = regulator_enable(pw->avdd);
	if (err) {
		dev_err(dev, "%s: fail to enable avdd", __func__);
		return err;
	}

	err = regulator_enable(pw->iovdd);
	if (err) {
		dev_err(dev, "%s: fail to enable iovdd", __func__);
		goto imx219_iovdd_fail;
	}

	err = regulator_enable(pw->dvdd);
	if (err) {
		dev_err(dev, "%s: fail to enable dvdd", __func__);
		goto imx219_dvdd_fail;
	}

	/* Wait for voltages to stabilize. */
	usleep_range(100, 200);

	err = clk_prepare_enable(pw->mclk);
	if (err < 0) {
		dev_err(&info->i2c_client->dev, "%s: Fail to enable MCLK\n",
			__func__);
		goto imx219_mclk_fail;
	}

	/* Wait several clocks before deasserting the reset (min. 0.5 uS). */
	usleep_range(1, 2);

	gpio_set_value(pw->reset_gpio, 1);

	/* Wait for internal XCLR low-to-high (max. 200 uS). */
	usleep_range(300, 310);

	info->powered_on = true;
	return 0;

imx219_mclk_fail:
	regulator_disable(pw->dvdd);

imx219_dvdd_fail:
	regulator_disable(pw->iovdd);

imx219_iovdd_fail:
	regulator_disable(pw->avdd);

	dev_err(&info->i2c_client->dev, "%s failed.\n", __func__);
	return err;
}

static int
imx219_power_off(struct imx219_info *info)
{
	struct imx219_power_rail *pw = &info->power;

	if (!info->powered_on) {
		dev_err(&info->i2c_client->dev, "Already off\n");
		return 0;
	}

	gpio_set_value(pw->reset_gpio, 0);

	/* Wait for software standby to activate (max. 10 uS). */
	usleep_range(20, 30);

	regulator_disable(pw->dvdd);
	regulator_disable(pw->iovdd);
	regulator_disable(pw->avdd);

	/* Wait for regulators to discharge. */
	usleep_range(30, 40);

	clk_disable_unprepare(pw->mclk);

	info->powered_on = false;
	return 0;
}

static int
imx219_get_sensor_id(struct imx219_info *info)
{
	int ret = 0;
	int i;
	bool power_stat;
	u8 bak = 0;

	dev_dbg(&info->i2c_client->dev, "%s\n", __func__);
	if (info->fuse_id.size)
		return 0;

	power_stat = info->powered_on;
	if (!power_stat) {
		ret = imx219_power_on(info);
		if (ret) {
			dev_err(&info->i2c_client->dev, "%s:Power on failed\n",
					__func__);
			return ret;
		}
	}

	for (i = 0; i < 4; i++) {
		ret |= imx219_read_reg(info->i2c_client, 0x0004 + i, &bak);
		info->fuse_id.data[i] = bak;
	}

	for (i = 0; i < 2; i++) {
		ret |= imx219_read_reg(info->i2c_client, 0x000d + i , &bak);
		info->fuse_id.data[i + 4] = bak;
	}

	if (!ret)
		info->fuse_id.size = 6;

	if (!power_stat)
		imx219_power_off(info);
	return ret;
}

static long
imx219_ioctl(struct file *file,
			 unsigned int cmd, unsigned long arg)
{
	int err = 0;
	struct imx219_info *info = file->private_data;

	switch (cmd) {
	case IMX219_IOCTL_SET_POWER:
		if (arg) {
			if (info->sysedpc)
				sysedp_set_state(info->sysedpc, 1);
			err = imx219_power_on(info);
		} else {
			err = imx219_power_off(info);
			if (info->sysedpc)
				sysedp_set_state(info->sysedpc, 0);
		}
		if (err) {

			dev_err(&info->i2c_client->dev,
				"%s:Failed to power on.\n", __func__);
			return -EFAULT;
		}
		break;
	case IMX219_IOCTL_SET_MODE:
	{
		struct imx219_mode mode;

		if (copy_from_user(&mode, (const void __user *)arg,
			sizeof(mode))) {
			dev_err(&info->i2c_client->dev,
				"%s:Failed to get mode from user.\n", __func__);
			return -EFAULT;
		}
		return imx219_set_mode(info, &mode);
	}
	case IMX219_IOCTL_SET_FRAME_LENGTH:
		err = imx219_set_frame_length(info, (u32)arg);
		break;
	case IMX219_IOCTL_SET_COARSE_TIME:
		err = imx219_set_coarse_time(info, (u32)arg);
		break;
	case IMX219_IOCTL_SET_GAIN:
	{
		struct imx219_gain gain;

		if (copy_from_user(&gain, (const void __user *)arg,
			sizeof(gain))) {
			dev_err(&info->i2c_client->dev,
				"%s:Failed to get gain from user\n", __func__);
			return -EFAULT;
		}
		err = imx219_set_gain(info, &gain);
		break;
	}
	case IMX219_IOCTL_GET_STATUS:
	{
		/* TODO:implement real get_status function
		 * return 0 for now
		 */
		u8 status = 0;

		if (copy_to_user((void __user *)arg, &status, sizeof(status))) {
			dev_err(&info->i2c_client->dev,
				"%s:Failed to copy status to user\n", __func__);
			return -EFAULT;
		}
		return 0;
	}
	case IMX219_IOCTL_GET_FUSEID:
	{
		err = imx219_get_sensor_id(info);

		if (err) {
			dev_err(&info->i2c_client->dev,
				"%s:Failed to get fuse id info.\n", __func__);
			return err;
		}
		if (copy_to_user((void __user *)arg, &info->fuse_id,
				sizeof(info->fuse_id))) {
			dev_err(&info->i2c_client->dev,
				"%s:Failed to copy fuseid\n", __func__);
			return -EFAULT;
		}
		return 0;
	}
	case IMX219_IOCTL_SET_GROUP_HOLD:
	{
		struct imx219_ae ae;

		if (copy_from_user(&ae, (const void __user *)arg, sizeof(ae))) {
			dev_err(&info->i2c_client->dev,
				"%s:fail group hold\n", __func__);
			return -EFAULT;
		}
		return imx219_set_group_hold(info, &ae);
	}
	case IMX219_IOCTL_GET_AFDAT:
	{
		err = imx219_get_af_data(info);

		if (err) {
			dev_err(&info->i2c_client->dev,
				"%s:Failed to get af data.\n", __func__);
			return err;
		}
		if (copy_to_user((void __user *)arg, info->afdat,
			sizeof(info->afdat))) {
			dev_err(&info->i2c_client->dev,
				"%s:Failed to copy status to user\n", __func__);
			return -EFAULT;
		}
		return 0;
	}
	case IMX219_IOCTL_SET_FLASH_MODE:
	{
		dev_dbg(&info->i2c_client->dev,
			"IMX219_IOCTL_SET_FLASH_MODE not used\n");
		return -ENODEV;/* not support on sensor strobe */
	}
	case IMX219_IOCTL_GET_FLASH_CAP:
		return -ENODEV;/* not support on sensor strobe */
	default:
		dev_err(&info->i2c_client->dev,
			"%s:unknown cmd: %u\n", __func__, cmd);
		err = -EINVAL;
	}

	return err;
}

static int
imx219_open(struct inode *inode, struct file *file)
{
	struct miscdevice *miscdev = file->private_data;
	struct imx219_info *info;

	info = container_of(miscdev, struct imx219_info, miscdev_info);
	/* check if the device is in use */
	if (atomic_xchg(&info->in_use, 1)) {
		dev_err(&info->i2c_client->dev, "%s:BUSY!\n", __func__);
		return -EBUSY;
	}

	file->private_data = info;
	return 0;
}

static int
imx219_release(struct inode *inode, struct file *file)
{
	struct imx219_info *info = file->private_data;

	if (info->powered_on)
		imx219_power_off(info);
	atomic_xchg(&info->in_use, 0);
	return 0;
}

static const struct file_operations imx219_fileops = {
	.owner = THIS_MODULE,
	.open = imx219_open,
	.unlocked_ioctl = imx219_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = imx219_ioctl,
#endif
	.release = imx219_release,
};

static const struct miscdevice imx219_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "imx219",
	.fops = &imx219_fileops,
};

static int imx219_check_module(struct imx219_info *info)
{
	u8 module_id[2];
	int err;

	err = imx219_power_on(info);
	if (err)
		return err;

	err = imx219_read_reg(info->i2c_client, 0x0000, &module_id[0]);
	if (err)
		goto check_fail;

	err = imx219_read_reg(info->i2c_client, 0x0001, &module_id[1]);
	if (err)
		goto check_fail;

	if (module_id[0] != 0x02 || module_id[1] != 0x19)
		err = -ENODEV;

check_fail:
	imx219_power_off(info);
	return err;
}
static int
imx219_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct imx219_info *info;
	struct device_node *np = client->dev.of_node;
	struct imx219_power_rail *pw = NULL;
	int err = 0;

	dev_info(&client->dev, "[IMX219]: probing sensor.\n");

	info = devm_kzalloc(&client->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	if (!np) {
		dev_err(&client->dev,
			"[IMX219]:%s:Unable to read DT info\n", __func__);
		return -EFAULT;
	}

	pw = &info->power;
	pw->reset_gpio = of_get_named_gpio(np, "reset-gpios", 0);
	err = devm_gpio_request_one(&client->dev, pw->reset_gpio,
					GPIOF_OUT_INIT_LOW, "cam-reset");
	if (err) {
		dev_err(&client->dev,
		"[IMX219]:%s:Unable to request cam-reset gpio\n", __func__);
		return err;
	}

	info->i2c_client = client;
	atomic_set(&info->in_use, 0);
	info->afdat_read = false;
	info->powered_on = false;

	pw->mclk = devm_clk_get(&info->i2c_client->dev, "clk_out_3");
	if (IS_ERR(pw->mclk)) {
		dev_err(&info->i2c_client->dev, "%s: unable to get clk_out_3\n",
			__func__);
		return PTR_ERR(pw->mclk);
	}

	pw->avdd = devm_regulator_get(&info->i2c_client->dev, "vana");
	if (IS_ERR(pw->avdd)) {
		dev_err(&info->i2c_client->dev, "%s Cannot get regulator vana\n",
			__func__);
		return PTR_ERR(pw->avdd);
	}

	pw->dvdd = devm_regulator_get(&info->i2c_client->dev, "vdig");
	if (IS_ERR(pw->dvdd)) {
		dev_err(&info->i2c_client->dev, "%s Cannot get regulator vdig\n",
			__func__);
		return PTR_ERR(pw->dvdd);
	}

	pw->iovdd = devm_regulator_get(&info->i2c_client->dev, "dovdd");
	if (IS_ERR(pw->iovdd)) {
		dev_err(&info->i2c_client->dev, "%s Cannot get regulator dvodd\n",
			__func__);
		return PTR_ERR(pw->iovdd);
	}

	info->miscdev_info = imx219_device;
	i2c_set_clientdata(client, info);

	err = imx219_check_module(info);
	if (err) {
		dev_err(&client->dev, "%s: Module check failed", __func__);
		return err;
	}

	err = misc_register(&info->miscdev_info);
	if (err) {
		dev_err(&info->i2c_client->dev,
			"%s:Unable to register misc device!\n", __func__);
		return err;
	}
	info->sysedpc = sysedp_create_consumer(np, np->name);
	if (IS_ERR_OR_NULL(info->sysedpc))
		dev_warn(&info->i2c_client->dev,
			 "%s: Create sysedp consumer failed!\n", __func__);

	return 0;
}

static int
imx219_remove(struct i2c_client *client)
{
	struct imx219_info *info = i2c_get_clientdata(client);

	sysedp_free_consumer(info->sysedpc);
	misc_deregister(&info->miscdev_info);
	return 0;
}

static const struct i2c_device_id imx219_id[] = {
	{ "imx219", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, imx219_id);

static struct i2c_driver imx219_i2c_driver = {
	.driver = {
		.name = "imx219",
		.owner = THIS_MODULE,
	},
	.probe = imx219_probe,
	.remove = imx219_remove,
	.id_table = imx219_id,
};
module_i2c_driver(imx219_i2c_driver);
