/*
 * Device driver for monitoring ambient light intensity (lux)
 * within the TAOS tsl258x family of devices
 *
 * Copyright (c) 2011, TAOS Corporation.
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
 * 51 Franklin Street, Fifth Floor, Boston, MA	02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include "../iio.h"

#define DEVICE_ID			"tsl2583"

#define MAX_DEVICE_REGS			32

/* Triton register offsets */
#define	TAOS_REG_MAX			8

/* Device Registers and Masks */
#define TSL258X_CNTRL			0x00
#define TSL258X_STATUS			0x00
#define TSL258X_ALS_TIME		0x01
#define TSL258X_INTERRUPT		0x02
#define TSL258X_GAIN			0x07
#define TSL258X_REVID			0x11
#define TSL258X_CHIPID			0x12
#define TSL258X_SMB_4			0x13
#define TSL258X_ALS_CHAN0LO		0x14
#define TSL258X_ALS_CHAN0HI		0x15
#define TSL258X_ALS_CHAN1LO		0x16
#define TSL258X_ALS_CHAN1HI		0x17
#define TSL258X_TMR_LO			0x18
#define TSL258X_TMR_HI			0x19

/* Skate cmd reg masks */
#define TSL258X_CMD_REG			0x80
#define TSL258X_CMD_BYTE_RW		0x00
#define TSL258X_CMD_WORD_BLK_RW		0x20
#define TSL258X_CMD_SPL_FN		0x60
#define TSL258X_CMD_ALS_INTCLR		0X01

/* Skate cntrl reg masks */
#define TSL258X_CNTL_REG_CLEAR		0x00
#define TSL258X_CNTL_ALS_INT_ENBL	0x10
#define TSL258X_CNTL_WAIT_TMR_ENBL	0x08
#define TSL258X_CNTL_ADC_ENBL		0x02
#define TSL258X_CNTL_PWRON		0x01
#define TSL258X_CNTL_ALSPON_ENBL	0x03
#define TSL258X_CNTL_INTALSPON_ENBL	0x13

/* Skate status reg masks */
#define TSL258X_STA_ADCVALID		0x01
#define TSL258X_STA_ALSINTR		0x10
#define TSL258X_STA_ADCINTR		0x10

/* Lux constants */
#define	MAX_LUX				65535

#define NR_GAIN_LEVELS			4

#define MAX_LUX_TABLE_LEVELS		11

enum {
	TAOS_CHIP_UNKNOWN = 0, TAOS_CHIP_WORKING = 1, TAOS_CHIP_SLEEP = 2,
	TAOS_CHIP_SUSPENDED = 3
} TAOS_CHIP_WORKING_STATUS;

/* Per-device data */

struct taos_settings {
	int als_time;
	int als_gain;
	int als_gain_adj0;
	int als_gain_adj1;
	int als_gain_trim;
	int als_cal_target;
};

struct taos_lux {
	unsigned int ratio;
	unsigned int ch0;
	unsigned int ch1;
};

struct tsl258x_chip {
	struct mutex als_mutex;
	struct i2c_client *client;
	struct iio_dev *iio_dev;
	struct taos_settings settings;
	struct taos_lux device_lux[MAX_LUX_TABLE_LEVELS];
	int als_time_scale;
	int als_saturation;
	int taos_chip_status;
	u16 cur_lux;
	u8 reg_cache[TAOS_REG_MAX];
};

static int taos_i2c_read(struct i2c_client *client, u8 reg);
static int taos_i2c_write_command(struct i2c_client *client, u8 reg);

/*
 * Initial values for device - these values can/will be changed by driver
 * and applications as needed.
 */
static const u8 default_taos_config[8] = {
	0x00, 0xee, 0x00, 0x03, 0x00, 0xFF, 0xFF, 0x00
}; /*	cntrl atime intC  Athl0 Athl1 Athh0 Athh1 gain */

static const struct taos_lux default_device_lux[] = {
	{  9830,  8520, 15729 },
	{ 12452, 10807, 23344 },
	{ 14746,  6383, 11705 },
	{ 17695,  4063,  6554 },
};

static void taos_set_gain(struct tsl258x_chip *chip, int val)
{
	/* Index = (0 - 3) Used to validate the gain selection index */
	static const s16 gainadj[NR_GAIN_LEVELS][2] = {
		{ 1, 1 },
		{ 8, 8 },
		{ 16, 16 },
		{ 107, 115 }
	};

	chip->settings.als_gain = val;
	chip->settings.als_gain_adj0 = gainadj[val][0];
	chip->settings.als_gain_adj1 = gainadj[val][1];
}

/*
 * Provides initial operational parameter defaults.
 * These defaults may be changed through the device's sysfs files.
 */
static void taos_defaults(struct tsl258x_chip *chip)
{
	/* Operational parameters */

	/* must be a multiple of 50mS */
	chip->settings.als_time = 450;

	/* assume clear glass as default */
	taos_set_gain(chip, 2);

	/* default gain trim to account for aperture effects */
	chip->settings.als_gain_trim = 1000;

	/* Known external ALS reading used for calibration */
	chip->settings.als_cal_target = 130;

	/* Initialize ALS data to defaults */
	chip->cur_lux = 0;

	/* Default register settings */
	memcpy(chip->reg_cache, default_taos_config, sizeof(chip->reg_cache));
}

/*
 * Read a byte at register (reg) location.
 * Return byte read or negative values on failure
 */
static int taos_i2c_read(struct i2c_client *client, u8 reg)
{
	int ret;

	/* select register to write */
	ret = i2c_smbus_write_byte(client, (TSL258X_CMD_REG | reg));
	if (ret < 0) {
		dev_err(&client->dev, "taos_i2c_read failed to write"
			" register %x\n", reg);
		return ret;
	}
	/* read the data */
	ret = i2c_smbus_read_byte(client);

	return ret;
}

/*
 * This function is used to send a command to device command/control register
 * All bytes sent using this command have their MSBit set - it's a command!
 * Return 0, or i2c_smbus_write_byte error code.
 */
static int taos_i2c_write_command(struct i2c_client *client, u8 reg)
{
	int ret;

	/* write the data */
	ret = i2c_smbus_write_byte(client, (TSL258X_CMD_REG | reg));
	if (ret < 0)
		dev_err(&client->dev, "FAILED: i2c_smbus_write_byte\n");

	return ret;
}

/*
 * Reads and calculates current lux value.
 * The raw ch0 and ch1 values of the ambient light sensed in the last
 * integration cycle are read from the device.
 * Time scale factor array values are adjusted based on the integration time.
 * The raw values are multiplied by a scale factor, and device gain is obtained
 * using gain index. Limit checks are done next, then the ratio of a multiple
 * of ch1 value, to the ch0 value, is calculated. The array device_lux[]
 * is then scanned to find the first ratio value that is just
 * above the ratio we just calculated. The ch0 and ch1 multiplier constants in
 * the array are then used along with the time scale factor array values, to
 * calculate the lux.
 */
static int taos_get_lux(struct i2c_client *client)
{
	u32 ch0, ch1; /* separated ch0/ch1 data from device */
	u32 lux; /* raw lux calculated from device data */
	u32 ratio;
	u8 buf[5];
	struct taos_lux *p;
	struct tsl258x_chip *chip = i2c_get_clientdata(client);
	int i, ret;
	u32 ch0lux = 0;
	u32 ch1lux = 0;

	if (mutex_trylock(&chip->als_mutex) == 0) {
		dev_info(&client->dev, "taos_get_lux device is busy\n");
		return chip->cur_lux; /* busy, so return LAST VALUE */
	}

	if (chip->taos_chip_status != TAOS_CHIP_WORKING) {
		/* device is not enabled */
		dev_err(&client->dev, "taos_get_lux device is not enabled\n");
		ret = -ENODEV;
		goto out_unlock;
	}

	ret = taos_i2c_read(client, TSL258X_CMD_REG);
	if (ret < 0) {
		dev_err(&client->dev, "taos_get_lux failed to read CMD_REG\n");
		goto out_unlock;
	}
	/* is data new & valid */
	if (!(ret & TSL258X_STA_ADCINTR)) {
		dev_err(&client->dev, "taos_get_lux data not valid\n");
		ret = chip->cur_lux; /* return LAST VALUE */
		goto out_unlock;
	}

	for (i = 0; i < 4; i++) {
		int reg = TSL258X_CMD_REG | (TSL258X_ALS_CHAN0LO + i);
		ret = taos_i2c_read(client, reg);
		buf[i] = ret;
		if (ret < 0) {
			dev_err(&client->dev, "taos_get_lux failed to read"
				" register %x\n", reg);
			goto out_unlock;
		}
	}

	/* clear status, really interrupt status (interrupts are off), but
	 * we use the bit anyway */
	ret = taos_i2c_write_command(client,
		TSL258X_CMD_REG | TSL258X_CMD_SPL_FN | TSL258X_CMD_ALS_INTCLR);
	if (ret < 0) {
		dev_err(&client->dev,
		"taos_i2c_write_command failed in "
		"taos_get_lux, err = %d\n", ret);
		goto out_unlock; /* have no data, so return failure */
	}

	/* extract ALS/lux data */
	ch0 = (buf[1] << 8) | buf[0];
	ch1 = (buf[3] << 8) | buf[2];

	if ((ch0 >= chip->als_saturation) || (ch1 >= chip->als_saturation)) {
		lux = MAX_LUX;
		goto out_update;
	}

	if (ch0 == 0) {
		/* have no data, so return LAST VALUE */
		ret = chip->cur_lux = 0;
		goto out_unlock;
	}
	/* calculate ratio */
	ratio = (ch1 << 15) / ch0;

	/* Find the right lux entry based on ratio */
	p = chip->device_lux;
	while (p->ratio && p->ratio < ratio)
		p++;

	ch0lux = DIV_ROUND_UP((ch0 * p->ch0), chip->settings.als_gain_adj0);
	ch1lux = DIV_ROUND_UP((ch1 * p->ch1), chip->settings.als_gain_adj1);
	lux = ch0lux - ch1lux;

	/* note: lux is 31 bit max at this point */
	if (ch1lux > ch0lux) {
		dev_dbg(&client->dev, "No Data - Return last value\n");
		lux = 0;
		goto out_update;
	}

	/* adjust for active time scale */
	if (!chip->als_time_scale)
		lux = 0;
	else
		lux = DIV_ROUND_UP(lux, chip->als_time_scale);

	/* adjust for active gain scale */
	lux >>= 13; /* tables have factor of 8192 builtin for accuracy */
	lux = DIV_ROUND_UP(lux * chip->settings.als_gain_trim, 1000);

	if (lux > MAX_LUX)
		lux = MAX_LUX;

out_update:
	/* Update the structure with the latest VALID lux. */
	chip->cur_lux = lux;
	ret = lux;

out_unlock:
	mutex_unlock(&chip->als_mutex);
	return ret;
}

/*
 * Obtain single reading and calculate the als_gain_trim (later used
 * to derive actual lux).
 * Return updated gain_trim value.
 */
int taos_als_calibrate(struct i2c_client *client)
{
	struct tsl258x_chip *chip = i2c_get_clientdata(client);
	u8 reg_val;
	unsigned int gain_trim_val;
	int ret;
	int lux_val;

	ret = i2c_smbus_write_byte(client, (TSL258X_CMD_REG | TSL258X_CNTRL));
	if (ret < 0) {
		dev_err(&client->dev, "taos_als_calibrate failed to reach the"
			" CNTRL register, ret=%d\n", ret);
		return ret;
	}

	reg_val = i2c_smbus_read_byte(client);
	if ((reg_val & (TSL258X_CNTL_ADC_ENBL | TSL258X_CNTL_PWRON))
			!= (TSL258X_CNTL_ADC_ENBL | TSL258X_CNTL_PWRON)) {
		dev_err(&client->dev, "taos_als_calibrate failed because the"
			" device is not powered on with ADC enabled\n");
		return -ENODATA;
	}

	ret = i2c_smbus_write_byte(client, (TSL258X_CMD_REG | TSL258X_STATUS));
	if (ret < 0) {
		dev_err(&client->dev, "taos_als_calibrate failed to reach the"
			" STATUS register, ret=%d\n", ret);
		return ret;
	}
	reg_val = i2c_smbus_read_byte(client);

	if ((reg_val & TSL258X_STA_ADCVALID) != TSL258X_STA_ADCVALID) {
		dev_err(&client->dev, "taos_als_calibrate failed because the"
			" STATUS did not indicate ADC valid.\n");
		return -ENODATA;
	}
	lux_val = taos_get_lux(client);
	if (lux_val < 0) {
		dev_err(&client->dev, "taos_als_calibrate failed to get lux\n");
		return lux_val;
	}
	gain_trim_val = (unsigned int) (((chip->settings.als_cal_target)
			* chip->settings.als_gain_trim) / lux_val);

	dev_info(&client->dev, "settings.als_cal_target = %d\n"
		"settings.als_gain_trim = %d\nlux_val = %d\n",
		chip->settings.als_cal_target,
		chip->settings.als_gain_trim,
		lux_val);

	if ((gain_trim_val < 250) || (gain_trim_val > 4000)) {
		dev_err(&client->dev, "taos_als_calibrate failed because"
			"trim_val of %d is out of range\n", gain_trim_val);
		return -ENODATA;
	}
	chip->settings.als_gain_trim = (int) gain_trim_val;

	return (int) gain_trim_val;
}

/*
 * Turn the device on.
 * Configuration must be set before calling this function.
 */
static int taos_chip_on(struct i2c_client *client)
{
	int i;
	int ret = 0;
	int als_count;
	int als_time;
	struct tsl258x_chip *chip = i2c_get_clientdata(client);

	/* and make sure we're not already on */
	if (chip->taos_chip_status == TAOS_CHIP_WORKING) {
		dev_info(&client->dev, "device is already enabled\n");
		return -EBUSY;
	}

	/* determine als integration regster */
	als_count = DIV_ROUND_UP(chip->settings.als_time * 100, 270);

	/* ensure at least one cycle */
	if (als_count == 0)
		als_count = 1;

	/* convert back to time (encompasses overrides) */
	als_time = DIV_ROUND_UP(als_count * 27, 10);
	chip->reg_cache[TSL258X_ALS_TIME] = 256 - als_count;

	/* Set the gain based on settings struct */
	chip->reg_cache[TSL258X_GAIN] = chip->settings.als_gain;

	/* set globals re scaling and saturation */
	chip->als_saturation = als_count * 922; /* 90% of full scale */
	chip->als_time_scale = DIV_ROUND_UP(als_time, 50);

	/* SKATE Specific power-on / adc enable sequence
	 * Power on the device 1st.
	 */
	ret = i2c_smbus_write_byte_data(client, TSL258X_CMD_REG | TSL258X_CNTRL,
					TSL258X_CNTL_PWRON);
	if (ret < 0) {
		dev_err(&client->dev, "taos_chip_on failed on CNTRL reg.\n");
		return ret;
	}

	/* Use the following shadow copy for our delay before enabling ADC.
	 * Write all the registers.
	 */
	for (i = 0; i < TAOS_REG_MAX; i++) {
		ret = i2c_smbus_write_byte_data(client, TSL258X_CMD_REG + i,
						chip->reg_cache[i]);
		if (ret < 0) {
			dev_err(&client->dev,
				"taos_chip_on failed on reg %d.\n", i);
			return ret;
		}
	}

	/* According to spec, need to wait 3ms before enabling ADC */

	mdelay(3);
	/* NOW enable the ADC
	 * initialize the desired mode of operation
	 */
	ret = i2c_smbus_write_byte_data(client, TSL258X_CMD_REG | TSL258X_CNTRL,
					TSL258X_CNTL_PWRON |
					TSL258X_CNTL_ADC_ENBL);
	if (ret < 0) {
		dev_err(&client->dev, "taos_chip_on failed on 2nd CTRL reg.\n");
		return ret;
	}
	chip->taos_chip_status = TAOS_CHIP_WORKING;
	return ret; /* returns result of last i2cwrite */
}

/* Turn the device OFF. */
static int taos_chip_off(struct i2c_client *client)
{
	struct tsl258x_chip *chip = i2c_get_clientdata(client);
	int ret;

	chip->taos_chip_status = TAOS_CHIP_SLEEP;
	/* turn device off */
	ret = i2c_smbus_write_byte_data(client, TSL258X_CMD_REG | TSL258X_CNTRL,
					TSL258X_CNTL_REG_CLEAR);

	return ret;
}

/* Sysfs Interface Functions */
static ssize_t taos_device_id(struct device *dev,
    struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", DEVICE_ID);
}

static ssize_t taos_power_state_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct tsl258x_chip *chip = indio_dev->dev_data;

	return sprintf(buf, "%d\n", chip->taos_chip_status);
}

static ssize_t taos_power_state_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct tsl258x_chip *chip = indio_dev->dev_data;
	unsigned long value;

	if (strict_strtoul(buf, 0, &value))
		return -EINVAL;

	if (value == 0)
		taos_chip_off(chip->client);
	else
		taos_chip_on(chip->client);

	return len;
}

static ssize_t taos_gain_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct tsl258x_chip *chip = indio_dev->dev_data;

	return sprintf(buf, "%d\n", chip->settings.als_gain);
}

static ssize_t taos_gain_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct tsl258x_chip *chip = indio_dev->dev_data;
	unsigned long value;
	if (strict_strtoul(buf, 0, &value))
		return -EINVAL;
	if (value) {
		if (value > NR_GAIN_LEVELS) {
			printk(KERN_INFO "Invalid Gain Index\n");
			return -EINVAL;
		}
		taos_set_gain(chip, value);
	}
	return len;
}

static ssize_t taos_als_time_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct tsl258x_chip *chip = indio_dev->dev_data;

	return sprintf(buf, "%d\n", chip->settings.als_time);
}

static ssize_t taos_als_time_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct tsl258x_chip *chip = indio_dev->dev_data;
	unsigned long value;

	if (strict_strtoul(buf, 0, &value))
		return -EINVAL;

	if (value)
		chip->settings.als_time = value;

	return len;
}

static ssize_t taos_als_trim_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct tsl258x_chip *chip = indio_dev->dev_data;

	return sprintf(buf, "%d\n", chip->settings.als_gain_trim);
}

static ssize_t taos_als_trim_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct tsl258x_chip *chip = indio_dev->dev_data;
	unsigned long value;

	if (strict_strtoul(buf, 0, &value))
		return -EINVAL;

	if (value)
		chip->settings.als_gain_trim = value;

	return len;
}

static ssize_t taos_als_cal_target_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct tsl258x_chip *chip = indio_dev->dev_data;

	return sprintf(buf, "%d\n", chip->settings.als_cal_target);
}

static ssize_t taos_als_cal_target_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct tsl258x_chip *chip = indio_dev->dev_data;
	unsigned long value;

	if (strict_strtoul(buf, 0, &value))
		return -EINVAL;

	if (value)
		chip->settings.als_cal_target = value;

	return len;
}

static ssize_t taos_lux_show(struct device *dev, struct device_attribute *attr,
    char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct tsl258x_chip *chip = indio_dev->dev_data;
	int lux = 0;

	lux = taos_get_lux(chip->client);

	return sprintf(buf, "%d\n", lux);
}

static ssize_t taos_do_calibrate(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct tsl258x_chip *chip = indio_dev->dev_data;
	unsigned long value;

	if (strict_strtoul(buf, 0, &value))
		return -EINVAL;

	if (value == 1)
		taos_als_calibrate(chip->client);

	return len;
}

static ssize_t taos_luxtable_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct tsl258x_chip *chip = indio_dev->dev_data;
	int i;
	int offset = 0;

	for (i = 0; i < ARRAY_SIZE(chip->device_lux); i++) {
		offset += sprintf(buf + offset, "%d,%d,%d,",
				  chip->device_lux[i].ratio,
				  chip->device_lux[i].ch0,
				  chip->device_lux[i].ch1);
		if (chip->device_lux[i].ratio == 0) {
			/* We just printed the first "0" entry.
			 * Now get rid of the extra "," and break.
			 */
			offset--;
			break;
		}
	}

	offset += sprintf(buf + offset, "\n");
	return offset;
}

static ssize_t taos_luxtable_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct tsl258x_chip *chip = indio_dev->dev_data;
	int value[MAX_LUX_TABLE_LEVELS * 3 + 1];
	int i, n;

	memset(value, 0, sizeof(value));
	get_options(buf, ARRAY_SIZE(value), value);

	/* We now have an array of ints starting at value[1], and
	 * enumerated by value[0].
	 * We expect each group of three ints is one table entry,
	 * and the last table entry is all 0.
	 */
	n = value[0];
	if ((n % 3) || n < 6 || n > (MAX_LUX_TABLE_LEVELS * 3)) {
		dev_dbg(dev, "invalid lux table size: %d\n", n);
		return -EINVAL;
	}
	if (value[(n - 2)] || value[(n - 1)] || value[n]) {
		dev_dbg(dev, "invalid lux table termination\n");
		return -EINVAL;
	}

	if (chip->taos_chip_status == TAOS_CHIP_WORKING)
		taos_chip_off(chip->client);

	/* Fill lux table from value array */
	for (i = 0; i < MAX_LUX_TABLE_LEVELS; i++) {
		chip->device_lux[i].ratio = value[1 + i*3];
		chip->device_lux[i].ch0 = value[1 + i*3 + 1];
		chip->device_lux[i].ch1 = value[1 + i*3 + 2];
	}

	taos_chip_on(chip->client);

	return len;
}

static DEVICE_ATTR(device_id, S_IRUGO, taos_device_id, NULL);
static DEVICE_ATTR(device_state, S_IRUGO | S_IWUSR,
		taos_power_state_show, taos_power_state_store);
static DEVICE_ATTR(als_gain, S_IRUGO | S_IWUSR,
		taos_gain_show, taos_gain_store);
static DEVICE_ATTR(als_time, S_IRUGO | S_IWUSR,
		taos_als_time_show, taos_als_time_store);
static DEVICE_ATTR(als_trim, S_IRUGO | S_IWUSR,
		taos_als_trim_show, taos_als_trim_store);
static DEVICE_ATTR(als_target, S_IRUGO | S_IWUSR,
		taos_als_cal_target_show, taos_als_cal_target_store);
static DEVICE_ATTR(lux, S_IRUGO, taos_lux_show, NULL);
static DEVICE_ATTR(calibrate, S_IWUSR, NULL, taos_do_calibrate);
static DEVICE_ATTR(lux_table, S_IRUGO | S_IWUSR,
		taos_luxtable_show, taos_luxtable_store);

static struct attribute *sysfs_attrs_ctrl[] = {
	&dev_attr_device_id.attr,
	&dev_attr_device_state.attr,
	&dev_attr_als_gain.attr,
	&dev_attr_als_time.attr,
	&dev_attr_als_trim.attr,
	&dev_attr_als_target.attr,
	&dev_attr_lux.attr,
	&dev_attr_calibrate.attr,
	&dev_attr_lux_table.attr,
	NULL
};

static struct attribute_group tsl258x_attribute_group = {
	.attrs = sysfs_attrs_ctrl,
};

/* Use the default register values to identify the Taos device */
static int taos_skate_device(struct i2c_client *c)
{
	int ret;
	u8 reg;

	ret = i2c_smbus_write_byte(c, (TSL258X_CMD_REG | TSL258X_CHIPID));

	if (ret < 0)
		return ret;

	reg = i2c_smbus_read_byte(c);

	/* "skateFN" */
	if ((reg & 0xf0) == 0x90)
		return 1;

	return 0;
}

/*
 * Client probe function - When a valid device is found, the driver's device
 * data structure is updated, and initialization completes successfully.
 */
static int __devinit taos_probe(struct i2c_client *clientp,
		      const struct i2c_device_id *idp)
{
	int ret = 0;
	static struct tsl258x_chip *chip;

	if (!i2c_check_functionality(clientp->adapter,
		I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&clientp->dev,
			"taos_probe() - i2c smbus byte data "
			"functions unsupported\n");
		return -EOPNOTSUPP;
	}
	if (!i2c_check_functionality(clientp->adapter,
		I2C_FUNC_SMBUS_WORD_DATA)) {
		dev_warn(&clientp->dev,
			"taos_probe() - i2c smbus word data "
			"functions unsupported\n");
	}
	if (!i2c_check_functionality(clientp->adapter,
		I2C_FUNC_SMBUS_BLOCK_DATA)) {
		dev_err(&clientp->dev,
			"taos_probe() - i2c smbus block data "
			"functions unsupported\n");
	}

	chip = kmalloc(sizeof(struct tsl258x_chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&clientp->dev, "taos_device_drvr: kmalloc for "
			"struct tsl258x_chip failed in taos_probe()\n");
		return -ENOMEM;
	}
	memset(chip, 0, sizeof(struct tsl258x_chip));
	chip->client = clientp;
	i2c_set_clientdata(clientp, chip);

	mutex_init(&chip->als_mutex);
	chip->taos_chip_status = TAOS_CHIP_UNKNOWN; /* uninitialized to start */

	ret = taos_skate_device(clientp);
	if (ret < 0) {
		dev_dbg(&clientp->dev, "probe failed: %d\n", ret);
		goto fail1;
	}

	memcpy(chip->device_lux, default_device_lux, sizeof(default_device_lux));

	dev_info(&clientp->dev, "skate device found\n");
	/* Load up the V2 defaults (these are hard coded defaults for now) */
	taos_defaults(chip);

	ret = i2c_smbus_write_byte(clientp, (TSL258X_CMD_REG | TSL258X_CNTRL));
	if (ret < 0) {
		dev_err(&clientp->dev, "i2c_smbus_write_byte() to cmd reg "
			"failed in taos_probe(), err = %d\n", ret);
		goto fail1;
	}

	chip->iio_dev = iio_allocate_device();
	if (!chip->iio_dev) {
		ret = -ENOMEM;
		dev_info(&clientp->dev, "iio allocation failed\n");
		goto fail1;
	}

	chip->iio_dev->attrs = &tsl258x_attribute_group;
	chip->iio_dev->dev.parent = &clientp->dev;
	chip->iio_dev->dev_data = (void *)(chip);
	chip->iio_dev->driver_module = THIS_MODULE;
	chip->iio_dev->modes = INDIO_DIRECT_MODE;
	ret = iio_device_register(chip->iio_dev);
	if (ret) {
		dev_info(&clientp->dev, "iio registration failed: %d\n", ret);
		goto fail1;
	}

	/* Make sure the chip is on */
	taos_chip_on(clientp);

	return 0;

fail1:
	kfree(chip);

	return ret;
}

static int __devexit taos_remove(struct i2c_client *client)
{
	struct tsl258x_chip *chip = i2c_get_clientdata(client);

	if (chip->taos_chip_status == TAOS_CHIP_WORKING)
		taos_chip_off(client);

	iio_device_unregister(chip->iio_dev);

	kfree(chip);
	return 0;
}

static int taos_suspend(struct i2c_client *client, pm_message_t state)
{
	struct tsl258x_chip *chip = i2c_get_clientdata(client);
	int ret = 0;

	mutex_lock(&chip->als_mutex);

	if (chip->taos_chip_status == TAOS_CHIP_WORKING) {
		ret = taos_chip_off(client);
		chip->taos_chip_status = TAOS_CHIP_SUSPENDED;
	}

	mutex_unlock(&chip->als_mutex);
	return ret;
}

static int taos_resume(struct i2c_client *client)
{
	struct tsl258x_chip *chip = i2c_get_clientdata(client);
	int ret = 0;

	mutex_lock(&chip->als_mutex);

	if (chip->taos_chip_status == TAOS_CHIP_SUSPENDED)
		ret = taos_chip_on(client);

	mutex_unlock(&chip->als_mutex);
	return ret;
}

static struct i2c_device_id taos_idtable[] = {
	{ DEVICE_ID, 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, taos_idtable);

/* Driver definition */
static struct i2c_driver taos_driver = {
	.driver = {
		.name = "tsl258x",
	},
	.id_table	= taos_idtable,
	.suspend	= taos_suspend,
	.resume		= taos_resume,
	.probe		= taos_probe,
	.remove		= __devexit_p(taos_remove),
};

static int __init taos_init(void)
{
	return i2c_add_driver(&taos_driver);
}

static void __exit taos_exit(void)
{
	i2c_del_driver(&taos_driver);
}

MODULE_AUTHOR("J. August Brenner<jbrenner@taosinc.com>");
MODULE_DESCRIPTION("TAOS 258x ambient light sensor driver");
MODULE_LICENSE("GPL");

module_init(taos_init);
module_exit(taos_exit);
