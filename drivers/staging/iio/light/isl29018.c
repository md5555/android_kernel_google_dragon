/*
 * A iio driver for the light sensor ISL 29018/29023/29035.
 *
 * IIO driver for monitoring ambient light intensity in luxi, proximity
 * sensing and infrared sensing.
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
 * 51 Franklin Street, Fifth Floor, Boston, MA	02110-1301, USA.
 */

#include <linux/acpi.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/jiffies.h>
#include <linux/slab.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/acpi.h>

#define CONVERSION_TIME_MS		200

#define ISL29018_REG_ADD_COMMAND1	0x00
#define COMMMAND1_OPMODE_SHIFT		5
#define COMMMAND1_OPMODE_MASK		(7 << COMMMAND1_OPMODE_SHIFT)
#define COMMMAND1_OPMODE_POWER_DOWN	0
#define COMMMAND1_OPMODE_ALS_ONCE	1
#define COMMMAND1_OPMODE_IR_ONCE	2
#define COMMMAND1_OPMODE_PROX_ONCE	3
#define COMMMAND1_OPMODE_ALS_CONT	5
#define COMMMAND1_OPMODE_IR_CONT	6
#define COMMMAND1_OPMODE_PROX_CONT	7
#define OPMODE_CONT_OFFSET		4

#define ISL29018_REG_ADD_COMMANDII	0x01
#define COMMANDII_RESOLUTION_SHIFT	2
#define COMMANDII_RESOLUTION_MASK	(0x3 << COMMANDII_RESOLUTION_SHIFT)

#define COMMANDII_RANGE_SHIFT		0
#define COMMANDII_RANGE_MASK		(0x3 << COMMANDII_RANGE_SHIFT)

#define COMMANDII_SCHEME_SHIFT		7
#define COMMANDII_SCHEME_MASK		(0x1 << COMMANDII_SCHEME_SHIFT)

#define ISL29018_REG_ADD_DATA_LSB	0x02
#define ISL29018_REG_ADD_DATA_MSB	0x03

#define ISL29018_REG_TEST		0x08
#define ISL29018_TEST_SHIFT		0
#define ISL29018_TEST_MASK		(0xFF << ISL29018_TEST_SHIFT)

#define ISL29035_REG_DEVICE_ID		0x0F
#define ISL29035_DEVICE_ID_SHIFT	0x03
#define ISL29035_DEVICE_ID_MASK		(0x7 << ISL29035_DEVICE_ID_SHIFT)
#define ISL29035_DEVICE_ID		0x5
#define ISL29035_BOUT_SHIFT		0x07
#define ISL29035_BOUT_MASK		(0x01 << ISL29035_BOUT_SHIFT)

/*
 * Indexing into the isl29018_cont_modes array involves calculating from
 * COMMMAND1_OPMODE_xxx values.
 */
#define CONT_MODE_ONCE			0
#define CONT_MODE_ALS			1
#define CONT_MODE_IR			2
#define CONT_MODE_PROX			3

static const char const *isl29018_cont_modes[] = {
	"once",
	"als",
	"ir",
	"prox",
};

struct isl29018_chip {
	struct device		*dev;
	struct regmap		*regmap;
	struct mutex		lock;
	int			type;
	unsigned int		lux_scale;
	unsigned int		lux_uscale;
	unsigned int		ir_comp_scale;
	unsigned int		ir_comp_uscale;
	unsigned int		range;
	unsigned int		adc_bit;
	unsigned long		opmode_trans_time;
	int			prox_scheme;
	int			default_opmode;
	bool			suspended;
};

static int isl29018_set_range(struct isl29018_chip *chip, unsigned long range,
		unsigned int *new_range)
{
	static const unsigned long supp_ranges[] = {1000, 4000, 16000, 64000};
	int i;

	for (i = 0; i < ARRAY_SIZE(supp_ranges); ++i) {
		if (range <= supp_ranges[i]) {
			*new_range = (unsigned int)supp_ranges[i];
			break;
		}
	}

	if (i >= ARRAY_SIZE(supp_ranges))
		return -EINVAL;

	return regmap_update_bits(chip->regmap, ISL29018_REG_ADD_COMMANDII,
			COMMANDII_RANGE_MASK, i << COMMANDII_RANGE_SHIFT);
}

static int isl29018_set_resolution(struct isl29018_chip *chip,
			unsigned long adcbit, unsigned int *conf_adc_bit)
{
	static const unsigned long supp_adcbit[] = {16, 12, 8, 4};
	int i;

	for (i = 0; i < ARRAY_SIZE(supp_adcbit); ++i) {
		if (adcbit >= supp_adcbit[i]) {
			*conf_adc_bit = (unsigned int)supp_adcbit[i];
			break;
		}
	}

	if (i >= ARRAY_SIZE(supp_adcbit))
		return -EINVAL;

	return regmap_update_bits(chip->regmap, ISL29018_REG_ADD_COMMANDII,
			COMMANDII_RESOLUTION_MASK,
			i << COMMANDII_RESOLUTION_SHIFT);
}

static int isl29018_set_opmode(struct isl29018_chip *chip, int mode)
{
	int current_mode;
	int status;

	status = regmap_read(chip->regmap, ISL29018_REG_ADD_COMMAND1,
			&current_mode);
	if (status < 0) {
		dev_err(chip->dev,
			"Error in reading current mode with err %d\n", status);
	}
	current_mode &= COMMMAND1_OPMODE_MASK;
	current_mode >>= COMMMAND1_OPMODE_SHIFT;
	if (current_mode == mode &&
			(mode == COMMMAND1_OPMODE_ALS_CONT ||
			 mode == COMMMAND1_OPMODE_IR_CONT ||
			 mode == COMMMAND1_OPMODE_PROX_CONT))
		return 0;

	status = regmap_write(chip->regmap, ISL29018_REG_ADD_COMMAND1,
			mode << COMMMAND1_OPMODE_SHIFT);
	if (status) {
		dev_err(chip->dev,
			"Error in setting operating mode err %d\n", status);
		return status;
	}

	chip->opmode_trans_time = jiffies_to_msecs(jiffies);

	return 0;
}

static int isl29018_opmode(struct isl29018_chip *chip, int opmode)
{
	if (chip->default_opmode == COMMMAND1_OPMODE_POWER_DOWN)
		return opmode;

	return opmode + OPMODE_CONT_OFFSET;
}

static int isl29018_restore_default_opmode(struct isl29018_chip *chip)
{
	/* Chip will automatically power down if this is the default opmode */
	if (chip->default_opmode == COMMMAND1_OPMODE_POWER_DOWN)
		return 0;

	return isl29018_set_opmode(chip, chip->default_opmode);
}

static int isl29018_read_sensor_input(struct isl29018_chip *chip, int mode)
{
	int status;
	int lsb;
	int msb;
	unsigned long diff;

	status = isl29018_set_opmode(chip, mode);
	if (status)
		return status;

	diff = jiffies_to_msecs(jiffies) - chip->opmode_trans_time;
	if (diff < CONVERSION_TIME_MS)
		msleep(CONVERSION_TIME_MS - diff);

	status = regmap_read(chip->regmap, ISL29018_REG_ADD_DATA_LSB, &lsb);
	if (status < 0) {
		dev_err(chip->dev,
			"Error in reading LSB DATA with err %d\n", status);
		return status;
	}

	status = regmap_read(chip->regmap, ISL29018_REG_ADD_DATA_MSB, &msb);
	if (status < 0) {
		dev_err(chip->dev,
			"Error in reading MSB DATA with error %d\n", status);
		return status;
	}
	dev_vdbg(chip->dev, "MSB 0x%x and LSB 0x%x\n", msb, lsb);

	return (msb << 8) | lsb;
}

/* Reads lux with IR compensation */
static int isl29018_read_lux(struct isl29018_chip *chip, int *lux)
{
	int lux_data, ir_data;
	unsigned int data_x_range;
	/* Use 64 bits to prevent overflow */
	u64 lux_unshifted, ir_unshifted;

	lux_data = isl29018_read_sensor_input(chip, isl29018_opmode(chip,
				COMMMAND1_OPMODE_ALS_ONCE));
	if (lux_data < 0)
		return lux_data;

	/* To support fractional scaling, separate the unshifted lux
	 * into two calculations: int scaling and micro-scaling.
	 * lux_uscale ranges from 0-999999, so about 20 bits.  Split
	 * the /1,000,000 in two to reduce the risk of over/underflow.
	 */
	data_x_range = lux_data * chip->range;
	/* Cast to 64 bit before multiplying so we don't get overflow */
	lux_unshifted = (u64)data_x_range * chip->lux_scale;
	lux_unshifted += data_x_range / 1000 * (chip->lux_uscale / 1000);
	lux_data = lux_unshifted >> chip->adc_bit;

	if (!chip->ir_comp_scale && !chip->ir_comp_uscale) {
		*lux = lux_data;
		return 0;
	}

	ir_data = isl29018_read_sensor_input(chip, isl29018_opmode(chip,
				COMMMAND1_OPMODE_IR_ONCE));
	if (ir_data < 0)
		return ir_data;

	/*
	 * The function for lux with IR compensation is
	 * lux' = lux * lux_scale + ir * ir_scale
	 *
	 * Since lux * lux_scale was done above, we just need to do the
	 * same for ir.  As above, split the /1,000,000 in two to reduce
	 * the risk of over/underflow.
	 */
	data_x_range = ir_data * chip->range;
	ir_unshifted = (u64)data_x_range * chip->ir_comp_scale;
	ir_unshifted += data_x_range / 1000 * (chip->ir_comp_uscale / 1000);
	ir_data = ir_unshifted >> chip->adc_bit;
	*lux = lux_data + ir_data;

	return 0;
}

static int isl29018_read_ir(struct isl29018_chip *chip, int *ir)
{
	int ir_data;

	ir_data = isl29018_read_sensor_input(chip, isl29018_opmode(chip,
				COMMMAND1_OPMODE_IR_ONCE));

	if (ir_data < 0)
		return ir_data;

	*ir = ir_data;

	return 0;
}

static int isl29018_read_proximity_ir(struct isl29018_chip *chip, int scheme,
		int *near_ir)
{
	int status;
	int prox_data = -1;
	int ir_data = -1;

	/* Do proximity sensing with required scheme */
	status = regmap_update_bits(chip->regmap, ISL29018_REG_ADD_COMMANDII,
			COMMANDII_SCHEME_MASK,
			scheme << COMMANDII_SCHEME_SHIFT);
	if (status) {
		dev_err(chip->dev, "Error in setting operating mode\n");
		return status;
	}

	prox_data = isl29018_read_sensor_input(chip, isl29018_opmode(chip,
				COMMMAND1_OPMODE_PROX_ONCE));
	if (prox_data < 0)
		return prox_data;

	if (scheme == 1) {
		*near_ir = prox_data;
		return 0;
	}

	ir_data = isl29018_read_sensor_input(chip, isl29018_opmode(chip,
				COMMMAND1_OPMODE_IR_ONCE));

	if (ir_data < 0)
		return ir_data;

	if (prox_data >= ir_data)
		*near_ir = prox_data - ir_data;
	else
		*near_ir = 0;

	return 0;
}

/* Sysfs interface */
/* lux_scale */
static ssize_t show_lux_scale(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct isl29018_chip *chip = iio_priv(indio_dev);

	return sprintf(buf, "%d\n", chip->lux_scale);
}

static ssize_t store_lux_scale(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct isl29018_chip *chip = iio_priv(indio_dev);
	unsigned long lval;

	lval = simple_strtoul(buf, NULL, 10);
	if (lval == 0)
		return -EINVAL;

	mutex_lock(&chip->lock);
	chip->lux_scale = lval;
	mutex_unlock(&chip->lock);

	return count;
}

static ssize_t get_sensor_data(struct device *dev, char *buf, int mode)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct isl29018_chip *chip = iio_priv(indio_dev);
	int value = 0;
	int status;

	mutex_lock(&chip->lock);
	switch (mode) {
	case COMMMAND1_OPMODE_PROX_ONCE:
		status = isl29018_read_proximity_ir(chip,
						chip->prox_scheme, &value);
	break;

	case COMMMAND1_OPMODE_ALS_ONCE:
		status = isl29018_read_lux(chip, &value);
		break;

	case COMMMAND1_OPMODE_IR_ONCE:
		status = isl29018_read_ir(chip, &value);
		break;

	default:
		dev_err(dev, "Mode %d is not supported\n", mode);
		mutex_unlock(&chip->lock);
		return -EBUSY;
	}

	isl29018_restore_default_opmode(chip);
	if (status < 0) {
		dev_err(dev, "Error in Reading data");
		mutex_unlock(&chip->lock);
		return status;
	}

	mutex_unlock(&chip->lock);

	return sprintf(buf, "%d\n", value);
}

static ssize_t show_continuous_available(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	char *start = buf;
	int i;

	for (i = 0; i < ARRAY_SIZE(isl29018_cont_modes); i++)
		buf += sprintf(buf, "%s ", isl29018_cont_modes[i]);

	buf += sprintf(buf, "\n");
	return buf - start;
}

/* Read operation mode */
static ssize_t show_continuous(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct isl29018_chip *chip = iio_priv(indio_dev);
	int mode = CONT_MODE_ONCE;

	if (chip->default_opmode != COMMMAND1_OPMODE_POWER_DOWN)
		mode = chip->default_opmode - OPMODE_CONT_OFFSET;

	return sprintf(buf, "%s\n", isl29018_cont_modes[mode]);
}

/* Store operation mode */
static ssize_t store_continuous(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct isl29018_chip *chip = iio_priv(indio_dev);
	ssize_t status;
	int opmode;
	int mode;

	mutex_lock(&chip->lock);
	for (mode = 0; mode < ARRAY_SIZE(isl29018_cont_modes); mode++) {
		if (sysfs_streq(isl29018_cont_modes[mode], buf))
			break;
	}

	if (mode == CONT_MODE_ONCE)
		opmode = COMMMAND1_OPMODE_POWER_DOWN;
	else if (mode < ARRAY_SIZE(isl29018_cont_modes))
		opmode = mode + OPMODE_CONT_OFFSET;
	else {
		status = -EINVAL;
		goto out;
	}

	status = isl29018_set_opmode(chip, opmode);
	if (status)
		goto out;

	chip->default_opmode = opmode;
	status = count;
out:
	mutex_unlock(&chip->lock);
	return status;
}

/* Read lux */
static ssize_t show_lux(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	return get_sensor_data(dev, buf, COMMMAND1_OPMODE_ALS_ONCE);
}

/* Read ir */
static ssize_t show_ir(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	return get_sensor_data(dev, buf, COMMMAND1_OPMODE_IR_ONCE);
}

/* Read nearest ir */
static ssize_t show_proxim_ir(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	return get_sensor_data(dev, buf, COMMMAND1_OPMODE_PROX_ONCE);
}

/* range */
static ssize_t show_range(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct isl29018_chip *chip = iio_priv(indio_dev);

	return sprintf(buf, "%u\n", chip->range);
}

static ssize_t store_range(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct isl29018_chip *chip = iio_priv(indio_dev);
	int status;
	unsigned long lval;
	unsigned int new_range;

	if (kstrtoul(buf, 10, &lval))
		return -EINVAL;

	if (!(lval == 1000UL || lval == 4000UL ||
			lval == 16000UL || lval == 64000UL)) {
		dev_err(dev, "The range is not supported\n");
		return -EINVAL;
	}

	mutex_lock(&chip->lock);
	status = isl29018_set_range(chip, lval, &new_range);
	if (status < 0) {
		mutex_unlock(&chip->lock);
		dev_err(dev,
			"Error in setting max range with err %d\n", status);
		return status;
	}
	chip->range = new_range;
	mutex_unlock(&chip->lock);

	return count;
}

/* resolution */
static ssize_t show_resolution(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct isl29018_chip *chip = iio_priv(indio_dev);

	return sprintf(buf, "%u\n", chip->adc_bit);
}

static ssize_t store_resolution(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct isl29018_chip *chip = iio_priv(indio_dev);
	int status;
	unsigned int val;
	unsigned int new_adc_bit;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (!(val == 4 || val == 8 || val == 12 || val == 16)) {
		dev_err(dev, "The resolution is not supported\n");
		return -EINVAL;
	}

	mutex_lock(&chip->lock);
	status = isl29018_set_resolution(chip, val, &new_adc_bit);
	if (status < 0) {
		mutex_unlock(&chip->lock);
		dev_err(dev, "Error in setting resolution\n");
		return status;
	}
	chip->adc_bit = new_adc_bit;
	mutex_unlock(&chip->lock);

	return count;
}

/* proximity scheme */
static ssize_t show_prox_infrared_suppression(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct isl29018_chip *chip = iio_priv(indio_dev);

	/* return the "proximity scheme" i.e. if the chip does on chip
	infrared suppression (1 means perform on chip suppression) */
	return sprintf(buf, "%d\n", chip->prox_scheme);
}

static ssize_t store_prox_infrared_suppression(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct isl29018_chip *chip = iio_priv(indio_dev);
	int val;

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;
	if (!(val == 0 || val == 1)) {
		dev_err(dev, "The mode is not supported\n");
		return -EINVAL;
	}

	/* get the  "proximity scheme" i.e. if the chip does on chip
	infrared suppression (1 means perform on chip suppression) */
	mutex_lock(&chip->lock);
	chip->prox_scheme = val;
	mutex_unlock(&chip->lock);

	return count;
}

/* Channel IO */
static int isl29018_write_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      int val,
			      int val2,
			      long mask)
{
	struct isl29018_chip *chip = iio_priv(indio_dev);
	int ret = -EINVAL;

	mutex_lock(&chip->lock);
	if (mask == IIO_CHAN_INFO_CALIBSCALE && chan->type == IIO_LIGHT) {
		/* With no write_raw_get_fmt(), val2 is a MICRO fraction. */
		if (chan->channel == 0) {
			chip->lux_scale = val;
			chip->lux_uscale = val2;
		} else {
			chip->ir_comp_scale = val;
			chip->ir_comp_uscale = val2;
		}
		ret = 0;
	}
	mutex_unlock(&chip->lock);

	return ret;
}

static int isl29018_read_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int *val,
			     int *val2,
			     long mask)
{
	int ret = -EINVAL;
	struct isl29018_chip *chip = iio_priv(indio_dev);

	mutex_lock(&chip->lock);
	if (chip->suspended) {
		mutex_unlock(&chip->lock);
		return -EBUSY;
	}
	switch (mask) {
	case IIO_CHAN_INFO_RAW:
	case IIO_CHAN_INFO_PROCESSED:
		switch (chan->type) {
		case IIO_LIGHT:
			ret = isl29018_read_lux(chip, val);
			isl29018_restore_default_opmode(chip);
			break;
		case IIO_INTENSITY:
			ret = isl29018_read_ir(chip, val);
			isl29018_restore_default_opmode(chip);
			break;
		case IIO_PROXIMITY:
			ret = isl29018_read_proximity_ir(chip,
					chip->prox_scheme, val);
			isl29018_restore_default_opmode(chip);
			break;
		default:
			break;
		}
		if (!ret)
			ret = IIO_VAL_INT;
		break;
	case IIO_CHAN_INFO_CALIBSCALE:
		if (chan->type != IIO_LIGHT)
			break;
		if (chan->channel == 0) {
			*val = chip->lux_scale;
			*val2 = chip->lux_uscale;
		} else {
			*val = chip->ir_comp_scale;
			*val2 = chip->ir_comp_uscale;
		}
		ret = IIO_VAL_INT_PLUS_MICRO;
		break;
	default:
		break;
	}
	mutex_unlock(&chip->lock);
	return ret;
}

#define ISL29018_LIGHT_CHANNEL(ch) {					\
	.type = IIO_LIGHT,						\
	.indexed = 1,							\
	.channel = ch,							\
	.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED) |		\
	BIT(IIO_CHAN_INFO_CALIBSCALE),					\
}

#define ISL29018_IR_CHANNEL {						\
	.type = IIO_INTENSITY,						\
	.modified = 1,							\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),			\
	.channel2 = IIO_MOD_LIGHT_IR,					\
}

#define ISL29018_PROXIMITY_CHANNEL {					\
	.type = IIO_PROXIMITY,						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),			\
}

static const struct iio_chan_spec isl29018_channels[] = {
	ISL29018_LIGHT_CHANNEL(0),
	ISL29018_LIGHT_CHANNEL(1),
	ISL29018_IR_CHANNEL,
	ISL29018_PROXIMITY_CHANNEL,
};

static const struct iio_chan_spec isl29023_channels[] = {
	ISL29018_LIGHT_CHANNEL(0),
	ISL29018_LIGHT_CHANNEL(1),
	ISL29018_IR_CHANNEL,
};

static IIO_DEVICE_ATTR(range, S_IRUGO | S_IWUSR, show_range, store_range, 0);
static IIO_CONST_ATTR(range_available, "1000 4000 16000 64000");
static IIO_DEVICE_ATTR(continuous_available, S_IRUGO, show_continuous_available,
					NULL, 0);
static IIO_DEVICE_ATTR(continuous, S_IRUGO | S_IWUSR, show_continuous,
					store_continuous, 0);
static IIO_CONST_ATTR(adc_resolution_available, "4 8 12 16");
static IIO_DEVICE_ATTR(adc_resolution, S_IRUGO | S_IWUSR,
					show_resolution, store_resolution, 0);
static IIO_DEVICE_ATTR(proximity_on_chip_ambient_infrared_suppression,
					S_IRUGO | S_IWUSR,
					show_prox_infrared_suppression,
					store_prox_infrared_suppression, 0);
static IIO_DEVICE_ATTR(illuminance0_input, S_IRUGO, show_lux, NULL, 0);
static IIO_DEVICE_ATTR(illuminance0_calibscale, S_IRUGO | S_IWUSR,
					show_lux_scale, store_lux_scale, 0);
static IIO_DEVICE_ATTR(intensity_infrared_raw, S_IRUGO, show_ir, NULL, 0);
static IIO_DEVICE_ATTR(proximity_raw, S_IRUGO, show_proxim_ir, NULL, 0);

#define ISL29018_DEV_ATTR(name) (&iio_dev_attr_##name.dev_attr.attr)
#define ISL29018_CONST_ATTR(name) (&iio_const_attr_##name.dev_attr.attr)
static struct attribute *isl29018_attributes[] = {
	ISL29018_DEV_ATTR(range),
	ISL29018_CONST_ATTR(range_available),
	ISL29018_DEV_ATTR(continuous_available),
	ISL29018_DEV_ATTR(continuous),
	ISL29018_DEV_ATTR(adc_resolution),
	ISL29018_CONST_ATTR(adc_resolution_available),
	ISL29018_DEV_ATTR(proximity_on_chip_ambient_infrared_suppression),
	ISL29018_DEV_ATTR(illuminance0_input),
	ISL29018_DEV_ATTR(illuminance0_calibscale),
	ISL29018_DEV_ATTR(intensity_infrared_raw),
	ISL29018_DEV_ATTR(proximity_raw),
	NULL
};

static struct attribute *isl29023_attributes[] = {
	ISL29018_DEV_ATTR(range),
	ISL29018_CONST_ATTR(range_available),
	ISL29018_DEV_ATTR(adc_resolution),
	ISL29018_CONST_ATTR(adc_resolution_available),
	NULL
};

static const struct attribute_group isl29018_group = {
	.attrs = isl29018_attributes,
};

static const struct attribute_group isl29023_group = {
	.attrs = isl29023_attributes,
};

static int isl29035_detect(struct isl29018_chip *chip)
{
	int status;
	unsigned int id;

	status = regmap_read(chip->regmap, ISL29035_REG_DEVICE_ID, &id);
	if (status < 0) {
		dev_err(chip->dev,
			"Error reading ID register with error %d\n",
			status);
		return status;
	}

	id = (id & ISL29035_DEVICE_ID_MASK) >> ISL29035_DEVICE_ID_SHIFT;

	if (id != ISL29035_DEVICE_ID)
		return -ENODEV;

	/* clear out brownout bit */
	return regmap_update_bits(chip->regmap, ISL29035_REG_DEVICE_ID,
				  ISL29035_BOUT_MASK, 0);
}

enum {
	isl29018,
	isl29023,
	isl29035,
};

static int isl29018_chip_init(struct isl29018_chip *chip)
{
	int status;
	unsigned int new_adc_bit;
	unsigned int new_range;

	if (chip->type == isl29035) {
		status = isl29035_detect(chip);
		if (status < 0)
			return status;
	}

	/* Code added per Intersil Application Note 1534:
	 *     When VDD sinks to approximately 1.8V or below, some of
	 * the part's registers may change their state. When VDD
	 * recovers to 2.25V (or greater), the part may thus be in an
	 * unknown mode of operation. The user can return the part to
	 * a known mode of operation either by (a) setting VDD = 0V for
	 * 1 second or more and then powering back up with a slew rate
	 * of 0.5V/ms or greater, or (b) via I2C disable all ALS/PROX
	 * conversions, clear the test registers, and then rewrite all
	 * registers to the desired values.
	 * ...
	 * FOR ISL29011, ISL29018, ISL29021, ISL29023
	 * 1. Write 0x00 to register 0x08 (TEST)
	 * 2. Write 0x00 to register 0x00 (CMD1)
	 * 3. Rewrite all registers to the desired values
	 *
	 * ISL29018 Data Sheet (FN6619.1, Feb 11, 2010) essentially says
	 * the same thing EXCEPT the data sheet asks for a 1ms delay after
	 * writing the CMD1 register.
	 */
	status = regmap_write(chip->regmap, ISL29018_REG_TEST, 0x0);
	if (status < 0) {
		dev_err(chip->dev, "Failed to clear isl29018 TEST reg.(%d)\n",
			status);
		return status;
	}

	/* See Intersil AN1534 comments above.
	 * "Operating Mode" (COMMAND1) register is reprogrammed when
	 * data is read from the device.
	 */
	status = regmap_write(chip->regmap, ISL29018_REG_ADD_COMMAND1, 0);
	if (status < 0) {
		dev_err(chip->dev, "Failed to clear isl29018 CMD1 reg.(%d)\n",
			status);
		return status;
	}

	usleep_range(1000, 2000);	/* per data sheet, page 10 */

	/* set defaults */
	status = isl29018_set_range(chip, chip->range, &new_range);
	if (status < 0) {
		dev_err(chip->dev, "Init of isl29018 fails\n");
		return status;
	}

	status = isl29018_set_resolution(chip, chip->adc_bit,
						&new_adc_bit);

	return 0;
}

static const struct iio_info isl29018_info = {
	.attrs = &isl29018_group,
	.driver_module = THIS_MODULE,
	.read_raw = &isl29018_read_raw,
	.write_raw = &isl29018_write_raw,
};

static const struct iio_info isl29023_info = {
	.attrs = &isl29023_group,
	.driver_module = THIS_MODULE,
	.read_raw = &isl29018_read_raw,
	.write_raw = &isl29018_write_raw,
};

static bool is_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case ISL29018_REG_ADD_DATA_LSB:
	case ISL29018_REG_ADD_DATA_MSB:
	case ISL29018_REG_ADD_COMMAND1:
	case ISL29018_REG_TEST:
	case ISL29035_REG_DEVICE_ID:
		return true;
	default:
		return false;
	}
}

/*
 * isl29018_regmap_config: regmap configuration.
 * Use RBTREE mechanism for caching.
 */
static const struct regmap_config isl29018_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.volatile_reg = is_volatile_reg,
	.max_register = ISL29018_REG_TEST,
	.num_reg_defaults_raw = ISL29018_REG_TEST + 1,
	.cache_type = REGCACHE_RBTREE,
};

/* isl29035_regmap_config: regmap configuration for ISL29035 */
static const struct regmap_config isl29035_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.volatile_reg = is_volatile_reg,
	.max_register = ISL29035_REG_DEVICE_ID,
	.num_reg_defaults_raw = ISL29035_REG_DEVICE_ID + 1,
	.cache_type = REGCACHE_RBTREE,
};

struct chip_info {
	const struct iio_chan_spec *channels;
	int num_channels;
	const struct iio_info *indio_info;
	const struct regmap_config *regmap_cfg;
};

static const struct chip_info chip_info_tbl[] = {
	[isl29018] = {
		.channels = isl29018_channels,
		.num_channels = ARRAY_SIZE(isl29018_channels),
		.indio_info = &isl29018_info,
		.regmap_cfg = &isl29018_regmap_config,
	},
	[isl29023] = {
		.channels = isl29023_channels,
		.num_channels = ARRAY_SIZE(isl29023_channels),
		.indio_info = &isl29023_info,
		.regmap_cfg = &isl29018_regmap_config,
	},
	[isl29035] = {
		.channels = isl29023_channels,
		.num_channels = ARRAY_SIZE(isl29023_channels),
		.indio_info = &isl29023_info,
		.regmap_cfg = &isl29035_regmap_config,
	},
};

static const char *isl29018_match_acpi_device(struct device *dev, int *data)
{
	const struct acpi_device_id *id;

	id = acpi_match_device(dev->driver->acpi_match_table, dev);

	if (!id)
		return NULL;

	*data = (int) id->driver_data;

	return dev_name(dev);
}

static int isl29018_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct isl29018_chip *chip;
	struct iio_dev *indio_dev;
	int err;
	const char *name = NULL;
	int dev_id = 0;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*chip));
	if (indio_dev == NULL) {
		dev_err(&client->dev, "iio allocation fails\n");
		return -ENOMEM;
	}
	chip = iio_priv(indio_dev);

	i2c_set_clientdata(client, indio_dev);
	chip->dev = &client->dev;

	if (id) {
		name = id->name;
		dev_id = id->driver_data;
	}

	if (ACPI_HANDLE(&client->dev))
		name = isl29018_match_acpi_device(&client->dev, &dev_id);

	mutex_init(&chip->lock);

	chip->type = dev_id;
	chip->lux_scale = 1;
	chip->lux_uscale = 0;
	chip->ir_comp_scale = 0;
	chip->ir_comp_uscale = 0;
	chip->range = 1000;
	chip->opmode_trans_time = 0;
	chip->default_opmode = COMMMAND1_OPMODE_POWER_DOWN;
	chip->adc_bit = 16;
	chip->suspended = false;

	chip->regmap = devm_regmap_init_i2c(client,
				chip_info_tbl[dev_id].regmap_cfg);
	if (IS_ERR(chip->regmap)) {
		err = PTR_ERR(chip->regmap);
		dev_err(chip->dev, "regmap initialization failed: %d\n", err);
		return err;
	}

	err = isl29018_chip_init(chip);
	if (err)
		return err;

	indio_dev->info = chip_info_tbl[dev_id].indio_info;
	indio_dev->channels = chip_info_tbl[dev_id].channels;
	indio_dev->num_channels = chip_info_tbl[dev_id].num_channels;
	indio_dev->name = name;
	indio_dev->dev.parent = &client->dev;
	indio_dev->modes = INDIO_DIRECT_MODE;
	err = devm_iio_device_register(&client->dev, indio_dev);
	if (err) {
		dev_err(&client->dev, "iio registration fails\n");
		return err;
	}

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int isl29018_suspend(struct device *dev)
{
	struct isl29018_chip *chip = iio_priv(dev_get_drvdata(dev));
	int status = 0;

	mutex_lock(&chip->lock);

	if (chip->default_opmode != COMMMAND1_OPMODE_POWER_DOWN)
		status = isl29018_set_opmode(chip, COMMMAND1_OPMODE_POWER_DOWN);

	if (!status)
		chip->suspended = true;

	mutex_unlock(&chip->lock);
	return status;
}

static int isl29018_resume(struct device *dev)
{
	struct isl29018_chip *chip = iio_priv(dev_get_drvdata(dev));
	int err;

	mutex_lock(&chip->lock);

	err = isl29018_chip_init(chip);
	if (!err)
		err = isl29018_restore_default_opmode(chip);

	if (!err)
		chip->suspended = false;

	mutex_unlock(&chip->lock);
	return err;
}

static SIMPLE_DEV_PM_OPS(isl29018_pm_ops, isl29018_suspend, isl29018_resume);
#define ISL29018_PM_OPS (&isl29018_pm_ops)
#else
#define ISL29018_PM_OPS NULL
#endif

static const struct acpi_device_id isl29018_acpi_match[] = {
	{"ISL29018", isl29018},
	{"ISL29023", isl29023},
	{"ISL29035", isl29035},
	{},
};
MODULE_DEVICE_TABLE(acpi, isl29018_acpi_match);

static const struct i2c_device_id isl29018_id[] = {
	{"isl29018", isl29018},
	{"isl29023", isl29023},
	{"isl29035", isl29035},
	{}
};

MODULE_DEVICE_TABLE(i2c, isl29018_id);

static const struct of_device_id isl29018_of_match[] = {
	{ .compatible = "isil,isl29018", },
	{ .compatible = "isil,isl29023", },
	{ .compatible = "isil,isl29035", },
	{ },
};
MODULE_DEVICE_TABLE(of, isl29018_of_match);

#ifdef CONFIG_ACPI
static const struct acpi_device_id isl29018_acpi_id[] = {
	/* Intersil PNP ID "LSD" + Device ID 0x2918 */
	{ "LSD2918", 0 },
	{ }
};
MODULE_DEVICE_TABLE(acpi, isl29018_acpi_id);
#endif

static struct i2c_driver isl29018_driver = {
	.class	= I2C_CLASS_HWMON,
	.driver	 = {
			.name = "isl29018",
			.acpi_match_table = ACPI_PTR(isl29018_acpi_match),
			.pm = ISL29018_PM_OPS,
			.owner = THIS_MODULE,
			.of_match_table = isl29018_of_match,
			.acpi_match_table = ACPI_PTR(isl29018_acpi_id),
		    },
	.probe	 = isl29018_probe,
	.id_table = isl29018_id,
};
module_i2c_driver(isl29018_driver);

MODULE_DESCRIPTION("ISL29018 Ambient Light Sensor driver");
MODULE_LICENSE("GPL");
