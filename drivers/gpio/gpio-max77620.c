/*
 * MAXIM MAX77620 GPIO driver
 *
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/debugfs.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mfd/max77620.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/regmap.h>

#define GPIO_REG_ADDR(offset) (MAX77620_REG_GPIO0 + offset)

struct max77620_gpio {
	struct gpio_chip	gpio_chip;
	struct device		*parent;
	struct device		*dev;
	int			gpio_base;
};

static inline struct max77620_gpio *to_max77620_gpio(struct gpio_chip *gpio)
{
	return container_of(gpio, struct max77620_gpio, gpio_chip);
}

static int max77620_gpio_dir_input(struct gpio_chip *gpio, unsigned offset)
{
	struct max77620_gpio *max77620_gpio = to_max77620_gpio(gpio);
	struct device *dev = max77620_gpio->dev;
	struct device *parent = max77620_gpio->parent;
	int ret;

	ret = max77620_reg_update(parent, MAX77620_PWR_SLAVE,
		GPIO_REG_ADDR(offset), MAX77620_CNFG_GPIO_DIR_MASK,
				MAX77620_CNFG_GPIO_DIR_INPUT);
	if (ret < 0)
		dev_err(dev, "CNFG_GPIOx dir update failed: %d\n", ret);
	return ret;
}

static int max77620_gpio_get(struct gpio_chip *gpio, unsigned offset)
{
	struct max77620_gpio *max77620_gpio = to_max77620_gpio(gpio);
	struct device *dev = max77620_gpio->dev;
	struct device *parent = max77620_gpio->parent;
	u8 val;
	int ret;

	ret = max77620_reg_read(parent, MAX77620_PWR_SLAVE,
				GPIO_REG_ADDR(offset), &val);
	if (ret < 0) {
		dev_err(dev, "CNFG_GPIOx read failed: %d\n", ret);
		return ret;
	}

	return !!(val & MAX77620_CNFG_GPIO_INPUT_VAL_MASK);
}

static int max77620_gpio_dir_output(struct gpio_chip *gpio, unsigned offset,
				int value)
{
	struct max77620_gpio *max77620_gpio = to_max77620_gpio(gpio);
	struct device *dev = max77620_gpio->dev;
	struct device *parent = max77620_gpio->parent;
	u8 val;
	int ret;

	if (value)
		val = MAX77620_CNFG_GPIO_OUTPUT_VAL_HIGH;
	else
		val = MAX77620_CNFG_GPIO_OUTPUT_VAL_LOW;

	ret = max77620_reg_update(parent, MAX77620_PWR_SLAVE,
			GPIO_REG_ADDR(offset),
			MAX77620_CNFG_GPIO_OUTPUT_VAL_MASK, val);
	if (ret < 0) {
		dev_err(dev, "CNFG_GPIOx val update failed: %d\n", ret);
		return ret;
	}

	ret = max77620_reg_update(parent, MAX77620_PWR_SLAVE,
		GPIO_REG_ADDR(offset), MAX77620_CNFG_GPIO_DIR_MASK,
				MAX77620_CNFG_GPIO_DIR_OUTPUT);
	if (ret < 0)
		dev_err(dev, "CNFG_GPIOx dir update failed: %d\n", ret);
	return ret;
}


static int max77620_gpio_set_debounce(struct gpio_chip *gpio,
		unsigned offset, unsigned debounce)
{
	struct max77620_gpio *max77620_gpio = to_max77620_gpio(gpio);
	struct device *dev = max77620_gpio->dev;
	struct device *parent = max77620_gpio->parent;
	u8 val;
	int ret;

	if (debounce == 0)
		val = MAX77620_CNFG_GPIO_DBNC_None;
	else if ((0 < debounce) && (debounce <= 8))
		val = MAX77620_CNFG_GPIO_DBNC_8ms;
	else if ((8 < debounce) && (debounce <= 16))
		val = MAX77620_CNFG_GPIO_DBNC_16ms;
	else if ((16 < debounce) && (debounce <= 32))
		val = MAX77620_CNFG_GPIO_DBNC_32ms;
	else {
		dev_err(dev, "%s(): illegal value %u\n", __func__, debounce);
		return -EINVAL;
	}

	ret = max77620_reg_update(parent, MAX77620_PWR_SLAVE,
		GPIO_REG_ADDR(offset), MAX77620_CNFG_GPIO_DBNC_MASK, val);
	if (ret < 0)
		dev_err(dev, "CNFG_GPIOx debounce update failed: %d\n", ret);
	return ret;
}

static void max77620_gpio_set(struct gpio_chip *gpio, unsigned offset,
			int value)
{
	struct max77620_gpio *max77620_gpio = to_max77620_gpio(gpio);
	struct device *dev = max77620_gpio->dev;
	struct device *parent = max77620_gpio->parent;
	u8 val;
	int ret;

	if (value)
		val = MAX77620_CNFG_GPIO_OUTPUT_VAL_HIGH;
	else
		val = MAX77620_CNFG_GPIO_OUTPUT_VAL_LOW;

	ret = max77620_reg_update(parent, MAX77620_PWR_SLAVE,
			GPIO_REG_ADDR(offset),
			MAX77620_CNFG_GPIO_OUTPUT_VAL_MASK, val);
	if (ret < 0)
		dev_err(dev, "CNFG_GPIOx val update failed: %d\n", ret);
}

static int max77620_gpio_probe(struct platform_device *pdev)
{
	struct max77620_gpio *max77620_gpio;
	int ret;

	max77620_gpio = devm_kzalloc(&pdev->dev,
				sizeof(*max77620_gpio), GFP_KERNEL);
	if (!max77620_gpio) {
		dev_err(&pdev->dev, "Could not allocate max77620_gpio\n");
		return -ENOMEM;
	}

	max77620_gpio->parent = pdev->dev.parent;
	max77620_gpio->dev = &pdev->dev;

	max77620_gpio->gpio_chip.owner = THIS_MODULE;
	max77620_gpio->gpio_chip.label = pdev->name;
	max77620_gpio->gpio_chip.dev = &pdev->dev;
	max77620_gpio->gpio_chip.direction_input = max77620_gpio_dir_input;
	max77620_gpio->gpio_chip.get = max77620_gpio_get;
	max77620_gpio->gpio_chip.direction_output = max77620_gpio_dir_output;
	max77620_gpio->gpio_chip.set_debounce = max77620_gpio_set_debounce;
	max77620_gpio->gpio_chip.set = max77620_gpio_set;
	max77620_gpio->gpio_chip.ngpio = MAX77620_GPIO_NR;
	max77620_gpio->gpio_chip.can_sleep = 1;
	max77620_gpio->gpio_chip.base = -1;
#ifdef CONFIG_OF_GPIO
	max77620_gpio->gpio_chip.of_node = pdev->dev.parent->of_node;
#endif

	platform_set_drvdata(pdev, max77620_gpio);

	ret = gpiochip_add(&max77620_gpio->gpio_chip);
	if (ret < 0) {
		dev_err(&pdev->dev, "gpio_init: Failed to add max77620_gpio\n");
		return ret;
	}
	max77620_gpio->gpio_base = max77620_gpio->gpio_chip.base;

	dev_info(&pdev->dev, "max77620 gpio successfully initialized\n");
	return 0;
}

static int max77620_gpio_remove(struct platform_device *pdev)
{
	struct max77620_gpio *max77620_gpio = platform_get_drvdata(pdev);

	gpiochip_remove(&max77620_gpio->gpio_chip);

	return 0;
}

static struct platform_driver max77620_gpio_driver = {
	.driver.name	= "max77620-gpio",
	.driver.owner	= THIS_MODULE,
	.probe		= max77620_gpio_probe,
	.remove		= max77620_gpio_remove,
};

static int __init max77620_gpio_init(void)
{
	return platform_driver_register(&max77620_gpio_driver);
}
subsys_initcall(max77620_gpio_init);

static void __exit max77620_gpio_exit(void)
{
	platform_driver_unregister(&max77620_gpio_driver);
}
module_exit(max77620_gpio_exit);

MODULE_DESCRIPTION("GPIO interface for MAX77620 PMIC");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Chaitanya Bandi <bandik@nvidia.com>");
MODULE_ALIAS("platform:max77620-gpio");
