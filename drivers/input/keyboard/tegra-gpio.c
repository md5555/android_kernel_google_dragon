/*
 * drivers/input/keyboard/tegra-gpio.c
 *
 * Keyboard class input driver for buttons directly connected to NVIDIA
 * Tegra SoC GPIOs
 *
 * Copyright (c) 2009, NVIDIA Corporation.
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
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/module.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/kthread.h>
#include "nvos.h"
#include "nvodm_services.h"
#include "nvodm_query_gpio.h"
#include "nvrm_interrupt.h"
#include "nvrm_module.h"
#include "nvrm_gpio.h"
#include "mach/nvrm_linux.h"

#define DEBUG_NVGPIO_KBD 0

#define logd(x) \
	do { \
		if (DEBUG_NVGPIO_KBD) \
			printk x; \
	} while (0) \

struct nvgpio_key_info {
	int			port;
	int			pin;
	int			pin_type;
	int			debounce_time;
	NvRmGpioPinHandle	gpio_pin;
	NvRmGpioInterruptHandle intr_handle;
	int			key_code;
};

static struct nvgpio_key_info s_nvgpio_key_info[] = {
	//port,		pin,	type,	bounce,	pin,	intr,	code
	{'c'-'a',	3,	0,	10,	NULL,	NULL,	KEY_MENU},
	{'d'-'a',	3,	0,	10,	NULL,	NULL,	KEY_HOME},
	{'d'-'a',	4,	0,	10,	NULL,	NULL,	KEY_BACK}
};

static int s_num_pins = sizeof(s_nvgpio_key_info)/sizeof(s_nvgpio_key_info[0]);

struct nvgpio_keyboard {
	struct input_dev *input_dev;
	char name[128];
};

struct nvgpio_keyboard *s_keyboard = NULL;
static struct platform_device *s_nvgpio_keyboard_device = NULL;

static void do_handle_interrupt(void *args)
{
	int key_pressed;
	NvRmGpioPinState state;
	struct nvgpio_key_info *key_info = (struct nvgpio_key_info*)args;

	NvRmGpioReadPins(s_hGpioGlobal, &key_info->gpio_pin, &state, 1);
	key_pressed = (state == NvRmGpioPinState_Low) ? 1 : 0;

	input_report_key(s_keyboard->input_dev, key_info->key_code, key_pressed);
	logd(("\n***gpio keyboard interrupt, port=%d, pin=%d, key_code=%d, press=%d",
		key_info->port, key_info->pin, key_info->key_code, key_pressed));
	NvRmGpioInterruptDone(key_info->intr_handle);
}

static int nvgpio_keyboard_open(struct input_dev *dev)
{
	return 0;
}

static void nvgpio_keyboard_close(struct input_dev *dev)
{
	return;
}

static int __devinit nvgpio_keyboard_probe(struct platform_device *pdev)
{
	int error;
	NvError nv_err;
	struct nvgpio_keyboard *keyboard;
	struct input_dev *input_dev;
	int i;

	keyboard = kzalloc(sizeof(struct nvgpio_keyboard), GFP_KERNEL);
	input_dev = input_allocate_device();
	error = -ENOMEM;
	if (!keyboard || !input_dev)
		goto fail;

	keyboard->input_dev = input_dev;
	input_set_drvdata(input_dev, keyboard);
	platform_set_drvdata(pdev, input_dev);

	for (i = 0; i < s_num_pins; i++) {
		nv_err = NvRmGpioAcquirePinHandle(s_hGpioGlobal, 
			s_nvgpio_key_info[i].port, s_nvgpio_key_info[i].pin, 
			&s_nvgpio_key_info[i].gpio_pin);
		if (nv_err != NvSuccess)
			goto fail_gpio_init;
		nv_err = NvRmGpioConfigPins(s_hGpioGlobal, 
					&s_nvgpio_key_info[i].gpio_pin, 1, 
					NvRmGpioPinMode_InputData);
		if (nv_err != NvSuccess)
			goto fail_gpio_init;
		nv_err = NvRmGpioInterruptRegister(s_hGpioGlobal, s_hRmGlobal, 
			s_nvgpio_key_info[i].gpio_pin, do_handle_interrupt, 
			NvRmGpioPinMode_InputInterruptAny,
			&s_nvgpio_key_info[i],
			&s_nvgpio_key_info[i].intr_handle, 
			s_nvgpio_key_info[i].debounce_time);
		if (nv_err != NvSuccess)
			goto fail_gpio_init;
		nv_err = NvRmGpioInterruptEnable(s_nvgpio_key_info[i].intr_handle);
		if (nv_err != NvSuccess)
			goto fail_gpio_init;
	}

	if (!strlen(keyboard->name))
		snprintf(keyboard->name, sizeof(keyboard->name),
			 "nvgpio keyboard");

	input_dev->name = keyboard->name;
	input_dev->open = nvgpio_keyboard_open;
	input_dev->close = nvgpio_keyboard_close;

	__set_bit(EV_KEY, input_dev->evbit);
	for (i = 0; i < s_num_pins; i++) {
		__set_bit(s_nvgpio_key_info[i].key_code, input_dev->keybit);
	}

	error = input_register_device(keyboard->input_dev);
	if (error)
		goto fail_input_register;

	logd(("\n*****nvgpio_keyboard_probe success"));
	s_keyboard = keyboard;
	return 0;

fail_input_register:
fail_gpio_init:
	for (i = 0; i < s_num_pins; i++) {

		if (s_nvgpio_key_info[i].intr_handle) {
			NvRmGpioInterruptMask(s_nvgpio_key_info[i].intr_handle,
				NV_TRUE);
			NvRmGpioInterruptUnregister(s_hGpioGlobal, s_hRmGlobal, 
				s_nvgpio_key_info[i].intr_handle);
			s_nvgpio_key_info[i].intr_handle = NULL;
		}
		if (s_nvgpio_key_info[i].gpio_pin) {
			NvRmGpioReleasePinHandles(s_hGpioGlobal, 
				&s_nvgpio_key_info[i].gpio_pin, 1);
			s_nvgpio_key_info[i].gpio_pin = NULL;
		}
	}
fail:
	input_free_device(input_dev);
	kfree(keyboard);
	return error;
}

static int __devexit nvgpio_keyboard_remove(struct platform_device *dev)
{
	int i;
	struct input_dev *input_dev = platform_get_drvdata(dev);
	struct nvgpio_keyboard *keyboard = platform_get_drvdata(dev);

	for (i = 0; i < s_num_pins; i++) {

		if (s_nvgpio_key_info[i].intr_handle) {
			NvRmGpioInterruptMask(s_nvgpio_key_info[i].intr_handle,
				NV_TRUE);
			NvRmGpioInterruptUnregister(s_hGpioGlobal, s_hRmGlobal, 
				s_nvgpio_key_info[i].intr_handle);
			s_nvgpio_key_info[i].intr_handle = NULL;
		}
		if (s_nvgpio_key_info[i].gpio_pin) {
			NvRmGpioReleasePinHandles(s_hGpioGlobal, 
				&s_nvgpio_key_info[i].gpio_pin, 1);
			s_nvgpio_key_info[i].gpio_pin = NULL;
		}
	}

	input_free_device(input_dev);
	kfree(keyboard);
	return 0;
}

static struct platform_driver s_nvgpio_keyboard_driver = {
	.driver	 = {
		.name   = "nvgpio_keyboard",
		.owner  = THIS_MODULE,
	},
	.probe	  = nvgpio_keyboard_probe,
	.remove	 = __devexit_p(nvgpio_keyboard_remove),
};

static int __init nvgpio_keyboard_init(void)
{
	int err;

	err = platform_driver_register(&s_nvgpio_keyboard_driver);
	if (err)
		goto error;

	s_nvgpio_keyboard_device = platform_device_alloc("nvgpio_keyboard", -1);
	err = -ENOMEM;
	if (!s_nvgpio_keyboard_device)
		goto error_unregister_driver;

	err = platform_device_add(s_nvgpio_keyboard_device);
	if (err)
		goto error_free_device;
	logd(("\n*****nvgpio_keyboard_init success"));
	return 0;

error_free_device:
	platform_device_put(s_nvgpio_keyboard_device);
error_unregister_driver:
	platform_driver_unregister(&s_nvgpio_keyboard_driver);
error:
	return err;
}

static void __exit nvgpio_keyboard_exit(void)
{
	platform_device_unregister(s_nvgpio_keyboard_device);
	platform_driver_unregister(&s_nvgpio_keyboard_driver);
}

module_init(nvgpio_keyboard_init);
module_exit(nvgpio_keyboard_exit);

#define DRIVER_DESC "Nvidia Gpio keyboard driver"
#define DRIVER_LICENSE "GPL"

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE(DRIVER_LICENSE);

