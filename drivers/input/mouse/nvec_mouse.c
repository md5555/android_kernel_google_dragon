/*
 * drivers/input/mouse/nvec_mouse.c
 *
 * Mouse class input driver for mice and touchpads connected to an NvEc
 * compliant embedded controller
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
#include "nvec.h"
#include "nvodm_services.h"
#include "nvodm_mouse.h"

#define DRIVER_DESC "NvEc mouse driver"
#define DRIVER_LICENSE "GPL"

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE(DRIVER_LICENSE);

enum nvec_mouse_type {
	nvec_mouse_type_none,
	nvec_mouse_type_standard,
	nvec_mouse_type_intellimouse,
	nvec_mouse_type_intelli5buttonmouse,
};

#define NVEC_PAYLOAD	32

struct nvec_mouse
{
	struct input_dev	*input_dev;
	struct task_struct	*task;
	NvOdmOsSemaphoreHandle	semaphore;
	char			name[128];
	NvOdmMouseDeviceHandle	hDevice;
	enum nvec_mouse_type	type;
	unsigned char		data[NVEC_PAYLOAD];
	int			valid_data_size;
	int			enableWake;
	int			packetSize;
	unsigned char		previousState;
};

static struct platform_device *nvec_mouse_device;

// Commands sent to the mouse.
//
static const NvU8 cmdReadId		= 0xF2;
static const NvU8 cmdSetSampleRate	= 0xF3;
static const NvU8 cmdEnable		= 0xF4;
static const NvU8 cmdReset		= 0xFF;
static const NvU8 cmdSetResolution	= 0xE8;
static const NvU8 cmdSetScaling1_1	= 0xE6;
static const NvU8 cmdSetResolution2_1	= 0xE7;
static const NvU8 cmdSetStreamMode	= 0xEA;

//
// Mouse Responses
//
static const NvU8 responseAck			= 0xFA;
static const NvU8 responseResend		= 0xFE;
static const NvU8 responseStandardMouseId	= 0x00;
static const NvU8 responseIntelliMouseId	= 0x03;
static const NvU8 responseIntelli5buttonMouseId	= 0x04;
static const NvU8 responseBatSuccess		= 0xAA;
static const NvU8 responseBatError		= 0xFC;


int nvec_mouse_cmd(struct nvec_mouse *mouse, unsigned int cmd,
	int resp_size)
{
	volatile int cnt = 3;

	while (cnt--) {
		if (!NvOdmMouseSendRequest(mouse->hDevice, cmd, resp_size,
			&mouse->valid_data_size, mouse->data)) {
			printk("**nvec_mouse_cmd: SendRequest: fail\n");
			return -1;
		} if (mouse->valid_data_size != resp_size) {
			printk("**nvec_mouse_cmd: not valid data size\n");
			continue;
		}
		if (mouse->valid_data_size == 0 ||
			mouse->data[0] == responseAck) {
			//printk("**nvec_mouse_cmd: responseACK\n");
			return 0;
		}
	}
	printk("**nvec_mouse_cmd error\n");

	return -1;
}

int nvec_IsStandardMouse(struct nvec_mouse *mouse)
{
	printk("**nvec_IsStandardMouse\n");
	if (!nvec_mouse_cmd(mouse, cmdReset, 3) &&
		(mouse->data[1] == responseBatSuccess) &&
		(mouse->data[2] == responseStandardMouseId)) {
		return 0;
	}
	pr_err("NvEc mouse is not present\n");
	return -ENODEV;
}

static int nvec_setSampleRate(struct nvec_mouse *mouse,
	unsigned int SamplesPerSecond)
{
	printk("**nvec_setSampleRate\n");
	if (!nvec_mouse_cmd(mouse, cmdSetSampleRate, 1) &&
		!nvec_mouse_cmd(mouse, SamplesPerSecond, 1)) {
		return 0;
	}
	printk("**nvec_setSampleRate error\n");
	return -EINVAL;
}

int nvec_IsIntellimouse(struct nvec_mouse *mouse)
{
	printk("**nvec_IsIntellimouse\n");
	if (((!nvec_setSampleRate(mouse, 0xC8)) &&
		(!nvec_setSampleRate(mouse, 0x64)) &&
		(!nvec_setSampleRate(mouse, 0x50)))
		&& ((!nvec_mouse_cmd(mouse, cmdReadId, 1)) &&
			(mouse->data[1] == responseIntelliMouseId))) {

		printk("mouse->data[1]=0x%x\n", mouse->data[1]);
		return 0;
	}
	printk("**nvec_IsIntellimouse error\n");
	return -ENODEV;
}

int nvec_IsIntelli5buttonmouse(struct nvec_mouse *mouse)
{
	printk("**nvec_IsIntelli5buttonmouse\n");
	if (((!nvec_setSampleRate(mouse, 0xC8)) &&
		 (!nvec_setSampleRate(mouse, 0xC8)) &&
		 (!nvec_setSampleRate(mouse, 0x50)))
		&& ((!nvec_mouse_cmd(mouse, cmdReadId, 1)) &&
		mouse->data[1] == responseIntelli5buttonMouseId)) {
		printk("mouse->data[1]=0x%x\n", mouse->data[1]);
		return 0;
	}
	printk("**nvec_IsIntelli5buttonmouse error\n");
	return -ENODEV;
}

static int nvec_mouse_receive_thread(void *arg)
{
	struct input_dev *input_dev = (struct input_dev *)arg;
	struct nvec_mouse *mouse = input_get_drvdata(input_dev);
	NvError nverr;
	NvU8	buttonState;
	NvU8	updatedState;
	NvS8	dataBuffer[4] = {0};
	NvU32   x,y;

	printk("**nvec_mouse_receive_thread\n");
	if (!NvOdmMouseEnableInterrupt(mouse->hDevice, mouse->semaphore)) {
		printk("**nvec_mouse_receive_thread: EnableInterrupt: fail\n");
		return -1;
	}

	while (!kthread_should_stop()) {
		unsigned char data[4];
		int size;

		nverr = NvOdmOsSemaphoreWaitTimeout(mouse->semaphore,1000);
		if (nverr == NvError_Timeout) {
			printk("**nvec_mouse_receive_thread: timeout\n");
			continue;
		}

		if (kthread_should_stop()) {
			printk("**nvec_mouse_receive_thread: should_stop\n");
			return 0;
		}

		if (!NvOdmMouseGetEventInfo(mouse->hDevice, &size, data))
			continue;

		/* Short packets are not valid? */
		if (size < 3) continue;

		NvOsMemcpy(&dataBuffer, data, 4);

		updatedState = dataBuffer[0] ^ mouse->previousState;
		if (updatedState) {
			buttonState = dataBuffer[0];

			if (updatedState & 0x01)
				input_report_key(input_dev, BTN_LEFT,
					buttonState & 0x01);
			if (updatedState & 0x02)
				input_report_key(input_dev, BTN_RIGHT,
					(buttonState>>1) & 0x01);
                        if (updatedState & 0x04)
				input_report_key(input_dev, BTN_MIDDLE,
					(buttonState>>2) & 0x01);

			mouse->previousState = dataBuffer[0];
		}
		else
		{
			x = dataBuffer[1];
			y = -dataBuffer[2];

			input_report_rel(input_dev, REL_X, x);
			input_report_rel(input_dev, REL_Y, y);

			input_sync(input_dev);
		}

		if (mouse->packetSize == 4 && dataBuffer[3])
			input_report_rel(input_dev, REL_WHEEL, dataBuffer[3]);

		input_sync(input_dev);
	}
	printk("**nvec_mouse_receive_thread end\n");
	return 0;
}

static int nvec_mouse_open(struct input_dev *dev)
{
	struct nvec_mouse *mouse = input_get_drvdata(dev);

	printk("**nvec_mouse_open\n");

	/* Set the resolution */
	/*
	 * Factor	Resolution
	 * ------	----------
	 * 0x00		1 count/mm
	 * 0x01		2 count/mm
	 * 0x02		4 count/mm
	 * 0x03		8 count/mm
	 */
	if (nvec_mouse_cmd(mouse, cmdSetResolution, 1) ||
		nvec_mouse_cmd(mouse, 2, 1)) {
		printk("**nvec_mouse_open: cmd fail\n");
		return -EINVAL;
	}

	/* Set scaling */
	if (nvec_mouse_cmd(mouse, cmdSetScaling1_1, 1)) {
		printk("**nvec_mouse_open: cmdsetscaling fail\n");
		return -EINVAL;
	}

	if (nvec_setSampleRate(mouse, 0x64)) {
		printk("**nvec_mouse_open: setsamplerate fail\n");
		return -EINVAL;
	}

	if (nvec_mouse_cmd(mouse, cmdEnable, 1)) {
		printk("**nvec_mouse_open: mouse cmd fail\n");
		return -EINVAL;
	}

	if (!NvOdmMouseStartStreaming(mouse->hDevice, mouse->packetSize)) {
		printk("**nvec_mouse_open: streaming fail\n");
		return -EINVAL;
	}

	printk("**nvec_mouse_open end\n");
	return 0;
}

static void nvec_mouse_close(struct input_dev *dev)
{
	printk("**nvec_mouse_close\n");
	return;
}


static int __devinit nvec_mouse_probe(struct platform_device *pdev)
{
	int error;
	struct nvec_mouse *mouse;
	struct input_dev *input_dev;

	printk("**nvec_mouse_probe\n");

	mouse = kzalloc(sizeof(struct nvec_mouse), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!mouse || !input_dev) {
		printk("**nvec_mouse_probe: input_allocate_device: fail\n");
		error = -ENOMEM;
	goto fail;
	}

	if (!NvOdmMouseDeviceOpen(&mouse->hDevice)) {
		printk("**nvec_mouse_probe: NvOdmMouseDeviceOpen: fail\n");
		pr_err("NvOdmMouseDeviceOpen failed\n");
		error = -ENODEV;
		goto fail;
	}

	mouse->input_dev = input_dev;
	input_set_drvdata(input_dev, mouse);

	mouse->type = nvec_mouse_type_none;
	mouse->enableWake = 0;

	/* Probe for the type of the mouse */
	if (!nvec_IsStandardMouse(mouse)) {
		if (!nvec_IsIntellimouse(mouse)) {
			mouse->packetSize = 4;
			if (!nvec_IsIntelli5buttonmouse(mouse)) {
				printk("**It is an Intelli5buttonmouse\n");
				mouse->type = nvec_mouse_type_intelli5buttonmouse;
			}
			else
			{
				printk("**It is an Intellimouse\n");
				mouse->type = nvec_mouse_type_intellimouse;
			}
		} else
		{
			printk("**It is an StandardMouse\n");
			mouse->packetSize = 3;
			mouse->type = nvec_mouse_type_standard;
		}
	} else {
		printk("**nvec_mouse_probe: not a mouse\n");
		error = -ENODEV;
		goto fail_no_mouse_found;
	}

	platform_set_drvdata(pdev, input_dev);

	mouse->semaphore = NvOdmOsSemaphoreCreate(0);
	if (mouse->semaphore == NULL) {
		error = -1;
		printk("**nvec_mouse_probe: NvOdmOsSemaphoreCreate: fail\n");
		pr_err("nvec_mouse: Semaphore creation failed\n");
		goto fail_semaphore_create;
	}

	mouse->task = kthread_create(nvec_mouse_receive_thread, input_dev,
		"nvec_mouse_thread");
	if (mouse->task == NULL) {
		printk("**nvec_mouse_probe: kthread_create: fail\n");
		error = -ENOMEM;
		goto fail_thread_create;
	}
	wake_up_process( mouse->task );

	if (!strlen(mouse->name))
		snprintf(mouse->name, sizeof(mouse->name), "nvec mouse");

	input_dev->name = mouse->name;
	input_dev->open = nvec_mouse_open;
	input_dev->close = nvec_mouse_close;

	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_REL);

	input_dev->keybit[BIT_WORD(BTN_MOUSE)] = BIT_MASK(BTN_LEFT) |
	BIT_MASK(BTN_RIGHT) | BIT_MASK(BTN_MIDDLE);
	input_dev->relbit[0] = BIT_MASK(REL_X) | BIT_MASK(REL_Y);
	input_dev->relbit[0] |= BIT_MASK(REL_WHEEL);

	error = input_register_device(mouse->input_dev);
	if (error) {
		printk("**nvec_mouse_probe: input_register_device: fail\n");
		goto fail_input_register;
	}

	mouse->previousState = 0;

	printk(KERN_INFO DRIVER_DESC ": registered NVEC mouse driver\n");
	return 0;

fail_input_register:
	(void)kthread_stop(mouse->task);
fail_thread_create:
	NvOdmOsSemaphoreDestroy(mouse->semaphore);
fail_semaphore_create:
fail_no_mouse_found:
	NvOdmMouseDeviceClose(mouse->hDevice);
fail:
	input_free_device(input_dev);
	kfree(mouse);

	return error;
}

static int __devexit nvec_mouse_remove(struct platform_device *dev)
{
	struct input_dev *input_dev = platform_get_drvdata(dev);
	struct nvec_mouse *mouse = platform_get_drvdata(dev);

	printk("**nvec_mouse_remove\n");
	(void)kthread_stop(mouse->task);
	NvOdmOsSemaphoreDestroy(mouse->semaphore);
	NvOdmMouseDeviceClose(mouse->hDevice);
	input_free_device(input_dev);
	kfree(mouse);

	printk("**nvec_mouse_remove end\n");
	return 0;
}

static struct platform_driver nvec_mouse_driver = {
	.driver		= {
		.name	= "nvec_mouse",
		.owner	= THIS_MODULE,
	},
	.probe		= nvec_mouse_probe,
	.remove		= __devexit_p(nvec_mouse_remove),
};


static int __init nvec_mouse_init(void)
{
	int err;

	printk("**nvec_mouse_init\n");
	err = platform_driver_register(&nvec_mouse_driver);
	if (err) {
		printk("**nvec_mouse_init: platform_driver_register: fail\n");
		goto error;
	}

	nvec_mouse_device = platform_device_alloc("nvec_mouse", -1);
	if (!nvec_mouse_device) {
		printk("**nvec_mouse_init: platform_device_alloc: fail\n");
		err = -ENOMEM;
		goto error_unregister_driver;
	}

	err = platform_device_add(nvec_mouse_device);
	if (err)
	{
		printk("**nvec_mouse_init: platform_device_add: fail\n");
		goto error_free_device;
	}

	printk("**nvec_mouse_init end\n");
	return 0;

error_free_device:
	platform_device_put(nvec_mouse_device);
error_unregister_driver:
	platform_driver_unregister(&nvec_mouse_driver);
error:
	return err;
}

static void __exit nvec_mouse_exit(void)
{
	printk("**nvec_mouse_exit\n");
	platform_device_unregister(nvec_mouse_device);
	platform_driver_unregister(&nvec_mouse_driver);
}

module_init(nvec_mouse_init);
module_exit(nvec_mouse_exit);

