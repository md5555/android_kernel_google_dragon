/*
 * drivers/input/touchscreen/tegra_odm.c
 *
 * Touchscreen class input driver for platforms using NVIDIA's Tegra ODM kit
 * driver interface
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
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/freezer.h>
#include <linux/tegra_devices.h>
#include <nvodm_services.h>
#include <nvodm_touch.h>

struct tegra_touch_driver_data
{
	struct input_dev	*input_dev;
	struct task_struct	*task;
	NvOdmOsSemaphoreHandle	semaphore;
	NvOdmTouchDeviceHandle	hTouchDevice;
	NvBool			bPollingMode;
	NvU32			pollingIntervalMS;
	NvOdmTouchCapabilities	caps;
	NvU32			MaxX;
	NvU32			MinX;
	NvU32			MaxY;
	NvU32			MinY;
	int			shutdown;
	struct early_suspend	early_suspend;
};

#define NVODM_TOUCH_NAME "nvodm_touch"

#define swapv(x, y) do { typeof(x) z = x; x = y; y = z; } while (0)

#ifdef CONFIG_HAS_EARLYSUSPEND
static void tegra_touch_early_suspend(struct early_suspend *es)
{
        struct tegra_touch_driver_data *touch;
        touch = container_of(es, struct tegra_touch_driver_data, early_suspend);

	if (touch && touch->hTouchDevice) {
		NvOdmTouchPowerOnOff(touch->hTouchDevice, NV_FALSE);
	}
	else {
		pr_err("tegra_touch_early_suspend: NULL handles passed\n");
	}
}

static void tegra_touch_late_resume(struct early_suspend *es)
{
        struct tegra_touch_driver_data *touch;
        touch = container_of(es, struct tegra_touch_driver_data, early_suspend);

	if (touch && touch->hTouchDevice) {
		NvOdmTouchPowerOnOff(touch->hTouchDevice, NV_TRUE);
	}
	else {
		pr_err("tegra_touch_late_resume: NULL handles passed\n");
	}
}
#else
static int tegra_touch_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct tegra_touch_driver_data *touch = platform_get_drvdata(pdev);

	if (touch && touch->hTouchDevice) {
		NvOdmTouchPowerOnOff(touch->hTouchDevice, NV_FALSE);
		return 0;
	}
	pr_err("tegra_touch_suspend: NULL handles passed\n");
	return -1;
}

static int tegra_touch_resume(struct platform_device *pdev)
{
	struct tegra_touch_driver_data *touch = platform_get_drvdata(pdev);

	if (touch && touch->hTouchDevice) {
		NvOdmTouchPowerOnOff(touch->hTouchDevice, NV_TRUE);
		return 0;
	}
	pr_err("tegra_touch_resume: NULL handles passed\n");
	return -1;
}
#endif

static int tegra_touch_thread(void *pdata)
{
	struct tegra_touch_driver_data *touch =
		(struct tegra_touch_driver_data*)pdata;
	NvOdmTouchCoordinateInfo c = {0};
	NvU32 x[2] = {0}, y[2] = {0}, i = 0, prev_x[2] = {0}, prev_y[2] = {0};
	NvBool bKeepReadingSamples = NV_FALSE;
	NvU32 fingers = 0;
	NvBool ToolDown[2] = {NV_FALSE, NV_FALSE};
	NvOdmTouchCapabilities *caps = &touch->caps;
	NvU32 max_fingers = caps->MaxNumberOfFingerCoordReported;

	/* touch event thread should be frozen before suspend */
	set_freezable_with_signal();

	for (;;) {
		if (touch->bPollingMode)
			msleep(touch->pollingIntervalMS); 
		else
			NvOdmOsSemaphoreWait(touch->semaphore);

		bKeepReadingSamples = NV_TRUE;
		while (bKeepReadingSamples) {
			if (!NvOdmTouchReadCoordinate(touch->hTouchDevice, &c)){
				pr_err("Couldn't read touch sample\n");
				bKeepReadingSamples = NV_FALSE;
				continue;
			}

			fingers = c.additionalInfo.Fingers;

			/*
			 * sometimes the HW reports num of fingers greater than
			 * the max supported. This happens when previously there
			 * were 2 fingers touching and one of them was lifted.
			 * We give away the previously stored state for the
			 * first finger, and the lifted finger is
			 * sent with flags indicating PEN UP.
			 */
			if (((fingers == 1) || (fingers>max_fingers)) &&
			     (ToolDown[1] == NV_TRUE)) {
				ToolDown[0] = NV_TRUE;
				input_report_abs(touch->input_dev,
					ABS_X, prev_x[0]);
				input_report_abs(touch->input_dev,
					ABS_Y, prev_y[0]);
				input_report_key(touch->input_dev,
						BTN_TOUCH, ToolDown[0]);
				ToolDown[1] = NV_FALSE;
				input_report_abs(touch->input_dev,
					ABS_HAT0X, prev_x[1]); // x
				input_report_abs(touch->input_dev,
					ABS_HAT0Y, prev_y[1]); // y
				input_report_key(touch->input_dev,
					BTN_2, ToolDown[1]);
				input_sync(touch->input_dev);
			}

			if (c.fingerstate & NvOdmTouchSampleIgnore)
				goto DoneWithSample;

			switch (fingers) {
			case 0:
				for (i=0; i<max_fingers; i++) {
					ToolDown[i] = NV_FALSE;
				}
				break;
			case 1:
				ToolDown[0] = NV_TRUE;
				ToolDown[1] = NV_FALSE;
				break;
			case 2:
				for (i=0; i<max_fingers; i++) {
					ToolDown[i] = NV_TRUE;
				}
				break;
			default:
				/* can occur because of sensor errors */
				c.fingerstate = NvOdmTouchSampleIgnore;;
				goto DoneWithSample;
			}

			/* from 1 finger to no fingers */
			if ((fingers == 0) && (ToolDown[0] == NV_TRUE)) {
				x[0] = prev_x[0];
				y[0] = prev_y[0];
				ToolDown[0] = NV_FALSE;
			}
			else if (fingers == 1) {
				x[0] = c.xcoord;
				y[0] = c.ycoord;
			}
			else {
				for (i = 0; i < fingers; i++) {
					x[i] = c.additionalInfo.multi_XYCoords[i][0];
					y[i] = c.additionalInfo.multi_XYCoords[i][1];
				}
			}

			/* transformation from touch to screen orientation */
			if (caps->Orientation & NvOdmTouchOrientation_V_FLIP) {
				y[0] = caps->YMaxPosition +
					caps->YMinPosition - y[0];
				y[1] = caps->YMaxPosition +
					caps->YMinPosition - y[1];
			}
			if (caps->Orientation & NvOdmTouchOrientation_H_FLIP) {
				x[0] = caps->XMaxPosition +
					caps->XMinPosition - x[0];
				x[1] = caps->XMaxPosition +
					caps->XMinPosition - x[1];
			}

			if (caps->Orientation & NvOdmTouchOrientation_XY_SWAP) {
				for (i = 0; i < max_fingers; i++)
					swapv(x[i],y[i]);
			}

			if (c.fingerstate & NvOdmTouchSampleValidFlag) {
				input_report_abs(touch->input_dev, ABS_X, x[0]);
				input_report_abs(touch->input_dev, ABS_Y, y[0]);
				prev_x[0] = x[0];
				prev_y[0] = y[0];
			}

			if (caps->IsPressureSupported) {
				input_report_abs(touch->input_dev,
					ABS_PRESSURE, 
					c.additionalInfo.Pressure[0]);
			}
			if (caps->IsWidthSupported) {
				input_report_abs(touch->input_dev,
					ABS_TOOL_WIDTH, 
					c.additionalInfo.width[0]);
			}

			/* Report down or up flag */
			input_report_key(touch->input_dev,
					BTN_TOUCH, ToolDown[0]);

			/* report co-ordinates for the 2nd finger */
			if (fingers == 2) {
				input_report_abs(touch->input_dev,
					ABS_HAT0X, x[1]); // x
				input_report_abs(touch->input_dev,
					ABS_HAT0Y, y[1]); // y
				input_report_key(touch->input_dev,
					BTN_2, ToolDown[1]);
				prev_x[1] = x[1];
				prev_y[1] = y[1];
			} else if (((fingers == 1) || (fingers == 0)) &&
				   (ToolDown[1] == NV_TRUE)) {
				ToolDown[1] = NV_FALSE;
				input_report_abs(touch->input_dev,
					ABS_HAT0X, prev_x[1]); // x
				input_report_abs(touch->input_dev,
					ABS_HAT0Y, prev_y[1]); // y
				input_report_key(touch->input_dev,
					BTN_2, ToolDown[1]);
			}
			input_sync(touch->input_dev);

DoneWithSample:
			bKeepReadingSamples = NV_FALSE;
			if (!touch->bPollingMode &&
				!NvOdmTouchHandleInterrupt(touch->hTouchDevice)) {
				/* Some more data to read keep going */
				bKeepReadingSamples = NV_TRUE;
			}
		}
	}

	return 0;
}

static int __init tegra_touch_probe(struct platform_device *pdev)
{
	struct tegra_touch_driver_data *touch = NULL;
	struct input_dev *input_dev = NULL;
	int err;
	NvOdmTouchCapabilities *caps;

	touch = kzalloc(sizeof(struct tegra_touch_driver_data), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (input_dev == NULL || touch == NULL) {
		input_free_device(input_dev);
		kfree(touch);
		err = -ENOMEM;
		pr_err("tegra_touch_probe: Failed to allocate input device\n");
		return err;
	}
	touch->semaphore = NvOdmOsSemaphoreCreate(0);
	if (!touch->semaphore) {
		err = -1;
		pr_err("tegra_touch_probe: Semaphore creation failed\n");
		goto err_semaphore_create_failed;
	}

	if (!NvOdmTouchDeviceOpen(&touch->hTouchDevice)) {
		err = -1;
		pr_err("tegra_touch_probe: NvOdmTouchDeviceOpen failed\n");
		goto err_open_failed;
	}
	touch->bPollingMode = NV_FALSE;
	if (!NvOdmTouchEnableInterrupt(touch->hTouchDevice, touch->semaphore)) {
		err = -1;
		pr_err("tegra_touch_probe: Interrupt failed, polling mode\n");
		touch->bPollingMode = NV_TRUE;
		touch->pollingIntervalMS = 10;
	}

	touch->task =
		kthread_create(tegra_touch_thread, touch, "tegra_touch_thread");

	if(touch->task == NULL) {
		err = -1;
		goto err_kthread_create_failed;
	}
	wake_up_process( touch->task );

	touch->input_dev = input_dev;
	touch->input_dev->name = NVODM_TOUCH_NAME;

	/* Will generate sync at the end of all input */
	set_bit(EV_SYN, touch->input_dev->evbit);

	/* Event is key input type */
	set_bit(EV_KEY, touch->input_dev->evbit);
	/* virtual key is BTN_TOUCH */
	set_bit(BTN_TOUCH, touch->input_dev->keybit);
	/* Input values are in absoulte values */
	set_bit(EV_ABS, touch->input_dev->evbit);

	NvOdmTouchDeviceGetCapabilities(touch->hTouchDevice, &touch->caps);

	caps = &touch->caps;

	if (caps->Orientation & NvOdmTouchOrientation_XY_SWAP) {
		touch->MaxY = caps->XMaxPosition;
		touch->MinY = caps->XMinPosition;
		touch->MaxX = caps->YMaxPosition;
		touch->MinX = caps->YMinPosition;

	} else {
		touch->MaxX = caps->XMaxPosition;
		touch->MinX = caps->XMinPosition;
		touch->MaxY = caps->YMaxPosition;
		touch->MinY = caps->YMinPosition;
	}

	input_set_abs_params(touch->input_dev, ABS_X, touch->MinX,
		touch->MaxX, 0, 0);
	input_set_abs_params(touch->input_dev, ABS_Y, touch->MinY,
		touch->MaxY, 0, 0);
	input_set_abs_params(touch->input_dev, ABS_HAT0X, touch->MinX,
		touch->MaxX, 0, 0);
	input_set_abs_params(touch->input_dev, ABS_HAT0Y, touch->MinY,
		touch->MaxY, 0, 0);

	if (caps->IsPressureSupported)
		input_set_abs_params(touch->input_dev, ABS_PRESSURE, 0, 
			caps->MaxNumberOfPressureReported, 0, 0);
	if (caps->IsWidthSupported)
		input_set_abs_params(touch->input_dev, ABS_TOOL_WIDTH, 0, 
			caps->MaxNumberOfWidthReported, 0, 0);

	platform_set_drvdata(pdev, touch);

	err = input_register_device(input_dev);
	if (err)
	{
		pr_err("tegra_touch_probe: Unable to register input device\n");
		goto err_input_register_device_failed;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
        touch->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
        touch->early_suspend.suspend = tegra_touch_early_suspend;
        touch->early_suspend.resume = tegra_touch_late_resume;
        register_early_suspend(&touch->early_suspend);
#endif
	printk(KERN_INFO NVODM_TOUCH_NAME 
		": Successfully registered the ODM touch driver %x\n", (NvU32)touch->hTouchDevice);
	return 0;

err_input_register_device_failed:
	NvOdmTouchDeviceClose(touch->hTouchDevice);
err_kthread_create_failed:
	/* FIXME How to destroy the thread? Maybe we should use workqueues? */
err_open_failed:
	NvOdmOsSemaphoreDestroy(touch->semaphore);
err_semaphore_create_failed:
	kfree(touch);
	input_free_device(input_dev);
	return err;
}

static int tegra_touch_remove(struct platform_device *pdev)
{
	struct tegra_touch_driver_data *touch = platform_get_drvdata(pdev);

#ifdef CONFIG_HAS_EARLYSUSPEND
        unregister_early_suspend(&touch->early_suspend);
#endif
        touch->shutdown = 1;
	/* FIXME How to destroy the thread? Maybe we should use workqueues? */
	input_unregister_device(touch->input_dev);
	/* NvOsSemaphoreDestroy(touch->semaphore); */
	input_unregister_device(touch->input_dev);
	kfree(touch);
	return 0;
}

static struct platform_driver tegra_touch_driver = {
	.probe	  = tegra_touch_probe,
	.remove	 = tegra_touch_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend = tegra_touch_suspend,
	.resume	 = tegra_touch_resume,
#endif
	.driver	 = {
		.name   = "tegra_touch",
	},
};

static int __devinit tegra_touch_init(void)
{
	return platform_driver_register(&tegra_touch_driver);
}

static void __exit tegra_touch_exit(void)
{
	platform_driver_unregister(&tegra_touch_driver);
}

module_init(tegra_touch_init);
module_exit(tegra_touch_exit);

MODULE_DESCRIPTION("Tegra ODM touch driver");

