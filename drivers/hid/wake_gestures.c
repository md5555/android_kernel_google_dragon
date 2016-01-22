/*
 * drivers/input/touchscreen/wake_gestures.c
 *
 *
 * Copyright (c) 2013, Dennis Rassmann <showp1984@gmail.com>
 * Copyright (c) 2013-15 Aaron Segaert <asegaert@gmail.com>
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/wake_gestures.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/input.h>
#include <linux/hrtimer.h>
#include <asm-generic/cputime.h>
#include <linux/wakelock.h>

/* Tuneables */
#define WG_DEBUG		1
#define WG_DEFAULT		0

/* shamu */
#define SWEEP_Y_MAX             1800
#define SWEEP_X_MAX             2560
#define SWEEP_EDGE		100
#define SWEEP_Y_LIMIT           SWEEP_Y_MAX-SWEEP_EDGE
#define SWEEP_X_LIMIT           SWEEP_X_MAX-SWEEP_EDGE
#define DT2W_FEATHER		150
#define DT2W_TIME 		600
#define DT2W_WINDOW		80

#define LOGTAG			"DoubleTapWake"

/* Resources */
static bool process_touches = true;
static unsigned long pwrtrigger_time[2] = {0, 0};
static unsigned long long tap_time_pre = 0;
static int touch_seqence = 0, x_pre = 0, y_pre = 0;
static int touch_x = 0, touch_y = 0;

static DEFINE_MUTEX(pwrkeyworklock);

static struct input_dev * wake_dev;
static struct device * pdev_dev;
static struct workqueue_struct *dt2w_input_wq;
static struct work_struct dt2w_input_work;
static struct wake_lock dt2w_wakelock;

extern bool scr_is_suspended;
extern bool wake_gestures_enabled;

void wg_setdev(struct input_dev * input_device) {
	wake_dev = input_device;
	return;
}

void wg_setpdev(struct device * pdevice) {
	pdev_dev = pdevice;
	return;
}


bool scr_suspended(void) {
	return scr_is_suspended;
}

/* PowerKey work func */
static void dt2w_wake_event(struct work_struct * dt2w_wake_event_work) {

	if (!mutex_trylock(&pwrkeyworklock))
        	return;

	input_report_key(wake_dev, KEY_WAKEUP, 1);
	input_sync(wake_dev);
	input_report_key(wake_dev, KEY_WAKEUP, 0);
	input_sync(wake_dev);

    	mutex_unlock(&pwrkeyworklock);

	return;
}
static DECLARE_WORK(dt2w_wake_event_work, dt2w_wake_event);

/* PowerKey trigger */
static void dt2w_trigger_wakeup(void) {

	pwrtrigger_time[1] = pwrtrigger_time[0];
	pwrtrigger_time[0] = jiffies_64;
	
	if (pwrtrigger_time[0] - pwrtrigger_time[1] < DT2W_WINDOW)
		return;

	schedule_work(&dt2w_wake_event_work);
        return;
}


/* Doubletap2wake */
static void dt2w_reset(void) {
	if (wake_lock_active(&dt2w_wakelock))
		wake_unlock(&dt2w_wakelock);
	process_touches = true;
	touch_seqence = 0;
	tap_time_pre = 0;
	x_pre = 0;
	y_pre = 0;
}

static unsigned int dt2w_calc_touch_feather(int coord, int prev_coord) {
	int calc_coord = 0;
	calc_coord = coord-prev_coord;
	if (calc_coord < 0)
		calc_coord = calc_coord * (-1);
	return calc_coord;
}

/* init a new touch */
static void dt2w_register_touch(int x, int y) {
	tap_time_pre = jiffies_64;
	x_pre = x;
	y_pre = y;
	touch_seqence++;
	wake_lock_timeout(&dt2w_wakelock, HZ/2);
}

/* Doubletap2wake main function */
static void dt2w_detect_wake(int x, int y, bool st)
{
        bool single_touch = st;

	if (x < SWEEP_EDGE || x > SWEEP_X_LIMIT)
       		goto cancel;
	if (y < SWEEP_EDGE || y > SWEEP_Y_LIMIT)
       		goto cancel;

	if ((single_touch) && (wake_gestures_enabled) && (process_touches)) {

		if (touch_seqence == 0) {
			dt2w_register_touch(x, y);
		} else if (touch_seqence == 1) {
			if ((dt2w_calc_touch_feather(x, x_pre) < DT2W_FEATHER) &&
			    (dt2w_calc_touch_feather(y, y_pre) < DT2W_FEATHER) &&
			    ((jiffies_64-tap_time_pre) < DT2W_TIME))
				touch_seqence++;
			else {
				dt2w_reset();
				dt2w_register_touch(x, y);
			}
		}

		if (touch_seqence == 2) {
			process_touches = false;
			dt2w_reset();
			dt2w_trigger_wakeup();
			return;
		}
	}

cancel:

	return;
}

static void dt2w_input_callback(struct work_struct *unused)
{
	dt2w_detect_wake(touch_x, touch_y, true);
}

static void dt2w_input_event(struct input_handle *handle, unsigned int type,
				unsigned int code, int value)
{
	if(!scr_suspended() || !wake_gestures_enabled) {
		return;
	}

	if (code == ABS_MT_SLOT) {
		return;
	}

	if (code == ABS_MT_TRACKING_ID && value == -1) {
		process_touches = true;
		queue_work_on(0, dt2w_input_wq, &dt2w_input_work);
	}

	if (code == ABS_MT_POSITION_X) {
		touch_x = value;
	}

	if (code == ABS_MT_POSITION_Y) {
		touch_y = value;
	}
}

static int dt2w_input_connect(struct input_handler *handler,
				struct input_dev *dev, const struct input_device_id *id) {
	struct input_handle *handle;
	int error;

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "wg";

	error = input_register_handle(handle);
	if (error)
		goto err2;

	error = input_open_device(handle);
	if (error)
		goto err1;

	printk("DoubleTapWake: Connected\n");

	return 0;
err1:
	input_unregister_handle(handle);
err2:
	kfree(handle);
	return error;
}

static void dt2w_input_disconnect(struct input_handle *handle) {
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id wg_ids[] = {
	{ 
		.flags = INPUT_DEVICE_ID_MATCH_KEYBIT,
		.keybit = { [BIT_WORD(0x008f)] = BIT_MASK(0x008f) },
	},
};

static struct input_handler dt2w_input_handler = {
	.event		= dt2w_input_event,
	.connect	= dt2w_input_connect,
	.disconnect	= dt2w_input_disconnect,
	.name		= "dt2w_input_proxy",
	.id_table	= wg_ids,
};

/*
 * INIT / EXIT stuff below here
 */
static int __init wake_gestures_init(void)
{
	int rc = 0;

	rc = input_register_handler(&dt2w_input_handler);
	if (rc) {
		pr_err("%s: Failed to register dt2w_input_handler\n", __func__);
		return -EAGAIN;
	}

	dt2w_input_wq = create_workqueue("dt2wiwq");
	if (!dt2w_input_wq) {
		pr_err("%s: Failed to create dt2wiwq workqueue\n", __func__);
		return -EFAULT;
	}
	INIT_WORK(&dt2w_input_work, dt2w_input_callback);
		
	wake_lock_init(&dt2w_wakelock, WAKE_LOCK_SUSPEND, "dt2w_wakelock");
	
	return 0;
}

static void __exit wake_gestures_exit(void)
{
	input_unregister_handler(&dt2w_input_handler);
	destroy_workqueue(dt2w_input_wq);
	input_free_device(wake_dev);
	wake_lock_destroy(&dt2w_wakelock);

	return;
}

module_init(wake_gestures_init);
module_exit(wake_gestures_exit);

