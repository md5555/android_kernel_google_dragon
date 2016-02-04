/*
 * Copyright(c) 2015 Hauke Mehrtens <hauke@hauke-m.de>
 *
 * Backport functionality introduced in Linux 4.5.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/leds.h>
#include <linux/export.h>
#include <linux/errno.h>
#include <linux/fs.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,19,0)
int led_set_brightness_sync(struct led_classdev *led_cdev,
			    enum led_brightness value)
{
	if (led_cdev->blink_delay_on || led_cdev->blink_delay_off)
		return -EBUSY;

	led_cdev->brightness = min(value, led_cdev->max_brightness);

	if (led_cdev->flags & LED_SUSPENDED)
		return 0;

	if (led_cdev->brightness_set_sync)
		return led_cdev->brightness_set_sync(led_cdev,
							 led_cdev->brightness);
	return -ENOTSUPP;
}
EXPORT_SYMBOL_GPL(led_set_brightness_sync);
#endif /* >= 3.19 */

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,2,0)
/**
 * no_seek_end_llseek - llseek implementation for fixed-sized devices
 * @file:	file structure to seek on
 * @offset:	file offset to seek to
 * @whence:	type of seek
 *
 */
loff_t no_seek_end_llseek(struct file *file, loff_t offset, int whence)
{
	switch (whence) {
	case SEEK_SET: case SEEK_CUR:
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,6,0)
		return generic_file_llseek_size(file, offset, whence,
						~0ULL, 0);
#else
		return generic_file_llseek_size(file, offset, whence,
						~0ULL);
#endif
	default:
		return -EINVAL;
	}
}
EXPORT_SYMBOL_GPL(no_seek_end_llseek);
#endif /* >= 3.2 */
