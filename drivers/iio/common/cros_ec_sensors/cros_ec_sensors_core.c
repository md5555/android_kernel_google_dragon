/*
 * cros_ec_sensors_core - Common function for Chrome OS EC sensor driver.
 *
 * Copyright (C) 2015 Google, Inc
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * This driver uses the cros-ec interface to communicate with the Chrome OS
 * EC about accelerometer data. Accelerometer access is presented through
 * iio sysfs.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/kernel.h>
#include <linux/mfd/cros_ec.h>
#include <linux/mfd/cros_ec_commands.h>
#include <linux/mfd/cros_ec_dev.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/platform_device.h>

#include "cros_ec_sensors_core.h"

int cros_ec_sensors_core_init(struct platform_device *pdev,
			      struct cros_ec_sensors_core_state *state)
{
	struct cros_ec_dev *ec = dev_get_drvdata(pdev->dev.parent);

	state->ec = ec->ec_dev;
	state->resp = devm_kzalloc(&pdev->dev, state->ec->max_response,
			GFP_KERNEL);
	if (state->resp == NULL)
		return -ENOMEM;

	mutex_init(&state->cmd_lock);
	/* Set up the host command structure. */
	state->msg.version = 2;
	state->msg.command = EC_CMD_MOTION_SENSE_CMD + ec->cmd_offset;
	state->msg.outdata = (u8 *)&state->param;
	state->msg.outsize = sizeof(struct ec_params_motion_sense);
	state->msg.indata = (u8 *)state->resp;
	state->msg.insize = state->ec->max_response;

	return 0;
}
EXPORT_SYMBOL_GPL(cros_ec_sensors_core_init);

/*
 * send_motion_host_cmd - send motion sense host command
 *
 * @st Pointer to state information for device.
 * @return 0 if ok, -ve on error.
 *
 * Note, when called, the sub-command is assumed to be set in param->cmd.
 */
int send_motion_host_cmd(struct cros_ec_sensors_core_state *state)
{
	int ret;
	/* Send host command. */
	ret = cros_ec_cmd_xfer_status(state->ec, &state->msg);

	/* Send host command. */
	if (ret > 0)
		/* We have some data */
		return 0;
	else
		return -EIO;
}
EXPORT_SYMBOL_GPL(send_motion_host_cmd);


