/*
 * drivers/w1/masters/tegra-w1.c
 *
 * ONE WIRE (OWR) bus driver for internal OWR controllers in NVIDIA Tegra SoCs
 *
 * Copyright (C) 2010 NVIDIA Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
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
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/tegra_devices.h>
#include <asm/uaccess.h>

#include "../w1.h"
#include "../w1_int.h"
#include "../w1_log.h"

#include <mach/nvrm_linux.h>
#include <nvrm_module.h>
#include <nvos.h>
#include <nvodm_query_discovery.h>

struct tegra_w1_dev
{
	NvRmOwrHandle		OwrHandle;
	NvOdmOwrPinMap		pin_map;
	struct w1_bus_master	bus_master;
};

static u8 tegra_w1_read_byte(void *data)
{
	struct tegra_w1_dev *dev = data;
	NvRmOwrTransactionInfo tInfo;
	NvError err;
	u8 buffer[1];

	tInfo.Flags = NvRmOwr_ReadByte;
	tInfo.NumBytes = 1;
	tInfo.Address = 0;
	tInfo.Offset = 0;

	err = NvRmOwrTransaction(dev->OwrHandle, dev->pin_map,
			buffer, tInfo.NumBytes, &tInfo, 1);
	if (err != NvSuccess)
	{
		printk(KERN_ERR "tegra_w1_read_byte failed 0x%x\r\n", err);
		err = -EIO;
	}

	if (!err)
		return buffer[0];
	else
		return 0;
}

static void tegra_w1_write_byte(void *data, u8 a_byte)
{
	struct tegra_w1_dev *dev = data;
	NvRmOwrTransactionInfo tInfo;
	NvError err;
	u8 buffer[1];

	tInfo.Flags = NvRmOwr_WriteByte;
	tInfo.NumBytes = 1;
	tInfo.Address = 0;
	tInfo.Offset = 0;
	buffer[0] = a_byte;

	err = NvRmOwrTransaction(dev->OwrHandle, dev->pin_map,
			buffer, tInfo.NumBytes, &tInfo, 1);
	if (err != NvSuccess)
	{
		printk(KERN_ERR "tegra_w1_write_byte failed 0x%x\r\n", err);
		err = -EIO;
	}
}

static u8 tegra_w1_read_bit(void *data)
{
	struct tegra_w1_dev *dev = data;
	NvRmOwrTransactionInfo tInfo;
	NvError err;
	u8 buffer[1];

	tInfo.Flags = NvRmOwr_ReadBit;
	tInfo.NumBytes = 1;
	tInfo.Address = 0;
	tInfo.Offset = 0;

	err = NvRmOwrTransaction(dev->OwrHandle, dev->pin_map,
			buffer, tInfo.NumBytes, &tInfo, 1);
	if (err != NvSuccess)
	{
		printk(KERN_ERR "tegra_w1_read_bit failed 0x%x\r\n", err);
		err = -EIO;
	}

	if (!err)
		return (buffer[0] & 0x1);
	else
		return 0;
}

static void tegra_w1_write_bit(void *data, u8 bit)
{
	struct tegra_w1_dev *dev = data;
	NvRmOwrTransactionInfo tInfo;
	NvError err;
	u8 buffer[1];

	tInfo.Flags = NvRmOwr_WriteBit;
	tInfo.NumBytes = 1;
	tInfo.Address = 0;
	tInfo.Offset = 0;
	buffer[0] = bit & 0x1;

	err = NvRmOwrTransaction(dev->OwrHandle, dev->pin_map,
			buffer, tInfo.NumBytes, &tInfo, 1);
	if (err != NvSuccess)
	{
		printk(KERN_ERR "tegra_w1_write_bit failed 0x%x\r\n", err);
		err = -EIO;
	}
}

/* Performs a write-0 or write-1 cycle and samples the level */
static u8 tegra_w1_touch_bit(void *data, u8 bit)
{
	struct tegra_w1_dev *dev = data;

	if (bit)
	{
		return tegra_w1_read_bit(dev);
	}
	else
	{
		tegra_w1_write_bit(dev, 0);
		return 0;
	}
}

static u8 tegra_w1_reset_bus(void *data)
{
	struct tegra_w1_dev *dev = data;
	NvRmOwrTransactionInfo tInfo;
	NvError err;
	u8 buffer[1];

	tInfo.Flags = NvRmOwr_CheckPresence;
	tInfo.NumBytes = 1;
	tInfo.Address = 0;
	tInfo.Offset = 0;

	err = NvRmOwrTransaction(dev->OwrHandle, dev->pin_map,
			buffer, tInfo.NumBytes, &tInfo, 1);
	if (err != NvSuccess)
	{
		printk(KERN_ERR "tegra_w1_reset_bus failed 0x%x\r\n", err);
		err = -EIO;
	}

	if (!err)
	{
		/* Device present */
		return 0;
	}
	else
	{
		/* No Device present */
		return 1;
	}
}

static int tegra_w1_probe(struct platform_device *pdev)
{
	struct tegra_w1_dev *dev;
	struct tegra_w1_platform_data *pdata = pdev->dev.platform_data;
	int ret;
	NvError err;

	printk(KERN_INFO "tegra_w1_probe\r\n");
	printk(KERN_INFO "Instance = %d, PinMuxConfig = %d\r\n",
			pdata->Instance, pdata->PinMuxConfig);

	if (pdata == NULL)
		return -ENODEV;

	dev = kzalloc(sizeof(struct tegra_w1_dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	err = NvRmOwrOpen(s_hRmGlobal, pdata->Instance, &dev->OwrHandle);
	if (err)
	{
		ret = -ENODEV;
		printk(KERN_INFO "Failed to open NvRmOwrOpen - returned %d\n", err);
		goto err_rmapi_failed;
	}

	dev->pin_map = pdata->PinMuxConfig;
	dev->bus_master.data = dev;
	dev->bus_master.read_byte = tegra_w1_read_byte;
	dev->bus_master.write_byte = tegra_w1_write_byte;
	dev->bus_master.read_bit = tegra_w1_read_bit;
	dev->bus_master.write_bit = tegra_w1_write_bit;
	dev->bus_master.touch_bit = tegra_w1_touch_bit;
	dev->bus_master.reset_bus = tegra_w1_reset_bus;

	if (tegra_w1_reset_bus(dev))
	{
		printk(KERN_INFO "No Device Present\n");
		ret = -ENODEV;
		goto err_device_not_found;
	}

	ret = w1_add_master_device(&dev->bus_master);
	if (ret)
	{
		printk(KERN_INFO "w1_add_master_device - failed %d\r\n", ret);
		goto err_w1_add_master_device_failed;
	}

	platform_set_drvdata(pdev, dev);

	return 0;

err_w1_add_master_device_failed:
err_device_not_found:
	NvRmOwrClose(dev->OwrHandle);
err_rmapi_failed:
	kfree(dev);
	return ret;
}

static int
tegra_w1_remove(struct platform_device *pdev)
{
	struct tegra_w1_dev *dev = platform_get_drvdata(pdev);

	NvRmOwrClose(dev->OwrHandle);
	w1_remove_master_device(&dev->bus_master);
	platform_set_drvdata(pdev, NULL);
	kfree(dev);
	return 0;
}

static int tegra_w1_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int tegra_w1_resume(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver tegra_w1_driver = {
	.probe   = tegra_w1_probe,
	.remove  = tegra_w1_remove,
	.suspend = tegra_w1_suspend,
	.resume  = tegra_w1_resume,
	.driver  =
	{
	.name  = "tegra_w1",
	.owner = THIS_MODULE,
	},
};

static int __init
tegra_w1_init(void)
{
	return platform_driver_register(&tegra_w1_driver);
}
module_init(tegra_w1_init);

static void __exit tegra_w1_exit(void)
{
	platform_driver_unregister(&tegra_w1_driver);
}
module_exit(tegra_w1_exit);
