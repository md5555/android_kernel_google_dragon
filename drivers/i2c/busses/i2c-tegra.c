/*
 * drivers/i2c/busses/i2c-tegra.c
 *
 * I2C bus driver for internal I2C controllers in NVIDIA Tegra SoCs
 *
 * Copyright (C) 2009 NVIDIA Corporation
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

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/tegra_devices.h>
#include <asm/uaccess.h>

#include <mach/nvrm_linux.h>
#include <nvrm_module.h>
#include <nvos.h>
#include <nvodm_query_discovery.h>

#define I2C_SMBUS_TRASNSPORT_GUID NV_ODM_GUID('I','2','c','S','m','B','u','s')

struct tegra_i2c_dev
{
	struct device		*dev;
	struct i2c_adapter	adapter;
	NvRmI2cHandle		I2cHandle;
	int			clock_in_khz;
	NvOdmI2cPinMap		pin_map;
};

static int tegra_i2c_xfer(struct i2c_adapter *adap,
	struct i2c_msg msgs[], int num)
{
	struct tegra_i2c_dev  *dev = i2c_get_adapdata(adap);
	NvRmI2cTransactionInfo *tInfo=NULL;
	NvU32 i;
	NvU32 len=0;
	int err = 0;
	int nverror = NvSuccess;
	NvU8 *tempBuffer,*buffer=NULL;

	tInfo = kzalloc(sizeof(NvRmI2cTransactionInfo)*num,GFP_KERNEL);
	if(!tInfo) {
		printk(KERN_ERR "tegra_i2c_xfer: Could not allocate memory\n");
		err = -ENOMEM;
		goto end;
	}

	for(i = 0 ; i < num ; i++) {
		len += msgs[i].len;
	}

	buffer = kzalloc(len,GFP_KERNEL);
	if(!buffer) {
		printk(KERN_ERR "tegra_i2c_xfer: Could not allocate memory\n");
		err = -ENOMEM;
		goto end;
	}

	/*
	 * NvRmI2cTransactions accepts all the messages at once. We can not
	 * send 1 message at a time. Hence clubbing all messages and
	 * representing them in a form that NvRmI2cTransactions understands
	 * */

	tempBuffer = buffer;

	for (i = 0 ;  i < num ; i++) {
		if (msgs[i].flags & I2C_M_NOSTART) {
			tInfo[i].Flags |= NVRM_I2C_NOSTOP;
		}
		if (msgs[i].flags & I2C_M_IGNORE_NAK) {
			tInfo[i].Flags |= NVRM_I2C_NOACK;
		}
		if (msgs[i].flags & I2C_M_RD) {
			tInfo[i].Flags |= NVRM_I2C_READ;
		}
		else {
			tInfo[i].Flags |= NVRM_I2C_WRITE;
			memcpy(tempBuffer,msgs[i].buf,msgs[i].len);
		}
		tempBuffer += msgs[i].len;
		tInfo[i].NumBytes = msgs[i].len;
		tInfo[i].Address = msgs[i].addr << 1;
		tInfo[i].Is10BitAddress =  (NvBool)(msgs[i].flags &  I2C_M_TEN);
	}

	nverror = NvRmI2cTransaction(dev->I2cHandle, dev->pin_map, 1000,
			dev->clock_in_khz, buffer, len, tInfo, num);

	if (nverror != NvSuccess) {
		switch (nverror) {
		case NvError_I2cDeviceNotFound:
			printk(KERN_ERR "NvRmI2cTransaction failed: No I2C "
					"device claimed the address 0x%x on "
					"adapter %d\n", msgs[i].addr,adap->nr);
			err = -ENXIO;
			break;
		case NvError_I2cReadFailed:
			printk(KERN_ERR "NvRmI2c Read failed %x\n", err);
			err = -EIO;
			break;
		case NvError_I2cWriteFailed:
			printk(KERN_ERR "NvRmI2c Write failed %x\n", err);
			err = -EIO;
			break;
		case NvError_Timeout:
			printk(KERN_ERR "NvRmI2c Timeout %x\n", err);
			err = -ETIMEDOUT;
			break;
		default:
			printk(KERN_ERR "NvRmI2c Transaction Error %x\n",err);
			err = -EIO;
			break;
		}
		goto end;
	}

	tempBuffer = buffer;

	for (i = 0 ; i < num ; i++) {
		if (tInfo[i].Flags & NVRM_I2C_READ) {
			memcpy(msgs[i].buf,tempBuffer,tInfo[i].NumBytes);
		}
		tempBuffer += tInfo[i].NumBytes;
	}

end:
	if(tInfo != NULL)
		kfree(tInfo);

	if(buffer != NULL)
		kfree(buffer);

	if (!err)
		return num;
	else
		return err;
}

static u32 tegra_i2c_func(struct i2c_adapter *adap)
{
	/* FIXME: For now keep it simple and don't support protocol mangling
	   features */
	return I2C_FUNC_I2C;
}

static const struct i2c_algorithm tegra_i2c_algo = {
	.master_xfer	= tegra_i2c_xfer,
	.functionality	= tegra_i2c_func,
};

static int tegra_i2c_probe(struct platform_device *pdev)
{
	struct tegra_i2c_dev *dev;
	struct tegra_i2c_platform_data *pdata = pdev->dev.platform_data;
	int ret;
	NvError err;
	const NvOdmPeripheralConnectivity *pConn = NULL;
	int i = 0;

	printk(KERN_INFO "tegra_i2c_probe\n");

	if (pdata == NULL)
		return -ENODEV;

	// Check if the I2C instance is dedicated for NvEc communication.
	// If yes, then don't allow it to register as platform driver.
	pConn = NvOdmPeripheralGetGuid(I2C_SMBUS_TRASNSPORT_GUID);
	if (pConn) {
		const NvOdmIoAddress *addr_list = pConn->AddressList;
		for (i = 0; i < pConn->NumAddress; i++, addr_list++) {
			if (addr_list->Interface == NvOdmIoModule_I2c &&
				addr_list->Instance == pdata->Instance) {
				return -ENODEV;
			}
		}
	}

	dev = kzalloc(sizeof(struct tegra_i2c_dev), GFP_KERNEL);
	if (!dev)
	{
		ret = -ENOMEM;
		return ret;
	}

	err = NvRmI2cOpen(s_hRmGlobal, pdata->IoModuleID,
		pdata->Instance, &dev->I2cHandle);
	if (err)
	{
		ret = -ENODEV;
		printk("Failed to open NvRmI2cOpen - returned %d\n", err);
		goto err_rmapi_failed;
	}
	dev->clock_in_khz = pdata->ClockInKHz;
	dev->pin_map = pdata->PinMuxConfig;

	dev->dev = &pdev->dev;
	platform_set_drvdata(pdev, dev);

	i2c_set_adapdata(&dev->adapter, dev);
	dev->adapter.algo = &tegra_i2c_algo;
	strncpy(dev->adapter.name, "Tegra I2C adapter",
		sizeof(dev->adapter.name));

	dev->adapter.nr = pdev->id;
	ret = i2c_add_numbered_adapter(&dev->adapter);
	if (ret)
	{
		dev_err(&pdev->dev, "i2c_add_adapter failed\n");
		goto err_i2c_add_adapter_failed;
	}

	return 0;

err_rmapi_failed:
err_i2c_add_adapter_failed:
	kfree(dev);
	return ret;
}

static int
tegra_i2c_remove(struct platform_device *pdev)
{
	struct tegra_i2c_dev *dev = platform_get_drvdata(pdev);

	NvRmI2cClose(dev->I2cHandle);
	platform_set_drvdata(pdev, NULL);
	i2c_del_adapter(&dev->adapter);
	kfree(dev);
	return 0;
}

static int tegra_i2c_suspend(struct platform_device *pdev, pm_message_t state)
{
	/* FIXME to be implemented */
	return 0;
}

static int tegra_i2c_resume(struct platform_device *pdev)
{
	/* FIXME to be implemented */
	return 0;
}

static struct platform_driver tegra_i2c_driver = {
	.probe   = tegra_i2c_probe,
	.remove  = tegra_i2c_remove,
	.suspend = tegra_i2c_suspend,
	.resume  = tegra_i2c_resume,
	.driver  =
	{
		.name  = "tegra_i2c",
		.owner = THIS_MODULE,
	},
};

/* I2C may be needed to bring up other drivers */
static int __init
tegra_i2c_init_driver(void)
{
	return platform_driver_register(&tegra_i2c_driver);
}
module_init(tegra_i2c_init_driver);

static void __exit tegra_i2c_exit_driver(void)
{
	platform_driver_unregister(&tegra_i2c_driver);
}
module_exit(tegra_i2c_exit_driver);
