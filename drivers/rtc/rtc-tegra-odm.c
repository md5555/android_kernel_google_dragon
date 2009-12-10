/*
 * drivers/rtc/rtc-tegra-odm.c
 *
 * Tegra ODM kit wrapper for RTC functionality implemented in Tegra
 * ODM PMU adaptation
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/rtc.h>
#include <linux/bcd.h>
#include <linux/platform_device.h>

#include <nvodm_pmu.h>

#define SET_YEAR(Y)  (Y-1900)
#define SET_MONTH(M) (M-1)

/* Create a custom rtc structrue and move this to that structure */
static NvOdmPmuDeviceHandle hPmu = NULL;

static int tegra_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	NvU32 now;

	if (hPmu == NULL)
		return -1;

	if (!NvOdmPmuReadRtc(hPmu, &now)) {
		printk("NvOdmPmuReadRtc failed\n");
		return -1;
	}

	 if (!now) {
		tm->tm_sec = 0;
		tm->tm_min = 0;
		tm->tm_hour = 0;
		tm->tm_mday = 1;
		tm->tm_mon = SET_MONTH(5);
		tm->tm_year = SET_YEAR(2009);
		tm->tm_wday = 0;
		tm->tm_yday = 0;
		tm->tm_isdst = 0;
	} else {
		rtc_time_to_tm(now, tm);
	}

	return 0;
}

static int tegra_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	unsigned long now;
	int ret;

	if (hPmu == NULL)
		return -1;

	ret = rtc_tm_to_time(tm, &now);
	if (ret != 0)
		return -1;

	if (!NvOdmPmuWriteRtc(hPmu, (NvU32)now)) {
		printk("NvOdmPmuWriteRtc failed\n");
		return -1;
	}
	return 0;
}

static struct rtc_class_ops tegra_rtc_ops = {
	.read_time	= tegra_rtc_read_time,
	.set_time	= tegra_rtc_set_time,
};

static int __init tegra_rtc_probe(struct platform_device *pdev)
{
	struct rtc_device *rtc;

	if (NvOdmPmuDeviceOpen(&hPmu) == NV_FALSE) {
		pr_debug("%s: NvOdmPmuDeviceOpen failed\n", pdev->name);
		return -ENXIO;
	}

	rtc = rtc_device_register(pdev->name, &pdev->dev,
		&tegra_rtc_ops, THIS_MODULE);

	if (IS_ERR(rtc)) {
		pr_debug("%s: can't register RTC device, err %ld\n",
			pdev->name, PTR_ERR(rtc));
		NvOdmPmuDeviceClose(hPmu);
		return -1;
	}
	platform_set_drvdata(pdev, rtc);

	return 0;
}

static int __exit tegra_rtc_remove(struct platform_device *pdev)
{
	struct rtc_device *rtc = platform_get_drvdata(pdev);;

	rtc_device_unregister(rtc);
	return 0;
}

#ifdef CONFIG_PM

static int tegra_rtc_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int tegra_rtc_resume(struct platform_device *pdev)
{
	return 0;
}

#endif

static void tegra_rtc_shutdown(struct platform_device *pdev)
{
}

MODULE_ALIAS("platform:tegra_rtc");

static struct platform_driver tegra_rtc_driver = {
	.remove		= __exit_p(tegra_rtc_remove),
	.shutdown	= tegra_rtc_shutdown,
#ifdef CONFIG_PM
	.suspend	= tegra_rtc_suspend,
	.resume		= tegra_rtc_resume,
#endif
	.driver		=  {
		.name  = "tegra_rtc",
		.owner = THIS_MODULE,
	},
};

static int __init rtc_init(void)
{
	return platform_driver_probe(&tegra_rtc_driver, tegra_rtc_probe);
}
module_init(rtc_init);

static void __exit rtc_exit(void)
{
	platform_driver_unregister(&tegra_rtc_driver);
}
module_exit(rtc_exit);

