/*
* Copyright (c) 2014 MediaTek Inc.
* Author: Chiawen Lee <chiawen.lee@mediatek.com>
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*/
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>

#include <linux/io.h>

#include "mt8173_mfgsys.h"
#include "mt8173_mfgdvfs.h"

#include "rgxdevice.h"
#include "pvrsrv_error.h"
#include "allocmem.h"
#include "osfunc.h"
#include "pvrsrv.h"
#include "power.h"
#include "rgxhwperf.h"

#define REG_GPU_POWER_CAL_EN     0x6320
#define REG_GPU_POWER_CAL_RESULT 0x6328

#define	REG_MFG_GPIO_REG_OFFSET  0x30
#define VAL_MFG_GPIO_INPUT_REG   BIT(0)
#define VAL_MFG_GPIO_OUTPUT_REG  BIT(16)
#define VAL_MFG_GPIO_OUTPUT_ACK  BIT(17)
#define VAL_MFG_GPIO_OUTPUT_DATA BIT(24)

#define HZ_TO_KHZ(x) (x / 1000)

static void mtk_gpu_dfs_mmpll(struct clk *mmpll, unsigned int freq_new)
{
	unsigned int err;

	/* mmpll setting is on mt_freghopping.c  */
	mtk_mfg_debug("mtk_gpu_dfs_mmpll: freq_new = %d\n", freq_new);

	err = clk_set_rate(mmpll, freq_new * 1000);
	if (err)
		pr_err("mtk_gpu_dfs_mmpll err %d\n", err);

	mtk_mfg_debug("mmpll get freq = %ld\n",  clk_get_rate(mmpll));
}

static void mtk_gpu_clock_switch(struct clk *mmpll,
				 unsigned int freq_old, unsigned int freq_new)
{
	if (freq_new == freq_old)
		return;

	mtk_gpu_dfs_mmpll(mmpll, freq_new);

	mtk_mfg_debug("mtk_gpu_clock_switch, freq_new = %d\n", freq_new);
}

static void mtk_gpu_volt_switch(struct regulator *vgpu,
				unsigned int volt_old, unsigned int volt_new)
{
	int ret;

	if (volt_new == volt_old)
		return;
	ret = regulator_set_voltage(vgpu, volt_new,
				    GPU_VOLT_TO_EXTBUCK_MAXVAL(volt_new));
	if (ret != 0) {
		pr_err("mtk_gpu_volt_switch: set volt %d %d, errno %d\n",
		       volt_old, volt_new, ret);
		return;
	}

	udelay(GPU_DVFS_VOLT_SETTLE_TIME(volt_old, volt_new));

	mtk_mfg_debug("mtk_gpu_volt_switch: volt_new = %d\n", volt_new);
}

#if !defined(PVR_DVFS)
/***********************************************************
* [note]
* 1. frequency ramp up need to wait voltage settle
* 2. frequency ramp down do not need to wait voltage settle
************************************************************/
static void mtk_mfg_fv_set_by_level(struct mtk_mfg_base *mfg_base, int level)
{
	unsigned int curr_freq, curr_volt;
	unsigned int target_freq, target_volt;

	curr_freq = mfg_base->curr_freq;
	curr_volt = mfg_base->curr_volt;

	target_freq = mfg_base->fv_table[level].freq;
	target_volt = mfg_base->fv_table[level].volt;

	pr_info("Setting dvfs level %d, freq %d kHz, volt %d uV\n",
		level, target_freq, target_volt);

	mutex_lock(&mfg_base->set_freq_lock);

	if (target_freq > curr_freq) {
		mtk_gpu_volt_switch(mfg_base->vgpu, curr_volt, target_volt);
		mtk_gpu_clock_switch(mfg_base->mmpll, curr_freq, target_freq);
	} else {
		mtk_gpu_clock_switch(mfg_base->mmpll, curr_freq, target_freq);
		mtk_gpu_volt_switch(mfg_base->vgpu, curr_volt, target_volt);
	}

	mfg_base->current_level = level;
	mfg_base->curr_freq = target_freq;
	mfg_base->curr_volt = target_volt;

	mutex_unlock(&mfg_base->set_freq_lock);

	if (1) {
		mfg_base->curr_freq = HZ_TO_KHZ(clk_get_rate(mfg_base->mmpll));
		mfg_base->curr_volt = regulator_get_voltage(mfg_base->vgpu);
	}

	pr_info("[MFG] mtk_gpufreq_set : freq = %d, volt = %d\n",
		mfg_base->curr_freq, mfg_base->curr_volt);
}

static void mtk_mfg_fv_set_initial(struct mtk_mfg_base *mfg_base,
				   unsigned int freq, unsigned int volt)
{
	unsigned int curr_freq, curr_volt;

	curr_freq = HZ_TO_KHZ(clk_get_rate(mfg_base->mmpll));
	curr_volt = regulator_get_voltage(mfg_base->vgpu);

	mutex_lock(&mfg_base->set_freq_lock);

	mtk_gpu_volt_switch(mfg_base->vgpu, curr_volt, volt);
	mtk_gpu_clock_switch(mfg_base->mmpll, curr_freq, freq);
	mfg_base->curr_freq = curr_freq;
	mfg_base->curr_volt = curr_volt;

	mutex_unlock(&mfg_base->set_freq_lock);

	if (1) {
		mfg_base->curr_freq = HZ_TO_KHZ(clk_get_rate(mfg_base->mmpll));
		mfg_base->curr_volt = regulator_get_voltage(mfg_base->vgpu);
	}

	pr_info("[MFG] Set GPU Initial Freq. : freq = %d, volt = %d\n",
		mfg_base->curr_freq, mfg_base->curr_volt);
}

static PVRSRV_DEVICE_NODE *mtk_mfg_get_rgx_device_node(void)
{
	int i;
	PVRSRV_DATA *psPVRSRVData = PVRSRVGetPVRSRVData();
	PVRSRV_DEVICE_NODE *node;

	for (i = 0; i < psPVRSRVData->ui32RegisteredDevices; i++) {
		node = psPVRSRVData->apsRegisteredDevNodes[i];
		if (node && node->psDevConfig &&
		    node->psDevConfig->eDeviceType == PVRSRV_DEVICE_TYPE_RGX) {
			return node;
		}
	}
	return NULL;
}

static void mtk_gpu_writeback_freq_to_rgx(u32 freq)
{
	PVRSRV_DEVICE_NODE *psDeviceNode = mtk_mfg_get_rgx_device_node();
	RGX_DATA *psRGXData = (RGX_DATA *)psDeviceNode->psDevConfig->hDevData;

	psRGXData->psRGXTimingInfo->ui32CoreClockSpeed = freq * 1000;
}


#if MTK_MFG_DEBUG_SYS

static struct mtk_mfg_base *debug_sys_mfg_base;

static ssize_t show_available_frequencies(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	ssize_t ret = 0;
	u32 i;

	for (i = 0; i < debug_sys_mfg_base->fv_table_length; i++)
		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%d\n",
				 debug_sys_mfg_base->fv_table[i].freq);

	return ret;
}

static ssize_t show_max_level(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 debug_sys_mfg_base->max_level);
}

static ssize_t set_max_level(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	unsigned long level;
	ssize_t ret;

	ret = kstrtoul(buf, 10, &level);
	if (ret)
		return ret;

	if (level >= debug_sys_mfg_base->fv_table_length)
		debug_sys_mfg_base->max_level = -1;
	else
		debug_sys_mfg_base->max_level = level;

	return count;
}

static ssize_t show_clock(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%lu\n",
			 clk_get_rate(debug_sys_mfg_base->mmpll) / 1000);
}

static ssize_t set_clock(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	unsigned long freq;
	int level;
	ssize_t ret;

	ret = kstrtoul(buf, 10, &freq);
	if (ret)
		return ret;

	if (debug_sys_mfg_base->max_level == -1) {
		mtk_gpu_clock_switch(debug_sys_mfg_base->mmpll,
				     debug_sys_mfg_base->curr_freq, freq);
		debug_sys_mfg_base->curr_freq =
			HZ_TO_KHZ(clk_get_rate(debug_sys_mfg_base->mmpll));
		return count;
	}

	if (!debug_sys_mfg_base->fv_table_length)
		return count;

	for (level = debug_sys_mfg_base->max_level; level > 0; level--) {
		if (debug_sys_mfg_base->fv_table[level].freq <= freq)
			break;
	}

	mtk_mfg_fv_set_by_level(debug_sys_mfg_base, level);
	freq = debug_sys_mfg_base->fv_table[level].freq;
	mtk_gpu_writeback_freq_to_rgx(freq);
	return count;
}

static ssize_t show_volt(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%u\n",
			 regulator_get_voltage(debug_sys_mfg_base->vgpu));
}

static ssize_t set_volt(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	unsigned long volt;
	ssize_t ret;

	ret = kstrtoul(buf, 10, &volt);
	if (ret)
		return ret;

	if (debug_sys_mfg_base->max_level != -1)
		return count;

	mtk_gpu_volt_switch(debug_sys_mfg_base->vgpu,
			    debug_sys_mfg_base->curr_volt, volt);

	debug_sys_mfg_base->curr_volt =
		regulator_get_voltage(debug_sys_mfg_base->vgpu);

	return count;
}

DEVICE_ATTR(available_frequencies, S_IRUGO, show_available_frequencies, NULL);
DEVICE_ATTR(max_level, S_IRUGO | S_IWUSR, show_max_level, set_max_level);
DEVICE_ATTR(clock, S_IRUGO | S_IWUSR, show_clock, set_clock);
DEVICE_ATTR(volt, S_IRUGO | S_IWUSR, show_volt, set_volt);

static struct attribute *mfgsys_sysfs_entries[] = {
	&dev_attr_available_frequencies.attr,
	&dev_attr_max_level.attr,
	&dev_attr_clock.attr,
	&dev_attr_volt.attr,
	NULL,
};

static const struct attribute_group mgfsys_attr_group = {
	.attrs	= mfgsys_sysfs_entries,
};

static int mtk_mfg_create_sysfs(struct mtk_mfg_base *mfg_base)
{
	int ret;
	struct device *dev = &mfg_base->pdev->dev;

	ret = sysfs_create_group(&dev->kobj, &mgfsys_attr_group);
	if (ret)
		dev_err(dev, "create sysfs group error, %d\n", ret);

	/* backup mfg_base */
	debug_sys_mfg_base = mfg_base;
	return ret;
}

void mtk_mfg_remove_sysfs(struct mtk_mfg_base *mfg_base)
{
	struct device *dev = &mfg_base->pdev->dev;

	sysfs_remove_group(&dev->kobj, &mgfsys_attr_group);
}
#else
static int mtk_mfg_create_sysfs(struct mtk_mfg_base *mfg_base)
{
	return 0;
}

static void mtk_mfg_remove_sysfs(struct mtk_mfg_base *mfg_base)
{
}
#endif

void mtk_mfg_gpu_dvfs_init(struct mtk_mfg_base *mfg_base)
{
	int index = 0;
	unsigned int curr_freq, curr_volt;

	mutex_init(&mfg_base->set_freq_lock);

	if (mfg_base->fv_table)	{
		curr_freq = mfg_base->fv_table[index].freq;
		curr_volt = mfg_base->fv_table[index].volt;
		mfg_base->current_level = index;
		mfg_base->max_level =  mfg_base->fv_table_length - 1;
	} else {
		curr_freq = GPU_DVFS_FREQ6;
		curr_volt = GPU_DVFS_VOLT1 * 1000;
		mfg_base->current_level = -1;
		mfg_base->max_level = -1;
	}

	mtk_mfg_fv_set_initial(mfg_base, curr_freq, curr_volt);

	if (mtk_mfg_create_sysfs(mfg_base))
		return;
}

void mtk_mfg_gpu_dvfs_deinit(struct mtk_mfg_base *mfg_base)
{
	mtk_mfg_remove_sysfs(mfg_base);
}
#endif
void MTKSysSetFreq(struct mtk_mfg_base *base, u32 freq)
{
	/* freq : khz */
	mtk_gpu_clock_switch(base->mmpll, base->curr_freq, freq);
	base->curr_freq = freq;

	if (1) {
		base->curr_freq = HZ_TO_KHZ(clk_get_rate(base->mmpll));
		pr_info("MTKSysSetFreq: freq = %d  %d\n",
			freq, base->curr_freq);
	}
}

void MTKSysSetVolt(struct mtk_mfg_base *base, u32 volt)
{
	/* volt : uV */
	mtk_gpu_volt_switch(base->vgpu, base->curr_volt, volt);
	base->curr_volt = volt;
	if (1) {
		base->curr_volt = regulator_get_voltage(base->vgpu);
		pr_info("MTKSysSetVolt: volt = %d  %d\n",
			volt, base->curr_volt);
	}
}
