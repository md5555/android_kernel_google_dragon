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
#include <linux/delay.h>
#include <linux/regulator/consumer.h>

#include "mt8173_mfgdvfs.h"
#include "mt8173_mfgsys.h"

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

void MTKSysSetFreq(struct mtk_mfg *mfg, u32 freq)
{
	/* freq : khz */
	mtk_gpu_clock_switch(mfg->mmpll, mfg->curr_freq, freq);
	mfg->curr_freq = HZ_TO_KHZ(clk_get_rate(mfg->mmpll));
	pr_info("MTKSysSetFreq: freq = %d  %d\n",
		freq, mfg->curr_freq);
}

void MTKSysSetVolt(struct mtk_mfg *mfg, u32 volt)
{
	/* volt : uV */
	mtk_gpu_volt_switch(mfg->vgpu, mfg->curr_volt, volt);
	mfg->curr_volt = regulator_get_voltage(mfg->vgpu);
	pr_info("MTKSysSetVolt: volt = %d  %d\n",
		volt, mfg->curr_volt);
}
