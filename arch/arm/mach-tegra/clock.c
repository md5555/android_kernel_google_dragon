/*
 * arch/arm/mach-tegra/clock.c
 *
 * Clock control code for NVIDIA Tegra SoCs
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
#include <linux/module.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/clk.h>

#include "nvrm_module.h"
#include "nvos.h"
#include "nvrm_power.h"
#include "mach/nvrm_linux.h"
#include "linux/parser.h"

struct clk {
	char *name;
	NvRmModuleID module;
	int instance;
};

static NvU32 s_PowerClient;

struct clk s_Clocks[] = {
	{ "cpu", NvRmModuleID_Cpu },
	{ "avp", NvRmModuleID_Avp },
	{ "vcp", NvRmModuleID_Vcp },
	{ "display", NvRmModuleID_Display },
	{ "ide", NvRmModuleID_Ide },
	{ "host", NvRmModuleID_GraphicsHost },
	{ "2D", NvRmModuleID_2D },
	{ "3D", NvRmModuleID_3D },
	{ "vg", NvRmModuleID_VG },
	{ "epp", NvRmModuleID_Epp },
	{ "isp", NvRmModuleID_Isp },
	{ "vi", NvRmModuleID_Vi },
	{ "usb", NvRmModuleID_Usb2Otg },
	{ "i2s", NvRmModuleID_I2s },
	{ "pwm", NvRmModuleID_Pwm },
	{ "twc", NvRmModuleID_Twc },
	{ "hsmmc", NvRmModuleID_Hsmmc },
	{ "sdio", NvRmModuleID_Sdio },
	{ "nand", NvRmModuleID_Nand },
	{ "i2c", NvRmModuleID_I2c },
	{ "spdif", NvRmModuleID_Spdif },
	{ "uart", NvRmModuleID_Uart },
	{ "timer", NvRmModuleID_Timer },
	{ "timerus", NvRmModuleID_TimerUs },
	{ "rtc", NvRmModuleID_Rtc },
	{ "ac97", NvRmModuleID_Ac97 },
	{ "bsea", NvRmModuleID_BseA },
	{ "vde", NvRmModuleID_Vde },
	{ "mpe", NvRmModuleID_Mpe },
	{ "csi", NvRmModuleID_Csi },
	{ "hdcp", NvRmModuleID_Hdcp },
	{ "hdmi", NvRmModuleID_Hdmi },
	{ "mipi", NvRmModuleID_Mipi },
	{ "tvo", NvRmModuleID_Tvo },
	{ "dsi", NvRmModuleID_Dsi },
	{ "dvc", NvRmModuleID_Dvc },
	{ "xio", NvRmModuleID_Xio },
	{ "spi", NvRmModuleID_Spi },
	{ "slink", NvRmModuleID_Slink },
	{ "fuse", NvRmModuleID_Fuse },
	{ "mio", NvRmModuleID_Mio },
	{ "kbc", NvRmModuleID_Kbc },
	{ "pmif", NvRmModuleID_Pmif },
	{ "ucq", NvRmModuleID_Ucq },
	{ "avp", NvRmModuleID_Avp },
	{ "event", NvRmModuleID_EventCtrl },
	{ "flow", NvRmModuleID_FlowCtrl },
	{ "rsema", NvRmModuleID_ResourceSema },
	{ "asema", NvRmModuleID_ArbitrationSema },
	{ "apri", NvRmModuleID_ArbPriority },
	{ "cache", NvRmModuleID_CacheMemCtrl },
	{ "vfir", NvRmModuleID_Vfir },
	{ "except", NvRmModuleID_ExceptionVector },
	{ "boot", NvRmModuleID_BootStrap },
	{ "stat", NvRmModuleID_SysStatMonitor },
	{ "cdev", NvRmModuleID_Cdev },
	{ "misc", NvRmModuleID_Misc },
	{ "pcie", NvRmModuleID_PcieDevice },
	{ "onewire", NvRmModuleID_OneWire },
	{ "snor", NvRmModuleID_SyncNor },
	{ "nor", NvRmModuleID_Nor },
};

enum {
	module_name,
	module_instance
};

static const match_table_t tokens = {
	{ module_name, "mod=%s" },
	{ module_instance, "inst=%u" }
};

#define MODULE_NAME_MAXCHARS 16

struct clk *clk_get(struct device *dev, const char *id)
{
	int i;
	struct clk *clk;
	char *p;
	substring_t args[MAX_OPT_ARGS];
	int instance;
	char name[MODULE_NAME_MAXCHARS];

	name[0] = 0;
	instance = 0;

	while ((p = strsep((char **)&id, ","))!= NULL) {
		int token;

		if (!*p)
			continue;

		token = match_token(p, tokens, args);
		switch (token) {
			case module_name:
				match_strlcpy(name, args, MODULE_NAME_MAXCHARS);
				break;
			case module_instance:
				match_int(args, &instance);
				break;
		}
	}

	for (i = 0; i < sizeof(s_Clocks)/sizeof(struct clk); i++) {
		if (strcmp(name, s_Clocks[i].name) == 0) {
			clk = kzalloc(sizeof(struct clk), GFP_KERNEL);
			clk->name = name;
			clk->module = s_Clocks[i].module;
			clk->instance = instance;
			return clk;
		}
	}
	return 0;
}
EXPORT_SYMBOL(clk_get);

void clk_put(struct clk *clk)
{
}
EXPORT_SYMBOL(clk_put);

int clk_enable(struct clk *clk)
{
	NvError e;

	e = NvRmPowerVoltageControl(s_hRmGlobal, clk->module, s_PowerClient,
		NvRmVoltsUnspecified, NvRmVoltsUnspecified, NULL, 0, NULL);
	if (e != NvSuccess) {
		goto fail;
	}

	/* enable clock, then voltage */
	e = NvRmPowerModuleClockControl(s_hRmGlobal,
		NVRM_MODULE_ID(clk->module,clk->instance),
		s_PowerClient, NV_TRUE);
	if (e != NvSuccess) {
		goto fail;
	}

	return 0;

fail:
	return -1;
}
EXPORT_SYMBOL(clk_enable);

void clk_disable(struct clk *clk)
{
	/* disable clock, then voltage */
	// FIXME: assert success
	(void)NvRmPowerModuleClockControl(s_hRmGlobal,
		NVRM_MODULE_ID(clk->module, clk->instance),
		s_PowerClient, NV_FALSE);

	(void)NvRmPowerVoltageControl(s_hRmGlobal,
		NVRM_MODULE_ID(clk->module, clk->instance),
		s_PowerClient, NvRmVoltsOff, NvRmVoltsOff, NULL, 0, NULL);
}
EXPORT_SYMBOL(clk_disable);

unsigned long clk_get_rate(struct clk *clk)
{
	NvError e;
	NvRmDfsClockUsage usage;
	unsigned long rate = 0;

	if (clk->module == NvRmModuleID_Cpu ||
		clk->module == NvRmModuleID_Avp) {
		NvRmDfsClockId id;
		id = (clk->module == NvRmModuleID_Cpu) ? NvRmDfsClockId_Cpu :
			NvRmDfsClockId_Avp;
		e = NvRmDfsGetClockUtilization(s_hRmGlobal, id, &usage);
		if (e != NvSuccess) {
			goto clean;
		}

		rate = usage.CurrentKHz * 1000;
	} else {
		NvRmFreqKHz freq;
		e = NvRmPowerModuleClockConfig(s_hRmGlobal,
			NVRM_MODULE_ID(clk->module, clk->instance),
			s_PowerClient, 0, 0, NULL, 0, &freq, 0);
		if (e != NvSuccess ) {
			goto clean;
		}
		rate = freq * 1000;
	}

clean:
	return rate;
}
EXPORT_SYMBOL(clk_get_rate);

long clk_round_rate(struct clk *clk, unsigned long rate)
{
	return rate;
}
EXPORT_SYMBOL(clk_round_rate);

int clk_set_rate(struct clk *clk, unsigned long rate)
{
	NvError e;
	NvRmFreqKHz freq;

	freq = rate / 1000;

	e = NvRmPowerModuleClockConfig(s_hRmGlobal,
		NVRM_MODULE_ID(clk->module,clk->instance), s_PowerClient,
		NvRmFreqUnspecified, NvRmFreqUnspecified, &freq, 1, &freq, 0);
	if (e != NvSuccess) {
		goto fail;
	}

	return 0;
fail:
	return -EIO;
}
EXPORT_SYMBOL(clk_set_rate);

int clk_register(struct clk *clk)
{
	return 0;
}
EXPORT_SYMBOL(clk_register);

void clk_unregister(struct clk *clk)
{
}
EXPORT_SYMBOL(clk_unregister);

void __init tegra_clk_init(void)
{
	NvRmPowerRegister(s_hRmGlobal, 0, &s_PowerClient);
}
