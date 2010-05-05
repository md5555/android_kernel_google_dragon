/* linux/arch/arm/mach-s5pv210/pd.c
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * based on linux/arch/arm/plat-samsung/clock.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/module.h>

#include <mach/pd.h>
#include <mach/regs-clock.h>

static LIST_HEAD(pd_domains);

static DEFINE_SPINLOCK(pd_lock);

static int s5pv210_pd_clk_on(struct pd_domain *pd)
{
	struct pd_domain *p;
	struct power_domain *parent_p;

	u32	tmp;

	parent_p = pd->parent_pd;

	list_for_each_entry(p, &pd_domains, list) {
		if ((p->parent_pd) == parent_p) {
			tmp = __raw_readl(p->clk_reg);
			tmp |= (p->clk_bit);
			__raw_writel(tmp, p->clk_reg);
		}
	}

	return 0;
}

static int s5pv210_pd_clk_off(struct pd_domain *pd)
{
	struct pd_domain *p;
	struct power_domain *parent_p;

	u32	tmp;

	parent_p = pd->parent_pd;

	list_for_each_entry(p, &pd_domains, list) {
		if ((p->parent_pd) == parent_p) {
			tmp = __raw_readl(p->clk_reg);
			tmp &= ~(p->clk_bit);
			__raw_writel(tmp, p->clk_reg);
		}
	}

	return 0;
}

static int s5pv210_pd_ctrl(struct pd_domain *pd, int enable)
{
	struct power_domain *parent_p;
	u32 pd_status;

	parent_p = pd->parent_pd;

	pd_status = __raw_readl(S5P_NORMAL_CFG);

	/*
	 * Before turn on some power domain, every clocks which were
	 * depended on power domain must turned on.
	*/
	if (enable == PD_ACTIVE) {
		parent_p->usage++;
		if (!(pd_status&parent_p->ctrlbit)) {
			s5pv210_pd_clk_on(pd);
			pd_status |= (parent_p->ctrlbit);
			__raw_writel(pd_status, S5P_NORMAL_CFG);
			s5pv210_pd_clk_off(pd);
			udelay(PD_CLOCK_TIME);
		}
	} else if (enable == PD_INACTIVE) {
		parent_p->usage--;
		if (parent_p->usage == 0) {
			pd_status &= ~(parent_p->ctrlbit);
			__raw_writel(pd_status, S5P_NORMAL_CFG);
		}
	}

	return 0;
}

struct pd_domain *s5pv210_pd_find(const char *id)
{
	struct pd_domain *p;
	struct pd_domain *check_pd = 0;

	list_for_each_entry(p, &pd_domains, list) {
		if (strcmp(id, p->name) == 0) {
			check_pd = p;
			break;
		}
	}

	return check_pd;
}

int s5pv210_pd_enable(const char *id)
{
	struct pd_domain *pd_p;

	pd_p = s5pv210_pd_find(id);

	if (!(pd_p)) {
		return -EINVAL;
	}
	spin_lock(&pd_lock);

	(pd_p->ctrl)(pd_p, 1);

	spin_unlock(&pd_lock);

	return 0;
}
EXPORT_SYMBOL(s5pv210_pd_enable);

int s5pv210_pd_disable(const char *id)
{
	struct pd_domain *pd_p;

	pd_p = s5pv210_pd_find(id);

	if (!(pd_p)) {
		return -EINVAL;
	}
	spin_lock(&pd_lock);

	(pd_p->ctrl)(pd_p, 0);

	spin_unlock(&pd_lock);

	return 0;
}
EXPORT_SYMBOL(s5pv210_pd_disable);

static struct power_domain pd_cam = {
	.nr_clks = 0,
	.usage = 0,
	.ctrlbit = S5PV210_PD_CAM,
};

static struct power_domain pd_tv = {
	.nr_clks = 0,
	.usage = 0,
	.ctrlbit = S5PV210_PD_TV,
};

static struct power_domain pd_lcd = {
	.nr_clks = 0,
	.usage = 0,
	.ctrlbit = S5PV210_PD_LCD,
};

static struct power_domain pd_audio = {
	.nr_clks = 0,
	.usage = 0,
	.ctrlbit = S5PV210_PD_AUDIO,
};

static struct power_domain pd_g3d = {
	.nr_clks = 0,
	.usage = 0,
	.ctrlbit = S5PV210_PD_G3D,
};

static struct power_domain pd_mfc = {
	.nr_clks = 0,
	.usage = 0,
	.ctrlbit = S5PV210_PD_MFC,
};

static struct pd_domain init_pd[] = {
	{
		.name		= "fimc_pd",
		.clk_reg 	= S5P_CLKGATE_IP0,
		.clk_bit 	= ((1<<24)|(1<<25)|(1<<26)),
		.ctrl		= s5pv210_pd_ctrl,
		.parent_pd	= &pd_cam,
	}, {
		.name		= "csis_pd",
		.clk_reg 	= S5P_CLKGATE_IP0,
		.clk_bit 	= (1<<31),
		.ctrl		= s5pv210_pd_ctrl,
		.parent_pd	= &pd_cam,
	}, {
		.name		= "jpeg_pd",
		.clk_reg 	= S5P_CLKGATE_IP0,
		.clk_bit 	= (1<<28),
		.ctrl		= s5pv210_pd_ctrl,
		.parent_pd	= &pd_cam,
	}, {
		.name		= "rotator_pd",
		.clk_reg 	= S5P_CLKGATE_IP0,
		.clk_bit 	= (1<<29),
		.ctrl		= s5pv210_pd_ctrl,
		.parent_pd	= &pd_cam,
	}, {
		.name		= "vp_pd",
		.clk_reg 	= S5P_CLKGATE_IP1,
		.clk_bit 	= (1<<8),
		.ctrl		= s5pv210_pd_ctrl,
		.parent_pd	= &pd_tv,
	}, {
		.name		= "mixer_pd",
		.clk_reg 	= S5P_CLKGATE_IP1,
		.clk_bit 	= (1<<9),
		.ctrl		= s5pv210_pd_ctrl,
		.parent_pd	= &pd_tv,
	}, {
		.name		= "tv_enc_pd",
		.clk_reg 	= S5P_CLKGATE_IP1,
		.clk_bit 	= (1<<10),
		.ctrl		= s5pv210_pd_ctrl,
		.parent_pd	= &pd_tv,
	}, {
		.name		= "hdmi_pd",
		.clk_reg 	= S5P_CLKGATE_IP1,
		.clk_bit 	= (1<<11),
		.ctrl		= s5pv210_pd_ctrl,
		.parent_pd	= &pd_tv,
	}, {
		.name		= "fimd_pd",
		.clk_reg 	= S5P_CLKGATE_IP1,
		.clk_bit 	= (1<<0),
		.ctrl		= s5pv210_pd_ctrl,
		.parent_pd	= &pd_lcd,
	}, {
		.name		= "dsim_pd",
		.clk_reg 	= S5P_CLKGATE_IP1,
		.clk_bit 	= (1<<2),
		.ctrl		= s5pv210_pd_ctrl,
		.parent_pd	= &pd_lcd,
	}, {
		.name		= "g2d_pd",
		.clk_reg 	= S5P_CLKGATE_IP0,
		.clk_bit 	= (1<<12),
		.ctrl		= s5pv210_pd_ctrl,
		.parent_pd	= &pd_lcd,
	}, {
		.name		= "g3d_pd",
		.clk_reg 	= S5P_CLKGATE_IP0,
		.clk_bit 	= (1<<8),
		.ctrl		= s5pv210_pd_ctrl,
		.parent_pd	= &pd_g3d,
	}, {
		.name		= "mfc_pd",
		.clk_reg 	= S5P_CLKGATE_IP0,
		.clk_bit 	= (1<<16),
		.ctrl		= s5pv210_pd_ctrl,
		.parent_pd	= &pd_mfc,
	},
};

static int s5pv210_register_pd(struct pd_domain *pd)
{
	spin_lock(&pd_lock);
	list_add(&pd->list, &pd_domains);
	spin_unlock(&pd_lock);

	return 0;
}

static void s5pv210_register_pds(void)
{
	struct pd_domain *pd_p;

	int i;
	int ret;

	pd_p = init_pd;

	for (i = 0 ; i < ARRAY_SIZE(init_pd) ; i++, pd_p++) {
		ret = s5pv210_register_pd(pd_p);

		if (ret < 0)
			printk(KERN_INFO "Failed to register %s sub-domain : %d",
				pd_p->name, ret);
	}
}
static int __init s5pv210_pd_init(void)
{
	printk(KERN_DEBUG "S5PV210 Power Domain API Enable\n");

	s5pv210_register_pds();

	return 0;
}

arch_initcall(s5pv210_pd_init);
