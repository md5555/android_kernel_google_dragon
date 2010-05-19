/* linux/arch/arm/plat-s5p/devs.c
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *              http://www.samsung.com/
 *
 * Base S5P platform device definitions
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/io.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>
#include <mach/hardware.h>
#include <mach/map.h>

#include <plat/devs.h>
#include <plat/gpio-cfg.h>
#include <plat/irqs.h>
#include <plat/fb.h>
#include <plat/fimc.h>
#include <plat/csis.h>

static struct resource s3c_wdt_resource[] = {
	[0] = {
		.start	= S5P_PA_WDT,
		.end	= S5P_PA_WDT + S5P_SZ_WDT - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_WDT,
		.end	= IRQ_WDT,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device s3c_device_wdt = {
	.name		= "s3c2410-wdt",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(s3c_wdt_resource),
	.resource	= s3c_wdt_resource,
};

static struct resource s3cfb_resource[] = {
	[0] = {
		.start	= S5P_PA_LCD,
		.end	= S5P_PA_LCD + S5P_SZ_LCD - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_LCD1,
		.end	= IRQ_LCD1,
		.flags	= IORESOURCE_IRQ,
	},
	[2] = {
		.start	= IRQ_LCD0,
		.end	= IRQ_LCD0,
		.flags	= IORESOURCE_IRQ,
	},
};

static u64 fb_dma_mask = 0xffffffffUL;

struct platform_device s3c_device_fb = {
	.name		= "s3cfb",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(s3cfb_resource),
	.resource	= s3cfb_resource,
	.dev		= {
		.dma_mask		= &fb_dma_mask,
		.coherent_dma_mask	= 0xffffffffUL
	}
};

static struct s3c_platform_fb default_fb_data __initdata = {
	.hw_ver		= 0x62,
	.nr_wins	= 5,
#if defined(CONFIG_FB_S3C_DEFAULT_WINDOW)
	.default_win	= CONFIG_FB_S3C_DEFAULT_WINDOW,
#else
	.default_win	= 0,
#endif
	.swap		= FB_SWAP_WORD | FB_SWAP_HWORD,
};

void __init s3cfb_set_platdata(struct s3c_platform_fb *pd)
{
	struct s3c_platform_fb *npd;
	int i;

	if (!pd)
		pd = &default_fb_data;

	npd = kmemdup(pd, sizeof(struct s3c_platform_fb), GFP_KERNEL);
	if (!npd)
		printk(KERN_ERR "%s: no memory for platform data\n", __func__);
	else {
		for (i = 0; i < npd->nr_wins; i++)
			npd->nr_buffers[i] = 1;

#if defined(CONFIG_FB_S3C_NR_BUFFERS)
		npd->nr_buffers[npd->default_win] = CONFIG_FB_S3C_NR_BUFFERS;
#else
		npd->nr_buffers[npd->default_win] = 1;
#endif

		s3cfb_get_clk_name(npd->clk_name);
		npd->cfg_gpio = s3cfb_cfg_gpio;
		npd->backlight_on = s3cfb_backlight_on;
		npd->reset_lcd = s3cfb_reset_lcd;
		npd->clk_on = s3cfb_clk_on;
		npd->clk_off = s3cfb_clk_off;

		s3c_device_fb.dev.platform_data = npd;
	}
}

static struct resource s3c_fimc0_resource[] = {
	[0] = {
		.start	= S5P_PA_FIMC0,
		.end	= S5P_PA_FIMC0 + S5P_SZ_FIMC0 - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_FIMC0,
		.end	= IRQ_FIMC0,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device s3c_device_fimc0 = {
	.name		= "s3c-fimc",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(s3c_fimc0_resource),
	.resource	= s3c_fimc0_resource,
};

static struct s3c_platform_fimc default_fimc0_data __initdata = {
	.default_cam	= CAMERA_PAR_A,
	.hw_ver		= 0x43,
};

void __init s3c_fimc0_set_platdata(struct s3c_platform_fimc *pd)
{
	struct s3c_platform_fimc *npd;

	if (!pd)
		pd = &default_fimc0_data;

	npd = kmemdup(pd, sizeof(struct s3c_platform_fimc), GFP_KERNEL);
	if (!npd)
		printk(KERN_ERR "%s: no memory for platform data\n", __func__);
	else {
		if (!npd->cfg_gpio)
			npd->cfg_gpio = s3c_fimc0_cfg_gpio;

		if (!npd->clk_on)
			npd->clk_on = s3c_fimc_clk_on;

		s3c_device_fimc0.dev.platform_data = npd;
	}
}

static struct resource s3c_fimc1_resource[] = {
	[0] = {
		.start	= S5P_PA_FIMC1,
		.end	= S5P_PA_FIMC1 + S5P_SZ_FIMC1 - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_FIMC1,
		.end	= IRQ_FIMC1,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device s3c_device_fimc1 = {
	.name		= "s3c-fimc",
	.id		= 1,
	.num_resources	= ARRAY_SIZE(s3c_fimc1_resource),
	.resource	= s3c_fimc1_resource,
};

static struct s3c_platform_fimc default_fimc1_data __initdata = {
	.default_cam	= CAMERA_PAR_A,
	.hw_ver		= 0x50,
};

void __init s3c_fimc1_set_platdata(struct s3c_platform_fimc *pd)
{
	struct s3c_platform_fimc *npd;

	if (!pd)
		pd = &default_fimc1_data;

	npd = kmemdup(pd, sizeof(struct s3c_platform_fimc), GFP_KERNEL);
	if (!npd)
		printk(KERN_ERR "%s: no memory for platform data\n", __func__);
	else {
		if (!npd->cfg_gpio)
			npd->cfg_gpio = s3c_fimc1_cfg_gpio;

		if (!npd->clk_on)
			npd->clk_on = s3c_fimc_clk_on;

		npd->hw_ver = 0x50;

		s3c_device_fimc1.dev.platform_data = npd;
	}
}

static struct resource s3c_fimc2_resource[] = {
	[0] = {
		.start	= S5P_PA_FIMC2,
		.end	= S5P_PA_FIMC2 + S5P_SZ_FIMC2 - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_FIMC2,
		.end	= IRQ_FIMC2,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device s3c_device_fimc2 = {
	.name		= "s3c-fimc",
	.id		= 2,
	.num_resources	= ARRAY_SIZE(s3c_fimc2_resource),
	.resource	= s3c_fimc2_resource,
};

static struct s3c_platform_fimc default_fimc2_data __initdata = {
	.default_cam	= CAMERA_PAR_A,
	.hw_ver		= 0x43,
};

void __init s3c_fimc2_set_platdata(struct s3c_platform_fimc *pd)
{
	struct s3c_platform_fimc *npd;

	if (!pd)
		pd = &default_fimc2_data;

	npd = kmemdup(pd, sizeof(struct s3c_platform_fimc), GFP_KERNEL);
	if (!npd)
		printk(KERN_ERR "%s: no memory for platform data\n", __func__);
	else {
		if (!npd->cfg_gpio)
			npd->cfg_gpio = s3c_fimc2_cfg_gpio;

		if (!npd->clk_on)
			npd->clk_on = s3c_fimc_clk_on;

		s3c_device_fimc2.dev.platform_data = npd;
	}
}

static struct resource s3c_ipc_resource[] = {
	[0] = {
		.start	= S5P_PA_IPC,
		.end	= S5P_PA_IPC + S5P_SZ_IPC - 1,
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device s3c_device_ipc = {
	.name		= "s3c-ipc",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(s3c_ipc_resource),
	.resource	= s3c_ipc_resource,
};

static struct resource s3c_csis_resource[] = {
	[0] = {
		.start	= S5P_PA_CSIS,
		.end	= S5P_PA_CSIS + S5P_SZ_CSIS - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_MIPICSI,
		.end	= IRQ_MIPICSI,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device s3c_device_csis = {
	.name		= "s3c-csis",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(s3c_csis_resource),
	.resource	= s3c_csis_resource,
};

static struct s3c_platform_csis default_csis_data __initdata = {
	.srclk_name	= "mout_mpll",
	.clk_name	= "sclk_csis",
	.clk_rate	= 166000000,
};

void __init s3c_csis_set_platdata(struct s3c_platform_csis *pd)
{
	struct s3c_platform_csis *npd;

	if (!pd)
		pd = &default_csis_data;

	npd = kmemdup(pd, sizeof(struct s3c_platform_csis), GFP_KERNEL);
	if (!npd)
		printk(KERN_ERR "%s: no memory for platform data\n", __func__);

	npd->cfg_gpio = s3c_csis_cfg_gpio;
	npd->cfg_phy_global = s3c_csis_cfg_phy_global;

	s3c_device_csis.dev.platform_data = npd;
}

static struct resource s5p_tvout_resources[] = {
	[0] = {
		.start	= S5P_PA_TVENC,
		.end	= S5P_PA_TVENC + S5P_SZ_TVENC - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= S5P_PA_VP,
		.end	= S5P_PA_VP + S5P_SZ_VP - 1,
		.flags	= IORESOURCE_MEM,
	},
	[2] = {
		.start	= S5P_PA_MIXER,
		.end	= S5P_PA_MIXER + S5P_SZ_MIXER - 1,
		.flags	= IORESOURCE_MEM,
	},
	[3] = {
		.start	= S5P_PA_HDMI,
		.end	= S5P_PA_HDMI + S5P_SZ_HDMI - 1,
		.flags	= IORESOURCE_MEM,
	},
	[4] = {
		.start	= S5P_I2C_HDMI_PHY,
		.end	= S5P_I2C_HDMI_PHY + S5P_I2C_HDMI_SZ_PHY - 1,
		.flags	= IORESOURCE_MEM,
	},
	[5] = {
		.start	= IRQ_MIXER,
		.end	= IRQ_MIXER,
		.flags	= IORESOURCE_IRQ,
	},
	[6] = {
		.start	= IRQ_HDMI,
		.end	= IRQ_HDMI,
		.flags	= IORESOURCE_IRQ,
	},
	[7] = {
		.start	= IRQ_TVENC,
		.end	= IRQ_TVENC,
		.flags	= IORESOURCE_IRQ,
	},
	[8] = {
		.start	= IRQ_EINT5,
		.end	= IRQ_EINT5,
		.flags	= IORESOURCE_IRQ,
	}
};

static u64 tvfb_dma_mask = 0xffffffffUL;

struct platform_device s5p_device_tvout = {
	.name		= "s5p-tvout",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(s5p_tvout_resources),
	.resource	= s5p_tvout_resources,
	.dev		= {
		.dma_mask		= &tvfb_dma_mask,
		.coherent_dma_mask	= 0xffffffffUL
	}
};

static struct resource s5p_cec_resources[] = {
	[0] = {
		.start	= S5P_PA_CEC,
		.end	= S5P_PA_CEC + S5P_SZ_CEC - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_CEC,
		.end	= IRQ_CEC,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device s5p_device_cec = {
	.name		= "s5p-cec",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(s5p_cec_resources),
	.resource	= s5p_cec_resources,
};

struct platform_device s5p_device_hpd = {
	.name		= "s5p-hpd",
	.id		= -1,
};
