/* linux/arch/arm/mach-s5pv210/mach-voguev210.c
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/serial_core.h>
#include <linux/gpio.h>
#include <linux/dm9000.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/setup.h>
#include <asm/mach-types.h>

#include <mach/map.h>
#include <mach/regs-clock.h>
#include <mach/regs-mem.h>
#include <mach/regs-gpio.h>

#include <plat/regs-serial.h>
#include <plat/gpio-cfg.h>
#include <plat/s5pv210.h>
#include <plat/devs.h>
#include <plat/cpu.h>
#include <plat/iic.h>
#include <plat/fb.h>
#include <plat/fimc.h>
#include <plat/csis.h>

#if defined(CONFIG_PM)
#include <mach/pm.h>
#endif

/* Following are default values for UCON, ULCON and UFCON UART registers */
#define S5PV210_UCON_DEFAULT	(S3C2410_UCON_TXILEVEL |	\
				 S3C2410_UCON_RXILEVEL |	\
				 S3C2410_UCON_TXIRQMODE |	\
				 S3C2410_UCON_RXIRQMODE |	\
				 S3C2410_UCON_RXFIFO_TOI |	\
				 S3C2443_UCON_RXERR_IRQEN)

#define S5PV210_ULCON_DEFAULT	S3C2410_LCON_CS8

#define S5PV210_UFCON_DEFAULT	(S3C2410_UFCON_FIFOMODE |	\
				 S5PV210_UFCON_TXTRIG4 |	\
				 S5PV210_UFCON_RXTRIG4)

extern void s5pv210_reserve_bootmem(void);

struct membank s3c_meminfo[] __initdata = {
	[0] = {
		.start = 0x20000000,
		.size = 512 * SZ_1M,
		.node = 0,
	},
	[1] = {
		.start = 0x40000000,
		.size = 512 * SZ_1M,
		.node = 0,
	},
};

static struct s3c2410_uartcfg voguev210_uartcfgs[] __initdata = {
	[0] = {
		.hwport		= 0,
		.flags		= 0,
		.ucon		= S5PV210_UCON_DEFAULT,
		.ulcon		= S5PV210_ULCON_DEFAULT,
		.ufcon		= S5PV210_UFCON_DEFAULT,
	},
	[1] = {
		.hwport		= 1,
		.flags		= 0,
		.ucon		= S5PV210_UCON_DEFAULT,
		.ulcon		= S5PV210_ULCON_DEFAULT,
		.ufcon		= S5PV210_UFCON_DEFAULT,
	},
	[2] = {
		.hwport		= 2,
		.flags		= 0,
		.ucon		= S5PV210_UCON_DEFAULT,
		.ulcon		= S5PV210_ULCON_DEFAULT,
		.ufcon		= S5PV210_UFCON_DEFAULT,
	},
	[3] = {
		.hwport		= 3,
		.flags		= 0,
		.ucon		= S5PV210_UCON_DEFAULT,
		.ulcon		= S5PV210_ULCON_DEFAULT,
		.ufcon		= S5PV210_UFCON_DEFAULT,
	},
};

#ifdef CONFIG_I2C_S3C2410
/* I2C0 */
static struct i2c_board_info i2c_devs0[] __initdata = {
};

#ifdef CONFIG_S3C_DEV_I2C1
/* I2C1 */
static struct i2c_board_info i2c_devs1[] __initdata = {
};
#endif

#ifdef CONFIG_S3C_DEV_I2C2
/* I2C2 */
static struct i2c_board_info i2c_devs2[] __initdata = {
};
#endif
#endif

#ifdef CONFIG_VIDEO_FIMC
static struct s3c_platform_fimc fimc_plat = {
#ifdef S5K6AA_ENABLED
	.default_cam	= CAMERA_CSI_C,
	.camera		= &s5k6aa,
#endif
};
#endif

#ifdef CONFIG_DM9000
static struct resource dm9000_resources[] = {
	[0] = {
		.start = S5PV210_PA_DM9000,
		.end   = S5PV210_PA_DM9000,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = S5PV210_PA_DM9000 + 2,
		.end   = S5PV210_PA_DM9000 + 2,
		.flags = IORESOURCE_MEM,
	},
	[2] = {
		.start = IRQ_EINT_GROUP(18, 0),
		.end   = IRQ_EINT_GROUP(18, 0),
		.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL,
	}
};

static struct dm9000_plat_data dm9000_platdata = {
	.flags = DM9000_PLATF_16BITONLY | DM9000_PLATF_NO_EEPROM,
};

struct platform_device device_dm9000 = {
	.name		= "dm9000",
	.id		=  0,
	.num_resources	= ARRAY_SIZE(dm9000_resources),
	.resource	= dm9000_resources,
	.dev		= {
		.platform_data = &dm9000_platdata,
	}
};

static void __init voguev210_dm9000_set(void)
{
	unsigned int tmp;

	tmp = ((0<<28)|(0<<24)|(5<<16)|(0<<12)|(0<<8)|(0<<4)|(0<<0));
	__raw_writel(tmp, (S5P_SROM_BW + 0x18));

	tmp = __raw_readl(S5P_SROM_BW);
	tmp &= ~(0xf << 20);

	tmp |= (0x1 << 20);		/* 16bit */
	__raw_writel(tmp, S5P_SROM_BW);

	tmp = __raw_readl(S5PV210_MP01_BASE);
	tmp &= ~(0xf << 20);
	tmp |= (2 << 20);

	__raw_writel(tmp, S5PV210_MP01_BASE);

	/* initialize gpio */
	s3c_gpio_cfgpin(S5PV210_GPJ0(0), 0xf);
	s3c_gpio_setpull(S5PV210_GPJ0(0), S3C_GPIO_PULL_UP);
}
#else
static void __init voguev210_dm9000_set(void) {}
#endif

static struct platform_device *voguev210_devices[] __initdata = {
#ifdef CONFIG_I2C_S3C2410
	&s3c_device_i2c0,
#ifdef CONFIG_S3C_DEV_I2C1
	&s3c_device_i2c1,
#endif
#ifdef CONFIG_S3C_DEV_I2C2
	&s3c_device_i2c2,
#endif
#endif

#ifdef CONFIG_FB_S3C
	&s3c_device_fb,
#endif

#ifdef CONFIG_VIDEO_FIMC
	&s3c_device_fimc0,
	&s3c_device_fimc1,
	&s3c_device_fimc2,
	&s3c_device_csis,
	&s3c_device_ipc,
#endif

#ifdef CONFIG_DM9000
	&device_dm9000,
#endif

#ifdef CONFIG_S3C2410_WATCHDOG
	&s3c_device_wdt,
#endif

#ifdef CONFIG_VIDEO_TV20
	&s5p_device_tvout,
	&s5p_device_cec,
	&s5p_device_hpd,
#endif
};

static void __init voguev210_fixup(struct machine_desc *desc,
				   struct tag *tags, char **cmdline,
				   struct meminfo *mi)
{
	u32 i = 0;

	for (i = 0; i < ARRAY_SIZE(s3c_meminfo); i++)
		mi->bank[i] = s3c_meminfo[i];

	mi->nr_banks = ARRAY_SIZE(s3c_meminfo);
}

static void __init voguev210_map_io(void)
{
	s5p_init_io(NULL, 0, S5P_VA_CHIPID);
	s3c24xx_init_clocks(24000000);
	s3c24xx_init_uarts(voguev210_uartcfgs, ARRAY_SIZE(voguev210_uartcfgs));
	s5pv210_reserve_bootmem();
}

static void __init voguev210_machine_init(void)
{
	voguev210_dm9000_set();

#ifdef CONFIG_I2C_S3C2410
	s3c_i2c0_set_platdata(NULL);
	i2c_register_board_info(0, i2c_devs0, ARRAY_SIZE(i2c_devs0));
#ifdef CONFIG_S3C_DEV_I2C1
	s3c_i2c1_set_platdata(NULL);
	i2c_register_board_info(1, i2c_devs1, ARRAY_SIZE(i2c_devs1));
#endif
#ifdef CONFIG_S3C_DEV_I2C2
	s3c_i2c2_set_platdata(NULL);
	i2c_register_board_info(2, i2c_devs2, ARRAY_SIZE(i2c_devs2));
#endif
#endif

#if defined(CONFIG_PM)
	s3c_pm_init();
	s3c_irq_wake(IRQ_RTC_ALARM, 1);
#endif

#ifdef CONFIG_FB_S3C
	s3cfb_set_platdata(NULL);
#endif

#ifdef CONFIG_VIDEO_FIMC
	s3c_fimc0_set_platdata(&fimc_plat);
	s3c_fimc1_set_platdata(&fimc_plat);
	s3c_fimc2_set_platdata(&fimc_plat);
	s3c_csis_set_platdata(NULL);
#endif

	platform_add_devices(voguev210_devices, ARRAY_SIZE(voguev210_devices));
}

MACHINE_START(VOGUEV210, "VOGUEV210")
	/* Maintainer: Kukjin Kim <kgene.kim@samsung.com> */
	.phys_io	= S3C_PA_UART & 0xfff00000,
	.io_pg_offst	= (((u32)S3C_VA_UART) >> 18) & 0xfffc,
	.boot_params	= S5P_PA_SDRAM + 0x100,
	.fixup		= voguev210_fixup,
	.init_irq	= s5pv210_init_irq,
	.map_io		= voguev210_map_io,
	.init_machine	= voguev210_machine_init,
	.timer		= &s5p_systimer,
MACHINE_END
