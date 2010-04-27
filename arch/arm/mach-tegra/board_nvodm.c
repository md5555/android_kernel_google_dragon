/*
 * arch/arm/mach-tegra/board-nvodm.c
 *
 * Board registration for ODM-kit generic Tegra boards
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
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/pm.h>
#include <linux/spi/spi.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <mach/platform.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>

#include <linux/io.h>
#include <linux/serial_8250.h>

#include <linux/delay.h>

extern struct sys_timer tegra_timer;

static void __init tegra_init_irq(void)
{
}

static void __init tegra_map_common_io(void)
{
}

static struct platform_device nvrm_device = {
    .name = "nvrm"
};

static void __init tegra_machine_init(void)
{
    (void) platform_device_register(&nvrm_device);
}

MACHINE_START(TEGRA_GENERIC, "Tegra generic")

    .boot_params  = 0x00000100,
    .map_io       = tegra_map_common_io,
    .init_irq     = tegra_init_irq,
    .init_machine = tegra_machine_init,
    .timer        = &tegra_timer,

MACHINE_END
