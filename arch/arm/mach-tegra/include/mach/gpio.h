/*
 * arch/arm/mach-tegra/include/mach/gpio.h
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

#ifndef __MACH_TEGRA_GPIO_H
#define __MACH_TEGRA_GPIO_H

#include <mach/irqs.h>

#define ARCH_NR_GPIOS		INT_GPIO_NR

#include <asm-generic/gpio.h>
#include <mach/gpio-names.h>

#define gpio_get_value		__gpio_get_value
#define gpio_set_value		__gpio_set_value
#define gpio_cansleep		__gpio_cansleep

static inline int gpio_to_irq(unsigned int gpio)
{
	if (gpio < ARCH_NR_GPIOS)
		return INT_GPIO_BASE + gpio;
	return -EINVAL;
}

static inline int irq_to_gpio(unsigned int irq)
{
	if ((irq >= INT_GPIO_BASE) && (irq < INT_GPIO_BASE + INT_GPIO_NR))
		return irq - INT_GPIO_BASE;
	return -EINVAL;
}

void tegra_gpio_enable(int gpio);
void tegra_gpio_disable(int gpio);
#endif
