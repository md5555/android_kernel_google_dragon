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

extern int gpio_set_value(unsigned gpio, int value);

extern int gpio_get_value(unsigned gpio);

extern int gpio_direction_input(unsigned gpio);

extern int gpio_direction_output(unsigned gpio, int value);

#include <asm-generic/gpio.h>

extern int gpio_request(unsigned gpio, const char *tag);

extern void gpio_free(unsigned gpio);

#endif
