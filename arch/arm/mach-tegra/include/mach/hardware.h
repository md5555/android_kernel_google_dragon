/*
 * arch/arm/mach-tegra/include/mach/hardware.h
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

#ifndef __MACH_TEGRA_HARDWARE_H

//do not let linux override bios's bus assignments
#define pcibios_assign_all_busses()         0

extern unsigned long pci_tegra_get_base(char *aperture);

#define PCIBIOS_MIN_IO          pci_tegra_get_base("io")
#define PCIBIOS_MIN_MEM         pci_tegra_get_base("mem")

#endif
