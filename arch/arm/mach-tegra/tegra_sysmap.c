/*
 * arch/arm/mach-tegra/tegra_sysmap.c
 *
 * APIs and procfs attributes for reading the hardware relocation table
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

#include <linux/string.h>
#include <mach/platform.h>
#include <mach/nvrm_linux.h>
#include <asm/irq.h>
#include "nvcommon.h"
#include "nvrm_module.h"
#include "nvrm_interrupt.h"

static NvRmModuleID tegra_map_name_to_mod(const char *name, int inst)
{
	if (!strcmp(name, "gpio"))
		return NVRM_MODULE_ID(NvRmPrivModuleID_Gpio, inst);
	else if (!strcmp(name, "pcie"))
		return NVRM_MODULE_ID(NvRmPrivModuleID_Pcie, inst);
	else if (!strcmp(name, "pl310"))
		return NVRM_MODULE_ID(NvRmPrivModuleID_Pl310, inst);
	else if (!strcmp(name, "scu"))
		return NVRM_MODULE_ID(NvRmPrivModuleID_ArmPerif, inst);
	else if (!strcmp(name, "usbotg"))
		return NVRM_MODULE_ID(NvRmModuleID_Usb2Otg, inst);
	else if (!strcmp(name, "mc"))
		return NVRM_MODULE_ID(NvRmPrivModuleID_MemoryController, inst);
	else if (!strcmp(name, "gart"))
		return NVRM_MODULE_ID(NvRmPrivModuleID_Gart, inst);
	else if (!strcmp(name, "iram"))
		return NVRM_MODULE_ID(NvRmPrivModuleID_Iram, inst);
	else if (!strcmp(name, "kbc"))
		return NVRM_MODULE_ID(NvRmModuleID_Kbc, 0);

	return (NvRmModuleID) 0;
}

unsigned long tegra_get_module_inst_base(const char *name, int inst)
{
	NvRmModuleID mod_id = tegra_map_name_to_mod(name, inst);
	NvRmPhysAddr phys = 0xffffffffUL;
	NvU32 len = 0;

	if (mod_id)
		NvRmModuleGetBaseAddress(s_hRmGlobal, mod_id, &phys, &len);

	if (!len)
		phys = 0xffffffffUL;

	return (unsigned long)phys;
}

unsigned long tegra_get_module_inst_size(const char *name, int inst)
{
	NvRmModuleID mod_id = tegra_map_name_to_mod(name, inst);
	NvRmPhysAddr phys = 0xffffffffUL;
	NvU32 len = 0;

	if (mod_id)
		NvRmModuleGetBaseAddress(s_hRmGlobal, mod_id, &phys, &len);

	return (unsigned long)len;
}

unsigned int tegra_get_module_inst_irq(const char *name, int inst,
	int mod_int_id)
{
	NvRmModuleID mod_id = tegra_map_name_to_mod(name, inst);
	NvU16 irq = 0xffff;

	if (mod_id)
		irq = NvRmGetIrqForLogicalInterrupt(s_hRmGlobal,
			mod_id, mod_int_id);

	if (irq==0xffff)
		return NO_IRQ;
	return irq;
}
