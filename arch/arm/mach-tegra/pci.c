/*
 *  arch/arm/mach-tegra/pci.c
 *
 *  PCIe host controller driver for TEGRA(2) SOCs
 *
 * Copyright (c) 2008-2009, NVIDIA Corporation.
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
#include <linux/pci.h>
#include <asm/mach/pci.h>
#include <linux/interrupt.h>
#include <linux/irq.h>

static int __init pcie_tegra_init(void);

static void __init pci_tegra_preinit(void);
static int __init pci_tegra_setup(int nr, struct pci_sys_data *data);
static struct pci_bus __init *pci_tegra_scan_bus(int nr,
	struct pci_sys_data *sys);

static int pci_tegra_read_conf(struct pci_bus *bus, u32 devfn,
	int where, int size, u32 *val);
static int pci_tegra_read_conf(struct pci_bus *bus, u32 devfn,
	int where, int size, u32 *val);

static int __init pcie_tegra_init(void);

unsigned long pci_tegra_get_base(char *aperture)
{
	if (!strcmp(aperture, "mem"))
		return 0x90000000;
	else if (!strcmp(aperture, "io"))
		return 0x82000000;
	else
		return (unsigned long)-1;
}

static int pci_tegra_read_conf(struct pci_bus *bus, u32 devfn,
	int where, int size, u32 *val)
{
	int i;

	for (i=0; i<size; i++)
		((__u8 *)val)[i] = 0xff;

	return PCIBIOS_SUCCESSFUL;
}

static int pci_tegra_write_conf(struct pci_bus *bus, u32 devfn,
	int where, int size, u32 val)
{
	return PCIBIOS_SUCCESSFUL;
}

static struct pci_ops pci_tegra_ops = {
	.read = pci_tegra_read_conf,
	.write = pci_tegra_write_conf,
};

static void __init pci_tegra_preinit(void)
{

}

static int __init pci_tegra_setup(int nr, struct pci_sys_data *data)
{
	return (nr == 0);
}

static struct pci_bus __init *pci_tegra_scan_bus(int nr,
	struct pci_sys_data *sys)
{
	if (nr == 0)
		return pci_scan_bus(sys->busnr, &pci_tegra_ops, sys);

	return NULL;
}

static struct hw_pci pci_tegra_data __initdata = {
	.nr_controllers = 2,
	.preinit = pci_tegra_preinit,
	.setup = pci_tegra_setup,
	.scan = pci_tegra_scan_bus,
	.swizzle = pci_std_swizzle,
};

late_initcall(pcie_tegra_init);

static int __init pcie_tegra_init(void)
{
	pci_common_init(&pci_tegra_data);
	return 0;
}
