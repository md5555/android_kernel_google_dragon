/*
 * Copyright (C) 2005, 2006 IBM Corporation
 * Copyright (C) 2014 Intel Corporation
 *
 * Authors:
 * Leendert van Doorn <leendert@watson.ibm.com>
 * Kylene Hall <kjhall@us.ibm.com>
 *
 * Maintained by: <tpmdd-devel@lists.sourceforge.net>
 *
 * Device driver for TCG/TCPA TPM (trusted platform module).
 * Specifications at www.trustedcomputinggroup.org
 *
 * This device driver implements the TPM interface as defined in
 * the TCG TPM Interface Spec version 1.2, revision 1.0.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, version 2 of the
 * License.
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/pnp.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/acpi.h>
#include <linux/freezer.h>
#include "tpm.h"
#include "tpm_tis_common.h"

static void read_mem_bytes(struct tpm_chip *chip, u32 addr, u8 len, u8 size, u8 *result)
{
	int i;

	if (size == 4)
		*(u32 *)result = ioread32(chip->vendor.iobase + addr);
	else if (size == 2)
		*(u16 *)result = ioread16(chip->vendor.iobase + addr);
	else if (size == 1 && len == 1)
		*result = ioread8(chip->vendor.iobase + addr);
	else {
		for (i = 0; i < len; i++) {
			result[i] = ioread8(chip->vendor.iobase + addr);
		}
	}
}

static void write_mem_bytes(struct tpm_chip *chip, u32 addr, u8 len, u8 size, u8 *value)
{
	int i;

	if (size == 4)
		iowrite32(*(u32 *)value, chip->vendor.iobase + addr);
	else if (size == 2)
		iowrite16(*(u16 *)value, chip->vendor.iobase + addr);
	else if (size == 1 && len == 1)
		iowrite8(*value, chip->vendor.iobase + addr);
	else {
		for (i = 0 ; i < len; i++) {
			iowrite8(value[i], chip->vendor.iobase + addr);
		}
	}
}

#if defined(CONFIG_PNP) && defined(CONFIG_ACPI)
static int is_itpm(struct pnp_dev *dev)
{
	struct acpi_device *acpi = pnp_acpi_device(dev);
	struct acpi_hardware_id *id;

	if (!acpi)
		return 0;

	list_for_each_entry(id, &acpi->pnp.ids, list) {
		if (!strcmp("INTC0102", id->id))
			return 1;
	}

	return 0;
}
#else
static inline int is_itpm(struct pnp_dev *dev)
{
	return 0;
}
#endif

static bool itpm;
module_param(itpm, bool, 0444);
MODULE_PARM_DESC(itpm, "Force iTPM workarounds (found on some Lenovo laptops)");

static const struct tpm_class_ops tpm_tis = {
	.status = tpm_tis_status,
	.recv = tpm_tis_recv,
	.send = tpm_tis_send,
	.cancel = tpm_tis_ready,
	.update_timeouts = tpm_tis_update_timeouts,
	.req_complete_mask = TPM_STS_DATA_AVAIL | TPM_STS_VALID,
	.req_complete_val = TPM_STS_DATA_AVAIL | TPM_STS_VALID,
	.req_canceled = tpm_tis_req_canceled,
	.read_bytes = read_mem_bytes,
	.write_bytes = write_mem_bytes,
};

static bool interrupts = true;
module_param(interrupts, bool, 0444);
MODULE_PARM_DESC(interrupts, "Enable interrupts");

static int tpm_tis_init(struct device *dev, acpi_handle acpi_dev_handle,
			resource_size_t start, resource_size_t len,
			unsigned int irq)
{
	struct tpm_chip *chip;
	struct priv_data *priv;

	priv = devm_kzalloc(dev, sizeof(struct priv_data), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	chip = tpmm_chip_alloc(dev, &tpm_tis);
	if (IS_ERR(chip))
		return PTR_ERR(chip);

	chip->vendor.priv = priv;
#ifdef CONFIG_ACPI
	chip->acpi_dev_handle = acpi_dev_handle;
#endif

	chip->vendor.iobase = devm_ioremap(dev, start, len);
	if (!chip->vendor.iobase)
		return -EIO;
	return tpm_tis_init_generic(dev, chip, irq, interrupts, itpm);
}

static SIMPLE_DEV_PM_OPS(tpm_tis_pm, tpm_pm_suspend, tpm_tis_resume);

#ifdef CONFIG_PNP
static int tpm_tis_pnp_init(struct pnp_dev *pnp_dev,
			    const struct pnp_device_id *pnp_id)
{
	resource_size_t start, len;
	unsigned int irq = 0;
	acpi_handle acpi_dev_handle = NULL;

	start = pnp_mem_start(pnp_dev, 0);
	len = pnp_mem_len(pnp_dev, 0);

	if (pnp_irq_valid(pnp_dev, 0))
		irq = pnp_irq(pnp_dev, 0);
	else
		interrupts = false;

	if (is_itpm(pnp_dev))
		itpm = true;

#ifdef CONFIG_ACPI
	if (pnp_acpi_device(pnp_dev))
		acpi_dev_handle = pnp_acpi_device(pnp_dev)->handle;
#endif

	return tpm_tis_init(&pnp_dev->dev, acpi_dev_handle, start, len, irq);
}

static struct pnp_device_id tpm_pnp_tbl[] = {
	{"PNP0C31", 0},		/* TPM */
	{"ATM1200", 0},		/* Atmel */
	{"IFX0102", 0},		/* Infineon */
	{"BCM0101", 0},		/* Broadcom */
	{"BCM0102", 0},		/* Broadcom */
	{"NSC1200", 0},		/* National */
	{"ICO0102", 0},		/* Intel */
	/* Add new here */
	{"", 0},		/* User Specified */
	{"", 0}			/* Terminator */
};
MODULE_DEVICE_TABLE(pnp, tpm_pnp_tbl);

static void tpm_tis_pnp_remove(struct pnp_dev *dev)
{
	struct tpm_chip *chip = pnp_get_drvdata(dev);

	tpm_chip_unregister(chip);
	tpm_tis_remove(chip);
}

static struct pnp_driver tis_pnp_driver = {
	.name = "tpm_tis",
	.id_table = tpm_pnp_tbl,
	.probe = tpm_tis_pnp_init,
	.remove = tpm_tis_pnp_remove,
	.driver	= {
		.pm = &tpm_tis_pm,
	},
};

#define TIS_HID_USR_IDX (sizeof(tpm_pnp_tbl) / sizeof(struct pnp_device_id) - 2)
module_param_string(hid, tpm_pnp_tbl[TIS_HID_USR_IDX].id,
		    sizeof(tpm_pnp_tbl[TIS_HID_USR_IDX].id), 0444);
MODULE_PARM_DESC(hid, "Set additional specific HID for this driver to probe");
#endif

static struct platform_driver tis_drv = {
	.driver = {
		.name		= "tpm_tis",
		.pm		= &tpm_tis_pm,
	},
};

static struct platform_device *pdev;

static bool force;
module_param(force, bool, 0444);
MODULE_PARM_DESC(force, "Force device probe rather than using ACPI entry");
static int __init init_tis(void)
{
	int rc;
#ifdef CONFIG_PNP
	if (!force)
		return pnp_register_driver(&tis_pnp_driver);
#endif

	rc = platform_driver_register(&tis_drv);
	if (rc < 0)
		return rc;
	pdev = platform_device_register_simple("tpm_tis", -1, NULL, 0);
	if (IS_ERR(pdev)) {
		rc = PTR_ERR(pdev);
		goto err_dev;
	}
	rc = tpm_tis_init(&pdev->dev, NULL, TIS_MEM_BASE, TIS_MEM_LEN, 0);
	if (rc)
		goto err_init;
	return 0;
err_init:
	platform_device_unregister(pdev);
err_dev:
	platform_driver_unregister(&tis_drv);
	return rc;
}

static void __exit cleanup_tis(void)
{
	struct tpm_chip *chip;
#ifdef CONFIG_PNP
	if (!force) {
		pnp_unregister_driver(&tis_pnp_driver);
		return;
	}
#endif
	chip = dev_get_drvdata(&pdev->dev);
	tpm_chip_unregister(chip);
	tpm_tis_remove(chip);
	platform_device_unregister(pdev);
	platform_driver_unregister(&tis_drv);
}

module_init(init_tis);
module_exit(cleanup_tis);
MODULE_AUTHOR("Leendert van Doorn (leendert@watson.ibm.com)");
MODULE_DESCRIPTION("TPM Driver");
MODULE_VERSION("2.0");
MODULE_LICENSE("GPL");
