/*
 * Linux platform device for DHD WLAN adapter
 *
 * Copyright (C) 1999-2014, Broadcom Corporation
 *
 *      Unless you and Broadcom execute a separate written software license
 * agreement governing use of this software, this software is licensed to you
 * under the terms of the GNU General Public License version 2 (the "GPL"),
 * available at http://www.broadcom.com/licenses/GPLv2.php, with the
 * following added to such license:
 *
 *      As a special exception, the copyright holders of this software give you
 * permission to link this software with independent modules, and to copy and
 * distribute the resulting executable under terms of your choice, provided that
 * you also meet, for each linked independent module, the terms and conditions of
 * the license of that module.  An independent module is a module which is not
 * derived from this software.  The special exception does not apply to any
 * modifications of the software.
 *
 *      Notwithstanding the above, under no circumstances may you combine this
 * software in any way with any other Broadcom software provided under a license
 * other than the GPL, without Broadcom's express prior written consent.
 *
 * $Id: dhd_linux_platdev.c 401742 2013-05-13 15:03:21Z $
 */
#include <typedefs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <bcmutils.h>
#include <linux_osl.h>
#include <dhd_dbg.h>
#include <dngl_stats.h>
#include <dhd.h>
#include <dhd_bus.h>
#include <dhd_linux.h>
#include <wl_android.h>

#define WIFI_PLAT_NAME		"bcmdhd_wlan"
#define WIFI_PLAT_NAME2		"bcm4329_wlan"
#define WIFI_PLAT_EXT		"bcmdhd_wifi_platform"

bool cfg_multichip = FALSE;
bcmdhd_wifi_platdata_t *dhd_wifi_platdata = NULL;
#if defined(DHD_OF_SUPPORT)
static bool dts_enabled = TRUE;
extern struct wifi_platform_data *wifi_plat_dev_parse_dt(struct device *);
extern const struct of_device_id wifi_dt_ids[];
#else
static bool dts_enabled = FALSE;
#endif /* defined(DHD_OF_SUPPORT) */

static int dhd_wifi_platform_load(void);

extern void *wl_cfg80211_get_dhdp(struct net_device *dev);

#ifdef ENABLE_4335BT_WAR
extern int bcm_bt_lock(int cookie);
extern void bcm_bt_unlock(int cookie);
static int lock_cookie_wifi = 'W' | 'i'<<8 | 'F'<<16 | 'i'<<24;	/* cookie is "WiFi" */
#endif /* ENABLE_4335BT_WAR */

wifi_adapter_info_t* dhd_wifi_platform_get_adapter(uint32 bus_type, uint32 bus_num, uint32 slot_num)
{
	int i;

	if (dhd_wifi_platdata == NULL)
		return NULL;

	for (i = 0; i < dhd_wifi_platdata->num_adapters; i++) {
		wifi_adapter_info_t *adapter = &dhd_wifi_platdata->adapters[i];
		if ((adapter->bus_type == -1 || adapter->bus_type == bus_type) &&
			(adapter->bus_num == -1 || adapter->bus_num == bus_num) &&
			(adapter->slot_num == -1 || adapter->slot_num == slot_num)) {
			DHD_TRACE(("found adapter info '%s'\n", adapter->name));
			return adapter;
		}
	}
	return NULL;
}

void* wifi_platform_prealloc(wifi_adapter_info_t *adapter, int section, unsigned long size)
{
	void *alloc_ptr = NULL;
	struct wifi_platform_data *plat_data;

	if (!adapter || !adapter->wifi_plat_data)
		return NULL;
	plat_data = adapter->wifi_plat_data;
	if (plat_data->mem_prealloc) {
		alloc_ptr = plat_data->mem_prealloc(plat_data, section, size);
		if (alloc_ptr) {
			DHD_INFO(("success alloc section %d\n", section));
			if (size != 0L)
				bzero(alloc_ptr, size);
			return alloc_ptr;
		} else {
			DHD_ERROR(("%s: failed to alloc static mem section %d\n",
				__FUNCTION__, section));
		}
	}

	return NULL;
}

void* wifi_platform_get_prealloc_func_ptr(wifi_adapter_info_t *adapter)
{
	struct wifi_platform_data *plat_data;

	if (!adapter || !adapter->wifi_plat_data)
		return NULL;
	plat_data = adapter->wifi_plat_data;
	return plat_data->mem_prealloc;
}

int wifi_platform_get_irq_number(wifi_adapter_info_t *adapter, unsigned long *irq_flags_ptr)
{
	if (adapter == NULL)
		return -1;
	if (irq_flags_ptr)
		*irq_flags_ptr = adapter->intr_flags;
	return adapter->irq_num;
}

int wifi_platform_set_power(wifi_adapter_info_t *adapter, bool on, unsigned long msec)
{
	int err = 0;
	struct wifi_platform_data *plat_data;

	if (!adapter || !adapter->wifi_plat_data)
		return -EINVAL;
	plat_data = adapter->wifi_plat_data;

	DHD_ERROR(("%s = %d\n", __FUNCTION__, on));
	if (plat_data->set_power) {
#ifdef ENABLE_4335BT_WAR
		if (on) {
			printk("WiFi: trying to acquire BT lock\n");
			if (bcm_bt_lock(lock_cookie_wifi) != 0)
				printk("** WiFi: timeout in acquiring bt lock**\n");
			printk("%s: btlock acquired\n", __FUNCTION__);
		}
		else {
			/* For a exceptional case, release btlock */
			bcm_bt_unlock(lock_cookie_wifi);
		}
#endif /* ENABLE_4335BT_WAR */

		err = plat_data->set_power(plat_data, on);
	}

	if (msec && !err)
		OSL_SLEEP(msec);

	adapter->powered_on = on && !err;

	return err;
}

int wifi_platform_bus_enumerate(wifi_adapter_info_t *adapter, bool device_present)
{
	int err = 0;
	struct wifi_platform_data *plat_data;

	if (!adapter || !adapter->wifi_plat_data)
		return -EINVAL;
	plat_data = adapter->wifi_plat_data;

	DHD_ERROR(("%s device present %d\n", __FUNCTION__, device_present));
	if (plat_data->set_carddetect) {
		err = plat_data->set_carddetect(plat_data, device_present);
	} else {
#if defined(CONFIG_ARCH_MSM) && defined(BCMPCIE)
		extern int msm_pcie_enumerate(u32 rc_idx);
		msm_pcie_enumerate(1);
#endif
	}
	return err;

}

int wifi_platform_get_wake_irq(wifi_adapter_info_t *adapter)
{
	struct wifi_platform_data *plat_data;

	if (!adapter || !adapter->wifi_plat_data)
		return -1;
	plat_data = adapter->wifi_plat_data;
	if (plat_data->get_wake_irq)
		return plat_data->get_wake_irq(plat_data);
	return -1;
}

bool wifi_process_partial_resume(wifi_adapter_info_t *adapter, int action)
{
#ifdef CONFIG_PARTIALRESUME
	struct wifi_platform_data *plat_data;

	if (!adapter || !adapter->wifi_plat_data)
		return false;
	plat_data = adapter->wifi_plat_data;
	if (plat_data->partial_resume)
		return plat_data->partial_resume(action);
	return false;
#else
	return false;
#endif
}

int wifi_platform_get_mac_addr(wifi_adapter_info_t *adapter, unsigned char *buf)
{
	struct wifi_platform_data *plat_data;

	DHD_ERROR(("%s\n", __FUNCTION__));
	if (!buf || !adapter || !adapter->wifi_plat_data)
		return -EINVAL;
	plat_data = adapter->wifi_plat_data;
	if (plat_data->get_mac_addr) {
		return plat_data->get_mac_addr(plat_data, buf);
	}
	return -EOPNOTSUPP;
}

void *wifi_platform_get_country_code(wifi_adapter_info_t *adapter, char *ccode,
				     u32 flags)
{
	/* get_country_code was added after 2.6.39 */
#if	(LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 39))
	struct wifi_platform_data *plat_data;

	if (!ccode || !adapter || !adapter->wifi_plat_data)
		return NULL;
	plat_data = adapter->wifi_plat_data;

	DHD_TRACE(("%s\n", __FUNCTION__));
	if (plat_data->get_country_code) {
		return plat_data->get_country_code(plat_data, ccode, flags);
	}
#endif /* (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 39)) */

	return NULL;
}

static int wifi_plat_dev_drv_probe(struct platform_device *pdev)
{
	const struct platform_device_id *pdev_id = platform_get_device_id(pdev);
	struct resource *resource = NULL;
	wifi_adapter_info_t *adapter;
	int error;

	adapter = devm_kzalloc(&pdev->dev, sizeof(wifi_adapter_info_t),
				GFP_KERNEL);
	if (!adapter)
		return -ENOMEM;

	adapter->name = "DHD generic adapter";
	adapter->bus_type = -1;
	adapter->bus_num = -1;
	adapter->slot_num = -1;
	adapter->irq_num = -1;

#if defined(DHD_OF_SUPPORT)
	adapter->wifi_plat_data = wifi_plat_dev_parse_dt(&pdev->dev);
	error = PTR_ERR_OR_ZERO(adapter->wifi_plat_data);
#else
	error = -ENXIO;
#endif

	if (!error) {
		/* Still one adapter for now */
		dhd_wifi_platdata->adapters = adapter;
	} else if (error != -ENXIO) {
		return error;
	} else if (pdev_id && pdev_id->driver_data) {
		/* Android style wifi platform data device ("bcmdhd_wlan" or "bcm4329_wlan")
		 * is kept for backward compatibility and supports only 1 adapter
		 */
		ASSERT(dhd_wifi_platdata != NULL);
		ASSERT(dhd_wifi_platdata->num_adapters == 1);

		dhd_wifi_platdata->adapters = adapter;

		adapter->wifi_plat_data = pdev->dev.platform_data;

		resource = platform_get_resource_byname(pdev, IORESOURCE_IRQ, "bcmdhd_wlan_irq");
		if (!resource)
			resource = platform_get_resource_byname(pdev, IORESOURCE_IRQ, "bcm4329_wlan_irq");
	} else {
		dhd_wifi_platdata = pdev->dev.platform_data;
	}

	if (!resource)
		resource = platform_get_resource(pdev, IORESOURCE_IRQ, 0);

	if (resource) {
		adapter->irq_num = resource->start;
		adapter->intr_flags = resource->flags & IRQF_TRIGGER_MASK;
	}

	platform_set_drvdata(pdev, adapter);
	return dhd_wifi_platform_load();
}

static int wifi_plat_dev_drv_remove(struct platform_device *pdev)
{
	wifi_adapter_info_t *adapter = platform_get_drvdata(pdev);

	ASSERT(dhd_wifi_platdata != NULL);

	if (adapter->powered_on) {
#ifdef BCMPCIE
		wifi_platform_bus_enumerate(adapter, FALSE);
		OSL_SLEEP(100);
		wifi_platform_set_power(adapter, FALSE, WIFI_TURNOFF_DELAY);
#else
		wifi_platform_set_power(adapter, FALSE, WIFI_TURNOFF_DELAY);
		wifi_platform_bus_enumerate(adapter, FALSE);
#endif /* BCMPCIE */
	}

	return 0;
}

static const struct platform_device_id wifi_platform_ids[] = {
	{ WIFI_PLAT_NAME, 1 },
	{ WIFI_PLAT_NAME2, 1 },
	{ WIFI_PLAT_EXT, 0 },
	{ /* sentinel */ }
};

static struct platform_driver wifi_platform_dev_driver = {
	.probe          = wifi_plat_dev_drv_probe,
	.remove         = wifi_plat_dev_drv_remove,
	.driver         = {
		.name   = WIFI_PLAT_NAME,
#if defined(DHD_OF_SUPPORT)
		.of_match_table = wifi_dt_ids,
#endif
	},
	.id_table	= wifi_platform_ids,
};

static int wifi_platdev_match(struct device *dev, void *data)
{
	char *name = (char*)data;
	struct platform_device *pdev = to_platform_device(dev);

	if (strcmp(pdev->name, name) == 0) {
		DHD_ERROR(("found wifi platform device %s\n", name));
		return TRUE;
	}

	return FALSE;
}

static int wifi_ctrlfunc_register_drv(void)
{
	int err = 0;
	struct device *dev;
	bool plat_dev_present;

	/* XXX: dtor: this is stupid. Whatever. I'll fix it later */
	dev = bus_find_device(&platform_bus_type, NULL, WIFI_PLAT_NAME,
			      wifi_platdev_match);
	if (dev) {
		plat_dev_present = true;
		put_device(dev);
	}

	dev = bus_find_device(&platform_bus_type, NULL, WIFI_PLAT_NAME2,
			      wifi_platdev_match);
	if (dev) {
		plat_dev_present = true;
		put_device(dev);
	}

	if (cfg_multichip) {
		dev = bus_find_device(&platform_bus_type, NULL, WIFI_PLAT_EXT,
				      wifi_platdev_match);
		if (dev) {
			plat_dev_present = true;
			put_device(dev);
		}
	}

	if (!dts_enabled && plat_dev_present) {
		DHD_ERROR(("no wifi platform data, skip\n"));
		return -ENXIO;
	}

	if (!cfg_multichip) {
		/* multi-chip support not enabled, build one adapter information for
		 * DHD (either SDIO, USB or PCIe)
		 */

		dhd_wifi_platdata = kzalloc(sizeof(bcmdhd_wifi_platdata_t), GFP_KERNEL);
		dhd_wifi_platdata->num_adapters = 1;
	}

	err = platform_driver_register(&wifi_platform_dev_driver);
	if (err) {
		DHD_ERROR(("%s: failed to register wifi ctrl func driver\n",
			__FUNCTION__));
		return err;
	}

	return 0;
}

void wifi_ctrlfunc_unregister_drv(void)
{
	platform_driver_unregister(&wifi_platform_dev_driver);

	if (!cfg_multichip) {
		kfree(dhd_wifi_platdata);
		dhd_wifi_platdata = NULL;
	}
}

int dhd_wifi_platform_register_drv(void)
{
	int err;

	err = wifi_ctrlfunc_register_drv();
	if (err) {
		if (err == -ENXIO) {
			/* wifi ctrl function does not exist */
			err = dhd_wifi_platform_load();
		} else {
			/* unregister driver due to initialization failure */
			wifi_ctrlfunc_unregister_drv();
		}
	}

	return err;
}

#ifdef BCMPCIE
static int dhd_wifi_platform_load_pcie(void)
{
	int err = 0;
	int i;
	wifi_adapter_info_t *adapter;

	BCM_REFERENCE(i);
	BCM_REFERENCE(adapter);

	if (dhd_wifi_platdata == NULL) {
		err = dhd_bus_register();
	} else {
			/* power up all adapters */
			for (i = 0; i < dhd_wifi_platdata->num_adapters; i++) {
				int retry = POWERUP_MAX_RETRY;
				adapter = &dhd_wifi_platdata->adapters[i];

				DHD_ERROR(("Power-up adapter '%s'\n", adapter->name));
				DHD_INFO((" - irq %d [flags %d], firmware: %s, nvram: %s\n",
					adapter->irq_num, adapter->intr_flags, adapter->fw_path,
					adapter->nv_path));
				DHD_INFO((" - bus type %d, bus num %d, slot num %d\n\n",
					adapter->bus_type, adapter->bus_num, adapter->slot_num));

				do {
					err = wifi_platform_set_power(adapter,
						TRUE, WIFI_TURNON_DELAY);
					if (err) {
						DHD_ERROR(("failed to power up %s,"
							" %d retry left\n",
							adapter->name, retry));
						/* WL_REG_ON state unknown, Power off forcely */
						wifi_platform_set_power(adapter,
							FALSE, WIFI_TURNOFF_DELAY);
						continue;
					} else {
						err = wifi_platform_bus_enumerate(adapter, TRUE);
						if (err) {
							DHD_ERROR(("failed to enumerate bus %s, "
								"%d retry left\n",
								adapter->name, retry));
							wifi_platform_set_power(adapter, FALSE,
								WIFI_TURNOFF_DELAY);
						} else {
							break;
						}
					}
				} while (retry--);

				if (!retry) {
					DHD_ERROR(("failed to power up %s, max retry reached**\n",
						adapter->name));
					return -ENODEV;
				}
			}

		err = dhd_bus_register();

		if (err) {
			DHD_ERROR(("%s: pcie_register_driver failed\n", __FUNCTION__));
				/* power down all adapters */
				for (i = 0; i < dhd_wifi_platdata->num_adapters; i++) {
					adapter = &dhd_wifi_platdata->adapters[i];
					wifi_platform_bus_enumerate(adapter, FALSE);
					wifi_platform_set_power(adapter,
						FALSE, WIFI_TURNOFF_DELAY);
				}
		}
	}

	return err;
}
#else
static int dhd_wifi_platform_load_pcie(void)
{
	return 0;
}
#endif /* BCMPCIE  */


void dhd_wifi_platform_unregister_drv(void)
{
	wifi_ctrlfunc_unregister_drv();
}

extern int dhd_watchdog_prio;
extern int dhd_dpc_prio;
extern uint dhd_deferred_tx;
#if defined(BCMLXSDMMC)
extern struct semaphore dhd_registration_sem;
#endif

#ifdef BCMSDIO
static int dhd_wifi_platform_load_sdio(void)
{
	int i;
	int err = 0;
	wifi_adapter_info_t *adapter;

	BCM_REFERENCE(i);
	BCM_REFERENCE(adapter);
	/* Sanity check on the module parameters
	 * - Both watchdog and DPC as tasklets are ok
	 * - If both watchdog and DPC are threads, TX must be deferred
	 */
	if (!(dhd_watchdog_prio < 0 && dhd_dpc_prio < 0) &&
		!(dhd_watchdog_prio >= 0 && dhd_dpc_prio >= 0 && dhd_deferred_tx))
		return -EINVAL;

#if defined(BCMLXSDMMC)
	if (dhd_wifi_platdata == NULL) {
		DHD_ERROR(("DHD wifi platform data is required for Android build\n"));
		return -EINVAL;
	}

	sema_init(&dhd_registration_sem, 0);
	/* power up all adapters */
	for (i = 0; i < dhd_wifi_platdata->num_adapters; i++) {
		bool chip_up = FALSE;
		int retry = POWERUP_MAX_RETRY;
		struct semaphore dhd_chipup_sem;

		adapter = &dhd_wifi_platdata->adapters[i];

		DHD_ERROR(("Power-up adapter '%s'\n", adapter->name));
		DHD_INFO((" - irq %d [flags %d], firmware: %s, nvram: %s\n",
			adapter->irq_num, adapter->intr_flags, adapter->fw_path, adapter->nv_path));
		DHD_INFO((" - bus type %d, bus num %d, slot num %d\n\n",
			adapter->bus_type, adapter->bus_num, adapter->slot_num));

		do {
			sema_init(&dhd_chipup_sem, 0);
			err = dhd_bus_reg_sdio_notify(&dhd_chipup_sem);
			if (err) {
				DHD_ERROR(("%s dhd_bus_reg_sdio_notify fail(%d)\n\n",
					__FUNCTION__, err));
				return err;
			}
			err = wifi_platform_set_power(adapter, TRUE, WIFI_TURNON_DELAY);
			if (err) {
				/* WL_REG_ON state unknown, Power off forcely */
				wifi_platform_set_power(adapter, FALSE, WIFI_TURNOFF_DELAY);
				continue;
			} else {
				wifi_platform_bus_enumerate(adapter, TRUE);
				err = 0;
			}

			if (down_timeout(&dhd_chipup_sem, msecs_to_jiffies(POWERUP_WAIT_MS)) == 0) {
				dhd_bus_unreg_sdio_notify();
				chip_up = TRUE;
				break;
			}

			DHD_ERROR(("failed to power up %s, %d retry left\n", adapter->name, retry));
			dhd_bus_unreg_sdio_notify();
			wifi_platform_set_power(adapter, FALSE, WIFI_TURNOFF_DELAY);
			wifi_platform_bus_enumerate(adapter, FALSE);
		} while (retry--);

		if (!chip_up) {
			DHD_ERROR(("failed to power up %s, max retry reached**\n", adapter->name));
			return -ENODEV;
		}

	}

	err = dhd_bus_register();

	if (err) {
		DHD_ERROR(("%s: sdio_register_driver failed\n", __FUNCTION__));
		goto fail;
	}


	/*
	 * Wait till MMC sdio_register_driver callback called and made driver attach.
	 * It's needed to make sync up exit from dhd insmod  and
	 * Kernel MMC sdio device callback registration
	 */
	err = down_timeout(&dhd_registration_sem, msecs_to_jiffies(DHD_REGISTRATION_TIMEOUT));
	if (err) {
		DHD_ERROR(("%s: sdio_register_driver timeout or error \n", __FUNCTION__));
		dhd_bus_unregister();
		goto fail;
	}

	return err;

fail:
	/* power down all adapters */
	for (i = 0; i < dhd_wifi_platdata->num_adapters; i++) {
		adapter = &dhd_wifi_platdata->adapters[i];
		wifi_platform_set_power(adapter, FALSE, WIFI_TURNOFF_DELAY);
		wifi_platform_bus_enumerate(adapter, FALSE);
	}
#else

	/* x86 bring-up PC needs no power-up operations */
	err = dhd_bus_register();

#endif

	return err;
}
#else /* BCMSDIO */
static int dhd_wifi_platform_load_sdio(void)
{
	return 0;
}
#endif /* BCMSDIO */

static int dhd_wifi_platform_load_usb(void)
{
	return 0;
}

static int dhd_wifi_platform_load()
{
	int err = 0;

		wl_android_init();

	if ((err = dhd_wifi_platform_load_usb()))
		goto end;
	else if ((err = dhd_wifi_platform_load_sdio()))
		goto end;
	else
		err = dhd_wifi_platform_load_pcie();

end:
	if (err)
		wl_android_exit();
	else
		wl_android_post_init();

	return err;
}
