/*
 * dhd_board_dragon.c
 *
 * Copyright (C) 1999-2014, Broadcom Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/resource.h>
#include <linux/platform_device.h>
#include <linux/wlan_plat.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/if.h>
#include <linux/random.h>
#include <linux/of_gpio.h>

static void (*wifi_status_cb)(int card_present, void *dev_id);
static void *wifi_status_cb_dev_id;
static int dragon_wifi_status_register(void (*callback)(int , void *), void *);

static int dragon_wifi_reset(int on);
static int dragon_wifi_power(int on);
static int dragon_wifi_set_carddetect(int val);
static int dragon_wifi_get_mac_addr(unsigned char *buf);
static void *dragon_wifi_get_country_code(char *country_iso_code, u32 flags);

static int dragon_wlan_pwr = -1;

static struct wifi_platform_data dragon_wifi_control = {
	.set_power	= dragon_wifi_power,
	.set_reset	= dragon_wifi_reset,
	.set_carddetect	= dragon_wifi_set_carddetect,
	.get_mac_addr	= dragon_wifi_get_mac_addr,
	.get_country_code	= dragon_wifi_get_country_code,
};

static struct resource wifi_resource[] = {
	[0] = {
		.name	= "bcmdhd_wlan_irq",
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL
				| IORESOURCE_IRQ_SHAREABLE,
	},
};

static struct platform_device dragon_wifi_device = {
	.name		= "bcmdhd_wlan",
	.id		= 1,
	.num_resources	= 1,
	.resource	= wifi_resource,
	.dev		= {
		.platform_data = &dragon_wifi_control,
	},
};

static int dragon_wifi_status_register(
		void (*callback)(int card_present, void *dev_id),
		void *dev_id)
{
	if (wifi_status_cb)
		return -EAGAIN;
	wifi_status_cb = callback;
	wifi_status_cb_dev_id = dev_id;
	return 0;
}

static int dragon_wifi_set_carddetect(int val)
{
	pr_debug("%s: %d\n", __func__, val);
	if (wifi_status_cb)
		wifi_status_cb(val, wifi_status_cb_dev_id);
	else
		pr_warn("%s: Nobody to notify\n", __func__);
	return 0;
}

static int dragon_wifi_power(int on)
{
	pr_info("%s: %d\n", __func__, on);

	if (dragon_wlan_pwr >= 0) {
		gpio_set_value(dragon_wlan_pwr, on);
		msleep(100);
	}

	return 0;
}

static int dragon_wifi_reset(int on)
{
	pr_debug("%s: do nothing\n", __func__);
	return 0;
}

static unsigned char dragon_mac_addr[IFHWADDRLEN] = { 0, 0x90, 0x4c, 0, 0, 0 };

static int __init dragon_mac_addr_setup(char *str)
{
	char macstr[IFHWADDRLEN*3];
	char *macptr = macstr;
	char *token;
	int i = 0;

	if (!str)
		return 0;
	pr_debug("wlan MAC = %s\n", str);
	if (strlen(str) >= sizeof(macstr))
		return 0;
	strcpy(macstr, str);

	while (((token = strsep(&macptr, ":")) != NULL) &&
		(i < IFHWADDRLEN)) {
		unsigned long val;
		int res;

		res = kstrtoul(token, 0x10, &val);
		if (res < 0)
			return 0;
		dragon_mac_addr[i++] = (u8)val;
	}

	return 1;
}

__setup("androidboot.wifimacaddr=", dragon_mac_addr_setup);

static int dragon_wifi_get_mac_addr(unsigned char *buf)
{
	uint rand_mac;

	if (!buf)
		return -EFAULT;

	if ((dragon_mac_addr[4] == 0) && (dragon_mac_addr[5] == 0)) {
		prandom_seed((uint)jiffies);
		rand_mac = prandom_u32();
		dragon_mac_addr[3] = (unsigned char)rand_mac;
		dragon_mac_addr[4] = (unsigned char)(rand_mac >> 8);
		dragon_mac_addr[5] = (unsigned char)(rand_mac >> 16);
	}
	memcpy(buf, dragon_mac_addr, IFHWADDRLEN);
	return 0;
}

#define WLC_CNTRY_BUF_SZ	4	/* Country string is 3 bytes + NULL */

static char dragon_country_code[WLC_CNTRY_BUF_SZ] = { 0 };

static int __init dragon_country_code_setup(char *str)
{
	if (!str)
		return 0;
	pr_debug("wlan country code = %s\n", str);
	if (strlen(str) >= sizeof(dragon_country_code))
		return 0;
	strcpy(dragon_country_code, str);
	return 1;
}
__setup("androidboot.wificountrycode=", dragon_country_code_setup);

struct cntry_locales_custom {
	char iso_abbrev[WLC_CNTRY_BUF_SZ];	/* ISO 3166-1 country abbreviation */
	char custom_locale[WLC_CNTRY_BUF_SZ];	/* Custom firmware locale */
	s32 custom_locale_rev;		/* Custom local revision default -1 */
};

struct cntry_locales_custom country_code_custom_table[] = {
/* Table should be filled out based on custom platform regulatory requirement */
	{"",   "XZ", 11},  /* Universal if Country code is unknown or empty */
	{"US", "US", 0},
	{"AE", "AE", 1},
	{"AR", "AR", 21},
	{"AT", "AT", 4},
	{"AU", "AU", 6},
	{"BE", "BE", 4},
	{"BG", "BG", 4},
	{"BN", "BN", 4},
	{"BR", "BR", 4},
	{"CA", "US", 0},   /* Previously was CA/31 */
	{"CH", "CH", 4},
	{"CY", "CY", 4},
	{"CZ", "CZ", 4},
	{"DE", "DE", 7},
	{"DK", "DK", 4},
	{"EE", "EE", 4},
	{"ES", "ES", 4},
	{"FI", "FI", 4},
	{"FR", "FR", 5},
	{"GB", "GB", 6},
	{"GR", "GR", 4},
	{"HK", "HK", 2},
	{"HR", "HR", 4},
	{"HU", "HU", 4},
	{"IE", "IE", 5},
	{"IN", "IN", 3},
	{"IS", "IS", 4},
	{"IT", "IT", 4},
	{"ID", "ID", 13},
	{"JP", "JP", 58},
	{"KR", "KR", 57},
	{"KW", "KW", 5},
	{"LI", "LI", 4},
	{"LT", "LT", 4},
	{"LU", "LU", 3},
	{"LV", "LV", 4},
	{"MA", "MA", 2},
	{"MT", "MT", 4},
	{"MX", "MX", 20},
	{"MY", "MY", 3},
	{"NL", "NL", 4},
	{"NO", "NO", 4},
	{"NZ", "NZ", 4},
	{"PL", "PL", 4},
	{"PT", "PT", 4},
	{"PY", "PY", 2},
	{"QA", "QA", 0},
	{"RO", "RO", 4},
	{"RU", "RU", 13},
	{"SA", "SA", 26},
	{"SE", "SE", 4},
	{"SG", "SG", 4},
	{"SI", "SI", 4},
	{"SK", "SK", 4},
	{"TH", "TH", 5},
	{"TR", "TR", 7},
	{"TW", "TW", 1},
	{"VN", "VN", 4},
	{"IR", "XZ", 11},	/* Universal if Country code is IRAN, (ISLAMIC REPUBLIC OF) */
	{"SD", "XZ", 11},	/* Universal if Country code is SUDAN */
	{"SY", "XZ", 11},	/* Universal if Country code is SYRIAN ARAB REPUBLIC */
	{"GL", "XZ", 11},	/* Universal if Country code is GREENLAND */
	{"PS", "XZ", 11},	/* Universal if Country code is PALESTINIAN TERRITORIES */
	{"TL", "XZ", 11},	/* Universal if Country code is TIMOR-LESTE (EAST TIMOR) */
	{"MH", "XZ", 11},	/* Universal if Country code is MARSHALL ISLANDS */
};

struct cntry_locales_custom country_code_nodfs_table[] = {
	{"",   "XZ", 40},  /* Universal if Country code is unknown or empty */
	{"US", "US", 172},
	{"AM", "E0", 26},
	{"AU", "AU", 37},
	{"BG", "E0", 26},
	{"BR", "BR", 18},
	{"CA", "US", 172},
	{"CH", "E0", 26},
	{"CY", "E0", 26},
	{"CZ", "E0", 26},
	{"DE", "E0", 26},
	{"DK", "E0", 26},
	{"DZ", "E0", 26},
	{"EE", "E0", 26},
	{"ES", "E0", 26},
	{"EU", "E0", 26},
	{"FI", "E0", 26},
	{"FR", "E0", 26},
	{"GB", "E0", 26},
	{"GR", "E0", 26},
	{"HK", "SG", 17},
	{"HR", "E0", 26},
	{"HU", "E0", 26},
	{"ID", "ID", 1},
	{"IE", "E0", 26},
	{"IL", "E0", 26},
	{"IN", "IN", 27},
	{"IQ", "E0", 26},
	{"IS", "E0", 26},
	{"IT", "E0", 26},
	{"JP", "JP", 83},
	{"KR", "KR", 79},
	{"KW", "E0", 26},
	{"KZ", "E0", 26},
	{"LI", "E0", 26},
	{"LT", "E0", 26},
	{"LU", "E0", 26},
	{"LV", "LV", 4},
	{"LY", "E0", 26},
	{"MA", "E0", 26},
	{"MT", "E0", 26},
	{"MY", "MY", 15},
	{"MX", "US", 172},
	{"NL", "E0", 26},
	{"NO", "E0", 26},
	{"OM", "E0", 26},
	{"PL", "E0", 26},
	{"PT", "E0", 26},
	{"QA", "QA", 0},
	{"RO", "E0", 26},
	{"RS", "E0", 26},
	{"SA", "SA", 26},
	{"SE", "E0", 26},
	{"SG", "SG", 17},
	{"SI", "E0", 26},
	{"SK", "E0", 26},
	{"SZ", "E0", 26},
	{"TH", "TH", 9},
	{"TN", "E0", 26},
	{"TR", "E0", 26},
	{"TW", "TW", 60},
	{"ZA", "E0", 26},
};

static void *dragon_wifi_get_country_code(char *country_iso_code, u32 flags)
{
	struct cntry_locales_custom *locales;
	int size, i;

	if (flags & WLAN_PLAT_NODFS_FLAG) {
		locales = country_code_nodfs_table;
		size = ARRAY_SIZE(country_code_nodfs_table);
	} else {
		locales = country_code_custom_table;
		size = ARRAY_SIZE(country_code_custom_table);
	}

	if (size == 0)
		 return NULL;

	if (!country_iso_code || country_iso_code[0] == 0)
		country_iso_code = dragon_country_code;

	for (i = 0; i < size; i++) {
		if (strcmp(country_iso_code, locales[i].iso_abbrev) == 0)
			return &locales[i];
	}
	/* if no country code matched return first universal code */
	return &locales[0];
}

int __init dhd_wifi_platform_register_device(void)
{
	struct device_node *np;
	int rc;

	np = of_find_compatible_node(NULL, NULL, "bcm,bcm4354");
	if (np) {
		rc = of_get_named_gpio(np, "wl_host_wake", 0);
		if (rc < 0)
			return rc;
		wifi_resource[0].start = wifi_resource[0].end =
			gpio_to_irq(rc);
		rc = of_get_named_gpio(np, "wl_reg_on", 0);
		if (rc < 0)
			return rc;
		dragon_wlan_pwr = rc;
	} else {
		pr_err("%s: No Wlan Node found!\n", __func__);
		return -1;
	}

	return platform_device_register(&dragon_wifi_device);
}
