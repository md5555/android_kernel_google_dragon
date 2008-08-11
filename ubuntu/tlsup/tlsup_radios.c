/* Toshiba Laptop Support -- tlsup_radios.c
 *
 * Radio kill switch support for T.L.S.
 *
 * Copyright 2008 Daniel Silverstone <dsilvers@digital-scurf.org>
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; specifically version 2
 * of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301 USA.
 *
 *
 *  Credits:
 *	Jonathan A. Buzzard - Toshiba HCI info, and critical tips on reverse
 *		engineering the Windows drivers
 *	Rob Miller - TV out and hotkeys help
 *      John Belmonte - The toshiba_acpi driver maintainance for some time.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/err.h>
#include <linux/rfkill.h>

#include "tlsup.h"

/* Module stuff */
static int radio_powerdown_time = 100;
static int powerup_settle_time = 200;

module_param(radio_powerdown_time, uint, 0400);
MODULE_PARM_DESC(radio_powerdown_time, "The number of milliseconds between powering off a radio and the device power.");
module_param(powerup_settle_time, uint, 0400);
MODULE_PARM_DESC(powerup_settle_time, "The number of milliseconds between powering on a device and applying radio power.");

/* HCI registers */
#define HCI_WIRELESS				0x0056

/* Value manipulation */
#define HCI_WIRELESS_PRESENT			0x0000
#define HCI_WIRELESS_STATUS			0x0001
#define HCI_WIRELESS_GLOBAL_RADIO_SWITCH	0x0001
#define	HCI_WIRELESS_3GHSDPA_POWER		0x4000
#define HCI_WIRELESS_3GHSDPA_RADIO		0x2000
#define	HCI_WIRELESS_BLUETOOTH_POWER		0x0040
#define HCI_WIRELESS_BLUETOOTH_RADIO		0x0080
#define HCI_WIRELESS_WLAN_RADIO			0x0200

/* Radio manipulation functions */

static u32 tlsup_radios_present = 0;

static int tlsup_test_radio_presence(u32 this_switch)
{
        u32 swvalue;
        u32 swaddr;
        u32 hci_result;
        u32 current_status;
        
        swvalue = 0;
        swaddr = HCI_WIRELESS_STATUS;
        
        if (tlsup_hci_read_2_reg(HCI_WIRELESS, &swvalue, &swaddr, &hci_result))
                return 0;
        if (hci_result != HCI_SUCCESS)
                return 0;
        
        current_status = (swvalue & this_switch) ? 1 : 0;
        
        /* Attempt to invert the current state */
        
        swvalue = !current_status;
        swaddr = this_switch;
        if (tlsup_hci_write_2_reg(HCI_WIRELESS, swvalue, swaddr, &hci_result))
                return 0;
        if (hci_result != HCI_SUCCESS)
                return 0;
        
        /* Read back the allegedly inverted switch status */
        swvalue = 0;
        swaddr = HCI_WIRELESS_STATUS;
        
        if (tlsup_hci_read_2_reg(HCI_WIRELESS, &swvalue, &swaddr, &hci_result))
                return 0;
        if (hci_result != HCI_SUCCESS)
                return 0;
        
        if (((swvalue & this_switch) ? 1 : 0) != !current_status)
                /* Failed to invert */
                return 0;
        
        /* inversion successful, attempt to restore previous value */
        
        swvalue = current_status;
        swaddr = this_switch;
        
        if (tlsup_hci_write_2_reg(HCI_WIRELESS, swvalue, swaddr, &hci_result))
                return 0;
        if (hci_result != HCI_SUCCESS)
                return 0;
        
        /* verify restoration of value */
        
        swvalue = 0;
        swaddr = HCI_WIRELESS_STATUS;
        
        if (tlsup_hci_read_2_reg(HCI_WIRELESS, &swvalue, &swaddr, &hci_result))
                return 0;
        if (hci_result != HCI_SUCCESS)
                return 0;
        
        if (((swvalue & this_switch) ? 1 : 0) != current_status)
                /* Failed to restore */
                return 0;
        
        /* Restoration successful */
        
        tlsup_radios_present |= this_switch;
        return 1;
}

static enum rfkill_state tlsup_read_switch_state(u32 this_switch)
{
        u32 swvalue = 0;
        u32 swaddr = HCI_WIRELESS_STATUS;
        u32 hci_result;
        
        if (tlsup_hci_read_2_reg(HCI_WIRELESS, &swvalue, &swaddr, &hci_result))
                return 0;
        if (hci_result != HCI_SUCCESS)
                return 0;
        
        return (swvalue & this_switch) ? RFKILL_STATE_ON : RFKILL_STATE_OFF;
}

static void tlsup_set_switch_state(u32 this_switch, enum rfkill_state state)
{
        u32 swvalue = state;
        u32 swaddr = this_switch;
        u32 hci_result;
        
        if (tlsup_hci_write_2_reg(HCI_WIRELESS, swvalue, swaddr, &hci_result)) {
                tlsup_err("Error setting switch state\n");
        }
        if (hci_result != HCI_SUCCESS) {
                tlsup_err("Error setting switch state\n");
        }
}

/* Radio rfkill driver */

struct tlsup_radio_power_control {
        u32 power_reg;
        u32 radio_reg;
};

static int tlsup_radio_switch_set(void *data, enum rfkill_state state)
{
        struct tlsup_radio_power_control *rpc = (struct tlsup_radio_power_control *)data;
        
        if (state == RFKILL_STATE_OFF) {
                if (rpc->radio_reg) {
                        tlsup_set_switch_state(rpc->radio_reg, RFKILL_STATE_OFF);
                        msleep(radio_powerdown_time);
                }
                
                if (rpc->power_reg)
                        tlsup_set_switch_state(rpc->power_reg, RFKILL_STATE_OFF);
        } else {
                if (rpc->power_reg) {
                        tlsup_set_switch_state(rpc->power_reg, RFKILL_STATE_ON);
                        msleep(powerup_settle_time);
                }
                if (rpc->radio_reg)
                        tlsup_set_switch_state(rpc->radio_reg, RFKILL_STATE_ON);
        }
        
        return 0;
}

static struct rfkill *hsdpa_rfkill_dev = NULL;
static struct rfkill *bluetooth_rfkill_dev = NULL;
static struct rfkill *wlan_rfkill_dev = NULL;

static struct rfkill *tlsup_register_rfkill(const char *name, enum rfkill_type rft,
                                            u32 optional_power,
                                            u32 optional_radio)
{
        struct tlsup_radio_power_control *rpc;
        struct rfkill *ret;
        enum rfkill_state curstate = RFKILL_STATE_OFF;
        
        ret = rfkill_allocate(NULL, rft);
        
        if (ret == NULL)
                return NULL;
        
        rpc = kmalloc(sizeof(struct tlsup_radio_power_control), GFP_KERNEL);
        if (rpc == NULL) {
                rfkill_free(ret);
                return NULL;
        }
        
        ret->name = name;
        ret->data = rpc;
        
        rpc->power_reg = optional_power;
        rpc->radio_reg = optional_radio;
        
        if (optional_power)
                curstate |= tlsup_read_switch_state(optional_power);
        if (optional_radio)
                curstate |= tlsup_read_switch_state(optional_radio);
        
        ret->state = curstate;
        
        if (optional_power)
                tlsup_set_switch_state(optional_power, curstate);
        if (optional_radio)
                tlsup_set_switch_state(optional_radio, curstate);
        
        ret->toggle_radio = tlsup_radio_switch_set;
        ret->user_claim_unsupported = 1;
        
        rfkill_register(ret);
        
        return ret;
}

/* Registration functions */

int tlsup_register_radios(void)
{
        int hsdpa_present = 0, bluetooth_present = 0, wlan_present = 0;
        int registered = 0;
        
        if (tlsup_test_radio_presence(HCI_WIRELESS_3GHSDPA_POWER)) {
                hsdpa_present = 1;
        }
        
        if (tlsup_test_radio_presence(HCI_WIRELESS_3GHSDPA_RADIO)) {
                hsdpa_present = 1;
        }
        
        if (tlsup_test_radio_presence(HCI_WIRELESS_BLUETOOTH_POWER)) {
                bluetooth_present = 1;
        }
        
        if (tlsup_test_radio_presence(HCI_WIRELESS_BLUETOOTH_RADIO)) {
                bluetooth_present = 1;
        }
        
        if (tlsup_test_radio_presence(HCI_WIRELESS_WLAN_RADIO)) {
                wlan_present = 1;
        }
        
        if (hsdpa_present) {
                tlsup_info("HSDPA Modem power switch %spresent, radio switch %spresent\n",
                           (tlsup_radios_present & HCI_WIRELESS_3GHSDPA_POWER) ? "" : "not ",
                           (tlsup_radios_present & HCI_WIRELESS_3GHSDPA_RADIO) ? "" : "not ");
                hsdpa_rfkill_dev = tlsup_register_rfkill("tlsup-hsdpa_3g_modem", RFKILL_TYPE_WLAN,
                                                         HCI_WIRELESS_3GHSDPA_POWER,
                                                         HCI_WIRELESS_3GHSDPA_RADIO);
                if (hsdpa_rfkill_dev) {
                        tlsup_info("HSDPA Modem claims to be %s\n",
                                   (hsdpa_rfkill_dev->state == RFKILL_STATE_OFF) ? "off" : "on");
                        registered++;
                } else {
                        tlsup_err("Failed to register HSDPA modem switch\n");
                }
        }
        
        if (bluetooth_present) {
                tlsup_info("Bluetooth power switch %spresent, radio switch %spresent\n",
                           (tlsup_radios_present & HCI_WIRELESS_BLUETOOTH_POWER) ? "" : "not ",
                           (tlsup_radios_present & HCI_WIRELESS_BLUETOOTH_RADIO) ? "" : "not ");
                bluetooth_rfkill_dev = tlsup_register_rfkill("tlsup-bluetooth", RFKILL_TYPE_BLUETOOTH,
                                                             HCI_WIRELESS_BLUETOOTH_POWER,
                                                             HCI_WIRELESS_BLUETOOTH_RADIO);
                if (bluetooth_rfkill_dev) {
                        tlsup_info("Bluetooth claims to be %s\n",
                                   (bluetooth_rfkill_dev->state == RFKILL_STATE_OFF) ? "off" : "on");
                        registered++;
                } else {
                        tlsup_err("Failed to register bluetooth switch\n");
                }
        }
        
        if (wlan_present) {
                tlsup_info("WLAN radio switch %spresent\n",
                           (tlsup_radios_present & HCI_WIRELESS_WLAN_RADIO) ? "" : "not ");
                wlan_rfkill_dev = tlsup_register_rfkill("tlsup-wlan", RFKILL_TYPE_WLAN,
                                                        0,
                                                        HCI_WIRELESS_WLAN_RADIO);
                if (wlan_rfkill_dev) {
                        tlsup_info("WLAN claims to be %s\n",
                                   (wlan_rfkill_dev->state == RFKILL_STATE_OFF) ? "off" : "on");
                        registered++;
                } else {
                        tlsup_err("Failed to register WLAN switch\n");
                }
        }
        
        return registered;
}

static inline void tlsup_free_rfkill_dev(struct rfkill *d)
{
        void *data = d->data;
        rfkill_unregister(d);
        kfree(data);
}

void tlsup_deregister_radios(void)
{
        if (hsdpa_rfkill_dev)
                tlsup_free_rfkill_dev(hsdpa_rfkill_dev);
        if (bluetooth_rfkill_dev)
                tlsup_free_rfkill_dev(bluetooth_rfkill_dev);
        if (wlan_rfkill_dev)
                tlsup_free_rfkill_dev(wlan_rfkill_dev);
}
