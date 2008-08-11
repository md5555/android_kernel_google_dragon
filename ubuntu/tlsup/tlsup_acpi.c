/* Toshiba Laptop Support -- tlsup_acpi.c
 *
 * ACPI support for T.L.S.
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

#include <acpi/acpi_drivers.h>

#include "tlsup.h"

/* Toshibas use one of two ACPI method named typically.
 * 
 * The one we detect goes in here.
 */
static const char *hci_method = NULL;

#define HCI_METHOD_1		"\\_SB_.VALD.GHCI"
#define HCI_METHOD_2		"\\_SB_.VALZ.GHCI"

/* Check if the provided method name is a valid ACPI method for
 * the machine.
 *
 * Return 0 if it is not valid, 1 if it is.
 */
static int is_valid_acpi_path(const char *method_name)
{
	acpi_handle handle;
	acpi_status status;

	status = acpi_get_handle(NULL, (char *)method_name, &handle);
	return !ACPI_FAILURE(status);
}


/* Detect the toshiba ACPI support and prepare the HCI.
 * Returns zero on success, -errno on failure.
 */
int tlsup_acpi_detect(void)
{
        if (acpi_disabled)
                return -ENODEV;
        
	if (is_valid_acpi_path(HCI_METHOD_1))
		hci_method = HCI_METHOD_1;
	else if (is_valid_acpi_path(HCI_METHOD_2))
		hci_method = HCI_METHOD_2;
	else
		return -ENODEV;
        
        return 0;
}

void tlsup_acpi_report(void)
{
	tlsup_info("    HCI method: %s\n", hci_method);
}

/* Toshiba HCI IO */

#define HCI_WORDS (6)

/* operations */
#define HCI_SET				0xff00
#define HCI_GET				0xfe00

static acpi_status tlsup_hci_raw_io(const u32 input[HCI_WORDS], u32 output[HCI_WORDS])
{
	struct acpi_object_list params;
	union acpi_object in_objs[HCI_WORDS];
	struct acpi_buffer results;
	union acpi_object out_objs[HCI_WORDS + 1];
	acpi_status status;
	int i;

	params.count = HCI_WORDS;
	params.pointer = in_objs;
	for (i = 0; i < HCI_WORDS; ++i) {
		in_objs[i].type = ACPI_TYPE_INTEGER;
		in_objs[i].integer.value = input[i];
	}

	results.length = sizeof(out_objs);
	results.pointer = out_objs;

	status = acpi_evaluate_object(NULL, (char *)hci_method, &params,
				      &results);
	if ((status == AE_OK) && (out_objs->package.count <= HCI_WORDS)) {
		for (i = 0; i < out_objs->package.count; ++i) {
			output[i] = out_objs->package.elements[i].integer.value;
		}
	}

	return status;
}

int tlsup_hci_read_1_reg(u32 reg, u32 *result, u32 *hci_status)
{
        u32 hci_input[HCI_WORDS] = { HCI_GET, reg, 0, 0, 0, 0 };
        u32 hci_output[HCI_WORDS];
        acpi_status status = tlsup_hci_raw_io(hci_input, hci_output);
        *result = hci_output[2];
        *hci_status = (status == AE_OK) ? hci_output[0] : HCI_FAILURE;
        return (status == AE_OK) ? 0 : -EIO;
}

int tlsup_hci_write_1_reg(u32 reg, u32 value, u32 *hci_status)
{
        u32 hci_input[HCI_WORDS] = { HCI_SET, reg, value, 0, 0, 0 };
        u32 hci_output[HCI_WORDS];
        acpi_status status = tlsup_hci_raw_io(hci_input, hci_output);
        *hci_status = (status == AE_OK) ? hci_output[0] : HCI_FAILURE;
        return (status == AE_OK) ? 0 : -EIO;
}

int tlsup_hci_read_2_reg(u32 reg, u32 *result1, u32 *result2, u32 *hci_status)
{
        u32 hci_input[HCI_WORDS] = { HCI_GET, reg, *result1, *result2, 0, 0 };
        u32 hci_output[HCI_WORDS];
        acpi_status status = tlsup_hci_raw_io(hci_input, hci_output);
        *result1 = hci_output[2];
        *result2 = hci_output[3];
        *hci_status = (status == AE_OK) ? hci_output[0] : HCI_FAILURE;
        return (status == AE_OK) ? 0 : -EIO;
}

int tlsup_hci_write_2_reg(u32 reg, u32 value1, u32 value2, u32 *hci_status)
{
        u32 hci_input[HCI_WORDS] = { HCI_SET, reg, value1, value2, 0, 0 };
        u32 hci_output[HCI_WORDS];
        acpi_status status = tlsup_hci_raw_io(hci_input, hci_output);
        *hci_status = (status == AE_OK) ? hci_output[0] : HCI_FAILURE;
        return (status == AE_OK) ? 0 : -EIO;
}

