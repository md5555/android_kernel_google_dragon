/* Toshiba Laptop Support -- tlsup_acpi.h
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

#ifndef TLSUP_ACPI_H
#define TLSUP_ACPI_H

/* Detect the toshiba ACPI support and prepare the HCI.
 * Returns zero on success, -errno on failure.
 */
extern int tlsup_acpi_detect(void);

/* Report the details of the chosen/found ACPI interface to the kernel
 * log
 */
extern void tlsup_acpi_report(void);

/* Toshiba HCI */

#define HCI_SUCCESS			0x0000
#define HCI_FAILURE			0x1000
#define HCI_NOT_SUPPORTED		0x8000
#define HCI_EMPTY			0x8c00

/* perform an HCI read operation, returning one u32 value and the
 * HCI status.
 *
 * The function itself returns 0 on success, -errno on failure.
 */
extern int tlsup_hci_read_1_reg(u32 reg, u32 *result, u32 *hci_status);

/* perform an HCI write operation with one u32 value and returning the
 * HCI status.
 *
 * The function itself returns 0 on success, -errno on failure.
 */
extern int tlsup_hci_write_1_reg(u32 reg, u32 value, u32 *hci_status);

/* perform an HCI read operation, returning two u32 values and the HCI
 * status.
 *
 * The two values are passed to the call, so zero them if you don't
 * want that.
 *
 * The function itself returns 0 on success, -errno on failure.
 */
extern int tlsup_hci_read_2_reg(u32 reg, u32 *result1, u32 *result2, u32 *hci_status);

/* perform an HCI write operation with two u32 values and returning
 * the HCI status.
 *
 * The function itself returns 0 on success, -errno on failure.
 */
extern int tlsup_hci_write_2_reg(u32 reg, u32 value1, u32 value2, u32 *hci_status);

#endif /* TLSUP_ACPI_H */
