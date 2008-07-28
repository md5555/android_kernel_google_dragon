/*
 * Part of Intel(R) Manageability Engine Interface Linux driver
 *
 * Copyright (c) 2003 - 2007 Intel Corp.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions, and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce at minimum a disclaimer
 *    substantially similar to the "NO WARRANTY" disclaimer below
 *    ("Disclaimer") and any redistribution must be conditioned upon
 *    including a substantially similar Disclaimer requirement for further
 *    binary redistribution.
 * 3. Neither the names of the above-listed copyright holders nor the names
 *    of any contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * Alternatively, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2 as published by the Free
 * Software Foundation.
 *
 * NO WARRANTY
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDERS OR CONTRIBUTORS BE LIABLE FOR SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGES.
 *
 */

#ifndef _HECI_H_
#define _HECI_H_

#include <linux/version.h>
#include <linux/spinlock.h>
#include <linux/list.h>
#include <linux/pci.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/module.h>
#include <linux/aio.h>
#include <linux/types.h>
#include "heci_data_structures.h"


extern const struct guid heci_pthi_guid;
extern const struct guid heci_wd_guid;
extern const __u8 start_wd_params[];
extern const __u8 stop_wd_params[];
extern const __u8 heci_wd_state_independence_msg[2][4];

/**
 * memory IO BAR definition
 */
#define     BAR_0                        0
#define     BAR_1                        1
#define     BAR_5                        5
/**
 * Number of queue lists used by this driver
 */
#define    PCI_HECI_DEVICE_ID1   0x2974
#define    PCI_HECI_DEVICE_ID2   0x2984
#define    PCI_HECI_DEVICE_ID3   0x2994
#define    PCI_HECI_DEVICE_ID4   0x29A4
#define    PCI_HECI_DEVICE_ID5   0x29B4
#define    PCI_HECI_DEVICE_ID6   0x29C4
#define    PCI_HECI_DEVICE_ID7   0x29E4
#define    PCI_HECI_DEVICE_ID8   0x29F4

/**
 * heci init function prototypes
 */
struct iamt_heci_device *init_heci_device(struct pci_dev *pdev);
void heci_reset(struct iamt_heci_device * device_object, int interrupts);
int heci_hw_init(struct iamt_heci_device * device_object);
int heci_initialize_clients(void *data);
struct heci_file_private *alloc_priv(struct file *file);
int heci_disconnect_host_client(struct iamt_heci_device * device_object,
				struct heci_file_private * file_extension);
void heci_initialize_list(struct io_heci_list *list,
			  struct iamt_heci_device * device_object);
void heci_flush_list(struct io_heci_list *list,
		     struct heci_file_private * file_extension);
void heci_flush_queues(struct iamt_heci_device * device_object,
		       struct heci_file_private * file_extension);

void heci_remove_client_from_file_list(struct iamt_heci_device * device_object,
				       __u8 host_client_id);

/**
 *  interrupt function prototype
 */
irqreturn_t heci_isr_interrupt(int irq, void *dev_id);
void heci_wd_timer(unsigned long data);
void heci_bh_handler(struct work_struct *work);
/**
 *  input output function prototype
 */
int heci_ioctl_get_version(struct iamt_heci_device * device, int if_num,
			   struct heci_message_data *u_msg,
			   struct heci_message_data k_msg,
			   struct heci_file_private * file_extension);
int heci_ioctl_connect_client(struct iamt_heci_device * device, int if_num,
			      struct heci_message_data *u_msg,
			      struct heci_message_data k_msg,
			      struct file *file);
int heci_ioctl_wd(struct iamt_heci_device * device, int if_num,
		  struct heci_message_data k_msg,
		  struct heci_file_private * file_extension);
int heci_ioctl_bypass_wd(struct iamt_heci_device * device, int if_num,
		  struct heci_message_data k_msg,
		  struct heci_file_private * file_extension);
int legacy_ioctl_send_message(struct iamt_heci_device * device, int if_num,
			      struct heci_message_data k_msg,
			      struct file *file);
int legacy_ioctl_receive_message(struct iamt_heci_device * device, int if_num,
				 struct heci_message_data *u_msg,
				 struct heci_message_data k_msg,
				 struct file *file);
int heci_start_read(struct iamt_heci_device * device, int if_num,
		    struct heci_file_private * file_extension);
int pthi_write(struct iamt_heci_device * device,
	       struct heci_cb_private *kernel_priv_cb);
int pthi_read(struct iamt_heci_device * device, int if_num, struct file *file,
	      char *ubuf, size_t length, loff_t* offset);
struct heci_cb_private* find_pthi_read_list_entry(struct iamt_heci_device* device,
					struct file* file, struct heci_file_private* file_extension);
void run_next_legacy_cmd(struct iamt_heci_device * device);

#endif				/* _HECI_H_ */
