/*
 * Part of Intel(R) Manageability Engine Interface Linux driver
 *
 * Copyright (c) 2007 Intel Corp.
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

#include <linux/module.h>
#include <linux/pci.h>
#include <linux/reboot.h>
#include <asm/uaccess.h>
#include <linux/poll.h>
#include <linux/init.h>
#include <linux/kdev_t.h>
#include <linux/moduleparam.h>
#include <linux/wait.h>
#include <linux/delay.h>

#include "kcompat.h"
#include "heci_data_structures.h"
#include "heci_interface.h"
#include "heci.h"


const __u8 watch_dog_data[] =
    { 1, 1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14 };
const __u8 start_wd_params[] = { 0x02, 0x12, 0x13, 0x10 };
const __u8 stop_wd_params[] = { 0x02, 0x02, 0x14, 0x10 };
const __u8 heci_wd_state_independence_msg[2][4] = {
	{0x05, 0x02, 0x51, 0x10}, 
	{0x05, 0x02, 0x52, 0x10} };
const struct guid heci_asf_guid =
    { 0x75B30CD6, 0xA29E, 0x4AF7, {0xA7, 0x12, 0xE6, 0x17, 0x43, 0x93,
				   0xC8, 0xA6} };
const struct guid heci_wd_guid =
    { 0x05B79A6F, 0x4628, 0x4D7F, {0x89, 0x9D, 0xA9, 0x15, 0x14, 0xCB,
				   0x32, 0xAB} };
const struct guid heci_pthi_guid =
    { 0x12f80028, 0xb4b7, 0x4b2d, {0xac, 0xa8, 0x46, 0xe0, 0xff, 0x65,
				   0x81, 0x4c} };


/**
 *  heci init function prototypes
 */
int host_start_message(struct iamt_heci_device * device_object);
int host_enum_clients_message(struct iamt_heci_device * device_object);
int allocate_me_clents_storage(struct iamt_heci_device * device_object);
void heci_disable(struct iamt_heci_device * device_object);
void host_init_wd(struct iamt_heci_device * device_object);
void host_init_legacy(struct iamt_heci_device * device_object);


/**
 * heci_initialize_list - Sets up a  queue  list.
 *
 * @list - An instance of our list structure
 * @device_object -Device object for our driver
 *
 * @return :
 * none;
 */
void heci_initialize_list(struct io_heci_list *list,
			  struct iamt_heci_device * device_object)
{
	/* initialize our queue list */
	INIT_LIST_HEAD(&list->heci_cb.cb_list);
	list->status = ESUCCESS;
	list->device_extension = device_object;
	return;
}

/**
 * heci_flush_queues - flush our queues list belong to file_extension.
 *
 * @device_object -Device object for our driver
 *
 * @return :
 * none;
 */
void heci_flush_queues(struct iamt_heci_device * device_object,
		       struct heci_file_private * file_extension)
{
	int i;
	if (!device_object || !file_extension)
		return;
	/* flush our queue list belong to file_extension */
	for (i = 0; i < NUMBER_OF_LISTS; i++) {
		DBG("remove list etnry belong to file_extension\n");
		heci_flush_list(device_object->io_list_array[i],
				file_extension);
	}

}


/**
 * heci_flush_list - remove list etnry belong to file_extension.
 *
 * @list - An instance of our list structure
 * @file_extension -extension of the file object

 * @return :
 * none;
 */
void heci_flush_list(struct io_heci_list *list,
		struct heci_file_private * file_extension)
{
	struct heci_file_private *file_extension_temp = NULL;
	struct heci_cb_private *kernel_priv_cb_pos = NULL, *kernel_priv_cb_next = NULL;

	if (!list || !file_extension)
		return;
	if (list->status == ESUCCESS
	    && !list_empty(&list->heci_cb.cb_list)) {
        list_for_each_entry_safe(kernel_priv_cb_pos, kernel_priv_cb_next, &list->heci_cb.cb_list, cb_list){

            if (kernel_priv_cb_pos) {
				file_extension_temp =
				    (struct heci_file_private *)
				    kernel_priv_cb_pos->file_private;
			}
			if (file_extension_temp) {
				if ((file_extension->host_client_id == file_extension_temp-> host_client_id)
				    && (file_extension->me_client_id == file_extension_temp-> me_client_id))
					list_del(&kernel_priv_cb_pos->cb_list);
			}

		}
	}
	return;
}

/**
 * init_heci_device - allocates and initializes the heci device structure
 * @pdev: The pci device structure
 *
 * @return :
 * The heci_device_device pointer on success, NULL on failure.
 */
struct iamt_heci_device *init_heci_device(struct pci_dev * pdev)
{
	int i;
	struct iamt_heci_device *device;
	device = kmalloc(sizeof(struct iamt_heci_device), GFP_KERNEL);
	if (!device) {
		return NULL;
	}

	/* setup our list array */
	device->io_list_array[0] = &device->read_list;
	device->io_list_array[1] = &device->write_list;
	device->io_list_array[2] = &device->write_waiting_list;
	device->io_list_array[3] = &device->control_write_list;
	device->io_list_array[4] = &device->control_read_list;
	device->io_list_array[5] = &device->pthi_cmd_list;
	device->io_list_array[6] = &device->pthi_read_complete_list;
	INIT_LIST_HEAD(&device->file_list);
	INIT_LIST_HEAD(&device->wd_file_extension.link);
	INIT_LIST_HEAD(&device->legacy_file_extension.link);
	spin_lock_init(&device->device_lock);
	init_waitqueue_head(&device->wait_received_message);
	init_waitqueue_head(&device->wait_stop_wd);
	device->open_handle_count = 0;
	device->num_heci_me_clients = 0;
	device->mem_base = 0;
	device->mem_length = 0;
	device->extra_write_index = 0;
	device->read_message_header = 0;
	device->mem_addr = NULL;
	device->asf_mode = FALSE;
	device->need_reset = FALSE;
	device->received_message = FALSE;
	device->heci_state = HECI_INITIALIZING;

	device->num_heci_me_clients = 0;
	device->legacy_current_cb = NULL;
	device->legacy_file_object = NULL;
	device->legacy_canceled = FALSE;
	device->legacy_flow_control_pending = FALSE;
	device->legacy_state = HECI_LEGACY_IDLE;
	device->legacy_message_buffer_index = 0;
	device->wd_pending = FALSE;
	device->wd_stoped = FALSE;
	device->wd_bypass = FALSE;

	device->me_clients = NULL;
	/* init work for schedule work */
	INIT_WORK(&device->work, NULL);
	for (i = 0; i < NUMBER_OF_LISTS; i++)
		heci_initialize_list(device->io_list_array[i], device);
	device->pdev = pdev;
	return device;
}

/**
 * heci_hw_init  - init host and fw to start work.
 *
 * @device_object -Device object for our driver
 *
 *@return:
 * 0 on success.
 * negative on failure
 */
int heci_hw_init(struct iamt_heci_device * device_object)
{
	int err = 0;
	device_object->host_hw_state =
	    read_heci_register(device_object, H_CSR);
	device_object->me_hw_state =
	    read_heci_register(device_object, ME_CSR_HA);
	DBG("host_hw_state = 0x%08x, mestate = 0x%08x.\n",
	    device_object->host_hw_state, device_object->me_hw_state);

	if ((device_object->host_hw_state & H_IS) == H_IS) {
		/* acknowledge interrupt and stop interupts */
		write_heci_register(device_object, H_CSR,
				    device_object->host_hw_state);
	}
	device_object->received_message = FALSE;
	DBG("reset in start the heci device.\n");

	heci_reset(device_object, TRUE);

	DBG("host_hw_state = 0x%08x, me_hw_state = 0x%08x.\n",
	    device_object->host_hw_state, device_object->me_hw_state);

	/* wait for ME to turn on ME_RDY */
	if (!device_object->received_message) {
		err =
		    wait_event_interruptible_timeout(device_object->
						     wait_received_message,
						     (device_object->
						      received_message),
						     HECI_INTEROP_TIMEOUT);
	}

	if (!err && !device_object->received_message) {
		device_object->heci_state = HECI_DISABLED;
		DBG("wait_event_interruptible_timeout failed on wait for ME to turn on ME_RDY.\n");
		return -ENODEV;
	} else {
		if (!(((device_object->host_hw_state & H_RDY) == H_RDY)
		      && ((device_object->me_hw_state & ME_RDY_HRA) ==
			  ME_RDY_HRA))) {
			device_object->heci_state = HECI_DISABLED;
			DBG("host_hw_state = 0x%08x, me_hw_state = 0x%08x.\n",
					device_object->host_hw_state,
					device_object->me_hw_state);

			if (!(device_object->host_hw_state & H_RDY) != H_RDY)
				DBG("host turn off H_RDY.\n");
			if (!(device_object->me_hw_state & ME_RDY_HRA) != ME_RDY_HRA)
				DBG("ME turn off ME_RDY.\n");
			HECI_ERR("link layer initialization failed.\n");
			return -ENODEV;
		}
	}
	device_object->received_message = FALSE;
	DBG("host_hw_state = 0x%08x, me_hw_state = 0x%08x.\n",
	    device_object->host_hw_state, device_object->me_hw_state);
	DBG("ME turn on ME_RDY and host turn on H_RDY.\n");
	HECI_INFO("link layer has been established.\n");
	return ESUCCESS;
}

/**
 * heci_reset  - reset host and fw.
 *
 * @device_object -Device object for our driver
 * @interrupts - if interrupt should be enable after reset.
 *
 * @return:
 * none;
 */
void heci_reset(struct iamt_heci_device * device_object, int interrupts)
{
       struct heci_file_private *file_extension_pos = NULL;
	struct heci_file_private *file_extension_next = NULL;
	struct heci_cb_private *kernel_priv_cb_pos = NULL, *kernel_priv_cb_next = NULL;
	int unexpected = 0;

	if (device_object->heci_state == HECI_RECOVERING_FROM_RESET) {
		device_object->need_reset = TRUE;
		return;
	}

	if (device_object->heci_state != HECI_INITIALIZING &&
	    device_object->heci_state != HECI_DISABLED && 
		device_object->heci_state != HECI_POWER_DOWN && 
		device_object->heci_state != HECI_POWER_UP)
		unexpected = 1; 
		 
	device_object->host_hw_state =
	    read_heci_register(device_object, H_CSR);

	DBG("before reset host_hw_state = 0x%08x.\n",
	    device_object->host_hw_state);

	device_object->host_hw_state |= (H_RST | H_IG);

	if (interrupts)
		device_object->host_hw_state |= (H_IE);
	else
		device_object->host_hw_state &= ~(H_IE);

	write_heci_register(device_object, H_CSR,
			    device_object->host_hw_state);

	device_object->host_hw_state =
	    read_heci_register(device_object, H_CSR);
	BUG_ON((device_object->host_hw_state & H_RST) != H_RST);
	BUG_ON((device_object->host_hw_state & H_RDY) != 0);

	device_object->host_hw_state &= ~H_RST;
	device_object->host_hw_state |= H_IG;

	write_heci_register(device_object, H_CSR,
			    device_object->host_hw_state);

	DBG("currently saved host_hw_state = 0x%08x.\n",
	    device_object->host_hw_state);

	device_object->need_reset = FALSE;

	if (device_object->heci_state != HECI_INITIALIZING) {
		if (device_object->heci_state != HECI_DISABLED && 
			device_object->heci_state != HECI_POWER_DOWN) {
			device_object->heci_state = HECI_RESETING;
		}
		list_for_each_entry_safe(file_extension_pos, file_extension_next, &device_object->file_list, link) {
			file_extension_pos->state =HECI_FILE_DISCONNECTED;
			file_extension_pos->flow_control_credentials =0;
			file_extension_pos->read_cb = NULL;
			file_extension_pos->timer_count = 0;
		}
		/* remove entry if already in list */
		DBG("list del legacy and wd file list.\n");
		heci_remove_client_from_file_list(device_object,
						  device_object->
						  wd_file_extension.
						  host_client_id);

		heci_remove_client_from_file_list(device_object,
						  device_object->
						  legacy_file_extension.
						  host_client_id);
		/* reset legacy parameters. */
		device_object->legacy_current_cb = NULL;
		device_object->legacy_message_buffer_size = 0;
		device_object->legacy_message_buffer_index = 0;
		device_object->legacy_canceled = FALSE;
		device_object->legacy_file_extension.file = NULL;
		device_object->legacy_ioctl = FALSE;
		device_object->legacy_state = HECI_LEGACY_IDLE;
		device_object->legacy_timer = 0;
		device_object->wd_due_counter = 0;
		device_object->extra_write_index = 0;
		device_object->wd_pending = FALSE;
	}

	device_object->num_heci_me_clients = 0;
	device_object->read_message_header = 0;
	device_object->stop = FALSE;
	device_object->wd_pending = 0;

	/* update the state of the registers after reset */
	device_object->host_hw_state =
	    read_heci_register(device_object, H_CSR);
	device_object->me_hw_state =
	    read_heci_register(device_object, ME_CSR_HA);

	DBG("after reset host_hw_state = 0x%08x, me_hw_state = 0x%08x.\n",
	    device_object->host_hw_state, device_object->me_hw_state);

	if (unexpected)
		HECI_ERR("unexpected heci reset.\n");
	//Wake up all readings so they can be interrupted
	list_for_each_entry_safe(file_extension_pos,file_extension_next, &device_object->file_list,link) {
			if (&file_extension_pos->rx_wait &&
				waitqueue_active (&file_extension_pos->rx_wait)) {
					HECI_INFO("Waking up client!\n");
					wake_up_interruptible(&file_extension_pos->rx_wait);
			}
		}
	// remove all waiting requests
	if (device_object->write_list.status == ESUCCESS && !list_empty(&device_object->write_list.heci_cb.cb_list)) {
		list_for_each_entry_safe(kernel_priv_cb_pos, kernel_priv_cb_next, &device_object->write_list.heci_cb.cb_list, cb_list){
				if (kernel_priv_cb_pos) {
					list_del(&kernel_priv_cb_pos->cb_list);
					kfree(kernel_priv_cb_pos->request_buffer.data);
					kernel_priv_cb_pos->request_buffer.data = NULL;
					kfree(kernel_priv_cb_pos->response_buffer.data);
					kernel_priv_cb_pos->response_buffer.data = NULL;
					kfree(kernel_priv_cb_pos);
					kernel_priv_cb_pos = NULL;
				}
		}
	}
}

/**
 * heci_disable  - reseting in disable routine.
 *
 * @device_object -Device object for our driver
 *
 * @return:
 * none;
 */
void heci_disable(struct iamt_heci_device * device_object)
{
	if (device_object->heci_state != HECI_INITIALIZING)
		HECI_ERR("driver stop request heci state is disable.\n");
	device_object->heci_state = HECI_DISABLED;
}

/**
 * heci_initialize_clients  -  routine.
 *
 * @device_object -Device object for our driver
 *
 * @return:
 * none;
 */
int heci_initialize_clients(void *data)
{

	int status;
	struct iamt_heci_device *device_object = (struct iamt_heci_device *) data;
	DBG("link is established start sending messages.\n");
	/* link is established start sending messages. */
	status = host_start_message(device_object);
	if (status) {
		DBG("start sending messages failed.\n");
		return -ENODEV;
	}
	/* enumerate clients */

	status = host_enum_clients_message(device_object);
	if (status) {
		DBG("enum clients failed.\n");
		return -ENODEV;
	}
	/* allocate storage for ME clients representation */
	status = allocate_me_clents_storage(device_object);
	if (status) {
		DBG("allocate clients failed.\n");
		return -ENODEV;
	}
	/*heci initialization wd */
	host_init_wd(device_object);
	/*heci initialization legacy client */
	host_init_legacy(device_object);
	if (device_object->need_reset) {
		device_object->need_reset = FALSE;
		device_object->heci_state = HECI_DISABLED;
		return -ENODEV;
	}

	memset(device_object->heci_host_clients, 0,
	       sizeof(device_object->heci_host_clients));
	device_object->open_handle_count = 0;
	device_object->heci_host_clients[0] |= 7;
	device_object->current_host_client_id = 3;
	device_object->heci_state = HECI_ENABLED;
	DBG("initialization heci clients successful.\n");
	return ESUCCESS;
}

/**
 * host_start_message - heci host send start message.
 *
 * @device_object - Device object for our driver
 *
 * @return :
 *  0 on success,
 *  negative on failure.
 */
int host_start_message(struct iamt_heci_device * device_object)
{
	long timeout = 60;	/* 60 second */

	struct heci_message_header *heci_header;
	struct hbm_host_version_request *host_start_req;
	struct hbm_host_stop_request *host_stop_req;
	int err = 0;
	/* host start message */
	msleep(100);
	heci_header =
	    (struct heci_message_header *) & device_object->
	    write_message_buffer[0];
	heci_header->host_address = 0;
	heci_header->me_address = 0;
	heci_header->length = sizeof(struct hbm_host_version_request);
	heci_header->message_complete = 1;
	heci_header->reserved = 0;

	host_start_req =
	    (struct hbm_host_version_request *) & device_object->
	    write_message_buffer[1];
	memset(host_start_req, 0, sizeof(host_start_req));
	host_start_req->command.command = HOST_START_REQ_CMD;
	host_start_req->reserved = 0;
	host_start_req->host_version.major_version = HBM_MAJOR_VERSION;
	host_start_req->host_version.minor_version = HBM_MINOR_VERSION;
	device_object->received_message = FALSE;
	if (!heci_write_message(device_object, heci_header,
				       (unsigned char *) (host_start_req),
				       heci_header->length)) {
		device_object->heci_state = HECI_DISABLED;
		DBG("send version to fw fail.\n");
		return -ENODEV;
	}
	DBG("call wait_event_interruptible_timeout  for response message. \n");
	/* wait for response */
	err =
	    wait_event_interruptible_timeout(device_object->
					     wait_received_message,
					     (device_object->
					      received_message),
					     timeout * HZ);
	if (!err && !device_object->received_message) {
		device_object->heci_state = HECI_DISABLED;
		DBG("wait_event_interruptible_timeout failed on host start response message. \n");
		return -ENODEV;
	}
	device_object->received_message = FALSE;
	DBG("wait_event_interruptible_timeout successful on host start response message. \n");
	if ((device_object->version.major_version != HBM_MAJOR_VERSION) ||
	    (device_object->version.minor_version != HBM_MINOR_VERSION)) {
		/* send stop message */
		heci_header->host_address = 0;
		heci_header->me_address = 0;
		heci_header->length = sizeof(struct hbm_host_stop_request);
		heci_header->message_complete = 1;
		heci_header->reserved = 0;

		host_stop_req =
		    (struct hbm_host_stop_request *) & device_object->
		    write_message_buffer[1];

		memset(host_stop_req, 0, sizeof(host_stop_req));
		host_stop_req->command.command = HOST_STOP_REQ_CMD;
		host_stop_req->reason = DRIVER_STOP_REQUEST;
		memset(host_stop_req->reserved, 0,
		       sizeof(host_stop_req->reserved));
		heci_write_message(device_object, heci_header,
				   (unsigned char *) (host_stop_req),
				   heci_header->length);
		DBG("version  mismatch.\n");
		return -ENODEV;
	}

	return ESUCCESS;
}

/**
 * host_enum_clients_message - host send enumeration client request message.
 *
 * @device_object - Device object for our driver
 *
 * @return :
 *  0 on success,
 *  negative on failure.
 */
int host_enum_clients_message(struct iamt_heci_device * device_object)
{
	long timeout = 5;	/*5 second */

	struct heci_message_header *heci_header;
	struct hbm_host_enumeration_request *host_enum_req;
	int err = 0;
	__u8 i, j;
	heci_header =
	    (struct heci_message_header *) & device_object->
	    write_message_buffer[0];
	/* enumerate clients */
	heci_header->host_address = 0;
	heci_header->me_address = 0;
	heci_header->length = sizeof(struct hbm_host_enumeration_request);
	heci_header->message_complete = 1;
	heci_header->reserved = 0;

	host_enum_req =
	    (struct hbm_host_enumeration_request *) & device_object->
	    write_message_buffer[1];
	memset(host_enum_req, 0, sizeof(host_enum_req));
	host_enum_req->command.command = HOST_ENUM_REQ_CMD;
	memset(host_enum_req->reserved, 0,
	       sizeof(host_enum_req->reserved));
	if (!heci_write_message(device_object, heci_header,
			       (unsigned char *) (host_enum_req),
			       heci_header->length)) {
		device_object->heci_state = HECI_DISABLED;
		DBG("send enumeration request fail.\n");
		return -ENODEV;
	}
	/* wait for response */
	device_object->received_message = FALSE;
	err =
	    wait_event_interruptible_timeout(device_object->
					     wait_received_message,
					     (device_object->
					      received_message),
					     timeout * HZ);
	if (!err && !device_object->received_message) {

		device_object->heci_state = HECI_DISABLED;

		DBG("wait_event_interruptible_timeout failed on enumeration cients response message. \n");
		return -ENODEV;
	}
	device_object->received_message = FALSE;
	/* count how many ME clients we have */
	for (i = 0; i < sizeof(device_object->heci_me_clients); i++)
		for (j = 0; j < 8; j++)
			if ((device_object->heci_me_clients[i] & (1 << j)) != 0)
				device_object->num_heci_me_clients++;
	return ESUCCESS;
}

/**
 * allocate_me_clents_storage - allocate storage for me clients
 *
 * @device_object - Device object for our driver
 *
 * @return :
 *  0 on success,
 *  negative on failure.
 */
int allocate_me_clents_storage(struct iamt_heci_device * device_object)
{
	long timeout = 10;	/*10 second */
	struct heci_message_header *heci_header;
	struct hbm_host_client_properties_request *host_cli_req;
	__u8 client_num, i, j;
	int err;
	heci_header =
	    (struct heci_message_header *) & device_object->
	    write_message_buffer[0];
	/* allocate storage for ME clients representation */
	if (device_object->num_heci_me_clients > 0) {
		kfree(device_object->me_clients);
		device_object->me_clients =
			    kcalloc(device_object->num_heci_me_clients,
                        sizeof(struct heci_me_client),
				    GFP_KERNEL);
		if (!device_object->me_clients) {
			device_object->heci_state = HECI_DISABLED;
			DBG("allocate me clents  memory failed.\n");
			return -ENOMEM;
		}

		client_num = 0;

		for (i = 0; i < sizeof(device_object->heci_me_clients);
		     i++) {
			for (j = 0; j < 8; j++) {
				if ((device_object->heci_me_clients[i] & (1 << j)) != 0) {
					device_object->me_clients[client_num].client_id = (i * 8) + j;
					device_object->me_clients[client_num].flow_control_credentials = 0;
					heci_header->host_address = 0;
					heci_header->me_address = 0;
					heci_header->length = sizeof(struct hbm_host_client_properties_request);
					heci_header->message_complete = 1;
					heci_header->reserved = 0;

					host_cli_req = (struct hbm_host_client_properties_request *)& device_object->write_message_buffer[1];
					memset(host_cli_req, 0, sizeof(struct hbm_host_client_properties_request));
                    host_cli_req->command.command = HOST_CLIENT_PROPERTEIS_REQ_CMD;
					host_cli_req->address = device_object->me_clients[client_num].client_id;
					memset(host_cli_req->reserved, 0, sizeof(host_cli_req->reserved));
					if (!heci_write_message(device_object, heci_header,
								(unsigned char *) (host_cli_req), heci_header->length)) {
						DBG("send client properteis request fail.\n");
						device_object->heci_state = HECI_DISABLED;
						kfree(device_object->me_clients);
						return -ENODEV;
					}
					/* wait for response */
					device_object->received_message = FALSE;
					err = wait_event_interruptible_timeout (device_object->wait_received_message,
							(device_object->received_message), timeout * HZ);
					if (!err && !device_object->received_message) {
						DBG("wait_event_interruptible_timeout failed on client properteis response message.\n");
						device_object->heci_state = HECI_DISABLED;
						kfree(device_object->me_clients);
						return -ENODEV;
					}
					device_object->received_message = FALSE;
					client_num++;
				}
			}
		}
	}
	return ESUCCESS;
}

/**
 * host_init_wd - heci initialization wd.
 *
 * @device_object - Device object for our driver
 *
 * @return :
 * none;
 */
void host_init_wd(struct iamt_heci_device * device_object)
{
	long timeout = 15;	/*15 second */
	__u8 i;
	int err = 0;
	/*look for WD client and connect to it */
	spin_lock_init(&device_object->wd_file_extension.file_lock);
	init_waitqueue_head(&device_object->wd_file_extension.wait);
	device_object->wd_file_extension.file = NULL;
	device_object->wd_file_extension.state = HECI_FILE_DISCONNECTED;
	device_object->wd_timeout = 0;
	device_object->asf_mode = FALSE;
	/*find ME ASF client - otherwise assume AMT mode */
	DBG("find ME ASF client - otherwise assume AMT mode.\n");
	for (i = 0; i < device_object->num_heci_me_clients; i++) {
		if (memcmp(&heci_asf_guid,
				&device_object->me_clients[i].properteis.
				protocol_name, sizeof(struct guid))==0) {
			device_object->asf_mode = TRUE;
			DBG("found ME ASF client.\n");
		}
	}
	if (device_object->asf_mode) {
		memcpy(device_object->wd_data,
		       stop_wd_params, HECI_WD_PARAMS_SIZE);

	}
        else {		/* AMT mode */

		DBG("assume AMT mode.\n");
		device_object->wd_timeout = AMT_WD_VALUE;
		DBG("device_object->wd_timeout=%d.\n",
		    device_object->wd_timeout);
		memcpy(device_object->wd_data, start_wd_params, HECI_WD_PARAMS_SIZE);
		memcpy(device_object->wd_data + HECI_WD_PARAMS_SIZE,
		       &device_object->wd_timeout, sizeof(__u16));
	}

	/* find ME WD client */
	for (i = 0; i < device_object->num_heci_me_clients; i++) {
		if (0 == memcmp(&heci_wd_guid, &device_object->me_clients[i].properteis.protocol_name, sizeof(struct guid))) {

			spin_lock_bh(&device_object->device_lock);
			device_object->wd_file_extension.me_client_id =
			    device_object->me_clients[i].client_id;
			device_object->wd_file_extension.state =
			    HECI_FILE_CONNECTING;

			device_object->wd_file_extension.host_client_id =
			    HECI_WD_HOST_CLIENT_ID;

			device_object->wd_file_extension.
			    flow_control_credentials = 0;
			device_object->wd_file_extension.timer_count = 0;
			list_add_tail(&device_object->wd_file_extension.
				      link, &device_object->file_list);
			spin_unlock_bh(&device_object->device_lock);
			break;
		}
	}
	DBG("check wd_file_ext\n");
	if (HECI_FILE_CONNECTING == device_object->wd_file_extension.state) {
		if (heci_connect(device_object,
				 &device_object->wd_file_extension)) {

			err =
			    wait_event_timeout
			    (device_object->wait_received_message,
			     (HECI_FILE_CONNECTED == device_object->wd_file_extension.state ||HECI_FILE_DISCONNECTED == device_object->wd_file_extension.state),
			     timeout * HZ);
			if (HECI_FILE_CONNECTED == device_object->wd_file_extension.state) {
				DBG("device_object->wd_timeout=%d.\n",
				    device_object->wd_timeout);
				if (device_object->wd_timeout != 0)
					device_object->wd_due_counter = 1;
				else
					device_object->wd_due_counter = 0;
				DBG("successfully to connect to WD client.\n");
			} else {

				heci_remove_client_from_file_list
				    (device_object,
				     device_object->wd_file_extension.
				     host_client_id);
				if (HECI_FILE_CONNECTED !=
				    device_object->wd_file_extension.state)
					DBG("wrong status received for WD client.\n");
				if (!err)
					DBG("wait_event_interruptible_timeout failed on client connect message fw response message err=%08x\n", err);
				DBG("failed to connect to WD client.\n");
				device_object->wd_file_extension.state =
				    HECI_FILE_DISCONNECTED;
			}
		} else {
			DBG("failed to call heci_connect for wd_file_extension.\n");
			heci_remove_client_from_file_list(device_object,
							  device_object->
							  wd_file_extension.
							  host_client_id);
			device_object->wd_file_extension.state =
			    HECI_FILE_DISCONNECTED;
		}
	} else {
		DBG("failed to find WD client.\n");
	}

	device_object->wd_timer.function = &heci_wd_timer;
	device_object->wd_timer.data = (unsigned long) device_object;
	return;
}


/**
 * host_init_legacy - heci initialization legacy client.
 *
 * @device_object - Device object for our driver
 *
 * @return :
 * none;
 */
void host_init_legacy(struct iamt_heci_device * device_object)
{
	long timeout = 15;	/*15 second */
	__u8 i;
	int err;


	spin_lock_init(&device_object->legacy_file_extension.file_lock);
	init_waitqueue_head(&device_object->legacy_file_extension.wait);
	spin_lock_init(&device_object->legacy_file_extension.read_io_lock);
	spin_lock_init(&device_object->legacy_file_extension.
		       write_io_lock);
	init_waitqueue_head(&device_object->legacy_file_extension.rx_wait);
	init_waitqueue_head(&device_object->legacy_file_extension.tx_wait);
	device_object->legacy_file_extension.reading_state = HECI_IDLE;
	device_object->legacy_file_extension.writing_state = HECI_IDLE;
	device_object->legacy_file_extension.read_pending = FALSE;
	device_object->legacy_file_extension.flow_control_credentials = 0;
	device_object->legacy_file_extension.read_cb = NULL;
	/* look for legacy client and connect to it */
	device_object->legacy_file_extension.file = NULL;
	device_object->legacy_file_extension.state =
	    HECI_FILE_DISCONNECTED;

	/* find ME PTHI client */
	for (i = 0; i < device_object->num_heci_me_clients; i++) {
		if (0 == memcmp(&heci_pthi_guid, &device_object->me_clients[i].properteis.  protocol_name, sizeof(struct guid))) {
			spin_lock_bh(&device_object->device_lock);
			device_object->legacy_file_extension.me_client_id =
			    device_object->me_clients[i].client_id;
			device_object->legacy_file_extension.state =
			    HECI_FILE_CONNECTING;
			device_object->legacy_file_extension.
			    host_client_id = HECI_LEGACY_HOST_CLIENT_ID;
			device_object->legacy_file_extension.
			    flow_control_credentials = 0;
			device_object->legacy_file_extension.timer_count = 0;
			list_add_tail(&device_object->
				      legacy_file_extension.link,
				      &device_object->file_list);
			spin_unlock_bh(&device_object->device_lock);
			break;
		}
	}
	if (device_object->asf_mode){
		device_object->legacy_file_extension.state =
			HECI_FILE_DISCONNECTED;
		heci_remove_client_from_file_list(device_object,
				device_object->
				legacy_file_extension.
				host_client_id);
		return;

	}
	if (device_object->legacy_file_extension.state == HECI_FILE_CONNECTING) {
		BUG_ON(device_object->me_clients[i].properteis.max_message_length != LEGACY_MTU);

		if (device_object->me_clients[i].properteis.max_message_length < LEGACY_MTU) {
			device_object->legacy_file_extension.state =
			    HECI_FILE_DISCONNECTED;
			DBG("legacy client buffer too small.\n");
		} else {
			if (heci_connect(device_object, &device_object-> legacy_file_extension)) {
				err = wait_event_timeout (device_object->wait_received_message,
					      (device_object->legacy_file_extension.state == HECI_FILE_CONNECTED ||
					      device_object->legacy_file_extension.state == HECI_FILE_DISCONNECTED), timeout * HZ);
				if ((device_object->legacy_file_extension.state != HECI_FILE_CONNECTED)) {
					heci_remove_client_from_file_list
					    (device_object,
					     device_object->
					     legacy_file_extension.
					     host_client_id);
					DBG("failed to connect to legacy client.\n");
					device_object->
					    legacy_file_extension.state =
					    HECI_FILE_DISCONNECTED;
				} else {
					DBG("successfully to connect to legacy client.\n");
					device_object->legacy_state =
					    HECI_LEGACY_IDLE;
				}
			} else {
				DBG("failed to call heci_connect for legacy_file_extension.\n");
				heci_remove_client_from_file_list
				    (device_object,
				     device_object->legacy_file_extension.
				     host_client_id);
				device_object->legacy_file_extension.
				    state = HECI_FILE_DISCONNECTED;
			}
		}
	} else {
		if (!device_object->asf_mode)
			DBG("failed to find legacy client.\n");
	}
	return;
}

/**
 * alloc_priv - allocates a private file structure and set it up.
 * @file: the file structure
 *
 * @return :
 * The allocated file or NULL on failure
 */
struct heci_file_private *alloc_priv(struct file * file)
{
	struct heci_file_private *priv;

	priv = kmalloc(sizeof(struct heci_file_private), GFP_KERNEL);
	if (!priv)
		return NULL;

	spin_lock_init(&priv->file_lock);
	spin_lock_init(&priv->read_io_lock);
	spin_lock_init(&priv->write_io_lock);
	init_waitqueue_head(&priv->wait);
	init_waitqueue_head(&priv->rx_wait);
	DBG("priv->rx_wait =%p\n", &priv->rx_wait);
	init_waitqueue_head(&priv->tx_wait);
	INIT_LIST_HEAD(&priv->link);
	priv->reading_state = HECI_IDLE;
	priv->writing_state = HECI_IDLE;
	priv->file = file;
	priv->flow_control_credentials = 0;
	priv->timer_count = 0;
	priv->me_client_id = 0;
	priv->read_cb = NULL;
	priv->status = ESUCCESS;
	priv->read_pending = FALSE;
	return priv;
}



/**
 * heci_disconnect_host_client  - send disconnect message  to fw from host client.
 *
 * @device_object -Device object for our driver
 * @file_extension -extension of the file object
 *
 * @return :
 *  0 on success,
 *  negative on failure.
 */
int heci_disconnect_host_client(struct iamt_heci_device * device_object,
				struct heci_file_private * file_extension)
{
	int return_status = ESUCCESS, err = 0;
	long timeout = 15;	/*15 second */

	struct heci_cb_private *kernel_priv_cb = NULL;

	struct heci_file_private *file_extension_list_temp = NULL;

       struct heci_cb_private *kernel_priv_cb_pos = NULL, *kernel_priv_cb_next = NULL;

	if ((!device_object) || (!file_extension))
		return -ENODEV;
	kernel_priv_cb = kmalloc(sizeof(struct heci_cb_private), GFP_KERNEL);
	if (!kernel_priv_cb)
		return -ENOMEM;
	if (file_extension->state == HECI_FILE_DISCONNECTING) {
		INIT_LIST_HEAD(&kernel_priv_cb->cb_list);
		kernel_priv_cb->file_private = file_extension;
		kernel_priv_cb->major_file_operations = HECI_CLOSE;
		spin_lock_bh(&device_object->device_lock);
		if (device_object->host_buffer_is_empty){
                    device_object->host_buffer_is_empty =FALSE;
                    if (heci_disconnect(device_object, file_extension)) {
			    list_add_tail(&kernel_priv_cb->cb_list,
				      &device_object->control_read_list.
				      heci_cb.cb_list);
                     }
                     else{
                             spin_unlock_bh(&device_object->device_lock);
			        return_status = -ENODEV;
			        DBG("failed to call heci_disconnect for file_extension.\n");
                             goto free;
                      }
		 }
               else{
                    kernel_priv_cb->file_private = file_extension;
		        DBG("add disconnect cb to control write list\n");
		        list_add_tail(&kernel_priv_cb->cb_list,
			      &device_object->control_write_list.heci_cb.cb_list);
               }
	        spin_unlock_bh(&device_object->device_lock);

		 err =
		    wait_event_timeout
		    (device_object->wait_received_message,
		     (HECI_FILE_DISCONNECTED == file_extension->state),
		     timeout * HZ);
		if (HECI_FILE_DISCONNECTED == file_extension->state) {
			return_status = ESUCCESS;
			DBG("successfully to disconnect from fw client.\n");
		} else {
			return_status = -ENODEV;
			if (HECI_FILE_DISCONNECTED != file_extension->state)
				DBG("wrong status received for client disconnect.\n");
			if (!err)
				DBG("wait_event_interruptible_timeout failed on client disconnect message fw response message err=%08x\n", err);
			DBG("failed to diconnect to fw client.\n");
		}

	}
	if (kernel_priv_cb) {
		spin_lock_bh(&device_object->device_lock);
		if (device_object->control_read_list.status == ESUCCESS
		    && !list_empty(&device_object->control_read_list.heci_cb.cb_list)) {
		    list_for_each_entry_safe(kernel_priv_cb_pos, kernel_priv_cb_next, &device_object->control_read_list.heci_cb.cb_list, cb_list){
				file_extension_list_temp =
				    (struct heci_file_private *)
				    kernel_priv_cb_pos->file_private;
				if (file_extension_list_temp) {
					if ((file_extension->host_client_id == file_extension_list_temp->host_client_id)
						&& (file_extension->me_client_id == file_extension_list_temp->me_client_id)) {
						list_del(&kernel_priv_cb_pos->cb_list);
					}
				}

			}
		}
		spin_unlock_bh(&device_object->device_lock);
free:
              kfree(kernel_priv_cb);
		kernel_priv_cb = NULL;


	}

    return return_status;
}


/**
 * heci_remove_client_from_file_list  - remove file extension from device file list
 *
 * @device_object -Device object for our driver
 * @host_client_id   -host client id to be removed
 *
 * @return :
 * none;
 */
void heci_remove_client_from_file_list(struct iamt_heci_device * device_object,
				       __u8 host_client_id)
{
	struct heci_file_private *file_extension_pos = NULL;
	struct heci_file_private *file_extension_next = NULL;
	list_for_each_entry_safe(file_extension_pos, file_extension_next, &device_object->file_list, link) {
		if (host_client_id == file_extension_pos->host_client_id) {
			DBG("remove  file extension node host client = %d, ME client = %d\n",
					file_extension_pos->host_client_id,
					file_extension_pos->me_client_id);
			list_del(&file_extension_pos->link);
			break;
		}


	}
}
