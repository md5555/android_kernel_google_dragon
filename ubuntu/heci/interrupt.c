/*
 * Part of Intel(R) Manageability Engine Interface Linux driver
 *
 * Copyright (c) 2006-2007 Intel Corp.
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

#include <asm/uaccess.h>
#include <linux/kthread.h>
#include "kcompat.h"
#include "heci.h"
#include "heci_interface.h"

/**
 *  interrupt function prototypes
 */

void heci_bh_handler(struct work_struct *work);
int heci_bh_read_handler(struct io_heci_list *complete_list,
		struct iamt_heci_device * device_object,
		__s32 * slots);
int heci_bh_write_handler(struct io_heci_list *complete_list,
		struct iamt_heci_device * device_object,
		__s32 * slots);
void heci_bh_read_bus_message(struct iamt_heci_device * device_object,
		struct heci_message_header * heci_header);
int heci_bh_read_pthi_message(struct io_heci_list *complete_list,
		struct iamt_heci_device * device_object,
		struct heci_message_header * heci_header);
int heci_bh_read_client_message(struct io_heci_list *complete_list,
		struct iamt_heci_device * device_object,
		struct heci_message_header * heci_header);
void heci_client_connect_response(struct iamt_heci_device * device_object,
		struct hbm_client_connect_response *
		connect_res);
void heci_client_disconnect_response(struct iamt_heci_device * device_object,
		struct hbm_client_connect_response *
		disconnect_res);
void heci_client_flow_control_response(struct iamt_heci_device * device_object,
		struct hbm_flow_control * flow_control);
void heci_client_disconnect_request(struct iamt_heci_device * device_object,
		struct hbm_client_disconnect_request *
		disconnect_req);


/**
 * heci_isr_interrupt - The ISR of the HECI device
 * @irq: The irq number
 * @dev_id: pointer to the device structure
 * @regs: the register values
 *
 * @return :
 * irqreturn_t
 */
irqreturn_t heci_isr_interrupt(int irq, void *dev_id)
{
	int err;
	struct iamt_heci_device *device = (struct iamt_heci_device *) dev_id;
	device->host_hw_state = read_heci_register(device, H_CSR);

	if ((device->host_hw_state & H_IS) != H_IS)
		return IRQ_NONE;

	/* disable interrupts */
	device->host_hw_state &= ~H_IE;
	/* acknowledge interrupt and stop interupts */
	write_heci_register(device, H_CSR, device->host_hw_state);
	/**
	 * Our device interrupted, schedule work the heci_bh_handler
	 * to handle the interrupt processing. This needs to be a
	 * workqueue item since the handler can sleep.
	 */
	PREPARE_WORK(&device->work, heci_bh_handler);
	DBG("schedule work the heci_bh_handler \n");
	err = schedule_work(&device->work);
	if (!err)
		HECI_ERR("schedule work the heci_bh_handler failed error=%x\n",
				err);
	return IRQ_HANDLED;
}

/**
 * heci_bh_handler - function called after ISR to handle the interrupt processing.
 * @data: pointer to the device structure
 *
 * NOTE: This function is called by schedule work
 * @return :
 * none;
 */
void heci_bh_handler(struct work_struct *work)
{
	struct iamt_heci_device *device = container_of(work, struct iamt_heci_device, work);
	struct io_heci_list complete_list;
	__s32 slots;
	int return_status;
	struct heci_cb_private *kernel_priv_cb_pos = NULL, *kernel_priv_cb_next = NULL;
	struct heci_file_private *file_extension = NULL;
	int bus_message_received = FALSE;
	struct task_struct *tsk;

	DBG("function called after ISR to handle the interrupt processing.\n");
	/* initialize our complete list */
	spin_lock_bh(&device->device_lock);
	heci_initialize_list(&complete_list, device);
	device->host_hw_state = read_heci_register(device, H_CSR);
	device->me_hw_state = read_heci_register(device, ME_CSR_HA);

	/* check if ME wants a reset */
	if (((device->me_hw_state & ME_RDY_HRA) == 0) &&
			(device->heci_state != HECI_RESETING)
			&& (device->heci_state != HECI_INITIALIZING)) {
		DBG("FW not ready.\n");
		heci_reset(device, TRUE);
		spin_unlock_bh(&device->device_lock);
		return;
	}

	/*  check if we need to start the device */
	if ((device->host_hw_state & H_RDY) == 0) {

		if ((device->me_hw_state & ME_RDY_HRA) == ME_RDY_HRA) {
			DBG("we need to start the device.\n");
			device->host_hw_state |= (H_IE | H_IG | H_RDY);
			write_heci_register(device, H_CSR,
					device->host_hw_state);
			if (device->heci_state == HECI_INITIALIZING) {

				device->received_message = TRUE;
				spin_unlock_bh(&device->device_lock);
				wake_up_interruptible(&device->
						wait_received_message);
				return;

			} else {
				spin_unlock_bh(&device->device_lock);
				tsk = kthread_run(heci_initialize_clients,
						device, "heci");
				if (IS_ERR(tsk)) {
					int rc = PTR_ERR(tsk);
					printk(KERN_WARNING "heci: "
							"Unable to start the thread for heci: %d\n",
							rc);
				}
				return;
			}


		} else {
			DBG("Enable interrupt FW not ready \n");
			device->host_hw_state |= (H_IE);
			write_heci_register(device, H_CSR,
					device->host_hw_state);
			spin_unlock_bh(&device->device_lock);
			return;
		}
	}
	/* check slots avalable for reading */
	slots = count_full_read_slots(device);
	DBG("slots =%08x  extra_write_index =%08x.\n", slots,
			device->extra_write_index);
	while ((slots > 0) && (!device->extra_write_index)) {
		DBG("slots =%08x  extra_write_index =%08x.\n", slots,
				device->extra_write_index);
		DBG("call heci_bh_read_handler.\n");
		return_status =
			heci_bh_read_handler(&complete_list, device, &slots);
		if (return_status != ESUCCESS)
			goto end;
	}
	return_status =
		heci_bh_write_handler(&complete_list, device, &slots);
end:
	DBG("end of bottom half function.\n");
	device->host_hw_state = read_heci_register(device, H_CSR);
	device->host_buffer_is_empty = host_buffer_is_empty(device);

	if ((device->host_hw_state & H_IS) == H_IS) {
		PREPARE_WORK(&device->work, heci_bh_handler);
		DBG("schedule work the heci_bh_handler.\n");
		return_status = schedule_work(&device->work);
		if (!return_status)
			HECI_ERR("schedule work the heci_bh_handler failed error=%x\n", return_status);
	} else {
		device->host_hw_state |= H_IE;
	}

	write_heci_register(device, H_CSR, device->host_hw_state);


	if (device->received_message
			&& waitqueue_active(&device->wait_received_message)) {
		DBG("received waiting bus message\n");
		bus_message_received = TRUE;
	}
	spin_unlock_bh(&device->device_lock);
	if (bus_message_received) {
		DBG("wake up device->wait_received_message\n");
		wake_up_interruptible(&device->wait_received_message);
		bus_message_received = FALSE;
	}
	if (complete_list.status != ESUCCESS || list_empty(&complete_list.heci_cb.cb_list)){
		return;
	}

	list_for_each_entry_safe(kernel_priv_cb_pos, kernel_priv_cb_next, &complete_list.heci_cb.cb_list, cb_list){

		file_extension =
			(struct heci_file_private *) kernel_priv_cb_pos->
			file_private;
		list_del(&kernel_priv_cb_pos->cb_list);
		if (file_extension && file_extension != &device->legacy_file_extension) {
			DBG("completing call back.\n");
			if (kernel_priv_cb_pos->major_file_operations == HECI_WRITE) {

				kfree(kernel_priv_cb_pos-> request_buffer.data);
				kernel_priv_cb_pos->request_buffer.data = NULL;
				kfree(kernel_priv_cb_pos);
				kernel_priv_cb_pos = NULL;
				DBG("completing write  call back.\n");
				file_extension->writing_state =
					HECI_WRITE_COMPLETE;
				if (&file_extension->tx_wait &&
						waitqueue_active (&file_extension->tx_wait)) {
					wake_up_interruptible
						(&file_extension->tx_wait);
				}
			} else if (kernel_priv_cb_pos->major_file_operations == HECI_READ
					&& HECI_READING == file_extension->reading_state) {
				DBG("completing read call back information= %lu\n",
						kernel_priv_cb_pos->information);
				file_extension->reading_state = HECI_READ_COMPLETE;
				if (&file_extension->rx_wait
						&& waitqueue_active (&file_extension->rx_wait)) {
					wake_up_interruptible
						(&file_extension->rx_wait);
				}

			}
		} else if (file_extension == &device->legacy_file_extension) {
			if (device->legacy_canceled != TRUE) {
				device->legacy_state = HECI_LEGACY_READ_COMPLETE;
				device->legacy_stall_timer = 0;
				memcpy(kernel_priv_cb_pos->response_buffer.
						data,
						device->
						legacy_message_buffer,
						device->
						legacy_message_buffer_index);
				list_add_tail(&kernel_priv_cb_pos->cb_list,
						&device->
						pthi_read_complete_list.
						heci_cb.cb_list);
				DBG("pthi read completed\n");
			} else {
				run_next_legacy_cmd(device);
			}
			if (&device->legacy_file_extension.wait) {
				DBG("completing pthi call back.\n");
				wake_up_interruptible(&device->
						legacy_file_extension.
						wait);

			}

		}
	}
	return;
}


/**
 * heci_bh_read_handler - bottom half read routine after ISR to handle the read processing.
 *
 *
 * @complete_list - An instance of our list structure
 * @device_object - Device object for our driver
 * @slots         - slots to read.
 * @return :
 *  0 on success,
 *  negative on failure.
 */
int heci_bh_read_handler(struct io_heci_list *complete_list,
		struct iamt_heci_device * device_object,
		__s32 * slots)
{
	struct heci_message_header *heci_header;
	int ret = ESUCCESS;
	struct heci_file_private *file_extension_pos = NULL;
	struct heci_file_private *file_extension_next = NULL;
	if (!device_object->read_message_header) {
		device_object->read_message_header =
			read_heci_register(device_object, ME_CB_RW);
		DBG("slots=%08x.\n", *slots);
		(*slots)--;
		DBG("slots=%08x.\n", *slots);
	}
	heci_header =
		(struct heci_message_header *) & device_object->read_message_header;
	DBG("heci_header->length =%d\n", heci_header->length);

	if ((heci_header->reserved)
			|| !(device_object->read_message_header)) {
		DBG("corrupted message header.\n");
		ret = -ECORRUPTED_MESSAGE_HEADER;
		goto end;
	}
	if (heci_header->host_address || heci_header->me_address) {

		list_for_each_entry_safe(file_extension_pos, file_extension_next, &device_object->file_list, link) {
			DBG("list_for_each_entry_safe read host client = %d, ME client = %d\n",
					file_extension_pos->host_client_id,
					file_extension_pos->me_client_id);
			if ((file_extension_pos->host_client_id == heci_header->host_address)
					&& (file_extension_pos->me_client_id == heci_header->me_address))
				break;
		}

		if (&file_extension_pos->link==&device_object->file_list) {
			DBG("corrupted message header\n");
			ret = -ECORRUPTED_MESSAGE_HEADER;
			goto end;

		}
	}
	if (((*slots) * sizeof(__u32)) < heci_header->length) {
		DBG("we can't read the message slots=%08x.\n", *slots);
		/* we can't read the message */
		ret = -ERANGE;
		goto end;
	}


	/* decide where to read the message too */
	if (!heci_header->host_address) {
		DBG("call heci_bh_read_bus_message.\n");
		heci_bh_read_bus_message(device_object, heci_header);
		DBG("end heci_bh_read_bus_message.\n");
	} else if (heci_header->host_address ==
			device_object->legacy_file_extension.host_client_id
			&& HECI_FILE_CONNECTED ==
			device_object->legacy_file_extension.state
			&& device_object->legacy_state == HECI_LEGACY_READING) {
		DBG("call heci_bh_read_legacy_message.\n");
		DBG("heci_header->length =%d\n", heci_header->length);
		ret =
			heci_bh_read_pthi_message(complete_list, device_object,
					heci_header);
		if (ret != ESUCCESS)
			goto end;
	} else {
		DBG("call heci_bh_read_client_message.\n");
		ret = heci_bh_read_client_message(complete_list,
				device_object,
				heci_header);
		if (ret != ESUCCESS)
			goto end;
	}

	/* reset the number of slots and header */
	*slots = count_full_read_slots(device_object);
	device_object->read_message_header = 0;

	if (*slots == -ESLOTS_OVERFLOW) {	/* overflow - reset */
		DBG("reseting due to slots overflow\n");
		/* set the event since message has been read */
		ret = -ERANGE;
		goto end;
	}
end:

	return ret;

}


/**
 * heci_bh_read_bus_message - bottom half read routine after ISR to handle the read bus message
 * command  processing.
 *
 *
 * @complete_list - An instance of our list structure
 * @device_object - Device object for our driver
 * @buffer        - message buffer will be filled
 * @heci_header   - header of bus message
 *
 * @return :
 * none;
 */
void heci_bh_read_bus_message(struct iamt_heci_device * device_object,
		struct heci_message_header * heci_header)
{
	struct heci_bus_message *heci_message;
	struct hbm_host_version_response *version_res;
	struct hbm_client_connect_response *connect_res;
	struct hbm_client_connect_response *disconnect_res;
	struct hbm_flow_control *flow_control;
	struct hbm_host_client_properties_response *properteis_res;
	struct hbm_host_enumeration_response *enum_res;
	struct hbm_client_disconnect_request *disconnect_req;
	struct hbm_host_stop_request *h_stop_req;
	int i;
	unsigned char *buffer;
	buffer = NULL;
	/*  read the message to our buffer */
	buffer = (unsigned char *) device_object->read_message_buffer;
	BUG_ON(heci_header->length >=
			sizeof(device_object->read_message_buffer));
	heci_read_slots(device_object, buffer, heci_header->length);
	heci_message = (struct heci_bus_message *) buffer;

	switch (*(__u8 *) heci_message) {
	case HOST_START_RES_CMD:
		version_res = (struct hbm_host_version_response *) heci_message;
		if (version_res->host_version_supported) {
			device_object->version.major_version =
				HBM_MAJOR_VERSION;
			device_object->version.minor_version =
				HBM_MINOR_VERSION;
		} else {
			device_object->version =
				version_res->me_max_version;
		}
		device_object->received_message = TRUE;
		DBG("host start response message received.\n");
		break;

	case CLIENT_CONNECT_RES_CMD:
		connect_res =
			(struct hbm_client_connect_response *) heci_message;
		heci_client_connect_response(device_object, connect_res);
		DBG("client connect response message received.\n");
		wake_up(&device_object->wait_received_message);
		break;

	case CLIENT_DISCONNECT_RES_CMD:
		disconnect_res =
			(struct hbm_client_connect_response *) heci_message;
		heci_client_disconnect_response(device_object,
				disconnect_res);
		DBG("client disconnect response message received.\n");
		wake_up(&device_object->wait_received_message);
		break;

	case FLOW_CONTROL_CMD:
		flow_control = (struct hbm_flow_control *) heci_message;
		heci_client_flow_control_response(device_object,
				flow_control);
		DBG("client flow control response message received.\n");
		break;
	case HOST_CLIENT_PROPERTEIS_RES_CMD:
		properteis_res =
			(struct hbm_host_client_properties_response *) heci_message;


		if (properteis_res->status != 0) {
			BUG_ON(1);
			break;
		}
		for (i = 0; i < device_object->num_heci_me_clients; i++) {
			if (device_object->me_clients[i].client_id ==
					properteis_res->address) {
				device_object->me_clients[i].properteis =
					properteis_res->client_properties;
				break;
			}

		}
		device_object->received_message = TRUE;
		break;
	case HOST_ENUM_RES_CMD:
		enum_res =
			(struct hbm_host_enumeration_response *) heci_message;
		memcpy(device_object->heci_me_clients,
				enum_res->valid_addresses, 32);
		device_object->received_message = TRUE;
		break;
	case HOST_STOP_RES_CMD:
		device_object->heci_state = HECI_DISABLED;
		DBG("Reseting becase of FW stop response\n");
		heci_reset(device_object, TRUE);
		break;
	case CLIENT_DISCONNECT_REQ_CMD:
		/* search for client */
		disconnect_req =
			(struct hbm_client_disconnect_request *) heci_message;
		heci_client_disconnect_request(device_object,
				disconnect_req);
		break;
	case ME_STOP_REQ_CMD:
		/* prepare stop request */
		heci_header =
			(struct heci_message_header *) & device_object->
			extra_message_buffer[0];
		heci_header->host_address = 0;
		heci_header->me_address = 0;
		heci_header->length = sizeof(struct hbm_host_stop_request);
		heci_header->message_complete = 1;
		heci_header->reserved = 0;
		h_stop_req =
			(struct hbm_host_stop_request *) & device_object->
			extra_message_buffer[1];
		memset(h_stop_req, 0, sizeof(struct hbm_host_stop_request));
		h_stop_req->command.command = HOST_STOP_REQ_CMD;
		h_stop_req->reason = DRIVER_STOP_REQUEST;
		h_stop_req->reserved[0] = 0;
		h_stop_req->reserved[1] = 0;
		device_object->extra_write_index = 2;
		break;

	default:
		BUG_ON(1);
		break;

	}

	return;

}

/**
 * heci_bh_read_legacy_message - bottom half read routine after ISR to handle the read legacy message
 * data  processing.
 *
 *
 * @complete_list - An instance of our list structure
 * @device_object - Device object for our driver
 * @buffer        - message buffer will be filled
 * @heci_header   - header of legacy message
 *
 * @return :
 *  0 on success,
 *  negative on failure.
 */
int heci_bh_read_pthi_message(struct io_heci_list *complete_list,
		struct iamt_heci_device * device_object,
		struct heci_message_header * heci_header)
{
	struct heci_file_private *file_extension = NULL;
	struct heci_cb_private *priv_cb = NULL;
	int return_status = ESUCCESS;
	unsigned char *buffer = NULL;
	BUG_ON(heci_header->me_address !=
			device_object->legacy_file_extension.me_client_id);
	BUG_ON(device_object->legacy_state != HECI_LEGACY_READING);
	buffer =
		(unsigned char *) (device_object->legacy_message_buffer +
				device_object->legacy_message_buffer_index);
	BUG_ON(sizeof(device_object->legacy_message_buffer) <
			(device_object->legacy_message_buffer_index +
			 heci_header->length));
	heci_read_slots(device_object, buffer, heci_header->length);

	device_object->legacy_message_buffer_index += heci_header->length;

	if (heci_header->message_complete) {
		DBG("pthi_message_buffer_index=%d\n", heci_header->length);
		DBG("completed  pthi read.\n ");
		if (!device_object->legacy_current_cb)
			return -ENODEV;
		priv_cb = device_object->legacy_current_cb;
		device_object->legacy_current_cb = NULL;
		file_extension =
			(struct heci_file_private *) priv_cb->file_private;
		if (!file_extension)
			return -ENODEV;
		device_object->legacy_stall_timer = 0;
		priv_cb->information =
			device_object->legacy_message_buffer_index;
		priv_cb->read_time = get_seconds();
		if (device_object->legacy_ioctl
				&& file_extension == &device_object->legacy_file_extension) {
			/* found the legacy cb */
			DBG("complete the pthi read cb.\n ");
			if (&device_object->legacy_file_extension) {
				DBG("add the pthi read cb to complete.\n ");
				list_add_tail(&priv_cb->cb_list,
						&complete_list->heci_cb.
						cb_list);

			}
		}
	}
	return return_status;

}

/**
 * heci_bh_read_client_message - bottom half read routine after ISR to handle the read heci client message
 * data  processing.
 *
 *
 * @complete_list - An instance of our list structure
 * @device_object - Device object for our driver
 * @buffer        - message buffer will be filled
 * @heci_header   - header of heci client message
 *
 * @return :
 *  0 on success,
 *  negative on failure.
 */
int heci_bh_read_client_message(struct io_heci_list *complete_list,
		struct iamt_heci_device * device_object,
		struct heci_message_header * heci_header)
{
	struct heci_file_private *file_extension = NULL;
	struct heci_cb_private *kernel_priv_cb_pos = NULL, *kernel_priv_cb_next = NULL;
	unsigned char *buffer = NULL;
	DBG ("Start client msg \n");
	if (device_object->read_list.status == ESUCCESS
			&& !list_empty(&device_object->read_list.heci_cb.cb_list)) {
		list_for_each_entry_safe(kernel_priv_cb_pos, kernel_priv_cb_next, &device_object->
				read_list.heci_cb.cb_list, cb_list){
			file_extension = (struct heci_file_private *) kernel_priv_cb_pos->file_private;
			if ((file_extension->host_client_id == heci_header->host_address)
					&& (file_extension->me_client_id == heci_header->me_address)
					&& (file_extension->state == HECI_FILE_CONNECTED)
					&& (HECI_READ_COMPLETE != file_extension->reading_state)) {
				spin_lock(&file_extension->read_io_lock);
				file_extension->reading_state =
					HECI_READING;
				buffer = (unsigned char *) (kernel_priv_cb_pos->response_buffer.data +
						kernel_priv_cb_pos->information);
				BUG_ON(kernel_priv_cb_pos->response_buffer.size < heci_header->length +
						kernel_priv_cb_pos->information);

				if (kernel_priv_cb_pos->response_buffer.size < heci_header->length +
						kernel_priv_cb_pos->information) {
					DBG(" message overflow.\n");
					list_del(&kernel_priv_cb_pos->cb_list);
					spin_unlock(&file_extension->
							read_io_lock);
					return -ENOMEM;
				}
				if (buffer)
					heci_read_slots(device_object,
							buffer,
							heci_header->
							length);
				kernel_priv_cb_pos->information +=
					heci_header->length;
				if (heci_header->message_complete) {
					file_extension->status = ESUCCESS;
					list_del(&kernel_priv_cb_pos->cb_list);
					spin_unlock(&file_extension->
							read_io_lock);
					DBG("completed read host client = %d, ME client = %d, data length = %lu\n",
							file_extension->host_client_id,
							file_extension->me_client_id,
							kernel_priv_cb_pos->information);
					*(kernel_priv_cb_pos->response_buffer.data + kernel_priv_cb_pos->information) = '\0';
					DBG("kernel_priv_cb_pos->response_buffer - %s\n",
							kernel_priv_cb_pos->response_buffer.data);
					list_add_tail(&kernel_priv_cb_pos->cb_list,
							&complete_list->heci_cb.cb_list);
				} else {
					spin_unlock(&file_extension->
							read_io_lock);
				}

				break;
			}

		}

	}
	DBG ("Message read\n");
	if (!buffer) {
		heci_read_slots(device_object,
				(unsigned char *) device_object->
				read_message_buffer, heci_header->length);
		DBG("discarding message, header=%08x.\n",
				*(__u32 *) device_object->read_message_buffer);
	}

	return ESUCCESS;
}


/**
 * heci_bh_write_handler - bottom half write routine after ISR to handle the write processing.
 *
 *
 * @complete_list - An instance of our list structure
 * @device_object - Device object for our driver
 * @slots         - slots to write.
 * @return :
 *  0 on success,
 *  negative on failure.
 */
int heci_bh_write_handler(struct io_heci_list *complete_list,
		struct iamt_heci_device * device_object,
		__s32 * slots)
{

	struct heci_message_header *heci_header = NULL;
	struct heci_file_private *file_extension = NULL;
	struct heci_cb_private *kernel_priv_cb_pos = NULL, *kernel_priv_cb_next = NULL;


	if (!host_buffer_is_empty(device_object)) {
		DBG("host buffer is not empty.\n");
		return ESUCCESS;
	}
	device_object->write_hang = -1;
	*slots = count_empty_write_slots(device_object);
	/* complete all waiting for write CB */
	DBG("complete all waiting for write CB.\n");
	if (device_object->write_waiting_list.status == ESUCCESS
			&& !list_empty(&device_object->write_waiting_list.heci_cb.cb_list)) {
		list_for_each_entry_safe(kernel_priv_cb_pos, kernel_priv_cb_next, &device_object->
				write_waiting_list.heci_cb.cb_list, cb_list){
			file_extension = (struct heci_file_private *) kernel_priv_cb_pos->file_private;
			file_extension->status = ESUCCESS;
			list_del(&kernel_priv_cb_pos->cb_list);
			if (HECI_WRITING == file_extension->writing_state
					&& kernel_priv_cb_pos->major_file_operations == HECI_WRITING
					&& file_extension != &device_object->legacy_file_extension) {
				DBG("HECI WRITE COMPLETE\n");
				file_extension->writing_state = HECI_WRITE_COMPLETE;
				list_add_tail(&kernel_priv_cb_pos->cb_list,
						&complete_list->heci_cb.
						cb_list);
			}
			if (file_extension == &device_object->legacy_file_extension) {
				DBG("check legacy flow control\n");
				if (device_object-> legacy_flow_control_pending) {
					if (((*slots) * sizeof(__u32)) >= (sizeof(struct heci_message_header)
								+ sizeof(struct hbm_flow_control))) {
						*slots -= (sizeof(struct heci_message_header) +
								sizeof(struct hbm_flow_control) + 3) / 4;
						if (!heci_send_flow_control (device_object,
									&device_object->legacy_file_extension)) {
							DBG("legacy flow control failed\n");
						} else {
							DBG("legacy flow control success\n");
							device_object->legacy_state = HECI_LEGACY_READING;
							device_object->legacy_flow_control_pending = FALSE;
							device_object->legacy_message_buffer_index = 0;
							device_object->legacy_message_buffer_size = 0;
							device_object->legacy_stall_timer = LEGACY_STALL_TIMER;
							device_object->host_buffer_is_empty =
								host_buffer_is_empty(device_object);
						}
					} else {
						return -ECOMPLETE_MESSAGE;
					}
				}
			}

		}
	}

	if (device_object->stop && !device_object->wd_pending) {
		device_object->wd_stoped = TRUE;
		wake_up_interruptible(&device_object->wait_stop_wd);
		return ESUCCESS;
	}

	if (device_object->extra_write_index != 0) {
		DBG("extra_write_index =%d\n",
				device_object->extra_write_index);
		heci_write_message(device_object,
				(struct heci_message_header *) &
				device_object->extra_message_buffer[0],
				(unsigned char *) &device_object->
				extra_message_buffer[1],
				(device_object->extra_write_index -
				 1) * sizeof(__u32));
		*slots -= device_object->extra_write_index;
		device_object->extra_write_index = 0;
	}
	if (device_object->heci_state == HECI_ENABLED){
		if (device_object->wd_pending
				&& flow_control_credentials(device_object,
					&device_object->wd_file_extension)) {
			if (!heci_send_wd(device_object))
				DBG("Wd send failed\n");
			else
				flow_control_reduce(device_object,
						&device_object->
						wd_file_extension);
			device_object->wd_pending = 0;

			if (device_object->wd_timeout != 0) {
				*slots -=
					(sizeof(struct heci_message_header) +
					 HECI_START_WD_DATA_SIZE + 3) / 4;
				device_object->wd_due_counter = 2;
			} else {
				*slots -=
					(sizeof(struct heci_message_header) +
					 HECI_WD_PARAMS_SIZE + 3) / 4;
				device_object->wd_due_counter = 0;
			}

		}
	}
	if (device_object->stop)
		return ~ENODEV;

	/* complete control write list CB */
	if (device_object->control_write_list.status == ESUCCESS) {
		/* complete control write list CB */
		DBG("complete control write list CB\n ");
		list_for_each_entry_safe(kernel_priv_cb_pos, kernel_priv_cb_next, &device_object->
				control_write_list.heci_cb.cb_list, cb_list){
			file_extension =
				(struct heci_file_private *) kernel_priv_cb_pos->
				file_private;
			if (!file_extension) {
				list_del(&kernel_priv_cb_pos->cb_list);
				return -ENODEV;
			}
			switch (kernel_priv_cb_pos->major_file_operations) {
			case HECI_CLOSE:
				/* send disconnect message */
				if ((*slots * sizeof(__u32)) >= (sizeof(struct heci_message_header) +
							sizeof(struct hbm_client_disconnect_request))) {
					*slots -= (sizeof(struct heci_message_header) +
							sizeof(struct hbm_client_disconnect_request) + 3) / 4;
					if (!heci_disconnect(device_object,
								file_extension)) {
						file_extension->status = ESUCCESS;
						kernel_priv_cb_pos->information = 0;
						list_del(&kernel_priv_cb_pos->cb_list);
						list_add_tail(&kernel_priv_cb_pos->cb_list,
								&complete_list->heci_cb.
								cb_list);
						return -ECOMPLETE_MESSAGE;
					} else {
						file_extension->state = HECI_FILE_DISCONNECTING;
						file_extension->status = ESUCCESS;
						kernel_priv_cb_pos->information = 0;
						list_del(&kernel_priv_cb_pos->cb_list);
						list_add_tail(&kernel_priv_cb_pos->cb_list,
								&device_object->control_read_list.
								heci_cb.cb_list);
						file_extension->timer_count = CONNECT_TIMEOUT;
					}
				} else {
					/* return the cancel routine */
					return -ECORRUPTED_MESSAGE_HEADER;
				}
				break;
			case HECI_READ:
				/* send flow control message */
				if ((*slots * sizeof(__u32)) >= (sizeof(struct heci_message_header) +
							sizeof(struct hbm_flow_control))) {
					*slots -= (sizeof(struct heci_message_header) +
							sizeof(struct hbm_flow_control) + 3) / 4;
					if (!heci_send_flow_control(device_object, file_extension)) {
						file_extension->status = -ENODEV;
						kernel_priv_cb_pos->information = 0;
						list_del(&kernel_priv_cb_pos->cb_list);
						list_add_tail(&kernel_priv_cb_pos->cb_list,
								&complete_list->heci_cb.
								cb_list);
						return -ENODEV;

					} else {
						list_del(&kernel_priv_cb_pos->cb_list);
						list_add_tail(&kernel_priv_cb_pos->cb_list,
								&device_object->read_list.
								heci_cb.cb_list);
					}
				} else {
					/* return the cancel routine */
					list_del(&kernel_priv_cb_pos->cb_list);
					return -ECORRUPTED_MESSAGE_HEADER;
				}
				break;
			case HECI_IOCTL:
				/* connect message */
				if (!other_client_is_connecting(device_object, file_extension)) {
					continue;
				}
				if ((*slots * sizeof(__u32)) >= (sizeof(struct heci_message_header) +
							sizeof(struct hbm_client_connect_request))) {
					file_extension->state = HECI_FILE_CONNECTING;
					*slots -= (sizeof(struct heci_message_header) +
							sizeof(struct hbm_client_connect_request) + 3) / 4;
					if (!heci_connect(device_object, file_extension)) {
						file_extension->status = -ENODEV;
						kernel_priv_cb_pos->information = 0;
						list_del(&kernel_priv_cb_pos->cb_list);
						return -ENODEV;
					} else {
						list_del(&kernel_priv_cb_pos->cb_list);
						list_add_tail(&kernel_priv_cb_pos->cb_list,
								&device_object->control_read_list.
								heci_cb.cb_list);
						file_extension->timer_count = CONNECT_TIMEOUT;
					}
				} else {
					/* return the cancel routine */
					list_del(&kernel_priv_cb_pos->cb_list);
					return -ECORRUPTED_MESSAGE_HEADER;
				}
				break;

			default:
				BUG_ON(1);
			}

		}
	}
	/* complete  write list CB */
	if (device_object->write_list.status == ESUCCESS
			&& !list_empty(&device_object->write_list.heci_cb.cb_list)) {
		DBG("complete write list CB \n");
		list_for_each_entry_safe(kernel_priv_cb_pos, kernel_priv_cb_next, &device_object->
				write_list.heci_cb.cb_list, cb_list){
			file_extension = (struct heci_file_private *) kernel_priv_cb_pos->file_private;

			if ((file_extension != NULL)
					&& (file_extension != &device_object->legacy_file_extension)) {
				if (!flow_control_credentials(device_object,
							file_extension)) {
					DBG("No flow control credentials for client %d, not sending\n",
							file_extension->host_client_id);
					continue;
				}
				if ((*slots * sizeof(__u32)) >= (sizeof(struct heci_message_header) +
							(kernel_priv_cb_pos->request_buffer.size - kernel_priv_cb_pos->information))) {
					heci_header = (struct heci_message_header *) & device_object->write_message_buffer[0];
					heci_header->host_address = file_extension->host_client_id;
					heci_header->me_address = file_extension->me_client_id;
					heci_header->length = ((kernel_priv_cb_pos->request_buffer.size) -
							(kernel_priv_cb_pos->information));
					heci_header->message_complete = 1;
					heci_header->reserved = 0;
					DBG("kernel_priv_cb_pos->request_buffer.size =%d  heci_header->message_complete= %d\n",
							kernel_priv_cb_pos->request_buffer.size,
							heci_header->message_complete);
					DBG("kernel_priv_cb_pos->information  =%lu\n",
							kernel_priv_cb_pos->information);
					DBG("heci_header->length  =%d\n",
							heci_header->length);
					*slots -= (sizeof(struct heci_message_header) + heci_header->length + 3) / 4;
					if (!heci_write_message (device_object, heci_header,
								(unsigned char *) (kernel_priv_cb_pos-> request_buffer.data + kernel_priv_cb_pos->information),
								heci_header->length)) {
						file_extension->status = -ENODEV;
						list_del(&kernel_priv_cb_pos->cb_list);
						list_add_tail(&kernel_priv_cb_pos->cb_list,
								&complete_list->heci_cb.
								cb_list);
						return -ENODEV;
					} else {
						flow_control_reduce
							(device_object, file_extension);
						file_extension->status = ESUCCESS;
						kernel_priv_cb_pos->information += heci_header->length;
						list_del(&kernel_priv_cb_pos->cb_list);
						list_add_tail(&kernel_priv_cb_pos->cb_list,
								&device_object->write_waiting_list.
								heci_cb.cb_list);
					}
				} else if (*slots == ((device_object->host_hw_state & H_CBD) >> 24)) {	/* buffer is still empty */
					heci_header = (struct heci_message_header *) & device_object->write_message_buffer[0];
					heci_header->host_address = file_extension->host_client_id;
					heci_header->me_address = file_extension->me_client_id;
					heci_header->length = (*slots * sizeof(__u32)) - sizeof(struct heci_message_header);
					heci_header->message_complete = 0;
					heci_header->reserved = 0;

					(*slots) -= (sizeof(struct heci_message_header) + heci_header->length + 3) / 4;
					if (!heci_write_message(device_object, heci_header,
								(unsigned char *) (kernel_priv_cb_pos->request_buffer.  data + kernel_priv_cb_pos->information),
								heci_header->length)) {
						file_extension->status = -ENODEV;
						list_del(&kernel_priv_cb_pos->cb_list);
						list_add_tail(&kernel_priv_cb_pos->cb_list, &complete_list->heci_cb.cb_list);
						return -ENODEV;
					} else {
						kernel_priv_cb_pos->information += heci_header->length;
						DBG("kernel_priv_cb_pos->request_buffer.size =%d  heci_header->message_complete= %d\n",
								kernel_priv_cb_pos->request_buffer.size,
								heci_header->message_complete);
						DBG("kernel_priv_cb_pos->information  =%lu\n", kernel_priv_cb_pos->information);
						DBG("heci_header->length  =%d\n", heci_header->length);
					}
					return -ECOMPLETE_MESSAGE;
				} else {
					return -ECORRUPTED_MESSAGE_HEADER;
				}
			} else if (file_extension == &device_object->legacy_file_extension) {	/* LEGACY IOCTL */
				DBG("complete pthi write cb\n");
				if (!flow_control_credentials(device_object, file_extension)) {
					DBG("No flow control credentials for pthi client %d, not sending\n",
							file_extension->host_client_id);
					continue;
				}
				if ((*slots * sizeof(__u32)) >= (sizeof(struct heci_message_header) +
							device_object->legacy_message_buffer_size - device_object->legacy_message_buffer_index)) {
					heci_header = (struct heci_message_header *) & device_object->write_message_buffer[0];
					heci_header->host_address = file_extension->host_client_id;
					heci_header->me_address = file_extension->me_client_id;
					heci_header->length = device_object->legacy_message_buffer_size -
						device_object->legacy_message_buffer_index;
					heci_header->message_complete = 1;
					heci_header->reserved = 0;

					*slots -= (sizeof(struct heci_message_header) + heci_header->length + 3) / 4;

					if (!heci_write_message (device_object, heci_header,
								(device_object->legacy_message_buffer + device_object->legacy_message_buffer_index),
								heci_header->length)) {
						device_object->legacy_state = HECI_LEGACY_IDLE;
						file_extension->status = -ENODEV;
						list_del(&kernel_priv_cb_pos->cb_list);
						return -ENODEV;
					} else {
						flow_control_reduce (device_object, file_extension);
						device_object->legacy_message_buffer_index += heci_header->length;
						list_del(&kernel_priv_cb_pos->cb_list);
						kernel_priv_cb_pos->information = device_object->legacy_message_buffer_index;
						file_extension->status = ESUCCESS;
						device_object->legacy_state = HECI_LEGACY_FLOW_CONTROL;
						device_object->legacy_flow_control_pending = TRUE;
						/* save legacy cb sent to pthi client */
						device_object->legacy_current_cb = kernel_priv_cb_pos;
						list_add_tail(&kernel_priv_cb_pos->cb_list,
								&device_object->write_waiting_list.heci_cb.  cb_list);

					}
				} else if (*slots == ((device_object->host_hw_state & H_CBD) >> 24)) {	/* buffer is still empty */
					heci_header = (struct heci_message_header *) & device_object->write_message_buffer[0];
					heci_header->host_address = file_extension->host_client_id;
					heci_header->me_address = file_extension->me_client_id;
					heci_header->length = (*slots * sizeof(__u32)) - sizeof(struct heci_message_header);
					heci_header->message_complete = 0;
					heci_header->reserved = 0;

					*slots -= (sizeof(struct heci_message_header) + heci_header->length + 3) / 4;

					if (!heci_write_message (device_object, heci_header, (device_object->legacy_message_buffer +
									device_object->legacy_message_buffer_index), heci_header->length)) {
						file_extension->status = -ENODEV;
						list_del(&kernel_priv_cb_pos->cb_list);
					} else {
						device_object->legacy_message_buffer_index += heci_header->length;
					}
					return -ECOMPLETE_MESSAGE;
				} else {
					return -ECORRUPTED_MESSAGE_HEADER;;
				}
			}

		}

	}
	return ESUCCESS;
}


/**
 * is_treat_specially_client  - check if the message belong
 * to the file extension .
 * @file_extension -extension of the file object
 * @connect_res    -connect response bus message
 * @device_object -Device object for our driver
 *
 * @return :
 * TRUE if empty
 * FALSE - otherwise.
 */
int is_treat_specially_client(struct heci_file_private * file_extension,
		struct hbm_client_connect_response *
		connect_res)
{
	int ret = FALSE;
	if ((file_extension->host_client_id == connect_res->host_address)
			&& (file_extension->me_client_id == connect_res->me_address)) {

		if (connect_res->status == 0) {
			DBG("client connect status = 0x%08x.\n",
					connect_res->status);
			file_extension->state = HECI_FILE_CONNECTED;
			file_extension->status = ESUCCESS;
		} else {
			DBG("client connect status = 0x%08x.\n",
					connect_res->status);
			file_extension->state = HECI_FILE_DISCONNECTED;
			file_extension->status = -ENODEV;
		}
		ret = TRUE;
	}
	DBG("client state = %d.\n", file_extension->state);
	return ret;

}

/**
 * heci_client_connect_response  - connect response bh routine
 *
 * @device_object -Device object for our driver
 * @connect_res    -connect response bus message
 * @complete_list - An instance of our list structure
 *
 * @return :
 * none;
 */
void heci_client_connect_response(struct iamt_heci_device * device_object,
		struct hbm_client_connect_response *
		connect_res)
{

	struct heci_file_private *file_extension = NULL;
	struct heci_cb_private *kernel_priv_cb_pos = NULL, *kernel_priv_cb_next = NULL;

	/* if WD or legacy client treat specially */

	if ((is_treat_specially_client(&(device_object->wd_file_extension),
					connect_res))
			|| (is_treat_specially_client(&
					(device_object->
					 legacy_file_extension),
					connect_res))) {
		return;
	}

	if (device_object->control_read_list.status == ESUCCESS
			&& !list_empty(&device_object->control_read_list.heci_cb.
				cb_list)) {
		list_for_each_entry_safe(kernel_priv_cb_pos, kernel_priv_cb_next, &device_object->control_read_list.heci_cb.cb_list, cb_list){
			file_extension =
				(struct heci_file_private *) kernel_priv_cb_pos->
				file_private;
			if (!file_extension) {
				list_del(&kernel_priv_cb_pos->cb_list);
				return;
			}
			if (HECI_IOCTL == kernel_priv_cb_pos->major_file_operations) {

				if (is_treat_specially_client (file_extension, connect_res)) {
					list_del(&kernel_priv_cb_pos->cb_list);
					file_extension->status = ESUCCESS;
					file_extension->timer_count = 0;
					break;
				}
			}
		}
	}
	return;
}

/**
 * heci_client_disconnect_response  - disconnect response bh routine
 *
 * @device_object -Device object for our driver
 * @disconnect_res    -disconnect response bus message
 * @complete_list - An instance of our list structure
 *
 * @return :
 * none;
 */
void heci_client_disconnect_response(struct iamt_heci_device * device_object,
		struct hbm_client_connect_response *
		disconnect_res)
{
	struct heci_file_private *file_extension = NULL;
	struct heci_cb_private *kernel_priv_cb_pos = NULL, *kernel_priv_cb_next = NULL;
	if (device_object->control_read_list.status == ESUCCESS
			&& !list_empty(&device_object->control_read_list.heci_cb.
				cb_list)) {
		list_for_each_entry_safe(kernel_priv_cb_pos, kernel_priv_cb_next, &device_object->control_read_list.heci_cb.cb_list, cb_list){
			file_extension =
				(struct heci_file_private *) kernel_priv_cb_pos->
				file_private;

			if (!file_extension) {
				list_del(&kernel_priv_cb_pos->cb_list);
				return;
			}

			DBG("list_for_each_entry_safe in control_read_list.\n");
			if ((file_extension->host_client_id == disconnect_res->host_address)
					&& (file_extension->me_client_id == disconnect_res->me_address)) {
				list_del(&kernel_priv_cb_pos->cb_list);
				if (disconnect_res->status == 0)
					file_extension->state = HECI_FILE_DISCONNECTED;
				file_extension->status = ESUCCESS;
				file_extension->timer_count = 0;
				break;
			}
		}
	}
	return;
}

/**
 * heci_client_flow_control_response  - flow control response bh routine
 *
 * @device_object -Device object for our driver
 * @flow_control    -flow control response bus message
 *
 * @return :
 * none;
 */
void heci_client_flow_control_response(struct iamt_heci_device * device_object,
		struct hbm_flow_control * flow_control)
{
	struct heci_file_private *file_extension_pos = NULL;
	struct heci_file_private *file_extension_next = NULL;
	int i;
	if (flow_control->host_address == 0) {	/* single receive buffer */
		for (i = 0; i < device_object->num_heci_me_clients; i++) {
			if (flow_control->me_address ==
					device_object->me_clients[i].client_id) {
				if (device_object->me_clients[i].properteis.single_receive_buffer != 0) {
					device_object->me_clients[i].
						flow_control_credentials++;
					DBG("received flow control message for ME client %d (single receive buffer).\n",
							flow_control->me_address);
					DBG("flow control credentials=%d.\n",
							device_object->me_clients[i].flow_control_credentials);
				} else {
					BUG_ON(1);	/* error in flow control */
				}
			}
		}
	} else {		/* normal connection */
		list_for_each_entry_safe(file_extension_pos, file_extension_next, &device_object->file_list, link) {
			DBG("list_for_each_entry_safe in file_list\n");

			DBG("file_extension  of host client %d ME client %d.\n",
					file_extension_pos->host_client_id,
					file_extension_pos->me_client_id);
			DBG("flow control message for host client %d ME client %d.\n",
					flow_control->host_address,
					flow_control->me_address);
			if ((file_extension_pos->host_client_id == flow_control->host_address)
					&& (file_extension_pos->me_client_id == flow_control->me_address)) {
				DBG("received flow control message for host client %d ME client %d.\n",
						flow_control->host_address,
						flow_control->me_address);
				file_extension_pos->flow_control_credentials++;
				DBG("flow control credentials=%d.\n",
						file_extension_pos->flow_control_credentials);
				break;
			}
		}
	}
	return;
}

/**
 * heci_client_disconnect_request  - disconnect request bh routine
 *
 * @device_object -Device object for our driver
 * @disconnect_req    -disconnect request bus message
 *
 * @return :
 * none;
 */
void heci_client_disconnect_request(struct iamt_heci_device * device_object,
		struct hbm_client_disconnect_request *
		disconnect_req)
{
	struct heci_message_header *heci_header;
	struct hbm_client_connect_response *disconnect_res;
	struct heci_file_private *file_extension_pos = NULL;
	struct heci_file_private *file_extension_next = NULL;

	list_for_each_entry_safe(file_extension_pos, file_extension_next, &device_object->file_list, link) {
		if ((file_extension_pos->host_client_id == disconnect_req->host_address)
				&& (file_extension_pos->me_client_id == disconnect_req->me_address)) {
			DBG("received disconnect request for host client %d ME client %d.\n",
					disconnect_req->host_address,
					disconnect_req->me_address);
			file_extension_pos->state = HECI_FILE_DISCONNECTED;
			file_extension_pos->timer_count = 0;
			if (file_extension_pos == &device_object->wd_file_extension) {
				device_object->wd_due_counter = 0;
				device_object->wd_pending = FALSE;
			} else if (file_extension_pos ==
					&device_object->legacy_file_extension) {
				device_object->legacy_timer = 0;
			}

			/* prepare disconnect response */
			heci_header = (struct heci_message_header *) & device_object->extra_message_buffer[0];
			heci_header->host_address = 0;
			heci_header->me_address = 0;
			heci_header->length = sizeof(struct hbm_client_connect_response);
			heci_header->message_complete = 1;
			heci_header->reserved = 0;

			disconnect_res = (struct hbm_client_connect_response *) &
				device_object->extra_message_buffer[1];
			disconnect_res->host_address =
				file_extension_pos->host_client_id;
			disconnect_res->me_address = file_extension_pos->me_client_id;
			*(__u8 *) (&disconnect_res->command) = CLIENT_DISCONNECT_RES_CMD;
			disconnect_res->status = 0;
			device_object->extra_write_index = 2;
			break;
		}
	}
	return;
}


/**
 * heci_timer - timer function .
 * @data: pointer to the device structure
 *
 * NOTE: This function is called by timer interrupt work
 * @return :
 * none;
 */
void heci_wd_timer(unsigned long data)
{
	struct iamt_heci_device *device = (struct iamt_heci_device *) data;
	DBG("send watchdog.\n");
	spin_lock_bh(&device->device_lock);
	if (device->heci_state != HECI_ENABLED) {
		mod_timer(&device->wd_timer, round_jiffies(jiffies + 2 * HZ));
		spin_unlock_bh(&device->device_lock);
		return;
	}
	if (device->wd_file_extension.state != HECI_FILE_CONNECTED) {
		mod_timer(&device->wd_timer, round_jiffies(jiffies + 2 * HZ));
		spin_unlock_bh(&device->device_lock);
		return;
	}
	/*** Watchdog ***/
	if (device->wd_due_counter != 0 && FALSE == device->wd_bypass) {
		if (--device->wd_due_counter == 0) {
			if (device->host_buffer_is_empty &&
					flow_control_credentials(device,
						&device->
						wd_file_extension)) {
				device->host_buffer_is_empty = FALSE;

				if (!heci_send_wd(device))
					DBG("Wd send failed\n");
				else
					flow_control_reduce(device,
							&device->
							wd_file_extension);
				if (device->wd_timeout != 0)
					device->wd_due_counter = 2;
				else
					device->wd_due_counter = 0;
			} else {
				device->wd_pending = TRUE;
			}
		}
	}
	if (device->legacy_stall_timer != 0) {
		if (--device->legacy_stall_timer == 0) {
			DBG("Reseting because of hang to PTHI\n");
			heci_reset(device, TRUE);
			device->legacy_message_buffer_size = 0;
			device->legacy_message_buffer_index = 0;
			device->legacy_canceled = FALSE;
			device->legacy_ioctl = TRUE;
			device->legacy_state = HECI_LEGACY_IDLE;
			device->legacy_timer = 0;
			spin_unlock_bh(&device->device_lock);

			if (device->legacy_current_cb) {
				kfree(device->legacy_current_cb->request_buffer.data);
				device->legacy_current_cb->request_buffer.data = NULL;
				kfree(device->legacy_current_cb->response_buffer.data);
				device->legacy_current_cb->response_buffer.data = NULL;
				kfree(device->legacy_current_cb);
			}
			spin_lock_bh(&device->device_lock);
			device->legacy_file_object = NULL;
			device->legacy_current_cb = NULL;
			run_next_legacy_cmd(device);
		}
	}
	mod_timer(&device->wd_timer, round_jiffies(jiffies + 2 * HZ));
	spin_unlock_bh(&device->device_lock);
	return;
}
