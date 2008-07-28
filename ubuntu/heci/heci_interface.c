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


#include "heci.h"
#include "heci_interface.h"



static const __u8 interface_start_wd_params[] =
    { 0x02, 0x12, 0x13, 0x10 };
static const __u8 interface_stop_wd_params[] =
    { 0x02, 0x02, 0x14, 0x10 };

/**
 * read_heci_register - Read a byte from the heci device
 * @device: the device structure
 * @offset: offset from which to read the data
 *
 * Return:
 * the byte read.
 */
__u32 read_heci_register(struct iamt_heci_device * device,
			    unsigned long offset)
{
    return readl(device->mem_addr + offset);
}

/**
 * write_heci_register - Write  4 bytes to the heci device
 * @device: the device structure
 * @offset: offset from which to write the data
 *
 * @value: the byte to write
 */
void write_heci_register(struct iamt_heci_device * device, unsigned long offset,
			 __u32 value)
{
    writel(value, device->mem_addr + offset);
}

/**
 * host_buffer_is_empty  - check if host buffer is empty.
 *
 * @device_object -Device object for our driver
 *
 * @return :
 * TRUE if empty
 * FALSE - otherwise.
 */
int host_buffer_is_empty(struct iamt_heci_device * device_object)
{
	char read_ptr, write_ptr;
	unsigned char buffer_depth, filled_slots, empty_slots;
	device_object->host_hw_state =
	    read_heci_register(device_object, H_CSR);
	read_ptr = (char) ((device_object->host_hw_state & H_CBRP) >> 8);
	write_ptr = (char) ((device_object->host_hw_state & H_CBWP) >> 16);
	buffer_depth =
	    (unsigned char) ((device_object->host_hw_state & H_CBD) >> 24);
	filled_slots = (unsigned char) (write_ptr - read_ptr);
	empty_slots = buffer_depth - filled_slots;

	if (filled_slots > 0)
		return FALSE;
	return TRUE;
}

/**
 * count_empty_write_slots  - count write empty slots.
 *
 * @device_object - Device object for our driver
 *
 *
 * @return :
 *  -1(ESLOTS_OVERFLOW) if overflow
 *  otherwise filed slots count
 */
__s32 count_empty_write_slots(struct iamt_heci_device * device_object)
{
	char read_ptr, write_ptr;
	unsigned char buffer_depth, filled_slots, empty_slots;

	read_ptr = (char) ((device_object->host_hw_state & H_CBRP) >> 8);
	write_ptr = (char) ((device_object->host_hw_state & H_CBWP) >> 16);
	buffer_depth =
	    (unsigned char) ((device_object->host_hw_state & H_CBD) >> 24);
	filled_slots = (unsigned char) (write_ptr - read_ptr);
	empty_slots = buffer_depth - filled_slots;

	if (filled_slots > buffer_depth)
		/* overfolw */
		return -ESLOTS_OVERFLOW;

	return (__s32) empty_slots;
}

/**
 * heci_write_message  - write a message to heci device.
 *
 * @heci_header          - header of  message
 * @write_buffer         - message buffer will be write
 * @write_length         - message size will be write
 *
 * @return :
 * TRUE if success
 * FALSE - otherwise.
 */
int heci_write_message(struct iamt_heci_device * device_object,
			     struct heci_message_header * header,
			     unsigned char *write_buffer,
			     unsigned long write_length)
{
	__u32 temp_msg = 0;
	unsigned long bytes_written = 0;
	char read_ptr, write_ptr;
	unsigned char buffer_depth, filled_slots, empty_slots;
	unsigned long dw_to_write;
	dw_to_write = ((write_length + 3) / 4);
	DBG("host_hw_state = 0x%08x.\n", device_object->host_hw_state);
	DBG("heci_write_message header=%08x.\n", *((__u32 *) header));

	read_ptr = (char) ((device_object->host_hw_state & H_CBRP) >> 8);
	write_ptr = (char) ((device_object->host_hw_state & H_CBWP) >> 16);
	buffer_depth =
	    (unsigned char) ((device_object->host_hw_state & H_CBD) >> 24);
	filled_slots = (unsigned char) (write_ptr - read_ptr);
	empty_slots = buffer_depth - filled_slots;
	DBG("filled = %hu, empty = %hu.\n", filled_slots, empty_slots);

	if (dw_to_write > empty_slots) {
		return FALSE;
	}

	write_heci_register(device_object, H_CB_WW,
			    *((__u32 *) header));

	while (write_length >= 4) {
		write_heci_register(device_object, H_CB_WW,
				    *(__u32 *) (write_buffer +
						   bytes_written));
		bytes_written += 4;
		write_length -= 4;
	}

	if (write_length > 0) {
		memcpy(&temp_msg, &write_buffer[bytes_written], write_length);
		write_heci_register(device_object, H_CB_WW, temp_msg);
	}

	device_object->host_hw_state |= H_IG;
	write_heci_register(device_object, H_CSR,
			    device_object->host_hw_state);
	device_object->me_hw_state =
	    read_heci_register(device_object, ME_CSR_HA);
	if ((device_object->me_hw_state & ME_RDY_HRA) != ME_RDY_HRA)
		return FALSE;

	device_object->write_hang = 0;
	return TRUE;
}

/**
 * count_full_read_slots  - reset host and fw.
 *
 * @device_object -Device object for our driver
 *
 *
 * @return :
 * -1(ESLOTS_OVERFLOW) if overflow
 * otherwise filed slots count
 */
__s32 count_full_read_slots(struct iamt_heci_device * device_object)
{

	char read_ptr, write_ptr;
	unsigned char buffer_depth, filled_slots, empty_slots;

	device_object->me_hw_state = read_heci_register(device_object, ME_CSR_HA);
	read_ptr = (char) ((device_object->me_hw_state & ME_CBRP_HRA) >> 8);
	write_ptr = (char) ((device_object->me_hw_state & ME_CBWP_HRA) >> 16);
	buffer_depth = (unsigned char) ((device_object->me_hw_state & ME_CBD_HRA) >>
			     24);
	filled_slots = (unsigned char) (write_ptr - read_ptr);
	empty_slots = buffer_depth - filled_slots;

	if (filled_slots > buffer_depth)
		/* overflow */
		return -ESLOTS_OVERFLOW;

	DBG("filled_slots =%08x  \n", filled_slots);
	return (__s32) filled_slots;
}

/**
 * heci_read_slots  - read a message from heci device.
 *
 * @device_object  - device object for our driver
 * @buffer         - message buffer will be write
 * @buffer_length  - message size will be read
 *
 * @return :
 * none;
 */
void heci_read_slots(struct iamt_heci_device * device_object,
		     unsigned char *buffer, unsigned long buffer_length)
{
	__u32 i = 0;
	unsigned char temp_buf[sizeof(__u32)];

	while (buffer_length >= sizeof(__u32)) {
		((__u32 *) buffer)[i] =
		    read_heci_register(device_object, ME_CB_RW);
		DBG("buffer[%d]= %d\n", i, ((__u32 *) buffer)[i]);
		i++;
		buffer_length -= sizeof(__u32);
	}

	if (buffer_length > 0) {
		*((__u32 *) & temp_buf) =
		    read_heci_register(device_object, ME_CB_RW);
		memcpy(&buffer[i * 4], temp_buf, buffer_length);
	}

	device_object->host_hw_state |= H_IG;
	write_heci_register(device_object, H_CSR,
			    device_object->host_hw_state);
	return;
}

/**
 * flow_control_credentials  - check flow_control credentials.
 *
 * @device_object -Device object for our driver
 * @file_extension -extension of the file object
 *
 * @return :
 * TRUE if flow_control_credentials >0
 * FALSE - otherwise.
 */
int flow_control_credentials(struct iamt_heci_device * device_object,
				   struct heci_file_private * file_extension)
{
	__u8 i;

	if (!device_object->num_heci_me_clients)
		return FALSE;
	if (file_extension == NULL)
		return FALSE;
	if (file_extension->flow_control_credentials > 0)
		return TRUE;

	for (i = 0; i < device_object->num_heci_me_clients; i++) {
		if (device_object->me_clients[i].client_id ==
		    file_extension->me_client_id) {
			if (device_object->me_clients[i].flow_control_credentials > 0) {
				BUG_ON(device_object->me_clients[i].
				       properteis.single_receive_buffer == 0);
				return TRUE;
			}
			return FALSE;
		}
	}
	BUG_ON(1);
	return FALSE;
}

/**
 * flow_control_reduce  - reduce flow_control .
 *
 * @device_object -Device object for our driver
 * @file_extension -extension of the file object
 *
 * @return :
 * none;
 */
void flow_control_reduce(struct iamt_heci_device * device_object,
			 struct heci_file_private * file_extension)
{
	__u8 i;

	if (!device_object->num_heci_me_clients)
		return;

	for (i = 0; i < device_object->num_heci_me_clients; i++) {
		if (device_object->me_clients[i].client_id == file_extension->me_client_id) {
			if (device_object->me_clients[i].properteis.single_receive_buffer != 0) {
				BUG_ON(device_object->me_clients[i].
				       flow_control_credentials <= 0);
				device_object->me_clients[i].
				    flow_control_credentials--;
			} else {
				BUG_ON(file_extension->
				       flow_control_credentials <= 0);
				file_extension->flow_control_credentials--;
			}
			return;
		}
	}
	BUG_ON(1);
}

/**
 * heci_send_flow_control  - send flow control to fw.
 *
 * @device_object -Device object for our driver
 * @file_extension -extension of the file object
 *
 * @return :
 * TRUE if success
 * FALSE - otherwise.
 */
int heci_send_flow_control(struct iamt_heci_device * device_object,
				 struct heci_file_private * file_extension)
{
	struct heci_message_header *heci_header;
	struct hbm_flow_control *heci_flow_control;

	heci_header =
	    (struct heci_message_header *) & device_object->
	    write_message_buffer[0];
	heci_header->host_address = 0;
	heci_header->me_address = 0;
	heci_header->length = sizeof(struct hbm_flow_control);
	heci_header->message_complete = 1;
	heci_header->reserved = 0;

	heci_flow_control =
	    (struct hbm_flow_control *) & device_object->
	    write_message_buffer[1];
	memset(heci_flow_control, 0, sizeof(heci_flow_control));
	heci_flow_control->host_address = file_extension->host_client_id;
	heci_flow_control->me_address = file_extension->me_client_id;
	heci_flow_control->command.command = FLOW_CONTROL_CMD;
	memset(heci_flow_control->reserved, 0, sizeof(heci_flow_control->reserved));
	DBG("sending flow control host client = %d, me client = %d\n",
	    file_extension->host_client_id, file_extension->me_client_id);
	if (!heci_write_message(device_object, heci_header,
			       (unsigned char *) heci_flow_control, sizeof(struct hbm_flow_control)))
		return FALSE;
	return TRUE;

}

/**
 * other_client_is_connecting  - check if other
 * client with the same client id is connected.
 *
 * @device_object -Device object for our driver
 * @file_extension -extension of the file object
 *
 * @return :
 * TRUE if other client is connected.
 * FALSE - otherwise.
 */
int other_client_is_connecting(struct iamt_heci_device * device_object,
				     struct heci_file_private * file_extension)
{

	struct heci_file_private *file_extension_pos = NULL;
	struct heci_file_private *file_extension_next = NULL;
        list_for_each_entry_safe(file_extension_pos, file_extension_next, &device_object->file_list, link) {
		if ((file_extension_pos->state == HECI_FILE_CONNECTING)
		    && (file_extension_pos != file_extension)
		    && file_extension->me_client_id ==
		    file_extension_pos->me_client_id)
			return TRUE;
	}
	return FALSE;
}

/**
 * heci_send_wd  - send watch dog message to fw.
 *
 * @device_object -Device object for our driver
 *
 * @return :
 * TRUE if success
 * FALSE - otherwise.
 */
int heci_send_wd(struct iamt_heci_device * device_object)
{
	struct heci_message_header *heci_header;

	heci_header =
	    (struct heci_message_header *) & device_object->
	    write_message_buffer[0];
	heci_header->host_address =
	    device_object->wd_file_extension.host_client_id;
	heci_header->me_address =
	    device_object->wd_file_extension.me_client_id;
	heci_header->message_complete = 1;
	heci_header->reserved = 0;

	if (!memcmp(device_object->wd_data, interface_start_wd_params,
	     HECI_WD_PARAMS_SIZE)) {
		heci_header->length = HECI_START_WD_DATA_SIZE;
	} else {
		BUG_ON(memcmp(device_object->wd_data, interface_stop_wd_params,
			HECI_WD_PARAMS_SIZE));
		heci_header->length = HECI_WD_PARAMS_SIZE;
	}

	if (!heci_write_message(device_object, heci_header,
			       device_object->wd_data,
			       heci_header->length))
		return FALSE;
	return TRUE;
}


/**
 * heci_disconnect  - send disconnect message  to fw.
 *
 * @device_object -Device object for our driver
 * @file_extension -extension of the file object
 *
 * @return :
 * TRUE if success
 * FALSE - otherwise.
 */
int heci_disconnect(struct iamt_heci_device * device_object,
			  struct heci_file_private * file_extension)
{
	struct heci_message_header *heci_header;
	struct hbm_client_disconnect_request *heci_cli_disconnect;

	heci_header =
	    (struct heci_message_header *) & device_object->
	    write_message_buffer[0];
	heci_header->host_address = 0;
	heci_header->me_address = 0;
	heci_header->length = sizeof(struct hbm_client_disconnect_request);
	heci_header->message_complete = 1;
	heci_header->reserved = 0;

	heci_cli_disconnect =
	    (struct hbm_client_disconnect_request *) & device_object->
	    write_message_buffer[1];
	memset(heci_cli_disconnect, 0, sizeof(heci_cli_disconnect));
	heci_cli_disconnect->host_address = file_extension->host_client_id;
	heci_cli_disconnect->me_address = file_extension->me_client_id;
	heci_cli_disconnect->command.command = CLIENT_DISCONNECT_REQ_CMD;
	heci_cli_disconnect->reserved[0] = 0;

	if (TRUE != heci_write_message(device_object, heci_header,
			       (unsigned char *) heci_cli_disconnect, sizeof(struct hbm_client_disconnect_request)))
		return FALSE;
	return TRUE;
}

/**
 * heci_connect  - send connect message  to fw.
 *
 * @device_object -Device object for our driver
 * @file_extension -extension of the file object
 *
 * @return :
 * TRUE if success
 * FALSE - otherwise.
 */
int heci_connect(struct iamt_heci_device * device_object,
		       struct heci_file_private * file_extension)
{
	struct heci_message_header *heci_header;
	struct hbm_client_connect_request *heci_cli_connect;

	heci_header =
	    (struct heci_message_header *) & device_object->
	    write_message_buffer[0];
	heci_header->host_address = 0;
	heci_header->me_address = 0;
	heci_header->length = sizeof(struct hbm_client_connect_request);
	heci_header->message_complete = 1;
	heci_header->reserved = 0;

	heci_cli_connect =
	    (struct hbm_client_connect_request *) & device_object->
	    write_message_buffer[1];
	heci_cli_connect->host_address = file_extension->host_client_id;
	heci_cli_connect->me_address = file_extension->me_client_id;
	heci_cli_connect->command.command = CLIENT_CONNECT_REQ_CMD;
	heci_cli_connect->reserved = 0;
	if (TRUE != heci_write_message(device_object, heci_header,
			       (unsigned char *) heci_cli_connect, sizeof(struct hbm_client_connect_request)))
		return FALSE;
	return TRUE;
}
