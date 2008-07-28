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

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/aio.h>
#include <linux/pci.h>
#include <linux/reboot.h>
#include <linux/poll.h>
#include <linux/init.h>
#include <linux/kdev_t.h>
#include <linux/ioctl.h>
#include <linux/cdev.h>
#include <linux/list.h>
#include <asm/uaccess.h>
#include <asm/unistd.h>
#include <linux/delay.h>

#include "heci_data_structures.h"
#include "heci.h"
#include "heci_interface.h"
#include "version.h"


/**
 * heci_ioctl_get_version - the get driver version IOCTL function
 * @device_object -Device object for our driver
 * @if_num  minor number
 * @*u_msg pointer to user data struct in user space
 * @k_msg data in kernel on the stack
 * @file_extension -extension of the file object
 *
 * @return :
 *  0 on success,
 *  negative on failure.
 */
int heci_ioctl_get_version(struct iamt_heci_device * device, int if_num,
			   struct heci_message_data *u_msg,
			   struct heci_message_data k_msg,
			   struct heci_file_private * file_extension)
{

	int return_status = ESUCCESS;
	struct heci_driver_version *version;
	struct heci_message_data res_msg;
	res_msg.data = NULL;
	if ((if_num != HECI_MINOR_NUMBER) || (!device)
	    || (!file_extension))
		return -ENODEV;


	if (k_msg.size < (sizeof(struct heci_driver_version) - 2)) {
		DBG("user buffer less than heci_driver_version.\n");
		return -EMSGSIZE;
	}

	res_msg.data = kmalloc(sizeof(struct heci_driver_version), GFP_KERNEL);
	if (!res_msg.data) {
		DBG("failed allocation response buffer size = %d.\n",
		    (int) sizeof(struct heci_driver_version));
		return -ENOMEM;

	}
	version = (struct heci_driver_version *) res_msg.data;
	version->major = MAJOR_VERSION;
	version->minor = MINOR_VERSION;
	version->hotfix = QUICK_FIX_NUMBER;
	if (k_msg.size < sizeof(struct heci_driver_version)) {
		res_msg.size = sizeof(struct heci_driver_version) - 2;
	} else {
		version->build = VER_BUILD;
		res_msg.size = sizeof(struct heci_driver_version);
	}
	return_status = file_extension->status;
	/* now copy the data to user space */
	if (copy_to_user(k_msg.data, res_msg.data, res_msg.size)) {
		return_status = -EFAULT;
		goto end;
	}
	if (put_user(res_msg.size, &u_msg->size)) {
		return_status = -EFAULT;
		goto end;
	}
end:
	kfree(res_msg.data);
	return return_status;
}

/**
 * heci_ioctl_connect_client - the connect to fw client IOCTL function
 * @device_object -Device object for our driver
 * @if_num  minor number
 * @*u_msg pointer to user data struct in user space
 * @k_msg data in kernel on the stack
 * @file_extension -extension of the file object
 *
 * @return :
 *  0 on success,
 *  negative on failure.
 */
int heci_ioctl_connect_client(struct iamt_heci_device * device, int if_num,
			      struct heci_message_data *u_msg,
			      struct heci_message_data k_msg,
			      struct file *file)
{

	int return_status = ESUCCESS;
	struct heci_message_data req_msg, res_msg;
	struct heci_cb_private *kernel_priv_cb = NULL;
	struct heci_client *client;
	struct heci_file_private *file_extension = NULL;
    struct heci_file_private *file_extension_pos = NULL;
	struct heci_file_private *file_extension_next = NULL;
	struct heci_file_private *file_extension_list_temp = NULL;

        struct heci_cb_private *kernel_priv_cb_pos = NULL, *kernel_priv_cb_next = NULL;
	long timeout = 15;	/*15 second */
	__u8 i;
	int err = 0;
	res_msg.data = NULL;
	req_msg.data = NULL;
	if ((if_num != HECI_MINOR_NUMBER) || (!device)
	    || (!file))
		return -ENODEV;
	file_extension = file->private_data;
	if (!file_extension)
		return -ENODEV;
	if (k_msg.size != sizeof(struct guid)) {
		DBG("user buffer size is not equal to size of struct guid(16).\n");
		return -EMSGSIZE;
	}
	req_msg.data = kmalloc(sizeof(struct guid), GFP_KERNEL);
	res_msg.data = kmalloc(sizeof(struct heci_client), GFP_KERNEL);


	if (!res_msg.data) {
		DBG("failed allocation response buffer size = %d.\n",
				(int) sizeof(struct heci_client));
		if (req_msg.data) {
			kfree(req_msg.data);
			req_msg.data = NULL;
			goto fail;
		}
	}
	if (!req_msg.data) {
		DBG("failed allocation request buffer size = %d.\n",
				(int) sizeof(struct guid));
		if (res_msg.data) {
			kfree(res_msg.data);
			res_msg.data = NULL;
			goto fail;
		}
	      fail:
		return -ENOMEM;
	}
	req_msg.size = sizeof(struct guid);
	res_msg.size = sizeof(struct heci_client);
	if (!k_msg.data) {
		return_status = -EIO;
		goto end;
	}

	/* copy the message to kernel space - use a pointer already copied into kernel space */
	if (copy_from_user(req_msg.data, k_msg.data, k_msg.size)) {
		return_status = -EFAULT;
		goto end;
	}
	/* buffered ioctl cb */
	kernel_priv_cb =
	    kmalloc(sizeof(struct heci_cb_private), GFP_KERNEL);
	if (!kernel_priv_cb) {
		return_status = -ENOMEM;
		goto end;
	}
	INIT_LIST_HEAD(&kernel_priv_cb->cb_list);
	kernel_priv_cb->response_buffer.data = res_msg.data;
	kernel_priv_cb->response_buffer.size = res_msg.size;
	kernel_priv_cb->request_buffer.data = req_msg.data;
	kernel_priv_cb->request_buffer.size = req_msg.size;
	kernel_priv_cb->major_file_operations = HECI_IOCTL;
	spin_lock_bh(&device->device_lock);
	if (device->heci_state != HECI_ENABLED) {
		return_status = -ENODEV;
		spin_unlock_bh(&device->device_lock);
		goto end;
	}
	if ((file_extension->state != HECI_FILE_INITIALIZING) &&
	    (file_extension->state != HECI_FILE_DISCONNECTED)) {
		return_status = -EBUSY;
		spin_unlock_bh(&device->device_lock);
		goto end;
	}

	/* find ME client we're trying to connect to */
	for (i = 0; i < device->num_heci_me_clients; i++) {
		if (0 == memcmp((struct guid *) req_msg.data,
			&device->me_clients[i].properteis.protocol_name,
			sizeof(struct guid))) {
			if (device->me_clients[i].properteis.fixed_address == 0) {
				file_extension->me_client_id =
				    device->me_clients[i].client_id;
				file_extension->state =
				    HECI_FILE_CONNECTING;
			}
			break;
		}
	}
	/*if we're connecting to PTHI client so we will use the exist connection */
	if (0 == memcmp((struct guid *) req_msg.data, &heci_pthi_guid,
		   sizeof(struct guid))) {

		if (device->legacy_file_extension.state != HECI_FILE_CONNECTED) {
			return_status = -ENODEV;
			spin_unlock_bh(&device->device_lock);
			goto end;
		}
		device->heci_host_clients[file_extension->host_client_id / 8] &=
		    ~(1 << (file_extension->host_client_id % 8));
                list_for_each_entry_safe(file_extension_pos, file_extension_next, &device->file_list, link) {
			if ((file_extension->host_client_id ==
			     file_extension_pos->host_client_id)
			    && (file_extension->me_client_id ==
				file_extension_pos->me_client_id)) {

				DBG("remove file extension node host client = %d, ME client = %d\n",
						file_extension_pos->host_client_id,
						file_extension_pos->me_client_id);
				list_del(&file_extension_pos->link);
			}

		}
		DBG("free file extension memory\n");
		kfree(file_extension);
		file_extension = NULL;
		file->private_data = &device->legacy_file_extension;
		client = (struct heci_client *) res_msg.data;
		client->max_message_length =
		    device->me_clients[i].properteis.max_message_length;
		client->protocol_version =
		    device->me_clients[i].properteis.protocol_version;
		return_status = device->legacy_file_extension.status;
		spin_unlock_bh(&device->device_lock);

		/* now copy the data to user space */
		if (copy_to_user(k_msg.data, res_msg.data, res_msg.size)) {
			return_status = -EFAULT;
			goto end;
		}
		if (put_user(res_msg.size, &u_msg->size)) {
			return_status = -EFAULT;
			goto end;
		}
		goto end;
	}
	spin_lock(&file_extension->file_lock);
	if (file_extension->state != HECI_FILE_CONNECTING) {
		return_status = -ENODEV;
		spin_unlock(&file_extension->file_lock);
		spin_unlock_bh(&device->device_lock);
		goto end;
	}
	spin_unlock(&file_extension->file_lock);
	/* prepare the output buffer */
	client = (struct heci_client *) res_msg.data;
	client->max_message_length =
	    device->me_clients[i].properteis.max_message_length;
	client->protocol_version =
	    device->me_clients[i].properteis.protocol_version;
	if (device->host_buffer_is_empty
	    && !other_client_is_connecting(device, file_extension)) {
		device->host_buffer_is_empty = FALSE;
		if (!heci_connect(device, file_extension)) {
			return_status = -ENODEV;
			spin_unlock_bh(&device->device_lock);
			goto end;
		} else {
			file_extension->timer_count = CONNECT_TIMEOUT;
			kernel_priv_cb->file_private = file_extension;
			list_add_tail(&kernel_priv_cb->cb_list,
				      &device->control_read_list.heci_cb.
				      cb_list);
		}


	} else {
		kernel_priv_cb->file_private = file_extension;
		DBG("add connect cb to control write list\n");
		list_add_tail(&kernel_priv_cb->cb_list,
			      &device->control_write_list.heci_cb.cb_list);
	}
	spin_unlock_bh(&device->device_lock);
	err =
	    wait_event_timeout(device->wait_received_message,
					     (HECI_FILE_CONNECTED == file_extension->state || HECI_FILE_DISCONNECTED == file_extension->state),
					     timeout * HZ);
	if (HECI_FILE_CONNECTED == file_extension->state) {
		DBG("successfully to connect to FW client.\n");
		return_status = file_extension->status;
		/* now copy the data to user space */
		if (copy_to_user(k_msg.data, res_msg.data, res_msg.size)) {
			return_status = -EFAULT;
			goto end;
		}
		if (put_user(res_msg.size, &u_msg->size)) {
			return_status = -EFAULT;
			goto end;
		}
		goto end;
	} else {
		DBG("failed to connect to FW client.file_extension->state = %d\n", file_extension->state);
		if (!err)
			DBG("wait_event_interruptible_timeout failed on client connect message fw response message\n");

		return_status = -EFAULT;
		goto remove_list;
	}

remove_list:
	if (kernel_priv_cb) {
		spin_lock_bh(&device->device_lock);
		if (device->control_read_list.status == ESUCCESS
				&& !list_empty(&device->control_read_list.heci_cb.cb_list)) {
			list_for_each_entry_safe(kernel_priv_cb_pos, kernel_priv_cb_next, &device->control_read_list.heci_cb.cb_list, cb_list){
				file_extension_list_temp =
					(struct heci_file_private *)
					kernel_priv_cb_pos->file_private;
				if (file_extension_list_temp) {
					if ((file_extension->host_client_id == file_extension_list_temp->host_client_id)
							&& (file_extension->me_client_id == file_extension_list_temp->me_client_id))
						list_del(&kernel_priv_cb_pos->cb_list);

				}

			}
		}
		if (device->control_write_list.status == ESUCCESS
				&& !list_empty(&device->control_write_list.heci_cb.cb_list)) {
			list_for_each_entry_safe(kernel_priv_cb_pos, kernel_priv_cb_next, &device->control_write_list.heci_cb.cb_list, cb_list){
				file_extension_list_temp =
					(struct heci_file_private *)
					kernel_priv_cb_pos->file_private;
				if (file_extension_list_temp) {
					if ((file_extension->host_client_id == file_extension_list_temp->host_client_id)
							&& (file_extension->me_client_id == file_extension_list_temp->me_client_id))
						list_del(&kernel_priv_cb_pos->cb_list);

				}

			}
		}
		spin_unlock_bh(&device->device_lock);
	}
end:
       DBG("free connect cb memory");
	kfree(req_msg.data);
	req_msg.data = NULL;
	kfree(res_msg.data);
	res_msg.data = NULL;
	kfree(kernel_priv_cb);
	kernel_priv_cb = NULL;
	return return_status;
}

/**
 * heci_ioctl_wd  - the wd IOCTL function
 * @device_object -Device object for our driver
 * @if_num  minor number
 * @k_msg data in kernel on the stack
 * @file_extension -extension of the file object
 *
 * @return :
 *  0 on success,
 *  negative on failure.
 */
int heci_ioctl_wd(struct iamt_heci_device * device, int if_num,
		  struct heci_message_data k_msg,
		  struct heci_file_private * file_extension)
{
	int return_status = ESUCCESS;
	struct heci_message_data req_msg;	/*in kernel on the stack */
	if (if_num != HECI_MINOR_NUMBER)
		return -ENODEV;
	spin_lock(&file_extension->file_lock);
	if (k_msg.size != HECI_WATCHDOG_DATA_SIZE) {
		DBG("User buffer has invalid size.\n");
		spin_unlock(&file_extension->file_lock);
		return -EMSGSIZE;
	}
	spin_unlock(&file_extension->file_lock);
	req_msg.data = kmalloc(HECI_WATCHDOG_DATA_SIZE, GFP_KERNEL);
	if (!req_msg.data) {
		DBG("failed allocation request buffer size = %d.\n",
		    HECI_WATCHDOG_DATA_SIZE);
		return -ENOMEM;
	}
	req_msg.size = HECI_WATCHDOG_DATA_SIZE;

	/* copy the message to kernel space - use a pointer already copied into kernel space */
	if (copy_from_user(req_msg.data, k_msg.data, req_msg.size)) {
		return_status = -EFAULT;
		goto end;
	}
	spin_lock_bh(&device->device_lock);
	if (device->heci_state != HECI_ENABLED) {
		return_status = -ENODEV;
		spin_unlock_bh(&device->device_lock);
		goto end;
	}

	if (device->wd_file_extension.state != HECI_FILE_CONNECTED) {
		return_status = -ENODEV;
		spin_unlock_bh(&device->device_lock);
		goto end;
	}
	if (!device->asf_mode) {
		return_status = -EIO;
		spin_unlock_bh(&device->device_lock);
		goto end;
	}

	memcpy(&device->wd_data[HECI_WD_PARAMS_SIZE], req_msg.data,
	       HECI_WATCHDOG_DATA_SIZE);

	device->wd_timeout = (req_msg.data[1] << 8) +
	    req_msg.data[0];
	if (device->wd_timeout == 0) {
		memcpy(device->wd_data, &stop_wd_params,
		       HECI_WD_PARAMS_SIZE);
		device->wd_pending = FALSE;
		device->wd_due_counter = 1;	/* next timer */
	} else {
		memcpy(device->wd_data, &start_wd_params,
		       HECI_WD_PARAMS_SIZE);
		device->wd_pending = FALSE;
		device->wd_due_counter = 1;
	}
	spin_unlock_bh(&device->device_lock);
end:
	kfree(req_msg.data);
	req_msg.data = NULL;
	return return_status;
}


/**
 * heci_ioctl_bypass_wd  - the bypass_wd IOCTL function
 * @device_object -Device object for our driver
 * @if_num  minor number
 * @k_msg data in kernel on the stack
 * @file_extension -extension of the file object
 * 
 * @return :
 *  0 on success,
 *  negative on failure.
 */
int heci_ioctl_bypass_wd(struct iamt_heci_device * device, int if_num,
		  struct heci_message_data k_msg,
		  struct heci_file_private * file_extension)
{
	__u8 flag=0;
	int return_status = ESUCCESS;

	if (if_num != HECI_MINOR_NUMBER)
		return -ENODEV;
	spin_lock(&file_extension->file_lock);
	if (k_msg.size < 1) {
		DBG("user buffer less than HECI_WATCHDOG_DATA_SIZE .\n");
		spin_unlock(&file_extension->file_lock);
		return -EMSGSIZE;
	}
	spin_unlock(&file_extension->file_lock);
	if (copy_from_user(&flag,k_msg.data,1)) {
		return_status = -EFAULT;
		goto end;
	}

	spin_lock_bh(&device->device_lock);
	flag = flag ?(TRUE):(FALSE);
	device->wd_bypass = flag;
	spin_unlock_bh(&device->device_lock);
end:
	return return_status;
}


/**
 * legacy_ioctl_send_message - send command data to pthi client
 * @device_object -Device object for our driver
 * @if_num  minor number
 * @*u_msg pointer to user data struct in user space
 * @k_msg data in kernel on the stack
 * @file_extension -extension of the file object
 *
 * @return :
 *  0 on success,
 *  negative on failure.
 */
int legacy_ioctl_send_message(struct iamt_heci_device * device, int if_num,
			      struct heci_message_data k_msg,
			      struct file *file)
{

	int return_status = ESUCCESS;
	struct heci_message_data req_msg, res_msg;	/*in kernel on the stack */
	struct heci_cb_private *kernel_priv_cb = NULL;
	struct heci_file_private *file_extension = file->private_data;
	__u8 i;
	req_msg.data = NULL;
	res_msg.data = NULL;
	if ((if_num != LEGACY_MINOR_NUMBER) || (!device))
		return -ENODEV;
	if (!file_extension)
		return -ENODEV;
	spin_lock_bh(&device->device_lock);
	for (i = 0; i < device->num_heci_me_clients; i++) {
		if (device->me_clients[i].client_id ==
		    device->legacy_file_extension.me_client_id)
			break;
	}

	BUG_ON(device->me_clients[i].client_id != file_extension->me_client_id);
	if ((i == device->num_heci_me_clients)
	    || (device->me_clients[i].client_id !=
		device->legacy_file_extension.me_client_id)) {
		spin_unlock_bh(&device->device_lock);
		return -ENODEV;
	} else if (k_msg.size >
		   device->me_clients[i].properteis.max_message_length
		   || k_msg.size <= 0) {
		spin_unlock_bh(&device->device_lock);
		return -EMSGSIZE;
	}


	spin_unlock_bh(&device->device_lock);
	req_msg.data = kmalloc(k_msg.size, GFP_KERNEL);
	res_msg.data = kmalloc(LEGACY_MTU, GFP_KERNEL);

	if (!res_msg.data) {
		DBG("failed allocation response buffer size = %d.\n",
				(int) sizeof(struct heci_client));
		if (req_msg.data) {

			kfree(req_msg.data);
			req_msg.data = NULL;
			return -ENOMEM;
		}
	}
	if (!req_msg.data) {
		DBG("failed allocation request buffer size = %d.\n",
				(int) sizeof(struct guid));
		if (res_msg.data) {
			kfree(res_msg.data);
			res_msg.data = NULL;
			return -ENOMEM;
		}
	}
	req_msg.size = k_msg.size;
	res_msg.size = LEGACY_MTU;
	if (!k_msg.data) {
		return_status = -EIO;
		goto end;
	}

	/* copy the message to kernel space - use a pointer already copied into kernel space */
	if (copy_from_user(req_msg.data, k_msg.data, k_msg.size)) {
		return_status = -EFAULT;
		goto end;
	}
	/* buffered ioctl cb */
	kernel_priv_cb =
	    kmalloc(sizeof(struct heci_cb_private), GFP_KERNEL);
	if (!kernel_priv_cb) {
		return_status = -ENOMEM;
		goto end;
	}
	INIT_LIST_HEAD(&kernel_priv_cb->cb_list);

	kernel_priv_cb->request_buffer.data = req_msg.data;
	kernel_priv_cb->request_buffer.size = req_msg.size;
	kernel_priv_cb->response_buffer.data = res_msg.data;
	kernel_priv_cb->response_buffer.size = res_msg.size;
	kernel_priv_cb->major_file_operations = HECI_IOCTL;
	kernel_priv_cb->information = 0;
	kernel_priv_cb->file_object = file;
	kernel_priv_cb->file_private = file_extension;
	spin_lock_bh(&device->device_lock);

	if (device->heci_state != HECI_ENABLED) {
		return_status = -ENODEV;
		spin_unlock_bh(&device->device_lock);
		goto end;
	}
	if (device->legacy_file_extension.state != HECI_FILE_CONNECTED) {
		return_status = -ENODEV;
		spin_unlock_bh(&device->device_lock);
		goto end;
	}


	if (!list_empty(&device->pthi_cmd_list.heci_cb.cb_list)
	    || device->legacy_state != HECI_LEGACY_IDLE) {
		DBG("pthi_state = %d\n", (int) device->legacy_state);
		DBG("add PTHI cb to pthi cmd waiting list");
		list_add_tail(&kernel_priv_cb->cb_list,
			      &device->pthi_cmd_list.heci_cb.cb_list);

	} else {
		DBG("call pthi write\n");
		return_status = pthi_write(device, kernel_priv_cb);

		if (ESUCCESS != return_status) {
			DBG("pthi write failed with status = %d\n",
			    return_status);
			spin_unlock_bh(&device->device_lock);
			goto end;
		}
	}
	spin_unlock_bh(&device->device_lock);
	return return_status;
end:
	kfree(req_msg.data);
	req_msg.data = NULL;
	kfree(res_msg.data);
	res_msg.data = NULL;
	kfree(kernel_priv_cb);
	kernel_priv_cb = NULL;
	return return_status;
}


/**
 * legacy_ioctl_receive_message - receive  command data from pthi client
 * @device_object -Device object for our driver
 * @if_num  minor number
 * @*u_msg pointer to user data struct in user space
 * @k_msg data in kernel on the stack
 * @file_extension -extension of the file object
 *
 * @return :
 *  0 on success,
 *  negative on failure.
 */
int legacy_ioctl_receive_message(struct iamt_heci_device * device, int if_num,
				 struct heci_message_data *u_msg,
				 struct heci_message_data k_msg,
				 struct file *file)
{

	int return_status = ESUCCESS;
	struct heci_message_data req_msg, res_msg;
	struct heci_cb_private *kernel_priv_cb = NULL;
	struct heci_file_private *file_extension = file->private_data;
	__u8 i;
	struct heci_file_private *file_extension_list_temp = NULL;
	struct heci_cb_private *kernel_priv_cb_pos = NULL, *kernel_priv_cb_next = NULL;


	res_msg.data = NULL;
	req_msg.data = NULL;

	if ((if_num != LEGACY_MINOR_NUMBER) || (!device))
		return -ENODEV;
	spin_lock_bh(&device->device_lock);
	if (!file_extension) {
		spin_unlock_bh(&device->device_lock);
		return -ENODEV;
	}
	for (i = 0; i < device->num_heci_me_clients; i++)
		if (device->me_clients[i].client_id ==
		    device->legacy_file_extension.me_client_id)
			break;

	BUG_ON(device->me_clients[i].client_id !=
	       file_extension->me_client_id);
	if ((i == device->num_heci_me_clients)
	    || (device->me_clients[i].client_id !=
		device->legacy_file_extension.me_client_id)) {
		spin_unlock_bh(&device->device_lock);
		return -ENODEV;
	}

	if (device->pthi_read_complete_list.status == ESUCCESS) {
		list_for_each_entry_safe(kernel_priv_cb_pos, kernel_priv_cb_next, &device->pthi_read_complete_list.heci_cb.cb_list, cb_list){
			file_extension_list_temp =
				(struct heci_file_private *)
				kernel_priv_cb_pos->file_private;
			if (file_extension_list_temp) {
				if ((file_extension == &device->legacy_file_extension)
						&& (kernel_priv_cb_pos->file_object == file)) {
					list_del(&kernel_priv_cb_pos->cb_list);
					kernel_priv_cb =
						kernel_priv_cb_pos;
					break;
				}
			}

		}
	}
	if (!kernel_priv_cb) {
		spin_unlock_bh(&device->device_lock);
		return -EAGAIN;
	}


	res_msg.data = kernel_priv_cb->response_buffer.data;
	res_msg.size = kernel_priv_cb->response_buffer.size;
	req_msg.data = kernel_priv_cb->request_buffer.data;
	req_msg.size = kernel_priv_cb->request_buffer.size;
	DBG("pthi kernel_priv_cb->response_buffer size - %d\n",
	    kernel_priv_cb->response_buffer.size);
	DBG("pthi kernel_priv_cb->information - %lu\n",
	    kernel_priv_cb->information);
	spin_unlock_bh(&device->device_lock);

	if (res_msg.size < kernel_priv_cb->information) {
		return_status = -EMSGSIZE;
		goto end;
	}
	/* now copy the data to user space */
	if (copy_to_user(k_msg.data,
				res_msg.data,
				kernel_priv_cb->information)) {
		return_status = -EFAULT;
		goto end;
	}
	if (put_user(kernel_priv_cb->information, &u_msg->size)) {
		return_status = -EFAULT;
		goto end;
	}
end:
	kfree(req_msg.data);
	kfree(res_msg.data);
	kfree(kernel_priv_cb);
	return return_status;
}

/**
 * find_pthi_read_list_entry - finds a PTHIlist entry for current file
 * @device_object -Device object for our driver
 *
 * @return :
 *  returned a list entry on success,
 *  NULL on failure.
 */
struct heci_cb_private* find_pthi_read_list_entry(struct iamt_heci_device* device,
					struct file* file, struct heci_file_private* file_extension)
{
	struct heci_file_private *file_extension_list_temp = NULL;
	struct heci_cb_private *kernel_priv_cb_pos = NULL, *kernel_priv_cb_next = NULL;

	if (device->pthi_read_complete_list.status == ESUCCESS
	    && !list_empty(&device->pthi_read_complete_list.heci_cb.
			   cb_list)) {
               list_for_each_entry_safe(kernel_priv_cb_pos, kernel_priv_cb_next, &device->pthi_read_complete_list.heci_cb.cb_list, cb_list){
			file_extension_list_temp =
			    (struct heci_file_private *)
			    kernel_priv_cb_pos->file_private;
			if (file_extension_list_temp) {
				if ((file_extension == &device->legacy_file_extension)
				    && (kernel_priv_cb_pos->file_object == file)) {
					return kernel_priv_cb_pos;
				}
			}
		}
	}
	return NULL;
}
/**
 * pthi_read - read data from pthi client
 * @device_object -Device object for our driver
 * @if_num  minor number
 * @*u_msg pointer to user data in user space
 * @length-user data length
 *
 * @return :
 *  returned data length on success,
 *  zero if no data to read,
 *  negative on failure.
 */
int pthi_read(struct iamt_heci_device * device, int if_num, struct file *file,
	      char *ubuf, size_t length, loff_t * offset)
{

	int return_status = ESUCCESS;
	struct heci_cb_private *kernel_priv_cb = NULL;
	struct heci_file_private *file_extension = file->private_data;
	__u8 i;
	unsigned long currtime = get_seconds();

	if ((if_num != HECI_MINOR_NUMBER) || (!device))
		return -ENODEV;

	if (!file_extension)
		return -ENODEV;

	spin_lock_bh(&device->device_lock);
	for (i = 0; i < device->num_heci_me_clients; i++)
		if (device->me_clients[i].client_id ==
		    device->legacy_file_extension.me_client_id)
			break;
	BUG_ON(device->me_clients[i].client_id != file_extension->me_client_id);
	if ((i == device->num_heci_me_clients)
	    || (device->me_clients[i].client_id !=
		device->legacy_file_extension.me_client_id)) {
		DBG("PTHI client not found\n");
		spin_unlock_bh(&device->device_lock);
		return -ENODEV;
	}
	kernel_priv_cb = find_pthi_read_list_entry(device, file, file_extension);
	if (!kernel_priv_cb) {
		spin_unlock_bh(&device->device_lock);
		return 0; /* No more data to read */
	}
	else {
		/* 15 sec for the message has expired */
		if (kernel_priv_cb && currtime - kernel_priv_cb->read_time > LEGACY_READ_TIMER) {
			list_del(&kernel_priv_cb->cb_list);
			spin_unlock_bh(&device->device_lock);
			return_status = -ETIMEDOUT;
			goto free;
		}
		/* if the whole message will fit remove it from the list */
		if ((kernel_priv_cb->information >= *offset)  &&
				(length >= (kernel_priv_cb->information - *offset))) {
			list_del(&kernel_priv_cb->cb_list);
		}
		/* end of the message has been reached */
		else if ((kernel_priv_cb->information > 0) &&
				(kernel_priv_cb->information <= *offset)) {
			list_del(&kernel_priv_cb->cb_list);
			return_status = 0;
			spin_unlock_bh(&device->device_lock);
			goto free;
		}
		/* else means that not full buffer will be read and do not remove message from deletion list */
	}
	DBG("pthi kernel_priv_cb->response_buffer size - %d\n",
	    kernel_priv_cb->response_buffer.size);
	DBG("pthi kernel_priv_cb->information - %lu\n",
	    kernel_priv_cb->information);
	spin_unlock_bh(&device->device_lock);

	/* length is being turncated to PAGE_SIZE, however, the information may be longer */
	length = length < (kernel_priv_cb->information - *offset) ?
						length : (kernel_priv_cb->information - *offset);

	if (copy_to_user
	    (ubuf, kernel_priv_cb->response_buffer.data + *offset, length)) {
		return_status = -EFAULT;
	} else {
		return_status = length;
		if ((*offset + length) < kernel_priv_cb->information) {
			*offset += length;
			goto out;
		}
	}
	DBG("free pthi cb memory\n");
free:
	*offset = 0;
	kfree(kernel_priv_cb->request_buffer.data);
	kfree(kernel_priv_cb->response_buffer.data);
	kfree(kernel_priv_cb);
out:
	return return_status;
}

/**
 * heci_start_read  - the start read client message function.
 * @device_object -Device object for our driver
 * @if_num  minor number
 * @file_extension -extension of the file object
 *
 * @return :
 *  0 on success,
 *  negative on failure.
 */
int heci_start_read(struct iamt_heci_device * device, int if_num,
		    struct heci_file_private * file_extension)
{
	int return_status = ESUCCESS;
	__u8 i;
	struct heci_cb_private *priv_cb = NULL;
	if ((if_num != HECI_MINOR_NUMBER) || (!device)
	    || (!file_extension)) {
		DBG("receive wrong function input param\n.");
		return -ENODEV;
	}
	if (file_extension->state != HECI_FILE_CONNECTED)
		return -ENODEV;
	spin_lock_bh(&device->device_lock);
	if (device->heci_state != HECI_ENABLED) {
		spin_unlock_bh(&device->device_lock);
		return -ENODEV;
	}
	spin_unlock_bh(&device->device_lock);
	DBG("check if read is pending\n");
	if (file_extension->read_pending
	    || file_extension->read_cb != NULL) {
		DBG("read is pending");
		return -EBUSY;
	}
	priv_cb = kmalloc(sizeof(struct heci_cb_private), GFP_KERNEL);
	if (!priv_cb)
		return -ENOMEM;

	DBG("allocation call back success\n");
	DBG("host client = %d, ME client = %d\n",
	    file_extension->host_client_id, file_extension->me_client_id);
	spin_lock_bh(&device->device_lock);
	for (i = 0; i < device->num_heci_me_clients; i++)
		if (device->me_clients[i].client_id ==
		    file_extension->me_client_id)
			break;

	BUG_ON(device->me_clients[i].client_id !=
	       file_extension->me_client_id);
	if ((i == device->num_heci_me_clients)) {
		return_status = -ENODEV;
		goto unlock;
	}

	priv_cb->response_buffer.size =
	    device->me_clients[i].properteis.max_message_length;
	spin_unlock_bh(&device->device_lock);
	priv_cb->response_buffer.data =
	    kmalloc(priv_cb->response_buffer.size, GFP_KERNEL);
	if (!priv_cb->response_buffer.data) {
		return_status = -ENOMEM;
		goto fail;
	}
	DBG("allocation call back data success\n");
	priv_cb->major_file_operations = HECI_READ;
	/* make sure information is zero before we start */
	priv_cb->information = 0;
	priv_cb->file_private = (void *) file_extension;
	file_extension->read_cb = priv_cb;
	spin_lock_bh(&device->device_lock);
	if (device->host_buffer_is_empty) {
		device->host_buffer_is_empty = FALSE;
		if (!heci_send_flow_control(device, file_extension)) {
			return_status = -ENODEV;
			goto unlock;
		} else {
			list_add_tail(&priv_cb->cb_list,
				      &device->read_list.heci_cb.cb_list);
		}
	} else {
		list_add_tail(&priv_cb->cb_list,
			      &device->control_write_list.heci_cb.cb_list);
	}
	spin_unlock_bh(&device->device_lock);
	return return_status;
unlock:
	spin_unlock_bh(&device->device_lock);
fail:
	kfree(priv_cb->response_buffer.data);
	priv_cb->response_buffer.data = NULL;
	priv_cb->file_private = NULL;
	kfree(priv_cb);
	return return_status;
}

/**
 * pthi_write - write legacy data to pthi client
 * @device -Device object for our driver
 * @kernel_priv_cb - heci call back struct
 * @return :
 *  0 on success,
 *  negative on failure.
 */
int pthi_write(struct iamt_heci_device * device,
	       struct heci_cb_private *kernel_priv_cb)
{
	int return_status = ESUCCESS;
	struct heci_message_header heci_header;

	if ((!device) || (!kernel_priv_cb))
		return -ENODEV;

	DBG("write data to pthi client\n");

	device->legacy_state = HECI_LEGACY_WRITING;
	device->legacy_current_cb = kernel_priv_cb;
	device->legacy_file_object = kernel_priv_cb->file_object;
	device->legacy_canceled = FALSE;
	device->legacy_ioctl = TRUE;
	device->legacy_message_buffer_size =
	    kernel_priv_cb->request_buffer.size;
	memcpy(device->legacy_message_buffer,
	       kernel_priv_cb->request_buffer.data,
	       kernel_priv_cb->request_buffer.size);
	if (flow_control_credentials
	    (device, &device->legacy_file_extension)
	    && device->host_buffer_is_empty) {
		device->host_buffer_is_empty = FALSE;
		if (kernel_priv_cb->request_buffer.size >
		    (((device->host_hw_state & H_CBD) >> 24) * sizeof(__u32)) - sizeof(struct heci_message_header)) {
			heci_header.length =
			    (((device->host_hw_state & H_CBD) >> 24) * sizeof(__u32)) - sizeof(struct heci_message_header);
			heci_header.message_complete = 0;
		} else {
			heci_header.length =
			    kernel_priv_cb->request_buffer.size;
			heci_header.message_complete = 1;
		}

		heci_header.host_address =
		    device->legacy_file_extension.host_client_id;
		heci_header.me_address =
		    device->legacy_file_extension.me_client_id;
		heci_header.reserved = 0;
		device->legacy_message_buffer_index += heci_header.length;
		if (!heci_write_message(device, &heci_header,
					       (unsigned char *) (device->legacy_message_buffer),
					       heci_header.length))
			return -ENODEV;

		if (heci_header.message_complete) {

			flow_control_reduce(device,
					    &device->
					    legacy_file_extension);
			device->legacy_flow_control_pending = TRUE;
			device->legacy_state = HECI_LEGACY_FLOW_CONTROL;
			DBG("add pthi cb to write waiting list\n");
			device->legacy_current_cb = kernel_priv_cb;
			device->legacy_file_object = kernel_priv_cb->file_object;
			list_add_tail(&kernel_priv_cb->cb_list,
				      &device->write_waiting_list.heci_cb.
				      cb_list);

		} else {
			DBG("message does not complete, so add pthi cb to write list\n");
			list_add_tail(&kernel_priv_cb->cb_list,
				      &device->write_list.heci_cb.cb_list);
		}



	} else {
		if (TRUE != device->host_buffer_is_empty)
			DBG("host buffer is not empty");
		DBG("No flow control credentials, so add legacy cb to write list\n");
		list_add_tail(&kernel_priv_cb->cb_list,
			      &device->write_list.heci_cb.cb_list);
	}
	return return_status;
}

/**
 * legacy_ioctl_send_message - send command data to pthi client
 * @device -Device object for our driver
 * @return :
 *  0 on success,
 *  negative on failure.
 */
void run_next_legacy_cmd(struct iamt_heci_device * device)
{
	struct heci_file_private *file_extension_temp = NULL;
	struct heci_cb_private *kernel_priv_cb_pos = NULL, *kernel_priv_cb_next = NULL;
	int legacy_write_status = ESUCCESS;

	if (!device)
		return;
	device->legacy_message_buffer_size = 0;
	device->legacy_message_buffer_index = 0;
	device->legacy_canceled = FALSE;
	device->legacy_ioctl = TRUE;
	device->legacy_state = HECI_LEGACY_IDLE;
	device->legacy_timer = 0;
	device->legacy_file_object = NULL;

	if (device->pthi_cmd_list.status == ESUCCESS
	    && !list_empty(&device->pthi_cmd_list.heci_cb.cb_list)) {
		DBG("complete pthi cmd_list CB\n ");
               list_for_each_entry_safe(kernel_priv_cb_pos, kernel_priv_cb_next, &device->pthi_cmd_list.heci_cb.cb_list, cb_list){
			list_del(&kernel_priv_cb_pos->cb_list);
			file_extension_temp = (struct heci_file_private *) kernel_priv_cb_pos->file_private;

			if ((file_extension_temp)
			    && (file_extension_temp == &device->legacy_file_extension)) {
				legacy_write_status =
				    pthi_write(device, kernel_priv_cb_pos);
				if (legacy_write_status != ESUCCESS) {
					DBG("pthi write failed with status = %d\n",
							legacy_write_status);
					return;
				}
				break;
			}
		}
	}
	return;
}
