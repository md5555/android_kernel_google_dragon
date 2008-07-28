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
#include <asm/uaccess.h>
#include <asm/unistd.h>

#include "kcompat.h"
#include "heci.h"
#include "heci_interface.h"
#include "version.h"


#define HECI_READ_TIMEOUT	45

#define  MAX_OPEN_HANDLE_COUNT			253
/**
 *  heci driver strings
 */
char heci_driver_name[] = "heci";
char heci_driver_string[] = "Intel(R) AMT Management Interface";
char heci_driver_version[] = DRIVER_VERSION;
char heci_copyright[] = "Copyright (c) 2003 - 2007 Intel Corporation.";


#ifdef HECI_DEBUG
DEF_PARM(int, debug, 1, 0644, "Debug enabled or not");
#else
DEF_PARM(int, debug, 0, 0644, "Debug enabled or not");
#endif

/* heci char device for registration */
static struct cdev heci_cdev = {
	.kobj = {.name = "heci", },
	.owner = THIS_MODULE,
};

/* iamt legacy char device for registration */
static struct cdev iamt_legacy_cdev = {
	.kobj = {.name = "iamt_legacy", },
	.owner = THIS_MODULE,
};

/* major number for device */
static int heci_major;
/* The device pointer */
static struct pci_dev *heci_device;

/* heci_pci_tbl - PCI Device ID Table */
static struct pci_device_id heci_pci_tbl[] = {
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, PCI_HECI_DEVICE_ID1)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, PCI_HECI_DEVICE_ID2)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, PCI_HECI_DEVICE_ID3)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, PCI_HECI_DEVICE_ID4)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, PCI_HECI_DEVICE_ID5)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, PCI_HECI_DEVICE_ID6)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, PCI_HECI_DEVICE_ID7)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, PCI_HECI_DEVICE_ID8)},
	/* required last entry */
	{0, }
};

MODULE_DEVICE_TABLE(pci, heci_pci_tbl);

/**
 * Local Function Prototypes
 */
static int __init heci_init_module(void);
static void __exit heci_exit_module(void);
static int __devinit heci_probe(struct pci_dev *pdev,
				const struct pci_device_id *ent);
static void __devexit heci_remove(struct pci_dev *pdev);
static int heci_open(struct inode *inode, struct file *file);
static int heci_release(struct inode *inode, struct file *file);
static unsigned int heci_legacy_poll(struct file *file, poll_table * wait);
static ssize_t heci_read(struct file *file, char __user * ubuf,
			 size_t length, loff_t * offset);
static int heci_ioctl(struct inode *inode, struct file *file,
		      unsigned int cmd, unsigned long data);
static ssize_t heci_write(struct file *file, const char __user * ubuf,
			  size_t length, loff_t * offset);
static unsigned int heci_poll(struct file *file, poll_table * wait);
#ifdef CONFIG_PM
static int heci_suspend(struct pci_dev* pdev, pm_message_t state);
static int heci_resume(struct pci_dev* pdev);
static __u16 g_sus_wd_timeout;
#endif
/**
 *  PCI driver structure
 */
static struct pci_driver heci_driver = {
	.name = heci_driver_name,
	.id_table = heci_pci_tbl,
	.probe = heci_probe,
	.remove = heci_remove,
	SHUTDOWN_METHOD(heci_remove)
#ifdef CONFIG_PM
	.suspend = heci_suspend, 
	.resume = heci_resume
#endif
};

/**
 * file operations structure will be use heci char device.
 */
static struct file_operations heci_fops = {
	.owner = THIS_MODULE,
	.read = heci_read,
	.ioctl = heci_ioctl,
	.open = heci_open,
	.release = heci_release,
	.write = heci_write,
	.poll = heci_poll,
};

/**
 * file operations structure will be use iamt legacy char device.
 */
static struct file_operations iamt_legacy_fops = {
	.owner = THIS_MODULE,
	.ioctl = heci_ioctl,
	.open = heci_open,
	.release = heci_release,
	.poll = heci_legacy_poll,
};

/**
 * For kernels withouth PCI shutdown support reboot notifier is essential
 */
HECI_REBOOT_NOTIFIER(heci_reboot_notifier, heci_driver, heci_remove)

/**
 * Set up the cdev structure for heci device.
 * @dev   - char device struct
 * @minor - minor number for registration char device
 * @fops  - file operations structure
 * @return :
 * 0 on success,
 * negative on failure
 */
static int heci_registration_cdev(struct cdev *dev, int minor,
				  struct file_operations *fops)
{
	int ret = ESUCCESS, devno = MKDEV(heci_major, minor);

	cdev_init(dev, fops);
	dev->owner = THIS_MODULE;
	dev->ops = fops;
	ret = cdev_add(dev, devno, 1);
	/* Fail gracefully if need be */
	if (ret) {
		kobject_put(&dev->kobj);
		HECI_ERR("Error %d registering heci device %d", ret, minor);
	}
	return ret;
}



/**
 * heci_init_module - Driver Registration Routine
 *
 * heci_init_module is the first routine called when the driver is
 * loaded. All it does is register with the PCI subsystem.
 *
 * @return :
 * 0 on success,
 * negative on failure
 */
static int __init heci_init_module(void)
{
	int ret = ESUCCESS;
	dev_t dev;
	HECI_INFO("%s - version %s\n", heci_driver_string, heci_driver_version);
	HECI_INFO("%s\n", heci_copyright);

	/* init pci module */
	ret = pci_register_driver(&heci_driver);
	if (ret < 0)
		goto end;

	REGISTER_REBOOT_NOTIFIER(heci_reboot_notifier);
	/* registration char devices */
	ret = alloc_chrdev_region(&dev, 0, MINORS_COUNT, "heci");

	heci_major = MAJOR(dev);
	/* Now registration two cdevs. */
	ret = heci_registration_cdev(&iamt_legacy_cdev, LEGACY_MINOR_NUMBER,
				   &iamt_legacy_fops);
	if (ret)
		goto unregister;

	ret = heci_registration_cdev(&heci_cdev, HECI_MINOR_NUMBER,
				   &heci_fops);
	if (ret) {
		cdev_del(&iamt_legacy_cdev);
		goto unregister;
	}
	return ret;

unregister:
	pci_unregister_driver(&heci_driver);
	unregister_chrdev_region(MKDEV(heci_major, 0), MINORS_COUNT);
end:
	return ret;
}

module_init(heci_init_module);


/**
 * heci_exit_module - Driver Exit Cleanup Routine
 *
 * heci_exit_module is called just before the driver is removed
 * from memory.
 *
 * @return :
 * none;
 */

static void __exit heci_exit_module(void)
{
	UNREGISTER_REBOOT_NOTIFIER(heci_reboot_notifier);
	pci_unregister_driver(&heci_driver);
	/* Now  unregister two cdevs. */
	cdev_del(&iamt_legacy_cdev);
	cdev_del(&heci_cdev);
	unregister_chrdev_region(MKDEV(heci_major, 0), MINORS_COUNT);
}

module_exit(heci_exit_module);


/**
 * heci_probe - Device Initialization Routine
 *
 * @pdev: PCI device information struct
 * @ent: entry in kcs_pci_tbl
 *
 * @return :
 * 0 on success,
 * negative on failure
 */
static int __devinit heci_probe(struct pci_dev *pdev,
				const struct pci_device_id *ent)
{
	struct iamt_heci_device *device = NULL;
	int i, err = 0;
	if (heci_device) {
		err = -EEXIST;
		goto end;
	}
	/* enable pci device */
	err = pci_enable_device(pdev);
	if (err) {
		HECI_ERR("Failed to enable pci device\n");
		goto end;
	}
	/* set PCI host mastering  */
	pci_set_master(pdev);
	/* pci request regions for heci driver */
	err = pci_request_regions(pdev, heci_driver_name);
	if (err) {
		HECI_ERR("Failed to get pci regions\n");
		goto disable_device;
	}
	/* allocates and initializes the heci device structure */
	device = init_heci_device(pdev);
	if (!device) {
		err = -ENOMEM;
		goto release_regions;
	}
	/* mapping  IO device memory */
	for (i = BAR_0; i <= BAR_5; i++) {
		if (pci_resource_len(pdev, i) == 0) {
			continue;
		}
		if (pci_resource_flags(pdev, i) & IORESOURCE_IO) {
			HECI_ERR("heci has an IO ports.\n");
			goto free_device;
		} else if (pci_resource_flags(pdev, i) & IORESOURCE_MEM) {
			if (device->mem_base) {
				HECI_ERR("Too many mem addresses.\n");
				goto free_device;
			}
			device->mem_base = pci_resource_start(pdev, i);
			device->mem_length = pci_resource_len(pdev, i);
		}
	}
	if (!device->mem_base) {
		HECI_ERR("No address to use.\n");
		err = -ENODEV;
		goto free_device;
	}
	device->mem_addr = ioremap_nocache(device->mem_base, device->mem_length);
	if (!device->mem_addr) {
		HECI_ERR(" Remap IO device memory failure.\n");
		err = -ENOMEM;
		goto free_device;
	}
	/* request and enable interrupt   */
	device->irq = pdev->irq;
       err = request_irq(device->irq, heci_isr_interrupt, IRQF_SHARED,
			heci_driver_name, device);
	if (err) {
		HECI_ERR("Request_irq failure. irq = %d \n", device->irq);
		goto unmap_memory;
	}

	if (heci_hw_init(device)) {
		HECI_ERR("init hw failure.\n");
		err = -ENODEV;
		goto release_irq;
	}
	init_timer(&device->wd_timer);
	heci_initialize_clients(device);
	if (device->heci_state != HECI_ENABLED) {
		err = -ENODEV;
		goto release_hw;
	}
	spin_lock_bh(&device->device_lock);
	heci_device = pdev;
	pci_set_drvdata(pdev, device);
	spin_unlock_bh(&device->device_lock);
	if (device->wd_timeout) {
		mod_timer(&device->wd_timer, jiffies);
	}
#ifdef CONFIG_PM
	g_sus_wd_timeout = 0;
#endif
	HECI_INFO("heci driver initialization successful.\n");
	return ESUCCESS;

release_hw:
	/* disable interrupts */
	device->host_hw_state = read_heci_register(device, H_CSR);
	device->host_hw_state &= ~H_IE;
	/* acknowledge interrupt and stop interupts */
	write_heci_register(device, H_CSR, device->host_hw_state);

	del_timer_sync(&device->wd_timer);


	flush_scheduled_work();

release_irq:
	free_irq(pdev->irq, device);
unmap_memory:
	if (device->mem_addr)
		iounmap(device->mem_addr);
free_device:
	kfree(device);
release_regions:
	pci_release_regions(pdev);
disable_device:
	pci_disable_device(pdev);
end:
	HECI_ERR("heci driver initialization failed.\n");
	return err;
}

/**
 * heci_remove - Device Removal Routine
 * @pdev: PCI device information struct
 *
 * heci_remove is called by the PCI subsystem to alert the driver
 * that it should release a PCI device.
 *
 * @return :
 * none;
 */
static void __devexit heci_remove(struct pci_dev *pdev)
{
	struct iamt_heci_device *device = pci_get_drvdata(pdev);
	int err = 0;

	if (heci_device != pdev)
		return;
	del_timer_sync(&device->wd_timer);
	if (device->wd_file_extension.state == HECI_FILE_CONNECTED
	    && device->wd_timeout) {
		spin_lock_bh(&device->device_lock);
		device->wd_timeout = 0;
		device->wd_due_counter = 0;
		memcpy(device->wd_data, stop_wd_params, HECI_WD_PARAMS_SIZE);
		device->stop = TRUE;
		if (device->host_buffer_is_empty &&
		    flow_control_credentials(device, &device->wd_file_extension)) {
			device->host_buffer_is_empty = FALSE;

			if (!heci_send_wd(device))
				DBG("Send stop WD  failed\n");
			else
				flow_control_reduce(device, &device->wd_file_extension);
			device->wd_pending = FALSE;

		} else {
			device->wd_pending = TRUE;
		}
		spin_unlock_bh(&device->device_lock);
		device->wd_stoped = FALSE;

		err =
		    wait_event_interruptible_timeout(device->wait_stop_wd,
						     (TRUE ==
						      device->wd_stoped),
						     10 * HZ);
		if (!device->wd_stoped)
			DBG("stop wd failed to complete.\n");
		else
			DBG("stop wd complete.\n");
	}

	heci_device = NULL;
	if (device->legacy_file_extension.status == HECI_FILE_CONNECTED) {
		device->legacy_file_extension.status = HECI_FILE_DISCONNECTING;
		heci_disconnect_host_client(device,
					    &device->legacy_file_extension);
	}
	if (device->wd_file_extension.status == HECI_FILE_CONNECTED) {
		device->wd_file_extension.status = HECI_FILE_DISCONNECTING;
		heci_disconnect_host_client(device,
					    &device->wd_file_extension);
	}
	/* remove entry if already in list */
	DBG("list del legacy and wd file list.\n");
	heci_remove_client_from_file_list(device, device->wd_file_extension.
					  host_client_id);
	heci_remove_client_from_file_list(device, device->legacy_file_extension.
					  host_client_id);
	flush_scheduled_work();
	/* disable interrupts */
	device->host_hw_state &= ~H_IE;
	/* acknowledge interrupt and stop interupts */
	write_heci_register(device, H_CSR, device->host_hw_state);
	free_irq(pdev->irq, device);
	pci_set_drvdata(pdev, NULL);

	if (device->mem_addr)
		iounmap(device->mem_addr);
	kfree(device);

	pci_release_regions(pdev);
	pci_disable_device(pdev);
}
/**
 * heci_clear_list - remove all callbacks associated with file
 * 		from heci_cb_list
 * @file: file informtion struct
 * @heci_cb_list: callbacks list
 * heci_clear_list is called to clear resources associated with file
 * when application calls close function or Ctrl-C was pressed
 *
 * @return :true if callback removed from the list, false otherwise
 */
static int heci_clear_list(struct iamt_heci_device *device, struct file *file, struct list_head *heci_cb_list) {
	struct heci_cb_private *kernel_priv_cb_pos = NULL, *kernel_priv_cb_next = NULL;
	struct file *file_temp = NULL;
	int return_status = FALSE;

	/* list all list member */
	list_for_each_entry_safe(kernel_priv_cb_pos, kernel_priv_cb_next, heci_cb_list, cb_list){
		file_temp = (struct file *)kernel_priv_cb_pos->file_object;
		/* check if list member associated with a file */
		if (file_temp == file) {
			/* remove member from the list */
			list_del(&kernel_priv_cb_pos->cb_list);
			/* check if cb equal to current legacy cb */
			if (device->legacy_current_cb == kernel_priv_cb_pos) {
				device->legacy_current_cb = NULL;
				/* send flow control to legacy client */
				heci_send_flow_control(device, &device->legacy_file_extension);
			}
			/* free all allocated buffers */
			kfree (kernel_priv_cb_pos->request_buffer.data);
			kernel_priv_cb_pos->request_buffer.data = NULL;
			kfree (kernel_priv_cb_pos->response_buffer.data);
			kernel_priv_cb_pos->response_buffer.data = NULL;
			kfree(kernel_priv_cb_pos);
			return_status = TRUE;
		}
	}
	return return_status;
}

/**
 * heci_clear_lists - remove all callbacks associated with file
 * @device: device informtion struct
 * @file: file informtion struct
 * heci_clear_lists is called to clear resources associated with file
 * when application calls close function or Ctrl-C was pressed
 *
 * @return :true if callback removed from the list, false otherwise
 */
static int heci_clear_lists(struct iamt_heci_device *device, struct file *file)
{
	int return_status = FALSE;

	/* remove callbacks associated with a file */
	heci_clear_list(device, file, &device->pthi_cmd_list.heci_cb.cb_list);
	if (heci_clear_list(device, file, &device->pthi_read_complete_list.heci_cb.cb_list))
		return_status = TRUE;
	heci_clear_list(device, file, &device->control_read_list.heci_cb.cb_list);
	if (heci_clear_list(device, file, &device->control_write_list.heci_cb.cb_list))
		return_status = TRUE;
	if (heci_clear_list(device, file, &device->write_waiting_list.heci_cb.cb_list))
		return_status = TRUE;
	if (heci_clear_list(device, file, &device->write_list.heci_cb.cb_list))
		return_status = TRUE;
	/* check if legacy_current_cb not NULL */
	if (device->legacy_current_cb && (!return_status)) {
		/* check file and legacy current cb association */
		if (device->legacy_current_cb->file_object == file) {
			/* remove cb */
			kfree (device->legacy_current_cb->request_buffer.data);
			device->legacy_current_cb->request_buffer.data = NULL;
			kfree (device->legacy_current_cb->response_buffer.data);
			device->legacy_current_cb->response_buffer.data = NULL;
			kfree(device->legacy_current_cb);
			device->legacy_current_cb = NULL;
			return_status = TRUE;
		}
	}
	return return_status;
}

/**
 * heci_open - the open function
 */
static int heci_open(struct inode *inode, struct file *file)
{
	struct heci_file_private *file_extension = NULL;
	int if_num = MINOR(inode->i_rdev);
	struct iamt_heci_device *device = NULL;
	if (!heci_device)
		return -ENODEV;
	device = pci_get_drvdata(heci_device);
	if (((if_num != LEGACY_MINOR_NUMBER)
	     && (if_num != HECI_MINOR_NUMBER)) || (!device))
		return -ENODEV;

	if (if_num != LEGACY_MINOR_NUMBER) {
		file_extension = alloc_priv(file);
		if (!file_extension)
			return -ENOMEM;
	} else {
		file->private_data =
		    (void *) &device->legacy_file_extension;
		return ESUCCESS;
	}
	spin_lock_bh(&device->device_lock);
	if (device->heci_state != HECI_ENABLED) {
		spin_unlock_bh(&device->device_lock);
		kfree(file_extension);
		file_extension = NULL;
		return -ENODEV;
	}
	if (device->open_handle_count >= MAX_OPEN_HANDLE_COUNT) {
		spin_unlock_bh(&device->device_lock);
		kfree(file_extension);
		file_extension = NULL;
		return -ENFILE;
	};
	device->open_handle_count++;
	list_add_tail(&file_extension->link, &device->file_list);
	while ((device->heci_host_clients[device->current_host_client_id / 8]
		& (1 << (device->current_host_client_id % 8))) != 0) {
		device->current_host_client_id++; /* allow overflow */
		DBG("current_host_client_id = %d\n", device->current_host_client_id);
		DBG("device->open_handle_count = %lu\n", device->open_handle_count);
	}
	DBG("current_host_client_id = %d\n", device->current_host_client_id);
	file_extension->host_client_id = device->current_host_client_id;
	device->heci_host_clients[file_extension->host_client_id / 8] |=
		    (1 << (file_extension->host_client_id % 8));
	spin_unlock_bh(&device->device_lock);
	spin_lock(&file_extension->file_lock);
	file_extension->state = HECI_FILE_INITIALIZING;
	file_extension->sm_state = 0;

	file->private_data = file_extension;
	spin_unlock(&file_extension->file_lock);

	return ESUCCESS;
}

/**
 * heci_release - the release function
 */
static int heci_release(struct inode *inode, struct file *file)
{
	int return_status = ESUCCESS;
	int if_num = MINOR(inode->i_rdev);
	struct heci_file_private *file_extension = file->private_data;
	struct heci_cb_private *kernel_priv_cb = NULL;
	struct iamt_heci_device *device = NULL;

	if (!heci_device)
		return -ENODEV;

	device = pci_get_drvdata(heci_device);
	if (((if_num != LEGACY_MINOR_NUMBER)
	     && (if_num != HECI_MINOR_NUMBER)) || (!device)
	    || (!file_extension))
		return -ENODEV;
	if (file_extension != &device->legacy_file_extension) {

		spin_lock(&file_extension->file_lock);
		if (file_extension->state == HECI_FILE_CONNECTED) {
			file_extension->state = HECI_FILE_DISCONNECTING;
			spin_unlock(&file_extension->file_lock);
			DBG("disconnecting  client host client = %d, ME client = %d\n",
					file_extension->host_client_id,
					file_extension->me_client_id);
			return_status =
			    heci_disconnect_host_client(device,
							file_extension);
			spin_lock(&file_extension->file_lock);
		}
		spin_lock_bh(&device->device_lock);
		heci_flush_queues(device, file_extension);
                DBG("remove client host client = %d, ME client = %d\n",
					file_extension->host_client_id,
					file_extension->me_client_id);
		device->heci_host_clients[file_extension->host_client_id / 8] &= ~(1 << (file_extension->host_client_id % 8));
		device->open_handle_count--;
		heci_remove_client_from_file_list(device, file_extension->host_client_id);
		spin_unlock_bh(&device->device_lock);

		/* free read cb */
		if (file_extension->read_cb != NULL) {
			spin_unlock(&file_extension->file_lock);
			kernel_priv_cb = file_extension->read_cb;
			kfree(kernel_priv_cb->response_buffer.data);
			kernel_priv_cb->response_buffer.data = NULL;
			kfree(kernel_priv_cb);
			kernel_priv_cb = NULL;

			file_extension->read_cb = NULL;
			spin_lock(&file_extension->file_lock);
		}
		spin_unlock(&file_extension->file_lock);
		kfree(file_extension);
		file->private_data = NULL;
	} else {
		spin_lock_bh(&device->device_lock);
		if (if_num != LEGACY_MINOR_NUMBER) {
			device->open_handle_count--;
		}
		if (device->legacy_file_object == file
		    && device->legacy_state != HECI_LEGACY_IDLE) {

			DBG("pthi canceled legacy state %d\n",
			    device->legacy_state);
			device->legacy_canceled = TRUE;
			if (device->legacy_state == HECI_LEGACY_READ_COMPLETE) {
				DBG("run next pthi legacy cb\n");
				run_next_legacy_cmd(device);
			}
		}

		if (heci_clear_lists(device, file)) {
		    device->legacy_state = HECI_LEGACY_IDLE;
		}
		spin_unlock_bh(&device->device_lock);
	}
	return return_status;
}


static struct heci_cb_private *find_read_list_entry(struct iamt_heci_device* device,
													struct heci_file_private *file_extension)
{
	struct heci_cb_private *kernel_priv_cb_pos = NULL, *kernel_priv_cb_next = NULL;
	struct heci_file_private *file_extension_list_temp = NULL;

	if (device->read_list.status == ESUCCESS
			&& !list_empty(&device->read_list.heci_cb.cb_list)) {
		DBG("remove read_list CB \n");
		list_for_each_entry_safe(kernel_priv_cb_pos, kernel_priv_cb_next, &device->read_list.heci_cb.cb_list, cb_list){
			file_extension_list_temp =
				(struct heci_file_private *)
				kernel_priv_cb_pos->file_private;
			if (file_extension_list_temp) {
				if ((file_extension->host_client_id == file_extension_list_temp->host_client_id)
						&& (file_extension->me_client_id == file_extension_list_temp->me_client_id))
					return kernel_priv_cb_pos;
			}
		}
	}
	return NULL;
}
/**
 * heci_read - the read client message function.
 */
static ssize_t heci_read(struct file *file, char __user * ubuf,
			 size_t length, loff_t * offset)
{
	int i;
	int return_status = ESUCCESS, err = ESUCCESS;
	int if_num = MINOR((file->f_dentry->d_inode->i_rdev));
	struct heci_file_private *file_extension = file->private_data;
	struct heci_cb_private *kernel_priv_cb_pos = NULL;
	struct heci_cb_private *kernel_priv_cb = NULL;
	struct iamt_heci_device *device = NULL;

	if (!heci_device)
		return -ENODEV;

	device = pci_get_drvdata(heci_device);
	if ((if_num != HECI_MINOR_NUMBER) || (!device) || (!file_extension))
		return -ENODEV;

	spin_lock_bh(&device->device_lock);
	if (device->heci_state != HECI_ENABLED) {
		spin_unlock_bh(&device->device_lock);
		return -ENODEV;
	}
	spin_unlock_bh(&device->device_lock);
	if (!file_extension)
		return -ENODEV;

	spin_lock(&file_extension->file_lock);
	if((file_extension->sm_state & HECI_WD_STATE_INDEPENDENCE_MSG_SENT) == 0) {
			spin_unlock(&file_extension->file_lock);
			/* Do not allow to read watchdog client */
 			for (i = 0; i < device->num_heci_me_clients; i++) {
				if (0 == memcmp(&heci_wd_guid, &device->me_clients[i].properteis.protocol_name, sizeof(struct guid))) {
					if (file_extension->me_client_id == device->me_clients[i].client_id)
						return -EBADF;
				}
			}
	} else {
		file_extension->sm_state &= ~HECI_WD_STATE_INDEPENDENCE_MSG_SENT;
		spin_unlock(&file_extension->file_lock);
	}
	if (file_extension == &device->legacy_file_extension) {
		return_status = pthi_read(device, if_num, file, ubuf, length, offset);
		goto out;
	}

	if (file_extension->read_cb && file_extension->read_cb->information > *offset) {
		kernel_priv_cb = file_extension->read_cb;
		goto copy_buffer;
	}
	else if (file_extension->read_cb && file_extension->read_cb->information > 0 &&
				file_extension->read_cb->information <= *offset) {
		kernel_priv_cb = file_extension->read_cb;
		return_status = 0;
	    goto free;
	}
	else if ((!file_extension->read_cb || file_extension->read_cb->information == 0) &&
			 *offset > 0) {
		*offset = 0;	/*Offset needs to be cleaned for contingous reads*/
		return_status = 0;
	    goto out;
	}

	spin_lock(&file_extension->read_io_lock);
	err = heci_start_read(device, if_num, file_extension);
	if (err != ESUCCESS && err != -EBUSY) {
		DBG("heci start read failure with status = %d\n", err);
		spin_unlock(&file_extension->read_io_lock);
		return_status = err;
		goto out;
	}
	if (HECI_READ_COMPLETE != file_extension->reading_state && !waitqueue_active (&file_extension->rx_wait)) {
		if (file->f_flags & O_NONBLOCK) {
			return_status = -EAGAIN;
			spin_unlock(&file_extension->read_io_lock);
			goto out;
		}
		spin_unlock(&file_extension->read_io_lock);

		if (wait_event_interruptible(file_extension->rx_wait,
			(HECI_READ_COMPLETE == file_extension->reading_state ||
			HECI_FILE_INITIALIZING == file_extension->state ||
			HECI_FILE_DISCONNECTED == file_extension->state ||
			HECI_FILE_DISCONNECTING == file_extension->state))) {
			if (signal_pending (current)) {
				return_status = -EINTR;
				goto out;
			}
			return -ERESTARTSYS;
		}

		if (HECI_FILE_INITIALIZING == file_extension->state ||
			HECI_FILE_DISCONNECTED == file_extension->state ||
			HECI_FILE_DISCONNECTING == file_extension->state) {
			return_status = -EBUSY;
			goto out;
		}
		spin_lock(&file_extension->read_io_lock);
	}

	kernel_priv_cb = file_extension->read_cb;

	if (!kernel_priv_cb) {
		spin_unlock(&file_extension->read_io_lock);
		return -ENODEV;
	}
	if (file_extension->reading_state != HECI_READ_COMPLETE) {
		spin_unlock(&file_extension->read_io_lock);
		return ESUCCESS;
	}
	spin_unlock(&file_extension->read_io_lock);
	/* now copy the data to user space */
copy_buffer:
	DBG("kernel_priv_cb->response_buffer size - %d\n", kernel_priv_cb->response_buffer.size);
	DBG("kernel_priv_cb->information - %lu\n", kernel_priv_cb->information);
	if (length == 0 || ubuf == NULL || *offset > kernel_priv_cb->information) {
		return_status = -EMSGSIZE;
		goto free;
	}

	/* length is being turncated to PAGE_SIZE, however, information size may be longer */
	length = (length < (kernel_priv_cb->information - *offset) ?
					length : (kernel_priv_cb->information - *offset));

	if (copy_to_user(ubuf, kernel_priv_cb->response_buffer.data + *offset, length)) {
		return_status = -EFAULT;
		goto free;
	}
	else {
		return_status = length;
		*offset += length;
		if ((unsigned long)*offset < kernel_priv_cb->information) {
			goto out;
		}
	}
free:
	spin_lock_bh(&device->device_lock);
    kernel_priv_cb_pos = find_read_list_entry(device, file_extension);
	/* Remove entry from read list */
	if (kernel_priv_cb_pos != NULL)
		list_del(&kernel_priv_cb_pos->cb_list);
	spin_unlock_bh(&device->device_lock);
	kfree(kernel_priv_cb->response_buffer.data);
	kernel_priv_cb->response_buffer.data = NULL;
	kfree(kernel_priv_cb);
	kernel_priv_cb = NULL;
	spin_lock(&file_extension->read_io_lock);
	file_extension->reading_state = HECI_IDLE;
	file_extension->read_cb = NULL;
	file_extension->read_pending = FALSE;
	spin_unlock(&file_extension->read_io_lock);
out:	DBG("end heci read return_status= %d\n", return_status);
	return return_status;
}

/**
 * heci_write - the write function.
 */
static ssize_t heci_write(struct file *file, const char __user * ubuf,
			  size_t length, loff_t * offset)
{
	int return_status = ESUCCESS;
	__u8 i;
	int if_num = MINOR((file->f_dentry->d_inode->i_rdev));
	struct heci_file_private *file_extension = file->private_data;
	struct heci_cb_private *priv_write_cb = NULL;
	struct heci_message_header heci_header;
	struct iamt_heci_device *device = NULL;
	unsigned long currtime = get_seconds();

	if (!heci_device)
		return -ENODEV;
	device = pci_get_drvdata(heci_device);

	if ((if_num != HECI_MINOR_NUMBER) || (!device) || (!file_extension))
		return -ENODEV;
	spin_lock_bh(&device->device_lock);

	if (device->heci_state != HECI_ENABLED) {
		spin_unlock_bh(&device->device_lock);
		return -ENODEV;
	}
	if (file_extension == &device->legacy_file_extension) {
		priv_write_cb = find_pthi_read_list_entry(device, file, file_extension);
		if ((priv_write_cb && currtime - priv_write_cb->read_time > LEGACY_READ_TIMER) ||
			(priv_write_cb && file_extension->reading_state == HECI_READ_COMPLETE)) {
				*offset = 0;
				list_del(&priv_write_cb->cb_list);
				kfree(priv_write_cb->request_buffer.data);
				kfree(priv_write_cb->response_buffer.data);
				kfree(priv_write_cb);
		}
	}

	//free entry used in read 
	if (file_extension->reading_state == HECI_READ_COMPLETE)
	{
		*offset = 0;
		priv_write_cb = find_read_list_entry(device, file_extension);
		if ( priv_write_cb != NULL) {
			list_del(&priv_write_cb->cb_list);
			kfree(priv_write_cb->response_buffer.data);
			priv_write_cb->response_buffer.data = NULL;
			kfree(priv_write_cb);
			priv_write_cb = NULL;
			spin_lock(&file_extension->read_io_lock);
			file_extension->reading_state = HECI_IDLE;
			file_extension->read_cb = NULL;
			file_extension->read_pending = FALSE;
			spin_unlock(&file_extension->read_io_lock);
		}
	}
	else if (file_extension->reading_state == HECI_IDLE &&  
		file_extension->read_pending == FALSE){
		*offset = 0;
	}

	spin_unlock_bh(&device->device_lock);

	priv_write_cb = kmalloc(sizeof(struct heci_cb_private), GFP_KERNEL);
	if (!priv_write_cb)
		return -ENOMEM;
	spin_lock(&file_extension->file_lock);
	priv_write_cb->request_buffer.data = NULL;
	priv_write_cb->response_buffer.data = NULL;
	priv_write_cb->file_object = file;
	priv_write_cb->file_private = file_extension;
	spin_unlock(&file_extension->file_lock);
	priv_write_cb->request_buffer.data = kmalloc(length, GFP_KERNEL);
	if (!priv_write_cb->request_buffer.data) {
		kfree(priv_write_cb);
		return -ENOMEM;
	}
	DBG("length =%d\n", (int) length);

	if (copy_from_user(priv_write_cb->request_buffer.data,
		ubuf, length)) {
		return_status = -EFAULT;
		goto fail;
	}
	
	spin_lock(&file_extension->file_lock);
	file_extension->sm_state = 0;
	if (length == 4 &&
		((memcmp(heci_wd_state_independence_msg[0], ubuf, 4) == 0) ||
		 (memcmp(heci_wd_state_independence_msg[1], ubuf, 4) == 0))) {
		file_extension->sm_state |= HECI_WD_STATE_INDEPENDENCE_MSG_SENT;
   }
	spin_unlock(&file_extension->file_lock);

	INIT_LIST_HEAD(&priv_write_cb->cb_list);
	if (file_extension == &device->legacy_file_extension) {
		priv_write_cb->response_buffer.data =
		    kmalloc(LEGACY_MTU, GFP_KERNEL);
		if (!priv_write_cb->response_buffer.data) {
			return_status = -ENOMEM;
			goto fail;
		}
		spin_lock_bh(&device->device_lock);
              if (device->heci_state != HECI_ENABLED) {
		    spin_unlock_bh(&device->device_lock);
		    return_status = -ENODEV;
		    goto fail;
	       }
		for (i = 0; i < device->num_heci_me_clients; i++) {
			if (device->me_clients[i].client_id ==
			    device->legacy_file_extension.me_client_id)
				break;
		}

		BUG_ON(device->me_clients[i].client_id !=
		       file_extension->me_client_id);
		if ((i == device->num_heci_me_clients)
		    || (device->me_clients[i].client_id != device->legacy_file_extension.me_client_id)) {
			spin_unlock_bh(&device->device_lock);
			return_status = -ENODEV;
			goto fail;
		} else if (length > device->me_clients[i].properteis.max_message_length || length <= 0) {
			spin_unlock_bh(&device->device_lock);
			return_status = -EMSGSIZE;
			goto fail;
		}


		priv_write_cb->response_buffer.size = LEGACY_MTU;
		priv_write_cb->major_file_operations = HECI_IOCTL;
		priv_write_cb->information = 0;
		priv_write_cb->request_buffer.size = length;
		if (device->legacy_file_extension.state != HECI_FILE_CONNECTED) {
			spin_unlock_bh(&device->device_lock);
			return_status = -ENODEV;
			goto fail;
		}

		if (!list_empty(&device->pthi_cmd_list.heci_cb.cb_list)
		    || device->legacy_state != HECI_LEGACY_IDLE) {
			DBG("pthi_state = %d\n", (int) device->legacy_state);
			DBG("add PTHI cb to pthi cmd waiting list\n");
			list_add_tail(&priv_write_cb->cb_list,
				      &device->pthi_cmd_list.heci_cb.
				      cb_list);
			return_status = length;
		} else {
			DBG("call pthi write");
			return_status = pthi_write(device, priv_write_cb);

			if (ESUCCESS != return_status) {
				DBG("pthi write failed with status = %d\n",
				    return_status);
				spin_unlock_bh(&device->device_lock);
				goto fail;
			};
			return_status = length;
		}
		spin_unlock_bh(&device->device_lock);
		return return_status;
	}

	priv_write_cb->major_file_operations = HECI_WRITE;
	/* make sure information is zero before we start */

	priv_write_cb->information = 0;
	priv_write_cb->request_buffer.size = length;

	spin_lock(&file_extension->write_io_lock);
	DBG("host client = %d, ME client = %d\n",
	    file_extension->host_client_id, file_extension->me_client_id);
	if (file_extension->state != HECI_FILE_CONNECTED) {
		return_status = -ENODEV;
		DBG("host client = %d,  is not connected to ME client = %d",
				file_extension->host_client_id,
				file_extension->me_client_id);

		goto unlock;
	}
	for (i = 0; i < device->num_heci_me_clients; i++) {
		if (device->me_clients[i].client_id ==
		    file_extension->me_client_id)
			break;
	}
	BUG_ON(device->me_clients[i].client_id != file_extension->me_client_id);
	if (i == device->num_heci_me_clients) {
		return_status = -ENODEV;
		goto unlock;
	}
	if (length > device->me_clients[i].properteis.max_message_length
			|| length <= 0) {
		return_status = -EINVAL;
		goto unlock;
	}
	priv_write_cb->file_private = file_extension;

	spin_lock_bh(&device->device_lock);
	if (flow_control_credentials(device, file_extension) &&
		device->host_buffer_is_empty) {
		spin_unlock_bh(&device->device_lock);
		device->host_buffer_is_empty = FALSE;
		if (length > ((((device->host_hw_state & H_CBD) >> 24) * sizeof(__u32)) - sizeof(struct heci_message_header))) {
			heci_header.length = (((device->host_hw_state & H_CBD) >> 24)
					* sizeof(__u32)) - sizeof(struct heci_message_header);
			heci_header.message_complete = 0;
		} else {
			heci_header.length = length;
			heci_header.message_complete = 1;
		}
		heci_header.host_address = file_extension->host_client_id;
		heci_header.me_address = file_extension->me_client_id;
		heci_header.reserved = 0;
		DBG("call heci_write_message header=%08x.\n",
		    *((__u32 *) & heci_header));
		spin_unlock(&file_extension->write_io_lock);
		/*  protect heci low level write */
		spin_lock_bh(&device->device_lock);
		if (!heci_write_message(device, &heci_header, (unsigned char *) (priv_write_cb->request_buffer.data),
					heci_header.length)) {
			spin_unlock_bh(&device->device_lock);
			kfree(priv_write_cb->request_buffer.data);
			priv_write_cb->request_buffer.data = NULL;
			kfree(priv_write_cb);
			return_status = -ENODEV;
			priv_write_cb->information = 0;
			return return_status;
		}
		file_extension->writing_state = HECI_WRITING;
		priv_write_cb->information = heci_header.length;
		if (heci_header.message_complete) {
			flow_control_reduce(device, file_extension);
			list_add_tail(&priv_write_cb->cb_list,
				      &device->write_waiting_list.heci_cb.
				      cb_list);
		} else {
			list_add_tail(&priv_write_cb->cb_list,
				      &device->write_list.heci_cb.cb_list);
		}
		spin_unlock_bh(&device->device_lock);

	} else {

		spin_unlock_bh(&device->device_lock);
		priv_write_cb->information = 0;
		file_extension->writing_state = HECI_WRITING;
		spin_unlock(&file_extension->write_io_lock);
		list_add_tail(&priv_write_cb->cb_list,
			      &device->write_list.heci_cb.cb_list);
	}
	return length;

unlock:
	spin_unlock(&file_extension->write_io_lock);
fail:
	kfree(priv_write_cb->request_buffer.data);
	priv_write_cb->request_buffer.data = NULL;
	kfree(priv_write_cb->response_buffer.data);
	priv_write_cb->response_buffer.data = NULL;
	kfree(priv_write_cb);
	return return_status;

}

/**
 * heci_ioctl - the IOCTL function
 */
static int heci_ioctl(struct inode *inode, struct file *file,
		      unsigned int cmd, unsigned long data)
{

	int return_status = ESUCCESS;
	int if_num = MINOR(inode->i_rdev);
	struct heci_file_private *file_extension = file->private_data;
	struct heci_message_data *u_msg = (struct heci_message_data *) data;	/* in user space */
	struct heci_message_data k_msg;	/* all in kernel on the stack */
	struct iamt_heci_device *device = NULL;


       if (!capable(CAP_SYS_ADMIN))
                       return -EPERM;
	if (!heci_device)
		return -ENODEV;
	device = pci_get_drvdata(heci_device);
	if (((if_num != LEGACY_MINOR_NUMBER)
	     && (if_num != HECI_MINOR_NUMBER)) || (!device)
	    || (!file_extension))
		return -ENODEV;
	if (device->heci_state != HECI_ENABLED)
		return -ENODEV;
	/* first copy from user all data needed */
	if (copy_from_user(&k_msg, u_msg, sizeof(k_msg))) {
		DBG("first copy from user all data needed filed\n");
		return -EFAULT;
	}
	DBG("user message size is %d\n", k_msg.size);

	switch (cmd) {
	case IOCTL_HECI_GET_VERSION:
		DBG(": IOCTL_HECI_GET_VERSION\n");
		return_status =
		    heci_ioctl_get_version(device, if_num, u_msg, k_msg,
					   file_extension);
		break;
	case IOCTL_HECI_CONNECT_CLIENT:
		DBG(": IOCTL_HECI_CONNECT_CLIENT.\n");
		return_status =
		    heci_ioctl_connect_client(device, if_num, u_msg, k_msg,
					      file);
		break;
	case IOCTL_HECI_WD:
		DBG(": IOCTL_HECI_WD.\n");
		return_status =
		    heci_ioctl_wd(device, if_num, k_msg, file_extension);
		break;
	case IOCTL_HECI_BYPASS_WD:
		DBG(":IOCTL_HECI_BYPASS_WD.\n");
		return_status =
		    heci_ioctl_bypass_wd(device,if_num,k_msg,file_extension);
		break;
	case IAMT_KCS_SEND_MESSAGE_COMMAND:
		DBG(": IAMT_KCS_SEND_MESSAGE_COMMAND.\n");
		return_status =
		    legacy_ioctl_send_message(device, if_num, k_msg, file);
		break;
	case IAMT_KCS_RECEIVE_MESSAGE_COMMAND:
		DBG(": IAMT_KCS_RECEIVE_MESSAGE_COMMAND.\n");
		return_status =
		    legacy_ioctl_receive_message(device, if_num, u_msg,
						 k_msg, file);
		break;

	default:
		return_status = -EINVAL;
		break;
	}
	return return_status;
}

/**
 * heci_legacy_poll - the poll function
 */
static unsigned int heci_legacy_poll(struct file *file, poll_table * wait)
{
	int if_num = MINOR((file->f_dentry->d_inode->i_rdev));
	unsigned int mask = 0;
	struct iamt_heci_device *device = NULL;
	struct heci_file_private *file_extension = file->private_data;


	if (!heci_device || !file_extension)
		return mask;

	device = pci_get_drvdata(heci_device);

	if ((if_num != LEGACY_MINOR_NUMBER) || (!device))
		return mask;

	spin_lock_bh(&device->device_lock);
	if (device->heci_state != HECI_ENABLED){
		spin_unlock_bh(&device->device_lock);
		return mask;
	}
	spin_unlock_bh(&device->device_lock);
	if (file_extension == &device->legacy_file_extension) {

		poll_wait(file, &device->legacy_file_extension.wait, wait);
		spin_lock(&device->legacy_file_extension.file_lock);
		if (device->legacy_state == HECI_LEGACY_READ_COMPLETE
		    && device->legacy_file_object == file) {
			mask |= (POLLIN | POLLRDNORM);
			spin_lock_bh(&device->device_lock);
			DBG("run next pthi legacy cb\n");
			run_next_legacy_cmd(device);
			spin_unlock_bh(&device->device_lock);
		}
		spin_unlock(&device->legacy_file_extension.file_lock);
	}
	return mask;
}

/**
 * heci_poll - the poll function
 */
static unsigned int heci_poll(struct file *file, poll_table * wait)
{
	int if_num = MINOR((file->f_dentry->d_inode->i_rdev));
	unsigned int mask = 0;
	struct heci_file_private *file_extension = file->private_data;
	struct iamt_heci_device *device = NULL;

	if (!heci_device)
		return mask;


	device = pci_get_drvdata(heci_device);

	if ((if_num != HECI_MINOR_NUMBER) || (!device)
	    || (!file_extension))
		return mask;

	spin_lock_bh(&device->device_lock);
	if (device->heci_state != HECI_ENABLED){
		spin_unlock_bh(&device->device_lock);
		return mask;

	}
	spin_unlock_bh(&device->device_lock);
	if (file_extension == &device->legacy_file_extension) {


		poll_wait(file, &device->legacy_file_extension.wait, wait);
		spin_lock(&device->legacy_file_extension.file_lock);
		if (device->legacy_state == HECI_LEGACY_READ_COMPLETE
		    && device->legacy_file_object == file) {
			mask |= (POLLIN | POLLRDNORM);
			spin_lock_bh(&device->device_lock);
			DBG("run next pthi cb");
			run_next_legacy_cmd(device);
			spin_unlock_bh(&device->device_lock);
		}
		spin_unlock(&device->legacy_file_extension.file_lock);

	} else{
		poll_wait(file, &file_extension->tx_wait, wait);
		spin_lock(&file_extension->write_io_lock);
		if (HECI_WRITE_COMPLETE == file_extension->writing_state)
			mask |= (POLLIN | POLLRDNORM);
		spin_unlock(&file_extension->write_io_lock);
	}
	return mask;
}

#ifdef CONFIG_PM
static int heci_suspend(struct pci_dev* pdev, pm_message_t state)
{
	struct iamt_heci_device *device = pci_get_drvdata(pdev);
	int err = 0;

	//Stop watchdog if exists
	del_timer_sync(&device->wd_timer);
	if (device->wd_file_extension.state == HECI_FILE_CONNECTED
		&& device->wd_timeout) {
			spin_lock_bh(&device->device_lock);
			g_sus_wd_timeout = device->wd_timeout;
			device->wd_timeout = 0;
			device->wd_due_counter = 0;
			memcpy(device->wd_data, stop_wd_params, HECI_WD_PARAMS_SIZE);
			device->stop = TRUE;
			if (device->host_buffer_is_empty &&
				flow_control_credentials(device, &device->wd_file_extension)) {
					device->host_buffer_is_empty = FALSE;
			
					if (!heci_send_wd(device))
						DBG("Send stop WD  failed\n");
			        else
						flow_control_reduce(device, &device->wd_file_extension);
					device->wd_pending = FALSE;
			} else {
				device->wd_pending = TRUE;
			}
			spin_unlock_bh(&device->device_lock);
			device->wd_stoped = FALSE;

			err =
				wait_event_interruptible_timeout(device->wait_stop_wd,
										(TRUE == device->wd_stoped), 10 * HZ);
			if (!device->wd_stoped)
				DBG("stop wd failed to complete.\n");
			else {
				DBG("stop wd complete %d.\n", err);
				err = 0;
			}
	}
	//Set new heci state
	spin_lock_bh(&device->device_lock);
	if (device->heci_state == HECI_ENABLED || 
			device->heci_state == HECI_RECOVERING_FROM_RESET) {
		device->heci_state = HECI_POWER_DOWN;
		heci_reset(device, FALSE);
	}
	spin_unlock_bh(&device->device_lock);

	pci_save_state(pdev);


	pci_disable_device(pdev);
	free_irq(pdev->irq, device);

	pci_set_power_state(pdev, PCI_D3hot);

	return err;
}

static int heci_resume(struct pci_dev* pdev)
{
	struct iamt_heci_device *device = NULL;
	int err = 0;

	pci_set_power_state(pdev, PCI_D0);
	pci_restore_state(pdev);

	device = pci_get_drvdata(pdev);
	if (!device) {
		return -ENODEV;
	}

	/* request and enable interrupt   */
	device->irq = pdev->irq;
	err = request_irq(device->irq, heci_isr_interrupt, IRQF_SHARED,
			heci_driver_name, device);
	if (err) {
		HECI_ERR("Request_irq failure. irq = %d \n", device->irq);
		return err;
	}

	spin_lock_bh(&device->device_lock);
	device->heci_state = HECI_POWER_UP;
	heci_reset(device, TRUE);	
	spin_unlock_bh(&device->device_lock);

	//Start watchdog if stopped in suspend
	if (g_sus_wd_timeout != 0) {
		device->wd_timeout = g_sus_wd_timeout;

		memcpy(device->wd_data, start_wd_params, HECI_WD_PARAMS_SIZE);
		memcpy(device->wd_data + HECI_WD_PARAMS_SIZE, &device->wd_timeout, 
				sizeof(__u16));
		device->wd_due_counter = 1;

		if (device->wd_timeout)
			mod_timer(&device->wd_timer, jiffies);
		g_sus_wd_timeout = 0;
	}
	return err;
}
#endif
MODULE_AUTHOR("Intel Corporation"); /* FIXME: Add email address here */
MODULE_DESCRIPTION("Intel(R) AMT Management Interface");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_VERSION(DRIVER_VERSION);
