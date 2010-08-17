/*
 *  tusbd/usbdevice.c
 *
 *  Copyright (C) 2007-2008 IncentivesPro
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifdef _USBD_ENABLE_STUB_

#include "usbd.h"
#include "../public/filter.h"
#include <linux/smp_lock.h>

int g_autosharing = 0;

static struct list_head s_device_list;
static spinlock_t s_device_list_lock;

///////////////////////////////////////////////////////////////////////////////
// USB device driver implementation

/* table of devices that work with this driver */
static struct usb_device_id usbd_device_table[] =
{
//	{ USB_DEVICE(0x0000, 0x0000) },
	{ .driver_info = 1 },
	{}                 /* Terminating entry */
};

MODULE_DEVICE_TABLE(usb, usbd_device_table);

static int usbd_probe(struct usb_interface *intf, const struct usb_device_id *id);
static void usbd_disconnect(struct usb_interface *intf);
static int usbd_pre_reset(struct usb_interface *iface);
static int usbd_post_reset(struct usb_interface *iface);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20)
int usbd_notify(struct notifier_block *self, unsigned long action, void *dev);

struct notifier_block usbd_nb = {
	.notifier_call = 	usbd_notify,
};
#endif

struct usb_driver usbd_driver = 
{
	.name = USBD_STUB_DRIVER_NAME,
	.id_table = usbd_device_table,
	.probe = usbd_probe,
	.disconnect = usbd_disconnect,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18)
	.pre_reset = usbd_pre_reset,
	.post_reset = usbd_post_reset,
#endif
};

void usbd_get_class_information(struct usb_device *udev, u8 *pclass, u8 *psubclass, u8 *pprotocol)
{
	int i, interface = -1;

	if( pclass ) *pclass = 0;
	if( psubclass ) *psubclass = 0;
	if( pprotocol ) *pprotocol = 0;

	if( udev->actconfig && udev->actconfig->desc.bNumInterfaces > 1 )
	{
		// We have multiple interfaces. Is this a composite device?

		// Generic composite device
		if( udev->descriptor.bDeviceClass == 0 ) 
			return;

		// IAD composite device
		if( (udev->descriptor.bDeviceClass == 0xEF) && 
			(udev->descriptor.bDeviceSubClass == 0x02) && 
			(udev->descriptor.bDeviceProtocol == 0x01) )
			return;
	}

	if( pclass ) *pclass = udev->descriptor.bDeviceClass;
	if( psubclass ) *psubclass = udev->descriptor.bDeviceSubClass;
	if( pprotocol ) *pprotocol = udev->descriptor.bDeviceProtocol;

	if( !udev->actconfig ) 
		return;

	// if bDeviceClass is non-zero - we have a valid class, return.
	// if bDeviceClass is zero - maybe a compiste device - lookup class information in the interface descriptor
	if( udev->descriptor.bDeviceClass )
		return;  

	// interfaces are stored in no particular order, so loop to 
	// locate an interface with the lowest number
	for(i=0; i<USB_MAXINTERFACES; i++)
	{
		if(udev->actconfig->interface[i] && udev->actconfig->interface[i]->cur_altsetting)
		{
			if(interface < 0 || 
				udev->actconfig->interface[i]->cur_altsetting->desc.bInterfaceNumber < 
				udev->actconfig->interface[interface]->cur_altsetting->desc.bInterfaceNumber)
			{
				interface = i;
			}
		}
	}
	
	if(interface >= 0)
	{
		if( pclass ) *pclass = udev->actconfig->interface[interface]->cur_altsetting->desc.bInterfaceClass;
		if( psubclass ) *psubclass = udev->actconfig->interface[interface]->cur_altsetting->desc.bInterfaceSubClass;
		if( pprotocol ) *pprotocol = udev->actconfig->interface[interface]->cur_altsetting->desc.bInterfaceProtocol;
	}
}

int usbd_probe(struct usb_interface *intf, const struct usb_device_id *id)
{
	int result = -ENODEV;

	TRACE("usbd_probe: ++\n");

	do
	{
		struct usb_device *udev = interface_to_usbdev(intf);
		struct usbdevice_descriptor *desc;
		struct waitable_queue_entry *pqentry;
		u16 vid,pid, rev;
		u8 class, subclass, protocol;
		unsigned long flags;

		if( strcmp(udev->bus->controller->driver->name, USBD_VHCI_DRIVER_NAME) == 0 )
		{
			TRACE("usbd_probe: our virtual device!\n");
			break;
		}

		vid = le16_to_cpu(udev->descriptor.idVendor);
		pid = le16_to_cpu(udev->descriptor.idProduct);
		rev = le16_to_cpu(udev->descriptor.bcdDevice);

		usbd_get_class_information(udev, &class, &subclass, &protocol);

		TRACE("usbd_probe: new interface vid=%04X pid=%04X bus_id=\"%s\" cur_interface=%u total_interfaces=%u\n",
				vid, 
				pid, 
				get_busid(&udev->dev), 
				intf->cur_altsetting->desc.bInterfaceNumber, 
				udev->actconfig->desc.bNumInterfaces);

		// check for exclusions
		usbd_wq_lock(&g_exclusions, flags);
		pqentry = usbd_wq_get_first(&g_exclusions);
		while(pqentry)
		{
			struct usbd_exclusion *pexclusion;

			pexclusion = (struct usbd_exclusion *)pqentry->data;

			if( pexclusion->vid == vid &&
				pexclusion->pid == pid &&
				!strcmp(pexclusion->bus_id, get_busid(&udev->dev)))
			{
				break;
			}
			pqentry = usbd_wq_get_next(&g_exclusions, pqentry);
		}
		usbd_wq_unlock(&g_exclusions, flags);

		if( pqentry ) 
		{
			TRACE("usbd_probe: device is excluded\n");
			break;
		}

		// check if this device is already shared
		desc = usbd_usbdevice_find1(
				vid, 
				pid, 
				(char*)get_busid(&udev->dev)); // device will be dereferenced in disconnect handler

		// if device is not shared and we are in autosharing mode - create a device entry for this device
		if( desc == NULL && g_autosharing && udev->descriptor.bDeviceClass != 0x09/*hub*/ &&
			usbd_is_autosharing_allowed(
				vid, pid, rev, 
				class, subclass, protocol,
				NULL))
		{
			TRACE("usbd_probe: autosharing this device\n");

			if( (desc=usbd_usbdevice_create()) )
			{
				desc->vid = vid;
				desc->pid = pid;
				strncpy(desc->bus_id, get_busid(&udev->dev), BUS_ID_SIZE);

				// remove reference that was added by usbd_usbdevice_create()
				// this will effectively free the autoshared device entry when device is disconnected
				//usbd_usbdevice_dereference(desc);
			}
		}

		if( desc )
		{
			if( usbd_usbdevice_change_state(desc, eDeviceInit) == 0 )
			{
				desc->total_interfaces = udev->actconfig ? udev->actconfig->desc.bNumInterfaces : 0;
				desc->is_ehci = (strcmp(udev->bus->controller->driver->name, "ehci_hcd") == 0);

				spin_lock_irqsave(&desc->pudev_lock,flags);

				if( desc->pudev ) 
					usb_put_dev(desc->pudev);

				desc->pudev = usb_get_dev(udev);

				spin_unlock_irqrestore(&desc->pudev_lock,flags);
			}

			if( ++desc->init_interfaces == desc->total_interfaces )
			{
				usbd_usbdevice_change_state(desc, eDeviceReady);
			}

			TRACE("usbd_probe: desc->init_interfaces=%d desc->total_interfaces=%d\n", desc->init_interfaces,desc->total_interfaces);

			usb_set_intfdata(intf, desc);

			result = 0;
			TRACE("usbd_probe: our device!\n");
		} else
		{
			TRACE("usbd_probe: NOT our device!\n");
		}

	} while(0);

	if( result != 0 ) 
	{
		/* something prevented us from registering this driver */
		TRACE("usbd_probe: error %d\n", result);
	}

	TRACE("usbd_probe: --\n");
	return result;
}

static void usbd_disconnect(struct usb_interface *intf)
{
	struct usbdevice_descriptor *desc;

	TRACE("usbd_disconnect: ++\n");
	
	/* prevent usbd_open() from racing usbd_disconnect() */
	lock_kernel();

	// delete context
	desc = usb_get_intfdata(intf);
	usb_set_intfdata(intf, NULL);

	unlock_kernel();

	if( desc )
	{
		usbd_usbdevice_change_state(desc, eDeviceInit);

		if( --desc->init_interfaces == 0 )
		{
			unsigned long flags;

			usbd_usbdevice_change_state(desc, eDeviceNotReady);

			spin_lock_irqsave(&desc->pudev_lock,flags);

			if( desc->pudev ) 
				usb_put_dev(desc->pudev);

			desc->pudev = NULL;

			spin_unlock_irqrestore(&desc->pudev_lock,flags);
		}

		TRACE("usbd_disconnect: desc->init_interfaces=%d desc->total_interfaces=%d\n", desc->init_interfaces,desc->total_interfaces);

		usbd_usbdevice_dereference(desc); // remove reference that was added in probe()
	}

	TRACE("usbd_disconnect: --\n");
}


static int usbd_pre_reset(struct usb_interface *iface)
{
	return 0;
}

static int usbd_post_reset(struct usb_interface *iface)
{
	return 0;
}

///////////////////////////////////////////////////////////////////////////////
// USB device notification

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20)

int usbd_notify_add(struct device *pdev)
{
	struct usb_interface *intf = to_usb_interface(pdev);

	TRACE("usbd_notify_add: ++ bus_id=\"%s\"\n", get_busid(pdev));

	if( !strchr(get_busid(pdev), ':') )
	{
		TRACE("usbd_notify_add: -- not a usb interface\n");
		return 0;
	}

	if( usbd_probe(intf, NULL) == 0 )
	{
		int result;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
		int state_in_sysfs;
#endif

		TRACE("usbd_notify_add: claiming interface\n");

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
		state_in_sysfs = pdev->kobj.state_in_sysfs;
		pdev->kobj.state_in_sysfs = 0;
#endif

		result= usb_driver_claim_interface(&usbd_driver, intf, usb_get_intfdata(intf));

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
		pdev->kobj.state_in_sysfs = state_in_sysfs;
#endif

		if (result < 0)
		{
			TRACE("usbd_notify_add: failed to claim interface\n");
			usbd_disconnect(intf);
		}		
	}

	TRACE("usbd_notify_add: --\n");
	return 0;
}

void usbd_notify_remove(struct device *pdev)
{
	// not required yet
}

int usbd_notify(struct notifier_block *self, unsigned long action, void *dev)
{
	struct device *pdev = dev;

	if(!dev) return NOTIFY_OK;

	switch (action) 
	{
		case BUS_NOTIFY_ADD_DEVICE:
			if( usbd_notify_add(pdev) )
				return NOTIFY_BAD;
			break;
		case BUS_NOTIFY_DEL_DEVICE:
			usbd_notify_remove(pdev);
			break;
	}
	return NOTIFY_OK;
}
#endif // LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20)


///////////////////////////////////////////////////////////////////////////////
// USB device object implementation

int usbd_usbdevice_init_module(void)
{
	int result = 0;

	INIT_LIST_HEAD(&s_device_list);
	spin_lock_init(&s_device_list_lock);

	result = usb_register(&usbd_driver);

	if( result != 0 )
	{
		TRACE("usbd_usbdevice_init_module: usb_register failed. Error number %d\n", result);
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20)
	result = bus_register_notifier(usbd_driver.drvwrap.driver.bus, &usbd_nb);
	if( result != 0 )
	{
		TRACE("usbd_init: bus_register_notifier failed. Error number %d\n", result);
	}
#endif
	return result;
}

void usbd_usbdevice_deinit_module(void)
{
	usb_deregister(&usbd_driver);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20)
	bus_unregister_notifier(usbd_driver.drvwrap.driver.bus, &usbd_nb);
#endif
}

struct usbdevice_descriptor *usbd_usbdevice_create(void)
{
	struct usbdevice_descriptor *pdesc;

   	pdesc = kmalloc(sizeof(*pdesc), GFP_KERNEL);

   	if( pdesc )
   	{
	  	memset(pdesc, 0, sizeof(*pdesc));

	  	pdesc->pminor = usbd_md_alloc_descriptor(pdesc);

	  	if( pdesc->pminor )
	  	{
		    usbd_cdev_alloc_minor(pdesc->pminor);

			if( pdesc->pminor->minor == -1 )
			{
				TRACE("usbd_usbdevice_create: usbd_cdev_alloc_minor failed.\n");
			} else
			{
				usbd_wq_init(&pdesc->queue_completed_unrb);
				usbd_wq_init(&pdesc->queue_pending_unrb);

				kref_init(&pdesc->reference);
				spin_lock_init(&pdesc->pudev_lock);
		        init_waitqueue_head(&pdesc->config_change_wait);

				pdesc->handle = -1;
				pdesc->vid = -1;
				pdesc->pid = -1;
				pdesc->state = eDeviceNotReady;
				pdesc->pudev = NULL;
				pdesc->total_interfaces = 0;
				pdesc->init_interfaces = 0;

				usbd_usbdevice_reference(pdesc);

				spin_lock(&s_device_list_lock);
				list_add(&pdesc->list, &s_device_list);
				spin_unlock(&s_device_list_lock);

				return pdesc;
			}
		}
		kfree(pdesc);
 	}

	return NULL;
}

void usbd_usbdevice_free(struct kref* pkref)
{
	struct usbdevice_descriptor *pdesc = container_of(pkref, struct usbdevice_descriptor, reference);

	TRACE("usbd_usbdevice_free++\n");

	spin_lock(&s_device_list_lock);
	if( pdesc->list.next != LIST_POISON1 ) list_del(&pdesc->list);
	spin_unlock(&s_device_list_lock);

	if( pdesc->pminor )
	{
		usbd_cdev_free_minor(pdesc->pminor->minor);
		usbd_md_free_descriptor(pdesc->pminor);
	}

	usbd_wq_deinit(&pdesc->queue_completed_unrb);
	usbd_wq_deinit(&pdesc->queue_pending_unrb);

	kfree(pdesc);
	TRACE("usbd_usbdevice_free--\n");
}

void usbd_usbdevice_destroy(struct usbdevice_descriptor *pdesc)
{
	if(!pdesc) return;

	spin_lock(&s_device_list_lock);
	list_del(&pdesc->list);
	spin_unlock(&s_device_list_lock);

	usbd_usbdevice_dereference(pdesc);

	TRACE("usbd_usbdevice_destroy\n");
}

struct usbdevice_descriptor *usbd_usbdevice_find1(unsigned short vid, unsigned short pid, char *bus_id)
{
	struct usbdevice_descriptor *pdesc = NULL;
	struct list_head *entry;

	TRACE("usbd_usbdevice_find1: ++\n");

	spin_lock(&s_device_list_lock);
	list_for_each(entry, &s_device_list)
	{
		pdesc = (struct usbdevice_descriptor *)entry;

        if( pdesc->vid == vid && pdesc->pid == pid && !strcmp(pdesc->bus_id, bus_id) )
        {
			usbd_usbdevice_reference(pdesc);
			spin_unlock(&s_device_list_lock);
			TRACE("usbd_usbdevice_find1: -- found!\n");
            return pdesc;
		}
	}
	spin_unlock(&s_device_list_lock);

	TRACE("usbd_usbdevice_find1: -- not found\n");
	return NULL;
}

struct usbdevice_descriptor *usbd_usbdevice_find2(unsigned long handle)
{
	struct list_head *entry;
	struct usbdevice_descriptor *pdesc = NULL;

	spin_lock(&s_device_list_lock);
	list_for_each(entry, &s_device_list)
	{
		pdesc = (struct usbdevice_descriptor *)entry;

		if( pdesc->handle == handle )
        {
			usbd_usbdevice_reference(pdesc);
			spin_unlock(&s_device_list_lock);
            return pdesc;
		}
	}
	spin_unlock(&s_device_list_lock);

	return NULL;
}

struct usbdevice_descriptor *usbd_usbdevice_find3(int minor)
{
	struct list_head *entry;
	struct usbdevice_descriptor *pdesc = NULL;

	spin_lock(&s_device_list_lock);
	list_for_each(entry, &s_device_list)
	{
		pdesc = (struct usbdevice_descriptor *)entry;

		if( pdesc->pminor->minor == minor )
        {
			usbd_usbdevice_reference(pdesc);
			spin_unlock(&s_device_list_lock);
            return pdesc;
		}
	}
	spin_unlock(&s_device_list_lock);

	return NULL;
}

int usbd_usbdevice_change_state(struct usbdevice_descriptor *pdesc, eDeviceState newstate)
{
	if( !pdesc ) return -1;

	if( pdesc->state != newstate ) 
	{
		pdesc->state = newstate;

		if(pdesc->state == eDeviceReady)
		{
			pdesc->config_changed = 1;
			wake_up(&pdesc->config_change_wait);
		}

		return usbd_usbdevice_refresh_state(pdesc);
	}

	return -1;
}

int usbd_usbdevice_change_configuration(struct usbdevice_descriptor *pdesc, struct usb_device *pudev, int configuration, int timeoutms)
{
	int result;
	char s[50];

	if( !pdesc ) 
		return -1;

	if( configuration <= 0 || configuration > 255 ) 
		return -1;

 	if( pudev->actconfig && 
		pudev->actconfig->desc.bConfigurationValue == configuration)
		return 0;

    TRACE("usbd_usbdevice_change_configuration: from %d to %d\n",
		pudev->actconfig ? pudev->actconfig->desc.bConfigurationValue : 0,
		configuration);

	pdesc->config_changed = 0;

	sprintf(s, "configuration %.8x %d", (u32)pdesc->handle, configuration);
	usbd_mc_add_event(g_pminorcontrol, s);

	result = wait_event_interruptible_timeout(
				pdesc->config_change_wait, 
				pdesc->config_changed, 
				msecs_to_jiffies(timeoutms));

	if( result == 0 )
		result = -ETIMEDOUT;
	else if( result > 0 )
		result = jiffies_to_msecs(result);
	else if( result == -ERESTARTSYS )
		result = -ERESTART;

	return result;
}

int usbd_usbdevice_refresh_state(struct usbdevice_descriptor *pdesc)
{
	char s[50];

	if( !pdesc ) return -1;

	if( pdesc->handle == (u32)-1 ) return 0;

	sprintf(s, "state %.8x %d", (u32)pdesc->handle, pdesc->state);
	usbd_mc_add_event(g_pminorcontrol, s);
	return 0;
}

int usbd_usbdevice_get_descriptor(struct usbdevice_descriptor *pdesc, struct usb_device *pudev)
{
	char buf[18];

	return usb_control_msg(pudev, usb_rcvctrlpipe(pudev, 0),
				USB_REQ_GET_DESCRIPTOR, USB_DIR_IN,
				(1 << 8) + 0, 0, buf, sizeof(buf),
				USB_CTRL_GET_TIMEOUT/2);
}

void usbd_usbdevice_reset(struct usbdevice_descriptor *pdesc, struct usb_device *pudev)
{
        struct waitable_queue_entry *pentry;
	struct usbd_usb_request *pureq = NULL;
        unsigned long flags;

        TRACE("usbd_usbdevice_reset++\n");
        /*
                we cannot call usb_unlink_urb when queue lock is held, so we should first 
                find the required entry, release the lock and unlink the urb.
                Then repeat this until no urbs are found.
        */
        do
        {
                usbd_wq_lock(&pdesc->queue_pending_unrb, flags);
        
                pentry = usbd_wq_get_first(&pdesc->queue_pending_unrb);

                while(pentry)
                {
			pureq = (struct usbd_usb_request *)pentry->data;

                        if( pureq->endpoint != (uint8)-1 )
                        {
                                // mark this unrb as processed
                                TRACE("usbd_usbdevice_reset: urb to be cancelled on %d endpoint\n", pureq->endpoint);
                   				pureq->endpoint = (uint8)-1;

                                // prevent urb from freeing when we release the spinlock
								if( pureq->purb )
									usb_get_urb(pureq->purb);
								else
									usbd_uc_get(&pureq->urbchain);
                                break;

                        }
                        pentry = usbd_wq_get_next(&pdesc->queue_pending_unrb, pentry);
                }
                usbd_wq_unlock(&pdesc->queue_pending_unrb, flags);

		if( pentry )
		{
			// if we have pentry - we have pureq too!
			if( pureq->purb )
			{
				usb_unlink_urb(pureq->purb);
				usb_put_urb(pureq->purb);
				TRACE("usbd_usbdevice_reset: urb cancelled\n");
			}
			else
			{
				usbd_uc_cancel(&pureq->urbchain);
				usbd_uc_put(&pureq->urbchain);
				TRACE("usbd_usbdevice_reset: urb chain cancelled\n");
			}
		}

        } while(pentry);

		TRACE("usbd_usbdevice_reset: urbs cancelled\n");

	{
		int res;
		                  TRACE_CRITICAL("before reset\n");
		    if( usb_lock_device_for_reset(pudev, NULL) >= 0 )
		    {
                	res = usb_reset_configuration(pudev);
			TRACE("usbd_usbdevice_reset: usb_reset_configuration returned %d\n",res);
       	        	usb_unlock_device(pudev);
		    }

			if( usbd_usbdevice_get_descriptor(pdesc, pudev) < 0 )
			{
			    if( usb_lock_device_for_reset(pudev, NULL) >= 0 )
			    {
					res = usb_reset_device(pudev);

					usb_unlock_device(pudev);
				}
			} else
			{
				TRACE("usbd_usbdevice_reset: get_descriptor succeeded\n");
			}
		                  TRACE_CRITICAL("after reset\n");
		}

		memset(pdesc->frame_delta_in, 0, sizeof(pdesc->frame_delta_in));
		memset(pdesc->frame_delta_out, 0, sizeof(pdesc->frame_delta_out));

		TRACE("usbd_usbdevice_reset--\n");
}

int usbd_usbdevice_inc_iso_counter(struct usbdevice_descriptor *pdesc, int pipe)
{
	if(!usb_pipeisoc(pipe)) return 0;
	
	if(usb_pipein(pipe))
		return (++pdesc->iso_counter_in[usb_pipeendpoint(pipe)]);
	else
		return (++pdesc->iso_counter_out[usb_pipeendpoint(pipe)]);
}

int usbd_usbdevice_dec_iso_counter(struct usbdevice_descriptor *pdesc, int pipe)
{
	if(!usb_pipeisoc(pipe)) return 0;
	
	if(usb_pipein(pipe))
		return (--pdesc->iso_counter_in[usb_pipeendpoint(pipe)]);
	else
		return (--pdesc->iso_counter_out[usb_pipeendpoint(pipe)]);
}

void usbd_usbdevice_reset_iso_counter(struct usbdevice_descriptor *pdesc, int pipe)
{
	if(!usb_pipeisoc(pipe)) return;
	
	if(usb_pipein(pipe))
		pdesc->iso_counter_in[usb_pipeendpoint(pipe)] = 0;
	else
		pdesc->iso_counter_out[usb_pipeendpoint(pipe)] = 0;
}

int usbd_usbdevice_get_frame_delta(struct usbdevice_descriptor *pdesc, int pipe)
{
	if(!usb_pipeisoc(pipe)) return 0;
	
	if(usb_pipein(pipe))
		return pdesc->frame_delta_in[usb_pipeendpoint(pipe)];
	else
		return pdesc->frame_delta_out[usb_pipeendpoint(pipe)];
}

void usbd_usbdevice_set_frame_delta(struct usbdevice_descriptor *pdesc, int pipe, int frame_delta)
{
	if(!usb_pipeisoc(pipe)) return;
	
	if(usb_pipein(pipe))
		pdesc->frame_delta_in[usb_pipeendpoint(pipe)] = frame_delta;
	else
		pdesc->frame_delta_out[usb_pipeendpoint(pipe)] = frame_delta;
}

struct usb_device *usbd_usbdevice_pudev_get(struct usbdevice_descriptor *pdesc)
{
	unsigned long flags;
	struct usb_device *result = NULL;

	spin_lock_irqsave(&pdesc->pudev_lock,flags);
	if(pdesc->pudev)
	{
		result = usb_get_dev(pdesc->pudev);
	}
	spin_unlock_irqrestore(&pdesc->pudev_lock,flags);

	return result;
}

void usbd_usbdevice_pudev_put(struct usbdevice_descriptor *pdesc, struct usb_device *pudev)
{
	usb_put_dev(pudev);
}

#endif // #ifdef _USBD_ENABLE_STUB_
