	/*
 *  tusbd/minor_control.c
 *
 *  Copyright (C) 2007-2008 IncentivesPro
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include "usbd.h"

ssize_t usbd_mc_write(void *, const char __user *, size_t);
ssize_t usbd_mc_read(void *, char __user *, size_t);
int usbd_mc_ioctl(void *, unsigned int, unsigned long);
int usbd_mc_open(void *);
int usbd_mc_release(void *);
unsigned int usbd_mc_poll(void *, struct file *, poll_table *wait);

int usbd_mc_share(struct minor_descriptor *pminorcontroldesc, const char *buf, size_t count);
int usbd_mc_unshare(struct minor_descriptor *pminorcontroldesc, const char *buf, size_t count);
int usbd_mc_addexclude(struct minor_descriptor *pminorcontroldesc, const char *buf, size_t count);
int usbd_mc_remexclude(struct minor_descriptor *pminorcontroldesc, const char *buf, size_t count);
int usbd_mc_reset(struct minor_descriptor *pminorcontroldesc, const char *buf, size_t count);
int usbd_mc_plug(struct minor_descriptor *pminorcontroldesc, const char *buf, size_t count);
int usbd_mc_unplug(struct minor_descriptor *pminorcontroldesc, const char *buf, size_t count);
int usbd_mc_setmode(struct minor_descriptor *pminorcontroldesc, const char *buf, size_t count);


struct minor_descriptor *usbd_mc_alloc_descriptor(void)
{
	struct minor_descriptor *pdesc;
   	pdesc = kmalloc(sizeof(*pdesc), GFP_KERNEL);

   	if( pdesc )
   	{
	  	memset(pdesc, 0, sizeof(*pdesc));

	  	pdesc->ops.open = usbd_mc_open;
	  	pdesc->ops.release = usbd_mc_release;
	  	pdesc->ops.ioctl = usbd_mc_ioctl;
	  	pdesc->ops.read = usbd_mc_read;
	  	pdesc->ops.write = usbd_mc_write;
	  	pdesc->ops.poll = usbd_mc_poll;

		pdesc->context = pdesc;

	  	pdesc->fifo = usbd_fifo_create();

	  	if( !pdesc->fifo )
	  	{
	  		kfree(pdesc);
	  		pdesc = NULL;
	  	}
  	}
   	return pdesc;
}

void usbd_mc_free_descriptor(struct minor_descriptor *pdesc)
{
	if( pdesc )
	{
		if( pdesc->fifo )
		{
	  		usbd_fifo_destroy(pdesc->fifo);
			pdesc->fifo = NULL;
		}
		kfree(pdesc);
	}
}

inline int usbd_mc_add_event(struct minor_descriptor *pdesc, const char *event)
{
	return usbd_fifo_add(pdesc->fifo, event, strlen(event)+1, 0);
}

ssize_t usbd_mc_read(void *context, char __user *buf, size_t count)
{
	struct minor_descriptor *pdesc = context;
	TRACE("usbd_mc_read\n");
	return usbd_fifo_get(pdesc->fifo, buf, count, 1);
}

ssize_t usbd_mc_write(void *context, const char __user *buf, size_t count)
{
	struct minor_descriptor *pminorcontroldesc = context;
	ssize_t result = 0;
	char kernelbuf[50];

	TRACE("usbd_mc_write: ++\n");
	TRACE("usbd_mc_write: Buf=0x%p\n", buf);

	do
	{
		if( count > sizeof(kernelbuf) )
		{
			TRACE("usbd_mc_write: too large write request\n");
			result = -EFAULT;
			break;
		}

		if( copy_from_user(kernelbuf, buf, count) != 0 )
		{
			TRACE("usbd_mc_write: cannot copy buffer\n");
			result = -EFAULT;
			break;
		}

		kernelbuf[count] = 0;

		TRACE("usbd_mc_write: command is \"%s\"\n", kernelbuf);

#ifdef _USBD_ENABLE_STUB_
		if( strnicmp(kernelbuf, "share", 5) == 0 )
		{
			TRACE("usbd_mc_write: SHARE command received\n");
			result = usbd_mc_share(pminorcontroldesc, kernelbuf+5, count-5);

		} else if( strnicmp(kernelbuf, "unshare", 7) == 0 )
		{
			TRACE("usbd_mc_write: UNSHARE command received\n");
			result = usbd_mc_unshare(pminorcontroldesc, kernelbuf+7, count-7);

		} else if( strnicmp(kernelbuf, "reset", 5) == 0 )
		{
			TRACE("usbd_mc_write: RESET command received\n");
			result = usbd_mc_reset(pminorcontroldesc, kernelbuf+5, count-5);

		} else if( strnicmp(kernelbuf, "addexclude", 10) == 0 )
		{
			TRACE("usbd_mc_write: ADDEXCLUDE command received\n");
			result = usbd_mc_addexclude(pminorcontroldesc, kernelbuf+10, count-10);
		} else if( strnicmp(kernelbuf, "remexclude", 10) == 0 )
		{
			TRACE("usbd_mc_write: REMEXCLUDE command received\n");
			result = usbd_mc_remexclude(pminorcontroldesc, kernelbuf+10, count-10);
		} else if( strnicmp(kernelbuf, "setmode", 7) == 0 )
		{
			TRACE("usbd_mc_write: SETMODE command received\n");
			result = usbd_mc_setmode(pminorcontroldesc, kernelbuf+7, count-7);
		}
#endif //#ifdef _USBD_ENABLE_STUB_

#ifdef _USBD_ENABLE_VHCI_
		if( strnicmp(kernelbuf, "plug", 4) == 0 )
		{
			TRACE("usbd_mc_write: PLUG command received\n");
			result = usbd_mc_plug(pminorcontroldesc, kernelbuf+4, count-4);

		} else if( strnicmp(kernelbuf, "unplug", 6) == 0 )
		{
			TRACE("usbd_mc_write: UNPLUG command received\n");
			result = usbd_mc_unplug(pminorcontroldesc, kernelbuf+6, count-6);
		}
#endif // #ifdef _USBD_ENABLE_VHCI_

	} while(0);

	if( result == 0 ) result = count;

	TRACE("usbd_mc_write: -- result=%d\n", result);

	return result;
}

int usbd_mc_ioctl(void *context, unsigned int cmd, unsigned long arg)
{

	TRACE("usbd_mc_ioctl\n");

	return 0;
}

int usbd_mc_open(void *context)
{
	struct minor_descriptor *pdesc = context;

	TRACE("usbd_mc_open\n");
	usbd_fifo_clear(pdesc->fifo);
	return 0;
}

int usbd_mc_release(void *context)
{
	struct minor_descriptor *pdesc = context;
	TRACE("usbd_mc_release\n");
	usbd_fifo_clear(pdesc->fifo);
	return 0;
}

unsigned int usbd_mc_poll(void *context, struct file *filp, poll_table *wait)
{
	struct minor_descriptor *pdesc = context;

	poll_wait(filp, &pdesc->fifo->waitqueue, wait);

	if( pdesc->fifo->num_entries)
	{
		TRACE("usbd_mc_poll: num_entries=%d\n", pdesc->fifo->num_entries);
		return ((POLLOUT | POLLWRNORM) | (POLLIN | POLLRDNORM));
	}

	return (POLLOUT | POLLWRNORM);
}

#ifdef _USBD_ENABLE_STUB_

int usbd_mc_share(struct minor_descriptor *pminorcontroldesc, const char *buf, size_t count)
{
	int result = -EFAULT;
	struct usbdevice_descriptor *pdesc = NULL;

	TRACE("usbd_mc_share: ++\n");

	do /*while(0)*/
	{
		u32 vid, pid, handle;
		char bus_id[BUS_ID_SIZE] = {0};
		char event_str[30];
		int refresh = 0;

		if( sscanf(buf, "%x-%x %s %x", &vid, &pid, bus_id, &handle) != 4 )
		{
			TRACE("usbd_mc_share: sscanf failed\n");
			result = -EINVAL;
			break;
		}

		TRACE("usbd_mc_share: device is vid=%x pid=%x bus_id=%s handle=%u\n", vid, pid, bus_id, handle);

		pdesc = usbd_usbdevice_find1(vid, pid, bus_id);

		if( pdesc )
		{
			TRACE("usbd_mc_share: device already shared, refreshing it's state\n");
			refresh = 1;
		} else
		{
			pdesc = usbd_usbdevice_create();

			if( pdesc == NULL )
			{
				TRACE("usbd_mc_share: usbd_usbdevice_create failed\n");
				result = -ENOMEM;
				break;
			}
		}

		pdesc->handle = handle;
		pdesc->vid = vid;
		pdesc->pid = pid;
		strncpy(pdesc->bus_id, bus_id, BUS_ID_SIZE);

		sprintf(event_str, "devnum %.8x %d %d", pdesc->handle, g_major, pdesc->pminor->minor);

		result = usbd_mc_add_event(pminorcontroldesc, event_str);

		if( result < 0 )
		{
			TRACE("usbd_mc_share: usbd_mc_add_event failed with error %d\n", result);
			result = -ENOMEM;
			break;
		} else
		{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,13)
		    usbd_bind_device_to_driver(pdesc, &usbd_driver);
#endif
		}

		if( refresh ) 
			usbd_usbdevice_refresh_state(pdesc);

		usbd_usbdevice_dereference(pdesc);

		result = 0;

	} while(0);

	if( result != 0 )
	{
		if(pdesc) usbd_usbdevice_destroy(pdesc);
	}

	TRACE("usbd_mc_share: -- result=%d\n", result);
	return result;
}

int usbd_mc_unshare(struct minor_descriptor *pminorcontroldesc, const char *buf, size_t count)
{
	u32 handle;
	struct usbdevice_descriptor *pdesc;
	int result = -EFAULT;

	TRACE("usbd_mc_unshare: ++\n");

	do /*while(0)*/
	{
		if( sscanf(buf, "%x", &handle) != 1 )
		{
			TRACE("usbd_mc_unshare: sscanf failed\n");
			result = -EINVAL;
			break;
		}

		TRACE("usbd_mc_unshare: handle=%u\n", handle);

		pdesc = usbd_usbdevice_find2(handle);

		if( pdesc == NULL )
		{
			TRACE("usbd_mc_unshare: device with handle %u is not found\n", handle);
			result = -ENODEV;
			break;
		}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,13)
		usbd_bind_device_to_driver(pdesc, NULL);
#else
		usbd_usbdevice_change_state(pdesc, eDeviceInit);
#endif
		usbd_usbdevice_destroy(pdesc);
		usbd_usbdevice_dereference(pdesc);

		result = 0;

	} while(0);

	TRACE("usbd_mc_unshare: -- result=%d\n", result);

	return result;
}

int usbd_mc_reset(struct minor_descriptor *pminorcontroldesc, const char *buf, size_t count)
{
	u32 handle;
	struct usbdevice_descriptor *pdesc;
	int result = -EFAULT;

	TRACE("usbd_mc_reset: ++\n");

	do /*while(0)*/
	{
		struct usb_device *pudev;

		if( sscanf(buf, "%x", &handle) != 1 )
		{
			TRACE("usbd_mc_reset: sscanf failed\n");
			result = -EINVAL;
			break;
		}

		TRACE("usbd_mc_reset: handle=%u\n", handle);

		pdesc = usbd_usbdevice_find2(handle);

		if( pdesc == NULL )
		{
			TRACE("usbd_mc_reset: device with handle %u is not found\n", handle);
			result = -ENODEV;
			break;
		}

		pudev = usbd_usbdevice_pudev_get(pdesc);

		if( pudev )
		{
			usbd_usbdevice_reset(pdesc, pudev);
			usbd_usbdevice_pudev_put(pdesc, pudev);
		}

		usbd_usbdevice_dereference(pdesc);

		result = 0;

	} while(0);

	TRACE("usbd_mc_reset: -- result=%d\n", result);

	return result;
}

int usbd_mc_addexclude(struct minor_descriptor *pminorcontroldesc, const char *buf, size_t count)
{
	int result = -EFAULT;

	TRACE("usbd_mc_addexclude: ++\n");

	do /*while(0)*/
	{
		u32 vid, pid, handle;
		char bus_id[BUS_ID_SIZE] = {0};
		struct waitable_queue_entry *pqentry;
		struct usbd_exclusion *pexclusion;

		if( sscanf(buf, "%x-%x %s %x", &vid, &pid, bus_id, &handle) != 4 )
		{
			TRACE("usbd_mc_addexclude: sscanf failed\n");
			result = -EINVAL;
			break;
		}

		TRACE("usbd_mc_addexclude: device is vid=%x pid=%x bus_id=%s handle=%u\n", vid, pid, bus_id, handle);

		pqentry = usbd_wq_alloc_entry(sizeof(struct usbd_exclusion), GFP_KERNEL);

		if( pqentry == NULL )
		{
			TRACE("usbd_mc_addexclude: no memory\n");
			result = -ENOMEM;
			break;
		}
		                                                                       	
		pexclusion = (struct usbd_exclusion *)pqentry->data;

		pexclusion->handle = handle;
		pexclusion->vid = vid;
		pexclusion->pid = pid;
		strncpy(pexclusion->bus_id, bus_id, BUS_ID_SIZE);

		usbd_wq_add_tail_locked(&g_exclusions, pqentry);

		result = 0;

	} while(0);

	TRACE("usbd_mc_addexclude: -- result=%d\n", result);
	return result;
}

int usbd_mc_remexclude(struct minor_descriptor *pminorcontroldesc, const char *buf, size_t count)
{
	u32 handle;
	int result = -EFAULT;

	TRACE("usbd_mc_remexclude: ++\n");

	do /*while(0)*/
	{
		unsigned long flags;
		struct waitable_queue_entry *pqentry;

		if( sscanf(buf, "%x", &handle) != 1 )
		{
			TRACE("usbd_mc_remexclude: sscanf failed\n");
			result = -EINVAL;
			break;
		}

		TRACE("usbd_mc_remexclude: handle=%u\n", handle);

		usbd_wq_lock(&g_exclusions, flags);
		pqentry = usbd_wq_get_first(&g_exclusions);
		while(pqentry)
		{
			struct usbd_exclusion *pexclusion;
			struct waitable_queue_entry *tmp;

			pexclusion = (struct usbd_exclusion *)pqentry->data;

			if( pexclusion->handle == handle )
			{
				tmp = pqentry;
				pqentry = usbd_wq_get_next(&g_exclusions, pqentry);
				usbd_wq_remove_entry(tmp);
				usbd_wq_free_entry(tmp);
				TRACE("usbd_mc_remexclude: exclusion removed\n");
			} else
			{
				pqentry = usbd_wq_get_next(&g_exclusions, pqentry);
			}
		}
		usbd_wq_unlock(&g_exclusions, flags);

		result = 0;

	} while(0);

	TRACE("usbd_mc_unshare: -- result=%d\n", result);

	return result;
}

int usbd_mc_setmode(struct minor_descriptor *pminorcontroldesc, const char *buf, size_t count)
{
	int result = -EFAULT;

	TRACE("usbd_mc_setmode: ++\n");

	do /*while(0)*/
	{
		u32 mode;

		if( sscanf(buf, "%x", &mode) != 1 )
		{
			TRACE("usbd_mc_setmode: sscanf failed\n");
			result = -EINVAL;
			break;
		}

		TRACE("usbd_mc_setmode: mode=%u\n", mode);

		g_autosharing = mode;

		result = 0;

	} while(0);

	TRACE("usbd_mc_setmode: -- result=%d\n", result);

	return result;
}

#endif // #ifdef _USBD_ENABLE_STUB_

#ifdef _USBD_ENABLE_VHCI_

int usbd_mc_plug(struct minor_descriptor *pminorcontroldesc, const char *buf, size_t count)
{
	int result = -EFAULT;
	struct vhci_device_descriptor *pdesc = NULL;

	TRACE("usbd_mc_plug: ++\n");

	do /*while(0)*/
	{
		u32 hserver, hdevice;
		char event_str[30];
		char speed_str[10];
		int speed;

		if( sscanf(buf, "%x %x %4s", &hserver, &hdevice, speed_str) != 3 )
		{
			TRACE("usbd_mc_plug: sscanf failed\n");
			result = -EINVAL;
			break;
		}

		if( strnicmp(speed_str, "low", 3) == 0 )
		{
			speed = USB_SPEED_LOW; 
		} else if( strnicmp(speed_str, "full", 4) == 0 )
		{
			speed = USB_SPEED_FULL; 
		} else if( strnicmp(speed_str, "high", 4) == 0 )
		{
			speed = USB_SPEED_HIGH; 
		} else
		{
			TRACE("usbd_mc_plug: invalid device speed\n");
			result = -EINVAL;
			break;
		}

		pdesc = usbd_vhci_device_find1(hserver, hdevice);

		if( pdesc != NULL )
		{
			usbd_vhci_device_dereference(pdesc);
			pdesc = NULL;
			TRACE("usbd_mc_plug: device with handle %u %u is already plugged\n", hserver, hdevice);
			result = -EISCONN;
			break;
		}

		pdesc = usbd_vhci_device_create();

		if( pdesc == NULL )
		{
			TRACE("usbd_mc_plug: usbd_vhci_device_create failed\n");
			result = -ENOMEM;
			break;
		}

		pdesc->hserver = hserver;
		pdesc->hdevice = hdevice;
		pdesc->speed = speed;

		sprintf(event_str, "vdevnum %.8x %.8x %d %d", pdesc->hserver, pdesc->hdevice, g_major, pdesc->pminor->minor);

		result = usbd_mc_add_event(pminorcontroldesc, event_str);

		if( result < 0 )
		{
			TRACE("usbd_mc_share: usbd_mc_add_event failed with error %d\n", result);
			result = -ENOMEM;
			break;
		}

		usbd_vhci_device_connect(pdesc);

		result = 0;

	} while(0);

	if( result != 0 )
	{
		if(pdesc) usbd_vhci_device_destroy(pdesc);
	}


	TRACE("usbd_mc_plug: -- result=%d\n", result);
	return result;
}

int usbd_mc_unplug(struct minor_descriptor *pminorcontroldesc, const char *buf, size_t count)
{
	int result = -EFAULT;
	struct vhci_device_descriptor *pdesc;

	TRACE("usbd_mc_unplug: ++\n");

	do /*while(0)*/
	{
		u32 hserver, hdevice;

		if( sscanf(buf, "%x %x", &hserver, &hdevice) != 2 )
		{
			TRACE("usbd_mc_unplug: sscanf failed\n");
			result = -EINVAL;
			break;
		}

		pdesc = usbd_vhci_device_find1(hserver, hdevice);

		if( pdesc == NULL )
		{
			TRACE("usbd_mc_unplug: device on with handle %u %u is not found\n", hserver, hdevice);
			result = -ENODEV;
			break;
		}

		usbd_vhci_device_disconnect(pdesc);
		usbd_vhci_device_destroy(pdesc);
		usbd_vhci_device_dereference(pdesc); // remove reference by usbd_vhci_device_find1

		result = 0;

	} while(0);

	TRACE("usbd_mc_unplug: -- result=%d\n", result);
	return result;
}

#endif // #ifdef _USBD_ENABLE_VHCI_
