/*
 *  tusbd/vhci_device.c
 *
 *  Copyright (C) 2007 IncentivesPro
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifdef _USBD_ENABLE_VHCI_

#include "usbd.h"
//#include "/usr/src/linux/drivers/usb/core/hcd.h"
#include HCD_H_PATH

#include <linux/platform_device.h>

struct usb_hcd *g_vhci_hcd = NULL;

///////////////////////////////////////////////////////////////////////////////
// HC device driver implementation

static int usbd_vhci_platform_driver_probe(struct platform_device *);
static int usbd_vhci_platform_driver_remove(struct platform_device *);
static void usbd_vhci_platform_device_release(struct device *dev);

static int usbd_vhci_start(struct usb_hcd *hcd);
static void usbd_vhci_stop(struct usb_hcd *hcd);
static int usbd_vhci_get_frame_number(struct usb_hcd *hcd);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24)
static int usbd_vhci_urb_dequeue(struct usb_hcd *hcd, struct urb *urb);
static int usbd_vhci_urb_enqueue(struct usb_hcd *hcd, struct usb_host_endpoint *ep, struct urb *urb, gfp_t mem_flags);
#else
static int usbd_vhci_urb_dequeue(struct usb_hcd *hcd, struct urb *urb, int status);
static int usbd_vhci_urb_enqueue(struct usb_hcd *hcd, struct urb *urb, gfp_t mem_flags);
#endif
static int usbd_vhci_hub_control(struct usb_hcd *hcd, u16 typeReq, u16 wValue, u16 wIndex, char *buf, u16 wLength);
static int usbd_vhci_hub_status_data(struct usb_hcd *hcd, char *buf);
static int usbd_vhci_bus_suspend(struct usb_hcd *hcd);
static int usbd_vhci_bus_resume(struct usb_hcd *hcd);

#define get_hcd_data(hcd) ((struct vhci_hcd_descriptor*)(hcd->hcd_priv))


static struct platform_driver usbd_vhci_platform_driver = 
{
	.probe	= usbd_vhci_platform_driver_probe,
	.remove	= usbd_vhci_platform_driver_remove,
	.driver	= 
	{
		.name = USBD_VHCI_DRIVER_NAME,
		.owner = THIS_MODULE,
	},
};

static struct platform_device usbd_vhci_platform_device =
{
	.name = USBD_VHCI_DRIVER_NAME,
	.id = -1,

	.dev = {
		//.driver = &vhci_driver,
		.release = usbd_vhci_platform_device_release,
	},

};

static struct hc_driver usbd_vhci_hc_driver = 
{
	.description	= "usb-vhci-driver",
	.product_desc	= "vhci",
	.hcd_priv_size	= sizeof(struct vhci_hcd_descriptor),

	.flags				= HCD_USB2,

	.start				= usbd_vhci_start,
	.stop				= usbd_vhci_stop,

	.urb_enqueue		= usbd_vhci_urb_enqueue,
	.urb_dequeue		= usbd_vhci_urb_dequeue,

	.get_frame_number	= usbd_vhci_get_frame_number,

	.hub_status_data	= usbd_vhci_hub_status_data,
	.hub_control		= usbd_vhci_hub_control,

	.bus_suspend		= usbd_vhci_bus_suspend,
	.bus_resume			= usbd_vhci_bus_resume,

};

static struct list_head s_device_list;
static spinlock_t s_device_list_lock;

static int usbd_vhci_platform_driver_probe(struct platform_device *pdev)
{
	struct vhci_hcd_descriptor *hcd_desc;
	int result = 0;

	TRACE("usbd_vhci_platform_driver_probe: ++\n");
	                                                                      	
	do /*while(0)*/
	{
		if (pdev->dev.dma_mask)
		{
			TRACE("usbd_vhci_platform_driver_probe: dma not supported!\n");
			return -EINVAL;
		}

		g_vhci_hcd = usb_create_hcd(&usbd_vhci_hc_driver, &pdev->dev, get_busid(&pdev->dev));

		if( g_vhci_hcd == NULL )
		{
			result = -ENOMEM;
			break;
		}

		hcd_desc = get_hcd_data(g_vhci_hcd);

		memset(hcd_desc, 0, sizeof(*hcd_desc));

		spin_lock_init(&hcd_desc->lock);

		result = usb_add_hcd(g_vhci_hcd, 0, 0);

		if( result != 0 )
		{
			usb_put_hcd(g_vhci_hcd);
			g_vhci_hcd = NULL;
			break;
		}

	} while(0);

	TRACE("usbd_vhci_platform_driver_probe: -- result = %d\n", result);
	return result;
}


static int usbd_vhci_platform_driver_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd;

	TRACE("usbd_vhci_platform_driver_remove: ++\n");

	hcd = platform_get_drvdata(pdev);
	TRACE("usbd_vhci_platform_driver_remove: 1\n");

	if( hcd )
	{
	TRACE("usbd_vhci_platform_driver_remove: 2\n");
		usb_remove_hcd(hcd);
	TRACE("usbd_vhci_platform_driver_remove: 3\n");
		usb_put_hcd(hcd);
	}

	TRACE("usbd_vhci_platform_driver_remove: --\n");
	return 0;
}

static void usbd_vhci_platform_device_release(struct device *dev)
{
	return;
}

static int usbd_vhci_start(struct usb_hcd *hcd)
{
	//struct vhci_hcd_descriptor *hcd_desc = get_hcd_data(hcd);

	TRACE("usbd_vhci_start\n");
	hcd->power_budget = 0;
	hcd->state = HC_STATE_RUNNING;
	hcd->uses_new_polling = 1;
	return 0;
}

static void usbd_vhci_stop(struct usb_hcd *hcd)
{
	TRACE("usbd_vhci_stop\n");
}

static int usbd_vhci_get_frame_number(struct usb_hcd *hcd)
{
	TRACE("usbd_vhci_get_frame_number\n");
	return 0;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24)
static int usbd_vhci_urb_enqueue(struct usb_hcd *hcd, struct usb_host_endpoint *ep, struct urb *urb, gfp_t mem_flags)
#else
static int usbd_vhci_urb_enqueue(struct usb_hcd *hcd, struct urb *urb, gfp_t mem_flags)
#endif
{
	int result = -EINPROGRESS;
	struct vhci_hcd_descriptor *hcd_desc = get_hcd_data(hcd);
	struct vhci_device_descriptor *pdev = NULL;
	unsigned long flags;
	int status = -1;

	TRACE("usbd_vhci_urb_enqueue: ++ urb=0x%p usb_device=0x%p parent=0x%p devnum=%d\n", urb, urb->dev, urb->dev->dev.parent, urb->dev->devnum);

	dump_urb(urb, 0);

	do /*while(0)*/
	{
		struct waitable_queue_entry *pqentry;

		if (!urb->transfer_buffer && urb->transfer_buffer_length)
		{
			result = -EINVAL;
			break;
		}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24)
		spin_lock(&urb->lock);
		status = urb->status;
		spin_unlock (&urb->lock);

		if( status != -EINPROGRESS )
#else
		if( urb->unlinked )
#endif
		{
			TRACE("usbd_vhci_urb_enqueue: urb already unlinked\n");
			result = 0;
			break;
		}

		/* this will also add a reference to device */
		pdev = usbd_vhci_device_find2(hcd, usb_pipedevice(urb->pipe));

		if( pdev == NULL )
		{
			TRACE("usbd_vhci_urb_enqueue: device not found\n");
			result = -ENODEV;
			break;
		}

		urb->hcpriv = pdev;

		if( usb_pipedevice(urb->pipe) == 0 && usb_pipetype(urb->pipe) == PIPE_CONTROL )
		{
			struct usb_ctrlrequest *ctrlreq = (struct usb_ctrlrequest *) urb->setup_packet;

			if( !ctrlreq )
			{
				TRACE("usbd_vhci_urb_enqueue: invalid control transfer urb\n");
				result = -EINVAL;
				break;
			}

			if( ctrlreq->bRequest == USB_REQ_SET_ADDRESS )
			{
				TRACE("usbd_vhci_urb_enqueue: USB_REQ_SET_ADDRESS address=%d\n", 
					ctrlreq->wValue);

				spin_lock_irqsave (&hcd_desc->lock, flags);
				pdev->address = le16_to_cpu(ctrlreq->wValue);
				spin_unlock_irqrestore(&hcd_desc->lock, flags);

				status = 0;
				result = 0;
				break;
			}
		}

		pqentry = usbd_wq_alloc_entry(sizeof(struct vhci_queue_data), GFP_ATOMIC);

		if( pqentry == NULL )
		{
			TRACE("usbd_vhci_urb_enqueue: no memory for unrb\n");
			result = -ENOMEM;
			break;
		}

		((struct vhci_queue_data*)pqentry->data)->unique_id = usbd_get_unique_id();
		((struct vhci_queue_data*)pqentry->data)->purb = urb;
		((struct vhci_queue_data*)pqentry->data)->punrb = NULL;

		usbd_wq_add_tail_locked(&pdev->queue_pending_transfer_unrb, pqentry);

		result = -EINPROGRESS;

	} while(0);

	switch(result)
	{
		case -EINPROGRESS: /* urb queued for transfer */
			result = 0;
			break;

		case 0: /* urb completed */
			usbd_vhci_device_complete_urb(urb, status);
			break;

		default: /* there was a error while processing the urb */
			if(pdev) 
				/* remove the reference added by usbd_vhci_device_find2 */
				usbd_vhci_device_dereference(pdev);
			break;
	}

	TRACE("usbd_vhci_urb_enqueue: --\n");
	return result;
}


#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24)
static int usbd_vhci_urb_dequeue(struct usb_hcd *hcd, struct urb *urb)
#else
static int usbd_vhci_urb_dequeue(struct usb_hcd *hcd, struct urb *urb, int status)
#endif
{
	//struct vhci_hcd_descriptor *hcd_desc = get_hcd_data(hcd);
	struct vhci_device_descriptor *pdev = NULL;
	struct waitable_queue_entry *pqentry = NULL;
	PUNRB punrb;

	TRACE("usbd_vhci_urb_dequeue: ++ urb=0x%p\n", urb);

	//dump_urb(urb, 0);

	do /*while(0)*/
	{
		pdev = (struct vhci_device_descriptor *)urb->hcpriv;
		
		if( !pdev ) 
		{
			TRACE("usbd_vhci_urb_dequeue: device not found!\n");
			break;
		}

		pqentry = usbd_vhci_device_dequeue_urb(pdev, urb);

		if( !pqentry ) 
		{
			TRACE("usbd_vhci_urb_dequeue: urb not found in queue\n");
			break;
		}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24)
		usbd_vhci_device_complete_urb(urb, -1);
#else
		usbd_vhci_device_complete_urb(urb, status);
#endif

		punrb = kmalloc(sizeof(UNRB_CANCEL), GFP_ATOMIC);

		if( !punrb )
		{
			TRACE("usbd_vhci_urb_dequeue: no memory for unrb\n");
			break;
		}

		punrb->Header.UniqueId = ((struct vhci_queue_data*)pqentry->data)->unique_id;
		punrb->Header.Size= sizeof(UNRB_CANCEL);
		punrb->Header.Function = UNRB_FUNCTION_CANCEL;
		punrb->Header.Status = 0;
		punrb->Header.Context = 0;

		((struct vhci_queue_data*)pqentry->data)->purb = NULL;
		((struct vhci_queue_data*)pqentry->data)->punrb = punrb;

		usbd_wq_add_tail_locked(&pdev->queue_pending_transfer_unrb, pqentry);

	} while(0);

	TRACE("usbd_vhci_urb_dequeue: --\n");
	return 0;
}

#define PORT_C_MASK \
	((USB_PORT_STAT_C_CONNECTION \
	| USB_PORT_STAT_C_ENABLE \
	| USB_PORT_STAT_C_SUSPEND \
	| USB_PORT_STAT_C_OVERCURRENT \
	| USB_PORT_STAT_C_RESET) << 16)

static int usbd_vhci_hub_control(struct usb_hcd *hcd, u16 typeReq, u16 wValue, u16 wIndex, char *buf, u16 wLength)
{
	struct vhci_hcd_descriptor *hcd_desc = get_hcd_data(hcd);
	int result = 0;
	int nport = -1;
	unsigned long flags;

	TRACE("usbd_vhci_hub_control: ++\n");

	if( !test_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags) )
		return -ETIMEDOUT;

	spin_lock_irqsave(&hcd_desc->lock, flags);

	switch( typeReq )
	{
		case GetHubDescriptor:
			TRACE("usbd_vhci_hub_control: GetHubDescriptor\n");
			{
				struct usb_hub_descriptor *desc = (struct usb_hub_descriptor *)buf;

				memset(desc, 0, sizeof(*desc));

				desc->bDescriptorType = 0x29;
				desc->bDescLength = 7 + VHCI_USB_MAX_CHILDREN_BYTES*2;
				desc->wHubCharacteristics = (__le16)cpu_to_le16(0x0001);
				desc->bNbrPorts = VHCI_USB_MAX_CHILDREN;

				memset(
					&desc->bitmap[0], 
					0, 
					VHCI_USB_MAX_CHILDREN_BYTES);

				memset(
					&desc->bitmap[VHCI_USB_MAX_CHILDREN_BYTES], 
					0xff, 
					VHCI_USB_MAX_CHILDREN_BYTES);
			}
			break;

		case GetHubStatus:
			TRACE("usbd_vhci_hub_control: GetHubStatus\n");
			*(__le32 *)buf = cpu_to_le32(0);
			break;

		case SetHubFeature:
			TRACE("usbd_vhci_hub_control: SetHubFeature\n");
			result = -EPIPE;
			break;

		case ClearHubFeature:
			TRACE("usbd_vhci_hub_control: ClearHubFeature\n");
			break;

		case GetPortStatus:
			TRACE("usbd_vhci_hub_control: GetPortStatus\n");

			nport = (wIndex&0xff)-1;
			TRACE("usbd_vhci_hub_control: port=%d\n", nport);

			if( nport < 0 || nport >= VHCI_USB_MAX_CHILDREN )
			{
				nport = -1;
				result = -EPIPE;
				break;
			}

			/* whoever resets or resumes must GetPortStatus to
			 * complete it!!
			 */
			
			if( hcd_desc->resuming && time_after_eq(jiffies, hcd_desc->re_timeout) )
			{
				// complete resume
				hcd_desc->port_status[nport] |= (USB_PORT_STAT_C_SUSPEND << 16);
				hcd_desc->port_status[nport] &= ~USB_PORT_STAT_SUSPEND;
			}

			if( (hcd_desc->port_status[nport]&USB_PORT_STAT_RESET) != 0 )
			{
				if( time_after_eq(jiffies, hcd_desc->re_timeout) )
				{
					TRACE("usbd_vhci_hub_control: completing reset\n");
					// complete reset
					hcd_desc->port_status[nport] |= (USB_PORT_STAT_C_RESET << 16);
					hcd_desc->port_status[nport] &= ~USB_PORT_STAT_RESET;

					if( hcd_desc->port_devices[nport] )
					{
						hcd_desc->port_devices[nport]->address = 0;
						hcd_desc->port_status[nport] |= USB_PORT_STAT_ENABLE;

						switch(hcd_desc->port_devices[nport]->speed)
						{
							case USB_SPEED_HIGH:
								hcd_desc->port_status[nport] |= USB_PORT_STAT_HIGH_SPEED;
								break;
							case USB_SPEED_LOW:
								hcd_desc->port_status[nport] |= USB_PORT_STAT_LOW_SPEED;
								break;
							default:
								break;
						}
					}
				} else
				{
					TRACE("usbd_vhci_hub_control: reset in progress\n");
				}
			}

			/*set_link_state (hcd_desc);*/
			((__le16 *)buf)[0] = cpu_to_le16(hcd_desc->port_status[nport]);
			((__le16 *)buf)[1] = cpu_to_le16(hcd_desc->port_status[nport] >> 16);
			break;

		case SetPortFeature:
			TRACE("usbd_vhci_hub_control: SetPortFeature\n");

			nport = (wIndex&0xff)-1;
			TRACE("usbd_vhci_hub_control: port=%d\n", nport);

			if( nport < 0 || nport >= VHCI_USB_MAX_CHILDREN )
			{
				nport = -1;
				result = -EPIPE;
				break;
			}

			switch( wValue )
			{
				case USB_PORT_FEAT_SUSPEND:
					TRACE("usbd_vhci_hub_control: Set SUSPEND\n");
					/*if(hcd_desc->active) 
					{
						hcd_desc->port_status |= USB_PORT_STAT_SUSPEND;

						set_link_state (hcd_desc);
						if (((1 << USB_DEVICE_B_HNP_ENABLE) & hcd_desc->devstatus) != 0)
							dev_dbg (hcd_descmy_dev(hcd_desc), "no HNP yet!\n");
					}*/
					break;
				case USB_PORT_FEAT_POWER:
					TRACE("usbd_vhci_hub_control: Set POWER\n");
					hcd_desc->port_status[nport] |= USB_PORT_STAT_POWER;
					/*set_link_state (hcd_desc);*/
					break;
				case USB_PORT_FEAT_RESET:
					TRACE("usbd_vhci_hub_control: Set RESET\n");
					/* if it's already enabled, disable */
					hcd_desc->port_status[nport] &= ~(USB_PORT_STAT_ENABLE
							| USB_PORT_STAT_LOW_SPEED
							| USB_PORT_STAT_HIGH_SPEED);
					//hcd_desc->devstatus = 0;
					/* 50msec reset signaling */
					hcd_desc->re_timeout = jiffies + msecs_to_jiffies(50);
{
		PUNRB punrb;
		struct vhci_device_descriptor *pdev = hcd_desc->port_devices[nport];

		if(pdev && pdev->address > 0)
		{
			struct waitable_queue_entry *pqentry = usbd_wq_alloc_entry(sizeof(struct vhci_queue_data), GFP_ATOMIC);

			punrb = kmalloc(sizeof(UNRB_RESET_PORT), GFP_ATOMIC);

			if( !punrb )
			{
				TRACE("usbd_vhci_hub_control: no memory for unrb\n");
			} else
			{
				((struct vhci_queue_data*)pqentry->data)->unique_id = usbd_get_unique_id();
				((struct vhci_queue_data*)pqentry->data)->purb = NULL;
				((struct vhci_queue_data*)pqentry->data)->punrb = punrb;

				punrb->Header.UniqueId = ((struct vhci_queue_data*)pqentry->data)->unique_id;
				punrb->Header.Size= sizeof(UNRB_RESET_PORT);
				punrb->Header.Function = UNRB_FUNCTION_RESET_PORT;
				punrb->Header.Status = 0;
				punrb->Header.Context = nport;

				usbd_wq_add_tail_locked(&pdev->queue_pending_transfer_unrb, pqentry);
			}
		} else
		{
			if(pdev && pdev->address <= 0)
			{
				TRACE("usbd_vhci_hub_control: address is null\n");
			} else
			{
				TRACE("usbd_vhci_hub_control: no pdev\n");
			}
		}
}

					/* FALLS THROUGH */
				default:
					TRACE("usbd_vhci_hub_control: Set 0x%04X\n", wValue);
					if( (hcd_desc->port_status[nport]&USB_PORT_STAT_POWER) != 0)
					{
						hcd_desc->port_status[nport] |= (1 << wValue);
						/*set_link_state (hcd_desc);*/
					}
			}
			break;

		case ClearPortFeature:
			TRACE("usbd_vhci_hub_control: ClearPortFeature\n");

			nport = (wIndex&0xff)-1;
			TRACE("usbd_vhci_hub_control: port=%d\n", nport);

			if( nport < 0 || nport >= VHCI_USB_MAX_CHILDREN )
			{
				nport = -1;
				result = -EPIPE;
				break;
			}

			switch (wValue) 
			{
				case USB_PORT_FEAT_C_RESET:
					TRACE("usbd_vhci_hub_control: Clear C_RESET\n");
					hcd_desc->port_status[nport] &= ~(1 << wValue);
					break;
				case USB_PORT_FEAT_SUSPEND:
					TRACE("usbd_vhci_hub_control: Clear SUSPEND\n");
					if( hcd_desc->port_status[nport]&USB_PORT_STAT_SUSPEND )
					{
						/* 20msec resume signaling */
						hcd_desc->resuming = 1;
						hcd_desc->re_timeout = jiffies + msecs_to_jiffies(20);
					}
					break;
				case USB_PORT_FEAT_POWER:
					TRACE("usbd_vhci_hub_control: Clear POWER\n");
					if( hcd_desc->port_status[nport]&USB_PORT_STAT_POWER )
					{
						TRACE("usbd_vhci_hub_control: power off\n");
						hcd_desc->resuming = 0;
					}
					/* FALLS THROUGH */
				default:
					TRACE("usbd_vhci_hub_control: Clear 0x%04X\n", wValue);
					hcd_desc->port_status[nport] &= ~(1 << wValue);
					/*set_link_state(hcd_desc);*/
			}
			break;


		default:
			TRACE("usbd_vhci_hub_control: Req=0x%04X Value=0x%04X Index=0x%04Xi Length=%d\n", typeReq, wValue, wIndex, wLength);
			/* "protocol stall" on error */
			result = -EPIPE;
	}
	spin_unlock_irqrestore(&hcd_desc->lock, flags);

	if( nport != -1 && (hcd_desc->port_status[nport]&PORT_C_MASK) != 0 )
		usb_hcd_poll_rh_status(hcd);

	TRACE("usbd_vhci_hub_control-- result = %d\n", result);

	return result;
}

static int usbd_vhci_hub_status_data(struct usb_hcd *hcd, char *buf)
{
	int result = 0;
	struct vhci_hcd_descriptor *hcd_desc = get_hcd_data(hcd);
	unsigned long *changed_ports_mask = (unsigned long*)buf;
	unsigned long flags;

	TRACE("usbd_vhci_hub_status_data++\n");

	spin_lock_irqsave(&hcd_desc->lock, flags);

	do /*while(0)*/
	{
		int i;
		int changed_ports = 0;

		if( !test_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags) ) break;

		/*if (hcd_desc->resuming && time_after_eq (jiffies, hcd_desc->re_timeout)) {
			hcd_desc->port_status |= (USB_PORT_STAT_C_SUSPEND << 16);
			hcd_desc->port_status &= ~USB_PORT_STAT_SUSPEND;
			set_link_state (hcd_desc);
		}*/

		for(i=0; i < VHCI_USB_MAX_CHILDREN; i++)
		{
			if( hcd_desc->port_status[i] & PORT_C_MASK )
			{
				TRACE("usbd_vhci_hub_status_data: port %d has changed. New port status is 0x%08lX\n", i, hcd_desc->port_status[i]);
				/* we modify buffer only when there are changed ports */
				if( changed_ports == 0 ) *changed_ports_mask = 0;
				changed_ports = 1;
				*changed_ports_mask |= 1 << ( i + 1);
			}
		}

		if (changed_ports)
		{
			result = VHCI_USB_MAX_CHILDREN_BYTES;
			if (hcd_desc->suspended)
  			{
		    		usb_hcd_resume_root_hub (hcd);
			}
		}

	} while(0);

	spin_unlock_irqrestore(&hcd_desc->lock, flags);

	TRACE("usbd_vhci_hub_status_data-- result = %d mask = 0x%08X\n", result, (u32)*changed_ports_mask);

	return result;
}

static int usbd_vhci_bus_suspend(struct usb_hcd *hcd)
{
	//struct vhci_hcd_descriptor *hcd_desc = get_hcd_data(hcd);
	struct vhci_hcd_descriptor *hcd_desc = get_hcd_data(hcd);
	hcd_desc->suspended = 1;
	TRACE("usbd_vhci_bus_suspend\n");
	return 0;
}

static int usbd_vhci_bus_resume(struct usb_hcd *hcd)
{
	//struct vhci_hcd_descriptor *hcd_desc = get_hcd_data(hcd);
	struct vhci_hcd_descriptor *hcd_desc = get_hcd_data(hcd);
	hcd_desc->suspended = 0;
	TRACE("usbd_vhci_bus_resume\n");
	return 0;
}


///////////////////////////////////////////////////////////////////////////////
// VHCI device object implementation

int usbd_vhci_device_init_module(void)
{
	int result = 0;

	TRACE("usbd_vhci_device_init_module: ++\n");

	do /*while(0)*/
	{
		INIT_LIST_HEAD(&s_device_list);
		spin_lock_init(&s_device_list_lock);

		/* register this driver with the platform subsystem */
		result = platform_driver_register(&usbd_vhci_platform_driver);

		if( result != 0 )
		{
			TRACE("usbd_vhci_device_init_module: platform_driver_register failed. Error number %d\n", result);
			break;
		}

		result = platform_device_register(&usbd_vhci_platform_device);
		if( result != 0 )
		{
			TRACE("usbd_vhci_device_init_module: platform_device_register failed. Error number %d\n", result);
			break;
		}

	} while(0);

	TRACE("usbd_vhci_device_init_module: -- result = %d\n", result);
	return result;
}

void usbd_vhci_device_deinit_module(void)
{
	TRACE("usbd_vhci_device_deinit_module: ++\n");

	spin_lock(&s_device_list_lock);
	while( !list_empty(&s_device_list) )
	{
		struct vhci_device_descriptor *pdev = (struct vhci_device_descriptor *)s_device_list.next;
		usbd_vhci_device_reference(pdev);
		spin_unlock(&s_device_list_lock);
		usbd_vhci_device_destroy(pdev);
		usbd_vhci_device_dereference(pdev);
		spin_lock(&s_device_list_lock);
	}
	spin_unlock(&s_device_list_lock);

	platform_device_unregister(&usbd_vhci_platform_device);
	platform_driver_unregister(&usbd_vhci_platform_driver);
	TRACE("usbd_vhci_device_deinit_module: --\n");
}

struct vhci_device_descriptor *usbd_vhci_device_create(void)
{
	struct vhci_device_descriptor *pdesc;

	do /*while(0)*/
	{
	   	pdesc = kmalloc(sizeof(*pdesc), GFP_KERNEL);

   		if( !pdesc )
		{
			TRACE("usbd_vhci_device_create: kmalloc failed.\n");
			break;
		}

	  	memset(pdesc, 0, sizeof(*pdesc));

	  	pdesc->pminor = usbd_mv_alloc_descriptor(pdesc);

	  	if( !pdesc->pminor ) 
		{
			TRACE("usbd_vhci_device_create: usbd_mv_alloc_descriptor failed.\n");
			break;
		}

	    usbd_cdev_alloc_minor(pdesc->pminor);

		if( pdesc->pminor->minor == -1 )
		{
			TRACE("usbd_vhci_device_create: usbd_cdev_alloc_minor failed.\n");
			break;
		}

		usbd_wq_init(&pdesc->queue_pending_transfer_unrb);
		usbd_wq_init(&pdesc->queue_pending_completion_unrb);

		kref_init(&pdesc->reference);
		pdesc->parent = g_vhci_hcd;
		pdesc->port = -1;

		spin_lock(&s_device_list_lock);
		list_add(&pdesc->list, &s_device_list);
		spin_unlock(&s_device_list_lock);

		return pdesc;

 	} while(0);

	if( pdesc ) 
	{
	  	if( pdesc->pminor ) 
		{
			usbd_cdev_free_minor(pdesc->pminor->minor); 
			usbd_mv_free_descriptor(pdesc->pminor);
		}

		kfree(pdesc);
	}

	return NULL;
}

void usbd_vhci_device_destroy(struct vhci_device_descriptor *pdesc)
{
	TRACE("usbd_vhci_device_destroy\n");
	if(!pdesc) return;

	usbd_vhci_device_disconnect(pdesc);

	spin_lock(&s_device_list_lock);
	list_del(&pdesc->list);
	spin_unlock(&s_device_list_lock);

	usbd_wq_deinit(&pdesc->queue_pending_transfer_unrb);
	usbd_wq_deinit(&pdesc->queue_pending_completion_unrb);

	usbd_vhci_device_dereference(pdesc);
}

void usbd_vhci_device_connect(struct vhci_device_descriptor *pdesc)
{
	unsigned long flags;
	int port;
	struct vhci_hcd_descriptor *hcd_desc = get_hcd_data(pdesc->parent);

	TRACE("usbd_vhci_device_connect: ++\n");

	spin_lock_irqsave(&hcd_desc->lock, flags);

	for(port=0; port<VHCI_USB_MAX_CHILDREN; port++)
	{
		if( hcd_desc->port_devices[port] == NULL )
		{
			TRACE("usbd_vhci_device_connect: found free port %d\n", port);
			usbd_vhci_device_reference(pdesc);

			pdesc->port = port;
			pdesc->address = -1;
			hcd_desc->port_devices[port] = pdesc;
			hcd_desc->port_status[port] |= USB_PORT_STAT_CONNECTION;
			hcd_desc->port_status[port] |= (USB_PORT_STAT_C_CONNECTION << 16);

			switch(pdesc->speed) 
			{
				case USB_SPEED_HIGH:
					hcd_desc->port_status[pdesc->port] |= USB_PORT_STAT_HIGH_SPEED;
					break;
				case USB_SPEED_LOW:
					hcd_desc->port_status[pdesc->port] |= USB_PORT_STAT_LOW_SPEED;
					break;
				default:
					break;
			}

			break;
		}		
	}

	spin_unlock_irqrestore(&hcd_desc->lock, flags);

	usb_hcd_poll_rh_status(pdesc->parent);
	TRACE("usbd_vhci_device_connect: --\n");
}

void usbd_vhci_device_disconnect(struct vhci_device_descriptor *pdesc)
{
	unsigned long flags;
	struct vhci_hcd_descriptor *hcd_desc = get_hcd_data(pdesc->parent);

	TRACE("usbd_vhci_device_disconnect: ++\n");

	if( pdesc->port != -1 )
	{
		TRACE("usbd_vhci_device_disconnect: disconnecting port %d\n", pdesc->port);

		spin_lock_irqsave(&hcd_desc->lock, flags);
		hcd_desc->port_status[pdesc->port] &= ~(USB_PORT_STAT_CONNECTION|USB_PORT_STAT_ENABLE|USB_PORT_STAT_HIGH_SPEED|USB_PORT_STAT_LOW_SPEED);
		hcd_desc->port_status[pdesc->port] |= (USB_PORT_STAT_C_CONNECTION << 16);
		hcd_desc->port_devices[pdesc->port] = NULL;
		pdesc->port = -1;
		spin_unlock_irqrestore(&hcd_desc->lock, flags);

		usb_hcd_poll_rh_status(pdesc->parent);

		usbd_vhci_device_cancel_all_urbs(pdesc);

		/* remove the reference that was added on device connection */
		usbd_vhci_device_dereference(pdesc);
	}
	TRACE("usbd_vhci_device_disconnect: --\n");
}


struct vhci_device_descriptor *usbd_vhci_device_find1(unsigned long hserver, unsigned long hdevice)
{
	struct vhci_device_descriptor *pdesc = NULL;
	struct list_head *entry;

	TRACE("usbd_vhci_device_find1: ++\n");

	spin_lock(&s_device_list_lock);
	list_for_each(entry, &s_device_list)
	{
		pdesc = (struct vhci_device_descriptor *)entry;

        if( pdesc->hserver == hserver && pdesc->hdevice == hdevice )
        {
			usbd_vhci_device_reference(pdesc);
			spin_unlock(&s_device_list_lock);
			TRACE("usbd_vhci_device_find1: -- found!\n");
            return pdesc;
		}
	}
	spin_unlock(&s_device_list_lock);

	TRACE("usbd_vhci_device_find1: -- not found\n");
	return NULL;
}

struct vhci_device_descriptor *usbd_vhci_device_find2(struct usb_hcd *hcd, int address)
{
	struct vhci_hcd_descriptor *hcd_desc = get_hcd_data(hcd);
	struct vhci_device_descriptor *pdesc = NULL;
	struct list_head *entry;

	TRACE("usbd_vhci_device_find2(): ++ address = %d\n", address);

	if( hcd_desc && address >= 0 )
	{
		spin_lock(&s_device_list_lock);
		list_for_each(entry, &s_device_list)
		{
			pdesc = (struct vhci_device_descriptor *)entry;

	  	    if( pdesc->address == address )
	        {
				if( pdesc->port != -1 )
				{
					usbd_vhci_device_reference(pdesc);
					spin_unlock(&s_device_list_lock);
					TRACE("usbd_vhci_device_find2: -- found!\n");
   	         		return pdesc;
				}
			}
		}
		spin_unlock(&s_device_list_lock);
	}

	TRACE("usbd_vhci_device_find2: -- not found\n");
	return NULL;
}

void usbd_vhci_device_complete_urb(struct urb* purb, int status)
{
	struct vhci_device_descriptor *pdev = purb->hcpriv;

	TRACE("usbd_vhci_device_complete_urb: ++ urb=0x%p\n", purb);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24)
	if( status != -1 )
	{
		/* set status if urb has not been cancelled yet */
		spin_lock(&purb->lock);
		if( purb->status == -EINPROGRESS ) purb->status = status;
		spin_unlock (&purb->lock);
	}
#endif

	purb->hcpriv = NULL;

	if( pdev )
	{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
		usb_hcd_giveback_urb(pdev->parent, purb, NULL);
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24)
		usb_hcd_giveback_urb(pdev->parent, purb);
#else
		usb_hcd_giveback_urb(pdev->parent, purb, status);
#endif
		/* remove reference added when the urb was submitted */
		usbd_vhci_device_dereference(pdev);
	}
	TRACE("usbd_vhci_device_complete_urb: --\n");
}

struct waitable_queue_entry *usbd_vhci_device_dequeue_urb(struct vhci_device_descriptor *pdev, struct urb* purb)
{
	unsigned long flags, flags2;
	struct waitable_queue_entry *pqentry;

	usbd_wq_lock(&pdev->queue_pending_transfer_unrb,flags);
	usbd_wq_lock(&pdev->queue_pending_completion_unrb,flags2);

	pqentry = usbd_wq_get_first(&pdev->queue_pending_completion_unrb);
	while(pqentry)
	{
		struct vhci_queue_data *pqdata = (struct vhci_queue_data *)pqentry->data;

		if( pqdata->purb == purb )
		{
			usbd_wq_remove_entry(pqentry);
			break;
		}
		pqentry = usbd_wq_get_next(&pdev->queue_pending_completion_unrb, pqentry);
	}

	if(pqentry == NULL)
	{
		pqentry = usbd_wq_get_first(&pdev->queue_pending_transfer_unrb);
		while(pqentry)
		{
			struct vhci_queue_data *pqdata = (struct vhci_queue_data *)pqentry->data;

			if( pqdata->purb == purb )
			{
				usbd_wq_remove_entry(pqentry);
				break;
			}
			pqentry = usbd_wq_get_next(&pdev->queue_pending_transfer_unrb, pqentry);
		}
	}

	usbd_wq_unlock(&pdev->queue_pending_completion_unrb,flags2);
	usbd_wq_unlock(&pdev->queue_pending_transfer_unrb,flags);

	return pqentry;
}

void usbd_vhci_device_cancel_all_urbs(struct vhci_device_descriptor *pdev)
{
	unsigned long flags, flags2;
	struct waitable_queue_entry *pqentry;
	struct waitable_queue tempqueue;

	usbd_wq_init(&tempqueue);

	usbd_wq_lock(&pdev->queue_pending_transfer_unrb,flags);
	usbd_wq_lock(&pdev->queue_pending_completion_unrb,flags2);

	while( !usbd_wq_is_empty(&pdev->queue_pending_transfer_unrb) )
	{
		usbd_wq_add_tail(
			&tempqueue, 
			usbd_wq_remove_head(&pdev->queue_pending_transfer_unrb));
	}

	while( !usbd_wq_is_empty(&pdev->queue_pending_completion_unrb) )
	{
		usbd_wq_add_tail(
			&tempqueue, 
			usbd_wq_remove_head(&pdev->queue_pending_completion_unrb));
	}

	usbd_wq_unlock(&pdev->queue_pending_completion_unrb,flags2);
	usbd_wq_unlock(&pdev->queue_pending_transfer_unrb,flags);

	while( !usbd_wq_is_empty(&tempqueue) )
	{
		struct vhci_queue_data *pqdata;

		pqentry = usbd_wq_remove_head(&tempqueue);
		pqdata = (struct vhci_queue_data *)pqentry->data;

		if( pqdata->purb )
			usbd_vhci_device_complete_urb(pqdata->purb, -ENODEV);

		if( pqdata->punrb )
			kfree(pqdata->punrb);

		usbd_wq_free_entry(pqentry);
	}

	usbd_wq_deinit(&tempqueue);
}



void usbd_vhci_device_free(struct kref* pkref)
{
	struct vhci_device_descriptor *pdesc = container_of(pkref, struct vhci_device_descriptor, reference);

	TRACE("usbd_vhci_device_free(pdev=0x%p)\n", pdesc);

	if( pdesc->pminor )
	{
		usbd_cdev_free_minor(pdesc->pminor->minor);
		usbd_mv_free_descriptor(pdesc->pminor);
	}

	kfree(pdesc);
}

#endif // _USBD_ENABLE_VHCI_
