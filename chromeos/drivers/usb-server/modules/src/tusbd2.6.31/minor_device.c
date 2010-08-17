/*
 *  tusbd/minor_device.c
 *
 *  Copyright (C) 2007 IncentivesPro
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifdef _USBD_ENABLE_STUB_

#include "usbd.h"

int usbd_md_open(void *);
int usbd_md_release(void *);
unsigned int usbd_md_poll(void *, struct file *, poll_table *wait);
int usbd_md_ioctl(void *context, unsigned int cmd, unsigned long arg);

int usbd_md_handle_getdescriptor(struct usbdevice_descriptor *pdev, struct usb_device *pudev, PUNRB punrb, struct waitable_queue_entry *pqentry);
int usbd_md_handle_selectconfiguration(struct usbdevice_descriptor *pdev, struct usb_device *pudev, PUNRB punrb);
int usbd_md_handle_selectinterface(struct usbdevice_descriptor *pdev, struct usb_device *pudev, PUNRB punrb);
int usbd_md_handle_controltransfer(struct usbdevice_descriptor *pdev, struct usb_device *pudev, PUNRB punrb, struct waitable_queue_entry *pqentry);
int usbd_md_handle_bulktransfer(struct usbdevice_descriptor *pdev, struct usb_device *pudev, PUNRB punrb, struct waitable_queue_entry *pqentry);
int usbd_md_handle_solid_bulktransfer(struct usbdevice_descriptor *pdev, struct usb_device *pudev, PUNRB punrb, struct waitable_queue_entry *pqentry);
int usbd_md_handle_partitioned_bulktransfer(struct usbdevice_descriptor *pdev, struct usb_device *pudev, PUNRB punrb, struct waitable_queue_entry *pqentry);
int usbd_md_handle_interrupttransfer(struct usbdevice_descriptor *pdev, struct usb_device *pudev, PUNRB punrb, struct waitable_queue_entry *pqentry);
int usbd_md_handle_isochtransfer(struct usbdevice_descriptor *pdev, struct usb_device *pudev, PUNRB punrb, struct waitable_queue_entry *pqentry);
int usbd_md_handle_solid_isochtransfer(struct usbdevice_descriptor *pdev, struct usb_device *pudev, PUNRB punrb, struct waitable_queue_entry *pqentry);
int usbd_md_handle_partitioned_isochtransfer(struct usbdevice_descriptor *pdev, struct usb_device *pudev, PUNRB punrb, struct waitable_queue_entry *pqentry);
int usbd_md_handle_clearstall(struct usbdevice_descriptor *pdev, struct usb_device *pudev, PUNRB punrb);
int usbd_md_handle_getcurrentframenumber(struct usbdevice_descriptor *pdev, struct usb_device *pudev, PUNRB punrb);
int usbd_md_handle_getportstatus(struct usbdevice_descriptor *pdev, struct usb_device *pudev, PUNRB punrb);
int usbd_md_handle_resetport(struct usbdevice_descriptor *pdev, struct usb_device *pudev, PUNRB punrb);
int usbd_md_handle_cancel(struct usbdevice_descriptor *pdev, struct usb_device *pudev, PUNRB punrb);
int usbd_md_handle_abortendpoint(struct usbdevice_descriptor *pdev, struct usb_device *pudev, PUNRB punrb);


#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
void usbd_md_urb_complete_getdescriptor(struct urb* purb, struct pt_regs* ptregs);
void usbd_md_urb_complete_controltransfer(struct urb* purb, struct pt_regs* ptregs);
void usbd_md_urb_complete_solid_bulktransfer(struct urb* purb, struct pt_regs* ptregs);
void usbd_md_urb_complete_interrupttransfer(struct urb* purb, struct pt_regs* ptregs);
void usbd_md_urb_complete_solid_isochtransfer(struct urb* purb, struct pt_regs* ptregs);
#else
void usbd_md_urb_complete_getdescriptor(struct urb* purb);
void usbd_md_urb_complete_controltransfer(struct urb* purb);
void usbd_md_urb_complete_solid_bulktransfer(struct urb* purb);
void usbd_md_urb_complete_interrupttransfer(struct urb* purb);
void usbd_md_urb_complete_solid_isochtransfer(struct urb* purb);
#endif

void usbd_md_urb_complete_partitioned_bulktransfer(struct urb_chain* pchain);
void usbd_md_urb_complete_partitioned_isochtransfer(struct urb_chain* pchain);

struct minor_descriptor *usbd_md_alloc_descriptor(struct usbdevice_descriptor *pdev)
{
	struct minor_descriptor *pdesc;
   	pdesc = kmalloc(sizeof(*pdesc), GFP_KERNEL);

   	if( pdesc )
   	{
	  	memset(pdesc, 0, sizeof(*pdesc));

	  	pdesc->ops.open = usbd_md_open;
	  	pdesc->ops.release = usbd_md_release;
	  	pdesc->ops.poll = usbd_md_poll;
	  	pdesc->ops.ioctl = usbd_md_ioctl;

	  	pdesc->context = pdev;
	  	pdesc->fifo = usbd_fifo_create();

	  	if( !pdesc->fifo )
	  	{
	  		kfree(pdesc);
	  		pdesc = NULL;
	  	}
  	}
   	return pdesc;
}

void usbd_md_free_descriptor(struct minor_descriptor *pdesc)
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

int usbd_md_read_unrb(struct usbdevice_descriptor *pdev, void __user *buf)
{
	ssize_t result = 0;
	unsigned long flags; 
	struct waitable_queue_entry *pqentry;

	TRACE("usbd_md_read_unrb: ++\n");

	usbd_wq_lock(&pdev->queue_completed_unrb, flags);
	pqentry = usbd_wq_remove_head(&pdev->queue_completed_unrb);
	usbd_wq_unlock(&pdev->queue_completed_unrb, flags);


	if( pqentry )
	{
		usbd_copy_n_free_unrb(buf, 1024*1024, pqentry);

	} else
	{
		TRACE("usbd_md_read_unrb: no more data\n");
		result = -ENODATA;
	}

	TRACE("usbd_md_read_unrb: -- returning %d\n", result);
	return result;
}

int usbd_md_write_unrb(struct usbdevice_descriptor *pdev, void __user *buf)
{
	ssize_t result = 0;
	PUNRB punrb = NULL;
	struct waitable_queue_entry *pqentry;
	struct usb_device *pudev;
	int pending = 0;

	TRACE("usbd_md_write_unrb: ++\n");

	if( (pudev=usbd_usbdevice_pudev_get(pdev)) == NULL ) 
	{
		TRACE_CRITICAL("usbd_md_write: -- no device\n");
		return -ENODEV;
	}

	do
	{
		pqentry = usbd_alloc_n_copy_unrb(buf, 1024*1024, pdev, pudev);

		if( !pqentry )
		{
			TRACE("usbd_md_write_unrb: cannot allocate UNRB\n");
			result = -ENOMEM;
			break;
		}

		punrb = (PUNRB)(pqentry->data+sizeof(struct usbd_usb_request));

		dump_unrb(punrb);

		switch( punrb->Header.Function )
		{
			case UNRB_FUNCTION_GET_DESCRIPTOR:
				usbd_wq_add_tail_locked(&pdev->queue_pending_unrb, pqentry);

				result = usbd_md_handle_getdescriptor(pdev, pudev, punrb, pqentry);

				if( result == 0 ) 
				{
					pending = 1; // unrb is not yet completed
				} else
				{
					if( result > 0 ) 
						result = 0;

					usbd_wq_remove_entry_locked(&pdev->queue_pending_unrb, pqentry);
				}
				break;

			case UNRB_FUNCTION_SELECT_CONFIGURATION:
				result = usbd_md_handle_selectconfiguration(pdev, pudev, punrb);
				if( result > 0 ) 
					result = 0;
				break;

			case UNRB_FUNCTION_SELECT_INTERFACE:
				result = usbd_md_handle_selectinterface(pdev, pudev, punrb);
				break;

			case UNRB_FUNCTION_CONTROL_TRANSFER:
				usbd_wq_add_tail_locked(&pdev->queue_pending_unrb, pqentry);

				result = usbd_md_handle_controltransfer(pdev, pudev, punrb, pqentry);

				if( result == 0 ) 
				{
					pending = 1; // unrb is not yet completed
				} else
				{
					if( result > 0 ) 
						result = 0;

					usbd_wq_remove_entry_locked(&pdev->queue_pending_unrb, pqentry);
				}
				break;

			case UNRB_FUNCTION_BULK_TRANSFER:
				usbd_wq_add_tail_locked(&pdev->queue_pending_unrb, pqentry);

				result = usbd_md_handle_bulktransfer(pdev, pudev, punrb, pqentry);

				if( result == 0 ) 
				{
					pending = 1; // unrb is not yet completed
				} else
				{
					if( result > 0 ) 
						result = 0;

					usbd_wq_remove_entry_locked(&pdev->queue_pending_unrb, pqentry);
				}
				break;

			case UNRB_FUNCTION_INTERRUPT_TRANSFER:
				usbd_wq_add_tail_locked(&pdev->queue_pending_unrb, pqentry);

				result = usbd_md_handle_interrupttransfer(pdev, pudev, punrb, pqentry);

				if( result == 0 ) 
				{
					pending = 1; // unrb is not yet completed
				} else
				{
					if( result > 0 ) 
						result = 0;

					usbd_wq_remove_entry_locked(&pdev->queue_pending_unrb, pqentry);
				}
				break;

			case UNRB_FUNCTION_ISOCH_TRANSFER:
				usbd_wq_add_tail_locked(&pdev->queue_pending_unrb, pqentry);

				result = usbd_md_handle_isochtransfer(pdev, pudev, punrb, pqentry);

				if( result == 0 ) 
				{
					pending = 1; // unrb is not yet completed
				} else
				{
					if( result > 0 ) 
						result = 0;

					usbd_wq_remove_entry_locked(&pdev->queue_pending_unrb, pqentry);
				}
				break;

			case UNRB_FUNCTION_CLEAR_STALL:
				result = usbd_md_handle_clearstall(pdev, pudev, punrb);
				break;

			case UNRB_FUNCTION_GET_CURRENT_FRAME_NUMBER:
				result = usbd_md_handle_getcurrentframenumber(pdev, pudev, punrb);
				break;

			case UNRB_FUNCTION_GET_PORT_STATUS:
				result = usbd_md_handle_getportstatus(pdev, pudev, punrb);
				break;

			case UNRB_FUNCTION_RESET_PORT:
				result = usbd_md_handle_resetport(pdev, pudev, punrb);
				break;

			case UNRB_FUNCTION_CANCEL:
				result = usbd_md_handle_cancel(pdev, pudev, punrb);
				break;

			case UNRB_FUNCTION_ABORT_ENDPOINT:
				result = usbd_md_handle_abortendpoint(pdev, pudev, punrb);
				break;

			default:
				result = -EINVAL;
				break;
		}

	} while(0);

	if( !pending && pqentry )
	{
		if( result != 0 ) 
		{
			// there was some non-usb error while processing unrb
			usbd_wq_free_entry(pqentry); 
		}
		else
		{
			TRACE("UNRB not pending\n");
           	dump_unrb(punrb);
			// add to the queue of completed unrbs
			usbd_wq_add_tail_locked(&pdev->queue_completed_unrb, pqentry);
		}
	}

	usbd_usbdevice_pudev_put(pdev, pudev);

	TRACE("usbd_md_write_unrb: -- result=%d\n", result);
	return result;
}

int usbd_md_ioctl(void *context, unsigned int cmd, unsigned long arg)
{
	struct usbdevice_descriptor *pdev = context;
	ssize_t result = 0;

	TRACE("usbd_md_ioctl: ++ cmd=%d arg=%lu\n", cmd, arg);

	switch(cmd)
	{
		case 0: /* read unrb */
			result = usbd_md_read_unrb(pdev, (void __user *)arg);
			break;
		case 1: /* write unrb */
			result = usbd_md_write_unrb(pdev, (void __user *)arg);
			break;
		default:
			TRACE("usbd_md_ioctl: invalid request\n");
			result = -EINVAL;
			break;
	}

	TRACE("usbd_md_ioctl: -- result=%d\n", result);

	return result;
}

int usbd_md_open(void *context)
{
	int result = 0;
	struct usbdevice_descriptor *pdev = context;

	TRACE("usbd_md_open: ++\n");

	usbd_usbdevice_reference(pdev);

	TRACE("usbd_md_open: -- result=%d\n", result);
	return result;
}

int usbd_md_release(void *context)
{
	struct usbdevice_descriptor *pdev = context;
	TRACE("usbd_md_release\n");

	usbd_usbdevice_dereference(pdev);

	return 0;
}

unsigned int usbd_md_poll(void *context, struct file *filp, poll_table *wait)
{
	struct usbdevice_descriptor *pdev = context;
	int empty;
	unsigned long flags;

//	TRACE("usbd_md_poll\n");
	poll_wait(filp, &pdev->queue_completed_unrb.waitqueue, wait);

	usbd_wq_lock(&pdev->queue_completed_unrb, flags);
	empty = usbd_wq_is_empty(&pdev->queue_completed_unrb);
	usbd_wq_unlock(&pdev->queue_completed_unrb, flags);

	if( !empty )
	{
		TRACE("usbd_md_poll: queue not empty\n");
		return ((POLLOUT | POLLWRNORM) | (POLLIN | POLLRDNORM));
	}

	return (POLLOUT | POLLWRNORM);
}

int usbd_md_handle_getdescriptor(struct usbdevice_descriptor *pdev, struct usb_device *pudev, PUNRB punrb, struct waitable_queue_entry *pqentry)
{
	int result = 0;
	struct urb *purb = NULL;
	struct usb_ctrlrequest *pcr = NULL;
	struct usbd_usb_request* pureq = (struct usbd_usb_request*)pqentry->data;

	do
	{
		purb = usb_alloc_urb(0, GFP_KERNEL);
		if( !purb )
		{
			TRACE("usbd_md_handle_getdescriptor: usb_alloc_urb failed\n");
			result = -ENOMEM;
			break;
		}

	   	pcr = kmalloc(sizeof(*pcr), GFP_KERNEL);
		if( !pcr )
		{
			TRACE("usbd_md_handle_getdescriptor: usb_alloc_urb failed\n");
			result = -ENOMEM;
			break;
		}

		pcr->bRequest = USB_REQ_GET_DESCRIPTOR;
		pcr->bRequestType = USB_DIR_IN + ((punrb->DescriptorRequest.RequestType&0x03)<<5) + (punrb->DescriptorRequest.RequestRecipient&0x1F);
		pcr->wValue = cpu_to_le16((punrb->DescriptorRequest.DescType << 8) + punrb->DescriptorRequest.DescIndex);
		pcr->wIndex = cpu_to_le16(punrb->DescriptorRequest.LangId);
		pcr->wLength = cpu_to_le16(punrb->DescriptorRequest.BufferSize);

		usb_fill_control_urb(
			purb,
			pudev,
			usb_rcvctrlpipe(pudev, 0),
			(unsigned char*)pcr,
			((char*)punrb)+sizeof(punrb->DescriptorRequest),
			punrb->DescriptorRequest.BufferSize,
			usbd_md_urb_complete_getdescriptor,
			pqentry);

		pureq->purb = purb;
		pureq->pusbdevice = pdev;
		pureq->endpoint = 0;

		usbd_usbdevice_reference(pdev);

		result = usb_submit_urb(purb, GFP_KERNEL);

		if( result == -EMSGSIZE )
		{
		    result = 1;
		    punrb->Header.Status = -EMSGSIZE;
		}

		if( result != 0 )
		{
			usbd_usbdevice_dereference(pdev);
			TRACE("usbd_md_handle_getdescriptor: usb_submit_urb failed with error %d\n", result);
		}
		
	} while(0);

	if( result != 0 )
	{
		if( purb ) usb_free_urb(purb);
		if( pcr ) kfree(pcr);                                                                    
	}

	return result;
}

int usbd_md_handle_selectconfiguration(struct usbdevice_descriptor *pdev, struct usb_device *pudev, PUNRB punrb)
{
	int i, result;

	punrb->Header.Status = 0;

	if( punrb->SelectConfiguration.ConfigurationNum == 0 )
	    return 0;

	result = usbd_usbdevice_change_configuration(
		pdev,
		pudev,
		punrb->SelectConfiguration.ConfigurationNum,
		15000);

	if( result == -ERESTART )
	{
	    TRACE("usbd_md_handle_selectconfiguration: usbd_usbdevice_change_configuration interrupted\n");
		return result;
	}

	if( result < 0 )
	{
	    TRACE("usbd_md_handle_selectconfiguration: usbd_usbdevice_change_configuration returned %d\n", result);
		punrb->Header.Status = -1;
		return 0;
	}

	// pudev may have changed
	pudev = usbd_usbdevice_pudev_get(pdev);

	if( pudev == NULL )
	{
	    TRACE_CRITICAL("usbd_md_handle_selectconfiguration: device is null after changing configuration.\n");
	    return 0;
	}

	for(i=0; i < punrb->SelectConfiguration.NumAlternates; i++)
	{
		struct usb_interface *iface;
		struct usb_host_interface *alt;
		
		iface = usb_ifnum_to_if(
			    pudev, 
			    punrb->SelectConfiguration.Alternates[i].InterfaceNum);
		
		if( !iface ) 
		{
		    TRACE("usbd_md_handle_selectconfiguration: interface %d not found\n", 
				punrb->SelectConfiguration.Alternates[i].InterfaceNum);
		    continue;
		}
		
		
		alt = usb_altnum_to_altsetting(iface, punrb->SelectConfiguration.Alternates[i].AlternateNum);
		
		if( !alt )
		{
		    TRACE("usbd_md_handle_selectconfiguration: interface %d altsetting %d not found\n", 
				punrb->SelectConfiguration.Alternates[i].InterfaceNum,
				punrb->SelectConfiguration.Alternates[i].AlternateNum);
		    continue;
		}
		
		if( iface->cur_altsetting )
		{
			if( iface->num_altsetting == 1 ) 
			{
				TRACE("usbd_md_handle_selectconfiguration: interface %d has only one altsetting\n", 
			    	punrb->SelectConfiguration.Alternates[i].InterfaceNum);
				continue;
		    }
		    
		    if( alt->desc.bAlternateSetting == iface->cur_altsetting->desc.bAlternateSetting )
		    {
    			TRACE("usbd_md_handle_selectconfiguration: interface %d current altsetting already set\n", 
    			    punrb->SelectConfiguration.Alternates[i].InterfaceNum);
				continue;
		    }
		}

		result = usb_set_interface(
					pudev, 
					punrb->SelectConfiguration.Alternates[i].InterfaceNum, 
					punrb->SelectConfiguration.Alternates[i].AlternateNum);
		
		if( result != 0 )
		{
			TRACE("usbd_md_handle_selectconfiguration: usb_set_interface failed with error %d\n", result);
		}

		if( iface->cur_altsetting )
		{
			int j;
			int pipe;

			for(j=0;j<iface->cur_altsetting->desc.bNumEndpoints;j++)
			{
				if(iface->cur_altsetting->endpoint[j].desc.bEndpointAddress&0x80)
					pipe = usb_rcvisocpipe(
							pudev, 
							iface->cur_altsetting->endpoint[j].desc.bEndpointAddress&0x0F);
				else
					pipe = usb_sndisocpipe(
							pudev, 
							iface->cur_altsetting->endpoint[j].desc.bEndpointAddress&0x0F);

				usbd_usbdevice_set_frame_delta(pdev, pipe, 0);
			}
		}
	}

	usbd_usbdevice_pudev_put(pdev, pudev);

	return 0; // ignore usb_set_interface() result here
}

int usbd_md_handle_clearstall(struct usbdevice_descriptor *pdev, struct usb_device *pudev, PUNRB punrb)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,10)
	int pipe = 0, endp;
	struct usb_host_endpoint *ep;

	ep = (punrb->ClearStall.Flags&UNRB_FLAG_DIRECTION_IN) ? 
		pudev->ep_in[punrb->ClearStall.Endpoint]: 
		pudev->ep_out[punrb->ClearStall.Endpoint];

	if(ep)
	{
		switch(ep->desc.bmAttributes&USB_ENDPOINT_XFERTYPE_MASK)
		{
			case USB_ENDPOINT_XFER_ISOC:
				pipe = 
					(punrb->ClearStall.Flags&UNRB_FLAG_DIRECTION_IN) ? 
					usb_rcvisocpipe(pudev, punrb->ClearStall.Endpoint): 
					usb_sndisocpipe(pudev, punrb->ClearStall.Endpoint);
				break;
			case USB_ENDPOINT_XFER_BULK:
				pipe = 
					(punrb->ClearStall.Flags&UNRB_FLAG_DIRECTION_IN) ? 
					usb_rcvbulkpipe(pudev, punrb->ClearStall.Endpoint): 
					usb_sndbulkpipe(pudev, punrb->ClearStall.Endpoint);
				break;
			case USB_ENDPOINT_XFER_INT:
				pipe = 
					(punrb->ClearStall.Flags&UNRB_FLAG_DIRECTION_IN) ? 
					usb_rcvintpipe(pudev, punrb->ClearStall.Endpoint): 
					usb_sndintpipe(pudev, punrb->ClearStall.Endpoint);
				break;
			case USB_ENDPOINT_XFER_CONTROL:
			default:
				break;
		}
	}
			
#endif

	if(ep && (ep->desc.bmAttributes&USB_ENDPOINT_XFERTYPE_MASK) == USB_ENDPOINT_XFER_ISOC )
	{
		usbd_usbdevice_set_frame_delta(pdev, pipe, 0);
		endp = usb_pipeendpoint(pipe);
		
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,30)
		if (usb_pipein (pipe))
		    endp |= USB_DIR_IN;

                 usb_reset_endpoint(pudev, endp);
#else
                 usb_settoggle(pudev, endp, usb_pipeout(pipe), 0);
#endif
		punrb->Header.Status = 0;

	} else punrb->Header.Status = usb_clear_halt(pudev, pipe);

	if( punrb->Header.Status != 0 )
	{
		TRACE("usbd_md_handle_clearstall: usb_clear_halt failed with error %d\n", punrb->Header.Status);
	}

	return 0;
}

int usbd_md_handle_getcurrentframenumber(struct usbdevice_descriptor *pdev, struct usb_device *pudev, PUNRB punrb)
{
	punrb->Header.Status = 0;
	punrb->GetCurrentFrameNumber.FrameNumber = usb_get_current_frame_number(pudev);
	return 0;
}


int usbd_md_handle_selectinterface(struct usbdevice_descriptor *pdev, struct usb_device *pudev, PUNRB punrb)
{
	struct usb_interface *iface;
	struct usb_host_interface *alt;

	punrb->Header.Status = 0;

	do
	{
		iface = usb_ifnum_to_if(
			    pudev, 
			    punrb->SelectInterface.InterfaceNum);
		
		if( !iface ) 
		{
		    TRACE("usbd_md_handle_selectinterface: interface %d not found\n", 
				punrb->SelectInterface.AlternateNum);
		    break;
		}
		
		
		alt = usb_altnum_to_altsetting(iface, punrb->SelectInterface.AlternateNum);
		
		if( !alt )
		{
		    TRACE("usbd_md_handle_selectinterface: interface %d altsetting %d not found\n", 
				punrb->SelectInterface.InterfaceNum,
				punrb->SelectInterface.AlternateNum);
		    break;
		}
		
		if( iface->cur_altsetting )
		{
			if( iface->num_altsetting == 1 ) 
			{
				TRACE("usbd_md_handle_selectinterface: interface %d has only one altsetting\n", 
			    	punrb->SelectInterface.InterfaceNum);
				break;
		    }
		    
		    if( alt->desc.bAlternateSetting == iface->cur_altsetting->desc.bAlternateSetting )
		    {
    			TRACE("usbd_md_handle_selectinterface: interface %d current altsetting already set\n", 
    			    punrb->SelectInterface.InterfaceNum);
				break;
		    }
		}

		punrb->Header.Status = 
			usb_set_interface(
				pudev, 
				punrb->SelectInterface.InterfaceNum, 
				punrb->SelectInterface.AlternateNum);

		if( punrb->Header.Status != 0 )
		{
			TRACE("usbd_md_handle_selectinterface: usb_set_interface failed with error %d\n", punrb->Header.Status);
		}

		if( iface->cur_altsetting )
		{
			int j;
			int pipe;

			for(j=0;j<iface->cur_altsetting->desc.bNumEndpoints;j++)
			{
				if(iface->cur_altsetting->endpoint[j].desc.bEndpointAddress&0x80)
					pipe = usb_rcvisocpipe(
							pudev, 
							iface->cur_altsetting->endpoint[j].desc.bEndpointAddress&0x0F);
				else
					pipe = usb_sndisocpipe(
							pudev, 
							iface->cur_altsetting->endpoint[j].desc.bEndpointAddress&0x0F);

				usbd_usbdevice_set_frame_delta(pdev, pipe, 0);
			}
		}


	} while(0);

	return 0;
}

int usbd_md_handle_getportstatus(struct usbdevice_descriptor *pdev, struct usb_device *pudev, PUNRB punrb)
{
	enum usb_device_state state = pudev->state;

	punrb->Header.Status = 0;
	punrb->GetPortStatus.PortStatus = 0;

	if( state != USB_STATE_SUSPENDED && state != USB_STATE_NOTATTACHED )
	{
		punrb->GetPortStatus.PortStatus |= UNRB_PORT_STATUS_CONNECTED;

		if( state == USB_STATE_CONFIGURED ) 
			punrb->GetPortStatus.PortStatus |= UNRB_PORT_STATUS_ENABLED;
	}

	return 0;
}

int usbd_md_handle_resetport(struct usbdevice_descriptor *pdev, struct usb_device *pudev, PUNRB punrb)
{
	punrb->Header.Status = -1;

	if( usb_lock_device_for_reset(pudev, NULL) >= 0 )
	{
		punrb->Header.Status = usb_reset_device(pudev);
		usb_unlock_device(pudev);
	}

	if( punrb->Header.Status != 0 )
	{
		TRACE("usbd_md_handle_resetport: failed with error %d\n", punrb->Header.Status);
	}

	return 0;
}

int usbd_md_handle_cancel(struct usbdevice_descriptor *pdev, struct usb_device *pudev, PUNRB punrb)
{
	struct waitable_queue_entry *pentry;
	struct usbd_usb_request* pureq = NULL;
	unsigned long flags;
	
	punrb->Header.Status = 0;

	usbd_wq_lock(&pdev->queue_pending_unrb, flags);

	pentry = usbd_wq_get_first(&pdev->queue_pending_unrb);

	while(pentry)
	{
		PUNRB ptmpunrb = (PUNRB)(pentry->data+sizeof(struct usbd_usb_request));
		if( ptmpunrb->Header.UniqueId == punrb->Header.UniqueId )
		{
			pureq = (struct usbd_usb_request*)pentry->data;

			if(pureq->purb)
				usb_get_urb(pureq->purb);
			else
				usbd_uc_get(&pureq->urbchain);
			break;
		}
		pentry = usbd_wq_get_next(&pdev->queue_pending_unrb, pentry);
	}
	usbd_wq_unlock(&pdev->queue_pending_unrb, flags);

	if( pentry )
	{
		// if we have pentry - we have pureq too!
		if(pureq->purb)
		{
			usb_unlink_urb(pureq->purb);
			usb_put_urb(pureq->purb);
		}
		else
		{
			usbd_uc_cancel(&pureq->urbchain);
			usbd_uc_put(&pureq->urbchain);
		}
	} else
	{
		TRACE("usbd_md_handle_cancel: urb not found\n");
	}

	return 0;
}

int usbd_md_handle_abortendpoint(struct usbdevice_descriptor *pdev, struct usb_device *pudev, PUNRB punrb)
{
	struct waitable_queue_entry *pentry;
	struct usbd_usb_request* pureq = NULL;
	unsigned long flags;

	punrb->Header.Status = 0;

	/*
		we cannot call usb_unlink_urb when queue lock is held, so we should first 
		find the required entry, release the lock and unlink the urb.
		Then repeat this until no urbs are found.
	*/

	do
	{
		usbd_wq_lock(&pdev->queue_pending_unrb, flags);
	
		pentry = usbd_wq_get_first(&pdev->queue_pending_unrb);

		while(pentry)
		{
			pureq = (struct usbd_usb_request*)pentry->data;

			if( punrb->AbortEndpoint.Endpoint == pureq->endpoint )
			{
				// mark this unrb as processed
				pureq->endpoint = (uint8)-1;
				// prevent urbs from freeing when we release the spinlock
				if(pureq->purb)
					usb_get_urb(pureq->purb);
				else
					usbd_uc_get(&pureq->urbchain);
				break;

			}
			pentry = usbd_wq_get_next(&pdev->queue_pending_unrb, pentry);
		}
		usbd_wq_unlock(&pdev->queue_pending_unrb, flags);

		if(pentry)
		{
			// if we have pentry - we have pureq too!
			if(pureq->purb)
			{
				usb_unlink_urb(pureq->purb);
				usb_put_urb(pureq->purb);
			}
			else
			{
				usbd_uc_cancel(&pureq->urbchain);
				usbd_uc_put(&pureq->urbchain);
			}
		}

	} while(pentry);

	return 0;
}


int usbd_md_handle_controltransfer(struct usbdevice_descriptor *pdev, struct usb_device *pudev, PUNRB punrb, struct waitable_queue_entry *pqentry)
{
	int result = 0;
	struct urb *purb = NULL;
	struct usb_ctrlrequest *pcr = NULL;
	struct usbd_usb_request* pureq = (struct usbd_usb_request*)pqentry->data;

	do
	{
		int pipe;

		purb = usb_alloc_urb(0, GFP_KERNEL);
		if( !purb )
		{
			TRACE("usbd_md_handle_controltransfer: usb_alloc_urb failed\n");
			result = -ENOMEM;
			break;
		}

	   	pcr = kmalloc(sizeof(*pcr), GFP_KERNEL);
		if( !pcr )
		{
			TRACE("usbd_md_handle_controltransfer: usb_alloc_urb failed\n");
			result = -ENOMEM;
			break;
		}

		pcr->bRequestType = punrb->ControlTransfer.RequestType;
		pcr->bRequest = punrb->ControlTransfer.Request;
		pcr->wValue = cpu_to_le16(punrb->ControlTransfer.Value);
		pcr->wIndex = cpu_to_le16(punrb->ControlTransfer.Index);
		pcr->wLength = cpu_to_le16(punrb->ControlTransfer.BufferSize);

		pipe = 
			(punrb->ControlTransfer.Flags&UNRB_FLAG_DIRECTION_IN) ? 
			usb_rcvctrlpipe(pudev, punrb->ControlTransfer.Endpoint): 
			usb_sndctrlpipe(pudev, punrb->ControlTransfer.Endpoint);

		usb_fill_control_urb(
			purb,
			pudev,
			pipe,
			(unsigned char*)pcr,
			((char*)punrb)+sizeof(punrb->ControlTransfer),
			punrb->ControlTransfer.BufferSize,
			usbd_md_urb_complete_controltransfer,
			pqentry);

		pureq->purb = purb;
		pureq->pusbdevice = pdev;
		pureq->endpoint = punrb->ControlTransfer.Endpoint;

		usbd_usbdevice_reference(pdev);

		result = usb_submit_urb(purb, GFP_KERNEL);

		if( result == -EMSGSIZE )
		{
		    result = 1;
		    punrb->Header.Status = -EMSGSIZE;
		}

		if( result != 0 )
		{
			usbd_usbdevice_dereference(pdev);
			TRACE("usbd_md_handle_controltransfer: usb_submit_urb failed with error %d\n", result);
		}
		
	} while(0);

	if( result != 0 )
	{
		if( purb ) usb_free_urb(purb);
		if( pcr ) kfree(pcr);                                                                    
	}

	return result;
}

int usbd_md_handle_bulktransfer(struct usbdevice_descriptor *pdev, struct usb_device *pudev, PUNRB punrb, struct waitable_queue_entry *pqentry)
{
	struct usbd_usb_request *pureq = (struct usbd_usb_request *)pqentry->data;

	TRACE("usbd_md_handle_bulktransfer: buffer %p\n",pureq->buffer);

	if(pureq->buffer)
	{
		TRACE("usbd_md_handle_bulktransfer: solid\n");
		return usbd_md_handle_solid_bulktransfer(pdev, pudev, punrb, pqentry);
	} else
	{
		TRACE("usbd_md_handle_bulktransfer: partitioned\n");
	}

	return usbd_md_handle_partitioned_bulktransfer(pdev, pudev, punrb, pqentry);
}

int usbd_md_handle_solid_bulktransfer(struct usbdevice_descriptor *pdev, struct usb_device *pudev, PUNRB punrb, struct waitable_queue_entry *pqentry)
{
	int result = 0;
	struct urb *purb = NULL;
	struct usbd_usb_request* pureq = (struct usbd_usb_request*)pqentry->data;

	do
	{
		int pipe;

		purb = usb_alloc_urb(0, GFP_KERNEL);
		if( !purb )
		{
			TRACE("usbd_md_handle_bulktransfer: usb_alloc_urb failed\n");
			result = -ENOMEM;
			break;
		}

		pipe = 
			(punrb->BulkTransfer.Flags&UNRB_FLAG_DIRECTION_IN) ? 
			usb_rcvbulkpipe(pudev, punrb->BulkTransfer.Endpoint): 
			usb_sndbulkpipe(pudev, punrb->BulkTransfer.Endpoint);

		usb_fill_bulk_urb(
			purb,
			pudev,
			pipe,
			((char*)punrb)+sizeof(punrb->BulkTransfer),
			punrb->BulkTransfer.BufferSize,
			usbd_md_urb_complete_solid_bulktransfer,
			pqentry);

		if( punrb->BulkTransfer.Flags&UNRB_FLAG_DIRECTION_IN )
		{
			if( (punrb->BulkTransfer.Flags&UNRB_FLAG_SHORT_TRANSFER_OK) == 0 )
			{
				purb->transfer_flags |= URB_SHORT_NOT_OK;
			}
		} /*else
		{
			if( punrb->InterruptTransfer.Flags&UNRB_FLAG_SHORT_TRANSFER_OK )
			{
				purb->transfer_flags |= URB_ZERO_PACKET;
			}
		}*/

		pureq->purb = purb;
		pureq->pusbdevice = pdev;
		pureq->endpoint = punrb->BulkTransfer.Endpoint;

		usbd_usbdevice_reference(pdev);

		result = usb_submit_urb(purb, GFP_KERNEL);

		if( result == -EMSGSIZE )
		{
		    result = 1;
		    punrb->Header.Status = -EMSGSIZE;
		}

		if( result == -ENOMEM )
		{
		    result = 1;
		    punrb->Header.Status = -ENOMEM;
		}

		if( result != 0 )
		{
			usbd_usbdevice_dereference(pdev);
			TRACE("usbd_md_handle_bulktransfer: usb_submit_urb failed with error %d for UniqueId=0x%.8X%.8X\n", result, (uint32)(punrb->Header.UniqueId>>32), (uint32)(punrb->Header.UniqueId));
		}
		
	} while(0);

	if( result != 0 )
	{
		if( purb ) usb_free_urb(purb);
	}

	return result;
}

int usbd_md_handle_partitioned_bulktransfer(struct usbdevice_descriptor *pdev, struct usb_device *pudev, PUNRB punrb, struct waitable_queue_entry *pqentry)
{
	int result = 0;
	struct usbd_usb_request *pureq = (struct usbd_usb_request *)pqentry->data;

	do
	{
		int pipe;

		usbd_usbdevice_reference(pdev);

		pureq->pusbdevice = pdev;
		pureq->endpoint = punrb->BulkTransfer.Endpoint;

		pipe = 
			(punrb->BulkTransfer.Flags&UNRB_FLAG_DIRECTION_IN) ? 
			usb_rcvbulkpipe(pudev, punrb->BulkTransfer.Endpoint): 
			usb_sndbulkpipe(pudev, punrb->BulkTransfer.Endpoint);

		result = usbd_uc_init(
					&pureq->urbchain, 
					pipe, 
					0,
					!!(punrb->BulkTransfer.Flags&UNRB_FLAG_SHORT_TRANSFER_OK),
					pdev->is_ehci, 
					pudev, 
					&pureq->bufchain,
					pqentry,
					usbd_md_urb_complete_partitioned_bulktransfer);

		if( result < 0 )
		{
			TRACE("usbd_md_handle_partitioned_bulktransfer: usbd_md_alloc_urb_chain failed\n");
			break;
		}
	
		result = usbd_uc_submit(&pureq->urbchain);

		if( result == -EMSGSIZE )
		{
		    result = 1;
		    punrb->Header.Status = -EMSGSIZE;
		}

		if( result == -ENOMEM )
		{
		    result = 1;
		    punrb->Header.Status = -ENOMEM;
		}

		if( result != 0 )
		{
			TRACE("usbd_md_handle_partitioned_bulktransfer: usbd_uc_submit failed with error %d for UniqueId=0x%.8X%.8X\n",
				result, 
				(uint32)(punrb->Header.UniqueId>>32), 
				(uint32)(punrb->Header.UniqueId));
			break;
		}
	} while(0);

	if( result != 0 )
	{
		usbd_usbdevice_dereference(pdev);
		usbd_uc_clean(&pureq->urbchain);
	}

	return result;
}

int usbd_md_handle_interrupttransfer(struct usbdevice_descriptor *pdev, struct usb_device *pudev, PUNRB punrb, struct waitable_queue_entry *pqentry)
{
	int result = 0;
	struct urb *purb = NULL;
	struct usbd_usb_request* pureq = (struct usbd_usb_request*)pqentry->data;

	do
	{
		int pipe;
		int interval;
		struct usb_host_endpoint *ep;

		purb = usb_alloc_urb(0, GFP_KERNEL);
		if( !purb )
		{
			TRACE("usbd_md_handle_interrupttransfer: usb_alloc_urb failed\n");
			result = -ENOMEM;
			break;
		}

		pipe = 
			(punrb->InterruptTransfer.Flags&UNRB_FLAG_DIRECTION_IN) ? 
			usb_rcvintpipe(pudev, punrb->InterruptTransfer.Endpoint): 
			usb_sndintpipe(pudev, punrb->InterruptTransfer.Endpoint);

		if( punrb->InterruptTransfer.Interval )
		{
			interval = punrb->InterruptTransfer.Interval;

		} else
		{
			// ep_in and ep_out are supported from 2.6.10
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,10)
			ep = (punrb->InterruptTransfer.Flags&UNRB_FLAG_DIRECTION_IN) ? 
				pudev->ep_in[punrb->InterruptTransfer.Endpoint]: 
				pudev->ep_out[punrb->InterruptTransfer.Endpoint];

			TRACE("usbd_md_handle_interrupttransfer: direction=%s ep=0x%p ep_in[%d]=0x%p ep_out[%d]=0x%p\n",
				(punrb->InterruptTransfer.Flags&UNRB_FLAG_DIRECTION_IN) ? "in":"out",
				ep,
				punrb->InterruptTransfer.Endpoint,
				pudev->ep_in[punrb->InterruptTransfer.Endpoint],
				punrb->InterruptTransfer.Endpoint,
				pudev->ep_out[punrb->InterruptTransfer.Endpoint]);

			if( ep )
			{
				interval = ep->desc.bInterval;
			} else
			{
				interval = 1;
			}
#else
			interval = 1;
#endif
		}

		TRACE("usbd_md_handle_interrupttransfer: interval is %d\n", interval);

		usb_fill_int_urb(
			purb,
			pudev,
			pipe,
			((char*)punrb)+sizeof(punrb->InterruptTransfer),
			punrb->InterruptTransfer.BufferSize,
			usbd_md_urb_complete_interrupttransfer,
			pqentry,
			interval);

		if( punrb->InterruptTransfer.Flags&UNRB_FLAG_DIRECTION_IN )
		{
			if( (punrb->InterruptTransfer.Flags&UNRB_FLAG_SHORT_TRANSFER_OK) == 0 )
			{
				purb->transfer_flags |= URB_SHORT_NOT_OK;
			}
		}/* else
		{
			if( punrb->InterruptTransfer.Flags&UNRB_FLAG_SHORT_TRANSFER_OK )
			{
				purb->transfer_flags |= URB_ZERO_PACKET;
			}
		}*/

		pureq->purb = purb;
		pureq->pusbdevice = pdev;
		pureq->endpoint = punrb->InterruptTransfer.Endpoint;

		usbd_usbdevice_reference(pdev);

		while(1)
		{
		result = usb_submit_urb(purb, GFP_KERNEL);

			if( result == 0 )
				break;
			
		if( result == -EMSGSIZE )
		{
		    result = 1;
		    punrb->Header.Status = -EMSGSIZE;
				break;
			}
			
			if( result == -ENOMEM )
			{
				if( (pudev->speed != USB_SPEED_HIGH) || (purb->interval > 128) )
				{
					result = 1;
					punrb->Header.Status = -ENOMEM;
					break;
				}
				// for hi-speed devices, try to increase the interval
				purb->interval <<= 1;
			}
		}

		if( result != 0 )
		{
			usbd_usbdevice_dereference(pdev);
			TRACE("usbd_md_handle_interrupttransfer: usb_submit_urb failed with error %d for UniqueId=0x%.8X%.8X\n", result, (uint32)(punrb->Header.UniqueId>>32), (uint32)(punrb->Header.UniqueId));
		}
		
	} while(0);

	if( result != 0 )
	{
		if( purb ) usb_free_urb(purb);
	}

	return result;
}

int usbd_md_handle_isochtransfer(struct usbdevice_descriptor *pdev, struct usb_device *pudev, PUNRB punrb, struct waitable_queue_entry *pqentry)
{
	struct usbd_usb_request *pureq = (struct usbd_usb_request *)pqentry->data;

	if(pureq->buffer)
		return usbd_md_handle_solid_isochtransfer(pdev, pudev, punrb, pqentry);

	return usbd_md_handle_partitioned_isochtransfer(pdev, pudev, punrb, pqentry);
}

int usbd_md_handle_partitioned_isochtransfer(struct usbdevice_descriptor *pdev, struct usb_device *pudev, PUNRB punrb, struct waitable_queue_entry *pqentry)
{
	int pipe, result = 0;
	struct usbd_usb_request* pureq = (struct usbd_usb_request*)pqentry->data;

	do
	{
		int i,j,k,frame_delta, current_frame, iso_counter;
		int urboffset;
		struct urb *purb;

		usbd_usbdevice_reference(pdev);

		pureq->pusbdevice = pdev;
		pureq->endpoint = punrb->IsochTransfer.Endpoint;

		pipe = (punrb->IsochTransfer.Flags&UNRB_FLAG_DIRECTION_IN) ? 
			usb_rcvisocpipe(pudev, punrb->IsochTransfer.Endpoint): 
			usb_sndisocpipe(pudev, punrb->IsochTransfer.Endpoint);

		iso_counter = usbd_usbdevice_inc_iso_counter(pdev, pipe);	

		result = usbd_uc_init(
					&pureq->urbchain, 
					pipe, 
					punrb->IsochTransfer.Interval,
					!!(punrb->IsochTransfer.Flags&UNRB_FLAG_SHORT_TRANSFER_OK),
					pdev->is_ehci, 
					pudev, 
					&pureq->bufchain, 
					pqentry,
					usbd_md_urb_complete_partitioned_isochtransfer);

		if( result < 0 )
		{
			TRACE("usbd_md_handle_partitioned_isochtransfer: usbd_uc_init failed\n");
			break;
		}

		pureq->urbchain.urbs[0]->start_frame = punrb->IsochTransfer.StartFrame;

		if( !(punrb->IsochTransfer.Flags&UNRB_FLAG_ISO_TRANSFER_ASAP) )
			pureq->urbchain.urbs[0]->transfer_flags &= ~URB_ISO_ASAP;

		// i - index of iso packet in UNRB
		// j - index of the current urb in chain
		// k - index of iso packet in current urb
		purb = pureq->urbchain.urbs[0];
		urboffset = 0;
		
		for(i=0,j=0,k=0;i<punrb->IsochTransfer.NumberOfPackets;i++,k++)
		{
			if( k >= purb->number_of_packets )
			{
				if( ++j >= pureq->urbchain.count ) 
					break;

				urboffset = 0;
				purb = pureq->urbchain.urbs[j];
				k = 0;
			}

			purb->iso_frame_desc[k].offset = urboffset;
			purb->iso_frame_desc[k].length = punrb->IsochTransfer.IsochPackets[i].Length;
			urboffset += purb->iso_frame_desc[k].length;
			//TRACE("usbd_md_handle_partitioned_isochtransfer: urb %d frame %d offset %d srcframe %d srcoffset %d\n", j,k,purb->iso_frame_desc[k].offset,i, punrb->IsochTransfer.IsochPackets[i].Offset);
		}
	
		//
		// Setup start frame
		//

		frame_delta = usbd_usbdevice_get_frame_delta(pdev, pipe);
		current_frame = usb_get_current_frame_number(pudev);

		if( !(pureq->urbchain.urbs[0]->transfer_flags&URB_ISO_ASAP) && (frame_delta == 0) )
		{
			frame_delta = (current_frame + 2) - pureq->urbchain.urbs[0]->start_frame;
			usbd_usbdevice_set_frame_delta(pdev, pipe, frame_delta);
		}

		if( iso_counter == 1 )
		{
			if( pureq->urbchain.urbs[0]->transfer_flags&URB_ISO_ASAP )
			{
				pureq->urbchain.urbs[0]->transfer_flags &= ~URB_ISO_ASAP;
				pureq->urbchain.urbs[0]->start_frame = current_frame + 1 - frame_delta;
			}
		}

		if( !(pureq->urbchain.urbs[0]->transfer_flags&URB_ISO_ASAP) )
		{
			pureq->urbchain.urbs[0]->start_frame += frame_delta;

			if( pureq->urbchain.urbs[0]->start_frame < current_frame+1 )
			{
				punrb->Header.Status = -EXDEV;
				punrb->Header.Size = GET_UNRB_ISOCH_TRANSFER_PACKETSIZE(punrb);
				punrb->IsochTransfer.BufferSize = 0;
				punrb->IsochTransfer.ErrorCount = punrb->IsochTransfer.NumberOfPackets;
			
				for(i=0;i<punrb->IsochTransfer.NumberOfPackets;i++)
				{
					punrb->IsochTransfer.IsochPackets[i].Status = -EINVAL;
					punrb->IsochTransfer.IsochPackets[i].Length = 0;
				}

			    result = 1;
				break;
			}
		}

		result = usbd_uc_submit(&pureq->urbchain);

		if( result == -EMSGSIZE )
		{
		    result = 1;
		    punrb->Header.Status = -EMSGSIZE;
		}

		if( result == -ENOMEM )
		{
		    result = 1;
		    punrb->Header.Status = -ENOMEM;
		}                                 

		if( result != 0 )
		{
			TRACE("usbd_md_handle_partitioned_isochtransfer: usbd_uc_submit failed with error %d for UniqueId=0x%.8X%.8X\n",
				result, 
				(uint32)(punrb->Header.UniqueId>>32), 
				(uint32)(punrb->Header.UniqueId));
			break;
		}
		
	} while(0);

	if( result != 0 )
	{
		usbd_usbdevice_dec_iso_counter(pdev, pipe);
		usbd_usbdevice_dereference(pdev);
		usbd_uc_clean(&pureq->urbchain);
	}

	return result;
}

int usbd_md_handle_solid_isochtransfer(struct usbdevice_descriptor *pdev, struct usb_device *pudev, PUNRB punrb, struct waitable_queue_entry *pqentry)
{
	int pipe, result = 0;
	struct urb *purb = NULL;
	struct usbd_usb_request* pureq = (struct usbd_usb_request*)pqentry->data;
	int urboffset;

	do
	{
		int i, frame_delta, current_frame, iso_counter;
		struct usb_host_endpoint *ep;

		usbd_usbdevice_reference(pdev);

		pipe = 
			(punrb->IsochTransfer.Flags&UNRB_FLAG_DIRECTION_IN) ? 
			usb_rcvisocpipe(pudev, punrb->IsochTransfer.Endpoint): 
			usb_sndisocpipe(pudev, punrb->IsochTransfer.Endpoint);

		iso_counter = usbd_usbdevice_inc_iso_counter(pdev, pipe);

		purb = usb_alloc_urb(punrb->IsochTransfer.NumberOfPackets, GFP_KERNEL);
		if( !purb )
		{
			TRACE("usbd_md_handle_solid_isochtransfer: usb_alloc_urb failed\n");
			result = -ENOMEM;
			break;
		}

		purb->dev		= pudev;
		purb->pipe		= pipe;
		purb->transfer_flags	= (punrb->IsochTransfer.Flags&UNRB_FLAG_ISO_TRANSFER_ASAP) ? URB_ISO_ASAP : 0;
		purb->transfer_buffer_length = punrb->IsochTransfer.BufferSize;
		purb->start_frame	= punrb->IsochTransfer.StartFrame;
		purb->number_of_packets	= punrb->IsochTransfer.NumberOfPackets;
		purb->context		= pqentry;
		purb->complete		= usbd_md_urb_complete_solid_isochtransfer;
		purb->transfer_buffer	= ((char*)punrb) + GET_UNRB_ISOCH_TRANSFER_PACKETSIZE(punrb);

		if( punrb->IsochTransfer.Interval )
		{
			purb->interval = punrb->IsochTransfer.Interval;

		} else
		{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,10)
			// ep_in and ep_out are supported from 2.6.10
			ep = (punrb->IsochTransfer.Flags&UNRB_FLAG_DIRECTION_IN) ? 
				pudev->ep_in[punrb->IsochTransfer.Endpoint]: 
				pudev->ep_out[punrb->IsochTransfer.Endpoint];
   	
			if( ep )
			{
				purb->interval = 1 << (ep->desc.bInterval - 1);
			} else
			{
				purb->interval = 1;
			}
#else
			purb->interval = 1;
#endif
		}

		TRACE("usbd_md_handle_solid_isochtransfer: interval is %d\n", purb->interval);

		for(urboffset=0,i=0;i<punrb->IsochTransfer.NumberOfPackets;i++)
		{
			purb->iso_frame_desc[i].offset = urboffset;
			purb->iso_frame_desc[i].length = punrb->IsochTransfer.IsochPackets[i].Length;
			urboffset += purb->iso_frame_desc[i].length;
		}

		//
		// Setup start frame
		//
		frame_delta = usbd_usbdevice_get_frame_delta(pdev, pipe);
		current_frame = usb_get_current_frame_number(pudev);

		if( !(purb->transfer_flags&URB_ISO_ASAP) && (frame_delta == 0) )
		{
			frame_delta = (current_frame + 2) - purb->start_frame;
			usbd_usbdevice_set_frame_delta(pdev, pipe, frame_delta);
		}

		if( iso_counter == 1 )
		{
			if( purb->transfer_flags&URB_ISO_ASAP )
			{
				purb->transfer_flags &= ~URB_ISO_ASAP;
				purb->start_frame = current_frame + 1 - frame_delta;
			}
		}

		if( !(purb->transfer_flags&URB_ISO_ASAP) )
		{
			purb->start_frame += frame_delta;

			if( purb->start_frame < current_frame+1 )
			{
				punrb->Header.Status = -EXDEV;
				punrb->Header.Size = GET_UNRB_ISOCH_TRANSFER_PACKETSIZE(punrb);
				punrb->IsochTransfer.BufferSize = 0;
				punrb->IsochTransfer.ErrorCount = punrb->IsochTransfer.NumberOfPackets;
			
				for(i=0;i<punrb->IsochTransfer.NumberOfPackets;i++)
				{
					punrb->IsochTransfer.IsochPackets[i].Status = -EINVAL;
					punrb->IsochTransfer.IsochPackets[i].Length = 0;
				}

			    result = 1;
				break;
			}
		}     

		pureq->purb = purb;
		pureq->pusbdevice = pdev;
		pureq->endpoint = punrb->IsochTransfer.Endpoint;

		TRACE("usbd_md_handle_solid_isochtransfer: flags=%d startframe=%d curframe=%d delta=%d\n", 
			purb->transfer_flags,
			purb->start_frame,
			usb_get_current_frame_number(pudev),
			usbd_usbdevice_get_frame_delta(pdev, pipe)			
		);

		result = usb_submit_urb(purb, GFP_KERNEL);

		if( result == -EMSGSIZE )
		{
		    result = 1;
		    punrb->Header.Status = -EMSGSIZE;
		}

		if( result == -ENOMEM )
		{
		    result = 1;
		    punrb->Header.Status = -ENOMEM;
		}

		if( result != 0 )
		{
			TRACE("usbd_md_handle_solid_isochtransfer: usb_submit_urb failed with error %d for UniqueId=0x%.8X%.8X\n", result, (uint32)(punrb->Header.UniqueId>>32), (uint32)(punrb->Header.UniqueId));
		}
		
	} while(0);

	if( result != 0 )
	{
		if( purb ) usb_free_urb(purb);
		usbd_usbdevice_dec_iso_counter(pdev, pipe);
		usbd_usbdevice_dereference(pdev);
	}

	return result;
}

void usbd_md_urb_complete_getdescriptor(
	struct urb* purb
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
	,struct pt_regs* ptregs
#endif
)
{
	struct waitable_queue_entry *pqentry = purb->context;
	struct usbd_usb_request *pureq = (struct usbd_usb_request *)pqentry->data;
	struct usbdevice_descriptor *pdev = pureq->pusbdevice;
	PUNRB punrb = (PUNRB)(pqentry->data+sizeof(struct usbd_usb_request));
	
	TRACE("usbd_md_urb_complete_getdescriptor\n");

	// remove this unrb from pending queue
	usbd_wq_remove_entry_locked(&pdev->queue_pending_unrb, pqentry);

	if( purb->actual_length < 0 )
	{
	    TRACE("actual_length=%d", purb->actual_length);
	    punrb->DescriptorRequest.BufferSize = 0;
	} else
	{
		punrb->DescriptorRequest.BufferSize = purb->actual_length;
   	}

	punrb->DescriptorRequest.Header.Size = sizeof(punrb->DescriptorRequest) + punrb->DescriptorRequest.BufferSize;
	punrb->DescriptorRequest.Header.Status = purb->status;

	TRACE("usbd_md_urb_complete_getdescriptor: urb completed with status %d\n", purb->status);
	dump_unrb(punrb);

	usbd_wq_add_tail_locked(&pdev->queue_completed_unrb, pqentry);

	if( purb->setup_packet ) kfree(purb->setup_packet);
	usb_free_urb(purb);

	usbd_usbdevice_dereference(pdev);
}

void usbd_md_urb_complete_controltransfer(
	struct urb* purb
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
	,struct pt_regs* ptregs
#endif
)
{
	struct waitable_queue_entry *pqentry = purb->context;
	struct usbd_usb_request *pureq = (struct usbd_usb_request *)pqentry->data;
	struct usbdevice_descriptor *pdev = pureq->pusbdevice;
	PUNRB punrb = (PUNRB)(pqentry->data+sizeof(struct usbd_usb_request));
	
	TRACE("usbd_md_urb_complete_controltransfer\n");

	// remove this unrb from pending queue
	usbd_wq_remove_entry_locked(&pdev->queue_pending_unrb, pqentry);

	if( purb->actual_length < 0 )
	{
	    TRACE("actual_length=%d", purb->actual_length);
	    punrb->ControlTransfer.BufferSize = 0;
	} else
	{
            punrb->ControlTransfer.BufferSize = purb->actual_length;
   	}

	punrb->Header.Size = sizeof(punrb->ControlTransfer);
	if(usb_pipein(purb->pipe))
		 punrb->Header.Size += punrb->ControlTransfer.BufferSize;

	punrb->ControlTransfer.Header.Status = purb->status;

	TRACE("usbd_md_urb_complete_controltransfer: urb completed with status %d\n", purb->status);
	dump_unrb(punrb);

	usbd_wq_add_tail_locked(&pdev->queue_completed_unrb, pqentry);

	if( purb->setup_packet ) kfree(purb->setup_packet);
	usb_free_urb(purb);

	usbd_usbdevice_dereference(pdev);
}

void usbd_md_urb_complete_solid_bulktransfer(
	struct urb* purb
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
	,struct pt_regs* ptregs
#endif
)
{
	struct waitable_queue_entry *pqentry = purb->context;
	struct usbd_usb_request *pureq = (struct usbd_usb_request *)pqentry->data;
	struct usbdevice_descriptor *pdev = pureq->pusbdevice;
	PUNRB punrb = (PUNRB)(pqentry->data+sizeof(struct usbd_usb_request));


	TRACE("usbd_md_urb_complete_bulktransfer\n");

	// remove this unrb from pending queue
	usbd_wq_remove_entry_locked(&pdev->queue_pending_unrb, pqentry);

	punrb->BulkTransfer.BufferSize = purb->actual_length;

	punrb->Header.Size = sizeof(punrb->BulkTransfer);
	if(usb_pipein(purb->pipe))
		 punrb->Header.Size += punrb->BulkTransfer.BufferSize;

	punrb->BulkTransfer.Header.Status = purb->status;

	TRACE("usbd_md_urb_complete_bulktransfer: urb completed with status %d\n", purb->status);
	dump_unrb(punrb);

	usbd_wq_add_tail_locked(&pdev->queue_completed_unrb, pqentry);

	usb_free_urb(purb);

	usbd_usbdevice_dereference(pdev);
}

void usbd_md_urb_complete_interrupttransfer(
	struct urb* purb
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
	,struct pt_regs* ptregs
#endif
)
{
	struct waitable_queue_entry *pqentry = purb->context;
	struct usbd_usb_request *pureq = (struct usbd_usb_request *)pqentry->data;
	struct usbdevice_descriptor *pdev = pureq->pusbdevice;
	PUNRB punrb = (PUNRB)(pqentry->data+sizeof(struct usbd_usb_request));

	TRACE("usbd_md_urb_complete_interrupttransfer\n");

	// remove this unrb from pending queue
	usbd_wq_remove_entry_locked(&pdev->queue_pending_unrb, pqentry);

	punrb->InterruptTransfer.BufferSize = purb->actual_length;

	punrb->Header.Size = sizeof(punrb->InterruptTransfer);
	if(usb_pipein(purb->pipe))
		 punrb->Header.Size += punrb->InterruptTransfer.BufferSize;

	punrb->InterruptTransfer.Header.Status = purb->status;

	TRACE("usbd_md_urb_complete_interrupttransfer: urb completed with status %d\n", purb->status);
	dump_unrb(punrb);

	usbd_wq_add_tail_locked(&pdev->queue_completed_unrb, pqentry);

	usbd_usbdevice_dereference(pdev);

	usb_free_urb(purb);
}

void usbd_md_urb_complete_solid_isochtransfer(
	struct urb* purb
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
	,struct pt_regs* ptregs
#endif
)
{
	int i;
	struct waitable_queue_entry *pqentry = purb->context;
	struct usbd_usb_request *pureq = (struct usbd_usb_request *)pqentry->data;
	struct usbdevice_descriptor *pdev = pureq->pusbdevice;
	PUNRB punrb = (PUNRB)(pqentry->data+sizeof(struct usbd_usb_request));

	usbd_usbdevice_dec_iso_counter(pdev, purb->pipe);
	purb->start_frame -= usbd_usbdevice_get_frame_delta(pdev, purb->pipe);

	// remove this unrb from pending queue
	usbd_wq_remove_entry_locked(&pdev->queue_pending_unrb, pqentry);

	punrb->IsochTransfer.Header.Status = purb->status;
	punrb->IsochTransfer.ErrorCount = purb->error_count;
	punrb->IsochTransfer.StartFrame = purb->start_frame;
	punrb->IsochTransfer.BufferSize = 
		usbd_pack_iso_buffer(
			purb->iso_frame_desc, purb->number_of_packets, 
			purb->transfer_buffer, purb->transfer_buffer, 1);

	for(i=0;i<punrb->IsochTransfer.NumberOfPackets;i++)
	{
		punrb->IsochTransfer.IsochPackets[i].Length = purb->iso_frame_desc[i].actual_length;
		punrb->IsochTransfer.IsochPackets[i].Status = purb->iso_frame_desc[i].status;
	}

	punrb->Header.Size = GET_UNRB_ISOCH_TRANSFER_PACKETSIZE(punrb);
	if(usb_pipein(purb->pipe))
		 punrb->Header.Size += punrb->IsochTransfer.BufferSize;

	TRACE("usbd_md_urb_complete_solid_isochtransfer: urb completed with status %d\n", purb->status);
	dump_unrb(punrb);

	usbd_wq_add_tail_locked(&pdev->queue_completed_unrb, pqentry);

	usbd_usbdevice_dereference(pdev);


#if defined(_USBD_USE_EHCI_FIX_) && LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,25) && LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,28)
	if(pdev->is_ehci && (purb->status == 0) && (atomic_read(&purb->kref.refcount) > 1))
		usb_put_urb(purb);
#endif

	usb_free_urb(purb);
}

void usbd_md_urb_complete_partitioned_bulktransfer(struct urb_chain* pchain)
{
	int i;
	struct waitable_queue_entry *pqentry = pchain->context;
	struct usbd_usb_request *pureq = (struct usbd_usb_request *)pqentry->data;
	struct usbdevice_descriptor *pdev = pureq->pusbdevice;
	PUNRB punrb = (PUNRB)(pqentry->data+sizeof(struct usbd_usb_request));

	TRACE("usbd_md_urb_complete_partitioned_bulktransfer\n");

	// remove this unrb from pending queue
	usbd_wq_remove_entry_locked(&pdev->queue_pending_unrb, pqentry);

	punrb->BulkTransfer.BufferSize = pchain->bytes;

	punrb->Header.Size = sizeof(punrb->BulkTransfer);
	if(usb_pipein(pchain->pipe))
		 punrb->BulkTransfer.Header.Size += punrb->BulkTransfer.BufferSize;

	punrb->BulkTransfer.Header.Status = pchain->status;

	for(i=0;i<pchain->count;i++)
	{
		pureq->bufchain.datasize[i] = pchain->urbs[i]->actual_length;
	}

	TRACE("usbd_md_urb_complete_partitioned_bulktransfer: urb completed with status %d\n", pchain->status);
	dump_unrb(punrb);

	usbd_uc_clean(pchain);

	usbd_wq_add_tail_locked(&pdev->queue_completed_unrb, pqentry);

	usbd_usbdevice_dereference(pdev);
}

void usbd_md_urb_complete_partitioned_isochtransfer(struct urb_chain* pchain)
{
	int i,j,k;
	struct waitable_queue_entry *pqentry = pchain->context;
	struct usbd_usb_request *pureq = (struct usbd_usb_request *)pqentry->data;
	struct usbdevice_descriptor *pdev = pureq->pusbdevice;
	PUNRB punrb = (PUNRB)(pqentry->data+sizeof(struct usbd_usb_request));

	TRACE("usbd_md_urb_complete_partitioned_isochtransfer\n");

	usbd_usbdevice_dec_iso_counter(pdev, pchain->pipe);
	pchain->urbs[0]->start_frame -= usbd_usbdevice_get_frame_delta(pdev, pchain->pipe);

	// remove this unrb from pending queue
	usbd_wq_remove_entry_locked(&pdev->queue_pending_unrb, pqentry);

	punrb->IsochTransfer.Header.Status = pchain->status;
	punrb->IsochTransfer.StartFrame = pchain->urbs[0]->start_frame;
	punrb->IsochTransfer.ErrorCount = 0;
	punrb->IsochTransfer.BufferSize = 0;

	for(i=0,k=0;i<pchain->count;i++)
	{
		struct urb *purb = pchain->urbs[i];

		pureq->bufchain.datasize[i] = 
			usbd_pack_iso_buffer(
				purb->iso_frame_desc, purb->number_of_packets, 
				purb->transfer_buffer, purb->transfer_buffer, 1);

		punrb->IsochTransfer.BufferSize += pureq->bufchain.datasize[i];
		punrb->IsochTransfer.ErrorCount += purb->error_count;

		for(j=0; j<purb->number_of_packets; j++)
		{
			// TRACE("usbd_md_urb_complete_partitioned_isochtransfer: urb %d frame %d offset %d len=%d srcframe %d\n", i,j,purb->iso_frame_desc[j].offset,purb->iso_frame_desc[j].actual_length,k);
			punrb->IsochTransfer.IsochPackets[k].Length = purb->iso_frame_desc[j].actual_length;
			punrb->IsochTransfer.IsochPackets[k].Status = purb->iso_frame_desc[j].status;
			k++;
		}
	}

	punrb->Header.Size = GET_UNRB_ISOCH_TRANSFER_PACKETSIZE(punrb);
	if(usb_pipein(pchain->pipe))
		 punrb->Header.Size += punrb->IsochTransfer.BufferSize;

	TRACE("usbd_md_urb_complete_partitioned_isochtransfer: urb completed with status %d\n", pchain->status);
	dump_unrb(punrb);

	usbd_uc_clean(pchain);

	usbd_wq_add_tail_locked(&pdev->queue_completed_unrb, pqentry);
           
	usbd_usbdevice_dereference(pdev);
}

#endif // #ifdef _USBD_ENABLE_STUB_
