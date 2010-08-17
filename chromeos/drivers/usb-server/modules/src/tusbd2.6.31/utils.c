/*
 *  tusbd/utils.c
 *
 *  Copyright (C) 2007-2008 IncentivesPro
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include "usbd.h"
#include <linux/mm.h>

int ConvertUnrbStatusToLinux(uint32 status);

#ifdef _USBD_ENABLE_STUB_

// returns the number of bytes copied
size_t usbd_copy_n_free_unrb(void __user *buf, size_t count, struct waitable_queue_entry *pqentry)
{
	struct usbd_usb_request *pureq = (struct usbd_usb_request *)pqentry->data;
	PUNRB punrb = (PUNRB)(pqentry->data + sizeof(struct usbd_usb_request));

	size_t result = 0;

	TRACE("usbd_copy_n_free_unrb: ++\n");

	do
	{
		uint8 flags;
		size_t unrbsize, buffersize, n;

		switch (punrb->Header.Function)
		{
			case UNRB_FUNCTION_BULK_TRANSFER:
				unrbsize 	= sizeof(punrb->BulkTransfer);
				buffersize	= punrb->BulkTransfer.BufferSize;
				flags		= punrb->BulkTransfer.Flags;
				break;
			case UNRB_FUNCTION_ISOCH_TRANSFER:
				unrbsize	= GET_UNRB_ISOCH_TRANSFER_PACKETSIZE(punrb);
				flags 		= punrb->IsochTransfer.Flags;
				buffersize	= punrb->IsochTransfer.BufferSize;
				break;
			case UNRB_FUNCTION_INTERRUPT_TRANSFER:
				unrbsize	= sizeof(punrb->InterruptTransfer);
				flags		= punrb->InterruptTransfer.Flags;
				buffersize	= punrb->InterruptTransfer.BufferSize;
				break;
			case UNRB_FUNCTION_GET_DESCRIPTOR:
				unrbsize	= sizeof(punrb->DescriptorRequest);
				flags		= UNRB_FLAG_DIRECTION_IN;
				buffersize	= punrb->DescriptorRequest.BufferSize;
				break;
			case UNRB_FUNCTION_SET_DESCRIPTOR:
				unrbsize	= sizeof(punrb->DescriptorRequest);
				flags		= 0; 
				buffersize	= punrb->DescriptorRequest.BufferSize;
				break;
			case UNRB_FUNCTION_CONTROL_TRANSFER:
				unrbsize	= sizeof(punrb->ControlTransfer);
				flags		= punrb->ControlTransfer.Flags;
				buffersize	= punrb->ControlTransfer.BufferSize;
				break;
			default:
				unrbsize	= punrb->Header.Size;
				buffersize	= 0;
				flags		= 0;
				break;
		}

		if(pureq->buffer)
		{
			// copy solid UNRB
			n = (flags&UNRB_FLAG_DIRECTION_IN) ? unrbsize+buffersize : unrbsize;
			n = MIN_OF(n, count);

			if( copy_to_user(buf, punrb, n) != 0 )
			{
				TRACE("usbd_copy_n_free_unrb: cannot copy to user buffer\n");
				result = 0;
				break;
			}

			result += n;
		} else
		{
			// copy partitioned UNRB

			if( !access_ok(VERIFY_WRITE, buf, punrb->Header.Size))
			{
				TRACE("usbd_copy_n_free_unrb: memory access check failed\n");
				break;
			}

			n = MIN_OF(unrbsize, count);

			if( __copy_to_user(buf, punrb, n) != 0 )
			{
				TRACE("usbd_copy_n_free_unrb: cannot copy to user buffer(2)\n");
				break;
			}

			result += n;
			count -= n;
			buf += n;

			if( flags&UNRB_FLAG_DIRECTION_IN )
			{
				n = MIN_OF(buffersize,count);

				if( usbd_bc_copy_to_user(&pureq->bufchain, buf, n) < n )
				{
					TRACE("usbd_copy_n_free_unrb: cannot copy to user buffer(3)\n");
					result = 0;
					break;
				}

				result += n;
			}
		}

	}while(0);

	usbd_bc_free(&pureq->bufchain);
	usbd_wq_free_entry(pqentry);
	TRACE("usbd_copy_n_free_unrb: -- result=%d\n", result);
	return result;
}


struct waitable_queue_entry *usbd_alloc_n_copy_unrb(void __user *buf, size_t count, struct usbdevice_descriptor* pusbdevice, struct usb_device *pudev)
{
	PUNRB pdstunrb;
	UNRB srcunrb;
	struct waitable_queue_entry *pqentry = NULL;
	struct usbd_usb_request *pureq;
	uint32 unrbsize, buffersize;
	uint8 endpoint;
	uint8 flags;
	int result = -1;


	do /* while(0) */
	{
		if( copy_from_user(&srcunrb, buf, sizeof(srcunrb)) != 0 )
		{
			TRACE("usbd_alloc_n_copy_partiotioned_unrb: cannot copy from user buffer (2)\n");
			break;
		}

		if( !access_ok(VERIFY_READ, buf, srcunrb.Header.Size))
		{
			TRACE("usbd_alloc_n_copy_partiotioned_unrb: memory access check failed\n");
			result = -EFAULT;
			break;
		}

		switch(srcunrb.Header.Function)
		{
			case UNRB_FUNCTION_BULK_TRANSFER:
				endpoint	= srcunrb.BulkTransfer.Endpoint;
				flags		= srcunrb.BulkTransfer.Flags;
				unrbsize	= sizeof(srcunrb.BulkTransfer);
				buffersize	= srcunrb.BulkTransfer.BufferSize;
				break;
			case UNRB_FUNCTION_ISOCH_TRANSFER:
				endpoint	= srcunrb.IsochTransfer.Endpoint;
				flags		= srcunrb.IsochTransfer.Flags;
				unrbsize	= GET_UNRB_ISOCH_TRANSFER_PACKETSIZE(&srcunrb);
				buffersize	= srcunrb.IsochTransfer.BufferSize;
				break;
			case UNRB_FUNCTION_INTERRUPT_TRANSFER:
				endpoint	= srcunrb.InterruptTransfer.Endpoint;
				flags		= srcunrb.InterruptTransfer.Flags;
				unrbsize	= sizeof(srcunrb.InterruptTransfer);
				buffersize	= srcunrb.InterruptTransfer.BufferSize;
				break;
			case UNRB_FUNCTION_GET_DESCRIPTOR:
				endpoint	= 0;
				flags		= UNRB_FLAG_DIRECTION_IN;
				unrbsize	= sizeof(srcunrb.DescriptorRequest);
				buffersize	= srcunrb.DescriptorRequest.BufferSize;
				break;
			case UNRB_FUNCTION_SET_DESCRIPTOR:
				endpoint	= 0;
				flags		= 0; 
				unrbsize	= sizeof(srcunrb.DescriptorRequest);
				buffersize	= srcunrb.DescriptorRequest.BufferSize;
				break;
			case UNRB_FUNCTION_CONTROL_TRANSFER:
				endpoint	= srcunrb.ControlTransfer.Endpoint;
				flags		= srcunrb.ControlTransfer.Flags;
				unrbsize	= sizeof(srcunrb.ControlTransfer);
				buffersize	= srcunrb.ControlTransfer.BufferSize;
				break;
			default:
				endpoint	= 0;
				flags		= 0;
				unrbsize	= srcunrb.Header.Size;
				buffersize	= 0;
				break;
		}

		// Check if we fit into maximum buffer size
		if( buffersize && (usbd_uc_maxbuffersize >= (sizeof(struct usbd_usb_request) + unrbsize + buffersize)) )
		{
			// allocate queue entry to hold the usbd_usb_request structure, UNRB body and buffer(for solid UNRBs)
			pqentry = 
				usbd_wq_alloc_entry(
					sizeof(struct usbd_usb_request) + unrbsize + buffersize, 
					GFP_KERNEL);

			// If this is an OUT transfer - copy te buffer. For IN transfers we just drop it.
			if( !(flags&UNRB_FLAG_DIRECTION_IN) )
				unrbsize += buffersize;

			buffersize = 0;
		} else
		{
			// allocate queue entry to hold the usbd_usb_request structure and UNRB body only
			// partitioned buffer will be allocated later
			pqentry = 
				usbd_wq_alloc_entry(
					sizeof(struct usbd_usb_request) + unrbsize, 
					GFP_KERNEL);
		}

		if( !pqentry )
		{
			TRACE("usbd_alloc_n_copy_partiotioned_unrb: cannot allocate UNRB unrbsize=%d + %d\n", unrbsize, sizeof(struct usbd_usb_request));
			break;
		}

		pureq = (struct usbd_usb_request *)pqentry->data;
		pdstunrb = (PUNRB)(pqentry->data + sizeof(struct usbd_usb_request));

		memset(pureq, 0, sizeof(*pureq));

		// use __copy_from_user because access check has already been performed earlier
		if( __copy_from_user(pdstunrb, buf, unrbsize) != 0 )
		{
			TRACE("usbd_alloc_n_copy_partiotioned_unrb: cannot copy from user buffer (3)\n");
			break;
		}

		if(buffersize == 0)
		{
			TRACE("usbd_alloc_n_copy_partiotioned_unrb: solid unrb\n");
			// solid UNRB - just set the buffer pointer to non-null value
			pureq->buffer = (void*)-1;
		}
		else
		{
			// partitioned UNRB - allocate buffer chains
			if( pdstunrb->Header.Function == UNRB_FUNCTION_ISOCH_TRANSFER )
			{
				result = usbd_bc_alloc_iso(
							&pureq->bufchain, 
							buffersize,
							pdstunrb->IsochTransfer.IsochPackets,
							pdstunrb->IsochTransfer.NumberOfPackets);

				if( result < 0 ) 
				{
					TRACE("usbd_alloc_n_copy_partiotioned_unrb: usbd_bc_alloc_iso failed\n");
					break;
				}

			} else
			{
				uint16 maxpacketsize = (flags&UNRB_FLAG_DIRECTION_IN) ? 
					pudev->ep_in[endpoint]->desc.wMaxPacketSize:
					pudev->ep_out[endpoint]->desc.wMaxPacketSize;

				// convert to cpu endianess and extract bits 0..10
				maxpacketsize = le16_to_cpu(maxpacketsize);
				maxpacketsize = (maxpacketsize & 0x7FF) * (1 + ((maxpacketsize >> 11) & 0x03));

				TRACE("usbd_alloc_n_copy_partiotioned_unrb: maxpacketsize=%d\n", maxpacketsize);

				result = usbd_bc_alloc(
					&pureq->bufchain,
					buffersize,
					maxpacketsize);

				if( result < 0 ) 
				{
					TRACE("usbd_alloc_n_copy_partiotioned_unrb: usbd_bc_alloc failed\n");
					break;
				}
			}

			if( !(flags&UNRB_FLAG_DIRECTION_IN) )
			{
				buf += unrbsize; // trim the buffer
				count -= unrbsize;

				// copy the buffer for OUT transfers
				if( usbd_bc_copy_from_user(
						&pureq->bufchain, 
						buf, 
						MIN_OF(buffersize,count)) < buffersize )
				{
					result = -1;
					break;
				}
			}
		}

		result = 0;

	} while(0);

	// cleanup on error
	if( result < 0 )
	{
		if( pqentry )
		{
			usbd_bc_free(&pureq->bufchain);
			usbd_wq_free_entry(pqentry);
		}
		return NULL;
	}

	return pqentry;
}

#endif // #ifdef _USBD_ENABLE_STUB_

uint64 usbd_get_unique_id(void)
{
	static uint64 UniqueId = 0;

	return (++UniqueId);
}

#ifdef _USBD_ENABLE_VHCI_
int usbd_pack_urb_bulk(struct urb *purb, uint64 unique_id, void *buf)
{
	int result = 0;
	int unrb_size = sizeof(UNRB_BULK_TRANSFER);
	PUNRB punrb = buf;

	punrb->Header.UniqueId = unique_id;
	punrb->Header.Size = unrb_size;
	punrb->Header.Function = UNRB_FUNCTION_BULK_TRANSFER;
	punrb->Header.Status = 0;
	punrb->Header.Context = 0;

	punrb->BulkTransfer.BufferSize	= purb->transfer_buffer_length;
	punrb->BulkTransfer.Endpoint	= usb_pipeendpoint(purb->pipe);
	punrb->BulkTransfer.Flags		= usb_pipein(purb->pipe) ? UNRB_FLAG_DIRECTION_IN : 0; 
	if(usb_pipein(purb->pipe) && !(purb->transfer_flags&URB_SHORT_NOT_OK))
	{
		punrb->BulkTransfer.Flags |= UNRB_FLAG_SHORT_TRANSFER_OK; 
	}

	if( purb->transfer_buffer && usb_pipeout(purb->pipe) )
	{
		memcpy((char*)buf+unrb_size, purb->transfer_buffer, purb->transfer_buffer_length);
		punrb->Header.Size += purb->transfer_buffer_length;
	}

	return result;
}

int usbd_pack_urb_interrupt(struct urb *purb, uint64 unique_id, void *buf)
{
	int result = 0;
	int unrb_size = sizeof(UNRB_INTERRUPT_TRANSFER);
	PUNRB punrb = buf;

	punrb->Header.UniqueId = unique_id;
	punrb->Header.Size = unrb_size;
	punrb->Header.Function = UNRB_FUNCTION_INTERRUPT_TRANSFER;
	punrb->Header.Status = 0;
	punrb->Header.Context = 0;

	punrb->InterruptTransfer.BufferSize	= purb->transfer_buffer_length;
	punrb->InterruptTransfer.Interval	= purb->interval;
	punrb->InterruptTransfer.Endpoint	= usb_pipeendpoint(purb->pipe);
	punrb->InterruptTransfer.Flags		= usb_pipein(purb->pipe) ? UNRB_FLAG_DIRECTION_IN : 0; 
	punrb->InterruptTransfer.Flags		|= (purb->transfer_flags&URB_SHORT_NOT_OK) ?  0 : UNRB_FLAG_SHORT_TRANSFER_OK; 

	if( purb->transfer_buffer && usb_pipeout(purb->pipe) )
	{
		memcpy((char*)buf+unrb_size, purb->transfer_buffer, purb->transfer_buffer_length);
		punrb->Header.Size += purb->transfer_buffer_length;
	}

	return result;
}

int usbd_pack_urb_isoch(struct urb *purb, uint64 unique_id, void *buf)
{
	int result = 0;
	PUNRB punrb = buf;
	int i;
	int unrb_size = sizeof(UNRB_ISOCH_TRANSFER) - sizeof(UNRB_ISOCH_PACKET_DESCRIPTOR) + 
					sizeof(UNRB_ISOCH_PACKET_DESCRIPTOR) * purb->number_of_packets;

	punrb->Header.UniqueId = unique_id;
	punrb->Header.Size = unrb_size;
	punrb->Header.Function = UNRB_FUNCTION_ISOCH_TRANSFER;
	punrb->Header.Status = 0;
	punrb->Header.Context = 0;
				
	punrb->IsochTransfer.Endpoint	= usb_pipeendpoint(purb->pipe);
	punrb->IsochTransfer.Flags		= usb_pipein(purb->pipe) ? UNRB_FLAG_DIRECTION_IN : 0; 
	punrb->IsochTransfer.Flags		|= (purb->transfer_flags&URB_SHORT_NOT_OK) ?  0 : UNRB_FLAG_SHORT_TRANSFER_OK; 
	punrb->IsochTransfer.Flags		|= (purb->transfer_flags&URB_ISO_ASAP) ?  UNRB_FLAG_ISO_TRANSFER_ASAP : 0; 

	punrb->IsochTransfer.BufferSize	= purb->transfer_buffer_length;
	punrb->IsochTransfer.Interval	= purb->interval;
	punrb->IsochTransfer.StartFrame	= purb->start_frame;
	punrb->IsochTransfer.ErrorCount = 0;
	punrb->IsochTransfer.NumberOfPackets = purb->number_of_packets;

	for(i=0;i<purb->number_of_packets;i++)
	{
		punrb->IsochTransfer.IsochPackets[i].Offset = purb->iso_frame_desc[i].offset;
		punrb->IsochTransfer.IsochPackets[i].Length = purb->iso_frame_desc[i].length;
		punrb->IsochTransfer.IsochPackets[i].Status = 0;
	}

	if( purb->transfer_buffer && usb_pipeout(purb->pipe) )
	{
		punrb->Header.Size += 
			usbd_pack_iso_buffer(
				purb->iso_frame_desc, purb->number_of_packets, 
				(char*)buf+unrb_size,  purb->transfer_buffer, 0);
	}
	return result;
}

int usbd_pack_urb_control(struct urb *purb, uint64 unique_id, struct usb_ctrlrequest *pctrlreq, void *buf)
{
	int result = 0;
	PUNRB punrb = buf;
	int unrb_size = sizeof(UNRB_CONTROL_TRANSFER);

	punrb->Header.UniqueId = unique_id;
	punrb->Header.Size= unrb_size;
	punrb->Header.Function = UNRB_FUNCTION_CONTROL_TRANSFER;
	punrb->Header.Status = 0;
	punrb->Header.Context = 0;

	punrb->ControlTransfer.Endpoint = usb_pipeendpoint(purb->pipe);
	punrb->ControlTransfer.Flags = usb_pipein(purb->pipe) ? UNRB_FLAG_DIRECTION_IN : 0;
	if(usb_pipein(purb->pipe))
		punrb->ControlTransfer.Flags |= (purb->transfer_flags&URB_SHORT_NOT_OK) ? 0 : UNRB_FLAG_SHORT_TRANSFER_OK; 
	punrb->ControlTransfer.RequestType = pctrlreq->bRequestType;
	punrb->ControlTransfer.Request = pctrlreq->bRequest;
	punrb->ControlTransfer.Value = le16_to_cpu(pctrlreq->wValue);
	punrb->ControlTransfer.Index = le16_to_cpu(pctrlreq->wIndex);
	punrb->ControlTransfer.BufferSize = purb->transfer_buffer_length;

	if( purb->transfer_buffer && usb_pipeout(purb->pipe) )
	{
		memcpy((char*)buf+unrb_size, purb->transfer_buffer, purb->transfer_buffer_length);
		punrb->Header.Size += purb->transfer_buffer_length;
	}

	return result;
}

int usbd_pack_urb_descriptor_request(struct urb *purb, uint64 unique_id, struct usb_ctrlrequest *pctrlreq, void *buf, int get_request)
{
	int result = 0;
	PUNRB punrb = buf;
	int unrb_size = sizeof(UNRB_DESCRIPTOR_REQUEST);

	punrb->Header.UniqueId = unique_id;
	punrb->Header.Size= unrb_size;
	punrb->Header.Function = get_request ? UNRB_FUNCTION_GET_DESCRIPTOR : UNRB_FUNCTION_SET_DESCRIPTOR;
	punrb->Header.Status = 0;
	punrb->Header.Context = 0;


	punrb->DescriptorRequest.LangId = le16_to_cpu(pctrlreq->wIndex);
	punrb->DescriptorRequest.BufferSize = purb->transfer_buffer_length;
	punrb->DescriptorRequest.DescType = (le16_to_cpu(pctrlreq->wValue) >> 8) & 0xFF;
	punrb->DescriptorRequest.DescIndex = le16_to_cpu(pctrlreq->wValue) & 0xFF;

	switch(pctrlreq->bRequestType&USB_TYPE_MASK)
	{
		case USB_TYPE_STANDARD: punrb->DescriptorRequest.RequestType = UNRB_RT_TYPE_STANDARD; break;
		case USB_TYPE_CLASS: punrb->DescriptorRequest.RequestType = UNRB_RT_TYPE_CLASS; break;
		case USB_TYPE_VENDOR: punrb->DescriptorRequest.RequestType = UNRB_RT_TYPE_VENDOR; break;
		case USB_TYPE_RESERVED: punrb->DescriptorRequest.RequestType = UNRB_RT_TYPE_RESERVED; break;
		default: punrb->DescriptorRequest.RequestType = 0;
	}

	switch(pctrlreq->bRequestType&USB_RECIP_MASK)
	{
		case USB_RECIP_DEVICE: punrb->DescriptorRequest.RequestRecipient = UNRB_RT_RECIPIENT_DEVICE; break;
		case USB_RECIP_INTERFACE: punrb->DescriptorRequest.RequestRecipient = UNRB_RT_RECIPIENT_INTERFACE; break;
		case USB_RECIP_ENDPOINT: punrb->DescriptorRequest.RequestRecipient = UNRB_RT_RECIPIENT_ENDPOINT; break;
		case USB_RECIP_OTHER: punrb->DescriptorRequest.RequestRecipient = UNRB_RT_RECIPIENT_OTHER; break;
		default: punrb->DescriptorRequest.RequestType = 0;
	}

	if( purb->transfer_buffer && !get_request )
	{
		memcpy((char*)buf+unrb_size, purb->transfer_buffer, purb->transfer_buffer_length);
		punrb->Header.Size += purb->transfer_buffer_length;
	}

	return result;
}

int usbd_pack_urb_set_interface(struct urb *purb, uint64 unique_id, struct usb_ctrlrequest *pctrlreq, void *buf)
{
	int result = 0;
	PUNRB punrb = buf;
	int full_unrb_size = sizeof(UNRB_SELECT_INTERFACE);

	punrb->Header.UniqueId = unique_id;
	punrb->Header.Size= full_unrb_size;
	punrb->Header.Function = UNRB_FUNCTION_SELECT_INTERFACE;
	punrb->Header.Status = 0;
	punrb->Header.Context = 0;

	punrb->SelectInterface.InterfaceNum = (uint8)le16_to_cpu(pctrlreq->wIndex);
	punrb->SelectInterface.AlternateNum = (uint8)le16_to_cpu(pctrlreq->wValue);

	return result;
}

int usbd_pack_urb_set_configuration(struct urb *purb, uint64 unique_id, struct usb_ctrlrequest *pctrlreq, void *buf)
{
	int result = 0;
	PUNRB punrb = buf;
	int full_unrb_size = sizeof(UNRB_SELECT_CONFIGURATION);

	punrb->Header.UniqueId = unique_id;
	punrb->Header.Size= full_unrb_size;
	punrb->Header.Function = UNRB_FUNCTION_SELECT_CONFIGURATION;
	punrb->Header.Status = 0;
	punrb->Header.Context = 0;

	punrb->SelectConfiguration.ConfigurationNum = (uint8)le16_to_cpu(pctrlreq->wValue);

	return result;
}

int usbd_pack_urb_clear_halt(struct urb *purb, uint64 unique_id, struct usb_ctrlrequest *pctrlreq, void *buf)
{
	int result = 0;
	PUNRB punrb = buf;
	int full_unrb_size = sizeof(UNRB_CLEAR_STALL);

	punrb->Header.UniqueId = unique_id;
	punrb->Header.Size= full_unrb_size;
	punrb->Header.Function = UNRB_FUNCTION_CLEAR_STALL;
	punrb->Header.Status = 0;
	punrb->Header.Context = 0;

	punrb->ClearStall.Endpoint	= usb_pipeendpoint(purb->pipe);
	punrb->ClearStall.Flags		= usb_pipein(purb->pipe) ? UNRB_FLAG_DIRECTION_IN : 0; 

	return result;
}

int usbd_pack_urb(struct urb *purb, uint64 unique_id, void *buf)
{
	int result = -EINVAL;

	do /*while(0)*/
	{
		struct usb_ctrlrequest *pctrlreq;

		if( !purb ) break;

		switch(usb_pipetype(purb->pipe))
		{
			case PIPE_BULK:
				result = usbd_pack_urb_bulk(purb, unique_id, buf);
				break;

			case PIPE_INTERRUPT:
				result = usbd_pack_urb_interrupt(purb, unique_id, buf);
				break;

			case PIPE_ISOCHRONOUS:
				result = usbd_pack_urb_isoch(purb, unique_id, buf);
				break;

			case PIPE_CONTROL:

				pctrlreq = (struct usb_ctrlrequest *)purb->setup_packet;
				if(!pctrlreq) break;

				if(pctrlreq->bRequestType == (USB_DIR_IN|USB_TYPE_STANDARD|USB_RECIP_DEVICE) &&
					pctrlreq->bRequest == USB_REQ_GET_DESCRIPTOR)
				{
					// standard GET_DESCRIPTOR request
					result = usbd_pack_urb_descriptor_request(purb, unique_id, pctrlreq, buf, 1);

				} else if(pctrlreq->bRequestType == (USB_DIR_OUT|USB_TYPE_STANDARD|USB_RECIP_DEVICE) &&
					pctrlreq->bRequest == USB_REQ_SET_DESCRIPTOR)
				{
					// standard SET_DESCRIPTOR request
					result = usbd_pack_urb_descriptor_request(purb, unique_id, pctrlreq, buf, 0);

				} else if(pctrlreq->bRequestType == (USB_DIR_OUT|USB_TYPE_STANDARD|USB_RECIP_DEVICE) &&
					pctrlreq->bRequest == USB_REQ_SET_CONFIGURATION)
				{
					// standard SET_CONFIGURATION request
					result = usbd_pack_urb_set_configuration(purb, unique_id, pctrlreq, buf);

				} else if(pctrlreq->bRequestType == (USB_DIR_OUT|USB_TYPE_STANDARD|USB_RECIP_INTERFACE) &&
					pctrlreq->bRequest == USB_REQ_SET_INTERFACE)
				{
					// standard SET_INTERFACE request
					result = usbd_pack_urb_set_interface(purb, unique_id, pctrlreq, buf);

				} else if(pctrlreq->bRequestType == (USB_DIR_OUT|USB_TYPE_STANDARD|USB_RECIP_ENDPOINT) &&
					pctrlreq->bRequest == USB_REQ_CLEAR_FEATURE &&
					pctrlreq->wValue == USB_ENDPOINT_HALT)
				{
					// clear endpoint HALT
					result = usbd_pack_urb_clear_halt(purb, unique_id, pctrlreq, buf);				

				} else
				{
					// generic control transfer
					result = usbd_pack_urb_control(purb, unique_id, pctrlreq, buf);
				}
				break;
		}

	} while(0);

	return result;
}


int usbd_unpack_urb(PUNRB punrb, struct urb *purb)
{
	int status = -1;

	status = ConvertUnrbStatusToLinux(punrb->Header.Status);

	switch( punrb->Header.Function )
	{
		case UNRB_FUNCTION_GET_DESCRIPTOR:
			TRACE("usbd_unpack_urb: GetDescriptor\n");

			purb->actual_length = MIN_OF(purb->transfer_buffer_length, punrb->DescriptorRequest.BufferSize);

			if( purb->transfer_buffer )
				memcpy(purb->transfer_buffer, (char*)punrb+sizeof(punrb->DescriptorRequest), purb->actual_length);
			break;

		case UNRB_FUNCTION_SET_DESCRIPTOR:
			TRACE("usbd_unpack_urb: SetDescriptor\n");

			purb->actual_length = MIN_OF(purb->transfer_buffer_length, punrb->DescriptorRequest.BufferSize);
			break;

		case UNRB_FUNCTION_SELECT_CONFIGURATION:
			TRACE("usbd_unpack_urb: SelectConfiguration\n");
			break;

		case UNRB_FUNCTION_SELECT_INTERFACE:
			TRACE("usbd_unpack_urb: SelectInterface\n");
			break;

		case UNRB_FUNCTION_CONTROL_TRANSFER:
			TRACE("usbd_unpack_urb: ControlTransfer\n");
			purb->actual_length = MIN_OF(purb->transfer_buffer_length, punrb->ControlTransfer.BufferSize);

			if( purb->transfer_buffer && usb_pipein(purb->pipe) )
				memcpy(purb->transfer_buffer, (char*)punrb+sizeof(punrb->ControlTransfer), purb->actual_length);
			break;

		case UNRB_FUNCTION_BULK_TRANSFER:
			TRACE("usbd_unpack_urb: BulkTransfer\n");
			purb->actual_length = MIN_OF(purb->transfer_buffer_length, punrb->BulkTransfer.BufferSize);

			if( purb->transfer_buffer && usb_pipein(purb->pipe) )
				memcpy(purb->transfer_buffer, (char*)punrb+sizeof(punrb->BulkTransfer), purb->actual_length);
			break;

		case UNRB_FUNCTION_ISOCH_TRANSFER:
			TRACE("usbd_unpack_urb: IsochTransfer\n");
			purb->start_frame = punrb->IsochTransfer.StartFrame;
			purb->error_count = punrb->IsochTransfer.ErrorCount;
			purb->actual_length = 0;

			do
			{
				uint8 *unrb_buffer = (uint8*)punrb + GET_UNRB_ISOCH_TRANSFER_PACKETSIZE(punrb);
				int i, n = MIN_OF(punrb->IsochTransfer.NumberOfPackets, purb->number_of_packets);

				for(i=0; i < n; i++)
				{
					purb->iso_frame_desc[i].status = ConvertUnrbStatusToLinux(punrb->IsochTransfer.IsochPackets[i].Status);
					purb->iso_frame_desc[i].actual_length = punrb->IsochTransfer.IsochPackets[i].Length;

					if( purb->transfer_buffer && usb_pipein(purb->pipe) )
					{
						memcpy(
							purb->transfer_buffer + purb->iso_frame_desc[i].offset,
							unrb_buffer + purb->actual_length,
							purb->iso_frame_desc[i].actual_length);
					}

					purb->actual_length += purb->iso_frame_desc[i].actual_length;
				}


			} while(0);
			break;

		case UNRB_FUNCTION_INTERRUPT_TRANSFER:
			TRACE("usbd_unpack_urb: InterruptTransfer\n");
			purb->actual_length = MIN_OF(purb->transfer_buffer_length, punrb->InterruptTransfer.BufferSize);

			if( purb->transfer_buffer && usb_pipein(purb->pipe) )
				memcpy(purb->transfer_buffer, (char*)punrb+sizeof(punrb->InterruptTransfer), purb->actual_length);
			break;

		case UNRB_FUNCTION_CLEAR_STALL:
			TRACE("usbd_unpack_urb: ClearStall\n");
			break;

		default:
			TRACE("usbd_unpack_urb: unknown function\n");
			status = -EINVAL;
			break;
	}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24)
	purb->status = status;
#endif

	//dump_unrb(punrb);

	return status;
}

#endif // #ifdef _USBD_ENABLE_VHCI_

int usbd_map_vmem(void *vmem, struct vm_area_struct *vma, unsigned long len)
{
	int result = 0;
	unsigned long start = vma->vm_start;
	unsigned long pfn;

	TRACE("usbd_map_vmem: ++\n");

	if( (vma->vm_end - vma->vm_start) > len )
	{
		TRACE("usbd_map_vmem: -- invalid length\n");
		return -EINVAL;
	}

	/* vmalloc'ed memory is not physically contiguous, 
	so we have to map each page separately */
	while (len > 0) 
	{
		pfn = vmalloc_to_pfn(vmem);
			
		result = remap_pfn_range(vma, start, pfn, PAGE_SIZE, PAGE_SHARED);

		if( result < 0 ) 
		{
			TRACE("usbd_map_vmem: cannot map the page\n");
			break;
		}
		start += PAGE_SIZE;
		vmem  += PAGE_SIZE;
		len -= PAGE_SIZE;
	}

	TRACE("usbd_map_vmem: -- result=%d\n", result);
	return result;
}

int ConvertUnrbStatusToLinux(uint32 status)
{
	int result;

	switch(status)
	{
		case UNRB_STATUS_SUCCESS:
			result = 0;
			break;
		case UNRB_STATUS_PENDING:
			result = -EINPROGRESS;
			break;
		case UNRB_STATUS_CANCELLED:
			result = -ECONNRESET;
			break;
		case UNRB_STATUS_ISO_TRANSFER_INCOMPLETE:
			result = -EXDEV;
			break;
		case UNRB_STATUS_TIMEOUT:
			result = -ETIMEDOUT;
			break;
		case UNRB_STATUS_REQUEST_FAILED:
			result = -EINVAL;
			break;
		case UNRB_STATUS_NO_DEVICE:
			result = -ENODEV;
			break;
		case UNRB_STATUS_SHORT_TRANSFER:
			result = -EREMOTEIO;
			break;
		case UNRB_STATUS_BABBLE_DETECTED:
			result = -EOVERFLOW;
			break;
		case UNRB_STATUS_DATA_UNDERRUN:
			result = -ENOSR;
			break;
		case UNRB_STATUS_DATA_OVERRUN:
			result = -ECOMM;
			break;
		case UNRB_STATUS_STALLED:
			result = -EPIPE;
			break;
		case UNRB_STATUS_CRC_ERROR:
			result = -EILSEQ;
			break;
		case UNRB_STATUS_PROTOCOL_ERROR:
			result = -EPROTO;
			break;
		case UNRB_STATUS_NOT_ENOUGH_MEMORY:
			result = -ENOMEM;
			break;
		case UNRB_STATUS_NOT_ENOUGH_BANDWIDTH:
			result = -EPROTO;
			break;
		case UNRB_STATUS_UNSUCCESSFUL:
		default:
			result = -EPROTO;
			break;
	}

	return result;		
}

size_t usbd_pack_iso_buffer(struct usb_iso_packet_descriptor *packets, int count, void *dst, void *src, int use_actual_length)
{
	uint8 *dstbuffer;
	size_t result;

	result = 0;
	dstbuffer = dst;

	for(; count; count--)
	{
		uint8 *srcbuffer = src + packets->offset;
		unsigned int length = use_actual_length ? packets->actual_length : packets->length;

		if(length && dstbuffer != srcbuffer)
			// buffers can overlap - use memmove
			memmove(dstbuffer, srcbuffer, length);

		dstbuffer += length;
		result += length;
		packets++;
	}

	TRACE("usbd_pack_iso_buffer: packed=%d\n",result);
	return result;
}
            
/*size_t usbd_unpack_iso_buffer(struct usb_iso_packet_descriptor *packets, int count, void *buffer);
{
	uint8 *srcbuffer;
	size_t result;

	result = 0;
	srcbuffer = buffer + packedlen;

	for(; count; count--)
	{
		uint8 *dstbuffer = buffer + packets[count-1].Offset - delta;

		srcbuffer -= packets[count-1].Length;

		if(packets[count-1].Length && dstbuffer != srcbuffer)
			// buffers can overlap - use memmove
			memmove(dstbuffer, srcbuffer, packets[count-1].Length);

		result += packets[count-1].Length;
	}

	TRACE("usbd_unpack_iso_buffer: packed=%d unpacked=%d\n", packedlen,result);
	return result;
}*/


#ifndef _USBD_DEBUG_BUILD_
void dump_urb(struct urb *purb, uint64 unique_id)
{
}
#else
void dump_urb(struct urb *purb, uint64 unique_id)
{
	const char* pipe_types[] = {"iso", "int", "ctrl", "bulk"};
	if(!purb) return;

	TRACE("++++ URB START ++++\n");
	TRACE("  URB=0x%p\n", purb);
	TRACE("  Pipe=0x%08X (dev: %d endp: %d dir: %s type: %s)\n", 
		purb->pipe, 
		usb_pipedevice(purb->pipe),
		usb_pipeendpoint(purb->pipe),
		usb_pipein(purb->pipe) ? "in" : "out",
		pipe_types[usb_pipetype(purb->pipe)]);
	TRACE("  Status=%d\n", purb->status);
	TRACE("  TransferFlags=0x%08X\n", purb->transfer_flags);
	TRACE("  Buffer=0x%08X\n", (u32)purb->transfer_buffer);
	TRACE("  BufferLength=%d\n", purb->transfer_buffer_length);
	TRACE("  ActualLength=%d\n", purb->actual_length);

	if( usb_pipecontrol(purb->pipe) )
	{
		TRACE("  SetupPacket=%02X %02X %02X %02X %02X %02X %02X %02X\n", 
			purb->setup_packet[0],purb->setup_packet[1],
			purb->setup_packet[2],purb->setup_packet[3],
			purb->setup_packet[4],purb->setup_packet[5],
			purb->setup_packet[6],purb->setup_packet[7]);

	} else if( usb_pipeisoc(purb->pipe) )
	{
		int i;
		TRACE("  StartFrame=%d\n", purb->start_frame);
		TRACE("  NumberOfPackets=%d\n", purb->number_of_packets);
		TRACE("  Interval=%d\n", purb->interval);
		TRACE("  ErrorCount=%d\n", purb->error_count);
		TRACE("  IsoFrames=\n");

		for(i=0;i<purb->number_of_packets;i++)
		{
			TRACE("    [%d] Offset=%d Length=%d ActualLength=%d Status=%d\n",
				i,
				purb->iso_frame_desc[i].offset, 
				purb->iso_frame_desc[i].length, 
				purb->iso_frame_desc[i].actual_length, 
				purb->iso_frame_desc[i].status);
		}
	} else if( usb_pipeint(purb->pipe) )
	{
		TRACE("  Interval=%d\n", purb->interval);
	}
	TRACE("---- URB END ----\n");
}
#endif //_USBD_DEBUG_BUILD_



#ifndef _USBD_DEBUG_BUILD_

void dump_unrb(PUNRB punrb)
{
}

#else

static void dump_unrb_header(PUNRB punrb);

void dump_unrb(PUNRB punrb)
{
	int i;
	//char *buf;
	if( !punrb ) return;

	TRACE("---- UNRB START\n");

	switch( punrb->Header.Function )
	{
		case UNRB_FUNCTION_GET_DESCRIPTOR:
			TRACE("UNRB_FUNCTION_GET_DESCRIPTOR\n");

			dump_unrb_header(punrb);

			TRACE("  DescriptorRequest\n");
			TRACE("    RequestType=%d\n",punrb->DescriptorRequest.RequestType);
			TRACE("    RequestRecipient=%d\n",punrb->DescriptorRequest.RequestRecipient);
			TRACE("    DescType=%d\n",punrb->DescriptorRequest.DescType);
			TRACE("    DescIndex=%d\n",punrb->DescriptorRequest.DescIndex);
			TRACE("    LangId=%d\n",punrb->DescriptorRequest.LangId);
			TRACE("    BufferSize=%d\n",punrb->DescriptorRequest.BufferSize);

			break;

		case UNRB_FUNCTION_SET_DESCRIPTOR:
			TRACE("UNRB_FUNCTION_SET_DESCRIPTOR\n");

			dump_unrb_header(punrb);

			TRACE("  DescriptorRequest\n");
			TRACE("    DescType=%d\n",punrb->DescriptorRequest.DescType);
			TRACE("    DescIndex=%d\n\n",punrb->DescriptorRequest.DescIndex);
			TRACE("    LangId=%d\n",punrb->DescriptorRequest.LangId);
			TRACE("    BufferSize=%d\n",punrb->DescriptorRequest.BufferSize);

			break;

		case UNRB_FUNCTION_SELECT_CONFIGURATION:
			TRACE("UNRB_FUNCTION_SELECT_CONFIGURATION\n");

			dump_unrb_header(punrb);

			TRACE("  SelectConfiguration\n");
			TRACE("    Configuration=%d\n", punrb->SelectConfiguration.ConfigurationNum);
			TRACE("    NumAlternates=%d\n", punrb->SelectConfiguration.NumAlternates);
			for(i=0;i<punrb->SelectConfiguration.NumAlternates;i++)
			{
				TRACE("      %.3d: InterfaceNum=%d\n", i, punrb->SelectConfiguration.Alternates[i].InterfaceNum);
				TRACE("           AlternateNum=%d\n", punrb->SelectConfiguration.Alternates[i].AlternateNum);
			}

			break;

		case UNRB_FUNCTION_SELECT_INTERFACE:
			TRACE("UNRB_FUNCTION_SELECT_INTERFACE\n");

			dump_unrb_header(punrb);

			TRACE("  SelectInterface\n");
			TRACE("    Interface=%d\n", punrb->SelectInterface.InterfaceNum);
			TRACE("    Alternate=%d\n", punrb->SelectInterface.AlternateNum);

			break;

		case UNRB_FUNCTION_CONTROL_TRANSFER:
			TRACE("UNRB_FUNCTION_CONTROL_TRANSFER\n");

			dump_unrb_header(punrb);

			TRACE("  ControlTransfer\n");
			TRACE("    Endpoint=%d\n", punrb->ControlTransfer.Endpoint);
			TRACE("    Flags=0x%.2X\n", punrb->ControlTransfer.Flags);
			TRACE("    RequestType=0x%.2X\n", punrb->ControlTransfer.RequestType);
			TRACE("    Request=0x%.2X\n", punrb->ControlTransfer.Request);
			TRACE("    Value=0x%.4X\n", punrb->ControlTransfer.Value);
			TRACE("    Index=0x%.4X\n", punrb->ControlTransfer.Index);
			TRACE("    BuferSize=%d\n", punrb->ControlTransfer.BufferSize);
			break;

		case UNRB_FUNCTION_BULK_TRANSFER:
			TRACE("UNRB_FUNCTION_BULK_TRANSFER\n");

			dump_unrb_header(punrb);

			TRACE("  BulkTransfer\n");
			TRACE("    Endpoint=%d\n", punrb->BulkTransfer.Endpoint);
			TRACE("    Flags=0x%.2X\n", punrb->BulkTransfer.Flags);
			TRACE("    BuferSize=%d\n", punrb->BulkTransfer.BufferSize);
			break;

		case UNRB_FUNCTION_INTERRUPT_TRANSFER:
			TRACE("UNRB_FUNCTION_INTERRUPT_TRANSFER\n");

			dump_unrb_header(punrb);

			TRACE("  InterruptTransfer\n");
			TRACE("    Endpoint=%d\n", punrb->InterruptTransfer.Endpoint);
			TRACE("    Flags=0x%.2X\n", punrb->InterruptTransfer.Flags);
			TRACE("    Interval=%d\n", punrb->InterruptTransfer.Interval);
			TRACE("    BuferSize=%d\n", punrb->InterruptTransfer.BufferSize);
			break;

		case UNRB_FUNCTION_ISOCH_TRANSFER:
			TRACE("UNRB_FUNCTION_ISOCH_TRANSFER\n");

			dump_unrb_header(punrb);

			TRACE("  IsochTransfer\n");
			TRACE("    Endpoint=%d\n", punrb->IsochTransfer.Endpoint);
			TRACE("    Flags=0x%.2X\n", punrb->IsochTransfer.Flags);
			TRACE("    BuferSize=%d\n", punrb->IsochTransfer.BufferSize);
			TRACE("    Interval=%d\n", punrb->IsochTransfer.Interval);
			TRACE("    StartFrame=%d\n", punrb->IsochTransfer.StartFrame);
			TRACE("    NumberOfPackets=%d\n", punrb->IsochTransfer.NumberOfPackets);
			TRACE("    ErrorCount=%d\n", punrb->IsochTransfer.ErrorCount);
			TRACE("    IsochPackets:\n");
			
			for(i=0;i<punrb->IsochTransfer.NumberOfPackets;i++)
			{
				TRACE("      %.3d: Offset=%d\n", i, punrb->IsochTransfer.IsochPackets[i].Offset);
				TRACE("           Length=%d\n", punrb->IsochTransfer.IsochPackets[i].Length);
				TRACE("           Status=%d\n", punrb->IsochTransfer.IsochPackets[i].Status);
			}
			break;

		case UNRB_FUNCTION_CLEAR_STALL:
			TRACE("UNRB_FUNCTION_CLEAR_STALL\n");

			dump_unrb_header(punrb);

			TRACE("  ClearStall\n");
			TRACE("    Endpoint=%d\n", punrb->ClearStall.Endpoint);
			TRACE("    Flags=0x%.2X\n", punrb->ClearStall.Flags);
			break;

		case UNRB_FUNCTION_GET_CURRENT_FRAME_NUMBER:
			TRACE("UNRB_FUNCTION_GET_CURRENT_FRAME_NUMBER\n");

			dump_unrb_header(punrb);

			TRACE("  GetCurrentFrameNumber\n");
			TRACE("    FrameNumber=%d\n", punrb->GetCurrentFrameNumber.FrameNumber);
			break;

		case UNRB_FUNCTION_GET_PORT_STATUS:
			TRACE("UNRB_FUNCTION_GET_PORT_STATUS\n");

			dump_unrb_header(punrb);

			TRACE("  GetPortStatus\n");
			switch(punrb->GetPortStatus.PortStatus)
			{
				case UNRB_PORT_STATUS_ENABLED:
					TRACE("    PortStatus=ENABLED\n");
					break;
				case UNRB_PORT_STATUS_CONNECTED:
					TRACE("    PortStatus=CONNECTED\n");
					break;
				default:
					TRACE("    PortStatus=unknown\n");
					break;
			}
			break;

		case UNRB_FUNCTION_RESET_PORT:
			TRACE("UNRB_FUNCTION_RESET_PORT\n");

			dump_unrb_header(punrb);
			break;

		case UNRB_FUNCTION_CANCEL:
			TRACE("UNRB_FUNCTION_CANCEL\n");

			dump_unrb_header(punrb);
			break;

		case UNRB_FUNCTION_ABORT_ENDPOINT:
			TRACE("UNRB_FUNCTION_ABORT_ENDPOINT\n");

			dump_unrb_header(punrb);
			TRACE("  AbortEndpoint\n");
			TRACE("    Endpoint=%d\n", punrb->AbortEndpoint.Endpoint);
			TRACE("    Flags=0x%.2X\n", punrb->AbortEndpoint.Flags);
			break;

		default:
			TRACE("UNRB_FUNCTION_unknown\n");
			dump_unrb_header(punrb);
			break;
	}
	TRACE("---- UNRB END\n");
}

static void dump_unrb_header(PUNRB punrb)
{
	TRACE("  Header\n");
	TRACE("    UniqueId=0x%.8X%.8X\n", (uint32)(punrb->Header.UniqueId>>32), (uint32)(punrb->Header.UniqueId));
	TRACE("    Size=%d\n", punrb->Header.Size);
	TRACE("    Function=%d\n", punrb->Header.Function);
	TRACE("    Status=%d\n", punrb->Header.Status);
	TRACE("    Context=0x%.8X\n", punrb->Header.Context);
}

#endif //_USBD_DEBUG_BUILD_
