/*
 *  tusbd/urb_chain.c
 *
 *  Copyright (C) 2007-2008 IncentivesPro
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifdef _USBD_ENABLE_STUB_

#include "usbd.h"

int usbd_uc_maxbuffersize = 32768;

int usbd_uc_abort(struct urb_chain *pchain, struct urb* startafter);
void usbd_uc_urb_complete(
	struct urb* purb
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
	,struct pt_regs* ptregs
#endif
);

int usbd_uc_init(
	struct urb_chain *pchain, 
	int pipe,
	int interval,
	int short_ok,
	int is_ehci,
	struct usb_device *pdev,
	struct buf_chain *pbufchain, 
	void *context,
	urb_chain_complete_t complete)
{
	int i, result = 0;
	struct usb_host_endpoint *ep;

	TRACE("usbd_uc_init: ++\n");

	pchain->pipe = pipe;
	pchain->short_ok = short_ok;
	pchain->is_ehci = is_ehci;
	pchain->pdev = pdev;
	pchain->count = pbufchain->count;
	pchain->status = 0;
	pchain->bytes = 0;
	pchain->context = context;
	pchain->complete = complete;
	
	spin_lock_init(&pchain->lock);

	ep = usb_pipein(pipe) ? 
			pdev->ep_in[usb_pipeendpoint(pipe)] : 
			pdev->ep_out[usb_pipeendpoint(pipe)];

	for(i=0; i<pchain->count; i++)
	{
		// check the result of previous iteration
		if( result < 0 ) 
			break;
                                 
		pchain->urbs[i] = usb_alloc_urb(pbufchain->isopackets[i], GFP_KERNEL);

		if( !pchain->urbs[i] )
		{
			TRACE_CRITICAL("usbd_uc_init: failed to allocate an urb - no memory\n");
			result = -ENOMEM;
			break;
		}

		pchain->urbs[i]->dev = NULL; // mark as unsubmitted
		pchain->urbs[i]->transfer_flags |= URB_NO_INTERRUPT;

		switch(usb_pipetype(pipe))
		{
			case PIPE_BULK:
				usb_fill_bulk_urb(
					pchain->urbs[i],
					pchain->pdev,
					pipe,
					pbufchain->buffers[i],
					usb_pipein(pipe) ? pbufchain->buffersize[i] : pbufchain->datasize[i],
					usbd_uc_urb_complete,
					pchain);

				if( usb_pipein(pipe) )
					pchain->urbs[i]->transfer_flags |= URB_SHORT_NOT_OK;

				break;

			case PIPE_ISOCHRONOUS:
				pchain->urbs[i]->dev				= pdev;
				pchain->urbs[i]->pipe				= pipe;
				pchain->urbs[i]->transfer_flags		= URB_ISO_ASAP;
				pchain->urbs[i]->transfer_buffer	= pbufchain->buffers[i];
				pchain->urbs[i]->transfer_buffer_length = usb_pipein(pipe) ? pbufchain->buffersize[i] : pbufchain->datasize[i];
				pchain->urbs[i]->start_frame		= -1;
				pchain->urbs[i]->number_of_packets	= pbufchain->isopackets[i];
				pchain->urbs[i]->context			= pchain;
				pchain->urbs[i]->complete			= usbd_uc_urb_complete;
				if(interval)
				{
					pchain->urbs[i]->interval 		= interval;
				} else
				{
					pchain->urbs[i]->interval 		= ep ? ep->desc.bInterval : 1;
					pchain->urbs[i]->interval 		= 1 << (pchain->urbs[i]->interval - 1);
				}
				break;

			/* TODO: support interrupt transfers
			case PIPE_INTERRUPT:
				usb_fill_int_urb(
					pchain->urbs[i],
					pchain->pdev,
					pipe,
					pbufchain->buffers[i],
					pbufchain->buffersize[i],
					usbd_uc_urb_complete,
					pchain,
					ep ? ep->desc.bInterval : 1);
				break;
			*/ 

			default:
				TRACE_CRITICAL("usbd_uc_init: invalid pipe type\n");
				result = -EINVAL;
				break;
		}
	}

	if( result == 0 )
	{
		// remove URB_NO_INTERRUPT flag on the last urb
		pchain->urbs[pchain->count-1]->transfer_flags &= ~URB_NO_INTERRUPT;
	}
	else
	{
		// cleanup on error
		for(i--; i>=0; i--) 
		{
			usb_free_urb(pchain->urbs[i]);
			pchain->urbs[i] = NULL;
		}
	}

	TRACE("usbd_uc_init: -- result=%d\n", result);
	return result;
}

int usbd_uc_clean(struct urb_chain *pchain)
{
	int i, result = 0;
	TRACE("usbd_uc_clean\n");

	usbd_uc_cancel(pchain);

	for(i=0; i<pchain->count; i++)
		if (pchain->urbs[i])
			usb_free_urb(pchain->urbs[i]);

	return result;
}

void usbd_uc_urb_complete(
	struct urb* purb
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
	,struct pt_regs* ptregs
#endif
)
{
	struct urb_chain *pchain = purb->context;

	/* 
		if urb completes with error, we should cancel all outstanding urbs in chain
	*/
	if(purb->status && purb->status != -ECONNRESET && purb->status != -ENOENT)
	{
		int abort = 0;

		spin_lock(&pchain->lock);
		if( pchain->status == 0 )
		{
			// if this was a short transfer and we're ok with them, leave chain status=0
			if( purb->status == -EREMOTEIO && pchain->short_ok )
			{
				pchain->status = 0;
				TRACE_CRITICAL("usbd_uc_urb_complete: short transfer detected and is ok. actual_length=%d\n", purb->actual_length);
			}
			else
			{
				pchain->status = purb->status;
				TRACE_CRITICAL("usbd_uc_urb_complete: early urb completed with status %d\n", purb->status);
			}

			abort = 1;
		}
		spin_unlock(&pchain->lock);

		if( abort )
			usbd_uc_abort(pchain, purb);
	}

#if defined(_USBD_USE_EHCI_FIX_) && LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,25) && LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,28)
	if(pchain->is_ehci && usb_pipeisoc(purb->pipe) && purb->status == 0)
	{
		if(atomic_read(&purb->kref.refcount)>1)
				usb_put_urb(purb);
	}
#endif

	purb->dev = NULL; // mark as unsubmitted

	pchain->bytes += purb->actual_length;

	//TRACE("usbd_uc_urb_complete: urb %p completed with status %d actual_length=%d chain bytes=%d\n", purb, purb->status, purb->actual_length, pchain->bytes);

	if( atomic_dec_and_test(&pchain->iocount) )
	{
		TRACE("usbd_uc_urb_complete: all urbs finished - completing the chain\n");
		pchain->complete(pchain);
	}
}

int usbd_uc_submit(struct urb_chain *pchain)
{
	int i, j, result = 0;
	unsigned long flags;

	TRACE("usbd_uc_submit: ++\n");

	spin_lock_irqsave(&pchain->lock, flags);

	atomic_set(&pchain->iocount, pchain->count);

	for(i=0, j=0; i<pchain->count; i++)
	{
		if(pchain->status != 0)
			break;
	
		pchain->urbs[i]->dev = pchain->pdev; // mark as submitted

		result = usb_submit_urb(pchain->urbs[i], GFP_ATOMIC);

		if( result != 0 ) 
			pchain->urbs[i]->dev = NULL; // mark as not-submitted

		spin_unlock(&pchain->lock);

		if( result == -ENXIO || result == -EAGAIN || result == -ENOMEM ) /*TODO: limit number of iterations*/
		{
			TRACE("usbd_uc_submit: retrying failed urb\n");
			// retry this urb 10 times
			if( j++ < 10 )
			{			
				i--;
				result = 0;
				yield ();
			}
		} else
		{
			cpu_relax();
			j = 0;
		}

		spin_lock(&pchain->lock);

		if( result != 0 )
		{
			TRACE("usbd_uc_submit: submit failed\n");
			pchain->status = -1;
			break;
		}
	}

	spin_unlock_irqrestore(&pchain->lock, flags);

	// on error, cancel all submitted urbs and don't call completion routine
	if( result < 0 )
		usbd_uc_abort(pchain,NULL);
	else
	{
		int unsubmitted = pchain->count - i;
		// is zero?
		if( unsubmitted > 0 && atomic_sub_and_test(unsubmitted, &pchain->iocount) )
		{
			TRACE("usbd_uc_submit: iocont is 0, calling completion\n");
			pchain->complete(pchain);
		}
	}

	TRACE("usbd_uc_submit: -- result=%d\n", result);

	return result;
}

int usbd_uc_get(struct urb_chain *pchain)
{
	int i;

	for(i=0; i<pchain->count; i++)
		usb_get_urb(pchain->urbs[i]);
	return 0;
}

int usbd_uc_put(struct urb_chain *pchain)
{
	int i;

	for(i=0; i<pchain->count; i++)
		usb_put_urb(pchain->urbs[i]);
	return 0;
}

int usbd_uc_cancel(struct urb_chain *pchain)
{
	unsigned long flags;
	int abort = 0;

	TRACE("usbd_uc_cancel: ++\n");

	spin_lock_irqsave(&pchain->lock, flags);
	if(pchain->status == 0)
	{
		pchain->status = -ECONNRESET;
		abort = 1;
	}
	spin_unlock_irqrestore(&pchain->lock, flags);

	if(abort)
		usbd_uc_abort(pchain, NULL);
	TRACE("usbd_uc_cancel: --\n");

	return 0;
}

int usbd_uc_abort(struct urb_chain *pchain, struct urb* startafter)
{
	int i, found = 0;

	for(i=0; i<pchain->count; i++)
	{
		if (!found && startafter)
		{
			if (pchain->urbs[i] == startafter) found=1;
		}
		else
		{
			if (pchain->urbs[i] && pchain->urbs[i]->dev) usb_unlink_urb(pchain->urbs[i]);
		}
	}
	return 0;
}

int usbd_uc_index_of(struct urb_chain *pchain, struct urb* purb)
{
	int i;
	for(i=0;i<pchain->count;i++) if(pchain->urbs[i] == purb) break;
	return (i == pchain->count) ? -1 : i;
}


int usbd_bc_alloc(struct buf_chain *pchain, size_t length, int multiplicity)
{
	size_t partsize, nalloc;
	int i;

	TRACE("usbd_bc_alloc: ++\n");

	partsize = usbd_uc_maxbuffersize - (usbd_uc_maxbuffersize % multiplicity);

	TRACE("usbd_bc_alloc: partsize = %d\n", partsize);

	pchain->count = 0;

	for(i=0; i<MAX_CHAIN_LENGTH && length>0; i++, length-=nalloc)
	{
		nalloc = MIN_OF(partsize, length);

		TRACE("usbd_bc_alloc: allocating %d bytes\n", nalloc);

		pchain->buffers[i] = kmalloc(nalloc, GFP_KERNEL);
		pchain->buffersize[i] = nalloc;
		pchain->datasize[i] = 0;
		pchain->isopackets[i] = 0;

		if (!pchain->buffers[i]) 
		{
			TRACE("usbd_bc_alloc: kmalloc failed\n");
			break;
		}
	}

	// cleanup on error
	if( length )
		for(i--;i >= 0;i--)
		{
			kfree(pchain->buffers[i]);
			pchain->buffers[i] = NULL;
			pchain->buffersize[i] = 0;
		}
	else
		pchain->count = i;

	TRACE("usbd_bc_alloc: -- result=%d\n", pchain->count ? 0 : -1);

	return pchain->count ? 0 : -1;
}

int usbd_bc_alloc_iso(
	struct buf_chain *pchain, 
	size_t length,
	UNRB_ISOCH_PACKET_DESCRIPTOR *isopackets,
	uint32 nisopackets)
{
	int i,lastpacket;
	size_t fulllength;

	TRACE("usbd_bc_alloc_iso: ++ isopackets=%d length=%d\n", nisopackets, length);

	lastpacket = 0;
	pchain->count = fulllength = 0;

	// loop through all iso packets, but watch for the number of used buffers and buffer size left
	for(i=0; i<nisopackets && pchain->count<MAX_CHAIN_LENGTH && length>0; i++)
	{
		uint32 curoffset, nextoffset;
		int islastpacket = (i == nisopackets-1);

//		TRACE("usbd_bc_alloc_iso: %03d: offset=%3d length=%d\n", i, isopackets[i].Offset, isopackets[i].Length);

		// calc offset of the current iso packet and the next one
		curoffset = isopackets[i].Offset;
		nextoffset = islastpacket ? isopackets[i].Offset + isopackets[i].Length: isopackets[i+1].Offset;

		// check buffer for linearity. current offset should be less than next offset
		if( curoffset > nextoffset )
		{
			// TODO: are there any devices with non-linear buffers?
			TRACE_CRITICAL("usbd_bc_alloc_iso: BUG!!! Non-linear iso buffer. Please report this.\n");
			break;
		}

		// see if we're about to overcome the maximum allowed buffer size
		if( (fulllength + isopackets[i].Length) > usbd_uc_maxbuffersize || islastpacket )
		{
			size_t nalloc;

			if( islastpacket )
				fulllength += isopackets[i].Length;
		
			nalloc = MIN_OF(fulllength, length);

			pchain->buffers[pchain->count] = kmalloc(nalloc, GFP_KERNEL);
			pchain->buffersize[pchain->count] = nalloc;
			pchain->datasize[pchain->count] = 0;
			pchain->isopackets[pchain->count] = (i - lastpacket);

			if( islastpacket )
				pchain->isopackets[pchain->count]++;

			TRACE("usbd_bc_alloc_iso: allocating buffer %d bytes, isopackets=%d\n", nalloc, pchain->isopackets[pchain->count]);

			if( !pchain->buffers[pchain->count] )
			{
				TRACE("usbd_bc_alloc_iso: kmalloc failed\n");
				break;
			}

			pchain->count++;
			length -= nalloc;
			fulllength = 0;
			lastpacket = i;
		}

		fulllength += isopackets[i].Length;
	}

	// cleanup on error
	if( i < nisopackets )
	{
		for(i=0; i<pchain->count; i++)
		{
			kfree(pchain->buffers[i]);
			pchain->buffers[i] = NULL;
		}
		pchain->count = 0;
	}

	TRACE("usbd_bc_alloc_iso: -- result=%d\n", pchain->count ? 0 : -1);

	return pchain->count ? 0 : -1;

}

int usbd_bc_copy_from_user(struct buf_chain *pchain, const void *buf, size_t length)
{
	int i;
	size_t n, result = 0;

	TRACE("usbd_bc_copy_from_user: count=%d\n", pchain->count);

	for(i=0; i<pchain->count && length>0; i++)
	{
		n = MIN_OF(length, pchain->buffersize[i]);

		if( __copy_from_user(pchain->buffers[i], buf, n) != 0 )
		{
			TRACE("usbd_bc_copy_from_user: cannot copy from user buffer\n");
			break;
		}

		pchain->datasize[i] = n;
		buf += n;
		length -= n;
		result += n;
	}

	return result;
}

int usbd_bc_copy_to_user(struct buf_chain *pchain, void *buf, size_t length)
{
	int i;
	size_t n, result = 0;

	TRACE("usbd_bc_copy_to_user: count=%d\n", pchain->count);

	for(i=0; i<pchain->count && length>0; i++)
	{
		n = MIN_OF(length, pchain->datasize[i]);

		if( __copy_to_user(buf, pchain->buffers[i], n) != 0 )
		{
			TRACE("usbd_bc_copy_to_user: cannot copy from user buffer\n");
			break;
		}

		buf += n;
		length -= n;
		result += n;
	}

	return result;
}

int usbd_bc_free(struct buf_chain *pchain)
{
	int i;

	for(i=0; i<pchain->count; i++)
	{
		if( pchain->buffers[i] )
		{
			kfree(pchain->buffers[i]);
			pchain->buffers[i] = NULL;
		}
	}

	return 0;
}

#endif // #ifdef _USBD_ENABLE_STUB_
