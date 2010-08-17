/*
 *  tusbd/minor_vhci.c
 *
 *  Copyright (C) 2007 IncentivesPro
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifdef _USBD_ENABLE_VHCI_

#include "usbd.h"
#include HCD_H_PATH

ssize_t usbd_mv_write(void *, const char __user *, size_t);
ssize_t usbd_mv_read(void *, char __user *, size_t);
int usbd_mv_ioctl(void *, unsigned int, unsigned long);
int usbd_mv_open(void *);
int usbd_mv_release(void *);
unsigned int usbd_mv_poll(void *, struct file *, poll_table *wait);
int usbd_mv_mmap(void *, struct vm_area_struct *);

struct minor_descriptor *usbd_mv_alloc_descriptor(struct vhci_device_descriptor *pdev)
{
	struct minor_descriptor *pdesc;
   	pdesc = kmalloc(sizeof(*pdesc), GFP_KERNEL);

   	if( pdesc )
   	{
	  	memset(pdesc, 0, sizeof(*pdesc));

	  	pdesc->ops.open = usbd_mv_open;
	  	pdesc->ops.release = usbd_mv_release;
	  	pdesc->ops.ioctl = usbd_mv_ioctl;
	  	pdesc->ops.read = usbd_mv_read;
	  	pdesc->ops.write = usbd_mv_write;
	  	pdesc->ops.poll = usbd_mv_poll;
	  	pdesc->ops.mmap = usbd_mv_mmap;

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

void usbd_mv_free_descriptor(struct minor_descriptor *pdesc)
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

ssize_t usbd_mv_read(void *context, char __user *buf, size_t count)
{
	TRACE("usbd_mv_read:\n");
	return 0;
}


ssize_t usbd_mv_write(void *context, const char __user *buf, size_t count)
{
	TRACE("usbd_mv_write\n");
	return 0;
}

int usbd_mv_ioctl(void *context, unsigned int cmd, unsigned long arg)
{
	struct vhci_device_descriptor *pdev = context;
	int result = 0;
	unsigned long flags;
	struct waitable_queue_entry *pqentry;
	struct vhci_queue_data *pqdata;
	PUNRB punrb;

	TRACE("usbd_mv_ioctl: ++ cmd=%d arg=%lu\n", cmd, arg);

	switch(cmd)
	{
		case 0: /* read unrb */

			if( pdev->reset_in_progress )
			{
				TRACE("usbd_mv_ioctl: reset in progress\n");
				result = -ENODATA;
				break;
			}

			punrb = (PUNRB)((char*)pdev->readbuf+arg);

			TRACE("usbd_mv_ioctl: read unrb\n");

			/* get next queued urb or unrb */
			usbd_wq_lock(&pdev->queue_pending_transfer_unrb,flags);
			pqentry = usbd_wq_remove_head(&pdev->queue_pending_transfer_unrb);
			usbd_wq_unlock(&pdev->queue_pending_transfer_unrb,flags);

			do /*while(0)*/
			{
				if( pqentry == NULL )
				{
					TRACE("usbd_mv_ioctl: no entry\n");
					result = -ENODATA;
					break;
				}

				pqdata = (struct vhci_queue_data *)pqentry->data;

				if( pqdata->purb )
				{
					result = usbd_pack_urb(pqdata->purb, pqdata->unique_id, punrb);

					if( result != 0 ) 
					{
						TRACE_CRITICAL("usbd_mv_ioctl: can not pack urb, result=%d\n", result);
						result = -ENODATA;
						usbd_vhci_device_complete_urb(pqdata->purb, -EINVAL);
						break;
					}

					dump_unrb(punrb);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24)
					spin_lock(&pqdata->purb->lock);
					if( pqdata->purb->status == -EINPROGRESS )
#else
					if( !pqdata->purb->unlinked )
#endif
					{
						/* urb is still pending, put it back into queue */
						usbd_wq_add_tail_locked(&pdev->queue_pending_completion_unrb, pqentry);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24)
						spin_unlock(&pqdata->purb->lock);
#endif

						TRACE("usbd_mv_ioctl: urb copied successfully\n");

						/* do not free this entry, as it was queued again */
						pqentry = NULL;
					} else
					{
						TRACE("usbd_mv_ioctl: urb has been cancelled!\n");
						/* somebody has cancelled the urb while we were messing with it */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24)
						spin_unlock(&pqdata->purb->lock);
#endif
						usbd_vhci_device_complete_urb(pqdata->purb, -1);
						result = -ENODATA;
					}
					break;
				}

				if( pqdata->punrb )
				{
					memcpy(punrb, pqdata->punrb, pqdata->punrb->Header.Size);
					dump_unrb(punrb);
					if(punrb->Header.Function == UNRB_FUNCTION_RESET_PORT) 
					{
						pdev->reset_in_progress = 1;

						/* put unrb into pending queue */
						usbd_wq_add_tail_locked(&pdev->queue_pending_completion_unrb, pqentry);

						pqentry = NULL;
					} else kfree(pqdata->punrb);
					TRACE("usbd_mv_ioctl: unrb copied successfully(2)\n");
					break;
				}

				result = -ENODATA;

			} while(0);

			if( pqentry ) 
				usbd_wq_free_entry(pqentry);

			break;

		case 1: /* write unrb */

			TRACE("usbd_mv_ioctl: write unrb\n");

			punrb = (PUNRB)((char*)pdev->writebuf+arg);

			dump_unrb(punrb);

			usbd_wq_lock(&pdev->queue_pending_completion_unrb,flags);
			pqentry = usbd_wq_get_first(&pdev->queue_pending_completion_unrb);
			while(pqentry)
			{
				pqdata = (struct vhci_queue_data *)pqentry->data;
			
				if( pqdata->unique_id == punrb->Header.UniqueId )
				{
					usbd_wq_remove_entry(pqentry);
					break;
				} 

				pqentry = usbd_wq_get_next(&pdev->queue_pending_completion_unrb, pqentry);
			}
			usbd_wq_unlock(&pdev->queue_pending_completion_unrb,flags);

			if( pqentry )
			{
				pqdata = (struct vhci_queue_data *)pqentry->data;

				if( pqdata->purb )
				{
					int status;
					TRACE("usbd_mv_ioctl: unpacking urb\n");
					status = usbd_unpack_urb(punrb, pqdata->purb);
					usbd_vhci_device_complete_urb(pqdata->purb, status);
				}
				if( pqdata->punrb )
				{
					if(punrb->Header.Function == UNRB_FUNCTION_RESET_PORT)
					{
						TRACE("usbd_mv_ioctl: reset completed\n");
						pdev->reset_in_progress = 0;
						usbd_wq_wake_up(&pdev->queue_pending_transfer_unrb);
					}

					TRACE("usbd_mv_ioctl: freeing unrb\n");
					kfree(pqdata->punrb);
				}

				usbd_wq_free_entry(pqentry);
			} else
			{
				TRACE("usbd_mv_ioctl: urb not found\n");
			}
			break;

		default:
			TRACE("usbd_mv_ioctl: invalid request\n");
			result = -EINVAL;
			break;
	}

	TRACE("usbd_mv_ioctl: -- result=%d\n", result);

	return result;
}

int usbd_mv_mmap(
	void *context,
	struct vm_area_struct *vma)
{
	int result;
	struct vhci_device_descriptor *pdev = context;
	TRACE("usbd_mv_mmap: ++\n");

	// map i/o buffer to user-space
	result = usbd_map_vmem(pdev->iobuf, vma, IO_BUFFER_SIZE*2);

	TRACE("usbd_mv_mmap: -- result=%d\n", result);
	return result;
}


int usbd_mv_open(void *context)
{
	int result = 0;
	struct vhci_device_descriptor *pdev = context;
	unsigned char *iobuf;

	TRACE("usbd_mv_open: ++\n");

	usbd_vhci_device_reference(pdev);

	iobuf = vmalloc(IO_BUFFER_SIZE*2);

	if( iobuf == NULL )
	{
		TRACE("usbd_mv_open: no memory for iobuf\n");
    	result = -ENOMEM;
    } else
	{
		pdev->iobuf = iobuf;
		pdev->readbuf = iobuf+0;
		pdev->writebuf = iobuf+IO_BUFFER_SIZE;
	}

	TRACE("usbd_mv_open: -- result=%d\n", result);
	return result;
}

int usbd_mv_release(void *context)
{
	struct vhci_device_descriptor *pdev = context;
	TRACE("usbd_mv_release\n");

	if( pdev->iobuf ) 
	{
		vfree(pdev->iobuf);
		pdev->iobuf = NULL;
	}

	usbd_vhci_device_dereference(pdev);
	return 0;
}

unsigned int usbd_mv_poll(void *context, struct file *filp, poll_table *wait)
{
	int empty;
	unsigned long flags;
	struct vhci_device_descriptor *pdev = context;

	poll_wait(filp, &pdev->queue_pending_transfer_unrb.waitqueue, wait);

	usbd_wq_lock(&pdev->queue_pending_transfer_unrb, flags);
	empty = usbd_wq_is_empty(&pdev->queue_pending_transfer_unrb);
	usbd_wq_unlock(&pdev->queue_pending_transfer_unrb, flags);

	if( !empty && !pdev->reset_in_progress )
	{
		TRACE("usbd_mv_poll: queue not empty\n");
		return ((POLLOUT | POLLWRNORM) | (POLLIN | POLLRDNORM));
	}

	return (POLLOUT | POLLWRNORM);
}

#endif // _USBD_ENABLE_VHCI_
