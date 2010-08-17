/*
 *  tusbd/fifo.c
 *
 *  Copyright (C) 2007-2008 IncentivesPro
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include "usbd.h"

struct fifo_descriptor *usbd_fifo_create(void)
{
	struct fifo_descriptor *pfifodesc;

   	pfifodesc = kmalloc(sizeof(*pfifodesc), GFP_KERNEL);

   	if( pfifodesc)
   	{
	  	memset(pfifodesc, 0, sizeof(*pfifodesc));

  		INIT_LIST_HEAD(&pfifodesc->entrylist);
		spin_lock_init(&pfifodesc->lock);
		init_waitqueue_head(&pfifodesc->waitqueue);

		return pfifodesc;
 	}

	return NULL;
}

void usbd_fifo_destroy(struct fifo_descriptor *pfifodesc)
{
	if( !pfifodesc ) return;

	usbd_fifo_clear(pfifodesc);

	kfree(pfifodesc);
}

void usbd_fifo_clear(struct fifo_descriptor *pfifodesc)
{
	spin_lock(&pfifodesc->lock);
	while( !list_empty(&pfifodesc->entrylist) )
	{
		struct list_head *entry = pfifodesc->entrylist.next;
		list_del(entry);
		kfree(entry);
	}
	pfifodesc->num_entries = 0;

	spin_unlock(&pfifodesc->lock);
}

int usbd_fifo_get(struct fifo_descriptor *pfifodesc, void *buf, int bufsize, int user_mode_buffer)
{
	int result = 0;

	if( !pfifodesc ) return -EINVAL;
	if( !buf ) return -EINVAL;
	if( !bufsize ) return -EINVAL;

	spin_lock(&pfifodesc->lock);

	if( !list_empty(&pfifodesc->entrylist) )
	{
		struct fifo_entry	*pfifoentry;

		pfifoentry = (struct fifo_entry*)pfifodesc->entrylist.next;

		// calc the nuber of bytes to copy
		result = MIN_OF(bufsize, pfifoentry->size - pfifoentry->read);

		if( user_mode_buffer )
		{
			if( copy_to_user(buf, pfifoentry->data+pfifoentry->read, result) != 0 )
			{
				TRACE("usbd_fifo_get: cannot copy to user buffer\n");
			}
		} else
		{
			memcpy(buf, pfifoentry->data+pfifoentry->read, result);
		}

		pfifoentry->read += result;

		// is all event copied?
		if( pfifoentry->read == pfifoentry->size )
		{
			list_del(&pfifoentry->entry);
			kfree(pfifoentry);
			pfifodesc->num_entries--;
		}
	}

	spin_unlock(&pfifodesc->lock);

	return result;
}

int usbd_fifo_add(struct fifo_descriptor *pfifodesc, const void *buf, int bufsize, int user_mode_buffer)
{
	struct fifo_entry *pfifoentry;

	if( !pfifodesc ) return -EINVAL;
	if( !buf ) return -EINVAL;

	if( !bufsize ) return -EINVAL;

	if( in_atomic() )
	{
	   	pfifoentry = kmalloc(sizeof(*pfifoentry)+bufsize-1, GFP_ATOMIC); /*TODO: this is wrong!*/
	} else
	{
	   	pfifoentry = kmalloc(sizeof(*pfifoentry)+bufsize-1, GFP_KERNEL);
	}

   	if( pfifoentry )
   	{
		if( user_mode_buffer )
		{
			if( copy_from_user(pfifoentry->data, buf, bufsize) != 0 )
			{
				TRACE("usbd_fifo_get: cannot copy to user buffer\n");
			}
		} else
		{
			memcpy(pfifoentry->data, buf, bufsize);
		}

   		pfifoentry->read = 0;
   		pfifoentry->size = bufsize;

		spin_lock(&pfifodesc->lock);
		list_add_tail(&pfifoentry->entry, &pfifodesc->entrylist);
		pfifodesc->num_entries++;
		spin_unlock(&pfifodesc->lock);

		wake_up(&pfifodesc->waitqueue);

		return 0;
   	}

   	return -ENOMEM;
}

int usbd_fifo_add_pointer(struct fifo_descriptor *pfifodesc, const void *buf, int bufsize, int user_mode_buffer)
{
	struct fifo_entry *pfifoentry;

	if( !pfifodesc ) return -EINVAL;
	if( !buf ) return -EINVAL;

	if( !bufsize ) return -EINVAL;

	if( in_atomic() )
	{
	   	pfifoentry = kmalloc(sizeof(*pfifoentry)+bufsize-1, GFP_ATOMIC); /*TODO: this is wrong!*/
	} else
	{
	   	pfifoentry = kmalloc(sizeof(*pfifoentry)+bufsize-1, GFP_KERNEL);
	}

   	if( pfifoentry )
   	{
		if( user_mode_buffer )
		{
			if( copy_from_user(pfifoentry->data, buf, bufsize) != 0 )
			{
				TRACE("usbd_fifo_get: cannot copy to user buffer\n");
			}
		} else
		{
			memcpy(pfifoentry->data, buf, bufsize);
		}

   		pfifoentry->read = 0;
   		pfifoentry->size = bufsize;

		spin_lock(&pfifodesc->lock);
		list_add_tail(&pfifoentry->entry, &pfifodesc->entrylist);
		pfifodesc->num_entries++;
		spin_unlock(&pfifodesc->lock);

		wake_up(&pfifodesc->waitqueue);

		return 0;
   	}

   	return -ENOMEM;
}
