/*
 *  tusbd/waitable_queue.c
 *
 *  Copyright (C) 2007-2008 IncentivesPro
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include "usbd.h"

void usbd_wq_init(struct waitable_queue *pqueue)
{
	if( !pqueue ) return;

  	memset(pqueue, 0, sizeof(*pqueue));

	INIT_LIST_HEAD(&pqueue->entrylist);
	spin_lock_init(&pqueue->lock);
	init_waitqueue_head(&pqueue->waitqueue);
}

void usbd_wq_deinit(struct waitable_queue *pqueue)
{
	unsigned long flags;

	if( !pqueue ) return;

	usbd_wq_lock(pqueue, flags);
	usbd_wq_clear(pqueue);
	usbd_wq_unlock(pqueue, flags);
}

struct waitable_queue_entry *usbd_wq_alloc_entry(unsigned long datasize, int mem_flags)
{
	struct waitable_queue_entry *pentry;

	if( datasize <= 0 ) return NULL;

   	pentry = kmalloc(sizeof(*pentry)+datasize-1, mem_flags);

   	if( pentry )
   	{
  		INIT_LIST_HEAD(&pentry->entry);
		pentry->datasize = datasize;
		return pentry;
 	}

	return NULL;
}

void usbd_wq_free_entry(struct waitable_queue_entry *pentry)
{
	if( pentry ) kfree(pentry);
}

int usbd_wq_is_empty(struct waitable_queue *pqueue)
{
	return list_empty(&pqueue->entrylist);
}

void usbd_wq_add_tail(struct waitable_queue *pqueue, struct waitable_queue_entry *pentry)
{
	if( !pentry || !pqueue ) return;

	list_add_tail(&pentry->entry, &pqueue->entrylist);

	wake_up(&pqueue->waitqueue);
}

struct waitable_queue_entry *usbd_wq_remove_head(struct waitable_queue *pqueue)
{
	struct waitable_queue_entry *result = NULL;

	if( !pqueue ) return NULL;

	if( !list_empty(&pqueue->entrylist) )
	{
		struct list_head *entry = pqueue->entrylist.next;

		list_del(entry);

		result = (struct waitable_queue_entry*)entry;
	}

	return result;
}

struct waitable_queue_entry *usbd_wq_get_first(struct waitable_queue *pqueue)
{
	if( !pqueue ) return NULL;

	return (struct waitable_queue_entry *)(list_empty(&pqueue->entrylist) ? NULL : pqueue->entrylist.next);
}

struct waitable_queue_entry *usbd_wq_get_next(struct waitable_queue *pqueue, struct waitable_queue_entry *pentry)
{
	if( !pqueue || !pentry ) return NULL;
	return (struct waitable_queue_entry *)((pentry->entry.next == &pqueue->entrylist ) ? NULL : pentry->entry.next);
}

void usbd_wq_remove_entry(struct waitable_queue_entry *pentry)
{
	if( !pentry ) return;
	if( pentry->entry.next == &pentry->entry || pentry->entry.prev == &pentry->entry ) return;
	if( pentry->entry.next == LIST_POISON1 || pentry->entry.prev == LIST_POISON2 ) return;

	list_del(&pentry->entry);
}

void usbd_wq_clear(struct waitable_queue *pqueue)
{
	while( !list_empty(&pqueue->entrylist) )
	{
		struct list_head *pentry = pqueue->entrylist.next;
		list_del(pentry);
		kfree(pentry);
	}
}

void usbd_wq_wake_up(struct waitable_queue *pqueue)
{
	wake_up(&pqueue->waitqueue);
}
