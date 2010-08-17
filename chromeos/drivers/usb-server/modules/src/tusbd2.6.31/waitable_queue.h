/*
 *  tusbd/waitable_queue.h
 *
 *  Copyright (C) 2007-2008 IncentivesPro
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef _WAITABLE_QUEUE_H
#define _WAITABLE_QUEUE_H

struct waitable_queue
{
	struct list_head entrylist;	// list of entries
	spinlock_t lock;
	wait_queue_head_t waitqueue;
};

struct waitable_queue_entry
{
	struct list_head entry;
	unsigned long datasize;
	u8 data[1];
};

void usbd_wq_init(struct waitable_queue *pqueue);
void usbd_wq_deinit(struct waitable_queue *pqueue);

struct waitable_queue_entry *usbd_wq_alloc_entry(unsigned long datasize, int mem_flags);
void usbd_wq_free_entry(struct waitable_queue_entry *pentry);

// void usbd_wq_lock(struct waitable_queue *pqueue);
#define usbd_wq_lock(__pqueue, __flags) spin_lock_irqsave(&(__pqueue)->lock,__flags)
// void usbd_wq_unlock(struct waitable_queue *pqueue);
#define usbd_wq_unlock(__pqueue, __flags) spin_unlock_irqrestore(&(__pqueue)->lock,__flags)

#define usbd_wq_add_tail_locked(__pqueue, __pentry) \
		do { \
			unsigned long __flags; \
			usbd_wq_lock((__pqueue), __flags); \
			usbd_wq_add_tail((__pqueue), (__pentry)); \
			usbd_wq_unlock((__pqueue), (__flags)); \
		}while(0)

#define usbd_wq_remove_entry_locked(__pqueue, __pentry) \
		do { \
			unsigned long __flags; \
			usbd_wq_lock((__pqueue), (__flags)); \
			usbd_wq_remove_entry((__pentry)); \
			usbd_wq_unlock((__pqueue), __flags); \
		}while(0)

// the following functions should be synchronized with usbd_wq_lock/usbd_wq_unlock
int usbd_wq_is_empty(struct waitable_queue *pqueue);
int usbd_wq_is_queued(struct waitable_queue_entry *pentry);
void usbd_wq_add_tail(struct waitable_queue *pqueue, struct waitable_queue_entry *pentry);
struct waitable_queue_entry *usbd_wq_remove_head(struct waitable_queue *pqueue);
struct waitable_queue_entry *usbd_wq_get_first(struct waitable_queue *pqueue);
struct waitable_queue_entry *usbd_wq_get_next(struct waitable_queue *pqueue, struct waitable_queue_entry *pentry);
void usbd_wq_remove_entry(struct waitable_queue_entry *pentry);
void usbd_wq_clear(struct waitable_queue *pqueue);
void usbd_wq_wake_up(struct waitable_queue *pqueue);


#endif // _WAITABLE_QUEUE_H
