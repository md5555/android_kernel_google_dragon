/*
 *  tusbd/fifo.h
 *
 *  Copyright (C) 2007-2008 IncentivesPro
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef _FIFO_H
#define _FIFO_H

struct fifo_descriptor
{
	struct list_head entrylist;	// list of entries
	spinlock_t lock;
	int num_entries;			// number of entries in fifo /*TODO: atomic operations*/
	wait_queue_head_t waitqueue;
};

struct fifo_entry
{
	struct list_head entry;
	int size;		// size in bytes of the event NOT including terminating zero
	int read;		// number of bytes read
	char ispointer:1;
	char isusermodepointer:1;
	union
	{
		u8 *pdata;
		u8 data[1];
	};
};

struct fifo_descriptor *usbd_fifo_create(void);
void usbd_fifo_destroy(struct fifo_descriptor *plogdesc);

void usbd_fifo_clear(struct fifo_descriptor *plogdesc);
int usbd_fifo_add(struct fifo_descriptor *pfifodesc, const void *buf, int bufsize, int user_mode_buffer);
// usbd_fifo_add_pointer does not copy the buffer, it queues a pointer to the buffer
int usbd_fifo_add_pointer(struct fifo_descriptor *pfifodesc, const void *buf, int bufsize, int user_mode_buffer);
int usbd_fifo_get(struct fifo_descriptor *plogdesc, void *buf, int bufsize, int user_mode_buffer);

#endif // _FIFO_H
