/*
 *  tusbd/urb_chain.h
 *
 *  Copyright (C) 2007-2008 IncentivesPro
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef _URB_CHAIN_H
#define _URB_CHAIN_H

#define MAX_CHAIN_LENGTH 16

struct urb_chain;

extern int usbd_uc_maxbuffersize;

typedef void (*urb_chain_complete_t)(struct urb_chain *);

struct urb_chain
{
	spinlock_t lock;
	struct urb* urbs[MAX_CHAIN_LENGTH];
	struct usb_device *pdev;
	int pipe;
	int is_ehci;
	int short_ok;
	int status;
	size_t bytes;
	unsigned char count;
	atomic_t iocount;
	void *context;
	urb_chain_complete_t complete;
};

struct buf_chain
{
	void *buffers[MAX_CHAIN_LENGTH]; // pointer to buffers
	size_t buffersize[MAX_CHAIN_LENGTH]; // size of each buffer
	size_t datasize[MAX_CHAIN_LENGTH]; // number of bytes in buffer
	char isopackets[MAX_CHAIN_LENGTH]; // number of iso packets in each buffer
	unsigned char count;	
};

int usbd_uc_init(
	struct urb_chain *pchain, 
	int pipe,
	int interval,
	int short_ok,
	int is_ehci,
	struct usb_device *pdev,
	struct buf_chain *pbufchain, 
	void *context,
	urb_chain_complete_t complete);

int usbd_uc_clean(struct urb_chain *pchain);
int usbd_uc_submit(struct urb_chain *pchain);
int usbd_uc_get(struct urb_chain *pchain);
int usbd_uc_put(struct urb_chain *pchain);
int usbd_uc_cancel(struct urb_chain *pchain);
int usbd_uc_index_of(struct urb_chain *pchain, struct urb* purb);

int usbd_bc_alloc(struct buf_chain *pchain, size_t length, int multiplicity);
int usbd_bc_alloc_iso(struct buf_chain *pchain, size_t length, UNRB_ISOCH_PACKET_DESCRIPTOR *isopackets, uint32 nisopackets);
int usbd_bc_copy_from_user(struct buf_chain *pchain, const void *buf, size_t length);
int usbd_bc_copy_to_user(struct buf_chain *pchain, void *buf, size_t length);
int usbd_bc_free(struct buf_chain *pchain);
size_t usbd_bc_unpack_iso(struct buf_chain *pchain, UNRB_ISOCH_PACKET_DESCRIPTOR *isopackets, uint32 nisopackets);
size_t usbd_bc_pack_iso(struct buf_chain *pchain, UNRB_ISOCH_PACKET_DESCRIPTOR *isopackets, uint32 nisopackets);

//#define usbd_uc_for_each_urb(uc, index, container) for(index=0,container=uc.urbs[0];index<uc.count;index++, container=uc.urbs[index])



#endif // _URB_CHAIN_H
