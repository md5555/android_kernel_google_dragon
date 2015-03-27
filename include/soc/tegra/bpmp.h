/*
 * Copyright (c) 2015, NVIDIA Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#ifndef __SOC_TEGRA_BPMP_H__
#define __SOC_TEGRA_BPMP_H__

#define BPMP_NR_CHANNEL		12
#define BPMP_NR_THREAD_CHAN	4
#define BPMP_NR_INBOX_CHAN	4

#define THREAD_CHAN_OFFSET	4
#define INBOX_CHAN_OFFSET	8

#define BPMP_MSG_SIZE		128
#define BPMP_MSG_DATA_SIZE	120

/* Message Types */
#define BPMP_MSG_TYPE_REQ	(1 << 0)
#define BPMP_MSG_TYPE_RESP	(1 << 1)
#define BPMP_MSG_TYPE_ATOMIC	(1 << 2)

/* Message Flags */
#define BPMP_MSG_DO_ACK		(1 << 0)
#define BPMP_MSG_RING_DOORBELL	(1 << 1)

struct tegra_bpmp_mbox_msg {
	s32 code;
	s32 flags;
	u32 type;
	int size;
	void *data;
};

struct mb_data {
	s32 code;
	s32 flags;
	u8 data[BPMP_MSG_DATA_SIZE];
} __packed;

#endif /* __SOC_TEGRA_BPMP_H__ */
