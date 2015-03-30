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

/* Messages */
#define MRQ_PING		0
#define MRQ_QUERY_TAG		1
#define MRQ_DO_IDLE		2
#define MRQ_TOLERATE_IDLE	3
#define MRQ_MODULE_LOAD		4
#define MRQ_MODULE_UNLOAD	5
#define MRQ_SWITCH_CLUSTER	6
#define MRQ_TRACE_MODIFY	7
#define MRQ_WRITE_TRACE		8
#define MRQ_THREADED_PING	9
#define MRQ_CPUIDLE_USAGE	10
#define MRQ_MODULE_MAIL		11
#define MRQ_SCX_ENABLE		12
#define MRQ_INIT_NR_CPUS	13
#define MRQ_BPMPIDLE_USAGE	14
#define MRQ_HEAP_USAGE		15
#define MRQ_SCLK_SKIP_SET_RATE	16
#define MRQ_ENABLE_SUSPEND	17
#define MRQ_PASR_MASK		18
#define MRQ_DEBUGFS		19

#define NR_MRQS			32

/* Tegra PM states as known to BPMP */
#define TEGRA_BPMP_PM_CC1	9
#define TEGRA_BPMP_PM_CC4	12
#define TEGRA_BPMP_PM_CC6	14
#define TEGRA_BPMP_PM_CC7	15
#define TEGRA_BPMP_PM_SC1	17
#define TEGRA_BPMP_PM_SC2	18
#define TEGRA_BPMP_PM_SC3	19
#define TEGRA_BPMP_PM_SC4	20
#define TEGRA_BPMP_PM_SC7	23

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


int tegra_bpmp_send(int mrq, void *data, int sz);
int tegra_bpmp_send_receive_atomic(int mrq, void *ob_data, int ob_sz,
				   void *ib_data, int ib_sz);
int tegra_bpmp_send_receive(int mrq, void *ob_data, int ob_sz,
			    void *ib_data, int ib_sz);

#endif /* __SOC_TEGRA_BPMP_H__ */
