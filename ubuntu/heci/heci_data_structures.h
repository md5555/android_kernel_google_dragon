/*
 * Part of Intel(R) Manageability Engine Interface Linux driver
 *
 * Copyright (c) 2003 - 2007 Intel Corp.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions, and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce at minimum a disclaimer
 *    substantially similar to the "NO WARRANTY" disclaimer below
 *    ("Disclaimer") and any redistribution must be conditioned upon
 *    including a substantially similar Disclaimer requirement for further
 *    binary redistribution.
 * 3. Neither the names of the above-listed copyright holders nor the names
 *    of any contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * Alternatively, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2 as published by the Free
 * Software Foundation.
 *
 * NO WARRANTY
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDERS OR CONTRIBUTORS BE LIABLE FOR SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGES.
 *
 */

#ifndef _HECI_DATA_STRUCTURES_H_
#define _HECI_DATA_STRUCTURES_H_

#include <linux/version.h>
#include <linux/spinlock.h>
#include <linux/list.h>
#include <linux/pci.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/module.h>
#include <linux/aio.h>
#include <linux/types.h>

/**
 * error code definition
 */
#define     ESUCCESS                     0
#define     ESLOTS_OVERFLOW              1
#define     ECORRUPTED_MESSAGE_HEADER    1000
#define     ECOMPLETE_MESSAGE            1001
#define     FC_MESSAGE_RESERVED_LENGTH           5

/**
 * Number of queue lists used by this driver
 */
#define NUMBER_OF_LISTS        7

#define LEGACY_MTU 4160
#pragma pack(1)


/**
 * HECI HW Section
 */

/* HECI addresses and defines */
#define H_CB_WW    0
#define H_CSR      4
#define ME_CB_RW   8
#define ME_CSR_HA  0xC


/* register bits - H_CSR */

#define H_CBD             0xFF000000
#define H_CBWP            0x00FF0000
#define H_CBRP            0x0000FF00
#define H_RST             0x00000010
#define H_RDY             0x00000008
#define H_IG              0x00000004
#define H_IS              0x00000002
#define H_IE              0x00000001


/* register bits - ME_CSR_HA */
#define ME_CBD_HRA        0xFF000000
#define ME_CBWP_HRA       0x00FF0000
#define ME_CBRP_HRA       0x0000FF00
#define ME_RST_HRA        0x00000010
#define ME_RDY_HRA        0x00000008
#define ME_IG_HRA         0x00000004
#define ME_IS_HRA         0x00000002
#define ME_IE_HRA         0x00000001

/**
 *  heci driver use additional char device for legacy mode
 */
#define  MINORS_COUNT	2

#define  LEGACY_MINOR_NUMBER	0
#define  HECI_MINOR_NUMBER	1
#define  MAX_OPEN_HANDLE_COUNT	253
/**
 * debug kernel print macro define
 */
#define	HECI_INFO(format, arg...)	        printk(KERN_INFO     "%s: " format, THIS_MODULE->name, ## arg)
#define	HECI_ERR(format, arg...)	            printk(KERN_ERR      "%s: " format, THIS_MODULE->name, ## arg)
#define HECI_WARN(format, arg...)            printk(KERN_WARNING  "%s: " format, THIS_MODULE->name, ## arg)


/* Module Parameters */
#define	DEF_PARM(type, name, init, perm, desc)	\
	type name = (init);	\
	MODULE_PARM_DESC(name, desc);	\
	module_param(name, type, perm)

extern int debug;

#define DBG(format, arg...) do {if (debug) \
printk(KERN_ERR "%s: " format , __func__ , ## arg); \
} while (0)

#ifdef HECI_DEBUG
#define assert(expr) do {} while (0)
#else
#define assert(expr) \
    if (!(expr)) {                                   \
    printk("Assertion failed! %s, %s, %s, line=%d\n", \
    #expr, __FILE__, __func__, __LINE__);          \
    }
#endif

/**
 * time to wait event
 */
#define HECI_INTEROP_TIMEOUT    (HZ * 7)

/**
 * watch dog definition
 */
#define HECI_WATCHDOG_DATA_SIZE         16
#define HECI_START_WD_DATA_SIZE         20
#define HECI_WD_PARAMS_SIZE             4

#define HECI_NO_MSG_SENT			0
#define HECI_WD_STATE_INDEPENDENCE_MSG_SENT 	(1 << 0)


#define HECI_WD_HOST_CLIENT_ID          1
#define HECI_LEGACY_HOST_CLIENT_ID      2

#undef FALSE
#undef TRUE
#define TRUE  1
#define FALSE 0

struct guid {
	__u32 data1;
	__u16 data2;
	__u16 data3;
	__u8 data4[8];
};

/* File state */
enum file_state {
	HECI_FILE_INITIALIZING = 0,
	HECI_FILE_CONNECTING,
	HECI_FILE_CONNECTED,
	HECI_FILE_DISCONNECTING,
	HECI_FILE_DISCONNECTED
};

/* HECI states */
enum heci_states{
	HECI_INITIALIZING = 0,
	HECI_ENABLED,
	HECI_RESETING,
	HECI_DISABLED,
	HECI_RECOVERING_FROM_RESET,
	HECI_POWER_DOWN,
	HECI_POWER_UP
};

enum legacy_states {
	HECI_LEGACY_IDLE,
	HECI_LEGACY_WRITING,
	HECI_LEGACY_FLOW_CONTROL,
	HECI_LEGACY_READING,
	HECI_LEGACY_READ_COMPLETE
};

enum heci_file_transaction_states {
	HECI_IDLE,
	HECI_WRITING,
	HECI_WRITE_COMPLETE,
	HECI_FLOW_CONTROL,
	HECI_READING,
	HECI_READ_COMPLETE
};

/* HECI CB */
enum heci_cb_major_types {
	HECI_READ = 0,
	HECI_WRITE,
	HECI_IOCTL,
	HECI_OPEN,
	HECI_CLOSE
};

/* HECI user data struct */
struct heci_message_data {
	__u32 size;
	char *data;
};
#define SECOND_TO_MILLI                 1000
#define SECOND_TO_MICRO                 SECOND_TO_MILLI * 1000
#define SECOND_TO_100NANO               SECOND_TO_MICRO * 10

#define CONNECT_TIMEOUT                 3	/* at least 2 seconds */

#define LEGACY_STALL_TIMER              12	/* seconds */
#define LEGACY_READ_TIMER               15	/* seconds */

struct heci_cb_private {
	struct list_head cb_list;
	enum heci_cb_major_types major_file_operations;
	void *file_private;
	struct heci_message_data request_buffer;
	struct heci_message_data response_buffer;
	unsigned long information;
	unsigned long read_time;
	struct file *file_object;
};

/* Private file struct */
struct heci_file_private {
	struct list_head link;
	struct file *file;
	enum file_state state;
	wait_queue_head_t tx_wait;
	wait_queue_head_t rx_wait;
	wait_queue_head_t wait;
	spinlock_t file_lock;
	spinlock_t read_io_lock;
	spinlock_t write_io_lock;
	int read_pending;
	int status;
	/* ID of client connected */
	__u8 host_client_id;
	__u8 me_client_id;
	__u8 flow_control_credentials;
	__u8 timer_count;
	enum heci_file_transaction_states reading_state;
	enum heci_file_transaction_states writing_state;
	int sm_state;
	struct heci_cb_private *read_cb;
};

struct io_heci_list {
	struct heci_cb_private heci_cb;
	int status;
	struct iamt_heci_device *device_extension;
};

struct heci_driver_version {
	__u8 major;
	__u8 minor;
	__u8 hotfix;
	__u16 build;
};


struct heci_client {
	__u32 max_message_length;
	__u8 protocol_version;
};
/*
 *  HECI BUS Interface Section
 */
struct heci_message_header {
	__u32 me_address:8;
	__u32 host_address:8;
	__u32 length:9;
	__u32 reserved:6;
	__u32 message_complete:1;
};


struct hbm_command {
	__u8 command:7;
	__u8 is_response:1;
};


struct heci_bus_message {
	struct hbm_command command;
	__u8 command_specific_data[];
};

struct hbm_version {
	__u8 minor_version;
	__u8 major_version;
};

struct hbm_host_version_request {
	struct hbm_command command;
	__u8 reserved;
	struct hbm_version host_version;
};

struct hbm_host_version_response {
	struct hbm_command command;
	int host_version_supported;
	struct hbm_version me_max_version;
};

struct hbm_host_stop_request {
	struct hbm_command command;
	__u8 reason;
	__u8 reserved[2];
};

struct hbm_host_stop_response {
	struct hbm_command command;
	__u8 reserved[3];
};

struct hbm_me_stop_request {
	struct hbm_command command;
	__u8 reason;
	__u8 reserved[2];
};

struct hbm_host_enumeration_request {
	struct hbm_command command;
	__u8 reserved[3];
};

struct hbm_host_enumeration_response {
	struct hbm_command command;
	__u8 reserved[3];
	__u8 valid_addresses[32];
};

struct heci_client_properties {
	struct guid protocol_name;
	__u8 protocol_version;
	__u8 max_number_of_connections;
	__u8 fixed_address;
	__u8 single_receive_buffer;
	__u32 max_message_length;
};

struct hbm_host_client_properties_request {
	struct hbm_command command;
	__u8 address;
	__u8 reserved[2];
};


struct hbm_host_client_properties_response {
	struct hbm_command command;
	__u8 address;
	__u8 status;
	__u8 reserved[1];
	struct heci_client_properties client_properties;
};

struct hbm_client_connect_request {
	struct hbm_command command;
	__u8 me_address;
	__u8 host_address;
	__u8 reserved;
};

struct hbm_client_connect_response {
	struct hbm_command command;
	__u8 me_address;
	__u8 host_address;
	__u8 status;
};

struct hbm_client_disconnect_request {
	struct hbm_command command;
	__u8 me_address;
	__u8 host_address;
	__u8 reserved[1];
};

struct hbm_flow_control {
	struct hbm_command command;
	__u8 me_address;
	__u8 host_address;
	__u8 reserved[FC_MESSAGE_RESERVED_LENGTH];
};

struct heci_me_client {
	struct heci_client_properties properteis;
	__u8 client_id;
	__u8 flow_control_credentials;
};

/* private device struct */
struct iamt_heci_device {
	struct pci_dev *pdev;	/* pointer to pci device struct */
	/*
	 * lists of queues
	 */
	struct io_heci_list *io_list_array[NUMBER_OF_LISTS];	/* array of pointers to  aio lists */
	struct io_heci_list read_list;	/* driver  read queue */
	struct io_heci_list write_list;	/* driver write queue */
	struct io_heci_list write_waiting_list;	/* driver write waiting queue */
	struct io_heci_list control_write_list;	/* driver managed write IOCTL list */
	struct io_heci_list control_read_list;	/* driver managed read IOCTL list */
	struct io_heci_list pthi_cmd_list;	/* driver managed PTHI list for cmd waiting */
	struct io_heci_list pthi_read_complete_list;	/* driver managed PTHI list for  read completed pthi command data */
	/*
	 * list of files
	 */
	struct list_head file_list;
	/*
	 * memory of device
	 */
	unsigned int mem_base;
	unsigned int mem_length;
	char *mem_addr;
	/*
	 * lock for the device
	 */
	spinlock_t device_lock;
	spinlock_t extra_lock;
	/*
	 * intterupts
	 */
	int irq;
	struct work_struct work;
	int received_message;

	struct timer_list timer;
	struct timer_list wd_timer;
	/*
	 * hw states of host and fw(ME)
	 */
	__u32 host_hw_state;
	__u32 me_hw_state;
	/*
	 * waiting queue for receive message from FW
	 */
	wait_queue_head_t wait_received_message;
	wait_queue_head_t wait_stop_wd;
	/*
	 * heci device  states
	 */
	enum heci_states heci_state;
	int stop;
     /**
      * virtual void GetParam(const char* UserParam);
      * read write messages to/from heci fw
      */
	__u32 extra_write_index;
	__u32 read_message_buffer[128];	/* used for control messages */
	__u32 write_message_buffer[128];	/* used for control messages */
	__u32 extra_message_buffer[8];	/* for control responses    */
	__u32 read_message_header;

	struct hbm_version version;

	int host_buffer_is_empty;
	struct heci_file_private wd_file_extension;
	struct heci_me_client *me_clients;	/* Note: memory has to be allocated */
	__u8 heci_me_clients[32];	/* list of existing clients */
	__u8 num_heci_me_clients;
	__u8 heci_host_clients[32];	/* list of existing clients */
	__u8 current_host_client_id;

	int wd_pending;
	int wd_stoped;
	__u16 wd_timeout;	/* seconds ((wd_data[1] << 8) + wd_data[0]) */
	unsigned char wd_data[HECI_START_WD_DATA_SIZE];


	__u16 wd_due_counter;
	int asf_mode;
	int	wd_bypass;	/* if true,don't refresh watchdog ME client */

	/* maybe this is not required */
	struct file *legacy_file_object;
	struct heci_file_private legacy_file_extension;
	int legacy_ioctl;
	int legacy_canceled;
	__u32 legacy_timer;
	__u32 legacy_stall_timer;
	unsigned char legacy_message_buffer[LEGACY_MTU];
	__u32 legacy_message_buffer_size;
	__u32 legacy_message_buffer_index;
	int legacy_flow_control_pending;
	enum legacy_states legacy_state;

	struct heci_cb_private *legacy_current_cb;
	__u8 write_hang;
	int need_reset;
	long open_handle_count;

};

/**
 * read_heci_register - Read a byte from the heci device
 * @device: the device structure
 * @offset: offset from which to read the data
 *
 * Return:
 * the byte read.
 */
__u32 read_heci_register(struct iamt_heci_device * device,
			    unsigned long offset);

/**
 * write_heci_register - Write  4 bytes to the heci device
 * @device: the device structure
 * @offset: offset from which to write the data
 *
 * @value: the byte to write
 */
void write_heci_register(struct iamt_heci_device * device, unsigned long offset,
			 __u32 value);

#endif				/* _HECI_DATA_STRUCTURES_H_ */
