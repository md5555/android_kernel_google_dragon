/*      $Id: $      */

/*
 * i2c IR lirc plugin for Hauppauge PVR 150 card
 *
 * Copyright (c) 2000 Gerd Knorr <kraxel@goldbach.in-berlin.de>
 * modified for PixelView (BT878P+W/FM) by
 *      Michal Kochanowicz <mkochano@pld.org.pl>
 *      Christoph Bartelmus <lirc@bartelmus.de>
 * modified for KNC ONE TV Station/Anubis Typhoon TView Tuner by
 *      Ulrich Mueller <ulrich.mueller42@web.de>
 * modified for Asus TV-Box and Creative/VisionTek BreakOut-Box by
 *      Stefan Jahn <stefan@lkcc.org>
 * modified for inclusion into kernel sources by
 *      Jerome Brock <jbrock@users.sourceforge.net>
 * modified for Leadtek Winfast PVR2000 by
 *      Thomas Reitmayr (treitmayr@yahoo.com)
 * modified for Hauppauge PVR-150 IR TX device by
 *      Mark Weaver <mark@npsl.co.uk>
 *
 * parts are cut&pasted from the lirc_pvr150.c driver
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */


#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <linux/version.h>
#include <linux/module.h>
#include <linux/kmod.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/i2c-algo-bit.h>
#include <linux/firmware.h>
#include <linux/vmalloc.h>

#include <linux/semaphore.h>

#include "kcompat.h"
#include "lirc_dev/lirc_dev.h"
#include "lirc.h"

/* We need to be able to reset the crappy IR chip by talking to the ivtv driver */
struct ivtv;
void ivtv_reset_ir_gpio(struct ivtv *itv);

struct IR 
{
	struct lirc_plugin l;
	
	/* Device info */
	struct semaphore   lock;
	int		   open;
	
	/* RX device */
	struct i2c_client  c_rx;
	
	/* RX device buffer & lock */
	struct lirc_buffer buf;
	struct semaphore   buf_sem;
	
	/* RX polling thread data */
	struct completion  *t_notify;
	struct completion  *t_notify2;
	int                shutdown;
	int                tpid;
	
	/* RX read data */
	unsigned char      b[3];

	/* TX device */
	struct i2c_client  c_tx;
	int		   need_boot;
	
	/* # devices, for shutdown */
	int                devs;
};

/* Minor -> data mapping */
static struct IR *ir_devices[MAX_IRCTL_DEVICES];

/* Block size for haup PVR-150 IR transmitter */
#define TX_BLOCK_SIZE	99

/* Hauppuage IR transmitter data */
typedef struct TX_DATA_s
{
	/* Boot block */
	unsigned char *boot_data;
	
	/* Start of binary data block */
	unsigned char *datap;
	
	/* End of binary data block */
	unsigned char *endp;
	
	/* Number of installed codesets */
	unsigned int num_code_sets;
	
	/* Pointers to codesets */
	unsigned char **code_sets;

	/* Global fixed data template */
	int fixed[TX_BLOCK_SIZE];
} TX_DATA;

static TX_DATA *tx_data;
struct semaphore tx_data_lock;

/* ----------------------------------------------------------------------- */

#define DEVICE_NAME "lirc_pvr150"

/* ----------------------------------------------------------------------- */
/* insmod parameters                                                       */

static int debug       = 0;    /* debug output */
static int disable_rx  = 0;    /* disable RX device */
static int disable_tx  = 0;    /* disable TX device */
static int minor       = -1;   /* minor number */

#define dprintk(fmt, args...)                                           \
	do{                                                             \
		if(debug) printk(KERN_DEBUG DEVICE_NAME ": " fmt,       \
				 ## args);                              \
	}while(0)

/* ----------------------------------------------------------------------- */

static int add_to_buf(struct IR *ir)
{
	__u16 code;
	unsigned char codes[2];
	unsigned char keybuf[6];
	int got_data = 0;
	int ret;
	int failures = 0;
	unsigned char sendbuf[1] = { 0 };

	if (lirc_buffer_full(&ir->buf)) {
		dprintk("buffer overflow\n");
		return -EOVERFLOW;
	}

	/* service the device as long as it is returning
	 * data and we have space
	 */
	do
	{
		/* Lock i2c bus for the duration.  RX/TX chips interfere so
		   this is worth it
		 */
		down(&ir->lock);
		
		/* Send random "poll command" (?)  Windows driver does this 
		   and it  is a good point to detect chip failure.
		 */
		ret = i2c_master_send(&ir->c_rx, sendbuf, 1);
		if (ret != 1)
		{
			printk(KERN_ERR
				"lirc_pvr150: i2c_master_send failed with %d\n", 
				ret);
			if (failures >= 3) {
				up(&ir->lock);
				printk(KERN_ERR "lirc_pvr150: unable to read "
				                "from the IR chip after 3 "
				                "resets, giving up\n");
				return ret;
			}

			/* Looks like the chip crashed, reset it */
			printk(KERN_ERR "lirc_pvr150: polling the IR receiver "
			                "chip failed, trying reset\n");
			
			ivtv_reset_ir_gpio(i2c_get_adapdata(ir->c_rx.adapter));
			set_current_state(TASK_UNINTERRUPTIBLE);
			schedule_timeout((100 * HZ + 999) / 1000);
			ir->need_boot = 1;
			
			++failures;
			up(&ir->lock);
			continue;
		}
		
		ret = i2c_master_recv(&ir->c_rx, keybuf, sizeof(keybuf));
		up(&ir->lock);
		if (ret != sizeof(keybuf))
		{
			printk(KERN_ERR
				"lirc_pvr150: i2c_master_recv failed with %d"
				" -- keeping last read buffer\n", ret);
		}
		else
		{
			ir->b[0] = keybuf[3];
			ir->b[1] = keybuf[4];
			ir->b[2] = keybuf[5];
			dprintk("key (0x%02x/0x%02x)\n", ir->b[0], ir->b[1]);
		}
	
		/* key pressed ? */
		if ((ir->b[0] & 0x80) == 0)
			return got_data ? 0 : -ENODATA;
		
		/* look what we have */
		code = (((__u16)ir->b[0]&0x7f)<<6) | (ir->b[1]>>2);
		
		codes[0] = (code >> 8) & 0xff;
		codes[1] = code & 0xff;
	
		/* return it */
		lirc_buffer_write_1( &ir->buf, codes );
		++got_data;
	} while (!lirc_buffer_full(&ir->buf));
	return 0;
}

/* Main function of the polling thread -- from lirc_dev.
 * We don't fit the LIRC model at all anymore.  This is horrible, but
 * basically we have a single RX/TX device with a nasty failure mode
 * that needs to be accounted for across the pair.  lirc lets us provide
 * fops, but prevents us from using the internal polling, etc. if we do
 * so.  Hence the replication.  Might be neater to extend the LIRC model
 * to account for this but I'd think it's a very special case of seriously
 * messed up hardware.
 */
static int lirc_thread(void *arg)
{
	struct IR *ir = arg;
	
	/* This thread doesn't need any user-level access,
	 * so get rid of all our resources
	 */
	daemonize("lirc_pvr150");
	
	if (ir->t_notify != NULL) {
		complete(ir->t_notify);
	}
	
	dprintk("poll thread started\n");
	
	do {
		if (ir->open) {
			set_current_state(TASK_INTERRUPTIBLE);

			/* This is ~113*2 + 24 + jitter (2*repeat gap + 
			   code length).  We use this interval as the chip 
			   resets every time you poll it (bad!).  This is 
			   therefore just sufficient to catch all of the
			   button presses.  It makes the remote much more
			   responsive.  You can see the difference by
			   running irw and holding down a button.  With
			   100ms, the old polling interval, you'll notice
			   breaks in the repeat sequence corresponding to
			   lost keypresses.
			*/
			schedule_timeout((260 * HZ) / 1000);
			if (ir->shutdown) {
				break;
			}
			if (!add_to_buf(ir)) {
				wake_up_interruptible(&ir->buf.wait_poll);
			}
		} else {
			/* if device not opened so we can sleep half a second */
			set_current_state(TASK_INTERRUPTIBLE);
			schedule_timeout(HZ/2);
		}
	} while (!ir->shutdown);
	
	if (ir->t_notify2 != NULL) {
		wait_for_completion(ir->t_notify2);
	}

	ir->tpid = -1;
	if (ir->t_notify != NULL) {
		complete(ir->t_notify);
	}
	
	dprintk("poll thread ended\n");
	return 0;
}

static int set_use_inc(void* data)
{
	struct IR *ir = data;

	if (ir->l.owner == NULL || try_module_get(ir->l.owner) == 0) {
		return -ENODEV;
	}

	/* lock bttv in memory while /dev/lirc is in use  */
	/* this is completely broken code. lirc_unregister_plugin()
	   must be possible even when the device is open */
	if (ir->c_rx.addr) i2c_use_client(&ir->c_rx);
	if (ir->c_tx.addr) i2c_use_client(&ir->c_tx);
	MOD_INC_USE_COUNT;

	return 0;
}

static void set_use_dec(void* data)
{
	struct IR *ir = data;

	if (ir->c_rx.addr) i2c_release_client(&ir->c_rx);
	if (ir->c_tx.addr) i2c_release_client(&ir->c_tx);
	MOD_DEC_USE_COUNT;
	if (ir->l.owner != NULL)
		module_put(ir->l.owner);
}

/* safe read of a uint32 (always network byte order) */
static __inline int read_uint32(unsigned char **data, 
				     unsigned char *endp, unsigned int *val)
{
	if (*data + 4 > endp)
		return 0;
	*val = ((*data)[0] << 24) | ((*data)[1] << 16) | 
	       ((*data)[2] << 8) | (*data)[3];
	*data += 4;
	return 1;
}

/* safe read of a uint8 */
static __inline int read_uint8(unsigned char **data, 
				    unsigned char *endp, unsigned char *val)
{
	if (*data + 1 > endp)
		return 0;
	*val = *((*data)++);
	return 1;
}

/* safe skipping of N bytes */
static __inline int skip(unsigned char **data, 
			      unsigned char *endp, unsigned int distance)
{
	if (*data + distance > endp)
		return 0;
	*data += distance;
	return 1;
}

/* decompress key data into the given buffer */
static int get_key_data(unsigned char *buf, 
			     unsigned int codeset, unsigned int key)
{
	unsigned char *data, *endp, *diffs, *key_block;
	unsigned char keys, ndiffs, id;
	unsigned int base, lim, pos, i;

	/* Binary search for the codeset */	
	for (base = 0, lim = tx_data->num_code_sets; lim; lim >>= 1) {
		pos = base + (lim >> 1);
		data = tx_data->code_sets[pos];
		
		if (!read_uint32(&data, tx_data->endp, &i))
			goto corrupt;
		
		if (i == codeset)
			break;
		else if (codeset > i) {
			base = pos + 1;
			--lim;
		}
	}
	/* Not found? */
	if (!lim)
		return -EPROTO;

	/* Set end of data block */
	endp = pos < tx_data->num_code_sets - 1 ?
		tx_data->code_sets[pos + 1] : tx_data->endp;

	/* Read the block header */
	if (!read_uint8(&data, endp, &keys) || 
	    !read_uint8(&data, endp, &ndiffs) || 
	    ndiffs > TX_BLOCK_SIZE || keys == 0)
	    	goto corrupt;

	/* Save diffs & skip */
	diffs = data;
	if (!skip(&data, endp, ndiffs))
		goto corrupt;
		
	/* Read the id of the first key */
	if (!read_uint8(&data, endp, &id))
		goto corrupt;

	/* Unpack the first key's data */
	for (i = 0; i < TX_BLOCK_SIZE; ++i)
	{
		if (tx_data->fixed[i] == -1)
		{
			if (!read_uint8(&data, endp, &buf[i]))
				goto corrupt;
		}
		else
		{
			buf[i] = (unsigned char)tx_data->fixed[i];
		}
	}
	
	/* Early out key found/not found */
	if (key == id)
		return 0;
	if (keys == 1)
		return -EPROTO;
		
	/* Sanity check */
	key_block = data;
	if (!skip(&data, endp, (keys - 1) * (ndiffs + 1)))
		goto corrupt;
		
	/* Binary search for the key */
	for (base = 0, lim = keys - 1; lim; lim >>= 1) {
		/* Seek to block */
		unsigned char *key_data;
		pos = base + (lim >> 1);
		key_data = key_block + (ndiffs + 1) * pos;

		if (*key_data == key) {
			/* skip key id */
			++key_data;
			
			/* found, so unpack the diffs */
			for (i = 0; i < ndiffs; ++i) {
				unsigned char val;
				if (!read_uint8(&key_data, endp, &val) ||
				    diffs[i] >= TX_BLOCK_SIZE)
					goto corrupt;
				buf[diffs[i]] = val;
			}
				
			return 0;
		} else if (key > *key_data) {
			base = pos + 1;
			--lim;
		}
	}
	/* Key not found */
	return -EPROTO;
	
corrupt:
	printk(KERN_ERR "lirc_pvr150: firmware is corrupt\n");
	return -EFAULT;
}

/* send a block of data to the IR TX device of the PVR-150 */
static int send_data_block(struct IR *ir, unsigned char *data_block)
{
	int i, j, ret;
	unsigned char buf[5];
	
	for (i = 0; i < TX_BLOCK_SIZE; )
	{
		int tosend = TX_BLOCK_SIZE - i;
		if (tosend > 4)
			tosend = 4;
		buf[0] = (unsigned char)(i + 1);
		for (j = 0; j < tosend; ++j)
			buf[1 + j] = data_block[i + j];
		dprintk("%02x %02x %02x %02x %02x",
			buf[0],buf[1],buf[2],buf[3],buf[4]);
		ret = i2c_master_send(&ir->c_tx, buf, tosend + 1);
		if (ret != tosend + 1) {
			printk(KERN_ERR
				"lirc_pvr150: i2c_master_send failed with %d\n", 
				ret);
			return ret < 0 ? ret : -EFAULT;
		}
		i += tosend;
	}
	return 0;
}

/* send boot data to the IR TX device of the PVR-150 */
static int send_boot_data(struct IR *ir)
{
	int ret;
	unsigned char buf[4];

	/* send the boot block */	
	ret = send_data_block(ir, tx_data->boot_data);
	if (ret != 0)
		return ret;

	/* kick it off? */
	buf[0] = 0x00;
	buf[1] = 0x20;
	ret = i2c_master_send(&ir->c_tx, buf, 2);
	if (ret != 2) {
		printk(KERN_ERR
			"lirc_pvr150: i2c_master_send failed with %d\n", ret);
		return ret < 0 ? ret : -EFAULT;
	}
	ret = i2c_master_send(&ir->c_tx, buf, 1);
	if (ret != 1) {
		printk(KERN_ERR
			"lirc_pvr150: i2c_master_send failed with %d\n", ret);
		return ret < 0 ? ret : -EFAULT;
	}

	/* Here comes the firmware version... (hopefully) */
	ret = i2c_master_recv(&ir->c_tx, buf, 4);
	if (ret != 4) {
		printk(KERN_ERR
			"lirc_pvr150: i2c_master_recv failed with %d\n", ret);
		return 0;
	}
	if (buf[0] != 0x80) {
		printk(KERN_ERR
			"lirc_pvr150: unexpected IR TX response: %02x\n", 
			buf[0]);
		return 0;
	}
	printk(KERN_INFO
		"lirc_pvr150: Hauppauge PVR-150 IR blaster: "
		"firmware version %d.%d.%d\n",
		buf[1], buf[2], buf[3]);
		
	return 0;
}

/* unload "firmware", lock held */
static void fw_unload_locked(void)
{
	if (tx_data) {
		if (tx_data->code_sets)
			vfree(tx_data->code_sets);

		if (tx_data->datap)
			vfree(tx_data->datap);

		vfree(tx_data);
		tx_data = NULL;
		dprintk("successfully unloaded PVR-150 IR "
			"blaster firmware\n");
	}
}

/* unload "firmware" for the IR TX device of the PVR-150 */
static void fw_unload(void)
{
	down(&tx_data_lock);
	fw_unload_locked();
	up(&tx_data_lock);
}

/* load "firmware" for the IR TX device of the PVR-150 */
static int fw_load(struct IR *ir)
{
	int ret;
	unsigned int i;
	unsigned char *data, version, num_global_fixed;
	const struct firmware *fw_entry = NULL;
	
	/* Already loaded? */
	down(&tx_data_lock);
	if (tx_data) {
		ret = 0;
		goto out;
	}

	/* Request codeset data file */
	ret = request_firmware(&fw_entry, "haup-ir-blaster.bin", &ir->c_tx.dev);
	if (ret != 0) {
		printk(KERN_ERR
			"lirc_pvr150: firmware haup-ir-blaster.bin "
			"not available (%d)\n", ret);
		ret = ret < 0 ? ret : -EFAULT;
		goto out;
	}
	printk(KERN_INFO
		"lirc_pvr150: firmware of size %zd loaded\n", fw_entry->size);
	
	/* Parse the file */	
	tx_data = vmalloc(sizeof(TX_DATA));
	if (tx_data == NULL) {
		printk(KERN_ERR
			"lirc_pvr150: out of memory\n");
		release_firmware(fw_entry);
		ret = -ENOMEM;
		goto out;
        }
	tx_data->code_sets = NULL;
	
	/* Copy the data so hotplug doesn't get confused and timeout */
	tx_data->datap = vmalloc(fw_entry->size);
	if (tx_data->datap == NULL) {
		printk(KERN_ERR
			"lirc_pvr150: out of memory\n");
		release_firmware(fw_entry);
		vfree(tx_data);
		ret = -ENOMEM;
		goto out;
	}
	memcpy(tx_data->datap, fw_entry->data, fw_entry->size);
	tx_data->endp = tx_data->datap + fw_entry->size;
	release_firmware(fw_entry); fw_entry = NULL;
			
	/* Check version */
	data = tx_data->datap;
	if (!read_uint8(&data, tx_data->endp, &version))
		goto corrupt;
	if (version != 1) {
		printk(KERN_ERR
			"lirc_pvr150: unsupported code set file version (%u, "
			"expected 1) -- please upgrade to a newer driver", 
			version);
		fw_unload_locked();
		ret = -EFAULT;
		goto out;
	}
	
	/* Save boot block for later */
	tx_data->boot_data = data;
	if (!skip(&data, tx_data->endp, TX_BLOCK_SIZE))
		goto corrupt;

	if (!read_uint32(&data, tx_data->endp, 
			      &tx_data->num_code_sets))
		goto corrupt;

	printk(KERN_INFO 
		"lirc_pvr150: %u codesets loaded\n", tx_data->num_code_sets);
	
	tx_data->code_sets = vmalloc(
		tx_data->num_code_sets * sizeof(char *));
	if (tx_data->code_sets == NULL) {
		fw_unload_locked();
		ret = -ENOMEM;
		goto out;
	}
	
	for (i = 0; i < TX_BLOCK_SIZE; ++i)
		tx_data->fixed[i] = -1;
	
	/* Read global fixed data template */
	if (!read_uint8(&data, tx_data->endp, &num_global_fixed) || 
	    num_global_fixed > TX_BLOCK_SIZE)
		goto corrupt;
	for (i = 0; i < num_global_fixed; ++i) {
		unsigned char pos, val;
		if (!read_uint8(&data, tx_data->endp, &pos) || 
		    !read_uint8(&data, tx_data->endp, &val) || 
		    pos >= TX_BLOCK_SIZE) {
			goto corrupt;
		}
		tx_data->fixed[pos] = (int)val;
	}

	/* Filch out the position of each code set */
	for (i = 0; i < tx_data->num_code_sets; ++i) {
		unsigned int id;
		unsigned char keys;
		unsigned char ndiffs;
		
		/* Save the codeset position */
		tx_data->code_sets[i] = data;
		
		/* Read header */
		if (!read_uint32(&data, tx_data->endp, &id) || 
		    !read_uint8(&data, tx_data->endp, &keys) || 
		    !read_uint8(&data, tx_data->endp, &ndiffs) || 
		    ndiffs > TX_BLOCK_SIZE || keys == 0)
		    	goto corrupt;

		/* skip diff positions */		    	
		if (!skip(&data, tx_data->endp, ndiffs)) 
			goto corrupt;
		
		/* After the diffs we have the first key id + data - 
		   global fixed */
		if (!skip(&data, tx_data->endp, 
			       1 + TX_BLOCK_SIZE - num_global_fixed))
			goto corrupt;
		
		/* Then we have keys-1 blocks of key id+diffs */
		if (!skip(&data, tx_data->endp,
			       (ndiffs + 1) * (keys - 1)))
			goto corrupt;
	}
	ret = 0;
	goto out;
		
corrupt:
	printk(KERN_ERR "lirc_pvr150: firmware is corrupt\n");
	fw_unload_locked();
	ret = -EFAULT;
	
out:
	up(&tx_data_lock);
	return ret;
}

/* initialise the IR TX device of the PVR-150 */
static int tx_init(struct IR *ir)
{
	int ret;
	
	/* Load 'firmware' */
	ret = fw_load(ir);
	if (ret != 0)
		return ret;
	
	/* Send boot block */
	ret = send_boot_data(ir);
	if (ret != 0)
		return ret;
	ir->need_boot = 0;
		
	/* Looks good */
	return 0;
}

/* do nothing stub to make LIRC happy */
static loff_t lseek(struct file *filep,loff_t offset,int orig)
{
        return(-ESPIPE);
}

/* copied from lirc_dev */
static ssize_t read(struct file *filep,char *outbuf,size_t n,loff_t *ppos)
{
	struct IR *ir = (struct IR *)filep->private_data;
	unsigned char buf[ir->buf.chunk_size];
	int ret=0, written=0;
	DECLARE_WAITQUEUE(wait, current);

	dprintk("read called\n");
	if (ir->c_rx.addr == 0)
		return -ENODEV;

	if (down_interruptible(&ir->buf_sem))
		return -ERESTARTSYS;

	if (n % ir->buf.chunk_size) {
		dprintk("read result = -EINVAL\n");
		up(&ir->buf_sem);
		return -EINVAL;
	}

	/* we add ourselves to the task queue before buffer check 
         * to avoid losing scan code (in case when queue is awaken somewhere 
	 * beetwen while condition checking and scheduling)
	 */
	add_wait_queue(&ir->buf.wait_poll, &wait);
	set_current_state(TASK_INTERRUPTIBLE);

	/* while we did't provide 'length' bytes, device is opened in blocking
	 * mode and 'copy_to_user' is happy, wait for data.
	 */
	while (written < n && ret == 0) { 
		if (lirc_buffer_empty(&ir->buf)) {
			/* According to the read(2) man page, 'written' can be
			 * returned as less than 'n', instead of blocking
			 * again, returning -EWOULDBLOCK, or returning
			 * -ERESTARTSYS */
			if (written) break;
			if (filep->f_flags & O_NONBLOCK) {
				ret = -EWOULDBLOCK;
				break;
			}
			if (signal_pending(current)) {
				ret = -ERESTARTSYS;
				break;
			}
			schedule();
			set_current_state(TASK_INTERRUPTIBLE);
		} else {
			lirc_buffer_read_1(&ir->buf, buf);
			ret = copy_to_user((void *)outbuf+written, buf,
					   ir->buf.chunk_size);
			written += ir->buf.chunk_size;
		}
	}

	remove_wait_queue(&ir->buf.wait_poll, &wait);
	set_current_state(TASK_RUNNING);
	up(&ir->buf_sem);
	
	dprintk("read result = %s (%d)\n",
		ret ? "-EFAULT" : "OK", ret);

	return ret ? ret : written;
}

/* send a keypress to the IR TX device of the PVR-150 */
static int send_code(struct IR *ir, unsigned int code, unsigned int key)
{
	unsigned char data_block[TX_BLOCK_SIZE];
	unsigned char buf[2];
	int i, ret;

	/* Get data for the codeset/key */
	ret = get_key_data(data_block, code, key);
	
	if (ret == -EPROTO) {
		printk(KERN_ERR
			"lirc_pvr150: failed to get data for code %u, "
			"key %u -- check lircd.conf entries\n",
			code, key);
		return ret;
	} else if (ret != 0)
		return ret;
	
	/* Send the data block */
	ret = send_data_block(ir, data_block);
	if (ret != 0)
		return ret;

	/* Send data block length? */
	buf[0] = 0x00;
	buf[1] = 0x40;
	ret = i2c_master_send(&ir->c_tx, buf, 2);
	if (ret != 2) {
		printk(KERN_ERR
			"lirc_pvr150: i2c_master_send failed with %d\n", ret);
		return ret < 0 ? ret : -EFAULT;
	}
	ret = i2c_master_send(&ir->c_tx, buf, 1);
	if (ret != 1) {
		printk(KERN_ERR
			"lirc_pvr150: i2c_master_send failed with %d\n", ret);
		return ret < 0 ? ret : -EFAULT;
	}

	/* Send finished download? */
	ret = i2c_master_recv(&ir->c_tx, buf, 1);
	if (ret != 1) {
		printk(KERN_ERR
			"lirc_pvr150: i2c_master_recv failed with %d\n", ret);
		return ret < 0 ? ret : -EFAULT;
	}
	if (buf[0] != 0xA0) {
		printk(KERN_ERR
			"lirc_pvr150: unexpected IR TX response #1: %02x\n", 
			buf[0]);
		return ret < 0 ? ret : -EFAULT;
	}

	/* Send prepare command? */
	buf[0] = 0x00;
	buf[1] = 0x80;
	ret = i2c_master_send(&ir->c_tx, buf, 2);
	if (ret != 2) {
		printk(KERN_ERR
			"lirc_pvr150: i2c_master_send failed with %d\n", ret);
		return ret < 0 ? ret : -EFAULT;
	}

	/* This bit NAKs until the device is ready, so we retry it
	   sleeping a bit each time.  This seems to be what the windows
	   driver does, approximately.
	   Try for up to 1s.
	*/
	for (i = 0; i < 20; ++i) {
		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_timeout((50 * HZ + 999) / 1000);
		ret = i2c_master_send(&ir->c_tx, buf, 1);
		if (ret == 1)
			break;
		dprintk("NAK expected: i2c_master_send "
			"failed with %d (try %d)\n", ret, i+1);
	}
	if (ret != 1) {
		printk(KERN_ERR
			"lirc_pvr150: IR TX chip never got ready: last "
			"i2c_master_send failed with %d\n", ret);
		return ret < 0 ? ret : -EFAULT;
	}

	/* Seems to be an 'ok' response */
	i = i2c_master_recv(&ir->c_tx, buf, 1);
	if (i != 1) {
		printk(KERN_ERR
			"lirc_pvr150: i2c_master_recv failed with %d\n", ret);
		return ret < 0 ? ret : -EFAULT;
	}
	if (buf[0] != 0x80) {
		printk(KERN_ERR
			"lirc_pvr150: unexpected IR TX response #2: %02x\n", 
			buf[0]);
		return -EFAULT;
	}
	
	/* Oh good, it worked */
	dprintk("sent code %u, key %u\n", code, key);
	return 0;
}

/* Write a code to the device.  We take in a 32-bit number (a lirc_t) and then
   decode this to a codeset/key index.  The key data is then decompressed and
   sent to the device.  We have a spin lock as per i2c documentation to prevent
   multiple concurrent sends which would probably cause the device to explode.
 */
static ssize_t write(struct file *filep,const char *buf,size_t n,
			  loff_t *ppos)
{
	struct IR *ir = (struct IR *)filep->private_data;
	size_t i;
	int failures = 0;

	if (ir->c_tx.addr == 0)
		return -ENODEV;

	/* Validate user parameters */
	if (n % sizeof(lirc_t)) 
		return(-EINVAL);

	/* Lock i2c bus for the duration */
	down(&ir->lock);
	
	/* Send each keypress */
	for (i = 0; i < n; )
	{
		int ret = 0;
		lirc_t command;
		
		if (copy_from_user(&command, buf + i, sizeof(command))) {
			up(&ir->lock);
			return -EFAULT;
		}
	
		/* Send boot data first if required */
		if (ir->need_boot == 1) 
		{
			ret = send_boot_data(ir);
			if (ret == 0)
				ir->need_boot = 0;
		}

		/* Send the code */
		if (ret == 0) {
			ret = send_code(ir, (unsigned)command >> 16, 
					    (unsigned)command & 0xFFFF);
			if (ret == -EPROTO) {
				up(&ir->lock);
				return ret;
			}
		}
				     
		/* Hmm, a failure.  If we've had a few then give up, otherwise
		   try a reset 
		 */
		if (ret != 0) {
			/* Looks like the chip crashed, reset it */
			printk(KERN_ERR "lirc_pvr150: sending to the IR "
			                "transmitter chip failed, trying "
			                "reset\n");

			if (failures >= 3) {
				printk(KERN_ERR "lirc_pvr150: unable to send "
				                "to the IR chip after 3 "
				                "resets, giving up\n");
				up(&ir->lock);
				return ret;
			}
			ivtv_reset_ir_gpio(i2c_get_adapdata(ir->c_tx.adapter));
			set_current_state(TASK_UNINTERRUPTIBLE);
			schedule_timeout((100 * HZ + 999) / 1000);
			ir->need_boot = 1;
			++failures;
		}
		else
		{
			i += sizeof(lirc_t);
		}
	}
	
	/* Release i2c bus */
	up(&ir->lock);

	/* All looks good */
	return n;
}

/* copied from lirc_dev */
static unsigned int poll(struct file *filep, poll_table * wait)
{
	struct IR *ir = (struct IR *)filep->private_data;
	unsigned int ret;

	dprintk("poll called\n");
	if (ir->c_rx.addr == 0)
		return -ENODEV;

	down(&ir->buf_sem);

	poll_wait(filep, &ir->buf.wait_poll, wait);

	dprintk("poll result = %s\n",
		lirc_buffer_empty(&ir->buf) ? "0" : "POLLIN|POLLRDNORM");

	ret = lirc_buffer_empty(&ir->buf) ? 0 : (POLLIN|POLLRDNORM);
	
	up(&ir->buf_sem);
	return ret;
}

static int ioctl(struct inode *node,struct file *filep,unsigned int cmd,
                      unsigned long arg)
{
	struct IR *ir = (struct IR *)filep->private_data;
        int result;
        unsigned long mode, features = 0;
        
        if (ir->c_rx.addr != 0)
        	features |= LIRC_CAN_REC_LIRCCODE;
        if (ir->c_tx.addr != 0)
        	features |= LIRC_CAN_SEND_PULSE;

        switch(cmd)
	{
	case LIRC_GET_LENGTH:
		result = put_user((unsigned long)13,
				  (unsigned long *)arg);
		break;
	case LIRC_GET_FEATURES:
		result=put_user(features,(unsigned long *) arg);
		if(result) return(result);
		break;
	case LIRC_GET_REC_MODE:
		if(!(features&LIRC_CAN_REC_MASK))
			return -ENOSYS;

		result = put_user(LIRC_REC2MODE
				  (features&LIRC_CAN_REC_MASK),
				  (unsigned long*)arg);
		break;
	case LIRC_SET_REC_MODE:
		if(!(features&LIRC_CAN_REC_MASK))
			return -ENOSYS;

		result = get_user(mode, (unsigned long*)arg);
		if(!result && !(LIRC_MODE2REC(mode) & features)) {
			result = -EINVAL;
		}
		break;
	case LIRC_GET_SEND_MODE:
		if(!(features&LIRC_CAN_SEND_MASK))
			return -ENOSYS;

		result=put_user(LIRC_MODE_PULSE,(unsigned long *) arg);
		if(result) return(result);
		break;
	case LIRC_SET_SEND_MODE:
		if(!(features&LIRC_CAN_SEND_MASK))
			return -ENOSYS;

		result=get_user(mode,(unsigned long *) arg);
		if(result) return(result);
		if(mode!=LIRC_MODE_PULSE) return(-EINVAL);
		break;
	default:
		return(-ENOIOCTLCMD);
	}
	return (0);
}

/* Open the IR device of the PVR-150.  Get hold of our IR structure and
   stash it in private_data for the file */
static int open(struct inode* node,struct file* filep)
{
	struct IR *ir;
	int ret;

	/* find our IR struct */
	unsigned minor = MINOR(node->i_rdev);
	if (minor >= MAX_IRCTL_DEVICES) {
		dprintk("minor %d: open result = -ENODEV\n",
			minor);
		return -ENODEV;
	}
	ir = ir_devices[minor];

	/* increment in use count */
	down(&ir->lock);
	++ir->open;
	ret = set_use_inc(ir);
	if (ret != 0) {
		--ir->open;
		up(&ir->lock);
		return ret;
	}
	up(&ir->lock);
	
	/* stash our IR struct */
	filep->private_data = ir;
	
        return(0);
}

/* Close the IR device of the PVR-150 */
static int close(struct inode* node,struct file* filep)
{
	/* find our IR struct */
	struct IR *ir = (struct IR *)filep->private_data;
	if (ir == NULL) {
		printk(KERN_ERR
			"lirc_pvr150: close: no private_data "
			"attached to the file!\n");
		return -ENODEV;
	}

	/* decrement in use count */	
	down(&ir->lock);
	--ir->open;
	set_use_dec(ir);
	up(&ir->lock);
	
        return(0);
}

static struct lirc_plugin lirc_template = {
	name:        "lirc_pvr150",
	set_use_inc: set_use_inc,
	set_use_dec: set_use_dec,
	owner:       THIS_MODULE
};

/* ----------------------------------------------------------------------- */

static int ir_attach(struct i2c_adapter *adap, int have_rx, int have_tx);
static int ir_detach(struct i2c_client *client);
static int ir_probe(struct i2c_adapter *adap);
static int ir_command(struct i2c_client *client, unsigned int cmd, void *arg);

static struct i2c_driver driver = {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 16)
        name:           "i2c ir driver",
        flags:          I2C_DF_NOTIFY,
#else
	.driver = {
		owner:  THIS_MODULE,
		name:   "i2c ir driver",
	},
#endif
        id:             I2C_DRIVERID_EXP3, /* FIXME */
        attach_adapter: ir_probe,
        detach_client:  ir_detach,
        command:        ir_command,
};

static struct i2c_client client_template = 
{
        name:   "unset",
        driver: &driver
};

static struct file_operations lirc_fops =
{
	llseek:  lseek,
	read:    read,
	write:   write,
	poll:    poll,
	ioctl:   ioctl,
	open:    open,
	release: close
};

static int i2c_attach(struct i2c_client *client, struct IR *ir)
{
	int ret;

	i2c_set_clientdata(client, ir);

	ret = i2c_attach_client(client);
	if (ret != 0) {
		client->addr = 0;
		return ret;
	}
	if (i2c_use_client(client) == NULL) {
		i2c_detach_client(client);
		client->addr = 0;
		return ret;
	}
	++ir->devs;
	return 0;
}

static int ir_attach(struct i2c_adapter *adap, int have_rx, int have_tx)
{
        struct IR *ir;
        int ret, i;
	
	printk("lirc_pvr150: chip found with %s\n",
		have_rx && have_tx ? "RX and TX" :
			have_rx ? "RX only" : "TX only");
			
        if (NULL == (ir = kmalloc(sizeof(struct IR),GFP_KERNEL)))
                return -ENOMEM;
	if (lirc_buffer_init(&ir->buf, 2, BUFLEN/2) != 0) {
		kfree(ir);
		return -ENOMEM;
	}
	init_MUTEX(&ir->lock);
	init_MUTEX(&ir->buf_sem);
	ir->open = 0;
	ir->devs = 0;
	ir->tpid = 0;
	ir->need_boot = 1;
	ir->shutdown = 0;
        ir->t_notify = ir->t_notify2 = NULL;
	for (i = 0; i < sizeof(ir->b); ++i)
		ir->b[0] = 0;
		
        memcpy(&ir->l,&lirc_template,sizeof(struct lirc_plugin));
        ir->l.minor = -1;

	/* initialise RX device */
        client_template.adapter = adap;
       	memcpy(&ir->c_rx,&client_template,sizeof(struct i2c_client));
        if (have_rx) {
		DECLARE_COMPLETION(tn);
		
        	/* I2C attach to device */
       	        ir->c_rx.addr = 0x71;
		strlcpy(ir->c_rx.name, "Hauppauge PVR150 RX", 
			sizeof(ir->c_rx.name));
		if ( (ret = i2c_attach(&ir->c_rx, ir)) != 0 )
			goto err;
		
		/* try to fire up polling thread */
		ir->t_notify = &tn;
		ir->tpid = kernel_thread(lirc_thread, ir, 0);
		if (ir->tpid < 0) {
			printk(KERN_ERR "lirc_pvr150: lirc_register_plugin: "
			       "cannot run poll thread\n");
			ret = -ECHILD;
			goto err;
		}
		wait_for_completion(&tn);
		ir->t_notify = NULL;
	}
	
	/* initialise TX device */
       	memcpy(&ir->c_tx,&client_template,sizeof(struct i2c_client));
	if (have_tx) {
        	/* I2C attach to device */
	        ir->c_tx.addr = 0x70;
		strlcpy(ir->c_tx.name, "Hauppauge PVR150 TX",
			sizeof(ir->c_tx.name));
		if ( (ret = i2c_attach(&ir->c_tx, ir)) != 0 )
			goto err;
	}	
	
	/* set lirc_dev stuff */
	ir->l.code_length = 13;
	ir->l.rbuf	  = &ir->buf;
	ir->l.fops        = &lirc_fops;
	ir->l.data        = ir;
	ir->l.minor       = minor;
	ir->l.sample_rate = 0;
	
	/* register with lirc */
	ir->l.minor = lirc_register_plugin(&ir->l);
	if (ir->l.minor < 0 || ir->l.minor >= MAX_IRCTL_DEVICES) {
		printk(KERN_ERR
			"lirc_pvr150: ir_attach: "
		        "\"minor\" must be between 0 and %d (%d)!\n",
		       MAX_IRCTL_DEVICES-1, ir->l.minor);
		ret = -EBADRQC;
		goto err;
	}
		
	/* store this for getting back in open() later on */
	ir_devices[ir->l.minor] = ir;

	/* if we have the tx device, load the 'firmware'.  We do this
	   after registering with lirc as otherwise hotplug seems to take
	   10s to create the lirc device.
	 */
	if (have_tx) {
		/* Special TX init */
		ret = tx_init(ir);
		if (ret != 0)
			goto err;			
	}
	return 0;

err:
	/* undo everything, hopefully... */
	if (ir->c_rx.addr) ir_detach(&ir->c_rx);
	if (ir->c_tx.addr) ir_detach(&ir->c_tx);
	return ret;
}

static int ir_detach(struct i2c_client *client)
{
	struct IR *ir = i2c_get_clientdata(client);
	down(&ir->lock);

	if (client == &ir->c_rx) {
		DECLARE_COMPLETION(tn);
		DECLARE_COMPLETION(tn2);
	
		/* end up polling thread */
		if (ir->tpid >= 0) {
			ir->t_notify = &tn;
			ir->t_notify2 = &tn2;
			ir->shutdown = 1;
			{
				struct task_struct *p;
				p = find_task_by_pid_ns(ir->tpid, &init_pid_ns);
				wake_up_process(p);
			}
			complete(&tn2);
			wait_for_completion(&tn);
			ir->t_notify = NULL;
			ir->t_notify2 = NULL;
		}

		/* unregister device */
		i2c_release_client(&ir->c_rx);
		i2c_detach_client(&ir->c_rx);
	} else if (client == &ir->c_tx) {
		i2c_release_client(&ir->c_tx);
		i2c_detach_client(&ir->c_tx);
	} else {
		up(&ir->lock);
		printk(KERN_ERR "lirc_pvr150: ir_detach: detached from "
				 "something we didn't attach to\n");
		return -ENODEV;
	}

	--ir->devs;
	if (ir->devs < 0)
	{
		up(&ir->lock);
		printk(KERN_ERR "lirc_pvr150: ir_detach: invalid "
				"device count\n");
		return -ENODEV;
	}
	else if (ir->devs == 0)
	{		
		/* unregister lirc plugin */
		if (ir->l.minor >= 0 && ir->l.minor < MAX_IRCTL_DEVICES)
		{
			lirc_unregister_plugin(ir->l.minor);		
			ir_devices[ir->l.minor] = NULL;
		}
	
		/* free memory */
		lirc_buffer_free(&ir->buf);
		up(&ir->lock);
		kfree(ir);
		return 0;
	}
	up(&ir->lock);
	return 0;
}

static int ir_probe(struct i2c_adapter *adap) 
{	
	struct i2c_client c; 
	char buf; 

#ifdef I2C_HW_B_CX2341X
	if (adap->id == (I2C_ALGO_BIT | I2C_HW_B_BT848) ||
		adap->id == (I2C_ALGO_BIT | I2C_HW_B_CX2341X))
#else
	if (adap->id == (I2C_ALGO_BIT | I2C_HW_B_BT848))
#endif
	{
		int have_rx = 0, have_tx = 0;
		
		/* The external IR receiver is at i2c address 0x71 on the PVR-150
		   The IR transmitter is at 0x70.
		*/
		memset(&c,0,sizeof(c));
		c.adapter = adap;
		c.addr = 0x70;
		
		if (!disable_rx) {
			if (i2c_master_recv(&c,&buf,1) == 1)
				have_rx = 1;
			dprintk("probe 0x70 @ %s: %s\n",
				adap->name, 
				have_rx ? "yes" : "no");
		}

		if (!disable_tx) {
			c.addr = 0x71;
			if (i2c_master_recv(&c,&buf,1) == 1)
				have_tx = 1;
			dprintk("probe 0x71 @ %s: %s\n",
				adap->name, 
				have_tx ? "yes" : "no");
		}
				
		if (have_rx || have_tx)
			return ir_attach(adap, have_rx, have_tx);
		else
		{
			printk(KERN_ERR "lirc_pvr150: %s: no devices found\n", adap->name);
		}
	}
		
	return 0;
}

static int ir_command(struct i2c_client *client,unsigned int cmd, void *arg)
{
	/* nothing */
	return 0;
}

/* ----------------------------------------------------------------------- */
#ifdef MODULE

int init_module(void)
{
	init_MUTEX(&tx_data_lock);
	request_module("ivtv");
	request_module("firmware_class");
	i2c_add_driver(&driver);
	return 0;
}

void cleanup_module(void)
{
	i2c_del_driver(&driver);
	/* if loaded */
	fw_unload();
}

MODULE_DESCRIPTION("Infrared receiver driver for Hauppauge PVR-150 card (i2c stack)");
MODULE_AUTHOR("Gerd Knorr, Michal Kochanowicz, Christoph Bartelmus, Ulrich Mueller, Stefan Jahn, Jerome Brock, Mark Weaver");
MODULE_LICENSE("GPL");

module_param(minor, int, 0444);
MODULE_PARM_DESC(minor, "Preferred minor device number");

module_param(debug, bool, 0644);
MODULE_PARM_DESC(debug, "Enable debugging messages");

module_param(disable_rx, bool, 0644);
MODULE_PARM_DESC(disable_rx, "Disable the IR receiver device");

module_param(disable_tx, bool, 0644);
MODULE_PARM_DESC(disable_tx, "Disable the IR transmitter device");

EXPORT_NO_SYMBOLS;

#endif /* MODULE */

/*
 * Overrides for Emacs so that we follow Linus's tabbing style.
 * ---------------------------------------------------------------------------
 * Local variables:
 * c-basic-offset: 8
 * End:
 */
