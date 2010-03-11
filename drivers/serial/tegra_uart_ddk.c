/*
 * drivers/serial/tegra_uart_ddk.c
 *
 * NvDdk and NvRm-based high-speed serial driver for NVIDIA Tegra SoCs
 *
 * Copyright (C) 2009 NVIDIA Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/module.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/tegra_devices.h>

#include "nvos.h"
#include "mach/nvrm_linux.h"
#include "nvddk_uart.h"
#include "nvassert.h"


/* Use workqueues for Tx, instead of executing inside caller context as the
 * caller it is not allowed to sleep in the caller context.
 * */
#define USE_WORKQUEUE_FOR_TX 1

struct tegra_uart_port  {
	struct uart_port uport;
	NvDdkUartHandle hUart;
	NvU8 *RxBuffer;
	struct task_struct *RxThread;
	NvOsSemaphoreHandle RxSemaphore;
#if USE_WORKQUEUE_FOR_TX
	struct work_struct TxWorkQueue;
#endif
};

int tegra_rx_thread(void *args)
{
	struct tegra_uart_port *t = (struct tegra_uart_port *)args;
	struct tty_struct *tty;
	NvU32 avail;
	NvError err;
	int ret;

	while (!kthread_should_stop()) {
		err = NvOsSemaphoreWaitTimeout(t->RxSemaphore,1000);
		if (err == NvError_Timeout)
			continue;

		NV_ASSERT(err == NvSuccess);
		if (kthread_should_stop())
			return 0;

		tty = t->uport.info->port.tty;
		err = NvDdkUartUpdateReceiveBuffer(t->hUart, &avail);
		if (err == NvError_UartOverrun) {
			tty_insert_flip_char(tty, 0, TTY_OVERRUN);
			t->uport.icount.overrun++;
		} else if (err == NvError_UartParity) {
			tty_insert_flip_char(tty, 0, TTY_PARITY);
			t->uport.icount.parity++;
		} else if (err == NvError_UartFraming) {
			tty_insert_flip_char(tty, 0, TTY_FRAME);
		} else  {
			while (avail) {
				NvU32 read;
				NvU32 to_read;

				to_read = (avail > SERIAL_XMIT_SIZE) ?
					SERIAL_XMIT_SIZE: avail;

				NV_ASSERT_SUCCESS(NvDdkUartRead(t->hUart, 
					t->RxBuffer, to_read, &read, 0));
				BUG_ON(read != to_read);
				avail -= read;
				ret = tty_insert_flip_string(tty,
					t->RxBuffer, read);
				BUG_ON(ret != read);
				tty_flip_buffer_push(tty);
			}
		}
	}

	return 0;
}


static unsigned int tegra_tx_empty(struct uart_port *u)
{
	NvU32 ToSend;
	NvU32 TxStatus;
	NvU32 RxAvail;
	NvU32 RxStatus;

	struct tegra_uart_port *t;
	t = container_of(u, struct tegra_uart_port, uport);

	/* Check if the Tx fifo is empty, if so retrun TIOCSER_TEMT, otherwise
	 * return 0 */
	NvDdkUartGetTransferStatus(t->hUart, &ToSend, 
		&TxStatus, &RxAvail, &RxStatus);

	return (ToSend == 0) ? TIOCSER_TEMT : 0;
}

static void tegra_set_mctrl(struct uart_port *u, unsigned int mctrl)
{
}

static unsigned int tegra_get_mctrl(struct uart_port *u)
{
	/* RI - Ring detector is active
	 * CD/DCD/CAR - Carrier detect is always active. For some reason 
	 *			  linux has different names for carrier detect.
	 * DSR - Data Set ready is active as the hardware doesn't support it.
	 *	   Don't know if the linux support this yet?
	 * CTS - Clear to send. Always set to active, as the hardware handles
	 *	   CTS automatically.
	 * */
	return TIOCM_RI | TIOCM_CD | TIOCM_DSR | TIOCM_CTS;
}

static void tegra_stop_tx(struct uart_port *u)
{
	struct tegra_uart_port *t;

	t = container_of(u, struct tegra_uart_port, uport);
	NvDdkUartStopWrite(t->hUart);
}

static int tegra_uart_chars_in_buffer(struct tegra_uart_port *t)
{
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&t->uport.lock, flags);
	ret = uart_circ_chars_pending(&t->uport.info->xmit);
	spin_unlock_irqrestore(&t->uport.lock, flags);
	return ret;
}

static void tegra_tx_locked(struct tegra_uart_port *t)
{
	struct circ_buf *txBuffer;
	NvU32 Written;
	NvU32 TimeOut = NV_WAIT_INFINITE;
	NvU32 writecount;
	unsigned long flags;
	NvError e;

	while (1) {
		writecount = tegra_uart_chars_in_buffer(t);
		if (!writecount)
			break;

		txBuffer = &t->uport.info->xmit;
		/* For wrap around case, transmit in 2 stages */
		if (writecount > (UART_XMIT_SIZE - txBuffer->tail)) {
			writecount = UART_XMIT_SIZE - txBuffer->tail;
		}

		e = NvDdkUartWrite(t->hUart, &txBuffer->buf[txBuffer->tail], 
			writecount, &Written, TimeOut);

		BUG_ON(e != NvSuccess);
		BUG_ON(writecount != Written);
		writecount = 0;

		spin_lock_irqsave(&t->uport.lock, flags);
		txBuffer->tail = (txBuffer->tail + Written) &
			(UART_XMIT_SIZE - 1);
		t->uport.icount.tx += Written;
		spin_unlock_irqrestore(&t->uport.lock, flags);
	}

	if (tegra_uart_chars_in_buffer(t) < WAKEUP_CHARS)
		uart_write_wakeup(&t->uport);
}

#if USE_WORKQUEUE_FOR_TX
static void tegra_tx_work(struct work_struct *work)
{
	struct tegra_uart_port *t;
	t = container_of(work, struct tegra_uart_port, TxWorkQueue);
	tegra_tx_locked(t);
}
#endif

static void tegra_start_tx_locked(struct uart_port *u)
{
	struct tegra_uart_port *t;
	t = container_of(u, struct tegra_uart_port, uport);

	if (uart_circ_empty(&u->info->xmit) || u->info->port.tty->stopped) {
		tegra_stop_tx(u);
		return;
	}

#if USE_WORKQUEUE_FOR_TX
	schedule_work(&t->TxWorkQueue);
#else
	tegra_tx_locked(t);
#endif
	return;
}

static void tegra_stop_rx(struct uart_port *u)
{
}

static void tegra_enable_ms(struct uart_port *u)
{
	struct tegra_uart_port *t;
	t = container_of(u, struct tegra_uart_port, uport);

	NV_ASSERT_SUCCESS(NvDdkUartSetFlowControlSignal(t->hUart, 
		NvDdkUartSignalName_Cts, 
		NvDdkUartFlowControl_Handshake));
}

static void tegra_break_ctl(struct uart_port *u, int ctl)
{
	struct tegra_uart_port *t;
	t = container_of(u, struct tegra_uart_port, uport);

	NvDdkUartSetBreakSignal(t->hUart, ctl ? NV_TRUE: NV_FALSE);
}

static void tegra_config_port(struct uart_port *u, int flags)
{
}

static void tegra_shutdown(struct uart_port *u)
{
	struct tegra_uart_port *t;
	t = container_of(u, struct tegra_uart_port, uport);
	
	(void)kthread_stop(t->RxThread);
	NvOsSemaphoreDestroy(t->RxSemaphore);
	NvDdkUartClose(t->hUart);

	t->RxSemaphore = 0;
	t->hUart = 0;
	t->RxThread = 0;
}

static int tegra_startup(struct uart_port *u)
{
	NvError e = NvSuccess;
	struct tegra_uart_port *t;
	t = container_of(u, struct tegra_uart_port, uport);
	int ret = 0;
	
	NV_CHECK_ERROR_CLEANUP(NvDdkUartOpen(s_hRmGlobal, u->line, &t->hUart));

	t->RxBuffer = NvOsAlloc(SERIAL_XMIT_SIZE);
	e = NvOsSemaphoreCreate(&t->RxSemaphore, 0);
	if (!t->RxBuffer || e != NvSuccess)
	{
		ret = -EINVAL;
		goto fail;
	}

	/* Start Rx */
	NV_CHECK_ERROR_CLEANUP(NvDdkUartStartReadOnBuffer(t->hUart, 
		t->RxSemaphore, SERIAL_XMIT_SIZE));
	t->RxThread = kthread_create(tegra_rx_thread, t, "UartRxthread/%d",
		u->line);
	if( IS_ERR(t->RxThread) )
	{
		goto fail;
	}

#if USE_WORKQUEUE_FOR_TX
	INIT_WORK(&t->TxWorkQueue, tegra_tx_work);
#endif
	wake_up_process(t->RxThread);

	return ret;

fail:
	if (e != NvSuccess)
		ret = -EINVAL;

	printk(KERN_ERR " %s: failed with error(%d)\n", __func__, ret);
	if (t->RxBuffer) NvOsFree(t->RxBuffer);
	if (t->RxSemaphore) NvOsSemaphoreDestroy(t->RxSemaphore);
	return ret;
}

void tegra_set_termios(struct uart_port *u, struct ktermios *termios,
					   struct ktermios *oldtermios)
{
	unsigned int bps;
	unsigned long flags;
	unsigned int c_cflag = termios->c_cflag;
	NvDdkUartConfiguarations c;
	struct tegra_uart_port *t;
	t = container_of(u, struct tegra_uart_port, uport);
							
	spin_lock_irqsave(&u->lock, flags);

	/* Get the current configuration, and modify the needed values */
	NvDdkUartGetConfiguration(t->hUart, &c);

	bps = uart_get_baud_rate(u, termios, oldtermios, 200, 4000000);
	c.UartBaudRate = bps;

	/* set parity */
	c.UartParityBit = NvDdkUartParity_None;
	if (PARENB == (c_cflag & PARENB)) {
		if (PARODD == (c_cflag & PARODD)) {
			c.UartParityBit = NvDdkUartParity_Odd;
		} else if (CMSPAR == (c_cflag & CMSPAR)) { 
			/* FIXME DDK doesn't support space parity? */
			/* data |= SPACE_PARITY; */
		} else {
			c.UartParityBit = NvDdkUartParity_Even;
		}
	}

	/* Set bits per char */ 
	c.UartDataLength = 8;
	switch (c_cflag & CSIZE) {
	case CS5:
		c.UartDataLength = 5;
		break;
	case CS6:
		c.UartDataLength = 6;
		break;
	case CS7:
		c.UartDataLength = 7;
	   break;
	default:
		c.UartDataLength = 8;
		break;
	}
	/* stop bits */ 
	if (c_cflag & CSTOPB) {
		c.UartStopBit = NvDdkUartStopBit_2;
	} else {
		/* otherwise 1 stop bit */
		c.UartStopBit = NvDdkUartStopBit_1;
	}

	/* 
	 * For now, always assume CTS/RTS flow control is always on.
	 */
	termios->c_cflag |= CRTSCTS;
	c_cflag |= CRTSCTS;

	NvDdkUartSetFlowControlSignal(t->hUart, NvDdkUartSignalName_Cts, 
		NvDdkUartFlowControl_Handshake);
	NvDdkUartSetFlowControlSignal(t->hUart, NvDdkUartSignalName_Rts, 
		NvDdkUartFlowControl_Handshake);

	u->ignore_status_mask = termios->c_iflag & INPCK;
	u->ignore_status_mask |= termios->c_iflag & IGNPAR;
	u->read_status_mask = (termios->c_cflag & CREAD);

	/* Set Transmit software time out */
	uart_update_timeout(u, c_cflag, bps);

	NvDdkUartClearReceiveBuffer(t->hUart);

	printk(KERN_INFO "Uart%d bps %d, parity %s, datalength %d,"
		"stopbits %d flow control %s\n", u->line, c.UartBaudRate, 
		(c.UartParityBit == NvDdkUartParity_None) ? "none" : 
		((c.UartParityBit == NvDdkUartParity_Odd) ? "odd" : "even"),
		c.UartDataLength, c.UartStopBit,
		(c_cflag & CRTSCTS) ? "on" : "off");

	NV_ASSERT_SUCCESS(NvDdkUartSetConfiguration(t->hUart, &c));
	spin_unlock_irqrestore(&u->lock, flags);
}

static const char *tegra_type(struct uart_port *u)
{
	return "TEGRA UART";
}

static void tegra_pm(struct uart_port *u, unsigned int state, 
	unsigned int oldstate)
{
}

static struct uart_ops tegra_uart_ops = {
	.tx_empty	= tegra_tx_empty,

	.set_mctrl	= tegra_set_mctrl,
	.get_mctrl	= tegra_get_mctrl,

	.stop_tx	= tegra_stop_tx,
	.start_tx	= tegra_start_tx_locked,
	.stop_rx	= tegra_stop_rx,

	.enable_ms	= tegra_enable_ms,
	.break_ctl	= tegra_break_ctl,

	.startup	= tegra_startup,
	.shutdown	= tegra_shutdown,
	.set_termios	= tegra_set_termios,

	.pm		= tegra_pm,
	.type		= tegra_type,

	.config_port	= tegra_config_port,
};

static int __init tegra_uart_probe(struct platform_device *pdev);
static int __devexit tegra_uart_remove(struct platform_device *pdev);

static struct platform_driver tegra_uart_platform_driver = {
	.remove	= tegra_uart_remove,
	.probe	= tegra_uart_probe,
	.driver = {
		.name = "tegra_uart"
		}
};

static struct uart_driver tegra_uart_driver = {
	.owner = THIS_MODULE,
	.driver_name = "tegra_uart",
	.dev_name = "ttyHS",
	.cons = 0,
};

static int __init tegra_uart_probe(struct platform_device *pdev)
{
	struct tegra_uart_port *t;
	struct uart_port *u;
	int ret;
	
	if (pdev->id < 0 || pdev->id > tegra_uart_driver.nr) {
		printk(KERN_ERR "Invalid Uart instance (%d) \n", pdev->id);
		return -ENODEV;
	}

	t = kzalloc(sizeof(struct tegra_uart_port), GFP_KERNEL);
	if (!t) {
		printk(KERN_ERR "%s: Failed to allocate memory\n", __func__);
		return -ENOMEM;
	}
	u = &t->uport;
	u->dev = &pdev->dev;
	platform_set_drvdata(pdev, u);
	u->line = pdev->id;
	u->ops = &tegra_uart_ops;
	u->type = ~PORT_UNKNOWN;

	ret = uart_add_one_port(&tegra_uart_driver, u);
	if (ret) {
		printk(KERN_INFO "%s: Failed(%d) to add uart port %s%d\n",
			__func__, ret, tegra_uart_driver.dev_name, u->line);
		kfree(t);
		platform_set_drvdata(pdev, NULL);
		return ret;
	}

	printk(KERN_INFO "Registered UART port %s%d\n", 
		tegra_uart_driver.dev_name, u->line);

	return ret;
}

static int __devexit tegra_uart_remove(struct platform_device *pdev)
{
	struct tegra_uart_port *t = platform_get_drvdata(pdev);
	struct uart_port *u;

	if (pdev->id < 0 || pdev->id > tegra_uart_driver.nr) {
		printk(KERN_ERR "Invalid Uart instance (%d) \n", pdev->id);
	}

	u = &t->uport;
	uart_remove_one_port(&tegra_uart_driver, u);
	platform_set_drvdata(pdev, NULL);

	printk(KERN_INFO "Unregistered UART port %s%d\n", 
		tegra_uart_driver.dev_name, u->line);
	kfree(t);
	return 0;
}

static int __init tegra_uart_init(void)
{
	int ret;

	tegra_uart_driver.nr  = NvRmModuleGetNumInstances(s_hRmGlobal, 
		NvRmModuleID_Uart);
 
	ret = uart_register_driver(&tegra_uart_driver);
	if (unlikely(ret)) {
		printk(KERN_ERR "%s failed to load\n", __func__);
		return ret;
	}
	
	ret = platform_driver_register(&tegra_uart_platform_driver);
	if (unlikely(ret)) {
		printk(KERN_ERR "%s failed to load\n", __func__);
		uart_unregister_driver(&tegra_uart_driver);
		return ret;
	}
	
	printk(KERN_INFO "Initialized tegra uart driver\n");
	return ret;
}

static void __exit tegra_uart_exit(void)
{
	printk(KERN_INFO "Unloading tegra uart driver\n");
	platform_driver_unregister(&tegra_uart_platform_driver);
	uart_unregister_driver(&tegra_uart_driver);
}

module_init(tegra_uart_init);
module_exit(tegra_uart_exit);
MODULE_DESCRIPTION("High speed UART driver for NVIDIA Tegra chipset (DDK)");

