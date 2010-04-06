/*
 * drivers/serial/tegra_hsuart.c
 *
 * High-speed serial driver for NVIDIA Tegra SoCs
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

// #define DEBUG           1
// #define VERBOSE_DEBUG   1

#include <linux/module.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <asm/io.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/termios.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/string.h>
#include <linux/pagemap.h>
#include <linux/tegra_devices.h>

#include "nvos.h"
#include "mach/nvrm_linux.h"
#include "mach/dma.h"
#include "nvassert.h"
#include "nvrm_drf.h"
#include "ap15/aruart.h"
#include "nvrm_interrupt.h"
#include "nvrm_power.h"


#define TX_EMPTY_STATUS (NV_DRF_DEF(UART, LSR, TMTY, EMPTY) | \
	NV_DRF_DEF(UART, LSR, THRE, EMPTY))

#define UART_RX_DMA_BUFFER_SIZE    (4 *  1024 * 16)

#define UART_TX_DMA_BUFFERS_TOTAL  2

static int tx_force_pio = 0;
static int rx_force_pio = 0;

struct tegra_uart_port {
	struct uart_port	uport;
	char			port_name[32];

	/* Module info */
	unsigned int		modid;
	NvU32			irq;
	void __iomem		*regs;
	NvOsPhysAddr		phys;
	NvU32			size;
	struct clk		*clk;
	unsigned int		baud;

	/* Register shadow */
	unsigned char		fcr_shadow;
	unsigned char		mcr_shadow;
	unsigned char		lcr_shadow;
	unsigned char		ier_shadow;
	bool           		use_cts_control;
	bool            	rts_active;

	/* FIFO watermarks which decides when the interrupts are triggered(PIO
	 * mode).  In the case of receive it is high watermark and in the case
	 * of trasmit it is low watermark. */
	int			tx_low_watermark;
	int			rx_high_watermark;

	/* Tx DMA Buffer info */
	struct dma_pool		*tx_dma_pool;
	void			*tx_dma_virt[UART_TX_DMA_BUFFERS_TOTAL];
	dma_addr_t		tx_dma_phys[UART_TX_DMA_BUFFERS_TOTAL];
	int			tx_dma_size[UART_TX_DMA_BUFFERS_TOTAL];

	/* Rm DMA handles */
	int			tx_dma;

	/* DMA requests */
	struct tegra_dma_req	rx_dma_req;
	int			rx_dma;

	unsigned int		tx_dma_id;
	struct tegra_dma_req	tx_dma_req[UART_TX_DMA_BUFFERS_TOTAL];

	/* Rx PIO buffers */
	unsigned char		*rx_pio_buffer;
	int			rx_pio_buffer_size;

	bool			use_rx_dma;
	bool			use_tx_dma;

	bool			tx_pio_inflight;

	struct work_struct	work;
	struct workqueue_struct	*work_queue;

};

static void tegra_set_baudrate(struct tegra_uart_port *t, unsigned int baud);
static void tegra_set_mctrl(struct uart_port *u, unsigned int mctrl);
static void do_handle_rx_pio(struct uart_port *u);
static void set_rts(struct tegra_uart_port *t, bool active);
static void set_dtr(struct tegra_uart_port *t, bool active);

static inline int tegra_uart_isbreak(struct uart_port *u)
{
	struct tegra_uart_port *t;
	unsigned char lsr;

	t = container_of(u, struct tegra_uart_port, uport);
	lsr = readb(t->regs + UART_LSR_0);
	if (lsr & NV_DRF_DEF(UART, LSR, BRK, BREAK)) {
		/* If FIFO read error without any data, reset the Rx FIFO */
		lsr = readb(t->regs + UART_LSR_0);
		if (!(lsr & 0x1) && (lsr & 0x80))  {
			unsigned char fcr = t->fcr_shadow;
			fcr |= 0x2;
			writeb(fcr, t->regs + UART_IIR_FCR_0);
		}
		return 1;
	}
	return 0;
}

void tegra_rx_dma_threshold_callback(struct tegra_dma_req *req, int err)
{
	struct uart_port *u = req->data;
	struct tegra_uart_port *t;
	unsigned long flags;

	t = container_of(u, struct tegra_uart_port, uport);

	spin_lock_irqsave(&u->lock, flags);

	if (t->rts_active)
		set_rts(t, false);
	tegra_dma_dequeue(t->rx_dma);
	if (t->rts_active)
		set_rts(t, true);

	spin_unlock_irqrestore(&u->lock, flags);
}

/* It is expected that the callers take the UART lock when this API is called.
 *
 * There are 2 contexts when this function is called:
 *
 * 1. DMA ISR - DMA ISR triggers the threshold complete calback, which calls the
 * dequue API which in-turn calls this callback. UART lock is taken during
 * the call to the threshold callback.
 *
 * 2. UART ISR - UART calls the dequue API which in-turn will call this API.
 * In this case, UART ISR takes the UART lock.
 * */
void tegra_rx_dma_complete_callback(struct tegra_dma_req *req, int err)
{
	struct uart_port *u = req->data;
	struct tegra_uart_port *t;
	struct tty_struct *tty = u->info->port.tty;

	/* If we are here, DMA is stopped */

	t = container_of(u, struct tegra_uart_port, uport);
	if (req->bytes_transferred) {
		t->uport.icount.rx += req->bytes_transferred;
		tty_insert_flip_string(tty,
			((unsigned char *)(req->virt_addr)),
			req->bytes_transferred);
		dev_dbg(u->dev, "Received %d bytes\n", req->bytes_transferred);
	}

	if (req->status == -TEGRA_DMA_REQ_ERROR_ABOTRED) {
		do_handle_rx_pio(u);
	}

	spin_unlock(&u->lock);
	tty_flip_buffer_push(u->info->port.tty);
	spin_lock(&u->lock);

	/* Enqueue the request again */
	tegra_dma_enqueue_req(t->rx_dma, req);
}

/* Lock already taken */
static void do_handle_rx_dma(struct uart_port *u)
{
	struct tegra_uart_port *t;

	t = container_of(u, struct tegra_uart_port, uport);
	if (t->rts_active)
		set_rts(t, false);
	tegra_dma_dequeue(t->rx_dma);
	if (t->rts_active)
		set_rts(t, true);
}

static char do_decode_rx_error(struct uart_port *u)
{
	struct tegra_uart_port *t;
	char flag = TTY_NORMAL;
	unsigned char lsr;

	t = container_of(u, struct tegra_uart_port, uport);

	lsr = readb(t->regs + UART_LSR_0);
	if (lsr & NV_DRF_DEF(UART, LSR, OVRF, OVERRUN_ERROR)) {
		/* Overrrun error  */
		flag |= TTY_OVERRUN;
		t->uport.icount.overrun++;
		dev_err(u->dev, "Got overrun errors\n");
	} else if (lsr & NV_DRF_DEF(UART, LSR, PERR, PARITY_ERR)) {
		/* Parity error */
		flag |= TTY_PARITY;
		t->uport.icount.parity++;
		dev_err(u->dev, "Got Parity errors\n");
	} else if (lsr & NV_DRF_DEF(UART, LSR, FERR, FRAME_ERR)) {
		flag |= TTY_FRAME;
		dev_err(u->dev, "Got frame errors\n");
	} else if (lsr & NV_DRF_DEF(UART, LSR, BRK, BREAK)) {
		dev_err(u->dev, "Got Break \n");
		/* If FIFO read error without any data, reset Rx FIFO */
		if (!(lsr & 0x1) && (lsr & 0x80)) {
			unsigned char fcr = t->fcr_shadow;
			fcr |= 0x2;
			writeb(fcr, t->regs + UART_IIR_FCR_0);
		}
	}
	return flag;
}

static void do_handle_rx_pio(struct uart_port *u)
{
	struct tegra_uart_port *t;
	struct tty_struct *tty = u->info->port.tty;

	t = container_of(u, struct tegra_uart_port, uport);

	do {
		char flag = TTY_NORMAL;
		unsigned char lsr;
		unsigned char ch;

		flag =  do_decode_rx_error(u);

		lsr = readb(t->regs + UART_LSR_0);
		if (!(lsr & NV_DRF_DEF(UART, LSR, RDR, DATA_IN_FIFO)))
			break;

		ch = readb(t->regs + UART_THR_DLAB_0_0);
		dev_vdbg(u->dev, "%c\n", ch);
		if (!uart_handle_sysrq_char(u, c)) {
			tty_insert_flip_char(tty, ch, flag);
			t->uport.icount.rx ++;
		}
	} while (1);

	return;
}


static void tegra_tx_dma_workqueue(struct work_struct *w)
{
	struct uart_port *u;
	struct tegra_uart_port *t;
	struct circ_buf *xmit;
	unsigned int to_send;
	unsigned long flags;
	unsigned int i;

	t = container_of(w, struct tegra_uart_port, work);
	u = &t->uport;
	xmit = &u->info->xmit;

	spin_lock_irqsave(&u->lock, flags);

	t = container_of(u, struct tegra_uart_port, uport);

	/* Check next DMA if it is already queued, just return */
	if (tegra_dma_is_req_inflight(t->tx_dma,
	    &t->tx_dma_req[(t->tx_dma_id+1) % UART_TX_DMA_BUFFERS_TOTAL])) {
		spin_unlock_irqrestore(&u->lock, flags);
		return;
	}

	/* Update the DMA tail pointer */
	xmit->tail += t->tx_dma_req[t->tx_dma_id].size;
	xmit->tail &= UART_XMIT_SIZE - 1;
	u->icount.tx += t->tx_dma_req[t->tx_dma_id].size;
	t->tx_dma_req[t->tx_dma_id].size = 0;

	if (uart_circ_empty(xmit)) {
		spin_unlock_irqrestore(&u->lock, flags);
		return;
	}

	to_send = CIRC_CNT_TO_END(xmit->head, xmit->tail, UART_XMIT_SIZE);

	/* if transfer size is lees than a Limit use polling method
	 * to send the data to FIFO otherwise use DMA/interrupt Mode.
	 */

	if(to_send < t->tx_low_watermark) {
		unsigned int timeout;
		/* before writing to FIFO make sure all prev. DMAs finished */
		i = t->tx_dma_id;
		while(tegra_dma_is_req_inflight(t->tx_dma, &t->tx_dma_req[i])){
			spin_unlock_irqrestore(&u->lock, flags);
			msleep(1); /* sleep for some micro seconds*/
			spin_lock_irqsave(&u->lock, flags);
		}

		/*Before writing to FIFO, Make sure FIFO is empty*/
                timeout = 10;
		do {
			unsigned char lsr;
			lsr = readb(t->regs + UART_LSR_0);
			if ((lsr & TX_EMPTY_STATUS) == TX_EMPTY_STATUS)
				break;
			spin_unlock_irqrestore(&u->lock, flags);
			timeout--;
			if(timeout == 0){
				spin_unlock_irqrestore(&u->lock, flags);
				BUG();
				return;
			}
			msleep(1); /* sleep for some micro seconds*/
			spin_lock_irqsave(&u->lock, flags);
		} while (timeout);

		/* because we waited for FIFO to be empty and tx_low_watermark
		will always be less than FIFO depth, never overwrites FIFO*/
		while (to_send) {
			writeb(xmit->buf[xmit->tail],
				t->regs + UART_THR_DLAB_0_0);
			xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
			u->icount.tx++;
			to_send--;
		}
		spin_unlock_irqrestore(&u->lock, flags);
	}
	else
	{
		to_send &= ~0x3;
		if (to_send) {
			// Use and Fill the next DMA buffer and queue it.
			i = (t->tx_dma_id + 1) % UART_TX_DMA_BUFFERS_TOTAL;
			memcpy(t->tx_dma_virt[i],
				xmit->buf + xmit->tail, to_send);
			wmb();

			t->fcr_shadow = NV_FLD_SET_DRF_DEF(UART,
				IIR_FCR, TX_TRIG, FIFO_COUNT_GREATER_4,
				 t->fcr_shadow);
			writeb(t->fcr_shadow, t->regs + UART_IIR_FCR_0);

			t->tx_dma_req[i].source_addr = 	t->tx_dma_phys[i];
			t->tx_dma_req[i].size = to_send;
			t->tx_pio_inflight = false;

			tegra_dma_enqueue_req(t->tx_dma, &t->tx_dma_req[i]);
			t->tx_dma_id = i;
			spin_unlock_irqrestore(&u->lock, flags);

			/* Wakeup tty layer if we are below threshold.
			   Unlock port lock before we wakeup */
			if(uart_circ_chars_pending(xmit) > 0)
				uart_write_wakeup(u);

		} else {
			spin_unlock_irqrestore(&u->lock, flags);
			// We should never be here
			BUG();
		}
	}

	return;
}

static void do_handle_modem_signal(struct uart_port *u)
{
	unsigned char msr;
	struct tegra_uart_port *t;

	t = container_of(u, struct tegra_uart_port, uport);
	msr = readb(t->regs + UART_MSR_0);
	if (msr & (NV_DRF_DEF(UART, MSR, CTS, ENABLE))) {
		dev_dbg(u->dev, "CTS triggered\n");
	}
	if (msr & (NV_DRF_DEF(UART, MSR, DSR, ENABLE))) {
		dev_dbg(u->dev, "DSR enabled\n");
	}
	if (msr & (NV_DRF_DEF(UART, MSR, CD, ENABLE))) {
		dev_dbg(u->dev, "CD enabled\n");
	}
	if (msr & (NV_DRF_DEF(UART, MSR, RI, ENABLE))) {
		dev_dbg(u->dev, "RI enabled\n");
	}
	return;
}
static void do_handle_tx_pio(struct uart_port *u)
{
	struct tegra_uart_port *t;
	struct circ_buf *xmit = &u->info->xmit;
	int count = 0;

	t = container_of(u, struct tegra_uart_port, uport);
	/* As long as there is room in the FIFO write the buffer without
	 * polling fifo status register */
	while (count < t->tx_low_watermark) {
		if (uart_circ_empty(xmit)) {
			/* Disable interrupts on FIFO empty and break */
			t->ier_shadow = NV_FLD_SET_DRF_DEF(UART, IER_DLAB_0,
				IE_THR, DISABLE, t->ier_shadow);
			writeb(t->ier_shadow, t->regs + UART_IER_DLAB_0_0);
			t->tx_pio_inflight = false;
			break;
		}

		writeb(xmit->buf[xmit->tail], t->regs + UART_THR_DLAB_0_0);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		u->icount.tx++;
		count++;
	}
	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
			uart_write_wakeup(u);
	return;
}

void tegra_tx_dma_complete_callback(struct tegra_dma_req *req, int err)
{
	struct uart_port *u = (struct uart_port *)req->data;
	struct tegra_uart_port *t;
	if (err == -TEGRA_DMA_REQ_ERROR_ABOTRED)
		return;

	t  = container_of(u, struct tegra_uart_port, uport);
	queue_work(t->work_queue, &t->work);
}

static irqreturn_t tegra_uart_isr(int irq, void *data)
{
	struct uart_port *u = (struct uart_port *)data;
	struct tegra_uart_port *t;
	unsigned char iir_fcr;
	unsigned char ier;
	unsigned long flags;

	spin_lock_irqsave(&u->lock, flags);
	t  = container_of(u, struct tegra_uart_port, uport);
	/* FIXME why do we need to loop here? */
	while (1) {
		iir_fcr = readb(t->regs + UART_IIR_FCR_0);
		if (iir_fcr & NV_DRF_DEF(UART, IIR_FCR, IS_STA, NO_INTR_PEND)) {
			spin_unlock_irqrestore(&u->lock, flags);
			return IRQ_HANDLED;
		}

		// dev_vdbg(u->dev, "tegra_uart_isr iir = 0x%x\n", iir_fcr);
		switch ((iir_fcr >> 1 ) & 0x7) {
		case 0: /* Modem signal change interrupt */
			do_handle_modem_signal(u);
			break;
		case 1: /* Transmit interrupt only triggered when using PIO */
			do_handle_tx_pio(u);
			break;
		case 4: /* End of data */
			/* As per hw spec, to clear EORD interrupt, we need
			 * to disable and then re-enable the interrupt.
			 */
			ier = t->ier_shadow;
			ier = NV_FLD_SET_DRF_DEF(UART, IER_DLAB_0, IE_EORD,
				DISABLE, ier);
			writeb(ier, t->regs + UART_IER_DLAB_0_0);
			ier = NV_FLD_SET_DRF_DEF(UART, IER_DLAB_0, IE_EORD,
				ENABLE, ier);
			writeb(ier, t->regs + UART_IER_DLAB_0_0);
			/* fallthrough */
		case 2: /* Receive */
		case 6: /* Rx timeout */
			if (likely(t->use_rx_dma)) {
				do_handle_rx_dma(u);
			} else {
				do_handle_rx_pio(u);

				spin_unlock_irqrestore(&u->lock, flags);
				tty_flip_buffer_push(u->info->port.tty);
				spin_lock_irqsave(&u->lock, flags);
			}
			break;
		case 3: /* Receive error */
			/* FIXME how to handle this? Why do we get here */
			do_decode_rx_error(u);
			break;
		case 5: /* break nothing to handle */
		case 7: /* break nothing to handle */
			break;
		}
	}
}

static void tegra_stop_rx(struct uart_port *u)
{
	struct tegra_uart_port *t;

	t = container_of(u, struct tegra_uart_port, uport);

	if (t->rts_active)
		set_rts(t, false);
	tegra_dma_dequeue(t->rx_dma);

	return;
}

static void tegra_uart_hw_deinit(struct tegra_uart_port *t)
{
	unsigned char fcr;

	/* Disable interrupts */
	writeb(0, t->regs + UART_IER_DLAB_0_0);

	/* Reset the Rx and Tx FIFOs */
	fcr = t->fcr_shadow;
	fcr = NV_FLD_SET_DRF_DEF(UART, IIR_FCR, TX_CLR, CLEAR, fcr);
	fcr = NV_FLD_SET_DRF_DEF(UART, IIR_FCR, RX_CLR, CLEAR, fcr);
	writeb(fcr, t->regs + UART_IIR_FCR_0);

	NvRmModuleReset(s_hRmGlobal, t->modid);
	clk_disable(t->clk);
	NvRmSetModuleTristate(s_hRmGlobal, t->modid, NV_TRUE);
	t->baud = 0;
}

static int tegra_uart_hw_init(struct tegra_uart_port *t)
{
	unsigned char fcr;
	unsigned char ier;
	NvError err;

	dev_vdbg(t->uport.dev, "+tegra_uart_hw_init\n");

	t->fcr_shadow = 0;
	t->mcr_shadow = 0;
	t->lcr_shadow = 0;
	t->ier_shadow = 0;
	t->baud = 0;

	err = NvRmSetModuleTristate(s_hRmGlobal, t->modid, NV_FALSE);
	if (err != NvSuccess) {
		dev_err(t->uport.dev, "NvRmSetModuleTristate failed\n");
		goto fail;
	}

	clk_enable(t->clk);
	NvRmModuleReset(s_hRmGlobal, t->modid);

	/* Reset the FIFO twice with some delay to make sure that the FIFOs are
	 * really flushed. Wait is needed as the clearing needs to cross
	 * multiple clock domains.
	 * */
	t->fcr_shadow = NV_DRF_DEF(UART, IIR_FCR, FCR_EN_FIFO, ENABLE);

	fcr = t->fcr_shadow;
	fcr = NV_FLD_SET_DRF_DEF(UART, IIR_FCR, RX_CLR, CLEAR, fcr);
	fcr = NV_FLD_SET_DRF_DEF(UART, IIR_FCR, TX_CLR, CLEAR, fcr);
	writeb(fcr, t->regs + UART_IIR_FCR_0);

	udelay(100);
	writeb(fcr, t->regs + UART_IIR_FCR_0);
	udelay(100);

	/* Set the trigger level
	 *
	 * For PIO mode:
	 *
	 * For receive, this will interrupt the CPU after that many number of
	 * bytes are received, for the remaining bytes the receive timeout
	 * interrupt is received.
	 *
	 *  Rx high watermark is set to 4.
	 *
	 * For transmit, if the trasnmit interrupt is enabled, this will
	 * interrupt the CPU when the number of entries in the FIFO reaches the
	 * low watermark.
	 *
	 *  Tx low watermark is set to 8.
	 *
	 *  For DMA mode:
	 *
	 *  Set the Tx trigger to 4. This should match the DMA burst size that
	 *  programmed in the DMA registers.
	 * */
	t->fcr_shadow = NV_FLD_SET_DRF_DEF(UART, IIR_FCR, RX_TRIG,
		FIFO_COUNT_GREATER_4, t->fcr_shadow);

	if (t->use_tx_dma) {
		t->fcr_shadow = NV_FLD_SET_DRF_DEF(UART, IIR_FCR, TX_TRIG,
			FIFO_COUNT_GREATER_4, t->fcr_shadow);
	} else {
		t->fcr_shadow = NV_FLD_SET_DRF_DEF(UART, IIR_FCR, TX_TRIG,
			FIFO_COUNT_GREATER_8, t->fcr_shadow);
	}

	writeb(t->fcr_shadow, t->regs + UART_IIR_FCR_0);
	t->tx_low_watermark = 8;
	t->rx_high_watermark = 4;

	if (t->use_rx_dma)
		t->fcr_shadow = NV_FLD_SET_DRF_DEF(UART, IIR_FCR, DMA, CHANGE,
			t->fcr_shadow);
	else
		t->fcr_shadow = NV_FLD_SET_DRF_DEF(UART, IIR_FCR, DMA,
			NO_CHANGE, t->fcr_shadow);
	writeb(t->fcr_shadow, t->regs + UART_IIR_FCR_0);

	/*
	 *  Enable IE_RXS for the receive status interrupts like line errros.
	 *  Enable IE_RX_TIMEOUT to get the bytes which cannot be DMA'd.
	 *
	 *  If using DMA mode, enable EORD instead of receive interrupt which
	 *  will interrupt after the UART is done with the receive instead of
	 *  the interrupt when the FIFO "threshold" is reached.
	 *
	 *  EORD is different interrupt than RX_TIMEOUT - RX_TIMEOUT occurs when
	 *  the DATA is sitting in the FIFO and couldn't be transferred to the
	 *  DMA as the DMA size alignment(4 bytes) is not met. EORD will be
	 *  triggered when there is a pause of the incomming data stream for 4
	 *  characters long.
	 *
	 *  For pauses in the data which is not aligned to 4 bytes, we get
	 *  both the EORD as well as RX_TIMEOUT - SW sees RX_TIMEOUT first
	 *  then the EORD.
	 *
	 *  Don't get confused, believe in the magic of nvidia hw...:-)
	 */
	ier = 0;
	ier = NV_FLD_SET_DRF_DEF(UART, IER_DLAB_0, IE_RXS, ENABLE, ier);
	ier = NV_FLD_SET_DRF_DEF(UART, IER_DLAB_0, IE_RX_TIMEOUT, ENABLE, ier);
	if (t->use_rx_dma)
		ier = NV_FLD_SET_DRF_DEF(UART, IER_DLAB_0, IE_EORD, ENABLE,ier);
	else
		ier = NV_FLD_SET_DRF_DEF(UART, IER_DLAB_0, IE_RHR, ENABLE, ier);
	t->ier_shadow = ier;
	writeb(ier, t->regs + UART_IER_DLAB_0_0);

	dev_vdbg(t->uport.dev, "-tegra_uart_hw_init\n");
	return 0;

fail:
	dev_err(t->uport.dev, "HW init failed\n");
	return -ENODEV;
}

static int tegra_uart_init_rx_dma(struct tegra_uart_port *t)
{
	dma_addr_t rx_dma_phys;
	void *rx_dma_virt;

	t->rx_dma = tegra_dma_allocate_channel(TEGRA_DMA_MODE_CONTINOUS);
	if (t->rx_dma < 0)
		return -ENODEV;

	memset(&t->rx_dma_req, 0, sizeof(t->rx_dma_req));

	t->rx_dma_req.size = UART_RX_DMA_BUFFER_SIZE;
	rx_dma_virt = dma_alloc_coherent(t->uport.dev,
		t->rx_dma_req.size, &rx_dma_phys, GFP_KERNEL);
	if (!rx_dma_virt) {
		dev_err(t->uport.dev, "DMA buffers allocate failed \n");
		goto fail;
	}
	t->rx_dma_req.dest_addr = rx_dma_phys;
	t->rx_dma_req.virt_addr = rx_dma_virt;

	t->rx_dma_req.source_addr = t->phys;
	t->rx_dma_req.source_wrap = 4;
	t->rx_dma_req.dest_wrap = 0;
	t->rx_dma_req.to_memory = 1;
	t->rx_dma_req.modid =  NvRmModuleID_Uart;
	t->rx_dma_req.instance = t->uport.line;
	t->rx_dma_req.complete = tegra_rx_dma_complete_callback;
	t->rx_dma_req.threshold = tegra_rx_dma_threshold_callback;
	t->rx_dma_req.data = &t->uport;
	INIT_LIST_HEAD(&(t->rx_dma_req.list));
	if (tegra_dma_enqueue_req(t->rx_dma, &t->rx_dma_req)) {
		dev_err(t->uport.dev, "Could not enqueue Rx DMA req\n");
		goto fail;
	}

	return 0;
fail:
	tegra_dma_free_channel(t->rx_dma);
	if (t->rx_dma_req.dest_addr)
		dma_free_coherent(t->uport.dev, t->rx_dma_req.size,
			t->rx_dma_req.virt_addr, t->rx_dma_req.dest_addr);
	return -ENODEV;
}

static int tegra_startup(struct uart_port *u)
{
	struct tegra_uart_port *t = container_of(u,
		struct tegra_uart_port, uport);
	int ret = 0;
	unsigned int i;

	t = container_of(u, struct tegra_uart_port, uport);
	sprintf(t->port_name, "tegra_uart_%d", u->line);

	NvRmModuleGetBaseAddress(s_hRmGlobal, t->modid, &t->phys, &t->size);
	t->regs = ioremap_nocache(t->phys, t->size);
	if (!t->regs) {
		dev_err(u->dev, "Cannot map UART registers\n");
		return -ENODEV;
	}
	t->irq = NvRmGetIrqForLogicalInterrupt(s_hRmGlobal, t->modid, 0);
	BUG_ON(t->irq == (NvU32)(-1));

	t->use_tx_dma = false;
	if (!tx_force_pio) {
		t->tx_dma = tegra_dma_allocate_channel(TEGRA_DMA_MODE_ONESHOT);
		if (t->tx_dma >= 0)
			t->use_tx_dma = true;
	}
	if (t->use_tx_dma) {
		/* alloc resources */
		t->tx_dma_pool = dma_pool_create(t->port_name, t->uport.dev,
			UART_XMIT_SIZE, 64, PAGE_SIZE);

		for(i=0; i<UART_TX_DMA_BUFFERS_TOTAL; i++) {
			/* setup */
			t->tx_dma_virt[i] = dma_pool_alloc(t->tx_dma_pool,
				GFP_KERNEL, &t->tx_dma_phys[i]);

			if (t->tx_dma_virt[i]) {
				/* Setup the DMA Tx request structure
				 * which doesn't change */
				INIT_LIST_HEAD(&t->tx_dma_req[i].list);
				t->tx_dma_size[i] = UART_XMIT_SIZE;
				t->tx_dma_req[i].modid =  NvRmModuleID_Uart;
				t->tx_dma_req[i].instance = u->line;
				t->tx_dma_req[i].complete =
						tegra_tx_dma_complete_callback;
				t->tx_dma_req[i].to_memory = 0;

				t->tx_dma_req[i].dest_addr = t->phys;
				t->tx_dma_req[i].dest_wrap = 4;
				t->tx_dma_req[i].source_wrap = 0;
				t->tx_dma_req[i].data = u;
				t->tx_dma_req[i].size = 0;
			} else {
				tegra_dma_free_channel(t->tx_dma);
				t->use_tx_dma = false;
			}
		}
		t->tx_dma_id = 0;
	}

	t->use_rx_dma = false;
	if (!rx_force_pio) {
		if (!tegra_uart_init_rx_dma(t))
			t->use_rx_dma = true;
	}
	ret = tegra_uart_hw_init(t);
	if (ret)
		goto fail;

	ret = request_irq(t->irq, tegra_uart_isr, IRQF_SHARED, t->port_name, u);
	if (ret) {
		dev_err(u->dev, "Failed to register ISR for IRQ %d\n", t->irq);
		goto fail;
	}
	/* Set the irq flags to irq valid, which is the default linux behaviour.
	 * For irqs used by Nv* APIs, IRQF_NOAUTOEN is also set */
	set_irq_flags(t->irq, IRQF_VALID);
	dev_info(u->dev,"Started UART port %d\n", u->line);

	return 0;
fail:
	dev_err(u->dev, "Tegra UART startup failed\n");
	return ret;
}

static void tegra_shutdown(struct uart_port *u)
{
	struct tegra_uart_port *t;
	unsigned long flags;
	unsigned int i;

	spin_lock_irqsave(&u->lock, flags);
	t = container_of(u, struct tegra_uart_port, uport);
	dev_vdbg(u->dev, "+tegra_shutdown\n");

	tegra_uart_hw_deinit(t);
	spin_unlock_irqrestore(&u->lock, flags);

	if (t->use_rx_dma) {
		dma_free_coherent(u->dev, t->rx_dma_req.size,
			t->rx_dma_req.virt_addr, t->rx_dma_req.dest_addr);
		tegra_dma_free_channel(t->rx_dma);
	}
	if (t->use_tx_dma) {
		for(i=0; i< UART_TX_DMA_BUFFERS_TOTAL; i++) {
			dma_pool_free(t->tx_dma_pool, t->tx_dma_virt[i],
							t->tx_dma_phys[i]);
		}
		dma_pool_destroy(t->tx_dma_pool);
		tegra_dma_free_channel(t->tx_dma);
	}

	free_irq(t->irq, u);
	dev_vdbg(u->dev, "-tegra_shutdown\n");
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

static void set_rts(struct tegra_uart_port *t, bool active)
{
	unsigned char mcr;
	mcr = t->mcr_shadow;
	if (active)
		mcr = NV_FLD_SET_DRF_DEF(UART, MCR, RTS, FORCE_RTS_LOW, mcr);
	else
		mcr = NV_FLD_SET_DRF_DEF(UART, MCR, RTS, FORCE_RTS_HI, mcr);
	if (mcr != t->mcr_shadow) {
		writeb(mcr, t->regs + UART_MCR_0);
		t->mcr_shadow = mcr;
	}
	return;
}

static void set_dtr(struct tegra_uart_port *t, bool active)
{
	unsigned char mcr;
	mcr = t->mcr_shadow;
	if (active)
		mcr = NV_FLD_SET_DRF_DEF(UART, MCR, DTR, FORCE_DTR_LOW, mcr);
	else
		mcr = NV_FLD_SET_DRF_DEF(UART, MCR, DTR, FORCE_DTR_HI, mcr);
	if (mcr != t->mcr_shadow) {
		writeb(mcr, t->regs + UART_MCR_0);
		t->mcr_shadow = mcr;
	}
	return;
}

static void tegra_set_mctrl(struct uart_port *u, unsigned int mctrl)
{
	unsigned char mcr;
	struct tegra_uart_port *t;

	dev_vdbg(u->dev, "tegra_set_mctrl called with %d\n", mctrl);
	t = container_of(u, struct tegra_uart_port, uport);

	mcr = t->mcr_shadow;
	if (mctrl & TIOCM_RTS) {
		t->rts_active = true;
		set_rts(t, true);
	} else {
		t->rts_active = false;
		set_rts(t, false);
	}

	if (mctrl & TIOCM_DTR)
		set_dtr(t, true);
	else
		set_dtr(t, false);
	return;
}

static void tegra_break_ctl(struct uart_port *u, int break_ctl)
{
	struct tegra_uart_port *t;
	unsigned char lcr;

	t = container_of(u, struct tegra_uart_port, uport);
	lcr = t->lcr_shadow;
	if (break_ctl)
		lcr = NV_FLD_SET_DRF_DEF(UART, LCR, SET_B, BREAK, lcr);
	else
		lcr = NV_FLD_SET_DRF_DEF(UART, LCR, SET_B, NO_BREAK, lcr);
	writeb(lcr, t->regs + UART_LCR_0);
	t->lcr_shadow = lcr;
}

static int tegra_request_port(struct uart_port *u)
{
	return 0;
}

static void tegra_release_port(struct uart_port *u)
{


}

static unsigned int tegra_tx_empty(struct uart_port *u)
{
	struct tegra_uart_port *t;
	unsigned char lsr;
	unsigned int ret;

	t = container_of(u, struct tegra_uart_port, uport);
	dev_vdbg(u->dev, "+tegra_tx_empty\n");

	if (t->use_tx_dma &&
	    tegra_dma_is_req_inflight(t->tx_dma, &t->tx_dma_req[t->tx_dma_id]))
		return 0;

	lsr = readb(t->regs + UART_LSR_0);
	if ((lsr & TX_EMPTY_STATUS) == TX_EMPTY_STATUS)
		ret = TIOCSER_TEMT;
	else
		ret = 0;
	dev_vdbg(u->dev, "-tegra_tx_empty\n");
	return 0;
}

static void tegra_stop_tx(struct uart_port *u)
{
	struct tegra_uart_port *t;
	unsigned int i;

	t = container_of(u, struct tegra_uart_port, uport);

	if (t->use_tx_dma) {
		i = (t->tx_dma_id+1) % UART_TX_DMA_BUFFERS_TOTAL;
		while (i!= t->tx_dma_id) {
			tegra_dma_dequeue_req(t->tx_dma, &t->tx_dma_req[i]);
			t->tx_dma_req[i].size = 0;
			i = (i+1) % UART_TX_DMA_BUFFERS_TOTAL;
		}
	}
	return;
}

static void tegra_start_tx_locked(struct uart_port *u)
{
	struct tegra_uart_port *t;

	t = container_of(u, struct tegra_uart_port, uport);

	dev_vdbg(t->uport.dev, "+tegra_start_tx_locked\n");

	if (!t->use_tx_dma) {
		/* Enable interrupt on transmit FIFO empty, if it is disabled */
		if (!t->tx_pio_inflight) {
			t->tx_pio_inflight = true;
			t->ier_shadow = NV_FLD_SET_DRF_DEF(UART, IER_DLAB_0,
				IE_THR, ENABLE, t->ier_shadow);
			writeb(t->ier_shadow, t->regs + UART_IER_DLAB_0_0);
		}
	} else {
		queue_work(t->work_queue, &t->work);
	}
	dev_vdbg(t->uport.dev, "-tegra_start_tx_locked\n");
}

static void tegra_enable_ms(struct uart_port *u)
{
}

#define UART_CLOCK_ACCURACY 5

static void tegra_set_baudrate(struct tegra_uart_port *t, unsigned int baud)
{
	unsigned int clock;
	unsigned long long minclock;
	unsigned long long maxclock;
	unsigned int actual_clock;
	unsigned int divisor;
	NvError err;
	unsigned char lcr;

	if (t->baud == baud)
		return;

	clock =  (baud * 16) / 1000;
	clock = clock ? clock : 1;

	minclock = (unsigned long long) baud * 16 * (100 - UART_CLOCK_ACCURACY);
	do_div(minclock, 1000 * 100);

	maxclock = (unsigned long long) baud * 16 * (100 + UART_CLOCK_ACCURACY);
	do_div(maxclock, 1000 * 100);

	err = NvRmPowerModuleClockConfig(s_hRmGlobal,
		t->modid, 0, (NvU32)minclock, (NvU32)maxclock,
		&clock, 1, &actual_clock, 0);
	if (err != NvSuccess) {
		dev_err(t->uport.dev, "Setting the UART clock failed min "
			"%lld ideal %d max %lld\n", minclock, clock, maxclock);
		return;
	}

	divisor = (actual_clock * 1000);
	do_div(divisor, 16);
	divisor += baud/2;
	do_div(divisor, baud);

	lcr = t->lcr_shadow;
	lcr = NV_FLD_SET_DRF_DEF(UART, LCR, DLAB, ENABLE, lcr);
	writeb(lcr, t->regs + UART_LCR_0);

	writel(divisor & 0xFF, t->regs + UART_THR_DLAB_0_0);
	writel(((divisor >> 8) & 0xFF), t->regs + UART_IER_DLAB_0_0);

	lcr = NV_FLD_SET_DRF_DEF(UART, LCR, DLAB, DISABLE, lcr);
	writeb(lcr, t->regs + UART_LCR_0);

	t->baud = baud;
	dev_info(t->uport.dev, "Baud %d clock freq %d and divisor of %d\n",
		baud, actual_clock, divisor);
}

void tegra_set_termios(struct uart_port *u, struct ktermios *termios,
					   struct ktermios *oldtermios)
{
	struct tegra_uart_port *t;
	unsigned int baud;
	unsigned long flags;
	unsigned int lcr;
	unsigned int c_cflag = termios->c_cflag;
	char debug_string[50];
	unsigned char mcr;

	t = container_of(u, struct tegra_uart_port, uport);
	dev_vdbg(t->uport.dev, "+tegra_set_termios\n");
	debug_string[0] = 0;
	strlcat(debug_string, "uart port setting: ", 50);

	spin_lock_irqsave(&u->lock, flags);

	/* Baud rate */
	baud = uart_get_baud_rate(u, termios, oldtermios, 200, 4000000);
	tegra_set_baudrate(t, baud);

	/* Parity */
	lcr = t->lcr_shadow;
	lcr = NV_FLD_SET_DRF_DEF(UART, LCR, PAR, NO_PARITY, lcr);
	if (PARENB == (c_cflag & PARENB)) {
		if (CMSPAR == (c_cflag & CMSPAR)) {
			strlcat(debug_string, "space parity ", 50);
			/* FIXME What is space parity? */
			/* data |= SPACE_PARITY; */
		} else if (c_cflag & PARODD) {
			strlcat(debug_string, "ODD parity ", 50);
			lcr = NV_FLD_SET_DRF_DEF(UART, LCR, PAR, PARITY, lcr);
			lcr = NV_FLD_SET_DRF_DEF(UART, LCR, EVEN, DISABLE, lcr);
			lcr = NV_FLD_SET_DRF_DEF(UART, LCR, SET_P, NO_PARITY,
				lcr);
		} else {
			strlcat(debug_string, "Even parity ", 50);
			lcr = NV_FLD_SET_DRF_DEF(UART, LCR, PAR, PARITY, lcr);
			lcr = NV_FLD_SET_DRF_DEF(UART, LCR, EVEN, ENABLE, lcr);
			lcr = NV_FLD_SET_DRF_DEF(UART, LCR, SET_P, NO_PARITY,
				lcr);
		}
	}

	switch (c_cflag & CSIZE) {
	case CS5:
		lcr = NV_FLD_SET_DRF_DEF(UART, LCR, WD_SIZE, WORD_LENGTH_5,lcr);
		strlcat(debug_string, "5", 50);
		break;
	case CS6:
		lcr = NV_FLD_SET_DRF_DEF(UART, LCR, WD_SIZE, WORD_LENGTH_6,lcr);
		strlcat(debug_string, "6", 50);
		break;
	case CS7:
		lcr = NV_FLD_SET_DRF_DEF(UART, LCR, WD_SIZE, WORD_LENGTH_7,lcr);
		strlcat(debug_string, "7", 50);
		break;
	default:
		lcr = NV_FLD_SET_DRF_DEF(UART, LCR, WD_SIZE, WORD_LENGTH_8,lcr);
		strlcat(debug_string, "8", 50);
		break;
	}

	/* Stop bits */
	if (termios->c_cflag & CSTOPB) {
		lcr = NV_FLD_SET_DRF_DEF(UART, LCR, STOP, ENABLE, lcr);
		strlcat(debug_string, "n2", 50);
	} else {
		lcr = NV_FLD_SET_DRF_DEF(UART, LCR, STOP, DISABLE, lcr);
		strlcat(debug_string, "n1", 50);
	}

	writeb(lcr, t->regs + UART_LCR_0);
	t->lcr_shadow = lcr;

	/* Flow control */
	if (termios->c_cflag & CRTSCTS)	{
		mcr = t->mcr_shadow;
		mcr = NV_FLD_SET_DRF_DEF(UART, MCR, CTS_EN, ENABLE, mcr);
		mcr = NV_FLD_SET_DRF_DEF(UART, MCR, RTS_EN, DISABLE, mcr);
		t->mcr_shadow = mcr;
		writeb(mcr, t->regs + UART_MCR_0);
		t->use_cts_control = true;
	} else {
		mcr = t->mcr_shadow;
		mcr = NV_FLD_SET_DRF_DEF(UART, MCR, CTS_EN, DISABLE, mcr);
		mcr = NV_FLD_SET_DRF_DEF(UART, MCR, RTS_EN, DISABLE, mcr);
		t->mcr_shadow = mcr;
		writeb(mcr, t->regs + UART_MCR_0);
		t->use_cts_control = false;
	}

	spin_unlock_irqrestore(&u->lock, flags);
	dev_info(u->dev, "%s\n", debug_string);
	dev_vdbg(t->uport.dev, "-tegra_set_termios\n");
	return;
}

/*
 * Flush any TX data submitted for DMA. Called when the TX circular
 * buffer is reset.
 */
static void tegra_flush_buffer(struct uart_port *u)
{
	struct tegra_uart_port *t;
	unsigned int i;

	dev_vdbg(u->dev, "tegra_flush_buffer called");

	t = container_of(u, struct tegra_uart_port, uport);

	if (t->use_tx_dma) {
		i = (t->tx_dma_id + 1) % UART_TX_DMA_BUFFERS_TOTAL;
		while (i != t->tx_dma_id) {
			tegra_dma_dequeue_req(t->tx_dma, &t->tx_dma_req[i]);
			t->tx_dma_req[i].size = 0;
			i = (i + 1) % UART_TX_DMA_BUFFERS_TOTAL;
		}
	}
	return;
}


static void tegra_pm(struct uart_port *u, unsigned int state,
	unsigned int oldstate) {

}

static const char *tegra_type(struct uart_port *u) {
	return 0;
}

static struct uart_ops tegra_uart_ops = {
	.tx_empty	= tegra_tx_empty,
	.set_mctrl	= tegra_set_mctrl,
	.get_mctrl	= tegra_get_mctrl,
	.stop_tx	= tegra_stop_tx,
	.start_tx	= tegra_start_tx_locked,
	.stop_rx	= tegra_stop_rx,
	.flush_buffer	= tegra_flush_buffer,
	.enable_ms	= tegra_enable_ms,
	.break_ctl	= tegra_break_ctl,
	.startup	= tegra_startup,
	.shutdown	= tegra_shutdown,
	.set_termios	= tegra_set_termios,
	.pm		= tegra_pm,
	.type		= tegra_type,
	.request_port	= tegra_request_port,
	.release_port	= tegra_release_port,
};

static int __init tegra_uart_probe(struct platform_device *pdev);
static int __devexit tegra_uart_remove(struct platform_device *pdev);
static int tegra_uart_suspend(struct platform_device *pdev, pm_message_t state);
static int tegra_uart_resume(struct platform_device *pdev);

static struct platform_driver tegra_uart_platform_driver = {
	.remove		= tegra_uart_remove,
	.probe		= tegra_uart_probe,
	.suspend	= tegra_uart_suspend,
	.resume		= tegra_uart_resume,
	.driver		= {
		.name	= "tegra_uart"
	}
};

static struct uart_driver tegra_uart_driver =
{
	.owner		= THIS_MODULE,
	.driver_name	= "tegra_uart",
	.dev_name	= "ttyHS",
	.cons		= 0,
};

static int tegra_uart_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct tegra_uart_port *t = platform_get_drvdata(pdev);
	struct uart_port *u;

	if (pdev->id < 0 || pdev->id > tegra_uart_driver.nr) {
		printk(KERN_ERR "Invalid Uart instance (%d) \n", pdev->id);
	}

	dev_err(t->uport.dev, "tegra_uart_suspend called \n");
	u = &t->uport;
	uart_suspend_port(&tegra_uart_driver, u);
	return 0;
}

static int tegra_uart_resume(struct platform_device *pdev)
{
	struct tegra_uart_port *t = platform_get_drvdata(pdev);
	struct uart_port *u;

	if (pdev->id < 0 || pdev->id > tegra_uart_driver.nr) {
		printk(KERN_ERR "Invalid Uart instance (%d) \n", pdev->id);
	}

	u = &t->uport;
	dev_err(t->uport.dev, "tegra_uart_resume called \n");
	uart_resume_port(&tegra_uart_driver, u);
	return 0;
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
	destroy_workqueue(t->work_queue);

	platform_set_drvdata(pdev, NULL);

	NvRmSetModuleTristate(s_hRmGlobal, t->modid, NV_TRUE);

	printk(KERN_INFO "Unregistered UART port %s%d\n",
		tegra_uart_driver.dev_name, u->line);
	kfree(t);
	return 0;
}

#define MAX_CLK_NAME_CHARS 48
static int __init tegra_uart_probe(struct platform_device *pdev)
{
	struct tegra_uart_port *t;
	struct uart_port *u;
	int ret;
	char clk_name[MAX_CLK_NAME_CHARS];
	char name[64];
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
	t->modid = NVRM_MODULE_ID(NvRmModuleID_Uart, u->line);

	snprintf(clk_name, MAX_CLK_NAME_CHARS, "mod=uart,inst=%d", pdev->id);
	t->clk = clk_get(&pdev->dev, clk_name);
	if (!t->clk) {
		dev_err(&pdev->dev, "Couldn't get the clock bailing out\n");
		goto fail;
	}

	if (NvRmSetModuleTristate(s_hRmGlobal,
		t->modid, NV_FALSE) != NvSuccess) {
		dev_err(u->dev, "No Pin Mux - not registering the port\n");
		goto fail;
	}
	/* Tristate the pins in boot. Drive it only when using the pins */
	NvRmSetModuleTristate(s_hRmGlobal, t->modid, NV_TRUE);

	ret = uart_add_one_port(&tegra_uart_driver, u);
	if (ret) {
		printk(KERN_INFO "%s: Failed(%d) to add uart port %s%d\n",
			__func__, ret, tegra_uart_driver.dev_name, u->line);
		kfree(t);
		platform_set_drvdata(pdev, NULL);
		return ret;
	}

	snprintf(name, sizeof(name), "tegra_hsuart_%d", u->line);
	t->work_queue = create_singlethread_workqueue(name);
	if (t->work_queue == NULL) {
		dev_err(u->dev, "Failed to create work queue\n");
		goto fail;
	}
	INIT_WORK(&t->work, tegra_tx_dma_workqueue);
	printk(KERN_INFO "Registered UART port %s%d\n",
		tegra_uart_driver.dev_name, u->line);
	return ret;
fail:
	kfree(t);
	return -ENODEV;
}

static int __init tegra_uart_init(void)
{
	int ret;

	tegra_uart_driver.nr  = NvRmModuleGetNumInstances(s_hRmGlobal,
		NvRmModuleID_Uart);

	ret = uart_register_driver(&tegra_uart_driver);
	if (unlikely(ret)) {
		printk(KERN_ERR "Could not register %s driver\n",
			tegra_uart_driver.driver_name);
		return ret;
	}

	ret = platform_driver_register(&tegra_uart_platform_driver);
	if (unlikely(ret)) {
		printk(KERN_ERR "Could not register the UART platfrom "
			"driver\n");
		uart_unregister_driver(&tegra_uart_driver);
		return ret;
	}

	printk(KERN_INFO "Initialized tegra uart driver\n");
	return 0;
}

static void __exit tegra_uart_exit(void)
{
	printk(KERN_INFO "Unloading tegra uart driver\n");
	platform_driver_unregister(&tegra_uart_platform_driver);
	uart_unregister_driver(&tegra_uart_driver);
}

module_init(tegra_uart_init);
module_exit(tegra_uart_exit);
MODULE_DESCRIPTION("High speed UART driver for tegra chipset");
