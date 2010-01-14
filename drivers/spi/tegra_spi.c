/*
 * drivers/spi/tegra_spi.c
 *
 * SPI bus driver for NVIDIA Tegra SoCs, based on NvRm APIs
 *
 * Copyright (c) 2009, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/workqueue.h>
#include <linux/tegra_devices.h>


#include "mach/nvrm_linux.h"
#include "mach/io.h"
#include "nvrm_spi.h"
#include "nvodm_query.h"

/* FIXME, don't include the Rm private API  */
#include "rm_spi_slink.h"

/* Cannot use spinlocks as the NvRm SPI apis uses mutextes and one cannot use
 * mutextes inside a spinlock.
 */
#define USE_SPINLOCK 0
#if USE_SPINLOCK
#define LOCK_T          spinlock_t
#define CREATELOCK(_l)  spin_lock_init(&(_l))
#define DELETELOCK(_l)
#define LOCK(_l)        spin_lock(&(_l))
#define UNLOCK(_l)      spin_unlock(&(_l))
#define ATOMIC(_l,_f)   spin_lock_irqsave(&(_l),(_f))
#define UNATOMIC(_l,_f) spin_unlock_irqrestore(&(_l),(_f))
#else
#define LOCK_T          struct mutex
#define CREATELOCK(_l)  mutex_init(&(_l))
#define DELETELOCK(_l) 
#define LOCK(_l)        mutex_lock(&(_l))
#define UNLOCK(_l)      mutex_unlock(&(_l))
#define ATOMIC(_l,_f)   local_irq_save((_f))
#define UNATOMIC(_l,_f) local_irq_restore((_f))
#endif

struct NvSpiShim {
	NvRmSpiHandle		hSpi;
	NvU32			IoModuleID;
	NvU32			Instance;
	NvU32			ChipSelect;
	NvU32			PinMuxConfig;

	NvU32			Mode;
	NvU32			BitsPerWord;
	NvU32			ClockInKHz;

	struct list_head	msg_queue;
	LOCK_T			lock;
	struct work_struct	work;
	struct workqueue_struct	*WorkQueue;
}; 

/* Only these signaling mode are supported */
#define NV_SUPPORTED_MODE_BITS (SPI_CPOL | SPI_CPHA)

static int tegra_spi_setup(struct spi_device *pSpiDevice)
{
	struct NvSpiShim  *pShimSpi;

	pShimSpi = spi_master_get_devdata(pSpiDevice->master);

	if (pSpiDevice->mode & ~NV_SUPPORTED_MODE_BITS) {
		dev_dbg(&pSpiDevice->dev, "setup: unsupported mode bits 0x%x\n",
			pSpiDevice->mode & ~NV_SUPPORTED_MODE_BITS);
	}

	pShimSpi->Mode = pSpiDevice->mode & ~NV_SUPPORTED_MODE_BITS;
	switch (pShimSpi->Mode) {
	case SPI_MODE_0:
		pShimSpi->Mode = NvOdmQuerySpiSignalMode_0;
		break;
	case SPI_MODE_1:
		pShimSpi->Mode = NvOdmQuerySpiSignalMode_1;
		break;
	case SPI_MODE_2:
		pShimSpi->Mode = NvOdmQuerySpiSignalMode_2;
		break;
	case SPI_MODE_3:
		pShimSpi->Mode = NvOdmQuerySpiSignalMode_3;
		break;
	}

	if (pSpiDevice->bits_per_word == 0)
		pSpiDevice->bits_per_word = 8;

	pShimSpi->BitsPerWord = pSpiDevice->bits_per_word;
	pShimSpi->ClockInKHz = pSpiDevice->max_speed_hz / 1000;
	pShimSpi->ChipSelect = pSpiDevice->chip_select;

	NvRmSpiSetSignalMode(pShimSpi->hSpi, pShimSpi->ChipSelect, 
		pShimSpi->Mode);
	return 0;
}

static int tegra_spi_transfer(struct spi_device *pSpiDevice,
	struct spi_message *msg)
{
	struct NvSpiShim  *pShimSpi;

	if (unlikely(list_empty(&msg->transfers) ||
		!pSpiDevice->max_speed_hz))
		return -EINVAL;

	/* FIXME validate the msg */

	pShimSpi = spi_master_get_devdata(pSpiDevice->master);

	/* Add the message to the queue and signal the worker thread */
	LOCK(pShimSpi->lock);
	list_add_tail(&msg->queue, &pShimSpi->msg_queue);
	queue_work(pShimSpi->WorkQueue, &pShimSpi->work);
	UNLOCK(pShimSpi->lock);

	return 0;
}

static void tegra_spi_cleanup(struct spi_device *pSpiDevice)
{
	/* Nothing to do. NvRmSpi API is stateless. 
	 * Isn't that bueatifull?
	 */
	return;
}

static void tegra_spi_workerthread(struct work_struct *w)
{
	struct NvSpiShim *pShimSpi;

	pShimSpi = container_of(w, struct NvSpiShim, work);

	LOCK(pShimSpi->lock);

	while (!list_empty(&pShimSpi->msg_queue)) {
		struct spi_message *m;
		struct spi_transfer *t = NULL;
		int status = 0;
		NvRmSpiTransactionInfo trans[64];
		NvU32 i= 0;
		NvU32  actual_length = 0;	 

		m = container_of(pShimSpi->msg_queue.next, 
			struct spi_message, queue);

		list_del_init(&m->queue);

		UNLOCK(pShimSpi->lock);

		list_for_each_entry(t, &m->transfers, transfer_list) {
			if (i==64) {
				status = -EINVAL;
				break;
			}
			if (t->tx_buf == NULL && t->rx_buf == NULL && t->len) {
				status = -EINVAL;
				break;
			}
			if (t->len) {
				trans[i].rxBuffer = t->rx_buf;
				trans[i].txBuffer = (NvU8 *)t->tx_buf;
				trans[i].len = t->len;
				actual_length += t->len;
			}
			if (t->cs_change) {
				/* What should we do here? */
			}
			i++;
		}

		if (!status && i) {
			NvRmSpiMultipleTransactions(pShimSpi->hSpi,
				pShimSpi->PinMuxConfig, pShimSpi->ChipSelect,
				pShimSpi->ClockInKHz, pShimSpi->BitsPerWord,
				trans, i);

			m->actual_length += actual_length;
		}

		m->status = status;
		m->complete(m->context);

		LOCK(pShimSpi->lock);
	}

	UNLOCK(pShimSpi->lock);
}

static int __init tegra_spi_probe(struct platform_device *pdev)
{
	struct spi_master	*pSpi;
	struct NvSpiShim	*pShimSpi;
	struct tegra_spi_platform_data *pdata = pdev->dev.platform_data;
	int			status= 0;
	NvError			err;
	char			name[64];

	pSpi = spi_alloc_master(&pdev->dev, sizeof *pShimSpi);
	if (pSpi == NULL) {
		dev_dbg(&pdev->dev, "master allocation failed\n");
		return -ENOMEM;
	}

	pSpi->setup = tegra_spi_setup;
	pSpi->transfer = tegra_spi_transfer;
	pSpi->cleanup = tegra_spi_cleanup;
	pSpi->num_chipselect = 4;
	pSpi->bus_num = pdev->id;

	dev_set_drvdata(&pdev->dev, pSpi);
	pShimSpi = spi_master_get_devdata(pSpi);
	pShimSpi->IoModuleID = pdata->IoModuleID;
	pShimSpi->Instance = pdata->Instance;
	pShimSpi->PinMuxConfig = pdata->PinMuxConfig;
 
	err = NvRmSpiOpen(s_hRmGlobal, pShimSpi->IoModuleID,
		pShimSpi->Instance, NV_TRUE, &pShimSpi->hSpi);
	if (err != NvSuccess) {
		dev_dbg(&pdev->dev, "NvRmSpiOpen failed (%d)\n", err);
		goto spi_open_failed;
	}

	/* FIXME: Do we need one workqueue for each instance or one 
	 * workqueue for all instances will do?
	 */
	snprintf(name, sizeof(name), "%s_%d", "tegra_spi_wq", 
		pdata->IoModuleID || pdata->Instance << 16);
	pShimSpi->WorkQueue = create_singlethread_workqueue(name);
	if (pShimSpi->WorkQueue == NULL) {
		dev_err(&pdev->dev, "Failed to create work queue\n");
		goto workQueueCreate_failed;
	}

	INIT_WORK(&pShimSpi->work, tegra_spi_workerthread);

	CREATELOCK(pShimSpi->lock);
	INIT_LIST_HEAD(&pShimSpi->msg_queue);

	status = spi_register_master(pSpi);
	if (status < 0) {
		dev_err(&pdev->dev, "spi_register_master failed %d\n", status);
		goto spi_register_failed;
	}

	dev_info(&pdev->dev, 
		"Registered spi device for mod,inst,mux (%d,%d,%d)\n", 
		pShimSpi->IoModuleID, pShimSpi->Instance,
		pShimSpi->PinMuxConfig);

	return status;
	
spi_register_failed:
	destroy_workqueue(pShimSpi->WorkQueue);
workQueueCreate_failed:
	NvRmSpiClose(pShimSpi->hSpi);
spi_open_failed:
	spi_master_put(pSpi);
	return status;
}
	
static int __exit tegra_spi_remove(struct platform_device *pdev)
{
	struct spi_master	*pSpi;
	struct NvSpiShim	*pShimSpi;

	pSpi = dev_get_drvdata(&pdev->dev);
	pShimSpi = spi_master_get_devdata(pSpi);

	spi_unregister_master(pSpi);
	NvRmSpiClose(pShimSpi->hSpi);
	destroy_workqueue(pShimSpi->WorkQueue);

	return 0;
}

static struct platform_driver tegra_spi_driver = {
	.driver	= {
		.name	= "tegra_spi",
		.owner	= THIS_MODULE,
	},
	.remove	= __exit_p(tegra_spi_remove),
};

static int __init tegra_spi_init(void)
{
	return platform_driver_probe(&tegra_spi_driver, tegra_spi_probe);
}
module_init(tegra_spi_init);

static void __exit tegra_spi_exit(void)
{
	platform_driver_unregister(&tegra_spi_driver);
}
module_exit(tegra_spi_exit);


