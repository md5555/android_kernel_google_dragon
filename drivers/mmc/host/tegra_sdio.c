/*
 * drivers/mmc/host/tegra_sdio.c
 *
 * MMC host driver for NVIDIA Tegra SoCs using NvDdk and NvRm APIs
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

// #define DEBUG
// #define VERBOSE_DEBUG

/*
 *  TODO: 
 *  1. Revisit locking
 *  2. use zero-copy DMA instead of buffer copy.
 *  3. Detect card removal properly and handle the trasactions on the fly
 *  4. Don't know how to enable SD card irq.
 *  5. Some of the values like max clock, voltages supported and bit rates are
 *  hard coded. Need to get those from DDK.
 */


#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>

#include <linux/mmc/host.h>

#include "mach/nvrm_linux.h"
#include "nvddk_sdio.h"
#include "nvos.h"
#include "nvodm_sdio.h"

#define DRIVER_DESC "Nvidia Tegra SDIO/SD driver"
#define DRIVER_NAME "tegra_sdio"

#ifdef VERBOSE_DEBUG
#define DBG(f, x...) \
	pr_info(DRIVER_NAME " [%s()]: " f, __func__,## x)
#else
#define DBG(f, x...) 
#endif

#define USE_SPINLOCK 0

#if USE_SPINLOCK
#define LOCK_T		spinlock_t
#define CREATELOCK(_l)	spin_lock_init(&(_l))
#define DELETELOCK(_l)
#define LOCK(_l)	spin_lock(&(_l))
#define UNLOCK(_l)	spin_unlock(&(_l))
#define ATOMIC(_l,_f)	spin_lock_irqsave(&(_l),(_f))
#define UNATOMIC(_l,_f)	spin_unlock_irqrestore(&(_l),(_f))
#else
#define LOCK_T		struct mutex
#define CREATELOCK(_l)	mutex_init(&(_l))
#define DELETELOCK(_l) 
#define LOCK(_l)	mutex_lock(&(_l))
#define UNLOCK(_l)	mutex_unlock(&(_l))
#define ATOMIC(_l,_f)	local_irq_save((_f))
#define UNATOMIC(_l,_f)	local_irq_restore((_f))
#endif

struct tegra_sdio_host{
	struct mmc_host			*mmc;
	struct platform_device		*pdev;
	LOCK_T				Lock;
	NvU32				Instance;
	NvDdkSdioSDBusVoltage		voltage;
	struct work_struct		Workqueue;
	struct mmc_request		*req;
	NvBool				bCardPresent;
	NvDdkSdioDeviceHandle		hDdk;
	NvDdkSdioHostCapabilities	hostCaps;
	NvDdkSdioInterfaceCapabilities	interfaceCaps;
	NvOsSemaphoreHandle		hSem;
	NvOsSemaphoreHandle		hCardSemaphore;
	NvOdmSdioHandle			hSdioHandle;
	struct task_struct		*cardDetectTask;
	NvBool				bKillCardDetectThread;
	unsigned int			clock;
	unsigned int			bus_width;
	NvU32				StartOffset;
};

#ifdef VERBOSE_DEBUG
static void tegra_DumpPackets(struct mmc_data *data)
{
	void *addr = (void *)sg_virt(data->sg);
	int i;

	for (i=0; i< sg_dma_len(data->sg); i++) {
		if (!(i%8))
			printk("\n");
		printk("%x ", ((unsigned char *)addr)[i]);
	}
	printk("\n");
}
#endif

static int tegra_sdio_carddetect(void *arg)
{
	struct tegra_sdio_host *host = (struct tegra_sdio_host *)arg;
	while (1) {
		NvOsSemaphoreWait(host->hCardSemaphore);
		if (host->bKillCardDetectThread)
			break;
		mmc_detect_change(host->mmc, 0);
	}
	return 0;
}

static inline NvDdkSdioRespType tegra_ConvertRespType(struct mmc_command *cmd)
{
	NvDdkSdioRespType resp = 0;

	switch (mmc_resp_type(cmd)) {
	case MMC_RSP_NONE:
		resp = NvDdkSdioRespType_NoResp;
		break;
	case MMC_RSP_R1: /* MMC_RSP_R5 & MMC_RSP_R6 */
		resp = NvDdkSdioRespType_R1;
		break;
	case MMC_RSP_R1B:
		resp = NvDdkSdioRespType_R1b;
		break;
	case MMC_RSP_R2:
		resp = NvDdkSdioRespType_R2;
		break;
	case MMC_RSP_R3:	/* Same as MMC_RSP_R4 */
		resp = NvDdkSdioRespType_R3;
		break;
	default:
		BUG_ON(1);
	}
	return resp;
}


static inline void tegra_GetRepsonse(struct tegra_sdio_host *host,
	struct mmc_command *cmd)
{
	if (cmd->flags & MMC_RSP_136) {
		int i;
		NvU32 resp[4];
		NvDdkSdioGetCommandResponse(host->hDdk, 0,
			tegra_ConvertRespType(cmd), resp);

		/* Re-format to the way the linux stack expects */
		for (i = 0;i < 4;i++) {
			cmd->resp[i] = resp[3-i] << 8;
			if (i != 3)
				cmd->resp[i] |= ((NvU8 *)resp)[(3-i) * 4 -1];
		}
	} else {
		NvDdkSdioGetCommandResponse(host->hDdk, 0, 
			tegra_ConvertRespType(cmd), 
			(NvU32 *)(cmd->resp));
	}
	return;
}

//
//  Work queue which does all the heavy lifting.
//

static void tegra_sdio_wq(struct work_struct *w)
{
	struct tegra_sdio_host *host = container_of(w, struct tegra_sdio_host, 
		Workqueue);
	struct mmc_request *req;
	NvDdkSdioCommand cmd;
	NvU32 status;
	NvError err;

	LOCK(host->Lock);

	req = host->req;

	NvOsMemset(&cmd, 0, sizeof(cmd));
	cmd.CommandCode = req->cmd->opcode;
	cmd.CommandType = NvDdkSdioCommandType_Normal;
	cmd.CmdArgument = req->cmd->arg;
	cmd.ResponseType = tegra_ConvertRespType(req->cmd);
	if (req->data) {
		struct mmc_data *data = req->data;
		void *addr;
		int sg_cnt;
		int size;
		NvBool cmd12Enable = NV_FALSE;

		/* FIXME only works for block addressing cards! */
		if (host->StartOffset)
			cmd.CmdArgument += host->StartOffset >> 9;

		BUG_ON(data->blksz * data->blocks > 524288);
		BUG_ON(data->blksz > host->mmc->max_blk_size);
		BUG_ON(data->blocks > 65535);

		sg_cnt = dma_map_sg(mmc_dev(host->mmc), data->sg, data->sg_len,
			DMA_BIDIRECTIONAL);
		BUG_ON(sg_cnt != 1);

		cmd.IsDataCommand = NV_TRUE;
		cmd.BlockSize = data->blksz;
		if (data->stop) {
			/* only support CMD12 of all the stop commands.
			 * This seems to work!
			 */
			if (data->stop->opcode == 12 && !data->stop->arg) {
				cmd12Enable = NV_TRUE;
			} else {
				BUG_ON(1);
			}
		}

		size = data->blocks * data->blksz;
		addr = (void *)sg_virt(data->sg);
		
		if (data->flags & MMC_DATA_READ)
			err = NvDdkSdioRead(host->hDdk, size, addr, &cmd, 
				cmd12Enable, &status);
		else
			err = NvDdkSdioWrite(host->hDdk, size, addr, &cmd,
				cmd12Enable, &status);

		if (err == NvSuccess && status == NvDdkSdioError_None) {
			req->cmd->error = 0;
			req->data->error = 0;
			data->bytes_xfered = data->blksz * data->blocks;

		} else {
			/* FIXME set the appropriate error code */
			DBG("Controller reported error 0x%x API error 0x%x\n",
				status, err);
			data->bytes_xfered = 0;
			req->cmd->error = -EIO;
			req->data->error = -EIO;
		}

		dma_unmap_sg(mmc_dev(host->mmc), data->sg,
			data->sg_len, DMA_BIDIRECTIONAL);

		tegra_GetRepsonse(host, req->cmd);

	} else {
		cmd.IsDataCommand = NV_FALSE;
		cmd.BlockSize = 512;
		err = NvDdkSdioSendCommand(host->hDdk, &cmd, &status);
		if (err == NvSuccess) {
			if (status == NvDdkSdioError_None)
				req->cmd->error = 0;
			else
				req->cmd->error = -EINVAL;
		} else
			DBG("DDK failed controller status = 0x%x\n", status);

		tegra_GetRepsonse(host, req->cmd);
	}

	UNLOCK(host->Lock);
 
	host->req = NULL;
	mmc_request_done(host->mmc, req);
	return;
}

//
//  MMC callbacks
//

static void tegra_sdio_request(struct mmc_host *mmc, struct mmc_request *req)
{
	struct tegra_sdio_host *host;

	host = mmc_priv(mmc);

	LOCK(host->Lock);

	host->req = req;
	if (!host->bCardPresent) {
		host->req->cmd->error = -ENOMEDIUM;
	}

	schedule_work(&host->Workqueue);
	UNLOCK(host->Lock);
}

static int tegra_sdio_get_ro(struct mmc_host *mmc)
{
	struct tegra_sdio_host *host;
	NvBool bIsWriteProtected = NV_FALSE;
	NvError err;

	host = mmc_priv(mmc);
	err = NvDdkSdioIsWriteProtected(host->hDdk, s_hGpioGlobal, 
		&bIsWriteProtected);
	if (err != NvSuccess) {
		pr_err(DRIVER_NAME "IsWriteProtected error(%d)\n", err);
		if (err == NvError_SdioCardNotPresent) {
			return 0;
		} else {
			return -ENOSYS;
		}
	}
	return (bIsWriteProtected == NV_TRUE) ? 1 : 0;
}


static void tegra_sdio_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct tegra_sdio_host *host;
	NvError err;

	host = mmc_priv(mmc);
 
	/* 
	 * We first get the MMC_POWER_OFF, then MMC_POWER_UP and 
	 * then the stack swtiches to the MMC_POWER_ON mode.
	 */
	if (ios->power_mode == MMC_POWER_OFF) {
		/* How do we power off the controller and the io? */
	}

	/* Set the clock. FIXME DDK doesn't allow the clock to be set to 0 */
	if (ios->clock) {
		if (ios->clock != host->clock) {
			NvU32 configuredFreq;
			err = NvDdkSdioSetClockFrequency(host->hDdk,
				ios->clock/1000, &configuredFreq);
			BUG_ON(err!=NvSuccess);
			DBG("SDIO clock set to (%dKHz) req (%dKHz) err (%d)\n", 
				configuredFreq, ios->clock/1000, err);
			host->clock = ios->clock;
		}
	}

	/* Set the bus width */
	if (host->bus_width != ios->bus_width) {
		NvDdkSdioDataWidth width;
		if (ios->bus_width == MMC_BUS_WIDTH_8) {
			width = NvDdkSdioDataWidth_8Bit;
		} else if (ios->bus_width == MMC_BUS_WIDTH_4) {
			width = NvDdkSdioDataWidth_4Bit;
		} else if (ios->bus_width == MMC_BUS_WIDTH_1) {
			width = NvDdkSdioDataWidth_1Bit;
		} else {
			/* Got wrong parameters */
			width = 0;
			BUG_ON(1);
		}
		NvDdkSdioSetHostBusWidth(host->hDdk, width);
		host->bus_width = ios->bus_width;
	}

	/* timming - is there any difference between SD high speed and mmc high
	 * speed? */
	if (ios->timing == MMC_TIMING_SD_HS || ios->timing == MMC_TIMING_SD_HS)
		NvDdkSdioHighSpeedEnable(host->hDdk);
	else 
		NvDdkSdioHighSpeedDisable(host->hDdk);

	return;
}

static int tegra_sdio_cardpresent(struct mmc_host *mmc)
{
	struct tegra_sdio_host *host;
	NvBool bIsCardPResent;

	host = mmc_priv(mmc);
	NvDdkSdioIsCardInserted(host->hDdk, &bIsCardPResent);

	DBG("Sdio slot %s:%s\n", mmc_hostname(mmc),
		((bIsCardPResent == NV_TRUE) ? "full" : "empty"));

	return (bIsCardPResent == NV_TRUE) ? 1 : 0;
}

static void tegra_sdio_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
	/* FIXME what should we do here? */
	return;
}

static struct mmc_host_ops tegra_sdio_ops = 
{
	.request = tegra_sdio_request,
	.set_ios = tegra_sdio_set_ios,
	.get_ro = tegra_sdio_get_ro,
	.get_cd = tegra_sdio_cardpresent,
	.enable_sdio_irq = tegra_sdio_enable_sdio_irq,
};

static int
tegra_sdio_probe(struct platform_device *pdev)
{
	struct tegra_sdio_host *host = NULL;
	struct mmc_host *mmc = NULL;
	int ret = -ENODEV;
	NvError err;
	struct tegra_sdio_pdata *pdata = pdev->dev.platform_data;

	mmc = mmc_alloc_host(sizeof(struct tegra_sdio_host), &pdev->dev);
	if (!mmc) {
		ret = -ENOMEM;
		goto fail;
	}

	dev_set_drvdata(&pdev->dev, host);

	host = mmc_priv(mmc);
	host->mmc = mmc;
	host->pdev = pdev;
	host->Instance = pdev->id;
	host->voltage = NvDdkSdioSDBusVoltage_invalid;
	CREATELOCK(host->Lock);
	INIT_WORK(&host->Workqueue, tegra_sdio_wq);
	host->hSdioHandle = NvOdmSdioOpen(pdev->id);
	if (!host->hSdioHandle) {
		goto fail;
	}
	NvDdkSdioGetCapabilities(s_hRmGlobal, &host->hostCaps,
		&host->interfaceCaps, pdev->id);

	/* Set SD/MMC configuration */
	mmc->ops = &tegra_sdio_ops;
	mmc->caps = 0;
	if (host->interfaceCaps.MmcInterfaceWidth >= 4) {
		mmc->caps |= MMC_CAP_4_BIT_DATA;
		if (host->interfaceCaps.MmcInterfaceWidth == 8) {
			mmc->caps |= MMC_CAP_8_BIT_DATA;
		}
	}

	mmc->caps |= MMC_CAP_SDIO_IRQ;

	/* Is there a caps bit to select if the controller supports high speed
	 * mode or not? */
	if (1) {
		mmc->caps |= MMC_CAP_SD_HIGHSPEED;
		mmc->caps |= MMC_CAP_MMC_HIGHSPEED;
	}

	/* Supports upto to 52M i.e compliant to high speed and EMMC and
	 * SD specs */
	mmc->f_max = 52000000;
	mmc->f_min = 400000;
	mmc->max_seg_size = 524288;
	mmc->max_req_size = 524288;
	mmc->max_phys_segs = 128;
	mmc->max_hw_segs = 1;
	mmc->max_blk_size = 512;
	mmc->max_blk_count = 65535;
	
	mmc->ocr_avail = 0;
	mmc->ocr_avail |= MMC_VDD_32_33|MMC_VDD_33_34;
	
	if ((NvOsSemaphoreCreate(&host->hCardSemaphore, 0) != NvSuccess)
		|| (NvOsSemaphoreCreate(&host->hSem, 0) != NvSuccess)) {
		goto fail;
	}

	host->bKillCardDetectThread = NV_FALSE;
	host->cardDetectTask = kthread_create(tegra_sdio_carddetect, 
		(void *)host, "sdio_card_detect/%d", pdev->id);
	if (IS_ERR(host->cardDetectTask)) {
		goto fail;
	}
	wake_up_process( host->cardDetectTask );

	err = NvDdkSdioOpen(s_hRmGlobal, s_hGpioGlobal, &host->hDdk, 
		&host->hSem, &host->hCardSemaphore, pdev->id);
	if (err != NvSuccess) {
		goto fail;
	}
	host->clock = 0;
	host->bus_width = (unsigned int)-1;
	if (pdata)
		host->StartOffset = pdata->StartOffset;

	ret = mmc_add_host(mmc);
	if (ret) {
		dev_err(&pdev->dev, "failed to add mmc host\n");
		goto fail;
	}

	platform_set_drvdata(pdev, mmc);
	dev_info(&pdev->dev, "Initialization done\n");

	return 0;

fail:
	if (host) {
		if (host->cardDetectTask) {
			host->bKillCardDetectThread = NV_TRUE;
			NvOsSemaphoreSignal(host->hCardSemaphore);
			(void)kthread_stop( host->cardDetectTask );
		}
		if (host->hSem)
			NvOsSemaphoreDestroy(host->hSem);
		if (host->hCardSemaphore)
			NvOsSemaphoreDestroy(host->hCardSemaphore);
		if (host->hSdioHandle)
			NvOdmSdioClose(host->hSdioHandle);
		if (host->hDdk)
			NvDdkSdioClose(host->hDdk);
	}
	if (mmc)
		mmc_free_host(mmc);

	return ret;
}

static int __devexit
tegra_sdio_remove(struct platform_device *pdev)
{
	struct tegra_sdio_host *host = dev_get_drvdata(&pdev->dev);

	if (!host) return 0;

	host->bKillCardDetectThread = NV_TRUE;
	NvOsSemaphoreSignal(host->hCardSemaphore);
	(void)kthread_stop( host->cardDetectTask );

	if (host->hDdk)
		NvDdkSdioClose(host->hDdk);

	if (host->hSdioHandle)
		NvOdmSdioClose(host->hSdioHandle);

	mmc_free_host(host->mmc);
	dev_set_drvdata(&pdev->dev, NULL);
	return 0;
}

static struct platform_driver tegra_sdio_driver = {
	.probe = tegra_sdio_probe,
	.remove = __devexit_p(tegra_sdio_remove),
	.suspend = NULL,
	.resume = NULL,
	.driver = {
		.name = "tegra-sdio",
		.owner = THIS_MODULE,
	},
};

static int __init tegra_sdio_init(void)
{
	return platform_driver_register(&tegra_sdio_driver);
}

static void __exit tegra_sdio_exit(void)
{
	platform_driver_unregister(&tegra_sdio_driver);
}

module_init(tegra_sdio_init);
module_exit(tegra_sdio_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_DESC);

