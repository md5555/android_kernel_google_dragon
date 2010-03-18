/*
 * drivers/mtd/devices/tegra_mtd_nand.c
 *
 * MTD-class device driver for the internal NAND controller in Tegra SoCs
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

#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/wakelock.h>
#include <linux/tegra_devices.h>

#include <asm/dma-mapping.h>

#include "mach/nvrm_linux.h"
#include "nvddk_nand.h"
#include "nvos.h"
#include "nvassert.h"
#include "nvodm_query.h"

#define DRIVER_NAME	"tegra_nand"
#define DRIVER_DESC	"Nvidia Tegra NAND Flash Controller driver"

#define NDFLASH_CS_MAX 8

#define NAND_SPARE_SIZE	 64

#ifdef CONFIG_MTD_PARTITIONS
static const char *part_probes[] = { "cmdlinepart", NULL,  };
#endif

static NvDdkNandHandle s_hNand;

struct tegra_nand_chip {
	spinlock_t	lock;
	uint32_t	chipsize;
	int		num_chips;
	int		curr_chip;
	uint32_t	chip_shift;
	uint32_t	page_shift;
	uint32_t	page_mask;
	uint32_t	column_mask; /* column within page */
	uint32_t	block_shift;
	NvU32		pagesPerBlock;
	NvU32		blocksPerDevice;
	NvU32		pagesPerDevice;
	NvU32		tagSize;
	NvU32		pageSize;
	void		*priv;
};

struct tegra_nand_info {
	struct tegra_nand_chip	chip;
	struct mtd_info		mtd;
	struct device		 *dev;
	struct mtd_partition	*parts;
	struct mutex		lock;
	unsigned long		*bb_bitmap; /* block map 1=good 0=bad/unknown */
};
#define MTD_TO_INFO(mtd) container_of((mtd), struct tegra_nand_info, mtd)

/* must be called with lock held */
static int check_block_isbad(struct mtd_info *mtd, loff_t offs,
	NvU8   pTagArea[NAND_SPARE_SIZE])
{
	struct tegra_nand_info *info = MTD_TO_INFO(mtd);
	uint32_t block = offs >> info->chip.block_shift;
	int ret = 0;
	NvU32 deviceNum;
	NandBlockInfo blockInfo = { .pTagBuffer = NULL };
	NvDdkNandDeviceInfo nandDevInfo;
	NvError e;
	NvU32 pageNumbers[NDFLASH_CS_MAX];
	NvU32 BadBlockMarkerOffset = 1;
	NvU32 i;
	NvU32 blockIdx;

	NvDdkNandResumeClocks(s_hNand);
	if (info->bb_bitmap[BIT_WORD(block)] & BIT_MASK(block))
		return 0;

	NV_CHECK_ERROR_CLEANUP(
		NvDdkNandGetDeviceInfo(s_hNand, 0, &nandDevInfo)
	);

	NvOsMemset(&blockInfo, 0, sizeof(blockInfo));

	blockInfo.pTagBuffer = pTagArea;

	deviceNum = (block * mtd->erasesize) / info->chip.chipsize;
	blockIdx = block % nandDevInfo.NoOfBlocks;

	NvDdkNandGetBlockInfo(s_hNand, deviceNum, blockIdx,
		&blockInfo, NV_FALSE);

	if (!blockInfo.IsFactoryGoodBlock) {
		/* It's a factory bad block */
		ret = 1;
		pr_info("Block %d is factory bad in chip %d, offset = "
			"%llx\n", block, deviceNum, offs);
	} else {
		for (i=0;i<NDFLASH_CS_MAX;i++)
			pageNumbers[i] = -1;

		/* Second byte of the spare area in the first page of the block
		 * has the run-time bad block marker */
		pageNumbers[deviceNum] = blockIdx * nandDevInfo.PagesPerBlock;
		NV_CHECK_ERROR_CLEANUP(
			NvDdkNandReadSpare(s_hNand, deviceNum, pageNumbers,
			  pTagArea, 0, NAND_SPARE_SIZE)
		);

		if (pTagArea[BadBlockMarkerOffset] != 0xFF) {
			ret = 1;
			pr_info("Runtime bad[0x%x] at b,c,o=%d,%d,%llx\n",
				pTagArea[BadBlockMarkerOffset], block,
				deviceNum, offs);
		}
	}

fail:
	NvDdkNandSuspendClocks(s_hNand);
	/* update the bitmap if the block is good */
	if (ret == 0)
		set_bit(block, info->bb_bitmap);
	return ret;
}

#ifdef CONFIG_MTD_PARTITIONS
static struct mtd_partition * tegra_parse_mtd_partitions(int *Num)
{
	int PartitionCount = 0;
	int err = 0;
	struct mtd_partition *parts = NULL;

	if (!tegra_was_boot_device("nand"))
		return NULL;

	do {
		err = tegra_get_partition_info_by_num(PartitionCount,
				  NULL, NULL, NULL, NULL);
		if (!err)
			PartitionCount++;
	}
	while (!err);

	if (PartitionCount) {
		int i;
		NvU64 StartSector, NumSectors;
		NvU32 SectorSize;
		char *Name = NULL;

		parts = kzalloc(sizeof(*parts)*PartitionCount, GFP_KERNEL);
		if (!parts)
			return NULL;
		memset(parts, 0, sizeof(*parts)*PartitionCount);

		for (i=0; i<PartitionCount; i++) {
			Name = NULL;
			if (tegra_get_partition_info_by_num(i, &Name,
				&StartSector, &NumSectors, &SectorSize))
				goto fail;

			parts[i].name = Name;
			parts[i].offset = (StartSector * (NvU64)SectorSize);
			parts[i].size = (NumSectors * (NvU64)SectorSize);
		}
	}
	*Num = PartitionCount;

	return parts;

 fail:
	while (parts && PartitionCount) {
		--PartitionCount;
		if (parts[PartitionCount].name) {
			NvOsFree(parts[PartitionCount].name);
			parts[PartitionCount].name = NULL;
		}
	}
	kfree(parts);
	parts = NULL;
	return NULL;
}
#endif

static int tegra_nand_block_isbad(struct mtd_info *mtd, loff_t offs)
{
	NvU8 TagArea[NAND_SPARE_SIZE];
	struct tegra_nand_info *info = MTD_TO_INFO(mtd);
	int ret;

	if (offs >= mtd->size)
		return -EINVAL;

	mutex_lock(&info->lock);
	ret = check_block_isbad(mtd, offs, TagArea);
	mutex_unlock(&info->lock);

	return ret;
}


static int tegra_nand_block_markbad(struct mtd_info *mtd, loff_t offs)
{
	NvError e = NvSuccess;
	struct tegra_nand_info *info = MTD_TO_INFO(mtd);
	uint32_t block = offs >> info->chip.block_shift;
	NvU32 pageNumbers[NDFLASH_CS_MAX];
	NvU32 i;
	NvU32 blockIdx;
	NvU32 deviceNum;
	NvU8 TagBuff[64];
	const NvU32  BadBlockMarkerOffset = 0x1;

	if (offs >= mtd->size)
		return -EINVAL;

	pr_info("tegra_nand: setting block %d bad\n", block);

	mutex_lock(&info->lock);
	offs &= ~(mtd->erasesize - 1);

	/* mark the block bad in our bitmap */
	clear_bit(block, info->bb_bitmap);
	mtd->ecc_stats.badblocks++;

	for (i=0;i<NDFLASH_CS_MAX;i++)
		pageNumbers[i] = -1;

	NvOsMemset(TagBuff, sizeof(TagBuff), 0x0);

	blockIdx = block % info->chip.blocksPerDevice;
	deviceNum = (block * mtd->erasesize) / info->chip.chipsize;
	NV_ASSERT(deviceNum < NDFLASH_CS_MAX);
	pageNumbers[deviceNum] = blockIdx * info->chip.pagesPerBlock;

	NvDdkNandResumeClocks(s_hNand);
	NV_CHECK_ERROR_CLEANUP(
		NvDdkNandWriteSpare(s_hNand, deviceNum, pageNumbers,
			TagBuff, BadBlockMarkerOffset, 1)
	);

	NvDdkNandSuspendClocks(s_hNand);
	mutex_unlock(&info->lock);
	return 0;
fail:
	NvDdkNandSuspendClocks(s_hNand);
	mutex_unlock(&info->lock);
	return -ENXIO;
}

static int tegra_nand_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	struct tegra_nand_info *info = MTD_TO_INFO(mtd);
	uint32_t num_blocks;
	NvError e;
	NvU32 pageNumbers[NDFLASH_CS_MAX];
	NvU32 i, blocksToErase = 1;
	NvU8  TagArea[NAND_SPARE_SIZE];
	NvU32 PageNumber = 0, ChipNumber = 0;
	NvU64 curAddr = instr->addr;

	for (i=0;i<NDFLASH_CS_MAX;i++)
		pageNumbers[i] = -1;

	pr_debug("tegra_nand_erase: addr=0x%08x len=%d\n", instr->addr,
		   instr->len);

	if ((instr->addr + instr->len) > mtd->size) {
		pr_err("tegra_nand_erase: Can't erase past end of device\n");
		instr->state = MTD_ERASE_FAILED;
		return -EINVAL;
	}

	if (instr->addr & (mtd->erasesize - 1)) {
		pr_err("tegra_nand_erase: addr=0x%08llx not block-aligned\n",
			   instr->addr);
		instr->state = MTD_ERASE_FAILED;
		return -EINVAL;
	}

	if (instr->len & (mtd->erasesize - 1)) {
		pr_err("tegra_nand_erase: len=%lld not block-aligned\n",
			   instr->len);
		instr->state = MTD_ERASE_FAILED;
		return -EINVAL;
	}

	mutex_lock(&info->lock);

	instr->state = MTD_ERASING;
	num_blocks = instr->len >> info->chip.block_shift;

	NvDdkNandResumeClocks(s_hNand);
	while(num_blocks) {
		if (check_block_isbad(mtd, curAddr, TagArea)) {
			pr_info("Skipping bad block at %llx\n", instr->addr);
			goto next_block;
		}

		pageNumbers[ChipNumber] = -1;

		/* get the nand flash chip number */
		ChipNumber = (NvU32)(curAddr >> info->chip.chip_shift);

		/* get the page number on the nand flash chip */
		PageNumber = (NvU32)((curAddr >> info->chip.page_shift) &
			info->chip.page_mask);

		NV_ASSERT(ChipNumber < NDFLASH_CS_MAX);
		pageNumbers[ChipNumber] = PageNumber;

		NV_CHECK_ERROR_CLEANUP(
			NvDdkNandErase(s_hNand, ChipNumber,
				pageNumbers, &blocksToErase)
		);

		NV_ASSERT(blocksToErase == 1);

next_block:
		curAddr += mtd->erasesize;
		num_blocks--;
	}

	instr->state = MTD_ERASE_DONE;
	NvDdkNandSuspendClocks(s_hNand);
	mutex_unlock(&info->lock);
	mtd_erase_callback(instr);
	return 0;

fail:
	pr_info("Erase of bad block(%x) failed \n",
		(NvU32)(curAddr >> info->chip.block_shift));
	instr->state = MTD_ERASE_FAILED;
	instr->fail_addr = curAddr;
	NvDdkNandSuspendClocks(s_hNand);
	mutex_unlock(&info->lock);
	return -EIO;
}

static int tegra_nand_read(struct mtd_info *mtd, loff_t from, size_t len,
	size_t *retlen,	uint8_t *buf)
{
	struct tegra_nand_info *info = MTD_TO_INFO(mtd);
	uint8_t *bufPtr = buf;
	NvError e;
	NvU32 pageNumbers[NDFLASH_CS_MAX];
	NvU32 i, pageCount = 1;
	NvS32 bytesLeft = len;
	NvU8  TagArea[NAND_SPARE_SIZE];
	NvU32 PageNumber = 0, ChipNumber = 0;
	NvU64 curAddr = from;

	mutex_lock(&info->lock);

	for (i=0;i<NDFLASH_CS_MAX;i++)
		pageNumbers[i] = -1;

	NvDdkNandResumeClocks(s_hNand);
	do {
		pageNumbers[ChipNumber] = -1;

		/* get the nand flash chip number */
		ChipNumber = (NvU32)(curAddr >> info->chip.chip_shift);

		/* get the page number on the nand flash chip */
		PageNumber = (NvU32)((curAddr >> info->chip.page_shift) &
			info->chip.page_mask);

		NV_ASSERT(ChipNumber < NDFLASH_CS_MAX);
		pageNumbers[ChipNumber] = PageNumber;

		if (check_block_isbad(mtd, curAddr, TagArea)) {
			pr_info("ReadError: BadBlock found at %llx!", curAddr);
			goto fail;
		}

		NV_CHECK_ERROR_CLEANUP(
			NvDdkNandRead(s_hNand, ChipNumber, pageNumbers,
				bufPtr, NULL, &pageCount, NV_FALSE)
		);

		NV_ASSERT(pageCount == 1);
		curAddr += info->chip.pageSize;
		bytesLeft -= info->chip.pageSize;
		bufPtr += info->chip.pageSize;

	} while (bytesLeft > 0);

	NvDdkNandSuspendClocks(s_hNand);
	*retlen = len;
	mutex_unlock(&info->lock);
	return 0;

fail:
	pr_info("%s: ReadError: NvDdkNandRead failed at %llx!", __func__, curAddr);
	NvDdkNandSuspendClocks(s_hNand);
	*retlen = len - bytesLeft;
	mutex_unlock(&info->lock);
	return -EINVAL;
}

static int do_read_oob(struct mtd_info *mtd, loff_t from,
	struct mtd_oob_ops *ops)
{
	struct tegra_nand_info *info = MTD_TO_INFO(mtd);
	uint8_t *oobbuf = ops->oobbuf;
	uint8_t *datbuf = ops->datbuf;
	uint32_t ooblen = oobbuf ? ops->ooblen : 0;
	NvError e;
	NvU32 pageNumbers[NDFLASH_CS_MAX];
	NvU32 i, pageCount = 1;
	NvU8 tempSpareBuffer[NAND_SPARE_SIZE];
	NvU32 PageNumber = 0, ChipNumber = 0;
	NvU64 curAddr = from;

	mutex_lock(&info->lock);

	if (check_block_isbad(mtd, curAddr, tempSpareBuffer)) {
		pr_info("%s: Reading oob data of bad block(%llx)!\n",
			__func__, (curAddr) >> info->chip.block_shift);
		mutex_unlock(&info->lock);
		return -EINVAL;
	}

	if (ooblen > info->chip.tagSize)
		return -EINVAL;

	for (i=0;i<NDFLASH_CS_MAX;i++)
		pageNumbers[i] = -1;

	/* get the nand flash chip number */
	ChipNumber = (NvU32)(curAddr >> info->chip.chip_shift);

	/* get the page number on the nand flash chip */
	PageNumber = (NvU32)((curAddr >> info->chip.page_shift) &
		info->chip.page_mask);

	NV_ASSERT(ChipNumber < NDFLASH_CS_MAX);
	pageNumbers[ChipNumber] = PageNumber;

	NvDdkNandResumeClocks(s_hNand);
	NV_CHECK_ERROR_CLEANUP(
		NvDdkNandRead(s_hNand, ChipNumber, pageNumbers,
			datbuf, tempSpareBuffer, &pageCount, NV_FALSE)
	);
	NvDdkNandSuspendClocks(s_hNand);

	NV_ASSERT(pageCount == 1);

	NvOsMemcpy(oobbuf, tempSpareBuffer, ooblen);

	ops->retlen = 0;
	ops->oobretlen = 0;

	mutex_unlock(&info->lock);
	return 0;

fail:
	pr_err("%s: Failed reading OOB addr(%llx)\n", __func__, curAddr);
	ops->retlen = 0;
	ops->oobretlen = 0;
	NvDdkNandSuspendClocks(s_hNand);

	mutex_unlock(&info->lock);
	return -EINVAL;
}

/* just does some parameter checking and calls do_read_oob */
static int tegra_nand_read_oob(struct mtd_info *mtd, loff_t from,
	struct mtd_oob_ops *ops)
{
	if (ops->datbuf && unlikely((from + ops->len) > mtd->size)) {
		pr_err("%s: Can't read past end of device.\n", __func__);
		return -EINVAL;
	}

	if (unlikely(ops->oobbuf && !ops->ooblen)) {
		pr_err("%s: Reading 0 bytes from OOB is meaningless\n",
			__func__);
		return -EINVAL;
	}

	if (unlikely(ops->mode != MTD_OOB_AUTO)) {
		if (ops->oobbuf && ops->datbuf) {
			pr_err("%s: can't read OOB + Data in non-AUTO mode.\n",
				__func__);
			return -EINVAL;
		}
		if ((ops->mode == MTD_OOB_RAW) && !ops->datbuf) {
			pr_err("%s: Raw mode only supports reading data.\n",
				__func__);
			return -EINVAL;
		}
	}

	NV_ASSERT(ops->oobbuf);

	return do_read_oob(mtd, from, ops);
}

static int tegra_nand_write(struct mtd_info *mtd, loff_t to, size_t len,
	size_t *retlen, const uint8_t *buf)
{
	struct tegra_nand_info *info = MTD_TO_INFO(mtd);
	NvU8 *bufPtr = (NvU8*)buf;
	NvError e;
	NvU32 pageNumbers[NDFLASH_CS_MAX];
	NvU32 i, pageCount = 1;
	NvS32 bytesLeft = len;
	NvU8  TagArea[NAND_SPARE_SIZE];
	NvU32 PageNumber = 0, ChipNumber = 0;
	NvU64 curAddr = to;

	pr_info("%s: write: to=0x%llx len=0x%x\n", __func__, to, len);

	mutex_lock(&info->lock);

	for (i=0;i<NDFLASH_CS_MAX;i++)
		pageNumbers[i] = -1;

	NvDdkNandResumeClocks(s_hNand);
	do {
		pageNumbers[ChipNumber] = -1;

		/* get the nand flash chip number */
		ChipNumber = (NvU32)(curAddr >> info->chip.chip_shift);

		/* get the page number on the nand flash chip */
		PageNumber = (NvU32)((curAddr >> info->chip.page_shift) &
			info->chip.page_mask);

		NV_ASSERT(ChipNumber < NDFLASH_CS_MAX);
		pageNumbers[ChipNumber] = PageNumber;

		if (check_block_isbad(mtd, curAddr, TagArea)) {
			pr_info("WriteError: BadBlock found at %llx!", curAddr);
			goto fail;
		}

		NV_CHECK_ERROR_CLEANUP(
			NvDdkNandWrite(s_hNand, ChipNumber,
				pageNumbers, bufPtr, NULL, &pageCount)
		);

		NV_ASSERT(pageCount == 1);
		curAddr += info->chip.pageSize;
		bytesLeft -= info->chip.pageSize;
		bufPtr += info->chip.pageSize;

	} while (bytesLeft > 0);

	*retlen = len;
	NvDdkNandSuspendClocks(s_hNand);
	mutex_unlock(&info->lock);
	return 0;

fail:
	NvDdkNandSuspendClocks(s_hNand);
	pr_info("WriteError: NvDdkNandWrite failed error(0x%x)", e);
	*retlen = len - bytesLeft;
	mutex_unlock(&info->lock);
	return -EINVAL;
}

static int do_write_oob(struct mtd_info *mtd, loff_t to,
	struct mtd_oob_ops *ops)
{
	struct tegra_nand_info *info = MTD_TO_INFO(mtd);
	uint8_t *oobbuf = ops->oobbuf;
	uint8_t *datbuf = ops->datbuf;
	uint32_t ooblen = oobbuf ? ops->ooblen : 0;
	NvError e;
	NvU32 pageNumbers[NDFLASH_CS_MAX];
	NvU32 i, pageCount = 1;
	NvU8 tempSpareBuffer[NAND_SPARE_SIZE];
	NvU32 PageNumber = 0, ChipNumber = 0;

	mutex_lock(&info->lock);

	if (ooblen > info->chip.tagSize)
		return -EINVAL;

	memset(tempSpareBuffer, 0xFF, info->chip.tagSize);
	memcpy(tempSpareBuffer, oobbuf, ooblen);

	for (i=0;i<NDFLASH_CS_MAX;i++)
		pageNumbers[i] = -1;

	/* get the nand flash chip number */
	ChipNumber = (NvU32)(to >> info->chip.chip_shift);

	/* get the page number on the nand flash chip */
	PageNumber = (NvU32)((to >> info->chip.page_shift) &
		info->chip.page_mask);

	NV_ASSERT(ChipNumber < NDFLASH_CS_MAX);
	pageNumbers[ChipNumber] = PageNumber;

	NvDdkNandResumeClocks(s_hNand);
	NV_CHECK_ERROR_CLEANUP(
		NvDdkNandWrite(s_hNand, ChipNumber, pageNumbers,
			datbuf, tempSpareBuffer, &pageCount)
	);

	NV_ASSERT(pageCount == 1);

	ops->retlen = 0;
	ops->oobretlen = 0;

	NvDdkNandSuspendClocks(s_hNand);
	mutex_unlock(&info->lock);
	return 0;

fail:
	NvDdkNandSuspendClocks(s_hNand);
	pr_info("%s: NvDdkNandWrite failed error(0x%x)", __func__ , e);
	ops->retlen = 0;
	ops->oobretlen = 0;

	mutex_unlock(&info->lock);
	return -EINVAL;
}

static int tegra_nand_write_oob(struct mtd_info *mtd, loff_t to,
	struct mtd_oob_ops *ops)
{
	struct tegra_nand_info *info = MTD_TO_INFO(mtd);

	if (unlikely(to & info->chip.column_mask)) {
		pr_err("%s: Unaligned write (to 0x%llx) not supported\n",
			   __func__, to);
		return -EINVAL;
	}

	if (unlikely(ops->oobbuf && !ops->ooblen)) {
		pr_err("%s: Writing 0 bytes to OOB is meaningless\n", __func__);
		return -EINVAL;
	}

	return do_write_oob(mtd, to, ops);
}

static int tegra_nand_suspend(struct platform_device *dev,
	pm_message_t state)
{
	struct mtd_info *mtd = platform_get_drvdata(dev);
	NvError Err;

	/* Call ddk suspend API */
	if (!s_hNand) {
		NvOsDebugPrintf("\n Nand: Ddk handle NULL in suspend ");
		return -1;
	}
	Err = NvDdkNandSuspend(s_hNand);
	if (Err != NvSuccess) {
		NvOsDebugPrintf("\n Nand Ddk Suspend error=0x%x ", Err);
		return -1;
	}
	return 0;
}

static int tegra_nand_resume(struct platform_device *dev)
{
	struct mtd_info *mtd = platform_get_drvdata(dev);
	NvError Err;

	/* call Ddk resume code */
	if (!s_hNand) {
		NvOsDebugPrintf("\n Nand: Ddk handle NULL in resume ");
		return -1;
	}
	Err = NvDdkNandResume(s_hNand);
	if (Err != NvSuccess) {
		NvOsDebugPrintf("\n Nand Ddk Resume error=0x%x ", Err);
		return -1;
	}
	return 0;
}

static int scan_bad_blocks(struct tegra_nand_info *info)
{
	struct mtd_info *mtd = &info->mtd;
	int num_blocks = mtd->size >> info->chip.block_shift, numBlocks;
	uint32_t block;
	int is_bad = 0;

	numBlocks = info->chip.blocksPerDevice * info->chip.num_chips;

	pr_debug("Scan: num_blocks = %d, numBlocks = %d, shift = %d\n",
		num_blocks, numBlocks,info->chip.block_shift);
	for (block = 0; block < num_blocks; ++block) {
		/* make sure the bit is cleared, meaning it's bad/unknown before
		 * we check. */
		clear_bit(block, info->bb_bitmap);

		is_bad = mtd->block_isbad(mtd, block << info->chip.block_shift);

		if (is_bad == 0)
			set_bit(block, info->bb_bitmap);
		else if (is_bad > 0)
			pr_info("block 0x%08x is bad.\n", block);
		else {
			pr_err("Fatal error (%d) while scanning for "
				   "bad blocks\n", is_bad);
			return is_bad;
		}
	}
	return 0;
}

/* Scans for nand flash devices, identifies them, and fills in the
 * device info. */
static int tegra_nand_scan(struct mtd_info *mtd)
{
	struct tegra_nand_info *info = MTD_TO_INFO(mtd);
	uint32_t tmp;
	uint32_t dev_id;
	uint32_t vendor_id;
	int err = 0;
	NvDdkNandDeviceInfo nandDevInfo;
	NvError e;

	NV_CHECK_ERROR_CLEANUP(
		NvDdkNandOpen(s_hRmGlobal, &s_hNand)
	);

	NV_CHECK_ERROR_CLEANUP(
		NvDdkNandGetDeviceInfo(s_hNand, 0, &nandDevInfo)
	);

	/*Get some info from the nand driver */
	vendor_id = nandDevInfo.VendorId;
	dev_id = nandDevInfo.DeviceId;

	pr_info("Found Nand Chip with Vendor = 0x%02x, DevId = 0x%02x\n",
		vendor_id, dev_id);

	info->chip.num_chips = nandDevInfo.NumberOfDevices;
	info->chip.chipsize = nandDevInfo.DeviceCapacityInKBytes * 1024;
	info->chip.pagesPerBlock = nandDevInfo.PagesPerBlock;
	info->chip.blocksPerDevice = nandDevInfo.NoOfBlocks;
	info->chip.pagesPerDevice = nandDevInfo.NoOfBlocks *
		nandDevInfo.PagesPerBlock;
	info->chip.tagSize = nandDevInfo.TagSize;
	info->chip.pageSize = nandDevInfo.PageSize;

	mtd->size = info->chip.num_chips * info->chip.chipsize;

	/* page_size is same as read and write size */
	mtd->writesize = nandDevInfo.PageSize;
	info->chip.column_mask = mtd->writesize - 1;

	/* Note: See oob layout description of why we only support 2k pages. */
	if (mtd->writesize > 2048) {
		pr_err("%s: Large page devices with pagesize > 2kb are NOT "
			   "supported\n", __func__);
		goto out_error;
	} else if (mtd->writesize < 2048) {
		pr_err("%s: Small page devices are NOT supported\n", __func__);
		goto out_error;
	}

	/* spare area, must be at least 64 bytes */
	/* FIXME: Nand driver doesn't expose spare area size? */
	tmp = NAND_SPARE_SIZE;

	if (tmp < 64) {
		pr_err("%s: Spare area (%d bytes) too small\n", __func__, tmp);
		goto out_error;
	}
	mtd->oobsize = tmp;
	mtd->oobavail = nandDevInfo.TagSize;

	/* data block size (erase size) (w/o spare) */
	tmp = nandDevInfo.PagesPerBlock * nandDevInfo.PageSize;
	mtd->erasesize = tmp;
	info->chip.block_shift = ffs(mtd->erasesize) - 1;

	/* used to select the appropriate chip/page in case multiple devices
	 * are connected */
	info->chip.chip_shift = ffs(info->chip.chipsize) - 1;
	info->chip.page_shift = ffs(mtd->writesize) - 1;
	info->chip.page_mask =
		(info->chip.chipsize >> info->chip.page_shift) - 1;

	/* now fill in the rest of the mtd fields */
	mtd->ecclayout = NULL;
	mtd->type = MTD_NANDFLASH;
	mtd->flags = MTD_CAP_NANDFLASH;

	mtd->erase = tegra_nand_erase;
	mtd->lock = NULL;
	mtd->point = NULL;
	mtd->unpoint = NULL;
	mtd->read = tegra_nand_read;
	mtd->write = tegra_nand_write;
	mtd->read_oob = tegra_nand_read_oob;
	mtd->write_oob = tegra_nand_write_oob;

	mtd->resume = tegra_nand_resume;
	mtd->suspend = tegra_nand_suspend;
	mtd->block_isbad = tegra_nand_block_isbad;
	mtd->block_markbad = tegra_nand_block_markbad;
	NvDdkNandSuspendClocks(s_hNand);

	return 0;

fail:
out_error:
	NvDdkNandSuspendClocks(s_hNand);
	pr_err("%s: NAND device scan aborted due to error(s).\n", __func__);
	return err;
}

static int __devinit tegra_nand_probe(struct platform_device *pdev)
{
	struct tegra_nand_info *info = NULL;
	struct tegra_nand_chip *chip = NULL;
	struct mtd_info *mtd = NULL;
#ifdef CONFIG_MTD_PARTITIONS
	struct mtd_partition *tegraparts = NULL;
	int numtegraparts;
#endif
	int err = 0;

	pr_info("%s: probing (%p)\n", __func__, pdev);

	info = kzalloc(sizeof(struct tegra_nand_info), GFP_KERNEL);
	if (!info) {
		pr_err("%s: no memory for flash info\n", __func__);
		return -ENOMEM;
	}

	info->dev = &pdev->dev;

	mutex_init(&info->lock);

	chip = &info->chip;
	chip->priv = &info->mtd;
	chip->curr_chip = -1;

	mtd = &info->mtd;
	mtd->priv = &info->chip;
	mtd->owner = THIS_MODULE;

	if (tegra_nand_scan(mtd))
	{
		err = -ENXIO;
		goto out_dis_irq;
	}

	/* alloc the bad block bitmap */
	info->bb_bitmap = kzalloc(BITS_TO_LONGS(mtd->size >>
		info->chip.block_shift) * sizeof(unsigned long), GFP_KERNEL);
	if (!info->bb_bitmap) {
		err = -ENOMEM;
		goto out_dis_irq;
	}

	if ((err = scan_bad_blocks(info)) != 0)
		goto out_free_bbbmap;

#ifdef CONFIG_MTD_PARTITIONS
	err = parse_mtd_partitions(mtd, part_probes, &info->parts, 0);
	if (err <= 0)
		tegraparts = tegra_parse_mtd_partitions(&numtegraparts);

	if (err > 0) {
		add_mtd_partitions(mtd, info->parts, err);
	} else if (err <= 0 && tegraparts) {
		err = add_mtd_partitions(mtd, tegraparts, numtegraparts);
	} else
#endif
	{
		if ((err = add_mtd_device(mtd)) != 0)
		   goto out_free_bbbmap;
	}

	dev_set_drvdata(&pdev->dev, info);

	pr_info("%s: probe done.\n", __func__);
	return 0;

out_free_bbbmap:
	kfree(info->bb_bitmap);

out_dis_irq:
	kfree(info);

	return err;
}

static int __devexit tegra_nand_remove(struct platform_device *pdev)
{
	struct tegra_nand_info *info = dev_get_drvdata(&pdev->dev);

	dev_set_drvdata(&pdev->dev, NULL);

	if (info) {
		kfree(info->bb_bitmap);
		kfree(info);
		NvDdkNandSuspendClocks(s_hNand);
		NvDdkNandClose(s_hNand);
		s_hNand = NULL;
	}

	return 0;
}

static struct platform_driver tegra_nand_driver = {
	.probe		= tegra_nand_probe,
	.remove		= __devexit_p(tegra_nand_remove),
	.suspend	= tegra_nand_suspend,
	.resume		= tegra_nand_resume,
	.driver		= {
		.name	= "tegra_nand",
		.owner	= THIS_MODULE,
	},
};

static int __init tegra_nand_init(void)
{
	return platform_driver_register(&tegra_nand_driver);
}

static void __exit tegra_nand_exit(void)
{
	platform_driver_unregister(&tegra_nand_driver);
}

module_init(tegra_nand_init);
module_exit(tegra_nand_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_DESC);
