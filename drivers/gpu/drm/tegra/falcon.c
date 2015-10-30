/*
 * Copyright (c) 2014-2015, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "drm.h"
#include "falcon.h"

#define FALCON_IDLE_TIMEOUT_DEFAULT_MS		100

/**
 * This is a number (actually the NVIDIA PCI vendor ID) that is implanted in
 * the header of the firmware binary. Used as a sanity check vs bad firmware
 * binary or endianness issues.
 */
#define FALCON_FW_MAGIC				0x10de

enum falcon_memory {
	FALCON_MEMORY_IMEM,
	FALCON_MEMORY_DATA,
};

static u32 falcon_readl(struct falcon *falcon, u32 offset)
{
	return readl(falcon->regs + offset);
}

static void falcon_writel(struct falcon *falcon, u32 value, u32 offset)
{
	writel(value, falcon->regs + offset);
}

static int falcon_wait_idle(struct falcon *falcon)
{
	return wait_for(
		!falcon_readl(falcon, FALCON_IDLESTATE),
		FALCON_IDLE_TIMEOUT_DEFAULT_MS);
}

static int falcon_dma_wait_idle(struct falcon *falcon)
{
	return wait_for(
		falcon_readl(falcon, FALCON_DMATRFCMD) & FALCON_DMATRFCMD_IDLE,
		FALCON_IDLE_TIMEOUT_DEFAULT_MS);
}

static int falcon_wait_mem_scrubbing(struct falcon *falcon)
{
	return wait_for(
		!(falcon_readl(falcon, FALCON_DMACTL) &
		  (FALCON_DMACTL_DMEM_SCRUBBING |
		   FALCON_DMACTL_IMEM_SCRUBBING)),
		FALCON_IDLE_TIMEOUT_DEFAULT_MS);
}

/**
 * Copies data pointed to by base+offset into falcon data or instruction
 * memory.
 */
static int falcon_copy_chunk(struct falcon *falcon,
			     phys_addr_t base,
			     unsigned int offset,
			     enum falcon_memory target)
{
	u32 cmd = FALCON_DMATRFCMD_SIZE_256B;

	if (target == FALCON_MEMORY_IMEM)
		cmd |= FALCON_DMATRFCMD_IMEM;

	falcon_writel(falcon, offset, FALCON_DMATRFMOFFS);
	falcon_writel(falcon, base, FALCON_DMATRFFBOFFS);
	falcon_writel(falcon, cmd, FALCON_DMATRFCMD);

	return falcon_dma_wait_idle(falcon);
}

static int falcon_setup_ucode_image(struct falcon *falcon,
				    const struct firmware *ucode_fw)
{
	/* image data is little endian. */
	u32 *ucode_vaddr = falcon->ucode_vaddr;
	struct falcon_ucode_v1 ucode;
	size_t i;

	/* copy the whole thing taking into account endianness */
	for (i = 0; i < ucode_fw->size / sizeof(u32); i++)
		ucode_vaddr[i] = le32_to_cpu(((u32 *)ucode_fw->data)[i]);

	ucode.bin_header = (struct falcon_ucode_bin_header_v1 *)ucode_vaddr;
	/* endian problems would show up right here */
	if (ucode.bin_header->bin_magic != FALCON_FW_MAGIC) {
		dev_err(falcon->dev, "failed to get firmware magic");
		return -EINVAL;
	}
	if (ucode.bin_header->bin_ver != 1) {
		dev_err(falcon->dev, "unsupported firmware version");
		return -ENOENT;
	}
	/* shouldn't be bigger than what firmware thinks */
	if (ucode.bin_header->bin_size > ucode_fw->size) {
		dev_err(falcon->dev, "ucode image size inconsistency");
		return -EINVAL;
	}

	ucode.os_header = (struct falcon_ucode_os_header_v1 *)
		(((void *)ucode_vaddr) +
		 ucode.bin_header->os_bin_header_offset);

	falcon->os.size = ucode.bin_header->os_bin_size;
	falcon->os.bin_data_offset = ucode.bin_header->os_bin_data_offset;
	falcon->os.code_offset = ucode.os_header->os_code_offset;
	falcon->os.code_size   = ucode.os_header->os_code_size;
	falcon->os.data_offset = ucode.os_header->os_data_offset;
	falcon->os.data_size   = ucode.os_header->os_data_size;

	return 0;
}

void falcon_free_ucode(struct falcon *falcon)
{
	if (falcon->ucode_vaddr) {
		falcon->ops->free(falcon, falcon->ucode_size,
				  falcon->ucode_paddr, falcon->ucode_vaddr);
		falcon->ucode_vaddr = NULL;
		falcon->ucode_paddr = 0;
	}
}

int falcon_read_ucode(struct falcon *falcon, const char *ucode_name)
{
	const struct firmware *ucode_fw;
	int err;

	falcon->ucode_paddr = 0;
	falcon->ucode_vaddr = NULL;

	err = request_firmware(&ucode_fw, ucode_name, falcon->dev);
	if (err) {
		dev_err(falcon->dev, "failed to get firmware\n");
		return err;
	}

	falcon->ucode_size = ucode_fw->size;

	falcon->ucode_vaddr = falcon->ops->alloc(falcon, ucode_fw->size,
						 &falcon->ucode_paddr);

	if (!falcon->ucode_vaddr) {
		dev_err(falcon->dev, "dma memory mapping failed");
		err = -ENOMEM;
		goto clean_up;
	}

	err = falcon_setup_ucode_image(falcon, ucode_fw);
	if (err) {
		dev_err(falcon->dev, "failed to parse firmware image\n");
		goto clean_up;
	}

	falcon->ucode_valid = true;

	release_firmware(ucode_fw);

	return 0;

clean_up:
	falcon_free_ucode(falcon);
	release_firmware(ucode_fw);

	return err;
}

int falcon_init(struct falcon *falcon)
{
	/* Check mandatory ops */
	if (!falcon->ops->alloc || !falcon->ops->free)
		return -EINVAL;

	mutex_init(&falcon->lock);

	return 0;
}

void falcon_exit(struct falcon *falcon)
{
	falcon_free_ucode(falcon);
}

int falcon_boot(struct falcon *falcon, const char *ucode_name)
{
	u32 offset;
	int err = 0;

	mutex_lock(&falcon->lock);

	if (falcon->booted)
		goto done;

	if (!falcon->ucode_valid) {
		err = falcon_read_ucode(falcon, ucode_name);
		if (err)
			goto done;
	}

	err = falcon_wait_mem_scrubbing(falcon);
	if (err)
		goto done;

	falcon_writel(falcon, 0, FALCON_DMACTL);

	falcon_writel(falcon, (falcon->ucode_paddr +
			       falcon->os.bin_data_offset) >> 8,
		      FALCON_DMATRFBASE);

	for (offset = 0; offset < falcon->os.data_size; offset += 256)
		falcon_copy_chunk(falcon,
				  falcon->os.data_offset + offset, offset,
				  FALCON_MEMORY_DATA);

	for (offset = 0; offset < falcon->os.code_size; offset += 256)
		falcon_copy_chunk(falcon,
				  falcon->os.code_offset + offset, offset,
				  FALCON_MEMORY_IMEM);

	/* setup falcon interrupts and enable interface */
	falcon_writel(falcon, FALCON_IRQMSET_EXT(0xff) |
			      FALCON_IRQMSET_SWGEN1 |
			      FALCON_IRQMSET_SWGEN0 |
			      FALCON_IRQMSET_EXTERR |
			      FALCON_IRQMSET_HALT |
			      FALCON_IRQMSET_WDTMR,
		      FALCON_IRQMSET);
	falcon_writel(falcon, FALCON_IRQDEST_EXT(0xff) |
			      FALCON_IRQDEST_SWGEN1 |
			      FALCON_IRQDEST_SWGEN0 |
			      FALCON_IRQDEST_EXTERR |
			      FALCON_IRQDEST_HALT,
		      FALCON_IRQDEST);

	falcon_writel(falcon, FALCON_ITFEN_MTHDEN |
			      FALCON_ITFEN_CTXEN,
		      FALCON_ITFEN);

	/* boot falcon */
	falcon_writel(falcon, 0x00000000, FALCON_BOOTVEC);
	falcon_writel(falcon, FALCON_CPUCTL_STARTCPU, FALCON_CPUCTL);

	err = falcon_wait_idle(falcon);
	if (err) {
		dev_err(falcon->dev, "boot failed due to timeout");
		goto done;
	}

	falcon->booted = true;

	dev_info(falcon->dev, "booted");

done:
	mutex_unlock(&falcon->lock);

	return err;
}

int falcon_power_on(struct falcon *falcon)
{
	return 0;
}

int falcon_power_off(struct falcon *falcon)
{
	mutex_lock(&falcon->lock);
	falcon->booted = false;
	mutex_unlock(&falcon->lock);

	return 0;
}

void falcon_execute_method(struct falcon *falcon, u32 method, u32 data)
{
	falcon_writel(falcon, method >> 2, FALCON_UCLASS_METHOD_OFFSET);
	falcon_writel(falcon, data, FALCON_UCLASS_METHOD_DATA);
}
