/*
 * arch/arm/mm/cache-tegra-cmc.c
 *
 * L2 cache controller support and cacheable pagetable walks for Tegra 1x SoCs
 *
 * Copyright (c) 2008-2009, NVIDIA Corporation.
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
#include <linux/spinlock.h>
#include <asm/page.h>
#include <asm/pgtable.h>
#include <asm/cacheflush.h>
#include <asm/pgtable-hwdef.h>

#include "mach/nvrm_linux.h"

#include "ap15/arapb_cmc.h"
#include "ap15/arapb_misc.h"
#include "ap15/arclk_rst.h"
#include "ap16/armselect.h"

#include "nvcommon.h"
#include "nvrm_hardware_access.h"
#include "nvrm_drf.h"
#include "nvrm_module.h"
#include "nvrm_init.h"

#define ENABLE_L2_PTW 1
#define CACHE_LINE_SIZE 32
#define CACHE_LINE_MASK (CACHE_LINE_SIZE-1)

static volatile NvU8 * s_pCmc = NULL;
static int s_HasL2Ptw = 0;

static DEFINE_SPINLOCK(s_CmcLock);

#define L2_POLL(VA)							\
	do {								\
		unsigned int _x;					\
		do {							\
			_x = NV_READ32((VA)+APB_CMC_MAINT_2_0);		\
		} while (NV_DRF_VAL(APB_CMC, MAINT_2, OPCODE, _x)!=0);	\
	} while (0);


static inline unsigned long tegra_cmc_mmu_remap(void *va)
{
	unsigned long addr = (unsigned long)va;
	unsigned long ofs = addr & ~(PAGE_MASK);
	unsigned long phys;
	unsigned long flags;

	addr &= PAGE_MASK;

	local_irqsave(flags);
	asm volatile("mcr p15, 0, %0, c7, c8, 0" : : "r"(addr) : "cc");
	asm volatile("mrc p15, 0, %0, c7, c4, 0" : "=r"(phys) : : "cc");
	local_irqrestore(flags);

	phys &= PAGE_MASK;
	phys |= ofs;

	return phys;
}

static void tegra_cmc_range_op(unsigned long start, unsigned long end,
	unsigned int operation)
{
	unsigned long flags;

	if (!s_pCmc)
		return;

	start &= ~CACHE_LINE_MASK;
	spin_lock_irqsave(&s_CmcLock, flags);

	for ( ; start<end ; start += CACHE_LINE_SIZE) {
		NV_WRITE32(s_pCmc + APB_CMC_MAINT_0_0, pfn);
		NV_WRITE32(s_pCmc + APB_CMC_MAINT_2_0, operation);
		L2_POLL(s_pCmc);
	} while (start<end);

	spin_unlock_irqrestore(&s_CmcLock, flags);
	dsb();
}

static void tegra_cmc_inv_range(unsigned long start, unsigned long end)
{
 	if (start & (CACHE_LINE_SIZE-1)) {
		start &= ~(CACHE_LINE_SIZE-1);
		tegra_cmc_range_op(start, start+CACHE_LINE_SIZE,
			APB_CMC_MAINT_2_0_OPCODE_CLEAN_INVALID_PHY);
		start += CACHE_LINE_SIZE-1;
	}
 	if (end & (CACHE_LINE_SIZE-1)) {
		end &= ~(CACHE_LINE_SIZE-1);
		tegra_cmc_range_op(end, end+CACHE_LINE_SIZE,
			APB_CMC_MAINT_2_0_OPCODE_CLEAN_INVALID_PHY);
	}

	tegra_cmc_range_op(start, end, APB_CMC_MAINT_2_0_OPCODE_INVALID_PHY);
}

static void tegra_cmc_clean_range(unsigned long start, unsigned long end)
{
	tegra_cmc_range_op(start, end, APB_CMC_MAINT_2_0_OPCODE_CLEAN_PHY);
}

static void tegra_cmc_flush_range(unsigned long start, unsigned long end)
{
	tegra_cmc_range_op(start, end,
		APB_CMC_MAINT_2_0_OPCODE_CLEAN_INVALID_PHY);
}

void cpu_v6_set_pte_ext(pte_t *ptep, pte_t pte, unsigned int ext)
{
#define WBWA (PTE_EXT_TEX(5) | PTE_CACHEABLE | PTE_BUFFERABLE)
#define SHAREDDEV (PTE_BUFFERABLE)
#define SO (0)
	static const NvU32 PteMasks[16] = { 
		SHAREDDEV, // L_PTE_MT_UNCACHED
		SHAREDDEV, // L_PTE_MT_BUFFERABLE
		SHAREDDEV, // L_PTE_MT_WRITETROUGH
		WBWA,	  // L_PTE_MT_WRITEBACK
		SHAREDDEV, // L_PTE_MT_DEV_SHARED
		0,
		WBWA,	  // L_PTE_MT_MINICACHE
		0,
		SHAREDDEV, // L_PTE_MT_DEV_WC
		0,
		SHAREDDEV, // L_PTE_MTD_DEV_CACHED
		SHAREDDEV, // L_PTE_MTD_DEV_NONSHARED
		0,
		0,
		0
	};
		
		
	unsigned long p;

	*ptep = pte;
	ptep = (pte_t*)(((char*)ptep)-2048);
	p = pte_val(pte);
	p &= ~(0x3fc | PTE_TYPE_MASK);
	p |= (ext | PTE_EXT_AP0 | 2);
	p |= PteMasks[(pte_val(pte) & L_PTE_MT_MASK)>>2];

	if (!(pte_val(pte) & L_PTE_WRITE) ||
		!(pte_val(pte) & L_PTE_DIRTY))
		p |= PTE_EXT_APX;

	if (pte_val(pte) & L_PTE_USER)
	{
		p |= PTE_EXT_AP1;
		if (p & PTE_EXT_APX)
			p &= ~(PTE_EXT_AP0 | PTE_EXT_APX);
	}

	if (!(pte_val(pte) & L_PTE_EXEC))
		p |= PTE_EXT_XN;



	if (!(pte_val(pte) & L_PTE_YOUNG) ||
		!(pte_val(pte) & L_PTE_PRESENT))
		p = 0;
		

	pte_val(pte) = p;
	*ptep = pte;


	asm volatile("mcr p15, 0, %0, c7, c10, 1" : : "r"(ptep) : "cc");
	dsb();

	if (!s_HasL2Ptw) {
		unsigned long addr = tegra_cmc_mmu_remap(ptep);
		tegra_cmc_clean_range(addr, addr+sizeof(*ptep));
	}
}

void tegra_cmc_disable(void)
{
	if (s_pCmc) {
		NvU32 Config;
		unsigned long flags;

		spin_lock_irqsave(&s_CmcLock, flags);
		NV_WRITE32(s_pCmc + APB_CMC_LOCK_0,
			NV_DRF_NUM(APB_CMC, LOCK, LOCK_BITMAP, ~0UL));
		NV_WRITE32(s_pCmc + APB_CMC_MAINT_2_0,
			NV_DRF_DEF(APB_CMC, MAINT_2, OPCODE, CLEAN_INVALID_WAY)|
			NV_DRF_NUM(APB_CMC, MAINT_2, WAY_BITMAP, ~0UL));
		L2_POLL(s_pCmc);
		Config = NV_READ32(s_pCmc + APB_CMC_CONFIG_0);
		Config = NV_FLD_SET_DRF_NUM(APB_CMC, CONFIG,
			ENABLE_CACHE, 0, Config);
		Config = NV_FLD_SET_DRF_NUM(APB_CMC, CONFIG,
			ENABLE_STEERING,0, Config);
		NV_WRITE32(s_pCmc + APB_CMC_CONFIG_0, Config);
		NV_WRITE32(s_pCmc + APB_CMC_LOCK_0, 0);
		s_pCmc = NULL;
		spin_unlock_irqrestore(&s_CmcLock, flags);

	}
}

void tegra_cmc_clean_pmd(pmd_t *pmd)
{
	unsigned long mva = (unsigned long)pmd & ~CACHE_LINE_MASK;
	unsigned long mpa = tegra_cmc_mmu_remap((void*)mva);

	asm volatile("mcr p15, 0, %0, c7, c10, 1" : : "r"(mva) : "cc");
	dsb();
	if (!s_HasL2Ptw) {
		tegra_cmc_clean_range(mpa, mpa+sizeof(*pmd));
	}
}

/*  Enables the L2 and L2 PTW, if present  */
void __init tegra_cmc_enable(void)
{
	volatile NvU8 *pCar = NULL;
	volatile NvU8 *pMsl = NULL;
	volatile NvU8 *pMisc = NULL;
	NvU32 temp;
	NvRmPhysAddr pa;

	if (s_pCmc)
		return;

	NvRmModuleGetBaseAddress(s_hRmGlobal,
		NVRM_MODULE_ID(NvRmPrivModuleID_ClockAndReset, 0), &pa, &temp);

	if (NvRmPhysicalMemMap(pa, temp, NVOS_MEM_READ_WRITE,
			NvOsMemAttribute_Uncached, (void**)&pCar) != NvSuccess)
		return;

	NvRmModuleGetBaseAddress(s_hRmGlobal,
		NVRM_MODULE_ID(NvRmModuleID_Misc, 0), &pa, &temp);

	if (NvRmPhysicalMemMap(pa, temp, NVOS_MEM_READ_WRITE,
		NvOsMemAttribute_Uncached, (void**)&pMisc) != NvSuccess)
		return;

	/*  FIXME: Relocation table for MSelect is wrong, so just
	 *  hack the physical address in here for now */
	if (NvRmPhysicalMemMap(0x50042000UL, 4096, NVOS_MEM_READ_WRITE,
		NvOsMemAttribute_Uncached, (void**)&pMsl) != NvSuccess)
		return;

	NvRmModuleGetBaseAddress(s_hRmGlobal,
		NVRM_MODULE_ID(NvRmModuleID_CacheMemCtrl, 0), &pa, &temp);

	if (NvRmPhysicalMemMap(pa, temp, NVOS_MEM_READ_WRITE,
		NvOsMemAttribute_Uncached, (void**)&s_pCmc) != NvSuccess)
		return;
		

	//  Take L2 out of reset.
	temp = NV_READ32(pCar + CLK_RST_CONTROLLER_RST_DEVICES_L_0);
	temp = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, RST_DEVICES_L,
		SWR_CACHE1_RST, 0, temp);
	NV_WRITE32(pCar + CLK_RST_CONTROLLER_RST_DEVICES_L_0, temp);

	//  Enable clock to CMC and cache RAMs
	temp = NV_READ32(pCar + CLK_RST_CONTROLLER_CLK_OUT_ENB_L_0);
	temp = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, CLK_OUT_ENB_L,
		CLK_ENB_CACHE1, 1, temp);
	NV_WRITE32(pCar + CLK_RST_CONTROLLER_CLK_OUT_ENB_L_0, temp);

	temp = NV_READ32(pCar + CLK_RST_CONTROLLER_MISC_CLK_ENB_0);
	temp = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, MISC_CLK_ENB,
		CLK_ENB_CRAM1, 1, temp);
	NV_WRITE32(pCar + CLK_RST_CONTROLLER_MISC_CLK_ENB_0, temp);

	temp = 0;
	asm volatile("mcr p15, 0, %0, c7, c14, 0" : : "r" (temp) : "memory");
	dsb();

	//  invalidate all ways in L2
	temp = NV_DRF_NUM(APB_CMC, LOCK, LOCK_BITMAP, ~0UL);
	NV_WRITE32(s_pCmc + APB_CMC_LOCK_0, temp);

	temp = NV_DRF_DEF(APB_CMC, MAINT_2, OPCODE, INVALID_WAY) |
		NV_DRF_NUM(APB_CMC, MAINT_2, WAY_BITMAP, ~0UL);
	NV_WRITE32(s_pCmc + APB_CMC_MAINT_2_0, temp);
	L2_POLL(s_pCmc);

	temp = NV_READ32(s_pCmc + APB_CMC_CONFIG_0);
	temp = NV_FLD_SET_DRF_NUM(APB_CMC, CONFIG, ENABLE_STEERING, 1, temp);
	temp = NV_FLD_SET_DRF_NUM(APB_CMC, CONFIG, ENABLE_CACHE, 1, temp);
	NV_WRITE32(s_pCmc + APB_CMC_CONFIG_0, temp);

	NV_WRITE32(s_pCmc + APB_CMC_LOCK_0, 0);

	temp = NV_READ32(pMisc + APB_MISC_GP_HIDREV_0);
	if (NV_DRF_VAL(APB_MISC_GP, HIDREV, CHIPID, temp)>=0x16 &&
		NV_DRF_VAL(APB_MISC_GP, HIDREV, MAJORREV, temp)!=0 &&
		(NV_DRF_VAL(APB_MISC_GP, HIDREV, MAJORREV, temp)>0x1 ||
		NV_DRF_VAL(APB_MISC_GP, HIDREV, MINORREV, temp)>=0x3)) {
#if ENABLE_L2_PTW
		temp = NV_READ32(pMsl + MSELECT_CONFIG_0);
		temp = NV_FLD_SET_DRF_NUM(MSELECT, CONFIG,
			ENABLE_PTW_L2, 1, temp);
		NV_WRITE32(pMsl + MSELECT_CONFIG_0, temp);
		//  update TTBR flags for TTB1 to reflect outer WBWA
		asm volatile("mrc p15, 0, %0, c2, c0, 1" : "=r"(temp) : : "cc");
		temp |= 0x8;  // WBWA
		asm volatile("mcr p15, 0, %0, c2, c0, 1" : : "r"(temp) : "cc");
		//  update TTBR flags for TTB0 to reflect outer WBWA
		asm volatile("mrc p15, 0, %0, c2, c0, 0" : "=r"(temp) : : "cc");
		temp |= 0x8;
		asm volatile("mcr p15, 0, %0, c2, c0, 0" : : "r"(temp) : "cc");
		s_HasL2Ptw = 1;
#else
		printk("L2 Page table walk supported, not enabled\n");
#endif
	}

	outer_cache.inv_range = tegra_cmc_inv_range;
	outer_cache.clean_range = tegra_cmc_clean_range;
	outer_cache.flush_range = tegra_cmc_flush_range;
}

