/* Copyright (C) 2015 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include "tegra_usb_phy.h"

#define   AHB_ARBITRATION_PRIORITY_CTRL           0x4
#define   AHB_PRIORITY_WEIGHT(x)        (((x) & 0x7) << 29)
#define   PRIORITY_SELECT_USB   BIT(6)
#define   PRIORITY_SELECT_USB2  BIT(18)
#define   PRIORITY_SELECT_USB3  BIT(17)
#define   PRIORITY_SELECT_SE BIT(14)

#define AHB_GIZMO_AHB_MEM               0xc
#define   ENB_FAST_REARBITRATE  BIT(2)
#define   DONT_SPLIT_AHB_WR     BIT(7)
#define   WR_WAIT_COMMIT_ON_1K  BIT(8)
#define   EN_USB_WAIT_COMMIT_ON_1K_STALL        BIT(9)

#define   RECOVERY_MODE BIT(31)
#define   BOOTLOADER_MODE       BIT(30)
#define   FORCED_RECOVERY_MODE  BIT(1)

#define AHB_GIZMO_USB           0x1c
#define AHB_GIZMO_USB2          0x78
#define AHB_GIZMO_USB3          0x7c
#define AHB_GIZMO_SE            0x4c
#define   IMMEDIATE     BIT(18)

#define AHB_MEM_PREFETCH_CFG5   0xc8
#define AHB_MEM_PREFETCH_CFG3   0xe0
#define AHB_MEM_PREFETCH_CFG4   0xe4
#define AHB_MEM_PREFETCH_CFG1   0xec
#define AHB_MEM_PREFETCH_CFG2   0xf0
#define AHB_MEM_PREFETCH_CFG6   0xcc
#define   PREFETCH_ENB  BIT(31)
#define   MST_ID(x)     (((x) & 0x1f) << 26)
#define   AHBDMA_MST_ID MST_ID(5)
#define   USB_MST_ID    MST_ID(6)
#define SDMMC4_MST_ID   MST_ID(12)
#define   USB2_MST_ID   MST_ID(18)
#define   USB3_MST_ID   MST_ID(17)
#define   SE_MST_ID     MST_ID(14)
#define   ADDR_BNDRY(x) (((x) & 0xf) << 21)
#define   INACTIVITY_TIMEOUT(x) (((x) & 0xffff) << 0)

static DEFINE_SPINLOCK(ahb_lock);

void ahb_gizmo_writel(unsigned long val, void __iomem *reg)
{
        unsigned long check;
        int retry = 10;
        unsigned long flags;

        /* Read and check if write is successful,
         * if val doesn't match with read, retry write.
         */
        spin_lock_irqsave(&ahb_lock, flags);
        do {
                writel(val, reg);
                check = readl(reg);
                if (likely(check == val))
                        break;
                else
                        pr_err("AHB register access fail for reg\n");
        } while (--retry);
        spin_unlock_irqrestore(&ahb_lock, flags);
}
EXPORT_SYMBOL_GPL(ahb_gizmo_writel);

static inline unsigned long gizmo_readl(unsigned long offset, struct tegra_usb_phy *phy)
{
        return readl(phy->gizmo_base + offset);
}

static inline void gizmo_writel(unsigned long value, unsigned long offset, struct tegra_usb_phy *phy)
{
        writel(value, phy->gizmo_base + offset);
}


void tegra_init_ahb_gizmo_settings(struct tegra_usb_phy *phy)
{
        unsigned long val;

        val = gizmo_readl(AHB_GIZMO_AHB_MEM, phy);
        val |= ENB_FAST_REARBITRATE | IMMEDIATE | DONT_SPLIT_AHB_WR;

      //  if (tegra_get_chip_id() == TEGRA_CHIPID_TEGRA11)
        //        val |= WR_WAIT_COMMIT_ON_1K;
#if defined(CONFIG_ARCH_TEGRA_14x_SOC) || defined(CONFIG_ARCH_TEGRA_12x_SOC)
        val |= WR_WAIT_COMMIT_ON_1K | EN_USB_WAIT_COMMIT_ON_1K_STALL;
#endif
        gizmo_writel(val, AHB_GIZMO_AHB_MEM, phy);

        val = gizmo_readl(AHB_GIZMO_USB, phy);
        val |= IMMEDIATE;
        gizmo_writel(val, AHB_GIZMO_USB, phy);

        val = gizmo_readl(AHB_GIZMO_USB2, phy);
        val |= IMMEDIATE;
        gizmo_writel(val, AHB_GIZMO_USB2, phy);

        val = gizmo_readl(AHB_GIZMO_USB3, phy);
        val |= IMMEDIATE;
        gizmo_writel(val, AHB_GIZMO_USB3, phy);

        val = gizmo_readl(AHB_ARBITRATION_PRIORITY_CTRL, phy);
        val |= PRIORITY_SELECT_USB | PRIORITY_SELECT_USB2 | PRIORITY_SELECT_USB3
                                | AHB_PRIORITY_WEIGHT(7);
#if !defined(CONFIG_ARCH_TEGRA_2x_SOC) && !defined(CONFIG_ARCH_TEGRA_3x_SOC)
        val |= PRIORITY_SELECT_SE;
#endif


        gizmo_writel(val, AHB_ARBITRATION_PRIORITY_CTRL, phy);

        val = gizmo_readl(AHB_MEM_PREFETCH_CFG1, phy);
        val &= ~MST_ID(~0);
        val |= PREFETCH_ENB | AHBDMA_MST_ID |
                ADDR_BNDRY(0xc) | INACTIVITY_TIMEOUT(0x1000);
        ahb_gizmo_writel(val,
                                phy->gizmo_base + AHB_MEM_PREFETCH_CFG1);

        val = gizmo_readl(AHB_MEM_PREFETCH_CFG2, phy);
        val &= ~MST_ID(~0);
        val |= PREFETCH_ENB | USB_MST_ID | ADDR_BNDRY(0xc) |
                INACTIVITY_TIMEOUT(0x1000);
        ahb_gizmo_writel(val,
                                phy->gizmo_base + AHB_MEM_PREFETCH_CFG2);

        val = gizmo_readl(AHB_MEM_PREFETCH_CFG3, phy);
        val &= ~MST_ID(~0);
        val |= PREFETCH_ENB | USB3_MST_ID | ADDR_BNDRY(0xc) |
                INACTIVITY_TIMEOUT(0x1000);
        ahb_gizmo_writel(val,
                                phy->gizmo_base + AHB_MEM_PREFETCH_CFG3);

        val = gizmo_readl(AHB_MEM_PREFETCH_CFG4, phy);
        val &= ~MST_ID(~0);
        val |= PREFETCH_ENB | USB2_MST_ID | ADDR_BNDRY(0xc) |
                INACTIVITY_TIMEOUT(0x1000);
        ahb_gizmo_writel(val,
                                phy->gizmo_base + AHB_MEM_PREFETCH_CFG4);

        /*
         * SDMMC controller is removed from AHB interface in T124 and
         * later versions of Tegra. Configure AHB prefetcher for SDMMC4
         * in T11x and T14x SOCs.
         */
#if defined(CONFIG_ARCH_TEGRA_11x_SOC) || defined(CONFIG_ARCH_TEGRA_14x_SOC)
        val = gizmo_readl(AHB_MEM_PREFETCH_CFG5, phy);
        val &= ~MST_ID(~0);
        val |= PREFETCH_ENB | SDMMC4_MST_ID;
        val &= ~SDMMC4_MST_ID;
        ahb_gizmo_writel(val,
                                phy->gizmo_base + AHB_MEM_PREFETCH_CFG5);
#endif

#if !defined(CONFIG_ARCH_TEGRA_2x_SOC) && !defined(CONFIG_ARCH_TEGRA_3x_SOC)
        val = gizmo_readl(AHB_MEM_PREFETCH_CFG6, phy);
        val &= ~MST_ID(~0);
        val |= PREFETCH_ENB | SE_MST_ID | ADDR_BNDRY(0xc) |
                INACTIVITY_TIMEOUT(0x1000);
        ahb_gizmo_writel(val,
                                phy->gizmo_base + AHB_MEM_PREFETCH_CFG6);
#endif
}
EXPORT_SYMBOL_GPL(tegra_init_ahb_gizmo_settings);
