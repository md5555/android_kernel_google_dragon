/*
 * arch/arm/mach-tegra/timer.c
 *
 * Timer and clock event support for NVIDIA Tegra SoCs
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

#include <linux/init.h>
#include <linux/time.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/clockchips.h>
#include <asm/mach/time.h>
#include <asm/io.h>

#include "ap15/artimer.h"
#include "ap15/artimerus.h"
#include "ap15/arclk_rst.h"
#include "nvcommon.h"
#include "nvrm_drf.h"
#include "nvrm_init.h"
#include "nvrm_module.h"
#include "nvrm_interrupt.h"
#include "nvrm_hardware_access.h"
#include "mach/nvrm_linux.h"
#include "nvos.h"
#include "nvassert.h"

/* CPU complex uses timer instance 2 */
#define NV_CPU_TIMER_INSTANCE   2
#define NV_CPU_SPARE_TIMER_INSTANCE  3 

static volatile NvU8 *s_NvTimerTick;
static volatile NvU8 *s_NvTimerUsec;

static irqreturn_t NvTimerIntrHandler(
    int irq,
    void *dev_id)
{
    struct clock_event_device *dev =
        (struct clock_event_device *)dev_id;
    NV_WRITE32(s_NvTimerTick + TIMER_TMR_PCR_0,
               NV_DRF_NUM(TIMER, TMR_PCR, INTR_CLR, 1));
    BUG_ON(dev == NULL);
    dev->event_handler(dev);
    return IRQ_HANDLED;
}

static inline void NvTimerUpdateHardware(
    unsigned long cycles,
    int periodic)
{
    NvU32 v;
    v  = NV_DRF_NUM(TIMER, TMR_PTV, TMR_PTV, cycles);
    if (periodic) {
        v |= NV_DRF_DEF(TIMER, TMR_PTV, PER, ENABLE);
    }
    v |= NV_DRF_DEF(TIMER, TMR_PTV, EN, ENABLE);
    NV_WRITE32(s_NvTimerTick + TIMER_TMR_PTV_0, v);
}

static inline int NvTimerSetEvent(
    unsigned long cycles,
    struct clock_event_device *dev)
{
    NvTimerUpdateHardware(cycles, 0);
    return 0;
}

static inline void NvTimerSetMode(
    enum clock_event_mode mode,
    struct clock_event_device *dev)
{
 
    NV_WRITE32(s_NvTimerTick + TIMER_TMR_PTV_0, 0);

    switch (mode) {
        case CLOCK_EVT_MODE_PERIODIC:
            NvTimerUpdateHardware((1000000/HZ)-1, 1);
            break;
        case CLOCK_EVT_MODE_ONESHOT:
            break;
        case CLOCK_EVT_MODE_RESUME:
        case CLOCK_EVT_MODE_SHUTDOWN:
        case CLOCK_EVT_MODE_UNUSED:
            break;
    }
}

static cycle_t NvReadTimerUs(void)
{
    return (cycle_t) NV_READ32(s_NvTimerUsec + TIMERUS_CNTR_1US_0);
}

/* Timers on Tegra run at microsecond resolution, so the
 * best way to approximate a divide by 1000 (ns-to-cycle)
 * and maintain a decent amount of precision is to multiply
 * by 16777 and shift right by 24.  This causes 1000.01ns
 * in kernel time to represent 1000ns in actual time. */

static struct clock_event_device s_NvTimer = {
    .name           = "timer0",
    .rating         = 300,
    .features       = CLOCK_EVT_FEAT_ONESHOT,
    .mult           = 16777,
    .shift          = 24,
    .set_next_event = NvTimerSetEvent,
    .set_mode       = NvTimerSetMode,
};

static struct irqaction s_NvTimerIrq = {
    .name    = "timer0",
    .flags   = IRQF_DISABLED | IRQF_TIMER | IRQF_TRIGGER_HIGH,
    .handler = NvTimerIntrHandler,
    .dev_id  = &s_NvTimer,
};

/*  Converting from clock-cycles to nanoseconds is trivial -
 *  just multiply by 1000 */
static struct clocksource s_NvClockUs =
{
    .name    = "timer_us",
    .rating  = 300,
    .read   = NvReadTimerUs,
    .mask   = 0xFFFFFFFFUL,
    .mult   = 1000,
    .shift  = 0,
    .flags  = CLOCK_SOURCE_IS_CONTINUOUS,
};

static void NvSpareTimerInit(void);

static void __init tegra_timer_init(void)
{
    NvRmPhysAddr Phys;
    NvU32 Len;
    volatile NvU8 *pCar = NULL;
    NvU32 OscFreq;

    NvRmModuleGetBaseAddress(s_hRmGlobal,
        NVRM_MODULE_ID(NvRmPrivModuleID_ClockAndReset, 0), &Phys, &Len);
    if (NvRmPhysicalMemMap(Phys, Len, NVOS_MEM_READ_WRITE,
            NvOsMemAttribute_Uncached, (void **)&pCar)!=NvSuccess)
    {
        NV_ASSERT(!"Error: Unable to get clock controller base address\n");
    }

    OscFreq = NV_READ32(pCar + CLK_RST_CONTROLLER_OSC_CTRL_0);

    NvRmModuleGetBaseAddress(s_hRmGlobal, 
        NVRM_MODULE_ID(NvRmModuleID_TimerUs, 0), &Phys, &Len);

    if (NvRmPhysicalMemMap(Phys, Len, NVOS_MEM_READ_WRITE,
            NvOsMemAttribute_Uncached, (void **)&s_NvTimerUsec)!=NvSuccess)
    {
        NV_ASSERT(!"ERROR: Unable to get microsecod timer base address\n");
    }

    switch (NV_DRF_VAL(CLK_RST_CONTROLLER, OSC_CTRL, OSC_FREQ, OscFreq))
    {
    case 0:  // 13MHz
        NV_WRITE32(s_NvTimerUsec + TIMERUS_USEC_CFG_0,
                   NV_DRF_NUM(TIMERUS, USEC_CFG, USEC_DIVIDEND, 0) |
                   NV_DRF_NUM(TIMERUS, USEC_CFG, USEC_DIVISOR, 12));
        break;
    case 1:  // 19.2 MHz
        NV_WRITE32(s_NvTimerUsec + TIMERUS_USEC_CFG_0,
                   NV_DRF_NUM(TIMERUS, USEC_CFG, USEC_DIVIDEND, 4) |
                   NV_DRF_NUM(TIMERUS, USEC_CFG, USEC_DIVISOR, 95));
        break;
    case 2: // 12 MHz
        NV_WRITE32(s_NvTimerUsec + TIMERUS_USEC_CFG_0,
                   NV_DRF_NUM(TIMERUS, USEC_CFG, USEC_DIVIDEND, 0) |
                   NV_DRF_NUM(TIMERUS, USEC_CFG, USEC_DIVISOR, 11));
        break;
    case 3:  // 26 MHz
    default:
        NV_WRITE32(s_NvTimerUsec + TIMERUS_USEC_CFG_0,
                   NV_DRF_NUM(TIMERUS, USEC_CFG, USEC_DIVIDEND, 0) |
                   NV_DRF_NUM(TIMERUS, USEC_CFG, USEC_DIVISOR, 25));
        break;
    }
    if (clocksource_register(&s_NvClockUs)) {
        NV_ASSERT(!"ERROR: Could not register microsecond timer\n");
    }

    NvRmModuleGetBaseAddress(s_hRmGlobal,
        NVRM_MODULE_ID(NvRmModuleID_Timer, NV_CPU_TIMER_INSTANCE), &Phys, &Len);
    if (NvRmPhysicalMemMap(Phys, Len, NVOS_MEM_READ_WRITE,
            NvOsMemAttribute_Uncached,
            (void **)&s_NvTimerTick)!=NvSuccess) {
        NV_ASSERT(!"ERROR: Unable to get tick-timer base address\n");
    }

    s_NvTimerIrq.irq = NvRmGetIrqForLogicalInterrupt(s_hRmGlobal,
         NVRM_MODULE_ID(NvRmModuleID_Timer, NV_CPU_TIMER_INSTANCE), 0);

    if (setup_irq(s_NvTimerIrq.irq, &s_NvTimerIrq)) {
        NV_ASSERT(!"ERROR: Could not configure timer IRQ\n");
    }


    s_NvTimer.max_delta_ns =
        clockevent_delta2ns(TIMER_TMR_PTV_0_TMR_PTV_DEFAULT_MASK,
                            &s_NvTimer);
    s_NvTimer.min_delta_ns = clockevent_delta2ns(1, &s_NvTimer);
    s_NvTimer.cpumask = cpu_all_mask;
    s_NvTimer.irq = s_NvTimerIrq.irq;
    clockevents_register_device(&s_NvTimer);

    NvSpareTimerInit();
}

struct sys_timer tegra_timer = 
{
    .init = tegra_timer_init,
};

static volatile NvU8 *s_NvSpareTimerTick;

static irqreturn_t NvSpareTimerIntrHandler(
    int irq,
    void *driver_data)
{
    NV_WRITE32(s_NvSpareTimerTick + TIMER_TMR_PCR_0, 
        NV_DRF_NUM(TIMER, TMR_PCR, INTR_CLR, 1));

    return IRQ_HANDLED;
}

static struct irqaction s_SpareTimerIrq = {
    .name    = "spare_timer",
    .flags   = IRQF_DISABLED,
    .handler = NvSpareTimerIntrHandler,
    .dev_id  = &s_NvSpareTimerTick,
};

void NvSpareTimerTrigger(unsigned long cycles)
{
    NvU32 v;
    v  = NV_DRF_NUM(TIMER, TMR_PTV, TMR_PTV, cycles);
    v |= NV_DRF_DEF(TIMER, TMR_PTV, EN, ENABLE);
    NV_WRITE32(s_NvSpareTimerTick + TIMER_TMR_PTV_0, v);
}

static void NvSpareTimerInit(void)
{
    int irq;
    NvRmPhysAddr Phys;
    NvU32 Len;

    NvRmModuleGetBaseAddress(s_hRmGlobal,
        NVRM_MODULE_ID(NvRmModuleID_Timer, NV_CPU_SPARE_TIMER_INSTANCE), &Phys, &Len);

    if (NvRmPhysicalMemMap(Phys, Len, NVOS_MEM_READ_WRITE,
            NvOsMemAttribute_Uncached,
            (void **)&s_NvSpareTimerTick)!=NvSuccess) {
        NV_ASSERT(!"ERROR: Unable to get tick-timer base address\n");
    }

    irq = NvRmGetIrqForLogicalInterrupt(s_hRmGlobal,
         NVRM_MODULE_ID(NvRmModuleID_Timer, NV_CPU_SPARE_TIMER_INSTANCE), 0);

    if (setup_irq(irq, &s_SpareTimerIrq)) {
        NV_ASSERT(!"ERROR: Could not configure timer IRQ\n");
    }
}

