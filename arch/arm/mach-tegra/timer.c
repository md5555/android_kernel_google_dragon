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
#include <mach/iomap.h>

struct tegra_timer {
	void __iomem *mmio;
	struct clock_event_device event;
};

#define CLK_RST_CONTROLLER_OSC_CTRL_0 0x50

#define TIMER1_OFFS  0x00  /* reserved for AVP */
#define TIMER2_OFFS  0x08  /* reserved for AVP */
#define TIMER3_OFFS  0x50  /* used as OS CPU event timer */
#define TIMER4_OFFS  0x58  /* reserved as LP2 wakeup trigger */

#define TIMER_TMR_PTV_0 0x0
#define TIMER_TMR_PCR_0 0x4

#define TIMERUS_OFFS 0x10
#define TIMERUS_CNTR_1US_0 0x0
#define TIMERUS_USEC_CFG_0 0x4

static int tegra_event_set_next(unsigned long cycles,
	struct clock_event_device *dev)
{
	struct tegra_timer *tmr = container_of(dev, struct tegra_timer, event);
	u32 reg;

	reg = 0x80000000 | ((1000000/HZ)*(cycles+1)-1);
	writel(reg, tmr->mmio + TIMER_TMR_PTV_0);

	return 0;
}

static void tegra_event_set_mode(enum clock_event_mode mode,
	struct clock_event_device *dev)
{
	struct tegra_timer *tmr = container_of(dev, struct tegra_timer, event);
	u32 reg;

	writel(0, tmr->mmio + TIMER_TMR_PTV_0);

	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		reg = 0xc0000000ul | ((1000000/HZ)-1);
		writel(reg, tmr->mmio + TIMER_TMR_PTV_0);
		break;
	case CLOCK_EVT_MODE_ONESHOT:
		break;
	case CLOCK_EVT_MODE_RESUME:
	case CLOCK_EVT_MODE_SHUTDOWN:
	case CLOCK_EVT_MODE_UNUSED:
		break;
    }
}

static struct tegra_timer tegra_clockevent = {
	.mmio	= IO_ADDRESS(TEGRA_TMR1_BASE + TIMER3_OFFS),
	.event	= {
		.name	= "timer_event",
		.rating	= 300,
		.features = CLOCK_EVT_FEAT_ONESHOT | CLOCK_EVT_FEAT_PERIODIC,
		.irq	= INT_TMR3,
		.mult	= 16777,
		.shift	= 24,
		.set_next_event = tegra_event_set_next,
		.set_mode	= tegra_event_set_mode,
	},
};

static irqreturn_t tegra_clockevent_interrupt(int irq, void *dev_id)
{
	struct tegra_timer *tmr = (struct tegra_timer *)dev_id;

	writel(1<<30, tmr->mmio + TIMER_TMR_PCR_0);
	tmr->event.event_handler(&tmr->event);
	return IRQ_HANDLED;
}

static struct irqaction tegra_clockevent_irq = {
	.name		= "timer_event",
	.irq		= INT_TMR3,
	.flags		= IRQF_DISABLED | IRQF_TIMER | IRQF_TRIGGER_HIGH,
	.handler	= tegra_clockevent_interrupt,
	.dev_id		= &tegra_clockevent,
};

static cycle_t tegra_clocksource_read(void)
{
	void __iomem *tmr = IO_ADDRESS(TEGRA_TMR1_BASE + TIMERUS_OFFS);
	return (cycle_t) readl(tmr + TIMERUS_CNTR_1US_0);
}

static struct clocksource tegra_clocksource =
{
	.name	= "timer_us",
	.rating	= 300,
	.read	= tegra_clocksource_read,
	.mask	= 0xFFFFFFFFUL,
	.mult	= 1000,
	.shift	= 0,
	.flags	= CLOCK_SOURCE_IS_CONTINUOUS,
};

static irqreturn_t tegra_lp2wake_interrupt(int irq, void *dev_id)
{
	void __iomem *tmr = (void __iomem *)dev_id;
	writel(1<<30, tmr + TIMER_TMR_PCR_0);
	return IRQ_HANDLED;
}

static struct irqaction tegra_lp2wake_irq = {
	.name		= "timer_lp2wake",
	.irq		= INT_TMR4,
	.flags		= IRQF_DISABLED,
	.handler	= tegra_lp2wake_interrupt,
	.dev_id		= IO_ADDRESS(TEGRA_TMR1_BASE + TIMER4_OFFS),
};

void tegra_lp2_set_trigger(unsigned long cycles)
{
	void __iomem *tmr = (void __iomem*)tegra_lp2wake_irq.dev_id;

	writel(0, tmr + TIMER_TMR_PTV_0);
	if (cycles) {
		u32 reg = 0x80000000ul | min(0x1ffffffful, cycles);
		writel(reg, tmr + TIMER_TMR_PTV_0);
	}
}

static unsigned long measure_input_freq(unsigned int *m, unsigned int *n)
{
	void __iomem *clk_rst = IO_ADDRESS(TEGRA_CLK_RESET_BASE);
	unsigned long osc = readl(clk_rst + CLK_RST_CONTROLLER_OSC_CTRL_0);
	osc >>= 30;

	switch (osc) {
	case 0: if (m && n) { *m=1; *n=13; } return 13000;
	case 1: if (m && n) { *m=5; *n=96; } return 19200;
	case 2: if (m && n) { *m=1; *n=12; } return 12000;
	case 3: if (m && n) { *m=1; *n=26; } return 26000;
	}
	return 0;
}

static void __init tegra_timer_init(void)
{
	void __iomem *tmr;
	unsigned int m, n;
	unsigned long val;

	tmr = IO_ADDRESS(TEGRA_TMR1_BASE + TIMERUS_OFFS);
	val = measure_input_freq(&m, &n);

	val = ((m-1)<<8) | (n-1);

	writel(val, tmr + TIMERUS_USEC_CFG_0);

	if (clocksource_register(&tegra_clocksource)) {
		pr_err("Failed to register clocksource\n");
		BUG();
	}

	tegra_clockevent.event.max_delta_ns =
		clockevent_delta2ns(0x1ffffffful, &tegra_clockevent.event);

	tegra_clockevent.event.min_delta_ns =
		clockevent_delta2ns(1, &tegra_clockevent.event);

	tegra_clockevent.event.cpumask = cpu_all_mask;

	if (setup_irq(tegra_clockevent_irq.irq, &tegra_clockevent_irq)) {
		pr_err("Failed to register clockevent IRQ\n");
		BUG();
	}
	if (setup_irq(tegra_lp2wake_irq.irq, &tegra_lp2wake_irq)) {
		pr_err("Failed to register LP2 wakeup timer IRQ\n");
		BUG();
	}

	clockevents_register_device(&tegra_clockevent.event);
}

struct sys_timer tegra_timer = {
	.init	= tegra_timer_init,
};

