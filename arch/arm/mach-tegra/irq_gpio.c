/*
 * arch/arm/mach-tegra/irq_gpio.c
 *
 * Second-level IRQ decoder for GPIOs
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

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/gpio.h>

#include <mach/nvrm_linux.h>
#include "nvrm_init.h"
#include "nvrm_module.h"
#include "nvrm_interrupt.h"
#include "nvrm_gpio.h"
#include "ap15/argpio.h"
#include "nvassert.h"
#include "nvrm_hardware_access.h"

#define NVRM_GPIO_INTERRUPT_TEST    0

// Size of a port register
#define NV_GPIO_PORT_REG_SIZE           (GPIO_CNF_1 - GPIO_CNF_0)   
// NUmber of GPIO ports per controller
#define NV_GPIO_PORTS_PER_CONTROLLER    (4)                         
// Number of GPIO pins per controller port
#define NV_GPIO_PINS_PER_PORT           (8)                         
#define NV_GPIO_PORT_REG_MASK           ((1 << NV_GPIO_PINS_PER_PORT) - 1)
#define NV_GPIO_MASK(bit, val)                \
    ((1 << ((bit) + NV_GPIO_PINS_PER_PORT)) | \
     (((val) << (bit)) & NV_GPIO_PORT_REG_MASK))

// Gpio register read/write macros
#define NV_GPIO_REGR( addr, Port, Reg ) \
    NV_READ32((NvU32)addr + ((Port) * NV_GPIO_PORT_REG_SIZE)+(GPIO_##Reg##_0));

#define NV_GPIO_REGW( addr, Port, Reg, data )                          \
    do                                                                 \
    {                                                                  \
        NV_WRITE32(((NvU32)(addr) + ((Port) * NV_GPIO_PORT_REG_SIZE) + \
                    (GPIO_##Reg##_0)), (data));                        \
    } while (0)

struct NvGpioInstance
{
    void __iomem *base;
    NvU32 phys;
    NvU32 size;
    u16 irqMain;
    u16 irqStart;
    u16 irqCount;
    u16 irqEnd;
};

#define MAX_GPIO_INSTANCES  10

static struct NvGpioInstance s_GpioInstances[MAX_GPIO_INSTANCES];
NvU32 s_GpioInstanceMax;

static void NvPrivGpioEnable(unsigned int irq, NvBool enable)
{
    NvU32 i;


    i = s_GpioInstanceMax;
    while (i--)
    {
        if ((irq >= s_GpioInstances[i].irqStart) 
            && (irq < s_GpioInstances[i].irqEnd))
        { 
            NvU32 relative_irq;
            NvU32 Port;
            NvU32 Bit;

        
            relative_irq = irq - s_GpioInstances[i].irqStart;

            // Get the controller number, port number and bit number.
            Port  = relative_irq / NV_GPIO_PINS_PER_PORT;
            Bit   = relative_irq - (Port * NV_GPIO_PINS_PER_PORT);

            // Enable/Disable the specified GPIO controller main interrupt.
            NV_GPIO_REGW(s_GpioInstances[i].base, Port, MSK_INT_ENB,
                NV_GPIO_MASK(Bit, enable));

            return;
        }
    }
    
    /* should not come here */
    NV_ASSERT(0);
    return;
}

static void NvPrivGpioMaskIrq(unsigned int irq)
{
    //printk("Disabling IRQ(%d)\n", irq);
    NvPrivGpioEnable(irq, NV_FALSE);
}

void NvPrivGpioUnMaskIrq(unsigned int irq)
{
    //printk("Enabling IRQ(%d)\n", irq);
    NvPrivGpioEnable(irq, NV_TRUE);
}

static void NvPrivGpioAckIrq(unsigned int irq)
{
    NvU32 i;

    i = s_GpioInstanceMax;
    while (i--)
    {
        if ((irq >= s_GpioInstances[i].irqStart) 
            && (irq < s_GpioInstances[i].irqEnd))
        { 
            NvU32 relative_irq;
            NvU32 Port;
            NvU32 Bit;
            NvU32 Mask;

            relative_irq = irq - s_GpioInstances[i].irqStart;

            // Get the controller number, port number and bit number.
            Port  = relative_irq / NV_GPIO_PINS_PER_PORT;
            Bit   = relative_irq - (Port * NV_GPIO_PINS_PER_PORT);
            Mask = 1 << Bit;

            NV_GPIO_REGW(s_GpioInstances[i].base, Port, INT_CLR, Mask);

            //printk("IRQ acked for %d\n", irq);
            return;
        }
    }
    
    /* should not come here */
    NV_ASSERT(0);
}

static struct irq_chip s_NvGpioIrqDispatch = {
    .name           = "GPIO",
    .ack            = NvPrivGpioAckIrq,
    .mask           = NvPrivGpioMaskIrq,
    .unmask         = NvPrivGpioUnMaskIrq,
};


static void gpio_irq_handler(unsigned int irqMain, struct irq_desc *desc)
{
	struct irq_chip *chip = get_irq_chip(irqMain);
    NvU32 i = 0;
    NvU32 Port;       // Controller port number
    NvU32 Bit;        // Port bit number
    NvU32 IntEnable;  // Port interrupt enable bits
    NvU32 IntStatus;  // Port interrupt status bits
    NvU32 Mask;       // Mask of interrupting bit within the port


    while (i < s_GpioInstanceMax && irqMain != s_GpioInstances[i].irqMain)
    {
        i++;
    }

    /* Something wrong in the setup? Why am i getting a callback for the 
     * non-GPIO interrupts? */
    NV_ASSERT (i != s_GpioInstanceMax);

    /* This does nothing on AP15, on AP20 this disables the interrupt
     * and issues a EOI - see gic_ack_irq in arch/arm/common/gic */
    chip->ack(irqMain);

    Port = NV_GPIO_PORTS_PER_CONTROLLER;
    while (Port--)
    {
        IntEnable = NV_GPIO_REGR(s_GpioInstances[i].base, Port, INT_ENB);
        IntStatus = NV_GPIO_REGR(s_GpioInstances[i].base, Port, INT_STA);

        // Is there a bit interrupting in this port?
        if ((IntStatus = (IntEnable & IntStatus)) != 0)
        {
            NvU32 irq;

            __asm__ __volatile__ (      \
                "clz %0, %1 \r\t"       \
                :"=r"(Bit)              \
                :"r"(IntStatus));
            Bit  = 31 - Bit;

            Mask = 1 << Bit;

            irq = s_GpioInstances[i].irqStart + 
                ((Port * NV_GPIO_PINS_PER_PORT) + Bit);

            /* Trigger the interrupt for the client requesting the callback on
             * this GPIO pin */
            generic_handle_irq(irq);
            goto out;
        }
    }

    /* Got a spurious interrupt? How is this possible? */
    NV_ASSERT(0);

out:
    /* Re-enable the interrupt on main interrupt controller */
    chip->unmask(irqMain);
    return;
}

void __init tegra_init_gpio(void)
{
    NvU32 i;
    NvU32 irq;
    NvU32 count;

    s_NvGpioIrqDispatch.set_type = NULL;
 
    s_GpioInstanceMax = NvRmModuleGetNumInstances(s_hRmGlobal, NvRmPrivModuleID_Gpio);
    BUG_ON(s_GpioInstanceMax > MAX_GPIO_INSTANCES);

    i = s_GpioInstanceMax;
    while (i--)
    {
        NvRmModuleGetBaseAddress(s_hRmGlobal,
            NVRM_MODULE_ID(NvRmPrivModuleID_Gpio, i), 
            &s_GpioInstances[i].phys, &s_GpioInstances[i].size);

        NV_ASSERT_SUCCESS(NvRmPhysicalMemMap(s_GpioInstances[i].phys, 
                s_GpioInstances[i].size , NVOS_MEM_READ_WRITE, 
                NvOsMemAttribute_Uncached,
                (void **)&s_GpioInstances[i].base));

        s_GpioInstances[i].irqMain = NvRmGetIrqForLogicalInterrupt(s_hRmGlobal, 
            NVRM_MODULE_ID(NvRmPrivModuleID_Gpio, i), 0xff);

        s_GpioInstances[i].irqStart = NvRmGetIrqForLogicalInterrupt(
            s_hRmGlobal, NVRM_MODULE_ID(NvRmPrivModuleID_Gpio, i), 0x0);

        count =  NV_GPIO_PINS_PER_PORT * NV_GPIO_PORTS_PER_CONTROLLER;
        s_GpioInstances[i].irqCount = count;
        s_GpioInstances[i].irqEnd =
            s_GpioInstances[i].irqStart + s_GpioInstances[i].irqCount;

        while (count--)
        {
            irq = NvRmGetIrqForLogicalInterrupt(s_hRmGlobal, 
                NVRM_MODULE_ID(NvRmPrivModuleID_Gpio,i), count);
            
             set_irq_chip(irq, &s_NvGpioIrqDispatch);
             /* FIXME, Ideally, we use handle_edge_irq for edge interrupts. But,
              * handle_level_irq should also work for most edge interrupts which
              * are handled one at a time. If the edge interrupt is because of
              * very short pulses, then this logic won't work.
              */
             set_irq_handler(irq, handle_level_irq);
             set_irq_flags(irq, IRQF_VALID);
        }

        set_irq_chained_handler(s_GpioInstances[i].irqMain, gpio_irq_handler);
    }

    return;
}


#if defined(CONFIG_GENERIC_GPIO)

int gpio_set_value(unsigned gpio, int value)
{
    return 0;
}

int gpio_get_value(unsigned gpio)
{
    return 0;
}

int gpio_direction_input(unsigned gpio)
{
    return 0;
}

int gpio_direction_output(unsigned gpio, int value)
{
    return 0;
}

int gpio_request(unsigned gpio, const char *tag)
{
    return 0;
}

void gpio_free(unsigned gpio)
{
}

#endif
