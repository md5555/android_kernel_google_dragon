/* linux/arch/arm/plat-s5p/include/plat/pm-s5p.h
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * S5P - Definitions for Power Management
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

/* helper functions to save/restore lists of registers. */

#ifdef CONFIG_PM
extern int s3c_irq_wake(unsigned int irqno, unsigned int state);
extern int s5p_irq_suspend(struct sys_device *dev, pm_message_t state);
extern int s5p_irq_resume(struct sys_device *dev);
#else
#define s3c_irq_wake NULL
#define s5p_irq_suspend NULL
#define s5p_irq_resume NULL
#endif
