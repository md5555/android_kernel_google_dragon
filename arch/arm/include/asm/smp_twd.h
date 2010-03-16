#ifndef __ASMARM_SMP_TWD_H
#define __ASMARM_SMP_TWD_H

struct clock_event_device;

extern void __iomem *twd_base;

#ifdef CONFIG_USE_ARM_TWD_PRESCALER
extern unsigned long timer_prescaler;
void twd_set_prescaler(void* unused);
#else
static inline void twd_set_prescaler(void* unused)
{
}
#endif

void twd_timer_stop(void);
int twd_timer_ack(void);
void twd_timer_setup(struct clock_event_device *);

#endif
