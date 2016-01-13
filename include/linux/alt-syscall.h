#ifndef _ALT_SYSCALL_H
#define _ALT_SYSCALL_H

#include <linux/errno.h>
#include <linux/list.h>
#include <linux/sched.h>

#define ALT_SYS_CALL_NAME_MAX	32

typedef void (*sys_call_ptr_t)(void);

struct alt_sys_call_table {
	char name[ALT_SYS_CALL_NAME_MAX + 1];
	sys_call_ptr_t *table;
	int size;
#ifdef CONFIG_COMPAT
	sys_call_ptr_t *compat_table;
	int compat_size;
#endif
	struct list_head node;
};

/*
 * arch_dup_sys_call_table should return the default syscall table, not
 * the current syscall table, since we want to explicitly not allow
 * syscall table composition. A selected syscall table should be treated
 * as a single execution personality.
 */

#ifdef CONFIG_ALT_SYSCALL
#include <asm/syscall.h>

int arch_dup_sys_call_table(struct alt_sys_call_table *table);
int arch_set_sys_call_table(struct alt_sys_call_table *table);

int register_alt_sys_call_table(struct alt_sys_call_table *table);
int set_alt_sys_call_table(char __user *name);
#else
static inline int arch_dup_sys_call_table(struct alt_sys_call_table *table)
{
	return -ENOSYS;
}
static inline int arch_set_sys_call_table(struct alt_sys_call_table *table)
{
	return -ENOSYS;
}
static inline int register_alt_sys_call_table(struct alt_sys_call_table *table)
{
	return -ENOSYS;
}
static inline int set_alt_sys_call_table(char __user *name)
{
	return -ENOSYS;
}
#endif

#endif /* _ALT_SYSCALL_H */
