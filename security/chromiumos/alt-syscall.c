/*
 * Chromium OS alt-syscall tables
 *
 * Copyright (C) 2015 Google, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/alt-syscall.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/prctl.h>
#include <linux/slab.h>
#include <linux/syscalls.h>

#include <asm/unistd.h>

struct syscall_whitelist_entry {
	unsigned int nr;
	sys_call_ptr_t alt;
};

struct syscall_whitelist {
	const char *name;
	const struct syscall_whitelist_entry *whitelist;
	unsigned int nr_whitelist;
#ifdef CONFIG_COMPAT
	const struct syscall_whitelist_entry *compat_whitelist;
	unsigned int nr_compat_whitelist;
#endif
};

/*
 * If an alt_syscall table allows prctl(), override it to prevent a process
 * from changing its syscall table.
 */
static asmlinkage long alt_sys_prctl(int option, unsigned long arg2,
				     unsigned long arg3, unsigned long arg4,
				     unsigned long arg5)
{
	if (option == PR_ALT_SYSCALL &&
	    arg2 == PR_ALT_SYSCALL_SET_SYSCALL_TABLE)
		return -EPERM;

	return sys_prctl(option, arg2, arg3, arg4, arg5);
}

#ifdef CONFIG_COMPAT
#define SYSCALL_WHITELIST_COMPAT(x)					\
	.compat_whitelist = x ## _compat_whitelist,			\
	.nr_compat_whitelist = ARRAY_SIZE(x ## _compat_whitelist),
#else
#define SYSCALL_WHITELIST_COMPAT(x)
#endif

#define SYSCALL_WHITELIST(x)						\
	{								\
		.name = #x,						\
		.whitelist = x ## _whitelist,				\
		.nr_whitelist = ARRAY_SIZE(x ## _whitelist),		\
		SYSCALL_WHITELIST_COMPAT(x)				\
	}

#ifdef CONFIG_COMPAT
#ifdef CONFIG_X86
#define __NR_compat_exit	__NR_ia32_exit
#define __NR_compat_open	__NR_ia32_open
#define __NR_compat_close	__NR_ia32_close
#define __NR_compat_read	__NR_ia32_read
#define __NR_compat_write	__NR_ia32_write
#define __NR_compat_prctl	__NR_ia32_prctl
#endif
#endif

static struct syscall_whitelist_entry read_write_test_whitelist[] = {
	{ __NR_exit, },
	{ __NR_open, },
	{ __NR_close, },
	{ __NR_read, },
	{ __NR_write, },
	{ __NR_prctl, (sys_call_ptr_t)alt_sys_prctl },
};

#ifdef CONFIG_COMPAT
static struct syscall_whitelist_entry read_write_test_compat_whitelist[] = {
	{ __NR_compat_exit, },
	{ __NR_compat_open, },
	{ __NR_compat_close, },
	{ __NR_compat_read, },
	{ __NR_compat_write, },
	{ __NR_compat_prctl, (sys_call_ptr_t)alt_sys_prctl },
};
#endif

static struct syscall_whitelist whitelists[] = {
	SYSCALL_WHITELIST(read_write_test),
};

static int alt_syscall_apply_whitelist(const struct syscall_whitelist *wl,
				       struct alt_sys_call_table *t)
{
	unsigned int i;
	DECLARE_BITMAP(whitelist, t->size);

	bitmap_zero(whitelist, t->size);
	for (i = 0; i < wl->nr_whitelist; i++) {
		unsigned int nr = wl->whitelist[i].nr;

		if (nr >= t->size)
			return -EINVAL;
		bitmap_set(whitelist, nr, 1);
		if (wl->whitelist[i].alt)
			t->table[nr] = wl->whitelist[i].alt;
	}

	for (i = 0; i < t->size; i++) {
		if (!test_bit(i, whitelist))
			t->table[i] = (sys_call_ptr_t)sys_ni_syscall;
	}

	return 0;
}

#ifdef CONFIG_COMPAT
static int
alt_syscall_apply_compat_whitelist(const struct syscall_whitelist *wl,
				   struct alt_sys_call_table *t)
{
	unsigned int i;
	DECLARE_BITMAP(whitelist, t->compat_size);

	bitmap_zero(whitelist, t->compat_size);
	for (i = 0; i < wl->nr_compat_whitelist; i++) {
		unsigned int nr = wl->compat_whitelist[i].nr;

		if (nr >= t->compat_size)
			return -EINVAL;
		bitmap_set(whitelist, nr, 1);
		if (wl->compat_whitelist[i].alt)
			t->compat_table[nr] = wl->compat_whitelist[i].alt;
	}

	for (i = 0; i < t->compat_size; i++) {
		if (!test_bit(i, whitelist))
			t->compat_table[i] = (sys_call_ptr_t)sys_ni_syscall;
	}

	return 0;
}
#else
static inline int
alt_syscall_apply_compat_whitelist(const struct syscall_whitelist *wl,
				   struct alt_sys_call_table *t)
{
	return 0;
}
#endif

static int alt_syscall_init_one(const struct syscall_whitelist *wl)
{
	struct alt_sys_call_table *t;
	int err;

	t = kzalloc(sizeof(*t), GFP_KERNEL);
	if (!t)
		return -ENOMEM;
	strncpy(t->name, wl->name, sizeof(t->name));

	err = arch_dup_sys_call_table(t);
	if (err)
		return err;

	err = alt_syscall_apply_whitelist(wl, t);
	if (err)
		return err;
	err = alt_syscall_apply_compat_whitelist(wl, t);
	if (err)
		return err;

	return register_alt_sys_call_table(t);
}

/*
 * Register an alternate syscall table for each whitelist.  Note that the
 * lack of a module_exit() is intentional - once a syscall table is registered
 * it cannot be unregistered.
 *
 * TODO(abrestic) Support unregistering syscall tables?
 */
static int chromiumos_alt_syscall_init(void)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(whitelists); i++) {
		int err;

		err = alt_syscall_init_one(&whitelists[i]);
		if (err)
			pr_warn("Failed to register syscall table %s: %d\n",
				whitelists[i].name, err);
	}

	return 0;
}
module_init(chromiumos_alt_syscall_init);
