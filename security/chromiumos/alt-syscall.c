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

static struct syscall_whitelist whitelists[] = {
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
