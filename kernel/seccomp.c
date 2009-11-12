/*
 * linux/kernel/seccomp.c
 *
 * Copyright 2004-2005  Andrea Arcangeli <andrea@cpushare.com>
 *
 * This defines a simple but solid secure-computing mode.
 */

#include <asm/processor.h>
#include <linux/mm.h>
#include <linux/seccomp.h>
#include <linux/sched.h>
#include <linux/compat.h>

/* #define SECCOMP_DEBUG 1 */
#define NR_SECCOMP_MODES 2

/*
 * Secure computing mode 1 allows only read/write/exit/sigreturn.
 * To be fully secure this must be combined with rlimit
 * to limit the stack allocations too.
 */
static int mode1_syscalls[] = {
	__NR_seccomp_read, __NR_seccomp_write, __NR_seccomp_exit, __NR_seccomp_sigreturn,
	0, /* null terminated */
};

#ifdef CONFIG_COMPAT
static int mode1_syscalls_32[] = {
	__NR_seccomp_read_32, __NR_seccomp_write_32, __NR_seccomp_exit_32, __NR_seccomp_sigreturn_32,
	0, /* null terminated */
};
#endif

static bool readonly_ip() {
	/* find_vma uses a cache, but we may want to speed this up with a hash
	 * of ips to vms if needed. */
	unsigned long ip = KSTK_EIP(current);
	struct vm_area_struct *map = find_vma(current->mm, ip);
	if (!map) {
		printk(KERN_ERR "no vma found for address %lx\n", ip);
		BUG();
	}

	/* Writable maps shouldn't call system calls. */
	/* TODO: could require VM_SHARED as well. */
	if (map->vm_flags & VM_WRITE) {
		return false;
	}
	return true;
}

void __secure_computing(int this_syscall)
{
	int mode = current->seccomp.mode;
	int * syscall;

	switch (mode) {
	case 1:
		syscall = mode1_syscalls;
#ifdef CONFIG_COMPAT
		if (is_compat_task())
			syscall = mode1_syscalls_32;
#endif
		do {
			if (*syscall == this_syscall)
				return;
		} while (*++syscall);
		break;
	case 2:  /* system calls from w^x pages only */
		/* TODO: we could always allow seccomp(1) system calls
		 *       since they will only be able to modify open fds */
#ifdef SECCOMP_DEBUG
		printk(KERN_INFO "%s[%d]: checking syscall #%d\n",
		       current->comm, task_pid_nr(current), this_syscall);
#endif
		if (readonly_ip())
			return;
		printk(KERN_INFO
			"%s[%d]: system call %d from writable ip:%lx\n",
			current->comm, task_pid_nr(current), this_syscall,
			KSTK_EIP(current));
		break;
	default:
		BUG();
	}

#ifdef SECCOMP_DEBUG
	dump_stack();
#endif
	do_exit(SIGKILL);
}

long prctl_get_seccomp(void)
{
	return current->seccomp.mode;
}

long prctl_set_seccomp(unsigned long seccomp_mode)
{
	long ret;

	/* can only move to a more secure mode */
	ret = -EPERM;
	if (unlikely(current->seccomp.mode)) {
		/* Upgrade from 2 to 1 is okay */
		if (seccomp_mode != 1)
			goto out;
	}

	ret = -EINVAL;
	if (seccomp_mode && seccomp_mode <= NR_SECCOMP_MODES) {
		current->seccomp.mode = seccomp_mode;
		set_thread_flag(TIF_SECCOMP);
#ifdef TIF_NOTSC
		if (seccomp_mode == 1)
			disable_TSC();
#endif
		ret = 0;
	}

 out:
	return ret;
}
