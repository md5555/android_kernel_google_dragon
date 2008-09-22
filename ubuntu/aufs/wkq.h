/*
 * Copyright (C) 2005-2008 Junjiro Okajima
 *
 * This program, aufs is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

/*
 * workqueue for asynchronous/super-io/delegated operations
 *
 * $Id: wkq.h,v 1.7 2008/09/15 03:16:36 sfjro Exp $
 */

#ifndef __AUFS_WKQ_H__
#define __AUFS_WKQ_H__

#ifdef __KERNEL__

#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/workqueue.h>
#include <linux/aufs_type.h>

/* ---------------------------------------------------------------------- */

/* internal workqueue named AUFS_WKQ_NAME */
struct au_wkq {
	struct workqueue_struct	*q;

	/* balancing */
	atomic_t		busy;

	/* accounting */
#ifdef CONFIG_AUFS_STAT
	unsigned int		max_busy;
#endif
};

/*
 * in the next operation, wait for the 'nowait' tasks in system-wide workqueue
 */
struct au_nowait_tasks {
	atomic_t		nw_len;
	wait_queue_head_t	nw_wq;
};

/* ---------------------------------------------------------------------- */

extern struct au_wkq *au_wkq;
typedef void (*au_wkq_func_t)(void *args);

/* wkq flags */
#define AuWkq_WAIT	1
#define AuWkq_DLGT	(1 << 1)
#define au_ftest_wkq(flags, name)	((flags) & AuWkq_##name)
#define au_fset_wkq(flags, name)	{ (flags) |= AuWkq_##name; }
#define au_fclr_wkq(flags, name)	{ (flags) &= ~AuWkq_##name; }
#ifndef CONFIG_AUFS_DLGT
#undef AuWkq_DLGT
#define AuWkq_DLGT	0
#endif

int au_wkq_run(au_wkq_func_t func, void *args, struct super_block *sb,
	       unsigned int flags);
int au_wkq_nowait(au_wkq_func_t func, void *args, struct super_block *sb,
		  int dlgt);
int __init au_wkq_init(void);
void au_wkq_fin(void);

/* ---------------------------------------------------------------------- */

static inline int au_test_wkq(struct task_struct *tsk)
{
	return (!tsk->mm && !strcmp(tsk->comm, AUFS_WKQ_NAME));
#if 0 /* reserved for future use, per-cpu workqueue */
	return (!tsk->mm
		&& !memcmp(tsk->comm, AUFS_WKQ_NAME "/",
			   sizeof(AUFS_WKQ_NAME)));
#endif
}

static inline int au_wkq_wait(au_wkq_func_t func, void *args, int dlgt)
{
	unsigned int flags = AuWkq_WAIT;
	if (unlikely(dlgt))
		au_fset_wkq(flags, DLGT);
	return au_wkq_run(func, args, /*sb*/NULL, flags);
}

static inline void au_wkq_max_busy_init(struct au_wkq *wkq)
{
#ifdef CONFIG_AUFS_STAT
	wkq->max_busy = 0;
#endif
}

/* todo: memory barrier? */
static inline void au_nwt_init(struct au_nowait_tasks *nwt)
{
	atomic_set(&nwt->nw_len, 0);
	smp_mb(); /* atomic_set */
	init_waitqueue_head(&nwt->nw_wq);
}

/* todo: make it void */
static inline int au_nwt_done(struct au_nowait_tasks *nwt)
{
	int ret;

	AuTraceEnter();

	ret = atomic_dec_return(&nwt->nw_len);
	if (!ret)
		wake_up_all(&nwt->nw_wq);
	return ret;
}

static inline int au_nwt_flush(struct au_nowait_tasks *nwt)
{
	wait_event(nwt->nw_wq, !atomic_read(&nwt->nw_len));
	return 0;
}

#endif /* __KERNEL__ */
#endif /* __AUFS_WKQ_H__ */
