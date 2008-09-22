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
 * lower (branch filesystem) inode and setting inotify
 *
 * $Id: hinode.h,v 1.9 2008/08/25 01:49:59 sfjro Exp $
 */

#ifndef __AUFS_HINODE_H__
#define __AUFS_HINODE_H__

#ifdef __KERNEL__

#include <linux/fs.h>
#include <linux/inotify.h>
#include <linux/aufs_type.h>
#include "super.h"
#include "vfsub.h"

/* ---------------------------------------------------------------------- */

struct au_hinotify {
#ifdef CONFIG_AUFS_HINOTIFY
	spinlock_t		hin_ignore_lock;
	struct list_head	hin_ignore_list;

	struct inotify_watch	hin_watch;
	struct inode		*hin_aufs_inode;	/* no get/put */
#endif
};

struct au_hinode {
	struct inode		*hi_inode;
	aufs_bindex_t		hi_id;
#ifdef CONFIG_AUFS_HINOTIFY
	struct au_hinotify	*hi_notify;
#endif

	/* reference to the copied-up whiteout with get/put */
	struct dentry		*hi_whdentry;
};

struct au_hin_ignore {
#ifdef CONFIG_AUFS_HINOTIFY
	struct list_head	ign_list;

	pid_t			ign_pid;
	__u32			ign_events, ign_handled;
	struct au_hinode	*ign_hinode;
#endif
};

/* ---------------------------------------------------------------------- */

#ifdef CONFIG_AUFS_HINOTIFY
/* inotify events */
static const __u32 AuInMask = (IN_MOVE | IN_DELETE | IN_CREATE
			       /* | IN_ACCESS */
			       | IN_MODIFY | IN_ATTRIB
			       /* | IN_DELETE_SELF | IN_MOVE_SELF */
	);

static inline
void au_hin_init(struct au_hinode *hinode, struct au_hinotify *val)
{
	hinode->hi_notify = val;
}

/* hinotify.c */
int au_hin_alloc(struct au_hinode *hinode, struct inode *inode,
		 struct inode *h_inode);
void au_hin_free(struct au_hinode *hinode);
void au_hin_ctl(struct au_hinode *hinode, const __u32 mask);
void au_reset_hinotify(struct inode *inode, unsigned int flags);

int au_hin_verify_gen(struct dentry *dentry);

int __init au_inotify_init(void);
void au_inotify_fin(void);

static inline void au_hin_suspend(struct au_hinode *hinode)
{
	au_hin_ctl(hinode, 0);
}

static inline void au_hin_resume(struct au_hinode *hinode)
{
	au_hin_ctl(hinode, AuInMask);
}

#else

static inline
void au_hin_init(struct au_hinode *hinode, struct au_hinotify *val)
{
	/* empty */
}

static inline
int au_hin_alloc(struct au_hinode *hinode, struct inode *inode,
		 struct inode *h_inode)
{
	return -EOPNOTSUPP;
}

static inline void au_hin_free(struct au_hinode *hinode)
{
	/* nothing */
}

static inline void au_reset_hinotify(struct inode *inode, unsigned int flags)
{
	/* nothing */
}

static inline int au_hin_verify_gen(struct dentry *dentry)
{
	return 0;
}

static inline int au_inotify_init(void)
{
	return 0;
}

#define au_inotify_fin()	do {} while (0)

static inline void au_hin_suspend(struct au_hinode *hinode)
{
	/* empty */
}

static inline void au_hin_resume(struct au_hinode *hinode)
{
	/* empty */
}
#endif /* CONFIG_AUFS_HINOTIFY */

#if defined(CONFIG_AUFS_HINOTIFY) && defined(CONFIG_AUFS_DEBUG)
static inline void au_hin_list_del(struct list_head *e)
{
	list_del_init(e);
}

void au_dbg_hin_list(struct vfsub_args *vargs);
#else
static inline void au_hin_list_del(struct list_head *e)
{
	list_del(e);
}

static inline void au_dbg_hin_list(struct vfsub_args *vargs)
{
	/* empty */
}
#endif /* CONFIG_AUFS_DEBUG */

/* ---------------------------------------------------------------------- */

#endif /* __KERNEL__ */
#endif /* __AUFS_HINODE_H__ */
