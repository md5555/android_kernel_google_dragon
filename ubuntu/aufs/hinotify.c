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
 * internal/hidden inotify handler
 *
 * $Id: hinotify.c,v 1.16 2008/09/08 02:39:54 sfjro Exp $
 */

#include "aufs.h"

/*
#ifdef DbgInotify
#define AuDbgHin(args...)	AuDbg(##args)
#else
#define AuDbgHin(args...)	do {} while ()
#endif
*/

static struct inotify_handle *in_handle;

AuCacheFuncs(hinotify, AuCache_HINOTIFY);

int au_hin_alloc(struct au_hinode *hinode, struct inode *inode,
		 struct inode *h_inode)
{
	int err;
	struct au_hinotify *hin;
	s32 wd;

	LKTRTrace("i%lu, hi%lu\n", inode->i_ino, h_inode->i_ino);

	err = -ENOMEM;
	hin = au_cache_alloc_hinotify();
	if (hin) {
		AuDebugOn(hinode->hi_notify);
		hinode->hi_notify = hin;
		spin_lock_init(&hin->hin_ignore_lock);
		INIT_LIST_HEAD(&hin->hin_ignore_list);
		hin->hin_aufs_inode = inode;

		inotify_init_watch(&hin->hin_watch);
		wd = inotify_add_watch(in_handle, &hin->hin_watch, h_inode,
				       AuInMask);
		if (wd >= 0)
			return 0; /* success */

		err = wd;
		put_inotify_watch(&hin->hin_watch);
		au_cache_free_hinotify(hin);
		hinode->hi_notify = NULL;
	}

	AuTraceErr(err);
	return err;
}

void au_hin_free(struct au_hinode *hinode)
{
	int err;
	struct au_hinotify *hin;

	AuTraceEnter();

	hin = hinode->hi_notify;
	if (unlikely(hin)) {
		err = 0;
		if (atomic_read(&hin->hin_watch.count))
			err = inotify_rm_watch(in_handle, &hin->hin_watch);
		if (unlikely(err))
			/* it means the watch is already removed */
			LKTRTrace("failed inotify_rm_watch() %d\n", err);
		au_cache_free_hinotify(hin);
		hinode->hi_notify = NULL;
	}
}

/* ---------------------------------------------------------------------- */

void au_hin_ctl(struct au_hinode *hinode, const __u32 mask)
{
	struct inode *h_inode;
	struct inotify_watch *watch;

	h_inode = hinode->hi_inode;
	LKTRTrace("hi%lu, sb %p, 0x%x\n", h_inode->i_ino, h_inode->i_sb, mask);
	IMustLock(h_inode);
	if (!hinode->hi_notify)
		return;

	watch = &hinode->hi_notify->hin_watch;
#if 0 /* reserved for future use */
	{
		u32 wd;
		wd = inotify_find_update_watch(in_handle, h_inode, mask);
		AuTraceErr(wd);
		/* ignore an err; */
	}
#else
	/* struct inotify_handle is hidden */
	mutex_lock(&h_inode->inotify_mutex);
	/* mutex_lock(&watch->ih->mutex); */
	watch->mask = mask;
	/* mutex_unlock(&watch->ih->mutex); */
	mutex_unlock(&h_inode->inotify_mutex);
#endif
	LKTRTrace("watch %p, mask %u\n", watch, watch->mask);
}

void au_reset_hinotify(struct inode *inode, unsigned int flags)
{
	aufs_bindex_t bindex, bend;
	struct inode *hi;
	struct dentry *iwhdentry;

	LKTRTrace("i%lu, 0x%x\n", inode->i_ino, flags);

	bend = au_ibend(inode);
	for (bindex = au_ibstart(inode); bindex <= bend; bindex++) {
		hi = au_h_iptr(inode, bindex);
		if (hi) {
			/* mutex_lock_nested(&hi->i_mutex, AuLsc_I_CHILD); */
			iwhdentry = au_hi_wh(inode, bindex);
			if (unlikely(iwhdentry))
				dget(iwhdentry);
			au_igrab(hi);
			au_set_h_iptr(inode, bindex, NULL, 0);
			au_set_h_iptr(inode, bindex, au_igrab(hi),
				      flags & ~AuHi_XINO);
			iput(hi);
			dput(iwhdentry);
			/* mutex_unlock(&hi->i_mutex); */
		}
	}
}

/* ---------------------------------------------------------------------- */

void au_unpin_gp(struct au_pin *args)
{
	struct au_pin1 *gp;

	gp = au_pin_gp(args);
	AuDebugOn(!gp);
	if (gp->dentry)
		LKTRTrace("%.*s\n", AuDLNPair(gp->dentry));
	else
		AuTraceEnter();

	au_do_unpin(gp, NULL);
}

int au_hin_verify_gen(struct dentry *dentry)
{
	struct super_block *sb = dentry->d_sb;
	au_gen_t sigen;
	struct inode *inode;

	if (!au_opt_test(au_mntflags(sb), UDBA_INOTIFY))
		return 0;

	sigen = au_sigen(dentry->d_sb);
	inode = dentry->d_inode;
	return (au_digen(dentry) != sigen
		|| (inode && au_iigen(inode) != sigen));
}

/* ---------------------------------------------------------------------- */

/* cf. fsnotify_change() */
__u32 vfsub_events_notify_change(struct iattr *ia)
{
	__u32 events;
	const unsigned int amtime = (ATTR_ATIME | ATTR_MTIME);

	events = 0;
	if ((ia->ia_valid & (ATTR_UID | ATTR_GID | ATTR_MODE))
	    || (ia->ia_valid & amtime) == amtime)
		events |= IN_ATTRIB;
	if ((ia->ia_valid & ATTR_SIZE)
	    || (ia->ia_valid & amtime) == ATTR_MTIME)
		events |= IN_MODIFY;
	return events;
}

void vfsub_ign_hinode(struct vfsub_args *vargs, __u32 events,
		      struct au_hinode *hinode)
{
	struct au_hinotify *hin;
	struct super_block *sb;
	struct au_hin_ignore *ign;

	if (!hinode)
		return;

	hin = hinode->hi_notify;
	if (!hin || !hin->hin_watch.mask)
		return;

	sb = hin->hin_aufs_inode->i_sb;
	AuDebugOn(!au_opt_test(au_mntflags(sb), UDBA_INOTIFY));

	ign = vargs->ignore + vargs->nignore++;
	ign->ign_events = events;
	ign->ign_handled = 0;
	ign->ign_hinode = hinode;

	{
		struct inode *h_inode;
		h_inode = hinode->hi_inode;
		if (!mutex_is_locked(&h_inode->i_mutex))
			au_dbg_blocked();
		IMustLock(h_inode);
	}
}

static void au_hin_ignore(struct au_hin_ignore *ign)
{
	struct au_hinode *hinode;
	__u32 events;
	struct au_hinotify *hin;
	struct inode *h_inode;

	hinode = ign->ign_hinode;
	events = ign->ign_events;
	LKTRTrace("0x%x\n", events);
	AuDebugOn(!hinode || !events);

	hin = hinode->hi_notify;
	h_inode = hinode->hi_inode;
	if (h_inode && hin) {
		LKTRTrace("hi%lu\n", h_inode->i_ino);
#ifdef DbgInotify
		AuDbg("hi%lu, 0x%x\n", h_inode->i_ino, events);
#endif

		spin_lock(&hin->hin_ignore_lock);
		list_add(&ign->ign_list, &hin->hin_ignore_list);
		spin_unlock(&hin->hin_ignore_lock);
		/* AuDbg("list_add %p, 0x%x\n", ign, events); */
	}
#if 1 /* todo: test dlgt */
	else
		/*
		 * it may happen by this scenario.
		 * - a file and its parent dir exist on two branches
		 * - a file on the upper branch is opened
		 * - the parent dir and the file are removed by udba
		 * - the parent is re-accessed, and new dentry/inode in
		 *   aufs is generated for it, based upon the one on the lower
		 *   branch
		 * - the opened file is re-accessed, re-validated, and it may be
		 *   re-connected to the new parent dentry
		 * it means the file in aufs cannot get the actual removed
		 * parent dir on the branch.
		 */
		INIT_LIST_HEAD(&ign->ign_list);
#endif
}

static void au_hin_unignore(struct au_hin_ignore *ign)
{
	struct au_hinode *hinode;
	__u32 events;
	struct au_hinotify *hin;
	struct inode *h_inode;

	hinode = ign->ign_hinode;
	events = ign->ign_events;
	LKTRTrace("0x%x\n", events);
	/* AuDbg("0x%x\n", events); */
	AuDebugOn(!hinode || !events);

	hin = hinode->hi_notify;
	h_inode = hinode->hi_inode;
	if (unlikely(!h_inode || !hin))
		return;
	LKTRTrace("hi%lu\n", h_inode->i_ino);
#ifdef DbgInotify
	AuDbg("hi%lu, 0x%x\n", h_inode->i_ino, events);
#endif

	spin_lock(&hin->hin_ignore_lock);
	au_hin_list_del(&ign->ign_list);
	spin_unlock(&hin->hin_ignore_lock);
	/* AuDbg("list_del %p, 0x%x\n", ign, events); */
}

static int au_hin_test_ignore(u32 mask, struct au_hinotify *hin)
{
	int do_ignore;
	struct au_hin_ignore *ign, *tmp;
	u32 events;

	do_ignore = 0;
	spin_lock(&hin->hin_ignore_lock);
	list_for_each_entry_safe(ign, tmp, &hin->hin_ignore_list, ign_list) {
		/* AuDbg("ign %p\n", ign); */
		if (ign->ign_pid == current->pid) {
			events = (mask & ign->ign_events);
			if (events) {
				do_ignore = 1;
				ign->ign_handled |= events;
				if (ign->ign_events == ign->ign_handled) {
					list_del_init(&ign->ign_list);
					/*
					AuDbg("list_del %p, 0x%x\n",
					      ign, events);
					*/
				}
				break;
			}
		}
	}
	spin_unlock(&hin->hin_ignore_lock);

	return do_ignore;
}

void vfsub_ignore(struct vfsub_args *vargs)
{
	int n;
	struct au_hin_ignore *ign;
	struct super_block *sb;
	struct au_hinode *hinode;
	struct inode *h_inode;

	n = vargs->nignore;
	if (!n)
		return;

	ign = vargs->ignore;
	hinode = ign->ign_hinode;
	sb = hinode->hi_notify->hin_aufs_inode->i_sb;
	h_inode = hinode->hi_inode;
	if (unlikely(au_opt_test(au_mntflags(sb), UDBA_INOTIFY))) {
		if (!mutex_is_locked(&h_inode->i_mutex))
			au_dbg_blocked();
		IMustLock(h_inode);
	}
	while (n-- > 0) {
		ign->ign_pid = current->pid;
		au_hin_ignore(ign++);
	}
}

void vfsub_unignore(struct vfsub_args *vargs)
{
	int n;
	struct au_hin_ignore *ign;

	n = vargs->nignore;
	if (!n)
		return;

	ign = vargs->ignore;
	while (n-- > 0)
		au_hin_unignore(ign++);
}

#ifdef CONFIG_AUFS_DEBUG
void au_dbg_hin_list(struct vfsub_args *vargs)
{
	int n;
	struct au_hin_ignore *ign;

	n = vargs->nignore;
	if (!n)
		return;

	ign = vargs->ignore;
	while (n-- > 0) {
		/* AuDebugOn(!list_empty(&ign++->ign_list)); */
		if (list_empty(&ign++->ign_list))
			continue;
		ign--;
		AuDbg("%d: pid %d, 0x%x\n",
		      n + 1, ign->ign_pid, ign->ign_events);
		ign++;
		au_dbg_blocked();
	}
}
#endif

/* ---------------------------------------------------------------------- */

static char *in_name(u32 mask)
{
#ifdef CONFIG_AUFS_DEBUG
#define test_ret(flag)	if (mask & flag) return #flag;
	test_ret(IN_ACCESS);
	test_ret(IN_MODIFY);
	test_ret(IN_ATTRIB);
	test_ret(IN_CLOSE_WRITE);
	test_ret(IN_CLOSE_NOWRITE);
	test_ret(IN_OPEN);
	test_ret(IN_MOVED_FROM);
	test_ret(IN_MOVED_TO);
	test_ret(IN_CREATE);
	test_ret(IN_DELETE);
	test_ret(IN_DELETE_SELF);
	test_ret(IN_MOVE_SELF);
	test_ret(IN_UNMOUNT);
	test_ret(IN_Q_OVERFLOW);
	test_ret(IN_IGNORED);
	return "";
#undef test_ret
#else
	return "??";
#endif
}

/* ---------------------------------------------------------------------- */

static struct dentry *lookup_wlock_by_name(char *name, unsigned int nlen,
					   struct inode *dir)
{
	struct dentry *dentry, *d, *parent;
	struct qstr *dname;

	LKTRTrace("%.*s, dir%lu\n", nlen, name, dir->i_ino);

	parent = d_find_alias(dir);
	if (!parent)
		return NULL;

	dentry = NULL;
	spin_lock(&dcache_lock);
	list_for_each_entry(d, &parent->d_subdirs, d_u.d_child) {
		LKTRTrace("%.*s\n", AuDLNPair(d));
		dname = &d->d_name;
		if (dname->len != nlen || memcmp(dname->name, name, nlen))
			continue;
		if (!atomic_read(&d->d_count) || !d->d_fsdata) {
			spin_lock(&d->d_lock);
			__d_drop(d);
			spin_unlock(&d->d_lock);
			continue;
		}

		dentry = dget(d);
		break;
	}
	spin_unlock(&dcache_lock);
	dput(parent);

	if (dentry) {
#if 0
	lktr_set_pid(current->pid, LktrArrayPid);
	AuDbgDentry(dentry);
	lktr_clear_pid(current->pid, LktrArrayPid);
#endif
		di_write_lock_child(dentry);
	}
	return dentry;
}

static struct inode *lookup_wlock_by_ino(struct super_block *sb,
					 aufs_bindex_t bindex, ino_t h_ino)
{
	struct inode *inode;
	struct au_xino_entry xinoe;
	int err;

	LKTRTrace("b%d, hi%lu\n", bindex, (unsigned long)h_ino);
	AuDebugOn(!au_opt_test_xino(au_mntflags(sb)));

	inode = NULL;
	err = au_xino_read(sb, bindex, h_ino, &xinoe);
	if (!err && xinoe.ino)
		inode = ilookup(sb, xinoe.ino);
	if (!inode)
		goto out;
	if (unlikely(inode->i_ino == AUFS_ROOT_INO)) {
		AuWarn("wrong root branch\n");
		iput(inode);
		inode = NULL;
		goto out;
	}

	ii_write_lock_child(inode);

 out:
	return inode;
}

static int hin_xino(struct inode *inode, struct inode *h_inode)
{
	int err;
	aufs_bindex_t bindex, bend, bfound, bstart;
	struct inode *h_i;

	LKTRTrace("i%lu, hi%lu\n", inode->i_ino, h_inode->i_ino);

	err = 0;
	if (unlikely(inode->i_ino == AUFS_ROOT_INO)) {
		AuWarn("branch root dir was changed\n");
		goto out;
	}

	bfound = -1;
	bend = au_ibend(inode);
	bstart = au_ibstart(inode);
#if 0 /* reserved for future use */
	if (bindex == bend) {
		/* keep this ino in rename case */
		goto out;
	}
#endif
	for (bindex = bstart; bindex <= bend; bindex++) {
		if (au_h_iptr(inode, bindex) == h_inode) {
			bfound = bindex;
			break;
		}
	}
	if (bfound < 0)
		goto out;

	for (bindex = bstart; bindex <= bend; bindex++) {
		h_i = au_h_iptr(inode, bindex);
		if (h_i)
			err = au_xino_write0(inode->i_sb, bindex, h_i->i_ino,
					     0);
		/* ignore this error */
		/* bad action? */
	}

	/* children inode number will be broken */

 out:
	AuTraceErr(err);
	return err;
}

static int hin_gen_tree(struct dentry *dentry)
{
	int err, i, j, ndentry;
	struct au_dcsub_pages dpages;
	struct au_dpage *dpage;
	struct dentry **dentries;

	LKTRTrace("%.*s\n", AuDLNPair(dentry));

	err = au_dpages_init(&dpages, GFP_NOFS);
	if (unlikely(err))
		goto out;
	err = au_dcsub_pages(&dpages, dentry, NULL, NULL);
	if (unlikely(err))
		goto out_dpages;

	for (i = 0; i < dpages.ndpage; i++) {
		dpage = dpages.dpages + i;
		dentries = dpage->dentries;
		ndentry = dpage->ndentry;
		for (j = 0; j < ndentry; j++) {
			struct dentry *d;
			d = dentries[j];
			LKTRTrace("%.*s\n", AuDLNPair(d));
			if (IS_ROOT(d))
				continue;

			d_drop(d);
			au_digen_dec(d);
			if (d->d_inode)
				/* todo: reset children xino?
				   cached children only? */
				au_iigen_dec(d->d_inode);
		}
	}

 out_dpages:
	au_dpages_free(&dpages);

	/* discard children */
	dentry_unhash(dentry);
	dput(dentry);
 out:
	AuTraceErr(err);
	return err;
}

/*
 * return 0 if processed.
 */
static int hin_gen_by_inode(char *name, unsigned int nlen, struct inode *inode,
			    const unsigned int isdir)
{
	int err;
	struct dentry *d;
	struct qstr *dname;

	LKTRTrace("%.*s, i%lu\n", nlen, name, inode->i_ino);

	err = 1;
	if (unlikely(inode->i_ino == AUFS_ROOT_INO)) {
		AuWarn("branch root dir was changed\n");
		err = 0;
		goto out;
	}

	if (!isdir) {
		AuDebugOn(!name);
		au_iigen_dec(inode);
		spin_lock(&dcache_lock);
		list_for_each_entry(d, &inode->i_dentry, d_alias) {
			dname = &d->d_name;
			if (dname->len != nlen
			    && memcmp(dname->name, name, nlen))
				continue;
			err = 0;
			spin_lock(&d->d_lock);
			__d_drop(d);
			au_digen_dec(d);
			spin_unlock(&d->d_lock);
			break;
		}
		spin_unlock(&dcache_lock);
	} else {
		au_fset_si(au_sbi(inode->i_sb), FAILED_REFRESH_DIRS);
		d = d_find_alias(inode);
		if (!d) {
			au_iigen_dec(inode);
			goto out;
		}

		dname = &d->d_name;
		if (dname->len == nlen && !memcmp(dname->name, name, nlen))
			err = hin_gen_tree(d);
		dput(d);
	}

 out:
	AuTraceErr(err);
	return err;
}

static int hin_gen_by_name(struct dentry *dentry, const unsigned int isdir)
{
	int err;
	struct inode *inode;

	LKTRTrace("%.*s\n", AuDLNPair(dentry));

	inode = dentry->d_inode;
	if (IS_ROOT(dentry)
	    /* || (inode && inode->i_ino == AUFS_ROOT_INO) */
		) {
		AuWarn("branch root dir was changed\n");
		return 0;
	}

	err = 0;
	if (!isdir) {
		d_drop(dentry);
		au_digen_dec(dentry);
		if (inode)
			au_iigen_dec(inode);
	} else {
		au_fset_si(au_sbi(dentry->d_sb), FAILED_REFRESH_DIRS);
		if (inode)
			err = hin_gen_tree(dentry);
	}

	AuTraceErr(err);
	return err;
}

static void hin_attr(struct inode *inode, struct inode *h_inode)
{
	struct dentry *h_dentry;

	LKTRTrace("i%lu, hi%lu\n", inode->i_ino, h_inode->i_ino);

	if (au_h_iptr(inode, au_ibstart(inode)) != h_inode)
		return;

	h_dentry = d_find_alias(h_inode);
	if (h_dentry) {
		au_update_fuse_h_inode(NULL, h_dentry);
		/* ignore an error*/
		dput(h_dentry);
	}

	au_cpup_attr_all(inode);
}

/* ---------------------------------------------------------------------- */

/* hinotify job flags */
#define AuHinJob_XINO0	1
#define AuHinJob_GEN	(1 << 1)
#define AuHinJob_DIRENT	(1 << 2)
#define AuHinJob_ATTR	(1 << 3)
#define AuHinJob_ISDIR	(1 << 4)
#define AuHinJob_TRYXINO0 (1 << 5)
#define AuHinJob_MNTPNT	(1 << 6)
#define au_ftest_hinjob(flags, name)	((flags) & AuHinJob_##name)
#define au_fset_hinjob(flags, name)	{ (flags) |= AuHinJob_##name; }
#define au_fclr_hinjob(flags, name)	{ (flags) &= ~AuHinJob_##name; }

struct hin_job_args {
	unsigned int flags;
	struct inode *inode, *h_inode, *dir, *h_dir;
	struct dentry *dentry;
	char *h_name;
	int h_nlen;
};

static int hin_job(struct hin_job_args *a)
{
	const unsigned int isdir = au_ftest_hinjob(a->flags, ISDIR);

	/* reset xino */
	if (au_ftest_hinjob(a->flags, XINO0) && a->inode)
		hin_xino(a->inode, a->h_inode);
	/* ignore this error */

	if (au_ftest_hinjob(a->flags, TRYXINO0)
	    && a->inode
	    && a->h_inode) {
		mutex_lock_nested(&a->h_inode->i_mutex, AuLsc_I_CHILD);
		if (!a->h_inode->i_nlink)
			hin_xino(a->inode, a->h_inode);
		/* ignore this error */
		mutex_unlock(&a->h_inode->i_mutex);
	}

	/* make the generation obsolete */
	if (au_ftest_hinjob(a->flags, GEN)) {
		int err = -1;
		if (a->inode)
			err = hin_gen_by_inode(a->h_name, a->h_nlen, a->inode,
					       isdir);
		if (err && a->dentry)
			hin_gen_by_name(a->dentry, isdir);
		/* ignore this error */
	}

	/* make dir entries obsolete */
	if (au_ftest_hinjob(a->flags, DIRENT) && a->inode) {
		struct au_vdir *vdir;
		IiMustWriteLock(a->inode);
		vdir = au_ivdir(a->inode);
		if (vdir)
			vdir->vd_jiffy = 0;
		/* IMustLock(a->inode); */
		/* a->inode->i_version++; */
	}

	/* update the attr */
	if (au_ftest_hinjob(a->flags, ATTR) && a->inode && a->h_inode)
		hin_attr(a->inode, a->h_inode);

	/* can do nothing but warn */
	if (au_ftest_hinjob(a->flags, MNTPNT)
	    && a->dentry
	    && d_mountpoint(a->dentry))
		AuWarn("mount-point %.*s is removed or renamed\n",
		       AuDLNPair(a->dentry));

	return 0;
}

/* ---------------------------------------------------------------------- */

enum { CHILD, PARENT };
struct postproc_args {
	struct inode *h_dir, *dir, *h_child_inode;
	u32 mask;
	unsigned int flags[2];
	unsigned int h_child_nlen;
	char h_child_name[];
};

static void postproc(void *_args)
{
	struct postproc_args *a = _args;
	struct super_block *sb;
	aufs_bindex_t bindex, bend, bfound;
	unsigned char xino, try_iput;
	int err;
	struct inode *inode;
	ino_t h_ino;
	struct hin_job_args args;
	struct dentry *dentry;
	struct au_sbinfo *sbinfo;

	AuDebugOn(!_args);
	AuDebugOn(!a->h_dir);
	AuDebugOn(!a->dir);
	AuDebugOn(!a->mask);
	LKTRTrace("mask 0x%x %s, i%lu, hi%lu, hci%lu\n",
		  a->mask, in_name(a->mask), a->dir->i_ino, a->h_dir->i_ino,
		  a->h_child_inode ? a->h_child_inode->i_ino : 0);

	inode = NULL;
	dentry = NULL;
	/*
	 * do not lock a->dir->i_mutex here
	 * because of d_revalidate() may cause a deadlock.
	 */
	sb = a->dir->i_sb;
	AuDebugOn(!sb);
	sbinfo = au_sbi(sb);
	AuDebugOn(!sbinfo);
	/* big aufs lock */
	si_noflush_write_lock(sb);

	ii_read_lock_parent(a->dir);
	bfound = -1;
	bend = au_ibend(a->dir);
	for (bindex = au_ibstart(a->dir); bindex <= bend; bindex++)
		if (au_h_iptr(a->dir, bindex) == a->h_dir) {
			bfound = bindex;
			break;
		}
	ii_read_unlock(a->dir);
	if (unlikely(bfound < 0))
		goto out;

	xino = !!au_opt_test_xino(au_mntflags(sb));
	h_ino = 0;
	if (a->h_child_inode)
		h_ino = a->h_child_inode->i_ino;

	if (a->h_child_nlen
	    && (au_ftest_hinjob(a->flags[CHILD], GEN)
		|| au_ftest_hinjob(a->flags[CHILD], MNTPNT)))
		dentry = lookup_wlock_by_name(a->h_child_name, a->h_child_nlen,
					      a->dir);
	try_iput = 0;
	if (dentry)
		inode = dentry->d_inode;
	if (xino && !inode && h_ino
	    && (au_ftest_hinjob(a->flags[CHILD], XINO0)
		|| au_ftest_hinjob(a->flags[CHILD], TRYXINO0)
		|| au_ftest_hinjob(a->flags[CHILD], GEN)
		|| au_ftest_hinjob(a->flags[CHILD], ATTR))) {
		inode = lookup_wlock_by_ino(sb, bfound, h_ino);
		try_iput = 1;
	    }

	args.flags = a->flags[CHILD];
	args.dentry = dentry;
	args.inode = inode;
	args.h_inode = a->h_child_inode;
	args.dir = a->dir;
	args.h_dir = a->h_dir;
	args.h_name = a->h_child_name;
	args.h_nlen = a->h_child_nlen;
	err = hin_job(&args);
	if (dentry) {
		if (dentry->d_fsdata)
			di_write_unlock(dentry);
		dput(dentry);
	}
	if (inode && try_iput) {
		ii_write_unlock(inode);
		iput(inode);
	}

	ii_write_lock_parent(a->dir);
	args.flags = a->flags[PARENT];
	args.dentry = NULL;
	args.inode = a->dir;
	args.h_inode = a->h_dir;
	args.dir = NULL;
	args.h_dir = NULL;
	args.h_name = NULL;
	args.h_nlen = 0;
	err = hin_job(&args);
	ii_write_unlock(a->dir);

 out:
	au_nwt_done(&sbinfo->si_nowait);
	si_write_unlock(sb);

	iput(a->h_child_inode);
	iput(a->h_dir);
	iput(a->dir);
	kfree(a);
}

static void aufs_inotify(struct inotify_watch *watch, u32 wd, u32 mask,
			 u32 cookie, const char *h_child_name,
			 struct inode *h_child_inode)
{
	struct au_hinotify *hinotify;
	struct postproc_args *args;
	int len, wkq_err;
	unsigned char isdir, isroot, wh;
	char *p;
	struct inode *dir;
	unsigned int flags[2];

	LKTRTrace("i%lu, wd %d, mask 0x%x %s, cookie 0x%x, hcname %s, hi%lu\n",
		  watch->inode->i_ino, wd, mask, in_name(mask), cookie,
		  h_child_name ? h_child_name : "",
		  h_child_inode ? h_child_inode->i_ino : 0);

	/* if IN_UNMOUNT happens, there must be another bug */
	if (mask & (IN_IGNORED | IN_UNMOUNT)) {
		put_inotify_watch(watch);
		return;
	}

#ifdef DbgInotify
	if (!h_child_name || strcmp(h_child_name, AUFS_XINO_FNAME))
		AuDbg("i%lu, wd %d, mask 0x%x %s, cookie 0x%x, hcname %s,"
		      " hi%lu\n",
		      watch->inode->i_ino, wd, mask, in_name(mask), cookie,
		      h_child_name ? h_child_name : "",
		      h_child_inode ? h_child_inode->i_ino : 0);
#endif

	hinotify = container_of(watch, struct au_hinotify, hin_watch);
	AuDebugOn(!hinotify || !hinotify->hin_aufs_inode);
	if (au_hin_test_ignore(mask, hinotify)) {
#ifdef DbgInotify
		AuDbg("ignored\n");
#endif
		return;
	}
#if 0 /* tmp debug */
	if (h_child_name && !strcmp(h_child_name, AUFS_XINO_FNAME))
	{
	AuDbg("i%lu, wd %d, mask 0x%x %s, cookie 0x%x, hcname %s, hi%lu\n",
		  watch->inode->i_ino, wd, mask, in_name(mask), cookie,
		  h_child_name ? h_child_name : "",
		  h_child_inode ? h_child_inode->i_ino : 0);
	//au_dbg_blocked();
	}
#endif

	dir = igrab(hinotify->hin_aufs_inode);
	if (!dir)
		return;
	isroot = (dir->i_ino == AUFS_ROOT_INO);
	len = 0;
	wh = 0;
	if (h_child_name) {
		len = strlen(h_child_name);
		if (!memcmp(h_child_name, AUFS_WH_PFX, AUFS_WH_PFX_LEN)) {
			h_child_name += AUFS_WH_PFX_LEN;
			len -= AUFS_WH_PFX_LEN;
			wh = 1;
		}
	}

	isdir = 0;
	if (h_child_inode)
		isdir = !!S_ISDIR(h_child_inode->i_mode);
	flags[PARENT] = AuHinJob_ISDIR;
	flags[CHILD] = 0;
	if (isdir)
		flags[CHILD] = AuHinJob_ISDIR;
	switch (mask & IN_ALL_EVENTS) {
	case IN_MODIFY:
		/*FALLTHROUGH*/
	case IN_ATTRIB:
		if (h_child_inode) {
			if (!wh)
				au_fset_hinjob(flags[CHILD], ATTR);
		} else
			au_fset_hinjob(flags[PARENT], ATTR);
		break;

		/* IN_MOVED_FROM is the first event in rename(2) */
	case IN_MOVED_FROM:
	case IN_MOVED_TO:
		AuDebugOn(!h_child_name || !h_child_inode);
		au_fset_hinjob(flags[CHILD], GEN);
		au_fset_hinjob(flags[CHILD], ATTR);
		if (1 || isdir)
			au_fset_hinjob(flags[CHILD], XINO0);
		au_fset_hinjob(flags[CHILD], MNTPNT);

		au_fset_hinjob(flags[PARENT], ATTR);
		au_fset_hinjob(flags[PARENT], DIRENT);
		break;

	case IN_CREATE:
		AuDebugOn(!h_child_name || !h_child_inode);
		au_fset_hinjob(flags[PARENT], ATTR);
		au_fset_hinjob(flags[PARENT], DIRENT);
		au_fset_hinjob(flags[CHILD], GEN);
		/* hard link */
		if (!isdir && h_child_inode->i_nlink > 1)
			au_fset_hinjob(flags[CHILD], ATTR);
		break;

	case IN_DELETE:
		/*
		 * aufs never be able to get this child inode.
		 * revalidation should be in d_revalidate()
		 * by checking i_nlink, i_generation or d_unhashed().
		 */
		AuDebugOn(!h_child_name);
		au_fset_hinjob(flags[PARENT], ATTR);
		au_fset_hinjob(flags[PARENT], DIRENT);
		au_fset_hinjob(flags[CHILD], GEN);
		au_fset_hinjob(flags[CHILD], TRYXINO0);
		au_fset_hinjob(flags[CHILD], MNTPNT);
		break;

	case IN_DELETE_SELF:
#if 0
		if (!isroot)
			au_fset_hinjob(flags[PARENT], GEN);
		/*FALLTHROUGH*/
#endif

	case IN_MOVE_SELF:
#if 0
		/*
		 * when an inotify is set to an aufs inode,
		 * such inode can be isolated and this event can be fired
		 * solely.
		 */
		AuDebugOn(h_child_name || h_child_inode);
		if (unlikely(isroot)) {
			AuWarn("root branch was moved\n");
			iput(dir);
			return;
		}
		au_fset_hinjob(flags[PARENT], XINO0);
		au_fset_hinjob(flags[PARENT], GEN);
		au_fset_hinjob(flags[PARENT], ATTR);
		au_fset_hinjob(flags[PARENT], DIRENT);
		/* au_fset_hinjob(flags[PARENT], MNTPNT); */
		break;
#endif

	case IN_ACCESS:
	default:
		AuDebugOn(1);
	}

	if (wh)
		h_child_inode = NULL;

	/* iput() and kfree() will be called in postproc() */
	/*
	 * inotify_mutex is already acquired and kmalloc/prune_icache may lock
	 * iprune_mutex. strange.
	 */
	lockdep_off();
	args = kmalloc(sizeof(*args) + len + 1, GFP_NOFS);
	lockdep_on();
	if (unlikely(!args)) {
		AuErr1("no memory\n");
		iput(dir);
		return;
	}
	args->flags[PARENT] = flags[PARENT];
	args->flags[CHILD] = flags[CHILD];
	args->mask = mask;
	args->dir = dir;
	args->h_dir = igrab(watch->inode);
	if (h_child_inode)
		h_child_inode = igrab(h_child_inode); /* can be NULL */
	args->h_child_inode = h_child_inode;
	args->h_child_nlen = len;
	if (len) {
		p = (void *)args;
		p += sizeof(*args);
		memcpy(p, h_child_name, len + 1);
	}

	lockdep_off();
	wkq_err = au_wkq_nowait(postproc, args, dir->i_sb, /*dlgt*/0);
	lockdep_on();
	if (unlikely(wkq_err))
		AuErr("wkq %d\n", wkq_err);
}

static void aufs_inotify_destroy(struct inotify_watch *watch)
{
	return;
}

static struct inotify_operations aufs_inotify_ops = {
	.handle_event	= aufs_inotify,
	.destroy_watch	= aufs_inotify_destroy
};

/* ---------------------------------------------------------------------- */

static void au_hin_destroy_cache(void)
{
	kmem_cache_destroy(au_cachep[AuCache_HINOTIFY]);
	au_cachep[AuCache_HINOTIFY] = NULL;
}

int __init au_inotify_init(void)
{
	int err;

	err = -ENOMEM;
	au_cachep[AuCache_HINOTIFY] = AuCache(au_hinotify);
	if (au_cachep[AuCache_HINOTIFY]) {
		err = 0;
		in_handle = inotify_init(&aufs_inotify_ops);
		if (IS_ERR(in_handle)) {
			err = PTR_ERR(in_handle);
			au_hin_destroy_cache();
		}
	}
	AuTraceErr(err);
	return err;
}

void au_inotify_fin(void)
{
	inotify_destroy(in_handle);
	if (au_cachep[AuCache_HINOTIFY])
		au_hin_destroy_cache();
}
