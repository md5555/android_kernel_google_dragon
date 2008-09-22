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
 * inode operation (rename entry)
 * todo: this is crazy monster
 *
 * $Id: i_op_ren.c,v 1.14 2008/09/22 03:52:12 sfjro Exp $
 */

#include "aufs.h"

enum { SRC, DST };

#define AuRen_ISDIR	1
#define AuRen_ISSAMEDIR	(1 << 1)
#define AuRen_WHSRC	(1 << 2)
#define AuRen_WHDST	(1 << 3)
#define AuRen_DLGT	(1 << 4)
#define AuRen_VFSLOCK	(1 << 5)
#define AuRen_PINNED	(1 << 6)
#define au_ftest_ren(flags, name)	((flags) & AuRen_##name)
#define au_fset_ren(flags, name)	{ (flags) |= AuRen_##name; }
#define au_fclr_ren(flags, name)	{ (flags) &= ~AuRen_##name; }
#ifndef CONFIG_AUFS_DLGT
#undef AuRen_DLGT
#define AuRen_DLGT	0
#endif

struct au_ren_args {
	/* original args */
	struct dentry *src_dentry, *dentry;
	struct inode *src_dir, *dir;

	struct dentry *h_dentry[2], *h_parent[2], *h_trap, *h_locked[2];
	/* todo: remove them */
	struct dentry *parent[2], *gparent[2];
	struct au_pin pin[2];
	struct au_nhash whlist;
	aufs_bindex_t btgt, bstart[2];
	/* do_rename() only */
	unsigned char need_diropq, bycpup;
	struct super_block *sb;
	unsigned int flags;
	unsigned int mnt_flags;
	struct au_ndx ndx;

	/* do_rename() only */
#ifdef CONFIG_AUFS_BR_NFS
	struct au_hin_ignore ign[3];
#else
	struct au_hin_ignore ign[2];
#endif
	struct vfsub_args vargs;
	struct au_whtmp_rmdir_args *thargs;
	struct dentry *wh_dentry[2], *h_dst, *h_src;
};

/* ---------------------------------------------------------------------- */

#define RevertFailure(fmt, args...) do { \
		AuIOErrWhck("revert failure: " fmt " (%d, %d)\n", \
			    ##args, err, rerr); \
		err = -EIO; \
	} while (0)

static noinline_for_stack
void au_ren_rev_diropq(int err, struct au_ren_args *a)
{
	int rerr;
	struct mutex *h_mtx;

	/* lock inode simply since inotify is not set to h_inode. */
	h_mtx = &au_h_dptr(a->src_dentry, a->btgt)->d_inode->i_mutex;
	mutex_lock_nested(h_mtx, AuLsc_I_CHILD);
	rerr = au_diropq_remove(a->src_dentry, a->btgt,
				au_ftest_ren(a->flags, DLGT));
	mutex_unlock(h_mtx);
	if (rerr)
		RevertFailure("remove diropq %.*s", AuDLNPair(a->src_dentry));
}

static noinline_for_stack
void au_ren_rev_rename(int err, struct au_ren_args *a)
{
	int rerr;
	struct dentry *h_d;
	struct qstr *name = &a->src_dentry->d_name;

	h_d = au_lkup_one(name->name, a->h_parent[SRC], name->len, &a->ndx);
	rerr = PTR_ERR(h_d);
	if (IS_ERR(h_d)) {
		RevertFailure("au_lkup_one %.*s", AuDLNPair(a->src_dentry));
		return;
	}

	AuDebugOn(h_d->d_inode);
	vfsub_args_reinit(&a->vargs);
	vfsub_ign_hinode(&a->vargs, IN_MOVED_FROM, au_pinned_hdir(a->pin + DST,
								  a->btgt));
	vfsub_ign_hinode(&a->vargs, IN_MOVED_TO, au_pinned_hdir(a->pin + SRC,
								a->btgt));
	rerr = vfsub_rename(au_pinned_h_dir(a->pin + DST),
			    au_h_dptr(a->src_dentry, a->btgt),
			    au_pinned_h_dir(a->pin + SRC), h_d, &a->vargs);
	d_drop(h_d);
	dput(h_d);
	/* au_set_h_dptr(a->src_dentry, a->btgt, NULL); */
	if (rerr)
		RevertFailure("rename %.*s", AuDLNPair(a->src_dentry));
}

static noinline_for_stack
void au_ren_rev_cpup(int err, struct au_ren_args *a)
{
	int rerr;

	vfsub_args_reinit(&a->vargs);
	vfsub_ign_hinode(&a->vargs, IN_DELETE, au_pinned_hdir(a->pin + DST,
							      a->btgt));
	rerr = vfsub_unlink(au_pinned_h_dir(a->pin + DST), a->h_dentry[DST],
			    &a->vargs);
	au_set_h_dptr(a->src_dentry, a->btgt, NULL);
	au_set_dbstart(a->src_dentry, a->bstart[SRC]);
	if (rerr)
		RevertFailure("unlink %.*s", AuDLNPair(a->h_dentry[DST]));
}

static noinline_for_stack
void au_ren_rev_whtmp(int err, struct au_ren_args *a)
{
	int rerr;
	struct dentry *h_d;
	struct mutex *h_mtx;
	struct qstr *name = &a->dentry->d_name;

	h_d = au_lkup_one(name->name, a->h_parent[DST], name->len, &a->ndx);
	rerr = PTR_ERR(h_d);
	if (IS_ERR(h_d)) {
		RevertFailure("lookup %.*s", AuLNPair(name));
		return;
	}
	if (h_d->d_inode) {
		d_drop(h_d);
		dput(h_d);
		return;
	}

	h_mtx = &a->h_dst->d_inode->i_mutex;
	mutex_lock_nested(h_mtx, AuLsc_I_CHILD);
	au_hin_resume(au_hi(a->src_dentry->d_inode, a->btgt));
	mutex_unlock(h_mtx);
	vfsub_args_reinit(&a->vargs);
	vfsub_ign_hinode(&a->vargs, IN_MOVED_TO | IN_MOVED_FROM,
			 au_pinned_hdir(a->pin + DST, a->btgt));
	rerr = vfsub_rename(au_pinned_h_dir(a->pin + DST), a->h_dst,
			    au_pinned_h_dir(a->pin + DST), h_d, &a->vargs);
	d_drop(h_d);
	dput(h_d);
	if (!rerr) {
		au_set_h_dptr(a->dentry, a->btgt, NULL);
		au_set_h_dptr(a->dentry, a->btgt, dget(a->h_dst));
	} else
		RevertFailure("rename %.*s", AuDLNPair(a->h_dst));
}

static noinline_for_stack
void au_ren_rev_whsrc(int err, struct au_ren_args *a)
{
	int rerr;

	rerr = au_wh_unlink_dentry(au_pinned_hdir(a->pin + SRC, a->btgt),
				   a->wh_dentry[SRC], a->src_dentry, /*dlgt*/0);
	if (rerr)
		RevertFailure("unlink %.*s", AuDLNPair(a->wh_dentry[SRC]));
}
#undef RevertFailure

/* ---------------------------------------------------------------------- */

static /* noinline_for_stack */
int au_ren_or_cpup(struct au_ren_args *a)
{
	int err;

	AuTraceEnter();

	if (au_dbstart(a->src_dentry) == a->btgt) {
		if (a->need_diropq && au_dbdiropq(a->src_dentry) == a->btgt)
			a->need_diropq = 0;
		vfsub_ign_hinode(&a->vargs, IN_MOVED_FROM,
				 au_pinned_hdir(a->pin + SRC, a->btgt));
		vfsub_ign_hinode(&a->vargs, IN_MOVED_TO,
				 au_pinned_hdir(a->pin + DST, a->btgt));
		/* nfs_rename() calls d_delete() */
		if (au_test_nfs(au_pinned_h_dir(a->pin + DST)->i_sb)
		    && a->h_dentry[DST]->d_inode
		    && (S_ISDIR(a->h_dentry[DST]->d_inode->i_mode)
			|| atomic_read(&a->h_dentry[DST]->d_count) <= 2))
			vfsub_ign_hinode(&a->vargs, IN_DELETE,
					 au_pinned_hdir(a->pin + DST, a->btgt));
		AuDebugOn(au_dbstart(a->src_dentry) != a->btgt);
		err = vfsub_rename(au_pinned_h_dir(a->pin + SRC),
				   au_h_dptr(a->src_dentry, a->btgt),
				   au_pinned_h_dir(a->pin + DST),
				   a->h_dentry[DST], &a->vargs);
	} else {
		struct mutex *h_mtx = &a->h_dentry[SRC]->d_inode->i_mutex;

		a->bycpup = 1;
		mutex_lock_nested(h_mtx, AuLsc_I_CHILD);
		au_set_dbstart(a->src_dentry, a->btgt);
		au_set_h_dptr(a->src_dentry, a->btgt, dget(a->h_dentry[DST]));
		err = au_sio_cpup_single(a->src_dentry, a->btgt, a->bstart[SRC],
					 -1, !AuCpup_DTIME, a->parent[DST]);
		if (unlikely(err)) {
			au_set_h_dptr(a->src_dentry, a->btgt, NULL);
			au_set_dbstart(a->src_dentry, a->bstart[SRC]);
		}
		mutex_unlock(h_mtx);
	}

	return err;
}

static /* noinline_for_stack */
int au_ren_del_whtmp(struct au_ren_args *a)
{
	int err;

	AuTraceEnter();

	if (au_test_nfs(a->h_dst->d_sb)
	    || !au_nhash_test_longer_wh(&a->whlist, a->btgt,
					au_sbi(a->sb)->si_dirwh)) {
		err = au_whtmp_rmdir(a->dir, a->btgt, a->h_dst, &a->whlist);
		if (unlikely(err))
			AuWarn("failed removing whtmp dir %.*s (%d), "
			       "ignored.\n", AuDLNPair(a->h_dst), err);
	} else {
		au_whtmp_kick_rmdir(a->dir, a->btgt, a->h_dst, &a->whlist,
				    a->thargs);
		dput(a->h_dst);
		a->thargs = NULL;
	}

	return 0;
}

static /* noinline_for_stack */
int au_ren_diropq(struct au_ren_args *a)
{
	int err;
	struct dentry *diropq;
	struct mutex *h_mtx;

	AuTraceEnter();

	err = 0;
	h_mtx = &au_h_dptr(a->src_dentry, a->btgt)->d_inode->i_mutex;
	mutex_lock_nested(h_mtx, AuLsc_I_CHILD);
	diropq = au_diropq_create(a->src_dentry, a->btgt,
				  au_ftest_ren(a->flags, DLGT));
	mutex_unlock(h_mtx);
	if (IS_ERR(diropq))
		err = PTR_ERR(diropq);
	dput(diropq);

	return err;
}

static /* noinline_for_stack */
int do_rename(struct au_ren_args *a)
{
	int err;
	aufs_bindex_t bindex, bend;
	struct dentry *h_d;

	LKTRTrace("%.*s/%.*s, %.*s/%.*s, "
		  "hd{%p, %p}, hp{%p, %p}, wh %p, btgt %d, bstart{%d, %d}, "
		  "flags 0x%x\n",
		  AuDLNPair(a->parent[SRC]), AuDLNPair(a->src_dentry),
		  AuDLNPair(a->parent[DST]), AuDLNPair(a->dentry),
		  a->h_dentry[SRC], a->h_dentry[DST],
		  a->h_parent[SRC], a->h_parent[DST],
		  &a->whlist, a->btgt,
		  a->bstart[SRC], a->bstart[DST],
		  a->flags);

	/* prepare workqueue args */
	if (au_ftest_ren(a->flags, ISDIR) && a->h_dentry[DST]->d_inode) {
		err = -ENOMEM;
		a->thargs = kmalloc(sizeof(*a->thargs), GFP_NOFS);
		if (unlikely(!a->thargs))
			goto out;
		a->h_dst = dget(a->h_dentry[DST]);
	}

	a->ndx.nfsmnt = au_nfsmnt(a->sb, a->btgt);
	if (unlikely(au_ftest_ren(a->flags, DLGT)))
		au_fset_ndx(a->ndx.flags, DLGT);

	/* create whiteout for src_dentry */
	if (au_ftest_ren(a->flags, WHSRC)) {
		a->wh_dentry[SRC] = au_wh_create(a->src_dentry, a->btgt,
						 a->h_parent[SRC], &a->ndx);
		err = PTR_ERR(a->wh_dentry[SRC]);
		if (IS_ERR(a->wh_dentry[SRC]))
			goto out_thargs;
	}

	/* lookup whiteout for dentry */
	if (au_ftest_ren(a->flags, WHDST)) {
		h_d = au_wh_lkup(a->h_parent[DST], &a->dentry->d_name, &a->ndx);
		err = PTR_ERR(h_d);
		if (IS_ERR(h_d))
			goto out_whsrc;
		if (!h_d->d_inode)
			dput(h_d);
		else
			a->wh_dentry[DST] = h_d;
	}

	/* rename dentry to tmpwh */
	if (a->thargs) {
		struct au_hinode *hinode;

		AuDbgDentry(a->h_dentry[DST]);
		err = au_whtmp_ren(a->dir, a->btgt, a->h_dentry[DST]);
		if (unlikely(err))
			goto out_whdst;
		AuDbgDentry(a->h_dentry[DST]);
		hinode = au_hi(a->dentry->d_inode, a->btgt);
		/* todo: bad approach? */
		mutex_lock_nested(&hinode->hi_inode->i_mutex, AuLsc_I_CHILD);
		au_hin_suspend(hinode);
		mutex_unlock(&hinode->hi_inode->i_mutex);
		au_set_h_dptr(a->dentry, a->btgt, NULL);
		AuDbgDentry(a->h_dentry[DST]);
		err = au_lkup_neg(a->dentry, a->btgt);
		if (unlikely(err))
			goto out_whtmp;
		a->h_dentry[DST] = au_h_dptr(a->dentry, a->btgt);
	}

	/* cpup src */
	if (a->h_dentry[DST]->d_inode && a->bstart[SRC] != a->btgt) {
		struct mutex *h_mtx = &a->h_dentry[SRC]->d_inode->i_mutex;

		mutex_lock_nested(h_mtx, AuLsc_I_CHILD);
		err = au_sio_cpup_simple(a->src_dentry, a->btgt, -1,
					 !AuCpup_DTIME);
		mutex_unlock(h_mtx);
		if (unlikely(err))
			goto out_whtmp;
	}

	/* rename by vfs_rename or cpup */
	a->need_diropq = au_ftest_ren(a->flags, ISDIR)
		&& (a->wh_dentry[DST]
		    || au_dbdiropq(a->dentry) == a->btgt
		    /* hide the lower to keep xino */
		    || a->btgt < au_dbend(a->dentry)
		    || au_opt_test(a->mnt_flags, ALWAYS_DIROPQ));
	a->bycpup = 0;
	vfsub_args_init(&a->vargs, a->ign, au_ftest_ren(a->flags, DLGT), 0);
	err = au_ren_or_cpup(a);
	if (unlikely(err))
		goto out_whtmp;

	/* make dir opaque */
	if (a->need_diropq) {
		err = au_ren_diropq(a);
		if (unlikely(err))
			goto out_rename;
	}

	/* update target timestamps */
	AuDebugOn(au_dbstart(a->src_dentry) != a->btgt);
	a->h_src = au_h_dptr(a->src_dentry, a->btgt);
	au_update_fuse_h_inode(NULL, a->h_src); /*ignore*/
	/* fsstack_copy_attr_atime(a->src_dentry->d_inode, a->h_src->d_inode); */
	a->src_dentry->d_inode->i_ctime = a->h_src->d_inode->i_ctime;

	/* remove whiteout for dentry */
	if (a->wh_dentry[DST]) {
		err = au_wh_unlink_dentry(au_pinned_hdir(a->pin + DST, a->btgt),
					  a->wh_dentry[DST], a->dentry,
					  /*dlgt*/0);
		if (unlikely(err))
			goto out_diropq;
	}

	/* remove whtmp */
	if (a->thargs)
		/* ignore this error */
		au_ren_del_whtmp(a);

	err = 0;
	goto out_success;

 out_diropq:
	if (a->need_diropq)
		au_ren_rev_diropq(err, a);
 out_rename:
	if (!a->bycpup)
		au_ren_rev_rename(err, a);
	else
		au_ren_rev_cpup(err, a);
 out_whtmp:
	if (a->thargs)
		au_ren_rev_whtmp(err, a);
 out_whdst:
	dput(a->wh_dentry[DST]);
	a->wh_dentry[DST] = NULL;
 out_whsrc:
	if (a->wh_dentry[SRC])
		au_ren_rev_whsrc(err, a);
	d_drop(a->src_dentry);
	bend = au_dbend(a->src_dentry);
	for (bindex = au_dbstart(a->src_dentry); bindex <= bend; bindex++) {
		h_d = au_h_dptr(a->src_dentry, bindex);
		if (h_d)
			d_drop(h_d);
	}
	d_drop(a->dentry);
	bend = au_dbend(a->dentry);
	for (bindex = au_dbstart(a->dentry); bindex <= bend; bindex++) {
		h_d = au_h_dptr(a->dentry, bindex);
		if (h_d)
			d_drop(h_d);
	}
	au_update_dbstart(a->dentry);
	if (a->thargs)
		d_drop(a->h_dst);
 out_success:
	dput(a->wh_dentry[SRC]);
	dput(a->wh_dentry[DST]);
 out_thargs:
	if (a->thargs) {
		dput(a->h_dst);
		kfree(a->thargs);
	}
 out:
	AuTraceErr(err);
	return err;
}

/*
 * test if @dentry dir can be rename destination or not.
 * success means, it is a logically empty dir.
 */
static int may_rename_dstdir(struct dentry *dentry, aufs_bindex_t btgt,
			     struct au_nhash *whlist)
{
	LKTRTrace("%.*s\n", AuDLNPair(dentry));

	return au_test_empty(dentry, whlist);
}

/*
 * test if @dentry dir can be rename source or not.
 * if it can, return 0 and @children is filled.
 * success means,
 * - or, it is a logically empty dir.
 * - or, it exists on writable branch and has no children including whiteouts
 *       on the lower branch.
 */
static int may_rename_srcdir(struct dentry *dentry, aufs_bindex_t btgt)
{
	int err;
	aufs_bindex_t bstart;

	LKTRTrace("%.*s\n", AuDLNPair(dentry));

	bstart = au_dbstart(dentry);
	if (bstart != btgt) {
		struct au_nhash *whlist;

		whlist = au_nhash_new(GFP_NOFS);
		err = PTR_ERR(whlist);
		if (IS_ERR(whlist))
			goto out;
		err = au_test_empty(dentry, whlist);
		au_nhash_del(whlist);
		goto out;
	}

	if (bstart == au_dbtaildir(dentry))
		return 0; /* success */

	err = au_test_empty_lower(dentry);

 out:
	if (/* unlikely */(err == -ENOTEMPTY)) {
		AuWarn1("renaming dir who has child(ren) on multiple branches,"
			" is not supported\n");
		err = -EXDEV;
	}
	AuTraceErr(err);
	return err;
}

/* mainly for link(2) and rename(2) */
int au_wbr(struct dentry *dentry, aufs_bindex_t btgt)
{
	aufs_bindex_t bdiropq, bwh;
	struct dentry *parent;

	LKTRTrace("%.*s, b%d\n", AuDLNPair(dentry), btgt);
	parent = dentry->d_parent;
	IMustLock(parent->d_inode); /* dir is locked */

	bdiropq = au_dbdiropq(parent);
	bwh = au_dbwh(dentry);
	if (au_br_rdonly(au_sbr(dentry->d_sb, btgt))
		     || (0 <= bdiropq && bdiropq < btgt)
		     || (0 <= bwh && bwh < btgt))
		btgt = -1;

	LKTRTrace("btgt %d\n", btgt);
	return btgt;
}

/*
 * simple tests for rename.
 * following the checks in vfs, plus the parent-child relationship.
 */
static int au_may_ren(struct au_ren_args *a)
{
	int err;
	struct inode *h_inode;

	AuTraceEnter();

	if (a->bstart[SRC] == a->btgt) {
		err = au_may_del(a->src_dentry, a->btgt, a->h_parent[SRC],
				 au_ftest_ren(a->flags, ISDIR), &a->ndx);
		if (unlikely(err))
			goto out;
		err = -EINVAL;
		if (unlikely(a->h_dentry[SRC] == a->h_trap))
			goto out;
	}

	err = 0;
	if (a->bstart[DST] != a->btgt)
		goto out;

	err = -EIO;
	h_inode = a->h_dentry[DST]->d_inode;
	if (!a->dentry->d_inode) {
		if (unlikely(h_inode))
			goto out;
		err = au_may_add(a->dentry, a->btgt, a->h_parent[DST],
				 au_ftest_ren(a->flags, ISDIR), &a->ndx);
	} else {
		if (unlikely(!h_inode || !h_inode->i_nlink))
			goto out;
		err = au_may_del(a->dentry, a->btgt, a->h_parent[DST],
				 au_ftest_ren(a->flags, ISDIR), &a->ndx);
		if (unlikely(err))
			goto out;
		err = -ENOTEMPTY;
		if (unlikely(a->h_dentry[DST] == a->h_trap))
			goto out;
		err = 0;
	}

 out:
	if (unlikely(err == -ENOENT || err == -EEXIST))
		err = -EIO;
	AuTraceErr(err);
	return err;
}

/*
 * locking order
 * (VFS)
 * - src_dir and dir by lock_rename()
 * - inode if exitsts
 * (aufs)
 * - lock all
 *   + src_dentry and dentry by aufs_read_and_write_lock2() which calls,
 *     + si_read_lock
 *     + di_write_lock2_child()
 *       + di_write_lock_child()
 *	   + ii_write_lock_child()
 *       + di_write_lock_child2()
 *	   + ii_write_lock_child2()
 *     + src_parent and parent
 *       + di_write_lock_parent()
 *	   + ii_write_lock_parent()
 *       + di_write_lock_parent2()
 *	   + ii_write_lock_parent2()
 *   + if udab=inotify is specified, lock grand parents (crazy)
 *     + di_read_lock_gparent()
 *       + ii_read_lock_gparent()
 *     + di_read_lock_gparent2()
 *       + ii_read_lock_gparent2()
 *     + mutex_lock_gparent()
 *     + mutex_lock_gparent2()
 *   + lower src_dir and dir by vfsub_lock_rename()?
 *   + verify the every relations between child, parent and grand parent. if any
 *     of them failed, unlock all and return -EBUSY.
 */
static void au_ren_pin_init(struct au_pin *first, struct dentry *d1,
			    struct au_pin *next, struct dentry *d2)
{
	AuTraceEnter();

	/* AuLsc_DI_PARENT3 is for higher gparent initially */
	au_pin_init(first, d1, /*di_locked*/1, AuLsc_DI_PARENT2,
		    AuLsc_I_PARENT2, /*do_gp*/1);
	/* AuLsc_DI_PARENT4 is for lower gparent initially */
	au_pin_init(next, d2, /*di_locked*/1, AuLsc_DI_PARENT3,
		    AuLsc_I_PARENT4, /*do_gp*/1);
}

static void au_ren_fake_pin(struct au_ren_args *a)
{
	int i;
	struct au_pin1 *p;
	struct inode *h_i;

	AuTraceEnter();

	/* they increment the ref counter */
	for (i = 0; i < 2; i++) {
		p = a->pin[i].pin + AuPin_PARENT;
		au_pin_set_parent(a->pin + i, a->parent[i]);
		dput(a->parent[i]);
		h_i = a->h_parent[i]->d_inode;
		au_pin_set_h_dir(a->pin + i, h_i);
		iput(h_i);

		if (!a->gparent[i]) {
			au_pin_set_gparent(a->pin + i, NULL);
			au_pin_set_h_gdir(a->pin + i, NULL);
		} else {
			au_pin_set_gparent(a->pin + i, a->gparent[i]);
			dput(a->gparent[i]);
			h_i = au_h_iptr(a->gparent[i]->d_inode, a->btgt);
			au_pin_set_h_gdir(a->pin + i, h_i);
			iput(h_i);
		}
	}
}

/* crazy */
/* cf. i_op.c: au_do_pin() */
static int au_ren_pin4(int higher, int lower, struct au_ren_args *a)
{
	int err, i, lsc;
	struct au_pin *p;
	struct au_pin1 *p4[4];
	struct inode *h_dir;

	LKTRTrace("%d, %d\n", higher, lower);

	p = a->pin + higher;
	p4[0] = au_pin_gp(p); /* highest */
	p4[1] = p->pin + AuPin_PARENT;
	p = a->pin + lower;
	p4[2] = au_pin_gp(p);
	p4[3] = p->pin + AuPin_PARENT;

	if (a->gparent[higher]) {
		au_pin_do_set_parent(p4[0], a->gparent[higher]);
		au_pin_do_set_dentry(p4[0], a->parent[higher]);
	}
	au_pin_do_set_parent(p4[1], a->parent[higher]);
	if (a->gparent[lower]) {
		au_pin_do_set_parent(p4[2], a->gparent[lower]);
		au_pin_do_set_dentry(p4[2], a->parent[lower]);
	}
	au_pin_do_set_parent(p4[3], a->parent[lower]);

	DiMustWriteLock(p4[3]->parent);
	di_write_unlock(p4[1]->parent);
	if (p4[2]->parent)
		di_read_lock_parent2(p4[2]->parent, AuLock_IR);
	di_write_lock_parent3(p4[1]->parent);
	if (p4[0]->parent)
		di_read_lock_parent4(p4[0]->parent, AuLock_IR);

	lsc = AuLsc_I_PARENT;
	for (i = 0; i < 4; i++, lsc++) {
		if (p4[i]->parent) {
			h_dir = au_h_iptr(p4[i]->parent->d_inode, a->btgt);
			au_pin_do_set_h_dir(p4[i], h_dir);
			mutex_lock_nested(&h_dir->i_mutex, lsc);
		}
	}

	err = 0;
	AuTraceErr(err);
	return err;
}

static struct dentry *au_ren_pin3(int higher, int lower, struct au_ren_args *a)
{
	struct dentry *h_trap;
	struct au_pin *p;
	int err;

	LKTRTrace("%d, %d\n", higher, lower);

	p = a->pin + higher;
	err = au_do_pin(p->pin + AuPin_PARENT, au_pin_gp(p), a->btgt,
			/*do_gp*/1);
	h_trap = ERR_PTR(err);
	if (unlikely(err))
		goto out;
	p = a->pin + lower;
	err = au_do_pin(p->pin + AuPin_PARENT, NULL, a->btgt, /*do_gp*/0);
	h_trap = ERR_PTR(err);
	if (unlikely(err)) {
		au_do_unpin(p->pin + AuPin_PARENT, au_pin_gp(p));
		goto out;
	}
	h_trap = au_pinned_h_parent(p, a->btgt);

 out:
	AuTraceErrPtr(h_trap);
	return h_trap;
}

static struct dentry *au_ren_pin(struct au_ren_args *a)
{
	struct dentry *h_trap;
	struct inode *h_gdir;
	int err, i, same_gp;

	AuTraceEnter();
	AuDebugOn(!au_opt_test(a->mnt_flags, UDBA_INOTIFY));

	vfsub_lock_rename_mutex(a->h_dentry[SRC]->d_sb);
	au_fset_ren(a->flags, VFSLOCK);

	/* gdir is not locked */
	same_gp = 0;
	if (!IS_ROOT(a->parent[SRC]))
		a->gparent[SRC] = dget_parent(a->parent[SRC]);
	if (!IS_ROOT(a->parent[DST])) {
		a->gparent[DST] = dget_parent(a->parent[DST]);
		same_gp = (a->gparent[SRC] == a->gparent[DST]);
	}

	/*
	 * patterns
	 * - gparent[SRC] is parent[DST]
	 * - parent[SRC] is gparent[DST]
	 * - gparent[SRC] is gparent[DST]
	 * - gparent[SRC] is a descendant of parent[DST]
	 * - parent[SRC] is an ancestor of gparent[DST]
	 * - not within grand parent range
	 */
	err = 0;
	h_trap = ERR_PTR(-EBUSY);
	if (a->gparent[SRC] == a->parent[DST]) {
		LKTRLabel(here);
		au_ren_pin_init(a->pin + DST, a->dentry, a->pin + SRC,
				a->src_dentry);
		h_trap = au_ren_pin3(DST, SRC, a);
		if (!IS_ERR(h_trap)) {
			h_gdir = au_pinned_h_dir(a->pin + DST);
			err = au_verify_parent(a->h_parent[SRC], h_gdir);
			if (unlikely(err))
				h_trap = ERR_PTR(-EBUSY);
		}
	} else if (a->parent[SRC] == a->gparent[DST] || same_gp) {
		LKTRLabel(here);
		au_ren_pin_init(a->pin + SRC, a->src_dentry, a->pin + DST,
				a->dentry);
		h_trap = au_ren_pin3(SRC, DST, a);
		if (!IS_ERR(h_trap)) {
			if (!same_gp)
				h_gdir = au_pinned_h_dir(a->pin + SRC);
			else
				h_gdir = au_pinned_h_gdir(a->pin + SRC);
			err = au_verify_parent(a->h_parent[DST], h_gdir);
			if (unlikely(err))
				h_trap = ERR_PTR(-EBUSY);
		}
	} else if (a->gparent[SRC]
		   && (h_trap = au_test_subdir(a->gparent[SRC],
					       a->parent[DST]))) {
		LKTRLabel(here);
		au_ren_pin_init(a->pin + DST, a->dentry, a->pin + SRC,
				a->src_dentry);
		if (a->gparent[DST]) {
			err = au_ren_pin4(DST, SRC, a);
			if (unlikely(err))
				h_trap = ERR_PTR(err);
		} else {
			struct dentry *t;
			t = au_ren_pin3(DST, SRC, a);
			AuDebugOn(t == h_trap);
		}
	} else /* if (a->gparent[DST]
		  && (h_trap = au_test_subdir(a->gparent[DST],
		  a->parent[SRC]))) */ {
		LKTRLabel(here);
		h_trap = NULL;
		if (a->gparent[DST])
			h_trap = au_test_subdir(a->gparent[DST],
						a->parent[SRC]);
		au_ren_pin_init(a->pin + SRC, a->src_dentry, a->pin + DST,
				a->dentry);
		err = au_ren_pin4(SRC, DST, a);
		if (unlikely(err))
			h_trap = ERR_PTR(err);
	}
	au_fset_ren(a->flags, PINNED);

	if (!IS_ERR(h_trap)) {
		err = 0;
		for (i = 0; !err && i < 2; i++) {
			h_gdir = au_pinned_h_gdir(a->pin + i);
			if (h_gdir)
				err = au_verify_parent(a->h_parent[i], h_gdir);
		}
		if (unlikely(err))
			h_trap = ERR_PTR(err);
	}

	dput(a->gparent[SRC]);
	dput(a->gparent[DST]);
	/* memset(a->gparent, 0, sizeof(a->gparent)); */
	AuTraceErrPtr(h_trap);
	return h_trap;
}

static void au_ren_unlock(struct au_ren_args *a)
{
	int i;

	AuTraceEnter();

	if (a->h_locked[0])
		vfsub_unlock_rename(a->h_locked[0], a->h_locked[1]);
	if (au_ftest_ren(a->flags, PINNED)) {
		au_unpin(a->pin + SRC);
		au_unpin(a->pin + DST);
		memset(a->gparent, 0, sizeof(a->gparent));
	}
	if (au_ftest_ren(a->flags, VFSLOCK))
		vfsub_unlock_rename_mutex(a->h_dentry[SRC]->d_sb);
	for (i = 0; i < 2; i++)
		if (unlikely(a->gparent[i])) {
			di_read_unlock(a->gparent[i], AuLock_IR);
			dput(a->gparent[i]);
		}
}

static int au_ren_lock(struct au_ren_args *a)
{
	int err;
	const int hinotify = au_opt_test(a->mnt_flags, UDBA_INOTIFY);

	AuTraceEnter();

	err = 0;
	if (!hinotify
	    || (au_ftest_ren(a->flags, ISSAMEDIR) && IS_ROOT(a->parent[SRC]))) {
		au_ren_pin_init(a->pin + SRC, a->src_dentry, a->pin + DST,
				a->dentry);
		LKTRLabel(here);
		a->h_locked[0] = a->h_parent[SRC];
		a->h_locked[1] = a->h_parent[DST];
		a->h_trap = vfsub_lock_rename(a->h_locked[0], a->h_locked[1]);
		au_ren_fake_pin(a);
	} else if (au_ftest_ren(a->flags, ISSAMEDIR)
		   && !IS_ROOT(a->parent[SRC])) {
		/* this and next block should not be compiled when
		   hinotify is not enabled */
		/* irregular/tricky rename lock */
		LKTRLabel(here);
		au_ren_pin_init(a->pin + SRC, a->src_dentry, a->pin + DST,
				a->dentry);
		a->gparent[SRC] = dget_parent(a->parent[SRC]);
		di_read_lock_parent2(a->gparent[SRC], AuLock_IR);
		a->h_locked[0] = a->h_parent[SRC];
		a->h_locked[1] = dget_parent(a->h_parent[SRC]);
		a->h_trap = vfsub_lock_rename(a->h_locked[0], a->h_locked[1]);
		err = au_verify_parent(a->h_parent[SRC],
				       a->h_locked[1]->d_inode);
		dput(a->h_locked[1]);
		if (!err)
			au_ren_fake_pin(a);
	} else {
		/* 3 or 4 dir locks. crazy */
		LKTRLabel(here);
		a->h_trap = au_ren_pin(a);
		if (IS_ERR(a->h_trap))
			err = PTR_ERR(a->h_trap);
	}

	if (!err && au_dbstart(a->src_dentry) == a->btgt)
		err = au_verify_parent(a->h_dentry[SRC],
				       a->h_parent[SRC]->d_inode);
	if (!err && au_dbstart(a->dentry) == a->btgt)
		err = au_verify_parent(a->h_dentry[DST],
				       a->h_parent[DST]->d_inode);
	if (unlikely(err)) {
		err = -EBUSY;
		au_ren_unlock(a);
	}
	AuTraceErr(err);
	return err;
}

int aufs_rename(struct inode *src_dir, struct dentry *src_dentry,
		struct inode *dir, struct dentry *dentry)
{
	int err;
	aufs_bindex_t bend, bindex;
	unsigned char do_dt_dstdir, hinotify;
	struct inode *inode[2];
	enum { PARENT, CHILD };
	/* reduce stack space */
	struct {
		struct au_ren_args a;
		struct au_dtime dt[2][2];
	} *p;
	struct au_wr_dir_args wr_dir_args = {
		/* .force_btgt	= -1, */
		.flags		= AuWrDir_ADD_ENTRY
	};

	//lktr_set_pid(current->pid, LktrArrayPid);
	LKTRTrace("i%lu, %.*s, i%lu, %.*s\n",
		  src_dir->i_ino, AuDLNPair(src_dentry),
		  dir->i_ino, AuDLNPair(dentry));
	AuDebugOn(IS_ROOT(src_dentry) || IS_ROOT(dentry));
	IMustLock(src_dir);
	IMustLock(dir);
	inode[DST] = dentry->d_inode;
	if (inode[DST]) {
		IMustLock(inode[DST]);
		au_igrab(inode[DST]);
	}

	err = -ENOMEM;
	BUILD_BUG_ON(sizeof(*p) > PAGE_SIZE);
	p = kzalloc(sizeof(*p), GFP_NOFS);
	if (unlikely(!p))
		goto out;

	err = -ENOTDIR;
	p->a.src_dir = src_dir;
	p->a.src_dentry = src_dentry;
	p->a.dir = dir;
	p->a.dentry = dentry;
	p->a.sb = src_dentry->d_sb;
	inode[SRC] = src_dentry->d_inode;
	p->a.flags = 0;
	if (S_ISDIR(inode[SRC]->i_mode)) {
		au_fset_ren(p->a.flags, ISDIR);
		if (unlikely(inode[DST] && !S_ISDIR(inode[DST]->i_mode)))
			goto out_free;
		aufs_read_and_write_lock2(dentry, src_dentry, AuLock_DIR);
	} else
		aufs_read_and_write_lock2(dentry, src_dentry, 0);

	p->a.mnt_flags = au_mntflags(p->a.sb);
	if (unlikely(au_test_dlgt(p->a.mnt_flags)))
		au_fset_ren(p->a.flags, DLGT);
	p->a.parent[SRC] = src_dentry->d_parent; /* dir inode is locked */
	p->a.parent[DST] = dentry->d_parent; /* dir inode is locked */
	au_fset_ren(p->a.flags, ISSAMEDIR); /* temporary */
	di_write_lock_parent(p->a.parent[DST]);

	/* which branch we process */
	p->a.bstart[SRC] = au_dbstart(src_dentry);
	p->a.bstart[DST] = au_dbstart(dentry);
	if (au_ftest_ren(p->a.flags, ISDIR))
		au_fset_wrdir(wr_dir_args.flags, ISDIR);
	wr_dir_args.force_btgt = p->a.bstart[SRC];
	if (dentry->d_inode && p->a.bstart[DST] < p->a.bstart[SRC])
		wr_dir_args.force_btgt = p->a.bstart[DST];
	wr_dir_args.force_btgt = au_wbr(dentry, wr_dir_args.force_btgt);
	err = au_wr_dir(dentry, src_dentry, &wr_dir_args);
	p->a.btgt = err;
	if (unlikely(err < 0))
		goto out_unlock;

	/* are they available to be renamed */
	err = 0;
	au_nhash_init(&p->a.whlist);
	if (au_ftest_ren(p->a.flags, ISDIR) && inode[DST]) {
		au_set_dbstart(dentry, p->a.bstart[DST]);
		err = may_rename_dstdir(dentry, p->a.btgt, &p->a.whlist);
		au_set_dbstart(dentry, p->a.btgt);
	}
	p->a.h_dentry[DST] = au_h_dptr(dentry, au_dbstart(dentry));
	if (unlikely(err))
		goto out_unlock;
	/* todo: minor optimize,
	   their sb may be same while their bindex differs? */
	p->a.h_dentry[SRC] = au_h_dptr(src_dentry, au_dbstart(src_dentry));
	if (au_ftest_ren(p->a.flags, ISDIR)) {
		err = may_rename_srcdir(src_dentry, p->a.btgt);
		if (unlikely(err))
			goto out_children;
	}

	/* prepare the writable parent dir on the same branch */
	if (p->a.bstart[DST] == p->a.btgt) {
		au_fset_ren(p->a.flags, WHDST);
	} else {
		err = au_cpup_dirs(dentry, p->a.btgt);
		if (unlikely(err))
			goto out_children;
	}

	if (src_dir != dir) {
		/*
		 * this temporary unlock is safe,
		 * because dir->i_mutex is locked.
		 */
		di_write_unlock(p->a.parent[DST]);
		di_write_lock_parent(p->a.parent[SRC]);
		err = au_wr_dir_need_wh
			(src_dentry, au_ftest_ren(p->a.flags, ISDIR),
			 &p->a.btgt);
		di_write_unlock(p->a.parent[SRC]);
		di_write_lock2_parent(p->a.parent[SRC], p->a.parent[DST],
				      /*isdir*/1);
		au_fclr_ren(p->a.flags, ISSAMEDIR);
	} else
		err = au_wr_dir_need_wh
			(src_dentry, au_ftest_ren(p->a.flags, ISDIR),
			 &p->a.btgt);
	if (unlikely(err < 0))
		goto out_children;
	if (err)
		au_fset_ren(p->a.flags, WHSRC);

	hinotify = au_opt_test(p->a.mnt_flags, UDBA_INOTIFY);
	p->a.h_parent[SRC] = au_h_dptr(p->a.parent[SRC], p->a.btgt);
	p->a.h_parent[DST] = au_h_dptr(p->a.parent[DST], p->a.btgt);
	err = au_ren_lock(&p->a);
	if (unlikely(err))
		goto out_children;

	if (!au_opt_test(p->a.mnt_flags, UDBA_NONE)) {
		p->a.ndx.nfsmnt	= au_nfsmnt(p->a.sb, p->a.btgt);
		if (unlikely(au_ftest_ren(p->a.flags, DLGT)))
			au_fset_ndx(p->a.ndx.flags, DLGT);
		err = au_may_ren(&p->a);
		if (unlikely(err))
			goto out_hdir;
		memset(&p->a.ndx, 0, sizeof(p->a.ndx));
	}

	/* store timestamps to be revertible */
	au_dtime_store(p->dt[PARENT] + SRC, p->a.parent[SRC],
		       p->a.h_parent[SRC],
		       au_pinned_hdir(p->a.pin + SRC, p->a.btgt),
		       au_pinned_hgdir(p->a.pin + SRC, p->a.btgt)
		       /* hgdir[SRC] */);
	if (!au_ftest_ren(p->a.flags, ISSAMEDIR))
		au_dtime_store(p->dt[PARENT] + DST, p->a.parent[DST],
			       p->a.h_parent[DST],
			       au_pinned_hdir(p->a.pin + DST, p->a.btgt),
			       au_pinned_hgdir(p->a.pin + DST, p->a.btgt)
			       /* hgdir[DST] */);
	do_dt_dstdir = 0;
	if (au_ftest_ren(p->a.flags, ISDIR)) {
		au_dtime_store(p->dt[CHILD] + SRC, src_dentry,
			       p->a.h_dentry[SRC], au_hi(inode[SRC], p->a.btgt),
			       au_pinned_hdir(p->a.pin + SRC, p->a.btgt));
		if (p->a.h_dentry[DST]->d_inode) {
			do_dt_dstdir = 1;
			au_dtime_store(p->dt[CHILD] + DST, dentry,
				       p->a.h_dentry[DST],
				       au_hi(inode[DST], p->a.btgt),
				       au_pinned_hdir(p->a.pin + DST,
						      p->a.btgt));
		}
	}

	err = do_rename(&p->a);
	if (unlikely(err))
		goto out_dt;

	/* update dir attributes */
	dir->i_version++;
	if (au_ftest_ren(p->a.flags, ISDIR)) {
		/* is this updating defined in POSIX? */
		/* mutex_lock(&inode[SRC]->i_mutex); */
		au_cpup_attr_timesizes(inode[SRC]);
		/* mutex_unlock(&inode[SRC]->i_mutex); */

		au_cpup_attr_nlink(dir);
		if (inode[DST]) {
			clear_nlink(inode[DST]);
			au_cpup_attr_timesizes(inode[DST]);
		}
	}
	if (au_ibstart(dir) == p->a.btgt)
		au_cpup_attr_timesizes(dir);

	if (!au_ftest_ren(p->a.flags, ISSAMEDIR)) {
		src_dir->i_version++;
		if (au_ftest_ren(p->a.flags, ISDIR))
			au_cpup_attr_nlink(src_dir);
		if (au_ibstart(src_dir) == p->a.btgt)
			au_cpup_attr_timesizes(src_dir);
	}

	/* todo: simple d_drop(src_dentry) is not enough? */
	/* dput/iput all lower dentries */
	au_set_dbwh(src_dentry, -1);
	bend = au_dbend(src_dentry);
	for (bindex = p->a.btgt + 1; bindex <= bend; bindex++) {
		struct dentry *hd;
		hd = au_h_dptr(src_dentry, bindex);
		if (hd)
			au_set_h_dptr(src_dentry, bindex, NULL);
	}
	au_set_dbend(src_dentry, p->a.btgt);

	if (au_opt_test(p->a.mnt_flags, PLINK)
	    && !au_plink_test(src_dentry->d_sb, inode[SRC])) {
		bend = au_ibend(inode[SRC]);
		for (bindex = p->a.btgt + 1; bindex <= bend; bindex++) {
			struct inode *hi;
			hi = au_h_iptr(inode[SRC], bindex);
			if (hi) {
				au_xino_write0(p->a.sb, bindex, hi->i_ino, 0);
				/* ignore this error */
				au_set_h_iptr(inode[SRC], bindex, NULL, 0);
			}
		}
		au_set_ibend(inode[SRC], p->a.btgt);
	}
	goto out_hdir; /* success */

 out_dt:
	au_dtime_revert(p->dt[PARENT] + SRC);
	if (!au_ftest_ren(p->a.flags, ISSAMEDIR))
		au_dtime_revert(p->dt[PARENT] + DST);
	if (au_ftest_ren(p->a.flags, ISDIR) && err != -EIO) {
		struct dentry *hd;

		hd = p->dt[CHILD][SRC].dt_h_dentry;
		mutex_lock_nested(&hd->d_inode->i_mutex, AuLsc_I_CHILD);
		au_dtime_revert(p->dt[CHILD] + SRC);
		mutex_unlock(&hd->d_inode->i_mutex);
		if (do_dt_dstdir) {
			hd = p->dt[CHILD][DST].dt_h_dentry;
			mutex_lock_nested(&hd->d_inode->i_mutex, AuLsc_I_CHILD);
			au_dtime_revert(p->dt[CHILD] + DST);
			mutex_unlock(&hd->d_inode->i_mutex);
		}
	}
 out_hdir:
	au_ren_unlock(&p->a);
 out_children:
	au_nhash_fin(&p->a.whlist);
 out_unlock:
	if (unlikely(err && au_ftest_ren(p->a.flags, ISDIR))) {
		au_update_dbstart(dentry);
		d_drop(dentry);
	}
	if (!err) {
		d_move(src_dentry, dentry);
#if 0
		lktr_set_pid(current->pid, LktrArrayPid);
		AuDbgDentry(src_dentry);
		AuDbgDentry(dentry);
		lktr_clear_pid(current->pid, LktrArrayPid);
#endif
#if 0
		if (inode[DST]) {
			inode[DST]->i_flags |= S_DEAD;
			ii_write_unlock(inode[DST]);
		}
#endif
	}
	if (au_ftest_ren(p->a.flags, ISSAMEDIR))
		di_write_unlock(p->a.parent[DST]);
	else
		di_write_unlock2(p->a.parent[SRC], p->a.parent[DST]);
	aufs_read_and_write_unlock2(dentry, src_dentry);
 out_free:
	kfree(p);
 out:
	iput(inode[DST]);
	AuTraceErr(err);
	//lktr_clear_pid(current->pid, LktrArrayPid);
	return err;
}
