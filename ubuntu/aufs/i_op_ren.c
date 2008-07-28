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
 * todo: this is monster
 *
 * $Id: i_op_ren.c,v 1.6 2008/06/02 02:38:21 sfjro Exp $
 */

#include "aufs.h"

enum { SRC, DST };

#define AuRen_ISDIR	1
#define AuRen_ISSAMEDIR	(1 << 1)
#define AuRen_WHSRC	(1 << 2)
#define AuRen_WHDST	(1 << 3)
#define AuRen_DLGT	(1 << 4)
#define au_ftest_ren(flags, name)	((flags) & AuRen_##name)
#define au_fset_ren(flags, name)	{ (flags) |= AuRen_##name; }
#define au_fclr_ren(flags, name)	{ (flags) &= ~AuRen_##name; }
#ifndef CONFIG_AUFS_DLGT
#undef AuRen_DLGT
#define AuRen_DLGT	0
#endif

struct rename_args {
	struct dentry *h_dentry[2], *parent[2], *h_parent[2], *h_trap;
	struct au_nhash whlist;
	aufs_bindex_t btgt, bstart[2];
	struct super_block *sb;
	unsigned int flags;
	unsigned int mnt_flags;
};

static noinline_for_stack int
do_rename(struct inode *src_dir, struct dentry *src_dentry,
	  struct inode *dir, struct dentry *dentry, struct rename_args *a)
{
	int err, need_diropq, bycpup, rerr;
	struct au_whtmp_rmdir_args *thargs;
	struct dentry *wh_dentry[2], *h_dst, *h_src;
	struct inode *h_dir[2];
	aufs_bindex_t bindex, bend;
	struct au_hin_ignore ign;
	struct vfsub_args vargs;
	struct au_ndx ndx = {
		.flags	= 0,
		.nd	= NULL,
		/* .br	= NULL */
	};

	LKTRTrace("%.*s/%.*s, %.*s/%.*s, "
		  "hd{%p, %p}, hp{%p, %p}, wh %p, btgt %d, bstart{%d, %d}, "
		  "flags 0x%x\n",
		  AuDLNPair(a->parent[SRC]), AuDLNPair(src_dentry),
		  AuDLNPair(a->parent[DST]), AuDLNPair(dentry),
		  a->h_dentry[SRC], a->h_dentry[DST],
		  a->h_parent[SRC], a->h_parent[DST],
		  &a->whlist, a->btgt,
		  a->bstart[SRC], a->bstart[DST],
		  a->flags);

	h_dir[SRC] = a->h_parent[SRC]->d_inode;
	h_dir[DST] = a->h_parent[DST]->d_inode;

	/* prepare workqueue args */
	h_dst = NULL;
	thargs = NULL;
	if (au_ftest_ren(a->flags, ISDIR) && a->h_dentry[DST]->d_inode) {
		err = -ENOMEM;
		thargs = kmalloc(sizeof(*thargs), GFP_TEMPORARY);
		if (unlikely(!thargs))
			goto out;
		h_dst = dget(a->h_dentry[DST]);
	}

	wh_dentry[SRC] = NULL;
	wh_dentry[DST] = NULL;
	ndx.nfsmnt = au_nfsmnt(a->sb, a->btgt);
	if (unlikely(au_ftest_ren(a->flags, DLGT)))
		au_fset_ndx(ndx.flags, DLGT);

	/* create whiteout for src_dentry */
	if (au_ftest_ren(a->flags, WHSRC)) {
		wh_dentry[SRC] = au_wh_create(src_dir, src_dentry, a->btgt,
					      a->h_parent[SRC], &ndx);
		err = PTR_ERR(wh_dentry[SRC]);
		if (IS_ERR(wh_dentry[SRC]))
			goto out_thargs;
	}

	/* lookup whiteout for dentry */
	if (au_ftest_ren(a->flags, WHDST)) {
		struct dentry *d;

		d = au_wh_lkup(a->h_parent[DST], &dentry->d_name, &ndx);
		err = PTR_ERR(d);
		if (IS_ERR(d))
			goto out_whsrc;
		if (!d->d_inode)
			dput(d);
		else
			wh_dentry[DST] = d;
	}

	/* rename dentry to tmpwh */
	if (thargs) {
		err = au_whtmp_ren(dir, dentry, a->btgt, /*noself*/0);
		if (unlikely(err))
			goto out_whdst;
		au_set_h_dptr(dentry, a->btgt, NULL);
		err = au_lkup_neg(dentry, a->btgt);
		if (unlikely(err))
			goto out_whtmp;
		a->h_dentry[DST] = au_h_dptr(dentry, a->btgt);
	}

	/* cpup src */
	if (a->h_dentry[DST]->d_inode && a->bstart[SRC] != a->btgt) {
		mutex_lock_nested(&a->h_dentry[SRC]->d_inode->i_mutex,
				  AuLsc_I_CHILD);
		err = au_sio_cpup_simple(src_dentry, a->btgt, -1,
					 !AuCpup_DTIME);
		mutex_unlock(&a->h_dentry[SRC]->d_inode->i_mutex);
		if (unlikely(err))
			goto out_whtmp;
	}

	/* rename by vfs_rename or cpup */
	need_diropq = au_ftest_ren(a->flags, ISDIR)
		&& (wh_dentry[DST]
		    || au_dbdiropq(dentry) == a->btgt
		    /* hide the lower to keep xino */
		    || a->btgt < au_dbend(dentry)
		    || au_opt_test(a->mnt_flags, ALWAYS_DIROPQ));
	bycpup = 0;
	if (au_dbstart(src_dentry) == a->btgt) {
		if (need_diropq && au_dbdiropq(src_dentry) == a->btgt)
			need_diropq = 0;
		vfsub_args_init(&vargs, &ign, au_ftest_ren(a->flags, DLGT), 0);
		if (unlikely(au_opt_test(a->mnt_flags, UDBA_INOTIFY)
			     && au_ftest_ren(a->flags, ISDIR)))
			vfsub_ign_hinode(&vargs, IN_MOVE_SELF,
					 au_hi(src_dentry->d_inode, a->btgt));
		AuDebugOn(au_dbstart(src_dentry) != a->btgt);
		err = vfsub_rename(h_dir[SRC], au_h_dptr(src_dentry, a->btgt),
				   h_dir[DST], a->h_dentry[DST], &vargs);
	} else {
		bycpup = 1;
		mutex_lock_nested(&a->h_dentry[SRC]->d_inode->i_mutex,
				  AuLsc_I_CHILD);
		au_set_dbstart(src_dentry, a->btgt);
		au_set_h_dptr(src_dentry, a->btgt, dget(a->h_dentry[DST]));
		err = au_sio_cpup_single(src_dentry, a->btgt, a->bstart[SRC],
					 -1, !AuCpup_DTIME);
		if (unlikely(err)) {
			au_set_h_dptr(src_dentry, a->btgt, NULL);
			au_set_dbstart(src_dentry, a->bstart[SRC]);
		}
		mutex_unlock(&a->h_dentry[SRC]->d_inode->i_mutex);
	}
	if (unlikely(err))
		goto out_whtmp;

	/* make dir opaque */
	if (need_diropq) {
		struct dentry *diropq;
		struct inode *h_inode;

		h_inode = au_h_dptr(src_dentry, a->btgt)->d_inode;
		au_hdir_lock(h_inode, src_dentry->d_inode, a->btgt);
		diropq = au_diropq_create(src_dentry, a->btgt,
					  au_ftest_ren(a->flags, DLGT));
		au_hdir_unlock(h_inode, src_dentry->d_inode, a->btgt);
		err = PTR_ERR(diropq);
		if (IS_ERR(diropq))
			goto out_rename;
		dput(diropq);
	}

	/* update target timestamps */
	AuDebugOn(au_dbstart(src_dentry) != a->btgt);
	h_src = au_h_dptr(src_dentry, a->btgt);
	au_update_fuse_h_inode(NULL, h_src); /*ignore*/
	/* fsstack_copy_attr_atime(src_dentry->d_inode, h_src->d_inode); */
	src_dentry->d_inode->i_ctime = h_src->d_inode->i_ctime;

	/* remove whiteout for dentry */
	if (wh_dentry[DST]) {
		err = au_wh_unlink_dentry(h_dir[DST], wh_dentry[DST],
					  dentry, dir, /*dlgt*/0);
		if (unlikely(err))
			goto out_diropq;
	}

	/* remove whtmp */
	if (thargs) {
		if (au_test_nfs(h_dst->d_sb)
		    || !au_nhash_test_longer_wh(&a->whlist, a->btgt,
						au_sbi(a->sb)->si_dirwh)) {
			err = au_whtmp_rmdir(h_dst, &a->whlist, a->btgt, dir,
					     dentry->d_inode, /*noself*/0);
			if (unlikely(err))
				AuWarn("failed removing whtmp dir %.*s (%d), "
				       "ignored.\n", AuDLNPair(h_dst), err);
		} else {
			au_whtmp_kick_rmdir(h_dst, &a->whlist, a->btgt, dir,
					    dentry->d_inode, /*noself*/0,
					    thargs);
			dput(h_dst);
			thargs = NULL;
		}
	}
	err = 0;
	goto out_success;

#define RevertFailure(fmt, args...) do { \
		AuIOErrWhck("revert failure: " fmt " (%d, %d)\n", \
			    ##args, err, rerr); \
		err = -EIO; \
	} while (0)

 out_diropq:
	if (need_diropq) {
		struct inode *h_inode;

		h_inode = au_h_dptr(src_dentry, a->btgt)->d_inode;
		/* lock inode simply since inotify is not set to h_inode. */
		mutex_lock_nested(&h_inode->i_mutex, AuLsc_I_PARENT);
		rerr = au_diropq_remove(src_dentry, a->btgt,
					au_ftest_ren(a->flags, DLGT));
		mutex_unlock(&h_inode->i_mutex);
		if (rerr)
			RevertFailure("remove diropq %.*s",
				      AuDLNPair(src_dentry));
	}
 out_rename:
	if (!bycpup) {
		struct dentry *d;
		struct qstr *name = &src_dentry->d_name;

		d = au_lkup_one(name->name, a->h_parent[SRC], name->len, &ndx);
		rerr = PTR_ERR(d);
		if (IS_ERR(d)) {
			RevertFailure("au_lkup_one %.*s",
				      AuDLNPair(src_dentry));
			goto out_whtmp;
		}
		AuDebugOn(d->d_inode);
		vfsub_args_init(&vargs, &ign, au_ftest_ren(a->flags, DLGT), 0);
		if (unlikely(au_opt_test(a->mnt_flags, UDBA_INOTIFY)
			     && au_ftest_ren(a->flags, ISDIR)))
			vfsub_ign_hinode(&vargs, IN_MOVE_SELF,
					 au_hi(src_dentry->d_inode, a->btgt));
		rerr = vfsub_rename(h_dir[DST], au_h_dptr(src_dentry, a->btgt),
				    h_dir[SRC], d, &vargs);
		d_drop(d);
		dput(d);
		/* au_set_h_dptr(src_dentry, a->btgt, NULL); */
		if (rerr)
			RevertFailure("rename %.*s", AuDLNPair(src_dentry));
	} else {
		vfsub_args_init(&vargs, NULL, au_ftest_ren(a->flags, DLGT), 0);
		rerr = vfsub_unlink(h_dir[DST], a->h_dentry[DST], &vargs);
		au_set_h_dptr(src_dentry, a->btgt, NULL);
		au_set_dbstart(src_dentry, a->bstart[SRC]);
		if (rerr)
			RevertFailure("unlink %.*s",
				      AuDLNPair(a->h_dentry[DST]));
	}
 out_whtmp:
	if (thargs) {
		struct dentry *d;
		struct qstr *name = &dentry->d_name;

		d = au_lkup_one(name->name, a->h_parent[DST], name->len, &ndx);
		rerr = PTR_ERR(d);
		if (IS_ERR(d)) {
			RevertFailure("lookup %.*s", AuLNPair(name));
			goto out_whdst;
		}
		if (d->d_inode) {
			d_drop(d);
			dput(d);
			goto out_whdst;
		}
		AuDebugOn(d->d_inode);
		vfsub_args_init(&vargs, &ign, au_ftest_ren(a->flags, DLGT), 0);
		if (unlikely(0 && au_opt_test(a->mnt_flags, UDBA_INOTIFY)
			     && au_ftest_ren(a->flags, ISDIR)))
			vfsub_ign_hinode(&vargs, IN_MOVE_SELF,
					 au_hi(dentry->d_inode, a->btgt));
		rerr = vfsub_rename(h_dir[DST], h_dst, h_dir[DST], d, &vargs);
		d_drop(d);
		dput(d);
		if (rerr) {
			RevertFailure("rename %.*s", AuDLNPair(h_dst));
			goto out_whdst;
		}
		au_set_h_dptr(dentry, a->btgt, NULL);
		au_set_h_dptr(dentry, a->btgt, dget(h_dst));
	}
 out_whdst:
	dput(wh_dentry[DST]);
	wh_dentry[DST] = NULL;
 out_whsrc:
	if (wh_dentry[SRC]) {
		rerr = au_wh_unlink_dentry(h_dir[SRC], wh_dentry[SRC],
					   src_dentry, src_dir, /*dlgt*/0);
		if (rerr)
			RevertFailure("unlink %.*s", AuDLNPair(wh_dentry[SRC]));
	}
#undef RevertFailure
	d_drop(src_dentry);
	bend = au_dbend(src_dentry);
	for (bindex = au_dbstart(src_dentry); bindex <= bend; bindex++) {
		struct dentry *hd;

		hd = au_h_dptr(src_dentry, bindex);
		if (hd)
			d_drop(hd);
	}
	d_drop(dentry);
	bend = au_dbend(dentry);
	for (bindex = au_dbstart(dentry); bindex <= bend; bindex++) {
		struct dentry *hd;

		hd = au_h_dptr(dentry, bindex);
		if (hd)
			d_drop(hd);
	}
	au_update_dbstart(dentry);
	if (thargs)
		d_drop(h_dst);
 out_success:
	dput(wh_dentry[SRC]);
	dput(wh_dentry[DST]);
 out_thargs:
	if (thargs) {
		dput(h_dst);
		kfree(thargs);
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

		whlist = au_nhash_new(GFP_TEMPORARY);
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

/* todo: meaningless lock if CONFIG_AUFS_DEBUG is disabled. */
static void au_hgdirs(struct au_hinode **hgdir, struct rename_args *a)
{
	struct dentry *gparent[2];
	struct inode *gdir;

	if (!au_opt_test(a->mnt_flags, UDBA_INOTIFY))
		return;

	gparent[SRC] = NULL;
	if (!IS_ROOT(a->parent[SRC])) {
		gparent[SRC] = dget_parent(a->parent[SRC]);
		gdir = gparent[SRC]->d_inode;
		if (gparent[SRC] != a->parent[DST]) {
			ii_read_lock_parent3(gdir);
			hgdir[SRC] = au_hi(gdir, a->btgt);
			ii_read_unlock(gdir);
		} else
			hgdir[SRC] = au_hi(gdir, a->btgt);
		dput(gparent[SRC]);
	}

	if (!au_ftest_ren(a->flags, ISSAMEDIR)
	    && !IS_ROOT(a->parent[DST])
	    && a->parent[DST] != gparent[SRC]) {
		gparent[DST] = dget_parent(a->parent[DST]);
		gdir = gparent[DST]->d_inode;
		if (gparent[DST] != a->parent[SRC]) {
			ii_read_lock_parent3(gdir);
			hgdir[DST] = au_hi(gdir, a->btgt);
			ii_read_unlock(gdir);
		} else
			hgdir[DST] = au_hi(gdir, a->btgt);
		dput(gparent[DST]);
	}
}

/*
 * simple tests for rename.
 * following the checks in vfs, plus the parent-child relationship.
 */
static int au_may_ren(struct inode *src_dir, struct dentry *src_dentry,
		      struct inode *dir, struct dentry *dentry,
		      struct rename_args *a, struct au_ndx *ndx)
{
	int err;
	struct inode *h_inode;

	AuTraceEnter();

	if (a->bstart[SRC] == a->btgt) {
		err = au_may_del(src_dentry, a->btgt, a->h_parent[SRC],
				 au_ftest_ren(a->flags, ISDIR), ndx);
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
	if (!dentry->d_inode) {
		if (unlikely(h_inode))
			goto out;
		err = au_may_add(dentry, a->btgt, a->h_parent[DST],
				 au_ftest_ren(a->flags, ISDIR), ndx);
	} else {
		if (unlikely(!h_inode || !h_inode->i_nlink))
			goto out;
		err = au_may_del(dentry, a->btgt, a->h_parent[DST],
				 au_ftest_ren(a->flags, ISDIR), ndx);
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

int aufs_rename(struct inode *src_dir, struct dentry *src_dentry,
		struct inode *dir, struct dentry *dentry)
{
	int err, do_dt_dstdir, flags;
	aufs_bindex_t bend, bindex;
	struct inode *inode[2], *dirs[2];
	struct au_hinode *hgdir[2], *hdir;
	enum { PARENT, CHILD };
	/* reduce stack space */
	struct {
		struct rename_args a;
		struct au_dtime dt[2][2];
	} *p;
	struct au_wr_dir_args wr_dir_args = {
		/* .force_btgt	= -1, */
		.flags		= AuWrDir_ADD_ENTRY
	};

	LKTRTrace("i%lu, %.*s, i%lu, %.*s\n",
		  src_dir->i_ino, AuDLNPair(src_dentry),
		  dir->i_ino, AuDLNPair(dentry));
	IMustLock(src_dir);
	IMustLock(dir);
	inode[DST] = dentry->d_inode;
	if (inode[DST]) {
		IMustLock(inode[DST]);
		igrab(inode[DST]);
	}

	err = -ENOMEM;
	BUILD_BUG_ON(sizeof(*p) > PAGE_SIZE);
	p = kmalloc(sizeof(*p), GFP_TEMPORARY);
	if (unlikely(!p))
		goto out;

	err = -ENOTDIR;
	p->a.sb = src_dentry->d_sb;
	inode[SRC] = src_dentry->d_inode;
	flags = 0;
	p->a.flags = 0;
	if (S_ISDIR(inode[SRC]->i_mode)) {
		flags = AuLock_DIR;
		au_fset_ren(p->a.flags, ISDIR);
		if (unlikely(inode[DST] && !S_ISDIR(inode[DST]->i_mode)))
			goto out_free;
	}

	aufs_read_and_write_lock2(dentry, src_dentry, flags);
	p->a.mnt_flags = au_mntflags(p->a.sb);
	if (unlikely(au_test_dlgt(p->a.mnt_flags)))
		au_fset_ren(p->a.flags, DLGT);
	p->a.parent[SRC] = src_dentry->d_parent; /* dir inode is locked */
	p->a.parent[DST] = dentry->d_parent; /* dir inode is locked */
	if (src_dir == dir) {
		au_fset_ren(p->a.flags, ISSAMEDIR);
		di_write_lock_parent(p->a.parent[DST]);
	} else
		di_write_lock2_parent(p->a.parent[SRC], p->a.parent[DST],
				      /*isdir*/1);

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
	err = au_wr_dir_need_wh(src_dentry, au_ftest_ren(p->a.flags, ISDIR),
				&p->a.btgt,
				au_ftest_ren(p->a.flags, ISSAMEDIR)
				? NULL : p->a.parent[DST]);
	if (unlikely(err < 0))
		goto out_children;
	if (err)
		au_fset_ren(p->a.flags, WHSRC);
	if (p->a.bstart[DST] == p->a.btgt) {
		au_fset_ren(p->a.flags, WHDST);
	} else {
		err = au_cpup_dirs(dentry, p->a.btgt,
				   au_ftest_ren(p->a.flags, ISSAMEDIR)
				   ? NULL : p->a.parent[SRC]);
		if (unlikely(err))
			goto out_children;
	}

	hgdir[SRC] = NULL;
	hgdir[DST] = NULL;
	au_hgdirs(hgdir, &p->a);
	p->a.h_parent[SRC] = au_h_dptr(p->a.parent[SRC], p->a.btgt);
	p->a.h_parent[DST] = au_h_dptr(p->a.parent[DST], p->a.btgt);
	dirs[0] = src_dir;
	dirs[1] = dir;

	AuDbgSleep_UdbaRace();
	p->a.h_trap = au_hdir_lock_rename(p->a.h_parent, dirs, p->a.btgt,
					  au_ftest_ren(p->a.flags, ISSAMEDIR));
	/* todo: revalidate the lower dentries? */

	if (!au_opt_test(p->a.mnt_flags, UDBA_NONE)) {
		struct au_ndx ndx = {
			.nfsmnt	= au_nfsmnt(p->a.sb, p->a.btgt),
			.flags	= 0,
			.nd	= NULL,
			/* .br	= NULL */
		};
		if (unlikely(au_ftest_ren(p->a.flags, DLGT)))
			au_fset_ndx(ndx.flags, DLGT);
		err = au_may_ren(src_dir, src_dentry, dir, dentry, &p->a, &ndx);
		if (unlikely(err))
			goto out_hdir;
	}

	/* store timestamps to be revertible */
	au_dtime_store(p->dt[PARENT] + SRC, p->a.parent[SRC],
		       p->a.h_parent[SRC], hgdir[SRC]);
	if (!au_ftest_ren(p->a.flags, ISSAMEDIR))
		au_dtime_store(p->dt[PARENT] + DST, p->a.parent[DST],
			       p->a.h_parent[DST], hgdir[DST]);
	do_dt_dstdir = 0;
	if (au_ftest_ren(p->a.flags, ISDIR)) {
		hdir = NULL;
		if (unlikely(au_opt_test(p->a.mnt_flags, UDBA_INOTIFY)))
			hdir = au_hi(p->a.parent[SRC]->d_inode, p->a.btgt);
		au_dtime_store(p->dt[CHILD] + SRC, src_dentry,
			       p->a.h_dentry[SRC], hdir);
		if (p->a.h_dentry[DST]->d_inode) {
			do_dt_dstdir = 1;
			if (unlikely(au_opt_test(p->a.mnt_flags, UDBA_INOTIFY)))
				hdir = au_hi(p->a.parent[DST]->d_inode,
					     p->a.btgt);
			au_dtime_store(p->dt[CHILD] + DST, dentry,
				       p->a.h_dentry[DST], hdir);
		}
	}

	err = do_rename(src_dir, src_dentry, dir, dentry, &p->a);
	if (unlikely(err))
		goto out_dt;
	au_hdir_unlock_rename(p->a.h_parent, dirs, p->a.btgt,
			      au_ftest_ren(p->a.flags, ISSAMEDIR));

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
	goto out_children; /* success */

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
	au_hdir_unlock_rename(p->a.h_parent, dirs, p->a.btgt,
			      au_ftest_ren(p->a.flags, ISSAMEDIR));
 out_children:
	au_nhash_fin(&p->a.whlist);
 out_unlock:
	if (unlikely(err && au_ftest_ren(p->a.flags, ISDIR))) {
		au_update_dbstart(dentry);
		d_drop(dentry);
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
	return err;
}
