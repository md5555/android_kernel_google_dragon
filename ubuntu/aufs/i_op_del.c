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
 * inode operations (del entry)
 *
 * $Id: i_op_del.c,v 1.6 2008/06/02 02:38:21 sfjro Exp $
 */

#include "aufs.h"

/* returns,
 * 0: wh is unnecessary
 * plus: wh is necessary
 * minus: error
 */
int au_wr_dir_need_wh(struct dentry *dentry, int isdir, aufs_bindex_t *bcpup,
		      struct dentry *locked)
{
	int need_wh, err;
	aufs_bindex_t bstart;
	struct dentry *h_dentry;
	struct super_block *sb;

	LKTRTrace("%.*s, isdir %d, *bcpup %d, locked %p\n",
		  AuDLNPair(dentry), isdir, *bcpup, locked);
	sb = dentry->d_sb;

	bstart = au_dbstart(dentry);
	LKTRTrace("bcpup %d, bstart %d\n", *bcpup, bstart);
	h_dentry = au_h_dptr(dentry, bstart);
	if (*bcpup < 0) {
		*bcpup = bstart;
		if (au_test_ro(sb, bstart, dentry->d_inode)) {
			err = AuWbrCopyup(au_sbi(sb), dentry);
			*bcpup = err;
			if (unlikely(err < 0))
				goto out;
		}
	} else
		AuDebugOn(bstart < *bcpup
			  || au_test_ro(sb, *bcpup, dentry->d_inode));
	LKTRTrace("bcpup %d, bstart %d\n", *bcpup, bstart);

	if (*bcpup != bstart) {
		err = au_cpup_dirs(dentry, *bcpup, locked);
		if (unlikely(err))
			goto out;
		need_wh = 1;
	} else {
		aufs_bindex_t old_bend, new_bend, bdiropq = -1;
		old_bend = au_dbend(dentry);
		if (isdir) {
			bdiropq = au_dbdiropq(dentry);
			au_set_dbdiropq(dentry, -1);
		}
		need_wh = au_lkup_dentry(dentry, bstart + 1, /*type*/0,
					 /*nd*/NULL);
		err = need_wh;
		if (isdir)
			au_set_dbdiropq(dentry, bdiropq);
		if (unlikely(err < 0))
			goto out;
		new_bend = au_dbend(dentry);
		if (!need_wh && old_bend != new_bend) {
			au_set_h_dptr(dentry, new_bend, NULL);
			au_set_dbend(dentry, old_bend);
#if 0 /* todo: remove this? */
		} else if (!au_h_dptr(dentry, new_bend)->d_inode) {
			LKTRTrace("negative\n");
			au_set_h_dptr(dentry, new_bend, NULL);
			au_set_dbend(dentry, old_bend);
			need_wh = 0;
#endif
		}
	}
	LKTRTrace("need_wh %d\n", need_wh);
	err = need_wh;

 out:
	AuTraceErr(err);
	return err;
}

/*
 * simple tests for the removal inode operations.
 * following the checks in vfs, plus the parent-child relationship.
 */
int au_may_del(struct dentry *dentry, aufs_bindex_t bindex,
	       struct dentry *h_parent, int isdir, struct au_ndx *ndx)
{
	int err, exist;
	struct super_block *sb;
	struct dentry *h_dentry;
	struct inode *h_inode;
	umode_t h_mode;

	LKTRTrace("%.*s/%.*s, b%d, dir %d\n",
		  AuDLNPair(h_parent), AuDLNPair(dentry), bindex, isdir);

	sb = dentry->d_sb;
	exist = !!dentry->d_inode;
	h_dentry = au_h_dptr(dentry, bindex);
	h_inode = h_dentry->d_inode;
	if (exist) {
		err = -ENOENT;
		if (unlikely(!h_inode || !h_inode->i_nlink))
			goto out;

		h_mode = h_inode->i_mode;
		if (!isdir) {
			err = -EISDIR;
			if (unlikely(S_ISDIR(h_mode)))
				goto out;
		} else if (unlikely(!S_ISDIR(h_mode))) {
			err = -ENOTDIR;
			goto out;
		}
	} else {
		/* rename(2) case */
		err = -EIO;
		if (unlikely(h_inode))
			goto out;
	}

	err = -ENOENT;
	/* expected parent dir is locked */
	if (unlikely(h_parent != h_dentry->d_parent))
		goto out;
	err = 0;

	/*
	 * some filesystem may unlink a dir and corrupt its consistency.
	 * so let's try heavy test.
	 */
	if (1 /*unlikely(au_opt_test(au_mntflags(sb), UDBA_INOTIFY))*/) {
		struct dentry *h_latest;
		struct qstr *qstr = &dentry->d_name;

		err = -EACCES;
		if (unlikely(au_test_h_perm(h_parent->d_inode,
					    MAY_EXEC | MAY_WRITE,
					    au_ftest_ndx(ndx->flags, DLGT))))
			goto out;

		h_latest = au_sio_lkup_one(qstr->name, h_parent, qstr->len,
					   ndx);
		err = -EIO;
		if (IS_ERR(h_latest))
			goto out;
		dput(h_latest);
		if (h_latest == h_dentry)
			err = 0;
	}

 out:
	AuTraceErr(err);
	return err;
}

static struct dentry *
lock_hdir_create_wh(struct dentry *dentry, int isdir, aufs_bindex_t *rbcpup,
		    struct au_dtime *dt)
{
	struct dentry *wh_dentry;
	int err, need_wh;
	struct dentry *h_parent, *parent, *gparent;
	struct inode *dir, *h_dir, *gdir;
	struct au_ndx ndx;
	struct super_block *sb;
	struct au_hinode *hgdir;
	aufs_bindex_t bcpup;
	unsigned int mnt_flags;

	LKTRTrace("%.*s, isdir %d\n", AuDLNPair(dentry), isdir);

	need_wh = au_wr_dir_need_wh(dentry, isdir, rbcpup, NULL);
	err = need_wh;
	wh_dentry = ERR_PTR(err);
	if (unlikely(err < 0))
		goto out;

	/* todo: meaningless lock if CONFIG_AUFS_DEBUG is disabled. */
	hgdir = NULL;
	bcpup = *rbcpup;
	sb = dentry->d_sb;
	mnt_flags = au_mntflags(sb);
	parent = dentry->d_parent; /* dir inode is locked */
	if (unlikely(au_opt_test(mnt_flags, UDBA_INOTIFY)
		     && !IS_ROOT(parent))) {
		gparent = dget_parent(parent);
		gdir = gparent->d_inode;
		ii_read_lock_parent2(gdir);
		hgdir = au_hi(gdir, bcpup);
		ii_read_unlock(gdir);
		dput(gparent);
	}
	dir = parent->d_inode;
	IMustLock(dir);
	h_parent = au_h_dptr(parent, bcpup);
	h_dir = h_parent->d_inode;

	AuDbgSleep_UdbaRace();
	au_hdir_lock(h_dir, dir, bcpup);
	/* todo: revalidate the lower dentry? */

	if (!au_opt_test(mnt_flags, UDBA_NONE) && au_dbstart(dentry) == bcpup) {
		ndx.nfsmnt = au_nfsmnt(sb, bcpup);
		ndx.flags = 0;
		if (unlikely(au_test_dlgt(mnt_flags)))
			au_fset_ndx(ndx.flags, DLGT);
		ndx.nd = NULL;
		/* ndx.br = au_sbr(sb, bcpup); */
		/* ndx.nd_file = NULL; */
		err = au_may_del(dentry, bcpup, h_parent, isdir, &ndx);
		wh_dentry = ERR_PTR(err);
		if (unlikely(err))
			goto out_dir;
	}

	au_dtime_store(dt, parent, h_parent, hgdir);
	wh_dentry = NULL;
	if (!need_wh)
		goto out; /* success, no need to create whiteout */

	ndx.nfsmnt = au_nfsmnt(sb, bcpup);
	ndx.flags = 0;
	if (unlikely(au_test_dlgt(mnt_flags)))
		au_fset_ndx(ndx.flags, DLGT);
	ndx.nd = NULL;
	/* ndx.br = NULL; */
	wh_dentry = au_wh_create(dir, dentry, bcpup, h_parent, &ndx);
	if (!IS_ERR(wh_dentry))
		goto out; /* success */
	/* returns with the parent is locked and wh_dentry is DGETed */

 out_dir:
	au_hdir_unlock(h_dir, dir, bcpup);
 out:
	AuTraceErrPtr(wh_dentry);
	return wh_dentry;
}

static int renwh_and_rmdir(struct dentry *dentry, aufs_bindex_t bindex,
			   struct au_nhash *whlist, struct inode *dir)
{
	int rmdir_later, err;
	struct dentry *h_dentry;
	struct inode *inode, *h_inode;
	struct super_block *sb;

	LKTRTrace("%.*s, b%d\n", AuDLNPair(dentry), bindex);

	inode = NULL;
	h_inode = NULL;
	sb = dentry->d_sb;
	if (unlikely(au_opt_test(au_mntflags(sb), UDBA_INOTIFY))) {
		inode = dentry->d_inode;
		h_inode = au_h_iptr(inode, bindex);
		au_hdir2_lock(h_inode, inode, bindex);
	}
	err = au_whtmp_ren(dir, dentry, bindex, /*noself*/1);
	if (unlikely(inode))
		au_hdir_unlock(h_inode, inode, bindex);
	if (unlikely(err))
		goto out;

	h_dentry = au_h_dptr(dentry, bindex);
	if (!au_test_nfs(h_dentry->d_sb)) {
		const int dirwh = au_sbi(sb)->si_dirwh;
		rmdir_later = (dirwh <= 1);
		if (!rmdir_later)
			rmdir_later = au_nhash_test_longer_wh(whlist, bindex,
							      dirwh);
		if (rmdir_later)
			return rmdir_later;
	}

	err = au_whtmp_rmdir(h_dentry, whlist, bindex, dir, dentry->d_inode,
			     /*noself*/1);
	if (unlikely(err)) {
		AuIOErr("rmdir %.*s, b%d failed, %d. ignored\n",
			AuDLNPair(h_dentry), bindex, err);
		err = 0;
	}

 out:
	AuTraceErr(err);
	return err;
}

static void epilog(struct inode *dir, struct dentry *dentry,
		   aufs_bindex_t bindex)
{
	/* todo: unnecessary? */
	d_drop(dentry);
	dentry->d_inode->i_ctime = dir->i_ctime;

	if (atomic_read(&dentry->d_count) == 1) {
		au_set_h_dptr(dentry, au_dbstart(dentry), NULL);
		au_update_dbstart(dentry);
	}
	if (au_ibstart(dir) == bindex)
		au_cpup_attr_timesizes(dir);
	dir->i_version++;
}

/* revert flags */
#define AuRev_DLGT	1
#define au_ftest_rev(flags, name)	((flags) & AuRev_##name)
#define au_fset_rev(flags, name)	{ (flags) |= AuRev_##name; }
#define au_fclr_rev(flags, name)	{ (flags) &= ~AuRev_##name; }
#ifndef CONFIG_AUFS_DLGT
#undef AuRev_DLGT
#define AuRev_DLGT	0
#endif

static int do_revert(int err, struct dentry *wh_dentry, struct dentry *dentry,
		     aufs_bindex_t bwh, struct au_dtime *dt, unsigned int flags)
{
	int rerr;
	struct inode *dir;

	dir = wh_dentry->d_parent->d_inode; /* dir inode is locked */
	IMustLock(dir);
	rerr = au_wh_unlink_dentry(dir, wh_dentry, dentry, dir,
				   au_ftest_rev(flags, DLGT));
	if (!rerr) {
		au_set_dbwh(dentry, bwh);
		au_dtime_revert(dt);
		return 0;
	}

	AuIOErr("%.*s reverting whiteout failed(%d, %d)\n",
		AuDLNPair(dentry), err, rerr);
	return -EIO;
}

/* ---------------------------------------------------------------------- */

int aufs_unlink(struct inode *dir, struct dentry *dentry)
{
	int err, dlgt;
	struct inode *inode, *h_dir;
	struct dentry *parent, *wh_dentry, *h_dentry;
	struct au_dtime dt;
	aufs_bindex_t bwh, bindex, bstart;
	struct super_block *sb;
	struct vfsub_args vargs;

	LKTRTrace("i%lu, %.*s\n", dir->i_ino, AuDLNPair(dentry));
	IMustLock(dir);
	inode = dentry->d_inode;
	if (unlikely(!inode))
		return -ENOENT; /* possible? */
	IMustLock(inode);

	aufs_read_lock(dentry, AuLock_DW);
	parent = dentry->d_parent; /* dir inode is locked */
	di_write_lock_parent(parent);

	bstart = au_dbstart(dentry);
	bwh = au_dbwh(dentry);
	bindex = -1;
	wh_dentry = lock_hdir_create_wh(dentry, /*isdir*/0, &bindex, &dt);
	err = PTR_ERR(wh_dentry);
	if (IS_ERR(wh_dentry))
		goto out;

	sb = dir->i_sb;
	dlgt = !!au_test_dlgt(au_mntflags(sb));
	AuDebugOn(au_dbstart(dentry) != bstart);
	h_dentry = au_h_dptr(dentry, bstart);
	dget(h_dentry);

	if (bindex == bstart) {
		vfsub_args_init(&vargs, NULL, dlgt, 0);
		h_dir = h_dentry->d_parent->d_inode; /* dir inode is locked */
		IMustLock(h_dir);
		err = vfsub_unlink(h_dir, h_dentry, &vargs);
	} else {
		/* dir inode is locked */
		AuDebugOn(!wh_dentry
			  || wh_dentry->d_parent != au_h_dptr(parent, bindex));
		h_dir = wh_dentry->d_parent->d_inode;
		IMustLock(h_dir);
		err = 0;
	}

	if (!err) {
		drop_nlink(inode);
#if 0 /* todo: update plink? */
		if (unlikely(!inode->i_nlink
			     && au_plink_test(sb, inode)
			     /* && atomic_read(&inode->i_count) == 2) */)) {
			au_debug_on();
			DbgInode(inode);
			au_debug_off();
		}
#endif
		epilog(dir, dentry, bindex);

		/* update target timestamps */
		if (bindex == bstart) {
			au_update_fuse_h_inode(NULL, h_dentry); /*ignore*/
			inode->i_ctime = h_dentry->d_inode->i_ctime;
		} else
			/* todo: this timestamp may be reverted later */
			inode->i_ctime = h_dir->i_ctime;
		goto out_unlock; /* success */
	}

	/* revert */
	if (wh_dentry) {
		int rerr;
		unsigned int rev_flags;

		rev_flags = 0;
		if (unlikely(dlgt))
			au_fset_rev(rev_flags, DLGT);
		rerr = do_revert(err, wh_dentry, dentry, bwh, &dt, rev_flags);
		if (rerr)
			err = rerr;
	}

 out_unlock:
	au_hdir_unlock(h_dir, dir, bindex);
	dput(wh_dentry);
	dput(h_dentry);
 out:
	di_write_unlock(parent);
	aufs_read_unlock(dentry, AuLock_DW);
	AuTraceErr(err);
	return err;
}

int aufs_rmdir(struct inode *dir, struct dentry *dentry)
{
	int err, rmdir_later;
	struct inode *inode, *h_dir;
	struct dentry *parent, *wh_dentry, *h_dentry;
	struct au_dtime dt;
	aufs_bindex_t bwh, bindex, bstart;
	struct au_whtmp_rmdir_args *args;
	struct au_nhash *whlist;
	struct super_block *sb;
	unsigned int mnt_flags;

	LKTRTrace("i%lu, %.*s\n", dir->i_ino, AuDLNPair(dentry));
	IMustLock(dir);
	inode = dentry->d_inode;
	if (unlikely(!inode))
		return -ENOENT; /* possible? */
	IMustLock(inode);

	whlist = au_nhash_new(GFP_TEMPORARY);
	err = PTR_ERR(whlist);
	if (IS_ERR(whlist))
		goto out;

	err = -ENOMEM;
	args = kmalloc(sizeof(*args), GFP_TEMPORARY);
	if (unlikely(!args))
		goto out_whlist;

	aufs_read_lock(dentry, AuLock_DW);
	parent = dentry->d_parent; /* dir inode is locked */
	di_write_lock_parent(parent);
	err = au_test_empty(dentry, whlist);
	if (unlikely(err))
		goto out_args;

	bstart = au_dbstart(dentry);
	bwh = au_dbwh(dentry);
	bindex = -1;
	wh_dentry = lock_hdir_create_wh(dentry, /*isdir*/1, &bindex, &dt);
	err = PTR_ERR(wh_dentry);
	if (IS_ERR(wh_dentry))
		goto out_args;

	AuDebugOn(au_dbstart(dentry) != bstart);
	h_dentry = au_h_dptr(dentry, bstart);
	dget(h_dentry);

	rmdir_later = 0;
	if (bindex == bstart) {
		h_dir = h_dentry->d_parent->d_inode; /* dir inode is locked */
		IMustLock(h_dir);
		err = renwh_and_rmdir(dentry, bstart, whlist, dir);
		if (err > 0) {
			rmdir_later = err;
			err = 0;
		}
	} else {
		/* dir inode is locked */
		AuDebugOn(!wh_dentry
			  || wh_dentry->d_parent != au_h_dptr(parent, bindex));
		h_dir = wh_dentry->d_parent->d_inode;
		IMustLock(h_dir);
		err = 0;
	}

	sb = dentry->d_sb;
	mnt_flags = au_mntflags(sb);
	if (!err) {
		if (unlikely(au_opt_test(mnt_flags, UDBA_INOTIFY)
			     && rmdir_later))
			au_reset_hinotify(inode, /*flags*/0);
		clear_nlink(inode);
		au_set_dbdiropq(dentry, -1);
		epilog(dir, dentry, bindex);

		if (rmdir_later) {
			au_whtmp_kick_rmdir(h_dentry, whlist, bstart, dir,
					    inode, /*noself*/1, args);
			args = NULL;
		}

		goto out_unlock; /* success */
	}

	/* revert */
	LKTRLabel(revert);
	if (wh_dentry) {
		int rerr;
		unsigned int rev_flags;

		rev_flags = 0;
		if (unlikely(au_test_dlgt(mnt_flags)))
			au_fset_rev(rev_flags, DLGT);
		rerr = do_revert(err, wh_dentry, dentry, bwh, &dt, rev_flags);
		if (rerr)
			err = rerr;
	}

 out_unlock:
	au_hdir_unlock(h_dir, dir, bindex);
	dput(wh_dentry);
	dput(h_dentry);
 out_args:
	di_write_unlock(parent);
	aufs_read_unlock(dentry, AuLock_DW);
	kfree(args);
 out_whlist:
	au_nhash_del(whlist);
 out:
	AuTraceErr(err);
	return err;
}
