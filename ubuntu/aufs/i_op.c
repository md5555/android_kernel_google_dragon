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
 * inode operations (except add/del/rename)
 *
 * $Id: i_op.c,v 1.19 2008/09/08 02:39:57 sfjro Exp $
 */

#include <linux/fs_stack.h>
#include <linux/uaccess.h>
#include "aufs.h"

static int silly_lock(struct inode *inode, struct nameidata *nd)
{
	int locked = 0;
	struct super_block *sb = inode->i_sb;

	LKTRTrace("i%lu, nd %p\n", inode->i_ino, nd);

	if (!nd || !nd->path.dentry) {
		si_read_lock(sb, AuLock_FLUSH);
		ii_read_lock_child(inode);
	} else if (nd->path.dentry->d_inode != inode) {
		locked = 1;
		/* lock child first, then parent */
		si_read_lock(sb, AuLock_FLUSH);
		ii_read_lock_child(inode);
		di_read_lock_parent(nd->path.dentry, 0);
	} else {
		locked = 2;
		aufs_read_lock(nd->path.dentry, AuLock_FLUSH | AuLock_IR);
	}
	return locked;
}

static void silly_unlock(int locked, struct inode *inode, struct nameidata *nd)
{
	struct super_block *sb = inode->i_sb;

	LKTRTrace("locked %d, i%lu, nd %p\n", locked, inode->i_ino, nd);

	switch (locked) {
	case 0:
		ii_read_unlock(inode);
		si_read_unlock(sb);
		break;
	case 1:
		di_read_unlock(nd->path.dentry, 0);
		ii_read_unlock(inode);
		si_read_unlock(sb);
		break;
	case 2:
		aufs_read_unlock(nd->path.dentry, AuLock_IR);
		break;
	default:
		BUG();
	}
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)
static int h_permission(struct inode *h_inode, int mask,
			struct vfsmount *h_mnt, int brperm, int dlgt)
{
	int err, submask;
	const int write_mask = (mask & (MAY_WRITE | MAY_APPEND));

	LKTRTrace("ino %lu, mask 0x%x, brperm 0x%x\n",
		  h_inode->i_ino, mask, brperm);

	err = -EACCES;
	if (unlikely((write_mask && IS_IMMUTABLE(h_inode))
		     || ((mask & MAY_EXEC) && S_ISREG(h_inode->i_mode)
			 && (h_mnt->mnt_flags & MNT_NOEXEC))
		    ))
		goto out;

	/*
	 * - skip hidden fs test in the case of write to ro branch.
	 * - nfs dir permission write check is optimized, but a policy for
	 *   link/rename requires a real check.
	 */
	submask = mask & ~MAY_APPEND;
	if ((write_mask && !au_br_writable(brperm))
	    || (au_test_nfs(h_inode->i_sb) && S_ISDIR(h_inode->i_mode)
		&& write_mask && !(mask & MAY_READ))
	    || !h_inode->i_op
	    || !h_inode->i_op->permission) {
		/* LKTRLabel(generic_permission); */
		err = generic_permission(h_inode, submask, NULL);
	} else {
		/* LKTRLabel(h_inode->permission); */
		err = h_inode->i_op->permission(h_inode, submask);
		AuTraceErr(err);
	}

#if 1 /* todo: export? */
	if (!err)
		err = au_security_inode_permission(h_inode, mask, NULL,
						   dlgt);
#endif

 out:
	AuTraceErr(err);
	return err;
}

static int aufs_permission(struct inode *inode, int mask)
{
	int err;
	aufs_bindex_t bindex, bend;
	unsigned char locked, dlgt;
	const unsigned char isdir = S_ISDIR(inode->i_mode);
	struct inode *h_inode;
	struct super_block *sb;
	unsigned int mnt_flags;
	const int write_mask = (mask & (MAY_WRITE | MAY_APPEND));

	LKTRTrace("ino %lu, mask 0x%x, isdir %d, write_mask %d\n",
		  inode->i_ino, mask, isdir, write_mask);

	sb = inode->i_sb;
	locked = silly_lock(inode, NULL);
	mnt_flags = au_mntflags(sb);
	dlgt = !!au_test_dlgt(mnt_flags);

	if (/* unlikely */(!isdir || write_mask
			   || au_test_dirperm1(mnt_flags))) {
		h_inode = au_h_iptr(inode, au_ibstart(inode));
		AuDebugOn(!h_inode
			  || ((h_inode->i_mode & S_IFMT)
			      != (inode->i_mode & S_IFMT)));
		err = 0;
		bindex = au_ibstart(inode);
		LKTRTrace("b%d\n", bindex);
		err = h_permission(h_inode, mask, au_sbr_mnt(sb, bindex),
				   au_sbr_perm(sb, bindex), dlgt);

		if (write_mask && !err) {
			/* test whether the upper writable branch exists */
			err = -EROFS;
			for (; bindex >= 0; bindex--)
				if (!au_br_rdonly(au_sbr(sb, bindex))) {
					err = 0;
					break;
				}
		}
		goto out;
	}

	/* non-write to dir */
	err = 0;
	bend = au_ibend(inode);
	for (bindex = au_ibstart(inode); !err && bindex <= bend; bindex++) {
		h_inode = au_h_iptr(inode, bindex);
		if (!h_inode)
			continue;
		AuDebugOn(!S_ISDIR(h_inode->i_mode));

		LKTRTrace("b%d\n", bindex);
		err = h_permission(h_inode, mask, au_sbr_mnt(sb, bindex),
				   au_sbr_perm(sb, bindex), dlgt);
	}

 out:
	silly_unlock(locked, inode, NULL);
	AuTraceErr(err);
	return err;
}
#else

static int h_permission(struct inode *h_inode, int mask,
			struct nameidata *fake_nd, int brperm, int dlgt)
{
	int err, submask;
	const int write_mask = (mask & (MAY_WRITE | MAY_APPEND));

	LKTRTrace("ino %lu, mask 0x%x, brperm 0x%x\n",
		  h_inode->i_ino, mask, brperm);

	err = -EACCES;
	if (unlikely((write_mask && IS_IMMUTABLE(h_inode))
		     || ((mask & MAY_EXEC) && S_ISREG(h_inode->i_mode)
			 && fake_nd && fake_nd->path.mnt
			 && (fake_nd->path.mnt->mnt_flags & MNT_NOEXEC))
		    ))
		goto out;

	/*
	 * - skip hidden fs test in the case of write to ro branch.
	 * - nfs dir permission write check is optimized, but a policy for
	 *   link/rename requires a real check.
	 */
	submask = mask & ~MAY_APPEND;
	if ((write_mask && !au_br_writable(brperm))
	    || (au_test_nfs(h_inode->i_sb) && S_ISDIR(h_inode->i_mode)
		&& write_mask && !(mask & MAY_READ))
	    || !h_inode->i_op
	    || !h_inode->i_op->permission) {
		/* LKTRLabel(generic_permission); */
		err = generic_permission(h_inode, submask, NULL);
	} else {
		/* LKTRLabel(h_inode->permission); */
		err = h_inode->i_op->permission(h_inode, submask, fake_nd);
		AuTraceErr(err);
	}

#if 1 /* todo: export? */
	if (!err)
		err = au_security_inode_permission(h_inode, mask, fake_nd,
						   dlgt);
#endif

 out:
	AuTraceErr(err);
	return err;
}

static int aufs_permission(struct inode *inode, int mask, struct nameidata *nd)
{
	int err;
	aufs_bindex_t bindex, bend;
	unsigned char locked, dlgt, do_nd;
	const unsigned char isdir = S_ISDIR(inode->i_mode);
	struct inode *h_inode;
	struct super_block *sb;
	unsigned int mnt_flags;
	struct path path;
	const int write_mask = (mask & (MAY_WRITE | MAY_APPEND));

	LKTRTrace("ino %lu, mask 0x%x, isdir %d, write_mask %d, "
		  "nd %d{%d, %d}\n",
		  inode->i_ino, mask, isdir, write_mask,
		  !!nd, nd ? !!nd->path.dentry : 0, nd ? !!nd->path.mnt : 0);

	sb = inode->i_sb;
	locked = silly_lock(inode, nd);
	do_nd = (nd && locked >= 1);
	mnt_flags = au_mntflags(sb);
	dlgt = !!au_test_dlgt(mnt_flags);

	if (/* unlikely */(!isdir || write_mask
			   || au_test_dirperm1(mnt_flags))) {
		h_inode = au_h_iptr(inode, au_ibstart(inode));
		AuDebugOn(!h_inode
			  || ((h_inode->i_mode & S_IFMT)
			      != (inode->i_mode & S_IFMT)));
		err = 0;
		bindex = au_ibstart(inode);
		LKTRTrace("b%d\n", bindex);
		if (do_nd) {
			path = nd->path;
			nd->path.mnt = au_sbr_mnt(sb, bindex);
			nd->path.dentry = au_h_dptr(nd->path.dentry, bindex);
			path_get(&nd->path);
			err = h_permission(h_inode, mask, nd,
					   au_sbr_perm(sb, bindex), dlgt);
			path_put(&nd->path);
			nd->path = path;
		} else {
			AuDebugOn(nd && nd->path.mnt);
			err = h_permission(h_inode, mask, nd,
					   au_sbr_perm(sb, bindex), dlgt);
		}

		if (write_mask && !err) {
			/* test whether the upper writable branch exists */
			err = -EROFS;
			for (; bindex >= 0; bindex--)
				if (!au_br_rdonly(au_sbr(sb, bindex))) {
					err = 0;
					break;
				}
		}
		goto out;
	}

	/* non-write to dir */
	if (do_nd)
		path = nd->path;
	else {
		path.mnt = NULL;
		path.dentry = NULL;
	}
	err = 0;
	bend = au_ibend(inode);
	for (bindex = au_ibstart(inode); !err && bindex <= bend; bindex++) {
		h_inode = au_h_iptr(inode, bindex);
		if (!h_inode)
			continue;
		AuDebugOn(!S_ISDIR(h_inode->i_mode));

		LKTRTrace("b%d\n", bindex);
		if (do_nd) {
			nd->path.mnt = au_sbr_mnt(sb, bindex);
			nd->path.dentry = au_h_dptr(path.dentry, bindex);
			path_get(&nd->path);
			err = h_permission(h_inode, mask, nd,
					   au_sbr_perm(sb, bindex), dlgt);
			path_put(&nd->path);
		} else {
			AuDebugOn(nd && nd->path.mnt);
			err = h_permission(h_inode, mask, nd,
					   au_sbr_perm(sb, bindex), dlgt);
		}
	}
	if (do_nd)
		nd->path = path;

 out:
	silly_unlock(locked, inode, nd);
	AuTraceErr(err);
	return err;
}
#endif /* KERNEL_VERSION(2, 6, 27) */

/* ---------------------------------------------------------------------- */

static struct dentry *aufs_lookup(struct inode *dir, struct dentry *dentry,
				  struct nameidata *nd)
{
	struct dentry *ret, *parent;
	int err, npositive;
	struct inode *inode, *h_inode;
	struct nameidata tmp_nd, *ndp;
	aufs_bindex_t bstart;
	struct mutex *mtx;
	struct super_block *sb;

	LKTRTrace("dir %lu, %.*s, nd{0x%x}\n",
		  dir->i_ino, AuDLNPair(dentry), nd ? nd->flags : 0);
	AuDebugOn(IS_ROOT(dentry));
	IMustLock(dir);

	sb = dir->i_sb;
	si_read_lock(sb, AuLock_FLUSH);
	err = au_alloc_dinfo(dentry);
	ret = ERR_PTR(err);
	if (unlikely(err))
		goto out;

	/* nd can be NULL */
	ndp = au_dup_nd(au_sbi(sb), &tmp_nd, nd);
	parent = dentry->d_parent; /* dir inode is locked */
	di_read_lock_parent(parent, AuLock_IR);
	npositive = au_lkup_dentry(dentry, au_dbstart(parent), /*type*/0, ndp);
	di_read_unlock(parent, AuLock_IR);
	err = npositive;
	ret = ERR_PTR(err);
	if (unlikely(err < 0))
		goto out_unlock;

	inode = NULL;
	if (npositive) {
		bstart = au_dbstart(dentry);
		h_inode = au_h_dptr(dentry, bstart)->d_inode;
		AuDebugOn(!h_inode);
		if (!S_ISDIR(h_inode->i_mode)) {
			/*
			 * stop 'race'-ing between hardlinks under different
			 * parents.
			 */
			mtx = &au_sbr(sb, bstart)->br_xino.xi_nondir_mtx;
			mutex_lock(mtx);
			inode = au_new_inode(dentry);
			mutex_unlock(mtx);
		} else
			inode = au_new_inode(dentry);
		ret = (void *)inode;
	}
	if (!IS_ERR(inode)) {
		ret = d_splice_alias(inode, dentry);
		if (unlikely(IS_ERR(ret) && inode))
			ii_write_unlock(inode);
		AuDebugOn(nd
			  && (nd->flags & LOOKUP_OPEN)
			  && nd->intent.open.file
			  && nd->intent.open.file->f_dentry);
		au_store_fmode_exec(nd, inode);
	}

 out_unlock:
	di_write_unlock(dentry);
 out:
	si_read_unlock(sb);
	AuTraceErrPtr(ret);
	return ret;
}

/* ---------------------------------------------------------------------- */

/*
 * decide the branch and the parent dir where we will create a new entry.
 * returns new bindex or an error.
 * copyup the parent dir if needed.
 */
int au_wr_dir(struct dentry *dentry, struct dentry *src_dentry,
	      struct au_wr_dir_args *args)
{
	int err;
	aufs_bindex_t bcpup, bstart, src_bstart;
	struct super_block *sb;
	struct dentry *parent;
	struct au_sbinfo *sbinfo;
	const int add_entry = au_ftest_wrdir(args->flags, ADD_ENTRY);

	LKTRTrace("%.*s, src %p, {%d, 0x%x}\n",
		  AuDLNPair(dentry), src_dentry, args->force_btgt, args->flags);

	sb = dentry->d_sb;
	sbinfo = au_sbi(sb);
	parent = dget_parent(dentry);
	bstart = au_dbstart(dentry);
	bcpup = bstart;
	if (args->force_btgt < 0) {
		if (src_dentry) {
			src_bstart = au_dbstart(src_dentry);
			if (src_bstart < bstart)
				bcpup = src_bstart;
		} else if (add_entry) {
			err = AuWbrCreate(sbinfo, dentry,
					  au_ftest_wrdir(args->flags, ISDIR));
			bcpup = err;
		}

		if (bcpup < 0 || au_test_ro(sb, bcpup, dentry->d_inode)) {
			if (add_entry)
				err = AuWbrCopyup(sbinfo, dentry);
			else {
				di_read_lock_parent(parent, !AuLock_IR);
				err = AuWbrCopyup(sbinfo, dentry);
				di_read_unlock(parent, !AuLock_IR);
			}
			bcpup = err;
			if (unlikely(err < 0))
				goto out;
		}
	} else {
		bcpup = args->force_btgt;
		AuDebugOn(au_test_ro(sb, bcpup, dentry->d_inode));
	}
	LKTRTrace("bstart %d, bcpup %d\n", bstart, bcpup);
	if (bstart < bcpup)
		au_update_dbrange(dentry, /*do_put_zero*/1);

	err = bcpup;
	if (bcpup == bstart)
		goto out; /* success */

	/* copyup the new parent into the branch we process */
	if (add_entry) {
		au_update_dbstart(dentry);
		IMustLock(parent->d_inode);
		DiMustWriteLock(parent);
		IiMustWriteLock(parent->d_inode);
	} else
		di_write_lock_parent(parent);

	err = 0;
	if (!au_h_dptr(parent, bcpup)) {
		if (bstart < bcpup)
			err = au_cpdown_dirs(dentry, bcpup);
		else
			err = au_cpup_dirs(dentry, bcpup);
	}
	if (!err && add_entry) {
		struct dentry *h_parent;
		struct inode *h_dir;

		h_parent = au_h_dptr(parent, bcpup);
		AuDebugOn(!h_parent);
		h_dir = h_parent->d_inode;
		AuDebugOn(!h_dir);
		mutex_lock_nested(&h_dir->i_mutex, AuLsc_I_PARENT);
		err = au_lkup_neg(dentry, bcpup);
		mutex_unlock(&h_dir->i_mutex);
		if (bstart < bcpup && au_dbstart(dentry) < 0) {
			au_set_dbstart(dentry, 0);
			au_update_dbrange(dentry, /*do_put_zero*/0);
		}
	}

	if (!add_entry)
		di_write_unlock(parent);
	if (!err)
		err = bcpup; /* success */
 out:
	dput(parent);
	LKTRTrace("err %d\n", err);
	AuTraceErr(err);
	return err;
}

/* ---------------------------------------------------------------------- */

struct dentry *au_do_pinned_h_parent(struct au_pin1 *pin, aufs_bindex_t bindex)
{
	if (pin && pin->parent)
		return au_h_dptr(pin->parent, bindex);
	return NULL;
}

void au_do_unpin(struct au_pin1 *p, struct au_pin1 *gp)
{
	LKTRTrace("%p, %p\n", p, gp);
	AuDebugOn(!p);

	if (unlikely(!p->h_dir))
		return;

	LKTRTrace("p{%.*s, %d, %d, %d, %d}\n",
		  AuDLNPair(p->dentry), p->lsc_di, p->lsc_hi,
		  !!p->parent, !!p->h_dir);

	mutex_unlock(&p->h_dir->i_mutex);
	if (unlikely(gp))
		au_do_unpin(gp, NULL);
	if (!p->di_locked)
		di_read_unlock(p->parent, AuLock_IR);
	iput(p->h_dir);
	dput(p->parent);
	p->parent = NULL;
	p->h_dir = NULL;
}

int au_do_pin(struct au_pin1 *p, struct au_pin1 *gp, const aufs_bindex_t bindex,
	      const int do_gp)
{
	int err;
	struct dentry *h_dentry;

	LKTRTrace("%.*s, %d, b%d, %d\n",
		  AuDLNPair(p->dentry), !!gp, bindex, do_gp);
	AuDebugOn(do_gp && !gp);
	/* AuDebugOn(!do_gp && gp); */

	err = 0;
	if (unlikely(IS_ROOT(p->dentry)))
		goto out;

	h_dentry = NULL;
	if (bindex <= au_dbend(p->dentry))
		h_dentry = au_h_dptr(p->dentry, bindex);

	p->parent = dget_parent(p->dentry);
	if (!p->di_locked)
		di_read_lock(p->parent, AuLock_IR, p->lsc_di);
	else
		DiMustAnyLock(p->parent);
	AuDebugOn(!p->parent->d_inode);
	p->h_dir = au_igrab(au_h_iptr(p->parent->d_inode, bindex));
	/* udba case */
	if (unlikely(p->do_verify && !p->h_dir)) {
		err = -EIO;
		if (!p->di_locked)
			di_read_unlock(p->parent, AuLock_IR);
		dput(p->parent);
		p->parent = NULL;
		goto out;
	}

	if (unlikely(do_gp)) {
		gp->dentry = p->parent;
		err = au_do_pin(gp, NULL, bindex, 0);
		if (unlikely(err))
			gp->dentry = NULL;
	}
	mutex_lock_nested(&p->h_dir->i_mutex, p->lsc_hi);
	if (!err) {
		/* todo: call d_revalidate() here? */
		if (!h_dentry
		    || !p->do_verify
		    || !au_verify_parent(h_dentry, p->h_dir))
			goto out; /* success */
		else {
			AuWarn1("bypassed %.*s/%.*s?\n",
				AuDLNPair(p->parent), AuDLNPair(p->dentry));
			err = -EIO;
		}
	}

	AuDbgDentry(p->dentry);
	AuDbgDentry(h_dentry);
	AuDbgDentry(p->parent);
	AuDbgInode(p->h_dir);
	if (h_dentry)
		AuDbgDentry(h_dentry->d_parent);

	au_do_unpin(p, gp);
	if (unlikely(do_gp))
		gp->dentry = NULL;

 out:
	AuTraceErr(err);
	return err;
}

void au_pin_init(struct au_pin *args, struct dentry *dentry, int di_locked,
		 int lsc_di, int lsc_hi, int do_gp)
{
	struct au_pin1 *p;
	unsigned char do_verify;

	AuTraceEnter();

	memset(args, 0, sizeof(*args));
	p = args->pin + AuPin_PARENT;
	p->dentry = dentry;
	p->di_locked = di_locked;
	p->lsc_di = lsc_di;
	p->lsc_hi = lsc_hi;
	p->do_verify = !au_opt_test(au_mntflags(dentry->d_sb), UDBA_NONE);
	if (!do_gp)
		return;

	do_verify = p->do_verify;
	p = au_pin_gp(args);
	if (unlikely(p)) {
		p->lsc_di = lsc_di + 1; /* child first */
		p->lsc_hi = lsc_hi - 1; /* parent first */
		p->do_verify = do_verify;
	}
}

int au_pin(struct au_pin *args, struct dentry *dentry, aufs_bindex_t bindex,
	   int di_locked, int do_gp)
{
	LKTRTrace("%.*s, b%d, di_locked %d, do_gp %d\n",
		  AuDLNPair(dentry), bindex, di_locked, do_gp);

	au_pin_init(args, dentry, di_locked, AuLsc_DI_PARENT, AuLsc_I_PARENT2,
		    do_gp);
	return au_do_pin(args->pin + AuPin_PARENT, au_pin_gp(args), bindex, do_gp);
}

/* ---------------------------------------------------------------------- */

struct au_icpup_args {
	aufs_bindex_t btgt;
	unsigned char isdir, hinotify, did_cpup; /* flags */
	struct dentry *h_dentry;
	struct inode *h_inode;
	struct au_pin pin;
	struct au_hin_ignore ign[2];
	struct vfsub_args vargs;
};

/* todo: refine it */
static int au_lock_and_icpup(struct dentry *dentry, loff_t sz,
			     struct au_icpup_args *a)
{
	int err;
	aufs_bindex_t bstart;
	struct super_block *sb;
	struct dentry *hi_wh, *parent;
	struct inode *inode;
	struct au_wr_dir_args wr_dir_args = {
		.force_btgt	= -1,
		.flags		= 0
	};

	LKTRTrace("%.*s, %lld\n", AuDLNPair(dentry), sz);

	di_write_lock_child(dentry);
	bstart = au_dbstart(dentry);
	sb = dentry->d_sb;
	inode = dentry->d_inode;
	a->isdir = !!S_ISDIR(inode->i_mode);
	if (unlikely(a->isdir))
		au_fset_wrdir(wr_dir_args.flags, ISDIR);
	/* plink or hi_wh() */
	if (bstart != au_ibstart(inode))
		wr_dir_args.force_btgt = au_ibstart(inode);
	err = au_wr_dir(dentry, /*src_dentry*/NULL, &wr_dir_args);
	if (unlikely(err < 0))
		goto out_dentry;
	a->btgt = err;
	a->did_cpup = (err != bstart);
	err = 0;

	/* crazy udba locks */
	a->hinotify = !!au_opt_test(au_mntflags(sb), UDBA_INOTIFY);
	parent = NULL;
	if (!IS_ROOT(dentry)) {
		parent = dget_parent(dentry);
		di_write_lock_parent(parent);
	}
	err = au_pin(&a->pin, dentry, a->btgt, /*di_locked*/!!parent,
		     /*dp_gp*/a->hinotify);
	if (unlikely(err)) {
		if (parent) {
			di_write_unlock(parent);
			dput(parent);
		}
		goto out_dentry;
	}
	a->h_dentry = au_h_dptr(dentry, bstart);
	a->h_inode = a->h_dentry->d_inode;
	AuDebugOn(!a->h_inode);
	mutex_lock_nested(&a->h_inode->i_mutex, AuLsc_I_CHILD);
	if (!a->did_cpup) {
		au_unpin_gp(&a->pin);
		if (parent) {
			au_pin_set_parent_lflag(&a->pin, /*lflag*/0);
			di_downgrade_lock(parent, AuLock_IR);
			dput(parent);
		}
		goto out; /* success */
	}

	hi_wh = NULL;
	if (!d_unhashed(dentry)) {
		if (parent) {
			au_pin_set_parent_lflag(&a->pin, /*lflag*/0);
			di_downgrade_lock(parent, AuLock_IR);
			dput(parent);
		}
		err = au_sio_cpup_simple(dentry, a->btgt, sz, AuCpup_DTIME);
		if (!err)
			a->h_dentry = au_h_dptr(dentry, a->btgt);
	} else {
		hi_wh = au_hi_wh(inode, a->btgt);
		if (!hi_wh) {
			err = au_sio_cpup_wh(dentry, a->btgt, sz,
					     /*file*/NULL);
			if (!err)
				hi_wh = au_hi_wh(inode, a->btgt);
			/* todo: revalidate hi_wh? */
		}
		if (parent) {
			au_pin_set_parent_lflag(&a->pin, /*lflag*/0);
			di_downgrade_lock(parent, AuLock_IR);
			dput(parent);
		}
		if (!hi_wh)
			a->h_dentry = au_h_dptr(dentry, a->btgt);
		else
			a->h_dentry = hi_wh; /* do not dget here */
	}

	mutex_unlock(&a->h_inode->i_mutex);
	a->h_inode = a->h_dentry->d_inode;
	AuDebugOn(!a->h_inode);
	if (!err) {
		mutex_lock_nested(&a->h_inode->i_mutex, AuLsc_I_CHILD);
		au_unpin_gp(&a->pin);
		goto out; /* success */
	}

	au_unpin(&a->pin);

 out_dentry:
	di_write_unlock(dentry);
 out:
	AuTraceErr(err);
	return err;
}

static int aufs_setattr(struct dentry *dentry, struct iattr *ia)
{
	int err;
	struct inode *inode;
	struct super_block *sb;
	__u32 events;
	struct file *file;
	loff_t sz;
	struct au_icpup_args *a;

	LKTRTrace("%.*s\n", AuDLNPair(dentry));
	inode = dentry->d_inode;
	IMustLock(inode);

	err = -ENOMEM;
	a = kzalloc(sizeof(*a), GFP_NOFS);
	if (unlikely(!a))
		goto out;

	file = NULL;
	sb = dentry->d_sb;
	si_read_lock(sb, AuLock_FLUSH);
	vfsub_args_init(&a->vargs, a->ign, au_test_dlgt(au_mntflags(sb)), 0);

	if (ia->ia_valid & ATTR_FILE) {
		/* currently ftruncate(2) only */
		file = ia->ia_file;
		fi_write_lock(file);
		ia->ia_file = au_h_fptr(file, au_fbstart(file));
	}

	sz = -1;
	if ((ia->ia_valid & ATTR_SIZE)
	    && ia->ia_size < i_size_read(inode))
		sz = ia->ia_size;
	err = au_lock_and_icpup(dentry, sz, a);
	if (unlikely(err < 0))
		goto out_si;
	if (a->did_cpup) {
		ia->ia_file = NULL;
		ia->ia_valid &= ~ATTR_FILE;
	}

	if ((ia->ia_valid & ATTR_SIZE)
	    && ia->ia_size < i_size_read(inode)) {
		err = vmtruncate(inode, ia->ia_size);
		if (unlikely(err))
			goto out_unlock;
	}

	if (ia->ia_valid & (ATTR_KILL_SUID | ATTR_KILL_SGID))
		ia->ia_valid &= ~ATTR_MODE;

	events = 0;
	if (unlikely(a->hinotify)) {
		events = vfsub_events_notify_change(ia);
		if (events) {
			if (unlikely(a->isdir))
				vfsub_ign_hinode(&a->vargs, events,
						 au_hi(inode, a->btgt));
			vfsub_ign_hinode(&a->vargs, events,
					 au_pinned_hdir(&a->pin, a->btgt));
		}
	}
	err = vfsub_notify_change(a->h_dentry, ia, &a->vargs);
	if (!err)
		au_cpup_attr_changeable(inode);

 out_unlock:
	mutex_unlock(&a->h_inode->i_mutex);
	au_unpin(&a->pin);
	di_write_unlock(dentry);
 out_si:
	if (file) {
		fi_write_unlock(file);
		ia->ia_file = file;
		ia->ia_valid |= ATTR_FILE;
	}
	si_read_unlock(sb);
	kfree(a);
 out:
	AuTraceErr(err);
	return err;
}

/* ---------------------------------------------------------------------- */

static int h_readlink(struct dentry *dentry, int bindex, char __user *buf,
		      int bufsiz)
{
	struct super_block *sb;
	struct dentry *h_dentry;

	LKTRTrace("%.*s, b%d, %d\n", AuDLNPair(dentry), bindex, bufsiz);

	h_dentry = au_h_dptr(dentry, bindex);
	if (unlikely(/* !h_dentry
			|| !h_dentry->d_inode
			|| */
		    !h_dentry->d_inode->i_op
		    || !h_dentry->d_inode->i_op->readlink))
		return -EINVAL;

	sb = dentry->d_sb;
	if (!au_test_ro(sb, bindex, dentry->d_inode)) {
		touch_atime(au_sbr_mnt(sb, bindex), h_dentry);
		au_update_fuse_h_inode(NULL, h_dentry); /*ignore*/
		fsstack_copy_attr_atime(dentry->d_inode, h_dentry->d_inode);
	}
	return h_dentry->d_inode->i_op->readlink(h_dentry, buf, bufsiz);
}

static int aufs_readlink(struct dentry *dentry, char __user *buf, int bufsiz)
{
	int err;

	LKTRTrace("%.*s, %d\n", AuDLNPair(dentry), bufsiz);

	aufs_read_lock(dentry, AuLock_IR);
	err = h_readlink(dentry, au_dbstart(dentry), buf, bufsiz);
	aufs_read_unlock(dentry, AuLock_IR);
	AuTraceErr(err);
	return err;
}

static void *aufs_follow_link(struct dentry *dentry, struct nameidata *nd)
{
	int err;
	char *buf;
	mm_segment_t old_fs;

	LKTRTrace("%.*s, nd %.*s\n",
		  AuDLNPair(dentry), AuDLNPair(nd->path.dentry));

	err = -ENOMEM;
	buf = __getname();
	if (unlikely(!buf))
		goto out;

	aufs_read_lock(dentry, AuLock_IR);
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	err = h_readlink(dentry, au_dbstart(dentry), (char __user *)buf,
			 PATH_MAX);
	set_fs(old_fs);
	aufs_read_unlock(dentry, AuLock_IR);

	if (err >= 0) {
		buf[err] = 0;
		/* will be freed by put_link */
		nd_set_link(nd, buf);
		return NULL; /* success */
	}
	__putname(buf);

 out:
	path_put(&nd->path);
	AuTraceErr(err);
	return ERR_PTR(err);
}

static void aufs_put_link(struct dentry *dentry, struct nameidata *nd,
			  void *cookie)
{
	LKTRTrace("%.*s\n", AuDLNPair(dentry));
	__putname(nd_get_link(nd));
}

/* ---------------------------------------------------------------------- */

static void aufs_truncate_range(struct inode *inode, loff_t start, loff_t end)
{
	AuUnsupport();
}

/* ---------------------------------------------------------------------- */

struct inode_operations aufs_symlink_iop = {
	.permission	= aufs_permission,
	.setattr	= aufs_setattr,
#ifdef CONFIG_AUFS_HIN_OR_FUSE
	.getattr	= aufs_getattr,
#endif

	.readlink	= aufs_readlink,
	.follow_link	= aufs_follow_link,
	.put_link	= aufs_put_link
};

struct inode_operations aufs_dir_iop = {
	.create		= aufs_create,
	.lookup		= aufs_lookup,
	.link		= aufs_link,
	.unlink		= aufs_unlink,
	.symlink	= aufs_symlink,
	.mkdir		= aufs_mkdir,
	.rmdir		= aufs_rmdir,
	.mknod		= aufs_mknod,
	.rename		= aufs_rename,

	.permission	= aufs_permission,
	.setattr	= aufs_setattr,
#ifdef CONFIG_AUFS_HIN_OR_FUSE
	.getattr	= aufs_getattr,
#endif

#if 0 /* reserved for future use */
	.setxattr	= aufs_setxattr,
	.getxattr	= aufs_getxattr,
	.listxattr	= aufs_listxattr,
	.removexattr	= aufs_removexattr
#endif
};

struct inode_operations aufs_iop = {
	.permission	= aufs_permission,
	.setattr	= aufs_setattr,
#ifdef CONFIG_AUFS_HIN_OR_FUSE
	.getattr	= aufs_getattr,
#endif

#if 0 /* reserved for future use */
	.setxattr	= aufs_setxattr,
	.getxattr	= aufs_getxattr,
	.listxattr	= aufs_listxattr,
	.removexattr	= aufs_removexattr,
#endif

	.truncate_range	= aufs_truncate_range
};
