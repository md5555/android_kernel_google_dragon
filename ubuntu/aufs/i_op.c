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
 * $Id: i_op.c,v 1.8 2008/06/09 01:10:26 sfjro Exp $
 */

#include <linux/fs_stack.h>
#include <linux/uaccess.h>
#include "aufs.h"

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
		err = h_inode->i_op->permission(h_inode, submask);
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
		aufs_read_unlock(nd->path.dentry, AuLock_FLUSH | AuLock_IR);
		break;
	default:
		BUG();
	}
}

static int aufs_permission(struct inode *inode, int mask)
{
	int err, locked, dlgt;
	aufs_bindex_t bindex, bend;
	struct inode *h_inode;
	struct super_block *sb;
	unsigned int mnt_flags;
	struct nameidata fake_nd, *p, *nd = NULL;
	const int write_mask = (mask & (MAY_WRITE | MAY_APPEND));
	const int nondir = !S_ISDIR(inode->i_mode);

	LKTRTrace("ino %lu, mask 0x%x, nondir %d, write_mask %d, "
		  "nd %d{%d, %d}\n",
		  inode->i_ino, mask, nondir, write_mask,
		  !!nd, nd ? !!nd->path.dentry : 0, nd ? !!nd->path.mnt : 0);

	sb = inode->i_sb;
	locked = silly_lock(inode, nd);
	mnt_flags = au_mntflags(sb);
	dlgt = !!au_test_dlgt(mnt_flags);

	if (nd)
		fake_nd = *nd;
	if (/* unlikely */(nondir || write_mask
			   || au_test_dirperm1(mnt_flags))) {
		h_inode = au_h_iptr(inode, au_ibstart(inode));
		AuDebugOn(!h_inode
			  || ((h_inode->i_mode & S_IFMT)
			      != (inode->i_mode & S_IFMT)));
		err = 0;
		bindex = au_ibstart(inode);
		p = au_fake_dm(&fake_nd, nd, sb, bindex);
		/* actual test will be delegated to LSM */
		if (IS_ERR(p))
			AuDebugOn(PTR_ERR(p) != -ENOENT);
		else {
			LKTRTrace("b%d\n", bindex);
			err = h_permission(h_inode, mask, p,
					   au_sbr_perm(sb, bindex), dlgt);
			au_fake_dm_release(p);
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
	err = 0;
	bend = au_ibend(inode);
	for (bindex = au_ibstart(inode); !err && bindex <= bend; bindex++) {
		h_inode = au_h_iptr(inode, bindex);
		if (!h_inode)
			continue;
		AuDebugOn(!S_ISDIR(h_inode->i_mode));

		p = au_fake_dm(&fake_nd, nd, sb, bindex);
		/* actual test will be delegated to LSM */
		if (IS_ERR(p))
			AuDebugOn(PTR_ERR(p) != -ENOENT);
		else {
			LKTRTrace("b%d\n", bindex);
			err = h_permission(h_inode, mask, p,
					   au_sbr_perm(sb, bindex), dlgt);
			au_fake_dm_release(p);
		}
	}

 out:
	silly_unlock(locked, inode, nd);
	AuTraceErr(err);
	return err;
}

/* ---------------------------------------------------------------------- */

static struct dentry *aufs_lookup(struct inode *dir, struct dentry *dentry,
				  struct nameidata *nd)
{
	struct dentry *ret, *parent;
	int err, npositive;
	struct inode *inode, *h_inode;
	struct nameidata tmp_nd, *ndp;

	LKTRTrace("dir %lu, %.*s, nd{0x%x}\n",
		  dir->i_ino, AuDLNPair(dentry), nd ? nd->flags : 0);
	AuDebugOn(IS_ROOT(dentry));
	IMustLock(dir);

	/* nd can be NULL */
	parent = dentry->d_parent; /* dir inode is locked */
	aufs_read_lock(parent, AuLock_FLUSH);
	err = au_alloc_dinfo(dentry);
	ret = ERR_PTR(err);
	if (unlikely(err))
		goto out;

	ndp = au_dup_nd(au_sbi(dir->i_sb), &tmp_nd, nd);
	npositive = au_lkup_dentry(dentry, au_dbstart(parent), /*type*/0, ndp);
	err = npositive;
	ret = ERR_PTR(err);
	if (unlikely(err < 0))
		goto out_unlock;
	inode = NULL;
	if (npositive) {
		/*
		 * stop 'race'-ing between hardlinks under different parents.
		 */
		h_inode = au_h_dptr(dentry, au_dbstart(dentry))->d_inode;
		AuDebugOn(!h_inode);
		if (h_inode->i_nlink == 1 || S_ISDIR(h_inode->i_mode))
			inode = au_new_inode(dentry);
		else {
			/* todo: this lock is too large, try br_xino_nondir mutex */
			static DEFINE_MUTEX(mtx);
			mutex_lock(&mtx);
			inode = au_new_inode(dentry);
			mutex_unlock(&mtx);
		}
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
	aufs_read_unlock(parent, !AuLock_IR);
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
	struct dentry *parent, *src_parent;
	struct au_sbinfo *sbinfo;
	const int add_entry = au_ftest_wrdir(args->flags, ADD_ENTRY);
	const int lock_srcdir = au_ftest_wrdir(args->flags, LOCK_SRCDIR);

	LKTRTrace("%.*s, src %p, {%d, 0x%x}\n",
		  AuDLNPair(dentry), src_dentry, args->force_btgt, args->flags);

	src_parent = NULL;
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
	if (src_dentry) {
		src_parent = dget_parent(src_dentry);
		if (lock_srcdir)
			di_write_lock_parent2(src_parent);
	}

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
			err = au_cpdown_dirs(dentry, bcpup, src_parent);
		else
			err = au_cpup_dirs(dentry, bcpup, src_parent);
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
	if (lock_srcdir)
		di_write_unlock(src_parent);
	dput(src_parent);
	if (!err)
		err = bcpup; /* success */
 out:
	dput(parent);
	LKTRTrace("err %d\n", err);
	AuTraceErr(err);
	return err;
}

/* ---------------------------------------------------------------------- */

static void au_hi_lock(struct inode *h_inode, int isdir, struct inode *inode,
		       aufs_bindex_t bindex)
{
	if (!isdir)
		mutex_lock_nested(&h_inode->i_mutex, AuLsc_I_CHILD);
	else
		au_hdir2_lock(h_inode, inode, bindex);
}

static void au_hi_unlock(struct inode *h_inode, int isdir, struct inode *inode,
			 aufs_bindex_t bindex)
{
	if (!isdir)
		mutex_unlock(&h_inode->i_mutex);
	else
		au_hdir_unlock(h_inode, inode, bindex);
}

struct au_icpup_args {
	aufs_bindex_t btgt;
	unsigned char isdir, did_cpup; /* flags */
	unsigned char hinotify;
	struct dentry *parent, *gparent, *h_dentry;
	struct inode *dir, *gdir, *h_inode, *h_dir;
};

static int au_lock_and_icpup(struct dentry *dentry, loff_t sz,
			     struct au_icpup_args *rargs)
{
	int err;
	aufs_bindex_t bstart;
	struct super_block *sb;
	struct dentry *hi_wh;
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
	rargs->isdir = S_ISDIR(inode->i_mode);
	if (rargs->isdir)
		au_fset_wrdir(wr_dir_args.flags, ISDIR);
	err = au_wr_dir(dentry, /*src_dentry*/NULL, &wr_dir_args);
	if (unlikely(err < 0))
		goto out_dentry;
	rargs->btgt = err;
	rargs->did_cpup = (err != bstart);
	err = 0;

	/* crazy udba locks */
	rargs->hinotify = !!au_opt_test(au_mntflags(sb), UDBA_INOTIFY);
	if (unlikely(!IS_ROOT(dentry))) {
		rargs->parent = dget_parent(dentry);
		rargs->dir = rargs->parent->d_inode;
		di_read_lock_parent(rargs->parent, AuLock_IR);
	}
	rargs->h_dentry = au_h_dptr(dentry, au_dbstart(dentry));
	rargs->h_inode = rargs->h_dentry->d_inode;
	AuDebugOn(!rargs->h_inode);

	if (!rargs->did_cpup) {
		au_hi_lock(rargs->h_inode, rargs->isdir, inode, rargs->btgt);
		/* todo: revalidate the lower dentry? */
		goto out; /* success */
	}

	if (unlikely(rargs->hinotify
		     && rargs->parent
		     && !IS_ROOT(rargs->parent))) {
		rargs->gparent = dget_parent(rargs->parent);
		rargs->gdir = rargs->gparent->d_inode;
		ii_read_lock_parent2(rargs->gdir);
	}

	hi_wh = NULL;
	rargs->h_dir = au_h_iptr(rargs->dir, rargs->btgt);
	au_hdir_lock(rargs->h_dir, rargs->dir, rargs->btgt);
	/* todo: revalidate the lower dentry? */
	au_hi_lock(rargs->h_inode, rargs->isdir, inode, bstart);
	if (!d_unhashed(dentry)) {
		err = au_sio_cpup_simple(dentry, rargs->btgt, sz, AuCpup_DTIME);
		if (!err)
			rargs->h_dentry = au_h_dptr(dentry, au_dbstart(dentry));
	} else {
		hi_wh = au_hi_wh(inode, rargs->btgt);
		if (!hi_wh) {
			err = au_sio_cpup_wh(dentry, rargs->btgt, sz,
					     /*file*/NULL);
			if (!err)
				hi_wh = au_hi_wh(inode, rargs->btgt);
			/* todo: revalidate hi_wh? */
		}
		if (!hi_wh)
			rargs->h_dentry = au_h_dptr(dentry, au_dbstart(dentry));
		else
			rargs->h_dentry = hi_wh; /* do not dget here */
	}

	au_hi_unlock(rargs->h_inode, rargs->isdir, inode, bstart);
	rargs->h_inode = rargs->h_dentry->d_inode;
	AuDebugOn(!rargs->h_inode);
	if (!err)
		au_hi_lock(rargs->h_inode, rargs->isdir, inode, rargs->btgt);
	au_hdir_unlock(rargs->h_dir, rargs->dir, rargs->btgt);
	if (!err)
		goto out; /* success */

	au_hi_unlock(rargs->h_inode, rargs->isdir, inode, rargs->btgt);
	if (unlikely(rargs->gdir)) {
		ii_read_unlock(rargs->gdir);
		dput(rargs->gparent);
	}
	if (unlikely(rargs->dir)) {
		di_read_unlock(rargs->parent, AuLock_IR);
		dput(rargs->parent);
	}

 out_dentry:
	di_write_unlock(dentry);
 out:
	AuTraceErr(err);
	return err;
}

static int aufs_do_setattr(struct dentry *dentry, struct iattr *ia,
			   struct file *file)
{
	int err;
	struct inode *inode;
	struct au_hin_ignore ign;
	struct vfsub_args vargs;
	struct super_block *sb;
	__u32 events;
	loff_t sz;
	struct au_icpup_args rargs;

	LKTRTrace("%.*s, ia_valid 0x%x\n", AuDLNPair(dentry), ia->ia_valid);
	inode = dentry->d_inode;
	IMustLock(inode);

	sb = dentry->d_sb;
	si_read_lock(sb, AuLock_FLUSH);

	if (file)
		fi_write_lock(file);

	sz = -1;
	if ((ia->ia_valid & ATTR_SIZE)
	    && ia->ia_size < i_size_read(inode))
		sz = ia->ia_size;
	memset(&rargs, 0, sizeof(rargs));
	err = au_lock_and_icpup(dentry, sz, &rargs);
	if (unlikely(err < 0))
		goto out;

	if ((ia->ia_valid & ATTR_SIZE)
	    && ia->ia_size < i_size_read(inode)) {
		err = vmtruncate(inode, ia->ia_size);
		if (unlikely(err))
			goto out_unlock;
	}

	if (ia->ia_valid & (ATTR_KILL_SUID | ATTR_KILL_SGID))
		ia->ia_valid &= ~ATTR_MODE;

	events = 0;
	vfsub_args_init(&vargs, &ign, au_test_dlgt(au_mntflags(sb)), 0);
	if (unlikely(rargs.hinotify && rargs.dir)) {
		events = vfsub_events_notify_change(ia);
		if (events)
			vfsub_ign_hinode(&vargs, events,
					 au_hi(rargs.dir, rargs.btgt));
	}
	err = vfsub_notify_change(rargs.h_dentry, ia, &vargs);
	if (!err)
		au_cpup_attr_changeable(inode);

 out_unlock:
	au_hi_unlock(rargs.h_inode, rargs.isdir, inode, rargs.btgt);
	if (unlikely(rargs.gdir)) {
		ii_read_unlock(rargs.gdir);
		dput(rargs.gparent);
	}
	if (unlikely(rargs.dir)) {
		di_read_unlock(rargs.parent, AuLock_IR);
		dput(rargs.parent);
	}
	di_write_unlock(dentry);
 out:
	if (file)
		fi_write_unlock(file);

	si_read_unlock(sb);
	AuTraceErr(err);
	return err;
}

static int aufs_setattr(struct dentry *dentry, struct iattr *ia)
{
	return aufs_do_setattr(dentry, ia, NULL);
}

int aufs_fsetattr(struct file *file, struct iattr *ia)
{
	return aufs_do_setattr(file->f_dentry, ia, file);
}

/* ---------------------------------------------------------------------- */

static int h_readlink(struct dentry *dentry, int bindex, char __user *buf,
		      int bufsiz)
{
	struct super_block *sb;
	struct dentry *h_dentry;

	LKTRTrace("%.*s, b%d, %d\n", AuDLNPair(dentry), bindex, bufsiz);

	h_dentry = au_h_dptr(dentry, bindex);
	if (unlikely(!h_dentry->d_inode->i_op
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
#ifdef CONFIG_AUFS_WORKAROUND_FUSE
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
#ifdef CONFIG_AUFS_WORKAROUND_FUSE
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
#ifdef CONFIG_AUFS_WORKAROUND_FUSE
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
