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
 * sub-routines for VFS
 *
 * $Id: vfsub.c,v 1.11 2008/08/04 00:32:35 sfjro Exp $
 */

#include <linux/uaccess.h>
#include "aufs.h"

/* ---------------------------------------------------------------------- */

void vfsub_args_init(struct vfsub_args *vargs, struct au_hin_ignore *ign,
		     int dlgt, int force_unlink)
{
	do_vfsub_args_reinit(vargs, ign);
	vargs->flags = 0;
	if (unlikely(dlgt))
		vfsub_fset(vargs->flags, DLGT);
	if (force_unlink)
		vfsub_fset(vargs->flags, FORCE_UNLINK);
}

/* ---------------------------------------------------------------------- */

struct file *vfsub_filp_open(const char *path, int oflags, int mode)
{
	struct file *err;

	LKTRTrace("%s\n", path);

	lockdep_off();
	err = filp_open(path, oflags, mode);
	lockdep_on();
	if (!IS_ERR(err))
		au_update_fuse_h_inode(err->f_vfsmnt, err->f_dentry); /*ignore*/
	return err;
}

int vfsub_path_lookup(const char *name, unsigned int flags,
		      struct nameidata *nd)
{
	int err;

	LKTRTrace("%s\n", name);

	/* lockdep_off(); */
	err = path_lookup(name, flags, nd);
	/* lockdep_on(); */
	if (!err)
		au_update_fuse_h_inode(nd->path.mnt, nd->path.dentry);
	/*ignore*/
	return err;
}

struct dentry *vfsub_lookup_one_len(const char *name, struct dentry *parent,
				    int len)
{
	struct dentry *d;

	LKTRTrace("%.*s/%.*s\n", AuDLNPair(parent), len, name);
	IMustLock(parent->d_inode);

	d = lookup_one_len(name, parent, len);
	if (!IS_ERR(d))
		au_update_fuse_h_inode(NULL, d); /*ignore*/
	return d;
}

#ifdef CONFIG_AUFS_LHASH_PATCH
struct dentry *vfsub__lookup_hash(struct qstr *name, struct dentry *parent,
				  struct nameidata *nd)
{
	struct dentry *d;

	LKTRTrace("%.*s/%.*s, nd %d\n",
		  AuDLNPair(parent), AuLNPair(name), !!nd);
	if (nd)
		LKTRTrace("nd{0x%x}\n", nd->flags);
	IMustLock(parent->d_inode);

	d = __lookup_hash(name, parent, nd);
	if (!IS_ERR(d))
		au_update_fuse_h_inode(NULL, d); /*ignore*/
	return d;
}
#endif

/* ---------------------------------------------------------------------- */

int do_vfsub_create(struct inode *dir, struct dentry *dentry, int mode,
		    struct nameidata *nd)
{
	int err;
	struct vfsmount *mnt;

	LKTRTrace("i%lu, %.*s, 0x%x\n", dir->i_ino, AuDLNPair(dentry), mode);
	IMustLock(dir);

	err = vfs_create(dir, dentry, mode, nd);
	if (!err) {
		mnt = NULL;
		if (nd)
			mnt = nd->path.mnt;
		/* dir inode is locked */
		au_update_fuse_h_inode(mnt, dentry->d_parent); /*ignore*/
		au_update_fuse_h_inode(mnt, dentry); /*ignore*/
	}
	return err;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)
#define VfsubSymlinkArgs	dir, dentry, symname
#else
#define VfsubSymlinkArgs	dir, dentry, symname, mode
#endif

int do_vfsub_symlink(struct inode *dir, struct dentry *dentry,
		     const char *symname, int mode)
{
	int err;

	LKTRTrace("i%lu, %.*s, %s, 0x%x\n",
		  dir->i_ino, AuDLNPair(dentry), symname, mode);
	IMustLock(dir);

	err = vfs_symlink(VfsubSymlinkArgs);
	if (!err) {
		/* dir inode is locked */
		au_update_fuse_h_inode(NULL, dentry->d_parent); /*ignore*/
		au_update_fuse_h_inode(NULL, dentry); /*ignore*/
	}
	return err;
}

int do_vfsub_mknod(struct inode *dir, struct dentry *dentry, int mode,
		   dev_t dev)
{
	int err;

	LKTRTrace("i%lu, %.*s, 0x%x\n", dir->i_ino, AuDLNPair(dentry), mode);
	IMustLock(dir);

	err = vfs_mknod(dir, dentry, mode, dev);
	if (!err) {
		/* dir inode is locked */
		au_update_fuse_h_inode(NULL, dentry->d_parent); /*ignore*/
		au_update_fuse_h_inode(NULL, dentry); /*ignore*/
	}
	return err;
}

int do_vfsub_link(struct dentry *src_dentry, struct inode *dir,
		  struct dentry *dentry)
{
	int err;

	LKTRTrace("%.*s, i%lu, %.*s\n",
		  AuDLNPair(src_dentry), dir->i_ino, AuDLNPair(dentry));
	IMustLock(dir);

	lockdep_off();
	err = vfs_link(src_dentry, dir, dentry);
	lockdep_on();
	if (!err) {
		LKTRTrace("src_i %p, dst_i %p\n",
			  src_dentry->d_inode, dentry->d_inode);
		/* fuse has different memory inode for the same inumber */
		au_update_fuse_h_inode(NULL, src_dentry); /*ignore*/
		/* dir inode is locked */
		au_update_fuse_h_inode(NULL, dentry->d_parent); /*ignore*/
		au_update_fuse_h_inode(NULL, dentry); /*ignore*/
	}
	return err;
}

int do_vfsub_rename(struct inode *src_dir, struct dentry *src_dentry,
		    struct inode *dir, struct dentry *dentry)
{
	int err;

	LKTRTrace("i%lu, %.*s, i%lu, %.*s\n",
		  src_dir->i_ino, AuDLNPair(src_dentry),
		  dir->i_ino, AuDLNPair(dentry));
	IMustLock(dir);
	IMustLock(src_dir);

	lockdep_off();
	err = vfs_rename(src_dir, src_dentry, dir, dentry);
	lockdep_on();
	if (!err) {
		/* dir inode is locked */
		au_update_fuse_h_inode(NULL, dentry->d_parent); /*ignore*/
		au_update_fuse_h_inode(NULL, src_dentry->d_parent); /*ignore*/
		au_update_fuse_h_inode(NULL, src_dentry); /*ignore*/
	}
	return err;
}

int do_vfsub_mkdir(struct inode *dir, struct dentry *dentry, int mode)
{
	int err;

	LKTRTrace("i%lu, %.*s, 0x%x\n", dir->i_ino, AuDLNPair(dentry), mode);
	IMustLock(dir);

	err = vfs_mkdir(dir, dentry, mode);
	if (!err) {
		/* dir inode is locked */
		au_update_fuse_h_inode(NULL, dentry->d_parent); /*ignore*/
		au_update_fuse_h_inode(NULL, dentry); /*ignore*/
	}
	return err;
}

int do_vfsub_rmdir(struct inode *dir, struct dentry *dentry)
{
	int err;

	LKTRTrace("i%lu, %.*s\n", dir->i_ino, AuDLNPair(dentry));
	IMustLock(dir);

	lockdep_off();
	err = vfs_rmdir(dir, dentry);
	lockdep_on();
	/* dir inode is locked */
	if (!err)
		au_update_fuse_h_inode(NULL, dentry->d_parent); /*ignore*/
	return err;
}

int do_vfsub_unlink(struct inode *dir, struct dentry *dentry)
{
	int err;

	LKTRTrace("i%lu, %.*s\n", dir->i_ino, AuDLNPair(dentry));
	IMustLock(dir);

	/* vfs_unlink() locks inode */
	lockdep_off();
	err = vfs_unlink(dir, dentry);
	lockdep_on();
	/* dir inode is locked */
	if (!err)
		au_update_fuse_h_inode(NULL, dentry->d_parent); /*ignore*/
	return err;
}

/* ---------------------------------------------------------------------- */

ssize_t do_vfsub_read_u(struct file *file, char __user *ubuf, size_t count,
			loff_t *ppos)
{
	ssize_t err;

	LKTRTrace("%.*s, cnt %lu, pos %lld\n",
		  AuDLNPair(file->f_dentry), (unsigned long)count, *ppos);

	/* todo: always off, regardless nfs branch? */
	au_br_nfs_lockdep_off(file->f_vfsmnt->mnt_sb);
	err = vfs_read(file, ubuf, count, ppos);
	au_br_nfs_lockdep_on(file->f_vfsmnt->mnt_sb);
	if (err >= 0)
		au_update_fuse_h_inode(file->f_vfsmnt, file->f_dentry);
	/*ignore*/
	return err;
}

/* todo: kernel_read()? */
ssize_t do_vfsub_read_k(struct file *file, void *kbuf, size_t count,
			loff_t *ppos)
{
	ssize_t err;
	mm_segment_t oldfs;

	oldfs = get_fs();
	set_fs(KERNEL_DS);
	err = do_vfsub_read_u(file, (char __user *)kbuf, count, ppos);
	set_fs(oldfs);
	return err;
}

ssize_t do_vfsub_write_u(struct file *file, const char __user *ubuf,
			 size_t count, loff_t *ppos)
{
	ssize_t err;

	LKTRTrace("%.*s, cnt %lu, pos %lld\n",
		  AuDLNPair(file->f_dentry), (unsigned long)count, *ppos);

	lockdep_off();
	err = vfs_write(file, ubuf, count, ppos);
	lockdep_on();
	if (err >= 0)
		au_update_fuse_h_inode(file->f_vfsmnt, file->f_dentry);
	/*ignore*/
	return err;
}

ssize_t do_vfsub_write_k(struct file *file, void *kbuf, size_t count,
			 loff_t *ppos)
{
	ssize_t err;
	mm_segment_t oldfs;

	oldfs = get_fs();
	set_fs(KERNEL_DS);
	err = do_vfsub_write_u(file, (const char __user *)kbuf, count, ppos);
	set_fs(oldfs);
	return err;
}

int do_vfsub_readdir(struct file *file, filldir_t filldir, void *arg)
{
	int err;

	LKTRTrace("%.*s\n", AuDLNPair(file->f_dentry));

	lockdep_off();
	err = vfs_readdir(file, filldir, arg);
	lockdep_on();
	if (err >= 0)
		au_update_fuse_h_inode(file->f_vfsmnt, file->f_dentry);
	/*ignore*/
	return err;
}

#ifdef CONFIG_AUFS_SPLICE_PATCH
long do_vfsub_splice_to(struct file *in, loff_t *ppos,
			struct pipe_inode_info *pipe, size_t len,
			unsigned int flags)
{
	long err;

	LKTRTrace("%.*s, pos %lld, len %lu, 0x%x\n",
		  AuDLNPair(in->f_dentry), *ppos, (unsigned long)len, flags);

	lockdep_off();
	err = vfs_splice_to(in, ppos, pipe, len, flags);
	lockdep_on();
	if (err >= 0)
		au_update_fuse_h_inode(in->f_vfsmnt, in->f_dentry); /*ignore*/
	return err;
}

long do_vfsub_splice_from(struct pipe_inode_info *pipe, struct file *out,
			  loff_t *ppos, size_t len, unsigned int flags)
{
	long err;

	LKTRTrace("%.*s, pos %lld, len %lu, 0x%x\n",
		  AuDLNPair(out->f_dentry), *ppos, (unsigned long)len, flags);

	lockdep_off();
	err = vfs_splice_from(pipe, out, ppos, len, flags);
	lockdep_on();
	if (err >= 0)
		au_update_fuse_h_inode(out->f_vfsmnt, out->f_dentry); /*ignore*/
	return err;
}
#endif

/* ---------------------------------------------------------------------- */

struct au_vfsub_mkdir_args {
	int *errp;
	struct inode *dir;
	struct dentry *dentry;
	int mode;
	struct vfsub_args *vargs;
};

static void au_call_vfsub_mkdir(void *args)
{
	struct au_vfsub_mkdir_args *a = args;
	*a->errp = vfsub_mkdir(a->dir, a->dentry, a->mode, a->vargs);
}

int vfsub_sio_mkdir(struct au_hinode *hdir, struct dentry *dentry, int mode,
		    int dlgt)
{
	int err, do_sio, wkq_err;
	struct inode *dir = hdir->hi_inode;
	struct au_hin_ignore ign;
	struct vfsub_args vargs;

	LKTRTrace("i%lu, %.*s\n", dir->i_ino, AuDLNPair(dentry));

	vfsub_args_init(&vargs, &ign, dlgt, /*force_unlink*/0);
	vfsub_ign_hinode(&vargs, IN_CREATE, hdir);
	do_sio = au_test_h_perm_sio(dir, MAY_EXEC | MAY_WRITE, dlgt);
	if (!do_sio)
		err = vfsub_mkdir(dir, dentry, mode, &vargs);
	else {
		struct au_vfsub_mkdir_args args = {
			.errp	= &err,
			.dir	= dir,
			.dentry	= dentry,
			.mode	= mode,
			.vargs	= &vargs
		};
		vfsub_fclr(vargs.flags, DLGT);
		wkq_err = au_wkq_wait(au_call_vfsub_mkdir, &args, /*dlgt*/0);
		if (unlikely(wkq_err))
			err = wkq_err;
	}

	AuTraceErr(err);
	return err;
}

struct au_vfsub_rmdir_args {
	int *errp;
	struct inode *dir;
	struct dentry *dentry;
	struct vfsub_args *vargs;
};

static void au_call_vfsub_rmdir(void *args)
{
	struct au_vfsub_rmdir_args *a = args;
	*a->errp = vfsub_rmdir(a->dir, a->dentry, a->vargs);
}

int vfsub_sio_rmdir(struct au_hinode *hdir, struct dentry *dentry, int dlgt)
{
	int err, do_sio, wkq_err;
	struct inode *dir = hdir->hi_inode;
	struct au_hin_ignore ign;
	struct vfsub_args vargs;

	LKTRTrace("i%lu, %.*s\n", dir->i_ino, AuDLNPair(dentry));

	vfsub_args_init(&vargs, &ign, dlgt, /*force_unlink*/0);
	vfsub_ign_hinode(&vargs, IN_DELETE, hdir);
	do_sio = au_test_h_perm_sio(dir, MAY_EXEC | MAY_WRITE, dlgt);
	if (!do_sio)
		err = vfsub_rmdir(dir, dentry, &vargs);
	else {
		struct au_vfsub_rmdir_args args = {
			.errp		= &err,
			.dir		= dir,
			.dentry		= dentry,
			.vargs		= &vargs
		};
		vfsub_fclr(vargs.flags, DLGT);
		wkq_err = au_wkq_wait(au_call_vfsub_rmdir, &args, /*dlgt*/0);
		if (unlikely(wkq_err))
			err = wkq_err;
	}

	AuTraceErr(err);
	return err;
}

/* ---------------------------------------------------------------------- */

struct notify_change_args {
	int *errp;
	struct dentry *h_dentry;
	struct iattr *ia;
	struct vfsub_args *vargs;
};

static void call_notify_change(void *args)
{
	struct notify_change_args *a = args;
	struct inode *h_inode;

	LKTRTrace("%.*s, ia_valid 0x%x\n",
		  AuDLNPair(a->h_dentry), a->ia->ia_valid);
	h_inode = a->h_dentry->d_inode;
	IMustLock(h_inode);

	*a->errp = -EPERM;
	if (!IS_IMMUTABLE(h_inode) && !IS_APPEND(h_inode)) {
		vfsub_ignore(a->vargs);
		lockdep_off();
		*a->errp = notify_change(a->h_dentry, a->ia);
		lockdep_on();
		if (!*a->errp)
			au_update_fuse_h_inode(NULL, a->h_dentry); /*ignore*/
		else
			vfsub_unignore(a->vargs);
		au_dbg_hin_list(a->vargs);
	}
	AuTraceErr(*a->errp);
}

#ifdef CONFIG_AUFS_DLGT
static void vfsub_notify_change_dlgt(struct notify_change_args *args,
				     unsigned int flags)
{
	if (!vfsub_ftest(flags, DLGT))
		call_notify_change(args);
	else {
		int wkq_err;
		wkq_err = au_wkq_wait(call_notify_change, args, /*dlgt*/1);
		if (unlikely(wkq_err))
			*args->errp = wkq_err;
	}
}
#else
static void vfsub_notify_change_dlgt(struct notify_change_args *args,
				     unsigned int flags)
{
	call_notify_change(args);
}
#endif

int vfsub_notify_change(struct dentry *dentry, struct iattr *ia,
			struct vfsub_args *vargs)
{
	int err;
	struct notify_change_args args = {
		.errp		= &err,
		.h_dentry	= dentry,
		.ia		= ia,
		.vargs		= vargs
	};

	vfsub_notify_change_dlgt(&args, vargs->flags);

	AuTraceErr(err);
	return err;
}

int vfsub_sio_notify_change(struct au_hinode *hdir, struct dentry *dentry,
			    struct iattr *ia)
{
	int err, wkq_err;
	struct au_hin_ignore ign;
	struct vfsub_args vargs;
	__u32 events;
	struct notify_change_args args = {
		.errp		= &err,
		.h_dentry	= dentry,
		.ia		= ia,
		.vargs		= &vargs
	};

	LKTRTrace("%.*s, 0x%x\n", AuDLNPair(dentry), ia->ia_valid);

	vfsub_args_init(&vargs, &ign, /*dlgt*/0, /*force_unlink*/0);
	events = vfsub_events_notify_change(ia);
	if (events)
		vfsub_ign_hinode(&vargs, events, hdir);
	wkq_err = au_wkq_wait(call_notify_change, &args, /*dlgt*/0);
	if (unlikely(wkq_err))
		err = wkq_err;

	AuTraceErr(err);
	return err;
}

/* ---------------------------------------------------------------------- */

struct unlink_args {
	int *errp;
	struct inode *dir;
	struct dentry *dentry;
	struct vfsub_args *vargs;
};

static void call_unlink(void *args)
{
	struct unlink_args *a = args;
	struct inode *h_inode;
	const int stop_sillyrename = (au_test_nfs(a->dentry->d_sb)
				      && atomic_read(&a->dentry->d_count) == 1);

	LKTRTrace("%.*s, stop_silly %d, cnt %d\n",
		  AuDLNPair(a->dentry), stop_sillyrename,
		  atomic_read(&a->dentry->d_count));

	if (!stop_sillyrename)
		dget(a->dentry);
	h_inode = a->dentry->d_inode;
	if (h_inode)
		atomic_inc_return(&h_inode->i_count);
	vfsub_ignore(a->vargs);
	*a->errp = do_vfsub_unlink(a->dir, a->dentry);
	if (unlikely(*a->errp || (a->dentry->d_flags & DCACHE_NFSFS_RENAMED)))
		vfsub_unignore(a->vargs);
	au_dbg_hin_list(a->vargs);
	if (!stop_sillyrename)
		dput(a->dentry);
	if (h_inode)
		iput(h_inode);

	AuTraceErr(*a->errp);
}

/*
 * @dir: must be locked.
 * @dentry: target dentry.
 */
int vfsub_unlink(struct inode *dir, struct dentry *dentry,
		 struct vfsub_args *vargs)
{
	int err;
	struct unlink_args args = {
		.errp	= &err,
		.dir	= dir,
		.dentry	= dentry,
		.vargs	= vargs
	};

	if (!vfsub_ftest(vargs->flags, DLGT)
	    && !vfsub_ftest(vargs->flags, FORCE_UNLINK))
		call_unlink(&args);
	else {
		int wkq_err;
		wkq_err = au_wkq_wait(call_unlink, &args,
				      vfsub_ftest(vargs->flags, DLGT));
		if (unlikely(wkq_err))
			err = wkq_err;
	}

	return err;
}

/* ---------------------------------------------------------------------- */

struct statfs_args {
	int *errp;
	void *arg;
	struct kstatfs *buf;
};

static void call_statfs(void *args)
{
	struct statfs_args *a = args;
	*a->errp = vfs_statfs(a->arg, a->buf);
}

#ifdef CONFIG_AUFS_DLGT
static void vfsub_statfs_dlgt(struct statfs_args *args, int dlgt)
{
	if (!dlgt)
		call_statfs(args);
	else {
		int wkq_err;
		wkq_err = au_wkq_wait(call_statfs, args, /*dlgt*/1);
		if (unlikely(wkq_err))
			*args->errp = wkq_err;
	}
}
#else
static void vfsub_statfs_dlgt(struct statfs_args *args, int dlgt)
{
	call_statfs(args);
}
#endif

int vfsub_statfs(void *arg, struct kstatfs *buf, int dlgt)
{
	int err;
	struct statfs_args args = {
		.errp	= &err,
		.arg	= arg,
		.buf	= buf
	};

	vfsub_statfs_dlgt(&args, dlgt);

	return err;
}
