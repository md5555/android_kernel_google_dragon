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
 * sub-routines for vfs in hinotify or dlgt mode
 *
 * $Id: hin_or_dlgt.c,v 1.6 2008/07/14 00:15:10 sfjro Exp $
 */

#include <linux/uaccess.h>
#include "aufs.h"

#if !defined(CONFIG_AUFS_HINOTIFY) && !defined(CONFIG_AUFS_DLGT)
#error mis-configuraion or Makefile
#endif

/* ---------------------------------------------------------------------- */

struct permission_args {
	int *errp;
	struct inode *inode;
	int mask;
	struct nameidata *nd;
};

static void call_permission(void *args)
{
	struct permission_args *a = args;
	*a->errp = do_vfsub_permission(a->inode, a->mask, a->nd);
}

int vfsub_permission(struct inode *inode, int mask, struct nameidata *nd,
		     int dlgt)
{
	if (!dlgt)
		return do_vfsub_permission(inode, mask, nd);
	else {
		int err, wkq_err;
		struct permission_args args = {
			.errp	= &err,
			.inode	= inode,
			.mask	= mask,
			.nd	= nd
		};
		wkq_err = au_wkq_wait(call_permission, &args, /*dlgt*/1);
		if (unlikely(wkq_err))
			err = wkq_err;
		return err;
	}
}

/* ---------------------------------------------------------------------- */

struct create_args {
	int *errp;
	struct inode *dir;
	struct dentry *dentry;
	int mode;
	struct nameidata *nd;
	struct vfsub_args *vargs;
};

static void call_create(void *args)
{
	struct create_args *a = args;
	vfsub_ignore(a->vargs);
	*a->errp = do_vfsub_create(a->dir, a->dentry, a->mode, a->nd);
	if (unlikely(*a->errp))
		vfsub_unignore(a->vargs);
	au_dbg_hin_list(a->vargs);
}

int vfsub_create(struct inode *dir, struct dentry *dentry, int mode,
		 struct nameidata *nd, struct vfsub_args *vargs)
{
	int err;
	struct create_args args = {
		.errp	= &err,
		.dir	= dir,
		.dentry	= dentry,
		.mode	= mode,
		.nd	= nd,
		.vargs	= vargs
	};

	if (!vfsub_ftest(vargs->flags, DLGT))
		call_create(&args);
	else {
		int wkq_err;
		wkq_err = au_wkq_wait(call_create, &args, /*dlgt*/1);
		if (unlikely(wkq_err))
			err = wkq_err;
	}
	return err;
}

struct symlink_args {
	int *errp;
	struct inode *dir;
	struct dentry *dentry;
	const char *symname;
	int mode;
	struct vfsub_args *vargs;
};

static void call_symlink(void *args)
{
	struct symlink_args *a = args;
	vfsub_ignore(a->vargs);
	*a->errp = do_vfsub_symlink(a->dir, a->dentry, a->symname, a->mode);
	if (unlikely(*a->errp))
		vfsub_unignore(a->vargs);
	au_dbg_hin_list(a->vargs);
}

int vfsub_symlink(struct inode *dir, struct dentry *dentry, const char *symname,
		  int mode, struct vfsub_args *vargs)
{
	int err;
	struct symlink_args args = {
		.errp		= &err,
		.dir		= dir,
		.dentry		= dentry,
		.symname	= symname,
		.mode		= mode,
		.vargs		= vargs
	};

	if (!vfsub_ftest(vargs->flags, DLGT))
		call_symlink(&args);
	else {
		int wkq_err;
		wkq_err = au_wkq_wait(call_symlink, &args, /*dlgt*/1);
		if (unlikely(wkq_err))
			err = wkq_err;
	}
	return err;
}

struct mknod_args {
	int *errp;
	struct inode *dir;
	struct dentry *dentry;
	int mode;
	dev_t dev;
	struct vfsub_args *vargs;
};

static void call_mknod(void *args)
{
	struct mknod_args *a = args;
	vfsub_ignore(a->vargs);
	*a->errp = do_vfsub_mknod(a->dir, a->dentry, a->mode, a->dev);
	if (unlikely(*a->errp))
		vfsub_unignore(a->vargs);
	au_dbg_hin_list(a->vargs);
}

int vfsub_mknod(struct inode *dir, struct dentry *dentry, int mode, dev_t dev,
		struct vfsub_args *vargs)
{
	int err;
	struct mknod_args args = {
		.errp	= &err,
		.dir	= dir,
		.dentry	= dentry,
		.mode	= mode,
		.dev	= dev,
		.vargs	= vargs
	};

	if (!vfsub_ftest(vargs->flags, DLGT))
		call_mknod(&args);
	else {
		int wkq_err;
		wkq_err = au_wkq_wait(call_mknod, &args, /*dlgt*/1);
		if (unlikely(wkq_err))
			err = wkq_err;
	}
	return err;
}

struct mkdir_args {
	int *errp;
	struct inode *dir;
	struct dentry *dentry;
	int mode;
	struct vfsub_args *vargs;
};

static void call_mkdir(void *args)
{
	struct mkdir_args *a = args;
	vfsub_ignore(a->vargs);
	*a->errp = do_vfsub_mkdir(a->dir, a->dentry, a->mode);
	if (unlikely(*a->errp))
		vfsub_unignore(a->vargs);
	au_dbg_hin_list(a->vargs);
}

int vfsub_mkdir(struct inode *dir, struct dentry *dentry, int mode,
		struct vfsub_args *vargs)
{
	int err;
	struct mkdir_args args = {
		.errp	= &err,
		.dir	= dir,
		.dentry	= dentry,
		.mode	= mode,
		.vargs	= vargs
	};

	if (!vfsub_ftest(vargs->flags, DLGT))
		call_mkdir(&args);
	else {
		int wkq_err;
		wkq_err = au_wkq_wait(call_mkdir, &args, /*dlgt*/1);
		if (unlikely(wkq_err))
			err = wkq_err;
	}
	return err;
}

/* ---------------------------------------------------------------------- */

struct link_args {
	int *errp;
	struct inode *dir;
	struct dentry *src_dentry, *dentry;
	struct vfsub_args *vargs;
};

static void call_link(void *args)
{
	struct link_args *a = args;
	vfsub_ignore(a->vargs);
	*a->errp = do_vfsub_link(a->src_dentry, a->dir, a->dentry);
	if (unlikely(*a->errp))
		vfsub_unignore(a->vargs);
	au_dbg_hin_list(a->vargs);
}

int vfsub_link(struct dentry *src_dentry, struct inode *dir,
	       struct dentry *dentry, struct vfsub_args *vargs)
{
	int err;
	struct link_args args = {
		.errp		= &err,
		.src_dentry	= src_dentry,
		.dir		= dir,
		.dentry		= dentry,
		.vargs		= vargs
	};

	if (!vfsub_ftest(vargs->flags, DLGT))
		call_link(&args);
	else {
		int wkq_err;
		wkq_err = au_wkq_wait(call_link, &args, /*dlgt*/1);
		if (unlikely(wkq_err))
			err = wkq_err;
	}
	return err;
}

struct rename_args {
	int *errp;
	struct inode *src_dir, *dir;
	struct dentry *src_dentry, *dentry;
	struct vfsub_args *vargs;
};

static void call_rename(void *args)
{
	struct rename_args *a = args;
	vfsub_ignore(a->vargs);
	*a->errp = do_vfsub_rename(a->src_dir, a->src_dentry, a->dir,
				   a->dentry);
	if (unlikely(*a->errp))
		vfsub_unignore(a->vargs);
	au_dbg_hin_list(a->vargs);
}

int vfsub_rename(struct inode *src_dir, struct dentry *src_dentry,
		 struct inode *dir, struct dentry *dentry,
		 struct vfsub_args *vargs)
{
	int err;
	struct rename_args args = {
		.errp		= &err,
		.src_dir	= src_dir,
		.src_dentry	= src_dentry,
		.dir		= dir,
		.dentry		= dentry,
		.vargs		= vargs
	};

	if (!vfsub_ftest(vargs->flags, DLGT))
		call_rename(&args);
	else {
		int wkq_err;
		wkq_err = au_wkq_wait(call_rename, &args, /*dlgt*/1);
		if (unlikely(wkq_err))
			err = wkq_err;
	}
	return err;
}

struct rmdir_args {
	int *errp;
	struct inode *dir;
	struct dentry *dentry;
	struct vfsub_args *vargs;
};

static void call_rmdir(void *args)
{
	struct rmdir_args *a = args;
	vfsub_ignore(a->vargs);
	*a->errp = do_vfsub_rmdir(a->dir, a->dentry);
	if (unlikely(*a->errp))
		vfsub_unignore(a->vargs);
	au_dbg_hin_list(a->vargs);
}

int vfsub_rmdir(struct inode *dir, struct dentry *dentry,
		struct vfsub_args *vargs)
{
	int err;
	struct rmdir_args args = {
		.errp	= &err,
		.dir	= dir,
		.dentry	= dentry,
		.vargs	= vargs
	};

	if (!vfsub_ftest(vargs->flags, DLGT))
		call_rmdir(&args);
	else {
		int wkq_err;
		wkq_err = au_wkq_wait(call_rmdir, &args, /*dlgt*/1);
		if (unlikely(wkq_err))
			err = wkq_err;
	}
	return err;
}

/* ---------------------------------------------------------------------- */

struct read_args {
	ssize_t *errp;
	struct file *file;
	union {
		void *kbuf;
		char __user *ubuf;
	};
	size_t count;
	loff_t *ppos;
};

static void call_read_k(void *args)
{
	struct read_args *a = args;
	LKTRTrace("%.*s, cnt %lu, pos %lld\n",
		  AuDLNPair(a->file->f_dentry), (unsigned long)a->count,
		  *a->ppos);
	*a->errp = do_vfsub_read_k(a->file, a->kbuf, a->count, a->ppos);
}

ssize_t vfsub_read_u(struct file *file, char __user *ubuf, size_t count,
		     loff_t *ppos, int dlgt)
{
	if (!dlgt)
		return do_vfsub_read_u(file, ubuf, count, ppos);
	else {
		int wkq_err;
		ssize_t err, read;
		struct read_args args = {
			.errp	= &err,
			.file	= file,
			.count	= count,
			.ppos	= ppos
		};

		if (unlikely(!count))
			return 0;

		/*
		 * workaround an application bug.
		 * generally, read(2) or write(2) may return the value shorter
		 * than requested. But many applications don't support it,
		 * for example bash.
		 */
		err = -ENOMEM;
		if (args.count > PAGE_SIZE)
			args.count = PAGE_SIZE;
		args.kbuf = kmalloc(args.count, GFP_NOFS);
		if (unlikely(!args.kbuf))
			goto out;

		read = 0;
		do {
			wkq_err = au_wkq_wait(call_read_k, &args, /*dlgt*/1);
			if (unlikely(wkq_err))
				err = wkq_err;
			if (unlikely(err > 0
				     && copy_to_user(ubuf, args.kbuf, err))) {
				err = -EFAULT;
				goto out_free;
			} else if (!err)
				break;
			else if (unlikely(err < 0))
				goto out_free;
			count -= err;
			/* do not read too much because of file i/o pointer */
			if (count < args.count)
				args.count = count;
			ubuf += err;
			read += err;
		} while (count);
		smp_mb(); /* flush ubuf */
		err = read;

	out_free:
		kfree(args.kbuf);
	out:
		return err;
	}
}

ssize_t vfsub_read_k(struct file *file, void *kbuf, size_t count, loff_t *ppos,
		     int dlgt)
{
	if (!dlgt)
		return do_vfsub_read_k(file, kbuf, count, ppos);
	else {
		ssize_t err;
		int wkq_err;
		struct read_args args = {
			.errp	= &err,
			.file	= file,
			.count	= count,
			.ppos	= ppos
		};
		args.kbuf = kbuf;
		wkq_err = au_wkq_wait(call_read_k, &args, /*dlgt*/1);
		if (unlikely(wkq_err))
			err = wkq_err;
		return err;
	}
}

struct write_args {
	ssize_t *errp;
	struct file *file;
	union {
		void *kbuf;
		const char __user *ubuf;
	};
	size_t count;
	loff_t *ppos;
	struct vfsub_args *vargs;
};

static void call_write_k(void *args)
{
	struct write_args *a = args;
	vfsub_ignore(a->vargs);
	*a->errp = do_vfsub_write_k(a->file, a->kbuf, a->count, a->ppos);
	if (unlikely(*a->errp <= 0))
		vfsub_unignore(a->vargs);
	au_dbg_hin_list(a->vargs);
}

ssize_t vfsub_write_u(struct file *file, const char __user *ubuf, size_t count,
		      loff_t *ppos, struct vfsub_args *vargs)
{
	ssize_t err;

	if (!vfsub_ftest(vargs->flags, DLGT)) {
		vfsub_ignore(vargs);
		err = do_vfsub_write_u(file, ubuf, count, ppos);
		if (unlikely(err <= 0))
			vfsub_unignore(vargs);
		au_dbg_hin_list(vargs);
	} else {
		ssize_t written;
		int wkq_err;
		struct write_args args = {
			.errp	= &err,
			.file	= file,
			.count	= count,
			.ppos	= ppos,
			.vargs	= vargs
		};

		if (unlikely(!count))
			return 0;

		/*
		 * workaround an application bug.
		 * generally, read(2) or write(2) may return the value shorter
		 * than requested. But many applications don't support it,
		 * for example bash.
		 */
		err = -ENOMEM;
		if (args.count > PAGE_SIZE)
			args.count = PAGE_SIZE;
		args.kbuf = kmalloc(args.count, GFP_NOFS);
		if (unlikely(!args.kbuf))
			goto out;

		written = 0;
		do {
			if (unlikely(copy_from_user(args.kbuf, ubuf,
						    args.count))) {
				err = -EFAULT;
				goto out_free;
			}

			wkq_err = au_wkq_wait(call_write_k, &args, /*dlgt*/1);
			if (unlikely(wkq_err))
				err = wkq_err;
			if (err > 0) {
				count -= err;
				if (count < args.count)
					args.count = count;
				ubuf += err;
				written += err;
			} else if (!err)
				break;
			else if (unlikely(err < 0))
				goto out_free;
		} while (count);
		err = written;

	out_free:
		kfree(args.kbuf);
	}
 out:
	return err;
}

ssize_t vfsub_write_k(struct file *file, void *kbuf, size_t count, loff_t *ppos,
		      struct vfsub_args *vargs)
{
	ssize_t err;
	struct write_args args = {
		.errp	= &err,
		.file	= file,
		.count	= count,
		.ppos	= ppos,
		.vargs	= vargs
	};

	args.kbuf = kbuf;
	if (!vfsub_ftest(vargs->flags, DLGT))
		call_write_k(&args);
	else {
		int wkq_err;
		wkq_err = au_wkq_wait(call_write_k, &args, /*dlgt*/1);
		if (unlikely(wkq_err))
			err = wkq_err;
	}
	return err;
}

struct readdir_args {
	int *errp;
	struct file *file;
	filldir_t filldir;
	void *arg;
};

static void call_readdir(void *args)
{
	struct readdir_args *a = args;
	*a->errp = do_vfsub_readdir(a->file, a->filldir, a->arg);
}

int vfsub_readdir(struct file *file, filldir_t filldir, void *arg, int dlgt)
{
	if (!dlgt)
		return do_vfsub_readdir(file, filldir, arg);
	else {
		int err, wkq_err;
		struct readdir_args args = {
			.errp		= &err,
			.file		= file,
			.filldir	= filldir,
			.arg		= arg
		};
		wkq_err = au_wkq_wait(call_readdir, &args, /*dlgt*/1);
		if (unlikely(wkq_err))
			err = wkq_err;
		return err;
	}
}

/* ---------------------------------------------------------------------- */

struct splice_to_args {
	long *errp;
	struct file *in;
	loff_t *ppos;
	struct pipe_inode_info *pipe;
	size_t len;
	unsigned int flags;
};

static void call_splice_to(void *args)
{
	struct splice_to_args *a = args;
	*a->errp = do_vfsub_splice_to(a->in, a->ppos, a->pipe, a->len,
				      a->flags);
}

long vfsub_splice_to(struct file *in, loff_t *ppos,
		     struct pipe_inode_info *pipe, size_t len,
		     unsigned int flags, int dlgt)
{
	if (!dlgt)
		return do_vfsub_splice_to(in, ppos, pipe, len, flags);
	else {
		long err;
		int wkq_err;
		struct splice_to_args args = {
			.errp	= &err,
			.in	= in,
			.ppos	= ppos,
			.pipe	= pipe,
			.len	= len,
			.flags	= flags
		};
		wkq_err = au_wkq_wait(call_splice_to, &args, /*dlgt*/1);
		if (unlikely(wkq_err))
			err = wkq_err;
		return err;
	}
}

struct splice_from_args {
	long *errp;
	struct pipe_inode_info *pipe;
	struct file *out;
	loff_t *ppos;
	size_t len;
	unsigned int flags;
	struct vfsub_args *vargs;
};

static void call_splice_from(void *args)
{
	struct splice_from_args *a = args;
	vfsub_ignore(a->vargs);
	*a->errp = do_vfsub_splice_from(a->pipe, a->out, a->ppos, a->len,
					a->flags);
	if (unlikely(*a->errp < 0))
		vfsub_unignore(a->vargs);
	au_dbg_hin_list(a->vargs);
}

long vfsub_splice_from(struct pipe_inode_info *pipe, struct file *out,
		       loff_t *ppos, size_t len, unsigned int flags,
		       struct vfsub_args *vargs)
{
	long err;
	struct splice_from_args args = {
		.errp	= &err,
		.pipe	= pipe,
		.out	= out,
		.ppos	= ppos,
		.len	= len,
		.flags	= flags,
		.vargs	= vargs
	};

	if (!vfsub_ftest(vargs->flags, DLGT))
		call_splice_from(&args);
	else {
		int wkq_err;
		wkq_err = au_wkq_wait(call_splice_from, &args, /*dlgt*/1);
		if (unlikely(wkq_err))
			err = wkq_err;
	}
	return err;
}

/* ---------------------------------------------------------------------- */

struct getattr_args {
	int *errp;
	struct vfsmount *mnt;
	struct dentry *dentry;
	struct kstat *st;
};

static void call_getattr(void *args)
{
	struct getattr_args *a = args;
	*a->errp = do_vfsub_getattr(a->mnt, a->dentry, a->st);
}

int vfsub_getattr(struct vfsmount *mnt, struct dentry *dentry, struct kstat *st,
		  int dlgt)
{
	if (!dlgt)
		return do_vfsub_getattr(mnt, dentry, st);
	else {
		int err, wkq_err;
		struct getattr_args args = {
			.errp	= &err,
			.mnt	= mnt,
			.dentry	= dentry,
			.st	= st
		};
		wkq_err = au_wkq_wait(call_getattr, &args, /*dlgt*/1);
		if (unlikely(wkq_err))
			err = wkq_err;
		return err;
	}
}
