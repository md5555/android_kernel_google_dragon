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
 * $Id: vfsub.h,v 1.11 2008/08/25 01:51:04 sfjro Exp $
 */

#ifndef __AUFS_VFSUB_H__
#define __AUFS_VFSUB_H__

#ifdef __KERNEL__

#include <linux/fs.h>
#include <linux/fs_stack.h>
#include <linux/inotify.h>
#include <linux/namei.h>
#include <linux/security.h>
#include <linux/splice.h>

/* ---------------------------------------------------------------------- */

/* vfsub flags */
#define Vfsub_DLGT		1		/* operation with delegation */
#define Vfsub_FORCE_UNLINK	(1 << 1)	/* force unlinking */
#define vfsub_ftest(flags, name)	((flags) & Vfsub_##name)
#define vfsub_fset(flags, name)		{ (flags) |= Vfsub_##name; }
#define vfsub_fclr(flags, name)		{ (flags) &= ~Vfsub_##name; }
#ifndef CONFIG_AUFS_DLGT
#undef Vfsub_DLGT
#define Vfsub_DLGT	0
#endif

struct au_hin_ignore;
struct vfsub_args {
#ifdef CONFIG_AUFS_HINOTIFY
	/* inotify events to be ignored */
	int			nignore;
	struct au_hin_ignore	*ignore;
#endif

	unsigned int		flags;
};

struct au_hinode;
#ifdef CONFIG_AUFS_HINOTIFY
static inline
void do_vfsub_args_reinit(struct vfsub_args *vargs, struct au_hin_ignore *ign)
{
	vargs->nignore = 0;
	vargs->ignore = ign;
}

static inline void vfsub_args_reinit(struct vfsub_args *vargs)
{
	vargs->nignore = 0;
}

__u32 vfsub_events_notify_change(struct iattr *ia);
void vfsub_ign_hinode(struct vfsub_args *vargs, __u32 events,
		      struct au_hinode *hinode);
void vfsub_ignore(struct vfsub_args *vargs);
void vfsub_unignore(struct vfsub_args *vargs);
#else
static inline
void do_vfsub_args_reinit(struct vfsub_args *vargs, struct au_hin_ignore *ign)
{
	/* empty */
}

static inline void vfsub_args_reinit(struct vfsub_args *vargs)
{
	/* empty */
}

static inline __u32 vfsub_events_notify_change(struct iattr *ia)
{
	return 0;
}

static inline void vfsub_ign_hinode(struct vfsub_args *vargs, __u32 events,
				    struct au_hinode *hinode)
{
	/* empty */
}

static inline void vfsub_ignore(struct vfsub_args *vargs)
{
	/* empty */
}

static inline void vfsub_unignore(struct vfsub_args *vargs)
{
	/* empty */
}
#endif /* CONFIG_AUFS_HINOTIFY */

void vfsub_args_init(struct vfsub_args *vargs, struct au_hin_ignore *ign,
		     int dlgt, int force_unlink);

/* ---------------------------------------------------------------------- */

/* inotify_inode_watched() is not exported */
static inline int au_test_inotify(struct inode *inode)
{
#ifdef CONFIG_INOTIFY
	return !list_empty(&inode->inotify_watches);
#endif
	return 0;
}

/* ---------------------------------------------------------------------- */

/* lock subclass for hidden inode */
/* default MAX_LOCKDEP_SUBCLASSES(8) is not enough */
/* reduce? gave up. */
enum {
	AuLsc_I_Begin = I_MUTEX_QUOTA, /* 4 */
	AuLsc_I_PARENT,		/* hidden inode, parent first */
	AuLsc_I_PARENT2,	/* copyup dirs */
	AuLsc_I_PARENT3,	/* rename with hinotify */
	AuLsc_I_PARENT4,	/* ditto */
	AuLsc_I_CHILD,
	AuLsc_I_CHILD2,
	AuLsc_I_End
};

#define IMustLock(i)	MtxMustLock(&(i)->i_mutex)

static inline void vfsub_lock_rename_mutex(struct super_block *sb)
{
	lockdep_off();
	mutex_lock(&sb->s_vfs_rename_mutex);
	lockdep_on();
}

static inline void vfsub_unlock_rename_mutex(struct super_block *sb)
{
	lockdep_off();
	mutex_unlock(&sb->s_vfs_rename_mutex);
	lockdep_on();
}

static inline
struct dentry *vfsub_lock_rename(struct dentry *d1, struct dentry *d2)
{
	struct dentry *d;

	lockdep_off();
	d = lock_rename(d1, d2);
	lockdep_on();
	return d;
}

static inline void vfsub_unlock_rename(struct dentry *d1, struct dentry *d2)
{
	lockdep_off();
	unlock_rename(d1, d2);
	lockdep_on();
}

static inline int au_verify_parent(struct dentry *dentry, struct inode *dir)
{
	IMustLock(dir);
	return (/* !dir->i_nlink || */ dentry->d_parent->d_inode != dir);
}

/* ---------------------------------------------------------------------- */

#ifdef CONFIG_AUFS_WORKAROUND_FUSE
/* br_fuse.c */
int au_update_fuse_h_inode(struct vfsmount *h_mnt, struct dentry *h_dentry);
#else
static inline
int au_update_fuse_h_inode(struct vfsmount *h_mnt, struct dentry *h_dentry)
{
	return 0;
}
#endif

#ifdef CONFIG_AUFS_BR_XFS
/* br_xfs.c */
dev_t au_h_rdev(struct inode *h_inode, struct vfsmount *h_mnt,
		struct dentry *h_dentry);
#else
static inline
dev_t au_h_rdev(struct inode *h_inode, struct vfsmount *h_mnt,
		struct dentry *h_dentry)
{
	return h_inode->i_rdev;
}
#endif

/* simple abstractions, for future use */
static inline
int do_vfsub_permission(struct inode *inode, int mask, struct nameidata *nd)
{
	LKTRTrace("i%lu, mask 0x%x, nd %d\n", inode->i_ino, mask, !!nd);
	IMustLock(inode);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)
	return inode_permission(inode, mask);
#else
	return permission(inode, mask, nd);
#endif
}

static inline
int vfsub_security_inode_permission(struct inode *inode, int mask,
				    struct nameidata *nd)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)
	return security_inode_permission(inode, mask);
#else
	return security_inode_permission(inode, mask, nd);
#endif
}

/* ---------------------------------------------------------------------- */

struct file *vfsub_filp_open(const char *path, int oflags, int mode);
int vfsub_path_lookup(const char *name, unsigned int flags,
		      struct nameidata *nd);
struct dentry *vfsub_lookup_one_len(const char *name, struct dentry *parent,
				    int len);

#ifdef CONFIG_AUFS_LHASH_PATCH
struct dentry *vfsub__lookup_hash(struct qstr *name, struct dentry *parent,
				  struct nameidata *nd);
#endif

/* ---------------------------------------------------------------------- */

int do_vfsub_create(struct inode *dir, struct dentry *dentry, int mode,
		    struct nameidata *nd);
int do_vfsub_symlink(struct inode *dir, struct dentry *dentry,
		     const char *symname, int mode);
int do_vfsub_mknod(struct inode *dir, struct dentry *dentry, int mode,
		   dev_t dev);
int do_vfsub_link(struct dentry *src_dentry, struct inode *dir,
		  struct dentry *dentry);
int do_vfsub_rename(struct inode *src_dir, struct dentry *src_dentry,
		    struct inode *dir, struct dentry *dentry);
int do_vfsub_mkdir(struct inode *dir, struct dentry *dentry, int mode);
int do_vfsub_rmdir(struct inode *dir, struct dentry *dentry);
int do_vfsub_unlink(struct inode *dir, struct dentry *dentry);

/* ---------------------------------------------------------------------- */

ssize_t do_vfsub_read_u(struct file *file, char __user *ubuf, size_t count,
			loff_t *ppos);
/* todo: kernel_read()? */
ssize_t do_vfsub_read_k(struct file *file, void *kbuf, size_t count,
			loff_t *ppos);
ssize_t do_vfsub_write_u(struct file *file, const char __user *ubuf,
			 size_t count, loff_t *ppos);
ssize_t do_vfsub_write_k(struct file *file, void *kbuf, size_t count,
			 loff_t *ppos);
int do_vfsub_readdir(struct file *file, filldir_t filldir, void *arg);

/* ---------------------------------------------------------------------- */

#ifndef CONFIG_AUFS_UNIONFS22_PATCH
static inline void vfsub_copy_inode_size(struct inode *inode,
					 struct inode *h_inode)
{
	spin_lock(&inode->i_lock);
	fsstack_copy_inode_size(inode, h_inode);
	spin_unlock(&inode->i_lock);
}
#else
static inline void vfsub_copy_inode_size(struct inode *inode,
					 struct inode *h_inode)
{
	fsstack_copy_inode_size(inode, h_inode);
}
#endif

#ifndef CONFIG_AUFS_UNIONFS23_PATCH
#define vfs_splice_to		do_splice_to
#define vfs_splice_from		do_splice_from
#endif

#ifdef CONFIG_AUFS_SPLICE_PATCH
long do_vfsub_splice_to(struct file *in, loff_t *ppos,
			struct pipe_inode_info *pipe, size_t len,
			unsigned int flags);
long do_vfsub_splice_from(struct pipe_inode_info *pipe, struct file *out,
			  loff_t *ppos, size_t len, unsigned int flags);
#else
static inline
long do_vfsub_splice_to(struct file *in, loff_t *ppos,
			struct pipe_inode_info *pipe, size_t len,
			unsigned int flags)
{
	return -ENOSYS;
}

static inline
long do_vfsub_splice_from(struct pipe_inode_info *pipe, struct file *out,
			  loff_t *ppos, size_t len, unsigned int flags)
{
	return -ENOSYS;
}
#endif /* CONFIG_AUFS_SPLICE_PATCH */

/* ---------------------------------------------------------------------- */

static inline loff_t vfsub_llseek(struct file *file, loff_t offset, int origin)
{
	loff_t err;

	LKTRTrace("%.*s\n", AuDLNPair(file->f_dentry));

	lockdep_off();
	err = vfs_llseek(file, offset, origin);
	lockdep_on();
	return err;
}

static inline int do_vfsub_getattr(struct vfsmount *mnt, struct dentry *dentry,
				   struct kstat *st)
{
	LKTRTrace("%.*s\n", AuDLNPair(dentry));
	return vfs_getattr(mnt, dentry, st);
}

/* ---------------------------------------------------------------------- */

#ifdef CONFIG_AUFS_HIN_OR_DLGT
/* hin_or_dlgt.c */
int vfsub_permission(struct inode *inode, int mask, struct nameidata *nd,
		     int dlgt);

int vfsub_create(struct inode *dir, struct dentry *dentry, int mode,
		 struct nameidata *nd, struct vfsub_args *vargs);
int vfsub_symlink(struct inode *dir, struct dentry *dentry, const char *symname,
		  int mode, struct vfsub_args *vargs);
int vfsub_mknod(struct inode *dir, struct dentry *dentry, int mode, dev_t dev,
		struct vfsub_args *vargs);
int vfsub_link(struct dentry *src_dentry, struct inode *dir,
	       struct dentry *dentry, struct vfsub_args *vargs);
int vfsub_rename(struct inode *src_dir, struct dentry *src_dentry,
		 struct inode *dir, struct dentry *dentry,
		 struct vfsub_args *vargs);
int vfsub_mkdir(struct inode *dir, struct dentry *dentry, int mode,
		struct vfsub_args *vargs);
int vfsub_rmdir(struct inode *dir, struct dentry *dentry,
		struct vfsub_args *vargs);

ssize_t vfsub_read_u(struct file *file, char __user *ubuf, size_t count,
		     loff_t *ppos, int dlgt);
ssize_t vfsub_read_k(struct file *file, void *kbuf, size_t count, loff_t *ppos,
		     int dlgt);
ssize_t vfsub_write_u(struct file *file, const char __user *ubuf, size_t count,
		      loff_t *ppos, struct vfsub_args *vargs);
ssize_t vfsub_write_k(struct file *file, void *kbuf, size_t count, loff_t *ppos,
		      struct vfsub_args *vargs);
int vfsub_readdir(struct file *file, filldir_t filldir, void *arg, int dlgt);
long vfsub_splice_to(struct file *in, loff_t *ppos,
		     struct pipe_inode_info *pipe, size_t len,
		     unsigned int flags, int dlgt);
long vfsub_splice_from(struct pipe_inode_info *pipe, struct file *out,
		       loff_t *ppos, size_t len, unsigned int flags,
		       struct vfsub_args *vargs);

int vfsub_getattr(struct vfsmount *mnt, struct dentry *dentry, struct kstat *st,
		  int dlgt);
#else

static inline
int vfsub_permission(struct inode *inode, int mask, struct nameidata *nd,
		     int dlgt)
{
	return do_vfsub_permission(inode, mask, nd);
}

static inline
int vfsub_create(struct inode *dir, struct dentry *dentry, int mode,
		 struct nameidata *nd, struct vfsub_args *vargs)
{
	return do_vfsub_create(dir, dentry, mode, nd);
}

static inline
int vfsub_symlink(struct inode *dir, struct dentry *dentry, const char *symname,
		  int mode, struct vfsub_args *vargs)
{
	return do_vfsub_symlink(dir, dentry, symname, mode);
}

static inline
int vfsub_mknod(struct inode *dir, struct dentry *dentry, int mode, dev_t dev,
		struct vfsub_args *vargs)
{
	return do_vfsub_mknod(dir, dentry, mode, dev);
}

static inline
int vfsub_link(struct dentry *src_dentry, struct inode *dir,
	       struct dentry *dentry, struct vfsub_args *vargs)
{
	return do_vfsub_link(src_dentry, dir, dentry);
}

static inline
int vfsub_rename(struct inode *src_dir, struct dentry *src_dentry,
		 struct inode *dir, struct dentry *dentry,
		 struct vfsub_args *vargs)
{
	return do_vfsub_rename(src_dir, src_dentry, dir, dentry);
}

static inline
int vfsub_mkdir(struct inode *dir, struct dentry *dentry, int mode,
		struct vfsub_args *vargs)
{
	return do_vfsub_mkdir(dir, dentry, mode);
}

static inline
int vfsub_rmdir(struct inode *dir, struct dentry *dentry,
		struct vfsub_args *vargs)
{
	return do_vfsub_rmdir(dir, dentry);
}

static inline
ssize_t vfsub_read_u(struct file *file, char __user *ubuf, size_t count,
		     loff_t *ppos, int dlgt)
{
	return do_vfsub_read_u(file, ubuf, count, ppos);
}

static inline
ssize_t vfsub_read_k(struct file *file, void *kbuf, size_t count, loff_t *ppos,
		     int dlgt)
{
	return do_vfsub_read_k(file, kbuf, count, ppos);
}

static inline
ssize_t vfsub_write_u(struct file *file, const char __user *ubuf, size_t count,
		      loff_t *ppos, struct vfsub_args *vargs)
{
	return do_vfsub_write_u(file, ubuf, count, ppos);
}

static inline
ssize_t vfsub_write_k(struct file *file, void *kbuf, size_t count, loff_t *ppos,
		      struct vfsub_args *vargs)
{
	return do_vfsub_write_k(file, kbuf, count, ppos);
}

static inline
int vfsub_readdir(struct file *file, filldir_t filldir, void *arg, int dlgt)
{
	return do_vfsub_readdir(file, filldir, arg);
}

static inline
long vfsub_splice_to(struct file *in, loff_t *ppos,
		     struct pipe_inode_info *pipe, size_t len,
		     unsigned int flags, int dlgt)
{
	return do_vfsub_splice_to(in, ppos, pipe, len, flags);
}

static inline
long vfsub_splice_from(struct pipe_inode_info *pipe, struct file *out,
		       loff_t *ppos, size_t len, unsigned int flags,
		       struct vfsub_args *vargs)
{
	return do_vfsub_splice_from(pipe, out, ppos, len, flags);
}

static inline
int vfsub_getattr(struct vfsmount *mnt, struct dentry *dentry, struct kstat *st,
		  int dlgt)
{
	return do_vfsub_getattr(mnt, dentry, st);
}
#endif /* HIN_OR_DLGT */

/* ---------------------------------------------------------------------- */

int vfsub_sio_mkdir(struct au_hinode *hdir, struct dentry *dentry, int mode,
		    int dlgt);
int vfsub_sio_rmdir(struct au_hinode *hdir, struct dentry *dentry, int dlgt);
int vfsub_sio_notify_change(struct au_hinode *hdir, struct dentry *dentry,
			    struct iattr *ia);

/* ---------------------------------------------------------------------- */

int vfsub_notify_change(struct dentry *dentry, struct iattr *ia,
			struct vfsub_args *vargs);
int vfsub_unlink(struct inode *dir, struct dentry *dentry,
		 struct vfsub_args *vargs);
int vfsub_statfs(void *arg, struct kstatfs *buf, int dlgt);

#endif /* __KERNEL__ */
#endif /* __AUFS_VFSUB_H__ */
