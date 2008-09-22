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
 * file operations
 *
 * $Id: file.h,v 1.5 2008/06/30 03:53:43 sfjro Exp $
 */

#ifndef __AUFS_FILE_H__
#define __AUFS_FILE_H__

#ifdef __KERNEL__

#include <linux/file.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/aufs_type.h>
#include "dentry.h"
#include "misc.h"
#include "super.h"

/* ---------------------------------------------------------------------- */

struct au_branch;
struct au_hfile {
	struct file		*hf_file;
	struct au_branch	*hf_br;
};

struct au_vdir;
struct au_finfo {
	atomic_t		fi_generation;

	struct au_rwsem		fi_rwsem;
	struct au_hfile		*fi_hfile;
	aufs_bindex_t		fi_bstart, fi_bend;

	union {
		struct vm_operations_struct	*fi_h_vm_ops;
		struct au_vdir			*fi_vdir_cache;
	};
};

/* ---------------------------------------------------------------------- */

/* file.c */
extern struct address_space_operations aufs_aop;
unsigned int au_file_roflags(unsigned int flags);
struct file *au_h_open(struct dentry *dentry, aufs_bindex_t bindex, int flags,
		       struct file *file);
int au_do_open(struct inode *inode, struct file *file,
	       int (*open)(struct file *file, int flags));
int au_reopen_nondir(struct file *file);
struct au_pin;
int au_ready_to_write(struct file *file, loff_t len, struct au_pin *pin);
int au_reval_and_lock_fdi(struct file *file, int (*reopen)(struct file *file),
			  int wlock, int locked);

/* f_op.c */
extern struct file_operations aufs_file_fop;
int aufs_flush(struct file *file, fl_owner_t id);

/* finfo.c */
struct au_finfo *au_fi(struct file *file);
struct au_branch *au_fbr(struct file *file, aufs_bindex_t bindex);
struct file *au_h_fptr(struct file *file, aufs_bindex_t bindex);

void au_hfput(struct au_hfile *hf);
void au_set_h_fptr(struct file *file, aufs_bindex_t bindex,
		   struct file *h_file);

void au_finfo_fin(struct file *file);
int au_finfo_init(struct file *file);

#ifdef CONFIG_AUFS_ROBR
/* robr.c */
struct file *au_robr_safe_file(struct vm_area_struct *vma);
void au_robr_reset_file(struct vm_area_struct *vma, struct file *file);
#else
static inline struct file *au_robr_safe_file(struct vm_area_struct *vma)
{
	struct file *file;

	file = vma->vm_file;
	if (file->private_data && au_test_aufs(file->f_dentry->d_sb))
		return file;
	return NULL;
}

static inline
void au_robr_reset_file(struct vm_area_struct *vma, struct file *file)
{
	vma->vm_file = file;
	/* smp_mb(); */ /* flush vm_file */
}
#endif /* CONFIG_AUFS_ROBR */

/* ---------------------------------------------------------------------- */

/* todo: memory barrier? */
static inline au_gen_t au_figen(struct file *f)
{
	return atomic_read(&au_fi(f)->fi_generation);
}

static inline int au_test_mmapped(struct file *f)
{
	return !!(au_fi(f)->fi_h_vm_ops);
}

static inline int au_test_aufs_file(struct file *f)
{
	return !(f->f_dentry->d_inode->i_mode
		 & (S_IFCHR | S_IFBLK | S_IFIFO | S_IFSOCK));
}

/* ---------------------------------------------------------------------- */

#if !defined(CONFIG_AUFS_MODULE) || defined(CONFIG_AUFS_DENY_WRITE_ACCESS_PATCH)
int au_store_fmode_exec(struct nameidata *nd, struct inode *inode);

static inline int au_deny_write_access(struct file *h_file)
{
	LKTRTrace("%.*s\n", AuDLNPair(h_file->f_dentry));
	return deny_write_access(h_file);
}

static inline void au_allow_write_access(struct file *h_file)
{
	allow_write_access(h_file);
}

#else

static inline int au_store_fmode_exec(struct nameidata *nd, struct inode *inode)
{
	/* nothing */
	return 0;
}

static inline int au_deny_write_access(struct file *h_file)
{
	/* nothing */
	return 0;
}

static inline void au_allow_write_access(struct file *h_file)
{
	/* nothing */
}
#endif /* CONFIG_AUFS_DENY_WRITE_ACCESS_PATCH */

/* ---------------------------------------------------------------------- */

/*
 * fi_read_lock, fi_write_lock,
 * fi_read_unlock, fi_write_unlock, fi_downgrade_lock
 */
AuSimpleRwsemFuncs(fi, struct file *f, au_fi(f)->fi_rwsem);

/* to debug easier, do not make them inlined functions */
#define FiMustReadLock(f) do { \
	SiMustAnyLock((f)->f_dentry->d_sb); \
	AuRwMustReadLock(&au_fi(f)->fi_rwsem); \
} while (0)

#define FiMustWriteLock(f) do { \
	SiMustAnyLock((f)->f_dentry->d_sb); \
	AuRwMustWriteLock(&au_fi(f)->fi_rwsem); \
} while (0)

#define FiMustAnyLock(f) do { \
	SiMustAnyLock((f)->f_dentry->d_sb); \
	AuRwMustAnyLock(&au_fi(f)->fi_rwsem); \
} while (0)

#define FiMustNoWaiters(f)	AuRwMustNoWaiters(&au_fi(f)->fi_rwsem)

/* ---------------------------------------------------------------------- */

/* todo: hard/soft set? */
static inline aufs_bindex_t au_fbstart(struct file *file)
{
	FiMustAnyLock(file);
	return au_fi(file)->fi_bstart;
}

static inline aufs_bindex_t au_fbend(struct file *file)
{
	FiMustAnyLock(file);
	return au_fi(file)->fi_bend;
}

static inline struct au_vdir *au_fvdir_cache(struct file *file)
{
	FiMustAnyLock(file);
	return au_fi(file)->fi_vdir_cache;
}

static inline void au_set_fbstart(struct file *file, aufs_bindex_t bindex)
{
	FiMustWriteLock(file);
	AuDebugOn(au_sbend(file->f_dentry->d_sb) < bindex);
	au_fi(file)->fi_bstart = bindex;
}

static inline void au_set_fbend(struct file *file, aufs_bindex_t bindex)
{
	FiMustWriteLock(file);
	AuDebugOn(au_sbend(file->f_dentry->d_sb) < bindex
		  || bindex < au_fbstart(file));
	au_fi(file)->fi_bend = bindex;
}

static inline void au_set_fvdir_cache(struct file *file,
				      struct au_vdir *vdir_cache)
{
	FiMustWriteLock(file);
	AuDebugOn(!S_ISDIR(file->f_dentry->d_inode->i_mode)
		  || (au_fi(file)->fi_vdir_cache && vdir_cache));
	au_fi(file)->fi_vdir_cache = vdir_cache;
}

static inline void au_update_figen(struct file *file)
{
	atomic_set(&au_fi(file)->fi_generation, au_digen(file->f_dentry));
	/* smp_mb(); */ /* atomic_set */
}

#endif /* __KERNEL__ */
#endif /* __AUFS_FILE_H__ */
