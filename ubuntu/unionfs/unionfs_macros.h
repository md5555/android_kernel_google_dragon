/*
 * Copyright (c) 2003-2006 Erez Zadok
 * Copyright (c) 2003-2006 Charles P. Wright
 * Copyright (c) 2005-2006 Josef Sipek
 * Copyright (c) 2005      Arun M. Krishnakumar
 * Copyright (c) 2005-2006 David P. Quigley
 * Copyright (c) 2003-2004 Mohammad Nayyer Zubair
 * Copyright (c) 2003      Puja Gupta
 * Copyright (c) 2003      Harikesavan Krishnan
 * Copyright (c) 2003-2006 Stony Brook University
 * Copyright (c) 2003-2006 The Research Foundation of State University of New York
 *
 * For specific licensing information, see the COPYING file distributed with
 * this package.
 *
 * This Copyright notice must be kept intact and distributed with all sources.
 */
/*
 *  $Id: unionfs_macros.h,v 1.13 2006/06/01 03:11:03 jsipek Exp $
 */

#ifndef __UNIONFS_H_
#error This file should only be included from unionfs.h!
#endif

/* Inode to private data */
static inline struct unionfs_inode_info *itopd(const struct inode *inode)
{
	return
	    &(container_of(inode, struct unionfs_inode_container, vfs_inode)->
	      info);
}

#define itohi_ptr(ino) (itopd(ino)->uii_inode)
#define ibstart(ino) (itopd(ino)->b_start)
#define ibend(ino) (itopd(ino)->b_end)

/* Superblock to private data */
#define stopd(super) ((struct unionfs_sb_info *)(super)->s_fs_info)
#define stopd_lhs(super) ((super)->s_fs_info)
#define sbstart(sb) 0
#define sbend(sb) stopd(sb)->b_end
#define sbmax(sb) (stopd(sb)->b_end + 1)

/* File to private Data */
#define ftopd(file) ((struct unionfs_file_info *)((file)->private_data))
#define ftopd_lhs(file) ((file)->private_data)
#define ftohf_ptr(file)  (ftopd(file)->ufi_file)
#define fbstart(file) (ftopd(file)->b_start)
#define fbend(file) (ftopd(file)->b_end)

/* File to hidden file. */
static inline struct file *ftohf(struct file *f)
{
	return ftopd(f)->ufi_file[fbstart(f)];
}

static inline struct file *ftohf_index(const struct file *f, int index)
{
	return ftopd(f)->ufi_file[index];
}

static inline void set_ftohf_index(struct file *f, int index, struct file *val)
{
	ftopd(f)->ufi_file[index] = val;
}

static inline void set_ftohf(struct file *f, struct file *val)
{
	ftopd(f)->ufi_file[fbstart(f)] = val;
}

/* Inode to hidden inode. */
static inline struct inode *itohi(const struct inode *i)
{
	return itopd(i)->uii_inode[ibstart(i)];
}

static inline struct inode *itohi_index(const struct inode *i, int index)
{
	return itopd(i)->uii_inode[index];
}

static inline void set_itohi_index(struct inode *i, int index,
				   struct inode *val)
{
	itopd(i)->uii_inode[index] = val;
}

static inline void set_itohi(struct inode *i, struct inode *val)
{
	itopd(i)->uii_inode[ibstart(i)] = val;
}

/* Superblock to hidden superblock. */
static inline struct super_block *stohs(const struct super_block *o)
{
	return stopd(o)->usi_data[sbstart(o)].sb;
}

static inline struct super_block *stohs_index(const struct super_block *o, int index)
{
	return stopd(o)->usi_data[index].sb;
}

static inline void set_stohs_index(struct super_block *o, int index,
				   struct super_block *val)
{
	stopd(o)->usi_data[index].sb = val;
}

static inline void set_stohs(struct super_block *o, struct super_block *val)
{
	stopd(o)->usi_data[sbstart(o)].sb = val;
}

/* Super to hidden mount. */
static inline struct vfsmount *stohiddenmnt_index(struct super_block *o,
						  int index)
{
	return stopd(o)->usi_data[index].hidden_mnt;
}

static inline void set_stohiddenmnt_index(struct super_block *o, int index,
					  struct vfsmount *val)
{
	stopd(o)->usi_data[index].hidden_mnt = val;
}

/* Branch count macros. */
static inline int branch_count(struct super_block *o, int index)
{
	return atomic_read(&stopd(o)->usi_data[index].sbcount);
}

static inline void set_branch_count(struct super_block *o, int index, int val)
{
	atomic_set(&stopd(o)->usi_data[index].sbcount, val);
}

static inline void branchget(struct super_block *o, int index)
{
	atomic_inc(&stopd(o)->usi_data[index].sbcount);
}

static inline void branchput(struct super_block *o, int index)
{
	atomic_dec(&stopd(o)->usi_data[index].sbcount);
}

/* Dentry macros */
static inline struct unionfs_dentry_info *dtopd(const struct dentry *dent)
{
	return (struct unionfs_dentry_info *)dent->d_fsdata;
}

#define dtopd_lhs(dent) ((dent)->d_fsdata)
#define dtopd_nocheck(dent) dtopd(dent)
#define dbstart(dent) (dtopd(dent)->udi_bstart)
#define set_dbstart(dent, val) do { dtopd(dent)->udi_bstart = val; } while(0)
#define dbend(dent) (dtopd(dent)->udi_bend)
#define set_dbend(dent, val) do { dtopd(dent)->udi_bend = val; } while(0)
#define dbopaque(dent) (dtopd(dent)->udi_bopaque)
#define set_dbopaque(dent, val) do { dtopd(dent)->udi_bopaque = val; } while (0)

static inline void set_dtohd_index(struct dentry *dent, int index,
				   struct dentry *val)
{
	dtopd(dent)->udi_dentry[index] = val;
}

static inline struct dentry *dtohd_index(const struct dentry *dent, int index)
{
	return dtopd(dent)->udi_dentry[index];
}

static inline struct dentry *dtohd(const struct dentry *dent)
{
	return dtopd(dent)->udi_dentry[dbstart(dent)];
}

#define set_dtohd_index_nocheck(dent, index, val) set_dtohd_index(dent, index, val)
#define dtohd_index_nocheck(dent, index) dtohd_index(dent, index)

#define dtohd_ptr(dent) (dtopd_nocheck(dent)->udi_dentry)

/* Macros for locking a dentry. */
#define lock_dentry(d) down(&dtopd(d)->udi_sem)
#define unlock_dentry(d) up(&dtopd(d)->udi_sem)
#define verify_locked(d)

/*
 *
 * vim:shiftwidth=8
 * vim:tabstop=8
 *
 * For Emacs:
 * Local variables:
 * c-basic-offset: 8
 * c-comment-only-line-offset: 0
 * c-offsets-alist: ((statement-block-intro . +) (knr-argdecl-intro . 0)
 *              (substatement-open . 0) (label . 0) (statement-cont . +))
 * indent-tabs-mode: t
 * tab-width: 8
 * End:
 */
