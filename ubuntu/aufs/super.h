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
 * super_block operations
 *
 * $Id: super.h,v 1.8 2008/06/02 02:39:58 sfjro Exp $
 */

#ifndef __AUFS_SUPER_H__
#define __AUFS_SUPER_H__

#ifdef __KERNEL__

#include <linux/fs.h>
#include <linux/cramfs_fs.h>
#include <linux/kobject.h>
#include <linux/magic.h>
#include <linux/aufs_types.h>
#include "misc.h"
#include "wkq.h"

typedef ssize_t (*au_readf_t)(struct file *, char __user *, size_t, loff_t *);
typedef ssize_t (*au_writef_t)(struct file *, const char __user *, size_t,
			       loff_t *);

struct au_wbr_copyup_operations {
	int (*copyup)(struct dentry *dentry);
};

struct au_wbr_create_operations {
	int (*create)(struct dentry *dentry, int isdir);
	int (*init)(struct super_block *sb);
	int (*fin)(struct super_block *sb);
};

struct au_wbr_mfs {
	struct mutex	mfs_lock; /* protect this structure */
	unsigned long	mfs_jiffy;
	unsigned long	mfs_expire;
	aufs_bindex_t	mfs_bindex;

	u64		mfsrr_bytes;
	u64		mfsrr_watermark;
};

/* sbinfo status flags */
/*
 * set true when refresh_dirs() failed at remount time.
 * then try refreshing dirs at access time again.
 * if it is false, refreshing dirs at access time is unnecesary
 */
#define AuSi_FAILED_REFRESH_DIRS	1
#define au_ftest_si(sbinfo, name)	((sbinfo)->au_si_status & AuSi_##name)
#define au_fset_si(sbinfo, name) \
	{ (sbinfo)->au_si_status |= AuSi_##name; }
#define au_fclr_si(sbinfo, name) \
	{ (sbinfo)->au_si_status &= ~AuSi_##name; }

struct au_branch;
struct au_sbinfo {
	/* nowait tasks in the system-wide workqueue */
	struct au_nowait_tasks	si_nowait;

	struct au_rwsem		si_rwsem;

	/* branch management */
	au_gen_t		si_generation;

	/* see above flags */
	unsigned char		au_si_status;

	aufs_bindex_t		si_bend;
	aufs_bindex_t		si_last_br_id;
	struct au_branch	**si_branch;

	/* policy to select a writable branch */
	unsigned char		si_wbr_copyup;
	unsigned char		si_wbr_create;
	struct au_wbr_copyup_operations *si_wbr_copyup_ops;
	struct au_wbr_create_operations *si_wbr_create_ops;

	/* round robin */
	atomic_t		si_wbr_rr_next;

	/* most free space */
	struct au_wbr_mfs	si_wbr_mfs;

	/* mount flags */
	/* include/asm-ia64/siginfo.h defines a macro named si_flags */
	unsigned int		si_mntflags;

	/* external inode number (bitmap and translation table) */
	au_readf_t		si_xread;
	au_writef_t		si_xwrite;
	struct file		*si_xib;
	struct mutex		si_xib_mtx; /* protect xib members */
	unsigned long		*si_xib_buf;
	unsigned long		si_xib_last_pindex;
	int			si_xib_next_bit;
	/* reserved for future use */
	/* unsigned long long	si_xib_limit; */	/* Max xib file size */

	/* readdir cache time, max, in HZ */
	unsigned long		si_rdcache;

	/*
	 * If the number of whiteouts are larger than si_dirwh, leave all of
	 * them after au_whtmp_ren to reduce the cost of rmdir(2).
	 * future fsck.aufs or kernel thread will remove them later.
	 * Otherwise, remove all whiteouts and the dir in rmdir(2).
	 */
	unsigned int		si_dirwh;

	/*
	 * rename(2) a directory with all children.
	 */
	/* reserved for future use */
	/* int			si_rendir; */

	/* pseudo_link list */ /* todo: dirty? */
	spinlock_t		si_plink_lock;
	struct list_head	si_plink;

#if defined(CONFIG_AUFS_EXPORT) \
	&& (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 26) || !defined(MmTree))
	/* dirty, for export, async ops, and sysfs */
	spinlock_t		si_mntcache_lock;
	struct vfsmount		*si_mntcache;	/* no get/put */
#endif

	/*
	 * sysfs and lifetime management.
	 * this is not a small structure and it may be a waste of memory in case
	 * of sysfs is disabled, particulary when many aufs-es are mounted.
	 */
	struct kobject		si_kobj;

#ifdef CONFIG_AUFS_ROBR
	/* locked vma list for mmap() */ /* todo: dirty? */
	spinlock_t		si_lvma_lock;
	struct list_head	si_lvma;
#endif

#ifdef CONFIG_AUFS_EXPORT /* reserved for future use */
	struct path		si_xinodir;
#endif

	/* dirty, necessary for unmounting, sysfs and sysrq */
	struct super_block	*si_sb;
};

/* ---------------------------------------------------------------------- */

/* policy to select one among writable branches */
#define AuWbrCopyup(sbinfo, args...) \
	(sbinfo)->si_wbr_copyup_ops->copyup(args)
#define AuWbrCreate(sbinfo, args...) \
	(sbinfo)->si_wbr_create_ops->create(args)

/* flags for si_read_lock()/aufs_read_lock()/di_read_lock() */
#define AuLock_DW		1		/* write-lock dentry */
#define AuLock_IR		(1 << 1)	/* read-lock inode */
#define AuLock_IW		(1 << 2)	/* write-lock inode */
#define AuLock_FLUSH		(1 << 3)	/* wait for 'nowait' tasks */
#define AuLock_DIR		(1 << 4)	/* target is a dir */
#define au_ftest_lock(flags, name)	((flags) & AuLock_##name)
#define au_fset_lock(flags, name)	{ (flags) |= AuLock_##name; }
#define au_fclr_lock(flags, name)	{ (flags) &= ~AuLock_##name; }

/* ---------------------------------------------------------------------- */

/* super.c */
extern struct file_system_type aufs_fs_type;
struct inode *au_iget_locked(struct super_block *sb, ino_t ino);

/* sbinfo.c */
void au_si_free(struct kobject *kobj);
int au_si_alloc(struct super_block *sb);
struct au_branch *au_sbr(struct super_block *sb, aufs_bindex_t bindex);
au_gen_t au_sigen_inc(struct super_block *sb);
int au_find_bindex(struct super_block *sb, struct au_branch *br);

void aufs_read_lock(struct dentry *dentry, int flags);
void aufs_read_unlock(struct dentry *dentry, int flags);
void aufs_write_lock(struct dentry *dentry);
void aufs_write_unlock(struct dentry *dentry);
void aufs_read_and_write_lock2(struct dentry *d1, struct dentry *d2, int isdir);
void aufs_read_and_write_unlock2(struct dentry *d1, struct dentry *d2);

aufs_bindex_t au_new_br_id(struct super_block *sb);

/* wbr_policy.c */
extern struct au_wbr_copyup_operations au_wbr_copyup_ops[];
extern struct au_wbr_create_operations au_wbr_create_ops[];
int au_cpdown_dirs(struct dentry *dentry, aufs_bindex_t bdst,
		   struct dentry *locked);

/* ---------------------------------------------------------------------- */

#if defined(CONFIG_AUFS_EXPORT) \
	&& (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 26) || !defined(MmTree))
static inline void au_mnt_init(struct au_sbinfo *sbinfo, struct vfsmount *mnt)
{
	spin_lock_init(&sbinfo->si_mntcache_lock);
	sbinfo->si_mntcache = mnt;
}

static inline void au_mnt_reset(struct au_sbinfo *sbinfo)
{
	spin_lock(&sbinfo->si_mntcache_lock);
	sbinfo->si_mntcache = NULL;
	spin_unlock(&sbinfo->si_mntcache_lock);
}
#else
static inline void au_mnt_init(struct au_sbinfo *sbinfo, struct vfsmount *mnt)
{
	/* emptr */
}

static inline void au_mnt_reset(struct au_sbinfo *sbinfo)
{
	/* emptr */
}
#endif /* EXPORT && < 2.6.26 */

/* ---------------------------------------------------------------------- */

static inline struct au_sbinfo *au_sbi(struct super_block *sb)
{
	return sb->s_fs_info;
}

static inline const char *au_sbtype(struct super_block *sb)
{
	return sb->s_type->name;
}

static inline int au_test_aufs(struct super_block *sb)
{
	return (sb->s_magic == AUFS_SUPER_MAGIC);
}

static inline int au_test_nfs(struct super_block *sb)
{
#ifdef CONFIG_AUFS_BR_NFS
	return (sb->s_magic == NFS_SUPER_MAGIC);
#else
	return 0;
#endif
}

static inline int au_test_fuse(struct super_block *sb)
{
#ifdef CONFIG_AUFS_WORKAROUND_FUSE
#ifdef FUSE_SUPER_MAGIC
	return (sb->s_magic == FUSE_SUPER_MAGIC);
#else
	return !strcmp(au_sbtype(sb), "fuse");
#endif
#endif
	return 0;
}

static inline int au_test_xfs(struct super_block *sb)
{
#ifdef CONFIG_AUFS_BR_XFS
#ifdef XFS_SB_MAGIC
	return (sb->s_magic == XFS_SB_MAGIC);
#else
	return !strcmp(au_sbtype(sb), "xfs");
#endif
#endif
	return 0;
}

static inline int au_test_tmpfs(struct super_block *sb)
{
#ifdef CONFIG_TMPFS
#define TMPFS_MAGIC 0x01021994
	return (sb->s_magic == TMPFS_MAGIC);
#endif
	return 0;
}

/* ---------------------------------------------------------------------- */

#ifdef CONFIG_AUFS_EXPORT
extern struct export_operations aufs_export_op;
static inline void au_init_export_op(struct super_block *sb)
{
	sb->s_export_op = &aufs_export_op;
	memset(&au_sbi(sb)->si_xinodir, 0, sizeof(struct path));
}

static inline int au_test_nfsd(struct task_struct *tsk)
{
	return (!tsk->mm && !strcmp(tsk->comm, "nfsd"));
}

static inline void au_nfsd_lockdep_off(void)
{
	if (au_test_nfsd(current))
		lockdep_off();
}

static inline void au_nfsd_lockdep_on(void)
{
	if (au_test_nfsd(current))
		lockdep_on();
}

static inline void au_export_put(struct au_sbinfo *sbinfo)
{
	path_put(&sbinfo->si_xinodir);
}
#else
static inline int au_test_nfsd(struct task_struct *tsk)
{
	return 0;
}

static inline void au_init_export_op(struct super_block *sb)
{
	/* nothing */
}

#define au_nfsd_lockdep_off()	do {} while (0)
#define au_nfsd_lockdep_on()	do {} while (0)

static inline void au_export_put(struct au_sbinfo *sbinfo)
{
	/* nothing */
}
#endif /* CONFIG_AUFS_EXPORT */

#ifdef CONFIG_AUFS_ROBR
static inline int au_test_nested(struct super_block *h_sb)
{
	return 0;
}

static inline void au_robr_lvma_init(struct au_sbinfo *sbinfo)
{
	spin_lock_init(&sbinfo->si_lvma_lock);
	INIT_LIST_HEAD(&sbinfo->si_lvma);
}
#else
static inline int au_test_nested(struct super_block *h_sb)
{
	int err = 0;
	if (unlikely(au_test_aufs(h_sb))) {
		err = -EINVAL;
		AuTraceErr(err);
	}
	return err;
}

static inline void au_robr_lvma_init(struct au_sbinfo *sbinfo)
{
	/* empty */
}
#endif /* CONFIG_AUFS_ROBR */

/* ---------------------------------------------------------------------- */

/* lock superblock. mainly for entry point functions */
/*
 * si_noflush_read_lock, si_noflush_write_lock,
 * si_read_unlock, si_write_unlock, si_downgrade_lock
 */
AuSimpleLockRwsemFuncs(si_noflush, struct super_block *sb,
		       au_sbi(sb)->si_rwsem);
AuSimpleUnlockRwsemFuncs(si, struct super_block *sb, au_sbi(sb)->si_rwsem);

static inline void si_read_lock(struct super_block *sb, int flags)
{
	if (au_ftest_lock(flags, FLUSH))
		au_nwt_flush(&au_sbi(sb)->si_nowait);
	si_noflush_read_lock(sb);
}

static inline void si_write_lock(struct super_block *sb)
{
	au_nwt_flush(&au_sbi(sb)->si_nowait);
	si_noflush_write_lock(sb);
}

static inline int si_read_trylock(struct super_block *sb, int flags)
{
	if (au_ftest_lock(flags, FLUSH))
		au_nwt_flush(&au_sbi(sb)->si_nowait);
	return si_noflush_read_trylock(sb);
}

static inline int si_write_trylock(struct super_block *sb, int flags)
{
	if (au_ftest_lock(flags, FLUSH))
		au_nwt_flush(&au_sbi(sb)->si_nowait);
	return si_noflush_write_trylock(sb);
}

/* to debug easier, do not make them inlined functions */
#define SiMustReadLock(sb)	AuRwMustReadLock(&au_sbi(sb)->si_rwsem)
#define SiMustWriteLock(sb)	AuRwMustWriteLock(&au_sbi(sb)->si_rwsem)
#define SiMustAnyLock(sb)	AuRwMustAnyLock(&au_sbi(sb)->si_rwsem)

/* ---------------------------------------------------------------------- */

static inline aufs_bindex_t au_sbend(struct super_block *sb)
{
	SiMustAnyLock(sb);
	return au_sbi(sb)->si_bend;
}

static inline unsigned int au_mntflags(struct super_block *sb)
{
	SiMustAnyLock(sb);
	return au_sbi(sb)->si_mntflags;
}

static inline au_gen_t au_sigen(struct super_block *sb)
{
	SiMustAnyLock(sb);
	return au_sbi(sb)->si_generation;
}

#endif /* __KERNEL__ */
#endif /* __AUFS_SUPER_H__ */
