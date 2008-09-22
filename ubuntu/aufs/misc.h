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
 * $Id: misc.h,v 1.7 2008/09/22 03:52:19 sfjro Exp $
 */

#ifndef __AUFS_MISC_H__
#define __AUFS_MISC_H__

#ifdef __KERNEL__

#include <linux/fs.h>
#include <linux/namei.h>
#include <linux/aufs_type.h>

/* ---------------------------------------------------------------------- */

typedef unsigned int au_gen_t;
/* see linux/include/linux/jiffies.h */
#define AuGenYounger(a, b)	((int)(b) - (int)(a) < 0)
#define AuGenOlder(a, b)	AufsGenYounger(b, a)

/* ---------------------------------------------------------------------- */

struct au_rwsem {
	struct rw_semaphore	rwsem;
#ifdef CONFIG_AUFS_DEBUG
	atomic_t		rcnt;
#endif
};

#ifdef CONFIG_AUFS_DEBUG
#define AuDbgRcntInit(rw) do { \
	atomic_set(&(rw)->rcnt, 0); \
	smp_mb(); /* atomic set */ \
} while (0)

#define AuDbgRcntInc(rw)	atomic_inc_return(&(rw)->rcnt)
#define AuDbgRcntDec(rw)	WARN_ON(atomic_dec_return(&(rw)->rcnt) < 0)
#else
#define AuDbgRcntInit(rw)	do {} while (0)
#define AuDbgRcntInc(rw)	do {} while (0)
#define AuDbgRcntDec(rw)	do {} while (0)
#endif /* CONFIG_AUFS_DEBUG */

static inline void au_rw_init_nolock(struct au_rwsem *rw)
{
	AuDbgRcntInit(rw);
	init_rwsem(&rw->rwsem);
}

static inline void au_rw_init_wlock(struct au_rwsem *rw)
{
	au_rw_init_nolock(rw);
	down_write(&rw->rwsem);
}

static inline void au_rw_init_wlock_nested(struct au_rwsem *rw,
					   unsigned int lsc)
{
	au_rw_init_nolock(rw);
	down_write_nested(&rw->rwsem, lsc);
}

static inline void au_rw_read_lock(struct au_rwsem *rw)
{
	down_read(&rw->rwsem);
	AuDbgRcntInc(rw);
}

static inline void au_rw_read_lock_nested(struct au_rwsem *rw, unsigned int lsc)
{
	down_read_nested(&rw->rwsem, lsc);
	AuDbgRcntInc(rw);
}

static inline void au_rw_read_unlock(struct au_rwsem *rw)
{
	AuDbgRcntDec(rw);
	up_read(&rw->rwsem);
	//au_dbg_sleep_jiffy(1 * HZ / HZ);
}

static inline void au_rw_dgrade_lock(struct au_rwsem *rw)
{
	AuDbgRcntInc(rw);
	downgrade_write(&rw->rwsem);
	//au_dbg_sleep_jiffy(1 * HZ / HZ);
}

static inline void au_rw_write_lock(struct au_rwsem *rw)
{
	down_write(&rw->rwsem);
}

static inline void au_rw_write_lock_nested(struct au_rwsem *rw,
					   unsigned int lsc)
{
	down_write_nested(&rw->rwsem, lsc);
}

static inline void au_rw_write_unlock(struct au_rwsem *rw)
{
	up_write(&rw->rwsem);
	//au_dbg_sleep_jiffy(1 * HZ / HZ);
}

/* why is not _nested version defined */
static inline int au_rw_read_trylock(struct au_rwsem *rw)
{
	int ret = down_read_trylock(&rw->rwsem);
	if (ret)
		AuDbgRcntInc(rw);
	return ret;
}

static inline int au_rw_write_trylock(struct au_rwsem *rw)
{
	return down_write_trylock(&rw->rwsem);
}

#undef AuDbgRcntInit
#undef AuDbgRcntInc
#undef AuDbgRcntDec

/* to debug easier, do not make them inlined functions */
#define AuRwMustNoWaiters(rw)	AuDebugOn(!list_empty(&(rw)->rwsem.wait_list))
#define AuRwMustAnyLock(rw)	AuDebugOn(down_write_trylock(&(rw)->rwsem))
#ifdef CONFIG_AUFS_DEBUG
#define AuRwMustReadLock(rw) do { \
	AuRwMustAnyLock(rw); \
	AuDebugOn(!atomic_read(&(rw)->rcnt)); \
} while (0)

#define AuRwMustWriteLock(rw) do { \
	AuRwMustAnyLock(rw); \
	AuDebugOn(atomic_read(&(rw)->rcnt)); \
} while (0)
#else
#define AuRwMustReadLock(rw)	AuRwMustAnyLock(rw)
#define AuRwMustWriteLock(rw)	AuRwMustAnyLock(rw)
#endif /* CONFIG_AUFS_DEBUG */

#define AuSimpleLockRwsemFuncs(prefix, param, rwsem) \
static inline void prefix##_read_lock(param) \
{ au_rw_read_lock(&(rwsem)); } \
static inline void prefix##_write_lock(param) \
{ au_rw_write_lock(&(rwsem)); } \
static inline int prefix##_read_trylock(param) \
{ return au_rw_read_trylock(&(rwsem)); } \
static inline int prefix##_write_trylock(param) \
{ return au_rw_write_trylock(&(rwsem)); }
/* static inline void prefix##_read_trylock_nested(param, lsc)
{au_rw_read_trylock_nested(&(rwsem, lsc));}
static inline void prefix##_write_trylock_nestd(param, lsc)
{au_rw_write_trylock_nested(&(rwsem), nested);} */

#define AuSimpleUnlockRwsemFuncs(prefix, param, rwsem) \
static inline void prefix##_read_unlock(param) \
{ au_rw_read_unlock(&(rwsem)); } \
static inline void prefix##_write_unlock(param) \
{ au_rw_write_unlock(&(rwsem)); } \
static inline void prefix##_downgrade_lock(param) \
{ au_rw_dgrade_lock(&(rwsem)); }

#define AuSimpleRwsemFuncs(prefix, param, rwsem) \
	AuSimpleLockRwsemFuncs(prefix, param, rwsem) \
	AuSimpleUnlockRwsemFuncs(prefix, param, rwsem)

/* ---------------------------------------------------------------------- */

void *au_kzrealloc(void *p, unsigned int nused, unsigned int new_sz, gfp_t gfp);

struct au_nd_store {
	unsigned int		flags;
	struct path		path;
	struct open_intent	intent;
};
struct au_sbinfo;
void au_nd_store(struct au_nd_store *store, struct nameidata *nd,
		 struct au_sbinfo *sbinfo);
void au_nd_revert(struct au_nd_store *store, struct nameidata *nd,
		  struct au_sbinfo *sbinfo);

struct nameidata *au_dup_nd(struct au_sbinfo *sbinfo, struct nameidata *dst,
			    struct nameidata *src);

struct nameidata *au_fake_dm(struct nameidata *fake_nd, struct nameidata *nd,
			     struct super_block *sb, aufs_bindex_t bindex);
void au_fake_dm_release(struct nameidata *fake_nd);
struct vfsub_args;
int au_h_create(struct inode *h_dir, struct dentry *h_dentry, int mode,
		struct vfsub_args *vargs, struct nameidata *nd,
		struct vfsmount *nfsmnt);

struct au_hinode;
int au_copy_file(struct file *dst, struct file *src, loff_t len,
		 struct au_hinode *hdir, struct super_block *sb,
		 struct vfsub_args *vargs);

#endif /* __KERNEL__ */
#endif /* __AUFS_MISC_H__ */
