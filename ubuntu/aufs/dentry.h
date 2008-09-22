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
 * lookup and dentry operations
 *
 * $Id: dentry.h,v 1.7 2008/09/01 02:54:54 sfjro Exp $
 */

#ifndef __AUFS_DENTRY_H__
#define __AUFS_DENTRY_H__

#ifdef __KERNEL__

#include <linux/fs.h>
#include <linux/namei.h>
#include <linux/aufs_type.h>
#include "misc.h"
#include "super.h"
#include "vfsub.h"

/* nameidata open_intent */
enum {
	AuIntent_AUFS,
	AuIntent_BRANCH,
	AuIntent_Last
};

struct au_hdintent {
	struct list_head	hdi_list;
	struct file		*hdi_file[AuIntent_Last];
};

struct au_hdentry {
	struct dentry		*hd_dentry;

#ifdef CONFIG_AUFS_BR_NFS
	spinlock_t		hd_lock; /* intest_list */
	struct list_head	*hd_intent_list;
#endif
};

struct au_dinfo {
	atomic_t		di_generation;

	struct au_rwsem		di_rwsem;
	aufs_bindex_t		di_bstart, di_bend, di_bwh, di_bdiropq;
	struct au_hdentry	*di_hdentry;
};

/* nameidata extension flags */
#define AuNdx_DLGT	1
#define AuNdx_DIRPERM1	(1 << 1)
#define au_ftest_ndx(flags, name)	((flags) & AuNdx_##name)
#define au_fset_ndx(flags, name)	{ (flags) |= AuNdx_##name; }
#define au_fclr_ndx(flags, name)	{ (flags) &= ~AuNdx_##name; }
#ifndef CONFIG_AUFS_DLGT
#undef AuNdx_DLGT
#define AuNdx_DLGT	0
#undef AuNdx_DIRPERM1
#define AuNdx_DIRPERM1	0
#endif

struct au_ndx {
	struct vfsmount *nfsmnt;
	unsigned int flags;
	struct nameidata *nd;
	struct au_branch *br;
	struct file *nd_file;
};

/* ---------------------------------------------------------------------- */

static inline void au_do_h_dentry_init(struct au_hdentry *hdentry)
{
	hdentry->hd_dentry = NULL;
}

#ifdef CONFIG_AUFS_BR_NFS
static inline void au_h_dentry_init(struct au_hdentry *hdentry)
{
	au_do_h_dentry_init(hdentry);
	spin_lock_init(&hdentry->hd_lock);
}

static inline void au_h_dentry_init_all(struct au_hdentry *hdentry, int n)
{
	while (n--)
		spin_lock_init(&hdentry[n].hd_lock);
}

/* br_nfs.c */
struct file *au_h_intent(struct dentry *dentry, aufs_bindex_t bindex,
			 struct file *file);
int au_br_nfs_h_intent(struct file *nd_file, struct dentry *dentry,
		       aufs_bindex_t bindex, struct nameidata *nd);
void au_hintent_put(struct au_hdentry *hd, int do_free);
int au_fake_intent(struct nameidata *nd, int perm);
int au_hin_after_reval(struct nameidata *nd, struct dentry *dentry,
		       aufs_bindex_t bindex, struct file *file);
struct dentry *au_lkup_hash(const char *name, struct dentry *parent, int len,
			    struct au_ndx *ndx);
#else

static inline void au_h_dentry_init(struct au_hdentry *hdentry)
{
	au_do_h_dentry_init(hdentry);
}

static inline void au_h_dentry_init_all(struct au_hdentry *hdentry, int n)
{
	/* nothing */
}

static inline
struct file *au_h_intent(struct dentry *dentry, aufs_bindex_t bindex,
			 struct file *file)
{
	/* return ERR_PTR(-ENOSYS); */
	return NULL;
}

static inline
int au_br_nfs_h_intent(struct file *nd_file, struct dentry *dentry,
		       aufs_bindex_t bindex, struct nameidata *nd)
{
	return 0;
}

static inline void au_hintent_put(struct au_hdentry *hd, int do_free)
{
	/* empty */
}

static inline int au_fake_intent(struct nameidata *nd, int perm)
{
	return 0;
}

static inline
int au_hin_after_reval(struct nameidata *nd, struct dentry *dentry,
		       aufs_bindex_t bindex, struct file *file)
{
	return 0;
}

#ifdef CONFIG_AUFS_DLGT
static inline
struct dentry *au_lkup_hash(const char *name, struct dentry *parent, int len,
			    struct au_ndx *ndx)
{
	/* return ERR_PTR(-ENOSYS); */
	return vfsub_lookup_one_len(name, parent, len);
}
#endif
#endif /* CONFIG_AUFS_BR_NFS */

#ifdef CONFIG_AUFS_DLGT
/* dlgt.c */
struct dentry *au_lkup_one_dlgt(const char *name, struct dentry *parent,
				int len, unsigned int flags);
#elif defined(CONFIG_AUFS_BR_NFS)
/* regardelss kernel version */
static inline
struct dentry *au_lkup_one_dlgt(const char *name, struct dentry *parent,
				int len, unsigned int flags)
{
	return vfsub_lookup_one_len(name, parent, len);
}
#endif

/* dentry.c */
extern struct dentry_operations aufs_dop;
#if defined(CONFIG_AUFS_BR_NFS) || defined(CONFIG_AUFS_DLGT)
struct dentry *au_lkup_one(const char *name, struct dentry *parent, int len,
			   struct au_ndx *ndx);
#else
static inline
struct dentry *au_lkup_one(const char *name, struct dentry *parent, int len,
			   struct au_ndx *ndx)
{
	/* todo? ndx->nd_file = NULL; */
	return vfsub_lookup_one_len(name, parent, len);
}
#endif
struct dentry *au_sio_lkup_one(const char *name, struct dentry *parent, int len,
			       struct au_ndx *ndx);
int au_lkup_dentry(struct dentry *dentry, aufs_bindex_t bstart, mode_t type,
		   struct nameidata *nd);
int au_lkup_neg(struct dentry *dentry, aufs_bindex_t bindex);
int au_refresh_hdentry(struct dentry *dentry, mode_t type);
int au_reval_dpath(struct dentry *dentry, au_gen_t sgen);

/* dinfo.c */
int au_alloc_dinfo(struct dentry *dentry);
struct au_dinfo *au_di(struct dentry *dentry);

void di_read_lock(struct dentry *d, int flags, unsigned int lsc);
void di_read_unlock(struct dentry *d, int flags);
void di_downgrade_lock(struct dentry *d, int flags);
void di_write_lock(struct dentry *d, unsigned int lsc);
void di_write_unlock(struct dentry *d);
void di_write_lock2_child(struct dentry *d1, struct dentry *d2, int isdir);
void di_write_lock2_parent(struct dentry *d1, struct dentry *d2, int isdir);
void di_write_unlock2(struct dentry *d1, struct dentry *d2);

struct dentry *au_h_dptr(struct dentry *dentry, aufs_bindex_t bindex);

aufs_bindex_t au_dbtail(struct dentry *dentry);
aufs_bindex_t au_dbtaildir(struct dentry *dentry);
#if 0 /* reserved for future use */
aufs_bindex_t au_dbtail_generic(struct dentry *dentry);
#endif

void au_set_dbdiropq(struct dentry *dentry, aufs_bindex_t bindex);
void au_set_h_dptr(struct dentry *dentry, aufs_bindex_t bindex,
		   struct dentry *h_dentry);

void au_update_dbrange(struct dentry *dentry, int do_put_zero);
void au_update_dbstart(struct dentry *dentry);
void au_update_dbend(struct dentry *dentry);
int au_find_dbindex(struct dentry *dentry, struct dentry *h_dentry);

/* ---------------------------------------------------------------------- */

/* todo: memory barrier? */
static inline au_gen_t au_digen(struct dentry *d)
{
	return atomic_read(&au_di(d)->di_generation);
}

#ifdef CONFIG_AUFS_HINOTIFY
static inline au_gen_t au_digen_dec(struct dentry *d)
{
	return atomic_dec_return(&au_di(d)->di_generation);
}

static inline void au_hin_di_reinit(struct dentry *d)
{
	d->d_fsdata = NULL;
}
#else
static inline void au_hin_di_reinit(struct dentry *d)
{
	/* empty */
}
#endif /* CONFIG_AUFS_HINOTIFY */

/* ---------------------------------------------------------------------- */

/* lock subclass for dinfo */
enum {
	AuLsc_DI_CHILD,		/* child first */
	AuLsc_DI_CHILD2,	/* rename(2), link(2), and cpup at hinotify */
	AuLsc_DI_CHILD3,	/* copyup dirs */
	AuLsc_DI_PARENT,
	AuLsc_DI_PARENT2,
	AuLsc_DI_PARENT3,
	AuLsc_DI_PARENT4
};

/*
 * di_read_lock_child, di_write_lock_child,
 * di_read_lock_child2, di_write_lock_child2,
 * di_read_lock_child3, di_write_lock_child3,
 * di_read_lock_parent, di_write_lock_parent,
 * di_read_lock_parent2, di_write_lock_parent2,
 * di_read_lock_parent3, di_write_lock_parent3,
 * di_read_lock_parent4, di_write_lock_parent4,
 */
#define AuReadLockFunc(name, lsc) \
static inline void di_read_lock_##name(struct dentry *d, int flags) \
{ di_read_lock(d, flags, AuLsc_DI_##lsc); }

#define AuWriteLockFunc(name, lsc) \
static inline void di_write_lock_##name(struct dentry *d) \
{ di_write_lock(d, AuLsc_DI_##lsc); }

#define AuRWLockFuncs(name, lsc) \
	AuReadLockFunc(name, lsc) \
	AuWriteLockFunc(name, lsc)

AuRWLockFuncs(child, CHILD);
AuRWLockFuncs(child2, CHILD2);
AuRWLockFuncs(child3, CHILD3);
AuRWLockFuncs(parent, PARENT);
AuRWLockFuncs(parent2, PARENT2);
AuRWLockFuncs(parent3, PARENT3);
AuRWLockFuncs(parent4, PARENT4);

#undef AuReadLockFunc
#undef AuWriteLockFunc
#undef AuRWLockFuncs

/* to debug easier, do not make them inlined functions */
#define DiMustReadLock(d) do { \
	SiMustAnyLock((d)->d_sb); \
	AuRwMustReadLock(&au_di(d)->di_rwsem); \
} while (0)

#define DiMustWriteLock(d) do { \
	SiMustAnyLock((d)->d_sb); \
	AuRwMustWriteLock(&au_di(d)->di_rwsem); \
} while (0)

#define DiMustAnyLock(d) do { \
	SiMustAnyLock((d)->d_sb); \
	AuRwMustAnyLock(&au_di(d)->di_rwsem); \
} while (0)

#define DiMustNoWaiters(d)	AuRwMustNoWaiters(&au_di(d)->di_rwsem)

/* ---------------------------------------------------------------------- */

static inline aufs_bindex_t au_dbstart(struct dentry *dentry)
{
	DiMustAnyLock(dentry);
	return au_di(dentry)->di_bstart;
}

static inline aufs_bindex_t au_dbend(struct dentry *dentry)
{
	DiMustAnyLock(dentry);
	return au_di(dentry)->di_bend;
}

static inline aufs_bindex_t au_dbwh(struct dentry *dentry)
{
	DiMustAnyLock(dentry);
	return au_di(dentry)->di_bwh;
}

static inline aufs_bindex_t au_dbdiropq(struct dentry *dentry)
{
	DiMustAnyLock(dentry);
	AuDebugOn(dentry->d_inode
		  && dentry->d_inode->i_mode
		  && !S_ISDIR(dentry->d_inode->i_mode));
	return au_di(dentry)->di_bdiropq;
}

/* todo: hard/soft set? */
static inline void au_set_dbstart(struct dentry *dentry, aufs_bindex_t bindex)
{
	DiMustWriteLock(dentry);
	AuDebugOn(au_sbend(dentry->d_sb) < bindex);
	/* */
	au_di(dentry)->di_bstart = bindex;
}

static inline void au_set_dbend(struct dentry *dentry, aufs_bindex_t bindex)
{
	DiMustWriteLock(dentry);
	AuDebugOn(au_sbend(dentry->d_sb) < bindex
		  || bindex < au_dbstart(dentry));
	au_di(dentry)->di_bend = bindex;
}

static inline void au_set_dbwh(struct dentry *dentry, aufs_bindex_t bindex)
{
	DiMustWriteLock(dentry);
	AuDebugOn(au_sbend(dentry->d_sb) < bindex);
	/* dbwh can be outside of bstart - bend range */
	au_di(dentry)->di_bwh = bindex;
}

static inline void au_hdput(struct au_hdentry *hd, int do_free)
{
	au_hintent_put(hd, do_free);
	dput(hd->hd_dentry);
}

static inline void au_update_digen(struct dentry *dentry)
{
	AuDebugOn(!dentry->d_sb);
	atomic_set(&au_di(dentry)->di_generation, au_sigen(dentry->d_sb));
	/* smp_mb(); */ /* atomic_set */
}

#endif /* __KERNEL__ */
#endif /* __AUFS_DENTRY_H__ */
