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
 * whiteout for logical deletion and opaque directory
 *
 * $Id: whout.h,v 1.3 2008/06/30 03:57:35 sfjro Exp $
 */

#ifndef __AUFS_WHOUT_H__
#define __AUFS_WHOUT_H__

#ifdef __KERNEL__

#include <linux/fs.h>
#include <linux/aufs_type.h>
#include "dir.h"
#include "opts.h"
#include "super.h"

int au_wh_name_alloc(const char *name, int len, struct qstr *wh);
void au_wh_name_free(struct qstr *wh);

struct au_ndx;
int au_wh_test(struct dentry *h_parent, struct qstr *wh_name, int try_sio,
	       struct au_ndx *ndx);
int au_diropq_test(struct dentry *h_dentry, struct au_ndx *ndx);

struct dentry *au_whtmp_lkup(struct dentry *h_parent, struct qstr *prefix,
			     struct au_ndx *ndx);
int au_whtmp_ren(struct inode *dir, aufs_bindex_t bindex,
		 struct dentry *h_dentry);
int au_wh_unlink_dentry(struct au_hinode *dir, struct dentry *wh_dentry,
			struct dentry *dentry, int dlgt);

struct au_branch;
int au_wh_init(struct dentry *h_parent, struct au_branch *br,
	       struct vfsmount *nfsmnt, struct super_block *sb,
	       aufs_bindex_t bindex);

/* diropq flags */
#define AuDiropq_CREATE	1
#define AuDiropq_DLGT	(1 << 1)
#define au_ftest_diropq(flags, name)	((flags) & AuDiropq_##name)
#define au_fset_diropq(flags, name)	{ (flags) |= AuDiropq_##name; }
#define au_fclr_diropq(flags, name)	{ (flags) &= ~AuDiropq_##name; }
#ifndef CONFIG_AUFS_DLGT
#undef AuDiropq_DLGT
#define AuDiropq_DLGT	0
#endif

struct dentry *au_diropq_sio(struct dentry *dentry, aufs_bindex_t bindex,
			     unsigned int flags);

struct dentry *au_wh_lkup(struct dentry *h_parent, struct qstr *base_name,
			  struct au_ndx *ndx);
struct dentry *au_wh_create(struct dentry *dentry, aufs_bindex_t bindex,
			    struct dentry *h_parent, struct au_ndx *ndx);

/* real rmdir the whiteout-ed dir */
struct au_whtmp_rmdir_args {
	struct inode *dir;
	aufs_bindex_t bindex;
	struct dentry *wh_dentry;
	struct au_nhash whlist;
};

struct au_nhash;
int au_whtmp_rmdir(struct inode *dir, aufs_bindex_t bindex,
		   struct dentry *wh_dentry, struct au_nhash *whlist);
void au_whtmp_kick_rmdir(struct inode *dir, aufs_bindex_t bindex,
			 struct dentry *wh_dentry, struct au_nhash *whlist,
			 struct au_whtmp_rmdir_args *args);

/* ---------------------------------------------------------------------- */

static inline
struct dentry *au_diropq_create(struct dentry *dentry, aufs_bindex_t bindex,
				int dlgt)
{
	unsigned int flags = AuDiropq_CREATE;
	if (unlikely(dlgt))
		au_fset_diropq(flags, DLGT);
	return au_diropq_sio(dentry, bindex, flags);
}

static inline
int au_diropq_remove(struct dentry *dentry, aufs_bindex_t bindex, int dlgt)
{
	unsigned int flags = !AuDiropq_CREATE;
	if (unlikely(dlgt))
		au_fset_diropq(flags, DLGT);
	return PTR_ERR(au_diropq_sio(dentry, bindex, flags));
}

/* ---------------------------------------------------------------------- */

#ifdef CONFIG_AUFS_ROBR
/* robr.c */
int au_test_robr_wh(struct qstr *name, struct dentry *h_parent,
		    struct qstr *wh_name, int try_sio, struct au_ndx *ndx);
int au_test_robr_shwh(struct super_block *sb, const struct qstr *name);
#else
static inline
int au_test_robr_wh(struct qstr *name, struct dentry *h_parent,
		    struct qstr *wh_name, int try_sio, struct au_ndx *ndx)
{
	return au_wh_test(h_parent, wh_name, try_sio, ndx);
}

static inline
int au_test_robr_shwh(struct super_block *sb, const struct qstr *name)
{
	if (unlikely(!au_opt_test(au_mntflags(sb), SHWH)
		     && !strncmp(name->name, AUFS_WH_PFX, AUFS_WH_PFX_LEN)))
		return -EPERM;
	return 0;
}
#endif /* CONFIG_AUFS_ROBR */

#endif /* __KERNEL__ */
#endif /* __AUFS_WHOUT_H__ */
