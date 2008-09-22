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
 * mount options/flags
 *
 * $Id: opts.h,v 1.6 2008/08/17 23:03:27 sfjro Exp $
 */

#ifndef __AUFS_OPTS_H__
#define __AUFS_OPTS_H__

#ifdef __KERNEL__

#include <linux/fs.h>
#include <linux/namei.h>
#include <linux/aufs_type.h>
#include "wkq.h"

/* ---------------------------------------------------------------------- */
/* mount flags */

/* external inode number bitmap and translation table */
#define AuOpt_XINO		1
#define AuOpt_XINODIR		(1 << 1)
#define AuOpt_TRUNC_XINO	(1 << 2)
#define AuOpt_UDBA_NONE		(1 << 3)	/* users direct branch access */
#define AuOpt_UDBA_REVAL	(1 << 4)
#define AuOpt_UDBA_INOTIFY	(1 << 5)
#define AuOpt_SHWH		(1 << 6)
#define AuOpt_PLINK		(1 << 7)
#define AuOpt_WARN_PERM		(1 << 8)
#define AuOpt_DIRPERM1		(1 << 9)
#define AuOpt_DLGT		(1 << 10)
#define AuOpt_COO_NONE		(1 << 11)	/* copyup on open */
#define AuOpt_COO_LEAF		(1 << 12)
#define AuOpt_COO_ALL		(1 << 13)
#define AuOpt_ALWAYS_DIROPQ	(1 << 14)
#define AuOpt_REFROF		(1 << 15)
#define AuOpt_VERBOSE		(1 << 16)

#if 1 /* ndef CONFIG_AUFS_EXPORT */ /* reserved for future use */
#undef AuOpt_XINODIR
#define AuOpt_XINODIR		0
#endif
#ifndef CONFIG_AUFS_HINOTIFY
#undef AuOpt_UDBA_INOTIFY
#define AuOpt_UDBA_INOTIFY	0
#endif
#ifndef CONFIG_AUFS_SHWH
#undef AuOpt_SHWH
#define AuOpt_SHWH		0
#endif
#ifndef CONFIG_AUFS_DLGT
#undef AuOpt_DIRPERM1
#define AuOpt_DIRPERM1		0
#undef AuOpt_DLGT
#define AuOpt_DLGT		0
#endif

/* policies to select one among multiple writable branches */
enum {
	AuWbrCreate_TDP,	/* top down parent */
	AuWbrCreate_RR,		/* round robin */
	AuWbrCreate_MFS,	/* most free space */
	AuWbrCreate_MFSV,	/* mfs with seconds */
	AuWbrCreate_MFSRR,	/* mfs then rr */
	AuWbrCreate_MFSRRV,	/* mfs then rr with seconds */
	AuWbrCreate_PMFS,	/* parent and mfs */
	AuWbrCreate_PMFSV,	/* parent and mfs with seconds */

	AuWbrCreate_Def = AuWbrCreate_TDP
};

enum {
	AuWbrCopyup_TDP,	/* top down parent */
	AuWbrCopyup_BUP,	/* bottom up parent */
	AuWbrCopyup_BU,		/* bottom up */

	AuWbrCopyup_Def = AuWbrCopyup_TDP
};

#define AuOptMask_COO		(AuOpt_COO_NONE \
				 | AuOpt_COO_LEAF \
				 | AuOpt_COO_ALL)
#define AuOptMask_UDBA		(AuOpt_UDBA_NONE \
				 | AuOpt_UDBA_REVAL \
				 | AuOpt_UDBA_INOTIFY)

#ifdef CONFIG_AUFS_COMPAT
#define AuOpt_DefExtra1	AuOpt_ALWAYS_DIROPQ
#else
#define AuOpt_DefExtra1	0
#endif

#define AuOpt_Def	(AuOpt_XINO \
			 | AuOpt_UDBA_REVAL \
			 | AuOpt_WARN_PERM \
			 | AuOpt_COO_NONE \
			 | AuOpt_PLINK \
			 | AuOpt_DefExtra1)

/* ---------------------------------------------------------------------- */

struct au_opt_add {
	aufs_bindex_t		bindex;
	char			*path;
	int			perm;
	struct nameidata	nd;
};

struct au_opt_del {
	char		*path;
	struct dentry	*h_root;
};

struct au_opt_mod {
	char		*path;
	int		perm;
	struct dentry	*h_root;
};

struct au_opt_xino {
	char		*path;
	struct file	*file;
};

struct au_opt_xinodir {
	char		*name;
	struct path	path;
};

struct au_opt_xino_itrunc {
	aufs_bindex_t	bindex;
};

struct au_opt_xino_trunc_v {
	unsigned long long	upper;
	int			step;
};

struct au_opt_wbr_create {
	int			wbr_create;
	int			mfs_second;
	unsigned long long	mfsrr_watermark;
};

struct au_opt {
	int type;
	union {
		struct au_opt_xino	xino;
		struct au_opt_xinodir	xinodir;
		struct au_opt_xino_itrunc xino_itrunc;
		struct au_opt_add	add;
		struct au_opt_del	del;
		struct au_opt_mod	mod;
		int			dirwh;
		int			rdcache;
		int			deblk;
		int			nhash;
		int			udba;
		int			coo;
		struct au_opt_wbr_create wbr_create;
		int			wbr_copyup;
	};
};

/* opts flags */
#define AuOpts_REMOUNT		1
#define AuOpts_REFRESH_DIR	(1 << 1)
#define AuOpts_REFRESH_NONDIR	(1 << 2)
#define AuOpts_TRUNC_XIB	(1 << 3)
#define au_ftest_opts(flags, name)	((flags) & AuOpts_##name)
#define au_fset_opts(flags, name)	{ (flags) |= AuOpts_##name; }
#define au_fclr_opts(flags, name)	{ (flags) &= ~AuOpts_##name; }

struct au_opts {
	struct au_opt	*opt;
	int		max_opt;

	unsigned int	given_udba;
	unsigned int	flags;
};

/* ---------------------------------------------------------------------- */

const char *au_optstr_br_perm(int brperm);
const char *au_optstr_udba(int udba);
const char *au_optstr_coo(int coo);
const char *au_optstr_wbr_copyup(int wbr_copyup);
const char *au_optstr_wbr_create(int wbr_create);

void au_opts_free(struct au_opts *opts);
int au_opts_parse(struct super_block *sb, unsigned long flags, char *str,
		  struct au_opts *opts);
int au_opts_mount(struct super_block *sb, struct au_opts *opts);
int au_opts_remount(struct super_block *sb, struct au_opts *opts);

/* ---------------------------------------------------------------------- */

#define au_opt_test(flags, name)	(flags & AuOpt_##name)

static inline int au_opt_test_xino(unsigned int flags)
{
	return (flags & (AuOpt_XINO | AuOpt_XINODIR));
}

#define au_opt_set(flags, name) do { \
	BUILD_BUG_ON(AuOpt_##name & (AuOptMask_COO | AuOptMask_UDBA)); \
	((flags) |= AuOpt_##name); \
} while (0)

#define au_opt_set_coo(flags, name) do { \
	(flags) &= ~AuOptMask_COO; \
	((flags) |= AuOpt_##name); \
} while (0)

#define au_opt_set_udba(flags, name) do { \
	(flags) &= ~AuOptMask_UDBA; \
	((flags) |= AuOpt_##name); \
} while (0)

#define au_opt_clr(flags, name)		{ ((flags) &= ~AuOpt_##name); }

static inline int au_test_dlgt(unsigned int flags)
{
	return (au_opt_test(flags, DLGT) && !au_test_wkq(current));
}

static inline int au_test_dirperm1(unsigned int flags)
{
	return (au_opt_test(flags, DIRPERM1) && !au_test_wkq(current));
}

#endif /* __KERNEL__ */
#endif /* __AUFS_OPTS_H__ */
