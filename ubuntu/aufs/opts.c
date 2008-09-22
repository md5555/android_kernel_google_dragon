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
 * $Id: opts.c,v 1.15 2008/09/01 02:55:31 sfjro Exp $
 */

#include <linux/types.h> /* a distribution requires */
#include <linux/parser.h>
#include "aufs.h"

/* ---------------------------------------------------------------------- */

enum {
	Opt_br,
	Opt_add, Opt_del, Opt_mod, Opt_reorder, Opt_append, Opt_prepend,
	Opt_idel, Opt_imod, Opt_ireorder,
	Opt_dirwh, Opt_rdcache, Opt_deblk, Opt_nhash, Opt_rendir,
	Opt_xino, Opt_zxino, Opt_noxino,
	Opt_trunc_xino, Opt_trunc_xino_v, Opt_notrunc_xino,
	Opt_trunc_xino_path, Opt_itrunc_xino,
	Opt_xinodir,
	Opt_trunc_xib, Opt_notrunc_xib,
	Opt_dirperm1, Opt_nodirperm1,
	Opt_shwh, Opt_noshwh,
	Opt_plink, Opt_noplink, Opt_list_plink, Opt_clean_plink,
	Opt_udba,
	/* Opt_lock, Opt_unlock, */
	Opt_cmd, Opt_cmd_args,
	Opt_diropq_a, Opt_diropq_w,
	Opt_warn_perm, Opt_nowarn_perm,
	Opt_wbr_copyup, Opt_wbr_create,
	Opt_coo,
	Opt_dlgt, Opt_nodlgt,
	Opt_refrof, Opt_norefrof,
	Opt_verbose, Opt_noverbose,
	Opt_tail, Opt_ignore, Opt_ignore_silent, Opt_err
};

static match_table_t options = {
	{Opt_br, "br=%s"},
	{Opt_br, "br:%s"},

	{Opt_add, "add=%d:%s"},
	{Opt_add, "add:%d:%s"},
	{Opt_add, "ins=%d:%s"},
	{Opt_add, "ins:%d:%s"},
	{Opt_append, "append=%s"},
	{Opt_append, "append:%s"},
	{Opt_prepend, "prepend=%s"},
	{Opt_prepend, "prepend:%s"},

	{Opt_del, "del=%s"},
	{Opt_del, "del:%s"},
	/* {Opt_idel, "idel:%d"}, */
	{Opt_mod, "mod=%s"},
	{Opt_mod, "mod:%s"},
	{Opt_imod, "imod:%d:%s"},

	{Opt_dirwh, "dirwh=%d"},
	{Opt_dirwh, "dirwh:%d"},

	{Opt_xino, "xino=%s"},
	{Opt_xino, "xino:%s"},
#if 0 /* def CONFIG_AUFS_EXPORT */ /* reserved for futur use */
	{Opt_xinodir, "xinodir=%s"},
	{Opt_xinodir, "xinodir:%s"},
#endif
	{Opt_noxino, "noxino"},
	{Opt_trunc_xino, "trunc_xino"},
	{Opt_trunc_xino_v, "trunc_xino_v=%d:%d"},
	{Opt_notrunc_xino, "notrunc_xino"},
	{Opt_trunc_xino_path, "trunc_xino=%s"},
	{Opt_trunc_xino_path, "trunc_xino:%s"},
	{Opt_itrunc_xino, "itrunc_xino=%d"},
	{Opt_itrunc_xino, "itrunc_xino:%d"},
	/* {Opt_zxino, "zxino=%s"}, */
	{Opt_trunc_xib, "trunc_xib"},
	{Opt_notrunc_xib, "notrunc_xib"},

	{Opt_plink, "plink"},
	{Opt_noplink, "noplink"},
#ifdef CONFIG_AUFS_DEBUG
	{Opt_list_plink, "list_plink"},
#endif
	{Opt_clean_plink, "clean_plink"},

	{Opt_udba, "udba=%s"},

	{Opt_diropq_a, "diropq=always"},
	{Opt_diropq_a, "diropq=a"},
	{Opt_diropq_w, "diropq=whiteouted"},
	{Opt_diropq_w, "diropq=w"},

	{Opt_warn_perm, "warn_perm"},
	{Opt_nowarn_perm, "nowarn_perm"},

#ifdef CONFIG_AUFS_DLGT
	{Opt_dlgt, "dlgt"},
	{Opt_nodlgt, "nodlgt"},

	{Opt_dirperm1, "dirperm1"},
	{Opt_nodirperm1, "nodirperm1"},
#endif

#ifdef CONFIG_AUFS_SHWH
	{Opt_shwh, "shwh"},
	{Opt_noshwh, "noshwh"},
#endif

	{Opt_rendir, "rendir=%d"},
	{Opt_rendir, "rendir:%d"},

	{Opt_refrof, "refrof"},
	{Opt_norefrof, "norefrof"},

	{Opt_verbose, "verbose"},
	{Opt_verbose, "v"},
	{Opt_noverbose, "noverbose"},
	{Opt_noverbose, "quiet"},
	{Opt_noverbose, "q"},
	{Opt_noverbose, "silent"},

	{Opt_rdcache, "rdcache=%d"},
	{Opt_rdcache, "rdcache:%d"},

	{Opt_coo, "coo=%s"},

	{Opt_wbr_create, "create=%s"},
	{Opt_wbr_create, "create:%s"},
	{Opt_wbr_create, "create_policy=%s"},
	{Opt_wbr_create, "create_policy:%s"},
	{Opt_wbr_copyup, "cpup=%s"},
	{Opt_wbr_copyup, "cpup:%s"},
	{Opt_wbr_copyup, "copyup=%s"},
	{Opt_wbr_copyup, "copyup:%s"},
	{Opt_wbr_copyup, "copyup_policy=%s"},
	{Opt_wbr_copyup, "copyup_policy:%s"},

	/* internal use for the scripts */
	{Opt_ignore_silent, "si=%s"},

#if 0 /* reserved for future use */
	{Opt_deblk, "deblk=%d"},
	{Opt_deblk, "deblk:%d"},
	{Opt_nhash, "nhash=%d"},
	{Opt_nhash, "nhash:%d"},
#endif

	{Opt_br, "dirs=%s"},
	{Opt_ignore, "debug=%d"},
	{Opt_ignore, "delete=whiteout"},
	{Opt_ignore, "delete=all"},
	{Opt_ignore, "imap=%s"},

	{Opt_err, NULL}
};

/* ---------------------------------------------------------------------- */

static const char *au_parser_pattern(int val, struct match_token *token)
{
	while (token->pattern) {
		if (token->token == val)
			return token->pattern;
		token++;
	}
	BUG();
	return "??";
}

/* ---------------------------------------------------------------------- */

#define RW		"rw"
#define RO		"ro"
#define WH		"wh"
#define RR		"rr"
#define NoLinkWH	"nolwh"

static match_table_t brperms = {
	{AuBrPerm_RR, RR},
	{AuBrPerm_RO, RO},
	{AuBrPerm_RW, RW},

	{AuBrPerm_RRWH, RR "+" WH},
	{AuBrPerm_ROWH, RO "+" WH},
	{AuBrPerm_RWNoLinkWH, RW "+" NoLinkWH},

	{AuBrPerm_ROWH, "nfsro"},
	{AuBrPerm_RO, NULL}
};

static noinline_for_stack int br_perm_val(char *perm)
{
	int val;
	substring_t args[MAX_OPT_ARGS];

	AuDebugOn(!perm || !*perm);
	LKTRTrace("perm %s\n", perm);
	val = match_token(perm, brperms, args);
	AuTraceErr(val);
	return val;
}

const char *au_optstr_br_perm(int brperm)
{
	return au_parser_pattern(brperm, (void *)brperms);
}

/* ---------------------------------------------------------------------- */

static match_table_t udbalevel = {
	{AuOpt_UDBA_REVAL, "reval"},
#ifdef CONFIG_AUFS_HINOTIFY
	{AuOpt_UDBA_INOTIFY, "inotify"},
#endif
	{AuOpt_UDBA_NONE, "none"},
	{-1, NULL}
};

static noinline_for_stack int udba_val(char *str)
{
	substring_t args[MAX_OPT_ARGS];
	return match_token(str, udbalevel, args);
}

const char *au_optstr_udba(int udba)
{
	return au_parser_pattern(udba, (void *)udbalevel);
}

/* ---------------------------------------------------------------------- */

static match_table_t coolevel = {
	{AuOpt_COO_NONE, "none"},
	{AuOpt_COO_LEAF, "leaf"},
	{AuOpt_COO_ALL, "all"},
	{-1, NULL}
};

static noinline_for_stack int coo_val(char *str)
{
	substring_t args[MAX_OPT_ARGS];
	return match_token(str, coolevel, args);
}

const char *au_optstr_coo(int coo)
{
	return au_parser_pattern(coo, (void *)coolevel);
}

/* ---------------------------------------------------------------------- */

static match_table_t au_wbr_create_policy = {
	{AuWbrCreate_TDP, "tdp"},
	{AuWbrCreate_TDP, "top-down-parent"},
	{AuWbrCreate_RR, "rr"},
	{AuWbrCreate_RR, "round-robin"},
	{AuWbrCreate_MFS, "mfs"},
	{AuWbrCreate_MFS, "most-free-space"},
	{AuWbrCreate_MFSV, "mfs:%d"},
	{AuWbrCreate_MFSV, "most-free-space:%d"},

	{AuWbrCreate_MFSRR, "mfsrr:%d"},
	{AuWbrCreate_MFSRRV, "mfsrr:%d:%d"},
	{AuWbrCreate_PMFS, "pmfs"},
	{AuWbrCreate_PMFSV, "pmfs:%d"},

	{-1, NULL}
};

/* cf. linux/lib/parser.c */
static int au_match_ull(substring_t *s, unsigned long long *result, int base)
{
	char *endp;
	char *buf;
	int ret;

	buf = kmalloc(s->to - s->from + 1, GFP_NOFS);
	if (!buf)
		return -ENOMEM;
	memcpy(buf, s->from, s->to - s->from);
	buf[s->to - s->from] = '\0';
	*result = simple_strtoull(buf, &endp, base);
	ret = 0;
	if (endp == buf)
		ret = -EINVAL;
	kfree(buf);
	return ret;
}

static int au_wbr_mfs_wmark(substring_t *arg, char *str,
			    struct au_opt_wbr_create *create)
{
	int err;
	unsigned long long ull;

	err = 0;
	if (!au_match_ull(arg, &ull, 0))
		create->mfsrr_watermark = ull;
	else {
		AuErr("bad integer in %s\n", str);
		err = -EINVAL;
	}

	AuTraceErr(err);
	return err;
}

static int au_wbr_mfs_sec(substring_t *arg, char *str,
			  struct au_opt_wbr_create *create)
{
	int n, err;

	err = 0;
	if (!match_int(arg, &n) && 0 <= n)
		create->mfs_second = n;
	else {
		AuErr("bad integer in %s\n", str);
		err = -EINVAL;
	}

	AuTraceErr(err);
	return err;
}

static noinline_for_stack
int au_wbr_create_val(char *str, struct au_opt_wbr_create *create)
{
	int err, e;
	substring_t args[MAX_OPT_ARGS];

	err = match_token(str, au_wbr_create_policy, args);
	create->wbr_create = err;
	switch (err) {
	case AuWbrCreate_MFSRRV:
		e = au_wbr_mfs_wmark(&args[0], str, create);
		if (!e)
			e = au_wbr_mfs_sec(&args[1], str, create);
		if (unlikely(e))
			err = e;
		break;
	case AuWbrCreate_MFSRR:
		e = au_wbr_mfs_wmark(&args[0], str, create);
		if (unlikely(e)) {
			err = e;
			break;
		}
		/*FALLTHROUGH*/
	case AuWbrCreate_MFS:
	case AuWbrCreate_PMFS:
		create->mfs_second = AUFS_MFS_SECOND_DEF;
		break;
	case AuWbrCreate_MFSV:
	case AuWbrCreate_PMFSV:
		e = au_wbr_mfs_sec(&args[0], str, create);
		if (unlikely(e))
			err = e;
		break;
	}

	return err;
}

const char *au_optstr_wbr_create(int wbr_create)
{
	return au_parser_pattern(wbr_create, (void *)au_wbr_create_policy);
}

static match_table_t au_wbr_copyup_policy = {
	{AuWbrCopyup_TDP, "tdp"},
	{AuWbrCopyup_TDP, "top-down-parent"},
	{AuWbrCopyup_BUP, "bup"},
	{AuWbrCopyup_BUP, "bottom-up-parent"},
	{AuWbrCopyup_BU, "bu"},
	{AuWbrCopyup_BU, "bottom-up"},
	{-1, NULL}
};

static noinline_for_stack int au_wbr_copyup_val(char *str)
{
	substring_t args[MAX_OPT_ARGS];
	return match_token(str, au_wbr_copyup_policy, args);
}

const char *au_optstr_wbr_copyup(int wbr_copyup)
{
	return au_parser_pattern(wbr_copyup, (void *)au_wbr_copyup_policy);
}

/* ---------------------------------------------------------------------- */

static const int lkup_dirflags = LOOKUP_FOLLOW | LOOKUP_DIRECTORY;

static void dump_opts(struct au_opts *opts)
{
#ifdef CONFIG_AUFS_DEBUG
	/* reduce stack space */
	union {
		struct au_opt_add *add;
		struct au_opt_del *del;
		struct au_opt_mod *mod;
		struct au_opt_xino *xino;
		struct au_opt_xinodir *xinodir;
		struct au_opt_xino_itrunc *xino_itrunc;
		struct au_opt_wbr_create *create;
	} u;
	struct au_opt *opt;

	AuTraceEnter();

	opt = opts->opt;
	while (/* opt < opts_tail && */ opt->type != Opt_tail) {
		switch (opt->type) {
		case Opt_add:
			u.add = &opt->add;
			LKTRTrace("add {b%d, %s, 0x%x, %p}\n",
				  u.add->bindex, u.add->path, u.add->perm,
				  u.add->nd.path.dentry);
			break;
		case Opt_del:
		case Opt_idel:
			u.del = &opt->del;
			LKTRTrace("del {%s, %p}\n", u.del->path, u.del->h_root);
			break;
		case Opt_mod:
		case Opt_imod:
			u.mod = &opt->mod;
			LKTRTrace("mod {%s, 0x%x, %p}\n",
				  u.mod->path, u.mod->perm, u.mod->h_root);
			break;
		case Opt_append:
			u.add = &opt->add;
			LKTRTrace("append {b%d, %s, 0x%x, %p}\n",
				  u.add->bindex, u.add->path, u.add->perm,
				  u.add->nd.path.dentry);
			break;
		case Opt_prepend:
			u.add = &opt->add;
			LKTRTrace("prepend {b%d, %s, 0x%x, %p}\n",
				  u.add->bindex, u.add->path, u.add->perm,
				  u.add->nd.path.dentry);
			break;
		case Opt_dirwh:
			LKTRTrace("dirwh %d\n", opt->dirwh);
			break;
		case Opt_rdcache:
			LKTRTrace("rdcache %d\n", opt->rdcache);
			break;
		case Opt_xino:
			u.xino = &opt->xino;
			LKTRTrace("xino {%s %.*s}\n",
				  u.xino->path,
				  AuDLNPair(u.xino->file->f_dentry));
			break;
		case Opt_xinodir:
			u.xinodir = &opt->xinodir;
			LKTRTrace("xinodir {%s %.*s}\n",
				  u.xinodir->name,
				  AuDLNPair(u.xinodir->path.dentry));
			break;
		case Opt_trunc_xino:
			LKTRLabel(trunc_xino);
			break;
		case Opt_notrunc_xino:
			LKTRLabel(notrunc_xino);
			break;
		case Opt_trunc_xino_path:
		case Opt_itrunc_xino:
			u.xino_itrunc = &opt->xino_itrunc;
			LKTRTrace("trunc_xino %d\n", u.xino_itrunc->bindex);
			break;

		case Opt_noxino:
			LKTRLabel(noxino);
			break;
		case Opt_trunc_xib:
			LKTRLabel(trunc_xib);
			break;
		case Opt_notrunc_xib:
			LKTRLabel(notrunc_xib);
			break;
		case Opt_dirperm1:
			LKTRLabel(dirperm1);
			break;
		case Opt_nodirperm1:
			LKTRLabel(nodirperm1);
			break;
		case Opt_shwh:
			LKTRLabel(shwh);
			break;
		case Opt_noshwh:
			LKTRLabel(noshwh);
			break;
		case Opt_plink:
			LKTRLabel(plink);
			break;
		case Opt_noplink:
			LKTRLabel(noplink);
			break;
		case Opt_list_plink:
			LKTRLabel(list_plink);
			break;
		case Opt_clean_plink:
			LKTRLabel(clean_plink);
			break;
		case Opt_udba:
			LKTRTrace("udba %d, %s\n",
				  opt->udba, au_optstr_udba(opt->udba));
			break;
		case Opt_diropq_a:
			LKTRLabel(diropq_a);
			break;
		case Opt_diropq_w:
			LKTRLabel(diropq_w);
			break;
		case Opt_warn_perm:
			LKTRLabel(warn_perm);
			break;
		case Opt_nowarn_perm:
			LKTRLabel(nowarn_perm);
			break;
		case Opt_dlgt:
			LKTRLabel(dlgt);
			break;
		case Opt_nodlgt:
			LKTRLabel(nodlgt);
			break;
		case Opt_refrof:
			LKTRLabel(refrof);
			break;
		case Opt_norefrof:
			LKTRLabel(norefrof);
			break;
		case Opt_verbose:
			LKTRLabel(verbose);
			break;
		case Opt_noverbose:
			LKTRLabel(noverbose);
			break;
		case Opt_coo:
			LKTRTrace("coo %d, %s\n",
				  opt->coo, au_optstr_coo(opt->coo));
			break;
		case Opt_wbr_create:
			u.create = &opt->wbr_create;
			LKTRTrace("create %d, %s\n", u.create->wbr_create,
				  au_optstr_wbr_create(u.create->wbr_create));
			switch (u.create->wbr_create) {
			case AuWbrCreate_MFSV:
			case AuWbrCreate_PMFSV:
				LKTRTrace("%d sec\n", u.create->mfs_second);
				break;
			case AuWbrCreate_MFSRR:
				LKTRTrace("%llu watermark\n",
					  u.create->mfsrr_watermark);
				break;
			case AuWbrCreate_MFSRRV:
				LKTRTrace("%llu watermark, %d sec\n",
					  u.create->mfsrr_watermark,
					  u.create->mfs_second);
				break;
			}
			break;
		case Opt_wbr_copyup:
			LKTRTrace("copyup %d, %s\n", opt->wbr_copyup,
				  au_optstr_wbr_copyup(opt->wbr_copyup));
			break;
		default:
			BUG();
		}
		opt++;
	}
#endif
}

void au_opts_free(struct au_opts *opts)
{
	struct au_opt *opt;

	AuTraceEnter();

	opt = opts->opt;
	while (opt->type != Opt_tail) {
		switch (opt->type) {
		case Opt_add:
		case Opt_append:
		case Opt_prepend:
			path_put(&opt->add.nd.path);
			break;
		case Opt_del:
		case Opt_idel:
			dput(opt->del.h_root);
			break;
		case Opt_mod:
		case Opt_imod:
			dput(opt->mod.h_root);
			break;
		case Opt_xino:
			fput(opt->xino.file);
			break;
		case Opt_xinodir:
			path_put(&opt->xinodir.path);
			break;
		}
		opt++;
	}
}

static int opt_add(struct au_opt *opt, char *opt_str, struct super_block *sb,
		   aufs_bindex_t bindex)
{
	int err;
	struct au_opt_add *add = &opt->add;
	char *p;

	LKTRTrace("%s, b%d\n", opt_str, bindex);

	add->bindex = bindex;
	add->perm = AuBrPerm_Last;
	add->path = opt_str;
	p = strchr(opt_str, '=');
	if (unlikely(p)) {
		*p++ = 0;
		if (*p)
			add->perm = br_perm_val(p);
	}

	/* LSM may detect it */
	/* do not superio. */
	err = vfsub_path_lookup(add->path, lkup_dirflags, &add->nd);
	if (!err) {
		if (!p /* && add->perm == AuBrPerm_Last */) {
			add->perm = AuBrPerm_RO;
			if (au_test_def_rr(add->nd.path.dentry->d_sb))
				add->perm = AuBrPerm_RR;
			if (!bindex && !(sb->s_flags & MS_RDONLY))
				add->perm = AuBrPerm_RW;
#ifdef CONFIG_AUFS_COMPAT
			add->perm = AuBrPerm_RW;
#endif
		}
		opt->type = Opt_add;
		goto out;
	}
	AuErr("lookup failed %s (%d)\n", add->path, err);
	err = -EINVAL;

 out:
	AuTraceErr(err);
	return err;
}

/* called without aufs lock */
int au_opts_parse(struct super_block *sb, unsigned long flags, char *str,
		  struct au_opts *opts)
{
	int err, n, token;
	struct dentry *root;
	struct au_opt *opt, *opt_tail;
	char *opt_str, *p;
	aufs_bindex_t bindex, bend;
	unsigned char skipped;
	union {
		struct au_opt_del *del;
		struct au_opt_mod *mod;
		struct au_opt_xino *xino;
		struct au_opt_xinodir *xinodir;
		struct au_opt_xino_itrunc *xino_itrunc;
		struct au_opt_wbr_create *create;
	} u;
	struct file *file;
	/* reduce the stack space */
	struct {
		substring_t args[MAX_OPT_ARGS];
		struct nameidata nd;
	} *a;

	LKTRTrace("%s, nopts %d\n", str, opts->max_opt);

	err = -ENOMEM;
	a = kmalloc(sizeof(*a), GFP_NOFS);
	if (unlikely(!a))
		goto out;

	root = sb->s_root;
	err = 0;
	bindex = 0;
	opt = opts->opt;
	opt_tail = opt + opts->max_opt - 1;
	opt->type = Opt_tail;
	while (!err && (opt_str = strsep(&str, ",")) && *opt_str) {
		err = -EINVAL;
		token = match_token(opt_str, options, a->args);
		LKTRTrace("%s, token %d, a->args[0]{%p, %p}\n",
			  opt_str, token, a->args[0].from, a->args[0].to);

		skipped = 0;
		switch (token) {
		case Opt_br:
			err = 0;
			while (!err && (opt_str = strsep(&a->args[0].from, ":"))
			       && *opt_str) {
				err = opt_add(opt, opt_str, sb, bindex++);
				if (unlikely(!err && ++opt > opt_tail)) {
					err = -E2BIG;
					break;
				}
				opt->type = Opt_tail;
				skipped = 1;
			}
			break;
		case Opt_add:
			if (unlikely(match_int(&a->args[0], &n))) {
				AuErr("bad integer in %s\n", opt_str);
				break;
			}
			bindex = n;
			err = opt_add(opt, a->args[1].from, sb, bindex);
			break;
		case Opt_append:
			err = opt_add(opt, a->args[0].from, sb,
				      /*dummy bindex*/1);
			if (!err)
				opt->type = token;
			break;
		case Opt_prepend:
			err = opt_add(opt, a->args[0].from, sb, /*bindex*/0);
			if (!err)
				opt->type = token;
			break;
		case Opt_del:
			u.del = &opt->del;
			u.del->path = a->args[0].from;
			LKTRTrace("del path %s\n", u.del->path);
			/* LSM may detect it */
			/* do not superio. */
			err = vfsub_path_lookup(u.del->path, lkup_dirflags,
						&a->nd);
			if (unlikely(err)) {
				AuErr("lookup failed %s (%d)\n",
				      u.del->path, err);
				break;
			}
			u.del->h_root = dget(a->nd.path.dentry);
			path_put(&a->nd.path);
			opt->type = token;
			break;
#if 0 /* reserved for future use */
		case Opt_idel:
			u.del = &opt->del;
			u.del->path = "(indexed)";
			if (unlikely(match_int(&a->args[0], &n))) {
				AuErr("bad integer in %s\n", opt_str);
				break;
			}
			bindex = n;
			aufs_read_lock(root, AuLock_FLUSH);
			if (bindex < 0 || au_sbend(sb) < bindex) {
				AuErr("out of bounds, %d\n", bindex);
				aufs_read_unlock(root, !AuLock_IR);
				break;
			}
			err = 0;
			u.del->h_root = dget(au_h_dptr(root, bindex));
			opt->type = token;
			aufs_read_unlock(root, !AuLock_IR);
			break;
#endif
		case Opt_mod:
			u.mod = &opt->mod;
			u.mod->path = a->args[0].from;
			p = strchr(u.mod->path, '=');
			if (unlikely(!p)) {
				AuErr("no permssion %s\n", opt_str);
				break;
			}
			*p++ = 0;
			u.mod->perm = br_perm_val(p);
			LKTRTrace("mod path %s, perm 0x%x, %s\n",
				  u.mod->path, u.mod->perm, p);
			/* LSM may detect it */
			/* do not superio. */
			err = vfsub_path_lookup(u.mod->path, lkup_dirflags,
						&a->nd);
			if (unlikely(err)) {
				AuErr("lookup failed %s (%d)\n",
				      u.mod->path, err);
				break;
			}
			u.mod->h_root = dget(a->nd.path.dentry);
			path_put(&a->nd.path);
			opt->type = token;
			break;
#ifdef IMOD /* reserved for future use */
		case Opt_imod:
			u.mod = &opt->mod;
			u.mod->path = "(indexed)";
			if (unlikely(match_int(&a->args[0], &n))) {
				AuErr("bad integer in %s\n", opt_str);
				break;
			}
			bindex = n;
			aufs_read_lock(root, AuLock_FLUSH);
			if (bindex < 0 || au_sbend(sb) < bindex) {
				AuErr("out of bounds, %d\n", bindex);
				aufs_read_unlock(root, !AuLock_IR);
				break;
			}
			u.mod->perm = br_perm_val(a->args[1].from);
			LKTRTrace("mod path %s, perm 0x%x, %s\n",
				  u.mod->path, u.mod->perm, a->args[1].from);
			err = 0;
			u.mod->h_root = dget(au_h_dptr(root, bindex));
			opt->type = token;
			aufs_read_unlock(root, !AuLock_IR);
			break;
#endif
		case Opt_xino:
			u.xino = &opt->xino;
			file = au_xino_create(sb, a->args[0].from, /*silent*/0);
			err = PTR_ERR(file);
			if (IS_ERR(file))
				break;
			err = -EINVAL;
			if (unlikely(file->f_dentry->d_sb == sb)) {
				fput(file);
				AuErr("%s must be outside\n", a->args[0].from);
				break;
			}
			err = 0;
			u.xino->file = file;
			u.xino->path = a->args[0].from;
			opt->type = token;
			break;

#if 0 /* def CONFIG_AUFS_EXPORT */ /* reserved for futur use */
		case Opt_xinodir:
			u.xinodir = &opt->xinodir;
			u.xinodir->name = a->args[0].from;
			err = vfsub_path_lookup(u.xinodir->name, lkup_dirflags,
						&a->nd);
			if (unlikely(err)) {
				AuErr("lookup failed %s (%d)\n",
				      u.xinodir->name, err);
				break;
			}
			u.xinodir->path = a->nd.path;
			/* do not path_put() */
			opt->type = token;
			break;
#endif

		case Opt_trunc_xino_path:
			u.xino_itrunc = &opt->xino_itrunc;
			p = a->args[0].from;
			LKTRTrace("trunc_xino path %s\n", p);
			/* LSM may detect it */
			/* do not superio. */
			err = vfsub_path_lookup(p, lkup_dirflags, &a->nd);
			if (unlikely(err)) {
				AuErr("lookup failed %s (%d)\n", p , err);
				break;
			}
			u.xino_itrunc->bindex = -1;
			aufs_read_lock(root, AuLock_FLUSH);
			bend = au_sbend(sb);
			for (bindex = 0; bindex <= bend; bindex++) {
				if (au_h_dptr(root, bindex)
				    == a->nd.path.dentry) {
					u.xino_itrunc->bindex = bindex;
					break;
				}
			}
			aufs_read_unlock(root, !AuLock_IR);
			path_put(&a->nd.path);
			if (unlikely(u.xino_itrunc->bindex < 0)) {
				AuErr("no such branch %s\n", p);
				err = -EINVAL;
				break;
			}
			opt->type = token;
			break;

		case Opt_itrunc_xino:
			u.xino_itrunc = &opt->xino_itrunc;
			if (unlikely(match_int(&a->args[0], &n))) {
				AuErr("bad integer in %s\n", opt_str);
				break;
			}
			u.xino_itrunc->bindex = n;
			aufs_read_lock(root, AuLock_FLUSH);
			if (n < 0 || au_sbend(sb) < n) {
				AuErr("out of bounds, %d\n", n);
				aufs_read_unlock(root, !AuLock_IR);
				break;
			}
			aufs_read_unlock(root, !AuLock_IR);
			err = 0;
			opt->type = token;
			break;

		case Opt_dirwh:
			if (unlikely(match_int(&a->args[0], &opt->dirwh)))
				break;
			err = 0;
			opt->type = token;
			break;

		case Opt_rdcache:
			if (unlikely(match_int(&a->args[0], &opt->rdcache)))
				break;
			err = 0;
			opt->type = token;
			break;

		case Opt_shwh:
			if (flags & MS_RDONLY) {
				err = 0;
				opt->type = token;
			} else
				AuErr("shwh requires ro\n");
			break;

		case Opt_trunc_xino:
		case Opt_notrunc_xino:
		case Opt_noxino:
		case Opt_trunc_xib:
		case Opt_notrunc_xib:
		case Opt_dirperm1:
		case Opt_nodirperm1:
		case Opt_noshwh:
		case Opt_plink:
		case Opt_noplink:
		case Opt_list_plink:
		case Opt_clean_plink:
		case Opt_diropq_a:
		case Opt_diropq_w:
		case Opt_warn_perm:
		case Opt_nowarn_perm:
		case Opt_dlgt:
		case Opt_nodlgt:
		case Opt_refrof:
		case Opt_norefrof:
		case Opt_verbose:
		case Opt_noverbose:
			err = 0;
			opt->type = token;
			break;

		case Opt_udba:
			opt->udba = udba_val(a->args[0].from);
			if (opt->udba >= 0) {
				err = 0;
				opt->type = token;
			} else
				AuErr("wrong value, %s\n", opt_str);
			break;

		case Opt_wbr_create:
			u.create = &opt->wbr_create;
			u.create->wbr_create
				= au_wbr_create_val(a->args[0].from, u.create);
			if (u.create->wbr_create >= 0) {
				err = 0;
				opt->type = token;
			} else
				AuErr("wrong value, %s\n", opt_str);
			break;
		case Opt_wbr_copyup:
			opt->wbr_copyup = au_wbr_copyup_val(a->args[0].from);
			if (opt->wbr_copyup >= 0) {
				err = 0;
				opt->type = token;
			} else
				AuErr("wrong value, %s\n", opt_str);
			break;

		case Opt_coo:
			opt->coo = coo_val(a->args[0].from);
			if (opt->coo >= 0) {
				err = 0;
				opt->type = token;
			} else
				AuErr("wrong value, %s\n", opt_str);
			break;

		case Opt_ignore:
#ifndef CONFIG_AUFS_COMPAT
			AuWarn("ignored %s\n", opt_str);
#endif
		case Opt_ignore_silent:
			skipped = 1;
			err = 0;
			break;
		case Opt_err:
			AuErr("unknown option %s\n", opt_str);
			break;
		}

		if (!err && !skipped) {
			if (unlikely(++opt > opt_tail)) {
				err = -E2BIG;
				opt--;
				opt->type = Opt_tail;
				break;
			}
			opt->type = Opt_tail;
		}
	}

	kfree(a);
	dump_opts(opts);
	if (unlikely(err))
		au_opts_free(opts);

 out:
	AuTraceErr(err);
	return err;
}

/*
 * returns,
 * plus: processed without an error
 * zero: unprocessed
 */
static int au_opt_simple(struct super_block *sb, struct au_opt *opt,
			 struct au_opts *opts)
{
	int err;
	struct au_sbinfo *sbinfo;
	struct au_opt_wbr_create *create;

	AuTraceEnter();

	err = 1; /* handled */
	sbinfo = au_sbi(sb);
	switch (opt->type) {
	case Opt_udba:
		sbinfo->si_mntflags &= ~AuOptMask_UDBA;
		sbinfo->si_mntflags |= opt->udba;
		opts->given_udba |= opt->udba;
		break;

	case Opt_plink:
		au_opt_set(sbinfo->si_mntflags, PLINK);
		break;
	case Opt_noplink:
		if (au_opt_test(sbinfo->si_mntflags, PLINK))
			au_plink_put(sb);
		au_opt_clr(sbinfo->si_mntflags, PLINK);
		break;
	case Opt_list_plink:
		if (au_opt_test(sbinfo->si_mntflags, PLINK))
			au_plink_list(sb);
		break;
	case Opt_clean_plink:
		if (au_opt_test(sbinfo->si_mntflags, PLINK))
			au_plink_put(sb);
		break;

	case Opt_diropq_a:
		au_opt_set(sbinfo->si_mntflags, ALWAYS_DIROPQ);
		break;
	case Opt_diropq_w:
		au_opt_clr(sbinfo->si_mntflags, ALWAYS_DIROPQ);
		break;

	case Opt_dlgt:
		au_opt_set(sbinfo->si_mntflags, DLGT);
		break;
	case Opt_nodlgt:
		au_opt_clr(sbinfo->si_mntflags, DLGT);
		break;

	case Opt_warn_perm:
		au_opt_set(sbinfo->si_mntflags, WARN_PERM);
		break;
	case Opt_nowarn_perm:
		au_opt_clr(sbinfo->si_mntflags, WARN_PERM);
		break;

	case Opt_refrof:
		au_opt_set(sbinfo->si_mntflags, REFROF);
		break;
	case Opt_norefrof:
		/* au_opt_set(sbinfo->si_mntflags, COO_LEAF); */
		au_opt_clr(sbinfo->si_mntflags, REFROF);
		break;

	case Opt_verbose:
		au_opt_set(sbinfo->si_mntflags, VERBOSE);
		break;
	case Opt_noverbose:
		au_opt_clr(sbinfo->si_mntflags, VERBOSE);
		break;

	case Opt_wbr_create:
		create = &opt->wbr_create;
		if (sbinfo->si_wbr_create_ops->fin) {
			err = sbinfo->si_wbr_create_ops->fin(sb);
			if (!err)
				err = 1;
		}
		sbinfo->si_wbr_create = create->wbr_create;
		sbinfo->si_wbr_create_ops
			= au_wbr_create_ops + create->wbr_create;
		switch (create->wbr_create) {
		case AuWbrCreate_MFSRRV:
		case AuWbrCreate_MFSRR:
			sbinfo->si_wbr_mfs.mfsrr_watermark
				= create->mfsrr_watermark;
			/*FALLTHROUGH*/
		case AuWbrCreate_MFS:
		case AuWbrCreate_MFSV:
		case AuWbrCreate_PMFS:
		case AuWbrCreate_PMFSV:
			sbinfo->si_wbr_mfs.mfs_expire = create->mfs_second * HZ;
			break;
		}
		if (sbinfo->si_wbr_create_ops->init)
			sbinfo->si_wbr_create_ops->init(sb); /* ignore */
		break;
	case Opt_wbr_copyup:
		sbinfo->si_wbr_copyup = opt->wbr_copyup;
		sbinfo->si_wbr_copyup_ops = au_wbr_copyup_ops + opt->wbr_copyup;
		break;

	case Opt_coo:
		sbinfo->si_mntflags &= ~AuOptMask_COO;
		sbinfo->si_mntflags |= opt->coo;
		break;

	case Opt_dirwh:
		sbinfo->si_dirwh = opt->dirwh;
		break;

	case Opt_rdcache:
		sbinfo->si_rdcache = opt->rdcache * HZ;
		break;

	case Opt_trunc_xino:
		au_opt_set(sbinfo->si_mntflags, TRUNC_XINO);
		break;
	case Opt_notrunc_xino:
		au_opt_clr(sbinfo->si_mntflags, TRUNC_XINO);
		break;

	case Opt_dirperm1:
		au_opt_set(sbinfo->si_mntflags, DIRPERM1);
		break;
	case Opt_nodirperm1:
		au_opt_clr(sbinfo->si_mntflags, DIRPERM1);
		break;

	case Opt_shwh:
		au_opt_set(sbinfo->si_mntflags, SHWH);
		break;
	case Opt_noshwh:
		au_opt_clr(sbinfo->si_mntflags, SHWH);
		break;

	case Opt_trunc_xino_path:
	case Opt_itrunc_xino:
		err = au_xino_trunc(sb, opt->xino_itrunc.bindex);
		if (!err)
			err = 1;
		break;

	case Opt_trunc_xib:
		au_fset_opts(opts->flags, TRUNC_XIB);
		break;
	case Opt_notrunc_xib:
		au_fclr_opts(opts->flags, TRUNC_XIB);
		break;

	default:
		err = 0;
		break;
	}

	AuTraceErr(err);
	return err;
}

/*
 * returns tri-state.
 * plus: processed without an error
 * zero: unprocessed
 * minus: error
 */
static int au_opt_br(struct super_block *sb, struct au_opt *opt,
		     struct au_opts *opts)
{
	int err, do_refresh;

	AuTraceEnter();

	err = 0;
	switch (opt->type) {
	case Opt_append:
		opt->add.bindex = au_sbend(sb) + 1;
		if (unlikely(opt->add.bindex < 0))
			opt->add.bindex = 0;
		goto add;
	case Opt_prepend:
		opt->add.bindex = 0;
	add:
	case Opt_add:
		err = au_br_add(sb, &opt->add,
				au_ftest_opts(opts->flags, REMOUNT));
		if (!err) {
			err = 1;
			au_fset_opts(opts->flags, REFRESH_DIR);
			if (unlikely(au_br_whable(opt->add.perm)))
				au_fset_opts(opts->flags, REFRESH_NONDIR);
		}
		break;

	case Opt_del:
	case Opt_idel:
		err = au_br_del(sb, &opt->del,
				au_ftest_opts(opts->flags, REMOUNT));
		if (!err) {
			err = 1;
			au_fset_opts(opts->flags, TRUNC_XIB);
			au_fset_opts(opts->flags, REFRESH_DIR);
			au_fset_opts(opts->flags, REFRESH_NONDIR);
		}
		break;

	case Opt_mod:
	case Opt_imod:
		err = au_br_mod(sb, &opt->mod,
				au_ftest_opts(opts->flags, REMOUNT),
				&do_refresh);
		if (!err) {
			err = 1;
			if (unlikely(do_refresh)) {
				au_fset_opts(opts->flags, REFRESH_DIR);
				au_fset_opts(opts->flags, REFRESH_NONDIR);
			}
		}
		break;
	}

	AuTraceErr(err);
	return err;
}

static int au_opt_xino(struct super_block *sb, struct au_opt *opt,
		       struct au_opt_xino **opt_xino,
		       struct au_opt_xinodir **opt_xinodir,
		       struct au_opts *opts)
{
	int err;
	const int remount = !!au_ftest_opts(opts->flags, REMOUNT);

	AuTraceEnter();

	err = 0;
	switch (opt->type) {
	case Opt_xino:
		err = au_xino_set(sb, &opt->xino, remount);
		if (!err)
			*opt_xino = &opt->xino;
		break;
#if 0 /* def CONFIG_AUFS_EXPORT */ /* reserved for futur use */
	case Opt_xinodir:
		err = au_xinodir_set(sb, &opt->xinodir, remount);
		if (!err)
			*opt_xinodir = &opt->xinodir;
		break;
#endif
	case Opt_noxino:
		au_xino_clr(sb);
		*opt_xino = (void *)-1;
		break;
	}

	AuTraceErr(err);
	return err;
}

static int verify_opts(struct super_block *sb, unsigned int pending,
		       int remount)
{
	int err;
	aufs_bindex_t bindex, bend;
	unsigned char do_plink, skip, do_free;
	struct au_branch *br;
	struct au_wbr *wbr;
	struct dentry *root;
	struct inode *dir, *h_dir;
	unsigned int mnt_flags;

	AuTraceEnter();
	mnt_flags = au_mntflags(sb);
	AuDebugOn(!(mnt_flags & AuOptMask_COO));
	AuDebugOn(!(mnt_flags & AuOptMask_UDBA));

	if (!(sb->s_flags & MS_RDONLY)) {
		if (unlikely(!au_br_writable(au_sbr_perm(sb, 0))))
			AuWarn("first branch should be rw\n");
		if (unlikely(au_opt_test(mnt_flags, SHWH)))
			AuWarn("shwh should be used with ro\n");
	}

	if (unlikely(au_opt_test((mnt_flags | pending), UDBA_INOTIFY)
		     && !au_opt_test_xino(mnt_flags)))
		AuWarn("udba=inotify requires xino\n");

	err = 0;
	root = sb->s_root;
	dir = sb->s_root->d_inode;
	do_plink = !!au_opt_test(mnt_flags, PLINK);
	bend = au_sbend(sb);
	for (bindex = 0; !err && bindex <= bend; bindex++) {
		skip = 0;
		h_dir = au_h_iptr(dir, bindex);
		br = au_sbr(sb, bindex);
		do_free = 0;
		wbr = br->br_wbr;
		if (wbr)
			wbr_wh_read_lock(wbr);
		switch (br->br_perm) {
		case AuBrPerm_RR:
		case AuBrPerm_RO:
		case AuBrPerm_RRWH:
		case AuBrPerm_ROWH:
			do_free = !!wbr;
			skip = (!wbr
				|| (!wbr->wbr_whbase
				    && !wbr->wbr_plink
				    && !wbr->wbr_tmp));
			break;

		case AuBrPerm_RWNoLinkWH:
			/* skip = (!br->br_whbase && !br->br_tmp); */
			skip = (!wbr || !wbr->wbr_whbase);
			if (skip && wbr) {
				if (do_plink)
					skip = !!wbr->wbr_plink;
				else
					skip = !wbr->wbr_plink;
			}
			break;

		case AuBrPerm_RW:
			/* skip = (br->br_whbase && br->br_tmp); */
			skip = (wbr && wbr->wbr_whbase);
			if (skip) {
				if (do_plink)
					skip = !!wbr->wbr_plink;
				else
					skip = !wbr->wbr_plink;
			}
			break;

		default:
			BUG();
		}
		if (wbr)
			wbr_wh_read_unlock(wbr);

		if (skip)
			continue;

		mutex_lock_nested(&h_dir->i_mutex, AuLsc_I_PARENT);
		if (wbr)
			wbr_wh_write_lock(wbr);
		err = au_wh_init(au_h_dptr(root, bindex), br,
				 au_nfsmnt(sb, bindex), sb, bindex);
		if (wbr)
			wbr_wh_write_unlock(wbr);
		mutex_unlock(&h_dir->i_mutex);

		if (!err && do_free) {
			kfree(wbr);
			br->br_wbr = NULL;
		}
	}

	AuTraceErr(err);
	return err;
}

int au_opts_mount(struct super_block *sb, struct au_opts *opts)
{
	int err;
	struct inode *dir;
	struct au_opt *opt;
	struct au_opt_xino *opt_xino, xino;
	struct au_opt_xinodir *opt_xinodir;
	aufs_bindex_t bend;
	struct au_sbinfo *sbinfo;
	unsigned int tmp;
	struct au_branch *br;

	AuTraceEnter();
	SiMustWriteLock(sb);
	DiMustWriteLock(sb->s_root);
	dir = sb->s_root->d_inode;
	IiMustWriteLock(dir);

	err = 0;
	opt_xino = NULL;
	opt_xinodir = NULL;
	opt = opts->opt;
	while (err >= 0 && opt->type != Opt_tail)
		err = au_opt_simple(sb, opt++, opts);
	if (err > 0)
		err = 0;
	else if (unlikely(err < 0))
		goto out;

	/* disable xino, xinodir, hinotify, dlgt temporary */
	sbinfo = au_sbi(sb);
	tmp = sbinfo->si_mntflags;
	au_opt_clr(sbinfo->si_mntflags, XINO);
	au_opt_clr(sbinfo->si_mntflags, XINODIR);
	au_opt_clr(sbinfo->si_mntflags, DLGT);
	au_opt_set_udba(sbinfo->si_mntflags, UDBA_REVAL);

	opt = opts->opt;
	while (err >= 0 && opt->type != Opt_tail)
		err = au_opt_br(sb, opt++, opts);
	if (err > 0)
		err = 0;
	else if (unlikely(err < 0))
		goto out;

	bend = au_sbend(sb);
	if (unlikely(bend < 0)) {
		err = -EINVAL;
		AuErr("no branches\n");
		goto out;
	}

	if (au_opt_test(tmp, XINO))
		au_opt_set(sbinfo->si_mntflags, XINO);
	else if (au_opt_test(tmp, XINODIR))
		au_opt_set(sbinfo->si_mntflags, XINODIR);
	opt = opts->opt;
	while (!err && opt->type != Opt_tail)
		err = au_opt_xino(sb, opt++, &opt_xino, &opt_xinodir, opts);
	if (unlikely(err))
		goto out;

	/* todo: test this error case? */
	err = verify_opts(sb, tmp, /*remount*/0);
	if (unlikely(err))
		goto out;

	/* enable xino */
	if (au_opt_test(tmp, XINO) && !opt_xino) {
		xino.file = au_xino_def(sb);
		err = PTR_ERR(xino.file);
		if (IS_ERR(xino.file))
			goto out;

		br = au_xino_def_br(sbinfo);
		err = au_xino_set(sb, &xino, /*remount*/0);
		fput(xino.file);
		if (unlikely(err))
			goto out;
		au_xino_def_br_set(br, sbinfo);
	}

	/* restore hinotify */
	sbinfo->si_mntflags &= ~AuOptMask_UDBA;
	sbinfo->si_mntflags |= (tmp & AuOptMask_UDBA);
	if (au_opt_test(tmp, UDBA_INOTIFY))
		au_reset_hinotify(dir, au_hi_flags(dir, 1) & ~AuHi_XINO);

	/* restore dlgt */
	if (au_opt_test(tmp, DLGT))
		au_opt_set(sbinfo->si_mntflags, DLGT);

 out:
	AuTraceErr(err);
	return err;
}

int au_opts_remount(struct super_block *sb, struct au_opts *opts)
{
	int err, rerr;
	struct inode *dir;
	struct au_opt_xino *opt_xino;
	struct au_opt_xinodir *opt_xinodir;
	struct au_opt *opt;
	unsigned char dlgt;
	struct au_sbinfo *sbinfo;

	AuTraceEnter();
	SiMustWriteLock(sb);
	DiMustWriteLock(sb->s_root);
	dir = sb->s_root->d_inode;
	IiMustWriteLock(dir);
	sbinfo = au_sbi(sb);

	err = 0;
	dlgt = !!au_opt_test(sbinfo->si_mntflags, DLGT);
	opt_xino = NULL;
	opt_xinodir = NULL;
	opt = opts->opt;
	while (err >= 0 && opt->type != Opt_tail) {
		err = au_opt_simple(sb, opt, opts);

		/* disable it temporary */
		dlgt = !!au_opt_test(sbinfo->si_mntflags, DLGT);
		au_opt_clr(sbinfo->si_mntflags, DLGT);

		if (!err)
			err = au_opt_br(sb, opt, opts);
		if (!err)
			err = au_opt_xino(sb, opt, &opt_xino, &opt_xinodir,
					  opts);

		/* restore it */
		if (unlikely(dlgt))
			au_opt_set(sbinfo->si_mntflags, DLGT);
		opt++;
	}
	if (err > 0)
		err = 0;
	AuTraceErr(err);

	/* go on even err */

	/* todo: test this error case? */
	au_opt_clr(sbinfo->si_mntflags, DLGT);
	rerr = verify_opts(sb, sbinfo->si_mntflags, /*remount*/1);
	if (unlikely(dlgt))
		au_opt_set(sbinfo->si_mntflags, DLGT);
	if (unlikely(rerr && !err))
		err = rerr;

	if (unlikely(au_ftest_opts(opts->flags, TRUNC_XIB))) {
		rerr = au_xib_trunc(sb);
		if (unlikely(rerr && !err))
			err = rerr;
	}

	/* they are handled by the caller */
	if (!au_ftest_opts(opts->flags, REFRESH_DIR)
	    && (opts->given_udba || au_opt_test_xino(sbinfo->si_mntflags)))
		au_fset_opts(opts->flags, REFRESH_DIR);

	LKTRTrace("status 0x%x\n", opts->flags);
	AuTraceErr(err);
	return err;
}
