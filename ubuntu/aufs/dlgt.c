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
 * lookup functions in 'delegate' mode
 *
 * $Id: dlgt.c,v 1.5 2008/08/11 02:50:34 sfjro Exp $
 */

#include "aufs.h"

/* ---------------------------------------------------------------------- */

struct au_lookup_one_len_args {
	struct dentry **errp;
	const char *name;
	struct dentry *parent;
	int len;
};

static void au_call_lookup_one_len(void *args)
{
	struct au_lookup_one_len_args *a = args;
	*a->errp = vfsub_lookup_one_len(a->name, a->parent, a->len);
}

struct dentry *au_lkup_one_dlgt(const char *name, struct dentry *parent,
				int len, unsigned int flags)
{
	struct dentry *dentry;
	int dirperm1;

	LKTRTrace("%.*s/%.*s, 0x%x\n", AuDLNPair(parent), len, name, flags);

	dirperm1 = au_ftest_ndx(flags, DIRPERM1);
	if (!dirperm1 && !au_ftest_ndx(flags, DLGT))
		dentry = vfsub_lookup_one_len(name, parent, len);
	else {
		int wkq_err;
		struct au_lookup_one_len_args args = {
			.errp	= &dentry,
			.name	= name,
			.parent	= parent,
			.len	= len
		};
		wkq_err = au_wkq_wait(au_call_lookup_one_len, &args,
				      /*dlgt*/!dirperm1);
		if (unlikely(wkq_err))
			dentry = ERR_PTR(wkq_err);
	}

	AuTraceErrPtr(dentry);
	return dentry;
}

/* ---------------------------------------------------------------------- */

struct security_inode_permission_args {
	int *errp;
	struct inode *h_inode;
	int mask;
	struct nameidata *fake_nd;
};

static void call_security_inode_permission(void *args)
{
	struct security_inode_permission_args *a = args;
	LKTRTrace("fsuid %d\n", current->fsuid);
	*a->errp = vfsub_security_inode_permission(a->h_inode, a->mask,
						   a->fake_nd);
}

int au_security_inode_permission(struct inode *h_inode, int mask,
				 struct nameidata *fake_nd, int dlgt)
{
	int err;

	AuTraceEnter();

	if (!dlgt)
		err = vfsub_security_inode_permission(h_inode, mask, fake_nd);
	else {
		int wkq_err;
		struct security_inode_permission_args args = {
			.errp		= &err,
			.h_inode	= h_inode,
			.mask		= mask,
			.fake_nd	= fake_nd
		};
		wkq_err = au_wkq_wait(call_security_inode_permission, &args,
				      /*dlgt*/1);
		if (unlikely(wkq_err))
			err = wkq_err;
	}

	AuTraceErr(err);
	return err;
}
