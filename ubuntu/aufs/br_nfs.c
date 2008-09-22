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
 * lookup functions for NFS branch in linux-2.6.19 and later
 *
 * $Id: br_nfs.c,v 1.7 2008/07/21 02:54:22 sfjro Exp $
 */

#include "aufs.h"

static struct file *au_find_h_intent(struct au_hdentry *hd, struct file *file)
{
	struct file *h_file, *hf;
	struct au_hdintent *hdi, *tmp, *do_free;

	LKTRTrace("%.*s\n", AuDLNPair(hd->hd_dentry));

	h_file = NULL;
	do_free = NULL;
	spin_lock(&hd->hd_lock);
	list_for_each_entry_safe(hdi, tmp, hd->hd_intent_list, hdi_list) {
		hf = hdi->hdi_file[AuIntent_BRANCH];
		if (hdi->hdi_file[AuIntent_AUFS] == file
		    && hf->f_dentry == hd->hd_dentry) {
			h_file = hf;
			do_free = hdi;
			list_del(&hdi->hdi_list);
			break;
		}
	}
	spin_unlock(&hd->hd_lock);
	kfree(do_free);

	return h_file;
}

struct file *au_h_intent(struct dentry *dentry, aufs_bindex_t bindex,
			 struct file *file)
{
	struct file *h_file;
	struct au_hdentry *hd = au_di(dentry)->di_hdentry + bindex;

	LKTRTrace("%.*s, b%d, f %p\n", AuDLNPair(dentry), bindex, file);
	DiMustAnyLock(dentry);
	AuDebugOn(bindex < au_di(dentry)->di_bstart
		  || bindex > au_di(dentry)->di_bend);

	h_file = NULL;
	if (!hd->hd_intent_list || !file)
		return h_file; /* success */

	/* AuDebugOn(au_test_wkq(current)); */
	h_file = au_find_h_intent(hd, file);
	return h_file;
}

static int au_set_h_intent(struct dentry *dentry, aufs_bindex_t bindex,
			   struct file *file, struct file *h_file)
{
	int err;
	struct au_hdentry *hd = au_di(dentry)->di_hdentry + bindex;
	struct au_hdintent *hdi;
	struct file *hf;

	LKTRTrace("%.*s, b%d, f %p\n", AuDLNPair(dentry), bindex, file);
	/* d_revalidate() holds read_lock */
	/* DiMustWriteLock(dentry); */
	AuDebugOn(bindex < au_di(dentry)->di_bstart
		  || bindex > au_di(dentry)->di_bend
		  || !file
		  || !h_file
		  /* || au_test_wkq(current) */);

	err = -ENOMEM;
	if (hd->hd_intent_list) {
		while (1) {
			hf = au_find_h_intent(hd, file);
			if (!hf)
				break;
			fput(hf);
			AuWarn("freed hfile %.*s b%d left\n",
			       AuDLNPair(dentry), bindex);
		}
	} else {
		spin_lock(&hd->hd_lock);
		if (!hd->hd_intent_list) {
			hd->hd_intent_list
				= kmalloc(sizeof(*hd->hd_intent_list),
					  GFP_ATOMIC);
			if (unlikely(!hd->hd_intent_list)) {
				spin_unlock(&hd->hd_lock);
				goto out;
			}
			INIT_LIST_HEAD(hd->hd_intent_list);
		}
		spin_unlock(&hd->hd_lock);
	}

	hdi = kmalloc(sizeof(*hdi), GFP_NOFS);
	if (unlikely(!hdi))
		goto out;

	err = 0;
	/* hdi->hdi_pid = current->pid; */
	hdi->hdi_file[AuIntent_AUFS] = file;
	hdi->hdi_file[AuIntent_BRANCH] = h_file;
	spin_lock(&hd->hd_lock);
	list_add(&hdi->hdi_list, hd->hd_intent_list);
	spin_unlock(&hd->hd_lock);

 out:
	AuTraceErr(err);
	return err;
}

int au_br_nfs_h_intent(struct file *nd_file, struct dentry *dentry,
		       aufs_bindex_t bindex, struct nameidata *nd)
{
	int err;

	AuTraceEnter();

	err = 0;
	if (!nd_file)
		goto out;

	AuDebugOn(!nd);
	err = au_set_h_intent(dentry, bindex, nd->intent.open.file, nd_file);
	if (unlikely(err)) {
		fput(nd_file);
		au_set_h_dptr(dentry, bindex, NULL);
		/* todo: update bstart and bend? */
	}

 out:
	AuTraceErr(err);
	return err;
}

/* ---------------------------------------------------------------------- */

void au_hintent_put(struct au_hdentry *hd, int do_free)
{
	struct au_hdintent *hdi, *tmp;
	struct file *hf;

	if (unlikely(hd->hd_intent_list)) {
		/* no spin lock */
		list_for_each_entry_safe(hdi, tmp, hd->hd_intent_list,
					 hdi_list) {
			LKTRTrace("hdi %p\n", hdi);
			hf = hdi->hdi_file[AuIntent_BRANCH];
			if (unlikely(hf))
				fput(hf);
			/* list_del(&hdi->hdi_list); */
			kfree(hdi);
		}
		if (do_free)
			kfree(hd->hd_intent_list);
	}
}

/* ---------------------------------------------------------------------- */

int au_fake_intent(/* struct au_ndsub *save,  */struct nameidata *nd,
		   int perm)
{
	int err;

	LKTRTrace("perm %d\n", perm);

	err = 0;
	nd->intent.open.file = NULL;
	if (nd->flags & LOOKUP_OPEN) {
		err = -ENFILE;
		nd->intent.open.file = get_empty_filp();
		if (unlikely(!nd->intent.open.file))
			goto out;

		err = 0;
		if (!au_br_writable(perm)) {
			nd->intent.open.flags = FMODE_READ
				| au_file_roflags(nd->intent.open.flags);
			nd->flags &= ~LOOKUP_CREATE;
		}
	}

 out:
	AuTraceErr(err);
	return err;
}

int au_hin_after_reval(struct nameidata *nd, struct dentry *dentry,
		       aufs_bindex_t bindex, struct file *file)
{
	int err;

	LKTRTrace("nd %p, %.*s, b%d, f %d\n",
		  nd, AuDLNPair(dentry), bindex, !!file);

	err = 0;
	if ((nd->flags & LOOKUP_OPEN)
	    && nd->intent.open.file
	    && !IS_ERR(nd->intent.open.file)) {
		if (nd->intent.open.file->f_dentry) {
			err = au_set_h_intent(dentry, bindex, file,
					      nd->intent.open.file);
			if (!err)
				nd->intent.open.file = NULL;
		}
		if (unlikely(nd->intent.open.file))
			put_filp(nd->intent.open.file);
	}

	return err;
}

#ifdef CONFIG_AUFS_DLGT
struct au_lookup_hash_args {
	struct dentry **errp;
	struct qstr *name;
	struct dentry *base;
	struct nameidata *nd;
};

static void au_call_lookup_hash(void *args)
{
	struct au_lookup_hash_args *a = args;
	*a->errp = vfsub__lookup_hash(a->name, a->base, a->nd);
}

static struct dentry *
au_lkup_hash_dlgt(struct qstr *this, struct dentry *parent,
		  struct nameidata *nd, unsigned int flags)
{
	struct dentry *dentry;
	int dirperm1;

	dirperm1 = au_ftest_ndx(flags, DIRPERM1);
	if (!dirperm1 && !au_ftest_ndx(flags, DLGT))
		dentry = vfsub__lookup_hash(this, parent, nd);
	else {
		int wkq_err;
		struct au_lookup_hash_args args = {
			.errp	= &dentry,
			.name	= this,
			.base	= parent,
			.nd	= nd
		};
		wkq_err = au_wkq_wait(au_call_lookup_hash, &args,
				      /*dlgt*/!dirperm1);
		if (unlikely(wkq_err))
			dentry = ERR_PTR(wkq_err);
	}

	AuTraceErrPtr(dentry);
	return dentry;
}
#else
static struct dentry *
au_lkup_hash_dlgt(struct qstr *this, struct dentry *parent,
		  struct nameidata *nd, unsigned int flags)
{
	return vfsub__lookup_hash(this, parent, nd);
}
#endif /* CONFIG_AUFS_DLGT */

struct dentry *au_lkup_hash(const char *name, struct dentry *parent,
			    int len, struct au_ndx *ndx)
{
	struct dentry *dentry;
	char *p;
	unsigned long hash;
	struct qstr this;
	unsigned int c;
	struct nameidata tmp_nd, *ndo;
	int err;

	LKTRTrace("%.*s/%.*s\n", AuDLNPair(parent), len, name);

	/* todo: export and call __lookup_one_len() in fs/namei.c? */
	dentry = ERR_PTR(-EACCES);
	this.name = name;
	this.len = len;
	if (unlikely(!len))
		goto out;

	p = (void *)name;
	hash = init_name_hash();
	while (len--) {
		c = *p++;
		if (unlikely(c == '/' || c == '\0'))
			goto out;
		hash = partial_name_hash(c, hash);
	}
	this.hash = end_name_hash(hash);

	ndo = ndx->nd;
	if (ndo) {
		tmp_nd = *ndo;
		err = au_fake_intent(&tmp_nd, ndx->br->br_perm);
		dentry = ERR_PTR(err);
		if (unlikely(err))
			goto out_intent;
	} else
		memset(&tmp_nd, 0, sizeof(tmp_nd));

	tmp_nd.path.dentry = parent;
	tmp_nd.path.mnt = ndx->nfsmnt;
	path_get(&tmp_nd.path);
	dentry = au_lkup_hash_dlgt(&this, parent, &tmp_nd, ndx->flags);
	if (!IS_ERR(dentry)) {
		/* why negative dentry for a new dir was unhashed? */
		if (unlikely(d_unhashed(dentry)))
			d_rehash(dentry);
		if (tmp_nd.intent.open.file
		    && tmp_nd.intent.open.file->f_dentry) {
			ndx->nd_file = tmp_nd.intent.open.file;
			tmp_nd.intent.open.file = NULL;
			/* au_br_get(ndx->br); */
		}
	}
	path_put(&tmp_nd.path);

 out_intent:
	if (tmp_nd.intent.open.file)
		put_filp(tmp_nd.intent.open.file);
 out:
	AuTraceErrPtr(dentry);
	return dentry;
}
