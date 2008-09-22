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
 * sub-routines for dentry cache
 *
 * $Id: dcsub.c,v 1.7 2008/07/21 02:54:22 sfjro Exp $
 */

#include "aufs.h"

static void au_dpage_free(struct au_dpage *dpage)
{
	int i;

	AuTraceEnter();
	AuDebugOn(!dpage);

	for (i = 0; i < dpage->ndentry; i++)
		dput(dpage->dentries[i]);
	free_page((unsigned long)dpage->dentries);
}

int au_dpages_init(struct au_dcsub_pages *dpages, gfp_t gfp)
{
	int err;
	void *p;

	AuTraceEnter();

	err = -ENOMEM;
	dpages->dpages = kmalloc(sizeof(*dpages->dpages), gfp);
	if (unlikely(!dpages->dpages))
		goto out;
	p = (void *)__get_free_page(gfp);
	if (unlikely(!p))
		goto out_dpages;
	dpages->dpages[0].ndentry = 0;
	dpages->dpages[0].dentries = p;
	dpages->ndpage = 1;
	return 0; /* success */

 out_dpages:
	kfree(dpages->dpages);
 out:
	AuTraceErr(err);
	return err;
}

void au_dpages_free(struct au_dcsub_pages *dpages)
{
	int i;

	AuTraceEnter();

	for (i = 0; i < dpages->ndpage; i++)
		au_dpage_free(dpages->dpages + i);
	kfree(dpages->dpages);
}

static int au_dpages_append(struct au_dcsub_pages *dpages,
			    struct dentry *dentry, gfp_t gfp)
{
	int err, sz;
	struct au_dpage *dpage;
	void *p;

	/* AuTraceEnter(); */

	dpage = dpages->dpages + dpages->ndpage - 1;
	AuDebugOn(!dpage);
	sz = PAGE_SIZE / sizeof(dentry);
	if (unlikely(dpage->ndentry >= sz)) {
		LKTRLabel(new dpage);
		err = -ENOMEM;
		sz = dpages->ndpage * sizeof(*dpages->dpages);
		p = au_kzrealloc(dpages->dpages, sz,
				 sz + sizeof(*dpages->dpages), gfp);
		if (unlikely(!p))
			goto out;
		dpages->dpages = p;
		dpage = dpages->dpages + dpages->ndpage;
		p = (void *)__get_free_page(gfp);
		if (unlikely(!p))
			goto out;
		dpage->ndentry = 0;
		dpage->dentries = p;
		dpages->ndpage++;
	}

	dpage->dentries[dpage->ndentry++] = dget(dentry);
	return 0; /* success */

 out:
	/* AuTraceErr(err); */
	return err;
}

int au_dcsub_pages(struct au_dcsub_pages *dpages, struct dentry *root,
		   au_dpages_test test, void *arg)
{
	int err;
	struct dentry *this_parent = root;
	struct list_head *next;
	struct super_block *sb = root->d_sb;

	AuTraceEnter();

	err = 0;
	spin_lock(&dcache_lock);
 repeat:
	next = this_parent->d_subdirs.next;
 resume:
	if (this_parent->d_sb == sb
	    && !IS_ROOT(this_parent)
	    && atomic_read(&this_parent->d_count)
	    && this_parent->d_inode
	    && (!test || test(this_parent, arg))) {
		err = au_dpages_append(dpages, this_parent, GFP_ATOMIC);
		if (unlikely(err))
			goto out;
	}

	while (next != &this_parent->d_subdirs) {
		struct list_head *tmp = next;
		struct dentry *dentry = list_entry(tmp, struct dentry,
						   d_u.d_child);
		next = tmp->next;
		if (unlikely(/*d_unhashed(dentry) || */!dentry->d_inode))
			continue;
		if (!list_empty(&dentry->d_subdirs)) {
			this_parent = dentry;
			goto repeat;
		}
		if (dentry->d_sb == sb
		    && atomic_read(&dentry->d_count)
		    && (!test || test(dentry, arg))) {
			err = au_dpages_append(dpages, dentry, GFP_ATOMIC);
			if (unlikely(err))
				goto out;
		}
	}

	if (this_parent != root) {
		next = this_parent->d_u.d_child.next;
		this_parent = this_parent->d_parent; /* dcache_lock is locked */
		goto resume;
	}
 out:
	spin_unlock(&dcache_lock);
#if 0 /* debug */
	if (!err) {
		int i, j;
		j = 0;
		for (i = 0; i < dpages->ndpage; i++) {
			if ((dpages->dpages + i)->ndentry)
				AuDbg("%d: %d\n",
				      i, (dpages->dpages + i)->ndentry);
			j += (dpages->dpages + i)->ndentry;
		}
		if (j)
			AuDbg("ndpage %d, %d\n", dpages->ndpage, j);
	}
#endif
	AuTraceErr(err);
	return err;
}

int au_dcsub_pages_rev(struct au_dcsub_pages *dpages, struct dentry *dentry,
		       int do_include, au_dpages_test test, void *arg)
{
	int err;

	AuTraceEnter();

	err = 0;
	spin_lock(&dcache_lock);
	if (do_include && (!test || test(dentry, arg))) {
		err = au_dpages_append(dpages, dentry, GFP_ATOMIC);
		if (unlikely(err))
			goto out;
	}
	while (!IS_ROOT(dentry)) {
		dentry = dentry->d_parent; /* dcache_lock is locked */
		if (!test || test(dentry, arg)) {
			err = au_dpages_append(dpages, dentry, GFP_ATOMIC);
			if (unlikely(err))
				break;
		}
	}

 out:
	spin_unlock(&dcache_lock);

	AuTraceErr(err);
	return err;
}

struct dentry *au_test_subdir(struct dentry *d1, struct dentry *d2)
{
	struct dentry *trap, **dentries;
	int err, i, j;
	struct au_dcsub_pages dpages;
	struct au_dpage *dpage;

	LKTRTrace("%.*s, %.*s\n", AuDLNPair(d1), AuDLNPair(d2));

	trap = ERR_PTR(-ENOMEM);
	err = au_dpages_init(&dpages, GFP_NOFS);
	if (unlikely(err))
		goto out;
	err = au_dcsub_pages_rev(&dpages, d1, /*do_include*/1, NULL, NULL);
	if (unlikely(err))
		goto out_dpages;

	trap = d1;
	for (i = 0; !err && i < dpages.ndpage; i++) {
		dpage = dpages.dpages + i;
		dentries = dpage->dentries;
		for (j = 0; !err && j < dpage->ndentry; j++) {
			struct dentry *d;
			d = dentries[j];
			err = (d == d2);
			if (!err)
				trap = d;
		}
	}
	if (!err)
		trap = NULL;

 out_dpages:
	au_dpages_free(&dpages);
 out:
	AuTraceErrPtr(trap);
	return trap;
}
