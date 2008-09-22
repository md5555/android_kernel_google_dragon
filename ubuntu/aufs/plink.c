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
 * pseudo-link
 *
 * $Id: plink.c,v 1.9 2008/09/01 02:55:35 sfjro Exp $
 */

#include "aufs.h"

struct pseudo_link {
	struct list_head list;
	struct inode *inode;
};

#ifdef CONFIG_AUFS_DEBUG
void au_plink_list(struct super_block *sb)
{
	struct au_sbinfo *sbinfo;
	struct list_head *plink_list;
	struct pseudo_link *plink;

	AuTraceEnter();
	SiMustAnyLock(sb);
	sbinfo = au_sbi(sb);
	AuDebugOn(!au_opt_test(au_mntflags(sb), PLINK));

	plink_list = &sbinfo->si_plink;
	spin_lock(&sbinfo->si_plink_lock);
	list_for_each_entry(plink, plink_list, list)
		AuDbg("%lu\n", plink->inode->i_ino);
	spin_unlock(&sbinfo->si_plink_lock);
}
#endif

int au_plink_test(struct super_block *sb, struct inode *inode)
{
	int found;
	struct au_sbinfo *sbinfo;
	struct list_head *plink_list;
	struct pseudo_link *plink;

	LKTRTrace("i%lu\n", inode->i_ino);
	SiMustAnyLock(sb);
	sbinfo = au_sbi(sb);
	AuDebugOn(!au_opt_test(au_mntflags(sb), PLINK));

	found = 0;
	plink_list = &sbinfo->si_plink;
	spin_lock(&sbinfo->si_plink_lock);
	list_for_each_entry(plink, plink_list, list)
		if (plink->inode == inode) {
			found = 1;
			break;
		}
	spin_unlock(&sbinfo->si_plink_lock);
	return found;
}

/* 20 is max digits length of ulong 64 */
#define PLINK_NAME_LEN	((20 + 1) * 2)

static int plink_name(char *name, int len, struct inode *inode,
		      aufs_bindex_t bindex)
{
	int rlen;
	struct inode *h_inode;

	LKTRTrace("i%lu, b%d\n", inode->i_ino, bindex);
	AuDebugOn(len != PLINK_NAME_LEN);
	h_inode = au_h_iptr(inode, bindex);
	AuDebugOn(!h_inode);
	rlen = snprintf(name, len, "%lu.%lu", inode->i_ino, h_inode->i_ino);
	AuDebugOn(rlen >= len);
	return rlen;
}

struct dentry *au_plink_lkup(struct super_block *sb, aufs_bindex_t bindex,
			     struct inode *inode)
{
	struct dentry *h_dentry, *h_parent;
	struct au_branch *br;
	struct au_wbr *wbr;
	struct inode *h_dir;
	char tgtname[PLINK_NAME_LEN];
	int len;
	struct au_ndx ndx = {
		.flags	= 0,
		.nd	= NULL,
		/* .br	= NULL */
	};

	LKTRTrace("b%d, i%lu\n", bindex, inode->i_ino);
	br = au_sbr(sb, bindex);
	wbr = br->br_wbr;
	AuDebugOn(!wbr);
	h_parent = wbr->wbr_plink;
	AuDebugOn(!h_parent);
	h_dir = h_parent->d_inode;
	AuDebugOn(!h_dir);

	len = plink_name(tgtname, sizeof(tgtname), inode, bindex);

	/* always superio. */
	ndx.nfsmnt = au_do_nfsmnt(br->br_mnt);
	mutex_lock_nested(&h_dir->i_mutex, AuLsc_I_CHILD2);
	h_dentry = au_sio_lkup_one(tgtname, h_parent, len, &ndx);
	mutex_unlock(&h_dir->i_mutex);
	return h_dentry;
}

static int do_whplink(char *tgt, int len, struct dentry *h_parent,
		      struct dentry *h_dentry, struct vfsmount *nfsmnt,
		      struct super_block *sb)
{
	int err, dlgt;
	struct dentry *h_tgt;
	struct inode *h_dir;
	struct vfsub_args vargs;
	struct au_ndx ndx = {
		.nfsmnt	= nfsmnt,
		.flags	= 0,
		.nd	= NULL,
		/* .br	= NULL */
	};

	AuTraceEnter();

	dlgt = !!au_test_dlgt(au_mntflags(sb));
	if (unlikely(dlgt))
		au_fset_ndx(ndx.flags, DLGT);
	h_tgt = au_lkup_one(tgt, h_parent, len, &ndx);
	err = PTR_ERR(h_tgt);
	if (IS_ERR(h_tgt))
		goto out;

	err = 0;
	vfsub_args_init(&vargs, NULL, dlgt, 0);
	/* wh.plink dir is not monitored */
	h_dir = h_parent->d_inode;
	if (unlikely(h_tgt->d_inode && h_tgt->d_inode != h_dentry->d_inode))
		err = vfsub_unlink(h_dir, h_tgt, &vargs);
	if (!err && !h_tgt->d_inode) {
		err = vfsub_link(h_dentry, h_dir, h_tgt, &vargs);
		/* todo: unnecessary? */
		/* inc_nlink(inode); */
	}
	dput(h_tgt);

 out:
	AuTraceErr(err);
	return err;
}

struct do_whplink_args {
	int *errp;
	char *tgt;
	int len;
	struct dentry *h_parent;
	struct dentry *h_dentry;
	struct vfsmount *nfsmnt;
	struct super_block *sb;
};

static void call_do_whplink(void *args)
{
	struct do_whplink_args *a = args;
	*a->errp = do_whplink(a->tgt, a->len, a->h_parent, a->h_dentry,
			      a->nfsmnt, a->sb);
}

static int whplink(struct dentry *h_dentry, struct inode *inode,
		   aufs_bindex_t bindex, struct super_block *sb)
{
	int err, len, wkq_err;
	struct au_branch *br;
	struct au_wbr *wbr;
	struct dentry *h_parent;
	struct inode *h_dir;
	char tgtname[PLINK_NAME_LEN];

	LKTRTrace("%.*s\n", AuDLNPair(h_dentry));
	br = au_sbr(inode->i_sb, bindex);
	wbr = br->br_wbr;
	AuDebugOn(!wbr);
	h_parent = wbr->wbr_plink;
	AuDebugOn(!h_parent);
	h_dir = h_parent->d_inode;
	AuDebugOn(!h_dir);

	len = plink_name(tgtname, sizeof(tgtname), inode, bindex);

	/* always superio. */
	mutex_lock_nested(&h_dir->i_mutex, AuLsc_I_CHILD2);
	if (!au_test_wkq(current)) {
		struct do_whplink_args args = {
			.errp		= &err,
			.tgt		= tgtname,
			.len		= len,
			.h_parent	= h_parent,
			.h_dentry	= h_dentry,
			.nfsmnt		= au_do_nfsmnt(br->br_mnt),
			.sb		= sb
		};
		wkq_err = au_wkq_wait(call_do_whplink, &args, /*dlgt*/0);
		if (unlikely(wkq_err))
			err = wkq_err;
	} else
		err = do_whplink(tgtname, len, h_parent, h_dentry,
				 au_do_nfsmnt(br->br_mnt), sb);
	mutex_unlock(&h_dir->i_mutex);

	AuTraceErr(err);
	return err;
}

void au_plink_append(struct super_block *sb, struct inode *inode,
		     struct dentry *h_dentry, aufs_bindex_t bindex)
{
	struct au_sbinfo *sbinfo;
	struct list_head *plink_list;
	struct pseudo_link *plink;
	int found, err, cnt;

	LKTRTrace("i%lu\n", inode->i_ino);
	SiMustAnyLock(sb);
	sbinfo = au_sbi(sb);
	AuDebugOn(!au_opt_test(au_mntflags(sb), PLINK));

	cnt = 0;
	found = 0;
	plink_list = &sbinfo->si_plink;
	spin_lock(&sbinfo->si_plink_lock);
	list_for_each_entry(plink, plink_list, list) {
		cnt++;
		if (plink->inode == inode) {
			found = 1;
			break;
		}
	}

	err = 0;
	if (!found) {
		plink = kmalloc(sizeof(*plink), GFP_ATOMIC);
		if (plink) {
			plink->inode = au_igrab(inode);
			list_add(&plink->list, plink_list);
			cnt++;
		} else
			err = -ENOMEM;
	}
	spin_unlock(&sbinfo->si_plink_lock);

#if 0 /* todo: test here */
	if (found)
		return; /* success */
#endif

	if (!err)
		err = whplink(h_dentry, inode, bindex, sb);

	if (unlikely(cnt > AUFS_PLINK_WARN))
		AuWarn1("unexpectedly many pseudo links, %d\n", cnt);
	if (unlikely(err))
		AuWarn("err %d, damaged pseudo link. ignored.\n", err);
}

static void do_put_plink(struct pseudo_link *plink, int do_del)
{
	AuTraceEnter();

	iput(plink->inode);
	if (do_del)
		list_del(&plink->list);
	kfree(plink);
}

void au_plink_put(struct super_block *sb)
{
	struct au_sbinfo *sbinfo;
	struct list_head *plink_list;
	struct pseudo_link *plink, *tmp;

	AuTraceEnter();
	SiMustWriteLock(sb);
	sbinfo = au_sbi(sb);
	AuDebugOn(!au_opt_test(au_mntflags(sb), PLINK));

	plink_list = &sbinfo->si_plink;
	/* spin_lock(&sbinfo->si_plink_lock); */
	list_for_each_entry_safe(plink, tmp, plink_list, list)
		do_put_plink(plink, 0);
	INIT_LIST_HEAD(plink_list);
	/* spin_unlock(&sbinfo->si_plink_lock); */
}

void au_plink_half_refresh(struct super_block *sb, aufs_bindex_t br_id)
{
	struct au_sbinfo *sbinfo;
	struct list_head *plink_list;
	struct pseudo_link *plink, *tmp;
	struct inode *inode;
	aufs_bindex_t bstart, bend, bindex;
	int do_put;

	AuTraceEnter();
	SiMustWriteLock(sb);
	sbinfo = au_sbi(sb);
	AuDebugOn(!au_opt_test(au_mntflags(sb), PLINK));

	plink_list = &sbinfo->si_plink;
	/* spin_lock(&sbinfo->si_plink_lock); */
	list_for_each_entry_safe(plink, tmp, plink_list, list) {
		do_put = 0;
		inode = au_igrab(plink->inode);
		ii_write_lock_child(inode);
		bstart = au_ibstart(inode);
		bend = au_ibend(inode);
		if (bstart >= 0) {
			for (bindex = bstart; bindex <= bend; bindex++) {
				if (!au_h_iptr(inode, bindex)
				    || au_ii_br_id(inode, bindex) != br_id)
					continue;
				au_set_h_iptr(inode, bindex, NULL, 0);
				do_put = 1;
				break;
			}
		} else
			do_put_plink(plink, 1);

		if (do_put) {
			for (bindex = bstart; bindex <= bend; bindex++)
				if (au_h_iptr(inode, bindex)) {
					do_put = 0;
					break;
				}
			if (do_put)
				do_put_plink(plink, 1);
		}
		ii_write_unlock(inode);
		iput(inode);
	}
	/* spin_unlock(&sbinfo->si_plink_lock); */
}
