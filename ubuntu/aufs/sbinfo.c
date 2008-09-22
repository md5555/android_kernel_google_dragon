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
 * superblock private data
 *
 * $Id: sbinfo.c,v 1.11 2008/07/27 22:49:36 sfjro Exp $
 */

#include <linux/smp_lock.h>
#include "aufs.h"

/*
 * they are necessary regardless sysfs is disabled.
 */
void au_si_free(struct kobject *kobj)
{
	struct au_sbinfo *sbinfo;
	struct super_block *sb;

	LKTRTrace("kobj %p\n", kobj);
	sbinfo = container_of(kobj, struct au_sbinfo, si_kobj);
	LKTRTrace("sbinfo %p\n", sbinfo);
	AuDebugOn(!list_empty(&sbinfo->si_plink));

	sb = sbinfo->si_sb;
	si_write_lock(sb);
	au_xino_clr(sb);
	au_br_free(sbinfo);
	kfree(sbinfo->si_branch);
	au_export_put(sbinfo);
	si_write_unlock(sb);

	kfree(sbinfo);
}

int au_si_alloc(struct super_block *sb)
{
	int err;
	struct au_sbinfo *sbinfo;

	AuTraceEnter();

	err = -ENOMEM;
	sbinfo = kmalloc(sizeof(*sbinfo), GFP_NOFS);
	if (unlikely(!sbinfo))
		goto out;
	sbinfo->si_branch = kzalloc(sizeof(*sbinfo->si_branch), GFP_NOFS);
	if (unlikely(!sbinfo->si_branch))
		goto out_sbinfo;

	memset(&sbinfo->si_kobj, 0, sizeof(sbinfo->si_kobj));
	err = sysaufs_si_init(sbinfo);
	if (unlikely(err))
		goto out_br;

	au_rw_init_wlock(&sbinfo->si_rwsem);
	sbinfo->si_generation = 0;
	sbinfo->au_si_status = 0;
	sbinfo->si_bend = -1;
	sbinfo->si_last_br_id = 0;

	sbinfo->si_wbr_copyup = AuWbrCopyup_Def;
	sbinfo->si_wbr_create = AuWbrCreate_Def;
	sbinfo->si_wbr_copyup_ops = au_wbr_copyup_ops + AuWbrCopyup_Def;
	sbinfo->si_wbr_create_ops = au_wbr_create_ops + AuWbrCreate_Def;

	sbinfo->si_mntflags = AuOpt_Def;

	sbinfo->si_xread = NULL;
	sbinfo->si_xwrite = NULL;
	sbinfo->si_xib = NULL;
	mutex_init(&sbinfo->si_xib_mtx);
	sbinfo->si_xib_buf = NULL;
	au_xino_def_br_set(NULL, sbinfo);
	/* leave si_xib_last_pindex and si_xib_next_bit */

	au_nwt_init(&sbinfo->si_nowait);

	sbinfo->si_rdcache = AUFS_RDCACHE_DEF * HZ;
	sbinfo->si_dirwh = AUFS_DIRWH_DEF;

	spin_lock_init(&sbinfo->si_plink_lock);
	INIT_LIST_HEAD(&sbinfo->si_plink);

	au_robr_lvma_init(sbinfo);

	/* leave other members for sysaufs and si_mnt. */
	sbinfo->si_sb = sb;

	sb->s_fs_info = sbinfo;

	au_debug_sbinfo_init(sbinfo);
	return 0; /* success */

 out_br:
	kfree(sbinfo->si_branch);
 out_sbinfo:
	kfree(sbinfo);
 out:
	AuTraceErr(err);
	return err;
}

/* ---------------------------------------------------------------------- */

struct au_branch *au_sbr(struct super_block *sb, aufs_bindex_t bindex)
{
	struct au_branch *br;

	SiMustAnyLock(sb);
	AuDebugOn(bindex < 0 || au_sbend(sb) < bindex);
	br = au_sbi(sb)->si_branch[0 + bindex];
	AuDebugOn(!br);
	return br;
}

au_gen_t au_sigen_inc(struct super_block *sb)
{
	au_gen_t gen;

	SiMustWriteLock(sb);
	gen = ++au_sbi(sb)->si_generation;
	au_update_digen(sb->s_root);
	au_update_iigen(sb->s_root->d_inode);
	sb->s_root->d_inode->i_version++;
	return gen;
}

int au_find_bindex(struct super_block *sb, struct au_branch *br)
{
	aufs_bindex_t bindex, bend;

	bend = au_sbend(sb);
	for (bindex = 0; bindex <= bend; bindex++)
		if (au_sbr(sb, bindex) == br)
			return bindex;
	return -1;
}

/* ---------------------------------------------------------------------- */

/* dentry and super_block lock. call at entry point */
void aufs_read_lock(struct dentry *dentry, int flags)
{
	si_read_lock(dentry->d_sb, flags);
	if (au_ftest_lock(flags, DW))
		di_write_lock_child(dentry);
	else
		di_read_lock_child(dentry, flags);
}

void aufs_read_unlock(struct dentry *dentry, int flags)
{
	if (au_ftest_lock(flags, DW))
		di_write_unlock(dentry);
	else
		di_read_unlock(dentry, flags);
	si_read_unlock(dentry->d_sb);
}

void aufs_write_lock(struct dentry *dentry)
{
	si_write_lock(dentry->d_sb);
	di_write_lock_child(dentry);
}

void aufs_write_unlock(struct dentry *dentry)
{
	di_write_unlock(dentry);
	si_write_unlock(dentry->d_sb);
}

void aufs_read_and_write_lock2(struct dentry *d1, struct dentry *d2, int flags)
{
	AuDebugOn(d1 == d2 || d1->d_sb != d2->d_sb);
	si_read_lock(d1->d_sb, flags);
	di_write_lock2_child(d1, d2, au_ftest_lock(flags, DIR));
}

void aufs_read_and_write_unlock2(struct dentry *d1, struct dentry *d2)
{
	AuDebugOn(d1 == d2 || d1->d_sb != d2->d_sb);
	di_write_unlock2(d1, d2);
	si_read_unlock(d1->d_sb);
}

/* ---------------------------------------------------------------------- */

aufs_bindex_t au_new_br_id(struct super_block *sb)
{
	aufs_bindex_t br_id;
	struct au_sbinfo *sbinfo;

	AuTraceEnter();
	SiMustWriteLock(sb);

	sbinfo = au_sbi(sb);
	while (1) {
		br_id = ++sbinfo->si_last_br_id;
		if (br_id && au_br_index(sb, br_id) < 0)
			return br_id;
	}
}
