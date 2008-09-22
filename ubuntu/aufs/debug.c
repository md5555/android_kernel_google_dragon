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
 * debug print functions
 *
 * $Id: debug.c,v 1.14 2008/09/22 03:52:03 sfjro Exp $
 */

#include "aufs.h"

atomic_t au_cond = ATOMIC_INIT(0);

char *au_plevel = KERN_DEBUG;
#define dpri(fmt, arg...) do { \
	if (LktrCond) \
		printk("%s" fmt, au_plevel, ##arg); \
} while (0)

/* ---------------------------------------------------------------------- */

void au_dpri_whlist(struct au_nhash *whlist)
{
	int i;
	struct hlist_head *head;
	struct au_vdir_wh *tpos;
	struct hlist_node *pos;

	for (i = 0; i < AuSize_NHASH; i++) {
		head = whlist->heads + i;
		hlist_for_each_entry(tpos, pos, head, wh_hash)
			dpri("b%d, %.*s, %d\n",
			     tpos->wh_bindex,
			     tpos->wh_str.len, tpos->wh_str.name,
			     tpos->wh_str.len);
	}
}

void au_dpri_vdir(struct au_vdir *vdir)
{
	int i;
	union au_vdir_deblk_p p;
	unsigned char *o;

	if (!vdir || IS_ERR(vdir)) {
		dpri("err %ld\n", PTR_ERR(vdir));
		return;
	}

	dpri("nblk %d, deblk %p, last{%d, %p}, ver %lu\n",
	     vdir->vd_nblk, vdir->vd_deblk,
	     vdir->vd_last.i, vdir->vd_last.p.p, vdir->vd_version);
	for (i = 0; i < vdir->vd_nblk; i++) {
		p.deblk = vdir->vd_deblk[i];
		o = p.p;
		dpri("[%d]: %p\n", i, o);
	}
}

static int do_pri_inode(aufs_bindex_t bindex, struct inode *inode,
			struct dentry *wh)
{
	char *n = NULL;
	int l = 0, ntfy = 0;

	if (!inode || IS_ERR(inode)) {
		dpri("i%d: err %ld\n", bindex, PTR_ERR(inode));
		return -1;
	}

	/* the type of i_blocks depends upon CONFIG_LSF */
	BUILD_BUG_ON(sizeof(inode->i_blocks) != sizeof(unsigned long)
		     && sizeof(inode->i_blocks) != sizeof(u64));
	if (wh) {
		n = (void *)wh->d_name.name;
		l = wh->d_name.len;
	}

	ntfy = au_test_inotify(inode);
	dpri("i%d: i%lu, %s, cnt %d, nl %u, 0%o, ntfy %d, sz %llu, blk %llu,"
	     " ct %lld, np %lu, st 0x%lx, f 0x%x, g %x%s%.*s\n",
	     bindex,
	     inode->i_ino, inode->i_sb ? au_sbtype(inode->i_sb) : "??",
	     atomic_read(&inode->i_count), inode->i_nlink, inode->i_mode,
	     ntfy,
	     i_size_read(inode), (unsigned long long)inode->i_blocks,
	     (long long)timespec_to_ns(&inode->i_ctime) & 0x0ffff,
	     inode->i_mapping ? inode->i_mapping->nrpages : 0,
	     inode->i_state, inode->i_flags, inode->i_generation,
	     l ? ", wh " : "", l, n);
	return 0;
}

void au_dpri_inode(struct inode *inode)
{
	struct au_iinfo *iinfo;
	aufs_bindex_t bindex;
	int err;

	err = do_pri_inode(-1, inode, NULL);
	if (err || !au_test_aufs(inode->i_sb))
		return;

	iinfo = au_ii(inode);
	if (!iinfo)
		return;
	dpri("i-1: bstart %d, bend %d, gen %d\n",
	     iinfo->ii_bstart, iinfo->ii_bend, au_iigen(inode));
	if (iinfo->ii_bstart < 0)
		return;
	for (bindex = iinfo->ii_bstart; bindex <= iinfo->ii_bend; bindex++)
		do_pri_inode(bindex, iinfo->ii_hinode[0 + bindex].hi_inode,
			     iinfo->ii_hinode[0 + bindex].hi_whdentry);
}

static int do_pri_dentry(aufs_bindex_t bindex, struct dentry *dentry,
			 struct list_head *intent)
{
	struct dentry *wh = NULL;

	if (!dentry || IS_ERR(dentry)) {
		dpri("d%d: err %ld\n", bindex, PTR_ERR(dentry));
		return -1;
	}
	/* do not call dget_parent() here */
	dpri("d%d: %.*s?/%.*s, %s, cnt %d, flags 0x%x, intent %d\n",
	     bindex,
	     AuDLNPair(dentry->d_parent), AuDLNPair(dentry),
	     dentry->d_sb ? au_sbtype(dentry->d_sb) : "??",
	     atomic_read(&dentry->d_count), dentry->d_flags, !!intent);
	if (bindex >= 0 && dentry->d_inode && au_test_aufs(dentry->d_sb)) {
		struct au_iinfo *iinfo = au_ii(dentry->d_inode);
		if (iinfo)
			wh = iinfo->ii_hinode[0 + bindex].hi_whdentry;
	}
	do_pri_inode(bindex, dentry->d_inode, wh);
	return 0;
}

static struct list_head *au_dbg_h_intent(struct au_dinfo *dinfo,
					 aufs_bindex_t bindex)
{
#ifdef CONFIG_AUFS_BR_NFS
	return dinfo->di_hdentry[0 + bindex].hd_intent_list;
#else
	return NULL;
#endif
}

void au_dpri_dentry(struct dentry *dentry)
{
	struct au_dinfo *dinfo;
	aufs_bindex_t bindex;
	int err;

	err = do_pri_dentry(-1, dentry, NULL);
	if (err || !au_test_aufs(dentry->d_sb))
		return;

	dinfo = au_di(dentry);
	if (!dinfo)
		return;
	dpri("d-1: bstart %d, bend %d, bwh %d, bdiropq %d, gen %d\n",
	     dinfo->di_bstart, dinfo->di_bend,
	     dinfo->di_bwh, dinfo->di_bdiropq, au_digen(dentry));
	if (dinfo->di_bstart < 0)
		return;
	for (bindex = dinfo->di_bstart; bindex <= dinfo->di_bend; bindex++)
		do_pri_dentry(bindex, dinfo->di_hdentry[0 + bindex].hd_dentry,
			      au_dbg_h_intent(dinfo, bindex));
}

static int do_pri_file(aufs_bindex_t bindex, struct file *file)
{
	char a[32];

	if (!file || IS_ERR(file)) {
		dpri("f%d: err %ld\n", bindex, PTR_ERR(file));
		return -1;
	}
	a[0] = 0;
	if (bindex < 0
	    && file->f_dentry
	    && au_test_aufs(file->f_dentry->d_sb)
	    && au_fi(file))
		snprintf(a, sizeof(a), ", mmapped %d", au_test_mmapped(file));
	dpri("f%d: mode 0x%x, flags 0%o, cnt %ld, pos %llu%s\n",
	     bindex, file->f_mode, file->f_flags, (long)file_count(file),
	     file->f_pos, a);
	if (file->f_dentry)
		do_pri_dentry(bindex, file->f_dentry, NULL);
	return 0;
}

void au_dpri_file(struct file *file)
{
	struct au_finfo *finfo;
	aufs_bindex_t bindex;
	int err;

	err = do_pri_file(-1, file);
	if (err || !file->f_dentry || !au_test_aufs(file->f_dentry->d_sb))
		return;

	finfo = au_fi(file);
	if (!finfo)
		return;
	if (finfo->fi_bstart < 0)
		return;
	for (bindex = finfo->fi_bstart; bindex <= finfo->fi_bend; bindex++) {
		struct au_hfile *hf;
		hf = finfo->fi_hfile + bindex;
		do_pri_file(bindex, hf ? hf->hf_file : NULL);
	}
}

static int do_pri_br(aufs_bindex_t bindex, struct au_branch *br)
{
	struct vfsmount *mnt;
	struct super_block *sb;

	if (!br || IS_ERR(br)
	    || !(mnt = br->br_mnt) || IS_ERR(mnt)
	    || !(sb = mnt->mnt_sb) || IS_ERR(sb)) {
		dpri("s%d: err %ld\n", bindex, PTR_ERR(br));
		return -1;
	}

	dpri("s%d: {perm 0x%x, cnt %d, wbr %p}, "
	     "%s, dev 0x%02x%02x, flags 0x%lx, cnt(BIAS) %d, active %d, "
	     "xino %d\n",
	     bindex, br->br_perm, au_br_count(br), br->br_wbr,
	     au_sbtype(sb), MAJOR(sb->s_dev), MINOR(sb->s_dev),
	     sb->s_flags, sb->s_count - S_BIAS,
	     atomic_read(&sb->s_active), !!br->br_xino.xi_file);
	return 0;
}

void au_dpri_sb(struct super_block *sb)
{
	struct au_sbinfo *sbinfo;
	aufs_bindex_t bindex;
	int err;
	/* to reuduce stack size */
	struct {
		struct vfsmount mnt;
		struct au_branch fake;
	} *a;

	/* this function can be called from magic sysrq */
	a = kzalloc(sizeof(*a), GFP_ATOMIC);
	if (unlikely(!a)) {
		dpri("no memory\n");
		return;
	}

	a->mnt.mnt_sb = sb;
	a->fake.br_perm = 0;
	a->fake.br_mnt = &a->mnt;
	a->fake.br_xino.xi_file = NULL;
	atomic_set(&a->fake.br_count, 0);
	smp_mb(); /* atomic_set */
	err = do_pri_br(-1, &a->fake);
	kfree(a);
	dpri("dev 0x%x\n", sb->s_dev);
	if (err || !au_test_aufs(sb))
		return;

	sbinfo = au_sbi(sb);
	if (!sbinfo)
		return;
	dpri("gen %u\n", sbinfo->si_generation);
	for (bindex = 0; bindex <= sbinfo->si_bend; bindex++)
		do_pri_br(bindex, sbinfo->si_branch[0 + bindex]);
}

/* ---------------------------------------------------------------------- */

void au_dbg_sleep(int sec)
{
	static DECLARE_WAIT_QUEUE_HEAD(wq);
	wait_event_timeout(wq, 0, sec * HZ);
}

void au_dbg_sleep_jiffy(int jiffy)
{
	static DECLARE_WAIT_QUEUE_HEAD(wq);
	wait_event_timeout(wq, 0, jiffy);
}

void au_dbg_iattr(struct iattr *ia)
{
#define AuBit(name)	if (ia->ia_valid & ATTR_ ## name) dpri(#name "\n")
	AuBit(MODE);
	AuBit(UID);
	AuBit(GID);
	AuBit(SIZE);
	AuBit(ATIME);
	AuBit(MTIME);
	AuBit(CTIME);
	AuBit(ATIME_SET);
	AuBit(MTIME_SET);
	AuBit(FORCE);
	AuBit(ATTR_FLAG);
	AuBit(KILL_SUID);
	AuBit(KILL_SGID);
	AuBit(FILE);
	AuBit(KILL_PRIV);
	AuBit(OPEN);
	AuBit(TIMES_SET);
#undef	AuBit
	dpri("ia_file %p\n", ia->ia_file);
}

/* ---------------------------------------------------------------------- */

void au_debug_sbinfo_init(struct au_sbinfo *sbinfo)
{
#ifdef ForceInotify
	au_opt_set_udba(sbinfo->si_mntflags, UDBA_INOTIFY);
#endif
#ifdef ForceDlgt
	au_opt_set(sbinfo->si_mntflags, DLGT);
#endif
#ifdef ForceNoPlink
	au_opt_clr(sbinfo->si_mntflags, PLINK);
#endif
#ifdef ForceNoXino
	au_opt_clr(sbinfo->si_mntflags, XINO);
#endif
#ifdef ForceNoRefrof
	au_opt_clr(sbinfo->si_mntflags, REFROF);
#endif
#ifdef ForceShwh
	au_opt_set(sbinfo->si_mntflags, SHWH);
#endif
}

int __init au_debug_init(void)
{
	aufs_bindex_t bindex;
	struct au_vdir_destr destr;

	bindex = -1;
	AuDebugOn(bindex >= 0);

	destr.len = -1;
	AuDebugOn(destr.len < NAME_MAX);

#ifdef CONFIG_4KSTACKS
	AuWarn("CONFIG_4KSTACKS is defined.\n");
#endif

#ifdef ForceBrs
	sysaufs_brs = 1;
#endif

#if 0 /* verbose debug */
	{
		union {
			struct au_branch *br;
			struct au_dinfo *di;
			struct au_finfo *fi;
			struct au_iinfo *ii;
			struct au_hinode *hi;
			struct au_sbinfo *si;
			struct au_vdir_destr *destr;
			struct au_vdir_de *de;
			struct au_vdir_wh *wh;
			struct au_vdir *vd;
		} u;

		pr_info("br{"
			"xino %d, "
			"id %d, perm %d, mnt %d, count %d, "
			"wbr %d, "
			"xup %d, xrun %d, "
			"gen %d, "
			"sa %d} %d\n",
			offsetof(typeof(*u.br), br_xino),
			offsetof(typeof(*u.br), br_id),
			offsetof(typeof(*u.br), br_perm),
			offsetof(typeof(*u.br), br_mnt),
			offsetof(typeof(*u.br), br_count),
			offsetof(typeof(*u.br), wbr),
			offsetof(typeof(*u.br), br_xino_upper),
			offsetof(typeof(*u.br), br_xino_running),
			offsetof(typeof(*u.br), br_generation),
			offsetof(typeof(*u.br), br_sabr),
			sizeof(*u.br));
		pr_info("di{gen %d, rwsem %d, bstart %d, bend %d, bwh %d, "
			"bdiropq %d, hdentry %d} %d\n",
			offsetof(typeof(*u.di), di_generation),
			offsetof(typeof(*u.di), di_rwsem),
			offsetof(typeof(*u.di), di_bstart),
			offsetof(typeof(*u.di), di_bend),
			offsetof(typeof(*u.di), di_bwh),
			offsetof(typeof(*u.di), di_bdiropq),
			offsetof(typeof(*u.di), di_hdentry),
			sizeof(*u.di));
		pr_info("fi{gen %d, rwsem %d, hfile %d, bstart %d, bend %d, "
			"h_vm_ops %d, vdir_cach %d} %d\n",
			offsetof(typeof(*u.fi), fi_generation),
			offsetof(typeof(*u.fi), fi_rwsem),
			offsetof(typeof(*u.fi), fi_hfile),
			offsetof(typeof(*u.fi), fi_bstart),
			offsetof(typeof(*u.fi), fi_bend),
			offsetof(typeof(*u.fi), fi_h_vm_ops),
			offsetof(typeof(*u.fi), fi_vdir_cache),
			sizeof(*u.fi));
		pr_info("ii{gen %d, hsb %d, "
			"rwsem %d, bstart %d, bend %d, hinode %d, vdir %d} "
			"%d\n",
			offsetof(typeof(*u.ii), ii_generation),
			offsetof(typeof(*u.ii), ii_hsb1),
			offsetof(typeof(*u.ii), ii_rwsem),
			offsetof(typeof(*u.ii), ii_bstart),
			offsetof(typeof(*u.ii), ii_bend),
			offsetof(typeof(*u.ii), ii_hinode),
			offsetof(typeof(*u.ii), ii_vdir),
			sizeof(*u.ii));
		pr_info("hi{inode %d, id %d, notify %d, wh %d} %d\n",
			offsetof(typeof(*u.hi), hi_inode),
			offsetof(typeof(*u.hi), hi_id),
			offsetof(typeof(*u.hi), hi_notify),
			offsetof(typeof(*u.hi), hi_whdentry),
			sizeof(*u.hi));
		pr_info("si{nwt %d, rwsem %d, gen %d, stat %d, "
			"bend %d, last id %d, br %d, "
			"cpup %d, creat %d, ops %d, ops %d, "
			"rr %d, mfs %d, "
			"mntflags %d, "
			"xread %d, xwrite %d, xib %d, xmtx %d, buf %d, "
			"xlast %d, xnext %d, "
			"rdcache %d, "
			"dirwh %d, "
			"pl_lock %d, pl %d, "
			"mnt %d, "
			"sys %d, "
			/* "lvma_l %d, lvma %d" */
			"} %d\n",
			offsetof(typeof(*u.si), si_nowait),
			offsetof(typeof(*u.si), si_rwsem),
			offsetof(typeof(*u.si), si_generation),
			offsetof(typeof(*u.si), au_si_status),
			offsetof(typeof(*u.si), si_bend),
			offsetof(typeof(*u.si), si_last_br_id),
			offsetof(typeof(*u.si), si_branch),
			offsetof(typeof(*u.si), si_wbr_copyup),
			offsetof(typeof(*u.si), si_wbr_create),
			offsetof(typeof(*u.si), si_wbr_copyup_ops),
			offsetof(typeof(*u.si), si_wbr_create_ops),
			offsetof(typeof(*u.si), si_wbr_rr_next),
			offsetof(typeof(*u.si), si_wbr_mfs),
			offsetof(typeof(*u.si), si_mntflags),
			offsetof(typeof(*u.si), si_xread),
			offsetof(typeof(*u.si), si_xwrite),
			offsetof(typeof(*u.si), si_xib),
			offsetof(typeof(*u.si), si_xib_mtx),
			offsetof(typeof(*u.si), si_xib_buf),
			offsetof(typeof(*u.si), si_xib_last_pindex),
			offsetof(typeof(*u.si), si_xib_next_bit),
			offsetof(typeof(*u.si), si_rdcache),
			offsetof(typeof(*u.si), si_dirwh),
			offsetof(typeof(*u.si), si_plink_lock),
			offsetof(typeof(*u.si), si_plink),
			offsetof(typeof(*u.si), si_mnt),
			offsetof(typeof(*u.si), si_sa),
			/*offsetof(typeof(*u.si), si_lvma_lock),
			offsetof(typeof(*u.si), si_lvma),*/
			sizeof(*u.si));
		pr_info("destr{len %d, name %d} %d\n",
			offsetof(typeof(*u.destr), len),
			offsetof(typeof(*u.destr), name),
			sizeof(*u.destr));
		pr_info("de{ino %d, type %d, str %d} %d\n",
			offsetof(typeof(*u.de), de_ino),
			offsetof(typeof(*u.de), de_type),
			offsetof(typeof(*u.de), de_str),
			sizeof(*u.de));
		pr_info("wh{hash %d, bindex %d, str %d} %d\n",
			offsetof(typeof(*u.wh), wh_hash),
			offsetof(typeof(*u.wh), wh_bindex),
			offsetof(typeof(*u.wh), wh_str),
			sizeof(*u.wh));
		pr_info("vd{deblk %d, nblk %d, last %d, ver %d, jiffy %d} %d\n",
			offsetof(typeof(*u.vd), vd_deblk),
			offsetof(typeof(*u.vd), vd_nblk),
			offsetof(typeof(*u.vd), vd_last),
			offsetof(typeof(*u.vd), vd_version),
			offsetof(typeof(*u.vd), vd_jiffy),
			sizeof(*u.vd));
	}
#endif

	return 0;
}
