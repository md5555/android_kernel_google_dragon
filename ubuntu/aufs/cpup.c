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
 * copy-up functions, see wbr_policy.c for copy-down
 *
 * $Id: cpup.c,v 1.17 2008/09/22 03:52:19 sfjro Exp $
 */

#include <linux/fs_stack.h>
#include <linux/uaccess.h>
#include "aufs.h"

/* todo? violent cpup_attr_*() functions don't care inode lock */

void au_cpup_attr_timesizes(struct inode *inode)
{
	struct inode *h_inode;

	LKTRTrace("i%lu\n", inode->i_ino);
	/* todo? IMustLock(inode); */
	h_inode = au_h_iptr(inode, au_ibstart(inode));
	AuDebugOn(!h_inode);
	/* todo? IMustLock(!h_inode); */

	fsstack_copy_attr_times(inode, h_inode);
	vfsub_copy_inode_size(inode, h_inode);
}

void au_cpup_attr_nlink(struct inode *inode)
{
	struct inode *h_inode;

	LKTRTrace("i%lu\n", inode->i_ino);
	/* todo? IMustLock(inode); */
	AuDebugOn(!inode->i_mode);

	h_inode = au_h_iptr(inode, au_ibstart(inode));
	inode->i_nlink = h_inode->i_nlink;

	/*
	 * fewer nlink makes find(1) noisy, but larger nlink doesn't.
	 * it may includes whplink directory.
	 */
	if (unlikely(S_ISDIR(h_inode->i_mode))) {
		aufs_bindex_t bindex, bend;
		bend = au_ibend(inode);
		for (bindex = au_ibstart(inode) + 1; bindex <= bend; bindex++) {
			h_inode = au_h_iptr(inode, bindex);
			if (h_inode)
				au_add_nlink(inode, h_inode);
		}
	}
}

void au_cpup_attr_changeable(struct inode *inode)
{
	struct inode *h_inode;

	LKTRTrace("i%lu\n", inode->i_ino);
	/* todo? IMustLock(inode); */
	h_inode = au_h_iptr(inode, au_ibstart(inode));
	AuDebugOn(!h_inode);

	inode->i_mode = h_inode->i_mode;
	inode->i_uid = h_inode->i_uid;
	inode->i_gid = h_inode->i_gid;
	au_cpup_attr_timesizes(inode);

	/* todo: remove this? */
	inode->i_flags = h_inode->i_flags;
}

void au_cpup_igen(struct inode *inode, struct inode *h_inode)
{
	struct au_iinfo *iinfo = au_ii(inode);
	iinfo->ii_higen = h_inode->i_generation;
	iinfo->ii_hsb1 = h_inode->i_sb;
}

void au_cpup_attr_all(struct inode *inode)
{
	struct inode *h_inode;

	LKTRTrace("i%lu\n", inode->i_ino);
	/* todo? IMustLock(inode); */
	h_inode = au_h_iptr(inode, au_ibstart(inode));
	AuDebugOn(!h_inode);

	au_cpup_attr_changeable(inode);
	if (inode->i_nlink > 0)
		au_cpup_attr_nlink(inode);

	switch (inode->i_mode & S_IFMT) {
	case S_IFBLK:
	case S_IFCHR:
		inode->i_rdev = au_h_rdev(h_inode, /*h_mnt*/NULL,
					  /*h_dentry*/NULL);
	}
	inode->i_blkbits = h_inode->i_blkbits;
	au_cpup_igen(inode, h_inode);
}

/* ---------------------------------------------------------------------- */

/* Note: dt_dentry and dt_hidden_dentry are not dget/dput-ed */

/* keep the timestamps of the parent dir when cpup */
void au_dtime_store(struct au_dtime *dt, struct dentry *dentry,
		    struct dentry *h_dentry, struct au_hinode *hinode,
		    struct au_hinode *hdir)
{
	struct inode *h_inode;

	LKTRTrace("%.*s, hdir %d\n", AuDLNPair(dentry), !!hdir);
	AuDebugOn(!dentry || !h_dentry || !h_dentry->d_inode);

	dt->dt_dentry = dentry;
	dt->dt_h_dentry = h_dentry;
	dt->dt_hinode = hinode;
	dt->dt_hdir = hdir;
	h_inode = h_dentry->d_inode;
	dt->dt_atime = h_inode->i_atime;
	dt->dt_mtime = h_inode->i_mtime;
	/* smp_mb(); */
}

void au_dtime_revert(struct au_dtime *dt)
{
	struct iattr attr;
	int err;
	struct au_hin_ignore ign[2];
	struct vfsub_args vargs;

	LKTRTrace("%.*s\n", AuDLNPair(dt->dt_dentry));

	attr.ia_atime = dt->dt_atime;
	attr.ia_mtime = dt->dt_mtime;
	attr.ia_valid = ATTR_FORCE | ATTR_MTIME | ATTR_MTIME_SET
		| ATTR_ATIME | ATTR_ATIME_SET;

	vfsub_args_init(&vargs, ign,
			au_test_dlgt(au_mntflags(dt->dt_dentry->d_sb)), 0);
	/*
	 * IN_ATTRIB should be divided into
	 * IN_ATTRIB_ATIME, IN_ATTRIB_MTIME ...,
	 * and define all ORed new IN_ATTRIB macro.
	 */
	vfsub_ign_hinode(&vargs, IN_ATTRIB, dt->dt_hinode);
	vfsub_ign_hinode(&vargs, IN_ATTRIB, dt->dt_hdir);
	err = vfsub_notify_change(dt->dt_h_dentry, &attr, &vargs);
	if (unlikely(err))
		AuWarn("restoring timestamps failed(%d). ignored\n", err);
}

/* ---------------------------------------------------------------------- */

static noinline_for_stack
int cpup_iattr(struct dentry *dst, aufs_bindex_t bindex, struct dentry *h_src,
	       struct au_hinode *hdir, struct vfsub_args *vargs)
{
	int err, sbits;
	struct dentry *h_dst;
	struct iattr ia;
	struct inode *h_isrc, *h_idst;

	h_dst = au_h_dptr(dst, bindex);
	LKTRTrace("%.*s\n", AuDLNPair(h_dst));
	h_idst = h_dst->d_inode;
	/* todo? IMustLock(h_idst); */
	h_isrc = h_src->d_inode;
	/* todo? IMustLock(h_isrc); */

	ia.ia_valid = ATTR_FORCE | ATTR_MODE | ATTR_UID | ATTR_GID
		| ATTR_ATIME | ATTR_MTIME
		| ATTR_ATIME_SET | ATTR_MTIME_SET;
	ia.ia_mode = h_isrc->i_mode;
	ia.ia_uid = h_isrc->i_uid;
	ia.ia_gid = h_isrc->i_gid;
	ia.ia_atime = h_isrc->i_atime;
	ia.ia_mtime = h_isrc->i_mtime;
	sbits = !!(ia.ia_mode & (S_ISUID | S_ISGID));

	vfsub_args_reinit(vargs);
	vfsub_ign_hinode(vargs, IN_ATTRIB, hdir);
	err = vfsub_notify_change(h_dst, &ia, vargs);

	/* is this nfs only? */
	if (!err && sbits && au_test_nfs(h_dst->d_sb)) {
		ia.ia_valid = ATTR_FORCE | ATTR_MODE;
		ia.ia_mode = h_isrc->i_mode;
		vfsub_args_reinit(vargs);
		vfsub_ign_hinode(vargs, IN_ATTRIB, hdir);
		err = vfsub_notify_change(h_dst, &ia, vargs);
	}

	/* todo? remove this? */
	if (!err)
		h_idst->i_flags = h_isrc->i_flags;

	AuTraceErr(err);
	return err;
}

/*
 * to support a sparse file which is opened with O_APPEND,
 * we need to close the file.
 */
static noinline_for_stack
int cpup_regular(struct dentry *dentry, aufs_bindex_t bdst, aufs_bindex_t bsrc,
		 loff_t len, struct au_hinode *hdir, struct vfsub_args *vargs)
{
	int err, i;
	struct super_block *sb;
	struct inode *h_inode;
	enum { SRC, DST };
	struct {
		aufs_bindex_t bindex;
		unsigned int flags;
		struct dentry *dentry;
		struct file *file;
		void *label, *label_file;
	} *h, hidden[] = {
		{
			.bindex = bsrc,
			.flags = O_RDONLY | O_NOATIME | O_LARGEFILE,
			.file = NULL,
			.label = &&out,
			.label_file = &&out_src_file
		},
		{
			.bindex = bdst,
			.flags = O_WRONLY | O_NOATIME | O_LARGEFILE,
			.file = NULL,
			.label = &&out_src_file,
			.label_file = &&out_dst_file
		}
	};

	LKTRTrace("dentry %.*s, bdst %d, bsrc %d, len %lld\n",
		  AuDLNPair(dentry), bdst, bsrc, len);
	AuDebugOn(bsrc <= bdst);
	AuDebugOn(!len);
	sb = dentry->d_sb;
	AuDebugOn(au_test_ro(sb, bdst, dentry->d_inode));
	/* bsrc branch can be ro/rw. */

	h = hidden;
	for (i = 0; i < 2; i++, h++) {
		h->dentry = au_h_dptr(dentry, h->bindex);
		AuDebugOn(!h->dentry);
		h_inode = h->dentry->d_inode;
		AuDebugOn(!h_inode || !S_ISREG(h_inode->i_mode));
		h->file = au_h_open(dentry, h->bindex, h->flags, /*file*/NULL);
		err = PTR_ERR(h->file);
		if (IS_ERR(h->file))
			goto *h->label;
		err = -EINVAL;
		if (unlikely(!h->file->f_op))
			goto *h->label_file;
	}

	/* stop updating while we copyup */
	IMustLock(hidden[SRC].dentry->d_inode);
	err = au_copy_file(hidden[DST].file, hidden[SRC].file, len, hdir, sb,
			   vargs);

 out_dst_file:
	fput(hidden[DST].file);
	au_sbr_put(sb, hidden[DST].bindex);
 out_src_file:
	fput(hidden[SRC].file);
	au_sbr_put(sb, hidden[SRC].bindex);
 out:
	AuTraceErr(err);
	return err;
}

static int au_do_cpup_regular(struct dentry *dentry, aufs_bindex_t bdst,
			      aufs_bindex_t bsrc, loff_t len,
			      struct au_hinode *hdir, struct dentry *h_dst,
			      struct vfsub_args *vargs)
{
	int err, rerr;
	loff_t l;

	AuTraceEnter();

	err = 0;
	l = i_size_read(au_h_iptr(dentry->d_inode, bsrc));
	if (len == -1 || l < len)
		len = l;
	if (len)
		err = cpup_regular(dentry, bdst, bsrc, len, hdir, vargs);
	if (!err)
		goto out; /* success */

	vfsub_args_reinit(vargs);
	vfsub_ign_hinode(vargs, IN_DELETE, hdir);
	rerr = vfsub_unlink(hdir->hi_inode, h_dst, vargs);
	if (rerr) {
		AuIOErr("failed unlinking cpup-ed %.*s(%d, %d)\n",
			AuDLNPair(h_dst), err, rerr);
		err = -EIO;
	}

 out:
	AuTraceErr(err);
	return err;
}

static int au_do_cpup_symlink(struct dentry *h_dst, struct dentry *h_src,
			      struct inode *h_dir, umode_t mode,
			      struct vfsub_args *vargs)
{
	int err, symlen;
	char *sym;
	mm_segment_t old_fs;

	AuTraceEnter();

	err = -ENOMEM;
	sym = __getname();
	if (unlikely(!sym))
		goto out;

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	symlen = h_src->d_inode->i_op->readlink(h_src, (char __user *)sym,
						PATH_MAX);
	err = symlen;
	set_fs(old_fs);

	if (symlen > 0) {
		sym[symlen] = 0;
		err = vfsub_symlink(h_dir, h_dst, sym, mode, vargs);
	}
	__putname(sym);

 out:
	AuTraceErr(err);
	return err;
}

/* return with hidden dst inode is locked */
static noinline_for_stack
int cpup_entry(struct dentry *dentry, aufs_bindex_t bdst, aufs_bindex_t bsrc,
	       loff_t len, unsigned int flags, struct dentry *dst_parent,
	       struct vfsub_args *vargs)
{
	int err;
	unsigned char isdir, hinotify;
	struct dentry *h_src, *h_dst, *h_parent, *gparent;
	struct inode *h_inode, *h_dir;
	struct au_dtime dt;
	umode_t mode;
	struct super_block *sb;
	struct au_hinode *hgdir, *hdir;
	unsigned int mnt_flags;
	const int do_dt = au_ftest_cpup(flags, DTIME);

	LKTRTrace("%.*s, i%lu, bdst %d, bsrc %d, len %lld, dtime %u\n",
		  AuDLNPair(dentry), dentry->d_inode->i_ino, bdst, bsrc, len,
		  do_dt);
	sb = dentry->d_sb;
	AuDebugOn(bdst >= bsrc || au_test_ro(sb, bdst, NULL));
	/* bsrc branch can be ro/rw. */

	h_src = au_h_dptr(dentry, bsrc);
	AuDebugOn(!h_src);
	h_inode = h_src->d_inode;
	AuDebugOn(!h_inode);
	AuDebugOn(h_inode != au_h_iptr(dentry->d_inode, bsrc));

	/* stop referencing while we are creating */
	h_dst = au_h_dptr(dentry, bdst);
	AuDebugOn(h_dst && h_dst->d_inode);
	h_parent = h_dst->d_parent; /* dir inode is locked */
	h_dir = h_parent->d_inode;
	IMustLock(h_dir);
	AuDebugOn(h_parent != h_dst->d_parent);

	hdir = NULL;
	mnt_flags = au_mntflags(sb);
	hinotify = !!au_opt_test(mnt_flags, UDBA_INOTIFY);
	if (unlikely(hinotify)) {
		hdir = au_hi(dst_parent->d_inode, bdst);
		AuDebugOn(hdir->hi_inode != h_dir);
	}

	if (do_dt) {
		hgdir = NULL;
		if (unlikely(hinotify && !IS_ROOT(dst_parent))) {
			gparent = dget_parent(dst_parent);
			hgdir = au_hi(gparent->d_inode, bdst);
			IMustLock(hgdir->hi_inode);
			dput(gparent);
		}
		au_dtime_store(&dt, dst_parent, h_parent, hdir, hgdir);
	}

	isdir = 0;
	vfsub_args_reinit(vargs);
	vfsub_ign_hinode(vargs, IN_CREATE, hdir);
	mode = h_inode->i_mode;
	switch (mode & S_IFMT) {
	case S_IFREG:
		/* stop updating while we are referencing */
		IMustLock(h_inode);
		err = au_h_create(h_dir, h_dst, mode | S_IWUSR, vargs, NULL,
				  au_nfsmnt(sb, bdst));
		if (!err)
			err = au_do_cpup_regular(dentry, bdst, bsrc, len,
						 hdir, h_dst, vargs);
		break;
	case S_IFDIR:
		isdir = 1;
		err = vfsub_mkdir(h_dir, h_dst, mode, vargs);
		if (!err) {
			/* setattr case: dir is not locked */
			if (0 && au_ibstart(dst_parent->d_inode) == bdst)
				au_cpup_attr_nlink(dst_parent->d_inode);
			au_cpup_attr_nlink(dentry->d_inode);
		}
		break;
	case S_IFLNK:
		err = au_do_cpup_symlink(h_dst, h_src, h_dir, mode, vargs);
		break;
	case S_IFCHR:
	case S_IFBLK:
		AuDebugOn(!capable(CAP_MKNOD));
		/*FALLTHROUGH*/
	case S_IFIFO:
	case S_IFSOCK:
		err = vfsub_mknod(h_dir, h_dst, mode,
				  au_h_rdev(h_inode, /*h_mnt*/NULL, h_src),
				  vargs);
		break;
	default:
		AuIOErr("Unknown inode type 0%o\n", mode);
		err = -EIO;
	}

	if (unlikely(hinotify
		     && !isdir
		     && au_opt_test_xino(mnt_flags)
		     && h_inode->i_nlink == 1
		     //&& dentry->d_inode->i_nlink == 1
		     && bdst < bsrc
		     && !au_ftest_cpup(flags, KEEPLINO)))
		au_xino_write0(sb, bsrc, h_inode->i_ino, /*ino*/0);
		/* ignore this error */

	if (do_dt)
		au_dtime_revert(&dt);
	AuTraceErr(err);
	return err;
}

/*
 * copyup the @dentry from @bsrc to @bdst.
 * the caller must set the both of hidden dentries.
 * @len is for truncating when it is -1 copyup the entire file.
 */
static int au_cpup_single(struct dentry *dentry, aufs_bindex_t bdst,
			  aufs_bindex_t bsrc, loff_t len, unsigned int flags,
			  struct dentry *dst_parent, struct vfsub_args *vargs)
{
	int err, rerr;
	unsigned int mnt_flags;
	aufs_bindex_t old_ibstart;
	unsigned char isdir, plink, hinotify;
	struct au_dtime dt;
	struct dentry *h_src, *h_dst, *h_parent, *gparent;
	struct inode *dst_inode, *h_dir, *inode;
	struct super_block *sb;
	struct au_hinode *hgdir, *hdir;

	LKTRTrace("%.*s, i%lu, bdst %d, bsrc %d, len %lld, flags 0x%x\n",
		  AuDLNPair(dentry), dentry->d_inode->i_ino, bdst, bsrc, len,
		  flags);
	sb = dentry->d_sb;
	AuDebugOn(bsrc <= bdst);
	h_dst = au_h_dptr(dentry, bdst);
	AuDebugOn(!h_dst || h_dst->d_inode);
	h_parent = h_dst->d_parent; /* dir inode is locked */
	h_dir = h_parent->d_inode;
	IMustLock(h_dir);
	h_src = au_h_dptr(dentry, bsrc);
	AuDebugOn(!h_src || !h_src->d_inode);
	inode = dentry->d_inode;
	IiMustWriteLock(inode);
	if (!dst_parent)
		dst_parent = dget_parent(dentry);
	else
		dget(dst_parent);

	mnt_flags = au_mntflags(sb);
	plink = !!au_opt_test(mnt_flags, PLINK);
	hinotify = !!au_opt_test(mnt_flags, UDBA_INOTIFY);
	hdir = NULL;
	if (unlikely(hinotify))
		hdir = au_hi(dst_parent->d_inode, bdst);
	dst_inode = au_h_iptr(inode, bdst);
	if (unlikely(dst_inode)) {
		if (unlikely(!plink)) {
			err = -EIO;
			AuIOErr("i%lu exists on a upper branch "
				"but plink is disabled\n", inode->i_ino);
			goto out;
		}

		if (dst_inode->i_nlink) {
			const int do_dt = au_ftest_cpup(flags, DTIME);

			h_src = au_plink_lkup(sb, bdst, inode);
			err = PTR_ERR(h_src);
			if (IS_ERR(h_src))
				goto out;
			AuDebugOn(!h_src->d_inode);

			if (do_dt) {
				hgdir = NULL;
				if (unlikely(hinotify && !IS_ROOT(dst_parent))) {
					gparent = dget_parent(dst_parent);
					hgdir = au_hi(gparent->d_inode, bdst);
					IMustLock(hgdir->hi_inode);
					dput(gparent);
				}
				au_dtime_store(&dt, dst_parent, h_parent, hdir,
					       hgdir);
			}
			vfsub_args_reinit(vargs);
			vfsub_ign_hinode(vargs, IN_CREATE, hdir);
			err = vfsub_link(h_src, h_dir, h_dst, vargs);
			if (do_dt)
				au_dtime_revert(&dt);
			dput(h_src);
			goto out;
		} else
			/* todo: cpup_wh_file? */
			/* udba work */
			au_update_brange(inode, 1);
	}

	old_ibstart = au_ibstart(inode);
	err = cpup_entry(dentry, bdst, bsrc, len, flags, dst_parent, vargs);
	if (unlikely(err))
		goto out;
	dst_inode = h_dst->d_inode;
	mutex_lock_nested(&dst_inode->i_mutex, AuLsc_I_CHILD2);

	/* todo: test dlgt? */
	err = cpup_iattr(dentry, bdst, h_src, hdir, vargs);
#if 0 /* reserved for future use */
	if (0 && !err)
		err = cpup_xattrs(h_src, h_dst);
#endif
	isdir = S_ISDIR(dst_inode->i_mode);
	if (!err) {
		if (bdst < old_ibstart)
			au_set_ibstart(inode, bdst);
		au_set_h_iptr(inode, bdst, au_igrab(dst_inode),
			      au_hi_flags(inode, isdir));
		mutex_unlock(&dst_inode->i_mutex);
		if (!isdir
		    && h_src->d_inode->i_nlink > 1
		    && plink)
			au_plink_append(sb, inode, h_dst, bdst);
		goto out; /* success */
	}

	/* revert */
	mutex_unlock(&dst_inode->i_mutex);
	hgdir = NULL;
	if (unlikely(au_opt_test(mnt_flags, UDBA_INOTIFY)
		     && !IS_ROOT(dst_parent))) {
		gparent = dget_parent(dst_parent);
		hgdir = au_hi(gparent->d_inode, bdst);
		dput(gparent);
	}
	au_dtime_store(&dt, dst_parent, h_parent, hdir, hgdir);
	vfsub_args_reinit(vargs);
	vfsub_ign_hinode(vargs, IN_DELETE, hdir);
	if (!isdir)
		rerr = vfsub_unlink(h_dir, h_dst, vargs);
	else
		rerr = vfsub_rmdir(h_dir, h_dst, vargs);
	au_dtime_revert(&dt);
	if (rerr) {
		AuIOErr("failed removing broken entry(%d, %d)\n", err, rerr);
		err = -EIO;
	}

 out:
	dput(dst_parent);
	AuTraceErr(err);
	return err;
}

struct au_cpup_single_args {
	int *errp;
	struct dentry *dentry;
	aufs_bindex_t bdst, bsrc;
	loff_t len;
	unsigned int flags;
	struct dentry *dst_parent;
	struct vfsub_args *vargs;
};

static void au_call_cpup_single(void *args)
{
	struct au_cpup_single_args *a = args;
	*a->errp = au_cpup_single(a->dentry, a->bdst, a->bsrc, a->len,
				  a->flags, a->dst_parent, a->vargs);
}

int au_sio_cpup_single(struct dentry *dentry, aufs_bindex_t bdst,
		       aufs_bindex_t bsrc, loff_t len, unsigned int flags,
		       struct dentry *dst_parent)
{
	int err, wkq_err;
	struct dentry *h_dentry;
	umode_t mode;
	struct au_hin_ignore ign;
	struct vfsub_args vargs;

	LKTRTrace("%.*s, i%lu, bdst %d, bsrc %d, len %lld, flags 0x%x\n",
		  AuDLNPair(dentry), dentry->d_inode->i_ino, bdst, bsrc, len,
		  flags);

	vfsub_args_init(&vargs, &ign, au_test_dlgt(au_mntflags(dentry->d_sb)),
			/*force_unlink*/0);
	h_dentry = au_h_dptr(dentry, bsrc);
	mode = h_dentry->d_inode->i_mode & S_IFMT;
	if ((mode != S_IFCHR && mode != S_IFBLK)
	    || capable(CAP_MKNOD))
		err = au_cpup_single(dentry, bdst, bsrc, len, flags,
				     dst_parent, &vargs);
	else {
		struct au_cpup_single_args args = {
			.errp		= &err,
			.dentry		= dentry,
			.bdst		= bdst,
			.bsrc		= bsrc,
			.len		= len,
			.flags		= flags,
			.dst_parent	= dst_parent,
			.vargs		= &vargs
		};
		vfsub_fclr(vargs.flags, DLGT);
		wkq_err = au_wkq_wait(au_call_cpup_single, &args, /*dlgt*/0);
		if (unlikely(wkq_err))
			err = wkq_err;
	}

	AuTraceErr(err);
	return err;
}

/*
 * copyup the @dentry from the first active hidden branch to @bdst,
 * using au_cpup_single().
 */
static int au_cpup_simple(struct dentry *dentry, aufs_bindex_t bdst, loff_t len,
			  unsigned int flags, struct vfsub_args *vargs)
{
	int err;
	struct inode *inode;
	aufs_bindex_t bsrc, bend;

	LKTRTrace("%.*s, bdst %d, len %lld, flags 0x%x\n",
		  AuDLNPair(dentry), bdst, len, flags);
	inode = dentry->d_inode;
	AuDebugOn(!S_ISDIR(inode->i_mode) && au_dbstart(dentry) < bdst);

	bend = au_dbend(dentry);
	for (bsrc = bdst + 1; bsrc <= bend; bsrc++)
		if (au_h_dptr(dentry, bsrc))
			break;
	AuDebugOn(!au_h_dptr(dentry, bsrc));

	err = au_lkup_neg(dentry, bdst);
	if (!err) {
		err = au_cpup_single(dentry, bdst, bsrc, len, flags, NULL,
				     vargs);
		if (!err)
			return 0; /* success */

		/* revert */
		au_set_h_dptr(dentry, bdst, NULL);
		au_set_dbstart(dentry, bsrc);
	}

	AuTraceErr(err);
	return err;
}

struct au_cpup_simple_args {
	int *errp;
	struct dentry *dentry;
	aufs_bindex_t bdst;
	loff_t len;
	unsigned int flags;
	struct vfsub_args *vargs;
};

static void au_call_cpup_simple(void *args)
{
	struct au_cpup_simple_args *a = args;
	*a->errp = au_cpup_simple(a->dentry, a->bdst, a->len, a->flags,
				  a->vargs);
}

int au_sio_cpup_simple(struct dentry *dentry, aufs_bindex_t bdst, loff_t len,
		       unsigned int flags)
{
	int err, wkq_err;
	unsigned char do_sio, dlgt;
	struct dentry *parent;
	struct inode *h_dir, *dir;
	struct au_hin_ignore ign;
	struct vfsub_args vargs;

	LKTRTrace("%.*s, b%d, len %lld, flags 0x%x\n",
		  AuDLNPair(dentry), bdst, len, flags);

	parent = dget_parent(dentry);
	dir = parent->d_inode;
	h_dir = au_h_iptr(dir, bdst);
	dlgt = !!au_test_dlgt(au_mntflags(dir->i_sb));
	do_sio = !!au_test_h_perm_sio(h_dir, MAY_EXEC | MAY_WRITE, dlgt);
	if (!do_sio) {
		/*
		 * testing CAP_MKNOD is for generic fs,
		 * but CAP_FSETID is for xfs only, currently.
		 */
		umode_t mode = dentry->d_inode->i_mode;
		do_sio = (((mode & (S_IFCHR | S_IFBLK))
			   && !capable(CAP_MKNOD))
			  || ((mode & (S_ISUID | S_ISGID))
			      && !capable(CAP_FSETID)));
	}
	vfsub_args_init(&vargs, &ign, dlgt, /*force_unlink*/0);
	if (!do_sio)
		err = au_cpup_simple(dentry, bdst, len, flags, &vargs);
	else {
		struct au_cpup_simple_args args = {
			.errp		= &err,
			.dentry		= dentry,
			.bdst		= bdst,
			.len		= len,
			.flags		= flags,
			.vargs		= &vargs
		};
		vfsub_fclr(vargs.flags, DLGT);
		wkq_err = au_wkq_wait(au_call_cpup_simple, &args, /*dlgt*/0);
		if (unlikely(wkq_err))
			err = wkq_err;
	}

	dput(parent);
	AuTraceErr(err);
	return err;
}

/* ---------------------------------------------------------------------- */

static int au_do_cpup_wh(struct dentry *dentry, aufs_bindex_t bdst,
			 struct dentry *wh_dentry, struct file *file,
			 loff_t len, struct vfsub_args *vargs)
{
	int err;
	struct au_dinfo *dinfo;
	aufs_bindex_t bstart;
	struct dentry *h_d_bdst, *h_d_bstart;

	AuTraceEnter();

	dinfo = au_di(dentry);
	bstart = dinfo->di_bstart;
	h_d_bdst = dinfo->di_hdentry[0 + bdst].hd_dentry;
	dinfo->di_bstart = bdst;
	dinfo->di_hdentry[0 + bdst].hd_dentry = wh_dentry;
	h_d_bstart = dinfo->di_hdentry[0 + bstart].hd_dentry;
	if (file)
		dinfo->di_hdentry[0 + bstart].hd_dentry
			= au_h_fptr(file, au_fbstart(file))->f_dentry;
	err = au_cpup_single(dentry, bdst, bstart, len, !AuCpup_DTIME,
			     /*h_parent*/NULL, vargs);
	if (!err && file) {
		err = au_reopen_nondir(file);
		dinfo->di_hdentry[0 + bstart].hd_dentry = h_d_bstart;
	}
	dinfo->di_hdentry[0 + bdst].hd_dentry = h_d_bdst;
	dinfo->di_bstart = bstart;

	AuTraceErr(err);
	return err;
}

/*
 * copyup the deleted file for writing.
 */
static int au_cpup_wh(struct dentry *dentry, aufs_bindex_t bdst, loff_t len,
		      struct file *file)
{
	int err;
	unsigned char dlgt;
	struct dentry *parent, *h_parent, *wh_dentry;
	struct super_block *sb;
	unsigned int mnt_flags;
	struct au_dtime dt;
	struct au_hin_ignore ign;
	struct vfsub_args vargs;
	struct au_hinode *hgdir, *hdir;
	struct au_ndx ndx = {
		.nd	= NULL,
		.flags	= 0,
		/* .br	= NULL */
	};

	LKTRTrace("%.*s, bdst %d, len %llu\n", AuDLNPair(dentry), bdst, len);
	AuDebugOn(S_ISDIR(dentry->d_inode->i_mode)
		  || (file && !(file->f_mode & FMODE_WRITE)));
	DiMustWriteLock(dentry);

	parent = dget_parent(dentry);
	IiMustAnyLock(parent->d_inode);
	h_parent = au_h_dptr(parent, bdst);
	AuDebugOn(!h_parent);

	sb = parent->d_sb;
	mnt_flags = au_mntflags(sb);
	dlgt = 0;
	ndx.nfsmnt = au_nfsmnt(sb, bdst);
	if (unlikely(au_test_dlgt(mnt_flags))) {
		dlgt = 1;
		au_fset_ndx(ndx.flags, DLGT);
	}
	wh_dentry = au_whtmp_lkup(h_parent, &dentry->d_name, &ndx);
	err = PTR_ERR(wh_dentry);
	if (IS_ERR(wh_dentry))
		goto out;

	hdir = NULL;
	hgdir = NULL;
	if (unlikely(au_opt_test(mnt_flags, UDBA_INOTIFY))) {
		hdir = au_hi(parent->d_inode, bdst);
		if (!IS_ROOT(parent)) {
			struct dentry *gparent;
			gparent = dget_parent(parent);
			hgdir = au_hi(gparent->d_inode, bdst);
			dput(gparent);
		}
	}
	au_dtime_store(&dt, parent, h_parent, hdir, hgdir);
	vfsub_args_init(&vargs, &ign, dlgt, /*force_unlink*/0);
	err = au_do_cpup_wh(dentry, bdst, wh_dentry, file, len, &vargs);
	if (unlikely(err))
		goto out_wh;

	AuDebugOn(!d_unhashed(dentry));
	/* dget first to force sillyrename on nfs */
	dget(wh_dentry);
	vfsub_args_reinit(&vargs);
	vfsub_ign_hinode(&vargs, IN_DELETE, hdir);
	err = vfsub_unlink(h_parent->d_inode, wh_dentry, &vargs);
	if (unlikely(err)) {
		AuIOErr("failed remove copied-up tmp file %.*s(%d)\n",
			AuDLNPair(wh_dentry), err);
		err = -EIO;
	}
	au_dtime_revert(&dt);
	au_set_hi_wh(dentry->d_inode, bdst, wh_dentry);

 out_wh:
	dput(wh_dentry);
 out:
	dput(parent);
	AuTraceErr(err);
	return err;
}

struct au_cpup_wh_args {
	int *errp;
	struct dentry *dentry;
	aufs_bindex_t bdst;
	loff_t len;
	struct file *file;
};

static void au_call_cpup_wh(void *args)
{
	struct au_cpup_wh_args *a = args;
	*a->errp = au_cpup_wh(a->dentry, a->bdst, a->len, a->file);
}

int au_sio_cpup_wh(struct dentry *dentry, aufs_bindex_t bdst, loff_t len,
		   struct file *file)
{
	int err, wkq_err;
	struct dentry *parent, *h_tmp, *h_parent;
	struct inode *dir, *h_dir, *h_tmpdir;
	struct au_wbr *wbr;

	AuTraceEnter();
	parent = dget_parent(dentry);
	dir = parent->d_inode;
	IiMustAnyLock(dir);

	h_tmp = NULL;
	h_parent = NULL;
	h_dir = au_igrab(au_h_iptr(dir, bdst));
	h_tmpdir = h_dir;
	if (unlikely(!h_dir->i_nlink)) {
		DiMustWriteLock(parent);
		wbr = au_sbr(dentry->d_sb, bdst)->br_wbr;
		AuDebugOn(!wbr);
		h_tmp = wbr->wbr_tmp;

		h_parent = dget(au_h_dptr(parent, bdst));
		au_set_h_dptr(parent, bdst, NULL);
		au_set_h_dptr(parent, bdst, dget(h_tmp));
		h_tmpdir = h_tmp->d_inode;
		au_set_h_iptr(dir, bdst, NULL, 0);
		au_set_h_iptr(dir, bdst, au_igrab(h_tmpdir), /*flags*/0);
		mutex_lock_nested(&h_tmpdir->i_mutex, AuLsc_I_PARENT3);
	}

	if (!au_test_h_perm_sio
	    (h_tmpdir, MAY_EXEC | MAY_WRITE,
	     au_test_dlgt(au_mntflags(dentry->d_sb))))
		err = au_cpup_wh(dentry, bdst, len, file);
	else {
		struct au_cpup_wh_args args = {
			.errp	= &err,
			.dentry	= dentry,
			.bdst	= bdst,
			.len	= len,
			.file	= file
		};
		wkq_err = au_wkq_wait(au_call_cpup_wh, &args, /*dlgt*/0);
		if (unlikely(wkq_err))
			err = wkq_err;
	}

	/* todo: is this restore safe? */
	if (unlikely(h_tmp)) {
		mutex_unlock(&h_tmpdir->i_mutex);
		au_set_h_iptr(dir, bdst, NULL, 0);
		au_set_h_iptr(dir, bdst, au_igrab(h_dir), /*flags*/0);
		au_set_h_dptr(parent, bdst, NULL);
		au_set_h_dptr(parent, bdst, h_parent);
	}
	iput(h_dir);
	dput(parent);

	AuTraceErr(err);
	return err;
}

/* ---------------------------------------------------------------------- */

/*
 * generic routine for both of copy-up and copy-down.
 * Although I've tried building a path by dcsub, I gave up this approach.
 * Since the ancestor directory may be moved/renamed during copy.
 */
/* cf. revalidate function in file.c */
int au_cp_dirs(struct dentry *dentry, aufs_bindex_t bdst,
	       int (*cp)(struct dentry *dentry, aufs_bindex_t bdst,
			 struct dentry *h_parent, void *arg),
	       void *arg)
{
	int err, hinotify;
	struct super_block *sb;
	struct dentry *d, *parent, *h_parent, *real_parent;
	struct au_pin pin;

	LKTRTrace("%.*s, b%d, parent i%lu\n",
		  AuDLNPair(dentry), bdst, (unsigned long)parent_ino(dentry));
	sb = dentry->d_sb;
	AuDebugOn(au_test_ro(sb, bdst, NULL));
	err = 0;
	parent = dget_parent(dentry);
	IiMustWriteLock(parent->d_inode);
	if (unlikely(IS_ROOT(parent)))
		goto out;

	/* do not use au_dpage */
	real_parent = parent;
	hinotify = !!au_opt_test(au_mntflags(sb), UDBA_INOTIFY);
	while (1) {
		dput(parent);
		parent = dget_parent(dentry);
		h_parent = au_h_dptr(parent, bdst);
		if (h_parent)
			goto out; /* success */

		/* find top dir which is needed to cpup */
		do {
			d = parent;
			dput(parent);
			parent = dget_parent(d);
			di_read_lock_parent3(parent, !AuLock_IR);
			h_parent = au_h_dptr(parent, bdst);
			di_read_unlock(parent, !AuLock_IR);
		} while (!h_parent);

		if (d != real_parent)
			di_write_lock_child3(d);

		/* somebody else might create while we were sleeping */
		if (!au_h_dptr(d, bdst) || !au_h_dptr(d, bdst)->d_inode) {
			if (au_h_dptr(d, bdst))
				au_update_dbstart(d);

			au_pin_init(&pin, d, /*di_locked*/0,
				    AuLsc_DI_PARENT3, AuLsc_I_PARENT2, hinotify);
			err = au_do_pin(pin.pin + AuPin_PARENT, au_pin_gp(&pin),
					bdst, hinotify);
			if (!err) {
				err = cp(d, bdst, h_parent, arg);
				au_unpin(&pin);
			}
		}

		if (d != real_parent)
			di_write_unlock(d);
		if (unlikely(err))
			break;
	}

 out:
	dput(parent);
	AuTraceErr(err);
	return err;
}

static int au_cpup_dir(struct dentry *dentry, aufs_bindex_t bdst,
		       struct dentry *h_parent, void *arg)
{
	int err;

	err = au_sio_cpup_simple(dentry, bdst, -1, AuCpup_DTIME);

	AuTraceErr(err);
	return err;
}

int au_cpup_dirs(struct dentry *dentry, aufs_bindex_t bdst)
{
	int err;

	err = au_cp_dirs(dentry, bdst, au_cpup_dir, NULL);

	AuTraceErr(err);
	return err;
}

int au_test_and_cpup_dirs(struct dentry *dentry, aufs_bindex_t bdst)
{
	int err;
	struct dentry *parent;
	struct inode *dir;

	parent = dget_parent(dentry);
	dir = parent->d_inode;
	LKTRTrace("%.*s, b%d, parent i%lu\n",
		  AuDLNPair(dentry), bdst, dir->i_ino);
	DiMustReadLock(parent);
	IiMustReadLock(dir);

	err = 0;
	if (au_h_iptr(dir, bdst))
		goto out;

	di_read_unlock(parent, AuLock_IR);
	di_write_lock_parent(parent);
	/* someone else might change our inode while we were sleeping */
	if (unlikely(!au_h_iptr(dir, bdst)))
		err = au_cpup_dirs(dentry, bdst);
	di_downgrade_lock(parent, AuLock_IR);

 out:
	dput(parent);
	AuTraceErr(err);
	return err;
}
