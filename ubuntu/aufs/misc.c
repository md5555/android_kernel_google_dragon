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
 * $Id: misc.c,v 1.17 2008/09/08 02:40:48 sfjro Exp $
 */

#include "aufs.h"

void *au_kzrealloc(void *p, unsigned int nused, unsigned int new_sz, gfp_t gfp)
{
	void *q;

	LKTRTrace("p %p, nused %d, sz %d\n", p, nused, new_sz);
	AuDebugOn(new_sz <= 0);
	if (new_sz <= nused)
		return p;

	q = krealloc(p, new_sz, gfp);
	if (q)
		memset(q + nused, 0, new_sz - nused);
	return q;
}

/* ---------------------------------------------------------------------- */

struct nameidata *au_dup_nd(struct au_sbinfo *sbinfo, struct nameidata *dst,
			    struct nameidata *src)
{
	LKTRTrace("src %p\n", src);

	if (src) {
		*dst = *src;
		dst->flags &= ~LOOKUP_PARENT;
		if (sbinfo->si_wbr_create == AuWbrCreate_TDP) {
			if ((dst->flags & LOOKUP_CREATE)
			    && !(dst->intent.open.flags & O_CREAT))
				dst->flags &= ~LOOKUP_CREATE;
		} else {
			dst->flags &= ~LOOKUP_CREATE;
			dst->intent.open.flags &= ~O_CREAT;
		}
	} else
		dst = NULL;

	return dst;
}

struct nameidata *au_fake_dm(struct nameidata *fake_nd, struct nameidata *nd,
			     struct super_block *sb, aufs_bindex_t bindex)
{
	LKTRTrace("nd %p, b%d\n", nd, bindex);

	if (!nd)
		return NULL;

	DiMustAnyLock(nd->path.dentry);

	fake_nd->path.dentry = NULL;
	fake_nd->path.mnt = NULL;

	if (bindex <= au_dbend(nd->path.dentry))
		fake_nd->path.dentry = au_h_dptr(nd->path.dentry, bindex);
	if (fake_nd->path.dentry) {
		fake_nd->path.mnt = au_sbr_mnt(sb, bindex);
		AuDebugOn(!fake_nd->path.mnt);
		path_get(&fake_nd->path);
	} else
		fake_nd = ERR_PTR(-ENOENT);

	AuTraceErrPtr(fake_nd);
	return fake_nd;
}

void au_fake_dm_release(struct nameidata *fake_nd)
{
	if (fake_nd)
		path_put(&fake_nd->path);
}

int au_h_create(struct inode *h_dir, struct dentry *h_dentry, int mode,
		struct vfsub_args *vargs, struct nameidata *nd,
		struct vfsmount *nfsmnt)
{
	int err;

	LKTRTrace("hi%lu, %.*s, 0%o, nd %d, nfsmnt %d\n",
		  h_dir->i_ino, AuDLNPair(h_dentry), mode, !!nd, !!nfsmnt);

	err = -ENOSYS;
	if (!nfsmnt)
		err = vfsub_create(h_dir, h_dentry, mode, /*nd*/NULL, vargs);
	else {
		struct nameidata fake_nd;

		if (nd)
			fake_nd = *nd;
		else
			memset(&fake_nd, 0, sizeof(fake_nd));
		fake_nd.path.dentry = h_dentry;
		fake_nd.path.mnt = nfsmnt;
		path_get(&fake_nd.path);
		fake_nd.flags = LOOKUP_CREATE;
		fake_nd.intent.open.flags = O_CREAT | FMODE_READ;
		fake_nd.intent.open.create_mode = mode;

		err = vfsub_create(h_dir, h_dentry, mode, &fake_nd, vargs);
		path_put(&fake_nd.path);
	}

	AuTraceErr(err);
	return err;
}

/* ---------------------------------------------------------------------- */

int au_copy_file(struct file *dst, struct file *src, loff_t len,
		 struct au_hinode *hdir, struct super_block *sb,
		 struct vfsub_args *vargs)
{
	int err, all_zero, do_kfree;
	unsigned long blksize;
	char *buf, *zp;
	/* reduce stack usage */
	struct iattr *ia;

	LKTRTrace("%.*s, %.*s\n",
		  AuDLNPair(dst->f_dentry), AuDLNPair(src->f_dentry));
	AuDebugOn(!(dst->f_mode & FMODE_WRITE));
#ifdef CONFIG_AUFS_DEBUG
	{
		struct dentry *parent;
		parent = dget_parent(dst->f_dentry);
		IMustLock(parent->d_inode);
		dput(parent);
	}
#endif

	err = -ENOMEM;
	blksize = dst->f_dentry->d_sb->s_blocksize;
	if (!blksize || PAGE_SIZE < blksize)
		blksize = PAGE_SIZE;
	LKTRTrace("blksize %lu\n", blksize);
	/* todo: use ZERO_PAGE(0) */
	BUILD_BUG_ON(KMALLOC_MAX_SIZE < 128 << 10);
	do_kfree = 1;
	if (blksize <= 64 << 10 && blksize * 2 >= sizeof(*ia)) {
		buf = kmalloc(blksize * 2, GFP_NOFS);
		if (unlikely(!buf))
			goto out;
		zp = buf + blksize;
		memset(zp, 0, blksize);
	} else {
		BUILD_BUG_ON(PAGE_SIZE * 2 < sizeof(*ia));
#if 0
		buf = (void *)__get_free_pages(GFP_NOFS, 1);
		zp = buf + PAGE_SIZE;
#endif
		do_kfree = 0;
		buf = (void *)__get_free_page(GFP_NOFS);
		if (unlikely(!buf))
			goto out;
		zp = (void *)get_zeroed_page(GFP_NOFS);
		if (unlikely(!zp))
			goto out_buf;
	}

#ifdef CONFIG_AUFS_DEBUG
	if (len > (1 << 22))
		AuWarn("copying a large file %lld\n", (long long)len);
#endif
	err = 0;
	all_zero = 0;
	src->f_pos = 0;
	dst->f_pos = 0;
	while (len) {
		size_t sz, rbytes, wbytes;
		char *p;

		LKTRTrace("len %lld\n", len);
		sz = blksize;
		if (len < blksize)
			sz = len;

		/* support LSM and notify */
		rbytes = 0;
		/* todo: signal_pending? */
		while (!rbytes || err == -EAGAIN || err == -EINTR) {
			rbytes = vfsub_read_k(src, buf, sz, &src->f_pos,
					      vfsub_ftest(vargs->flags, DLGT));
			err = rbytes;
		}
		if (unlikely(err < 0))
			break;

		all_zero = 0;
		if (len >= rbytes && rbytes == blksize) {
#if 1
			all_zero = !memcmp(buf, zp, rbytes);
#else /* reserved for future use */
			unsigned long long *ullp;
			size_t n, i;

			all_zero = 1;
			ullp = (void *)buf;
			n = rbytes / sizeof(*ullp);
			i = n;
			while (n-- > 0 && all_zero)
				all_zero = !*ullp++;
			p = (void *)ullp;
			i *= sizeof(*ullp);
			for (; all_zero && i < rbytes; i++)
				all_zero = !*p++;
#endif
		}
		if (!all_zero) {
			wbytes = rbytes;
			p = buf;
			while (wbytes) {
				size_t b;
				/* support LSM and notify */
				vfsub_args_reinit(vargs);
				vfsub_ign_hinode(vargs, IN_MODIFY, hdir);
				b = vfsub_write_k(dst, p, wbytes, &dst->f_pos,
						  vargs);
				err = b;
				/* todo: signal_pending? */
				if (unlikely(err == -EAGAIN || err == -EINTR))
					continue;
				if (unlikely(err < 0))
					break;
				wbytes -= b;
				p += b;
			}
		} else {
			loff_t res;
			LKTRLabel(hole);
			res = vfsub_llseek(dst, rbytes, SEEK_CUR);
			err = res;
			if (unlikely(res < 0))
				break;
		}
		len -= rbytes;
		err = 0;
	}

	/* the last block may be a hole */
	if (unlikely(!err && all_zero)) {
		struct dentry *h_d = dst->f_dentry;
		struct inode *h_i = h_d->d_inode;

		LKTRLabel(last hole);
		do {
			/* todo: signal_pending? */
			vfsub_args_reinit(vargs);
			vfsub_ign_hinode(vargs, IN_MODIFY, hdir);
			err = vfsub_write_k(dst, "\0", 1, &dst->f_pos, vargs);
		} while (err == -EAGAIN || err == -EINTR);
		if (err == 1) {
			ia = (void *)buf;
			ia->ia_size = dst->f_pos;
			ia->ia_valid = ATTR_SIZE | ATTR_FILE;
			ia->ia_file = dst;
			vfsub_args_reinit(vargs);
			vfsub_ign_hinode(vargs, vfsub_events_notify_change(ia),
					 hdir);
			mutex_lock_nested(&h_i->i_mutex, AuLsc_I_CHILD2);
			err = vfsub_notify_change(h_d, ia, vargs);
			mutex_unlock(&h_i->i_mutex);
		}
	}
	if (do_kfree)
		kfree(buf);
	else
		free_page((unsigned long)zp);

 out_buf:
	if (unlikely(!do_kfree))
		free_page((unsigned long)buf);
 out:
	AuTraceErr(err);
	return err;
}
