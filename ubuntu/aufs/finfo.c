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
 * file private data
 *
 * $Id: finfo.c,v 1.4 2008/06/30 03:50:21 sfjro Exp $
 */

#include "aufs.h"

struct au_finfo *au_fi(struct file *file)
{
	struct au_finfo *finfo = file->private_data;
	AuDebugOn(!finfo
		  || !finfo->fi_hfile
		  || (0 < finfo->fi_bend
		      && (/* au_sbi(file->f_dentry->d_sb)->si_bend
			     < finfo->fi_bend
			     || */ finfo->fi_bend < finfo->fi_bstart)));
	return finfo;
}

struct au_branch *au_fbr(struct file *file, aufs_bindex_t bindex)
{
	struct au_finfo *finfo = au_fi(file);
	struct au_hfile *hf;

	FiMustAnyLock(file);
	AuDebugOn(!finfo
		  || finfo->fi_bstart < 0
		  || bindex < finfo->fi_bstart
		  || finfo->fi_bend < bindex);
	hf = finfo->fi_hfile + bindex;
	AuDebugOn(hf->hf_br && au_br_count(hf->hf_br) <= 0);
	return hf->hf_br;
}

struct file *au_h_fptr(struct file *file, aufs_bindex_t bindex)
{
	struct au_finfo *finfo = au_fi(file);
	struct au_hfile *hf;

	FiMustAnyLock(file);
	AuDebugOn(!finfo
		  || finfo->fi_bstart < 0
		  || bindex < finfo->fi_bstart
		  || finfo->fi_bend < bindex);
	hf = finfo->fi_hfile + bindex;
	AuDebugOn(hf->hf_file
		  && file_count(hf->hf_file) <= 0
		  && au_br_count(hf->hf_br) <= 0);
	return hf->hf_file;
}

void au_hfput(struct au_hfile *hf)
{
	if (hf->hf_file->f_mode & FMODE_EXEC)
		au_allow_write_access(hf->hf_file);
	fput(hf->hf_file);
	hf->hf_file = NULL;
	AuDebugOn(!hf->hf_br);
	au_br_put(hf->hf_br);
	hf->hf_br = NULL;
}

void au_set_h_fptr(struct file *file, aufs_bindex_t bindex, struct file *val)
{
	struct au_finfo *finfo = au_fi(file);
	struct au_hfile *hf;

	FiMustWriteLock(file);
	AuDebugOn(!finfo
		  || finfo->fi_bstart < 0
		  || bindex < finfo->fi_bstart
		  || finfo->fi_bend < bindex);
	AuDebugOn(val && file_count(val) <= 0);
	hf = finfo->fi_hfile + bindex;
	AuDebugOn(val && hf->hf_file);
	if (hf->hf_file)
		au_hfput(hf);
	if (val) {
		hf->hf_file = val;
		hf->hf_br = au_sbr(file->f_dentry->d_sb, bindex);
	}
}

void au_finfo_fin(struct file *file)
{
	struct au_finfo *finfo;
	struct dentry *dentry;
	aufs_bindex_t bindex, bend;

	dentry = file->f_dentry;
	LKTRTrace("%.*s\n", AuDLNPair(dentry));
	SiMustAnyLock(dentry->d_sb);

	fi_write_lock(file);
	bend = au_fbend(file);
	bindex = au_fbstart(file);
	if (bindex >= 0)
		for (; bindex <= bend; bindex++)
			au_set_h_fptr(file, bindex, NULL);

	finfo = au_fi(file);
#ifdef CONFIG_AUFS_DEBUG
	if (finfo->fi_bstart >= 0) {
		bend = au_fbend(file);
		for (bindex = finfo->fi_bstart; bindex <= bend; bindex++) {
			struct au_hfile *hf;
			hf = finfo->fi_hfile + bindex;
			AuDebugOn(hf->hf_file || hf->hf_br);
		}
	}
#endif

	kfree(finfo->fi_hfile);
	fi_write_unlock(file);
	au_cache_free_finfo(finfo);
}

int au_finfo_init(struct file *file)
{
	struct au_finfo *finfo;
	struct dentry *dentry;
	union {
		void *p;
		unsigned long ul;
	} u;

	dentry = file->f_dentry;
	LKTRTrace("%.*s\n", AuDLNPair(dentry));
	AuDebugOn(!dentry->d_inode);

	finfo = au_cache_alloc_finfo();
	if (finfo) {
		finfo->fi_hfile = kcalloc(au_sbend(dentry->d_sb) + 1,
					  sizeof(*finfo->fi_hfile), GFP_NOFS);
		if (finfo->fi_hfile) {
			au_rw_init_wlock(&finfo->fi_rwsem);
			finfo->fi_bstart = -1;
			finfo->fi_bend = -1;
			atomic_set(&finfo->fi_generation, au_digen(dentry));
			/* smp_mb(); */ /* atomic_set */

			/*
			 * a dirty trick for handling FMODE_EXEC and
			 * deny_write_access().
			 * because FMODE_EXEC flag is not passed to
			 * f_op->open(),
			 * aufs set it to file->private_data temporary in lookup
			 * or dentry revalidation operations.
			 * restore the flag to f_mode here.
			 */
			u.p = file->private_data;
			if (u.ul & FMODE_EXEC) {
				file->f_mode |= FMODE_EXEC;
				smp_mb(); /* flush f_mode */
			}

			file->private_data = finfo;
			return 0; /* success */
		}
		au_cache_free_finfo(finfo);
	}

	AuTraceErr(-ENOMEM);
	return -ENOMEM;
}
