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
 * 'robr', aufs as readonly branch of another aufs
 *
 * $Id: robr.c,v 1.6 2008/07/21 02:53:51 sfjro Exp $
 */

#include "aufs.h"

/* ---------------------------------------------------------------------- */

int au_test_robr_wh(struct qstr *name, struct dentry *h_parent,
		    struct qstr *wh_name, int try_sio, struct au_ndx *ndx)
{
	if (strncmp(name->name, AUFS_WH_PFX, AUFS_WH_PFX_LEN))
		return au_wh_test(h_parent, wh_name, try_sio, ndx);
	return -EPERM;
}

int au_test_robr_shwh(struct super_block *sb, const struct qstr *name)
{
	return 0;
}

/* ---------------------------------------------------------------------- */

struct au_robr_lvma {
	struct list_head list;
	struct vm_area_struct *vma;
};

struct file *au_robr_safe_file(struct vm_area_struct *vma)
{
	struct file *file = vma->vm_file;
	struct super_block *sb = file->f_dentry->d_sb;
	struct au_robr_lvma *lvma, *entry;
	struct au_sbinfo *sbinfo;
	unsigned char found, warn;

	AuTraceEnter();
	AuDebugOn(!au_test_aufs(sb));

	warn = 0;
	found = 0;
	sbinfo = au_sbi(sb);
	spin_lock(&sbinfo->si_lvma_lock);
	list_for_each_entry(entry, &sbinfo->si_lvma, list) {
		found = (entry->vma == vma);
		if (unlikely(found))
			break;
	}
	if (!found) {
		lvma = kmalloc(sizeof(*lvma), GFP_ATOMIC);
		if (lvma) {
			lvma->vma = vma;
			list_add(&lvma->list, &sbinfo->si_lvma);
		} else {
			warn = 1;
			file = NULL;
		}
	} else
		file = NULL;
	spin_unlock(&sbinfo->si_lvma_lock);

	if (unlikely(warn))
		AuWarn1("no memory for lvma\n");
	return file;
}

void au_robr_reset_file(struct vm_area_struct *vma, struct file *file)
{
	struct super_block *sb = file->f_dentry->d_sb;
	struct au_robr_lvma *entry, *found;
	struct au_sbinfo *sbinfo;

	AuTraceEnter();
	AuDebugOn(!au_test_aufs(sb));

	vma->vm_file = file;
	/* smp_mb(); */ /* flush vm_file */

	found = NULL;
	sbinfo = au_sbi(sb);
	spin_lock(&sbinfo->si_lvma_lock);
	list_for_each_entry(entry, &sbinfo->si_lvma, list)
		if (entry->vma == vma) {
			found = entry;
			break;
		}
	AuDebugOn(!found);
	list_del(&found->list);
	spin_unlock(&sbinfo->si_lvma_lock);
	kfree(found);
}
