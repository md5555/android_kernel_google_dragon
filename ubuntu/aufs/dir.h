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
 * directory operations
 *
 * $Id: dir.h,v 1.3 2008/05/26 04:04:23 sfjro Exp $
 */

#ifndef __AUFS_DIR_H__
#define __AUFS_DIR_H__

#ifdef __KERNEL__

#include <linux/fs.h>
#include <linux/aufs_type.h>

/* ---------------------------------------------------------------------- */

/* need to be faster and smaller */

/* todo: changeable? */
#define AuSize_DEBLK	512
#define AuSize_NHASH	32
#if AuSize_DEBLK < NAME_MAX || PAGE_SIZE < AuSize_DEBLK
#error invalid size AuSize_DEBLK
#endif

typedef char au_vdir_deblk_t[AuSize_DEBLK];

struct au_nhash {
	struct hlist_head heads[AuSize_NHASH];
};

struct au_vdir_destr {
	unsigned char	len;
	char		name[0];
} __packed;

struct au_vdir_dehstr {
	struct hlist_node	hash;
	struct au_vdir_destr	*str;
};

struct au_vdir_de {
	ino_t			de_ino;
	unsigned char		de_type;
	/* caution: packed */
	struct au_vdir_destr	de_str;
} __packed;

struct au_vdir_wh {
	struct hlist_node	wh_hash;
	aufs_bindex_t		wh_bindex;
#ifdef CONFIG_AUFS_SHWH
	ino_t			wh_ino;
	unsigned char		wh_type;
	/* caution: packed */
#endif
	struct au_vdir_destr	wh_str;
} __packed;

union au_vdir_deblk_p {
	unsigned char		*p;
	au_vdir_deblk_t		*deblk;
	struct au_vdir_de	*de;
};

struct au_vdir {
	au_vdir_deblk_t	**vd_deblk;
	int		vd_nblk;
	struct {
		int			i;
		union au_vdir_deblk_p	p;
	} vd_last;

	unsigned long	vd_version;
	unsigned long	vd_jiffy;
};

/* ---------------------------------------------------------------------- */

/* dir.c */
extern struct file_operations aufs_dir_fop;
int au_test_empty_lower(struct dentry *dentry);
int au_test_empty(struct dentry *dentry, struct au_nhash *whlist);

/* vdir.c */
struct au_nhash *au_nhash_new(gfp_t gfp);
void au_nhash_del(struct au_nhash *nhash);
void au_nhash_init(struct au_nhash *nhash);
void au_nhash_move(struct au_nhash *dst, struct au_nhash *src);
void au_nhash_fin(struct au_nhash *nhash);
int au_nhash_test_longer_wh(struct au_nhash *whlist, aufs_bindex_t btgt,
			    int limit);
int au_nhash_test_known_wh(struct au_nhash *whlist, char *name, int namelen);
int au_nhash_append_wh(struct au_nhash *whlist, char *name, int namelen,
		       ino_t ino, unsigned int d_type, aufs_bindex_t bindex,
		    unsigned char shwh);
void au_vdir_free(struct au_vdir *vdir);
int au_vdir_init(struct file *file);
int au_vdir_fill_de(struct file *file, void *dirent, filldir_t filldir);

/* ---------------------------------------------------------------------- */

static inline
void au_shwh_init_wh(struct au_vdir_wh *wh, ino_t ino, unsigned char d_type)
{
#ifdef CONFIG_AUFS_SHWH
	wh->wh_ino = ino;
	wh->wh_type = d_type;
#endif
}

static inline void au_add_nlink(struct inode *dir, struct inode *h_dir)
{
	AuDebugOn(!S_ISDIR(dir->i_mode) || !S_ISDIR(h_dir->i_mode));
	dir->i_nlink += h_dir->i_nlink - 2;
	if (unlikely(h_dir->i_nlink < 2))
		dir->i_nlink += 2;
}

static inline void au_sub_nlink(struct inode *dir, struct inode *h_dir)
{
	AuDebugOn(!S_ISDIR(dir->i_mode) || !S_ISDIR(h_dir->i_mode));
	dir->i_nlink -= h_dir->i_nlink - 2;
	if (unlikely(h_dir->i_nlink < 2))
		dir->i_nlink -= 2;
}

#endif /* __KERNEL__ */
#endif /* __AUFS_DIR_H__ */
