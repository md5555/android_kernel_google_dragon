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
 * module initialization and module-global
 *
 * $Id: module.h,v 1.8 2008/08/25 01:50:37 sfjro Exp $
 */

#ifndef __AUFS_MODULE_H__
#define __AUFS_MODULE_H__

#ifdef __KERNEL__

/* module parameters */
extern short aufs_nwkq;
extern int sysaufs_brs;

/* ---------------------------------------------------------------------- */

extern char au_esc_chars[];
extern int au_dir_roflags;

/* kmem cache */
enum {
	AuCache_DINFO,
	AuCache_ICNTNR,
	AuCache_FINFO,
	AuCache_VDIR,
	AuCache_DEHSTR,
#ifdef CONFIG_AUFS_HINOTIFY
	AuCache_HINOTIFY,
#endif
	AuCache_Last
};

extern struct kmem_cache *au_cachep[];

#define AuCacheArgs(type, sz)	(type), (sz), 0, SLAB_RECLAIM_ACCOUNT, NULL
#define AuCache(type) \
	kmem_cache_create(AuCacheArgs(#type, sizeof(struct type)))

/* ---------------------------------------------------------------------- */

#define AuCacheFuncs(name, index) \
static inline void *au_cache_alloc_##name(void) \
{ return kmem_cache_alloc(au_cachep[index], GFP_NOFS); } \
static inline void au_cache_free_##name(void *p) \
{ kmem_cache_free(au_cachep[index], p); }

AuCacheFuncs(dinfo, AuCache_DINFO);
AuCacheFuncs(icntnr, AuCache_ICNTNR);
AuCacheFuncs(finfo, AuCache_FINFO);
AuCacheFuncs(vdir, AuCache_VDIR);
AuCacheFuncs(dehstr, AuCache_DEHSTR);

#endif /* __KERNEL__ */
#endif /* __AUFS_MODULE_H__ */
