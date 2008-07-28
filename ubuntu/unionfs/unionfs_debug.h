/*
 * Copyright (c) 2003-2006 Erez Zadok
 * Copyright (c) 2003-2006 Charles P. Wright
 * Copyright (c) 2005-2006 Josef Sipek
 * Copyright (c) 2005      Arun M. Krishnakumar
 * Copyright (c) 2005-2006 David P. Quigley
 * Copyright (c) 2003-2004 Mohammad Nayyer Zubair
 * Copyright (c) 2003      Puja Gupta
 * Copyright (c) 2003      Harikesavan Krishnan
 * Copyright (c) 2003-2006 Stony Brook University
 * Copyright (c) 2003-2006 The Research Foundation of State University of New York
 *
 * For specific licensing information, see the COPYING file distributed with
 * this package.
 *
 * This Copyright notice must be kept intact and distributed with all sources.
 */
/*
 *  $Id: unionfs_debug.h,v 1.3 2006/06/01 21:25:18 jsipek Exp $
 */

#ifndef __UNIONFS_H_
#error This file should only be included from unionfs.h!
#endif

#ifdef UNIONFS_DEBUG
#define DEFAULT_DEBUG_MASK	0
#else
#define DEFAULT_DEBUG_MASK	(~0)
#endif

/* debug print levels */
#define PRINT_NONE		0x0000
#define PRINT_MAIN_ENTRY	0x0001
#define PRINT_MAIN_EXIT		0x0002
#define PRINT_UTILITY_ENTRY	0x0004
#define PRINT_UTILITY_EXIT	0x0008
#define PRINT_MISC_ENTRY	0x0010
#define PRINT_MISC_EXIT		0x0020
#define PRINT_DATA_DENTRY	0x0040
#define PRINT_DATA_FILE		0x0080
#define PRINT_DATA_INODE	0x0100
#define PRINT_DATA_SB		0x0200
#define PRINT_DEBUG		0x0400
#define __PRINT_DEBUG_XATTR	0x0800
#define PRINT_DEBUG_XATTR	(PRINT_DEBUG | __PRINT_DEBUG_XATTR)
#define __PRINT_DEBUG_WHITEOUT	0x1000
#define PRINT_DEBUG_WHITEOUT	(PRINT_DEBUG | __PRINT_DEBUG_WHITEOUT)

#define PRINT_MAX		(0x2000 - 1)
#define PRINT_ALL		(~PRINT_NONE)

extern unsigned int get_debug_mask(void);
extern int set_debug_mask(int val);

/* print inode */
extern void unionfs_print_inode(const unsigned int req, const char *prefix, const struct inode *inode);

/* check inode */
extern void unionfs_checkinode(const unsigned int req, const struct inode *inode, const char *msg);

/* prunt file */
extern void unionfs_print_file(const unsigned int req, const char *prefix, const struct file *file);

/* print dentry */
extern void unionfs_print_dentry(const unsigned int req, const char *prefix, const struct dentry *dentry);

extern void unionfs_print_dentry_nocheck(const unsigned int req, const char *prefix, const struct dentry *dentry);

/* print superblock */
extern void unionfs_print_sb(const unsigned int req, const char *prefix, const struct super_block *sb);

/* print message */
extern int unionfs_print(const unsigned int req, const char *fmt, ...);

/* forced print-debugging functions */
#define force_print_dentry(prefix, ptr) \
		unionfs_print_dentry(PRINT_ALL, (prefix), (ptr))
#define force_print_dentry_nocheck(prefix, ptr) \
		unionfs_print_dentry_nocheck(PRINT_ALL, (prefix), (ptr))
#define force_print_file(prefix, ptr) \
		unionfs_print_file(PRINT_ALL, (prefix), (ptr))
#define force_print_inode(prefix, ptr) \
		unionfs_print_inode(PRINT_ALL, (prefix), (ptr))
#define force_print_sb(prefix, ptr) \
		unionfs_print_sb(PRINT_ALL, (prefix), (ptr))

#ifdef UNIONFS_DEBUG
/*
 * Full-fledged debugging enabled
 */

#define print_dentry(prefix, ptr) \
		unionfs_print_dentry(PRINT_DATA_DENTRY, (prefix), (ptr))
#define print_dentry_nocheck(prefix, ptr) \
		unionfs_print_dentry_nocheck(PRINT_DATA_DENTRY, (prefix), (ptr))
#define print_file(prefix, ptr) \
		unionfs_print_file(PRINT_DATA_FILE, (prefix), (ptr))
#define print_inode(prefix, ptr) \
		unionfs_print_inode(PRINT_DATA_INODE, (prefix), (ptr))
#define print_sb(prefix, ptr) \
		unionfs_print_sb(PRINT_DATA_SB, (prefix), (ptr))
#define dprint(req, fmt, args...) \
		unionfs_print(req, fmt, ## args)

#define checkinode(ptr, msg) \
		unionfs_checkinode(PRINT_DEBUG, (ptr), (msg))

#define __print_entryexit(req, ee, fmt, args...) \
		unionfs_print((req), \
			ee "  %s %s:%d" fmt "\n", \
			__FUNCTION__, \
			__FILE__, \
			__LINE__, \
			##args)

#define print_entry(fmt, args...) \
		__print_entryexit(PRINT_MAIN_ENTRY, \
			"IN: ", " " fmt, ##args)

#define print_entry_location() \
		__print_entryexit(PRINT_MAIN_ENTRY, \
			"IN: ", "")

#define print_exit_location() \
		__print_entryexit(PRINT_MAIN_EXIT, \
			"OUT:", "")

#define print_exit_status(status) \
		__print_entryexit(PRINT_MAIN_EXIT, \
			"OUT:", ", STATUS: %d", status)

static inline void __print_exit_pointer(unsigned int req, void *status)
{
	if (IS_ERR(status))
		__print_entryexit(req, "OUT:", ", STATUS: %ld",
				PTR_ERR(status));
	else
		__print_entryexit(req, "OUT:", ", STATUS: 0x%p",
				status);
}
#define print_exit_pointer(status) \
		__print_exit_pointer(PRINT_MAIN_EXIT, status)

#define print_util_entry(fmt, args...) \
		__print_entryexit(PRINT_UTILITY_ENTRY, \
			"IN: ", " " fmt, ##args)

#define print_util_entry_location() \
		__print_entryexit(PRINT_UTILITY_ENTRY, \
			"IN: ", "")

#define print_util_exit_location() \
		__print_entryexit(PRINT_UTILITY_EXIT, \
			"OUT:", "")

#define print_util_exit_status(status) \
		__print_entryexit(PRINT_UTILITY_EXIT, \
			"OUT:", ", STATUS: %d", status)

#define print_util_exit_pointer(status) \
		__print_exit_pointer(PRINT_UTILITY_EXIT, status)

#else /* UNIONFS_DEBUG */
/*
 * Full-fledged debugging disabled
 */

#define print_dentry(prefix, ptr)
#define print_dentry_nocheck(prefix, ptr)
#define print_file(prefix, ptr)
#define print_inode(prefix, ptr)
#define print_sb(prefix, ptr)
#define dprint(req, fmt, args...)

#define checkinode(ptr, msg)

#define print_entry(args...)
#define print_entry_location()
#define print_exit_location()
#define print_exit_status(status)
#define print_exit_pointer(status)
#define print_util_entry(args...)
#define print_util_entry_location()
#define print_util_exit_location()
#define print_util_exit_status(status)
#define print_util_exit_pointer(status)

#endif /* ! UNIONFS_DEBUG */


