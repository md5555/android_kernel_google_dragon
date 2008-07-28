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
 *  $Id: unionfs_imap.h,v 1.1 2006/05/30 21:38:45 jsipek Exp $
 */

#ifndef __UNIONFS_H_
#error This file should only be included from unionfs.h!
#endif

#ifdef UNIONFS_IMAP

/*UUID typedef needed later*/
typedef uint8_t uuid_t[16];

/*
* Defines,structs,and functions for persistent used by kernel and user
*/
#define MAX_MAPS 256
#define UUID_LEN 16
#define FORWARDMAP_MAGIC 0x4b1cb38f
#define REVERSEMAP_MAGIC 0Xfcafad71
#define FORWARDMAP_VERSION 0x02
#define REVERSEMAP_VERSION 0x01
#define FIRST_VALID_INODE 3
struct fmaphdr {
	uint32_t magic;
	uint32_t version;
	uint8_t usedbranches;
	uint8_t uuid[UUID_LEN];
};

struct rmaphdr {
	uint32_t magic;
	uint32_t version;
	uint8_t fwduuid[UUID_LEN];
	uint8_t revuuid[UUID_LEN];
	fsid_t fsid;
};
struct bmapent {
	fsid_t fsid;
	uint8_t uuid[UUID_LEN];
};
struct fmapent {
	uint8_t fsnum;
	uint64_t inode;
};

/* Persistant Inode functions */
extern int read_uin(struct super_block *sb, uint8_t branchnum,
		    ino_t inode_number, int flag, ino_t * uino);
extern int write_uin(struct super_block *sb, ino_t ino, int bindex,
		     ino_t hidden_ino);
extern int get_lin(struct super_block *sb, ino_t inode_number,
		   struct fmapent *entry);
extern int parse_imap_option(struct super_block *sb,
			     struct unionfs_dentry_info *hidden_root_info,
			     char *options);
extern void cleanup_imap_data(struct super_block *sb);

#endif				/*#ifdef UNIONFS_IMAP */

/*
 *
 * vim:shiftwidth=8
 * vim:tabstop=8
 *
 * For Emacs:
 * Local variables:
 * c-basic-offset: 8
 * c-comment-only-line-offset: 0
 * c-offsets-alist: ((statement-block-intro . +) (knr-argdecl-intro . 0)
 *              (substatement-open . 0) (label . 0) (statement-cont . +))
 * indent-tabs-mode: t
 * tab-width: 8
 * End:
 */
