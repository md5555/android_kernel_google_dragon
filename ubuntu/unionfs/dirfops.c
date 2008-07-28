/*
 * Copyright (c) 2003-2006 Erez Zadok
 * Copyright (c) 2003-2006 Charles P. Wright
 * Copyright (c) 2005-2006 Josef 'Jeff' Sipek
 * Copyright (c) 2005-2006 Junjiro Okajima
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
 *  $Id: dirfops.c,v 1.25 2006/08/05 01:28:46 jro Exp $
 */

#include "unionfs.h"

/* Make sure our rdstate is playing by the rules. */
static void verify_rdstate_offset(struct unionfs_dir_state *rdstate)
{
	BUG_ON(rdstate->uds_offset >= DIREOF);
	BUG_ON(rdstate->uds_cookie >= MAXRDCOOKIE);
}

struct unionfs_getdents_callback {
	struct unionfs_dir_state *rdstate;
	void *dirent;
	int entries_written;
	int filldir_called;
	int filldir_error;
	filldir_t filldir;
	struct super_block *sb;
};

/* copied from generic filldir in fs/readir.c */
static int unionfs_filldir(void *dirent, const char *name, int namelen,
			   loff_t offset, u64 ino, unsigned int d_type)
{
	struct unionfs_getdents_callback *buf =
	    (struct unionfs_getdents_callback *)dirent;
	struct filldir_node *found = NULL;
	int err = 0;
	int is_wh_entry = 0;

	dprint(PRINT_DEBUG, "unionfs_filldir name=%*s\n", namelen, name);

	buf->filldir_called++;

	if ((namelen > WHLEN) && !strncmp(name, WHPFX, WHLEN)) {
		name += WHLEN;
		namelen -= WHLEN;
		is_wh_entry = 1;
	}

	found = find_filldir_node(buf->rdstate, name, namelen);

	if (found)
		goto out;

	/* if 'name' isn't a whiteout filldir it. */
	if (!is_wh_entry) {
		off_t pos = rdstate2offset(buf->rdstate);
		ino_t unionfs_ino = ino;
#ifdef UNIONFS_IMAP
		if (stopd(buf->sb)->usi_persistent)
			err = read_uin(buf->sb, buf->rdstate->uds_bindex,
				       ino, O_CREAT, &unionfs_ino);
#endif
		if (!err) {
			err = buf->filldir(buf->dirent, name, namelen, pos,
					   unionfs_ino, d_type);
			buf->rdstate->uds_offset++;
			verify_rdstate_offset(buf->rdstate);
		}
	}
	/* If we did fill it, stuff it in our hash, otherwise return an error */
	if (err) {
		buf->filldir_error = err;
		goto out;
	}
	buf->entries_written++;
	if ((err = add_filldir_node(buf->rdstate, name, namelen,
				    buf->rdstate->uds_bindex, is_wh_entry)))
		buf->filldir_error = err;

      out:
	return err;
}

static int unionfs_readdir(struct file *file, void *dirent, filldir_t filldir)
{
	int err = 0;
	struct file *hidden_file = NULL;
	struct inode *inode = NULL;
	struct unionfs_getdents_callback buf;
	struct unionfs_dir_state *uds;
	int bend;
	loff_t offset;

	print_entry("file = %p, pos = %llx", file, file->f_pos);

	print_file("In unionfs_readdir()", file);

	if ((err = unionfs_file_revalidate(file, 0)))
		goto out;

	inode = file->f_dentry->d_inode;
	checkinode(inode, "unionfs_readdir");

	uds = ftopd(file)->rdstate;
	if (!uds) {
		if (file->f_pos == DIREOF) {
			goto out;
		} else if (file->f_pos > 0) {
			uds = find_rdstate(inode, file->f_pos);
			if (!uds) {
				err = -ESTALE;
				goto out;
			}
			ftopd(file)->rdstate = uds;
		} else {
			init_rdstate(file);
			uds = ftopd(file)->rdstate;
		}
	}
	bend = fbend(file);

	while (uds->uds_bindex <= bend) {
		hidden_file = ftohf_index(file, uds->uds_bindex);
		if (!hidden_file) {
			dprint(PRINT_DEBUG,
				    "Incremented bindex to %d of %d,"
				    " because hidden file is NULL.\n",
				    uds->uds_bindex, bend);
			uds->uds_bindex++;
			uds->uds_dirpos = 0;
			continue;
		}

		/* prepare callback buffer */
		buf.filldir_called = 0;
		buf.filldir_error = 0;
		buf.entries_written = 0;
		buf.dirent = dirent;
		buf.filldir = filldir;
		buf.rdstate = uds;
		buf.sb = inode->i_sb;

		/* Read starting from where we last left off. */
		offset = vfs_llseek(hidden_file, uds->uds_dirpos, 0);
		if (offset < 0) {
			err = offset;
			goto out;
		}
		dprint(PRINT_DEBUG, "calling readdir for %d.%lld (offset = %lld)\n",
			    uds->uds_bindex, uds->uds_dirpos, offset);
		err = vfs_readdir(hidden_file, unionfs_filldir, (void *)&buf);
		dprint(PRINT_DEBUG,
			    "readdir on %d.%lld = %d (entries written %d, filldir called %d)\n",
			    uds->uds_bindex, (long long)uds->uds_dirpos, err,
			    buf.entries_written, buf.filldir_called);
		/* Save the position for when we continue. */

		offset = vfs_llseek(hidden_file, 0, 1);
		if (offset < 0) {
			err = offset;
			goto out;
		}
		uds->uds_dirpos = offset;

		/* Copy the atime. */
		fist_copy_attr_atime(inode, hidden_file->f_dentry->d_inode);

		if (err < 0) {
			goto out;
		}

		if (buf.filldir_error) {
			break;
		}

		if (!buf.entries_written) {
			uds->uds_bindex++;
			uds->uds_dirpos = 0;
		}
	}

	if (!buf.filldir_error && uds->uds_bindex >= bend) {
		dprint(PRINT_DEBUG,
			    "Discarding rdstate because readdir is over (hashsize = %d)\n",
			    uds->uds_hashentries);
		/* Save the number of hash entries for next time. */
		itopd(inode)->uii_hashsize = uds->uds_hashentries;
		free_rdstate(uds);
		ftopd(file)->rdstate = NULL;
		file->f_pos = DIREOF;
	} else {
		file->f_pos = rdstate2offset(uds);
		dprint(PRINT_DEBUG, "rdstate now has a cookie of %u (err = %d)\n",
			    uds->uds_cookie, err);
	}

      out:
	checkinode(inode, "post unionfs_readdir");
	print_exit_status(err);
	return err;
}

/* This is not meant to be a generic repositioning function.  If you do
 * things that aren't supported, then we return EINVAL.
 *
 * What is allowed:
 *  (1) seeking to the same position that you are currently at
 *	This really has no effect, but returns where you are.
 *  (2) seeking to the end of the file, if you've read everything
 *	This really has no effect, but returns where you are.
 *  (3) seeking to the beginning of the file
 *	This throws out all state, and lets you begin again.
 */
static loff_t unionfs_dir_llseek(struct file *file, loff_t offset, int origin)
{
	struct unionfs_dir_state *rdstate;
	loff_t err;

	print_entry(" file=%p, offset=0x%llx, origin = %d", file, offset,
		    origin);

	if ((err = unionfs_file_revalidate(file, 0)))
		goto out;

	rdstate = ftopd(file)->rdstate;

	/* We let users seek to their current position, but not anywhere else. */
	if (!offset) {
		switch (origin) {
		case SEEK_SET:
			if (rdstate) {
				free_rdstate(rdstate);
				ftopd(file)->rdstate = NULL;
			}
			init_rdstate(file);
			err = 0;
			break;
		case SEEK_CUR:
			if (file->f_pos) {
				if (file->f_pos == DIREOF)
					err = DIREOF;
				else
					BUG_ON(file->f_pos !=
					       rdstate2offset(rdstate));
				err = file->f_pos;
			} else {
				err = 0;
			}
			break;
		case SEEK_END:
			/* Unsupported, because we would break everything.  */
			err = -EINVAL;
			break;
		}
	} else {
		switch (origin) {
		case SEEK_SET:
			if (rdstate) {
				if (offset == rdstate2offset(rdstate)) {
					err = offset;
				} else if (file->f_pos == DIREOF) {
					err = DIREOF;
				} else {
					err = -EINVAL;
				}
			} else {
				if ((rdstate =
				     find_rdstate(file->f_dentry->d_inode,
						  offset))) {
					ftopd(file)->rdstate = rdstate;
					err = rdstate->uds_offset;
				} else {
					err = -EINVAL;
				}
			}
			break;
		case SEEK_CUR:
		case SEEK_END:
			/* Unsupported, because we would break everything.  */
			err = -EINVAL;
			break;
		}
	}

      out:
	print_exit_status((int)err);
	return err;
}

/* Trimmed directory options, we shouldn't pass everything down since
 * we don't want to operate on partial directories.
 */
struct file_operations unionfs_dir_fops = {
	.llseek = unionfs_dir_llseek,
	.read = generic_read_dir,
	.readdir = unionfs_readdir,
	.unlocked_ioctl = unionfs_ioctl,
	.open = unionfs_open,
	.release = unionfs_file_release,
	.flush = unionfs_flush,
};

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
