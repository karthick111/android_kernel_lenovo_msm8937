/*
 * fs/sdcardfs/super.c
 *
 * Copyright (c) 2015 Lenovo Co. Ltd
 *   Authors: liaohs , jixj

 *                      
 * This program has been developed as a stackable file system based on
 * the WrapFS which written by 
 *
 * Copyright (c) 1998-2014 Erez Zadok
 * Copyright (c) 2009	   Shrikar Archak
 * Copyright (c) 2003-2014 Stony Brook University
 * Copyright (c) 2003-2014 The Research Foundation of SUNY
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "sdcardfs.h"

/*
 * The inode cache is used with alloc_inode for both our inode info and the
 * vfs inode.
 */
static struct kmem_cache *sdcardfs_inode_cachep;
static LIST_HEAD(sdcardfs_list);
static DEFINE_SPINLOCK(sdcardfs_list_lock);

void sdcardfs_add_super(struct sdcardfs_sb_info *sbi, struct super_block *sb)
{
	sbi->s_sb = sb;
	INIT_LIST_HEAD(&sbi->s_list);

	spin_lock(&sdcardfs_list_lock);
	list_add_tail(&sbi->s_list, &sdcardfs_list);
	spin_unlock(&sdcardfs_list_lock);
}

static void sdcardfs_remove_super(struct sdcardfs_sb_info *sbi)
{
	spin_lock(&sdcardfs_list_lock);
	list_del(&sbi->s_list);
	spin_unlock(&sdcardfs_list_lock);
}

void sdcardfs_truncate_share(struct super_block *sb, struct inode *lower_inode, loff_t newsize)
{
	struct list_head *p;
	struct sdcardfs_sb_info *sbi;
	struct super_block *lower_sb = lower_inode->i_sb;
	struct inode *inode;

	spin_lock(&sdcardfs_list_lock);
	p = sdcardfs_list.next;
	while (p != &sdcardfs_list) {
		sbi = list_entry(p, struct sdcardfs_sb_info, s_list);
		if (sbi->s_sb == sb || sbi->lower_sb != lower_sb) {
			p = p->next;
			continue;
		}
		spin_unlock(&sdcardfs_list_lock);
		inode = ilookup(sbi->s_sb, lower_inode->i_ino);
		if (inode) {
			truncate_setsize(inode, newsize);
			iput(inode);
		}
		spin_lock(&sdcardfs_list_lock);
		p = p->next;
	}
	spin_unlock(&sdcardfs_list_lock);
}

void sdcardfs_drop_shared_icache(struct super_block *sb, struct inode *lower_inode)
{
	struct list_head *p;
	struct sdcardfs_sb_info *sbi;
	struct super_block *lower_sb = lower_inode->i_sb;

	spin_lock(&sdcardfs_list_lock);
	p = sdcardfs_list.next;
	while (p != &sdcardfs_list) {
		sbi = list_entry(p, struct sdcardfs_sb_info, s_list);
		if (sbi->s_sb == sb || sbi->lower_sb != lower_sb) {
			p = p->next;
			continue;
		}
		spin_unlock(&sdcardfs_list_lock);
		sdcardfs_drop_sb_icache(sbi->s_sb, lower_inode->i_ino);
		spin_lock(&sdcardfs_list_lock);
		p = p->next;
	}
	spin_unlock(&sdcardfs_list_lock);
}

/* final actions when unmounting a file system */
static void sdcardfs_put_super(struct super_block *sb)
{
	struct sdcardfs_sb_info *spd;
	struct super_block *s;

	spd = SDCARDFS_SB(sb);
	if (!spd)
		return;
//2015.01.04  merge from N9150	
	printk(KERN_ERR "sdcardfs: umounted dev_name %s\n", 
				spd->devpath ? spd->devpath : "");
	if(spd->devpath)
		kfree(spd->devpath);
//end for devpath
	if(spd->obbpath_s) {
		kfree(spd->obbpath_s);
		path_put(&spd->obbpath);
	}

	/* decrement lower super references */
	s = sdcardfs_lower_super(sb);
	sdcardfs_set_lower_super(sb, NULL);
	atomic_dec(&s->s_active);

	if(spd->pkgl_id)
		packagelist_destroy(spd->pkgl_id);

	sdcardfs_remove_super(spd);
	kfree(spd);
	sb->s_fs_info = NULL;
}

static int sdcardfs_statfs(struct dentry *dentry, struct kstatfs *buf)
{
	int err;
	struct path lower_path;
	//u32 min_blocks;
	struct sdcardfs_sb_info *sbi = SDCARDFS_SB(dentry->d_sb);

	if (sbi->flag && SDCARDFS_MOUNT_ACCESS_DISABLE) {
		SDFS_DBG("sdcardfs_statfs access disable\n");
		return -ENOENT;
	}

	sdcardfs_get_lower_path(dentry, &lower_path);
	err = vfs_statfs(&lower_path, buf);
	sdcardfs_put_lower_path(dentry, &lower_path);
#if 0
	if (sbi->options.reserved_mb) {
		/* Invalid statfs informations. */
		if (buf->f_bsize == 0) {
			printk(KERN_ERR "Returned block size is zero.\n");
			return -EINVAL;
		}
	
		min_blocks = ((sbi->options.reserved_mb * 1024 * 1024)/buf->f_bsize);
		buf->f_blocks -= min_blocks;
	
		if (buf->f_bavail > min_blocks)
			buf->f_bavail -= min_blocks;
		else
			buf->f_bavail = 0;
	
		/* Make reserved blocks invisiable to media storage */
		buf->f_bfree = buf->f_bavail;
	}
#endif
	/* set return buf to our f/s to avoid confusing user-level utils */
	buf->f_type = SDCARDFS_SUPER_MAGIC;

	//dump_stack();
	return err;
}

/*
 * @flags: numeric mount options
 * @options: mount options string
 */
static int sdcardfs_remount_fs(struct super_block *sb,
			int *flags, char *options)
{
	int err = 0;

	SDFS_DBG("options: %s \n",options);
	/*
	 * The VFS will take care of "ro" and "rw" flags among others.  We
	 * can safely accept a few flags (RDONLY, MANDLOCK), and honor
	 * SILENT, but anything else left over is an error.
	 */
	if ((*flags & ~(MS_RDONLY | MS_MANDLOCK | MS_SILENT)) != 0) {
		printk(KERN_ERR
		       "sdcardfs: remount flags 0x%x unsupported\n", *flags);
		err = -EINVAL;
	}

	return err;
}

/*
 * Called by iput() when the inode reference count reached zero
 * and the inode is not hashed anywhere.  Used to clear anything
 * that needs to be, before the inode is completely destroyed and put
 * on the inode free list.
 */
static void sdcardfs_evict_inode(struct inode *inode)
{
	struct inode *lower_inode;

	truncate_inode_pages(&inode->i_data, 0);
	clear_inode(inode);
	/*
	 * Decrement a reference to a lower_inode, which was incremented
	 * by our read_inode when it was created initially.
	 */
	lower_inode = sdcardfs_lower_inode(inode);
	sdcardfs_set_lower_inode(inode, NULL);
	iput(lower_inode);
}

static struct inode *sdcardfs_alloc_inode(struct super_block *sb)
{
	struct sdcardfs_inode_info *i;

	i = kmem_cache_alloc(sdcardfs_inode_cachep, GFP_KERNEL);
	if (!i)
		return NULL;

	/* memset everything up to the inode to 0 */
	memset(i, 0, offsetof(struct sdcardfs_inode_info, vfs_inode));

	i->vfs_inode.i_version = 1;
	return &i->vfs_inode;
}

static void sdcardfs_destroy_inode(struct inode *inode)
{
	kmem_cache_free(sdcardfs_inode_cachep, SDCARDFS_I(inode));
}

/* sdcardfs inode cache constructor */
static void init_once(void *obj)
{
	struct sdcardfs_inode_info *i = obj;

	inode_init_once(&i->vfs_inode);
}

int sdcardfs_init_inode_cache(void)
{
	int err = 0;

	sdcardfs_inode_cachep =
		kmem_cache_create("sdcardfs_inode_cache",
				  sizeof(struct sdcardfs_inode_info), 0,
				  SLAB_RECLAIM_ACCOUNT, init_once);
	if (!sdcardfs_inode_cachep)
		err = -ENOMEM;
	return err;
}

/* sdcardfs inode cache destructor */
void sdcardfs_destroy_inode_cache(void)
{
	if (sdcardfs_inode_cachep)
		kmem_cache_destroy(sdcardfs_inode_cachep);
}

/*
 * Used only in nfs, to kill any pending RPC tasks, so that subsequent
 * code can actually succeed and won't leave tasks that need handling.
 */
static void sdcardfs_umount_begin(struct super_block *sb)
{
	struct super_block *lower_sb;

	lower_sb = sdcardfs_lower_super(sb);
	if (lower_sb && lower_sb->s_op && lower_sb->s_op->umount_begin)
		lower_sb->s_op->umount_begin(lower_sb);
}

static int sdcardfs_show_options(struct seq_file *m, struct dentry *root)
{
	struct sdcardfs_sb_info *sbi = SDCARDFS_SB(root->d_sb);
	struct sdcardfs_mount_options *opts = &sbi->options;
	//dump_stack();
	//SDFS_DBG("show_options: %s \n",m->buf); 
	if (opts->fs_low_uid != 0)
		seq_printf(m, ",uid=%u", opts->fs_low_uid);
	if (opts->fs_low_gid != 0)
		seq_printf(m, ",gid=%u", opts->fs_low_gid);

	if(((opts->upper_perms.uid != 0) || (opts->upper_perms.gid != 0)) &&
	    (opts->upper_perms.fmask!= 0) && (opts->upper_perms.dmask!= 0)) {
		seq_printf(m, ",upper=%u:%u:%04o:%04o",
		    opts->upper_perms.uid,
		    opts->upper_perms.gid,
		    opts->upper_perms.fmask,
		    opts->upper_perms.dmask);
	}

	if (opts->derive == DERIVE_NONE)
		seq_printf(m, ",derive=none");
	else if (opts->derive == DERIVE_LEGACY)
		seq_printf(m, ",derive=legacy");
	else if (opts->derive == DERIVE_UNIFIED)
		seq_printf(m, ",derive=unified");
	else if (opts->derive == DERIVE_PUBLIC)
		seq_printf(m, ",derive=public");
	else if (opts->derive == DERIVE_MULTI)
		seq_printf(m, ",derive=multi");

	if (opts->reserved_mb != 0)
		seq_printf(m, ",reserved=%uMB", opts->reserved_mb);

	return 0;
};

const struct super_operations sdcardfs_sops = {
	.put_super	= sdcardfs_put_super,
	.statfs		= sdcardfs_statfs,
	.remount_fs	= sdcardfs_remount_fs,
	.evict_inode	= sdcardfs_evict_inode,
	.umount_begin	= sdcardfs_umount_begin,
	.show_options	= sdcardfs_show_options,
	.alloc_inode	= sdcardfs_alloc_inode,
	.destroy_inode	= sdcardfs_destroy_inode,
	.drop_inode	= generic_delete_inode,
};
