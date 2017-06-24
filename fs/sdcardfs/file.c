/*
 * fs/sdcardfs/file.c
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
#ifdef CONFIG_SDCARD_FS_FADV_NOACTIVE
#include <linux/backing-dev.h>
#endif

static ssize_t sdcardfs_read(struct file *file, char __user *buf,
			   size_t count, loff_t *ppos)
{
	int err;
	struct file *lower_file;
	struct dentry *dentry = file->f_path.dentry;
	const struct cred *saved_cred;
	OVERRIDE_CRED(SDCARDFS_SB(dentry->d_sb), saved_cred);

	SDFS_DBG("d_name='%s'\n",file->f_path.dentry->d_name.name);

#ifdef CONFIG_SDCARD_FS_FADV_NOACTIVE
	struct backing_dev_info *bdi;
#endif

	lower_file = sdcardfs_lower_file(file);

#ifdef CONFIG_SDCARD_FS_FADV_NOACTIVE
	if (file->f_mode & FMODE_NOACTIVE) {
		if (!(lower_file->f_mode & FMODE_NOACTIVE)) {
			bdi = lower_file->f_mapping->backing_dev_info;
			lower_file->f_ra.ra_pages = bdi->ra_pages * 2;
			spin_lock(&lower_file->f_lock);
			lower_file->f_mode |= FMODE_NOACTIVE;
			spin_unlock(&lower_file->f_lock);
		}
	}
#endif

	err = vfs_read(lower_file, buf, count, ppos);
	/* update our inode atime upon a successful lower read */
	if (err >= 0)
		fsstack_copy_attr_atime(dentry->d_inode,
					file_inode(lower_file));

	REVERT_CRED(saved_cred);
	return err;
}

static ssize_t sdcardfs_write(struct file *file, const char __user *buf,
			    size_t count, loff_t *ppos)
{
	int err = 0;
	struct file *lower_file;
	struct dentry *dentry = file->f_path.dentry;
	const struct cred *saved_cred;
	OVERRIDE_CRED(SDCARDFS_SB(dentry->d_sb), saved_cred);

	SDFS_DBG("d_name='%s'\n",file->f_path.dentry->d_name.name);

	/* check disk space */
	if (!check_min_free_space(dentry, count, 0)) {
		printk(KERN_INFO "No minimum free space.\n");
		//return -ENOSPC;
		err = -ENOSPC;
		goto out_revert_cred;
	}

	lower_file = sdcardfs_lower_file(file);
	err = vfs_write(lower_file, buf, count, ppos);
	/* update our inode times+sizes upon a successful lower write */
	if (err >= 0) {
		fsstack_copy_inode_size(dentry->d_inode,
					file_inode(lower_file));
		fsstack_copy_attr_times(dentry->d_inode,
					file_inode(lower_file));
	}

out_revert_cred:
	REVERT_CRED(saved_cred);
	return err;
}

//static int sdcardfs_readdir(struct file *file, void *dirent, filldir_t filldir)
static int sdcardfs_readdir(struct file *file, struct dir_context *ctx)
{
	int err = 0;
	struct file *lower_file = NULL;
	struct dentry *dentry = file->f_path.dentry;

	const struct cred *saved_cred;
	OVERRIDE_CRED(SDCARDFS_SB(dentry->d_sb), saved_cred);

	SDFS_DBG("d_name='%s'\n",file->f_path.dentry->d_name.name);

	lower_file = sdcardfs_lower_file(file);

	lower_file->f_pos = file->f_pos;
	err = iterate_dir(lower_file, ctx);
	file->f_pos = lower_file->f_pos;
	if (err >= 0)		/* copy the atime */
		fsstack_copy_attr_atime(dentry->d_inode,
					file_inode(lower_file));

	REVERT_CRED(saved_cred);
	return err;
}

static long sdcardfs_unlocked_ioctl(struct file *file, unsigned int cmd,
				  unsigned long arg)
{
	long err = -ENOTTY;
	struct file *lower_file;
	const struct cred *saved_cred;
	struct sdcardfs_sb_info *sbi = SDCARDFS_SB(file->f_path.dentry->d_sb);
	OVERRIDE_CRED(SDCARDFS_SB(file->f_path.dentry->d_sb), saved_cred);

	if (cmd == SDCARDFS_IOC_DIS_ACCESS) {
		if (!capable(CAP_SYS_ADMIN)) {
			err = -EPERM;
			goto out;
		}
		sbi->flag |= SDCARDFS_MOUNT_ACCESS_DISABLE;
		err = 0;
		SDFS_ERR( "unlocked ioctl disable access\n");
		goto out;
	}

	lower_file = sdcardfs_lower_file(file);

	/* XXX: use vfs_ioctl if/when VFS exports it */
	if (!lower_file || !lower_file->f_op)
		goto out;
	if (lower_file->f_op->unlocked_ioctl)
		err = lower_file->f_op->unlocked_ioctl(lower_file, cmd, arg);
     //merge from latest wrapfs
	/* some ioctls can change inode attributes (EXT2_IOC_SETFLAGS) */
	if (!err)
		sdcardfs_copy_inode_attr(file_inode(file),
				      file_inode(lower_file));
out:
	REVERT_CRED(saved_cred);
	return err;
}

#ifdef CONFIG_COMPAT
static long sdcardfs_compat_ioctl(struct file *file, unsigned int cmd,
				unsigned long arg)
{
	long err = -ENOTTY;
	struct file *lower_file;
	const struct cred *saved_cred;
	OVERRIDE_CRED(SDCARDFS_SB(file->f_path.dentry->d_sb), saved_cred);

	lower_file = sdcardfs_lower_file(file);

	/* XXX: use vfs_ioctl if/when VFS exports it */
	if (!lower_file || !lower_file->f_op)
		goto out;
	if (lower_file->f_op->compat_ioctl)
		err = lower_file->f_op->compat_ioctl(lower_file, cmd, arg);

out:
	REVERT_CRED(saved_cred);
	return err;
}
#endif

static int sdcardfs_mmap(struct file *file, struct vm_area_struct *vma)
{
	int err = 0;
	bool willwrite;
	struct file *lower_file;
	const struct vm_operations_struct *saved_vm_ops = NULL;
	const struct cred *saved_cred;
	OVERRIDE_CRED(SDCARDFS_SB(file->f_path.dentry->d_sb), saved_cred);

	/* this might be deferred to mmap's writepage */
	willwrite = ((vma->vm_flags | VM_SHARED | VM_WRITE) == vma->vm_flags);

	SDFS_DBG("vmflags =0x%lx, willwrite=%d \n", vma->vm_flags, willwrite);
	/*
	 * File systems which do not implement ->writepage may use
	 * generic_file_readonly_mmap as their ->mmap op.  If you call
	 * generic_file_readonly_mmap with VM_WRITE, you'd get an -EINVAL.
	 * But we cannot call the lower ->mmap op, so we can't tell that
	 * writeable mappings won't work.  Therefore, our only choice is to
	 * check if the lower file system supports the ->writepage, and if
	 * not, return EINVAL (the same error that
	 * generic_file_readonly_mmap returns in that case).
	 */
	lower_file = sdcardfs_lower_file(file);
	if (willwrite && !lower_file->f_mapping->a_ops->writepage) {
		err = -EINVAL;
		printk(KERN_ERR "sdcardfs: lower file system does not "
		       "support writeable mmap\n");
		goto out;
	}

	/* DTS2014061002949 for sdcardfs file mmap problem begin */
	/*
	 * find and save lower vm_ops.
	 *
	 * XXX: the VFS should have a cleaner way of finding the lower vm_ops
	 */
	if (!SDCARDFS_F(file)->lower_vm_ops) {
		err = lower_file->f_op->mmap(lower_file, vma);
		if (err) {
			printk(KERN_ERR "sdcardfs: lower mmap failed %d\n", err);
			goto out;
		}
		saved_vm_ops = vma->vm_ops; /* save: came from lower ->mmap */
/* ######## ATTENTION ############
 	2014.01.04   remove unnecessary do_munmap in latest wrapfs
*/
//		err = do_munmap(current->mm, vma->vm_start,
//				vma->vm_end - vma->vm_start);
//		if (err) {
//			printk(KERN_ERR "sdcardfs: do_munmap failed %d\n", err);
//			goto out;
//		}
	}

	/*
	 * Next 3 lines are all I need from generic_file_mmap.  I definitely
	 * don't want its test for ->readpage which returns -ENOEXEC.
	 */
	file_accessed(file);
	vma->vm_ops = &sdcardfs_vm_ops;
//	fput(file);
//	get_file(lower_file);
//	vma->vm_file = lower_file;
	/* DTS2014061002949 for sdcardfs file mmap problem end */

	file->f_mapping->a_ops = &sdcardfs_aops; /* set our aops */
	if (!SDCARDFS_F(file)->lower_vm_ops) /* save for our ->fault */
		SDCARDFS_F(file)->lower_vm_ops = saved_vm_ops;

out:
	REVERT_CRED(saved_cred);
	return err;
}

static int sdcardfs_open(struct inode *inode, struct file *file)
{
	int err = 0;
	struct file *lower_file = NULL;
	struct path lower_path;
	struct dentry *dentry = file->f_path.dentry;
	struct dentry *parent = dget_parent(dentry);
	struct sdcardfs_sb_info *sbi = SDCARDFS_SB(dentry->d_sb); 
	const struct cred *saved_cred = NULL;
	int has_rw;

	SDFS_DBG("d_name='%s'\n",file->f_path.dentry->d_name.name);

	/* don't open unhashed/deleted files */
	if (d_unhashed(dentry)) {
		err = -ENOENT;
		SDFS_ERR("d_unhashed() fail \n");	
		goto out_err;
	}
	
	has_rw = get_caller_has_rw_locked(sbi->pkgl_id, sbi->options.derive);

	if(!check_caller_access_to_name(parent->d_inode, dentry->d_name.name, 
				sbi->options.derive, 
				open_flags_to_access_mode(file->f_flags), has_rw)) {
		printk(KERN_INFO "%s: need to check the caller's gid in packages.list\n" 
                         "	dentry: %s, task:%s\n",
						 __func__, dentry->d_name.name, current->comm);
		err = -EACCES;
		SDFS_ERR("check_caller_access_to_name() fail \n");	
		goto out_err;
	}

	/* save current_cred and override it */
	OVERRIDE_CRED(sbi, saved_cred);

	if (sbi->flag && SDCARDFS_MOUNT_ACCESS_DISABLE) {
		err = -ENOENT;
		SDFS_DBG("sdcardfs_open access disable\n");
		goto out_revert_cred;
	}

	file->private_data =
		kzalloc(sizeof(struct sdcardfs_file_info), GFP_KERNEL);
	if (!SDCARDFS_F(file)) {
		err = -ENOMEM;
		SDFS_ERR("kzalloc() fail \n");
		goto out_revert_cred;
	}

	/* open lower object and link sdcardfs's file struct to lower's */
	sdcardfs_get_lower_path(file->f_path.dentry, &lower_path);
	lower_file = dentry_open(&lower_path, file->f_flags, current_cred());
	//path_put(&lower_path);  //move to out_err:
	/*lenovo-sw jixj 2015.1.31 add begin, it cause unlink file but space don't free */
	path_put(&lower_path);
	/*lenovo-sw jixj 2015.1.31 add end*/
	if (IS_ERR(lower_file)) {
		err = PTR_ERR(lower_file);
		lower_file = sdcardfs_lower_file(file);
		if (lower_file) {
			sdcardfs_set_lower_file(file, NULL);
			fput(lower_file); /* fput calls dput for lower_dentry */
		}
	} else {
		sdcardfs_set_lower_file(file, lower_file);
	}

	if (err)
		kfree(SDCARDFS_F(file));
	else {
		//2015.01.04  merge from Nxx50
		mutex_lock(&inode->i_mutex);
		sdcardfs_copy_inode_attr(inode, sdcardfs_lower_inode(inode));
		fix_derived_permission(inode);
		mutex_unlock(&inode->i_mutex);
	}

out_revert_cred:
	REVERT_CRED(saved_cred);
out_err:
	dput(parent);
	return err;
}

static int sdcardfs_flush(struct file *file, fl_owner_t id)
{
	int err = 0;
	struct file *lower_file = NULL;
	const struct cred *saved_cred;
	OVERRIDE_CRED(SDCARDFS_SB(file->f_path.dentry->d_sb), saved_cred);

	SDFS_DBG("d_name='%s'\n",file->f_path.dentry->d_name.name);

	lower_file = sdcardfs_lower_file(file);
	if (lower_file && lower_file->f_op && lower_file->f_op->flush) {
	//  2015.01.04 . merge from latest wrapfs . it is important for filemap
		filemap_write_and_wait(file->f_mapping);
	// end for filemap 	
		err = lower_file->f_op->flush(lower_file, id);
	}
	REVERT_CRED(saved_cred);

	return err;
}

/* release all lower object references & free the file info structure */
static int sdcardfs_file_release(struct inode *inode, struct file *file)
{
	struct file *lower_file;

	lower_file = sdcardfs_lower_file(file);
	if (lower_file) {
		sdcardfs_set_lower_file(file, NULL);
		fput(lower_file);
	}

	kfree(SDCARDFS_F(file));
	return 0;
}

static int sdcardfs_fsync(struct file *file, loff_t start, loff_t end,
			int datasync)
{
	int err;
	struct file *lower_file;
	struct path lower_path;
	struct dentry *dentry = file->f_path.dentry;
	const struct cred *saved_cred;
	OVERRIDE_CRED(SDCARDFS_SB(dentry->d_sb), saved_cred);

	SDFS_DBG("d_name='%s'\n",file->f_path.dentry->d_name.name);
	//2015.01.04  merge from latest wrapfs	
	//err = generic_file_fsync(file, start, end, datasync);
	// in kernel v3.16 generic_file_fsync will will triger a kernel panic
	err = __generic_file_fsync(file, start, end, datasync);
	if (err)
		goto out;
	lower_file = sdcardfs_lower_file(file);
	sdcardfs_get_lower_path(dentry, &lower_path);
	err = vfs_fsync_range(lower_file, start, end, datasync);
	sdcardfs_put_lower_path(dentry, &lower_path);
out:
	REVERT_CRED(saved_cred);
	return err;
}

static int sdcardfs_fasync(int fd, struct file *file, int flag)
{
	int err = 0;
	struct file *lower_file = NULL;
	const struct cred *saved_cred;
	OVERRIDE_CRED(SDCARDFS_SB(file->f_path.dentry->d_sb), saved_cred);

	SDFS_DBG("d_name='%s'\n",file->f_path.dentry->d_name.name);

	lower_file = sdcardfs_lower_file(file);
	if (lower_file->f_op && lower_file->f_op->fasync)
		err = lower_file->f_op->fasync(fd, lower_file, flag);
	REVERT_CRED(saved_cred);

	return err;
}

static ssize_t sdcardfs_aio_read(struct kiocb *iocb, const struct iovec *iov,
			       unsigned long nr_segs, loff_t pos)
{
	int err = -EINVAL;
	struct file *file = iocb->ki_filp; 
	struct file *lower_file;
	const struct cred *saved_cred;
	OVERRIDE_CRED(SDCARDFS_SB(file->f_path.dentry->d_sb), saved_cred);

	SDFS_DBG(" Just DBG: ! \n");

	lower_file = sdcardfs_lower_file(file);
	if (!lower_file->f_op->aio_read)
		goto out;
	/*
	 * It appears safe to rewrite this iocb, because in
	 * do_io_submit@fs/aio.c, iocb is a just copy from user.
	 */
	get_file(lower_file); /* prevent lower_file from being released */
	iocb->ki_filp = lower_file;
	err = lower_file->f_op->aio_read(iocb, iov, nr_segs, pos);
	iocb->ki_filp = file;
	fput(lower_file);
	/* update upper inode atime as needed */
	if (err >= 0 || err == -EIOCBQUEUED)
		fsstack_copy_attr_atime(file->f_path.dentry->d_inode,
					file_inode(lower_file));
out:
	REVERT_CRED(saved_cred);
	return err;
}

static ssize_t sdcardfs_aio_write(struct kiocb *iocb, const struct iovec *iov,
				unsigned long nr_segs, loff_t pos)
{
	int err = -EINVAL;
	struct file *file = iocb->ki_filp;
	struct file *lower_file;
	const struct cred *saved_cred;
	OVERRIDE_CRED(SDCARDFS_SB(file->f_path.dentry->d_sb), saved_cred);

	SDFS_DBG(" Just DBG: ! \n");

	lower_file = sdcardfs_lower_file(file);
	if (!lower_file->f_op->aio_write)
		goto out;
	/*
	 * It appears safe to rewrite this iocb, because in
	 * do_io_submit@fs/aio.c, iocb is a just copy from user.
	 */
	get_file(lower_file); /* prevent lower_file from being released */
	iocb->ki_filp = lower_file;
	err = lower_file->f_op->aio_write(iocb, iov, nr_segs, pos);
	iocb->ki_filp = file;
	fput(lower_file);
	/* update upper inode times/sizes as needed */
	if (err >= 0 || err == -EIOCBQUEUED) {
		fsstack_copy_inode_size(file->f_path.dentry->d_inode,
					file_inode(lower_file));
		fsstack_copy_attr_times(file->f_path.dentry->d_inode,
					file_inode(lower_file));
	}
out:
	REVERT_CRED(saved_cred);
	return err;
}

/*
 * Wrapfs cannot use generic_file_llseek as ->llseek, because it would
 * only set the offset of the upper file.  So we have to implement our
 * own method to set both the upper and lower file offsets
 * consistently.
 */
static loff_t sdcardfs_file_llseek(struct file *file, loff_t offset, int whence)
{
	int err;
	struct file *lower_file;
	const struct cred *saved_cred;
	OVERRIDE_CRED(SDCARDFS_SB(file->f_path.dentry->d_sb), saved_cred);

	SDFS_DBG("d_name='%s'\n",file->f_path.dentry->d_name.name);

	err = generic_file_llseek(file, offset, whence);
	if (err < 0)
		goto out;

	lower_file = sdcardfs_lower_file(file);
	err = generic_file_llseek(lower_file, offset, whence);

out:
	REVERT_CRED(saved_cred);
	return err;
}
#if 1  //current kernel version don't support these api
/*
 * Wrapfs read_iter, redirect modified iocb to lower read_iter
 */
ssize_t
sdcardfs_read_iter(struct kiocb *iocb, struct iov_iter *iter)
{
	int err;
	struct file *file = iocb->ki_filp, *lower_file;

	lower_file = sdcardfs_lower_file(file);
	if (!lower_file->f_op->read_iter) {
		err = -EINVAL;
		goto out;
	}

	get_file(lower_file); /* prevent lower_file from being released */
	iocb->ki_filp = lower_file;
	err = lower_file->f_op->read_iter(iocb, iter);
	iocb->ki_filp = file;
	fput(lower_file);
	/* update upper inode atime as needed */
	if (err >= 0 || err == -EIOCBQUEUED)
		fsstack_copy_attr_atime(file->f_path.dentry->d_inode,
					file_inode(lower_file));
out:
	return err;
}

/*
 * Wrapfs write_iter, redirect modified iocb to lower write_iter
 */
ssize_t
sdcardfs_write_iter(struct kiocb *iocb, struct iov_iter *iter)
{
	int err;
	struct file *file = iocb->ki_filp, *lower_file;

	lower_file = sdcardfs_lower_file(file);
	if (!lower_file->f_op->write_iter) {
		err = -EINVAL;
		goto out;
	}

	get_file(lower_file); /* prevent lower_file from being released */
	iocb->ki_filp = lower_file;
	err = lower_file->f_op->write_iter(iocb, iter);
	iocb->ki_filp = file;
	fput(lower_file);
	/* update upper inode times/sizes as needed */
	if (err >= 0 || err == -EIOCBQUEUED) {
		fsstack_copy_inode_size(file->f_path.dentry->d_inode,
					file_inode(lower_file));
		fsstack_copy_attr_times(file->f_path.dentry->d_inode,
					file_inode(lower_file));
	}
out:
	return err;
}
#endif
const struct file_operations sdcardfs_main_fops = {
	.llseek		= generic_file_llseek,
	.read		= sdcardfs_read,
	.write		= sdcardfs_write,
	.unlocked_ioctl	= sdcardfs_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= sdcardfs_compat_ioctl,
#endif
	.mmap		= sdcardfs_mmap,
	.open		= sdcardfs_open,
	.flush		= sdcardfs_flush,
	.release	= sdcardfs_file_release,
	.fsync		= sdcardfs_fsync,
	.fasync		= sdcardfs_fasync,
	.aio_read	= sdcardfs_aio_read,
	.aio_write	= sdcardfs_aio_write,
	.read_iter	= sdcardfs_read_iter,
	.write_iter	= sdcardfs_write_iter,
};

/* trimmed directory options */
const struct file_operations sdcardfs_dir_fops = {
	.llseek		= sdcardfs_file_llseek, //2015.01.04  merge from latest wrapfs	
	.read		= generic_read_dir,
	.iterate	= sdcardfs_readdir,
//	.readdir	= sdcardfs_readdir,
	.unlocked_ioctl	= sdcardfs_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= sdcardfs_compat_ioctl,
#endif
	.open		= sdcardfs_open,
	.release	= sdcardfs_file_release,
	.flush		= sdcardfs_flush,
	.fsync		= sdcardfs_fsync,
	.fasync		= sdcardfs_fasync,
};
