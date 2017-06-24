/*
 * fs/sdcardfs/mmap.c
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

static int sdcardfs_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
	int err;
	struct file *file, *lower_file;
	const struct vm_operations_struct *lower_vm_ops;
	struct vm_area_struct lower_vma;
	const struct cred *saved_cred = NULL;
	/* save current_cred and override it */
	OVERRIDE_CRED(SDCARDFS_SB(vma->vm_file->f_path.dentry->d_sb), saved_cred);

	SDFS_DBG(" Just DBG: sdcardfs fault! \n");

	memcpy(&lower_vma, vma, sizeof(struct vm_area_struct));
	file = lower_vma.vm_file;
	lower_vm_ops = SDCARDFS_F(file)->lower_vm_ops;
	BUG_ON(!lower_vm_ops);

	lower_file = sdcardfs_lower_file(file);
	/*
	 * XXX: vm_ops->fault may be called in parallel.  Because we have to
	 * resort to temporarily changing the vma->vm_file to point to the
	 * lower file, a concurrent invocation of sdcardfs_fault could see a
	 * different value.  In this workaround, we keep a different copy of
	 * the vma structure in our stack, so we never expose a different
	 * value of the vma->vm_file called to us, even temporarily.  A
	 * better fix would be to change the calling semantics of ->fault to
	 * take an explicit file pointer.
	 */
	lower_vma.vm_file = lower_file;
	err = lower_vm_ops->fault(&lower_vma, vmf);
	REVERT_CRED(saved_cred);
	return err;
}
// 2014.10.11 merge vm_operations_struct->page_mkwrite from latest wrapfs to fix kernel panic,
// when using filemap in lenovo gallery apk
static int sdcardfs_page_mkwrite(struct vm_area_struct *vma,
			       struct vm_fault *vmf)
{
	int err = 0;
	struct file *file, *lower_file;
	const struct vm_operations_struct *lower_vm_ops;
	struct vm_area_struct lower_vma;
	const struct cred *saved_cred = NULL;
	/* save current_cred and override it */
	OVERRIDE_CRED(SDCARDFS_SB(vma->vm_file->f_path.dentry->d_sb), saved_cred);

	SDFS_DBG(" Just DBG: sdcardfs page_mkwrite ! \n");

	memcpy(&lower_vma, vma, sizeof(struct vm_area_struct));
	file = lower_vma.vm_file;
	lower_vm_ops = SDCARDFS_F(file)->lower_vm_ops;
	BUG_ON(!lower_vm_ops);
	if (!lower_vm_ops->page_mkwrite)
		goto out;

	lower_file = sdcardfs_lower_file(file);
	/*
	 * XXX: vm_ops->page_mkwrite may be called in parallel.
	 * Because we have to resort to temporarily changing the
	 * vma->vm_file to point to the lower file, a concurrent
	 * invocation of sdcardfs_page_mkwrite could see a different
	 * value.  In this workaround, we keep a different copy of the
	 * vma structure in our stack, so we never expose a different
	 * value of the vma->vm_file called to us, even temporarily.
	 * A better fix would be to change the calling semantics of
	 * ->page_mkwrite to take an explicit file pointer.
	 */
	lower_vma.vm_file = lower_file;
	err = lower_vm_ops->page_mkwrite(&lower_vma, vmf);
out:
	REVERT_CRED(saved_cred);
	return err;
}

static ssize_t sdcardfs_direct_IO(int rw, struct kiocb *iocb, struct iov_iter *iov,
			loff_t offset)
{
	/*
	 * This function should never be called directly.  We need it
	 * to exist, to get past a check in open_check_o_direct(),
	 * which is called from do_last().
	 */
	SDFS_ERR(" Just DBG: sdcardfs direct_IO operation is not supported ! \n");

	return -EINVAL;
}

const struct address_space_operations sdcardfs_aops = {
	.direct_IO = sdcardfs_direct_IO,
};

const struct vm_operations_struct sdcardfs_vm_ops = {
	.fault		= sdcardfs_fault,
	.page_mkwrite	= sdcardfs_page_mkwrite,
};
