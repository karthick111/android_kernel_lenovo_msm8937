/*
 * The sdcardfs
 *
 * Copyright (c) 2015 Lenovo Co. Ltd
 *   Authors: liaohs , jixj
 *
 * The sdcardfs v2.0 
 *   This file system replaces the sdcard daemon on Android 
 *   On version 2.0, some of the daemon functions have been ported  
 *   to support the multi-user concepts of Android 4.4
 *                      
 * This program has been developed as a stackable file system based on
 * the WrapFS which written by 
 *
 * Revision History
 * 2014.06.24 : Release Version 2.1.0
 *    - Add sdcardfs version
 *    - Add kernel log when put_super
 * 2014.07.21 : Release Version 2.1.1
 *    - Add sdcardfs_copy_inode_attr() to fix permission issue
 *    - Delete mmap_sem lock in sdcardfs_setattr() to avoid 
 * 2014.10.11 : Release Version 2.2.1
 *    - Add vm_operations_struct->page_mkwrite to fix kernel panic when using filemap in lenovo gallery apk
 *    - Add sdcardfs_file_llseek
 *    - Support sdcardfs xattr inode interface
 *    - update sdcardfs_fsync
 * 2015.01.06  Release Version 2.2.2
 *     - remove unnecessary do_munmap
 *     - call filemap_write_and_wait in sdcardfs_flush
 *     - add vm_operations_struct->close to fix photo lost captured by lenovo camera apk
 * 2015.01.20  Release Version 2.5.0
 *     - support case-insensitive search in EXT4 to fix photo lost which captured by lenovo camera/gallery apk
 *     - remove vm_operations_struct->close 
 *
 */

#define SDCARDFS_VERSION "2.5.0"
