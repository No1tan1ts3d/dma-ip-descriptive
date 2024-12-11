/* MD:
 * This file is part of the Xilinx DMA IP Core driver for Linux
 *
 * Copyright (c) 2017-2022, Xilinx, Inc. All rights reserved.
 * Copyright (c) 2022, Advanced Micro Devices, Inc. All rights reserved.
 *
 * This source code is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * The full GNU General Public License is included in this distribution in
 * the file called "COPYING".
 * 
 * This source code is modified to include debug prints and detailed comments.
 */

/* MD: Format string for debug messages includes function name */
#define pr_fmt(fmt)	KBUILD_MODNAME ":%s: " fmt, __func__

#include "qdma_debugfs.h"

/* MD:****************************************************************************/
/* MD:*
 * qdma_debugfs_init() - function to initialize debugfs
 *
 * This function creates a directory in the Linux debugfs filesystem for QDMA
 * debugging purposes. The directory name depends on whether it's being built
 * for PF (Physical Function) or VF (Virtual Function).
 *
 * @param[in]: qdma_debugfs_root - pointer to store the created debugfs root
 *
 * @return	0: success
 * @return	-ENOENT: if directory creation fails
 *****************************************************************************/
int qdma_debugfs_init(struct dentry **qdma_debugfs_root)
{
    struct dentry *debugfs_root = NULL;

    /* MD: Print debug message indicating initialization start */
    pr_debug("Initializing QDMA debugfs\n");

#ifndef __QDMA_VF__
    /* MD: Create directory for Physical Function (PF) */
    debugfs_root = debugfs_create_dir("qdma-pf", NULL);
    if (!debugfs_root) {
        pr_err("Failed to create qdma-pf directory in debugfs\n");
        return -ENOENT;
    }
    pr_debug("Created qdma-pf directory in Linux debug file system\n");
#else
    /* MD: Create directory for Virtual Function (VF) */
    debugfs_root = debugfs_create_dir("qdma-vf", NULL);
    if (!debugfs_root) {
        pr_err("Failed to create qdma-vf directory in debugfs\n");
        return -ENOENT;
    }
    pr_debug("Created qdma-vf directory in Linux debug file system\n");
#endif

    /* MD: Store the created debugfs root directory */
    *qdma_debugfs_root = debugfs_root;

    pr_debug("QDMA debugfs initialization completed successfully\n");
    return 0;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_debugfs_exit() - function to cleanup debugfs
 *
 * This function removes the QDMA directory and all its contents from the
 * Linux debugfs filesystem. It performs a recursive removal to ensure
 * all subdirectories and files are properly cleaned up.
 *
 * @param[in]: qdma_debugfs_root - root directory to be removed
 *
 * @return	None
 *****************************************************************************/
void qdma_debugfs_exit(struct dentry *qdma_debugfs_root)
{
    /* MD: Print debug message before cleanup starts */
    pr_debug("Starting QDMA debugfs cleanup\n");

    /* MD: Remove the debugfs directory and all its contents recursively */
    debugfs_remove_recursive(qdma_debugfs_root);

#ifndef __QDMA_VF__
    pr_debug("Removed qdma_pf directory from Linux debug file system\n");
#else
    pr_debug("Removed qdma_vf directory from Linux debug file system\n");
#endif

    pr_debug("QDMA debugfs cleanup completed\n");
}
