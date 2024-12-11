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

#define pr_fmt(fmt) KBUILD_MODNAME ":%s: " fmt, __func__

#include "qdma_debugfs_queue.h"
#include "libqdma_export.h"
#include "qdma_regs.h"
#include "qdma_context.h"
#include "qdma_descq.h"
#include <linux/uaccess.h>

#ifdef DEBUGFS
#define DEBUGFS_QUEUE_DESC_SZ	(100)
#define DEBUGFS_QUEUE_INFO_SZ	(256)
#define DEBUGFS_QUEUE_CTXT_SZ	(24 * 1024)

#define DEBUGFS_CTXT_ELEM(reg, pos, size)   \
	((reg >> pos) & ~(~0 << size))

#define DBGFS_QUEUE_INFO_SZ	256
#define DBGFS_ERR_BUFLEN    (64)

enum dbgfs_queue_info_type {
	DBGFS_QINFO_INFO = 0,
	DBGFS_QINFO_CNTXT = 1,
	DBGFS_QINFO_DESC = 2,
	DBGFS_QINFO_END,
};

enum dbgfs_cntxt_word {
	DBGFS_CNTXT_W0 = 0,
	DBGFS_CNTXT_W1 = 1,
	DBGFS_CNTXT_W2 = 2,
	DBGFS_CNTXT_W3 = 3,
	DBGFS_CNTXT_W4 = 4,
	DBGFS_CNTXT_W5 = 5,
	DBGFS_CNTXT_W6 = 6,
	DBGFS_CNTXT_W7 = 7,
};

enum dbgfs_cmpt_queue_info_type {
	DBGFS_CMPT_QINFO_INFO = 0,
	DBGFS_CMPT_QINFO_CNTXT = 1,
	DBGFS_CMPT_QINFO_DESC = 2,
	DBGFS_CMPT_QINFO_END,
};

struct dbgfs_q_dbgf {
	char name[DBGFS_DBG_FNAME_SZ];
	struct file_operations fops;
};

struct dbgfs_q_priv {
	unsigned long dev_hndl;
	unsigned long qhndl;
	char *data;
	int datalen;
};

enum dbgfs_desc_type {
	DBGFS_DESC_TYPE_C2H = 0,
	DBGFS_DESC_TYPE_H2C = DBGFS_DESC_TYPE_C2H,
	DBGFS_DESC_TYPE_CMPT = 1,
	DBGFS_DESC_TYPE_END = 2,
};

/* MD: * structure to hold file ops */
static struct dbgfs_q_dbgf qf[DBGFS_QINFO_END];

/* MD: * structure to hold file ops */
static struct dbgfs_q_dbgf cmpt_qf[DBGFS_CMPT_QINFO_END];
static int q_dbg_file_open(struct inode *inode, struct file *fp);
static int q_dbg_file_release(struct inode *inode, struct file *fp);
static int qdbg_info_read(unsigned long dev_hndl, unsigned long id, char **data,
		int *data_len, enum dbgfs_desc_type type);
static int qdbg_desc_read(unsigned long dev_hndl, unsigned long id, char **data,
		int *data_len, enum dbgfs_desc_type type);
static int qdbg_cntxt_read(unsigned long dev_hndl, unsigned long id,
		char **data, int *data_len, enum dbgfs_desc_type type);

/* MD: ****************************************************************************/
/* MD: *
 * cmpt_q_dbg_file_open() - static function that provides generic open
 *
 * @param[in]	inode:	pointer to file inode
 * @param[in]	fp:	pointer to file structure
 *
 * @return	0: success
 * @return	<0: error
 *****************************************************************************/
static int cmpt_q_dbg_file_open(struct inode *inode, struct file *fp)
{
	pr_debug("Opening file for completion queue debug\n");
	return q_dbg_file_open(inode, fp);
}

/* MD: ****************************************************************************/
/* MD: *
 * cmpt_q_dbg_file_release() - static function that provides generic release
 *
 * @param[in]	inode:	pointer to file inode
 * @param[in]	fp:	pointer to file structure
 *
 * @return	0: success
 * @return	<0: error
 *****************************************************************************/
static int cmpt_q_dbg_file_release(struct inode *inode, struct file *fp)
{
	pr_debug("Releasing file for completion queue debug\n");
	return q_dbg_file_release(inode, fp);
}

/* MD: ****************************************************************************/
/* MD: *
 * cmpt_q_dbg_file_read() - static function that provides common read
 *
 * @param[in]	fp:	pointer to file structure
 * @param[out]	user_buffer: pointer to user buffer
 * @param[in]	count: size of data to read
 * @param[in/out]	ppos: pointer to offset read
 * @param[in]	type: information type
 *
 * @return	>0: size read
 * @return	<0: error
 *****************************************************************************/
static ssize_t cmpt_q_dbg_file_read(struct file *fp,
		char __user *user_buffer,
		size_t count, loff_t *ppos,
		enum dbgfs_queue_info_type type)
{
	char *buf = NULL;
	int buf_len = 0;
	int len = 0;
	int rv = 0;
	struct dbgfs_q_priv *qpriv =
		(struct dbgfs_q_priv *)fp->private_data;

	// MD:  Check if data is already loaded
	if (qpriv->data == NULL && qpriv->datalen == 0) {
		pr_debug("Loading data for type: %d\n", type);
		// MD:  Load data based on the type requested
		if (type == DBGFS_QINFO_INFO) {
			rv = qdbg_info_read(qpriv->dev_hndl, qpriv->qhndl,
					&buf, &buf_len, DBGFS_DESC_TYPE_CMPT);
		} else if (type == DBGFS_QINFO_CNTXT) {
			rv = qdbg_cntxt_read(qpriv->dev_hndl, qpriv->qhndl,
					&buf, &buf_len, DBGFS_DESC_TYPE_CMPT);
		} else if (type == DBGFS_QINFO_DESC) {
			rv = qdbg_desc_read(qpriv->dev_hndl, qpriv->qhndl,
					&buf, &buf_len, DBGFS_DESC_TYPE_CMPT);
		}

		// MD:  Check for errors during data loading
		if (rv < 0) {
			pr_err("Failed to read data for type: %d\n", type);
			goto cmpt_q_dbg_file_read_exit;
		}

		// MD:  Store the loaded data in the private structure
		qpriv->datalen = rv;
		qpriv->data = buf;
	}

	// MD:  Assign the buffer and its length
	buf = qpriv->data;
	len = qpriv->datalen;

	// MD:  Ensure buffer is not NULL
	if (!buf) {
		pr_err("Buffer is NULL after data load\n");
		goto cmpt_q_dbg_file_read_exit;
	}

	// MD:  Check if the read position exceeds the data length
	if (*ppos >= len) {
		pr_debug("Position exceeds data length, nothing to read\n");
		rv = 0;
		goto cmpt_q_dbg_file_read_exit;
	}

	// MD:  Adjust count if it exceeds the remaining data length
	if (*ppos + count > len)
		count = len - *ppos;

	// MD:  Copy data to user buffer and check for errors
	if (copy_to_user(user_buffer, buf + *ppos, count)) {
		pr_err("Failed to copy data to user buffer\n");
		rv = -EFAULT;
		goto cmpt_q_dbg_file_read_exit;
	}

	// MD:  Update the position pointer
	*ppos += count;

	pr_debug("cmpt q read size %zu\n", count);

	return count;

cmpt_q_dbg_file_read_exit:
	// MD:  Free the buffer and reset data pointers
	kfree(buf);
	qpriv->data = NULL;
	qpriv->datalen = 0;
	return rv;
}

/* MD: ****************************************************************************/
/* MD: *
 * cmpt_q_info_open() - static function that executes info open
 *
 * @param[in]	inode:	pointer to file inode
 * @param[in]	fp:	pointer to file structure
 *
 * @return	0: success
 * @return	<0: error
 *****************************************************************************/
static int cmpt_q_info_open(struct inode *inode, struct file *fp)
{
    // MD:  Open the debug file for queue information
    printk(KERN_DEBUG "Opening queue info debug file\n");
    return cmpt_q_dbg_file_open(inode, fp);
}

/* MD: ****************************************************************************/
/* MD: *
 * cmpt_q_info_read() - static function that executes info read
 *
 * @param[in]	fp:	pointer to file structure
 * @param[out]	user_buffer: pointer to user buffer
 * @param[in]	count: size of data to read
 * @param[in/out]	ppos: pointer to offset read
 *
 * @return	>0: size read
 * @return	<0: error
 *****************************************************************************/
static ssize_t cmpt_q_info_read(struct file *fp, char __user *user_buffer,
			size_t count, loff_t *ppos)
{
    // MD:  Read the queue information from the debug file
    printk(KERN_DEBUG "Reading queue info debug file\n");
    return cmpt_q_dbg_file_read(fp, user_buffer, count, ppos,
			DBGFS_QINFO_INFO);
}

/* MD: ****************************************************************************/
/* MD: *
 * cmpt_q_cntxt_open() - static function that executes context open
 *
 * @param[in]	inode:	pointer to file inode
 * @param[in]	fp:	pointer to file structure
 *
 * @return	0: success
 * @return	<0: error
 *****************************************************************************/
static int cmpt_q_cntxt_open(struct inode *inode, struct file *fp)
{
    // MD:  Open the debug file for queue context
    printk(KERN_DEBUG "Opening queue context debug file\n");
    return cmpt_q_dbg_file_open(inode, fp);
}

/* MD: ****************************************************************************/
/* MD: *
 * cmpt_q_cntxt_read() - static function that executes context read
 *
 * @param[in]	fp:	pointer to file structure
 * @param[out]	user_buffer: pointer to user buffer
 * @param[in]	count: size of data to read
 * @param[in/out]	ppos: pointer to offset read
 *
 * @return	>0: size read
 * @return	<0: error
 *****************************************************************************/
static ssize_t cmpt_q_cntxt_read(struct file *fp, char __user *user_buffer,
			size_t count, loff_t *ppos)
{
    // MD:  Read the queue context from the debug file
    printk(KERN_DEBUG "Reading queue context debug file\n");
    return cmpt_q_dbg_file_read(fp, user_buffer, count, ppos,
			DBGFS_QINFO_CNTXT);
}

/* MD: ****************************************************************************/
/* MD: *
 * cmpt_q_desc_open() - static function that executes descriptor open
 *
 * @param[in]	inode:	pointer to file inode
 * @param[in]	fp:	pointer to file structure
 *
 * @return	0: success
 * @return	<0: error
 *****************************************************************************/
static int cmpt_q_desc_open(struct inode *inode, struct file *fp)
{
    // MD:  Open the debug file for queue descriptor
    printk(KERN_DEBUG "Opening queue descriptor debug file\n");
    return cmpt_q_dbg_file_open(inode, fp);
}

/* MD: ****************************************************************************/
/* MD: *
 * cmpt_q_desc_read() - static function that executes descriptor read
 *
 * @param[in]	fp:	pointer to file structure
 * @param[out]	user_buffer: pointer to user buffer
 * @param[in]	count: size of data to read
 * @param[in/out]	ppos: pointer to offset read
 *
 * @return	>0: size read
 * @return	<0: error
 *****************************************************************************/
static ssize_t cmpt_q_desc_read(struct file *fp, char __user *user_buffer,
			size_t count, loff_t *ppos)
{
    // MD:  Read the queue descriptor from the debug file
    printk(KERN_DEBUG "Reading queue descriptor debug file\n");
    return cmpt_q_dbg_file_read(fp, user_buffer, count, ppos,
			DBGFS_QINFO_DESC);
}

/* MD: ****************************************************************************/
/* MD: *
 * create_cmpt_q_dbg_files() - static function to create cmpt queue dbg files
 *
 * @param[in]	queue_root:	debugfs root for a queue
 *
 * @return	0: success
 * @return	<0: error
 *****************************************************************************/
static int create_cmpt_q_dbg_files(struct qdma_descq *descq,
		struct dentry *queue_root)
{
    struct dentry  *fp[DBGFS_QINFO_END] = { NULL };
    struct file_operations *fops = NULL;
    int i = 0;

    // MD:  Initialize the debug file structures
    memset(cmpt_qf, 0, sizeof(struct dbgfs_q_dbgf) * DBGFS_CMPT_QINFO_END);

    // MD:  Iterate over each debug file type and set up file operations
    for (i = 0; i < DBGFS_CMPT_QINFO_END; i++) {
        fops = &cmpt_qf[i].fops;
        fops->owner = THIS_MODULE;
        switch (i) {
        case DBGFS_CMPT_QINFO_INFO:
            // MD:  Set up file operations for queue info
            snprintf(cmpt_qf[i].name, DBGFS_DBG_FNAME_SZ, "%s", "info");
            fops->open = cmpt_q_info_open;
            fops->read = cmpt_q_info_read;
            fops->release = cmpt_q_dbg_file_release;
            printk(KERN_DEBUG "Setting up queue info debug file operations\n");
            break;
        case DBGFS_CMPT_QINFO_CNTXT:
            // MD:  Set up file operations for queue context
            snprintf(cmpt_qf[i].name, DBGFS_DBG_FNAME_SZ, "%s", "cntxt");
            fops->open = cmpt_q_cntxt_open;
            fops->read = cmpt_q_cntxt_read;
            fops->release = cmpt_q_dbg_file_release;
            printk(KERN_DEBUG "Setting up queue context debug file operations\n");
            break;
        case DBGFS_CMPT_QINFO_DESC:
            // MD:  Set up file operations for queue descriptor
            snprintf(cmpt_qf[i].name, DBGFS_DBG_FNAME_SZ, "%s", "desc");
            fops->open = cmpt_q_desc_open;
            fops->read = cmpt_q_desc_read;
            fops->release = cmpt_q_dbg_file_release;
            printk(KERN_DEBUG "Setting up queue descriptor debug file operations\n");
            break;
        }
    }

    // MD:  Create the debug files in the filesystem
    for (i = 0; i < DBGFS_CMPT_QINFO_END; i++) {
        fp[i] = debugfs_create_file(cmpt_qf[i].name, 0644, queue_root,
					descq, &cmpt_qf[i].fops);
        if (!fp[i]) {
            printk(KERN_ERR "Failed to create debug file: %s\n", cmpt_qf[i].name);
            return -1; // MD:  Return error if file creation fails
        }
    }
    printk(KERN_DEBUG "Successfully created all debug files\n");
    return 0;
}

/* MD: ****************************************************************************/
/* MD: *
 * q_dbg_file_open() - generic queue debug file open
 *
 * @param[in]	inode:	pointer to file inode
 * @param[in]	fp:	pointer to file structure
 *
 * @return	0: success
 * @return	<0: error
 *****************************************************************************/
static int q_dbg_file_open(struct inode *inode, struct file *fp)
{
    unsigned long dev_id = -1;
    int qidx = -1;
    struct dbgfs_q_priv *priv = NULL;
    int rv = 0;
    unsigned char dev_name[QDMA_DEV_NAME_SZ] = {0};
    unsigned char *lptr = NULL, *rptr = NULL;
    struct dentry *direction_dir = NULL;
    struct dentry *qid_dir = NULL;
    struct dentry *qroot_dir = NULL;
    struct dentry *dev_dir = NULL;
    struct qdma_descq *descq = NULL;

    if (!inode || !fp)
        return -EINVAL; // MD:  Return error if inode or file pointer is NULL

    descq = inode->i_private;
    if (!descq)
        return -EINVAL; // MD:  Return error if descriptor queue is NULL

    direction_dir = fp->f_path.dentry->d_parent;
    qid_dir = direction_dir->d_parent;
    qroot_dir = qid_dir->d_parent;
    dev_dir = qroot_dir->d_parent;

    // MD:  Convert queue ID string to integer
    rv = kstrtoint((const char *)qid_dir->d_iname, 0, &qidx);
    if (rv < 0) {
        rv = -ENODEV;
        return rv; // MD:  Return error if conversion fails
    }

    // MD:  Convert colon-separated b:d:f to hex
    rptr = dev_dir->d_iname;
    lptr = dev_name;
    while (*rptr) {
        if (*rptr == ':') {
            rptr++;
            continue;
        }
        *lptr++ = *rptr++;
    }

    // MD:  Convert device name string to hex integer
    rv = kstrtoul((const char *)dev_name, 16, &dev_id);
    if (rv < 0) {
        pr_err("%s, kstrtoint failed for %s, Error:%d\n", __func__,
            dev_dir->d_iname, rv);
        rv = -ENODEV;
        return rv; // MD:  Return error if conversion fails
    }

    // MD:  Allocate memory for private data structure
    priv = (struct dbgfs_q_priv *) kzalloc(sizeof(struct dbgfs_q_priv),
            GFP_KERNEL);
    if (!priv) {
        rv = -ENOMEM;
        return rv; // MD:  Return error if memory allocation fails
    }

    // MD:  Set device handle and queue handle in private data
    priv->dev_hndl = (unsigned long)descq->xdev;
    priv->qhndl = qdma_device_get_id_from_descq(descq->xdev, descq);

    // MD:  Assign private data to file pointer
    fp->private_data = priv;

    printk(KERN_DEBUG "Opened debug file for queue with ID: %d\n", qidx);
    return 0; // MD:  Return success
}

/* MD: ****************************************************************************/
/* MD: *
 * q_dbg_file_release() - function that provides generic release
 *
 * @param[in]	inode:	pointer to file inode
 * @param[in]	fp:	pointer to file structure
 *
 * @return	0: success
 * @return	<0: error
 *****************************************************************************/
static int q_dbg_file_release(struct inode *inode, struct file *fp)
{
    // MD:  Free the allocated private data
    kfree(fp->private_data);
    fp->private_data = NULL;

    printk(KERN_DEBUG "Released debug file resources\n");
    return 0; // MD:  Return success
}

/* MD: ****************************************************************************/
/* MD: *
 * qdbg_cntxt_read() - reads queue context for a queue
 *
 * @param[in]	dev_hndl:	xdev device handle
 * @param[in]	id: queue handle
 * @param[out]	buf: buffer to collect the context info
 * @param[in]	buflen: buffer len
 * @param[in]	type: context type
 *
 * @return	>0: size read
 * @return	<0: error
 *****************************************************************************/
static int qdbg_cntxt_read(unsigned long dev_hndl, unsigned long id,
		char **data, int *data_len, enum dbgfs_desc_type type)
{
    int rv = 0;
    char *buf = NULL;
    int buflen = DEBUGFS_QUEUE_CTXT_SZ;
    struct qdma_descq *descq = NULL;
    struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;

    if (!xdev)
        return -EINVAL; // MD:  Return error if device handle is invalid

    if (xdev_check_hndl(__func__, xdev->conf.pdev, dev_hndl) < 0)
        return -EINVAL; // MD:  Return error if handle check fails

    // MD:  Allocate memory for context buffer
    buf = (char *) kzalloc(buflen, GFP_KERNEL);
    if (!buf)
        return -ENOMEM; // MD:  Return error if memory allocation fails

    // MD:  Get descriptor queue by ID
    descq = qdma_device_get_descq_by_id(xdev, id, buf, buflen, 0);
    if (!descq) {
        kfree(buf);
        return -EINVAL; // MD:  Return error if descriptor queue retrieval fails
    }

    // MD:  Dump the context of the descriptor queue
    rv = qdma_descq_context_dump(descq, buf, buflen);
    if (rv < 0) {
        pr_err("%s: Failed to dump the context, rv = %d",
                descq->conf.name, rv);
        return descq->xdev->hw.qdma_get_error_code(rv); // MD:  Return error code
    }

    buf[rv] = '\0'; // MD:  Null-terminate the buffer

    *data = buf;
    *data_len = buflen;

    printk(KERN_DEBUG "Read queue context for ID: %lu, size: %d\n", id, rv);
    return rv; // MD:  Return size read
}

/* MD: ****************************************************************************/
/* MD: *
 * qdbg_info_read() - reads queue info for a queue
 *
 * @param[in]	dev_hndl:	xdev device handle
 * @param[in]	id: queue handle
 * @param[out]	data: buffer pointer to collect the queue info
 * @param[out]	data_len: buffer len pointer
 * @param[in]	type: ring type
 *
 * @return	>0: size read
 * @return	<0: error
 *****************************************************************************/
static int qdbg_info_read(unsigned long dev_hndl, unsigned long id, char **data,
		int *data_len, enum dbgfs_desc_type type)
{
    int len = 0;
    char *buf = NULL;
    int buflen = DEBUGFS_QUEUE_INFO_SZ;
    struct qdma_descq *descq = NULL;
    struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;

    // MD:  Allocate memory for the buffer
    buf = (char *) kzalloc(buflen, GFP_KERNEL);
    if (!buf)
        return -ENOMEM; // MD:  Return error if memory allocation fails

    // MD:  Retrieve the descriptor queue by ID
    descq = qdma_device_get_descq_by_id(xdev, id, buf, buflen, 0);
    if (!descq) {
        kfree(buf); // MD:  Free allocated memory if retrieval fails
        return -EINVAL; // MD:  Return error if descriptor queue is not found
    }

    // MD:  Dump the state of the descriptor queue into the buffer
    len = qdma_descq_dump_state(descq, buf + len, buflen - len);

    // MD:  Assign buffer and its length to output parameters
    *data = buf;
    *data_len = buflen;

    printk(KERN_DEBUG "Read queue info for ID: %lu, size: %d\n", id, len);
    return len; // MD:  Return the size read
}

/* MD: ****************************************************************************/
/* MD: *
 * qdbg_desc_read() - reads descriptors of a queue
 *
 * @param[in]	dev_hndl:	xdev device handle
 * @param[in]	id: queue handle
 * @param[out]	data: buffer pointer to collect the queue descriptors
 * @param[out]	data_len: buffer len pointer
 * @param[in]	type: ring type
 *
 * @return	>0: size read
 * @return	<0: error
 *****************************************************************************/
static int qdbg_desc_read(unsigned long dev_hndl, unsigned long id, char **data,
		int *data_len, enum dbgfs_desc_type type)
{
    int len = 0;
    int rngsz = 0;
    struct qdma_descq *descq = NULL;
    struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;
    char *buf = NULL;
    int buflen = 0;

    // MD:  Retrieve the descriptor queue by ID
    descq = qdma_device_get_descq_by_id(xdev, id, buf, buflen, 0);
    if (!descq)
        return -EINVAL; // MD:  Return error if descriptor queue is not found

    // MD:  Determine the ring size based on the descriptor type
    if (type != DBGFS_DESC_TYPE_CMPT)
        rngsz = descq->conf.rngsz;
    else
        rngsz = descq->conf.rngsz_cmpt;

    // MD:  Calculate buffer length, including space for header
    buflen = (rngsz * DEBUGFS_QUEUE_DESC_SZ) + 128;

    // MD:  Allocate memory for the buffer
    buf = (char *) kzalloc(buflen, GFP_KERNEL);
    if (!buf)
        return -ENOMEM; // MD:  Return error if memory allocation fails

    // MD:  Dump the descriptors or completion entries into the buffer
    if (type != DBGFS_DESC_TYPE_CMPT) {
        len += qdma_queue_dump_desc(dev_hndl, id,
                0, rngsz-1, buf + len, buflen - len);
    } else {
        len += qdma_queue_dump_cmpt(dev_hndl, id,
                0, rngsz-1, buf + len, buflen - len);
    }

    // MD:  Assign buffer and its length to output parameters
    *data = buf;
    *data_len = buflen;

    printk(KERN_DEBUG "Read queue descriptors for ID: %lu, size: %d\n", id, len);
    return len; // MD:  Return the size read
}

/* MD: ****************************************************************************/
/* MD: *
 * q_dbg_file_read() - static function that provides common read
 *
 * @param[in]	fp:	pointer to file structure
 * @param[out]	user_buffer: pointer to user buffer
 * @param[in]	count: size of data to read
 * @param[in/out]	ppos: pointer to offset read
 * @param[in]	type: information type
 *
 * @return	>0: size read
 * @return	<0: error
 *****************************************************************************/
static ssize_t q_dbg_file_read(struct file *fp, char __user *user_buffer,
		size_t count, loff_t *ppos, enum dbgfs_queue_info_type type)
{
    char *buf = NULL;
    int buf_len = 0, len = 0, rv = 0;
    struct dbgfs_q_priv *qpriv = (struct dbgfs_q_priv *)fp->private_data;

    // MD:  Check if data is already available in the private structure
    if (qpriv->data == NULL && qpriv->datalen == 0) {
        // MD:  Read the appropriate data based on the type
        if (type == DBGFS_QINFO_INFO) {
            rv = qdbg_info_read(qpriv->dev_hndl, qpriv->qhndl,
                    &buf, &buf_len, DBGFS_DESC_TYPE_C2H);
        } else if (type == DBGFS_QINFO_CNTXT) {
            rv = qdbg_cntxt_read(qpriv->dev_hndl, qpriv->qhndl,
                    &buf, &buf_len, DBGFS_DESC_TYPE_C2H);
        } else if (type == DBGFS_QINFO_DESC) {
            rv = qdbg_desc_read(qpriv->dev_hndl, qpriv->qhndl,
                    &buf, &buf_len, DBGFS_DESC_TYPE_C2H);
        }

        // MD:  Check for errors during data retrieval
        if (rv < 0)
            goto q_dbg_file_read_exit;

        // MD:  Store the retrieved data in the private structure
        qpriv->datalen = rv;
        qpriv->data = buf;
    }

    buf = qpriv->data;
    len = qpriv->datalen;

    // MD:  Ensure buffer is valid
    if (!buf)
        goto q_dbg_file_read_exit;

    // MD:  Write data to user buffer
    if (*ppos >= len) {
        rv = 0; // MD:  No more data to read
        goto q_dbg_file_read_exit;
    }

    // MD:  Adjust count if it exceeds the remaining data length
    if (*ppos + count > len)
        count = len - *ppos;

    // MD:  Copy data to user space
    if (copy_to_user(user_buffer, buf + *ppos, count)) {
        rv = -EFAULT; // MD:  Return error if copy fails
        goto q_dbg_file_read_exit;
    }

    *ppos += count; // MD:  Update the position
    rv = count; // MD:  Return the number of bytes read

    pr_debug("Number of bytes written to user space is %zu\n", count);

q_dbg_file_read_exit:
    // MD:  Free the buffer and reset private data
    kfree(buf);
    qpriv->data = NULL;
    qpriv->datalen = 0;
    return rv;
}

/* MD: ****************************************************************************/
/* MD: *
 * q_info_open() - static function that executes info file open
 *
 * @param[in]	inode:	pointer to file inode
 * @param[in]	fp:	pointer to file structure
 *
 * @return	0: success
 * @return	<0: error
 *****************************************************************************/
static int q_info_open(struct inode *inode, struct file *fp)
{
    // MD:  Open the debug file for queue information
    printk(KERN_DEBUG "Opening queue info debug file\n");
    return q_dbg_file_open(inode, fp);
}

/* MD: ****************************************************************************/
/* MD: *
 * q_info_read() - static function that executes info file read
 *
 * @param[in]	fp:	pointer to file structure
 * @param[out]	user_buffer: pointer to user buffer
 * @param[in]	count: size of data to read
 * @param[in/out]	ppos: pointer to offset read
 *
 * @return	>0: size read
 * @return	<0: error
 *****************************************************************************/
static ssize_t q_info_read(struct file *fp, char __user *user_buffer,
		size_t count, loff_t *ppos)
{
    // MD:  Read the queue information from the debug file
    printk(KERN_DEBUG "Reading queue info debug file\n");
    return q_dbg_file_read(fp, user_buffer, count, ppos, DBGFS_QINFO_INFO);
}

/* MD: ****************************************************************************/
/* MD: *
 * q_cntxt_open() - static function that executes cntxt file open
 *
 * @param[in]	inode:	pointer to file inode
 * @param[in]	fp:	pointer to file structure
 *
 * @return	0: success
 * @return	<0: error
 *****************************************************************************/
static int q_cntxt_open(struct inode *inode, struct file *fp)
{
    // MD:  Open the debug file for queue context
    printk(KERN_DEBUG "Opening queue context debug file\n");
    return q_dbg_file_open(inode, fp);
}

/* MD: ****************************************************************************/
/* MD: *
 * q_cntxt_read() - static function that performs cntxt file read
 *
 * @param[in]	fp:	pointer to file structure
 * @param[out]	user_buffer: pointer to user buffer
 * @param[in]	count: size of data to read
 * @param[in/out]	ppos: pointer to offset read
 *
 * @return	>0: size read
 * @return	<0: error
 *****************************************************************************/
static ssize_t q_cntxt_read(struct file *fp, char __user *user_buffer,
		size_t count, loff_t *ppos)
{
    // MD:  Read the queue context from the debug file
    printk(KERN_DEBUG "Reading queue context debug file\n");
    return q_dbg_file_read(fp, user_buffer, count, ppos, DBGFS_QINFO_CNTXT);
}

/* MD: ****************************************************************************/
/* MD: *
 * q_desc_open() - static function that executes desc file open
 *
 * @param[in]	inode:	pointer to file inode
 * @param[in]	fp:	pointer to file structure
 *
 * @return	0: success
 * @return	<0: error
 *****************************************************************************/
static int q_desc_open(struct inode *inode, struct file *fp)
{
    // MD:  Open the debug file for queue descriptor
    printk(KERN_DEBUG "Opening queue descriptor debug file\n");
    return q_dbg_file_open(inode, fp);
}

/* MD: ****************************************************************************/
/* MD: *
 * q_desc_read() - static function that executes desc read
 *
 * @param[in]	fp:	pointer to file structure
 * @param[out]	user_buffer: pointer to user buffer
 * @param[in]	count: size of data to read
 * @param[in/out]	ppos: pointer to offset read
 *
 * @return	>0: size read
 * @return	<0: error
 *****************************************************************************/
static ssize_t q_desc_read(struct file *fp, char __user *user_buffer,
		size_t count, loff_t *ppos)
{
    // MD:  Read the queue descriptor from the debug file
    printk(KERN_DEBUG "Reading queue descriptor debug file\n");
    return q_dbg_file_read(fp, user_buffer, count, ppos, DBGFS_QINFO_DESC);
}

/* MD: ****************************************************************************/
/* MD: *
 * create_q_dbg_files() - static function to create queue debug files
 *
 * @param[in]	queue_root:	debugfs root for a queue
 *
 * @return	0: success
 * @return	<0: error
 *****************************************************************************/
static int create_q_dbg_files(struct qdma_descq *descq,
		struct dentry *queue_root)
{
    struct dentry  *fp[DBGFS_QINFO_END] = { NULL };
    struct file_operations *fops = NULL;
    int i = 0;

    // MD:  Initialize the debug file structures
    memset(qf, 0, sizeof(struct dbgfs_q_dbgf) * DBGFS_QINFO_END);

    // MD:  Iterate over each debug file type and set up file operations
    for (i = 0; i < DBGFS_QINFO_END; i++) {
        fops = &qf[i].fops;
        fops->owner = THIS_MODULE;
        switch (i) {
        case DBGFS_QINFO_INFO:
            // MD:  Set up file operations for queue info
            snprintf(qf[i].name, DBGFS_DBG_FNAME_SZ, "%s", "info");
            fops->open = q_info_open;
            fops->read = q_info_read;
            fops->release = q_dbg_file_release;
            printk(KERN_DEBUG "Setting up queue info debug file operations\n");
            break;
        case DBGFS_QINFO_CNTXT:
            // MD:  Set up file operations for queue context
            snprintf(qf[i].name, DBGFS_DBG_FNAME_SZ, "%s", "cntxt");
            fops->open = q_cntxt_open;
            fops->read = q_cntxt_read;
            fops->release = q_dbg_file_release;
            printk(KERN_DEBUG "Setting up queue context debug file operations\n");
            break;
        case DBGFS_QINFO_DESC:
            // MD:  Set up file operations for queue descriptor
            snprintf(qf[i].name, DBGFS_DBG_FNAME_SZ, "%s", "desc");
            fops->open = q_desc_open;
            fops->read = q_desc_read;
            fops->release = q_dbg_file_release;
            printk(KERN_DEBUG "Setting up queue descriptor debug file operations\n");
            break;
        }
    }

    // MD:  Create the debug files in the filesystem
    for (i = 0; i < DBGFS_QINFO_END; i++) {
        fp[i] = debugfs_create_file(qf[i].name, 0644, queue_root,
				descq, &qf[i].fops);
        if (!fp[i]) {
            printk(KERN_ERR "Failed to create debug file: %s\n", qf[i].name);
            return -1; // MD:  Return error if file creation fails
        }
    }
    printk(KERN_DEBUG "Successfully created all debug files\n");
    return 0;
}

/* MD: ****************************************************************************/
/* MD: *
 * dbgfs_queue_init() - queue initialization function
 *
 * @param[in]	conf:	queue configuration
 * @param[in]	pair_conf:	pair queue configuration
 * @param[in]	dbgfs_queues_root:	root directory for all queues
 *
 * @return	0: success
 * @return	<0: error
 *****************************************************************************/
int dbgfs_queue_init(struct qdma_queue_conf *conf,
		struct qdma_descq *pairq,
		struct dentry *dbgfs_queues_root)
{
    char qname[16] = {0}; // MD:  Buffer to store queue name
    char qdir[8] = {0}; // MD:  Buffer to store queue direction
    struct dentry *dbgfs_qidx_root = NULL;
    struct dentry *dbgfs_queue_root = NULL;
    struct dentry *dbgfs_cmpt_queue_root = NULL;
    struct qdma_descq *descq = container_of(conf, struct qdma_descq, conf);
    int rv = 0;

    if (!descq)
        return -EINVAL; // MD:  Return error if descriptor queue is NULL

    snprintf(qname, 16, "%u", conf->qidx); // MD:  Format queue index as string

    spin_lock(&descq->xdev->qidx_lock); // MD:  Lock the queue index

    /* MD: * Create queue root only if it is not created */
    if (pairq->dbgfs_qidx_root) {
        dbgfs_qidx_root = pairq->dbgfs_qidx_root;
    } else {
        /* MD:  Create a directory for the queue in debugfs */
        dbgfs_qidx_root = debugfs_create_dir(qname, dbgfs_queues_root);
        if (!dbgfs_qidx_root) {
            pr_err("Failed to create queue [%s] directory\n", qname);
            spin_unlock(&descq->xdev->qidx_lock);
            return -1; // MD:  Return error if directory creation fails
        }
    }

    /* MD:  Determine the direction of the queue and create a directory */
    if (conf->q_type == Q_C2H)
        snprintf(qdir, 8, "%s", "c2h");
    else if (conf->q_type == Q_H2C)
        snprintf(qdir, 8, "%s", "h2c");
    else
        snprintf(qdir, 8, "%s", "cmpt");

    dbgfs_queue_root = debugfs_create_dir(qdir, dbgfs_qidx_root);
    if (!dbgfs_queue_root) {
        pr_err("Failed to create %s directory under %s", qdir, qname);
        goto dbgfs_queue_init_fail; // MD:  Jump to failure handling
    }

    /* MD:  Create a directory for the completion queue if applicable */
    if ((conf->q_type == Q_C2H) && conf->st) {
        dbgfs_cmpt_queue_root = debugfs_create_dir("cmpt", dbgfs_qidx_root);
        if (!dbgfs_cmpt_queue_root) {
            pr_err("Failed to create cmpt directory under %s", qname);
            goto dbgfs_queue_init_fail; // MD:  Jump to failure handling
        }
    }

    /* MD:  Initialize file operations and create all the files */
    rv = create_q_dbg_files(descq, dbgfs_queue_root);
    if (rv < 0) {
        pr_err("Failed to create qdbg files, removing %s dir\n", qdir);
        debugfs_remove_recursive(dbgfs_queue_root);
        goto dbgfs_queue_init_fail; // MD:  Jump to failure handling
    }

    if (dbgfs_cmpt_queue_root) {
        rv = create_cmpt_q_dbg_files(descq, dbgfs_cmpt_queue_root);
        if (rv < 0) {
            pr_err("Failed to create cmptq dbg files, removing cmpt dir\n");
            debugfs_remove_recursive(dbgfs_cmpt_queue_root);
            goto dbgfs_queue_init_fail; // MD:  Jump to failure handling
        }
    }

    // MD:  Assign the created directories to the descriptor queue
    descq->dbgfs_qidx_root = dbgfs_qidx_root;
    descq->dbgfs_queue_root = dbgfs_queue_root;
    descq->dbgfs_cmpt_queue_root = dbgfs_cmpt_queue_root;
    spin_unlock(&descq->xdev->qidx_lock); // MD:  Unlock the queue index

    return 0; // MD:  Return success

dbgfs_queue_init_fail:
    if (pairq->dbgfs_qidx_root) {
        spin_unlock(&descq->xdev->qidx_lock);
        return -1; // MD:  Return error if pair queue root exists
    }
    pr_err("Failed to init q debug files, removing [%s] dir\n", qname);
    debugfs_remove_recursive(dbgfs_qidx_root);
    spin_unlock(&descq->xdev->qidx_lock);
    return -1; // MD:  Return error
}

/* MD: ****************************************************************************/
/* MD: *
 * dbgfs_queue_exit() - Debugfs queue teardown function
 *
 * This function is responsible for cleaning up the debugfs entries 
 * associated with a given queue configuration. It removes the debugfs 
 * directories recursively to free resources.
 *
 * @param[in] conf: Queue configuration structure pointer.
 * @param[in] pairq: Pair queue configuration structure pointer (can be NULL).
 *
 *****************************************************************************/
void dbgfs_queue_exit(struct qdma_queue_conf *conf,
                      struct qdma_descq *pairq)
{
    // MD:  Retrieve the qdma_descq structure containing 'conf'.
    struct qdma_descq *descq = container_of(conf, struct qdma_descq, conf);

    // MD:  Check if descq is NULL to avoid dereferencing a null pointer.
    if (!descq) {
        printk(KERN_DEBUG "dbgfs_queue_exit: descq is NULL, skipping cleanup.\n");
        return;
    }

    // MD:  Remove the debugfs directory for this queue and its associated components.
    debugfs_remove_recursive(descq->dbgfs_queue_root);
    printk(KERN_DEBUG "dbgfs_queue_exit: Removed dbgfs_queue_root for queue.\n");

    debugfs_remove_recursive(descq->dbgfs_cmpt_queue_root);
    printk(KERN_DEBUG "dbgfs_queue_exit: Removed dbgfs_cmpt_queue_root for queue.\n");

    // MD:  Set the debugfs pointers to NULL after cleanup.
    descq->dbgfs_queue_root = NULL;
    descq->dbgfs_cmpt_queue_root = NULL;

    // MD:  If the pair queue is NULL, remove the queue index root directory.
    if (!pairq) {
        debugfs_remove_recursive(descq->dbgfs_qidx_root);
        printk(KERN_DEBUG "dbgfs_queue_exit: Removed dbgfs_qidx_root for queue.\n");
    }

    // MD:  Set the queue index root pointer to NULL after cleanup.
    descq->dbgfs_qidx_root = NULL;

    printk(KERN_DEBUG "dbgfs_queue_exit: Cleanup completed for queue.\n");
}

#endif
