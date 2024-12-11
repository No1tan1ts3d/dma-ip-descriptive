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

#ifdef DEBUGFS
#define pr_fmt(fmt) KBUILD_MODNAME ":%s: " fmt, __func__

#include "qdma_debugfs_dev.h"
#include "qdma_reg_dump.h"
#include "qdma_access_common.h"
#include "qdma_context.h"
#include "libqdma_export.h"
#include "qdma_intr.h"
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/mutex.h>

// MD:  Enumeration for debug file types
enum dbgfs_dev_dbg_file_type {
	DBGFS_DEV_DBGF_INFO = 0,
	DBGFS_DEV_DBGF_REGS = 1,
	DBGFS_DEV_DBGF_REG_INFO = 2,
	DBGFS_DEV_DBGF_END,
};

// MD:  Enumeration for interrupt file types
enum dbgfs_dev_intr_file_type {
	DBFS_DEV_INTR_CTX = 0,
	DBFS_DEV_INTR_ENTRIES = 1,
	DBGFS_DEV_INTR_END,
};

// MD:  Structure for debug file operations
struct dbgfs_dev_dbgf {
	char name[DBGFS_DBG_FNAME_SZ];
	struct file_operations fops;
};

// MD:  Structure for interrupt debug file operations
struct dbgfs_dev_intr_dbgf {
	char name[DBGFS_DBG_FNAME_SZ];
	struct file_operations fops;
};

// MD:  Structure for device private data
struct dbgfs_dev_priv {
	unsigned long dev_hndl;
	char dev_name[QDMA_DEV_NAME_SZ];
	char *data;
	int datalen;
};

// MD:  Enumeration for BAR types
enum bar_type {
	DEBUGFS_BAR_CONFIG = 0,
	DEBUGFS_BAR_USER = 1,
	DEBUGFS_BAR_BYPASS = 2,
};

// MD:  Array to hold debug file operations
static struct dbgfs_dev_dbgf dbgf[DBGFS_DEV_DBGF_END];

// MD:  Array to hold interrupt file operations
static struct dbgfs_dev_dbgf dbg_intrf[DBGFS_DEV_INTR_END];

/* MD: ****************************************************************************/
/* MD: *
 * dump_banner() - static helper function to dump a device banner
 *
 * @param[in]	dev_name:	qdma device name
 * @param[out]	buf:	buffer to which banner is dumped
 * @param[in]	buf_sz:	size of the buffer passed to func
 *
 * @return	len: length of the buffer printed
 *****************************************************************************/
static int dump_banner(char *dev_name, char *buf, int buf_sz)
{
	int len = 0;
	char seperator[81] = {0};

	// MD:  Fill the separator with '#' characters
	memset(seperator, '#', 80);

	/* MD: * Banner consumes three lines, so size should be min 240 (80 * 3)
	 * If changed, check the minimum buffer size required
	 */
	if (buf_sz < (3 * DEBGFS_LINE_SZ))
		return -1;

	// MD:  Print the separator line
	len += snprintf(buf + len, buf_sz - len, "%s\n", seperator);
	// MD:  Print the device name and banner
	len += snprintf(buf + len, buf_sz - len,
			"###\t\t\t\tqdma%s, reg dump\n",
			dev_name);
	// MD:  Print the separator line again
	len += snprintf(buf + len, buf_sz - len, "%s\n", seperator);

	return len;
}

#define BANNER_LEN (81 * 5)

/* MD: ****************************************************************************/
/* MD: *
 * dbgfs_dump_qdma_regs() - static function to dump qdma device registers
 *
 * @param[in]	dev_hndl:	handle to the qdma device
 * @param[in]	dev_name:	name of the qdma device
 * @param[out]	data:	pointer to buffer where data will be stored
 * @param[out]	data_len:	length of the data buffer
 *
 * @return	len: length of the buffer printed
 * @return	<0: error
 *****************************************************************************/

static int dbgfs_dump_qdma_regs(unsigned long dev_hndl, char *dev_name,
		char **data, int *data_len)
{
	int len = 0; // MD:  Initialize length of data written to buffer
	int rv; // MD:  Return value to track success or failure of operations
	char *buf = NULL; // MD:  Pointer to buffer for storing register data
	int buflen; // MD:  Length of the buffer
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl; // MD:  Cast device handle to device structure

	// MD:  Check if the device handle is valid
	if (!xdev)
		return -EINVAL;

#ifndef __QDMA_VF__
	// MD:  Get the buffer length required for register dump for non-VF
	rv = qdma_acc_reg_dump_buf_len((void *)dev_hndl,
			xdev->version_info.ip_type,
			xdev->version_info.device_type, &buflen);
#else
	// MD:  Get the buffer length required for register dump for VF
	rv = qdma_acc_reg_dump_buf_len((void *)dev_hndl,
			xdev->version_info.ip_type,
			xdev->version_info.device_type, &buflen);
#endif
	if (rv < 0) {
		// MD:  Log an error message if getting buffer length fails
		pr_err("Failed to get reg dump buffer length\n");
		return rv;
	}
	buflen += BANNER_LEN; // MD:  Add space for the banner

	// MD:  Allocate memory for the buffer
	buf = (char *) kzalloc(buflen, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	// MD:  Print the banner with device info
	rv = dump_banner(dev_name, buf + len, buflen - len);
	if (rv < 0) {
		// MD:  Log a warning if there is insufficient space for the banner
		pr_warn("Insufficient space to dump register banner, err = %d\n", rv);
		kfree(buf); // MD:  Free the allocated buffer
		return len;
	}
	len += rv; // MD:  Update length with the number of bytes written

	// MD:  Dump the configuration registers
	rv = qdma_config_reg_dump(dev_hndl, buf + len, buflen - len);
	if (rv < 0) {
		// MD:  Log a warning if dumping configuration registers fails
		pr_warn("Not able to dump Config Bar register values, err = %d\n", rv);
		*data = buf; // MD:  Set data pointer to buffer
		*data_len = buflen; // MD:  Set data length to buffer length
		return len;
	}
	len += rv; // MD:  Update length with the number of bytes written

	*data = buf; // MD:  Set data pointer to buffer
	*data_len = buflen; // MD:  Set data length to buffer length

	return len; // MD:  Return the total length of data written to the buffer
}

/* MD: ****************************************************************************/
/* MD: *
 * dbgfs_dump_qdma_reg_info() - static function to dump qdma device registers
 *
 * @param[in]	dev_hndl:	handle to the qdma device
 * @param[in]	dev_name:	name of the qdma device
 * @param[out]	data:	pointer to buffer where data will be stored
 * @param[out]	data_len:	length of the data buffer
 *
 * @return	len: length of the buffer printed
 * @return	<0: error
 *****************************************************************************/
static int dbgfs_dump_qdma_reg_info(unsigned long dev_hndl, char *dev_name,
		char **data, int *data_len)
{
	int len = 0; // MD:  Initialize length of data written to buffer
	int rv; // MD:  Return value to track success or failure of operations
	char *buf = NULL; // MD:  Pointer to buffer for storing register data
	int buflen; // MD:  Length of the buffer
	int num_regs; // MD:  Number of registers to dump
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl; // MD:  Cast device handle to device structure

	// MD:  Check if the device handle is valid
	if (!xdev)
		return -EINVAL;

	// MD:  Get the buffer length required for register dump and number of registers
	rv = qdma_acc_reg_info_len((void *)dev_hndl,
			xdev->version_info.ip_type,
			xdev->version_info.device_type, &buflen, &num_regs);
	if (rv < 0) {
		// MD:  Log an error message if getting buffer length fails
		pr_err("Failed to get reg dump buffer length\n");
		return rv;
	}
	buflen += BANNER_LEN; // MD:  Add space for the banner

	// MD:  Allocate memory for the buffer
	buf = (char *) kzalloc(buflen, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	// MD:  Print the banner with device info
	rv = dump_banner(dev_name, buf + len, buflen - len);
	if (rv < 0) {
		// MD:  Log a warning if there is insufficient space for the banner
		pr_warn("Insufficient space to dump register banner, err = %d\n", rv);
		kfree(buf); // MD:  Free the allocated buffer
		return len;
	}
	len += rv; // MD:  Update length with the number of bytes written

	// MD:  Dump the configuration registers
	rv = qdma_config_reg_info_dump(dev_hndl, 0, num_regs, buf + len, buflen - len);
	if (rv < 0) {
		// MD:  Log a warning if dumping configuration registers fails
		pr_warn("Not able to dump Config Bar register values, err = %d\n", rv);
		*data = buf; // MD:  Set data pointer to buffer
		*data_len = buflen; // MD:  Set data length to buffer length
		return len;
	}
	len += rv; // MD:  Update length with the number of bytes written

	*data = buf; // MD:  Set data pointer to buffer
	*data_len = buflen; // MD:  Set data length to buffer length

	return len; // MD:  Return the total length of data written to the buffer
}

/* MD: ****************************************************************************/
/* MD: *
 * dbgfs_dump_qdma_info() - static function to dump qdma device registers
 *
 * @dev_hndl:	handle to the qdma device
 * @dev_name:	name of the qdma device
 * @data:	pointer to buffer where data will be stored
 * @data_len:	length of the data buffer
 *
 * @return	len: length of the buffer printed
 * @return	<0: error
 *****************************************************************************/
static int dbgfs_dump_qdma_info(unsigned long dev_hndl, char *dev_name,
		char **data, int *data_len)
{
	int len = 0; // MD:  Initialize length of data written to buffer
	char *buf = NULL; // MD:  Pointer to buffer for storing register data
	int buflen = DEBUGFS_DEV_INFO_SZ; // MD:  Length of the buffer
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl; // MD:  Cast device handle to device structure
	struct qdma_dev_conf *conf = NULL; // MD:  Pointer to device configuration

	// MD:  Check if the device handle is valid
	if (!xdev)
		return -EINVAL;

	// MD:  Validate the device handle
	if (xdev_check_hndl(__func__, xdev->conf.pdev, dev_hndl) < 0)
		return -EINVAL;

	conf = &xdev->conf; // MD:  Get the device configuration

	// MD:  Allocate memory for the buffer
	buf = (char *) kzalloc(buflen, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	// MD:  Dump various configuration details into the buffer
	len += snprintf(buf + len, buflen - len, "%-36s: %s\n", "Master PF",
			(conf->master_pf) ? "True" : "False");
	len += snprintf(buf + len, buflen - len, "%-36s: %d\n", "QBase",
			conf->qsets_base);
	len += snprintf(buf + len, buflen - len, "%-36s: %u\n", "Max Qsets",
			conf->qsets_max);
	len += snprintf(buf + len, buflen - len, "%-36s: %d\n",
			"Number of VFs", xdev->vf_count);
	len += snprintf(buf + len, buflen - len, "%-36s: %d\n",
			"Max number of VFs", conf->vf_max);
	len += snprintf(buf + len, buflen - len, "%-36s: %s mode\n",
			"Driver Mode",
			mode_name_list[conf->qdma_drv_mode].name);

	*data = buf; // MD:  Set data pointer to buffer
	*data_len = buflen; // MD:  Set data length to buffer length

	return len; // MD:  Return the total length of data written to the buffer
}

/* MD: ****************************************************************************/
/* MD: *
 * dbgfs_dump_intr_cntx() - static function to dump interrupt context
 *
 * @dev_hndl:	handle to the qdma device
 * @dev_name:	name of the qdma device
 * @data:	pointer to buffer where data will be stored
 * @data_len:	length of the data buffer
 *
 * @return	len: length of the buffer printed
 * @return	<0: error
 *****************************************************************************/
static int dbgfs_dump_intr_cntx(unsigned long dev_hndl, char *dev_name,
		char **data, int *data_len)
{
	int len = 0; // MD:  Initialize length of data written to buffer
	int rv = 0; // MD:  Return value to track success or failure of operations
	char *buf = NULL; // MD:  Pointer to buffer for storing interrupt context
	int i = 0; // MD:  Loop counter
	int ring_index = 0; // MD:  Index for interrupt rings
	struct qdma_indirect_intr_ctxt intr_ctxt; // MD:  Structure to hold interrupt context
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl; // MD:  Cast device handle to device structure
	int buflen = DEBUGFS_INTR_CNTX_SZ; // MD:  Length of the buffer

	// MD:  Allocate memory for the buffer
	buf = (char *) kzalloc(buflen, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	// MD:  Check if interrupt aggregation is enabled and add the interrupt context
	if ((xdev->conf.qdma_drv_mode == INDIRECT_INTR_MODE) ||
			(xdev->conf.qdma_drv_mode == AUTO_MODE)) {
		for (i = 0; i < QDMA_NUM_DATA_VEC_FOR_INTR_CXT; i++) {
			// MD:  Get the interrupt ring index
			ring_index = get_intr_ring_index(
						xdev,
						(i + xdev->dvec_start_idx));
			// MD:  Read the interrupt context for the current ring
			rv = qdma_intr_context_read(
						xdev,
						ring_index,
						&intr_ctxt);
			if (rv < 0) {
				// MD:  Log an error message if reading the interrupt context fails
				len += snprintf(buf + len, buflen - len,
					"%s read intr context failed %d.\n",
					xdev->conf.name, rv);
				*data = buf; // MD:  Set data pointer to buffer
				*data_len = buflen; // MD:  Set data length to buffer length
				return rv;
			}

			// MD:  Dump the interrupt context into the buffer
			rv = xdev->hw.qdma_dump_intr_context(xdev,
						&intr_ctxt, ring_index,
						buf + len, buflen - len);
			if (rv < 0) {
				pr_err("Failed to dump intr context, rv = %d",
						rv);
				return xdev->hw.qdma_get_error_code(rv);
			}
			len += rv; // MD:  Update length with the number of bytes written
		}
	}

	*data = buf; // MD:  Set data pointer to buffer
	*data_len = buflen; // MD:  Set data length to buffer length

	return len; // MD:  Return the total length of data written to the buffer
}

/* MD: ****************************************************************************/
/* MD: *
 * dbgfs_dump_intr_ring() - static function to dump interrupt ring
 *
 * @param[in]	dev_hndl:	handle to the qdma device
 * @param[in]	dev_name:	name of the qdma device
 * @param[out]	data:	pointer to buffer where data will be stored
 * @param[out]	data_len:	length of the data buffer
 *
 * @return	len: length of the buffer printed
 * @return	<0: error
 *****************************************************************************/
static int dbgfs_dump_intr_ring(unsigned long dev_hndl, char *dev_name,
		char **data, int *data_len)
{
	int len = 0; // MD:  Initialize length of data written to buffer
	int rv = 0; // MD:  Return value to track success or failure
	char *buf = NULL; // MD:  Buffer to store the interrupt ring data
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl; // MD:  Cast device handle to device structure

	// MD:  Get the vector index for the interrupt ring
	unsigned int vector_idx = xdev->msix[xdev->dvec_start_idx].entry;
	// MD:  Calculate the number of entries in the interrupt ring
	int num_entries = (xdev->conf.intr_rngsz + 1) * 512;
	// MD:  Calculate the buffer length required
	int buflen = (45 * num_entries) + 1;

	// MD:  Allocate memory for the buffer
	buf = (char *) kzalloc(buflen, GFP_KERNEL);
	if (!buf)
		return -ENOMEM; // MD:  Return error if memory allocation fails

	// MD:  Dump the interrupt ring into the buffer
	rv = qdma_intr_ring_dump(dev_hndl, vector_idx, 0,
		 num_entries - 1, buf, buflen);
	if (rv < 0) {
		// MD:  Log an error message if dumping the interrupt ring fails
		len += snprintf(buf + len, buflen - len,
					   "%s read intr context failed %d.\n",
					   xdev->conf.name, rv);
		kfree(buf); // MD:  Free the allocated buffer
		return rv; // MD:  Return the error code
	}

	len = strlen(buf); // MD:  Update length with the number of bytes written
	buf[len++] = '\n'; // MD:  Add a newline character at the end
	*data = buf; // MD:  Set the data pointer to the buffer
	*data_len = buflen; // MD:  Set the data length

	return len; // MD:  Return the total length of data written to the buffer
}

/* MD: ****************************************************************************/
/* MD: *
 * dev_dbg_file_open() - static function that provides generic open
 *
 * @param[in]	inode:	pointer to file inode
 * @param[in]	fp:	pointer to file structure
 *
 * @return	0: success
 * @return	<0: error
 *****************************************************************************/
static int dev_dbg_file_open(struct inode *inode, struct file *fp)
{
	unsigned long dev_id = -1; // MD:  Device ID initialized to an invalid value
	int rv = 0; // MD:  Return value to track success or failure
	unsigned char dev_name[QDMA_DEV_NAME_SZ] = {0}; // MD:  Buffer to store device name
	unsigned char *lptr = NULL, *rptr = NULL; // MD:  Pointers for parsing device name
	struct dentry *dev_dir = NULL; // MD:  Directory entry for the device
	struct dbgfs_dev_priv *priv = NULL; // MD:  Private data structure for the device
	struct xlnx_dma_dev *xdev = NULL; // MD:  Pointer to the device structure

	if (!inode || !fp)
		return -EINVAL; // MD:  Return error if inode or file pointer is invalid

	dev_dir = fp->f_path.dentry->d_parent; // MD:  Get the parent directory entry
	xdev = inode->i_private; // MD:  Get the device structure from inode
	if (!xdev)
		return -EINVAL; // MD:  Return error if device structure is invalid

	// MD:  Convert colon-separated b:d:f to a hex string
	rptr = dev_dir->d_iname;
	lptr = dev_name;
	while (*rptr) {
		if (*rptr == ':') {
			rptr++;
			continue;
		}
		*lptr++ = *rptr++;
	}

	// MD:  Convert the hex string to a device ID
	rv = kstrtoul((const char *)dev_name, 16, &dev_id);
	if (rv < 0) {
		rv = -ENODEV; // MD:  Set error code for invalid device
		return rv;
	}

	// MD:  Allocate memory for the private data structure
	priv = (struct dbgfs_dev_priv *) kzalloc(sizeof(struct dbgfs_dev_priv),
			GFP_KERNEL);
	if (!priv) {
		rv = -ENOMEM; // MD:  Set error code for memory allocation failure
		return rv;
	}

	priv->dev_hndl = (unsigned long)xdev; // MD:  Set the device handle
	snprintf(priv->dev_name, QDMA_DEV_NAME_SZ, "%s", dev_name); // MD:  Copy the device name
	fp->private_data = priv; // MD:  Set the private data for the file

	return 0; // MD:  Return success
}

/* MD: ****************************************************************************/
/* MD: *
 * dev_dbg_file_release() - static function that provides generic release
 *
 * @param[in]	inode:	pointer to file inode
 * @param[in]	fp:	pointer to file structure
 *
 * @return	0: success
 * @return	<0: error
 *****************************************************************************/
static int dev_dbg_file_release(struct inode *inode, struct file *fp)
{
    // MD:  Free the private data associated with the file pointer
    kfree(fp->private_data);

    // MD:  Set the private data pointer to NULL to avoid dangling references
    fp->private_data = NULL;

    return 0; // MD:  Return success
}

/* MD: ****************************************************************************/
/* MD: *
 * dev_dbg_file_read() - static function that provides common read
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
static ssize_t dev_dbg_file_read(struct file *fp, char __user *user_buffer,
		size_t count, loff_t *ppos, enum dbgfs_dev_dbg_file_type type)
{
    char *buf = NULL;
    int buf_len = 0;
    int len = 0;
    int rv = 0;
    struct dbgfs_dev_priv *dev_priv =
        (struct dbgfs_dev_priv *)fp->private_data;

    // MD:  Check if data is not already loaded
    if (dev_priv->data == NULL && dev_priv->datalen == 0) {
        // MD:  Load data based on the type of debug file
        if (type == DBGFS_DEV_DBGF_INFO) {
            rv = dbgfs_dump_qdma_info(dev_priv->dev_hndl,
                    dev_priv->dev_name, &buf, &buf_len);
        } else if (type == DBGFS_DEV_DBGF_REGS) {
            rv = dbgfs_dump_qdma_regs(dev_priv->dev_hndl,
                    dev_priv->dev_name, &buf, &buf_len);
        } else if (type == DBGFS_DEV_DBGF_REG_INFO) {
            rv = dbgfs_dump_qdma_reg_info(dev_priv->dev_hndl,
                    dev_priv->dev_name, &buf, &buf_len);
        }

        // MD:  Check for errors during data loading
        if (rv < 0)
            goto dev_dbg_file_read_exit;

        // MD:  Store the loaded data and its length in the private structure
        dev_priv->datalen = rv;
        dev_priv->data = buf;
    }

    buf = dev_priv->data;
    len = dev_priv->datalen;

    // MD:  If no data is available, exit
    if (!buf)
        goto dev_dbg_file_read_exit;

    // MD:  Check if the read position is beyond the data length
    if (*ppos >= len) {
        rv = 0;
        goto dev_dbg_file_read_exit;
    }

    // MD:  Adjust count if it exceeds the available data length
    if (*ppos + count > len)
        count = len - *ppos;

    // MD:  Copy data to user buffer and check for errors
    if (copy_to_user(user_buffer, buf + *ppos, count)) {
        rv = -EFAULT;
        goto dev_dbg_file_read_exit;
    }

    // MD:  Update the position pointer and return the number of bytes read
    *ppos += count;
    rv = count;

    pr_debug("Number of bytes written to user space is %zu\n", count);

dev_dbg_file_read_exit:
    // MD:  Free the buffer and reset the private data
    kfree(buf);
    dev_priv->data = NULL;
    dev_priv->datalen = 0;
    return rv;
}

/* MD: ****************************************************************************/
/* MD: *
 * dev_intr_file_read() - static function that provides common read
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
static ssize_t dev_intr_file_read(struct file *fp, char __user *user_buffer,
		size_t count, loff_t *ppos, enum dbgfs_dev_intr_file_type type)
{
    char *buf = NULL;
    int buf_len = 0;
    int len = 0;
    int rv = 0;
    struct dbgfs_dev_priv *dev_priv =
        (struct dbgfs_dev_priv *)fp->private_data;

    // MD:  Check if data is not already loaded
    if (dev_priv->data == NULL && dev_priv->datalen == 0) {
        // MD:  Load data based on the type of interrupt file
        if (type == DBFS_DEV_INTR_CTX) {
            rv = dbgfs_dump_intr_cntx(dev_priv->dev_hndl,
                    dev_priv->dev_name, &buf, &buf_len);
        } else if (type == DBFS_DEV_INTR_ENTRIES) {
            rv = dbgfs_dump_intr_ring(dev_priv->dev_hndl,
                    dev_priv->dev_name, &buf, &buf_len);
        }

        // MD:  Check for errors during data loading
        if (rv < 0)
            goto dev_intr_file_read_exit;

        // MD:  Store the loaded data and its length in the private structure
        dev_priv->datalen = rv;
        dev_priv->data = buf;
    }

    buf = dev_priv->data;
    len = dev_priv->datalen;

    // MD:  If no data is available, exit
    if (!buf)
        goto dev_intr_file_read_exit;

    // MD:  Check if the read position is beyond the data length
    if (*ppos >= len) {
        rv = 0;
        goto dev_intr_file_read_exit;
    }

    // MD:  Adjust count if it exceeds the available data length
    if (*ppos + count > len)
        count = len - *ppos;

    // MD:  Copy data to user buffer and check for errors
    if (copy_to_user(user_buffer, buf + *ppos, count)) {
        rv = -EFAULT;
        goto dev_intr_file_read_exit;
    }

    // MD:  Update the position pointer and return the number of bytes read
    *ppos += count;
    rv = count;

    pr_debug("Number of bytes written to user space is %zu\n", count);

dev_intr_file_read_exit:
    // MD:  Free the buffer and reset the private data
    kfree(buf);
    dev_priv->data = NULL;
    dev_priv->datalen = 0;
    return rv;
}

/* MD: ****************************************************************************/
/* MD: *
 * dev_info_open() - static function that executes info open
 *
 * @param[in]	inode:	pointer to file inode
 * @param[in]	fp:	pointer to file structure
 *
 * @return	0: success
 * @return	<0: error
 *****************************************************************************/
static int dev_info_open(struct inode *inode, struct file *fp)
{
    // MD:  Open the debug file for device information
    pr_debug("Opening device info file\n");
    return dev_dbg_file_open(inode, fp);
}

/* MD: ****************************************************************************/
/* MD: *
 * dev_info_read() - static function that executes info read
 *
 * @param[in]	fp:	pointer to file structure
 * @param[out]	user_buffer: pointer to user buffer
 * @param[in]	count: size of data to read
 * @param[in/out]	ppos: pointer to offset read
 *
 * @return	>0: size read
 * @return	<0: error
 *****************************************************************************/
static ssize_t dev_info_read(struct file *fp, char __user *user_buffer,
		size_t count, loff_t *ppos)
{
    // MD:  Read the device information from the debug file
    pr_debug("Reading device info, count: %zu, position: %lld\n", count, *ppos);
    return dev_dbg_file_read(fp, user_buffer, count, ppos, DBGFS_DEV_DBGF_INFO);
}

/* MD: ****************************************************************************/
/* MD: *
 * dev_regs_open() - static function that opens regs debug file
 *
 * @param[in]	inode:	pointer to file inode
 * @param[in]	fp:	pointer to file structure
 *
 * @return	0: success
 * @return	<0: error
 *****************************************************************************/
static int dev_regs_open(struct inode *inode, struct file *fp)
{
    // MD:  Open the debug file for device registers
    pr_debug("Opening device registers file\n");
    return dev_dbg_file_open(inode, fp);
}

/* MD: ****************************************************************************/
/* MD: *
 * dev_reg_info_open() - static function that opens regInfo debug file
 *
 * @param[in]	inode:	pointer to file inode
 * @param[in]	fp:	pointer to file structure
 *
 * @return	0: success
 * @return	<0: error
 *****************************************************************************/
static int dev_reg_info_open(struct inode *inode, struct file *fp)
{
    // MD:  Open the debug file for register information
    pr_debug("Opening device register info file\n");
    return dev_dbg_file_open(inode, fp);
}

/* MD: ****************************************************************************/
/* MD: *
 * dev_regs_read() - static function that executes info read
 *
 * @param[in]	fp:	pointer to file structure
 * @param[out]	user_buffer: pointer to user buffer
 * @param[in]	count: size of data to read
 * @param[in/out]	ppos: pointer to offset read
 *
 * @return	>0: size read
 * @return	<0: error
 *****************************************************************************/
static ssize_t dev_regs_read(struct file *fp, char __user *user_buffer,
		size_t count, loff_t *ppos)
{
    // MD:  Read the device registers from the debug file
    pr_debug("Reading device registers, count: %zu, position: %lld\n", count, *ppos);
    return dev_dbg_file_read(fp, user_buffer, count, ppos, DBGFS_DEV_DBGF_REGS);
}

/* MD: ****************************************************************************/
/* MD: *
 * dev_reg_info_read() - static function that executes info read
 *
 * @param[in]	fp:	pointer to file structure
 * @param[out]	user_buffer: pointer to user buffer
 * @param[in]	count: size of data to read
 * @param[in/out]	ppos: pointer to offset read
 *
 * @return	>0: size read
 * @return	<0: error
 *****************************************************************************/
static ssize_t dev_reg_info_read(struct file *fp, char __user *user_buffer,
		size_t count, loff_t *ppos)
{
    // MD:  Read the register information from the debug file
    pr_debug("Reading register info, count: %zu, position: %lld\n", count, *ppos);
    return dev_dbg_file_read(fp, user_buffer, count, ppos, DBGFS_DEV_DBGF_REG_INFO);
}

/* MD: ****************************************************************************/
/* MD: *
 * dev_intr_cntx_open() - static function to open interrupt context debug file
 *
 * @param[in]	inode:	pointer to file inode
 * @param[in]	fp:	pointer to file structure
 *
 * @return	0: success
 * @return	<0: error
 *****************************************************************************/
static int dev_intr_cntx_open(struct inode *inode, struct file *fp)
{
    // MD:  Open the debug file for interrupt context
    pr_debug("Opening interrupt context file\n");
    return dev_dbg_file_open(inode, fp);
}

/* MD: ****************************************************************************/
/* MD: *
 * dev_intr_cntx_read() - static function that executes interrupt context read
 *			  file
 *
 * @param[in]	fp:	pointer to file structure
 * @param[out]	user_buffer: pointer to user buffer
 * @param[in]	count: size of data to read
 * @param[in/out]	ppos: pointer to offset read
 *
 * @return	>0: size read
 * @return	<0: error
 *****************************************************************************/
static ssize_t dev_intr_cntx_read(struct file *fp, char __user *user_buffer,
		size_t count, loff_t *ppos)
{
    // MD:  Read the interrupt context from the debug file
    pr_debug("Reading interrupt context, count: %zu, position: %lld\n", count, *ppos);
    return dev_intr_file_read(fp, user_buffer, count, ppos, DBFS_DEV_INTR_CTX);
}

/* MD: ****************************************************************************/
/* MD: *
 * dev_intr_ring_open() - static function to open interrupt ring entries file
 *
 * @param[in]	inode:	pointer to file inode
 * @param[in]	fp:	pointer to file structure
 *
 * @return	0: success
 * @return	<0: error
 *****************************************************************************/
static int dev_intr_ring_open(struct inode *inode, struct file *fp)
{
    // MD:  Open the debug file for interrupt ring entries
    pr_debug("Opening interrupt ring entries file\n");
    return dev_dbg_file_open(inode, fp);
}

/* MD: ****************************************************************************/
/* MD: *
 * dev_intr_ring_read() - static function that reads interrupt ring entries to
 *						  file
 *
 * @param[in]	fp:	pointer to file structure
 * @param[out]	user_buffer: pointer to user buffer
 * @param[in]	count: size of data to read
 * @param[in/out]	ppos: pointer to offset read
 *
 * @return	>0: size read
 * @return	<0: error
 *****************************************************************************/
static ssize_t dev_intr_ring_read(struct file *fp, char __user *user_buffer,
		size_t count, loff_t *ppos)
{
    // MD:  Read the interrupt ring entries from the debug file
    pr_debug("Reading interrupt ring entries, count: %zu, position: %lld\n", count, *ppos);
    return dev_intr_file_read(fp, user_buffer, count, ppos, DBFS_DEV_INTR_ENTRIES);
}

/* MD: ****************************************************************************/
/* MD: *
 * create_dev_dbg_files() - static function to create queue debug files
 *
 * @param[in]	dev_root:	debugfs root for the device
 *
 * @return	>0: size read
 * @return	<0: error
 *****************************************************************************/
static int create_dev_dbg_files(struct xlnx_dma_dev *xdev, struct dentry *dev_root)
{
    struct dentry *fp[DBGFS_DEV_DBGF_END] = { NULL };
    struct file_operations *fops = NULL;
    int i = 0;

    // MD:  Initialize the debug file operations structure
    memset(dbgf, 0, sizeof(struct dbgfs_dev_dbgf) * DBGFS_DEV_DBGF_END);

    // MD:  Set up file operations for each debug file type
    for (i = 0; i < DBGFS_DEV_DBGF_END; i++) {
        fops = &dbgf[i].fops;
        fops->owner = THIS_MODULE;
        switch (i) {
        case DBGFS_DEV_DBGF_INFO:
            snprintf(dbgf[i].name, 64, "%s", "qdma_info");
            fops->open = dev_info_open;
            fops->read = dev_info_read;
            fops->release = dev_dbg_file_release;
            break;
        case DBGFS_DEV_DBGF_REGS:
            snprintf(dbgf[i].name, 64, "%s", "qdma_regs");
            fops->open = dev_regs_open;
            fops->read = dev_regs_read;
            fops->release = dev_dbg_file_release;
            break;
        case DBGFS_DEV_DBGF_REG_INFO:
            snprintf(dbgf[i].name, 64, "%s", "qdma_reg_info");
            fops->open = dev_reg_info_open;
            fops->read = dev_reg_info_read;
            fops->release = dev_dbg_file_release;
            break;
        }
    }

    // MD:  Create the debug files in the debugfs root directory
    for (i = 0; i < DBGFS_DEV_DBGF_END; i++) {
        fp[i] = debugfs_create_file(dbgf[i].name, 0644, dev_root, xdev, &dbgf[i].fops);
        if (!fp[i]) {
            pr_err("Failed to create debug file: %s\n", dbgf[i].name);
            return -1;
        }
    }
    return 0;
}

/* MD: ****************************************************************************/
/* MD: *
 * create_dev_intr_files() - static function to create intr ring files
 *
 * @param[in]	dev_root:	debugfs root for the device
 *
 * @return	>0: size read
 * @return	<0: error
 *****************************************************************************/
static int create_dev_intr_files(struct xlnx_dma_dev *xdev,
		struct dentry *intr_root)
{
	struct dentry  *fp[DBGFS_DEV_DBGF_END] = { NULL }; // MD:  Array to hold file pointers
	struct file_operations *fops = NULL; // MD:  Pointer to file operations
	int i = 0;
	char intr_dir[16] = {0}; // MD:  Buffer to hold directory name
	struct dentry *dbgfs_intr_ring = NULL; // MD:  Pointer to the interrupt ring directory

	// MD:  Create a directory name based on the interrupt ring index
	snprintf(intr_dir, 8, "%d",
			 get_intr_ring_index(xdev,
			 xdev->dvec_start_idx));

	// MD:  Create a directory for the interrupt ring in debugfs
	dbgfs_intr_ring = debugfs_create_dir(intr_dir, intr_root);
	memset(dbg_intrf, 0, sizeof(
		struct dbgfs_dev_intr_dbgf) * DBGFS_DEV_INTR_END);

	// MD:  Initialize file operations for each interrupt debug file
	for (i = 0; i < DBGFS_DEV_INTR_END; i++) {
		fops = &dbg_intrf[i].fops;
		fops->owner = THIS_MODULE;
		switch (i) {
		case DBFS_DEV_INTR_CTX:
			snprintf(dbg_intrf[i].name, 64, "%s", "cntxt");
			fops->open = dev_intr_cntx_open;
			fops->read = dev_intr_cntx_read;
			fops->release = dev_dbg_file_release;
			break;
		case DBFS_DEV_INTR_ENTRIES:
			snprintf(dbg_intrf[i].name, 64, "%s", "entries");
			fops->open = dev_intr_ring_open;
			fops->read = dev_intr_ring_read;
			fops->release = dev_dbg_file_release;
			break;
		}
	}

	// MD:  Create the debug files for each interrupt type
	for (i = 0; i < DBGFS_DEV_INTR_END; i++) {
		fp[i] = debugfs_create_file(dbg_intrf[i].name, 0644,
					dbgfs_intr_ring,
					xdev, &dbg_intrf[i].fops);
		if (!fp[i])
			return -1; // MD:  Return error if file creation fails
	}
	return 0; // MD:  Return success
}

/* MD: ****************************************************************************/
/* MD: *
 * dbgfs_dev_init() - function to initialize device debugfs files
 *
 * @param[in]	xdev:	Xilinx dma device
 * @param[in]	qdma_debugfs_root:	root file in debugfs
 *
 * @return	=0: success
 * @return	<0: error
 *****************************************************************************/
int dbgfs_dev_init(struct xlnx_dma_dev *xdev)
{
	char dname[QDMA_DEV_NAME_SZ] = {0}; // MD:  Buffer for device name
	struct dentry *dbgfs_dev_root = NULL;
	struct dentry *dbgfs_queues_root = NULL;
	struct dentry *dbgfs_intr_root = NULL;
	struct pci_dev *pdev = xdev->conf.pdev;
	int rv = 0;

	// MD:  Check if the debugfs root for the device is already created
	if (!xdev->conf.debugfs_dev_root) {
		// MD:  Create a unique directory name for the device
		snprintf(dname, QDMA_DEV_NAME_SZ, "%04x:%02x:%02x:%x",
				pci_domain_nr(pdev->bus),
				pdev->bus->number,
				PCI_SLOT(pdev->devfn),
				PCI_FUNC(pdev->devfn));
		// MD:  Create a directory for the device in debugfs
		dbgfs_dev_root = debugfs_create_dir(dname, qdma_debugfs_root);
		if (!dbgfs_dev_root) {
			pr_err("Failed to create device directory\n");
			return -1; // MD:  Return error if directory creation fails
		}
		xdev->dbgfs_dev_root = dbgfs_dev_root;
	} else
		xdev->dbgfs_dev_root = xdev->conf.debugfs_dev_root;

	// MD:  Create debug files for the QDMA device
	rv = create_dev_dbg_files(xdev, xdev->dbgfs_dev_root);
	if (rv < 0) {
		pr_err("Failed to create device debug files\n");
		goto dbgfs_dev_init_fail;
	}

	// MD:  Create a directory for the queues in debugfs
	dbgfs_queues_root = debugfs_create_dir("queues", xdev->dbgfs_dev_root);
	if (!dbgfs_queues_root) {
		pr_err("Failed to create queues directory under device directory\n");
		goto dbgfs_dev_init_fail;
	}

	// MD:  If the driver mode supports interrupt rings, create them
	if ((xdev->conf.qdma_drv_mode == INDIRECT_INTR_MODE) ||
			(xdev->conf.qdma_drv_mode == AUTO_MODE)) {
		// MD:  Create a directory for the interrupt rings in debugfs
		dbgfs_intr_root = debugfs_create_dir("intr_rings",
				xdev->dbgfs_dev_root);
		if (!dbgfs_queues_root) {
			pr_err("Failed to create intr_ring directory under device directory\n");
			goto dbgfs_dev_init_fail;
		}

		// MD:  Create debug files for interrupt rings
		rv = create_dev_intr_files(xdev, dbgfs_intr_root);
		if (rv < 0) {
			pr_err("Failed to create intr ring files\n");
			goto dbgfs_dev_init_fail;
		}
	}
	xdev->dbgfs_queues_root = dbgfs_queues_root;
	xdev->dbgfs_intr_root = dbgfs_intr_root;
	spin_lock_init(&xdev->qidx_lock);

	return 0; // MD:  Return success

dbgfs_dev_init_fail:
	// MD:  Cleanup in case of failure
	debugfs_remove_recursive(xdev->dbgfs_dev_root);
	return -1; // MD:  Return error
}

/* MD: ****************************************************************************/
/* MD: *
 * dbgfs_dev_exit() - function to cleanup device debugfs files
 *
 * @param[in]	xdev:	Xilinx dma device
 *
 *****************************************************************************/
void dbgfs_dev_exit(struct xlnx_dma_dev *xdev)
{
	// MD:  Remove the debugfs directory if it was created by this driver
	if (!xdev->conf.debugfs_dev_root)
		debugfs_remove_recursive(xdev->dbgfs_dev_root);
	xdev->dbgfs_dev_root = NULL;
	xdev->dbgfs_queues_root = NULL;
	xdev->dbgfs_intr_root = NULL;
}

#endif
