/* MD:
 * This file is part of the Xilinx DMA IP Core driver for Linux
 *
 * Copyright (c) 2016-present, Xilinx, Inc.
 * All rights reserved.
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
 */

#define pr_fmt(fmt)	KBUILD_MODNAME ":%s: " fmt, __func__

#include "libxdma_api.h"
#include "xdma_cdev.h"

#define write_register(v, mem, off) iowrite32(v, mem)

/* MD:*
 * copy_desc_data() - Copy descriptor data to user buffer
 *
 * @transfer: Pointer to the xdma_transfer structure
 * @buf: User buffer to copy data into
 * @buf_offset: Current offset in the user buffer
 * @buf_size: Total size of the user buffer
 *
 * This function copies descriptor data from the transfer structure to the
 * user buffer, ensuring that the buffer does not overflow.
 *
 * @return 0 on success, -EINVAL on error, -ENOMEM if buffer is too small
 */
static int copy_desc_data(struct xdma_transfer *transfer, char __user *buf,
		size_t *buf_offset, size_t buf_size)
{
	int i;
	int copy_err;
	int rc = 0;

	if (!buf) {
		pr_err("Invalid user buffer\n");
		return -EINVAL; // MD: Return error if user buffer is invalid
	}

	if (!buf_offset) {
		pr_err("Invalid user buffer offset\n");
		return -EINVAL; // MD: Return error if buffer offset is invalid
	}

	/* MD: Fill user buffer with descriptor data */
	for (i = 0; i < transfer->desc_num; i++) {
		if (*buf_offset + sizeof(struct xdma_desc) <= buf_size) {
			copy_err = copy_to_user(&buf[*buf_offset],
				transfer->desc_virt + i,
				sizeof(struct xdma_desc));

			if (copy_err) {
				dbg_sg("Copy to user buffer failed\n");
				*buf_offset = buf_size; // MD: Set offset to buffer size on error
				rc = -EINVAL; // MD: Return error if copy fails
			} else {
				*buf_offset += sizeof(struct xdma_desc); // MD: Increment buffer offset
			}
		} else {
			rc = -ENOMEM; // MD: Return error if buffer is too small
		}
	}

	return rc; // MD: Return result code
}

/* MD:*
 * char_bypass_read() - Read data from the bypass channel
 *
 * @file: File structure representing the device
 * @buf: User buffer to read data into
 * @count: Number of bytes to read
 * @pos: File position (unused)
 *
 * This function reads data from the bypass channel of the XDMA device into
 * the user buffer. It checks for valid buffer size and ensures that the
 * bypass BAR is present before proceeding with the read operation.
 *
 * @return Number of bytes read on success, negative error code on failure
 */
static ssize_t char_bypass_read(struct file *file, char __user *buf,
		size_t count, loff_t *pos)
{
	struct xdma_dev *xdev;
	struct xdma_engine *engine;
	struct xdma_cdev *xcdev = (struct xdma_cdev *)file->private_data;
	struct xdma_transfer *transfer;
	struct list_head *idx;
	size_t buf_offset = 0;
	int rc = 0;

	rc = xcdev_check(__func__, xcdev, 1);
	if (rc < 0)
		return rc; // MD: Return error if device check fails
	xdev = xcdev->xdev;
	engine = xcdev->engine;

	dbg_sg("In %s()\n", __func__); // MD: Debug statement for function entry

	if (count & 3) {
		dbg_sg("Buffer size must be a multiple of 4 bytes\n");
		return -EINVAL; // MD: Return error if buffer size is not a multiple of 4
	}

	if (!buf) {
		dbg_sg("Caught NULL pointer\n");
		return -EINVAL; // MD: Return error if buffer is NULL
	}

	if (xdev->bypass_bar_idx < 0) {
		dbg_sg("Bypass BAR not present - unsupported operation\n");
		return -ENODEV; // MD: Return error if bypass BAR is not present
	}

	spin_lock(&engine->lock); // MD: Lock the engine to ensure thread safety

	if (!list_empty(&engine->transfer_list)) {
		list_for_each(idx, &engine->transfer_list) {
			transfer = list_entry(idx, struct xdma_transfer, entry);

			rc = copy_desc_data(transfer, buf, &buf_offset, count);
		}
	}

	spin_unlock(&engine->lock); // MD: Unlock the engine

	if (rc < 0)
		return rc; // MD: Return error code if copy failed
	else
		return buf_offset; // MD: Return number of bytes read
}

static ssize_t char_bypass_write(struct file *file, const char __user *buf,
		size_t count, loff_t *pos)
{
	struct xdma_dev *xdev;
	struct xdma_engine *engine;
	struct xdma_cdev *xcdev = (struct xdma_cdev *)file->private_data;

	u32 desc_data;
	void __iomem *bypass_addr;
	size_t buf_offset = 0;
	int rc = 0;
	int copy_err;

	// MD: Check the validity of the xdma_cdev structure
	rc = xcdev_check(__func__, xcdev, 1);
	if (rc < 0)
		return rc;

	// MD: Retrieve the xdma device and engine from the character device
	xdev = xcdev->xdev;
	engine = xcdev->engine;

	// MD: Ensure the buffer size is a multiple of 4 bytes
	if (count & 3) {
		dbg_sg("Buffer size must be a multiple of 4 bytes\n");
		return -EINVAL;
	}

	// MD: Check for NULL buffer
	if (!buf) {
		dbg_sg("Caught NULL pointer\n");
		return -EINVAL;
	}

	// MD: Check if the bypass BAR is present
	if (xdev->bypass_bar_idx < 0) {
		dbg_sg("Bypass BAR not present - unsupported operation\n");
		return -ENODEV;
	}

	dbg_sg("In %s()\n", __func__);

	// MD: Lock the engine to ensure exclusive access
	spin_lock(&engine->lock);

	// MD: Calculate the bypass address
	bypass_addr = xdev->bar[xdev->bypass_bar_idx];
	bypass_addr = (void __iomem *)((u32 __iomem *)bypass_addr + engine->bypass_offset);

	// MD: Write data from the user buffer to the bypass BAR
	while (buf_offset < count) {
		copy_err = copy_from_user(&desc_data, &buf[buf_offset], sizeof(u32));
		if (!copy_err) {
			write_register(desc_data, bypass_addr, bypass_addr - engine->bypass_offset);
			buf_offset += sizeof(u32);
			rc = buf_offset; // MD: Update return code to reflect bytes written
		} else {
			dbg_sg("Error reading data from userspace buffer\n");
			rc = -EINVAL;
			break;
		}
	}

	// MD: Unlock the engine
	spin_unlock(&engine->lock);

	return rc; // MD: Return the number of bytes written or an error code
}

/* MD:
 * Character device file operations for bypass operation
 */
static const struct file_operations bypass_fops = {
	.owner = THIS_MODULE,
	.open = char_open,
	.release = char_close,
	.read = char_bypass_read,
	.write = char_bypass_write,
	.mmap = bridge_mmap,
};

// MD: Initialize the character device with the bypass operations
void cdev_bypass_init(struct xdma_cdev *xcdev)
{
	cdev_init(&xcdev->cdev, &bypass_fops);
}
