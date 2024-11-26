/*
 * This file is part of the Xilinx DMA IP Core driver for Linux
 *
 * Copyright (c) 2017-2022, Xilinx, Inc. All rights reserved.
 * Copyright (c) 2022-2023, Advanced Micro Devices, Inc. All rights reserved.
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
 * This source code is modified to include debug prints and detailed comments
 */


/* Define format for debug prints - includes function name for better tracking */
#define pr_fmt(fmt)	KBUILD_MODNAME ":%s: " fmt, __func__

/* Include required headers */

/* Include necessary header files for character device implementation */
#include "cdev.h"

/* System header files required for memory management and device operations */
#include <asm/cacheflush.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/aio.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/kthread.h>
#include <linux/version.h>

/* Conditional include for kernel version compatibility */
#if KERNEL_VERSION(3, 16, 0) <= LINUX_VERSION_CODE
#include <linux/uio.h>
#endif

/* Include QDMA specific headers */
#include "qdma_mod.h"
#include "libqdma/xdev.h"

/* Debug print macro */
#define QDMA_CDEV_DEBUG(fmt, ...) \
    pr_debug("QDMA_CDEV: %s:%d: " fmt, __func__, __LINE__, ##__VA_ARGS__)

/*
 * @struct - xlnx_phy_dev
 * @brief	xilinx board device data members
 */

/*
 * Structure representing physical device information
 * Used to maintain list of QDMA devices in the system
 */
struct xlnx_phy_dev {
	struct list_head list_head;			/**< List management for device tracking */
	unsigned int major;					/**< major number per board */
	unsigned int device_bus;			/**< PCIe device bus number per board */
	unsigned int dma_device_index;		/* Index of DMA device */
};

/* Global list head for physical devices */
static LIST_HEAD(xlnx_phy_dev_list);

/* Mutex to protect physical device list access */
static DEFINE_MUTEX(xlnx_phy_dev_mutex);


/*
 * Debug print to track list initialization
 */
static inline void debug_print_list_status(void)
{
    pr_debug("Xilinx physical device list initialized\n");
}

/* Initialize the list and mutex at module load time */
static void __init init_xlnx_lists(void)
{
    debug_print_list_status();
}

/* 
 * Structure to handle asynchronous I/O operations
 * This structure manages the state and results of asynchronous I/O requests
 */
struct cdev_async_io {
    ssize_t res2;               /* Result storage for async operation */
    unsigned long req_count;    /* Total number of requests queued */
    unsigned long cmpl_count;   /* Number of completed requests */
    unsigned long err_cnt;      /* Count of errors encountered */
    struct qdma_io_cb *qiocb;   /* I/O control block for QDMA operations */
    struct qdma_request **reqv; /* Array of QDMA request pointers */
    struct kiocb *iocb;         /* Kernel I/O control block */
    struct work_struct wrk_itm; /* Work item for deferred processing */
};

/* 
 * Enumeration defining QDMA character device IOCTL commands
 * Used for controlling device behavior through IOCTL interface
 */
enum qdma_cdev_ioctl_cmd {
    QDMA_CDEV_IOCTL_NO_MEMCPY,  /* Command to disable memory copy operations */
    QDMA_CDEV_IOCTL_CMDS        /* Total number of supported IOCTL commands */
};


/* Global variables for device management */
static struct class *qdma_class;         /* Device class for QDMA devices */
static struct kmem_cache *cdev_cache;    /* Kernel memory cache for async I/O structures */

/*
 * cdev_gen_read_write - Common handler for both read and write operations
 * @file: File pointer
 * @buf: User buffer
 * @count: Number of bytes to transfer
 * @pos: File position
 * @write: Boolean indicating write (true) or read (false) operation
 *
 * This function handles both read and write operations for the character device.
 * It maps user buffers, prepares DMA requests and initiates transfers.
 *
 * Returns: Number of bytes transferred on success, negative error code on failure
 */

/* Function prototypes for core operations */
static ssize_t cdev_gen_read_write(struct file *file, char __user *buf,
        size_t count, loff_t *pos, bool write); 					/* Generic read/write handler */
static void unmap_user_buf(struct qdma_io_cb *iocb, bool write); 	/* Buffer unmapping utility */
static inline void iocb_release(struct qdma_io_cb *iocb); 			/* I/O control block cleanup */

/*
 * xlnx_phy_dev_list_remove - Remove a physical device from the global list
 * @phy_dev: Physical device to remove
 */

/* 
 * Helper function to remove a physical device from the global device list
 * Ensures thread-safe removal using mutex protection
 */
static inline void xlnx_phy_dev_list_remove(struct xlnx_phy_dev *phy_dev)
{
	/* Check for null pointer to prevent crashes */
	if (!phy_dev)
		return;

    /* Lock the mutex before modifying the global list */
    mutex_lock(&xlnx_phy_dev_mutex);
    list_del(&phy_dev->list_head);  	/* Remove device from linked list */
    mutex_unlock(&xlnx_phy_dev_mutex);  /* Release the mutex 
}

static inline void xlnx_phy_dev_list_add(struct xlnx_phy_dev *phy_dev)
{
	if (!phy_dev)
		return;

	mutex_lock(&xlnx_phy_dev_mutex);
	list_add_tail(&phy_dev->list_head, &xlnx_phy_dev_list);
	mutex_unlock(&xlnx_phy_dev_mutex);
}

/* 
 * Handler function called when a DMA request is completed
 * @req: Pointer to the completed DMA request
 * @bytes_done: Number of bytes transferred
 * @err: Error code (0 if successful, negative if error occurred)
 * Returns: 0 on success, negative error code on failure
 */
static int qdma_req_completed(struct qdma_request *req,
		       unsigned int bytes_done, int err)
{
	/* Get the I/O control block associated with this request */
	struct qdma_io_cb *qiocb = container_of(req,
						struct qdma_io_cb,
						req);
	struct cdev_async_io *caio = NULL;
	bool free_caio = false;
	ssize_t res, res2;

	/* Debug print for tracking request completion */
    pr_debug("QDMA: Request completed - bytes_done=%u, err=%d\n", 
             bytes_done, err);

	/* Validate the I/O control block structure */
	if (qiocb) {
		caio = (struct cdev_async_io *)qiocb->private;
	} else {
		pr_err("Invalid Data structure. Probable memory corruption");
		return -EINVAL;
	}

	/* Validate the async I/O structure */
	if (!caio) {
		pr_err("Invalid Data structure. Probable memory corruption");
		return -EINVAL;
	}

    /* Debug print for tracking request status */
    pr_debug("QDMA: Processing completion - req_count=%lu, cmpl_count=%lu\n", 
             caio->req_count, caio->cmpl_count);

	/* Cleanup and process completion */
	/* Unmap user buffer and release I/O control block */
	unmap_user_buf(qiocb, req->write);
	iocb_release(qiocb);

	/* Update completion status and error tracking */
	caio->res2 |= (err < 0) ? err : 0;
	if (caio->res2)
		caio->err_cnt++;

	/* Increment completion counter */
	caio->cmpl_count++;

	/* Handle final completion */
	/* Check if all requests in batch are completed */
	if (caio->cmpl_count == caio->req_count) {
		QDMA_CDEV_DEBUG("All requests completed: total=%lu, errors=%lu\n", 
                        caio->cmpl_count, caio->err_cnt);
		/* Calculate final results */
		res = caio->cmpl_count - caio->err_cnt;
		res2 = caio->res2;

		pr_debug("QDMA: All requests completed - success=%ld, errors=%lu\n", 
                res, caio->err_cnt);

/* Handle completion based on kernel version */
#ifdef RHEL_RELEASE_VERSION
#if RHEL_RELEASE_VERSION(9, 0) < RHEL_RELEASE_CODE
		caio->iocb->ki_complete(caio->iocb, res);
#elif RHEL_RELEASE_VERSION(8, 0) < RHEL_RELEASE_CODE
		caio->iocb->ki_complete(caio->iocb, res, res2);
#else
		aio_complete(caio->iocb, res, res2);
#endif
#else

/* Complete the async I/O operation */
#if KERNEL_VERSION(5, 16, 0) <= LINUX_VERSION_CODE
		caio->iocb->ki_complete(caio->iocb, res);
#elif KERNEL_VERSION(4, 1, 0) <= LINUX_VERSION_CODE
		caio->iocb->ki_complete(caio->iocb, res, res2);
#else
		aio_complete(caio->iocb, res, res2);
#endif
#endif
		/* Free the I/O control block and mark for cache free */
		kfree(caio->qiocb);
		free_caio = true;
	}

	/* Free the cache if all operations are complete */
	if (free_caio)
		kmem_cache_free(cdev_cache, caio);

	return 0;
}

/*
 * @brief File operations structure for QDMA character device
 * This structure defines the supported file operations for the QDMA character device,
 * including standard operations like open, close, read, write, and ioctl.
 */
static const struct file_operations cdev_gen_fops = {
    .owner = THIS_MODULE,
    .open = cdev_gen_open,
    .release = cdev_gen_close,
    .write = cdev_gen_write,
#if KERNEL_VERSION(3, 16, 0) <= LINUX_VERSION_CODE
    /* For newer kernels (3.16+), use the newer iterator-based write operation */
    .write_iter = cdev_write_iter,
#else
    /* For older kernels, use the traditional async I/O write */
    .aio_write = cdev_aio_write,
#endif
    .read = cdev_gen_read,
#if KERNEL_VERSION(3, 16, 0) <= LINUX_VERSION_CODE
    /* For newer kernels (3.16+), use the newer iterator-based read operation */
    .read_iter = cdev_read_iter,
#else
    /* For older kernels, use the traditional async I/O read */
    .aio_read = cdev_aio_read,
#endif
    .unlocked_ioctl = cdev_gen_ioctl,
    .llseek = cdev_gen_llseek,
};

/*
 * @brief Opens the QDMA character device
 * This function is called when a process opens the character device.
 * It initializes the private data and calls any extra open handlers if defined.
 *
 * @param inode Pointer to the inode structure
 * @param file Pointer to the file structure
 * @return 0 on success, negative error code on failure
 */

static int cdev_gen_open(struct inode *inode, struct file *file)
{
	struct qdma_cdev *xcdev = container_of(inode->i_cdev, struct qdma_cdev,
						cdev);
	file->private_data = xcdev;

	if (xcdev->fp_open_extra)
		return xcdev->fp_open_extra(xcdev);

	return 0;
}

/*
 * @brief Closes the QDMA character device
 * This function is called when a process closes the character device.
 * It performs cleanup and calls any extra close handlers if defined.
 *
 * @param inode Pointer to the inode structure
 * @param file Pointer to the file structure
 * @return 0 on success, negative error code on failure
 */
static int cdev_gen_close(struct inode *inode, struct file *file)
{
	struct qdma_cdev *xcdev = (struct qdma_cdev *)file->private_data;

	/* Call device-specific close handler if exists */
	if (xcdev && xcdev->fp_close_extra)
		return xcdev->fp_close_extra(xcdev);

	return 0;
}

/*
 * @brief Implements seek functionality for the QDMA character device
 * Handles different seek operations (SEEK_SET, SEEK_CUR, SEEK_END) and
 * updates the file position accordingly.
 *
 * @param file Pointer to the file structure
 * @param off Offset for the seek operation
 * @param whence Type of seek operation (0: SET, 1: CUR, 2: END)
 * @return New file position on success, negative error code on failure
 */
static loff_t cdev_gen_llseek(struct file *file, loff_t off, int whence)
{
	struct qdma_cdev *xcdev = (struct qdma_cdev *)file->private_data;

	loff_t newpos = 0;

    if (!xcdev) {
        pr_err("QDMA: NULL device in llseek operation\n");
        return -EINVAL;
    }

    pr_debug("QDMA: Seek request for %s: offset=%lld, whence=%d\n", 
             xcdev->name, off, whence);

    switch (whence) {
    case 0: /* SEEK_SET */
        newpos = off;
        pr_debug("QDMA: SEEK_SET to position %lld\n", newpos);
        break;
    case 1: /* SEEK_CUR */
        newpos = file->f_pos + off;
        pr_debug("QDMA: SEEK_CUR to position %lld\n", newpos);
        break;
    case 2: /* SEEK_END, @TODO should work from end of address space */
        newpos = UINT_MAX + off;
        pr_debug("QDMA: SEEK_END to position %lld\n", newpos);
        break;
	default: /* can't happen */
		pr_err("QDMA: Invalid seek operation: whence=%d\n", whence);
		return -EINVAL;
	}

	if (newpos < 0) {
        pr_err("QDMA: Invalid seek position: %lld\n", newpos);
        return -EINVAL;
    }

    file->f_pos = newpos;
    pr_debug("%s: New position=%lld\n", xcdev->name, (signed long long)newpos);

    return newpos;
}

/*
 * @brief IOCTL handler for character device operations
 * This function processes IOCTL commands for the QDMA character device
 *
 * @param file Pointer to the file structure
 * @param cmd IOCTL command to be processed
 * @param arg User-space argument for the IOCTL command
 * @return 0 on success, negative error code on failure
 */
static long cdev_gen_ioctl(struct file *file, unsigned int cmd,
			unsigned long arg)
{
	struct qdma_cdev *xcdev = (struct qdma_cdev *)file->private_data;

	pr_debug("%s: processing ioctl cmd=%u, arg=%lu\n", 
		 xcdev->name, cmd, arg);

	switch (cmd) {
	case QDMA_CDEV_IOCTL_NO_MEMCPY:
		/* Set the no_memcpy flag based on user input */
		if (get_user(xcdev->no_memcpy, (unsigned char *)arg)) {
			pr_err("%s: failed to get user data for NO_MEMCPY\n", 
				xcdev->name);
			return -EFAULT;
		}
		pr_debug("%s: NO_MEMCPY flag set to %d\n", 
			 xcdev->name, xcdev->no_memcpy);
		return 0;
	default:
		break;
	}

	/* If device has extra IOCTL handling, call it */
	if (xcdev->fp_ioctl_extra) {
		pr_debug("%s: forwarding to extra ioctl handler\n", xcdev->name);
		return xcdev->fp_ioctl_extra(xcdev, cmd, arg);
	}

	pr_err("%s: unsupported ioctl command %u\n", xcdev->name, cmd);
	return -EINVAL;
}

/*
 * @brief Release resources associated with an I/O control block
 * Frees memory allocated for scatter-gather list and resets buffer pointers
 *
 * @param iocb Pointer to the I/O control block to be released
 */
static inline void iocb_release(struct qdma_io_cb *iocb)
{
	pr_debug("Releasing IOCB resources\n");

	if (iocb->pages) {
		pr_debug("Clearing pages pointer\n");
		iocb->pages = NULL;
	}
	
	if (iocb->sgl) {
		pr_debug("Freeing scatter-gather list\n");
		kfree(iocb->sgl);
		iocb->sgl = NULL;
	}
	
	iocb->buf = NULL;
}

/*
 * @brief Unmap user buffer pages and handle cleanup
 * Releases mapped pages and handles dirty page marking for read operations
 *
 * @param iocb Pointer to the I/O control block
 * @param write Boolean indicating if operation was write (true) or read (false)
 */
static void unmap_user_buf(struct qdma_io_cb *iocb, bool write)
{
	int i;

	if (!iocb->pages || !iocb->pages_nr) {
		pr_debug("No pages to unmap\n");
		return;
	}

	pr_debug("Unmapping %u user pages, write=%d\n", iocb->pages_nr, write);

	for (i = 0; i < iocb->pages_nr; i++) {
		if (iocb->pages[i]) {
			if (!write) {
				pr_debug("Marking page %d as dirty\n", i);
				set_page_dirty(iocb->pages[i]);
			}
			put_page(iocb->pages[i]);
		} else {
			pr_debug("Encountered NULL page at index %d\n", i);
			break;
		}
	}

	if (i != iocb->pages_nr) {
		pr_err("Page count mismatch during unmap: mapped=%d, expected=%u\n", 
			i, iocb->pages_nr);
	}

	iocb->pages_nr = 0;
}

/*
 * @brief Map a user buffer to scatter-gather list for DMA operations
 * Pins user pages in memory and creates scatter-gather list for DMA transfer
 *
 * @param iocb Pointer to the I/O control block
 * @param write Boolean indicating if operation is write (true) or read (false)
 * @return 0 on success, negative error code on failure
 */
static int map_user_buf_to_sgl(struct qdma_io_cb *iocb, bool write)
{
	unsigned long len = iocb->len;
	char *buf = iocb->buf;
	struct qdma_sw_sg *sg;
	unsigned int pg_off = offset_in_page(buf);
	unsigned int pages_nr = (len + pg_off + PAGE_SIZE - 1) >> PAGE_SHIFT;
	int i;
	int rv;

	pr_debug("QDMA: Mapping user buffer - len=%lu, pages_nr=%u, write=%d\n", 
             len, pages_nr, write);

    /* Handle special case for zero length */
    if (len == 0)
        pages_nr = 1;
    if (pages_nr == 0)
        return -EINVAL;

    /* Initialize the I/O control block */
    iocb->pages_nr = 0;

    /* Allocate memory for scatter-gather list and page pointers */
    sg = kmalloc(pages_nr * (sizeof(struct qdma_sw_sg) +
            sizeof(struct page *)), GFP_KERNEL);
    if (!sg) {
        pr_err("QDMA: Failed to allocate SG list for %u pages\n", pages_nr);
        return -ENOMEM;
    }

    /* Clear the allocated memory */
    memset(sg, 0, pages_nr * (sizeof(struct qdma_sw_sg) +
            sizeof(struct page *)));
    iocb->sgl = sg;

    /* Setup page pointer array after scatter-gather list */
    iocb->pages = (struct page **)(sg + pages_nr);

    /* Pin down user pages in memory */
    rv = get_user_pages_fast((unsigned long)buf, pages_nr, 1/* write */,
                iocb->pages);
    
    /* Handle page pinning failures */
    if (rv < 0) {
        pr_err("QDMA: Failed to pin down %u user pages, error %d\n",
            pages_nr, rv);
        goto err_out;
    }
    if (rv != pages_nr) {
        pr_err("QDMA: Only pinned %d of %u requested pages\n",
            rv, pages_nr);
        iocb->pages_nr = rv;
        rv = -EFAULT;
        goto err_out;
    }

    /* Verify no duplicate pages were pinned */
    for (i = 1; i < pages_nr; i++) {
        if (iocb->pages[i - 1] == iocb->pages[i]) {
            pr_err("QDMA: Detected duplicate pages at indices %d and %d\n",
                i - 1, i);
            iocb->pages_nr = pages_nr;
            rv = -EFAULT;
            goto err_out;
        }
    }

    /* Build scatter-gather list from pinned pages */
    sg = iocb->sgl;
    for (i = 0; i < pages_nr; i++, sg++) {
        unsigned int offset = offset_in_page(buf);
        unsigned int nbytes = min_t(unsigned int, PAGE_SIZE - offset,
                        len);
        struct page *pg = iocb->pages[i];

        /* Ensure cache coherency */
        flush_dcache_page(pg);

        /* Setup scatter-gather entry */
        sg->next = sg + 1;
        sg->pg = pg;
        sg->offset = offset;
        sg->len = nbytes;
        sg->dma_addr = 0UL;

        pr_debug("QDMA: SG entry %d - offset=%u, len=%u\n", 
                i, offset, nbytes);

        buf += nbytes;
        len -= nbytes;
    }

    /* Terminate the scatter-gather list */
    iocb->sgl[pages_nr - 1].next = NULL;
    iocb->pages_nr = pages_nr;

    pr_debug("QDMA: Successfully mapped %u pages to SG list\n", pages_nr);
    return 0;

err_out:
    unmap_user_buf(iocb, write);
    iocb_release(iocb);
    return rv;
}

/*
 * @brief Generic read/write handler for character device operations
 *
 * @param file File pointer associated with the device
 * @param buf User space buffer for data transfer
 * @param count Number of bytes to transfer
 * @param pos File position pointer
 * @param write Boolean flag: true for write operation, false for read
 * @return ssize_t Number of bytes transferred or negative error code
 */
static ssize_t cdev_gen_read_write(struct file *file, char __user *buf,
		size_t count, loff_t *pos, bool write)
{
	struct qdma_cdev *xcdev = (struct qdma_cdev *)file->private_data;
	struct qdma_io_cb iocb;
	struct qdma_request *req = &iocb.req;
	ssize_t res = 0;
	int rv;
	unsigned long qhndl;

	/* Validate device context */
    if (!xcdev) {
        pr_err("QDMA: Invalid device context - file=0x%p, buf=0x%p, count=%llu, pos=%llu, write=%d\n",
            file, buf, (u64)count, (u64)*pos, write);
        return -EINVAL;
    }

    /* Check if read/write handler is available */
    if (!xcdev->fp_rw) {
        pr_err("QDMA: %s - No read/write handler available\n", xcdev->name);
        return -EINVAL;
    }

    /* Select appropriate queue handle based on operation */
    qhndl = write ? xcdev->h2c_qhndl : xcdev->c2h_qhndl;

    pr_debug("QDMA: %s - Starting %s operation: qhandle=0x%lx, buf=0x%p, count=%llu, pos=%llu\n",
        xcdev->name, write ? "write" : "read", qhndl, buf, (u64)count, (u64)*pos);

    /* Initialize I/O control block */
    memset(&iocb, 0, sizeof(struct qdma_io_cb));
    iocb.buf = buf;
    iocb.len = count;

    /* Map user buffer to scatter-gather list */
    rv = map_user_buf_to_sgl(&iocb, write);
    if (rv < 0) {
        pr_err("QDMA: Failed to map user buffer to SGL, err=%d\n", rv);
        return rv;
    }

	/* Setup DMA request parameters */
    req->sgcnt = iocb.pages_nr;
    req->sgl = iocb.sgl;
    req->write = write ? 1 : 0;
    req->dma_mapped = 0;
    req->udd_len = 0;
    req->ep_addr = (u64)*pos;
    req->count = count;
    req->timeout_ms = 10 * 1000;    /* 10 seconds timeout */
    req->fp_done = NULL;            /* blocking operation */
    req->h2c_eot = 1;              /* End of transfer marker */

    /* Submit the DMA request */
    pr_debug("QDMA: %s - Submitting DMA request: sgcnt=%u, count=%llu\n",
        xcdev->name, req->sgcnt, (u64)req->count);
    
    res = xcdev->fp_rw(xcdev->xcb->xpdev->dev_hndl, qhndl, req);

    /* Cleanup resources */
    unmap_user_buf(&iocb, write);
    iocb_release(&iocb);

    if (res < 0) {
        pr_err("QDMA: %s - DMA operation failed with error %ld\n", 
            xcdev->name, res);
    } else {
        pr_debug("QDMA: %s - DMA operation completed successfully, transferred %ld bytes\n",
            xcdev->name, res);
    }

    return res;
}

/*
 * @brief Write operation handler for character device
 */
static ssize_t cdev_gen_write(struct file *file, const char __user *buf,
                size_t count, loff_t *pos)
{
    pr_debug("QDMA: Initiating write operation\n");
    return cdev_gen_read_write(file, (char *)buf, count, pos, 1);
}

/*
 * @brief Read operation handler for character device
 */
static ssize_t cdev_gen_read(struct file *file, char __user *buf,
                size_t count, loff_t *pos)
{
    pr_debug("QDMA: Initiating read operation\n");
    return cdev_gen_read_write(file, (char *)buf, count, pos, 0);
}


/*
 * cdev_gen_read - Read from character device
 * @file: File structure
 * @buf: User buffer
 * @count: Number of bytes to read
 * @pos: File position
 *
 * Handles read requests from user space
 */
static ssize_t cdev_gen_read(struct file *file, char __user *buf,
				size_t count, loff_t *pos)
{
	return cdev_gen_read_write(file, (char *)buf, count, pos, 0);
}

/*
 * @brief Asynchronous write operation handler
 *
 * @param iocb Kernel I/O control block
 * @param io Array of I/O vectors for scatter/gather operations
 * @param count Number of I/O vectors
 * @param pos File position
 * @return ssize_t Number of bytes written or negative error code
 */
static ssize_t cdev_aio_write(struct kiocb *iocb, const struct iovec *io,
				unsigned long count, loff_t pos)
{
	struct qdma_cdev *xcdev =
		(struct qdma_cdev *)iocb->ki_filp->private_data;
	struct cdev_async_io *caio;
	int rv = 0;
	unsigned long i;
	unsigned long qhndl;

	/* Validate device context */
    if (!xcdev) {
        pr_err("QDMA: Invalid device context - count=%llu, pos=%llu\n",
            (u64)count, (u64)pos);
        return -EINVAL;
    }

    /* Verify read/write handler availability */
    if (!xcdev->fp_rw) {
        pr_err("QDMA: %s - No async write handler assigned\n", xcdev->name);
        return -EINVAL;
    }

    pr_debug("QDMA: %s - Starting async write operation: vectors=%lu, pos=%llu\n",
        xcdev->name, count, (u64)pos);

    /* Allocate async I/O context */
    caio = kmem_cache_alloc(cdev_cache, GFP_KERNEL);
    if (!caio) {
        pr_err("QDMA: Failed to allocate async I/O context\n");
        return -ENOMEM;
    }

    /* Initialize async I/O context */
    memset(caio, 0, sizeof(struct cdev_async_io));
    caio->qiocb = kzalloc(count * (sizeof(struct qdma_io_cb) +
            sizeof(_request *)), GFP_KERNEL);
    if (!caio->qiocb) {
        pr_err("QDMA: Failed to allocate I/O control blocks\n");
        kmem_cache_free(cdev_cache, caio);
        return -ENOMEM;
    }

    /* Setup request vectors */
    caio->reqv = (struct qdma_request **)(caio->qiocb + count);
    for (i = 0; i < count; i++) {
        pr_debug("QDMA: Setting up request vector %lu\n", i);
        
        caio->qiocb[i].private = caio;
        caio->reqv[i] = &(caio->qiocb[i].req);
        caio->qiocb[i].buf = io[i].iov_base;
        caio->qiocb[i].len = io[i].iov_len;
        
        rv = map_user_buf_to_sgl(&(caio->qiocb[i]), true);
        if (rv < 0) {
            pr_err("QDMA: Failed to map user buffer for vector %lu\n", i);
            break;
        }

        /* Configure DMA request parameters */
        caio->reqv[i]->write = 1;
        caio->reqv[i]->sgcnt = caio->qiocb[i].pages_nr;
        caio->reqv[i]->sgl = caio->qiocb[i].sgl;
        caio->reqv[i]->dma_mapped = false;
        caio->reqv[i]->udd_len = 0;
        caio->reqv[i]->ep_addr = (u64)pos;
        pos += io[i].iov_len;
        caio->reqv[i]->no_memcpy = xcdev->no_memcpy ? 1 : 0;
        caio->reqv[i]->count = io->iov_len;
        caio->reqv[i]->timeout_ms = 10 * 1000;
        caio->reqv[i]->fp_done = qdma_req_completed;
    }

    /* Process the requests if any were successfully setup */
    if (i > 0) {
        iocb->private = caio;
        caio->iocb = iocb;
        caio->req_count = i;
        qhndl = xcdev->h2c_qhndl;
        
        pr_debug("QDMA: %s - Submitting %lu async write requests\n",
            xcdev->name, caio->req_count);
            
        rv = xcdev->fp_aiorw(xcdev->xcb->xpdev->dev_hndl, qhndl,
                     caio->req_count, caio->reqv);
        if (rv >= 0) {
            rv = -EIOCBQUEUED;
            pr_debug("QDMA: Async write requests queued successfully\n");
        } else {
            pr_err("QDMA: Failed to queue async write requests\n");
        }
    } else {
        pr_err("QDMA: Setup failed for all vectors, rv=%d, count=%lu\n",
            rv, caio->req_count);
        kfree(caio->qiocb);
        kmem_cache_free(cdev_cache, caio);
    }

    return rv;
}

/*
 * @brief Asynchronous read operation handler
 *
 * @param iocb Kernel I/O control block
 * @param io Array of I/O vectors for scatter/gather operations
 * @param count Number of I/O vectors
 * @param pos File position
 * @return ssize_t Number of bytes read or negative error code
 */
static ssize_t cdev_aio_read(struct kiocb *iocb, const struct iovec *io,
						unsigned long count, loff_t pos)
{
	struct qdma_cdev *xcdev =
		(struct qdma_cdev *)iocb->ki_filp->private_data;
	struct cdev_async_io *caio;
	int rv = 0;
	unsigned long i;
	unsigned long qhndl;

	/* Validate device context */
    if (!xcdev) {
        pr_err("ERROR: file 0x%p, xcdev NULL, count=%llu, pos=%llu\n",
                iocb->ki_filp, (u64)count, (u64)pos);
        return -EINVAL;
    }

    /* Check if read/write handler is assigned */
    if (!xcdev->fp_rw) {
        pr_err("ERROR: No read/write handler assigned for device %s\n", 
               xcdev->name);
        return -EINVAL;
    }

    pr_debug("Starting async read: device=%s, count=%lu, pos=%lld\n", 
             xcdev->name, count, pos);

    /* Allocate async I/O control structure from cache */
    caio = kmem_cache_alloc(cdev_cache, GFP_KERNEL);
    if (!caio) {
        pr_err("ERROR: Failed to allocate async I/O control structure\n");
        return -ENOMEM;
    }
    memset(caio, 0, sizeof(struct cdev_async_io));

    /* Allocate I/O control blocks and request vector */
    caio->qiocb = kzalloc(count * (sizeof(struct qdma_io_cb) +
            sizeof(struct qdma_request *)), GFP_KERNEL);
    if (!caio->qiocb) {
        pr_err("ERROR: Failed to allocate I/O control blocks\n");
        kmem_cache_free(cdev_cache, caio);
        return -ENOMEM;
    }

    /* Setup request vector after I/O control blocks */
    caio->reqv = (struct qdma_request **)(caio->qiocb + count);

    /* Initialize each I/O request */
    for (i = 0; i < count; i++) {
        pr_debug("Setting up request %lu of %lu\n", i+1, count);
        
        /* Setup I/O control block */
        caio->qiocb[i].private = caio;
        caio->reqv[i] = &(caio->qiocb[i].req);
        caio->qiocb[i].buf = io[i].iov_base;
        caio->qiocb[i].len = io[i].iov_len;

        /* Map user buffer to scatter-gather list */
        rv = map_user_buf_to_sgl(&(caio->qiocb[i]), false);
        if (rv < 0) {
            pr_err("ERROR: Failed to map user buffer for request %lu\n", i);
            break;
        }

        /* Configure DMA request parameters */
        caio->reqv[i]->write = 0;  /* Read operation */
        caio->reqv[i]->sgcnt = caio->qiocb[i].pages_nr;
        caio->reqv[i]->sgl = caio->qiocb[i].sgl;
        caio->reqv[i]->dma_mapped = false;
        caio->reqv[i]->udd_len = 0;
        caio->reqv[i]->ep_addr = (u64)pos;
        pos += io[i].iov_len;
        caio->reqv[i]->no_memcpy = xcdev->no_memcpy ? 1 : 0;
        caio->reqv[i]->count = io->iov_len;
        caio->reqv[i]->timeout_ms = 10 * 1000;  /* 10 seconds timeout */
        caio->reqv[i]->fp_done = qdma_req_completed;
    }

    /* Submit requests if any were successfully prepared */
    if (i > 0) {
        pr_debug("Submitting %lu async read requests\n", i);
        iocb->private = caio;
        caio->iocb = iocb;
        caio->req_count = i;
        qhndl = xcdev->c2h_qhndl;
        
        /* Submit batch request to QDMA */
        rv = xcdev->fp_aiorw(xcdev->xcb->xpdev->dev_hndl, qhndl,
                     caio->req_count, caio->reqv);
        if (rv >= 0) {
            pr_debug("Successfully queued async read requests\n");
            rv = -EIOCBQUEUED;
        } else {
            pr_err("ERROR: Failed to submit async read requests, rv=%d\n", rv);
        }
    } else {
        pr_err("ERROR: Failed to prepare requests, rv=%d, count=%lu\n", 
               rv, caio->req_count);
        kfree(caio->qiocb);
        kmem_cache_free(cdev_cache, caio);
    }

    return rv;
}

/**
 * Kernel version specific implementations for read/write operations
 * For kernel versions 3.16.0 and above, we use the _iter variants
 * Otherwise fall back to aio_ variants
 */
#if KERNEL_VERSION(3, 16, 0) <= LINUX_VERSION_CODE
/*
 * @brief Write implementation for newer kernel versions using iov_iter
 * @param iocb Kernel I/O control block
 * @param io I/O vector iterator
 * @return Number of bytes written or negative error code
 */
static ssize_t cdev_write_iter(struct kiocb *iocb, struct iov_iter *io)
{
	pr_debug("QDMA: Starting write_iter operation, segments=%d\n", io->nr_segs);
    return cdev_aio_write(iocb, io->iov, io->nr_segs, iocb->ki_pos);
}

/*
 * @brief Read implementation for newer kernel versions using iov_iter
 * @param iocb Kernel I/O control block
 * @param io I/O vector iterator
 * @return Number of bytes read or negative error code
 */
static ssize_t cdev_read_iter(struct kiocb *iocb, struct iov_iter *io)
{
	pr_debug("QDMA: Starting read_iter operation, segments=%d\n", io->nr_segs);
    return cdev_aio_read(iocb, io->iov, io->nr_segs, iocb->ki_pos);
}
#endif

/*
 * @brief File operations structure for the character device
 * This structure defines all the supported operations on the character device
 */
static const struct file_operations cdev_gen_fops = {
    .owner = THIS_MODULE,
    .open = cdev_gen_open,
    .release = cdev_gen_close,
    .write = cdev_gen_write,
#if KERNEL_VERSION(3, 16, 0) <= LINUX_VERSION_CODE
    .write_iter = cdev_write_iter,    /* Modern kernel write operation */
#else
    .aio_write = cdev_aio_write,      /* Legacy kernel write operation */
#endif
    .read = cdev_gen_read,
#if KERNEL_VERSION(3, 16, 0) <= LINUX_VERSION_CODE
    .read_iter = cdev_read_iter,      /* Modern kernel read operation */
#else
    .aio_read = cdev_aio_read,        /* Legacy kernel read operation */
#endif
    .unlocked_ioctl = cdev_gen_ioctl,
    .llseek = cdev_gen_llseek,
};

/*
 * @brief Destroys a QDMA character device
 * This function cleans up and destroys a QDMA character device,
 * including its system device node and cdev structure
 *
 * @param xcdev Pointer to the QDMA character device to destroy
 */
void qdma_cdev_destroy(struct qdma_cdev *xcdev)
{
	if (!xcdev) {
        pr_err("QDMA: xcdev is NULL, cannot destroy device\n");
        return;
    }

    pr_debug("QDMA: Destroying character device %s (minor=%d)\n", 
             xcdev->name, xcdev->minor);

    /* Destroy the device node in sysfs if it exists */
    if (xcdev->sys_device) {
        pr_debug("QDMA: Destroying system device node for %s\n", xcdev->name);
        device_destroy(qdma_class, xcdev->cdevno);
    }

    /* Remove the character device */
    pr_debug("QDMA: Removing cdev structure for %s\n", xcdev->name);
    cdev_del(&xcdev->cdev);

    /* Free the device structure */
    kfree(xcdev);
    pr_debug("QDMA: Character device destruction complete\n");
}

/*
 * @brief Creates a new QDMA character device
 * This function initializes and creates a new QDMA character device,
 * setting up the necessary structures and registering it with the system
 *
 * @param xcb Pointer to the QDMA character device control block
 * @param pdev Pointer to the PCI device
 * @param qconf Pointer to the queue configuration
 * @param minor Minor number for the device
 * @param qhndl Queue handle
 * @param xcdev_pp Pointer to store the created device
 * @param ebuf Error buffer for storing error messages
 * @param ebuflen Length of the error buffer
 * @return 0 on success, negative error code on failure
 */
int qdma_cdev_create(struct qdma_cdev_cb *xcb, struct pci_dev *pdev,
			struct qdma_queue_conf *qconf, unsigned int minor,
			unsigned long qhndl, struct qdma_cdev **xcdev_pp,
			char *ebuf, int ebuflen)
{
	struct qdma_cdev *xcdev;
    int rv;
    unsigned long *priv_data;

    pr_debug("QDMA: Creating character device for queue %s\n", qconf->name);

    /* Allocate memory for the character device structure */
    xcdev = kzalloc(sizeof(struct qdma_cdev) + strlen(qconf->name) + 1,
            GFP_KERNEL);
    if (!xcdev) {
        pr_err("QDMA: Failed to allocate memory for cdev %s (%lu bytes)\n",
               qconf->name, sizeof(struct qdma_cdev));
        if (ebuf && ebuflen) {
            rv = snprintf(ebuf, ebuflen,
                "Failed to allocate cdev %s (%lu bytes)\n",
                qconf->name, sizeof(struct qdma_cdev));
            ebuf[rv] = '\0';
        }
        return -ENOMEM;
    }

    /* Initialize the character device */
    xcdev->cdev.owner = THIS_MODULE;
    xcdev->xcb = xcb;

    /* Set up queue handle based on queue type */
    priv_data = (qconf->q_type == Q_C2H) ?
            &xcdev->c2h_qhndl : &xcdev->h2c_qhndl;
    *priv_data = qhndl;
    xcdev->dir_init = (1 << qconf->q_type);
    strcpy(xcdev->name, qconf->name);

    /* Validate and set up minor number */
    xcdev->minor = minor;
    if (xcdev->minor >= xcb->cdev_minor_cnt) {
        pr_err("QDMA: No character device slots left for %s\n", qconf->name);
        if (ebuf && ebuflen) {
            rv = snprintf(ebuf, ebuflen, "No cdev slots left for %s\n",
                    qconf->name);
            ebuf[rv] = '\0';
        }
        rv = -ENOSPC;
        goto err_out;
    }

    /* Create the device number */
    xcdev->cdevno = MKDEV(xcb->cdev_major, xcdev->minor);
    pr_debug("QDMA: Assigned device number major=%d, minor=%d\n", 
             xcb->cdev_major, xcdev->minor);

    /* Initialize the character device with fops */
    cdev_init(&xcdev->cdev, &cdev_gen_fops);

    /* Add character device to system */
    rv = cdev_add(&xcdev->cdev, xcdev->cdevno, 1);
    if (rv < 0) {
        pr_err("QDMA: cdev_add failed %d for device %s\n", rv, xcdev->name);
        if (ebuf && ebuflen) {
            int l = snprintf(ebuf, ebuflen,
                    "cdev_add failed %d for %s\n",
                    rv, qconf->name);
            ebuf[l] = '\0';
        }
        goto err_out;
    }

    /* Create device node in sysfs */
    if (qdma_class) {
        pr_debug("QDMA: Creating sysfs device entry for %s\n", xcdev->name);
        xcdev->sys_device = device_create(qdma_class, &(pdev->dev),
                xcdev->cdevno, NULL, "%s", xcdev->name);
        if (IS_ERR(xcdev->sys_device)) {ERR(xcdev->sys_device);
            pr_err("QDMA: device_create failed %d for %s\n",
                rv, xcdev->name);
            if (ebuf && ebuflen) {
                int l = snprintf(ebuf, ebuflen,
                        "device_create failed %d for %s\n",
                        rv, qconf->name);
                ebuf[l] = '\0';
            }
            goto del_cdev;
        }
    }

    /* Set up function pointers for read/write operations */
    xcdev->fp_rw = qdma_request_submit;
    xcdev->fp_aiorw = qdma_batch_request_submit;

    /* Store the created device in the output pointer */
    *xcdev_pp = xcdev;
    pr_debug("QDMA: Successfully created character device %s\n", xcdev->name);
    return 0;

del_cdev:
    pr_debug("QDMA: Cleaning up partially created device %s\n", xcdev->name);
    cdev_del(&xcdev->cdev);

err_out:
    kfree(xcdev);
    pr_err("QDMA: Failed to create character device %s\n", qconf->name);
    return rv;
}

/*
 * @brief Cleans up QDMA character device resources
 * This function handles the cleanup of device-specific resources
 * for a QDMA character device control block
 *
 * @param xcb Pointer to the QDMA character device control block
 */
void qdma_cdev_device_cleanup(struct qdma_cdev_cb *xcb)
{
    if (!xcb->cdev_major) {
        pr_debug("QDMA: Device cleanup skipped - no major number assigned\n");
        return;
    }

    pr_debug("QDMA: Cleaning up device with major number %d\n", xcb->cdev_major);
    xcb->cdev_major = 0;
}

/*
 * @brief Initializes QDMA character device resources
 * Handles device initialization including major number allocation
 * and physical device list management
 *
 * @param xcb pointer to the QDMA character device control block
 * @return 0 on success, negative error code on failure
 */
int qdma_cdev_device_init(struct qdma_cdev_cb *xcb)
{
	dev_t dev;
	int rv;
	struct xlnx_phy_dev *phy_dev, *tmp, *new_phy_dev;
	struct xlnx_dma_dev *xdev = NULL;

	pr_debug("QDMA: Initializing character device resources\n");
    
    spin_lock_init(&xcb->lock);
    xcb->cdev_minor_cnt = QDMA_MINOR_MAX;

    if (xcb->cdev_major) {
        pr_warn("QDMA: Major number %d already exists\n", xcb->cdev_major);
        return -EINVAL;
    }

    /* Check if same bus id device exists in global list */
    mutex_lock(&xlnx_phy_dev_mutex);
    xdev = (struct xlnx_dma_dev *)xcb->xpdev->dev_hndl;
    
    list_for_each_entry_safe(phy_dev, tmp, &xlnx_phy_dev_list, list_head) {
        if (phy_dev->device_bus == xcb->xpdev->pdev->bus->number &&
            phy_dev->dma_device_index == xdev->dma_device_index) {
            xcb->cdev_major = phy_dev->major;
            pr_debug("QDMA: Found existing device - reusing major number %d\n", 
                    phy_dev->major);
            mutex_unlock(&xlnx_phy_dev_mutex);
            return 0;
        }
    }
    mutex_unlock(&xlnx_phy_dev_mutex);

    /* Allocate a dynamically allocated char device node */
    rv = alloc_chrdev_region(&dev, 0, xcb->cdev_minor_cnt, QDMA_CDEV_CLASS_NAME);
    if (rv) {
        pr_err("QDMA: Failed to allocate char device region, error %d\n", rv);
        return rv;
    }
    
    xcb->cdev_major = MAJOR(dev);
    pr_debug("QDMA: Allocated major number %d\n", xcb->cdev_major);

    /* Create new physical device entry */
    new_phy_dev = kzalloc(sizeof(struct xlnx_phy_dev), GFP_KERNEL);
    if (!new_phy_dev) {
        pr_err("QDMA: Failed to allocate memory for physical device\n");
        unregister_chrdev_region(dev, xcb->cdev_minor_cnt);
        return -ENOMEM;
    }

    new_phy_dev->major = xcb->cdev_major;
    new_phy_dev->device_bus = xcb->xpdev->pdev->bus->number;
    new_phy_dev->dma_device_index = xdev->dma_device_index;
    
    xlnx_phy_dev_list_add(new_phy_dev);
    pr_debug("QDMA: Added new physical device to list (major=%d, bus=%d, idx=%d)\n",
             new_phy_dev->major, new_phy_dev->device_bus, 
             new_phy_dev->dma_device_index);

    return 0;
}

/**
 * @brief Initialize QDMA character device driver
 * Creates the device class and initializes the cache for async I/O operations
 *
 * @return 0 on success, negative error code on failure
 */
int qdma_cdev_init(void)
{
    pr_debug("QDMA: Initializing character device driver\n");
    
    qdma_class = class_create(THIS_MODULE, QDMA_CDEV_CLASS_NAME);
    if (IS_ERR(qdma_class)) {
        pr_err("QDMA: Failed to create device class 0x%lx\n", 
               (unsigned long)qdma_class);
        qdma_class = NULL;
        return -ENODEV;
    }

    /* Initialize cache for async I/O operations */
    cdev_cache = kmem_cache_create("cdev_cache",
                    sizeof(struct cdev_async_io),
                    0,
                    SLAB_HWCACHE_ALIGN,
                    NULL);
    if (!cdev_cache) {
        pr_err("QDMA: Failed to allocate cdev cache\n");
        class_destroy(qdma_class);
        return -ENOMEM;
    }

    pr_debug("QDMA: Character device driver initialized successfully\n");
    return 0;
}

/**
 * @brief Cleanup QDMA character device driver resources
 * Handles the cleanup of all driver-wide resources including
 * physical devices, cache, and device class
 */
void qdma_cdev_cleanup(void)
{
    struct xlnx_phy_dev *phy_dev, *tmp;
    
    pr_debug("QDMA: Starting character device driver cleanup\n");

    /* Cleanup all physical devices */
    list_for_each_entry_safe(phy_dev, tmp, &xlnx_phy_dev_list, list_head) {
        pr_debug("QDMA: Unregistering device with major number %d\n", 
                phy_dev->major);
        unregister_chrdev_region(MKDEV(phy_dev->major, 0), QDMA_MINOR_MAX);
        xlnx_phy_dev_list_remove(phy_dev);
        kfree();
    }

    /* Destroy cache and class */
    if (cdev_cache) {
        pr_debug("QDMA: Destroying cdev cache\n");
        kmem_cache_destroy(cdev_cache);
    }
    
    if (qdma_class) {
        pr_debug("QDMA: Destroying device class\n");
        class_destroy(qdma_class);
    }

    pr_debug("QDMA: Character device driver cleanup completed\n");
}
