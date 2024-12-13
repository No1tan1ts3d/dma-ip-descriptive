/*
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
 */

/**
 * @file
 * @brief This file contains the declarations for QDMA PCIe device
 */

/* Define format string for debug prints to include function name */
#define pr_fmt(fmt)	KBUILD_MODNAME ":%s: " fmt, __func__

/* Standard Linux kernel includes */
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>

/* QDMA driver specific includes */
#include "qdma_regs.h"          /* QDMA register definitions */
#include "xdev.h"               /* QDMA device structures */
#include "qdma_mbox.h"          /* Mailbox communication */
#include "qdma_intr.h"          /* Interrupt handling */
#include "qdma_resource_mgmt.h" /* Resource management */
#include "qdma_access_common.h" /* Common access functions */

/* Conditional includes based on configuration */
#ifdef DEBUGFS
#include "qdma_debugfs_dev.h"   /* Debug filesystem support */
#endif
#ifdef __XRT__
#include "qdma_access_errors.h" /* XRT specific error handling */
#endif

/* Compatibility definitions for different kernel versions */
#ifdef __LIST_NEXT_ENTRY__
#define list_next_entry(pos, member) \
	list_entry((pos)->member.next, typeof(*(pos)), member)
#endif

/* Queue configuration for non-VF mode */
#ifndef __QDMA_VF__
#ifndef QDMA_QBASE
#define QDMA_QBASE 0
#endif
#ifndef QDMA_TOTAL_Q
/**
 * Queue capacity configuration:
 * - CPM5 supports 4095 Queues
 * - Other designs support 2048 Queues
 * Note: Final queue max is determined by device capabilities
 */
#define QDMA_TOTAL_Q 2048
#endif
#endif

/* Global device list and synchronization */
static LIST_HEAD(xdev_list);    /* List of all QDMA devices */
static DEFINE_MUTEX(xdev_mutex);/* Mutex for device list access */

/* Compatibility macro for list operations */
#ifndef list_last_entry
#define list_last_entry(ptr, type, member) \
		list_entry((ptr)->prev, type, member)
#endif

/**
 * Structure for QDMA resource locking
 * @node: List node for resource tracking
 * @lock: Mutex for resource access synchronization
 */
struct qdma_resource_lock {
	struct list_head node;
	struct mutex lock;
};

/*****************************************************************************/
/**
 * pci_dma_mask_set() - Configure and validate DMA capabilities for the device
 *
 * @param[in]	pdev:	pointer to struct pci_dev
 *
 * @return	0: on success
 * @return	-EINVAL: if no suitable DMA mask can be set
 * 
 * This function attempts to set up DMA addressing capabilities in the following order:
 * 1. First tries 64-bit DMA addressing
 * 2. Falls back to 32-bit DMA if 64-bit is not supported
 * 3. Returns error if neither mode is supported
 *****************************************************************************/
static int pci_dma_mask_set(struct pci_dev *pdev)
{
	printk(KERN_INFO "QDMA: Configuring DMA mask for device %s\n", 
	       dev_name(&pdev->dev));

	/* Try setting 64-bit DMA mask */
	if (!dma_set_mask(&pdev->dev, DMA_BIT_MASK(64))) {
		printk(KERN_INFO "QDMA: Successfully configured 64-bit DMA mask\n");
		dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64));
	} else if (!dma_set_mask(&pdev->dev, DMA_BIT_MASK(32))) {
		/* Fall back to 32-bit DMA mask */
		printk(KERN_INFO "QDMA: Falling back to 32-bit DMA mask\n");
		dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
		dev_info(&pdev->dev, "Using a 32-bit DMA mask.\n");
	} else {
		/* Neither 64-bit nor 32-bit DMA supported */
		dev_err(&pdev->dev, "No suitable DMA addressing mode available\n");
		printk(KERN_ERR "QDMA: Failed to set any DMA mask\n");
		return -EINVAL;
	}

	return 0;
}

#if KERNEL_VERSION(3, 5, 0) <= LINUX_VERSION_CODE
/**
 * PCIe capability control functions for kernel version 3.5.0 and later
 */

/**
 * pci_enable_relaxed_ordering() - Enable relaxed ordering for PCIe device
 * @param[in]	pdev:	pointer to struct pci_dev
 */
static void pci_enable_relaxed_ordering(struct pci_dev *pdev)
{
	printk(KERN_DEBUG "QDMA: Enabling relaxed ordering\n");
	pcie_capability_set_word(pdev, PCI_EXP_DEVCTL, PCI_EXP_DEVCTL_RELAX_EN);
}

/**
 * pci_disable_relaxed_ordering() - Disable relaxed ordering for PCIe device
 * @param[in]	pdev:	pointer to struct pci_dev
 */
static void pci_disable_relaxed_ordering(struct pci_dev *pdev)
{
	printk(KERN_DEBUG "QDMA: Disabling relaxed ordering\n");
	pcie_capability_clear_word(pdev, PCI_EXP_DEVCTL,
			PCI_EXP_DEVCTL_RELAX_EN);
}

/**
 * pci_enable_extended_tag() - Enable extended tag for PCIe device
 * @param[in]	pdev:	pointer to struct pci_dev
 */
static void pci_enable_extended_tag(struct pci_dev *pdev)
{
	printk(KERN_DEBUG "QDMA: Enabling extended tag\n");
	pcie_capability_set_word(pdev, PCI_EXP_DEVCTL, PCI_EXP_DEVCTL_EXT_TAG);
}

/**
 * pci_disable_extended_tag() - Disable extended tag for PCIe device
 * @param[in]	pdev:	pointer to struct pci_dev
 */
static void pci_disable_extended_tag(struct pci_dev *pdev)
{
	printk(KERN_DEBUG "QDMA: Disabling extended tag\n");
	pcie_capability_clear_word(pdev, PCI_EXP_DEVCTL,
			PCI_EXP_DEVCTL_EXT_TAG);
}

#else

#endif /* For kernel version < 3.5.0 */

/**
 * pci_enable_relaxed_ordering() - Enable relaxed ordering for PCIe device
 *
 * @param[in]	pdev:	pointer to struct pci_dev
 *
 * This function enables relaxed ordering by setting the appropriate bit in 
 * the PCIe device control register
 */
static void pci_enable_relaxed_ordering(struct pci_dev *pdev)
{
	u16 v;
	int pos;

	printk(KERN_DEBUG "QDMA: Attempting to enable relaxed ordering\n");
	
	/* Get the PCIe capability position in config space */
	pos = pci_pcie_cap(pdev);
	if (pos > 0) {
		/* Read current device control register value */
		pci_read_config_word(pdev, pos + PCI_EXP_DEVCTL, &v);
		/* Set the relaxed ordering enable bit */
		v |= PCI_EXP_DEVCTL_RELAX_EN;
		/* Write back the modified value */
		pci_write_config_word(pdev, pos + PCI_EXP_DEVCTL, v);
		printk(KERN_DEBUG "QDMA: Relaxed ordering enabled successfully\n");
	} else {
		printk(KERN_WARNING "QDMA: PCIe capability position not found\n");
	}
}

/**
 * pci_disable_relaxed_ordering() - Disable relaxed ordering for PCIe device
 *
 * @param[in]	pdev:	pointer to struct pci_dev
 *
 * This function disables relaxed ordering by clearing the appropriate bit in
 * the PCIe device control register
 */
static void pci_disable_relaxed_ordering(struct pci_dev *pdev)
{
	u16 v;
	int pos;

	printk(KERN_DEBUG "QDMA: Attempting to disable relaxed ordering\n");
	
	pos = pci_pcie_cap(pdev);
	if (pos > 0) {
		pci_read_config_word(pdev, pos + PCI_EXP_DEVCTL, &v);
		v &= ~(PCI_EXP_DEVCTL_RELAX_EN);
		pci_write_config_word(pdev, pos + PCI_EXP_DEVCTL, v);
		printk(KERN_DEBUG "QDMA: Relaxed ordering disabled successfully\n");
	} else {
		printk(KERN_WARNING "QDMA: PCIe capability position not found\n");
	}
}

/**
 * pci_enable_extended_tag() - Enable extended tag for PCIe device
 *
 * @param[in]	pdev:	pointer to struct pci_dev
 *
 * This function enables extended tag capability by setting the appropriate bit
 * in the PCIe device control register
 */
static void pci_enable_extended_tag(struct pci_dev *pdev)
{
	u16 v;
	int pos;

	printk(KERN_DEBUG "QDMA: Attempting to enable extended tag\n");
	
	pos = pci_pcie_cap(pdev);
	if (pos > 0) {
		pci_read_config_word(pdev, pos + PCI_EXP_DEVCTL, &v);
		v |= PCI_EXP_DEVCTL_EXT_TAG;
		pci_write_config_word(pdev, pos + PCI_EXP_DEVCTL, v);
		printk(KERN_DEBUG "QDMA: Extended tag enabled successfully\n");
	} else {
		printk(KERN_WARNING "QDMA: PCIe capability position not found\n");
	}
}

/**
 * pci_disable_extended_tag() - Disable extended tag for PCIe device
 *
 * @param[in]	pdev:	pointer to struct pci_dev
 *
 * This function disables extended tag capability by clearing the appropriate bit
 * in the PCIe device control register
 */
static void pci_disable_extended_tag(struct pci_dev *pdev)
{
	u16 v;
	int pos;

	printk(KERN_DEBUG "QDMA: Attempting to disable extended tag\n");
	
	pos = pci_pcie_cap(pdev);
	if (pos > 0) {
		pci_read_config_word(pdev, pos + PCI_EXP_DEVCTL, &v);
		v &= ~(PCI_EXP_DEVCTL_EXT_TAG);
		pci_write_config_word(pdev, pos + PCI_EXP_DEVCTL, v);
		printk(KERN_DEBUG "QDMA: Extended tag disabled successfully\n");
	} else {
		printk(KERN_WARNING "QDMA: PCIe capability position not found\n");
	}
}
#endif

#if defined(__QDMA_VF__)
/**
 * xdev_reset_work() - Reset work handler for Virtual Function
 *
 * @param[in]	work:	pointer to work_struct
 *
 * This function handles the reset sequence for the Virtual Function device:
 * 1. Handles PF reset request
 * 2. Handles PF offline request
 * 3. Reconfigures the device after reset
 */
static void xdev_reset_work(struct work_struct *work)
{
	struct xlnx_dma_dev *xdev = container_of(work, struct xlnx_dma_dev,
								reset_work);
	struct pci_dev *pdev = xdev->conf.pdev;
	int rv = 0;

	printk(KERN_INFO "QDMA: VF reset work started, current state: %d\n", 
	       xdev->reset_state);

	if (xdev->reset_state == RESET_STATE_RECV_PF_RESET_REQ) {
		printk(KERN_INFO "QDMA: Processing PF reset request\n");

		/* Offline device and disable capabilities */
		qdma_device_offline(pdev, (unsigned long)xdev, XDEV_FLR_ACTIVE);
		pci_disable_extended_tag(pdev);
		pci_disable_relaxed_ordering(pdev);
		pci_release_regions(pdev);
		pci_disable_device(pdev);

#ifndef __XRT__
		/* Request PCI regions */
		rv = pci_request_regions(pdev, "qdma-vf");
		if (rv) {
			pr_err("QDMA: Cannot obtain PCI resources\n");
			return;
		}
#endif

		/* Enable and configure device */
		rv = pci_enable_device(pdev);
		if (rv) {
			pr_err("QDMA: Cannot enable PCI device\n");
#ifndef __XRT__
			pci_release_regions(pdev);
#endif
			return;
		}

		/* Configure device capabilities */
		pci_enable_relaxed_ordering(pdev);
		pci_enable_extended_tag(pdev);
		pci_set_master(pdev);
		pci_dma_mask_set(pdev);
		pcie_set_readrq(pdev, 512);

		/* Bring device online */
		qdma_device_online(pdev, (unsigned long)xdev, XDEV_FLR_ACTIVE);

		if (xdev->reset_state == RESET_STATE_RECV_PF_RESET_DONE) {
			xdev->reset_state = RESET_STATE_IDLE;
			printk(KERN_INFO "QDMA: Reset completed, device now idle\n");
		}
	} else if (xdev->reset_state == RESET_STATE_RECV_PF_OFFLINE_REQ) {
		printk(KERN_INFO "QDMA: Processing PF offline request\n");
		qdma_device_offline(pdev, (unsigned long)xdev,
							XDEV_FLR_INACTIVE);
	}
}
#endif

/*****************************************************************************/
/**
 * xdev_list_first() - Handler to return the first xdev entry from the list
 *
 * @return	pointer to first xlnx_dma_dev on success
 * @return	NULL on failure
 *
 * This function safely retrieves the first QDMA device entry from the global
 * device list under mutex protection.
 *****************************************************************************/
struct xlnx_dma_dev *xdev_list_first(void)
{
    struct xlnx_dma_dev *xdev;

    printk(KERN_DEBUG "QDMA: Retrieving first device from list\n");
    
    /* Protect access to global device list */
    mutex_lock(&xdev_mutex);
    
    /* Get first entry from the device list */
    xdev = list_first_entry(&xdev_list, struct xlnx_dma_dev, list_head);
    
    mutex_unlock(&xdev_mutex);

    if (xdev) {
        printk(KERN_DEBUG "QDMA: Found first device (BDF: %05x)\n", 
               xdev->conf.bdf);
    } else {
        printk(KERN_DEBUG "QDMA: No devices found in list\n");
    }

    return xdev;
}

/*****************************************************************************/
/**
 * xdev_list_next() - Handler to return the next xdev entry from the list
 *
 * @param[in]	xdev:	pointer to current xdev
 *
 * @return	pointer to next xlnx_dma_dev on success
 * @return	NULL on failure
 *
 * This function safely retrieves the next QDMA device entry following the
 * provided device in the global device list under mutex protection.
 *****************************************************************************/
struct xlnx_dma_dev *xdev_list_next(struct xlnx_dma_dev *xdev)
{
    struct xlnx_dma_dev *next;

    printk(KERN_DEBUG "QDMA: Getting next device after BDF %05x\n", 
           xdev->conf.bdf);
    
    /* Protect access to global device list */
    mutex_lock(&xdev_mutex);
    
    /* Get next entry from the device list */
    next = list_next_entry(xdev, list_head);
    
    mutex_unlock(&xdev_mutex);

    if (next) {
        printk(KERN_DEBUG "QDMA: Found next device (BDF: %05x)\n", 
               next->conf.bdf);
    } else {
        printk(KERN_DEBUG "QDMA: No more devices in list\n");
    }

    return next;
}

/*****************************************************************************/
/**
 * xdev_list_dump() - List the DMA device details
 *
 * @param[in]	buflen:	length of the input buffer
 * @param[out]	buf:	message buffer to store device information
 *
 * @return	length of data written to buffer
 *
 * This function dumps information about all QDMA devices in the system into
 * the provided buffer. The information includes:
 * - QDMA device number
 * - PCI bus number
 * - PCI slot number
 * - PCI function number
 *****************************************************************************/
int xdev_list_dump(char *buf, int buflen)
{
    struct xlnx_dma_dev *xdev, *tmp;
    int len = 0;

    printk(KERN_DEBUG "QDMA: Dumping device list information\n");
    
    /* Protect access to global device list */
    mutex_lock(&xdev_mutex);
    
    /* Iterate through all devices safely */
    list_for_each_entry_safe(xdev, tmp, &xdev_list, list_head) {
        /* Format device information into buffer */
        len += snprintf(buf + len, buflen - len,
                "qdma%05x\t%02x:%02x.%02x\n",
                xdev->conf.bdf, 
                xdev->conf.pdev->bus->number,
                PCI_SLOT(xdev->conf.pdev->devfn),
                PCI_FUNC(xdev->conf.pdev->devfn));
                
        printk(KERN_DEBUG "QDMA: Added device BDF %05x to dump\n", 
               xdev->conf.bdf);
               
        /* Check if buffer is full */
        if (len >= buflen) {
            printk(KERN_WARNING "QDMA: Buffer full, truncating device list\n");
            break;
        }
    }
    
    mutex_unlock(&xdev_mutex);

    /* Ensure buffer is null-terminated */
    buf[len] = '\0';
    
    printk(KERN_DEBUG "QDMA: Device list dump complete, wrote %d bytes\n", len);
    
    return len;
}

