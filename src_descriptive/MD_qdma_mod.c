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
 */

#define pr_fmt(fmt)	KBUILD_MODNAME ":%s: " fmt, __func__
// This macro defines a format for printk messages, including the module name and function name.

#include "qdma_mod.h"
// Include the header file for the QDMA module.

#include <linux/module.h>   // Core header for loading LKMs into the kernel
#include <linux/kernel.h>   // Contains types, macros, functions for the kernel
#include <linux/version.h>  // Provides version information
#include <linux/types.h>    // Contains type definitions
#include <linux/errno.h>    // Defines error codes
#include <linux/pci.h>      // Provides PCI driver support
#include <linux/aer.h>      // Advanced Error Reporting for PCI
#include <linux/vmalloc.h>  // Memory allocation functions

#include "nl.h"             // Include netlink header
#include "libqdma/xdev.h"   // Include QDMA device header

/* include early, to verify it depends only on the headers above */
#include "version.h"        // Include version information header

#define QDMA_DEFAULT_TOTAL_Q 2048
// Define the default total number of queues for QDMA.

static char version[] =
	DRV_MODULE_DESC " v" DRV_MODULE_VERSION "\n";
// Define a static string for the module version.

MODULE_AUTHOR("Xilinx, Inc.");
MODULE_DESCRIPTION(DRV_MODULE_DESC);
MODULE_VERSION(DRV_MODULE_VERSION);
MODULE_LICENSE("Dual BSD/GPL");
// Set module metadata: author, description, version, and license.

static char mode[500] = {0};
module_param_string(mode, mode, sizeof(mode), 0);
MODULE_PARM_DESC(mode, "Load the driver in different modes, dflt is auto mode, format is \"<bus_num>:<pf_num>:<mode>\" and multiple comma separated entries can be specified");
// Define a module parameter 'mode' to specify driver modes, with a description.

static char config_bar[500] = {0};
module_param_string(config_bar, config_bar, sizeof(config_bar), 0);
MODULE_PARM_DESC(config_bar, "specify the config bar number, dflt is 0, format is \"<bus_num>:<pf_num>:<bar_num>\" and multiple comma separated entries can be specified");
// Define a module parameter 'config_bar' to specify configuration BAR numbers.

static char master_pf[500] = {0};
module_param_string(master_pf, master_pf, sizeof(master_pf), 0);
MODULE_PARM_DESC(master_pf, "specify the master_pf, dflt is 0, format is \"<bus_num>:<master_pf>\" and multiple comma separated entries can be specified");
// Define a module parameter 'master_pf' to specify the master physical function.

static unsigned int num_threads;
module_param(num_threads, uint, 0644);
MODULE_PARM_DESC(num_threads, "Number of threads to be created each for request and writeback processing");
// Define a module parameter 'num_threads' for the number of threads for processing.

#include "pci_ids.h"
// Include PCI IDs header for device identification.

/*
 * xpdev helper functions
 */
static LIST_HEAD(xpdev_list); // Initialize a list head for managing PCI devices.
static DEFINE_MUTEX(xpdev_mutex); // Define a mutex for synchronizing access to the device list.

static int xpdev_qdata_realloc(struct xlnx_pci_dev *xpdev, unsigned int qmax);
// Function prototype for reallocating queue data.

static int xpdev_map_bar(struct xlnx_pci_dev *xpdev,
		void __iomem **regs, u8 bar_num);
// Function prototype for mapping a PCI BAR.

static void xpdev_unmap_bar(struct xlnx_pci_dev *xpdev, void __iomem **regs);
// Function prototype for unmapping a PCI BAR.

#ifdef __QDMA_VF__
void qdma_flr_resource_free(unsigned long dev_hndl);
// Function prototype for freeing resources in a Virtual Function context.
#endif

/*****************************************************************************/
/**
 * show_intr_rngsz() - Handler to show the intr_rngsz configuration value
 *
 * @dev :   PCIe device handle
 * @attr:   intr_rngsz configuration value
 * @buf :   buffer to hold the configured value
 *
 * This function retrieves and displays the interrupt ring size configuration
 * for a given PCIe device.
 *
 * @note    none
 *
 * Return:  Returns length of the buffer on success, <0 on failure
 *
 *****************************************************************************/
static ssize_t show_intr_rngsz(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct xlnx_pci_dev *xpdev; // Pointer to the Xilinx PCI device structure
	int len; // Variable to store the length of the output string
	unsigned int rngsz = 0; // Variable to store the interrupt ring size

	// Retrieve the device-specific data from the device structure
	xpdev = (struct xlnx_pci_dev *)dev_get_drvdata(dev);
	if (!xpdev) {
		pr_err("Failed to get device data\n"); // Debug print statement
		return -EINVAL; // Return error if device data is not found
	}

	// Get the interrupt ring size using the device handle
	rngsz = qdma_get_intr_rngsz(xpdev->dev_hndl);
	// Format the ring size into the buffer
	len = scnprintf(buf, PAGE_SIZE, "%u\n", rngsz);
	if (len <= 0) {
		pr_err("Copying rngsz to buffer failed with err: %d\n", len); // Debug print statement
	}

	return len; // Return the length of the data written to the buffer
}

/*****************************************************************************/
/**
 * set_intr_rngsz() - Handler to set the intr_rngsz configuration value
 *
 * @dev :   PCIe device handle
 * @attr:   intr_rngsz configuration value
 * @buf :   buffer to hold the configured value
 * @count : the number of bytes of data in the buffer
 *
 * This function sets the interrupt ring size configuration for a given PCIe
 * device based on the input buffer.
 *
 * @note    none
 *
 * Return:  Returns length of the buffer on success, <0 on failure
 *
 *****************************************************************************/
static ssize_t set_intr_rngsz(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct xlnx_pci_dev *xpdev; // Pointer to the Xilinx PCI device structure
	unsigned int rngsz = 0; // Variable to store the new interrupt ring size
	int err = 0; // Variable to store error codes

	// Retrieve the device-specific data from the device structure
	xpdev = (struct xlnx_pci_dev *)dev_get_drvdata(dev);
	if (!xpdev) {
		pr_err("Failed to get device data\n"); // Debug print statement
		return -EINVAL; // Return error if device data is not found
	}

	// Convert the input buffer to an unsigned integer
	err = kstrtouint(buf, 0, &rngsz);
	if (err < 0) {
		pr_err("Failed to convert buffer to unsigned int\n"); // Debug print statement
		return err; // Return error if conversion fails
	}

	// Set the interrupt ring size using the device handle
	err = qdma_set_intr_rngsz(xpdev->dev_hndl, (u32)rngsz);
	if (err < 0) {
		pr_err("Failed to set interrupt ring size in hardware\n"); // Debug print statement
		return err; // Return error if setting fails
	}

	return count; // Return the number of bytes processed
}

/*****************************************************************************/
/**
 * show_qmax() - Handler to show the qmax configuration value
 *
 * @dev :   PCIe device handle
 * @attr:   qmax configuration value
 * @buf :   buffer to hold the configured value
 *
 * This function retrieves and displays the maximum number of queues (qmax)
 * configuration for a given PCIe device.
 *
 * @note    none
 *
 * Return:  Returns length of the buffer on success, <0 on failure
 *
 *****************************************************************************/
static ssize_t show_qmax(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct xlnx_pci_dev *xpdev; // Pointer to the Xilinx PCI device structure
	int len; // Variable to store the length of the output string
	unsigned int qmax = 0; // Variable to store the qmax value

	// Retrieve the device-specific data from the device structure
	xpdev = (struct xlnx_pci_dev *)dev_get_drvdata(dev);
	if (!xpdev) {
		pr_err("Failed to get device data\n"); // Debug print statement
		return -EINVAL; // Return error if device data is not found
	}

	// Get the qmax value using the device handle
	qmax = qdma_get_qmax(xpdev->dev_hndl);
	// Format the qmax value into the buffer
	len = scnprintf(buf, PAGE_SIZE, "%u\n", qmax);
	if (len <= 0) {
		pr_err("Copying qmax to buffer failed with err: %d\n", len); // Debug print statement
	}

	return len; // Return the length of the data written to the buffer
}

#ifndef __QDMA_VF__
/*****************************************************************************/
/**
 * set_qmax() - Handler to set the qmax configuration value
 *
 * @dev :   PCIe device handle
 * @attr:   qmax configuration value
 * @buf :   buffer containing the new qmax value
 * @count : the number of bytes of data in the buffer
 *
 * This function sets the maximum number of queues (qmax) configuration for a
 * given PCIe device based on the input buffer.
 *
 * @note    none
 *
 * Return:  Returns length of the buffer on success, <0 on failure
 *
 *****************************************************************************/
static ssize_t set_qmax(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct xlnx_pci_dev *xpdev; // Pointer to the Xilinx PCI device structure
	unsigned int qmax = 0; // Variable to store the new qmax value
	int err = 0; // Variable to store error codes

	// Retrieve the device-specific data from the device structure
	xpdev = (struct xlnx_pci_dev *)dev_get_drvdata(dev);
	if (!xpdev) {
		pr_err("Failed to get device data\n"); // Debug print statement
		return -EINVAL; // Return error if device data is not found
	}

	// Convert the input buffer to an unsigned integer
	err = kstrtouint(buf, 0, &qmax);
	if (err < 0) {
		pr_err("Failed to convert buffer to unsigned int\n"); // Debug print statement
		return err; // Return error if conversion fails
	}

	// Set the qmax value using the device handle
	err = qdma_set_qmax(xpdev->dev_hndl, -1, qmax);
	if (err < 0) {
		pr_err("Failed to set qmax in hardware\n"); // Debug print statement
		return err; // Return error if setting fails
	}

	// Reallocate queue data based on the new qmax value
	if (!err) {
		xpdev_qdata_realloc(xpdev, qmax);
		pr_debug("Reallocated queue data for qmax: %u\n", qmax); // Debug print statement
	}

	return count; // Return the number of bytes processed
}
#endif
