/* MD:
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
 * This source code is modified to include debug prints and detailed comments.
 */

#define pr_fmt(fmt)	KBUILD_MODNAME ":%s: " fmt, __func__
// MD: This macro defines a format for printk messages, including the module name and function name.

#include "qdma_mod.h"
// MD: Include the header file for the QDMA module.

#include <linux/module.h>   // MD: Core header for loading LKMs into the kernel
#include <linux/kernel.h>   // MD: Contains types, macros, functions for the kernel
#include <linux/version.h>  // MD: Provides version information
#include <linux/types.h>    // MD: Contains type definitions
#include <linux/errno.h>    // MD: Defines error codes
#include <linux/pci.h>      // MD: Provides PCI driver support
#include <linux/aer.h>      // MD: Advanced Error Reporting for PCI
#include <linux/vmalloc.h>  // MD: Memory allocation functions

#include "nl.h"             // MD: Include netlink header
#include "libqdma/xdev.h"   // MD: Include QDMA device header

/* MD: include early, to verify it depends only on the headers above */
#include "version.h"        // MD: Include version information header

#define QDMA_DEFAULT_TOTAL_Q 2048
// MD: Define the default total number of queues for QDMA.

static char version[] =
	DRV_MODULE_DESC " v" DRV_MODULE_VERSION "\n";
// MD: Define a static string for the module version.

MODULE_AUTHOR("Xilinx, Inc.");
MODULE_DESCRIPTION(DRV_MODULE_DESC);
MODULE_VERSION(DRV_MODULE_VERSION);
MODULE_LICENSE("Dual BSD/GPL");
// MD: Set module metadata: author, description, version, and license.

static char mode[500] = {0};
module_param_string(mode, mode, sizeof(mode), 0);
MODULE_PARM_DESC(mode, "Load the driver in different modes, dflt is auto mode, format is \"<bus_num>:<pf_num>:<mode>\" and multiple comma separated entries can be specified");
// MD: Define a module parameter 'mode' to specify driver modes, with a description.

static char config_bar[500] = {0};
module_param_string(config_bar, config_bar, sizeof(config_bar), 0);
MODULE_PARM_DESC(config_bar, "specify the config bar number, dflt is 0, format is \"<bus_num>:<pf_num>:<bar_num>\" and multiple comma separated entries can be specified");
// MD: Define a module parameter 'config_bar' to specify configuration BAR numbers.

static char master_pf[500] = {0};
module_param_string(master_pf, master_pf, sizeof(master_pf), 0);
MODULE_PARM_DESC(master_pf, "specify the master_pf, dflt is 0, format is \"<bus_num>:<master_pf>\" and multiple comma separated entries can be specified");
// MD: Define a module parameter 'master_pf' to specify the master physical function.

static unsigned int num_threads;
module_param(num_threads, uint, 0644);
MODULE_PARM_DESC(num_threads, "Number of threads to be created each for request and writeback processing");
// MD: Define a module parameter 'num_threads' for the number of threads for processing.

#include "pci_ids.h"
// MD: Include PCI IDs header for device identification.

/* MD:
 * xpdev helper functions
 */
static LIST_HEAD(xpdev_list); // MD: Initialize a list head for managing PCI devices.
static DEFINE_MUTEX(xpdev_mutex); // MD: Define a mutex for synchronizing access to the device list.

static int xpdev_qdata_realloc(struct xlnx_pci_dev *xpdev, unsigned int qmax);
// MD: Function prototype for reallocating queue data.

static int xpdev_map_bar(struct xlnx_pci_dev *xpdev,
		void __iomem **regs, u8 bar_num);
// MD: Function prototype for mapping a PCI BAR.

static void xpdev_unmap_bar(struct xlnx_pci_dev *xpdev, void __iomem **regs);
// MD: Function prototype for unmapping a PCI BAR.

#ifdef __QDMA_VF__
void qdma_flr_resource_free(unsigned long dev_hndl);
// MD: Function prototype for freeing resources in a Virtual Function context.
#endif

/* MD:***************************************************************************/
/* MD:
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
	struct xlnx_pci_dev *xpdev; // MD: Pointer to the Xilinx PCI device structure
	int len; // MD: Variable to store the length of the output string
	unsigned int rngsz = 0; // MD: Variable to store the interrupt ring size

	// MD: Retrieve the device-specific data from the device structure
	xpdev = (struct xlnx_pci_dev *)dev_get_drvdata(dev);
	if (!xpdev) {
		pr_err("Failed to get device data\n"); // MD: Debug print statement
		return -EINVAL; // MD: Return error if device data is not found
	}

	// MD: Get the interrupt ring size using the device handle
	rngsz = qdma_get_intr_rngsz(xpdev->dev_hndl);
	// MD: Format the ring size into the buffer
	len = scnprintf(buf, PAGE_SIZE, "%u\n", rngsz);
	if (len <= 0) {
		pr_err("Copying rngsz to buffer failed with err: %d\n", len); // MD: Debug print statement
	}

	return len; // MD: Return the length of the data written to the buffer
}

/* MD:***************************************************************************/
/* MD:
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
	struct xlnx_pci_dev *xpdev; // MD: Pointer to the Xilinx PCI device structure
	unsigned int rngsz = 0; // MD: Variable to store the new interrupt ring size
	int err = 0; // MD: Variable to store error codes

	// MD: Retrieve the device-specific data from the device structure
	xpdev = (struct xlnx_pci_dev *)dev_get_drvdata(dev);
	if (!xpdev) {
		pr_err("Failed to get device data\n"); // MD: Debug print statement
		return -EINVAL; // MD: Return error if device data is not found
	}

	// MD: Convert the input buffer to an unsigned integer
	err = kstrtouint(buf, 0, &rngsz);
	if (err < 0) {
		pr_err("Failed to convert buffer to unsigned int\n"); // MD: Debug print statement
		return err; // MD: Return error if conversion fails
	}

	// MD: Set the interrupt ring size using the device handle
	err = qdma_set_intr_rngsz(xpdev->dev_hndl, (u32)rngsz);
	if (err < 0) {
		pr_err("Failed to set interrupt ring size in hardware\n"); // MD: Debug print statement
		return err; // MD: Return error if setting fails
	}

	return count; // MD: Return the number of bytes processed
}

/* MD:***************************************************************************/
/* MD:
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
	struct xlnx_pci_dev *xpdev; // MD: Pointer to the Xilinx PCI device structure
	int len; // MD: Variable to store the length of the output string
	unsigned int qmax = 0; // MD: Variable to store the qmax value

	// MD: Retrieve the device-specific data from the device structure
	xpdev = (struct xlnx_pci_dev *)dev_get_drvdata(dev);
	if (!xpdev) {
		pr_err("Failed to get device data\n"); // MD: Debug print statement
		return -EINVAL; // MD: Return error if device data is not found
	}

	// MD: Get the qmax value using the device handle
	qmax = qdma_get_qmax(xpdev->dev_hndl);
	// MD: Format the qmax value into the buffer
	len = scnprintf(buf, PAGE_SIZE, "%u\n", qmax);
	if (len <= 0) {
		pr_err("Copying qmax to buffer failed with err: %d\n", len); // MD: Debug print statement
	}

	return len; // MD: Return the length of the data written to the buffer
}

#ifndef __QDMA_VF__
/* MD:***************************************************************************/
/* MD:
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
	struct xlnx_pci_dev *xpdev; // MD: Pointer to the Xilinx PCI device structure
	unsigned int qmax = 0; // MD: Variable to store the new qmax value
	int err = 0; // MD: Variable to store error codes

	// MD: Retrieve the device-specific data from the device structure
	xpdev = (struct xlnx_pci_dev *)dev_get_drvdata(dev);
	if (!xpdev) {
		pr_err("Failed to get device data\n"); // MD: Debug print statement
		return -EINVAL; // MD: Return error if device data is not found
	}

	// MD: Convert the input buffer to an unsigned integer
	err = kstrtouint(buf, 0, &qmax);
	if (err < 0) {
		pr_err("Failed to convert buffer to unsigned int\n"); // MD: Debug print statement
		return err; // MD: Return error if conversion fails
	}

	// MD: Set the qmax value using the device handle
	err = qdma_set_qmax(xpdev->dev_hndl, -1, qmax);
	if (err < 0) {
		pr_err("Failed to set qmax in hardware\n"); // MD: Debug print statement
		return err; // MD: Return error if setting fails
	}

	// MD: Reallocate queue data based on the new qmax value
	if (!err) {
		xpdev_qdata_realloc(xpdev, qmax);
		pr_debug("Reallocated queue data for qmax: %u\n", qmax); // MD: Debug print statement
	}

	return count; // MD: Return the number of bytes processed
}
#endif

/* MD:***************************************************************************/
/* MD:
 * show_cmpl_status_acc() - Handler to show the cmpl_status_acc configuration value
 *
 * @dev :   PCIe device handle
 * @attr:   cmpl_status_acc configuration value
 * @buf :   buffer to hold the configured value
 *
 * This function retrieves and displays the completion status accumulator
 * configuration for a given PCIe device.
 *
 * @note    none
 *
 * Return:  Returns length of the buffer on success, <0 on failure
 *
 *****************************************************************************/
static ssize_t show_cmpl_status_acc(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct xlnx_pci_dev *xpdev; // MD: Pointer to the Xilinx PCI device structure
	int len; // MD: Variable to store the length of the output string
	unsigned int cmpl_status_acc = 0; // MD: Variable to store the completion status accumulator value

	// MD: Retrieve the device-specific data from the device structure
	xpdev = (struct xlnx_pci_dev *)dev_get_drvdata(dev);
	if (!xpdev) {
		pr_err("Failed to get device data\n"); // MD: Debug print statement
		return -EINVAL; // MD: Return error if device data is not found
	}

	// MD: Get the completion status accumulator value using the device handle
	cmpl_status_acc = qdma_get_wb_intvl(xpdev->dev_hndl);
	// MD: Format the completion status accumulator value into the buffer
	len = scnprintf(buf, PAGE_SIZE, "%u\n", cmpl_status_acc);
	if (len <= 0) {
		pr_err("Copying cmpl status acc value to buffer failed with err: %d\n", len); // MD: Debug print statement
	}

	return len; // MD: Return the length of the data written to the buffer
}

/* MD:***************************************************************************/
/* MD:
 * set_cmpl_status_acc() - Handler to set the cmpl_status_acc configuration value
 *
 * @dev :   PCIe device handle
 * @attr:   cmpl_status_acc configuration value
 * @buf :   buffer containing the new cmpl_status_acc value
 * @count : the number of bytes of data in the buffer
 *
 * This function sets the completion status accumulator configuration for a
 * given PCIe device based on the input buffer.
 *
 * @note    This function is only available if QDMA_CSR_REG_UPDATE is defined.
 *
 * Return:  Returns length of the buffer on success, <0 on failure
 *
 *****************************************************************************/
static ssize_t set_cmpl_status_acc(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef QDMA_CSR_REG_UPDATE
	struct pci_dev *pdev = to_pci_dev(dev); // MD: Convert device to PCI device
	struct xlnx_pci_dev *xpdev; // MD: Pointer to the Xilinx PCI device structure
	unsigned int cmpl_status_acc = 0; // MD: Variable to store the new cmpl_status_acc value
	int err = 0; // MD: Variable to store error codes

	// MD: Check if the PCI device is valid
	if (!pdev)
		return -EINVAL; // MD: Return error if PCI device is not valid

	// MD: Retrieve the device-specific data from the device structure
	xpdev = (struct xlnx_pci_dev *)dev_get_drvdata(dev);
	if (!xpdev)
		return -EINVAL; // MD: Return error if device data is not found

	// MD: Convert the input buffer to an unsigned integer
	err = kstrtoint(buf, 0, &cmpl_status_acc);
	if (err < 0) {
		pr_err("Failed to convert buffer to integer for cmpl_status_acc\n"); // MD: Debug print statement
		return err; // MD: Return error if conversion fails
	}

	// MD: Set the cmpl_status_acc value using the device handle
	err = qdma_set_cmpl_status_acc(xpdev->dev_hndl, cmpl_status_acc);
	if (err < 0) {
		pr_err("Failed to set cmpl_status_acc in hardware\n"); // MD: Debug print statement
		return err; // MD: Return error if setting fails
	}

	return count; // MD: Return the number of bytes processed
#else
	pr_warn("QDMA CSR completion status accumulation update is not allowed\n"); // MD: Warning if the feature is not enabled
	return -EPERM; // MD: Return permission error if the feature is not enabled
#endif
}

/* MD:***************************************************************************/
/* MD:
 * show_c2h_buf_sz() - Handler to show the C2H buffer size configuration value
 *
 * @dev :   PCIe device handle
 * @attr:   C2H buffer size configuration attribute
 * @buf :   Buffer to hold the configured value
 *
 * This function retrieves the buffer size configuration for the C2H path
 * and formats it into a string for display.
 *
 * @note    none
 *
 * Return:  Returns length of the buffer on success, <0 on failure
 *
 *****************************************************************************/
static ssize_t show_c2h_buf_sz(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct xlnx_pci_dev *xpdev; // MD: Pointer to the Xilinx PCI device structure
	int len = 0; // MD: Variable to store the length of the output string
	int i; // MD: Loop index
	unsigned int c2h_buf_sz[QDMA_GLOBAL_CSR_ARRAY_SZ] = {0}; // MD: Array to store buffer sizes

	// MD: Retrieve the device-specific data from the device structure
	xpdev = (struct xlnx_pci_dev *)dev_get_drvdata(dev);
	if (!xpdev) {
		pr_err("Failed to get device data\n"); // MD: Debug print statement
		return -EINVAL; // MD: Return error if device data is not found
	}

	// MD: Get the buffer sizes using the device handle
	qdma_get_buf_sz(xpdev->dev_hndl, c2h_buf_sz);

	// MD: Format the first buffer size into the output buffer
	len += scnprintf(buf + len, PAGE_SIZE - len, "%u", c2h_buf_sz[0]);
	// MD: Format the remaining buffer sizes into the output buffer
	for (i = 1; i < QDMA_GLOBAL_CSR_ARRAY_SZ; i++)
		len += scnprintf(buf + len, PAGE_SIZE - len, " %u", c2h_buf_sz[i]);
	// MD: Add a newline character at the end
	len += scnprintf(buf + len, PAGE_SIZE - len, "%s", "\n\0");

	return len; // MD: Return the length of the data written to the buffer
}

/* MD:***************************************************************************/
/* MD:
 * set_c2h_buf_sz() - Handler to set the C2H buffer size configuration value
 *
 * @dev :   PCIe device handle
 * @attr:   C2H buffer size configuration attribute
 * @buf :   Buffer containing the new configuration values
 * @count : The number of bytes of data in the buffer
 *
 * This function sets the buffer size configuration for the C2H path based on
 * the input buffer. It supports updating multiple buffer sizes.
 *
 * @note    none
 *
 * Return:  Returns length of the buffer on success, <0 on failure
 *
 *****************************************************************************/
static ssize_t set_c2h_buf_sz(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef QDMA_CSR_REG_UPDATE
	struct pci_dev *pdev = to_pci_dev(dev); // MD: Convert device to PCI device
	struct xlnx_pci_dev *xpdev; // MD: Pointer to the Xilinx PCI device structure
	int err = 0; // MD: Variable to store error codes
	char *s = (char *)buf, *p = NULL; // MD: Pointers for string tokenization
	const char *tc = " "; // MD: Token character is a space
	unsigned int c2h_buf_sz[QDMA_GLOBAL_CSR_ARRAY_SZ] = {0}; // MD: Array to store buffer sizes
	int i = 0; // MD: Loop index

	if (!pdev) {
		pr_err("Failed to get PCI device\n"); // MD: Debug print statement
		return -EINVAL; // MD: Return error if PCI device is not found
	}

	// MD: Retrieve the device-specific data from the device structure
	xpdev = (struct xlnx_pci_dev *)dev_get_drvdata(dev);
	if (!xpdev) {
		pr_err("Failed to get device data\n"); // MD: Debug print statement
		return -EINVAL; // MD: Return error if device data is not found
	}

	// MD: Get the current buffer sizes to restore values if fewer than 16 are configured
	qdma_get_buf_sz(xpdev->dev_hndl, c2h_buf_sz);

	// MD: Tokenize the input buffer and convert each token to an integer
	while (((p = strsep(&s, tc)) != NULL) && (i < QDMA_GLOBAL_CSR_ARRAY_SZ)) {
		if (*p == 0)
			continue;

		err = kstrtoint(p, 0, &c2h_buf_sz[i]);
		if (err < 0) {
			pr_err("Failed to convert buffer size entry\n"); // MD: Debug print statement
			goto input_err;
		}

		i++;
	}

	if (p) {
		// MD: Warn if more than 16 entries are provided and ignore extras
		pr_warn("Found more than 16 buffer size entries. Ignoring extra entries\n");
	}

	// MD: Set the buffer sizes using the device handle
	err = qdma_set_buf_sz(xpdev->dev_hndl, c2h_buf_sz);

input_err:
	return err ? err : count; // MD: Return error or the number of bytes processed
#else
	pr_warn("QDMA CSR C2H buffer size update is not allowed\n");
	return -EPERM; // MD: Return permission error if updates are not allowed
#endif
}

/* MD:***************************************************************************/
/* MD:
 * show_glbl_rng_sz() - Handler to show the global ring size configuration value
 *
 * @dev :   PCIe device handle
 * @attr:   ring_sz configuration value
 * @buf :   buffer to hold the configured value
 *
 * This function retrieves and displays the global ring size configuration
 * for a given PCIe device.
 *
 * @note    none
 *
 * Return:  Returns length of the buffer on success, <0 on failure
 *
 *****************************************************************************/
static ssize_t show_glbl_rng_sz(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct xlnx_pci_dev *xpdev; // MD: Pointer to the Xilinx PCI device structure
	struct xlnx_dma_dev *xdev = NULL; // MD: Pointer to the Xilinx DMA device structure
	int len = 0; // MD: Variable to store the length of the output string
	int i; // MD: Loop counter
	unsigned int glbl_ring_sz[QDMA_GLOBAL_CSR_ARRAY_SZ] = {0}; // MD: Array to store ring sizes

	// MD: Retrieve the device-specific data from the device structure
	xpdev = (struct xlnx_pci_dev *)dev_get_drvdata(dev);
	if (!xpdev) {
		pr_err("Failed to get device data\n"); // MD: Debug print statement
		return -EINVAL; // MD: Return error if device data is not found
	}

	// MD: Cast the device handle to the DMA device structure
	xdev = (struct xlnx_dma_dev *)(xpdev->dev_hndl);

	// MD: Retrieve the global ring sizes
	qdma_get_ring_sizes(xdev, 0, QDMA_GLOBAL_CSR_ARRAY_SZ, glbl_ring_sz);

	// MD: Format the first ring size into the buffer
	len += scnprintf(buf + len, PAGE_SIZE - len, "%u", glbl_ring_sz[0]);

	// MD: Format the remaining ring sizes into the buffer
	for (i = 1; i < QDMA_GLOBAL_CSR_ARRAY_SZ; i++)
		len += scnprintf(buf + len, PAGE_SIZE - len, " %u", glbl_ring_sz[i]);

	// MD: Add a newline character to the buffer
	len += scnprintf(buf + len, PAGE_SIZE - len, "%s", "\n\0");

	return len; // MD: Return the length of the data written to the buffer
}

/* MD:***************************************************************************/
/* MD:
 * set_glbl_rng_sz() - Handler to set the global ring size configuration value
 *
 * @dev :   PCIe device handle
 * @attr:   buf_sz configuration value
 * @buf :   buffer containing the new configuration
 * @count : the number of bytes of data in the buffer
 *
 * This function sets the global ring size configuration for a given PCIe
 * device based on the input buffer.
 *
 * @note    none
 *
 * Return:  Returns length of the buffer on success, <0 on failure
 *
 *****************************************************************************/
static ssize_t set_glbl_rng_sz(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef QDMA_CSR_REG_UPDATE
	struct pci_dev *pdev = to_pci_dev(dev); // MD: Convert device to PCI device
	struct xlnx_pci_dev *xpdev; // MD: Pointer to the Xilinx PCI device structure
	int err = 0; // MD: Variable to store error codes
	char *s = (char *)buf, *p = NULL; // MD: Pointers for string tokenization
	const char *tc = " "; // MD: Token character is a space
	unsigned int glbl_ring_sz[QDMA_GLOBAL_CSR_ARRAY_SZ] = {0}; // MD: Array to store ring sizes
	int i = 0; // MD: Loop counter

	if (!pdev) {
		pr_err("Failed to convert device to PCI device\n"); // MD: Debug print statement
		return -EINVAL; // MD: Return error if conversion fails
	}

	// MD: Retrieve the device-specific data from the device structure
	xpdev = (struct xlnx_pci_dev *)dev_get_drvdata(dev);
	if (!xpdev) {
		pr_err("Failed to get device data\n"); // MD: Debug print statement
		return -EINVAL; // MD: Return error if device data is not found
	}

	// MD: Retrieve the current global ring sizes
	qdma_get_glbl_rng_sz(xpdev->dev_hndl, glbl_ring_sz);

	// MD: Parse the input buffer and update the ring sizes
	while (((p = strsep(&s, tc)) != NULL) && (i < QDMA_GLOBAL_CSR_ARRAY_SZ)) {
		if (*p == 0)
			continue;

		err = kstrtoint(p, 0, &glbl_ring_sz[i]);
		if (err < 0) {
			pr_err("Failed to convert string to integer\n"); // MD: Debug print statement
			goto input_err; // MD: Jump to error handling
		}

		i++;
	}

	if (p) {
		// MD: Warn if more than 16 entries are provided
		pr_warn("Found more than 16 ring size entries. Ignoring extra entries\n");
	}

	// MD: Set the new global ring sizes
	err = qdma_set_glbl_rng_sz(xpdev->dev_hndl, glbl_ring_sz);

input_err:
	return err ? err : count; // MD: Return error or count
#else
	pr_warn("QDMA CSR global ring size update is not allowed\n"); // MD: Debug print statement
	return -EPERM; // MD: Return permission error if updates are not allowed
#endif
}

static ssize_t show_c2h_timer_cnt(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct xlnx_pci_dev *xpdev; // MD: Pointer to the Xilinx PCI device structure
	int len = 0; // MD: Variable to store the length of the output string
	int i; // MD: Loop counter
	unsigned int c2h_timer_cnt[QDMA_GLOBAL_CSR_ARRAY_SZ] = {0}; // MD: Array to store timer count values

	// MD: Retrieve the device-specific data from the device structure
	xpdev = (struct xlnx_pci_dev *)dev_get_drvdata(dev);
	if (!xpdev) {
		pr_err("Failed to get device data\n"); // MD: Debug print statement
		return -EINVAL; // MD: Return error if device data is not found
	}

	// MD: Get the timer count values using the device handle
	qdma_get_timer_cnt(xpdev->dev_hndl, c2h_timer_cnt);

	// MD: Format the first timer count value into the buffer
	len += scnprintf(buf + len, PAGE_SIZE - len, "%u", c2h_timer_cnt[0]);
	// MD: Format the remaining timer count values into the buffer
	for (i = 1; i < QDMA_GLOBAL_CSR_ARRAY_SZ; i++)
		len += scnprintf(buf + len, PAGE_SIZE - len, " %u", c2h_timer_cnt[i]);
	// MD: Add a newline character at the end
	len += scnprintf(buf + len, PAGE_SIZE - len, "%s", "\n\0");

	return len; // MD: Return the length of the data written to the buffer
}

static ssize_t set_c2h_timer_cnt(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef QDMA_CSR_REG_UPDATE
	struct pci_dev *pdev = to_pci_dev(dev); // MD: Convert device to PCI device
	struct xlnx_pci_dev *xpdev; // MD: Pointer to the Xilinx PCI device structure
	int err = 0; // MD: Variable to store error codes
	char *s = (char *)buf, *p = NULL; // MD: Pointers for string tokenization
	const char *tc = " "; // MD: Token character is a space
	unsigned int c2h_timer_cnt[QDMA_GLOBAL_CSR_ARRAY_SZ] = {0}; // MD: Array to store new timer count values
	int i = 0; // MD: Loop counter

	if (!pdev) {
		pr_err("Failed to get PCI device\n"); // MD: Debug print statement
		return -EINVAL; // MD: Return error if PCI device is not found
	}

	// MD: Retrieve the device-specific data from the device structure
	xpdev = (struct xlnx_pci_dev *)dev_get_drvdata(dev);
	if (!xpdev) {
		pr_err("Failed to get device data\n"); // MD: Debug print statement
		return -EINVAL; // MD: Return error if device data is not found
	}

	// MD: Get the current timer count values
	qdma_get_timer_cnt(xpdev->dev_hndl, c2h_timer_cnt);

	// MD: Parse the input buffer and update the timer count values
	while (((p = strsep(&s, tc)) != NULL) && (i < QDMA_GLOBAL_CSR_ARRAY_SZ)) {
		if (*p == 0)
			continue;

		err = kstrtoint(p, 0, &c2h_timer_cnt[i]);
		if (err < 0) {
			pr_err("Failed to convert string to integer\n"); // MD: Debug print statement
			goto input_err;
		}

		if (c2h_timer_cnt[i] > 255) {
			pr_warn("Timer count at index %d is %d - out of range [0-255]\n", i, c2h_timer_cnt[i]);
			err = -EINVAL;
			goto input_err;
		}
		i++;
	}

	if (p) {
		pr_warn("Found more than 16 timer entries. Ignoring extra entries\n");
	}

	// MD: Set the new timer count values
	err = qdma_set_timer_cnt(xpdev->dev_hndl, c2h_timer_cnt);

input_err:
	return err ? err : count; // MD: Return error or the number of bytes processed
#else
	pr_warn("QDMA CSR C2H timer counter update is not allowed\n");
	return -EPERM; // MD: Return permission error if updates are not allowed
#endif
}

/* MD:***************************************************************************/
/* MD:
 * show_c2h_cnt_th() - Handler to show global CSR c2h_cnt_th configuration value
 *
 * @dev :   PCIe device handle
 * @attr:   c2h_cnt_th configuration value
 * @buf :   buffer to hold the configured value
 *
 * This function retrieves and displays the C2H (Completion to Host) count
 * threshold configuration for a given PCIe device.
 *
 * @note    none
 *
 * Return:  Returns length of the buffer on success, <0 on failure
 *
 *****************************************************************************/
static ssize_t show_c2h_cnt_th(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct xlnx_pci_dev *xpdev; // MD: Pointer to the Xilinx PCI device structure
	int len = 0; // MD: Variable to store the length of the output string
	int i; // MD: Loop counter
	unsigned int c2h_cnt_th[QDMA_GLOBAL_CSR_ARRAY_SZ] = {0}; // MD: Array to store count thresholds

	// MD: Retrieve the device-specific data from the device structure
	xpdev = (struct xlnx_pci_dev *)dev_get_drvdata(dev);
	if (!xpdev) {
		pr_err("Failed to get device data\n"); // MD: Debug print statement
		return -EINVAL; // MD: Return error if device data is not found
	}

	// MD: Get the count thresholds using the device handle
	qdma_get_cnt_thresh(xpdev->dev_hndl, c2h_cnt_th);

	// MD: Format the first count threshold into the buffer
	len += scnprintf(buf + len, PAGE_SIZE - len, "%u", c2h_cnt_th[0]);
	// MD: Format the remaining count thresholds into the buffer
	for (i = 1; i < QDMA_GLOBAL_CSR_ARRAY_SZ; i++)
		len += scnprintf(buf + len, PAGE_SIZE - len, " %u", c2h_cnt_th[i]);
	// MD: Add a newline character at the end
	len += scnprintf(buf + len, PAGE_SIZE - len, "%s", "\n\0");

	return len; // MD: Return the length of the data written to the buffer
}

/* MD:***************************************************************************/
/* MD:
 * set_c2h_cnt_th() - Handler to set global CSR c2h_cnt_th configuration
 *
 * @dev :   PCIe device handle
 * @attr:   c2h_cnt_th configuration value
 * @buf :   buffer containing new configuration
 * @count : the number of bytes of data in the buffer
 *
 * This function sets the C2H count threshold configuration for a given PCIe
 * device based on the input buffer.
 *
 * @note    none
 *
 * Return:  Returns length of the buffer on success, <0 on failure
 *
 *****************************************************************************/
static ssize_t set_c2h_cnt_th(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef QDMA_CSR_REG_UPDATE
	struct pci_dev *pdev = to_pci_dev(dev); // MD: Convert device to PCI device
	struct xlnx_pci_dev *xpdev; // MD: Pointer to the Xilinx PCI device structure
	int err = 0; // MD: Variable to store error codes
	char *s = (char *)buf, *p = NULL; // MD: Pointers for string tokenization
	const char *tc = " "; // MD: Token character is a space
	unsigned int c2h_cnt_th[QDMA_GLOBAL_CSR_ARRAY_SZ] = {0}; // MD: Array to store count thresholds
	int i = 0; // MD: Loop counter

	if (!pdev) {
		pr_err("Failed to get PCI device\n"); // MD: Debug print statement
		return -EINVAL; // MD: Return error if PCI device is not found
	}

	// MD: Retrieve the device-specific data from the device structure
	xpdev = (struct xlnx_pci_dev *)dev_get_drvdata(dev);
	if (!xpdev) {
		pr_err("Failed to get device data\n"); // MD: Debug print statement
		return -EINVAL; // MD: Return error if device data is not found
	}

	// MD: Get the current count thresholds to restore values if fewer than 16 are configured
	qdma_get_cnt_thresh(xpdev->dev_hndl, c2h_cnt_th);

	// MD: Parse the input buffer and update the count thresholds
	while (((p = strsep(&s, tc)) != NULL) && (i < QDMA_GLOBAL_CSR_ARRAY_SZ)) {
		if (*p == 0)
			continue;

		err = kstrtoint(p, 0, &c2h_cnt_th[i]);
		if (err < 0) {
			pr_err("Failed to convert input to integer\n"); // MD: Debug print statement
			goto input_err;
		}

		if (c2h_cnt_th[i] > 255) {
			pr_warn("Counter threshold at index %d is %d - out of range [0-255]\n",
				i, c2h_cnt_th[i]);
			err = -EINVAL;
			goto input_err;
		}
		i++;
	}

	if (p) {
		pr_warn("Found more than 16 counter entries. Ignoring extra entries\n");
	}

	// MD: Set the new count thresholds using the device handle
	err = qdma_set_cnt_thresh(xpdev->dev_hndl, c2h_cnt_th);

input_err:
	return err ? err : count; // MD: Return error or count based on success
#else
	pr_warn("QDMA CSR C2H counter threshold update is not allowed\n");
	return -EPERM; // MD: Return permission error if updates are not allowed
#endif
}

// MD: Define device attributes for qmax and intr_rngsz, which are readable and writable by the user.
static DEVICE_ATTR(qmax, S_IWUSR | S_IRUGO, show_qmax, set_qmax);
static DEVICE_ATTR(intr_rngsz, S_IWUSR | S_IRUGO, show_intr_rngsz, set_intr_rngsz);

#ifndef __QDMA_VF__
// MD: Define additional device attributes for non-virtual functions (VF).
static DEVICE_ATTR(buf_sz, S_IWUSR | S_IRUGO, show_c2h_buf_sz, set_c2h_buf_sz);
static DEVICE_ATTR(glbl_rng_sz, S_IWUSR | S_IRUGO, show_glbl_rng_sz, set_glbl_rng_sz);
static DEVICE_ATTR(c2h_timer_cnt, S_IWUSR | S_IRUGO, show_c2h_timer_cnt, set_c2h_timer_cnt);
static DEVICE_ATTR(c2h_cnt_th, S_IWUSR | S_IRUGO, show_c2h_cnt_th, set_c2h_cnt_th);
static DEVICE_ATTR(cmpl_status_acc, S_IWUSR | S_IRUGO, show_cmpl_status_acc, set_cmpl_status_acc);
#endif

// MD: Array of attributes for PCI devices, including qmax and intr_rngsz.
static struct attribute *pci_device_attrs[] = {
    &dev_attr_qmax.attr,
    &dev_attr_intr_rngsz.attr,
    NULL, // MD: Null-terminated array
};

// MD: Array of attributes for PCI master devices, including additional attributes for non-VF.
static struct attribute *pci_master_device_attrs[] = {
    &dev_attr_qmax.attr,
    &dev_attr_intr_rngsz.attr,
#ifndef __QDMA_VF__
    &dev_attr_buf_sz.attr,
    &dev_attr_glbl_rng_sz.attr,
    &dev_attr_c2h_timer_cnt.attr,
    &dev_attr_c2h_cnt_th.attr,
    &dev_attr_cmpl_status_acc.attr,
#endif
    NULL, // MD: Null-terminated array
};

// MD: Define an attribute group for PCI devices, named "qdma", with the specified attributes.
static struct attribute_group pci_device_attr_group = {
    .name  = "qdma",
    .attrs = pci_device_attrs,
};

// MD: Define an attribute group for PCI master devices, named "qdma", with the specified attributes.
static struct attribute_group pci_master_device_attr_group = {
    .name  = "qdma",
    .attrs = pci_master_device_attrs,
};

static inline void xpdev_list_remove(struct xlnx_pci_dev *xpdev)
{
    // MD: Lock the mutex to ensure exclusive access to the device list
    mutex_lock(&xpdev_mutex);
    // MD: Remove the device from the list
    list_del(&xpdev->list_head);
    // MD: Unlock the mutex after the operation
    mutex_unlock(&xpdev_mutex);
}

static inline void xpdev_list_add(struct xlnx_pci_dev *xpdev)
{
    // MD: Lock the mutex to ensure exclusive access to the device list
    mutex_lock(&xpdev_mutex);
    // MD: Add the device to the end of the list
    list_add_tail(&xpdev->list_head, &xpdev_list);
    // MD: Unlock the mutex after the operation
    mutex_unlock(&xpdev_mutex);
}

int xpdev_list_dump(char *buf, int buflen)
{
    struct xlnx_pci_dev *xpdev, *tmp;
    char *cur = buf; // MD: Pointer to the current position in the buffer
    char *const end = buf + buflen; // MD: Pointer to the end of the buffer
    int base_end = 0; // MD: Variable to store the end of the queue set range
    int qmax_val = 0; // MD: Variable to store the maximum number of queue sets

    // MD: Check if the buffer is valid and has a positive length
    if (!buf || !buflen)
        return -EINVAL; // MD: Return an invalid argument error

    // MD: Lock the mutex to ensure exclusive access to the device list
    mutex_lock(&xpdev_mutex);
    // MD: Iterate over each device in the list safely
    list_for_each_entry_safe(xpdev, tmp, &xpdev_list, list_head) {
        struct pci_dev *pdev;
        struct qdma_dev_conf conf;
        int rv;

        // MD: Get the device configuration
        rv = qdma_device_get_config(xpdev->dev_hndl, &conf, NULL, 0);
        if (rv < 0) {
            // MD: If configuration retrieval fails, log an error
            cur += snprintf(cur, cur - end,
            "ERR! unable to get device config for idx %05x\n",
            xpdev->idx);
            if (cur >= end)
                goto handle_truncation; // MD: Handle buffer truncation
            break;
        }

        pdev = conf.pdev; // MD: Get the PCI device from the configuration

        // MD: Calculate the end of the queue set range
        base_end = (int)(conf.qsets_base + conf.qsets_max - 1);
        if (base_end < 0)
            base_end = 0;
        qmax_val = conf.qsets_max; // MD: Get the maximum number of queue sets

        // MD: Format the device information into the buffer
        if (qmax_val) {
            cur += snprintf(cur, end - cur,
#ifdef __QDMA_VF__
                    "qdmavf%05x\t%s\tmax QP: %d, %d~%d\n",
#else
                    "qdma%05x\t%s\tmax QP: %d, %d~%d\n",
#endif
                    xpdev->idx, dev_name(&pdev->dev),
                    qmax_val, conf.qsets_base,
                    base_end);
        } else {
            cur += snprintf(cur, end - cur,
#ifdef __QDMA_VF__
                    "qdmavf%05x\t%s\tmax QP: 0, -~-\n",
#else
                    "qdma%05x\t%s\tmax QP: 0, -~-\n",
#endif
                    xpdev->idx, dev_name(&pdev->dev));
        }
        if (cur >= end)
            goto handle_truncation; // MD: Handle buffer truncation
    }
    // MD: Unlock the mutex after the operation
    mutex_unlock(&xpdev_mutex);

    return cur - buf; // MD: Return the number of characters written to the buffer

handle_truncation:
    // MD: Handle the case where the buffer is too small
    mutex_unlock(&xpdev_mutex);
    pr_warn("ERR! str truncated. req=%lu, avail=%u", cur - buf, buflen);
    *buf = '\0'; // MD: Null-terminate the buffer
    return cur - buf; // MD: Return the number of characters written to the buffer
}

/* MD:
 * is_first_pfdev() - Function to find the first PF device available in the card
 * 
 * @bus_number: The bus number to check for the first PF device.
 * 
 * This function checks if the given bus number corresponds to the first
 * Physical Function (PF) device on the card. It returns true if it is the
 * first PF device, otherwise false.
 * 
 * Return: true if it is the first PF device, false otherwise.
 */
static bool is_first_pfdev(u8 bus_number)
{
    struct xlnx_pci_dev *_xpdev, *tmp;

    // MD: Lock the mutex to ensure exclusive access to the device list
    mutex_lock(&xpdev_mutex);

    // MD: Check if the device list is empty
    if (list_empty(&xpdev_list)) {
        mutex_unlock(&xpdev_mutex);
        return true; // MD: Return true if no devices are in the list
    }

    // MD: Iterate over the device list to find a matching bus number
    list_for_each_entry_safe(_xpdev, tmp, &xpdev_list, list_head) {
        struct pci_dev *pdev = _xpdev->pdev;
        // MD: Check if the bus number matches
        if (pdev->bus->number == bus_number) {
            mutex_unlock(&xpdev_mutex);
            return false; // MD: Return false if a matching device is found
        }
    }

    mutex_unlock(&xpdev_mutex);
    return true; // MD: Return true if no matching device is found
}

/* MD:
 * extract_mod_param() - Extract the device mode and config bar per function
 * 
 * @pdev: The PCIe device to extract parameters for.
 * @param_type: The type of module parameter to extract (DRV_MODE, CONFIG_BAR, MASTER_PF).
 * 
 * This function extracts the specified module parameter for a given PCIe device.
 * It supports extracting the driver mode, configuration BAR, and master PF settings.
 * 
 * Return: The extracted parameter value, or an error code if extraction fails.
 */
static u8 extract_mod_param(struct pci_dev *pdev,
                            enum qdma_drv_mod_param_type param_type)
{
    char p[600];
    char *ptr, *mod;

    u8 dev_fn = PCI_FUNC(pdev->devfn); // MD: Get the function number of the device
#ifdef __QDMA_VF__
    u16 device_id = pdev->device; // MD: Get the device ID for virtual functions
#endif

    // MD: Initialize the pointer to the parameter string
    ptr = p;
    if (param_type == DRV_MODE) {
        if (mode[0] == '\0')
            return 0; // MD: Return 0 if the mode string is empty
        strncpy(p, mode, sizeof(p) - 1); // MD: Copy the mode string
    } else if (param_type == CONFIG_BAR) {
        if (config_bar[0] == '\0')
            return 0; // MD: Return 0 if the config_bar string is empty
        strncpy(p, config_bar, sizeof(p) - 1); // MD: Copy the config_bar string
    } else if (param_type == MASTER_PF) {
        if (master_pf[0] == '\0')
            return is_first_pfdev(pdev->bus->number); // MD: Check if it's the first PF
        strncpy(p, master_pf, sizeof(p) - 1); // MD: Copy the master_pf string
    } else {
        pr_err("Invalid module param type received\n");
        return -EINVAL; // MD: Return error for invalid parameter type
    }

    // MD: Parse the parameter string for the specified type
    while ((mod = strsep(&ptr, ","))) {
        unsigned int bus_num, func_num, param = 0;
        int fields;

        if (!strlen(mod))
            continue; // MD: Skip empty strings

        if (param_type == MASTER_PF) {
            fields = sscanf(mod, "%x:%x", &bus_num, &param);

            if (fields != 2) {
                pr_warn("invalid mode string \"%s\"\n", mod);
                continue; // MD: Warn and continue if the string is invalid
            }

            if ((bus_num == pdev->bus->number) && (dev_fn == param))
                return 1; // MD: Return 1 if the bus and function numbers match
        } else {
            fields = sscanf(mod, "%x:%x:%x", &bus_num, &func_num, &param);

            if (fields != 3) {
                pr_warn("invalid mode string \"%s\"\n", mod);
                continue; // MD: Warn and continue if the string is invalid
            }

#ifndef __QDMA_VF__
            if ((bus_num == pdev->bus->number) && (func_num == dev_fn))
                return param; // MD: Return the parameter if bus and function match
#else
            if ((bus_num == pdev->bus->number) &&
                (((device_id >> VF_PF_IDENTIFIER_SHIFT) &
                  VF_PF_IDENTIFIER_MASK) == func_num))
                return param; // MD: Return the parameter for virtual functions
#endif
        }
    }

    return 0; // MD: Return 0 if no matching parameter is found
}

struct xlnx_pci_dev *xpdev_find_by_idx(unsigned int idx, char *buf, int buflen)
{
    struct xlnx_pci_dev *xpdev, *tmp;

    // MD: Lock the mutex to ensure exclusive access to the device list
    mutex_lock(&xpdev_mutex);
    // MD: Iterate over the device list to find the device with the specified index
    list_for_each_entry_safe(xpdev, tmp, &xpdev_list, list_head) {
        if (xpdev->idx == idx) {
            // MD: Unlock the mutex before returning the found device
            mutex_unlock(&xpdev_mutex);
            return xpdev;
        }
    }
    // MD: Unlock the mutex if the device is not found
    mutex_unlock(&xpdev_mutex);

    // MD: If a buffer is provided, write an error message to it
    if (buf && buflen)
        snprintf(buf, buflen, "NO device found at index %05x!\n", idx);

    return NULL; // MD: Return NULL if the device is not found
}

struct xlnx_qdata *xpdev_queue_get(struct xlnx_pci_dev *xpdev,
            unsigned int qidx, u8 q_type, bool check_qhndl,
            char *ebuf, int ebuflen)
{
    struct xlnx_qdata *qdata;

    // MD: Check if the queue index is within the valid range
    if (qidx >= xpdev->qmax) {
        pr_debug("qdma%05x QID %u too big, %05x.\n",
            xpdev->idx, qidx, xpdev->qmax);
        if (ebuf && ebuflen) {
            snprintf(ebuf, ebuflen, "QID %u too big, %u.\n",
                     qidx, xpdev->qmax);
        }
        return NULL;
    }

    // MD: Calculate the queue data pointer based on the queue type
    qdata = xpdev->qdata + qidx;
    if (q_type == Q_C2H)
        qdata += xpdev->qmax;
    if (q_type == Q_CMPT)
        qdata += (2 * xpdev->qmax);

    // MD: Check if the queue handle is valid if required
    if (check_qhndl && (!qdata->qhndl && !qdata->xcdev)) {
        pr_debug("qdma%05x QID %u NOT configured.\n", xpdev->idx, qidx);
        if (ebuf && ebuflen) {
            snprintf(ebuf, ebuflen,
                    "QID %u NOT configured.\n", qidx);
        }

        return NULL;
    }

    return qdata; // MD: Return the queue data pointer
}

int xpdev_queue_delete(struct xlnx_pci_dev *xpdev, unsigned int qidx, u8 q_type,
            char *ebuf, int ebuflen)
{
    struct xlnx_qdata *qdata = xpdev_queue_get(xpdev, qidx, q_type, 1, ebuf,
                        ebuflen);
    int rv = 0;

    // MD: Return error if the queue data is not found
    if (!qdata)
        return -EINVAL;

    // MD: Check if the queue type is not completion and the device context is valid
    if (q_type != Q_CMPT) {
        if (!qdata->xcdev)
            return -EINVAL;
    }

    // MD: Remove the queue if the handle is valid
    if (qdata->qhndl != QDMA_QUEUE_IDX_INVALID)
        rv = qdma_queue_remove(xpdev->dev_hndl, qdata->qhndl,
                    ebuf, ebuflen);
    else
        pr_err("qidx %u/%u, type %d, qhndl invalid.\n",
            qidx, xpdev->qmax, q_type);
    if (rv < 0)
        goto exit;

    // MD: Update the device context and destroy it if necessary
    if (q_type != Q_CMPT) {
        spin_lock(&xpdev->cdev_lock);
        qdata->xcdev->dir_init &= ~(1 << (q_type ? 1 : 0));
        spin_unlock(&xpdev->cdev_lock);

        if (!qdata->xcdev->dir_init)
            qdma_cdev_destroy(qdata->xcdev);
    }

    // MD: Clear the queue data
    memset(qdata, 0, sizeof(*qdata));
exit:
    return rv; // MD: Return the result of the operation
}

#if KERNEL_VERSION(3, 16, 0) <= LINUX_VERSION_CODE
static void xpdev_queue_delete_all(struct xlnx_pci_dev *xpdev)
{
    int i;

    // MD: Delete all queues for the device
    for (i = 0; i < xpdev->qmax; i++) {
        xpdev_queue_delete(xpdev, i, 0, NULL, 0);
        xpdev_queue_delete(xpdev, i, 1, NULL, 0);
    }
}
#endif

int xpdev_queue_add(struct xlnx_pci_dev *xpdev, struct qdma_queue_conf *qconf,
            char *ebuf, int ebuflen)
{
    struct xlnx_qdata *qdata;
    struct qdma_cdev *xcdev = NULL;
    struct xlnx_qdata *qdata_tmp;
    struct qdma_dev_conf dev_config;
    u8 dir;
    unsigned long qhndl;
    int rv;

    // MD: Add a new queue to the device
    rv = qdma_queue_add(xpdev->dev_hndl, qconf, &qhndl, ebuf, ebuflen);
    if (rv < 0)
        return rv;

    pr_debug("qdma%05x idx %u, st %d, q_type %s, added, qhndl 0x%lx.\n",
        xpdev->idx, qconf->qidx, qconf->st,
        q_type_list[qconf->q_type].name, qhndl);

    // MD: Get the queue data for the new queue
    qdata = xpdev_queue_get(xpdev, qconf->qidx, qconf->q_type, 0, ebuf,
                ebuflen);
    if (!qdata) {
        pr_err("q added 0x%lx, get failed, idx 0x%x.\n",
            qhndl, qconf->qidx);
        return rv;
    }

    // MD: Handle non-completion queues
    if (qconf->q_type != Q_CMPT) {
        dir = (qconf->q_type == Q_C2H) ? 0 : 1;
        spin_lock(&xpdev->cdev_lock);
        qdata_tmp = xpdev_queue_get(xpdev, qconf->qidx,
                dir, 0, NULL, 0);
        if (qdata_tmp) {
            // MD: Only one cdev per queue pair
            if (qdata_tmp->xcdev) {
                unsigned long *priv_data;

                qdata->qhndl = qhndl;
                qdata->xcdev = qdata_tmp->xcdev;
                priv_data = (qconf->q_type == Q_C2H) ?
                        &qdata->xcdev->c2h_qhndl :
                        &qdata->xcdev->h2c_qhndl;
                *priv_data = qhndl;
                qdata->xcdev->dir_init |= (1 << qconf->q_type);

                spin_unlock(&xpdev->cdev_lock);
                return 0;
            }
        }
        spin_unlock(&xpdev->cdev_lock);
    }

    // MD: Get the device configuration
    rv = qdma_device_get_config(xpdev->dev_hndl, &dev_config, NULL, 0);
    if (rv < 0) {
        pr_err("Failed to get conf for qdma device '%05x'\n",
                xpdev->idx);
        return rv;
    }

    // MD: Create the cdev for the queue
    if (qconf->q_type != Q_CMPT) {
        rv = qdma_cdev_create(&xpdev->cdev_cb, xpdev->pdev, qconf,
                (dev_config.qsets_base + qconf->qidx),
                qhndl, &xcdev, ebuf, ebuflen);

        qdata->xcdev = xcdev;
    }

    qdata->qhndl = qhndl;

    return rv; // MD: Return the result of the operation
}

static void nl_work_handler_q_start(struct work_struct *work)
{
    // MD: Retrieve the xlnx_nl_work structure from the work_struct
    struct xlnx_nl_work *nl_work = container_of(work, struct xlnx_nl_work, work);
    struct xlnx_pci_dev *xpdev = nl_work->xpdev; // MD: Get the associated PCI device
    struct xlnx_nl_work_q_ctrl *qctrl = &nl_work->qctrl; // MD: Queue control structure
    unsigned int qidx = qctrl->qidx; // MD: Starting queue index
    u8 is_qp = qctrl->is_qp; // MD: Flag indicating if it's a queue pair
    u8 q_type = qctrl->q_type; // MD: Queue type
    int i;
    char *ebuf = nl_work->buf; // MD: Error buffer
    int rv = 0; // MD: Return value

    // MD: Iterate over the number of queues to start
    for (i = 0; i < qctrl->qcnt; i++, qidx++) {
        struct xlnx_qdata *qdata;

    q_start:
        // MD: Retrieve queue data for the specified index and type
        qdata = xpdev_queue_get(xpdev, qidx, q_type, 1, ebuf, nl_work->buflen);
        if (!qdata) {
            pr_err("%s, idx %u, q_type %s, get failed.\n",
                dev_name(&xpdev->pdev->dev), qidx, q_type_list[q_type].name);
            snprintf(ebuf, nl_work->buflen,
                "Q idx %u, q_type %s, get failed.\n", qidx, q_type_list[q_type].name);
            goto send_resp;
        }

        // MD: Start the queue
        rv = qdma_queue_start(xpdev->dev_hndl, qdata->qhndl, ebuf, nl_work->buflen);
        if (rv < 0) {
            pr_err("%s, idx %u, q_type %s, start failed %d.\n",
                dev_name(&xpdev->pdev->dev), qidx, q_type_list[q_type].name, rv);
            snprintf(ebuf, nl_work->buflen,
                "Q idx %u, q_type %s, start failed %d.\n", qidx, q_type_list[q_type].name, rv);
            goto send_resp;
        }

        // MD: Handle queue pairs
        if (qctrl->q_type != Q_CMPT) {
            if (is_qp && q_type == qctrl->q_type) {
                q_type = !qctrl->q_type; // MD: Toggle queue type
                goto q_start; // MD: Restart with the new queue type
            }
            q_type = qctrl->q_type; // MD: Reset queue type
        }
    }

    // MD: Format success message
    snprintf(ebuf, nl_work->buflen, "%u Queues started, idx %u ~ %u.\n",
        qctrl->qcnt, qctrl->qidx, qidx - 1);

send_resp:
    nl_work->q_start_handled = 1; // MD: Mark work as handled
    nl_work->ret = rv; // MD: Set return value
    wake_up_interruptible(&nl_work->wq); // MD: Wake up waiting processes
}

static struct xlnx_nl_work *xpdev_nl_work_alloc(struct xlnx_pci_dev *xpdev)
{
    struct xlnx_nl_work *nl_work;

    // MD: Allocate memory for the work structure
    nl_work = kzalloc(sizeof(*nl_work), GFP_ATOMIC);
    if (!nl_work) {
        pr_err("qdma%05x %s: OOM.\n", xpdev->idx, dev_name(&xpdev->pdev->dev));
        return NULL;
    }

    nl_work->xpdev = xpdev; // MD: Associate the work with the PCI device

    return nl_work;
}

int xpdev_nl_queue_start(struct xlnx_pci_dev *xpdev, void *nl_info, u8 is_qp,
            u8 q_type, unsigned short qidx, unsigned short qcnt)
{
    struct xlnx_nl_work *nl_work = xpdev_nl_work_alloc(xpdev);
    struct xlnx_nl_work_q_ctrl *qctrl;
    char ebuf[XNL_EBUFLEN];
    int rv = 0;

    if (!nl_work)
        return -ENOMEM;

    qctrl = &nl_work->qctrl;
    qctrl->is_qp = is_qp;
    qctrl->q_type = q_type;
    qctrl->qidx = qidx;
    qctrl->qcnt = qcnt;

    INIT_WORK(&nl_work->work, nl_work_handler_q_start);
    init_waitqueue_head(&nl_work->wq);
    nl_work->q_start_handled = 0;
    nl_work->buf = ebuf;
    nl_work->buflen = XNL_EBUFLEN;
    queue_work(xpdev->nl_task_wq, &nl_work->work);
    wait_event_interruptible(nl_work->wq, nl_work->q_start_handled);
    rv = nl_work->ret;
    kfree(nl_work);
    xnl_respond_buffer(nl_info, ebuf, strlen(ebuf), rv);

    return rv;
}

static void xpdev_free(struct xlnx_pci_dev *p)
{
    xpdev_list_remove(p); // MD: Remove device from list

    if (p->nl_task_wq)
        destroy_workqueue(p->nl_task_wq); // MD: Destroy workqueue if it exists

    kfree(p->qdata); // MD: Free queue data
    kfree(p); // MD: Free device structure
}

static int xpdev_qdata_realloc(struct xlnx_pci_dev *xpdev, unsigned int qmax)
{
    if (!xpdev)
        return -EINVAL;

    kfree(xpdev->qdata); // MD: Free existing queue data
    xpdev->qdata = NULL;

    if (!qmax)
        return 0;

    // MD: Allocate new queue data
    xpdev->qdata = kzalloc(qmax * 3 * sizeof(struct xlnx_qdata), GFP_KERNEL);
    if (!xpdev->qdata) {
        pr_err("OMM, xpdev->qdata, sz %u.\n", qmax);
        return -ENOMEM;
    }
    xpdev->qmax = qmax; // MD: Update maximum queue count

    return 0;
}

static struct xlnx_pci_dev *xpdev_alloc(struct pci_dev *pdev, unsigned int qmax)
{
    int sz = sizeof(struct xlnx_pci_dev); // MD: Calculate the size of the xlnx_pci_dev structure
    struct xlnx_pci_dev *xpdev = kzalloc(sz, GFP_KERNEL); // MD: Allocate memory for the device structure
    char name[80]; // MD: Buffer to hold the name for the workqueue

    // MD: Check if memory allocation was successful
    if (!xpdev) {
        xpdev = vmalloc(sz); // MD: Try to allocate memory using vmalloc if kzalloc fails
        if (xpdev)
            memset(xpdev, 0, sz); // MD: Zero out the allocated memory
    }

    // MD: If memory allocation failed, log an error and return NULL
    if (!xpdev) {
        pr_err("OMM, qmax %u, sz %u.\n", qmax, sz);
        return NULL;
    }

    spin_lock_init(&xpdev->cdev_lock); // MD: Initialize the spinlock for the device
    xpdev->pdev = pdev; // MD: Assign the PCI device to the xlnx_pci_dev structure
    xpdev->qmax = qmax; // MD: Set the maximum number of queues
    xpdev->idx = 0xFF; // MD: Initialize the device index

    // MD: Reallocate queue data if qmax is greater than zero
    if (qmax && (xpdev_qdata_realloc(xpdev, qmax) < 0))
        goto free_xpdev; // MD: If reallocation fails, free the device and return NULL

    // MD: Create a single-threaded workqueue for the device
    snprintf(name, 80, "qdma_%s_nl_wq", dev_name(&pdev->dev));
    xpdev->nl_task_wq = create_singlethread_workqueue(name);
    if (!xpdev->nl_task_wq) {
        pr_err("%s failed to allocate nl_task_wq.\n", dev_name(&pdev->dev));
        goto free_xpdev; // MD: If workqueue creation fails, free the device and return NULL
    }

    xpdev_list_add(xpdev); // MD: Add the device to the global list
    return xpdev; // MD: Return the allocated device

free_xpdev:
    xpdev_free(xpdev); // MD: Free the allocated device
    return NULL; // MD: Return NULL to indicate failure
}

static int xpdev_map_bar(struct xlnx_pci_dev *xpdev, void __iomem **regs, u8 bar_num)
{
    int map_len;

    // MD: Determine the length of the BAR to map
    map_len = pci_resource_len(xpdev->pdev, (int)bar_num);
    if (map_len > QDMA_MAX_BAR_LEN_MAPPED)
        map_len = QDMA_MAX_BAR_LEN_MAPPED; // MD: Limit the mapping length to a predefined maximum

    // MD: Map the BAR into kernel virtual address space
    *regs = pci_iomap(xpdev->pdev, bar_num, map_len);
    if (!(*regs)) {
        pr_err("unable to map bar %d.\n", bar_num); // MD: Log an error if mapping fails
        return -ENOMEM; // MD: Return error code for memory allocation failure
    }

    return 0; // MD: Return success
}

static void xpdev_unmap_bar(struct xlnx_pci_dev *xpdev, void __iomem **regs)
{
    // MD: Unmap the BAR if it is currently mapped
    if (*regs) {
        pci_iounmap(xpdev->pdev, *regs); // MD: Unmap the BAR
        *regs = NULL; // MD: Mark the BAR as unmapped
    }
}

int qdma_device_read_user_register(struct xlnx_pci_dev *xpdev, u32 reg_addr, u32 *value)
{
    struct xlnx_dma_dev *xdev = NULL;
    int rv = 0;

    if (!xpdev)
        return -EINVAL; // MD: Return error if the device is not valid

    xdev = (struct xlnx_dma_dev *)(xpdev->dev_hndl);

    if (xdev->conf.bar_num_user < 0) {
        pr_err("AXI Master Lite bar is not present\n"); // MD: Log an error if the BAR is not present
        return -EINVAL;
    }

    // MD: Map the AXI Master Lite bar
    rv = xpdev_map_bar(xpdev, &xpdev->user_bar_regs, xdev->conf.bar_num_user);
    if (rv < 0)
        return rv; // MD: Return error if mapping fails

    *value = readl(xpdev->user_bar_regs + reg_addr); // MD: Read the register value

    // MD: Unmap the AXI Master Lite bar after accessing it
    xpdev_unmap_bar(xpdev, &xpdev->user_bar_regs);

    return 0; // MD: Return success
}

int qdma_device_write_user_register(struct xlnx_pci_dev *xpdev, u32 reg_addr, u32 value)
{
    struct xlnx_dma_dev *xdev = NULL;
    int rv = 0;

    if (!xpdev)
        return -EINVAL; // MD: Return error if the device is not valid

    xdev = (struct xlnx_dma_dev *)(xpdev->dev_hndl);

    if (xdev->conf.bar_num_user < 0) {
        pr_err("AXI Master Lite bar is not present\n"); // MD: Log an error if the BAR is not present
        return -EINVAL;
    }

    // MD: Map the AXI Master Lite bar
    rv = xpdev_map_bar(xpdev, &xpdev->user_bar_regs, xdev->conf.bar_num_user);
    if (rv < 0)
        return rv; // MD: Return error if mapping fails

    writel(value, xpdev->user_bar_regs + reg_addr); // MD: Write the value to the register

    // MD: Unmap the AXI Master Lite bar after accessing it
    xpdev_unmap_bar(xpdev, &xpdev->user_bar_regs);

    return 0; // MD: Return success
}

int qdma_device_read_bypass_register(struct xlnx_pci_dev *xpdev, u32 reg_addr, u32 *value)
{
    struct xlnx_dma_dev *xdev = NULL;
    int rv = 0;

    if (!xpdev)
        return -EINVAL; // MD: Return error if the device is not valid

    xdev = (struct xlnx_dma_dev *)(xpdev->dev_hndl);

    if (xdev->conf.bar_num_bypass < 0) {
        pr_err("AXI Bridge Master bar is not present\n"); // MD: Log an error if the BAR is not present
        return -EINVAL;
    }

    // MD: Map the AXI Bridge Master bar
    rv = xpdev_map_bar(xpdev, &xpdev->bypass_bar_regs, xdev->conf.bar_num_bypass);
    if (rv < 0)
        return rv; // MD: Return error if mapping fails

    *value = readl(xpdev->bypass_bar_regs + reg_addr); // MD: Read the register value

    // MD: Unmap the AXI Bridge Master bar after accessing it
    xpdev_unmap_bar(xpdev, &xpdev->bypass_bar_regs);

    return 0; // MD: Return success
}

int qdma_device_write_bypass_register(struct xlnx_pci_dev *xpdev,
		u32 reg_addr, u32 value)
{
	struct xlnx_dma_dev *xdev = NULL; // MD: Pointer to the DMA device structure
	int rv = 0; // MD: Variable to store return values

	if (!xpdev) // MD: Check if the device structure is valid
		return -EINVAL; // MD: Return error if invalid

	xdev = (struct xlnx_dma_dev *)(xpdev->dev_hndl); // MD: Retrieve the DMA device handle

	if (xdev->conf.bar_num_bypass < 0) { // MD: Check if the bypass BAR is configured
		pr_err("AXI Bridge Master bar is not present\n"); // MD: Log error
		return -EINVAL; // MD: Return error if not configured
	}

	// MD: Map the AXI Bridge Master BAR for register access
	rv = xpdev_map_bar(xpdev, &xpdev->bypass_bar_regs, xdev->conf.bar_num_bypass);
	if (rv < 0) // MD: Check if mapping was successful
		return rv; // MD: Return error if mapping failed

	// MD: Write the value to the specified register address
	writel(value, xpdev->bypass_bar_regs + reg_addr);

	// MD: Unmap the AXI Bridge Master BAR after accessing it
	xpdev_unmap_bar(xpdev, &xpdev->bypass_bar_regs);

	return 0; // MD: Return success
}

static int probe_one(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct qdma_dev_conf conf; // MD: Configuration structure for the QDMA device
	struct xlnx_pci_dev *xpdev = NULL; // MD: Pointer to the PCI device structure
	unsigned long dev_hndl; // MD: Device handle
	int rv; // MD: Variable to store return values

#ifdef __x86_64__
	pr_info("%s: func 0x%x, p/v %d/%d,0x%p.\n",
		dev_name(&pdev->dev), PCI_FUNC(pdev->devfn),
		pdev->is_physfn, pdev->is_virtfn, pdev->physfn); // MD: Log device info
#endif

	memset(&conf, 0, sizeof(conf)); // MD: Initialize the configuration structure

	// MD: Extract driver mode from module parameters
	conf.qdma_drv_mode = (enum qdma_drv_mode)extract_mod_param(pdev, DRV_MODE);
	conf.vf_max = 0; // MD: Set maximum virtual functions to 0

#ifndef __QDMA_VF__
	// MD: Extract master physical function from module parameters
	conf.master_pf = extract_mod_param(pdev, MASTER_PF);
	if (conf.master_pf)
		pr_info("Configuring '%02x:%02x:%x' as master pf\n",
				pdev->bus->number,
				PCI_SLOT(pdev->devfn),
				PCI_FUNC(pdev->devfn)); // MD: Log master PF configuration
#endif

	pr_info("Driver is loaded in %s(%d) mode\n",
				mode_name_list[conf.qdma_drv_mode].name,
				conf.qdma_drv_mode); // MD: Log driver mode

	if (conf.qdma_drv_mode == LEGACY_INTR_MODE)
		intr_legacy_init(); // MD: Initialize legacy interrupt mode if applicable

	conf.intr_rngsz = QDMA_INTR_COAL_RING_SIZE; // MD: Set interrupt ring size
	conf.pdev = pdev; // MD: Assign PCI device to configuration

	// MD: Initialize all BAR numbers to -1
	conf.bar_num_config = -1;
	conf.bar_num_user = -1;
	conf.bar_num_bypass = -1;

	// MD: Extract configuration BAR number from module parameters
	conf.bar_num_config = extract_mod_param(pdev, CONFIG_BAR);
	conf.qsets_max = 0; // MD: Set maximum queue sets to 0
	conf.qsets_base = -1; // MD: Set base queue set to -1
	conf.msix_qvec_max = 32; // MD: Set maximum MSI-X vectors
	conf.user_msix_qvec_max = 1; // MD: Set maximum user MSI-X vectors

#ifdef __QDMA_VF__
	conf.fp_flr_free_resource = qdma_flr_resource_free; // MD: Assign resource free function for VF
#endif

	if (conf.master_pf)
		conf.data_msix_qvec_max = 5; // MD: Set data MSI-X vectors for master PF
	else
		conf.data_msix_qvec_max = 6; // MD: Set data MSI-X vectors for non-master PF

	// MD: Open the QDMA device with the specified configuration
	rv = qdma_device_open(DRV_MODULE_NAME, &conf, &dev_hndl);
	if (rv < 0)
		return rv; // MD: Return error if device open fails

	// MD: Allocate and initialize the PCI device structure
	xpdev = xpdev_alloc(pdev, conf.qsets_max);
	if (!xpdev) {
		rv = -EINVAL; // MD: Set error code for invalid argument
		goto close_device; // MD: Jump to device close on failure
	}

	xpdev->dev_hndl = dev_hndl; // MD: Assign device handle
	xpdev->idx = conf.bdf; // MD: Assign bus-device-function index

	xpdev->cdev_cb.xpdev = xpdev; // MD: Assign PCI device to callback structure
	rv = qdma_cdev_device_init(&xpdev->cdev_cb); // MD: Initialize character device
	if (rv < 0)
		goto close_device; // MD: Jump to device close on failure

	// MD: Create sysfs attribute files
	if (conf.master_pf) {
		rv = sysfs_create_group(&pdev->dev.kobj, &pci_master_device_attr_group);
		if (rv < 0)
			goto close_device; // MD: Jump to device close on failure
	} else {
		rv = sysfs_create_group(&pdev->dev.kobj, &pci_device_attr_group);
		if (rv < 0)
			goto close_device; // MD: Jump to device close on failure
	}

	dev_set_drvdata(&pdev->dev, xpdev); // MD: Set driver data for the PCI device

	return 0; // MD: Return success

close_device:
	qdma_device_close(pdev, dev_hndl); // MD: Close the QDMA device

	if (xpdev)
		xpdev_free(xpdev); // MD: Free the PCI device structure

	return rv; // MD: Return error code
}

static void xpdev_device_cleanup(struct xlnx_pci_dev *xpdev)
{
	struct xlnx_qdata *qdata = xpdev->qdata; // MD: Pointer to queue data
	struct xlnx_qdata *qmax = qdata + (xpdev->qmax * 2); // MD: End of queue data (h2c and c2h)

	for (; qdata != qmax; qdata++) { // MD: Iterate over all queue data
		if (qdata->xcdev) { // MD: Check if the queue data has a character device
			// MD: Check if either h2c(1) or c2h(2) bit is set, but not both
			if (qdata->xcdev->dir_init == 1 || qdata->xcdev->dir_init == 2) {
				qdma_cdev_destroy(qdata->xcdev); // MD: Destroy the character device
			} else { // MD: Both bits are set, so remove one
				spin_lock(&xpdev->cdev_lock); // MD: Acquire lock
				qdata->xcdev->dir_init >>= 1; // MD: Shift bits to remove one
				spin_unlock(&xpdev->cdev_lock); // MD: Release lock
			}
		}
		memset(qdata, 0, sizeof(*qdata)); // MD: Clear the queue data
	}
}

static void remove_one(struct pci_dev *pdev)
{
    struct xlnx_dma_dev *xdev = NULL;
    struct xlnx_pci_dev *xpdev = dev_get_drvdata(&pdev->dev);

    // MD: Check if the device-specific data is available
    if (!xpdev) {
        pr_info("%s NOT attached.\n", dev_name(&pdev->dev));
        return;
    }

    // MD: Retrieve the DMA device handle from the PCI device structure
    xdev = (struct xlnx_dma_dev *)(xpdev->dev_hndl);

    // MD: Log the device information
    pr_info("%s pdev 0x%p, xdev 0x%p, hndl 0x%lx, qdma%05x.\n",
        dev_name(&pdev->dev), pdev, xpdev, xpdev->dev_hndl, xpdev->idx);

    // MD: Remove the appropriate sysfs group based on whether it's a master PF
    if (xdev->conf.master_pf)
        sysfs_remove_group(&pdev->dev.kobj, &pci_master_device_attr_group);
    else
        sysfs_remove_group(&pdev->dev.kobj, &pci_device_attr_group);

    // MD: Clean up the character device associated with the QDMA
    qdma_cdev_device_cleanup(&xpdev->cdev_cb);

    // MD: Clean up the device-specific data
    xpdev_device_cleanup(xpdev);

    // MD: Close the QDMA device
    qdma_device_close(pdev, xpdev->dev_hndl);

    // MD: Free the allocated memory for the PCI device structure
    xpdev_free(xpdev);

    // MD: Clear the device-specific data from the PCI device structure
    dev_set_drvdata(&pdev->dev, NULL);
}

#if defined(CONFIG_PCI_IOV) && !defined(__QDMA_VF__)
static int sriov_config(struct pci_dev *pdev, int num_vfs)
{
    struct xlnx_pci_dev *xpdev = dev_get_drvdata(&pdev->dev);
    int total_vfs = pci_sriov_get_totalvfs(pdev);

    // MD: Check if the device-specific data is available
    if (!xpdev) {
        pr_info("%s NOT attached.\n", dev_name(&pdev->dev));
        return -EINVAL;
    }

    // MD: Log the device information
    pr_debug("%s pdev 0x%p, xdev 0x%p, hndl 0x%lx, qdma%05x.\n",
        dev_name(&pdev->dev), pdev, xpdev, xpdev->dev_hndl, xpdev->idx);

    // MD: Adjust the number of VFs if it exceeds the total available
    if (num_vfs > total_vfs) {
        pr_info("%s, clamp down # of VFs %d -> %d.\n",
            dev_name(&pdev->dev), num_vfs, total_vfs);
        num_vfs = total_vfs;
    }

    // MD: Configure the SR-IOV for the QDMA device
    return qdma_device_sriov_config(pdev, xpdev->dev_hndl, num_vfs);
}
#endif

static pci_ers_result_t qdma_error_detected(struct pci_dev *pdev,
                                            pci_channel_state_t state)
{
    struct xlnx_pci_dev *xpdev = dev_get_drvdata(&pdev->dev);

    switch (state) {
    case pci_channel_io_normal:
        return PCI_ERS_RESULT_CAN_RECOVER;
    case pci_channel_io_frozen:
        pr_warn("dev 0x%p,0x%p, frozen state error, reset controller\n",
                pdev, xpdev);
        pci_disable_device(pdev);
        return PCI_ERS_RESULT_NEED_RESET;
    case pci_channel_io_perm_failure:
        pr_warn("dev 0x%p,0x%p, failure state error, req. disconnect\n",
                pdev, xpdev);
        return PCI_ERS_RESULT_DISCONNECT;
    }
    return PCI_ERS_RESULT_NEED_RESET;
}

static pci_ers_result_t qdma_slot_reset(struct pci_dev *pdev)
{
    struct xlnx_pci_dev *xpdev = dev_get_drvdata(&pdev->dev);

    // MD: Check if the device-specific data is available
    if (!xpdev) {
        pr_info("%s NOT attached.\n", dev_name(&pdev->dev));
        return PCI_ERS_RESULT_DISCONNECT;
    }

    // MD: Log the restart information
    pr_info("0x%p restart after slot reset\n", xpdev);
    if (pci_enable_device_mem(pdev)) {
        pr_info("0x%p failed to renable after slot reset\n", xpdev);
        return PCI_ERS_RESULT_DISCONNECT;
    }

    // MD: Restore PCI device state
    pci_set_master(pdev);
    pci_restore_state(pdev);
    pci_save_state(pdev);

    return PCI_ERS_RESULT_RECOVERED;
}

static void qdma_error_resume(struct pci_dev *pdev)
{
    struct xlnx_pci_dev *xpdev = dev_get_drvdata(&pdev->dev);

    // MD: Check if the device-specific data is available
    if (!xpdev) {
        pr_info("%s NOT attached.\n", dev_name(&pdev->dev));
        return;
    }

    // MD: Log the resume information
    pr_info("dev 0x%p,0x%p.\n", pdev, xpdev);

    // MD: Clear non-fatal AER status based on kernel version
#ifdef RHEL_RELEASE_VERSION
#if RHEL_RELEASE_VERSION(8, 3) > RHEL_RELEASE_CODE
    pci_cleanup_aer_uncorrect_error_status(pdev);
#else
    pci_aer_clear_nonfatal_status(pdev);
#endif
#else
#if KERNEL_VERSION(5, 7, 0) <= LINUX_VERSION_CODE
    pci_aer_clear_nonfatal_status(pdev);
#else
    pci_cleanup_aer_uncorrect_error_status(pdev);
#endif
#endif
}

#ifdef __QDMA_VF__
void qdma_flr_resource_free(unsigned long dev_hndl)
{
    // MD: Cast the device handle to a DMA device structure
    struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)(dev_hndl);
    // MD: Retrieve the PCI device structure from the DMA device
    struct xlnx_pci_dev *xpdev = dev_get_drvdata(&xdev->conf.pdev->dev);

    // MD: Clean up the device resources
    xpdev_device_cleanup(xpdev);
    // MD: Reset queue set configurations
    xdev->conf.qsets_max = 0;
    xdev->conf.qsets_base = -1;
}
#endif

#if KERNEL_VERSION(4, 13, 0) <= LINUX_VERSION_CODE
static void qdma_reset_prepare(struct pci_dev *pdev)
{
    // MD: Retrieve the PCI device structure
    struct xlnx_pci_dev *xpdev = dev_get_drvdata(&pdev->dev);
    struct xlnx_dma_dev *xdev = NULL;

    // MD: Cast the device handle to a DMA device structure
    xdev = (struct xlnx_dma_dev *)(xpdev->dev_hndl);
    pr_info("%s pdev 0x%p, xdev 0x%p, hndl 0x%lx, qdma%05x.\n",
        dev_name(&pdev->dev), pdev, xpdev, xpdev->dev_hndl, xpdev->idx);

    // MD: Take the device offline for reset preparation
    qdma_device_offline(pdev, xpdev->dev_hndl, XDEV_FLR_ACTIVE);

    // MD: Check for specific IP type and device type for FLR quirks
    if ((xdev->version_info.ip_type == QDMA_VERSAL_HARD_IP) &&
        (xdev->version_info.device_type == QDMA_DEVICE_VERSAL_CPM4))
        qdma_device_flr_quirk_set(pdev, xpdev->dev_hndl);

    // MD: Delete all queues and clean up the device
    xpdev_queue_delete_all(xpdev);
    xpdev_device_cleanup(xpdev);
    xdev->conf.qsets_max = 0;
    xdev->conf.qsets_base = -1;

    // MD: Check for FLR quirks again after cleanup
    if ((xdev->version_info.ip_type == QDMA_VERSAL_HARD_IP) &&
        (xdev->version_info.device_type == QDMA_DEVICE_VERSAL_CPM4))
        qdma_device_flr_quirk_check(pdev, xpdev->dev_hndl);
}

static void qdma_reset_done(struct pci_dev *pdev)
{
    // MD: Retrieve the PCI device structure
    struct xlnx_pci_dev *xpdev = dev_get_drvdata(&pdev->dev);

    pr_info("%s pdev 0x%p, xdev 0x%p, hndl 0x%lx, qdma%05x.\n",
        dev_name(&pdev->dev), pdev, xpdev, xpdev->dev_hndl, xpdev->idx);
    // MD: Bring the device back online after reset
    qdma_device_online(pdev, xpdev->dev_hndl, XDEV_FLR_ACTIVE);
}

#elif KERNEL_VERSION(3, 16, 0) <= LINUX_VERSION_CODE
static void qdma_reset_notify(struct pci_dev *pdev, bool prepare)
{
    // MD: Retrieve the PCI device structure
    struct xlnx_pci_dev *xpdev = dev_get_drvdata(&pdev->dev);
    struct xlnx_dma_dev *xdev = NULL;

    // MD: Cast the device handle to a DMA device structure
    xdev = (struct xlnx_dma_dev *)(xpdev->dev_hndl);

    pr_info("%s prepare %d, pdev 0x%p, xdev 0x%p, hndl 0x%lx, qdma%05x.\n",
        dev_name(&pdev->dev), prepare, pdev, xpdev, xpdev->dev_hndl,
        xpdev->idx);

    if (prepare) {
        // MD: Take the device offline for reset preparation
        qdma_device_offline(pdev, xpdev->dev_hndl, XDEV_FLR_ACTIVE);
        // MD: Check for specific IP type and device type for FLR quirks
        if ((xdev->version_info.ip_type == QDMA_VERSAL_HARD_IP) &&
            (xdev->version_info.device_type == QDMA_DEVICE_VERSAL_CPM4))
            qdma_device_flr_quirk_set(pdev, xpdev->dev_hndl);

        // MD: Delete all queues and clean up the device
        xpdev_queue_delete_all(xpdev);
        xpdev_device_cleanup(xpdev);
        xdev->conf.qsets_max = 0;
        xdev->conf.qsets_base = -1;

        // MD: Check for FLR quirks again after cleanup
        if ((xdev->version_info.ip_type == QDMA_VERSAL_HARD_IP) &&
            (xdev->version_info.device_type == QDMA_DEVICE_VERSAL_CPM4))
            qdma_device_flr_quirk_check(pdev, xpdev->dev_hndl);
    } else {
        // MD: Bring the device back online after reset
        qdma_device_online(pdev, xpdev->dev_hndl, XDEV_FLR_ACTIVE);
    }
}
#endif

// MD: Define PCI error handlers
static const struct pci_error_handlers qdma_err_handler = {
    .error_detected = qdma_error_detected,
    .slot_reset = qdma_slot_reset,
    .resume = qdma_error_resume,
#if KERNEL_VERSION(4, 13, 0) <= LINUX_VERSION_CODE
    .reset_prepare = qdma_reset_prepare,
    .reset_done = qdma_reset_done,
#elif KERNEL_VERSION(3, 16, 0) <= LINUX_VERSION_CODE
    .reset_notify = qdma_reset_notify,
#endif
};

// MD: Define the PCI driver structure
static struct pci_driver pci_driver = {
    .name = DRV_MODULE_NAME,
    .id_table = pci_ids,
    .probe = probe_one,
    .remove = remove_one,
    /* MD: .shutdown = shutdown_one, */
#if defined(CONFIG_PCI_IOV) && !defined(__QDMA_VF__)
    .sriov_configure = sriov_config,
#endif
    .err_handler = &qdma_err_handler,
};

// MD: Module initialization function
static int __init qdma_mod_init(void)
{
    int rv;

    pr_info("%s", version);

    // MD: Initialize the QDMA library
    rv = libqdma_init(num_threads, NULL);
    if (rv < 0)
        return rv;

    // MD: Initialize the Xilinx Netlink interface
    rv = xlnx_nl_init();
    if (rv < 0)
        return rv;

    // MD: Initialize the QDMA character device
    rv = qdma_cdev_init();
    if (rv < 0)
        return rv;

    // MD: Register the PCI driver
    return pci_register_driver(&pci_driver);
}

// MD: Module exit function
static void __exit qdma_mod_exit(void)
{
    // MD: Unregister the PCI driver
    pci_unregister_driver(&pci_driver);

    // MD: Cleanup the Xilinx Netlink interface
    xlnx_nl_exit();

    // MD: Cleanup the QDMA character device
    qdma_cdev_cleanup();

    // MD: Exit the QDMA library
    libqdma_exit();
}

// MD: Register module initialization and exit functions
module_init(qdma_mod_init);
module_exit(qdma_mod_exit);





