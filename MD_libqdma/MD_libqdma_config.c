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
 */

/* MD:*
 * @file
 * @brief This file contains the definitions for qdma configuration APIs
 *
 */
#define pr_fmt(fmt)	KBUILD_MODNAME ":%s: " fmt, __func__

#include "libqdma_export.h"
#include "qdma_descq.h"
#include "qdma_device.h"
#include "qdma_thread.h"
#include "qdma_regs.h"
#include "qdma_context.h"
#include "qdma_intr.h"
#include "thread.h"
#include "version.h"
#include "qdma_resource_mgmt.h"

/* MD:****************************************************************************/
/* MD:*
 * qdma_set_qmax() - Handler function to set the qmax configuration value
 *
 * @param[in]	dev_hndl:	qdma device handle
 * @param[in]	qsets_max:	qmax configuration value
 * @param[in]	forced:	whether to force set the value
 *
 * @return	0 on success
 * @return	< 0 on failure
 *****************************************************************************/
int qdma_set_qmax(unsigned long dev_hndl, int qbase, u32 qsets_max)
{
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;
	int rv = 0;

	/* MD: Check if the device handle is valid */
	if (!xdev) {
		pr_err("xdev is invalid");
		return -EINVAL;
	}

	/* MD: Debug: Log the initial qbase and qsets_max values */
	pr_info("Setting qmax with qbase: %d, qsets_max: %u", qbase, qsets_max);

	/* MD: Update the device with requested qmax and qbase */
	rv = qdma_dev_update(xdev->dma_device_index, xdev->func_id, qsets_max, &qbase);
	if (rv < 0) {
		pr_err("Failed to update dev entry, err = %d", rv);
		return -EINVAL;
	}

	/* MD: Clean up the device configuration */
	qdma_device_cleanup(xdev);

	/* MD: Retrieve the updated queue information */
	rv = qdma_dev_qinfo_get(xdev->dma_device_index, xdev->func_id, &qbase, &qsets_max);
	if (rv < 0) {
		pr_err("Failed to get qinfo, err = %d", rv);
		return -EINVAL;
	}

	/* MD: Update the device configuration with new values */
	xdev->conf.qsets_max = qsets_max;
	xdev->conf.qsets_base = qbase;

	/* MD: Initialize the device with the new configuration */
	rv = qdma_device_init(xdev);
	if (rv < 0) {
		pr_warn("qdma_init failed, err = %d", rv);
		qdma_device_cleanup(xdev);
		return -EINVAL;
	}

	/* MD: Debug: Log the successful configuration */
	pr_info("Successfully set qmax with qbase: %d, qsets_max: %u", qbase, qsets_max);

	return 0;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_get_qmax() - Handler function to get the qmax configuration value
 *
 * @param[in]	dev_hndl:	qdma device handle
 *
 * @return	qmax value on success
 * @return	< 0 on failure
 *****************************************************************************/
unsigned int qdma_get_qmax(unsigned long dev_hndl)
{
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;

	/* MD: Check if the device handle is valid */
	if (!xdev) {
		pr_err("xdev is invalid");
		return -EINVAL;
	}

	/* MD: Debug: Log the current qsets_max value */
	pr_info("Getting qmax, current qsets_max: %u", xdev->conf.qsets_max);

	/* MD: Return the current qsets_max value of the device */
	return xdev->conf.qsets_max;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_set_intr_rngsz() - Handler function to set the intr_ring_size value
 *
 * @param[in]	dev_hndl:		qdma device handle
 * @param[in]	intr_rngsz:		interrupt aggregation ring size
 *
 * @return	0 on success
 * @return	<0 on failure
 *****************************************************************************/
int qdma_set_intr_rngsz(unsigned long dev_hndl, u32 intr_rngsz)
{
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;
	int rv = -1;

	/* MD: Check if the device handle is valid */
	if (!xdev) {
		pr_err("xdev is invalid");
		return -EINVAL;
	}

	/* MD: Check if the new intr_rngsz is the same as the current one */
	if (intr_rngsz == xdev->conf.intr_rngsz) {
		pr_err("xdev 0x%p, Current intr_rngsz is same as [%d]. Nothing to be done\n",
					xdev, intr_rngsz);
		return rv;
	}

	/* MD: Check if interrupt aggregation is enabled */
	if ((xdev->conf.qdma_drv_mode != INDIRECT_INTR_MODE) &&
			(xdev->conf.qdma_drv_mode != AUTO_MODE)) {
		pr_err("xdev 0x%p, interrupt aggregation is disabled\n", xdev);
		return rv;
	}

	/* MD: Check if active queues exist, which would prevent changing intr_rngsz */
	if (qdma_get_active_queue_count(xdev->conf.pdev->bus->number)) {
		pr_err("xdev 0x%p, FMAP prog done, cannot modify intr ring size [%d]\n",
				xdev, xdev->conf.intr_rngsz);
		return rv;
	}

	/* MD: Validate the intr_rngsz value */
	if (intr_rngsz > QDMA_INDIRECT_INTR_RING_SIZE_32KB) {
		pr_err("Invalid intr ring size\n");
		return rv;
	}

	/* MD: Update the intr_rngsz if FMAP programming is not done yet */
	qdma_device_interrupt_cleanup(xdev);
	xdev->conf.intr_rngsz = intr_rngsz;
	qdma_device_interrupt_setup(xdev);

	pr_info("Successfully set intr_rngsz to %d for xdev 0x%p\n", intr_rngsz, xdev);
	return 0;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_get_intr_rngsz() - Handler function to get the intr_ring_size value
 *
 * @param[in]	dev_hndl:	qdma device handle
 *
 * @return	interrupt ring size on success
 * @return	<0 on failure
 *****************************************************************************/
unsigned int qdma_get_intr_rngsz(unsigned long dev_hndl)
{
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;

	/* MD: Check if the device handle is valid */
	if (!xdev) {
		pr_err("xdev is invalid");
		return -EINVAL;
	}

	/* MD: Check if interrupt aggregation is enabled */
	if ((xdev->conf.qdma_drv_mode != INDIRECT_INTR_MODE) &&
			(xdev->conf.qdma_drv_mode != AUTO_MODE)) {
		pr_info("xdev 0x%p, interrupt aggregation is disabled\n", xdev);
		return 0;
	}

	/* MD: Log and return the current intr_rngsz value */
	pr_info("xdev 0x%p, intr ring_size = %d\n", xdev, xdev->conf.intr_rngsz);
	return xdev->conf.intr_rngsz;
}

#ifndef __QDMA_VF__
#ifdef QDMA_CSR_REG_UPDATE
/* MD:****************************************************************************/
/* MD:*
 * qdma_set_buf_sz() - Handler function to set the buf_sz value
 *
 * @param[in]	dev_hndl:		qdma device handle
 * @param[in]	buf_sz:		interrupt aggregation ring size
 *
 * @return	0 on success
 * @return	<0 on failure
 *****************************************************************************/
int qdma_set_buf_sz(unsigned long dev_hndl, u32 *buf_sz)
{
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;
	int rv = -1;

	/* MD: Check if the device handle is valid */
	if (!xdev) {
		pr_err("xdev is invalid");
		return -EINVAL;
	}

	/* MD: Check if active queues exist, which would prevent changing buf_sz */
	if (qdma_get_active_queue_count(xdev->conf.pdev->bus->number)) {
		pr_err("xdev 0x%p, FMAP prog done, cannot modify buf size\n", xdev);
		return rv;
	}

	/* MD: Write the given buffer sizes to the registers */
	rv = xdev->hw.qdma_global_csr_conf(xdev, 0, QDMA_GLOBAL_CSR_ARRAY_SZ,
			buf_sz, QDMA_CSR_BUF_SZ, QDMA_HW_ACCESS_WRITE);
	if (rv < 0) {
		pr_err("set global buffer size failed, err = %d", rv);
		return xdev->hw.qdma_get_error_code(rv);
	}

	qdma_csr_read(xdev, &xdev->csr_info);
	pr_info("Successfully set buffer size for xdev 0x%p\n", xdev);
	return rv;
}
#endif

/* MD:****************************************************************************/
/* MD:*
 * qdma_get_buf_sz() - Handler function to get the buf_sz value
 *
 * @param[in]	dev_hndl:	qdma device handle
 *
 * @return	buffer size on success
 * @return	<0 on failure
 *****************************************************************************/
unsigned int qdma_get_buf_sz(unsigned long dev_hndl, u32 *buf_sz)
{
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;

	/* MD: Check if the device handle is valid */
	if (!xdev) {
		pr_err("xdev is invalid");
		return -EINVAL;
	}

	/* MD: Read the buffer size from the hardware configuration */
	if (xdev->hw.qdma_global_csr_conf(xdev, 0, QDMA_GLOBAL_CSR_ARRAY_SZ,
			buf_sz, QDMA_CSR_BUF_SZ, QDMA_HW_ACCESS_READ)) {
		pr_err("Failed to read buffer size");
		return -EINVAL;
	}

	pr_info("Successfully retrieved buffer size for xdev 0x%p\n", xdev);
	return 0;
}

#ifdef QDMA_CSR_REG_UPDATE
/* MD:****************************************************************************/
/* MD:*
 * qdma_set_glbl_rng_sz() - Handler function to set the glbl_rng_sz value
 *
 * @param[in]	dev_hndl:		qdma device handle
 * @param[in]	glbl_rng_sz:	interrupt aggregation ring size
 *
 * @return	0 on success
 * @return	<0 on failure
 *****************************************************************************/
int qdma_set_glbl_rng_sz(unsigned long dev_hndl, u32 *glbl_rng_sz)
{
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;
	int rv = -1;

	/* MD: Check if the device handle is valid */
	if (!xdev) {
		pr_err("xdev is invalid");
		return -EINVAL;
	}

	/* MD: Check if active queues exist, which would prevent changing glbl_rng_sz */
	if (qdma_get_active_queue_count(xdev->conf.pdev->bus->number)) {
		pr_err("xdev 0x%p, FMAP prog done, cannot modify glbl_rng_sz\n", xdev);
		return rv;
	}

	/* MD: Write the given ring sizes to the registers */
	rv = xdev->hw.qdma_global_csr_conf(xdev, 0, QDMA_GLOBAL_CSR_ARRAY_SZ,
			glbl_rng_sz, QDMA_CSR_RING_SZ, QDMA_HW_ACCESS_WRITE);
	if (rv < 0) {
		pr_err("Failed to write glbl rng size, err = %d", rv);
		return -EINVAL;
	}

	/* MD: Read back the configuration to ensure it was set correctly */
	rv = qdma_csr_read(xdev, &xdev->csr_info);
	if (unlikely(rv < 0)) {
		pr_err("Failed to read glbl csr, err = %d", rv);
		return rv;
	}

	pr_info("Successfully set global ring size for xdev 0x%p\n", xdev);
	return 0;
}
#endif

/* MD:****************************************************************************/
/* MD:*
 * qdma_get_glbl_rng_sz() - Handler function to get the glbl_rng_sz value
 *
 * @param[in]	dev_hndl:	qdma device handle
 *
 * @return	glbl_rng_sz size on success
 * @return	<0 on failure
 *****************************************************************************/
unsigned int qdma_get_glbl_rng_sz(unsigned long dev_hndl, u32 *glbl_rng_sz)
{
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;

	/* MD: Check if the device handle is valid */
	if (!xdev) {
		pr_err("xdev is invalid");
		return -EINVAL;
	}

	/* MD: Read the global ring size from the hardware configuration */
	if (xdev->hw.qdma_global_csr_conf(xdev, 0, QDMA_GLOBAL_CSR_ARRAY_SZ,
			glbl_rng_sz, QDMA_CSR_RING_SZ, QDMA_HW_ACCESS_READ)) {
		pr_err("Failed to read global ring size");
		return -EINVAL;
	}

	pr_info("Successfully retrieved global ring size for xdev 0x%p\n", xdev);
	return 0;
}

#ifdef QDMA_CSR_REG_UPDATE
/* MD:****************************************************************************/
/* MD:*
 * qdma_set_timer_cnt() - Handler function to set the timer count values
 *
 * @param[in]	dev_hndl:	qdma device handle
 * @param[in]	tmr_cnt:	Array of 16 timer count values
 *
 * @return	0 on success
 * @return	<0 on failure
 *****************************************************************************/
int qdma_set_timer_cnt(unsigned long dev_hndl, u32 *tmr_cnt)
{
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;
	int rv = -1;

	/* MD: Check if the device handle is valid */
	if (!xdev) {
		pr_err("xdev is invalid");
		return -EINVAL;
	}

	/* MD: Check if active queues exist, which would prevent changing timer count */
	if (qdma_get_active_queue_count(xdev->conf.pdev->bus->number)) {
		pr_err("xdev 0x%p, FMAP prog done, cannot modify timer count\n", xdev);
		return rv;
	}

	/* MD: Write the timer count values to the registers */
	rv = xdev->hw.qdma_global_csr_conf(xdev, 0, QDMA_GLOBAL_CSR_ARRAY_SZ,
			tmr_cnt, QDMA_CSR_TIMER_CNT, QDMA_HW_ACCESS_WRITE);
	if (unlikely(rv < 0)) {
		pr_err("global timer set failed, err = %d", rv);
		return xdev->hw.qdma_get_error_code(rv);
	}

	/* MD: Read back the configuration to ensure it was set correctly */
	rv = qdma_csr_read(xdev, &xdev->csr_info);
	if (unlikely(rv < 0)) {
		pr_err("Failed to read timer configuration, err = %d", rv);
		return rv;
	}

	pr_info("Successfully set timer count for xdev 0x%p\n", xdev);
	return 0;
}
#endif

/* MD:****************************************************************************/
/* MD:*
 * qdma_get_timer_cnt() - Handler function to get the timer_cnt value
 *
 * @param[in]	dev_hndl:	qdma device handle
 *
 * @return	timer_cnt on success
 * @return	<0 on failure
 *****************************************************************************/
unsigned int qdma_get_timer_cnt(unsigned long dev_hndl, u32 *tmr_cnt)
{
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;
	int rv = -1;

	/* MD: Check if the device handle is valid */
	if (!xdev) {
		pr_err("xdev is invalid");
		return -EINVAL;
	}

	/* MD: Read the timer count from the hardware configuration */
	rv = xdev->hw.qdma_global_csr_conf(xdev, 0, QDMA_GLOBAL_CSR_ARRAY_SZ,
			tmr_cnt, QDMA_CSR_TIMER_CNT, QDMA_HW_ACCESS_READ);
	if (unlikely(rv < 0)) {
		pr_err("get global timer failed, err = %d", rv);
		return xdev->hw.qdma_get_error_code(rv);
	}

	pr_info("Successfully retrieved timer count for xdev 0x%p\n", xdev);
	return 0;
}

#ifdef QDMA_CSR_REG_UPDATE
/* MD:****************************************************************************/
/* MD:*
 * qdma_set_cnt_thresh() - Handler function to set the counter threshold value
 *
 * @param[in]	dev_hndl:	qdma device handle
 * @param[in]	cnt_th:		Array of 16 timer count values
 *
 * @return	QDMA_OPERATION_SUCCESSFUL on success
 * @return	<0 on failure
 *****************************************************************************/
int qdma_set_cnt_thresh(unsigned long dev_hndl, unsigned int *cnt_th)
{
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;
	int rv = -1;

	/* MD: Check if the device handle is valid */
	if (!xdev) {
		pr_err("xdev is invalid");
		return -EINVAL;
	}

	/* MD: Check if active queues exist, which would prevent changing cnt_th */
	if (qdma_get_active_queue_count(xdev->conf.pdev->bus->number)) {
		pr_err("xdev 0x%p, FMAP prog done, cannot modify threshold count\n", xdev);
		return rv;
	}

	/* MD: Write the counter threshold values to the registers */
	rv = xdev->hw.qdma_global_csr_conf(xdev, 0, QDMA_GLOBAL_CSR_ARRAY_SZ, cnt_th,
			QDMA_CSR_CNT_TH, QDMA_HW_ACCESS_WRITE);
	if (unlikely(rv < 0)) {
		pr_err("set global counter failed, err = %d", rv);
		return xdev->hw.qdma_get_error_code(rv);
	}

	/* MD: Read back the configuration to ensure it was set correctly */
	rv = qdma_csr_read(xdev, &xdev->csr_info);
	if (unlikely(rv < 0)) {
		pr_err("Failed to read counter threshold configuration, err = %d", rv);
		return rv;
	}

	pr_info("Successfully set counter threshold for xdev 0x%p\n", xdev);
	return 0;
}
#endif

/* MD:****************************************************************************/
/* MD:*
 * qdma_get_cnt_thresh() - Handler function to get the counter thresh value
 *
 * @param[in]	dev_hndl:	qdma device handle
 *
 * @return	counter threshold values on success
 * @return	<0 on failure
 *****************************************************************************/
unsigned int qdma_get_cnt_thresh(unsigned long dev_hndl, u32 *cnt_th)
{
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;
	int rv = -1;

	/* MD: Check if the device handle is valid */
	if (!xdev) {
		pr_err("xdev is invalid");
		return -EINVAL;
	}

	/* MD: Read the counter threshold from the hardware configuration */
	rv = xdev->hw.qdma_global_csr_conf(xdev, 0, QDMA_GLOBAL_CSR_ARRAY_SZ, cnt_th,
			QDMA_CSR_CNT_TH, QDMA_HW_ACCESS_READ);
	if (unlikely(rv < 0)) {
		pr_err("get global counter failed, err = %d", rv);
		return xdev->hw.qdma_get_error_code(rv);
	}

	pr_info("Successfully retrieved counter threshold for xdev 0x%p\n", xdev);
	return 0;
}

#ifdef QDMA_CSR_REG_UPDATE
/* MD:****************************************************************************/
/* MD:*
 * qdma_set_cmpl_status_acc() - Handler function to set the cmpl_status_acc
 * configuration value
 *
 * @param[in]	dev_hndl:	qdma device handle
 * @param[in]	cmpl_status_acc:	Writeback Accumulation value
 *
 * @return	QDMA_OPERATION_SUCCESSFUL on success
 * @return	<0 on failure
 *****************************************************************************/
int qdma_set_cmpl_status_acc(unsigned long dev_hndl, u32 cmpl_status_acc)
{
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;
	int rv = 0;

	/* MD: Check if the device handle is valid */
	if (!xdev) {
		pr_err("xdev is invalid");
		return -EINVAL;
	}

	/* MD: Check if active queues exist, which would prevent changing cmpl_status_acc */
	if (qdma_get_active_queue_count(xdev->conf.pdev->bus->number)) {
		pr_err("xdev 0x%p, FMAP prog done, cannot modify cmpt acc\n", xdev);
		return -EINVAL;
	}

	/* MD: Write the given cmpl_status_acc value to the register */
	rv = xdev->hw.qdma_global_writeback_interval_conf(xdev, cmpl_status_acc,
							QDMA_HW_ACCESS_WRITE);
	if (unlikely(rv < 0)) {
		pr_err("set global writeback intvl failed, err = %d", rv);
		return xdev->hw.qdma_get_error_code(rv);
	}

	/* MD: Read back the configuration to ensure it was set correctly */
	rv = qdma_csr_read(xdev, &xdev->csr_info);
	if (unlikely(rv < 0)) {
		pr_err("Failed to read completion status accumulation configuration, err = %d", rv);
		return rv;
	}

	pr_info("Successfully set completion status accumulation for xdev 0x%p\n", xdev);
	return 0;
}
#endif

/* MD:****************************************************************************/
/* MD:*
 * qdma_get_cmpl_status_acc() - Handler function to get the cmpl_status_acc
 * configuration value
 *
 * @param[in] dev_hndl:		qdma device handle
 *
 * Handler function to get the writeback accumulation value
 *
 * @return	cmpl_status_acc on success
 * @return	<0 on failure
 *****************************************************************************/
unsigned int qdma_get_wb_intvl(unsigned long dev_hndl)
{
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;
	unsigned int wb_intvl;
	int rv = -1;

	/* MD: Check if the device handle is valid */
	if (!xdev) {
		pr_err("xdev is invalid");
		return -EINVAL;
	}

	/* MD: Read the current cmpl_status_acc value from the register and return */
	rv = xdev->hw.qdma_global_writeback_interval_conf(xdev, &wb_intvl,
							QDMA_HW_ACCESS_READ);
	if (unlikely(rv < 0)) {
		pr_err("read global writeback intvl failed, err = %d", rv);
		return xdev->hw.qdma_get_error_code(rv);
	}

	pr_info("Successfully retrieved writeback interval for xdev 0x%p\n", xdev);
	return wb_intvl;
}
#endif
