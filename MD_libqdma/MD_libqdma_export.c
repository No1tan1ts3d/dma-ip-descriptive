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

/* MD:*
 * @file
 * @brief This file contains the definitions for libqdma interfaces
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
#include "qdma_st_c2h.h"
#include "thread.h"
#include "version.h"
#include "qdma_resource_mgmt.h"
#include "qdma_mbox.h"
#include "qdma_platform.h"

#ifdef DEBUGFS
#include "qdma_debugfs_queue.h"

/* MD:* debugfs root */
struct dentry *qdma_debugfs_root;
static bool qdma_debufs_cleanup = true;
#endif

#define QDMA_Q_PEND_LIST_COMPLETION_TIMEOUT 1000 /* MD: msec */

struct drv_mode_name mode_name_list[] = {
	{ AUTO_MODE,			"auto"},
	{ POLL_MODE,			"poll"},
	{ DIRECT_INTR_MODE,		"direct interrupt"},
	{ INDIRECT_INTR_MODE,	"indirect interrupt"},
	{ LEGACY_INTR_MODE,		"legacy interrupt"}
};

struct qdma_q_type q_type_list[] = {
	{"H2C", Q_H2C},
	{"C2H", Q_C2H},
	{"CMPT", Q_CMPT},
	{"BI", Q_H2C_C2H},
};

/* MD: ********************* static function definitions ************************ */
#ifdef __QDMA_VF__

/* MD:****************************************************************************/
/* MD:*
 * qdma_dev_notify_qadd() - Notify the addition of a queue to the device
 *
 * @param[in]	descq:		Descriptor queue structure
 * @param[in]	q_type:		Type of queue being added
 *
 * @return	0 on success
 * @return	<0 on failure
 *****************************************************************************/
static int qdma_dev_notify_qadd(struct qdma_descq *descq, enum queue_type_t q_type)
{
	struct mbox_msg *m;
	int rv = 0;
	struct xlnx_dma_dev *xdev = descq->xdev;

	/* MD: Allocate a mailbox message */
	m = qdma_mbox_msg_alloc();
	if (!m) {
		pr_err("Failed to allocate mbox msg");
		return -ENOMEM;
	}

	/* MD: Compose the mailbox message for queue addition notification */
	qdma_mbox_compose_vf_notify_qadd(xdev->func_id, descq->qidx_hw, (enum qdma_dev_q_type)q_type, m->raw);

	/* MD: Send the mailbox message */
	rv = qdma_mbox_msg_send(xdev, m, 1, QDMA_MBOX_MSG_TIMEOUT_MS);
	if (rv < 0) {
		pr_err("%s, mbox failed for queue add %d.\n", xdev->conf.name, rv);
		goto free_msg;
	}

	/* MD: Check the response status */
	rv = qdma_mbox_vf_response_status(m->raw);
	if (rv < 0) {
		pr_err("mbox_vf_response_status failed, err = %d", rv);
		rv = -EINVAL;
	}

free_msg:
	/* MD: Free the mailbox message */
	qdma_mbox_msg_free(m);
	return rv;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_dev_notify_qdel() - Notify the deletion of a queue from the device
 *
 * @param[in]	descq:		Descriptor queue structure
 * @param[in]	q_type:		Type of queue being deleted
 *
 * @return	0 on success
 * @return	<0 on failure
 *****************************************************************************/
static int qdma_dev_notify_qdel(struct qdma_descq *descq, enum queue_type_t q_type)
{
	struct mbox_msg *m;
	int rv = 0;
	struct xlnx_dma_dev *xdev = descq->xdev;

	/* MD: Allocate a mailbox message */
	m = qdma_mbox_msg_alloc();
	if (!m) {
		pr_err("Failed to allocate mbox msg");
		return -ENOMEM;
	}

	/* MD: Compose the mailbox message for queue deletion notification */
	qdma_mbox_compose_vf_notify_qdel(xdev->func_id, descq->qidx_hw, (enum qdma_dev_q_type)q_type, m->raw);

	/* MD: Send the mailbox message */
	rv = qdma_mbox_msg_send(xdev, m, 1, QDMA_MBOX_MSG_TIMEOUT_MS);
	if (rv < 0) {
		pr_err("%s, mbox failed for queue delete %d.\n", xdev->conf.name, rv);
		goto free_msg;
	}

	/* MD: Check the response status */
	rv = qdma_mbox_vf_response_status(m->raw);
	if (rv < 0) {
		pr_err("mbox_vf_response_status failed, err = %d", rv);
		rv = -EINVAL;
	}

free_msg:
	/* MD: Free the mailbox message */
	qdma_mbox_msg_free(m);
	return rv;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_dev_get_active_qcnt() - Get the count of active queues
 *
 * @param[in]	xdev:		Pointer to the DMA device structure
 * @param[out]	h2c_qs:		Pointer to store the count of H2C queues
 * @param[out]	c2h_qs:		Pointer to store the count of C2H queues
 * @param[out]	cmpt_qs:	Pointer to store the count of completion queues
 *
 * @return	0 on success
 * @return	<0 on failure
 *****************************************************************************/
static int qdma_dev_get_active_qcnt(struct xlnx_dma_dev *xdev,
		u32 *h2c_qs, u32 *c2h_qs, u32 *cmpt_qs)
{
	struct mbox_msg *m;
	int rv = 0;

	/* MD: Allocate a mailbox message */
	m = qdma_mbox_msg_alloc();
	if (!m) {
		pr_err("Failed to allocate mbox msg");
		return -ENOMEM;
	}

	/* MD: Compose the mailbox message to get active queue count */
	qdma_mbox_compose_vf_get_device_active_qcnt(xdev->func_id, m->raw);

	/* MD: Send the mailbox message */
	rv = qdma_mbox_msg_send(xdev, m, 1, QDMA_MBOX_MSG_TIMEOUT_MS);
	if (rv < 0) {
		pr_err("%s, mbox failed for queue add %d.\n", xdev->conf.name, rv);
		goto free_msg;
	}

	/* MD: Check the response status */
	rv = qdma_mbox_vf_response_status(m->raw);
	if (rv < 0) {
		pr_err("mbox_vf_response_status failed, err = %d", rv);
		rv = -EINVAL;
		goto free_msg;
	}

	/* MD: Retrieve the active queue counts */
	*h2c_qs = (uint32_t)qdma_mbox_vf_active_queues_get(m->raw, QDMA_DEV_Q_TYPE_H2C);
	*c2h_qs = (uint32_t)qdma_mbox_vf_active_queues_get(m->raw, QDMA_DEV_Q_TYPE_C2H);
	*cmpt_qs = (uint32_t)qdma_mbox_vf_active_queues_get(m->raw, QDMA_DEV_Q_TYPE_CMPT);

	pr_info("Active queues - H2C: %u, C2H: %u, CMPT: %u\n", *h2c_qs, *c2h_qs, *cmpt_qs);

free_msg:
	/* MD: Free the mailbox message */
	qdma_mbox_msg_free(m);
	return rv;
}

#endif

/* MD:****************************************************************************/
/* MD:*
 * qdma_validate_qconfig() - Validate the queue configuration
 *
 * @param[in]	xdev:		Pointer to the DMA device structure
 * @param[in]	qconf:		Pointer to the queue configuration structure
 * @param[in]	buf:		Buffer to store error messages
 * @param[in]	buflen:		Length of the buffer
 *
 * @return	0 on success
 * @return	<0 on failure
 *****************************************************************************/
static int qdma_validate_qconfig(struct xlnx_dma_dev *xdev,
				struct qdma_queue_conf *qconf,
				char *buf, int buflen)
{
	/* MD: Check if the device or queue configuration is NULL */
	if (!xdev || !qconf) {
		pr_err("Invalid input received, xdev=%p, qconf=%p", xdev, qconf);
		return -EINVAL;
	}

	/* MD: Ensure the input buffer is not empty */
	if (!buf || !buflen) {
		pr_err("Invalid argument: buf=%p, buflen=%d", buf, buflen);
		return -EINVAL;
	}

	/* MD: Validate the completion trigger mode */
	if (qconf->cmpl_trig_mode > TRIG_MODE_COMBO) {
		pr_err("Invalid trigger mode: %d", qconf->cmpl_trig_mode);
		snprintf(buf, buflen, "qdma%05x : Invalid trigger mode %d passed\n",
			xdev->conf.bdf, qconf->cmpl_trig_mode);
		return -EINVAL;
	}

	/* MD: Check for unsupported descriptor sizes and trigger modes for specific device types */
	if (xdev->version_info.ip_type == QDMA_VERSAL_HARD_IP &&
		xdev->version_info.device_type == QDMA_DEVICE_VERSAL_CPM4) {
		if ((qconf->cmpl_desc_sz == CMPT_DESC_SZ_64B) ||
				(qconf->sw_desc_sz == DESC_SZ_64B)) {
			pr_err("Invalid descriptor sw desc: %d, cmpl desc: %d",
					qconf->sw_desc_sz, qconf->sw_desc_sz);
			snprintf(buf, buflen, "qdma%05x : 64B desc size is not supported\n",
				xdev->conf.bdf);
			return -EINVAL;
		}

		if (qconf->cmpl_trig_mode == TRIG_MODE_COMBO) {
			pr_err("Invalid trigger mode: %d", qconf->cmpl_trig_mode);
			snprintf(buf, buflen, "qdma%05x : Trigger mode COMBO is not supported\n",
				xdev->conf.bdf);
			return -EINVAL;
		}
	}

	pr_info("Queue configuration validated successfully for xdev 0x%p\n", xdev);
	return 0;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_request_wait_for_cmpl() - static function to monitor the
 *                                wait completion
 *
 * @param[in]  xdev:   pointer to xlnx_dma_dev structure
 * @param[in]  descq:  pointer to qdma_descq structure
 * @param[in]  req:    qdma request
 *
 * @return     0: success
 * @return     <0: error
 *****************************************************************************/
static int qdma_request_wait_for_cmpl(struct xlnx_dma_dev *xdev,
            struct qdma_descq *descq, struct qdma_request *req)
{
    struct qdma_sgt_req_cb *cb = qdma_req_cb_get(req);

    /* MD: If timeout is specified, wait until timeout or callback completion */
    if (req->timeout_ms) {
#ifdef __XRT__
        qdma_waitq_wait_event_timeout(cb->wq, cb->done &&
                (descq->pidx == descq->cidx),
                msecs_to_jiffies(req->timeout_ms));
#else
        qdma_waitq_wait_event_timeout(cb->wq, cb->done,
                msecs_to_jiffies(req->timeout_ms));
#endif
    } else {
        qdma_waitq_wait_event(cb->wq, cb->done);
    }

    lock_descq(descq);
    /* MD: If callback is not done, request timed out, delete from list */
    if (!cb->done)
        list_del(&cb->list);

    /* MD: If callback is not done but status is updated, return I/O error */
    if (!cb->done || cb->status) {
        pr_err("%s: req 0x%p, %c,%u,%u/%u,0x%llx, done %d, err %d, tm %u.\n",
                descq->conf.name,
                req, req->write ? 'W' : 'R',
                cb->offset,
                cb->left,
                req->count,
                req->ep_addr,
                cb->done,
                cb->status,
                req->timeout_ms);
        qdma_descq_dump(descq, NULL, 0, 1);
        unlock_descq(descq);

        return -EIO;
    }

    unlock_descq(descq);
    return 0;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_request_submit_st_c2h() - static function to handle the
 *                                st c2h request
 *
 * @param[in]  xdev:   pointer to xlnx_dma_dev structure
 * @param[in]  descq:  pointer to qdma_descq structure
 * @param[in]  req:    qdma request
 *
 * @return     0: success
 * @return     <0: error
 *****************************************************************************/
static ssize_t qdma_request_submit_st_c2h(struct xlnx_dma_dev *xdev,
            struct qdma_descq *descq, struct qdma_request *req)
{
    struct qdma_dev *qdev;
    struct qdma_sgt_req_cb *cb = qdma_req_cb_get(req);
    int wait = req->fp_done ? 0 : 1;
    int rv = 0;

    /* MD: Validate xdev */
    if (!xdev) {
        pr_err("xdev is invalid");
        return -EINVAL;
    }

    qdev = xdev_2_qdev(xdev);
    /* MD: Ensure qdev is not NULL */
    if (!qdev) {
        pr_err("dev %s, qdev null.\n",
            dev_name(&xdev->conf.pdev->dev));
        return -EINVAL;
    }

    pr_debug("%s: data len %u, sgl 0x%p, sgl cnt %u, tm %u ms.\n",
            descq->conf.name,
            req->count, req->sgl, req->sgcnt, req->timeout_ms);

    /* MD: Initialize request count */
    cb->left = req->count;

    lock_descq(descq);
    if (descq->q_stop_wait) {
        unlock_descq(descq);
        return 0;
    }
    if ((descq->q_state == Q_STATE_ONLINE) &&
            !descq->q_stop_wait) {
        /* MD: Add to pending list before cidx/pidx update */
        list_add_tail(&cb->list, &descq->pend_list);
        /* MD: Read data from device */
        descq_st_c2h_read(descq, req, 1, 1);
        if (!cb->left) {
            list_del(&cb->list);
            unlock_descq(descq);
            return req->count;
        }
        descq->pend_list_empty = 0;
        unlock_descq(descq);
    } else {
        unlock_descq(descq);
        pr_err("%s descq %s NOT online.\n",
            xdev->conf.name, descq->conf.name);
        return -EINVAL;
    }

    /* MD: Wake up completion thread if associated */
    if (descq->cmplthp)
        qdma_kthread_wakeup(descq->cmplthp);

    if (!wait) {
        pr_debug("%s: cb 0x%p, data len 0x%x NO wait.\n",
            descq->conf.name, cb, req->count);
        return 0;
    }

    /* MD: Wait for request completion */
    rv = qdma_request_wait_for_cmpl(xdev, descq, req);
    if (rv < 0) {
        if (!req->dma_mapped)
            sgl_unmap(xdev->conf.pdev, req->sgl, req->sgcnt,
                DMA_FROM_DEVICE);
        return rv;
    }

    /* MD: Return number of processed requests */
    return req->count - cb->left;
}

/* MD: ********************* public function definitions ************************ */

/* MD:****************************************************************************/
/* MD:*
 * qdma_queue_get_config() - Retrieve the configuration of a queue
 *
 * @param[in]	dev_hndl:	Device handle returned from qdma_device_open()
 * @param[in]	id:		Queue index
 * @param[in]	buflen:		Length of the input buffer
 * @param[out]	buf:		Message buffer
 * @param[out]	qconf:		Pointer to hold the qdma_queue_conf structure
 *
 * @return	0: success
 * @return	<0: error
 *****************************************************************************/
int qdma_queue_get_config(unsigned long dev_hndl, unsigned long id,
		struct qdma_queue_conf *qconf,
		char *buf, int buflen)
{
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;
	struct qdma_descq *descq;

	/* MD: Ensure the input buffer is not empty */
	if (!buf || !buflen) {
		pr_err("invalid argument: buf=%p, buflen=%d", buf, buflen);
		return -EINVAL;
	}

	/* MD: Validate the device handle */
	if (!xdev) {
		pr_err("dev_hndl is NULL");
		snprintf(buf, buflen, "dev_hndl is NULL");
		return -EINVAL;
	}

	if (xdev_check_hndl(__func__, xdev->conf.pdev, dev_hndl) < 0) {
		pr_err("Invalid dev_hndl passed");
		snprintf(buf, buflen, "Invalid dev_hndl passed");
		return -EINVAL;
	}

	/* MD: Retrieve the descriptor queue by ID */
	descq = qdma_device_get_descq_by_id(xdev, id, buf, buflen, 0);
	if (!descq) {
		pr_err("Invalid qid(%lu)", id);
		snprintf(buf, buflen, "Invalid qid(%lu)\n", id);
		return -EINVAL;
	}

	/* MD: Copy the queue configuration */
	memcpy(qconf, &descq->conf, sizeof(struct qdma_queue_conf));

	snprintf(buf, buflen, "Queue configuration for %s id %u is stored in qconf param",
		descq->conf.name, descq->conf.qidx);

	return 0;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_device_capabilities_info() - Retrieve the capabilities of a device
 *
 * @dev_hndl:	Handle returned from qdma_device_open()
 * @dev_attr:	Pointer to hold all the device attributes
 *
 * Return:	0 for success and <0 for error
 *****************************************************************************/
int qdma_device_capabilities_info(unsigned long dev_hndl,
		struct qdma_dev_attributes *dev_attr)
{
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;

	/* MD: Validate the device handle */
	if (!xdev) {
		pr_err("dev_hndl is NULL");
		return -EINVAL;
	}

	/* MD: Validate the device attributes pointer */
	if (!dev_attr) {
		pr_err("dev_attr is NULL");
		return -EINVAL;
	}

	if (xdev_check_hndl(__func__, xdev->conf.pdev, dev_hndl) < 0) {
		pr_err("Invalid dev_hndl passed");
		return -EINVAL;
	}

	/* MD: Copy the device capabilities */
	memcpy(dev_attr, &(xdev->dev_cap), sizeof(struct qdma_dev_attributes));

	return 0;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_config_reg_info_dump() - Dump the detailed field information of register
 *
 * @param[in] dev_hndl:	Handle returned from qdma_device_open()
 * @param[in] reg_addr: Register address info to be dumped
 * @param[in] num_regs: Number of registers to be dumped
 * @param[in] buf:	Buffer containing the output
 * @param[in] buflen:	Length of the buffer
 *
 * @return:		Length of output buffer
 *****************************************************************************/
int qdma_config_reg_info_dump(unsigned long dev_hndl, uint32_t reg_addr,
				uint32_t num_regs, char *buf, int buflen)
{
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;
	int rv;

	/* MD: Ensure the input buffer is not empty */
	if (!buf || !buflen) {
		pr_err("invalid argument: buf=%p, buflen=%d\n", buf, buflen);
		return -EINVAL;
	}

	/* MD: Validate the device handle */
	if (!xdev) {
		pr_err("dev_hndl is NULL");
		snprintf(buf, buflen, "dev_hndl is NULL");
		return -EINVAL;
	}

	if (xdev_check_hndl(__func__, xdev->conf.pdev, dev_hndl) < 0) {
		pr_err("Invalid dev_hndl passed");
		snprintf(buf, buflen, "Invalid dev_hndl passed\n");
		return -EINVAL;
	}

	/* MD: Check if the feature is supported */
	if (xdev->hw.qdma_dump_reg_info == NULL) {
		pr_err("Err: Feature not supported\n");
		snprintf(buf, buflen, "Err: Feature not supported\n");
		return -EPERM;
	}

	/* MD: Dump the register information */
	rv = xdev->hw.qdma_dump_reg_info((void *)dev_hndl, reg_addr,
			num_regs, buf, buflen);
	return rv;
}

#ifndef __QDMA_VF__
/* MD:****************************************************************************/
/* MD:*
 * qdma_config_reg_dump() - Display config registers in a string buffer
 *
 * @param[in]	dev_hndl:	Device handle returned from qdma_device_open()
 * @param[in]	buflen:		Length of the input buffer
 * @param[out]	buf:		Message buffer
 *
 * @return	Success: If optional message buffer used then strlen of buf,
 *		otherwise 0
 * @return	<0: error
 *****************************************************************************/
int qdma_config_reg_dump(unsigned long dev_hndl, char *buf, int buflen)
{
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;
	int rv;

	/* MD: Ensure the input buffer is not empty */
	if (!buf || !buflen) {
		pr_err("invalid argument: buf=%p, buflen=%d", buf, buflen);
		return -EINVAL;
	}

	/* MD: Validate the device handle */
	if (!xdev) {
		pr_err("dev_hndl is NULL");
		snprintf(buf, buflen, "dev_hndl is NULL");
		return -EINVAL;
	}

	if (xdev_check_hndl(__func__, xdev->conf.pdev, dev_hndl) < 0) {
		pr_err("Invalid dev_hndl passed");
		snprintf(buf, buflen, "Invalid dev_hndl passed");
		return -EINVAL;
	}

	/* MD: Dump the configuration registers */
	rv = xdev->hw.qdma_dump_config_regs((void *)dev_hndl, 0, buf, buflen);
	return rv;
}

#else

/* MD:****************************************************************************/
/* MD:*
 * qdma_config_read_reg_list() - Read a list of registers
 *
 * @param[in]	xdev:		Pointer to the DMA device structure
 * @param[in]	group_num:	Group number of registers
 * @param[out]	num_regs:	Pointer to store the number of registers
 * @param[out]	reg_list:	Pointer to store the register data
 *
 * @return	0 on success
 * @return	<0 on failure
 *****************************************************************************/
static int qdma_config_read_reg_list(struct xlnx_dma_dev *xdev,
			uint16_t group_num,
			uint16_t *num_regs, struct qdma_reg_data *reg_list)
{
	struct mbox_msg *m = qdma_mbox_msg_alloc();
	int rv;

	/* MD: Allocate a mailbox message */
	if (!m)
		return -ENOMEM;

	/* MD: Compose the mailbox message for register read */
	qdma_mbox_compose_reg_read(xdev->func_id, group_num, m->raw);

	/* MD: Send the mailbox message */
	rv = qdma_mbox_msg_send(xdev, m, 1, QDMA_MBOX_MSG_TIMEOUT_MS);
	if (rv < 0) {
		if (rv != -ENODEV)
			pr_err("%s, reg read mbox failed %d.\n",
				xdev->conf.name, rv);
		goto err_out;
	}

	/* MD: Retrieve the register list */
	rv = qdma_mbox_vf_reg_list_get(m->raw, num_regs, reg_list);
	if (rv < 0) {
		pr_err("qdma_mbox_vf_reg_list_get failed with error = %d", rv);
		goto err_out;
	}

err_out:
	/* MD: Free the mailbox message */
	qdma_mbox_msg_free(m);
	return rv;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_config_reg_dump() - Display configuration registers in a string buffer
 *
 * @param[in]	dev_hndl:	Device handle returned from qdma_device_open()
 * @param[in]	buflen:		Length of the input buffer
 * @param[out]	buf:		Message buffer
 *
 * @return	Success: If optional message buffer used then strlen of buf,
 *	otherwise 0
 * @return	<0: Error
 *****************************************************************************/
int qdma_config_reg_dump(unsigned long dev_hndl, char *buf, int buflen)
{
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;
	int rv = 0;
	struct qdma_reg_data *reg_list;
	uint16_t num_regs = 0, group_num = 0;
	int len = 0, rcv_len = 0;

	/* MD: Ensure the input buffer is not empty */
	if (!buf || !buflen) {
		pr_err("Invalid argument: buf=%p, buflen=%d", buf, buflen);
		return -EINVAL;
	}

	/* MD: Validate the device handle */
	if (!xdev) {
		pr_err("dev_hndl is NULL");
		snprintf(buf, buflen, "dev_hndl is NULL");
		return -EINVAL;
	}

	if (xdev_check_hndl(__func__, xdev->conf.pdev, dev_hndl) < 0) {
		pr_err("Invalid dev_hndl passed");
		snprintf(buf, buflen, "Invalid dev_hndl passed");
		return -EINVAL;
	}

	/* MD: Allocate memory for register list */
	reg_list = kzalloc((QDMA_MAX_REGISTER_DUMP * sizeof(struct qdma_reg_data)), GFP_KERNEL);
	if (!reg_list) {
		pr_err("%s: reg_list OOM", xdev->conf.name);
		snprintf(buf, buflen, "reg_list OOM");
		return -ENOMEM;
	}

	/* MD: Iterate through register groups and dump configuration */
	for (group_num = 0; group_num < QDMA_REG_READ_GROUP_3; group_num++) {
		/* MD: Reset the reg_list with 0's */
		memset(reg_list, 0, (QDMA_MAX_REGISTER_DUMP * sizeof(struct qdma_reg_data)));
		rv = qdma_config_read_reg_list(xdev, group_num, &num_regs, reg_list);
		if (rv < 0) {
			pr_err("Failed to read config registers, rv = %d", rv);
			snprintf(buf, buflen, "Failed to read config regs");
			goto free_reg_list;
		}

		/* MD: Dump the configuration register list */
		rcv_len = xdev->hw.qdma_dump_config_reg_list((void *)dev_hndl, num_regs, reg_list, buf + len, buflen - len);
		if (rcv_len < 0) {
			pr_err("%s: failed with error = %d", __func__, rv);
			snprintf(buf, buflen, "Failed to dump config regs");
			goto free_reg_list;
		}
		len += rcv_len;
	}

free_reg_list:
	kfree(reg_list);
	return len;
}
#endif
/* MD:****************************************************************************/
/* MD:*
 * qdma_queue_dump() - Display a queue's state in a string buffer
 *
 * @param[in]	dev_hndl:	Device handle returned from qdma_device_open()
 * @param[in]	id:		Queue index
 * @param[in]	buflen:		Length of the input buffer
 * @param[out]	buf:		Message buffer
 *
 * @return	Success: If optional message buffer used then strlen of buf,
 *	otherwise 0
 * @return	<0: Error
 *****************************************************************************/
int qdma_queue_dump(unsigned long dev_hndl, unsigned long id, char *buf, int buflen)
{
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;
	struct qdma_descq *descq;
	int rv = 0;

	/* MD: Ensure the input buffer is not empty */
	if (!buf || !buflen) {
		pr_err("Invalid argument: buf=%p, buflen=%d", buf, buflen);
		return -EINVAL;
	}

	/* MD: Validate the device handle */
	if (!xdev) {
		pr_err("dev_hndl is NULL");
		snprintf(buf, buflen, "dev_hndl is NULL");
		return -EINVAL;
	}

	if (xdev_check_hndl(__func__, xdev->conf.pdev, dev_hndl) < 0) {
		pr_err("Invalid dev_hndl passed");
		snprintf(buf, buflen, "Invalid dev_hndl passed");
		return -EINVAL;
	}

	/* MD: Retrieve the descriptor queue by ID */
	descq = qdma_device_get_descq_by_id(xdev, id, buf, buflen, 0);
	if (!descq) {
		pr_err("Invalid qid(%lu)", id);
		snprintf(buf, buflen, "Invalid qid(%lu)\n", id);
		return -EINVAL;
	}

	/* MD: Dump the descriptor queue context */
	rv = qdma_descq_context_dump(descq, buf, buflen);
	if (rv < 0) {
		snprintf(buf, buflen, "%s dump context failed %d.\n", descq->conf.name, rv);
		return rv;
	}

	return buflen;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_queue_dump_desc() - Display a queue's descriptor ring from index start
 *                          to end in a string buffer
 *
 * @param[in]	dev_hndl:	Device handle returned from qdma_device_open()
 * @param[in]	id:		Queue index
 * @param[in]	start:		Start index
 * @param[in]	end:		End index
 * @param[in]	buflen:		Length of the input buffer
 * @param[out]	buf:		Message buffer
 *
 * @return	Success: If optional message buffer used then strlen of buf,
 *		otherwise 0
 * @return	<0: Error
 *****************************************************************************/
int qdma_queue_dump_desc(unsigned long dev_hndl, unsigned long id,
			unsigned int start, unsigned int end, char *buf,
			int buflen)
{
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;
	struct qdma_descq *descq;
	int len = 0;

	/* MD: Ensure the input buffer is not empty */
	if (!buf || !buflen) {
		pr_err("Invalid argument: buf=%p, buflen=%d", buf, buflen);
		return -EINVAL;
	}

	/* MD: Validate the device handle */
	if (!xdev) {
		pr_err("dev_hndl is NULL");
		snprintf(buf, buflen, "dev_hndl is NULL");
		return -EINVAL;
	}

	if (xdev_check_hndl(__func__, xdev->conf.pdev, dev_hndl) < 0) {
		pr_err("Invalid dev_hndl passed");
		snprintf(buf, buflen, "Invalid dev_hndl passed");
		return -EINVAL;
	}

	/* MD: Retrieve the descriptor queue details based on the queue ID */
	descq = qdma_device_get_descq_by_id(xdev, id, buf, buflen, 1);
	if (!descq) {
		pr_err("Invalid qid(%lu)", id);
		snprintf(buf, buflen, "Invalid qid(%lu)\n", id);
		return -EINVAL;
	}

	/* MD: Validate the range of start and end indexes */
	if (start > end) {
		pr_err("start/end param passed is invalid, <start> shall be less than <end>");
		snprintf(buf, buflen, "start/end param passed is invalid, <start> shall be less than <end>");
		return -EINVAL;
	}

	/* MD: Dump the queue state */
	len = qdma_descq_dump_state(descq, buf, buflen);
	if (descq->q_state != Q_STATE_ONLINE)
		return len;

	/* MD: Dump the queue descriptor state */
	len += qdma_descq_dump_desc(descq, start, end, buf + len, buflen - len);
	return len;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_queue_dump_cmpt() - Display a queue's completion ring from index start
 *                          to end in a string buffer
 *
 * @param[in]	dev_hndl:	Device handle returned from qdma_device_open()
 * @param[in]	id:		Queue index
 * @param[in]	start:		Start index
 * @param[in]	end:		End index
 * @param[in]	buflen:		Length of the input buffer
 * @param[out]	buf:		Message buffer
 *
 * @return	Success: If optional message buffer used then strlen of buf,
 *		otherwise 0
 * @return	<0: Error
 *****************************************************************************/
int qdma_queue_dump_cmpt(unsigned long dev_hndl, unsigned long id,
			unsigned int start, unsigned int end, char *buf,
			int buflen)
{
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;
	struct qdma_descq *descq;
	int len = 0;

	/* MD: Ensure the input buffer is not empty */
	if (!buf || !buflen) {
		pr_err("Invalid argument: buf=%p, buflen=%d", buf, buflen);
		return -EINVAL;
	}

	/* MD: Validate the device handle */
	if (!xdev) {
		pr_err("dev_hndl is NULL");
		snprintf(buf, buflen, "dev_hndl is NULL");
		return -EINVAL;
	}

	if (xdev_check_hndl(__func__, xdev->conf.pdev, dev_hndl) < 0) {
		pr_err("Invalid dev_hndl passed");
		snprintf(buf, buflen, "Invalid dev_hndl passed");
		return -EINVAL;
	}

	/* MD: Retrieve the descriptor queue details based on the queue ID */
	descq = qdma_device_get_descq_by_id(xdev, id, buf, buflen, 1);
	if (!descq) {
		pr_err("Invalid qid(%lu)", id);
		snprintf(buf, buflen, "Invalid qid(%lu)\n", id);
		return -EINVAL;
	}

	/* MD: Validate the range of start and end indexes */
	if (start > end) {
		pr_err("start/end param passed is invalid, <start> shall be less than <end>");
		snprintf(buf, buflen, "start/end param passed is invalid, <start> shall be less than <end>");
		return -EINVAL;
	}

	/* MD: Dump the descriptor state */
	len = qdma_descq_dump_state(descq, buf, buflen);
	/* MD: If the descriptor is in online state, dump the completion state */
	if (descq->q_state == Q_STATE_ONLINE)
		len += qdma_descq_dump_cmpt(descq, start, end, buf + len, buflen - len);

	return len;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_queue_remove() - remove a queue (i.e., offline, NOT ready for dma)
 *
 * @param[in]	dev_hndl:	dev_hndl returned from qdma_device_open()
 * @param[in]	id:		queue index
 * @param[in]	buflen:		length of the input buffer
 * @param[out]	buf:		message buffer
 *
 * @return	0: success
 * @return	<0: error
 *****************************************************************************/
int qdma_queue_remove(unsigned long dev_hndl, unsigned long id, char *buf,
			int buflen)
{
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;
	struct qdma_descq *descq;
#ifdef DEBUGFS
	struct qdma_descq *pair_descq;
#endif
	struct qdma_dev *qdev;
	int rv = 0;

	/* MD:* Ensure the input buffer is valid */
	if (!buf || !buflen) {
		pr_err("invalid argument: buf=%p, buflen=%d", buf, buflen);
		return -EINVAL;
	}

	/* MD:* Validate the device handle */
	if (!xdev) {
		pr_err("dev_hndl is NULL");
		snprintf(buf, buflen, "dev_hndl is NULL");
		return -EINVAL;
	}

	if (xdev_check_hndl(__func__, xdev->conf.pdev, dev_hndl) < 0) {
		pr_err("Invalid dev_hndl passed");
		snprintf(buf, buflen, "Invalid dev_hndl passed");
		return -EINVAL;
	}

	/* MD:* Retrieve the descriptor queue by ID */
	descq = qdma_device_get_descq_by_id(xdev, id, buf, buflen, 1);
#ifdef DEBUGFS
	/* MD:* Retrieve the paired descriptor queue if DEBUGFS is enabled */
	pair_descq = qdma_device_get_pair_descq_by_id(xdev, id, buf, buflen, 1);
#endif

	/* MD:* Convert xdev to qdev and validate */
	qdev = xdev_2_qdev(xdev);
	if (!qdev) {
		pr_err("dev %s, qdev null.\n", dev_name(&xdev->conf.pdev->dev));
		return -EINVAL;
	}

	/* MD:* Validate the descriptor queue */
	if (!descq) {
		pr_err("Invalid qid(%lu)", id);
		snprintf(buf, buflen, "Invalid qid(%lu)\n", id);
		return -EINVAL;
	}

	/* MD:* Check if the queue is in a state that allows deletion */
	if (descq->q_state != Q_STATE_ENABLED) {
		snprintf(buf, buflen,
			"queue %s, id %u cannot be deleted. Invalid q state: %s\n",
			descq->conf.name,
			descq->conf.qidx,
			q_state_list[descq->q_state].name);

		pr_err("queue %s, id %u cannot be deleted. Invalid q state: %s",
			descq->conf.name,
			descq->conf.qidx,
			q_state_list[descq->q_state].name);
		return -EINVAL;
	}

#ifdef DEBUGFS
	/* MD:* Exit debugfs queue if pair_descq is not NULL */
	if (pair_descq)
		dbgfs_queue_exit(&descq->conf, pair_descq);
	else
		dbgfs_queue_exit(&descq->conf, NULL);
#endif

#ifndef __QDMA_VF__
	/* MD:* Decrement the active queue count for non-VF */
	rv = qdma_dev_decrement_active_queue(xdev->dma_device_index,
			xdev->func_id,
			(enum qdma_dev_q_type)descq->conf.q_type);
	if (rv < 0) {
		pr_err("Failed to decrement the active %s queue",
				q_type_list[descq->conf.q_type].name);
		return rv;
	}

	/* MD:* Handle completion queue decrement for C2H type */
	if (descq->conf.st && (descq->conf.q_type == Q_C2H)) {
		rv = qdma_dev_decrement_active_queue(xdev->dma_device_index,
				xdev->func_id, QDMA_DEV_Q_TYPE_CMPT);
		if (rv < 0) {
			pr_err("Failed to decrement the active CMPT queue");
			qdma_dev_increment_active_queue(xdev->dma_device_index,
				xdev->func_id,
				(enum qdma_dev_q_type)descq->conf.q_type);
			return rv;
		}
	}
#else
	/* MD:* Notify queue deletion for VF */
	rv = qdma_dev_notify_qdel(descq,
			(enum queue_type_t)descq->conf.q_type);
	if (rv < 0) {
		pr_err("Failed to decrement active %s queue count",
				q_type_list[descq->conf.q_type].name);
		return rv;
	}
	if (descq->conf.st && (descq->conf.q_type == Q_C2H)) {
		rv = qdma_dev_notify_qdel(descq, Q_CMPT);
		if (rv < 0) {
			pr_err("Failed to decrement active CMPT queue count");
			qdma_dev_notify_qadd(descq,
					(enum queue_type_t)descq->conf.q_type);
			return rv;
		}
	}
#endif

	/* MD:* Lock the descriptor queue, change its state, and unlock */
	lock_descq(descq);
	descq->q_state = Q_STATE_DISABLED;
	unlock_descq(descq);

#ifndef __QDMA_VF__
	/* MD:* Clear legacy interrupts if in LEGACY_INTR_MODE */
	if (xdev->conf.qdma_drv_mode == LEGACY_INTR_MODE)
		intr_legacy_clear(descq);
#endif

	/* MD:* Log the successful deletion of the queue */
	snprintf(buf, buflen, "queue %s, id %u deleted.\n",
		descq->conf.name, descq->conf.qidx);

	pr_debug("queue %s, id %u deleted.\n",
				descq->conf.name, descq->conf.qidx);

	return 0;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_queue_config() - configure the queue with qconf parameters
 *
 * @param[in]	dev_hndl:	dev_hndl returned from qdma_device_open()
 * @param[in]	qid:		queue id
 * @param[in]	qconf:		queue configuration parameters
 * @param[in]	buflen:		length of the input buffer
 * @param[out]	buf:		message buffer
 *
 * @return	0: success
 * @return	<0: error
 *****************************************************************************/
int qdma_queue_config(unsigned long dev_hndl, unsigned long qid,
			struct qdma_queue_conf *qconf, char *buf, int buflen)
{
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;
	struct qdma_dev *qdev;
	struct qdma_descq *descq = NULL;
	int rv = 0;

	/* MD:* Ensure that input buffer is not empty, else return error */
	if (!buf || !buflen) {
		pr_err("Invalid argument: buf=%p, buflen=%d", buf, buflen);
		return -EINVAL;
	}

	/* MD:* Ensure that the dev_hndl passed is valid */
	if (!xdev) {
		pr_err("dev_hndl is NULL");
		snprintf(buf, buflen, "dev_hndl is NULL");
		return -EINVAL;
	}

	/* MD:* Validate the device handle */
	if (xdev_check_hndl(__func__, xdev->conf.pdev, dev_hndl) < 0) {
		pr_err("Invalid dev_hndl passed");
		snprintf(buf, buflen, "Invalid dev_hndl passed");
		return -EINVAL;
	}

	/* MD:* Convert xdev to qdev and ensure it is not NULL */
	qdev = xdev_2_qdev(xdev);
	if (!qdev) {
		pr_err("dev %s, qdev null.\n", dev_name(&xdev->conf.pdev->dev));
		snprintf(buf, buflen, "Q not already added. Add Q first\n");
		return -EINVAL;
	}

	/* MD:* Retrieve the descriptor queue for the given qid */
	descq = qdma_device_get_descq_by_id(xdev, qid, NULL, 0, 0);
	if (!descq) {
		pr_err("Invalid queue ID! qid=%lu, max=%u\n", qid, qdev->qmax);
		snprintf(buf, buflen, "Invalid queue ID qid=%lu, max=%u, base=%u\n",
				 qid, qdev->qmax, qdev->qbase);
		return -EINVAL;
	}

	/* MD:* Lock the descriptor queue and check its state */
	lock_descq(descq);
	if (descq->q_state != Q_STATE_ENABLED) {
		pr_err("queue_%lu Invalid state! Q not in enabled state\n", qid);
		snprintf(buf, buflen, "Error. Required Q state=%s, Current Q state=%s\n",
				q_state_list[Q_STATE_ENABLED].name,
				q_state_list[descq->q_state].name);
		unlock_descq(descq);
		return -EINVAL;
	}
	unlock_descq(descq);

	/* MD:* Validate the queue configuration */
	rv = qdma_validate_qconfig(xdev, qconf, buf, buflen);
	if (rv != 0)
		return rv;

	/* MD:* Configure the descriptor queue */
	lock_descq(descq);
	qdma_descq_config(descq, qconf, 1);
	unlock_descq(descq);

	/* MD:* Log the successful configuration */
	snprintf(buf, buflen, "Queue %s id %u is configured with the qconf passed ",
		descq->conf.name, descq->conf.qidx);

	return 0;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_queue_list() - Display all configured queues in a string buffer
 *
 * @param[in]	dev_hndl:	dev_hndl returned from qdma_device_open()
 * @param[in]	qidx:		Queue index
 * @param[in]	num_q:		Number of Queues to list
 * @param[in]	buflen:		length of the input buffer
 * @param[out]	buf:		message buffer
 *
 * @return	success: if optional message buffer used then strlen of buf,
 *	otherwise 0
 * @return	<0: error
 *****************************************************************************/
int qdma_queue_list(unsigned long dev_hndl, int qidx, int num_q, char *buf,
			int buflen)
{
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;
	struct qdma_dev *qdev;
	struct qdma_descq *descq = NULL;
	char *cur = buf;
	char * const end = buf + buflen;
	uint32_t h2c_qcnt = 0, c2h_qcnt = 0, cmpt_qcnt = 0;
	int i;

	/* MD: Ensure the input buffer is valid */
	if (!buf || !buflen) {
		pr_err("invalid argument: buf=%p, buflen=%d", buf, buflen);
		return -EINVAL;
	}

	/* MD: Validate the device handle */
	if (!xdev) {
		pr_err("dev_hndl is NULL");
		snprintf(buf, buflen, "dev_hndl is NULL");
		return -EINVAL;
	}

	if (xdev_check_hndl(__func__, xdev->conf.pdev, dev_hndl) < 0) {
		pr_err("Invalid dev_hndl passed");
		snprintf(buf, buflen, "Invalid dev_hndl passed");
		return -EINVAL;
	}

	qdev = xdev_2_qdev(xdev);
	/* MD: Ensure qdev is not NULL */
	if (!qdev) {
		pr_err("dev %s, qdev null.\n", dev_name(&xdev->conf.pdev->dev));
		snprintf(buf, buflen, "Q not already added. Add Q first\n");
		return -EINVAL;
	}

#ifndef __QDMA_VF__
	/* MD: Get the count of active H2C, C2H, and CMPT queues */
	h2c_qcnt = qdma_get_device_active_queue_count(
			xdev->dma_device_index,
			xdev->func_id,
			QDMA_DEV_Q_TYPE_H2C);

	c2h_qcnt = qdma_get_device_active_queue_count(
			xdev->dma_device_index,
			xdev->func_id,
			QDMA_DEV_Q_TYPE_C2H);

	cmpt_qcnt = qdma_get_device_active_queue_count(
			xdev->dma_device_index,
			xdev->func_id,
			QDMA_DEV_Q_TYPE_CMPT);
#else
	qdma_dev_get_active_qcnt(xdev, &h2c_qcnt, &c2h_qcnt, &cmpt_qcnt);
#endif

	/* MD: Print the queue counts to the buffer */
	cur += snprintf(cur, end - cur, "H2C Q: %u, C2H Q: %u, CMPT Q %u.\n",
				h2c_qcnt, c2h_qcnt, cmpt_qcnt);
	if (cur >= end)
		goto handle_truncation;

	/* MD: Traverse and dump the descriptors for H2C queues */
	if (h2c_qcnt) {
		descq = qdev->h2c_descq + qidx;
		for (i = qidx; i < (qidx + num_q); i++, descq++) {
			lock_descq(descq);
			if (descq->q_state != Q_STATE_DISABLED)
				cur += qdma_descq_dump(descq, cur, end - cur, 0);
			unlock_descq(descq);

			if (cur >= end)
				goto handle_truncation;
		}
	}

	/* MD: Traverse and dump the descriptors for C2H queues */
	if (c2h_qcnt) {
		descq = qdev->c2h_descq + qidx;
		for (i = qidx; i < (qidx + num_q); i++, descq++) {
			lock_descq(descq);
			if (descq->q_state != Q_STATE_DISABLED)
				cur += qdma_descq_dump(descq, cur, end - cur, 0);
			unlock_descq(descq);

			if (cur >= end)
				goto handle_truncation;
		}
	}

	/* MD: Traverse and dump the descriptors for CMPT queues */
	if (cmpt_qcnt) {
		descq = qdev->cmpt_descq + qidx;
		for (i = qidx; i < (qidx + num_q); i++, descq++) {
			lock_descq(descq);
			if (descq->q_state != Q_STATE_DISABLED)
				cur += qdma_descq_dump(descq, cur, end - cur, 0);
			unlock_descq(descq);

			if (cur >= end)
				goto handle_truncation;
		}
	}

	return cur - buf;

handle_truncation:
	/* MD: Handle buffer truncation */
	*buf = '\0';
	return buf - cur;
}

#define Q_PRESENT_H2C_MASK (1 << Q_H2C)
#define Q_PRESENT_C2H_MASK (1 << Q_C2H)
#define Q_PRESENT_CMPT_MASK (1 << Q_CMPT)
#define Q_MODE_SHIFT 3
#define Q_MODE_MASK  (1 << Q_MODE_SHIFT)

static int is_usable_queue(struct xlnx_dma_dev *xdev, int qidx, int q_type, int st)
{
    struct qdma_dev *qdev = xdev_2_qdev(xdev);
    int refmask = 0x0;
    int cmptq_chkmask = 0x0;
    int h2cq_chkmask = 0x0;
    int c2hq_chkmask = 0x0;
    int reqmask = 0;
    struct qdma_descq *h2c_descq = qdev->h2c_descq + qidx;
    struct qdma_descq *c2h_descq = qdev->c2h_descq + qidx;
    struct qdma_descq *cmpt_descq = qdev->cmpt_descq + qidx;

    // MD: Check the state of the H2C descriptor queue
    lock_descq(h2c_descq);
    if (h2c_descq->q_state != Q_STATE_DISABLED) {
        refmask |= (1 << Q_H2C);
        if (h2c_descq->conf.st)
            refmask |= (1 << Q_MODE_SHIFT);
    }
    unlock_descq(h2c_descq);

    // MD: Check the state of the C2H descriptor queue
    lock_descq(c2h_descq);
    if (c2h_descq->q_state != Q_STATE_DISABLED) {
        refmask |= (1 << Q_C2H);
        if (c2h_descq->conf.st)
            refmask |= (1 << Q_MODE_SHIFT);
    }
    unlock_descq(c2h_descq);

    // MD: Check the state of the CMPT descriptor queue
    lock_descq(cmpt_descq);
    if (cmpt_descq->q_state != Q_STATE_DISABLED)
        refmask |= (1 << Q_CMPT);
    unlock_descq(cmpt_descq);

    // MD: Set the required mask based on the queue type and mode
    reqmask = (1 << q_type);
    if (st)
        reqmask |= Q_MODE_MASK;

    // MD: Validate the queue addition based on the type
    if (q_type == Q_CMPT) {
        cmptq_chkmask |= (Q_PRESENT_CMPT_MASK | Q_MODE_MASK);
        if (refmask & cmptq_chkmask) {
            pr_err("Q_CMPT: refmask not valid");
            goto q_reject;
        }
    } else if (q_type == Q_H2C) {
        c2hq_chkmask = (Q_PRESENT_C2H_MASK | Q_MODE_MASK);
        if (st && (refmask & Q_PRESENT_CMPT_MASK)) {
            pr_err("Q_H2C: CMPT q given to MM");
            goto q_reject; // MD: CMPT q given to MM
        }
        if (st && (refmask & Q_PRESENT_C2H_MASK) && (refmask & c2hq_chkmask) == Q_PRESENT_C2H_MASK) {
            pr_err("Q_H2C: MM mode c2h q present");
            goto q_reject; // MD: MM mode c2h q present
        }
        if (!st && (refmask & Q_PRESENT_C2H_MASK) && (refmask & c2hq_chkmask) == c2hq_chkmask) {
            pr_err("Q_H2C: ST mode c2h q present");
            goto q_reject; // MD: ST mode c2h q present
        }
        if (refmask & Q_PRESENT_H2C_MASK) {
            pr_err("Q_H2C: h2c q already present");
            goto q_reject; // MD: h2c q already present
        }
    } else {
        h2cq_chkmask |= (Q_PRESENT_H2C_MASK | Q_MODE_MASK);
        if (st && (refmask & Q_PRESENT_CMPT_MASK)) {
            pr_err("!Q_H2C: CMPT q given to MM");
            goto q_reject; // MD: CMPT q given to MM
        }
        if (st && (refmask & Q_PRESENT_H2C_MASK) && (refmask & h2cq_chkmask) == Q_PRESENT_H2C_MASK) {
            pr_err("!Q_H2C: MM mode h2c q present");
            goto q_reject; // MD: MM mode h2c q present
        }
        if (!st && (refmask & Q_PRESENT_H2C_MASK) && (refmask & h2cq_chkmask) == h2cq_chkmask) {
            pr_err("!Q_H2C: ST mode h2c q present");
            goto q_reject; // MD: ST mode h2c q present
        }
        if (refmask & Q_PRESENT_C2H_MASK) {
            pr_err("!Q_H2C: c2h q already present");
            goto q_reject; // MD: c2h q already present
        }
    }
    return 0;

q_reject:
    pr_err("Q addition feasibility check failed");
    return -1;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_queue_add() - add a queue
 *
 * @param[in]	dev_hndl:	dev_hndl returned from qdma_device_open()
 * @param[in]	qconf:		queue configuration parameters
 * @param[in]	qhndl:	list of unsigned long values that are the opaque qhndl
 * @param[in]	buflen:		length of the input buffer
 * @param[out]	buf:		message buffer
 *
 * @return	0: success
 * @return	<0: error
 *****************************************************************************/
int qdma_queue_add(unsigned long dev_hndl, struct qdma_queue_conf *qconf,
			unsigned long *qhndl, char *buf, int buflen)
{
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;
	unsigned int qcnt;
	struct qdma_descq *descq;
	struct qdma_dev *qdev;
#ifdef DEBUGFS
	struct qdma_descq *pairq;
#endif
#ifdef __QDMA_VF__
	uint32_t h2c_qcnt = 0, c2h_qcnt = 0, cmpt_qcnt = 0;
#endif
	int rv = 0;

	/* MD:* Ensure the input buffer is not empty */
	if (!buf || !buflen) {
		pr_err("invalid argument: buf=%p, buflen=%d", buf, buflen);
		return -EINVAL;
	}

	/* MD:* Validate the device handle */
	if (!xdev) {
		pr_err("dev_hndl is NULL");
		snprintf(buf, buflen, "dev_hndl is NULL");
		return -EINVAL;
	}

	if (xdev_check_hndl(__func__, xdev->conf.pdev, dev_hndl) < 0) {
		pr_err("Invalid dev_hndl passed");
		snprintf(buf, buflen, "Invalid dev_hndl passed");
		return -EINVAL;
	}

	/* MD:* Validate the queue configuration */
	if (!qconf) {
		pr_err("Invalid qconf %p", qconf);
		snprintf(buf, buflen, "%s, add, qconf NULL.\n", xdev->conf.name);
		return -EINVAL;
	}

	/* MD:* Validate the queue handle */
	if (!qhndl) {
		pr_warn("qhndl NULL.\n");
		snprintf(buf, buflen, "%s, add, qhndl NULL.\n", xdev->conf.name);
		return -EINVAL;
	}

	/* MD:* Check for valid queue type */
	if (qconf->q_type > Q_CMPT) {
		pr_err("Invalid queue type passed");
		snprintf(buf, buflen, "Invalid queue type passed");
		return -EINVAL;
	}

	/* MD:* Check for ST mode compatibility */
	if (qconf->st && (qconf->q_type == Q_CMPT)) {
		pr_err("Cannot create independent completion ring for ST mode. It is supported along with C2H direction");
		snprintf(buf, buflen, "Cannot create independent completion ring for ST mode. It is supported along with C2H direction");
		return -EINVAL;
	}

	/* MD:* Check if MM Completions are enabled */
	if ((qconf->q_type == Q_CMPT) && !xdev->dev_cap.mm_cmpt_en) {
		pr_err("MM Completions not enabled");
		snprintf(buf, buflen, "MM Completions not enabled");
		return -EINVAL;
	}

	qdev = xdev_2_qdev(xdev);

	/* MD:* Reset qhandle to an invalid value */
	*qhndl = QDMA_QUEUE_IDX_INVALID;

	/* MD:* Check if the requested mode is enabled */
	if ((qconf->st && !xdev->dev_cap.st_en) ||
	    (!qconf->st && !xdev->dev_cap.mm_en)) {
		pr_warn("%s, %s mode not enabled.\n", xdev->conf.name, qconf->st ? "ST" : "MM");
		snprintf(buf, buflen, "qdma%05x %s mode not enabled.\n", xdev->conf.bdf, qconf->st ? "ST" : "MM");
		return -EINVAL;
	}

	/* MD:* Validate the queue configuration */
	rv = qdma_validate_qconfig(xdev, qconf, buf, buflen);
	if (rv < 0)
		return rv;

	spin_lock(&qdev->lock);

	/* MD:* Validate the queue index */
	if ((qconf->qidx != QDMA_QUEUE_IDX_INVALID) && (qconf->qidx >= qdev->qmax)) {
		spin_unlock(&qdev->lock);
		snprintf(buf, buflen, "qdma%05x invalid idx %u >= %u.\n", xdev->conf.bdf, qconf->qidx, qdev->qmax);
		pr_err("Invalid queue index, qid = %d, qmax = %d", qconf->qidx, qdev->qmax);
		return -EINVAL;
	}

	/* MD:* Check for available queue index */
#ifndef __QDMA_VF__
	qcnt = qdma_get_device_active_queue_count(xdev->dma_device_index, xdev->func_id, qconf->q_type);
#else
	spin_unlock(&qdev->lock);
	qdma_dev_get_active_qcnt(xdev, &h2c_qcnt, &c2h_qcnt, &cmpt_qcnt);
	spin_lock(&qdev->lock);
	if (qconf->q_type == Q_H2C)
		qcnt = h2c_qcnt;
	else if (qconf->q_type == Q_C2H)
		qcnt = c2h_qcnt;
	else
		qcnt = cmpt_qcnt;
#endif
	if (qcnt >= qdev->qmax) {
		spin_unlock(&qdev->lock);
		pr_warn("No free queues %u/%u.\n", qcnt, qdev->qmax);
		snprintf(buf, buflen, "qdma%05x No free queues %u/%u.\n", xdev->conf.bdf, qcnt, qdev->qmax);
		return -EIO;
	}

	spin_unlock(&qdev->lock);

	/* MD:* Determine the descriptor queue based on queue type */
	if (qconf->q_type == Q_C2H)
		descq = qdev->c2h_descq;
	else if (qconf->q_type == Q_H2C)
		descq = qdev->h2c_descq;
	else
		descq = qdev->cmpt_descq;

	/* MD:* Allocate a free queue index if not specified */
	if (qconf->qidx == QDMA_QUEUE_IDX_INVALID) {
		int i;

		/* MD:* Loop through the queue list to find an empty descriptor queue */
		for (i = 0; i < qdev->qmax; i++, descq++) {
			if (is_usable_queue(xdev, i, qconf->q_type, qconf->st) < 0)
				continue;

			descq += i;
			lock_descq(descq);

			/* MD:* Enable the descriptor queue */
			descq->q_state = Q_STATE_ENABLED;
			/* MD:* Assign the queue index */
			qconf->qidx = i;
			unlock_descq(descq);

			break;
		}

		/* MD:* No free descriptor queue found */
		if (i == qdev->qmax) {
			pr_warn("no free %s qp found, %u.\n", qconf->st ? "ST" : "MM", qdev->qmax);
			rv = -EPERM;
			snprintf(buf, buflen, "qdma%05x no %s QP, %u.\n", xdev->conf.bdf, qconf->st ? "ST" : "MM", qdev->qmax);
			return rv;
		}
	} else {
		/* MD:* Check queue compatibility */
		if (is_usable_queue(xdev, qconf->qidx, qconf->q_type, qconf->st) < 0) {
			pr_err("Queue compatibility check failed against existing queues\n");
			snprintf(buf, buflen, "Queue compatibility check failed against existing queues\n");
			return -EPERM;
		}

		descq += qconf->qidx;

		lock_descq(descq);

		/* MD:* Enable the descriptor queue */
		descq->q_state = Q_STATE_ENABLED;

		unlock_descq(descq);
	}

	/* MD:* Prepare the queue resources */
	rv = qdma_device_prep_q_resource(xdev);
	if (rv < 0) {
#ifdef __QDMA_VF__
		lock_descq(descq);
		descq->q_state = Q_STATE_DISABLED;
		unlock_descq(descq);
#endif
		return rv;
	}
#ifndef __QDMA_VF__
	if (xdev->conf.qdma_drv_mode == LEGACY_INTR_MODE) {
		rv = intr_legacy_setup(descq);
		if (rv > 0) {
			/* MD:* Support only 1 queue in legacy interrupt mode */
			intr_legacy_clear(descq);
			lock_descq(descq);
			descq->q_state = Q_STATE_DISABLED;
			unlock_descq(descq);
			pr_debug("qdma%05x - Q%u - No free queues %u/%u.\n", xdev->conf.bdf, descq->conf.qidx, rv, 1);
			rv = -EINVAL;
			snprintf(buf, buflen, "qdma%05x No free queues %u/%d.\n", xdev->conf.bdf, qcnt, 1);
			return rv;
		} else if (rv < 0) {
			rv = -EINVAL;
			lock_descq(descq);
			descq->q_state = Q_STATE_DISABLED;
			unlock_descq(descq);
			pr_debug("qdma%05x Legacy interrupt setup failed.\n", xdev->conf.bdf);
			snprintf(buf, buflen, "qdma%05x Legacy interrupt setup failed.\n", xdev->conf.bdf);
			return rv;
		}
	}
#endif
	/* MD:* Configure the descriptor queue */
	qdma_descq_config(descq, qconf, 0);
#ifndef __QDMA_VF__
	rv = qdma_dev_increment_active_queue(xdev->dma_device_index, xdev->func_id, (enum qdma_dev_q_type)qconf->q_type);
	if (rv < 0) {
		pr_err("Failed to increment active %s queue count", q_type_list[qconf->q_type].name);
		return rv;
	}
	if (qconf->st && (qconf->q_type == Q_C2H)) {
		rv = qdma_dev_increment_active_queue(xdev->dma_device_index, xdev->func_id, QDMA_DEV_Q_TYPE_CMPT);
		if (rv < 0) {
			pr_err("Failed to increment CMPT queue count");
			qdma_dev_decrement_active_queue(xdev->dma_device_index, xdev->func_id, (enum qdma_dev_q_type)qconf->q_type);
			return rv;
		}
	}
#else
	rv = qdma_dev_notify_qadd(descq, (enum queue_type_t)qconf->q_type);
	if (rv < 0) {
		pr_err("Failed to increment active %s queue count", q_type_list[qconf->q_type].name);
		return rv;
	}
	if (qconf->st && (qconf->q_type == Q_C2H)) {
		rv = qdma_dev_notify_qadd(descq, Q_CMPT);
		if (rv < 0) {
			pr_err("Failed to increment active CMPT queue count");
			qdma_dev_notify_qdel(descq, (enum queue_type_t)qconf->q_type);
			return rv;
		}
	}
#endif

	/* MD:* Copy back the name in config */
	memcpy(qconf->name, descq->conf.name, QDMA_QUEUE_NAME_MAXLEN);
	*qhndl = (unsigned long)descq->conf.qidx;
	if (qconf->q_type == Q_C2H)
		*qhndl += qdev->qmax;
	if (qconf->q_type == Q_CMPT)
		*qhndl += (2 * qdev->qmax);
	descq->q_hndl = *qhndl;

#ifdef DEBUGFS
	if (qconf->q_type != Q_CMPT) {
		if (qconf->q_type == Q_H2C)
			pairq = qdev->c2h_descq + qconf->qidx;
		else
			pairq = qdev->h2c_descq + qconf->qidx;

		rv = dbgfs_queue_init(&descq->conf, pairq, xdev->dbgfs_queues_root);
		if (rv < 0) {
			pr_err("failed to create queue debug files for the queue %d\n", descq->conf.qidx);
		}
	}
#endif

	pr_debug("added %s, %s, qidx %u.\n", descq->conf.name, q_type_list[qconf->q_type].name, qconf->qidx);

	snprintf(buf, buflen, "%s %s added.\n", descq->conf.name, q_type_list[qconf->q_type].name);

	return 0;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_queue_start() - start a queue (i.e, online, ready for dma)
 *
 * @param[in]	dev_hndl:	dev_hndl returned from qdma_device_open()
 * @param[in]	id:		queue index
 * @param[in]	buflen:		length of the input buffer
 * @param[out]	buf:		message buffer
 *
 * @return	0: success
 * @return	<0: error
 *****************************************************************************/
int qdma_queue_start(unsigned long dev_hndl, unsigned long id,
		     char *buf, int buflen)
{
	struct qdma_descq *descq;
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;
	int rv;

	/* MD:* Ensure the input buffer is valid */
	if (!buf || !buflen) {
		pr_err("Invalid argument: buf=%p, buflen=%d", buf, buflen);
		return -EINVAL;
	}

	/* MD:* Validate the device handle */
	if (!xdev) {
		pr_err("dev_hndl is NULL");
		snprintf(buf, buflen, "dev_hndl is NULL");
		return -EINVAL;
	}

	if (xdev_check_hndl(__func__, xdev->conf.pdev, dev_hndl) < 0) {
		pr_err("Invalid dev_hndl passed");
		snprintf(buf, buflen, "Invalid dev_hndl passed");
		return -EINVAL;
	}

	/* MD:* Retrieve the descriptor queue by ID */
	descq = qdma_device_get_descq_by_id(xdev, id, buf, buflen, 1);
	if (!descq) {
		pr_err("Invalid qid(%lu)", id);
		snprintf(buf, buflen, "Invalid qid(%lu)\n", id);
		return -EINVAL;
	}

	lock_descq(descq);
	/* MD:* Check if the descriptor queue is enabled */
	if (descq->q_state != Q_STATE_ENABLED) {
		pr_err("%s invalid state, q_state %d.\n", descq->conf.name, descq->q_state);
		snprintf(buf, buflen, "%s invalid state, q_state %d.\n", descq->conf.name, descq->q_state);
		unlock_descq(descq);
		return -EINVAL;
	}

	/* MD:* Check for compatibility with EQDMA Soft IP and Vivado release */
	if ((xdev->version_info.ip_type == EQDMA_SOFT_IP) &&
		(xdev->version_info.vivado_release >= QDMA_VIVADO_2020_2)) {

		if (xdev->dev_cap.desc_eng_mode == QDMA_DESC_ENG_BYPASS_ONLY) {
			pr_err("Err: Bypass Only Design is not supported\n");
			snprintf(buf, buflen, "%s Bypass Only Design is not supported\n", descq->conf.name);
			unlock_descq(descq);
			return -EINVAL;
		}

		if (descq->conf.desc_bypass) {
			if (xdev->dev_cap.desc_eng_mode == QDMA_DESC_ENG_INTERNAL_ONLY) {
				pr_err("Err: Bypass mode not supported in Internal Mode only design\n");
				snprintf(buf, buflen, "%s Bypass mode not supported in Internal Mode only design\n", descq->conf.name);
				unlock_descq(descq);
				return -EINVAL;
			}
		}
	}

	/* MD:* Validate aperture size */
	if ((descq->conf.aperture_size != 0) &&
			((descq->conf.aperture_size & (descq->conf.aperture_size - 1)))) {
		pr_err("Err: %s Power of 2 aperture size supported\n", descq->conf.name);
		snprintf(buf, buflen, "Err:%s Power of 2 aperture size supported\n", descq->conf.name);
		unlock_descq(descq);
		return -ERANGE;
	}

	/* MD:* Complete the queue configuration */
	rv = qdma_descq_config_complete(descq);
	if (rv < 0) {
		pr_err("%s 0x%x setup failed.\n", descq->conf.name, descq->qidx_hw);
		snprintf(buf, buflen, "%s config failed.\n", descq->conf.name);
		unlock_descq(descq);
		return -EIO;
	}
	unlock_descq(descq);

	/* MD:* Allocate the queue resources */
	rv = qdma_descq_alloc_resource(descq);
	if (rv < 0) {
		pr_err("%s alloc resource failed.\n", descq->conf.name);
		snprintf(buf, buflen, "%s alloc resource failed.\n", descq->conf.name);
		return rv;
	}

	/* MD:* Program the hardware contexts */
	rv = qdma_descq_prog_hw(descq);
	if (rv < 0) {
		pr_err("%s 0x%x setup failed.\n", descq->conf.name, descq->qidx_hw);
		snprintf(buf, buflen, "%s prog. context failed.\n", descq->conf.name);
		goto clear_context;
	}

	/* MD:* Configure interrupt mode if applicable */
	if (descq->xdev->num_vecs) {
		unsigned long flags;
		struct intr_info_t *dev_intr_info_list = &descq->xdev->dev_intr_info_list[descq->intr_id];

		spin_lock_irqsave(&dev_intr_info_list->vec_q_list, flags);
		list_add_tail(&descq->intr_list, &dev_intr_info_list->intr_list);
		dev_intr_info_list->intr_list_cnt++;
		spin_unlock_irqrestore(&dev_intr_info_list->vec_q_list, flags);
	}

	/* MD:* Add work to the thread */
	qdma_thread_add_work(descq);

	snprintf(buf, buflen, "queue %s, idx %u started\n", descq->conf.name, descq->conf.qidx);

	/* MD:* Set the descriptor queue to online state */
	lock_descq(descq);
	descq->q_state = Q_STATE_ONLINE;
	unlock_descq(descq);

	return 0;

clear_context:
	/* MD:* Clear the context and free resources on failure */
	qdma_descq_context_clear(descq->xdev, descq->qidx_hw, descq->conf.st, descq->conf.q_type, 1);
	qdma_descq_free_resource(descq);

	return rv;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_get_queue_state() - Retrieve the state of a specified queue
 *
 * @param[in]	dev_hndl:	qdma device handle
 * @param[in]	id:		queue index
 * @param[out]	q_state:	pointer to store the queue state
 * @param[out]	buf:		message buffer
 * @param[in]	buflen:		length of the input buffer
 *
 * @return	0 on success
 * @return	<0 on failure
 *****************************************************************************/
int qdma_get_queue_state(unsigned long dev_hndl, unsigned long id,
		struct qdma_q_state *q_state, char *buf, int buflen)
{
	struct qdma_descq *descq;
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;

	/* MD:* Ensure the input buffer is valid */
	if (!buf || !buflen) {
		pr_err("Invalid argument: buf=%p, buflen=%d", buf, buflen);
		return -EINVAL;
	}

	/* MD:* Validate the device handle */
	if (!xdev) {
		pr_err("dev_hndl is NULL");
		snprintf(buf, buflen, "dev_hndl is NULL");
		return -EINVAL;
	}

	if (xdev_check_hndl(__func__, xdev->conf.pdev, dev_hndl) < 0) {
		pr_err("Invalid dev_hndl passed");
		snprintf(buf, buflen, "Invalid dev_hndl passed");
		return -EINVAL;
	}

	/* MD:* Retrieve the descriptor queue by ID */
	descq = qdma_device_get_descq_by_id(xdev, id, buf, buflen, 1);
	if (!descq) {
		pr_err("Invalid qid(%lu)", id);
		snprintf(buf, buflen, "Invalid qid(%lu)\n", id);
		return -EINVAL;
	}

	/* MD:* Validate the q_state pointer */
	if (!q_state) {
		pr_err("Invalid q_state:%p", q_state);
		return -EINVAL;
	}

	/* MD:* Lock the descriptor queue to safely access its state */
	lock_descq(descq);

	/* MD:* Populate the q_state structure with queue details */
	q_state->st = descq->conf.st;
	q_state->q_type = (enum queue_type_t)descq->conf.q_type;
	q_state->qidx = descq->conf.qidx;
	q_state->qstate = descq->q_state;

	/* MD:* Format the queue state information into the buffer */
	snprintf(buf, buflen, "queue state for %s id %u : %s\n",
		descq->conf.name, descq->conf.qidx,
		q_state_list[descq->q_state].name);

	/* MD:* Unlock the descriptor queue after accessing its state */
	unlock_descq(descq);

	pr_info("Successfully retrieved queue state for id %lu\n", id);
	return 0;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_queue_stop() - stop a queue (i.e., offline, NOT ready for dma)
 *
 * @param[in]	dev_hndl:	dev_hndl returned from qdma_device_open()
 * @param[in]	id:		queue index
 * @param[in]	buflen:		length of the input buffer
 * @param[out]	buf:		message buffer
 *
 * @return	0: success
 * @return	<0: error
 *****************************************************************************/
int qdma_queue_stop(unsigned long dev_hndl, unsigned long id, char *buf,
			int buflen)
{
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;
	struct qdma_descq *descq;
	struct qdma_sgt_req_cb *cb, *tmp;
	struct qdma_request *req;
	unsigned int pend_list_empty = 0;

	/* MD:* Ensure the input buffer is not empty, else return error */
	if (!buf || !buflen) {
		pr_err("invalid argument: buf=%p, buflen=%d", buf, buflen);
		return -EINVAL;
	}

	/* MD:* Validate the device handle */
	if (!xdev) {
		pr_err("dev_hndl is NULL");
		snprintf(buf, buflen, "dev_hndl is NULL");
		return -EINVAL;
	}

	if (xdev_check_hndl(__func__, xdev->conf.pdev, dev_hndl) < 0) {
		pr_err("Invalid dev_hndl passed");
		snprintf(buf, buflen, "Invalid dev_hndl passed");
		return -EINVAL;
	}

	/* MD:* Retrieve the descriptor queue by ID */
	descq = qdma_device_get_descq_by_id(xdev, id, buf, buflen, 1);
	if (!descq) {
		pr_err("Invalid qid(%ld)", id);
		return -EINVAL;
	}

	lock_descq(descq);
	/* MD:* Ensure the queue is online before proceeding */
	if (descq->q_state != Q_STATE_ONLINE) {
		unlock_descq(descq);
		pr_err("%s invalid state, q_state %d.\n",
		descq->conf.name, descq->q_state);
		snprintf(buf, buflen,
			"queue %s, idx %u stop failed.\n",
			 descq->conf.name, descq->conf.qidx);
		return -EINVAL;
	}
	pend_list_empty = descq->pend_list_empty;

	descq->q_stop_wait = 1;
	unlock_descq(descq);
	/* MD:* Wait for pending list to be empty if necessary */
	if (!pend_list_empty) {
		qdma_waitq_wait_event_timeout(descq->pend_list_wq,
			descq->pend_list_empty,
			msecs_to_jiffies(QDMA_Q_PEND_LIST_COMPLETION_TIMEOUT));
	}
	lock_descq(descq);
	/* MD:* Free the descriptor queue by updating the state */
	descq->q_state = Q_STATE_ENABLED;
	descq->q_stop_wait = 0;
	list_for_each_entry_safe(cb, tmp, &descq->pend_list, list) {
		req = (struct qdma_request *)cb;
		cb->done = 1;
		cb->status = -ENXIO;
		if (req->fp_done) {
			list_del(&cb->list);
			req->fp_done(req, 0, -ENXIO);
		} else
			qdma_waitq_wakeup(&cb->wq);
	}
	list_for_each_entry_safe(cb, tmp, &descq->work_list, list) {
		req = (struct qdma_request *)cb;
		cb->done = 1;
		cb->status = -ENXIO;
		if (req->fp_done) {
			qdma_work_queue_del(descq, cb);
			req->fp_done(req, 0, -ENXIO);
		} else
			qdma_waitq_wakeup(&cb->wq);
	}
	unlock_descq(descq);

	/* MD:* Remove the work thread associated with the current queue */
	qdma_thread_remove_work(descq);

	/* MD:* Clear the queue context */
	qdma_descq_context_clear(descq->xdev, descq->qidx_hw,
					descq->conf.st, descq->conf.q_type, 0);

	/* MD:* If the device is in direct/indirect interrupt mode,
	 *  delete the interrupt list for the queue
	 */
	if ((descq->xdev->conf.qdma_drv_mode != POLL_MODE) &&
		(descq->xdev->conf.qdma_drv_mode != LEGACY_INTR_MODE)) {
		unsigned long flags;
		struct intr_info_t *dev_intr_info_list =
			&descq->xdev->dev_intr_info_list[descq->intr_id];

		spin_lock_irqsave(&dev_intr_info_list->vec_q_list, flags);
		list_del(&descq->intr_list);
		dev_intr_info_list->intr_list_cnt--;
		spin_unlock_irqrestore(&dev_intr_info_list->vec_q_list, flags);
	}

	/* MD:* Free the queue resources */
	qdma_descq_free_resource(descq);
	/* MD:* Reset the total completed descriptors count */
	descq->total_cmpl_descs = 0;

	/* MD:* Fill the return buffer indicating that queue is stopped */
	snprintf(buf, buflen, "queue %s, idx %u stopped.\n",
			descq->conf.name, descq->conf.qidx);

	pr_info("Queue %s, idx %u successfully stopped.\n", descq->conf.name, descq->conf.qidx);

	return 0;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_get_queue_count() - Function to fetch the total number of queues
 *
 * @param[in]	dev_hndl:	dev_hndl returned from qdma_device_open()
 * @param[out]	q_count:	total q count
 * @param[out]	buf:		message buffer
 * @param[in]	buflen:	length of the input buffer
 *
 * @return	0: success
 * @return	<0: error
 *****************************************************************************/
int qdma_get_queue_count(unsigned long dev_hndl,
					struct qdma_queue_count *q_count,
					char *buf, int buflen)
{
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;

	/* MD: Ensure the input buffer and q_count are not empty */
	if (!buf || !buflen || !q_count) {
		pr_err("invalid argument: buf=%p, buflen=%d", buf, buflen);
		return -EINVAL;
	}

	/* MD: Validate the device handle */
	if (!xdev) {
		pr_err("dev_hndl is NULL");
		snprintf(buf, buflen, "dev_hndl is NULL");
		return -EINVAL;
	}

#ifndef __QDMA_VF__
	/* MD: Fetch the active queue counts for H2C, C2H, and CMPT */
	q_count->h2c_qcnt = qdma_get_device_active_queue_count(
			xdev->dma_device_index,
			xdev->func_id,
			QDMA_DEV_Q_TYPE_H2C);

	q_count->c2h_qcnt = qdma_get_device_active_queue_count(
			xdev->dma_device_index,
			xdev->func_id,
			QDMA_DEV_Q_TYPE_C2H);

	q_count->cmpt_qcnt = qdma_get_device_active_queue_count(
			xdev->dma_device_index,
			xdev->func_id,
			QDMA_DEV_Q_TYPE_CMPT);
#else
	/* MD: For VF, use the device's active queue count function */
	qdma_dev_get_active_qcnt(xdev, &(q_count->h2c_qcnt),
							 &(q_count->c2h_qcnt),
							 &(q_count->cmpt_qcnt));
#endif

	/* MD: Log the queue counts for debugging */
	pr_debug("h2c_qcnt = %d, c2h_qcnt = %d, cmpt_qcnt = %d",
			q_count->h2c_qcnt,
			q_count->c2h_qcnt,
			q_count->cmpt_qcnt);
	return 0;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_intr_ring_dump() - Display the interrupt ring info of a vector
 *
 * @param[in]	dev_hndl:	dev_hndl returned from qdma_device_open()
 * @param[in]	vector_idx:	vector number
 * @param[in]	start_idx:	interrupt ring start idx
 * @param[in]	end_idx:	interrupt ring end idx
 * @param[in]	buflen:		length of the input buffer
 * @param[out]	buf:		message buffer
 *
 * @return	0: success
 * @return	<0: error
 *****************************************************************************/
int qdma_intr_ring_dump(unsigned long dev_hndl, unsigned int vector_idx,
	int start_idx, int end_idx, char *buf, int buflen)
{
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;
	union qdma_intr_ring *ring_entry;
	struct intr_coal_conf *coal_entry;
	char *cur = buf;
	char * const end = buf + buflen;
	int counter = 0;
	u32 data[2];

	/* MD: Ensure the input buffer is not empty */
	if (!buf || !buflen) {
		pr_err("invalid argument: buf=%p, buflen=%d", buf, buflen);
		return -EINVAL;
	}

	/* MD: Validate the device handle */
	if (!xdev) {
		pr_err("dev_hndl is NULL");
		snprintf(buf, buflen, "dev_hndl is NULL");
		return -EINVAL;
	}

	if (xdev_check_hndl(__func__, xdev->conf.pdev, dev_hndl) < 0) {
		pr_err("Invalid dev_hndl passed");
		snprintf(buf, buflen, "Invalid dev_hndl passed");
		return -EINVAL;
	}

	/* MD: Check if interrupt aggregation is enabled */
	if ((xdev->conf.qdma_drv_mode != INDIRECT_INTR_MODE) &&
			(xdev->conf.qdma_drv_mode != AUTO_MODE)) {
		pr_err("Interrupt aggregation not enabled in %s mode",
				mode_name_list[xdev->conf.qdma_drv_mode].name);
		snprintf(buf, buflen,
				"Interrupt aggregation not enabled in %s mode",
				mode_name_list[xdev->conf.qdma_drv_mode].name);
		return -EINVAL;
	}

	/* MD: Validate the vector index */
	if ((vector_idx < xdev->dvec_start_idx) ||
		(vector_idx >=
		(xdev->dvec_start_idx + QDMA_NUM_DATA_VEC_FOR_INTR_CXT))) {
		pr_err("Vector idx %u is invalid. Shall be in range: %d -  %d.\n",
			vector_idx,
			xdev->dvec_start_idx,
			(xdev->dvec_start_idx +
			QDMA_NUM_DATA_VEC_FOR_INTR_CXT - 1));
		snprintf(buf, buflen,
			"Vector idx %u is invalid. Shall be in range: %d -  %d.\n",
			vector_idx,
			xdev->dvec_start_idx,
			(xdev->dvec_start_idx +
			QDMA_NUM_DATA_VEC_FOR_INTR_CXT - 1));
		return -EINVAL;
	}

	/* MD: Get the interrupt entry based on vector index */
	coal_entry = xdev->intr_coal_list + (vector_idx - xdev->dvec_start_idx);

	/* MD: Validate the interrupt ring entry indexes */
	if (start_idx > coal_entry->intr_rng_num_entries) {
		pr_err("start_idx %d is invalid. Shall be less than: %d\n",
					start_idx,
					coal_entry->intr_rng_num_entries);
		snprintf(buf, buflen,
			"start_idx %d is invalid. Shall be less than: %d\n",
			start_idx,
			coal_entry->intr_rng_num_entries);
		return -EINVAL;
	}

	if (end_idx == -1 || end_idx >= coal_entry->intr_rng_num_entries)
		end_idx = coal_entry->intr_rng_num_entries - 1;

	if (start_idx == -1)
		start_idx = 0;

	if (start_idx > end_idx) {
		pr_err("start_idx can't be greater than end_idx\n");
		snprintf(buf, buflen,
			"start_idx can't be greater than end_idx\n");
		return -EINVAL;
	}

	/* MD: Read the ring entries based on the range given and update the input buffer */
	for (counter = start_idx; counter <= end_idx; counter++) {
		ring_entry = coal_entry->intr_ring_base + counter;
		memcpy(data, ring_entry, sizeof(u32) * 2);
		cur += snprintf(cur, end - cur,
				"intr_ring_entry = %d: 0x%08x 0x%08x\n",
				counter, data[1], data[0]);
		if (cur >= end)
			goto handle_truncation;
	}

	return 0;
handle_truncation:
	*buf = '\0';
	return 0;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_software_version_info - Provides the QDMA software version
 *
 * @param[out]   software_version:   Buffer to store the libqdma software version
 * @param[in]    length:             Length of the buffer
 *
 * @return  0 on success
 * @return  <0 on error
 *****************************************************************************/
int qdma_software_version_info(char *software_version, int length)
{
    /* MD: Check if the software_version buffer is valid */
    if (!software_version) {
        pr_err("Invalid input software_version: %p", software_version);
        return -EINVAL;
    }

    /* MD: Copy the software version string into the provided buffer */
    snprintf(software_version, length, "%s", LIBQDMA_VERSION_STR);
    pr_info("Software version info retrieved: %s", software_version);

    return 0;
}

#ifdef __QDMA_VF__
/* MD:****************************************************************************/
/* MD:*
 * qdma_vf_qconf - Call for VF to request qmax number of Qs
 *
 * @param[in]   dev_hndl:   Device handle returned from qdma_device_open()
 * @param[in]   qmax:       Number of queues requested by VF
 *
 * @return  0 on success
 * @return  <0 on error
 *****************************************************************************/
int qdma_vf_qconf(unsigned long dev_hndl, int qmax)
{
    int err = 0;
    int qbase = -1;
    struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;

    /* MD: Check if the device handle is valid */
    if (!xdev) {
        pr_err("Invalid dev handle\n");
        return -EINVAL;
    }

    /* MD: Set the queue configuration for the device */
    err = device_set_qconf(xdev, &qmax, &qbase);
    if (err < 0) {
        pr_err("Setting qconf failed\n");
        return err;
    }

    /* MD: Clean up the device configuration */
    qdma_device_cleanup(xdev);

    /* MD: Update the device configuration with new queue base and max values */
    xdev->conf.qsets_base = qbase;
    xdev->conf.qsets_max = qmax;

    /* MD: Initialize the device with the new configuration */
    err = qdma_device_init(xdev);
    if (err < 0) {
        pr_warn("qdma_init failed %d.\n", err);
        qdma_device_cleanup(xdev);
    }

    pr_info("VF qconf set with qbase: %d, qmax: %d", qbase, qmax);
    return err;
}
#endif

/* MD:****************************************************************************/
/* MD:*
 * sgl_unmap() - unmap the sg list from host pages
 *
 * @param[in]	pdev:	pointer to struct pci_dev
 * @param[in]	sg:		qdma sg request list
 * @param[in]	sgcnt:	number of sg lists
 * @param[in]	dir:	direction of the DMA transfer
 *			DMA_BIDIRECTIONAL = 0,	DMA_TO_DEVICE = 1,
 *			DMA_FROM_DEVICE = 2, DMA_NONE = 3,
 *
 * @return	0: success
 * @return	<0: error
 *****************************************************************************/
void sgl_unmap(struct pci_dev *pdev, struct qdma_sw_sg *sg, unsigned int sgcnt,
		 enum dma_data_direction dir)
{
	int i;

	// MD: Loop through each scatter-gather (sg) entry to unmap it
	for (i = 0; i < sgcnt; i++, sg++) {
		// MD: Check if the page pointer is NULL, if so, exit the loop
		if (!sg->pg)
			break;

		// MD: If dma_addr is set, unmap the page and reset dma_addr
		if (sg->dma_addr) {
			dma_unmap_page(&pdev->dev, sg->dma_addr - sg->offset,
							PAGE_SIZE, dir);
			sg->dma_addr = 0UL;  // MD: Reset the dma address to zero
			printf("Unmapped sg entry %d, reset dma_addr to 0.\n", i);  // MD: Debug print
		}
	}
}

/* MD:****************************************************************************/
/* MD:*
 * sgl_map() - map sg list to host pages
 *
 * @param[in]	pdev:	pointer to struct pci_dev
 * @param[in]	sg:		qdma sg request list
 * @param[in]	sgcnt:	number of sg lists
 * @param[in]	dir:	direction of the DMA transfer
 *			DMA_BIDIRECTIONAL = 0,	DMA_TO_DEVICE = 1,
 *			DMA_FROM_DEVICE = 2, DMA_NONE = 3,
 *
 * @return	0: success
 * @return	<0: error
 *****************************************************************************/
int sgl_map(struct pci_dev *pdev, struct qdma_sw_sg *sgl, unsigned int sgcnt,
		enum dma_data_direction dir)
{
	int i;
	struct qdma_sw_sg *sg = sgl;

	// MD: Map each sg entry to a DMA page
	for (i = 0; i < sgcnt; i++, sg++) {
		// MD: Map the page to DMA address
		sg->dma_addr = dma_map_page(&pdev->dev, sg->pg, 0, PAGE_SIZE, dir);
		if (unlikely(dma_mapping_error(&pdev->dev, sg->dma_addr))) {
			pr_err("Mapping error for sg %d, length %u.\n", i, sg->len);  // MD: Error print
			// MD: If mapping fails, unmap all previous successfully mapped entries
			if (i)
				sgl_unmap(pdev, sgl, i, dir);
			return -EIO;  // MD: Return input/output error
		}
		sg->dma_addr += sg->offset;  // MD: Adjust dma_addr by the offset
		printf("Mapped sg entry %d to dma_addr %lu.\n", i, sg->dma_addr);  // MD: Debug print
	}

	return 0;  // MD: Return success
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_request_submit() - submit a scatter-gather list of data for dma
 * operation (for both read and write)
 * This is a callback function called from upper layer(character device)
 * to handle the read/write request on the queues
 *
 * @param[in]	dev_hndl:	hndl retured from qdma_device_open()
 * @param[in]	id:			queue index
 * @param[in]	req:		qdma request
 *
 * @return	# of bytes transferred
 * @return	<0: error
 *****************************************************************************/
ssize_t qdma_request_submit(unsigned long dev_hndl, unsigned long id,
			struct qdma_request *req)
{
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;
	struct qdma_descq *descq;
	struct qdma_sgt_req_cb *cb;
	enum dma_data_direction dir;
	int wait = 0;
	int rv = 0;

	// MD: Validate device handle
	if (!xdev) {
		pr_err("dev_hndl is NULL");
		return -EINVAL;
	}

	// MD: Check if the device handle is valid
	if (xdev_check_hndl(__func__, xdev->conf.pdev, dev_hndl) < 0) {
		pr_err("Invalid dev_hndl passed");
		return -EINVAL;
	}

	// MD: Validate request pointer
	if (!req) {
		pr_err("req is NULL");
		return -EINVAL;
	}

	// MD: Determine if waiting for completion is needed
	wait = req->fp_done ? 0 : 1;

	// MD: Retrieve and validate the descriptor queue
	descq = qdma_device_get_descq_by_id(xdev, id, NULL, 0, 1);
	if (!descq) {
		pr_err("Invalid qid(%ld)", id);
		return -EINVAL;
	}

	// MD: Check if the operation is on a completion queue
	if (descq->conf.q_type == Q_CMPT) {
		pr_err("Error : Transfer initiated on completion queue\n");
		return -EINVAL;
	}

	// MD: Get the request callback structure
	cb = qdma_req_cb_get(req);

	// MD: Debug print for request details
	pr_debug("%s %s-%s, data len %u, sg cnt %u. ping_pong_en=%d\n",
		descq->conf.name, descq->conf.st ? "ST" : "MM",
		(descq->conf.q_type == Q_C2H) ? "C2H" : "H2C",
		req->count, req->sgcnt, descq->conf.ping_pong_en);

	// MD: Determine the direction of the DMA transfer
	dir = (descq->conf.q_type == Q_C2H) ?  DMA_FROM_DEVICE : DMA_TO_DEVICE;

	// MD: Validate the direction of the request
	if ((req->write && (descq->conf.q_type != Q_H2C)) ||
	    (!req->write && (descq->conf.q_type != Q_C2H))) {
		pr_err("%s: bad direction, %c.\n",
			descq->conf.name, req->write ? 'W' : 'R');
		return -EINVAL;
	}

	// MD: Reset the callback request structure
	memset(cb, 0, QDMA_REQ_OPAQUE_SIZE);
	// MD: Initialize the wait queue
	qdma_waitq_init(&cb->wq);

	// MD: Debug print for DMA mapping initiation
	pr_debug("%s: data len %u, ep 0x%llx, sgl 0x%p, sgl cnt %u, tm %u ms.\n",
		descq->conf.name, req->count, req->ep_addr, req->sgl,
		req->sgcnt, req->timeout_ms);

	// MD: Handle streaming mode C2H read operation
	if (descq->conf.st && (descq->conf.q_type == Q_C2H))
		return qdma_request_submit_st_c2h(xdev, descq, req);

	// MD: Map the scatter-gather list to DMA if not already done
	if (!req->dma_mapped) {
		rv = sgl_map(xdev->conf.pdev,  req->sgl, req->sgcnt, dir);
		if (rv < 0) {
			pr_err("%s map sgl %u failed, %u.\n",
				descq->conf.name, req->sgcnt, req->count);
			goto unmap_sgl;
		}
		cb->unmap_needed = 1;
	}

	// MD: Lock the descriptor queue and check its state
	lock_descq(descq);
	if (descq->q_state != Q_STATE_ONLINE) {
		unlock_descq(descq);
		pr_err("%s descq %s NOT online.\n",
			xdev->conf.name, descq->conf.name);
		rv = -EINVAL;
		goto unmap_sgl;
	}
	// MD: Add the request callback to the work queue
	qdma_work_queue_add(descq, cb);
	unlock_descq(descq);

	// MD: Debug print for request submission
	pr_debug("%s: cb 0x%p submitted.\n", descq->conf.name, cb);

	// MD: Process the scatter-gather transaction request
	qdma_descq_proc_sgt_request(descq);

	// MD: Handle completion waiting
	if (!wait)
		return 0;

	rv = qdma_request_wait_for_cmpl(xdev, descq, req);
	if (rv < 0)
		goto unmap_sgl;

	return cb->offset;

unmap_sgl:
	// MD: Unmap the scatter-gather list if necessary
	if (!req->dma_mapped)
		sgl_unmap(xdev->conf.pdev,  req->sgl, req->sgcnt, dir);

	return rv;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_batch_request_submit() - submit a batch of scatter-gather list of data
 * for dma operation (for both read and write)
 * This is a callback function called from upper layer(character device)
 * to handle the read/write request on the queues
 *
 * @param[in]	dev_hndl:	hndl retured from qdma_device_open()
 * @param[in]	id:			queue index
 * @param[in]	count:		Number of qdma requests
 * @param[in]	reqv:		qdma request vector
 *
 * @return	# of bytes transferred
 * @return	<0: error
 *****************************************************************************/
ssize_t qdma_batch_request_submit(unsigned long dev_hndl, unsigned long id,
			  unsigned long count, struct qdma_request **reqv)
{
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;
	struct qdma_descq *descq;
	struct qdma_sgt_req_cb *cb;
	enum dma_data_direction dir;
	int rv = 0;
	unsigned long i;
	struct qdma_request *req;
	int st_c2h = 0;

	// MD: Validate device handle
	if (!xdev) {
		pr_err("dev_hndl is NULL");
		return -EINVAL;
	}

	// MD: Verify device handle validity using internal check function
	if (xdev_check_hndl(__func__, xdev->conf.pdev, dev_hndl) < 0) {
		pr_err("Invalid dev_hndl passed");
		return -EINVAL;
	}

	// MD: Validate request vector pointer
	if (!reqv) {
		pr_err("reqv is NULL");
		return -EINVAL;
	}

	// MD: Get descriptor queue for the specified ID
	descq = qdma_device_get_descq_by_id(xdev, id, NULL, 0, 0);
	if (!descq) {
		pr_err("Invalid qid(%ld)", id);
		return -EINVAL;
	}

	// MD: Ensure operation is not on completion queue
	if (descq->conf.q_type == Q_CMPT) {
		pr_err("Error : Transfer initiated on completion queue\n");
		return -EINVAL;
	}

	// MD: Check if this is a streaming C2H operation
	st_c2h = (descq->conf.st && (descq->conf.q_type == Q_C2H)) ? 1 : 0;
	pr_debug("Queue type: %s, st_c2h=%d\n", 
		st_c2h ? "Streaming C2H" : "Other", st_c2h);

	// MD: Determine DMA transfer direction
	dir = (descq->conf.q_type == Q_C2H) ? DMA_FROM_DEVICE : DMA_TO_DEVICE;

	// MD: Get first request from vector for validation
	req = reqv[0];
	
	// MD: Validate request direction matches queue type
	if ((req->write && (descq->conf.q_type != Q_H2C))
			|| (!req->write && (descq->conf.q_type != Q_C2H))) {
		pr_err("%s: bad direction, %c.\n", descq->conf.name,
				req->write ? 'W' : 'R');
		return -EINVAL;
	}

	// MD: Ensure callback function is provided
	if (!req->fp_done) {
		pr_err("%s: missing fp_done.\n", descq->conf.name);
		return -EINVAL;
	}

	// MD: Handle streaming C2H requests
	if (st_c2h) {
		pr_debug("Processing %lu streaming C2H requests\n", count);
		for (i = 0; i < count; i++) {
			req = reqv[i];
			cb = qdma_req_cb_get(req);
			memset(cb, 0, QDMA_REQ_OPAQUE_SIZE);

			rv = qdma_request_submit_st_c2h(xdev, descq, req);
			if ((rv < 0) || (rv == req->count)) {
                pr_debug("Request %lu completed with status %d\n", i, rv);
				req->fp_done(req, rv, rv);
            }
		}
		return 0;
	} else {
		struct pci_dev *pdev = xdev->conf.pdev;

		// MD: Handle non-streaming requests
		for (i = 0; i < count; i++) {
			req = reqv[i];
			cb = qdma_req_cb_get(req);
			memset(cb, 0, QDMA_REQ_OPAQUE_SIZE);

			// MD: Map the scatter-gather list to DMA if not already done
			if (!req->dma_mapped) {
				rv = sgl_map(pdev, req->sgl, req->sgcnt, dir);
				if (unlikely(rv < 0)) {
					pr_err("%s map sgl %u failed, %u.\n",
						descq->conf.name,
						req->sgcnt,
						req->count);
					req->fp_done(req, 0, rv);
				}
				cb->unmap_needed = 1;
			}
		}
	}

	// MD: Lock the descriptor queue and check its state
	lock_descq(descq);
	if (unlikely(descq->q_state != Q_STATE_ONLINE)) {
		unlock_descq(descq);
		pr_err("%s descq %s NOT online.\n", xdev->conf.name,
				descq->conf.name);
		return -EINVAL;
	}

	// MD: Add each request callback to the work queue
	for (i = 0; i < count; i++) {
		req = reqv[i];
		cb = qdma_req_cb_get(req);
		list_add_tail(&cb->list, &descq->work_list);
	}
	unlock_descq(descq);

	// MD: Process the scatter-gather transaction requests
	qdma_descq_proc_sgt_request(descq);

	return 0;
}

#ifdef TANDEM_BOOT_SUPPORTED
/* MD:****************************************************************************/
/* MD:*
 * qdma_init_st_ctxt() - Initialize the QDMA ST context
 *
 * @param dev_hndl: Device handle returned from qdma_device_open()
 * @param buflen: Input buffer length
 * @param buf: Error message buffer, can be NULL/0 (i.e., optional)
 *
 * @return 0: Success
 * @return <0: Error
 *****************************************************************************/
int qdma_init_st_ctxt(unsigned long dev_hndl, char *buf, int buflen)
{
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;
	int rv = 0;

	// MD: Ensure that the input buffer is not empty, else return error
	if (!buf || !buflen) {
		pr_err("Invalid argument: buf=%p, buflen=%d\n", buf, buflen);
		return -EINVAL;
	}

	// MD: Validate the device handle
	if (!xdev) {
		pr_err("dev_hndl is NULL");
		snprintf(buf, buflen, "dev_hndl is NULL");
		return -EINVAL;
	}

	// MD: Check if the device handle is valid
	if (xdev_check_hndl(__func__, xdev->conf.pdev, dev_hndl) < 0) {
		pr_err("Invalid dev_hndl passed");
		snprintf(buf, buflen, "Invalid dev_hndl passed\n");
		return -EINVAL;
	}

	// MD: Check if the device is a supported Versal Hard IP type
	if ((xdev->version_info.ip_type == QDMA_VERSAL_HARD_IP) &&
		((xdev->version_info.device_type == QDMA_DEVICE_VERSAL_CPM4) ||
		(xdev->version_info.device_type == QDMA_DEVICE_VERSAL_CPM5))) {
		// MD: Check if the ST context initialization function is available
		if (xdev->hw.qdma_init_st_ctxt == NULL) {
			pr_err("Err: Feature not supported\n");
			snprintf(buf, buflen, "Err: Feature not supported\n");
			return -EPERM;
		}

		// MD: Initialize the streaming context
		rv = xdev->hw.qdma_init_st_ctxt((void *)dev_hndl);

		// MD: Check the result of the initialization
		if (rv == QDMA_SUCCESS) {
			snprintf(buf, buflen,
			"qdma%05x : Streaming Enabled successfully\n",
			xdev->conf.bdf);
			pr_info("Streaming Enabled successfully\n");
		} else {
			snprintf(buf, buflen,
			"qdma%05x : Failed to Enable Streaming\n",
			xdev->conf.bdf);
			pr_info("Failed to Enable Streaming\n");
			return -EINVAL;
		}
	} else {
		// MD: Log that streaming was enabled during initialization
		snprintf(buf, buflen,
			"qdma%05x : Streaming Enabled during Initialization\n",
			xdev->conf.bdf);
		pr_info("Streaming Enabled during Initialization\n");
	}

	return rv;
}
#endif

/* MD:****************************************************************************/
/* MD:*
 * libqdma_init() - Initialize the QDMA core library
 *
 * @param[in] num_threads: Number of threads to be created each for request
 *  processing and writeback processing
 *
 * @return 0: Success
 * @return <0: Error
 *****************************************************************************/
int libqdma_init(unsigned int num_threads, void *debugfs_root)
{
	int ret;

	// MD: Ensure the size of qdma scatter gather request size is valid
	if (sizeof(struct qdma_sgt_req_cb) > QDMA_REQ_OPAQUE_SIZE) {
		pr_err("DMA req. opaque data size too big %lu > %lu.\n",
			sizeof(struct qdma_sgt_req_cb), QDMA_REQ_OPAQUE_SIZE);
		return -1;
	}
	if (sizeof(struct qdma_flq) > QDMA_FLQ_SIZE) {
		pr_err("QDMA FLQ data size too big %lu > %d",
		       sizeof(struct qdma_flq), QDMA_FLQ_SIZE);
		return -1;
	}

	// MD: Create the QDMA threads
	ret = qdma_threads_create(num_threads);
	if (ret < 0) {
		pr_err("qdma_threads_create failed for num_thread=%d",
			       num_threads);
		return ret;
	}

#ifdef DEBUGFS
	// MD: Initialize debugfs if root is provided
	if (debugfs_root) {
		qdma_debugfs_root = debugfs_root;
		qdma_debufs_cleanup = false;
		return 0;
	}

	// MD: Initialize debugfs
	ret = qdma_debugfs_init(&qdma_debugfs_root);
	if (ret < 0) {
		pr_err("qdma_debugfs_init failed for num_thread=%d",
				num_threads);
		return ret;
	}
#endif
	return 0;
}

/* MD:****************************************************************************/
/* MD:*
 * libqdma_exit() - Cleanup the QDMA core library before exiting
 *
 * @return none
 *****************************************************************************/
void libqdma_exit(void)
{
#ifdef DEBUGFS
	// MD: Cleanup debugfs if needed
	if (qdma_debufs_cleanup)
		qdma_debugfs_exit(qdma_debugfs_root);
#endif
	// MD: Destroy the QDMA threads
	qdma_threads_destroy();
}

#ifdef __LIBQDMA_MOD__
/* MD:* For module support only */
#include "version.h"

static char version[] =
	DRV_MODULE_DESC " " DRV_MODULE_NAME " v" DRV_MODULE_VERSION "\n";

MODULE_AUTHOR("Xilinx, Inc.");
MODULE_DESCRIPTION(DRV_MODULE_DESC);
MODULE_VERSION(DRV_MODULE_VERSION);
MODULE_LICENSE("Dual BSD/GPL");

/* MD:****************************************************************************/
/* MD:*
 * libqdma_mod_init() - libqdma module initialization routine
 *
 * Returns: none
 *****************************************************************************/
static int __init libqdma_mod_init(void)
{
	pr_info("%s", version);

	// MD: Invoke libqdma_init to initialize the libqdma library
	return libqdma_init();
}

/* MD:****************************************************************************/
/* MD:*
 * libqdma_mod_exit() - Cleanup the QDMA core library before exiting
 *
 * Returns: none
 *****************************************************************************/
static void __exit libqdma_mod_exit(void)
{
	// MD: Invoke libqdma_exit to cleanup the libqdma library resources
	libqdma_exit();
}

module_init(libqdma_mod_init);
module_exit(libqdma_mod_exit);
#endif /* MD: ifdef __LIBQDMA_MOD__ */
