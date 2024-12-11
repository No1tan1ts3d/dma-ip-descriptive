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

#define pr_fmt(fmt)	KBUILD_MODNAME ":%s: " fmt, __func__

#include <linux/kernel.h>
#include <linux/pci.h>
#include "qdma_device.h"
#include "qdma_descq.h"
#include "qdma_intr.h"
#include "qdma_regs.h"
#include "qdma_context.h"
#include "qdma_access_common.h"
#include "qdma_mbox_protocol.h"

/* MD: *
 * make_intr_context - Initializes the interrupt context for the DMA device.
 * @xdev: Pointer to the DMA device structure.
 * @ctxt: Pointer to the array of interrupt context structures to be initialized.
 *
 * This function checks the driver mode and sets up the interrupt coalescing
 * context for the DMA device. It returns 0 on success or a negative error code
 * on failure.
 */
static int make_intr_context(struct xlnx_dma_dev *xdev,
			     struct qdma_indirect_intr_ctxt *ctxt)
{
	int i;

	// MD:  Check if the driver mode is valid (either INDIRECT_INTR_MODE or AUTO_MODE)
	if ((xdev->conf.qdma_drv_mode != INDIRECT_INTR_MODE) &&
			(xdev->conf.qdma_drv_mode != AUTO_MODE)) {
		pr_err("Invalid driver mode: %d", xdev->conf.qdma_drv_mode);
		return -EINVAL; // MD:  Return error if mode is invalid
	}

	// MD:  Program the coalescing context
	// MD:  Loop through the number of vectors defined for interrupt context
	for (i = 0; i < QDMA_NUM_DATA_VEC_FOR_INTR_CXT; i++) {
		struct intr_coal_conf *entry = (xdev->intr_coal_list + i);

		// MD:  Initialize the interrupt context entry
		ctxt[i].valid = 1; // MD:  Mark the entry as valid
		ctxt[i].vec = entry->vec_id; // MD:  Set the vector ID
		ctxt[i].baddr_4k = entry->intr_ring_bus; // MD:  Set the base address
		ctxt[i].color = entry->color; // MD:  Set the color
		ctxt[i].page_size = xdev->conf.intr_rngsz; // MD:  Set the page size
		ctxt[i].func_id = xdev->func_id; // MD:  Set the function ID

		// MD:  Debug print statement to track the initialization of each context
		pr_info("Initialized context[%d]: valid=%d, vec=%d, baddr_4k=%lx, color=%d, page_size=%d, func_id=%d\n",
			i, ctxt[i].valid, ctxt[i].vec, ctxt[i].baddr_4k, ctxt[i].color, ctxt[i].page_size, ctxt[i].func_id);
	}

	return 0; // MD:  Return success
}

#ifndef __QDMA_VF__
/* MD: *
 * make_sw_context - Initialize the software context for a QDMA descriptor queue.
 * @descq: Pointer to the descriptor queue structure.
 * @sw_ctxt: Pointer to the software context structure to be populated.
 *
 * This function initializes the software context based on the configuration
 * of the QDMA descriptor queue. It also handles specific configurations
 * based on queue type and mode.
 */
static int make_sw_context(struct qdma_descq *descq,
                           struct qdma_descq_sw_ctxt *sw_ctxt)
{
    // MD:  Clear the software context memory
    memset(sw_ctxt, 0, sizeof(struct qdma_descq_sw_ctxt));
    printf("Initializing software context...\n");

    // MD:  Configure interrupt vector based on driver mode
    if ((descq->xdev->conf.qdma_drv_mode == INDIRECT_INTR_MODE) ||
        (descq->xdev->conf.qdma_drv_mode == AUTO_MODE)) {
        sw_ctxt->vec = get_intr_ring_index(descq->xdev, descq->intr_id);
        sw_ctxt->intr_aggr = 0x01;  // MD:  Enable interrupt aggregation
        printf("Indirect or Auto mode: intr_id=%d, vec=%d\n", descq->intr_id, sw_ctxt->vec);
    } else {
        sw_ctxt->vec = descq->intr_id;
        printf("Direct mode: intr_id=%d, vec=%d\n", descq->intr_id, sw_ctxt->vec);
    }

    // MD:  Set other fields in the software context
    sw_ctxt->ring_bs_addr = descq->desc_bus;
    sw_ctxt->wbi_chk = descq->conf.cmpl_status_pend_chk;
    sw_ctxt->wbi_intvl_en = descq->conf.cmpl_status_acc_en;
    sw_ctxt->rngsz_idx = descq->conf.desc_rng_sz_idx;
    sw_ctxt->bypass = descq->conf.desc_bypass;
    sw_ctxt->wbk_en = descq->conf.wb_status_en;
    sw_ctxt->irq_en = descq->conf.irq_en;
    sw_ctxt->is_mm = ~descq->conf.st;
    sw_ctxt->qen = 1;  // MD:  Queue enable
    printf("Context initialized with ring_bs_addr=0x%lx, qen=%d\n", descq->desc_bus, sw_ctxt->qen);

    // MD:  Handle descriptor bypass and size configuration
    if (descq->conf.desc_bypass && (descq->conf.sw_desc_sz == DESC_SZ_64B)) {
        sw_ctxt->desc_sz = descq->conf.sw_desc_sz;
        printf("Bypass mode with 64B descriptor size.\n");
    } else {
        sw_ctxt->fetch_max = FETCH_MAX_NUM;
        if (!descq->conf.st) { // MD:  Memory-mapped H2C/C2H
            sw_ctxt->desc_sz = DESC_SZ_32B;
            sw_ctxt->mm_chn = descq->channel;
            sw_ctxt->host_id = descq->channel;
            printf("MM H2C/C2H: Channel=%d, Descriptor Size=32B.\n", descq->channel);
        } else if (descq->conf.q_type == Q_C2H) {  // MD:  Streamed C2H
            sw_ctxt->frcd_en = descq->conf.fetch_credit;
            sw_ctxt->desc_sz = DESC_SZ_8B;
            printf("Streamed C2H: Descriptor Size=8B.\n");
        } else if (descq->conf.q_type == Q_H2C) { // MD:  Streamed H2C
            sw_ctxt->frcd_en = descq->conf.fetch_credit;
            sw_ctxt->desc_sz = DESC_SZ_16B;
            printf("Streamed H2C: Descriptor Size=16B.\n");
        } else {
            sw_ctxt->desc_sz = DESC_SZ_16B; // MD:  Default case
            printf("Default descriptor size set to 16B.\n");
        }
    }

    // MD:  Set function ID and IRQ configurations
    sw_ctxt->fnc_id = descq->xdev->func_id;
    sw_ctxt->irq_arm = descq->conf.irq_en;
    printf("Function ID=%d, IRQ Arm=%d\n", sw_ctxt->fnc_id, sw_ctxt->irq_arm);

    // MD:  Adjustments for specific configurations
    if (descq->conf.st && (descq->conf.q_type == Q_C2H)) {
        sw_ctxt->irq_en = 0;
        sw_ctxt->irq_arm = 0;
        sw_ctxt->wbk_en = 0;
        sw_ctxt->wbi_chk = 0;
        printf("Streamed C2H adjustments: IRQ disabled, WBI disabled.\n");
    }

    // MD:  Disable marker response where applicable
    if ((!descq->conf.desc_bypass) &&
        ((!descq->conf.st) || (descq->conf.q_type == Q_H2C))) {
        sw_ctxt->mrkr_dis = 1;
        printf("Marker response disabled.\n");
    }

#ifdef ERR_DEBUG
    // MD:  Inject errors for debugging if enabled
    if (descq->induce_err & (1 << param)) {
        sw_ctxt->fnc_id = 0xFFF;
        pr_info("Induced error: Context Command Error\n");
        printf("Debug: Induced error injected.\n");
    }
#endif

    printf("Software context creation complete.\n");
    return 0;
}

static int make_qid2vec_context(struct qdma_descq *descq,
                                struct qdma_qid2vec *cntxt)
{
    u32 vec_num = 0; // MD:  Vector number for interrupts
    u32 en_coal = 0; // MD:  Enable interrupt coalescing
    struct xlnx_dma_dev *xdev = NULL;

    // MD:  Validate input parameters
    BUG_ON(!descq);
    BUG_ON(!cntxt);

    xdev = descq->xdev;
    vec_num = descq->intr_id;

    // MD:  Initialize the qid2vec context structure
    memset(cntxt, 0, sizeof(struct qdma_qid2vec));
    printf("Creating qid2vec context for queue id %d\n", descq->queue_id);

    // MD:  Configure interrupt coalescing based on driver mode
    if ((xdev->conf.qdma_drv_mode == INDIRECT_INTR_MODE) ||
        (xdev->conf.qdma_drv_mode == AUTO_MODE)) {
        vec_num = get_intr_ring_index(xdev, descq->intr_id);
        en_coal = 1;
        printf("Indirect/Auto interrupt mode: vec_num=%d, en_coal=%d\n", vec_num, en_coal);
    }

    // MD:  Set C2H or H2C-specific context fields
    if (descq->conf.q_type == Q_C2H) {
        cntxt->c2h_en_coal = en_coal;
        cntxt->c2h_vector = vec_num;
        printf("C2H queue: c2h_vector=%d, c2h_en_coal=%d\n", cntxt->c2h_vector, cntxt->c2h_en_coal);
    } else if (descq->conf.q_type == Q_H2C) {
        cntxt->h2c_en_coal = en_coal;
        cntxt->h2c_vector = vec_num;
        printf("H2C queue: h2c_vector=%d, h2c_en_coal=%d\n", cntxt->h2c_vector, cntxt->h2c_en_coal);
    }

    // MD:  Debug output for context values
    pr_debug("qid2vec context:\n"
             "c2h_vector=%x\nc2h_en_coal=%x\n"
             "h2c_vector=%x\nh2c_en_coal=%x\n",
             cntxt->c2h_vector, cntxt->c2h_en_coal,
             cntxt->h2c_vector, cntxt->h2c_en_coal);

    return 0;
}

/* MD:  Prefetch context setup for streamed queues */
static int make_prefetch_context(struct qdma_descq *descq,
                                 struct qdma_descq_prefetch_ctxt *pfetch_ctxt)
{
    // MD:  Validate input parameters
    BUG_ON(!descq);
    BUG_ON(!pfetch_ctxt);

    // MD:  Initialize the prefetch context structure
    memset(pfetch_ctxt, 0, sizeof(struct qdma_descq_prefetch_ctxt));
    printf("Creating prefetch context for queue id %d\n", descq->queue_id);

    // MD:  Configure prefetch context fields
    pfetch_ctxt->valid = 1; // MD:  Mark context as valid
    pfetch_ctxt->bypass = descq->conf.pfetch_bypass;
    pfetch_ctxt->bufsz_idx = descq->conf.c2h_buf_sz_idx;
    pfetch_ctxt->pfch_en = descq->conf.pfetch_en;

    printf("Prefetch context: bypass=%d, bufsz_idx=%d, pfch_en=%d\n",
           pfetch_ctxt->bypass, pfetch_ctxt->bufsz_idx, pfetch_ctxt->pfch_en);

    return 0;
}

/* MD:  Writeback context setup for streamed C2H */
static int make_cmpt_context(struct qdma_descq *descq,
                             struct qdma_descq_cmpt_ctxt *cmpt_ctxt)
{
    int ring_index;

    // MD:  Validate input parameters
    BUG_ON(!descq);
    BUG_ON(!cmpt_ctxt);

    // MD:  Initialize the completion context structure
    memset(cmpt_ctxt, 0, sizeof(struct qdma_descq_cmpt_ctxt));
    printf("Creating completion context for queue id %d\n", descq->queue_id);

    // MD:  Set completion context fields
    cmpt_ctxt->en_stat_desc = descq->conf.cmpl_stat_en;
    cmpt_ctxt->en_int = descq->conf.cmpl_en_intr;
    cmpt_ctxt->trig_mode = descq->conf.cmpl_trig_mode;
    cmpt_ctxt->fnc_id = descq->xdev->func_id;
    cmpt_ctxt->timer_idx = descq->conf.cmpl_timer_idx;
    cmpt_ctxt->counter_idx = descq->conf.cmpl_cnt_th_idx;
    cmpt_ctxt->color = 1; // MD:  Enable color bit
    cmpt_ctxt->ringsz_idx = descq->conf.cmpl_rng_sz_idx;

    cmpt_ctxt->bs_addr = descq->desc_cmpt_bus;
    cmpt_ctxt->desc_sz = descq->conf.cmpl_desc_sz;
    cmpt_ctxt->full_upd = descq->conf.adaptive_rx;
    cmpt_ctxt->valid = 1; // MD:  Mark context as valid
    cmpt_ctxt->ovf_chk_dis = descq->conf.cmpl_ovf_chk_dis;

    // MD:  Configure interrupt vector and aggregation based on driver mode
    if ((descq->xdev->conf.qdma_drv_mode == INDIRECT_INTR_MODE) ||
        (descq->xdev->conf.qdma_drv_mode == AUTO_MODE)) {
        ring_index = get_intr_ring_index(descq->xdev, descq->intr_id);
        cmpt_ctxt->vec = ring_index;
        cmpt_ctxt->int_aggr = 1;
        printf("Indirect/Auto interrupt mode: vec=%d, int_aggr=%d\n", cmpt_ctxt->vec, cmpt_ctxt->int_aggr);
    } else {
        cmpt_ctxt->vec = descq->intr_id;
        printf("Direct interrupt mode: vec=%d\n", cmpt_ctxt->vec);
    }

    return 0;
}
#endif

#ifdef __QDMA_VF__

/* MD:  Setup interrupt context for a Virtual Function (VF) */
int qdma_descq_context_setup(struct qdma_descq *descq)
{
    struct xlnx_dma_dev *xdev = descq->xdev; // MD:  Get the device structure from descq
    struct mbox_msg *m = qdma_mbox_msg_alloc(); // MD:  Allocate memory for a mailbox message
    struct mbox_descq_conf descq_conf; // MD:  Descriptor queue configuration structure
    int rv; // MD:  Return value
    enum mbox_cmpt_ctxt_type cmpt_ctxt_type = QDMA_MBOX_CMPT_CTXT_NONE; // MD:  Completion context type

    // MD:  Check for memory allocation failure
    if (!m)
        return -ENOMEM;

    // MD:  Initialize the descriptor queue configuration structure
    memset(&descq_conf, 0, sizeof(struct mbox_descq_conf));

    // MD:  Set the descriptor bus and completion ring bus addresses
    descq_conf.ring_bs_addr = descq->desc_bus;
    descq_conf.cmpt_ring_bs_addr = descq->desc_cmpt_bus;

    // MD:  Configure descriptor and queue settings
    descq_conf.en_bypass = descq->conf.desc_bypass;
    descq_conf.irq_arm = descq->conf.irq_en;
    descq_conf.wbi_intvl_en = descq->conf.cmpl_status_acc_en;
    descq_conf.wbi_chk = descq->conf.cmpl_status_pend_chk;
    descq_conf.at = descq->conf.at;
    descq_conf.wbk_en = descq->conf.wb_status_en;
    descq_conf.irq_en = descq->conf.irq_en;
    descq_conf.pfch_en = descq->conf.pfetch_en;
    descq_conf.en_bypass_prefetch = descq->conf.pfetch_bypass;
    descq_conf.dis_overflow_check = descq->conf.cmpl_ovf_chk_dis;
    descq_conf.cmpt_int_en = descq->conf.cmpl_en_intr;
    descq_conf.cmpl_stat_en = descq->conf.cmpl_stat_en;

    // MD:  Determine descriptor size based on queue type and bypass settings
    if (descq->conf.desc_bypass && (descq->conf.sw_desc_sz == DESC_SZ_64B)) {
        descq_conf.desc_sz = descq->conf.sw_desc_sz;
    } else {
        if (descq->conf.q_type != Q_CMPT) {
            if (!descq->conf.st) // MD:  Memory-mapped queues (H2C or C2H)
                descq_conf.desc_sz = DESC_SZ_32B;
            else if (descq->conf.q_type == Q_C2H) { // MD:  Streaming C2H
                descq_conf.desc_sz = DESC_SZ_8B;
                descq_conf.forced_en = descq->conf.fetch_credit;
            } else // MD:  Streaming H2C
                descq_conf.desc_sz = DESC_SZ_16B;
        }
    }

    // MD:  Configure completion context parameters
    descq_conf.cmpt_desc_sz = descq->conf.cmpl_desc_sz;
    descq_conf.triggermode = descq->conf.cmpl_trig_mode;
    descq_conf.cmpt_at = descq->conf.at;
    descq_conf.cmpt_color = 1;
    descq_conf.cmpt_full_upd = 0;
    descq_conf.func_id = descq->xdev->func_id;

    // MD:  Set threshold and size parameters
    descq_conf.cnt_thres = xdev->csr_info.c2h_cnt_th[descq->conf.cmpl_cnt_th_idx];
    descq_conf.timer_thres = xdev->csr_info.c2h_timer_cnt[descq->conf.cmpl_timer_idx];
    descq_conf.ringsz = descq->conf.rngsz;
    descq_conf.bufsz = descq->conf.c2h_bufsz;
    descq_conf.cmpt_ringsz = descq->conf.rngsz_cmpt;

    // MD:  Configure interrupt settings based on driver mode
    if ((xdev->conf.qdma_drv_mode == INDIRECT_INTR_MODE) ||
        (xdev->conf.qdma_drv_mode == AUTO_MODE)) {
        int ring_index = get_intr_ring_index(descq->xdev, descq->intr_id);
        descq_conf.intr_id = ring_index & 0xFFF;
        descq_conf.intr_aggr = 1; // MD:  Enable interrupt aggregation
    } else {
        descq_conf.intr_id = descq->intr_id;
    }

    // MD:  Determine completion context type based on queue type and mode
    if (!descq->conf.st) {
        if (descq->conf.q_type == Q_CMPT)
            cmpt_ctxt_type = QDMA_MBOX_CMPT_CTXT_ONLY;
        else
            cmpt_ctxt_type = QDMA_MBOX_CMPT_CTXT_NONE;
    } else if (descq->conf.q_type == Q_C2H) {
        cmpt_ctxt_type = QDMA_MBOX_CMPT_WITH_ST;
    }

    // MD:  Compose the mailbox message to write the queue context
    qdma_mbox_compose_vf_qctxt_write(xdev->func_id, descq->qidx_hw,
                                     descq->conf.st, descq->conf.q_type,
                                     cmpt_ctxt_type, &descq_conf, m->raw);

    // MD:  Send the mailbox message
    rv = qdma_mbox_msg_send(xdev, m, 1, QDMA_MBOX_MSG_TIMEOUT_MS);
    if (rv < 0) {
        if (rv != -ENODEV)
            pr_err("%s, qid_hw 0x%x, %s mbox failed %d.\n",
                   xdev->conf.name, descq->qidx_hw, descq->conf.name, rv);
        goto err_out;
    }

    // MD:  Check the response status
    rv = qdma_mbox_vf_response_status(m->raw);
    if (rv < 0) {
        pr_err("mbox_vf_response_status failed, err = %d", rv);
        rv = -EINVAL;
    }

err_out:
    qdma_mbox_msg_free(m); // MD:  Free the mailbox message
    return rv;
}

int qdma_descq_context_setup(struct qdma_descq *descq)
{
	// MD:  Retrieve the xlnx_dma_dev structure associated with the descriptor queue.
	struct xlnx_dma_dev *xdev = descq->xdev;
	// MD:  Allocate a mailbox message for communication with the hardware.
	struct mbox_msg *m = qdma_mbox_msg_alloc();
	struct mbox_descq_conf descq_conf;
	int rv;
	enum mbox_cmpt_ctxt_type cmpt_ctxt_type = QDMA_MBOX_CMPT_CTXT_NONE;

	// MD:  Check if mailbox message allocation failed.
	if (!m) {
		pr_err("Failed to allocate mailbox message for descriptor queue context setup.\n");
		return -ENOMEM;
	}

	// MD:  Zero out the descriptor queue configuration structure.
	memset(&descq_conf, 0, sizeof(struct mbox_descq_conf));

	// MD:  Set the base address of the descriptor ring and completion ring.
	descq_conf.ring_bs_addr = descq->desc_bus;
	descq_conf.cmpt_ring_bs_addr = descq->desc_cmpt_bus;

	// MD:  Set various flags for descriptor queue configuration.
	descq_conf.en_bypass = descq->conf.desc_bypass;
	descq_conf.irq_arm = descq->conf.irq_en;
	descq_conf.wbi_intvl_en = descq->conf.cmpl_status_acc_en;
	descq_conf.wbi_chk = descq->conf.cmpl_status_pend_chk;
	descq_conf.at = descq->conf.at;
	descq_conf.wbk_en = descq->conf.wb_status_en;
	descq_conf.irq_en = descq->conf.irq_en;
	descq_conf.pfch_en = descq->conf.pfetch_en;
	descq_conf.en_bypass_prefetch = descq->conf.pfetch_bypass;
	descq_conf.dis_overflow_check = descq->conf.cmpl_ovf_chk_dis;
	descq_conf.cmpt_int_en = descq->conf.cmpl_en_intr;
	descq_conf.cmpl_stat_en = descq->conf.cmpl_stat_en;

	// MD:  Set descriptor size based on the configuration.
	if (descq->conf.desc_bypass &&
			(descq->conf.sw_desc_sz == DESC_SZ_64B)) {
		descq_conf.desc_sz = descq->conf.sw_desc_sz;
	} else {
		if (descq->conf.q_type != Q_CMPT) {
			if (!descq->conf.st) { /* MD:  mm h2c/c2h */
				descq_conf.desc_sz = DESC_SZ_32B;
			} else if (descq->conf.q_type) { /* MD:  st c2h */
				descq_conf.desc_sz = DESC_SZ_8B;
				descq_conf.forced_en = descq->conf.fetch_credit;
			} else { /* MD:  st h2c */
				descq_conf.desc_sz = DESC_SZ_16B;
			}
		}
	}

	// MD:  Set additional configuration parameters.
	descq_conf.cmpt_desc_sz = descq->conf.cmpl_desc_sz;
	descq_conf.triggermode = descq->conf.cmpl_trig_mode;
	descq_conf.cmpt_at = descq->conf.at;
	descq_conf.cmpt_color = 1;
	descq_conf.cmpt_full_upd = 0;
	descq_conf.func_id = descq->xdev->func_id;
	descq_conf.cnt_thres = xdev->csr_info.c2h_cnt_th[descq->conf.cmpl_cnt_th_idx];
	descq_conf.timer_thres = xdev->csr_info.c2h_timer_cnt[descq->conf.cmpl_timer_idx];
	descq_conf.ringsz = descq->conf.rngsz;
	descq_conf.bufsz = descq->conf.c2h_bufsz;
	descq_conf.cmpt_ringsz = descq->conf.rngsz_cmpt;

	// MD:  Check the interrupt aggregation mode (INDIRECT_INTR_MODE or AUTO_MODE).
	if ((descq->xdev->conf.qdma_drv_mode == INDIRECT_INTR_MODE) ||
			(descq->xdev->conf.qdma_drv_mode == AUTO_MODE)) {
		int ring_index = get_intr_ring_index(descq->xdev, descq->intr_id);
		descq_conf.intr_id = ring_index & 0xFFF;
		descq_conf.intr_aggr = 1;
		pr_debug("Using interrupt aggregation mode. Ring index: %d\n", ring_index);
	} else {
		descq_conf.intr_id = descq->intr_id;
		pr_debug("Using direct interrupt mode. Interrupt ID: %d\n", descq_conf.intr_id);
	}

	// MD:  Determine the completion context type based on queue type and state.
	if (!descq->conf.st) {
		if (descq->conf.q_type == Q_CMPT) {
			cmpt_ctxt_type = QDMA_MBOX_CMPT_CTXT_ONLY;
		} else {
			cmpt_ctxt_type = QDMA_MBOX_CMPT_CTXT_NONE;
		}
	} else {
		if (descq->conf.q_type == Q_C2H) {
			cmpt_ctxt_type = QDMA_MBOX_CMPT_WITH_ST;
		}
	}

	// MD:  Compose the mailbox message for descriptor queue context write.
	pr_debug("Composing mailbox message for descriptor queue context write.\n");
	qdma_mbox_compose_vf_qctxt_write(xdev->func_id, descq->qidx_hw,
			descq->conf.st, descq->conf.q_type,
			cmpt_ctxt_type, &descq_conf, m->raw);

	// MD:  Send the composed message to the device.
	rv = qdma_mbox_msg_send(xdev, m, 1, QDMA_MBOX_MSG_TIMEOUT_MS);
	if (rv < 0) {
		pr_err("Failed to send mailbox message for descriptor queue context setup (Error: %d).\n", rv);
		goto err_out;
	}

	// MD:  Check the response status from the mailbox operation.
	rv = qdma_mbox_vf_response_status(m->raw);
	if (rv < 0) {
		pr_err("Mailbox response status failed with error %d.\n", rv);
		rv = -EINVAL;
	}

err_out:
	// MD:  Free the allocated mailbox message and return.
	qdma_mbox_msg_free(m);
	return rv;
}

int qdma_descq_context_dump(struct qdma_descq *descq, char *buf, int buflen)
{
	int rv = 0;
	int ring_index = -1;
	int ring_count = 0;
	int len = 0;
	struct qdma_descq_context queue_context;
	struct qdma_indirect_intr_ctxt intr_ctxt;

	// MD:  Read the descriptor queue context.
	pr_debug("Reading descriptor queue context for queue index %d.\n", descq->qidx_hw);
	rv = qdma_descq_context_read(descq->xdev, descq->qidx_hw,
			descq->conf.st, descq->conf.q_type, &queue_context);
	if (rv < 0) {
		pr_err("Failed to read queue context (Error: %d).\n", rv);
		return rv;
	}

	// MD:  Dump the queue context into the provided buffer.
	pr_debug("Dumping descriptor queue context to buffer.\n");
	rv = descq->xdev->hw.qdma_dump_queue_context(descq->xdev,
				descq->conf.st,
				(enum qdma_dev_q_type)descq->conf.q_type,
				&queue_context,
				buf, buflen);
	if (rv < 0) {
		pr_err("Failed to dump queue context (Error: %d).\n", rv);
		return descq->xdev->hw.qdma_get_error_code(rv);
	}
	len = rv;

	// MD:  If interrupt aggregation is enabled, read and dump the interrupt context.
	if ((descq->xdev->conf.qdma_drv_mode == INDIRECT_INTR_MODE) ||
			(descq->xdev->conf.qdma_drv_mode == AUTO_MODE)) {
		for (ring_count = 0; ring_count < QDMA_NUM_DATA_VEC_FOR_INTR_CXT; ring_count++) {
			ring_index = get_intr_ring_index(
						descq->xdev,
						(descq->xdev->dvec_start_idx +
								ring_count));

			pr_debug("Reading interrupt context for ring index %d.\n", ring_index);
			rv = qdma_intr_context_read(descq->xdev,
						ring_index, &intr_ctxt);
			if (rv < 0) {
				pr_err("Failed to read interrupt context for ring %d (Error: %d).\n",
						ring_index, rv);
				return rv;
			}

			pr_debug("Dumping interrupt context for ring index %d.\n", ring_index);
			rv = descq->xdev->hw.qdma_dump_intr_context(descq->xdev,
						&intr_ctxt, ring_index,
						buf + len, buflen - len);
			if (rv < 0) {
				pr_err("Failed to dump interrupt context (Error: %d).\n", rv);
				return descq->xdev->hw.qdma_get_error_code(rv);
			}
			len += rv;
		}
	}

	// MD:  Return the length of the dumped context.
	return len;
}

#else /* MD:  PF only */

int qdma_prog_intr_context(struct xlnx_dma_dev *xdev,
		struct mbox_msg_intr_ctxt *ictxt)
{
	int i = 0;
	int rv;
	int ring_index;
	struct qdma_indirect_intr_ctxt *ctxt;

	// MD:  Iterate over all interrupt rings defined in the context message.
	for (i = 0; i < ictxt->num_rings; i++) {
		ring_index = ictxt->ring_index_list[i];

		// MD:  Retrieve the context for the current ring.
		ctxt = &ictxt->ictxt[i];
		// MD:  Configure the interrupt context for the current ring (write operation).
		pr_debug("Writing interrupt context for ring %d\n", ring_index);
		rv = xdev->hw.qdma_indirect_intr_ctx_conf(xdev, ring_index,
							  ctxt,
							  QDMA_HW_ACCESS_WRITE);
		if (rv < 0) {
			pr_err("Interrupt context write failed for ring %d, error = %d\n", ring_index, rv);
			return xdev->hw.qdma_get_error_code(rv);
		}
	}

	// MD:  Return success after configuring all interrupt contexts.
	return 0;
}

int qdma_intr_context_setup(struct xlnx_dma_dev *xdev)
{
	struct qdma_indirect_intr_ctxt ctxt[QDMA_NUM_DATA_VEC_FOR_INTR_CXT];
	int i = 0;
	int rv;
	int ring_index;

	// MD:  Only proceed if indirect interrupt mode or auto mode is enabled.
	if ((xdev->conf.qdma_drv_mode != INDIRECT_INTR_MODE) &&
			(xdev->conf.qdma_drv_mode != AUTO_MODE)) {
		pr_debug("Skipping interrupt context setup as mode is not indirect or auto.\n");
		return 0;
	}

	// MD:  Initialize the context array to zero.
	memset(ctxt, 0, sizeof(struct qdma_indirect_intr_ctxt) *
	       QDMA_NUM_DATA_VEC_FOR_INTR_CXT);

	// MD:  Prepare the interrupt context for all vectors.
	pr_debug("Preparing interrupt context for all vectors.\n");
	rv = make_intr_context(xdev, ctxt);
	if (rv < 0) {
		pr_err("Failed to prepare interrupt context, error = %d\n", rv);
		return rv;
	}

	// MD:  Iterate over each vector to configure the interrupt context.
	for (i = 0; i <  QDMA_NUM_DATA_VEC_FOR_INTR_CXT; i++) {
		ring_index = get_intr_ring_index(xdev,
				(i + xdev->dvec_start_idx));

		// MD:  Clear the interrupt context for the current ring.
		pr_debug("Clearing interrupt context for ring %d\n", ring_index);
		rv = xdev->hw.qdma_indirect_intr_ctx_conf(xdev, ring_index,
							  NULL,
							  QDMA_HW_ACCESS_CLEAR);
		if (rv < 0) {
			pr_err("Failed to clear interrupt context for ring %d, error = %d\n", ring_index, rv);
			return xdev->hw.qdma_get_error_code(rv);
		}

		// MD:  Write the interrupt context for the current ring.
		pr_debug("Writing interrupt context for ring %d\n", ring_index);
		rv = xdev->hw.qdma_indirect_intr_ctx_conf(xdev,
				ring_index, &ctxt[i], QDMA_HW_ACCESS_WRITE);
		if (rv < 0) {
			pr_err("Failed to write interrupt context for ring %d, error = %d\n", ring_index, rv);
			return xdev->hw.qdma_get_error_code(rv);
		}
	}

	// MD:  Return success after configuring all interrupt contexts.
	return 0;
}

int qdma_descq_context_clear(struct xlnx_dma_dev *xdev, unsigned int qid_hw,
                             bool st, u8 type, bool clr)
{
    int rv = 0; // MD:  Return value to track success or failure of operations

    if (clr) {
        // MD:  If clearing is requested
        if (type != Q_CMPT) {
            // MD:  Clear software context
            rv = xdev->hw.qdma_sw_ctx_conf(xdev, type, qid_hw, NULL, QDMA_HW_ACCESS_CLEAR);
            if (rv < 0) {
                pr_err("Fail to clear sw context, rv = %d", rv);
                return xdev->hw.qdma_get_error_code(rv);
            }

            // MD:  Clear hardware context
            rv = xdev->hw.qdma_hw_ctx_conf(xdev, type, qid_hw, NULL, QDMA_HW_ACCESS_CLEAR);
            if (rv < 0) {
                pr_err("Fail to clear hw context, rv = %d", rv);
                return xdev->hw.qdma_get_error_code(rv);
            }

            // MD:  Clear credit context
            rv = xdev->hw.qdma_credit_ctx_conf(xdev, type, qid_hw, NULL, QDMA_HW_ACCESS_CLEAR);
            if (rv < 0) {
                pr_err("Fail to clear credit context, rv = %d", rv);
                return xdev->hw.qdma_get_error_code(rv);
            }

            // MD:  Clear prefetch context if the queue is ST C2H
            if (st && (type == Q_C2H)) {
                rv = xdev->hw.qdma_pfetch_ctx_conf(xdev, qid_hw, NULL, QDMA_HW_ACCESS_CLEAR);
                if (rv < 0) {
                    pr_err("Fail to clear pfetch context, rv = %d", rv);
                    return xdev->hw.qdma_get_error_code(rv);
                }
            }
        }

        // MD:  Clear completion context if the queue is ST C2H or MM cmpt
        if ((st && (type == Q_C2H)) || (!st && (type == Q_CMPT))) {
            rv = xdev->hw.qdma_cmpt_ctx_conf(xdev, qid_hw, NULL, QDMA_HW_ACCESS_CLEAR);
            if (rv < 0) {
                pr_err("Fail to clear cmpt context, rv = %d", rv);
                return xdev->hw.qdma_get_error_code(rv);
            }
        }

    } else {
        // MD:  If invalidation is requested
        if (type != Q_CMPT) {
            // MD:  Invalidate software context
            rv = xdev->hw.qdma_sw_ctx_conf(xdev, type, qid_hw, NULL, QDMA_HW_ACCESS_INVALIDATE);
            if (rv < 0) {
                pr_err("Fail to invalidate sw context, rv = %d", rv);
                return xdev->hw.qdma_get_error_code(rv);
            }

            // MD:  Invalidate hardware context
            rv = xdev->hw.qdma_hw_ctx_conf(xdev, type, qid_hw, NULL, QDMA_HW_ACCESS_INVALIDATE);
            if (rv < 0) {
                pr_err("Fail to invalidate hw context, rv = %d", rv);
                return xdev->hw.qdma_get_error_code(rv);
            }

            // MD:  Invalidate credit context
            rv = xdev->hw.qdma_credit_ctx_conf(xdev, type, qid_hw, NULL, QDMA_HW_ACCESS_INVALIDATE);
            if (rv < 0) {
                pr_err("Fail to invalidate credit context, rv = %d", rv);
                return xdev->hw.qdma_get_error_code(rv);
            }

            // MD:  Invalidate prefetch context if the queue is ST C2H
            if (st && (type == Q_C2H)) {
                rv = xdev->hw.qdma_pfetch_ctx_conf(xdev, qid_hw, NULL, QDMA_HW_ACCESS_INVALIDATE);
                if (rv < 0) {
                    pr_err("Fail to invalidate pfetch context, rv = %d", rv);
                    return xdev->hw.qdma_get_error_code(rv);
                }
            }
        }

        // MD:  Invalidate completion context if the queue is ST C2H or MM cmpt
        if ((st && (type == Q_C2H)) || (!st && (type == Q_CMPT))) {
            rv = xdev->hw.qdma_cmpt_ctx_conf(xdev, qid_hw, NULL, QDMA_HW_ACCESS_INVALIDATE);
            if (rv < 0) {
                pr_err("Fail to invalidate cmpt context, rv = %d", rv);
                return xdev->hw.qdma_get_error_code(rv);
            }
        }
    }

    return 0; // MD:  Return success
}

int qdma_descq_context_setup(struct qdma_descq *descq)
{
    struct qdma_descq_context context;

    // MD:  Initialize the context structure to zero
    memset(&context, 0, sizeof(context));

    // MD:  Check if the queue type is not completion (Q_CMPT)
    if (descq->conf.q_type != Q_CMPT) {
        // MD:  Create software context
        make_sw_context(descq, &context.sw_ctxt);
        pr_debug("Software context created for queue index %d", descq->qidx_hw);

        // MD:  Check if QID to vector context is supported and not in poll mode
        if (descq->xdev->dev_cap.qid2vec_ctx) {
            if (descq->xdev->conf.qdma_drv_mode != POLL_MODE) {
                make_qid2vec_context(descq, &context.qid2vec);
                pr_debug("QID to vector context created for queue index %d", descq->qidx_hw);
            }
        }

        // MD:  If the queue is streaming C2H, create prefetch context
        if (descq->conf.st && (descq->conf.q_type == Q_C2H)) {
            make_prefetch_context(descq, &context.pfetch_ctxt);
            pr_debug("Prefetch context created for queue index %d", descq->qidx_hw);
        }
    }

    // MD:  Create completion context if the queue is ST C2H or MM CMPT
    if ((descq->conf.st && (descq->conf.q_type == Q_C2H)) ||
        (!descq->conf.st && (descq->conf.q_type == Q_CMPT))) {
        make_cmpt_context(descq, &context.cmpt_ctxt);
        pr_debug("Completion context created for queue index %d", descq->qidx_hw);
    }

    // MD:  Program the descriptor queue context
    return qdma_descq_context_program(descq->xdev, descq->qidx_hw,
                                      descq->conf.st, descq->conf.q_type, &context);
}

int qdma_descq_context_read(struct xlnx_dma_dev *xdev, unsigned int qid_hw,
                            bool st, u8 type, struct qdma_descq_context *context)
{
    int rv = 0;

    // MD:  Initialize the context structure to zero
    memset(context, 0, sizeof(struct qdma_descq_context));

    // MD:  Check if the queue type is not completion (Q_CMPT)
    if (type != Q_CMPT) {
        // MD:  Read software context
        rv = xdev->hw.qdma_sw_ctx_conf(xdev, type, qid_hw,
                                       &(context->sw_ctxt), QDMA_HW_ACCESS_READ);
        if (rv < 0) {
            pr_err("Failed to read sw context, rv = %d", rv);
            return xdev->hw.qdma_get_error_code(rv);
        }
        pr_debug("Software context read for queue ID %d", qid_hw);

        // MD:  Read hardware context
        rv = xdev->hw.qdma_hw_ctx_conf(xdev, type, qid_hw,
                                       &(context->hw_ctxt), QDMA_HW_ACCESS_READ);
        if (rv < 0) {
            pr_err("Failed to read hw context, rv = %d", rv);
            return xdev->hw.qdma_get_error_code(rv);
        }
        pr_debug("Hardware context read for queue ID %d", qid_hw);

        // MD:  Read credit context
        rv = xdev->hw.qdma_credit_ctx_conf(xdev, type, qid_hw,
                                           &(context->cr_ctxt), QDMA_HW_ACCESS_READ);
        if (rv < 0) {
            pr_err("Failed to read credit context, rv = %d", rv);
            return xdev->hw.qdma_get_error_code(rv);
        }
        pr_debug("Credit context read for queue ID %d", qid_hw);

        // MD:  Read function map context
        rv = xdev->hw.qdma_fmap_conf(xdev, xdev->func_id,
                                     &(context->fmap), QDMA_HW_ACCESS_READ);
        if (rv < 0) {
            pr_err("Failed to read fmap context, rv = %d", rv);
            return xdev->hw.qdma_get_error_code(rv);
        }
        pr_debug("Function map context read for function ID %d", xdev->func_id);

        // MD:  If the queue is streaming and type is valid, read prefetch context
        if (st && type) {
            rv = xdev->hw.qdma_pfetch_ctx_conf(xdev, qid_hw,
                                               &(context->pfetch_ctxt), QDMA_HW_ACCESS_READ);
            if (rv < 0) {
                pr_err("Failed to read pfetch context, rv = %d", rv);
                return xdev->hw.qdma_get_error_code(rv);
            }
            pr_debug("Prefetch context read for queue ID %d", qid_hw);
        }
    }

    // MD:  Read completion context if the queue is ST C2H or MM CMPT
    if ((st && (type == Q_C2H)) || (!st && (type == Q_CMPT))) {
        rv = xdev->hw.qdma_cmpt_ctx_conf(xdev, qid_hw,
                                         &(context->cmpt_ctxt), QDMA_HW_ACCESS_READ);
        if (rv < 0) {
            pr_err("Failed to read cmpt context, rv = %d", rv);
            return xdev->hw.qdma_get_error_code(rv);
        }
        pr_debug("Completion context read for queue ID %d", qid_hw);
    }

    return 0; // MD:  Return success
}

int qdma_intr_context_read(struct xlnx_dma_dev *xdev,
	int ring_index, struct qdma_indirect_intr_ctxt *ctxt)
{
	int rv = 0; // MD:  Initialize return value to track success or failure

	// MD:  Clear the context structure to ensure no residual data
	memset(ctxt, 0, sizeof(struct qdma_indirect_intr_ctxt));

	// MD:  Read the interrupt context configuration for the specified ring index
	rv = xdev->hw.qdma_indirect_intr_ctx_conf(xdev, ring_index, ctxt,
						  QDMA_HW_ACCESS_READ);
	if (rv < 0) {
		// MD:  Log an error message if reading the interrupt context fails
		pr_err("Failed to read intr context, rv = %d", rv);
		return xdev->hw.qdma_get_error_code(rv);
	}

	return 0; // MD:  Return success
}

int qdma_descq_context_program(struct xlnx_dma_dev *xdev, unsigned int qid_hw,
			bool st, u8 type, struct qdma_descq_context *context)
{
	int rv; // MD:  Variable to store return values from function calls

	// MD:  Always clear the context before programming
	rv = qdma_descq_context_clear(xdev, qid_hw, st, type, 1);
	if (rv < 0) {
		// MD:  Log an error message if clearing the context fails
		pr_err("failed to clear the context, rv= %d", rv);
		return xdev->hw.qdma_get_error_code(rv);
	}

	if (type != Q_CMPT) {
		// MD:  Program the software context
		rv = xdev->hw.qdma_sw_ctx_conf(xdev, type, qid_hw,
				&context->sw_ctxt, QDMA_HW_ACCESS_WRITE);
		if (rv < 0) {
			// MD:  Log an error message if programming the software context fails
			pr_err("failed to program sw context, rv= %d", rv);
			return xdev->hw.qdma_get_error_code(rv);
		}

		// MD:  Program the QID to vector context if supported and not in poll mode
		if (xdev->dev_cap.qid2vec_ctx) {
			if (xdev->conf.qdma_drv_mode != POLL_MODE)
				xdev->hw.qdma_qid2vec_conf(xdev, type, qid_hw,
						&context->qid2vec,
						QDMA_HW_ACCESS_WRITE);
		}

		// MD:  Program the prefetch context if the queue is streaming and type is valid
		if (st && type) {
			rv = xdev->hw.qdma_pfetch_ctx_conf(xdev, qid_hw,
							&context->pfetch_ctxt,
							QDMA_HW_ACCESS_WRITE);
			if (rv < 0) {
				// MD:  Log an error message if programming the prefetch context fails
				pr_err("failed to program pfetch context, rv= %d", rv);
				return xdev->hw.qdma_get_error_code(rv);
			}
		}
	}

	// MD:  Program the completion context if the queue is ST C2H or MM CMPT
	if ((st && (type == Q_C2H)) || (!st && (type == Q_CMPT))) {
		rv = xdev->hw.qdma_cmpt_ctx_conf(xdev, qid_hw,
						 &context->cmpt_ctxt,
						 QDMA_HW_ACCESS_WRITE);
		if (rv < 0) {
			// MD:  Log an error message if programming the completion context fails
			pr_err("failed to program cmpt context, rv= %d", rv);
			return xdev->hw.qdma_get_error_code(rv);
		}
	}

	return 0; // MD:  Return success
}

int qdma_descq_context_dump(struct qdma_descq *descq, char *buf, int buflen)
{
	int rv = 0; // MD:  Initialize return value
	int ring_index = -1; // MD:  Variable to store ring index
	int ring_count = 0; // MD:  Counter for rings
	int len = 0; // MD:  Length of data written to buffer
	struct qdma_indirect_intr_ctxt intr_ctxt; // MD:  Structure to hold interrupt context

	// MD:  Read and dump the queue context into the buffer
	rv = descq->xdev->hw.qdma_read_dump_queue_context(descq->xdev,
				descq->xdev->func_id,
				descq->qidx_hw,
				descq->conf.st, descq->conf.q_type,
				buf, buflen);
	if (rv < 0) {
		// MD:  Log an error message if dumping the queue context fails
		pr_err("Failed to dump queue context, rv = %d", rv);
		return descq->xdev->hw.qdma_get_error_code(rv);
	}
	len = rv; // MD:  Update length with the number of bytes written

	// MD:  If interrupt aggregation is enabled, add the interrupt context
	if ((descq->xdev->conf.qdma_drv_mode == INDIRECT_INTR_MODE) ||
			(descq->xdev->conf.qdma_drv_mode == AUTO_MODE)) {
		for (ring_count = 0;
				ring_count < QDMA_NUM_DATA_VEC_FOR_INTR_CXT;
				ring_count++) {
			// MD:  Get the interrupt ring index
			ring_index = get_intr_ring_index(
						descq->xdev,
						(descq->xdev->dvec_start_idx +
								ring_count));

			// MD:  Read the interrupt context for the current ring
			rv = qdma_intr_context_read(descq->xdev,
						ring_index, &intr_ctxt);
			if (rv < 0) {
				// MD:  Log an error message if reading the interrupt context fails
				pr_err("Failed to read intr context for ring %d, rv = %d",
						ring_index, rv);
				return rv;
			}

			// MD:  Dump the interrupt context into the buffer
			rv = descq->xdev->hw.qdma_dump_intr_context(descq->xdev,
						&intr_ctxt, ring_index,
						buf + len, buflen - len);
			if (rv < 0) {
				// MD:  Log an error message if dumping the interrupt context fails
				pr_err("Failed to dump intr context, rv = %d", rv);
				return descq->xdev->hw.qdma_get_error_code(rv);
			}
			len += rv; // MD:  Update length with the number of bytes written
		}
	}

	return len; // MD:  Return the total length of data written to the buffer
}

#endif

