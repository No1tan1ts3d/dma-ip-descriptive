#include "qdma_mbox.h"
#include "qdma_access_common.h"
#include "qdma_resource_mgmt.h"

#ifdef __QDMA_VF__

/*
 * Function to set the queue range for a device
 */
static int device_set_qrange(struct xlnx_dma_dev *xdev)
{
    struct qdma_dev *qdev = xdev_2_qdev(xdev);
    struct mbox_msg *m;
    int rv = 0;

    // Check if the QDMA device structure is valid
    if (!qdev) {
        pr_err("dev %s, qdev null.\n", dev_name(&xdev->conf.pdev->dev));
        return -EINVAL;
    }

    // Allocate a mailbox message
    m = qdma_mbox_msg_alloc();
    if (!m)
        return -ENOMEM;

    // Compose a mailbox message to program the VF function map
    qdma_mbox_compose_vf_fmap_prog(xdev->func_id, qdev->qmax, (int)qdev->qbase, m->raw);

    // Send the mailbox message and check for errors
    rv = qdma_mbox_msg_send(xdev, m, 1, QDMA_MBOX_MSG_TIMEOUT_MS);
    if (rv < 0) {
        if (rv != -ENODEV)
            pr_info("%s set q range (fmap) failed %d.\n", xdev->conf.name, rv);
        goto err_out;
    }

    // Get the response status from the VF
    rv = qdma_mbox_vf_response_status(m->raw);
    if (rv < 0) {
        pr_err("mbox_vf_response_status failed, err = %d", rv);
        rv = -EINVAL;
    }

    // Debug information about the function ID and queue range
    pr_debug("%s, func id %u/%u, Q 0x%x + 0x%x.\n", xdev->conf.name, xdev->func_id, xdev->func_id_parent, qdev->qbase, qdev->qmax);

    // Mark the queue range as initialized if no errors occurred
    if (!rv)
        qdev->init_qrange = 1;

err_out:
    // Free the mailbox message
    qdma_mbox_msg_free(m);
    return rv;
}

/*
 * Function to set the queue configuration for a device
 */
int device_set_qconf(struct xlnx_dma_dev *xdev, int *qmax, int *qbase)
{
    struct mbox_msg *m;
    int rv = 0;

    // Allocate a mailbox message
    m = qdma_mbox_msg_alloc();
    if (!m)
        return -ENOMEM;

    // Compose a mailbox message to request queue configuration
    qdma_mbox_compose_vf_qreq(xdev->func_id, (uint16_t)*qmax & 0xFFFF, *qbase, m->raw);

    // Send the mailbox message and check for errors
    rv = qdma_mbox_msg_send(xdev, m, 1, QDMA_MBOX_MSG_TIMEOUT_MS);
    if (rv < 0) {
        pr_info("%s set q range (qconf) failed %d.\n", xdev->conf.name, rv);
        goto err_out;
    }

    // Get the queue information from the VF
    rv = qdma_mbox_vf_qinfo_get(m->raw, qbase, (uint16_t *)qmax);
    if (rv < 0) {
        pr_err("mbox_vf_qinfo_get failed, err = %d", rv);
        rv = -EINVAL;
    }

err_out:
    // Free the mailbox message
    qdma_mbox_msg_free(m);
    return rv;
}

#else
/*
 * Set the queue range for a QDMA device
 */
static int device_set_qrange(struct xlnx_dma_dev *xdev)
{
    struct qdma_dev *qdev = xdev_2_qdev(xdev);
    struct qdma_fmap_cfg fmap;
    int rv = 0;

    // Initialize the fmap configuration structure to zero
    memset(&fmap, 0, sizeof(struct qdma_fmap_cfg));

    // Check if the QDMA device structure is valid
    if (!qdev) {
        pr_err("dev %s, qdev null.\n", dev_name(&xdev->conf.pdev->dev));
        return -EINVAL;
    }

    // Set the maximum and base queue values in the fmap configuration
    fmap.qmax = qdev->qmax;
    fmap.qbase = qdev->qbase;

    // Configure the hardware with the fmap settings
    rv = xdev->hw.qdma_fmap_conf(xdev, xdev->func_id, &fmap, QDMA_HW_ACCESS_WRITE);
    if (rv < 0) {
        pr_err("FMAP write failed, err %d\n", rv);
        return xdev->hw.qdma_get_error_code(rv);
    }

    // Mark the queue range as initialized
    qdev->init_qrange = 1;

    // Debug information about the function ID and queue range
    pr_debug("%s, func id %u, Q 0x%x + 0x%x.\n", xdev->conf.name, xdev->func_id, qdev->qbase, qdev->qmax);

    return rv;
}
#endif /* ifndef __QDMA_VF__ */

/*
 * Setup interrupts for a QDMA device
 */
int qdma_device_interrupt_setup(struct xlnx_dma_dev *xdev)
{
    int rv = 0;

    // Check if the driver mode is set to indirect or auto mode
    if ((xdev->conf.qdma_drv_mode == INDIRECT_INTR_MODE) || (xdev->conf.qdma_drv_mode == AUTO_MODE)) {
        // Setup the interrupt ring
        rv = intr_ring_setup(xdev);
        if (rv) {
            pr_err("Failed to setup intr ring, err %d", rv);
            return -EINVAL;
        }

        // Check if interrupt coalescing is enabled
        if (xdev->intr_coal_list != NULL) {
            rv = qdma_intr_context_setup(xdev);
            if (rv)
                return rv;
        } else {
            // Log information about the lack of aggregation
            pr_info("dev %s intr vec[%d] >= queues[%d], No aggregation\n",
                    dev_name(&xdev->conf.pdev->dev),
                    (xdev->num_vecs - xdev->dvec_start_idx),
                    xdev->conf.qsets_max);
            pr_warn("Changing the system mode to direct interrupt mode");
            xdev->conf.qdma_drv_mode = DIRECT_INTR_MODE;
        }
    }

#ifndef __QDMA_VF__
    // Setup error interrupts if not in polling or legacy mode and if master PF
    if (((xdev->conf.qdma_drv_mode != POLL_MODE) && (xdev->conf.qdma_drv_mode != LEGACY_INTR_MODE)) && xdev->conf.master_pf) {
        rv = qdma_err_intr_setup(xdev);
        if (rv < 0) {
            pr_err("Failed to setup err intr, err %d", rv);
            return -EINVAL;
        }

        // Enable hardware error interrupts
        rv = xdev->hw.qdma_hw_error_enable(xdev, xdev->hw.qdma_max_errors);
        if (rv < 0) {
            pr_err("Failed to enable error interrupt, err = %d", rv);
            return -EINVAL;
        }

        // Rearm hardware error interrupts
        rv = xdev->hw.qdma_hw_error_intr_rearm(xdev);
        if (rv < 0) {
            pr_err("Failed to rearm error interrupt with error = %d", rv);
            return -EINVAL;
        }
    }
#endif
    return rv;
}

/*
 * Clean up QDMA device interrupts
 */
void qdma_device_interrupt_cleanup(struct xlnx_dma_dev *xdev)
{
    // Check if interrupt coalescing list exists and teardown the interrupt ring
    if (xdev->intr_coal_list)
        intr_ring_teardown(xdev);
}

/*
 * Prepare QDMA device queue resources
 */
int qdma_device_prep_q_resource(struct xlnx_dma_dev *xdev)
{
    struct qdma_dev *qdev = xdev_2_qdev(xdev);
    int rv = 0;

    // Check if queue range is already initialized
    if (qdev->init_qrange)
        goto done;

    // Read CSR (Control and Status Register) information
    rv = qdma_csr_read(xdev, &xdev->csr_info);
    if (rv < 0)
        goto done;

    // Set device queue range
    rv = device_set_qrange(xdev);
    if (rv < 0)
        goto done;

done:
    return rv;
}

/*
 * Initialize QDMA device
 */
int qdma_device_init(struct xlnx_dma_dev *xdev)
{
    int i = 0;
    struct qdma_fmap_cfg fmap;

#ifndef __QDMA_VF__
    int rv = 0;
#endif
    int qmax = xdev->conf.qsets_max;
    struct qdma_descq *descq;
    struct qdma_dev *qdev;

    // Initialize fmap configuration to zero
    memset(&fmap, 0, sizeof(struct qdma_fmap_cfg));

    // Allocate memory for QDMA device structure
    qdev = kzalloc(sizeof(struct qdma_dev), GFP_KERNEL);
    if (!qdev) {
        pr_err("dev %s qmax %d OOM.\n", dev_name(&xdev->conf.pdev->dev), qmax);
        intr_teardown(xdev);
        return -ENOMEM;
    }

    // Initialize spin lock for QDMA device
    spin_lock_init(&qdev->lock);
    xdev->dev_priv = (void *)qdev;

#ifndef __QDMA_VF__
    // Initialize context memory for master physical function
    if (xdev->conf.master_pf) {
        rv = xdev->hw.qdma_init_ctxt_memory(xdev);
        if (rv < 0) {
            pr_err("init ctxt write failed, err %d\n", rv);
            return xdev->hw.qdma_get_error_code(rv);
        }
    }
#endif

    // Allocate memory for descriptor queues
    qdev->h2c_descq = kmalloc(sizeof(struct qdma_descq) * qmax, GFP_KERNEL);
    qdev->c2h_descq = kmalloc(sizeof(struct qdma_descq) * qmax, GFP_KERNEL);
    qdev->cmpt_descq = kmalloc(sizeof(struct qdma_descq) * qmax, GFP_KERNEL);

    qdev->qmax = qmax;
    qdev->init_qrange = 0;
    qdev->qbase = xdev->conf.qsets_base;

    // Initialize descriptor queues
    for (i = 0, descq = qdev->h2c_descq; i < qdev->qmax; i++, descq++)
        qdma_descq_init(descq, xdev, i, i);
    for (i = 0, descq = qdev->c2h_descq; i < qdev->qmax; i++, descq++)
        qdma_descq_init(descq, xdev, i, i);
    for (i = 0, descq = qdev->cmpt_descq; i < qdev->qmax; i++, descq++)
        qdma_descq_init(descq, xdev, i, i);

#ifndef __QDMA_VF__
    // Set default global CSR for QDMA
    xdev->hw.qdma_set_default_global_csr(xdev);
    for (i = 0; i < xdev->dev_cap.mm_channel_max; i++) {
        xdev->hw.qdma_mm_channel_conf(xdev, i, 1, 1);
        xdev->hw.qdma_mm_channel_conf(xdev, i, 0, 1);
    }
#endif
    return 0;
}

#define QDMA_BUF_LEN 256

/*
 * Clean up QDMA device
 */
void qdma_device_cleanup(struct xlnx_dma_dev *xdev)
{
    int i;
    struct qdma_dev *qdev = xdev_2_qdev(xdev);
    struct qdma_descq *descq;
    char buf[QDMA_BUF_LEN];

    // Check if QDMA device structure is valid
    if (!qdev) {
        pr_info("dev %s, qdev null.\n", dev_name(&xdev->conf.pdev->dev));
        return;
    }

    // Stop and remove descriptor queues
    for (i = 0, descq = qdev->h2c_descq; i < qdev->qmax; i++, descq++) {
        if (descq->q_state == Q_STATE_ONLINE)
            qdma_queue_stop((unsigned long int)xdev, i, buf, QDMA_BUF_LEN);
    }

    for (i = 0, descq = qdev->c2h_descq; i < qdev->qmax; i++, descq++) {
        if (descq->q_state == Q_STATE_ONLINE)
            qdma_queue_stop((unsigned long int)xdev, i + qdev->qmax, buf, QDMA_BUF_LEN);
    }

    for (i = 0, descq = qdev->h2c_descq; i < qdev->qmax; i++, descq++) {
        if (descq->q_state == Q_STATE_ENABLED)
            qdma_queue_remove((unsigned long int)xdev, i, buf, QDMA_BUF_LEN);
    }

    for (i = 0, descq = qdev->c2h_descq; i < qdev->qmax; i++, descq++) {
        if (descq->q_state == Q_STATE_ENABLED)
            qdma_queue_remove((unsigned long int)xdev, i + qdev->qmax, buf, QDMA_BUF_LEN);
    }

    // Free allocated memory for descriptor queues and QDMA device
    kfree(qdev->h2c_descq);
    kfree(qdev->c2h_descq);
    kfree(qdev->cmpt_descq);
    xdev->dev_priv = NULL;
    kfree(qdev);
}

/*
 * Get the ID of a QDMA descriptor queue from the descriptor queue pointer
 */
long qdma_device_get_id_from_descq(struct xlnx_dma_dev *xdev,
                                   struct qdma_descq *descq)
{
    struct qdma_dev *qdev;
    unsigned long idx;
    unsigned long idx_max;
    struct qdma_descq *_descq;

    // Check if the xlnx_dma_dev structure is NULL
    if (!xdev) {
        pr_info("xdev NULL.\n");
        return -EINVAL;
    }

    // Convert the external device to a QDMA device structure
    qdev = xdev_2_qdev(xdev);

    // Check if the qdma_dev structure is NULL
    if (!qdev) {
        pr_err("dev %s, qdev null.\n", dev_name(&xdev->conf.pdev->dev));
        return -EINVAL;
    }

    // Determine the descriptor queue type and set the starting index
    if (descq->conf.q_type == Q_H2C) {
        _descq = qdev->h2c_descq;
        idx = 0;
    } else if (descq->conf.q_type == Q_C2H) {
        _descq = qdev->c2h_descq;
        idx = qdev->qmax;
    } else {
        _descq = qdev->cmpt_descq;
        idx = 2 * qdev->qmax;
    }

    // Calculate the maximum index for the descriptor queue
    idx_max = (idx + (2 * qdev->qmax));

    // Iterate through the descriptor queues to find the matching descriptor queue
    for (; idx < idx_max; idx++, _descq++)
        if (_descq == descq)
            return idx;

    // Return error if the descriptor queue is not found
    return -EINVAL;
}

/*
 * Get a QDMA descriptor queue by its ID
 */
struct qdma_descq *qdma_device_get_descq_by_id(struct xlnx_dma_dev *xdev,
                                               unsigned long idx, char *buf, int buflen, int init)
{
    struct qdma_dev *qdev;
    struct qdma_descq *descq;

    // Check if the xlnx_dma_dev structure is NULL
    if (!xdev) {
        pr_info("xdev NULL.\n");
        return NULL;
    }

    // Convert the external device to a QDMA device structure
    qdev = xdev_2_qdev(xdev);

    // Check if the qdma_dev structure is NULL
    if (!qdev) {
        pr_err("dev %s, qdev null.\n", dev_name(&xdev->conf.pdev->dev));
        return NULL;
    }

    // Determine the descriptor queue based on the index
    if (idx >= qdev->qmax) {
        idx -= qdev->qmax;
        if (idx >= qdev->qmax) {
            idx -= qdev->qmax;
            if (idx >= qdev->qmax) {
                pr_info("%s, q idx too big 0x%lx > 0x%x.\n", xdev->conf.name, idx, qdev->qmax);
                if (buf && buflen)
                    snprintf(buf, buflen, "%s, q idx too big 0x%lx > 0x%x.\n", xdev->conf.name, idx, qdev->qmax);
                return NULL;
            }
            descq = qdev->cmpt_descq + idx;
        } else {
            descq = qdev->c2h_descq + idx;
        }
    } else {
        descq = qdev->h2c_descq + idx;
    }

    // Initialize the descriptor queue if requested
    if (init) {
        lock_descq(descq);
        if (descq->q_state == Q_STATE_DISABLED) {
            pr_info("%s, idx 0x%lx, q 0x%p state invalid.\n", xdev->conf.name, idx, descq);
            if (buf && buflen)
                snprintf(buf, buflen, "%s, idx 0x%lx, q 0x%p state invalid.\n", xdev->conf.name, idx, descq);
            unlock_descq(descq);
            return NULL;
        }
        unlock_descq(descq);
    }

    return descq;
}

#ifdef DEBUGFS
/*
 * Get a paired QDMA descriptor queue by its ID
 */
struct qdma_descq *qdma_device_get_pair_descq_by_id(struct xlnx_dma_dev *xdev,
                                                    unsigned long idx, char *buf, int buflen, int init)
{
    struct qdma_dev *qdev;
    struct qdma_descq *pair_descq;

    // Check if the xlnx_dma_dev structure is NULL
    if (!xdev) {
        pr_info("xdev NULL.\n");
        return NULL;
    }

    // Convert the external device to a QDMA device structure
    qdev = xdev_2_qdev(xdev);

    // Check if the qdma_dev structure is NULL
    if (!qdev) {
        pr_err("dev %s, qdev null.\n", dev_name(&xdev->conf.pdev->dev));
        return NULL;
    }

    // Determine the paired descriptor queue based on the index
    if (idx >= qdev->qmax) {
        idx -= qdev->qmax;
        if (idx >= qdev->qmax) {
            pr_debug("%s, q idx too big 0x%lx > 0x%x.\n", xdev->conf.name, idx, qdev->qmax);
            if (buf && buflen)
                snprintf(buf, buflen, "%s, q idx too big 0x%lx > 0x%x.\n", xdev->conf.name, idx, qdev->qmax);
            return NULL;
        }
        pair_descq = qdev->h2c_descq + idx;
    } else {
        pair_descq = qdev->c2h_descq + idx;
    }

    // Initialize the paired descriptor queue if requested
    if (init) {
        lock_descq(pair_descq);
        if (pair_descq->q_state == Q_STATE_DISABLED) {
            pr_debug("%s, idx 0x%lx, q 0x%p state invalid.\n", xdev->conf.name, idx, pair_descq);
            if (buf && buflen)
                snprintf(buf, buflen, "%s, idx 0x%lx, q 0x%p state invalid.\n", xdev->conf.name, idx, pair_descq);
            unlock_descq(pair_descq);
            return NULL;
        }
        unlock_descq(pair_descq);
    }

    return pair_descq;
}
#endif

/*
 * Get the descriptor queue by hardware queue ID
 */
struct qdma_descq *qdma_device_get_descq_by_hw_qid(struct xlnx_dma_dev *xdev,
                                                   unsigned long qidx_hw, u8 c2h)
{
    struct qdma_dev *qdev;
    struct qdma_descq *descq;
    unsigned long qidx_sw = 0;

    // Check if the xdev pointer is NULL
    if (!xdev) {
        pr_info("xdev NULL.\n");
        return NULL;
    }

    // Convert the external device to a QDMA device structure
    qdev = xdev_2_qdev(xdev);

    // Check if the qdev pointer is NULL
    if (!qdev) {
        pr_err("dev %s, qdev null.\n", dev_name(&xdev->conf.pdev->dev));
        return NULL;
    }

    // Calculate the software queue index
    qidx_sw = qidx_hw - qdev->qbase;

    // Determine the descriptor queue based on the direction (C2H or H2C)
    if (c2h)
        descq = &qdev->c2h_descq[qidx_sw];
    else
        descq = &qdev->h2c_descq[qidx_sw];

    return descq;
}

/*
 * Trigger a reset for all Virtual Functions (VFs) from the Physical Function (PF)
 */
void qdma_pf_trigger_vf_reset(unsigned long dev_hndl)
{
    struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;
    unsigned int sleep_timeout = 0;
    struct mbox_msg *m = NULL;
    struct qdma_vf_info *vf = NULL;
    int i = 0, rv = 0;
    u8 vf_count_online = 0, no_bye_count = 0;
    uint32_t active_queue_count = 0;

    // Check if the xdev pointer is NULL
    if (!xdev) {
        pr_err("xdev NULL\n");
        return;
    }

    // Check if there are any VFs
    if (!xdev->vf_count) {
        pr_debug("VF count is zero\n");
        return;
    }

    vf_count_online = xdev->vf_count_online;
    vf = (struct qdma_vf_info *)xdev->vf_info;

    // Set the reset state to wait for BYEs
    xdev->reset_state = RESET_STATE_PF_WAIT_FOR_BYES;

    // Iterate over each online VF
    for (i = 0; i < vf_count_online; i++) {
        // Calculate the active queue count for the VF
        active_queue_count = qdma_get_device_active_queue_count(
            xdev->dma_device_index, vf[i].func_id, QDMA_DEV_Q_TYPE_H2C);
        active_queue_count += qdma_get_device_active_queue_count(
            xdev->dma_device_index, vf[i].func_id, QDMA_DEV_Q_TYPE_C2H);

        // Calculate the sleep timeout based on active queues
        sleep_timeout = QDMA_MBOX_MSG_TIMEOUT_MS + (200 * active_queue_count);

        pr_info("VF reset in progress. Wait for 0x%x ms\n", sleep_timeout);

        // Allocate a mailbox message
        m = qdma_mbox_msg_alloc();
        if (!m) {
            pr_err("Memory allocation for mbox message failed\n");
            return;
        }

        pr_debug("xdev->func_id=%d, vf[i].func_id=%d, i = %d",
                 xdev->func_id, vf[i].func_id, i);

        // Compose and send the VF reset message
        qdma_mbox_compose_vf_reset_message(m->raw, xdev->func_id, vf[i].func_id);
        rv = qdma_mbox_msg_send(xdev, m, 1, QDMA_MBOX_MSG_TIMEOUT_MS);

        // Wait for the VF to acknowledge the reset
        if (rv == 0) {
            qdma_waitq_wait_event_timeout(xdev->wq,
                                          (xdev->vf_count_online == (vf_count_online - i - 1)),
                                          msecs_to_jiffies(sleep_timeout));
        }

        // Check if the VF did not acknowledge the reset
        if (xdev->vf_count_online != (vf_count_online - i - 1)) {
            xdev->vf_count_online--;
            no_bye_count++;
            pr_warn("BYE not recv from func=%d, online_count=%d\n",
                    vf[i].func_id, xdev->vf_count_online);
        }
    }

    // Reset the state to idle after processing all VFs
    xdev->reset_state = RESET_STATE_IDLE;
    pr_debug("no_bye_count=%d\n", no_bye_count);
}

/*
 * Trigger offline state for all Virtual Functions (VFs) from the Physical Function (PF)
 */
void qdma_pf_trigger_vf_offline(unsigned long dev_hndl)
{
    struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;
    struct mbox_msg *m = NULL;
    struct qdma_vf_info *vf = NULL;
    int i = 0, rv = 0;
    unsigned int sleep_timeout = 0;
    u8 vf_count_online = 0;
    uint32_t active_queue_count = 0;

    // Check if the xdev pointer is NULL
    if (!xdev) {
        pr_err("xdev NULL\n");
        return;
    }

    // Check if there are any VFs
    if (!xdev->vf_count) {
        pr_debug("VF count is zero\n");
        return;
    }

    vf_count_online = xdev->vf_count_online;
    vf = (struct qdma_vf_info *)xdev->vf_info;

    // Iterate over each online VF
    for (i = 0; i < vf_count_online; i++) {
        // Calculate the active queue count for the VF
        active_queue_count = qdma_get_device_active_queue_count(
            xdev->dma_device_index, vf[i].func_id, QDMA_DEV_Q_TYPE_H2C);
        active_queue_count += qdma_get_device_active_queue_count(
            xdev->dma_device_index, vf[i].func_id, QDMA_DEV_Q_TYPE_C2H);

        // Calculate the sleep timeout based on active queues
        sleep_timeout = QDMA_MBOX_MSG_TIMEOUT_MS + (200 * active_queue_count);

        pr_info("VF offline in progress. Wait for 0x%x ms\n", sleep_timeout);

        // Allocate a mailbox message
        m = qdma_mbox_msg_alloc();
        if (!m) {
            pr_err("Memory allocation for mbox message failed\n");
            return;
        }

        pr_info("xdev->func_id=%d, vf[i].func_id=%d, i = %d",
                xdev->func_id, vf[i].func_id, i);

        // Compose and send the PF offline message
        qdma_mbox_compose_pf_offline(m->raw, xdev->func_id, vf[i].func_id);
        rv = qdma_mbox_msg_send(xdev, m, 1, QDMA_MBOX_MSG_TIMEOUT_MS);

        // Wait for the VF to acknowledge the offline state
        if (!rv) {
            qdma_waitq_wait_event_timeout(xdev->wq,
                                          vf[i].func_id == QDMA_FUNC_ID_INVALID,
                                          msecs_to_jiffies(sleep_timeout));
        }

        // Check if the VF did not acknowledge the offline state
        if (vf[i].func_id != QDMA_FUNC_ID_INVALID)
            pr_warn("BYE not recv from func=%d, online_count=%d\n",
                    vf[i].func_id, xdev->vf_count_online);
    }

    pr_debug("xdev->vf_count_online=%d\n", xdev->vf_count_online);
}
