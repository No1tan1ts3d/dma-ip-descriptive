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

#define pr_fmt(fmt) KBUILD_MODNAME ":%s: " fmt, __func__

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/errno.h>

#include "xdev.h"
#include "qdma_mbox.h"
#include <linux/sched.h>

#ifdef __QDMA_VF__

/* MD:
 * Function to take a VF offline by sending a "bye" message.
 * This function allocates a mailbox message, composes the offline message,
 * and sends it with retries.
 */
int xdev_sriov_vf_offline(struct xlnx_dma_dev *xdev, u16 func_id)
{
    int rv;
    struct mbox_msg *m = qdma_mbox_msg_alloc();

    // MD: Check if message allocation failed
    if (!m)
        return -ENOMEM;

    // MD: Compose the VF offline message
    qdma_mbox_compose_vf_offline(xdev->func_id, m->raw);

    // MD: Send the "bye" message with retries
    rv = qdma_mbox_msg_send(xdev, m, 0, QDMA_MBOX_MSG_TIMEOUT_MS);
    if (rv < 0)
        pr_info("%s, send bye failed %d.\n", xdev->conf.name, rv);

    return rv;
}

/* MD:
 * Function to reset a VF offline by sending a "reset bye" message.
 * This function allocates a mailbox message, composes the reset offline message,
 * sends it, and then frees the message.
 */
int xdev_sriov_vf_reset_offline(struct xlnx_dma_dev *xdev)
{
    int rv;
    struct mbox_msg *m = qdma_mbox_msg_alloc();

    // MD: Check if message allocation failed
    if (!m)
        return -ENOMEM;

    // MD: Compose the VF reset offline message
    qdma_mbox_compose_vf_reset_offline(xdev->func_id, m->raw);

    // MD: Send the "reset bye" message
    rv = qdma_mbox_msg_send(xdev, m, 1, QDMA_MBOX_MSG_TIMEOUT_MS);
    if (rv < 0)
        pr_err("%s, send reset bye failed %d.\n", xdev->conf.name, rv);

    // MD: Free the allocated mailbox message
    qdma_mbox_msg_free(m);
    return rv;
}

/* MD:
 * Bring a Virtual Function (VF) online for a given device
 */
int xdev_sriov_vf_online(struct xlnx_dma_dev *xdev, u16 func_id)
{
    int rv;
    int qbase = -1;
    struct mbox_msg *m = qdma_mbox_msg_alloc();

    // MD: Check if mailbox message allocation was successful
    if (!m)
        return -ENOMEM;

    // MD: Compose a message to bring the VF online
    qmda_mbox_compose_vf_online(xdev->func_id, 0, &qbase, m->raw);

    // MD: Send the composed message
    rv = qdma_mbox_msg_send(xdev, m, 1, QDMA_MBOX_MSG_TIMEOUT_MS);
    if (rv < 0) {
        pr_err("%s, send hello failed %d.\n", xdev->conf.name, rv);
        qdma_mbox_msg_free(m);
        return rv;
    }

    // MD: Retrieve device information for the VF
    rv = qdma_mbox_vf_dev_info_get(m->raw, &xdev->dev_cap, &xdev->dma_device_index);
    if (rv < 0) {
        pr_info("%s, failed to get dev info %d.\n", xdev->conf.name, rv);
        rv = -EINVAL;
    } else {
        pr_info("%s: num_pfs:%d, num_qs:%d, flr_present:%d, st_en:%d, mm_en:%d, mm_cmpt_en:%d, mailbox_en:%d, mm_channel_max:%d, qid2vec_ctx:%d, cmpt_ovf_chk_dis:%d, mailbox_intr:%d, sw_desc_64b:%d, cmpt_desc_64b:%d, dynamic_bar:%d, legacy_intr:%d, cmpt_trig_count_timer:%d",
                xdev->conf.name,
                xdev->dev_cap.num_pfs,
                xdev->dev_cap.num_qs,
                xdev->dev_cap.flr_present,
                xdev->dev_cap.st_en,
                xdev->dev_cap.mm_en,
                xdev->dev_cap.mm_cmpt_en,
                xdev->dev_cap.mailbox_en,
                xdev->dev_cap.mm_channel_max,
                xdev->dev_cap.qid2vec_ctx,
                xdev->dev_cap.cmpt_ovf_chk_dis,
                xdev->dev_cap.mailbox_intr,
                xdev->dev_cap.sw_desc_64b,
                xdev->dev_cap.cmpt_desc_64b,
                xdev->dev_cap.dynamic_bar,
                xdev->dev_cap.legacy_intr,
                xdev->dev_cap.cmpt_trig_count_timer);
    }

    // MD: Free the allocated mailbox message
    qdma_mbox_msg_free(m);
    return rv;
}

#elif defined(CONFIG_PCI_IOV)

/* MD:
 * Disable SR-IOV for a given device
 */
void xdev_sriov_disable(struct xlnx_dma_dev *xdev)
{
    struct pci_dev *pdev = xdev->conf.pdev;
    unsigned int sleep_timeout = (50 * xdev->vf_count); /* MD: 50ms per vf */

    // MD: Return if no VFs are currently enabled
    if (!xdev->vf_count)
        return;

    // MD: Disable SR-IOV on the PCI device
    pci_disable_sriov(pdev);

    // MD: Wait for all VFs to be disabled
    qdma_waitq_wait_event_timeout(xdev->wq, (xdev->vf_count == 0), msecs_to_jiffies(sleep_timeout));

    // MD: Stop the mailbox and free VF information
    qdma_mbox_stop(xdev);
    kfree(xdev->vf_info);
    xdev->vf_info = NULL;
    xdev->vf_count = 0;
}

/* MD:
 * Enable SR-IOV for a given device with a specified number of VFs
 */
int xdev_sriov_enable(struct xlnx_dma_dev *xdev, int num_vfs)
{
    struct pci_dev *pdev = xdev->conf.pdev;
    int current_vfs = pci_num_vf(pdev);
    struct qdma_vf_info *vf;
    int i;
    int rv;

    // MD: Check if VFs are already enabled
    if (current_vfs) {
        dev_err(&pdev->dev, "%d VFs already enabled!\n", current_vfs);
        return current_vfs;
    }

    // MD: Allocate memory for VF information
    vf = kmalloc(num_vfs * (sizeof(struct qdma_vf_info)), GFP_KERNEL);
    if (!vf) {
        pr_info("%s failed to allocate memory for VFs, %d * %ld.\n", xdev->conf.name, num_vfs, sizeof(struct qdma_vf_info));
        return -ENOMEM;
    }

    // MD: Initialize VF function IDs
    for (i = 0; i < num_vfs; i++)
        vf[i].func_id = QDMA_FUNC_ID_INVALID;

    // MD: Initialize wait queue and set VF count
    qdma_waitq_init(&xdev->wq);
    xdev->vf_count = num_vfs;
    xdev->vf_info = vf;

    pr_debug("%s: req %d, current %d, assigned %d.\n", xdev->conf.name, num_vfs, current_vfs, pci_vfs_assigned(pdev));

    // MD: Start the mailbox for communication
    qdma_mbox_start(xdev);

    // MD: Enable SR-IOV on the PCI device
    rv = pci_enable_sriov(pdev, num_vfs);
    if (rv) {
        pr_info("%s, enable sriov %d failed %d.\n", xdev->conf.name, num_vfs, rv);
        xdev_sriov_disable(xdev);
        return 0;
    }

    pr_debug("%s: done, req %d, current %d, assigned %d.\n", xdev->conf.name, num_vfs, pci_num_vf(pdev), pci_vfs_assigned(pdev));

    return num_vfs;
}

/* MD:
 * Configure SR-IOV for a device
 */
int qdma_device_sriov_config(struct pci_dev *pdev, unsigned long dev_hndl, int num_vfs)
{
    struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;
    int rv;

    // MD: Check if device handle is valid
    if (!dev_hndl)
        return -EINVAL;

    // MD: Validate the device handle
    rv = xdev_check_hndl(__func__, pdev, dev_hndl);
    if (rv < 0)
        return rv;

    // MD: Disable SR-IOV if num_vfs is zero
    if (!num_vfs) {
        xdev_sriov_disable(xdev);
        return 0;
    }

    // MD: Check if mailbox is enabled
    if (!xdev->dev_cap.mailbox_en) {
        dev_err(&pdev->dev, "Mailbox not enabled in this device");
        return -EPERM;
    }

    // MD: Enable SR-IOV with the specified number of VFs
    rv = xdev_sriov_enable(xdev, num_vfs);
    if (rv < 0)
        return rv;

    return xdev->vf_count;
}

void xdev_sriov_vf_offline(struct xlnx_dma_dev *xdev, u16 func_id)
{
    struct qdma_vf_info *vf = (struct qdma_vf_info *)xdev->vf_info;
    int i;

    // MD: Decrement the count of online virtual functions
    xdev->vf_count_online--;

    // MD: Iterate through the list of virtual functions
    for (i = 0; i < xdev->vf_count; i++, vf++) {
        // MD: Check if the current VF matches the given function ID
        if (vf->func_id == func_id) {
            /* MD:*
             * Do not mark func_id as invalid during PF FLR (Function Level Reset)
             * because after the PF (Physical Function) comes back up, it needs
             * a valid func_id list to send out the RESET_DONE message.
             */
            if (xdev->reset_state == RESET_STATE_IDLE)
                vf->func_id = QDMA_FUNC_ID_INVALID;

            // MD: Reset the queue base and maximum for the VF
            vf->qbase = 0;
            vf->qmax = 0;
        }
    }

    // MD: Wake up any processes waiting on the wait queue
    qdma_waitq_wakeup(&xdev->wq);
}

int xdev_sriov_vf_online(struct xlnx_dma_dev *xdev, u16 func_id)
{
    struct qdma_vf_info *vf = (struct qdma_vf_info *)xdev->vf_info;
    int i;

    // MD: Increment the count of online virtual functions
    xdev->vf_count_online++;

    // MD: Check if the device is in the idle reset state
    if (xdev->reset_state == RESET_STATE_IDLE) {
        // MD: Iterate through the list of virtual functions
        for (i = 0; i < xdev->vf_count; i++, vf++) {
            // MD: Find a VF slot with an invalid function ID
            if (vf->func_id == QDMA_FUNC_ID_INVALID) {
                vf->func_id = func_id; // MD: Assign the new function ID
                return 0; // MD: Success
            }
        }
        // MD: Log an error if no free slot is available
        pr_info("%s, func 0x%x, NO free slot.\n", xdev->conf.name, func_id);
        qdma_waitq_wakeup(&xdev->wq); // MD: Wake up any waiting processes
        return -EINVAL; // MD: Return error
    } else {
        return 0; // MD: If not in idle state, return success
    }
}

int xdev_sriov_vf_fmap(struct xlnx_dma_dev *xdev, u16 func_id,
                       unsigned short qbase, unsigned short qmax)
{
    struct qdma_vf_info *vf = (struct qdma_vf_info *)xdev->vf_info;
    int i;

    // MD: Iterate through the list of virtual functions
    for (i = 0; i < xdev->vf_count; i++, vf++) {
        // MD: Check if the current VF matches the given function ID
        if (vf->func_id == func_id) {
            vf->qbase = qbase; // MD: Set the queue base
            vf->qmax = qmax;   // MD: Set the maximum queue
            return 0; // MD: Success
        }
    }

    // MD: Log an error if no matching function ID is found
    pr_info("%s, func 0x%x, NO match.\n", xdev->conf.name, func_id);
    return -EINVAL; // MD: Return error
}

#endif /* MD: if defined(CONFIG_PCI_IOV) && !defined(__QDMA_VF__) */
