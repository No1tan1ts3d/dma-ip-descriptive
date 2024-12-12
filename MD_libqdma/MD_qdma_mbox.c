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
#include <linux/version.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/jiffies.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/sched.h>

#include "qdma_compat.h"
#include "xdev.h"
#include "qdma_device.h"
#include "qdma_regs.h"
#include "qdma_context.h"
#include "qdma_intr.h"
#include "qdma_mbox.h"

#define MBOX_TIMER_INTERVAL	(1)

#ifdef __QDMA_VF__
#define QDMA_DEV QDMA_DEV_VF
#else
#define QDMA_DEV QDMA_DEV_PF
#endif

/* MD:
 * Function to send a message via the hardware mailbox
 */
static int mbox_hw_send(struct qdma_mbox *mbox, struct mbox_msg *m)
{
    struct xlnx_dma_dev *xdev = mbox->xdev;
    int rv;

    // MD: Lock the hardware transmit lock to ensure exclusive access
    spin_lock_bh(&mbox->hw_tx_lock);
    // MD: Send the message using the QDMA mailbox send function
    rv = qdma_mbox_send(xdev, QDMA_DEV, m->raw);
    // MD: Unlock the hardware transmit lock after sending
    spin_unlock_bh(&mbox->hw_tx_lock);

    // MD: Debug information about the message sending status
    pr_debug("Message sent via hardware mailbox, return value: %d\n", rv);

    return rv;
}

/* MD:
 * Function to receive a message via the hardware mailbox
 */
static int mbox_hw_rcv(struct qdma_mbox *mbox, struct mbox_msg *m)
{
    struct xlnx_dma_dev *xdev = mbox->xdev;
    int rv;

    // MD: Lock the hardware receive lock to ensure exclusive access
    spin_lock_bh(&mbox->hw_rx_lock);
    // MD: Clear the message buffer before receiving
    memset(m->raw, 0, MBOX_MSG_REG_MAX * sizeof(uint32_t));
    // MD: Receive the message using the QDMA mailbox receive function
    rv = qdma_mbox_rcv(xdev, QDMA_DEV, m->raw);
    // MD: Unlock the hardware receive lock after receiving
    spin_unlock_bh(&mbox->hw_rx_lock);

    // MD: Debug information about the message receiving status
    pr_debug("Message received via hardware mailbox, return value: %d\n", rv);

    return rv;
}

/* MD:
 * Mailbox message destruction function
 * Frees the memory allocated for a mailbox message.
 */
static void mbox_msg_destroy(struct kref *kref)
{
    // MD: Retrieve the mbox_msg structure from the kref
    struct mbox_msg *m = container_of(kref, struct mbox_msg, refcnt);

#ifdef RHEL_RELEASE_VERSION
#if RHEL_RELEASE_VERSION(8, 1) <= RHEL_RELEASE_CODE
    // MD: Free memory using vfree for RHEL version 8.1 or later
    vfree(m);
#else
    // MD: Free memory using kfree for older RHEL versions
    kfree(m);
#endif
#else
    // MD: Free memory using kfree for non-RHEL systems
    kfree(m);
#endif
}

/* MD:
 * Free a mailbox message
 * Decrements the reference count and destroys the message if it reaches zero.
 */
void __qdma_mbox_msg_free(const char *f, struct mbox_msg *m)
{
    // MD: Decrement the reference count and call mbox_msg_destroy if it reaches zero
    kref_put(&m->refcnt, mbox_msg_destroy);
}

/* MD:
 * Allocate a new mailbox message
 * Initializes the message structure and returns a pointer to it.
 */
struct mbox_msg *qdma_mbox_msg_alloc(void)
{
#ifdef RHEL_RELEASE_VERSION
#if RHEL_RELEASE_VERSION(8, 1) <= RHEL_RELEASE_CODE
    // MD: Allocate memory using vmalloc for RHEL version 8.1 or later
    struct mbox_msg *m = vmalloc(sizeof(struct mbox_msg));
#else
    // MD: Allocate memory using kzalloc for older RHEL versions
    struct mbox_msg *m = kzalloc(sizeof(struct mbox_msg), GFP_KERNEL);
#endif
#else
    // MD: Allocate memory using kzalloc for non-RHEL systems
    struct mbox_msg *m = kzalloc(sizeof(struct mbox_msg), GFP_KERNEL);
#endif
    if (!m)
        return NULL;

    // MD: Initialize the message structure
    memset(m, 0, sizeof(struct mbox_msg));
    kref_init(&m->refcnt);
    qdma_waitq_init(&m->waitq);

    return m;
}

/* MD:
 * Send a mailbox message
 * Queues the message for transmission and optionally waits for a response.
 */
int qdma_mbox_msg_send(struct xlnx_dma_dev *xdev, struct mbox_msg *m,
                       bool wait_resp, unsigned int timeout_ms)
{
    struct qdma_mbox *mbox = &xdev->mbox;

    // MD: Initialize message response and retry parameters
    m->resp_op_matched = 0;
    m->wait_resp = wait_resp ? 1 : 0;
    m->retry_cnt = (timeout_ms / 1000) + 1;

#if defined(__QDMA_VF__)
    // MD: Check if the device is in a valid reset state
    if (xdev->reset_state == RESET_STATE_INVALID)
        return -EINVAL;
#endif

    // MD: Queue the message to ensure order
    spin_lock_bh(&mbox->list_lock);
    list_add_tail(&m->list, &mbox->tx_todo_list);
    spin_unlock_bh(&mbox->list_lock);

    // MD: Start the transmission
    queue_work(mbox->workq, &mbox->tx_work);

    // MD: Return immediately if not waiting for a response
    if (!wait_resp)
        return 0;

    // MD: Wait for a response with a timeout
    qdma_waitq_wait_event_timeout(m->waitq, m->resp_op_matched,
                                  msecs_to_jiffies(timeout_ms));

    // MD: Check if the response was not matched
    if (!m->resp_op_matched) {
        // MD: Remove the message from the mailbox list
        spin_lock_bh(&mbox->list_lock);
        list_del(&m->list);
        spin_unlock_bh(&mbox->list_lock);

        // MD: Log an error message for timeout
        pr_err("%s mbox timed out. timeout %u ms.\n",
               xdev->conf.name, timeout_ms);

        return -EPIPE;
    }

    return 0;
}

/* MD:
 * mbox rx message processing
 * This function processes a single received mailbox message.
 */
#ifdef __QDMA_VF__

static int mbox_rcv_one_msg(struct qdma_mbox *mbox)
{
    struct mbox_msg *m_rcv = &mbox->rx; // MD: Received message
    struct mbox_msg *m_snd = NULL, *tmp1 = NULL, *tmp2 = NULL;
    struct mbox_msg *m_resp = NULL; // MD: Response message
    int rv = 0;

    // MD: Check if the function ID needs to be updated
    if (mbox->xdev->func_id == mbox->xdev->func_id_parent) {
        mbox->xdev->func_id = qdma_mbox_vf_func_id_get(m_rcv->raw, QDMA_DEV);
        mbox->xdev->func_id_parent = qdma_mbox_vf_parent_func_id_get(m_rcv->raw);
    }

    // MD: Lock the list to check for expired requests
    spin_lock_bh(&mbox->list_lock);
    if (!list_empty(&mbox->rx_pend_list)) {
        // MD: Iterate over pending list to find a matching response
        list_for_each_entry_safe(tmp1, tmp2, &mbox->rx_pend_list, list) {
            if (qdma_mbox_is_msg_response(tmp1->raw, m_rcv->raw)) {
                m_snd = tmp1;
                m_snd->resp_op_matched = 1;
                break;
            }
        }
    }
    spin_unlock_bh(&mbox->list_lock);

    // MD: If no pending requests, allocate a new response message
    if (list_empty(&mbox->rx_pend_list)) {
        m_resp = qdma_mbox_msg_alloc();
        if (!m_resp) {
            pr_err("Failed to allocate mbox msg\n");
            return -ENOMEM;
        }

        // MD: Handle the received message
        rv = qdma_mbox_vf_rcv_msg_handler(m_rcv->raw, m_resp->raw);
        if (rv == QDMA_MBOX_VF_RESET) {
            // MD: Handle VF reset request
            qdma_mbox_msg_send(mbox->xdev, m_resp, 0, QDMA_MBOX_MSG_TIMEOUT_MS);
            mbox->xdev->reset_state = RESET_STATE_RECV_PF_RESET_REQ;
            queue_work(mbox->xdev->workq, &(mbox->xdev->reset_work));
        } else if (rv == QDMA_MBOX_PF_RESET_DONE) {
            // MD: Handle PF reset done
            mbox->xdev->reset_state = RESET_STATE_RECV_PF_RESET_DONE;
            qdma_waitq_wakeup(&mbox->xdev->wq);
            qdma_mbox_msg_send(mbox->xdev, m_resp, 0, QDMA_MBOX_MSG_TIMEOUT_MS);
        } else if (rv == QDMA_MBOX_PF_BYE) {
            // MD: Handle PF offline request
            qdma_mbox_msg_send(mbox->xdev, m_resp, 0, QDMA_MBOX_MSG_TIMEOUT_MS);
            mbox->xdev->reset_state = RESET_STATE_RECV_PF_OFFLINE_REQ;
            queue_work(mbox->xdev->workq, &(mbox->xdev->reset_work));
        } else {
            pr_err("func_id=0x%x parent=0x%x", mbox->xdev->func_id, mbox->xdev->func_id_parent);
        }
    }

    // MD: If a matching request is found, process it
    if (m_snd) {
        spin_lock_bh(&mbox->list_lock);
        list_del(&m_snd->list);
        memcpy(m_snd->raw, m_rcv->raw, MBOX_MSG_REG_MAX * sizeof(uint32_t));
        // MD: Wake up any waiters for the response
        qdma_waitq_wakeup(&m_snd->waitq);
        spin_unlock_bh(&mbox->list_lock);
    }

    return 0;
}

#else
/* MD:
 * Function to receive a single mailbox message
 */
static int mbox_rcv_one_msg(struct qdma_mbox *mbox)
{
    struct mbox_msg *m = &mbox->rx;
    struct mbox_msg *m_resp = qdma_mbox_msg_alloc();
    struct mbox_msg *m_snd = NULL, *tmp1 = NULL, *tmp2 = NULL;
    int rv;

    // MD: Check if message allocation failed
    if (!m_resp)
        return -ENOMEM;

    // MD: Handle the received message and prepare a response
    rv = qdma_mbox_pf_rcv_msg_handler(mbox->xdev,
                                      mbox->xdev->dma_device_index,
                                      mbox->xdev->func_id,
                                      m->raw, m_resp->raw);

    // MD: Handle different response types
    if (rv == QDMA_MBOX_VF_OFFLINE || rv == QDMA_MBOX_VF_RESET_BYE) {
#ifdef CONFIG_PCI_IOV
        uint16_t vf_func_id = qdma_mbox_vf_func_id_get(m->raw, QDMA_DEV);

        // MD: Mark the VF as offline
        xdev_sriov_vf_offline(mbox->xdev, vf_func_id);
#endif
        if (rv == QDMA_MBOX_VF_RESET_BYE)
            qdma_mbox_msg_send(mbox->xdev, m_resp, 0, QDMA_MBOX_MSG_TIMEOUT_MS);
        else
            qdma_mbox_msg_free(m_resp);
    } else if (rv == QDMA_MBOX_VF_ONLINE) {
#ifdef CONFIG_PCI_IOV
        uint16_t vf_func_id = qdma_mbox_vf_func_id_get(m->raw, QDMA_DEV);

        // MD: Mark the VF as online
        xdev_sriov_vf_online(mbox->xdev, vf_func_id);
#endif
        qdma_mbox_msg_send(mbox->xdev, m_resp, 0, QDMA_MBOX_MSG_TIMEOUT_MS);
    } else if (rv == QDMA_MBOX_VF_RESET || rv == QDMA_MBOX_PF_RESET_DONE || rv == QDMA_MBOX_PF_BYE) {
        spin_lock_bh(&mbox->list_lock);
        if (!list_empty(&mbox->rx_pend_list)) {
            list_for_each_entry_safe(tmp1, tmp2, &mbox->rx_pend_list, list) {
                if (qdma_mbox_is_msg_response(tmp1->raw, m->raw)) {
                    m_snd = tmp1;
                    m_snd->resp_op_matched = 1;
                    break;
                }
            }
        }
        if (m_snd) {
            list_del(&m_snd->list);
            memcpy(m_snd->raw, m->raw, MBOX_MSG_REG_MAX * sizeof(uint32_t));
            qdma_waitq_wakeup(&m_snd->waitq);
            qdma_mbox_msg_free(m_snd);
        }
        qdma_mbox_msg_free(m_resp);
        spin_unlock_bh(&mbox->list_lock);
    } else {
        qdma_mbox_msg_send(mbox->xdev, m_resp, 0, QDMA_MBOX_MSG_TIMEOUT_MS);
    }

    return 0;
}
#endif

/* MD:
 * Function to stop the mailbox timer
 */
static inline void mbox_timer_stop(struct qdma_mbox *mbox)
{
    // MD: Delete the timer associated with the mailbox
    del_timer(&mbox->timer);
}

/* MD:
 * Function to start the mailbox timer
 */
static inline void mbox_timer_start(struct qdma_mbox *mbox)
{
    struct timer_list *timer = &mbox->timer;

    // MD: Start the timer with a specified interval
    qdma_timer_start(timer, MBOX_TIMER_INTERVAL);
}

/* MD:
 * tx & rx workqueue handler
 * Handles the transmission and reception of mailbox messages.
 */
static void mbox_tx_work(struct work_struct *work)
{
    // MD: Retrieve the qdma_mbox structure from the work structure
    struct qdma_mbox *mbox = container_of(work, struct qdma_mbox, tx_work);

    while (1) {
        struct mbox_msg *m = NULL;

        // MD: Lock the list to safely access the tx_todo_list
        spin_lock_bh(&mbox->list_lock);
        if (list_empty(&mbox->tx_todo_list)) {
            // MD: Unlock and exit if there are no messages to send
            spin_unlock_bh(&mbox->list_lock);
            break;
        }

        // MD: Get the first message from the tx_todo_list
        m = list_first_entry(&mbox->tx_todo_list, struct mbox_msg, list);
        spin_unlock_bh(&mbox->list_lock);

        // MD: Attempt to send the message via hardware
        if (mbox_hw_send(mbox, m) == 0) {
            mbox->send_busy = 0;
            spin_lock_bh(&mbox->list_lock);
            // MD: Message sent successfully, remove it from the list
            list_del(&m->list);
            spin_unlock_bh(&mbox->list_lock);

            // MD: Check if a response is needed
            if (m->wait_resp) {
                spin_lock_bh(&mbox->list_lock);
                // MD: Add the message to the rx_pend_list for awaiting response
                list_add_tail(&m->list, &mbox->rx_pend_list);
                spin_unlock_bh(&mbox->list_lock);
            } else {
                // MD: Free the message if no response is needed
                qdma_mbox_msg_free(m);
            }
        } else {
            // MD: Check if the device is offline
            if (!xlnx_dma_device_flag_check(mbox->xdev, XDEV_FLAG_OFFLINE)) {
                if (!m->wait_resp) {
                    m->retry_cnt--;
                    if (!m->retry_cnt) {
                        spin_lock_bh(&mbox->list_lock);
                        list_del(&m->list);
                        spin_unlock_bh(&mbox->list_lock);
                        qdma_mbox_msg_free(m);
                        break;
                    }
                }
                mbox->send_busy = 1;
                mbox_timer_start(mbox);
            } else {
                qdma_mbox_msg_free(m);
            }
            break;
        }
    }
}

static void mbox_rx_work(struct work_struct *work)
{
    // MD: Retrieve the qdma_mbox structure from the work structure
    struct qdma_mbox *mbox = container_of(work, struct qdma_mbox, rx_work);
    struct xlnx_dma_dev *xdev = mbox->xdev;
    struct mbox_msg *m = &mbox->rx;
    int rv;
    int mbox_stop = 0;

    // MD: Receive a message from the hardware
    rv = mbox_hw_rcv(mbox, m);
    while (rv == 0) {
        // MD: Check if the device is offline
        if (unlikely(xlnx_dma_device_flag_check(xdev, XDEV_FLAG_OFFLINE)))
            break;
        else if (mbox_rcv_one_msg(mbox) == -EINVAL)
            break;
        rv = mbox_hw_rcv(mbox, m);
    }

    // MD: Handle the case where all zero messages are received
    if (rv == -QDMA_ERR_MBOX_ALL_ZERO_MSG) {
#ifdef __QDMA_VF__
        if (xdev->reset_state == RESET_STATE_IDLE) {
            mbox_stop = 1;
            pr_info("func_id=0x%x parent=0x%x %s: rcv'ed all zeros msg, disable mbox processing.\n",
                    xdev->func_id, xdev->func_id_parent, xdev->conf.name);
        }
#else
        mbox_stop = 1;
        pr_info("PF func_id=0x%x %s: rcv'ed all zero mbox msg, disable mbox processing.\n",
                xdev->func_id, xdev->conf.name);
#endif
    } else if (xlnx_dma_device_flag_check(xdev, XDEV_FLAG_OFFLINE)) {
        mbox_stop = 1;
    }

    // MD: Handle polling and interrupt modes
    if (mbox->rx_poll) {
        if (mbox_stop == 1)
            mbox_timer_stop(mbox);
        else
            mbox_timer_start(mbox);
    } else {
        if (mbox_stop != 0)
            qdma_mbox_disable_interrupts(xdev, QDMA_DEV);
    }
}

/* MD:
 * Non-interrupt mode: use timer for periodic checking of new messages
 */
#if KERNEL_VERSION(4, 15, 0) <= LINUX_VERSION_CODE
static void mbox_timer_handler(struct timer_list *t)
#else
static void mbox_timer_handler(unsigned long arg)
#endif
{
#if KERNEL_VERSION(4, 15, 0) <= LINUX_VERSION_CODE
    // MD: Retrieve the qdma_mbox structure from the timer
    struct qdma_mbox *mbox = from_timer(mbox, t, timer);
#else
    // MD: Cast the argument to qdma_mbox structure
    struct qdma_mbox *mbox = (struct qdma_mbox *)arg;
#endif

    // MD: Check if RX polling is enabled and queue the RX work
    if (mbox->rx_poll)
        queue_work(mbox->workq, &mbox->rx_work);

    // MD: Check if TX is busy and queue the TX work
    if (mbox->send_busy)
        queue_work(mbox->workq, &mbox->tx_work);
}

/* MD:
 * Check if mailbox interrupt is available
 */
bool qdma_mbox_is_irq_availabe(struct xlnx_dma_dev *xdev)
{
    // MD: MBOX is available in all QDMA Soft Devices for Vivado release > 2019.1
    if (((xdev->version_info.device_type == QDMA_DEVICE_SOFT) &&
        (xdev->version_info.vivado_release >= QDMA_VIVADO_2019_1)))
        return true;

    // MD: MBOX is available in all Versal Hard IP from CPM5 onwards
    if ((xdev->version_info.ip_type == QDMA_VERSAL_HARD_IP) &&
        (xdev->version_info.device_type == QDMA_DEVICE_VERSAL_CPM5))
        return true;

    return false;
}

/* MD:
 * Mailbox initialization and cleanup
 */
void qdma_mbox_stop(struct xlnx_dma_dev *xdev)
{
    struct qdma_mbox *mbox = &xdev->mbox;
    uint8_t retry_count = 100;
    int rv = 0;

#ifndef __QDMA_VF__
    // MD: Check if mailbox is enabled in device capabilities
    if (!xdev->dev_cap.mailbox_en)
        return;
#endif

    // MD: Wait for TX todo list to be empty
    do {
        spin_lock_bh(&mbox->list_lock);
        if (list_empty(&mbox->tx_todo_list))
            break;
        spin_unlock_bh(&mbox->list_lock);
        mdelay(10); // MD: Sleep 10ms for messages to be sent or freed
    } while (1);
    spin_unlock_bh(&mbox->list_lock);

    // MD: Check mailbox output status with retries
    do {
        rv = qdma_mbox_out_status(xdev, QDMA_DEV);
        if (!rv)
            break;
        retry_count--;
        mdelay(10);
    } while (retry_count != 0);

    // MD: Stop the mailbox timer
    mbox_timer_stop(&xdev->mbox);

    // MD: Debug information about retry count
    pr_debug("func_id=%d retry_count=%d\n", xdev->func_id, retry_count);

    // MD: Disable interrupts if available and not in RX polling mode
    if (qdma_mbox_is_irq_availabe(xdev)) {
        if (!xdev->mbox.rx_poll)
            qdma_mbox_disable_interrupts(xdev, QDMA_DEV);
    }
}

void qdma_mbox_start(struct xlnx_dma_dev *xdev)
{
#ifndef __QDMA_VF__
    // MD: Check if mailbox is enabled in device capabilities
    if (!xdev->dev_cap.mailbox_en)
        return;
#endif

    // MD: Start the mailbox timer or enable interrupts based on RX polling mode
    if (xdev->mbox.rx_poll)
        mbox_timer_start(&xdev->mbox);
    else
        qdma_mbox_enable_interrupts(xdev, QDMA_DEV);
}

void qdma_mbox_poll_start(struct xlnx_dma_dev *xdev)
{
#ifndef __QDMA_VF__
    // MD: Check if mailbox is enabled in device capabilities
    if (!xdev->dev_cap.mailbox_en)
        return;
#endif

    // MD: Enable RX polling and start the mailbox timer
    xdev->mbox.rx_poll = 1;
    qdma_mbox_disable_interrupts(xdev, QDMA_DEV);
    mbox_timer_start(&xdev->mbox);
}

void qdma_mbox_cleanup(struct xlnx_dma_dev *xdev)
{
    struct qdma_mbox *mbox = &xdev->mbox;

    // MD: Check if the workqueue exists
    if (mbox->workq) {
        // MD: Stop the mailbox timer
        mbox_timer_stop(mbox);

        // MD: Disable interrupts if not in RX polling mode
        if (!xdev->mbox.rx_poll)
            qdma_mbox_disable_interrupts(xdev, QDMA_DEV);

        // MD: Flush and destroy the workqueue
        flush_workqueue(mbox->workq);
        destroy_workqueue(mbox->workq);
    }
}

/* MD:
 * Initialize the QDMA mailbox
 */
int qdma_mbox_init(struct xlnx_dma_dev *xdev)
{
    struct qdma_mbox *mbox = &xdev->mbox;
    struct timer_list *timer = &mbox->timer;
#ifndef __QDMA_VF__
    int i;
#endif
    struct mbox_msg m;
    char name[80];
    int rv;

    // MD: Associate the mailbox with the device
    mbox->xdev = xdev;

    // MD: Initialize spin locks for thread safety
    spin_lock_init(&mbox->lock);
    spin_lock_init(&mbox->list_lock);
    spin_lock_init(&mbox->hw_rx_lock);
    spin_lock_init(&mbox->hw_tx_lock);

    // MD: Initialize lists and work structures for message handling
    INIT_LIST_HEAD(&mbox->tx_todo_list);
    INIT_LIST_HEAD(&mbox->rx_pend_list);
    INIT_WORK(&mbox->tx_work, mbox_tx_work);
    INIT_WORK(&mbox->rx_work, mbox_rx_work);

    // MD: Create a single-threaded workqueue for mailbox operations
    snprintf(name, 80, "%s_mbox_wq", xdev->conf.name);
    mbox->workq = create_singlethread_workqueue(name);

    // MD: Check if workqueue creation failed
    if (!mbox->workq) {
        pr_info("%s OOM, mbox workqueue.\n", xdev->conf.name);
        goto cleanup;
    }

    // MD: Read and discard any incoming messages in the buffer
#ifndef __QDMA_VF__
    for (i = 0; i < 256; i++)
        rv = mbox_hw_rcv(mbox, &m);
#else
    rv = mbox_hw_rcv(mbox, &m);
#endif

    // MD: Acknowledge any received messages in the queue
    qdma_mbox_hw_init(xdev, QDMA_DEV);

    // MD: Check if mailbox interrupts are available
    if (qdma_mbox_is_irq_availabe(xdev)) {
        // MD: Enable interrupts if not in polling or legacy interrupt mode
        if ((xdev->conf.qdma_drv_mode != POLL_MODE) &&
            (xdev->conf.qdma_drv_mode != LEGACY_INTR_MODE)) {
            mbox->rx_poll = 0;
            qdma_mbox_enable_interrupts(xdev, QDMA_DEV);
        } else {
            mbox->rx_poll = 1;
        }
    } else {
        mbox->rx_poll = 1;
    }

    // MD: Set up the mailbox timer
    qdma_timer_setup(timer, mbox_timer_handler, mbox);

    return 0;

cleanup:
    // MD: Clean up resources if initialization fails
    qdma_mbox_cleanup(xdev);
    return -ENOMEM;
}
