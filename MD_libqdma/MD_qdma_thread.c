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

#include "qdma_thread.h"

#include <linux/kernel.h>

#include "qdma_descq.h"
#include "thread.h"
#include "xdev.h"

/* MD: ********************* global variables *********************************** */

static unsigned int thread_cnt; // MD: Number of completion status threads
static struct qdma_kthread *cs_threads; // MD: Array of completion status threads

static spinlock_t qcnt_lock; // MD: Spinlock for protecting per_cpu_qcnt
static unsigned int cpu_count; // MD: Number of CPUs
static unsigned int *per_cpu_qcnt; // MD: Array to track queue count per CPU

/* MD: ********************* static function declarations *********************** */

// MD: Function to check if there are pending completion statuses
static int qdma_thread_cmpl_status_pend(struct list_head *work_item);

// MD: Function to process completion statuses
static int qdma_thread_cmpl_status_proc(struct list_head *work_item);

/* MD: ********************* static function definitions ************************ */

// MD: Check if there are pending completion statuses for a descriptor queue
static int qdma_thread_cmpl_status_pend(struct list_head *work_item)
{
    struct qdma_descq *descq = list_entry(work_item, struct qdma_descq, cmplthp_list);
    int pend = 0;

    // MD: Lock the descriptor queue to ensure exclusive access
    lock_descq(descq);
    // MD: Check if there are pending or work items in the descriptor queue
    pend = !list_empty(&descq->pend_list) || !list_empty(&descq->work_list);
    // MD: Unlock the descriptor queue after checking
    unlock_descq(descq);

    return pend;
}

// MD: Process completion statuses for a descriptor queue
static int qdma_thread_cmpl_status_proc(struct list_head *work_item)
{
    struct qdma_descq *descq;

    // MD: Retrieve the descriptor queue from the work item
    descq = list_entry(work_item, struct qdma_descq, cmplthp_list);
    // MD: Update the completion status for the descriptor queue
    qdma_descq_service_cmpl_update(descq, 0, 1);
    return 0;
}

/* MD: ********************* public function definitions ************************ */

// MD: Remove a work item from a completion thread
void qdma_thread_remove_work(struct qdma_descq *descq)
{
    struct qdma_kthread *cmpl_thread;
    int cpu_idx = cpu_count;

    // MD: Lock the descriptor queue to ensure exclusive access
    lock_descq(descq);
    // MD: Retrieve the completion thread associated with the descriptor queue
    cmpl_thread = descq->cmplthp;
    // MD: Clear the completion thread pointer in the descriptor queue
    descq->cmplthp = NULL;

    // MD: Check if the descriptor queue was assigned to a CPU
    if (descq->cpu_assigned) {
        descq->cpu_assigned = 0;
        cpu_idx = descq->intr_work_cpu;
    }

    // MD: Debug information about the removal of the work item
    pr_debug("%s removing from thread %s, %u.\n",
             descq->conf.name, cmpl_thread ? cmpl_thread->name : "?",
             cpu_idx);

    // MD: Unlock the descriptor queue after processing
    unlock_descq(descq);

    // MD: Decrement the queue count for the CPU if it was assigned
    if (cpu_idx < cpu_count) {
        spin_lock(&qcnt_lock);
        per_cpu_qcnt[cpu_idx]--;
        spin_unlock(&qcnt_lock);
    }

    // MD: Remove the descriptor queue from the completion thread if it exists
    if (cmpl_thread) {
        lock_thread(cmpl_thread);
        list_del(&descq->cmplthp_list);
        cmpl_thread->work_cnt--;
        unlock_thread(cmpl_thread);
    }
}

/* MD:
 * Add work to a QDMA thread
 */
void qdma_thread_add_work(struct qdma_descq *descq)
{
    struct qdma_kthread *thp = cs_threads;
    unsigned int v = 0;
    int i, idx = thread_cnt;

    // MD: Check if the driver mode is not polling
    if (descq->xdev->conf.qdma_drv_mode != POLL_MODE) {
        // MD: Lock the CPU queue count
        spin_lock(&qcnt_lock);
        idx = cpu_count - 1;
        v = per_cpu_qcnt[idx];

        // MD: Find the CPU with the least queue count
        for (i = idx - 1; i >= 0 && v; i--) {
            if (per_cpu_qcnt[i] < v) {
                idx = i;
                v = per_cpu_qcnt[i];
            }
        }

        // MD: Increment the queue count for the selected CPU
        per_cpu_qcnt[idx]++;
        spin_unlock(&qcnt_lock);

        // MD: Lock the descriptor queue and assign the CPU
        lock_descq(descq);
        descq->cpu_assigned = 1;
        descq->intr_work_cpu = idx;
        unlock_descq(descq);

        // MD: Debug information about CPU assignment
        pr_debug("%s 0x%p assigned to cpu %u.\n", descq->conf.name, descq, idx);

        return;
    }

    // MD: Polled mode: Find the thread with the least work count
    for (i = 0; i < thread_cnt; i++, thp++) {
        lock_thread(thp);
        if (idx == thread_cnt) {
            v = thp->work_cnt;
            idx = i;
        } else if (!thp->work_cnt) {
            idx = i;
            unlock_thread(thp);
            break;
        } else if (thp->work_cnt < v)
            idx = i;
        unlock_thread(thp);
    }

    // MD: Assign the work to the selected thread
    thp = cs_threads + idx;
    lock_thread(thp);
    list_add_tail(&descq->cmplthp_list, &thp->work_list);
    descq->intr_work_cpu = idx;
    thp->work_cnt++;
    unlock_thread(thp);

    // MD: Debug information about thread assignment
    pr_debug("%s 0x%p assigned to cmpl status thread %s,%u.\n",
             descq->conf.name, descq, thp->name, thp->work_cnt);

    // MD: Lock the descriptor queue and assign the completion thread
    lock_descq(descq);
    descq->cmplthp = thp;
    unlock_descq(descq);
}

/* MD:
 * Create QDMA threads
 */
int qdma_threads_create(unsigned int num_threads)
{
    struct qdma_kthread *thp;
    int i;
    int rv;

    // MD: Check if threads are already created
    if (thread_cnt) {
        pr_warn("threads already created!");
        return 0;
    }

    // MD: Initialize the CPU queue count lock
    spin_lock_init(&qcnt_lock);

    // MD: Get the number of online CPUs
    cpu_count = num_online_cpus();
    per_cpu_qcnt = kzalloc(cpu_count * sizeof(unsigned int), GFP_KERNEL);
    if (!per_cpu_qcnt)
        return -ENOMEM;

    // MD: Determine the number of threads to create
    thread_cnt = (num_threads == 0) ? cpu_count : num_threads;

    // MD: Allocate memory for the threads
    cs_threads = kzalloc(thread_cnt * sizeof(struct qdma_kthread), GFP_KERNEL);
    if (!cs_threads)
        return -ENOMEM;

    // MD: Create DMA writeback monitoring threads
    thp = cs_threads;
    for (i = 0; i < thread_cnt; i++, thp++) {
        thp->cpu = i;
        thp->kth_timeout = 0;
        rv = qdma_kthread_start(thp, "qdma_cmpl_status_th", i);
        if (rv < 0)
            goto cleanup_threads;
        thp->fproc = qdma_thread_cmpl_status_proc;
        thp->fpending = qdma_thread_cmpl_status_pend;
    }

    return 0;

cleanup_threads:
    // MD: Cleanup in case of failure
    kfree(cs_threads);
    cs_threads = NULL;
    thread_cnt = 0;

    return rv;
}

/* MD:
 * Destroy QDMA threads
 */
void qdma_threads_destroy(void)
{
    int i;
    struct qdma_kthread *thp;

    // MD: Free the CPU queue count memory
    if (per_cpu_qcnt) {
        spin_lock(&qcnt_lock);
        kfree(per_cpu_qcnt);
        per_cpu_qcnt = NULL;
        spin_unlock(&qcnt_lock);
    }

    // MD: Return if no threads are created
    if (!thread_cnt)
        return;

    // MD: Stop and free DMA writeback monitoring threads
    thp = cs_threads;
    for (i = 0; i < thread_cnt; i++, thp++)
        if (thp->fproc)
            qdma_kthread_stop(thp);

    kfree(cs_threads);
    cs_threads = NULL;
    thread_cnt = 0;
}
