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

#include "thread.h"

#include <linux/kernel.h>

/* MD:
 * Kernel thread function wrappers
 */

// MD: Function to dump the state of a QDMA kernel thread
int qdma_kthread_dump(struct qdma_kthread *thp, char *buf, int buflen, int detail)
{
    int len = 0;

    // MD: Check if buffer is valid
    if (!buf || !buflen)
        return 0;

    // MD: Lock the thread to ensure exclusive access
    lock_thread(thp);

    // MD: Print basic thread information
    len += snprintf(buf, buflen, "%s, cpu %u, work %u.\n", thp->name, thp->cpu, thp->work_cnt);

    // MD: Additional detailed information can be added here if needed
    if (detail)
        ;

    // MD: Unlock the thread after accessing
    unlock_thread(thp);

    // MD: Null-terminate the buffer
    buf[len] = '\0';
    return len;
}

// MD: Function to check if there is pending work for a thread
static inline int xthread_work_pending(struct qdma_kthread *thp)
{
    struct list_head *work_item, *next;

    // MD: Check if there are any work items assigned to this thread
    if (list_empty(&thp->work_list))
        return 0;

    // MD: Check if any work item has pending work to do
    list_for_each_safe(work_item, next, &thp->work_list) {
        if (thp->fpending && thp->fpending(work_item))
            return 1;
    }
    return 0;
}

// MD: Function to reschedule a thread
static inline void xthread_reschedule(struct qdma_kthread *thp)
{
    if (thp->kth_timeout) {
        pr_debug_thread("%s rescheduling for %u seconds", thp->name, thp->kth_timeout);
        qdma_waitq_wait_event_timeout(thp->waitq, thp->schedule, msecs_to_jiffies(thp->kth_timeout));
    } else {
        pr_debug_thread("%s rescheduling", thp->name);
        qdma_waitq_wait_event(thp->waitq, thp->schedule);
    }
}

// MD: Main function for the kernel thread
static int xthread_main(void *data)
{
    struct qdma_kthread *thp = (struct qdma_kthread *)data;

    pr_debug_thread("%s UP.\n", thp->name);

    // MD: Disallow SIGPIPE signal
    disallow_signal(SIGPIPE);

    // MD: Initialize the thread if a function is provided
    if (thp->finit)
        thp->finit(thp);

    // MD: Main loop for the thread
    while (!kthread_should_stop()) {
        struct list_head *work_item, *next;

        pr_debug_thread("%s interruptible\n", thp->name);

        // MD: Check if there is any work to do
        lock_thread(thp);
        if (!xthread_work_pending(thp)) {
            unlock_thread(thp);
            xthread_reschedule(thp);
            lock_thread(thp);
        }
        thp->schedule = 0;

        // MD: Process work items if available
        if (thp->work_cnt) {
            pr_debug_thread("%s processing %u work items\n", thp->name, thp->work_cnt);
            list_for_each_safe(work_item, next, &thp->work_list) {
                thp->fproc(work_item);
            }
        }
        unlock_thread(thp);
        schedule();
    }

    pr_debug_thread("%s, work done.\n", thp->name);

    // MD: Finalize the thread if a function is provided
    if (thp->fdone)
        thp->fdone(thp);

    pr_debug_thread("%s, exit.\n", thp->name);
    return 0;
}

// MD: Function to start a QDMA kernel thread
int qdma_kthread_start(struct qdma_kthread *thp, char *name, int id)
{
    int len;

    // MD: Check if the thread task is already running
    if (thp->task) {
        pr_warn("kthread %s task already running?\n", thp->name);
        return -EINVAL;
    }

#ifdef __QDMA_VF__
    // MD: Format the thread name for a virtual function
    len = snprintf(thp->name, sizeof(thp->name), "%s_vf_%d", name, id);
    if (len < 0)
        return -EINVAL;
#else
    // MD: Format the thread name for a physical function
    len = snprintf(thp->name, sizeof(thp->name), "%s%d", name, id);
    if (len < 0)
        return -EINVAL;
#endif
    thp->id = id;

    // MD: Initialize the thread's lock and work list
    spin_lock_init(&thp->lock);
    INIT_LIST_HEAD(&thp->work_list);
    qdma_waitq_init(&thp->waitq);

    // MD: Create the kernel thread on a specific CPU node
    thp->task = kthread_create_on_node(xthread_main, (void *)thp, cpu_to_node(thp->cpu), "%s", thp->name);
    if (IS_ERR(thp->task)) {
        pr_err("kthread %s, create task failed: 0x%lx\n", thp->name, (unsigned long)IS_ERR(thp->task));
        thp->task = NULL;
        return -EFAULT;
    }

    // MD: Bind the thread to a specific CPU
    kthread_bind(thp->task, thp->cpu);

    pr_debug_thread("kthread 0x%p, %s, cpu %u, task 0x%p.\n", thp, thp->name, thp->cpu, thp->task);

    // MD: Wake up the thread to start execution
    wake_up_process(thp->task);
    return 0;
}

// MD: Function to stop a QDMA kernel thread
int qdma_kthread_stop(struct qdma_kthread *thp)
{
    int rv;

    // MD: Check if the thread task is already stopped
    if (!thp->task) {
        pr_debug_thread("kthread %s, already stopped.\n", thp->name);
        return 0;
    }

    // MD: Signal the thread to stop and wait for it to finish
    thp->schedule = 1;
    rv = kthread_stop(thp->task);
    if (rv < 0) {
        pr_warn("kthread %s, stop err %d.\n", thp->name, rv);
        return rv;
    }

    pr_debug_thread("kthread %s, 0x%p, stopped.\n", thp->name, thp->task);
    thp->task = NULL;

    return 0;
}
