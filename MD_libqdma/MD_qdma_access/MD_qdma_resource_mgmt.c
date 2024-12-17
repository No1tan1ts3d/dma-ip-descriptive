/* MD:
 * Copyright (c) 2019-2022, Xilinx, Inc. All rights reserved.
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

#include "qdma_resource_mgmt.h"
#include "qdma_platform.h"
#include "qdma_list.h"
#include "qdma_access_errors.h"

#ifdef ENABLE_WPP_TRACING
#include "qdma_resource_mgmt.tmh"
#endif

/* MD:* Structure to represent a resource entry */
struct qdma_resource_entry {
    int qbase; /* MD:*< Queue base */
    uint32_t total_q; /* MD:*< Total number of queues */
    struct qdma_list_head node; /* MD:*< List node for linking entries */
};

/* MD:* Structure to represent a device entry per function */
struct qdma_dev_entry {
    uint16_t func_id; /* MD:*< Function ID */
    uint32_t active_h2c_qcnt; /* MD:*< Active H2C queue count */
    uint32_t active_c2h_qcnt; /* MD:*< Active C2H queue count */
    uint32_t active_cmpt_qcnt; /* MD:*< Active completion queue count */
    struct qdma_resource_entry entry; /* MD:*< Resource entry */
};

/* MD:* Structure to hold the qconf_entry structure */
struct qdma_resource_master {
    uint32_t dma_device_index; /* MD:*< DMA device index */
    uint32_t pci_bus_start; /* MD:*< Starting PCI bus number */
    uint32_t pci_bus_end; /* MD:*< Ending PCI bus number */
    uint32_t total_q; /* MD:*< Total queues managed */
    int qbase; /* MD:*< Queue base */
    struct qdma_list_head node; /* MD:*< Node for master resource list */
    struct qdma_list_head dev_list; /* MD:*< List of device entries */
    struct qdma_list_head free_list; /* MD:*< List of free resources */
    uint32_t active_qcnt; /* MD:*< Active queue count */
};

/* MD:* Head of the master resource list */
static QDMA_LIST_HEAD(master_resource_list);

/* MD:*
 * qdma_find_master_resource_entry() - Find a master resource entry by bus range
 *
 * @bus_start: Starting PCI bus number
 * @bus_end: Ending PCI bus number
 *
 * Return: Pointer to the found qdma_resource_master or NULL if not found
 */
static struct qdma_resource_master *qdma_find_master_resource_entry(
        uint32_t bus_start, uint32_t bus_end)
{
    struct qdma_list_head *entry, *tmp;

    qdma_resource_lock_take(); // MD: Acquire lock for resource list
    qdma_list_for_each_safe(entry, tmp, &master_resource_list) {
        struct qdma_resource_master *q_resource =
            (struct qdma_resource_master *)QDMA_LIST_GET_DATA(entry);

        if (q_resource->pci_bus_start == bus_start &&
            q_resource->pci_bus_end == bus_end) {
            qdma_resource_lock_give(); // MD: Release lock
            return q_resource;
        }
    }
    qdma_resource_lock_give(); // MD: Release lock

    return NULL;
}

/* MD:*
 * qdma_get_master_resource_entry() - Get a master resource entry by DMA index
 *
 * @dma_device_index: DMA device index
 *
 * Return: Pointer to the found qdma_resource_master or NULL if not found
 */
static struct qdma_resource_master *qdma_get_master_resource_entry(
        uint32_t dma_device_index)
{
    struct qdma_list_head *entry, *tmp;

    qdma_resource_lock_take(); // MD: Acquire lock for resource list
    qdma_list_for_each_safe(entry, tmp, &master_resource_list) {
        struct qdma_resource_master *q_resource =
            (struct qdma_resource_master *)QDMA_LIST_GET_DATA(entry);

        if (q_resource->dma_device_index == dma_device_index) {
            qdma_resource_lock_give(); // MD: Release lock
            return q_resource;
        }
    }
    qdma_resource_lock_give(); // MD: Release lock

    return NULL;
}

/* MD:*
 * qdma_get_dev_entry() - Get a device entry by DMA index and function ID
 *
 * @dma_device_index: DMA device index
 * @func_id: Function ID
 *
 * Return: Pointer to the found qdma_dev_entry or NULL if not found
 */
static struct qdma_dev_entry *qdma_get_dev_entry(uint32_t dma_device_index,
                        uint16_t func_id)
{
    struct qdma_list_head *entry, *tmp;
    struct qdma_resource_master *q_resource =
            qdma_get_master_resource_entry(dma_device_index);

    if (!q_resource)
        return NULL;

    qdma_resource_lock_take(); // MD: Acquire lock for device list
    qdma_list_for_each_safe(entry, tmp, &q_resource->dev_list) {
        struct qdma_dev_entry *dev_entry = (struct qdma_dev_entry *)
            QDMA_LIST_GET_DATA(entry);

        if (dev_entry->func_id == func_id) {
            qdma_resource_lock_give(); // MD: Release lock
            return dev_entry;
        }
    }
    qdma_resource_lock_give(); // MD: Release lock

    return NULL;
}

/* MD:*
 * qdma_free_entry_create() - Create a new free entry for resource management
 *
 * @q_base: Queue base
 * @total_q: Total number of queues
 *
 * Return: Pointer to the newly created qdma_resource_entry or NULL on failure
 */
static struct qdma_resource_entry *qdma_free_entry_create(int q_base,
                                                          uint32_t total_q)
{
    // MD: Allocate memory for a new resource entry
    struct qdma_resource_entry *entry = (struct qdma_resource_entry *)
        qdma_calloc(1, sizeof(struct qdma_resource_master));
    if (entry == NULL) {
        printk(KERN_ERR "Failed to allocate memory for qdma_resource_entry\n");
        return NULL;
    }

    // MD: Initialize the entry with provided queue base and total queues
    entry->total_q = total_q;
    entry->qbase = q_base;

    return entry;
}

/* MD:*
 * qdma_submit_to_free_list() - Submit a device entry to the free list
 *
 * @dev_entry: Device entry containing resource information
 * @head: Head of the free list
 */
static void qdma_submit_to_free_list(struct qdma_dev_entry *dev_entry,
                                     struct qdma_list_head *head)
{
    struct qdma_resource_entry *streach_node = NULL;
    struct qdma_list_head *entry, *tmp;
    struct qdma_resource_entry *new_node = NULL; // MD: New node for free list

    // MD: Check if there are any resources to free
    if (!dev_entry->entry.total_q)
        return;

    // MD: If the free list is empty, add the new node directly
    if (qdma_list_is_empty(head)) {
        new_node = qdma_free_entry_create(dev_entry->entry.qbase,
                                          dev_entry->entry.total_q);
        if (new_node == NULL)
            return;
        QDMA_LIST_SET_DATA(&new_node->node, new_node);
        qdma_list_add_tail(&new_node->node, head);
        // MD: Reset device entry resource parameters
        dev_entry->entry.qbase = -1;
        dev_entry->entry.total_q = 0;
    } else {
        // MD: Traverse the free list to find the appropriate position for the new node
        qdma_list_for_each_safe(entry, tmp, head) {
            struct qdma_resource_entry *node =
                (struct qdma_resource_entry *)QDMA_LIST_GET_DATA(entry);

            // MD: Insert the free slot at the appropriate place
            if ((node->qbase > dev_entry->entry.qbase) ||
                qdma_list_is_last_entry(entry, head)) {
                new_node = qdma_free_entry_create(dev_entry->entry.qbase,
                                                  dev_entry->entry.total_q);
                if (new_node == NULL)
                    return;
                QDMA_LIST_SET_DATA(&new_node->node, new_node);
                if (node->qbase > dev_entry->entry.qbase)
                    qdma_list_insert_before(&new_node->node, &node->node);
                else
                    qdma_list_add_tail(&new_node->node, head);
                // MD: Reset device entry resource parameters
                dev_entry->entry.qbase = -1;
                dev_entry->entry.total_q = 0;
                break;
            }
        }
    }

    // MD: De-fragment the free list by merging contiguous resource chunks
    qdma_list_for_each_safe(entry, tmp, head) {
        struct qdma_resource_entry *node =
            (struct qdma_resource_entry *)QDMA_LIST_GET_DATA(entry);

        if (!streach_node)
            streach_node = node;
        else {
            if ((streach_node->qbase + streach_node->total_q) == (uint32_t)node->qbase) {
                streach_node->total_q += node->total_q;
                qdma_list_del(&node->node);
                qdma_memfree(node);
            } else
                streach_node = node;
        }
    }
}

/* MD:*
 * qdma_get_resource_node() - Return the best free list entry node that can
 *                            accommodate the new request
 *
 * @qmax: Maximum number of queues requested
 * @qbase: Preferred queue base
 * @free_list_head: Head of the free list
 *
 * Return: Pointer to the best fit qdma_resource_entry or NULL if not found
 */
static struct qdma_resource_entry *qdma_get_resource_node(uint32_t qmax,
                                                          int qbase,
                                                          struct qdma_list_head *free_list_head)
{
    struct qdma_list_head *entry, *tmp;
    struct qdma_resource_entry *best_fit_node = NULL;

    // MD: Try to honor the requested queue base
    if (qbase >= 0) {
        qdma_list_for_each_safe(entry, tmp, free_list_head) {
            struct qdma_resource_entry *node =
                (struct qdma_resource_entry *)QDMA_LIST_GET_DATA(entry);

            if ((qbase >= node->qbase) &&
                (node->qbase + node->total_q) >= (qbase + qmax)) {
                best_fit_node = node;
                goto fragment_free_list;
            }
        }
    }
    best_fit_node = NULL;

    // MD: Find the best node to accommodate the queue resource request
    qdma_list_for_each_safe(entry, tmp, free_list_head) {
        struct qdma_resource_entry *node =
            (struct qdma_resource_entry *)QDMA_LIST_GET_DATA(entry);

        if (node->total_q >= qmax) {
            if (!best_fit_node || (best_fit_node->total_q >= node->total_q)) {
                best_fit_node = node;
                qbase = best_fit_node->qbase;
            }
        }
    }

fragment_free_list:
    if (!best_fit_node)
        return NULL;

    if ((qbase == best_fit_node->qbase) && (qmax == best_fit_node->total_q))
        return best_fit_node;

    // MD: Split the free resource node accordingly
    if ((qbase == best_fit_node->qbase) && (qmax != best_fit_node->total_q)) {
        // MD: Create an extra node to hold the extra queues from this node
        struct qdma_resource_entry *new_entry = NULL;
        int lqbase = best_fit_node->qbase + qmax;
        uint32_t lqmax = best_fit_node->total_q - qmax;

        new_entry = qdma_free_entry_create(lqbase, lqmax);
        if (new_entry == NULL)
            return NULL;
        QDMA_LIST_SET_DATA(&new_entry->node, new_entry);
        qdma_list_insert_after(&new_entry->node, &best_fit_node->node);
        best_fit_node->total_q -= lqmax;
    } else if ((qbase > best_fit_node->qbase) &&
               ((qbase + qmax) == (best_fit_node->qbase + best_fit_node->total_q))) {
        // MD: Create an extra node to hold the extra queues from this node
        struct qdma_resource_entry *new_entry = NULL;
        int lqbase = best_fit_node->qbase;
        uint32_t lqmax = qbase - best_fit_node->qbase;

        new_entry = qdma_free_entry_create(lqbase, lqmax);
        if (new_entry == NULL)
            return NULL;
        QDMA_LIST_SET_DATA(&new_entry->node, new_entry);
        qdma_list_insert_before(&new_entry->node, &best_fit_node->node);
        best_fit_node->total_q = qmax;
        best_fit_node->qbase = qbase;
    } else {
        // MD: Create two extra nodes to hold the extra queues from this node
        struct qdma_resource_entry *new_entry = NULL;
        int lqbase = best_fit_node->qbase;
        uint32_t lqmax = qbase - best_fit_node->qbase;

        new_entry = qdma_free_entry_create(lqbase, lqmax);
        if (new_entry == NULL)
            return NULL;
        QDMA_LIST_SET_DATA(&new_entry->node, new_entry);
        qdma_list_insert_before(&new_entry->node, &best_fit_node->node);

        best_fit_node->qbase = qbase;
        best_fit_node->total_q -= lqmax;

        lqbase = best_fit_node->qbase + qmax;
        lqmax = best_fit_node->total_q - qmax;

        new_entry = qdma_free_entry_create(lqbase, lqmax);
        if (new_entry == NULL)
            return NULL;
        QDMA_LIST_SET_DATA(&new_entry->node, new_entry);
        qdma_list_insert_after(&new_entry->node, &best_fit_node->node);
        best_fit_node->total_q = qmax;
    }

    return best_fit_node;
}

/* MD:*
 * qdma_request_q_resource() - Request queue resources for a device entry
 *
 * @dev_entry: Device entry containing resource information
 * @new_qmax: Maximum number of new queues requested
 * @new_qbase: Preferred base for the new queues
 * @free_list_head: Head of the free list
 *
 * Return: 0 on success, negative error code on failure
 */
static int qdma_request_q_resource(struct qdma_dev_entry *dev_entry,
                                    uint32_t new_qmax, int new_qbase,
                                    struct qdma_list_head *free_list_head)
{
    uint32_t qmax = dev_entry->entry.total_q;
    int qbase = dev_entry->entry.qbase;
    struct qdma_resource_entry *free_entry_node = NULL;
    int rv = QDMA_SUCCESS;

    // MD: Submit already allocated queues back to the free list before requesting new resources
    qdma_submit_to_free_list(dev_entry, free_list_head);

    // MD: If no new queues are requested, return success
    if (!new_qmax)
        return 0;

    // MD: Check if the request can be accommodated
    free_entry_node = qdma_get_resource_node(new_qmax, new_qbase, free_list_head);
    if (free_entry_node == NULL) {
        // MD: Request cannot be accommodated. Restore the dev_entry
        free_entry_node = qdma_get_resource_node(qmax, qbase, free_list_head);
        rv = -QDMA_ERR_RM_NO_QUEUES_LEFT;
        qdma_log_error("%s: Not enough queues, err:%d\n", __func__, -QDMA_ERR_RM_NO_QUEUES_LEFT);
        if (free_entry_node == NULL) {
            dev_entry->entry.qbase = -1;
            dev_entry->entry.total_q = 0;
            return rv;
        }
    }

    // MD: Update the device entry with the allocated queue base and total queues
    dev_entry->entry.qbase = free_entry_node->qbase;
    dev_entry->entry.total_q = free_entry_node->total_q;

    // MD: Remove the allocated node from the free list and free its memory
    qdma_list_del(&free_entry_node->node);
    qdma_memfree(free_entry_node);

    return rv;
}

/* MD:*
 * qdma_master_resource_create() - Create a master resource for QDMA
 *
 * @bus_start: Starting PCI bus number
 * @bus_end: Ending PCI bus number
 * @q_base: Queue base
 * @total_q: Total number of queues
 * @dma_device_index: Pointer to store the DMA device index
 *
 * Return: QDMA_SUCCESS on success, negative error code on failure
 */
int qdma_master_resource_create(uint32_t bus_start, uint32_t bus_end,
        int q_base, uint32_t total_q, uint32_t *dma_device_index)
{
    struct qdma_resource_master *q_resource;
    struct qdma_resource_entry *free_entry;
    static int index;

    // MD: Check if a master resource already exists for the given bus range
    q_resource = qdma_find_master_resource_entry(bus_start, bus_end);
    if (q_resource) {
        *dma_device_index = q_resource->dma_device_index;
        qdma_log_debug("%s: Resource already created", __func__);
        qdma_log_debug("for this device(%d)\n", q_resource->dma_device_index);
        return -QDMA_ERR_RM_RES_EXISTS;
    }

    // MD: Assign a new DMA device index
    *dma_device_index = index;

    // MD: Allocate memory for a new master resource
    q_resource = (struct qdma_resource_master *)qdma_calloc(1, sizeof(struct qdma_resource_master));
    if (!q_resource) {
        qdma_log_error("%s: no memory for q_resource, err:%d\n", __func__, -QDMA_ERR_NO_MEM);
        return -QDMA_ERR_NO_MEM;
    }

    // MD: Allocate memory for a new free entry
    free_entry = (struct qdma_resource_entry *)qdma_calloc(1, sizeof(struct qdma_resource_entry));
    if (!free_entry) {
        qdma_memfree(q_resource);
        qdma_log_error("%s: no memory for free_entry, err:%d\n", __func__, -QDMA_ERR_NO_MEM);
        return -QDMA_ERR_NO_MEM;
    }

    // MD: Initialize the master resource and add it to the master resource list
    qdma_resource_lock_take();
    q_resource->dma_device_index = index;
    q_resource->pci_bus_start = bus_start;
    q_resource->pci_bus_end = bus_end;
    q_resource->total_q = total_q;
    q_resource->qbase = q_base;
    qdma_list_init_head(&q_resource->dev_list);
    qdma_list_init_head(&q_resource->free_list);
    QDMA_LIST_SET_DATA(&q_resource->node, q_resource);
    QDMA_LIST_SET_DATA(&q_resource->free_list, q_resource);
    qdma_list_add_tail(&q_resource->node, &master_resource_list);

    // MD: Initialize the free entry and add it to the free list
    free_entry->total_q = total_q;
    free_entry->qbase = q_base;
    QDMA_LIST_SET_DATA(&free_entry->node, free_entry);
    qdma_list_add_tail(&free_entry->node, &q_resource->free_list);
    qdma_resource_lock_give();

    qdma_log_debug("%s: New master resource created at %d", __func__, index);
    ++index;

    return QDMA_SUCCESS;
}

/* MD:*
 * qdma_master_resource_destroy() - Destroy the master resource entry
 *
 * @dma_device_index: DMA device index
 *
 * This function destroys the master resource entry associated with the
 * specified DMA device index. It first checks if the resource exists and
 * if the device list is empty before proceeding to free the resources.
 */
void qdma_master_resource_destroy(uint32_t dma_device_index)
{
    struct qdma_resource_master *q_resource =
            qdma_get_master_resource_entry(dma_device_index);
    struct qdma_list_head *entry, *tmp;

    // MD: Check if the master resource exists
    if (!q_resource)
        return;

    // MD: Acquire resource lock
    qdma_resource_lock_take();

    // MD: Check if the device list is empty
    if (!qdma_list_is_empty(&q_resource->dev_list)) {
        qdma_resource_lock_give();
        return;
    }

    // MD: Iterate over the free list and free each entry
    qdma_list_for_each_safe(entry, tmp, &q_resource->free_list) {
        struct qdma_resource_entry *free_entry =
            (struct qdma_resource_entry *)QDMA_LIST_GET_DATA(entry);

        qdma_list_del(&free_entry->node);
        qdma_memfree(free_entry);
    }

    // MD: Remove and free the master resource
    qdma_list_del(&q_resource->node);
    qdma_memfree(q_resource);

    // MD: Release resource lock
    qdma_resource_lock_give();
}

/* MD:*
 * qdma_dev_entry_create() - Create a device entry
 *
 * @dma_device_index: DMA device index
 * @func_id: Function ID
 *
 * Return: 0 on success, negative error code on failure
 *
 * This function creates a device entry for the specified DMA device index
 * and function ID. It checks if the resource exists and if the device entry
 * already exists before creating a new entry.
 */
int qdma_dev_entry_create(uint32_t dma_device_index, uint16_t func_id)
{
    struct qdma_resource_master *q_resource =
            qdma_get_master_resource_entry(dma_device_index);
    struct qdma_dev_entry *dev_entry;

    // MD: Check if the master resource exists
    if (!q_resource) {
        qdma_log_error("%s: Queue resource not found, err: %d\n",
                    __func__, -QDMA_ERR_RM_RES_NOT_EXISTS);
        return -QDMA_ERR_RM_RES_NOT_EXISTS;
    }

    // MD: Check if the device entry already exists
    dev_entry = qdma_get_dev_entry(dma_device_index, func_id);
    if (!dev_entry) {
        // MD: Acquire resource lock
        qdma_resource_lock_take();

        // MD: Allocate memory for the new device entry
        dev_entry = (struct qdma_dev_entry *)
            qdma_calloc(1, sizeof(struct qdma_dev_entry));
        if (dev_entry == NULL) {
            qdma_resource_lock_give();
            qdma_log_error("%s: Insufficient memory, err:%d\n",
                        __func__, -QDMA_ERR_NO_MEM);
            return -QDMA_ERR_NO_MEM;
        }

        // MD: Initialize the device entry
        dev_entry->func_id = func_id;
        dev_entry->entry.qbase = -1;
        dev_entry->entry.total_q = 0;
        QDMA_LIST_SET_DATA(&dev_entry->entry.node, dev_entry);

        // MD: Add the device entry to the device list
        qdma_list_add_tail(&dev_entry->entry.node, &q_resource->dev_list);

        // MD: Release resource lock
        qdma_resource_lock_give();

        qdma_log_info("%s: Created the dev entry successfully\n", __func__);
    } else {
        qdma_log_error("%s: Dev entry already created, err = %d\n",
                        __func__, -QDMA_ERR_RM_DEV_EXISTS);
        return -QDMA_ERR_RM_DEV_EXISTS;
    }

    return QDMA_SUCCESS;
}

/* MD:*
 * qdma_dev_entry_destroy() - Destroy a device entry
 *
 * @dma_device_index: DMA device index
 * @func_id: Function ID
 *
 * This function destroys the device entry associated with the specified
 * DMA device index and function ID. It submits the entry to the free list
 * before freeing the memory.
 */
void qdma_dev_entry_destroy(uint32_t dma_device_index, uint16_t func_id)
{
    struct qdma_resource_master *q_resource =
            qdma_get_master_resource_entry(dma_device_index);
    struct qdma_dev_entry *dev_entry;

    // MD: Check if the master resource exists
    if (!q_resource) {
        qdma_log_error("%s: Queue resource not found.\n", __func__);
        return;
    }

    // MD: Check if the device entry exists
    dev_entry = qdma_get_dev_entry(dma_device_index, func_id);
    if (!dev_entry) {
        qdma_log_error("%s: Dev entry not found\n", __func__);
        return;
    }

    // MD: Acquire resource lock
    qdma_resource_lock_take();

    // MD: Submit the entry to the free list
    qdma_submit_to_free_list(dev_entry, &q_resource->free_list);

    // MD: Remove and free the device entry
    qdma_list_del(&dev_entry->entry.node);
    qdma_memfree(dev_entry);

    // MD: Release resource lock
    qdma_resource_lock_give();
}

/* MD:*
 * qdma_dev_update() - Update device entry with new queue configuration
 *
 * @dma_device_index: DMA device index
 * @func_id: Function ID
 * @qmax: Maximum number of queues
 * @qbase: Pointer to store the base queue index
 *
 * Return: 0 on success, negative error code on failure
 *
 * This function updates the device entry with a new queue configuration.
 * It checks if there are any active queues before requesting new resources.
 */
int qdma_dev_update(uint32_t dma_device_index, uint16_t func_id,
            uint32_t qmax, int *qbase)
{
    struct qdma_resource_master *q_resource =
            qdma_get_master_resource_entry(dma_device_index);
    struct qdma_dev_entry *dev_entry;
    int rv;

    // MD: Check if the master resource exists
    if (!q_resource) {
        qdma_log_error("%s: Queue resource not found, err: %d\n",
                __func__, -QDMA_ERR_RM_RES_NOT_EXISTS);
        return -QDMA_ERR_RM_RES_NOT_EXISTS;
    }

    // MD: Check if the device entry exists
    dev_entry = qdma_get_dev_entry(dma_device_index, func_id);
    if (!dev_entry) {
        qdma_log_error("%s: Dev Entry not found, err: %d\n",
                    __func__, -QDMA_ERR_RM_DEV_NOT_EXISTS);
        return -QDMA_ERR_RM_DEV_NOT_EXISTS;
    }

    // MD: Acquire resource lock
    qdma_resource_lock_take();

    // MD: Check if there are any active queues
    if (dev_entry->active_h2c_qcnt ||
            dev_entry->active_c2h_qcnt ||
            dev_entry->active_cmpt_qcnt) {
        qdma_resource_lock_give();
        qdma_log_error("%s: Qs active. Config blocked, err: %d\n",
                __func__, -QDMA_ERR_RM_QMAX_CONF_REJECTED);
        return -QDMA_ERR_RM_QMAX_CONF_REJECTED;
    }

    // MD: Request new queue resources
    rv = qdma_request_q_resource(dev_entry, qmax, *qbase,
                &q_resource->free_list);

    // MD: Update the base queue index
    *qbase = dev_entry->entry.qbase;

    // MD: Release resource lock
    qdma_resource_lock_give();

    return rv;
}

/* MD:*
 * qdma_dev_qinfo_get() - Retrieve queue information for a specific device
 *
 * @dma_device_index: DMA device index
 * @func_id: Function ID
 * @qbase: Pointer to store the base queue index
 * @qmax: Pointer to store the maximum number of queues
 *
 * Return: QDMA_SUCCESS on success, negative error code on failure
 */
int qdma_dev_qinfo_get(uint32_t dma_device_index, uint16_t func_id,
                       int *qbase, uint32_t *qmax)
{
    struct qdma_resource_master *q_resource =
            qdma_get_master_resource_entry(dma_device_index);
    struct qdma_dev_entry *dev_entry;

    // MD: Check if the queue resource exists
    if (!q_resource) {
        qdma_log_error("%s: Queue resource not found, err: %d\n",
                __func__, -QDMA_ERR_RM_RES_NOT_EXISTS);
        return -QDMA_ERR_RM_RES_NOT_EXISTS;
    }

    // MD: Retrieve the device entry for the given function ID
    dev_entry = qdma_get_dev_entry(dma_device_index, func_id);

    // MD: Check if the device entry exists
    if (!dev_entry) {
        qdma_log_debug("%s: Dev Entry not created yet\n", __func__);
        return -QDMA_ERR_RM_DEV_NOT_EXISTS;
    }

    // MD: Lock the resource to safely access queue information
    qdma_resource_lock_take();
    *qbase = dev_entry->entry.qbase;
    *qmax = dev_entry->entry.total_q;
    qdma_resource_lock_give();

    return QDMA_SUCCESS;
}

/* MD:*
 * qdma_dev_is_queue_in_range() - Check if a queue ID is within the valid range
 *
 * @dma_device_index: DMA device index
 * @func_id: Function ID
 * @qid_hw: Queue ID to check
 *
 * Return: QDMA_DEV_Q_IN_RANGE if in range, QDMA_DEV_Q_OUT_OF_RANGE otherwise
 */
enum qdma_dev_q_range qdma_dev_is_queue_in_range(uint32_t dma_device_index,
                                                 uint16_t func_id,
                                                 uint32_t qid_hw)
{
    struct qdma_resource_master *q_resource =
            qdma_get_master_resource_entry(dma_device_index);
    struct qdma_dev_entry *dev_entry;
    uint32_t qmax;

    // MD: Check if the queue resource exists
    if (!q_resource) {
        qdma_log_error("%s: Queue resource not found, err: %d\n",
                __func__, -QDMA_ERR_RM_RES_NOT_EXISTS);
        return QDMA_DEV_Q_OUT_OF_RANGE;
    }

    // MD: Retrieve the device entry for the given function ID
    dev_entry = qdma_get_dev_entry(dma_device_index, func_id);

    // MD: Check if the device entry exists
    if (!dev_entry) {
        qdma_log_error("%s: Dev entry not found, err: %d\n",
                __func__, -QDMA_ERR_RM_DEV_NOT_EXISTS);
        return QDMA_DEV_Q_OUT_OF_RANGE;
    }

    // MD: Lock the resource to safely check queue range
    qdma_resource_lock_take();
    qmax = dev_entry->entry.qbase + dev_entry->entry.total_q;
    if (dev_entry->entry.total_q && (qid_hw < qmax) &&
            ((int)qid_hw >= dev_entry->entry.qbase)) {
        qdma_resource_lock_give();
        return QDMA_DEV_Q_IN_RANGE;
    }
    qdma_resource_lock_give();

    return QDMA_DEV_Q_OUT_OF_RANGE;
}

/* MD:*
 * qdma_dev_increment_active_queue() - Increment the active queue count
 *
 * @dma_device_index: DMA device index
 * @func_id: Function ID
 * @q_type: Queue type (H2C, C2H, CMPT)
 *
 * Return: QDMA_SUCCESS on success, negative error code on failure
 */
int qdma_dev_increment_active_queue(uint32_t dma_device_index, uint16_t func_id,
                                    enum qdma_dev_q_type q_type)
{
    struct qdma_resource_master *q_resource =
            qdma_get_master_resource_entry(dma_device_index);
    struct qdma_dev_entry *dev_entry;
    int rv = QDMA_SUCCESS;
    uint32_t *active_qcnt = NULL;

    // MD: Check if the queue resource exists
    if (!q_resource) {
        qdma_log_error("%s: Queue resource not found, err: %d\n",
                __func__, -QDMA_ERR_RM_RES_NOT_EXISTS);
        return -QDMA_ERR_RM_RES_NOT_EXISTS;
    }

    // MD: Retrieve the device entry for the given function ID
    dev_entry = qdma_get_dev_entry(dma_device_index, func_id);

    // MD: Check if the device entry exists
    if (!dev_entry) {
        qdma_log_error("%s: Dev Entry not found, err: %d\n",
                    __func__, -QDMA_ERR_RM_DEV_NOT_EXISTS);
        return -QDMA_ERR_RM_DEV_NOT_EXISTS;
    }

    // MD: Lock the resource to safely increment the active queue count
    qdma_resource_lock_take();
    switch (q_type) {
    case QDMA_DEV_Q_TYPE_H2C:
        active_qcnt = &dev_entry->active_h2c_qcnt;
        break;
    case QDMA_DEV_Q_TYPE_C2H:
        active_qcnt = &dev_entry->active_c2h_qcnt;
        break;
    case QDMA_DEV_Q_TYPE_CMPT:
        active_qcnt = &dev_entry->active_cmpt_qcnt;
        break;
    default:
        rv = -QDMA_ERR_RM_DEV_NOT_EXISTS;
    }

    // MD: Check if there are queues available to increment
    if (active_qcnt && (dev_entry->entry.total_q < ((*active_qcnt) + 1))) {
        qdma_resource_lock_give();
        return -QDMA_ERR_RM_NO_QUEUES_LEFT;
    }

    // MD: Increment the active queue count
    if (active_qcnt) {
        *active_qcnt = (*active_qcnt) + 1;
        q_resource->active_qcnt++;
    }
    qdma_resource_lock_give();

    return rv;
}

/* MD:*
 * qdma_dev_decrement_active_queue() - Decrement the active queue count
 *
 * @dma_device_index: DMA device index
 * @func_id: Function ID
 * @q_type: Queue type (H2C, C2H, CMPT)
 *
 * Return: QDMA_SUCCESS on success, negative error code on failure
 */
int qdma_dev_decrement_active_queue(uint32_t dma_device_index, uint16_t func_id,
                                    enum qdma_dev_q_type q_type)
{
    struct qdma_resource_master *q_resource =
            qdma_get_master_resource_entry(dma_device_index);
    struct qdma_dev_entry *dev_entry;
    int rv = QDMA_SUCCESS;

    // MD: Check if the queue resource exists
    if (!q_resource) {
        qdma_log_error("%s: Queue resource not found, err: %d\n",
                __func__, -QDMA_ERR_RM_RES_NOT_EXISTS);
        return -QDMA_ERR_RM_RES_NOT_EXISTS;
    }

    // MD: Retrieve the device entry for the given function ID
    dev_entry = qdma_get_dev_entry(dma_device_index, func_id);

    // MD: Check if the device entry exists
    if (!dev_entry) {
        qdma_log_error("%s: Dev entry not found, err: %d\n",
                __func__, -QDMA_ERR_RM_DEV_NOT_EXISTS);
        return -QDMA_ERR_RM_DEV_NOT_EXISTS;
    }

    // MD: Lock the resource to safely decrement the active queue count
    qdma_resource_lock_take();
    switch (q_type) {
    case QDMA_DEV_Q_TYPE_H2C:
        if (dev_entry->active_h2c_qcnt)
            dev_entry->active_h2c_qcnt--;
        break;
    case QDMA_DEV_Q_TYPE_C2H:
        if (dev_entry->active_c2h_qcnt)
            dev_entry->active_c2h_qcnt--;
        break;
    case QDMA_DEV_Q_TYPE_CMPT:
        if (dev_entry->active_cmpt_qcnt)
            dev_entry->active_cmpt_qcnt--;
        break;
    default:
        rv = -QDMA_ERR_RM_DEV_NOT_EXISTS;
    }
    q_resource->active_qcnt--;
    qdma_resource_lock_give();

    return rv;
}

/* MD:*
 * qdma_get_active_queue_count() - Get the total active queue count
 *
 * @dma_device_index: DMA device index
 *
 * Return: Total active queue count
 */
uint32_t qdma_get_active_queue_count(uint32_t dma_device_index)
{
    struct qdma_resource_master *q_resource =
            qdma_get_master_resource_entry(dma_device_index);
    uint32_t q_cnt;

    // MD: Check if the queue resource exists
    if (!q_resource)
        return QDMA_SUCCESS;

    // MD: Lock the resource to safely access the active queue count
    qdma_resource_lock_take();
    q_cnt = q_resource->active_qcnt;
    qdma_resource_lock_give();

    return q_cnt;
}

/* MD:*
 * qdma_get_device_active_queue_count() - Get the active queue count for a specific device
 *
 * @dma_device_index: DMA device index
 * @func_id: Function ID
 * @q_type: Queue type (H2C, C2H, CMPT)
 *
 * Return: Active queue count for the specified device and queue type
 */
int qdma_get_device_active_queue_count(uint32_t dma_device_index,
                                       uint16_t func_id,
                                       enum qdma_dev_q_type q_type)
{
    struct qdma_resource_master *q_resource =
            qdma_get_master_resource_entry(dma_device_index);
    struct qdma_dev_entry *dev_entry;
    uint32_t dev_active_qcnt = 0;

    // MD: Check if the queue resource exists
    if (!q_resource)
        return -QDMA_ERR_RM_RES_NOT_EXISTS;

    // MD: Retrieve the device entry for the given function ID
    dev_entry = qdma_get_dev_entry(dma_device_index, func_id);

    // MD: Check if the device entry exists
    if (!dev_entry)
        return -QDMA_ERR_RM_DEV_NOT_EXISTS;

    // MD: Lock the resource to safely access the active queue count
    qdma_resource_lock_take();
    switch (q_type) {
    case QDMA_DEV_Q_TYPE_H2C:
        dev_active_qcnt = dev_entry->active_h2c_qcnt;
        break;
    case QDMA_DEV_Q_TYPE_C2H:
        dev_active_qcnt = dev_entry->active_c2h_qcnt;
        break;
    case QDMA_DEV_Q_TYPE_CMPT:
        dev_active_qcnt = dev_entry->active_cmpt_qcnt;
        break;
    default:
        dev_active_qcnt = 0;
    }
    qdma_resource_lock_give();

    return dev_active_qcnt;
}
