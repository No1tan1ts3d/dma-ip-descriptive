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

#define pr_fmt(fmt)     KBUILD_MODNAME ":%s: " fmt, __func__

#include "qdma_descq.h"

#include <asm/cacheflush.h>
#include <linux/kernel.h>
#include <linux/delay.h>

#include "qdma_device.h"
#include "qdma_intr.h"
#include "qdma_regs.h"
#include "qdma_thread.h"
#include "qdma_context.h"
#include "thread.h"
#include "qdma_compat.h"
#include "qdma_st_c2h.h"
#include "qdma_access_common.h"
#include "qdma_ul_ext.h"
#include "version.h"

/* MD:
 * ST C2H descq (i.e., freelist) RX buffers
 */

// MD: Function to free a single software scatter-gather descriptor
static inline void flq_free_one(struct qdma_sw_sg *sdesc, struct qdma_c2h_desc *desc)
{
    if (sdesc) {
        // MD: Check if the DMA address is set and clear it
        if (sdesc->dma_addr) {
            desc->dst_addr = 0UL;
            sdesc->dma_addr = 0UL;
        }

        // MD: Check if the page is set and clear it
        if (sdesc->pg) {
            sdesc->pg = NULL;
            sdesc->offset = 0;
        }
    } else {
        // MD: Log an error if the software descriptor is NULL
        pr_err("%s: sdesc is NULL", __func__);
    }
}

// MD: Function to fill a single software scatter-gather descriptor
static inline int flq_fill_one(struct qdma_descq *descq, struct qdma_sw_sg *sdesc, struct qdma_c2h_desc *desc)
{
    struct qdma_flq *flq = (struct qdma_flq *)descq->flq;
    struct qdma_sw_pg_sg *pg_sdesc;
    unsigned int pg_idx = 0;
    unsigned int buf_sz;

    // MD: Check if the freelist queue is NULL
    if (!flq) {
        pr_err("%s: flq is NULL", __func__);
        return -EINVAL;
    }

    // MD: Check if the number of pages is zero
    if (flq->num_pages == 0) {
        pr_err("%s: flq->num_pages is NULL", __func__);
        return -EINVAL;
    }

    buf_sz = flq->desc_buf_size;
    pg_idx = (flq->alloc_idx & flq->num_pgs_mask);

    // MD: Check if the page index is valid
    if (pg_idx >= flq->num_pages) {
        pr_err("%s: pg_idx %d is Invalid", __func__, pg_idx);
        return -EINVAL;
    }

    pg_sdesc = flq->pg_sdesc + pg_idx;

    // MD: Check if the page descriptor or its base is NULL
    if (!pg_sdesc || !pg_sdesc->pg_base) {
        pr_err("%s: pg_sdesc is NULL", __func__);
        return -EINVAL;
    }

    // MD: Check if there is enough space in the page
    if (pg_sdesc->pg_offset + buf_sz > flq->max_pg_offset) {
        pr_err("%s: memory full, alloc_idx %u, recycle_idx = %u offset = %u, pg_idx = %u",
               __func__, flq->alloc_idx, flq->recycle_idx, pg_sdesc->pg_offset, pg_idx);
        return -ENOMEM;
    }

    // MD: Set the software descriptor fields
    sdesc->pg = pg_sdesc->pg_base;
    sdesc->offset = pg_sdesc->pg_offset;
    sdesc->dma_addr = pg_sdesc->pg_dma_base_addr + pg_sdesc->pg_offset;
    sdesc->len = descq->conf.c2h_bufsz;
    desc->dst_addr = sdesc->dma_addr;

    // MD: Increment the page reference count
#if KERNEL_VERSION(4, 6, 0) < LINUX_VERSION_CODE
    page_ref_inc(pg_sdesc->pg_base);
#else
    atomic_inc(&(pg_sdesc->pg_base->_count));
#endif

    // MD: Update the page offset and allocation index if necessary
    pg_sdesc->pg_offset += buf_sz;
    if ((pg_sdesc->pg_offset + buf_sz) > flq->max_pg_offset)
        flq->alloc_idx++;

    return 0;
}

// MD: Function to unmap a single page from the device
static inline void flq_unmap_page_one(struct qdma_sw_pg_sg *pg_sdesc, struct device *dev, unsigned char pg_order)
{
    // MD: Check if the page descriptor and its DMA base address are valid
    if (pg_sdesc && pg_sdesc->pg_dma_base_addr) {
        // MD: Unmap the page from the device
        dma_unmap_page(dev, pg_sdesc->pg_dma_base_addr, PAGE_SIZE << pg_order, DMA_FROM_DEVICE);
        pg_sdesc->pg_dma_base_addr = 0UL;
    }
}

/* MD:
 * Free a single page in the QDMA software page scatter-gather list
 */
static inline void flq_free_page_one(struct qdma_sw_pg_sg *pg_sdesc,
                                     struct device *dev,
                                     unsigned char pg_order,
                                     unsigned int pg_shift)
{
    unsigned int page_count = 0;
    unsigned int i = 0;

    // MD: Check if the page descriptor and base are valid
    if (pg_sdesc && pg_sdesc->pg_base) {
        // MD: Calculate the number of pages to free (+1 for dma_unmap)
        page_count = (pg_sdesc->pg_offset >> pg_shift) + 1;

        // MD: Unmap the DMA page
        flq_unmap_page_one(pg_sdesc, dev, pg_order);

        // MD: Release each page
        for (i = 0; i < page_count; i++)
            put_page(pg_sdesc->pg_base);

        // MD: Reset the page descriptor
        pg_sdesc->pg_base = NULL;
        pg_sdesc->pg_dma_base_addr = 0UL;
    }
}

/* MD:
 * Free all page resources associated with a QDMA descriptor queue
 */
void descq_flq_free_page_resource(struct qdma_descq *descq)
{
    struct xlnx_dma_dev *xdev = descq->xdev;
    struct device *dev = &xdev->conf.pdev->dev;
    struct qdma_flq *flq = (struct qdma_flq *)descq->flq;
    struct qdma_sw_pg_sg *pg_sdesc = flq->pg_sdesc;
    unsigned char pg_order = flq->desc_pg_order;
    int i;

    // MD: Free each page in the scatter-gather list
    for (i = 0; i < flq->num_pages; i++, pg_sdesc++)
        flq_free_page_one(pg_sdesc, dev, pg_order, flq->desc_pg_shift);

    // MD: Free the scatter-gather list and reset the structure
    kfree(flq->pg_sdesc);
    flq->pg_sdesc = NULL;
    memset(flq, 0, sizeof(struct qdma_flq));
}

/* MD:
 * Free all resources associated with a QDMA descriptor queue
 */
void descq_flq_free_resource(struct qdma_descq *descq)
{
    struct qdma_flq *flq = (struct qdma_flq *)descq->flq;
    struct qdma_sw_sg *sdesc = flq->sdesc;
    struct qdma_c2h_desc *desc = flq->desc;
    int i;

    // MD: Ensure the software descriptor is valid
    if (!sdesc) {
        pr_err("%s: sdesc is Invalid", __func__);
        return;
    }

    // MD: Free each descriptor in the queue
    for (i = 0; i < flq->size; i++, sdesc++, desc++)
        flq_free_one(sdesc, desc);

    // MD: Free the software descriptor list and reset the structure
    kfree(flq->sdesc);
    flq->sdesc = NULL;
    flq->sdesc_info = NULL;
}

/* MD:
 * Allocate and map a single page in the QDMA software page scatter-gather list
 */
static inline int flq_fill_page_one(struct qdma_sw_pg_sg *pg_sdesc,
                                    struct device *dev,
                                    int node, unsigned char pg_order, gfp_t gfp)
{
    struct page *pg;
    dma_addr_t mapping;

    // MD: Allocate a page with the specified order
    pg = alloc_pages_node(node, __GFP_COMP | gfp, pg_order);
    if (unlikely(!pg)) {
        pr_err("%s: failed to allocate the pages, order %d.\n",
               __func__, pg_order);
        return -ENOMEM;
    }

    // MD: Map the page for DMA
    mapping = dma_map_page(dev, pg, 0, (PAGE_SIZE << pg_order), DMA_FROM_DEVICE);
    if (unlikely(dma_mapping_error(dev, mapping))) {
        dev_err(dev, "page 0x%p mapping error 0x%llx.\n",
                pg, (unsigned long long)mapping);
        __free_pages(pg, pg_order);
        return -EINVAL;
    }

    // MD: Set the page descriptor fields
    pg_sdesc->pg_base = pg;
    pg_sdesc->pg_dma_base_addr = mapping;
    pg_sdesc->pg_offset = 0;
    return 0;
}

int descq_flq_alloc_resource(struct qdma_descq *descq)
{
    struct xlnx_dma_dev *xdev = descq->xdev;
    struct qdma_flq *flq = (struct qdma_flq *)descq->flq;
    struct device *dev = &xdev->conf.pdev->dev;
    int node = dev_to_node(dev);
    struct qdma_sw_pg_sg *pg_sdesc = NULL;
    struct qdma_sw_sg *sdesc, *prev = NULL;
    struct qdma_sdesc_info *sinfo, *sprev = NULL;
    struct qdma_c2h_desc *desc = flq->desc;
    int i;
    int rv = 0;
    unsigned int div_bits = 0;

    // MD: Calculate the number of buffers per page
    div_bits = flq->desc_pg_shift;
    flq->num_bufs_per_pg = (flq->max_pg_offset >> div_bits);

    // MD: Check if the number of buffers per page is zero
    if (flq->num_bufs_per_pg == 0) {
        pr_err("%s: conf_c2h_bufsz = %d, flq_c2h_buf_size = %d, flq->max_pg_offset = %d, flq->num_bufs_per_pg = %d, flq->desc_pg_order = %d, div_bits = %d",
               __func__,
               descq->conf.c2h_bufsz,
               flq->desc_buf_size,
               flq->max_pg_offset,
               flq->num_bufs_per_pg,
               flq->desc_pg_order,
               div_bits);
        return -EINVAL;
    }

    // MD: Calculate the number of pages required
    flq->num_pages = (flq->size / flq->num_bufs_per_pg) +
                     ((flq->size % flq->num_bufs_per_pg) == 0 ? 0 : 1);
    flq->num_pages = get_next_powof2(flq->num_pages);

    // MD: Ensure the number of pages is sufficient to prevent index overlap
    if (flq->num_pages < flq->size)
        flq->num_pages = (flq->num_pages << 1);
    flq->num_pgs_mask = (flq->num_pages - 1);

    pr_debug("%s: conf_c2h_bufsz = %d, flq_c2h_buf_size = %d, num_bufs_per_pg = %d, num_pages = %d, flq->size = %d",
             __func__,
             descq->conf.c2h_bufsz,
             flq->desc_buf_size,
             flq->num_bufs_per_pg,
             flq->num_pages,
             flq->size);

    // MD: Allocate memory for page descriptors
    pg_sdesc = kzalloc_node(flq->num_pages * (sizeof(struct qdma_sw_pg_sg)),
                            GFP_KERNEL, node);

    if (!pg_sdesc) {
        pr_err("%s: OOM, sz %d * %ld.\n",
               __func__,
               flq->num_pages,
               (sizeof(struct qdma_sw_pg_sg)));
        return -ENOMEM;
    }
    flq->pg_sdesc = pg_sdesc;

    // MD: Fill each page with descriptors
    for (pg_sdesc = flq->pg_sdesc, i = 0; i < flq->num_pages; i++, pg_sdesc++) {
        rv = flq_fill_page_one(pg_sdesc, dev, node, flq->desc_pg_order, GFP_KERNEL);
        if (rv < 0) {
            descq_flq_free_page_resource(descq);
            return rv;
        }
    }

    // MD: Allocate memory for scatter-gather descriptors and info
    sdesc = kzalloc_node(flq->size * (sizeof(struct qdma_sw_sg) +
                                      sizeof(struct qdma_sdesc_info)),
                         GFP_KERNEL, node);
    if (!sdesc) {
        pr_err("%s: OOM, sz %d * %ld.\n",
               __func__,
               flq->size,
               ((sizeof(struct qdma_sw_sg) +
                 sizeof(struct qdma_sdesc_info))));
        descq_flq_free_page_resource(descq);
        return -ENOMEM;
    }

    flq->sdesc = sdesc;
    flq->sdesc_info = sinfo = (struct qdma_sdesc_info *)(sdesc + flq->size);
    flq->alloc_idx = 0;

    // MD: Create a linked list ring for the scatter-gather descriptors
    for (i = 0; i < flq->size; i++, prev = sdesc, sdesc++, sprev = sinfo, sinfo++) {
        if (prev)
            prev->next = sdesc;
        if (sprev)
            sprev->next = sinfo;
    }

    // MD: Link the last entry to the first to complete the ring
    prev->next = flq->sdesc;
    sprev->next = flq->sdesc_info;

    // MD: Fill each scatter-gather descriptor
    for (sdesc = flq->sdesc, i = 0; i < flq->size; i++, sdesc++, desc++) {
        rv = flq_fill_one(descq, sdesc, desc);
        if (rv < 0) {
            descq_flq_free_resource(descq);
            descq_flq_free_page_resource(descq);
            return rv;
        }
    }

    return 0;
}

static inline int flq_refill_pages(struct qdma_descq *descq,
                                   int count, bool recycle, gfp_t gfp)
{
    struct xlnx_dma_dev *xdev = descq->xdev;
    struct device *dev = &xdev->conf.pdev->dev;
    int node = dev_to_node(dev);
    struct qdma_flq *flq = (struct qdma_flq *)descq->flq;
    unsigned int n_recycle_index = flq->recycle_idx;
    unsigned int div_bits = flq->desc_pg_shift;
    struct qdma_sw_pg_sg *pg_sdesc = flq->pg_sdesc;
    unsigned int free_bufs_in_pg;
    unsigned int i = 0, j = 0;
    int rv;

    // MD: Loop through the number of pages to refill
    for (i = 0; i < count; ) {
        // MD: Calculate the current page descriptor based on the recycle index
        pg_sdesc = flq->pg_sdesc + (n_recycle_index & flq->num_pgs_mask);

        // MD: Check if the page descriptor is valid
        if (!pg_sdesc) {
            pr_err("pg_sdesc is NULL");
            return -EINVAL;
        }

        // MD: Calculate the number of free buffers in the current page
        free_bufs_in_pg = (pg_sdesc->pg_offset >> div_bits);
        free_bufs_in_pg = min_t(unsigned int, free_bufs_in_pg, count - i);

        // MD: Refill the buffers in the current page
        for (j = 0; j < free_bufs_in_pg; j++) {
            pg_sdesc->pg_offset -= flq->desc_buf_size;
            if (!recycle && !descq->conf.fp_descq_c2h_packet)
                put_page(pg_sdesc->pg_base);
        }
        i += free_bufs_in_pg;

        // MD: Ensure the loop does not exceed the count
        BUG_ON(i > count);

        // MD: Move to the next page if the current page is fully utilized
        if (!pg_sdesc->pg_offset)
            n_recycle_index++;
    }

    // MD: Update the recycle index if recycling is enabled
    if (recycle) {
        flq->recycle_idx = n_recycle_index;
    } else {
        // MD: Refill pages until the recycle index catches up with the allocation index
        while (1) {
            pg_sdesc = flq->pg_sdesc + (flq->recycle_idx & flq->num_pgs_mask);

            // MD: Check if the page descriptor is valid
            if (!pg_sdesc) {
                pr_err("pg_sdesc is NULL");
                return -EINVAL;
            }

            // MD: Stop if the page offset is not zero or recycle index equals allocation index
            if (pg_sdesc->pg_offset || flq->recycle_idx == flq->alloc_idx)
                break;

            // MD: Unmap and refill the page
            flq_unmap_page_one(pg_sdesc, dev, flq->desc_pg_order);
            put_page(pg_sdesc->pg_base);
            rv = flq_fill_page_one(pg_sdesc, dev, node, flq->desc_pg_order, gfp);
            if (rv < 0)
                break;

            flq->recycle_idx++;
        }
    }

    return 0;
}

/* MD:
 * Refill the Free List Queue (FLQ) for a QDMA descriptor queue
 */
static int qdma_flq_refill(struct qdma_descq *descq, int idx, int count,
                           int recycle, gfp_t gfp)
{
    struct qdma_flq *flq = (struct qdma_flq *)descq->flq;
    struct qdma_sw_sg *sdesc = flq->sdesc + idx;
    struct qdma_c2h_desc *desc = flq->desc + idx;
    struct qdma_sdesc_info *sinfo = flq->sdesc_info + idx;
    int i;
    int rv;

    // MD: If not recycling, refill pages
    if (!recycle) {
        rv = flq_refill_pages(descq, count, recycle, gfp);
        if (unlikely(rv < 0)) {
            pr_err("%s: flq_refill_pages failed rv %d error",
                   descq->conf.name, rv);
        }
    }

    // MD: Iterate over the count to refill descriptors
    for (i = 0; i < count; i++, idx++, sdesc++, desc++, sinfo++) {
        // MD: Wrap around if index exceeds the size
        if (idx == flq->size) {
            idx = 0;
            sdesc = flq->sdesc;
            desc = flq->desc;
            sinfo = flq->sdesc_info;
        }

        // MD: If recycling, set the length of the descriptor
        if (recycle) {
            sdesc->len = descq->conf.c2h_bufsz;
        } else {
            // MD: Free the descriptor and refill it
            flq_free_one(sdesc, desc);
            rv = flq_fill_one(descq, sdesc, desc);
            if (unlikely(rv < 0)) {
                pr_err("%s: rv %d error", descq->conf.name, rv);
                if (rv == -ENOMEM)
                    flq->alloc_fail++;
                else
                    flq->mapping_err++;

                break;
            }
        }
        // MD: Reset the status bits and increment available descriptors
        sinfo->fbits = 0;
        descq->avail++;
    }

    // MD: Check if work and pending lists are empty
    if (list_empty(&descq->work_list) &&
        list_empty(&descq->pend_list)) {
        descq->pend_list_empty = 1;
        // MD: Wake up the wait queue if the queue is in stop-wait state
        if (descq->q_stop_wait)
            qdma_waitq_wakeup(&descq->pend_list_wq);
    }

    return i; // MD: Return the number of descriptors processed
}

/* MD:
 * Read descriptors from a C2H (Card to Host) queue
 */
int descq_st_c2h_read(struct qdma_descq *descq, struct qdma_request *req,
                      bool update_pidx, bool refill)
{
    // MD: Retrieve the request control block
    struct qdma_sgt_req_cb *cb = qdma_req_cb_get(req);
    struct qdma_flq *flq = (struct qdma_flq *)descq->flq;
    unsigned int pidx = flq->pidx_pend;
    struct qdma_sw_sg *fsg = flq->sdesc + pidx;
    struct qdma_sw_sg *tsg = req->sgl;
    unsigned int fsgcnt = ring_idx_delta(descq->pidx, pidx, flq->size);
    unsigned int tsgoff = cb->sg_offset;
    unsigned int foff = 0;
    int i = 0, j = 0;
    int rv = 0;
    unsigned int copied = 0;

    // MD: Return if there are no descriptors to process
    if (!fsgcnt)
        return 0;

    // MD: Adjust the scatter-gather list to the current index
    if (cb->sg_idx) {
        for (; tsg && j < cb->sg_idx; j++)
            tsg = tsg->next;

        if (!tsg) {
            pr_err("tsg error, index %u/%u.\n", cb->sg_idx, req->sgcnt);
            return -EINVAL;
        }
    }

    // MD: Process each descriptor in the queue
    while ((i < fsgcnt) && tsg) {
        unsigned int flen = fsg->len;
        unsigned char *faddr = page_address(fsg->pg) + fsg->offset;

        foff = 0;

        // MD: Process each segment in the scatter-gather list
        while (flen && tsg) {
            unsigned int toff = tsg->offset + tsgoff;
            unsigned int copy = min_t(unsigned int, flen, tsg->len - tsgoff);
            u64 *pkt_tx_time = (u64 *)(page_address(fsg->pg) + fsg->offset);

            // MD: Copy data if memory copy is not disabled
            if (!req->no_memcpy) {
                memcpy(page_address(tsg->pg) + toff, faddr, copy);
                flush_dcache_page(tsg->pg);
            }

            // MD: Handle ping-pong mode for latency measurement
            if (descq->conf.ping_pong_en && *pkt_tx_time == descq->ping_pong_tx_time) {
                u64 latency;

                pr_debug("pkt tx_time=%llu\n", *pkt_tx_time);

                latency = descq->ping_pong_rx_time - *pkt_tx_time;

                // MD: Calculate minimum latency
                if (descq->xdev->ping_pong_lat_min > latency || descq->xdev->ping_pong_lat_min == 0) {
                    pr_info("ping_pong_lat_min=%llu curr=%llu\n", descq->xdev->ping_pong_lat_min, latency);
                    descq->xdev->ping_pong_lat_min = latency;
                }

                // MD: Calculate maximum latency
                if (descq->xdev->ping_pong_lat_max < latency || descq->xdev->ping_pong_lat_max == 0) {
                    pr_info("ping_pong_lat_max=%llu curr=%llu\n", descq->xdev->ping_pong_lat_max, latency);
                    descq->xdev->ping_pong_lat_max = latency;
                }

                // MD: Sum of latencies for average calculation
                descq->xdev->ping_pong_lat_total += latency;
            } else if (descq->conf.ping_pong_en && *pkt_tx_time != descq->ping_pong_tx_time) {
                pr_err("Error: pkt tx=%llu descq->tx_time=%llu %p\n", *pkt_tx_time, descq->ping_pong_tx_time, descq);
            }

            // MD: Update pointers and counters
            faddr += copy;
            flen -= copy;
            foff += copy;
            tsgoff += copy;
            copied += copy;

            // MD: Move to the next segment if the current one is fully processed
            if (tsgoff == tsg->len) {
                tsg = tsg->next;
                tsgoff = 0;
                j++;
            }
        }

        // MD: Move to the next descriptor if the current one is fully processed
        if (foff == fsg->len) {
            pidx = ring_idx_incr(pidx, 1, descq->conf.rngsz);
            i++;
            foff = 0;
            fsg = fsg->next;
        }
    }

    // MD: Increment the completion descriptor count
    incr_cmpl_desc_cnt(descq, i);

    // MD: Refill the queue if required
    if (refill && i)
        qdma_flq_refill(descq, flq->pidx_pend, i, 1, GFP_ATOMIC);

    // MD: Update the pending index
    flq->pidx_pend = ring_idx_incr(flq->pidx_pend, i, flq->size);
    if (foff) {
        fsg->offset += foff;
        fsg->len -= foff;
    }

    // MD: Update the producer index if required
    if (i && update_pidx) {
        i = ring_idx_decr(flq->pidx_pend, 1, flq->size);
        descq->pidx_info.pidx = i;
        rv = queue_pidx_update(descq->xdev, descq->conf.qidx, descq->conf.q_type, &descq->pidx_info);
        if (unlikely(rv < 0)) {
            pr_err("%s: Failed to update pidx\n", descq->conf.name);
            return -EINVAL;
        }
    }

    // MD: Update the request control block
    cb->sg_idx = j;
    cb->sg_offset = tsgoff;
    cb->left -= copied;

    // MD: Update the packet data length
    flq->pkt_dlen -= copied;

    return copied;
}

/* MD:
 * Process default C2H packets for a QDMA descriptor queue
 */
static int qdma_c2h_packets_proc_dflt(struct qdma_descq *descq)
{
    struct qdma_sgt_req_cb *cb = NULL, *tmp = NULL;

    // MD: Iterate over each entry in the pending list safely
    list_for_each_entry_safe(cb, tmp, &descq->pend_list, list) {
        int rv;

        // MD: Check for zero-length DMA transfer
        if (!cb->left) {
            pr_debug("%s, cb 0x%p pending, zero len.\n", descq->conf.name, cb);
            qdma_sgt_req_done(descq, cb, 0);
            return 0;
        }

        // MD: Read data from the descriptor queue
        rv = descq_st_c2h_read(descq, (struct qdma_request *)cb, 0, 0);
        if (rv < 0) {
            pr_info("req 0x%p, error %d.\n", cb, rv);
            qdma_sgt_req_done(descq, cb, rv);
            continue;
        }

        // MD: Complete the request if no data is left
        if (!cb->left)
            qdma_sgt_req_done(descq, cb, 0);
        else
            break; // MD: No more data available
    }

    return 0;
}

/* MD:
 * Move to the next completion entry in the descriptor queue
 */
void cmpt_next(struct qdma_descq *descq)
{
    // MD: Move the current completion descriptor pointer forward
    u8 *desc_cmpt_cur = (u8 *)descq->desc_cmpt_cur + descq->cmpt_entry_len;
    descq->desc_cmpt_cur = desc_cmpt_cur;

    // MD: Check if the completion index has wrapped around
    if (unlikely(++descq->cidx_cmpt == descq->conf.rngsz_cmpt)) {
        descq->cidx_cmpt = 0;
        descq->color ^= 1; // MD: Toggle the color bit
        descq->desc_cmpt_cur = descq->desc_cmpt; // MD: Reset to the start
    }
}

/* MD:
 * Check if a completion entry is new based on the color bit
 */
static inline bool is_new_cmpl_entry(struct qdma_descq *descq,
                                     struct qdma_ul_cmpt_info *cmpl)
{
    return cmpl->f.color == descq->color;
}

/* MD:
 * Parse a completion entry from the descriptor queue
 */
int parse_cmpl_entry(struct qdma_descq *descq, struct qdma_ul_cmpt_info *cmpl)
{
    __be64 *cmpt = (__be64 *)descq->desc_cmpt_cur;

    // MD: Ensure memory ordering
    dma_rmb();

#if 0
    // MD: Debug: Print the completion entry in hex format
    print_hex_dump(KERN_INFO, "cmpl entry ", DUMP_PREFIX_OFFSET,
                   16, 1, (void *)cmpt, descq->cmpt_entry_len, false);
#endif

    // MD: Parse the completion entry fields
    cmpl->entry = cmpt;
    cmpl->f.format = (cmpt[0] & F_C2H_CMPT_ENTRY_F_FORMAT) ? 1 : 0;
    cmpl->f.color = (cmpt[0] & F_C2H_CMPT_ENTRY_F_COLOR) ? 1 : 0;
    cmpl->f.err = (cmpt[0] & F_C2H_CMPT_ENTRY_F_ERR) ? 1 : 0;
    cmpl->f.eot = (cmpt[0] & F_C2H_CMPT_ENTRY_F_EOT) ? 1 : 0;
    cmpl->f.desc_used = (cmpt[0] & F_C2H_CMPT_ENTRY_F_DESC_USED) ? 1 : 0;

    // MD: Determine the length of the completion entry
    if (!cmpl->f.format && cmpl->f.desc_used) {
        cmpl->len = (cmpt[0] >> S_C2H_CMPT_ENTRY_LENGTH) & M_C2H_CMPT_ENTRY_LENGTH;
        // MD: Zero length transfer allowed
    } else {
        cmpl->len = 0;
    }

    return 0;
}

/* MD:
 * Calculate the number of fragments needed for a given length
 */
static int get_fl_nr(unsigned int len,
                     unsigned int c2h_bufsz,
                     unsigned int pg_shift, unsigned int pg_mask,
                     unsigned int *last_len)
{
    unsigned int l_len = len;
    int l_fl_nr = 1;

    // MD: Check if the length is a power of 2
    if ((len & (len - 1)) == 0) {
        // MD: pr_info("Len is ^2");
        l_fl_nr = len ? ((len + pg_mask) >> pg_shift) : 1;
        l_len = (len & pg_mask);
        if (l_len == 0)
            l_len = c2h_bufsz;
    } else {
        // MD: pr_info("Len is non ^2");
        if (len) {
            if (len <= c2h_bufsz)
                l_fl_nr = 1;
            else {
                l_fl_nr = 0;
                while (l_len >= c2h_bufsz) {
                    l_fl_nr++;
                    l_len -= c2h_bufsz;
                }

                if (l_len)
                    l_fl_nr++;
                else
                    l_len = c2h_bufsz;
            }
        }
    }

    *last_len = l_len;
    return l_fl_nr;
}

/* MD:
 * Process received packet
 */
static int rcv_pkt(struct qdma_descq *descq, struct qdma_ul_cmpt_info *cmpl,
                   unsigned int len)
{
    unsigned int pidx = cmpl->pidx;
    struct qdma_flq *flq = (struct qdma_flq *)descq->flq;
    unsigned int pg_shift = flq->desc_pg_shift;
    unsigned int pg_mask = (1 << pg_shift) - 1;
    unsigned int rngsz = descq->conf.rngsz;
    unsigned int l_len = 0;
    int fl_nr = get_fl_nr(len, descq->conf.c2h_bufsz, pg_shift, pg_mask, &l_len);
    unsigned int last = ring_idx_incr(cmpl->pidx, fl_nr - 1, rngsz);
    unsigned int next = ring_idx_incr(last, 1, rngsz);
    struct qdma_sw_sg *sdesc = flq->sdesc + last;
    unsigned int cidx_next = ring_idx_incr(descq->cidx_cmpt, 1, descq->conf.rngsz_cmpt);

    // MD: Handle zero-length packets
    if (!len)
        sdesc->len = 0;
    else
        sdesc->len = l_len;

    // MD: Check if there are enough available descriptors
    if (descq->avail < fl_nr)
        return -EBUSY;

    // MD: Update available descriptors
    descq->avail -= fl_nr;

    // MD: Process packet using callback if available
    if (descq->conf.fp_descq_c2h_packet) {
        int rv = descq->conf.fp_descq_c2h_packet(descq->q_hndl, descq->conf.quld, len, fl_nr, flq->sdesc + pidx,
                                                 descq->conf.cmpl_udd_en ? (unsigned char *)cmpl->entry : NULL);

        if (rv < 0)
            return rv;
        flq->pidx_pend = next;
    } else {
        int i;
        struct qdma_sdesc_info *sinfo = flq->sdesc_info + pidx;

        // MD: Mark descriptors as valid and update completion index
        for (i = 0; i < fl_nr; i++, sinfo = sinfo->next) {
            WARN_ON(sinfo->f.valid);
            sinfo->f.valid = 1;
            sinfo->cidx = cidx_next;
        }

        // MD: Set Start-of-Packet (SOP) and End-of-Packet (EOP) flags
        flq->sdesc_info[pidx].f.sop = 1;
        flq->sdesc_info[last].f.eop = 1;

        // MD: Update packet data length and UDD count
        flq->pkt_dlen += len;
        if (descq->conf.cmpl_udd_en)
            flq->udd_cnt++;
    }
    cmpl->pidx = next;

    return 0;
}

/* MD:
 * Process UDD only completion
 */
int rcv_udd_only(struct qdma_descq *descq, struct qdma_ul_cmpt_info *cmpl)
{
#ifdef XMP_DISABLE_ST_C2H_CMPL
    __be64 cmpt_entry = cmpl->entry[0];
#endif
    struct qdma_flq *flq = (struct qdma_flq *)descq->flq;

    pr_debug("%s, rcv udd.\n", descq->conf.name);
#if 0
    print_hex_dump(KERN_INFO, "cmpl entry: ", DUMP_PREFIX_OFFSET,
                   16, 1, (void *)cmpl->entry, descq->cmpt_entry_len,
                   false);
#endif

    // MD: Process UDD only without using descriptors
    if (descq->conf.fp_descq_c2h_packet) {
        return descq->conf.fp_descq_c2h_packet(descq->q_hndl, descq->conf.quld, 0, 0, NULL,
                                               (unsigned char *)cmpl->entry);
    }
#ifdef XMP_DISABLE_ST_C2H_CMPL
    if ((cmpt_entry & (1 << 20)) > 0) {
        __be16 pkt_cnt = (cmpt_entry >> 32) & 0xFFFF;
        __be16 pkt_len = (cmpt_entry >> 48) & 0xFFFF;
        int i;

        pr_info("pkt %u * %u.\n", pkt_len, pkt_cnt);
        for (i = 0; i < pkt_cnt; i++) {
            int rv = rcv_pkt(descq, cmpl, pkt_len);

            if (rv < 0)
                break;
        }
    }
#endif
    flq->udd_cnt++;

    return 0;
}

/* MD:
 * Adjust C2H counter averages
 */
static void descq_adjust_c2h_cntr_avgs(struct qdma_descq *descq)
{
    int i;
    struct xlnx_dma_dev *xdev = descq->xdev;
    struct global_csr_conf *csr_info = &xdev->csr_info;
    uint8_t latecy_optimized = descq->conf.latency_optimize;

    // MD: Update moving average for pending packets
    descq->c2h_pend_pkt_moving_avg = csr_info->c2h_cnt_th[descq->cmpt_cidx_info.counter_idx];

    // MD: Determine high threshold index
    if (descq->sorted_c2h_cntr_idx == (QDMA_GLOBAL_CSR_ARRAY_SZ - 1))
        i = xdev->sorted_c2h_cntr_idx[descq->sorted_c2h_cntr_idx];
    else
        i = xdev->sorted_c2h_cntr_idx[descq->sorted_c2h_cntr_idx + 1];

    // MD: Calculate high threshold
    if (latecy_optimized)
        descq->c2h_pend_pkt_avg_thr_hi = (csr_info->c2h_cnt_th[i] + csr_info->c2h_cnt_th[i]);
    else
        descq->c2h_pend_pkt_avg_thr_hi = (descq->c2h_pend_pkt_moving_avg + csr_info->c2h_cnt_th[i]);

    // MD: Determine low threshold index
    if (descq->sorted_c2h_cntr_idx > 0)
        i = xdev->sorted_c2h_cntr_idx[descq->sorted_c2h_cntr_idx - 1];
    else
        i = xdev->sorted_c2h_cntr_idx[descq->sorted_c2h_cntr_idx];

    // MD: Calculate low threshold
    if (latecy_optimized)
        descq->c2h_pend_pkt_avg_thr_lo = (csr_info->c2h_cnt_th[i] + csr_info->c2h_cnt_th[i]);
    else
        descq->c2h_pend_pkt_avg_thr_lo = (descq->c2h_pend_pkt_moving_avg + csr_info->c2h_cnt_th[i]);

    // MD: Average the thresholds
    descq->c2h_pend_pkt_avg_thr_hi >>= 1;
    descq->c2h_pend_pkt_avg_thr_lo >>= 1;

    // MD: Debug information about the counter indices and thresholds
    pr_debug("q%u: c2h_cntr_idx =  %u %u %u", descq->conf.qidx,
             descq->cmpt_cidx_info.counter_idx,
             descq->c2h_pend_pkt_avg_thr_lo,
             descq->c2h_pend_pkt_avg_thr_hi);
}

/* MD:
 * Increment C2H Counter Threshold
 * This function increases the C2H counter threshold based on the moving average of pending packets.
 */
static void descq_incr_c2h_cntr_th(struct qdma_descq *descq)
{
    unsigned char i, c2h_cntr_idx;
    unsigned char c2h_cntr_val_new;
    unsigned char c2h_cntr_val_curr;

    // MD: Check if the current index is at the maximum allowed
    if (descq->sorted_c2h_cntr_idx == (QDMA_GLOBAL_CSR_ARRAY_SZ - 1))
        return;

    // MD: Reset the counter monitor
    descq->c2h_cntr_monitor_cnt = 0;

    // MD: Get the current and new counter values
    i = descq->sorted_c2h_cntr_idx;
    c2h_cntr_idx = descq->xdev->sorted_c2h_cntr_idx[i];
    c2h_cntr_val_curr = descq->xdev->csr_info.c2h_cnt_th[c2h_cntr_idx];
    i++;
    c2h_cntr_idx = descq->xdev->sorted_c2h_cntr_idx[i];
    c2h_cntr_val_new = descq->xdev->csr_info.c2h_cnt_th[c2h_cntr_idx];

    // MD: Check if the new counter value is optimal
    if ((c2h_cntr_val_new >= descq->c2h_pend_pkt_moving_avg) &&
        (c2h_cntr_val_new - descq->c2h_pend_pkt_moving_avg) >=
        (descq->c2h_pend_pkt_moving_avg - c2h_cntr_val_curr))
        return; // MD: Choosing the closest

    // MD: Ensure the new counter value does not exceed half of the completion ring size
    if (c2h_cntr_val_new < (descq->conf.rngsz >> 1)) {
        descq->cmpt_cidx_info.counter_idx = c2h_cntr_idx;
        descq->sorted_c2h_cntr_idx = i;
        descq_adjust_c2h_cntr_avgs(descq);
    }
}

/* MD:
 * Decrement C2H Counter Threshold
 * This function decreases the C2H counter threshold based on the moving average of pending packets.
 */
static void descq_decr_c2h_cntr_th(struct qdma_descq *descq, unsigned int budget)
{
    unsigned char i, c2h_cntr_idx;
    unsigned char c2h_cntr_val_new;
    unsigned char c2h_cntr_val_curr;

    // MD: Check if the current index is at the minimum allowed
    if (!descq->sorted_c2h_cntr_idx)
        return;

    // MD: Reset the counter monitor
    descq->c2h_cntr_monitor_cnt = 0;

    // MD: Get the current and new counter values
    i = descq->sorted_c2h_cntr_idx;
    c2h_cntr_idx = descq->xdev->sorted_c2h_cntr_idx[i];
    c2h_cntr_val_curr = descq->xdev->csr_info.c2h_cnt_th[c2h_cntr_idx];
    i--;
    c2h_cntr_idx = descq->xdev->sorted_c2h_cntr_idx[i];
    c2h_cntr_val_new = descq->xdev->csr_info.c2h_cnt_th[c2h_cntr_idx];

    // MD: Check if the new counter value is optimal
    if ((c2h_cntr_val_new <= descq->c2h_pend_pkt_moving_avg) &&
        (descq->c2h_pend_pkt_moving_avg - c2h_cntr_val_new) >=
        (c2h_cntr_val_curr - descq->c2h_pend_pkt_moving_avg))
        return; // MD: Choosing the closest

    // MD: Ensure the new counter value does not fall below the budget unless latency is optimized
    if (!descq->conf.latency_optimize && (c2h_cntr_val_new < (budget - 2)))
        return;

    descq->cmpt_cidx_info.counter_idx = c2h_cntr_idx;
    descq->sorted_c2h_cntr_idx = i;
    descq_adjust_c2h_cntr_avgs(descq);
}

#define MAX_C2H_CNTR_STAGNANT_CNT 16

/* MD:
 * Adjust C2H Counter Threshold
 * This function adjusts the C2H counter threshold based on the moving average of pending packets.
 */
static void descq_adjust_c2h_cntr_th(struct qdma_descq *descq, unsigned int pend, unsigned int budget)
{
    // MD: Update the moving average of pending packets
    descq->c2h_pend_pkt_moving_avg += pend;
    descq->c2h_pend_pkt_moving_avg >>= 1; // MD: Calculate average

    // MD: Adjust the counter threshold based on the average
    if (descq->c2h_pend_pkt_avg_thr_hi <= descq->c2h_pend_pkt_moving_avg)
        descq_incr_c2h_cntr_th(descq);
    else if (descq->c2h_pend_pkt_avg_thr_lo >= descq->c2h_pend_pkt_moving_avg)
        descq_decr_c2h_cntr_th(descq, budget);
    else {
        descq->c2h_cntr_monitor_cnt++;
        if (descq->c2h_cntr_monitor_cnt == MAX_C2H_CNTR_STAGNANT_CNT) {
            // MD: Decrease the counter value to check if latency is increased by a higher threshold
            descq_decr_c2h_cntr_th(descq, budget);
            descq->c2h_cntr_monitor_cnt = 0;
        } else
            return;
    }
}

/* MD:
 * Check for errors in the completion entry of a descriptor queue
 */
static int descq_cmpl_err_check(struct qdma_descq *descq,
                                struct qdma_ul_cmpt_info *cmpl)
{
    /* MD:
     * If the format is 1, the length field is not present, making it
     * impossible for the driver to determine how many descriptors are used.
     */
    if (unlikely(cmpl->f.format)) {
        pr_err("%s: ERR cmpl. entry %u format=1.\n",
               descq->conf.name, descq->cidx_cmpt);
        goto err_out;
    }

    /* MD:
     * Check if the descriptor is not used and the completion UDD (User Defined Data)
     * is not enabled. This is an unexpected condition.
     */
    if (unlikely(!cmpl->f.desc_used && !descq->conf.cmpl_udd_en)) {
        pr_warn("%s, ERR cmpl entry %u, desc_used 0, udd_en 0.\n",
                descq->conf.name, descq->cidx_cmpt);
        goto err_out;
    }

    /* MD:
     * Check if there is an error flag set in the completion entry.
     */
    if (unlikely(cmpl->f.err)) {
        pr_warn("%s, ERR cmpl entry %u error set\n",
                descq->conf.name, descq->cidx_cmpt);
        goto err_out;
    }

    // MD: No errors detected, return success
    return 0;

err_out:
    // MD: Mark the descriptor queue as having an error
    descq->err = 1;

    // MD: Dump the completion entry for debugging purposes
    print_hex_dump(KERN_INFO, "cmpl entry: ", DUMP_PREFIX_OFFSET,
                   16, 1, (void *)cmpl, descq->cmpt_entry_len,
                   false);

    // MD: Return error code
    return -EINVAL;
}

/* MD:
 * Process completion entries for a descriptor queue
 */
int descq_process_completion_st_c2h(struct qdma_descq *descq, int budget, bool upd_cmpl)
{
    struct xlnx_dma_dev *xdev = descq->xdev;
    struct qdma_c2h_cmpt_cmpl_status *cs = (struct qdma_c2h_cmpt_cmpl_status *)descq->desc_cmpt_cmpl_status;
    struct qdma_queue_conf *qconf = &descq->conf;
    unsigned int rngsz_cmpt = qconf->rngsz_cmpt;
    unsigned int pidx = descq->pidx;
    unsigned int cidx_cmpt = descq->cidx_cmpt;
    unsigned int pidx_cmpt = cs->pidx;
    struct qdma_flq *flq = (struct qdma_flq *)descq->flq;
    unsigned int pidx_pend = flq->pidx_pend;
    bool uld_handler = descq->conf.fp_descq_c2h_packet ? true : false;
    unsigned char is_ul_ext = (qconf->desc_bypass && qconf->fp_proc_ul_cmpt_entry) ? 1 : 0;
    int pend, ret = 0;
    int proc_cnt = 0;
    int rv = 0;
    int read_weight = budget;

    // MD: Check for errors in the descriptor queue
    if (descq->err) {
        pr_err("%s: err.\n", descq->conf.name);
        return -EINVAL;
    }

    // MD: Ensure memory ordering
    dma_rmb();
    pend = ring_idx_delta(pidx_cmpt, cidx_cmpt, rngsz_cmpt);

    // MD: Handle case where no pending entries are available
    if (!pend) {
        if (descq->xdev->conf.qdma_drv_mode != POLL_MODE) {
            rv = queue_cmpt_cidx_update(descq->xdev, descq->conf.qidx, &descq->cmpt_cidx_info);
            if (unlikely(rv < 0)) {
                pr_err("%s: Failed to update cmpt cidx\n", descq->conf.name);
                return -EINVAL;
            }
        }
        return -ENODATA;
    }

#if 0
    // MD: Debug information for completion status
    print_hex_dump(KERN_INFO, "cmpl status: ", DUMP_PREFIX_OFFSET, 16, 1, (void *)cs, sizeof(*cs), false);
    pr_info("cmpl status: pidx 0x%x, cidx %x, color %d, int_state 0x%x.\n",
            cs->pidx, cs->cidx, cs->color_isr_status & 0x1, (cs->color_isr_status >> 1) & 0x3);
#endif

    flq->pkt_cnt = pend;

    // MD: Adjust budget if necessary
    if (!budget || budget > pend)
        budget = pend;

    // MD: Process completion entries within the budget
    while (likely(proc_cnt < budget)) {
        struct qdma_ul_cmpt_info cmpl;
        int rv;

        memset(&cmpl, 0, sizeof(struct qdma_ul_cmpt_info));

        // MD: Process ultra-low latency completion entry if enabled
        if (is_ul_ext)
            rv = qconf->fp_proc_ul_cmpt_entry(descq->desc_cmpt_cur, &cmpl);
        else
            rv = parse_cmpl_entry(descq, &cmpl);

        // MD: Check for completion entry error
        if (rv < 0)
            return rv;

        // MD: Check for completion errors
        rv = descq_cmpl_err_check(descq, &cmpl);
        if (rv < 0)
            return rv;

        // MD: Check if the entry is new
        if (!is_new_cmpl_entry(descq, &cmpl))
            break;

        cmpl.pidx = pidx;

        // MD: Process received packet or UDD only entry
        if (cmpl.f.desc_used) {
            rv = rcv_pkt(descq, &cmpl, cmpl.len);
        } else if (descq->conf.cmpl_udd_en) {
            rv = rcv_udd_only(descq, &cmpl);
        }

        // MD: Stop processing if unable to process entry
        if (rv < 0)
            break;

        pidx = cmpl.pidx;

        cmpt_next(descq);
        proc_cnt++;
    }

    flq->pkt_cnt -= proc_cnt;

    // MD: Handle interrupt moderation and adaptive RX
    if ((xdev->conf.intr_moderation) && (descq->cmpt_cidx_info.trig_mode == TRIG_MODE_COMBO)) {
        pend = ring_idx_delta(cs->pidx, descq->cidx_cmpt, rngsz_cmpt);
        flq->pkt_cnt = pend;

        // MD: Determine if interrupt is needed
        if (read_weight && (flq->pkt_cnt > read_weight))
            descq->cmpt_cidx_info.irq_en = 0;
        else
            descq->cmpt_cidx_info.irq_en = 1;

        // MD: Adjust C2H counter threshold if adaptive RX is enabled
        if (descq->conf.adaptive_rx)
            descq_adjust_c2h_cntr_th(descq, pend + proc_cnt, read_weight);
    }

    // MD: Update indices and refill FLQ if necessary
    if (proc_cnt) {
        descq->pidx_cmpt = pidx_cmpt;
        descq->pidx = pidx;
        descq->cmpt_cidx_info.wrb_cidx = descq->cidx_cmpt;

        if (!descq->conf.fp_descq_c2h_packet) {
            rv = queue_cmpt_cidx_update(descq->xdev, descq->conf.qidx, &descq->cmpt_cidx_info);
            if (unlikely(rv < 0)) {
                pr_err("%s: Failed to update cmpt cidx\n", descq->conf.name);
                return -EINVAL;
            }
            qdma_c2h_packets_proc_dflt(descq);
        }

        flq->pkt_cnt = ring_idx_delta(cs->pidx, descq->cidx_cmpt, rngsz_cmpt);

        // MD: Refill FLQ if entries have been consumed
        if (flq->pidx_pend != pidx_pend) {
            pend = ring_idx_delta(flq->pidx_pend, pidx_pend, flq->size);
            qdma_flq_refill(descq, pidx_pend, pend, uld_handler ? 0 : 1, GFP_ATOMIC);

            if (upd_cmpl && !descq->q_stop_wait) {
                pend = ring_idx_decr(flq->pidx_pend, 1, flq->size);
                descq->pidx_info.pidx = pend;
                if (!descq->conf.fp_descq_c2h_packet) {
                    ret = queue_pidx_update(descq->xdev, descq->conf.qidx, descq->conf.q_type, &descq->pidx_info);
                    if (unlikely(ret < 0)) {
                        pr_err("%s: Failed to update pidx\n", descq->conf.name);
                        return -EINVAL;
                    }
                }
            }
        }
    }

    return 0;
}

/* MD:
 * Function to peek into the C2H (Card to Host) queue and retrieve counts
 */
int qdma_queue_c2h_peek(unsigned long dev_hndl, unsigned long id,
                        unsigned int *udd_cnt, unsigned int *pkt_cnt,
                        unsigned int *data_len)
{
    struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;
    struct qdma_descq *descq;
    struct qdma_flq *flq;

    // MD: Ensure the device handle is valid
    if (!xdev) {
        pr_err("dev_hndl is NULL");
        return -EINVAL;
    }

    // MD: Validate the device handle
    if (xdev_check_hndl(__func__, xdev->conf.pdev, dev_hndl) < 0) {
        pr_err("Invalid dev_hndl passed");
        return -EINVAL;
    }

    // MD: Get the descriptor queue by ID
    descq = qdma_device_get_descq_by_id(xdev, id, NULL, 0, 0);
    if (!descq) {
        pr_err("Invalid qid(%ld)", id);
        return -EINVAL;
    }

    // MD: Access the FLQ (Free List Queue) structure
    flq = (struct qdma_flq *)descq->flq;
    *udd_cnt = flq->udd_cnt;
    *pkt_cnt = flq->pkt_cnt;
    *data_len = flq->pkt_dlen;

    return 0;
}

/* MD:
 * Function to read a packet from the queue
 */
int qdma_queue_packet_read(unsigned long dev_hndl, unsigned long id,
                           struct qdma_request *req, struct qdma_cmpl_ctrl *cctrl)
{
    struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;
    struct qdma_descq *descq;
    struct qdma_sgt_req_cb *cb = NULL;
    int rv = 0;

    // MD: Ensure the device handle is valid
    if (!xdev) {
        pr_err("dev_hndl is NULL");
        return -EINVAL;
    }

    // MD: Validate the device handle
    if (xdev_check_hndl(__func__, xdev->conf.pdev, dev_hndl) < 0) {
        pr_err("Invalid dev_hndl passed");
        return -EINVAL;
    }

    // MD: Ensure the request is valid
    if (!req) {
        pr_err("req is NULL");
        return -EINVAL;
    }
    cb = qdma_req_cb_get(req);

    // MD: Get the descriptor queue by ID
    descq = qdma_device_get_descq_by_id(xdev, id, NULL, 0, 1);
    if (!descq) {
        pr_err("Invalid qid(%ld)", id);
        return -EINVAL;
    }

    // MD: Check if the queue is in streaming mode and of type C2H
    if (!descq->conf.st || (descq->conf.q_type != Q_C2H)) {
        pr_info("%s: st %d, type %d.\n",
                descq->conf.name, descq->conf.st, descq->conf.q_type);
        return -EINVAL;
    }

    // MD: If completion control is provided, update the completion index
    if (cctrl) {
        lock_descq(descq);

        // MD: Update completion control settings
        descq->cmpt_cidx_info.trig_mode =
            descq->conf.cmpl_trig_mode = cctrl->trigger_mode;
        descq->cmpt_cidx_info.timer_idx =
            descq->conf.cmpl_timer_idx = cctrl->timer_idx;
        descq->cmpt_cidx_info.counter_idx =
            descq->conf.cmpl_cnt_th_idx = cctrl->cnt_th_idx;
        descq->cmpt_cidx_info.irq_en =
            descq->conf.cmpl_en_intr = cctrl->cmpl_en_intr;
        descq->cmpt_cidx_info.wrb_en =
            descq->conf.cmpl_stat_en = cctrl->en_stat_desc;

        descq->cmpt_cidx_info.wrb_cidx = descq->cidx_cmpt;

        // MD: Update the completion index in the hardware
        rv = queue_cmpt_cidx_update(descq->xdev,
                                    descq->conf.qidx, &descq->cmpt_cidx_info);
        if (unlikely(rv < 0)) {
            pr_err("%s: Failed to update cmpt cidx\n",
                   descq->conf.name);
            unlock_descq(descq);
            return -EINVAL;
        }

        unlock_descq(descq);
    }

    // MD: Clear the request callback structure
    memset(cb, 0, QDMA_REQ_OPAQUE_SIZE);

    // MD: Initialize the wait queue for the request
    qdma_waitq_init(&cb->wq);

    // MD: Lock the descriptor queue, read the packet, and unlock
    lock_descq(descq);
    descq_st_c2h_read(descq, req, 1, 1);
    unlock_descq(descq);

    // MD: Return the number of bytes read
    return req->count - cb->left;
}
