/* MD: QDMA Netlink Driver Header - Handles communication between userspace and kernel */
/*
 * This file is part of the Xilinx DMA IP Core driver for Linux
 *
 * Copyright (c) 2017-2022, Xilinx, Inc. All rights reserved.
 * Copyright (c) 2022-2023, Advanced Micro Devices, Inc. All rights reserved.
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

/* MD: Configure print format to include module name and function name for better debugging */
#define pr_fmt(fmt)     KBUILD_MODNAME ":%s: " fmt, __func__

/* MD: System header includes for netlink functionality */
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/netlink.h>
#include <linux/genetlink.h>
#include <net/sock.h>

/* MD: QDMA specific header includes */
#include "nl.h"
#include "qdma_nl.h"
#include "libqdma/libqdma_export.h"
#include "qdma_mod.h"
#include "version.h"

/* MD: Debug macro definition for development/testing */
#ifdef DEBUG
/* MD: Function prototype for debugging netlink attributes */
static int xnl_dump_attrs(struct genl_info *info);
#else
/* MD: Empty macro when debug is disabled */
#define xnl_dump_attrs(info)
#endif

/* MD: Netlink attribute policy definitions for kernel versions >= 5.2.0 */
#if KERNEL_VERSION(5, 2, 0) <= LINUX_VERSION_CODE
/* MD: Define attribute policies for netlink message validation */
static const struct nla_policy xnl_policy[XNL_ATTR_MAX] = {
    [XNL_ATTR_GENMSG] =       { .type = NLA_NUL_STRING },
    [XNL_ATTR_DRV_INFO] =     { .type = NLA_NUL_STRING },
    /* MD: Device and queue configuration attributes */
    [XNL_ATTR_DEV_IDX] =      { .type = NLA_U32 },
    [XNL_ATTR_PCI_BUS] =      { .type = NLA_U32 },
    [XNL_ATTR_PCI_DEV] =      { .type = NLA_U32 },
    [XNL_ATTR_PCI_FUNC] =     { .type = NLA_U32 },
    [XNL_ATTR_DEV_CFG_BAR] =  { .type = NLA_U32 },
    [XNL_ATTR_DEV_USR_BAR] =  { .type = NLA_U32 },
    [XNL_ATTR_DEV_QSET_MAX] = { .type = NLA_U32 },
    [XNL_ATTR_DEV_QSET_QBASE] = { .type = NLA_U32 },
    /* MD: Queue specific attributes */
    [XNL_ATTR_QIDX] =         { .type = NLA_U32 },
    [XNL_ATTR_NUM_Q] =        { .type = NLA_U32 },
    [XNL_ATTR_QFLAG] =        { .type = NLA_U32 },
    /* MD: Debug and status attributes */
    [XNL_ATTR_CMPT_DESC_SIZE] = { .type = NLA_U32 },
    [XNL_ATTR_PIPE_GL_MAX] =   { .type = NLA_U32 },
    [XNL_ATTR_PIPE_FLOW_ID] =  { .type = NLA_U32 },
    [XNL_ATTR_PIPE_SLR_ID] =   { .type = NLA_U32 },
    [XNL_ATTR_PIPE_TDEST] =    { .type = NLA_U32 },
    [XNL_ATTR_DEV_STM_BAR] =   { .type = NLA_U32 },
    [XNL_ATTR_Q_STATE] =       { .type = NLA_U32 },
    [XNL_ATTR_ERROR] =         { .type = NLA_U32 },
    [XNL_ATTR_PING_PONG_EN] =  { .type = NLA_U32 },
    /* MD: Binary data attributes with size constraints */
    [XNL_ATTR_DEV] =          { .type = NLA_BINARY,
                                .len = QDMA_DEV_ATTR_STRUCT_SIZE },
    [XNL_ATTR_GLOBAL_CSR] =   { .type = NLA_BINARY,
                                .len = QDMA_DEV_GLOBAL_CSR_STRUCT_SIZE },
    /* MD: Error information attribute for debug builds */
#ifdef ERR_DEBUG
    [XNL_ATTR_QPARAM_ERR_INFO] = { .type = NLA_U32 },
#endif
};
