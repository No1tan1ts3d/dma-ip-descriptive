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

/* MD: Default buffer size for C2H (Card to Host) transfers */
#define QDMA_C2H_DEFAULT_BUF_SZ (4096)

/* MD: Maximum line length for debug dumps */
#define DUMP_LINE_SZ            (81)

/* MD: Maximum number of queues that can be dumped at once */
#define QDMA_Q_DUMP_MAX_QUEUES  (100)

/* MD: Size of buffer for queue dump operations */
#define QDMA_Q_DUMP_LINE_SZ     (25 * 1024)

/* MD: Size of buffer for queue list operations */
#define QDMA_Q_LIST_LINE_SZ     (200)

/* MD: Debug print for tracking buffer sizes */
static inline void debug_print_sizes(void)
{
    pr_debug("QDMA Buffer Sizes - C2H Default: %d, Queue Dump: %d, Queue List: %d\n",
             QDMA_C2H_DEFAULT_BUF_SZ, QDMA_Q_DUMP_LINE_SZ, QDMA_Q_LIST_LINE_SZ);
}

/* MD: Debug print for tracking maximum queue limits */
static inline void debug_print_queue_limits(void)
{
    pr_debug("QDMA Maximum Queue Limits - Dump Queues: %d\n", 
             QDMA_Q_DUMP_MAX_QUEUES);
}

/* Forward declaration for device listing function */
static int xnl_dev_list(struct sk_buff *skb2, struct genl_info *info);

/* RHEL (Red Hat Enterprise Linux) specific version check */
#ifdef RHEL_RELEASE_VERSION
#if RHEL_RELEASE_VERSION(8, 3) > RHEL_RELEASE_CODE

/* 
 * Netlink attribute policy definitions
 * Defines the type and validation rules for netlink message attributes
 */
static struct nla_policy xnl_policy[XNL_ATTR_MAX] = {
    /* Basic message attributes */
    [XNL_ATTR_GENMSG] =      { .type = NLA_NUL_STRING },    /* Generic message string */
    [XNL_ATTR_DRV_INFO] =    { .type = NLA_NUL_STRING },    /* Driver information string */

    /* PCI device identification attributes */
    [XNL_ATTR_DEV_IDX] =     { .type = NLA_U32 },          /* Device index */
    [XNL_ATTR_PCI_BUS] =     { .type = NLA_U32 },          /* PCI bus number */
    [XNL_ATTR_PCI_DEV] =     { .type = NLA_U32 },          /* PCI device number */
    [XNL_ATTR_PCI_FUNC] =    { .type = NLA_U32 },          /* PCI function number */
    
    /* Device BAR and queue configuration attributes */
    [XNL_ATTR_DEV_CFG_BAR] = { .type = NLA_U32 },          /* Config BAR number */
    [XNL_ATTR_DEV_USR_BAR] = { .type = NLA_U32 },          /* User BAR number */
    [XNL_ATTR_DEV_QSET_MAX] = { .type = NLA_U32 },         /* Maximum queue sets */
    [XNL_ATTR_DEV_QSET_QBASE] = { .type = NLA_U32 },       /* Queue base number */

    /* Device version and capability attributes */
    [XNL_ATTR_VERSION_INFO] = { .type = NLA_NUL_STRING },   /* Version information */
    [XNL_ATTR_DEVICE_TYPE] = { .type = NLA_NUL_STRING },    /* Device type string */
    [XNL_ATTR_IP_TYPE] =     { .type = NLA_NUL_STRING },    /* IP type information */
    [XNL_ATTR_DEV_NUMQS] =   { .type = NLA_U32 },          /* Number of queues */
    [XNL_ATTR_DEV_NUM_PFS] = { .type = NLA_U32 },          /* Number of PFs */

    /* Device feature configuration attributes */
    [XNL_ATTR_DEV_MM_CHANNEL_MAX] = { .type = NLA_U32 },    /* Max MM channels */
    [XNL_ATTR_DEV_MAILBOX_ENABLE] = { .type = NLA_U32 },    /* Mailbox enable flag */
    [XNL_ATTR_DEV_FLR_PRESENT] =    { .type = NLA_U32 },    /* FLR support flag */
    [XNL_ATTR_DEV_ST_ENABLE] =      { .type = NLA_U32 },    /* ST mode enable */
    [XNL_ATTR_DEV_MM_ENABLE] =      { .type = NLA_U32 },    /* MM mode enable */
    [XNL_ATTR_DEV_MM_CMPT_ENABLE] = { .type = NLA_U32 },    /* MM completion enable */

    /* Register access attributes */
    [XNL_ATTR_REG_BAR_NUM] = { .type = NLA_U32 },          /* Register BAR number */
    [XNL_ATTR_REG_ADDR] =    { .type = NLA_U32 },          /* Register address */
    [XNL_ATTR_REG_VAL] =     { .type = NLA_U32 },          /* Register value */

    /* Queue configuration attributes */
    [XNL_ATTR_QIDX] =         { .type = NLA_U32 },         /* Queue index */
    [XNL_ATTR_NUM_Q] =        { .type = NLA_U32 },         /* Number of queues */
    [XNL_ATTR_QFLAG] =        { .type = NLA_U32 },         /* Queue flags */
    [XNL_ATTR_CMPT_DESC_SIZE] = { .type = NLA_U32 },       /* Completion descriptor size */
    [XNL_ATTR_SW_DESC_SIZE] =   { .type = NLA_U32 },       /* Software descriptor size */
    [XNL_ATTR_QRNGSZ_IDX] =    { .type = NLA_U32 },        /* Queue ring size index */
    [XNL_ATTR_C2H_BUFSZ_IDX] = { .type = NLA_U32 },        /* C2H buffer size index */
    [XNL_ATTR_CMPT_TIMER_IDX] = { .type = NLA_U32 },       /* Completion timer index */
    [XNL_ATTR_CMPT_CNTR_IDX] = { .type = NLA_U32 },        /* Completion counter index */
    [XNL_ATTR_MM_CHANNEL] =     { .type = NLA_U32 },        /* MM channel numberXNL_ATTR_CMPT_TRIG_MODE] = { .type = NLA_U32 },       /* Completion trigger mode */
    [XNL_ATTR_CMPT_ENTRIES_CNT] = { .type = NLA_U32 },     /* Completion entries count */
    [XNL_ATTR_RANGE_START] =   { .type = NLA_U32 },        /* Range start */
    [XNL_ATTR_RANGE_END] =     { .type = NLA_U32 },        /* Range end */

    /* Interrupt and pipeline attributes */
    [XNL_ATTR_INTR_VECTOR_IDX] = { .type = NLA_U32 },      /* Interrupt vector index */
    [XNL_ATTR_PIPE_GL_MAX] =     { .type = NLA_U32 },      /* Pipeline max global */
    [XNL_ATTR_PIPE_FLOW_ID] =    { .type = NLA_U32 },      /* Pipeline flow ID */
    [XNL_ATTR_PIPE_SLR_ID] =     { .type = NLA_U32 },      /* Pipeline SLR ID */
    [XNL_ATTR_PIPE_TDEST] =      { .type = NLA_U32 },      /* Pipeline TDEST */

    /* Additional device attributes */
    [XNL_ATTR_DEV_STM_BAR] =    { .type = NLA_U32 },       /* STM BAR number */
    [XNL_ATTR_Q_STATE] =        { .type = NLA_U32 },       /* Queue state */
    [XNL_ATTR_ERROR] =          { .type = NLA_U32 },       /* Error code */
    [XNL_ATTR_PING_PONG_EN] =   { .type = NLA_U32 },       /* Ping-pong enable */

    /* Binary data attributes */
    [XNL_ATTR_DEV] = {
        .type = NLA_BINARY,
        .len = QDMA_DEV_ATTR_STRUCT_SIZE,                   /* Device attributes struct */
    },
    [XNL_ATTR_GLOBAL_CSR] = {
        .type = NLA_BINARY,
        .len = QDMA_DEV_GLOBAL_CSR_STRUCT_SIZE,            /* Global CSR struct */
    },

#ifdef ERR_DEBUG
    /* Debug attributes */
    [XNL_ATTR_QPARAM_ERR_INFO] = { .type = NLA_U32 },      /* Queue parameter error info */
#endif
};

/* 
 * Netlink attribute policies for QDMA driver
 * Only applied for kernel versions outside 5.2.0 to 5.9.0 range
 */
#endif
#else
#if ((KERNEL_VERSION(5, 2, 0) > LINUX_VERSION_CODE) || (LINUX_VERSION_CODE > \
        KERNEL_VERSION(5, 9, 0)))
static struct nla_policy xnl_policy[XNL_ATTR_MAX] = {
    /* Basic message attributes */
    [XNL_ATTR_GENMSG] =     { .type = NLA_NUL_STRING },    /* Generic message string */
    [XNL_ATTR_DRV_INFO] =   { .type = NLA_NUL_STRING },    /* Driver information string */

    /* PCI device identification attributes */
    [XNL_ATTR_DEV_IDX] =        { .type = NLA_U32 },       /* Device index */
    [XNL_ATTR_PCI_BUS] =        { .type = NLA_U32 },       /* PCI bus number */
    [XNL_ATTR_PCI_DEV] =        { .type = NLA_U32 },       /* PCI device number */
    [XNL_ATTR_PCI_FUNC] =       { .type = NLA_U32 },       /* PCI function number */
    [XNL_ATTR_DEV_CFG_BAR] =    { .type = NLA_U32 },       /* Config BAR number */
    [XNL_ATTR_DEV_USR_BAR] =    { .type = NLA_U32 },       /* User BAR number */
    [XNL_ATTR_DEV_QSET_MAX] =   { .type = NLA_U32 },       /* Maximum queue sets */
    [XNL_ATTR_DEV_QSET_QBASE] = { .type = NLA_U32 },       /* Queue base number */

    /* Device identification and capability attributes */
    [XNL_ATTR_VERSION_INFO] =   { .type = NLA_NUL_STRING }, /* Version information */
    [XNL_ATTR_DEVICE_TYPE] =    { .type = NLA_NUL_STRING }, /* Device type */
    [XNL_ATTR_IP_TYPE] =        { .type = NLA_NUL_STRING }, /* IP type */
    [XNL_ATTR_DEV_NUMQS] =      { .type = NLA_U32 },       /* Number of queues */
    [XNL_ATTR_DEV_NUM_PFS] =    { .type = NLA_U32 },       /* Number of physical functions */
    [XNL_ATTR_DEV_MM_CHANNEL_MAX] = { .type = NLA_U32 },   /* Max memory-mapped channels */
    [XNL_ATTR_DEV_MAILBOX_ENABLE] = { .type = NLA_U32 },   /* Mailbox enable flag */
    [XNL_ATTR_DEV_FLR_PRESENT] = { .type = NLA_U32 },      /* FLR support flag */
    [XNL_ATTR_DEV_ST_ENABLE] =   { .type = NLA_U32 },      /* Stream mode enable */
    [XNL_ATTR_DEV_MM_ENABLE] =   { .type = NLA_U32 },      /* Memory-mapped mode enable */
    [XNL_ATTR_DEV_MM_CMPT_ENABLE] = { .type = NLA_U32 },   /* MM completion enable */

    /* Register access attributes */
    [XNL_ATTR_REG_BAR_NUM] =    { .type = NLA_U32 },       /* Register BAR number */
    [XNL_ATTR_REG_ADDR] =       { .type = NLA_U32 },       /* Register address */
    [XNL_ATTR_REG_VAL] =        { .type = NLA_U32 },       /* Register value */

    /* CSR (Control/Status Register) attributes */
    [XNL_ATTR_CSR_INDEX] =      { .type = NLA_U32 },       /* CSR index */
    [XNL_ATTR_CSR_COUNT] =      { .type = NLA_U32 },       /* CSR count */

    /* Queue configuration attributes */
    [XNL_ATTR_QIDX] =           { .type = NLA_U32 },       /* Queue index */
    [XNL_ATTR_NUM_Q] =          { .type = NLA_U32 },       /* Number of queues */
    [XNL_ATTR_QFLAG] =          { .type = NLA_U32 },       /* Queue flags */
    [XNL_ATTR_CMPT_DESC_SIZE] = { .type = NLA_U32 },       /* Completion descriptor size */
    [XNL_ATTR_SW_DESC_SIZE] =   { .type = NLA_U32 },       /* Software descriptor size */
    [XNL_ATTR_QRNGSZ_IDX] =     { .type = NLA_U32 },       /* Queue ring size index */
    [XNL_ATTR_C2H_BUFSZ_IDX] =  { .type = NLA_U32 },       /* C2H buffer size index */
    [XNL_ATTR_CMPT_TIMER_IDX] = { .type = NLA_U32 },       /* Completion timer index */
    [XN_CMPT_CNTR_IDX] =  { .type = NLA_U32 },       /* Completion counter index */
    [XNL_ATTR_MM_CHANNEL] =     { .type = NLA_U32 },       /* MM channel number */
    [XNL_ATTR_CMPT_TRIG_MODE] = { .type = NLA_U32 },       /* Completion trigger mode */
    [XNL_ATTR_CMPT_ENTRIES_CNT] = { .type = NLA_U32 },     /* Completion entries count */

    /* Range attributes */
    [XNL_ATTR_RANGE_START] =    { .type = NLA_U32 },       /* Range start */
    [XNL_ATTR_RANGE_END] =      { .type = NLA_U32 },       /* Range end */

    /* Interrupt and pipeline attributes */
    [XNL_ATTR_INTR_VECTOR_IDX] = { .type = NLA_U32 },      /* Interrupt vector index */
    [XNL_ATTR_INTR_VECTOR_START_IDX] = { .type = NLA_U32 }, /* Start vector index */
    [XNL_ATTR_INTR_VECTOR_END_IDX] = { .type = NLA_U32 },   /* End vector index */
    [XNL_ATTR_RSP_BUF_LEN] =    { .type = NLA_U32 },       /* Response buffer length */
    [XNL_ATTR_PIPE_GL_MAX] =    { .type = NLA_U32 },       /* Max global pipeline */
    [XNL_ATTR_PIPE_FLOW_ID] =   { .type = NLA_U32 },       /* Pipeline flow ID */
    [XNL_ATTR_PIPE_SLR_ID] =    { .type = NLA_U32 },       /* Pipeline SLR ID */
    [XNL_ATTR_PIPE_TDEST] =     { .type = NLA_U32 },       /* Pipeline TDEST */

    /* Device status and configuration attributes */
    [XNL_ATTR_DEV_STM_BAR] =    { .type = NLA_U32 },       /* STM BAR number */
    [XNL_ATTR_Q_STATE] =        { .type = NLA_U32 },       /* Queue state */
    [XNL_ATTR_ERROR] =          { .type = NLA_U32 },       /* Error code */
    [XNL_ATTR_PING_PONG_EN] =   { .type = NLA_U32 },       /* Ping-pong enable */
    [XNL_ATTR_APERTURE_SZ] =    { .type = NLA_U32 },       /* Aperture size */

    /* Performance monitoring attributes */
    [XNL_ATTR_DEV_STAT_PING_PONG_LATMIN1] = { .type = NLA_U32 }, /* Min latency 1 */
    [XNL_ATTR_DEV_STAT_PING_PONG_LATMIN2] = { .type = NLA_U32 }, /* Min latency 2 */
    [XNL_ATTR_DEV_STAT_PING_PONG_LATMAX1] = { .type = NLA_U32 }, /* Max latency 1 */
    [XNL_ATTR_DEV_STAT_PING_PONG_LATMAX2] = { .type = NLA_U32 }, /* Max latency 2 */
    [XNL_ATTR_DEV_STAT_PING_PONG_LATAVG1] = { .type = NLA_U32 }, /* Avg latency 1 */
    [XNL_ATTR_DEV_STAT_PING_PONG_LATAVG2] = { .type = NLA_U32 }, /* Avg latency 2 */

    /* Binary data attributes */
    [XNL_ATTR_DEV] = {
        .type = NLA_BINARY,
        .len = QDMA_DEV_ATTR_STRUCT_SIZE,                   /* Device attributes struct */
    },
    [XNL_ATTR_GLOBAL_CSR] = {
        .type = NLA_BINARY,
        .len = QDMA_DEV_GLOBAL_CSR_STRUCT_SIZE,            /* Global CSR struct */
    },

#ifdef ERR_DEBUG
    /* Debug attributes */
    [XNL_ATTR_QPARAM_ERR_INFO] = { .type = NLA_U32 },      /* Queue parameter error info */
#endif
};

#endif
#endif

/* Forward declaration for buffer response handler with completion info */
static int xnl_respond_buffer_cmpt(struct genl_info *info, char *buf,
        int buflen, int error, long int cmpt_entries);

/* 
 * Forward declarations for QDMA netlink operations 
 * Each function handles specific device/queue management functionality
 */

/* Device management operations */
static int xnl_dev_info(struct sk_buff *, struct genl_info *);        /* Get device info */
static int xnl_dev_version_capabilities(struct sk_buff *skb2,         /* Get version/capabilities */
        struct genl_info *info);
static int xnl_dev_stat(struct sk_buff *, struct genl_info *);        /* Get device statistics */
static int xnl_dev_stat_clear(struct sk_buff *, struct genl_info *);  /* Clear device statistics */

/* Queue management operations */
static int xnl_q_list(struct sk_buff *, struct genl_info *);          /* List all queues */
static int xnl_q_add(struct sk_buff *, struct genl_info *);           /* Add new queue */
static int xnl_q_start(struct sk_buff *, struct genl_info *);         /* Start queue */
static int xnl_q_stop(struct sk_buff *, struct genl_info *);          /* Stop queue */
static int xnl_q_del(struct sk_buff *, struct genl_info *);           /* Delete queue */

/* Queue debug/dump operations */
static int xnl_q_dump(struct sk_buff *, struct genl_info *);          /* Dump queue info */
static int xnl_q_dump_desc(struct sk_buff *, struct genl_info *);     /* Dump descriptors */
static int xnl_q_dump_cmpt(struct sk_buff *, struct genl_info *);     /* Dump completions */

/* Register and configuration operations */
static int xnl_config_reg_dump(struct sk_buff *, struct genl_info *); /* Dump register config */
static int xnl_register_read(struct sk_buff *, struct genl_info *);   /* Read register */
static int xnl_register_write(struct sk_buff *, struct genl_info *);  /* Write register */

/* Queue data operations */
static int xnl_q_read_pkt(struct sk_buff *, struct genl_info *);      /* Read packet data */
static int xnl_q_read_udd(struct sk_buff *, struct genl_info *);      /* Read user defined data */
static int xnl_q_cmpt_read(struct sk_buff *, struct genl_info *);     /* Read completion data */

/* Interrupt and CSR operations */
static int xnl_intr_ring_dump(struct sk_buff *, struct genl_info *);  /* Dump interrupt ring */
static int xnl_get_global_csr(struct sk_buff *skb2,                   /* Get global CSR */
        struct genl_info *info);
static int xnl_get_queue_state(struct sk_buff *, struct genl_info *); /* Get queue state */
static int xnl_config_reg_info_dump(struct sk_buff *,                 /* Dump register info */
        struct genl_info *);

/* Debug macro for tracking function entry/exit */
#define XNL_FUNC_TRACE() \
    pr_debug("-> %s\n", __func__)

/* Debug macro for parameter validation */
#define XNL_PARAM_DEBUG(fmt, ...) \
    pr_debug("%s: " fmt "\n", __func__, ##__VA_ARGS__)

/* 
 * Function prototypes and version-specific definitions for QDMA netlink operations
 */

#ifdef TANDEM_BOOT_SUPPORTED
/*
 * @brief Enable streaming mode during tandem boot
 * @param skb2: Socket buffer containing the request
 * @param info: Generic netlink info structure
 * @return 0 on success, negative value on failure
 */
static int xnl_en_st(struct sk_buff *skb2, struct genl_info *info);
#endif

#ifdef ERR_DEBUG
/*
 * @brief Induce artificial errors for debug/testing purposes
 * @param skb2: Socket buffer containing the request
 * @param info: Generic netlink info structure
 * @return 0 on success, negative value on failure
 *
 * This function is only available when ERR_DEBUG is defined
 */
static int xnl_err_induce(struct sk_buff *skb2, struct genl_info *info);
#endif

/* 
 * Version-specific policy definitions for generic netlink operations
 * These ensure compatibility across different kernel versions
 */
#ifdef RHEL_RELEASE_VERSION
/* For Red Hat Enterprise Linux versions */
#if RHEL_RELEASE_VERSION(8, 3) > RHEL_RELEASE_CODE
#define GENL_OPS_POLICY    /* Define policy for RHEL versions before 8.3 */
#endif
#else
/* For mainline kernel versions */
#if ((KERNEL_VERSION(5, 2, 0) > LINUX_VERSION_CODE) || (LINUX_VERSION_CODE > \
		KERNEL_VERSION(5, 9, 0)))
/* Define policy for kernel versions outside 5.2.0 to 5.9.0 range */
#define GENL_OPS_POLICY
#endif
#endif

/* Add debug prints for kernel version checks */
static inline void debug_print_version_info(void)
{
    #ifdef RHEL_RELEASE_VERSION
    pr_debug("QDMA: Running on RHEL, version code: %d\n", RHEL_RELEASE_CODE);
    #else
    pr_debug("QDMA: Running on kernel version: %d.%d.%d\n",
             (LINUX_VERSION_CODE >> 16) & 0xFF,
             (LINUX_VERSION_CODE >> 8) & 0xFF,
             LINUX_VERSION_CODE & 0xFF);
    #endif

    #ifdef GENL_OPS_POLICY
    pr_debug("QDMA: Using explicit netlink policy definitions\n");
    #else
    pr_debug("QDMA: Using kernel provided netlink policy handling\n");
    #endif
}

/* 
 * Array of Generic Netlink operations supported by the QDMA driver
 * Only compiled when GENL_OPS_POLICY is defined for kernel version compatibility
 */
#ifdef GENL_OPS_POLICY
static struct genl_ops xnl_ops[] = {
    /* Device listing and capabilities */
    {
        .cmd = XNL_CMD_DEV_LIST,      /* Command to list all QDMA devices */
        .policy = xnl_policy,          /* Attribute policy for validation */
        .doit = xnl_dev_list,          /* Handler function */
    },
    {
        .cmd = XNL_CMD_DEV_CAP,       /* Query device capabilities */
        .policy = xnl_policy,
        .doit = xnl_dev_version_capabilities,
    },
    {
        .cmd = XNL_CMD_DEV_INFO,      /* Get device information */
        .policy = xnl_policy,
        .doit = xnl_dev_info,
    },
    
    /* Device statistics operations */
    {
        .cmd = XNL_CMD_DEV_STAT,      /* Get device statistics */
        .policy = xnl_policy,
        .doit = xnl_dev_stat,
    },
    {
        .cmd = XNL_CMD_DEV_STAT_CLEAR, /* Clear device statistics */
        .policy = xnl_policy,
        .doit = xnl_dev_stat_clear,
    },

    /* Queue management operations */
    {
        .cmd = XNL_CMD_Q_LIST,        /* List all queues */
        .policy = xnl_policy,
        .doit = xnl_q_list,
    },
    {
        .cmd = XNL_CMD_Q_ADD,         /* Add new queue */
        .policy = xnl_policy,
        .doit = xnl_q_add,
    },
    {
        .cmd = XNL_CMD_Q_START,       /* Start queue operation */
        .policy = xnl_policy,
        .doit = xnl_q_start,
    },
    {
        .cmd = XNL_CMD_Q_STOP,        /* Stop queue operation */
        .policy = xnl_policy,
        .doit = xnl_q_stop,
    },
    {
        .cmd = XNL_CMD_Q_DEL,         /* Delete queue */
        .policy = xnl_policy,
        .doit = xnl_q_del,
    },

    /* Queue debugging and diagnostic operations */
    {
        .cmd = XNL_CMD_Q_DUMP,        /* Dump queue contents */
        .policy = xnl_policy,
        .doit = xnl_q_dump,
    },
    {
        .cmd = XNL_CMD_Q_DESC,        /* Dump queue descriptors */
        .policy = xnl_policy,
        .doit = xnl_q_dump_desc,
    },
    {
        .cmd = XNL_CMD_Q_CMPT,        /* Dump completion queue */
        .policy = xnl_policy,
        .doit = xnl_q_dump_cmpt,
    },
    {
        .cmd = XNL_CMD_Q_UDD,         /* Read user defined data */
        .policy = xnl_policy,
        .doit = xnl_q_read_udd,
    },
    {
        .cmd = XNL_CMD_Q_RX_PKT,      /* Read received packets */
        .policy = xnl_policy,
        .doit = xnl_q_read_pkt,
    },
    {
        .cmd = XNL_CMD_Q_CMPT_READ,   /* Read completion data */
        .policy = xnl_policy,
        .doit = xnl_q_cmpt_read,
    },

    /* Register and hardware access operations */
    {
        .cmd = XNL_CMD_REG_DUMP,      /* Dump registers */
        .policy = xnl_policy,
        .doit = xnl_config_reg_dump,
    },
    {
        .cmd = XNL_CMD_REG_INFO_READ, /* Read register info */
        .policy = xnl_policy,
        .doit = xnl_config_reg_info_dump,
    },
    {
        .cmd = XNL_CMD_INTR_RING_DUMP, /* Dump interrupt ring */
        .policy = xnl_policy,
        .doit = xnl_intr_ring_dump,
    },
    {
        .cmd = XNL_CMD_REG_RD,        /* Read register value */
        .policy = xnl_policy,
        .doit = xnl_register_read,
    },
    {
        .cmd = XNL_CMD_REG_WRT,       /* Write register value */
        .policy = xnl_policy,
        .doit = xnl_register_write,
    },

    /* Global CSR and queue state operations */
    {
        .cmd = XNL_CMD_GLOBAL_CSR,    /* Get global CSR settings */
        .policy = xnl_policy,
        .doit = xnl_get_global_csr,
    },
    {
        .cmd = XNL_CMD_GET_Q_STATE,   /* Get queue state */
        .policy = xnl_policynl_get_queue_state,
    },

/* 
 * Conditional netlink command registration blocks
 * These commands are only available when specific features are enabled
 */

#ifdef TANDEM_BOOT_SUPPORTED
    /* 
     * Command registration for enabling streaming mode in tandem boot
     * This operation is only available when TANDEM_BOOT_SUPPORTED is defined
     */
    {
        .cmd = XNL_CMD_EN_ST,         /* Command ID for enabling streaming mode */
        .policy = xnl_policy,         /* Policy rules for command attributes */
        .doit = xnl_en_st,           /* Handler function for streaming enable */
    },
#endif

#ifdef ERR_DEBUG
    /*
     * Command registration for error injection testing
     * This operation is only available when ERR_DEBUG is defined
     * Used for debugging and testing error handling paths
     */
    {
        .cmd = XNL_CMD_Q_ERR_INDUCE,  /* Command ID for error injection */
        .policy = xnl_policy,         /* Policy rules for command attributes */
        .doit = xnl_err_induce,       /* Handler function for error injection */
    }
#endif
};

/* Debug prints for tracking command registration */
static inline void debug_print_cmd_registration(void)
{
    #ifdef TANDEM_BOOT_SUPPORTED
        pr_debug("QDMA: Registered tandem boot streaming enable command\n");
    #endif
    
    #ifdef ERR_DEBUG
        pr_debug("QDMA: Registered error injection command for debugging\n");
    #endif
}
#else

/* Debug macro for tracking netlink commands */
#define XNL_DEBUG_CMD(cmd) \
    pr_debug("QDMA: Processing netlink command %s (#%d)\n", \
             #cmd, cmd)

/* Example usage in a handler function:
static int xnl_dev_list(struct sk_buff *skb2, struct genl_info *info)
{_DEBUG_CMD(XNL_CMD_DEV_LIST);
    // Rest of the function implementation
}
*/

/*
 * Array of Generic Netlink operations supported by the QDMA driver
 * Each operation maps a command ID to its handler function
 * Note: This version does not use the policy attribute as it's defined
 * outside of GENL_OPS_POLICY
 */
static struct genl_ops xnl_ops[] = {
    /* Device management operations */
    {
        .cmd = XNL_CMD_DEV_LIST,           /* List all QDMA devices */
        .doit = xnl_dev_list,              /* Handler for device listing */
    },
    {
        .cmd = XNL_CMD_DEV_CAP,            /* Query device capabilities */
        .doit = xnl_dev_version_capabilities, /* Handler for capability info */
    },
    {
        .cmd = XNL_CMD_DEV_INFO,           /* Get device information */
        .doit = xnl_dev_info,              /* Handler for device info */
    },
    {
        .cmd = XNL_CMD_DEV_STAT,           /* Get device statistics */
        .doit = xnl_dev_stat,              /* Handler for device stats */
    },
    {
        .cmd = XNL_CMD_DEV_STAT_CLEAR,     /* Clear device statistics */
        .doit = xnl_dev_stat_clear,        /* Handler for clearing stats */
    },

    /* Queue management operations */
    {
        .cmd = XNL_CMD_Q_LIST,             /* List all queues */
        .doit = xnl_q_list,                /* Handler for queue listing */
    },
    {
        .cmd = XNL_CMD_Q_ADD,              /* Add new queue */
        .doit = xnl_q_add,                 /* Handler for queue addition */
    },
    {
        .cmd = XNL_CMD_Q_START,            /* Start queue */
        .doit = xnl_q_start,               /* Handler for starting queue */
    },
    {
        .cmd = XNL_CMD_Q_STOP,             /* Stop queue */
        .doit = xnl_q_stop,                /* Handler for stopping queue */
    },
    {
        .cmd = XNL_CMD_Q_DEL,              /* Delete queue */
        .doit = xnl_q_del,                 /* Handler for queue deletion */
    },
    {
        .cmd = XNL_CMD_Q_DUMP,             /* Dump queue data */
        .doit = xnl_q_dump,                /* Handler for queue dump */
    },
    {
        .cmd = XNL_CMD_Q_DESC,             /* Dump queue descriptors */
        .doit = xnl_q_dump_desc,           /* Handler for descriptor dump */
    },

    /* Register and Configuration operations */
    {
        .cmd = XNL_CMD_REG_DUMP,           /* Dump registers */
        .doit = xnl_config_reg_dump,       /* Handler for register dump */
    },
    {
        .cmd = XNL_CMD_REG_INFO_READ,      /* Read register info */
        .doit = xnl_config_reg_info_dump,  /* Handler for register info */
    },

    /* Completion and Data handling operations */
    {
        .cmd = XNL_CMD_Q_CMPT,             /* Queue completion */
        .doit = xnl_q_dump_cmpt,           /* Handler for completion dump */
    },
    {
        .cmd = XNL_CMD_Q_UDD,              /* User defined data */
        .doit = xnl_q_read_udd,            /* Handler for UDD reading */
    },
    {
        .cmd = XNL_CMD_Q_RX_PKT,           /* Receive packet */
        .doit = xnl_q_read_pkt,            /* Handler for packet reading */
    },
    {
        .cmd = XNL_CMD_Q_CMPT_READ,        /* Read completion data */
        .doit = xnl_q_cmpt_read,           /* Handler for completion reading */
    },

    /* System and Debug operations */
    {
        .cmd = XNL_CMD_INTR_RING_DUMP,     /* Dump interrupt ring */
        .doit = xnl_intr_ring_dump,        /* Handler for interrupt ring dump */
    },
    {
        .cmd = XNL_CMD_REG_RD,             /* Read register */
        .doit = xnl_register_read,         /* Handler for register reading */
    },
    {
        .cmd = XNL_CMD_REG_WRT,            /* Write register */
        .doit = xnl_register_write,        /* Handler for register writing */
    },
    {
        .cmd = XNL_CMD_GLOBAL_CSR,         /* Global CSR access */
        .doit = xnl_get_global_csr,        /* Handler for global CSR access */
    },
    {
        .cmd = XNL_CMD_GET_Q_STATE,        /* Get queue state */
        .doit = xnl_get_queue_state,       /* Handler for queue state query */
    },

// Define debug macros for function tracing and parameter validation
#ifdef DEBUG
#define TRACE_FUNC() pr_debug("Entering %s\n", __func__)
#define VALIDATE_PARAM(param, condition, err_msg) \
    if (!(condition)) { \
        pr_err("Validation failed for %s: %s\n", #param, err_msg); \
        return -EINVAL; \
    }
#else
#define TRACE_FUNC()
#define VALIDATE_PARAM(param, condition, err_msg)
#endif

// Comprehensive comment block explaining the purpose of each function group
/*
 * Function Group: Command Handlers
 * These functions handle specific commands received via netlink.
 * Each function is responsible for executing a particular operation
 * related to the QDMA device, such as enabling streaming mode or
 * inducing errors for debugging purposes.
 */

// Command Handlers
#ifdef TANDEM_BOOT_SUPPORTED
/**
 * @brief Enable streaming mode during tandem boot.
 * This function is invoked when the XNL_CMD_EN_ST command is received.
 * It configures the device to enable streaming mode.
 */
{
    .cmd = XNL_CMD_EN_ST,
    .doit = xnl_en_st,
},
#endif

#ifdef ERR_DEBUG
/**
 * @brief Induce artificial errors for debugging/testing purposes.
 * This function is invoked when the XNL_CMD_Q_ERR_INDUCE command is received.
 * It sets up conditions to simulate errors in the queue for testing.
 */
{
    .cmd = XNL_CMD_Q_ERR_INDUCE,
    .doit = xnl_err_induce,
}
#endif

// Define the generic netlink family
static struct genl_family xnl_family = {
#ifdef GENL_ID_GENERATE
    .id = GENL_ID_GENERATE,
#endif
    .hdrsize = 0,
#ifdef __QDMA_VF__
    .name = XNL_NAME_VF,
#else
    .name = XNL_NAME_PF,
#endif
#ifndef __GENL_REG_FAMILY_OPS_FUNC__
    .ops = xnl_ops,
    .n_ops = ARRAY_SIZE(xnl_ops),
#endif
    .maxattr = XNL_ATTR_MAX - 1,
};

// Define debug macros for function tracing and parameter validation
#ifdef DEBUG
#define TRACE_FUNC() pr_debug("Entering %s\n", __func__)
#define VALIDATE_PARAM(param, condition, err_msg) \
    if (!(condition)) { \
        pr_err("Validation failed for %s: %s\n", #param, err_msg); \
        return -EINVAL; \
    }
#else
#define TRACE_FUNC()
#define VALIDATE_PARAM(param, condition, err_msg)
#endif

/*
 * Allocates and initializes a new netlink message buffer
 * 
 * @param op       The netlink operation type
 * @param min_sz   Minimum size required for the message
 * @param hdr      Pointer to store the message header
 * @param info     Generic netlink info structure
 * 
 * @return Pointer to sk_buff on success, NULL on failure
 */
static struct sk_buff *xnl_msg_alloc(enum xnl_op_t op, int min_sz,
                void **hdr, struct genl_info *info)
{
    struct sk_buff *skb;
    void *p;
    /* Calculate buffer size - use GOODSIZE if min_sz is too small */
    unsigned long sz = min_sz < NLMSG_GOODSIZE ? NLMSG_GOODSIZE : min_sz;

    /* Debug print for allocation request */
    pr_debug("QDMA: Allocating netlink message buffer of size %lu bytes\n", sz);

    /* Allocate new generic netlink message */
    skb = genlmsg_new(sz, GFP_KERNEL);
    if (!skb) {
        pr_err("QDMA: failed to allocate skb of size %lu bytes\n", sz);
        return NULL;
    }

    /* Initialize message header */
    p = genlmsg_put(skb, 0, info->snd_seq + 1, &xnl_family, 0, op);
    if (!p) {
        pr_err("QDMA: skb too small for message header\n");
        nlmsg_free(skb);
        return NULL;
    }

    pr_debug("QDMA: Successfully allocated netlink message buffer\n");

    *hdr = p;
    return skb;
}

/*
 * Adds a string attribute to a netlink message
 * 
 * @param skb   Socket buffer containing the message
 * @param type  Attribute type
 * @param s     String to add as attribute
 * 
 * @return 0 on success, -EINVAL on failure
 */
static inline int xnl_msg_add_attr_str(struct sk_buff *skb,
                    enum xnl_attr_t type, char *s)
{
    int rv;

    pr_debug("QDMA: Adding string attribute type %d: %s\n", type, s);

    /* Add string as netlink attribute */
    rv = nla_put_string(skb, type, s);
    if (rv != 0) {
        pr_err("QDMA: Failed to add string attribute, error %d\n", rv);
        return -EINVAL;
    }

    pr_debug("QDMA: Successfully added string attribute\n");
    return 0;
}

/*
 * Adds a data blob attribute to a netlink message
 * 
 * @param skb   Socket buffer containing the message
 * @param type  Attribute type
 * @param s     Pointer to data to add
 * @param len   Length of data in bytes
 * 
 * @return 0 on success, -EINVAL on failure
 */
static inline int xnl_msg_add_attr_data(struct sk_buff *skb,
        enum xnl_attr_t type, void *s, unsigned int len)
{
    int rv;

    pr_debug("QDMA: Adding data attribute type %d, length %u bytes\n", 
             type, len);

    /* Add data as netlink attribute */
    rv = nla_put(skb, type, len, s);
    if (rv != 0) {
        pr_err("QDMA: Failed to add data attribute, error %d\n", rv);
        return -EINVAL;
    }

    pr_debug("QDMA: Successfully added data attribute\n");
    return 0;
}

/**
 * @brief Add an unsigned 32-bit attribute to a netlink message
 *
 * @param skb Pointer to the socket buffer containing the message
 * @param type The attribute type to add
 * @param v The 32-bit value to add as an attribute
 * @return 0 on success, -EINVAL on failure
 */
static inline int xnl_msg_add_attr_uint(struct sk_buff *skb,
                    enum xnl_attr_t type, u32 v)
{
    int rv;

    /* Debug print before adding attribute */
    pr_debug("QDMA: Adding u32 attribute type %d, value %u\n", type, v);

    /* Add the attribute to the netlink message */
    rv = nla_put_u32(skb, type, v);
    if (rv != 0) {
        pr_err("QDMA: Failed to add u32 attribute type %d, error %d\n", 
               type, rv);
        return -EINVAL;
    }

    pr_debug("QDMA: Successfully added u32 attribute\n");
    return 0;
}

/*
 * @brief Send a netlink message to a specific port
 *
 * This function finalizes and sends a netlink message to a userspace recipient.
 * It handles the message completion and unicast transmission.
 *
 * @param skb_tx Socket buffer containing the message to send
 * @param hdr Pointer to the message header
 * @param info Generic netlink info structure containing destination info
 * @return 0 on success, error code on failure
 */
static inline int xnl_msg_send(struct sk_buff *skb_tx, void *hdr,
                struct genl_info *info)
{
    int rv;

    /* Debug print before finalizing message */
    pr_debug("QDMA: Finalizing netlink message for port %d\n", 
             info->snd_portid);

    /* Finalize the message by updating length fields */
    genlmsg_end(skb_tx, hdr);

    /* Send the message to the specified port */
    pr_debug("QDMA: Sending unicast message to port %d\n", 
             info->snd_portid);
    rv = genlmsg_unicast(genl_info_net(info), skb_tx, info->snd_portid);
    if (rv) {
        pr_err("QDMA: Failed to send message to port %d, error %d\n",
               info->snd_portid, rv);
        return rv;
    }

    pr_debug("QDMA: Successfully sent message to port %d\n", 
             info->snd_portid);
    return 0;
}

/*
 * Debug function to dump netlink attributes
 * Only compiled when DEBUG is defined
 * 
 * @param info: Generic netlink info structure containing message attributes
 * @return: 0 on success
 */
#ifdef DEBUG
static int xnl_dump_attrs(struct genl_info *info)
{
    int i;

    /* Log sequence number and port ID for message tracking */
    pr_info("snd_seq 0x%x, portid 0x%x.\n",
        info->snd_seq, info->snd_portid);

    /* Disabled debug dump of raw netlink and generic netlink headers */
#if 0
    print_hex_dump_bytes("nlhdr", DUMP_PREFIX_OFFSET, info->nlhdr,
            sizeof(struct nlmsghdr));
    pr_info("\n"); {
    print_hex_dump_bytes("genlhdr", DUMP_PREFIX_OFFSET, info->genlhdr,
            sizeof(struct genlmsghdr));
    pr_info("\n");
#endif

    /* Log netlink header details for debugging */
    pr_info("nlhdr: len %u, type %u, flags 0x%x, seq 0x%x, pid %u.\n",
        info->nlhdr->nlmsg_len,
        info->nlhdr->nlmsg_type, 
        info->nlhdr->nlmsg_flags,
        info->nlhdr->nlmsg_seq,
        info->nlhdr->nlmsg_pid);

    /* Log generic netlink header details */
    pr_info("genlhdr: cmd 0x%x %s, version %u, reserved 0x%x.\n",
        info->genlhdr->cmd, xnl_op_str[info->genlhdr->cmd],
        info->genlhdr->version,
        info->genlhdr->reserved);

    /* Iterate through all possible attributes */
    for (i = 0; i < XNL_ATTR_MAX; i++) {
        struct nlattr *na = info->attrs[i];

        if (na) {
            /* Handle kernel version specific attribute policy checking */
#if ((KERNEL_VERSION(5, 2, 0) > LINUX_VERSION_CODE) || (LINUX_VERSION_CODE > \
        KERNEL_VERSION(5, 9, 0)))
            if (xnl_policy[i].type == NLA_NUL_STRING) {
#else
            if (1) {
#endif
                /* Handle string attributes */
                char *s = (char *)nla_data(na);

                if (s)
                    pr_info("attr %d, %s, str %s.\n",
                        i, xnl_attr_str[i], s);
                else
                    pr_info("attr %d, %s, str NULL.\n",
                        i, xnl_attr_str[i]);
            } else {
                /* Handle 32-bit unsigned integer attributes */
                u32 v = nla_get_u32(na);
                pr_info("attr %s, u32 0x%x.\n",
                    xnl_attr_str[i], v);
            }
        }
    }

    return 0;
}
#else
#define xnl_dump_attrs(info)  /* Empty macro when DEBUG not defined */
#endif

/*
 * Respond to netlink request with completion buffer
 * Sends buffer data along with error code and completion entry count
 *
 * @param info: Generic netlink info structure
 * @param buf: Response buffer to send
 * @param buflen: Length of response buffer
 * @param error: Error code to send
 * @param cmpt_entries: Number of completion entries
 * @return: 0 on success, negative error code on failure
 */
static int xnl_respond_buffer_cmpt(struct genl_info *info, char *buf,
        int buflen, int error, long int cmpt_entries)
{
    struct sk_buff *skb;
    void *hdr;
    int rv;

    /* Allocate new socket buffer for response */
    pr_debug("Allocating response message for cmd %d\n", info->genlhdr->cmd);
    skb = xnl_msg_alloc(info->genlhdr->cmd, buflen, &hdr, info);
    if (!skb) {
        pr_err("Failed to allocate socket buffer\n");
        return -ENOMEM;
    }

    /* Add the message buffer as string attribute */
    pr_debug("Adding message buffer attribute\n");
    rv = xnl_msg_add_attr_str(skb, XNL_ATTR_GENMSG, buf);
    if (rv != 0) {
        pr_err("xnl_msg_add_attr_str() failed: %d\n", rv);
        nlmsg_free(skb);
        return rv;
    }

    /* Add error code as attribute */
    pr_debug("Adding error code attribute: %d\n", error);
    rv = xnl_msg_add_attr_uint(skb, XNL_ATTR_ERROR, error);
    if (rv != 0) {
        pr_err("xnl_msg_add_attr_uint() failed: %d\n", rv);
        nlmsg_free(skb);
        return rv;
    }

    /* Add completion entry count as attribute */
    pr_debug("Adding completion entry count: %ld\n", cmpt_entries);
    rv = xnl_msg_add_attr_uint(skb, XNL_ATTR_CMPT_ENTRIES_CNT,
            cmpt_entries);
    if (rv != 0) {
        pr_err("xnl_msg_add_attr_uint() failed: %d\n", rv);
        nlmsg_free(skb);
        return rv;
    }

    /* Send the message */
    pr_debug("Sending response message\n");
    rv = xnl_msg_send(skb, hdr, info);

    return rv;
}

/*
 * @brief Sends a response buffer through netlink with error status
 * This function creates and sends a netlink message containing a string buffer and error code
 *  
 * @param info Pointer to the generic netlink info structure
 * @param buf Buffer containing the response message
 * @param buflen Length of the response buffer
 * @param error Error code to be included in response
 * @return 0 on success, negative error code on failure
 */
int xnl_respond_buffer(struct genl_info *info, char *buf, int buflen, int error)
{
    struct sk_buff *skb;
    void *hdr;
    int rv;

    /* Debug print for tracking response parameters */
    pr_debug("QDMA: Sending response - buflen=%d, error=%d\n", buflen, error);

    /* Allocate a new netlink message */
    skb = xnl_msg_alloc(info->genlhdr->cmd, buflen, &hdr, info);
    if (!skb) {
        pr_err("QDMA: Failed to allocate skb for response\n");
        return -ENOMEM;
    }

    /* Add the message string as an attribute */
    rv = xnl_msg_add_attr_str(skb, XNL_ATTR_GENMSG, buf);
    if (rv != 0) {
        pr_err("QDMA: Failed to add message attribute: %d\n", rv);
        nlmsg_free(skb);
        return rv;
    }

    /* Add the error code as an attribute */
    rv = xnl_msg_add_attr_uint(skb, XNL_ATTR_ERROR, error);
    if (rv != 0) {
        pr_err("QDMA: Failed to add error attribute: %d\n", rv);
        nlmsg_free(skb);
        return rv;
    }

    /* Send the netlink message */
    rv = xnl_msg_send(skb, hdr, info);
    pr_debug("QDMA: Response sent with result: %d\n", rv);

    return rv;
}

/*
 * @brief Sends raw data through netlink
 * This function creates and sends a netlink message containing binary/raw data
 *
 * @param info Pointer to the generic netlink info structure
 * @param buf Buffer containing the data to send
 * @param buflen Length of the data buffer
 * @return 0 on success, negative error code on failure
 */
static int xnl_respond_data(struct genl_info *info, void *buf, int buflen)
{
    struct sk_buff *skb;
    void *hdr;
    int rv;

    /* Debug print for tracking data response */
    pr_debug("QDMA: Sending data response - buflen=%d\n", buflen);

    /* Allocate a new netlink message */
    skb = xnl_msg_alloc(info->genlhdr->cmd, buflen, &hdr, info);
    if (!skb) {
        pr_err("QDMA: Failed to allocate skb for data response\n");
        return -ENOMEM;
    }

    /* Add the data as a binary attribute */
    rv = xnl_msg_add_attr_data(skb, XNL_ATTR_GLOBAL_CSR, buf, buflen);
    if (rv != 0) {
        pr_err("QDMA: Failed to add data attribute: %d\n", rv);
        nlmsg_free(skb);
        return rv;
    }

    /* Send the netlink message */
    rv = xnl_msg_send(skb, hdr, info);
    pr_debug("QDMA: Data response sent with result: %d\n", rv);

    return rv;
}

/*
 * @brief Allocates memory for netlink message buffer with error handling
 * 
 * This function allocates kernel memory for netlink message buffers and includes
 * error handling with user notification via netlink.
 *
 * @param l Size of memory to allocate in bytes
 * @param info Netlink message info structure for error response
 * @return Pointer to allocated memory buffer, or NULL on failure
 */
static char *xnl_mem_alloc(int l, struct genl_info *info)
{
    char ebuf[XNL_ERR_BUFLEN];  /* Error message buffer */
    char *buf;                   /* Allocated memory buffer */
    int rv;                      /* Return value for error handling */

    /* Debug print for tracking memory allocation requests */
    pr_debug("QDMA: Attempting to allocate %d bytes\n", l);

    /* Attempt to allocate memory */
    buf = kmalloc(l, GFP_KERNEL);
    if (buf) {
        /* Zero out the allocated memory for safety */
        memset(buf, 0, l);
        pr_debug("QDMA: Successfully allocated %d bytes at %p\n", l, buf);
        return buf;
    }

    /* Memory allocation failed - handle error */
    pr_err("QDMA: Memory allocation failed - OOM %d bytes\n", l);

    /* Prepare error message for user */
    rv = snprintf(ebuf, XNL_ERR_BUFLEN, "ERR! xnl OOM %d.\n", l);

    /* Send error message back to user space via netlink */
    xnl_respond_buffer(info, ebuf, XNL_ERR_BUFLEN, rv);

    return NULL;
}

/*
 * @brief Validates and retrieves PCI device context from netlink message
 * 
 * This function checks the incoming netlink message for required device index
 * attribute and retrieves the corresponding PCI device context.
 *
 * @param info Netlink message info structure containing device parameters
 * @return Pointer to PCI device context, or NULL if validation fails
 */
static struct xlnx_pci_dev *xnl_rcv_check_xpdev(struct genl_info *info)
{
    u32 idx;                        /* Device index from netlink message */
    struct xlnx_pci_dev *xpdev;     /* PCI device context pointer */
    char err[XNL_ERR_BUFLEN];       /* Error message buffer */
    int rv = 0;                     /* Return value for error handling */

    /* Validate input parameter */
    if (info == NULL) {
        pr_err("QDMA: Null info pointer received\n");
        return NULL;
    }

    /* Debug print for tracking message processing */
    pr_debug("QDMA: Processing netlink message cmd=%d\n", info->genlhdr->cmd);

    /* Check for required device index attribute */
    if (!info->attrs[XNL_ATTR_DEV_IDX]) {
        /* Prepare error message for missing attribute */
        snprintf(err, sizeof(err),
            "command %s missing attribute XNL_ATTR_DEV_IDX",
            xnl_op_str[info->genlhdr->cmd]);
        rv = -EINVAL;
        pr_err("QDMA: %s\n", err);
        goto respond_error;
    }

    /* Extract device index from netlink attributes */
    idx = nla_get_u32(info->attrs[XNL_ATTR_DEV_IDX]);
    pr_debug("QDMA: Looking up device with index %u\n", idx);

    /* Look up PCI device context using index */
    xpdev = xpdev_find_by_idx(idx, err, sizeof(err));
    if (!xpdev) {
        rv = -EINVAL;
        pr_err("QDMA: Device lookup failed for idx=%u\n", idx);
        goto respond_error;
    }

    pr_debug("QDMA: Successfully found device idx=%u\n", idx);
    return xpdev;

respond_error:
    /* Send error message back to user space via netlink */
    xnl_respond_buffer(info, err, strlen(err), rv);
    return NULL;
}

/*
 * @brief Checks the validity of a queue index and retrieves the associated queue data
 * 
 * This function attempts to retrieve the queue data for a given queue index (qidx)
 * from the specified PCI device. If the queue data is not found, it logs an error
 * and sends a response back to the user space indicating the invalid queue index.
 *
 * @param info Pointer to the generic netlink info structure containing attributes
 * @param xpdev Pointer to the Xilinx PCI device structure
 * @param qconf Pointer to the queue configuration structure
 * @param buf Buffer for storing error messages
 * @param buflen Length of the buffer
 * @return Pointer to the queue data structure if successful, NULL otherwise
 */
static struct xlnx_qdata *xnl_rcv_check_qidx(struct genl_info *info,
                                             struct xlnx_pci_dev *xpdev,
                                             struct qdma_queue_conf *qconf, 
                                             char *buf, int buflen)
{
    char ebuf[XNL_ERR_BUFLEN];  // Buffer for error messages
    struct xlnx_qdata *qdata;

    // Attempt to retrieve the queue data for the specified queue index
    qdata = xpdev_queue_get(xpdev, qconf->qidx, qconf->q_type, 1, ebuf, XNL_ERR_BUFLEN);

    // Check if the queue data retrieval was unsuccessful
    if (!qdata) {
        // Log an error message indicating the invalid queue index
        snprintf(ebuf, XNL_ERR_BUFLEN, "ERR! qidx %u invalid.\n", qconf->qidx);
        pr_debug("QDMA: %s", ebuf);  // Debug print for tracking errors

        // Send an error response back to user space
        xnl_respond_buffer(info, ebuf, XNL_ERR_BUFLEN, 0);
    }

    return qdata;  // Return the queue data or NULL if not found
}

/**
 * @brief Checks for the presence of a specific attribute in the netlink message
 * 
 * This function verifies if a given attribute is present in the netlink message.
 * If the attribute is missing, it logs a warning and optionally writes an error
 * message to the provided buffer.
 *
 * @param xnl_attr The attribute type to check
 * @param info Pointer to the generic netlink info structure containing attributes
 * @param qidx Queue index for error reporting
 * @param buf Buffer for storing error messages
 * @param buflen Length of the buffer
 * @return 0 if the attribute is present, -1 if missing
 */
static int xnl_chk_attr(enum xnl_attr_t xnl_attr, struct genl_info *info,
                        unsigned short qidx, char *buf, int buflen)
{
    int rv = 0;  // Return value indicating success or failure

    // Check if the specified attribute is missing
    if (!info->attrs[xnl_attr]) {
        // If a buffer is provided, write an error message to it
        if (buf) {
            rv += snprintf(buf, buflen, "Missing attribute %s for qidx = %u\n",
                           xnl_attr_str[xnl_attr], qidx);
            pr_debug("QDMA: Missing attribute %s for qidx = %u\n", 
                     xnl_attr_str[xnl_attr], qidx);  // Debug print for missing attribute
        }
        rv = -1;  // Set return value to indicate failure
    }

    return rv;  // Return success or failure
}


static void xnl_extract_extra_config_attr(struct genl_info *info,
					struct qdma_queue_conf *qconf)
{
	u32 f = nla_get_u32(info->attrs[XNL_ATTR_QFLAG]);

	/* Extract flags and assign values to configuration settings */
	qconf->desc_bypass = (f & XNL_F_DESC_BYPASS_EN) ? 1 : 0;
	qconf->pfetch_bypass = (f & XNL_F_PFETCH_BYPASS_EN) ? 1 : 0;
	qconf->pfetch_en = (f & XNL_F_PFETCH_EN) ? 1 : 0;
	qconf->wb_status_en = (f & XNL_F_CMPL_STATUS_EN) ? 1 : 0;
	qconf->cmpl_status_acc_en = (f & XNL_F_CMPL_STATUS_ACC_EN) ? 1 : 0;
	qconf->cmpl_status_pend_chk = (f & XNL_F_CMPL_STATUS_PEND_CHK) ? 1 : 0;
	qconf->fetch_credit = (f & XNL_F_FETCH_CREDIT) ? 1 : 0;
	qconf->cmpl_stat_en = (f & XNL_F_CMPL_STATUS_DESC_EN) ? 1 : 0;
	qconf->cmpl_en_intr = (f & XNL_F_C2H_CMPL_INTR_EN) ? 1 : 0;
	qconf->cmpl_udd_en = (f & XNL_F_CMPL_UDD_EN) ? 1 : 0;
	qconf->cmpl_ovf_chk_dis = (f & XNL_F_CMPT_OVF_CHK_DIS) ? 1 : 0;
	

	/* Additional settings specific to completion queue type */
	if (qconf->q_type == Q_CMPT)
		qconf->cmpl_udd_en = 1;

	/* Extract additional configuration attributes based on presence in message */
	if (xnl_chk_attr(XNL_ATTR_QRNGSZ_IDX, info, qconf->qidx, NULL, 0) == 0) {
		// Debug print for tracking attribute extraction
		pr_debug("Extracted QRNGSZ_IDX: %d\n", nla_get_u32(info->attrs[XNL_ATTR_QRNGSZ_IDX]));
		qconf->desc_rng_sz_idx = qconf->cmpl_rng_sz_idx =
				nla_get_u32(info->attrs[XNL_ATTR_QRNGSZ_IDX]);
	}
	if (xnl_chk_attr(XNL_ATTR_C2H_BUFSZ_IDX, info,
			qconf->qidx, NULL, 0) == 0) {
		// Debug print for tracking attribute extraction
		pr_debug("Extracted C2H_BUFSZ_IDX: %d\n", nla_get_u32(info->attrs[XNL_ATTR_C2H_BUFSZ_IDX]));
		qconf->c2h_buf_sz_idx =
			nla_get_u32(info->attrs[XNL_ATTR_C2H_BUFSZ_IDX]);
	}
	if (xnl_chk_attr(XNL_ATTR_CMPT_TIMER_IDX, info,
			qconf->qidx, NULL, 0) == 0) {
		// Debug print for tracking attribute extraction
		pr_debug("Extracted CMPT_TIMER_IDX: %d\n", nla_get_u32(info->attrs[XNL_ATTR_CMPT_TIMER_IDX]));
		qconf->cmpl_timer_idx =
			nla_get_u32(info->attrs[XNL_ATTR_CMPT_TIMER_IDX]);
	}
	if (xnl_chk_attr(XNL_ATTR_CMPT_CNTR_IDX, info,
			qconf->qidx, NULL, 0) == 0) {
		// Debug print for tracking attribute extraction
		pr_debug("Extracted CMPT_CNTR_IDX: %d\n", nla_get_u32(info->attrs[XNL_ATTR_CMPT_CNTR_IDX]));
		qconf->cmpl_cnt_th_idx =
			nla_get_u32(info->attrs[XNL_ATTR_CMPT_CNTR_IDX]);
	}
	if (xnl_chk_attr(XNL_ATTR_MM_CHANNEL, info, qconf->qidx, NULL, 0) == 0) {
		// Debug print for tracking attribute extraction
		pr_debug("Extracted MM_CHANNEL: %d\n", nla_get_u32(info->attrs[XNL_ATTR_MM_CHANNEL]));
		qconf->mm_channel =
			nla_get_u32(info->attrs[XNL_ATTR_MM_CHANNEL]);
	}
	if (xnl_chk_attr(XNL_ATTR_CMPT_DESC_SIZE,
				info, qconf->qidx, NULL, 0) == 0) {
		// Debug print for tracking attribute extraction
		pr_debug("Extracted CMPT_DESC_SIZE: %d\n", nla_get_u32(info->attrs[XNL_ATTR_CMPT_DESC_SIZE]));
		qconf->cmpl_desc_sz =
			nla_get_u32(info->attrs[XNL_ATTR_CMPT_DESC_SIZE]);
	}
	if (xnl_chk_attr(XNL_ATTR_SW_DESC_SIZE,
				info, qconf->qidx, NULL, 0) == 0) {
		// Debug print for tracking attribute extraction
		pr_debug("Extracted SW_DESC_SIZE: %d\n", nla_get_u32(info->attrs[XNL_ATTR_SW_DESC_SIZE]));
		qconf->sw_desc_sz =
			nla_get_u32(info->attrs[XNL_ATTR_SW_DESC_SIZE]);
	}
	if (xnl_chk_attr(XNL_ATTR_PING_PONG_EN,
					 info, qconf->qidx, NULL, 0) == 0) {
		// Debug print for tracking attribute extraction
		pr_debug("Extracted PING_PONG_EN: %d\n", nla_get_u32(info->attrs[XNL_ATTR_PING_PONG_EN]));
		qconf->ping_pong_en = 1;
	}
	if (xnl_chk_attr(XNL_ATTR_APERTURE_SZ,
					 info, qconf->qidx, NULL, 0) == 0) {
		// Debug print for tracking attribute extraction
		pr_debug("Extracted APERTURE_SZ: %d\n", nla_get_u32(info->attrs[XNL_ATTR_APERTURE_SZ]));
		qconf->aperture_size =
			nla_get_u32(info->attrs[XNL_ATTR_APERTURE_SZ]);
	}
	if (xnl_chk_attr(XNL_ATTR_CMPT_TRIG_MODE, info,
				qconf->qidx, NULL, 0) == 0) {
		// Debug print for tracking attribute extraction
		pr_debug("Extracted CMPT_TRIG_MODE: %d\n", nla_get_u32(info->attrs[XNL_ATTR_CMPT_TRIG_MODE]));
		qconf->cmpl_trig_mode =
			nla_get_u32(info->attrs[XNL_ATTR_CMPT_TRIG_MODE]);
	} else {
		// Set default value for missing or invalid CMPT_TRIG_MODE attribute
		qconf->cmpl_trig_mode = 1;
	}
}

static int xnl_dev_list(struct sk_buff *skb2, struct genl_info *info)
{
    char *buf;
    int rv;

    // Check if the info parameter is NULL, return an error if it is
    if (info == NULL) {
        pr_err("xnl_dev_list: info is NULL\n");
        return -EINVAL;
    }

    // Dump the attributes for debugging purposes
    xnl_dump_attrs(info);

    // Allocate memory for the response buffer
    buf = xnl_mem_alloc(XNL_RESP_BUFLEN_MAX, info);
    if (!buf) {
        pr_err("xnl_dev_list: Memory allocation failed\n");
        return -ENOMEM;
    }

    // Call the function to dump the list of devices into the buffer
    rv = xpdev_list_dump(buf, XNL_RESP_BUFLEN_MAX);
    if (rv < 0) {
        // Log an error message if the device list dump fails
        pr_err("xpdev_list_dump() failed: %d\n", rv);
        goto free_msg_buff;
    }

    // Send the response buffer back to the user space
    rv = xnl_respond_buffer(info, buf, strlen(buf), rv);

free_msg_buff:
    // Free the allocated buffer
    kfree(buf);
    return rv;
}

