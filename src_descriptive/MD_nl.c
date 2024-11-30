/* MD : MD: QDMA Netlink Driver Header - Handles communication between userspace and kernel */
/* MD :
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
 * 
 * This source code is modified to include debug prints and detailed comments.
 */

/* MD : MD: Configure print format to include module name and function name for better debugging */
#define pr_fmt(fmt)     KBUILD_MODNAME ":%s: " fmt, __func__

/* MD : MD: System header includes for netlink functionality */
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/netlink.h>
#include <linux/genetlink.h>
#include <net/sock.h>

/* MD : MD: QDMA specific header includes */
#include "nl.h"
#include "qdma_nl.h"
#include "libqdma/libqdma_export.h"
#include "qdma_mod.h"
#include "version.h"

/* MD : MD: Default buffer size for C2H (Card to Host) transfers */
#define QDMA_C2H_DEFAULT_BUF_SZ (4096)

/* MD : MD: Maximum line length for debug dumps */
#define DUMP_LINE_SZ            (81)

/* MD : MD: Maximum number of queues that can be dumped at once */
#define QDMA_Q_DUMP_MAX_QUEUES  (100)

/* MD : MD: Size of buffer for queue dump operations */
#define QDMA_Q_DUMP_LINE_SZ     (25 * 1024)

/* MD : MD: Size of buffer for queue list operations */
#define QDMA_Q_LIST_LINE_SZ     (200)

/* MD : MD: Debug print for tracking buffer sizes */
static inline void debug_print_sizes(void)
{
    pr_debug("QDMA Buffer Sizes - C2H Default: %d, Queue Dump: %d, Queue List: %d\n",
             QDMA_C2H_DEFAULT_BUF_SZ, QDMA_Q_DUMP_LINE_SZ, QDMA_Q_LIST_LINE_SZ);
}

/* MD : MD: Debug print for tracking maximum queue limits */
static inline void debug_print_queue_limits(void)
{
    pr_debug("QDMA Maximum Queue Limits - Dump Queues: %d\n", 
             QDMA_Q_DUMP_MAX_QUEUES);
}

/* MD : Forward declaration for device listing function */
static int xnl_dev_list(struct sk_buff *skb2, struct genl_info *info);

/* MD : RHEL (Red Hat Enterprise Linux) specific version check */
#ifdef RHEL_RELEASE_VERSION
#if RHEL_RELEASE_VERSION(8, 3) > RHEL_RELEASE_CODE

/* MD : 
 * Netlink attribute policy definitions
 * Defines the type and validation rules for netlink message attributes
 */
static struct nla_policy xnl_policy[XNL_ATTR_MAX] = {
    /* MD : Basic message attributes */
    [XNL_ATTR_GENMSG] =      { .type = NLA_NUL_STRING },    /* MD : Generic message string */
    [XNL_ATTR_DRV_INFO] =    { .type = NLA_NUL_STRING },    /* MD : Driver information string */

    /* MD : PCI device identification attributes */
    [XNL_ATTR_DEV_IDX] =     { .type = NLA_U32 },          /* MD : Device index */
    [XNL_ATTR_PCI_BUS] =     { .type = NLA_U32 },          /* MD : PCI bus number */
    [XNL_ATTR_PCI_DEV] =     { .type = NLA_U32 },          /* MD : PCI device number */
    [XNL_ATTR_PCI_FUNC] =    { .type = NLA_U32 },          /* MD : PCI function number */
    
    /* MD : Device BAR and queue configuration attributes */
    [XNL_ATTR_DEV_CFG_BAR] = { .type = NLA_U32 },          /* MD : Config BAR number */
    [XNL_ATTR_DEV_USR_BAR] = { .type = NLA_U32 },          /* MD : User BAR number */
    [XNL_ATTR_DEV_QSET_MAX] = { .type = NLA_U32 },         /* MD : Maximum queue sets */
    [XNL_ATTR_DEV_QSET_QBASE] = { .type = NLA_U32 },       /* MD : Queue base number */

    /* MD : Device version and capability attributes */
    [XNL_ATTR_VERSION_INFO] = { .type = NLA_NUL_STRING },   /* MD : Version information */
    [XNL_ATTR_DEVICE_TYPE] = { .type = NLA_NUL_STRING },    /* MD : Device type string */
    [XNL_ATTR_IP_TYPE] =     { .type = NLA_NUL_STRING },    /* MD : IP type information */
    [XNL_ATTR_DEV_NUMQS] =   { .type = NLA_U32 },          /* MD : Number of queues */
    [XNL_ATTR_DEV_NUM_PFS] = { .type = NLA_U32 },          /* MD : Number of PFs */

    /* MD : Device feature configuration attributes */
    [XNL_ATTR_DEV_MM_CHANNEL_MAX] = { .type = NLA_U32 },    /* MD : Max MM channels */
    [XNL_ATTR_DEV_MAILBOX_ENABLE] = { .type = NLA_U32 },    /* MD : Mailbox enable flag */
    [XNL_ATTR_DEV_FLR_PRESENT] =    { .type = NLA_U32 },    /* MD : FLR support flag */
    [XNL_ATTR_DEV_ST_ENABLE] =      { .type = NLA_U32 },    /* MD : ST mode enable */
    [XNL_ATTR_DEV_MM_ENABLE] =      { .type = NLA_U32 },    /* MD : MM mode enable */
    [XNL_ATTR_DEV_MM_CMPT_ENABLE] = { .type = NLA_U32 },    /* MD : MM completion enable */

    /* MD : Register access attributes */
    [XNL_ATTR_REG_BAR_NUM] = { .type = NLA_U32 },          /* MD : Register BAR number */
    [XNL_ATTR_REG_ADDR] =    { .type = NLA_U32 },          /* MD : Register address */
    [XNL_ATTR_REG_VAL] =     { .type = NLA_U32 },          /* MD : Register value */

    /* MD : Queue configuration attributes */
    [XNL_ATTR_QIDX] =         { .type = NLA_U32 },         /* MD : Queue index */
    [XNL_ATTR_NUM_Q] =        { .type = NLA_U32 },         /* MD : Number of queues */
    [XNL_ATTR_QFLAG] =        { .type = NLA_U32 },         /* MD : Queue flags */
    [XNL_ATTR_CMPT_DESC_SIZE] = { .type = NLA_U32 },       /* MD : Completion descriptor size */
    [XNL_ATTR_SW_DESC_SIZE] =   { .type = NLA_U32 },       /* MD : Software descriptor size */
    [XNL_ATTR_QRNGSZ_IDX] =    { .type = NLA_U32 },        /* MD : Queue ring size index */
    [XNL_ATTR_C2H_BUFSZ_IDX] = { .type = NLA_U32 },        /* MD : C2H buffer size index */
    [XNL_ATTR_CMPT_TIMER_IDX] = { .type = NLA_U32 },       /* MD : Completion timer index */
    [XNL_ATTR_CMPT_CNTR_IDX] = { .type = NLA_U32 },        /* MD : Completion counter index */
    [XNL_ATTR_MM_CHANNEL] =     { .type = NLA_U32 },        /* MD : MM channel numberXNL_ATTR_CMPT_TRIG_MODE] = { .type = NLA_U32 },       /* MD : Completion trigger mode */
    [XNL_ATTR_CMPT_ENTRIES_CNT] = { .type = NLA_U32 },     /* MD : Completion entries count */
    [XNL_ATTR_RANGE_START] =   { .type = NLA_U32 },        /* MD : Range start */
    [XNL_ATTR_RANGE_END] =     { .type = NLA_U32 },        /* MD : Range end */

    /* MD : Interrupt and pipeline attributes */
    [XNL_ATTR_INTR_VECTOR_IDX] = { .type = NLA_U32 },      /* MD : Interrupt vector index */
    [XNL_ATTR_PIPE_GL_MAX] =     { .type = NLA_U32 },      /* MD : Pipeline max global */
    [XNL_ATTR_PIPE_FLOW_ID] =    { .type = NLA_U32 },      /* MD : Pipeline flow ID */
    [XNL_ATTR_PIPE_SLR_ID] =     { .type = NLA_U32 },      /* MD : Pipeline SLR ID */
    [XNL_ATTR_PIPE_TDEST] =      { .type = NLA_U32 },      /* MD : Pipeline TDEST */

    /* MD : Additional device attributes */
    [XNL_ATTR_DEV_STM_BAR] =    { .type = NLA_U32 },       /* MD : STM BAR number */
    [XNL_ATTR_Q_STATE] =        { .type = NLA_U32 },       /* MD : Queue state */
    [XNL_ATTR_ERROR] =          { .type = NLA_U32 },       /* MD : Error code */
    [XNL_ATTR_PING_PONG_EN] =   { .type = NLA_U32 },       /* MD : Ping-pong enable */

    /* MD : Binary data attributes */
    [XNL_ATTR_DEV] = {
        .type = NLA_BINARY,
        .len = QDMA_DEV_ATTR_STRUCT_SIZE,                   /* MD : Device attributes struct */
    },
    [XNL_ATTR_GLOBAL_CSR] = {
        .type = NLA_BINARY,
        .len = QDMA_DEV_GLOBAL_CSR_STRUCT_SIZE,            /* MD : Global CSR struct */
    },

#ifdef ERR_DEBUG
    /* MD : Debug attributes */
    [XNL_ATTR_QPARAM_ERR_INFO] = { .type = NLA_U32 },      /* MD : Queue parameter error info */
#endif
};

/* MD : 
 * Netlink attribute policies for QDMA driver
 * Only applied for kernel versions outside 5.2.0 to 5.9.0 range
 */
#endif
#else
#if ((KERNEL_VERSION(5, 2, 0) > LINUX_VERSION_CODE) || (LINUX_VERSION_CODE > \
        KERNEL_VERSION(5, 9, 0)))
static struct nla_policy xnl_policy[XNL_ATTR_MAX] = {
    /* MD : Basic message attributes */
    [XNL_ATTR_GENMSG] =     { .type = NLA_NUL_STRING },    /* MD : Generic message string */
    [XNL_ATTR_DRV_INFO] =   { .type = NLA_NUL_STRING },    /* MD : Driver information string */

    /* MD : PCI device identification attributes */
    [XNL_ATTR_DEV_IDX] =        { .type = NLA_U32 },       /* MD : Device index */
    [XNL_ATTR_PCI_BUS] =        { .type = NLA_U32 },       /* MD : PCI bus number */
    [XNL_ATTR_PCI_DEV] =        { .type = NLA_U32 },       /* MD : PCI device number */
    [XNL_ATTR_PCI_FUNC] =       { .type = NLA_U32 },       /* MD : PCI function number */
    [XNL_ATTR_DEV_CFG_BAR] =    { .type = NLA_U32 },       /* MD : Config BAR number */
    [XNL_ATTR_DEV_USR_BAR] =    { .type = NLA_U32 },       /* MD : User BAR number */
    [XNL_ATTR_DEV_QSET_MAX] =   { .type = NLA_U32 },       /* MD : Maximum queue sets */
    [XNL_ATTR_DEV_QSET_QBASE] = { .type = NLA_U32 },       /* MD : Queue base number */

    /* MD : Device identification and capability attributes */
    [XNL_ATTR_VERSION_INFO] =   { .type = NLA_NUL_STRING }, /* MD : Version information */
    [XNL_ATTR_DEVICE_TYPE] =    { .type = NLA_NUL_STRING }, /* MD : Device type */
    [XNL_ATTR_IP_TYPE] =        { .type = NLA_NUL_STRING }, /* MD : IP type */
    [XNL_ATTR_DEV_NUMQS] =      { .type = NLA_U32 },       /* MD : Number of queues */
    [XNL_ATTR_DEV_NUM_PFS] =    { .type = NLA_U32 },       /* MD : Number of physical functions */
    [XNL_ATTR_DEV_MM_CHANNEL_MAX] = { .type = NLA_U32 },   /* MD : Max memory-mapped channels */
    [XNL_ATTR_DEV_MAILBOX_ENABLE] = { .type = NLA_U32 },   /* MD : Mailbox enable flag */
    [XNL_ATTR_DEV_FLR_PRESENT] = { .type = NLA_U32 },      /* MD : FLR support flag */
    [XNL_ATTR_DEV_ST_ENABLE] =   { .type = NLA_U32 },      /* MD : Stream mode enable */
    [XNL_ATTR_DEV_MM_ENABLE] =   { .type = NLA_U32 },      /* MD : Memory-mapped mode enable */
    [XNL_ATTR_DEV_MM_CMPT_ENABLE] = { .type = NLA_U32 },   /* MD : MM completion enable */

    /* MD : Register access attributes */
    [XNL_ATTR_REG_BAR_NUM] =    { .type = NLA_U32 },       /* MD : Register BAR number */
    [XNL_ATTR_REG_ADDR] =       { .type = NLA_U32 },       /* MD : Register address */
    [XNL_ATTR_REG_VAL] =        { .type = NLA_U32 },       /* MD : Register value */

    /* MD : CSR (Control/Status Register) attributes */
    [XNL_ATTR_CSR_INDEX] =      { .type = NLA_U32 },       /* MD : CSR index */
    [XNL_ATTR_CSR_COUNT] =      { .type = NLA_U32 },       /* MD : CSR count */

    /* MD : Queue configuration attributes */
    [XNL_ATTR_QIDX] =           { .type = NLA_U32 },       /* MD : Queue index */
    [XNL_ATTR_NUM_Q] =          { .type = NLA_U32 },       /* MD : Number of queues */
    [XNL_ATTR_QFLAG] =          { .type = NLA_U32 },       /* MD : Queue flags */
    [XNL_ATTR_CMPT_DESC_SIZE] = { .type = NLA_U32 },       /* MD : Completion descriptor size */
    [XNL_ATTR_SW_DESC_SIZE] =   { .type = NLA_U32 },       /* MD : Software descriptor size */
    [XNL_ATTR_QRNGSZ_IDX] =     { .type = NLA_U32 },       /* MD : Queue ring size index */
    [XNL_ATTR_C2H_BUFSZ_IDX] =  { .type = NLA_U32 },       /* MD : C2H buffer size index */
    [XNL_ATTR_CMPT_TIMER_IDX] = { .type = NLA_U32 },       /* MD : Completion timer index */
    [XN_CMPT_CNTR_IDX] =  { .type = NLA_U32 },       /* MD : Completion counter index */
    [XNL_ATTR_MM_CHANNEL] =     { .type = NLA_U32 },       /* MD : MM channel number */
    [XNL_ATTR_CMPT_TRIG_MODE] = { .type = NLA_U32 },       /* MD : Completion trigger mode */
    [XNL_ATTR_CMPT_ENTRIES_CNT] = { .type = NLA_U32 },     /* MD : Completion entries count */

    /* MD : Range attributes */
    [XNL_ATTR_RANGE_START] =    { .type = NLA_U32 },       /* MD : Range start */
    [XNL_ATTR_RANGE_END] =      { .type = NLA_U32 },       /* MD : Range end */

    /* MD : Interrupt and pipeline attributes */
    [XNL_ATTR_INTR_VECTOR_IDX] = { .type = NLA_U32 },      /* MD : Interrupt vector index */
    [XNL_ATTR_INTR_VECTOR_START_IDX] = { .type = NLA_U32 }, /* MD : Start vector index */
    [XNL_ATTR_INTR_VECTOR_END_IDX] = { .type = NLA_U32 },   /* MD : End vector index */
    [XNL_ATTR_RSP_BUF_LEN] =    { .type = NLA_U32 },       /* MD : Response buffer length */
    [XNL_ATTR_PIPE_GL_MAX] =    { .type = NLA_U32 },       /* MD : Max global pipeline */
    [XNL_ATTR_PIPE_FLOW_ID] =   { .type = NLA_U32 },       /* MD : Pipeline flow ID */
    [XNL_ATTR_PIPE_SLR_ID] =    { .type = NLA_U32 },       /* MD : Pipeline SLR ID */
    [XNL_ATTR_PIPE_TDEST] =     { .type = NLA_U32 },       /* MD : Pipeline TDEST */

    /* MD : Device status and configuration attributes */
    [XNL_ATTR_DEV_STM_BAR] =    { .type = NLA_U32 },       /* MD : STM BAR number */
    [XNL_ATTR_Q_STATE] =        { .type = NLA_U32 },       /* MD : Queue state */
    [XNL_ATTR_ERROR] =          { .type = NLA_U32 },       /* MD : Error code */
    [XNL_ATTR_PING_PONG_EN] =   { .type = NLA_U32 },       /* MD : Ping-pong enable */
    [XNL_ATTR_APERTURE_SZ] =    { .type = NLA_U32 },       /* MD : Aperture size */

    /* MD : Performance monitoring attributes */
    [XNL_ATTR_DEV_STAT_PING_PONG_LATMIN1] = { .type = NLA_U32 }, /* MD : Min latency 1 */
    [XNL_ATTR_DEV_STAT_PING_PONG_LATMIN2] = { .type = NLA_U32 }, /* MD : Min latency 2 */
    [XNL_ATTR_DEV_STAT_PING_PONG_LATMAX1] = { .type = NLA_U32 }, /* MD : Max latency 1 */
    [XNL_ATTR_DEV_STAT_PING_PONG_LATMAX2] = { .type = NLA_U32 }, /* MD : Max latency 2 */
    [XNL_ATTR_DEV_STAT_PING_PONG_LATAVG1] = { .type = NLA_U32 }, /* MD : Avg latency 1 */
    [XNL_ATTR_DEV_STAT_PING_PONG_LATAVG2] = { .type = NLA_U32 }, /* MD : Avg latency 2 */

    /* MD : Binary data attributes */
    [XNL_ATTR_DEV] = {
        .type = NLA_BINARY,
        .len = QDMA_DEV_ATTR_STRUCT_SIZE,                   /* MD : Device attributes struct */
    },
    [XNL_ATTR_GLOBAL_CSR] = {
        .type = NLA_BINARY,
        .len = QDMA_DEV_GLOBAL_CSR_STRUCT_SIZE,            /* MD : Global CSR struct */
    },

#ifdef ERR_DEBUG
    /* MD : Debug attributes */
    [XNL_ATTR_QPARAM_ERR_INFO] = { .type = NLA_U32 },      /* MD : Queue parameter error info */
#endif
};

#endif
#endif

/* MD : Forward declaration for buffer response handler with completion info */
static int xnl_respond_buffer_cmpt(struct genl_info *info, char *buf,
        int buflen, int error, long int cmpt_entries);

/* MD : 
 * Forward declarations for QDMA netlink operations 
 * Each function handles specific device/queue management functionality
 */

/* MD : Device management operations */
static int xnl_dev_info(struct sk_buff *, struct genl_info *);        /* MD : Get device info */
static int xnl_dev_version_capabilities(struct sk_buff *skb2,         /* MD : Get version/capabilities */
        struct genl_info *info);
static int xnl_dev_stat(struct sk_buff *, struct genl_info *);        /* MD : Get device statistics */
static int xnl_dev_stat_clear(struct sk_buff *, struct genl_info *);  /* MD : Clear device statistics */

/* MD : Queue management operations */
static int xnl_q_list(struct sk_buff *, struct genl_info *);          /* MD : List all queues */
static int xnl_q_add(struct sk_buff *, struct genl_info *);           /* MD : Add new queue */
static int xnl_q_start(struct sk_buff *, struct genl_info *);         /* MD : Start queue */
static int xnl_q_stop(struct sk_buff *, struct genl_info *);          /* MD : Stop queue */
static int xnl_q_del(struct sk_buff *, struct genl_info *);           /* MD : Delete queue */

/* MD : Queue debug/dump operations */
static int xnl_q_dump(struct sk_buff *, struct genl_info *);          /* MD : Dump queue info */
static int xnl_q_dump_desc(struct sk_buff *, struct genl_info *);     /* MD : Dump descriptors */
static int xnl_q_dump_cmpt(struct sk_buff *, struct genl_info *);     /* MD : Dump completions */

/* MD : Register and configuration operations */
static int xnl_config_reg_dump(struct sk_buff *, struct genl_info *); /* MD : Dump register config */
static int xnl_register_read(struct sk_buff *, struct genl_info *);   /* MD : Read register */
static int xnl_register_write(struct sk_buff *, struct genl_info *);  /* MD : Write register */

/* MD : Queue data operations */
static int xnl_q_read_pkt(struct sk_buff *, struct genl_info *);      /* MD : Read packet data */
static int xnl_q_read_udd(struct sk_buff *, struct genl_info *);      /* MD : Read user defined data */
static int xnl_q_cmpt_read(struct sk_buff *, struct genl_info *);     /* MD : Read completion data */

/* MD : Interrupt and CSR operations */
static int xnl_intr_ring_dump(struct sk_buff *, struct genl_info *);  /* MD : Dump interrupt ring */
static int xnl_get_global_csr(struct sk_buff *skb2,                   /* MD : Get global CSR */
        struct genl_info *info);
static int xnl_get_queue_state(struct sk_buff *, struct genl_info *); /* MD : Get queue state */
static int xnl_config_reg_info_dump(struct sk_buff *,                 /* MD : Dump register info */
        struct genl_info *);

/* MD : Debug macro for tracking function entry/exit */
#define XNL_FUNC_TRACE() \
    pr_debug("-> %s\n", __func__)

/* MD : Debug macro for parameter validation */
#define XNL_PARAM_DEBUG(fmt, ...) \
    pr_debug("%s: " fmt "\n", __func__, ##__VA_ARGS__)

/* MD : 
 * Function prototypes and version-specific definitions for QDMA netlink operations
 */

#ifdef TANDEM_BOOT_SUPPORTED
/* MD :
 * @brief Enable streaming mode during tandem boot
 * @param skb2: Socket buffer containing the request
 * @param info: Generic netlink info structure
 * @return 0 on success, negative value on failure
 */
static int xnl_en_st(struct sk_buff *skb2, struct genl_info *info);
#endif

#ifdef ERR_DEBUG
/* MD :
 * @brief Induce artificial errors for debug/testing purposes
 * @param skb2: Socket buffer containing the request
 * @param info: Generic netlink info structure
 * @return 0 on success, negative value on failure
 *
 * This function is only available when ERR_DEBUG is defined
 */
static int xnl_err_induce(struct sk_buff *skb2, struct genl_info *info);
#endif

/* MD : 
 * Version-specific policy definitions for generic netlink operations
 * These ensure compatibility across different kernel versions
 */
#ifdef RHEL_RELEASE_VERSION
/* MD : For Red Hat Enterprise Linux versions */
#if RHEL_RELEASE_VERSION(8, 3) > RHEL_RELEASE_CODE
#define GENL_OPS_POLICY    /* MD : Define policy for RHEL versions before 8.3 */
#endif
#else
/* MD : For mainline kernel versions */
#if ((KERNEL_VERSION(5, 2, 0) > LINUX_VERSION_CODE) || (LINUX_VERSION_CODE > \
		KERNEL_VERSION(5, 9, 0)))
/* MD : Define policy for kernel versions outside 5.2.0 to 5.9.0 range */
#define GENL_OPS_POLICY
#endif
#endif

/* MD : Add debug prints for kernel version checks */
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

/* MD : 
 * Array of Generic Netlink operations supported by the QDMA driver
 * Only compiled when GENL_OPS_POLICY is defined for kernel version compatibility
 */
#ifdef GENL_OPS_POLICY
static struct genl_ops xnl_ops[] = {
    /* MD : Device listing and capabilities */
    {
        .cmd = XNL_CMD_DEV_LIST,      /* MD : Command to list all QDMA devices */
        .policy = xnl_policy,          /* MD : Attribute policy for validation */
        .doit = xnl_dev_list,          /* MD : Handler function */
    },
    {
        .cmd = XNL_CMD_DEV_CAP,       /* MD : Query device capabilities */
        .policy = xnl_policy,
        .doit = xnl_dev_version_capabilities,
    },
    {
        .cmd = XNL_CMD_DEV_INFO,      /* MD : Get device information */
        .policy = xnl_policy,
        .doit = xnl_dev_info,
    },
    
    /* MD : Device statistics operations */
    {
        .cmd = XNL_CMD_DEV_STAT,      /* MD : Get device statistics */
        .policy = xnl_policy,
        .doit = xnl_dev_stat,
    },
    {
        .cmd = XNL_CMD_DEV_STAT_CLEAR, /* MD : Clear device statistics */
        .policy = xnl_policy,
        .doit = xnl_dev_stat_clear,
    },

    /* MD : Queue management operations */
    {
        .cmd = XNL_CMD_Q_LIST,        /* MD : List all queues */
        .policy = xnl_policy,
        .doit = xnl_q_list,
    },
    {
        .cmd = XNL_CMD_Q_ADD,         /* MD : Add new queue */
        .policy = xnl_policy,
        .doit = xnl_q_add,
    },
    {
        .cmd = XNL_CMD_Q_START,       /* MD : Start queue operation */
        .policy = xnl_policy,
        .doit = xnl_q_start,
    },
    {
        .cmd = XNL_CMD_Q_STOP,        /* MD : Stop queue operation */
        .policy = xnl_policy,
        .doit = xnl_q_stop,
    },
    {
        .cmd = XNL_CMD_Q_DEL,         /* MD : Delete queue */
        .policy = xnl_policy,
        .doit = xnl_q_del,
    },

    /* MD : Queue debugging and diagnostic operations */
    {
        .cmd = XNL_CMD_Q_DUMP,        /* MD : Dump queue contents */
        .policy = xnl_policy,
        .doit = xnl_q_dump,
    },
    {
        .cmd = XNL_CMD_Q_DESC,        /* MD : Dump queue descriptors */
        .policy = xnl_policy,
        .doit = xnl_q_dump_desc,
    },
    {
        .cmd = XNL_CMD_Q_CMPT,        /* MD : Dump completion queue */
        .policy = xnl_policy,
        .doit = xnl_q_dump_cmpt,
    },
    {
        .cmd = XNL_CMD_Q_UDD,         /* MD : Read user defined data */
        .policy = xnl_policy,
        .doit = xnl_q_read_udd,
    },
    {
        .cmd = XNL_CMD_Q_RX_PKT,      /* MD : Read received packets */
        .policy = xnl_policy,
        .doit = xnl_q_read_pkt,
    },
    {
        .cmd = XNL_CMD_Q_CMPT_READ,   /* MD : Read completion data */
        .policy = xnl_policy,
        .doit = xnl_q_cmpt_read,
    },

    /* MD : Register and hardware access operations */
    {
        .cmd = XNL_CMD_REG_DUMP,      /* MD : Dump registers */
        .policy = xnl_policy,
        .doit = xnl_config_reg_dump,
    },
    {
        .cmd = XNL_CMD_REG_INFO_READ, /* MD : Read register info */
        .policy = xnl_policy,
        .doit = xnl_config_reg_info_dump,
    },
    {
        .cmd = XNL_CMD_INTR_RING_DUMP, /* MD : Dump interrupt ring */
        .policy = xnl_policy,
        .doit = xnl_intr_ring_dump,
    },
    {
        .cmd = XNL_CMD_REG_RD,        /* MD : Read register value */
        .policy = xnl_policy,
        .doit = xnl_register_read,
    },
    {
        .cmd = XNL_CMD_REG_WRT,       /* MD : Write register value */
        .policy = xnl_policy,
        .doit = xnl_register_write,
    },

    /* MD : Global CSR and queue state operations */
    {
        .cmd = XNL_CMD_GLOBAL_CSR,    /* MD : Get global CSR settings */
        .policy = xnl_policy,
        .doit = xnl_get_global_csr,
    },
    {
        .cmd = XNL_CMD_GET_Q_STATE,   /* MD : Get queue state */
        .policy = xnl_policynl_get_queue_state,
    },

/* MD : 
 * Conditional netlink command registration blocks
 * These commands are only available when specific features are enabled
 */

#ifdef TANDEM_BOOT_SUPPORTED
    /* MD : 
     * Command registration for enabling streaming mode in tandem boot
     * This operation is only available when TANDEM_BOOT_SUPPORTED is defined
     */
    {
        .cmd = XNL_CMD_EN_ST,         /* MD : Command ID for enabling streaming mode */
        .policy = xnl_policy,         /* MD : Policy rules for command attributes */
        .doit = xnl_en_st,           /* MD : Handler function for streaming enable */
    },
#endif

#ifdef ERR_DEBUG
    /* MD :
     * Command registration for error injection testing
     * This operation is only available when ERR_DEBUG is defined
     * Used for debugging and testing error handling paths
     */
    {
        .cmd = XNL_CMD_Q_ERR_INDUCE,  /* MD : Command ID for error injection */
        .policy = xnl_policy,         /* MD : Policy rules for command attributes */
        .doit = xnl_err_induce,       /* MD : Handler function for error injection */
    }
#endif
};

/* MD : Debug prints for tracking command registration */
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

/* MD : Debug macro for tracking netlink commands */
#define XNL_DEBUG_CMD(cmd) \
    pr_debug("QDMA: Processing netlink command %s (#%d)\n", \
             #cmd, cmd)

/* MD : Example usage in a handler function:
static int xnl_dev_list(struct sk_buff *skb2, struct genl_info *info)
{_DEBUG_CMD(XNL_CMD_DEV_LIST);
    // MD : Rest of the function implementation
}
*/

/* MD :
 * Array of Generic Netlink operations supported by the QDMA driver
 * Each operation maps a command ID to its handler function
 * Note: This version does not use the policy attribute as it's defined
 * outside of GENL_OPS_POLICY
 */
static struct genl_ops xnl_ops[] = {
    /* MD : Device management operations */
    {
        .cmd = XNL_CMD_DEV_LIST,           /* MD : List all QDMA devices */
        .doit = xnl_dev_list,              /* MD : Handler for device listing */
    },
    {
        .cmd = XNL_CMD_DEV_CAP,            /* MD : Query device capabilities */
        .doit = xnl_dev_version_capabilities, /* MD : Handler for capability info */
    },
    {
        .cmd = XNL_CMD_DEV_INFO,           /* MD : Get device information */
        .doit = xnl_dev_info,              /* MD : Handler for device info */
    },
    {
        .cmd = XNL_CMD_DEV_STAT,           /* MD : Get device statistics */
        .doit = xnl_dev_stat,              /* MD : Handler for device stats */
    },
    {
        .cmd = XNL_CMD_DEV_STAT_CLEAR,     /* MD : Clear device statistics */
        .doit = xnl_dev_stat_clear,        /* MD : Handler for clearing stats */
    },

    /* MD : Queue management operations */
    {
        .cmd = XNL_CMD_Q_LIST,             /* MD : List all queues */
        .doit = xnl_q_list,                /* MD : Handler for queue listing */
    },
    {
        .cmd = XNL_CMD_Q_ADD,              /* MD : Add new queue */
        .doit = xnl_q_add,                 /* MD : Handler for queue addition */
    },
    {
        .cmd = XNL_CMD_Q_START,            /* MD : Start queue */
        .doit = xnl_q_start,               /* MD : Handler for starting queue */
    },
    {
        .cmd = XNL_CMD_Q_STOP,             /* MD : Stop queue */
        .doit = xnl_q_stop,                /* MD : Handler for stopping queue */
    },
    {
        .cmd = XNL_CMD_Q_DEL,              /* MD : Delete queue */
        .doit = xnl_q_del,                 /* MD : Handler for queue deletion */
    },
    {
        .cmd = XNL_CMD_Q_DUMP,             /* MD : Dump queue data */
        .doit = xnl_q_dump,                /* MD : Handler for queue dump */
    },
    {
        .cmd = XNL_CMD_Q_DESC,             /* MD : Dump queue descriptors */
        .doit = xnl_q_dump_desc,           /* MD : Handler for descriptor dump */
    },

    /* MD : Register and Configuration operations */
    {
        .cmd = XNL_CMD_REG_DUMP,           /* MD : Dump registers */
        .doit = xnl_config_reg_dump,       /* MD : Handler for register dump */
    },
    {
        .cmd = XNL_CMD_REG_INFO_READ,      /* MD : Read register info */
        .doit = xnl_config_reg_info_dump,  /* MD : Handler for register info */
    },

    /* MD : Completion and Data handling operations */
    {
        .cmd = XNL_CMD_Q_CMPT,             /* MD : Queue completion */
        .doit = xnl_q_dump_cmpt,           /* MD : Handler for completion dump */
    },
    {
        .cmd = XNL_CMD_Q_UDD,              /* MD : User defined data */
        .doit = xnl_q_read_udd,            /* MD : Handler for UDD reading */
    },
    {
        .cmd = XNL_CMD_Q_RX_PKT,           /* MD : Receive packet */
        .doit = xnl_q_read_pkt,            /* MD : Handler for packet reading */
    },
    {
        .cmd = XNL_CMD_Q_CMPT_READ,        /* MD : Read completion data */
        .doit = xnl_q_cmpt_read,           /* MD : Handler for completion reading */
    },

    /* MD : System and Debug operations */
    {
        .cmd = XNL_CMD_INTR_RING_DUMP,     /* MD : Dump interrupt ring */
        .doit = xnl_intr_ring_dump,        /* MD : Handler for interrupt ring dump */
    },
    {
        .cmd = XNL_CMD_REG_RD,             /* MD : Read register */
        .doit = xnl_register_read,         /* MD : Handler for register reading */
    },
    {
        .cmd = XNL_CMD_REG_WRT,            /* MD : Write register */
        .doit = xnl_register_write,        /* MD : Handler for register writing */
    },
    {
        .cmd = XNL_CMD_GLOBAL_CSR,         /* MD : Global CSR access */
        .doit = xnl_get_global_csr,        /* MD : Handler for global CSR access */
    },
    {
        .cmd = XNL_CMD_GET_Q_STATE,        /* MD : Get queue state */
        .doit = xnl_get_queue_state,       /* MD : Handler for queue state query */
    },

// MD : Define debug macros for function tracing and parameter validation
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

// MD : Comprehensive comment block explaining the purpose of each function group
/* MD :
 * Function Group: Command Handlers
 * These functions handle specific commands received via netlink.
 * Each function is responsible for executing a particular operation
 * related to the QDMA device, such as enabling streaming mode or
 * inducing errors for debugging purposes.
 */

// MD : Command Handlers
#ifdef TANDEM_BOOT_SUPPORTED
/* MD :*
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
/* MD :*
 * @brief Induce artificial errors for debugging/testing purposes.
 * This function is invoked when the XNL_CMD_Q_ERR_INDUCE command is received.
 * It sets up conditions to simulate errors in the queue for testing.
 */
{
    .cmd = XNL_CMD_Q_ERR_INDUCE,
    .doit = xnl_err_induce,
}
#endif

// MD : Define the generic netlink family
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

// MD : Define debug macros for function tracing and parameter validation
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

/* MD :
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
    /* MD : Calculate buffer size - use GOODSIZE if min_sz is too small */
    unsigned long sz = min_sz < NLMSG_GOODSIZE ? NLMSG_GOODSIZE : min_sz;

    /* MD : Debug print for allocation request */
    pr_debug("QDMA: Allocating netlink message buffer of size %lu bytes\n", sz);

    /* MD : Allocate new generic netlink message */
    skb = genlmsg_new(sz, GFP_KERNEL);
    if (!skb) {
        pr_err("QDMA: failed to allocate skb of size %lu bytes\n", sz);
        return NULL;
    }

    /* MD : Initialize message header */
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

/* MD :
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

    /* MD : Add string as netlink attribute */
    rv = nla_put_string(skb, type, s);
    if (rv != 0) {
        pr_err("QDMA: Failed to add string attribute, error %d\n", rv);
        return -EINVAL;
    }

    pr_debug("QDMA: Successfully added string attribute\n");
    return 0;
}

/* MD :
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

    /* MD : Add data as netlink attribute */
    rv = nla_put(skb, type, len, s);
    if (rv != 0) {
        pr_err("QDMA: Failed to add data attribute, error %d\n", rv);
        return -EINVAL;
    }

    pr_debug("QDMA: Successfully added data attribute\n");
    return 0;
}

/* MD :*
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

    /* MD : Debug print before adding attribute */
    pr_debug("QDMA: Adding u32 attribute type %d, value %u\n", type, v);

    /* MD : Add the attribute to the netlink message */
    rv = nla_put_u32(skb, type, v);
    if (rv != 0) {
        pr_err("QDMA: Failed to add u32 attribute type %d, error %d\n", 
               type, rv);
        return -EINVAL;
    }

    pr_debug("QDMA: Successfully added u32 attribute\n");
    return 0;
}

/* MD :
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

    /* MD : Debug print before finalizing message */
    pr_debug("QDMA: Finalizing netlink message for port %d\n", 
             info->snd_portid);

    /* MD : Finalize the message by updating length fields */
    genlmsg_end(skb_tx, hdr);

    /* MD : Send the message to the specified port */
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

/* MD :
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

    /* MD : Log sequence number and port ID for message tracking */
    pr_info("snd_seq 0x%x, portid 0x%x.\n",
        info->snd_seq, info->snd_portid);

    /* MD : Disabled debug dump of raw netlink and generic netlink headers */
#if 0
    print_hex_dump_bytes("nlhdr", DUMP_PREFIX_OFFSET, info->nlhdr,
            sizeof(struct nlmsghdr));
    pr_info("\n"); {
    print_hex_dump_bytes("genlhdr", DUMP_PREFIX_OFFSET, info->genlhdr,
            sizeof(struct genlmsghdr));
    pr_info("\n");
#endif

    /* MD : Log netlink header details for debugging */
    pr_info("nlhdr: len %u, type %u, flags 0x%x, seq 0x%x, pid %u.\n",
        info->nlhdr->nlmsg_len,
        info->nlhdr->nlmsg_type, 
        info->nlhdr->nlmsg_flags,
        info->nlhdr->nlmsg_seq,
        info->nlhdr->nlmsg_pid);

    /* MD : Log generic netlink header details */
    pr_info("genlhdr: cmd 0x%x %s, version %u, reserved 0x%x.\n",
        info->genlhdr->cmd, xnl_op_str[info->genlhdr->cmd],
        info->genlhdr->version,
        info->genlhdr->reserved);

    /* MD : Iterate through all possible attributes */
    for (i = 0; i < XNL_ATTR_MAX; i++) {
        struct nlattr *na = info->attrs[i];

        if (na) {
            /* MD : Handle kernel version specific attribute policy checking */
#if ((KERNEL_VERSION(5, 2, 0) > LINUX_VERSION_CODE) || (LINUX_VERSION_CODE > \
        KERNEL_VERSION(5, 9, 0)))
            if (xnl_policy[i].type == NLA_NUL_STRING) {
#else
            if (1) {
#endif
                /* MD : Handle string attributes */
                char *s = (char *)nla_data(na);

                if (s)
                    pr_info("attr %d, %s, str %s.\n",
                        i, xnl_attr_str[i], s);
                else
                    pr_info("attr %d, %s, str NULL.\n",
                        i, xnl_attr_str[i]);
            } else {
                /* MD : Handle 32-bit unsigned integer attributes */
                u32 v = nla_get_u32(na);
                pr_info("attr %s, u32 0x%x.\n",
                    xnl_attr_str[i], v);
            }
        }
    }

    return 0;
}
#else
#define xnl_dump_attrs(info)  /* MD : Empty macro when DEBUG not defined */
#endif

/* MD :
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

    /* MD : Allocate new socket buffer for response */
    pr_debug("Allocating response message for cmd %d\n", info->genlhdr->cmd);
    skb = xnl_msg_alloc(info->genlhdr->cmd, buflen, &hdr, info);
    if (!skb) {
        pr_err("Failed to allocate socket buffer\n");
        return -ENOMEM;
    }

    /* MD : Add the message buffer as string attribute */
    pr_debug("Adding message buffer attribute\n");
    rv = xnl_msg_add_attr_str(skb, XNL_ATTR_GENMSG, buf);
    if (rv != 0) {
        pr_err("xnl_msg_add_attr_str() failed: %d\n", rv);
        nlmsg_free(skb);
        return rv;
    }

    /* MD : Add error code as attribute */
    pr_debug("Adding error code attribute: %d\n", error);
    rv = xnl_msg_add_attr_uint(skb, XNL_ATTR_ERROR, error);
    if (rv != 0) {
        pr_err("xnl_msg_add_attr_uint() failed: %d\n", rv);
        nlmsg_free(skb);
        return rv;
    }

    /* MD : Add completion entry count as attribute */
    pr_debug("Adding completion entry count: %ld\n", cmpt_entries);
    rv = xnl_msg_add_attr_uint(skb, XNL_ATTR_CMPT_ENTRIES_CNT,
            cmpt_entries);
    if (rv != 0) {
        pr_err("xnl_msg_add_attr_uint() failed: %d\n", rv);
        nlmsg_free(skb);
        return rv;
    }

    /* MD : Send the message */
    pr_debug("Sending response message\n");
    rv = xnl_msg_send(skb, hdr, info);

    return rv;
}

/* MD :
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

    /* MD : Debug print for tracking response parameters */
    pr_debug("QDMA: Sending response - buflen=%d, error=%d\n", buflen, error);

    /* MD : Allocate a new netlink message */
    skb = xnl_msg_alloc(info->genlhdr->cmd, buflen, &hdr, info);
    if (!skb) {
        pr_err("QDMA: Failed to allocate skb for response\n");
        return -ENOMEM;
    }

    /* MD : Add the message string as an attribute */
    rv = xnl_msg_add_attr_str(skb, XNL_ATTR_GENMSG, buf);
    if (rv != 0) {
        pr_err("QDMA: Failed to add message attribute: %d\n", rv);
        nlmsg_free(skb);
        return rv;
    }

    /* MD : Add the error code as an attribute */
    rv = xnl_msg_add_attr_uint(skb, XNL_ATTR_ERROR, error);
    if (rv != 0) {
        pr_err("QDMA: Failed to add error attribute: %d\n", rv);
        nlmsg_free(skb);
        return rv;
    }

    /* MD : Send the netlink message */
    rv = xnl_msg_send(skb, hdr, info);
    pr_debug("QDMA: Response sent with result: %d\n", rv);

    return rv;
}

/* MD :
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

    /* MD : Debug print for tracking data response */
    pr_debug("QDMA: Sending data response - buflen=%d\n", buflen);

    /* MD : Allocate a new netlink message */
    skb = xnl_msg_alloc(info->genlhdr->cmd, buflen, &hdr, info);
    if (!skb) {
        pr_err("QDMA: Failed to allocate skb for data response\n");
        return -ENOMEM;
    }

    /* MD : Add the data as a binary attribute */
    rv = xnl_msg_add_attr_data(skb, XNL_ATTR_GLOBAL_CSR, buf, buflen);
    if (rv != 0) {
        pr_err("QDMA: Failed to add data attribute: %d\n", rv);
        nlmsg_free(skb);
        return rv;
    }

    /* MD : Send the netlink message */
    rv = xnl_msg_send(skb, hdr, info);
    pr_debug("QDMA: Data response sent with result: %d\n", rv);

    return rv;
}

/* MD :
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
    char ebuf[XNL_ERR_BUFLEN];  /* MD : Error message buffer */
    char *buf;                   /* MD : Allocated memory buffer */
    int rv;                      /* MD : Return value for error handling */

    /* MD : Debug print for tracking memory allocation requests */
    pr_debug("QDMA: Attempting to allocate %d bytes\n", l);

    /* MD : Attempt to allocate memory */
    buf = kmalloc(l, GFP_KERNEL);
    if (buf) {
        /* MD : Zero out the allocated memory for safety */
        memset(buf, 0, l);
        pr_debug("QDMA: Successfully allocated %d bytes at %p\n", l, buf);
        return buf;
    }

    /* MD : Memory allocation failed - handle error */
    pr_err("QDMA: Memory allocation failed - OOM %d bytes\n", l);

    /* MD : Prepare error message for user */
    rv = snprintf(ebuf, XNL_ERR_BUFLEN, "ERR! xnl OOM %d.\n", l);

    /* MD : Send error message back to user space via netlink */
    xnl_respond_buffer(info, ebuf, XNL_ERR_BUFLEN, rv);

    return NULL;
}

/* MD :
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
    u32 idx;                        /* MD : Device index from netlink message */
    struct xlnx_pci_dev *xpdev;     /* MD : PCI device context pointer */
    char err[XNL_ERR_BUFLEN];       /* MD : Error message buffer */
    int rv = 0;                     /* MD : Return value for error handling */

    /* MD : Validate input parameter */
    if (info == NULL) {
        pr_err("QDMA: Null info pointer received\n");
        return NULL;
    }

    /* MD : Debug print for tracking message processing */
    pr_debug("QDMA: Processing netlink message cmd=%d\n", info->genlhdr->cmd);

    /* MD : Check for required device index attribute */
    if (!info->attrs[XNL_ATTR_DEV_IDX]) {
        /* MD : Prepare error message for missing attribute */
        snprintf(err, sizeof(err),
            "command %s missing attribute XNL_ATTR_DEV_IDX",
            xnl_op_str[info->genlhdr->cmd]);
        rv = -EINVAL;
        pr_err("QDMA: %s\n", err);
        goto respond_error;
    }

    /* MD : Extract device index from netlink attributes */
    idx = nla_get_u32(info->attrs[XNL_ATTR_DEV_IDX]);
    pr_debug("QDMA: Looking up device with index %u\n", idx);

    /* MD : Look up PCI device context using index */
    xpdev = xpdev_find_by_idx(idx, err, sizeof(err));
    if (!xpdev) {
        rv = -EINVAL;
        pr_err("QDMA: Device lookup failed for idx=%u\n", idx);
        goto respond_error;
    }

    pr_debug("QDMA: Successfully found device idx=%u\n", idx);
    return xpdev;

respond_error:
    /* MD : Send error message back to user space via netlink */
    xnl_respond_buffer(info, err, strlen(err), rv);
    return NULL;
}

/* MD :
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
    char ebuf[XNL_ERR_BUFLEN];  // MD : Buffer for error messages
    struct xlnx_qdata *qdata;

    // MD : Attempt to retrieve the queue data for the specified queue index
    qdata = xpdev_queue_get(xpdev, qconf->qidx, qconf->q_type, 1, ebuf, XNL_ERR_BUFLEN);

    // MD : Check if the queue data retrieval was unsuccessful
    if (!qdata) {
        // MD : Log an error message indicating the invalid queue index
        snprintf(ebuf, XNL_ERR_BUFLEN, "ERR! qidx %u invalid.\n", qconf->qidx);
        pr_debug("QDMA: %s", ebuf);  // MD : Debug print for tracking errors

        // MD : Send an error response back to user space
        xnl_respond_buffer(info, ebuf, XNL_ERR_BUFLEN, 0);
    }

    return qdata;  // MD : Return the queue data or NULL if not found
}

/* MD :*
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
    int rv = 0;  // MD : Return value indicating success or failure

    // MD : Check if the specified attribute is missing
    if (!info->attrs[xnl_attr]) {
        // MD : If a buffer is provided, write an error message to it
        if (buf) {
            rv += snprintf(buf, buflen, "Missing attribute %s for qidx = %u\n",
                           xnl_attr_str[xnl_attr], qidx);
            pr_debug("QDMA: Missing attribute %s for qidx = %u\n", 
                     xnl_attr_str[xnl_attr], qidx);  // MD : Debug print for missing attribute
        }
        rv = -1;  // MD : Set return value to indicate failure
    }

    return rv;  // MD : Return success or failure
}


static void xnl_extract_extra_config_attr(struct genl_info *info,
					struct qdma_queue_conf *qconf)
{
	u32 f = nla_get_u32(info->attrs[XNL_ATTR_QFLAG]);

	/* MD : Extract flags and assign values to configuration settings */
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
	

	/* MD : Additional settings specific to completion queue type */
	if (qconf->q_type == Q_CMPT)
		qconf->cmpl_udd_en = 1;

	/* MD : Extract additional configuration attributes based on presence in message */
	if (xnl_chk_attr(XNL_ATTR_QRNGSZ_IDX, info, qconf->qidx, NULL, 0) == 0) {
		// MD : Debug print for tracking attribute extraction
		pr_debug("Extracted QRNGSZ_IDX: %d\n", nla_get_u32(info->attrs[XNL_ATTR_QRNGSZ_IDX]));
		qconf->desc_rng_sz_idx = qconf->cmpl_rng_sz_idx =
				nla_get_u32(info->attrs[XNL_ATTR_QRNGSZ_IDX]);
	}
	if (xnl_chk_attr(XNL_ATTR_C2H_BUFSZ_IDX, info,
			qconf->qidx, NULL, 0) == 0) {
		// MD : Debug print for tracking attribute extraction
		pr_debug("Extracted C2H_BUFSZ_IDX: %d\n", nla_get_u32(info->attrs[XNL_ATTR_C2H_BUFSZ_IDX]));
		qconf->c2h_buf_sz_idx =
			nla_get_u32(info->attrs[XNL_ATTR_C2H_BUFSZ_IDX]);
	}
	if (xnl_chk_attr(XNL_ATTR_CMPT_TIMER_IDX, info,
			qconf->qidx, NULL, 0) == 0) {
		// MD : Debug print for tracking attribute extraction
		pr_debug("Extracted CMPT_TIMER_IDX: %d\n", nla_get_u32(info->attrs[XNL_ATTR_CMPT_TIMER_IDX]));
		qconf->cmpl_timer_idx =
			nla_get_u32(info->attrs[XNL_ATTR_CMPT_TIMER_IDX]);
	}
	if (xnl_chk_attr(XNL_ATTR_CMPT_CNTR_IDX, info,
			qconf->qidx, NULL, 0) == 0) {
		// MD : Debug print for tracking attribute extraction
		pr_debug("Extracted CMPT_CNTR_IDX: %d\n", nla_get_u32(info->attrs[XNL_ATTR_CMPT_CNTR_IDX]));
		qconf->cmpl_cnt_th_idx =
			nla_get_u32(info->attrs[XNL_ATTR_CMPT_CNTR_IDX]);
	}
	if (xnl_chk_attr(XNL_ATTR_MM_CHANNEL, info, qconf->qidx, NULL, 0) == 0) {
		// MD : Debug print for tracking attribute extraction
		pr_debug("Extracted MM_CHANNEL: %d\n", nla_get_u32(info->attrs[XNL_ATTR_MM_CHANNEL]));
		qconf->mm_channel =
			nla_get_u32(info->attrs[XNL_ATTR_MM_CHANNEL]);
	}
	if (xnl_chk_attr(XNL_ATTR_CMPT_DESC_SIZE,
				info, qconf->qidx, NULL, 0) == 0) {
		// MD : Debug print for tracking attribute extraction
		pr_debug("Extracted CMPT_DESC_SIZE: %d\n", nla_get_u32(info->attrs[XNL_ATTR_CMPT_DESC_SIZE]));
		qconf->cmpl_desc_sz =
			nla_get_u32(info->attrs[XNL_ATTR_CMPT_DESC_SIZE]);
	}
	if (xnl_chk_attr(XNL_ATTR_SW_DESC_SIZE,
				info, qconf->qidx, NULL, 0) == 0) {
		// MD : Debug print for tracking attribute extraction
		pr_debug("Extracted SW_DESC_SIZE: %d\n", nla_get_u32(info->attrs[XNL_ATTR_SW_DESC_SIZE]));
		qconf->sw_desc_sz =
			nla_get_u32(info->attrs[XNL_ATTR_SW_DESC_SIZE]);
	}
	if (xnl_chk_attr(XNL_ATTR_PING_PONG_EN,
					 info, qconf->qidx, NULL, 0) == 0) {
		// MD : Debug print for tracking attribute extraction
		pr_debug("Extracted PING_PONG_EN: %d\n", nla_get_u32(info->attrs[XNL_ATTR_PING_PONG_EN]));
		qconf->ping_pong_en = 1;
	}
	if (xnl_chk_attr(XNL_ATTR_APERTURE_SZ,
					 info, qconf->qidx, NULL, 0) == 0) {
		// MD : Debug print for tracking attribute extraction
		pr_debug("Extracted APERTURE_SZ: %d\n", nla_get_u32(info->attrs[XNL_ATTR_APERTURE_SZ]));
		qconf->aperture_size =
			nla_get_u32(info->attrs[XNL_ATTR_APERTURE_SZ]);
	}
	if (xnl_chk_attr(XNL_ATTR_CMPT_TRIG_MODE, info,
				qconf->qidx, NULL, 0) == 0) {
		// MD : Debug print for tracking attribute extraction
		pr_debug("Extracted CMPT_TRIG_MODE: %d\n", nla_get_u32(info->attrs[XNL_ATTR_CMPT_TRIG_MODE]));
		qconf->cmpl_trig_mode =
			nla_get_u32(info->attrs[XNL_ATTR_CMPT_TRIG_MODE]);
	} else {
		// MD : Set default value for missing or invalid CMPT_TRIG_MODE attribute
		qconf->cmpl_trig_mode = 1;
	}
}

static int xnl_dev_list(struct sk_buff *skb2, struct genl_info *info)
{
    char *buf;
    int rv;

    // MD : Check if the info parameter is NULL, return an error if it is
    if (info == NULL) {
        pr_err("xnl_dev_list: info is NULL\n");
        return -EINVAL;
    }

    // MD : Dump the attributes for debugging purposes
    xnl_dump_attrs(info);

    // MD : Allocate memory for the response buffer
    buf = xnl_mem_alloc(XNL_RESP_BUFLEN_MAX, info);
    if (!buf) {
        pr_err("xnl_dev_list: Memory allocation failed\n");
        return -ENOMEM;
    }

    // MD : Call the function to dump the list of devices into the buffer
    rv = xpdev_list_dump(buf, XNL_RESP_BUFLEN_MAX);
    if (rv < 0) {
        // MD : Log an error message if the device list dump fails
        pr_err("xpdev_list_dump() failed: %d\n", rv);
        goto free_msg_buff;
    }

    // MD : Send the response buffer back to the user space
    rv = xnl_respond_buffer(info, buf, strlen(buf), rv);

free_msg_buff:
    // MD : Free the allocated buffer
    kfree(buf);
    return rv;
}

static int xnl_dev_info(struct sk_buff *skb2, struct genl_info *info)
{
    // MD : Function takes a socket buffer and generic netlink info as parameters
    
    // MD : Declare required variables
    struct sk_buff *skb;          // MD : New socket buffer for response
    void *hdr;                    // MD : Header pointer for netlink message 
    struct xlnx_pci_dev *xpdev;   // MD : Xilinx PCI device structure
    struct pci_dev *pdev;         // MD : Linux PCI device structure  
    struct qdma_dev_conf conf;    // MD : QDMA device configuration
    int rv;                       // MD : Return value

    // MD : Validate input parameter
    if (info == NULL)
        return -EINVAL;

    // MD : Debug print netlink attributes
    xnl_dump_attrs(info);

    // MD : Get Xilinx device handle and validate
    xpdev = xnl_rcv_check_xpdev(info);
    if (!xpdev)
        return -EINVAL;
    pdev = xpdev->pdev;  // MD : Get Linux PCI device pointer

    // MD : Get QDMA device configuration
    rv = qdma_device_get_config(xpdev->dev_hndl, &conf, NULL, 0); 
    if (rv < 0)
        return rv;

    // MD : Allocate new netlink message
    skb = xnl_msg_alloc(XNL_CMD_DEV_INFO, 0, &hdr, info);
    if (!skb)
        return -ENOMEM;

    // MD : Add device information attributes to message:

    // MD : PCI bus number
    rv = xnl_msg_add_attr_uint(skb, XNL_ATTR_PCI_BUS, pdev->bus->number);
    if (rv < 0) {
        pr_err("xnl_msg_add_attr_uint() failed: %d", rv);
        goto free_skb;
    }

    // MD : PCI device number 
    rv = xnl_msg_add_attr_uint(skb, XNL_ATTR_PCI_DEV, PCI_SLOT(pdev->devfn));
    if (rv < 0) {
        pr_err("xnl_msg_add_attr_uint() failed: %d", rv);
        goto free_skb;
    }

    // MD : PCI function number
    rv = xnl_msg_add_attr_uint(skb, XNL_ATTR_PCI_FUNC, PCI_FUNC(pdev->devfn));
    if (rv < 0) {
        pr_err("xnl_msg_add_attr_uint() failed: %d", rv);
        goto free_skb;
    }

    // MD : Config BAR number
    rv = xnl_msg_add_attr_uint(skb, XNL_ATTR_DEV_CFG_BAR, conf.bar_num_config);
    if (rv < 0) {
        pr_err("xnl_msg_add_attr_uint() failed: %d", rv);
        goto free_skb;
    }

    // MD : User BAR number
    rv = xnl_msg_add_attr_uint(skb, XNL_ATTR_DEV_USR_BAR, conf.bar_num_user);
    if (rv < 0) {
        pr_err("xnl_msg_add_attr_uint() failed: %d", rv);
        goto free_skb;
    }

    // MD : Maximum queue sets
    rv = xnl_msg_add_attr_uint(skb, XNL_ATTR_DEV_QSET_MAX, conf.qsets_max);
    if (rv < 0) {
        pr_err("xnl_msg_add_attr_uint() failed: %d", rv);
        goto free_skb;
    }

    // MD : Queue set base
    rv = xnl_msg_add_attr_uint(skb, XNL_ATTR_DEV_QSET_QBASE, conf.qsets_base);
    if (rv < 0) {
        pr_err("xnl_msg_add_attr_uint() failed: %d", rv);
        goto free_skb;
    }

    // MD : Send the netlink message
    rv = xnl_msg_send(skb, hdr, info);
    return rv;

free_skb:
    // MD : Error handler - free socket buffer
    nlmsg_free(skb);
    return rv;
}

static int xnl_dev_version_capabilities(struct sk_buff *skb2, struct genl_info *info)
{
    // MD : Function to get and send QDMA device version and capabilities information
    struct sk_buff *skb;        // MD : Socket buffer for netlink message
    void *hdr;                  // MD : Header for netlink message  
    struct xlnx_pci_dev *xpdev; // MD : Xilinx PCIe device structure
    struct qdma_version_info ver_info;      // MD : Structure to hold version info
    struct qdma_dev_attributes dev_attr;    // MD : Structure to hold device attributes
    char buf[XNL_RESP_BUFLEN_MIN];         // MD : Buffer for formatted string response
    int buflen = XNL_RESP_BUFLEN_MIN;      // MD : Buffer length
    int rv = 0;                            // MD : Return value

    // MD : Validate input parameter
    if (info == NULL)
        return -EINVAL;

    // MD : Debug dump of netlink attributes 
    xnl_dump_attrs(info);

    // MD : Get Xilinx device handle and validate
    xpdev = xnl_rcv_check_xpdev(info);
    if (!xpdev)
        return -EINVAL;

    // MD : Allocate new netlink message
    skb = xnl_msg_alloc(XNL_CMD_DEV_CAP, 0, &hdr, info);
    if (!skb)
        return -ENOMEM;

    // MD : Get device version information
    rv = qdma_device_version_info(xpdev->dev_hndl, &ver_info);
    if (rv < 0) {
        pr_err("qdma_device_version_info() failed: %d", rv);
        goto free_skb;
    }

    // MD : Get device capabilities information  
    rv = qdma_device_capabilities_info(xpdev->dev_hndl, &dev_attr);
    if (rv < 0) {
        pr_err("qdma_device_capabilities_info() failed: %d", rv);
        goto free_skb;
    }

    // MD : Format version information into buffer
    rv = snprintf(buf + rv, buflen,
            "=============Hardware Version============\n\n");
    rv += snprintf(buf + rv, buflen - rv,
            "RTL Version         : %s\n", ver_info.rtl_version_str);
    rv += snprintf(buf + rv,
            buflen - rv,
            "Vivado ReleaseID    : %s\n",
            ver_info.vivado_release_str); 
    rv += snprintf(buf + rv,
            buflen - rv,
            "QDMA Device Type    : %s\n",
            ver_info.device_type_str);
    rv += snprintf(buf + rv,
            buflen - rv,
            "QDMA IP Type    : %s\n",
            ver_info.ip_str);
    rv += snprintf(buf + rv,
            buflen - rv,
            "============Software Version============\n\n");
    rv += snprintf(buf + rv,
            buflen - rv,
            "qdma driver version : %s\n\n",
            DRV_MODULE_VERSION);

    // MD : Add debug print
    pr_debug("DEV_CAP: Version info formatted into buffer\n");

    // MD : Add version info as string attribute to netlink message
    rv = xnl_msg_add_attr_str(skb, XNL_ATTR_VERSION_INFO, buf);
    if (rv != 0) {
        pr_err("xnl_msg_add_attr_str() failed: %d", rv);
        goto free_skb;
    }

    // MD : Add device type as string attribute
    rv = xnl_msg_add_attr_str(skb, XNL_ATTR_DEVICE_TYPE,
            ver_info.device_type_str);
    if (rv != 0) {
        pr_err("xnl_msg_add_attr_str() failed: %d", rv);
        goto free_skb;
    }

    // MD : Add IP type as string attribute 
    rv = xnl_msg_add_attr_str(skb, XNL_ATTR_IP_TYPE, ver_info.ip_str);
    if (rv != 0) {
        pr_err("xnl_msg_add_attr_str() failed: %d", rv);
        goto free_skb;
    }

    // MD : Add device capabilities as uint attributes:

    // MD : Memory mapped mode enable
    rv = xnl_msg_add_attr_uint(skb, XNL_ATTR_DEV_MM_ENABLE, dev_attr.mm_en);
    if (rv < 0) {
        pr_err("xnl_msg_add_attr_uint() failed: %d", rv);
        goto free_skb;
    }

    // MD : Stream mode enable
    rv = xnl_msg_add_attr_uint(skb, XNL_ATTR_DEV_ST_ENABLE, dev_attr.st_en);
    if (rv < 0) {
        pr_err("xnl_msg_add_attr_uint() failed: %d", rv);
        goto free_skb;
    }

    // MD : Memory mapped completion enable
    rv = xnl_msg_add_attr_uint(skb, XNL_ATTR_DEV_MM_CMPT_ENABLE,
            dev_attr.mm_cmpt_en);
    if (rv < 0) {
        pr_err("xnl_msg_add_attr_uint() failed: %d", rv);
        goto free_skb;
    }

    // MD : Number of queues
    rv = xnl_msg_add_attr_uint(skb, XNL_ATTR_DEV_NUMQS,
            dev_attr.num_qs);
    if (rv < 0) {
        pr_err("xnl_msg_add_attr_uint() failed: %d", rv);
        goto free_skb;
    }

    // MD : Number of physical functions
    rv = xnl_msg_add_attr_uint(skb, XNL_ATTR_DEV_NUM_PFS, dev_attr.);
    if (rv < 0) {
        pr_err("xnl_msg_add_attr_uint() failed: %d", rv);
        goto free_skb;
    }

    // MD : Maximum memory mapped channels
    rv = xnl_msg_add_attr_uint(skb, XNL_ATTR_DEV_MM_CHANNEL_MAX,
            dev_attr.mm_channel_max);
    if (rv < 0) {
        pr_err("xnl_msg_add_attr_uint() failed: %d", rv);
        goto free_skb;
    }

    // MD : Mailbox enable
    rv = xnl_msg_add_attr_uint(skb, XNL_ATTR_DEV_MAILBOX_ENABLE,
            dev_attr.mailbox_en);
    if (rv < 0) {
        pr_err("xnl_msg_add_attr_uint() failed: %d", rv);
        goto free_skb;
    }

    // MD : FLR present
    rv = xnl_msg_add_attr_uint(skb, XNL_ATTR_DEV_FLR_PRESENT,
            dev_attr.flr_present);
    if (rv < 0) {
        pr_err("xnl_msg_add_attr_uint() failed: %d", rv);
        goto free_skb;
    }

    // MD : Debug mode
    rv = xnl_msg_add_attr_uint(skb, XNL_ATTR_DEBUG_EN,
            dev_attr.debug_mode);
    if (rv < 0) {
        pr_err("xnl_msg_add_attr_uint() failed: %d", rv);
        goto free_skb;
    }

    // MD : Descriptor engine mode
    rv = xnl_msg_add_attr_uint(skb, XNL_ATTR_DESC_ENGINE_MODE,
            dev_attr.desc_eng_mode);
    if (rv < 0) {
        pr_err("xnl_msg_add_attr_uint() failed: %d", rv);
        goto free_skb;
    }

    // MD : Add entire device attributes structure as binary data
    rv = xnl_msg_add_attr_data(skb, XNL_ATTR_DEV,
            (void *)&dev_attr, sizeof(struct qdma_dev_attributes));
    if (rv != 0) {
        pr_err("xnl_msg_add_attr_data() failed: %d", rv);
        return rv;
    }

    // MD : Send the netlink message
    rv = xnl_msg_send(skb, hdr, info);
    return rv;

free_skb:
    // MD : Error handler - free socket buffer
    nlmsg_free(skb);
    return rv;
}

static int xnl_dev_stat(struct sk_buff *skb2, struct genl_info *info)
{
    // MD : Socket buffer and header for netlink message
    struct sk_buff *skb;
    void *hdr;
    // MD : Xilinx PCI device structure
    struct xlnx_pci_dev *xpdev;
    int rv;

    // MD : Packet counters for different modes (MM=Memory Mapped, ST=Stream)
    unsigned long long mmh2c_pkts = 0;  // MD : MM Host to Card packets
    unsigned long long mmc2h_pkts = 0;  // MD : MM Card to Host packets  
    unsigned long long sth2c_pkts = 0;  // MD : ST Host to Card packets
    unsigned long long stc2h_pkts = 0;  // MD : ST Card to Host packets

    // MD : Latency statistics for ping-pong measurements
    unsigned long long min_ping_pong_lat = 0;  // MD : Minimum latency
    unsigned long long max_ping_pong_lat = 0;  // MD : Maximum latency 
    unsigned long long total_ping_pong_lat = 0; // MD : Total latency
    unsigned long long avg_ping_pong_lat = 0;   // MD : Average latency
    unsigned int pkts;

    pr_debug("DEV_STAT: Entry\n");

    // MD : Validate input parameter
    if (info == NULL) {
        pr_err("DEV_STAT: info is NULL\n");
        return -EINVAL;
    }

    // MD : Debug dump netlink attributes
    xnl_dump_attrs(info);

    // MD : Get Xilinx device handle and validate
    xpdev = xnl_rcv_check_xpdev(info);
    if (!xpdev) {
        pr_err("DEV_STAT: Failed to get xpdev\n");
        return -EINVAL;
    }

    // MD : Allocate new netlink message
    skb = xnl_msg_alloc(XNL_CMD_DEV_STAT, 0, &hdr, info);
    if (!skb) {
        pr_err("DEV_STAT: Failed to allocate SK buffer\n");
        return -ENOMEM;
    }

    pr_debug("DEV_STAT: Getting device statistics\n");

    // MD : Get packet statistics from device
    qdma_device_get_mmh2c_pkts(xpdev->dev_hndl, &mmh2c_pkts);
    qdma_device_get_mmc2h_pkts(xpdev->dev_hndl, &mmc2h_pkts);
    qdma_device_get_sth2c_pkts(xpdev->dev_hndl, &sth2c_pkts);
    qdma_device_get_stc2h_pkts(xpdev->dev_hndl, &stc2h_pkts);

    // MD : Get latency statistics
    qdma_device_get_ping_pong_min_lat(xpdev->dev_hndl, &min_ping_pong_lat);
    qdma_device_get_ping_pong_max_lat(xpdev->dev_hndl, &max_ping_pong_lat);
    qdma_device_get_ping_pong_tot_lat(xpdev->dev_hndl, &total_ping_pong_lat);

    pr_debug("DEV_STAT: MM H2C pkts=%llu, MM C2H pkts=%llu\n", 
             mmh2c_pkts, mmc2h_pkts);
    pr_debug("DEV_STAT: ST H2C pkts=%llu, ST C2H pkts=%llu\n", 
             sth2c_pkts, stc2h_pkts);
    // MD : Add MM H2C packet counts (split into 32-bit chunks)
    pkts = mmh2c_pkts;
    rv = xnl_msg_add_attr_uint(skb, XNL_ATTR_DEV_STAT_MMH2C_PKTS1, pkts);
    if (rv < 0) {
        pr_err("DEV_STAT: Failed to add MMH2C pkts1 attribute: %d\n", rv);
        return rv;
    }
    
    pkts = (mmh2c_pkts >> 32);
    rv = xnl_msg_add_attr_uint(skb, XNL_ATTR_DEV_STAT_MMH2C_PKTS2, pkts);
    if (rv < 0) {
        pr_err("DEV_STAT: Failed to add MMH2C pkts2 attribute: %d\n", rv);
        return rv;
    }

    // MD : Add MM C2H packet counts
    pkts = mmc2h_pkts;
    rv = xnl_msg_add_attr_uint(skb, XNL_ATTR_DEV_STAT_MMC2H_PKTS1, pkts);
    if (rv < 0) {
        pr_err("DEV_STAT: Failed to add MMC2H pkts1 attribute: %d\n", rv);
        return rv;
    }

    pkts = (mmc2h_pkts >> 32);
    rv = xnl_msg_add_attr_uint(skb, XNL_ATTR_DEV_STAT_MMC2H_PKTS2, pkts);
    if (rv < 0) {
        pr_err("DEV_STAT: Failed to add MMC2H pkts2 attribute: %d\n", rv);
        return rv;
    }

    // MD : Add ST H2C packet counts
    pkts = sth2c_pkts;
    rv = xnl_msg_add_attr_uint(skb, XNL_ATTR_DEV_STAT_STH2C_PKTS1, pkts);
    if (rv < 0) {
        pr_err("DEV_STAT: Failed to add STH2C pkts1 d\n", rv);
        return rv;
    }

    pkts = (sth2c_pkts >> 32);
    rv = xnl_msg_add_attr_uint(skb, XNL_ATTR_DEV_STAT_STH2C_PKTS2, pkts);
    if (rv < 0) {
        pr_err("DEV_STAT: Failed to add STH2C pkts2 attribute: %d\n", rv);
        return rv;
    }

    // MD : Add ST C2H packet counts
    pkts = stc2h_pkts;
    rv = xnl_msg_add_attr_uint(skb, XNL_ATTR_DEV_STAT_STC2H_PKTS1, pkts);
    if (rv < 0) {
        pr_err("DEV_STAT: Failed to add STC2H pkts1 attribute: %d\n", rv);
        return rv;
    }

    pkts = (stc2h_pkts >> 32);
    rv = xnl_msg_add_attr_uint(skb, XNL_ATTR_DEV_STAT_STC2H_PKTS2, pkts);
    if (rv < 0) {
        pr_err("DEV_STAT: Failed to add STC2H pkts2 attribute: %d\n", rv);
        return rv;
    }

    // MD : Add ping-pong latency statistics
    pkts = min_ping_pong_lat;
    rv = xnl_msg_add_attr_uint(skb, XNL_ATTR_DEV_STAT_PING_PONG_LATMIN1, pkts);
    if (rv < 0) {
        pr_err("DEV_STAT: Failed to add min latency1 attribute: %d\n", rv);
        return rv;
    }

    // MD : Calculate average latency
    if (stc2h_pkts != 0) {
        avg_ping_pong_lat = total_ping_pong_lat / stc2h_pkts;
        pr_debug("DEV_STAT: Average ping-pong latency=%llu\n", avg_ping_pong_lat);
    } else {
        pr_err("DEV_STAT: No C2H packets to calculate average latency\n");
    }

    // MD : Add average latency values
    pkts = avg_ping_pong_lat;
    rv = xnl_msg_add_attr_uint(skb, XNL_ATTR_DEV_STAT_PING_PONG_LATAVG1, pkts);
    if (rv < 0) {
        pr_err("DEV_STAT: Failed to add avg latency1 attribute: %d\n", rv);
        return rv;
    }

    pkts = (avg_ping_pong_lat >> 32);
    rv = xnl_msg_add_attr_uint(skb, XNL_ATTR_DEV_STAT_PING_PONG_LATAVG2, pkts);
    if (rv < 0) {
        pr_err("DEV_STAT: Failed to add avg latency2 attribute: %d\n", rv);
        return rv;
    }

    pr_debug("DEV_STAT: Sending message\n");
    rv = xnl_msg_send(skb, hdr, info);

    return rv;
}

static int xnl_dev_stat_clear(struct sk_buff *skb2, struct genl_info *info)
{
    // MD : Declare a pointer to the Xilinx PCI device structure
    struct xlnx_pci_dev *xpdev;
    // MD : Variable to store return values
    int rv;
    // MD : Buffer to hold response message
    char *buf;

    // MD : Check if the info parameter is NULL, return error if true
    if (info == NULL) {
        pr_err("DEV_STAT_CLEAR: info is NULL\n");
        return -EINVAL;
    }

    // MD : Debug: Print netlink attributes for inspection
    xnl_dump_attrs(info);

    // MD : Retrieve and validate the Xilinx PCI device handle from netlink info
    xpdev = xnl_rcv_check_xpdev(info);
    if (!xpdev) {
        pr_err("DEV_STAT_CLEAR: Failed to get xpdev\n");
        return -EINVAL;
    }

    // MD : Allocate memory for the response buffer
    buf = xnl_mem_alloc(XNL_RESP_BUFLEN_MIN, info);
    if (!buf) {
        pr_err("DEV_STAT_CLEAR: Failed to allocate memory for buffer\n");
        return -ENOMEM;
    }

    // MD : Clear the device statistics using the device handle
    qdma_device_clear_stats(xpdev->dev_hndl);
    pr_debug("DEV_STAT_CLEAR: Device statistics cleared\n");

    // MD : Initialize the buffer with an empty string
    buf[0] = '\0';

    // MD : Send a response back to the user space indicating success
    rv = xnl_respond_buffer(info, buf, XNL_RESP_BUFLEN_MAX, 0);
    pr_debug("DEV_STAT_CLEAR: Response sent\n");

    // MD : Free the allocated buffer memory
    kfree(buf);
    return rv;
}

static int xnl_get_queue_state(struct sk_buff *skb2, struct genl_info *info)
{
    // MD : Declare a pointer to the Xilinx PCI device structure
    struct xlnx_pci_dev *xpdev;
    // MD : Structure to hold queue configuration
    struct qdma_queue_conf qconf;
    // MD : Buffer for error messages or responses
    char buf[XNL_RESP_BUFLEN_MIN];
    // MD : Pointer to queue data structure
    struct xlnx_qdata *qdata;
    // MD : Variable to store return values
    int rv = 0;
    // MD : Flag to indicate if the queue is a queue pair
    unsigned char is_qp;
    // MD : Flags to represent queue state
    unsigned int q_flags;
    // MD : Socket buffer for netlink message
    struct sk_buff *skb;
    // MD : Header for netlink message
    void *hdr;
    // MD : Structure to hold queue state
    struct qdma_q_state qstate;

    // MD : Check if the info parameter is NULL, return error if true
    if (info == NULL)
        return 0;

    // MD : Debug: Print netlink attributes for inspection
    xnl_dump_attrs(info);

    // MD : Allocate a new netlink message
    skb = xnl_msg_alloc(XNL_CMD_DEV_STAT, 0, &hdr, info);
    if (!skb)
        return -ENOMEM;

    // MD : Retrieve and validate the Xilinx PCI device handle from netlink info
    xpdev = xnl_rcv_check_xpdev(info);
    if (!xpdev) {
        nlmsg_free(skb); // MD : Free the socket buffer if device retrieval fails
        return -EINVAL;
    }

    // MD : Get queue configuration from netlink info
    rv = qconf_get(&qconf, info, buf, XNL_RESP_BUFLEN_MIN, &is_qp);
    if (rv < 0) {
        nlmsg_free(skb); // MD : Free the socket buffer if configuration retrieval fails
        return -EINVAL;
    }

    // MD : Check if the queue is a queue pair, return error if true
    if (is_qp)
        return -EINVAL;

    // MD : Retrieve queue data based on queue index
    qdata = xnl_rcv_check_qidx(info, xpdev, &qconf, buf, XNL_RESP_BUFLEN_MIN);
    if (!qdata) {
        nlmsg_free(skb); // MD : Free the socket buffer if queue data retrieval fails
        return -EINVAL;
    }

    // MD : Get the current state of the queue
    rv = qdma_get_queue_state(xpdev->dev_hndl, qdata->qhndl, &qstate, buf, XNL_RESP_BUFLEN_MIN);
    if (rv < 0) {
        xnl_respond_buffer(info, buf, XNL_RESP_BUFLEN_MAX, rv); // MD : Respond with error
        nlmsg_free(skb); // MD : Free the socket buffer
        pr_err("qdma_get_queue_state() failed: %d", rv); // MD : Log error
        return rv;
    }

    // MD : Initialize queue flags
    q_flags = 0;

    // MD : Set queue mode flags based on state
    if (qstate.st)
        q_flags |= XNL_F_QMODE_ST;
    else
        q_flags |= XNL_F_QMODE_MM;

    // MD : Set queue direction flags based on type
    if (qstate.q_type == Q_C2H)
        q_flags |= XNL_F_QDIR_C2H;
    else if (qstate.q_type == Q_H2C)
        q_flags |= XNL_F_QDIR_H2C;
    else
        q_flags |= XNL_F_Q_CMPL;

    // MD : Add queue flags attribute to the netlink message
    rv = xnl_msg_add_attr_uint(skb, XNL_ATTR_QFLAG, q_flags);
    if (rv < 0) {
        nlmsg_free(skb); // MD : Free the socket buffer if adding attribute fails
        pr_err("xnl_msg_add_attr_uint() failed: %d", rv); // MD : Log error
        return rv;
    }

    // MD : Add queue index attribute to the netlink message
    rv = xnl_msg_add_attr_uint(skb, XNL_ATTR_QIDX, qstate.qidx);
    if (rv < 0) {
        nlmsg_free(skb); // MD : Free the socket buffer if adding attribute fails
        pr_err("xnl_msg_add_attr_uint() failed: %d", rv); // MD : Log error
        return rv;
    }

    // MD : Add queue state attribute to the netlink message
    rv = xnl_msg_add_attr_uint(skb, XNL_ATTR_Q_STATE, qstate.qstate);
    if (rv < 0) {
        nlmsg_free(skb); // MD : Free the socket buffer if adding attribute fails
        pr_err("xnl_msg_add_attr_uint() failed: %d", rv); // MD : Log error
        return rv;
    }

    // MD : Send the netlink message
    rv = xnl_msg_send(skb, hdr, info);

    return rv;
}

static int xnl_q_list(struct sk_buff *skb2, struct genl_info *info)
{
    // MD : Declare necessary variables
    struct xlnx_pci_dev *xpdev;  // MD : Pointer to Xilinx PCI device structure
    char *buf;                   // MD : Buffer to hold response data
    int rv = 0;                  // MD : Return value
    char ebuf[XNL_RESP_BUFLEN_MIN]; // MD : Error buffer for messages
    struct qdma_queue_conf qconf;   // MD : Queue configuration structure
    unsigned char is_qp;         // MD : Flag to check if queue pair
    uint32_t qmax = 0;           // MD : Maximum number of queues
    uint32_t buflen = 0, max_buflen = 0; // MD : Buffer lengths
    struct qdma_queue_count q_count; // MD : Structure to hold queue counts
    unsigned int qidx, num_q;    // MD : Queue index and number of queues
    struct xlnx_qdata *qdata;    // MD : Pointer to queue data

    // MD : Check if the info parameter is NULL
    if (info == NULL)
        return -EINVAL;

    // MD : Debug: Print netlink attributes for inspection
    xnl_dump_attrs(info);

    // MD : Retrieve and validate the Xilinx PCI device handle from netlink info
    xpdev = xnl_rcv_check_xpdev(info);
    if (!xpdev)
        return -EINVAL;

    // MD : Allocate memory for the response buffer
    buf = xnl_mem_alloc(XNL_RESP_BUFLEN_MIN, info);
    if (!buf)
        return -ENOMEM;
    buflen = XNL_RESP_BUFLEN_MIN;

    // MD : Get queue configuration from netlink info
    rv = qconf_get(&qconf, info, ebuf, XNL_RESP_BUFLEN_MIN, &is_qp);
    if (rv < 0)
        return rv;

    // MD : Get queue index and number of queues from attributes
    qidx = qconf.qidx;
    num_q = nla_get_u32(info->attrs[XNL_ATTR_NUM_Q]);

    // MD : Retrieve the count of queues
    rv = qdma_get_queue_count(xpdev->dev_hndl, &q_count, buf, buflen);
    if (rv < 0) {
        rv += snprintf(buf, XNL_RESP_BUFLEN_MIN, "Failed to get queue count\n");
        goto send_rsp;
    }

    // MD : Calculate the maximum number of queues
    qmax = q_count.h2c_qcnt + q_count.c2h_qcnt;
    if (!qmax) {
        rv += snprintf(buf, 8, "Zero Qs\n\n");
        goto send_rsp;
    }

    // MD : Check the queue index validity
    qdata = xnl_rcv_check_qidx(info, xpdev, &qconf, buf, buflen);
    if (!qdata)
        goto send_rsp;

    // MD : Re-fetch the number of queues from attributes
    num_q = nla_get_u32(info->attrs[XNL_ATTR_NUM_Q]);

    // MD : Check if the number of queues exceeds the maximum allowed
    if (num_q > QDMA_Q_DUMP_MAX_QUEUES) {
        pr_err("Can not dump more than %d queues\n", QDMA_Q_DUMP_MAX_QUEUES);
        rv += snprintf(buf, 40, "Can not dump more than %d queues\n", QDMA_Q_DUMP_MAX_QUEUES);
        goto send_rsp;
    }

    // MD : Free the initial buffer and allocate a larger one based on the number of queues
    kfree(buf);
    max_buflen = (num_q * 2 * QDMA_Q_LIST_LINE_SZ);
    buf = xnl_mem_alloc(max_buflen, info);
    if (!buf)
        return -ENOMEM;

    buflen = max_buflen;

    // MD : List the queues and store the result in the buffer
    rv = qdma_queue_list(xpdev->dev_hndl, qidx, num_q, buf, buflen);
    if (rv < 0) {
        pr_err("qdma_queue_list() failed: %d", rv);
        goto send_rsp;
    }

send_rsp:
    // MD : Send the response buffer back to the user space
    rv = xnl_respond_buffer(info, buf, max_buflen, rv);
    kfree(buf); // MD : Free the allocated buffer
    return rv;
}

static int xnl_q_add(struct sk_buff *skb2, struct genl_info *info)
{
    // MD : Declare variables for device, queue configuration, and buffer management
    struct xlnx_pci_dev *xpdev = NULL;
    struct qdma_queue_conf qconf;
    char *buf, *cur, *end;
    int rv = 0;  // MD : Return value for function calls
    int rv2 = 0; // MD : Secondary return value for response
    unsigned char is_qp; // MD : Flag to check if queue pair
    unsigned int num_q;  // MD : Number of queues
    unsigned int i;      // MD : Loop counter
    unsigned short qidx; // MD : Queue index
    unsigned char dir;   // MD : Direction of queue
    int buf_len = XNL_RESP_BUFLEN_MAX; // MD : Buffer length

    // MD : Check if the info parameter is NULL, return error if true
    if (info == NULL)
        return -EINVAL;

    // MD : Debug: Print netlink attributes for inspection
    pr_debug("xnl_q_add: Dumping netlink attributes\n");
    xnl_dump_attrs(info);

    // MD : Retrieve and validate the Xilinx PCI device handle from netlink info
    xpdev = xnl_rcv_check_xpdev(info);
    if (!xpdev)
        return -EINVAL;

    // MD : Check if response buffer length attribute is present and update buffer length
    if (info->attrs[XNL_ATTR_RSP_BUF_LEN])
        buf_len = nla_get_u32(info->attrs[XNL_ATTR_RSP_BUF_LEN]);

    // MD : Allocate memory for the response buffer
    buf = xnl_mem_alloc(buf_len, info);
    if (!buf)
        return -ENOMEM;
    cur = buf;
    end = buf + buf_len;

    // MD : Check if maximum queue size is zero, log and respond if true
    if (unlikely(!qdma_get_qmax(xpdev->dev_hndl))) {
        pr_info("xnl_q_add: 0 sized Qs\n");
        rv += snprintf(cur, end - cur, "Zero Qs\n");
        goto send_resp;
    }

    // MD : Get queue configuration from netlink info
    rv = qconf_get(&qconf, info, cur, end - cur, &is_qp);
    if (rv < 0)
        goto free_buf;

    qidx = qconf.qidx;

    // MD : Check if the number of queues attribute is present and valid
    rv = xnl_chk_attr(XNL_ATTR_NUM_Q, info, qidx, cur, end - cur);
    if (rv < 0)
        goto send_resp;
    num_q = nla_get_u32(info->attrs[XNL_ATTR_NUM_Q]);

    // MD : Validate queue type
    if (qconf.q_type > Q_CMPT) {
        pr_err("xnl_q_add: Invalid q type received\n");
        rv += snprintf(cur, end - cur, "Invalid q type received\n");
        goto send_resp;
    }

    // MD : Set direction and iterate over the number of queues
    dir = qconf.q_type;
    for (i = 0; i < num_q; i++) {
        if (qconf.q_type != Q_CMPT)
            qconf.q_type = dir;
add_q:
        if (qidx != QDMA_QUEUE_IDX_INVALID)
            qconf.qidx = qidx + i;
        // MD : Add queue to the device
        rv = xpdev_queue_add(xpdev, &qconf, cur, end - cur);
        if (rv < 0) {
            pr_err("xnl_q_add: xpdev_queue_add() failed: %d\n", rv);
            goto send_resp;
        }
        cur = buf + strlen(buf);
        // MD : Handle queue pair direction toggling
        if (qconf.q_type != Q_CMPT) {
            if (is_qp && (dir == qconf.q_type)) {
                qconf.q_type = (~qconf.q_type) & 0x1;
                goto add_q;
            }
        }
    }
/* MD : Suppress Q additions prints if num_q's greater than 2048.
 * And print only consolidated Q's added, to overcome attr failure.
 * TODO: This is a workaround. Need to comeup with proper fix.
*/
    // MD : Handle large number of queues by suppressing individual prints
    if (num_q > 2048) {
        memset(buf, 0, strlen(buf) + 1);
        snprintf(buf, 25, "Added %u Queues.\n", i);
    } else {
        cur += snprintf(cur, end - cur, "Added %u Queues.\n", i);
    }

send_resp:
    // MD : Send response back to user space
    rv2 = xnl_respond_buffer(info, buf, strlen(buf), rv);
free_buf:
    // MD : Free allocated buffer memory
    kfree(buf);
    return rv < 0 ? rv : rv2;
}

static int xnl_q_buf_idx_get(struct xlnx_pci_dev *xpdev)
{
    // MD : Structure to hold global CSR (Control and Status Register) configuration
    struct global_csr_conf csr;
    int i, rv; // MD : Loop index and return value

    // MD : Initialize the CSR structure to zero
    memset(&csr, 0, sizeof(struct global_csr_conf));
    pr_debug("xnl_q_buf_idx_get: Initialized CSR structure to zero\n");

    // MD : Retrieve the global CSR configuration from the device
    rv = qdma_global_csr_get(xpdev->dev_hndl, 0, QDMA_GLOBAL_CSR_ARRAY_SZ, &csr);
    if (rv < 0) {
        pr_err("xnl_q_buf_idx_get: Failed to get global CSR, rv=%d\n", rv);
        return 0; // MD : Return 0 if retrieval fails
    }
    pr_debug("xnl_q_buf_idx_get: Retrieved global CSR configuration\n");

    // MD : Iterate over the CSR array to find the buffer index with default size
    for (i = 0; i < QDMA_GLOBAL_CSR_ARRAY_SZ; i++) {
        if (csr.c2h_buf_sz[i] == QDMA_C2H_DEFAULT_BUF_SZ) {
            pr_debug("xnl_q_buf_idx_get: Found default buffer size at index %d\n", i);
            return i; // MD : Return the index if default buffer size is found
        }
    }

    pr_debug("xnl_q_buf_idx_get: No default buffer size found, returning 0\n");
    return 0; // MD : Return 0 if no default buffer size is found
}

static int xnl_q_start(struct sk_buff *skb2, struct genl_info *info)
{
    // MD : Declare necessary structures and variables
    struct xlnx_pci_dev *xpdev;                // MD : Xilinx PCI device structure
    struct qdma_queue_conf qconf;              // MD : QDMA queue configuration
    struct qdma_queue_conf qconf_old;          // MD : Previous QDMA queue configuration
    char buf[XNL_RESP_BUFLEN_MIN];             // MD : Buffer for response messages
    struct xlnx_qdata *qdata;                  // MD : Queue data structure
    int rv = 0;                                // MD : Return value
    unsigned char is_qp;                       // MD : Flag for queue pair
    unsigned short num_q;                      // MD : Number of queues
    unsigned int i;                            // MD : Loop index
    unsigned short qidx;                       // MD : Queue index
    unsigned char dir;                         // MD : Queue direction
    unsigned char is_bufsz_idx = 1;            // MD : Buffer size index flag

    // MD : Check if the info parameter is NULL, return 0 if true
    if (info == NULL)
        return 0;

    // MD : Debug: Print netlink attributes for inspection
    xnl_dump_attrs(info);

    // MD : Retrieve and validate the Xilinx PCI device handle from netlink info
    xpdev = xnl_rcv_check_xpdev(info);
    if (!xpdev)
        return 0;

    // MD : Check if the maximum number of queues is zero, log and respond if true
    if (unlikely(!qdma_get_qmax(xpdev->dev_hndl))) {
        rv += snprintf(buf, 8, "Zero Qs\n");
        goto send_resp;
    }

    // MD : Get queue configuration from netlink info
    rv = qconf_get(&qconf, info, buf, XNL_RESP_BUFLEN_MIN, &is_qp);
    if (rv < 0)
        goto send_resp;

    // MD : Set the initial queue index
    qidx = qconf.qidx;

    // MD : Check for the number of queues attribute, respond if missing
    rv = xnl_chk_attr(XNL_ATTR_NUM_Q, info, qidx, buf, XNL_RESP_BUFLEN_MIN);
    if (rv < 0)
        goto send_resp;
    num_q = nla_get_u32(info->attrs[XNL_ATTR_NUM_Q]);

    // MD : Extract additional configuration attributes
    xnl_extract_extra_config_attr(info, &qconf);

    // MD : Validate queue type for MM completion mode
    if (qconf.st && (qconf.q_type == Q_CMPT)) {
        rv += snprintf(buf, 40, "MM CMPL is valid only for MM Mode");
        goto send_resp;
    }

    // MD : Check for invalid queue type
    if (qconf.q_type > Q_CMPT) {
        pr_err("Invalid q type received");
        rv += snprintf(buf, 40, "Invalid q type received");
        goto send_resp;
    }

    // MD : Check for buffer size index attribute
    if (!info->attrs[XNL_ATTR_C2H_BUFSZ_IDX])
        is_bufsz_idx = 0;

    // MD : Check for MM channel attribute
    if (!info->attrs[XNL_ATTR_MM_CHANNEL])
        qconf.mm_channel = 0;

    // MD : Set the queue direction
    dir = qconf.q_type;
    for (i = qidx; i < (qidx + num_q); i++) {
        if (qconf.q_type != Q_CMPT)
            qconf.q_type = dir;
reconfig:
        qconf.qidx = i;
        // MD : Validate queue index and retrieve queue data
        qdata = xnl_rcv_check_qidx(info, xpdev, &qconf, buf, XNL_RESP_BUFLEN_MIN);
        if (!qdata)
            goto send_resp;

        // MD : Get the current queue configuration
        rv = qdma_queue_get_config(xpdev->dev_hndl, qdata->qhndl, &qconf_old, buf, XNL_RESP_BUFLEN_MIN);
        if (rv < 0)
            goto send_resp;

        // MD : Adjust buffer size index if needed
        if (qconf.q_type != Q_CMPT) {
            if (qconf_old.st && qconf_old.q_type && !is_bufsz_idx)
                qconf.c2h_buf_sz_idx = xnl_q_buf_idx_get(xpdev);
        }

        // MD : Configure the queue with the new settings
        rv = qdma_queue_config(xpdev->dev_hndl, qdata->qhndl, &qconf, buf, XNL_RESP_BUFLEN_MIN);
        if (rv < 0) {
            pr_err("qdma_queue_config failed: %d", rv);
            goto send_resp;
        }

        // MD : Handle queue pair direction toggling
        if (qconf.q_type != Q_CMPT) {
            if (is_qp && (dir == qconf.q_type)) {
                qconf.q_type = (~qconf.q_type) & 0x1;
                goto reconfig;
            }
        }
    }

    // MD : Start the queue
    rv = xpdev_nl_queue_start(xpdev, info, is_qp, qconf.q_type, qidx, num_q);
    if (rv < 0) {
        snprintf(buf, XNL_RESP_BUFLEN_MIN, "qdma%05x OOM.\n", xpdev->idx);
        goto send_resp;
    }

    return 0;

send_resp:
    // MD : Send response back to user space
    rv = xnl_respond_buffer(info, buf, XNL_RESP_BUFLEN_MIN, rv);

    return rv;
}

static int xnl_q_stop(struct sk_buff *skb2, struct genl_info *info)
{
    // MD : Declare variables for device, queue configuration, and response buffer
    struct xlnx_pci_dev *xpdev;
    struct qdma_queue_conf qconf;
    char buf[XNL_RESP_BUFLEN_MIN];
    struct xlnx_qdata *qdata;
    int rv = 0, rv2 = 0;
    unsigned char is_qp;
    unsigned short num_q;
    unsigned int i;
    unsigned short qidx;
    unsigned char dir;

    // MD : Check if the info parameter is NULL, return 0 if true
    if (info == NULL) {
        pr_debug("xnl_q_stop: info is NULL\n");
        return 0;
    }

    // MD : Debug: Print netlink attributes for inspection
    xnl_dump_attrs(info);

    // MD : Retrieve and validate the Xilinx PCI device handle from netlink info
    xpdev = xnl_rcv_check_xpdev(info);
    if (!xpdev) {
        pr_debug("xnl_q_stop: Failed to get xpdev\n");
        return 0;
    }

    // MD : Check if the maximum number of queues is zero
    if (unlikely(!qdma_get_qmax(xpdev->dev_hndl))) {
        rv += snprintf(buf, 8, "Zero Qs\n");
        pr_debug("xnl_q_stop: Zero queues available\n");
        goto send_resp;
    }

    // MD : Get queue configuration from netlink info
    rv = qconf_get(&qconf, info, buf, XNL_RESP_BUFLEN_MIN, &is_qp);
    if (rv < 0) {
        pr_debug("xnl_q_stop: qconf_get failed\n");
        goto send_resp;
    }

    // MD : Check if the number of queues attribute is missing
    if (!info->attrs[XNL_ATTR_NUM_Q]) {
        pr_warn("xnl_q_stop: Missing attribute 'XNL_ATTR_NUM_Q'\n");
        return -1;
    }

    // MD : Validate queue type
    if (qconf.q_type > Q_CMPT) {
        pr_err("xnl_q_stop: Invalid q type received\n");
        rv += snprintf(buf, 40, "Invalid q type received");
        goto send_resp;
    }

    // MD : Get the number of queues from netlink attributes
    num_q = nla_get_u32(info->attrs[XNL_ATTR_NUM_Q]);

    // MD : Initialize queue index and direction
    qidx = qconf.qidx;
    dir = qconf.q_type;

    // MD : Iterate over each queue to stop
    for (i = qidx; i < (qidx + num_q); i++) {
        if (qconf.q_type != Q_CMPT)
            qconf.q_type = dir;
stop_q:
        qconf.qidx = i;
        // MD : Check queue index and retrieve queue data
        qdata = xnl_rcv_check_qidx(info, xpdev, &qconf, buf, XNL_RESP_BUFLEN_MIN);
        if (!qdata) {
            pr_debug("xnl_q_stop: Invalid queue index\n");
            goto send_resp;
        }

        // MD : Stop the queue
        rv = qdma_queue_stop(xpdev->dev_hndl, qdata->qhndl, buf, XNL_RESP_BUFLEN_MIN);
        if (rv < 0) {
            pr_err("xnl_q_stop: qdma_queue_stop() failed: %d\n", rv);
            goto send_resp;
        }

        // MD : Handle queue pair direction toggling
        if (qconf.q_type != Q_CMPT) {
            if (is_qp && (dir == qconf.q_type)) {
                qconf.q_type = (~qconf.q_type) & 0x1;
                goto stop_q;
            }
        }
    }

    // MD : Prepare response message indicating stopped queues
    rv2 = snprintf(buf + rv, XNL_RESP_BUFLEN_MIN - rv, "Stopped Queues %u -> %u.\n", qidx, i - 1);

send_resp:
    // MD : Send response back to user space
    rv = xnl_respond_buffer(info, buf, XNL_RESP_BUFLEN_MIN, rv);
    return rv;
}

static int xnl_q_del(struct sk_buff *skb2, struct genl_info *info)
{
    // MD : Declare variables for device, queue configuration, and response buffer
    struct xlnx_pci_dev *xpdev;
    struct qdma_queue_conf qconf;
    char buf[XNL_RESP_BUFLEN_MIN];
    int rv = 0, rv2 = 0;
    unsigned char is_qp;
    unsigned short num_q;
    unsigned int i;
    unsigned short qidx;
    unsigned char dir;

    // MD : Check if the info parameter is NULL, return 0 if true
    if (info == NULL) {
        pr_err("xnl_q_del: info is NULL\n");
        return 0;
    }

    // MD : Debug: Print netlink attributes for inspection
    xnl_dump_attrs(info);

    // MD : Retrieve and validate the Xilinx PCI device handle from netlink info
    xpdev = xnl_rcv_check_xpdev(info);
    if (!xpdev) {
        pr_err("xnl_q_del: Failed to get xpdev\n");
        return 0;
    }

    // MD : Check if the maximum number of queues is zero
    if (unlikely(!qdma_get_qmax(xpdev->dev_hndl))) {
        pr_info("xnl_q_del: Zero queues available\n");
        rv += snprintf(buf, 8, "Zero Qs\n");
        goto send_resp;
    }

    // MD : Get queue configuration from netlink info
    rv = qconf_get(&qconf, info, buf, XNL_RESP_BUFLEN_MIN, &is_qp);
    if (rv < 0) {
        pr_err("xnl_q_del: qconf_get failed\n");
        goto send_resp;
    }

    // MD : Validate queue type
    if (qconf.q_type > Q_CMPT) {
        pr_err("xnl_q_del: Invalid q type received\n");
        rv += snprintf(buf, 40, "Invalid q type received");
        goto send_resp;
    }

    // MD : Check if the number of queues attribute is present
    if (!info->attrs[XNL_ATTR_NUM_Q]) {
        pr_warn("xnl_q_del: Missing attribute 'XNL_ATTR_NUM_Q'\n");
        return -1;
    }
    // MD : Retrieve the number of queues to delete
    num_q = nla_get_u32(info->attrs[XNL_ATTR_NUM_Q]);

    // MD : Initialize queue index and direction
    qidx = qconf.qidx;
    dir = qconf.q_type;

    // MD : Loop through each queue to delete
    for (i = qidx; i < (qidx + num_q); i++) {
        if (qconf.q_type != Q_CMPT)
            qconf.q_type = dir;
del_q:
        qconf.qidx = i;
        // MD : Attempt to delete the queue
        rv = xpdev_queue_delete(xpdev, qconf.qidx, qconf.q_type, buf, XNL_RESP_BUFLEN_MIN);
        if (rv < 0) {
            pr_err("xnl_q_del: xpdev_queue_delete() failed: %d\n", rv);
            goto send_resp;
        }
        // MD : Handle queue pair direction toggling
        if (qconf.q_type != Q_CMPT) {
            if (is_qp && (dir == qconf.q_type)) {
                qconf.q_type = (~qconf.q_type) & 0x1;
                goto del_q;
            }
        }
    }

    // MD : Prepare response message indicating the range of deleted queues
    rv2 = snprintf(buf + rv, XNL_RESP_BUFLEN_MIN - rv, "Deleted Queues %u -> %u.\n", qidx, i - 1);

send_resp:
    // MD : Send response back to user space
    rv = xnl_respond_buffer(info, buf, XNL_RESP_BUFLEN_MIN, rv);
    return rv;
}

static int xnl_config_reg_dump(struct sk_buff *skb2, struct genl_info *info)
{
    // MD : Declare a pointer to the Xilinx PCI device structure
    struct xlnx_pci_dev *xpdev;
    // MD : Buffer to hold the response message
    char *buf;
    // MD : Default buffer length for response
    int buf_len = XNL_RESP_BUFLEN_MAX;
    // MD : Variable to store return values
    int rv = 0;

    // MD : Check if the info parameter is NULL, return 0 if true
    if (info == NULL) {
        pr_err("CONFIG_REG_DUMP: info is NULL\n");
        return 0;
    }

    // MD : Debug: Print netlink attributes for inspection
    xnl_dump_attrs(info);

    // MD : Retrieve and validate the Xilinx PCI device handle from netlink info
    xpdev = xnl_rcv_check_xpdev(info);
    if (!xpdev) {
        pr_err("CONFIG_REG_DUMP: Failed to get xpdev\n");
        return 0;
    }

    // MD : Check if a custom response buffer length is provided
    if (info->attrs[XNL_ATTR_RSP_BUF_LEN])
        buf_len = nla_get_u32(info->attrs[XNL_ATTR_RSP_BUF_LEN]);

    // MD : Allocate memory for the response buffer
    buf = xnl_mem_alloc(buf_len, info);
    if (!buf) {
        pr_err("CONFIG_REG_DUMP: Failed to allocate memory for buffer\n");
        return -ENOMEM;
    }

    // MD : Dump the configuration registers into the buffer
    qdma_config_reg_dump(xpdev->dev_hndl, buf, buf_len);
    pr_debug("CONFIG_REG_DUMP: Configuration registers dumped\n");

    // MD : Send the response buffer back to the user space
    rv = xnl_respond_buffer(info, buf, buf_len, rv);
    pr_debug("CONFIG_REG_DUMP: Response sent\n");

    // MD : Free the allocated buffer memory
    kfree(buf);
    return rv;
}

static int xnl_config_reg_info_dump(struct sk_buff *skb2, struct genl_info *info)
{
    // MD : Declare a pointer to the Xilinx PCI device structure
    struct xlnx_pci_dev *xpdev;
    // MD : Buffer to hold the response message
    char *buf;
    // MD : Default buffer length for response
    int buf_len = XNL_RESP_BUFLEN_MAX;
    // MD : Variable to store return values
    int rv = 0;
    // MD : Variables to store register address and number of registers
    uint32_t reg_addr = 0;
    uint32_t num_regs = 0;

    // MD : Check if the info parameter is NULL, return 0 if true
    if (info == NULL) {
        pr_err("CONFIG_REG_INFO_DUMP: info is NULL\n");
        return 0;
    }

    // MD : Debug: Print netlink attributes for inspection
    xnl_dump_attrs(info);

    // MD : Retrieve and validate the Xilinx PCI device handle from netlink info
    xpdev = xnl_rcv_check_xpdev(info);
    if (!xpdev) {
        pr_err("CONFIG_REG_INFO_DUMP: Failed to get xpdev\n");
        return 0;
    }

    // MD : Check if response buffer length attribute is present and update buf_len
    if (info->attrs[XNL_ATTR_RSP_BUF_LEN])
        buf_len = nla_get_u32(info->attrs[XNL_ATTR_RSP_BUF_LEN]);

    // MD : Check if register address attribute is present and update reg_addr
    if (info->attrs[XNL_ATTR_REG_ADDR])
        reg_addr = nla_get_u32(info->attrs[XNL_ATTR_REG_ADDR]);

    // MD : Check if number of registers attribute is present and update num_regs
    if (info->attrs[XNL_ATTR_NUM_REGS])
        num_regs = nla_get_u32(info->attrs[XNL_ATTR_NUM_REGS]);

    // MD : Allocate memory for the response buffer
    buf = xnl_mem_alloc(buf_len, info);
    if (!buf) {
        pr_err("CONFIG_REG_INFO_DUMP: Failed to allocate memory for buffer\n");
        return -ENOMEM;
    }

    // MD : Dump the configuration register information into the buffer
    qdma_config_reg_info_dump(xpdev->dev_hndl, reg_addr, num_regs, buf, buf_len);

    // MD : Send the response buffer back to the user space
    rv = xnl_respond_buffer(info, buf, buf_len, rv);

    // MD : Free the allocated buffer memory
    kfree(buf);
    return rv;
}

static int xnl_q_dump(struct sk_buff *skb2, struct genl_info *info)
{
    // MD : Declare necessary structures and variables
    struct xlnx_pci_dev *xpdev;  // MD : Xilinx PCI device structure
    struct qdma_queue_conf qconf; // MD : QDMA queue configuration
    struct xlnx_qdata *qdata;    // MD : Queue data structure
    char *buf;                   // MD : Buffer for storing dump information
    char ebuf[XNL_RESP_BUFLEN_MIN]; // MD : Error buffer for messages
    int rv;                      // MD : Return value
    unsigned char is_qp;         // MD : Flag for queue pair
    unsigned int num_q;          // MD : Number of queues to dump
    unsigned int i;              // MD : Loop index
    unsigned short qidx;         // MD : Queue index
    unsigned char dir;           // MD : Queue direction
    int buf_len = XNL_RESP_BUFLEN_MAX; // MD : Buffer length
    unsigned int buf_idx = 0;    // MD : Buffer index for writing
    char banner[DUMP_LINE_SZ];   // MD : Banner for formatting output

    // MD : Check if the info parameter is NULL
    if (info == NULL)
        return 0;

    // MD : Debug: Print netlink attributes for inspection
    xnl_dump_attrs(info);

    // MD : Retrieve and validate the Xilinx PCI device handle
    xpdev = xnl_rcv_check_xpdev(info);
    if (!xpdev)
        return 0;

    // MD : Get queue configuration from netlink info
    rv = qconf_get(&qconf, info, ebuf, XNL_RESP_BUFLEN_MIN, &is_qp);
    if (rv < 0)
        return rv;

    // MD : Check if response buffer length attribute is present
    if (info->attrs[XNL_ATTR_RSP_BUF_LEN])
        buf_len = nla_get_u32(info->attrs[XNL_ATTR_RSP_BUF_LEN]);

    // MD : Allocate memory for the buffer
    buf = xnl_mem_alloc(buf_len, info);
    if (!buf)
        return -ENOMEM;

    // MD : Validate queue type
    if (qconf.q_type > Q_CMPT) {
        pr_err("Invalid q type received");
        rv += snprintf(buf, 40, "Invalid q type received");
        goto send_resp;
    }

    // MD : Check if number of queues attribute is present
    if (!info->attrs[XNL_ATTR_NUM_Q]) {
        pr_warn("Missing attribute 'XNL_ATTR_NUM_Q'");
        kfree(buf);
        return -1;
    }
    num_q = nla_get_u32(info->attrs[XNL_ATTR_NUM_Q]);

    // MD : Check if the number of queues exceeds the maximum allowed
    if (num_q > QDMA_Q_DUMP_MAX_QUEUES) {
        pr_err("Can not dump more than %d queues\n", QDMA_Q_DUMP_MAX_QUEUES);
        rv += snprintf(buf, 40, "Can not dump more than %d queues\n", QDMA_Q_DUMP_MAX_QUEUES);
        goto send_resp;
    }

    // MD : Free the initial buffer and allocate a new one based on the number of queues
    kfree(buf);
    buf_len = (num_q * QDMA_Q_DUMP_LINE_SZ);
    buf = xnl_mem_alloc(buf_len, info);
    if (!buf)
        return -ENOMEM;

    // MD : Initialize queue index and direction
    qidx = qconf.qidx;
    dir = qconf.q_type;

    // MD : Create a banner for formatting the output
    for (i = 0; i < DUMP_LINE_SZ - 5; i++)
        snprintf(banner + i, DUMP_LINE_SZ - 5, "*");

    // MD : Iterate over each queue and dump its information
    for (i = qidx; i < (qidx + num_q); i++) {
        if (qconf.q_type != Q_CMPT)
            qconf.q_type = dir;

    dump_q:
        qconf.qidx = i;
        qdata = xnl_rcv_check_qidx(info, xpdev, &qconf, buf + buf_idx, buf_len - buf_idx);
        if (!qdata)
            goto send_resp;

        // MD : Add banner and queue information to the buffer
        buf_idx += snprintf(buf + buf_idx, DUMP_LINE_SZ, "\n%s", banner);
        buf_idx += snprintf(buf + buf_idx, buf_len - buf_idx,
#ifndef __QDMA_VF__
                "\n%40s qdma%05x %s QID# %u\n",
#else
                "\n%40s qdmavf%05x %s QID# %u\n",
#endif
                "Context Dump for", xpdev->idx, q_type_list[qconf.q_type].name, qconf.qidx);
        buf_idx += snprintf(buf + buf_idx, buf_len - buf_idx, "\n%s\n", banner);

        // MD : Dump the queue information
        rv = qdma_queue_dump(xpdev->dev_hndl, qdata->qhndl, buf + buf_idx, buf_len - buf_idx);
        buf_idx = strlen(buf);
        if (rv < 0) {
            pr_err("qdma_queue_dump() failed: %d", rv);
            goto send_resp;
        }

        // MD : Handle queue pair dumping
        if (qconf.q_type != Q_CMPT) {
            if (is_qp && (dir == qconf.q_type)) {
                qconf.q_type = (~qconf.q_type) & 0x1;
                goto dump_q;
            }
        }
    }

    // MD : Add final message to the buffer
    rv = snprintf(buf + buf_idx, buf_len - buf_idx, "Dumped Queues %u -> %u.\n", qidx, i - 1);
    buf_idx += rv;

send_resp:
    // MD : Send the response buffer back to the user space
    rv = xnl_respond_buffer(info, buf, buf_len, rv);
    kfree(buf); // MD : Free the allocated buffer
    return rv;
}

static int xnl_q_dump_desc(struct sk_buff *skb2, struct genl_info *info)
{
    // MD : Declare variables for device, queue configuration, and data
    struct xlnx_pci_dev *xpdev;
    struct qdma_queue_conf qconf;
    struct xlnx_qdata *qdata;
    u32 v1; // MD : Start range for descriptor dump
    u32 v2; // MD : End range for descriptor dump
    char *buf; // MD : Buffer for response
    char ebuf[XNL_RESP_BUFLEN_MIN]; // MD : Error buffer
    int rv; // MD : Return value
    unsigned char is_qp; // MD : Flag for queue pair
    unsigned int num_q; // MD : Number of queues
    unsigned int i; // MD : Loop index
    unsigned short qidx; // MD : Queue index
    unsigned char dir; // MD : Queue direction
    int buf_len = XNL_RESP_BUFLEN_MAX; // MD : Buffer length
    unsigned int buf_idx = 0; // MD : Buffer index

    // MD : Check if info is NULL, return 0 if true
    if (info == NULL)
        return 0;

    // MD : Debug: Print netlink attributes for inspection
    pr_debug("xnl_q_dump_desc: Dumping netlink attributes\n");
    xnl_dump_attrs(info);

    // MD : Retrieve start and end range for descriptor dump
    v1 = nla_get_u32(info->attrs[XNL_ATTR_RANGE_START]);
    v2 = nla_get_u32(info->attrs[XNL_ATTR_RANGE_END]);

    // MD : Retrieve and validate the Xilinx PCI device handle
    xpdev = xnl_rcv_check_xpdev(info);
    if (!xpdev)
        return 0;

    // MD : Check if maximum queue count is zero, return 0 if true
    if (unlikely(!qdma_get_qmax(xpdev->dev_hndl)))
        return 0;

    // MD : Get queue configuration
    rv = qconf_get(&qconf, info, ebuf, XNL_RESP_BUFLEN_MIN, &is_qp);
    if (rv < 0)
        return rv;

    // MD : Check if response buffer length attribute is present
    if (info->attrs[XNL_ATTR_RSP_BUF_LEN])
        buf_len = nla_get_u32(info->attrs[XNL_ATTR_RSP_BUF_LEN]);

    // MD : Allocate memory for the response buffer
    buf = xnl_mem_alloc(buf_len, info);
    if (!buf) {
        rv = snprintf(ebuf, XNL_RESP_BUFLEN_MIN, "%s OOM %d.\n", __func__, buf_len);
        xnl_respond_buffer(info, ebuf, XNL_RESP_BUFLEN_MIN, rv);
        return -ENOMEM;
    }

    // MD : Check if number of queues attribute is present
    if (!info->attrs[XNL_ATTR_NUM_Q]) {
        pr_warn("Missing attribute 'XNL_ATTR_NUM_Q'");
        return -1;
    }
    num_q = nla_get_u32(info->attrs[XNL_ATTR_NUM_Q]);

    // MD : Validate queue type
    if (qconf.q_type > Q_CMPT) {
        pr_err("Invalid q type received");
        rv += snprintf(buf, 40, "Invalid q type received");
        goto send_resp;
    }

    // MD : Set initial queue index and direction
    qidx = qconf.qidx;
    dir = qconf.q_type;

    // MD : Iterate over each queue
    for (i = qidx; i < (qidx + num_q); i++) {
        qconf.q_type = dir;
    dump_q:
        qconf.qidx = i;
        // MD : Check and retrieve queue data
        qdata = xnl_rcv_check_qidx(info, xpdev, &qconf, buf + buf_idx, buf_len - buf_idx);
        if (!qdata)
            goto send_resp;

        // MD : Dump queue descriptors
        rv = qdma_queue_dump_desc(xpdev->dev_hndl, qdata->qhndl, v1, v2, buf + buf_idx, buf_len - buf_idx);
        buf_idx = strlen(buf);

        // MD : Check for errors in dumping descriptors
        if (rv < 0) {
            pr_err("qdma_queue_dump_desc() failed: %d", rv);
            goto send_resp;
        }

        // MD : Handle queue pair direction change
        if (is_qp && (dir == qconf.q_type)) {
            qconf.q_type = (~qconf.q_type) & 0x1;
            goto dump_q;
        }
    }

    // MD : Add final message to buffer
    rv = snprintf(buf + buf_idx, buf_len - buf_idx, "Dumped descs of queues %u -> %u.\n", qidx, i - 1);

send_resp:
    // MD : Send response buffer back to user space
    rv = xnl_respond_buffer(info, buf, buf_len, rv);

    // MD : Free allocated buffer memory
    kfree(buf);
    return rv;
}

static int xnl_q_dump_cmpt(struct sk_buff *skb2, struct genl_info *info)
{
    // MD : Declare variables for device, queue configuration, and data structures
    struct xlnx_pci_dev *xpdev;
    struct qdma_queue_conf qconf;
    struct xlnx_qdata *qdata;
    u32 v1; // MD : Start range for dumping
    u32 v2; // MD : End range for dumping
    char *buf; // MD : Buffer for response
    char ebuf[XNL_RESP_BUFLEN_MIN]; // MD : Error buffer
    int rv; // MD : Return value
    unsigned char is_qp; // MD : Flag for queue pair
    unsigned int num_q; // MD : Number of queues
    unsigned int i; // MD : Loop index
    unsigned short qidx; // MD : Queue index
    unsigned char dir; // MD : Queue direction
    int buf_len = XNL_RESP_BUFLEN_MAX; // MD : Buffer length
    unsigned int buf_idx = 0; // MD : Buffer index

    // MD : Check if info is NULL and return 0 if true
    if (info == NULL)
        return 0;

    // MD : Debug: Print netlink attributes for inspection
    xnl_dump_attrs(info);

    // MD : Retrieve range start and end from netlink attributes
    v1 = nla_get_u32(info->attrs[XNL_ATTR_RANGE_START]);
    v2 = nla_get_u32(info->attrs[XNL_ATTR_RANGE_END]);

    // MD : Retrieve and validate the Xilinx PCI device handle
    xpdev = xnl_rcv_check_xpdev(info);
    if (!xpdev)
        return 0;

    // MD : Check if maximum queue count is zero
    if (unlikely(!qdma_get_qmax(xpdev->dev_hndl)))
        return 0;

    // MD : Get queue configuration
    rv = qconf_get(&qconf, info, ebuf, XNL_RESP_BUFLEN_MIN, &is_qp);
    if (rv < 0)
        return rv;

    // MD : Check if response buffer length attribute is present
    if (info->attrs[XNL_ATTR_RSP_BUF_LEN])
        buf_len = nla_get_u32(info->attrs[XNL_ATTR_RSP_BUF_LEN]);

    // MD : Allocate memory for the response buffer
    buf = xnl_mem_alloc(buf_len, info);
    if (!buf) {
        rv = snprintf(ebuf, XNL_RESP_BUFLEN_MIN, "%s OOM %d.\n", __func__, buf_len);
        xnl_respond_buffer(info, ebuf, XNL_RESP_BUFLEN_MIN, rv);
        return -ENOMEM;
    }

    // MD : Check if number of queues attribute is present
    if (!info->attrs[XNL_ATTR_NUM_Q]) {
        pr_warn("Missing attribute 'XNL_ATTR_NUM_Q'");
        return -1;
    }
    num_q = nla_get_u32(info->attrs[XNL_ATTR_NUM_Q]);

    // MD : Validate queue type
    if (qconf.q_type > Q_CMPT) {
        pr_err("Invalid q type received");
        rv += snprintf(buf, 40, "Invalid q type received");
        goto send_resp;
    }

    // MD : Initialize queue index and direction
    qidx = qconf.qidx;
    dir = qconf.q_type;

    // MD : Iterate over the queues
    for (i = qidx; i < (qidx + num_q); i++) {
        if (qconf.q_type != Q_CMPT)
            qconf.q_type = dir;
dump_q:
        qconf.qidx = i;
        // MD : Check and retrieve queue data
        qdata = xnl_rcv_check_qidx(info, xpdev, &qconf, buf + buf_idx, buf_len - buf_idx);
        if (!qdata)
            goto send_resp;

        // MD : Dump queue completion descriptors
        rv = qdma_queue_dump_cmpt(xpdev->dev_hndl, qdata->qhndl, v1, v2, buf + buf_idx, buf_len - buf_idx);
        buf_idx = strlen(buf);
        if (rv < 0) {
            pr_err("qdma_queue_dump_cmpt() failed: %d", rv);
            goto send_resp;
        }

        // MD : Handle queue pair direction change
        if (qconf.q_type != Q_CMPT) {
            if (is_qp && (dir == qconf.q_type)) {
                qconf.q_type = (~qconf.q_type) & 0x1;
                goto dump_q;
            }
        }
    }

    // MD : Append queue dump information to buffer
    rv = snprintf(buf + buf_idx, buf_len - buf_idx, "Dumped descs of queues %u -> %u.\n", qidx, i - 1);
    buf_idx += rv;

send_resp:
    // MD : Send response buffer back to user space
    rv = xnl_respond_buffer(info, buf, buf_len, rv);

    // MD : Free allocated buffer memory
    kfree(buf);
    return rv;
}

static int xnl_q_read_udd(struct sk_buff *skb2, struct genl_info *info)
{
    // MD : Initialize return value
    int rv = 0;
    // MD : Declare queue configuration structure
    struct qdma_queue_conf qconf;
    // MD : Pointer for response buffer
    char *buf;
    // MD : Flag to indicate if queue pair
    unsigned char is_qp;
    // MD : Xilinx PCI device structure
    struct xlnx_pci_dev *xpdev;
    // MD : Queue data structure
    struct xlnx_qdata *qdata;
    // MD : Set buffer length to maximum response buffer length
    int buf_len = XNL_RESP_BUFLEN_MAX;

    // MD : Check if info is NULL, return error if true
    if (info == NULL)
        return -EINVAL;

    // MD : Debug: Print netlink attributes for inspection
    xnl_dump_attrs(info);

    // MD : Allocate memory for the response buffer
    buf = xnl_mem_alloc(XNL_RESP_BUFLEN_MAX, info);
    if (!buf)
        return -ENOMEM;

    // MD : Retrieve and validate the Xilinx PCI device handle
    xpdev = xnl_rcv_check_xpdev(info);
    if (!xpdev) {
        kfree(buf);
        return -EINVAL;
    }

    // MD : Get queue configuration from netlink info
    rv = qconf_get(&qconf, info, buf, XNL_RESP_BUFLEN_MAX, &is_qp);
    if (rv < 0)
        goto send_resp;

    // MD : Check and retrieve queue data using queue index
    qdata = xnl_rcv_check_qidx(info, xpdev, &qconf, buf, XNL_RESP_BUFLEN_MAX);
    if (!qdata)
        goto send_resp;

    // MD : Read user-defined data from the completion queue
    rv = qdma_descq_get_cmpt_udd(xpdev->dev_hndl, qdata->qhndl, buf, XNL_RESP_BUFLEN_MAX);
    if (rv < 0)
        goto send_resp;

send_resp:
    // MD : Send response buffer back to user space
    rv = xnl_respond_buffer(info, buf, buf_len, rv);

    // MD : Free allocated buffer memory
    kfree(buf);
    return rv;
}

static int xnl_q_cmpt_read(struct sk_buff *skb2, struct genl_info *info)
{
    // MD : Initialize return and error values
    int rv = 0, err = 0;
    // MD : Declare queue configuration structure
    struct qdma_queue_conf qconf;
    // MD : Pointer for response buffer
    char *buf = NULL;
    // MD : Flag to indicate if queue pair
    unsigned char is_qp = 0;
    // MD : Xilinx PCI device structure
    struct xlnx_pci_dev *xpdev = NULL;
    // MD : Queue data structure
    struct xlnx_qdata *qdata = NULL;
    // MD : Set buffer length to maximum response buffer length
    int buf_len = XNL_RESP_BUFLEN_MAX;
    // MD : Variables for completion entries
    u32 num_entries = 0;
    u8 *cmpt_entries = NULL, *cmpt_entry_list = NULL;
    u32 cmpt_entry_len = 0;
    u32 count = 0, diff_len = 0;
    // MD : Additional queue configuration attributes
    struct qdma_queue_conf qconf_attr;

    // MD : Check if info is NULL, return error if true
    if (info == NULL)
        return -EINVAL;

    // MD : Debug: Print netlink attributes for inspection
    xnl_dump_attrs(info);

    // MD : Retrieve and validate the Xilinx PCI device handle
    xpdev = xnl_rcv_check_xpdev(info);
    if (!xpdev)
        return -EINVAL;

    // MD : Check if response buffer length attribute is set, update buffer length
    if (info->attrs[XNL_ATTR_RSP_BUF_LEN])
        buf_len = nla_get_u32(info->attrs[XNL_ATTR_RSP_BUF_LEN]);

    // MD : Allocate memory for the response buffer
    buf = xnl_mem_alloc(buf_len, info);
    if (!buf)
        return -ENOMEM;

    // MD : Get queue configuration from netlink info
    rv = qconf_get(&qconf, info, buf, buf_len, &is_qp);
    if (rv < 0)
        goto send_resp;

    // MD : Set queue type to completion
    qconf.q_type = Q_CMPT;
    // MD : Check and retrieve queue data using queue index
    qdata = xnl_rcv_check_qidx(info, xpdev, &qconf, buf, buf_len);
    if (!qdata)
        goto send_resp;

    // MD : Get queue configuration attributes
    rv = qdma_queue_get_config(xpdev->dev_hndl, qdata->qhndl, &qconf_attr, buf, buf_len);
    if (rv < 0)
        goto send_resp;

    // MD : Read completion data from the descriptor queue
    rv = qdma_descq_read_cmpt_data(xpdev->dev_hndl, qdata->qhndl, &num_entries, &cmpt_entries, buf, buf_len);
    if (rv < 0)
        goto free_cmpt;

    // MD : If there are entries, process them
    if (num_entries != 0) {
        memset(buf, '\0', buf_len);
        cmpt_entry_list = cmpt_entries;
        cmpt_entry_len = 8 << qconf_attr.cmpl_desc_sz;
        for (count = 0; count < num_entries; count++) {
            // MD : Dump completion entry to buffer
            hex_dump_to_buffer(cmpt_entry_list, cmpt_entry_len, 32, 4, buf + diff_len, buf_len - diff_len, 0);
            diff_len = strlen(buf);
            if (cmpt_entry_len > 32) {
                diff_len += snprintf(buf + diff_len, buf_len - diff_len, " ");
                hex_dump_to_buffer(cmpt_entry_list + 32, cmpt_entry_len, 32, 4, buf + diff_len, buf_len - diff_len, 0);
                diff_len = strlen(buf);
            }
            buf[diff_len++] = '\n';
            cmpt_entry_list += cmpt_entry_len;
        }
    }

free_cmpt:
    // MD : Free completion entries memory
    kfree(cmpt_entries);
send_resp:
    // MD : Send response buffer back to user space
    err = rv;
    rv = xnl_respond_buffer_cmpt(info, buf, buf_len, err, num_entries);
    // MD : Free allocated buffer memory
    kfree(buf);
    return rv;
}

#ifdef ERR_DEBUG
static int xnl_err_induce(struct sk_buff *skb2, struct genl_info *info)
{
    // MD : Declare pointers and variables for device, queue configuration, and error handling
    struct xlnx_pci_dev *xpdev;  // MD : Xilinx PCI device structure
    struct qdma_queue_conf qconf; // MD : Queue configuration structure
    struct xlnx_qdata *qdata;    // MD : Queue data structure
    char *buf;                   // MD : Buffer for response message
    char ebuf[XNL_RESP_BUFLEN_MIN]; // MD : Buffer for error messages
    unsigned char is_qp;         // MD : Flag to indicate if queue pair
    int rv;                      // MD : Return value
    u32 err;                     // MD : Error code

    // MD : Check if the info parameter is NULL, return 0 if true
    if (info == NULL)
        return 0;

    // MD : Debug: Print netlink attributes for inspection
    xnl_dump_attrs(info);

    // MD : Retrieve and validate the Xilinx PCI device handle from netlink info
    xpdev = xnl_rcv_check_xpdev(info);
    if (!xpdev)
        return 0;

    // MD : Get queue configuration from netlink info
    rv = qconf_get(&qconf, info, ebuf, XNL_RESP_BUFLEN_MIN, &is_qp);
    if (rv < 0)
        return rv;

    // MD : Allocate memory for the response buffer
    buf = xnl_mem_alloc(XNL_RESP_BUFLEN_MAX, info);
    if (!buf) {
        // MD : Handle out-of-memory error
        rv = snprintf(ebuf, XNL_RESP_BUFLEN_MIN, "%s OOM %d.\n",
                      __func__, XNL_RESP_BUFLEN_MAX);
        xnl_respond_buffer(info, ebuf, XNL_RESP_BUFLEN_MIN, rv);
        return -ENOMEM;
    }

    // MD : Check and validate queue index
    qdata = xnl_rcv_check_qidx(info, xpdev, &qconf, buf, XNL_RESP_BUFLEN_MIN);
    if (!qdata)
        goto send_resp;

    // MD : Retrieve error information from netlink attributes
    err = nla_get_u32(info->attrs[XNL_ATTR_QPARAM_ERR_INFO]);

    // MD : Induce error in the queue and check for success
    if (qdma_queue_set_err_induction(xpdev->dev_hndl, qdata->qhndl, err,
                                     buf, XNL_RESP_BUFLEN_MAX)) {
        // MD : Log failure to induce error
        rv += snprintf(buf + rv, XNL_RESP_BUFLEN_MAX,
                       "Failed to set induce err\n");
        goto send_resp;
    }

    // MD : Log successful error induction
    rv += snprintf(buf + rv, XNL_RESP_BUFLEN_MAX,
                   "queue error induced\n");

send_resp:
    // MD : Send response back to user space
    rv = xnl_respond_buffer(info, buf, XNL_RESP_BUFLEN_MAX, rv);

    // MD : Free allocated buffer memory
    kfree(buf);
    return rv;
}
#endif

static int xnl_q_read_pkt(struct sk_buff *skb2, struct genl_info *info)
{
#if 0
    // MD : Declare necessary structures and variables
    struct xlnx_pci_dev *xpdev;          // MD : Xilinx PCI device structure
    struct qdma_queue_conf qconf;        // MD : QDMA queue configuration
    struct xlnx_qdata *qdata;            // MD : Xilinx queue data
    char *buf;                           // MD : Buffer for response
    char ebuf[XNL_RESP_BUFLEN_MIN];      // MD : Error buffer
    int rv;                              // MD : Return value
    unsigned char is_qp;                 // MD : Flag for queue pair
    unsigned int num_q;                  // MD : Number of queues
    unsigned int i;                      // MD : Loop index
    unsigned short qidx;                 // MD : Queue index
    int buf_len = XNL_RESP_BUFLEN_MAX;   // MD : Buffer length

    // MD : Check if the info parameter is NULL, return 0 if true
    if (info == NULL)
        return 0;

    // MD : Debug: Print netlink attributes for inspection
    xnl_dump_attrs(info);

    // MD : Retrieve and validate the Xilinx PCI device handle from netlink info
    xpdev = xnl_rcv_check_xpdev(info);
    if (!xpdev)
        return 0;

    // MD : Get queue configuration from netlink info
    rv = qconf_get(&qconf, info, ebuf, XNL_RESP_BUFLEN_MIN, &is_qp);
    if (rv < 0)
        return rv;

    // MD : Check if response buffer length attribute is present
    if (info->attrs[XNL_ATTR_RSP_BUF_LEN])
        buf_len = nla_get_u32(info->attrs[XNL_ATTR_RSP_BUF_LEN]);

    // MD : Allocate memory for the response buffer
    buf = xnl_mem_alloc(buf_len, info);
    if (!buf) {
        // MD : Log error if memory allocation fails
        rv = snprintf(ebuf, XNL_RESP_BUFLEN_MIN, "%s OOM %d.\n", __func__, buf_len);
        xnl_respond_buffer(info, ebuf, XNL_RESP_BUFLEN_MIN, rv);
        return -ENOMEM;
    }

    // MD : Check if the number of queues attribute is present
    if (!info->attrs[XNL_ATTR_NUM_Q]) {
        pr_warn("Missing attribute 'XNL_ATTR_NUM_Q'");
        return -1;
    }
    num_q = nla_get_u32(info->attrs[XNL_ATTR_NUM_Q]);

    // MD : Iterate over each queue and dump RX packets
    qidx = qconf.qidx;
    for (i = qidx; i < (qidx + num_q); i++) {
        qconf.q_type = 1;  // MD : Set queue type
        qconf.qidx = i;    // MD : Set current queue index
        qdata = xnl_rcv_check_qidx(info, xpdev, &qconf, buf, buf_len);
        if (!qdata)
            goto send_resp;

        // MD : Dump RX packets for the current queue
        rv = qdma_queue_dump_rx_packet(xpdev->dev_hndl, qdata->qhndl, buf, buf_len);
        if (rv < 0) {
            pr_err("qdma_queue_dump_rx_packet() failed: %d", rv);
            goto send_resp;
        }
    }

send_resp:
    // MD : Send response buffer back to user space
    rv = xnl_respond_buffer(info, buf, buf_len, rv);

    // MD : Free the allocated buffer memory
    kfree(buf);
    return rv;
#endif
    // MD : Inform that the function is not supported
    pr_info("NOT supported.\n");
    return -EINVAL;
}

static int xnl_intr_ring_dump(struct sk_buff *skb2, struct genl_info *info)
{
    // MD : Declare a pointer to the Xilinx PCI device structure
    struct xlnx_pci_dev *xpdev;
    // MD : Buffer to hold response message
    char *buf;
    // MD : Variables to store interrupt vector index and range
    unsigned int vector_idx = 0;
    int start_idx = 0, end_idx = 0;
    // MD : Variable to store return values
    int rv = 0;
    // MD : Default buffer length for response
    int buf_len = XNL_RESP_BUFLEN_MAX;

    // MD : Check if the info parameter is NULL, return error if true
    if (info == NULL) {
        pr_err("INTR_RING_DUMP: info is NULL\n");
        return 0;
    }

    // MD : Debug: Print netlink attributes for inspection
    xnl_dump_attrs(info);

    // MD : Retrieve and validate the Xilinx PCI device handle from netlink info
    xpdev = xnl_rcv_check_xpdev(info);
    if (!xpdev) {
        pr_err("INTR_RING_DUMP: Failed to get xpdev\n");
        return 0;
    }

    // MD : Check for the presence of the interrupt vector index attribute
    if (!info->attrs[XNL_ATTR_INTR_VECTOR_IDX]) {
        pr_warn("INTR_RING_DUMP: Missing attribute 'XNL_ATTR_INTR_VECTOR_IDX'\n");
        return -1;
    }

    // MD : Retrieve interrupt vector index and range from netlink attributes
    vector_idx = nla_get_u32(info->attrs[XNL_ATTR_INTR_VECTOR_IDX]);
    start_idx = nla_get_u32(info->attrs[XNL_ATTR_INTR_VECTOR_START_IDX]);
    end_idx = nla_get_u32(info->attrs[XNL_ATTR_INTR_VECTOR_END_IDX]);

    // MD : Check if a custom response buffer length is specified
    if (info->attrs[XNL_ATTR_RSP_BUF_LEN])
        buf_len = nla_get_u32(info->attrs[XNL_ATTR_RSP_BUF_LEN]);

    // MD : Allocate memory for the response buffer
    buf = xnl_mem_alloc(buf_len, info);
    if (!buf) {
        pr_err("INTR_RING_DUMP: Failed to allocate memory for buffer\n");
        return -ENOMEM;
    }

    // MD : Check device index and vector index to determine interrupt type
    if (xpdev->idx == 0) {
        if (vector_idx == 0) {
            rv += snprintf(buf + rv, buf_len,
                "vector_idx %u is for error interrupt\n", vector_idx);
            goto send_resp;
        } else if (vector_idx == 1) {
            rv += snprintf(buf + rv, buf_len,
                "vector_idx %u is for user interrupt\n", vector_idx);
            goto send_resp;
        }
    } else {
        if (vector_idx == 0) {
            rv += snprintf(buf + rv, buf_len,
                "vector_idx %u is for user interrupt\n", vector_idx);
            goto send_resp;
        }
    }

    // MD : Perform the interrupt ring dump operation
    rv = qdma_intr_ring_dump(xpdev->dev_hndl, vector_idx, start_idx, end_idx, buf, buf_len);
    if (rv < 0) {
        pr_err("INTR_RING_DUMP: qdma_intr_ring_dump() failed: %d\n", rv);
        goto send_resp;
    }

send_resp:
    // MD : Send the response buffer back to user space
    rv = xnl_respond_buffer(info, buf, buf_len, rv);

    // MD : Free the allocated buffer memory
    kfree(buf);
    return rv;
}

static int xnl_register_read(struct sk_buff *skb2, struct genl_info *info)
{
    // MD : Declare variables for socket buffer, message header, and device structures
    struct sk_buff *skb;
    void *hdr;
    struct xlnx_pci_dev *xpdev;
    struct qdma_dev_conf conf;
    char buf[XNL_RESP_BUFLEN_MIN]; // MD : Buffer for response message
    unsigned int bar_num = 0, reg_addr = 0; // MD : Variables for BAR number and register address
    uint32_t reg_val = 0; // MD : Variable to store register value
    int rv = 0, err = 0; // MD : Return value and error code

    // MD : Check if the info parameter is NULL, return error if true
    if (info == NULL) {
        pr_err("REGISTER_READ: info is NULL\n");
        return -EINVAL;
    }

    // MD : Debug: Print netlink attributes for inspection
    xnl_dump_attrs(info);

    // MD : Retrieve and validate the Xilinx PCI device handle from netlink info
    xpdev = xnl_rcv_check_xpdev(info);
    if (!xpdev) {
        pr_err("REGISTER_READ: Failed to get xpdev\n");
        return -EINVAL;
    }

    // MD : Get QDMA device configuration
    rv = qdma_device_get_config(xpdev->dev_hndl, &conf, NULL, 0);
    if (rv < 0) {
        pr_err("REGISTER_READ: Failed to get device config, error = %d\n", rv);
        return rv;
    }

    // MD : Allocate a new netlink message
    skb = xnl_msg_alloc(XNL_CMD_REG_RD, 0, &hdr, info);
    if (!skb) {
        pr_err("REGISTER_READ: Failed to allocate SK buffer\n");
        return -ENOMEM;
    }

    // MD : Check for required attributes and retrieve their values
    if (!info->attrs[XNL_ATTR_REG_BAR_NUM]) {
        pr_warn("REGISTER_READ: Missing attribute 'XNL_ATTR_REG_BAR_NUM'\n");
        return -EINVAL;
    }
    if (!info->attrs[XNL_ATTR_REG_ADDR]) {
        pr_warn("REGISTER_READ: Missing attribute 'XNL_ATTR_REG_ADDR'\n");
        return -EINVAL;
    }

    bar_num = nla_get_u32(info->attrs[XNL_ATTR_REG_BAR_NUM]);
    reg_addr = nla_get_u32(info->attrs[XNL_ATTR_REG_ADDR]);

    // MD : Read the register value based on the BAR number
    if (bar_num == conf.bar_num_config) {
        rv = qdma_device_read_config_register(xpdev->dev_hndl, reg_addr, &reg_val);
        if (rv < 0) {
            pr_err("REGISTER_READ: Config bar register read failed, error = %d\n", rv);
            return rv;
        }
    } else if (bar_num == conf.bar_num_user) {
        rv = qdma_device_read_user_register(xpdev, reg_addr, &reg_val);
        if (rv < 0) {
            pr_err("REGISTER_READ: AXI Master Lite bar register read failed, error = %d\n", rv);
            return rv;
        }
    } else if (bar_num == conf.bar_num_bypass) {
        rv = qdma_device_read_bypass_register(xpdev, reg_addr, &reg_val);
        if (rv < 0) {
            pr_err("REGISTER_READ: AXI Bridge Master bar register read failed, error = %d\n", rv);
            return rv;
        }
    } else {
        rv += snprintf(buf + rv, XNL_RESP_BUFLEN_MIN, "Invalid bar number\n");
        goto send_resp;
    }

    // MD : Add the register value as an attribute to the netlink message
    err = xnl_msg_add_attr_uint(skb, XNL_ATTR_REG_VAL, reg_val);
    if (err < 0) {
        pr_err("REGISTER_READ: Failed to add register value attribute, error = %d\n", err);
        return err;
    }

    // MD : Send the netlink message
    err = xnl_msg_send(skb, hdr, info);
    return err;

send_resp:
    // MD : Send response buffer back to user space in case of error
    rv = xnl_respond_buffer(info, buf, XNL_RESP_BUFLEN_MIN, err);
    nlmsg_free(skb); // MD : Free the socket buffer
    return rv;
}

static int xnl_register_write(struct sk_buff *skb2, struct genl_info *info)
{
    // MD : Declare a pointer to the Xilinx PCI device structure
    struct xlnx_pci_dev *xpdev;
    // MD : Structure to hold QDMA device configuration
    struct qdma_dev_conf conf;
    // MD : Buffer to hold response message
    char buf[XNL_RESP_BUFLEN_MIN];
    // MD : Variables to store BAR number, register address, and register value
    unsigned int bar_num = 0, reg_addr = 0;
    uint32_t reg_val = 0;
    // MD : Variable to store return values
    int rv = 0;

    // MD : Check if the info parameter is NULL, return error if true
    if (info == NULL) {
        pr_err("REGISTER_WRITE: info is NULL\n");
        return -EINVAL;
    }

    // MD : Debug: Print netlink attributes for inspection
    xnl_dump_attrs(info);

    // MD : Retrieve and validate the Xilinx PCI device handle from netlink info
    xpdev = xnl_rcv_check_xpdev(info);
    if (!xpdev) {
        pr_err("REGISTER_WRITE: Failed to get xpdev\n");
        return -EINVAL;
    }

    // MD : Get the QDMA device configuration
    rv = qdma_device_get_config(xpdev->dev_hndl, &conf, NULL, 0);
    if (rv < 0) {
        pr_err("REGISTER_WRITE: Failed to get device config, error = %d\n", rv);
        return rv;
    }

    // MD : Check for the presence of required attributes
    if (!info->attrs[XNL_ATTR_REG_BAR_NUM]) {
        pr_warn("REGISTER_WRITE: Missing attribute 'XNL_ATTR_REG_BAR_NUM'\n");
        return -EINVAL;
    }

    if (!info->attrs[XNL_ATTR_REG_ADDR]) {
        pr_warn("REGISTER_WRITE: Missing attribute 'XNL_ATTR_REG_ADDR'\n");
        return -EINVAL;
    }

    if (!info->attrs[XNL_ATTR_REG_VAL]) {
        pr_warn("REGISTER_WRITE: Missing attribute 'XNL_ATTR_REG_VAL'\n");
        return -EINVAL;
    }

    // MD : Extract values from netlink attributes
    bar_num = nla_get_u32(info->attrs[XNL_ATTR_REG_BAR_NUM]);
    reg_addr = nla_get_u32(info->attrs[XNL_ATTR_REG_ADDR]);
    reg_val = nla_get_u32(info->attrs[XNL_ATTR_REG_VAL]);

    // MD : Write to the appropriate register based on the BAR number
    if (bar_num == conf.bar_num_config) {
        rv = qdma_device_write_config_register(xpdev->dev_hndl, reg_addr, reg_val);
        if (rv < 0) {
            pr_err("REGISTER_WRITE: Config bar register write failed, error = %d\n", rv);
            return rv;
        }
    } else if (bar_num == conf.bar_num_user) {
        rv = qdma_device_write_user_register(xpdev, reg_addr, reg_val);
        if (rv < 0) {
            pr_err("REGISTER_WRITE: AXI Master Lite bar register write failed, error = %d\n", rv);
            return rv;
        }
    } else if (bar_num == conf.bar_num_bypass) {
        rv = qdma_device_write_bypass_register(xpdev, reg_addr, reg_val);
        if (rv < 0) {
            pr_err("REGISTER_WRITE: AXI Bridge Master bar register write failed, error = %d\n", rv);
            return rv;
        }
    } else {
        // MD : Handle invalid BAR number
        rv += snprintf(buf + rv, XNL_RESP_BUFLEN_MIN, "Invalid bar number\n");
        goto send_resp;
    }

send_resp:
    // MD : Send response back to user space
    rv = xnl_respond_buffer(info, buf, XNL_RESP_BUFLEN_MIN, rv);
    return rv;
}

static int xnl_get_global_csr(struct sk_buff *skb2, struct genl_info *info)
{
    // MD : Structure to hold global CSR (Control and Status Register) configuration
    struct global_csr_conf *csr;
    // MD : Pointer to Xilinx PCI device structure
    struct xlnx_pci_dev *xpdev;
    // MD : Variable to store return values
    int rv;
    // MD : Variables to hold CSR index and count
    u8 index = 0, count = 0;

    // MD : Check if the info parameter is NULL, return error if true
    if (info == NULL) {
        pr_err("GET_GLOBAL_CSR: info is NULL\n");
        return -EINVAL;
    }

    // MD : Debug: Print netlink attributes for inspection
    xnl_dump_attrs(info);

    // MD : Retrieve and validate the Xilinx PCI device handle from netlink info
    xpdev = xnl_rcv_check_xpdev(info);
    if (!xpdev) {
        pr_err("GET_GLOBAL_CSR: Failed to get xpdev\n");
        return 0;
    }

    // MD : Allocate memory for the global CSR configuration structure
    csr = kmalloc(sizeof(struct global_csr_conf), GFP_KERNEL);
    if (!csr) {
        pr_err("GET_GLOBAL_CSR: Failed to allocate memory for CSR\n");
        return -ENOMEM;
    }

    // MD : Check if CSR index attribute is present, return error if missing
    if (!info->attrs[XNL_ATTR_CSR_INDEX]) {
        pr_warn("GET_GLOBAL_CSR: Missing attribute 'XNL_ATTR_CSR_INDEX'\n");
        kfree(csr);
        return -EINVAL;
    }

    // MD : Check if CSR count attribute is present, return error if missing
    if (!info->attrs[XNL_ATTR_CSR_COUNT]) {
        pr_warn("GET_GLOBAL_CSR: Missing attribute 'XNL_ATTR_CSR_COUNT'\n");
        kfree(csr);
        return -EINVAL;
    }

    // MD : Retrieve CSR index and count from netlink attributes
    index = nla_get_u32(info->attrs[XNL_ATTR_CSR_INDEX]);
    count = nla_get_u32(info->attrs[XNL_ATTR_CSR_COUNT]);

    // MD : Get the global CSR configuration from the device
    rv = qdma_global_csr_get(xpdev->dev_hndl, index, count, csr);
    if (rv < 0) {
        pr_err("GET_GLOBAL_CSR: qdma_global_csr_get() failed: %d\n", rv);
        goto free_msg_buff;
    }

    // MD : Send the CSR data back to user space
    rv = xnl_respond_data(info, (void *)csr, sizeof(struct global_csr_conf));

free_msg_buff:
    // MD : Free the allocated CSR structure memory
    kfree(csr);
    return rv;
}

#ifdef TANDEM_BOOT_SUPPORTED
static int xnl_en_st(struct sk_buff *skb2, struct genl_info *info)
{
    // MD : Declare a pointer to the Xilinx PCI device structure
    struct xlnx_pci_dev *xpdev;
    // MD : Variable to store return values
    int rv;
    // MD : Buffer to hold response message
    char *buf;

    // MD : Check if the info parameter is NULL, return error if true
    if (info == NULL) {
        pr_err("EN_ST: info is NULL\n");
        return -EINVAL;
    }

    // MD : Debug: Print netlink attributes for inspection
    xnl_dump_attrs(info);

    // MD : Retrieve and validate the Xilinx PCI device handle from netlink info
    xpdev = xnl_rcv_check_xpdev(info);
    if (!xpdev) {
        pr_err("EN_ST: Failed to get xpdev\n");
        return -EINVAL;
    }

    // MD : Allocate memory for the response buffer
    buf = xnl_mem_alloc(XNL_RESP_BUFLEN_MIN, info);
    if (!buf) {
        pr_err("EN_ST: Failed to allocate memory for buffer\n");
        return -ENOMEM;
    }

    // MD : Initialize the stream context for the device
    qdma_init_st_ctxt(xpdev->dev_hndl, buf, XNL_RESP_BUFLEN_MAX);
    pr_debug("EN_ST: Stream context initialized\n");

    // MD : Send a response back to the user space indicating success
    rv = xnl_respond_buffer(info, buf, XNL_RESP_BUFLEN_MAX, 0);
    pr_debug("EN_ST: Response sent\n");

    // MD : Free the allocated buffer memory
    kfree(buf);
    return rv;
}
#endif

int xlnx_nl_init(void)
{
    int rv; // MD : Variable to store the return value of registration functions

    // MD : Check if the macro __GENL_REG_FAMILY_OPS_FUNC__ is defined
    // MD : This determines which registration function to use
#ifdef __GENL_REG_FAMILY_OPS_FUNC__
    // MD : Register the generic netlink family with operations
    rv = genl_register_family_with_ops(&xnl_family, xnl_ops, ARRAY_SIZE(xnl_ops));
    pr_debug("xlnx_nl_init: Registered family with ops, rv=%d\n", rv);
#else
    // MD : Register the generic netlink family without operations
    rv = genl_register_family(&xnl_family);
    pr_debug("xlnx_nl_init: Registered family without ops, rv=%d\n", rv);
#endif

    // MD : Check if registration was successful
    if (rv)
        pr_err("genl_register_family failed %d.\n", rv);

    return rv; // MD : Return the result of the registration
}

void xlnx_nl_exit(void)
{
    int rv; // MD : Variable to store the return value of the unregistration function

    // MD : Unregister the generic netlink family
    rv = genl_unregister_family(&xnl_family);
    pr_debug("xlnx_nl_exit: Unregistered family, rv=%d\n", rv);

    // MD : Check if unregistration was successful
    if (rv)
        pr_err("genl_unregister_family failed %d.\n", rv);
}


