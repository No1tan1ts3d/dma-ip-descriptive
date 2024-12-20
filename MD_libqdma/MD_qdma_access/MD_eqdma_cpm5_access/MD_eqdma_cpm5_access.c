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

#include "eqdma_cpm5_access.h"
#include "eqdma_cpm5_reg.h"
#include "qdma_reg_dump.h"

#ifdef ENABLE_WPP_TRACING
#include "eqdma_cpm5_access.tmh"
#endif

#define UNUSED(x) (void)(x) // MD: Macro to suppress unused variable warnings

/* MD:* EQDMA Context array size definitions */
#define EQDMA_CPM5_FMAP_NUM_WORDS                 2
#define EQDMA_CPM5_SW_CONTEXT_NUM_WORDS           8
#define EQDMA_CPM5_HW_CONTEXT_NUM_WORDS           2
#define EQDMA_CPM5_PFETCH_CONTEXT_NUM_WORDS       2
#define EQDMA_CPM5_CR_CONTEXT_NUM_WORDS           1
#define EQDMA_CPM5_CMPT_CONTEXT_NUM_WORDS         6
#define EQDMA_CPM5_IND_INTR_CONTEXT_NUM_WORDS     4

#define EQDMA_CPM5_VF_USER_BAR_ID                 2 // MD: User BAR ID for VF

/* MD:* Register group start addresses */
#define EQDMA_CPM5_REG_GROUP_1_START_ADDR	0x000
#define EQDMA_CPM5_REG_GROUP_2_START_ADDR	0x804
#define EQDMA_CPM5_REG_GROUP_3_START_ADDR	0xB00
#define EQDMA_CPM5_REG_GROUP_4_START_ADDR	0x5014

/* MD:* Error mask definitions */
#define EQDMA_CPM5_TOTAL_LEAF_ERROR_AGGREGATORS 11
#define EQDMA_CPM5_GLBL_TRQ_ERR_ALL_MASK		0xB3
#define EQDMA_CPM5_GLBL_DSC_ERR_ALL_MASK		0x1F9037E
#define EQDMA_CPM5_C2H_ERR_ALL_MASK				0x3F6DF
#define EQDMA_CPM5_C2H_FATAL_ERR_ALL_MASK		0x1FDF1B
#define EQDMA_CPM5_H2C_ERR_ALL_MASK				0x3F
#define EQDMA_CPM5_SBE_ERR_ALL_MASK				0xFFFFFFFF
#define EQDMA_CPM5_DBE_ERR_ALL_MASK				0xFFFFFFFF
#define EQDMA_CPM5_MM_C2H_ERR_ALL_MASK			0x70000003
#define EQDMA_CPM5_MM_H2C0_ERR_ALL_MASK		    0x3041013E

/* MD:* H2C Throttle settings */
#define EQDMA_CPM5_H2C_THROT_DATA_THRESH       0x5000
#define EQDMA_CPM5_THROT_EN_DATA               1
#define EQDMA_CPM5_THROT_EN_REQ                0
#define EQDMA_CPM5_H2C_THROT_REQ_THRESH        0xC0

/* MD:* Auxillary Bitmasks for fields spanning multiple words */
#define EQDMA_CPM5_SW_CTXT_PASID_GET_H_MASK              GENMASK(21, 12)
#define EQDMA_CPM5_SW_CTXT_PASID_GET_L_MASK              GENMASK(11, 0)
#define EQDMA_CPM5_SW_CTXT_VIRTIO_DSC_BASE_GET_H_MASK    GENMASK_ULL(63, 53)
#define EQDMA_CPM5_SW_CTXT_VIRTIO_DSC_BASE_GET_M_MASK    GENMASK_ULL(52, 21)
#define EQDMA_CPM5_SW_CTXT_VIRTIO_DSC_BASE_GET_L_MASK    GENMASK_ULL(20, 0)
#define EQDMA_CPM5_CMPL_CTXT_PASID_GET_H_MASK            GENMASK(21, 9)
#define EQDMA_CPM5_CMPL_CTXT_PASID_GET_L_MASK            GENMASK(8, 0)
#define EQDMA_CPM5_INTR_CTXT_PASID_GET_H_MASK            GENMASK(21, 9)
#define EQDMA_CPM5_INTR_CTXT_PASID_GET_L_MASK            GENMASK(8, 0)

#define EQDMA_CPM5_OFFSET_GLBL2_PF_BARLITE_EXT		0x10C

#define QDMA_OFFSET_GLBL2_PF_BARLITE_INT		0x104
#define QDMA_GLBL2_PF3_BAR_MAP_MASK				GENMASK(23, 18)
#define QDMA_GLBL2_PF2_BAR_MAP_MASK				GENMASK(17, 12)
#define QDMA_GLBL2_PF1_BAR_MAP_MASK				GENMASK(11, 6)
#define QDMA_GLBL2_PF0_BAR_MAP_MASK				GENMASK(5, 0)

#define EQDMA_CPM5_GLBL2_DBG_MODE_EN_MASK			BIT(4)
#define EQDMA_CPM5_GLBL2_DESC_ENG_MODE_MASK			GENMASK(3, 2)
#define EQDMA_CPM5_GLBL2_FLR_PRESENT_MASK			BIT(1)
#define EQDMA_CPM5_GLBL2_MAILBOX_EN_MASK			BIT(0)

#define EQDMA_CPM5_DEFAULT_C2H_INTR_TIMER_TICK     50
#define PREFETCH_QUEUE_COUNT_STEP                   4
#define EQDMA_CPM5_DEFAULT_CMPT_COAL_MAX_BUF_SZ    0x3F

/* MD: TODO: This is work around and this needs to be auto generated from ODS */
/* MD:* EQDMA_CPM5_IND_REG_SEL_FMAP */
#define EQDMA_CPM5_FMAP_CTXT_W1_QID_MAX_MASK         GENMASK(12, 0)
#define EQDMA_CPM5_FMAP_CTXT_W0_QID_MASK             GENMASK(11, 0)

/* MD:* Function prototypes for error processing */
static void eqdma_cpm5_hw_st_h2c_err_process(void *dev_hndl);
static void eqdma_cpm5_hw_st_c2h_err_process(void *dev_hndl);
static void eqdma_cpm5_hw_desc_err_process(void *dev_hndl);
static void eqdma_cpm5_hw_trq_err_process(void *dev_hndl);
static void eqdma_cpm5_hw_ram_sbe_err_process(void *dev_hndl);
static void eqdma_cpm5_hw_ram_dbe_err_process(void *dev_hndl);
static void eqdma_cpm5_mm_h2c0_err_process(void *dev_hndl);
static void eqdma_cpm5_mm_c2h0_err_process(void *dev_hndl);

/* MD:
 * Structure to hold error information for EQDMA CPM5 hardware descriptor errors.
 * Each entry in the array corresponds to a specific error type, with associated
 * error message, mask addresses, status addresses, and a pointer to the error
 * processing function.
 */
static struct eqdma_cpm5_hw_err_info eqdma_cpm5_err_info[EQDMA_CPM5_ERRS_ALL] = {
    /* MD: Descriptor errors */
    {
        EQDMA_CPM5_DSC_ERR_POISON, // MD: Error code for poison error
        "Poison error", // MD: Description of the error
        EQDMA_CPM5_GLBL_DSC_ERR_MSK_ADDR, // MD: Mask address for the error
        EQDMA_CPM5_GLBL_DSC_ERR_STS_ADDR, // MD: Status address for the error
        GLBL_DSC_ERR_STS_POISON_MASK, // MD: Specific mask for poison error
        GLBL_ERR_STAT_ERR_DSC_MASK, // MD: General descriptor error status mask
        &eqdma_cpm5_hw_desc_err_process // MD: Function to process this error
    },
    {
        EQDMA_CPM5_DSC_ERR_UR_CA,
        "Unsupported request or completer aborted error",
        EQDMA_CPM5_GLBL_DSC_ERR_MSK_ADDR,
        EQDMA_CPM5_GLBL_DSC_ERR_STS_ADDR,
        GLBL_DSC_ERR_STS_UR_CA_MASK,
        GLBL_ERR_STAT_ERR_DSC_MASK,
        &eqdma_cpm5_hw_desc_err_process
    },
    {
        EQDMA_CPM5_DSC_ERR_BCNT,
        "Unexpected Byte count in completion error",
        EQDMA_CPM5_GLBL_DSC_ERR_MSK_ADDR,
        EQDMA_CPM5_GLBL_DSC_ERR_STS_ADDR,
        GLBL_DSC_ERR_STS_BCNT_MASK,
        GLBL_ERR_STAT_ERR_DSC_MASK,
        &eqdma_cpm5_hw_desc_err_process
    },
    {
        EQDMA_CPM5_DSC_ERR_PARAM,
        "Parameter mismatch error",
        EQDMA_CPM5_GLBL_DSC_ERR_MSK_ADDR,
        EQDMA_CPM5_GLBL_DSC_ERR_STS_ADDR,
        GLBL_DSC_ERR_STS_PARAM_MASK,
        GLBL_ERR_STAT_ERR_DSC_MASK,
        &eqdma_cpm5_hw_desc_err_process
    },
    {
        EQDMA_CPM5_DSC_ERR_ADDR,
        "Address mismatch error",
        EQDMA_CPM5_GLBL_DSC_ERR_MSK_ADDR,
        EQDMA_CPM5_GLBL_DSC_ERR_STS_ADDR,
        GLBL_DSC_ERR_STS_ADDR_MASK,
        GLBL_ERR_STAT_ERR_DSC_MASK,
        &eqdma_cpm5_hw_desc_err_process
    },
    {
        EQDMA_CPM5_DSC_ERR_TAG,
        "Unexpected tag error",
        EQDMA_CPM5_GLBL_DSC_ERR_MSK_ADDR,
        EQDMA_CPM5_GLBL_DSC_ERR_STS_ADDR,
        GLBL_DSC_ERR_STS_TAG_MASK,
        GLBL_ERR_STAT_ERR_DSC_MASK,
        &eqdma_cpm5_hw_desc_err_process
    },
    {
        EQDMA_CPM5_DSC_ERR_FLR,
        "FLR error",
        EQDMA_CPM5_GLBL_DSC_ERR_MSK_ADDR,
        EQDMA_CPM5_GLBL_DSC_ERR_STS_ADDR,
        GLBL_DSC_ERR_STS_FLR_MASK,
        GLBL_ERR_STAT_ERR_DSC_MASK,
        &eqdma_cpm5_hw_desc_err_process
    },
    {
        EQDMA_CPM5_DSC_ERR_TIMEOUT,
        "Timed out error",
        EQDMA_CPM5_GLBL_DSC_ERR_MSK_ADDR,
        EQDMA_CPM5_GLBL_DSC_ERR_STS_ADDR,
        GLBL_DSC_ERR_STS_TIMEOUT_MASK,
        GLBL_ERR_STAT_ERR_DSC_MASK,
        &eqdma_cpm5_hw_desc_err_process
    },
    {
        EQDMA_CPM5_DSC_ERR_DAT_POISON,
        "Poison data error",
        EQDMA_CPM5_GLBL_DSC_ERR_MSK_ADDR,
        EQDMA_CPM5_GLBL_DSC_ERR_STS_ADDR,
        GLBL_DSC_ERR_STS_DAT_POISON_MASK,
        GLBL_ERR_STAT_ERR_DSC_MASK,
        &eqdma_cpm5_hw_desc_err_process
    },
    {
        EQDMA_CPM5_DSC_ERR_FLR_CANCEL,
        "Descriptor fetch cancelled due to FLR error",
        EQDMA_CPM5_GLBL_DSC_ERR_MSK_ADDR,
        EQDMA_CPM5_GLBL_DSC_ERR_STS_ADDR,
        GLBL_DSC_ERR_STS_FLR_CANCEL_MASK,
        GLBL_ERR_STAT_ERR_DSC_MASK,
        &eqdma_cpm5_hw_desc_err_process
    },
    {
        EQDMA_CPM5_DSC_ERR_DMA,
        "DMA engine error",
        EQDMA_CPM5_GLBL_DSC_ERR_MSK_ADDR,
        EQDMA_CPM5_GLBL_DSC_ERR_STS_ADDR,
        GLBL_DSC_ERR_STS_DMA_MASK,
        GLBL_ERR_STAT_ERR_DSC_MASK,
        &eqdma_cpm5_hw_desc_err_process
    },
    {
        EQDMA_CPM5_DSC_ERR_DSC,
        "Invalid PIDX update error",
        EQDMA_CPM5_GLBL_DSC_ERR_MSK_ADDR,
        EQDMA_CPM5_GLBL_DSC_ERR_STS_ADDR,
        GLBL_DSC_ERR_STS_DSC_MASK,
        GLBL_ERR_STAT_ERR_DSC_MASK,
        &eqdma_cpm5_hw_desc_err_process
    },
    {
        EQDMA_CPM5_DSC_ERR_RQ_CANCEL,
        "Descriptor fetch cancelled due to disable register status error",
        EQDMA_CPM5_GLBL_DSC_ERR_MSK_ADDR,
        EQDMA_CPM5_GLBL_DSC_ERR_STS_ADDR,
        GLBL_DSC_ERR_STS_RQ_CANCEL_MASK,
        GLBL_ERR_STAT_ERR_DSC_MASK,
        &eqdma_cpm5_hw_desc_err_process
    },
    {
        EQDMA_CPM5_DSC_ERR_DBE,
        "UNC_ERR_RAM_DBE error",
        EQDMA_CPM5_GLBL_DSC_ERR_MSK_ADDR,
        EQDMA_CPM5_GLBL_DSC_ERR_STS_ADDR,
        GLBL_DSC_ERR_STS_DBE_MASK,
        GLBL_ERR_STAT_ERR_DSC_MASK,
        &eqdma_cpm5_hw_desc_err_process
    },
    {
        EQDMA_CPM5_DSC_ERR_SBE,
        "UNC_ERR_RAM_SBE error",
        EQDMA_CPM5_GLBL_DSC_ERR_MSK_ADDR,
        EQDMA_CPM5_GLBL_DSC_ERR_STS_ADDR,
        GLBL_DSC_ERR_STS_SBE_MASK,
        GLBL_ERR_STAT_ERR_DSC_MASK,
        &eqdma_cpm5_hw_desc_err_process
    },
    {
        EQDMA_CPM5_DSC_ERR_PORT_ID,
        "Port ID Error",
        EQDMA_CPM5_GLBL_DSC_ERR_MSK_ADDR,
        EQDMA_CPM5_GLBL_DSC_ERR_STS_ADDR,
        GLBL_DSC_ERR_STS_PORT_ID_MASK,
        GLBL_ERR_STAT_ERR_DSC_MASK,
        &eqdma_cpm5_hw_desc_err_process
    },
    {
        EQDMA_CPM5_DSC_ERR_ALL,
        "All Descriptor errors",
        EQDMA_CPM5_GLBL_DSC_ERR_MSK_ADDR,
        EQDMA_CPM5_GLBL_DSC_ERR_STS_ADDR,
        EQDMA_CPM5_GLBL_DSC_ERR_ALL_MASK,
        GLBL_ERR_STAT_ERR_DSC_MASK,
        &eqdma_cpm5_hw_desc_err_process
    }

    /* MD: TRQ errors */
    {
        EQDMA_CPM5_TRQ_ERR_CSR_UNMAPPED, // MD: Error code for unmapped CSR access
        "Access targeted unmapped register space via CSR pathway error", // MD: Description
        EQDMA_CPM5_GLBL_TRQ_ERR_MSK_ADDR, // MD: Mask address for the error
        EQDMA_CPM5_GLBL_TRQ_ERR_STS_ADDR, // MD: Status address for the error
        GLBL_TRQ_ERR_STS_CSR_UNMAPPED_MASK, // MD: Specific mask for CSR unmapped error
        GLBL_ERR_STAT_ERR_TRQ_MASK, // MD: General TRQ error status mask
        &eqdma_cpm5_hw_trq_err_process // MD: Function to process this error
    },
    {
        EQDMA_CPM5_TRQ_ERR_VF_ACCESS, // MD: Error code for VF access violation
        "VF attempted to access Global register space or Function map", // MD: Description
        EQDMA_CPM5_GLBL_TRQ_ERR_MSK_ADDR,
        EQDMA_CPM5_GLBL_TRQ_ERR_STS_ADDR,
        GLBL_TRQ_ERR_STS_VF_ACCESS_ERR_MASK,
        GLBL_ERR_STAT_ERR_TRQ_MASK,
        &eqdma_cpm5_hw_trq_err_process
    },
    {
        EQDMA_CPM5_TRQ_ERR_TCP_CSR_TIMEOUT, // MD: Error code for CSR timeout
        "Timeout on request to dma internal csr register", // MD: Description
        EQDMA_CPM5_GLBL_TRQ_ERR_MSK_ADDR,
        EQDMA_CPM5_GLBL_TRQ_ERR_STS_ADDR,
        GLBL_TRQ_ERR_STS_TCP_CSR_TIMEOUT_MASK,
        GLBL_ERR_STAT_ERR_TRQ_MASK,
        &eqdma_cpm5_hw_trq_err_process
    },
    {
        EQDMA_CPM5_TRQ_ERR_QSPC_UNMAPPED, // MD: Error code for unmapped queue space
        "Access targeted unmapped register via queue space pathway", // MD: Description
        EQDMA_CPM5_GLBL_TRQ_ERR_MSK_ADDR,
        EQDMA_CPM5_GLBL_TRQ_ERR_STS_ADDR,
        GLBL_TRQ_ERR_STS_QSPC_UNMAPPED_MASK,
        GLBL_ERR_STAT_ERR_TRQ_MASK,
        &eqdma_cpm5_hw_trq_err_process
    },
    {
        EQDMA_CPM5_TRQ_ERR_QID_RANGE, // MD: Error code for QID range error
        "Qid range error", // MD: Description
        EQDMA_CPM5_GLBL_TRQ_ERR_MSK_ADDR,
        EQDMA_CPM5_GLBL_TRQ_ERR_STS_ADDR,
        GLBL_TRQ_ERR_STS_QID_RANGE_MASK,
        GLBL_ERR_STAT_ERR_TRQ_MASK,
        &eqdma_cpm5_hw_trq_err_process
    },
    {
        EQDMA_CPM5_TRQ_ERR_TCP_QSPC_TIMEOUT, // MD: Error code for queue space timeout
        "Timeout on request to dma internal queue space register", // MD: Description
        EQDMA_CPM5_GLBL_TRQ_ERR_MSK_ADDR,
        EQDMA_CPM5_GLBL_TRQ_ERR_STS_ADDR,
        GLBL_TRQ_ERR_STS_TCP_QSPC_TIMEOUT_MASK,
        GLBL_ERR_STAT_ERR_TRQ_MASK,
        &eqdma_cpm5_hw_trq_err_process
    },
    {
        EQDMA_CPM5_TRQ_ERR_ALL, // MD: Error code for all TRQ errors
        "All TRQ errors", // MD: Description
        EQDMA_CPM5_GLBL_TRQ_ERR_MSK_ADDR,
        EQDMA_CPM5_GLBL_TRQ_ERR_STS_ADDR,
        EQDMA_CPM5_GLBL_TRQ_ERR_ALL_MASK,
        GLBL_ERR_STAT_ERR_TRQ_MASK,
        &eqdma_cpm5_hw_trq_err_process
    },

    /* MD: C2H Errors */
    {
        EQDMA_CPM5_ST_C2H_ERR_MTY_MISMATCH, // MD: Error code for MTY mismatch
        "MTY mismatch error", // MD: Description
        EQDMA_CPM5_C2H_ERR_MASK_ADDR,
        EQDMA_CPM5_C2H_ERR_STAT_ADDR,
        C2H_ERR_STAT_MTY_MISMATCH_MASK,
        GLBL_ERR_STAT_ERR_C2H_ST_MASK,
        &eqdma_cpm5_hw_st_c2h_err_process
    },
    {
        EQDMA_CPM5_ST_C2H_ERR_LEN_MISMATCH, // MD: Error code for length mismatch
        "Packet length mismatch error", // MD: Description
        EQDMA_CPM5_C2H_ERR_MASK_ADDR,
        EQDMA_CPM5_C2H_ERR_STAT_ADDR,
        C2H_ERR_STAT_LEN_MISMATCH_MASK,
        GLBL_ERR_STAT_ERR_C2H_ST_MASK,
        &eqdma_cpm5_hw_st_c2h_err_process
    },
    {
        EQDMA_CPM5_ST_C2H_ERR_SH_CMPT_DSC, // MD: Error code for shared CMPT descriptor error
        "A Shared CMPT queue has encountered a descriptor error", // MD: Description
        EQDMA_CPM5_C2H_ERR_MASK_ADDR,
        EQDMA_CPM5_C2H_ERR_STAT_ADDR,
        C2H_ERR_STAT_SH_CMPT_DSC_ERR_MASK,
        GLBL_ERR_STAT_ERR_C2H_ST_MASK,
        &eqdma_cpm5_hw_st_c2h_err_process
    },
    {
        EQDMA_CPM5_ST_C2H_ERR_QID_MISMATCH, // MD: Error code for QID mismatch
        "Qid mismatch error", // MD: Description
        EQDMA_CPM5_C2H_ERR_MASK_ADDR,
        EQDMA_CPM5_C2H_ERR_STAT_ADDR,
        C2H_ERR_STAT_QID_MISMATCH_MASK,
        GLBL_ERR_STAT_ERR_C2H_ST_MASK,
        &eqdma_cpm5_hw_st_c2h_err_process
    },
    {
        EQDMA_CPM5_ST_C2H_ERR_DESC_RSP_ERR, // MD: Error code for descriptor response error
        "Descriptor error bit set", // MD: Description
        EQDMA_CPM5_C2H_ERR_MASK_ADDR,
        EQDMA_CPM5_C2H_ERR_STAT_ADDR,
        C2H_ERR_STAT_DESC_RSP_ERR_MASK,
        GLBL_ERR_STAT_ERR_C2H_ST_MASK,
        &eqdma_cpm5_hw_st_c2h_err_process
    },
    {
        EQDMA_CPM5_ST_C2H_ERR_ENG_WPL_DATA_PAR_ERR, // MD: Error code for data parity error
        "Data parity error", // MD: Description
        EQDMA_CPM5_C2H_ERR_MASK_ADDR,
        EQDMA_CPM5_C2H_ERR_STAT_ADDR,
        C2H_ERR_STAT_ENG_WPL_DATA_PAR_ERR_MASK,
        GLBL_ERR_STAT_ERR_C2H_ST_MASK,
        &eqdma_cpm5_hw_st_c2h_err_process
    },
    {
        EQDMA_CPM5_ST_C2H_ERR_MSI_INT_FAIL, // MD: Error code for MSI interrupt failure
        "MSI got a fail response error", // MD: Description
        EQDMA_CPM5_C2H_ERR_MASK_ADDR,
        EQDMA_CPM5_C2H_ERR_STAT_ADDR,
        C2H_ERR_STAT_MSI_INT_FAIL_MASK,
        GLBL_ERR_STAT_ERR_C2H_ST_MASK,
        &eqdma_cpm5_hw_st_c2h_err_process
    },
    {
        EQDMA_CPM5_ST_C2H_ERR_ERR_DESC_CNT, // MD: Error code for descriptor count error
        "Descriptor count error", // MD: Description
        EQDMA_CPM5_C2H_ERR_MASK_ADDR,
        EQDMA_CPM5_C2H_ERR_STAT_ADDR,
        C2H_ERR_STAT_ERR_DESC_CNT_MASK,
        GLBL_ERR_STAT_ERR_C2H_ST_MASK,
        &eqdma_cpm5_hw_st_c2h_err_process
    },
    {
        EQDMA_CPM5_ST_C2H_ERR_PORTID_CTXT_MISMATCH, // MD: Error code for port ID context mismatch
        "Port id in packet and pfetch ctxt mismatch error", // MD: Description
        EQDMA_CPM5_C2H_ERR_MASK_ADDR,
        EQDMA_CPM5_C2H_ERR_STAT_ADDR,
        C2H_ERR_STAT_PORT_ID_CTXT_MISMATCH_MASK,
        GLBL_ERR_STAT_ERR_C2H_ST_MASK,
        &eqdma_cpm5_hw_st_c2h_err_process
    },
    {
        EQDMA_CPM5_ST_C2H_ERR_CMPT_INV_Q_ERR, // MD: Error code for invalid queue writeback
        "Writeback on invalid queue error", // MD: Description
        EQDMA_CPM5_C2H_ERR_MASK_ADDR,
        EQDMA_CPM5_C2H_ERR_STAT_ADDR,
        C2H_ERR_STAT_WRB_INV_Q_ERR_MASK,
        GLBL_ERR_STAT_ERR_C2H_ST_MASK,
        &eqdma_cpm5_hw_st_c2h_err_process
    },
    {
        EQDMA_CPM5_ST_C2H_ERR_CMPT_QFULL_ERR, // MD: Error code for completion queue full
        "Completion queue gets full error", // MD: Description
        EQDMA_CPM5_C2H_ERR_MASK_ADDR,
        EQDMA_CPM5_C2H_ERR_STAT_ADDR,
        C2H_ERR_STAT_WRB_QFULL_ERR_MASK,
        GLBL_ERR_STAT_ERR_C2H_ST_MASK,
        &eqdma_cpm5_hw_st_c2h_err_process
    },
    {
        EQDMA_CPM5_ST_C2H_ERR_CMPT_CIDX_ERR, // MD: Error code for bad CIDX update
        "Bad CIDX update by the software error", // MD: Description
        EQDMA_CPM5_C2H_ERR_MASK_ADDR,
        EQDMA_CPM5_C2H_ERR_STAT_ADDR,
        C2H_ERR_STAT_WRB_CIDX_ERR_MASK,
        GLBL_ERR_STAT_ERR_C2H_ST_MASK,
        &eqdma_cpm5_hw_st_c2h_err_process
    },
    {
        EQDMA_CPM5_ST_C2H_ERR_CMPT_PRTY_ERR, // MD: Error code for C2H completion parity error
        "C2H completion Parity error", // MD: Description
        EQDMA_CPM5_C2H_ERR_MASK_ADDR,
        EQDMA_CPM5_C2H_ERR_STAT_ADDR,
        C2H_ERR_STAT_WRB_PRTY_ERR_MASK,
        GLBL_ERR_STAT_ERR_C2H_ST_MASK,
        &eqdma_cpm5_hw_st_c2h_err_process
    },
    {
        EQDMA_CPM5_ST_C2H_ERR_AVL_RING_DSC, // MD: Error code for available ring descriptor error
        "Available ring fetch returns descriptor with error", // MD: Description
        EQDMA_CPM5_C2H_ERR_MASK_ADDR,
        EQDMA_CPM5_C2H_ERR_STAT_ADDR,
        C2H_ERR_STAT_AVL_RING_DSC_ERR_MASK,
        GLBL_ERR_STAT_ERR_C2H_ST_MASK,
        &eqdma_cpm5_hw_st_c2h_err_process
    },
    {
        EQDMA_CPM5_ST_C2H_ERR_HDR_ECC_UNC, // MD: Error code for multi-bit ECC error on header
        "multi-bit ecc error on c2h packet header", // MD: Description
        EQDMA_CPM5_C2H_ERR_MASK_ADDR,
        EQDMA_CPM5_C2H_ERR_STAT_ADDR,
        C2H_ERR_STAT_HDR_ECC_UNC_ERR_MASK,
        GLBL_ERR_STAT_ERR_C2H_ST_MASK,
        &eqdma_cpm5_hw_st_c2h_err_process
    },
    {
        EQDMA_CPM5_ST_C2H_ERR_HDR_ECC_COR, // MD: Error code for single-bit ECC error on header
        "single-bit ecc error on c2h packet header", // MD: Description
        EQDMA_CPM5_C2H_ERR_MASK_ADDR,
        EQDMA_CPM5_C2H_ERR_STAT_ADDR,
        C2H_ERR_STAT_HDR_ECC_COR_ERR_MASK,
        GLBL_ERR_STAT_ERR_C2H_ST_MASK,
        &eqdma_cpm5_hw_st_c2h_err_process
    },
    {
        EQDMA_CPM5_ST_C2H_ERR_WRB_PORT_ID_ERR, // MD: Error code for port ID error
        "Port ID error", // MD: Description
        EQDMA_CPM5_C2H_ERR_MASK_ADDR,
        EQDMA_CPM5_C2H_ERR_STAT_ADDR,
        C2H_ERR_STAT_WRB_PORT_ID_ERR_MASK,
        GLBL_ERR_STAT_ERR_C2H_ST_MASK,
        &eqdma_cpm5_hw_st_c2h_err_process
    },
    {
        EQDMA_CPM5_ST_C2H_ERR_ALL, // MD: Error code for all C2H errors
        "All C2h errors", // MD: Description
        EQDMA_CPM5_C2H_ERR_MASK_ADDR,
        EQDMA_CPM5_C2H_ERR_STAT_ADDR,
        EQDMA_CPM5_C2H_ERR_ALL_MASK,
        GLBL_ERR_STAT_ERR_C2H_ST_MASK,
        &eqdma_cpm5_hw_st_c2h_err_process
    }

    /* MD: C2H Fatal Errors */
    {
        EQDMA_CPM5_ST_FATAL_ERR_MTY_MISMATCH, // MD: Error code for MTY mismatch
        "Fatal MTY mismatch error", // MD: Description
        EQDMA_CPM5_C2H_FATAL_ERR_MASK_ADDR, // MD: Mask address for the error
        EQDMA_CPM5_C2H_FATAL_ERR_STAT_ADDR, // MD: Status address for the error
        C2H_FATAL_ERR_STAT_MTY_MISMATCH_MASK, // MD: Specific mask for MTY mismatch
        GLBL_ERR_STAT_ERR_C2H_ST_MASK, // MD: General C2H error status mask
        &eqdma_cpm5_hw_st_c2h_err_process // MD: Function to process this error
    },
    {
        EQDMA_CPM5_ST_FATAL_ERR_LEN_MISMATCH, // MD: Error code for length mismatch
        "Fatal Len mismatch error", // MD: Description
        EQDMA_CPM5_C2H_FATAL_ERR_MASK_ADDR,
        EQDMA_CPM5_C2H_FATAL_ERR_STAT_ADDR,
        C2H_FATAL_ERR_STAT_LEN_MISMATCH_MASK,
        GLBL_ERR_STAT_ERR_C2H_ST_MASK,
        &eqdma_cpm5_hw_st_c2h_err_process
    },
    {
        EQDMA_CPM5_ST_FATAL_ERR_QID_MISMATCH, // MD: Error code for QID mismatch
        "Fatal Qid mismatch error", // MD: Description
        EQDMA_CPM5_C2H_FATAL_ERR_MASK_ADDR,
        EQDMA_CPM5_C2H_FATAL_ERR_STAT_ADDR,
        C2H_FATAL_ERR_STAT_QID_MISMATCH_MASK,
        GLBL_ERR_STAT_ERR_C2H_ST_MASK,
        &eqdma_cpm5_hw_st_c2h_err_process
    },
    {
        EQDMA_CPM5_ST_FATAL_ERR_TIMER_FIFO_RAM_RDBE, // MD: Error code for RAM double bit error
        "RAM double bit fatal error", // MD: Description
        EQDMA_CPM5_C2H_FATAL_ERR_MASK_ADDR,
        EQDMA_CPM5_C2H_FATAL_ERR_STAT_ADDR,
        C2H_FATAL_ERR_STAT_TIMER_FIFO_RAM_RDBE_MASK,
        GLBL_ERR_STAT_ERR_C2H_ST_MASK,
        &eqdma_cpm5_hw_st_c2h_err_process
    },
    {
        EQDMA_CPM5_ST_FATAL_ERR_PFCH_II_RAM_RDBE, // MD: Error code for prefetch RAM double bit error
        "RAM double bit fatal error", // MD: Description
        EQDMA_CPM5_C2H_FATAL_ERR_MASK_ADDR,
        EQDMA_CPM5_C2H_FATAL_ERR_STAT_ADDR,
        C2H_FATAL_ERR_STAT_PFCH_LL_RAM_RDBE_MASK,
        GLBL_ERR_STAT_ERR_C2H_ST_MASK,
        &eqdma_cpm5_hw_st_c2h_err_process
    },
    {
        EQDMA_CPM5_ST_FATAL_ERR_CMPT_CTXT_RAM_RDBE, // MD: Error code for completion context RAM double bit error
        "RAM double bit fatal error", // MD: Description
        EQDMA_CPM5_C2H_FATAL_ERR_MASK_ADDR,
        EQDMA_CPM5_C2H_FATAL_ERR_STAT_ADDR,
        C2H_FATAL_ERR_STAT_WRB_CTXT_RAM_RDBE_MASK,
        GLBL_ERR_STAT_ERR_C2H_ST_MASK,
        &eqdma_cpm5_hw_st_c2h_err_process
    },
    {
        EQDMA_CPM5_ST_FATAL_ERR_PFCH_CTXT_RAM_RDBE, // MD: Error code for prefetch context RAM double bit error
        "RAM double bit fatal error", // MD: Description
        EQDMA_CPM5_C2H_FATAL_ERR_MASK_ADDR,
        EQDMA_CPM5_C2H_FATAL_ERR_STAT_ADDR,
        C2H_FATAL_ERR_STAT_PFCH_CTXT_RAM_RDBE_MASK,
        GLBL_ERR_STAT_ERR_C2H_ST_MASK,
        &eqdma_cpm5_hw_st_c2h_err_process
    },
    {
        EQDMA_CPM5_ST_FATAL_ERR_DESC_REQ_FIFO_RAM_RDBE, // MD: Error code for descriptor request FIFO RAM double bit error
        "RAM double bit fatal error", // MD: Description
        EQDMA_CPM5_C2H_FATAL_ERR_MASK_ADDR,
        EQDMA_CPM5_C2H_FATAL_ERR_STAT_ADDR,
        C2H_FATAL_ERR_STAT_DESC_REQ_FIFO_RAM_RDBE_MASK,
        GLBL_ERR_STAT_ERR_C2H_ST_MASK,
        &eqdma_cpm5_hw_st_c2h_err_process
    },
    {
        EQDMA_CPM5_ST_FATAL_ERR_INT_CTXT_RAM_RDBE, // MD: Error code for interrupt context RAM double bit error
        "RAM double bit fatal error", // MD: Description
        EQDMA_CPM5_C2H_FATAL_ERR_MASK_ADDR,
        EQDMA_CPM5_C2H_FATAL_ERR_STAT_ADDR,
        C2H_FATAL_ERR_STAT_INT_CTXT_RAM_RDBE_MASK,
        GLBL_ERR_STAT_ERR_C2H_ST_MASK,
        &eqdma_cpm5_hw_st_c2h_err_process
    },
    {
        EQDMA_CPM5_ST_FATAL_ERR_CMPT_COAL_DATA_RAM_RDBE, // MD: Error code for completion coalescing data RAM double bit error
        "RAM double bit fatal error", // MD: Description
        EQDMA_CPM5_C2H_FATAL_ERR_MASK_ADDR,
        EQDMA_CPM5_C2H_FATAL_ERR_STAT_ADDR,
        C2H_FATAL_ERR_STAT_WRB_COAL_DATA_RAM_RDBE_MASK,
        GLBL_ERR_STAT_ERR_C2H_ST_MASK,
        &eqdma_cpm5_hw_st_c2h_err_process
    },
    {
        EQDMA_CPM5_ST_FATAL_ERR_CMPT_FIFO_RAM_RDBE, // MD: Error code for completion FIFO RAM double bit error
        "RAM double bit fatal error", // MD: Description
        EQDMA_CPM5_C2H_FATAL_ERR_MASK_ADDR,
        EQDMA_CPM5_C2H_FATAL_ERR_STAT_ADDR,
        C2H_FATAL_ERR_STAT_CMPT_FIFO_RAM_RDBE_MASK,
        GLBL_ERR_STAT_ERR_C2H_ST_MASK,
        &eqdma_cpm5_hw_st_c2h_err_process
    },
    {
        EQDMA_CPM5_ST_FATAL_ERR_QID_FIFO_RAM_RDBE, // MD: Error code for QID FIFO RAM double bit error
        "RAM double bit fatal error", // MD: Description
        EQDMA_CPM5_C2H_FATAL_ERR_MASK_ADDR,
        EQDMA_CPM5_C2H_FATAL_ERR_STAT_ADDR,
        C2H_FATAL_ERR_STAT_QID_FIFO_RAM_RDBE_MASK,
        GLBL_ERR_STAT_ERR_C2H_ST_MASK,
        &eqdma_cpm5_hw_st_c2h_err_process
    },
    {
        EQDMA_CPM5_ST_FATAL_ERR_PAYLOAD_FIFO_RAM_RDBE, // MD: Error code for payload FIFO RAM double bit error
        "RAM double bit fatal error", // MD: Description
        EQDMA_CPM5_C2H_FATAL_ERR_MASK_ADDR,
        EQDMA_CPM5_C2H_FATAL_ERR_STAT_ADDR,
        C2H_FATAL_ERR_STAT_PLD_FIFO_RAM_RDBE_MASK,
        GLBL_ERR_STAT_ERR_C2H_ST_MASK,
        &eqdma_cpm5_hw_st_c2h_err_process
    },
    {
        EQDMA_CPM5_ST_FATAL_ERR_WPL_DATA_PAR, // MD: Error code for WPL data parity error
        "RAM double bit fatal error", // MD: Description
        EQDMA_CPM5_C2H_FATAL_ERR_MASK_ADDR,
        EQDMA_CPM5_C2H_FATAL_ERR_STAT_ADDR,
        C2H_FATAL_ERR_STAT_WPL_DATA_PAR_ERR_MASK,
        GLBL_ERR_STAT_ERR_C2H_ST_MASK,
        &eqdma_cpm5_hw_st_c2h_err_process
    },
    {
        EQDMA_CPM5_ST_FATAL_ERR_AVL_RING_FIFO_RAM_RDBE, // MD: Error code for available ring FIFO RAM double bit error
        "RAM double bit fatal error", // MD: Description
        EQDMA_CPM5_C2H_FATAL_ERR_MASK_ADDR,
        EQDMA_CPM5_C2H_FATAL_ERR_STAT_ADDR,
        C2H_FATAL_ERR_STAT_AVL_RING_FIFO_RAM_RDBE_MASK,
        GLBL_ERR_STAT_ERR_C2H_ST_MASK,
        &eqdma_cpm5_hw_st_c2h_err_process
    },
    {
        EQDMA_CPM5_ST_FATAL_ERR_HDR_ECC_UNC, // MD: Error code for header ECC uncorrectable error
        "RAM double bit fatal error", // MD: Description
        EQDMA_CPM5_C2H_FATAL_ERR_MASK_ADDR,
        EQDMA_CPM5_C2H_FATAL_ERR_STAT_ADDR,
        C2H_FATAL_ERR_STAT_HDR_ECC_UNC_ERR_MASK,
        GLBL_ERR_STAT_ERR_C2H_ST_MASK,
        &eqdma_cpm5_hw_st_c2h_err_process
    },
    {
        EQDMA_CPM5_ST_FATAL_ERR_ALL, // MD: Error code for all fatal errors
        "All fatal errors", // MD: Description
        EQDMA_CPM5_C2H_FATAL_ERR_MASK_ADDR,
        EQDMA_CPM5_C2H_FATAL_ERR_STAT_ADDR,
        EQDMA_CPM5_C2H_FATAL_ERR_ALL_MASK,
        GLBL_ERR_STAT_ERR_C2H_ST_MASK,
        &eqdma_cpm5_hw_st_c2h_err_process
    },

    /* MD: H2C St Errors */
    {
        EQDMA_CPM5_ST_H2C_ERR_ZERO_LEN_DESC, // MD: Error code for zero length descriptor
        "Zero length descriptor error", // MD: Description
        EQDMA_CPM5_H2C_ERR_MASK_ADDR, // MD: Mask address for the error
        EQDMA_CPM5_H2C_ERR_STAT_ADDR, // MD: Status address for the error
        H2C_ERR_STAT_ZERO_LEN_DS_MASK, // MD: Specific mask for zero length descriptor
        GLBL_ERR_STAT_ERR_H2C_ST_MASK, // MD: General H2C error status mask
        &eqdma_cpm5_hw_st_h2c_err_process // MD: Function to process this error
    },
    {
        EQDMA_CPM5_ST_H2C_ERR_SDI_MRKR_REQ_MOP, // MD: Error code for non-EOP descriptor
        "A non-EOP descriptor received", // MD: Description
        EQDMA_CPM5_H2C_ERR_MASK_ADDR,
        EQDMA_CPM5_H2C_ERR_STAT_ADDR,
        H2C_ERR_STAT_SDI_MRKR_REQ_MOP_ERR_MASK,
        GLBL_ERR_STAT_ERR_H2C_ST_MASK,
        &eqdma_cpm5_hw_st_h2c_err_process
    },
    {
        EQDMA_CPM5_ST_H2C_ERR_NO_DMA_DSC, // MD: Error code for no DMA descriptor
        "No DMA descriptor received error", // MD: Description
        EQDMA_CPM5_H2C_ERR_MASK_ADDR,
        EQDMA_CPM5_H2C_ERR_STAT_ADDR,
        H2C_ERR_STAT_NO_DMA_DS_MASK,
        GLBL_ERR_STAT_ERR_H2C_ST_MASK,
        &eqdma_cpm5_hw_st_h2c_err_process
    },
    {
        EQDMA_CPM5_ST_H2C_ERR_SBE, // MD: Error code for single bit error
        "Single bit error detected on H2C-ST data error", // MD: Description
        EQDMA_CPM5_H2C_ERR_MASK_ADDR,
        EQDMA_CPM5_H2C_ERR_STAT_ADDR,
        H2C_ERR_STAT_SBE_MASK,
        GLBL_ERR_STAT_ERR_H2C_ST_MASK,
        &eqdma_cpm5_hw_st_h2c_err_process
    },
    {
        EQDMA_CPM5_ST_H2C_ERR_DBE, // MD: Error code for double bit error
        "Double bit error detected on H2C-ST data error", // MD: Description
        EQDMA_CPM5_H2C_ERR_MASK_ADDR,
        EQDMA_CPM5_H2C_ERR_STAT_ADDR,
        H2C_ERR_STAT_DBE_MASK,
        GLBL_ERR_STAT_ERR_H2C_ST_MASK,
        &eqdma_cpm5_hw_st_h2c_err_process
    },
    {
        EQDMA_CPM5_ST_H2C_ERR_PAR, // MD: Error code for internal data parity error
        "Internal data parity error", // MD: Description
        EQDMA_CPM5_H2C_ERR_MASK_ADDR,
        EQDMA_CPM5_H2C_ERR_STAT_ADDR,
        H2C_ERR_STAT_PAR_ERR_MASK,
        GLBL_ERR_STAT_ERR_H2C_ST_MASK,
        &eqdma_cpm5_hw_st_h2c_err_process
    },
    {
        EQDMA_CPM5_ST_H2C_ERR_ALL, // MD: Error code for all H2C errors
        "All H2C errors", // MD: Description
        EQDMA_CPM5_H2C_ERR_MASK_ADDR,
        EQDMA_CPM5_H2C_ERR_STAT_ADDR,
        EQDMA_CPM5_H2C_ERR_ALL_MASK,
        GLBL_ERR_STAT_ERR_H2C_ST_MASK,
        &eqdma_cpm5_hw_st_h2c_err_process
    },



    /* MD: SBE Errors */
    {
        EQDMA_CPM5_SBE_1_ERR_RC_RRQ_EVEN_RAM, // MD: Error code for RC RRQ Even RAM single bit ECC error
        "RC RRQ Even RAM single bit ECC error.", // MD: Description
        EQDMA_CPM5_RAM_SBE_MSK_1_A_ADDR, // MD: Mask address for the error
        EQDMA_CPM5_RAM_SBE_STS_1_A_ADDR, // MD: Status address for the error
        RAM_SBE_STS_1_A_RC_RRQ_EVEN_RAM_MASK, // MD: Specific mask for this error
        GLBL_ERR_STAT_ERR_RAM_SBE_MASK, // MD: General RAM SBE error status mask
        &eqdma_cpm5_hw_ram_sbe_err_process // MD: Function to process this error
    },
    {
        EQDMA_CPM5_SBE_1_ERR_TAG_ODD_RAM, // MD: Error code for Tag Odd RAM single bit ECC error
        "Tag Odd Ram single bit ECC error.", // MD: Description
        EQDMA_CPM5_RAM_SBE_MSK_1_A_ADDR,
        EQDMA_CPM5_RAM_SBE_STS_1_A_ADDR,
        RAM_SBE_STS_1_A_TAG_ODD_RAM_MASK,
        GLBL_ERR_STAT_ERR_RAM_SBE_MASK,
        &eqdma_cpm5_hw_ram_sbe_err_process
    },
    {
        EQDMA_CPM5_SBE_1_ERR_TAG_EVEN_RAM, // MD: Error code for Tag Even RAM single bit ECC error
        "Tag Even Ram single bit ECC error.", // MD: Description
        EQDMA_CPM5_RAM_SBE_MSK_1_A_ADDR,
        EQDMA_CPM5_RAM_SBE_STS_1_A_ADDR,
        RAM_SBE_STS_1_A_TAG_EVEN_RAM_MASK,
        GLBL_ERR_STAT_ERR_RAM_SBE_MASK,
        &eqdma_cpm5_hw_ram_sbe_err_process
    },
    {
        EQDMA_CPM5_SBE_1_ERR_PFCH_CTXT_CAM_RAM_0, // MD: Error code for Pfch Ctxt CAM RAM 0 single bit ECC error
        "Pfch Ctxt CAM RAM 0 single bit ECC error.", // MD: Description
        EQDMA_CPM5_RAM_SBE_MSK_1_A_ADDR,
        EQDMA_CPM5_RAM_SBE_STS_1_A_ADDR,
        RAM_SBE_STS_1_A_PFCH_CTXT_CAM_RAM_0_MASK,
        GLBL_ERR_STAT_ERR_RAM_SBE_MASK,
        &eqdma_cpm5_hw_ram_sbe_err_process
    },
    {
        EQDMA_CPM5_SBE_1_ERR_PFCH_CTXT_CAM_RAM_1, // MD: Error code for Pfch Ctxt CAM RAM 1 single bit ECC error
        "Pfch Ctxt CAM RAM 1 single bit ECC error.", // MD: Description
        EQDMA_CPM5_RAM_SBE_MSK_1_A_ADDR,
        EQDMA_CPM5_RAM_SBE_STS_1_A_ADDR,
        RAM_SBE_STS_1_A_PFCH_CTXT_CAM_RAM_1_MASK,
        GLBL_ERR_STAT_ERR_RAM_SBE_MASK,
        &eqdma_cpm5_hw_ram_sbe_err_process
    },
    {
        EQDMA_CPM5_SBE_1_ERR_ALL, // MD: Error code for all SBE errors
        "All SBE Errors.", // MD: Description
        EQDMA_CPM5_RAM_SBE_MSK_1_A_ADDR,
        EQDMA_CPM5_RAM_SBE_STS_1_A_ADDR,
        EQDMA_CPM5_SBE_ERR_ALL_MASK,
        GLBL_ERR_STAT_ERR_RAM_SBE_MASK,
        &eqdma_cpm5_hw_ram_sbe_err_process
    },
    {
        EQDMA_CPM5_SBE_ERR_MI_H2C0_DAT, // MD: Error code for H2C MM data buffer single bit ECC error
        "H2C MM data buffer single bit ECC error", // MD: Description
        EQDMA_CPM5_RAM_SBE_MSK_A_ADDR,
        EQDMA_CPM5_RAM_SBE_STS_A_ADDR,
        RAM_SBE_STS_A_MI_H2C0_DAT_MASK,
        GLBL_ERR_STAT_ERR_RAM_SBE_MASK,
        &eqdma_cpm5_hw_ram_sbe_err_process
    },
    {
        EQDMA_CPM5_SBE_ERR_MI_H2C1_DAT, // MD: Error code for H2C MM data buffer single bit ECC error
        "H2C MM data buffer single bit ECC error", // MD: Description
        EQDMA_CPM5_RAM_SBE_MSK_A_ADDR,
        EQDMA_CPM5_RAM_SBE_STS_A_ADDR,
        RAM_SBE_STS_A_MI_H2C1_DAT_MASK,
        GLBL_ERR_STAT_ERR_RAM_SBE_MASK,
        &eqdma_cpm5_hw_ram_sbe_err_process
    },
    {
        EQDMA_CPM5_SBE_ERR_MI_H2C2_DAT, // MD: Error code for H2C MM data buffer single bit ECC error
        "H2C MM data buffer single bit ECC error", // MD: Description
        EQDMA_CPM5_RAM_SBE_MSK_A_ADDR,
        EQDMA_CPM5_RAM_SBE_STS_A_ADDR,
        RAM_SBE_STS_A_MI_H2C2_DAT_MASK,
        GLBL_ERR_STAT_ERR_RAM_SBE_MASK,
        &eqdma_cpm5_hw_ram_sbe_err_process
    },
    {
        EQDMA_CPM5_SBE_ERR_MI_H2C3_DAT, // MD: Error code for H2C MM data buffer single bit ECC error
        "H2C MM data buffer single bit ECC error", // MD: Description
        EQDMA_CPM5_RAM_SBE_MSK_A_ADDR,
        EQDMA_CPM5_RAM_SBE_STS_A_ADDR,
        RAM_SBE_STS_A_MI_H2C3_DAT_MASK,
        GLBL_ERR_STAT_ERR_RAM_SBE_MASK,
        &eqdma_cpm5_hw_ram_sbe_err_process
    },
    {
        EQDMA_CPM5_SBE_ERR_MI_C2H0_DAT, // MD: Error code for C2H MM data buffer single bit ECC error
        "C2H MM data buffer single bit ECC error", // MD: Description
        EQDMA_CPM5_RAM_SBE_MSK_A_ADDR,
        EQDMA_CPM5_RAM_SBE_STS_A_ADDR,
        RAM_SBE_STS_A_MI_C2H0_DAT_MASK,
        GLBL_ERR_STAT_ERR_RAM_SBE_MASK,
        &eqdma_cpm5_hw_ram_sbe_err_process
    },
    {
        EQDMA_CPM5_SBE_ERR_MI_C2H1_DAT, // MD: Error code for C2H MM data buffer single bit ECC error
        "C2H MM data buffer single bit ECC error", // MD: Description
        EQDMA_CPM5_RAM_SBE_MSK_A_ADDR,
        EQDMA_CPM5_RAM_SBE_STS_A_ADDR,
        RAM_SBE_STS_A_MI_C2H1_DAT_MASK,
        GLBL_ERR_STAT_ERR_RAM_SBE_MASK,
        &eqdma_cpm5_hw_ram_sbe_err_process
    },
    {
        EQDMA_CPM5_SBE_ERR_MI_C2H2_DAT, // MD: Error code for C2H MM data buffer single bit ECC error
        "C2H MM data buffer single bit ECC error", // MD: Description
        EQDMA_CPM5_RAM_SBE_MSK_A_ADDR,
        EQDMA_CPM5_RAM_SBE_STS_A_ADDR,
        RAM_SBE_STS_A_MI_C2H2_DAT_MASK,
        GLBL_ERR_STAT_ERR_RAM_SBE_MASK,
        &eqdma_cpm5_hw_ram_sbe_err_process
    },
    {
        EQDMA_CPM5_SBE_ERR_MI_C2H3_DAT, // MD: Error code for C2H MM data buffer single bit ECC error
        "C2H MM data buffer single bit ECC error", // MD: Description
        EQDMA_CPM5_RAM_SBE_MSK_A_ADDR,
        EQDMA_CPM5_RAM_SBE_STS_A_ADDR,
        RAM_SBE_STS_A_MI_C2H3_DAT_MASK,
        GLBL_ERR_STAT_ERR_RAM_SBE_MASK,
        &eqdma_cpm5_hw_ram_sbe_err_process
    },
    {
        EQDMA_CPM5_SBE_ERR_H2C_RD_BRG_DAT, // MD: Error code for bridge master read single bit ECC error
        "Bridge master read single bit ECC error", // MD: Description
        EQDMA_CPM5_RAM_SBE_MSK_A_ADDR,
        EQDMA_CPM5_RAM_SBE_STS_A_ADDR,
        RAM_SBE_STS_A_H2C_RD_BRG_DAT_MASK,
        GLBL_ERR_STAT_ERR_RAM_SBE_MASK,
        &eqdma_cpm5_hw_ram_sbe_err_process
    },
    {
        EQDMA_CPM5_SBE_ERR_H2C_WR_BRG_DAT, // MD: Error code for bridge master write single bit ECC error
        "Bridge master write single bit ECC error", // MD: Description
        EQDMA_CPM5_RAM_SBE_MSK_A_ADDR,
        EQDMA_CPM5_RAM_SBE_STS_A_ADDR,
        RAM_SBE_STS_A_H2C_WR_BRG_DAT_MASK,
        GLBL_ERR_STAT_ERR_RAM_SBE_MASK,
        &eqdma_cpm5_hw_ram_sbe_err_process
    },
    {
        EQDMA_CPM5_SBE_ERR_C2H_RD_BRG_DAT, // MD: Error code for bridge slave read data buffer single bit ECC error
        "Bridge slave read data buffer single bit ECC error", // MD: Description
        EQDMA_CPM5_RAM_SBE_MSK_A_ADDR,
        EQDMA_CPM5_RAM_SBE_STS_A_ADDR,
        RAM_SBE_STS_A_C2H_RD_BRG_DAT_MASK,
        GLBL_ERR_STAT_ERR_RAM_SBE_MASK,
        &eqdma_cpm5_hw_ram_sbe_err_process
    },
    {
        EQDMA_CPM5_SBE_ERR_C2H_WR_BRG_DAT, // MD: Error code for bridge slave write data buffer single bit ECC error
        "Bridge slave write data buffer single bit ECC error", // MD: Description
        EQDMA_CPM5_RAM_SBE_MSK_A_ADDR,
        EQDMA_CPM5_RAM_SBE_STS_A_ADDR,
        RAM_SBE_STS_A_C2H_WR_BRG_DAT_MASK,
        GLBL_ERR_STAT_ERR_RAM_SBE_MASK,
        &eqdma_cpm5_hw_ram_sbe_err_process
    },
    {
        EQDMA_CPM5_SBE_ERR_FUNC_MAP, // MD: Error code for function map RAM single bit ECC error
        "Function map RAM single bit ECC error", // MD: Description
        EQDMA_CPM5_RAM_SBE_MSK_A_ADDR,
        EQDMA_CPM5_RAM_SBE_STS_A_ADDR,
        RAM_SBE_STS_A_FUNC_MAP_MASK,
        GLBL_ERR_STAT_ERR_RAM_SBE_MASK,
        &eqdma_cpm5_hw_ram_sbe_err_process
    },
    {
        EQDMA_CPM5_SBE_ERR_DSC_HW_CTXT, // MD: Error code for descriptor engine hardware context RAM single bit ECC error
        "Descriptor engine hardware context RAM single bit ECC error", // MD: Description
        EQDMA_CPM5_RAM_SBE_MSK_A_ADDR,
        EQDMA_CPM5_RAM_SBE_STS_A_ADDR,
        RAM_SBE_STS_A_DSC_HW_CTXT_MASK,
        GLBL_ERR_STAT_ERR_RAM_SBE_MASK,
        &eqdma_cpm5_hw_ram_sbe_err_process
    },
    {
        EQDMA_CPM5_SBE_ERR_DSC_CRD_RCV, // MD: Error code for descriptor engine receive credit context RAM single bit ECC error
        "Descriptor engine receive credit context RAM single bit ECC error", // MD: Description
        EQDMA_CPM5_RAM_SBE_MSK_A_ADDR,
        EQDMA_CPM5_RAM_SBE_STS_A_ADDR,
        RAM_SBE_STS_A_DSC_CRD_RCV_MASK,
        GLBL_ERR_STAT_ERR_RAM_SBE_MASK,
        &eqdma_cpm5_hw_ram_sbe_err_process
    },
    {
        EQDMA_CPM5_SBE_ERR_DSC_SW_CTXT, // MD: Error code for descriptor engine software context RAM single bit ECC error
        "Descriptor engine software context RAM single bit ECC error", // MD: Description
        EQDMA_CPM5_RAM_SBE_MSK_A_ADDR,
        EQDMA_CPM5_RAM_SBE_STS_A_ADDR,
        RAM_SBE_STS_A_DSC_SW_CTXT_MASK,
        GLBL_ERR_STAT_ERR_RAM_SBE_MASK,
        &eqdma_cpm5_hw_ram_sbe_err_process
    },
    {
        EQDMA_CPM5_SBE_ERR_DSC_CPLI, // MD: Error code for descriptor engine fetch completion information RAM single bit ECC error
        "Descriptor engine fetch completion information RAM single bit ECC error", // MD: Description
        EQDMA_CPM5_RAM_SBE_MSK_A_ADDR,
        EQDMA_CPM5_RAM_SBE_STS_A_ADDR,
        RAM_SBE_STS_A_DSC_CPLI_MASK,
        GLBL_ERR_STAT_ERR_RAM_SBE_MASK,
        &eqdma_cpm5_hw_ram_sbe_err_process
    },
    {
        EQDMA_CPM5_SBE_ERR_DSC_CPLD, // MD: Error code for descriptor engine fetch completion data RAM single bit ECC error
        "Descriptor engine fetch completion data RAM single bit ECC error", // MD: Description
        EQDMA_CPM5_RAM_SBE_MSK_A_ADDR,
        EQDMA_CPM5_RAM_SBE_STS_A_ADDR,
        RAM_SBE_STS_A_DSC_CPLD_MASK,
        GLBL_ERR_STAT_ERR_RAM_SBE_MASK,
        &eqdma_cpm5_hw_ram_sbe_err_process
    },
    {
        EQDMA_CPM5_SBE_ERR_MI_TL_SLV_FIFO_RAM, // MD: Error code for TL Slave FIFO RAM single bit ECC error
        "TL Slave FIFO RAM single bit ECC error", // MD: Description
        EQDMA_CPM5_RAM_SBE_MSK_A_ADDR,
        EQDMA_CPM5_RAM_SBE_STS_A_ADDR,
        RAM_SBE_STS_A_MI_TL_SLV_FIFO_RAM_MASK,
        GLBL_ERR_STAT_ERR_RAM_SBE_MASK,
        &eqdma_cpm5_hw_ram_sbe_err_process
    },
    {
        EQDMA_CPM5_SBE_ERR_TIMER_FIFO_RAM, // MD: Error code for Timer FIFO RAM single bit ECC error
        "Timer fifo RAM single bit ECC error", // MD: Description
        EQDMA_CPM5_RAM_SBE_MSK_A_ADDR,
        EQDMA_CPM5_RAM_SBE_STS_A_ADDR,
        RAM_SBE_STS_A_TIMER_FIFO_RAM_MASK,
        GLBL_ERR_STAT_ERR_RAM_SBE_MASK,
        &eqdma_cpm5_hw_ram_sbe_err_process
    },
    {
        EQDMA_CPM5_SBE_ERR_QID_FIFO_RAM, // MD: Error code for C2H ST QID FIFO RAM single bit ECC error
        "C2H ST QID FIFO RAM single bit ECC error", // MD: Description
        EQDMA_CPM5_RAM_SBE_MSK_A_ADDR,
        EQDMA_CPM5_RAM_SBE_STS_A_ADDR,
        RAM_SBE_STS_A_QID_FIFO_RAM_MASK,
        GLBL_ERR_STAT_ERR_RAM_SBE_MASK,
        &eqdma_cpm5_hw_ram_sbe_err_process
    },
    {
        EQDMA_CPM5_SBE_ERR_WRB_COAL_DATA_RAM, // MD: Error code for Writeback Coalescing RAM single bit ECC error
        "Writeback Coalescing RAM single bit ECC error", // MD: Description
        EQDMA_CPM5_RAM_SBE_MSK_A_ADDR,
        EQDMA_CPM5_RAM_SBE_STS_A_ADDR,
        RAM_SBE_STS_A_WRB_COAL_DATA_RAM_MASK,
        GLBL_ERR_STAT_ERR_RAM_SBE_MASK,
        &eqdma_cpm5_hw_ram_sbe_err_process
    },
    {
        EQDMA_CPM5_SBE_ERR_INT_CTXT_RAM, // MD: Error code for Interrupt context RAM single bit ECC error
        "Interrupt context RAM single bit ECC error", // MD: Description
        EQDMA_CPM5_RAM_SBE_MSK_A_ADDR,
        EQDMA_CPM5_RAM_SBE_STS_A_ADDR,
        RAM_SBE_STS_A_INT_CTXT_RAM_MASK,
        GLBL_ERR_STAT_ERR_RAM_SBE_MASK,
        &eqdma_cpm5_hw_ram_sbe_err_process
    },
    {
        EQDMA_CPM5_SBE_ERR_DESC_REQ_FIFO_RAM, // MD: Error code for C2H ST descriptor request RAM single bit ECC error
        "C2H ST descriptor request RAM single bit ECC error", // MD: Description
        EQDMA_CPM5_RAM_SBE_MSK_A_ADDR,
        EQDMA_CPM5_RAM_SBE_STS_A_ADDR,
        RAM_SBE_STS_A_DESC_REQ_FIFO_RAM_MASK,
        GLBL_ERR_STAT_ERR_RAM_SBE_MASK,
        &eqdma_cpm5_hw_ram_sbe_err_process
    },
    {
        EQDMA_CPM5_SBE_ERR_PFCH_CTXT_RAM, // MD: Error code for C2H ST prefetch RAM single bit ECC error
        "C2H ST prefetch RAM single bit ECC error", // MD: Description
        EQDMA_CPM5_RAM_SBE_MSK_A_ADDR,
        EQDMA_CPM5_RAM_SBE_STS_A_ADDR,
        RAM_SBE_STS_A_PFCH_CTXT_RAM_MASK,
        GLBL_ERR_STAT_ERR_RAM_SBE_MASK,
        &eqdma_cpm5_hw_ram_sbe_err_process
    },
    {
        EQDMA_CPM5_SBE_ERR_WRB_CTXT_RAM, // MD: Error code for C2H ST completion context RAM single bit ECC error
        "C2H ST completion context RAM single bit ECC error", // MD: Description
        EQDMA_CPM5_RAM_SBE_MSK_A_ADDR,
        EQDMA_CPM5_RAM_SBE_STS_A_ADDR,
        RAM_SBE_STS_A_WRB_CTXT_RAM_MASK,
        GLBL_ERR_STAT_ERR_RAM_SBE_MASK,
        &eqdma_cpm5_hw_ram_sbe_err_process
    },
    {
        EQDMA_CPM5_SBE_ERR_PFCH_LL_RAM, // MD: Error code for C2H ST prefetch list RAM single bit ECC error
        "C2H ST prefetch list RAM single bit ECC error", // MD: Description
        EQDMA_CPM5_RAM_SBE_MSK_A_ADDR,
        EQDMA_CPM5_RAM_SBE_STS_A_ADDR,
        RAM_SBE_STS_A_PFCH_LL_RAM_MASK,
        GLBL_ERR_STAT_ERR_RAM_SBE_MASK,
        &eqdma_cpm5_hw_ram_sbe_err_process
    },
    {
        EQDMA_CPM5_SBE_ERR_PEND_FIFO_RAM, // MD: Error code for Pend FIFO RAM single bit ECC error
        "Pend FIFO RAM single bit ECC error", // MD: Description
        EQDMA_CPM5_RAM_SBE_MSK_A_ADDR,
        EQDMA_CPM5_RAM_SBE_STS_A_ADDR,
        RAM_SBE_STS_A_PEND_FIFO_RAM_MASK,
        GLBL_ERR_STAT_ERR_RAM_SBE_MASK,
        &eqdma_cpm5_hw_ram_sbe_err_process
    },
    {
        EQDMA_CPM5_SBE_ERR_RC_RRQ_ODD_RAM, // MD: Error code for RC RRQ Odd RAM single bit ECC error
        "RC RRQ Odd RAM single bit ECC error.", // MD: Description
        EQDMA_CPM5_RAM_SBE_MSK_A_ADDR,
        EQDMA_CPM5_RAM_SBE_STS_A_ADDR,
        RAM_SBE_STS_A_RC_RRQ_ODD_RAM_MASK,
        GLBL_ERR_STAT_ERR_RAM_SBE_MASK,
        &eqdma_cpm5_hw_ram_sbe_err_process
    },
    {
        EQDMA_CPM5_SBE_ERR_ALL, // MD: Error code for all SBE errors
        "All SBE errors", // MD: Description
        EQDMA_CPM5_RAM_SBE_MSK_A_ADDR,
        EQDMA_CPM5_RAM_SBE_STS_A_ADDR,
        EQDMA_CPM5_SBE_ERR_ALL_MASK,
        GLBL_ERR_STAT_ERR_RAM_SBE_MASK,
        &eqdma_cpm5_hw_ram_sbe_err_process
    },

    /* MD: DBE Errors */
    {
        EQDMA_CPM5_DBE_1_ERR_RC_RRQ_EVEN_RAM, // MD: Error code for RC RRQ Even RAM double bit ECC error
        "RC RRQ Odd RAM double bit ECC error.", // MD: Description
        EQDMA_CPM5_RAM_DBE_MSK_1_A_ADDR, // MD: Mask address for the error
        EQDMA_CPM5_RAM_DBE_STS_1_A_ADDR, // MD: Status address for the error
        RAM_DBE_STS_1_A_RC_RRQ_EVEN_RAM_MASK, // MD: Specific mask for this error
        GLBL_ERR_STAT_ERR_RAM_DBE_MASK, // MD: General RAM DBE error status mask
        &eqdma_cpm5_hw_ram_dbe_err_process // MD: Function to process this error
    },
    {
        EQDMA_CPM5_DBE_1_ERR_TAG_ODD_RAM, // MD: Error code for Tag Odd RAM double bit ECC error
        "Tag Odd Ram double bit ECC error.", // MD: Description
        EQDMA_CPM5_RAM_DBE_MSK_1_A_ADDR,
        EQDMA_CPM5_RAM_DBE_STS_1_A_ADDR,
        RAM_DBE_STS_1_A_TAG_ODD_RAM_MASK,
        GLBL_ERR_STAT_ERR_RAM_DBE_MASK,
        &eqdma_cpm5_hw_ram_dbe_err_process
    },
    {
        EQDMA_CPM5_DBE_1_ERR_TAG_EVEN_RAM, // MD: Error code for Tag Even RAM double bit ECC error
        "Tag Even Ram double bit ECC error.", // MD: Description
        EQDMA_CPM5_RAM_DBE_MSK_1_A_ADDR,
        EQDMA_CPM5_RAM_DBE_STS_1_A_ADDR,
        RAM_DBE_STS_1_A_TAG_EVEN_RAM_MASK,
        GLBL_ERR_STAT_ERR_RAM_DBE_MASK,
        &eqdma_cpm5_hw_ram_dbe_err_process
    },
    {
        EQDMA_CPM5_DBE_1_ERR_PFCH_CTXT_CAM_RAM_0, // MD: Error code for Pfch Ctxt CAM RAM 0 double bit ECC error
        "Pfch Ctxt CAM RAM 0 double bit ECC error.", // MD: Description
        EQDMA_CPM5_RAM_DBE_MSK_1_A_ADDR,
        EQDMA_CPM5_RAM_DBE_STS_1_A_ADDR,
        RAM_DBE_STS_1_A_PFCH_CTXT_CAM_RAM_0_MASK,
        GLBL_ERR_STAT_ERR_RAM_DBE_MASK,
        &eqdma_cpm5_hw_ram_dbe_err_process
    },
    {
        EQDMA_CPM5_DBE_1_ERR_PFCH_CTXT_CAM_RAM_1, // MD: Error code for Pfch Ctxt CAM RAM 1 double bit ECC error
        "Pfch Ctxt CAM RAM double bit ECC error.", // MD: Description
        EQDMA_CPM5_RAM_DBE_MSK_1_A_ADDR,
        EQDMA_CPM5_RAM_DBE_STS_1_A_ADDR,
        RAM_DBE_STS_1_A_PFCH_CTXT_CAM_RAM_0_MASK,
        GLBL_ERR_STAT_ERR_RAM_DBE_MASK,
        &eqdma_cpm5_hw_ram_dbe_err_process
    },
    {
        EQDMA_CPM5_DBE_1_ERR_ALL, // MD: Error code for all DBE errors
        "All DBE errors", // MD: Description
        EQDMA_CPM5_RAM_DBE_MSK_1_A_ADDR,
        EQDMA_CPM5_RAM_DBE_STS_1_A_ADDR,
        EQDMA_CPM5_DBE_ERR_ALL_MASK,
        GLBL_ERR_STAT_ERR_RAM_DBE_MASK,
        &eqdma_cpm5_hw_ram_dbe_err_process
    },
    {
        EQDMA_CPM5_DBE_ERR_MI_H2C0_DAT, // MD: Error code for H2C MM data buffer double bit ECC error
        "H2C MM data buffer double bit ECC error", // MD: Description
        EQDMA_CPM5_RAM_DBE_MSK_A_ADDR,
        EQDMA_CPM5_RAM_DBE_STS_A_ADDR,
        RAM_DBE_STS_A_MI_H2C0_DAT_MASK,
        GLBL_ERR_STAT_ERR_RAM_DBE_MASK,
        &eqdma_cpm5_hw_ram_dbe_err_process
    },
    {
        EQDMA_CPM5_DBE_ERR_MI_H2C1_DAT, // MD: Error code for H2C MM data buffer double bit ECC error
        "H2C MM data buffer double bit ECC error", // MD: Description
        EQDMA_CPM5_RAM_DBE_MSK_A_ADDR,
        EQDMA_CPM5_RAM_DBE_STS_A_ADDR,
        RAM_DBE_STS_A_MI_H2C1_DAT_MASK,
        GLBL_ERR_STAT_ERR_RAM_DBE_MASK,
        &eqdma_cpm5_hw_ram_dbe_err_process
    },
    {
        EQDMA_CPM5_DBE_ERR_MI_H2C2_DAT, // MD: Error code for H2C MM data buffer double bit ECC error
        "H2C MM data buffer double bit ECC error", // MD: Description
        EQDMA_CPM5_RAM_DBE_MSK_A_ADDR,
        EQDMA_CPM5_RAM_DBE_STS_A_ADDR,
        RAM_DBE_STS_A_MI_H2C2_DAT_MASK,
        GLBL_ERR_STAT_ERR_RAM_DBE_MASK,
        &eqdma_cpm5_hw_ram_dbe_err_process
    },
    {
        EQDMA_CPM5_DBE_ERR_MI_H2C3_DAT, // MD: Error code for H2C MM data buffer double bit ECC error
        "H2C MM data buffer double bit ECC error", // MD: Description
        EQDMA_CPM5_RAM_DBE_MSK_A_ADDR,
        EQDMA_CPM5_RAM_DBE_STS_A_ADDR,
        RAM_DBE_STS_A_MI_H2C3_DAT_MASK,
        GLBL_ERR_STAT_ERR_RAM_DBE_MASK,
        &eqdma_cpm5_hw_ram_dbe_err_process
    },
    {
        EQDMA_CPM5_DBE_ERR_MI_C2H0_DAT, // MD: Error code for C2H MM data buffer double bit ECC error
        "C2H MM data buffer double bit ECC error", // MD: Description
        EQDMA_CPM5_RAM_DBE_MSK_A_ADDR,
        EQDMA_CPM5_RAM_DBE_STS_A_ADDR,
        RAM_DBE_STS_A_MI_C2H0_DAT_MASK,
        GLBL_ERR_STAT_ERR_RAM_DBE_MASK,
        &eqdma_cpm5_hw_ram_dbe_err_process
    },
    {
        EQDMA_CPM5_DBE_ERR_MI_C2H1_DAT, // MD: Error code for C2H MM data buffer double bit ECC error
        "C2H MM data buffer double bit ECC error", // MD: Description
        EQDMA_CPM5_RAM_DBE_MSK_A_ADDR,
        EQDMA_CPM5_RAM_DBE_STS_A_ADDR,
        RAM_DBE_STS_A_MI_C2H1_DAT_MASK,
        GLBL_ERR_STAT_ERR_RAM_DBE_MASK,
        &eqdma_cpm5_hw_ram_dbe_err_process
    },
    {
        EQDMA_CPM5_DBE_ERR_MI_C2H2_DAT, // MD: Error code for C2H MM data buffer double bit ECC error
        "C2H MM data buffer double bit ECC error", // MD: Description
        EQDMA_CPM5_RAM_DBE_MSK_A_ADDR,
        EQDMA_CPM5_RAM_DBE_STS_A_ADDR,
        RAM_DBE_STS_A_MI_C2H2_DAT_MASK,
        GLBL_ERR_STAT_ERR_RAM_DBE_MASK,
        &eqdma_cpm5_hw_ram_dbe_err_process
    },
    {
        EQDMA_CPM5_DBE_ERR_MI_C2H3_DAT, // MD: Error code for C2H MM data buffer double bit ECC error
        "C2H MM data buffer double bit ECC error", // MD: Description
        EQDMA_CPM5_RAM_DBE_MSK_A_ADDR,
        EQDMA_CPM5_RAM_DBE_STS_A_ADDR,
        RAM_DBE_STS_A_MI_C2H3_DAT_MASK,
        GLBL_ERR_STAT_ERR_RAM_DBE_MASK,
        &eqdma_cpm5_hw_ram_dbe_err_process
    },
    {
        EQDMA_CPM5_DBE_ERR_H2C_RD_BRG_DAT, // MD: Error code for bridge master read double bit ECC error
        "Bridge master read double bit ECC error", // MD: Description
        EQDMA_CPM5_RAM_DBE_MSK_A_ADDR,
        EQDMA_CPM5_RAM_DBE_STS_A_ADDR,
        RAM_DBE_STS_A_H2C_RD_BRG_DAT_MASK,
        GLBL_ERR_STAT_ERR_RAM_DBE_MASK,
        &eqdma_cpm5_hw_ram_dbe_err_process
    },
    {
        EQDMA_CPM5_DBE_ERR_H2C_WR_BRG_DAT, // MD: Error code for bridge master write double bit ECC error
        "Bridge master write double bit ECC error", // MD: Description
        EQDMA_CPM5_RAM_DBE_MSK_A_ADDR,
        EQDMA_CPM5_RAM_DBE_STS_A_ADDR,
        RAM_DBE_STS_A_H2C_WR_BRG_DAT_MASK,
        GLBL_ERR_STAT_ERR_RAM_DBE_MASK,
        &eqdma_cpm5_hw_ram_dbe_err_process
    },
    {
        EQDMA_CPM5_DBE_ERR_C2H_RD_BRG_DAT, // MD: Error code for bridge slave read data buffer double bit ECC error
        "Bridge slave read data buffer double bit ECC error", // MD: Description
        EQDMA_CPM5_RAM_DBE_MSK_A_ADDR,
        EQDMA_CPM5_RAM_DBE_STS_A_ADDR,
        RAM_DBE_STS_A_C2H_RD_BRG_DAT_MASK,
        GLBL_ERR_STAT_ERR_RAM_DBE_MASK,
        &eqdma_cpm5_hw_ram_dbe_err_process
    },
    {
        EQDMA_CPM5_DBE_ERR_C2H_WR_BRG_DAT, // MD: Error code for bridge slave write data buffer double bit ECC error
        "Bridge slave write data buffer double bit ECC error", // MD: Description
        EQDMA_CPM5_RAM_DBE_MSK_A_ADDR,
        EQDMA_CPM5_RAM_DBE_STS_A_ADDR,
        RAM_DBE_STS_A_C2H_WR_BRG_DAT_MASK,
        GLBL_ERR_STAT_ERR_RAM_DBE_MASK,
        &eqdma_cpm5_hw_ram_dbe_err_process
    },
    {
        EQDMA_CPM5_DBE_ERR_FUNC_MAP, // MD: Error code for function map RAM double bit ECC error
        "Function map RAM double bit ECC error", // MD: Description
        EQDMA_CPM5_RAM_DBE_MSK_A_ADDR,
        EQDMA_CPM5_RAM_DBE_STS_A_ADDR,
        RAM_DBE_STS_A_FUNC_MAP_MASK,
        GLBL_ERR_STAT_ERR_RAM_DBE_MASK,
        &eqdma_cpm5_hw_ram_dbe_err_process
    },
    {
        EQDMA_CPM5_DBE_ERR_DSC_HW_CTXT, // MD: Error code for descriptor engine hardware context RAM double bit ECC error
        "Descriptor engine hardware context RAM double bit ECC error", // MD: Description
        EQDMA_CPM5_RAM_DBE_MSK_A_ADDR,
        EQDMA_CPM5_RAM_DBE_STS_A_ADDR,
        RAM_DBE_STS_A_DSC_HW_CTXT_MASK,
        GLBL_ERR_STAT_ERR_RAM_DBE_MASK,
        &eqdma_cpm5_hw_ram_dbe_err_process
    },
    {
        EQDMA_CPM5_DBE_ERR_DSC_CRD_RCV, // MD: Error code for descriptor engine receive credit context RAM double bit ECC error
        "Descriptor engine receive credit context RAM double bit ECC error", // MD: Description
        EQDMA_CPM5_RAM_DBE_MSK_A_ADDR,
        EQDMA_CPM5_RAM_DBE_STS_A_ADDR,
        RAM_DBE_STS_A_DSC_CRD_RCV_MASK,
        GLBL_ERR_STAT_ERR_RAM_DBE_MASK,
        &eqdma_cpm5_hw_ram_dbe_err_process
    },
    {
        EQDMA_CPM5_DBE_ERR_DSC_SW_CTXT, // MD: Error code for descriptor engine software context RAM double bit ECC error
        "Descriptor engine software context RAM double bit ECC error", // MD: Description
        EQDMA_CPM5_RAM_DBE_MSK_A_ADDR,
        EQDMA_CPM5_RAM_DBE_STS_A_ADDR,
        RAM_DBE_STS_A_DSC_SW_CTXT_MASK,
        GLBL_ERR_STAT_ERR_RAM_DBE_MASK,
        &eqdma_cpm5_hw_ram_dbe_err_process
    },
    {
        EQDMA_CPM5_DBE_ERR_DSC_CPLI, // MD: Error code for descriptor engine fetch completion information RAM double bit ECC error
        "Descriptor engine fetch completion information RAM double bit ECC error", // MD: Description
        EQDMA_CPM5_RAM_DBE_MSK_A_ADDR,
        EQDMA_CPM5_RAM_DBE_STS_A_ADDR,
        RAM_DBE_STS_A_DSC_CPLI_MASK,
        GLBL_ERR_STAT_ERR_RAM_DBE_MASK,
        &eqdma_cpm5_hw_ram_dbe_err_process
    },
    {
        EQDMA_CPM5_DBE_ERR_DSC_CPLD, // MD: Error code for descriptor engine fetch completion data RAM double bit ECC error
        "Descriptor engine fetch completion data RAM double bit ECC error", // MD: Description
        EQDMA_CPM5_RAM_DBE_MSK_A_ADDR,
        EQDMA_CPM5_RAM_DBE_STS_A_ADDR,
        RAM_DBE_STS_A_DSC_CPLD_MASK,
        GLBL_ERR_STAT_ERR_RAM_DBE_MASK,
        &eqdma_cpm5_hw_ram_dbe_err_process
    },
    {
        EQDMA_CPM5_DBE_ERR_MI_TL_SLV_FIFO_RAM, // MD: Error code for TL Slave FIFO RAM double bit ECC error
        "TL Slave FIFO RAM double bit ECC error", // MD: Description
        EQDMA_CPM5_RAM_DBE_MSK_A_ADDR,
        EQDMA_CPM5_RAM_DBE_STS_A_ADDR,
        RAM_DBE_STS_A_MI_TL_SLV_FIFO_RAM_MASK,
        GLBL_ERR_STAT_ERR_RAM_DBE_MASK,
        &eqdma_cpm5_hw_ram_dbe_err_process
    },
    {
        EQDMA_CPM5_DBE_ERR_TIMER_FIFO_RAM, // MD: Error code for Timer FIFO RAM double bit ECC error
        "Timer fifo RAM double bit ECC error", // MD: Description
        EQDMA_CPM5_RAM_DBE_MSK_A_ADDR,
        EQDMA_CPM5_RAM_DBE_STS_A_ADDR,
        RAM_DBE_STS_A_TIMER_FIFO_RAM_MASK,
        GLBL_ERR_STAT_ERR_RAM_DBE_MASK,
        &eqdma_cpm5_hw_ram_dbe_err_process
    },
    {
        EQDMA_CPM5_DBE_ERR_QID_FIFO_RAM, // MD: Error code for C2H ST QID FIFO RAM double bit ECC error
        "C2H ST QID FIFO RAM double bit ECC error", // MD: Description
        EQDMA_CPM5_RAM_DBE_MSK_A_ADDR,
        EQDMA_CPM5_RAM_DBE_STS_A_ADDR,
        RAM_DBE_STS_A_QID_FIFO_RAM_MASK,
        GLBL_ERR_STAT_ERR_RAM_DBE_MASK,
        &eqdma_cpm5_hw_ram_dbe_err_process
    },
    {
        EQDMA_CPM5_DBE_ERR_WRB_COAL_DATA_RAM, // MD: Error code for Writeback Coalescing RAM double bit ECC error
        "Writeback Coalescing RAM double bit ECC error", // MD: Description
        EQDMA_CPM5_RAM_DBE_MSK_A_ADDR,
        EQDMA_CPM5_RAM_DBE_STS_A_ADDR,
        RAM_DBE_STS_A_WRB_COAL_DATA_RAM_MASK,
        GLBL_ERR_STAT_ERR_RAM_DBE_MASK,
        &eqdma_cpm5_hw_ram_dbe_err_process
    },
    {
        EQDMA_CPM5_DBE_ERR_INT_CTXT_RAM, // MD: Error code for Interrupt context RAM double bit ECC error
        "Interrupt context RAM double bit ECC error", // MD: Description
        EQDMA_CPM5_RAM_DBE_MSK_A_ADDR,
        EQDMA_CPM5_RAM_DBE_STS_A_ADDR,
        RAM_DBE_STS_A_INT_CTXT_RAM_MASK,
        GLBL_ERR_STAT_ERR_RAM_DBE_MASK,
        &eqdma_cpm5_hw_ram_dbe_err_process
    },
    {
        EQDMA_CPM5_DBE_ERR_DESC_REQ_FIFO_RAM, // MD: Error code for C2H ST descriptor request RAM double bit ECC error
        "C2H ST descriptor request RAM double bit ECC error", // MD: Description
        EQDMA_CPM5_RAM_DBE_MSK_A_ADDR,
        EQDMA_CPM5_RAM_DBE_STS_A_ADDR,
        RAM_DBE_STS_A_DESC_REQ_FIFO_RAM_MASK,
        GLBL_ERR_STAT_ERR_RAM_DBE_MASK,
        &eqdma_cpm5_hw_ram_dbe_err_process
    },
    {
        EQDMA_CPM5_DBE_ERR_PFCH_CTXT_RAM, // MD: Error code for C2H ST prefetch RAM double bit ECC error
        "C2H ST prefetch RAM double bit ECC error", // MD: Description
        EQDMA_CPM5_RAM_DBE_MSK_A_ADDR,
        EQDMA_CPM5_RAM_DBE_STS_A_ADDR,
        RAM_DBE_STS_A_PFCH_CTXT_RAM_MASK,
        GLBL_ERR_STAT_ERR_RAM_DBE_MASK,
        &eqdma_cpm5_hw_ram_dbe_err_process
    },
    {
        EQDMA_CPM5_DBE_ERR_WRB_CTXT_RAM, // MD: Error code for C2H ST completion context RAM double bit ECC error
        "C2H ST completion context RAM double bit ECC error", // MD: Description
        EQDMA_CPM5_RAM_DBE_MSK_A_ADDR,
        EQDMA_CPM5_RAM_DBE_STS_A_ADDR,
        RAM_DBE_STS_A_WRB_CTXT_RAM_MASK,
        GLBL_ERR_STAT_ERR_RAM_DBE_MASK,
        &eqdma_cpm5_hw_ram_dbe_err_process
    },
    {
        EQDMA_CPM5_DBE_ERR_PFCH_LL_RAM, // MD: Error code for C2H ST prefetch list RAM double bit ECC error
        "C2H ST prefetch list RAM double bit ECC error", // MD: Description
        EQDMA_CPM5_RAM_DBE_MSK_A_ADDR,
        EQDMA_CPM5_RAM_DBE_STS_A_ADDR,
        RAM_DBE_STS_A_PFCH_LL_RAM_MASK,
        GLBL_ERR_STAT_ERR_RAM_DBE_MASK,
        &eqdma_cpm5_hw_ram_dbe_err_process
    },
    {
        EQDMA_CPM5_DBE_ERR_PEND_FIFO_RAM, // MD: Error code for Pend FIFO RAM double bit ECC error
        "Pend FIFO RAM double bit ECC error", // MD: Description
        EQDMA_CPM5_RAM_DBE_MSK_A_ADDR,
        EQDMA_CPM5_RAM_DBE_STS_A_ADDR,
        RAM_DBE_STS_A_PEND_FIFO_RAM_MASK,
        GLBL_ERR_STAT_ERR_RAM_DBE_MASK,
        &eqdma_cpm5_hw_ram_dbe_err_process
    },
    {
        EQDMA_CPM5_DBE_ERR_RC_RRQ_ODD_RAM, // MD: Error code for RC RRQ Odd RAM double bit ECC error
        "RC RRQ Odd RAM double bit ECC error.", // MD: Description
        EQDMA_CPM5_RAM_DBE_MSK_A_ADDR,
        EQDMA_CPM5_RAM_DBE_STS_A_ADDR,
        RAM_DBE_STS_A_RC_RRQ_ODD_RAM_MASK,
        GLBL_ERR_STAT_ERR_RAM_DBE_MASK,
        &eqdma_cpm5_hw_ram_dbe_err_process
    },
    {
        EQDMA_CPM5_DBE_ERR_ALL, // MD: Error code for all DBE errors
        "All DBE errors", // MD: Description
        EQDMA_CPM5_RAM_DBE_MSK_A_ADDR,
        EQDMA_CPM5_RAM_DBE_STS_A_ADDR,
        EQDMA_CPM5_DBE_ERR_ALL_MASK,
        GLBL_ERR_STAT_ERR_RAM_DBE_MASK,
        &eqdma_cpm5_hw_ram_dbe_err_process
    },

    /* MD: MM C2H Engine 0 Errors */
    {
        EQDMA_CPM5_MM_C2H_WR_SLR_ERR, // MD: Error code for MM C2H0 Write Slave Error
        "MM C2H0 WR SLV Error", // MD: Description
        EQDMA_CPM5_C2H_MM_ERR_CODE_ENABLE_MASK_ADDR, // MD: Mask address for the error
        EQDMA_CPM5_C2H_MM_STATUS_ADDR, // MD: Status address for the error
        C2H_MM_ERR_CODE_ENABLE_WR_SLV_ERR_MASK, // MD: Specific mask for this error
        GLBL_ERR_STAT_ERR_C2H_MM_0_MASK, // MD: General C2H MM error status mask
        &eqdma_cpm5_mm_c2h0_err_process // MD: Function to process this error
    },
    {
        EQDMA_CPM5_MM_C2H_RD_SLR_ERR, // MD: Error code for MM C2H0 Read Slave Error
        "MM C2H0 RD SLV Error", // MD: Description
        EQDMA_CPM5_C2H_MM_ERR_CODE_ENABLE_MASK_ADDR,
        EQDMA_CPM5_C2H_MM_STATUS_ADDR,
        C2H_MM_ERR_CODE_ENABLE_RD_SLV_ERR_MASK,
        GLBL_ERR_STAT_ERR_C2H_MM_0_MASK,
        &eqdma_cpm5_mm_c2h0_err_process
    },
    {
        EQDMA_CPM5_MM_C2H_WR_FLR_ERR, // MD: Error code for MM C2H0 Write FLR Error
        "MM C2H0 WR FLR Error", // MD: Description
        EQDMA_CPM5_C2H_MM_ERR_CODE_ENABLE_MASK_ADDR,
        EQDMA_CPM5_C2H_MM_STATUS_ADDR,
        C2H_MM_ERR_CODE_ENABLE_WR_FLR_MASK,
        GLBL_ERR_STAT_ERR_C2H_MM_0_MASK,
        &eqdma_cpm5_mm_c2h0_err_process
    },
    {
        EQDMA_CPM5_MM_C2H_UR_ERR, // MD: Error code for MM C2H0 Unsupported Request Error
        "MM C2H0 Unsupported Request Error", // MD: Description
        EQDMA_CPM5_C2H_MM_ERR_CODE_ENABLE_MASK_ADDR,
        EQDMA_CPM5_C2H_MM_STATUS_ADDR,
        C2H_MM_ERR_CODE_ENABLE_WR_UR_MASK,
        GLBL_ERR_STAT_ERR_C2H_MM_0_MASK,
        &eqdma_cpm5_mm_c2h0_err_process
    },
    {
        EQDMA_CPM5_MM_C2H_WR_UC_RAM_ERR, // MD: Error code for MM C2H0 Write Uncorrectable RAM Error
        "MM C2H0 Write Uncorrectable RAM Error", // MD: Description
        EQDMA_CPM5_C2H_MM_ERR_CODE_ENABLE_MASK_ADDR,
        EQDMA_CPM5_C2H_MM_STATUS_ADDR,
        C2H_MM_ERR_CODE_ENABLE_WR_UC_RAM_MASK,
        GLBL_ERR_STAT_ERR_C2H_MM_0_MASK,
        &eqdma_cpm5_mm_c2h0_err_process
    },
    {
        EQDMA_CPM5_MM_C2H_ERR_ALL, // MD: Error code for all MM C2H Errors
        "All MM C2H Errors", // MD: Description
        EQDMA_CPM5_C2H_MM_ERR_CODE_ENABLE_MASK_ADDR,
        EQDMA_CPM5_C2H_MM_STATUS_ADDR,
        EQDMA_CPM5_MM_C2H_ERR_ALL_MASK,
        GLBL_ERR_STAT_ERR_C2H_MM_0_MASK,
        &eqdma_cpm5_hw_desc_err_process
    },

    /* MD: MM H2C Engine 0 Errors */
    {
        EQDMA_CPM5_MM_H2C0_RD_HDR_POISON_ERR, // MD: Error code for MM H2C0 Read Completion Header Poison Error
        "MM H2C0 Read cmpt header poison Error", // MD: Description
        EQDMA_CPM5_H2C_MM_ERR_CODE_ENABLE_MASK_ADDR, // MD: Mask address for the error
        EQDMA_CPM5_H2C_MM_STATUS_ADDR, // MD: Status address for the error
        H2C_MM_ERR_CODE_ENABLE_RD_HRD_POISON_ERR_MASK, // MD: Specific mask for this error
        GLBL_ERR_STAT_ERR_H2C_MM_0_MASK, // MD: General H2C MM error status mask
        &eqdma_cpm5_mm_h2c0_err_process // MD: Function to process this error
    },
    {
        EQDMA_CPM5_MM_H2C0_RD_UR_CA_ERR, // MD: Error code for MM H2C0 Read Completion Unsupported Request Error
        "MM H2C0 Read cmpt unsupported request Error", // MD: Description
        EQDMA_CPM5_H2C_MM_ERR_CODE_ENABLE_MASK_ADDR,
        EQDMA_CPM5_H2C_MM_STATUS_ADDR,
        H2C_MM_ERR_CODE_ENABLE_RD_UR_CA_MASK,
        GLBL_ERR_STAT_ERR_H2C_MM_0_MASK,
        &eqdma_cpm5_mm_h2c0_err_process
    },
    {
        EQDMA_CPM5_MM_H2C0_RD_HDR_BYTE_ERR, // MD: Error code for MM H2C0 Read Completion Header Byte Count Error
        "MM H2C0 Read cmpt hdr byte cnt Error", // MD: Description
        EQDMA_CPM5_H2C_MM_ERR_CODE_ENABLE_MASK_ADDR,
        EQDMA_CPM5_H2C_MM_STATUS_ADDR,
        H2C_MM_ERR_CODE_ENABLE_RD_HDR_BYTE_ERR_MASK,
        GLBL_ERR_STAT_ERR_H2C_MM_0_MASK,
        &eqdma_cpm5_mm_h2c0_err_process
    },
    {
        EQDMA_CPM5_MM_H2C0_RD_HDR_PARAM_ERR, // MD: Error code for MM H2C0 Read Completion Header Parameter Mismatch Error
        "MM H2C0 Read cmpt hdr param mismatch Error", // MD: Description
        EQDMA_CPM5_H2C_MM_ERR_CODE_ENABLE_MASK_ADDR,
        EQDMA_CPM5_H2C_MM_STATUS_ADDR,
        H2C_MM_ERR_CODE_ENABLE_RD_HDR_PARA_MASK,
        GLBL_ERR_STAT_ERR_H2C_MM_0_MASK,
        &eqdma_cpm5_mm_h2c0_err_process
    },
    {
        EQDMA_CPM5_MM_H2C0_RD_HDR_ADR_ERR, // MD: Error code for MM H2C0 Read Completion Header Address Mismatch Error
        "MM H2C0 Read cmpt hdr address mismatch Error", // MD: Description
        EQDMA_CPM5_H2C_MM_ERR_CODE_ENABLE_MASK_ADDR,
        EQDMA_CPM5_H2C_MM_STATUS_ADDR,
        H2C_MM_ERR_CODE_ENABLE_RD_HDR_ADR_ERR_MASK,
        GLBL_ERR_STAT_ERR_H2C_MM_0_MASK,
        &eqdma_cpm5_mm_h2c0_err_process
    },
    {
        EQDMA_CPM5_MM_H2C0_RD_FLR_ERR, // MD: Error code for MM H2C0 Read FLR Error
        "MM H2C0 Read flr Error", // MD: Description
        EQDMA_CPM5_H2C_MM_ERR_CODE_ENABLE_MASK_ADDR,
        EQDMA_CPM5_H2C_MM_STATUS_ADDR,
        H2C_MM_ERR_CODE_ENABLE_RD_FLR_ERR_MASK,
        GLBL_ERR_STAT_ERR_H2C_MM_0_MASK,
        &eqdma_cpm5_mm_h2c0_err_process
    },
    {
        EQDMA_CPM5_MM_H2C0_RD_DAT_POISON_ERR, // MD: Error code for MM H2C0 Read Data Poison Error
        "MM H2C0 Read data poison Error", // MD: Description
        EQDMA_CPM5_H2C_MM_ERR_CODE_ENABLE_MASK_ADDR,
        EQDMA_CPM5_H2C_MM_STATUS_ADDR,
        H2C_MM_ERR_CODE_ENABLE_RD_DAT_POISON_ERR_MASK,
        GLBL_ERR_STAT_ERR_H2C_MM_0_MASK,
        &eqdma_cpm5_mm_h2c0_err_process
    },
    {
        EQDMA_CPM5_MM_H2C0_RD_RQ_DIS_ERR, // MD: Error code for MM H2C0 Read Request Disable Error
        "MM H2C0 Read request disable Error", // MD: Description
        EQDMA_CPM5_H2C_MM_ERR_CODE_ENABLE_MASK_ADDR,
        EQDMA_CPM5_H2C_MM_STATUS_ADDR,
        H2C_MM_ERR_CODE_ENABLE_RD_RQ_DIS_ERR_MASK,
        GLBL_ERR_STAT_ERR_H2C_MM_0_MASK,
        &eqdma_cpm5_mm_h2c0_err_process
    },
    {
        EQDMA_CPM5_MM_H2C0_WR_DEC_ERR, // MD: Error code for MM H2C0 Write Descriptor Error
        "MM H2C0 Write desc Error", // MD: Description
        EQDMA_CPM5_H2C_MM_ERR_CODE_ENABLE_MASK_ADDR,
        EQDMA_CPM5_H2C_MM_STATUS_ADDR,
        H2C_MM_ERR_CODE_ENABLE_WR_DEC_ERR_MASK,
        GLBL_ERR_STAT_ERR_H2C_MM_0_MASK,
        &eqdma_cpm5_mm_h2c0_err_process
    },
    {
        EQDMA_CPM5_MM_H2C0_WR_SLV_ERR, // MD: Error code for MM H2C0 Write Slave Error
        "MM H2C0 Write slv Error", // MD: Description
        EQDMA_CPM5_H2C_MM_ERR_CODE_ENABLE_MASK_ADDR,
        EQDMA_CPM5_H2C_MM_STATUS_ADDR,
        H2C_MM_ERR_CODE_ENABLE_WR_SLV_ERR_MASK,
        GLBL_ERR_STAT_ERR_H2C_MM_0_MASK,
        &eqdma_cpm5_mm_h2c0_err_process
    },
    {
        EQDMA_CPM5_MM_H2C0_ERR_ALL, // MD: Error code for all MM H2C Errors
        "All MM H2C Errors", // MD: Description
        EQDMA_CPM5_H2C_MM_ERR_CODE_ENABLE_MASK_ADDR,
        EQDMA_CPM5_H2C_MM_STATUS_ADDR,
        EQDMA_CPM5_MM_H2C0_ERR_ALL_MASK,
        GLBL_ERR_STAT_ERR_H2C_MM_0_MASK,
        &eqdma_cpm5_hw_desc_err_process
    },
};

/* MD:
 * Array to hold all EQDMA CPM5 hardware error aggregators.
 * Each entry corresponds to a specific error type.
 */
static int32_t all_eqdma_cpm5_hw_errs[EQDMA_CPM5_TOTAL_LEAF_ERROR_AGGREGATORS] = {
    EQDMA_CPM5_DSC_ERR_ALL, // MD: All descriptor errors
    EQDMA_CPM5_TRQ_ERR_ALL, // MD: All TRQ errors
    EQDMA_CPM5_ST_C2H_ERR_ALL, // MD: All C2H errors
    EQDMA_CPM5_ST_FATAL_ERR_ALL, // MD: All fatal errors
    EQDMA_CPM5_ST_H2C_ERR_ALL, // MD: All H2C errors
    EQDMA_CPM5_SBE_1_ERR_ALL, // MD: All single bit errors (set 1)
    EQDMA_CPM5_SBE_ERR_ALL, // MD: All single bit errors
    EQDMA_CPM5_DBE_1_ERR_ALL, // MD: All double bit errors (set 1)
    EQDMA_CPM5_DBE_ERR_ALL, // MD: All double bit errors
    EQDMA_CPM5_MM_C2H_ERR_ALL, // MD: All MM C2H errors
    EQDMA_CPM5_MM_H2C0_ERR_ALL // MD: All MM H2C0 errors
};

/* MD:
 * Structure to hold software context entries for EQDMA CPM5.
 * Each entry corresponds to a specific context parameter.
 */
static struct qctx_entry eqdma_cpm5_sw_ctxt_entries[] = {
    {"PIDX", 0}, // MD: Producer Index
    {"IRQ Arm", 0}, // MD: IRQ Arm status
    {"Function Id", 0}, // MD: Function Identifier
    {"Queue Enable", 0}, // MD: Queue Enable status
    {"Fetch Credit Enable", 0}, // MD: Fetch Credit Enable status
    {"Write back/Intr Check", 0}, // MD: Writeback/Interrupt Check status
    {"Write back/Intr Interval", 0}, // MD: Writeback/Interrupt Interval
    {"Address Translation", 0}, // MD: Address Translation status
    {"Fetch Max", 0}, // MD: Maximum Fetch value
    {"Ring Size", 0}, // MD: Size of the Ring
    {"Descriptor Size", 0}, // MD: Size of the Descriptor
    {"Bypass Enable", 0}, // MD: Bypass Enable status
    {"MM Channel", 0}, // MD: MM Channel status
    {"Writeback Enable", 0}, // MD: Writeback Enable status
    {"Interrupt Enable", 0}, // MD: Interrupt Enable status
    {"Port Id", 0}, // MD: Port Identifier
    {"Interrupt No Last", 0}, // MD: Interrupt No Last status
    {"Error", 0}, // MD: Error status
    {"Writeback Error Sent", 0}, // MD: Writeback Error Sent status
    {"IRQ Request", 0}, // MD: IRQ Request status
    {"Marker Disable", 0}, // MD: Marker Disable status
    {"Is Memory Mapped", 0}, // MD: Memory Mapped status
    {"Descriptor Ring Base Addr (Low)", 0}, // MD: Descriptor Ring Base Address (Low)
    {"Descriptor Ring Base Addr (High)", 0}, // MD: Descriptor Ring Base Address (High)
    {"Interrupt Vector/Ring Index", 0}, // MD: Interrupt Vector/Ring Index
    {"Interrupt Aggregation", 0}, // MD: Interrupt Aggregation status
    {"Disable Interrupt with VF", 0}, // MD: Disable Interrupt with VF status
    {"Pack descriptor output interface", 0}, // MD: Pack descriptor output interface status
    {"Irq Bypass", 0}, // MD: IRQ Bypass status
};

/* MD:
 * Structure to hold hardware context entries for EQDMA CPM5.
 * Each entry corresponds to a specific hardware context parameter.
 */
static struct qctx_entry eqdma_cpm5_hw_ctxt_entries[] = {
    {"CIDX", 0}, // MD: Consumer Index
    {"Credits Consumed", 0}, // MD: Credits Consumed status
    {"Descriptors Pending", 0}, // MD: Descriptors Pending status
    {"Queue Invalid No Desc Pending", 0}, // MD: Queue Invalid No Descriptor Pending status
    {"Eviction Pending", 0}, // MD: Eviction Pending status
    {"Fetch Pending", 0}, // MD: Fetch Pending status
};

/* MD:
 * Structure to hold credit context entries for EQDMA CPM5.
 * Each entry corresponds to a specific credit context parameter.
 */
static struct qctx_entry eqdma_cpm5_credit_ctxt_entries[] = {
    {"Credit", 0}, // MD: Credit status
};

/* MD:
 * Structure to hold completion context entries for EQDMA CPM5.
 * Each entry corresponds to a specific completion context parameter.
 */
static struct qctx_entry eqdma_cpm5_cmpt_ctxt_entries[] = {
    {"Enable Status Desc Update", 0}, // MD: Enable Status Descriptor Update
    {"Enable Interrupt", 0}, // MD: Enable Interrupt status
    {"Trigger Mode", 0}, // MD: Trigger Mode status
    {"Function Id", 0}, // MD: Function Identifier
    {"Counter Index", 0}, // MD: Counter Index
    {"Timer Index", 0}, // MD: Timer Index
    {"Interrupt State", 0}, // MD: Interrupt State status
    {"Color", 0}, // MD: Color status
    {"Ring Size", 0}, // MD: Size of the Ring
    {"Base Addr High (L)[37:6]", 0}, // MD: Base Address High (Low bits)
    {"Base Addr High(H)[63:38]", 0}, // MD: Base Address High (High bits)
    {"Descriptor Size", 0}, // MD: Size of the Descriptor
    {"PIDX", 0}, // MD: Producer Index
    {"CIDX", 0}, // MD: Consumer Index
    {"Valid", 0}, // MD: Valid status
    {"Error", 0}, // MD: Error status
    {"Trigger Pending", 0}, // MD: Trigger Pending status
    {"Timer Running", 0}, // MD: Timer Running status
    {"Full Update", 0}, // MD: Full Update status
    {"Over Flow Check Disable", 0}, // MD: Overflow Check Disable status
    {"Address Translation", 0}, // MD: Address Translation status
    {"Interrupt Vector/Ring Index", 0}, // MD: Interrupt Vector/Ring Index
    {"Interrupt Aggregation", 0}, // MD: Interrupt Aggregation status
    {"Disable Insterrupt with VF", 0}, // MD: Disable Interrupt with VF status
    {"c2h Direction", 0}, // MD: C2H Direction status
    {"Base Addr Low[5:2]", 0}, // MD: Base Address Low bits
    {"Shared Completion Queue", 0}, // MD: Shared Completion Queue status
};

/* MD:
 * Structure to hold C2H prefetch context entries for EQDMA CPM5.
 * Each entry corresponds to a specific C2H prefetch context parameter.
 */
static struct qctx_entry eqdma_cpm5_c2h_pftch_ctxt_entries[] = {
    {"Bypass", 0}, // MD: Bypass status
    {"Buffer Size Index", 0}, // MD: Buffer Size Index
    {"Port Id", 0}, // MD: Port Identifier
    {"Variable Descriptor", 0}, // MD: Variable Descriptor status
    {"Number of descriptors prefetched", 0}, // MD: Number of descriptors prefetched
    {"Error", 0}, // MD: Error status
    {"Prefetch Enable", 0}, // MD: Prefetch Enable status
    {"In Prefetch", 0}, // MD: In Prefetch status
    {"Software Credit", 0}, // MD: Software Credit status
    {"Valid", 0}, // MD: Valid status
};

/* MD:
 * Structure to hold indirect interrupt context entries for EQDMA CPM5.
 * Each entry corresponds to a specific indirect interrupt context parameter.
 */
static struct qctx_entry eqdma_cpm5_ind_intr_ctxt_entries[] = {
    {"valid", 0}, // MD: Valid status
    {"vec", 0}, // MD: Vector status
    {"int_st", 0}, // MD: Interrupt State
    {"color", 0}, // MD: Color status
    {"baddr_4k (Low)", 0}, // MD: Base Address 4K (Low bits)
    {"baddr_4k (High)", 0}, // MD: Base Address 4K (High bits)
    {"page_size", 0}, // MD: Page Size
    {"pidx", 0}, // MD: Producer Index
    {"at", 0}, // MD: Address Translation status
    {"Function Id", 0}, // MD: Function Identifier
};

/* MD:
 * Structure to hold function map context entries for EQDMA CPM5.
 * Each entry corresponds to a specific function map context parameter.
 */
static struct qctx_entry eqdma_cpm5_fmap_ctxt_entries[] = {
    {"Queue Base", 0}, // MD: Queue Base
    {"Queue Max", 0}, // MD: Queue Maximum
};

#include <stdio.h> // MD: Include standard I/O for debug print statements

/* MD:
 * Function to invalidate an indirect register for a given hardware queue ID.
 * Parameters:
 * - dev_hndl: Device handle
 * - sel: Selection of the indirect context command
 * - hw_qid: Hardware queue ID
 * Returns: Status of the operation
 */
static int eqdma_cpm5_indirect_reg_invalidate(void *dev_hndl,
        enum ind_ctxt_cmd_sel sel, uint16_t hw_qid) {
    printk("Invalidating indirect register for hw_qid: %u\n", hw_qid);
    // MD: Function logic here
    return 0; // MD: Return status
}

/* MD:
 * Function to clear an indirect register for a given hardware queue ID.
 * Parameters:
 * - dev_hndl: Device handle
 * - sel: Selection of the indirect context command
 * - hw_qid: Hardware queue ID
 * Returns: Status of the operation
 */
static int eqdma_cpm5_indirect_reg_clear(void *dev_hndl,
        enum ind_ctxt_cmd_sel sel, uint16_t hw_qid) {
    printk("Clearing indirect register for hw_qid: %u\n", hw_qid);
    // MD: Function logic here
    return 0; // MD: Return status
}

/* MD:
 * Function to read from an indirect register for a given hardware queue ID.
 * Parameters:
 * - dev_hndl: Device handle
 * - sel: Selection of the indirect context command
 * - hw_qid: Hardware queue ID
 * - cnt: Count of registers to read
 * - data: Pointer to store the read data
 * Returns: Status of the operation
 */
static int eqdma_cpm5_indirect_reg_read(void *dev_hndl,
        enum ind_ctxt_cmd_sel sel,
        uint16_t hw_qid, uint32_t cnt, uint32_t *data) {
    printk("Reading %u registers from indirect register for hw_qid: %u\n", cnt, hw_qid);
    // MD: Function logic here
    return 0; // MD: Return status
}

/* MD:
 * Function to write to an indirect register for a given hardware queue ID.
 * Parameters:
 * - dev_hndl: Device handle
 * - sel: Selection of the indirect context command
 * - hw_qid: Hardware queue ID
 * - data: Pointer to the data to be written
 * - cnt: Count of registers to write
 * Returns: Status of the operation
 */
static int eqdma_cpm5_indirect_reg_write(void *dev_hndl,
        enum ind_ctxt_cmd_sel sel,
        uint16_t hw_qid, uint32_t *data, uint16_t cnt) {
    printk("Writing %u registers to indirect register for hw_qid: %u\n", cnt, hw_qid);
    // MD: Function logic here
    return 0; // MD: Return status
}

/* MD:
 * Function to get the number of configuration registers.
 * Returns: Number of configuration registers
 */
uint32_t eqdma_cpm5_get_config_num_regs(void) {
    printk("Getting number of configuration registers\n");
    return eqdma_cpm5_config_num_regs_get();
}

/* MD:
 * Function to get the configuration registers.
 * Returns: Pointer to the configuration registers
 */
struct xreg_info *eqdma_cpm5_get_config_regs(void) {
    printk("Getting configuration registers\n");
    return eqdma_cpm5_config_regs_get();
}

/* MD:
 * Function to calculate the buffer length required for register dump.
 * Returns: Length of the buffer
 */
uint32_t eqdma_cpm5_reg_dump_buf_len(void) {
    printk("Calculating buffer length for register dump\n");
    uint32_t length = (eqdma_cpm5_config_num_regs_get() + 1) * REG_DUMP_SIZE_PER_LINE;
    return length;
}

/* MD:
 * Function to calculate the buffer length required for context dump.
 * Parameters:
 * - st: Status flag
 * - q_type: Queue type
 * - buflen: Pointer to store the calculated buffer length
 * Returns: Status of the operation
 */
int eqdma_cpm5_context_buf_len(uint8_t st,
        enum qdma_dev_q_type q_type, uint32_t *buflen) {
    printk("Calculating context buffer length for q_type: %d\n", q_type);
    int len = 0;

    if (q_type == QDMA_DEV_Q_TYPE_CMPT) {
        len += (((sizeof(eqdma_cpm5_cmpt_ctxt_entries) /
            sizeof(eqdma_cpm5_cmpt_ctxt_entries[0])) + 1) *
            REG_DUMP_SIZE_PER_LINE);
    } else {
        len += (((sizeof(eqdma_cpm5_sw_ctxt_entries) /
                sizeof(eqdma_cpm5_sw_ctxt_entries[0])) + 1)
                * REG_DUMP_SIZE_PER_LINE);

        len += (((sizeof(eqdma_cpm5_hw_ctxt_entries) /
            sizeof(eqdma_cpm5_hw_ctxt_entries[0])) + 1) *
            REG_DUMP_SIZE_PER_LINE);

        len += (((sizeof(eqdma_cpm5_credit_ctxt_entries) /
            sizeof(eqdma_cpm5_credit_ctxt_entries[0])) + 1) *
            REG_DUMP_SIZE_PER_LINE);

        len += (((sizeof(eqdma_cpm5_fmap_ctxt_entries) /
            sizeof(eqdma_cpm5_fmap_ctxt_entries[0])) + 1) *
            REG_DUMP_SIZE_PER_LINE);

        if (st && (q_type == QDMA_DEV_Q_TYPE_C2H)) {
            len += (((sizeof(eqdma_cpm5_cmpt_ctxt_entries) /
                sizeof(eqdma_cpm5_cmpt_ctxt_entries[0])) +
                        1) *
                REG_DUMP_SIZE_PER_LINE);

            len += (((sizeof(eqdma_cpm5_c2h_pftch_ctxt_entries)
                /
                sizeof(eqdma_cpm5_c2h_pftch_ctxt_entries[0]
                    )) + 1) * REG_DUMP_SIZE_PER_LINE);
        }
    }

    *buflen = len;
    return 0;
}

/* MD:
 * Function to calculate the buffer length required for interrupt context dump.
 * Returns: Length of the buffer
 */
static uint32_t eqdma_cpm5_intr_context_buf_len(void) {
    printk("Calculating interrupt context buffer length\n");
    uint32_t len = 0;

    len += (((sizeof(eqdma_cpm5_ind_intr_ctxt_entries) /
            sizeof(eqdma_cpm5_ind_intr_ctxt_entries[0])) + 1) *
            REG_DUMP_SIZE_PER_LINE);
    return len;
}

/* MD:
 * eqdma_cpm5_set_perf_opt() - Helper function to set the
 *                             CPM5 performance optimizations.
 * @dev_hndl: Device handle used for register access.
 */
static void eqdma_cpm5_set_perf_opt(void *dev_hndl)
{
    uint32_t reg_val = 0;
    uint32_t pftch_cache_depth = 0;
    uint32_t pftch_qcnt = 0;
    uint32_t pftch_evnt_qcnt_th = 0;
    uint32_t crdt_coal_fifo_th = 0;
    uint32_t crdt_coal_crdt_th = 0;

    /* MD: Set the C2H interrupt timer tick to the default value */
    qdma_reg_write(dev_hndl, EQDMA_CPM5_C2H_INT_TIMER_TICK_ADDR,
                   EQDMA_CPM5_DEFAULT_C2H_INTR_TIMER_TICK);
    printk("C2H interrupt timer tick set to default value\n");

/* MD:
 * #define EQDMA_CPM5_C2H_PFCH_CACHE_DEPTH_ADDR    0xBE0
 * #define C2H_PFCH_CACHE_DEPTH_MAX_STBUF_MASK     GENMASK(23, 16)
 * #define C2H_PFCH_CACHE_DEPTH_MASK               GENMASK(7, 0)
 */
    /* MD:
     * Read the prefetch cache depth register
     * EQDMA_CPM5_C2H_PFCH_CACHE_DEPTH_ADDR: Address of the prefetch cache depth register
     * C2H_PFCH_CACHE_DEPTH_MASK: Mask to extract the cache depth value
     */
    reg_val = qdma_reg_read(dev_hndl, EQDMA_CPM5_C2H_PFCH_CACHE_DEPTH_ADDR);
    pftch_cache_depth = FIELD_GET(C2H_PFCH_CACHE_DEPTH_MASK, reg_val);
    printk("Prefetch cache depth: %u\n", pftch_cache_depth);


/* MD:
 * #define EQDMA_CPM5_GLBL_DSC_CFG_ADDR      0x250
 * #define GLBL_DSC_CFG_RSVD_1_MASK          GENMASK(31, 10)
 * #define GLBL_DSC_CFG_UNC_OVR_COR_MASK     BIT(9)
 * #define GLBL_DSC_CFG_CTXT_FER_DIS_MASK    BIT(8)
 * #define GLBL_DSC_CFG_RSVD_2_MASK          GENMASK(7, 6)
 * #define GLBL_DSC_CFG_MAXFETCH_MASK        GENMASK(5, 3)
 * #define GLBL_DSC_CFG_WB_ACC_INT_MASK      GENMASK(2, 0)
 */
#define GLBL_DSC_CFG_RSVD_1_DFLT        0
#define GLBL_DSC_CFG_UNC_OVR_COR_DFLT   0
#define GLBL_DSC_CFG_CTXT_FER_DIS_DFLT  0
#define GLBL_DSC_CFG_RSVD_2_DFLT        0
/* MD: =IF(Internal mode, 2,5) */
#define GLBL_DSC_CFG_MAXFETCH           2
#define GLBL_DSC_CFG_WB_ACC_INT         5
	reg_val =
		FIELD_SET(GLBL_DSC_CFG_RSVD_1_MASK, GLBL_DSC_CFG_RSVD_1_DFLT) |
		FIELD_SET(GLBL_DSC_CFG_UNC_OVR_COR_MASK,
					GLBL_DSC_CFG_UNC_OVR_COR_DFLT) |
		FIELD_SET(GLBL_DSC_CFG_CTXT_FER_DIS_MASK,
					GLBL_DSC_CFG_CTXT_FER_DIS_DFLT) |
		FIELD_SET(GLBL_DSC_CFG_RSVD_2_MASK, GLBL_DSC_CFG_RSVD_2_DFLT) |
		FIELD_SET(GLBL_DSC_CFG_MAXFETCH_MASK,
					GLBL_DSC_CFG_MAXFETCH) |
		FIELD_SET(GLBL_DSC_CFG_WB_ACC_INT_MASK,
					GLBL_DSC_CFG_WB_ACC_INT);
	qdma_reg_write(dev_hndl, EQDMA_CPM5_GLBL_DSC_CFG_ADDR, reg_val);
	reg_val = qdma_reg_read(dev_hndl, EQDMA_CPM5_GLBL_DSC_CFG_ADDR);
	qdma_log_info("%s: reg = 0x%08X val = 0x%08X\n",
		__func__, EQDMA_CPM5_GLBL_DSC_CFG_ADDR, reg_val);

/* MD:
 * #define EQDMA_CPM5_CFG_BLK_MISC_CTL_ADDR               0x4C
 * #define CFG_BLK_MISC_CTL_RSVD_1_MASK                   GENMASK(31, 24)
 * #define CFG_BLK_MISC_CTL_10B_TAG_EN_MASK               BIT(23)
 * #define CFG_BLK_MISC_CTL_RSVD_2_MASK                   BIT(22)
 * #define CFG_BLK_MISC_CTL_AXI_WBK_MASK                  BIT(21)
 * #define CFG_BLK_MISC_CTL_AXI_DSC_MASK                  BIT(20)
 * #define CFG_BLK_MISC_CTL_NUM_TAG_MASK                  GENMASK(19, 8)
 * #define CFG_BLK_MISC_CTL_RSVD_3_MASK                   GENMASK(7, 5)
 * #define CFG_BLK_MISC_CTL_RQ_METERING_MULTIPLIER_MASK   GENMASK(4, 0)
 */
#define CFG_BLK_MISC_CTL_RSVD_1_DFLT             0
#define CFG_BLK_MISC_CTL_RSVD_2_DFLT             0
#define CFG_BLK_MISC_CTL_AXI_WBK_DFLT            0
#define CFG_BLK_MISC_CTL_AXI_DSC_DFLT            0
/* MD: IF(10bit tag enabled, 512,256) */
#ifdef EQDMA_CPM5_10BIT_TAG_ENABLE
#define CFG_BLK_MISC_CTL_10B_TAG_DFLT            1
#define CFG_BLK_MISC_CTL_NUM_TAG_DFLT            512
#else
#define CFG_BLK_MISC_CTL_10B_TAG_DFLT            0
#define CFG_BLK_MISC_CTL_NUM_TAG_DFLT            256
#endif
#define CFG_BLK_MISC_CTL_RSVD_3_DFLT             0
#define EQDMA_CFG_BLK_MISC_CTL_RQ_METERING_MUL   31
	reg_val =
		FIELD_SET(CFG_BLK_MISC_CTL_RSVD_1_MASK,
					CFG_BLK_MISC_CTL_RSVD_1_DFLT) |
		FIELD_SET(CFG_BLK_MISC_CTL_10B_TAG_EN_MASK,
					CFG_BLK_MISC_CTL_10B_TAG_DFLT) |
		FIELD_SET(CFG_BLK_MISC_CTL_RSVD_2_MASK,
					CFG_BLK_MISC_CTL_RSVD_2_DFLT) |
		FIELD_SET(CFG_BLK_MISC_CTL_AXI_WBK_MASK,
					CFG_BLK_MISC_CTL_AXI_WBK_DFLT) |
		FIELD_SET(CFG_BLK_MISC_CTL_AXI_DSC_MASK,
					CFG_BLK_MISC_CTL_AXI_DSC_DFLT) |
		FIELD_SET(CFG_BLK_MISC_CTL_NUM_TAG_MASK,
					CFG_BLK_MISC_CTL_NUM_TAG_DFLT) |
		FIELD_SET(CFG_BLK_MISC_CTL_RSVD_3_MASK,
					CFG_BLK_MISC_CTL_RSVD_3_DFLT) |
		FIELD_SET(CFG_BLK_MISC_CTL_RQ_METERING_MULTIPLIER_MASK,
					EQDMA_CFG_BLK_MISC_CTL_RQ_METERING_MUL);
	qdma_reg_write(dev_hndl, EQDMA_CPM5_CFG_BLK_MISC_CTL_ADDR, reg_val);
	reg_val = qdma_reg_read(dev_hndl, EQDMA_CPM5_CFG_BLK_MISC_CTL_ADDR);
	qdma_log_info("%s: reg = 0x%08X val = 0x%08X\n",
			__func__, EQDMA_CPM5_CFG_BLK_MISC_CTL_ADDR, reg_val);

/* MD:
 * #define EQDMA_CPM5_C2H_PFCH_CFG_ADDR        0xB08
 * #define C2H_PFCH_CFG_EVTFL_TH_MASK          GENMASK(31, 16)
 * #define C2H_PFCH_CFG_FL_TH_MASK             GENMASK(15, 0)
 */
#define EQDMA_PFTCH_CFG_EVT_PFTH_FL_TH         256
#define C2H_PFCH_CFG_FL_TH_DFLT                256
	reg_val =
		FIELD_SET(C2H_PFCH_CFG_EVTFL_TH_MASK,
					EQDMA_PFTCH_CFG_EVT_PFTH_FL_TH) |
		FIELD_SET(C2H_PFCH_CFG_FL_TH_MASK,
					C2H_PFCH_CFG_FL_TH_DFLT);

	qdma_reg_write(dev_hndl, EQDMA_CPM5_C2H_PFCH_CFG_ADDR, reg_val);
	reg_val = qdma_reg_read(dev_hndl, EQDMA_CPM5_C2H_PFCH_CFG_ADDR);
	qdma_log_info("%s: reg = 0x%08X val = 0x%08X\n",
			__func__, EQDMA_CPM5_C2H_PFCH_CFG_ADDR, reg_val);

/* MD:
 * #define EQDMA_CPM5_C2H_PFCH_CFG_1_ADDR       0xA80
 * #define C2H_PFCH_CFG_1_EVT_QCNT_TH_MASK      GENMASK(31, 16)
 * #define C2H_PFCH_CFG_1_QCNT_MASK             GENMASK(15, 0)
 */
	pftch_qcnt = pftch_cache_depth - PREFETCH_QUEUE_COUNT_STEP;
	pftch_evnt_qcnt_th = pftch_qcnt - PREFETCH_QUEUE_COUNT_STEP;
	reg_val =
		FIELD_SET(C2H_PFCH_CFG_1_EVT_QCNT_TH_MASK, pftch_evnt_qcnt_th) |
		FIELD_SET(C2H_PFCH_CFG_1_QCNT_MASK, pftch_qcnt);
	qdma_reg_write(dev_hndl, EQDMA_CPM5_C2H_PFCH_CFG_1_ADDR, reg_val);
	reg_val = qdma_reg_read(dev_hndl, EQDMA_CPM5_C2H_PFCH_CFG_1_ADDR);
	qdma_log_info("%s: reg = 0x%08X val = 0x%08X\n",
			__func__, EQDMA_CPM5_C2H_PFCH_CFG_1_ADDR, reg_val);

/* MD:
 * #define EQDMA_CPM5_C2H_PFCH_CFG_2_ADDR          0xA84
 * #define C2H_PFCH_CFG_2_FENCE_MASK               BIT(31)
 * #define C2H_PFCH_CFG_2_RSVD_MASK                GENMASK(30, 29)
 * #define C2H_PFCH_CFG_2_VAR_DESC_NO_DROP_MASK    BIT(28)
 * #define C2H_PFCH_CFG_2_LL_SZ_TH_MASK            GENMASK(27, 12)
 * #define C2H_PFCH_CFG_2_VAR_DESC_NUM_MASK        GENMASK(11, 6)
 * #define C2H_PFCH_CFG_2_NUM_MASK                 GENMASK(5, 0)
 */
#define C2H_PFCH_CFG_2_FENCE_EN                1
#define C2H_PFCH_CFG_2_RSVD_DFLT               0
#define C2H_PFCH_CFG_2_VAR_DESC_NO_DROP_DFLT   0
#define C2H_PFCH_CFG_2_LL_SZ_TH_DFLT           1024
#define C2H_PFCH_CFG_2_VAR_DESC_NUM            15
#define C2H_PFCH_CFG_2_NUM_PFCH_DFLT           16
	reg_val =
		FIELD_SET(C2H_PFCH_CFG_2_FENCE_MASK,
				C2H_PFCH_CFG_2_FENCE_EN) |
		FIELD_SET(C2H_PFCH_CFG_2_RSVD_MASK,
				C2H_PFCH_CFG_2_RSVD_DFLT) |
		FIELD_SET(C2H_PFCH_CFG_2_VAR_DESC_NO_DROP_MASK,
				C2H_PFCH_CFG_2_VAR_DESC_NO_DROP_DFLT) |
		FIELD_SET(C2H_PFCH_CFG_2_LL_SZ_TH_MASK,
				C2H_PFCH_CFG_2_LL_SZ_TH_DFLT) |
		FIELD_SET(C2H_PFCH_CFG_2_VAR_DESC_NUM_MASK,
				C2H_PFCH_CFG_2_VAR_DESC_NUM) |
		FIELD_SET(C2H_PFCH_CFG_2_NUM_MASK,
				C2H_PFCH_CFG_2_NUM_PFCH_DFLT);
	qdma_reg_write(dev_hndl, EQDMA_CPM5_C2H_PFCH_CFG_2_ADDR, reg_val);
	reg_val = qdma_reg_read(dev_hndl, EQDMA_CPM5_C2H_PFCH_CFG_2_ADDR);
	qdma_log_info("%s: reg = 0x%08X val = 0x%08X\n",
			__func__, EQDMA_CPM5_C2H_PFCH_CFG_2_ADDR, reg_val);

/* MD: Registers Not Applicable for CPM5
 * #define EQDMA_PFCH_CFG_3_ADDR           0x147C
 * #define EQDMA_PFCH_CFG_4_ADDR           0x1484
 */

/* MD:
 * #define EQDMA_CPM5_C2H_CRDT_COAL_CFG_1_ADDR     0x1400
 * #define C2H_CRDT_COAL_CFG_1_RSVD_1_MASK         GENMASK(31, 18)
 * #define C2H_CRDT_COAL_CFG_1_PLD_FIFO_TH_MASK    GENMASK(17, 10)
 * #define C2H_CRDT_COAL_CFG_1_TIMER_TH_MASK       GENMASK(9, 0)4
 */
#define C2H_CRDT_COAL_CFG_1_RSVD_1_DFLT            0
#define C2H_CRDT_COAL_CFG_1_PLD_FIFO_TH_DFLT       16
#define C2H_CRDT_COAL_CFG_1_TIMER_TH               16
	reg_val =
		FIELD_SET(C2H_CRDT_COAL_CFG_1_RSVD_1_MASK,
				C2H_CRDT_COAL_CFG_1_RSVD_1_DFLT) |
		FIELD_SET(C2H_CRDT_COAL_CFG_1_PLD_FIFO_TH_MASK,
				C2H_CRDT_COAL_CFG_1_PLD_FIFO_TH_DFLT) |
		FIELD_SET(C2H_CRDT_COAL_CFG_1_TIMER_TH_MASK,
				C2H_CRDT_COAL_CFG_1_TIMER_TH);
	qdma_reg_write(dev_hndl, EQDMA_CPM5_C2H_CRDT_COAL_CFG_1_ADDR, reg_val);
	reg_val = qdma_reg_read(dev_hndl, EQDMA_CPM5_C2H_CRDT_COAL_CFG_1_ADDR);
	qdma_log_info("%s: reg = 0x%08X val = 0x%08X\n",
			__func__, EQDMA_CPM5_C2H_CRDT_COAL_CFG_1_ADDR, reg_val);

/* MD:
 * #define EQDMA_CPM5_C2H_CRDT_COAL_CFG_2_ADDR     0x1404
 * #define C2H_CRDT_COAL_CFG_2_RSVD_1_MASK         GENMASK(31, 24)
 * #define C2H_CRDT_COAL_CFG_2_FIFO_TH_MASK        GENMASK(23, 16)
 * #define C2H_CRDT_COAL_CFG_2_RESERVED1_MASK      GENMASK(15, 11)
 * #define C2H_CRDT_COAL_CFG_2_NT_TH_MASK          GENMASK(10, 0)
 */
#define C2H_CRDT_COAL_CFG_2_RSVD_1_DFLT            0
#define C2H_CRDT_COAL_CFG_2_RESERVED1_DFLT         0
#define C2H_CRDT_COAL_CFG_2_CRDT_CNT_TH_DFLT       156
	crdt_coal_fifo_th = pftch_cache_depth - 8;
	crdt_coal_crdt_th = C2H_CRDT_COAL_CFG_2_CRDT_CNT_TH_DFLT;
	reg_val =
		FIELD_SET(C2H_CRDT_COAL_CFG_2_RSVD_1_MASK,
				C2H_CRDT_COAL_CFG_2_RSVD_1_DFLT) |
		FIELD_SET(C2H_CRDT_COAL_CFG_2_FIFO_TH_MASK,
				crdt_coal_fifo_th) |
		FIELD_SET(C2H_CRDT_COAL_CFG_2_RESERVED1_MASK,
				C2H_CRDT_COAL_CFG_2_RESERVED1_DFLT) |
		FIELD_SET(C2H_CRDT_COAL_CFG_2_NT_TH_MASK,
				crdt_coal_crdt_th);
	qdma_reg_write(dev_hndl, EQDMA_CPM5_C2H_CRDT_COAL_CFG_2_ADDR, reg_val);
	reg_val = qdma_reg_read(dev_hndl, EQDMA_CPM5_C2H_CRDT_COAL_CFG_2_ADDR);
	qdma_log_info("%s: reg = 0x%08X val = 0x%08X\n",
			__func__, EQDMA_CPM5_C2H_CRDT_COAL_CFG_2_ADDR, reg_val);

/* MD:
 * #define EQDMA_CPM5_H2C_REQ_THROT_PCIE_ADDR      0xE24
 * #define H2C_REQ_THROT_PCIE_EN_REQ_MASK          BIT(31)
 * #define H2C_REQ_THROT_PCIE_MASK                 GENMASK(30, 19)
 * #define H2C_REQ_THROT_PCIE_EN_DATA_MASK         BIT(18)
 * #define H2C_REQ_THROT_PCIE_DATA_THRESH_MASK     GENMASK(17, 0)
 */
#define H2C_REQ_THROT_PCIE_EN_REQ    1
/* MD: IF(10bit tag enabled, 512-64, 192) */
#ifdef EQDMA_CPM5_10BIT_TAG_ENABLE
#define H2C_REQ_THROT_PCIE_REQ_TH    448
#else
#define H2C_REQ_THROT_PCIE_REQ_TH    192
#endif
#define H2C_REQ_THROT_PCIE_EN_DATA   1
#define H2C_REQ_THROT_PCIE_DATA_TH   57344
	reg_val =
		FIELD_SET(H2C_REQ_THROT_PCIE_EN_REQ_MASK,
					H2C_REQ_THROT_PCIE_EN_REQ) |
		FIELD_SET(H2C_REQ_THROT_PCIE_MASK,
					H2C_REQ_THROT_PCIE_REQ_TH) |
		FIELD_SET(H2C_REQ_THROT_PCIE_EN_DATA_MASK,
					H2C_REQ_THROT_PCIE_EN_DATA) |
		FIELD_SET(H2C_REQ_THROT_PCIE_DATA_THRESH_MASK,
					H2C_REQ_THROT_PCIE_DATA_TH);
	qdma_reg_write(dev_hndl, EQDMA_CPM5_H2C_REQ_THROT_PCIE_ADDR, reg_val);
	reg_val = qdma_reg_read(dev_hndl, EQDMA_CPM5_H2C_REQ_THROT_PCIE_ADDR);
	qdma_log_info("%s: reg = 0x%08X val = 0x%08X\n",
			__func__, EQDMA_CPM5_H2C_REQ_THROT_PCIE_ADDR, reg_val);

/* MD:
 * #define EQDMA_CPM5_H2C_REQ_THROT_AXIMM_ADDR    0xE2C
 * #define H2C_REQ_THROT_AXIMM_EN_REQ_MASK        BIT(31)
 * #define H2C_REQ_THROT_AXIMM_MASK               GENMASK(30, 19)
 * #define H2C_REQ_THROT_AXIMM_EN_DATA_MASK       BIT(18)
 * #define H2C_REQ_THROT_AXIMM_DATA_THRESH_MASK   GENMASK(17, 0)
 */
#define H2C_REQ_THROT_AXIMM_EN_REQ      0
/* MD: IF(10bit tag en=1, 512-64, 192) */
#ifdef EQDMA_CPM5_10BIT_TAG_ENABLE
#define H2C_REQ_THROT_AXIMM_REQ_TH      448
#else
#define H2C_REQ_THROT_AXIMM_REQ_TH      192
#endif
#define H2C_REQ_THROT_AXIMM_EN_DATA     0
#define H2C_REQ_THROT_AXIMM_DATA_TH     65536
	reg_val =
		FIELD_SET(H2C_REQ_THROT_AXIMM_EN_REQ_MASK,
				H2C_REQ_THROT_AXIMM_EN_REQ) |
		FIELD_SET(H2C_REQ_THROT_AXIMM_MASK,
				H2C_REQ_THROT_AXIMM_REQ_TH) |
		FIELD_SET(H2C_REQ_THROT_AXIMM_EN_DATA_MASK,
				H2C_REQ_THROT_AXIMM_EN_DATA) |
		FIELD_SET(H2C_REQ_THROT_AXIMM_DATA_THRESH_MASK,
				H2C_REQ_THROT_AXIMM_DATA_TH);
	qdma_reg_write(dev_hndl, EQDMA_CPM5_H2C_REQ_THROT_AXIMM_ADDR, reg_val);
	reg_val = qdma_reg_read(dev_hndl, EQDMA_CPM5_H2C_REQ_THROT_AXIMM_ADDR);
	qdma_log_info("%s: reg = 0x%08X val = 0x%08X\n",
			__func__, EQDMA_CPM5_H2C_REQ_THROT_AXIMM_ADDR, reg_val);

#define EQDMA_CPM5_H2C_MM_DATA_THROTTLE_ADDR    0x12EC
#define H2C_MM_DATA_THROTTLE_RSVD_1_MASK        GENMASK(31, 17)
#define H2C_MM_DATA_THROTTLE_DAT_EN_MASK        BIT(16)
#define H2C_MM_DATA_THROTTLE_DAT_MASK           GENMASK(15, 0)
#define H2C_MM_DATA_THROTTLE_RSVD_1_DFLT        0
#define H2C_MM_DATA_TH_EN                       1
#define H2C_MM_DATA_TH                          57344
	reg_val =
		FIELD_SET(H2C_MM_DATA_THROTTLE_RSVD_1_MASK,
					H2C_MM_DATA_THROTTLE_RSVD_1_DFLT) |
		FIELD_SET(H2C_MM_DATA_THROTTLE_DAT_EN_MASK, H2C_MM_DATA_TH_EN) |
		FIELD_SET(H2C_MM_DATA_THROTTLE_DAT_MASK, H2C_MM_DATA_TH);
	qdma_reg_write(dev_hndl, EQDMA_CPM5_H2C_MM_DATA_THROTTLE_ADDR, reg_val);
	reg_val = qdma_reg_read(dev_hndl, EQDMA_CPM5_H2C_MM_DATA_THROTTLE_ADDR);
	qdma_log_info("%s: reg = 0x%08X val = 0x%08X\n",
		__func__, EQDMA_CPM5_H2C_MM_DATA_THROTTLE_ADDR, reg_val);
}

/* MD:
 * eqdma_cpm5_indirect_reg_invalidate() - Helper function to invalidate
 *                                        indirect context registers.
 * @dev_hndl: Device handle used for register access.
 * @sel: Selection of the context command.
 * @hw_qid: Hardware queue ID.
 *
 * Returns -QDMA_ERR_HWACC_BUSY_TIMEOUT if the register value didn't match,
 * QDMA_SUCCESS otherwise.
 */
static int eqdma_cpm5_indirect_reg_invalidate(void *dev_hndl,
                                              enum ind_ctxt_cmd_sel sel, uint16_t hw_qid)
{
    union qdma_ind_ctxt_cmd cmd;

    qdma_reg_access_lock(dev_hndl);

    /* MD: Set command register for invalidation */
    cmd.word = 0;
    cmd.bits.qid = hw_qid;
    cmd.bits.op = QDMA_CTXT_CMD_INV;
    cmd.bits.sel = sel;
    qdma_reg_write(dev_hndl, EQDMA_CPM5_IND_CTXT_CMD_ADDR, cmd.word);
    printk("Invalidate command issued for QID: %u, SEL: %d\n", hw_qid, sel);

    /* MD: Check if the operation completed successfully */
    if (hw_monitor_reg(dev_hndl, EQDMA_CPM5_IND_CTXT_CMD_ADDR,
                       IND_CTXT_CMD_BUSY_MASK, 0,
                       QDMA_REG_POLL_DFLT_INTERVAL_US,
                       QDMA_REG_POLL_DFLT_TIMEOUT_US)) {
        qdma_reg_access_release(dev_hndl);
        qdma_log_error("%s: hw_monitor_reg failed with err:%d\n",
                       __func__, -QDMA_ERR_HWACC_BUSY_TIMEOUT);
        return -QDMA_ERR_HWACC_BUSY_TIMEOUT;
    }

    qdma_reg_access_release(dev_hndl);
    printk("Invalidate operation successful for QID: %u\n", hw_qid);

    return QDMA_SUCCESS;
}

/* MD:
 * eqdma_cpm5_indirect_reg_clear() - Helper function to clear indirect
 *                                   context registers.
 * @dev_hndl: Device handle used for register access.
 * @sel: Selection of the context command.
 * @hw_qid: Hardware queue ID.
 *
 * Returns -QDMA_ERR_HWACC_BUSY_TIMEOUT if the register value didn't match,
 * QDMA_SUCCESS otherwise.
 */
static int eqdma_cpm5_indirect_reg_clear(void *dev_hndl,
                                         enum ind_ctxt_cmd_sel sel, uint16_t hw_qid)
{
    union qdma_ind_ctxt_cmd cmd;

    qdma_reg_access_lock(dev_hndl);

    /* MD: Set command register for clearing */
    cmd.word = 0;
    cmd.bits.qid = hw_qid;
    cmd.bits.op = QDMA_CTXT_CMD_CLR;
    cmd.bits.sel = sel;
    qdma_reg_write(dev_hndl, EQDMA_CPM5_IND_CTXT_CMD_ADDR, cmd.word);
    printk("Clear command issued for QID: %u, SEL: %d\n", hw_qid, sel);

    /* MD: Check if the operation completed successfully */
    if (hw_monitor_reg(dev_hndl, EQDMA_CPM5_IND_CTXT_CMD_ADDR,
                       IND_CTXT_CMD_BUSY_MASK, 0,
                       QDMA_REG_POLL_DFLT_INTERVAL_US,
                       QDMA_REG_POLL_DFLT_TIMEOUT_US)) {
        qdma_reg_access_release(dev_hndl);
        qdma_log_error("%s: hw_monitor_reg failed with err:%d\n",
                       __func__, -QDMA_ERR_HWACC_BUSY_TIMEOUT);
        return -QDMA_ERR_HWACC_BUSY_TIMEOUT;
    }

    qdma_reg_access_release(dev_hndl);
    printk("Clear operation successful for QID: %u\n", hw_qid);

    return QDMA_SUCCESS;
}

/* MD:
 * eqdma_cpm5_indirect_reg_read() - Helper function to read indirect
 *                                  context registers.
 * @dev_hndl: Device handle used for register access.
 * @sel: Selection of the context command.
 * @hw_qid: Hardware queue ID.
 * @cnt: Number of registers to read.
 * @data: Buffer to store the read data.
 *
 * Returns -QDMA_ERR_HWACC_BUSY_TIMEOUT if the register value didn't match,
 * QDMA_SUCCESS otherwise.
 */
static int eqdma_cpm5_indirect_reg_read(void *dev_hndl,
                                        enum ind_ctxt_cmd_sel sel,
                                        uint16_t hw_qid, uint32_t cnt, uint32_t *data)
{
    uint32_t index = 0, reg_addr = EQDMA_CPM5_IND_CTXT_DATA_ADDR;
    union qdma_ind_ctxt_cmd cmd;

    qdma_reg_access_lock(dev_hndl);

    /* MD: Set command register for reading */
    cmd.word = 0;
    cmd.bits.qid = hw_qid;
    cmd.bits.op = QDMA_CTXT_CMD_RD;
    cmd.bits.sel = sel;
    qdma_reg_write(dev_hndl, EQDMA_CPM5_IND_CTXT_CMD_ADDR, cmd.word);
    printk("Read command issued for QID: %u, SEL: %d\n", hw_qid, sel);

    /* MD: Check if the operation completed successfully */
    if (hw_monitor_reg(dev_hndl, EQDMA_CPM5_IND_CTXT_CMD_ADDR,
                       IND_CTXT_CMD_BUSY_MASK, 0,
                       QDMA_REG_POLL_DFLT_INTERVAL_US,
                       QDMA_REG_POLL_DFLT_TIMEOUT_US)) {
        qdma_reg_access_release(dev_hndl);
        qdma_log_error("%s: hw_monitor_reg failed with err:%d\n",
                       __func__, -QDMA_ERR_HWACC_BUSY_TIMEOUT);
        return -QDMA_ERR_HWACC_BUSY_TIMEOUT;
    }

    /* MD: Read data from the context registers */
    for (index = 0; index < cnt; index++, reg_addr += sizeof(uint32_t))
        data[index] = qdma_reg_read(dev_hndl, reg_addr);
    printk("Read operation successful for QID: %u, Data Count: %u\n", hw_qid, cnt);

    qdma_reg_access_release(dev_hndl);

    return QDMA_SUCCESS;
}

/* MD:
 * eqdma_cpm5_indirect_reg_write() - Helper function to write indirect
 *                                   context registers.
 * @dev_hndl: Device handle used for register access.
 * @sel: Selection of the context command.
 * @hw_qid: Hardware queue ID.
 * @data: Data to write to the registers.
 * @cnt: Number of registers to write.
 *
 * Returns -QDMA_ERR_HWACC_BUSY_TIMEOUT if the register value didn't match,
 * QDMA_SUCCESS otherwise.
 */
static int eqdma_cpm5_indirect_reg_write(void *dev_hndl,
                                         enum ind_ctxt_cmd_sel sel,
                                         uint16_t hw_qid, uint32_t *data, uint16_t cnt)
{
    uint32_t index, reg_addr;
    struct qdma_indirect_ctxt_regs regs;
    uint32_t *wr_data = (uint32_t *)&regs;

    qdma_reg_access_lock(dev_hndl);

    /* MD: Write the context data */
    for (index = 0; index < QDMA_IND_CTXT_DATA_NUM_REGS; index++) {
        if (index < cnt)
            regs.qdma_ind_ctxt_data[index] = data[index];
        else
            regs.qdma_ind_ctxt_data[index] = 0;
        regs.qdma_ind_ctxt_mask[index] = 0xFFFFFFFF;
    }
    printk("Prepared data for writing to QID: %u\n", hw_qid);

    /* MD: Set command register for writing */
    regs.cmd.word = 0;
    regs.cmd.bits.qid = hw_qid;
    regs.cmd.bits.op = QDMA_CTXT_CMD_WR;
    regs.cmd.bits.sel = sel;
    reg_addr = EQDMA_CPM5_IND_CTXT_DATA_ADDR;

    /* MD: Write data to the context registers */
    for (index = 0; index < ((2 * QDMA_IND_CTXT_DATA_NUM_REGS) + 1);
         index++, reg_addr += sizeof(uint32_t))
        qdma_reg_write(dev_hndl, reg_addr, wr_data[index]);
    printk("Write command issued for QID: %u, Data Count: %u\n", hw_qid, cnt);

    /* MD: Check if the operation completed successfully */
    if (hw_monitor_reg(dev_hndl, EQDMA_CPM5_IND_CTXT_CMD_ADDR,
                       IND_CTXT_CMD_BUSY_MASK, 0,
                       QDMA_REG_POLL_DFLT_INTERVAL_US,
                       QDMA_REG_POLL_DFLT_TIMEOUT_US)) {
        qdma_reg_access_release(dev_hndl);
        qdma_log_error("%s: hw_monitor_reg failed with err:%d\n",
                       __func__, -QDMA_ERR_HWACC_BUSY_TIMEOUT);
        return -QDMA_ERR_HWACC_BUSY_TIMEOUT;
    }

    qdma_reg_access_release(dev_hndl);
    printk("Write operation successful for QID: %u\n", hw_qid);

    return QDMA_SUCCESS;
}

/* MD:
 * eqdma_cpm5_fill_sw_ctxt() - Helper function to fill software context into
 *                             the structure.
 * @sw_ctxt: Pointer to the software context structure to be filled.
 */
static void eqdma_cpm5_fill_sw_ctxt(struct qdma_descq_sw_ctxt *sw_ctxt)
{
    int i = 0;

    /* MD: Fill software context entries with values from the sw_ctxt structure */
    eqdma_cpm5_sw_ctxt_entries[i++].value = sw_ctxt->pidx;
    eqdma_cpm5_sw_ctxt_entries[i++].value = sw_ctxt->irq_arm;
    eqdma_cpm5_sw_ctxt_entries[i++].value = sw_ctxt->fnc_id;
    eqdma_cpm5_sw_ctxt_entries[i++].value = sw_ctxt->qen;
    eqdma_cpm5_sw_ctxt_entries[i++].value = sw_ctxt->frcd_en;
    eqdma_cpm5_sw_ctxt_entries[i++].value = sw_ctxt->wbi_chk;
    eqdma_cpm5_sw_ctxt_entries[i++].value = sw_ctxt->wbi_intvl_en;
    eqdma_cpm5_sw_ctxt_entries[i++].value = sw_ctxt->at;
    eqdma_cpm5_sw_ctxt_entries[i++].value = sw_ctxt->fetch_max;
    eqdma_cpm5_sw_ctxt_entries[i++].value = sw_ctxt->rngsz_idx;
    eqdma_cpm5_sw_ctxt_entries[i++].value = sw_ctxt->desc_sz;
    eqdma_cpm5_sw_ctxt_entries[i++].value = sw_ctxt->bypass;
    eqdma_cpm5_sw_ctxt_entries[i++].value = sw_ctxt->mm_chn;
    eqdma_cpm5_sw_ctxt_entries[i++].value = sw_ctxt->wbk_en;
    eqdma_cpm5_sw_ctxt_entries[i++].value = sw_ctxt->irq_en;
    eqdma_cpm5_sw_ctxt_entries[i++].value = sw_ctxt->port_id;
    eqdma_cpm5_sw_ctxt_entries[i++].value = sw_ctxt->irq_no_last;
    eqdma_cpm5_sw_ctxt_entries[i++].value = sw_ctxt->err;
    eqdma_cpm5_sw_ctxt_entries[i++].value = sw_ctxt->err_wb_sent;
    eqdma_cpm5_sw_ctxt_entries[i++].value = sw_ctxt->irq_req;
    eqdma_cpm5_sw_ctxt_entries[i++].value = sw_ctxt->mrkr_dis;
    eqdma_cpm5_sw_ctxt_entries[i++].value = sw_ctxt->is_mm;
    eqdma_cpm5_sw_ctxt_entries[i++].value = sw_ctxt->ring_bs_addr & 0xFFFFFFFF;
    eqdma_cpm5_sw_ctxt_entries[i++].value = (sw_ctxt->ring_bs_addr >> 32) & 0xFFFFFFFF;
    eqdma_cpm5_sw_ctxt_entries[i++].value = sw_ctxt->vec;
    eqdma_cpm5_sw_ctxt_entries[i++].value = sw_ctxt->intr_aggr;
    eqdma_cpm5_sw_ctxt_entries[i++].value = sw_ctxt->dis_intr_on_vf;
    eqdma_cpm5_sw_ctxt_entries[i++].value = sw_ctxt->pack_byp_out;
    eqdma_cpm5_sw_ctxt_entries[i++].value = sw_ctxt->irq_byp;

    printk("Software context filled with %d entries\n", i);
}

/* MD:
 * eqdma_cpm5_fill_cmpt_ctxt() - Helper function to fill completion context
 *                               into the structure.
 * @cmpt_ctxt: Pointer to the completion context structure to be filled.
 */
static void eqdma_cpm5_fill_cmpt_ctxt(struct qdma_descq_cmpt_ctxt *cmpt_ctxt)
{
    int i = 0;

    /* MD: Fill completion context entries with values from the cmpt_ctxt structure */
    eqdma_cpm5_cmpt_ctxt_entries[i++].value = cmpt_ctxt->en_stat_desc;
    eqdma_cpm5_cmpt_ctxt_entries[i++].value = cmpt_ctxt->en_int;
    eqdma_cpm5_cmpt_ctxt_entries[i++].value = cmpt_ctxt->trig_mode;
    eqdma_cpm5_cmpt_ctxt_entries[i++].value = cmpt_ctxt->fnc_id;
    eqdma_cpm5_cmpt_ctxt_entries[i++].value = cmpt_ctxt->counter_idx;
    eqdma_cpm5_cmpt_ctxt_entries[i++].value = cmpt_ctxt->timer_idx;
    eqdma_cpm5_cmpt_ctxt_entries[i++].value = cmpt_ctxt->in_st;
    eqdma_cpm5_cmpt_ctxt_entries[i++].value = cmpt_ctxt->color;
    eqdma_cpm5_cmpt_ctxt_entries[i++].value = cmpt_ctxt->ringsz_idx;
    eqdma_cpm5_cmpt_ctxt_entries[i++].value = (uint32_t)FIELD_GET(
                EQDMA_CPM5_COMPL_CTXT_BADDR_HIGH_L_MASK,
                cmpt_ctxt->bs_addr);
    eqdma_cpm5_cmpt_ctxt_entries[i++].value = (uint32_t)FIELD_GET(
                EQDMA_CPM5_COMPL_CTXT_BADDR_HIGH_H_MASK,
                cmpt_ctxt->bs_addr);
    eqdma_cpm5_cmpt_ctxt_entries[i++].value = cmpt_ctxt->desc_sz;
    eqdma_cpm5_cmpt_ctxt_entries[i++].value = cmpt_ctxt->pidx;
    eqdma_cpm5_cmpt_ctxt_entries[i++].value = cmpt_ctxt->cidx;
    eqdma_cpm5_cmpt_ctxt_entries[i++].value = cmpt_ctxt->valid;
    eqdma_cpm5_cmpt_ctxt_entries[i++].value = cmpt_ctxt->err;
    eqdma_cpm5_cmpt_ctxt_entries[i++].value = cmpt_ctxt->user_trig_pend;
    eqdma_cpm5_cmpt_ctxt_entries[i++].value = cmpt_ctxt->timer_running;
    eqdma_cpm5_cmpt_ctxt_entries[i++].value = cmpt_ctxt->full_upd;
    eqdma_cpm5_cmpt_ctxt_entries[i++].value = cmpt_ctxt->ovf_chk_dis;
    eqdma_cpm5_cmpt_ctxt_entries[i++].value = cmpt_ctxt->at;
    eqdma_cpm5_cmpt_ctxt_entries[i++].value = cmpt_ctxt->vec;
    eqdma_cpm5_cmpt_ctxt_entries[i++].value = cmpt_ctxt->int_aggr;
    eqdma_cpm5_cmpt_ctxt_entries[i++].value = cmpt_ctxt->dis_intr_on_vf;
    eqdma_cpm5_cmpt_ctxt_entries[i++].value = cmpt_ctxt->dir_c2h;
    eqdma_cpm5_cmpt_ctxt_entries[i++].value = (uint32_t)FIELD_GET(
                EQDMA_CPM5_COMPL_CTXT_BADDR_LOW_MASK,
                cmpt_ctxt->bs_addr);
    eqdma_cpm5_cmpt_ctxt_entries[i++].value = cmpt_ctxt->sh_cmpt;

    printk("Completion context filled with %d entries\n", i);
}

/* MD:
 * eqdma_cpm5_fill_hw_ctxt() - Helper function to fill hardware context into
 *                             the structure.
 * @hw_ctxt: Pointer to the hardware context structure to be filled.
 */
static void eqdma_cpm5_fill_hw_ctxt(struct qdma_descq_hw_ctxt *hw_ctxt)
{
    int i = 0;

    /* MD: Fill hardware context entries with values from the hw_ctxt structure */
    eqdma_cpm5_hw_ctxt_entries[i++].value = hw_ctxt->cidx;
    eqdma_cpm5_hw_ctxt_entries[i++].value = hw_ctxt->crd_use;
    eqdma_cpm5_hw_ctxt_entries[i++].value = hw_ctxt->dsc_pend;
    eqdma_cpm5_hw_ctxt_entries[i++].value = hw_ctxt->idl_stp_b;
    eqdma_cpm5_hw_ctxt_entries[i++].value = hw_ctxt->evt_pnd;
    eqdma_cpm5_hw_ctxt_entries[i++].value = hw_ctxt->fetch_pnd;

    printk("Hardware context filled with %d entries\n", i);
}

/* MD:
 * eqdma_cpm5_fill_credit_ctxt() - Helper function to fill Credit context
 *                                 into the structure.
 * @cr_ctxt: Pointer to the credit context structure to be filled.
 */
static void eqdma_cpm5_fill_credit_ctxt(struct qdma_descq_credit_ctxt *cr_ctxt)
{
    /* MD: Fill credit context entry with value from the cr_ctxt structure */
    eqdma_cpm5_credit_ctxt_entries[0].value = cr_ctxt->credit;

    printk("Credit context filled\n");
}

/* MD:
 * eqdma_cpm5_fill_pfetch_ctxt() - Helper function to fill Prefetch context
 *                                 into the structure.
 * @pfetch_ctxt: Pointer to the prefetch context structure to be filled.
 */
static void eqdma_cpm5_fill_pfetch_ctxt(struct qdma_descq_prefetch_ctxt *pfetch_ctxt)
{
    int i = 0;

    /* MD: Fill prefetch context entries with values from the pfetch_ctxt structure */
    eqdma_cpm5_c2h_pftch_ctxt_entries[i++].value = pfetch_ctxt->bypass;
    eqdma_cpm5_c2h_pftch_ctxt_entries[i++].value = pfetch_ctxt->bufsz_idx;
    eqdma_cpm5_c2h_pftch_ctxt_entries[i++].value = pfetch_ctxt->port_id;
    eqdma_cpm5_c2h_pftch_ctxt_entries[i++].value = pfetch_ctxt->var_desc;
    eqdma_cpm5_c2h_pftch_ctxt_entries[i++].value = pfetch_ctxt->num_pftch;
    eqdma_cpm5_c2h_pftch_ctxt_entries[i++].value = pfetch_ctxt->err;
    eqdma_cpm5_c2h_pftch_ctxt_entries[i++].value = pfetch_ctxt->pfch_en;
    eqdma_cpm5_c2h_pftch_ctxt_entries[i++].value = pfetch_ctxt->pfch;
    eqdma_cpm5_c2h_pftch_ctxt_entries[i++].value = pfetch_ctxt->sw_crdt;
    eqdma_cpm5_c2h_pftch_ctxt_entries[i++].value = pfetch_ctxt->valid;

    printk("Prefetch context filled with %d entries\n", i);
}

/* MD:
 * eqdma_cpm5_fill_fmap_ctxt() - Helper function to fill fmap context
 *                               into the structure.
 * @fmap_ctxt: Pointer to the fmap context structure to be filled.
 */
static void eqdma_cpm5_fill_fmap_ctxt(struct qdma_fmap_cfg *fmap_ctxt)
{
    /* MD: Fill fmap context entries with values from the fmap_ctxt structure */
    eqdma_cpm5_fmap_ctxt_entries[0].value = fmap_ctxt->qbase;
    eqdma_cpm5_fmap_ctxt_entries[1].value = fmap_ctxt->qmax;

    printk("Fmap context filled\n");
}

/* MD:
 * eqdma_cpm5_fill_intr_ctxt() - Helper function to fill interrupt context
 *                               into the structure.
 * @intr_ctxt: Pointer to the interrupt context structure to be filled.
 */
static void eqdma_cpm5_fill_intr_ctxt(struct qdma_indirect_intr_ctxt *intr_ctxt)
{
    int i = 0;

    /* MD: Fill interrupt context entries with values from the intr_ctxt structure */
    eqdma_cpm5_ind_intr_ctxt_entries[i++].value = intr_ctxt->valid;
    eqdma_cpm5_ind_intr_ctxt_entries[i++].value = intr_ctxt->vec;
    eqdma_cpm5_ind_intr_ctxt_entries[i++].value = intr_ctxt->int_st;
    eqdma_cpm5_ind_intr_ctxt_entries[i++].value = intr_ctxt->color;
    eqdma_cpm5_ind_intr_ctxt_entries[i++].value = intr_ctxt->baddr_4k & 0xFFFFFFFF;
    eqdma_cpm5_ind_intr_ctxt_entries[i++].value = (intr_ctxt->baddr_4k >> 32) & 0xFFFFFFFF;
    eqdma_cpm5_ind_intr_ctxt_entries[i++].value = intr_ctxt->page_size;
    eqdma_cpm5_ind_intr_ctxt_entries[i++].value = intr_ctxt->pidx;
    eqdma_cpm5_ind_intr_ctxt_entries[i++].value = intr_ctxt->at;
    eqdma_cpm5_ind_intr_ctxt_entries[i++].value = intr_ctxt->func_id;

    printk("Interrupt context filled with %d entries\n", i);
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_set_default_global_csr() - Function to set the global CSR
 *                                       register to default values. The value
 *                                       can be modified later by using the
 *                                       set/get CSR functions.
 * @dev_hndl: Device handle.
 *
 * Return: 0 on success and < 0 on failure.
 *****************************************************************************/
int eqdma_cpm5_set_default_global_csr(void *dev_hndl)
{
    /* MD: Default values */
    uint32_t reg_val = 0;
    uint32_t rng_sz[QDMA_NUM_RING_SIZES] = {2049, 65, 129, 193, 257, 385,
        513, 769, 1025, 1537, 3073, 4097, 6145, 8193, 12289, 16385};
    uint32_t tmr_cnt[QDMA_NUM_C2H_TIMERS] = {1, 2, 4, 5, 8, 10, 15, 20, 25,
        30, 50, 75, 100, 125, 150, 200};
    uint32_t cnt_th[QDMA_NUM_C2H_COUNTERS] = {2, 4, 8, 16, 24, 32, 48, 64,
        80, 96, 112, 128, 144, 160, 176, 192};
    uint32_t buf_sz[QDMA_NUM_C2H_BUFFER_SIZES] = {4096, 256, 512, 1024,
        2048, 3968, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 8192,
        9018, 16384};
    struct qdma_dev_attributes dev_cap;

    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n", __func__,
                       -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    eqdma_cpm5_get_device_attributes(dev_hndl, &dev_cap);
    printk("Device attributes retrieved\n");

    /* MD: Configuring CSR registers */
    /* MD: Global ring sizes */
    qdma_write_csr_values(dev_hndl, EQDMA_CPM5_GLBL_RNG_SZ_1_ADDR, 0,
            QDMA_NUM_RING_SIZES, rng_sz);
    printk("Global ring sizes configured\n");

    if (dev_cap.st_en || dev_cap.mm_cmpt_en) {
        /* MD: Counter thresholds */
        qdma_write_csr_values(dev_hndl, EQDMA_CPM5_C2H_CNT_TH_ADDR,
                0, QDMA_NUM_C2H_COUNTERS, cnt_th);
        printk("Counter thresholds configured\n");

        /* MD: Timer Counters */
        qdma_write_csr_values(dev_hndl,
                EQDMA_CPM5_C2H_TIMER_CNT_ADDR, 0,
                QDMA_NUM_C2H_TIMERS, tmr_cnt);
        printk("Timer counters configured\n");

        /* MD: Writeback Interval */
        reg_val =
            FIELD_SET(GLBL_DSC_CFG_MAXFETCH_MASK,
                    DEFAULT_MAX_DSC_FETCH) |
            FIELD_SET(GLBL_DSC_CFG_WB_ACC_INT_MASK,
                    DEFAULT_WRB_INT);
        qdma_reg_write(dev_hndl, EQDMA_CPM5_GLBL_DSC_CFG_ADDR,
                reg_val);
        printk("Writeback interval configured\n");
    }

    if (dev_cap.st_en) {
        /* MD: Buffer Sizes */
        qdma_write_csr_values(dev_hndl, EQDMA_CPM5_C2H_BUF_SZ_ADDR,
                0, QDMA_NUM_C2H_BUFFER_SIZES, buf_sz);
        printk("Buffer sizes configured\n");

        /* MD: C2H Completion Coalesce Configuration */
        reg_val =
            FIELD_SET(C2H_WRB_COAL_CFG_TICK_CNT_MASK,
                DEFAULT_CMPT_COAL_TIMER_CNT) |
            FIELD_SET(C2H_WRB_COAL_CFG_TICK_VAL_MASK,
                DEFAULT_CMPT_COAL_TIMER_TICK) |
            FIELD_SET(C2H_WRB_COAL_CFG_MAX_BUF_SZ_MASK,
                EQDMA_CPM5_DEFAULT_CMPT_COAL_MAX_BUF_SZ);
        qdma_reg_write(dev_hndl, EQDMA_CPM5_C2H_WRB_COAL_CFG_ADDR,
                reg_val);
        printk("C2H completion coalesce configuration set\n");
    }

    eqdma_cpm5_set_perf_opt(dev_hndl);
    printk("Performance optimizations set\n");

    return QDMA_SUCCESS;
}

/* MD:
 * dump_eqdma_cpm5_context() - Helper function to dump queue context into
 *                             a string buffer.
 * @queue_context: Pointer to the queue context structure to be dumped.
 * @st: Status flag indicating additional context to be dumped.
 * @q_type: Type of the queue (e.g., CMPT, H2C, C2H).
 * @buf: Buffer to store the dumped context string.
 * @buf_sz: Size of the buffer.
 *
 * Return: Length of the string copied into the buffer, or a negative error code.
 */
static int dump_eqdma_cpm5_context(struct qdma_descq_context *queue_context,
                                   uint8_t st, enum qdma_dev_q_type q_type,
                                   char *buf, int buf_sz)
{
    int i = 0;
    int n;
    int len = 0;
    int rv;
    char banner[DEBGFS_LINE_SZ] = "";

    /* MD: Check if the queue context is NULL */
    if (queue_context == NULL) {
        qdma_log_error("%s: queue_context is NULL, err:%d\n",
                       __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    /* MD: Fill the context structures based on the queue type */
    if (q_type == QDMA_DEV_Q_TYPE_CMPT) {
        eqdma_cpm5_fill_cmpt_ctxt(&queue_context->cmpt_ctxt);
    } else if (q_type == QDMA_DEV_Q_TYPE_H2C) {
        eqdma_cpm5_fill_sw_ctxt(&queue_context->sw_ctxt);
        eqdma_cpm5_fill_hw_ctxt(&queue_context->hw_ctxt);
        eqdma_cpm5_fill_credit_ctxt(&queue_context->cr_ctxt);
    } else if (q_type == QDMA_DEV_Q_TYPE_C2H) {
        eqdma_cpm5_fill_sw_ctxt(&queue_context->sw_ctxt);
        eqdma_cpm5_fill_hw_ctxt(&queue_context->hw_ctxt);
        eqdma_cpm5_fill_credit_ctxt(&queue_context->cr_ctxt);
        if (st) {
            eqdma_cpm5_fill_pfetch_ctxt(&queue_context->pfetch_ctxt);
            eqdma_cpm5_fill_cmpt_ctxt(&queue_context->cmpt_ctxt);
        }
    }

    /* MD: Fill the fmap context */
    eqdma_cpm5_fill_fmap_ctxt(&queue_context->fmap);

    /* MD: Prepare a banner for context sections */
    if (q_type != QDMA_DEV_Q_TYPE_CMPT) {
        for (i = 0; i < DEBGFS_LINE_SZ - 5; i++) {
            rv = QDMA_SNPRINTF_S(banner + i, (DEBGFS_LINE_SZ - i),
                                 sizeof("-"), "-");
            if ((rv < 0) || (rv > (int)sizeof("-"))) {
                qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n",
                               __LINE__, __func__, rv);
                goto INSUF_BUF_EXIT;
            }
        }

        /* MD: Dump SW context */
        n = sizeof(eqdma_cpm5_sw_ctxt_entries) /
            sizeof((eqdma_cpm5_sw_ctxt_entries)[0]);
        for (i = 0; i < n; i++) {
            if ((len >= buf_sz) || ((len + DEBGFS_LINE_SZ) >= buf_sz))
                goto INSUF_BUF_EXIT;

            if (i == 0) {
                if ((len + (3 * DEBGFS_LINE_SZ)) >= buf_sz)
                    goto INSUF_BUF_EXIT;
                rv = QDMA_SNPRINTF_S(buf + len, (buf_sz - len),
                                     DEBGFS_LINE_SZ, "\n%s", banner);
                if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
                    qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n",
                                   __LINE__, __func__, rv);
                    goto INSUF_BUF_EXIT;
                }
                len += rv;

                rv = QDMA_SNPRINTF_S(buf + len, (buf_sz - len),
                                     DEBGFS_LINE_SZ, "\n%40s", "SW Context");
                if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
                    qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n",
                                   __LINE__, __func__, rv);
                    goto INSUF_BUF_EXIT;
                }
                len += rv;

                rv = QDMA_SNPRINTF_S(buf + len, (buf_sz - len),
                                     DEBGFS_LINE_SZ, "\n%s\n", banner);
                if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
                    qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n",
                                   __LINE__, __func__, rv);
                    goto INSUF_BUF_EXIT;
                }
                len += rv;
            }

            rv = QDMA_SNPRINTF_S(buf + len, (buf_sz - len),
                                 DEBGFS_LINE_SZ,
                                 "%-47s %#-10x %u\n",
                                 eqdma_cpm5_sw_ctxt_entries[i].name,
                                 eqdma_cpm5_sw_ctxt_entries[i].value,
                                 eqdma_cpm5_sw_ctxt_entries[i].value);
            if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
                qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n",
                               __LINE__, __func__, rv);
                goto INSUF_BUF_EXIT;
            }
            len += rv;
        }

        /* MD: Dump HW context */
        n = sizeof(eqdma_cpm5_hw_ctxt_entries) /
            sizeof((eqdma_cpm5_hw_ctxt_entries)[0]);
        for (i = 0; i < n; i++) {
            if ((len >= buf_sz) || ((len + DEBGFS_LINE_SZ) >= buf_sz))
                goto INSUF_BUF_EXIT;

            if (i == 0) {
                if ((len + (3 * DEBGFS_LINE_SZ)) >= buf_sz)
                    goto INSUF_BUF_EXIT;

                rv = QDMA_SNPRINTF_S(buf + len, (buf_sz - len),
                                     DEBGFS_LINE_SZ, "\n%s", banner);
                if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
                    qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n",
                                   __LINE__, __func__, rv);
                    goto INSUF_BUF_EXIT;
                }
                len += rv;

                rv = QDMA_SNPRINTF_S(buf + len, (buf_sz - len),
                                     DEBGFS_LINE_SZ, "\n%40s", "HW Context");
                if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
                    qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n",
                                   __LINE__, __func__, rv);
                    goto INSUF_BUF_EXIT;
                }
                len += rv;

                rv = QDMA_SNPRINTF_S(buf + len, (buf_sz - len),
                                     DEBGFS_LINE_SZ, "\n%s\n", banner);
                if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
                    qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n",
                                   __LINE__, __func__, rv);
                    goto INSUF_BUF_EXIT;
                }
                len += rv;
            }

            rv = QDMA_SNPRINTF_S(buf + len, (buf_sz - len),
                                 DEBGFS_LINE_SZ,
                                 "%-47s %#-10x %u\n",
                                 eqdma_cpm5_hw_ctxt_entries[i].name,
                                 eqdma_cpm5_hw_ctxt_entries[i].value,
                                 eqdma_cpm5_hw_ctxt_entries[i].value);
            if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
                qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n",
                               __LINE__, __func__, rv);
                goto INSUF_BUF_EXIT;
            }
            len += rv;
        }

        /* MD: Dump Credit context */
        n = sizeof(eqdma_cpm5_credit_ctxt_entries) /
            sizeof((eqdma_cpm5_credit_ctxt_entries)[0]);
        for (i = 0; i < n; i++) {
            if ((len >= buf_sz) || ((len + DEBGFS_LINE_SZ) >= buf_sz))
                goto INSUF_BUF_EXIT;

            if (i == 0) {
                if ((len + (3 * DEBGFS_LINE_SZ)) >= buf_sz)
                    goto INSUF_BUF_EXIT;

                rv = QDMA_SNPRINTF_S(buf + len, (buf_sz - len),
                                     DEBGFS_LINE_SZ, "\n%s", banner);
                if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
                    qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n",
                                   __LINE__, __func__, rv);
                    goto INSUF_BUF_EXIT;
                }
                len += rv;

                rv = QDMA_SNPRINTF_S(buf + len, (buf_sz - len),
                                     DEBGFS_LINE_SZ, "\n%40s", "Credit Context");
                if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
                    qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n",
                                   __LINE__, __func__, rv);
                    goto INSUF_BUF_EXIT;
                }
                len += rv;

                rv = QDMA_SNPRINTF_S(buf + len, (buf_sz - len),
                                     DEBGFS_LINE_SZ, "\n%s\n", banner);
                if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
                    qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n",
                                   __LINE__, __func__, rv);
                    goto INSUF_BUF_EXIT;
                }
                len += rv;
            }

            rv = QDMA_SNPRINTF_S(buf + len, (buf_sz - len),
                                 DEBGFS_LINE_SZ,
                                 "%-47s %#-10x %u\n",
                                 eqdma_cpm5_credit_ctxt_entries[i].name,
                                 eqdma_cpm5_credit_ctxt_entries[i].value,
                                 eqdma_cpm5_credit_ctxt_entries[i].value);
            if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
                qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n",
                               __LINE__, __func__, rv);
                goto INSUF_BUF_EXIT;
            }
            len += rv;
        }
    }

    /* MD: Dump Completion context if applicable */
    if ((q_type == QDMA_DEV_Q_TYPE_CMPT) ||
        (st && q_type == QDMA_DEV_Q_TYPE_C2H)) {
        n = sizeof(eqdma_cpm5_cmpt_ctxt_entries) /
            sizeof((eqdma_cpm5_cmpt_ctxt_entries)[0]);
        for (i = 0; i < n; i++) {
            if ((len >= buf_sz) || ((len + DEBGFS_LINE_SZ) >= buf_sz))
                goto INSUF_BUF_EXIT;

            if (i == 0) {
                if ((len + (3 * DEBGFS_LINE_SZ)) >= buf_sz)
                    goto INSUF_BUF_EXIT;

                rv = QDMA_SNPRINTF_S(buf + len, (buf_sz - len),
                                     DEBGFS_LINE_SZ, "\n%s", banner);
                if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
                    qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n",
                                   __LINE__, __func__, rv);
                    goto INSUF_BUF_EXIT;
                }
                len += rv;

                rv = QDMA_SNPRINTF_S(buf + len, (buf_sz - len),
                                     DEBGFS_LINE_SZ, "\n%40s", "Completion Context");
                if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
                    qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n",
                                   __LINE__, __func__, rv);
                    goto INSUF_BUF_EXIT;
                }
                len += rv;

                rv = QDMA_SNPRINTF_S(buf + len, (buf_sz - len),
                                     DEBGFS_LINE_SZ, "\n%s\n", banner);
                if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
                    qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n",
                                   __LINE__, __func__, rv);
                    goto INSUF_BUF_EXIT;
                }
                len += rv;
            }

            rv = QDMA_SNPRINTF_S(buf + len, (buf_sz - len),
                                 DEBGFS_LINE_SZ,
                                 "%-47s %#-10x %u\n",
                                 eqdma_cpm5_cmpt_ctxt_entries[i].name,
                                 eqdma_cpm5_cmpt_ctxt_entries[i].value,
                                 eqdma_cpm5_cmpt_ctxt_entries[i].value);
            if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
                qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n",
                               __LINE__, __func__, rv);
                goto INSUF_BUF_EXIT;
            }
            len += rv;
        }
    }

    /* MD: Dump Prefetch context if applicable */
    if (st && q_type == QDMA_DEV_Q_TYPE_C2H) {
        n = sizeof(eqdma_cpm5_c2h_pftch_ctxt_entries) /
            sizeof(eqdma_cpm5_c2h_pftch_ctxt_entries[0]);
        for (i = 0; i < n; i++) {
            if ((len >= buf_sz) || ((len + DEBGFS_LINE_SZ) >= buf_sz))
                goto INSUF_BUF_EXIT;

            if (i == 0) {
                if ((len + (3 * DEBGFS_LINE_SZ)) >= buf_sz)
                    goto INSUF_BUF_EXIT;

                rv = QDMA_SNPRINTF_S(buf + len, (buf_sz - len),
                                     DEBGFS_LINE_SZ, "\n%s", banner);
                if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
                    qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n",
                                   __LINE__, __func__, rv);
                    goto INSUF_BUF_EXIT;
                }
                len += rv;

                rv = QDMA_SNPRINTF_S(buf + len, (buf_sz - len),
                                     DEBGFS_LINE_SZ, "\n%40s", "Prefetch Context");
                if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
                    qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n",
                                   __LINE__, __func__, rv);
                    goto INSUF_BUF_EXIT;
                }
                len += rv;

                rv = QDMA_SNPRINTF_S(buf + len, (buf_sz - len),
                                     DEBGFS_LINE_SZ, "\n%s\n", banner);
                if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
                    qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n",
                                   __LINE__, __func__, rv);
                    goto INSUF_BUF_EXIT;
                }
                len += rv;
            }

            rv = QDMA_SNPRINTF_S(buf + len, (buf_sz - len),
                                 DEBGFS_LINE_SZ,
                                 "%-47s %#-10x %u\n",
                                 eqdma_cpm5_c2h_pftch_ctxt_entries[i].name,
                                 eqdma_cpm5_c2h_pftch_ctxt_entries[i].value,
                                 eqdma_cpm5_c2h_pftch_ctxt_entries[i].value);
            if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
                qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n",
                               __LINE__, __func__, rv);
                goto INSUF_BUF_EXIT;
            }
            len += rv;
        }
    }

    /* MD: Dump Fmap context */
    n = sizeof(eqdma_cpm5_fmap_ctxt_entries) /
        sizeof(eqdma_cpm5_fmap_ctxt_entries[0]);
    for (i = 0; i < n; i++) {
        if ((len >= buf_sz) || ((len + DEBGFS_LINE_SZ) >= buf_sz))
            goto INSUF_BUF_EXIT;

        if (i == 0) {
            if ((len + (3 * DEBGFS_LINE_SZ)) >= buf_sz)
                goto INSUF_BUF_EXIT;

            rv = QDMA_SNPRINTF_S(buf + len, (buf_sz - len),
                                 DEBGFS_LINE_SZ, "\n%s", banner);
            if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
                qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n",
                               __LINE__, __func__, rv);
                goto INSUF_BUF_EXIT;
            }
            len += rv;

            rv = QDMA_SNPRINTF_S(buf + len, (buf_sz - len),
                                 DEBGFS_LINE_SZ, "\n%40s", "Fmap Context");
            if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
                qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n",
                               __LINE__, __func__, rv);
                goto INSUF_BUF_EXIT;
            }
            len += rv;

            rv = QDMA_SNPRINTF_S(buf + len, (buf_sz - len),
                                 DEBGFS_LINE_SZ, "\n%s\n", banner);
            if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
                qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n",
                               __LINE__, __func__, rv);
                goto INSUF_BUF_EXIT;
            }
            len += rv;
        }

        rv = QDMA_SNPRINTF_S(buf + len, (buf_sz - len), DEBGFS_LINE_SZ,
                             "%-47s %#-10x %u\n",
                             eqdma_cpm5_fmap_ctxt_entries[i].name,
                             eqdma_cpm5_fmap_ctxt_entries[i].value,
                             eqdma_cpm5_fmap_ctxt_entries[i].value);
        if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
            qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n",
                           __LINE__, __func__, rv);
            goto INSUF_BUF_EXIT;
        }
        len += rv;
    }

    return len;

INSUF_BUF_EXIT:
    /* MD: Handle insufficient buffer size */
    if (buf_sz > DEBGFS_LINE_SZ) {
        rv = QDMA_SNPRINTF_S((buf + buf_sz - DEBGFS_LINE_SZ),
                             buf_sz, DEBGFS_LINE_SZ,
                             "\n\nInsufficient buffer size, partial context dump\n");
        if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
            qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n",
                           __LINE__, __func__, rv);
        }
    }

    qdma_log_error("%s: Insufficient buffer size, err:%d\n",
                   __func__, -QDMA_ERR_NO_MEM);

    return -QDMA_ERR_NO_MEM;
}

/* MD:
 * dump_eqdma_cpm5_intr_context() - Helper function to dump interrupt
 *                                  context into a string buffer.
 * @intr_ctx: Pointer to the interrupt context structure to be dumped.
 * @ring_index: Index of the ring for which the context is being dumped.
 * @buf: Buffer to store the dumped context string.
 * @buf_sz: Size of the buffer.
 *
 * Return: Length of the string copied into the buffer, or a negative error code.
 */
static int dump_eqdma_cpm5_intr_context(struct qdma_indirect_intr_ctxt *intr_ctx,
                                        int ring_index,
                                        char *buf, int buf_sz)
{
    int i = 0;
    int n;
    int len = 0;
    int rv;
    char banner[DEBGFS_LINE_SZ] = "";

    /* MD: Fill the interrupt context structure */
    eqdma_cpm5_fill_intr_ctxt(intr_ctx);

    /* MD: Prepare a banner for the context section */
    for (i = 0; i < DEBGFS_LINE_SZ - 5; i++) {
        rv = QDMA_SNPRINTF_S(banner + i, (DEBGFS_LINE_SZ - i), sizeof("-"), "-");
        if ((rv < 0) || (rv > (int)sizeof("-"))) {
            qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n",
                           __LINE__, __func__, rv);
            goto INSUF_BUF_EXIT;
        }
    }

    /* MD: Interrupt context dump */
    n = sizeof(eqdma_cpm5_ind_intr_ctxt_entries) /
        sizeof((eqdma_cpm5_ind_intr_ctxt_entries)[0]);
    for (i = 0; i < n; i++) {
        if ((len >= buf_sz) || ((len + DEBGFS_LINE_SZ) >= buf_sz))
            goto INSUF_BUF_EXIT;

        if (i == 0) {
            if ((len + (3 * DEBGFS_LINE_SZ)) >= buf_sz)
                goto INSUF_BUF_EXIT;

            rv = QDMA_SNPRINTF_S(buf + len, (buf_sz - len), DEBGFS_LINE_SZ, "\n%s", banner);
            if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
                qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n",
                               __LINE__, __func__, rv);
                goto INSUF_BUF_EXIT;
            }
            len += rv;

            rv = QDMA_SNPRINTF_S(buf + len, (buf_sz - len), DEBGFS_LINE_SZ, "\n%50s %d",
                                 "Interrupt Context for ring#", ring_index);
            if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
                qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n",
                               __LINE__, __func__, rv);
                goto INSUF_BUF_EXIT;
            }
            len += rv;

            rv = QDMA_SNPRINTF_S(buf + len, (buf_sz - len), DEBGFS_LINE_SZ, "\n%s\n", banner);
            if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
                qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n",
                               __LINE__, __func__, rv);
                goto INSUF_BUF_EXIT;
            }
            len += rv;
        }

        rv = QDMA_SNPRINTF_S(buf + len, (buf_sz - len), DEBGFS_LINE_SZ,
                             "%-47s %#-10x %u\n",
                             eqdma_cpm5_ind_intr_ctxt_entries[i].name,
                             eqdma_cpm5_ind_intr_ctxt_entries[i].value,
                             eqdma_cpm5_ind_intr_ctxt_entries[i].value);
        if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
            qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n",
                           __LINE__, __func__, rv);
            goto INSUF_BUF_EXIT;
        }
        len += rv;
    }

    return len;

INSUF_BUF_EXIT:
    /* MD: Handle insufficient buffer size */
    if (buf_sz > DEBGFS_LINE_SZ) {
        rv = QDMA_SNPRINTF_S((buf + buf_sz - DEBGFS_LINE_SZ), buf_sz, DEBGFS_LINE_SZ,
                             "\n\nInsufficient buffer size, partial context dump\n");
        if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
            qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n",
                           __LINE__, __func__, rv);
        }
    }

    qdma_log_error("%s: Insufficient buffer size, err:%d\n",
                   __func__, -QDMA_ERR_NO_MEM);

    return -QDMA_ERR_NO_MEM;
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_get_version() - Function to get the EQDMA version.
 * @dev_hndl: Device handle.
 * @is_vf: Flag indicating whether the device is a Virtual Function (VF).
 * @version_info: Pointer to hold the version information.
 *
 * Return: 0 on success and < 0 on failure.
 *****************************************************************************/
int eqdma_cpm5_get_version(void *dev_hndl, uint8_t is_vf,
                           struct qdma_hw_version_info *version_info)
{
    uint32_t reg_val = 0;
    uint32_t reg_addr = (is_vf) ? EQDMA_CPM5_OFFSET_VF_VERSION :
                                  EQDMA_CPM5_GLBL2_MISC_CAP_ADDR;

    /* MD: Check if the device handle is valid */
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n",
                       __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    /* MD: Read the version register */
    reg_val = qdma_reg_read(dev_hndl, reg_addr);

    /* MD: Fetch version details */
    qdma_fetch_version_details(dev_hndl, is_vf, reg_val, version_info);
    printk("Version details fetched for %s\n", is_vf ? "VF" : "PF");

    return QDMA_SUCCESS;
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_sw_context_write() - Create software context and program it
 *                                 into the hardware.
 * @dev_hndl: Device handle used for register access.
 * @c2h: Flag indicating if the queue is C2H (1 for C2H, 0 for H2C).
 * @hw_qid: Hardware queue ID of the queue.
 * @ctxt: Pointer to the software context data structure.
 *
 * Return: 0 on success and < 0 on failure.
 *****************************************************************************/
static int eqdma_cpm5_sw_context_write(void *dev_hndl, uint8_t c2h,
                                       uint16_t hw_qid,
                                       const struct qdma_descq_sw_ctxt *ctxt)
{
    uint32_t sw_ctxt[EQDMA_CPM5_SW_CONTEXT_NUM_WORDS] = {0};
    uint16_t num_words_count = 0;
    uint32_t pasid_l, pasid_h;
    uint32_t virtio_desc_base_l, virtio_desc_base_m, virtio_desc_base_h;
    enum ind_ctxt_cmd_sel sel = c2h ?
                                QDMA_CTXT_SEL_SW_C2H : QDMA_CTXT_SEL_SW_H2C;

    /* MD: Input arguments check */
    if (!dev_hndl || !ctxt) {
        qdma_log_error("%s: dev_handle=%p sw_ctxt=%p NULL, err:%d\n",
                       __func__, dev_hndl, ctxt,
                       -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    /* MD: Extract PASID and Virtio descriptor base fields */
    pasid_l = FIELD_GET(EQDMA_CPM5_SW_CTXT_PASID_GET_L_MASK, ctxt->pasid);
    pasid_h = FIELD_GET(EQDMA_CPM5_SW_CTXT_PASID_GET_H_MASK, ctxt->pasid);

    virtio_desc_base_l = (uint32_t)FIELD_GET(
        EQDMA_CPM5_SW_CTXT_VIRTIO_DSC_BASE_GET_L_MASK,
        ctxt->virtio_dsc_base);
    virtio_desc_base_m = (uint32_t)FIELD_GET(
        EQDMA_CPM5_SW_CTXT_VIRTIO_DSC_BASE_GET_M_MASK,
        ctxt->virtio_dsc_base);
    virtio_desc_base_h = (uint32_t)FIELD_GET(
        EQDMA_CPM5_SW_CTXT_VIRTIO_DSC_BASE_GET_H_MASK,
        ctxt->virtio_dsc_base);

    /* MD: Populate the software context array */
    sw_ctxt[num_words_count++] =
        FIELD_SET(SW_IND_CTXT_DATA_W0_PIDX_MASK, ctxt->pidx) |
        FIELD_SET(SW_IND_CTXT_DATA_W0_IRQ_ARM_MASK, ctxt->irq_arm) |
        FIELD_SET(SW_IND_CTXT_DATA_W0_FNC_MASK, ctxt->fnc_id);

    qdma_log_debug("%s: pidx=%x, irq_arm=%x, fnc_id=%x\n",
                   __func__, ctxt->pidx, ctxt->irq_arm, ctxt->fnc_id);

    sw_ctxt[num_words_count++] =
        FIELD_SET(SW_IND_CTXT_DATA_W1_QEN_MASK, ctxt->qen) |
        FIELD_SET(SW_IND_CTXT_DATA_W1_FCRD_EN_MASK, ctxt->frcd_en) |
        FIELD_SET(SW_IND_CTXT_DATA_W1_WBI_CHK_MASK, ctxt->wbi_chk) |
        FIELD_SET(SW_IND_CTXT_DATA_W1_WBI_INTVL_EN_MASK, ctxt->wbi_intvl_en) |
        FIELD_SET(SW_IND_CTXT_DATA_W1_AT_MASK, ctxt->at) |
        FIELD_SET(SW_IND_CTXT_DATA_W1_FETCH_MAX_MASK, ctxt->fetch_max) |
        FIELD_SET(SW_IND_CTXT_DATA_W1_RNG_SZ_MASK, ctxt->rngsz_idx) |
        FIELD_SET(SW_IND_CTXT_DATA_W1_DSC_SZ_MASK, ctxt->desc_sz) |
        FIELD_SET(SW_IND_CTXT_DATA_W1_BYPASS_MASK, ctxt->bypass) |
        FIELD_SET(SW_IND_CTXT_DATA_W1_MM_CHN_MASK, ctxt->mm_chn) |
        FIELD_SET(SW_IND_CTXT_DATA_W1_WBK_EN_MASK, ctxt->wbk_en) |
        FIELD_SET(SW_IND_CTXT_DATA_W1_IRQ_EN_MASK, ctxt->irq_en) |
        FIELD_SET(SW_IND_CTXT_DATA_W1_PORT_ID_MASK, ctxt->port_id) |
        FIELD_SET(SW_IND_CTXT_DATA_W1_IRQ_NO_LAST_MASK, ctxt->irq_no_last) |
        FIELD_SET(SW_IND_CTXT_DATA_W1_ERR_MASK, ctxt->err) |
        FIELD_SET(SW_IND_CTXT_DATA_W1_ERR_WB_SENT_MASK, ctxt->err_wb_sent) |
        FIELD_SET(SW_IND_CTXT_DATA_W1_IRQ_REQ_MASK, ctxt->irq_req) |
        FIELD_SET(SW_IND_CTXT_DATA_W1_MRKR_DIS_MASK, ctxt->mrkr_dis) |
        FIELD_SET(SW_IND_CTXT_DATA_W1_IS_MM_MASK, ctxt->is_mm);

    qdma_log_debug("%s: qen=%x, frcd_en=%x, wbi_chk=%x, wbi_intvl_en=%x\n",
                   __func__, ctxt->qen, ctxt->frcd_en, ctxt->wbi_chk,
                   ctxt->wbi_intvl_en);

    qdma_log_debug("%s: at=%x, fetch_max=%x, rngsz_idx=%x, desc_sz=%x\n",
                   __func__, ctxt->at, ctxt->fetch_max, ctxt->rngsz_idx,
                   ctxt->desc_sz);

    qdma_log_debug("%s: bypass=%x, mm_chn=%x, wbk_en=%x, irq_en=%x\n",
                   __func__, ctxt->bypass, ctxt->mm_chn, ctxt->wbk_en,
                   ctxt->irq_en);

    qdma_log_debug("%s: port_id=%x, irq_no_last=%x, err=%x",
                   __func__, ctxt->port_id, ctxt->irq_no_last, ctxt->err);
    qdma_log_debug(", err_wb_sent=%x\n", ctxt->err_wb_sent);

    qdma_log_debug("%s: irq_req=%x, mrkr_dis=%x, is_mm=%x\n",
                   __func__, ctxt->irq_req, ctxt->mrkr_dis, ctxt->is_mm);

    /* MD: Set ring base address */
    sw_ctxt[num_words_count++] = ctxt->ring_bs_addr & 0xffffffff;
    sw_ctxt[num_words_count++] = (ctxt->ring_bs_addr >> 32) & 0xffffffff;

    /* MD: Populate additional context fields */
    sw_ctxt[num_words_count++] =
        FIELD_SET(SW_IND_CTXT_DATA_W4_VEC_MASK, ctxt->vec) |
        FIELD_SET(SW_IND_CTXT_DATA_W4_INT_AGGR_MASK, ctxt->intr_aggr) |
        FIELD_SET(SW_IND_CTXT_DATA_W4_DIS_INTR_ON_VF_MASK, ctxt->dis_intr_on_vf) |
        FIELD_SET(SW_IND_CTXT_DATA_W4_VIRTIO_EN_MASK, ctxt->virtio_en) |
        FIELD_SET(SW_IND_CTXT_DATA_W4_PACK_BYP_OUT_MASK, ctxt->pack_byp_out) |
        FIELD_SET(SW_IND_CTXT_DATA_W4_IRQ_BYP_MASK, ctxt->irq_byp) |
        FIELD_SET(SW_IND_CTXT_DATA_W4_HOST_ID_MASK, ctxt->host_id) |
        FIELD_SET(SW_IND_CTXT_DATA_W4_PASID_L_MASK, pasid_l);

    sw_ctxt[num_words_count++] =
        FIELD_SET(SW_IND_CTXT_DATA_W5_PASID_H_MASK, pasid_h) |
        FIELD_SET(SW_IND_CTXT_DATA_W5_PASID_EN_MASK, ctxt->pasid_en) |
        FIELD_SET(SW_IND_CTXT_DATA_W5_VIRTIO_DSC_BASE_L_MASK, virtio_desc_base_l);

    sw_ctxt[num_words_count++] =
        FIELD_SET(SW_IND_CTXT_DATA_W6_VIRTIO_DSC_BASE_M_MASK, virtio_desc_base_m);

    sw_ctxt[num_words_count++] =
        FIELD_SET(SW_IND_CTXT_DATA_W7_VIRTIO_DSC_BASE_H_MASK, virtio_desc_base_h);

    qdma_log_debug("%s: vec=%x, intr_aggr=%x\n",
                   __func__, ctxt->vec, ctxt->intr_aggr);

    /* MD: Write the software context to the hardware */
    return eqdma_cpm5_indirect_reg_write(dev_hndl, sel, hw_qid,
                                         sw_ctxt, num_words_count);
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_sw_context_read() - Read software context from the hardware.
 * @dev_hndl: Device handle used for register access.
 * @c2h: Flag indicating if the queue is C2H (1 for C2H, 0 for H2C).
 * @hw_qid: Hardware queue ID of the queue.
 * @ctxt: Pointer to the output context data structure to be filled.
 *
 * Return: 0 on success and < 0 on failure.
 *****************************************************************************/
static int eqdma_cpm5_sw_context_read(void *dev_hndl, uint8_t c2h,
                                      uint16_t hw_qid,
                                      struct qdma_descq_sw_ctxt *ctxt)
{
    int rv = QDMA_SUCCESS;
    uint32_t sw_ctxt[EQDMA_CPM5_SW_CONTEXT_NUM_WORDS] = {0};
    uint32_t pasid_l, pasid_h;
    uint32_t virtio_desc_base_l, virtio_desc_base_m, virtio_desc_base_h;
    enum ind_ctxt_cmd_sel sel = c2h ?
                                QDMA_CTXT_SEL_SW_C2H : QDMA_CTXT_SEL_SW_H2C;

    /* MD: Input arguments check */
    if (!dev_hndl || !ctxt) {
        qdma_log_error("%s: dev_handle=%p sw_ctxt=%p NULL, err:%d\n",
                       __func__, dev_hndl, ctxt,
                       -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    /* MD: Read the software context from the hardware */
    rv = eqdma_cpm5_indirect_reg_read(dev_hndl, sel, hw_qid,
                                      EQDMA_CPM5_SW_CONTEXT_NUM_WORDS, sw_ctxt);
    if (rv < 0)
        return rv;

    /* MD: Extract fields from the context data */
    ctxt->pidx = FIELD_GET(SW_IND_CTXT_DATA_W0_PIDX_MASK, sw_ctxt[0]);
    ctxt->irq_arm = (uint8_t)(FIELD_GET(SW_IND_CTXT_DATA_W0_IRQ_ARM_MASK, sw_ctxt[0]));
    ctxt->fnc_id = FIELD_GET(SW_IND_CTXT_DATA_W0_FNC_MASK, sw_ctxt[0]);

    qdma_log_debug("%s: pidx=%x, irq_arm=%x, fnc_id=%x",
                   __func__, ctxt->pidx, ctxt->irq_arm, ctxt->fnc_id);

    ctxt->qen = FIELD_GET(SW_IND_CTXT_DATA_W1_QEN_MASK, sw_ctxt[1]);
    ctxt->frcd_en = FIELD_GET(SW_IND_CTXT_DATA_W1_FCRD_EN_MASK, sw_ctxt[1]);
    ctxt->wbi_chk = FIELD_GET(SW_IND_CTXT_DATA_W1_WBI_CHK_MASK, sw_ctxt[1]);
    ctxt->wbi_intvl_en = FIELD_GET(SW_IND_CTXT_DATA_W1_WBI_INTVL_EN_MASK, sw_ctxt[1]);
    ctxt->at = FIELD_GET(SW_IND_CTXT_DATA_W1_AT_MASK, sw_ctxt[1]);
    ctxt->fetch_max = (uint8_t)FIELD_GET(SW_IND_CTXT_DATA_W1_FETCH_MAX_MASK, sw_ctxt[1]);
    ctxt->rngsz_idx = (uint8_t)(FIELD_GET(SW_IND_CTXT_DATA_W1_RNG_SZ_MASK, sw_ctxt[1]));
    ctxt->desc_sz = (uint8_t)(FIELD_GET(SW_IND_CTXT_DATA_W1_DSC_SZ_MASK, sw_ctxt[1]));
    ctxt->bypass = (uint8_t)(FIELD_GET(SW_IND_CTXT_DATA_W1_BYPASS_MASK, sw_ctxt[1]));
    ctxt->mm_chn = (uint8_t)(FIELD_GET(SW_IND_CTXT_DATA_W1_MM_CHN_MASK, sw_ctxt[1]));
    ctxt->wbk_en = (uint8_t)(FIELD_GET(SW_IND_CTXT_DATA_W1_WBK_EN_MASK, sw_ctxt[1]));
    ctxt->irq_en = (uint8_t)(FIELD_GET(SW_IND_CTXT_DATA_W1_IRQ_EN_MASK, sw_ctxt[1]));
    ctxt->port_id = (uint8_t)(FIELD_GET(SW_IND_CTXT_DATA_W1_PORT_ID_MASK, sw_ctxt[1]));
    ctxt->irq_no_last = (uint8_t)(FIELD_GET(SW_IND_CTXT_DATA_W1_IRQ_NO_LAST_MASK, sw_ctxt[1]));
    ctxt->err = (uint8_t)(FIELD_GET(SW_IND_CTXT_DATA_W1_ERR_MASK, sw_ctxt[1]));
    ctxt->err_wb_sent = (uint8_t)(FIELD_GET(SW_IND_CTXT_DATA_W1_ERR_WB_SENT_MASK, sw_ctxt[1]));
    ctxt->irq_req = (uint8_t)(FIELD_GET(SW_IND_CTXT_DATA_W1_IRQ_REQ_MASK, sw_ctxt[1]));
    ctxt->mrkr_dis = (uint8_t)(FIELD_GET(SW_IND_CTXT_DATA_W1_MRKR_DIS_MASK, sw_ctxt[1]));
    ctxt->is_mm = (uint8_t)(FIELD_GET(SW_IND_CTXT_DATA_W1_IS_MM_MASK, sw_ctxt[1]));

    qdma_log_debug("%s: qen=%x, frcd_en=%x, wbi_chk=%x, wbi_intvl_en=%x\n",
                   __func__, ctxt->qen, ctxt->frcd_en, ctxt->wbi_chk,
                   ctxt->wbi_intvl_en);
    qdma_log_debug("%s: at=%x, fetch_max=%x, rngsz_idx=%x, desc_sz=%x\n",
                   __func__, ctxt->at, ctxt->fetch_max, ctxt->rngsz_idx,
                   ctxt->desc_sz);
    qdma_log_debug("%s: bypass=%x, mm_chn=%x, wbk_en=%x, irq_en=%x\n",
                   __func__, ctxt->bypass, ctxt->mm_chn, ctxt->wbk_en,
                   ctxt->irq_en);
    qdma_log_debug("%s: port_id=%x, irq_no_last=%x,",
                   __func__, ctxt->port_id, ctxt->irq_no_last);
    qdma_log_debug(" err=%x, err_wb_sent=%x\n",
                   ctxt->err, ctxt->err_wb_sent);
    qdma_log_debug("%s: irq_req=%x, mrkr_dis=%x, is_mm=%x\n",
                   __func__, ctxt->irq_req, ctxt->mrkr_dis, ctxt->is_mm);

    /* MD: Set ring base address */
    ctxt->ring_bs_addr = ((uint64_t)sw_ctxt[3] << 32) | (sw_ctxt[2]);

    /* MD: Extract additional fields */
    ctxt->vec = FIELD_GET(SW_IND_CTXT_DATA_W4_VEC_MASK, sw_ctxt[4]);
    ctxt->intr_aggr = (uint8_t)(FIELD_GET(SW_IND_CTXT_DATA_W4_INT_AGGR_MASK, sw_ctxt[4]));
    ctxt->dis_intr_on_vf = (uint8_t)(FIELD_GET(SW_IND_CTXT_DATA_W4_DIS_INTR_ON_VF_MASK, sw_ctxt[4]));
    ctxt->virtio_en = (uint8_t)(FIELD_GET(SW_IND_CTXT_DATA_W4_VIRTIO_EN_MASK, sw_ctxt[4]));
    ctxt->pack_byp_out = (uint8_t)(FIELD_GET(SW_IND_CTXT_DATA_W4_PACK_BYP_OUT_MASK, sw_ctxt[4]));
    ctxt->irq_byp = (uint8_t)(FIELD_GET(SW_IND_CTXT_DATA_W4_IRQ_BYP_MASK, sw_ctxt[4]));
    ctxt->host_id = (uint8_t)(FIELD_GET(SW_IND_CTXT_DATA_W4_HOST_ID_MASK, sw_ctxt[4]));
    pasid_l = FIELD_GET(SW_IND_CTXT_DATA_W4_PASID_L_MASK, sw_ctxt[4]);

    pasid_h = FIELD_GET(SW_IND_CTXT_DATA_W5_PASID_H_MASK, sw_ctxt[5]);
    ctxt->pasid_en = (uint8_t)FIELD_GET(SW_IND_CTXT_DATA_W5_PASID_EN_MASK, sw_ctxt[5]);
    virtio_desc_base_l = FIELD_GET(SW_IND_CTXT_DATA_W5_VIRTIO_DSC_BASE_L_MASK, sw_ctxt[5]);
    virtio_desc_base_m = FIELD_GET(SW_IND_CTXT_DATA_W6_VIRTIO_DSC_BASE_M_MASK, sw_ctxt[6]);
    virtio_desc_base_h = FIELD_GET(SW_IND_CTXT_DATA_W7_VIRTIO_DSC_BASE_H_MASK, sw_ctxt[6]);

    /* MD: Set PASID and Virtio descriptor base fields */
    ctxt->pasid = FIELD_SET(EQDMA_CPM5_SW_CTXT_PASID_GET_L_MASK, pasid_l) |
                  FIELD_SET(EQDMA_CPM5_SW_CTXT_PASID_GET_H_MASK, pasid_h);

    ctxt->virtio_dsc_base = FIELD_SET(EQDMA_CPM5_SW_CTXT_VIRTIO_DSC_BASE_GET_L_MASK, (uint64_t)virtio_desc_base_l) |
                            FIELD_SET(EQDMA_CPM5_SW_CTXT_VIRTIO_DSC_BASE_GET_M_MASK, (uint64_t)virtio_desc_base_m) |
                            FIELD_SET(EQDMA_CPM5_SW_CTXT_VIRTIO_DSC_BASE_GET_H_MASK, (uint64_t)virtio_desc_base_h);

    qdma_log_debug("%s: vec=%x, intr_aggr=%x\n",
                   __func__, ctxt->vec, ctxt->intr_aggr);

    return QDMA_SUCCESS;
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_sw_context_clear() - Clear software context in the hardware.
 * @dev_hndl: Device handle used for register access.
 * @c2h: Flag indicating if the queue is C2H (1 for C2H, 0 for H2C).
 * @hw_qid: Hardware queue ID of the queue.
 *
 * Return: 0 on success and < 0 on failure.
 *****************************************************************************/
static int eqdma_cpm5_sw_context_clear(void *dev_hndl, uint8_t c2h,
                                       uint16_t hw_qid)
{
    enum ind_ctxt_cmd_sel sel = c2h ?
                                QDMA_CTXT_SEL_SW_C2H : QDMA_CTXT_SEL_SW_H2C;

    /* MD: Input argument check */
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n", __func__,
                       -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    /* MD: Clear the software context */
    return eqdma_cpm5_indirect_reg_clear(dev_hndl, sel, hw_qid);
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_sw_context_invalidate() - Invalidate software context in the hardware.
 * @dev_hndl: Device handle used for register access.
 * @c2h: Flag indicating if the queue is C2H (1 for C2H, 0 for H2C).
 * @hw_qid: Hardware queue ID of the queue.
 *
 * Return: 0 on success and < 0 on failure.
 *****************************************************************************/
static int eqdma_cpm5_sw_context_invalidate(void *dev_hndl, uint8_t c2h,
                                            uint16_t hw_qid)
{
    enum ind_ctxt_cmd_sel sel = c2h ?
                                QDMA_CTXT_SEL_SW_C2H : QDMA_CTXT_SEL_SW_H2C;

    /* MD: Input argument check */
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n", __func__,
                       -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    /* MD: Invalidate the software context */
    return eqdma_cpm5_indirect_reg_invalidate(dev_hndl, sel, hw_qid);
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_sw_ctx_conf() - Configure software context based on access type.
 * @dev_hndl: Device handle used for register access.
 * @c2h: Flag indicating if the queue is C2H (1 for C2H, 0 for H2C).
 * @hw_qid: Hardware queue ID of the queue.
 * @ctxt: Pointer to the context data structure.
 * @access_type: HW access type (qdma_hw_access_type enum) value.
 *
 * Return: 0 on success and < 0 on failure.
 *****************************************************************************/
int eqdma_cpm5_sw_ctx_conf(void *dev_hndl, uint8_t c2h, uint16_t hw_qid,
                           struct qdma_descq_sw_ctxt *ctxt,
                           enum qdma_hw_access_type access_type)
{
    int rv = QDMA_SUCCESS;

    /* MD: Configure the software context based on the access type */
    switch (access_type) {
    case QDMA_HW_ACCESS_READ:
        rv = eqdma_cpm5_sw_context_read(dev_hndl, c2h, hw_qid, ctxt);
        break;
    case QDMA_HW_ACCESS_WRITE:
        rv = eqdma_cpm5_sw_context_write(dev_hndl, c2h, hw_qid, ctxt);
        break;
    case QDMA_HW_ACCESS_CLEAR:
        rv = eqdma_cpm5_sw_context_clear(dev_hndl, c2h, hw_qid);
        break;
    case QDMA_HW_ACCESS_INVALIDATE:
        rv = eqdma_cpm5_sw_context_invalidate(dev_hndl, c2h, hw_qid);
        break;
    default:
        qdma_log_error("%s: access_type(%d) invalid, err:%d\n",
                       __func__, access_type, -QDMA_ERR_INV_PARAM);
        rv = -QDMA_ERR_INV_PARAM;
        break;
    }

    return rv;
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_pfetch_context_write() - Create prefetch context and program it into the hardware.
 * @dev_hndl: Device handle used for register access.
 * @hw_qid: Hardware queue ID of the queue.
 * @ctxt: Pointer to the prefetch context data structure.
 *
 * Return: 0 on success and < 0 on failure.
 *****************************************************************************/
static int eqdma_cpm5_pfetch_context_write(void *dev_hndl, uint16_t hw_qid,
                                           const struct qdma_descq_prefetch_ctxt *ctxt)
{
    uint32_t pfetch_ctxt[EQDMA_CPM5_PFETCH_CONTEXT_NUM_WORDS] = {0};
    enum ind_ctxt_cmd_sel sel = QDMA_CTXT_SEL_PFTCH;
    uint32_t sw_crdt_l, sw_crdt_h;
    uint16_t num_words_count = 0;

    /* MD: Input argument check */
    if (!dev_hndl || !ctxt) {
        qdma_log_error("%s: dev_handle or pfetch ctxt NULL, err:%d\n",
                       __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    /* MD: Extract software credit fields */
    sw_crdt_l = FIELD_GET(QDMA_PFTCH_CTXT_SW_CRDT_GET_L_MASK, ctxt->sw_crdt);
    sw_crdt_h = FIELD_GET(QDMA_PFTCH_CTXT_SW_CRDT_GET_H_MASK, ctxt->sw_crdt);

    qdma_log_debug("%s: sw_crdt_l=%u, sw_crdt_h=%u, hw_qid=%hu\n",
                   __func__, sw_crdt_l, sw_crdt_h, hw_qid);

    /* MD: Populate the prefetch context array */
    pfetch_ctxt[num_words_count++] =
        FIELD_SET(PREFETCH_CTXT_DATA_W0_BYPASS_MASK, ctxt->bypass) |
        FIELD_SET(PREFETCH_CTXT_DATA_W0_BUF_SZ_IDX_MASK, ctxt->bufsz_idx) |
        FIELD_SET(PREFETCH_CTXT_DATA_W0_PORT_ID_MASK, ctxt->port_id) |
        FIELD_SET(PREFETCH_CTXT_DATA_W0_NUM_PFCH_MASK, ctxt->num_pftch) |
        FIELD_SET(PREFETCH_CTXT_DATA_W0_VAR_DESC_MASK, ctxt->var_desc) |
        FIELD_SET(PREFETCH_CTXT_DATA_W0_ERR_MASK, ctxt->err) |
        FIELD_SET(PREFETCH_CTXT_DATA_W0_PFCH_EN_MASK, ctxt->pfch_en) |
        FIELD_SET(PREFETCH_CTXT_DATA_W0_PFCH_MASK, ctxt->pfch) |
        FIELD_SET(PREFETCH_CTXT_DATA_W0_SW_CRDT_L_MASK, sw_crdt_l);

    qdma_log_debug("%s: bypass=%x, bufsz_idx=%x, port_id=%x\n",
                   __func__, ctxt->bypass, ctxt->bufsz_idx, ctxt->port_id);
    qdma_log_debug("%s: err=%x, pfch_en=%x, pfch=%x, ctxt->valid=%x\n",
                   __func__, ctxt->err, ctxt->pfch_en, ctxt->pfch, ctxt->valid);

    pfetch_ctxt[num_words_count++] =
        FIELD_SET(PREFETCH_CTXT_DATA_W1_SW_CRDT_H_MASK, sw_crdt_h) |
        FIELD_SET(PREFETCH_CTXT_DATA_W1_VALID_MASK, ctxt->valid);

    /* MD: Write the prefetch context to the hardware */
    return eqdma_cpm5_indirect_reg_write(dev_hndl, sel, hw_qid,
                                         pfetch_ctxt, num_words_count);
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_pfetch_context_read() - Read prefetch context from the hardware.
 * @dev_hndl: Device handle used for register access.
 * @hw_qid: Hardware queue ID of the queue.
 * @ctxt: Pointer to the output context data structure to be filled.
 *
 * Return: 0 on success and < 0 on failure.
 *****************************************************************************/
static int eqdma_cpm5_pfetch_context_read(void *dev_hndl, uint16_t hw_qid,
                                          struct qdma_descq_prefetch_ctxt *ctxt)
{
    int rv = QDMA_SUCCESS;
    uint32_t pfetch_ctxt[EQDMA_CPM5_PFETCH_CONTEXT_NUM_WORDS] = {0};
    enum ind_ctxt_cmd_sel sel = QDMA_CTXT_SEL_PFTCH;
    uint32_t sw_crdt_l, sw_crdt_h;

    /* MD: Input arguments check */
    if (!dev_hndl || !ctxt) {
        qdma_log_error("%s: dev_handle or pfetch ctxt NULL, err:%d\n",
                       __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    /* MD: Read the prefetch context from the hardware */
    rv = eqdma_cpm5_indirect_reg_read(dev_hndl, sel, hw_qid,
                                      EQDMA_CPM5_PFETCH_CONTEXT_NUM_WORDS, pfetch_ctxt);
    if (rv < 0)
        return rv;

    /* MD: Extract fields from the context data */
    ctxt->bypass = FIELD_GET(PREFETCH_CTXT_DATA_W0_BYPASS_MASK, pfetch_ctxt[0]);
    ctxt->bufsz_idx = FIELD_GET(PREFETCH_CTXT_DATA_W0_BUF_SZ_IDX_MASK, pfetch_ctxt[0]);
    ctxt->num_pftch = (uint16_t)FIELD_GET(PREFETCH_CTXT_DATA_W0_NUM_PFCH_MASK, pfetch_ctxt[0]);
    ctxt->port_id = FIELD_GET(PREFETCH_CTXT_DATA_W0_PORT_ID_MASK, pfetch_ctxt[0]);
    ctxt->var_desc = (uint8_t)FIELD_GET(PREFETCH_CTXT_DATA_W0_VAR_DESC_MASK, pfetch_ctxt[0]);
    ctxt->err = (uint8_t)(FIELD_GET(PREFETCH_CTXT_DATA_W0_ERR_MASK, pfetch_ctxt[0]));
    ctxt->pfch_en = (uint8_t)(FIELD_GET(PREFETCH_CTXT_DATA_W0_PFCH_EN_MASK, pfetch_ctxt[0]));
    ctxt->pfch = (uint8_t)(FIELD_GET(PREFETCH_CTXT_DATA_W0_PFCH_MASK, pfetch_ctxt[0]));
    sw_crdt_l = FIELD_GET(PREFETCH_CTXT_DATA_W0_SW_CRDT_L_MASK, pfetch_ctxt[0]);

    sw_crdt_h = FIELD_GET(PREFETCH_CTXT_DATA_W1_SW_CRDT_H_MASK, pfetch_ctxt[1]);
    ctxt->valid = (uint8_t)(FIELD_GET(PREFETCH_CTXT_DATA_W1_VALID_MASK, pfetch_ctxt[1]));

    ctxt->sw_crdt = FIELD_SET(QDMA_PFTCH_CTXT_SW_CRDT_GET_L_MASK, sw_crdt_l) |
                    FIELD_SET(QDMA_PFTCH_CTXT_SW_CRDT_GET_H_MASK, sw_crdt_h);

    qdma_log_debug("%s: sw_crdt_l=%u, sw_crdt_h=%u, hw_qid=%hu\n",
                   __func__, sw_crdt_l, sw_crdt_h, hw_qid);
    qdma_log_debug("%s: bypass=%x, bufsz_idx=%x, port_id=%x\n",
                   __func__, ctxt->bypass, ctxt->bufsz_idx, ctxt->port_id);
    qdma_log_debug("%s: err=%x, pfch_en=%x, pfch=%x, ctxt->valid=%x\n",
                   __func__, ctxt->err, ctxt->pfch_en, ctxt->pfch, ctxt->valid);

    return QDMA_SUCCESS;
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_pfetch_context_clear() - Clear prefetch context in the hardware.
 * @dev_hndl: Device handle used for register access.
 * @hw_qid: Hardware queue ID of the queue.
 *
 * Return: 0 on success and < 0 on failure.
 *****************************************************************************/
static int eqdma_cpm5_pfetch_context_clear(void *dev_hndl, uint16_t hw_qid)
{
    enum ind_ctxt_cmd_sel sel = QDMA_CTXT_SEL_PFTCH;

    /* MD: Input argument check */
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n", __func__,
                       -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    /* MD: Clear the prefetch context */
    return eqdma_cpm5_indirect_reg_clear(dev_hndl, sel, hw_qid);
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_pfetch_context_invalidate() - Invalidate prefetch context in the hardware.
 * @dev_hndl: Device handle used for register access.
 * @hw_qid: Hardware queue ID of the queue.
 *
 * Return: 0 on success and < 0 on failure.
 *****************************************************************************/
static int eqdma_cpm5_pfetch_context_invalidate(void *dev_hndl, uint16_t hw_qid)
{
    enum ind_ctxt_cmd_sel sel = QDMA_CTXT_SEL_PFTCH;

    /* MD: Input argument check */
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n", __func__,
                       -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    /* MD: Invalidate the prefetch context */
    return eqdma_cpm5_indirect_reg_invalidate(dev_hndl, sel, hw_qid);
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_pfetch_ctx_conf() - Configure prefetch context based on access type.
 * @dev_hndl: Device handle used for register access.
 * @hw_qid: Hardware queue ID of the queue.
 * @ctxt: Pointer to the context data structure.
 * @access_type: HW access type (qdma_hw_access_type enum) value.
 *
 * Return: 0 on success and < 0 on failure.
 *****************************************************************************/
int eqdma_cpm5_pfetch_ctx_conf(void *dev_hndl, uint16_t hw_qid,
                               struct qdma_descq_prefetch_ctxt *ctxt,
                               enum qdma_hw_access_type access_type)
{
    int rv = QDMA_SUCCESS;

    /* MD: Configure the prefetch context based on the access type */
    switch (access_type) {
    case QDMA_HW_ACCESS_READ:
        rv = eqdma_cpm5_pfetch_context_read(dev_hndl, hw_qid, ctxt);
        break;
    case QDMA_HW_ACCESS_WRITE:
        rv = eqdma_cpm5_pfetch_context_write(dev_hndl, hw_qid, ctxt);
        break;
    case QDMA_HW_ACCESS_CLEAR:
        rv = eqdma_cpm5_pfetch_context_clear(dev_hndl, hw_qid);
        break;
    case QDMA_HW_ACCESS_INVALIDATE:
        rv = eqdma_cpm5_pfetch_context_invalidate(dev_hndl, hw_qid);
        break;
    default:
        qdma_log_error("%s: access_type(%d) invalid, err:%d\n",
                       __func__, access_type, -QDMA_ERR_INV_PARAM);
        rv = -QDMA_ERR_INV_PARAM;
        break;
    }

    return rv;
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_cmpt_context_write() - Create completion context and program it into the hardware.
 * @dev_hndl: Device handle used for register access.
 * @hw_qid: Hardware queue ID of the queue.
 * @ctxt: Pointer to the completion context data structure.
 *
 * Return: 0 on success and < 0 on failure.
 *****************************************************************************/
static int eqdma_cpm5_cmpt_context_write(void *dev_hndl, uint16_t hw_qid,
                                         const struct qdma_descq_cmpt_ctxt *ctxt)
{
    uint32_t cmpt_ctxt[EQDMA_CPM5_CMPT_CONTEXT_NUM_WORDS] = {0};
    uint16_t num_words_count = 0;
    uint32_t baddr4_high_l, baddr4_high_h, baddr4_low, pidx_l, pidx_h, pasid_l, pasid_h;
    enum ind_ctxt_cmd_sel sel = QDMA_CTXT_SEL_CMPT;

    /* MD: Input arguments check */
    if (!dev_hndl || !ctxt) {
        qdma_log_error("%s: dev_handle or cmpt ctxt NULL, err:%d\n",
                       __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    /* MD: Validate trigger mode */
    if (ctxt->trig_mode > QDMA_CMPT_UPDATE_TRIG_MODE_TMR_CNTR) {
        qdma_log_error("%s: trig_mode(%d) > (%d) is invalid, err:%d\n",
                       __func__, ctxt->trig_mode,
                       QDMA_CMPT_UPDATE_TRIG_MODE_TMR_CNTR,
                       -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    /* MD: Extract address and PASID fields */
    baddr4_high_l = (uint32_t)FIELD_GET(EQDMA_CPM5_COMPL_CTXT_BADDR_HIGH_L_MASK, ctxt->bs_addr);
    baddr4_high_h = (uint32_t)FIELD_GET(EQDMA_CPM5_COMPL_CTXT_BADDR_HIGH_H_MASK, ctxt->bs_addr);
    baddr4_low = (uint32_t)FIELD_GET(EQDMA_CPM5_COMPL_CTXT_BADDR_LOW_MASK, ctxt->bs_addr);

    pidx_l = FIELD_GET(QDMA_COMPL_CTXT_PIDX_GET_L_MASK, ctxt->pidx);
    pidx_h = FIELD_GET(QDMA_COMPL_CTXT_PIDX_GET_H_MASK, ctxt->pidx);

    pasid_l = FIELD_GET(EQDMA_CPM5_CMPL_CTXT_PASID_GET_L_MASK, ctxt->pasid);
    pasid_h = FIELD_GET(EQDMA_CPM5_CMPL_CTXT_PASID_GET_H_MASK, ctxt->pasid);

    /* MD: Populate the completion context array */
    cmpt_ctxt[num_words_count++] =
        FIELD_SET(CMPL_CTXT_DATA_W0_EN_STAT_DESC_MASK, ctxt->en_stat_desc) |
        FIELD_SET(CMPL_CTXT_DATA_W0_EN_INT_MASK, ctxt->en_int) |
        FIELD_SET(CMPL_CTXT_DATA_W0_TRIG_MODE_MASK, ctxt->trig_mode) |
        FIELD_SET(CMPL_CTXT_DATA_W0_FNC_ID_MASK, ctxt->fnc_id) |
        FIELD_SET(CMPL_CTXT_DATA_W0_CNTER_IX_MASK, ctxt->counter_idx) |
        FIELD_SET(CMPL_CTXT_DATA_W0_TIMER_IX_MASK, ctxt->timer_idx) |
        FIELD_SET(CMPL_CTXT_DATA_W0_INT_ST_MASK, ctxt->in_st) |
        FIELD_SET(CMPL_CTXT_DATA_W0_COLOR_MASK, ctxt->color) |
        FIELD_SET(CMPL_CTXT_DATA_W0_QSIZE_IX_MASK, ctxt->ringsz_idx);

    cmpt_ctxt[num_words_count++] =
        FIELD_SET(CMPL_CTXT_DATA_W1_BADDR4_HIGH_L_MASK, baddr4_high_l);

    cmpt_ctxt[num_words_count++] =
        FIELD_SET(CMPL_CTXT_DATA_W2_BADDR4_HIGH_H_MASK, baddr4_high_h) |
        FIELD_SET(CMPL_CTXT_DATA_W2_DESC_SIZE_MASK, ctxt->desc_sz) |
        FIELD_SET(CMPL_CTXT_DATA_W2_PIDX_L_MASK, pidx_l);

    cmpt_ctxt[num_words_count++] =
        FIELD_SET(CMPL_CTXT_DATA_W3_PIDX_H_MASK, pidx_h) |
        FIELD_SET(CMPL_CTXT_DATA_W3_CIDX_MASK, ctxt->cidx) |
        FIELD_SET(CMPL_CTXT_DATA_W3_VALID_MASK, ctxt->valid) |
        FIELD_SET(CMPL_CTXT_DATA_W3_ERR_MASK, ctxt->err) |
        FIELD_SET(CMPL_CTXT_DATA_W3_USER_TRIG_PEND_MASK, ctxt->user_trig_pend);

    cmpt_ctxt[num_words_count++] =
        FIELD_SET(CMPL_CTXT_DATA_W4_TIMER_RUNNING_MASK, ctxt->timer_running) |
        FIELD_SET(CMPL_CTXT_DATA_W4_FULL_UPD_MASK, ctxt->full_upd) |
        FIELD_SET(CMPL_CTXT_DATA_W4_OVF_CHK_DIS_MASK, ctxt->ovf_chk_dis) |
        FIELD_SET(CMPL_CTXT_DATA_W4_AT_MASK, ctxt->at) |
        FIELD_SET(CMPL_CTXT_DATA_W4_VEC_MASK, ctxt->vec) |
        FIELD_SET(CMPL_CTXT_DATA_W4_INT_AGGR_MASK, ctxt->int_aggr) |
        FIELD_SET(CMPL_CTXT_DATA_W4_DIS_INTR_ON_VF_MASK, ctxt->dis_intr_on_vf) |
        FIELD_SET(CMPL_CTXT_DATA_W4_VIO_MASK, ctxt->vio) |
        FIELD_SET(CMPL_CTXT_DATA_W4_DIR_C2H_MASK, ctxt->dir_c2h) |
        FIELD_SET(CMPL_CTXT_DATA_W4_HOST_ID_MASK, ctxt->host_id) |
        FIELD_SET(CMPL_CTXT_DATA_W4_PASID_L_MASK, pasid_l);

    cmpt_ctxt[num_words_count++] =
        FIELD_SET(CMPL_CTXT_DATA_W5_PASID_H_MASK, pasid_h) |
        FIELD_SET(CMPL_CTXT_DATA_W5_PASID_EN_MASK, ctxt->pasid_en) |
        FIELD_SET(CMPL_CTXT_DATA_W5_BADDR4_LOW_MASK, baddr4_low) |
        FIELD_SET(CMPL_CTXT_DATA_W5_VIO_EOP_MASK, ctxt->vio_eop) |
        FIELD_SET(CMPL_CTXT_DATA_W5_SH_CMPT_MASK, ctxt->sh_cmpt);

    /* MD: Write the completion context to the hardware */
    return eqdma_cpm5_indirect_reg_write(dev_hndl, sel, hw_qid,
                                         cmpt_ctxt, num_words_count);
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_cmpt_context_read() - Read completion context from the hardware.
 * @dev_hndl: Device handle used for register access.
 * @hw_qid: Hardware queue ID of the queue.
 * @ctxt: Pointer to the context data structure to be filled.
 *
 * Return: 0 on success and < 0 on failure.
 *****************************************************************************/
static int eqdma_cpm5_cmpt_context_read(void *dev_hndl, uint16_t hw_qid,
                                        struct qdma_descq_cmpt_ctxt *ctxt)
{
    int rv = QDMA_SUCCESS;
    uint32_t cmpt_ctxt[EQDMA_CPM5_CMPT_CONTEXT_NUM_WORDS] = {0};
    enum ind_ctxt_cmd_sel sel = QDMA_CTXT_SEL_CMPT;
    uint32_t baddr4_high_l, baddr4_high_h, baddr4_low, pidx_l, pidx_h, pasid_l, pasid_h;

    /* MD: Input arguments check */
    if (!dev_hndl || !ctxt) {
        qdma_log_error("%s: dev_handle or cmpt ctxt NULL, err:%d\n",
                       __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    /* MD: Read the completion context from the hardware */
    rv = eqdma_cpm5_indirect_reg_read(dev_hndl, sel, hw_qid,
                                      EQDMA_CPM5_CMPT_CONTEXT_NUM_WORDS, cmpt_ctxt);
    if (rv < 0)
        return rv;

    /* MD: Extract fields from the context data */
    ctxt->en_stat_desc = FIELD_GET(CMPL_CTXT_DATA_W0_EN_STAT_DESC_MASK, cmpt_ctxt[0]);
    ctxt->en_int = FIELD_GET(CMPL_CTXT_DATA_W0_EN_INT_MASK, cmpt_ctxt[0]);
    ctxt->trig_mode = FIELD_GET(CMPL_CTXT_DATA_W0_TRIG_MODE_MASK, cmpt_ctxt[0]);
    ctxt->fnc_id = FIELD_GET(CMPL_CTXT_DATA_W0_FNC_ID_MASK, cmpt_ctxt[0]);
    ctxt->counter_idx = (uint8_t)(FIELD_GET(CMPL_CTXT_DATA_W0_CNTER_IX_MASK, cmpt_ctxt[0]));
    ctxt->timer_idx = (uint8_t)(FIELD_GET(CMPL_CTXT_DATA_W0_TIMER_IX_MASK, cmpt_ctxt[0]));
    ctxt->in_st = (uint8_t)(FIELD_GET(CMPL_CTXT_DATA_W0_INT_ST_MASK, cmpt_ctxt[0]));
    ctxt->color = (uint8_t)(FIELD_GET(CMPL_CTXT_DATA_W0_COLOR_MASK, cmpt_ctxt[0]));
    ctxt->ringsz_idx = (uint8_t)(FIELD_GET(CMPL_CTXT_DATA_W0_QSIZE_IX_MASK, cmpt_ctxt[0]));

    baddr4_high_l = FIELD_GET(CMPL_CTXT_DATA_W1_BADDR4_HIGH_L_MASK, cmpt_ctxt[1]);

    baddr4_high_h = FIELD_GET(CMPL_CTXT_DATA_W2_BADDR4_HIGH_H_MASK, cmpt_ctxt[2]);
    ctxt->desc_sz = (uint8_t)(FIELD_GET(CMPL_CTXT_DATA_W2_DESC_SIZE_MASK, cmpt_ctxt[2]));
    pidx_l = FIELD_GET(CMPL_CTXT_DATA_W2_PIDX_L_MASK, cmpt_ctxt[2]);

    pidx_h = FIELD_GET(CMPL_CTXT_DATA_W3_PIDX_H_MASK, cmpt_ctxt[3]);
    ctxt->cidx = (uint16_t)(FIELD_GET(CMPL_CTXT_DATA_W3_CIDX_MASK, cmpt_ctxt[3]));
    ctxt->valid = (uint8_t)(FIELD_GET(CMPL_CTXT_DATA_W3_VALID_MASK, cmpt_ctxt[3]));
    ctxt->err = (uint8_t)(FIELD_GET(CMPL_CTXT_DATA_W3_ERR_MASK, cmpt_ctxt[3]));
    ctxt->user_trig_pend = (uint8_t)(FIELD_GET(CMPL_CTXT_DATA_W3_USER_TRIG_PEND_MASK, cmpt_ctxt[3]));

    ctxt->timer_running = FIELD_GET(CMPL_CTXT_DATA_W4_TIMER_RUNNING_MASK, cmpt_ctxt[4]);
    ctxt->full_upd = FIELD_GET(CMPL_CTXT_DATA_W4_FULL_UPD_MASK, cmpt_ctxt[4]);
    ctxt->ovf_chk_dis = FIELD_GET(CMPL_CTXT_DATA_W4_OVF_CHK_DIS_MASK, cmpt_ctxt[4]);
    ctxt->at = FIELD_GET(CMPL_CTXT_DATA_W4_AT_MASK, cmpt_ctxt[4]);
    ctxt->vec = FIELD_GET(CMPL_CTXT_DATA_W4_VEC_MASK, cmpt_ctxt[4]);
    ctxt->int_aggr = (uint8_t)(FIELD_GET(CMPL_CTXT_DATA_W4_INT_AGGR_MASK, cmpt_ctxt[4]));
    ctxt->dis_intr_on_vf = (uint8_t)FIELD_GET(CMPL_CTXT_DATA_W4_DIS_INTR_ON_VF_MASK, cmpt_ctxt[4]);
    ctxt->vio = (uint8_t)FIELD_GET(CMPL_CTXT_DATA_W4_VIO_MASK, cmpt_ctxt[4]);
    ctxt->dir_c2h = (uint8_t)FIELD_GET(CMPL_CTXT_DATA_W4_DIR_C2H_MASK, cmpt_ctxt[4]);
    ctxt->host_id = (uint8_t)FIELD_GET(CMPL_CTXT_DATA_W4_HOST_ID_MASK, cmpt_ctxt[4]);
    pasid_l = FIELD_GET(CMPL_CTXT_DATA_W4_PASID_L_MASK, cmpt_ctxt[4]);

    pasid_h = (uint32_t)FIELD_GET(CMPL_CTXT_DATA_W5_PASID_H_MASK, cmpt_ctxt[5]);
    ctxt->pasid_en = (uint8_t)FIELD_GET(CMPL_CTXT_DATA_W5_PASID_EN_MASK, cmpt_ctxt[5]);
    baddr4_low = (uint8_t)FIELD_GET(CMPL_CTXT_DATA_W5_BADDR4_LOW_MASK, cmpt_ctxt[5]);
    ctxt->vio_eop = (uint8_t)FIELD_GET(CMPL_CTXT_DATA_W5_VIO_EOP_MASK, cmpt_ctxt[5]);
    ctxt->sh_cmpt = (uint8_t)FIELD_GET(CMPL_CTXT_DATA_W5_SH_CMPT_MASK, cmpt_ctxt[5]);

    /* MD: Set base address and PASID fields */
    ctxt->bs_addr = FIELD_SET(EQDMA_CPM5_COMPL_CTXT_BADDR_HIGH_L_MASK, (uint64_t)baddr4_high_l) |
                    FIELD_SET(EQDMA_CPM5_COMPL_CTXT_BADDR_HIGH_H_MASK, (uint64_t)baddr4_high_h) |
                    FIELD_SET(EQDMA_CPM5_COMPL_CTXT_BADDR_LOW_MASK, (uint64_t)baddr4_low);

    ctxt->pasid = FIELD_SET(EQDMA_CPM5_CMPL_CTXT_PASID_GET_L_MASK, pasid_l) |
                  FIELD_SET(EQDMA_CPM5_CMPL_CTXT_PASID_GET_H_MASK, (uint64_t)pasid_h);

    ctxt->pidx = FIELD_SET(QDMA_COMPL_CTXT_PIDX_GET_L_MASK, pidx_l) |
                 FIELD_SET(QDMA_COMPL_CTXT_PIDX_GET_H_MASK, pidx_h);

    return QDMA_SUCCESS;
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_cmpt_context_clear() - Clear completion context in the hardware.
 * @dev_hndl: Device handle used for register access.
 * @hw_qid: Hardware queue ID of the queue.
 *
 * Return: 0 on success and < 0 on failure.
 *****************************************************************************/
static int eqdma_cpm5_cmpt_context_clear(void *dev_hndl, uint16_t hw_qid)
{
    enum ind_ctxt_cmd_sel sel = QDMA_CTXT_SEL_CMPT;

    /* MD: Input argument check */
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n", __func__,
                       -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    /* MD: Clear the completion context */
    return eqdma_cpm5_indirect_reg_clear(dev_hndl, sel, hw_qid);
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_cmpt_context_invalidate() - Invalidate completion context in the hardware.
 * @dev_hndl: Device handle used for register access.
 * @hw_qid: Hardware queue ID of the queue.
 *
 * Return: 0 on success and < 0 on failure.
 *****************************************************************************/
static int eqdma_cpm5_cmpt_context_invalidate(void *dev_hndl, uint16_t hw_qid)
{
    enum ind_ctxt_cmd_sel sel = QDMA_CTXT_SEL_CMPT;

    /* MD: Input argument check */
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n", __func__,
                       -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    /* MD: Invalidate the completion context */
    return eqdma_cpm5_indirect_reg_invalidate(dev_hndl, sel, hw_qid);
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_cmpt_ctx_conf() - Configure completion context based on access type.
 * @dev_hndl: Device handle used for register access.
 * @hw_qid: Hardware queue ID of the queue.
 * @ctxt: Pointer to the context data structure.
 * @access_type: HW access type (qdma_hw_access_type enum) value.
 *
 * Return: 0 on success and < 0 on failure.
 *****************************************************************************/
int eqdma_cpm5_cmpt_ctx_conf(void *dev_hndl, uint16_t hw_qid,
                             struct qdma_descq_cmpt_ctxt *ctxt,
                             enum qdma_hw_access_type access_type)
{
    int rv = QDMA_SUCCESS;

    /* MD: Configure the completion context based on the access type */
    switch (access_type) {
    case QDMA_HW_ACCESS_READ:
        rv = eqdma_cpm5_cmpt_context_read(dev_hndl, hw_qid, ctxt);
        break;
    case QDMA_HW_ACCESS_WRITE:
        rv = eqdma_cpm5_cmpt_context_write(dev_hndl, hw_qid, ctxt);
        break;
    case QDMA_HW_ACCESS_CLEAR:
        rv = eqdma_cpm5_cmpt_context_clear(dev_hndl, hw_qid);
        break;
    case QDMA_HW_ACCESS_INVALIDATE:
        rv = eqdma_cpm5_cmpt_context_invalidate(dev_hndl, hw_qid);
        break;
    default:
        qdma_log_error("%s: access_type(%d) invalid, err:%d\n",
                       __func__, access_type, -QDMA_ERR_INV_PARAM);
        rv = -QDMA_ERR_INV_PARAM;
        break;
    }

    return rv;
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_hw_context_read() - Read hardware context from the hardware.
 * @dev_hndl: Device handle used for register access.
 * @c2h: Flag indicating if the queue is C2H (1 for C2H, 0 for H2C).
 * @hw_qid: Hardware queue ID of the queue.
 * @ctxt: Pointer to the output context data structure to be filled.
 *
 * Return: 0 on success and < 0 on failure.
 *****************************************************************************/
static int eqdma_cpm5_hw_context_read(void *dev_hndl, uint8_t c2h,
                                      uint16_t hw_qid, struct qdma_descq_hw_ctxt *ctxt)
{
    int rv = QDMA_SUCCESS;
    uint32_t hw_ctxt[EQDMA_CPM5_HW_CONTEXT_NUM_WORDS] = {0};
    enum ind_ctxt_cmd_sel sel = c2h ? QDMA_CTXT_SEL_HW_C2H : QDMA_CTXT_SEL_HW_H2C;

    /* MD: Input arguments check */
    if (!dev_hndl || !ctxt) {
        qdma_log_error("%s: dev_handle or hw_ctxt NULL, err:%d\n",
                       __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    /* MD: Read the hardware context from the hardware */
    rv = eqdma_cpm5_indirect_reg_read(dev_hndl, sel, hw_qid,
                                      EQDMA_CPM5_HW_CONTEXT_NUM_WORDS, hw_ctxt);
    if (rv < 0)
        return rv;

    /* MD: Extract fields from the context data */
    ctxt->cidx = FIELD_GET(HW_IND_CTXT_DATA_W0_CIDX_MASK, hw_ctxt[0]);
    ctxt->crd_use = (uint16_t)(FIELD_GET(HW_IND_CTXT_DATA_W0_CRD_USE_MASK, hw_ctxt[0]));

    ctxt->dsc_pend = (uint8_t)(FIELD_GET(HW_IND_CTXT_DATA_W1_DSC_PND_MASK, hw_ctxt[1]));
    ctxt->idl_stp_b = (uint8_t)(FIELD_GET(HW_IND_CTXT_DATA_W1_IDL_STP_B_MASK, hw_ctxt[1]));
    ctxt->evt_pnd = (uint8_t)(FIELD_GET(HW_IND_CTXT_DATA_W1_EVT_PND_MASK, hw_ctxt[1]));
    ctxt->fetch_pnd = (uint8_t)(FIELD_GET(HW_IND_CTXT_DATA_W1_DSC_PND_MASK, hw_ctxt[1]));

    qdma_log_debug("%s: cidx=%hu, crd_use=%hu, dsc_pend=%x\n",
                   __func__, ctxt->cidx, ctxt->crd_use, ctxt->dsc_pend);
    qdma_log_debug("%s: idl_stp_b=%x, evt_pnd=%x, fetch_pnd=%x\n",
                   __func__, ctxt->idl_stp_b, ctxt->evt_pnd, ctxt->fetch_pnd);

    return QDMA_SUCCESS;
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_hw_context_clear() - Clear hardware context in the hardware.
 * @dev_hndl: Device handle used for register access.
 * @c2h: Flag indicating if the queue is C2H (1 for C2H, 0 for H2C).
 * @hw_qid: Hardware queue ID of the queue.
 *
 * Return: 0 on success and < 0 on failure.
 *****************************************************************************/
static int eqdma_cpm5_hw_context_clear(void *dev_hndl, uint8_t c2h,
                                       uint16_t hw_qid)
{
    enum ind_ctxt_cmd_sel sel = c2h ? QDMA_CTXT_SEL_HW_C2H : QDMA_CTXT_SEL_HW_H2C;

    /* MD: Input argument check */
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n", __func__,
                       -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    /* MD: Clear the hardware context */
    return eqdma_cpm5_indirect_reg_clear(dev_hndl, sel, hw_qid);
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_hw_context_invalidate() - Invalidate hardware context in the hardware.
 * @dev_hndl: Device handle used for register access.
 * @c2h: Flag indicating if the queue is C2H (1 for C2H, 0 for H2C).
 * @hw_qid: Hardware queue ID of the queue.
 *
 * Return: 0 on success and < 0 on failure.
 *****************************************************************************/
static int eqdma_cpm5_hw_context_invalidate(void *dev_hndl, uint8_t c2h,
                                            uint16_t hw_qid)
{
    enum ind_ctxt_cmd_sel sel = c2h ? QDMA_CTXT_SEL_HW_C2H : QDMA_CTXT_SEL_HW_H2C;

    /* MD: Input argument check */
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n", __func__,
                       -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    /* MD: Invalidate the hardware context */
    return eqdma_cpm5_indirect_reg_invalidate(dev_hndl, sel, hw_qid);
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_hw_ctx_conf() - Configure hardware context based on access type.
 * @dev_hndl: Device handle used for register access.
 * @c2h: Flag indicating if the queue is C2H (1 for C2H, 0 for H2C).
 * @hw_qid: Hardware queue ID of the queue.
 * @ctxt: Pointer to the context data structure.
 * @access_type: HW access type (qdma_hw_access_type enum) value.
 *               Note: QDMA_HW_ACCESS_WRITE is not supported.
 *
 * Return: 0 on success and < 0 on failure.
 *****************************************************************************/
int eqdma_cpm5_hw_ctx_conf(void *dev_hndl, uint8_t c2h, uint16_t hw_qid,
                           struct qdma_descq_hw_ctxt *ctxt,
                           enum qdma_hw_access_type access_type)
{
    int rv = QDMA_SUCCESS;

    /* MD: Validate c2h value */
    if (c2h > 1) {
        qdma_log_error("%s: c2h(%d) invalid, err:%d\n",
                       __func__, c2h, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    /* MD: Configure the hardware context based on the access type */
    switch (access_type) {
    case QDMA_HW_ACCESS_READ:
        rv = eqdma_cpm5_hw_context_read(dev_hndl, c2h, hw_qid, ctxt);
        break;
    case QDMA_HW_ACCESS_CLEAR:
        rv = eqdma_cpm5_hw_context_clear(dev_hndl, c2h, hw_qid);
        break;
    case QDMA_HW_ACCESS_INVALIDATE:
        rv = eqdma_cpm5_hw_context_invalidate(dev_hndl, c2h, hw_qid);
        break;
    case QDMA_HW_ACCESS_WRITE:
    default:
        qdma_log_error("%s: access_type=%d is invalid, err:%d\n",
                       __func__, access_type, -QDMA_ERR_INV_PARAM);
        rv = -QDMA_ERR_INV_PARAM;
        break;
    }

    return rv;
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_credit_context_read() - Read credit context from the hardware.
 * @dev_hndl: Device handle used for register access.
 * @c2h: Flag indicating if the queue is C2H (1 for C2H, 0 for H2C).
 * @hw_qid: Hardware queue ID of the queue.
 * @ctxt: Pointer to the context data structure.
 *
 * Return: 0 on success and < 0 on failure.
 *****************************************************************************/
static int eqdma_cpm5_credit_context_read(void *dev_hndl, uint8_t c2h,
                                          uint16_t hw_qid,
                                          struct qdma_descq_credit_ctxt *ctxt)
{
    int rv = QDMA_SUCCESS;
    uint32_t cr_ctxt[EQDMA_CPM5_CR_CONTEXT_NUM_WORDS] = {0};
    enum ind_ctxt_cmd_sel sel = c2h ? QDMA_CTXT_SEL_CR_C2H : QDMA_CTXT_SEL_CR_H2C;

    /* MD: Input arguments check */
    if (!dev_hndl || !ctxt) {
        qdma_log_error("%s: dev_hndl=%p credit_ctxt=%p, err:%d\n",
                       __func__, dev_hndl, ctxt, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    /* MD: Read the credit context from the hardware */
    rv = eqdma_cpm5_indirect_reg_read(dev_hndl, sel, hw_qid,
                                      EQDMA_CPM5_CR_CONTEXT_NUM_WORDS, cr_ctxt);
    if (rv < 0)
        return rv;

    /* MD: Extract credit value */
    ctxt->credit = FIELD_GET(CRED_CTXT_DATA_W0_CREDT_MASK, cr_ctxt[0]);

    qdma_log_debug("%s: credit=%u\n", __func__, ctxt->credit);

    return QDMA_SUCCESS;
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_credit_context_clear() - Clear credit context in the hardware.
 * @dev_hndl: Device handle used for register access.
 * @c2h: Flag indicating if the queue is C2H (1 for C2H, 0 for H2C).
 * @hw_qid: Hardware queue ID of the queue.
 *
 * Return: 0 on success and < 0 on failure.
 *****************************************************************************/
static int eqdma_cpm5_credit_context_clear(void *dev_hndl, uint8_t c2h,
                                           uint16_t hw_qid)
{
    enum ind_ctxt_cmd_sel sel = c2h ? QDMA_CTXT_SEL_CR_C2H : QDMA_CTXT_SEL_CR_H2C;

    /* MD: Input argument check */
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n", __func__,
                       -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    /* MD: Clear the credit context */
    return eqdma_cpm5_indirect_reg_clear(dev_hndl, sel, hw_qid);
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_credit_context_invalidate() - Invalidate credit context in the hardware.
 * @dev_hndl: Device handle used for register access.
 * @c2h: Flag indicating if the queue is C2H (1 for C2H, 0 for H2C).
 * @hw_qid: Hardware queue ID of the queue.
 *
 * Return: 0 on success and < 0 on failure.
 *****************************************************************************/
static int eqdma_cpm5_credit_context_invalidate(void *dev_hndl, uint8_t c2h,
                                                uint16_t hw_qid)
{
    enum ind_ctxt_cmd_sel sel = c2h ? QDMA_CTXT_SEL_CR_C2H : QDMA_CTXT_SEL_CR_H2C;

    /* MD: Input argument check */
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n", __func__,
                       -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    /* MD: Invalidate the credit context */
    return eqdma_cpm5_indirect_reg_invalidate(dev_hndl, sel, hw_qid);
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_credit_ctx_conf() - Configure credit context based on access type.
 * @dev_hndl: Device handle used for register access.
 * @c2h: Flag indicating if the queue is C2H (1 for C2H, 0 for H2C).
 * @hw_qid: Hardware queue ID of the queue.
 * @ctxt: Pointer to the credit context data structure.
 * @access_type: HW access type (qdma_hw_access_type enum) value.
 *               QDMA_HW_ACCESS_WRITE is not supported.
 *
 * Return: 0 on success and < 0 on failure.
 *****************************************************************************/
int eqdma_cpm5_credit_ctx_conf(void *dev_hndl, uint8_t c2h,
                               uint16_t hw_qid, struct qdma_descq_credit_ctxt *ctxt,
                               enum qdma_hw_access_type access_type)
{
    int rv = QDMA_SUCCESS;

    /* MD: Validate C2H flag */
    if (c2h > 1) {
        qdma_log_error("%s: c2h(%d) invalid, err:%d\n",
                       __func__, c2h, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    /* MD: Configure the credit context based on the access type */
    switch (access_type) {
    case QDMA_HW_ACCESS_READ:
        rv = eqdma_cpm5_credit_context_read(dev_hndl, c2h, hw_qid, ctxt);
        break;
    case QDMA_HW_ACCESS_CLEAR:
        rv = eqdma_cpm5_credit_context_clear(dev_hndl, c2h, hw_qid);
        break;
    case QDMA_HW_ACCESS_INVALIDATE:
        rv = eqdma_cpm5_credit_context_invalidate(dev_hndl, c2h, hw_qid);
        break;
    case QDMA_HW_ACCESS_WRITE:
    default:
        qdma_log_error("%s: Invalid access type=%d, err:%d\n",
                       __func__, access_type, -QDMA_ERR_INV_PARAM);
        rv = -QDMA_ERR_INV_PARAM;
        break;
    }

    return rv;
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_fmap_context_write() - Create fmap context and program it into the hardware.
 * @dev_hndl: Device handle used for register access.
 * @func_id: Function ID of the device.
 * @config: Pointer to the fmap data structure.
 *
 * Return: 0 on success and < 0 on failure.
 *****************************************************************************/
static int eqdma_cpm5_fmap_context_write(void *dev_hndl, uint16_t func_id,
                                         const struct qdma_fmap_cfg *config)
{
    uint32_t fmap[EQDMA_CPM5_FMAP_NUM_WORDS] = {0};
    uint16_t num_words_count = 0;
    enum ind_ctxt_cmd_sel sel = QDMA_CTXT_SEL_FMAP;

    /* MD: Input arguments check */
    if (!dev_hndl || !config) {
        qdma_log_error("%s: dev_handle=%p fmap=%p NULL, err:%d\n",
                       __func__, dev_hndl, config, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    qdma_log_debug("%s: func_id=%hu, qbase=%hu, qmax=%hu\n", __func__,
                   func_id, config->qbase, config->qmax);

    /* MD: Populate the fmap context array */
    fmap[num_words_count++] = FIELD_SET(EQDMA_CPM5_FMAP_CTXT_W0_QID_MASK, config->qbase);
    fmap[num_words_count++] = FIELD_SET(EQDMA_CPM5_FMAP_CTXT_W1_QID_MAX_MASK, config->qmax);

    /* MD: Write the fmap context to the hardware */
    return eqdma_cpm5_indirect_reg_write(dev_hndl, sel, func_id, fmap, num_words_count);
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_fmap_context_read() - Read fmap context from the hardware.
 * @dev_hndl: Device handle used for register access.
 * @func_id: Function ID of the device.
 * @config: Pointer to the output fmap data structure.
 *
 * Return: 0 on success and < 0 on failure.
 *****************************************************************************/
static int eqdma_cpm5_fmap_context_read(void *dev_hndl, uint16_t func_id,
                                        struct qdma_fmap_cfg *config)
{
    int rv = QDMA_SUCCESS;
    uint32_t fmap[EQDMA_CPM5_FMAP_NUM_WORDS] = {0};
    enum ind_ctxt_cmd_sel sel = QDMA_CTXT_SEL_FMAP;

    /* MD: Input arguments check */
    if (!dev_hndl || !config) {
        qdma_log_error("%s: dev_handle=%p fmap=%p NULL, err:%d\n",
                       __func__, dev_hndl, config, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    /* MD: Read the fmap context from the hardware */
    rv = eqdma_cpm5_indirect_reg_read(dev_hndl, sel, func_id, EQDMA_CPM5_FMAP_NUM_WORDS, fmap);
    if (rv < 0)
        return rv;

    /* MD: Extract fields from the context data */
    config->qbase = FIELD_GET(EQDMA_CPM5_FMAP_CTXT_W0_QID_MASK, fmap[0]);
    config->qmax = FIELD_GET(EQDMA_CPM5_FMAP_CTXT_W1_QID_MAX_MASK, fmap[1]);

    qdma_log_debug("%s: func_id=%hu, qbase=%hu, qmax=%hu\n", __func__,
                   func_id, config->qbase, config->qmax);

    return QDMA_SUCCESS;
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_fmap_context_clear() - Clear fmap context in the hardware.
 * @dev_hndl: Device handle used for register access.
 * @func_id: Function ID of the device.
 *
 * Return: 0 on success and < 0 on failure.
 *****************************************************************************/
static int eqdma_cpm5_fmap_context_clear(void *dev_hndl, uint16_t func_id)
{
    enum ind_ctxt_cmd_sel sel = QDMA_CTXT_SEL_FMAP;

    /* MD: Input argument check */
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n", __func__,
                       -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    qdma_log_debug("%s: func_id=%hu\n", __func__, func_id);

    /* MD: Clear the fmap context */
    return eqdma_cpm5_indirect_reg_clear(dev_hndl, sel, func_id);
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_fmap_conf() - Configure fmap context based on access type.
 * @dev_hndl: Device handle used for register access.
 * @func_id: Function ID of the device.
 * @config: Pointer to the fmap data structure.
 * @access_type: HW access type (qdma_hw_access_type enum) value.
 *               QDMA_HW_ACCESS_INVALIDATE is not supported.
 *
 * Return: 0 on success and < 0 on failure.
 *****************************************************************************/
int eqdma_cpm5_fmap_conf(void *dev_hndl, uint16_t func_id,
                         struct qdma_fmap_cfg *config,
                         enum qdma_hw_access_type access_type)
{
    int rv = QDMA_SUCCESS;

    /* MD: Configure the fmap context based on the access type */
    switch (access_type) {
    case QDMA_HW_ACCESS_READ:
        rv = eqdma_cpm5_fmap_context_read(dev_hndl, func_id, config);
        break;
    case QDMA_HW_ACCESS_WRITE:
        rv = eqdma_cpm5_fmap_context_write(dev_hndl, func_id, config);
        break;
    case QDMA_HW_ACCESS_CLEAR:
        rv = eqdma_cpm5_fmap_context_clear(dev_hndl, func_id);
        break;
    case QDMA_HW_ACCESS_INVALIDATE:
    default:
        qdma_log_error("%s: access_type(%d) invalid, err:%d\n",
                       __func__, access_type, -QDMA_ERR_INV_PARAM);
        rv = -QDMA_ERR_INV_PARAM;
        break;
    }

    return rv;
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_indirect_intr_context_write() - Create indirect interrupt context and program it.
 * @dev_hndl: Device handle used for register access.
 * @ring_index: Indirect interrupt ring index.
 * @ctxt: Pointer to the interrupt context data structure.
 *
 * Return: 0 on success and < 0 on failure.
 *****************************************************************************/
static int eqdma_cpm5_indirect_intr_context_write(void *dev_hndl,
                                                  uint16_t ring_index,
                                                  const struct qdma_indirect_intr_ctxt *ctxt)
{
    uint32_t intr_ctxt[EQDMA_CPM5_IND_INTR_CONTEXT_NUM_WORDS] = {0};
    enum ind_ctxt_cmd_sel sel = QDMA_CTXT_SEL_INT_COAL;
    uint32_t baddr_l, baddr_m, baddr_h, pasid_l, pasid_h;
    uint16_t num_words_count = 0;

    /* MD: Input arguments check */
    if (!dev_hndl || !ctxt) {
        qdma_log_error("%s: dev_hndl=%p intr_ctxt=%p, err:%d\n",
                       __func__, dev_hndl, ctxt, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    /* MD: Extract address and PASID fields */
    baddr_l = (uint32_t)FIELD_GET(QDMA_INTR_CTXT_BADDR_GET_L_MASK, ctxt->baddr_4k);
    baddr_m = (uint32_t)FIELD_GET(QDMA_INTR_CTXT_BADDR_GET_M_MASK, ctxt->baddr_4k);
    baddr_h = (uint32_t)FIELD_GET(QDMA_INTR_CTXT_BADDR_GET_H_MASK, ctxt->baddr_4k);

    pasid_l = FIELD_GET(EQDMA_CPM5_INTR_CTXT_PASID_GET_L_MASK, ctxt->pasid);
    pasid_h = FIELD_GET(EQDMA_CPM5_INTR_CTXT_PASID_GET_H_MASK, ctxt->pasid);

    /* MD: Populate the interrupt context array */
    intr_ctxt[num_words_count++] =
        FIELD_SET(INTR_CTXT_DATA_W0_VALID_MASK, ctxt->valid) |
        FIELD_SET(INTR_CTXT_DATA_W0_VEC_MASK, ctxt->vec) |
        FIELD_SET(INTR_CTXT_DATA_W0_INT_ST_MASK, ctxt->int_st) |
        FIELD_SET(INTR_CTXT_DATA_W0_COLOR_MASK, ctxt->color) |
        FIELD_SET(INTR_CTXT_DATA_W0_BADDR_4K_L_MASK, baddr_l);

    intr_ctxt[num_words_count++] =
        FIELD_SET(INTR_CTXT_DATA_W1_BADDR_4K_M_MASK, baddr_m);

    intr_ctxt[num_words_count++] =
        FIELD_SET(INTR_CTXT_DATA_W2_BADDR_4K_H_MASK, baddr_h) |
        FIELD_SET(INTR_CTXT_DATA_W2_PAGE_SIZE_MASK, ctxt->page_size) |
        FIELD_SET(INTR_CTXT_DATA_W2_PIDX_MASK, ctxt->pidx) |
        FIELD_SET(INTR_CTXT_DATA_W2_AT_MASK, ctxt->at) |
        FIELD_SET(INTR_CTXT_DATA_W2_HOST_ID_MASK, ctxt->host_id) |
        FIELD_SET(INTR_CTXT_DATA_W2_PASID_L_MASK, pasid_l);

    intr_ctxt[num_words_count++] =
        FIELD_SET(INTR_CTXT_DATA_W3_PASID_H_MASK, pasid_h) |
        FIELD_SET(INTR_CTXT_DATA_W3_PASID_EN_MASK, ctxt->pasid_en) |
        FIELD_SET(INTR_CTXT_DATA_W3_FUNC_MASK, ctxt->func_id);

    /* MD: Write the interrupt context to the hardware */
    return eqdma_cpm5_indirect_reg_write(dev_hndl, sel, ring_index,
                                         intr_ctxt, num_words_count);
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_indirect_intr_context_read() - Read indirect interrupt context from the hardware.
 * @dev_hndl: Device handle used for register access.
 * @ring_index: Indirect interrupt ring index.
 * @ctxt: Pointer to the output context data structure.
 *
 * Return: 0 on success and < 0 on failure.
 *****************************************************************************/
static int eqdma_cpm5_indirect_intr_context_read(void *dev_hndl,
                                                 uint16_t ring_index,
                                                 struct qdma_indirect_intr_ctxt *ctxt)
{
    int rv = QDMA_SUCCESS;
    uint32_t intr_ctxt[EQDMA_CPM5_IND_INTR_CONTEXT_NUM_WORDS] = {0};
    enum ind_ctxt_cmd_sel sel = QDMA_CTXT_SEL_INT_COAL;
    uint64_t baddr_l, baddr_m, baddr_h, pasid_l, pasid_h;

    /* MD: Input arguments check */
    if (!dev_hndl || !ctxt) {
        qdma_log_error("%s: dev_hndl=%p intr_ctxt=%p, err:%d\n",
                       __func__, dev_hndl, ctxt, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    /* MD: Read the interrupt context from the hardware */
    rv = eqdma_cpm5_indirect_reg_read(dev_hndl, sel, ring_index,
                                      EQDMA_CPM5_IND_INTR_CONTEXT_NUM_WORDS, intr_ctxt);
    if (rv < 0)
        return rv;

    /* MD: Extract fields from the context data */
    ctxt->valid = FIELD_GET(INTR_CTXT_DATA_W0_VALID_MASK, intr_ctxt[0]);
    ctxt->vec = FIELD_GET(INTR_CTXT_DATA_W0_VEC_MASK, intr_ctxt[0]);
    ctxt->int_st = (uint8_t)(FIELD_GET(INTR_CTXT_DATA_W0_INT_ST_MASK, intr_ctxt[0]));
    ctxt->color = (uint8_t)(FIELD_GET(INTR_CTXT_DATA_W0_COLOR_MASK, intr_ctxt[0]));
    baddr_l = FIELD_GET(INTR_CTXT_DATA_W0_BADDR_4K_L_MASK, intr_ctxt[0]);

    baddr_m = FIELD_GET(INTR_CTXT_DATA_W1_BADDR_4K_M_MASK, intr_ctxt[1]);

    baddr_h = FIELD_GET(INTR_CTXT_DATA_W2_BADDR_4K_H_MASK, intr_ctxt[2]);
    ctxt->page_size = FIELD_GET(INTR_CTXT_DATA_W2_PAGE_SIZE_MASK, intr_ctxt[2]);
    ctxt->pidx = (uint16_t)(FIELD_GET(INTR_CTXT_DATA_W2_PIDX_MASK, intr_ctxt[2]));
    ctxt->at = (uint8_t)(FIELD_GET(INTR_CTXT_DATA_W2_AT_MASK, intr_ctxt[2]));
    ctxt->host_id = (uint8_t)(FIELD_GET(INTR_CTXT_DATA_W2_HOST_ID_MASK, intr_ctxt[2]));
    pasid_l = (uint8_t)(FIELD_GET(INTR_CTXT_DATA_W2_PASID_L_MASK, intr_ctxt[2]));

    pasid_h = FIELD_GET(INTR_CTXT_DATA_W3_PASID_H_MASK, intr_ctxt[3]);
    ctxt->pasid_en = (uint8_t)FIELD_GET(INTR_CTXT_DATA_W3_PASID_EN_MASK, intr_ctxt[3]);

    ctxt->func_id = (uint16_t)FIELD_GET(INTR_CTXT_DATA_W3_FUNC_MASK, intr_ctxt[3]);

    ctxt->baddr_4k = FIELD_SET(QDMA_INTR_CTXT_BADDR_GET_L_MASK, baddr_l) |
                     FIELD_SET(QDMA_INTR_CTXT_BADDR_GET_M_MASK, baddr_m) |
                     FIELD_SET(QDMA_INTR_CTXT_BADDR_GET_H_MASK, baddr_h);

    ctxt->pasid = FIELD_SET(EQDMA_CPM5_INTR_CTXT_PASID_GET_L_MASK, pasid_l) |
                  FIELD_SET(EQDMA_CPM5_INTR_CTXT_PASID_GET_H_MASK, pasid_h);

    return QDMA_SUCCESS;
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_indirect_intr_context_clear() - Clear indirect interrupt context.
 * @dev_hndl: Device handle used for register access.
 * @ring_index: Indirect interrupt ring index.
 *
 * Return: 0 on success and < 0 on failure.
 *****************************************************************************/
static int eqdma_cpm5_indirect_intr_context_clear(void *dev_hndl,
                                                  uint16_t ring_index)
{
    enum ind_ctxt_cmd_sel sel = QDMA_CTXT_SEL_INT_COAL;

    /* MD: Input argument check */
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n", __func__,
                       -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    /* MD: Clear the indirect interrupt context */
    return eqdma_cpm5_indirect_reg_clear(dev_hndl, sel, ring_index);
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_indirect_intr_context_invalidate() - Invalidate indirect interrupt context.
 * @dev_hndl: Device handle used for register access.
 * @ring_index: Indirect interrupt ring index.
 *
 * Return: 0 on success and < 0 on failure.
 *****************************************************************************/
static int eqdma_cpm5_indirect_intr_context_invalidate(void *dev_hndl,
                                                       uint16_t ring_index)
{
    enum ind_ctxt_cmd_sel sel = QDMA_CTXT_SEL_INT_COAL;

    /* MD: Input argument check */
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n", __func__,
                       -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    /* MD: Invalidate the indirect interrupt context */
    return eqdma_cpm5_indirect_reg_invalidate(dev_hndl, sel, ring_index);
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_indirect_intr_ctx_conf() - Configure indirect interrupt context.
 * @dev_hndl: Device handle used for register access.
 * @ring_index: Indirect interrupt ring index.
 * @ctxt: Pointer to context data.
 * @access_type: HW access type (qdma_hw_access_type enum) value.
 *
 * Return: 0 on success and < 0 on failure.
 *****************************************************************************/
int eqdma_cpm5_indirect_intr_ctx_conf(void *dev_hndl, uint16_t ring_index,
                                      struct qdma_indirect_intr_ctxt *ctxt,
                                      enum qdma_hw_access_type access_type)
{
    int rv = QDMA_SUCCESS;

    /* MD: Configure the indirect interrupt context based on the access type */
    switch (access_type) {
    case QDMA_HW_ACCESS_READ:
        rv = eqdma_cpm5_indirect_intr_context_read(dev_hndl, ring_index, ctxt);
        break;
    case QDMA_HW_ACCESS_WRITE:
        rv = eqdma_cpm5_indirect_intr_context_write(dev_hndl, ring_index, ctxt);
        break;
    case QDMA_HW_ACCESS_CLEAR:
        rv = eqdma_cpm5_indirect_intr_context_clear(dev_hndl, ring_index);
        break;
    case QDMA_HW_ACCESS_INVALIDATE:
        rv = eqdma_cpm5_indirect_intr_context_invalidate(dev_hndl, ring_index);
        break;
    default:
        qdma_log_error("%s: access_type=%d is invalid, err:%d\n",
                       __func__, access_type, -QDMA_ERR_INV_PARAM);
        rv = -QDMA_ERR_INV_PARAM;
        break;
    }

    return rv;
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_dump_config_regs() - Get QDMA config register dump in a buffer.
 * @dev_hndl: Device handle used for register access.
 * @is_vf: Whether the device is a Virtual Function (VF).
 * @buf: Pointer to buffer to be filled.
 * @buflen: Length of the buffer.
 *
 * Return: Length up to which the buffer is filled on success and < 0 on failure.
 *****************************************************************************/
int eqdma_cpm5_dump_config_regs(void *dev_hndl, uint8_t is_vf,
                                char *buf, uint32_t buflen)
{
    uint32_t i = 0, j = 0;
    struct xreg_info *reg_info;
    uint32_t num_regs = eqdma_cpm5_config_num_regs_get();
    uint32_t len = 0, val = 0;
    int rv = QDMA_SUCCESS;
    char name[DEBGFS_GEN_NAME_SZ] = "";
    struct qdma_dev_attributes dev_cap;

    /* MD: Input argument check */
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n",
                       __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    /* MD: Check if buffer length is sufficient */
    if (buflen < eqdma_cpm5_reg_dump_buf_len()) {
        qdma_log_error("%s: Buffer too small, err:%d\n",
                       __func__, -QDMA_ERR_NO_MEM);
        return -QDMA_ERR_NO_MEM;
    }

    /* MD: Check if the function is called for a VF */
    if (is_vf) {
        qdma_log_error("%s: Wrong API used for VF, err:%d\n",
                       __func__, -QDMA_ERR_HWACC_FEATURE_NOT_SUPPORTED);
        return -QDMA_ERR_HWACC_FEATURE_NOT_SUPPORTED;
    }

    /* MD: Get device attributes */
    eqdma_cpm5_get_device_attributes(dev_hndl, &dev_cap);

    /* MD: Get configuration registers */
    reg_info = eqdma_cpm5_config_regs_get();

    /* MD: Iterate over each register and dump its value */
    for (i = 0; i < num_regs; i++) {
        if ((GET_CAPABILITY_MASK(dev_cap.mm_en, dev_cap.st_en,
                                 dev_cap.mm_cmpt_en, dev_cap.mailbox_en)
             & reg_info[i].mode) == 0)
            continue;

        /* MD: Skip debug registers if debug mode is not enabled */
        if (dev_cap.debug_mode == 0 && reg_info[i].is_debug_reg == 1)
            continue;

        for (j = 0; j < reg_info[i].repeat; j++) {
            rv = QDMA_SNPRINTF_S(name, DEBGFS_GEN_NAME_SZ, DEBGFS_GEN_NAME_SZ,
                                 "%s", reg_info[i].name);
            if ((rv < 0) || (rv > DEBGFS_GEN_NAME_SZ)) {
                qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n",
                               __LINE__, __func__, rv);
                return -QDMA_ERR_NO_MEM;
            }
            val = qdma_reg_read(dev_hndl, (reg_info[i].addr + (j * 4)));
            rv = dump_reg(buf + len, buflen - len,
                          (reg_info[i].addr + (j * 4)), name, val);
            if (rv < 0) {
                qdma_log_error("%s Buff too small, err:%d\n",
                               __func__, -QDMA_ERR_NO_MEM);
                return -QDMA_ERR_NO_MEM;
            }
            len += rv;
        }
    }

    return len;
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_dump_queue_context() - Function to get QDMA queue context dump
 *                                   in a buffer.
 * @dev_hndl: Device handle.
 * @st: Queue Mode (ST or MM).
 * @q_type: Queue type (H2C/C2H/CMPT).
 * @ctxt_data: Queue Context data structure.
 * @buf: Pointer to buffer to be filled.
 * @buflen: Length of the buffer.
 *
 * Return: Length up to which the buffer is filled on success, and < 0 on failure.
 *****************************************************************************/
int eqdma_cpm5_dump_queue_context(void *dev_hndl,
                                  uint8_t st,
                                  enum qdma_dev_q_type q_type,
                                  struct qdma_descq_context *ctxt_data,
                                  char *buf, uint32_t buflen)
{
    int rv = 0;
    uint32_t req_buflen = 0;

    /* MD: Validate input arguments */
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n",
                       __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    if (!ctxt_data) {
        qdma_log_error("%s: ctxt_data is NULL, err:%d\n",
                       __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    if (!buf) {
        qdma_log_error("%s: buf is NULL, err:%d\n",
                       __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    if (q_type >= QDMA_DEV_Q_TYPE_MAX) {
        qdma_log_error("%s: invalid q_type, err:%d\n",
                       __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    /* MD: Get required buffer length */
    rv = eqdma_cpm5_context_buf_len(st, q_type, &req_buflen);
    if (rv != QDMA_SUCCESS)
        return rv;

    /* MD: Check if provided buffer is sufficient */
    if (buflen < req_buflen) {
        qdma_log_error("%s: Too small buffer(%d), reqd(%d), err:%d\n",
                       __func__, buflen, req_buflen, -QDMA_ERR_NO_MEM);
        return -QDMA_ERR_NO_MEM;
    }

    /* MD: Dump the queue context into the buffer */
    rv = dump_eqdma_cpm5_context(ctxt_data, st, q_type, buf, buflen);

    return rv;
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_dump_intr_context() - Function to get QDMA interrupt context
 *                                  dump in a buffer.
 * @dev_hndl: Device handle.
 * @intr_ctx: Interrupt Context data structure.
 * @ring_index: Ring index.
 * @buf: Pointer to buffer to be filled.
 * @buflen: Length of the buffer.
 *
 * Return: Length up to which the buffer is filled on success, and < 0 on failure.
 *****************************************************************************/
int eqdma_cpm5_dump_intr_context(void *dev_hndl,
                                 struct qdma_indirect_intr_ctxt *intr_ctx,
                                 int ring_index,
                                 char *buf, uint32_t buflen)
{
    int rv = 0;
    uint32_t req_buflen = 0;

    /* MD: Validate input arguments */
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n",
                       __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }
    if (!intr_ctx) {
        qdma_log_error("%s: intr_ctx is NULL, err:%d\n",
                       __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    if (!buf) {
        qdma_log_error("%s: buf is NULL, err:%d\n",
                       __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    /* MD: Get required buffer length */
    req_buflen = eqdma_cpm5_intr_context_buf_len();
    if (buflen < req_buflen) {
        qdma_log_error("%s: Too small buffer(%d), reqd(%d), err:%d\n",
                       __func__, buflen, req_buflen, -QDMA_ERR_NO_MEM);
        return -QDMA_ERR_NO_MEM;
    }

    /* MD: Dump the interrupt context into the buffer */
    rv = dump_eqdma_cpm5_intr_context(intr_ctx, ring_index, buf, buflen);

    return rv;
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_read_dump_queue_context() - Function to read and dump the
 *                                        queue context in a buffer.
 * @dev_hndl: Device handle.
 * @func_id: Function ID.
 * @hw_qid: Queue ID.
 * @st: Queue Mode (ST or MM).
 * @q_type: Queue type (H2C/C2H/CMPT).
 * @buf: Pointer to buffer to be filled.
 * @buflen: Length of the buffer.
 *
 * Return: Length up to which the buffer is filled on success, and < 0 on failure.
 *****************************************************************************/
int eqdma_cpm5_read_dump_queue_context(void *dev_hndl,
                                       uint16_t func_id,
                                       uint16_t qid_hw,
                                       uint8_t st,
                                       enum qdma_dev_q_type q_type,
                                       char *buf, uint32_t buflen)
{
    int rv = QDMA_SUCCESS;
    uint32_t req_buflen = 0;
    struct qdma_descq_context context;

    /* MD: Validate input arguments */
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n",
                       __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    if (!buf) {
        qdma_log_error("%s: buf is NULL, err:%d\n",
                       __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    if (q_type >= QDMA_DEV_Q_TYPE_MAX) {
        qdma_log_error("%s: Not supported for q_type, err = %d\n",
                       __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    /* MD: Get required buffer length */
    rv = eqdma_cpm5_context_buf_len(st, q_type, &req_buflen);
    if (rv != QDMA_SUCCESS)
        return rv;

    /* MD: Check if provided buffer is sufficient */
    if (buflen < req_buflen) {
        qdma_log_error("%s: Too small buffer(%d), reqd(%d), err:%d\n",
                       __func__, buflen, req_buflen, -QDMA_ERR_NO_MEM);
        return -QDMA_ERR_NO_MEM;
    }

    /* MD: Initialize context structure */
    qdma_memset(&context, 0, sizeof(struct qdma_descq_context));

    /* MD: Read and configure contexts based on queue type */
    if (q_type != QDMA_DEV_Q_TYPE_CMPT) {
        rv = eqdma_cpm5_sw_ctx_conf(dev_hndl, (uint8_t)q_type,
                                    qid_hw, &(context.sw_ctxt),
                                    QDMA_HW_ACCESS_READ);
        if (rv < 0) {
            qdma_log_error("%s: Failed to read sw context, err = %d",
                           __func__, rv);
            return rv;
        }

        rv = eqdma_cpm5_hw_ctx_conf(dev_hndl, (uint8_t)q_type,
                                    qid_hw, &(context.hw_ctxt),
                                    QDMA_HW_ACCESS_READ);
        if (rv < 0) {
            qdma_log_error("%s: Failed to read hw context, err = %d",
                           __func__, rv);
            return rv;
        }

        rv = eqdma_cpm5_credit_ctx_conf(dev_hndl, (uint8_t)q_type,
                                        qid_hw, &(context.cr_ctxt),
                                        QDMA_HW_ACCESS_READ);
        if (rv < 0) {
            qdma_log_error("%s: Failed to read credit context, err = %d",
                           __func__, rv);
            return rv;
        }

        if (st && (q_type == QDMA_DEV_Q_TYPE_C2H)) {
            rv = eqdma_cpm5_pfetch_ctx_conf(dev_hndl,
                                            qid_hw,
                                            &(context.pfetch_ctxt),
                                            QDMA_HW_ACCESS_READ);
            if (rv < 0) {
                qdma_log_error("%s: Failed to read pftech context, err = %d",
                               __func__, rv);
                return rv;
            }
        }
    }

    if ((st && (q_type == QDMA_DEV_Q_TYPE_C2H)) ||
        (!st && (q_type == QDMA_DEV_Q_TYPE_CMPT))) {
        rv = eqdma_cpm5_cmpt_ctx_conf(dev_hndl, qid_hw,
                                      &(context.cmpt_ctxt),
                                      QDMA_HW_ACCESS_READ);
        if (rv < 0) {
            qdma_log_error("%s: Failed to read cmpt context, err = %d",
                           __func__, rv);
            return rv;
        }
    }

    rv = eqdma_cpm5_fmap_conf(dev_hndl, func_id,
                              &(context.fmap), QDMA_HW_ACCESS_READ);
    if (rv < 0) {
        qdma_log_error("%s: Failed to read fmap context, err = %d",
                       __func__, rv);
        return rv;
    }

    /* MD: Dump the queue context into the buffer */
    rv = dump_eqdma_cpm5_context(&context, st, q_type, buf, buflen);

    return rv;
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_get_user_bar() - Function to get the AXI Master Lite (user bar) number.
 * @dev_hndl: Device handle.
 * @is_vf: Flag indicating whether the device is a Virtual Function (VF).
 * @func_id: Function ID of the PF.
 * @user_bar: Pointer to hold the AXI Master Lite bar number.
 *
 * Return: 0 on success and < 0 on failure.
 *****************************************************************************/
int eqdma_cpm5_get_user_bar(void *dev_hndl, uint8_t is_vf,
                            uint16_t func_id, uint8_t *user_bar)
{
    /* MD: TODO: In future, user bar is identified using RR */
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n",
                       __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }
    UNUSED(func_id);
    UNUSED(is_vf);

    *user_bar = 2;  // MD: Default user bar number

    return QDMA_SUCCESS;
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_hw_ram_sbe_err_process() - Function to dump SBE error debug information.
 * @dev_hndl: Device handle.
 *
 * Return: void
 *****************************************************************************/
static void eqdma_cpm5_hw_ram_sbe_err_process(void *dev_hndl)
{
    /* MD: Dump Single Bit Error (SBE) status registers */
    eqdma_cpm5_dump_reg_info(dev_hndl, EQDMA_CPM5_RAM_SBE_STS_A_ADDR, 1, NULL, 0);
    eqdma_cpm5_dump_reg_info(dev_hndl, EQDMA_CPM5_RAM_SBE_STS_1_A_ADDR, 1, NULL, 0);
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_hw_ram_dbe_err_process() - Function to dump DBE error debug information.
 * @dev_hndl: Device handle.
 *
 * Return: void
 *****************************************************************************/
static void eqdma_cpm5_hw_ram_dbe_err_process(void *dev_hndl)
{
    /* MD: Dump Double Bit Error (DBE) status registers */
    eqdma_cpm5_dump_reg_info(dev_hndl, EQDMA_CPM5_RAM_DBE_STS_A_ADDR, 1, NULL, 0);
    eqdma_cpm5_dump_reg_info(dev_hndl, EQDMA_CPM5_RAM_DBE_STS_1_A_ADDR, 1, NULL, 0);
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_hw_desc_err_process() - Function to dump Descriptor Error information.
 * @dev_hndl: Device handle.
 *
 * Return: void
 *****************************************************************************/
static void eqdma_cpm5_hw_desc_err_process(void *dev_hndl)
{
    int i = 0;
    uint32_t desc_err_reg_list[] = {
        EQDMA_CPM5_GLBL_DSC_ERR_STS_ADDR,
        EQDMA_CPM5_GLBL_DSC_ERR_LOG0_ADDR,
        EQDMA_CPM5_GLBL_DSC_ERR_LOG1_ADDR,
        EQDMA_CPM5_GLBL_DSC_DBG_DAT0_ADDR,
        EQDMA_CPM5_GLBL_DSC_DBG_DAT1_ADDR,
        EQDMA_CPM5_GLBL_DSC_ERR_LOG2_ADDR
    };
    int desc_err_num_regs = sizeof(desc_err_reg_list) / sizeof(uint32_t);

    /* MD: Dump Descriptor Error status and log registers */
    for (i = 0; i < desc_err_num_regs; i++) {
        eqdma_cpm5_dump_reg_info(dev_hndl, desc_err_reg_list[i], 1, NULL, 0);
    }
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_hw_trq_err_process() - Function to dump Target Access Error information.
 * @dev_hndl: Device handle.
 *
 * Return: void
 *****************************************************************************/
static void eqdma_cpm5_hw_trq_err_process(void *dev_hndl)
{
    int i = 0;
    uint32_t trq_err_reg_list[] = {
        EQDMA_CPM5_GLBL_TRQ_ERR_STS_ADDR,
        EQDMA_CPM5_GLBL_TRQ_ERR_LOG_ADDR
    };
    int trq_err_reg_num_regs = sizeof(trq_err_reg_list) / sizeof(uint32_t);

    /* MD: Dump Target Access Error status and log registers */
    for (i = 0; i < trq_err_reg_num_regs; i++) {
        eqdma_cpm5_dump_reg_info(dev_hndl, trq_err_reg_list[i], 1, NULL, 0);
    }
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_hw_err_dump_ctxt_info() - Dump the important context fields on HW error.
 * @dev_hndl: Device handle.
 * @first_err_qid_reg: Register containing the first error QID.
 * @en_st: Flag indicating if ST Mode is enabled.
 * @c2h: Flag indicating if C2H Mode is enabled.
 *
 * Return: void
 *****************************************************************************/
static void eqdma_cpm5_hw_err_dump_ctxt_info(void *dev_hndl,
                                             uint32_t first_err_qid_reg,
                                             uint8_t en_st, uint8_t c2h)
{
    uint16_t first_err_qid = 0;
    struct qdma_descq_sw_ctxt sw_ctxt;
    struct qdma_descq_hw_ctxt hw_ctxt;
    struct qdma_descq_cmpt_ctxt cmpt_ctxt;

    /* MD: Read the first error QID from the register */
    first_err_qid = qdma_reg_read(dev_hndl, first_err_qid_reg);

    /* MD: Read and fill software and hardware contexts */
    eqdma_cpm5_sw_context_read(dev_hndl, c2h, first_err_qid, &sw_ctxt);
    eqdma_cpm5_hw_context_read(dev_hndl, c2h, first_err_qid, &hw_ctxt);
    eqdma_cpm5_fill_sw_ctxt(&sw_ctxt);
    eqdma_cpm5_fill_hw_ctxt(&hw_ctxt);

    /* MD: Compare and log differences between SW and HW contexts */
    if (sw_ctxt.pidx != hw_ctxt.cidx) {
        qdma_log_info("\n%40s\n", "SW Context:");
        qdma_log_info("%-47s %#-10x %u\n",
                      eqdma_cpm5_sw_ctxt_entries[0].name,
                      eqdma_cpm5_sw_ctxt_entries[0].value,
                      eqdma_cpm5_sw_ctxt_entries[0].value);
        qdma_log_info("\n%40s\n", "HW Context:");
        qdma_log_info("%-47s %#-10x %u\n",
                      eqdma_cpm5_hw_ctxt_entries[0].name,
                      eqdma_cpm5_hw_ctxt_entries[0].value,
                      eqdma_cpm5_hw_ctxt_entries[0].value);
    }

    /* MD: Log software context errors */
    if (sw_ctxt.err != 0) {
        qdma_log_info("%-47s %#-10x %u\n",
                      eqdma_cpm5_sw_ctxt_entries[17].name,
                      eqdma_cpm5_sw_ctxt_entries[17].value,
                      eqdma_cpm5_sw_ctxt_entries[17].value);
    }

    qdma_log_info("%-47s %#-10x %u\n",
                  eqdma_cpm5_sw_ctxt_entries[18].name,
                  eqdma_cpm5_sw_ctxt_entries[18].value,
                  eqdma_cpm5_sw_ctxt_entries[18].value);

    qdma_log_info("%-47s %#-10x %u\n",
                  eqdma_cpm5_sw_ctxt_entries[19].name,
                  eqdma_cpm5_sw_ctxt_entries[19].value,
                  eqdma_cpm5_sw_ctxt_entries[19].value);

    /* MD: If ST and C2H modes are enabled, read and log completion context */
    if (en_st && c2h) {
        eqdma_cpm5_cmpt_context_read(dev_hndl, first_err_qid, &cmpt_ctxt);
        eqdma_cpm5_fill_cmpt_ctxt(&cmpt_ctxt);

        qdma_log_info("\n%40s\n", "CMPT Context:");
        qdma_log_info("%-47s %#-10x %u\n",
                      eqdma_cpm5_cmpt_ctxt_entries[6].name,
                      eqdma_cpm5_cmpt_ctxt_entries[6].value,
                      eqdma_cpm5_cmpt_ctxt_entries[6].value);
        qdma_log_info("%-47s %#-10x %u\n",
                      eqdma_cpm5_cmpt_ctxt_entries[12].name,
                      eqdma_cpm5_cmpt_ctxt_entries[12].value,
                      eqdma_cpm5_cmpt_ctxt_entries[12].value);
        qdma_log_info("%-47s %#-10x %u\n",
                      eqdma_cpm5_cmpt_ctxt_entries[13].name,
                      eqdma_cpm5_cmpt_ctxt_entries[13].value,
                      eqdma_cpm5_cmpt_ctxt_entries[13].value);

        if (cmpt_ctxt.err != 0) {
            qdma_log_info("%-47s %#-10x %u\n",
                          eqdma_cpm5_cmpt_ctxt_entries[15].name,
                          eqdma_cpm5_cmpt_ctxt_entries[15].value,
                          eqdma_cpm5_cmpt_ctxt_entries[15].value);
        }
    }
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_hw_st_h2c_err_process() - Function to dump MM H2C Error information.
 * @dev_hndl: Device handle used for register access.
 *
 * This function iterates over a list of H2C error registers, dumping their
 * information for debugging purposes. It also dumps context information for
 * the first error queue ID.
 *
 * Return: void
 *****************************************************************************/
static void eqdma_cpm5_hw_st_h2c_err_process(void *dev_hndl)
{
    int i = 0;
    uint32_t st_h2c_err_reg_list[] = {
        EQDMA_CPM5_H2C_ERR_STAT_ADDR,
        EQDMA_CPM5_H2C_FIRST_ERR_QID_ADDR,
        EQDMA_CPM5_H2C_DBG_REG0_ADDR,
        EQDMA_CPM5_H2C_DBG_REG1_ADDR,
        EQDMA_CPM5_H2C_DBG_REG2_ADDR,
        EQDMA_CPM5_H2C_DBG_REG3_ADDR,
        EQDMA_CPM5_H2C_DBG_REG4_ADDR
    };
    int st_h2c_err_num_regs = sizeof(st_h2c_err_reg_list) / sizeof(uint32_t);

    for (i = 0; i < st_h2c_err_num_regs; i++) {
        eqdma_cpm5_dump_reg_info(dev_hndl, st_h2c_err_reg_list[i], 1, NULL, 0);
    }

    eqdma_cpm5_hw_err_dump_ctxt_info(dev_hndl, EQDMA_CPM5_H2C_FIRST_ERR_QID_ADDR, 1, 1);
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_hw_st_c2h_err_process() - Function to dump MM C2H Error information.
 * @dev_hndl: Device handle used for register access.
 *
 * This function iterates over a list of C2H error registers, dumping their
 * information for debugging purposes. It also dumps context information for
 * the first error queue ID.
 *
 * Return: void
 *****************************************************************************/
static void eqdma_cpm5_hw_st_c2h_err_process(void *dev_hndl)
{
    int i = 0;
    uint32_t st_c2h_err_reg_list[] = {
        EQDMA_CPM5_C2H_ERR_STAT_ADDR,
        EQDMA_CPM5_C2H_FATAL_ERR_STAT_ADDR,
        EQDMA_CPM5_C2H_FIRST_ERR_QID_ADDR,
        EQDMA_CPM5_C2H_STAT_S_AXIS_C2H_ACCEPTED_ADDR,
        EQDMA_CPM5_C2H_STAT_S_AXIS_WRB_ACCEPTED_ADDR,
        EQDMA_CPM5_C2H_STAT_DESC_RSP_PKT_ACCEPTED_ADDR,
        EQDMA_CPM5_C2H_STAT_AXIS_PKG_CMP_ADDR,
        EQDMA_CPM5_C2H_STAT_DBG_DMA_ENG_0_ADDR,
        EQDMA_CPM5_C2H_STAT_DBG_DMA_ENG_1_ADDR,
        EQDMA_CPM5_C2H_STAT_DBG_DMA_ENG_2_ADDR,
        EQDMA_CPM5_C2H_STAT_DBG_DMA_ENG_3_ADDR,
        EQDMA_CPM5_C2H_STAT_DESC_RSP_DROP_ACCEPTED_ADDR,
        EQDMA_CPM5_C2H_STAT_DESC_RSP_ERR_ACCEPTED_ADDR
    };
    int st_c2h_err_num_regs = sizeof(st_c2h_err_reg_list) / sizeof(uint32_t);

    for (i = 0; i < st_c2h_err_num_regs; i++) {
        eqdma_cpm5_dump_reg_info(dev_hndl, st_c2h_err_reg_list[i], 1, NULL, 0);
    }

    eqdma_cpm5_hw_err_dump_ctxt_info(dev_hndl, EQDMA_CPM5_C2H_FIRST_ERR_QID_ADDR, 1, 1);
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_mm_c2h0_err_process() - Function to dump MM C2H Error information.
 * @dev_hndl: Device handle used for register access.
 *
 * This function iterates over a list of MM C2H error registers, dumping their
 * information for debugging purposes. It also dumps context information for
 * the error info address.
 *
 * Return: void
 *****************************************************************************/
static void eqdma_cpm5_mm_c2h0_err_process(void *dev_hndl)
{
    int i = 0;
    uint32_t mm_c2h_err_reg_list[] = {
        EQDMA_CPM5_C2H_MM_STATUS_ADDR,
        EQDMA_CPM5_C2H_MM_CMPL_DESC_CNT_ADDR,
        EQDMA_CPM5_C2H_MM_ERR_CODE_ADDR,
        EQDMA_CPM5_C2H_MM_ERR_INFO_ADDR,
        EQDMA_CPM5_C2H_MM_DBG_ADDR
    };
    int mm_c2h_err_num_regs = sizeof(mm_c2h_err_reg_list) / sizeof(uint32_t);

    for (i = 0; i < mm_c2h_err_num_regs; i++) {
        eqdma_cpm5_dump_reg_info(dev_hndl, mm_c2h_err_reg_list[i], 1, NULL, 0);
    }

    eqdma_cpm5_hw_err_dump_ctxt_info(dev_hndl, EQDMA_CPM5_C2H_MM_ERR_INFO_ADDR, 0, 1);
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_mm_h2c0_err_process() - Function to dump MM H2C Error information.
 * @dev_hndl: Device handle used for register access.
 *
 * This function iterates over a list of MM H2C error registers, dumping their
 * information for debugging purposes. It also dumps context information for
 * the error info address.
 *
 * Return: void
 *****************************************************************************/
static void eqdma_cpm5_mm_h2c0_err_process(void *dev_hndl)
{
    int i = 0;
    uint32_t mm_h2c_err_reg_list[] = {
        EQDMA_CPM5_H2C_MM_STATUS_ADDR,
        EQDMA_CPM5_H2C_MM_CMPL_DESC_CNT_ADDR,
        EQDMA_CPM5_H2C_MM_ERR_CODE_ADDR,
        EQDMA_CPM5_H2C_MM_ERR_INFO_ADDR,
        EQDMA_CPM5_H2C_MM_DBG_ADDR
    };
    int mm_h2c_err_num_regs = sizeof(mm_h2c_err_reg_list) / sizeof(uint32_t);

    for (i = 0; i < mm_h2c_err_num_regs; i++) {
        eqdma_cpm5_dump_reg_info(dev_hndl, mm_h2c_err_reg_list[i], 1, NULL, 0);
    }

    eqdma_cpm5_hw_err_dump_ctxt_info(dev_hndl, EQDMA_CPM5_H2C_MM_ERR_INFO_ADDR, 0, 1);
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_hw_get_error_name() - Function to get the error in string format.
 * @err_idx: Error index.
 *
 * This function returns the error name corresponding to the given error index.
 * If the error index is invalid, it logs an error and returns NULL.
 *
 * Return: Error name string on success, NULL on failure.
 *****************************************************************************/
const char *eqdma_cpm5_hw_get_error_name(uint32_t err_idx)
{
    if (err_idx >= EQDMA_CPM5_ERRS_ALL) {
        qdma_log_error("%s: err_idx=%d is invalid, returning NULL\n",
                       __func__, (enum eqdma_cpm5_error_idx)err_idx);
        return NULL;
    }

    return eqdma_cpm5_err_info[(enum eqdma_cpm5_error_idx)err_idx].err_name;
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_hw_error_process() - Function to find the error that got
 * triggered and call the handler qdma_hw_error_handler of that
 * particular error.
 *
 * @dev_hndl: Device handle used for register access.
 *
 * Return: 0 on success and < 0 on failure.
 *****************************************************************************/
int eqdma_cpm5_hw_error_process(void *dev_hndl)
{
    uint32_t glbl_err_stat = 0, err_stat = 0;
    uint32_t bit = 0, i = 0;
    int32_t idx = 0;
    struct qdma_dev_attributes dev_cap;
    uint32_t hw_err_position[EQDMA_CPM5_TOTAL_LEAF_ERROR_AGGREGATORS] = {
        EQDMA_CPM5_DSC_ERR_POISON,
        EQDMA_CPM5_TRQ_ERR_CSR_UNMAPPED,
        EQDMA_CPM5_ST_C2H_ERR_MTY_MISMATCH,
        EQDMA_CPM5_ST_FATAL_ERR_MTY_MISMATCH,
        EQDMA_CPM5_ST_H2C_ERR_ZERO_LEN_DESC,
        EQDMA_CPM5_SBE_1_ERR_RC_RRQ_EVEN_RAM,
        EQDMA_CPM5_SBE_ERR_MI_H2C0_DAT,
        EQDMA_CPM5_DBE_1_ERR_RC_RRQ_EVEN_RAM,
        EQDMA_CPM5_DBE_ERR_MI_H2C0_DAT,
        EQDMA_CPM5_MM_C2H_WR_SLR_ERR,
        EQDMA_CPM5_MM_H2C0_RD_HDR_POISON_ERR
    };

    /* MD: Check if the device handle is valid */
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n",
                       __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    /* MD: Retrieve device attributes */
    eqdma_cpm5_get_device_attributes(dev_hndl, &dev_cap);

    /* MD: Read the global error status register */
    glbl_err_stat = qdma_reg_read(dev_hndl, EQDMA_CPM5_GLBL_ERR_STAT_ADDR);

    /* MD: If no global error is detected, return */
    if (!glbl_err_stat)
        return QDMA_HW_ERR_NOT_DETECTED;

    qdma_log_info("%s: Global Err Reg(0x%x) = 0x%x\n",
                  __func__, EQDMA_CPM5_GLBL_ERR_STAT_ADDR, glbl_err_stat);

    /* MD: Iterate through all error aggregators */
    for (i = 0; i < EQDMA_CPM5_TOTAL_LEAF_ERROR_AGGREGATORS; i++) {
        bit = hw_err_position[i];

        /* MD: Skip streaming errors if streaming is not enabled */
        if ((!dev_cap.st_en) &&
            (bit == EQDMA_CPM5_ST_C2H_ERR_MTY_MISMATCH ||
             bit == EQDMA_CPM5_ST_FATAL_ERR_MTY_MISMATCH ||
             bit == EQDMA_CPM5_ST_H2C_ERR_ZERO_LEN_DESC))
            continue;

        /* MD: Read the error status register for the current error */
        err_stat = qdma_reg_read(dev_hndl, eqdma_cpm5_err_info[bit].stat_reg_addr);
        if (err_stat) {
            qdma_log_info("addr = 0x%08x val = 0x%08x",
                          eqdma_cpm5_err_info[bit].stat_reg_addr, err_stat);

            /* MD: Process the hardware error */
            eqdma_cpm5_err_info[bit].eqdma_cpm5_hw_err_process(dev_hndl);
            for (idx = bit; idx < all_eqdma_cpm5_hw_errs[i]; idx++) {
                /* MD: Call the platform-specific handler */
                if (err_stat & eqdma_cpm5_err_info[idx].leaf_err_mask)
                    qdma_log_error("%s detected %s\n",
                                   __func__, eqdma_cpm5_hw_get_error_name(idx));
            }
            /* MD: Clear the error status register */
            qdma_reg_write(dev_hndl, eqdma_cpm5_err_info[bit].stat_reg_addr, err_stat);
        }
    }

    /* MD: Write 1 to the global status register to clear the bits */
    qdma_reg_write(dev_hndl, EQDMA_CPM5_GLBL_ERR_STAT_ADDR, glbl_err_stat);

    return QDMA_SUCCESS;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_hw_error_enable() - Function to enable all or a specific error
 *
 * @dev_hndl: Device handle used for register access.
 * @err_idx: Error index to enable.
 *
 * Return: 0 on success and < 0 on failure.
 *****************************************************************************/
int eqdma_cpm5_hw_error_enable(void *dev_hndl, uint32_t err_idx)
{
    uint32_t idx = 0, i = 0;
    uint32_t reg_val = 0;
    struct qdma_dev_attributes dev_cap;

    /* MD: Check if the device handle is valid */
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n",
                       __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    /* MD: Validate the error index */
    if (err_idx > EQDMA_CPM5_ERRS_ALL) {
        qdma_log_error("%s: err_idx=%d is invalid, err:%d\n",
                       __func__, (enum eqdma_cpm5_error_idx)err_idx,
                       -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    /* MD: Retrieve device attributes */
    eqdma_cpm5_get_device_attributes(dev_hndl, &dev_cap);

    /* MD: Enable all errors if specified */
    if (err_idx == EQDMA_CPM5_ERRS_ALL) {
        for (i = 0; i < EQDMA_CPM5_TOTAL_LEAF_ERROR_AGGREGATORS; i++) {
            idx = all_eqdma_cpm5_hw_errs[i];

            /* MD: Skip streaming errors if streaming is not enabled */
            if (!dev_cap.st_en) {
                if (idx == EQDMA_CPM5_ST_C2H_ERR_ALL ||
                    idx == EQDMA_CPM5_ST_FATAL_ERR_ALL ||
                    idx == EQDMA_CPM5_ST_H2C_ERR_ALL)
                    continue;
            }

            /* MD: Enable the leaf error */
            reg_val = eqdma_cpm5_err_info[idx].leaf_err_mask;
            qdma_reg_write(dev_hndl, eqdma_cpm5_err_info[idx].mask_reg_addr, reg_val);

            /* MD: Enable the global error */
            reg_val = qdma_reg_read(dev_hndl, EQDMA_CPM5_GLBL_ERR_MASK_ADDR);
            reg_val |= FIELD_SET(eqdma_cpm5_err_info[idx].global_err_mask, 1);
            qdma_reg_write(dev_hndl, EQDMA_CPM5_GLBL_ERR_MASK_ADDR, reg_val);
        }
    } else {
        /* MD: Skip streaming errors if streaming is not enabled */
        if (!dev_cap.st_en) {
            if (err_idx >= EQDMA_CPM5_ST_C2H_ERR_MTY_MISMATCH &&
                err_idx <= EQDMA_CPM5_ST_H2C_ERR_ALL)
                return QDMA_SUCCESS;
        }

        /* MD: Enable the specific leaf error */
        reg_val = qdma_reg_read(dev_hndl, eqdma_cpm5_err_info[err_idx].mask_reg_addr);
        reg_val |= FIELD_SET(eqdma_cpm5_err_info[err_idx].leaf_err_mask, 1);
        qdma_reg_write(dev_hndl, eqdma_cpm5_err_info[err_idx].mask_reg_addr, reg_val);

        /* MD: Enable the specific global error */
        reg_val = qdma_reg_read(dev_hndl, EQDMA_CPM5_GLBL_ERR_MASK_ADDR);
        reg_val |= FIELD_SET(eqdma_cpm5_err_info[err_idx].global_err_mask, 1);
        qdma_reg_write(dev_hndl, EQDMA_CPM5_GLBL_ERR_MASK_ADDR, reg_val);
    }

    return QDMA_SUCCESS;
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_get_device_attributes() - Function to get the QDMA device attributes.
 * @dev_hndl: Device handle used for register access.
 * @dev_info: Pointer to hold the device information.
 *
 * Return: 0 on success and < 0 on failure.
 *****************************************************************************/
int eqdma_cpm5_get_device_attributes(void *dev_hndl,
                                     struct qdma_dev_attributes *dev_info)
{
    uint32_t reg_val = 0;

    /* MD: Input arguments check */
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n",
                       __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }
    if (!dev_info) {
        qdma_log_error("%s: dev_info is NULL, err:%d\n",
                       __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    /* MD: Read number of PFs */
    reg_val = qdma_reg_read(dev_hndl, QDMA_OFFSET_GLBL2_PF_BARLITE_INT);
    dev_info->num_pfs = FIELD_GET(QDMA_GLBL2_PF0_BAR_MAP_MASK, reg_val);

    /* MD: Read number of queues */
    reg_val = qdma_reg_read(dev_hndl, EQDMA_CPM5_GLBL2_CHANNEL_CAP_ADDR);
    dev_info->num_qs = FIELD_GET(GLBL2_CHANNEL_CAP_MULTIQ_MAX_MASK, reg_val);

    /* MD: Adjust for maximum queues if necessary */
    if (dev_info->num_qs == 0xFFF)
        dev_info->num_qs++;

    /* MD: Read miscellaneous capabilities */
    reg_val = qdma_reg_read(dev_hndl, EQDMA_CPM5_GLBL2_MISC_CAP_ADDR);
    dev_info->mailbox_en = FIELD_GET(EQDMA_CPM5_GLBL2_MAILBOX_EN_MASK, reg_val);
    dev_info->flr_present = FIELD_GET(EQDMA_CPM5_GLBL2_FLR_PRESENT_MASK, reg_val);
    dev_info->mm_cmpt_en = 0;
    dev_info->debug_mode = FIELD_GET(EQDMA_CPM5_GLBL2_DBG_MODE_EN_MASK, reg_val);
    dev_info->desc_eng_mode = FIELD_GET(EQDMA_CPM5_GLBL2_DESC_ENG_MODE_MASK, reg_val);

    /* MD: Check if ST/MM is enabled */
    reg_val = qdma_reg_read(dev_hndl, EQDMA_CPM5_GLBL2_CHANNEL_MDMA_ADDR);
    dev_info->st_en = (FIELD_GET(GLBL2_CHANNEL_MDMA_C2H_ST_MASK, reg_val) &&
                       FIELD_GET(GLBL2_CHANNEL_MDMA_H2C_ST_MASK, reg_val)) ? 1 : 0;
    dev_info->mm_en = (FIELD_GET(GLBL2_CHANNEL_MDMA_C2H_ENG_MASK, reg_val) &&
                       FIELD_GET(GLBL2_CHANNEL_MDMA_H2C_ENG_MASK, reg_val)) ? 1 : 0;

    /* MD: Set number of MM channels */
    dev_info->mm_channel_max = 2; // MD: Hardcoded for CPM5

    /* MD: Set additional device attributes */
    dev_info->qid2vec_ctx = 0;
    dev_info->cmpt_ovf_chk_dis = 1;
    dev_info->mailbox_intr = 1;
    dev_info->sw_desc_64b = 1;
    dev_info->cmpt_desc_64b = 1;
    dev_info->dynamic_bar = 1;
    dev_info->legacy_intr = 1;
    dev_info->cmpt_trig_count_timer = 1;

    return QDMA_SUCCESS;
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_init_ctxt_memory() - Function to initialize the context memory.
 * @dev_hndl: Device handle used for register access.
 *
 * Return: Returns the platform-specific error code.
 *****************************************************************************/
int eqdma_cpm5_init_ctxt_memory(void *dev_hndl)
{
#ifdef ENABLE_INIT_CTXT_MEMORY
    uint32_t data[QDMA_REG_IND_CTXT_REG_COUNT];
    uint16_t i = 0;
    struct qdma_dev_attributes dev_info;

    /* MD: Input argument check */
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n",
                       __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    /* MD: Initialize context memory */
    qdma_memset(data, 0, sizeof(uint32_t) * QDMA_REG_IND_CTXT_REG_COUNT);
    eqdma_cpm5_get_device_attributes(dev_hndl, &dev_info);

    /* MD: Clear context registers for each queue */
    for (; i < dev_info.num_qs; i++) {
        int sel = QDMA_CTXT_SEL_SW_C2H;
        int rv;

#ifdef TANDEM_BOOT_SUPPORTED
        for (; sel <= QDMA_CTXT_SEL_CR_H2C; sel++) {
            rv = eqdma_cpm5_indirect_reg_clear(dev_hndl, (enum ind_ctxt_cmd_sel)sel, i);
            if (rv < 0)
                return rv;
        }
#else
        for (; sel <= QDMA_CTXT_SEL_PFTCH; sel++) {
            /* MD: Skip PFTCH and CMPT context setup if ST mode is not enabled */
            if ((dev_info.st_en == 0) &&
                ((sel == QDMA_CTXT_SEL_PFTCH) || (sel == QDMA_CTXT_SEL_CMPT))) {
                qdma_log_debug("%s: ST context is skipped: sel = %d\n", __func__, sel);
                continue;
            }

            rv = eqdma_cpm5_indirect_reg_clear(dev_hndl, (enum ind_ctxt_cmd_sel)sel, i);
            if (rv < 0)
                return rv;
        }
#endif
    }

    /* MD: Clear FMAP context for each PF */
    for (i = 0; i < dev_info.num_pfs; i++)
        eqdma_cpm5_indirect_reg_clear(dev_hndl, QDMA_CTXT_SEL_FMAP, i);

#else
    /* MD: Input argument check */
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n",
                       __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }
#endif
    return QDMA_SUCCESS;
}

#ifdef TANDEM_BOOT_SUPPORTED

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_init_st_ctxt() - Initialize the Stream (ST) context for all queues
 *
 * @dev_hndl: device handle pointer
 *
 * This function initializes the Stream context including completion (CMPT) and 
 * prefetch (PFTCH) contexts for all queues in the device.
 *
 * Return: 
 *   - QDMA_SUCCESS on successful initialization
 *   - Error code on failure
 *****************************************************************************/
int eqdma_cpm5_init_st_ctxt(void *dev_hndl)
{
    /* MD: Array to hold context register data */
    uint32_t data[QDMA_REG_IND_CTXT_REG_COUNT];
    uint16_t i = 0;
    struct qdma_dev_attributes dev_info;

    /* MD: Debug: Function entry */
    qdma_log_debug("%s: Entering function\n", __func__);

    /* MD: Validate input parameters */
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n",
                    __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    /* MD: Initialize context data array to zero */
    qdma_memset(data, 0, sizeof(uint32_t) * QDMA_REG_IND_CTXT_REG_COUNT);
    
    /* MD: Get device attributes including number of queues and ST enable status */
    eqdma_cpm5_get_device_attributes(dev_hndl, &dev_info);
    qdma_log_debug("%s: Device has %d queues, ST mode enabled: %d\n", 
                   __func__, dev_info.num_qs, dev_info.st_en);

    /* MD: Iterate through all queues */
    for (; i < dev_info.num_qs; i++) {
        int sel = QDMA_CTXT_SEL_CMPT;
        int rv;

        /* MD: Debug: Processing queue */
        qdma_log_debug("%s: Processing queue %d\n", __func__, i);

        /* MD: Initialize different context types for each queue */
        for (; sel <= QDMA_CTXT_SEL_PFTCH; sel++) {
            /* MD: Skip PFTCH and CMPT context setup if ST mode is disabled */
            if ((dev_info.st_en == 0) &&
                ((sel == QDMA_CTXT_SEL_PFTCH) ||
                (sel == QDMA_CTXT_SEL_CMPT))) {
                qdma_log_debug("%s: ST context is skipped: sel = %d\n",
                    __func__, sel);
                continue;
            }

            /* MD: Clear the indirect context registers */
            qdma_log_debug("%s: Clearing context type %d for queue %d\n", 
                          __func__, sel, i);
            rv = eqdma_cpm5_indirect_reg_clear(dev_hndl,
                    (enum ind_ctxt_cmd_sel)sel, i);
            if (rv < 0) {
                qdma_log_error("%s: Failed to clear context %d for queue %d, err:%d\n",
                              __func__, sel, i, rv);
                return rv;
            }
        }
    }

    qdma_log_debug("%s: ST context initialization completed successfully\n", __func__);
    return QDMA_SUCCESS;
}
#endif

/* MD:****************************************************************************/
/* MD:*
 * get_reg_entry() - Get the register entry index from the register address
 *
 * @reg_addr:    Register address to look up
 * @reg_entry:   Pointer to store the found register entry index
 *
 * Return: 
 *   - 0 on successful lookup
 *   - -QDMA_ERR_INV_PARAM if register not found
 *****************************************************************************/
static int get_reg_entry(uint32_t reg_addr, int *reg_entry)
{
    uint32_t i = 0;
    struct xreg_info *reg_info;
    uint32_t num_regs = eqdma_cpm5_config_num_regs_get();

    /* MD: Debug: Function entry */
    qdma_log_debug("%s: Looking up register address 0x%08x\n", __func__, reg_addr);

    reg_info = eqdma_cpm5_config_regs_get();

    /* MD: Search for the register in the register info array */
    for (i = 0; (i < num_regs - 1); i++) {
        if (reg_info[i].addr == reg_addr) {
            *reg_entry = i;
            qdma_log_debug("%s: Found register at index %d\n", __func__, i);
            break;
        }
    }

    /* MD: Check if register was found */
    if (i >= num_regs - 1) {
        qdma_log_error("%s: 0x%08x is missing register list, err:%d\n",
                    __func__,
                    reg_addr,
                    -QDMA_ERR_INV_PARAM);
        *reg_entry = -1;
        return -QDMA_ERR_INV_PARAM;
    }

    return 0;
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_dump_config_reg_list() - Dump configuration registers to a buffer
 *
 * @dev_hndl:   Device handle pointer
 * @total_regs: Maximum number of registers to read
 * @reg_list:   Array containing register addresses and values to dump
 * @buf:        Output buffer to store the formatted register dump
 * @buflen:     Length of the output buffer
 *
 * This function formats and dumps the contents of specified configuration
 * registers into a human-readable format in the provided buffer.
 *
 * Return: 
 *   - Number of bytes written to buffer on success
 *   - Error code on failure
 *****************************************************************************/
int eqdma_cpm5_dump_config_reg_list(void *dev_hndl, uint32_t total_regs,
        struct qdma_reg_data *reg_list, char *buf, uint32_t buflen)
{
    uint32_t j = 0, len = 0;         /* MD: Buffer length and loop counters */
    uint32_t reg_count = 0;          /* MD: Number of registers processed */
    int reg_data_entry;              /* MD: Index in register info array */
    int rv = 0;                      /* MD: Return value for function calls */
    char name[DEBGFS_GEN_NAME_SZ] = "";  /* MD: Buffer for register name */
    struct xreg_info *reg_info = eqdma_cpm5_config_regs_get();
    struct qdma_dev_attributes dev_cap;

    /* MD: Debug: Function entry */
    qdma_log_debug("%s: Entering function, total_regs=%u\n", __func__, total_regs);

    /* MD: Validate input parameters */
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n",
                __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    if (!buf) {
        qdma_log_error("%s: buf is NULL, err:%d\n",
                __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    /* MD: Get device capabilities including debug mode status */
    eqdma_cpm5_get_device_attributes(dev_hndl, &dev_cap);
    qdma_log_debug("%s: Debug mode status: %d\n", __func__, dev_cap.debug_mode);

    /* MD: Process each register in the list */
    for (reg_count = 0; (reg_count < total_regs);) {
        /* MD: Skip debug registers if debug mode is not enabled */
        if (dev_cap.debug_mode == 0 &&
                reg_info[reg_count].is_debug_reg == 1) {
            qdma_log_debug("%s: Skipping debug register at index %d\n", 
                          __func__, reg_count);
            continue;
        }

        /* MD: Look up register entry in register info table */
        rv = get_reg_entry(reg_list[reg_count].reg_addr,
                    &reg_data_entry);
        if (rv < 0) {
            qdma_log_error("%s: register missing in list, err:%d\n",
                           __func__,
                           -QDMA_ERR_INV_PARAM);
            return rv;
        }

        /* MD: Handle registers that repeat multiple times */
        for (j = 0; j < reg_info[reg_data_entry].repeat; j++) {
            /* MD: Format register name with index for repeated registers */
            rv = QDMA_SNPRINTF_S(name, DEBGFS_GEN_NAME_SZ,
                    DEBGFS_GEN_NAME_SZ,
                    "%s_%d",
                    reg_info[reg_data_entry].name, j);
            if ((rv < 0) || (rv > DEBGFS_GEN_NAME_SZ)) {
                qdma_log_error(
                    "%d:%s snprintf failed, err:%d\n",
                    __LINE__, __func__,
                    rv);
                return -QDMA_ERR_NO_MEM;
            }

            /* MD: Debug: Register processing */
            qdma_log_debug("%s: Dumping register %s at addr 0x%08x\n",
                          __func__, name, 
                          (reg_info[reg_data_entry].addr + (j * 4)));

            /* MD: Format and append register information to buffer */
            rv = dump_reg(buf + len, buflen - len,
                (reg_info[reg_data_entry].addr + (j * 4)),
                    name,
                    reg_list[reg_count + j].reg_val);
            if (rv < 0) {
                qdma_log_error(
                "%s Buff too small, err:%d\n",
                __func__,
                -QDMA_ERR_NO_MEM);
                return -QDMA_ERR_NO_MEM;
            }
            len += rv;
        }
        reg_count += j;
    }

    qdma_log_debug("%s: Successfully dumped %u registers, total length: %u\n",
                   __func__, reg_count, len);
    return len;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_read_reg_list() - Read register values from specified groups
 *
 * @dev_hndl:     Device handle
 * @is_vf:        Whether PF or VF (1 for VF, 0 for PF)
 * @reg_rd_group: Register group to read (1-4)
 * @total_regs:   Pointer to store total registers read
 * @reg_list:     Array to store register addresses and values
 *
 * This function reads registers from specified groups and stores their values.
 * Only supported for VF operations.
 *
 * Return: 
 *   - 0 on success
 *   - Error code on failure
 *****************************************************************************/
int eqdma_cpm5_read_reg_list(void *dev_hndl, uint8_t is_vf,
        uint16_t reg_rd_group,
        uint16_t *total_regs,
        struct qdma_reg_data *reg_list)
{
    uint16_t reg_count = 0, i = 0, j = 0;
    struct xreg_info *reg_info;
    uint32_t num_regs = eqdma_cpm5_config_num_regs_get();
    struct xreg_info *eqdma_cpm5_config_regs = eqdma_cpm5_config_regs_get();
    struct qdma_dev_attributes dev_cap;
    uint32_t reg_start_addr = 0;
    int reg_index = 0;
    int rv = 0;

    /* MD: Debug: Function entry */
    qdma_log_debug("%s: Entering function, reg_rd_group=%u\n", 
                   __func__, reg_rd_group);

    /* MD: Only VF operations are supported */
    if (!is_vf) {
        qdma_log_error("%s: not supported for PF, err:%d\n",
                __func__,
                -QDMA_ERR_HWACC_FEATURE_NOT_SUPPORTED);
        return -QDMA_ERR_HWACC_FEATURE_NOT_SUPPORTED;
    }

    /* MD: Validate input parameters */
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n",
                       __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    if (!reg_list) {
        qdma_log_error("%s: reg_list is NULL, err:%d\n",
                       __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    /* MD: Get device capabilities */
    eqdma_cpm5_get_device_attributes(dev_hndl, &dev_cap);
    qdma_log_debug("%s: Device capabilities - mm_en:%d, st_en:%d, debug_mode:%d\n",
                   __func__, dev_cap.mm_en, dev_cap.st_en, dev_cap.debug_mode);

    /* MD: Determine register group start address */
    switch (reg_rd_group) {
    case QDMA_REG_READ_GROUP_1:
            reg_start_addr = EQDMA_CPM5_REG_GROUP_1_START_ADDR;
            break;
    case QDMA_REG_READ_GROUP_2:
            reg_start_addr = EQDMA_CPM5_REG_GROUP_2_START_ADDR;
            break;
    case QDMA_REG_READ_GROUP_3:
            reg_start_addr = EQDMA_CPM5_REG_GROUP_3_START_ADDR;
            break;
    case QDMA_REG_READ_GROUP_4:
            reg_start_addr = EQDMA_CPM5_REG_GROUP_4_START_ADDR;
            break;
    default:
        qdma_log_error("%s: Invalid slot received\n",
               __func__);
        return -QDMA_ERR_INV_PARAM;
    }

    /* MD: Get register entry index for start address */
    rv = get_reg_entry(reg_start_addr, &reg_index);
    if (rv < 0) {
        qdma_log_error("%s: register missing in list, err:%d\n",
                       __func__,
                       -QDMA_ERR_INV_PARAM);
        return rv;
    }
    reg_info = &eqdma_cpm5_config_regs[reg_index];

    /* MD: Read registers within the group */
    for (i = 0, reg_count = 0;
            ((i < num_regs - 1 - reg_index) &&
            (reg_count < QDMA_MAX_REGISTER_DUMP)); i++) {

        /* MD: Skip registers based on capability mask and access type */
        if (((GET_CAPABILITY_MASK(dev_cap.mm_en, dev_cap.st_en,
                dev_cap.mm_cmpt_en, dev_cap.mailbox_en)
                & reg_info[i].mode) == 0) ||
            (reg_info[i].read_type == QDMA_REG_READ_PF_ONLY)) {
            qdma_log_debug("%s: Skipping register at index %d due to capabilities\n",
                          __func__, i);
            continue;
        }

        /* MD: Skip debug registers if debug mode is disabled */
        if (dev_cap.debug_mode == 0 &&
                reg_info[i].is_debug_reg == 1) {
            qdma_log_debug("%s: Skipping debug register at index %d\n",
                          __func__, i);
            continue;
        }

        /* MD: Handle registers that repeat multiple times */
        for (j = 0; j < reg_info[i].repeat &&
                (reg_count < QDMA_MAX_REGISTER_DUMP);
                j++) {
            reg_list[reg_count].reg_addr =
                    (reg_info[i].addr + (j * 4));
            reg_list[reg_count].reg_val =
                qdma_reg_read(dev_hndl,
                    reg_list[reg_count].reg_addr);
            
            qdma_log_debug("%s: Read register 0x%08x = 0x%08x\n",
                          __func__, reg_list[reg_count].reg_addr,
                          reg_list[reg_count].reg_val);
            reg_count++;
        }
    }

    *total_regs = reg_count;
    qdma_log_debug("%s: Successfully read %u registers\n", __func__, reg_count);
    return rv;
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_write_global_ring_sizes() - Set global ring size array
 *
 * @dev_hndl:     Device handle
 * @index:        Starting index in ring size array
 * @count:        Number of entries to write
 * @glbl_rng_sz:  Array containing ring sizes to write
 *
 * This function writes the global ring sizes starting from the specified index.
 * The total number of entries (index + count) must not exceed QDMA_NUM_RING_SIZES.
 *
 * Return: 
 *   - QDMA_SUCCESS on success
 *   - Error code on failure
 *****************************************************************************/
static int eqdma_cpm5_write_global_ring_sizes(void *dev_hndl, uint8_t index,
                uint8_t count, const uint32_t *glbl_rng_sz)
{
    /* MD: Debug: Function entry */
    qdma_log_debug("%s: Entering function, index=%u, count=%u\n",
                   __func__, index, count);

    /* MD: Validate input parameters */
    if (!dev_hndl || !glbl_rng_sz || !count) {
        qdma_log_error("%s: dev_hndl=%p glbl_rng_sz=%p, err:%func__, dev_hndl, glbl_rng_sz,-QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    /* MD: Check if requested range is within bounds */

    if ((index + count) > QDMA_NUM_RING_SIZES) 
    {
        qdma_log_error("%s: index=%u count=%u > %d, err:%d\n",
                       __func__, index, count,
                       QDMA_NUM_RING_SIZES,
                       -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    /* MD: Write ring sizes to CSR registers */
    qdma_log_debug("%s: Writing %u ring sizes starting at index %u\n",
                   __func__, count, index);
    qdma_write_csr_values(dev_hndl, EQDMA_CPM5_GLBL_RNG_SZ_1_ADDR,
            index, count,
            glbl_rng_sz);

    qdma_log_debug("%s: Successfully wrote global ring sizes\n", __func__);
    return QDMA_SUCCESS;
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_read_global_ring_sizes() - Read global ring size array
 *
 * @dev_hndl:     Device handle
 * @index:        Index from where the values need to be read
 * @count:        Number of entries to read
 * @glbl_rng_sz:  Pointer to array to store the read values
 *
 * This function reads the global ring sizes from hardware starting at specified index.
 * The total number of entries (index + count) must not exceed QDMA_NUM_RING_SIZES.
 *
 * Return: 
 *   - QDMA_SUCCESS on success
 *   - Error code on failure
 *****************************************************************************/
static int eqdma_cpm5_read_global_ring_sizes(void *dev_hndl, uint8_t index,
                uint8_t count, uint32_t *glbl_rng_sz)
{
    /* MD: Debug: Function entry */
    qdma_log_debug("%s: Entering function, index=%u, count=%u\n",
                   __func__, index, count);

    /* MD: Validate input parameters */
    if (!dev_hndl || !glbl_rng_sz || !count) {
        qdma_log_error("%s: dev_hndl=%p glbl_rng_sz=%p, err:%d\n",
                       __func__, dev_hndl, glbl_rng_sz,
                       -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    /* MD: Check if requested range is within bounds */
    if ((index + count) > QDMA_NUM_RING_SIZES) {
        qdma_log_error("%s: index=%u count=%u > %d, err:%d\n",
                       __func__, index, count,
                       QDMA_NUM_C2H_BUFFER_SIZES,
                       -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    /* MD: Read ring sizes from CSR registers */
    qdma_log_debug("%s: Reading %u ring sizes starting at index %u\n",
                   __func__, count, index);
    qdma_read_csr_values(dev_hndl, EQDMA_CPM5_GLBL_RNG_SZ_1_ADDR,
            index, count,
            glbl_rng_sz);

    qdma_log_debug("%s: Successfully read global ring sizes\n", __func__);
    return QDMA_SUCCESS;
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_write_global_timer_count() - Set global timer count values
 *
 * @dev_hndl:      Device handle
 * @index:         Index from where the values need to be written
 * @count:         Number of entries to write
 * @glbl_tmr_cnt:  Array containing timer values to write
 *
 * This function writes the global timer count values to hardware.
 * Requires either ST or MM completion mode to be enabled.
 * The total number of entries (index + count) must not exceed QDMA_NUM_C2H_TIMERS.
 *
 * Return: 
 *   - QDMA_SUCCESS on success
 *   - Error code on failure
 *****************************************************************************/
static int eqdma_cpm5_write_global_timer_count(void *dev_hndl, uint8_t
        index, uint8_t count, const uint32_t *glbl_tmr_cnt)
{
    struct qdma_dev_attributes dev_cap;

    /* MD: Debug: Function entry */
    qdma_log_debug("%s: Entering function, index=%u, count=%u\n",
                   __func__, index, count);

    /* MD: Validate input parameters */
    if (!dev_hndl || !glbl_tmr_cnt || !count) {
        qdma_log_error("%s: dev_hndl=%p glbl_tmr_cnt=%p, err:%d\n",
                       __func__, dev_hndl, glbl_tmr_cnt,
                       -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    /* MD: Check if requested range is within bounds */
    if ((index + count) > QDMA_NUM_C2H_TIMERS) {
        qdma_log_error("%s: index=%u count=%u > %d, err:%d\n",
                       __func__, index, count,
                       QDMA_NUM_C2H_TIMERS,
                       -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    /* MD: Get device capabilities */
    eqdma_cpm5_get_device_attributes(dev_hndl, &dev_cap);
    qdma_log_debug("%s: Device capabilities - ST: %d, MM completion: %d\n",
                   __func__, dev_cap.st_en, dev_cap.mm_cmpt_en);

    /* MD: Write timer values if supported */
    if (dev_cap.st_en || dev_cap.mm_cmpt_en) {
        qdma_log_debug("%s: Writing %u timer values starting at index %u\n",
                      __func__, count, index);
        qdma_write_csr_values(dev_hndl,
                EQDMA_CPM5_C2H_TIMER_CNT_ADDR,
                index, count, glbl_tmr_cnt);
    } else {
        qdma_log_error("%s: ST or MM cmpt not supported, err:%d\n",
                __func__,
                -QDMA_ERR_HWACC_FEATURE_NOT_SUPPORTED);
        return -QDMA_ERR_HWACC_FEATURE_NOT_SUPPORTED;
    }

    qdma_log_debug("%s: Successfully wrote timer values\n", __func__);
    return QDMA_SUCCESS;
}

/* MD: ****************************************************************************/
/* MD:
 * eqdma_cpm5_read_global_timer_count() - Read global timer count values
 *
 * @dev_hndl:      Device handle
 * @index:         Index from where the values need to be read
 * @count:         Number of entries to read
 * @glbl_tmr_cnt:  Pointer to array to store the read values
 *
 * This function reads the global timer count values from hardware.
 * Requires either ST or MM completion mode to be enabled.
 * The total number of entries (index + count) must not exceed QDMA_NUM_C2H_TIMERS.
 *
 * Return: 
 *   - QDMA_SUCCESS on success
 *   - Error code on failure
 **************************************************************************** */
static int eqdma_cpm5_read_global_timer_count(void *dev_hndl,
        uint8_t index, uint8_t count, uint32_t *glbl_tmr_cnt)
{
    struct qdma_dev_attributes dev_cap;

    /* MD: Debug: Function entry */
    qdma_log_debug("%s: Entering function, index=%u, count=%u\n",
                   __func__, index, count);

    /* MD: Validate input parameters */
    if (!dev_hndl || !glbl_tmr_cnt || !count) {
        qdma_log_error("%s: dev_hndl=%p glbl_tmr_cnt=%p, err:%d\n",
                       __func__, dev_hndl, glbl_tmr_cnt,
                       -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    /* MD: Check if requested range is within bounds */
    if ((index + count) > QDMA_NUM_C2H_TIMERS) {
        qdma_log_error("%s: index=%u count=%u > %d, err:%d\n",
                       __func__, index, count,
                       QDMA_NUM_C2H_TIMERS,
                       -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    /* MD: Get device capabilities */
    eqdma_cpm5_get_device_attributes(dev_hndl, &dev_cap);
    qdma_log_debug("%s: Device capabilities - ST: %d, MM completion: %d\n",
                   _cap.st_en, dev_cap.mm_cmpt_en);

    /* MD: Read timer values if supported */
    if (dev_cap.st_en || dev_cap.mm_cmpt_en) {
        qdma_log_debug("%s: Reading %u timer values starting at index %u\n",
                      __func__, count, index);
        qdma_read_csr_values(dev_hndl,
                EQDMA_CPM5_C2H_TIMER_CNT_ADDR, index,
                count, glbl_tmr_cnt);
    } else {
        qdma_log_error("%s: ST or MM cmpt not supported, err:%d\n",
                __func__,
                -QDMA_ERR_HWACC_FEATURE_NOT_SUPPORTED);
        return -QDMA_ERR_HWACC_FEATURE_NOT_SUPPORTED;
    }

    qdma_log_debug("%s: Successfully read timer values\n", __func__);
    return QDMA_SUCCESS;
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_write_global_buffer_sizes() - function to set the buffer sizes.
 *
 * This function writes the buffer sizes to the device using the provided indices
 * and count of entries. It checks for valid parameters and device capabilities
 * before writing the values.
 *
 * @param dev_hndl: Device handle.
 * @param index: Index from where the values need to be written.
 * @param count: Number of entries to be written.
 * @param glbl_buf_sz: Pointer to the array having the values to write.
 *
 * Note: (index + count) shall not exceed 16.
 *
 * Return: 0 for success and negative values for failure.
 *****************************************************************************/
static int eqdma_cpm5_write_global_buffer_sizes(void *dev_hndl, uint8_t index,
		uint8_t count, const uint32_t *glbl_buf_sz)
{
	struct qdma_dev_attributes dev_cap;

	// MD: Check for null pointers and zero count to ensure valid parameters.
	if (!dev_hndl || !glbl_buf_sz || !count) {
		qdma_log_error("%s: Invalid parameters: dev_hndl=%p, glbl_buf_sz=%p, count=%u, err:%d\n",
					   __func__, dev_hndl, glbl_buf_sz, count, -QDMA_ERR_INV_PARAM);
		return -QDMA_ERR_INV_PARAM;
	}

	// MD: Ensure the index and count do not exceed the maximum buffer sizes.
	if ((index + count) > QDMA_NUM_C2H_BUFFER_SIZES) {
		qdma_log_error("%s: Buffer overflow attempt: index=%u, count=%u, max=%d, err:%d\n",
					   __func__, index, count, QDMA_NUM_C2H_BUFFER_SIZES, -QDMA_ERR_INV_PARAM);
		return -QDMA_ERR_INV_PARAM;
	}

	// MD: Retrieve device capabilities to check if streaming is enabled.
	eqdma_cpm5_get_device_attributes(dev_hndl, &dev_cap);

	// MD: If streaming is enabled, write the buffer sizes to the device.
	if (dev_cap.st_en) {
		qdma_write_csr_values(dev_hndl, EQDMA_CPM5_C2H_BUF_SZ_ADDR, index, count, glbl_buf_sz);
	} else {
		// MD: Log error if streaming is not supported by the device.
		qdma_log_error("%s: Streaming not supported on this device, err:%d\n",
				__func__, -QDMA_ERR_HWACC_FEATURE_NOT_SUPPORTED);
		return -QDMA_ERR_HWACC_FEATURE_NOT_SUPPORTED;
	}

	return QDMA_SUCCESS;
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_read_global_buffer_sizes() - function to get the buffer sizes.
 *
 * This function reads the buffer sizes from the device using the provided indices
 * and count of entries. It checks for valid parameters and device capabilities
 * before reading the values.
 *
 * @param dev_hndl: Device handle.
 * @param index: Index from where the values need to be read.
 * @param count: Number of entries to be read.
 * @param glbl_buf_sz: Pointer to array to hold the values read.
 *
 * Note: (index + count) shall not exceed 16.
 *
 * Return: 0 for success and negative values for failure.
 *****************************************************************************/
static int eqdma_cpm5_read_global_buffer_sizes(void *dev_hndl, uint8_t index,
		uint8_t count, uint32_t *glbl_buf_sz)
{
	struct qdma_dev_attributes dev_cap;

	// MD: Check for null pointers and zero count to ensure valid parameters.
	if (!dev_hndl || !glbl_buf_sz || !count) {
		qdma_log_error("%s: Invalid parameters: dev_hndl=%p, glbl_buf_sz=%p, count=%u, err:%d\n",
					   __func__, dev_hndl, glbl_buf_sz, count, -QDMA_ERR_INV_PARAM);
		return -QDMA_ERR_INV_PARAM;
	}

	// MD: Ensure the index and count do not exceed the maximum buffer sizes.
	if ((index + count) > QDMA_NUM_C2H_BUFFER_SIZES) {
		qdma_log_error("%s: Buffer overflow attempt: index=%u, count=%u, max=%d, err:%d\n",
					   __func__, index, count, QDMA_NUM_C2H_BUFFER_SIZES, -QDMA_ERR_INV_PARAM);
		return -QDMA_ERR_INV_PARAM;
	}

	// MD: Retrieve device capabilities to check if streaming is enabled.
	eqdma_cpm5_get_device_attributes(dev_hndl, &dev_cap);

	// MD: If streaming is enabled, read the buffer sizes from the device.
	if (dev_cap.st_en) {
		qdma_read_csr_values(dev_hndl, EQDMA_CPM5_C2H_BUF_SZ_ADDR, index, count, glbl_buf_sz);
	} else {
		// MD: Log error if streaming is not supported by the device.
		qdma_log_error("%s: Streaming not supported on this device, err:%d\n",
					__func__, -QDMA_ERR_HWACC_FEATURE_NOT_SUPPORTED);
		return -QDMA_ERR_HWACC_FEATURE_NOT_SUPPORTED;
	}

	return QDMA_SUCCESS;
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_global_csr_conf() - function to configure global csr
 *
 * This function configures the global CSR (Control and Status Register) based on
 * the specified type and access method. It supports reading and writing operations
 * for different CSR types, ensuring that the parameters are valid and the operations
 * are supported.
 *
 * @param dev_hndl: Device handle.
 * @param index: Index from where the values need to be read.
 * @param count: Number of entries to be read.
 * @param csr_val: Pointer to the CSR value.
 * @param csr_type: Type of the CSR to configure (qdma_global_csr_type enum).
 * @param access_type: HW access type (qdma_hw_access_type enum).
 *                     Note: QDMA_HW_ACCESS_CLEAR and QDMA_HW_ACCESS_INVALIDATE are not supported.
 *
 * Note: (index + count) shall not exceed 16.
 *
 * Return: 0 for success and negative values for failure.
 *****************************************************************************/
int eqdma_cpm5_global_csr_conf(void *dev_hndl, uint8_t index,
				uint8_t count,
				uint32_t *csr_val,
				enum qdma_global_csr_type csr_type,
				enum qdma_hw_access_type access_type)
{
	int rv = QDMA_SUCCESS; // MD: Initialize return value to success

	// MD: Debug: Log entry into the function
	qdma_log_debug("%s: Entering function with csr_type=%d, access_type=%d\n",
				   __func__, csr_type, access_type);

	switch (csr_type) {
	case QDMA_CSR_RING_SZ:
		switch (access_type) {
		case QDMA_HW_ACCESS_READ:
			// MD: Read global ring sizes
			rv = eqdma_cpm5_read_global_ring_sizes(
						dev_hndl,
						index,
						count,
						csr_val);
			break;
		case QDMA_HW_ACCESS_WRITE:
			// MD: Write global ring sizes
			rv = eqdma_cpm5_write_global_ring_sizes(
						dev_hndl,
						index,
						count,
						csr_val);
			break;
		default:
			// MD: Log error for invalid access type
			qdma_log_error("%s: access_type(%d) invalid, err:%d\n",
							__func__,
							access_type,
						   -QDMA_ERR_INV_PARAM);
			rv = -QDMA_ERR_INV_PARAM;
			break;
		}
		break;
	case QDMA_CSR_TIMER_CNT:
		switch (access_type) {
		case QDMA_HW_ACCESS_READ:
			// MD: Read global timer count
			rv = eqdma_cpm5_read_global_timer_count(
						dev_hndl,
						index,
						count,
						csr_val);
			break;
		case QDMA_HW_ACCESS_WRITE:
			// MD: Write global timer count
			rv = eqdma_cpm5_write_global_timer_count(
						dev_hndl,
						index,
						count,
						csr_val);
			break;
		default:
			// MD: Log error for invalid access type
			qdma_log_error("%s: access_type(%d) invalid, err:%d\n",
							__func__,
							access_type,
						   -QDMA_ERR_INV_PARAM);
			rv = -QDMA_ERR_INV_PARAM;
			break;
		}
		break;
	case QDMA_CSR_CNT_TH:
		switch (access_type) {
		case QDMA_HW_ACCESS_READ:
			// MD: Read global counter threshold
			rv = eqdma_cpm5_read_global_counter_threshold(
						dev_hndl,
						index,
						count,
						csr_val);
			break;
		case QDMA_HW_ACCESS_WRITE:
			// MD: Write global counter threshold
			rv = eqdma_cpm5_write_global_counter_threshold(
						dev_hndl,
						index,
						count,
						csr_val);
			break;
		default:
			// MD: Log error for invalid access type
			qdma_log_error("%s: access_type(%d) invalid, err:%d\n",
							__func__,
							access_type,
						   -QDMA_ERR_INV_PARAM);
			rv = -QDMA_ERR_INV_PARAM;
			break;
		}
		break;
	case QDMA_CSR_BUF_SZ:
		switch (access_type) {
		case QDMA_HW_ACCESS_READ:
			// MD: Read global buffer sizes
			rv = eqdma_cpm5_read_global_buffer_sizes(dev_hndl,
						index,
						count,
						csr_val);
			break;
		case QDMA_HW_ACCESS_WRITE:
			// MD: Write global buffer sizes
			rv = eqdma_cpm5_write_global_buffer_sizes(dev_hndl,
						index,
						count,
						csr_val);
			break;
		default:
			// MD: Log error for invalid access type
			qdma_log_error("%s: access_type(%d) invalid, err:%d\n",
							__func__,
							access_type,
						   -QDMA_ERR_INV_PARAM);
			rv = -QDMA_ERR_INV_PARAM;
			break;
		}
		break;
	default:
		// MD: Log error for invalid CSR type
		qdma_log_error("%s: csr_type(%d) invalid, err:%d\n",
						__func__,
						csr_type,
					   -QDMA_ERR_INV_PARAM);
		rv = -QDMA_ERR_INV_PARAM;
		break;
	}

	// MD: Debug: Log exit from the function
	qdma_log_debug("%s: Exiting function with return value=%d\n", __func__, rv);

	return rv;
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_global_writeback_interval_write() - function to set the writeback interval
 *
 * This function sets the writeback interval for the device. It checks for valid
 * parameters and device capabilities before applying the configuration.
 *
 * @param dev_hndl: Device handle.
 * @param wb_int: Writeback Interval.
 *
 * Return: 0 for success and negative values for failure.
 *****************************************************************************/
static int eqdma_cpm5_global_writeback_interval_write(void *dev_hndl,
		enum qdma_wrb_interval wb_int)
{
	uint32_t reg_val; // MD: Register value to be modified
	struct qdma_dev_attributes dev_cap; // MD: Device capabilities

	// MD: Check for null device handle
	if (!dev_hndl) {
		qdma_log_error("%s: dev_handle is NULL, err:%d\n", __func__,
					   -QDMA_ERR_INV_PARAM);
		return -QDMA_ERR_INV_PARAM;
	}

	// MD: Check if the writeback interval is within valid range
	if (wb_int >= QDMA_NUM_WRB_INTERVALS) {
		qdma_log_error("%s: wb_int=%d is invalid, err:%d\n",
					   __func__, wb_int,
					   -QDMA_ERR_INV_PARAM);
		return -QDMA_ERR_INV_PARAM;
	}

	// MD: Retrieve device capabilities
	eqdma_cpm5_get_device_attributes(dev_hndl, &dev_cap);

	// MD: Check if streaming or memory-mapped completion is enabled
	if (dev_cap.st_en || dev_cap.mm_cmpt_en) {
		// MD: Read current register value
		reg_val = qdma_reg_read(dev_hndl, EQDMA_CPM5_GLBL_DSC_CFG_ADDR);
		// MD: Set the writeback interval in the register
		reg_val |= FIELD_SET(GLBL_DSC_CFG_WB_ACC_INT_MASK, wb_int);
		// MD: Write the updated register value back
		qdma_reg_write(dev_hndl, EQDMA_CPM5_GLBL_DSC_CFG_ADDR, reg_val);
	} else {
		// MD: Log error if neither streaming nor memory-mapped completion is supported
		qdma_log_error("%s: ST or MM cmpt not supported, err:%d\n",
			   __func__, -QDMA_ERR_HWACC_FEATURE_NOT_SUPPORTED);
		return -QDMA_ERR_HWACC_FEATURE_NOT_SUPPORTED;
	}

	return QDMA_SUCCESS;
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_global_writeback_interval_read() - function to get the writeback interval
 *
 * This function retrieves the current writeback interval setting from the device.
 * It checks for valid parameters and device capabilities before reading the value.
 *
 * @param dev_hndl: Device handle.
 * @param wb_int: Pointer to the data to hold Writeback Interval.
 *
 * Return: 0 for success and negative values for failure.
 *****************************************************************************/
static int eqdma_cpm5_global_writeback_interval_read(void *dev_hndl,
		enum qdma_wrb_interval *wb_int)
{
	uint32_t reg_val; // MD: Variable to store register value
	struct qdma_dev_attributes dev_cap; // MD: Structure to hold device capabilities

	// MD: Check for null device handle
	if (!dev_hndl) {
		qdma_log_error("%s: dev_handle is NULL, err:%d\n", __func__,
					   -QDMA_ERR_INV_PARAM);
		return -QDMA_ERR_INV_PARAM;
	}

	// MD: Check for null writeback interval pointer
	if (!wb_int) {
		qdma_log_error("%s: wb_int is NULL, err:%d\n", __func__,
					   -QDMA_ERR_INV_PARAM);
		return -QDMA_ERR_INV_PARAM;
	}

	// MD: Retrieve device capabilities
	eqdma_cpm5_get_device_attributes(dev_hndl, &dev_cap);

	// MD: Check if streaming or memory-mapped completion is enabled
	if (dev_cap.st_en || dev_cap.mm_cmpt_en) {
		// MD: Read the current register value
		reg_val = qdma_reg_read(dev_hndl, EQDMA_CPM5_GLBL_DSC_CFG_ADDR);
		// MD: Extract the writeback interval from the register value
		*wb_int = (enum qdma_wrb_interval)FIELD_GET(
				GLBL_DSC_CFG_WB_ACC_INT_MASK, reg_val);
	} else {
		// MD: Log error if neither streaming nor memory-mapped completion is supported
		qdma_log_error("%s: ST or MM cmpt not supported, err:%d\n",
			   __func__, -QDMA_ERR_HWACC_FEATURE_NOT_SUPPORTED);
		return -QDMA_ERR_HWACC_FEATURE_NOT_SUPPORTED;
	}

	return QDMA_SUCCESS;
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_global_writeback_interval_conf() - function to configure the writeback interval
 *
 * This function configures the writeback interval for the device based on the
 * specified access type. It supports reading and writing operations.
 *
 * @param dev_hndl: Device handle.
 * @param wb_int: Pointer to the data to hold Writeback Interval.
 * @param access_type: HW access type (qdma_hw_access_type enum).
 *                     Note: QDMA_HW_ACCESS_CLEAR and QDMA_HW_ACCESS_INVALIDATE are not supported.
 *
 * Return: 0 for success and negative values for failure.
 *****************************************************************************/
int eqdma_cpm5_global_writeback_interval_conf(void *dev_hndl,
				enum qdma_wrb_interval *wb_int,
				enum qdma_hw_access_type access_type)
{
	int rv = QDMA_SUCCESS; // MD: Initialize return value to success

	// MD: Debug: Log entry into the function
	qdma_log_debug("%s: Entering function with access_type=%d\n", __func__, access_type);

	switch (access_type) {
	case QDMA_HW_ACCESS_READ:
		// MD: Read the writeback interval
		rv = eqdma_cpm5_global_writeback_interval_read(dev_hndl, wb_int);
		break;
	case QDMA_HW_ACCESS_WRITE:
		// MD: Write the writeback interval
		rv = eqdma_cpm5_global_writeback_interval_write(dev_hndl, *wb_int);
		break;
	case QDMA_HW_ACCESS_CLEAR:
	case QDMA_HW_ACCESS_INVALIDATE:
	default:
		// MD: Log error for invalid access type
		qdma_log_error("%s: access_type(%d) invalid, err:%d\n",
						__func__,
						access_type,
					   -QDMA_ERR_INV_PARAM);
		rv = -QDMA_ERR_INV_PARAM;
		break;
	}

	// MD: Debug: Log exit from the function
	qdma_log_debug("%s: Exiting function with return value=%d\n", __func__, rv);

	return rv;
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_mm_channel_conf() - Function to enable/disable the MM channel
 *
 * This function enables or disables the Memory-Mapped (MM) channel based on the
 * specified parameters. It checks for valid parameters and device capabilities
 * before applying the configuration.
 *
 * @param dev_hndl: Device handle.
 * @param channel: MM channel number.
 * @param is_c2h: Queue direction. Set 1 for C2H and 0 for H2C.
 * @param enable: Enable or disable MM channel.
 *
 * Note: Presently, we have only 1 MM channel.
 *
 * Return: 0 for success and negative values for failure.
 *****************************************************************************/
int eqdma_cpm5_mm_channel_conf(void *dev_hndl, uint8_t channel,
		uint8_t is_c2h, uint8_t enable)
{
	uint32_t reg_addr = (is_c2h) ? EQDMA_CPM5_C2H_MM_CTL_ADDR : EQDMA_CPM5_H2C_MM_CTL_ADDR;
	struct qdma_dev_attributes dev_cap;

	// MD: Check for null device handle
	if (!dev_hndl) {
		qdma_log_error("%s: dev_handle is NULL, err:%d\n",
				__func__, -QDMA_ERR_INV_PARAM);
		return -QDMA_ERR_INV_PARAM;
	}

	// MD: Retrieve device capabilities
	eqdma_cpm5_get_device_attributes(dev_hndl, &dev_cap);

	// MD: Check if MM is enabled and configure the channel
	if (dev_cap.mm_en) {
		qdma_reg_write(dev_hndl, reg_addr + (channel * QDMA_MM_CONTROL_STEP), enable);
	}

	return QDMA_SUCCESS;
}

/* MD:****************************************************************************/
/* MD:*
 * eqdma_cpm5_dump_reg_info() - Function to dump register information
 *
 * This function retrieves and logs the register information for debugging purposes.
 * It checks for valid parameters and iterates through the register list to read
 * and log each register's value and bitfields.
 *
 * @param dev_hndl: Device handle.
 * @param reg_addr: Starting register address.
 * @param num_regs: Number of registers to read.
 * @param buf: Buffer to store the register information.
 * @param buflen: Length of the buffer.
 *
 * Return: Length of data written to buffer or negative values for failure.
 *****************************************************************************/
int eqdma_cpm5_dump_reg_info(void *dev_hndl, uint32_t reg_addr,
		uint32_t num_regs, char *buf, uint32_t buflen)
{
	uint32_t total_num_regs = eqdma_cpm5_config_num_regs_get();
	struct xreg_info *config_regs = eqdma_cpm5_config_regs_get();
	struct qdma_dev_attributes dev_cap;
	const char *bitfield_name;
	uint32_t i = 0, num_regs_idx = 0, k = 0, j = 0,
			bitfield = 0, lsb = 0, msb = 31;
	int rv = 0;
	uint32_t reg_val;
	uint32_t data_len = 0;

	// MD: Check for null device handle
	if (!dev_hndl) {
		qdma_log_error("%s: dev_handle is NULL, err:%d\n",
				__func__, -QDMA_ERR_INV_PARAM);
		return -QDMA_ERR_INV_PARAM;
	}

	// MD: Retrieve device capabilities
	eqdma_cpm5_get_device_attributes(dev_hndl, &dev_cap);

	// MD: Find the starting register in the configuration list
	for (i = 0; i < total_num_regs; i++) {
		if (reg_addr == config_regs[i].addr) {
			j = i;
			break;
		}
	}

	// MD: Check if the register was found
	if (i == total_num_regs) {
		qdma_log_error("%s: Register not found err:%d\n",
				__func__, -QDMA_ERR_INV_PARAM);
		if (buf)
			QDMA_SNPRINTF_S(buf, buflen,
					DEBGFS_LINE_SZ,
					"Register not found 0x%x\n", reg_addr);

		return -QDMA_ERR_INV_PARAM;
	}

	// MD: Determine the range of registers to read
	num_regs_idx = (j + num_regs < total_num_regs) ? (j + num_regs) : total_num_regs;

	// MD: Iterate through the registers and read their values
	for (; j < num_regs_idx; j++) {
		reg_val = qdma_reg_read(dev_hndl, config_regs[j].addr);

		// MD: Log or store the register value
		if (buf) {
			rv = QDMA_SNPRINTF_S(buf, buflen,
						DEBGFS_LINE_SZ,
						"\n%-40s 0x%-7x %-#10x %-10d\n",
						config_regs[j].name,
						config_regs[j].addr,
						reg_val, reg_val);
			if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
				qdma_log_error(
					"%s: Insufficient buffer, err:%d\n",
					__func__, -QDMA_ERR_NO_MEM);
				return -QDMA_ERR_NO_MEM;
			}
			buf += rv;
			data_len += rv;
			buflen -= rv;
		} else {
			qdma_log_info("%-40s 0x%-7x %-#10x %-10d\n",
						  config_regs[j].name,
						  config_regs[j].addr,
						  reg_val, reg_val);
		}

		// MD: Iterate through the bitfields of the register
		for (k = 0; k < config_regs[j].num_bitfields; k++) {
			// MD: Skip debug registers if debug mode is not enabled
			if (dev_cap.debug_mode == 0 && config_regs[j].is_debug_reg == 1)
				continue;

			bitfield = config_regs[j].bitfields[k].field_mask;
			bitfield_name = config_regs[i].bitfields[k].field_name;
			lsb = 0;
			msb = 31;

			// MD: Determine the least significant bit of the bitfield
			while (!(BIT(lsb) & bitfield))
				lsb++;

			// MD: Determine the most significant bit of the bitfield
			while (!(BIT(msb) & bitfield))
				msb--;

			// MD: Log or store the bitfield value
			if (msb != lsb) {
				if (buf) {
					rv = QDMA_SNPRINTF_S(buf, buflen,
							DEBGFS_LINE_SZ,
							"%-40s [%2u,%2u]   %#-10x\n",
							bitfield_name,
							msb, lsb,
							(reg_val & bitfield) >> lsb);
					if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
						qdma_log_error(
							"%s: Insufficient buffer, err:%d\n",
							__func__,
							-QDMA_ERR_NO_MEM);
						return -QDMA_ERR_NO_MEM;
					}
					buf += rv;
					data_len += rv;
					buflen -= rv;
				} else {
					qdma_log_info(
						"%-40s [%2u,%2u]   %#-10x\n",
						bitfield_name,
						msb, lsb,
						(reg_val & bitfield) >> lsb);
				}
			} else {
				if (buf) {
					rv = QDMA_SNPRINTF_S(buf, buflen,
							DEBGFS_LINE_SZ,
							"%-40s [%5u]   %#-10x\n",
							bitfield_name,
							lsb,
							(reg_val & bitfield) >> lsb);
					if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
						qdma_log_error(
							"%s: Insufficient buffer, err:%d\n",
							__func__,
							-QDMA_ERR_NO_MEM);
						return -QDMA_ERR_NO_MEM;
					}
					buf += rv;
					data_len += rv;
					buflen -= rv;
				} else {
					qdma_log_info(
						"%-40s [%5u]   %#-10x\n",
						bitfield_name,
						lsb,
						(reg_val & bitfield) >> lsb);
				}
			}
		}
	}

	return data_len;
}
