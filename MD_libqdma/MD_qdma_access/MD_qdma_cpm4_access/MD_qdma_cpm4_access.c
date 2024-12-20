#include <linux/kernel.h> // MD: For printk()

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

#include "qdma_cpm4_access.h"
#include "qdma_cpm4_reg.h"
#include "qdma_reg_dump.h"

#ifdef ENABLE_WPP_TRACING
#include "qdma_cpm4_access.tmh"
#endif

/* MD:* QDMA CPM4 Hard Context array size */
#define QDMA_CPM4_SW_CONTEXT_NUM_WORDS              4
#define QDMA_CPM4_CMPT_CONTEXT_NUM_WORDS            4
#define QDMA_CPM4_QID2VEC_CONTEXT_NUM_WORDS         1
#define QDMA_CPM4_HW_CONTEXT_NUM_WORDS              2
#define QDMA_CPM4_CR_CONTEXT_NUM_WORDS              1
#define QDMA_CPM4_IND_INTR_CONTEXT_NUM_WORDS        3
#define QDMA_CPM4_PFETCH_CONTEXT_NUM_WORDS          2

#define QDMA_CPM4_VF_USER_BAR_ID   2

#define QDMA_CPM4_REG_GROUP_1_START_ADDR	0x000
#define QDMA_CPM4_REG_GROUP_2_START_ADDR	0x400
#define QDMA_CPM4_REG_GROUP_3_START_ADDR	0xB00
#define QDMA_CPM4_REG_GROUP_4_START_ADDR	0x1014

#define QDMA_CPM4_REG_TRQ_SEL_FMAP_STEP	4

#define QDMA_CPM4_IND_CTXT_DATA_NUM_REGS	4

#define QDMA_CPM4_TOTAL_LEAF_ERROR_AGGREGATORS	7
#define QDMA_CPM4_GLBL_TRQ_ERR_ALL_MASK			0XB3
#define QDMA_CPM4_GLBL_DSC_ERR_ALL_MASK			0X1F9037E
#define QDMA_CPM4_C2H_ERR_ALL_MASK				0X3F6DF
#define QDMA_CPM4_C2H_FATAL_ERR_ALL_MASK			0X1FDF1B
#define QDMA_CPM4_H2C_ERR_ALL_MASK				0X3F
#define QDMA_CPM4_SBE_ERR_ALL_MASK				0XFFFFFFFF
#define QDMA_CPM4_DBE_ERR_ALL_MASK				0XFFFFFFFF

#define QDMA_CPM4_OFFSET_DMAP_SEL_INT_CIDX                  0x6400
#define QDMA_CPM4_OFFSET_DMAP_SEL_H2C_DSC_PIDX          0x6404
#define QDMA_CPM4_OFFSET_DMAP_SEL_C2H_DSC_PIDX          0x6408
#define QDMA_CPM4_OFFSET_DMAP_SEL_CMPT_CIDX               0x640C

#define QDMA_CPM4_OFFSET_VF_DMAP_SEL_INT_CIDX             0x3000
#define QDMA_CPM4_OFFSET_VF_DMAP_SEL_H2C_DSC_PIDX     0x3004
#define QDMA_CPM4_OFFSET_VF_DMAP_SEL_C2H_DSC_PIDX     0x3008
#define QDMA_CPM4_OFFSET_VF_DMAP_SEL_CMPT_CIDX          0x300C

#define QDMA_CPM4_DMA_SEL_INT_SW_CIDX_MASK               GENMASK(15, 0)
#define QDMA_CPM4_DMA_SEL_INT_RING_IDX_MASK              GENMASK(23, 16)
#define QDMA_CPM4_DMA_SEL_DESC_PIDX_MASK                   GENMASK(15, 0)
#define QDMA_CPM4_DMA_SEL_IRQ_EN_MASK                        BIT(16)
#define QDMA_CPM4_DMAP_SEL_CMPT_IRQ_EN_MASK             BIT(28)
#define QDMA_CPM4_DMAP_SEL_CMPT_STS_DESC_EN_MASK    BIT(27)
#define QDMA_CPM4_DMAP_SEL_CMPT_TRG_MODE_MASK        GENMASK(26, 24)
#define QDMA_CPM4_DMAP_SEL_CMPT_TMR_CNT_MASK          GENMASK(23, 20)
#define QDMA_CPM4_DMAP_SEL_CMPT_CNT_THRESH_MASK     GENMASK(19, 16)
#define QDMA_CPM4_DMAP_SEL_CMPT_WRB_CIDX_MASK        GENMASK(15, 0)
#define QDMA_CPM4_INTR_CTXT_BADDR_GET_H_MASK     GENMASK_ULL(63, 35)
#define QDMA_CPM4_INTR_CTXT_BADDR_GET_L_MASK     GENMASK_ULL(34, 12)
#define QDMA_CPM4_COMPL_CTXT_BADDR_GET_H_MASK    GENMASK_ULL(63, 42)
#define QDMA_CPM4_COMPL_CTXT_BADDR_GET_M_MASK    GENMASK_ULL(41, 10)
#define QDMA_CPM4_COMPL_CTXT_BADDR_GET_L_MASK    GENMASK_ULL(9, 6)
#define QDMA_CPM4_COMPL_CTXT_PIDX_GET_H_MASK     GENMASK(15, 8)
#define QDMA_CPM4_COMPL_CTXT_PIDX_GET_L_MASK     GENMASK(7, 0)
#define QDMA_CPM4_QID2VEC_H2C_VECTOR             GENMASK(16, 9)
#define QDMA_CPM4_QID2VEC_H2C_COAL_EN            BIT(17)

#define QDMA_CPM4_DEFAULT_PFCH_STOP_THRESH            256

static void qdma_cpm4_hw_st_h2c_err_process(void *dev_hndl)
{
    printk(KERN_DEBUG "Processing H2C error\n");
    // MD: Add error processing logic here
}

static void qdma_cpm4_hw_st_c2h_err_process(void *dev_hndl)
{
    printk(KERN_DEBUG "Processing C2H error\n");
    // MD: Add error processing logic here
}

static void qdma_cpm4_hw_desc_err_process(void *dev_hndl)
{
    printk(KERN_DEBUG "Processing descriptor error\n");
    // MD: Add error processing logic here
}

static void qdma_cpm4_hw_trq_err_process(void *dev_hndl)
{
    printk(KERN_DEBUG "Processing TRQ error\n");
    // MD: Add error processing logic here
}

static void qdma_cpm4_hw_ram_sbe_err_process(void *dev_hndl)
{
    printk(KERN_DEBUG "Processing RAM SBE error\n");
    // MD: Add error processing logic here
}

static void qdma_cpm4_hw_ram_dbe_err_process(void *dev_hndl)
{
    printk(KERN_DEBUG "Processing RAM DBE error\n");
    // MD: Add error processing logic here
}

// MD: Structure to hold QDMA CPM4 hardware error information
static struct qdma_cpm4_hw_err_info qdma_cpm4_err_info[QDMA_CPM4_ERRS_ALL] = {
    /* MD: Descriptor errors */
    {
        QDMA_CPM4_DSC_ERR_POISON, // MD: Error code for poison error
        "Poison error", // MD: Description of the error
        QDMA_CPM4_GLBL_DSC_ERR_MSK_ADDR, // MD: Address for global descriptor error mask
        QDMA_CPM4_GLBL_DSC_ERR_STS_ADDR, // MD: Address for global descriptor error status
        GLBL_DSC_ERR_STS_POISON_MASK, // MD: Mask for poison error status
        GLBL_ERR_STAT_ERR_DSC_MASK, // MD: Mask for descriptor error status
        &qdma_cpm4_hw_desc_err_process // MD: Function pointer to handle descriptor errors
    },
	{
		QDMA_CPM4_DSC_ERR_UR_CA,
		"Unsupported request or completer aborted error",
		QDMA_CPM4_GLBL_DSC_ERR_MSK_ADDR,
		QDMA_CPM4_GLBL_DSC_ERR_STS_ADDR,
		GLBL_DSC_ERR_STS_UR_CA_MASK,
		GLBL_ERR_STAT_ERR_DSC_MASK,
		&qdma_cpm4_hw_desc_err_process
	},
	{
		QDMA_CPM4_DSC_ERR_PARAM,
		"Parameter mismatch error",
		QDMA_CPM4_GLBL_DSC_ERR_MSK_ADDR,
		QDMA_CPM4_GLBL_DSC_ERR_STS_ADDR,
		GLBL_DSC_ERR_STS_PARAM_MASK,
		GLBL_ERR_STAT_ERR_DSC_MASK,
		&qdma_cpm4_hw_desc_err_process
	},
	{
		QDMA_CPM4_DSC_ERR_ADDR,
		"Address mismatch error",
		QDMA_CPM4_GLBL_DSC_ERR_MSK_ADDR,
		QDMA_CPM4_GLBL_DSC_ERR_STS_ADDR,
		GLBL_DSC_ERR_STS_ADDR_MASK,
		GLBL_ERR_STAT_ERR_DSC_MASK,
		&qdma_cpm4_hw_desc_err_process
	},
	{
		QDMA_CPM4_DSC_ERR_TAG,
		"Unexpected tag error",
		QDMA_CPM4_GLBL_DSC_ERR_MSK_ADDR,
		QDMA_CPM4_GLBL_DSC_ERR_STS_ADDR,
		GLBL_DSC_ERR_STS_TAG_MASK,
		GLBL_ERR_STAT_ERR_DSC_MASK,
		&qdma_cpm4_hw_desc_err_process
	},
	{
		QDMA_CPM4_DSC_ERR_FLR,
		"FLR error",
		QDMA_CPM4_GLBL_DSC_ERR_MSK_ADDR,
		QDMA_CPM4_GLBL_DSC_ERR_STS_ADDR,
		GLBL_DSC_ERR_STS_FLR_MASK,
		GLBL_ERR_STAT_ERR_DSC_MASK,
		&qdma_cpm4_hw_desc_err_process
	},
	{
		QDMA_CPM4_DSC_ERR_TIMEOUT,
		"Timed out error",
		QDMA_CPM4_GLBL_DSC_ERR_MSK_ADDR,
		QDMA_CPM4_GLBL_DSC_ERR_STS_ADDR,
		GLBL_DSC_ERR_STS_TIMEOUT_MASK,
		GLBL_ERR_STAT_ERR_DSC_MASK,
		&qdma_cpm4_hw_desc_err_process
	},
	{
		QDMA_CPM4_DSC_ERR_DAT_POISON,
		"Poison data error",
		QDMA_CPM4_GLBL_DSC_ERR_MSK_ADDR,
		QDMA_CPM4_GLBL_DSC_ERR_STS_ADDR,
		GLBL_DSC_ERR_STS_DAT_POISON_MASK,
		GLBL_ERR_STAT_ERR_DSC_MASK,
		&qdma_cpm4_hw_desc_err_process
	},
	{
		QDMA_CPM4_DSC_ERR_FLR_CANCEL,
		"Descriptor fetch cancelled due to FLR error",
		QDMA_CPM4_GLBL_DSC_ERR_MSK_ADDR,
		QDMA_CPM4_GLBL_DSC_ERR_STS_ADDR,
		GLBL_DSC_ERR_STS_FLR_CANCEL_MASK,
		GLBL_ERR_STAT_ERR_DSC_MASK,
		&qdma_cpm4_hw_desc_err_process
	},
	{
		QDMA_CPM4_DSC_ERR_DMA,
		"DMA engine error",
		QDMA_CPM4_GLBL_DSC_ERR_MSK_ADDR,
		QDMA_CPM4_GLBL_DSC_ERR_STS_ADDR,
		GLBL_DSC_ERR_STS_DMA_MASK,
		GLBL_ERR_STAT_ERR_DSC_MASK,
		&qdma_cpm4_hw_desc_err_process
	},
	{
		QDMA_CPM4_DSC_ERR_DSC,
		"Invalid PIDX update error",
		QDMA_CPM4_GLBL_DSC_ERR_MSK_ADDR,
		QDMA_CPM4_GLBL_DSC_ERR_STS_ADDR,
		GLBL_DSC_ERR_STS_DSC_MASK,
		GLBL_ERR_STAT_ERR_DSC_MASK,
		&qdma_cpm4_hw_desc_err_process
	},
	{
		QDMA_CPM4_DSC_ERR_RQ_CANCEL,
		"Descriptor fetch cancelled due to disable register status error",
		QDMA_CPM4_GLBL_DSC_ERR_MSK_ADDR,
		QDMA_CPM4_GLBL_DSC_ERR_STS_ADDR,
		GLBL_DSC_ERR_STS_RQ_CANCEL_MASK,
		GLBL_ERR_STAT_ERR_DSC_MASK,
		&qdma_cpm4_hw_desc_err_process
	},
	{
		QDMA_CPM4_DSC_ERR_DBE,
		"UNC_ERR_RAM_DBE error",
		QDMA_CPM4_GLBL_DSC_ERR_MSK_ADDR,
		QDMA_CPM4_GLBL_DSC_ERR_STS_ADDR,
		GLBL_DSC_ERR_STS_DBE_MASK,
		GLBL_ERR_STAT_ERR_DSC_MASK,
		&qdma_cpm4_hw_desc_err_process
	},
	{
		QDMA_CPM4_DSC_ERR_SBE,
		"UNC_ERR_RAM_SBE error",
		QDMA_CPM4_GLBL_DSC_ERR_MSK_ADDR,
		QDMA_CPM4_GLBL_DSC_ERR_STS_ADDR,
		GLBL_DSC_ERR_STS_SBE_MASK,
		GLBL_ERR_STAT_ERR_DSC_MASK,
		&qdma_cpm4_hw_desc_err_process
	},
	{
		QDMA_CPM4_DSC_ERR_ALL,
		"All Descriptor errors",
		QDMA_CPM4_GLBL_DSC_ERR_MSK_ADDR,
		QDMA_CPM4_GLBL_DSC_ERR_STS_ADDR,
		QDMA_CPM4_DBE_ERR_ALL_MASK,
		GLBL_ERR_STAT_ERR_DSC_MASK,
		&qdma_cpm4_hw_desc_err_process
	},

    /* MD: TRQ errors */
    {
        QDMA_CPM4_TRQ_ERR_UNMAPPED, // MD: Error code for unmapped register space access
        "Access targeted unmapped register space via CSR pathway error", // MD: Description of the error
        QDMA_CPM4_GLBL_TRQ_ERR_MSK_ADDR, // MD: Address for global TRQ error mask
        QDMA_CPM4_GLBL_TRQ_ERR_STS_ADDR, // MD: Address for global TRQ error status
        GLBL_TRQ_ERR_STS_UNMAPPED_MASK, // MD: Mask for unmapped register space error status
        GLBL_ERR_STAT_ERR_TRQ_MASK, // MD: Mask for TRQ error status
        &qdma_cpm4_hw_trq_err_process // MD: Function pointer to handle TRQ errors
    },
	{
		QDMA_CPM4_TRQ_ERR_QID_RANGE,
		"Qid range error",
		QDMA_CPM4_GLBL_TRQ_ERR_MSK_ADDR,
		QDMA_CPM4_GLBL_TRQ_ERR_STS_ADDR,
		GLBL_TRQ_ERR_STS_QID_RANGE_MASK,
		GLBL_ERR_STAT_ERR_TRQ_MASK,
		&qdma_cpm4_hw_trq_err_process
	},
	{
		QDMA_CPM4_TRQ_ERR_VF_ACCESS_ERR,
		"VF attempted to access Global register space or Function map",
		QDMA_CPM4_GLBL_TRQ_ERR_MSK_ADDR,
		QDMA_CPM4_GLBL_TRQ_ERR_STS_ADDR,
		GLBL_TRQ_ERR_STS_VF_ACCESS_ERR_MASK,
		GLBL_ERR_STAT_ERR_TRQ_MASK,
		&qdma_cpm4_hw_trq_err_process
	},
	{
		QDMA_CPM4_TRQ_ERR_TCP_TIMEOUT,
		"Timeout on request to dma internal csr register",
		QDMA_CPM4_GLBL_TRQ_ERR_MSK_ADDR,
		QDMA_CPM4_GLBL_TRQ_ERR_STS_ADDR,
		GLBL_TRQ_ERR_STS_TCP_TIMEOUT_MASK,
		GLBL_ERR_STAT_ERR_TRQ_MASK,
		&qdma_cpm4_hw_trq_err_process
	},
	{
		QDMA_CPM4_TRQ_ERR_ALL,
		"All TRQ errors",
		QDMA_CPM4_GLBL_TRQ_ERR_MSK_ADDR,
		QDMA_CPM4_GLBL_TRQ_ERR_STS_ADDR,
		QDMA_CPM4_GLBL_TRQ_ERR_ALL_MASK,
		GLBL_ERR_STAT_ERR_TRQ_MASK,
		&qdma_cpm4_hw_trq_err_process
	},

	/* MD: C2H Errors*/
	{
		QDMA_CPM4_ST_C2H_ERR_MTY_MISMATCH,
		"MTY mismatch error",
		QDMA_CPM4_C2H_ERR_MASK_ADDR,
		QDMA_CPM4_C2H_ERR_STAT_ADDR,
		C2H_ERR_STAT_MTY_MISMATCH_MASK,
		GLBL_ERR_STAT_ERR_C2H_ST_MASK,
		&qdma_cpm4_hw_st_c2h_err_process
	},
	{
		QDMA_CPM4_ST_C2H_ERR_LEN_MISMATCH,
		"Packet length mismatch error",
		QDMA_CPM4_C2H_ERR_MASK_ADDR,
		QDMA_CPM4_C2H_ERR_STAT_ADDR,
		C2H_ERR_STAT_LEN_MISMATCH_MASK,
		GLBL_ERR_STAT_ERR_C2H_ST_MASK,
		&qdma_cpm4_hw_st_c2h_err_process
	},
	{
		QDMA_CPM4_ST_C2H_ERR_QID_MISMATCH,
		"Qid mismatch error",
		QDMA_CPM4_C2H_ERR_MASK_ADDR,
		QDMA_CPM4_C2H_ERR_STAT_ADDR,
		C2H_ERR_STAT_QID_MISMATCH_MASK,
		GLBL_ERR_STAT_ERR_C2H_ST_MASK,
		&qdma_cpm4_hw_st_c2h_err_process
	},
	{
		QDMA_CPM4_ST_C2H_ERR_DESC_RSP_ERR,
		"Descriptor error bit set",
		QDMA_CPM4_C2H_ERR_MASK_ADDR,
		QDMA_CPM4_C2H_ERR_STAT_ADDR,
		C2H_ERR_STAT_DESC_RSP_ERR_MASK,
		GLBL_ERR_STAT_ERR_C2H_ST_MASK,
		&qdma_cpm4_hw_st_c2h_err_process
	},
	{
		QDMA_CPM4_ST_C2H_ERR_ENG_WPL_DATA_PAR_ERR,
		"Data parity error",
		QDMA_CPM4_C2H_ERR_MASK_ADDR,
		QDMA_CPM4_C2H_ERR_STAT_ADDR,
		C2H_ERR_STAT_ENG_WPL_DATA_PAR_ERR_MASK,
		GLBL_ERR_STAT_ERR_C2H_ST_MASK,
		&qdma_cpm4_hw_st_c2h_err_process
	},
	{
		QDMA_CPM4_ST_C2H_ERR_MSI_INT_FAIL,
		"MSI got a fail response error",
		QDMA_CPM4_C2H_ERR_MASK_ADDR,
		QDMA_CPM4_C2H_ERR_STAT_ADDR,
		C2H_ERR_STAT_MSI_INT_FAIL_MASK,
		GLBL_ERR_STAT_ERR_C2H_ST_MASK,
		&qdma_cpm4_hw_st_c2h_err_process
	},
	{
		QDMA_CPM4_ST_C2H_ERR_ERR_DESC_CNT,
		"Descriptor count error",
		QDMA_CPM4_C2H_ERR_MASK_ADDR,
		QDMA_CPM4_C2H_ERR_STAT_ADDR,
		C2H_ERR_STAT_ERR_DESC_CNT_MASK,
		GLBL_ERR_STAT_ERR_C2H_ST_MASK,
		&qdma_cpm4_hw_st_c2h_err_process
	},
	{
		QDMA_CPM4_ST_C2H_ERR_PORTID_CTXT_MISMATCH,
		"Port id in packet and pfetch ctxt mismatch error",
		QDMA_CPM4_C2H_ERR_MASK_ADDR,
		QDMA_CPM4_C2H_ERR_STAT_ADDR,
		C2H_ERR_STAT_PORT_ID_CTXT_MISMATCH_MASK,
		GLBL_ERR_STAT_ERR_C2H_ST_MASK,
		&qdma_cpm4_hw_st_c2h_err_process
	},
	{
		QDMA_CPM4_ST_C2H_ERR_PORTID_BYP_IN_MISMATCH,
		"Port id in packet and bypass in mismatch error",
		QDMA_CPM4_C2H_ERR_MASK_ADDR,
		QDMA_CPM4_C2H_ERR_STAT_ADDR,
		C2H_ERR_STAT_PORT_ID_CTXT_MISMATCH_MASK,
		GLBL_ERR_STAT_ERR_C2H_ST_MASK,
		&qdma_cpm4_hw_st_c2h_err_process
	},
	{
		QDMA_CPM4_ST_C2H_ERR_WRB_INV_Q_ERR,
		"Writeback on invalid queue error",
		QDMA_CPM4_C2H_ERR_MASK_ADDR,
		QDMA_CPM4_C2H_ERR_STAT_ADDR,
		C2H_ERR_STAT_WRB_INV_Q_ERR_MASK,
		GLBL_ERR_STAT_ERR_C2H_ST_MASK,
		&qdma_cpm4_hw_st_c2h_err_process
	},
	{
		QDMA_CPM4_ST_C2H_ERR_WRB_QFULL_ERR,
		"Completion queue gets full error",
		QDMA_CPM4_C2H_ERR_MASK_ADDR,
		QDMA_CPM4_C2H_ERR_STAT_ADDR,
		C2H_ERR_STAT_WRB_QFULL_ERR_MASK,
		GLBL_ERR_STAT_ERR_C2H_ST_MASK,
		&qdma_cpm4_hw_st_c2h_err_process
	},
	{
		QDMA_CPM4_ST_C2H_ERR_WRB_CIDX_ERR,
		"Bad CIDX update by the software error",
		QDMA_CPM4_C2H_ERR_MASK_ADDR,
		QDMA_CPM4_C2H_ERR_STAT_ADDR,
		C2H_ERR_STAT_WRB_CIDX_ERR_MASK,
		GLBL_ERR_STAT_ERR_C2H_ST_MASK,
		&qdma_cpm4_hw_st_c2h_err_process
	},
	{
		QDMA_CPM4_ST_C2H_ERR_WRB_PRTY_ERR,
		"C2H completion Parity error",
		QDMA_CPM4_C2H_ERR_MASK_ADDR,
		QDMA_CPM4_C2H_ERR_STAT_ADDR,
		C2H_ERR_STAT_WRB_PRTY_ERR_MASK,
		GLBL_ERR_STAT_ERR_C2H_ST_MASK,
		&qdma_cpm4_hw_st_c2h_err_process
	},
	{
		QDMA_CPM4_ST_C2H_ERR_ALL,
		"All C2h errors",
		QDMA_CPM4_C2H_ERR_MASK_ADDR,
		QDMA_CPM4_C2H_ERR_STAT_ADDR,
		QDMA_CPM4_C2H_ERR_ALL_MASK,
		GLBL_ERR_STAT_ERR_C2H_ST_MASK,
		&qdma_cpm4_hw_st_c2h_err_process
	},

	/* MD: C2H fatal errors */
	{
		QDMA_CPM4_ST_FATAL_ERR_MTY_MISMATCH,
		"Fatal MTY mismatch error",
		QDMA_CPM4_C2H_FATAL_ERR_MASK_ADDR,
		QDMA_CPM4_C2H_FATAL_ERR_STAT_ADDR,
		C2H_FATAL_ERR_STAT_MTY_MISMATCH_MASK,
		GLBL_ERR_STAT_ERR_C2H_ST_MASK,
		&qdma_cpm4_hw_st_c2h_err_process
	},
	{
		QDMA_CPM4_ST_FATAL_ERR_LEN_MISMATCH,
		"Fatal Len mismatch error",
		QDMA_CPM4_C2H_FATAL_ERR_MASK_ADDR,
		QDMA_CPM4_C2H_FATAL_ERR_STAT_ADDR,
		C2H_FATAL_ERR_STAT_LEN_MISMATCH_MASK,
		GLBL_ERR_STAT_ERR_C2H_ST_MASK,
		&qdma_cpm4_hw_st_c2h_err_process
	},
	{
		QDMA_CPM4_ST_FATAL_ERR_QID_MISMATCH,
		"Fatal Qid mismatch error",
		QDMA_CPM4_C2H_FATAL_ERR_MASK_ADDR,
		QDMA_CPM4_C2H_FATAL_ERR_STAT_ADDR,
		C2H_FATAL_ERR_STAT_QID_MISMATCH_MASK,
		GLBL_ERR_STAT_ERR_C2H_ST_MASK,
		&qdma_cpm4_hw_st_c2h_err_process
	},
	{
		QDMA_CPM4_ST_FATAL_ERR_TIMER_FIFO_RAM_RDBE,
		"RAM double bit fatal error",
		QDMA_CPM4_C2H_FATAL_ERR_MASK_ADDR,
		QDMA_CPM4_C2H_FATAL_ERR_STAT_ADDR,
		C2H_FATAL_ERR_STAT_TIMER_FIFO_RAM_RDBE_MASK,
		GLBL_ERR_STAT_ERR_C2H_ST_MASK,
		&qdma_cpm4_hw_st_c2h_err_process
	},
	{
		QDMA_CPM4_ST_FATAL_ERR_PFCH_II_RAM_RDBE,
		"RAM double bit fatal error",
		QDMA_CPM4_C2H_FATAL_ERR_MASK_ADDR,
		QDMA_CPM4_C2H_FATAL_ERR_STAT_ADDR,
		C2H_FATAL_ERR_STAT_PFCH_LL_RAM_RDBE_MASK,
		GLBL_ERR_STAT_ERR_C2H_ST_MASK,
		&qdma_cpm4_hw_st_c2h_err_process
	},
	{
		QDMA_CPM4_ST_FATAL_ERR_WRB_CTXT_RAM_RDBE,
		"RAM double bit fatal error",
		QDMA_CPM4_C2H_FATAL_ERR_MASK_ADDR,
		QDMA_CPM4_C2H_FATAL_ERR_STAT_ADDR,
		C2H_FATAL_ERR_STAT_WRB_CTXT_RAM_RDBE_MASK,
		GLBL_ERR_STAT_ERR_C2H_ST_MASK,
		&qdma_cpm4_hw_st_c2h_err_process
	},
	{
		QDMA_CPM4_ST_FATAL_ERR_PFCH_CTXT_RAM_RDBE,
		"RAM double bit fatal error",
		QDMA_CPM4_C2H_FATAL_ERR_MASK_ADDR,
		QDMA_CPM4_C2H_FATAL_ERR_STAT_ADDR,
		C2H_FATAL_ERR_STAT_PFCH_CTXT_RAM_RDBE_MASK,
		GLBL_ERR_STAT_ERR_C2H_ST_MASK,
		&qdma_cpm4_hw_st_c2h_err_process
	},
	{
		QDMA_CPM4_ST_FATAL_ERR_DESC_REQ_FIFO_RAM_RDBE,
		"RAM double bit fatal error",
		QDMA_CPM4_C2H_FATAL_ERR_MASK_ADDR,
		QDMA_CPM4_C2H_FATAL_ERR_STAT_ADDR,
		C2H_FATAL_ERR_STAT_DESC_REQ_FIFO_RAM_RDBE_MASK,
		GLBL_ERR_STAT_ERR_C2H_ST_MASK,
		&qdma_cpm4_hw_st_c2h_err_process
	},
	{
		QDMA_CPM4_ST_FATAL_ERR_INT_CTXT_RAM_RDBE,
		"RAM double bit fatal error",
		QDMA_CPM4_C2H_FATAL_ERR_MASK_ADDR,
		QDMA_CPM4_C2H_FATAL_ERR_STAT_ADDR,
		C2H_FATAL_ERR_STAT_INT_CTXT_RAM_RDBE_MASK,
		GLBL_ERR_STAT_ERR_C2H_ST_MASK,
		&qdma_cpm4_hw_st_c2h_err_process
	},
	{
		QDMA_CPM4_ST_FATAL_ERR_INT_QID2VEC_RAM_RDBE,
		"RAM double bit fatal error",
		QDMA_CPM4_C2H_FATAL_ERR_MASK_ADDR,
		QDMA_CPM4_C2H_FATAL_ERR_STAT_ADDR,
		C2H_FATAL_ERR_STAT_INT_QID2VEC_RAM_RDBE_MASK,
		GLBL_ERR_STAT_ERR_C2H_ST_MASK,
		&qdma_cpm4_hw_st_c2h_err_process
	},
	{
		QDMA_CPM4_ST_FATAL_ERR_WRB_COAL_DATA_RAM_RDBE,
		"RAM double bit fatal error",
		QDMA_CPM4_C2H_FATAL_ERR_MASK_ADDR,
		QDMA_CPM4_C2H_FATAL_ERR_STAT_ADDR,
		C2H_FATAL_ERR_STAT_WRB_COAL_DATA_RAM_RDBE_MASK,
		GLBL_ERR_STAT_ERR_C2H_ST_MASK,
		&qdma_cpm4_hw_st_c2h_err_process
	},
	{
		QDMA_CPM4_ST_FATAL_ERR_TUSER_FIFO_RAM_RDBE,
		"RAM double bit fatal error",
		QDMA_CPM4_C2H_FATAL_ERR_MASK_ADDR,
		QDMA_CPM4_C2H_FATAL_ERR_STAT_ADDR,
		C2H_FATAL_ERR_STAT_TUSER_FIFO_RAM_RDBE_MASK,
		GLBL_ERR_STAT_ERR_C2H_ST_MASK,
		&qdma_cpm4_hw_st_c2h_err_process
	},
	{
		QDMA_CPM4_ST_FATAL_ERR_QID_FIFO_RAM_RDBE,
		"RAM double bit fatal error",
		QDMA_CPM4_C2H_FATAL_ERR_MASK_ADDR,
		QDMA_CPM4_C2H_FATAL_ERR_STAT_ADDR,
		C2H_FATAL_ERR_STAT_QID_FIFO_RAM_RDBE_MASK,
		GLBL_ERR_STAT_ERR_C2H_ST_MASK,
		&qdma_cpm4_hw_st_c2h_err_process
	},
	{
		QDMA_CPM4_ST_FATAL_ERR_PAYLOAD_FIFO_RAM_RDBE,
		"RAM double bit fatal error",
		QDMA_CPM4_C2H_FATAL_ERR_MASK_ADDR,
		QDMA_CPM4_C2H_FATAL_ERR_STAT_ADDR,
		C2H_FATAL_ERR_STAT_PLD_FIFO_RAM_RDBE_MASK,
		GLBL_ERR_STAT_ERR_C2H_ST_MASK,
		&qdma_cpm4_hw_st_c2h_err_process
	},
	{
		QDMA_CPM4_ST_FATAL_ERR_WPL_DATA_PAR_ERR,
		"RAM double bit fatal error",
		QDMA_CPM4_C2H_FATAL_ERR_MASK_ADDR,
		QDMA_CPM4_C2H_FATAL_ERR_STAT_ADDR,
		C2H_FATAL_ERR_STAT_WPL_DATA_PAR_ERR_MASK,
		GLBL_ERR_STAT_ERR_C2H_ST_MASK,
		&qdma_cpm4_hw_st_c2h_err_process
	},
	{
		QDMA_CPM4_ST_FATAL_ERR_ALL,
		"All fatal errors",
		QDMA_CPM4_C2H_FATAL_ERR_MASK_ADDR,
		QDMA_CPM4_C2H_FATAL_ERR_STAT_ADDR,
		QDMA_CPM4_C2H_FATAL_ERR_ALL_MASK,
		GLBL_ERR_STAT_ERR_C2H_ST_MASK,
		&qdma_cpm4_hw_st_c2h_err_process
	},

	/* MD: H2C St errors */
	{
		QDMA_CPM4_ST_H2C_ERR_ZERO_LEN_DESC_ERR,
		"Zero length descriptor error",
		QDMA_CPM4_H2C_ERR_MASK_ADDR,
		QDMA_CPM4_H2C_ERR_STAT_ADDR,
		H2C_ERR_STAT_ZERO_LEN_DS_MASK,
		GLBL_ERR_STAT_ERR_H2C_ST_MASK,
		&qdma_cpm4_hw_st_h2c_err_process
	},
	{
		QDMA_CPM4_ST_H2C_ERR_SDI_MRKR_REQ_MOP_ERR,
		"A non-EOP descriptor received",
		QDMA_CPM4_H2C_ERR_MASK_ADDR,
		QDMA_CPM4_H2C_ERR_STAT_ADDR,
		H2C_ERR_STAT_SDI_MRKR_REQ_MOP_ERR_MASK,
		GLBL_ERR_STAT_ERR_H2C_ST_MASK,
		&qdma_cpm4_hw_st_h2c_err_process
	},
	{
		QDMA_CPM4_ST_H2C_ERR_NO_DMA_DSC,
		"No DMA descriptor received error",
		QDMA_CPM4_H2C_ERR_MASK_ADDR,
		QDMA_CPM4_H2C_ERR_STAT_ADDR,
		H2C_ERR_STAT_NO_DMA_DS_MASK,
		GLBL_ERR_STAT_ERR_H2C_ST_MASK,
		&qdma_cpm4_hw_st_h2c_err_process
	},
	{
		QDMA_CPM4_ST_H2C_ERR_DBE,
		"Double bit error detected on H2C-ST data error",
		QDMA_CPM4_H2C_ERR_MASK_ADDR,
		QDMA_CPM4_H2C_ERR_STAT_ADDR,
		H2C_ERR_STAT_DBE_MASK,
		GLBL_ERR_STAT_ERR_H2C_ST_MASK,
		&qdma_cpm4_hw_st_h2c_err_process
	},
	{
		QDMA_CPM4_ST_H2C_ERR_SBE,
		"Single bit error detected on H2C-ST data error",
		QDMA_CPM4_H2C_ERR_MASK_ADDR,
		QDMA_CPM4_H2C_ERR_STAT_ADDR,
		H2C_ERR_STAT_SBE_MASK,
		GLBL_ERR_STAT_ERR_H2C_ST_MASK,
		&qdma_cpm4_hw_st_h2c_err_process
	},
	{
		QDMA_CPM4_ST_H2C_ERR_ALL,
		"All H2C errors",
		QDMA_CPM4_H2C_ERR_MASK_ADDR,
		QDMA_CPM4_H2C_ERR_STAT_ADDR,
		QDMA_CPM4_H2C_ERR_ALL_MASK,
		GLBL_ERR_STAT_ERR_H2C_ST_MASK,
		&qdma_cpm4_hw_st_h2c_err_process
	},

	/* MD: SBE errors */
	{
		QDMA_CPM4_SBE_ERR_MI_H2C0_DAT,
		"H2C MM data buffer single bit ECC error",
		QDMA_CPM4_RAM_SBE_MSK_A_ADDR,
		QDMA_CPM4_RAM_SBE_STS_A_ADDR,
		RAM_SBE_STS_A_MI_H2C0_DAT_MASK,
		GLBL_ERR_STAT_ERR_RAM_SBE_MASK,
		&qdma_cpm4_hw_ram_sbe_err_process
	},
	{
		QDMA_CPM4_SBE_ERR_MI_C2H0_DAT,
		"C2H MM data buffer single bit ECC error",
		QDMA_CPM4_RAM_SBE_MSK_A_ADDR,
		QDMA_CPM4_RAM_SBE_STS_A_ADDR,
		RAM_SBE_STS_A_MI_C2H0_DAT_MASK,
		GLBL_ERR_STAT_ERR_RAM_SBE_MASK,
		&qdma_cpm4_hw_ram_sbe_err_process
	},
	{
		QDMA_CPM4_SBE_ERR_H2C_RD_BRG_DAT,
		"Bridge master read single bit ECC error",
		QDMA_CPM4_RAM_SBE_MSK_A_ADDR,
		QDMA_CPM4_RAM_SBE_STS_A_ADDR,
		RAM_SBE_STS_A_H2C_RD_BRG_DAT_MASK,
		GLBL_ERR_STAT_ERR_RAM_SBE_MASK,
		&qdma_cpm4_hw_ram_sbe_err_process
	},
	{
		QDMA_CPM4_SBE_ERR_H2C_WR_BRG_DAT,
		"Bridge master write single bit ECC error",
		QDMA_CPM4_RAM_SBE_MSK_A_ADDR,
		QDMA_CPM4_RAM_SBE_STS_A_ADDR,
		RAM_SBE_STS_A_H2C_WR_BRG_DAT_MASK,
		GLBL_ERR_STAT_ERR_RAM_SBE_MASK,
		&qdma_cpm4_hw_ram_sbe_err_process
	},
	{
		QDMA_CPM4_SBE_ERR_C2H_RD_BRG_DAT,
		"Bridge slave read data buffer single bit ECC error",
		QDMA_CPM4_RAM_SBE_MSK_A_ADDR,
		QDMA_CPM4_RAM_SBE_STS_A_ADDR,
		RAM_SBE_STS_A_C2H_RD_BRG_DAT_MASK,
		GLBL_ERR_STAT_ERR_RAM_SBE_MASK,
		&qdma_cpm4_hw_ram_sbe_err_process
	},
	{
		QDMA_CPM4_SBE_ERR_C2H_WR_BRG_DAT,
		"Bridge slave write data buffer single bit ECC error",
		QDMA_CPM4_RAM_SBE_MSK_A_ADDR,
		QDMA_CPM4_RAM_SBE_STS_A_ADDR,
		RAM_SBE_STS_A_C2H_WR_BRG_DAT_MASK,
		GLBL_ERR_STAT_ERR_RAM_SBE_MASK,
		&qdma_cpm4_hw_ram_sbe_err_process
	},
	{
		QDMA_CPM4_SBE_ERR_FUNC_MAP,
		"Function map RAM single bit ECC error",
		QDMA_CPM4_RAM_SBE_MSK_A_ADDR,
		QDMA_CPM4_RAM_SBE_STS_A_ADDR,
		RAM_SBE_STS_A_FUNC_MAP_MASK,
		GLBL_ERR_STAT_ERR_RAM_SBE_MASK,
		&qdma_cpm4_hw_ram_sbe_err_process
	},
	{
		QDMA_CPM4_SBE_ERR_DSC_HW_CTXT,
		"Descriptor engine hardware context RAM single bit ECC error",
		QDMA_CPM4_RAM_SBE_MSK_A_ADDR,
		QDMA_CPM4_RAM_SBE_STS_A_ADDR,
		RAM_SBE_STS_A_DSC_HW_CTXT_MASK,
		GLBL_ERR_STAT_ERR_RAM_SBE_MASK,
		&qdma_cpm4_hw_ram_sbe_err_process
	},
	{
		QDMA_CPM4_SBE_ERR_DSC_CRD_RCV,
		"Descriptor engine receive credit context RAM single bit ECC error",
		QDMA_CPM4_RAM_SBE_MSK_A_ADDR,
		QDMA_CPM4_RAM_SBE_STS_A_ADDR,
		RAM_SBE_STS_A_DSC_CRD_RCV_MASK,
		GLBL_ERR_STAT_ERR_RAM_SBE_MASK,
		&qdma_cpm4_hw_ram_sbe_err_process
	},
	{
		QDMA_CPM4_SBE_ERR_DSC_SW_CTXT,
		"Descriptor engine software context RAM single bit ECC error",
		QDMA_CPM4_RAM_SBE_MSK_A_ADDR,
		QDMA_CPM4_RAM_SBE_STS_A_ADDR,
		RAM_SBE_STS_A_DSC_SW_CTXT_MASK,
		GLBL_ERR_STAT_ERR_RAM_SBE_MASK,
		&qdma_cpm4_hw_ram_sbe_err_process
	},
	{
		QDMA_CPM4_SBE_ERR_DSC_CPLI,
		"Descriptor engine fetch completion information RAM single bit ECC error",
		QDMA_CPM4_RAM_SBE_MSK_A_ADDR,
		QDMA_CPM4_RAM_SBE_STS_A_ADDR,
		RAM_SBE_STS_A_DSC_CPLI_MASK,
		GLBL_ERR_STAT_ERR_RAM_SBE_MASK,
		&qdma_cpm4_hw_ram_sbe_err_process
	},
	{
		QDMA_CPM4_SBE_ERR_DSC_CPLD,
		"Descriptor engine fetch completion data RAM single bit ECC error",
		QDMA_CPM4_RAM_SBE_MSK_A_ADDR,
		QDMA_CPM4_RAM_SBE_STS_A_ADDR,
		RAM_SBE_STS_A_DSC_CPLD_MASK,
		GLBL_ERR_STAT_ERR_RAM_SBE_MASK,
		&qdma_cpm4_hw_ram_sbe_err_process
	},
	{
		QDMA_CPM4_SBE_ERR_PASID_CTXT_RAM,
		"Pasid ctxt FIFO RAM single bit ECC error",
		QDMA_CPM4_RAM_SBE_MSK_A_ADDR,
		QDMA_CPM4_RAM_SBE_STS_A_ADDR,
		RAM_SBE_STS_A_PASID_CTXT_RAM_MASK,
		GLBL_ERR_STAT_ERR_RAM_SBE_MASK,
		&qdma_cpm4_hw_ram_sbe_err_process
	},
	{
		QDMA_CPM4_SBE_ERR_TIMER_FIFO_RAM,
		"Timer fifo RAM single bit ECC error",
		QDMA_CPM4_RAM_SBE_MSK_A_ADDR,
		QDMA_CPM4_RAM_SBE_STS_A_ADDR,
		RAM_SBE_STS_A_TIMER_FIFO_RAM_MASK,
		GLBL_ERR_STAT_ERR_RAM_SBE_MASK,
		&qdma_cpm4_hw_ram_sbe_err_process
	},
	{
		QDMA_CPM4_SBE_ERR_PAYLOAD_FIFO_RAM,
		"C2H ST payload FIFO RAM single bit ECC error",
		QDMA_CPM4_RAM_SBE_MSK_A_ADDR,
		QDMA_CPM4_RAM_SBE_STS_A_ADDR,
		RAM_SBE_STS_A_PLD_FIFO_RAM_MASK,
		GLBL_ERR_STAT_ERR_RAM_SBE_MASK,
		&qdma_cpm4_hw_ram_sbe_err_process
	},
	{
		QDMA_CPM4_SBE_ERR_QID_FIFO_RAM,
		"C2H ST QID FIFO RAM single bit ECC error",
		QDMA_CPM4_RAM_SBE_MSK_A_ADDR,
		QDMA_CPM4_RAM_SBE_STS_A_ADDR,
		RAM_SBE_STS_A_QID_FIFO_RAM_MASK,
		GLBL_ERR_STAT_ERR_RAM_SBE_MASK,
		&qdma_cpm4_hw_ram_sbe_err_process
	},
	{
		QDMA_CPM4_SBE_ERR_TUSER_FIFO_RAM,
		"C2H ST TUSER FIFO RAM single bit ECC error",
		QDMA_CPM4_RAM_SBE_MSK_A_ADDR,
		QDMA_CPM4_RAM_SBE_STS_A_ADDR,
		RAM_SBE_STS_A_TUSER_FIFO_RAM_MASK,
		GLBL_ERR_STAT_ERR_RAM_SBE_MASK,
		&qdma_cpm4_hw_ram_sbe_err_process
	},
	{
		QDMA_CPM4_SBE_ERR_WRB_COAL_DATA_RAM,
		"Writeback Coalescing RAM single bit ECC error",
		QDMA_CPM4_RAM_SBE_MSK_A_ADDR,
		QDMA_CPM4_RAM_SBE_STS_A_ADDR,
		RAM_SBE_STS_A_WRB_COAL_DATA_RAM_MASK,
		GLBL_ERR_STAT_ERR_RAM_SBE_MASK,
		&qdma_cpm4_hw_ram_sbe_err_process
	},
	{
		QDMA_CPM4_SBE_ERR_INT_QID2VEC_RAM,
		"Interrupt QID2VEC RAM single bit ECC error",
		QDMA_CPM4_RAM_SBE_MSK_A_ADDR,
		QDMA_CPM4_RAM_SBE_STS_A_ADDR,
		RAM_SBE_STS_A_INT_QID2VEC_RAM_MASK,
		GLBL_ERR_STAT_ERR_RAM_SBE_MASK,
		&qdma_cpm4_hw_ram_sbe_err_process
	},
	{
		QDMA_CPM4_SBE_ERR_INT_CTXT_RAM,
		"Interrupt context RAM single bit ECC error",
		QDMA_CPM4_RAM_SBE_MSK_A_ADDR,
		QDMA_CPM4_RAM_SBE_STS_A_ADDR,
		RAM_SBE_STS_A_INT_CTXT_RAM_MASK,
		GLBL_ERR_STAT_ERR_RAM_SBE_MASK,
		&qdma_cpm4_hw_ram_sbe_err_process
	},
	{
		QDMA_CPM4_SBE_ERR_DESC_REQ_FIFO_RAM,
		"C2H ST descriptor request RAM single bit ECC error",
		QDMA_CPM4_RAM_SBE_MSK_A_ADDR,
		QDMA_CPM4_RAM_SBE_STS_A_ADDR,
		RAM_SBE_STS_A_DESC_REQ_FIFO_RAM_MASK,
		GLBL_ERR_STAT_ERR_RAM_SBE_MASK,
		&qdma_cpm4_hw_ram_sbe_err_process
	},
	{
		QDMA_CPM4_SBE_ERR_PFCH_CTXT_RAM,
		"C2H ST prefetch RAM single bit ECC error",
		QDMA_CPM4_RAM_SBE_MSK_A_ADDR,
		QDMA_CPM4_RAM_SBE_STS_A_ADDR,
		RAM_SBE_STS_A_PFCH_CTXT_RAM_MASK,
		GLBL_ERR_STAT_ERR_RAM_SBE_MASK,
		&qdma_cpm4_hw_ram_sbe_err_process
	},
	{
		QDMA_CPM4_SBE_ERR_WRB_CTXT_RAM,
		"C2H ST completion context RAM single bit ECC error",
		QDMA_CPM4_RAM_SBE_MSK_A_ADDR,
		QDMA_CPM4_RAM_SBE_STS_A_ADDR,
		RAM_SBE_STS_A_WRB_CTXT_RAM_MASK,
		GLBL_ERR_STAT_ERR_RAM_SBE_MASK,
		&qdma_cpm4_hw_ram_sbe_err_process
	},
	{
		QDMA_CPM4_SBE_ERR_PFCH_LL_RAM,
		"C2H ST prefetch list RAM single bit ECC error",
		QDMA_CPM4_RAM_SBE_MSK_A_ADDR,
		QDMA_CPM4_RAM_SBE_STS_A_ADDR,
		RAM_SBE_STS_A_PFCH_LL_RAM_MASK,
		GLBL_ERR_STAT_ERR_RAM_SBE_MASK,
		&qdma_cpm4_hw_ram_sbe_err_process
	},
	{
		QDMA_CPM4_SBE_ERR_ALL,
		"All SBE errors",
		QDMA_CPM4_RAM_SBE_MSK_A_ADDR,
		QDMA_CPM4_RAM_SBE_STS_A_ADDR,
		QDMA_CPM4_SBE_ERR_ALL_MASK,
		GLBL_ERR_STAT_ERR_RAM_SBE_MASK,
		&qdma_cpm4_hw_ram_sbe_err_process
	},


	/* MD: DBE errors */
	{
		QDMA_CPM4_DBE_ERR_MI_H2C0_DAT,
		"H2C MM data buffer single bit ECC error",
		QDMA_CPM4_RAM_DBE_MSK_A_ADDR,
		QDMA_CPM4_RAM_DBE_STS_A_ADDR,
		RAM_DBE_STS_A_MI_H2C0_DAT_MASK,
		GLBL_ERR_STAT_ERR_RAM_DBE_MASK,
		&qdma_cpm4_hw_ram_dbe_err_process
	},
	{
		QDMA_CPM4_DBE_ERR_MI_C2H0_DAT,
		"C2H MM data buffer single bit ECC error",
		QDMA_CPM4_RAM_DBE_MSK_A_ADDR,
		QDMA_CPM4_RAM_DBE_STS_A_ADDR,
		RAM_DBE_STS_A_MI_C2H0_DAT_MASK,
		GLBL_ERR_STAT_ERR_RAM_DBE_MASK,
		&qdma_cpm4_hw_ram_dbe_err_process
	},
	{
		QDMA_CPM4_DBE_ERR_H2C_RD_BRG_DAT,
		"Bridge master read single bit ECC error",
		QDMA_CPM4_RAM_DBE_MSK_A_ADDR,
		QDMA_CPM4_RAM_DBE_STS_A_ADDR,
		RAM_DBE_STS_A_H2C_RD_BRG_DAT_MASK,
		GLBL_ERR_STAT_ERR_RAM_DBE_MASK,
		&qdma_cpm4_hw_ram_dbe_err_process
	},
	{
		QDMA_CPM4_DBE_ERR_H2C_WR_BRG_DAT,
		"Bridge master write single bit ECC error",
		QDMA_CPM4_RAM_DBE_MSK_A_ADDR,
		QDMA_CPM4_RAM_DBE_STS_A_ADDR,
		RAM_DBE_STS_A_H2C_WR_BRG_DAT_MASK,
		GLBL_ERR_STAT_ERR_RAM_DBE_MASK,
		&qdma_cpm4_hw_ram_dbe_err_process
	},
	{
		QDMA_CPM4_DBE_ERR_C2H_RD_BRG_DAT,
		"Bridge slave read data buffer single bit ECC error",
		QDMA_CPM4_RAM_DBE_MSK_A_ADDR,
		QDMA_CPM4_RAM_DBE_STS_A_ADDR,
		RAM_DBE_STS_A_C2H_RD_BRG_DAT_MASK,
		GLBL_ERR_STAT_ERR_RAM_DBE_MASK,
		&qdma_cpm4_hw_ram_dbe_err_process
	},
	{
		QDMA_CPM4_DBE_ERR_C2H_WR_BRG_DAT,
		"Bridge slave write data buffer single bit ECC error",
		QDMA_CPM4_RAM_DBE_MSK_A_ADDR,
		QDMA_CPM4_RAM_DBE_STS_A_ADDR,
		RAM_DBE_STS_A_C2H_WR_BRG_DAT_MASK,
		GLBL_ERR_STAT_ERR_RAM_DBE_MASK,
		&qdma_cpm4_hw_ram_dbe_err_process
	},
	{
		QDMA_CPM4_DBE_ERR_FUNC_MAP,
		"Function map RAM single bit ECC error",
		QDMA_CPM4_RAM_DBE_MSK_A_ADDR,
		QDMA_CPM4_RAM_DBE_STS_A_ADDR,
		RAM_DBE_STS_A_FUNC_MAP_MASK,
		GLBL_ERR_STAT_ERR_RAM_DBE_MASK,
		&qdma_cpm4_hw_ram_dbe_err_process
	},
	{
		QDMA_CPM4_DBE_ERR_DSC_HW_CTXT,
		"Descriptor engine hardware context RAM single bit ECC error",
		QDMA_CPM4_RAM_DBE_MSK_A_ADDR,
		QDMA_CPM4_RAM_DBE_STS_A_ADDR,
		RAM_DBE_STS_A_DSC_HW_CTXT_MASK,
		GLBL_ERR_STAT_ERR_RAM_DBE_MASK,
		&qdma_cpm4_hw_ram_dbe_err_process
	},
	{
		QDMA_CPM4_DBE_ERR_DSC_CRD_RCV,
		"Descriptor engine receive credit context RAM single bit ECC error",
		QDMA_CPM4_RAM_DBE_MSK_A_ADDR,
		QDMA_CPM4_RAM_DBE_STS_A_ADDR,
		RAM_DBE_STS_A_DSC_CRD_RCV_MASK,
		GLBL_ERR_STAT_ERR_RAM_DBE_MASK,
		&qdma_cpm4_hw_ram_dbe_err_process
	},
	{
		QDMA_CPM4_DBE_ERR_DSC_SW_CTXT,
		"Descriptor engine software context RAM single bit ECC error",
		QDMA_CPM4_RAM_DBE_MSK_A_ADDR,
		QDMA_CPM4_RAM_DBE_STS_A_ADDR,
		RAM_DBE_STS_A_DSC_SW_CTXT_MASK,
		GLBL_ERR_STAT_ERR_RAM_DBE_MASK,
		&qdma_cpm4_hw_ram_dbe_err_process
	},
	{
		QDMA_CPM4_DBE_ERR_DSC_CPLI,
		"Descriptor engine fetch completion information RAM single bit ECC error",
		QDMA_CPM4_RAM_DBE_MSK_A_ADDR,
		QDMA_CPM4_RAM_DBE_STS_A_ADDR,
		RAM_DBE_STS_A_DSC_CPLI_MASK,
		GLBL_ERR_STAT_ERR_RAM_DBE_MASK,
		&qdma_cpm4_hw_ram_dbe_err_process
	},
	{
		QDMA_CPM4_DBE_ERR_DSC_CPLD,
		"Descriptor engine fetch completion data RAM single bit ECC error",
		QDMA_CPM4_RAM_DBE_MSK_A_ADDR,
		QDMA_CPM4_RAM_DBE_STS_A_ADDR,
		RAM_DBE_STS_A_DSC_CPLD_MASK,
		GLBL_ERR_STAT_ERR_RAM_DBE_MASK,
		&qdma_cpm4_hw_ram_dbe_err_process
	},
	{
		QDMA_CPM4_DBE_ERR_PASID_CTXT_RAM,
		"PASID CTXT RAM single bit ECC error",
		QDMA_CPM4_RAM_DBE_MSK_A_ADDR,
		QDMA_CPM4_RAM_DBE_STS_A_ADDR,
		RAM_DBE_STS_A_PASID_CTXT_RAM_MASK,
		GLBL_ERR_STAT_ERR_RAM_DBE_MASK,
		&qdma_cpm4_hw_ram_dbe_err_process
	},
	{
		QDMA_CPM4_DBE_ERR_TIMER_FIFO_RAM,
		"Timer fifo RAM single bit ECC error",
		QDMA_CPM4_RAM_DBE_MSK_A_ADDR,
		QDMA_CPM4_RAM_DBE_STS_A_ADDR,
		RAM_DBE_STS_A_TIMER_FIFO_RAM_MASK,
		GLBL_ERR_STAT_ERR_RAM_DBE_MASK,
		&qdma_cpm4_hw_ram_dbe_err_process
	},
	{
		QDMA_CPM4_DBE_ERR_PAYLOAD_FIFO_RAM,
		"Payload fifo RAM single bit ECC error",
		QDMA_CPM4_RAM_DBE_MSK_A_ADDR,
		QDMA_CPM4_RAM_DBE_STS_A_ADDR,
		RAM_DBE_STS_A_PLD_FIFO_RAM_MASK,
		GLBL_ERR_STAT_ERR_RAM_DBE_MASK,
		&qdma_cpm4_hw_ram_dbe_err_process
	},
	{
		QDMA_CPM4_DBE_ERR_QID_FIFO_RAM,
		"C2H ST QID FIFO RAM single bit ECC error",
		QDMA_CPM4_RAM_DBE_MSK_A_ADDR,
		QDMA_CPM4_RAM_DBE_STS_A_ADDR,
		RAM_DBE_STS_A_QID_FIFO_RAM_MASK,
		GLBL_ERR_STAT_ERR_RAM_DBE_MASK,
		&qdma_cpm4_hw_ram_dbe_err_process
	},
	{
		QDMA_CPM4_DBE_ERR_WRB_COAL_DATA_RAM,
		"Writeback Coalescing RAM single bit ECC error",
		QDMA_CPM4_RAM_DBE_MSK_A_ADDR,
		QDMA_CPM4_RAM_DBE_STS_A_ADDR,
		RAM_DBE_STS_A_WRB_COAL_DATA_RAM_MASK,
		GLBL_ERR_STAT_ERR_RAM_DBE_MASK,
		&qdma_cpm4_hw_ram_dbe_err_process
	},
	{
		QDMA_CPM4_DBE_ERR_INT_QID2VEC_RAM,
		"QID2VEC RAM single bit ECC error",
		QDMA_CPM4_RAM_DBE_MSK_A_ADDR,
		QDMA_CPM4_RAM_DBE_STS_A_ADDR,
		RAM_DBE_STS_A_INT_QID2VEC_RAM_MASK,
		GLBL_ERR_STAT_ERR_RAM_DBE_MASK,
		&qdma_cpm4_hw_ram_dbe_err_process
	},
	{
		QDMA_CPM4_DBE_ERR_INT_CTXT_RAM,
		"Interrupt context RAM single bit ECC error",
		QDMA_CPM4_RAM_DBE_MSK_A_ADDR,
		QDMA_CPM4_RAM_DBE_STS_A_ADDR,
		RAM_DBE_STS_A_INT_CTXT_RAM_MASK,
		GLBL_ERR_STAT_ERR_RAM_DBE_MASK,
		&qdma_cpm4_hw_ram_dbe_err_process
	},
	{
		QDMA_CPM4_DBE_ERR_DESC_REQ_FIFO_RAM,
		"C2H ST descriptor request RAM single bit ECC error",
		QDMA_CPM4_RAM_DBE_MSK_A_ADDR,
		QDMA_CPM4_RAM_DBE_STS_A_ADDR,
		RAM_DBE_STS_A_DESC_REQ_FIFO_RAM_MASK,
		GLBL_ERR_STAT_ERR_RAM_DBE_MASK,
		&qdma_cpm4_hw_ram_dbe_err_process
	},
	{
		QDMA_CPM4_DBE_ERR_PFCH_CTXT_RAM,
		"C2H ST prefetch RAM single bit ECC error",
		QDMA_CPM4_RAM_DBE_MSK_A_ADDR,
		QDMA_CPM4_RAM_DBE_STS_A_ADDR,
		RAM_DBE_STS_A_PFCH_CTXT_RAM_MASK,
		GLBL_ERR_STAT_ERR_RAM_DBE_MASK,
		&qdma_cpm4_hw_ram_dbe_err_process
	},
	{
		QDMA_CPM4_DBE_ERR_WRB_CTXT_RAM,
		"C2H ST completion context RAM single bit ECC error",
		QDMA_CPM4_RAM_DBE_MSK_A_ADDR,
		QDMA_CPM4_RAM_DBE_STS_A_ADDR,
		RAM_DBE_STS_A_WRB_CTXT_RAM_MASK,
		GLBL_ERR_STAT_ERR_RAM_DBE_MASK,
		&qdma_cpm4_hw_ram_dbe_err_process
	},
	{
		QDMA_CPM4_DBE_ERR_PFCH_LL_RAM,
		"C2H ST prefetch list RAM single bit ECC error",
		QDMA_CPM4_RAM_DBE_MSK_A_ADDR,
		QDMA_CPM4_RAM_DBE_STS_A_ADDR,
		RAM_DBE_STS_A_PFCH_LL_RAM_MASK,
		GLBL_ERR_STAT_ERR_RAM_DBE_MASK,
		&qdma_cpm4_hw_ram_dbe_err_process
	},
	{
		QDMA_CPM4_DBE_ERR_ALL,
		"All DBE errors",
		QDMA_CPM4_RAM_DBE_MSK_A_ADDR,
		QDMA_CPM4_RAM_DBE_STS_A_ADDR,
		QDMA_CPM4_DBE_ERR_ALL_MASK,
		GLBL_ERR_STAT_ERR_RAM_DBE_MASK,
		&qdma_cpm4_hw_ram_dbe_err_process
	}
};

#include <linux/kernel.h> // MD: For printk()

static int32_t all_qdma_cpm4_hw_errs[
    QDMA_CPM4_TOTAL_LEAF_ERROR_AGGREGATORS] = {
    QDMA_CPM4_DSC_ERR_ALL,
    QDMA_CPM4_TRQ_ERR_ALL,
    QDMA_CPM4_ST_C2H_ERR_ALL,
    QDMA_CPM4_ST_FATAL_ERR_ALL,
    QDMA_CPM4_ST_H2C_ERR_ALL,
    QDMA_CPM4_SBE_ERR_ALL,
    QDMA_CPM4_DBE_ERR_ALL
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized all_qdma_cpm4_hw_errs\n");

union qdma_cpm4_ind_ctxt_cmd {
    uint32_t word;
    struct {
        uint32_t busy:1;
        uint32_t sel:4;
        uint32_t op:2;
        uint32_t qid:11;
        uint32_t rsvd:14;
    } bits;
};

struct qdma_cpm4_indirect_ctxt_regs {
    uint32_t qdma_ind_ctxt_data[QDMA_CPM4_IND_CTXT_DATA_NUM_REGS];
    uint32_t qdma_ind_ctxt_mask[QDMA_CPM4_IND_CTXT_DATA_NUM_REGS];
    union qdma_cpm4_ind_ctxt_cmd cmd;
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized qdma_cpm4_indirect_ctxt_regs\n");

static struct qctx_entry qdma_cpm4_sw_ctxt_entries[] = {
    {"PIDX", 0},
    {"IRQ Arm", 0},
    {"Queue Enable", 0},
    {"Fetch Credit Enable", 0},
    {"Write back/Intr Check", 0},
    {"Write back/Intr Interval", 0},
    {"Function Id", 0},
    {"Ring Size", 0},
    {"Descriptor Size", 0},
    {"Bypass Enable", 0},
    {"MM Channel", 0},
    {"Writeback Enable", 0},
    {"Interrupt Enable", 0},
    {"Port Id", 0},
    {"Interrupt No Last", 0},
    {"Error", 0},
    {"Writeback Error Sent", 0},
    {"IRQ Request", 0},
    {"Marker Disable", 0},
    {"Is Memory Mapped", 0},
    {"Descriptor Ring Base Addr (Low)", 0},
    {"Descriptor Ring Base Addr (High)", 0},
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized qdma_cpm4_sw_ctxt_entries\n");

static struct qctx_entry qdma_cpm4_hw_ctxt_entries[] = {
    {"CIDX", 0},
    {"Credits Consumed", 0},
    {"Descriptors Pending", 0},
    {"Queue Invalid No Desc Pending", 0},
    {"Eviction Pending", 0},
    {"Fetch Pending", 0},
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized qdma_cpm4_hw_ctxt_entries\n");

static struct qctx_entry qdma_cpm4_credit_ctxt_entries[] = {
    {"Credit", 0},
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized qdma_cpm4_credit_ctxt_entries\n");

static struct qctx_entry qdma_cpm4_fmap_ctxt_entries[] = {
    {"Queue Base", 0},
    {"Queue Max", 0},
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized qdma_cpm4_fmap_ctxt_entries\n");

static struct qctx_entry qdma_cpm4_cmpt_ctxt_entries[] = {
    {"Enable Status Desc Update", 0},
    {"Enable Interrupt", 0},
    {"Trigger Mode", 0},
    {"Function Id", 0},
    {"Counter Index", 0},
    {"Timer Index", 0},
    {"Interrupt State", 0},
    {"Color", 0},
    {"Ring Size", 0},
    {"Base Address (Low)", 0},
    {"Base Address (High)", 0},
    {"Descriptor Size", 0},
    {"PIDX", 0},
    {"CIDX", 0},
    {"Valid", 0},
    {"Error", 0},
    {"Trigger Pending", 0},
    {"Timer Running", 0},
    {"Full Update", 0},
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized qdma_cpm4_cmpt_ctxt_entries\n");

static struct qctx_entry qdma_cpm4_c2h_pftch_ctxt_entries[] = {
    {"Bypass", 0},
    {"Buffer Size Index", 0},
    {"Port Id", 0},
    {"Error", 0},
    {"Prefetch Enable", 0},
    {"In Prefetch", 0},
    {"Software Credit", 0},
    {"Valid", 0},
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized qdma_cpm4_c2h_pftch_ctxt_entries\n");

static struct qctx_entry qdma_cpm4_qid2vec_ctxt_entries[] = {
    {"c2h_vector", 0},
    {"c2h_en_coal", 0},
    {"h2c_vector", 0},
    {"h2c_en_coal", 0},
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized qdma_cpm4_qid2vec_ctxt_entries\n");

static struct qctx_entry qdma_cpm4_ind_intr_ctxt_entries[] = {
    {"valid", 0},
    {"vec", 0},
    {"int_st", 0},
    {"color", 0},
    {"baddr_4k (Low)", 0},
    {"baddr_4k (High)", 0},
    {"page_size", 0},
    {"pidx", 0},
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized qdma_cpm4_ind_intr_ctxt_entries\n");

static int qdma_cpm4_indirect_reg_invalidate(void *dev_hndl,
        enum ind_ctxt_cmd_sel sel, uint16_t hw_qid) {
    printk(KERN_DEBUG "Invalidating indirect register for QID: %u\n", hw_qid);
    // MD: Function implementation
    return 0;
}

static int qdma_cpm4_indirect_reg_clear(void *dev_hndl,
        enum ind_ctxt_cmd_sel sel, uint16_t hw_qid) {
    printk(KERN_DEBUG "Clearing indirect register for QID: %u\n", hw_qid);
    // MD: Function implementation
    return 0;
}

static int qdma_cpm4_indirect_reg_read(void *dev_hndl,
        enum ind_ctxt_cmd_sel sel,
        uint16_t hw_qid, uint32_t cnt, uint32_t *data) {
    printk(KERN_DEBUG "Reading indirect register for QID: %u, Count: %u\n", hw_qid, cnt);
    // MD: Function implementation
    return 0;
}

static int qdma_cpm4_indirect_reg_write(void *dev_hndl,
        enum ind_ctxt_cmd_sel sel,
        uint16_t hw_qid, uint32_t *data, uint16_t cnt) {
    printk(KERN_DEBUG "Writing to indirect register for QID: %u, Count: %u\n", hw_qid, cnt);
    // MD: Function implementation
    return 0;
}

uint32_t qdma_cpm4_get_config_num_regs(void) {
    uint32_t num_regs = qdma_cpm4_config_num_regs_get();
    printk(KERN_DEBUG "Number of QDMA CPM4 configuration registers: %u\n", num_regs);
    return num_regs;
}

struct xreg_info *qdma_cpm4_get_config_regs(void) {
    printk(KERN_DEBUG "Returning pointer to QDMA CPM4 configuration registers\n");
    return qdma_cpm4_config_regs_get();
}

uint32_t qdma_cpm4_reg_dump_buf_len(void) {
    uint32_t length = (qdma_cpm4_config_num_regs_get() + 1) * REG_DUMP_SIZE_PER_LINE;
    printk(KERN_DEBUG "QDMA CPM4 register dump buffer length: %u\n", length);
    return length;
}

#include <stdio.h> // MD: Include necessary headers for printk

int qdma_cpm4_context_buf_len(uint8_t st,
		enum qdma_dev_q_type q_type, uint32_t *req_buflen)
{
	uint32_t len = 0; // MD: Initialize buffer length
	int rv = 0; // MD: Return value

	// MD: Check if the queue type is completion
	if (q_type == QDMA_DEV_Q_TYPE_CMPT) {
		// MD: Calculate length for completion context entries
		len += (((sizeof(qdma_cpm4_cmpt_ctxt_entries) /
			sizeof(qdma_cpm4_cmpt_ctxt_entries[0])) + 1) *
			REG_DUMP_SIZE_PER_LINE);
		printk(KERN_DEBUG "Calculated length for CMPT context: %u\n", len);
	} else {
		// MD: Calculate length for software context entries
		len += (((sizeof(qdma_cpm4_sw_ctxt_entries) /
				sizeof(qdma_cpm4_sw_ctxt_entries[0])) + 1) *
				REG_DUMP_SIZE_PER_LINE);

		// MD: Calculate length for hardware context entries
		len += (((sizeof(qdma_cpm4_hw_ctxt_entries) /
			sizeof(qdma_cpm4_hw_ctxt_entries[0])) + 1) *
			REG_DUMP_SIZE_PER_LINE);

		// MD: Calculate length for credit context entries
		len += (((sizeof(qdma_cpm4_credit_ctxt_entries) /
			sizeof(qdma_cpm4_credit_ctxt_entries[0])) + 1) *
			REG_DUMP_SIZE_PER_LINE);

		// MD: Calculate length for fmap context entries
		len += (((sizeof(qdma_cpm4_fmap_ctxt_entries) /
			sizeof(qdma_cpm4_fmap_ctxt_entries[0])) + 1) *
			REG_DUMP_SIZE_PER_LINE);

		// MD: Additional length calculation for C2H queue type
		if (st && (q_type == QDMA_DEV_Q_TYPE_C2H)) {
			len += (((sizeof(qdma_cpm4_cmpt_ctxt_entries) /
			sizeof(qdma_cpm4_cmpt_ctxt_entries[0])) + 1) *
			REG_DUMP_SIZE_PER_LINE);

			len += (((sizeof(qdma_cpm4_c2h_pftch_ctxt_entries) /
			sizeof(qdma_cpm4_c2h_pftch_ctxt_entries[0])) + 1) *
			REG_DUMP_SIZE_PER_LINE);
		}
		printk(KERN_DEBUG "Calculated length for SW, HW, Credit, FMAP, and C2H contexts: %u\n", len);
	}

	*req_buflen = len; // MD: Set the requested buffer length
	return rv; // MD: Return success
}

static uint32_t qdma_cpm4_intr_context_buf_len(void)
{
	uint32_t len = 0; // MD: Initialize buffer length

	// MD: Calculate length for interrupt context entries
	len += (((sizeof(qdma_cpm4_ind_intr_ctxt_entries) /
			sizeof(qdma_cpm4_ind_intr_ctxt_entries[0])) + 1) *
			REG_DUMP_SIZE_PER_LINE);
	printk(KERN_DEBUG "Calculated length for interrupt context: %u\n", len);
	return len; // MD: Return calculated length
}

/* MD:
 * qdma_acc_fill_sw_ctxt() - Helper function to fill sw context into structure
 */
static void qdma_cpm4_fill_sw_ctxt(struct qdma_descq_sw_ctxt *sw_ctxt)
{
	// MD: Fill software context entries with values from sw_ctxt
	qdma_cpm4_sw_ctxt_entries[0].value = sw_ctxt->pidx;
	qdma_cpm4_sw_ctxt_entries[1].value = sw_ctxt->irq_arm;
	qdma_cpm4_sw_ctxt_entries[2].value = sw_ctxt->qen;
	qdma_cpm4_sw_ctxt_entries[3].value = sw_ctxt->frcd_en;
	qdma_cpm4_sw_ctxt_entries[4].value = sw_ctxt->wbi_chk;
	qdma_cpm4_sw_ctxt_entries[5].value = sw_ctxt->wbi_intvl_en;
	qdma_cpm4_sw_ctxt_entries[6].value = sw_ctxt->fnc_id;
	qdma_cpm4_sw_ctxt_entries[7].value = sw_ctxt->rngsz_idx;
	qdma_cpm4_sw_ctxt_entries[8].value = sw_ctxt->desc_sz;
	qdma_cpm4_sw_ctxt_entries[9].value = sw_ctxt->bypass;
	qdma_cpm4_sw_ctxt_entries[10].value = sw_ctxt->mm_chn;
	qdma_cpm4_sw_ctxt_entries[11].value = sw_ctxt->wbk_en;
	qdma_cpm4_sw_ctxt_entries[12].value = sw_ctxt->irq_en;
	qdma_cpm4_sw_ctxt_entries[13].value = sw_ctxt->port_id;
	qdma_cpm4_sw_ctxt_entries[14].value = sw_ctxt->irq_no_last;
	qdma_cpm4_sw_ctxt_entries[15].value = sw_ctxt->err;
	qdma_cpm4_sw_ctxt_entries[16].value = sw_ctxt->err_wb_sent;
	qdma_cpm4_sw_ctxt_entries[17].value = sw_ctxt->irq_req;
	qdma_cpm4_sw_ctxt_entries[18].value = sw_ctxt->mrkr_dis;
	qdma_cpm4_sw_ctxt_entries[19].value = sw_ctxt->is_mm;
	qdma_cpm4_sw_ctxt_entries[20].value =
			sw_ctxt->ring_bs_addr & 0xFFFFFFFF;
	qdma_cpm4_sw_ctxt_entries[21].value =
		(sw_ctxt->ring_bs_addr >> 32) & 0xFFFFFFFF;
	printk(KERN_DEBUG "Filled software context\n");
}

/* MD:
 * qdma_acc_fill_cmpt_ctxt() - Helper function to fill completion context
 *                         into structure
 */
static void qdma_cpm4_fill_cmpt_ctxt(struct qdma_descq_cmpt_ctxt *cmpt_ctxt)
{
	// MD: Fill completion context entries with values from cmpt_ctxt
	qdma_cpm4_cmpt_ctxt_entries[0].value = cmpt_ctxt->en_stat_desc;
	qdma_cpm4_cmpt_ctxt_entries[1].value = cmpt_ctxt->en_int;
	qdma_cpm4_cmpt_ctxt_entries[2].value = cmpt_ctxt->trig_mode;
	qdma_cpm4_cmpt_ctxt_entries[3].value = cmpt_ctxt->fnc_id;
	qdma_cpm4_cmpt_ctxt_entries[4].value = cmpt_ctxt->counter_idx;
	qdma_cpm4_cmpt_ctxt_entries[5].value = cmpt_ctxt->timer_idx;
	qdma_cpm4_cmpt_ctxt_entries[6].value = cmpt_ctxt->in_st;
	qdma_cpm4_cmpt_ctxt_entries[7].value = cmpt_ctxt->color;
	qdma_cpm4_cmpt_ctxt_entries[8].value = cmpt_ctxt->ringsz_idx;
	qdma_cpm4_cmpt_ctxt_entries[9].value =
			cmpt_ctxt->bs_addr & 0xFFFFFFFF;
	qdma_cpm4_cmpt_ctxt_entries[10].value =
		(cmpt_ctxt->bs_addr >> 32) & 0xFFFFFFFF;
	qdma_cpm4_cmpt_ctxt_entries[11].value = cmpt_ctxt->desc_sz;
	qdma_cpm4_cmpt_ctxt_entries[12].value = cmpt_ctxt->pidx;
	qdma_cpm4_cmpt_ctxt_entries[13].value = cmpt_ctxt->cidx;
	qdma_cpm4_cmpt_ctxt_entries[14].value = cmpt_ctxt->valid;
	qdma_cpm4_cmpt_ctxt_entries[15].value = cmpt_ctxt->err;
	qdma_cpm4_cmpt_ctxt_entries[16].value = cmpt_ctxt->user_trig_pend;
	qdma_cpm4_cmpt_ctxt_entries[17].value = cmpt_ctxt->timer_running;
	qdma_cpm4_cmpt_ctxt_entries[18].value = cmpt_ctxt->full_upd;
	printk(KERN_DEBUG "Filled completion context\n");
}

/* MD:
 * qdma_acc_fill_hw_ctxt() - Helper function to fill HW context into structure
 */
static void qdma_cpm4_fill_hw_ctxt(struct qdma_descq_hw_ctxt *hw_ctxt)
{
	// MD: Fill hardware context entries with values from hw_ctxt
	qdma_cpm4_hw_ctxt_entries[0].value = hw_ctxt->cidx;
	qdma_cpm4_hw_ctxt_entries[1].value = hw_ctxt->crd_use;
	qdma_cpm4_hw_ctxt_entries[2].value = hw_ctxt->dsc_pend;
	qdma_cpm4_hw_ctxt_entries[3].value = hw_ctxt->idl_stp_b;
	qdma_cpm4_hw_ctxt_entries[4].value = hw_ctxt->evt_pnd;
	qdma_cpm4_hw_ctxt_entries[5].value = hw_ctxt->fetch_pnd;
	printk(KERN_DEBUG "Filled hardware context\n");
}

/* MD:
 * qdma_acc_fill_credit_ctxt() - Helper function to fill Credit context
 *                           into structure
 */
static void qdma_cpm4_fill_credit_ctxt(
		struct qdma_descq_credit_ctxt *cr_ctxt)
{
	// MD: Fill credit context entries with values from cr_ctxt
	qdma_cpm4_credit_ctxt_entries[0].value = cr_ctxt->credit;
	printk(KERN_DEBUG "Filled credit context\n");
}

/* MD:
 * qdma_acc_fill_pfetch_ctxt() - Helper function to fill Prefetch context
 *                           into structure
 */
static void qdma_cpm4_fill_pfetch_ctxt(
		struct qdma_descq_prefetch_ctxt *pfetch_ctxt)
{
	// MD: Fill prefetch context entries with values from pfetch_ctxt
	qdma_cpm4_c2h_pftch_ctxt_entries[0].value = pfetch_ctxt->bypass;
	qdma_cpm4_c2h_pftch_ctxt_entries[1].value = pfetch_ctxt->bufsz_idx;
	qdma_cpm4_c2h_pftch_ctxt_entries[2].value = pfetch_ctxt->port_id;
	qdma_cpm4_c2h_pftch_ctxt_entries[3].value = pfetch_ctxt->err;
	qdma_cpm4_c2h_pftch_ctxt_entries[4].value = pfetch_ctxt->pfch_en;
	qdma_cpm4_c2h_pftch_ctxt_entries[5].value = pfetch_ctxt->pfch;
	qdma_cpm4_c2h_pftch_ctxt_entries[6].value = pfetch_ctxt->sw_crdt;
	qdma_cpm4_c2h_pftch_ctxt_entries[7].value = pfetch_ctxt->valid;
	printk(KERN_DEBUG "Filled prefetch context\n");
}

#include <stdio.h> // MD: Include necessary headers for printk

/* MD:
 * qdma_cpm4_fill_fmap_ctxt() - Helper function to fill fmap context into structure
 */
static void qdma_cpm4_fill_fmap_ctxt(struct qdma_fmap_cfg *fmap_ctxt)
{
    // MD: Fill fmap context entries with values from fmap_ctxt
    qdma_cpm4_fmap_ctxt_entries[0].value = fmap_ctxt->qbase;
    qdma_cpm4_fmap_ctxt_entries[1].value = fmap_ctxt->qmax;
    printk(KERN_DEBUG "Filled fmap context\n");
}

/* MD:
 * qdma_cpm4_fill_qid2vec_ctxt() - Helper function to fill qid2vec context into structure
 */
static void qdma_cpm4_fill_qid2vec_ctxt(struct qdma_qid2vec *qid2vec_ctxt)
{
    // MD: Fill qid2vec context entries with values from qid2vec_ctxt
    qdma_cpm4_qid2vec_ctxt_entries[0].value = qid2vec_ctxt->c2h_vector;
    qdma_cpm4_qid2vec_ctxt_entries[1].value = qid2vec_ctxt->c2h_en_coal;
    qdma_cpm4_qid2vec_ctxt_entries[2].value = qid2vec_ctxt->h2c_vector;
    qdma_cpm4_qid2vec_ctxt_entries[3].value = qid2vec_ctxt->h2c_en_coal;
    printk(KERN_DEBUG "Filled qid2vec context\n");
}

/* MD:
 * qdma_cpm4_fill_intr_ctxt() - Helper function to fill interrupt context into structure
 */
static void qdma_cpm4_fill_intr_ctxt(struct qdma_indirect_intr_ctxt *intr_ctxt)
{
    // MD: Fill interrupt context entries with values from intr_ctxt
    qdma_cpm4_ind_intr_ctxt_entries[0].value = intr_ctxt->valid;
    qdma_cpm4_ind_intr_ctxt_entries[1].value = intr_ctxt->vec;
    qdma_cpm4_ind_intr_ctxt_entries[2].value = intr_ctxt->int_st;
    qdma_cpm4_ind_intr_ctxt_entries[3].value = intr_ctxt->color;
    qdma_cpm4_ind_intr_ctxt_entries[4].value = intr_ctxt->baddr_4k & 0xFFFFFFFF;
    qdma_cpm4_ind_intr_ctxt_entries[5].value = (intr_ctxt->baddr_4k >> 32) & 0xFFFFFFFF;
    qdma_cpm4_ind_intr_ctxt_entries[6].value = intr_ctxt->page_size;
    qdma_cpm4_ind_intr_ctxt_entries[7].value = intr_ctxt->pidx;
    printk(KERN_DEBUG "Filled interrupt context\n");
}

/* MD:
 * dump_cpm4_context() - Helper function to dump queue context into string
 *
 * return len - length of the string copied into buffer
 */
static int dump_cpm4_context(struct qdma_descq_context *queue_context,
                             uint8_t st, enum qdma_dev_q_type q_type,
                             char *buf, int buf_sz)
{
    int i = 0;
    int n;
    int len = 0;
    int rv;
    char banner[DEBGFS_LINE_SZ] = "";

    // MD: Validate queue context
    if (queue_context == NULL) {
        qdma_log_error("%s: queue_context is NULL, err:%d\n", __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Validate queue type
    if (q_type >= QDMA_DEV_Q_TYPE_CMPT) {
        qdma_log_error("%s: Invalid queue type(%d), err:%d\n", __func__, q_type, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Fill various contexts
    qdma_cpm4_fill_sw_ctxt(&queue_context->sw_ctxt);
    qdma_cpm4_fill_hw_ctxt(&queue_context->hw_ctxt);
    qdma_cpm4_fill_credit_ctxt(&queue_context->cr_ctxt);
    qdma_cpm4_fill_qid2vec_ctxt(&queue_context->qid2vec);
    if (st && (q_type == QDMA_DEV_Q_TYPE_C2H)) {
        qdma_cpm4_fill_pfetch_ctxt(&queue_context->pfetch_ctxt);
        qdma_cpm4_fill_cmpt_ctxt(&queue_context->cmpt_ctxt);
    }
    qdma_cpm4_fill_fmap_ctxt(&queue_context->fmap);

    // MD: Prepare banner for context dump
    if (q_type != QDMA_DEV_Q_TYPE_CMPT) {
        for (i = 0; i < DEBGFS_LINE_SZ - 5; i++) {
            rv = QDMA_SNPRINTF_S(banner + i, (DEBGFS_LINE_SZ - i), sizeof("-"), "-");
            if ((rv < 0) || (rv > (int)sizeof("-"))) {
                qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n", __LINE__, __func__, rv);
                goto INSUF_BUF_EXIT;
            }
        }

        // MD: SW context dump
        n = sizeof(qdma_cpm4_sw_ctxt_entries) / sizeof((qdma_cpm4_sw_ctxt_entries)[0]);
        for (i = 0; i < n; i++) {
            if ((len >= buf_sz) || ((len + DEBGFS_LINE_SZ) >= buf_sz))
                goto INSUF_BUF_EXIT;

            if (i == 0) {
                if ((len + (3 * DEBGFS_LINE_SZ)) >= buf_sz)
                    goto INSUF_BUF_EXIT;
                rv = QDMA_SNPRINTF_S(buf + len, (buf_sz - len), DEBGFS_LINE_SZ, "\n%s", banner);
                if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
                    qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n", __LINE__, __func__, rv);
                    goto INSUF_BUF_EXIT;
                }
                len += rv;

                rv = QDMA_SNPRINTF_S(buf + len, (buf_sz - len), DEBGFS_LINE_SZ, "\n%40s", "SW Context");
                if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
                    qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n", __LINE__, __func__, rv);
                    goto INSUF_BUF_EXIT;
                }
                len += rv;

                rv = QDMA_SNPRINTF_S(buf + len, (buf_sz - len), DEBGFS_LINE_SZ, "\n%s\n", banner);
                if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
                    qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n", __LINE__, __func__, rv);
                    goto INSUF_BUF_EXIT;
                }
                len += rv;
            }

            rv = QDMA_SNPRINTF_S(buf + len, (buf_sz - len), DEBGFS_LINE_SZ,
                                 "%-47s %#-10x %u\n",
                                 qdma_cpm4_sw_ctxt_entries[i].name,
                                 qdma_cpm4_sw_ctxt_entries[i].value,
                                 qdma_cpm4_sw_ctxt_entries[i].value);
            if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
                qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n", __LINE__, __func__, rv);
                goto INSUF_BUF_EXIT;
            }
            len += rv;
        }

        // MD: HW context dump
        n = sizeof(qdma_cpm4_hw_ctxt_entries) / sizeof((qdma_cpm4_hw_ctxt_entries)[0]);
        for (i = 0; i < n; i++) {
            if ((len >= buf_sz) || ((len + DEBGFS_LINE_SZ) >= buf_sz))
                goto INSUF_BUF_EXIT;

            if (i == 0) {
                if ((len + (3 * DEBGFS_LINE_SZ)) >= buf_sz)
                    goto INSUF_BUF_EXIT;

                rv = QDMA_SNPRINTF_S(buf + len, (buf_sz - len), DEBGFS_LINE_SZ, "\n%s", banner);
                if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
                    qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n", __LINE__, __func__, rv);
                    goto INSUF_BUF_EXIT;
                }
                len += rv;

                rv = QDMA_SNPRINTF_S(buf + len, (buf_sz - len), DEBGFS_LINE_SZ, "\n%40s", "HW Context");
                if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
                    qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n", __LINE__, __func__, rv);
                    goto INSUF_BUF_EXIT;
                }
                len += rv;

                rv = QDMA_SNPRINTF_S(buf + len, (buf_sz - len), DEBGFS_LINE_SZ, "\n%s\n", banner);
                if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
                    qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n", __LINE__, __func__, rv);
                    goto INSUF_BUF_EXIT;
                }
                len += rv;
            }

            rv = QDMA_SNPRINTF_S(buf + len, (buf_sz - len), DEBGFS_LINE_SZ,
                                 "%-47s %#-10x %u\n",
                                 qdma_cpm4_hw_ctxt_entries[i].name,
                                 qdma_cpm4_hw_ctxt_entries[i].value,
                                 qdma_cpm4_hw_ctxt_entries[i].value);
            if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
                qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n", __LINE__, __func__, rv);
                goto INSUF_BUF_EXIT;
            }
            len += rv;
        }

        // MD: Credit context dump
        n = sizeof(qdma_cpm4_credit_ctxt_entries) / sizeof((qdma_cpm4_credit_ctxt_entries)[0]);
        for (i = 0; i < n; i++) {
            if ((len >= buf_sz) || ((len + DEBGFS_LINE_SZ) >= buf_sz))
                goto INSUF_BUF_EXIT;

            if (i == 0) {
                if ((len + (3 * DEBGFS_LINE_SZ)) >= buf_sz)
                    goto INSUF_BUF_EXIT;

                rv = QDMA_SNPRINTF_S(buf + len, (buf_sz - len), DEBGFS_LINE_SZ, "\n%s", banner);
                if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
                    qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n", __LINE__, __func__, rv);
                    goto INSUF_BUF_EXIT;
                }
                len += rv;

                rv = QDMA_SNPRINTF_S(buf + len, (buf_sz - len), DEBGFS_LINE_SZ, "\n%40s", "Credit Context");
                if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
                    qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n", __LINE__, __func__, rv);
                    goto INSUF_BUF_EXIT;
                }
                len += rv;

                rv = QDMA_SNPRINTF_S(buf + len, (buf_sz - len), DEBGFS_LINE_SZ, "\n%s\n", banner);
                if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
                    qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n", __LINE__, __func__, rv);
                    goto INSUF_BUF_EXIT;
                }
                len += rv;
            }

            rv = QDMA_SNPRINTF_S(buf + len, (buf_sz - len), DEBGFS_LINE_SZ,
                                 "%-47s %#-10x %u\n",
                                 qdma_cpm4_credit_ctxt_entries[i].name,
                                 qdma_cpm4_credit_ctxt_entries[i].value,
                                 qdma_cpm4_credit_ctxt_entries[i].value);
            if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
                qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n", __LINE__, __func__, rv);
                goto INSUF_BUF_EXIT;
            }
            len += rv;
        }
    }

    // MD: QID2VEC context dump
    n = sizeof(qdma_cpm4_qid2vec_ctxt_entries) / sizeof((qdma_cpm4_qid2vec_ctxt_entries)[0]);
    for (i = 0; i < n; i++) {
        if ((len >= buf_sz) || ((len + DEBGFS_LINE_SZ) >= buf_sz))
            goto INSUF_BUF_EXIT;

        if (i == 0) {
            if ((len + (3 * DEBGFS_LINE_SZ)) >= buf_sz)
                goto INSUF_BUF_EXIT;
            rv = QDMA_SNPRINTF_S(buf + len, (buf_sz - len), DEBGFS_LINE_SZ, "\n%s", banner);
            if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
                qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n", __LINE__, __func__, rv);
                goto INSUF_BUF_EXIT;
            }
            len += rv;

            rv = QDMA_SNPRINTF_S(buf + len, (buf_sz - len), DEBGFS_LINE_SZ, "\n%40s", "QID2VEC Context");
            if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
                qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n", __LINE__, __func__, rv);
                goto INSUF_BUF_EXIT;
            }
            len += rv;

            rv = QDMA_SNPRINTF_S(buf + len, (buf_sz - len), DEBGFS_LINE_SZ, "\n%s\n", banner);
            if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
                qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n", __LINE__, __func__, rv);
                goto INSUF_BUF_EXIT;
            }
            len += rv;
        }

        rv = QDMA_SNPRINTF_S(buf + len, (buf_sz - len), DEBGFS_LINE_SZ,
                             "%-47s %#-10x %u\n",
                             qdma_cpm4_qid2vec_ctxt_entries[i].name,
                             qdma_cpm4_qid2vec_ctxt_entries[i].value,
                             qdma_cpm4_qid2vec_ctxt_entries[i].value);
        if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
            qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n", __LINE__, __func__, rv);
            goto INSUF_BUF_EXIT;
        }
        len += rv;
    }

    // MD: Completion and Prefetch context dump for specific queue types
    if ((q_type == QDMA_DEV_Q_TYPE_CMPT) || (st && q_type == QDMA_DEV_Q_TYPE_C2H)) {
        // MD: Completion context dump
        n = sizeof(qdma_cpm4_cmpt_ctxt_entries) / sizeof((qdma_cpm4_cmpt_ctxt_entries)[0]);
        for (i = 0; i < n; i++) {
            if ((len >= buf_sz) || ((len + DEBGFS_LINE_SZ) >= buf_sz))
                goto INSUF_BUF_EXIT;

            if (i == 0) {
                if ((len + (3 * DEBGFS_LINE_SZ)) >= buf_sz)
                    goto INSUF_BUF_EXIT;

                rv = QDMA_SNPRINTF_S(buf + len, (buf_sz - len), DEBGFS_LINE_SZ, "\n%s", banner);
                if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
                    qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n", __LINE__, __func__, rv);
                    goto INSUF_BUF_EXIT;
                }
                len += rv;

                rv = QDMA_SNPRINTF_S(buf + len, (buf_sz - len), DEBGFS_LINE_SZ, "\n%40s", "Completion Context");
                if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
                    qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n", __LINE__, __func__, rv);
                    goto INSUF_BUF_EXIT;
                }
                len += rv;

                rv = QDMA_SNPRINTF_S(buf + len, (buf_sz - len), DEBGFS_LINE_SZ, "\n%s\n", banner);
                if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
                    qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n", __LINE__, __func__, rv);
                    goto INSUF_BUF_EXIT;
                }
                len += rv;
            }

            rv = QDMA_SNPRINTF_S(buf + len, (buf_sz - len), DEBGFS_LINE_SZ,
                                 "%-47s %#-10x %u\n",
                                 qdma_cpm4_cmpt_ctxt_entries[i].name,
                                 qdma_cpm4_cmpt_ctxt_entries[i].value,
                                 qdma_cpm4_cmpt_ctxt_entries[i].value);
            if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
                qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n", __LINE__, __func__, rv);
                goto INSUF_BUF_EXIT;
            }
            len += rv;
        }
    }

    if (st && q_type == QDMA_DEV_Q_TYPE_C2H) {
        // MD: Prefetch context dump
        n = sizeof(qdma_cpm4_c2h_pftch_ctxt_entries) / sizeof(qdma_cpm4_c2h_pftch_ctxt_entries[0]);
        for (i = 0; i < n; i++) {
            if ((len >= buf_sz) || ((len + DEBGFS_LINE_SZ) >= buf_sz))
                goto INSUF_BUF_EXIT;

            if (i == 0) {
                if ((len + (3 * DEBGFS_LINE_SZ)) >= buf_sz)
                    goto INSUF_BUF_EXIT;

                rv = QDMA_SNPRINTF_S(buf + len, (buf_sz - len), DEBGFS_LINE_SZ, "\n%s", banner);
                if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
                    qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n", __LINE__, __func__, rv);
                    goto INSUF_BUF_EXIT;
                }
                len += rv;

                rv = QDMA_SNPRINTF_S(buf + len, (buf_sz - len), DEBGFS_LINE_SZ, "\n%40s", "Prefetch Context");
                if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
                    qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n", __LINE__, __func__, rv);
                    goto INSUF_BUF_EXIT;
                }
                len += rv;

                rv = QDMA_SNPRINTF_S(buf + len, (buf_sz - len), DEBGFS_LINE_SZ, "\n%s\n", banner);
                if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
                    qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n", __LINE__, __func__, rv);
                    goto INSUF_BUF_EXIT;
                }
                len += rv;
            }

            rv = QDMA_SNPRINTF_S(buf + len, (buf_sz - len), DEBGFS_LINE_SZ,
                                 "%-47s %#-10x %u\n",
                                 qdma_cpm4_c2h_pftch_ctxt_entries[i].name,
                                 qdma_cpm4_c2h_pftch_ctxt_entries[i].value,
                                 qdma_cpm4_c2h_pftch_ctxt_entries[i].value);
            if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
                qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n", __LINE__, __func__, rv);
                goto INSUF_BUF_EXIT;
            }
            len += rv;
        }
    }

    // MD: Fmap context dump
    n = sizeof(qdma_cpm4_fmap_ctxt_entries) / sizeof(qdma_cpm4_fmap_ctxt_entries[0]);
    for (i = 0; i < n; i++) {
        if ((len >= buf_sz) || ((len + DEBGFS_LINE_SZ) >= buf_sz))
            goto INSUF_BUF_EXIT;

        if (i == 0) {
            if ((len + (3 * DEBGFS_LINE_SZ)) >= buf_sz)
                goto INSUF_BUF_EXIT;

            rv = QDMA_SNPRINTF_S(buf + len, (buf_sz - len), DEBGFS_LINE_SZ, "\n%s", banner);
            if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
                qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n", __LINE__, __func__, rv);
                goto INSUF_BUF_EXIT;
            }
            len += rv;

            rv = QDMA_SNPRINTF_S(buf + len, (buf_sz - len), DEBGFS_LINE_SZ, "\n%40s", "Fmap Context");
            if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
                qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n", __LINE__, __func__, rv);
                goto INSUF_BUF_EXIT;
            }
            len += rv;

            rv = QDMA_SNPRINTF_S(buf + len, (buf_sz - len), DEBGFS_LINE_SZ, "\n%s\n", banner);
            if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
                qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n", __LINE__, __func__, rv);
                goto INSUF_BUF_EXIT;
            }
            len += rv;
        }

        rv = QDMA_SNPRINTF_S(buf + len, (buf_sz - len), DEBGFS_LINE_SZ,
                             "%-47s %#-10x %u\n",
                             qdma_cpm4_fmap_ctxt_entries[i].name,
                             qdma_cpm4_fmap_ctxt_entries[i].value,
                             qdma_cpm4_fmap_ctxt_entries[i].value);
        if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
            qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n", __LINE__, __func__, rv);
            goto INSUF_BUF_EXIT;
        }
        len += rv;
    }

    return len;

INSUF_BUF_EXIT:
    if (buf_sz > DEBGFS_LINE_SZ) {
        rv = QDMA_SNPRINTF_S((buf + buf_sz - DEBGFS_LINE_SZ), buf_sz, DEBGFS_LINE_SZ,
                             "\n\nInsufficient buffer size, partial context dump\n");
        if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
            qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n", __LINE__, __func__, rv);
        }
    }

    qdma_log_error("%s: Insufficient buffer size, err:%d\n", __func__, -QDMA_ERR_NO_MEM);

    return -QDMA_ERR_NO_MEM;
}

/* MD:
 * dump_cpm4_intr_context() - Helper function to dump interrupt context into string
 *
 * This function fills the interrupt context and then formats it into a string buffer.
 * It handles buffer size checks and logs errors if the buffer is insufficient.
 *
 * return len - length of the string copied into buffer
 */
static int dump_cpm4_intr_context(struct qdma_indirect_intr_ctxt *intr_ctx,
                                  int ring_index,
                                  char *buf, int buf_sz)
{
    int i = 0;
    int n;
    int len = 0; // MD: Initialize length of the string copied into buffer
    int rv; // MD: Return value for snprintf operations
    char banner[DEBGFS_LINE_SZ] = ""; // MD: Banner for context dump

    // MD: Fill the interrupt context with values from intr_ctx
    qdma_cpm4_fill_intr_ctxt(intr_ctx);

    // MD: Prepare banner for context dump
    for (i = 0; i < DEBGFS_LINE_SZ - 5; i++) {
        rv = QDMA_SNPRINTF_S(banner + i, (DEBGFS_LINE_SZ - i), sizeof("-"), "-");
        if ((rv < 0) || (rv > (int)sizeof("-"))) {
            qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n", __LINE__, __func__, rv);
            goto INSUF_BUF_EXIT;
        }
    }

    // MD: Interrupt context dump
    n = sizeof(qdma_cpm4_ind_intr_ctxt_entries) / sizeof((qdma_cpm4_ind_intr_ctxt_entries)[0]);
    for (i = 0; i < n; i++) {
        if ((len >= buf_sz) || ((len + DEBGFS_LINE_SZ) >= buf_sz))
            goto INSUF_BUF_EXIT;

        if (i == 0) {
            if ((len + (3 * DEBGFS_LINE_SZ)) >= buf_sz)
                goto INSUF_BUF_EXIT;

            rv = QDMA_SNPRINTF_S(buf + len, (buf_sz - len), DEBGFS_LINE_SZ, "\n%s", banner);
            if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
                qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n", __LINE__, __func__, rv);
                goto INSUF_BUF_EXIT;
            }
            len += rv;

            rv = QDMA_SNPRINTF_S(buf + len, (buf_sz - len), DEBGFS_LINE_SZ, "\n%50s %d", "Interrupt Context for ring#", ring_index);
            if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
                qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n", __LINE__, __func__, rv);
                goto INSUF_BUF_EXIT;
            }
            len += rv;

            rv = QDMA_SNPRINTF_S(buf + len, (buf_sz - len), DEBGFS_LINE_SZ, "\n%s\n", banner);
            if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
                qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n", __LINE__, __func__, rv);
                goto INSUF_BUF_EXIT;
            }
            len += rv;
        }

        rv = QDMA_SNPRINTF_S(buf + len, (buf_sz - len), DEBGFS_LINE_SZ,
                             "%-47s %#-10x %u\n",
                             qdma_cpm4_ind_intr_ctxt_entries[i].name,
                             qdma_cpm4_ind_intr_ctxt_entries[i].value,
                             qdma_cpm4_ind_intr_ctxt_entries[i].value);
        if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
            qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n", __LINE__, __func__, rv);
            goto INSUF_BUF_EXIT;
        }
        len += rv;
    }

    return len; // MD: Return the length of the string copied into buffer

INSUF_BUF_EXIT:
    if (buf_sz > DEBGFS_LINE_SZ) {
        rv = QDMA_SNPRINTF_S((buf + buf_sz - DEBGFS_LINE_SZ), buf_sz, DEBGFS_LINE_SZ,
                             "\n\nInsufficient buffer size, partial intr context dump\n");
        if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
            qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n", __LINE__, __func__, rv);
        }
    }

    qdma_log_error("%s: Insufficient buffer size, err:%d\n", __func__, -QDMA_ERR_NO_MEM);

    return -QDMA_ERR_NO_MEM; // MD: Return error code for insufficient buffer size
}

/* MD:
 * qdma_cpm4_indirect_reg_invalidate() - Helper function to invalidate indirect context registers.
 *
 * This function sets the command register to invalidate the indirect context registers.
 * It checks if the operation completes successfully and logs an error if it fails.
 *
 * return -QDMA_ERR_HWACC_BUSY_TIMEOUT if register value didn't match, QDMA_SUCCESS otherwise
 */
static int qdma_cpm4_indirect_reg_invalidate(void *dev_hndl,
                                             enum ind_ctxt_cmd_sel sel, uint16_t hw_qid)
{
    union qdma_cpm4_ind_ctxt_cmd cmd;

    qdma_reg_access_lock(dev_hndl); // MD: Lock register access

    // MD: Set command register for invalidation
    cmd.word = 0;
    cmd.bits.qid = hw_qid;
    cmd.bits.op = QDMA_CTXT_CMD_INV;
    cmd.bits.sel = sel;
    qdma_reg_write(dev_hndl, QDMA_CPM4_IND_CTXT_CMD_ADDR, cmd.word);

    // MD: Check if the operation completed successfully
    if (hw_monitor_reg(dev_hndl, QDMA_CPM4_IND_CTXT_CMD_ADDR,
                       IND_CTXT_CMD_BUSY_MASK, 0,
                       QDMA_REG_POLL_DFLT_INTERVAL_US,
                       QDMA_REG_POLL_DFLT_TIMEOUT_US)) {
        qdma_reg_access_release(dev_hndl); // MD: Release register access
        qdma_log_error("%s: hw_monitor_reg failed, err:%d\n",
                       __func__,
                       -QDMA_ERR_HWACC_BUSY_TIMEOUT);
        return -QDMA_ERR_HWACC_BUSY_TIMEOUT;
    }

    qdma_reg_access_release(dev_hndl); // MD: Release register access

    return QDMA_SUCCESS;
}

/* MD:
 * qdma_cpm4_indirect_reg_clear() - Helper function to clear indirect context registers.
 *
 * This function sets the command register to clear the indirect context registers.
 * It checks if the operation completes successfully and logs an error if it fails.
 *
 * return -QDMA_ERR_HWACC_BUSY_TIMEOUT if register value didn't match, QDMA_SUCCESS otherwise
 */
static int qdma_cpm4_indirect_reg_clear(void *dev_hndl,
                                        enum ind_ctxt_cmd_sel sel, uint16_t hw_qid)
{
    union qdma_cpm4_ind_ctxt_cmd cmd;

    qdma_reg_access_lock(dev_hndl); // MD: Lock register access

    // MD: Set command register for clearing
    cmd.word = 0;
    cmd.bits.qid = hw_qid;
    cmd.bits.op = QDMA_CTXT_CMD_CLR;
    cmd.bits.sel = sel;
    qdma_reg_write(dev_hndl, QDMA_CPM4_IND_CTXT_CMD_ADDR, cmd.word);

    // MD: Check if the operation completed successfully
    if (hw_monitor_reg(dev_hndl, QDMA_CPM4_IND_CTXT_CMD_ADDR,
                       IND_CTXT_CMD_BUSY_MASK, 0,
                       QDMA_REG_POLL_DFLT_INTERVAL_US,
                       QDMA_REG_POLL_DFLT_TIMEOUT_US)) {
        qdma_reg_access_release(dev_hndl); // MD: Release register access
        qdma_log_error("%s: hw_monitor_reg failed, err:%d\n",
                       __func__,
                       -QDMA_ERR_HWACC_BUSY_TIMEOUT);
        return -QDMA_ERR_HWACC_BUSY_TIMEOUT;
    }

    qdma_reg_access_release(dev_hndl); // MD: Release register access

    return QDMA_SUCCESS;
}

/* MD:
 * qdma_cpm4_indirect_reg_read() - Helper function to read indirect context registers.
 *
 * This function sets the command register to read the indirect context registers.
 * It reads the data from the registers and checks if the operation completes successfully.
 * Logs an error if it fails.
 *
 * return -QDMA_ERR_HWACC_BUSY_TIMEOUT if register value didn't match, QDMA_SUCCESS otherwise
 */
static int qdma_cpm4_indirect_reg_read(void *dev_hndl,
                                       enum ind_ctxt_cmd_sel sel,
                                       uint16_t hw_qid, uint32_t cnt, uint32_t *data)
{
    uint32_t index = 0, reg_addr = QDMA_CPM4_IND_CTXT_DATA_3_ADDR;
    union qdma_cpm4_ind_ctxt_cmd cmd;

    qdma_reg_access_lock(dev_hndl); // MD: Lock register access

    // MD: Set command register for reading
    cmd.word = 0;
    cmd.bits.qid = hw_qid;
    cmd.bits.op = QDMA_CTXT_CMD_RD;
    cmd.bits.sel = sel;
    qdma_reg_write(dev_hndl, QDMA_CPM4_IND_CTXT_CMD_ADDR, cmd.word);

    // MD: Check if the operation completed successfully
    if (hw_monitor_reg(dev_hndl, QDMA_CPM4_IND_CTXT_CMD_ADDR,
                       IND_CTXT_CMD_BUSY_MASK, 0,
                       QDMA_REG_POLL_DFLT_INTERVAL_US,
                       QDMA_REG_POLL_DFLT_TIMEOUT_US)) {
        qdma_reg_access_release(dev_hndl); // MD: Release register access
        qdma_log_error("%s: hw_monitor_reg failed, err:%d\n",
                       __func__,
                       -QDMA_ERR_HWACC_BUSY_TIMEOUT);
        return -QDMA_ERR_HWACC_BUSY_TIMEOUT;
    }

    // MD: Read data from context registers
    for (index = 0; index < cnt; index++, reg_addr += sizeof(uint32_t))
        data[index] = qdma_reg_read(dev_hndl, reg_addr);

    qdma_reg_access_release(dev_hndl); // MD: Release register access

    return QDMA_SUCCESS;
}

/* MD:
 * qdma_cpm4_indirect_reg_write() - Helper function to write indirect context registers.
 *
 * This function sets the command register to write to the indirect context registers.
 * It writes the data to the registers and checks if the operation completes successfully.
 * Logs an error if it fails.
 *
 * return -QDMA_ERR_HWACC_BUSY_TIMEOUT if register value didn't match, QDMA_SUCCESS otherwise
 */
static int qdma_cpm4_indirect_reg_write(void *dev_hndl,
                                        enum ind_ctxt_cmd_sel sel,
                                        uint16_t hw_qid, uint32_t *data, uint16_t cnt)
{
    uint32_t index, reg_addr;
    struct qdma_cpm4_indirect_ctxt_regs regs;
    uint32_t *wr_data = (uint32_t *)&regs;

    qdma_reg_access_lock(dev_hndl); // MD: Lock register access

    // MD: Write the context data
    for (index = 0; index < QDMA_CPM4_IND_CTXT_DATA_NUM_REGS; index++) {
        if (index < cnt)
            regs.qdma_ind_ctxt_data[index] = data[index];
        else
            regs.qdma_ind_ctxt_data[index] = 0;
        regs.qdma_ind_ctxt_mask[index] = 0xFFFFFFFF;
    }

    // MD: Set command register for writing
    regs.cmd.word = 0;
    regs.cmd.bits.qid = hw_qid;
    regs.cmd.bits.op = QDMA_CTXT_CMD_WR;
    regs.cmd.bits.sel = sel;
    reg_addr = QDMA_CPM4_IND_CTXT_DATA_3_ADDR;

    // MD: Write data and command to registers
    for (index = 0; index < ((2 * QDMA_CPM4_IND_CTXT_DATA_NUM_REGS) + 1);
         index++, reg_addr += sizeof(uint32_t))
        qdma_reg_write(dev_hndl, reg_addr, wr_data[index]);

    // MD: Check if the operation completed successfully
    if (hw_monitor_reg(dev_hndl, QDMA_CPM4_IND_CTXT_CMD_ADDR,
                       IND_CTXT_CMD_BUSY_MASK, 0,
                       QDMA_REG_POLL_DFLT_INTERVAL_US,
                       QDMA_REG_POLL_DFLT_TIMEOUT_US)) {
        qdma_reg_access_release(dev_hndl); // MD: Release register access
        qdma_log_error("%s: hw_monitor_reg failed, err:%d\n",
                       __func__,
                       -QDMA_ERR_HWACC_BUSY_TIMEOUT);
        return -QDMA_ERR_HWACC_BUSY_TIMEOUT;
    }

    qdma_reg_access_release(dev_hndl); // MD: Release register access

    return QDMA_SUCCESS;
}

/* MD:
 * qdma_cpm4_qid2vec_write() - Create qid2vec context and program it
 *
 * This function writes the qid2vec context for a given queue.
 * It reads the current context, modifies it based on the queue type (C2H or H2C),
 * and writes it back to the hardware.
 *
 * @dev_hndl: Device handle
 * @c2h: Is C2H queue
 * @hw_qid: Hardware QID of the queue
 * @ctxt: Pointer to the context data
 *
 * Return: 0 on success, negative error code on failure
 */
static int qdma_cpm4_qid2vec_write(void *dev_hndl, uint8_t c2h,
                                   uint16_t hw_qid, struct qdma_qid2vec *ctxt)
{
    uint32_t qid2vec = 0;
    enum ind_ctxt_cmd_sel sel = QDMA_CTXT_SEL_FMAP;
    int rv = 0;

    // MD: Validate input parameters
    if (!dev_hndl || !ctxt) {
        qdma_log_error("%s: dev_hndl=%p qid2vec=%p, err:%d\n",
                       __func__, dev_hndl, ctxt, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Read current qid2vec context
    rv = qdma_cpm4_indirect_reg_read(dev_hndl, sel, hw_qid, 1, &qid2vec);
    if (rv < 0)
        return rv;

    // MD: Modify context based on queue type
    if (c2h) {
        qid2vec = qid2vec & (QDMA_CPM4_QID2VEC_H2C_VECTOR |
                             QDMA_CPM4_QID2VEC_H2C_COAL_EN);
        qid2vec |= FIELD_SET(C2H_QID2VEC_MAP_C2H_VECTOR_MASK, ctxt->c2h_vector) |
                   FIELD_SET(C2H_QID2VEC_MAP_C2H_EN_COAL_MASK, ctxt->c2h_en_coal);
    } else {
        qid2vec = qid2vec & (C2H_QID2VEC_MAP_C2H_VECTOR_MASK |
                             C2H_QID2VEC_MAP_C2H_EN_COAL_MASK);
        qid2vec |= FIELD_SET(QDMA_CPM4_QID2VEC_H2C_VECTOR, ctxt->h2c_vector) |
                   FIELD_SET(QDMA_CPM4_QID2VEC_H2C_COAL_EN, ctxt->h2c_en_coal);
    }

    // MD: Write modified context back to hardware
    return qdma_cpm4_indirect_reg_write(dev_hndl, sel, hw_qid, &qid2vec, QDMA_CPM4_QID2VEC_CONTEXT_NUM_WORDS);
}

/* MD:
 * qdma_cpm4_qid2vec_read() - Read qid2vec context
 *
 * This function reads the qid2vec context for a given queue and populates the context structure.
 *
 * @dev_hndl: Device handle
 * @c2h: Is C2H queue
 * @hw_qid: Hardware QID of the queue
 * @ctxt: Pointer to the context data
 *
 * Return: 0 on success, negative error code on failure
 */
static int qdma_cpm4_qid2vec_read(void *dev_hndl, uint8_t c2h,
                                  uint16_t hw_qid, struct qdma_qid2vec *ctxt)
{
    int rv = 0;
    uint32_t qid2vec[QDMA_CPM4_QID2VEC_CONTEXT_NUM_WORDS] = {0};
    enum ind_ctxt_cmd_sel sel = QDMA_CTXT_SEL_FMAP;

    // MD: Validate input parameters
    if (!dev_hndl || !ctxt) {
        qdma_log_error("%s: dev_hndl=%p qid2vec=%p, err:%d\n",
                       __func__, dev_hndl, ctxt, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Read qid2vec context from hardware
    rv = qdma_cpm4_indirect_reg_read(dev_hndl, sel, hw_qid, QDMA_CPM4_QID2VEC_CONTEXT_NUM_WORDS, qid2vec);
    if (rv < 0)
        return rv;

    // MD: Populate context structure based on queue type
    if (c2h) {
        ctxt->c2h_vector = FIELD_GET(C2H_QID2VEC_MAP_C2H_VECTOR_MASK, qid2vec[0]);
        ctxt->c2h_en_coal = (uint8_t)(FIELD_GET(C2H_QID2VEC_MAP_C2H_EN_COAL_MASK, qid2vec[0]));
    } else {
        ctxt->h2c_vector = (uint8_t)(FIELD_GET(QDMA_CPM4_QID2VEC_H2C_VECTOR, qid2vec[0]));
        ctxt->h2c_en_coal = (uint8_t)(FIELD_GET(QDMA_CPM4_QID2VEC_H2C_COAL_EN, qid2vec[0]));
    }

    return QDMA_SUCCESS;
}

/* MD:
 * qdma_cpm4_qid2vec_clear() - Clear qid2vec context
 *
 * This function clears the qid2vec context for a given queue.
 *
 * @dev_hndl: Device handle
 * @hw_qid: Hardware QID of the queue
 *
 * Return: 0 on success, negative error code on failure
 */
static int qdma_cpm4_qid2vec_clear(void *dev_hndl, uint16_t hw_qid)
{
    enum ind_ctxt_cmd_sel sel = QDMA_CTXT_SEL_FMAP;

    // MD: Validate input parameter
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n", __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Clear qid2vec context in hardware
    return qdma_cpm4_indirect_reg_clear(dev_hndl, sel, hw_qid);
}

/* MD:
 * qdma_cpm4_qid2vec_invalidate() - Invalidate qid2vec context
 *
 * This function invalidates the qid2vec context for a given queue.
 *
 * @dev_hndl: Device handle
 * @hw_qid: Hardware QID of the queue
 *
 * Return: 0 on success, negative error code on failure
 */
static int qdma_cpm4_qid2vec_invalidate(void *dev_hndl, uint16_t hw_qid)
{
    enum ind_ctxt_cmd_sel sel = QDMA_CTXT_SEL_FMAP;

    // MD: Validate input parameter
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n", __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Invalidate qid2vec context in hardware
    return qdma_cpm4_indirect_reg_invalidate(dev_hndl, sel, hw_qid);
}

/* MD:
 * qdma_cpm4_qid2vec_conf() - Configure qid2vector context
 *
 * This function configures the qid2vec context based on the specified access type.
 *
 * @dev_hndl: Device handle
 * @c2h: Is C2H queue
 * @hw_qid: Hardware QID of the queue
 * @ctxt: Pointer to the context data
 * @access_type: HW access type (qdma_hw_access_type enum) value
 *
 * Return: 0 on success, negative error code on failure
 */
int qdma_cpm4_qid2vec_conf(void *dev_hndl, uint8_t c2h, uint16_t hw_qid,
                           struct qdma_qid2vec *ctxt,
                           enum qdma_hw_access_type access_type)
{
    int ret_val = 0;

    // MD: Configure qid2vec context based on access type
    switch (access_type) {
    case QDMA_HW_ACCESS_READ:
        ret_val = qdma_cpm4_qid2vec_read(dev_hndl, c2h, hw_qid, ctxt);
        break;
    case QDMA_HW_ACCESS_WRITE:
        ret_val = qdma_cpm4_qid2vec_write(dev_hndl, c2h, hw_qid, ctxt);
        break;
    case QDMA_HW_ACCESS_CLEAR:
        ret_val = qdma_cpm4_qid2vec_clear(dev_hndl, hw_qid);
        break;
    case QDMA_HW_ACCESS_INVALIDATE:
        ret_val = qdma_cpm4_qid2vec_invalidate(dev_hndl, hw_qid);
        break;
    default:
        qdma_log_error("%s: access_type=%d is invalid, err:%d\n", __func__, access_type, -QDMA_ERR_INV_PARAM);
        ret_val = -QDMA_ERR_INV_PARAM;
        break;
    }

    return ret_val;
}

/* MD:
 * qdma_cpm4_fmap_write() - Create fmap context and program it
 *
 * This function writes the fmap context for a given function ID.
 *
 * @dev_hndl: Device handle
 * @func_id: Function ID of the device
 * @config: Pointer to the fmap data structure
 *
 * Return: 0 on success, negative error code on failure
 */
static int qdma_cpm4_fmap_write(void *dev_hndl, uint16_t func_id,
                                const struct qdma_fmap_cfg *config)
{
    uint32_t fmap = 0;

    // MD: Validate input parameters
    if (!dev_hndl || !config) {
        qdma_log_error("%s: dev_handle or config is NULL, err:%d\n", __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Set fmap context fields
    fmap = FIELD_SET(TRQ_SEL_FMAP_0_QID_BASE_MASK, config->qbase) |
           FIELD_SET(TRQ_SEL_FMAP_0_QID_MAX_MASK, config->qmax);

    // MD: Write fmap context to hardware
    qdma_reg_write(dev_hndl, QDMA_CPM4_TRQ_SEL_FMAP_0_ADDR + func_id * QDMA_CPM4_REG_TRQ_SEL_FMAP_STEP, fmap);
    return QDMA_SUCCESS;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_fmap_read() - Read fmap context
 *
 * This function reads the fmap context for a given function ID and populates the output structure.
 *
 * @dev_hndl: Device handle
 * @func_id: Function ID of the device
 * @config: Pointer to the output fmap data
 *
 * Return: 0 on success, negative error code on failure
 *****************************************************************************/
static int qdma_cpm4_fmap_read(void *dev_hndl, uint16_t func_id,
                               struct qdma_fmap_cfg *config)
{
    uint32_t fmap = 0;

    // MD: Validate input parameters
    if (!dev_hndl || !config) {
        qdma_log_error("%s: dev_handle=%p fmap=%p NULL, err:%d\n",
                       __func__, dev_hndl, config, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Read fmap context from hardware
    fmap = qdma_reg_read(dev_hndl, QDMA_CPM4_TRQ_SEL_FMAP_0_ADDR +
                         func_id * QDMA_CPM4_REG_TRQ_SEL_FMAP_STEP);

    // MD: Populate output structure with read values
    config->qbase = FIELD_GET(TRQ_SEL_FMAP_0_QID_BASE_MASK, fmap);
    config->qmax = (uint16_t)(FIELD_GET(TRQ_SEL_FMAP_0_QID_MAX_MASK, fmap));

    return QDMA_SUCCESS;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_fmap_clear() - Clear fmap context
 *
 * This function clears the fmap context for a given function ID.
 *
 * @dev_hndl: Device handle
 * @func_id: Function ID of the device
 *
 * Return: 0 on success, negative error code on failure
 *****************************************************************************/
static int qdma_cpm4_fmap_clear(void *dev_hndl, uint16_t func_id)
{
    uint32_t fmap = 0;

    // MD: Validate input parameter
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n", __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Clear fmap context in hardware
    qdma_reg_write(dev_hndl, QDMA_CPM4_TRQ_SEL_FMAP_0_ADDR +
                   func_id * QDMA_CPM4_REG_TRQ_SEL_FMAP_STEP, fmap);

    return QDMA_SUCCESS;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_fmap_conf() - Configure fmap context
 *
 * This function configures the fmap context based on the specified access type.
 * Note: QDMA_HW_ACCESS_INVALIDATE is unsupported.
 *
 * @dev_hndl: Device handle
 * @func_id: Function ID of the device
 * @config: Pointer to the fmap data
 * @access_type: HW access type (qdma_hw_access_type enum) value
 *
 * Return: 0 on success, negative error code on failure
 *****************************************************************************/
int qdma_cpm4_fmap_conf(void *dev_hndl, uint16_t func_id,
                        struct qdma_fmap_cfg *config,
                        enum qdma_hw_access_type access_type)
{
    int ret_val = 0;

    // MD: Configure fmap context based on access type
    switch (access_type) {
    case QDMA_HW_ACCESS_READ:
        ret_val = qdma_cpm4_fmap_read(dev_hndl, func_id, config);
        break;
    case QDMA_HW_ACCESS_WRITE:
        ret_val = qdma_cpm4_fmap_write(dev_hndl, func_id, config);
        break;
    case QDMA_HW_ACCESS_CLEAR:
        ret_val = qdma_cpm4_fmap_clear(dev_hndl, func_id);
        break;
    case QDMA_HW_ACCESS_INVALIDATE:
    default:
        qdma_log_error("%s: access_type=%d is invalid, err:%d\n",
                       __func__, access_type, -QDMA_ERR_INV_PARAM);
        ret_val = -QDMA_ERR_INV_PARAM;
        break;
    }

    return ret_val;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_sw_context_write() - Create SW context and program it
 *
 * This function writes the software context for a given queue.
 * It validates the input parameters, prepares the context data, and writes it to the hardware.
 *
 * @dev_hndl: Device handle
 * @c2h: Is C2H queue
 * @hw_qid: Hardware QID of the queue
 * @ctxt: Pointer to the SW context data structure
 *
 * Return: 0 on success, negative error code on failure
 *****************************************************************************/
static int qdma_cpm4_sw_context_write(void *dev_hndl, uint8_t c2h,
                                      uint16_t hw_qid,
                                      const struct qdma_descq_sw_ctxt *ctxt)
{
    uint32_t sw_ctxt[QDMA_CPM4_SW_CONTEXT_NUM_WORDS] = {0};
    uint16_t num_words_count = 0;
    enum ind_ctxt_cmd_sel sel = c2h ? QDMA_CTXT_SEL_SW_C2H : QDMA_CTXT_SEL_SW_H2C;

    // MD: Validate input parameters
    if (!dev_hndl || !ctxt) {
        qdma_log_error("%s: dev_hndl or ctxt is NULL, err:%d\n", __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Validate descriptor size and ring size index
    if ((ctxt->desc_sz > QDMA_DESC_SIZE_64B) || (ctxt->rngsz_idx >= QDMA_NUM_RING_SIZES)) {
        qdma_log_error("%s: Invalid desc_sz(%d)/rngidx(%d), err:%d\n",
                       __func__, ctxt->desc_sz, ctxt->rngsz_idx, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Prepare SW context data
    sw_ctxt[num_words_count++] =
        FIELD_SET(SW_IND_CTXT_DATA_W0_PIDX_MASK, ctxt->pidx) |
        FIELD_SET(SW_IND_CTXT_DATA_W0_IRQ_ARM_MASK, ctxt->irq_arm);

    sw_ctxt[num_words_count++] =
        FIELD_SET(SW_IND_CTXT_DATA_W1_QEN_MASK, ctxt->qen) |
        FIELD_SET(SW_IND_CTXT_DATA_W1_FCRD_EN_MASK, ctxt->frcd_en) |
        FIELD_SET(SW_IND_CTXT_DATA_W1_WBI_CHK_MASK, ctxt->wbi_chk) |
        FIELD_SET(SW_IND_CTXT_DATA_W1_WBI_INTVL_EN_MASK, ctxt->wbi_intvl_en) |
        FIELD_SET(SW_IND_CTXT_DATA_W1_FNC_ID_MASK, ctxt->fnc_id) |
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

    sw_ctxt[num_words_count++] = ctxt->ring_bs_addr & 0xffffffff;
    sw_ctxt[num_words_count++] = (ctxt->ring_bs_addr >> 32) & 0xffffffff;

    // MD: Write SW context to hardware
    return qdma_cpm4_indirect_reg_write(dev_hndl, sel, hw_qid, sw_ctxt, num_words_count);
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_sw_context_read() - Read SW context
 *
 * This function reads the software context for a given queue and populates the output structure.
 *
 * @dev_hndl: Device handle
 * @c2h: Is C2H queue
 * @hw_qid: Hardware QID of the queue
 * @ctxt: Pointer to the output context data
 *
 * Return: 0 on success, negative error code on failure
 *****************************************************************************/
static int qdma_cpm4_sw_context_read(void *dev_hndl, uint8_t c2h,
                                     uint16_t hw_qid,
                                     struct qdma_descq_sw_ctxt *ctxt)
{
    int rv = 0;
    uint32_t sw_ctxt[QDMA_CPM4_SW_CONTEXT_NUM_WORDS] = {0};
    enum ind_ctxt_cmd_sel sel = c2h ? QDMA_CTXT_SEL_SW_C2H : QDMA_CTXT_SEL_SW_H2C;
    struct qdma_qid2vec qid2vec_ctxt = {0};

    // MD: Validate input parameters
    if (!dev_hndl || !ctxt) {
        qdma_log_error("%s: dev_hndl=%p sw_ctxt=%p, err:%d\n",
                       __func__, dev_hndl, ctxt, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Read SW context from hardware
    rv = qdma_cpm4_indirect_reg_read(dev_hndl, sel, hw_qid,
                                     QDMA_CPM4_SW_CONTEXT_NUM_WORDS, sw_ctxt);
    if (rv < 0)
        return rv;

    // MD: Populate output structure with read values
    ctxt->pidx = FIELD_GET(SW_IND_CTXT_DATA_W0_PIDX_MASK, sw_ctxt[0]);
    ctxt->irq_arm = (uint8_t)(FIELD_GET(SW_IND_CTXT_DATA_W0_IRQ_ARM_MASK, sw_ctxt[0]));

    ctxt->qen = FIELD_GET(SW_IND_CTXT_DATA_W1_QEN_MASK, sw_ctxt[1]);
    ctxt->frcd_en = FIELD_GET(SW_IND_CTXT_DATA_W1_FCRD_EN_MASK, sw_ctxt[1]);
    ctxt->wbi_chk = FIELD_GET(SW_IND_CTXT_DATA_W1_WBI_CHK_MASK, sw_ctxt[1]);
    ctxt->wbi_intvl_en = FIELD_GET(SW_IND_CTXT_DATA_W1_WBI_INTVL_EN_MASK, sw_ctxt[1]);
    ctxt->fnc_id = (uint8_t)(FIELD_GET(SW_IND_CTXT_DATA_W1_FNC_ID_MASK, sw_ctxt[1]));
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

    ctxt->ring_bs_addr = ((uint64_t)sw_ctxt[3] << 32) | (sw_ctxt[2]);

    // MD: Read the QID2VEC Context Data
    rv = qdma_cpm4_qid2vec_read(dev_hndl, c2h, hw_qid, &qid2vec_ctxt);
    if (rv < 0)
        return rv;

    // MD: Populate vector and interrupt aggregation fields based on queue type
    if (c2h) {
        ctxt->vec = qid2vec_ctxt.c2h_vector;
        ctxt->intr_aggr = qid2vec_ctxt.c2h_en_coal;
    } else {
        ctxt->vec = qid2vec_ctxt.h2c_vector;
        ctxt->intr_aggr = qid2vec_ctxt.h2c_en_coal;
    }

    return QDMA_SUCCESS;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_sw_context_clear() - Clear SW context
 *
 * This function clears the software context for a given queue.
 *
 * @dev_hndl: Device handle
 * @c2h: Is C2H queue
 * @hw_qid: Hardware QID of the queue
 *
 * Return: 0 on success, negative error code on failure
 *****************************************************************************/
static int qdma_cpm4_sw_context_clear(void *dev_hndl, uint8_t c2h,
                                      uint16_t hw_qid)
{
    enum ind_ctxt_cmd_sel sel = c2h ? QDMA_CTXT_SEL_SW_C2H : QDMA_CTXT_SEL_SW_H2C;

    // MD: Validate input parameter
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n", __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Clear SW context in hardware
    return qdma_cpm4_indirect_reg_clear(dev_hndl, sel, hw_qid);
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_sw_context_invalidate() - Invalidate SW context
 *
 * This function invalidates the software context for a given queue.
 *
 * @dev_hndl: Device handle
 * @c2h: Is C2H queue
 * @hw_qid: Hardware QID of the queue
 *
 * Return: 0 on success, negative error code on failure
 *****************************************************************************/
static int qdma_cpm4_sw_context_invalidate(void *dev_hndl, uint8_t c2h,
                                           uint16_t hw_qid)
{
    enum ind_ctxt_cmd_sel sel = c2h ? QDMA_CTXT_SEL_SW_C2H : QDMA_CTXT_SEL_SW_H2C;

    // MD: Validate input parameter
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n", __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Invalidate SW context in hardware
    return qdma_cpm4_indirect_reg_invalidate(dev_hndl, sel, hw_qid);
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_sw_ctx_conf() - Configure SW context
 *
 * This function configures the software context based on the specified access type.
 *
 * @dev_hndl: Device handle
 * @c2h: Is C2H queue
 * @hw_qid: Hardware QID of the queue
 * @ctxt: Pointer to the context data
 * @access_type: HW access type (qdma_hw_access_type enum) value
 *
 * Return: 0 on success, negative error code on failure
 *****************************************************************************/
int qdma_cpm4_sw_ctx_conf(void *dev_hndl, uint8_t c2h, uint16_t hw_qid,
                          struct qdma_descq_sw_ctxt *ctxt,
                          enum qdma_hw_access_type access_type)
{
    int ret_val = 0;

    // MD: Configure SW context based on access type
    switch (access_type) {
    case QDMA_HW_ACCESS_READ:
        ret_val = qdma_cpm4_sw_context_read(dev_hndl, c2h, hw_qid, ctxt);
        break;
    case QDMA_HW_ACCESS_WRITE:
        ret_val = qdma_cpm4_sw_context_write(dev_hndl, c2h, hw_qid, ctxt);
        break;
    case QDMA_HW_ACCESS_CLEAR:
        ret_val = qdma_cpm4_sw_context_clear(dev_hndl, c2h, hw_qid);
        break;
    case QDMA_HW_ACCESS_INVALIDATE:
        ret_val = qdma_cpm4_sw_context_invalidate(dev_hndl, c2h, hw_qid);
        break;
    default:
        qdma_log_error("%s: access_type=%d is invalid, err:%d\n",
                       __func__, access_type, -QDMA_ERR_INV_PARAM);
        ret_val = -QDMA_ERR_INV_PARAM;
        break;
    }
    return ret_val;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_pfetch_context_write() - Create prefetch context and program it
 *
 * This function writes the prefetch context for a given queue.
 * It prepares the context data and writes it to the hardware.
 *
 * @dev_hndl: Device handle
 * @hw_qid: Hardware QID of the queue
 * @ctxt: Pointer to the prefetch context data structure
 *
 * Return: 0 on success, negative error code on failure
 *****************************************************************************/
static int qdma_cpm4_pfetch_context_write(void *dev_hndl, uint16_t hw_qid,
                                          const struct qdma_descq_prefetch_ctxt *ctxt)
{
    uint32_t pfetch_ctxt[QDMA_CPM4_PFETCH_CONTEXT_NUM_WORDS] = {0};
    enum ind_ctxt_cmd_sel sel = QDMA_CTXT_SEL_PFTCH;
    uint32_t sw_crdt_l, sw_crdt_h;
    uint16_t num_words_count = 0;

    // MD: Validate input parameters
    if (!dev_hndl || !ctxt) {
        qdma_log_error("%s: dev_hndl=%p pfetch_ctxt=%p, err:%d\n",
                       __func__, dev_hndl, ctxt, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Extract software credit fields
    sw_crdt_l = FIELD_GET(QDMA_PFTCH_CTXT_SW_CRDT_GET_L_MASK, ctxt->sw_crdt);
    sw_crdt_h = FIELD_GET(QDMA_PFTCH_CTXT_SW_CRDT_GET_H_MASK, ctxt->sw_crdt);

    // MD: Prepare prefetch context data
    pfetch_ctxt[num_words_count++] =
        FIELD_SET(PREFETCH_CTXT_DATA_W0_BYPASS_MASK, ctxt->bypass) |
        FIELD_SET(PREFETCH_CTXT_DATA_W0_BUF_SIZE_IDX_MASK, ctxt->bufsz_idx) |
        FIELD_SET(PREFETCH_CTXT_DATA_W0_PORT_ID_MASK, ctxt->port_id) |
        FIELD_SET(PREFETCH_CTXT_DATA_W0_ERR_MASK, ctxt->err) |
        FIELD_SET(PREFETCH_CTXT_DATA_W0_PFCH_EN_MASK, ctxt->pfch_en) |
        FIELD_SET(PREFETCH_CTXT_DATA_W0_PFCH_MASK, ctxt->pfch) |
        FIELD_SET(PREFETCH_CTXT_DATA_W0_SW_CRDT_L_MASK, sw_crdt_l);

    pfetch_ctxt[num_words_count++] =
        FIELD_SET(PREFETCH_CTXT_DATA_W1_SW_CRDT_H_MASK, sw_crdt_h) |
        FIELD_SET(PREFETCH_CTXT_DATA_W1_VALID_MASK, ctxt->valid);

    // MD: Write prefetch context to hardware
    return qdma_cpm4_indirect_reg_write(dev_hndl, sel, hw_qid, pfetch_ctxt, num_words_count);
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_pfetch_context_read() - Read prefetch context
 *
 * This function reads the prefetch context for a given queue and populates the output structure.
 *
 * @dev_hndl: Device handle
 * @hw_qid: Hardware QID of the queue
 * @ctxt: Pointer to the output context data
 *
 * Return: 0 on success, negative error code on failure
 *****************************************************************************/
static int qdma_cpm4_pfetch_context_read(void *dev_hndl, uint16_t hw_qid,
                                         struct qdma_descq_prefetch_ctxt *ctxt)
{
    int rv = 0;
    uint32_t pfetch_ctxt[QDMA_CPM4_PFETCH_CONTEXT_NUM_WORDS] = {0};
    enum ind_ctxt_cmd_sel sel = QDMA_CTXT_SEL_PFTCH;
    uint32_t sw_crdt_l, sw_crdt_h;

    // MD: Validate input parameters
    if (!dev_hndl || !ctxt) {
        qdma_log_error("%s: dev_hndl=%p pfetch_ctxt=%p, err:%d\n",
                       __func__, dev_hndl, ctxt, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Read prefetch context from hardware
    rv = qdma_cpm4_indirect_reg_read(dev_hndl, sel, hw_qid,
                                     QDMA_CPM4_PFETCH_CONTEXT_NUM_WORDS, pfetch_ctxt);
    if (rv < 0)
        return rv;

    // MD: Populate output structure with read values
    ctxt->bypass = (uint8_t)(FIELD_GET(PREFETCH_CTXT_DATA_W0_BYPASS_MASK, pfetch_ctxt[0]));
    ctxt->bufsz_idx = (uint8_t)(FIELD_GET(PREFETCH_CTXT_DATA_W0_BUF_SIZE_IDX_MASK, pfetch_ctxt[0]));
    ctxt->port_id = (uint8_t)(FIELD_GET(PREFETCH_CTXT_DATA_W0_PORT_ID_MASK, pfetch_ctxt[0]));
    ctxt->err = (uint8_t)(FIELD_GET(PREFETCH_CTXT_DATA_W0_ERR_MASK, pfetch_ctxt[0]));
    ctxt->pfch_en = (uint8_t)(FIELD_GET(PREFETCH_CTXT_DATA_W0_PFCH_EN_MASK, pfetch_ctxt[0]));
    ctxt->pfch = (uint8_t)(FIELD_GET(PREFETCH_CTXT_DATA_W0_PFCH_MASK, pfetch_ctxt[0]));
    sw_crdt_l = (uint32_t)FIELD_GET(PREFETCH_CTXT_DATA_W0_SW_CRDT_L_MASK, pfetch_ctxt[0]);

    sw_crdt_h = (uint32_t)FIELD_GET(PREFETCH_CTXT_DATA_W1_SW_CRDT_H_MASK, pfetch_ctxt[1]);
    ctxt->valid = (uint8_t)(FIELD_GET(PREFETCH_CTXT_DATA_W1_VALID_MASK, pfetch_ctxt[1]));

    ctxt->sw_crdt = (uint16_t)(FIELD_SET(QDMA_PFTCH_CTXT_SW_CRDT_GET_L_MASK, sw_crdt_l) |
                               FIELD_SET(QDMA_PFTCH_CTXT_SW_CRDT_GET_H_MASK, sw_crdt_h));

    return QDMA_SUCCESS;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_pfetch_context_clear() - Clear prefetch context
 *
 * This function clears the prefetch context for a given queue.
 *
 * @dev_hndl: Device handle
 * @hw_qid: Hardware QID of the queue
 *
 * Return: 0 on success, negative error code on failure
 *****************************************************************************/
static int qdma_cpm4_pfetch_context_clear(void *dev_hndl, uint16_t hw_qid)
{
    enum ind_ctxt_cmd_sel sel = QDMA_CTXT_SEL_PFTCH;

    // MD: Validate input parameter
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n", __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Clear prefetch context in hardware
    return qdma_cpm4_indirect_reg_clear(dev_hndl, sel, hw_qid);
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_pfetch_context_invalidate() - Invalidate prefetch context
 *
 * This function invalidates the prefetch context for a given queue.
 *
 * @dev_hndl: Device handle
 * @hw_qid: Hardware QID of the queue
 *
 * Return: 0 on success, negative error code on failure
 *****************************************************************************/
static int qdma_cpm4_pfetch_context_invalidate(void *dev_hndl, uint16_t hw_qid)
{
    enum ind_ctxt_cmd_sel sel = QDMA_CTXT_SEL_PFTCH;

    // MD: Validate input parameter
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n", __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Invalidate prefetch context in hardware
    return qdma_cpm4_indirect_reg_invalidate(dev_hndl, sel, hw_qid);
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_pfetch_ctx_conf() - Configure prefetch context
 *
 * This function configures the prefetch context based on the specified access type.
 *
 * @dev_hndl: Device handle
 * @hw_qid: Hardware QID of the queue
 * @ctxt: Pointer to context data
 * @access_type: HW access type (qdma_hw_access_type enum) value
 *
 * Return: 0 on success, negative error code on failure
 *****************************************************************************/
int qdma_cpm4_pfetch_ctx_conf(void *dev_hndl, uint16_t hw_qid,
                              struct qdma_descq_prefetch_ctxt *ctxt,
                              enum qdma_hw_access_type access_type)
{
    int ret_val = 0;

    // MD: Configure prefetch context based on access type
    switch (access_type) {
    case QDMA_HW_ACCESS_READ:
        ret_val = qdma_cpm4_pfetch_context_read(dev_hndl, hw_qid, ctxt);
        break;
    case QDMA_HW_ACCESS_WRITE:
        ret_val = qdma_cpm4_pfetch_context_write(dev_hndl, hw_qid, ctxt);
        break;
    case QDMA_HW_ACCESS_CLEAR:
        ret_val = qdma_cpm4_pfetch_context_clear(dev_hndl, hw_qid);
        break;
    case QDMA_HW_ACCESS_INVALIDATE:
        ret_val = qdma_cpm4_pfetch_context_invalidate(dev_hndl, hw_qid);
        break;
    default:
        qdma_log_error("%s: access_type=%d is invalid, err:%d\n",
                       __func__, access_type, -QDMA_ERR_INV_PARAM);
        ret_val = -QDMA_ERR_INV_PARAM;
        break;
    }

    return ret_val;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_cmpt_context_write() - Create completion context and program it
 *
 * This function writes the completion context for a given queue.
 * It validates the input parameters, prepares the context data, and writes it to the hardware.
 *
 * @dev_hndl: Device handle
 * @hw_qid: Hardware QID of the queue
 * @ctxt: Pointer to the completion context data structure
 *
 * Return: 0 on success, negative error code on failure
 *****************************************************************************/
static int qdma_cpm4_cmpt_context_write(void *dev_hndl, uint16_t hw_qid,
                                        const struct qdma_descq_cmpt_ctxt *ctxt)
{
    uint32_t cmpt_ctxt[QDMA_CPM4_CMPT_CONTEXT_NUM_WORDS] = {0};
    uint16_t num_words_count = 0;
    uint32_t baddr_l, baddr_h, baddr_m, pidx_l, pidx_h;
    enum ind_ctxt_cmd_sel sel = QDMA_CTXT_SEL_CMPT;

    // MD: Validate input parameters
    if (!dev_hndl || !ctxt) {
        qdma_log_error("%s: dev_hndl=%p cmpt_ctxt=%p, err:%d\n",
                       __func__, dev_hndl, ctxt, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Validate descriptor size, ring size index, counter index, timer index, and trigger mode
    if ((ctxt->desc_sz > QDMA_DESC_SIZE_32B) ||
        (ctxt->ringsz_idx >= QDMA_NUM_RING_SIZES) ||
        (ctxt->counter_idx >= QDMA_NUM_C2H_COUNTERS) ||
        (ctxt->timer_idx >= QDMA_NUM_C2H_TIMERS) ||
        (ctxt->trig_mode > QDMA_CMPT_UPDATE_TRIG_MODE_TMR_CNTR)) {
        qdma_log_error("%s Inv dsz(%d)/ridx(%d)/cntr(%d)/tmr(%d)/tm(%d), err:%d\n",
                       __func__, ctxt->desc_sz, ctxt->ringsz_idx, ctxt->counter_idx,
                       ctxt->timer_idx, ctxt->trig_mode, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Extract base address and producer index fields
    baddr_l = (uint32_t)FIELD_GET(QDMA_CPM4_COMPL_CTXT_BADDR_GET_L_MASK, ctxt->bs_addr);
    baddr_m = (uint32_t)FIELD_GET(QDMA_CPM4_COMPL_CTXT_BADDR_GET_M_MASK, ctxt->bs_addr);
    baddr_h = (uint32_t)FIELD_GET(QDMA_CPM4_COMPL_CTXT_BADDR_GET_H_MASK, ctxt->bs_addr);

    pidx_l = FIELD_GET(QDMA_CPM4_COMPL_CTXT_PIDX_GET_L_MASK, ctxt->pidx);
    pidx_h = FIELD_GET(QDMA_CPM4_COMPL_CTXT_PIDX_GET_H_MASK, ctxt->pidx);

    // MD: Prepare completion context data
    cmpt_ctxt[num_words_count++] =
        FIELD_SET(CMPL_CTXT_DATA_W0_EN_STAT_DESC_MASK, ctxt->en_stat_desc) |
        FIELD_SET(CMPL_CTXT_DATA_W0_EN_INT_MASK, ctxt->en_int) |
        FIELD_SET(CMPL_CTXT_DATA_W0_TRIG_MODE_MASK, ctxt->trig_mode) |
        FIELD_SET(CMPL_CTXT_DATA_W0_FNC_ID_MASK, ctxt->fnc_id) |
        FIELD_SET(CMPL_CTXT_DATA_W0_CNTER_IDX_MASK, ctxt->counter_idx) |
        FIELD_SET(CMPL_CTXT_DATA_W0_TIMER_IDX_MASK, ctxt->timer_idx) |
        FIELD_SET(CMPL_CTXT_DATA_W0_INT_ST_MASK, ctxt->in_st) |
        FIELD_SET(CMPL_CTXT_DATA_W0_COLOR_MASK, ctxt->color) |
        FIELD_SET(CMPL_CTXT_DATA_W0_QSIZE_IDX_MASK, ctxt->ringsz_idx) |
        FIELD_SET(CMPL_CTXT_DATA_W0_BADDR_64_L_MASK, baddr_l);

    cmpt_ctxt[num_words_count++] =
        FIELD_SET(CMPL_CTXT_DATA_W1_BADDR_64_M_MASK, baddr_m);

    cmpt_ctxt[num_words_count++] =
        FIELD_SET(CMPL_CTXT_DATA_W2_BADDR_64_H_MASK, baddr_h) |
        FIELD_SET(CMPL_CTXT_DATA_W2_DESC_SIZE_MASK, ctxt->desc_sz) |
        FIELD_SET(CMPL_CTXT_DATA_W2_PIDX_L_MASK, pidx_l);

    cmpt_ctxt[num_words_count++] =
        FIELD_SET(CMPL_CTXT_DATA_W3_PIDX_H_MASK, pidx_h) |
        FIELD_SET(CMPL_CTXT_DATA_W3_CIDX_MASK, ctxt->cidx) |
        FIELD_SET(CMPL_CTXT_DATA_W3_VALID_MASK, ctxt->valid) |
        FIELD_SET(CMPL_CTXT_DATA_W3_ERR_MASK, ctxt->err) |
        FIELD_SET(CMPL_CTXT_DATA_W3_USER_TRIG_PEND_MASK, ctxt->user_trig_pend) |
        FIELD_SET(CMPL_CTXT_DATA_W3_TIMER_RUNNING_MASK, ctxt->timer_running) |
        FIELD_SET(CMPL_CTXT_DATA_W3_FULL_UPD_MASK, ctxt->full_upd);

    // MD: Write completion context to hardware
    return qdma_cpm4_indirect_reg_write(dev_hndl, sel, hw_qid, cmpt_ctxt, num_words_count);
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_cmpt_context_read() - Read completion context
 *
 * This function reads the completion context for a given queue and populates the output structure.
 *
 * @dev_hndl: Device handle
 * @hw_qid: Hardware QID of the queue
 * @ctxt: Pointer to the context data
 *
 * Return: 0 on success, negative error code on failure
 *****************************************************************************/
static int qdma_cpm4_cmpt_context_read(void *dev_hndl, uint16_t hw_qid,
                                       struct qdma_descq_cmpt_ctxt *ctxt)
{
    int rv = 0;
    uint32_t cmpt_ctxt[QDMA_CPM4_CMPT_CONTEXT_NUM_WORDS] = {0};
    enum ind_ctxt_cmd_sel sel = QDMA_CTXT_SEL_CMPT;
    uint32_t baddr_l, baddr_h, baddr_m, pidx_l, pidx_h;

    // MD: Validate input parameters
    if (!dev_hndl || !ctxt) {
        qdma_log_error("%s: dev_hndl=%p cmpt_ctxt=%p, err:%d\n",
                       __func__, dev_hndl, ctxt, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Read completion context from hardware
    rv = qdma_cpm4_indirect_reg_read(dev_hndl, sel, hw_qid,
                                     QDMA_CPM4_CMPT_CONTEXT_NUM_WORDS, cmpt_ctxt);
    if (rv < 0)
        return rv;

    // MD: Populate output structure with read values
    ctxt->en_stat_desc = FIELD_GET(CMPL_CTXT_DATA_W0_EN_STAT_DESC_MASK, cmpt_ctxt[0]);
    ctxt->en_int = FIELD_GET(CMPL_CTXT_DATA_W0_EN_INT_MASK, cmpt_ctxt[0]);
    ctxt->trig_mode = FIELD_GET(CMPL_CTXT_DATA_W0_TRIG_MODE_MASK, cmpt_ctxt[0]);
    ctxt->fnc_id = (uint8_t)(FIELD_GET(CMPL_CTXT_DATA_W0_FNC_ID_MASK, cmpt_ctxt[0]));
    ctxt->counter_idx = (uint8_t)(FIELD_GET(CMPL_CTXT_DATA_W0_CNTER_IDX_MASK, cmpt_ctxt[0]));
    ctxt->timer_idx = (uint8_t)(FIELD_GET(CMPL_CTXT_DATA_W0_TIMER_IDX_MASK, cmpt_ctxt[0]));
    ctxt->in_st = (uint8_t)(FIELD_GET(CMPL_CTXT_DATA_W0_INT_ST_MASK, cmpt_ctxt[0]));
    ctxt->color = (uint8_t)(FIELD_GET(CMPL_CTXT_DATA_W0_COLOR_MASK, cmpt_ctxt[0]));
    ctxt->ringsz_idx = (uint8_t)(FIELD_GET(CMPL_CTXT_DATA_W0_QSIZE_IDX_MASK, cmpt_ctxt[0]));

    baddr_l = FIELD_GET(CMPL_CTXT_DATA_W0_BADDR_64_L_MASK, cmpt_ctxt[0]);
    baddr_m = FIELD_GET(CMPL_CTXT_DATA_W1_BADDR_64_M_MASK, cmpt_ctxt[1]);
    baddr_h = FIELD_GET(CMPL_CTXT_DATA_W2_BADDR_64_H_MASK, cmpt_ctxt[2]);

    ctxt->desc_sz = (uint8_t)(FIELD_GET(CMPL_CTXT_DATA_W2_DESC_SIZE_MASK, cmpt_ctxt[2]));
    pidx_l = FIELD_GET(CMPL_CTXT_DATA_W2_PIDX_L_MASK, cmpt_ctxt[2]);

    pidx_h = FIELD_GET(CMPL_CTXT_DATA_W3_PIDX_H_MASK, cmpt_ctxt[3]);
    ctxt->cidx = (uint16_t)(FIELD_GET(CMPL_CTXT_DATA_W3_CIDX_MASK, cmpt_ctxt[3]));
    ctxt->valid = (uint8_t)(FIELD_GET(CMPL_CTXT_DATA_W3_VALID_MASK, cmpt_ctxt[3]));
    ctxt->err = (uint8_t)(FIELD_GET(CMPL_CTXT_DATA_W3_ERR_MASK, cmpt_ctxt[3]));
    ctxt->user_trig_pend = (uint8_t)(FIELD_GET(CMPL_CTXT_DATA_W3_USER_TRIG_PEND_MASK, cmpt_ctxt[3]));

    ctxt->timer_running = (uint8_t)(FIELD_GET(CMPL_CTXT_DATA_W3_TIMER_RUNNING_MASK, cmpt_ctxt[3]));
    ctxt->full_upd = (uint8_t)(FIELD_GET(CMPL_CTXT_DATA_W3_FULL_UPD_MASK, cmpt_ctxt[3]));

    ctxt->bs_addr = FIELD_SET(QDMA_CPM4_COMPL_CTXT_BADDR_GET_L_MASK, (uint64_t)baddr_l) |
                    FIELD_SET(QDMA_CPM4_COMPL_CTXT_BADDR_GET_M_MASK, (uint64_t)baddr_m) |
                    FIELD_SET(QDMA_CPM4_COMPL_CTXT_BADDR_GET_H_MASK, (uint64_t)baddr_h);

    ctxt->pidx = (uint16_t)(FIELD_SET(QDMA_CPM4_COMPL_CTXT_PIDX_GET_L_MASK, pidx_l) |
                            FIELD_SET(QDMA_CPM4_COMPL_CTXT_PIDX_GET_H_MASK, pidx_h));

    return QDMA_SUCCESS;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_cmpt_context_clear() - Clear completion context
 *
 * This function clears the completion context for a given queue.
 *
 * @dev_hndl: Device handle
 * @hw_qid: Hardware QID of the queue
 *
 * Return: 0 on success, negative error code on failure
 *****************************************************************************/
static int qdma_cpm4_cmpt_context_clear(void *dev_hndl, uint16_t hw_qid)
{
    enum ind_ctxt_cmd_sel sel = QDMA_CTXT_SEL_CMPT;

    // MD: Validate input parameter
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n", __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Clear completion context in hardware
    return qdma_cpm4_indirect_reg_clear(dev_hndl, sel, hw_qid);
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_cmpt_context_invalidate() - Invalidate completion context
 *
 * This function invalidates the completion context for a given queue.
 *
 * @dev_hndl: Device handle
 * @hw_qid: Hardware QID of the queue
 *
 * Return: 0 on success, negative error code on failure
 *****************************************************************************/
static int qdma_cpm4_cmpt_context_invalidate(void *dev_hndl, uint16_t hw_qid)
{
    enum ind_ctxt_cmd_sel sel = QDMA_CTXT_SEL_CMPT;

    // MD: Validate input parameter
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n", __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Invalidate completion context in hardware
    return qdma_cpm4_indirect_reg_invalidate(dev_hndl, sel, hw_qid);
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_cmpt_ctx_conf() - Configure completion context
 *
 * This function configures the completion context based on the specified access type.
 *
 * @dev_hndl: Device handle
 * @hw_qid: Hardware QID of the queue
 * @ctxt: Pointer to context data
 * @access_type: HW access type (qdma_hw_access_type enum) value
 *
 * Return: 0 on success, negative error code on failure
 *****************************************************************************/
int qdma_cpm4_cmpt_ctx_conf(void *dev_hndl, uint16_t hw_qid,
                            struct qdma_descq_cmpt_ctxt *ctxt,
                            enum qdma_hw_access_type access_type)
{
    int ret_val = 0;

    // MD: Validate input parameter
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n", __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Configure completion context based on access type
    switch (access_type) {
    case QDMA_HW_ACCESS_READ:
        ret_val = qdma_cpm4_cmpt_context_read(dev_hndl, hw_qid, ctxt);
        break;
    case QDMA_HW_ACCESS_WRITE:
        ret_val = qdma_cpm4_cmpt_context_write(dev_hndl, hw_qid, ctxt);
        break;
    case QDMA_HW_ACCESS_CLEAR:
        ret_val = qdma_cpm4_cmpt_context_clear(dev_hndl, hw_qid);
        break;
    case QDMA_HW_ACCESS_INVALIDATE:
        ret_val = qdma_cpm4_cmpt_context_invalidate(dev_hndl, hw_qid);
        break;
    default:
        qdma_log_error("%s: access_type=%d is invalid, err:%d\n", __func__, access_type, -QDMA_ERR_INV_PARAM);
        ret_val = -QDMA_ERR_INV_PARAM;
        break;
    }

    return ret_val;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_hw_context_read() - Read hardware context
 *
 * This function reads the hardware context for a given queue and populates the output structure.
 *
 * @dev_hndl: Device handle
 * @c2h: Is C2H queue
 * @hw_qid: Hardware QID of the queue
 * @ctxt: Pointer to the output context data
 *
 * Return: 0 on success, negative error code on failure
 *****************************************************************************/
static int qdma_cpm4_hw_context_read(void *dev_hndl, uint8_t c2h,
                                     uint16_t hw_qid, struct qdma_descq_hw_ctxt *ctxt)
{
    int rv = 0;
    uint32_t hw_ctxt[QDMA_CPM4_HW_CONTEXT_NUM_WORDS] = {0};
    enum ind_ctxt_cmd_sel sel = c2h ? QDMA_CTXT_SEL_HW_C2H : QDMA_CTXT_SEL_HW_H2C;

    // MD: Validate input parameters
    if (!dev_hndl || !ctxt) {
        qdma_log_error("%s: dev_hndl=%p hw_ctxt=%p, err:%d\n", __func__, dev_hndl, ctxt, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Read hardware context from hardware
    rv = qdma_cpm4_indirect_reg_read(dev_hndl, sel, hw_qid, QDMA_CPM4_HW_CONTEXT_NUM_WORDS, hw_ctxt);
    if (rv < 0)
        return rv;

    // MD: Populate output structure with read values
    ctxt->cidx = FIELD_GET(HW_IND_CTXT_DATA_W0_CIDX_MASK, hw_ctxt[0]);
    ctxt->crd_use = (uint16_t)(FIELD_GET(HW_IND_CTXT_DATA_W0_CRD_USE_MASK, hw_ctxt[0]));

    ctxt->dsc_pend = (uint8_t)(FIELD_GET(HW_IND_CTXT_DATA_W1_DSC_PND_MASK, hw_ctxt[1]));
    ctxt->idl_stp_b = (uint8_t)(FIELD_GET(HW_IND_CTXT_DATA_W1_IDL_STP_B_MASK, hw_ctxt[1]));
    ctxt->fetch_pnd = (uint8_t)(FIELD_GET(HW_IND_CTXT_DATA_W1_FETCH_PND_MASK, hw_ctxt[1]));

    return QDMA_SUCCESS;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_hw_context_clear() - Clear hardware context
 *
 * This function clears the hardware context for a given queue.
 *
 * @dev_hndl: Device handle
 * @c2h: Is C2H queue
 * @hw_qid: Hardware QID of the queue
 *
 * Return: 0 on success, negative error code on failure
 *****************************************************************************/
static int qdma_cpm4_hw_context_clear(void *dev_hndl, uint8_t c2h, uint16_t hw_qid)
{
    enum ind_ctxt_cmd_sel sel = c2h ? QDMA_CTXT_SEL_HW_C2H : QDMA_CTXT_SEL_HW_H2C;

    // MD: Validate input parameter
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n", __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Clear hardware context in hardware
    return qdma_cpm4_indirect_reg_clear(dev_hndl, sel, hw_qid);
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_hw_context_invalidate() - Invalidate hardware context
 *
 * This function invalidates the hardware context for a given queue.
 *
 * @dev_hndl: Device handle
 * @c2h: Is C2H queue
 * @hw_qid: Hardware QID of the queue
 *
 * Return: 0 on success, negative error code on failure
 *****************************************************************************/
static int qdma_cpm4_hw_context_invalidate(void *dev_hndl, uint8_t c2h, uint16_t hw_qid)
{
    enum ind_ctxt_cmd_sel sel = c2h ? QDMA_CTXT_SEL_HW_C2H : QDMA_CTXT_SEL_HW_H2C;

    // MD: Validate input parameter
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n", __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Invalidate hardware context in hardware
    return qdma_cpm4_indirect_reg_invalidate(dev_hndl, sel, hw_qid);
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_hw_ctx_conf() - Configure HW context
 *
 * This function configures the hardware context based on the specified access type.
 * Note: QDMA_HW_ACCESS_WRITE is unsupported.
 *
 * @dev_hndl: Device handle
 * @c2h: Is C2H queue
 * @hw_qid: Hardware QID of the queue
 * @ctxt: Pointer to context data
 * @access_type: HW access type (qdma_hw_access_type enum) value
 *
 * Return: 0 on success, negative error code on failure
 *****************************************************************************/
int qdma_cpm4_hw_ctx_conf(void *dev_hndl, uint8_t c2h, uint16_t hw_qid,
                          struct qdma_descq_hw_ctxt *ctxt,
                          enum qdma_hw_access_type access_type)
{
    int ret_val = 0;

    // MD: Configure hardware context based on access type
    switch (access_type) {
    case QDMA_HW_ACCESS_READ:
        ret_val = qdma_cpm4_hw_context_read(dev_hndl, c2h, hw_qid, ctxt);
        break;
    case QDMA_HW_ACCESS_CLEAR:
        ret_val = qdma_cpm4_hw_context_clear(dev_hndl, c2h, hw_qid);
        break;
    case QDMA_HW_ACCESS_INVALIDATE:
        ret_val = qdma_cpm4_hw_context_invalidate(dev_hndl, c2h, hw_qid);
        break;
    case QDMA_HW_ACCESS_WRITE:
    default:
        qdma_log_error("%s: access_type=%d is invalid, err:%d\n", __func__, access_type, -QDMA_ERR_INV_PARAM);
        ret_val = -QDMA_ERR_INV_PARAM;
        break;
    }

    return ret_val;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_indirect_intr_context_write() - Create indirect interrupt context and program it
 *
 * This function writes the indirect interrupt context for a given ring index.
 * It prepares the context data and writes it to the hardware.
 *
 * @dev_hndl: Device handle
 * @ring_index: Indirect interrupt ring index
 * @ctxt: Pointer to the interrupt context data structure
 *
 * Return: 0 on success, negative error code on failure
 *****************************************************************************/
static int qdma_cpm4_indirect_intr_context_write(void *dev_hndl, uint16_t ring_index,
                                                 const struct qdma_indirect_intr_ctxt *ctxt)
{
    uint32_t intr_ctxt[QDMA_CPM4_IND_INTR_CONTEXT_NUM_WORDS] = {0};
    enum ind_ctxt_cmd_sel sel = QDMA_CTXT_SEL_INT_COAL;
    uint16_t num_words_count = 0;
    uint32_t baddr_l, baddr_h;

    // MD: Validate input parameters
    if (!dev_hndl || !ctxt) {
        qdma_log_error("%s: dev_hndl=%p intr_ctxt=%p, err:%d\n", __func__, dev_hndl, ctxt, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Validate page size
    if (ctxt->page_size > QDMA_INDIRECT_INTR_RING_SIZE_32KB) {
        qdma_log_error("%s: ctxt->page_size=%u is too big, err:%d\n", __func__, ctxt->page_size, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Extract base address fields
    baddr_l = (uint32_t)FIELD_GET(QDMA_CPM4_INTR_CTXT_BADDR_GET_L_MASK, ctxt->baddr_4k);
    baddr_h = (uint32_t)FIELD_GET(QDMA_CPM4_INTR_CTXT_BADDR_GET_H_MASK, ctxt->baddr_4k);

    // MD: Prepare interrupt context data
    intr_ctxt[num_words_count++] =
        FIELD_SET(INTR_CTXT_DATA_W0_VALID_MASK, ctxt->valid) |
        FIELD_SET(INTR_CTXT_DATA_W0_VEC_MASK, ctxt->vec) |
        FIELD_SET(INTR_CTXT_DATA_W0_INT_ST_MASK, ctxt->int_st) |
        FIELD_SET(INTR_CTXT_DATA_W0_COLOR_MASK, ctxt->color) |
        FIELD_SET(INTR_CTXT_DATA_W0_BADDR_4K_L_MASK, baddr_l);

    intr_ctxt[num_words_count++] =
        FIELD_SET(INTR_CTXT_DATA_W1_BADDR_4K_H_MASK, baddr_h) |
        FIELD_SET(INTR_CTXT_DATA_W1_PAGE_SIZE_MASK, ctxt->page_size);

    intr_ctxt[num_words_count++] =
        FIELD_SET(INTR_CTXT_DATA_W2_PIDX_MASK, ctxt->pidx);

    // MD: Write interrupt context to hardware
    return qdma_cpm4_indirect_reg_write(dev_hndl, sel, ring_index, intr_ctxt, num_words_count);
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_indirect_intr_context_read() - Read indirect interrupt context
 *
 * This function reads the indirect interrupt context for a given ring index and populates the output structure.
 *
 * @dev_hndl: Device handle
 * @ring_index: Indirect interrupt ring index
 * @ctxt: Pointer to the output context data
 *
 * Return: 0 on success, negative error code on failure
 *****************************************************************************/
static int qdma_cpm4_indirect_intr_context_read(void *dev_hndl, uint16_t ring_index,
                                                struct qdma_indirect_intr_ctxt *ctxt)
{
    int rv = 0;
    uint32_t intr_ctxt[QDMA_CPM4_IND_INTR_CONTEXT_NUM_WORDS] = {0};
    enum ind_ctxt_cmd_sel sel = QDMA_CTXT_SEL_INT_COAL;
    uint64_t baddr_l, baddr_h;

    // MD: Validate input parameters
    if (!dev_hndl || !ctxt) {
        qdma_log_error("%s: dev_hndl=%p intr_ctxt=%p, err:%d\n", __func__, dev_hndl, ctxt, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Read interrupt context from hardware
    rv = qdma_cpm4_indirect_reg_read(dev_hndl, sel, ring_index, QDMA_CPM4_IND_INTR_CONTEXT_NUM_WORDS, intr_ctxt);
    if (rv < 0)
        return rv;

    // MD: Populate output structure with read values
    ctxt->valid = FIELD_GET(INTR_CTXT_DATA_W0_VALID_MASK, intr_ctxt[0]);
    ctxt->vec = FIELD_GET(INTR_CTXT_DATA_W0_VEC_MASK, intr_ctxt[0]);
    ctxt->int_st = FIELD_GET(INTR_CTXT_DATA_W0_INT_ST_MASK, intr_ctxt[0]);
    ctxt->color = (uint8_t)(FIELD_GET(INTR_CTXT_DATA_W0_COLOR_MASK, intr_ctxt[0]));
    baddr_l = FIELD_GET(INTR_CTXT_DATA_W0_BADDR_4K_L_MASK, intr_ctxt[0]);

    baddr_h = FIELD_GET(INTR_CTXT_DATA_W1_BADDR_4K_H_MASK, intr_ctxt[1]);
    ctxt->page_size = (uint8_t)(FIELD_GET(INTR_CTXT_DATA_W1_PAGE_SIZE_MASK, intr_ctxt[1]));
    ctxt->pidx = FIELD_GET(INTR_CTXT_DATA_W2_PIDX_MASK, intr_ctxt[2]);

    ctxt->baddr_4k = FIELD_SET(QDMA_CPM4_INTR_CTXT_BADDR_GET_L_MASK, baddr_l) |
                     FIELD_SET(QDMA_CPM4_INTR_CTXT_BADDR_GET_H_MASK, baddr_h);

    return QDMA_SUCCESS;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_indirect_intr_context_clear() - Clear indirect interrupt context
 *
 * This function clears the indirect interrupt context for a given ring index.
 *
 * @dev_hndl: Device handle
 * @ring_index: Indirect interrupt ring index
 *
 * Return: 0 on success, negative error code on failure
 *****************************************************************************/
static int qdma_cpm4_indirect_intr_context_clear(void *dev_hndl, uint16_t ring_index)
{
    enum ind_ctxt_cmd_sel sel = QDMA_CTXT_SEL_INT_COAL;

    // MD: Validate input parameter
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n", __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Clear interrupt context in hardware
    return qdma_cpm4_indirect_reg_clear(dev_hndl, sel, ring_index);
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_indirect_intr_context_invalidate() - Invalidate indirect interrupt context
 *
 * This function invalidates the indirect interrupt context for a given ring index.
 *
 * @dev_hndl: Device handle
 * @ring_index: Indirect interrupt ring index
 *
 * Return: 0 on success, negative error code on failure
 *****************************************************************************/
static int qdma_cpm4_indirect_intr_context_invalidate(void *dev_hndl, uint16_t ring_index)
{
    enum ind_ctxt_cmd_sel sel = QDMA_CTXT_SEL_INT_COAL;

    // MD: Validate input parameter
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n", __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Invalidate interrupt context in hardware
    return qdma_cpm4_indirect_reg_invalidate(dev_hndl, sel, ring_index);
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_indirect_intr_ctx_conf() - Configure indirect interrupt context
 *
 * This function configures the indirect interrupt context based on the specified access type.
 *
 * @dev_hndl: Device handle
 * @ring_index: Indirect interrupt ring index
 * @ctxt: Pointer to context data
 * @access_type: HW access type (qdma_hw_access_type enum) value
 *
 * Return: 0 on success, negative error code on failure
 *****************************************************************************/
int qdma_cpm4_indirect_intr_ctx_conf(void *dev_hndl, uint16_t ring_index,
                                     struct qdma_indirect_intr_ctxt *ctxt,
                                     enum qdma_hw_access_type access_type)
{
    int ret_val = 0;

    // MD: Configure interrupt context based on access type
    switch (access_type) {
    case QDMA_HW_ACCESS_READ:
        ret_val = qdma_cpm4_indirect_intr_context_read(dev_hndl, ring_index, ctxt);
        break;
    case QDMA_HW_ACCESS_WRITE:
        ret_val = qdma_cpm4_indirect_intr_context_write(dev_hndl, ring_index, ctxt);
        break;
    case QDMA_HW_ACCESS_CLEAR:
        ret_val = qdma_cpm4_indirect_intr_context_clear(dev_hndl, ring_index);
        break;
    case QDMA_HW_ACCESS_INVALIDATE:
        ret_val = qdma_cpm4_indirect_intr_context_invalidate(dev_hndl, ring_index);
        break;
    default:
        qdma_log_error("%s: access_type=%d is invalid, err:%d\n", __func__, access_type, -QDMA_ERR_INV_PARAM);
        ret_val = -QDMA_ERR_INV_PARAM;
        break;
    }

    return ret_val;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_set_default_global_csr() - Set global CSR register to default values
 *
 * This function sets the global CSR register to default values. The values can be modified later using the set/get CSR functions.
 *
 * @dev_hndl: Device handle
 *
 * Return: 0 on success, negative error code on failure
 *****************************************************************************/
int qdma_cpm4_set_default_global_csr(void *dev_hndl)
{
    // MD: Default values
    uint32_t reg_val = 0;
    uint32_t rng_sz[QDMA_NUM_RING_SIZES] = {2049, 65, 129, 193, 257,
                                            385, 513, 769, 1025, 1537, 3073, 4097, 6145,
                                            8193, 12289, 16385};
    uint32_t tmr_cnt[QDMA_NUM_C2H_TIMERS] = {1, 2, 4, 5, 8, 10, 15, 20, 25,
                                             30, 50, 75, 100, 125, 150, 200};
    uint32_t cnt_th[QDMA_NUM_C2H_COUNTERS] = {2, 4, 8, 16, 24,
                                              32, 48, 64, 80, 96, 112, 128, 144,
                                              160, 176, 192};
    uint32_t buf_sz[QDMA_NUM_C2H_BUFFER_SIZES] = {4096, 256, 512, 1024,
                                                  2048, 3968, 4096, 4096, 4096, 4096, 4096, 4096,
                                                  4096, 8192, 9018, 16384};
    struct qdma_dev_attributes dev_cap;

    // MD: Validate input parameter
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n", __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Get device attributes
    qdma_cpm4_get_device_attributes(dev_hndl, &dev_cap);

    // MD: Configuring CSR registers
    // MD: Global ring sizes
    qdma_write_csr_values(dev_hndl, QDMA_CPM4_GLBL_RNG_SZ_1_ADDR, 0, QDMA_NUM_RING_SIZES, rng_sz);

    if (dev_cap.st_en || dev_cap.mm_cmpt_en) {
        // MD: Counter thresholds
        qdma_write_csr_values(dev_hndl, QDMA_CPM4_C2H_CNT_TH_1_ADDR, 0, QDMA_NUM_C2H_COUNTERS, cnt_th);

        // MD: Timer Counters
        qdma_write_csr_values(dev_hndl, QDMA_CPM4_C2H_TIMER_CNT_1_ADDR, 0, QDMA_NUM_C2H_TIMERS, tmr_cnt);

        // MD: Writeback Interval
        reg_val = FIELD_SET(GLBL_DSC_CFG_MAXFETCH_MASK, DEFAULT_MAX_DSC_FETCH) |
                  FIELD_SET(GLBL_DSC_CFG_WB_ACC_INT_MASK, DEFAULT_WRB_INT);
        qdma_reg_write(dev_hndl, QDMA_CPM4_GLBL_DSC_CFG_ADDR, reg_val);
    }

    if (dev_cap.st_en) {
        // MD: Buffer Sizes
        qdma_write_csr_values(dev_hndl, QDMA_CPM4_C2H_BUF_SZ_0_ADDR, 0, QDMA_NUM_C2H_BUFFER_SIZES, buf_sz);

        // MD: Prefetch Configuration
        reg_val = FIELD_SET(C2H_PFCH_CFG_FL_TH_MASK, QDMA_CPM4_DEFAULT_PFCH_STOP_THRESH) |
                  FIELD_SET(C2H_PFCH_CFG_NUM_MASK, DEFAULT_PFCH_NUM_ENTRIES_PER_Q) |
                  FIELD_SET(C2H_PFCH_CFG_QCNT_MASK, DEFAULT_PFCH_MAX_Q_CNT) |
                  FIELD_SET(C2H_PFCH_CFG_EVT_QCNT_TH_MASK, DEFAULT_C2H_INTR_TIMER_TICK);
        qdma_reg_write(dev_hndl, QDMA_CPM4_C2H_PFCH_CFG_ADDR, reg_val);

        // MD: C2H interrupt timer tick
        qdma_reg_write(dev_hndl, QDMA_CPM4_C2H_INT_TIMER_TICK_ADDR, DEFAULT_C2H_INTR_TIMER_TICK);

        // MD: C2H Completion Coalesce Configuration
        reg_val = FIELD_SET(C2H_WRB_COAL_CFG_TICK_CNT_MASK, DEFAULT_CMPT_COAL_TIMER_CNT) |
                  FIELD_SET(C2H_WRB_COAL_CFG_TICK_VAL_MASK, DEFAULT_CMPT_COAL_TIMER_TICK) |
                  FIELD_SET(C2H_WRB_COAL_CFG_MAX_BUF_SZ_MASK, DEFAULT_CMPT_COAL_MAX_BUF_SZ);
        qdma_reg_write(dev_hndl, QDMA_CPM4_C2H_WRB_COAL_CFG_ADDR, reg_val);

#if 0
        // MD: H2C throttle Configuration
        reg_val = FIELD_SET(QDMA_H2C_DATA_THRESH_MASK, DEFAULT_H2C_THROT_DATA_THRESH) |
                  FIELD_SET(QDMA_H2C_REQ_THROT_EN_DATA_MASK, DEFAULT_THROT_EN_DATA);
        qdma_reg_write(dev_hndl, QDMA_OFFSET_H2C_REQ_THROT, reg_val);
#endif
    }

    return QDMA_SUCCESS;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_queue_pidx_update() - Update the descriptor PIDX
 *
 * This function updates the descriptor PIDX for a given queue.
 *
 * @dev_hndl: Device handle
 * @is_vf: Whether PF or VF
 * @qid: Queue id relative to the PF/VF calling this API
 * @is_c2h: Queue direction. Set 1 for C2H and 0 for H2C
 * @reg_info: Data needed for the PIDX register update
 *
 * Return: 0 on success, negative error code on failure
 *****************************************************************************/
int qdma_cpm4_queue_pidx_update(void *dev_hndl, uint8_t is_vf, uint16_t qid,
                                uint8_t is_c2h, const struct qdma_q_pidx_reg_info *reg_info)
{
    uint32_t reg_addr = 0;
    uint32_t reg_val = 0;

    // MD: Validate input parameters
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n", __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }
    if (!reg_info) {
        qdma_log_error("%s: reg_info is NULL, err:%d\n", __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Determine register address based on queue type
    if (!is_vf) {
        reg_addr = (is_c2h) ? QDMA_CPM4_OFFSET_DMAP_SEL_C2H_DSC_PIDX : QDMA_CPM4_OFFSET_DMAP_SEL_H2C_DSC_PIDX;
    } else {
        reg_addr = (is_c2h) ? QDMA_CPM4_OFFSET_VF_DMAP_SEL_C2H_DSC_PIDX : QDMA_CPM4_OFFSET_VF_DMAP_SEL_H2C_DSC_PIDX;
    }

    reg_addr += (qid * QDMA_PIDX_STEP);

    // MD: Prepare register value
    reg_val = FIELD_SET(QDMA_CPM4_DMA_SEL_DESC_PIDX_MASK, reg_info->pidx) |
              FIELD_SET(QDMA_CPM4_DMA_SEL_IRQ_EN_MASK, reg_info->irq_en);

    // MD: Write PIDX register
    qdma_reg_write(dev_hndl, reg_addr, reg_val);

    return QDMA_SUCCESS;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_queue_cmpt_cidx_update() - Update the CMPT CIDX
 *
 * This function updates the Completion CIDX register for a given queue.
 *
 * @dev_hndl: Device handle
 * @is_vf: Whether PF or VF
 * @qid: Queue ID relative to the PF/VF calling this API
 * @reg_info: Data needed for the CIDX register update
 *
 * Return: 0 on success, negative error code on failure
 *****************************************************************************/
int qdma_cpm4_queue_cmpt_cidx_update(void *dev_hndl, uint8_t is_vf,
                                     uint16_t qid, const struct qdma_q_cmpt_cidx_reg_info *reg_info)
{
    uint32_t reg_addr = (is_vf) ? QDMA_CPM4_OFFSET_VF_DMAP_SEL_CMPT_CIDX : QDMA_CPM4_OFFSET_DMAP_SEL_CMPT_CIDX;
    uint32_t reg_val = 0;

    // MD: Validate input parameters
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n", __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    if (!reg_info) {
        qdma_log_error("%s: reg_info is NULL, err:%d\n", __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Calculate register address
    reg_addr += (qid * QDMA_CMPT_CIDX_STEP);

    // MD: Prepare register value
    reg_val =
        FIELD_SET(QDMA_CPM4_DMAP_SEL_CMPT_WRB_CIDX_MASK, reg_info->wrb_cidx) |
        FIELD_SET(QDMA_CPM4_DMAP_SEL_CMPT_CNT_THRESH_MASK, reg_info->counter_idx) |
        FIELD_SET(QDMA_CPM4_DMAP_SEL_CMPT_TMR_CNT_MASK, reg_info->timer_idx) |
        FIELD_SET(QDMA_CPM4_DMAP_SEL_CMPT_TRG_MODE_MASK, reg_info->trig_mode) |
        FIELD_SET(QDMA_CPM4_DMAP_SEL_CMPT_STS_DESC_EN_MASK, reg_info->wrb_en) |
        FIELD_SET(QDMA_CPM4_DMAP_SEL_CMPT_IRQ_EN_MASK, reg_info->irq_en);

    // MD: Write to register
    qdma_reg_write(dev_hndl, reg_addr, reg_val);

    return QDMA_SUCCESS;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_queue_intr_cidx_update() - Update the interrupt CIDX
 *
 * This function updates the Interrupt CIDX register for a given queue.
 *
 * @dev_hndl: Device handle
 * @is_vf: Whether PF or VF
 * @qid: Queue ID relative to the PF/VF calling this API
 * @reg_info: Data needed for the CIDX register update
 *
 * Return: 0 on success, negative error code on failure
 *****************************************************************************/
int qdma_cpm4_queue_intr_cidx_update(void *dev_hndl, uint8_t is_vf,
                                     uint16_t qid, const struct qdma_intr_cidx_reg_info *reg_info)
{
    uint32_t reg_addr = (is_vf) ? QDMA_CPM4_OFFSET_VF_DMAP_SEL_INT_CIDX : QDMA_CPM4_OFFSET_DMAP_SEL_INT_CIDX;
    uint32_t reg_val = 0;

    // MD: Validate input parameters
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n", __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    if (!reg_info) {
        qdma_log_error("%s: reg_info is NULL, err:%d\n", __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Calculate register address
    reg_addr += qid * QDMA_INT_CIDX_STEP;

    // MD: Prepare register value
    reg_val =
        FIELD_SET(QDMA_CPM4_DMA_SEL_INT_SW_CIDX_MASK, reg_info->sw_cidx) |
        FIELD_SET(QDMA_CPM4_DMA_SEL_INT_RING_IDX_MASK, reg_info->rng_idx);

    // MD: Write to register
    qdma_reg_write(dev_hndl, reg_addr, reg_val);

    return QDMA_SUCCESS;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cmp_get_user_bar() - Get the AXI Master Lite (user bar) number
 *
 * This function retrieves the AXI Master Lite (user bar) number for a given function ID.
 *
 * @dev_hndl: Device handle
 * @is_vf: Whether PF or VF
 * @func_id: Function ID of the PF
 * @user_bar: Pointer to hold the AXI Master Lite (user bar) number
 *
 * Return: 0 on success, negative error code on failure
 *****************************************************************************/
int qdma_cmp_get_user_bar(void *dev_hndl, uint8_t is_vf,
                          uint16_t func_id, uint8_t *user_bar)
{
    uint8_t bar_found = 0;
    uint8_t bar_idx = 0;
    uint32_t user_bar_id = 0;
    uint32_t reg_addr = 0;

    // MD: Validate input parameters
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n", __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    if (!user_bar) {
        qdma_log_error("%s: user_bar is NULL, err:%d\n", __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Determine register address based on VF or PF
    reg_addr = (is_vf) ? QDMA_CPM4_GLBL2_PF_VF_BARLITE_EXT_ADDR : QDMA_CPM4_GLBL2_PF_BARLITE_EXT_ADDR;

    if (!is_vf) {
        // MD: Read user bar ID from register
        user_bar_id = qdma_reg_read(dev_hndl, reg_addr);
        user_bar_id = (user_bar_id >> (6 * func_id)) & 0x3F;
    } else {
        *user_bar = QDMA_CPM4_VF_USER_BAR_ID;
        return QDMA_SUCCESS;
    }

    // MD: Find the user bar
    for (bar_idx = 0; bar_idx < QDMA_BAR_NUM; bar_idx++) {
        if (user_bar_id & (1 << bar_idx)) {
            *user_bar = bar_idx;
            bar_found = 1;
            break;
        }
    }
    if (bar_found == 0) {
        *user_bar = 0;
        qdma_log_error("%s: Bar not found, err:%d\n", __func__, -QDMA_ERR_HWACC_BAR_NOT_FOUND);
        return -QDMA_ERR_HWACC_BAR_NOT_FOUND;
    }

    return QDMA_SUCCESS;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_hw_ram_sbe_err_process() - Function to dump SBE error debug info
 *
 * This function dumps the Single Bit Error (SBE) debug information for the hardware RAM.
 *
 * @dev_hndl: Device handle
 *
 * Return: void
 *****************************************************************************/
static void qdma_cpm4_hw_ram_sbe_err_process(void *dev_hndl)
{
    // MD: Dump SBE status register information
    qdma_cpm4_dump_reg_info(dev_hndl, QDMA_CPM4_RAM_SBE_STS_A_ADDR, 1, NULL, 0);
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_hw_ram_dbe_err_process() - Function to dump DBE error debug info
 *
 * This function dumps the Double Bit Error (DBE) debug information for the hardware RAM.
 *
 * @dev_hndl: Device handle
 *
 * Return: void
 *****************************************************************************/
static void qdma_cpm4_hw_ram_dbe_err_process(void *dev_hndl)
{
    // MD: Dump DBE status register information
    qdma_cpm4_dump_reg_info(dev_hndl, QDMA_CPM4_RAM_DBE_STS_A_ADDR, 1, NULL, 0);
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_hw_desc_err_process() - Function to dump Descriptor Error info
 *
 * This function dumps the descriptor error debug information for the hardware.
 *
 * @dev_hndl: Device handle
 *
 * Return: void
 *****************************************************************************/
static void qdma_cpm4_hw_desc_err_process(void *dev_hndl)
{
    int i = 0;
    uint32_t desc_err_reg_list[] = {
        QDMA_CPM4_GLBL_DSC_ERR_STS_ADDR,
        QDMA_CPM4_GLBL_DSC_ERR_LOG0_ADDR,
        QDMA_CPM4_GLBL_DSC_ERR_LOG1_ADDR,
        QDMA_CPM4_GLBL_DSC_DBG_DAT0_ADDR,
        QDMA_CPM4_GLBL_DSC_DBG_DAT1_ADDR
    };
    int desc_err_num_regs = sizeof(desc_err_reg_list) / sizeof(uint32_t);

    // MD: Iterate over descriptor error registers and dump information
    for (i = 0; i < desc_err_num_regs; i++) {
        qdma_cpm4_dump_reg_info(dev_hndl, desc_err_reg_list[i], 1, NULL, 0);
    }
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_hw_trq_err_process() - Function to dump Target Access Error info
 *
 * This function dumps the target access error debug information for the hardware.
 *
 * @dev_hndl: Device handle
 *
 * Return: void
 *****************************************************************************/
static void qdma_cpm4_hw_trq_err_process(void *dev_hndl)
{
    int i = 0;
    uint32_t trq_err_reg_list[] = {
        QDMA_CPM4_GLBL_TRQ_ERR_STS_ADDR,
        QDMA_CPM4_GLBL_TRQ_ERR_LOG_ADDR
    };
    int trq_err_reg_num_regs = sizeof(trq_err_reg_list) / sizeof(uint32_t);

    // MD: Iterate over target access error registers and dump information
    for (i = 0; i < trq_err_reg_num_regs; i++) {
        qdma_cpm4_dump_reg_info(dev_hndl, trq_err_reg_list[i], 1, NULL, 0);
    }
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_hw_st_h2c_err_process() - Function to dump MM H2C Error info
 *
 * This function dumps the Memory-Mapped Host-to-Card (H2C) error debug information.
 *
 * @dev_hndl: Device handle
 *
 * Return: void
 *****************************************************************************/
static void qdma_cpm4_hw_st_h2c_err_process(void *dev_hndl)
{
    int i = 0;
    uint32_t st_h2c_err_reg_list[] = {
        QDMA_CPM4_H2C_ERR_STAT_ADDR,
        QDMA_CPM4_H2C_FIRST_ERR_QID_ADDR,
        QDMA_CPM4_H2C_DBG_REG0_ADDR,
        QDMA_CPM4_H2C_DBG_REG1_ADDR,
        QDMA_CPM4_H2C_DBG_REG2_ADDR,
        QDMA_CPM4_H2C_DBG_REG3_ADDR,
        QDMA_CPM4_H2C_DBG_REG4_ADDR
    };
    int st_h2c_err_num_regs = sizeof(st_h2c_err_reg_list) / sizeof(uint32_t);

    // MD: Iterate over H2C error registers and dump information
    for (i = 0; i < st_h2c_err_num_regs; i++) {
        qdma_cpm4_dump_reg_info(dev_hndl, st_h2c_err_reg_list[i], 1, NULL, 0);
    }
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_hw_st_c2h_err_process() - Function to dump MM C2H Error info
 *
 * This function iterates over a list of C2H error registers and dumps their
 * information using the qdma_cpm4_dump_reg_info function.
 *
 * @dev_hndl: Device handle
 *
 * Return: void
 *****************************************************************************/
static void qdma_cpm4_hw_st_c2h_err_process(void *dev_hndl)
{
    int i = 0;
    uint32_t st_c2h_err_reg_list[] = {
        QDMA_CPM4_C2H_ERR_STAT_ADDR,
        QDMA_CPM4_C2H_FATAL_ERR_STAT_ADDR,
        QDMA_CPM4_C2H_FIRST_ERR_QID_ADDR,
        QDMA_CPM4_C2H_STAT_S_AXIS_C2H_ACCEPTED_ADDR,
        QDMA_CPM4_C2H_STAT_S_AXIS_WRB_ACCEPTED_ADDR,
        QDMA_CPM4_C2H_STAT_DESC_RSP_PKT_ACCEPTED_ADDR,
        QDMA_CPM4_C2H_STAT_AXIS_PKG_CMP_ADDR,
        QDMA_CPM4_C2H_STAT_DBG_DMA_ENG_0_ADDR,
        QDMA_CPM4_C2H_STAT_DBG_DMA_ENG_1_ADDR,
        QDMA_CPM4_C2H_STAT_DBG_DMA_ENG_2_ADDR,
        QDMA_CPM4_C2H_STAT_DBG_DMA_ENG_3_ADDR,
        QDMA_CPM4_C2H_STAT_DESC_RSP_DROP_ACCEPTED_ADDR,
        QDMA_CPM4_C2H_STAT_DESC_RSP_ERR_ACCEPTED_ADDR
    };
    int st_c2h_err_num_regs = sizeof(st_c2h_err_reg_list) / sizeof(uint32_t);

    // MD: Iterate over each error register and dump its information
    for (i = 0; i < st_c2h_err_num_regs; i++) {
        qdma_cpm4_dump_reg_info(dev_hndl, st_c2h_err_reg_list[i], 1, NULL, 0);
    }
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_hw_get_error_name() - Function to get the error in string format
 *
 * This function returns the error name corresponding to the given error index.
 *
 * @err_idx: Error index
 *
 * Return: String representing the error name on success, NULL on failure
 *****************************************************************************/
const char *qdma_cpm4_hw_get_error_name(uint32_t err_idx)
{
    // MD: Validate error index
    if (err_idx >= QDMA_CPM4_ERRS_ALL) {
        qdma_log_error("%s: err_idx=%d is invalid, returning NULL\n",
                       __func__, (enum qdma_cpm4_error_idx)err_idx);
        return NULL;
    }

    // MD: Return the error name
    return qdma_cpm4_err_info[(enum qdma_cpm4_error_idx)err_idx].err_name;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_hw_error_process() - Function to find the error that got
 * triggered and call the handler qdma_hw_error_handler of that
 * particular error.
 *
 * This function reads the global error status register and processes each
 * detected error by calling the appropriate error handler.
 *
 * @dev_hndl: Device handle
 *
 * Return: 0 on success, negative error code on failure
 *****************************************************************************/
int qdma_cpm4_hw_error_process(void *dev_hndl)
{
    uint32_t glbl_err_stat = 0, err_stat = 0;
    uint32_t i = 0, j = 0;
    int32_t idx = 0;
    struct qdma_dev_attributes dev_cap;
    uint32_t hw_err_position[QDMA_CPM4_TOTAL_LEAF_ERROR_AGGREGATORS] = {
        QDMA_CPM4_DSC_ERR_POISON,
        QDMA_CPM4_TRQ_ERR_UNMAPPED,
        QDMA_CPM4_ST_C2H_ERR_MTY_MISMATCH,
        QDMA_CPM4_ST_FATAL_ERR_MTY_MISMATCH,
        QDMA_CPM4_ST_H2C_ERR_ZERO_LEN_DESC_ERR,
        QDMA_CPM4_SBE_ERR_MI_H2C0_DAT,
        QDMA_CPM4_DBE_ERR_MI_H2C0_DAT
    };

    // MD: Validate device handle
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n", __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Get device attributes
    qdma_cpm4_get_device_attributes(dev_hndl, &dev_cap);

    // MD: Read global error status register
    glbl_err_stat = qdma_reg_read(dev_hndl, QDMA_CPM4_GLBL_ERR_STAT_ADDR);
    if (!glbl_err_stat)
        return QDMA_HW_ERR_NOT_DETECTED;

    qdma_log_info("%s: Global Err Reg(0x%x) = 0x%x\n",
                  __func__, QDMA_CPM4_GLBL_ERR_STAT_ADDR, glbl_err_stat);

    // MD: Process each error aggregator
    for (i = 0; i < QDMA_CPM4_TOTAL_LEAF_ERROR_AGGREGATORS; i++) {
        j = hw_err_position[i];

        // MD: Skip certain errors if streaming is not enabled
        if ((!dev_cap.st_en) &&
            (j == QDMA_CPM4_ST_C2H_ERR_MTY_MISMATCH ||
             j == QDMA_CPM4_ST_FATAL_ERR_MTY_MISMATCH ||
             j == QDMA_CPM4_ST_H2C_ERR_ZERO_LEN_DESC_ERR))
            continue;

        // MD: Read error status register
        err_stat = qdma_reg_read(dev_hndl, qdma_cpm4_err_info[j].stat_reg_addr);
        if (err_stat) {
            qdma_log_info("addr = 0x%08x val = 0x%08x",
                          qdma_cpm4_err_info[j].stat_reg_addr, err_stat);

            // MD: Call the error processing function
            qdma_cpm4_err_info[j].qdma_cpm4_hw_err_process(dev_hndl);
            for (idx = j; idx < all_qdma_cpm4_hw_errs[i]; idx++) {
                // MD: Call the platform-specific handler
                if (err_stat & qdma_cpm4_err_info[idx].leaf_err_mask)
                    qdma_log_error("%s detected %s\n",
                                   __func__,
                                   qdma_cpm4_hw_get_error_name(idx));
            }
            // MD: Clear the error status register
            qdma_reg_write(dev_hndl, qdma_cpm4_err_info[j].stat_reg_addr, err_stat);
        }
    }

    // MD: Write 1 to the global status register to clear the bits
    qdma_reg_write(dev_hndl, QDMA_CPM4_GLBL_ERR_STAT_ADDR, glbl_err_stat);

    return QDMA_SUCCESS;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_hw_error_enable() - Enable all or a specific error
 *
 * This function enables all or a specific error based on the error index provided.
 * It writes to the appropriate registers to enable the error handling.
 *
 * @dev_hndl: Device handle
 * @err_idx: Error index
 *
 * Return: 0 on success, negative error code on failure
 *****************************************************************************/
int qdma_cpm4_hw_error_enable(void *dev_hndl, uint32_t err_idx)
{
    uint32_t idx = 0, i = 0;
    uint32_t reg_val = 0;
    struct qdma_dev_attributes dev_cap;

    // MD: Validate device handle
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n", __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Validate error index
    if (err_idx > QDMA_CPM4_ERRS_ALL) {
        qdma_log_error("%s: err_idx=%d is invalid, err:%d\n", __func__, (enum qdma_cpm4_error_idx)err_idx, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Get device attributes
    qdma_cpm4_get_device_attributes(dev_hndl, &dev_cap);

    if (err_idx == QDMA_CPM4_ERRS_ALL) {
        // MD: Enable all errors
        for (i = 0; i < QDMA_CPM4_TOTAL_LEAF_ERROR_AGGREGATORS; i++) {
            idx = all_qdma_cpm4_hw_errs[i];

            // MD: Skip streaming registers in MM only bitstreams
            if (!dev_cap.st_en) {
                if (idx == QDMA_CPM4_ST_C2H_ERR_ALL || idx == QDMA_CPM4_ST_FATAL_ERR_ALL || idx == QDMA_CPM4_ST_H2C_ERR_ALL)
                    continue;
            }

            // MD: Enable leaf error
            reg_val = qdma_cpm4_err_info[idx].leaf_err_mask;
            qdma_reg_write(dev_hndl, qdma_cpm4_err_info[idx].mask_reg_addr, reg_val);

            // MD: Enable global error
            reg_val = qdma_reg_read(dev_hndl, QDMA_CPM4_GLBL_ERR_MASK_ADDR);
            reg_val |= FIELD_SET(qdma_cpm4_err_info[idx].global_err_mask, 1);
            qdma_reg_write(dev_hndl, QDMA_CPM4_GLBL_ERR_MASK_ADDR, reg_val);
        }
    } else {
        // MD: Enable specific error
        if (!dev_cap.st_en) {
            if (err_idx >= QDMA_CPM4_ST_C2H_ERR_MTY_MISMATCH && err_idx <= QDMA_CPM4_ST_H2C_ERR_ALL)
                return QDMA_SUCCESS;
        }

        // MD: Enable leaf error
        reg_val = qdma_reg_read(dev_hndl, qdma_cpm4_err_info[err_idx].mask_reg_addr);
        reg_val |= FIELD_SET(qdma_cpm4_err_info[err_idx].leaf_err_mask, 1);
        qdma_reg_write(dev_hndl, qdma_cpm4_err_info[err_idx].mask_reg_addr, reg_val);

        // MD: Enable global error
        reg_val = qdma_reg_read(dev_hndl, QDMA_CPM4_GLBL_ERR_MASK_ADDR);
        reg_val |= FIELD_SET(qdma_cpm4_err_info[err_idx].global_err_mask, 1);
        qdma_reg_write(dev_hndl, QDMA_CPM4_GLBL_ERR_MASK_ADDR, reg_val);
    }

    return QDMA_SUCCESS;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_get_device_attributes() - Get QDMA device attributes
 *
 * This function retrieves the attributes of the QDMA device and populates the provided structure.
 *
 * @dev_hndl: Device handle
 * @dev_info: Pointer to hold the device info
 *
 * Return: 0 on success, negative error code on failure
 *****************************************************************************/
int qdma_cpm4_get_device_attributes(void *dev_hndl, struct qdma_dev_attributes *dev_info)
{
    uint8_t count = 0;
    uint32_t reg_val = 0;

    // MD: Validate device handle
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n", __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }
    // MD: Validate device info pointer
    if (!dev_info) {
        qdma_log_error("%s: dev_info is NULL, err:%d\n", __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Get number of PFs
    reg_val = qdma_reg_read(dev_hndl, QDMA_CPM4_GLBL2_PF_BARLITE_INT_ADDR);
    if (FIELD_GET(GLBL2_PF_BARLITE_INT_PF0_BAR_MAP_MASK, reg_val))
        count++;
    if (FIELD_GET(GLBL2_PF_BARLITE_INT_PF1_BAR_MAP_MASK, reg_val))
        count++;
    if (FIELD_GET(GLBL2_PF_BARLITE_INT_PF2_BAR_MAP_MASK, reg_val))
        count++;
    if (FIELD_GET(GLBL2_PF_BARLITE_INT_PF3_BAR_MAP_MASK, reg_val))
        count++;
    dev_info->num_pfs = count;

    // MD: Get number of Qs
    reg_val = qdma_reg_read(dev_hndl, QDMA_CPM4_GLBL2_CHANNEL_CAP_ADDR);
    dev_info->num_qs = FIELD_GET(GLBL2_CHANNEL_CAP_MULTIQ_MAX_MASK, reg_val);

    // MD: Check if FLR is present
    reg_val = qdma_reg_read(dev_hndl, QDMA_CPM4_GLBL2_MISC_CAP_ADDR);
    dev_info->mailbox_en = FIELD_GET(QDMA_GLBL2_MAILBOX_EN_MASK, reg_val);
    dev_info->flr_present = FIELD_GET(QDMA_GLBL2_FLR_PRESENT_MASK, reg_val);
    dev_info->mm_cmpt_en = 0;

    // MD: Check if ST/MM is enabled
    reg_val = qdma_reg_read(dev_hndl, QDMA_CPM4_GLBL2_CHANNEL_MDMA_ADDR);
    dev_info->mm_en = (FIELD_GET(GLBL2_CHANNEL_MDMA_C2H_ENG_MASK, reg_val) && FIELD_GET(GLBL2_CHANNEL_MDMA_H2C_ENG_MASK, reg_val)) ? 1 : 0;
    dev_info->st_en = (FIELD_GET(GLBL2_CHANNEL_MDMA_C2H_ST_MASK, reg_val) && FIELD_GET(GLBL2_CHANNEL_MDMA_H2C_ST_MASK, reg_val)) ? 1 : 0;

    // MD: Set maximum number of MM channels for Versal Hard
    dev_info->mm_channel_max = 2;

    // MD: Set other device attributes
    dev_info->debug_mode = 0;
    dev_info->desc_eng_mode = 0;
    dev_info->qid2vec_ctx = 1;
    dev_info->cmpt_ovf_chk_dis = 0;
    dev_info->mailbox_intr = 0;
    dev_info->sw_desc_64b = 0;
    dev_info->cmpt_desc_64b = 0;
    dev_info->dynamic_bar = 0;
    dev_info->legacy_intr = 0;
    dev_info->cmpt_trig_count_timer = 0;

    return QDMA_SUCCESS;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_credit_context_read() - Read credit context
 *
 * This function reads the credit context for a given queue and populates the output structure.
 *
 * @dev_hndl: Device handle
 * @c2h: Is C2H queue
 * @hw_qid: Hardware QID of the queue
 * @ctxt: Pointer to the context data
 *
 * Return: 0 on success, negative error code on failure
 *****************************************************************************/
static int qdma_cpm4_credit_context_read(void *dev_hndl, uint8_t c2h,
                                         uint16_t hw_qid,
                                         struct qdma_descq_credit_ctxt *ctxt)
{
    int rv = QDMA_SUCCESS;
    uint32_t cr_ctxt[QDMA_CPM4_CR_CONTEXT_NUM_WORDS] = {0};
    enum ind_ctxt_cmd_sel sel = c2h ? QDMA_CTXT_SEL_CR_C2H : QDMA_CTXT_SEL_CR_H2C;

    // MD: Validate input parameters
    if (!dev_hndl || !ctxt) {
        qdma_log_error("%s: dev_hndl=%p credit_ctxt=%p, err:%d\n",
                       __func__, dev_hndl, ctxt, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Read credit context from hardware
    rv = qdma_cpm4_indirect_reg_read(dev_hndl, sel, hw_qid,
                                     QDMA_CPM4_CR_CONTEXT_NUM_WORDS, cr_ctxt);
    if (rv < 0)
        return rv;

    // MD: Populate output structure with read values
    ctxt->credit = FIELD_GET(CRED_CTXT_DATA_W0_CREDT_MASK, cr_ctxt[0]);

    qdma_log_debug("%s: credit=%u\n", __func__, ctxt->credit);

    return QDMA_SUCCESS;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_credit_context_clear() - Clear credit context
 *
 * This function clears the credit context for a given queue.
 *
 * @dev_hndl: Device handle
 * @c2h: Is C2H queue
 * @hw_qid: Hardware QID of the queue
 *
 * Return: 0 on success, negative error code on failure
 *****************************************************************************/
static int qdma_cpm4_credit_context_clear(void *dev_hndl, uint8_t c2h,
                                          uint16_t hw_qid)
{
    enum ind_ctxt_cmd_sel sel = c2h ? QDMA_CTXT_SEL_CR_C2H : QDMA_CTXT_SEL_CR_H2C;

    // MD: Validate input parameter
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n", __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Clear credit context in hardware
    return qdma_cpm4_indirect_reg_clear(dev_hndl, sel, hw_qid);
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_credit_context_invalidate() - Invalidate credit context
 *
 * This function invalidates the credit context for a given queue.
 *
 * @dev_hndl: Device handle
 * @c2h: Is C2H queue
 * @hw_qid: Hardware QID of the queue
 *
 * Return: 0 on success, negative error code on failure
 *****************************************************************************/
static int qdma_cpm4_credit_context_invalidate(void *dev_hndl, uint8_t c2h,
                                               uint16_t hw_qid)
{
    enum ind_ctxt_cmd_sel sel = c2h ? QDMA_CTXT_SEL_CR_C2H : QDMA_CTXT_SEL_CR_H2C;

    // MD: Validate input parameter
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n", __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Invalidate credit context in hardware
    return qdma_cpm4_indirect_reg_invalidate(dev_hndl, sel, hw_qid);
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_credit_ctx_conf() - Configure credit context
 *
 * This function configures the credit context based on the specified access type.
 * Note: QDMA_HW_ACCESS_WRITE is not supported.
 *
 * @dev_hndl: Device handle
 * @c2h: Is C2H queue
 * @hw_qid: Hardware QID of the queue
 * @ctxt: Pointer to the context data
 * @access_type: HW access type (qdma_hw_access_type enum) value
 *
 * Return: 0 on success, negative error code on failure
 *****************************************************************************/
int qdma_cpm4_credit_ctx_conf(void *dev_hndl, uint8_t c2h, uint16_t hw_qid,
                              struct qdma_descq_credit_ctxt *ctxt,
                              enum qdma_hw_access_type access_type)
{
    int rv = QDMA_SUCCESS;

    // MD: Configure credit context based on access type
    switch (access_type) {
    case QDMA_HW_ACCESS_READ:
        rv = qdma_cpm4_credit_context_read(dev_hndl, c2h, hw_qid, ctxt);
        break;
    case QDMA_HW_ACCESS_CLEAR:
        rv = qdma_cpm4_credit_context_clear(dev_hndl, c2h, hw_qid);
        break;
    case QDMA_HW_ACCESS_INVALIDATE:
        rv = qdma_cpm4_credit_context_invalidate(dev_hndl, c2h, hw_qid);
        break;
    case QDMA_HW_ACCESS_WRITE:
    default:
        qdma_log_error("%s: Invalid access type=%d, err:%d\n", __func__, access_type, -QDMA_ERR_INV_PARAM);
        rv = -QDMA_ERR_INV_PARAM;
        break;
    }

    return rv;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_dump_config_regs() - Function to get QDMA config register dump in a buffer
 *
 * This function retrieves the configuration registers of the QDMA device and fills them into a buffer.
 * It checks for valid device handle, buffer size, and device capabilities before proceeding.
 *
 * @dev_hndl: Device handle
 * @is_vf: Whether PF or VF
 * @buf: Pointer to buffer to be filled
 * @buflen: Length of the buffer
 *
 * Return: Length up-till the buffer is filled on success, negative error code on failure
 *****************************************************************************/
int qdma_cpm4_dump_config_regs(void *dev_hndl, uint8_t is_vf, char *buf, uint32_t buflen)
{
    uint32_t i = 0, j = 0;
    struct xreg_info *reg_info;
    uint32_t num_regs = qdma_cpm4_config_num_regs_get();
    uint32_t len = 0, val = 0;
    int rv = QDMA_SUCCESS;
    char name[DEBGFS_GEN_NAME_SZ] = "";
    struct qdma_dev_attributes dev_cap;

    // MD: Validate device handle
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n", __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Check if buffer size is sufficient
    if (buflen < qdma_cpm4_reg_dump_buf_len()) {
        qdma_log_error("%s: Buffer too small, err:%d\n", __func__, -QDMA_ERR_NO_MEM);
        return -QDMA_ERR_NO_MEM;
    }

    // MD: Check if the function is being used for VF
    if (is_vf) {
        qdma_log_error("%s: Wrong API used for VF, err:%d\n", __func__, -QDMA_ERR_HWACC_FEATURE_NOT_SUPPORTED);
        return -QDMA_ERR_HWACC_FEATURE_NOT_SUPPORTED;
    }

    // MD: Get device attributes
    qdma_cpm4_get_device_attributes(dev_hndl, &dev_cap);

    // MD: Get configuration registers
    reg_info = qdma_cpm4_config_regs_get();

    // MD: Iterate over each register and fill the buffer
    for (i = 0; i < num_regs; i++) {
        if ((GET_CAPABILITY_MASK(dev_cap.mm_en, dev_cap.st_en, dev_cap.mm_cmpt_en, dev_cap.mailbox_en) & reg_info[i].mode) == 0)
            continue;

        for (j = 0; j < reg_info[i].repeat; j++) {
            rv = QDMA_SNPRINTF_S(name, DEBGFS_GEN_NAME_SZ, DEBGFS_GEN_NAME_SZ, "%s_%d", reg_info[i].name, j);
            if ((rv < 0) || (rv > DEBGFS_GEN_NAME_SZ)) {
                qdma_log_error("%d:%s QDMA_SNPRINTF_S() failed, err:%d\n", __LINE__, __func__, rv);
                return -QDMA_ERR_NO_MEM;
            }

            val = qdma_reg_read(dev_hndl, (reg_info[i].addr + (j * 4)));
            rv = dump_reg(buf + len, buflen - len, (reg_info[i].addr + (j * 4)), name, val);
            if (rv < 0) {
                qdma_log_error("%s Buff too small, err:%d\n", __func__, -QDMA_ERR_NO_MEM);
                return -QDMA_ERR_NO_MEM;
            }
            len += rv;
        }
    }

    return len;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_dump_cpm4_queue_context() - Function to get QDMA queue context dump in a buffer
 *
 * This function retrieves the queue context of the QDMA device and fills it into a buffer.
 * It checks for valid device handle, context data, buffer, and queue type before proceeding.
 *
 * @dev_hndl: Device handle
 * @st: Stream type
 * @q_type: Queue type
 * @ctxt_data: Pointer to the context data
 * @buf: Pointer to buffer to be filled
 * @buflen: Length of the buffer
 *
 * Return: Length up-till the buffer is filled on success, negative error code on failure
 *****************************************************************************/
int qdma_cpm4_dump_queue_context(void *dev_hndl, uint8_t st, enum qdma_dev_q_type q_type, struct qdma_descq_context *ctxt_data, char *buf, uint32_t buflen)
{
    int rv = 0;
    uint32_t req_buflen = 0;

    // MD: Validate device handle
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n", __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Validate context data
    if (!ctxt_data) {
        qdma_log_error("%s: ctxt_data is NULL, err:%d\n", __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Validate buffer
    if (!buf) {
        qdma_log_error("%s: buf is NULL, err:%d\n", __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Validate queue type
    if (q_type >= QDMA_DEV_Q_TYPE_MAX) {
        qdma_log_error("%s: invalid q_type, err:%d\n", __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Get required buffer length
    rv = qdma_cpm4_context_buf_len(st, q_type, &req_buflen);
    if (rv != QDMA_SUCCESS)
        return rv;

    // MD: Check if buffer size is sufficient
    if (buflen < req_buflen) {
        qdma_log_error("%s: Too small buffer(%d), reqd(%d), err:%d\n", __func__, buflen, req_buflen, -QDMA_ERR_NO_MEM);
        return -QDMA_ERR_NO_MEM;
    }

    // MD: Dump context into buffer
    rv = dump_cpm4_context(ctxt_data, st, q_type, buf, buflen);

    return rv;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_dump_intr_context() - Function to get QDMA interrupt context dump in a buffer
 *
 * This function retrieves the interrupt context for a specified ring index and dumps it into a buffer.
 * It checks for valid input parameters, calculates the required buffer length, and performs the context dump.
 *
 * @dev_hndl: Device handle
 * @intr_ctx: Pointer to the interrupt context structure
 * @ring_index: Ring index for which the context is to be dumped
 * @buf: Pointer to the buffer to be filled
 * @buflen: Length of the buffer
 *
 * Return: Length up to which the buffer is filled on success, negative error code on failure
 *****************************************************************************/
int qdma_cpm4_dump_intr_context(void *dev_hndl,
                                struct qdma_indirect_intr_ctxt *intr_ctx,
                                int ring_index,
                                char *buf, uint32_t buflen)
{
    int rv = 0;
    uint32_t req_buflen = 0;

    // MD: Validate device handle
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n", __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Validate interrupt context pointer
    if (!intr_ctx) {
        qdma_log_error("%s: intr_ctx is NULL, err:%d\n", __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Validate buffer pointer
    if (!buf) {
        qdma_log_error("%s: buf is NULL, err:%d\n", __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Calculate required buffer length for interrupt context
    req_buflen = qdma_cpm4_intr_context_buf_len();
    if (buflen < req_buflen) {
        qdma_log_error("%s: Too small buffer(%d), reqd(%d), err:%d\n", __func__, buflen, req_buflen, -QDMA_ERR_NO_MEM);
        return -QDMA_ERR_NO_MEM;
    }

    // MD: Dump interrupt context into buffer
    rv = dump_cpm4_intr_context(intr_ctx, ring_index, buf, buflen);

    return rv;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_init_ctxt_memory() - Initialize the context for all queues
 *
 * This function initializes the context memory for all queues by clearing
 * the indirect context registers. It handles both ST and MM modes based on
 * the device attributes.
 *
 * @dev_hndl: Device handle
 *
 * Return: 0 on success, negative error code on failure
 *****************************************************************************/
int qdma_cpm4_init_ctxt_memory(void *dev_hndl)
{
#ifdef ENABLE_INIT_CTXT_MEMORY
    uint32_t data[QDMA_REG_IND_CTXT_REG_COUNT];
    uint16_t i = 0;
    struct qdma_dev_attributes dev_info;

    // MD: Validate input parameter
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n", __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Initialize context memory
    qdma_memset(data, 0, sizeof(uint32_t) * QDMA_REG_IND_CTXT_REG_COUNT);
    qdma_cpm4_get_device_attributes(dev_hndl, &dev_info);
    qdma_log_info("%s: clearing the context for all qs", __func__);

    // MD: Clear context for each queue
    for (; i < dev_info.num_qs; i++) {
        int sel = QDMA_CTXT_SEL_SW_C2H;
        int rv;

#ifdef TANDEM_BOOT_SUPPORTED
        for (; sel <= QDMA_CTXT_SEL_CR_H2C; sel++) {
            rv = qdma_cpm4_indirect_reg_clear(dev_hndl, (enum ind_ctxt_cmd_sel)sel, i);
            if (rv < 0)
                return rv;
        }
#else
        for (; sel <= QDMA_CTXT_SEL_PFTCH; sel++) {
            // MD: Skip PFTCH and CMPT context setup if ST mode is not enabled
            if ((dev_info.st_en == 0) && (sel == QDMA_CTXT_SEL_PFTCH || sel == QDMA_CTXT_SEL_CMPT)) {
                qdma_log_debug("%s: ST context is skipped:", __func__);
                qdma_log_debug(" sel = %d", sel);
                continue;
            }

            rv = qdma_cpm4_indirect_reg_clear(dev_hndl, (enum ind_ctxt_cmd_sel)sel, i);
            if (rv < 0)
                return rv;
        }
#endif
    }

    // MD: Clear fmap for each PF
    for (i = 0; i < dev_info.num_pfs; i++)
        qdma_cpm4_fmap_clear(dev_hndl, i);
#else
    // MD: Validate input parameter
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n", __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }
#endif
    return 0;
}

#ifdef TANDEM_BOOT_SUPPORTED
/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_init_st_ctxt() - Initialize the ST context
 *
 * This function initializes the ST context by clearing the indirect context
 * registers for ST mode queues.
 *
 * @dev_hndl: Device handle
 *
 * Return: Returns the platform-specific error code
 *****************************************************************************/
int qdma_cpm4_init_st_ctxt(void *dev_hndl)
{
    uint32_t data[QDMA_REG_IND_CTXT_REG_COUNT];
    uint16_t i = 0;
    struct qdma_dev_attributes dev_info;

    // MD: Validate input parameter
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n", __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Initialize context memory
    qdma_memset(data, 0, sizeof(uint32_t) * QDMA_REG_IND_CTXT_REG_COUNT);
    qdma_cpm4_get_device_attributes(dev_hndl, &dev_info);

    // MD: Clear context for each ST mode queue
    for (; i < dev_info.num_qs; i++) {
        int sel = QDMA_CTXT_SEL_CMPT;
        int rv;

        for (; sel <= QDMA_CTXT_SEL_PFTCH; sel++) {
            // MD: Skip PFTCH and CMPT context setup if ST mode is not enabled
            if ((dev_info.st_en == 0) && ((sel == QDMA_CTXT_SEL_PFTCH) || (sel == QDMA_CTXT_SEL_CMPT))) {
                qdma_log_debug("%s: ST context is skipped:", __func__);
                qdma_log_debug("sel = %d\n", sel);
                continue;
            }

            rv = qdma_cpm4_indirect_reg_clear(dev_hndl, (enum ind_ctxt_cmd_sel)sel, i);
            if (rv < 0)
                return rv;
        }
    }

    return QDMA_SUCCESS;
}
#endif

/* MD:****************************************************************************/
/* MD:*
 * get_reg_entry() - Get register entry
 *
 * This function retrieves the register entry index for a given register address.
 *
 * @reg_addr: Register address
 * @reg_entry: Pointer to store the register entry index
 *
 * Return: 0 on success, negative error code on failure
 *****************************************************************************/
static int get_reg_entry(uint32_t reg_addr, int *reg_entry)
{
    uint32_t i = 0;
    struct xreg_info *reg_info;
    uint32_t num_regs = qdma_cpm4_config_num_regs_get();

    reg_info = qdma_cpm4_config_regs_get();

    // MD: Search for the register entry
    for (i = 0; (i < num_regs - 1); i++) {
        if (reg_info[i].addr == reg_addr) {
            *reg_entry = i;
            break;
        }
    }

    // MD: Check if register entry was found
    if (i >= num_regs - 1) {
        qdma_log_error("%s: 0x%08x is missing register list, err:%d\n", __func__, reg_addr, -QDMA_ERR_INV_PARAM);
        *reg_entry = -1;
        return -QDMA_ERR_INV_PARAM;
    }

    return 0;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_dump_config_reg_list() - Dump the registers
 *
 * This function dumps the configuration registers into a provided buffer.
 * It iterates over the list of registers, retrieves their values, and formats them into the buffer.
 *
 * @dev_hndl: Device handle
 * @total_regs: Max registers to read
 * @reg_list: Array of reg addr and reg values
 * @buf: Pointer to buffer to be filled
 * @buflen: Length of the buffer
 *
 * Return: Returns the length of data written to the buffer on success, negative error code on failure
 *****************************************************************************/
int qdma_cpm4_dump_config_reg_list(void *dev_hndl, uint32_t total_regs,
                                   struct qdma_reg_data *reg_list, char *buf, uint32_t buflen)
{
    uint32_t j = 0, len = 0;
    uint32_t reg_count = 0;
    int reg_data_entry;
    int rv = 0;
    char name[DEBGFS_GEN_NAME_SZ] = "";
    struct xreg_info *reg_info = qdma_cpm4_config_regs_get();

    // MD: Validate input parameters
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n", __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    if (!buf) {
        qdma_log_error("%s: buf is NULL, err:%d\n", __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Iterate over the list of registers
    for (reg_count = 0; (reg_count < total_regs);) {

        // MD: Get the register entry for the current register address
        rv = get_reg_entry(reg_list[reg_count].reg_addr, &reg_data_entry);
        if (rv < 0) {
            qdma_log_error("%s: register missing in list, err:%d\n", __func__, -QDMA_ERR_INV_PARAM);
            return rv;
        }

        // MD: Iterate over repeated registers
        for (j = 0; j < reg_info[reg_data_entry].repeat; j++) {
            // MD: Format the register name with index
            rv = QDMA_SNPRINTF_S(name, DEBGFS_GEN_NAME_SZ, DEBGFS_GEN_NAME_SZ, "%s_%d", reg_info[reg_data_entry].name, j);
            if ((rv < 0) || (rv > DEBGFS_GEN_NAME_SZ)) {
                qdma_log_error("%d:%s snprintf failed, err:%d\n", __LINE__, __func__, rv);
                return -QDMA_ERR_NO_MEM;
            }

            // MD: Dump the register value into the buffer
            rv = dump_reg(buf + len, buflen - len, (reg_info[reg_data_entry].addr + (j * 4)), name, reg_list[reg_count + j].reg_val);
            if (rv < 0) {
                qdma_log_error("%s Buff too small, err:%d\n", __func__, -QDMA_ERR_NO_MEM);
                return -QDMA_ERR_NO_MEM;
            }
            len += rv;
        }
        reg_count += j;
    }

    return len; // MD: Return the length of data written to the buffer
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_read_reg_list() - Read the register values
 *
 * This function reads the register values for a given device handle and register slot.
 * It validates the input parameters, determines the starting address based on the slot,
 * and reads the register values into the provided list.
 *
 * @dev_hndl: Device handle
 * @is_vf: Whether PF or VF
 * @reg_rd_slot: Register read slot
 * @total_regs: Max registers to read
 * @reg_list: Array of reg addr and reg values
 *
 * Return: Returns the platform-specific error code
 *****************************************************************************/
int qdma_cpm4_read_reg_list(void *dev_hndl, uint8_t is_vf,
                            uint16_t reg_rd_slot,
                            uint16_t *total_regs,
                            struct qdma_reg_data *reg_list)
{
    uint16_t reg_count = 0, i = 0, j = 0;
    uint32_t num_regs = qdma_cpm4_config_num_regs_get();
    struct xreg_info *reg_info = qdma_cpm4_config_regs_get();
    struct qdma_dev_attributes dev_cap;
    uint32_t reg_start_addr = 0;
    int reg_index = 0;
    int rv = 0;

    // MD: Check if the operation is supported for VF
    if (!is_vf) {
        qdma_log_error("%s: not supported for PF, err:%d\n", __func__, -QDMA_ERR_HWACC_FEATURE_NOT_SUPPORTED);
        return -QDMA_ERR_HWACC_FEATURE_NOT_SUPPORTED;
    }

    // MD: Validate input parameters
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n", __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    if (!reg_list) {
        qdma_log_error("%s: reg_list is NULL, err:%d\n", __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Get device attributes
    qdma_cpm4_get_device_attributes(dev_hndl, &dev_cap);

    // MD: Determine the starting address based on the register read slot
    switch (reg_rd_slot) {
    case QDMA_REG_READ_GROUP_1:
        reg_start_addr = QDMA_CPM4_REG_GROUP_1_START_ADDR;
        break;
    case QDMA_REG_READ_GROUP_2:
        reg_start_addr = QDMA_CPM4_REG_GROUP_2_START_ADDR;
        break;
    case QDMA_REG_READ_GROUP_3:
        reg_start_addr = QDMA_CPM4_REG_GROUP_3_START_ADDR;
        break;
    case QDMA_REG_READ_GROUP_4:
        reg_start_addr = QDMA_CPM4_REG_GROUP_4_START_ADDR;
        break;
    default:
        qdma_log_error("%s: Invalid slot received\n", __func__);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Get the register entry index
    rv = get_reg_entry(reg_start_addr, &reg_index);
    if (rv < 0) {
        qdma_log_error("%s: register missing in list, err:%d\n", __func__, -QDMA_ERR_INV_PARAM);
        return rv;
    }

    // MD: Read the register values into the list
    for (i = 0, reg_count = 0; ((i < num_regs - 1 - reg_index) && (reg_count < QDMA_MAX_REGISTER_DUMP)); i++) {
        if (((GET_CAPABILITY_MASK(dev_cap.mm_en, dev_cap.st_en, dev_cap.mm_cmpt_en, dev_cap.mailbox_en) & reg_info[i].mode) == 0) ||
            (reg_info[i].read_type == QDMA_REG_READ_PF_ONLY))
            continue;

        for (j = 0; j < reg_info[i].repeat && (reg_count < QDMA_MAX_REGISTER_DUMP); j++) {
            reg_list[reg_count].reg_addr = (reg_info[i].addr + (j * 4));
            reg_list[reg_count].reg_val = qdma_reg_read(dev_hndl, reg_list[reg_count].reg_addr);
            reg_count++;
        }
    }

    *total_regs = reg_count;
    return rv;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_write_global_ring_sizes() - Set the global ring size array
 *
 * This function writes the global ring size array for a given device handle.
 * It validates the input parameters and writes the values to the hardware.
 *
 * @dev_hndl: Device handle
 * @index: Index from where the values need to be written
 * @count: Number of entries to be written
 * @glbl_rng_sz: Pointer to the array having the values to write
 *
 * (index + count) shall not be more than 16
 *
 * Return: 0 on success, negative error code on failure
 *****************************************************************************/
static int qdma_cpm4_write_global_ring_sizes(void *dev_hndl, uint8_t index,
                                             uint8_t count, const uint32_t *glbl_rng_sz)
{
    // MD: Validate input parameters
    if (!dev_hndl || !glbl_rng_sz || !count) {
        qdma_log_error("%s: dev_hndl=%p glbl_rng_sz=%p, err:%d\n", __func__, dev_hndl, glbl_rng_sz, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    if ((index + count) > QDMA_NUM_RING_SIZES) {
        qdma_log_error("%s: index=%u count=%u > %d, err:%d\n", __func__, index, count, QDMA_NUM_RING_SIZES, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Write global ring size values to hardware
    qdma_write_csr_values(dev_hndl, QDMA_CPM4_GLBL_RNG_SZ_1_ADDR, index, count, glbl_rng_sz);

    return QDMA_SUCCESS;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_read_global_ring_sizes() - Function to get the global ring size array
 *
 * This function reads the global ring size values starting from a specified index.
 * It validates the input parameters and ensures the index and count do not exceed the limit.
 *
 * @dev_hndl: Device handle
 * @index: Index from where the values need to be read
 * @count: Number of entries to be read
 * @glbl_rng_sz: Pointer to array to hold the values read
 *
 * (index + count) shall not be more than 16
 *
 * Return: 0 on success, negative error code on failure
 *****************************************************************************/
static int qdma_cpm4_read_global_ring_sizes(void *dev_hndl, uint8_t index,
                                            uint8_t count, uint32_t *glbl_rng_sz)
{
    // MD: Validate input parameters
    if (!dev_hndl || !glbl_rng_sz || !count) {
        qdma_log_error("%s: dev_hndl=%p glbl_rng_sz=%p, err:%d\n",
                       __func__, dev_hndl, glbl_rng_sz, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Check if index and count exceed the limit
    if ((index + count) > QDMA_NUM_RING_SIZES) {
        qdma_log_error("%s: index=%u count=%u > %d, err:%d\n",
                       __func__, index, count, QDMA_NUM_C2H_BUFFER_SIZES, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Read CSR values for global ring sizes
    qdma_read_csr_values(dev_hndl, QDMA_CPM4_GLBL_RNG_SZ_1_ADDR, index, count, glbl_rng_sz);

    return QDMA_SUCCESS;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_write_global_timer_count() - Function to set the timer values
 *
 * This function writes the global timer count values starting from a specified index.
 * It validates the input parameters and ensures the index and count do not exceed the limit.
 * It also checks if the ST or MM completion is supported before writing.
 *
 * @dev_hndl: Device handle
 * @glbl_tmr_cnt: Pointer to the array having the values to write
 * @index: Index from where the values need to be written
 * @count: Number of entries to be written
 *
 * (index + count) shall not be more than 16
 *
 * Return: 0 on success, negative error code on failure
 *****************************************************************************/
static int qdma_cpm4_write_global_timer_count(void *dev_hndl, uint8_t index,
                                              uint8_t count, const uint32_t *glbl_tmr_cnt)
{
    struct qdma_dev_attributes dev_cap;

    // MD: Validate input parameters
    if (!dev_hndl || !glbl_tmr_cnt || !count) {
        qdma_log_error("%s: dev_hndl=%p glbl_tmr_cnt=%p, err:%d\n",
                       __func__, dev_hndl, glbl_tmr_cnt, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Check if index and count exceed the limit
    if ((index + count) > QDMA_NUM_C2H_TIMERS) {
        qdma_log_error("%s: index=%u count=%u > %d, err:%d\n",
                       __func__, index, count, QDMA_NUM_C2H_TIMERS, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Get device attributes
    qdma_cpm4_get_device_attributes(dev_hndl, &dev_cap);

    // MD: Check if ST or MM completion is supported
    if (dev_cap.st_en || dev_cap.mm_cmpt_en) {
        // MD: Write CSR values for global timer count
        qdma_write_csr_values(dev_hndl, QDMA_CPM4_C2H_TIMER_CNT_1_ADDR, index, count, glbl_tmr_cnt);
    } else {
        qdma_log_error("%s: ST or MM cmpt not supported, err:%d\n",
                       __func__, -QDMA_ERR_HWACC_FEATURE_NOT_SUPPORTED);
        return -QDMA_ERR_HWACC_FEATURE_NOT_SUPPORTED;
    }

    return QDMA_SUCCESS;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_read_global_timer_count() - Function to get the timer values
 *
 * This function reads the global timer count values starting from a specified index.
 * It validates the input parameters and ensures the index and count do not exceed the limit.
 * It also checks if the ST or MM completion is supported before reading.
 *
 * @dev_hndl: Device handle
 * @index: Index from where the values need to be read
 * @count: Number of entries to be read
 * @glbl_tmr_cnt: Pointer to array to hold the values read
 *
 * (index + count) shall not be more than 16
 *
 * Return: 0 on success, negative error code on failure
 *****************************************************************************/
static int qdma_cpm4_read_global_timer_count(void *dev_hndl, uint8_t index,
                                             uint8_t count, uint32_t *glbl_tmr_cnt)
{
    struct qdma_dev_attributes dev_cap;

    // MD: Validate input parameters
    if (!dev_hndl || !glbl_tmr_cnt || !count) {
        qdma_log_error("%s: dev_hndl=%p glbl_tmr_cnt=%p, err:%d\n",
                       __func__, dev_hndl, glbl_tmr_cnt, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Check if index and count exceed the limit
    if ((index + count) > QDMA_NUM_C2H_TIMERS) {
        qdma_log_error("%s: index=%u count=%u > %d, err:%d\n",
                       __func__, index, count, QDMA_NUM_C2H_TIMERS, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Get device attributes
    qdma_cpm4_get_device_attributes(dev_hndl, &dev_cap);

    // MD: Check if ST or MM completion is supported
    if (dev_cap.st_en || dev_cap.mm_cmpt_en) {
        // MD: Read CSR values for global timer count
        qdma_read_csr_values(dev_hndl, QDMA_CPM4_C2H_TIMER_CNT_1_ADDR, index, count, glbl_tmr_cnt);
    } else {
        qdma_log_error("%s: ST or MM cmpt not supported, err:%d\n",
                       __func__, -QDMA_ERR_HWACC_FEATURE_NOT_SUPPORTED);
        return -QDMA_ERR_HWACC_FEATURE_NOT_SUPPORTED;
    }

    return QDMA_SUCCESS;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_write_global_counter_threshold() - Set global counter threshold values
 *
 * This function writes the counter threshold values starting from a specified index.
 * It validates the input parameters and writes the values to the hardware if supported.
 *
 * @dev_hndl: Device handle
 * @index: Index from where the values need to be written
 * @count: Number of entries to be written
 * @glbl_cnt_th: Pointer to the array having the values to write
 *
 * (index + count) shall not be more than 16
 *
 * Return: 0 on success, negative error code on failure
 *****************************************************************************/
static int qdma_cpm4_write_global_counter_threshold(void *dev_hndl,
                                                    uint8_t index,
                                                    uint8_t count, const uint32_t *glbl_cnt_th)
{
    struct qdma_dev_attributes dev_cap;

    // MD: Validate input parameters
    if (!dev_hndl || !glbl_cnt_th || !count) {
        qdma_log_error("%s: dev_hndl=%p glbl_cnt_th=%p, err:%d\n",
                       __func__, dev_hndl, glbl_cnt_th, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Check if index and count exceed the maximum allowed
    if ((index + count) > QDMA_NUM_C2H_COUNTERS) {
        qdma_log_error("%s: index=%u count=%u > %d, err:%d\n",
                       __func__, index, count, QDMA_NUM_C2H_BUFFER_SIZES, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Get device attributes
    qdma_cpm4_get_device_attributes(dev_hndl, &dev_cap);

    // MD: Check if ST or MM completion is enabled
    if (dev_cap.st_en || dev_cap.mm_cmpt_en)
        qdma_write_csr_values(dev_hndl, QDMA_CPM4_C2H_CNT_TH_1_ADDR, index, count, glbl_cnt_th);
    else {
        qdma_log_error("%s: ST or MM cmpt not supported, err:%d\n",
                       __func__, -QDMA_ERR_HWACC_FEATURE_NOT_SUPPORTED);
        return -QDMA_ERR_HWACC_FEATURE_NOT_SUPPORTED;
    }

    return QDMA_SUCCESS;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_read_global_counter_threshold() - Get global counter threshold values
 *
 * This function reads the counter threshold values starting from a specified index.
 * It validates the input parameters and reads the values from the hardware if supported.
 *
 * @dev_hndl: Device handle
 * @index: Index from where the values need to be read
 * @count: Number of entries to be read
 * @glbl_cnt_th: Pointer to array to hold the values read
 *
 * (index + count) shall not be more than 16
 *
 * Return: 0 on success, negative error code on failure
 *****************************************************************************/
static int qdma_cpm4_read_global_counter_threshold(void *dev_hndl,
                                                   uint8_t index,
                                                   uint8_t count, uint32_t *glbl_cnt_th)
{
    struct qdma_dev_attributes dev_cap;

    // MD: Validate input parameters
    if (!dev_hndl || !glbl_cnt_th || !count) {
        qdma_log_error("%s: dev_hndl=%p glbl_cnt_th=%p, err:%d\n",
                       __func__, dev_hndl, glbl_cnt_th, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Check if index and count exceed the maximum allowed
    if ((index + count) > QDMA_NUM_C2H_COUNTERS) {
        qdma_log_error("%s: index=%u count=%u > %d, err:%d\n",
                       __func__, index, count, QDMA_NUM_C2H_COUNTERS, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Get device attributes
    qdma_cpm4_get_device_attributes(dev_hndl, &dev_cap);

    // MD: Check if ST or MM completion is enabled
    if (dev_cap.st_en || dev_cap.mm_cmpt_en)
        qdma_read_csr_values(dev_hndl, QDMA_CPM4_C2H_CNT_TH_1_ADDR, index, count, glbl_cnt_th);
    else {
        qdma_log_error("%s: ST or MM cmpt not supported, err:%d\n",
                       __func__, -QDMA_ERR_HWACC_FEATURE_NOT_SUPPORTED);
        return -QDMA_ERR_HWACC_FEATURE_NOT_SUPPORTED;
    }

    return QDMA_SUCCESS;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_write_global_buffer_sizes() - Set global buffer sizes
 *
 * This function writes the buffer sizes starting from a specified index.
 * It validates the input parameters and writes the values to the hardware if supported.
 *
 * @dev_hndl: Device handle
 * @index: Index from where the values need to be written
 * @count: Number of entries to be written
 * @glbl_buf_sz: Pointer to the array having the values to write
 *
 * (index + count) shall not be more than 16
 *
 * Return: 0 on success, negative error code on failure
 *****************************************************************************/
static int qdma_cpm4_write_global_buffer_sizes(void *dev_hndl,
                                               uint8_t index,
                                               uint8_t count, const uint32_t *glbl_buf_sz)
{
    struct qdma_dev_attributes dev_cap;

    // MD: Validate input parameters
    if (!dev_hndl || !glbl_buf_sz || !count) {
        qdma_log_error("%s: dev_hndl=%p glbl_buf_sz=%p, err:%d\n",
                       __func__, dev_hndl, glbl_buf_sz, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Check if index and count exceed the maximum allowed
    if ((index + count) > QDMA_NUM_C2H_BUFFER_SIZES) {
        qdma_log_error("%s: index=%u count=%u > %d, err:%d\n",
                       __func__, index, count, QDMA_NUM_C2H_BUFFER_SIZES, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Get device attributes
    qdma_cpm4_get_device_attributes(dev_hndl, &dev_cap);

    // MD: Check if ST is enabled
    if (dev_cap.st_en)
        qdma_write_csr_values(dev_hndl, QDMA_CPM4_C2H_BUF_SZ_0_ADDR, index, count, glbl_buf_sz);
    else {
        qdma_log_error("%s: ST not supported, err:%d\n",
                       __func__, -QDMA_ERR_HWACC_FEATURE_NOT_SUPPORTED);
        return -QDMA_ERR_HWACC_FEATURE_NOT_SUPPORTED;
    }

    return QDMA_SUCCESS;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_read_global_buffer_sizes() - Function to get the buffer sizes
 *
 * This function reads the global buffer sizes starting from a specified index.
 * It validates the input parameters and reads the values into the provided array.
 *
 * @dev_hndl: Device handle
 * @index: Index from where the values need to be read
 * @count: Number of entries to be read
 * @glbl_buf_sz: Pointer to array to hold the values read
 *
 * (index + count) shall not be more than 16
 *
 * Return: 0 on success, negative error code on failure
 *****************************************************************************/
static int qdma_cpm4_read_global_buffer_sizes(void *dev_hndl, uint8_t index,
                                              uint8_t count, uint32_t *glbl_buf_sz)
{
    struct qdma_dev_attributes dev_cap;

    // MD: Validate input parameters
    if (!dev_hndl || !glbl_buf_sz || !count) {
        qdma_log_error("%s: dev_hndl=%p glbl_buf_sz=%p, err:%d\n",
                       __func__, dev_hndl, glbl_buf_sz, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Check if the requested range is within bounds
    if ((index + count) > QDMA_NUM_C2H_BUFFER_SIZES) {
        qdma_log_error("%s: index=%u count=%u > %d, err:%d\n",
                       __func__, index, count, QDMA_NUM_C2H_BUFFER_SIZES, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Get device attributes
    qdma_cpm4_get_device_attributes(dev_hndl, &dev_cap);

    // MD: Read buffer sizes if supported
    if (dev_cap.st_en) {
        qdma_read_csr_values(dev_hndl, QDMA_CPM4_C2H_BUF_SZ_0_ADDR, index, count, glbl_buf_sz);
    } else {
        qdma_log_error("%s: ST is not supported, err:%d\n",
                       __func__, -QDMA_ERR_HWACC_FEATURE_NOT_SUPPORTED);
        return -QDMA_ERR_HWACC_FEATURE_NOT_SUPPORTED;
    }

    return QDMA_SUCCESS;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_global_csr_conf() - Function to configure global CSR
 *
 * This function configures the global CSR based on the specified CSR type and access type.
 * It supports reading and writing operations for various CSR types.
 *
 * @dev_hndl: Device handle
 * @index: Index from where the values need to be read
 * @count: Number of entries to be read
 * @csr_val: Pointer to CSR value
 * @csr_type: Type of the CSR (qdma_global_csr_type enum) to configure
 * @access_type: HW access type (qdma_hw_access_type enum) value
 *               QDMA_HW_ACCESS_CLEAR - Not supported
 *               QDMA_HW_ACCESS_INVALIDATE - Not supported
 *
 * (index + count) shall not be more than 16
 *
 * Return: 0 on success, negative error code on failure
 *****************************************************************************/
int qdma_cpm4_global_csr_conf(void *dev_hndl, uint8_t index, uint8_t count,
                              uint32_t *csr_val,
                              enum qdma_global_csr_type csr_type,
                              enum qdma_hw_access_type access_type)
{
    int rv = QDMA_SUCCESS;

    // MD: Configure global CSR based on CSR type and access type
    switch (csr_type) {
    case QDMA_CSR_RING_SZ:
        switch (access_type) {
        case QDMA_HW_ACCESS_READ:
            rv = qdma_cpm4_read_global_ring_sizes(dev_hndl, index, count, csr_val);
            break;
        case QDMA_HW_ACCESS_WRITE:
            rv = qdma_cpm4_write_global_ring_sizes(dev_hndl, index, count, csr_val);
            break;
        default:
            qdma_log_error("%s: access_type(%d) invalid, err:%d\n",
                           __func__, access_type, -QDMA_ERR_INV_PARAM);
            rv = -QDMA_ERR_INV_PARAM;
            break;
        }
        break;
    case QDMA_CSR_TIMER_CNT:
        switch (access_type) {
        case QDMA_HW_ACCESS_READ:
            rv = qdma_cpm4_read_global_timer_count(dev_hndl, index, count, csr_val);
            break;
        case QDMA_HW_ACCESS_WRITE:
            rv = qdma_cpm4_write_global_timer_count(dev_hndl, index, count, csr_val);
            break;
        default:
            qdma_log_error("%s: access_type(%d) invalid, err:%d\n",
                           __func__, access_type, -QDMA_ERR_INV_PARAM);
            rv = -QDMA_ERR_INV_PARAM;
            break;
        }
        break;
    case QDMA_CSR_CNT_TH:
        switch (access_type) {
        case QDMA_HW_ACCESS_READ:
            rv = qdma_cpm4_read_global_counter_threshold(dev_hndl, index, count, csr_val);
            break;
        case QDMA_HW_ACCESS_WRITE:
            rv = qdma_cpm4_write_global_counter_threshold(dev_hndl, index, count, csr_val);
            break;
        default:
            qdma_log_error("%s: access_type(%d) invalid, err:%d\n",
                           __func__, access_type, -QDMA_ERR_INV_PARAM);
            rv = -QDMA_ERR_INV_PARAM;
            break;
        }
        break;
    case QDMA_CSR_BUF_SZ:
        switch (access_type) {
        case QDMA_HW_ACCESS_READ:
            rv = qdma_cpm4_read_global_buffer_sizes(dev_hndl, index, count, csr_val);
            break;
        case QDMA_HW_ACCESS_WRITE:
            rv = qdma_cpm4_write_global_buffer_sizes(dev_hndl, index, count, csr_val);
            break;
        default:
            qdma_log_error("%s: access_type(%d) invalid, err:%d\n",
                           __func__, access_type, -QDMA_ERR_INV_PARAM);
            rv = -QDMA_ERR_INV_PARAM;
            break;
        }
        break;
    default:
        qdma_log_error("%s: csr_type(%d) invalid, err:%d\n",
                       __func__, csr_type, -QDMA_ERR_INV_PARAM);
        rv = -QDMA_ERR_INV_PARAM;
        break;
    }

    return rv;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_global_writeback_interval_write() - Set the writeback interval
 *
 * This function sets the global writeback interval for the device.
 *
 * @dev_hndl: Device handle
 * @wb_int: Writeback Interval
 *
 * Return: 0 on success, negative error code on failure
 *****************************************************************************/
static int qdma_cpm4_global_writeback_interval_write(void *dev_hndl,
                                                     enum qdma_wrb_interval wb_int)
{
    uint32_t reg_val;
    struct qdma_dev_attributes dev_cap;

    // MD: Validate input parameter
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n", __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Validate writeback interval
    if (wb_int >= QDMA_NUM_WRB_INTERVALS) {
        qdma_log_error("%s: wb_int=%d is invalid, err:%d\n", __func__, wb_int, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Get device attributes
    qdma_cpm4_get_device_attributes(dev_hndl, &dev_cap);

    // MD: Check if ST or MM completion is enabled
    if (dev_cap.st_en || dev_cap.mm_cmpt_en) {
        // MD: Read current register value
        reg_val = qdma_reg_read(dev_hndl, QDMA_CPM4_GLBL_DSC_CFG_ADDR);
        // MD: Set writeback interval
        reg_val |= FIELD_SET(GLBL_DSC_CFG_WB_ACC_INT_MASK, wb_int);
        // MD: Write updated register value
        qdma_reg_write(dev_hndl, QDMA_CPM4_GLBL_DSC_CFG_ADDR, reg_val);
    } else {
        qdma_log_error("%s: ST or MM cmpt not supported, err:%d\n", __func__, -QDMA_ERR_HWACC_FEATURE_NOT_SUPPORTED);
        return -QDMA_ERR_HWACC_FEATURE_NOT_SUPPORTED;
    }

    return QDMA_SUCCESS;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_global_writeback_interval_read() - Get the writeback interval
 *
 * This function retrieves the global writeback interval for the device.
 *
 * @dev_hndl: Device handle
 * @wb_int: Pointer to the data to hold Writeback Interval
 *
 * Return: 0 on success, negative error code on failure
 *****************************************************************************/
static int qdma_cpm4_global_writeback_interval_read(void *dev_hndl,
                                                    enum qdma_wrb_interval *wb_int)
{
    uint32_t reg_val;
    struct qdma_dev_attributes dev_cap;

    // MD: Validate input parameters
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n", __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    if (!wb_int) {
        qdma_log_error("%s: wb_int is NULL, err:%d\n", __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Get device attributes
    qdma_cpm4_get_device_attributes(dev_hndl, &dev_cap);

    // MD: Check if ST or MM completion is enabled
    if (dev_cap.st_en || dev_cap.mm_cmpt_en) {
        // MD: Read current register value
        reg_val = qdma_reg_read(dev_hndl, QDMA_CPM4_GLBL_DSC_CFG_ADDR);
        // MD: Extract writeback interval
        *wb_int = (enum qdma_wrb_interval)FIELD_GET(GLBL_DSC_CFG_WB_ACC_INT_MASK, reg_val);
    } else {
        qdma_log_error("%s: ST or MM cmpt not supported, err:%d\n", __func__, -QDMA_ERR_HWACC_FEATURE_NOT_SUPPORTED);
        return -QDMA_ERR_HWACC_FEATURE_NOT_SUPPORTED;
    }

    return QDMA_SUCCESS;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_global_writeback_interval_conf() - Configure the writeback interval
 *
 * This function configures the global writeback interval based on the specified access type.
 * Note: QDMA_HW_ACCESS_CLEAR and QDMA_HW_ACCESS_INVALIDATE are not supported.
 *
 * @dev_hndl: Device handle
 * @wb_int: Pointer to the data to hold Writeback Interval
 * @access_type: HW access type (qdma_hw_access_type enum) value
 *
 * Return: 0 on success, negative error code on failure
 *****************************************************************************/
int qdma_cpm4_global_writeback_interval_conf(void *dev_hndl,
                                             enum qdma_wrb_interval *wb_int,
                                             enum qdma_hw_access_type access_type)
{
    int rv = QDMA_SUCCESS;

    // MD: Configure writeback interval based on access type
    switch (access_type) {
    case QDMA_HW_ACCESS_READ:
        rv = qdma_cpm4_global_writeback_interval_read(dev_hndl, wb_int);
        break;
    case QDMA_HW_ACCESS_WRITE:
        rv = qdma_cpm4_global_writeback_interval_write(dev_hndl, *wb_int);
        break;
    case QDMA_HW_ACCESS_CLEAR:
    case QDMA_HW_ACCESS_INVALIDATE:
    default:
        qdma_log_error("%s: access_type(%d) invalid, err:%d\n", __func__, access_type, -QDMA_ERR_INV_PARAM);
        rv = -QDMA_ERR_INV_PARAM;
        break;
    }

    return rv;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_mm_channel_conf() - Enable/disable the MM channel
 *
 * This function enables or disables the Memory-Mapped (MM) channel based on the specified parameters.
 * Currently, only one MM channel is supported.
 *
 * @dev_hndl: Device handle
 * @channel: MM channel number
 * @is_c2h: Queue direction. Set 1 for C2H and 0 for H2C
 * @enable: Enable or disable MM channel
 *
 * Return: 0 on success, negative error code on failure
 *****************************************************************************/
int qdma_cpm4_mm_channel_conf(void *dev_hndl, uint8_t channel,
                              uint8_t is_c2h, uint8_t enable)
{
    uint32_t reg_addr = (is_c2h) ? QDMA_CPM4_C2H_CHANNEL_CTL_ADDR : QDMA_CPM4_H2C_CHANNEL_CTL_ADDR;
    struct qdma_dev_attributes dev_cap;

    // MD: Validate input parameter
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n", __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Get device attributes
    qdma_cpm4_get_device_attributes(dev_hndl, &dev_cap);

    // MD: Enable or disable MM channel if supported
    if (dev_cap.mm_en) {
        qdma_reg_write(dev_hndl, reg_addr + (channel * QDMA_MM_CONTROL_STEP), enable);
    }

    return QDMA_SUCCESS;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_cpm4_dump_reg_info() - Dump register information
 *
 * This function dumps the register information for a specified address and number of registers.
 * It formats the output into a buffer or logs it directly if the buffer is not provided.
 *
 * @dev_hndl: Device handle
 * @reg_addr: Starting register address
 * @num_regs: Number of registers to dump
 * @buf: Buffer to store the formatted output
 * @buflen: Length of the buffer
 *
 * Return: Length of data written to buffer on success, negative error code on failure
 *****************************************************************************/
int qdma_cpm4_dump_reg_info(void *dev_hndl, uint32_t reg_addr,
                            uint32_t num_regs, char *buf, uint32_t buflen)
{
    uint32_t total_num_regs = qdma_cpm4_config_num_regs_get();
    struct xreg_info *config_regs = qdma_cpm4_config_regs_get();
    const char *bitfield_name;
    uint32_t i = 0, num_regs_idx = 0, k = 0, j = 0, bitfield = 0, lsb = 0, msb = 31;
    int rv = 0;
    uint32_t reg_val;
    uint32_t data_len = 0;

    // MD: Validate input parameter
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n", __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Find the starting register index
    for (i = 0; i < total_num_regs; i++) {
        if (reg_addr == config_regs[i].addr) {
            j = i;
            break;
        }
    }

    // MD: Check if register was found
    if (i == total_num_regs) {
        qdma_log_error("%s: Register not found err:%d\n", __func__, -QDMA_ERR_INV_PARAM);
        if (buf)
            QDMA_SNPRINTF_S(buf, buflen, DEBGFS_LINE_SZ, "Register not found 0x%x\n", reg_addr);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Determine the number of registers to dump
    num_regs_idx = (j + num_regs < total_num_regs) ? (j + num_regs) : total_num_regs;

    // MD: Dump register information
    for (; j < num_regs_idx; j++) {
        reg_val = qdma_reg_read(dev_hndl, config_regs[j].addr);

        if (buf) {
            rv = QDMA_SNPRINTF_S(buf, buflen, DEBGFS_LINE_SZ,
                                 "\n%-40s 0x%-7x %-#10x %-10d\n",
                                 config_regs[j].name,
                                 config_regs[j].addr,
                                 reg_val, reg_val);
            if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
                qdma_log_error("%s: Insufficient buffer, err:%d\n", __func__, -QDMA_ERR_NO_MEM);
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

        // MD: Dump bitfield information
        for (k = 0; k < config_regs[j].num_bitfields; k++) {
            bitfield = config_regs[j].bitfields[k].field_mask;
            bitfield_name = config_regs[i].bitfields[k].field_name;
            lsb = 0;
            msb = 31;

            while (!(BIT(lsb) & bitfield))
                lsb++;

            while (!(BIT(msb) & bitfield))
                msb--;

            if (msb != lsb) {
                if (buf) {
                    rv = QDMA_SNPRINTF_S(buf, buflen, DEBGFS_LINE_SZ,
                                         "%-40s [%2u,%2u]   %#-10x\n",
                                         bitfield_name,
                                         msb, lsb,
                                         (reg_val & bitfield) >> lsb);
                    if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
                        qdma_log_error("%s: Insufficient buffer, err:%d\n", __func__, -QDMA_ERR_NO_MEM);
                        return -QDMA_ERR_NO_MEM;
                    }
                    buf += rv;
                    data_len += rv;
                    buflen -= rv;
                } else {
                    qdma_log_info("%-40s [%2u,%2u]   %#-10x\n",
                                  bitfield_name,
                                  msb, lsb,
                                  (reg_val & bitfield) >> lsb);
                }
            } else {
                if (buf) {
                    rv = QDMA_SNPRINTF_S(buf, buflen, DEBGFS_LINE_SZ,
                                         "%-40s [%5u]   %#-10x\n",
                                         bitfield_name,
                                         lsb,
                                         (reg_val & bitfield) >> lsb);
                    if ((rv < 0) || (rv > DEBGFS_LINE_SZ)) {
                        qdma_log_error("%s: Insufficient buffer, err:%d\n", __func__, -QDMA_ERR_NO_MEM);
                        return -QDMA_ERR_NO_MEM;
                    }
                    buf += rv;
                    data_len += rv;
                    buflen -= rv;
                } else {
                    qdma_log_info("%-40s [%5u]   %#-10x\n",
                                  bitfield_name,
                                  lsb,
                                  (reg_val & bitfield) >> lsb);
                }
            }
        }
    }

    return data_len;
}
