/* MD:
 * Copyright (c) 2019-2022 Xilinx, Inc. All rights reserved.
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

#include "eqdma_soft_reg.h"
#include "qdma_reg_dump.h"
#include <linux/kernel.h> // MD: For printk()

#ifdef ENABLE_WPP_TRACING
#include "eqdma_soft_reg_dump.tmh"
#endif

/* MD:
 * Structure to hold register field information for CFG_BLK_IDENTIFIER.
 */
static struct regfield_info cfg_blk_identifier_field_info[] = {
    {"CFG_BLK_IDENTIFIER", CFG_BLK_IDENTIFIER_MASK},
    {"CFG_BLK_IDENTIFIER_1", CFG_BLK_IDENTIFIER_1_MASK},
    {"CFG_BLK_IDENTIFIER_RSVD_1", CFG_BLK_IDENTIFIER_RSVD_1_MASK},
    {"CFG_BLK_IDENTIFIER_VERSION", CFG_BLK_IDENTIFIER_VERSION_MASK},
};

/* MD:
 * Structure to hold register field information for CFG_BLK_PCIE_MAX_PLD_SIZE.
 */
static struct regfield_info cfg_blk_pcie_max_pld_size_field_info[] = {
    {"CFG_BLK_PCIE_MAX_PLD_SIZE_RSVD_1", CFG_BLK_PCIE_MAX_PLD_SIZE_RSVD_1_MASK},
    {"CFG_BLK_PCIE_MAX_PLD_SIZE_PROG", CFG_BLK_PCIE_MAX_PLD_SIZE_PROG_MASK},
    {"CFG_BLK_PCIE_MAX_PLD_SIZE_RSVD_2", CFG_BLK_PCIE_MAX_PLD_SIZE_RSVD_2_MASK},
    {"CFG_BLK_PCIE_MAX_PLD_SIZE_ISSUED", CFG_BLK_PCIE_MAX_PLD_SIZE_ISSUED_MASK},
};

/* MD:
 * Structure to hold register field information for CFG_BLK_PCIE_MAX_READ_REQ_SIZE.
 */
static struct regfield_info cfg_blk_pcie_max_read_req_size_field_info[] = {
    {"CFG_BLK_PCIE_MAX_READ_REQ_SIZE_RSVD_1", CFG_BLK_PCIE_MAX_READ_REQ_SIZE_RSVD_1_MASK},
    {"CFG_BLK_PCIE_MAX_READ_REQ_SIZE_PROG", CFG_BLK_PCIE_MAX_READ_REQ_SIZE_PROG_MASK},
    {"CFG_BLK_PCIE_MAX_READ_REQ_SIZE_RSVD_2", CFG_BLK_PCIE_MAX_READ_REQ_SIZE_RSVD_2_MASK},
    {"CFG_BLK_PCIE_MAX_READ_REQ_SIZE_ISSUED", CFG_BLK_PCIE_MAX_READ_REQ_SIZE_ISSUED_MASK},
};

/* MD:
 * Structure to hold register field information for CFG_BLK_SYSTEM_ID.
 */
static struct regfield_info cfg_blk_system_id_field_info[] = {
    {"CFG_BLK_SYSTEM_ID_RSVD_1", CFG_BLK_SYSTEM_ID_RSVD_1_MASK},
    {"CFG_BLK_SYSTEM_ID_INST_TYPE", CFG_BLK_SYSTEM_ID_INST_TYPE_MASK},
    {"CFG_BLK_SYSTEM_ID", CFG_BLK_SYSTEM_ID_MASK},
};

/* MD:
 * Structure to hold register field information for CFG_BLK_MSIX_ENABLE.
 */
static struct regfield_info cfg_blk_msix_enable_field_info[] = {
    {"CFG_BLK_MSIX_ENABLE", CFG_BLK_MSIX_ENABLE_MASK},
};

/* MD:
 * Structure to hold register field information for CFG_PCIE_DATA_WIDTH.
 */
static struct regfield_info cfg_pcie_data_width_field_info[] = {
    {"CFG_PCIE_DATA_WIDTH_RSVD_1", CFG_PCIE_DATA_WIDTH_RSVD_1_MASK},
    {"CFG_PCIE_DATA_WIDTH_DATAPATH", CFG_PCIE_DATA_WIDTH_DATAPATH_MASK},
};

/* MD:
 * Structure to hold register field information for CFG_PCIE_CTL.
 */
static struct regfield_info cfg_pcie_ctl_field_info[] = {
    {"CFG_PCIE_CTL_RSVD_1", CFG_PCIE_CTL_RSVD_1_MASK},
    {"CFG_PCIE_CTL_MGMT_AXIL_CTRL", CFG_PCIE_CTL_MGMT_AXIL_CTRL_MASK},
    {"CFG_PCIE_CTL_RSVD_2", CFG_PCIE_CTL_RSVD_2_MASK},
    {"CFG_PCIE_CTL_RRQ_DISABLE", CFG_PCIE_CTL_RRQ_DISABLE_MASK},
    {"CFG_PCIE_CTL_RELAXED_ORDERING", CFG_PCIE_CTL_RELAXED_ORDERING_MASK},
};

/* MD:
 * Structure to hold register field information for CFG_BLK_MSI_ENABLE.
 */
static struct regfield_info cfg_blk_msi_enable_field_info[] = {
    {"CFG_BLK_MSI_ENABLE", CFG_BLK_MSI_ENABLE_MASK},
};

/* MD:
 * Structure to hold register field information for CFG_AXI_USER_MAX_PLD_SIZE.
 */
static struct regfield_info cfg_axi_user_max_pld_size_field_info[] = {
    {"CFG_AXI_USER_MAX_PLD_SIZE_RSVD_1", CFG_AXI_USER_MAX_PLD_SIZE_RSVD_1_MASK},
    {"CFG_AXI_USER_MAX_PLD_SIZE_ISSUED", CFG_AXI_USER_MAX_PLD_SIZE_ISSUED_MASK},
    {"CFG_AXI_USER_MAX_PLD_SIZE_RSVD_2", CFG_AXI_USER_MAX_PLD_SIZE_RSVD_2_MASK},
    {"CFG_AXI_USER_MAX_PLD_SIZE_PROG", CFG_AXI_USER_MAX_PLD_SIZE_PROG_MASK},
};

/* MD:
 * Structure to hold register field information for CFG_AXI_USER_MAX_READ_REQ_SIZE.
 */
static struct regfield_info cfg_axi_user_max_read_req_size_field_info[] = {
    {"CFG_AXI_USER_MAX_READ_REQ_SIZE_RSVD_1", CFG_AXI_USER_MAX_READ_REQ_SIZE_RSVD_1_MASK},
    {"CFG_AXI_USER_MAX_READ_REQ_SIZE_USISSUED", CFG_AXI_USER_MAX_READ_REQ_SIZE_USISSUED_MASK},
    {"CFG_AXI_USER_MAX_READ_REQ_SIZE_RSVD_2", CFG_AXI_USER_MAX_READ_REQ_SIZE_RSVD_2_MASK},
    {"CFG_AXI_USER_MAX_READ_REQ_SIZE_USPROG", CFG_AXI_USER_MAX_READ_REQ_SIZE_USPROG_MASK},
};

/* MD:
 * Structure to hold register field information for CFG_BLK_MISC_CTL.
 */
static struct regfield_info cfg_blk_misc_ctl_field_info[] = {
    {"CFG_BLK_MISC_CTL_RSVD_1", CFG_BLK_MISC_CTL_RSVD_1_MASK},
    {"CFG_BLK_MISC_CTL_10B_TAG_EN", CFG_BLK_MISC_CTL_10B_TAG_EN_MASK},
    {"CFG_BLK_MISC_CTL_RSVD_2", CFG_BLK_MISC_CTL_RSVD_2_MASK},
    {"CFG_BLK_MISC_CTL_AXI_WBK", CFG_BLK_MISC_CTL_AXI_WBK_MASK},
    {"CFG_BLK_MISC_CTL_AXI_DSC", CFG_BLK_MISC_CTL_AXI_DSC_MASK},
    {"CFG_BLK_MISC_CTL_NUM_TAG", CFG_BLK_MISC_CTL_NUM_TAG_MASK},
    {"CFG_BLK_MISC_CTL_RSVD_3", CFG_BLK_MISC_CTL_RSVD_3_MASK},
    {"CFG_BLK_MISC_CTL_RQ_METERING_MULTIPLIER", CFG_BLK_MISC_CTL_RQ_METERING_MULTIPLIER_MASK},
}

/* MD:
 * Structure to hold register field information for CFG_PL_CRED_CTL.
 * This structure defines the fields related to credit control in the PL (Programmable Logic) layer.
 */
static struct regfield_info cfg_pl_cred_ctl_field_info[] = {
    {"CFG_PL_CRED_CTL_RSVD_1", CFG_PL_CRED_CTL_RSVD_1_MASK},
    {"CFG_PL_CRED_CTL_SLAVE_CRD_RLS", CFG_PL_CRED_CTL_SLAVE_CRD_RLS_MASK},
    {"CFG_PL_CRED_CTL_RSVD_2", CFG_PL_CRED_CTL_RSVD_2_MASK},
    {"CFG_PL_CRED_CTL_MASTER_CRD_RST", CFG_PL_CRED_CTL_MASTER_CRD_RST_MASK},
};

/* MD:
 * Structure to hold register field information for CFG_BLK_SCRATCH.
 * This structure defines the fields related to the scratch register block.
 */
static struct regfield_info cfg_blk_scratch_field_info[] = {
    {"CFG_BLK_SCRATCH", CFG_BLK_SCRATCH_MASK},
};

/* MD:
 * Structure to hold register field information for CFG_GIC.
 * This structure defines the fields related to the GIC (Generic Interrupt Controller).
 */
static struct regfield_info cfg_gic_field_info[] = {
    {"CFG_GIC_RSVD_1", CFG_GIC_RSVD_1_MASK},
    {"CFG_GIC_GIC_IRQ", CFG_GIC_GIC_IRQ_MASK},
};

/* MD:
 * Structure to hold register field information for RAM_SBE_MSK_1_A.
 * This structure defines the fields related to single-bit error masking in RAM.
 */
static struct regfield_info ram_sbe_msk_1_a_field_info[] = {
    {"RAM_SBE_MSK_1_A", RAM_SBE_MSK_1_A_MASK},
};

/* MD:
 * Structure to hold register field information for RAM_SBE_STS_1_A.
 * This structure defines the fields related to single-bit error status in RAM.
 */
static struct regfield_info ram_sbe_sts_1_a_field_info[] = {
    {"RAM_SBE_STS_1_A_RSVD", RAM_SBE_STS_1_A_RSVD_MASK},
    {"RAM_SBE_STS_1_A_PFCH_CTXT_CAM_RAM_1", RAM_SBE_STS_1_A_PFCH_CTXT_CAM_RAM_1_MASK},
    {"RAM_SBE_STS_1_A_PFCH_CTXT_CAM_RAM_0", RAM_SBE_STS_1_A_PFCH_CTXT_CAM_RAM_0_MASK},
    {"RAM_SBE_STS_1_A_TAG_EVEN_RAM", RAM_SBE_STS_1_A_TAG_EVEN_RAM_MASK},
    {"RAM_SBE_STS_1_A_TAG_ODD_RAM", RAM_SBE_STS_1_A_TAG_ODD_RAM_MASK},
    {"RAM_SBE_STS_1_A_RC_RRQ_EVEN_RAM", RAM_SBE_STS_1_A_RC_RRQ_EVEN_RAM_MASK},
};

/* MD:
 * Structure to hold register field information for RAM_DBE_MSK_1_A.
 * This structure defines the fields related to double-bit error masking in RAM.
 */
static struct regfield_info ram_dbe_msk_1_a_field_info[] = {
    {"RAM_DBE_MSK_1_A", RAM_DBE_MSK_1_A_MASK},
};

/* MD:
 * Structure to hold register field information for RAM_DBE_STS_1_A.
 * This structure defines the fields related to double-bit error status in RAM.
 */
static struct regfield_info ram_dbe_sts_1_a_field_info[] = {
    {"RAM_DBE_STS_1_A_RSVD", RAM_DBE_STS_1_A_RSVD_MASK},
    {"RAM_DBE_STS_1_A_PFCH_CTXT_CAM_RAM_1", RAM_DBE_STS_1_A_PFCH_CTXT_CAM_RAM_1_MASK},
    {"RAM_DBE_STS_1_A_PFCH_CTXT_CAM_RAM_0", RAM_DBE_STS_1_A_PFCH_CTXT_CAM_RAM_0_MASK},
    {"RAM_DBE_STS_1_A_TAG_EVEN_RAM", RAM_DBE_STS_1_A_TAG_EVEN_RAM_MASK},
    {"RAM_DBE_STS_1_A_TAG_ODD_RAM", RAM_DBE_STS_1_A_TAG_ODD_RAM_MASK},
    {"RAM_DBE_STS_1_A_RC_RRQ_EVEN_RAM", RAM_DBE_STS_1_A_RC_RRQ_EVEN_RAM_MASK},
};

/* MD:
 * Structure to hold register field information for RAM_SBE_MSK_A.
 * This structure defines the fields related to single-bit error masking in RAM.
 */
static struct regfield_info ram_sbe_msk_a_field_info[] = {
    {"RAM_SBE_MSK_A", RAM_SBE_MSK_A_MASK},
};

/* MD:
 * Structure to hold register field information for RAM_SBE_STS_A.
 * This structure defines the fields related to single-bit error status in RAM.
 */
static struct regfield_info ram_sbe_sts_a_field_info[] = {
    {"RAM_SBE_STS_A_RC_RRQ_ODD_RAM", RAM_SBE_STS_A_RC_RRQ_ODD_RAM_MASK},
    {"RAM_SBE_STS_A_PEND_FIFO_RAM", RAM_SBE_STS_A_PEND_FIFO_RAM_MASK},
    {"RAM_SBE_STS_A_PFCH_LL_RAM", RAM_SBE_STS_A_PFCH_LL_RAM_MASK},
    {"RAM_SBE_STS_A_WRB_CTXT_RAM", RAM_SBE_STS_A_WRB_CTXT_RAM_MASK},
    {"RAM_SBE_STS_A_PFCH_CTXT_RAM", RAM_SBE_STS_A_PFCH_CTXT_RAM_MASK},
    {"RAM_SBE_STS_A_DESC_REQ_FIFO_RAM", RAM_SBE_STS_A_DESC_REQ_FIFO_RAM_MASK},
    {"RAM_SBE_STS_A_INT_CTXT_RAM", RAM_SBE_STS_A_INT_CTXT_RAM_MASK},
    {"RAM_SBE_STS_A_WRB_COAL_DATA_RAM", RAM_SBE_STS_A_WRB_COAL_DATA_RAM_MASK},
    {"RAM_SBE_STS_A_QID_FIFO_RAM", RAM_SBE_STS_A_QID_FIFO_RAM_MASK},
    {"RAM_SBE_STS_A_TIMER_FIFO_RAM", RAM_SBE_STS_A_TIMER_FIFO_RAM_MASK},
    {"RAM_SBE_STS_A_MI_TL_SLV_FIFO_RAM", RAM_SBE_STS_A_MI_TL_SLV_FIFO_RAM_MASK},
    {"RAM_SBE_STS_A_DSC_CPLD", RAM_SBE_STS_A_DSC_CPLD_MASK},
    {"RAM_SBE_STS_A_DSC_CPLI", RAM_SBE_STS_A_DSC_CPLI_MASK},
    {"RAM_SBE_STS_A_DSC_SW_CTXT", RAM_SBE_STS_A_DSC_SW_CTXT_MASK},
    {"RAM_SBE_STS_A_DSC_CRD_RCV", RAM_SBE_STS_A_DSC_CRD_RCV_MASK},
    {"RAM_SBE_STS_A_DSC_HW_CTXT", RAM_SBE_STS_A_DSC_HW_CTXT_MASK},
    {"RAM_SBE_STS_A_FUNC_MAP", RAM_SBE_STS_A_FUNC_MAP_MASK},
    {"RAM_SBE_STS_A_C2H_WR_BRG_DAT", RAM_SBE_STS_A_C2H_WR_BRG_DAT_MASK},
    {"RAM_SBE_STS_A_C2H_RD_BRG_DAT", RAM_SBE_STS_A_C2H_RD_BRG_DAT_MASK},
    {"RAM_SBE_STS_A_H2C_WR_BRG_DAT", RAM_SBE_STS_A_H2C_WR_BRG_DAT_MASK},
    {"RAM_SBE_STS_A_H2C_RD_BRG_DAT", RAM_SBE_STS_A_H2C_RD_BRG_DAT_MASK},
    {"RAM_SBE_STS_A_MI_C2H3_DAT", RAM_SBE_STS_A_MI_C2H3_DAT_MASK},
    {"RAM_SBE_STS_A_MI_C2H2_DAT", RAM_SBE_STS_A_MI_C2H2_DAT_MASK},
    {"RAM_SBE_STS_A_MI_C2H1_DAT", RAM_SBE_STS_A_MI_C2H1_DAT_MASK},
    {"RAM_SBE_STS_A_MI_C2H0_DAT", RAM_SBE_STS_A_MI_C2H0_DAT_MASK},
    {"RAM_SBE_STS_A_MI_H2C3_DAT", RAM_SBE_STS_A_MI_H2C3_DAT_MASK},
    {"RAM_SBE_STS_A_MI_H2C2_DAT", RAM_SBE_STS_A_MI_H2C2_DAT_MASK},
    {"RAM_SBE_STS_A_MI_H2C1_DAT", RAM_SBE_STS_A_MI_H2C1_DAT_MASK},
    {"RAM_SBE_STS_A_MI_H2C0_DAT", RAM_SBE_STS_A_MI_H2C0_DAT_MASK},
};

/* MD:
 * Structure to hold register field information for RAM_DBE_MSK_A.
 * This structure defines the fields related to double-bit error masking in RAM.
 */
static struct regfield_info ram_dbe_msk_a_field_info[] = {
    {"RAM_DBE_MSK_A", RAM_DBE_MSK_A_MASK},
};

#include <linux/kernel.h>  // MD: For printk

/* MD:
 * RAM Double Bit Error Status A Field Information
 */
static struct regfield_info ram_dbe_sts_a_field_info[] = {
    {"RAM_DBE_STS_A_RC_RRQ_ODD_RAM", RAM_DBE_STS_A_RC_RRQ_ODD_RAM_MASK},
    {"RAM_DBE_STS_A_PEND_FIFO_RAM", RAM_DBE_STS_A_PEND_FIFO_RAM_MASK},
    {"RAM_DBE_STS_A_PFCH_LL_RAM", RAM_DBE_STS_A_PFCH_LL_RAM_MASK},
    {"RAM_DBE_STS_A_WRB_CTXT_RAM", RAM_DBE_STS_A_WRB_CTXT_RAM_MASK},
    {"RAM_DBE_STS_A_PFCH_CTXT_RAM", RAM_DBE_STS_A_PFCH_CTXT_RAM_MASK},
    {"RAM_DBE_STS_A_DESC_REQ_FIFO_RAM", RAM_DBE_STS_A_DESC_REQ_FIFO_RAM_MASK},
    {"RAM_DBE_STS_A_INT_CTXT_RAM", RAM_DBE_STS_A_INT_CTXT_RAM_MASK},
    {"RAM_DBE_STS_A_WRB_COAL_DATA_RAM", RAM_DBE_STS_A_WRB_COAL_DATA_RAM_MASK},
    {"RAM_DBE_STS_A_QID_FIFO_RAM", RAM_DBE_STS_A_QID_FIFO_RAM_MASK},
    {"RAM_DBE_STS_A_TIMER_FIFO_RAM", RAM_DBE_STS_A_TIMER_FIFO_RAM_MASK},
    {"RAM_DBE_STS_A_MI_TL_SLV_FIFO_RAM", RAM_DBE_STS_A_MI_TL_SLV_FIFO_RAM_MASK},
    {"RAM_DBE_STS_A_DSC_CPLD", RAM_DBE_STS_A_DSC_CPLD_MASK},
    {"RAM_DBE_STS_A_DSC_CPLI", RAM_DBE_STS_A_DSC_CPLI_MASK},
    {"RAM_DBE_STS_A_DSC_SW_CTXT", RAM_DBE_STS_A_DSC_SW_CTXT_MASK},
    {"RAM_DBE_STS_A_DSC_CRD_RCV", RAM_DBE_STS_A_DSC_CRD_RCV_MASK},
    {"RAM_DBE_STS_A_DSC_HW_CTXT", RAM_DBE_STS_A_DSC_HW_CTXT_MASK},
    {"RAM_DBE_STS_A_FUNC_MAP", RAM_DBE_STS_A_FUNC_MAP_MASK},
    {"RAM_DBE_STS_A_C2H_WR_BRG_DAT", RAM_DBE_STS_A_C2H_WR_BRG_DAT_MASK},
    {"RAM_DBE_STS_A_C2H_RD_BRG_DAT", RAM_DBE_STS_A_C2H_RD_BRG_DAT_MASK},
    {"RAM_DBE_STS_A_H2C_WR_BRG_DAT", RAM_DBE_STS_A_H2C_WR_BRG_DAT_MASK},
    {"RAM_DBE_STS_A_H2C_RD_BRG_DAT", RAM_DBE_STS_A_H2C_RD_BRG_DAT_MASK},
    {"RAM_DBE_STS_A_MI_C2H3_DAT", RAM_DBE_STS_A_MI_C2H3_DAT_MASK},
    {"RAM_DBE_STS_A_MI_C2H2_DAT", RAM_DBE_STS_A_MI_C2H2_DAT_MASK},
    {"RAM_DBE_STS_A_MI_C2H1_DAT", RAM_DBE_STS_A_MI_C2H1_DAT_MASK},
    {"RAM_DBE_STS_A_MI_C2H0_DAT", RAM_DBE_STS_A_MI_C2H0_DAT_MASK},
    {"RAM_DBE_STS_A_MI_H2C3_DAT", RAM_DBE_STS_A_MI_H2C3_DAT_MASK},
    {"RAM_DBE_STS_A_MI_H2C2_DAT", RAM_DBE_STS_A_MI_H2C2_DAT_MASK},
    {"RAM_DBE_STS_A_MI_H2C1_DAT", RAM_DBE_STS_A_MI_H2C1_DAT_MASK},
    {"RAM_DBE_STS_A_MI_H2C0_DAT", RAM_DBE_STS_A_MI_H2C0_DAT_MASK},
};

/* MD:
 * Global Identifier Field Information
 */
static struct regfield_info glbl2_identifier_field_info[] = {
    {"GLBL2_IDENTIFIER", GLBL2_IDENTIFIER_MASK},
    {"GLBL2_IDENTIFIER_VERSION", GLBL2_IDENTIFIER_VERSION_MASK},
};

/* MD:
 * Global Channel Instance Field Information
 */
static struct regfield_info glbl2_channel_inst_field_info[] = {
    {"GLBL2_CHANNEL_INST_RSVD_1", GLBL2_CHANNEL_INST_RSVD_1_MASK},
    {"GLBL2_CHANNEL_INST_C2H_ST", GLBL2_CHANNEL_INST_C2H_ST_MASK},
    {"GLBL2_CHANNEL_INST_H2C_ST", GLBL2_CHANNEL_INST_H2C_ST_MASK},
    {"GLBL2_CHANNEL_INST_RSVD_2", GLBL2_CHANNEL_INST_RSVD_2_MASK},
    {"GLBL2_CHANNEL_INST_C2H_ENG", GLBL2_CHANNEL_INST_C2H_ENG_MASK},
    {"GLBL2_CHANNEL_INST_RSVD_3", GLBL2_CHANNEL_INST_RSVD_3_MASK},
    {"GLBL2_CHANNEL_INST_H2C_ENG", GLBL2_CHANNEL_INST_H2C_ENG_MASK},
};

/* MD:
 * Global Channel MDMA Field Information
 */
static struct regfield_info glbl2_channel_mdma_field_info[] = {
    {"GLBL2_CHANNEL_MDMA_RSVD_1", GLBL2_CHANNEL_MDMA_RSVD_1_MASK},
    {"GLBL2_CHANNEL_MDMA_C2H_ST", GLBL2_CHANNEL_MDMA_C2H_ST_MASK},
    {"GLBL2_CHANNEL_MDMA_H2C_ST", GLBL2_CHANNEL_MDMA_H2C_ST_MASK},
    {"GLBL2_CHANNEL_MDMA_RSVD_2", GLBL2_CHANNEL_MDMA_RSVD_2_MASK},
    {"GLBL2_CHANNEL_MDMA_C2H_ENG", GLBL2_CHANNEL_MDMA_C2H_ENG_MASK},
    {"GLBL2_CHANNEL_MDMA_RSVD_3", GLBL2_CHANNEL_MDMA_RSVD_3_MASK},
    {"GLBL2_CHANNEL_MDMA_H2C_ENG", GLBL2_CHANNEL_MDMA_H2C_ENG_MASK},
};

/* MD:
 * Global Channel Stream Field Information
 */
static struct regfield_info glbl2_channel_strm_field_info[] = {
    {"GLBL2_CHANNEL_STRM_RSVD_1", GLBL2_CHANNEL_STRM_RSVD_1_MASK},
    {"GLBL2_CHANNEL_STRM_C2H_ST", GLBL2_CHANNEL_STRM_C2H_ST_MASK},
    {"GLBL2_CHANNEL_STRM_H2C_ST", GLBL2_CHANNEL_STRM_H2C_ST_MASK},
    {"GLBL2_CHANNEL_STRM_RSVD_2", GLBL2_CHANNEL_STRM_RSVD_2_MASK},
    {"GLBL2_CHANNEL_STRM_C2H_ENG", GLBL2_CHANNEL_STRM_C2H_ENG_MASK},
    {"GLBL2_CHANNEL_STRM_RSVD_3", GLBL2_CHANNEL_STRM_RSVD_3_MASK},
    {"GLBL2_CHANNEL_STRM_H2C_ENG", GLBL2_CHANNEL_STRM_H2C_ENG_MASK},
};

/* MD:
 * Global Channel Capability Field Information
 */
static struct regfield_info glbl2_channel_cap_field_info[] = {
    {"GLBL2_CHANNEL_CAP_RSVD_1", GLBL2_CHANNEL_CAP_RSVD_1_MASK},
    {"GLBL2_CHANNEL_CAP_MULTIQ_MAX", GLBL2_CHANNEL_CAP_MULTIQ_MAX_MASK},
};

/* MD:
 * Global Channel PASID Capability Field Information
 */
static struct regfield_info glbl2_channel_pasid_cap_field_info[] = {
    {"GLBL2_CHANNEL_PASID_CAP_RSVD_1", GLBL2_CHANNEL_PASID_CAP_RSVD_1_MASK},
    {"GLBL2_CHANNEL_PASID_CAP_BRIDGEEN", GLBL2_CHANNEL_PASID_CAP_BRIDGEEN_MASK},
    {"GLBL2_CHANNEL_PASID_CAP_DMAEN", GLBL2_CHANNEL_PASID_CAP_DMAEN_MASK},
};

#include <linux/kernel.h>  // MD: For printk

/* MD:
 * glbl2_system_id_field_info - Structure to hold system ID field information.
 */
static struct regfield_info glbl2_system_id_field_info[] = {
    {"GLBL2_SYSTEM_ID_RSVD_1", GLBL2_SYSTEM_ID_RSVD_1_MASK},
    {"GLBL2_SYSTEM_ID", GLBL2_SYSTEM_ID_MASK},
};

/* MD:
 * glbl2_misc_cap_field_info - Structure to hold miscellaneous capabilities field information.
 */
static struct regfield_info glbl2_misc_cap_field_info[] = {
    {"GLBL2_MISC_CAP", GLBL2_MISC_CAP_MASK},
};

/* MD:
 * glbl2_rrq_brg_throt_field_info - Structure to hold bridge throttle field information.
 */
static struct regfield_info glbl2_rrq_brg_throt_field_info[] = {
    {"GLBL2_RRQ_BRG_THROT_REQ_EN", GLBL2_RRQ_BRG_THROT_REQ_EN_MASK},
    {"GLBL2_RRQ_BRG_THROT_REQ", GLBL2_RRQ_BRG_THROT_REQ_MASK},
    {"GLBL2_RRQ_BRG_THROT_DAT_EN", GLBL2_RRQ_BRG_THROT_DAT_EN_MASK},
    {"GLBL2_RRQ_BRG_THROT_DAT", GLBL2_RRQ_BRG_THROT_DAT_MASK},
};

/* MD:
 * glbl2_rrq_pcie_throt_field_info - Structure to hold PCIe throttle field information.
 */
static struct regfield_info glbl2_rrq_pcie_throt_field_info[] = {
    {"GLBL2_RRQ_PCIE_THROT_REQ_EN", GLBL2_RRQ_PCIE_THROT_REQ_EN_MASK},
    {"GLBL2_RRQ_PCIE_THROT_REQ", GLBL2_RRQ_PCIE_THROT_REQ_MASK},
    {"GLBL2_RRQ_PCIE_THROT_DAT_EN", GLBL2_RRQ_PCIE_THROT_DAT_EN_MASK},
    {"GLBL2_RRQ_PCIE_THROT_DAT", GLBL2_RRQ_PCIE_THROT_DAT_MASK},
};

/* MD:
 * glbl2_rrq_aximm_throt_field_info - Structure to hold AXI-MM throttle field information.
 */
static struct regfield_info glbl2_rrq_aximm_throt_field_info[] = {
    {"GLBL2_RRQ_AXIMM_THROT_REQ_EN", GLBL2_RRQ_AXIMM_THROT_REQ_EN_MASK},
    {"GLBL2_RRQ_AXIMM_THROT_REQ", GLBL2_RRQ_AXIMM_THROT_REQ_MASK},
    {"GLBL2_RRQ_AXIMM_THROT_DAT_EN", GLBL2_RRQ_AXIMM_THROT_DAT_EN_MASK},
    {"GLBL2_RRQ_AXIMM_THROT_DAT", GLBL2_RRQ_AXIMM_THROT_DAT_MASK},
};

/* MD:
 * glbl2_rrq_pcie_lat0_field_info - Structure to hold PCIe latency 0 field information.
 */
static struct regfield_info glbl2_rrq_pcie_lat0_field_info[] = {
    {"GLBL2_RRQ_PCIE_LAT0_MAX", GLBL2_RRQ_PCIE_LAT0_MAX_MASK},
    {"GLBL2_RRQ_PCIE_LAT0_MIN", GLBL2_RRQ_PCIE_LAT0_MIN_MASK},
};

/* MD:
 * glbl2_rrq_pcie_lat1_field_info - Structure to hold PCIe latency 1 field information.
 */
static struct regfield_info glbl2_rrq_pcie_lat1_field_info[] = {
    {"GLBL2_RRQ_PCIE_LAT1_RSVD", GLBL2_RRQ_PCIE_LAT1_RSVD_MASK},
    {"GLBL2_RRQ_PCIE_LAT1_OVFL", GLBL2_RRQ_PCIE_LAT1_OVFL_MASK},
    {"GLBL2_RRQ_PCIE_LAT1_AVG", GLBL2_RRQ_PCIE_LAT1_AVG_MASK},
};

/* MD:
 * glbl2_rrq_aximm_lat0_field_info - Structure to hold AXI-MM latency 0 field information.
 */
static struct regfield_info glbl2_rrq_aximm_lat0_field_info[] = {
    {"GLBL2_RRQ_AXIMM_LAT0_MAX", GLBL2_RRQ_AXIMM_LAT0_MAX_MASK},
    {"GLBL2_RRQ_AXIMM_LAT0_MIN", GLBL2_RRQ_AXIMM_LAT0_MIN_MASK},
};

/* MD:
 * glbl2_rrq_aximm_lat1_field_info - Structure to hold AXI-MM latency 1 field information.
 */
static struct regfield_info glbl2_rrq_aximm_lat1_field_info[] = {
    {"GLBL2_RRQ_AXIMM_LAT1_RSVD", GLBL2_RRQ_AXIMM_LAT1_RSVD_MASK},
    {"GLBL2_RRQ_AXIMM_LAT1_OVFL", GLBL2_RRQ_AXIMM_LAT1_OVFL_MASK},
    {"GLBL2_RRQ_AXIMM_LAT1_AVG", GLBL2_RRQ_AXIMM_LAT1_AVG_MASK},
};

/* MD:
 * glbl2_dbg_pcie_rq0_field_info - Structure to hold PCIe request queue 0 debug field information.
 */
static struct regfield_info glbl2_dbg_pcie_rq0_field_info[] = {
    {"GLBL2_PCIE_RQ0_NPH_AVL", GLBL2_PCIE_RQ0_NPH_AVL_MASK},
    {"GLBL2_PCIE_RQ0_RCB_AVL", GLBL2_PCIE_RQ0_RCB_AVL_MASK},
    {"GLBL2_PCIE_RQ0_SLV_RD_CREDS", GLBL2_PCIE_RQ0_SLV_RD_CREDS_MASK},
    {"GLBL2_PCIE_RQ0_TAG_EP", GLBL2_PCIE_RQ0_TAG_EP_MASK},
};

/* MD:
 * glbl2_dbg_pcie_rq1_field_info - Structure to hold PCIe request queue 1 debug field information.
 */
static struct regfield_info glbl2_dbg_pcie_rq1_field_info[] = {
    {"GLBL2_PCIE_RQ1_RSVD_1", GLBL2_PCIE_RQ1_RSVD_1_MASK},
    {"GLBL2_PCIE_RQ1_TAG_FL", GLBL2_PCIE_RQ1_TAG_FL_MASK},
    {"GLBL2_PCIE_RQ1_WTLP_HEADER_FIFO_FL", GLBL2_PCIE_RQ1_WTLP_HEADER_FIFO_FL_MASK},
    {"GLBL2_PCIE_RQ1_WTLP_HEADER_FIFO_EP", GLBL2_PCIE_RQ1_WTLP_HEADER_FIFO_EP_MASK},
    {"GLBL2_PCIE_RQ1_RQ_FIFO_EP", GLBL2_PCIE_RQ1_RQ_FIFO_EP_MASK},
    {"GLBL2_PCIE_RQ1_RQ_FIFO_FL", GLBL2_PCIE_RQ1_RQ_FIFO_FL_MASK},
    {"GLBL2_PCIE_RQ1_TLPSM", GLBL2_PCIE_RQ1_TLPSM_MASK},
    {"GLBL2_PCIE_RQ1_TLPSM512", GLBL2_PCIE_RQ1_TLPSM512_MASK},
    {"GLBL2_PCIE_RQ1_RREQ_RCB_OK", GLBL2_PCIE_RQ1_RREQ_RCB_OK_MASK},
    {"GLBL2_PCIE_RQ1_RREQ0_SLV", GLBL2_PCIE_RQ1_RREQ0_SLV_MASK},
    {"GLBL2_PCIE_RQ1_RREQ0_VLD", GLBL2_PCIE_RQ1_RREQ0_VLD_MASK},
    {"GLBL2_PCIE_RQ1_RREQ0_RDY", GLBL2_PCIE_RQ1_RREQ0_RDY_MASK},
    {"GLBL2_PCIE_RQ1_RREQ1_SLV", GLBL2_PCIE_RQ1_RREQ1_SLV_MASK},
    {"GLBL2_PCIE_RQ1_RREQ1_VLD", GLBL2_PCIE_RQ1_RREQ1_VLD_MASK},
    {"GLBL2_PCIE_RQ1_RREQ1_RDY", GLBL2_PCIE_RQ1_RREQ1_RDY_MASK},
    {"GLBL2_PCIE_RQ1_WTLP_REQ", GLBL2_PCIE_RQ1_WTLP_REQ_MASK},
    {"GLBL2_PCIE_RQ1_WTLP_STRADDLE", GLBL2_PCIE_RQ1_WTLP_STRADDLE_MASK},
};

/* MD:
 * glbl2_dbg_aximm_wr0_field_info - Structure to hold AXI-MM write 0 debug field information.
 */
static struct regfield_info glbl2_dbg_aximm_wr0_field_info[] = {
    {"GLBL2_AXIMM_WR0_RSVD_1", GLBL2_AXIMM_WR0_RSVD_1_MASK},
    {"GLBL2_AXIMM_WR0_WR_REQ", GLBL2_AXIMM_WR0_WR_REQ_MASK},
    {"GLBL2_AXIMM_WR0_WR_CHN", GLBL2_AXIMM_WR0_WR_CHN_MASK},
    {"GLBL2_AXIMM_WR0_WTLP_DATA_FIFO_EP", GLBL2_AXIMM_WR0_WTLP_DATA_FIFO_EP_MASK},
    {"GLBL2_AXIMM_WR0_WPL_FIFO_EP", GLBL2_AXIMM_WR0_WPL_FIFO_EP_MASK},
    {"GLBL2_AXIMM_WR0_BRSP_CLAIM_CHN", GLBL2_AXIMM_WR0_BRSP_CLAIM_CHN_MASK},
    {"GLBL2_AXIMM_WR0_WRREQ_CNT", GLBL2_AXIMM_WR0_WRREQ_CNT_MASK},
    {"GLBL2_AXIMM_WR0_BID", GLBL2_AXIMM_WR0_BID_MASK},
    {"GLBL2_AXIMM_WR0_BVALID", GLBL2_AXIMM_WR0_BVALID_MASK},
    {"GLBL2_AXIMM_WR0_BREADY", GLBL2_AXIMM_WR0_BREADY_MASK},
    {"GLBL2_AXIMM_WR0_WVALID", GLBL2_AXIMM_WR0_WVALID_MASK},
    {"GLBL2_AXIMM_WR0_WREADY", GLBL2_AXIMM_WR0_WREADY_MASK},
    {"GLBL2_AXIMM_WR0_AWID", GLBL2_AXIMM_WR0_AWID_MASK},
    {"GLBL2_AXIMM_WR0_AWVALID", GLBL2_AXIMM_WR0_AWVALID_MASK},
    {"GLBL2_AXIMM_WR0_AWREADY", GLBL2_AXIMM_WR0_AWREADY_MASK},
};

/* MD:
 * glbl2_dbg_aximm_wr1_field_info - Structure to hold AXI-MM write 1 debug field information.
 */
static struct regfield_info glbl2_dbg_aximm_wr1_field_info[] = {
    {"GLBL2_AXIMM_WR1_RSVD_1", GLBL2_AXIMM_WR1_RSVD_1_MASK},
    {"GLBL2_AXIMM_WR1_BRSP_CNT4", GLBL2_AXIMM_WR1_BRSP_CNT4_MASK},
    {"GLBL2_AXIMM_WR1_BRSP_CNT3", GLBL2_AXIMM_WR1_BRSP_CNT3_MASK},
    {"GLBL2_AXIMM_WR1_BRSP_CNT2", GLBL2_AXIMM_WR1_BRSP_CNT2_MASK},
    {"GLBL2_AXIMM_WR1_BRSP_CNT1", GLBL2_AXIMM_WR1_BRSP_CNT1_MASK},
    {"GLBL2_AXIMM_WR1_BRSP_CNT0", GLBL2_AXIMM_WR1_BRSP_CNT0_MASK},
};

#include <linux/kernel.h>  // MD: For printk

/* MD:
 * glbl2_dbg_aximm_rd0_field_info - Structure to hold AXI MM Read 0 field information.
 */
static struct regfield_info glbl2_dbg_aximm_rd0_field_info[] = {
    {"GLBL2_AXIMM_RD0_RSVD_1", GLBL2_AXIMM_RD0_RSVD_1_MASK},
    {"GLBL2_AXIMM_RD0_PND_CNT", GLBL2_AXIMM_RD0_PND_CNT_MASK},
    {"GLBL2_AXIMM_RD0_RD_REQ", GLBL2_AXIMM_RD0_RD_REQ_MASK},
    {"GLBL2_AXIMM_RD0_RD_CHNL", GLBL2_AXIMM_RD0_RD_CHNL_MASK},
    {"GLBL2_AXIMM_RD0_RRSP_CLAIM_CHNL", GLBL2_AXIMM_RD0_RRSP_CLAIM_CHNL_MASK},
    {"GLBL2_AXIMM_RD0_RID", GLBL2_AXIMM_RD0_RID_MASK},
    {"GLBL2_AXIMM_RD0_RVALID", GLBL2_AXIMM_RD0_RVALID_MASK},
    {"GLBL2_AXIMM_RD0_RREADY", GLBL2_AXIMM_RD0_RREADY_MASK},
    {"GLBL2_AXIMM_RD0_ARID", GLBL2_AXIMM_RD0_ARID_MASK},
    {"GLBL2_AXIMM_RD0_ARVALID", GLBL2_AXIMM_RD0_ARVALID_MASK},
    {"GLBL2_AXIMM_RD0_ARREADY", GLBL2_AXIMM_RD0_ARREADY_MASK},
};

/* MD:
 * glbl2_dbg_aximm_rd1_field_info - Structure to hold AXI MM Read 1 field information.
 */
static struct regfield_info glbl2_dbg_aximm_rd1_field_info[] = {
    {"GLBL2_AXIMM_RD1_RSVD_1", GLBL2_AXIMM_RD1_RSVD_1_MASK},
    {"GLBL2_AXIMM_RD1_RRSP_CNT4", GLBL2_AXIMM_RD1_RRSP_CNT4_MASK},
    {"GLBL2_AXIMM_RD1_RRSP_CNT3", GLBL2_AXIMM_RD1_RRSP_CNT3_MASK},
    {"GLBL2_AXIMM_RD1_RRSP_CNT2", GLBL2_AXIMM_RD1_RRSP_CNT2_MASK},
    {"GLBL2_AXIMM_RD1_RRSP_CNT1", GLBL2_AXIMM_RD1_RRSP_CNT1_MASK},
    {"GLBL2_AXIMM_RD1_RRSP_CNT0", GLBL2_AXIMM_RD1_RRSP_CNT0_MASK},
};

/* MD:
 * glbl2_dbg_fab0_field_info - Structure to hold Fabric 0 field information.
 */
static struct regfield_info glbl2_dbg_fab0_field_info[] = {
    {"GLBL2_FAB0_H2C_INB_CONV_IN_VLD", GLBL2_FAB0_H2C_INB_CONV_IN_VLD_MASK},
    {"GLBL2_FAB0_H2C_INB_CONV_IN_RDY", GLBL2_FAB0_H2C_INB_CONV_IN_RDY_MASK},
    {"GLBL2_FAB0_H2C_SEG_IN_VLD", GLBL2_FAB0_H2C_SEG_IN_VLD_MASK},
    {"GLBL2_FAB0_H2C_SEG_IN_RDY", GLBL2_FAB0_H2C_SEG_IN_RDY_MASK},
    {"GLBL2_FAB0_H2C_SEG_OUT_VLD", GLBL2_FAB0_H2C_SEG_OUT_VLD_MASK},
    {"GLBL2_FAB0_H2C_SEG_OUT_RDY", GLBL2_FAB0_H2C_SEG_OUT_RDY_MASK},
    {"GLBL2_FAB0_H2C_MST_CRDT_STAT", GLBL2_FAB0_H2C_MST_CRDT_STAT_MASK},
    {"GLBL2_FAB0_C2H_SLV_AFIFO_FULL", GLBL2_FAB0_C2H_SLV_AFIFO_FULL_MASK},
    {"GLBL2_FAB0_C2H_SLV_AFIFO_EMPTY", GLBL2_FAB0_C2H_SLV_AFIFO_EMPTY_MASK},
    {"GLBL2_FAB0_C2H_DESEG_SEG_VLD", GLBL2_FAB0_C2H_DESEG_SEG_VLD_MASK},
    {"GLBL2_FAB0_C2H_DESEG_SEG_RDY", GLBL2_FAB0_C2H_DESEG_SEG_RDY_MASK},
    {"GLBL2_FAB0_C2H_DESEG_OUT_VLD", GLBL2_FAB0_C2H_DESEG_OUT_VLD_MASK},
    {"GLBL2_FAB0_C2H_DESEG_OUT_RDY", GLBL2_FAB0_C2H_DESEG_OUT_RDY_MASK},
    {"GLBL2_FAB0_C2H_INB_DECONV_OUT_VLD", GLBL2_FAB0_C2H_INB_DECONV_OUT_VLD_MASK},
    {"GLBL2_FAB0_C2H_INB_DECONV_OUT_RDY", GLBL2_FAB0_C2H_INB_DECONV_OUT_RDY_MASK},
    {"GLBL2_FAB0_C2H_DSC_CRDT_AFIFO_FULL", GLBL2_FAB0_C2H_DSC_CRDT_AFIFO_FULL_MASK},
    {"GLBL2_FAB0_C2H_DSC_CRDT_AFIFO_EMPTY", GLBL2_FAB0_C2H_DSC_CRDT_AFIFO_EMPTY_MASK},
    {"GLBL2_FAB0_IRQ_IN_AFIFO_FULL", GLBL2_FAB0_IRQ_IN_AFIFO_FULL_MASK},
    {"GLBL2_FAB0_IRQ_IN_AFIFO_EMPTY", GLBL2_FAB0_IRQ_IN_AFIFO_EMPTY_MASK},
    {"GLBL2_FAB0_IMM_CRD_AFIFO_EMPTY", GLBL2_FAB0_IMM_CRD_AFIFO_EMPTY_MASK},
};

/* MD:
 * glbl2_dbg_fab1_field_info - Structure to hold Fabric 1 field information.
 */
static struct regfield_info glbl2_dbg_fab1_field_info[] = {
    {"GLBL2_FAB1_BYP_OUT_CRDT_STAT", GLBL2_FAB1_BYP_OUT_CRDT_STAT_MASK},
    {"GLBL2_FAB1_TM_DSC_STS_CRDT_STAT", GLBL2_FAB1_TM_DSC_STS_CRDT_STAT_MASK},
    {"GLBL2_FAB1_C2H_CMN_AFIFO_FULL", GLBL2_FAB1_C2H_CMN_AFIFO_FULL_MASK},
    {"GLBL2_FAB1_C2H_CMN_AFIFO_EMPTY", GLBL2_FAB1_C2H_CMN_AFIFO_EMPTY_MASK},
    {"GLBL2_FAB1_RSVD_1", GLBL2_FAB1_RSVD_1_MASK},
    {"GLBL2_FAB1_C2H_BYP_IN_AFIFO_FULL", GLBL2_FAB1_C2H_BYP_IN_AFIFO_FULL_MASK},
    {"GLBL2_FAB1_RSVD_2", GLBL2_FAB1_RSVD_2_MASK},
    {"GLBL2_FAB1_C2H_BYP_IN_AFIFO_EMPTY", GLBL2_FAB1_C2H_BYP_IN_AFIFO_EMPTY_MASK},
    {"GLBL2_FAB1_RSVD_3", GLBL2_FAB1_RSVD_3_MASK},
    {"GLBL2_FAB1_H2C_BYP_IN_AFIFO_FULL", GLBL2_FAB1_H2C_BYP_IN_AFIFO_FULL_MASK},
    {"GLBL2_FAB1_RSVD_4", GLBL2_FAB1_RSVD_4_MASK},
    {"GLBL2_FAB1_H2C_BYP_IN_AFIFO_EMPTY", GLBL2_FAB1_H2C_BYP_IN_AFIFO_EMPTY_MASK},
};

/* MD:
 * glbl2_dbg_match_sel_field_info - Structure to hold Match Select field information.
 */
static struct regfield_info glbl2_dbg_match_sel_field_info[] = {
    {"GLBL2_MATCH_SEL_RSV", GLBL2_MATCH_SEL_RSV_MASK},
    {"GLBL2_MATCH_SEL_CSR_SEL", GLBL2_MATCH_SEL_CSR_SEL_MASK},
    {"GLBL2_MATCH_SEL_CSR_EN", GLBL2_MATCH_SEL_CSR_EN_MASK},
    {"GLBL2_MATCH_SEL_ROTATE1", GLBL2_MATCH_SEL_ROTATE1_MASK},
    {"GLBL2_MATCH_SEL_ROTATE0", GLBL2_MATCH_SEL_ROTATE0_MASK},
    {"GLBL2_MATCH_SEL_SEL", GLBL2_MATCH_SEL_SEL_MASK},
};

/* MD:
 * glbl2_dbg_match_msk_field_info - Structure to hold Match Mask field information.
 */
static struct regfield_info glbl2_dbg_match_msk_field_info[] = {
    {"GLBL2_MATCH_MSK", GLBL2_MATCH_MSK_MASK},
};

/* MD:
 * glbl2_dbg_match_pat_field_info - Structure to hold Match Pattern field information.
 */
static struct regfield_info glbl2_dbg_match_pat_field_info[] = {
    {"GLBL2_MATCH_PAT_PATTERN", GLBL2_MATCH_PAT_PATTERN_MASK},
};

/* MD:
 * glbl_rng_sz_1_field_info - Structure to hold Ring Size 1 field information.
 */
static struct regfield_info glbl_rng_sz_1_field_info[] = {
    {"GLBL_RNG_SZ_1_RSVD_1", GLBL_RNG_SZ_1_RSVD_1_MASK},
    {"GLBL_RNG_SZ_1_RING_SIZE", GLBL_RNG_SZ_1_RING_SIZE_MASK},
};

/* MD:
 * glbl_rng_sz_2_field_info - Structure to hold Ring Size 2 field information.
 */
static struct regfield_info glbl_rng_sz_2_field_info[] = {
    {"GLBL_RNG_SZ_2_RSVD_1", GLBL_RNG_SZ_2_RSVD_1_MASK},
    {"GLBL_RNG_SZ_2_RING_SIZE", GLBL_RNG_SZ_2_RING_SIZE_MASK},
};

/* MD:
 * glbl_rng_sz_3_field_info - Structure to hold Ring Size 3 field information.
 */
static struct regfield_info glbl_rng_sz_3_field_info[] = {
    {"GLBL_RNG_SZ_3_RSVD_1", GLBL_RNG_SZ_3_RSVD_1_MASK},
    {"GLBL_RNG_SZ_3_RING_SIZE", GLBL_RNG_SZ_3_RING_SIZE_MASK},
};

/* MD:
 * glbl_rng_sz_4_field_info - Structure to hold Ring Size 4 field information.
 */
static struct regfield_info glbl_rng_sz_4_field_info[] = {
    {"GLBL_RNG_SZ_4_RSVD_1", GLBL_RNG_SZ_4_RSVD_1_MASK},
    {"GLBL_RNG_SZ_4_RING_SIZE", GLBL_RNG_SZ_4_RING_SIZE_MASK},
};

/* MD:
 * glbl_rng_sz_5_field_info - Structure to hold Ring Size 5 field information.
 */
static struct regfield_info glbl_rng_sz_5_field_info[] = {
    {"GLBL_RNG_SZ_5_RSVD_1", GLBL_RNG_SZ_5_RSVD_1_MASK},
    {"GLBL_RNG_SZ_5_RING_SIZE", GLBL_RNG_SZ_5_RING_SIZE_MASK},
};

/* MD:
 * glbl_rng_sz_6_field_info - Structure to hold Ring Size 6 field information.
 */
static struct regfield_info glbl_rng_sz_6_field_info[] = {
    {"GLBL_RNG_SZ_6_RSVD_1", GLBL_RNG_SZ_6_RSVD_1_MASK},
    {"GLBL_RNG_SZ_6_RING_SIZE", GLBL_RNG_SZ_6_RING_SIZE_MASK},
};

/* MD:
 * glbl_rng_sz_7_field_info - Structure to hold Ring Size 7 field information.
 */
static struct regfield_info glbl_rng_sz_7_field_info[] = {
    {"GLBL_RNG_SZ_7_RSVD_1", GLBL_RNG_SZ_7_RSVD_1_MASK},
    {"GLBL_RNG_SZ_7_RING_SIZE", GLBL_RNG_SZ_7_RING_SIZE_MASK},
};

/* MD:
 * glbl_rng_sz_8_field_info - Structure to hold Ring Size 8 field information.
 */
static struct regfield_info glbl_rng_sz_8_field_info[] = {
    {"GLBL_RNG_SZ_8_RSVD_1", GLBL_RNG_SZ_8_RSVD_1_MASK},
    {"GLBL_RNG_SZ_8_RING_SIZE", GLBL_RNG_SZ_8_RING_SIZE_MASK},
};

/* MD:
 * glbl_rng_sz_9_field_info - Structure to hold Ring Size 9 field information.
 */
static struct regfield_info glbl_rng_sz_9_field_info[] = {
    {"GLBL_RNG_SZ_9_RSVD_1", GLBL_RNG_SZ_9_RSVD_1_MASK},
    {"GLBL_RNG_SZ_9_RING_SIZE", GLBL_RNG_SZ_9_RING_SIZE_MASK},
};

/* MD:
 * glbl_rng_sz_a_field_info - Structure to hold Ring Size A field information.
 */
static struct regfield_info glbl_rng_sz_a_field_info[] = {
    {"GLBL_RNG_SZ_A_RSVD_1", GLBL_RNG_SZ_A_RSVD_1_MASK},
    {"GLBL_RNG_SZ_A_RING_SIZE", GLBL_RNG_SZ_A_RING_SIZE_MASK},
};

/* MD:
 * glbl_rng_sz_b_field_info - Structure to hold global ring size B field information.
 */
static struct regfield_info glbl_rng_sz_b_field_info[] = {
    {"GLBL_RNG_SZ_B_RSVD_1", GLBL_RNG_SZ_B_RSVD_1_MASK},
    {"GLBL_RNG_SZ_B_RING_SIZE", GLBL_RNG_SZ_B_RING_SIZE_MASK},
};

/* MD:
 * glbl_rng_sz_c_field_info - Structure to hold global ring size C field information.
 */
static struct regfield_info glbl_rng_sz_c_field_info[] = {
    {"GLBL_RNG_SZ_C_RSVD_1", GLBL_RNG_SZ_C_RSVD_1_MASK},
    {"GLBL_RNG_SZ_C_RING_SIZE", GLBL_RNG_SZ_C_RING_SIZE_MASK},
};

/* MD:
 * glbl_rng_sz_d_field_info - Structure to hold global ring size D field information.
 */
static struct regfield_info glbl_rng_sz_d_field_info[] = {
    {"GLBL_RNG_SZ_D_RSVD_1", GLBL_RNG_SZ_D_RSVD_1_MASK},
    {"GLBL_RNG_SZ_D_RING_SIZE", GLBL_RNG_SZ_D_RING_SIZE_MASK},
};

/* MD:
 * glbl_rng_sz_e_field_info - Structure to hold global ring size E field information.
 */
static struct regfield_info glbl_rng_sz_e_field_info[] = {
    {"GLBL_RNG_SZ_E_RSVD_1", GLBL_RNG_SZ_E_RSVD_1_MASK},
    {"GLBL_RNG_SZ_E_RING_SIZE", GLBL_RNG_SZ_E_RING_SIZE_MASK},
};

/* MD:
 * glbl_rng_sz_f_field_info - Structure to hold global ring size F field information.
 */
static struct regfield_info glbl_rng_sz_f_field_info[] = {
    {"GLBL_RNG_SZ_F_RSVD_1", GLBL_RNG_SZ_F_RSVD_1_MASK},
    {"GLBL_RNG_SZ_F_RING_SIZE", GLBL_RNG_SZ_F_RING_SIZE_MASK},
};

/* MD:
 * glbl_rng_sz_10_field_info - Structure to hold global ring size 10 field information.
 */
static struct regfield_info glbl_rng_sz_10_field_info[] = {
    {"GLBL_RNG_SZ_10_RSVD_1", GLBL_RNG_SZ_10_RSVD_1_MASK},
    {"GLBL_RNG_SZ_10_RING_SIZE", GLBL_RNG_SZ_10_RING_SIZE_MASK},
};

/* MD:
 * glbl_err_stat_field_info - Structure to hold global error status field information.
 */
static struct regfield_info glbl_err_stat_field_info[] = {
    {"GLBL_ERR_STAT_RSVD_1", GLBL_ERR_STAT_RSVD_1_MASK},
    {"GLBL_ERR_STAT_ERR_FAB", GLBL_ERR_STAT_ERR_FAB_MASK},
    {"GLBL_ERR_STAT_ERR_H2C_ST", GLBL_ERR_STAT_ERR_H2C_ST_MASK},
    {"GLBL_ERR_STAT_ERR_BDG", GLBL_ERR_STAT_ERR_BDG_MASK},
    {"GLBL_ERR_STAT_IND_CTXT_CMD_ERR", GLBL_ERR_STAT_IND_CTXT_CMD_ERR_MASK},
    {"GLBL_ERR_STAT_ERR_C2H_ST", GLBL_ERR_STAT_ERR_C2H_ST_MASK},
    {"GLBL_ERR_STAT_ERR_C2H_MM_1", GLBL_ERR_STAT_ERR_C2H_MM_1_MASK},
    {"GLBL_ERR_STAT_ERR_C2H_MM_0", GLBL_ERR_STAT_ERR_C2H_MM_0_MASK},
    {"GLBL_ERR_STAT_ERR_H2C_MM_1", GLBL_ERR_STAT_ERR_H2C_MM_1_MASK},
    {"GLBL_ERR_STAT_ERR_H2C_MM_0", GLBL_ERR_STAT_ERR_H2C_MM_0_MASK},
    {"GLBL_ERR_STAT_ERR_TRQ", GLBL_ERR_STAT_ERR_TRQ_MASK},
    {"GLBL_ERR_STAT_ERR_DSC", GLBL_ERR_STAT_ERR_DSC_MASK},
    {"GLBL_ERR_STAT_ERR_RAM_DBE", GLBL_ERR_STAT_ERR_RAM_DBE_MASK},
    {"GLBL_ERR_STAT_ERR_RAM_SBE", GLBL_ERR_STAT_ERR_RAM_SBE_MASK},
};

/* MD:
 * glbl_err_mask_field_info - Structure to hold global error mask field information.
 */
static struct regfield_info glbl_err_mask_field_info[] = {
    {"GLBL_ERR", GLBL_ERR_MASK},
};

/* MD:
 * glbl_dsc_cfg_field_info - Structure to hold global descriptor configuration field information.
 */
static struct regfield_info glbl_dsc_cfg_field_info[] = {
    {"GLBL_DSC_CFG_RSVD_1", GLBL_DSC_CFG_RSVD_1_MASK},
    {"GLBL_DSC_CFG_C2H_UODSC_LIMIT", GLBL_DSC_CFG_C2H_UODSC_LIMIT_MASK},
    {"GLBL_DSC_CFG_H2C_UODSC_LIMIT", GLBL_DSC_CFG_H2C_UODSC_LIMIT_MASK},
    {"GLBL_DSC_CFG_UNC_OVR_COR", GLBL_DSC_CFG_UNC_OVR_COR_MASK},
    {"GLBL_DSC_CFG_CTXT_FER_DIS", GLBL_DSC_CFG_CTXT_FER_DIS_MASK},
    {"GLBL_DSC_CFG_RSVD_2", GLBL_DSC_CFG_RSVD_2_MASK},
    {"GLBL_DSC_CFG_MAXFETCH", GLBL_DSC_CFG_MAXFETCH_MASK},
    {"GLBL_DSC_CFG_WB_ACC_INT", GLBL_DSC_CFG_WB_ACC_INT_MASK},
};

/* MD:
 * glbl_dsc_err_sts_field_info - Structure to hold global descriptor error status field information.
 */
static struct regfield_info glbl_dsc_err_sts_field_info[] = {
    {"GLBL_DSC_ERR_STS_RSVD_1", GLBL_DSC_ERR_STS_RSVD_1_MASK},
    {"GLBL_DSC_ERR_STS_PORT_ID", GLBL_DSC_ERR_STS_PORT_ID_MASK},
    {"GLBL_DSC_ERR_STS_SBE", GLBL_DSC_ERR_STS_SBE_MASK},
    {"GLBL_DSC_ERR_STS_DBE", GLBL_DSC_ERR_STS_DBE_MASK},
    {"GLBL_DSC_ERR_STS_RQ_CANCEL", GLBL_DSC_ERR_STS_RQ_CANCEL_MASK},
    {"GLBL_DSC_ERR_STS_DSC", GLBL_DSC_ERR_STS_DSC_MASK},
    {"GLBL_DSC_ERR_STS_DMA", GLBL_DSC_ERR_STS_DMA_MASK},
    {"GLBL_DSC_ERR_STS_FLR_CANCEL", GLBL_DSC_ERR_STS_FLR_CANCEL_MASK},
    {"GLBL_DSC_ERR_STS_RSVD_2", GLBL_DSC_ERR_STS_RSVD_2_MASK},
    {"GLBL_DSC_ERR_STS_DAT_POISON", GLBL_DSC_ERR_STS_DAT_POISON_MASK},
    {"GLBL_DSC_ERR_STS_TIMEOUT", GLBL_DSC_ERR_STS_TIMEOUT_MASK},
    {"GLBL_DSC_ERR_STS_FLR", GLBL_DSC_ERR_STS_FLR_MASK},
    {"GLBL_DSC_ERR_STS_TAG", GLBL_DSC_ERR_STS_TAG_MASK},
    {"GLBL_DSC_ERR_STS_ADDR", GLBL_DSC_ERR_STS_ADDR_MASK},
    {"GLBL_DSC_ERR_STS_PARAM", GLBL_DSC_ERR_STS_PARAM_MASK},
    {"GLBL_DSC_ERR_STS_BCNT", GLBL_DSC_ERR_STS_BCNT_MASK},
    {"GLBL_DSC_ERR_STS_UR_CA", GLBL_DSC_ERR_STS_UR_CA_MASK},
    {"GLBL_DSC_ERR_STS_POISON", GLBL_DSC_ERR_STS_POISON_MASK},
};

/* MD:
 * glbl_dsc_err_msk_field_info - Structure to hold global descriptor error mask field information.
 */
static struct regfield_info glbl_dsc_err_msk_field_info[] = {
    {"GLBL_DSC_ERR_MSK", GLBL_DSC_ERR_MSK_MASK},
};

/* MD:
 * glbl_dsc_err_log0_field_info - Structure to hold global descriptor error log 0 field information.
 */
static struct regfield_info glbl_dsc_err_log0_field_info[] = {
    {"GLBL_DSC_ERR_LOG0_VALID", GLBL_DSC_ERR_LOG0_VALID_MASK},
    {"GLBL_DSC_ERR_LOG0_SEL", GLBL_DSC_ERR_LOG0_SEL_MASK},
    {"GLBL_DSC_ERR_LOG0_RSVD_1", GLBL_DSC_ERR_LOG0_RSVD_1_MASK},
    {"GLBL_DSC_ERR_LOG0_QID", GLBL_DSC_ERR_LOG0_QID_MASK},
};

/* MD:
 * glbl_dsc_err_log1_field_info - Structure to hold global descriptor error log 1 field information.
 */
static struct regfield_info glbl_dsc_err_log1_field_info[] = {
    {"GLBL_DSC_ERR_LOG1_RSVD_1", GLBL_DSC_ERR_LOG1_RSVD_1_MASK},
    {"GLBL_DSC_ERR_LOG1_CIDX", GLBL_DSC_ERR_LOG1_CIDX_MASK},
    {"GLBL_DSC_ERR_LOG1_RSVD_2", GLBL_DSC_ERR_LOG1_RSVD_2_MASK},
    {"GLBL_DSC_ERR_LOG1_SUB_TYPE", GLBL_DSC_ERR_LOG1_SUB_TYPE_MASK},
    {"GLBL_DSC_ERR_LOG1_ERR_TYPE", GLBL_DSC_ERR_LOG1_ERR_TYPE_MASK},
};

/* MD:
 * glbl_trq_err_sts_field_info - Structure to hold global TRQ error status field information.
 */
static struct regfield_info glbl_trq_err_sts_field_info[] = {
    {"GLBL_TRQ_ERR_STS_RSVD_1", GLBL_TRQ_ERR_STS_RSVD_1_MASK},
    {"GLBL_TRQ_ERR_STS_TCP_QSPC_TIMEOUT", GLBL_TRQ_ERR_STS_TCP_QSPC_TIMEOUT_MASK},
    {"GLBL_TRQ_ERR_STS_RSVD_2", GLBL_TRQ_ERR_STS_RSVD_2_MASK},
    {"GLBL_TRQ_ERR_STS_QID_RANGE", GLBL_TRQ_ERR_STS_QID_RANGE_MASK},
    {"GLBL_TRQ_ERR_STS_QSPC_UNMAPPED", GLBL_TRQ_ERR_STS_QSPC_UNMAPPED_MASK},
    {"GLBL_TRQ_ERR_STS_TCP_CSR_TIMEOUT", GLBL_TRQ_ERR_STS_TCP_CSR_TIMEOUT_MASK},
    {"GLBL_TRQ_ERR_STS_RSVD_3", GLBL_TRQ_ERR_STS_RSVD_3_MASK},
    {"GLBL_TRQ_ERR_STS_VF_ACCESS_ERR", GLBL_TRQ_ERR_STS_VF_ACCESS_ERR_MASK},
    {"GLBL_TRQ_ERR_STS_CSR_UNMAPPED", GLBL_TRQ_ERR_STS_CSR_UNMAPPED_MASK},
};

/* MD:
 * glbl_trq_err_msk_field_info - Structure to hold global TRQ error mask field information.
 */
static struct regfield_info glbl_trq_err_msk_field_info[] = {
    {"GLBL_TRQ_ERR_MSK", GLBL_TRQ_ERR_MSK_MASK},
};

/* MD:
 * glbl_trq_err_log_field_info - Structure to hold global TRQ error log field information.
 */
static struct regfield_info glbl_trq_err_log_field_info[] = {
    {"GLBL_TRQ_ERR_LOG_SRC", GLBL_TRQ_ERR_LOG_SRC_MASK},
    {"GLBL_TRQ_ERR_LOG_TARGET", GLBL_TRQ_ERR_LOG_TARGET_MASK},
    {"GLBL_TRQ_ERR_LOG_FUNC", GLBL_TRQ_ERR_LOG_FUNC_MASK},
    {"GLBL_TRQ_ERR_LOG_ADDRESS", GLBL_TRQ_ERR_LOG_ADDRESS_MASK},
};

/* MD:
 * glbl_dsc_dbg_dat0_field_info - Structure to hold global descriptor debug data 0 field information.
 */
static struct regfield_info glbl_dsc_dbg_dat0_field_info[] = {
    {"GLBL_DSC_DAT0_RSVD_1", GLBL_DSC_DAT0_RSVD_1_MASK},
    {"GLBL_DSC_DAT0_CTXT_ARB_DIR", GLBL_DSC_DAT0_CTXT_ARB_DIR_MASK},
    {"GLBL_DSC_DAT0_CTXT_ARB_QID", GLBL_DSC_DAT0_CTXT_ARB_QID_MASK},
    {"GLBL_DSC_DAT0_CTXT_ARB_REQ", GLBL_DSC_DAT0_CTXT_ARB_REQ_MASK},
    {"GLBL_DSC_DAT0_IRQ_FIFO_FL", GLBL_DSC_DAT0_IRQ_FIFO_FL_MASK},
    {"GLBL_DSC_DAT0_TMSTALL", GLBL_DSC_DAT0_TMSTALL_MASK},
    {"GLBL_DSC_DAT0_RRQ_STALL", GLBL_DSC_DAT0_RRQ_STALL_MASK},
    {"GLBL_DSC_DAT0_RCP_FIFO_SPC_STALL", GLBL_DSC_DAT0_RCP_FIFO_SPC_STALL_MASK},
    {"GLBL_DSC_DAT0_RRQ_FIFO_SPC_STALL", GLBL_DSC_DAT0_RRQ_FIFO_SPC_STALL_MASK},
    {"GLBL_DSC_DAT0_FAB_MRKR_RSP_STALL", GLBL_DSC_DAT0_FAB_MRKR_RSP_STALL_MASK},
    {"GLBL_DSC_DAT0_DSC_OUT_STALL", GLBL_DSC_DAT0_DSC_OUT_STALL_MASK},
};

#include <linux/kernel.h>  // MD: For printk

/* MD:
 * Define register field information for various global descriptor debug data.
 */

/* MD: Global Descriptor Debug Data 1 Field Information */
static struct regfield_info glbl_dsc_dbg_dat1_field_info[] = {
    {"GLBL_DSC_DAT1_RSVD_1", GLBL_DSC_DAT1_RSVD_1_MASK},
    {"GLBL_DSC_DAT1_EVT_SPC_C2H", GLBL_DSC_DAT1_EVT_SPC_C2H_MASK},
    {"GLBL_DSC_DAT1_EVT_SP_H2C", GLBL_DSC_DAT1_EVT_SP_H2C_MASK},
    {"GLBL_DSC_DAT1_DSC_SPC_C2H", GLBL_DSC_DAT1_DSC_SPC_C2H_MASK},
    {"GLBL_DSC_DAT1_DSC_SPC_H2C", GLBL_DSC_DAT1_DSC_SPC_H2C_MASK},
};

/* MD: Global Descriptor Debug Control Field Information */
static struct regfield_info glbl_dsc_dbg_ctl_field_info[] = {
    {"GLBL_DSC_CTL_RSVD_1", GLBL_DSC_CTL_RSVD_1_MASK},
    {"GLBL_DSC_CTL_LAT_QID", GLBL_DSC_CTL_LAT_QID_MASK},
    {"GLBL_DSC_CTL_DSC_ENG_LAT_CLR", GLBL_DSC_CTL_DSC_ENG_LAT_CLR_MASK},
    {"GLBL_DSC_CTL_SELECT", GLBL_DSC_CTL_SELECT_MASK},
};

/* MD: Global Descriptor Error Log 2 Field Information */
static struct regfield_info glbl_dsc_err_log2_field_info[] = {
    {"GLBL_DSC_ERR_LOG2_OLD_PIDX", GLBL_DSC_ERR_LOG2_OLD_PIDX_MASK},
    {"GLBL_DSC_ERR_LOG2_NEW_PIDX", GLBL_DSC_ERR_LOG2_NEW_PIDX_MASK},
};

/* MD: Global Global Interrupt Configuration Field Information */
static struct regfield_info glbl_glbl_interrupt_cfg_field_info[] = {
    {"GLBL_GLBL_INTERRUPT_CFG_RSVD_1", GLBL_GLBL_INTERRUPT_CFG_RSVD_1_MASK},
    {"GLBL_GLBL_INTERRUPT_CFG_LGCY_INTR_PENDING", GLBL_GLBL_INTERRUPT_CFG_LGCY_INTR_PENDING_MASK},
    {"GLBL_GLBL_INTERRUPT_CFG_EN_LGCY_INTR", GLBL_GLBL_INTERRUPT_CFG_EN_LGCY_INTR_MASK},
};

/* MD: Global Virtual Channel Host Profile Field Information */
static struct regfield_info glbl_vch_host_profile_field_info[] = {
    {"GLBL_VCH_HOST_PROFILE_RSVD_1", GLBL_VCH_HOST_PROFILE_RSVD_1_MASK},
    {"GLBL_VCH_HOST_PROFILE_2C_MM", GLBL_VCH_HOST_PROFILE_2C_MM_MASK},
    {"GLBL_VCH_HOST_PROFILE_2C_ST", GLBL_VCH_HOST_PROFILE_2C_ST_MASK},
    {"GLBL_VCH_HOST_PROFILE_VCH_DSC", GLBL_VCH_HOST_PROFILE_VCH_DSC_MASK},
    {"GLBL_VCH_HOST_PROFILE_VCH_INT_MSG", GLBL_VCH_HOST_PROFILE_VCH_INT_MSG_MASK},
    {"GLBL_VCH_HOST_PROFILE_VCH_INT_AGGR", GLBL_VCH_HOST_PROFILE_VCH_INT_AGGR_MASK},
    {"GLBL_VCH_HOST_PROFILE_VCH_CMPT", GLBL_VCH_HOST_PROFILE_VCH_CMPT_MASK},
    {"GLBL_VCH_HOST_PROFILE_VCH_C2H_PLD", GLBL_VCH_HOST_PROFILE_VCH_C2H_PLD_MASK},
};

/* MD: Global Bridge Host Profile Field Information */
static struct regfield_info glbl_bridge_host_profile_field_info[] = {
    {"GLBL_BRIDGE_HOST_PROFILE_RSVD_1", GLBL_BRIDGE_HOST_PROFILE_RSVD_1_MASK},
    {"GLBL_BRIDGE_HOST_PROFILE_BDGID", GLBL_BRIDGE_HOST_PROFILE_BDGID_MASK},
};

/* MD: AXI MM IRQ Destination Address Field Information */
static struct regfield_info aximm_irq_dest_addr_field_info[] = {
    {"AXIMM_IRQ_DEST_ADDR_ADDR", AXIMM_IRQ_DEST_ADDR_ADDR_MASK},
};

/* MD: Fabric Error Log Field Information */
static struct regfield_info fab_err_log_field_info[] = {
    {"FAB_ERR_LOG_RSVD_1", FAB_ERR_LOG_RSVD_1_MASK},
    {"FAB_ERR_LOG_SRC", FAB_ERR_LOG_SRC_MASK},
};

/* MD: Global Request Error Status Field Information */
static struct regfield_info glbl_req_err_sts_field_info[] = {
    {"GLBL_REQ_ERR_STS_RSVD_1", GLBL_REQ_ERR_STS_RSVD_1_MASK},
    {"GLBL_REQ_ERR_STS_RC_DISCONTINUE", GLBL_REQ_ERR_STS_RC_DISCONTINUE_MASK},
    {"GLBL_REQ_ERR_STS_RC_PRTY", GLBL_REQ_ERR_STS_RC_PRTY_MASK},
    {"GLBL_REQ_ERR_STS_RC_FLR", GLBL_REQ_ERR_STS_RC_FLR_MASK},
    {"GLBL_REQ_ERR_STS_RC_TIMEOUT", GLBL_REQ_ERR_STS_RC_TIMEOUT_MASK},
    {"GLBL_REQ_ERR_STS_RC_INV_BCNT", GLBL_REQ_ERR_STS_RC_INV_BCNT_MASK},
    {"GLBL_REQ_ERR_STS_RC_INV_TAG", GLBL_REQ_ERR_STS_RC_INV_TAG_MASK},
    {"GLBL_REQ_ERR_STS_RC_START_ADDR_MISMCH", GLBL_REQ_ERR_STS_RC_START_ADDR_MISMCH_MASK},
    {"GLBL_REQ_ERR_STS_RC_RID_TC_ATTR_MISMCH", GLBL_REQ_ERR_STS_RC_RID_TC_ATTR_MISMCH_MASK},
    {"GLBL_REQ_ERR_STS_RC_NO_DATA", GLBL_REQ_ERR_STS_RC_NO_DATA_MASK},
    {"GLBL_REQ_ERR_STS_RC_UR_CA_CRS", GLBL_REQ_ERR_STS_RC_UR_CA_CRS_MASK},
    {"GLBL_REQ_ERR_STS_RC_POISONED", GLBL_REQ_ERR_STS_RC_POISONED_MASK},
};

/* MD: Global Request Error Mask Field Information */
static struct regfield_info glbl_req_err_msk_field_info[] = {
    {"GLBL_REQ_ERR_MSK", GLBL_REQ_ERR_MSK_MASK},
};

/* MD: Global Descriptor Debug Latency 0 A Field Information */
static struct regfield_info glbl_dsc_dbg_lat0_a_field_info[] = {
    {"GLBL_DSC_LAT0_A_LAT_MAX", GLBL_DSC_LAT0_A_LAT_MAX_MASK},
    {"GLBL_DSC_LAT0_A_LAT_MIN", GLBL_DSC_LAT0_A_LAT_MIN_MASK},
};

/* MD: Global Descriptor Debug Latency 1 A Field Information */
static struct regfield_info glbl_dsc_dbg_lat1_a_field_info[] = {
    {"GLBL_DSC_LAT1_A_RSVD", GLBL_DSC_LAT1_A_RSVD_MASK},
    {"GLBL_DSC_LAT1_A_LAT_OVF", GLBL_DSC_LAT1_A_LAT_OVF_MASK},
    {"GLBL_DSC_LAT1_A_LAT_AVG", GLBL_DSC_LAT1_A_LAT_AVG_MASK},
};

/* MD: Global Descriptor Credit Counter 0 A Field Information */
static struct regfield_info glbl_dsc_crd_ctr0_a_field_info[] = {
    {"GLBL_DSC_CRD_CTR0_A_CRD_RCV_CNT", GLBL_DSC_CRD_CTR0_A_CRD_RCV_CNT_MASK},
};

/* MD: Global Descriptor Credit Counter 1 A Field Information */
static struct regfield_info glbl_dsc_crd_ctr1_a_field_info[] = {
    {"GLBL_DSC_CRD_CTR1_A_CRD_RCV_CNT", GLBL_DSC_CRD_CTR1_A_CRD_RCV_CNT_MASK},
};

/* MD: Global Descriptor Credit Counter 2 A Field Information */
static struct regfield_info glbl_dsc_crd_ctr2_a_field_info[] = {
    {"GLBL_DSC_CRD_CTR2_A_CRD_RCV_NRDY_CNT", GLBL_DSC_CRD_CTR2_A_CRD_RCV_NRDY_CNT_MASK},
};

/* MD: Global Descriptor Credit Counter 3 A Field Information */
static struct regfield_info glbl_dsc_crd_ctr3_a_field_info[] = {
    {"GLBL_DSC_CRD_CTR3_A_CRD_RCV_NRDY_CNT", GLBL_DSC_CRD_CTR3_A_CRD_RCV_NRDY_CNT_MASK},
};

/* MD: Global Descriptor Immediate Credit Counter 0 A Field Information */
static struct regfield_info glbl_dsc_imm_crd_ctr0_a_field_info[] = {
    {"GLBL_DSC_IMM_CRD_CTR0_A_RCV_CNT", GLBL_DSC_IMM_CRD_CTR0_A_RCV_CNT_MASK},
};

/* MD: Global Descriptor Immediate Credit Counter 1 A Field Information */
static struct regfield_info glbl_dsc_imm_crd_ctr1_a_field_info[] = {
    {"GLBL_DSC_IMM_CRD_CTR1_A_RCV_CNT", GLBL_DSC_IMM_CRD_CTR1_A_RCV_CNT_MASK},
};

/* MD:
 * Structure to hold register field information for immediate credit counters.
 */
static struct regfield_info glbl_dsc_imm_crd_ctr2_a_field_info[] = {
    {"GLBL_DSC_IMM_CRD_CTR2_A_RCV_NRDY_CNT",
     GLBL_DSC_IMM_CRD_CTR2_A_RCV_NRDY_CNT_MASK},
};

/* MD:
 * Structure to hold register field information for immediate credit counters.
 */
static struct regfield_info glbl_dsc_imm_crd_ctr3_a_field_info[] = {
    {"GLBL_DSC_IMM_CRD_CTR3_A_RCV_NRDY_CNT",
     GLBL_DSC_IMM_CRD_CTR3_A_RCV_NRDY_CNT_MASK},
};

/* MD:
 * Structure to hold register field information for H2C output counters.
 */
static struct regfield_info glbl_dsc_h2c_out_ctr0_a_field_info[] = {
    {"GLBL_DSC_H2C_OUT_CTR0_A_H2CVLD_CNT",
     GLBL_DSC_H2C_OUT_CTR0_A_H2CVLD_CNT_MASK},
};

/* MD:
 * Structure to hold register field information for H2C output counters.
 */
static struct regfield_info glbl_dsc_h2c_out_ctr1_a_field_info[] = {
    {"GLBL_DSC_H2C_OUT_CTR1_A_H2CVLD_CNT",
     GLBL_DSC_H2C_OUT_CTR1_A_H2CVLD_CNT_MASK},
};

/* MD:
 * Structure to hold register field information for H2C output counters.
 */
static struct regfield_info glbl_dsc_h2c_out_ctr2_a_field_info[] = {
    {"GLBL_DSC_H2C_OUT_CTR2_A_H2CVLD_NRDY_CNT",
     GLBL_DSC_H2C_OUT_CTR2_A_H2CVLD_NRDY_CNT_MASK},
};

/* MD:
 * Structure to hold register field information for H2C output counters.
 */
static struct regfield_info glbl_dsc_h2c_out_ctr3_a_field_info[] = {
    {"GLBL_DSC_H2C_OUT_CTR3_A_H2CVLD_NRDY_CNT",
     GLBL_DSC_H2C_OUT_CTR3_A_H2CVLD_NRDY_CNT_MASK},
};

/* MD:
 * Structure to hold register field information for C2H output counters.
 */
static struct regfield_info glbl_dsc_c2h_out_ctr0_a_field_info[] = {
    {"GLBL_DSC_C2H_OUT_CTR0_A_C2HVLD_CNT",
     GLBL_DSC_C2H_OUT_CTR0_A_C2HVLD_CNT_MASK},
};

/* MD:
 * Structure to hold register field information for C2H output counters.
 */
static struct regfield_info glbl_dsc_c2h_out_ctr1_a_field_info[] = {
    {"GLBL_DSC_C2H_OUT_CTR1_A_C2HVLD_CNT",
     GLBL_DSC_C2H_OUT_CTR1_A_C2HVLD_CNT_MASK},
};

/* MD:
 * Structure to hold register field information for C2H output counters.
 */
static struct regfield_info glbl_dsc_c2h_out_ctr2_a_field_info[] = {
    {"GLBL_DSC_C2H_OUT_CTR2_A_C2HVLD_NRDY_CNT",
     GLBL_DSC_C2H_OUT_CTR2_A_C2HVLD_NRDY_CNT_MASK},
};

/* MD:
 * Structure to hold register field information for C2H output counters.
 */
static struct regfield_info glbl_dsc_c2h_out_ctr3_a_field_info[] = {
    {"GLBL_DSC_C2H_OUT_CTR3_A_C2HVLD_NRDY_CNT",
     GLBL_DSC_C2H_OUT_CTR3_A_C2HVLD_NRDY_CNT_MASK},
};

/* MD:
 * Structure to hold register field information for user counters.
 */
static struct regfield_info t_field_info[] = {
    {"T_USER_CTR_MAX",
     T_USER_CTR_MAX_MASK},
};

/* MD:
 * Structure to hold register field information for performance counter control.
 */
static struct regfield_info glbl_perf_cntr_ctl_a1_field_info[] = {
    {"GLBL_PERF_CNTR_CTL_A1_RSVD",
     GLBL_PERF_CNTR_CTL_A1_RSVD_MASK},
    {"GLBL_PERF_CNTR_CTL_A1_USER_CTR_CLEAR",
     GLBL_PERF_CNTR_CTL_A1_USER_CTR_CLEAR_MASK},
    {"GLBL_PERF_CNTR_CTL_A1_USER_CTR_READ",
     GLBL_PERF_CNTR_CTL_A1_USER_CTR_READ_MASK},
    {"GLBL_PERF_CNTR_CTL_A1_USER_CTR_MAX",
     GLBL_PERF_CNTR_CTL_A1_USER_CTR_MAX_MASK},
};

/* MD:
 * Structure to hold register field information for free counters.
 */
static struct regfield_info glbl_free_cnt_a0_field_info[] = {
    {"GLBL_FREE_CNT_A0_S",
     GLBL_FREE_CNT_A0_S_MASK},
};

/* MD:
 * Structure to hold register field information for free counters.
 */
static struct regfield_info glbl_free_cnt_a1_field_info[] = {
    {"GLBL_FREE_CNT_A1_RSVD",
     GLBL_FREE_CNT_A1_RSVD_MASK},
    {"GLBL_FREE_CNT_A1_S",
     GLBL_FREE_CNT_A1_S_MASK},
};

/* MD:
 * Structure to hold register field information for AXIS H2C counters.
 */
static struct regfield_info glbl_axis_h2c_cnt_a0_field_info[] = {
    {"GLBL_AXIS_H2C_CNT_A0_MPKT_CNTS",
     GLBL_AXIS_H2C_CNT_A0_MPKT_CNTS_MASK},
};

/* MD:
 * Structure to hold register field information for AXIS H2C counters.
 */
static struct regfield_info glbl_axis_h2c_cnt_a1_field_info[] = {
    {"GLBL_AXIS_H2C_CNT_A1_MIDLE_CNTS",
     GLBL_AXIS_H2C_CNT_A1_MIDLE_CNTS_MASK},
    {"GLBL_AXIS_H2C_CNT_A1_MPKT_CNTS",
     GLBL_AXIS_H2C_CNT_A1_MPKT_CNTS_MASK},
};

/* MD:
 * Structure to hold register field information for AXIS H2C counters.
 */
static struct regfield_info glbl_axis_h2c_cnt_a2_field_info[] = {
    {"GLBL_AXIS_H2C_CNT_A2_MIDLE_CNTS",
     GLBL_AXIS_H2C_CNT_A2_MIDLE_CNTS_MASK},
};

/* MD:
 * Structure to hold register field information for AXIS H2C counters.
 */
static struct regfield_info glbl_axis_h2c_cnt_a3_field_info[] = {
    {"GLBL_AXIS_H2C_CNT_A3_MACTV_CNTS",
     GLBL_AXIS_H2C_CNT_A3_MACTV_CNTS_MASK},
};

/* MD:
 * Structure to hold register field information for AXIS H2C counters.
 */
static struct regfield_info glbl_axis_h2c_cnt_a4_field_info[] = {
    {"GLBL_AXIS_H2C_CNT_A4_MBUSY_CNTS",
     GLBL_AXIS_H2C_CNT_A4_MBUSY_CNTS_MASK},
    {"GLBL_AXIS_H2C_CNT_A4_MACTV_CNTS",
     GLBL_AXIS_H2C_CNT_A4_MACTV_CNTS_MASK},
};

/* MD:
 * Structure to hold register field information for AXIS H2C counters.
 */
static struct regfield_info glbl_axis_h2c_cnt_a5_field_info[] = {
    {"GLBL_AXIS_H2C_CNT_A5_MBUSY_CNTS",
     GLBL_AXIS_H2C_CNT_A5_MBUSY_CNTS_MASK},
};

/* MD:
 * Structure to hold register field information for AXIS C2H counters.
 */
static struct regfield_info glbl_axis_c2h_cnt_a0_field_info[] = {
    {"GLBL_AXIS_C2H_CNT_A0_SPKT_CNTS",
     GLBL_AXIS_C2H_CNT_A0_SPKT_CNTS_MASK},
};

/* MD:
 * Structure to hold register field information for AXIS C2H counters.
 */
static struct regfield_info glbl_axis_c2h_cnt_a1_field_info[] = {
    {"GLBL_AXIS_C2H_CNT_A1_SIDLE_CNTS",
     GLBL_AXIS_C2H_CNT_A1_SIDLE_CNTS_MASK},
    {"GLBL_AXIS_C2H_CNT_A1_SPKT_CNTS",
     GLBL_AXIS_C2H_CNT_A1_SPKT_CNTS_MASK},
};

/* MD:
 * Structure to hold register field information for AXIS C2H counters.
 */
static struct regfield_info glbl_axis_c2h_cnt_a2_field_info[] = {
    {"GLBL_AXIS_C2H_CNT_A2_SIDLE_CNTS",
     GLBL_AXIS_C2H_CNT_A2_SIDLE_CNTS_MASK},
};

/* MD:
 * Structure to hold register field information for AXIS C2H counters.
 */
static struct regfield_info glbl_axis_c2h_cnt_a3_field_info[] = {
    {"GLBL_AXIS_C2H_CNT_A3_SACTV_CNTS",
     GLBL_AXIS_C2H_CNT_A3_SACTV_CNTS_MASK},
};

/* MD:
 * Structure to hold register field information for AXIS C2H counters.
 */
static struct regfield_info glbl_axis_c2h_cnt_a4_field_info[] = {
    {"GLBL_AXIS_C2H_CNT_A4_SBUSY_CNTS",
     GLBL_AXIS_C2H_CNT_A4_SBUSY_CNTS_MASK},
    {"GLBL_AXIS_C2H_CNT_A4_SACTV_CNTS",
     GLBL_AXIS_C2H_CNT_A4_SACTV_CNTS_MASK},
};

/* MD:
 * Structure to hold register field information for AXIS C2H counters.
 */
static struct regfield_info glbl_axis_c2h_cnt_a5_field_info[] = {
    {"GLBL_AXIS_C2H_CNT_A5_SBUSY_CNTS",
     GLBL_AXIS_C2H_CNT_A5_SBUSY_CNTS_MASK},
};

/* MD:
 * Structure to hold register field information for M_AXI write counters.
 */
static struct regfield_info glbl_m_axi_wr_cnt_a0_field_info[] = {
    {"GLBL_M_AXI_WR_CNT_A0_PKT_CNTS",
     GLBL_M_AXI_WR_CNT_A0_PKT_CNTS_MASK},
};

/* MD:
 * Structure to hold register field information for M_AXI write counters.
 */
static struct regfield_info glbl_m_axi_wr_cnt_a1_field_info[] = {
    {"GLBL_M_AXI_WR_CNT_A1_IDLE_CNTS",
     GLBL_M_AXI_WR_CNT_A1_IDLE_CNTS_MASK},
    {"GLBL_M_AXI_WR_CNT_A1_PKT_CNTS",
     GLBL_M_AXI_WR_CNT_A1_PKT_CNTS_MASK},
};

/* MD:
 * Structure to hold register field information for M_AXI write counters.
 */
static struct regfield_info glbl_m_axi_wr_cnt_a2_field_info[] = {
    {"GLBL_M_AXI_WR_CNT_A2_IDLE_CNTS",
     GLBL_M_AXI_WR_CNT_A2_IDLE_CNTS_MASK},
};

/* MD:
 * Structure to hold register field information for M_AXI write counters.
 */
static struct regfield_info glbl_m_axi_wr_cnt_a3_field_info[] = {
    {"GLBL_M_AXI_WR_CNT_A3_ACTV_CNTS",
     GLBL_M_AXI_WR_CNT_A3_ACTV_CNTS_MASK},
};

/* MD:
 * Structure to hold register field information for M_AXI write counters.
 */
static struct regfield_info glbl_m_axi_wr_cnt_a4_field_info[] = {
    {"GLBL_M_AXI_WR_CNT_A4_BUSY_CNTS",
     GLBL_M_AXI_WR_CNT_A4_BUSY_CNTS_MASK},
    {"GLBL_M_AXI_WR_CNT_A4_ACTV_CNTS",
     GLBL_M_AXI_WR_CNT_A4_ACTV_CNTS_MASK},
};

/* MD:
 * Structure to hold register field information for M_AXI write counters.
 */
static struct regfield_info glbl_m_axi_wr_cnt_a5_field_info[] = {
    {"GLBL_M_AXI_WR_CNT_A5_BUSY_CNTS",
     GLBL_M_AXI_WR_CNT_A5_BUSY_CNTS_MASK},
};

/* MD:
 * Structure to hold register field information for M_AXI read counters.
 */
static struct regfield_info glbl_m_axi_rd_cnt_a0_field_info[] = {
    {"GLBL_M_AXI_RD_CNT_A0_PKT_CNTS",
     GLBL_M_AXI_RD_CNT_A0_PKT_CNTS_MASK},
};

#include <linux/kernel.h> // MD: Include for printk

// MD: Structure to hold register field information for AXI read counters
static struct regfield_info glbl_m_axi_rd_cnt_a1_field_info[] = {
    {"GLBL_M_AXI_RD_CNT_A1_IDLE_CNTS", GLBL_M_AXI_RD_CNT_A1_IDLE_CNTS_MASK},
    {"GLBL_M_AXI_RD_CNT_A1_PKT_CNTS", GLBL_M_AXI_RD_CNT_A1_PKT_CNTS_MASK},
};

// MD: Structure to hold register field information for AXI read counters
static struct regfield_info glbl_m_axi_rd_cnt_a2_field_info[] = {
    {"GLBL_M_AXI_RD_CNT_A2_IDLE_CNTS", GLBL_M_AXI_RD_CNT_A2_IDLE_CNTS_MASK},
};

// MD: Structure to hold register field information for AXI read counters
static struct regfield_info glbl_m_axi_rd_cnt_a3_field_info[] = {
    {"GLBL_M_AXI_RD_CNT_A3_ACTV_CNTS", GLBL_M_AXI_RD_CNT_A3_ACTV_CNTS_MASK},
};

// MD: Structure to hold register field information for AXI read counters
static struct regfield_info glbl_m_axi_rd_cnt_a4_field_info[] = {
    {"GLBL_M_AXI_RD_CNT_A4_BUSY_CNTS", GLBL_M_AXI_RD_CNT_A4_BUSY_CNTS_MASK},
    {"GLBL_M_AXI_RD_CNT_A4_ACTV_CNTS", GLBL_M_AXI_RD_CNT_A4_ACTV_CNTS_MASK},
};

// MD: Structure to hold register field information for AXI read counters
static struct regfield_info glbl_m_axi_rd_cnt_a5_field_info[] = {
    {"GLBL_M_AXI_RD_CNT_A5_BUSY_CNTS", GLBL_M_AXI_RD_CNT_A5_BUSY_CNTS_MASK},
};

// MD: Structure to hold register field information for AXIB write counters
static struct regfield_info glbl_m_axib_wr_cnt_a0_field_info[] = {
    {"GLBL_M_AXIB_WR_CNT_A0_PKT_CNTS", GLBL_M_AXIB_WR_CNT_A0_PKT_CNTS_MASK},
};

// MD: Structure to hold register field information for AXIB write counters
static struct regfield_info glbl_m_axib_wr_cnt_a1_field_info[] = {
    {"GLBL_M_AXIB_WR_CNT_A1_IDLE_CNTS", GLBL_M_AXIB_WR_CNT_A1_IDLE_CNTS_MASK},
    {"GLBL_M_AXIB_WR_CNT_A1_PKT_CNTS", GLBL_M_AXIB_WR_CNT_A1_PKT_CNTS_MASK},
};

// MD: Structure to hold register field information for AXIB write counters
static struct regfield_info glbl_m_axib_wr_cnt_a2_field_info[] = {
    {"GLBL_M_AXIB_WR_CNT_A2_IDLE_CNTS", GLBL_M_AXIB_WR_CNT_A2_IDLE_CNTS_MASK},
};

// MD: Structure to hold register field information for AXIB write counters
static struct regfield_info glbl_m_axib_wr_cnt_a3_field_info[] = {
    {"GLBL_M_AXIB_WR_CNT_A3_ACTV_CNTS", GLBL_M_AXIB_WR_CNT_A3_ACTV_CNTS_MASK},
};

// MD: Structure to hold register field information for AXIB write counters
static struct regfield_info glbl_m_axib_wr_cnt_a4_field_info[] = {
    {"GLBL_M_AXIB_WR_CNT_A4_BUSY_CNTS", GLBL_M_AXIB_WR_CNT_A4_BUSY_CNTS_MASK},
    {"GLBL_M_AXIB_WR_CNT_A4_ACTV_CNTS", GLBL_M_AXIB_WR_CNT_A4_ACTV_CNTS_MASK},
};

// MD: Structure to hold register field information for AXIB write counters
static struct regfield_info glbl_m_axib_wr_cnt_a5_field_info[] = {
    {"GLBL_M_AXIB_WR_CNT_A5_BUSY_CNTS", GLBL_M_AXIB_WR_CNT_A5_BUSY_CNTS_MASK},
};

// MD: Structure to hold register field information for AXIB read counters
static struct regfield_info glbl_m_axib_rd_cnt_a0_field_info[] = {
    {"GLBL_M_AXIB_RD_CNT_A0_PKT_CNTS", GLBL_M_AXIB_RD_CNT_A0_PKT_CNTS_MASK},
};

// MD: Structure to hold register field information for AXIB read counters
static struct regfield_info glbl_m_axib_rd_cnt_a1_field_info[] = {
    {"GLBL_M_AXIB_RD_CNT_A1_IDLE_CNTS", GLBL_M_AXIB_RD_CNT_A1_IDLE_CNTS_MASK},
    {"GLBL_M_AXIB_RD_CNT_A1_PKT_CNTS", GLBL_M_AXIB_RD_CNT_A1_PKT_CNTS_MASK},
};

// MD: Structure to hold register field information for AXIB read counters
static struct regfield_info glbl_m_axib_rd_cnt_a2_field_info[] = {
    {"GLBL_M_AXIB_RD_CNT_A2_IDLE_CNTS", GLBL_M_AXIB_RD_CNT_A2_IDLE_CNTS_MASK},
};

// MD: Structure to hold register field information for AXIB read counters
static struct regfield_info glbl_m_axib_rd_cnt_a3_field_info[] = {
    {"GLBL_M_AXIB_RD_CNT_A3_ACTV_CNTS", GLBL_M_AXIB_RD_CNT_A3_ACTV_CNTS_MASK},
};

// MD: Structure to hold register field information for AXIB read counters
static struct regfield_info glbl_m_axib_rd_cnt_a4_field_info[] = {
    {"GLBL_M_AXIB_RD_CNT_A4_BUSY_CNTS", GLBL_M_AXIB_RD_CNT_A4_BUSY_CNTS_MASK},
    {"GLBL_M_AXIB_RD_CNT_A4_ACTV_CNTS", GLBL_M_AXIB_RD_CNT_A4_ACTV_CNTS_MASK},
};

// MD: Structure to hold register field information for AXIB read counters
static struct regfield_info glbl_m_axib_rd_cnt_a5_field_info[] = {
    {"GLBL_M_AXIB_RD_CNT_A5_BUSY_CNTS", GLBL_M_AXIB_RD_CNT_A5_BUSY_CNTS_MASK},
};

// MD: Structure to hold register field information for S AXI write counters
static struct regfield_info glbl_s_axi_wr_cnt_a0_field_info[] = {
    {"GLBL_S_AXI_WR_CNT_A0_PKT_CNTS", GLBL_S_AXI_WR_CNT_A0_PKT_CNTS_MASK},
};

// MD: Structure to hold register field information for S AXI write counters
static struct regfield_info glbl_s_axi_wr_cnt_a1_field_info[] = {
    {"GLBL_S_AXI_WR_CNT_A1_IDLE_CNTS", GLBL_S_AXI_WR_CNT_A1_IDLE_CNTS_MASK},
    {"GLBL_S_AXI_WR_CNT_A1_PKT_CNTS", GLBL_S_AXI_WR_CNT_A1_PKT_CNTS_MASK},
};

// MD: Structure to hold register field information for S AXI write counters
static struct regfield_info glbl_s_axi_wr_cnt_a2_field_info[] = {
    {"GLBL_S_AXI_WR_CNT_A2_IDLE_CNTS", GLBL_S_AXI_WR_CNT_A2_IDLE_CNTS_MASK},
};

// MD: Structure to hold register field information for S AXI write counters
static struct regfield_info glbl_s_axi_wr_cnt_a3_field_info[] = {
    {"GLBL_S_AXI_WR_CNT_A3_ACTV_CNTS", GLBL_S_AXI_WR_CNT_A3_ACTV_CNTS_MASK},
};

// MD: Structure to hold register field information for S AXI write counters
static struct regfield_info glbl_s_axi_wr_cnt_a4_field_info[] = {
    {"GLBL_S_AXI_WR_CNT_A4_BUSY_CNTS", GLBL_S_AXI_WR_CNT_A4_BUSY_CNTS_MASK},
    {"GLBL_S_AXI_WR_CNT_A4_ACTV_CNTS", GLBL_S_AXI_WR_CNT_A4_ACTV_CNTS_MASK},
};

#include <linux/kernel.h> // MD: Include for printk

// MD: Structure to hold register field information for S AXI write counters
static struct regfield_info glbl_s_axi_wr_cnt_a5_field_info[] = {
    {"GLBL_S_AXI_WR_CNT_A5_BUSY_CNTS", GLBL_S_AXI_WR_CNT_A5_BUSY_CNTS_MASK},
};

// MD: Structure to hold register field information for S AXI read counters
static struct regfield_info glbl_s_axi_rd_cnt_a0_field_info[] = {
    {"GLBL_S_AXI_RD_CNT_A0_PKT_CNTS", GLBL_S_AXI_RD_CNT_A0_PKT_CNTS_MASK},
};

// MD: Structure to hold register field information for S AXI read counters
static struct regfield_info glbl_s_axi_rd_cnt_a1_field_info[] = {
    {"GLBL_S_AXI_RD_CNT_A1_IDLE_CNTS", GLBL_S_AXI_RD_CNT_A1_IDLE_CNTS_MASK},
    {"GLBL_S_AXI_RD_CNT_A1_PKT_CNTS", GLBL_S_AXI_RD_CNT_A1_PKT_CNTS_MASK},
};

// MD: Structure to hold register field information for S AXI read counters
static struct regfield_info glbl_s_axi_rd_cnt_a2_field_info[] = {
    {"GLBL_S_AXI_RD_CNT_A2_IDLE_CNTS", GLBL_S_AXI_RD_CNT_A2_IDLE_CNTS_MASK},
};

// MD: Structure to hold register field information for S AXI read counters
static struct regfield_info glbl_s_axi_rd_cnt_a3_field_info[] = {
    {"GLBL_S_AXI_RD_CNT_A3_ACTV_CNTS", GLBL_S_AXI_RD_CNT_A3_ACTV_CNTS_MASK},
};

// MD: Structure to hold register field information for S AXI read counters
static struct regfield_info glbl_s_axi_rd_cnt_a4_field_info[] = {
    {"GLBL_S_AXI_RD_CNT_A4_BUSY_CNTS", GLBL_S_AXI_RD_CNT_A4_BUSY_CNTS_MASK},
    {"GLBL_S_AXI_RD_CNT_A4_ACTV_CNTS", GLBL_S_AXI_RD_CNT_A4_ACTV_CNTS_MASK},
};

// MD: Structure to hold register field information for S AXI read counters
static struct regfield_info glbl_s_axi_rd_cnt_a5_field_info[] = {
    {"GLBL_S_AXI_RD_CNT_A5_BUSY_CNTS", GLBL_S_AXI_RD_CNT_A5_BUSY_CNTS_MASK},
};

// MD: Structure to hold register field information for S AXIS completion counters
static struct regfield_info glbl_s_axis_cmp_cnt_a0_field_info[] = {
    {"GLBL_S_AXIS_CMP_CNT_A0_PKT_CNTS", GLBL_S_AXIS_CMP_CNT_A0_PKT_CNTS_MASK},
};

// MD: Structure to hold register field information for S AXIS completion counters
static struct regfield_info glbl_s_axis_cmp_cnt_a1_field_info[] = {
    {"GLBL_S_AXIS_CMP_CNT_A1_IDLE_CNTS", GLBL_S_AXIS_CMP_CNT_A1_IDLE_CNTS_MASK},
    {"GLBL_S_AXIS_CMP_CNT_A1_PKT_CNTS", GLBL_S_AXIS_CMP_CNT_A1_PKT_CNTS_MASK},
};

// MD: Structure to hold register field information for S AXIS completion counters
static struct regfield_info glbl_s_axis_cmp_cnt_a2_field_info[] = {
    {"GLBL_S_AXIS_CMP_CNT_A2_IDLE_CNTS", GLBL_S_AXIS_CMP_CNT_A2_IDLE_CNTS_MASK},
};

// MD: Structure to hold register field information for S AXIS completion counters
static struct regfield_info glbl_s_axis_cmp_cnt_a3_field_info[] = {
    {"GLBL_S_AXIS_CMP_CNT_A3_ACTV_CNTS", GLBL_S_AXIS_CMP_CNT_A3_ACTV_CNTS_MASK},
};

// MD: Structure to hold register field information for S AXIS completion counters
static struct regfield_info glbl_s_axis_cmp_cnt_a4_field_info[] = {
    {"GLBL_S_AXIS_CMP_CNT_A4_BUSY_CNTS", GLBL_S_AXIS_CMP_CNT_A4_BUSY_CNTS_MASK},
    {"GLBL_S_AXIS_CMP_CNT_A4_ACTV_CNTS", GLBL_S_AXIS_CMP_CNT_A4_ACTV_CNTS_MASK},
};

// MD: Structure to hold register field information for S AXIS completion counters
static struct regfield_info glbl_s_axis_cmp_cnt_a5_field_info[] = {
    {"GLBL_S_AXIS_CMP_CNT_A5_BUSY_CNTS", GLBL_S_AXIS_CMP_CNT_A5_BUSY_CNTS_MASK},
};

// MD: Structure to hold register field information for indirect context data
static struct regfield_info ind_ctxt_data_field_info[] = {
    {"IND_CTXT_DATA_DATA", IND_CTXT_DATA_DATA_MASK},
};

// MD: Structure to hold register field information for indirect context mask
static struct regfield_info ind_ctxt_mask_field_info[] = {
    {"IND_CTXT", IND_CTXT_MASK},
};

// MD: Structure to hold register field information for indirect context command
static struct regfield_info ind_ctxt_cmd_field_info[] = {
    {"IND_CTXT_CMD_RSVD_1", IND_CTXT_CMD_RSVD_1_MASK},
    {"IND_CTXT_CMD_QID", IND_CTXT_CMD_QID_MASK},
    {"IND_CTXT_CMD_OP", IND_CTXT_CMD_OP_MASK},
    {"IND_CTXT_CMD_SEL", IND_CTXT_CMD_SEL_MASK},
    {"IND_CTXT_CMD_BUSY", IND_CTXT_CMD_BUSY_MASK},
};

// MD: Structure to hold register field information for C2H timer count
static struct regfield_info c2h_timer_cnt_field_info[] = {
    {"C2H_TIMER_CNT_RSVD_1", C2H_TIMER_CNT_RSVD_1_MASK},
    {"C2H_TIMER_CNT", C2H_TIMER_CNT_MASK},
};

// MD: Structure to hold register field information for C2H count threshold
static struct regfield_info c2h_cnt_th_field_info[] = {
    {"C2H_CNT_TH_RSVD_1", C2H_CNT_TH_RSVD_1_MASK},
    {"C2H_CNT_TH_THESHOLD_CNT", C2H_CNT_TH_THESHOLD_CNT_MASK},
};

// MD: Structure to hold register field information for C2H prefetch configuration 1
static struct regfield_info c2h_pfch_cfg_1_field_info[] = {
    {"C2H_PFCH_CFG_1_EVT_QCNT_TH", C2H_PFCH_CFG_1_EVT_QCNT_TH_MASK},
    {"C2H_PFCH_CFG_1_QCNT", C2H_PFCH_CFG_1_QCNT_MASK},
};

// MD: Structure to hold register field information for C2H prefetch configuration 2
static struct regfield_info c2h_pfch_cfg_2_field_info[] = {
    {"C2H_PFCH_CFG_2_FENCE", C2H_PFCH_CFG_2_FENCE_MASK},
    {"C2H_PFCH_CFG_2_RSVD", C2H_PFCH_CFG_2_RSVD_MASK},
    {"C2H_PFCH_CFG_2_VAR_DESC_NO_DROP", C2H_PFCH_CFG_2_VAR_DESC_NO_DROP_MASK},
    {"C2H_PFCH_CFG_2_LL_SZ_TH", C2H_PFCH_CFG_2_LL_SZ_TH_MASK},
    {"C2H_PFCH_CFG_2_VAR_DESC_NUM", C2H_PFCH_CFG_2_VAR_DESC_NUM_MASK},
    {"C2H_PFCH_CFG_2_NUM", C2H_PFCH_CFG_2_NUM_MASK},
};

// MD: Structure to hold register field information for C2H statistics S AXIS C2H accepted
static struct regfield_info c2h_stat_s_axis_c2h_accepted_field_info[] = {
    {"C2H_STAT_S_AXIS_C2H_ACCEPTED", C2H_STAT_S_AXIS_C2H_ACCEPTED_MASK},
};

// MD: Structure to hold register field information for C2H statistics S AXIS WRB accepted
static struct regfield_info c2h_stat_s_axis_wrb_accepted_field_info[] = {
    {"C2H_STAT_S_AXIS_WRB_ACCEPTED", C2H_STAT_S_AXIS_WRB_ACCEPTED_MASK},
};

// MD: Structure to hold register field information for C2H statistics descriptor response packet accepted
static struct regfield_info c2h_stat_desc_rsp_pkt_accepted_field_info[] = {
    {"C2H_STAT_DESC_RSP_PKT_ACCEPTED_D", C2H_STAT_DESC_RSP_PKT_ACCEPTED_D_MASK},
};

// MD: Structure to hold register field information for C2H statistics AXIS package completion
static struct regfield_info c2h_stat_axis_pkg_cmp_field_info[] = {
    {"C2H_STAT_AXIS_PKG_CMP", C2H_STAT_AXIS_PKG_CMP_MASK},
};

// MD: Structure to hold register field information for C2H statistics descriptor response accepted
static struct regfield_info c2h_stat_desc_rsp_accepted_field_info[] = {
    {"C2H_STAT_DESC_RSP_ACCEPTED_D", C2H_STAT_DESC_RSP_ACCEPTED_D_MASK},
};

// MD: Structure to hold register field information for C2H statistics descriptor response completion
static struct regfield_info c2h_stat_desc_rsp_cmp_field_info[] = {
    {"C2H_STAT_DESC_RSP_CMP_D", C2H_STAT_DESC_RSP_CMP_D_MASK},
};

// MD: Structure to hold register field information for C2H statistics WRQ output
static struct regfield_info c2h_stat_wrq_out_field_info[] = {
    {"C2H_STAT_WRQ_OUT", C2H_STAT_WRQ_OUT_MASK},
};

// MD: Structure to hold register field information for C2H statistics WPL read enable accepted
static struct regfield_info c2h_stat_wpl_ren_accepted_field_info[] = {
    {"C2H_STAT_WPL_REN_ACCEPTED", C2H_STAT_WPL_REN_ACCEPTED_MASK},
};



// MD: Structure to hold register field information
static struct regfield_info c2h_stat_total_wrq_len_field_info[] = {
    {"C2H_STAT_TOTAL_WRQ_LEN", C2H_STAT_TOTAL_WRQ_LEN_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_stat_total_wrq_len_field_info\n");

static struct regfield_info c2h_stat_total_wpl_len_field_info[] = {
    {"C2H_STAT_TOTAL_WPL_LEN", C2H_STAT_TOTAL_WPL_LEN_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_stat_total_wpl_len_field_info\n");

static struct regfield_info c2h_buf_sz_field_info[] = {
    {"C2H_BUF_SZ_IZE", C2H_BUF_SZ_IZE_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_buf_sz_field_info\n");

// MD: Structure to hold error status field information
static struct regfield_info c2h_err_stat_field_info[] = {
    {"C2H_ERR_STAT_RSVD_1", C2H_ERR_STAT_RSVD_1_MASK},
    {"C2H_ERR_STAT_WRB_PORT_ID_ERR", C2H_ERR_STAT_WRB_PORT_ID_ERR_MASK},
    {"C2H_ERR_STAT_HDR_PAR_ERR", C2H_ERR_STAT_HDR_PAR_ERR_MASK},
    {"C2H_ERR_STAT_HDR_ECC_COR_ERR", C2H_ERR_STAT_HDR_ECC_COR_ERR_MASK},
    {"C2H_ERR_STAT_HDR_ECC_UNC_ERR", C2H_ERR_STAT_HDR_ECC_UNC_ERR_MASK},
    {"C2H_ERR_STAT_AVL_RING_DSC_ERR", C2H_ERR_STAT_AVL_RING_DSC_ERR_MASK},
    {"C2H_ERR_STAT_WRB_PRTY_ERR", C2H_ERR_STAT_WRB_PRTY_ERR_MASK},
    {"C2H_ERR_STAT_WRB_CIDX_ERR", C2H_ERR_STAT_WRB_CIDX_ERR_MASK},
    {"C2H_ERR_STAT_WRB_QFULL_ERR", C2H_ERR_STAT_WRB_QFULL_ERR_MASK},
    {"C2H_ERR_STAT_WRB_INV_Q_ERR", C2H_ERR_STAT_WRB_INV_Q_ERR_MASK},
    {"C2H_ERR_STAT_RSVD_2", C2H_ERR_STAT_RSVD_2_MASK},
    {"C2H_ERR_STAT_PORT_ID_CTXT_MISMATCH", C2H_ERR_STAT_PORT_ID_CTXT_MISMATCH_MASK},
    {"C2H_ERR_STAT_ERR_DESC_CNT", C2H_ERR_STAT_ERR_DESC_CNT_MASK},
    {"C2H_ERR_STAT_RSVD_3", C2H_ERR_STAT_RSVD_3_MASK},
    {"C2H_ERR_STAT_MSI_INT_FAIL", C2H_ERR_STAT_MSI_INT_FAIL_MASK},
    {"C2H_ERR_STAT_ENG_WPL_DATA_PAR_ERR", C2H_ERR_STAT_ENG_WPL_DATA_PAR_ERR_MASK},
    {"C2H_ERR_STAT_RSVD_4", C2H_ERR_STAT_RSVD_4_MASK},
    {"C2H_ERR_STAT_DESC_RSP_ERR", C2H_ERR_STAT_DESC_RSP_ERR_MASK},
    {"C2H_ERR_STAT_QID_MISMATCH", C2H_ERR_STAT_QID_MISMATCH_MASK},
    {"C2H_ERR_STAT_SH_CMPT_DSC_ERR", C2H_ERR_STAT_SH_CMPT_DSC_ERR_MASK},
    {"C2H_ERR_STAT_LEN_MISMATCH", C2H_ERR_STAT_LEN_MISMATCH_MASK},
    {"C2H_ERR_STAT_MTY_MISMATCH", C2H_ERR_STAT_MTY_MISMATCH_MASK},
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_err_stat_field_info\n");

// MD: Structure to hold error mask field information
static struct regfield_info c2h_err_mask_field_info[] = {
    {"C2H_ERR_EN", C2H_ERR_EN_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_err_mask_field_info\n");

// MD: Structure to hold fatal error status field information
static struct regfield_info c2h_fatal_err_stat_field_info[] = {
    {"C2H_FATAL_ERR_STAT_RSVD_1", C2H_FATAL_ERR_STAT_RSVD_1_MASK},
    {"C2H_FATAL_ERR_STAT_HDR_ECC_UNC_ERR", C2H_FATAL_ERR_STAT_HDR_ECC_UNC_ERR_MASK},
    {"C2H_FATAL_ERR_STAT_AVL_RING_FIFO_RAM_RDBE", C2H_FATAL_ERR_STAT_AVL_RING_FIFO_RAM_RDBE_MASK},
    {"C2H_FATAL_ERR_STAT_WPL_DATA_PAR_ERR", C2H_FATAL_ERR_STAT_WPL_DATA_PAR_ERR_MASK},
    {"C2H_FATAL_ERR_STAT_PLD_FIFO_RAM_RDBE", C2H_FATAL_ERR_STAT_PLD_FIFO_RAM_RDBE_MASK},
    {"C2H_FATAL_ERR_STAT_QID_FIFO_RAM_RDBE", C2H_FATAL_ERR_STAT_QID_FIFO_RAM_RDBE_MASK},
    {"C2H_FATAL_ERR_STAT_CMPT_FIFO_RAM_RDBE", C2H_FATAL_ERR_STAT_CMPT_FIFO_RAM_RDBE_MASK},
    {"C2H_FATAL_ERR_STAT_WRB_COAL_DATA_RAM_RDBE", C2H_FATAL_ERR_STAT_WRB_COAL_DATA_RAM_RDBE_MASK},
    {"C2H_FATAL_ERR_STAT_RESERVED2", C2H_FATAL_ERR_STAT_RESERVED2_MASK},
    {"C2H_FATAL_ERR_STAT_INT_CTXT_RAM_RDBE", C2H_FATAL_ERR_STAT_INT_CTXT_RAM_RDBE_MASK},
    {"C2H_FATAL_ERR_STAT_DESC_REQ_FIFO_RAM_RDBE", C2H_FATAL_ERR_STAT_DESC_REQ_FIFO_RAM_RDBE_MASK},
    {"C2H_FATAL_ERR_STAT_PFCH_CTXT_RAM_RDBE", C2H_FATAL_ERR_STAT_PFCH_CTXT_RAM_RDBE_MASK},
    {"C2H_FATAL_ERR_STAT_WRB_CTXT_RAM_RDBE", C2H_FATAL_ERR_STAT_WRB_CTXT_RAM_RDBE_MASK},
    {"C2H_FATAL_ERR_STAT_PFCH_LL_RAM_RDBE", C2H_FATAL_ERR_STAT_PFCH_LL_RAM_RDBE_MASK},
    {"C2H_FATAL_ERR_STAT_TIMER_FIFO_RAM_RDBE", C2H_FATAL_ERR_STAT_TIMER_FIFO_RAM_RDBE_MASK},
    {"C2H_FATAL_ERR_STAT_QID_MISMATCH", C2H_FATAL_ERR_STAT_QID_MISMATCH_MASK},
    {"C2H_FATAL_ERR_STAT_RESERVED1", C2H_FATAL_ERR_STAT_RESERVED1_MASK},
    {"C2H_FATAL_ERR_STAT_LEN_MISMATCH", C2H_FATAL_ERR_STAT_LEN_MISMATCH_MASK},
    {"C2H_FATAL_ERR_STAT_MTY_MISMATCH", C2H_FATAL_ERR_STAT_MTY_MISMATCH_MASK},
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_fatal_err_stat_field_info\n");

// MD: Structure to hold fatal error mask field information
static struct regfield_info c2h_fatal_err_mask_field_info[] = {
    {"C2H_FATAL_ERR_C2HEN", C2H_FATAL_ERR_C2HEN_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_fatal_err_mask_field_info\n");

// MD: Structure to hold fatal error enable field information
static struct regfield_info c2h_fatal_err_enable_field_info[] = {
    {"C2H_FATAL_ERR_ENABLE_RSVD_1", C2H_FATAL_ERR_ENABLE_RSVD_1_MASK},
    {"C2H_FATAL_ERR_ENABLE_WPL_PAR_INV", C2H_FATAL_ERR_ENABLE_WPL_PAR_INV_MASK},
    {"C2H_FATAL_ERR_ENABLE_WRQ_DIS", C2H_FATAL_ERR_ENABLE_WRQ_DIS_MASK},
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_fatal_err_enable_field_info\n");

// MD: Structure to hold global error interrupt field information
static struct regfield_info glbl_err_int_field_info[] = {
    {"GLBL_ERR_INT_RSVD_1", GLBL_ERR_INT_RSVD_1_MASK},
    {"GLBL_ERR_INT_HOST_ID", GLBL_ERR_INT_HOST_ID_MASK},
    {"GLBL_ERR_INT_DIS_INTR_ON_VF", GLBL_ERR_INT_DIS_INTR_ON_VF_MASK},
    {"GLBL_ERR_INT_ARM", GLBL_ERR_INT_ARM_MASK},
    {"GLBL_ERR_INT_EN_COAL", GLBL_ERR_INT_EN_COAL_MASK},
    {"GLBL_ERR_INT_VEC", GLBL_ERR_INT_VEC_MASK},
    {"GLBL_ERR_INT_FUNC", GLBL_ERR_INT_FUNC_MASK},
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized glbl_err_int_field_info\n");

// MD: Structure to hold prefetch configuration field information
static struct regfield_info c2h_pfch_cfg_field_info[] = {
    {"C2H_PFCH_CFG_EVTFL_TH", C2H_PFCH_CFG_EVTFL_TH_MASK},
    {"C2H_PFCH_CFG_FL_TH", C2H_PFCH_CFG_FL_TH_MASK},
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_pfch_cfg_field_info\n");

// MD: Structure to hold interrupt timer tick field information
static struct regfield_info c2h_int_timer_tick_field_info[] = {
    {"C2H_INT_TIMER_TICK", C2H_INT_TIMER_TICK_MASK},
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_int_timer_tick_field_info\n");

// MD: Structure to hold descriptor response drop accepted field information
static struct regfield_info c2h_stat_desc_rsp_drop_accepted_field_info[] = {
    {"C2H_STAT_DESC_RSP_DROP_ACCEPTED_D", C2H_STAT_DESC_RSP_DROP_ACCEPTED_D_MASK},
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_stat_desc_rsp_drop_accepted_field_info\n");



// MD: Structure to hold descriptor response error accepted field information
static struct regfield_info c2h_stat_desc_rsp_err_accepted_field_info[] = {
    {"C2H_STAT_DESC_RSP_ERR_ACCEPTED_D", C2H_STAT_DESC_RSP_ERR_ACCEPTED_D_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_stat_desc_rsp_err_accepted_field_info\n");

// MD: Structure to hold descriptor request field information
static struct regfield_info c2h_stat_desc_req_field_info[] = {
    {"C2H_STAT_DESC_REQ", C2H_STAT_DESC_REQ_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_stat_desc_req_field_info\n");

// MD: Structure to hold DMA engine 0 debug field information
static struct regfield_info c2h_stat_dbg_dma_eng_0_field_info[] = {
    {"C2H_STAT_DMA_ENG_0_S_AXIS_C2H_TVALID", C2H_STAT_DMA_ENG_0_S_AXIS_C2H_TVALID_MASK},
    {"C2H_STAT_DMA_ENG_0_S_AXIS_C2H_TREADY", C2H_STAT_DMA_ENG_0_S_AXIS_C2H_TREADY_MASK},
    {"C2H_STAT_DMA_ENG_0_S_AXIS_WRB_TVALID", C2H_STAT_DMA_ENG_0_S_AXIS_WRB_TVALID_MASK},
    {"C2H_STAT_DMA_ENG_0_S_AXIS_WRB_TREADY", C2H_STAT_DMA_ENG_0_S_AXIS_WRB_TREADY_MASK},
    {"C2H_STAT_DMA_ENG_0_PLD_FIFO_IN_RDY", C2H_STAT_DMA_ENG_0_PLD_FIFO_IN_RDY_MASK},
    {"C2H_STAT_DMA_ENG_0_QID_FIFO_IN_RDY", C2H_STAT_DMA_ENG_0_QID_FIFO_IN_RDY_MASK},
    {"C2H_STAT_DMA_ENG_0_ARB_FIFO_OUT_VLD", C2H_STAT_DMA_ENG_0_ARB_FIFO_OUT_VLD_MASK},
    {"C2H_STAT_DMA_ENG_0_ARB_FIFO_OUT_QID", C2H_STAT_DMA_ENG_0_ARB_FIFO_OUT_QID_MASK},
    {"C2H_STAT_DMA_ENG_0_WRB_FIFO_IN_RDY", C2H_STAT_DMA_ENG_0_WRB_FIFO_IN_RDY_MASK},
    {"C2H_STAT_DMA_ENG_0_WRB_FIFO_OUT_CNT", C2H_STAT_DMA_ENG_0_WRB_FIFO_OUT_CNT_MASK},
    {"C2H_STAT_DMA_ENG_0_WRB_SM_CS", C2H_STAT_DMA_ENG_0_WRB_SM_CS_MASK},
    {"C2H_STAT_DMA_ENG_0_MAIN_SM_CS", C2H_STAT_DMA_ENG_0_MAIN_SM_CS_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_stat_dbg_dma_eng_0_field_info\n");

// MD: Structure to hold DMA engine 1 debug field information
static struct regfield_info c2h_stat_dbg_dma_eng_1_field_info[] = {
    {"C2H_STAT_DMA_ENG_1_WRB_USER_0_CMPT_TYPE", C2H_STAT_DMA_ENG_1_WRB_USER_0_CMPT_TYPE_MASK},
    {"C2H_STAT_DMA_ENG_1_DESC_RSP_FIFO_OUT_VLD", C2H_STAT_DMA_ENG_1_DESC_RSP_FIFO_OUT_VLD_MASK},
    {"C2H_STAT_DMA_ENG_1_QID_FIFO_OUT_CNT", C2H_STAT_DMA_ENG_1_QID_FIFO_OUT_CNT_MASK},
    {"C2H_STAT_DMA_ENG_1_PLD_FIFO_OUT_CNT", C2H_STAT_DMA_ENG_1_PLD_FIFO_OUT_CNT_MASK},
    {"C2H_STAT_DMA_ENG_1_PLD_ST_FIFO_CNT", C2H_STAT_DMA_ENG_1_PLD_ST_FIFO_CNT_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_stat_dbg_dma_eng_1_field_info\n");

// MD: Structure to hold DMA engine 2 debug field information
static struct regfield_info c2h_stat_dbg_dma_eng_2_field_info[] = {
    {"C2H_STAT_DMA_ENG_2_WRB_USER_1_CMPT_TYPE", C2H_STAT_DMA_ENG_2_WRB_USER_1_CMPT_TYPE_MASK},
    {"C2H_STAT_DMA_ENG_2_DESC_RSP_FIFO_OUT_VLD_1", C2H_STAT_DMA_ENG_2_DESC_RSP_FIFO_OUT_VLD_1_MASK},
    {"C2H_STAT_DMA_ENG_2_QID_FIFO_OUT_CNT_1", C2H_STAT_DMA_ENG_2_QID_FIFO_OUT_CNT_1_MASK},
    {"C2H_STAT_DMA_ENG_2_PLD_FIFO_OUT_CNT_1", C2H_STAT_DMA_ENG_2_PLD_FIFO_OUT_CNT_1_MASK},
    {"C2H_STAT_DMA_ENG_2_PLD_ST_FIFO_CNT_1", C2H_STAT_DMA_ENG_2_PLD_ST_FIFO_CNT_1_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_stat_dbg_dma_eng_2_field_info\n");

// MD: Structure to hold DMA engine 3 debug field information
static struct regfield_info c2h_stat_dbg_dma_eng_3_field_info[] = {
    {"C2H_STAT_DMA_ENG_3_QID_FIFO_OUT_DAT_HAS_CMPT", C2H_STAT_DMA_ENG_3_QID_FIFO_OUT_DAT_HAS_CMPT_MASK},
    {"C2H_STAT_DMA_ENG_3_QID_FIFO_OUT_DAT_MARKER", C2H_STAT_DMA_ENG_3_QID_FIFO_OUT_DAT_MARKER_MASK},
    {"C2H_STAT_DMA_ENG_3_QID_FIFO_OUT_DAT_DROP_REQ", C2H_STAT_DMA_ENG_3_QID_FIFO_OUT_DAT_DROP_REQ_MASK},
    {"C2H_STAT_DMA_ENG_3_QID_FIFO_OUT_DAT_QID", C2H_STAT_DMA_ENG_3_QID_FIFO_OUT_DAT_QID_MASK},
    {"C2H_STAT_DMA_ENG_3_WR_HDR_FIFO_OUT_CNT", C2H_STAT_DMA_ENG_3_WR_HDR_FIFO_OUT_CNT_MASK},
    {"C2H_STAT_DMA_ENG_3_QID_FIFO_OUT_VLD", C2H_STAT_DMA_ENG_3_QID_FIFO_OUT_VLD_MASK},
    {"C2H_STAT_DMA_ENG_3_PLD_FIFO_OUT_VLD", C2H_STAT_DMA_ENG_3_PLD_FIFO_OUT_VLD_MASK},
    {"C2H_STAT_DMA_ENG_3_PLD_ST_FIFO_OUT_VLD", C2H_STAT_DMA_ENG_3_PLD_ST_FIFO_OUT_VLD_MASK},
    {"C2H_STAT_DMA_ENG_3_PLD_ST_FIFO_OUT_DATA_EOP", C2H_STAT_DMA_ENG_3_PLD_ST_FIFO_OUT_DATA_EOP_MASK},
    {"C2H_STAT_DMA_ENG_3_PLD_ST_FIFO_OUT_DATA_AVL_IDX_ENABLE", C2H_STAT_DMA_ENG_3_PLD_ST_FIFO_OUT_DATA_AVL_IDX_ENABLE_MASK},
    {"C2H_STAT_DMA_ENG_3_PLD_ST_FIFO_OUT_DATA_DROP", C2H_STAT_DMA_ENG_3_PLD_ST_FIFO_OUT_DATA_DROP_MASK},
    {"C2H_STAT_DMA_ENG_3_PLD_ST_FIFO_OUT_DATA_ERR", C2H_STAT_DMA_ENG_3_PLD_ST_FIFO_OUT_DATA_ERR_MASK},
    {"C2H_STAT_DMA_ENG_3_DESC_CNT_FIFO_IN_RDY", C2H_STAT_DMA_ENG_3_DESC_CNT_FIFO_IN_RDY_MASK},
    {"C2H_STAT_DMA_ENG_3_DESC_RSP_FIFO_IN_RDY", C2H_STAT_DMA_ENG_3_DESC_RSP_FIFO_IN_RDY_MASK},
    {"C2H_STAT_DMA_ENG_3_PLD_PKT_ID_LARGER", C2H_STAT_DMA_ENG_3_PLD_PKT_ID_LARGER_MASK},
    {"C2H_STAT_DMA_ENG_3_WCP_FIFO_IN_RDY", C2H_STAT_DMA_ENG_3_WCP_FIFO_IN_RDY_MASK},
    {"C2H_STAT_DMA_ENG_3_PLD_ST_FIFO_IN_RDY", C2H_STAT_DMA_ENG_3_PLD_ST_FIFO_IN_RDY_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_stat_dbg_dma_eng_3_field_info\n");

// MD: Structure to hold prefetch error context field information
static struct regfield_info c2h_dbg_pfch_err_ctxt_field_info[] = {
    {"C2H_PFCH_ERR_CTXT_RSVD_1", C2H_PFCH_ERR_CTXT_RSVD_1_MASK},
    {"C2H_PFCH_ERR_CTXT_ERR_STAT", C2H_PFCH_ERR_CTXT_ERR_STAT_MASK},
    {"C2H_PFCH_ERR_CTXT_CMD_WR", C2H_PFCH_ERR_CTXT_CMD_WR_MASK},
    {"C2H_PFCH_ERR_CTXT_QID", C2H_PFCH_ERR_CTXT_QID_MASK},
    {"C2H_PFCH_ERR_CTXT_DONE", C2H_PFCH_ERR_CTXT_DONE_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_dbg_pfch_err_ctxt_field_info\n");

// MD: Structure to hold first error QID field information
static struct regfield_info c2h_first_err_qid_field_info[] = {
    {"C2H_FIRST_ERR_QID_RSVD_1", C2H_FIRST_ERR_QID_RSVD_1_MASK},
    {"C2H_FIRST_ERR_QID_ERR_TYPE", C2H_FIRST_ERR_QID_ERR_TYPE_MASK},
    {"C2H_FIRST_ERR_QID_RSVD", C2H_FIRST_ERR_QID_RSVD_MASK},
    {"C2H_FIRST_ERR_QID_QID", C2H_FIRST_ERR_QID_QID_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_first_err_qid_field_info\n");

// MD: Structure to hold number of WRB in field information
static struct regfield_info stat_num_wrb_in_field_info[] = {
    {"STAT_NUM_WRB_IN_RSVD_1", STAT_NUM_WRB_IN_RSVD_1_MASK},
    {"STAT_NUM_WRB_IN_WRB_CNT", STAT_NUM_WRB_IN_WRB_CNT_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized stat_num_wrb_in_field_info\n");

// MD: Structure to hold number of WRB out field information
static struct regfield_info stat_num_wrb_out_field_info[] = {
    {"STAT_NUM_WRB_OUT_RSVD_1", STAT_NUM_WRB_OUT_RSVD_1_MASK},
    {"STAT_NUM_WRB_OUT_WRB_CNT", STAT_NUM_WRB_OUT_WRB_CNT_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized stat_num_wrb_out_field_info\n");

// MD: Structure to hold number of WRB drop field information
static struct regfield_info stat_num_wrb_drp_field_info[] = {
    {"STAT_NUM_WRB_DRP_RSVD_1", STAT_NUM_WRB_DRP_RSVD_1_MASK},
    {"STAT_NUM_WRB_DRP_WRB_CNT", STAT_NUM_WRB_DRP_WRB_CNT_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized stat_num_wrb_drp_field_info\n");

// MD: Structure to hold number of status descriptor out field information
static struct regfield_info stat_num_stat_desc_out_field_info[] = {
    {"STAT_NUM_STAT_DESC_OUT_RSVD_1", STAT_NUM_STAT_DESC_OUT_RSVD_1_MASK},
    {"STAT_NUM_STAT_DESC_OUT_CNT", STAT_NUM_STAT_DESC_OUT_CNT_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized stat_num_stat_desc_out_field_info\n");



// MD: Structure to hold DMA engine 5 debug field information
static struct regfield_info c2h_stat_dbg_dma_eng_5_field_info[] = {
    {"C2H_STAT_DMA_ENG_5_ARB_FIFO_IN_RDY", C2H_STAT_DMA_ENG_5_ARB_FIFO_IN_RDY_MASK},
    {"C2H_STAT_DMA_ENG_5_WRB_SM_VIRT_CH", C2H_STAT_DMA_ENG_5_WRB_SM_VIRT_CH_MASK},
    {"C2H_STAT_DMA_ENG_5_WRB_FIFO_IN_REQ", C2H_STAT_DMA_ENG_5_WRB_FIFO_IN_REQ_MASK},
    {"C2H_STAT_DMA_ENG_5_ARB_FIFO_OUT_CNT", C2H_STAT_DMA_ENG_5_ARB_FIFO_OUT_CNT_MASK},
    {"C2H_STAT_DMA_ENG_5_ARB_FIFO_OUT_DATA_LEN", C2H_STAT_DMA_ENG_5_ARB_FIFO_OUT_DATA_LEN_MASK},
    {"C2H_STAT_DMA_ENG_5_ARB_FIFO_OUT_DATA_VIRT_CH", C2H_STAT_DMA_ENG_5_ARB_FIFO_OUT_DATA_VIRT_CH_MASK},
    {"C2H_STAT_DMA_ENG_5_ARB_FIFO_OUT_DATA_VAR_DESC", C2H_STAT_DMA_ENG_5_ARB_FIFO_OUT_DATA_VAR_DESC_MASK},
    {"C2H_STAT_DMA_ENG_5_ARB_FIFO_OUT_DATA_DROP_REQ", C2H_STAT_DMA_ENG_5_ARB_FIFO_OUT_DATA_DROP_REQ_MASK},
    {"C2H_STAT_DMA_ENG_5_ARB_FIFO_OUT_DATA_NUM_BUF_OV", C2H_STAT_DMA_ENG_5_ARB_FIFO_OUT_DATA_NUM_BUF_OV_MASK},
    {"C2H_STAT_DMA_ENG_5_ARB_FIFO_OUT_DATA_MARKER", C2H_STAT_DMA_ENG_5_ARB_FIFO_OUT_DATA_MARKER_MASK},
    {"C2H_STAT_DMA_ENG_5_ARB_FIFO_OUT_DATA_HAS_CMPT", C2H_STAT_DMA_ENG_5_ARB_FIFO_OUT_DATA_HAS_CMPT_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_stat_dbg_dma_eng_5_field_info\n");

// MD: Structure to hold prefetch QID field information
static struct regfield_info c2h_dbg_pfch_qid_field_info[] = {
    {"C2H_PFCH_QID_RSVD_1", C2H_PFCH_QID_RSVD_1_MASK},
    {"C2H_PFCH_QID_ERR_CTXT", C2H_PFCH_QID_ERR_CTXT_MASK},
    {"C2H_PFCH_QID_TARGET", C2H_PFCH_QID_TARGET_MASK},
    {"C2H_PFCH_QID_QID_OR_TAG", C2H_PFCH_QID_QID_OR_TAG_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_dbg_pfch_qid_field_info\n");

// MD: Structure to hold prefetch field information
static struct regfield_info c2h_dbg_pfch_field_info[] = {
    {"C2H_PFCH_DATA", C2H_PFCH_DATA_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_dbg_pfch_field_info\n");

// MD: Structure to hold interrupt debug field information
static struct regfield_info c2h_int_dbg_field_info[] = {
    {"C2H_INT_RSVD_1", C2H_INT_RSVD_1_MASK},
    {"C2H_INT_INT_COAL_SM", C2H_INT_INT_COAL_SM_MASK},
    {"C2H_INT_INT_SM", C2H_INT_INT_SM_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_int_dbg_field_info\n");

// MD: Structure to hold immediate accepted field information
static struct regfield_info c2h_stat_imm_accepted_field_info[] = {
    {"C2H_STAT_IMM_ACCEPTED_RSVD_1", C2H_STAT_IMM_ACCEPTED_RSVD_1_MASK},
    {"C2H_STAT_IMM_ACCEPTED_CNT", C2H_STAT_IMM_ACCEPTED_CNT_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_stat_imm_accepted_field_info\n");

// MD: Structure to hold marker accepted field information
static struct regfield_info c2h_stat_marker_accepted_field_info[] = {
    {"C2H_STAT_MARKER_ACCEPTED_RSVD_1", C2H_STAT_MARKER_ACCEPTED_RSVD_1_MASK},
    {"C2H_STAT_MARKER_ACCEPTED_CNT", C2H_STAT_MARKER_ACCEPTED_CNT_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_stat_marker_accepted_field_info\n");

// MD: Structure to hold disable completion accepted field information
static struct regfield_info c2h_stat_disable_cmp_accepted_field_info[] = {
    {"C2H_STAT_DISABLE_CMP_ACCEPTED_RSVD_1", C2H_STAT_DISABLE_CMP_ACCEPTED_RSVD_1_MASK},
    {"C2H_STAT_DISABLE_CMP_ACCEPTED_CNT", C2H_STAT_DISABLE_CMP_ACCEPTED_CNT_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_stat_disable_cmp_accepted_field_info\n");

// MD: Structure to hold payload FIFO credit count field information
static struct regfield_info c2h_pld_fifo_crdt_cnt_field_info[] = {
    {"C2H_PLD_FIFO_CRDT_CNT_RSVD_1", C2H_PLD_FIFO_CRDT_CNT_RSVD_1_MASK},
    {"C2H_PLD_FIFO_CRDT_CNT_CNT", C2H_PLD_FIFO_CRDT_CNT_CNT_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_pld_fifo_crdt_cnt_field_info\n");

// MD: Structure to hold dynamic interrupt request field information
static struct regfield_info c2h_intr_dyn_req_field_info[] = {
    {"C2H_INTR_DYN_REQ_RSVD_1", C2H_INTR_DYN_REQ_RSVD_1_MASK},
    {"C2H_INTR_DYN_REQ_CNT", C2H_INTR_DYN_REQ_CNT_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_intr_dyn_req_field_info\n");

// MD: Structure to hold dynamic interrupt miscellaneous field information
static struct regfield_info c2h_intr_dyn_misc_field_info[] = {
    {"C2H_INTR_DYN_MISC_RSVD_1", C2H_INTR_DYN_MISC_RSVD_1_MASK},
    {"C2H_INTR_DYN_MISC_CNT", C2H_INTR_DYN_MISC_CNT_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_intr_dyn_misc_field_info\n");

// MD: Structure to hold drop length mismatch field information
static struct regfield_info c2h_drop_len_mismatch_field_info[] = {
    {"C2H_DROP_LEN_MISMATCH_RSVD_1", C2H_DROP_LEN_MISMATCH_RSVD_1_MASK},
    {"C2H_DROP_LEN_MISMATCH_CNT", C2H_DROP_LEN_MISMATCH_CNT_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_drop_len_mismatch_field_info\n");

// MD: Structure to hold drop descriptor response length field information
static struct regfield_info c2h_drop_desc_rsp_len_field_info[] = {
    {"C2H_DROP_DESC_RSP_LEN_RSVD_1", C2H_DROP_DESC_RSP_LEN_RSVD_1_MASK},
    {"C2H_DROP_DESC_RSP_LEN_CNT", C2H_DROP_DESC_RSP_LEN_CNT_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_drop_desc_rsp_len_field_info\n");

// MD: Structure to hold drop QID FIFO length field information
static struct regfield_info c2h_drop_qid_fifo_len_field_info[] = {
    {"C2H_DROP_QID_FIFO_LEN_RSVD_1", C2H_DROP_QID_FIFO_LEN_RSVD_1_MASK},
    {"C2H_DROP_QID_FIFO_LEN_CNT", C2H_DROP_QID_FIFO_LEN_CNT_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_drop_qid_fifo_len_field_info\n");

// MD: Structure to hold drop payload count field information
static struct regfield_info c2h_drop_pld_cnt_field_info[] = {
    {"C2H_DROP_PLD_CNT_RSVD_1", C2H_DROP_PLD_CNT_RSVD_1_MASK},
    {"C2H_DROP_PLD_CNT_CNT", C2H_DROP_PLD_CNT_CNT_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_drop_pld_cnt_field_info\n");

// MD: Structure to hold completion format 0 field information
static struct regfield_info c2h_cmpt_format_0_field_info[] = {
    {"C2H_CMPT_FORMAT_0_DESC_ERR_LOC", C2H_CMPT_FORMAT_0_DESC_ERR_LOC_MASK},
    {"C2H_CMPT_FORMAT_0_COLOR_LOC", C2H_CMPT_FORMAT_0_COLOR_LOC_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_cmpt_format_0_field_info\n");

// MD: Structure to hold completion format 1 field information
static struct regfield_info c2h_cmpt_format_1_field_info[] = {
    {"C2H_CMPT_FORMAT_1_DESC_ERR_LOC", C2H_CMPT_FORMAT_1_DESC_ERR_LOC_MASK},
    {"C2H_CMPT_FORMAT_1_COLOR_LOC", C2H_CMPT_FORMAT_1_COLOR_LOC_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_cmpt_format_1_field_info\n");

// MD: Structure to hold completion format 2 field information
static struct regfield_info c2h_cmpt_format_2_field_info[] = {
    {"C2H_CMPT_FORMAT_2_DESC_ERR_LOC", C2H_CMPT_FORMAT_2_DESC_ERR_LOC_MASK},
    {"C2H_CMPT_FORMAT_2_COLOR_LOC", C2H_CMPT_FORMAT_2_COLOR_LOC_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_cmpt_format_2_field_info\n");



// MD: Structure to hold completion format 3 field information
static struct regfield_info c2h_cmpt_format_3_field_info[] = {
    {"C2H_CMPT_FORMAT_3_DESC_ERR_LOC", C2H_CMPT_FORMAT_3_DESC_ERR_LOC_MASK},
    {"C2H_CMPT_FORMAT_3_COLOR_LOC", C2H_CMPT_FORMAT_3_COLOR_LOC_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_cmpt_format_3_field_info\n");

// MD: Structure to hold completion format 4 field information
static struct regfield_info c2h_cmpt_format_4_field_info[] = {
    {"C2H_CMPT_FORMAT_4_DESC_ERR_LOC", C2H_CMPT_FORMAT_4_DESC_ERR_LOC_MASK},
    {"C2H_CMPT_FORMAT_4_COLOR_LOC", C2H_CMPT_FORMAT_4_COLOR_LOC_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_cmpt_format_4_field_info\n");

// MD: Structure to hold completion format 5 field information
static struct regfield_info c2h_cmpt_format_5_field_info[] = {
    {"C2H_CMPT_FORMAT_5_DESC_ERR_LOC", C2H_CMPT_FORMAT_5_DESC_ERR_LOC_MASK},
    {"C2H_CMPT_FORMAT_5_COLOR_LOC", C2H_CMPT_FORMAT_5_COLOR_LOC_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_cmpt_format_5_field_info\n");

// MD: Structure to hold completion format 6 field information
static struct regfield_info c2h_cmpt_format_6_field_info[] = {
    {"C2H_CMPT_FORMAT_6_DESC_ERR_LOC", C2H_CMPT_FORMAT_6_DESC_ERR_LOC_MASK},
    {"C2H_CMPT_FORMAT_6_COLOR_LOC", C2H_CMPT_FORMAT_6_COLOR_LOC_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_cmpt_format_6_field_info\n");

// MD: Structure to hold prefetch cache depth field information
static struct regfield_info c2h_pfch_cache_depth_field_info[] = {
    {"C2H_PFCH_CACHE_DEPTH_MAX_STBUF", C2H_PFCH_CACHE_DEPTH_MAX_STBUF_MASK},
    {"C2H_PFCH_CACHE_DEPTH", C2H_PFCH_CACHE_DEPTH_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_pfch_cache_depth_field_info\n");

// MD: Structure to hold write-back coalescing buffer depth field information
static struct regfield_info c2h_wrb_coal_buf_depth_field_info[] = {
    {"C2H_WRB_COAL_BUF_DEPTH_RSVD_1", C2H_WRB_COAL_BUF_DEPTH_RSVD_1_MASK},
    {"C2H_WRB_COAL_BUF_DEPTH_BUFFER", C2H_WRB_COAL_BUF_DEPTH_BUFFER_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_wrb_coal_buf_depth_field_info\n");

// MD: Structure to hold prefetch credit field information
static struct regfield_info c2h_pfch_crdt_field_info[] = {
    {"C2H_PFCH_CRDT_RSVD_1", C2H_PFCH_CRDT_RSVD_1_MASK},
    {"C2H_PFCH_CRDT_RSVD_2", C2H_PFCH_CRDT_RSVD_2_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_pfch_crdt_field_info\n");

// MD: Structure to hold status has completion accepted field information
static struct regfield_info c2h_stat_has_cmpt_accepted_field_info[] = {
    {"C2H_STAT_HAS_CMPT_ACCEPTED_RSVD_1", C2H_STAT_HAS_CMPT_ACCEPTED_RSVD_1_MASK},
    {"C2H_STAT_HAS_CMPT_ACCEPTED_CNT", C2H_STAT_HAS_CMPT_ACCEPTED_CNT_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_stat_has_cmpt_accepted_field_info\n");

// MD: Structure to hold status has payload accepted field information
static struct regfield_info c2h_stat_has_pld_accepted_field_info[] = {
    {"C2H_STAT_HAS_PLD_ACCEPTED_RSVD_1", C2H_STAT_HAS_PLD_ACCEPTED_RSVD_1_MASK},
    {"C2H_STAT_HAS_PLD_ACCEPTED_CNT", C2H_STAT_HAS_PLD_ACCEPTED_CNT_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_stat_has_pld_accepted_field_info\n");

// MD: Structure to hold payload packet ID field information
static struct regfield_info c2h_pld_pkt_id_field_info[] = {
    {"C2H_PLD_PKT_ID_CMPT_WAIT", C2H_PLD_PKT_ID_CMPT_WAIT_MASK},
    {"C2H_PLD_PKT_ID_DATA", C2H_PLD_PKT_ID_DATA_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_pld_pkt_id_field_info\n");

// MD: Structure to hold payload packet ID 1 field information
static struct regfield_info c2h_pld_pkt_id_1_field_info[] = {
    {"C2H_PLD_PKT_ID_1_CMPT_WAIT", C2H_PLD_PKT_ID_1_CMPT_WAIT_MASK},
    {"C2H_PLD_PKT_ID_1_DATA", C2H_PLD_PKT_ID_1_DATA_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_pld_pkt_id_1_field_info\n");

// MD: Structure to hold drop payload count 1 field information
static struct regfield_info c2h_drop_pld_cnt_1_field_info[] = {
    {"C2H_DROP_PLD_CNT_1_RSVD_1", C2H_DROP_PLD_CNT_1_RSVD_1_MASK},
    {"C2H_DROP_PLD_CNT_1_CNT", C2H_DROP_PLD_CNT_1_CNT_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_drop_pld_cnt_1_field_info\n");

// MD: Structure to hold H2C error status field information
static struct regfield_info h2c_err_stat_field_info[] = {
    {"H2C_ERR_STAT_RSVD_1", H2C_ERR_STAT_RSVD_1_MASK},
    {"H2C_ERR_STAT_PAR_ERR", H2C_ERR_STAT_PAR_ERR_MASK},
    {"H2C_ERR_STAT_SBE", H2C_ERR_STAT_SBE_MASK},
    {"H2C_ERR_STAT_DBE", H2C_ERR_STAT_DBE_MASK},
    {"H2C_ERR_STAT_NO_DMA_DS", H2C_ERR_STAT_NO_DMA_DS_MASK},
    {"H2C_ERR_STAT_SDI_MRKR_REQ_MOP_ERR", H2C_ERR_STAT_SDI_MRKR_REQ_MOP_ERR_MASK},
    {"H2C_ERR_STAT_ZERO_LEN_DS", H2C_ERR_STAT_ZERO_LEN_DS_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized h2c_err_stat_field_info\n");

// MD: Structure to hold H2C error mask field information
static struct regfield_info h2c_err_mask_field_info[] = {
    {"H2C_ERR_EN", H2C_ERR_EN_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized h2c_err_mask_field_info\n");

// MD: Structure to hold first error QID field information for H2C
static struct regfield_info h2c_first_err_qid_field_info[] = {
    {"H2C_FIRST_ERR_QID_RSVD_1", H2C_FIRST_ERR_QID_RSVD_1_MASK},
    {"H2C_FIRST_ERR_QID_ERR_TYPE", H2C_FIRST_ERR_QID_ERR_TYPE_MASK},
    {"H2C_FIRST_ERR_QID_RSVD_2", H2C_FIRST_ERR_QID_RSVD_2_MASK},
    {"H2C_FIRST_ERR_QID_QID", H2C_FIRST_ERR_QID_QID_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized h2c_first_err_qid_field_info\n");



// MD: Structure to hold H2C debug register 0 field information
static struct regfield_info h2c_dbg_reg0_field_info[] = {
    {"H2C_REG0_NUM_DSC_RCVD", H2C_REG0_NUM_DSC_RCVD_MASK},
    {"H2C_REG0_NUM_WRB_SENT", H2C_REG0_NUM_WRB_SENT_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized h2c_dbg_reg0_field_info\n");

// MD: Structure to hold H2C debug register 1 field information
static struct regfield_info h2c_dbg_reg1_field_info[] = {
    {"H2C_REG1_NUM_REQ_SENT", H2C_REG1_NUM_REQ_SENT_MASK},
    {"H2C_REG1_NUM_CMP_SENT", H2C_REG1_NUM_CMP_SENT_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized h2c_dbg_reg1_field_info\n");

// MD: Structure to hold H2C debug register 2 field information
static struct regfield_info h2c_dbg_reg2_field_info[] = {
    {"H2C_REG2_RSVD_1", H2C_REG2_RSVD_1_MASK},
    {"H2C_REG2_NUM_ERR_DSC_RCVD", H2C_REG2_NUM_ERR_DSC_RCVD_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized h2c_dbg_reg2_field_info\n");

// MD: Structure to hold H2C debug register 3 field information
static struct regfield_info h2c_dbg_reg3_field_info[] = {
    {"H2C_REG3_RSVD_1", H2C_REG3_RSVD_1_MASK},
    {"H2C_REG3_DSCO_FIFO_EMPTY", H2C_REG3_DSCO_FIFO_EMPTY_MASK},
    {"H2C_REG3_DSCO_FIFO_FULL", H2C_REG3_DSCO_FIFO_FULL_MASK},
    {"H2C_REG3_CUR_RC_STATE", H2C_REG3_CUR_RC_STATE_MASK},
    {"H2C_REG3_RDREQ_LINES", H2C_REG3_RDREQ_LINES_MASK},
    {"H2C_REG3_RDATA_LINES_AVAIL", H2C_REG3_RDATA_LINES_AVAIL_MASK},
    {"H2C_REG3_PEND_FIFO_EMPTY", H2C_REG3_PEND_FIFO_EMPTY_MASK},
    {"H2C_REG3_PEND_FIFO_FULL", H2C_REG3_PEND_FIFO_FULL_MASK},
    {"H2C_REG3_CUR_RQ_STATE", H2C_REG3_CUR_RQ_STATE_MASK},
    {"H2C_REG3_DSCI_FIFO_FULL", H2C_REG3_DSCI_FIFO_FULL_MASK},
    {"H2C_REG3_DSCI_FIFO_EMPTY", H2C_REG3_DSCI_FIFO_EMPTY_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized h2c_dbg_reg3_field_info\n");

// MD: Structure to hold H2C debug register 4 field information
static struct regfield_info h2c_dbg_reg4_field_info[] = {
    {"H2C_REG4_RDREQ_ADDR", H2C_REG4_RDREQ_ADDR_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized h2c_dbg_reg4_field_info\n");

// MD: Structure to hold H2C fatal error enable field information
static struct regfield_info h2c_fatal_err_en_field_info[] = {
    {"H2C_FATAL_ERR_EN_RSVD_1", H2C_FATAL_ERR_EN_RSVD_1_MASK},
    {"H2C_FATAL_ERR_EN_H2C", H2C_FATAL_ERR_EN_H2C_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized h2c_fatal_err_en_field_info\n");

// MD: Structure to hold H2C request throttle PCIe field information
static struct regfield_info h2c_req_throt_pcie_field_info[] = {
    {"H2C_REQ_THROT_PCIE_EN_REQ", H2C_REQ_THROT_PCIE_EN_REQ_MASK},
    {"H2C_REQ_THROT_PCIE", H2C_REQ_THROT_PCIE_MASK},
    {"H2C_REQ_THROT_PCIE_EN_DATA", H2C_REQ_THROT_PCIE_EN_DATA_MASK},
    {"H2C_REQ_THROT_PCIE_DATA_THRESH", H2C_REQ_THROT_PCIE_DATA_THRESH_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized h2c_req_throt_pcie_field_info\n");

// MD: Structure to hold H2C alignment debug register 0 field information
static struct regfield_info h2c_aln_dbg_reg0_field_info[] = {
    {"H2C_ALN_REG0_NUM_PKT_SENT", H2C_ALN_REG0_NUM_PKT_SENT_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized h2c_aln_dbg_reg0_field_info\n");

// MD: Structure to hold H2C request throttle AXI-MM field information
static struct regfield_info h2c_req_throt_aximm_field_info[] = {
    {"H2C_REQ_THROT_AXIMM_EN_REQ", H2C_REQ_THROT_AXIMM_EN_REQ_MASK},
    {"H2C_REQ_THROT_AXIMM", H2C_REQ_THROT_AXIMM_MASK},
    {"H2C_REQ_THROT_AXIMM_EN_DATA", H2C_REQ_THROT_AXIMM_EN_DATA_MASK},
    {"H2C_REQ_THROT_AXIMM_DATA_THRESH", H2C_REQ_THROT_AXIMM_DATA_THRESH_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized h2c_req_throt_aximm_field_info\n");

// MD: Structure to hold C2H MM control field information
static struct regfield_info c2h_mm_ctl_field_info[] = {
    {"C2H_MM_CTL_RESERVED1", C2H_MM_CTL_RESERVED1_MASK},
    {"C2H_MM_CTL_ERRC_EN", C2H_MM_CTL_ERRC_EN_MASK},
    {"C2H_MM_CTL_RESERVED0", C2H_MM_CTL_RESERVED0_MASK},
    {"C2H_MM_CTL_RUN", C2H_MM_CTL_RUN_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_mm_ctl_field_info\n");

// MD: Structure to hold C2H MM status field information
static struct regfield_info c2h_mm_status_field_info[] = {
    {"C2H_MM_STATUS_RSVD_1", C2H_MM_STATUS_RSVD_1_MASK},
    {"C2H_MM_STATUS_RUN", C2H_MM_STATUS_RUN_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_mm_status_field_info\n");

// MD: Structure to hold C2H MM completion descriptor count field information
static struct regfield_info c2h_mm_cmpl_desc_cnt_field_info[] = {
    {"C2H_MM_CMPL_DESC_CNT_C2H_CO", C2H_MM_CMPL_DESC_CNT_C2H_CO_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_mm_cmpl_desc_cnt_field_info\n");

// MD: Structure to hold C2H MM error code enable mask field information
static struct regfield_info c2h_mm_err_code_enable_mask_field_info[] = {
    {"C2H_MM_ERR_CODE_ENABLE_RESERVED1", C2H_MM_ERR_CODE_ENABLE_RESERVED1_MASK},
    {"C2H_MM_ERR_CODE_ENABLE_WR_UC_RAM", C2H_MM_ERR_CODE_ENABLE_WR_UC_RAM_MASK},
    {"C2H_MM_ERR_CODE_ENABLE_WR_UR", C2H_MM_ERR_CODE_ENABLE_WR_UR_MASK},
    {"C2H_MM_ERR_CODE_ENABLE_WR_FLR", C2H_MM_ERR_CODE_ENABLE_WR_FLR_MASK},
    {"C2H_MM_ERR_CODE_ENABLE_RESERVED0", C2H_MM_ERR_CODE_ENABLE_RESERVED0_MASK},
    {"C2H_MM_ERR_CODE_ENABLE_RD_SLV_ERR", C2H_MM_ERR_CODE_ENABLE_RD_SLV_ERR_MASK},
    {"C2H_MM_ERR_CODE_ENABLE_WR_SLV_ERR", C2H_MM_ERR_CODE_ENABLE_WR_SLV_ERR_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_mm_err_code_enable_mask_field_info\n");

// MD: Structure to hold C2H MM error code field information
static struct regfield_info c2h_mm_err_code_field_info[] = {
    {"C2H_MM_ERR_CODE_RESERVED1", C2H_MM_ERR_CODE_RESERVED1_MASK},
    {"C2H_MM_ERR_CODE_CIDX", C2H_MM_ERR_CODE_CIDX_MASK},
    {"C2H_MM_ERR_CODE_RESERVED0", C2H_MM_ERR_CODE_RESERVED0_MASK},
    {"C2H_MM_ERR_CODE_SUB_TYPE", C2H_MM_ERR_CODE_SUB_TYPE_MASK},
    {"C2H_MM_ERR_CODE", C2H_MM_ERR_CODE_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_mm_err_code_field_info\n");



// MD: Structure to hold C2H MM error information field information
static struct regfield_info c2h_mm_err_info_field_info[] = {
    {"C2H_MM_ERR_INFO_VALID", C2H_MM_ERR_INFO_VALID_MASK},
    {"C2H_MM_ERR_INFO_SEL", C2H_MM_ERR_INFO_SEL_MASK},
    {"C2H_MM_ERR_INFO_RSVD_1", C2H_MM_ERR_INFO_RSVD_1_MASK},
    {"C2H_MM_ERR_INFO_QID", C2H_MM_ERR_INFO_QID_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_mm_err_info_field_info\n");

// MD: Structure to hold C2H MM performance monitor control field information
static struct regfield_info c2h_mm_perf_mon_ctl_field_info[] = {
    {"C2H_MM_PERF_MON_CTL_RSVD_1", C2H_MM_PERF_MON_CTL_RSVD_1_MASK},
    {"C2H_MM_PERF_MON_CTL_IMM_START", C2H_MM_PERF_MON_CTL_IMM_START_MASK},
    {"C2H_MM_PERF_MON_CTL_RUN_START", C2H_MM_PERF_MON_CTL_RUN_START_MASK},
    {"C2H_MM_PERF_MON_CTL_IMM_CLEAR", C2H_MM_PERF_MON_CTL_IMM_CLEAR_MASK},
    {"C2H_MM_PERF_MON_CTL_RUN_CLEAR", C2H_MM_PERF_MON_CTL_RUN_CLEAR_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_mm_perf_mon_ctl_field_info\n");

// MD: Structure to hold C2H MM performance monitor cycle count 0 field information
static struct regfield_info c2h_mm_perf_mon_cycle_cnt0_field_info[] = {
    {"C2H_MM_PERF_MON_CYCLE_CNT0_CYC_CNT", C2H_MM_PERF_MON_CYCLE_CNT0_CYC_CNT_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_mm_perf_mon_cycle_cnt0_field_info\n");

// MD: Structure to hold C2H MM performance monitor cycle count 1 field information
static struct regfield_info c2h_mm_perf_mon_cycle_cnt1_field_info[] = {
    {"C2H_MM_PERF_MON_CYCLE_CNT1_RSVD_1", C2H_MM_PERF_MON_CYCLE_CNT1_RSVD_1_MASK},
    {"C2H_MM_PERF_MON_CYCLE_CNT1_CYC_CNT", C2H_MM_PERF_MON_CYCLE_CNT1_CYC_CNT_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_mm_perf_mon_cycle_cnt1_field_info\n");

// MD: Structure to hold C2H MM performance monitor data count 0 field information
static struct regfield_info c2h_mm_perf_mon_data_cnt0_field_info[] = {
    {"C2H_MM_PERF_MON_DATA_CNT0_DCNT", C2H_MM_PERF_MON_DATA_CNT0_DCNT_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_mm_perf_mon_data_cnt0_field_info\n");

// MD: Structure to hold C2H MM performance monitor data count 1 field information
static struct regfield_info c2h_mm_perf_mon_data_cnt1_field_info[] = {
    {"C2H_MM_PERF_MON_DATA_CNT1_RSVD_1", C2H_MM_PERF_MON_DATA_CNT1_RSVD_1_MASK},
    {"C2H_MM_PERF_MON_DATA_CNT1_DCNT", C2H_MM_PERF_MON_DATA_CNT1_DCNT_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_mm_perf_mon_data_cnt1_field_info\n");

// MD: Structure to hold C2H MM debug field information
static struct regfield_info c2h_mm_dbg_field_info[] = {
    {"C2H_MM_RSVD_1", C2H_MM_RSVD_1_MASK},
    {"C2H_MM_RRQ_ENTRIES", C2H_MM_RRQ_ENTRIES_MASK},
    {"C2H_MM_DAT_FIFO_SPC", C2H_MM_DAT_FIFO_SPC_MASK},
    {"C2H_MM_RD_STALL", C2H_MM_RD_STALL_MASK},
    {"C2H_MM_RRQ_FIFO_FI", C2H_MM_RRQ_FIFO_FI_MASK},
    {"C2H_MM_WR_STALL", C2H_MM_WR_STALL_MASK},
    {"C2H_MM_WRQ_FIFO_FI", C2H_MM_WRQ_FIFO_FI_MASK},
    {"C2H_MM_WBK_STALL", C2H_MM_WBK_STALL_MASK},
    {"C2H_MM_DSC_FIFO_EP", C2H_MM_DSC_FIFO_EP_MASK},
    {"C2H_MM_DSC_FIFO_FL", C2H_MM_DSC_FIFO_FL_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_mm_dbg_field_info\n");

// MD: Structure to hold H2C MM control field information
static struct regfield_info h2c_mm_ctl_field_info[] = {
    {"H2C_MM_CTL_RESERVED1", H2C_MM_CTL_RESERVED1_MASK},
    {"H2C_MM_CTL_ERRC_EN", H2C_MM_CTL_ERRC_EN_MASK},
    {"H2C_MM_CTL_RESERVED0", H2C_MM_CTL_RESERVED0_MASK},
    {"H2C_MM_CTL_RUN", H2C_MM_CTL_RUN_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized h2c_mm_ctl_field_info\n");

// MD: Structure to hold H2C MM status field information
static struct regfield_info h2c_mm_status_field_info[] = {
    {"H2C_MM_STATUS_RSVD_1", H2C_MM_STATUS_RSVD_1_MASK},
    {"H2C_MM_STATUS_RUN", H2C_MM_STATUS_RUN_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized h2c_mm_status_field_info\n");

// MD: Structure to hold H2C MM completion descriptor count field information
static struct regfield_info h2c_mm_cmpl_desc_cnt_field_info[] = {
    {"H2C_MM_CMPL_DESC_CNT_H2C_CO", H2C_MM_CMPL_DESC_CNT_H2C_CO_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized h2c_mm_cmpl_desc_cnt_field_info\n");

// MD: Structure to hold H2C MM error code enable mask field information
static struct regfield_info h2c_mm_err_code_enable_mask_field_info[] = {
    {"H2C_MM_ERR_CODE_ENABLE_RESERVED5", H2C_MM_ERR_CODE_ENABLE_RESERVED5_MASK},
    {"H2C_MM_ERR_CODE_ENABLE_WR_SLV_ERR", H2C_MM_ERR_CODE_ENABLE_WR_SLV_ERR_MASK},
    {"H2C_MM_ERR_CODE_ENABLE_WR_DEC_ERR", H2C_MM_ERR_CODE_ENABLE_WR_DEC_ERR_MASK},
    {"H2C_MM_ERR_CODE_ENABLE_RESERVED4", H2C_MM_ERR_CODE_ENABLE_RESERVED4_MASK},
    {"H2C_MM_ERR_CODE_ENABLE_RD_RQ_DIS_ERR", H2C_MM_ERR_CODE_ENABLE_RD_RQ_DIS_ERR_MASK},
    {"H2C_MM_ERR_CODE_ENABLE_RESERVED3", H2C_MM_ERR_CODE_ENABLE_RESERVED3_MASK},
    {"H2C_MM_ERR_CODE_ENABLE_RD_DAT_POISON_ERR", H2C_MM_ERR_CODE_ENABLE_RD_DAT_POISON_ERR_MASK},
    {"H2C_MM_ERR_CODE_ENABLE_RESERVED2", H2C_MM_ERR_CODE_ENABLE_RESERVED2_MASK},
    {"H2C_MM_ERR_CODE_ENABLE_RD_FLR_ERR", H2C_MM_ERR_CODE_ENABLE_RD_FLR_ERR_MASK},
    {"H2C_MM_ERR_CODE_ENABLE_RESERVED1", H2C_MM_ERR_CODE_ENABLE_RESERVED1_MASK},
    {"H2C_MM_ERR_CODE_ENABLE_RD_HDR_ADR_ERR", H2C_MM_ERR_CODE_ENABLE_RD_HDR_ADR_ERR_MASK},
    {"H2C_MM_ERR_CODE_ENABLE_RD_HDR_PARA", H2C_MM_ERR_CODE_ENABLE_RD_HDR_PARA_MASK},
    {"H2C_MM_ERR_CODE_ENABLE_RD_HDR_BYTE_ERR", H2C_MM_ERR_CODE_ENABLE_RD_HDR_BYTE_ERR_MASK},
    {"H2C_MM_ERR_CODE_ENABLE_RD_UR_CA", H2C_MM_ERR_CODE_ENABLE_RD_UR_CA_MASK},
    {"H2C_MM_ERR_CODE_ENABLE_RD_HRD_POISON_ERR", H2C_MM_ERR_CODE_ENABLE_RD_HRD_POISON_ERR_MASK},
    {"H2C_MM_ERR_CODE_ENABLE_RESERVED0", H2C_MM_ERR_CODE_ENABLE_RESERVED0_MASK},
    
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized h2c_mm_err_code_enable_mask_field_info\n");



// MD: Structure to hold H2C MM error code field information
static struct regfield_info h2c_mm_err_code_field_info[] = {
    {"H2C_MM_ERR_CODE_RSVD_1", H2C_MM_ERR_CODE_RSVD_1_MASK},
    {"H2C_MM_ERR_CODE_CIDX", H2C_MM_ERR_CODE_CIDX_MASK},
    {"H2C_MM_ERR_CODE_RESERVED0", H2C_MM_ERR_CODE_RESERVED0_MASK},
    {"H2C_MM_ERR_CODE_SUB_TYPE", H2C_MM_ERR_CODE_SUB_TYPE_MASK},
    {"H2C_MM_ERR_CODE", H2C_MM_ERR_CODE_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized h2c_mm_err_code_field_info\n");

// MD: Structure to hold H2C MM error information field information
static struct regfield_info h2c_mm_err_info_field_info[] = {
    {"H2C_MM_ERR_INFO_VALID", H2C_MM_ERR_INFO_VALID_MASK},
    {"H2C_MM_ERR_INFO_SEL", H2C_MM_ERR_INFO_SEL_MASK},
    {"H2C_MM_ERR_INFO_RSVD_1", H2C_MM_ERR_INFO_RSVD_1_MASK},
    {"H2C_MM_ERR_INFO_QID", H2C_MM_ERR_INFO_QID_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized h2c_mm_err_info_field_info\n");

// MD: Structure to hold H2C MM performance monitor control field information
static struct regfield_info h2c_mm_perf_mon_ctl_field_info[] = {
    {"H2C_MM_PERF_MON_CTL_RSVD_1", H2C_MM_PERF_MON_CTL_RSVD_1_MASK},
    {"H2C_MM_PERF_MON_CTL_IMM_START", H2C_MM_PERF_MON_CTL_IMM_START_MASK},
    {"H2C_MM_PERF_MON_CTL_RUN_START", H2C_MM_PERF_MON_CTL_RUN_START_MASK},
    {"H2C_MM_PERF_MON_CTL_IMM_CLEAR", H2C_MM_PERF_MON_CTL_IMM_CLEAR_MASK},
    {"H2C_MM_PERF_MON_CTL_RUN_CLEAR", H2C_MM_PERF_MON_CTL_RUN_CLEAR_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized h2c_mm_perf_mon_ctl_field_info\n");

// MD: Structure to hold H2C MM performance monitor cycle count 0 field information
static struct regfield_info h2c_mm_perf_mon_cycle_cnt0_field_info[] = {
    {"H2C_MM_PERF_MON_CYCLE_CNT0_CYC_CNT", H2C_MM_PERF_MON_CYCLE_CNT0_CYC_CNT_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized h2c_mm_perf_mon_cycle_cnt0_field_info\n");

// MD: Structure to hold H2C MM performance monitor cycle count 1 field information
static struct regfield_info h2c_mm_perf_mon_cycle_cnt1_field_info[] = {
    {"H2C_MM_PERF_MON_CYCLE_CNT1_RSVD_1", H2C_MM_PERF_MON_CYCLE_CNT1_RSVD_1_MASK},
    {"H2C_MM_PERF_MON_CYCLE_CNT1_CYC_CNT", H2C_MM_PERF_MON_CYCLE_CNT1_CYC_CNT_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized h2c_mm_perf_mon_cycle_cnt1_field_info\n");

// MD: Structure to hold H2C MM performance monitor data count 0 field information
static struct regfield_info h2c_mm_perf_mon_data_cnt0_field_info[] = {
    {"H2C_MM_PERF_MON_DATA_CNT0_DCNT", H2C_MM_PERF_MON_DATA_CNT0_DCNT_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized h2c_mm_perf_mon_data_cnt0_field_info\n");

// MD: Structure to hold H2C MM performance monitor data count 1 field information
static struct regfield_info h2c_mm_perf_mon_data_cnt1_field_info[] = {
    {"H2C_MM_PERF_MON_DATA_CNT1_RSVD_1", H2C_MM_PERF_MON_DATA_CNT1_RSVD_1_MASK},
    {"H2C_MM_PERF_MON_DATA_CNT1_DCNT", H2C_MM_PERF_MON_DATA_CNT1_DCNT_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized h2c_mm_perf_mon_data_cnt1_field_info\n");

// MD: Structure to hold H2C MM debug field information
static struct regfield_info h2c_mm_dbg_field_info[] = {
    {"H2C_MM_RSVD_1", H2C_MM_RSVD_1_MASK},
    {"H2C_MM_RRQ_ENTRIES", H2C_MM_RRQ_ENTRIES_MASK},
    {"H2C_MM_DAT_FIFO_SPC", H2C_MM_DAT_FIFO_SPC_MASK},
    {"H2C_MM_RD_STALL", H2C_MM_RD_STALL_MASK},
    {"H2C_MM_RRQ_FIFO_FI", H2C_MM_RRQ_FIFO_FI_MASK},
    {"H2C_MM_WR_STALL", H2C_MM_WR_STALL_MASK},
    {"H2C_MM_WRQ_FIFO_FI", H2C_MM_WRQ_FIFO_FI_MASK},
    {"H2C_MM_WBK_STALL", H2C_MM_WBK_STALL_MASK},
    {"H2C_MM_DSC_FIFO_EP", H2C_MM_DSC_FIFO_EP_MASK},
    {"H2C_MM_DSC_FIFO_FL", H2C_MM_DSC_FIFO_FL_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized h2c_mm_dbg_field_info\n");

// MD: Structure to hold H2C MM data throttle field information
static struct regfield_info h2c_mm_data_throttle_field_info[] = {
    {"H2C_MM_DATA_THROTTLE_RSVD_1", H2C_MM_DATA_THROTTLE_RSVD_1_MASK},
    {"H2C_MM_DATA_THROTTLE_DAT_EN", H2C_MM_DATA_THROTTLE_DAT_EN_MASK},
    {"H2C_MM_DATA_THROTTLE_DAT", H2C_MM_DATA_THROTTLE_DAT_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized h2c_mm_data_throttle_field_info\n");

// MD: Structure to hold C2H credit coalescing configuration 1 field information
static struct regfield_info c2h_crdt_coal_cfg_1_field_info[] = {
    {"C2H_CRDT_COAL_CFG_1_RSVD_1", C2H_CRDT_COAL_CFG_1_RSVD_1_MASK},
    {"C2H_CRDT_COAL_CFG_1_PLD_FIFO_TH", C2H_CRDT_COAL_CFG_1_PLD_FIFO_TH_MASK},
    {"C2H_CRDT_COAL_CFG_1_TIMER_TH", C2H_CRDT_COAL_CFG_1_TIMER_TH_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_crdt_coal_cfg_1_field_info\n");

// MD: Structure to hold C2H credit coalescing configuration 2 field information
static struct regfield_info c2h_crdt_coal_cfg_2_field_info[] = {
    {"C2H_CRDT_COAL_CFG_2_RSVD_1", C2H_CRDT_COAL_CFG_2_RSVD_1_MASK},
    {"C2H_CRDT_COAL_CFG_2_FIFO_TH", C2H_CRDT_COAL_CFG_2_FIFO_TH_MASK},
    {"C2H_CRDT_COAL_CFG_2_RESERVED1", C2H_CRDT_COAL_CFG_2_RESERVED1_MASK},
    {"C2H_CRDT_COAL_CFG_2_NT_TH", C2H_CRDT_COAL_CFG_2_NT_TH_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_crdt_coal_cfg_2_field_info\n");

// MD: Structure to hold C2H prefetch bypass QID field information
static struct regfield_info c2h_pfch_byp_qid_field_info[] = {
    {"C2H_PFCH_BYP_QID_RSVD_1", C2H_PFCH_BYP_QID_RSVD_1_MASK},
    {"C2H_PFCH_BYP_QID", C2H_PFCH_BYP_QID_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_pfch_byp_qid_field_info\n");

// MD: Structure to hold C2H prefetch bypass tag field information
static struct regfield_info c2h_pfch_byp_tag_field_info[] = {
    {"C2H_PFCH_BYP_TAG_RSVD_1", C2H_PFCH_BYP_TAG_RSVD_1_MASK},
    {"C2H_PFCH_BYP_TAG_BYP_QID", C2H_PFCH_BYP_TAG_BYP_QID_MASK},
    {"C2H_PFCH_BYP_TAG_RSVD_2", C2H_PFCH_BYP_TAG_RSVD_2_MASK},
    {"C2H_PFCH_BYP_TAG", C2H_PFCH_BYP_TAG_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_pfch_byp_tag_field_info\n");



// MD: Structure to hold C2H water mark field information
static struct regfield_info c2h_water_mark_field_info[] = {
    {"C2H_WATER_MARK_HIGH_WM", C2H_WATER_MARK_HIGH_WM_MASK},
    {"C2H_WATER_MARK_LOW_WM", C2H_WATER_MARK_LOW_WM_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_water_mark_field_info\n");

// MD: Structure to hold C2H notify empty field information
static struct regfield_info c2h_notify_empty_field_info[] = {
    {"C2H_NOTIFY_EMPTY_RSVD_1", C2H_NOTIFY_EMPTY_RSVD_1_MASK},
    {"C2H_NOTIFY_EMPTY_NOE", C2H_NOTIFY_EMPTY_NOE_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_notify_empty_field_info\n");

// MD: Structure to hold C2H S_AXIS C2H accepted 1 field information
static struct regfield_info c2h_stat_s_axis_c2h_accepted_1_field_info[] = {
    {"C2H_STAT_S_AXIS_C2H_ACCEPTED_1", C2H_STAT_S_AXIS_C2H_ACCEPTED_1_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_stat_s_axis_c2h_accepted_1_field_info\n");

// MD: Structure to hold C2H S_AXIS WRB accepted 1 field information
static struct regfield_info c2h_stat_s_axis_wrb_accepted_1_field_info[] = {
    {"C2H_STAT_S_AXIS_WRB_ACCEPTED_1", C2H_STAT_S_AXIS_WRB_ACCEPTED_1_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_stat_s_axis_wrb_accepted_1_field_info\n");

// MD: Structure to hold C2H descriptor response packet accepted 1 field information
static struct regfield_info c2h_stat_desc_rsp_pkt_accepted_1_field_info[] = {
    {"C2H_STAT_DESC_RSP_PKT_ACCEPTED_1_D", C2H_STAT_DESC_RSP_PKT_ACCEPTED_1_D_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_stat_desc_rsp_pkt_accepted_1_field_info\n");

// MD: Structure to hold C2H axis package completion 1 field information
static struct regfield_info c2h_stat_axis_pkg_cmp_1_field_info[] = {
    {"C2H_STAT_AXIS_PKG_CMP_1", C2H_STAT_AXIS_PKG_CMP_1_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_stat_axis_pkg_cmp_1_field_info\n");

// MD: Structure to hold C2H S_AXIS WRB accepted 2 field information
static struct regfield_info c2h_stat_s_axis_wrb_accepted_2_field_info[] = {
    {"C2H_STAT_S_AXIS_WRB_ACCEPTED_2", C2H_STAT_S_AXIS_WRB_ACCEPTED_2_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_stat_s_axis_wrb_accepted_2_field_info\n");

// MD: Structure to hold C2H ST payload FIFO depth field information
static struct regfield_info c2h_st_pld_fifo_depth_field_info[] = {
    {"C2H_ST_PLD_FIFO_DEPTH", C2H_ST_PLD_FIFO_DEPTH_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_st_pld_fifo_depth_field_info\n");

// MD: Structure to hold C2H DMA engine 6 debug field information
static struct regfield_info c2h_stat_dbg_dma_eng_6_field_info[] = {
    {"C2H_STAT_DMA_ENG_6_RSVD", C2H_STAT_DMA_ENG_6_RSVD_MASK},
    {"C2H_STAT_DMA_ENG_6_PLD_ST_FIFO_OUT_DATA_QID", C2H_STAT_DMA_ENG_6_PLD_ST_FIFO_OUT_DATA_QID_MASK},
    {"C2H_STAT_DMA_ENG_6_PLD_STS_FIFO_OUT_DATA_PLD_ST_PKT_ID", C2H_STAT_DMA_ENG_6_PLD_STS_FIFO_OUT_DATA_PLD_ST_PKT_ID_MASK},
    {"C2H_STAT_DMA_ENG_6_PLD_PKT_ID_LARGER_PLD_ST", C2H_STAT_DMA_ENG_6_PLD_PKT_ID_LARGER_PLD_ST_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_stat_dbg_dma_eng_6_field_info\n");

// MD: Structure to hold C2H DMA engine 7 debug field information
static struct regfield_info c2h_stat_dbg_dma_eng_7_field_info[] = {
    {"C2H_STAT_DMA_ENG_7_RSVD", C2H_STAT_DMA_ENG_7_RSVD_MASK},
    {"C2H_STAT_DMA_ENG_7_PLD_ST_FIFO_OUT_DATA_QID_1", C2H_STAT_DMA_ENG_7_PLD_ST_FIFO_OUT_DATA_QID_1_MASK},
    {"C2H_STAT_DMA_ENG_7_PLD_STS_FIFO_OUT_DATA_PLD_ST_PKT_ID_1", C2H_STAT_DMA_ENG_7_PLD_STS_FIFO_OUT_DATA_PLD_ST_PKT_ID_1_MASK},
    {"C2H_STAT_DMA_ENG_7_PLD_PKT_ID_LARGER_PLD_ST_1", C2H_STAT_DMA_ENG_7_PLD_PKT_ID_LARGER_PLD_ST_1_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_stat_dbg_dma_eng_7_field_info\n");

// MD: Structure to hold C2H PCIe completion 1 field information
static struct regfield_info c2h_stat_pcie_cmp_1_field_info[] = {
    {"C2H_STAT_PCIE_CMP_1_DEPTH", C2H_STAT_PCIE_CMP_1_DEPTH_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_stat_pcie_cmp_1_field_info\n");

// MD: Structure to hold C2H payload FIFO almost full field information
static struct regfield_info c2h_pld_fifo_almost_full_field_info[] = {
    {"C2H_PLD_FIFO_ALMOST_FULL_ENABLE", C2H_PLD_FIFO_ALMOST_FULL_ENABLE_MASK},
    {"C2H_PLD_FIFO_ALMOST_FULL_TH", C2H_PLD_FIFO_ALMOST_FULL_TH_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized c2h_pld_fifo_almost_full_field_info\n");

// MD: Structure to hold prefetch configuration 3 field information
static struct regfield_info pfch_cfg_3_field_info[] = {
    {"PFCH_CFG_3_RSVD", PFCH_CFG_3_RSVD_MASK},
    {"PFCH_CFG_3_VAR_DESC_FL_FREE_CNT_TH", PFCH_CFG_3_VAR_DESC_FL_FREE_CNT_TH_MASK},
    {"PFCH_CFG_3_VAR_DESC_LG_PKT_CAM_CN_TH", PFCH_CFG_3_VAR_DESC_LG_PKT_CAM_CN_TH_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized pfch_cfg_3_field_info\n");

// MD: Structure to hold completion configuration 0 field information
static struct regfield_info cmpt_cfg_0_field_info[] = {
    {"CMPT_CFG_0_RSVD", CMPT_CFG_0_RSVD_MASK},
    {"CMPT_CFG_0_VIO_SPRS_INT_AFTER_RTY", CMPT_CFG_0_VIO_SPRS_INT_AFTER_RTY_MASK},
    {"CMPT_CFG_0_VIO_EVNT_SUP_EN", CMPT_CFG_0_VIO_EVNT_SUP_EN_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized cmpt_cfg_0_field_info\n");

// MD: Structure to hold prefetch configuration 4 field information
static struct regfield_info pfch_cfg_4_field_info[] = {
    {"PFCH_CFG_4_GLB_EVT_TIMER_TICK", PFCH_CFG_4_GLB_EVT_TIMER_TICK_MASK},
    {"PFCH_CFG_4_DISABLE_GLB_EVT_TIMER", PFCH_CFG_4_DISABLE_GLB_EVT_TIMER_MASK},
    {"PFCH_CFG_4_EVT_TIMER_TICK", PFCH_CFG_4_EVT_TIMER_TICK_MASK},
    {"PFCH_CFG_4_DISABLE_EVT_TIMER", PFCH_CFG_4_DISABLE_EVT_TIMER_MASK},
    // MD: Add more fields if necessary
};

// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized pfch_cfg_4_field_info\n");

// MD: Structure to hold EQDMA configuration registers information

static struct xreg_info eqdma_config_regs[] = {
{"CFG_BLK_IDENTIFIER", 0x00,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(cfg_blk_identifier_field_info),
	cfg_blk_identifier_field_info
},
{"CFG_BLK_PCIE_MAX_PLD_SIZE", 0x08,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(cfg_blk_pcie_max_pld_size_field_info),
	cfg_blk_pcie_max_pld_size_field_info
},
{"CFG_BLK_PCIE_MAX_READ_REQ_SIZE", 0x0c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(cfg_blk_pcie_max_read_req_size_field_info),
	cfg_blk_pcie_max_read_req_size_field_info
},
{"CFG_BLK_SYSTEM_ID", 0x10,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(cfg_blk_system_id_field_info),
	cfg_blk_system_id_field_info
},
{"CFG_BLK_MSIX_ENABLE", 0x014,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(cfg_blk_msix_enable_field_info),
	cfg_blk_msix_enable_field_info
},
{"CFG_PCIE_DATA_WIDTH", 0x18,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(cfg_pcie_data_width_field_info),
	cfg_pcie_data_width_field_info
},
{"CFG_PCIE_CTL", 0x1c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(cfg_pcie_ctl_field_info),
	cfg_pcie_ctl_field_info
},
{"CFG_BLK_MSI_ENABLE", 0x20,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(cfg_blk_msi_enable_field_info),
	cfg_blk_msi_enable_field_info
},
{"CFG_AXI_USER_MAX_PLD_SIZE", 0x40,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(cfg_axi_user_max_pld_size_field_info),
	cfg_axi_user_max_pld_size_field_info
},
{"CFG_AXI_USER_MAX_READ_REQ_SIZE", 0x44,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(cfg_axi_user_max_read_req_size_field_info),
	cfg_axi_user_max_read_req_size_field_info
},
{"CFG_BLK_MISC_CTL", 0x4c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(cfg_blk_misc_ctl_field_info),
	cfg_blk_misc_ctl_field_info
},
{"CFG_PL_CRED_CTL", 0x68,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(cfg_pl_cred_ctl_field_info),
	cfg_pl_cred_ctl_field_info
},
{"CFG_BLK_SCRATCH", 0x80,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(cfg_blk_scratch_field_info),
	cfg_blk_scratch_field_info
},
{"CFG_GIC", 0xa0,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(cfg_gic_field_info),
	cfg_gic_field_info
},
{"RAM_SBE_MSK_1_A", 0xe0,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(ram_sbe_msk_1_a_field_info),
	ram_sbe_msk_1_a_field_info
},
{"RAM_SBE_STS_1_A", 0xe4,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(ram_sbe_sts_1_a_field_info),
	ram_sbe_sts_1_a_field_info
},
{"RAM_DBE_MSK_1_A", 0xe8,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(ram_dbe_msk_1_a_field_info),
	ram_dbe_msk_1_a_field_info
},
{"RAM_DBE_STS_1_A", 0xec,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(ram_dbe_sts_1_a_field_info),
	ram_dbe_sts_1_a_field_info
},
{"RAM_SBE_MSK_A", 0xf0,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(ram_sbe_msk_a_field_info),
	ram_sbe_msk_a_field_info
},
{"RAM_SBE_STS_A", 0xf4,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(ram_sbe_sts_a_field_info),
	ram_sbe_sts_a_field_info
},
{"RAM_DBE_MSK_A", 0xf8,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(ram_dbe_msk_a_field_info),
	ram_dbe_msk_a_field_info
},
{"RAM_DBE_STS_A", 0xfc,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(ram_dbe_sts_a_field_info),
	ram_dbe_sts_a_field_info
},
{"GLBL2_IDENTIFIER", 0x100,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl2_identifier_field_info),
	glbl2_identifier_field_info
},
{"GLBL2_CHANNEL_INST", 0x114,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl2_channel_inst_field_info),
	glbl2_channel_inst_field_info
},
{"GLBL2_CHANNEL_MDMA", 0x118,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl2_channel_mdma_field_info),
	glbl2_channel_mdma_field_info
},
{"GLBL2_CHANNEL_STRM", 0x11c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl2_channel_strm_field_info),
	glbl2_channel_strm_field_info
},
{"GLBL2_CHANNEL_CAP", 0x120,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl2_channel_cap_field_info),
	glbl2_channel_cap_field_info
},
{"GLBL2_CHANNEL_PASID_CAP", 0x128,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl2_channel_pasid_cap_field_info),
	glbl2_channel_pasid_cap_field_info
},
{"GLBL2_SYSTEM_ID", 0x130,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl2_system_id_field_info),
	glbl2_system_id_field_info
},
{"GLBL2_MISC_CAP", 0x134,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl2_misc_cap_field_info),
	glbl2_misc_cap_field_info
},
{"GLBL2_RRQ_BRG_THROT", 0x158,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl2_rrq_brg_throt_field_info),
	glbl2_rrq_brg_throt_field_info
},
{"GLBL2_RRQ_PCIE_THROT", 0x15c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl2_rrq_pcie_throt_field_info),
	glbl2_rrq_pcie_throt_field_info
},
{"GLBL2_RRQ_AXIMM_THROT", 0x160,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl2_rrq_aximm_throt_field_info),
	glbl2_rrq_aximm_throt_field_info
},
{"GLBL2_RRQ_PCIE_LAT0", 0x164,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl2_rrq_pcie_lat0_field_info),
	glbl2_rrq_pcie_lat0_field_info
},
{"GLBL2_RRQ_PCIE_LAT1", 0x168,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl2_rrq_pcie_lat1_field_info),
	glbl2_rrq_pcie_lat1_field_info
},
{"GLBL2_RRQ_AXIMM_LAT0", 0x16c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl2_rrq_aximm_lat0_field_info),
	glbl2_rrq_aximm_lat0_field_info
},
{"GLBL2_RRQ_AXIMM_LAT1", 0x170,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl2_rrq_aximm_lat1_field_info),
	glbl2_rrq_aximm_lat1_field_info
},
{"GLBL2_DBG_PCIE_RQ0", 0x1b8,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl2_dbg_pcie_rq0_field_info),
	glbl2_dbg_pcie_rq0_field_info
},
{"GLBL2_DBG_PCIE_RQ1", 0x1bc,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl2_dbg_pcie_rq1_field_info),
	glbl2_dbg_pcie_rq1_field_info
},
{"GLBL2_DBG_AXIMM_WR0", 0x1c0,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl2_dbg_aximm_wr0_field_info),
	glbl2_dbg_aximm_wr0_field_info
},
{"GLBL2_DBG_AXIMM_WR1", 0x1c4,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl2_dbg_aximm_wr1_field_info),
	glbl2_dbg_aximm_wr1_field_info
},
{"GLBL2_DBG_AXIMM_RD0", 0x1c8,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl2_dbg_aximm_rd0_field_info),
	glbl2_dbg_aximm_rd0_field_info
},
{"GLBL2_DBG_AXIMM_RD1", 0x1cc,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl2_dbg_aximm_rd1_field_info),
	glbl2_dbg_aximm_rd1_field_info
},
{"GLBL2_DBG_FAB0", 0x1d0,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl2_dbg_fab0_field_info),
	glbl2_dbg_fab0_field_info
},
{"GLBL2_DBG_FAB1", 0x1d4,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl2_dbg_fab1_field_info),
	glbl2_dbg_fab1_field_info
},
{"GLBL2_DBG_MATCH_SEL", 0x1f4,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl2_dbg_match_sel_field_info),
	glbl2_dbg_match_sel_field_info
},
{"GLBL2_DBG_MATCH_MSK", 0x1f8,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl2_dbg_match_msk_field_info),
	glbl2_dbg_match_msk_field_info
},
{"GLBL2_DBG_MATCH_PAT", 0x1fc,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl2_dbg_match_pat_field_info),
	glbl2_dbg_match_pat_field_info
},
{"GLBL_RNG_SZ_1", 0x204,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(glbl_rng_sz_1_field_info),
	glbl_rng_sz_1_field_info
},
{"GLBL_RNG_SZ_2", 0x208,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(glbl_rng_sz_2_field_info),
	glbl_rng_sz_2_field_info
},
{"GLBL_RNG_SZ_3", 0x20c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(glbl_rng_sz_3_field_info),
	glbl_rng_sz_3_field_info
},
{"GLBL_RNG_SZ_4", 0x210,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(glbl_rng_sz_4_field_info),
	glbl_rng_sz_4_field_info
},
{"GLBL_RNG_SZ_5", 0x214,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(glbl_rng_sz_5_field_info),
	glbl_rng_sz_5_field_info
},
{"GLBL_RNG_SZ_6", 0x218,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(glbl_rng_sz_6_field_info),
	glbl_rng_sz_6_field_info
},
{"GLBL_RNG_SZ_7", 0x21c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(glbl_rng_sz_7_field_info),
	glbl_rng_sz_7_field_info
},
{"GLBL_RNG_SZ_8", 0x220,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(glbl_rng_sz_8_field_info),
	glbl_rng_sz_8_field_info
},
{"GLBL_RNG_SZ_9", 0x224,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(glbl_rng_sz_9_field_info),
	glbl_rng_sz_9_field_info
},
{"GLBL_RNG_SZ_A", 0x228,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(glbl_rng_sz_a_field_info),
	glbl_rng_sz_a_field_info
},
{"GLBL_RNG_SZ_B", 0x22c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(glbl_rng_sz_b_field_info),
	glbl_rng_sz_b_field_info
},
{"GLBL_RNG_SZ_C", 0x230,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(glbl_rng_sz_c_field_info),
	glbl_rng_sz_c_field_info
},
{"GLBL_RNG_SZ_D", 0x234,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(glbl_rng_sz_d_field_info),
	glbl_rng_sz_d_field_info
},
{"GLBL_RNG_SZ_E", 0x238,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(glbl_rng_sz_e_field_info),
	glbl_rng_sz_e_field_info
},
{"GLBL_RNG_SZ_F", 0x23c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(glbl_rng_sz_f_field_info),
	glbl_rng_sz_f_field_info
},
{"GLBL_RNG_SZ_10", 0x240,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(glbl_rng_sz_10_field_info),
	glbl_rng_sz_10_field_info
},
{"GLBL_ERR_STAT", 0x248,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(glbl_err_stat_field_info),
	glbl_err_stat_field_info
},
{"GLBL_ERR_MASK", 0x24c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(glbl_err_mask_field_info),
	glbl_err_mask_field_info
},
{"GLBL_DSC_CFG", 0x250,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(glbl_dsc_cfg_field_info),
	glbl_dsc_cfg_field_info
},
{"GLBL_DSC_ERR_STS", 0x254,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(glbl_dsc_err_sts_field_info),
	glbl_dsc_err_sts_field_info
},
{"GLBL_DSC_ERR_MSK", 0x258,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(glbl_dsc_err_msk_field_info),
	glbl_dsc_err_msk_field_info
},
{"GLBL_DSC_ERR_LOG0", 0x25c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(glbl_dsc_err_log0_field_info),
	glbl_dsc_err_log0_field_info
},
{"GLBL_DSC_ERR_LOG1", 0x260,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(glbl_dsc_err_log1_field_info),
	glbl_dsc_err_log1_field_info
},
{"GLBL_TRQ_ERR_STS", 0x264,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(glbl_trq_err_sts_field_info),
	glbl_trq_err_sts_field_info
},
{"GLBL_TRQ_ERR_MSK", 0x268,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(glbl_trq_err_msk_field_info),
	glbl_trq_err_msk_field_info
},
{"GLBL_TRQ_ERR_LOG", 0x26c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(glbl_trq_err_log_field_info),
	glbl_trq_err_log_field_info
},
{"GLBL_DSC_DBG_DAT0", 0x270,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(glbl_dsc_dbg_dat0_field_info),
	glbl_dsc_dbg_dat0_field_info
},
{"GLBL_DSC_DBG_DAT1", 0x274,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(glbl_dsc_dbg_dat1_field_info),
	glbl_dsc_dbg_dat1_field_info
},
{"GLBL_DSC_DBG_CTL", 0x278,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(glbl_dsc_dbg_ctl_field_info),
	glbl_dsc_dbg_ctl_field_info
},
{"GLBL_DSC_ERR_LOG2", 0x27c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(glbl_dsc_err_log2_field_info),
	glbl_dsc_err_log2_field_info
},
{"GLBL_GLBL_INTERRUPT_CFG", 0x2c4,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(glbl_glbl_interrupt_cfg_field_info),
	glbl_glbl_interrupt_cfg_field_info
},
{"GLBL_VCH_HOST_PROFILE", 0x2c8,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(glbl_vch_host_profile_field_info),
	glbl_vch_host_profile_field_info
},
{"GLBL_BRIDGE_HOST_PROFILE", 0x308,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(glbl_bridge_host_profile_field_info),
	glbl_bridge_host_profile_field_info
},
{"AXIMM_IRQ_DEST_ADDR", 0x30c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(aximm_irq_dest_addr_field_info),
	aximm_irq_dest_addr_field_info
},
{"FAB_ERR_LOG", 0x314,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(fab_err_log_field_info),
	fab_err_log_field_info
},
{"GLBL_REQ_ERR_STS", 0x318,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_req_err_sts_field_info),
	glbl_req_err_sts_field_info
},
{"GLBL_REQ_ERR_MSK", 0x31c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_req_err_msk_field_info),
	glbl_req_err_msk_field_info
},
{"GLBL_DSC_DBG_LAT0_A", 0x320,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_dsc_dbg_lat0_a_field_info),
	glbl_dsc_dbg_lat0_a_field_info
},
{"GLBL_DSC_DBG_LAT1_A", 0x324,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_dsc_dbg_lat1_a_field_info),
	glbl_dsc_dbg_lat1_a_field_info
},
{"GLBL_DSC_CRD_CTR0_A", 0x328,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_dsc_crd_ctr0_a_field_info),
	glbl_dsc_crd_ctr0_a_field_info
},
{"GLBL_DSC_CRD_CTR1_A", 0x32c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_dsc_crd_ctr1_a_field_info),
	glbl_dsc_crd_ctr1_a_field_info
},
{"GLBL_DSC_CRD_CTR2_A", 0x330,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_dsc_crd_ctr2_a_field_info),
	glbl_dsc_crd_ctr2_a_field_info
},
{"GLBL_DSC_CRD_CTR3_A", 0x334,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_dsc_crd_ctr3_a_field_info),
	glbl_dsc_crd_ctr3_a_field_info
},
{"GLBL_DSC_IMM_CRD_CTR0_A", 0x338,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_dsc_imm_crd_ctr0_a_field_info),
	glbl_dsc_imm_crd_ctr0_a_field_info
},
{"GLBL_DSC_IMM_CRD_CTR1_A", 0x33c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_dsc_imm_crd_ctr1_a_field_info),
	glbl_dsc_imm_crd_ctr1_a_field_info
},
{"GLBL_DSC_IMM_CRD_CTR2_A", 0x340,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_dsc_imm_crd_ctr2_a_field_info),
	glbl_dsc_imm_crd_ctr2_a_field_info
},
{"GLBL_DSC_IMM_CRD_CTR3_A", 0x344,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_dsc_imm_crd_ctr3_a_field_info),
	glbl_dsc_imm_crd_ctr3_a_field_info
},
{"GLBL_DSC_H2C_OUT_CTR0_A", 0x348,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_dsc_h2c_out_ctr0_a_field_info),
	glbl_dsc_h2c_out_ctr0_a_field_info
},
{"GLBL_DSC_H2C_OUT_CTR1_A", 0x34c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_dsc_h2c_out_ctr1_a_field_info),
	glbl_dsc_h2c_out_ctr1_a_field_info
},
{"GLBL_DSC_H2C_OUT_CTR2_A", 0x350,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_dsc_h2c_out_ctr2_a_field_info),
	glbl_dsc_h2c_out_ctr2_a_field_info
},
{"GLBL_DSC_H2C_OUT_CTR3_A", 0x354,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_dsc_h2c_out_ctr3_a_field_info),
	glbl_dsc_h2c_out_ctr3_a_field_info
},
{"GLBL_DSC_C2H_OUT_CTR0_A", 0x358,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_dsc_c2h_out_ctr0_a_field_info),
	glbl_dsc_c2h_out_ctr0_a_field_info
},
{"GLBL_DSC_C2H_OUT_CTR1_A", 0x35c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_dsc_c2h_out_ctr1_a_field_info),
	glbl_dsc_c2h_out_ctr1_a_field_info
},
{"GLBL_DSC_C2H_OUT_CTR2_A", 0x360,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_dsc_c2h_out_ctr2_a_field_info),
	glbl_dsc_c2h_out_ctr2_a_field_info
},
{"GLBL_DSC_C2H_OUT_CTR3_A", 0x364,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_dsc_c2h_out_ctr3_a_field_info),
	glbl_dsc_c2h_out_ctr3_a_field_info
},
{"T", 0x368,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(t_field_info),
	t_field_info
},
{"GLBL_PERF_CNTR_CTL_A1", 0x36c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_perf_cntr_ctl_a1_field_info),
	glbl_perf_cntr_ctl_a1_field_info
},
{"GLBL_FREE_CNT_A0", 0x370,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_free_cnt_a0_field_info),
	glbl_free_cnt_a0_field_info
},
{"GLBL_FREE_CNT_A1", 0x374,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_free_cnt_a1_field_info),
	glbl_free_cnt_a1_field_info
},
{"GLBL_AXIS_H2C_CNT_A0", 0x378,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_axis_h2c_cnt_a0_field_info),
	glbl_axis_h2c_cnt_a0_field_info
},
{"GLBL_AXIS_H2C_CNT_A1", 0x37c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_axis_h2c_cnt_a1_field_info),
	glbl_axis_h2c_cnt_a1_field_info
},
{"GLBL_AXIS_H2C_CNT_A2", 0x380,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_axis_h2c_cnt_a2_field_info),
	glbl_axis_h2c_cnt_a2_field_info
},
{"GLBL_AXIS_H2C_CNT_A3", 0x384,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_axis_h2c_cnt_a3_field_info),
	glbl_axis_h2c_cnt_a3_field_info
},
{"GLBL_AXIS_H2C_CNT_A4", 0x388,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_axis_h2c_cnt_a4_field_info),
	glbl_axis_h2c_cnt_a4_field_info
},
{"GLBL_AXIS_H2C_CNT_A5", 0x38c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_axis_h2c_cnt_a5_field_info),
	glbl_axis_h2c_cnt_a5_field_info
},
{"GLBL_AXIS_C2H_CNT_A0", 0x390,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_axis_c2h_cnt_a0_field_info),
	glbl_axis_c2h_cnt_a0_field_info
},
{"GLBL_AXIS_C2H_CNT_A1", 0x394,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_axis_c2h_cnt_a1_field_info),
	glbl_axis_c2h_cnt_a1_field_info
},
{"GLBL_AXIS_C2H_CNT_A2", 0x398,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_axis_c2h_cnt_a2_field_info),
	glbl_axis_c2h_cnt_a2_field_info
},
{"GLBL_AXIS_C2H_CNT_A3", 0x39c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_axis_c2h_cnt_a3_field_info),
	glbl_axis_c2h_cnt_a3_field_info
},
{"GLBL_AXIS_C2H_CNT_A4", 0x3a0,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_axis_c2h_cnt_a4_field_info),
	glbl_axis_c2h_cnt_a4_field_info
},
{"GLBL_AXIS_C2H_CNT_A5", 0x3a4,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_axis_c2h_cnt_a5_field_info),
	glbl_axis_c2h_cnt_a5_field_info
},
{"GLBL_M_AXI_WR_CNT_A0", 0x3a8,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_m_axi_wr_cnt_a0_field_info),
	glbl_m_axi_wr_cnt_a0_field_info
},
{"GLBL_M_AXI_WR_CNT_A1", 0x3ac,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_m_axi_wr_cnt_a1_field_info),
	glbl_m_axi_wr_cnt_a1_field_info
},
{"GLBL_M_AXI_WR_CNT_A2", 0x3b0,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_m_axi_wr_cnt_a2_field_info),
	glbl_m_axi_wr_cnt_a2_field_info
},
{"GLBL_M_AXI_WR_CNT_A3", 0x3b4,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_m_axi_wr_cnt_a3_field_info),
	glbl_m_axi_wr_cnt_a3_field_info
},
{"GLBL_M_AXI_WR_CNT_A4", 0x3b8,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_m_axi_wr_cnt_a4_field_info),
	glbl_m_axi_wr_cnt_a4_field_info
},
{"GLBL_M_AXI_WR_CNT_A5", 0x3bc,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_m_axi_wr_cnt_a5_field_info),
	glbl_m_axi_wr_cnt_a5_field_info
},
{"GLBL_M_AXI_RD_CNT_A0", 0x3c0,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_m_axi_rd_cnt_a0_field_info),
	glbl_m_axi_rd_cnt_a0_field_info
},
{"GLBL_M_AXI_RD_CNT_A1", 0x3c4,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_m_axi_rd_cnt_a1_field_info),
	glbl_m_axi_rd_cnt_a1_field_info
},
{"GLBL_M_AXI_RD_CNT_A2", 0x3c8,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_m_axi_rd_cnt_a2_field_info),
	glbl_m_axi_rd_cnt_a2_field_info
},
{"GLBL_M_AXI_RD_CNT_A3", 0x3cc,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_m_axi_rd_cnt_a3_field_info),
	glbl_m_axi_rd_cnt_a3_field_info
},
{"GLBL_M_AXI_RD_CNT_A4", 0x3d0,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_m_axi_rd_cnt_a4_field_info),
	glbl_m_axi_rd_cnt_a4_field_info
},
{"GLBL_M_AXI_RD_CNT_A5", 0x3d4,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_m_axi_rd_cnt_a5_field_info),
	glbl_m_axi_rd_cnt_a5_field_info
},
{"GLBL_M_AXIB_WR_CNT_A0", 0x3d8,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_m_axib_wr_cnt_a0_field_info),
	glbl_m_axib_wr_cnt_a0_field_info
},
{"GLBL_M_AXIB_WR_CNT_A1", 0x3dc,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_m_axib_wr_cnt_a1_field_info),
	glbl_m_axib_wr_cnt_a1_field_info
},
{"GLBL_M_AXIB_WR_CNT_A2", 0x3e0,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_m_axib_wr_cnt_a2_field_info),
	glbl_m_axib_wr_cnt_a2_field_info
},
{"GLBL_M_AXIB_WR_CNT_A3", 0x3e4,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_m_axib_wr_cnt_a3_field_info),
	glbl_m_axib_wr_cnt_a3_field_info
},
{"GLBL_M_AXIB_WR_CNT_A4", 0x3e8,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_m_axib_wr_cnt_a4_field_info),
	glbl_m_axib_wr_cnt_a4_field_info
},
{"GLBL_M_AXIB_WR_CNT_A5", 0x3ec,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_m_axib_wr_cnt_a5_field_info),
	glbl_m_axib_wr_cnt_a5_field_info
},
{"GLBL_M_AXIB_RD_CNT_A0", 0x3f0,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_m_axib_rd_cnt_a0_field_info),
	glbl_m_axib_rd_cnt_a0_field_info
},
{"GLBL_M_AXIB_RD_CNT_A1", 0x3f4,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_m_axib_rd_cnt_a1_field_info),
	glbl_m_axib_rd_cnt_a1_field_info
},
{"GLBL_M_AXIB_RD_CNT_A2", 0x3f8,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_m_axib_rd_cnt_a2_field_info),
	glbl_m_axib_rd_cnt_a2_field_info
},
{"GLBL_M_AXIB_RD_CNT_A3", 0x3fc,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_m_axib_rd_cnt_a3_field_info),
	glbl_m_axib_rd_cnt_a3_field_info
},
{"GLBL_M_AXIB_RD_CNT_A4", 0x400,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_m_axib_rd_cnt_a4_field_info),
	glbl_m_axib_rd_cnt_a4_field_info
},
{"GLBL_M_AXIB_RD_CNT_A5", 0x404,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_m_axib_rd_cnt_a5_field_info),
	glbl_m_axib_rd_cnt_a5_field_info
},
{"GLBL_S_AXI_WR_CNT_A0", 0x408,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_s_axi_wr_cnt_a0_field_info),
	glbl_s_axi_wr_cnt_a0_field_info
},
{"GLBL_S_AXI_WR_CNT_A1", 0x40c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_s_axi_wr_cnt_a1_field_info),
	glbl_s_axi_wr_cnt_a1_field_info
},
{"GLBL_S_AXI_WR_CNT_A2", 0x410,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_s_axi_wr_cnt_a2_field_info),
	glbl_s_axi_wr_cnt_a2_field_info
},
{"GLBL_S_AXI_WR_CNT_A3", 0x414,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_s_axi_wr_cnt_a3_field_info),
	glbl_s_axi_wr_cnt_a3_field_info
},
{"GLBL_S_AXI_WR_CNT_A4", 0x418,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_s_axi_wr_cnt_a4_field_info),
	glbl_s_axi_wr_cnt_a4_field_info
},
{"GLBL_S_AXI_WR_CNT_A5", 0x41c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_s_axi_wr_cnt_a5_field_info),
	glbl_s_axi_wr_cnt_a5_field_info
},
{"GLBL_S_AXI_RD_CNT_A0", 0x420,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_s_axi_rd_cnt_a0_field_info),
	glbl_s_axi_rd_cnt_a0_field_info
},
{"GLBL_S_AXI_RD_CNT_A1", 0x424,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_s_axi_rd_cnt_a1_field_info),
	glbl_s_axi_rd_cnt_a1_field_info
},
{"GLBL_S_AXI_RD_CNT_A2", 0x428,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_s_axi_rd_cnt_a2_field_info),
	glbl_s_axi_rd_cnt_a2_field_info
},
{"GLBL_S_AXI_RD_CNT_A3", 0x42c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_s_axi_rd_cnt_a3_field_info),
	glbl_s_axi_rd_cnt_a3_field_info
},
{"GLBL_S_AXI_RD_CNT_A4", 0x430,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_s_axi_rd_cnt_a4_field_info),
	glbl_s_axi_rd_cnt_a4_field_info
},
{"GLBL_S_AXI_RD_CNT_A5", 0x434,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_s_axi_rd_cnt_a5_field_info),
	glbl_s_axi_rd_cnt_a5_field_info
},
{"GLBL_S_AXIS_CMP_CNT_A0", 0x438,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_s_axis_cmp_cnt_a0_field_info),
	glbl_s_axis_cmp_cnt_a0_field_info
},
{"GLBL_S_AXIS_CMP_CNT_A1", 0x43c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_s_axis_cmp_cnt_a1_field_info),
	glbl_s_axis_cmp_cnt_a1_field_info
},
{"GLBL_S_AXIS_CMP_CNT_A2", 0x440,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_s_axis_cmp_cnt_a2_field_info),
	glbl_s_axis_cmp_cnt_a2_field_info
},
{"GLBL_S_AXIS_CMP_CNT_A3", 0x444,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_s_axis_cmp_cnt_a3_field_info),
	glbl_s_axis_cmp_cnt_a3_field_info
},
{"GLBL_S_AXIS_CMP_CNT_A4", 0x448,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_s_axis_cmp_cnt_a4_field_info),
	glbl_s_axis_cmp_cnt_a4_field_info
},
{"GLBL_S_AXIS_CMP_CNT_A5", 0x44c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(glbl_s_axis_cmp_cnt_a5_field_info),
	glbl_s_axis_cmp_cnt_a5_field_info
},
{"IND_CTXT_DATA", 0x804,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(ind_ctxt_data_field_info),
	ind_ctxt_data_field_info
},
{"IND_CTXT_MASK", 0x824,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(ind_ctxt_mask_field_info),
	ind_ctxt_mask_field_info
},
{"IND_CTXT_CMD", 0x844,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(ind_ctxt_cmd_field_info),
	ind_ctxt_cmd_field_info
},
{"C2H_TIMER_CNT", 0xa00,
	1, 0, 0, 0,
	0, QDMA_COMPLETION_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_timer_cnt_field_info),
	c2h_timer_cnt_field_info
},
{"C2H_CNT_TH", 0xa40,
	1, 0, 0, 0,
	0, QDMA_COMPLETION_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_cnt_th_field_info),
	c2h_cnt_th_field_info
},
{"C2H_PFCH_CFG_1", 0xa80,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(c2h_pfch_cfg_1_field_info),
	c2h_pfch_cfg_1_field_info
},
{"C2H_PFCH_CFG_2", 0xa84,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(c2h_pfch_cfg_2_field_info),
	c2h_pfch_cfg_2_field_info
},
{"C2H_STAT_S_AXIS_C2H_ACCEPTED", 0xa88,
	1, 0, 0, 0,
	1, QDMA_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(c2h_stat_s_axis_c2h_accepted_field_info),
	c2h_stat_s_axis_c2h_accepted_field_info
},
{"C2H_STAT_S_AXIS_WRB_ACCEPTED", 0xa8c,
	1, 0, 0, 0,
	1, QDMA_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(c2h_stat_s_axis_wrb_accepted_field_info),
	c2h_stat_s_axis_wrb_accepted_field_info
},
{"C2H_STAT_DESC_RSP_PKT_ACCEPTED", 0xa90,
	1, 0, 0, 0,
	1, QDMA_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(c2h_stat_desc_rsp_pkt_accepted_field_info),
	c2h_stat_desc_rsp_pkt_accepted_field_info
},
{"C2H_STAT_AXIS_PKG_CMP", 0xa94,
	1, 0, 0, 0,
	1, QDMA_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(c2h_stat_axis_pkg_cmp_field_info),
	c2h_stat_axis_pkg_cmp_field_info
},
{"C2H_STAT_DESC_RSP_ACCEPTED", 0xa98,
	1, 0, 0, 0,
	1, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_stat_desc_rsp_accepted_field_info),
	c2h_stat_desc_rsp_accepted_field_info
},
{"C2H_STAT_DESC_RSP_CMP", 0xa9c,
	1, 0, 0, 0,
	1, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_stat_desc_rsp_cmp_field_info),
	c2h_stat_desc_rsp_cmp_field_info
},
{"C2H_STAT_WRQ_OUT", 0xaa0,
	1, 0, 0, 0,
	1, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_stat_wrq_out_field_info),
	c2h_stat_wrq_out_field_info
},
{"C2H_STAT_WPL_REN_ACCEPTED", 0xaa4,
	1, 0, 0, 0,
	1, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_stat_wpl_ren_accepted_field_info),
	c2h_stat_wpl_ren_accepted_field_info
},
{"C2H_STAT_TOTAL_WRQ_LEN", 0xaa8,
	1, 0, 0, 0,
	1, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_stat_total_wrq_len_field_info),
	c2h_stat_total_wrq_len_field_info
},
{"C2H_STAT_TOTAL_WPL_LEN", 0xaac,
	1, 0, 0, 0,
	1, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_stat_total_wpl_len_field_info),
	c2h_stat_total_wpl_len_field_info
},
{"C2H_BUF_SZ", 0xab0,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_buf_sz_field_info),
	c2h_buf_sz_field_info
},
{"C2H_ERR_STAT", 0xaf0,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(c2h_err_stat_field_info),
	c2h_err_stat_field_info
},
{"C2H_ERR_MASK", 0xaf4,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(c2h_err_mask_field_info),
	c2h_err_mask_field_info
},
{"C2H_FATAL_ERR_STAT", 0xaf8,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(c2h_fatal_err_stat_field_info),
	c2h_fatal_err_stat_field_info
},
{"C2H_FATAL_ERR_MASK", 0xafc,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(c2h_fatal_err_mask_field_info),
	c2h_fatal_err_mask_field_info
},
{"C2H_FATAL_ERR_ENABLE", 0xb00,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(c2h_fatal_err_enable_field_info),
	c2h_fatal_err_enable_field_info
},
{"GLBL_ERR_INT", 0xb04,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(glbl_err_int_field_info),
	glbl_err_int_field_info
},
{"C2H_PFCH_CFG", 0xb08,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_pfch_cfg_field_info),
	c2h_pfch_cfg_field_info
},
{"C2H_INT_TIMER_TICK", 0xb0c,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_int_timer_tick_field_info),
	c2h_int_timer_tick_field_info
},
{"C2H_STAT_DESC_RSP_DROP_ACCEPTED", 0xb10,
	1, 0, 0, 0,
	1, QDMA_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(c2h_stat_desc_rsp_drop_accepted_field_info),
	c2h_stat_desc_rsp_drop_accepted_field_info
},
{"C2H_STAT_DESC_RSP_ERR_ACCEPTED", 0xb14,
	1, 0, 0, 0,
	1, QDMA_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(c2h_stat_desc_rsp_err_accepted_field_info),
	c2h_stat_desc_rsp_err_accepted_field_info
},
{"C2H_STAT_DESC_REQ", 0xb18,
	1, 0, 0, 0,
	1, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_stat_desc_req_field_info),
	c2h_stat_desc_req_field_info
},
{"C2H_STAT_DBG_DMA_ENG_0", 0xb1c,
	1, 0, 0, 0,
	1, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_stat_dbg_dma_eng_0_field_info),
	c2h_stat_dbg_dma_eng_0_field_info
},
{"C2H_STAT_DBG_DMA_ENG_1", 0xb20,
	1, 0, 0, 0,
	1, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_stat_dbg_dma_eng_1_field_info),
	c2h_stat_dbg_dma_eng_1_field_info
},
{"C2H_STAT_DBG_DMA_ENG_2", 0xb24,
	1, 0, 0, 0,
	1, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_stat_dbg_dma_eng_2_field_info),
	c2h_stat_dbg_dma_eng_2_field_info
},
{"C2H_STAT_DBG_DMA_ENG_3", 0xb28,
	1, 0, 0, 0,
	1, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_stat_dbg_dma_eng_3_field_info),
	c2h_stat_dbg_dma_eng_3_field_info
},
{"C2H_DBG_PFCH_ERR_CTXT", 0xb2c,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_dbg_pfch_err_ctxt_field_info),
	c2h_dbg_pfch_err_ctxt_field_info
},
{"C2H_FIRST_ERR_QID", 0xb30,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(c2h_first_err_qid_field_info),
	c2h_first_err_qid_field_info
},
{"STAT_NUM_WRB_IN", 0xb34,
	1, 0, 0, 0,
	1, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(stat_num_wrb_in_field_info),
	stat_num_wrb_in_field_info
},
{"STAT_NUM_WRB_OUT", 0xb38,
	1, 0, 0, 0,
	1, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(stat_num_wrb_out_field_info),
	stat_num_wrb_out_field_info
},
{"STAT_NUM_WRB_DRP", 0xb3c,
	1, 0, 0, 0,
	1, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(stat_num_wrb_drp_field_info),
	stat_num_wrb_drp_field_info
},
{"STAT_NUM_STAT_DESC_OUT", 0xb40,
	1, 0, 0, 0,
	1, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(stat_num_stat_desc_out_field_info),
	stat_num_stat_desc_out_field_info
},
{"STAT_NUM_DSC_CRDT_SENT", 0xb44,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(stat_num_dsc_crdt_sent_field_info),
	stat_num_dsc_crdt_sent_field_info
},
{"STAT_NUM_FCH_DSC_RCVD", 0xb48,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(stat_num_fch_dsc_rcvd_field_info),
	stat_num_fch_dsc_rcvd_field_info
},
{"STAT_NUM_BYP_DSC_RCVD", 0xb4c,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(stat_num_byp_dsc_rcvd_field_info),
	stat_num_byp_dsc_rcvd_field_info
},
{"C2H_WRB_COAL_CFG", 0xb50,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_wrb_coal_cfg_field_info),
	c2h_wrb_coal_cfg_field_info
},
{"C2H_INTR_H2C_REQ", 0xb54,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(c2h_intr_h2c_req_field_info),
	c2h_intr_h2c_req_field_info
},
{"C2H_INTR_C2H_MM_REQ", 0xb58,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(c2h_intr_c2h_mm_req_field_info),
	c2h_intr_c2h_mm_req_field_info
},
{"C2H_INTR_ERR_INT_REQ", 0xb5c,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(c2h_intr_err_int_req_field_info),
	c2h_intr_err_int_req_field_info
},
{"C2H_INTR_C2H_ST_REQ", 0xb60,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(c2h_intr_c2h_st_req_field_info),
	c2h_intr_c2h_st_req_field_info
},
{"C2H_INTR_H2C_ERR_C2H_MM_MSIX_ACK", 0xb64,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_intr_h2c_err_c2h_mm_msix_ack_field_info),
	c2h_intr_h2c_err_c2h_mm_msix_ack_field_info
},
{"C2H_INTR_H2C_ERR_C2H_MM_MSIX_FAIL", 0xb68,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_intr_h2c_err_c2h_mm_msix_fail_field_info),
	c2h_intr_h2c_err_c2h_mm_msix_fail_field_info
},
{"C2H_INTR_H2C_ERR_C2H_MM_MSIX_NO_MSIX", 0xb6c,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_intr_h2c_err_c2h_mm_msix_no_msix_field_info),
	c2h_intr_h2c_err_c2h_mm_msix_no_msix_field_info
},
{"C2H_INTR_H2C_ERR_C2H_MM_CTXT_INVAL", 0xb70,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_intr_h2c_err_c2h_mm_ctxt_inval_field_info),
	c2h_intr_h2c_err_c2h_mm_ctxt_inval_field_info
},
{"C2H_INTR_C2H_ST_MSIX_ACK", 0xb74,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(c2h_intr_c2h_st_msix_ack_field_info),
	c2h_intr_c2h_st_msix_ack_field_info
},
{"C2H_INTR_C2H_ST_MSIX_FAIL", 0xb78,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(c2h_intr_c2h_st_msix_fail_field_info),
	c2h_intr_c2h_st_msix_fail_field_info
},
{"C2H_INTR_C2H_ST_NO_MSIX", 0xb7c,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_intr_c2h_st_no_msix_field_info),
	c2h_intr_c2h_st_no_msix_field_info
},
{"C2H_INTR_C2H_ST_CTXT_INVAL", 0xb80,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_intr_c2h_st_ctxt_inval_field_info),
	c2h_intr_c2h_st_ctxt_inval_field_info
},
{"C2H_STAT_WR_CMP", 0xb84,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_stat_wr_cmp_field_info),
	c2h_stat_wr_cmp_field_info
},
{"C2H_STAT_DBG_DMA_ENG_4", 0xb88,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_stat_dbg_dma_eng_4_field_info),
	c2h_stat_dbg_dma_eng_4_field_info
},
{"C2H_STAT_DBG_DMA_ENG_5", 0xb8c,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_stat_dbg_dma_eng_5_field_info),
	c2h_stat_dbg_dma_eng_5_field_info
},
{"C2H_DBG_PFCH_QID", 0xb90,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_dbg_pfch_qid_field_info),
	c2h_dbg_pfch_qid_field_info
},
{"C2H_DBG_PFCH", 0xb94,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_dbg_pfch_field_info),
	c2h_dbg_pfch_field_info
},
{"C2H_INT_DBG", 0xb98,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_int_dbg_field_info),
	c2h_int_dbg_field_info
},
{"C2H_STAT_IMM_ACCEPTED", 0xb9c,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_stat_imm_accepted_field_info),
	c2h_stat_imm_accepted_field_info
},
{"C2H_STAT_MARKER_ACCEPTED", 0xba0,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_stat_marker_accepted_field_info),
	c2h_stat_marker_accepted_field_info
},
{"C2H_STAT_DISABLE_CMP_ACCEPTED", 0xba4,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_stat_disable_cmp_accepted_field_info),
	c2h_stat_disable_cmp_accepted_field_info
},
{"C2H_PLD_FIFO_CRDT_CNT", 0xba8,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_pld_fifo_crdt_cnt_field_info),
	c2h_pld_fifo_crdt_cnt_field_info
},
{"C2H_INTR_DYN_REQ", 0xbac,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_intr_dyn_req_field_info),
	c2h_intr_dyn_req_field_info
},
{"C2H_INTR_DYN_MISC", 0xbb0,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_intr_dyn_misc_field_info),
	c2h_intr_dyn_misc_field_info
},
{"C2H_DROP_LEN_MISMATCH", 0xbb4,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_drop_len_mismatch_field_info),
	c2h_drop_len_mismatch_field_info
},
{"C2H_DROP_DESC_RSP_LEN", 0xbb8,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_drop_desc_rsp_len_field_info),
	c2h_drop_desc_rsp_len_field_info
},
{"C2H_DROP_QID_FIFO_LEN", 0xbbc,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_drop_qid_fifo_len_field_info),
	c2h_drop_qid_fifo_len_field_info
},
{"C2H_DROP_PLD_CNT", 0xbc0,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_drop_pld_cnt_field_info),
	c2h_drop_pld_cnt_field_info
},
{"C2H_CMPT_FORMAT_0", 0xbc4,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_cmpt_format_0_field_info),
	c2h_cmpt_format_0_field_info
},
{"C2H_CMPT_FORMAT_1", 0xbc8,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_cmpt_format_1_field_info),
	c2h_cmpt_format_1_field_info
},
{"C2H_CMPT_FORMAT_2", 0xbcc,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_cmpt_format_2_field_info),
	c2h_cmpt_format_2_field_info
},
{"C2H_CMPT_FORMAT_3", 0xbd0,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_cmpt_format_3_field_info),
	c2h_cmpt_format_3_field_info
},
{"C2H_CMPT_FORMAT_4", 0xbd4,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_cmpt_format_4_field_info),
	c2h_cmpt_format_4_field_info
},
{"C2H_CMPT_FORMAT_5", 0xbd8,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_cmpt_format_5_field_info),
	c2h_cmpt_format_5_field_info
},
{"C2H_CMPT_FORMAT_6", 0xbdc,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_cmpt_format_6_field_info),
	c2h_cmpt_format_6_field_info
},
{"C2H_PFCH_CACHE_DEPTH", 0xbe0,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_pfch_cache_depth_field_info),
	c2h_pfch_cache_depth_field_info
},
{"C2H_WRB_COAL_BUF_DEPTH", 0xbe4,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_wrb_coal_buf_depth_field_info),
	c2h_wrb_coal_buf_depth_field_info
},
{"C2H_PFCH_CRDT", 0xbe8,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_pfch_crdt_field_info),
	c2h_pfch_crdt_field_info
},
{"C2H_STAT_HAS_CMPT_ACCEPTED", 0xbec,
	1, 0, 0, 0,
	1, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_stat_has_cmpt_accepted_field_info),
	c2h_stat_has_cmpt_accepted_field_info
},
{"C2H_STAT_HAS_PLD_ACCEPTED", 0xbf0,
	1, 0, 0, 0,
	1, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_stat_has_pld_accepted_field_info),
	c2h_stat_has_pld_accepted_field_info
},
{"C2H_PLD_PKT_ID", 0xbf4,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_pld_pkt_id_field_info),
	c2h_pld_pkt_id_field_info
},
{"C2H_PLD_PKT_ID_1", 0xbf8,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_pld_pkt_id_1_field_info),
	c2h_pld_pkt_id_1_field_info
},
{"C2H_DROP_PLD_CNT_1", 0xbfc,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_drop_pld_cnt_1_field_info),
	c2h_drop_pld_cnt_1_field_info
},
{"H2C_ERR_STAT", 0xe00,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(h2c_err_stat_field_info),
	h2c_err_stat_field_info
},
{"H2C_ERR_MASK", 0xe04,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(h2c_err_mask_field_info),
	h2c_err_mask_field_info
},
{"H2C_FIRST_ERR_QID", 0xe08,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(h2c_first_err_qid_field_info),
	h2c_first_err_qid_field_info
},
{"H2C_DBG_REG0", 0xe0c,
	1, 0, 0, 0,
	1, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(h2c_dbg_reg0_field_info),
	h2c_dbg_reg0_field_info
},
{"H2C_DBG_REG1", 0xe10,
	1, 0, 0, 0,
	1, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(h2c_dbg_reg1_field_info),
	h2c_dbg_reg1_field_info
},
{"H2C_DBG_REG2", 0xe14,
	1, 0, 0, 0,
	1, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(h2c_dbg_reg2_field_info),
	h2c_dbg_reg2_field_info
},
{"H2C_DBG_REG3", 0xe18,
	1, 0, 0, 0,
	1, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(h2c_dbg_reg3_field_info),
	h2c_dbg_reg3_field_info
},
{"H2C_DBG_REG4", 0xe1c,
	1, 0, 0, 0,
	1, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(h2c_dbg_reg4_field_info),
	h2c_dbg_reg4_field_info
},
{"H2C_FATAL_ERR_EN", 0xe20,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(h2c_fatal_err_en_field_info),
	h2c_fatal_err_en_field_info
},
{"H2C_REQ_THROT_PCIE", 0xe24,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(h2c_req_throt_pcie_field_info),
	h2c_req_throt_pcie_field_info
},
{"H2C_ALN_DBG_REG0", 0xe28,
	1, 0, 0, 0,
	1, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(h2c_aln_dbg_reg0_field_info),
	h2c_aln_dbg_reg0_field_info
},
{"H2C_REQ_THROT_AXIMM", 0xe2c,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(h2c_req_throt_aximm_field_info),
	h2c_req_throt_aximm_field_info
},
{"C2H_MM_CTL", 0x1004,
	1, 0, 0, 0,
	0, QDMA_MM_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_mm_ctl_field_info),
	c2h_mm_ctl_field_info
},
{"C2H_MM_STATUS", 0x1040,
	1, 0, 0, 0,
	0, QDMA_MM_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_mm_status_field_info),
	c2h_mm_status_field_info
},
{"C2H_MM_CMPL_DESC_CNT", 0x1048,
	1, 0, 0, 0,
	0, QDMA_MM_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_mm_cmpl_desc_cnt_field_info),
	c2h_mm_cmpl_desc_cnt_field_info
},
{"C2H_MM_ERR_CODE_ENABLE_MASK", 0x1054,
	1, 0, 0, 0,
	0, QDMA_MM_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_mm_err_code_enable_mask_field_info),
	c2h_mm_err_code_enable_mask_field_info
},
{"C2H_MM_ERR_CODE", 0x1058,
	1, 0, 0, 0,
	0, QDMA_MM_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_mm_err_code_field_info),
	c2h_mm_err_code_field_info
},
{"C2H_MM_ERR_INFO", 0x105c,
	1, 0, 0, 0,
	0, QDMA_MM_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_mm_err_info_field_info),
	c2h_mm_err_info_field_info
},
{"C2H_MM_PERF_MON_CTL", 0x10c0,
	1, 0, 0, 0,
	0, QDMA_MM_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_mm_perf_mon_ctl_field_info),
	c2h_mm_perf_mon_ctl_field_info
},
{"C2H_MM_PERF_MON_CYCLE_CNT0", 0x10c4,
	1, 0, 0, 0,
	0, QDMA_MM_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_mm_perf_mon_cycle_cnt0_field_info),
	c2h_mm_perf_mon_cycle_cnt0_field_info
},
{"C2H_MM_PERF_MON_CYCLE_CNT1", 0x10c8,
	1, 0, 0, 0,
	0, QDMA_MM_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_mm_perf_mon_cycle_cnt1_field_info),
	c2h_mm_perf_mon_cycle_cnt1_field_info
},
{"C2H_MM_PERF_MON_DATA_CNT0", 0x10cc,
	1, 0, 0, 0,
	0, QDMA_MM_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_mm_perf_mon_data_cnt0_field_info),
	c2h_mm_perf_mon_data_cnt0_field_info
},
{"C2H_MM_PERF_MON_DATA_CNT1", 0x10d0,
	1, 0, 0, 0,
	0, QDMA_MM_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_mm_perf_mon_data_cnt1_field_info),
	c2h_mm_perf_mon_data_cnt1_field_info
},
{"C2H_MM_DBG", 0x10e8,
	1, 0, 0, 0,
	0, QDMA_MM_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_mm_dbg_field_info),
	c2h_mm_dbg_field_info
},
{"H2C_MM_CTL", 0x1204,
	1, 0, 0, 0,
	0, QDMA_MM_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(h2c_mm_ctl_field_info),
	h2c_mm_ctl_field_info
},
{"H2C_MM_STATUS", 0x1240,
	1, 0, 0, 0,
	0, QDMA_MM_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(h2c_mm_status_field_info),
	h2c_mm_status_field_info
},
{"H2C_MM_CMPL_DESC_CNT", 0x1248,
	1, 0, 0, 0,
	0, QDMA_MM_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(h2c_mm_cmpl_desc_cnt_field_info),
	h2c_mm_cmpl_desc_cnt_field_info
},
{"H2C_MM_ERR_CODE_ENABLE_MASK", 0x1254,
	1, 0, 0, 0,
	0, QDMA_MM_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(h2c_mm_err_code_enable_mask_field_info),
	h2c_mm_err_code_enable_mask_field_info
},
{"H2C_MM_ERR_CODE", 0x1258,
	1, 0, 0, 0,
	0, QDMA_MM_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(h2c_mm_err_code_field_info),
	h2c_mm_err_code_field_info
},
{"H2C_MM_ERR_INFO", 0x125c,
	1, 0, 0, 0,
	0, QDMA_MM_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(h2c_mm_err_info_field_info),
	h2c_mm_err_info_field_info
},
{"H2C_MM_PERF_MON_CTL", 0x12c0,
	1, 0, 0, 0,
	0, QDMA_MM_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(h2c_mm_perf_mon_ctl_field_info),
	h2c_mm_perf_mon_ctl_field_info
},
{"H2C_MM_PERF_MON_CYCLE_CNT0", 0x12c4,
	1, 0, 0, 0,
	0, QDMA_MM_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(h2c_mm_perf_mon_cycle_cnt0_field_info),
	h2c_mm_perf_mon_cycle_cnt0_field_info
},
{"H2C_MM_PERF_MON_CYCLE_CNT1", 0x12c8,
	1, 0, 0, 0,
	0, QDMA_MM_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(h2c_mm_perf_mon_cycle_cnt1_field_info),
	h2c_mm_perf_mon_cycle_cnt1_field_info
},
{"H2C_MM_PERF_MON_DATA_CNT0", 0x12cc,
	1, 0, 0, 0,
	0, QDMA_MM_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(h2c_mm_perf_mon_data_cnt0_field_info),
	h2c_mm_perf_mon_data_cnt0_field_info
},
{"H2C_MM_PERF_MON_DATA_CNT1", 0x12d0,
	1, 0, 0, 0,
	0, QDMA_MM_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(h2c_mm_perf_mon_data_cnt1_field_info),
	h2c_mm_perf_mon_data_cnt1_field_info
},
{"H2C_MM_DBG", 0x12e8,
	1, 0, 0, 0,
	0, QDMA_MM_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(h2c_mm_dbg_field_info),
	h2c_mm_dbg_field_info
},
{"H2C_MM_DATA_THROTTLE", 0x12ec,
	1, 0, 0, 0,
	0, QDMA_MM_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(h2c_mm_data_throttle_field_info),
	h2c_mm_data_throttle_field_info
},
{"C2H_CRDT_COAL_CFG_1", 0x1400,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_crdt_coal_cfg_1_field_info),
	c2h_crdt_coal_cfg_1_field_info
},
{"C2H_CRDT_COAL_CFG_2", 0x1404,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_crdt_coal_cfg_2_field_info),
	c2h_crdt_coal_cfg_2_field_info
},
{"C2H_PFCH_BYP_QID", 0x1408,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_pfch_byp_qid_field_info),
	c2h_pfch_byp_qid_field_info
},
{"C2H_PFCH_BYP_TAG", 0x140c,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_pfch_byp_tag_field_info),
	c2h_pfch_byp_tag_field_info
},
{"C2H_WATER_MARK", 0x1410,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_water_mark_field_info),
	c2h_water_mark_field_info
},
{"C2H_NOTIFY_EMPTY", 0x1450,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_notify_empty_field_info),
	c2h_notify_empty_field_info
},
{"C2H_STAT_S_AXIS_C2H_ACCEPTED_1", 0x1454,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_stat_s_axis_c2h_accepted_1_field_info),
	c2h_stat_s_axis_c2h_accepted_1_field_info
},
{"C2H_STAT_S_AXIS_WRB_ACCEPTED_1", 0x1458,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_stat_s_axis_wrb_accepted_1_field_info),
	c2h_stat_s_axis_wrb_accepted_1_field_info
},
{"C2H_STAT_DESC_RSP_PKT_ACCEPTED_1", 0x145c,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_stat_desc_rsp_pkt_accepted_1_field_info),
	c2h_stat_desc_rsp_pkt_accepted_1_field_info
},
{"C2H_STAT_AXIS_PKG_CMP_1", 0x1460,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_stat_axis_pkg_cmp_1_field_info),
	c2h_stat_axis_pkg_cmp_1_field_info
},
{"C2H_STAT_S_AXIS_WRB_ACCEPTED_2", 0x1464,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_stat_s_axis_wrb_accepted_2_field_info),
	c2h_stat_s_axis_wrb_accepted_2_field_info
},
{"C2H_ST_PLD_FIFO_DEPTH", 0x1468,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_st_pld_fifo_depth_field_info),
	c2h_st_pld_fifo_depth_field_info
},
{"C2H_STAT_DBG_DMA_ENG_6", 0x146c,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_stat_dbg_dma_eng_6_field_info),
	c2h_stat_dbg_dma_eng_6_field_info
},
{"C2H_STAT_DBG_DMA_ENG_7", 0x1470,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_stat_dbg_dma_eng_7_field_info),
	c2h_stat_dbg_dma_eng_7_field_info
},
{"C2H_STAT_PCIE_CMP_1", 0x1474,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_stat_pcie_cmp_1_field_info),
	c2h_stat_pcie_cmp_1_field_info
},
{"C2H_PLD_FIFO_ALMOST_FULL", 0x1478,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_pld_fifo_almost_full_field_info),
	c2h_pld_fifo_almost_full_field_info
},
{"PFCH_CFG_3", 0x147c,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(pfch_cfg_3_field_info),
	pfch_cfg_3_field_info
},
{"CMPT_CFG_0", 0x1480,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(cmpt_cfg_0_field_info),
	cmpt_cfg_0_field_info
},
{"PFCH_CFG_4", 0x1484,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(pfch_cfg_4_field_info),
	pfch_cfg_4_field_info
},

};
// MD: Debug print statement to track initialization
printk(KERN_DEBUG "Initialized eqdma_config_regs\n");



uint32_t eqdma_config_num_regs_get(void)
{
    uint32_t num_regs = sizeof(eqdma_config_regs) / sizeof(eqdma_config_regs[0]);
    printk(KERN_DEBUG "Number of EQDMA configuration registers: %u\n", num_regs);
    return num_regs;
}

struct xreg_info *eqdma_config_regs_get(void)
{
    printk(KERN_DEBUG "Returning pointer to EQDMA configuration registers\n");
    return eqdma_config_regs;
}
