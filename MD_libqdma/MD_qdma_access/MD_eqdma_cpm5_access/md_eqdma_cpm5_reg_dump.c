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

#include "eqdma_cpm5_reg.h" // MD: Include header for register definitions
#include "qdma_reg_dump.h"  // MD: Include header for register dump utilities

#ifdef ENABLE_WPP_TRACING
#include "eqdma_cpm5_reg_dump.tmh" // MD: Include tracing header if enabled
#endif

// MD: Structure to hold information about register fields
static struct regfield_info cfg_blk_identifier_field_info[] = {
    {"CFG_BLK_IDENTIFIER", CFG_BLK_IDENTIFIER_MASK},
    {"CFG_BLK_IDENTIFIER_1", CFG_BLK_IDENTIFIER_1_MASK},
    {"CFG_BLK_IDENTIFIER_RSVD_1", CFG_BLK_IDENTIFIER_RSVD_1_MASK},
    {"CFG_BLK_IDENTIFIER_VERSION", CFG_BLK_IDENTIFIER_VERSION_MASK},
};

// MD: Debug: Log the initialization of block identifier field info
qdma_log_debug("Initialized cfg_blk_identifier_field_info\n");

// MD: Structure to hold information about PCIe max payload size fields
static struct regfield_info cfg_blk_pcie_max_pld_size_field_info[] = {
    {"CFG_BLK_PCIE_MAX_PLD_SIZE_RSVD_1", CFG_BLK_PCIE_MAX_PLD_SIZE_RSVD_1_MASK},
    {"CFG_BLK_PCIE_MAX_PLD_SIZE_PROG", CFG_BLK_PCIE_MAX_PLD_SIZE_PROG_MASK},
    {"CFG_BLK_PCIE_MAX_PLD_SIZE_RSVD_2", CFG_BLK_PCIE_MAX_PLD_SIZE_RSVD_2_MASK},
    {"CFG_BLK_PCIE_MAX_PLD_SIZE_ISSUED", CFG_BLK_PCIE_MAX_PLD_SIZE_ISSUED_MASK},
};

// MD: Debug: Log the initialization of PCIe max payload size field info
qdma_log_debug("Initialized cfg_blk_pcie_max_pld_size_field_info\n");

// MD: Structure to hold information about PCIe max read request size fields
static struct regfield_info cfg_blk_pcie_max_read_req_size_field_info[] = {
    {"CFG_BLK_PCIE_MAX_READ_REQ_SIZE_RSVD_1", CFG_BLK_PCIE_MAX_READ_REQ_SIZE_RSVD_1_MASK},
    {"CFG_BLK_PCIE_MAX_READ_REQ_SIZE_PROG", CFG_BLK_PCIE_MAX_READ_REQ_SIZE_PROG_MASK},
    {"CFG_BLK_PCIE_MAX_READ_REQ_SIZE_RSVD_2", CFG_BLK_PCIE_MAX_READ_REQ_SIZE_RSVD_2_MASK},
    {"CFG_BLK_PCIE_MAX_READ_REQ_SIZE_ISSUED", CFG_BLK_PCIE_MAX_READ_REQ_SIZE_ISSUED_MASK},
};

// MD: Debug: Log the initialization of PCIe max read request size field info
qdma_log_debug("Initialized cfg_blk_pcie_max_read_req_size_field_info\n");

// MD: Structure to hold information about system ID fields
static struct regfield_info cfg_blk_system_id_field_info[] = {
    {"CFG_BLK_SYSTEM_ID_RSVD_1", CFG_BLK_SYSTEM_ID_RSVD_1_MASK},
    {"CFG_BLK_SYSTEM_ID_INST_TYPE", CFG_BLK_SYSTEM_ID_INST_TYPE_MASK},
    {"CFG_BLK_SYSTEM_ID", CFG_BLK_SYSTEM_ID_MASK},
};

// MD: Debug: Log the initialization of system ID field info
qdma_log_debug("Initialized cfg_blk_system_id_field_info\n");

// MD: Structure to hold information about MSIX enable fields
static struct regfield_info cfg_blk_msix_enable_field_info[] = {
    {"CFG_BLK_MSIX_ENABLE", CFG_BLK_MSIX_ENABLE_MASK},
};

// MD: Debug: Log the initialization of MSIX enable field info
qdma_log_debug("Initialized cfg_blk_msix_enable_field_info\n");

// MD: Structure to hold information about PCIe data width fields
static struct regfield_info cfg_pcie_data_width_field_info[] = {
    {"CFG_PCIE_DATA_WIDTH_RSVD_1", CFG_PCIE_DATA_WIDTH_RSVD_1_MASK},
    {"CFG_PCIE_DATA_WIDTH_DATAPATH", CFG_PCIE_DATA_WIDTH_DATAPATH_MASK},
};

// MD: Debug: Log the initialization of PCIe data width field info
qdma_log_debug("Initialized cfg_pcie_data_width_field_info\n");

// MD: Structure to hold information about PCIe control fields
static struct regfield_info cfg_pcie_ctl_field_info[] = {
    {"CFG_PCIE_CTL_RSVD_1", CFG_PCIE_CTL_RSVD_1_MASK},
    {"CFG_PCIE_CTL_MGMT_AXIL_CTRL", CFG_PCIE_CTL_MGMT_AXIL_CTRL_MASK},
    {"CFG_PCIE_CTL_RSVD_2", CFG_PCIE_CTL_RSVD_2_MASK},
    {"CFG_PCIE_CTL_RRQ_DISABLE", CFG_PCIE_CTL_RRQ_DISABLE_MASK},
    {"CFG_PCIE_CTL_RELAXED_ORDERING", CFG_PCIE_CTL_RELAXED_ORDERING_MASK},
};

// MD: Debug: Log the initialization of PCIe control field info
qdma_log_debug("Initialized cfg_pcie_ctl_field_info\n");

// MD: Structure to hold information about MSI enable fields
static struct regfield_info cfg_blk_msi_enable_field_info[] = {
    {"CFG_BLK_MSI_ENABLE", CFG_BLK_MSI_ENABLE_MASK},
};

// MD: Debug: Log the initialization of MSI enable field info
qdma_log_debug("Initialized cfg_blk_msi_enable_field_info\n");

// MD: Structure to hold information about AXI user max payload size fields
static struct regfield_info cfg_axi_user_max_pld_size_field_info[] = {
    {"CFG_AXI_USER_MAX_PLD_SIZE_RSVD_1", CFG_AXI_USER_MAX_PLD_SIZE_RSVD_1_MASK},
    {"CFG_AXI_USER_MAX_PLD_SIZE_ISSUED", CFG_AXI_USER_MAX_PLD_SIZE_ISSUED_MASK},
    {"CFG_AXI_USER_MAX_PLD_SIZE_RSVD_2", CFG_AXI_USER_MAX_PLD_SIZE_RSVD_2_MASK},
    {"CFG_AXI_USER_MAX_PLD_SIZE_PROG", CFG_AXI_USER_MAX_PLD_SIZE_PROG_MASK},
};

// MD: Debug: Log the initialization of AXI user max payload size field info
qdma_log_debug("Initialized cfg_axi_user_max_pld_size_field_info\n");

// MD: Structure to hold information about AXI user max read request size fields
static struct regfield_info cfg_axi_user_max_read_req_size_field_info[] = {
    {"CFG_AXI_USER_MAX_READ_REQ_SIZE_RSVD_1", CFG_AXI_USER_MAX_READ_REQ_SIZE_RSVD_1_MASK},
    {"CFG_AXI_USER_MAX_READ_REQ_SIZE_USISSUED", CFG_AXI_USER_MAX_READ_REQ_SIZE_USISSUED_MASK},
    {"CFG_AXI_USER_MAX_READ_REQ_SIZE_RSVD_2", CFG_AXI_USER_MAX_READ_REQ_SIZE_RSVD_2_MASK},
    {"CFG_AXI_USER_MAX_READ_REQ_SIZE_USPROG", CFG_AXI_USER_MAX_READ_REQ_SIZE_USPROG_MASK},
};

// MD: Debug: Log the initialization of AXI user max read request size field info
qdma_log_debug("Initialized cfg_axi_user_max_read_req_size_field_info\n");

// MD: Structure to hold information about miscellaneous control fields
static struct regfield_info cfg_blk_misc_ctl_field_info[] = {
    {"CFG_BLK_MISC_CTL_RSVD_1", CFG_BLK_MISC_CTL_RSVD_1_MASK},
    {"CFG_BLK_MISC_CTL_10B_TAG_EN", CFG_BLK_MISC_CTL_10B_TAG_EN_MASK},
    {"CFG_BLK_MISC_CTL_RSVD_2", CFG_BLK_MISC_CTL_RSVD_2_MASK},
    {"CFG_BLK_MISC_CTL_AXI_WBK", CFG_BLK_MISC_CTL_AXI_WBK_MASK},
    {"CFG_BLK_MISC_CTL_AXI_DSC", CFG_BLK_MISC_CTL_AXI_DSC_MASK},
    {"CFG_BLK_MISC_CTL_NUM_TAG", CFG_BLK_MISC_CTL_NUM_TAG_MASK},
    {"CFG_BLK_MISC_CTL_RSVD_3", CFG_BLK_MISC_CTL_RSVD_3_MASK},
    {"CFG_BLK_MISC_CTL_RQ_METERING_MULTIPLIER", CFG_BLK_MISC_CTL_RQ_METERING_MULTIPLIER_MASK},
};

// MD: Debug: Log the initialization of miscellaneous control field info
qdma_log_debug("Initialized cfg_blk_misc_ctl_field_info\n");

// MD: Structure to hold information about PL credit control fields
static struct regfield_info cfg_pl_cred_ctl_field_info[] = {
    {"CFG_PL_CRED_CTL_RSVD_1", CFG_PL_CRED_CTL_RSVD_1_MASK},
    {"CFG_PL_CRED_CTL_SLAVE_CRD_RLS", CFG_PL_CRED_CTL_SLAVE_CRD_RLS_MASK},
    {"CFG_PL_CRED_CTL_RSVD_2", CFG_PL_CRED_CTL_RSVD_2_MASK},
    {"CFG_PL_CRED_CTL_MASTER_CRD_RST", CFG_PL_CRED_CTL_MASTER_CRD_RST_MASK},
};

// MD: Debug: Log the initialization of PL credit control field info
qdma_log_debug("Initialized cfg_pl_cred_ctl_field_info\n");

// MD: Structure to hold information about scratch fields
static struct regfield_info cfg_blk_scratch_field_info[] = {
    {"CFG_BLK_SCRATCH", CFG_BLK_SCRATCH_MASK},
};

// MD: Debug: Log the initialization of scratch field info
qdma_log_debug("Initialized cfg_blk_scratch_field_info\n");

// MD: Structure to hold information about GIC fields
static struct regfield_info cfg_gic_field_info[] = {
    {"CFG_GIC_RSVD_1", CFG_GIC_RSVD_1_MASK},
    {"CFG_GIC_GIC_IRQ", CFG_GIC_GIC_IRQ_MASK},
};

// MD: Debug: Log the initialization of GIC field info
qdma_log_debug("Initialized cfg_gic_field_info\n");

// MD: Structure to hold information about RAM SBE mask fields
static struct regfield_info ram_sbe_msk_1_a_field_info[] = {
    {"RAM_SBE_MSK_1_A", RAM_SBE_MSK_1_A_MASK},
};

// MD: Debug: Log the initialization of RAM SBE mask field info
qdma_log_debug("Initialized ram_sbe_msk_1_a_field_info\n");

// MD: Structure to hold information about RAM SBE status fields for set 1 A
static struct regfield_info ram_sbe_sts_1_a_field_info[] = {
    {"RAM_SBE_STS_1_A_RSVD", RAM_SBE_STS_1_A_RSVD_MASK},
    {"RAM_SBE_STS_1_A_PFCH_CTXT_CAM_RAM_1", RAM_SBE_STS_1_A_PFCH_CTXT_CAM_RAM_1_MASK},
    {"RAM_SBE_STS_1_A_PFCH_CTXT_CAM_RAM_0", RAM_SBE_STS_1_A_PFCH_CTXT_CAM_RAM_0_MASK},
    {"RAM_SBE_STS_1_A_TAG_EVEN_RAM", RAM_SBE_STS_1_A_TAG_EVEN_RAM_MASK},
    {"RAM_SBE_STS_1_A_TAG_ODD_RAM", RAM_SBE_STS_1_A_TAG_ODD_RAM_MASK},
    {"RAM_SBE_STS_1_A_RC_RRQ_EVEN_RAM", RAM_SBE_STS_1_A_RC_RRQ_EVEN_RAM_MASK},
};

// MD: Debug: Log the initialization of RAM SBE status field info for set 1 A
qdma_log_debug("Initialized ram_sbe_sts_1_a_field_info\n");

// MD: Structure to hold information about RAM DBE mask fields for set 1 A
static struct regfield_info ram_dbe_msk_1_a_field_info[] = {
    {"RAM_DBE_MSK_1_A", RAM_DBE_MSK_1_A_MASK},
};

// MD: Debug: Log the initialization of RAM DBE mask field info for set 1 A
qdma_log_debug("Initialized ram_dbe_msk_1_a_field_info\n");

// MD: Structure to hold information about RAM DBE status fields for set 1 A
static struct regfield_info ram_dbe_sts_1_a_field_info[] = {
    {"RAM_DBE_STS_1_A_RSVD", RAM_DBE_STS_1_A_RSVD_MASK},
    {"RAM_DBE_STS_1_A_PFCH_CTXT_CAM_RAM_1", RAM_DBE_STS_1_A_PFCH_CTXT_CAM_RAM_1_MASK},
    {"RAM_DBE_STS_1_A_PFCH_CTXT_CAM_RAM_0", RAM_DBE_STS_1_A_PFCH_CTXT_CAM_RAM_0_MASK},
    {"RAM_DBE_STS_1_A_TAG_EVEN_RAM", RAM_DBE_STS_1_A_TAG_EVEN_RAM_MASK},
    {"RAM_DBE_STS_1_A_TAG_ODD_RAM", RAM_DBE_STS_1_A_TAG_ODD_RAM_MASK},
    {"RAM_DBE_STS_1_A_RC_RRQ_EVEN_RAM", RAM_DBE_STS_1_A_RC_RRQ_EVEN_RAM_MASK},
};

// MD: Debug: Log the initialization of RAM DBE status field info for set 1 A
qdma_log_debug("Initialized ram_dbe_sts_1_a_field_info\n");

// MD: Structure to hold information about RAM SBE mask fields for set A
static struct regfield_info ram_sbe_msk_a_field_info[] = {
    {"RAM_SBE_MSK_A", RAM_SBE_MSK_A_MASK},
};

// MD: Debug: Log the initialization of RAM SBE mask field info for set A
qdma_log_debug("Initialized ram_sbe_msk_a_field_info\n");

// MD: Structure to hold information about RAM SBE status fields for set A
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

// MD: Debug: Log the initialization of RAM SBE status field info for set A
qdma_log_debug("Initialized ram_sbe_sts_a_field_info\n");

// MD: Structure to hold information about RAM DBE mask fields for set A
static struct regfield_info ram_dbe_msk_a_field_info[] = {
    {"RAM_DBE_MSK_A", RAM_DBE_MSK_A_MASK},
};

// MD: Debug: Log the initialization of RAM DBE mask field info for set A
qdma_log_debug("Initialized ram_dbe_msk_a_field_info\n");

// MD: Structure to hold information about RAM DBE status fields for set A
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

// MD: Debug: Log the initialization of RAM DBE status field info for set A
qdma_log_debug("Initialized ram_dbe_sts_a_field_info\n");

// MD: Structure to hold information about global identifier fields
static struct regfield_info glbl2_identifier_field_info[] = {
    {"GLBL2_IDENTIFIER", GLBL2_IDENTIFIER_MASK},
    {"GLBL2_IDENTIFIER_VERSION", GLBL2_IDENTIFIER_VERSION_MASK},
};

// MD: Debug: Log the initialization of global identifier field info
qdma_log_debug("Initialized glbl2_identifier_field_info\n");

// MD: Structure to hold information about global channel instance fields
static struct regfield_info glbl2_channel_inst_field_info[] = {
    {"GLBL2_CHANNEL_INST_RSVD_1", GLBL2_CHANNEL_INST_RSVD_1_MASK},
    {"GLBL2_CHANNEL_INST_C2H_ST", GLBL2_CHANNEL_INST_C2H_ST_MASK},
    {"GLBL2_CHANNEL_INST_H2C_ST", GLBL2_CHANNEL_INST_H2C_ST_MASK},
    {"GLBL2_CHANNEL_INST_RSVD_2", GLBL2_CHANNEL_INST_RSVD_2_MASK},
    {"GLBL2_CHANNEL_INST_C2H_ENG", GLBL2_CHANNEL_INST_C2H_ENG_MASK},
    {"GLBL2_CHANNEL_INST_RSVD_3", GLBL2_CHANNEL_INST_RSVD_3_MASK},
    {"GLBL2_CHANNEL_INST_H2C_ENG", GLBL2_CHANNEL_INST_H2C_ENG_MASK},
};

// MD: Debug: Log the initialization of global channel instance field info
qdma_log_debug("Initialized glbl2_channel_inst_field_info\n");

// MD: Structure to hold information about global channel MDMA fields
static struct regfield_info glbl2_channel_mdma_field_info[] = {
    {"GLBL2_CHANNEL_MDMA_RSVD_1", GLBL2_CHANNEL_MDMA_RSVD_1_MASK},
    {"GLBL2_CHANNEL_MDMA_C2H_ST", GLBL2_CHANNEL_MDMA_C2H_ST_MASK},
    {"GLBL2_CHANNEL_MDMA_H2C_ST", GLBL2_CHANNEL_MDMA_H2C_ST_MASK},
    {"GLBL2_CHANNEL_MDMA_RSVD_2", GLBL2_CHANNEL_MDMA_RSVD_2_MASK},
    {"GLBL2_CHANNEL_MDMA_C2H_ENG", GLBL2_CHANNEL_MDMA_C2H_ENG_MASK},
    {"GLBL2_CHANNEL_MDMA_RSVD_3", GLBL2_CHANNEL_MDMA_RSVD_3_MASK},
    {"GLBL2_CHANNEL_MDMA_H2C_ENG", GLBL2_CHANNEL_MDMA_H2C_ENG_MASK},
};

// MD: Debug: Log the initialization of global channel MDMA field info
qdma_log_debug("Initialized glbl2_channel_mdma_field_info\n");

// MD: Structure to hold information about global channel stream fields
static struct regfield_info glbl2_channel_strm_field_info[] = {
    {"GLBL2_CHANNEL_STRM_RSVD_1", GLBL2_CHANNEL_STRM_RSVD_1_MASK},
    {"GLBL2_CHANNEL_STRM_C2H_ST", GLBL2_CHANNEL_STRM_C2H_ST_MASK},
    {"GLBL2_CHANNEL_STRM_H2C_ST", GLBL2_CHANNEL_STRM_H2C_ST_MASK},
    {"GLBL2_CHANNEL_STRM_RSVD_2", GLBL2_CHANNEL_STRM_RSVD_2_MASK},
    {"GLBL2_CHANNEL_STRM_C2H_ENG", GLBL2_CHANNEL_STRM_C2H_ENG_MASK},
    {"GLBL2_CHANNEL_STRM_RSVD_3", GLBL2_CHANNEL_STRM_RSVD_3_MASK},
    {"GLBL2_CHANNEL_STRM_H2C_ENG", GLBL2_CHANNEL_STRM_H2C_ENG_MASK},
};

// MD: Debug: Log the initialization of global channel stream field info
qdma_log_debug("Initialized glbl2_channel_strm_field_info\n");

// MD: Structure to hold information about global channel capability fields
static struct regfield_info glbl2_channel_cap_field_info[] = {
    {"GLBL2_CHANNEL_CAP_RSVD_1", GLBL2_CHANNEL_CAP_RSVD_1_MASK},
    {"GLBL2_CHANNEL_CAP_MULTIQ_MAX", GLBL2_CHANNEL_CAP_MULTIQ_MAX_MASK},
};

// MD: Debug: Log the initialization of global channel capability field info
qdma_log_debug("Initialized glbl2_channel_cap_field_info\n");

// MD: Structure to hold information about global channel PASID capability fields
static struct regfield_info glbl2_channel_pasid_cap_field_info[] = {
    {"GLBL2_CHANNEL_PASID_CAP_RSVD_1", GLBL2_CHANNEL_PASID_CAP_RSVD_1_MASK},
    {"GLBL2_CHANNEL_PASID_CAP_BRIDGEEN", GLBL2_CHANNEL_PASID_CAP_BRIDGEEN_MASK},
    {"GLBL2_CHANNEL_PASID_CAP_DMAEN", GLBL2_CHANNEL_PASID_CAP_DMAEN_MASK},
};

// MD: Debug: Log the initialization of global channel PASID capability field info
qdma_log_debug("Initialized glbl2_channel_pasid_cap_field_info\n");

// MD: Structure to hold information about global system ID fields
static struct regfield_info glbl2_system_id_field_info[] = {
    {"GLBL2_SYSTEM_ID_RSVD_1", GLBL2_SYSTEM_ID_RSVD_1_MASK},
    {"GLBL2_SYSTEM_ID", GLBL2_SYSTEM_ID_MASK},
};

// MD: Debug: Log the initialization of global system ID field info
qdma_log_debug("Initialized glbl2_system_id_field_info\n");

// MD: Structure to hold information about global miscellaneous capability fields
static struct regfield_info glbl2_misc_cap_field_info[] = {
    {"GLBL2_MISC_CAP", GLBL2_MISC_CAP_MASK},
};

// MD: Debug: Log the initialization of global miscellaneous capability field info
qdma_log_debug("Initialized glbl2_misc_cap_field_info\n");

// MD: Structure to hold information about global debug PCIe RQ0 fields
static struct regfield_info glbl2_dbg_pcie_rq0_field_info[] = {
    {"GLBL2_PCIE_RQ0_NPH_AVL", GLBL2_PCIE_RQ0_NPH_AVL_MASK},
    {"GLBL2_PCIE_RQ0_RCB_AVL", GLBL2_PCIE_RQ0_RCB_AVL_MASK},
    {"GLBL2_PCIE_RQ0_SLV_RD_CREDS", GLBL2_PCIE_RQ0_SLV_RD_CREDS_MASK},
    {"GLBL2_PCIE_RQ0_TAG_EP", GLBL2_PCIE_RQ0_TAG_EP_MASK},
};

// MD: Debug: Log the initialization of global debug PCIe RQ0 field info
qdma_log_debug("Initialized glbl2_dbg_pcie_rq0_field_info\n");

// MD: Structure to hold information about global debug PCIe RQ1 fields
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

// MD: Debug: Log the initialization of global debug PCIe RQ1 field info
qdma_log_debug("Initialized glbl2_dbg_pcie_rq1_field_info\n");

// MD: Structure to hold information about global debug AXIMM WR0 fields
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

// MD: Debug: Log the initialization of global debug AXIMM WR0 field info
qdma_log_debug("Initialized glbl2_dbg_aximm_wr0_field_info\n");

// MD: Structure to hold information about global debug AXIMM WR1 fields
static struct regfield_info glbl2_dbg_aximm_wr1_field_info[] = {
    {"GLBL2_AXIMM_WR1_RSVD_1", GLBL2_AXIMM_WR1_RSVD_1_MASK},
    {"GLBL2_AXIMM_WR1_BRSP_CNT4", GLBL2_AXIMM_WR1_BRSP_CNT4_MASK},
    {"GLBL2_AXIMM_WR1_BRSP_CNT3", GLBL2_AXIMM_WR1_BRSP_CNT3_MASK},
    {"GLBL2_AXIMM_WR1_BRSP_CNT2", GLBL2_AXIMM_WR1_BRSP_CNT2_MASK},
    {"GLBL2_AXIMM_WR1_BRSP_CNT1", GLBL2_AXIMM_WR1_BRSP_CNT1_MASK},
    {"GLBL2_AXIMM_WR1_BRSP_CNT0", GLBL2_AXIMM_WR1_BRSP_CNT0_MASK},
};

// MD: Debug: Log the initialization of global debug AXIMM WR1 field info
qdma_log_debug("Initialized glbl2_dbg_aximm_wr1_field_info\n");

// MD: Structure to hold information about global debug AXIMM RD0 fields
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

// MD: Debug: Log the initialization of global debug AXIMM RD0 field info
qdma_log_debug("Initialized glbl2_dbg_aximm_rd0_field_info\n");

// MD: Structure to hold information about global debug AXIMM RD1 fields
static struct regfield_info glbl2_dbg_aximm_rd1_field_info[] = {
    {"GLBL2_AXIMM_RD1_RSVD_1", GLBL2_AXIMM_RD1_RSVD_1_MASK},
    {"GLBL2_AXIMM_RD1_RRSP_CNT4", GLBL2_AXIMM_RD1_RRSP_CNT4_MASK},
    {"GLBL2_AXIMM_RD1_RRSP_CNT3", GLBL2_AXIMM_RD1_RRSP_CNT3_MASK},
    {"GLBL2_AXIMM_RD1_RRSP_CNT2", GLBL2_AXIMM_RD1_RRSP_CNT2_MASK},
    {"GLBL2_AXIMM_RD1_RRSP_CNT1", GLBL2_AXIMM_RD1_RRSP_CNT1_MASK},
    {"GLBL2_AXIMM_RD1_RRSP_CNT0", GLBL2_AXIMM_RD1_RRSP_CNT0_MASK},
};

// MD: Debug: Log the initialization of global debug AXIMM RD1 field info
qdma_log_debug("Initialized glbl2_dbg_aximm_rd1_field_info\n");

// MD: Structure to hold information about global debug FAB0 fields
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

// MD: Debug: Log the initialization of global debug FAB0 field info
qdma_log_debug("Initialized glbl2_dbg_fab0_field_info\n");

// MD: Structure to hold information about global debug FAB1 fields
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

// MD: Debug: Log the initialization of global debug FAB1 field info
qdma_log_debug("Initialized glbl2_dbg_fab1_field_info\n");

// MD: Structure to hold information about global debug match select fields
static struct regfield_info glbl2_dbg_match_sel_field_info[] = {
    {"GLBL2_MATCH_SEL_RSV", GLBL2_MATCH_SEL_RSV_MASK},
    {"GLBL2_MATCH_SEL_CSR_SEL", GLBL2_MATCH_SEL_CSR_SEL_MASK},
    {"GLBL2_MATCH_SEL_CSR_EN", GLBL2_MATCH_SEL_CSR_EN_MASK},
    {"GLBL2_MATCH_SEL_ROTATE1", GLBL2_MATCH_SEL_ROTATE1_MASK},
    {"GLBL2_MATCH_SEL_ROTATE0", GLBL2_MATCH_SEL_ROTATE0_MASK},
    {"GLBL2_MATCH_SEL_SEL", GLBL2_MATCH_SEL_SEL_MASK},
};

// MD: Debug: Log the initialization of global debug match select field info
qdma_log_debug("Initialized glbl2_dbg_match_sel_field_info\n");

// MD: Structure to hold information about global debug match mask fields
static struct regfield_info glbl2_dbg_match_msk_field_info[] = {
    {"GLBL2_MATCH_MSK", GLBL2_MATCH_MSK_MASK},
};

// MD: Debug: Log the initialization of global debug match mask field info
qdma_log_debug("Initialized glbl2_dbg_match_msk_field_info\n");

// MD: Structure to hold information about global debug match pattern fields
static struct regfield_info glbl2_dbg_match_pat_field_info[] = {
    {"GLBL2_MATCH_PAT_PATTERN", GLBL2_MATCH_PAT_PATTERN_MASK},
};

// MD: Debug: Log the initialization of global debug match pattern field info
qdma_log_debug("Initialized glbl2_dbg_match_pat_field_info\n");

// MD: Structure to hold information about global ring size 1 fields
static struct regfield_info glbl_rng_sz_1_field_info[] = {
    {"GLBL_RNG_SZ_1_RSVD_1", GLBL_RNG_SZ_1_RSVD_1_MASK},
    {"GLBL_RNG_SZ_1_RING_SIZE", GLBL_RNG_SZ_1_RING_SIZE_MASK},
};

// MD: Debug: Log the initialization of global ring size 1 field info
qdma_log_debug("Initialized glbl_rng_sz_1_field_info\n");

// MD: Structure to hold information about global ring size 2 fields
static struct regfield_info glbl_rng_sz_2_field_info[] = {
    {"GLBL_RNG_SZ_2_RSVD_1", GLBL_RNG_SZ_2_RSVD_1_MASK},
    {"GLBL_RNG_SZ_2_RING_SIZE", GLBL_RNG_SZ_2_RING_SIZE_MASK},
};

// MD: Debug: Log the initialization of global ring size 2 field info
qdma_log_debug("Initialized glbl_rng_sz_2_field_info\n");

// MD: Structure to hold information about global ring size 3 fields
static struct regfield_info glbl_rng_sz_3_field_info[] = {
    {"GLBL_RNG_SZ_3_RSVD_1", GLBL_RNG_SZ_3_RSVD_1_MASK},
    {"GLBL_RNG_SZ_3_RING_SIZE", GLBL_RNG_SZ_3_RING_SIZE_MASK},
};

// MD: Debug: Log the initialization of global ring size 3 field info
qdma_log_debug("Initialized glbl_rng_sz_3_field_info\n");

// MD: Structure to hold information about global ring size 4 fields
static struct regfield_info glbl_rng_sz_4_field_info[] = {
    {"GLBL_RNG_SZ_4_RSVD_1", GLBL_RNG_SZ_4_RSVD_1_MASK},
    {"GLBL_RNG_SZ_4_RING_SIZE", GLBL_RNG_SZ_4_RING_SIZE_MASK},
};

// MD: Debug: Log the initialization of global ring size 4 field info
qdma_log_debug("Initialized glbl_rng_sz_4_field_info\n");

// MD: Structure to hold information about global ring size 5 fields
static struct regfield_info glbl_rng_sz_5_field_info[] = {
    {"GLBL_RNG_SZ_5_RSVD_1", GLBL_RNG_SZ_5_RSVD_1_MASK},
    {"GLBL_RNG_SZ_5_RING_SIZE", GLBL_RNG_SZ_5_RING_SIZE_MASK},
};

// MD: Debug: Log the initialization of global ring size 5 field info
qdma_log_debug("Initialized glbl_rng_sz_5_field_info\n");

// MD: Structure to hold information about global ring size 6 fields
static struct regfield_info glbl_rng_sz_6_field_info[] = {
    {"GLBL_RNG_SZ_6_RSVD_1", GLBL_RNG_SZ_6_RSVD_1_MASK},
    {"GLBL_RNG_SZ_6_RING_SIZE", GLBL_RNG_SZ_6_RING_SIZE_MASK},
};

// MD: Debug: Log the initialization of global ring size 6 field info
qdma_log_debug("Initialized glbl_rng_sz_6_field_info\n");

// MD: Structure to hold information about global ring size 7 fields
static struct regfield_info glbl_rng_sz_7_field_info[] = {
    {"GLBL_RNG_SZ_7_RSVD_1", GLBL_RNG_SZ_7_RSVD_1_MASK},
    {"GLBL_RNG_SZ_7_RING_SIZE", GLBL_RNG_SZ_7_RING_SIZE_MASK},
};

// MD: Debug: Log the initialization of global ring size 7 field info
qdma_log_debug("Initialized glbl_rng_sz_7_field_info\n");

// MD: Structure to hold information about global ring size 8 fields
static struct regfield_info glbl_rng_sz_8_field_info[] = {
    {"GLBL_RNG_SZ_8_RSVD_1", GLBL_RNG_SZ_8_RSVD_1_MASK},
    {"GLBL_RNG_SZ_8_RING_SIZE", GLBL_RNG_SZ_8_RING_SIZE_MASK},
};

// MD: Debug: Log the initialization of global ring size 8 field info
qdma_log_debug("Initialized glbl_rng_sz_8_field_info\n");

// MD: Structure to hold information about global ring size 9 fields
static struct regfield_info glbl_rng_sz_9_field_info[] = {
    {"GLBL_RNG_SZ_9_RSVD_1", GLBL_RNG_SZ_9_RSVD_1_MASK},
    {"GLBL_RNG_SZ_9_RING_SIZE", GLBL_RNG_SZ_9_RING_SIZE_MASK},
};

// MD: Debug: Log the initialization of global ring size 9 field info
qdma_log_debug("Initialized glbl_rng_sz_9_field_info\n");

// MD: Structure to hold information about global ring size A fields
static struct regfield_info glbl_rng_sz_a_field_info[] = {
    {"GLBL_RNG_SZ_A_RSVD_1", GLBL_RNG_SZ_A_RSVD_1_MASK},
    {"GLBL_RNG_SZ_A_RING_SIZE", GLBL_RNG_SZ_A_RING_SIZE_MASK},
};

// MD: Debug: Log the initialization of global ring size A field info
qdma_log_debug("Initialized glbl_rng_sz_a_field_info\n");

// MD: Structure to hold information about global ring size B fields
static struct regfield_info glbl_rng_sz_b_field_info[] = {
    {"GLBL_RNG_SZ_B_RSVD_1", GLBL_RNG_SZ_B_RSVD_1_MASK},
    {"GLBL_RNG_SZ_B_RING_SIZE", GLBL_RNG_SZ_B_RING_SIZE_MASK},
};

// MD: Debug: Log the initialization of global ring size B field info
qdma_log_debug("Initialized glbl_rng_sz_b_field_info\n");

// MD: Structure to hold information about global ring size C fields
static struct regfield_info glbl_rng_sz_c_field_info[] = {
    {"GLBL_RNG_SZ_C_RSVD_1", GLBL_RNG_SZ_C_RSVD_1_MASK},
    {"GLBL_RNG_SZ_C_RING_SIZE", GLBL_RNG_SZ_C_RING_SIZE_MASK},
};

// MD: Debug: Log the initialization of global ring size C field info
qdma_log_debug("Initialized glbl_rng_sz_c_field_info\n");

// MD: Structure to hold information about global ring size D fields
static struct regfield_info glbl_rng_sz_d_field_info[] = {
    {"GLBL_RNG_SZ_D_RSVD_1", GLBL_RNG_SZ_D_RSVD_1_MASK},
    {"GLBL_RNG_SZ_D_RING_SIZE", GLBL_RNG_SZ_D_RING_SIZE_MASK},
};

// MD: Debug: Log the initialization of global ring size D field info
qdma_log_debug("Initialized glbl_rng_sz_d_field_info\n");

// MD: Structure to hold information about global ring size E fields
static struct regfield_info glbl_rng_sz_e_field_info[] = {
    {"GLBL_RNG_SZ_E_RSVD_1", GLBL_RNG_SZ_E_RSVD_1_MASK},
    {"GLBL_RNG_SZ_E_RING_SIZE", GLBL_RNG_SZ_E_RING_SIZE_MASK},
};

// MD: Debug: Log the initialization of global ring size E field info
qdma_log_debug("Initialized glbl_rng_sz_e_field_info\n");

// MD: Structure to hold information about global ring size F fields
static struct regfield_info glbl_rng_sz_f_field_info[] = {
    {"GLBL_RNG_SZ_F_RSVD_1", GLBL_RNG_SZ_F_RSVD_1_MASK},
    {"GLBL_RNG_SZ_F_RING_SIZE", GLBL_RNG_SZ_F_RING_SIZE_MASK},
};

// MD: Debug: Log the initialization of global ring size F field info
qdma_log_debug("Initialized glbl_rng_sz_f_field_info\n");

// MD: Structure to hold information about global ring size 10 fields
static struct regfield_info glbl_rng_sz_10_field_info[] = {
    {"GLBL_RNG_SZ_10_RSVD_1", GLBL_RNG_SZ_10_RSVD_1_MASK},
    {"GLBL_RNG_SZ_10_RING_SIZE", GLBL_RNG_SZ_10_RING_SIZE_MASK},
};

// MD: Debug: Log the initialization of global ring size 10 field info
qdma_log_debug("Initialized glbl_rng_sz_10_field_info\n");

// MD: Structure to hold information about global error status fields
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

// MD: Debug: Log the initialization of global error status field info
qdma_log_debug("Initialized glbl_err_stat_field_info\n");

// MD: Structure to hold information about global error mask fields
static struct regfield_info glbl_err_mask_field_info[] = {
    {"GLBL_ERR", GLBL_ERR_MASK},
};

// MD: Debug: Log the initialization of global error mask field info
qdma_log_debug("Initialized glbl_err_mask_field_info\n");

// MD: Structure to hold information about global descriptor configuration fields
static struct regfield_info glbl_dsc_cfg_field_info[] = {
    {"GLBL_DSC_CFG_RSVD_1", GLBL_DSC_CFG_RSVD_1_MASK},
    {"GLBL_DSC_CFG_UNC_OVR_COR", GLBL_DSC_CFG_UNC_OVR_COR_MASK},
    {"GLBL_DSC_CFG_CTXT_FER_DIS", GLBL_DSC_CFG_CTXT_FER_DIS_MASK},
    {"GLBL_DSC_CFG_RSVD_2", GLBL_DSC_CFG_RSVD_2_MASK},
    {"GLBL_DSC_CFG_MAXFETCH", GLBL_DSC_CFG_MAXFETCH_MASK},
    {"GLBL_DSC_CFG_WB_ACC_INT", GLBL_DSC_CFG_WB_ACC_INT_MASK},
};

// MD: Debug: Log the initialization of global descriptor configuration field info
qdma_log_debug("Initialized glbl_dsc_cfg_field_info\n");

// MD: Structure to hold information about global descriptor error status fields
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

// MD: Debug: Log the initialization of global descriptor error status field info
qdma_log_debug("Initialized glbl_dsc_err_sts_field_info\n");

// MD: Structure to hold information about global descriptor error mask fields
static struct regfield_info glbl_dsc_err_msk_field_info[] = {
    {"GLBL_DSC_ERR_MSK", GLBL_DSC_ERR_MSK_MASK},
};

// MD: Debug: Log the initialization of global descriptor error mask field info
qdma_log_debug("Initialized glbl_dsc_err_msk_field_info\n");

// MD: Structure to hold information about global descriptor error log0 fields
static struct regfield_info glbl_dsc_err_log0_field_info[] = {
    {"GLBL_DSC_ERR_LOG0_VALID", GLBL_DSC_ERR_LOG0_VALID_MASK},
    {"GLBL_DSC_ERR_LOG0_SEL", GLBL_DSC_ERR_LOG0_SEL_MASK},
    {"GLBL_DSC_ERR_LOG0_RSVD_1", GLBL_DSC_ERR_LOG0_RSVD_1_MASK},
    {"GLBL_DSC_ERR_LOG0_QID", GLBL_DSC_ERR_LOG0_QID_MASK},
};

// MD: Debug: Log the initialization of global descriptor error log0 field info
qdma_log_debug("Initialized glbl_dsc_err_log0_field_info\n");

// MD: Structure to hold information about global descriptor error log1 fields
static struct regfield_info glbl_dsc_err_log1_field_info[] = {
    {"GLBL_DSC_ERR_LOG1_RSVD_1", GLBL_DSC_ERR_LOG1_RSVD_1_MASK},
    {"GLBL_DSC_ERR_LOG1_CIDX", GLBL_DSC_ERR_LOG1_CIDX_MASK},
    {"GLBL_DSC_ERR_LOG1_RSVD_2", GLBL_DSC_ERR_LOG1_RSVD_2_MASK},
    {"GLBL_DSC_ERR_LOG1_SUB_TYPE", GLBL_DSC_ERR_LOG1_SUB_TYPE_MASK},
    {"GLBL_DSC_ERR_LOG1_ERR_TYPE", GLBL_DSC_ERR_LOG1_ERR_TYPE_MASK},
};

// MD: Debug: Log the initialization of global descriptor error log1 field info
qdma_log_debug("Initialized glbl_dsc_err_log1_field_info\n");

// MD: Structure to hold information about global transaction queue error status fields
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

// MD: Debug: Log the initialization of global transaction queue error status field info
qdma_log_debug("Initialized glbl_trq_err_sts_field_info\n");

// MD: Structure to hold information about global transaction queue error mask fields
static struct regfield_info glbl_trq_err_msk_field_info[] = {
    {"GLBL_TRQ_ERR_MSK", GLBL_TRQ_ERR_MSK_MASK},
};

// MD: Debug: Log the initialization of global transaction queue error mask field info
qdma_log_debug("Initialized glbl_trq_err_msk_field_info\n");

// MD: Structure to hold information about global transaction queue error log fields
static struct regfield_info glbl_trq_err_log_field_info[] = {
    {"GLBL_TRQ_ERR_LOG_SRC", GLBL_TRQ_ERR_LOG_SRC_MASK},
    {"GLBL_TRQ_ERR_LOG_TARGET", GLBL_TRQ_ERR_LOG_TARGET_MASK},
    {"GLBL_TRQ_ERR_LOG_FUNC", GLBL_TRQ_ERR_LOG_FUNC_MASK},
    {"GLBL_TRQ_ERR_LOG_ADDRESS", GLBL_TRQ_ERR_LOG_ADDRESS_MASK},
};

// MD: Debug: Log the initialization of global transaction queue error log field info
qdma_log_debug("Initialized glbl_trq_err_log_field_info\n");

// MD: Structure to hold information about global descriptor debug data0 fields
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

// MD: Debug: Log the initialization of global descriptor debug data0 field info
qdma_log_debug("Initialized glbl_dsc_dbg_dat0_field_info\n");

// MD: Structure to hold information about global descriptor debug data1 fields
static struct regfield_info glbl_dsc_dbg_dat1_field_info[] = {
    {"GLBL_DSC_DAT1_RSVD_1", GLBL_DSC_DAT1_RSVD_1_MASK},
    {"GLBL_DSC_DAT1_EVT_SPC_C2H", GLBL_DSC_DAT1_EVT_SPC_C2H_MASK},
    {"GLBL_DSC_DAT1_EVT_SP_H2C", GLBL_DSC_DAT1_EVT_SP_H2C_MASK},
    {"GLBL_DSC_DAT1_DSC_SPC_C2H", GLBL_DSC_DAT1_DSC_SPC_C2H_MASK},
    {"GLBL_DSC_DAT1_DSC_SPC_H2C", GLBL_DSC_DAT1_DSC_SPC_H2C_MASK},
};

// MD: Debug: Log the initialization of global descriptor debug data1 field info
qdma_log_debug("Initialized glbl_dsc_dbg_dat1_field_info\n");

// MD: Structure to hold information about global descriptor debug control fields
static struct regfield_info glbl_dsc_dbg_ctl_field_info[] = {
    {"GLBL_DSC_CTL_RSVD_1", GLBL_DSC_CTL_RSVD_1_MASK},
    {"GLBL_DSC_CTL_SELECT", GLBL_DSC_CTL_SELECT_MASK},
};

// MD: Debug: Log the initialization of global descriptor debug control field info
qdma_log_debug("Initialized glbl_dsc_dbg_ctl_field_info\n");

// MD: Structure to hold information about global descriptor error log2 fields
static struct regfield_info glbl_dsc_err_log2_field_info[] = {
    {"GLBL_DSC_ERR_LOG2_OLD_PIDX", GLBL_DSC_ERR_LOG2_OLD_PIDX_MASK},
    {"GLBL_DSC_ERR_LOG2_NEW_PIDX", GLBL_DSC_ERR_LOG2_NEW_PIDX_MASK},
};

// MD: Debug: Log the initialization of global descriptor error log2 field info
qdma_log_debug("Initialized glbl_dsc_err_log2_field_info\n");

// MD: Structure to hold information about global interrupt configuration fields
static struct regfield_info glbl_glbl_interrupt_cfg_field_info[] = {
    {"GLBL_GLBL_INTERRUPT_CFG_RSVD_1", GLBL_GLBL_INTERRUPT_CFG_RSVD_1_MASK},
    {"GLBL_GLBL_INTERRUPT_CFG_LGCY_INTR_PENDING", GLBL_GLBL_INTERRUPT_CFG_LGCY_INTR_PENDING_MASK},
    {"GLBL_GLBL_INTERRUPT_CFG_EN_LGCY_INTR", GLBL_GLBL_INTERRUPT_CFG_EN_LGCY_INTR_MASK},
};

// MD: Debug: Log the initialization of global interrupt configuration field info
qdma_log_debug("Initialized glbl_glbl_interrupt_cfg_field_info\n");

// MD: Structure to hold information about virtual channel host profile fields
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

// MD: Debug: Log the initialization of virtual channel host profile field info
qdma_log_debug("Initialized glbl_vch_host_profile_field_info\n");

// MD: Structure to hold information about bridge host profile fields
static struct regfield_info glbl_bridge_host_profile_field_info[] = {
    {"GLBL_BRIDGE_HOST_PROFILE_RSVD_1", GLBL_BRIDGE_HOST_PROFILE_RSVD_1_MASK},
    {"GLBL_BRIDGE_HOST_PROFILE_BDGID", GLBL_BRIDGE_HOST_PROFILE_BDGID_MASK},
};

// MD: Debug: Log the initialization of bridge host profile field info
qdma_log_debug("Initialized glbl_bridge_host_profile_field_info\n");

// MD: Structure to hold information about AXIMM IRQ destination address fields
static struct regfield_info aximm_irq_dest_addr_field_info[] = {
    {"AXIMM_IRQ_DEST_ADDR_ADDR", AXIMM_IRQ_DEST_ADDR_ADDR_MASK},
};

// MD: Debug: Log the initialization of AXIMM IRQ destination address field info
qdma_log_debug("Initialized aximm_irq_dest_addr_field_info\n");

// MD: Structure to hold information about fabric error log fields
static struct regfield_info fab_err_log_field_info[] = {
    {"FAB_ERR_LOG_RSVD_1", FAB_ERR_LOG_RSVD_1_MASK},
    {"FAB_ERR_LOG_SRC", FAB_ERR_LOG_SRC_MASK},
};

// MD: Debug: Log the initialization of fabric error log field info
qdma_log_debug("Initialized fab_err_log_field_info\n");

// MD: Structure to hold information about indirect context data fields
static struct regfield_info ind_ctxt_data_field_info[] = {
    {"IND_CTXT_DATA_DATA", IND_CTXT_DATA_DATA_MASK},
};

// MD: Debug: Log the initialization of indirect context data field info
qdma_log_debug("Initialized ind_ctxt_data_field_info\n");

// MD: Structure to hold information about indirect context mask fields
static struct regfield_info ind_ctxt_mask_field_info[] = {
    {"IND_CTXT", IND_CTXT_MASK},
};

// MD: Debug: Log the initialization of indirect context mask field info
qdma_log_debug("Initialized ind_ctxt_mask_field_info\n");

// MD: Structure to hold information about indirect context command fields
static struct regfield_info ind_ctxt_cmd_field_info[] = {
    {"IND_CTXT_CMD_RSVD_1", IND_CTXT_CMD_RSVD_1_MASK},
    {"IND_CTXT_CMD_QID", IND_CTXT_CMD_QID_MASK},
    {"IND_CTXT_CMD_OP", IND_CTXT_CMD_OP_MASK},
    {"IND_CTXT_CMD_SEL", IND_CTXT_CMD_SEL_MASK},
    {"IND_CTXT_CMD_BUSY", IND_CTXT_CMD_BUSY_MASK},
};

// MD: Debug: Log the initialization of indirect context command field info
qdma_log_debug("Initialized ind_ctxt_cmd_field_info\n");

// MD: Structure to hold information about C2H timer count fields
static struct regfield_info c2h_timer_cnt_field_info[] = {
    {"C2H_TIMER_CNT_RSVD_1", C2H_TIMER_CNT_RSVD_1_MASK},
    {"C2H_TIMER_CNT", C2H_TIMER_CNT_MASK},
};

// MD: Debug: Log the initialization of C2H timer count field info
qdma_log_debug("Initialized c2h_timer_cnt_field_info\n");

// MD: Structure to hold information about C2H count threshold fields
static struct regfield_info c2h_cnt_th_field_info[] = {
    {"C2H_CNT_TH_RSVD_1", C2H_CNT_TH_RSVD_1_MASK},
    {"C2H_CNT_TH_THESHOLD_CNT", C2H_CNT_TH_THESHOLD_CNT_MASK},
};

// MD: Debug: Log the initialization of C2H count threshold field info
qdma_log_debug("Initialized c2h_cnt_th_field_info\n");

// MD: Structure to hold information about C2H status S_AXIS C2H accepted fields
static struct regfield_info c2h_stat_s_axis_c2h_accepted_field_info[] = {
    {"C2H_STAT_S_AXIS_C2H_ACCEPTED_RSVD_1", C2H_STAT_S_AXIS_C2H_ACCEPTED_RSVD_1_MASK},
    {"C2H_STAT_S_AXIS_C2H_ACCEPTED", C2H_STAT_S_AXIS_C2H_ACCEPTED_MASK},
};

// MD: Debug: Log the initialization of C2H status S_AXIS C2H accepted field info
qdma_log_debug("Initialized c2h_stat_s_axis_c2h_accepted_field_info\n");

// MD: Structure to hold information about C2H status S_AXIS WRB accepted fields
static struct regfield_info c2h_stat_s_axis_wrb_accepted_field_info[] = {
    {"C2H_STAT_S_AXIS_WRB_ACCEPTED_RSVD_1", C2H_STAT_S_AXIS_WRB_ACCEPTED_RSVD_1_MASK},
    {"C2H_STAT_S_AXIS_WRB_ACCEPTED", C2H_STAT_S_AXIS_WRB_ACCEPTED_MASK},
};

// MD: Debug: Log the initialization of C2H status S_AXIS WRB accepted field info
qdma_log_debug("Initialized c2h_stat_s_axis_wrb_accepted_field_info\n");

// MD: Structure to hold information about C2H status descriptor response packet accepted fields
static struct regfield_info c2h_stat_desc_rsp_pkt_accepted_field_info[] = {
    {"C2H_STAT_DESC_RSP_PKT_ACCEPTED_RSVD_1", C2H_STAT_DESC_RSP_PKT_ACCEPTED_RSVD_1_MASK},
    {"C2H_STAT_DESC_RSP_PKT_ACCEPTED_D", C2H_STAT_DESC_RSP_PKT_ACCEPTED_D_MASK},
};

// MD: Debug: Log the initialization of C2H status descriptor response packet accepted field info
qdma_log_debug("Initialized c2h_stat_desc_rsp_pkt_accepted_field_info\n");

// MD: Structure to hold information about C2H status AXIS package complete fields
static struct regfield_info c2h_stat_axis_pkg_cmp_field_info[] = {
    {"C2H_STAT_AXIS_PKG_CMP_RSVD_1", C2H_STAT_AXIS_PKG_CMP_RSVD_1_MASK},
    {"C2H_STAT_AXIS_PKG_CMP", C2H_STAT_AXIS_PKG_CMP_MASK},
};

// MD: Debug: Log the initialization of C2H status AXIS package complete field info
qdma_log_debug("Initialized c2h_stat_axis_pkg_cmp_field_info\n");

// MD: Structure to hold information about C2H status descriptor response accepted fields
static struct regfield_info c2h_stat_desc_rsp_accepted_field_info[] = {
    {"C2H_STAT_DESC_RSP_ACCEPTED_RSVD_1", C2H_STAT_DESC_RSP_ACCEPTED_RSVD_1_MASK},
    {"C2H_STAT_DESC_RSP_ACCEPTED_D", C2H_STAT_DESC_RSP_ACCEPTED_D_MASK},
};

// MD: Debug: Log the initialization of C2H status descriptor response accepted field info
qdma_log_debug("Initialized c2h_stat_desc_rsp_accepted_field_info\n");

// MD: Structure to hold information about C2H status descriptor response complete fields
static struct regfield_info c2h_stat_desc_rsp_cmp_field_info[] = {
    {"C2H_STAT_DESC_RSP_CMP_RSVD_1", C2H_STAT_DESC_RSP_CMP_RSVD_1_MASK},
    {"C2H_STAT_DESC_RSP_CMP_D", C2H_STAT_DESC_RSP_CMP_D_MASK},
};

// MD: Debug: Log the initialization of C2H status descriptor response complete field info
qdma_log_debug("Initialized c2h_stat_desc_rsp_cmp_field_info\n");

// MD: Structure to hold information about C2H status WRQ out fields
static struct regfield_info c2h_stat_wrq_out_field_info[] = {
    {"C2H_STAT_WRQ_OUT_RSVD_1", C2H_STAT_WRQ_OUT_RSVD_1_MASK},
    {"C2H_STAT_WRQ_OUT", C2H_STAT_WRQ_OUT_MASK},
};

// MD: Debug: Log the initialization of C2H status WRQ out field info
qdma_log_debug("Initialized c2h_stat_wrq_out_field_info\n");

// MD: Structure to hold information about C2H status WPL REN accepted fields
static struct regfield_info c2h_stat_wpl_ren_accepted_field_info[] = {
    {"C2H_STAT_WPL_REN_ACCEPTED_RSVD_1", C2H_STAT_WPL_REN_ACCEPTED_RSVD_1_MASK},
    {"C2H_STAT_WPL_REN_ACCEPTED", C2H_STAT_WPL_REN_ACCEPTED_MASK},
};

// MD: Debug: Log the initialization of C2H status WPL REN accepted field info
qdma_log_debug("Initialized c2h_stat_wpl_ren_accepted_field_info\n");

// MD: Structure to hold information about C2H status total WRQ length fields
static struct regfield_info c2h_stat_total_wrq_len_field_info[] = {
    {"C2H_STAT_TOTAL_WRQ_LEN_RSVD_1", C2H_STAT_TOTAL_WRQ_LEN_RSVD_1_MASK},
    {"C2H_STAT_TOTAL_WRQ_LEN", C2H_STAT_TOTAL_WRQ_LEN_MASK},
};

// MD: Debug: Log the initialization of C2H status total WRQ length field info
qdma_log_debug("Initialized c2h_stat_total_wrq_len_field_info\n");

// MD: Structure to hold information about C2H status total WPL length fields
static struct regfield_info c2h_stat_total_wpl_len_field_info[] = {
    {"C2H_STAT_TOTAL_WPL_LEN_RSVD_1", C2H_STAT_TOTAL_WPL_LEN_RSVD_1_MASK},
    {"C2H_STAT_TOTAL_WPL_LEN", C2H_STAT_TOTAL_WPL_LEN_MASK},
};

// MD: Debug: Log the initialization of C2H status total WPL length field info
qdma_log_debug("Initialized c2h_stat_total_wpl_len_field_info\n");

// MD: Structure to hold information about C2H buffer size fields
static struct regfield_info c2h_buf_sz_field_info[] = {
    {"C2H_BUF_SZ_IZE", C2H_BUF_SZ_IZE_MASK},
};

// MD: Debug: Log the initialization of C2H buffer size field info
qdma_log_debug("Initialized c2h_buf_sz_field_info\n");

// MD: Structure to hold information about C2H error status fields
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

// MD: Debug: Log the initialization of C2H error status field info
qdma_log_debug("Initialized c2h_err_stat_field_info\n");

// MD: Structure to hold information about C2H error mask fields
static struct regfield_info c2h_err_mask_field_info[] = {
    {"C2H_ERR_EN", C2H_ERR_EN_MASK},
};

// MD: Debug: Log the initialization of C2H error mask field info
qdma_log_debug("Initialized c2h_err_mask_field_info\n");

// MD: Structure to hold information about C2H fatal error status fields
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

// MD: Debug: Log the initialization of C2H fatal error status field info
qdma_log_debug("Initialized c2h_fatal_err_stat_field_info\n");

// MD: Structure to hold information about C2H fatal error mask fields
static struct regfield_info c2h_fatal_err_mask_field_info[] = {
    {"C2H_FATAL_ERR_C2HEN", C2H_FATAL_ERR_C2HEN_MASK},
};

// MD: Debug: Log the initialization of C2H fatal error mask field info
qdma_log_debug("Initialized c2h_fatal_err_mask_field_info\n");

// MD: Structure to hold information about C2H fatal error enable fields
static struct regfield_info c2h_fatal_err_enable_field_info[] = {
    {"C2H_FATAL_ERR_ENABLE_RSVD_1", C2H_FATAL_ERR_ENABLE_RSVD_1_MASK},
    {"C2H_FATAL_ERR_ENABLE_WPL_PAR_INV", C2H_FATAL_ERR_ENABLE_WPL_PAR_INV_MASK},
    {"C2H_FATAL_ERR_ENABLE_WRQ_DIS", C2H_FATAL_ERR_ENABLE_WRQ_DIS_MASK},
};

// MD: Debug: Log the initialization of C2H fatal error enable field info
qdma_log_debug("Initialized c2h_fatal_err_enable_field_info\n");

// MD: Structure to hold information about global error interrupt fields
static struct regfield_info glbl_err_int_field_info[] = {
    {"GLBL_ERR_INT_RSVD_1", GLBL_ERR_INT_RSVD_1_MASK},
    {"GLBL_ERR_INT_HOST_ID", GLBL_ERR_INT_HOST_ID_MASK},
    {"GLBL_ERR_INT_DIS_INTR_ON_VF", GLBL_ERR_INT_DIS_INTR_ON_VF_MASK},
    {"GLBL_ERR_INT_ARM", GLBL_ERR_INT_ARM_MASK},
    {"GLBL_ERR_INT_EN_COAL", GLBL_ERR_INT_EN_COAL_MASK},
    {"GLBL_ERR_INT_VEC", GLBL_ERR_INT_VEC_MASK},
    {"GLBL_ERR_INT_FUNC", GLBL_ERR_INT_FUNC_MASK},
};

// MD: Debug: Log the initialization of global error interrupt field info
qdma_log_debug("Initialized glbl_err_int_field_info\n");

// MD: Structure to hold information about C2H prefetch configuration fields
static struct regfield_info c2h_pfch_cfg_field_info[] = {
    {"C2H_PFCH_CFG_EVTFL_TH", C2H_PFCH_CFG_EVTFL_TH_MASK},
    {"C2H_PFCH_CFG_FL_TH", C2H_PFCH_CFG_FL_TH_MASK},
};

// MD: Debug: Log the initialization of C2H prefetch configuration field info
qdma_log_debug("Initialized c2h_pfch_cfg_field_info\n");

// MD: Structure to hold information about C2H prefetch configuration 1 fields
static struct regfield_info c2h_pfch_cfg_1_field_info[] = {
    {"C2H_PFCH_CFG_1_EVT_QCNT_TH", C2H_PFCH_CFG_1_EVT_QCNT_TH_MASK},
    {"C2H_PFCH_CFG_1_QCNT", C2H_PFCH_CFG_1_QCNT_MASK},
};

// MD: Debug: Log the initialization of C2H prefetch configuration 1 field info
qdma_log_debug("Initialized c2h_pfch_cfg_1_field_info\n");

// MD: Structure to hold information about C2H prefetch configuration 2 fields
static struct regfield_info c2h_pfch_cfg_2_field_info[] = {
    {"C2H_PFCH_CFG_2_FENCE", C2H_PFCH_CFG_2_FENCE_MASK},
    {"C2H_PFCH_CFG_2_RSVD", C2H_PFCH_CFG_2_RSVD_MASK},
    {"C2H_PFCH_CFG_2_VAR_DESC_NO_DROP", C2H_PFCH_CFG_2_VAR_DESC_NO_DROP_MASK},
    {"C2H_PFCH_CFG_2_LL_SZ_TH", C2H_PFCH_CFG_2_LL_SZ_TH_MASK},
    {"C2H_PFCH_CFG_2_VAR_DESC_NUM", C2H_PFCH_CFG_2_VAR_DESC_NUM_MASK},
    {"C2H_PFCH_CFG_2_NUM", C2H_PFCH_CFG_2_NUM_MASK},
};

// MD: Debug: Log the initialization of C2H prefetch configuration 2 field info
qdma_log_debug("Initialized c2h_pfch_cfg_2_field_info\n");

// MD: Structure to hold information about C2H interrupt timer tick fields
static struct regfield_info c2h_int_timer_tick_field_info[] = {
    {"C2H_INT_TIMER_TICK", C2H_INT_TIMER_TICK_MASK},
};

// MD: Debug: Log the initialization of C2H interrupt timer tick field info
qdma_log_debug("Initialized c2h_int_timer_tick_field_info\n");

// MD: Structure to hold information about C2H status descriptor response drop accepted fields
static struct regfield_info c2h_stat_desc_rsp_drop_accepted_field_info[] = {
    {"C2H_STAT_DESC_RSP_DROP_ACCEPTED_RSVD_1", C2H_STAT_DESC_RSP_DROP_ACCEPTED_RSVD_1_MASK},
    {"C2H_STAT_DESC_RSP_DROP_ACCEPTED_D", C2H_STAT_DESC_RSP_DROP_ACCEPTED_D_MASK},
};

// MD: Debug: Log the initialization of C2H status descriptor response drop accepted field info
qdma_log_debug("Initialized c2h_stat_desc_rsp_drop_accepted_field_info\n");

// MD: Structure to hold information about C2H status descriptor response error accepted fields
static struct regfield_info c2h_stat_desc_rsp_err_accepted_field_info[] = {
    {"C2H_STAT_DESC_RSP_ERR_ACCEPTED_RSVD_1", C2H_STAT_DESC_RSP_ERR_ACCEPTED_RSVD_1_MASK},
    {"C2H_STAT_DESC_RSP_ERR_ACCEPTED_D", C2H_STAT_DESC_RSP_ERR_ACCEPTED_D_MASK},
};

// MD: Debug: Log the initialization of C2H status descriptor response error accepted field info
qdma_log_debug("Initialized c2h_stat_desc_rsp_err_accepted_field_info\n");

// MD: Structure to hold information about C2H status descriptor request fields
static struct regfield_info c2h_stat_desc_req_field_info[] = {
    {"C2H_STAT_DESC_REQ_RSVD_1", C2H_STAT_DESC_REQ_RSVD_1_MASK},
    {"C2H_STAT_DESC_REQ", C2H_STAT_DESC_REQ_MASK},
};

// MD: Debug: Log the initialization of C2H status descriptor request field info
qdma_log_debug("Initialized c2h_stat_desc_req_field_info\n");

// MD: Structure to hold information about C2H status debug DMA engine 0 fields
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
};

// MD: Debug: Log the initialization of C2H status debug DMA engine 0 field info
qdma_log_debug("Initialized c2h_stat_dbg_dma_eng_0_field_info\n");

// MD: Structure to hold information about C2H status debug DMA engine 1 fields
static struct regfield_info c2h_stat_dbg_dma_eng_1_field_info[] = {
    {"C2H_STAT_DMA_ENG_1_RSVD_1", C2H_STAT_DMA_ENG_1_RSVD_1_MASK},
    {"C2H_STAT_DMA_ENG_1_QID_FIFO_OUT_CNT", C2H_STAT_DMA_ENG_1_QID_FIFO_OUT_CNT_MASK},
    {"C2H_STAT_DMA_ENG_1_PLD_FIFO_OUT_CNT", C2H_STAT_DMA_ENG_1_PLD_FIFO_OUT_CNT_MASK},
    {"C2H_STAT_DMA_ENG_1_PLD_ST_FIFO_CNT", C2H_STAT_DMA_ENG_1_PLD_ST_FIFO_CNT_MASK},
};

// MD: Debug: Log the initialization of C2H status debug DMA engine 1 field info
qdma_log_debug("Initialized c2h_stat_dbg_dma_eng_1_field_info\n");

// MD: Structure to hold information about C2H status debug DMA engine 2 fields
static struct regfield_info c2h_stat_dbg_dma_eng_2_field_info[] = {
    {"C2H_STAT_DMA_ENG_2_RSVD_1", C2H_STAT_DMA_ENG_2_RSVD_1_MASK},
    {"C2H_STAT_DMA_ENG_2_QID_FIFO_OUT_CNT", C2H_STAT_DMA_ENG_2_QID_FIFO_OUT_CNT_MASK},
    {"C2H_STAT_DMA_ENG_2_PLD_FIFO_OUT_CNT", C2H_STAT_DMA_ENG_2_PLD_FIFO_OUT_CNT_MASK},
    {"C2H_STAT_DMA_ENG_2_PLD_ST_FIFO_CNT", C2H_STAT_DMA_ENG_2_PLD_ST_FIFO_CNT_MASK},
};

// MD: Debug: Log the initialization of C2H status debug DMA engine 2 field info
qdma_log_debug("Initialized c2h_stat_dbg_dma_eng_2_field_info\n");

// MD: Structure to hold information about C2H debug DMA engine 3 fields
static struct regfield_info c2h_stat_dbg_dma_eng_3_field_info[] = {
    {"C2H_STAT_DMA_ENG_3_RSVD_1", C2H_STAT_DMA_ENG_3_RSVD_1_MASK},
    {"C2H_STAT_DMA_ENG_3_WRQ_FIFO_OUT_CNT", C2H_STAT_DMA_ENG_3_WRQ_FIFO_OUT_CNT_MASK},
    {"C2H_STAT_DMA_ENG_3_QID_FIFO_OUT_VLD", C2H_STAT_DMA_ENG_3_QID_FIFO_OUT_VLD_MASK},
    {"C2H_STAT_DMA_ENG_3_PLD_FIFO_OUT_VLD", C2H_STAT_DMA_ENG_3_PLD_FIFO_OUT_VLD_MASK},
    {"C2H_STAT_DMA_ENG_3_PLD_ST_FIFO_OUT_VLD", C2H_STAT_DMA_ENG_3_PLD_ST_FIFO_OUT_VLD_MASK},
    {"C2H_STAT_DMA_ENG_3_PLD_ST_FIFO_OUT_DATA_EOP", C2H_STAT_DMA_ENG_3_PLD_ST_FIFO_OUT_DATA_EOP_MASK},
    {"C2H_STAT_DMA_ENG_3_PLD_ST_FIFO_OUT_DATA_AVL_IDX_ENABLE", C2H_STAT_DMA_ENG_3_PLD_ST_FIFO_OUT_DATA_AVL_IDX_ENABLE_MASK},
    {"C2H_STAT_DMA_ENG_3_PLD_ST_FIFO_OUT_DATA_DROP", C2H_STAT_DMA_ENG_3_PLD_ST_FIFO_OUT_DATA_DROP_MASK},
    {"C2H_STAT_DMA_ENG_3_PLD_ST_FIFO_OUT_DATA_ERR", C2H_STAT_DMA_ENG_3_PLD_ST_FIFO_OUT_DATA_ERR_MASK},
    {"C2H_STAT_DMA_ENG_3_DESC_CNT_FIFO_IN_RDY", C2H_STAT_DMA_ENG_3_DESC_CNT_FIFO_IN_RDY_MASK},
    {"C2H_STAT_DMA_ENG_3_DESC_RSP_FIFO_IN_RDY", C2H_STAT_DMA_ENG_3_DESC_RSP_FIFO_IN_RDY_MASK},
    {"C2H_STAT_DMA_ENG_3_PLD_PKT_ID_LARGER_0", C2H_STAT_DMA_ENG_3_PLD_PKT_ID_LARGER_0_MASK},
    {"C2H_STAT_DMA_ENG_3_WRQ_VLD", C2H_STAT_DMA_ENG_3_WRQ_VLD_MASK},
    {"C2H_STAT_DMA_ENG_3_WRQ_RDY", C2H_STAT_DMA_ENG_3_WRQ_RDY_MASK},
    {"C2H_STAT_DMA_ENG_3_WRQ_FIFO_OUT_RDY", C2H_STAT_DMA_ENG_3_WRQ_FIFO_OUT_RDY_MASK},
    {"C2H_STAT_DMA_ENG_3_WRQ_PACKET_OUT_DATA_DROP", C2H_STAT_DMA_ENG_3_WRQ_PACKET_OUT_DATA_DROP_MASK},
    {"C2H_STAT_DMA_ENG_3_WRQ_PACKET_OUT_DATA_ERR", C2H_STAT_DMA_ENG_3_WRQ_PACKET_OUT_DATA_ERR_MASK},
    {"C2H_STAT_DMA_ENG_3_WRQ_PACKET_OUT_DATA_MARKER", C2H_STAT_DMA_ENG_3_WRQ_PACKET_OUT_DATA_MARKER_MASK},
    {"C2H_STAT_DMA_ENG_3_WRQ_PACKET_PRE_EOR", C2H_STAT_DMA_ENG_3_WRQ_PACKET_PRE_EOR_MASK},
    {"C2H_STAT_DMA_ENG_3_WCP_FIFO_IN_RDY", C2H_STAT_DMA_ENG_3_WCP_FIFO_IN_RDY_MASK},
    {"C2H_STAT_DMA_ENG_3_PLD_ST_FIFO_IN_RDY", C2H_STAT_DMA_ENG_3_PLD_ST_FIFO_IN_RDY_MASK},
};

// MD: Debug: Log the initialization of C2H debug DMA engine 3 field info
qdma_log_debug("Initialized c2h_stat_dbg_dma_eng_3_field_info\n");

// MD: Structure to hold information about C2H debug prefetch error context fields
static struct regfield_info c2h_dbg_pfch_err_ctxt_field_info[] = {
    {"C2H_PFCH_ERR_CTXT_RSVD_1", C2H_PFCH_ERR_CTXT_RSVD_1_MASK},
    {"C2H_PFCH_ERR_CTXT_ERR_STAT", C2H_PFCH_ERR_CTXT_ERR_STAT_MASK},
    {"C2H_PFCH_ERR_CTXT_CMD_WR", C2H_PFCH_ERR_CTXT_CMD_WR_MASK},
    {"C2H_PFCH_ERR_CTXT_QID", C2H_PFCH_ERR_CTXT_QID_MASK},
    {"C2H_PFCH_ERR_CTXT_DONE", C2H_PFCH_ERR_CTXT_DONE_MASK},
};

// MD: Debug: Log the initialization of C2H debug prefetch error context field info
qdma_log_debug("Initialized c2h_dbg_pfch_err_ctxt_field_info\n");

// MD: Structure to hold information about C2H first error QID fields
static struct regfield_info c2h_first_err_qid_field_info[] = {
    {"C2H_FIRST_ERR_QID_RSVD_1", C2H_FIRST_ERR_QID_RSVD_1_MASK},
    {"C2H_FIRST_ERR_QID_ERR_TYPE", C2H_FIRST_ERR_QID_ERR_TYPE_MASK},
    {"C2H_FIRST_ERR_QID_RSVD", C2H_FIRST_ERR_QID_RSVD_MASK},
    {"C2H_FIRST_ERR_QID_QID", C2H_FIRST_ERR_QID_QID_MASK},
};

// MD: Debug: Log the initialization of C2H first error QID field info
qdma_log_debug("Initialized c2h_first_err_qid_field_info\n");

// MD: Structure to hold information about status number of WRB in fields
static struct regfield_info stat_num_wrb_in_field_info[] = {
    {"STAT_NUM_WRB_IN_RSVD_1", STAT_NUM_WRB_IN_RSVD_1_MASK},
    {"STAT_NUM_WRB_IN_WRB_CNT", STAT_NUM_WRB_IN_WRB_CNT_MASK},
};

// MD: Debug: Log the initialization of status number of WRB in field info
qdma_log_debug("Initialized stat_num_wrb_in_field_info\n");

// MD: Structure to hold information about status number of WRB out fields
static struct regfield_info stat_num_wrb_out_field_info[] = {
    {"STAT_NUM_WRB_OUT_RSVD_1", STAT_NUM_WRB_OUT_RSVD_1_MASK},
    {"STAT_NUM_WRB_OUT_WRB_CNT", STAT_NUM_WRB_OUT_WRB_CNT_MASK},
};

// MD: Debug: Log the initialization of status number of WRB out field info
qdma_log_debug("Initialized stat_num_wrb_out_field_info\n");

// MD: Structure to hold information about status number of WRB drop fields
static struct regfield_info stat_num_wrb_drp_field_info[] = {
    {"STAT_NUM_WRB_DRP_RSVD_1", STAT_NUM_WRB_DRP_RSVD_1_MASK},
    {"STAT_NUM_WRB_DRP_WRB_CNT", STAT_NUM_WRB_DRP_WRB_CNT_MASK},
};

// MD: Debug: Log the initialization of status number of WRB drop field info
qdma_log_debug("Initialized stat_num_wrb_drp_field_info\n");

// MD: Structure to hold information about status number of stat descriptor out fields
static struct regfield_info stat_num_stat_desc_out_field_info[] = {
    {"STAT_NUM_STAT_DESC_OUT_RSVD_1", STAT_NUM_STAT_DESC_OUT_RSVD_1_MASK},
    {"STAT_NUM_STAT_DESC_OUT_CNT", STAT_NUM_STAT_DESC_OUT_CNT_MASK},
};

// MD: Debug: Log the initialization of status number of stat descriptor out field info
qdma_log_debug("Initialized stat_num_stat_desc_out_field_info\n");

// MD: Structure to hold information about status number of descriptor credit sent fields
static struct regfield_info stat_num_dsc_crdt_sent_field_info[] = {
    {"STAT_NUM_DSC_CRDT_SENT_RSVD_1", STAT_NUM_DSC_CRDT_SENT_RSVD_1_MASK},
    {"STAT_NUM_DSC_CRDT_SENT_CNT", STAT_NUM_DSC_CRDT_SENT_CNT_MASK},
};

// MD: Debug: Log the initialization of status number of descriptor credit sent field info
qdma_log_debug("Initialized stat_num_dsc_crdt_sent_field_info\n");

// MD: Structure to hold information about status number of fetch descriptor received fields
static struct regfield_info stat_num_fch_dsc_rcvd_field_info[] = {
    {"STAT_NUM_FCH_DSC_RCVD_RSVD_1", STAT_NUM_FCH_DSC_RCVD_RSVD_1_MASK},
    {"STAT_NUM_FCH_DSC_RCVD_DSC_CNT", STAT_NUM_FCH_DSC_RCVD_DSC_CNT_MASK},
};

// MD: Debug: Log the initialization of status number of fetch descriptor received field info
qdma_log_debug("Initialized stat_num_fch_dsc_rcvd_field_info\n");

// MD: Structure to hold information about status number of bypass descriptor received fields
static struct regfield_info stat_num_byp_dsc_rcvd_field_info[] = {
    {"STAT_NUM_BYP_DSC_RCVD_RSVD_1", STAT_NUM_BYP_DSC_RCVD_RSVD_1_MASK},
    {"STAT_NUM_BYP_DSC_RCVD_DSC_CNT", STAT_NUM_BYP_DSC_RCVD_DSC_CNT_MASK},
};

// MD: Debug: Log the initialization of status number of bypass descriptor received field info
qdma_log_debug("Initialized stat_num_byp_dsc_rcvd_field_info\n");

// MD: Structure to hold information about C2H WRB coalescing configuration fields
static struct regfield_info c2h_wrb_coal_cfg_field_info[] = {
    {"C2H_WRB_COAL_CFG_MAX_BUF_SZ", C2H_WRB_COAL_CFG_MAX_BUF_SZ_MASK},
    {"C2H_WRB_COAL_CFG_TICK_VAL", C2H_WRB_COAL_CFG_TICK_VAL_MASK},
    {"C2H_WRB_COAL_CFG_TICK_CNT", C2H_WRB_COAL_CFG_TICK_CNT_MASK},
    {"C2H_WRB_COAL_CFG_SET_GLB_FLUSH", C2H_WRB_COAL_CFG_SET_GLB_FLUSH_MASK},
    {"C2H_WRB_COAL_CFG_DONE_GLB_FLUSH", C2H_WRB_COAL_CFG_DONE_GLB_FLUSH_MASK},
};

// MD: Debug: Log the initialization of C2H WRB coalescing configuration field info
qdma_log_debug("Initialized c2h_wrb_coal_cfg_field_info\n");

// MD: Structure to hold information about C2H interrupt H2C request fields
static struct regfield_info c2h_intr_h2c_req_field_info[] = {
    {"C2H_INTR_H2C_REQ_RSVD_1", C2H_INTR_H2C_REQ_RSVD_1_MASK},
    {"C2H_INTR_H2C_REQ_CNT", C2H_INTR_H2C_REQ_CNT_MASK},
};

// MD: Debug: Log the initialization of C2H interrupt H2C request field info
qdma_log_debug("Initialized c2h_intr_h2c_req_field_info\n");

// MD: Structure to hold information about C2H interrupt C2H MM request fields
static struct regfield_info c2h_intr_c2h_mm_req_field_info[] = {
    {"C2H_INTR_C2H_MM_REQ_RSVD_1", C2H_INTR_C2H_MM_REQ_RSVD_1_MASK},
    {"C2H_INTR_C2H_MM_REQ_CNT", C2H_INTR_C2H_MM_REQ_CNT_MASK},
};

// MD: Debug: Log the initialization of C2H interrupt C2H MM request field info
qdma_log_debug("Initialized c2h_intr_c2h_mm_req_field_info\n");

// MD: Structure to hold information about C2H interrupt error interrupt request fields
static struct regfield_info c2h_intr_err_int_req_field_info[] = {
    {"C2H_INTR_ERR_INT_REQ_RSVD_1", C2H_INTR_ERR_INT_REQ_RSVD_1_MASK},
    {"C2H_INTR_ERR_INT_REQ_CNT", C2H_INTR_ERR_INT_REQ_CNT_MASK},
};

// MD: Debug: Log the initialization of C2H interrupt error interrupt request field info
qdma_log_debug("Initialized c2h_intr_err_int_req_field_info\n");

// MD: Structure to hold information about C2H interrupt C2H ST request fields
static struct regfield_info c2h_intr_c2h_st_req_field_info[] = {
    {"C2H_INTR_C2H_ST_REQ_RSVD_1", C2H_INTR_C2H_ST_REQ_RSVD_1_MASK},
    {"C2H_INTR_C2H_ST_REQ_CNT", C2H_INTR_C2H_ST_REQ_CNT_MASK},
};

// MD: Debug: Log the initialization of C2H interrupt C2H ST request field info
qdma_log_debug("Initialized c2h_intr_c2h_st_req_field_info\n");

// MD: Structure to hold information about C2H interrupt H2C error C2H MM MSIX acknowledgment fields
static struct regfield_info c2h_intr_h2c_err_c2h_mm_msix_ack_field_info[] = {
    {"C2H_INTR_H2C_ERR_C2H_MM_MSIX_ACK_RSVD_1", C2H_INTR_H2C_ERR_C2H_MM_MSIX_ACK_RSVD_1_MASK},
    {"C2H_INTR_H2C_ERR_C2H_MM_MSIX_ACK_CNT", C2H_INTR_H2C_ERR_C2H_MM_MSIX_ACK_CNT_MASK},
};

// MD: Debug: Log the initialization of C2H interrupt H2C error C2H MM MSIX acknowledgment field info
qdma_log_debug("Initialized c2h_intr_h2c_err_c2h_mm_msix_ack_field_info\n");

// MD: Structure to hold information about C2H interrupt H2C error C2H MM MSIX failure fields
static struct regfield_info c2h_intr_h2c_err_c2h_mm_msix_fail_field_info[] = {
    {"C2H_INTR_H2C_ERR_C2H_MM_MSIX_FAIL_RSVD_1", C2H_INTR_H2C_ERR_C2H_MM_MSIX_FAIL_RSVD_1_MASK},
    {"C2H_INTR_H2C_ERR_C2H_MM_MSIX_FAIL_CNT", C2H_INTR_H2C_ERR_C2H_MM_MSIX_FAIL_CNT_MASK},
};

// MD: Debug: Log the initialization of C2H interrupt H2C error C2H MM MSIX failure field info
qdma_log_debug("Initialized c2h_intr_h2c_err_c2h_mm_msix_fail_field_info\n");

// MD: Structure to hold information about C2H interrupt H2C error C2H MM MSIX no MSIX fields
static struct regfield_info c2h_intr_h2c_err_c2h_mm_msix_no_msix_field_info[] = {
    {"C2H_INTR_H2C_ERR_C2H_MM_MSIX_NO_MSIX_RSVD_1", C2H_INTR_H2C_ERR_C2H_MM_MSIX_NO_MSIX_RSVD_1_MASK},
    {"C2H_INTR_H2C_ERR_C2H_MM_MSIX_NO_MSIX_CNT", C2H_INTR_H2C_ERR_C2H_MM_MSIX_NO_MSIX_CNT_MASK},
};

// MD: Debug: Log the initialization of C2H interrupt H2C error C2H MM MSIX no MSIX field info
qdma_log_debug("Initialized c2h_intr_h2c_err_c2h_mm_msix_no_msix_field_info\n");

// MD: Structure to hold information about C2H interrupt H2C error C2H MM context invalid fields
static struct regfield_info c2h_intr_h2c_err_c2h_mm_ctxt_inval_field_info[] = {
    {"C2H_INTR_H2C_ERR_C2H_MM_CTXT_INVAL_RSVD_1", C2H_INTR_H2C_ERR_C2H_MM_CTXT_INVAL_RSVD_1_MASK},
    {"C2H_INTR_H2C_ERR_C2H_MM_CTXT_INVAL_CNT", C2H_INTR_H2C_ERR_C2H_MM_CTXT_INVAL_CNT_MASK},
};

// MD: Debug: Log the initialization of C2H interrupt H2C error C2H MM context invalid field info
qdma_log_debug("Initialized c2h_intr_h2c_err_c2h_mm_ctxt_inval_field_info\n");

// MD: Structure to hold information about C2H interrupt C2H ST MSIX acknowledgment fields
static struct regfield_info c2h_intr_c2h_st_msix_ack_field_info[] = {
    {"C2H_INTR_C2H_ST_MSIX_ACK_RSVD_1", C2H_INTR_C2H_ST_MSIX_ACK_RSVD_1_MASK},
    {"C2H_INTR_C2H_ST_MSIX_ACK_CNT", C2H_INTR_C2H_ST_MSIX_ACK_CNT_MASK},
};

// MD: Debug: Log the initialization of C2H interrupt C2H ST MSIX acknowledgment field info
qdma_log_debug("Initialized c2h_intr_c2h_st_msix_ack_field_info\n");

// MD: Structure to hold information about C2H interrupt C2H ST MSIX failure fields
static struct regfield_info c2h_intr_c2h_st_msix_fail_field_info[] = {
    {"C2H_INTR_C2H_ST_MSIX_FAIL_RSVD_1", C2H_INTR_C2H_ST_MSIX_FAIL_RSVD_1_MASK},
    {"C2H_INTR_C2H_ST_MSIX_FAIL_CNT", C2H_INTR_C2H_ST_MSIX_FAIL_CNT_MASK},
};

// MD: Debug: Log the initialization of C2H interrupt C2H ST MSIX failure field info
qdma_log_debug("Initialized c2h_intr_c2h_st_msix_fail_field_info\n");

// MD: Structure to hold information about C2H interrupt C2H ST no MSIX fields
static struct regfield_info c2h_intr_c2h_st_no_msix_field_info[] = {
    {"C2H_INTR_C2H_ST_NO_MSIX_RSVD_1", C2H_INTR_C2H_ST_NO_MSIX_RSVD_1_MASK},
    {"C2H_INTR_C2H_ST_NO_MSIX_CNT", C2H_INTR_C2H_ST_NO_MSIX_CNT_MASK},
};

// MD: Debug: Log the initialization of C2H interrupt C2H ST no MSIX field info
qdma_log_debug("Initialized c2h_intr_c2h_st_no_msix_field_info\n");

// MD: Structure to hold information about C2H interrupt C2H ST context invalid fields
static struct regfield_info c2h_intr_c2h_st_ctxt_inval_field_info[] = {
    {"C2H_INTR_C2H_ST_CTXT_INVAL_RSVD_1", C2H_INTR_C2H_ST_CTXT_INVAL_RSVD_1_MASK},
    {"C2H_INTR_C2H_ST_CTXT_INVAL_CNT", C2H_INTR_C2H_ST_CTXT_INVAL_CNT_MASK},
};

// MD: Debug: Log the initialization of C2H interrupt C2H ST context invalid field info
qdma_log_debug("Initialized c2h_intr_c2h_st_ctxt_inval_field_info\n");

// MD: Structure to hold information about C2H status write completion fields
static struct regfield_info c2h_stat_wr_cmp_field_info[] = {
    {"C2H_STAT_WR_CMP_RSVD_1", C2H_STAT_WR_CMP_RSVD_1_MASK},
    {"C2H_STAT_WR_CMP_CNT", C2H_STAT_WR_CMP_CNT_MASK},
};

// MD: Debug: Log the initialization of C2H status write completion field info
qdma_log_debug("Initialized c2h_stat_wr_cmp_field_info\n");

// MD: Structure to hold information about C2H status debug DMA engine 4 fields
static struct regfield_info c2h_stat_dbg_dma_eng_4_field_info[] = {
    {"C2H_STAT_DMA_ENG_4_RSVD_1", C2H_STAT_DMA_ENG_4_RSVD_1_MASK},
    {"C2H_STAT_DMA_ENG_4_WRQ_FIFO_OUT_CNT", C2H_STAT_DMA_ENG_4_WRQ_FIFO_OUT_CNT_MASK},
    {"C2H_STAT_DMA_ENG_4_QID_FIFO_OUT_VLD", C2H_STAT_DMA_ENG_4_QID_FIFO_OUT_VLD_MASK},
    {"C2H_STAT_DMA_ENG_4_PLD_FIFO_OUT_VLD", C2H_STAT_DMA_ENG_4_PLD_FIFO_OUT_VLD_MASK},
    {"C2H_STAT_DMA_ENG_4_PLD_ST_FIFO_OUT_VLD", C2H_STAT_DMA_ENG_4_PLD_ST_FIFO_OUT_VLD_MASK},
    {"C2H_STAT_DMA_ENG_4_PLD_ST_FIFO_OUT_DATA_EOP", C2H_STAT_DMA_ENG_4_PLD_ST_FIFO_OUT_DATA_EOP_MASK},
    {"C2H_STAT_DMA_ENG_4_PLD_ST_FIFO_OUT_DATA_AVL_IDX_ENABLE", C2H_STAT_DMA_ENG_4_PLD_ST_FIFO_OUT_DATA_AVL_IDX_ENABLE_MASK},
    {"C2H_STAT_DMA_ENG_4_PLD_ST_FIFO_OUT_DATA_DROP", C2H_STAT_DMA_ENG_4_PLD_ST_FIFO_OUT_DATA_DROP_MASK},
    {"C2H_STAT_DMA_ENG_4_PLD_ST_FIFO_OUT_DATA_ERR", C2H_STAT_DMA_ENG_4_PLD_ST_FIFO_OUT_DATA_ERR_MASK},
    {"C2H_STAT_DMA_ENG_4_DESC_CNT_FIFO_IN_RDY", C2H_STAT_DMA_ENG_4_DESC_CNT_FIFO_IN_RDY_MASK},
    {"C2H_STAT_DMA_ENG_4_DESC_RSP_FIFO_IN_RDY", C2H_STAT_DMA_ENG_4_DESC_RSP_FIFO_IN_RDY_MASK},
    {"C2H_STAT_DMA_ENG_4_PLD_PKT_ID_LARGER_0", C2H_STAT_DMA_ENG_4_PLD_PKT_ID_LARGER_0_MASK},
    {"C2H_STAT_DMA_ENG_4_WRQ_VLD", C2H_STAT_DMA_ENG_4_WRQ_VLD_MASK},
    {"C2H_STAT_DMA_ENG_4_WRQ_RDY", C2H_STAT_DMA_ENG_4_WRQ_RDY_MASK},
    {"C2H_STAT_DMA_ENG_4_WRQ_FIFO_OUT_RDY", C2H_STAT_DMA_ENG_4_WRQ_FIFO_OUT_RDY_MASK},
    {"C2H_STAT_DMA_ENG_4_WRQ_PACKET_OUT_DATA_DROP", C2H_STAT_DMA_ENG_4_WRQ_PACKET_OUT_DATA_DROP_MASK},
    {"C2H_STAT_DMA_ENG_4_WRQ_PACKET_OUT_DATA_ERR", C2H_STAT_DMA_ENG_4_WRQ_PACKET_OUT_DATA_ERR_MASK},
    {"C2H_STAT_DMA_ENG_4_WRQ_PACKET_OUT_DATA_MARKER", C2H_STAT_DMA_ENG_4_WRQ_PACKET_OUT_DATA_MARKER_MASK},
    {"C2H_STAT_DMA_ENG_4_WRQ_PACKET_PRE_EOR", C2H_STAT_DMA_ENG_4_WRQ_PACKET_PRE_EOR_MASK},
    {"C2H_STAT_DMA_ENG_4_WCP_FIFO_IN_RDY", C2H_STAT_DMA_ENG_4_WCP_FIFO_IN_RDY_MASK},
    {"C2H_STAT_DMA_ENG_4_PLD_ST_FIFO_IN_RDY", C2H_STAT_DMA_ENG_4_PLD_ST_FIFO_IN_RDY_MASK},
};

// MD: Debug: Log the initialization of C2H status debug DMA engine 4 field info
qdma_log_debug("Initialized c2h_stat_dbg_dma_eng_4_field_info\n");

// MD: Structure to hold information about C2H status debug DMA engine 5 fields
static struct regfield_info c2h_stat_dbg_dma_eng_5_field_info[] = {
    {"C2H_STAT_DMA_ENG_5_RSVD_1", C2H_STAT_DMA_ENG_5_RSVD_1_MASK},
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
};

// MD: Debug: Log the initialization of C2H status debug DMA engine 5 field info
qdma_log_debug("Initialized c2h_stat_dbg_dma_eng_5_field_info\n");

// MD: Structure to hold information about C2H debug prefetch QID fields
static struct regfield_info c2h_dbg_pfch_qid_field_info[] = {
    {"C2H_PFCH_QID_RSVD_1", C2H_PFCH_QID_RSVD_1_MASK},
    {"C2H_PFCH_QID_ERR_CTXT", C2H_PFCH_QID_ERR_CTXT_MASK},
    {"C2H_PFCH_QID_TARGET", C2H_PFCH_QID_TARGET_MASK},
    {"C2H_PFCH_QID_QID_OR_TAG", C2H_PFCH_QID_QID_OR_TAG_MASK},
};

// MD: Debug: Log the initialization of C2H debug prefetch QID field info
qdma_log_debug("Initialized c2h_dbg_pfch_qid_field_info\n");

// MD: Structure to hold information about C2H debug prefetch fields
static struct regfield_info c2h_dbg_pfch_field_info[] = {
    {"C2H_PFCH_DATA", C2H_PFCH_DATA_MASK},
};

// MD: Debug: Log the initialization of C2H debug prefetch field info
qdma_log_debug("Initialized c2h_dbg_pfch_field_info\n");

// MD: Structure to hold information about C2H interrupt debug fields
static struct regfield_info c2h_int_dbg_field_info[] = {
    {"C2H_INT_RSVD_1", C2H_INT_RSVD_1_MASK},
    {"C2H_INT_INT_COAL_SM", C2H_INT_INT_COAL_SM_MASK},
    {"C2H_INT_INT_SM", C2H_INT_INT_SM_MASK},
};

// MD: Debug: Log the initialization of C2H interrupt debug field info
qdma_log_debug("Initialized c2h_int_dbg_field_info\n");

// MD: Structure to hold information about C2H status immediate accepted fields
static struct regfield_info c2h_stat_imm_accepted_field_info[] = {
    {"C2H_STAT_IMM_ACCEPTED_RSVD_1", C2H_STAT_IMM_ACCEPTED_RSVD_1_MASK},
    {"C2H_STAT_IMM_ACCEPTED_CNT", C2H_STAT_IMM_ACCEPTED_CNT_MASK},
};

// MD: Debug: Log the initialization of C2H status immediate accepted field info
qdma_log_debug("Initialized c2h_stat_imm_accepted_field_info\n");

// MD: Structure to hold information about C2H status marker accepted fields
static struct regfield_info c2h_stat_marker_accepted_field_info[] = {
    {"C2H_STAT_MARKER_ACCEPTED_RSVD_1", C2H_STAT_MARKER_ACCEPTED_RSVD_1_MASK},
    {"C2H_STAT_MARKER_ACCEPTED_CNT", C2H_STAT_MARKER_ACCEPTED_CNT_MASK},
};

// MD: Debug: Log the initialization of C2H status marker accepted field info
qdma_log_debug("Initialized c2h_stat_marker_accepted_field_info\n");

// MD: Structure to hold information about C2H status disable completion accepted fields
static struct regfield_info c2h_stat_disable_cmp_accepted_field_info[] = {
    {"C2H_STAT_DISABLE_CMP_ACCEPTED_RSVD_1", C2H_STAT_DISABLE_CMP_ACCEPTED_RSVD_1_MASK},
    {"C2H_STAT_DISABLE_CMP_ACCEPTED_CNT", C2H_STAT_DISABLE_CMP_ACCEPTED_CNT_MASK},
};

// MD: Debug: Log the initialization of C2H status disable completion accepted field info
qdma_log_debug("Initialized c2h_stat_disable_cmp_accepted_field_info\n");

// MD: Structure to hold information about C2H payload FIFO credit count fields
static struct regfield_info c2h_pld_fifo_crdt_cnt_field_info[] = {
    {"C2H_PLD_FIFO_CRDT_CNT_RSVD_1", C2H_PLD_FIFO_CRDT_CNT_RSVD_1_MASK},
    {"C2H_PLD_FIFO_CRDT_CNT_CNT", C2H_PLD_FIFO_CRDT_CNT_CNT_MASK},
};

// MD: Debug: Log the initialization of C2H payload FIFO credit count field info
qdma_log_debug("Initialized c2h_pld_fifo_crdt_cnt_field_info\n");

// MD: Structure to hold information about C2H interrupt dynamic request fields
static struct regfield_info c2h_intr_dyn_req_field_info[] = {
    {"C2H_INTR_DYN_REQ_RSVD_1", C2H_INTR_DYN_REQ_RSVD_1_MASK},
    {"C2H_INTR_DYN_REQ_CNT", C2H_INTR_DYN_REQ_CNT_MASK},
};

// MD: Debug: Log the initialization of C2H interrupt dynamic request field info
qdma_log_debug("Initialized c2h_intr_dyn_req_field_info\n");

// MD: Structure to hold information about C2H interrupt dynamic miscellaneous fields
static struct regfield_info c2h_intr_dyn_misc_field_info[] = {
    {"C2H_INTR_DYN_MISC_RSVD_1", C2H_INTR_DYN_MISC_RSVD_1_MASK},
    {"C2H_INTR_DYN_MISC_CNT", C2H_INTR_DYN_MISC_CNT_MASK},
};

// MD: Debug: Log the initialization of C2H interrupt dynamic miscellaneous field info
qdma_log_debug("Initialized c2h_intr_dyn_misc_field_info\n");

// MD: Structure to hold information about C2H drop length mismatch fields
static struct regfield_info c2h_drop_len_mismatch_field_info[] = {
    {"C2H_DROP_LEN_MISMATCH_RSVD_1", C2H_DROP_LEN_MISMATCH_RSVD_1_MASK},
    {"C2H_DROP_LEN_MISMATCH_CNT", C2H_DROP_LEN_MISMATCH_CNT_MASK},
};

// MD: Debug: Log the initialization of C2H drop length mismatch field info
qdma_log_debug("Initialized c2h_drop_len_mismatch_field_info\n");

// MD: Structure to hold information about C2H drop descriptor response length fields
static struct regfield_info c2h_drop_desc_rsp_len_field_info[] = {
    {"C2H_DROP_DESC_RSP_LEN_RSVD_1", C2H_DROP_DESC_RSP_LEN_RSVD_1_MASK},
    {"C2H_DROP_DESC_RSP_LEN_CNT", C2H_DROP_DESC_RSP_LEN_CNT_MASK},
};

// MD: Debug: Log the initialization of C2H drop descriptor response length field info
qdma_log_debug("Initialized c2h_drop_desc_rsp_len_field_info\n");

// MD: Structure to hold information about C2H drop QID FIFO length fields
static struct regfield_info c2h_drop_qid_fifo_len_field_info[] = {
    {"C2H_DROP_QID_FIFO_LEN_RSVD_1", C2H_DROP_QID_FIFO_LEN_RSVD_1_MASK},
    {"C2H_DROP_QID_FIFO_LEN_CNT", C2H_DROP_QID_FIFO_LEN_CNT_MASK},
};

// MD: Debug: Log the initialization of C2H drop QID FIFO length field info
qdma_log_debug("Initialized c2h_drop_qid_fifo_len_field_info\n");

// MD: Structure to hold information about C2H drop payload count fields
static struct regfield_info c2h_drop_pld_cnt_field_info[] = {
    {"C2H_DROP_PLD_CNT_RSVD_1", C2H_DROP_PLD_CNT_RSVD_1_MASK},
    {"C2H_DROP_PLD_CNT_CNT", C2H_DROP_PLD_CNT_CNT_MASK},
};

// MD: Debug: Log the initialization of C2H drop payload count field info
qdma_log_debug("Initialized c2h_drop_pld_cnt_field_info\n");

// MD: Structure to hold information about C2H completion format 0 fields
static struct regfield_info c2h_cmpt_format_0_field_info[] = {
    {"C2H_CMPT_FORMAT_0_DESC_ERR_LOC", C2H_CMPT_FORMAT_0_DESC_ERR_LOC_MASK},
    {"C2H_CMPT_FORMAT_0_COLOR_LOC", C2H_CMPT_FORMAT_0_COLOR_LOC_MASK},
};

// MD: Debug: Log the initialization of C2H completion format 0 field info
qdma_log_debug("Initialized c2h_cmpt_format_0_field_info\n");

// MD: Structure to hold information about C2H completion format 1 fields
static struct regfield_info c2h_cmpt_format_1_field_info[] = {
    {"C2H_CMPT_FORMAT_1_DESC_ERR_LOC", C2H_CMPT_FORMAT_1_DESC_ERR_LOC_MASK},
    {"C2H_CMPT_FORMAT_1_COLOR_LOC", C2H_CMPT_FORMAT_1_COLOR_LOC_MASK},
};

// MD: Debug: Log the initialization of C2H completion format 1 field info
qdma_log_debug("Initialized c2h_cmpt_format_1_field_info\n");

// MD: Structure to hold information about C2H completion format 2 fields
static struct regfield_info c2h_cmpt_format_2_field_info[] = {
    {"C2H_CMPT_FORMAT_2_DESC_ERR_LOC", C2H_CMPT_FORMAT_2_DESC_ERR_LOC_MASK},
    {"C2H_CMPT_FORMAT_2_COLOR_LOC", C2H_CMPT_FORMAT_2_COLOR_LOC_MASK},
};

// MD: Debug: Log the initialization of C2H completion format 2 field info
qdma_log_debug("Initialized c2h_cmpt_format_2_field_info\n");

// MD: Structure to hold information about C2H completion format 3 fields
static struct regfield_info c2h_cmpt_format_3_field_info[] = {
    {"C2H_CMPT_FORMAT_3_DESC_ERR_LOC", C2H_CMPT_FORMAT_3_DESC_ERR_LOC_MASK},
    {"C2H_CMPT_FORMAT_3_COLOR_LOC", C2H_CMPT_FORMAT_3_COLOR_LOC_MASK},
};

// MD: Debug: Log the initialization of C2H completion format 3 field info
qdma_log_debug("Initialized c2h_cmpt_format_3_field_info\n");

// MD: Structure to hold information about C2H completion format 4 fields
static struct regfield_info c2h_cmpt_format_4_field_info[] = {
    {"C2H_CMPT_FORMAT_4_DESC_ERR_LOC", C2H_CMPT_FORMAT_4_DESC_ERR_LOC_MASK},
    {"C2H_CMPT_FORMAT_4_COLOR_LOC", C2H_CMPT_FORMAT_4_COLOR_LOC_MASK},
};

// MD: Debug: Log the initialization of C2H completion format 4 field info
qdma_log_debug("Initialized c2h_cmpt_format_4_field_info\n");

// MD: Structure to hold information about C2H completion format 5 fields
static struct regfield_info c2h_cmpt_format_5_field_info[] = {
    {"C2H_CMPT_FORMAT_5_DESC_ERR_LOC", C2H_CMPT_FORMAT_5_DESC_ERR_LOC_MASK},
    {"C2H_CMPT_FORMAT_5_COLOR_LOC", C2H_CMPT_FORMAT_5_COLOR_LOC_MASK},
};

// MD: Debug: Log the initialization of C2H completion format 5 field info
qdma_log_debug("Initialized c2h_cmpt_format_5_field_info\n");

// MD: Structure to hold information about C2H completion format 6 fields
static struct regfield_info c2h_cmpt_format_6_field_info[] = {
    {"C2H_CMPT_FORMAT_6_DESC_ERR_LOC", C2H_CMPT_FORMAT_6_DESC_ERR_LOC_MASK},
    {"C2H_CMPT_FORMAT_6_COLOR_LOC", C2H_CMPT_FORMAT_6_COLOR_LOC_MASK},
};

// MD: Debug: Log the initialization of C2H completion format 6 field info
qdma_log_debug("Initialized c2h_cmpt_format_6_field_info\n");

// MD: Structure to hold information about C2H prefetch cache depth fields
static struct regfield_info c2h_pfch_cache_depth_field_info[] = {
    {"C2H_PFCH_CACHE_DEPTH_MAX_STBUF", C2H_PFCH_CACHE_DEPTH_MAX_STBUF_MASK},
    {"C2H_PFCH_CACHE_DEPTH", C2H_PFCH_CACHE_DEPTH_MASK},
};

// MD: Debug: Log the initialization of C2H prefetch cache depth field info
qdma_log_debug("Initialized c2h_pfch_cache_depth_field_info\n");

// MD: Structure to hold information about C2H write-back coalescing buffer depth fields
static struct regfield_info c2h_wrb_coal_buf_depth_field_info[] = {
    {"C2H_WRB_COAL_BUF_DEPTH_RSVD_1", C2H_WRB_COAL_BUF_DEPTH_RSVD_1_MASK},
    {"C2H_WRB_COAL_BUF_DEPTH_BUFFER", C2H_WRB_COAL_BUF_DEPTH_BUFFER_MASK},
};

// MD: Debug: Log the initialization of C2H write-back coalescing buffer depth field info
qdma_log_debug("Initialized c2h_wrb_coal_buf_depth_field_info\n");

// MD: Structure to hold information about C2H prefetch credit fields
static struct regfield_info c2h_pfch_crdt_field_info[] = {
    {"C2H_PFCH_CRDT_RSVD_1", C2H_PFCH_CRDT_RSVD_1_MASK},
    {"C2H_PFCH_CRDT_RSVD_2", C2H_PFCH_CRDT_RSVD_2_MASK},
};

// MD: Debug: Log the initialization of C2H prefetch credit field info
qdma_log_debug("Initialized c2h_pfch_crdt_field_info\n");

// MD: Structure to hold information about C2H status has completion accepted fields
static struct regfield_info c2h_stat_has_cmpt_accepted_field_info[] = {
    {"C2H_STAT_HAS_CMPT_ACCEPTED_RSVD_1", C2H_STAT_HAS_CMPT_ACCEPTED_RSVD_1_MASK},
    {"C2H_STAT_HAS_CMPT_ACCEPTED_CNT", C2H_STAT_HAS_CMPT_ACCEPTED_CNT_MASK},
};

// MD: Debug: Log the initialization of C2H status has completion accepted field info
qdma_log_debug("Initialized c2h_stat_has_cmpt_accepted_field_info\n");

// MD: Structure to hold information about C2H status has payload accepted fields
static struct regfield_info c2h_stat_has_pld_accepted_field_info[] = {
    {"C2H_STAT_HAS_PLD_ACCEPTED_RSVD_1", C2H_STAT_HAS_PLD_ACCEPTED_RSVD_1_MASK},
    {"C2H_STAT_HAS_PLD_ACCEPTED_CNT", C2H_STAT_HAS_PLD_ACCEPTED_CNT_MASK},
};

// MD: Debug: Log the initialization of C2H status has payload accepted field info
qdma_log_debug("Initialized c2h_stat_has_pld_accepted_field_info\n");

// MD: Structure to hold information about C2H payload packet ID fields
static struct regfield_info c2h_pld_pkt_id_field_info[] = {
    {"C2H_PLD_PKT_ID_CMPT_WAIT", C2H_PLD_PKT_ID_CMPT_WAIT_MASK},
    {"C2H_PLD_PKT_ID_DATA", C2H_PLD_PKT_ID_DATA_MASK},
};

// MD: Debug: Log the initialization of C2H payload packet ID field info
qdma_log_debug("Initialized c2h_pld_pkt_id_field_info\n");

// MD: Structure to hold information about C2H payload packet ID 1 fields
static struct regfield_info c2h_pld_pkt_id_1_field_info[] = {
    {"C2H_PLD_PKT_ID_1_CMPT_WAIT", C2H_PLD_PKT_ID_1_CMPT_WAIT_MASK},
    {"C2H_PLD_PKT_ID_1_DATA", C2H_PLD_PKT_ID_1_DATA_MASK},
};

// MD: Debug: Log the initialization of C2H payload packet ID 1 field info
qdma_log_debug("Initialized c2h_pld_pkt_id_1_field_info\n");

// MD: Structure to hold information about C2H drop payload count 1 fields
static struct regfield_info c2h_drop_pld_cnt_1_field_info[] = {
    {"C2H_DROP_PLD_CNT_1_RSVD_1", C2H_DROP_PLD_CNT_1_RSVD_1_MASK},
    {"C2H_DROP_PLD_CNT_1_CNT", C2H_DROP_PLD_CNT_1_CNT_MASK},
};

// MD: Debug: Log the initialization of C2H drop payload count 1 field info
qdma_log_debug("Initialized c2h_drop_pld_cnt_1_field_info\n");

// MD: Structure to hold information about H2C error status fields
static struct regfield_info h2c_err_stat_field_info[] = {
    {"H2C_ERR_STAT_RSVD_1", H2C_ERR_STAT_RSVD_1_MASK},
    {"H2C_ERR_STAT_PAR_ERR", H2C_ERR_STAT_PAR_ERR_MASK},
    {"H2C_ERR_STAT_SBE", H2C_ERR_STAT_SBE_MASK},
    {"H2C_ERR_STAT_DBE", H2C_ERR_STAT_DBE_MASK},
    {"H2C_ERR_STAT_NO_DMA_DS", H2C_ERR_STAT_NO_DMA_DS_MASK},
    {"H2C_ERR_STAT_SDI_MRKR_REQ_MOP_ERR", H2C_ERR_STAT_SDI_MRKR_REQ_MOP_ERR_MASK},
    {"H2C_ERR_STAT_ZERO_LEN_DS", H2C_ERR_STAT_ZERO_LEN_DS_MASK},
};

// MD: Debug: Log the initialization of H2C error status field info
qdma_log_debug("Initialized h2c_err_stat_field_info\n");

// MD: Structure to hold information about H2C error mask fields
static struct regfield_info h2c_err_mask_field_info[] = {
    {"H2C_ERR_EN", H2C_ERR_EN_MASK},
};

// MD: Debug: Log the initialization of H2C error mask field info
qdma_log_debug("Initialized h2c_err_mask_field_info\n");

// MD: Structure to hold information about H2C first error QID fields
static struct regfield_info h2c_first_err_qid_field_info[] = {
    {"H2C_FIRST_ERR_QID_RSVD_1", H2C_FIRST_ERR_QID_RSVD_1_MASK},
    {"H2C_FIRST_ERR_QID_ERR_TYPE", H2C_FIRST_ERR_QID_ERR_TYPE_MASK},
    {"H2C_FIRST_ERR_QID_RSVD_2", H2C_FIRST_ERR_QID_RSVD_2_MASK},
    {"H2C_FIRST_ERR_QID_QID", H2C_FIRST_ERR_QID_QID_MASK},
};

// MD: Debug: Log the initialization of H2C first error QID field info
qdma_log_debug("Initialized h2c_first_err_qid_field_info\n");

// MD: Structure to hold information about H2C debug register 0 fields
static struct regfield_info h2c_dbg_reg0_field_info[] = {
    {"H2C_REG0_NUM_DSC_RCVD", H2C_REG0_NUM_DSC_RCVD_MASK},
    {"H2C_REG0_NUM_WRB_SENT", H2C_REG0_NUM_WRB_SENT_MASK},
};

// MD: Debug: Log the initialization of H2C debug register 0 field info
qdma_log_debug("Initialized h2c_dbg_reg0_field_info\n");

// MD: Structure to hold information about H2C debug register 1 fields
static struct regfield_info h2c_dbg_reg1_field_info[] = {
    {"H2C_REG1_NUM_REQ_SENT", H2C_REG1_NUM_REQ_SENT_MASK},
    {"H2C_REG1_NUM_CMP_SENT", H2C_REG1_NUM_CMP_SENT_MASK},
};

// MD: Debug: Log the initialization of H2C debug register 1 field info
qdma_log_debug("Initialized h2c_dbg_reg1_field_info\n");

// MD: Structure to hold information about H2C debug register 2 fields
static struct regfield_info h2c_dbg_reg2_field_info[] = {
    {"H2C_REG2_RSVD_1", H2C_REG2_RSVD_1_MASK},
    {"H2C_REG2_NUM_ERR_DSC_RCVD", H2C_REG2_NUM_ERR_DSC_RCVD_MASK},
};

// MD: Debug: Log the initialization of H2C debug register 2 field info
qdma_log_debug("Initialized h2c_dbg_reg2_field_info\n");

// MD: Structure to hold information about H2C debug register 3 fields
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
};

// MD: Debug: Log the initialization of H2C debug register 3 field info
qdma_log_debug("Initialized h2c_dbg_reg3_field_info\n");

// MD: Structure to hold information about H2C debug register 4 fields
static struct regfield_info h2c_dbg_reg4_field_info[] = {
    {"H2C_REG4_RDREQ_ADDR", H2C_REG4_RDREQ_ADDR_MASK},
};

// MD: Debug: Log the initialization of H2C debug register 4 field info
qdma_log_debug("Initialized h2c_dbg_reg4_field_info\n");

// MD: Structure to hold information about H2C fatal error enable fields
static struct regfield_info h2c_fatal_err_en_field_info[] = {
    {"H2C_FATAL_ERR_EN_RSVD_1", H2C_FATAL_ERR_EN_RSVD_1_MASK},
    {"H2C_FATAL_ERR_EN_H2C", H2C_FATAL_ERR_EN_H2C_MASK},
};

// MD: Debug: Log the initialization of H2C fatal error enable field info
qdma_log_debug("Initialized h2c_fatal_err_en_field_info\n");

// MD: Structure to hold information about H2C request throttle PCIe fields
static struct regfield_info h2c_req_throt_pcie_field_info[] = {
    {"H2C_REQ_THROT_PCIE_EN_REQ", H2C_REQ_THROT_PCIE_EN_REQ_MASK},
    {"H2C_REQ_THROT_PCIE", H2C_REQ_THROT_PCIE_MASK},
    {"H2C_REQ_THROT_PCIE_EN_DATA", H2C_REQ_THROT_PCIE_EN_DATA_MASK},
    {"H2C_REQ_THROT_PCIE_DATA_THRESH", H2C_REQ_THROT_PCIE_DATA_THRESH_MASK},
};

// MD: Debug: Log the initialization of H2C request throttle PCIe field info
qdma_log_debug("Initialized h2c_req_throt_pcie_field_info\n");

// MD: Structure to hold information about H2C alignment debug register 0 fields
static struct regfield_info h2c_aln_dbg_reg0_field_info[] = {
    {"H2C_ALN_REG0_NUM_PKT_SENT", H2C_ALN_REG0_NUM_PKT_SENT_MASK},
};

// MD: Debug: Log the initialization of H2C alignment debug register 0 field info
qdma_log_debug("Initialized h2c_aln_dbg_reg0_field_info\n");

// MD: Structure to hold information about H2C request throttle AXIMM fields
static struct regfield_info h2c_req_throt_aximm_field_info[] = {
    {"H2C_REQ_THROT_AXIMM_EN_REQ", H2C_REQ_THROT_AXIMM_EN_REQ_MASK},
    {"H2C_REQ_THROT_AXIMM", H2C_REQ_THROT_AXIMM_MASK},
    {"H2C_REQ_THROT_AXIMM_EN_DATA", H2C_REQ_THROT_AXIMM_EN_DATA_MASK},
    {"H2C_REQ_THROT_AXIMM_DATA_THRESH", H2C_REQ_THROT_AXIMM_DATA_THRESH_MASK},
};

// MD: Debug: Log the initialization of H2C request throttle AXIMM field info
qdma_log_debug("Initialized h2c_req_throt_aximm_field_info\n");

// MD: Structure to hold information about C2H MM control fields
static struct regfield_info c2h_mm_ctl_field_info[] = {
    {"C2H_MM_CTL_RESERVED1", C2H_MM_CTL_RESERVED1_MASK},
    {"C2H_MM_CTL_ERRC_EN", C2H_MM_CTL_ERRC_EN_MASK},
    {"C2H_MM_CTL_RESERVED0", C2H_MM_CTL_RESERVED0_MASK},
    {"C2H_MM_CTL_RUN", C2H_MM_CTL_RUN_MASK},
};

// MD: Debug: Log the initialization of C2H MM control field info
qdma_log_debug("Initialized c2h_mm_ctl_field_info\n");

// MD: Structure to hold information about C2H MM status fields
static struct regfield_info c2h_mm_status_field_info[] = {
    {"C2H_MM_STATUS_RSVD_1", C2H_MM_STATUS_RSVD_1_MASK},
    {"C2H_MM_STATUS_RUN", C2H_MM_STATUS_RUN_MASK},
};

// MD: Debug: Log the initialization of C2H MM status field info
qdma_log_debug("Initialized c2h_mm_status_field_info\n");

// MD: Structure to hold information about C2H MM completion descriptor count fields
static struct regfield_info c2h_mm_cmpl_desc_cnt_field_info[] = {
    {"C2H_MM_CMPL_DESC_CNT_C2H_CO", C2H_MM_CMPL_DESC_CNT_C2H_CO_MASK},
};

// MD: Debug: Log the initialization of C2H MM completion descriptor count field info
qdma_log_debug("Initialized c2h_mm_cmpl_desc_cnt_field_info\n");

// MD: Structure to hold information about C2H MM error code enable mask fields
static struct regfield_info c2h_mm_err_code_enable_mask_field_info[] = {
    {"C2H_MM_ERR_CODE_ENABLE_RESERVED1", C2H_MM_ERR_CODE_ENABLE_RESERVED1_MASK},
    {"C2H_MM_ERR_CODE_ENABLE_WR_UC_RAM", C2H_MM_ERR_CODE_ENABLE_WR_UC_RAM_MASK},
    {"C2H_MM_ERR_CODE_ENABLE_WR_UR", C2H_MM_ERR_CODE_ENABLE_WR_UR_MASK},
    {"C2H_MM_ERR_CODE_ENABLE_WR_FLR", C2H_MM_ERR_CODE_ENABLE_WR_FLR_MASK},
    {"C2H_MM_ERR_CODE_ENABLE_RESERVED0", C2H_MM_ERR_CODE_ENABLE_RESERVED0_MASK},
    {"C2H_MM_ERR_CODE_ENABLE_RD_SLV_ERR", C2H_MM_ERR_CODE_ENABLE_RD_SLV_ERR_MASK},
    {"C2H_MM_ERR_CODE_ENABLE_WR_SLV_ERR", C2H_MM_ERR_CODE_ENABLE_WR_SLV_ERR_MASK},
};

// MD: Debug: Log the initialization of C2H MM error code enable mask field info
qdma_log_debug("Initialized c2h_mm_err_code_enable_mask_field_info\n");

// MD: Structure to hold information about C2H MM error code fields
static struct regfield_info c2h_mm_err_code_field_info[] = {
    {"C2H_MM_ERR_CODE_RESERVED1", C2H_MM_ERR_CODE_RESERVED1_MASK},
    {"C2H_MM_ERR_CODE_CIDX", C2H_MM_ERR_CODE_CIDX_MASK},
    {"C2H_MM_ERR_CODE_RESERVED0", C2H_MM_ERR_CODE_RESERVED0_MASK},
    {"C2H_MM_ERR_CODE_SUB_TYPE", C2H_MM_ERR_CODE_SUB_TYPE_MASK},
    {"C2H_MM_ERR_CODE", C2H_MM_ERR_CODE_MASK},
};

// MD: Debug: Log the initialization of C2H MM error code field info
qdma_log_debug("Initialized c2h_mm_err_code_field_info\n");

// MD: Structure to hold information about C2H MM error info fields
static struct regfield_info c2h_mm_err_info_field_info[] = {
    {"C2H_MM_ERR_INFO_VALID", C2H_MM_ERR_INFO_VALID_MASK},
    {"C2H_MM_ERR_INFO_SEL", C2H_MM_ERR_INFO_SEL_MASK},
    {"C2H_MM_ERR_INFO_RSVD_1", C2H_MM_ERR_INFO_RSVD_1_MASK},
    {"C2H_MM_ERR_INFO_QID", C2H_MM_ERR_INFO_QID_MASK},
};

// MD: Debug: Log the initialization of C2H MM error info field info
qdma_log_debug("Initialized c2h_mm_err_info_field_info\n");

// MD: Structure to hold information about C2H MM performance monitor control fields
static struct regfield_info c2h_mm_perf_mon_ctl_field_info[] = {
    {"C2H_MM_PERF_MON_CTL_RSVD_1", C2H_MM_PERF_MON_CTL_RSVD_1_MASK},
    {"C2H_MM_PERF_MON_CTL_IMM_START", C2H_MM_PERF_MON_CTL_IMM_START_MASK},
    {"C2H_MM_PERF_MON_CTL_RUN_START", C2H_MM_PERF_MON_CTL_RUN_START_MASK},
    {"C2H_MM_PERF_MON_CTL_IMM_CLEAR", C2H_MM_PERF_MON_CTL_IMM_CLEAR_MASK},
    {"C2H_MM_PERF_MON_CTL_RUN_CLEAR", C2H_MM_PERF_MON_CTL_RUN_CLEAR_MASK},
};

// MD: Debug: Log the initialization of C2H MM performance monitor control field info
qdma_log_debug("Initialized c2h_mm_perf_mon_ctl_field_info\n");

// MD: Structure to hold information about C2H MM performance monitor cycle count 0 fields
static struct regfield_info c2h_mm_perf_mon_cycle_cnt0_field_info[] = {
    {"C2H_MM_PERF_MON_CYCLE_CNT0_CYC_CNT", C2H_MM_PERF_MON_CYCLE_CNT0_CYC_CNT_MASK},
};

// MD: Debug: Log the initialization of C2H MM performance monitor cycle count 0 field info
qdma_log_debug("Initialized c2h_mm_perf_mon_cycle_cnt0_field_info\n");

// MD: Structure to hold information about C2H MM performance monitor cycle count 1 fields
static struct regfield_info c2h_mm_perf_mon_cycle_cnt1_field_info[] = {
    {"C2H_MM_PERF_MON_CYCLE_CNT1_RSVD_1", C2H_MM_PERF_MON_CYCLE_CNT1_RSVD_1_MASK},
    {"C2H_MM_PERF_MON_CYCLE_CNT1_CYC_CNT", C2H_MM_PERF_MON_CYCLE_CNT1_CYC_CNT_MASK},
};

// MD: Debug: Log the initialization of C2H MM performance monitor cycle count 1 field info
qdma_log_debug("Initialized c2h_mm_perf_mon_cycle_cnt1_field_info\n");

// MD: Structure to hold information about C2H MM performance monitor data count 0 fields
static struct regfield_info c2h_mm_perf_mon_data_cnt0_field_info[] = {
    {"C2H_MM_PERF_MON_DATA_CNT0_DCNT", C2H_MM_PERF_MON_DATA_CNT0_DCNT_MASK},
};

// MD: Debug: Log the initialization of C2H MM performance monitor data count 0 field info
qdma_log_debug("Initialized c2h_mm_perf_mon_data_cnt0_field_info\n");

// MD: Structure to hold information about C2H MM performance monitor data count 1 fields
static struct regfield_info c2h_mm_perf_mon_data_cnt1_field_info[] = {
    {"C2H_MM_PERF_MON_DATA_CNT1_RSVD_1", C2H_MM_PERF_MON_DATA_CNT1_RSVD_1_MASK},
    {"C2H_MM_PERF_MON_DATA_CNT1_DCNT", C2H_MM_PERF_MON_DATA_CNT1_DCNT_MASK},
};

// MD: Debug: Log the initialization of C2H MM performance monitor data count 1 field info
qdma_log_debug("Initialized c2h_mm_perf_mon_data_cnt1_field_info\n");

// MD: Structure to hold information about C2H MM debug fields
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
};

// MD: Debug: Log the initialization of C2H MM debug field info
qdma_log_debug("Initialized c2h_mm_dbg_field_info\n");

// MD: Structure to hold information about H2C MM control fields
static struct regfield_info h2c_mm_ctl_field_info[] = {
    {"H2C_MM_CTL_RESERVED1", H2C_MM_CTL_RESERVED1_MASK},
    {"H2C_MM_CTL_ERRC_EN", H2C_MM_CTL_ERRC_EN_MASK},
    {"H2C_MM_CTL_RESERVED0", H2C_MM_CTL_RESERVED0_MASK},
    {"H2C_MM_CTL_RUN", H2C_MM_CTL_RUN_MASK},
};

// MD: Debug: Log the initialization of H2C MM control field info
qdma_log_debug("Initialized h2c_mm_ctl_field_info\n");

// MD: Structure to hold information about H2C MM status fields
static struct regfield_info h2c_mm_status_field_info[] = {
    {"H2C_MM_STATUS_RSVD_1", H2C_MM_STATUS_RSVD_1_MASK},
    {"H2C_MM_STATUS_RUN", H2C_MM_STATUS_RUN_MASK},
};

// MD: Debug: Log the initialization of H2C MM status field info
qdma_log_debug("Initialized h2c_mm_status_field_info\n");

// MD: Structure to hold information about H2C MM completion descriptor count fields
static struct regfield_info h2c_mm_cmpl_desc_cnt_field_info[] = {
    {"H2C_MM_CMPL_DESC_CNT_H2C_CO", H2C_MM_CMPL_DESC_CNT_H2C_CO_MASK},
};

// MD: Debug: Log the initialization of H2C MM completion descriptor count field info
qdma_log_debug("Initialized h2c_mm_cmpl_desc_cnt_field_info\n");

// MD: Structure to hold information about H2C MM error code enable mask fields
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

// MD: Debug: Log the initialization of H2C MM error code enable mask field info
qdma_log_debug("Initialized h2c_mm_err_code_enable_mask_field_info\n");

// MD: Structure to hold information about H2C MM error code fields
static struct regfield_info h2c_mm_err_code_field_info[] = {
    {"H2C_MM_ERR_CODE_RSVD_1", H2C_MM_ERR_CODE_RSVD_1_MASK},
    {"H2C_MM_ERR_CODE_CIDX", H2C_MM_ERR_CODE_CIDX_MASK},
    {"H2C_MM_ERR_CODE_RESERVED0", H2C_MM_ERR_CODE_RESERVED0_MASK},
    {"H2C_MM_ERR_CODE_SUB_TYPE", H2C_MM_ERR_CODE_SUB_TYPE_MASK},
    {"H2C_MM_ERR_CODE", H2C_MM_ERR_CODE_MASK},
};

// MD: Debug: Log the initialization of H2C MM error code field info
qdma_log_debug("Initialized h2c_mm_err_code_field_info\n");

// MD: Structure to hold information about H2C MM error info fields
static struct regfield_info h2c_mm_err_info_field_info[] = {
    {"H2C_MM_ERR_INFO_VALID", H2C_MM_ERR_INFO_VALID_MASK},
    {"H2C_MM_ERR_INFO_SEL", H2C_MM_ERR_INFO_SEL_MASK},
    {"H2C_MM_ERR_INFO_RSVD_1", H2C_MM_ERR_INFO_RSVD_1_MASK},
    {"H2C_MM_ERR_INFO_QID", H2C_MM_ERR_INFO_QID_MASK},
};

// MD: Debug: Log the initialization of H2C MM error info field info
qdma_log_debug("Initialized h2c_mm_err_info_field_info\n");

// MD: Structure to hold information about H2C MM performance monitor control fields
static struct regfield_info h2c_mm_perf_mon_ctl_field_info[] = {
    {"H2C_MM_PERF_MON_CTL_RSVD_1", H2C_MM_PERF_MON_CTL_RSVD_1_MASK},
    {"H2C_MM_PERF_MON_CTL_IMM_START", H2C_MM_PERF_MON_CTL_IMM_START_MASK},
    {"H2C_MM_PERF_MON_CTL_RUN_START", H2C_MM_PERF_MON_CTL_RUN_START_MASK},
    {"H2C_MM_PERF_MON_CTL_IMM_CLEAR", H2C_MM_PERF_MON_CTL_IMM_CLEAR_MASK},
    {"H2C_MM_PERF_MON_CTL_RUN_CLEAR", H2C_MM_PERF_MON_CTL_RUN_CLEAR_MASK},
};

// MD: Debug: Log the initialization of H2C MM performance monitor control field info
qdma_log_debug("Initialized h2c_mm_perf_mon_ctl_field_info\n");

// MD: Structure to hold information about H2C MM performance monitor cycle count 0 fields
static struct regfield_info h2c_mm_perf_mon_cycle_cnt0_field_info[] = {
    {"H2C_MM_PERF_MON_CYCLE_CNT0_CYC_CNT", H2C_MM_PERF_MON_CYCLE_CNT0_CYC_CNT_MASK},
};

// MD: Debug: Log the initialization of H2C MM performance monitor cycle count 0 field info
qdma_log_debug("Initialized h2c_mm_perf_mon_cycle_cnt0_field_info\n");

// MD: Structure to hold information about H2C MM performance monitor cycle count 1 fields
static struct regfield_info h2c_mm_perf_mon_cycle_cnt1_field_info[] = {
    {"H2C_MM_PERF_MON_CYCLE_CNT1_RSVD_1", H2C_MM_PERF_MON_CYCLE_CNT1_RSVD_1_MASK},
    {"H2C_MM_PERF_MON_CYCLE_CNT1_CYC_CNT", H2C_MM_PERF_MON_CYCLE_CNT1_CYC_CNT_MASK},
};

// MD: Debug: Log the initialization of H2C MM performance monitor cycle count 1 field info
qdma_log_debug("Initialized h2c_mm_perf_mon_cycle_cnt1_field_info\n");

// MD: Structure to hold information about H2C MM performance monitor data count 0 fields
static struct regfield_info h2c_mm_perf_mon_data_cnt0_field_info[] = {
    {"H2C_MM_PERF_MON_DATA_CNT0_DCNT", H2C_MM_PERF_MON_DATA_CNT0_DCNT_MASK},
};

// MD: Debug: Log the initialization of H2C MM performance monitor data count 0 field info
qdma_log_debug("Initialized h2c_mm_perf_mon_data_cnt0_field_info\n");

// MD: Structure to hold information about H2C MM performance monitor data count 1 fields
static struct regfield_info h2c_mm_perf_mon_data_cnt1_field_info[] = {
    {"H2C_MM_PERF_MON_DATA_CNT1_RSVD_1", H2C_MM_PERF_MON_DATA_CNT1_RSVD_1_MASK},
    {"H2C_MM_PERF_MON_DATA_CNT1_DCNT", H2C_MM_PERF_MON_DATA_CNT1_DCNT_MASK},
};

// MD: Debug: Log the initialization of H2C MM performance monitor data count 1 field info
qdma_log_debug("Initialized h2c_mm_perf_mon_data_cnt1_field_info\n");

// MD: Structure to hold information about H2C MM debug fields
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
};

// MD: Debug: Log the initialization of H2C MM debug field info
qdma_log_debug("Initialized h2c_mm_dbg_field_info\n");

// MD: Structure to hold information about C2H credit coalescing configuration 1 fields
static struct regfield_info c2h_crdt_coal_cfg_1_field_info[] = {
    {"C2H_CRDT_COAL_CFG_1_RSVD_1", C2H_CRDT_COAL_CFG_1_RSVD_1_MASK},
    {"C2H_CRDT_COAL_CFG_1_PLD_FIFO_TH", C2H_CRDT_COAL_CFG_1_PLD_FIFO_TH_MASK},
    {"C2H_CRDT_COAL_CFG_1_TIMER_TH", C2H_CRDT_COAL_CFG_1_TIMER_TH_MASK},
};

// MD: Debug: Log the initialization of C2H credit coalescing configuration 1 field info
qdma_log_debug("Initialized c2h_crdt_coal_cfg_1_field_info\n");

// MD: Structure to hold information about C2H credit coalescing configuration 2 fields
static struct regfield_info c2h_crdt_coal_cfg_2_field_info[] = {
    {"C2H_CRDT_COAL_CFG_2_RSVD_1", C2H_CRDT_COAL_CFG_2_RSVD_1_MASK},
    {"C2H_CRDT_COAL_CFG_2_FIFO_TH", C2H_CRDT_COAL_CFG_2_FIFO_TH_MASK},
    {"C2H_CRDT_COAL_CFG_2_RESERVED1", C2H_CRDT_COAL_CFG_2_RESERVED1_MASK},
    {"C2H_CRDT_COAL_CFG_2_NT_TH", C2H_CRDT_COAL_CFG_2_NT_TH_MASK},
};

// MD: Debug: Log the initialization of C2H credit coalescing configuration 2 field info
qdma_log_debug("Initialized c2h_crdt_coal_cfg_2_field_info\n");

// MD: Structure to hold information about C2H prefetch bypass QID fields
static struct regfield_info c2h_pfch_byp_qid_field_info[] = {
    {"C2H_PFCH_BYP_QID_RSVD_1", C2H_PFCH_BYP_QID_RSVD_1_MASK},
    {"C2H_PFCH_BYP_QID", C2H_PFCH_BYP_QID_MASK},
};

// MD: Debug: Log the initialization of C2H prefetch bypass QID field info
qdma_log_debug("Initialized c2h_pfch_byp_qid_field_info\n");

// MD: Structure to hold information about C2H prefetch bypass tag fields
static struct regfield_info c2h_pfch_byp_tag_field_info[] = {
    {"C2H_PFCH_BYP_TAG_RSVD_1", C2H_PFCH_BYP_TAG_RSVD_1_MASK},
    {"C2H_PFCH_BYP_TAG_BYP_QID", C2H_PFCH_BYP_TAG_BYP_QID_MASK},
    {"C2H_PFCH_BYP_TAG_RSVD_2", C2H_PFCH_BYP_TAG_RSVD_2_MASK},
    {"C2H_PFCH_BYP_TAG", C2H_PFCH_BYP_TAG_MASK},
};

// MD: Debug: Log the initialization of C2H prefetch bypass tag field info
qdma_log_debug("Initialized c2h_pfch_byp_tag_field_info\n");

// MD: Structure to hold information about C2H water mark fields
static struct regfield_info c2h_water_mark_field_info[] = {
    {"C2H_WATER_MARK_HIGH_WM", C2H_WATER_MARK_HIGH_WM_MASK},
    {"C2H_WATER_MARK_LOW_WM", C2H_WATER_MARK_LOW_WM_MASK},
};

// MD: Debug: Log the initialization of C2H water mark field info
qdma_log_debug("Initialized c2h_water_mark_field_info\n");

// MD: Structure to hold information about C2H notify empty fields
static struct regfield_info c2h_notify_empty_field_info[] = {
    {"C2H_NOTIFY_EMPTY_RSVD_1", C2H_NOTIFY_EMPTY_RSVD_1_MASK},
    {"C2H_NOTIFY_EMPTY_NOE", C2H_NOTIFY_EMPTY_NOE_MASK},
};

// MD: Debug: Log the initialization of C2H notify empty field info
qdma_log_debug("Initialized c2h_notify_empty_field_info\n");

// MD: Structure to hold information about C2H status S_AXIS C2H accepted 1 fields
static struct regfield_info c2h_stat_s_axis_c2h_accepted_1_field_info[] = {
    {"C2H_STAT_S_AXIS_C2H_ACCEPTED_1", C2H_STAT_S_AXIS_C2H_ACCEPTED_1_MASK},
};

// MD: Debug: Log the initialization of C2H status S_AXIS C2H accepted 1 field info
qdma_log_debug("Initialized c2h_stat_s_axis_c2h_accepted_1_field_info\n");

// MD: Structure to hold information about C2H status S_AXIS WRB accepted 1 fields
static struct regfield_info c2h_stat_s_axis_wrb_accepted_1_field_info[] = {
    {"C2H_STAT_S_AXIS_WRB_ACCEPTED_1", C2H_STAT_S_AXIS_WRB_ACCEPTED_1_MASK},
};

// MD: Debug: Log the initialization of C2H status S_AXIS WRB accepted 1 field info
qdma_log_debug("Initialized c2h_stat_s_axis_wrb_accepted_1_field_info\n");

// MD: Structure to hold information about C2H status descriptor response packet accepted 1 fields
static struct regfield_info c2h_stat_desc_rsp_pkt_accepted_1_field_info[] = {
    {"C2H_STAT_DESC_RSP_PKT_ACCEPTED_1_D", C2H_STAT_DESC_RSP_PKT_ACCEPTED_1_D_MASK},
};

// MD: Debug: Log the initialization of C2H status descriptor response packet accepted 1 field info
qdma_log_debug("Initialized c2h_stat_desc_rsp_pkt_accepted_1_field_info\n");

// MD: Structure to hold information about C2H status AXIS package complete 1 fields
static struct regfield_info c2h_stat_axis_pkg_cmp_1_field_info[] = {
    {"C2H_STAT_AXIS_PKG_CMP_1", C2H_STAT_AXIS_PKG_CMP_1_MASK},
};

// MD: Debug: Log the initialization of C2H status AXIS package complete 1 field info
qdma_log_debug("Initialized c2h_stat_axis_pkg_cmp_1_field_info\n");

// MD: Structure to hold information about C2H status S_AXIS WRB accepted 2 fields
static struct regfield_info c2h_stat_s_axis_wrb_accepted_2_field_info[] = {
    {"C2H_STAT_S_AXIS_WRB_ACCEPTED_2", C2H_STAT_S_AXIS_WRB_ACCEPTED_2_MASK},
};

// MD: Debug: Log the initialization of C2H status S_AXIS WRB accepted 2 field info
qdma_log_debug("Initialized c2h_stat_s_axis_wrb_accepted_2_field_info\n");

// MD: Structure to hold information about C2H ST payload FIFO depth fields
static struct regfield_info c2h_st_pld_fifo_depth_field_info[] = {
    {"C2H_ST_PLD_FIFO_DEPTH", C2H_ST_PLD_FIFO_DEPTH_MASK},
};

// MD: Debug: Log the initialization of C2H ST payload FIFO depth field info
qdma_log_debug("Initialized c2h_st_pld_fifo_depth_field_info\n");

// MD: Structure to hold information about global channel capability fields
static struct xreg_info eqdma_cpm5_config_regs[] = {
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
{"C2H_STAT_S_AXIS_C2H_ACCEPTED", 0xa88,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(c2h_stat_s_axis_c2h_accepted_field_info),
	c2h_stat_s_axis_c2h_accepted_field_info
},
{"C2H_STAT_S_AXIS_WRB_ACCEPTED", 0xa8c,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(c2h_stat_s_axis_wrb_accepted_field_info),
	c2h_stat_s_axis_wrb_accepted_field_info
},
{"C2H_STAT_DESC_RSP_PKT_ACCEPTED", 0xa90,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(c2h_stat_desc_rsp_pkt_accepted_field_info),
	c2h_stat_desc_rsp_pkt_accepted_field_info
},
{"C2H_STAT_AXIS_PKG_CMP", 0xa94,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(c2h_stat_axis_pkg_cmp_field_info),
	c2h_stat_axis_pkg_cmp_field_info
},
{"C2H_STAT_DESC_RSP_ACCEPTED", 0xa98,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_stat_desc_rsp_accepted_field_info),
	c2h_stat_desc_rsp_accepted_field_info
},
{"C2H_STAT_DESC_RSP_CMP", 0xa9c,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_stat_desc_rsp_cmp_field_info),
	c2h_stat_desc_rsp_cmp_field_info
},
{"C2H_STAT_WRQ_OUT", 0xaa0,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_stat_wrq_out_field_info),
	c2h_stat_wrq_out_field_info
},
{"C2H_STAT_WPL_REN_ACCEPTED", 0xaa4,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_stat_wpl_ren_accepted_field_info),
	c2h_stat_wpl_ren_accepted_field_info
},
{"C2H_STAT_TOTAL_WRQ_LEN", 0xaa8,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_stat_total_wrq_len_field_info),
	c2h_stat_total_wrq_len_field_info
},
{"C2H_STAT_TOTAL_WPL_LEN", 0xaac,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
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
{"C2H_INT_TIMER_TICK", 0xb0c,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_int_timer_tick_field_info),
	c2h_int_timer_tick_field_info
},
{"C2H_STAT_DESC_RSP_DROP_ACCEPTED", 0xb10,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(c2h_stat_desc_rsp_drop_accepted_field_info),
	c2h_stat_desc_rsp_drop_accepted_field_info
},
{"C2H_STAT_DESC_RSP_ERR_ACCEPTED", 0xb14,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_VF,
	ARRAY_SIZE(c2h_stat_desc_rsp_err_accepted_field_info),
	c2h_stat_desc_rsp_err_accepted_field_info
},
{"C2H_STAT_DESC_REQ", 0xb18,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_stat_desc_req_field_info),
	c2h_stat_desc_req_field_info
},
{"C2H_STAT_DBG_DMA_ENG_0", 0xb1c,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_stat_dbg_dma_eng_0_field_info),
	c2h_stat_dbg_dma_eng_0_field_info
},
{"C2H_STAT_DBG_DMA_ENG_1", 0xb20,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_stat_dbg_dma_eng_1_field_info),
	c2h_stat_dbg_dma_eng_1_field_info
},
{"C2H_STAT_DBG_DMA_ENG_2", 0xb24,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_stat_dbg_dma_eng_2_field_info),
	c2h_stat_dbg_dma_eng_2_field_info
},
{"C2H_STAT_DBG_DMA_ENG_3", 0xb28,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
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
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(stat_num_wrb_in_field_info),
	stat_num_wrb_in_field_info
},
{"STAT_NUM_WRB_OUT", 0xb38,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(stat_num_wrb_out_field_info),
	stat_num_wrb_out_field_info
},
{"STAT_NUM_WRB_DRP", 0xb3c,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(stat_num_wrb_drp_field_info),
	stat_num_wrb_drp_field_info
},
{"STAT_NUM_STAT_DESC_OUT", 0xb40,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
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
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_stat_has_cmpt_accepted_field_info),
	c2h_stat_has_cmpt_accepted_field_info
},
{"C2H_STAT_HAS_PLD_ACCEPTED", 0xbf0,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
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
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(h2c_dbg_reg0_field_info),
	h2c_dbg_reg0_field_info
},
{"H2C_DBG_REG1", 0xe10,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(h2c_dbg_reg1_field_info),
	h2c_dbg_reg1_field_info
},
{"H2C_DBG_REG2", 0xe14,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(h2c_dbg_reg2_field_info),
	h2c_dbg_reg2_field_info
},
{"H2C_DBG_REG3", 0xe18,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(h2c_dbg_reg3_field_info),
	h2c_dbg_reg3_field_info
},
{"H2C_DBG_REG4", 0xe1c,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
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
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
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
{"C2H_WATER_MARK", 0x1500,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_water_mark_field_info),
	c2h_water_mark_field_info
},
{"C2H_NOTIFY_EMPTY", 0x1800,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_notify_empty_field_info),
	c2h_notify_empty_field_info
},
{"C2H_STAT_S_AXIS_C2H_ACCEPTED_1", 0x1804,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_stat_s_axis_c2h_accepted_1_field_info),
	c2h_stat_s_axis_c2h_accepted_1_field_info
},
{"C2H_STAT_S_AXIS_WRB_ACCEPTED_1", 0x1808,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_stat_s_axis_wrb_accepted_1_field_info),
	c2h_stat_s_axis_wrb_accepted_1_field_info
},
{"C2H_STAT_DESC_RSP_PKT_ACCEPTED_1", 0x180c,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_stat_desc_rsp_pkt_accepted_1_field_info),
	c2h_stat_desc_rsp_pkt_accepted_1_field_info
},
{"C2H_STAT_AXIS_PKG_CMP_1", 0x1810,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_stat_axis_pkg_cmp_1_field_info),
	c2h_stat_axis_pkg_cmp_1_field_info
},
{"C2H_STAT_S_AXIS_WRB_ACCEPTED_2", 0x1814,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_stat_s_axis_wrb_accepted_2_field_info),
	c2h_stat_s_axis_wrb_accepted_2_field_info
},
{"C2H_ST_PLD_FIFO_DEPTH", 0x1818,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_st_pld_fifo_depth_field_info),
	c2h_st_pld_fifo_depth_field_info
},

};

// MD: Function to get the number of configuration registers
uint32_t eqdma_cpm5_config_num_regs_get(void)
{
    // MD: Calculate the number of elements in the eqdma_cpm5_config_regs array
    uint32_t num_regs = sizeof(eqdma_cpm5_config_regs) / sizeof(eqdma_cpm5_config_regs[0]);

    // MD: Debug: Log the number of configuration registers
    qdma_log_debug("Number of configuration registers: %u\n", num_regs);

    return num_regs;
}

// MD: Function to get the pointer to the configuration registers array
struct xreg_info *eqdma_cpm5_config_regs_get(void)
{
    // MD: Debug: Log the retrieval of the configuration registers array
    qdma_log_debug("Retrieving pointer to configuration registers array\n");

    return eqdma_cpm5_config_regs;
}
