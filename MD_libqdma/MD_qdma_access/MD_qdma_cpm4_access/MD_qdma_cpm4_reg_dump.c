/* MD :
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

#include "qdma_cpm4_reg.h"
#include "qdma_reg_dump.h"

#ifdef ENABLE_WPP_TRACING
#include "qdma_cpm4_reg_dump.tmh"
#endif

// MD : Structure to hold register field information for block identifiers
static struct regfield_info cfg_blk_identifier_field_info[] = {
    {"CFG_BLK_IDENTIFIER", CFG_BLK_IDENTIFIER_MASK},
    {"CFG_BLK_IDENTIFIER_1", CFG_BLK_IDENTIFIER_1_MASK},
    {"CFG_BLK_IDENTIFIER_RSVD_1", CFG_BLK_IDENTIFIER_RSVD_1_MASK},
    {"CFG_BLK_IDENTIFIER_VERSION", CFG_BLK_IDENTIFIER_VERSION_MASK},
    // MD : Debug: Track initialization of block identifier field info
    printk(KERN_DEBUG "Initialized cfg_blk_identifier_field_info with %d entries\n", sizeof(cfg_blk_identifier_field_info)/sizeof(cfg_blk_identifier_field_info[0]));
};

// MD : Structure to hold register field information for bus device fields
static struct regfield_info cfg_blk_busdev_field_info[] = {
    {"CFG_BLK_BUSDEV_BDF", CFG_BLK_BUSDEV_BDF_MASK},
    // MD : Debug: Track initialization of bus device field info
    printk(KERN_DEBUG "Initialized cfg_blk_busdev_field_info with %d entries\n", sizeof(cfg_blk_busdev_field_info)/sizeof(cfg_blk_busdev_field_info[0]));
};

// MD : Structure to hold register field information for PCIe max payload size
static struct regfield_info cfg_blk_pcie_max_pld_size_field_info[] = {
    {"CFG_BLK_PCIE_MAX_PLD_SIZE", CFG_BLK_PCIE_MAX_PLD_SIZE_MASK},
    // MD : Debug: Track initialization of PCIe max payload size field info
    printk(KERN_DEBUG "Initialized cfg_blk_pcie_max_pld_size_field_info with %d entries\n", sizeof(cfg_blk_pcie_max_pld_size_field_info)/sizeof(cfg_blk_pcie_max_pld_size_field_info[0]));
};

// MD : Structure to hold register field information for PCIe max read request size
static struct regfield_info cfg_blk_pcie_max_read_req_size_field_info[] = {
    {"CFG_BLK_PCIE_MAX_READ_REQ_SIZE", CFG_BLK_PCIE_MAX_READ_REQ_SIZE_MASK},
    // MD : Debug: Track initialization of PCIe max read request size field info
    printk(KERN_DEBUG "Initialized cfg_blk_pcie_max_read_req_size_field_info with %d entries\n", sizeof(cfg_blk_pcie_max_read_req_size_field_info)/sizeof(cfg_blk_pcie_max_read_req_size_field_info[0]));
};

// MD : Structure to hold register field information for system ID
static struct regfield_info cfg_blk_system_id_field_info[] = {
    {"CFG_BLK_SYSTEM_ID", CFG_BLK_SYSTEM_ID_MASK},
    // MD : Debug: Track initialization of system ID field info
    printk(KERN_DEBUG "Initialized cfg_blk_system_id_field_info with %d entries\n", sizeof(cfg_blk_system_id_field_info)/sizeof(cfg_blk_system_id_field_info[0]));
};

// MD : Structure to hold register field information for MSI enable fields
static struct regfield_info cfg_blk_msi_enable_field_info[] = {
    {"CFG_BLK_MSI_ENABLE_3", CFG_BLK_MSI_ENABLE_3_MASK},
    {"CFG_BLK_MSI_ENABLE_MSIX3", CFG_BLK_MSI_ENABLE_MSIX3_MASK},
    {"CFG_BLK_MSI_ENABLE_2", CFG_BLK_MSI_ENABLE_2_MASK},
    {"CFG_BLK_MSI_ENABLE_MSIX2", CFG_BLK_MSI_ENABLE_MSIX2_MASK},
    {"CFG_BLK_MSI_ENABLE_1", CFG_BLK_MSI_ENABLE_1_MASK},
    {"CFG_BLK_MSI_ENABLE_MSIX1", CFG_BLK_MSI_ENABLE_MSIX1_MASK},
    {"CFG_BLK_MSI_ENABLE_0", CFG_BLK_MSI_ENABLE_0_MASK},
    {"CFG_BLK_MSI_ENABLE_MSIX0", CFG_BLK_MSI_ENABLE_MSIX0_MASK},
    // MD : Debug: Track initialization of MSI enable field info
    printk(KERN_DEBUG "Initialized cfg_blk_msi_enable_field_info with %d entries\n", sizeof(cfg_blk_msi_enable_field_info)/sizeof(cfg_blk_msi_enable_field_info[0]));
};

// MD : Structure to hold register field information for PCIe data width
static struct regfield_info cfg_pcie_data_width_field_info[] = {
    {"CFG_PCIE_DATA_WIDTH_DATAPATH", CFG_PCIE_DATA_WIDTH_DATAPATH_MASK},
    // MD : Debug: Track initialization of PCIe data width field info
    printk(KERN_DEBUG "Initialized cfg_pcie_data_width_field_info with %d entries\n", sizeof(cfg_pcie_data_width_field_info)/sizeof(cfg_pcie_data_width_field_info[0]));
};

// MD : Structure to hold register field information for PCIe control fields
static struct regfield_info cfg_pcie_ctl_field_info[] = {
    {"CFG_PCIE_CTL_RRQ_DISABLE", CFG_PCIE_CTL_RRQ_DISABLE_MASK},
    {"CFG_PCIE_CTL_RELAXED_ORDERING", CFG_PCIE_CTL_RELAXED_ORDERING_MASK},
    // MD : Debug: Track initialization of PCIe control field info
    printk(KERN_DEBUG "Initialized cfg_pcie_ctl_field_info with %d entries\n", sizeof(cfg_pcie_ctl_field_info)/sizeof(cfg_pcie_ctl_field_info[0]));
};

// MD : Structure to hold register field information for AXI user max payload size
static struct regfield_info cfg_axi_user_max_pld_size_field_info[] = {
    {"CFG_AXI_USER_MAX_PLD_SIZE_ISSUED", CFG_AXI_USER_MAX_PLD_SIZE_ISSUED_MASK},
    {"CFG_AXI_USER_MAX_PLD_SIZE_PROG", CFG_AXI_USER_MAX_PLD_SIZE_PROG_MASK},
    // MD : Debug: Track initialization of AXI user max payload size field info
    printk(KERN_DEBUG "Initialized cfg_axi_user_max_pld_size_field_info with %d entries\n", sizeof(cfg_axi_user_max_pld_size_field_info)/sizeof(cfg_axi_user_max_pld_size_field_info[0]));
};

// MD : Structure to hold register field information for AXI user max read request size
static struct regfield_info cfg_axi_user_max_read_req_size_field_info[] = {
    {"CFG_AXI_USER_MAX_READ_REQ_SIZE_USISSUED", CFG_AXI_USER_MAX_READ_REQ_SIZE_USISSUED_MASK},
    {"CFG_AXI_USER_MAX_READ_REQ_SIZE_USPROG", CFG_AXI_USER_MAX_READ_REQ_SIZE_USPROG_MASK},
    // MD : Debug: Track initialization of AXI user max read request size field info
    printk(KERN_DEBUG "Initialized cfg_axi_user_max_read_req_size_field_info with %d entries\n", sizeof(cfg_axi_user_max_read_req_size_field_info)/sizeof(cfg_axi_user_max_read_req_size_field_info[0]));
};

// MD : Structure to hold register field information for miscellaneous control fields
static struct regfield_info cfg_blk_misc_ctl_field_info[] = {
    {"CFG_BLK_MISC_CTL_NUM_TAG", CFG_BLK_MISC_CTL_NUM_TAG_MASK},
    {"CFG_BLK_MISC_CTL_RQ_METERING_MULTIPLIER", CFG_BLK_MISC_CTL_RQ_METERING_MULTIPLIER_MASK},
    // MD : Debug: Track initialization of miscellaneous control field info
    printk(KERN_DEBUG "Initialized cfg_blk_misc_ctl_field_info with %d entries\n", sizeof(cfg_blk_misc_ctl_field_info)/sizeof(cfg_blk_misc_ctl_field_info[0]));
};

// MD : Structure to hold register field information for scratch registers
static struct regfield_info cfg_blk_scratch_0_field_info[] = {
    {"CFG_BLK_SCRATCH_0", CFG_BLK_SCRATCH_0_MASK},
    // MD : Debug: Track initialization of scratch 0 field info
    printk(KERN_DEBUG "Initialized cfg_blk_scratch_0_field_info with %d entries\n", sizeof(cfg_blk_scratch_0_field_info)/sizeof(cfg_blk_scratch_0_field_info[0]));
};

static struct regfield_info cfg_blk_scratch_1_field_info[] = {
    {"CFG_BLK_SCRATCH_1", CFG_BLK_SCRATCH_1_MASK},
    // MD : Debug: Track initialization of scratch 1 field info
    printk(KERN_DEBUG "Initialized cfg_blk_scratch_1_field_info with %d entries\n", sizeof(cfg_blk_scratch_1_field_info)/sizeof(cfg_blk_scratch_1_field_info[0]));
};

static struct regfield_info cfg_blk_scratch_2_field_info[] = {
    {"CFG_BLK_SCRATCH_2", CFG_BLK_SCRATCH_2_MASK},
    // MD : Debug: Track initialization of scratch 2 field info
    printk(KERN_DEBUG "Initialized cfg_blk_scratch_2_field_info with %d entries\n", sizeof(cfg_blk_scratch_2_field_info)/sizeof(cfg_blk_scratch_2_field_info[0]));
};

static struct regfield_info cfg_blk_scratch_3_field_info[] = {
    {"CFG_BLK_SCRATCH_3", CFG_BLK_SCRATCH_3_MASK},
    // MD : Debug: Track initialization of scratch 3 field info
    printk(KERN_DEBUG "Initialized cfg_blk_scratch_3_field_info with %d entries\n", sizeof(cfg_blk_scratch_3_field_info)/sizeof(cfg_blk_scratch_3_field_info[0]));
};

static struct regfield_info cfg_blk_scratch_4_field_info[] = {
    {"CFG_BLK_SCRATCH_4", CFG_BLK_SCRATCH_4_MASK},
    // MD : Debug: Track initialization of scratch 4 field info
    printk(KERN_DEBUG "Initialized cfg_blk_scratch_4_field_info with %d entries\n", sizeof(cfg_blk_scratch_4_field_info)/sizeof(cfg_blk_scratch_4_field_info[0]));
};

static struct regfield_info cfg_blk_scratch_5_field_info[] = {
    {"CFG_BLK_SCRATCH_5", CFG_BLK_SCRATCH_5_MASK},
    // MD : Debug: Track initialization of scratch 5 field info
    printk(KERN_DEBUG "Initialized cfg_blk_scratch_5_field_info with %d entries\n", sizeof(cfg_blk_scratch_5_field_info)/sizeof(cfg_blk_scratch_5_field_info[0]));
};

static struct regfield_info cfg_blk_scratch_6_field_info[] = {
    {"CFG_BLK_SCRATCH_6", CFG_BLK_SCRATCH_6_MASK},
    // MD : Debug: Track initialization of scratch 6 field info
    printk(KERN_DEBUG "Initialized cfg_blk_scratch_6_field_info with %d entries\n", sizeof(cfg_blk_scratch_6_field_info)/sizeof(cfg_blk_scratch_6_field_info[0]));
};

// MD : Structure to hold register field information for scratch register 7
static struct regfield_info cfg_blk_scratch_7_field_info[] = {
    {"CFG_BLK_SCRATCH_7", CFG_BLK_SCRATCH_7_MASK},
    // MD : Debug: Track initialization of scratch 7 field info
    printk(KERN_DEBUG "Initialized cfg_blk_scratch_7_field_info with %d entries\n", sizeof(cfg_blk_scratch_7_field_info)/sizeof(cfg_blk_scratch_7_field_info[0]));
};

// MD : Structure to hold register field information for RAM single-bit error mask A
static struct regfield_info ram_sbe_msk_a_field_info[] = {
    {"RAM_SBE_MSK_A", RAM_SBE_MSK_A_MASK},
    // MD : Debug: Track initialization of RAM SBE mask A field info
    printk(KERN_DEBUG "Initialized ram_sbe_msk_a_field_info with %d entries\n", sizeof(ram_sbe_msk_a_field_info)/sizeof(ram_sbe_msk_a_field_info[0]));
};

// MD : Structure to hold register field information for RAM single-bit error status A
static struct regfield_info ram_sbe_sts_a_field_info[] = {
    {"RAM_SBE_STS_A_RSVD_1", RAM_SBE_STS_A_RSVD_1_MASK},
    {"RAM_SBE_STS_A_PFCH_LL_RAM", RAM_SBE_STS_A_PFCH_LL_RAM_MASK},
    {"RAM_SBE_STS_A_WRB_CTXT_RAM", RAM_SBE_STS_A_WRB_CTXT_RAM_MASK},
    {"RAM_SBE_STS_A_PFCH_CTXT_RAM", RAM_SBE_STS_A_PFCH_CTXT_RAM_MASK},
    {"RAM_SBE_STS_A_DESC_REQ_FIFO_RAM", RAM_SBE_STS_A_DESC_REQ_FIFO_RAM_MASK},
    {"RAM_SBE_STS_A_INT_CTXT_RAM", RAM_SBE_STS_A_INT_CTXT_RAM_MASK},
    {"RAM_SBE_STS_A_INT_QID2VEC_RAM", RAM_SBE_STS_A_INT_QID2VEC_RAM_MASK},
    {"RAM_SBE_STS_A_WRB_COAL_DATA_RAM", RAM_SBE_STS_A_WRB_COAL_DATA_RAM_MASK},
    {"RAM_SBE_STS_A_TUSER_FIFO_RAM", RAM_SBE_STS_A_TUSER_FIFO_RAM_MASK},
    {"RAM_SBE_STS_A_QID_FIFO_RAM", RAM_SBE_STS_A_QID_FIFO_RAM_MASK},
    {"RAM_SBE_STS_A_PLD_FIFO_RAM", RAM_SBE_STS_A_PLD_FIFO_RAM_MASK},
    {"RAM_SBE_STS_A_TIMER_FIFO_RAM", RAM_SBE_STS_A_TIMER_FIFO_RAM_MASK},
    {"RAM_SBE_STS_A_PASID_CTXT_RAM", RAM_SBE_STS_A_PASID_CTXT_RAM_MASK},
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
    {"RAM_SBE_STS_A_RSVD_2", RAM_SBE_STS_A_RSVD_2_MASK},
    {"RAM_SBE_STS_A_MI_C2H0_DAT", RAM_SBE_STS_A_MI_C2H0_DAT_MASK},
    {"RAM_SBE_STS_A_RSVD_3", RAM_SBE_STS_A_RSVD_3_MASK},
    {"RAM_SBE_STS_A_MI_H2C0_DAT", RAM_SBE_STS_A_MI_H2C0_DAT_MASK},
    // MD : Debug: Track initialization of RAM SBE status A field info
    printk(KERN_DEBUG "Initialized ram_sbe_sts_a_field_info with %d entries\n", sizeof(ram_sbe_sts_a_field_info)/sizeof(ram_sbe_sts_a_field_info[0]));
};

// MD : Structure to hold register field information for RAM double-bit error mask A
static struct regfield_info ram_dbe_msk_a_field_info[] = {
    {"RAM_DBE_MSK_A", RAM_DBE_MSK_A_MASK},
    // MD : Debug: Track initialization of RAM DBE mask A field info
    printk(KERN_DEBUG "Initialized ram_dbe_msk_a_field_info with %d entries\n", sizeof(ram_dbe_msk_a_field_info)/sizeof(ram_dbe_msk_a_field_info[0]));
};

// MD : Structure to hold register field information for RAM double-bit error status A
static struct regfield_info ram_dbe_sts_a_field_info[] = {
    {"RAM_DBE_STS_A_RSVD_1", RAM_DBE_STS_A_RSVD_1_MASK},
    {"RAM_DBE_STS_A_PFCH_LL_RAM", RAM_DBE_STS_A_PFCH_LL_RAM_MASK},
    {"RAM_DBE_STS_A_WRB_CTXT_RAM", RAM_DBE_STS_A_WRB_CTXT_RAM_MASK},
    {"RAM_DBE_STS_A_PFCH_CTXT_RAM", RAM_DBE_STS_A_PFCH_CTXT_RAM_MASK},
    {"RAM_DBE_STS_A_DESC_REQ_FIFO_RAM", RAM_DBE_STS_A_DESC_REQ_FIFO_RAM_MASK},
    {"RAM_DBE_STS_A_INT_CTXT_RAM", RAM_DBE_STS_A_INT_CTXT_RAM_MASK},
    {"RAM_DBE_STS_A_INT_QID2VEC_RAM", RAM_DBE_STS_A_INT_QID2VEC_RAM_MASK},
    {"RAM_DBE_STS_A_WRB_COAL_DATA_RAM", RAM_DBE_STS_A_WRB_COAL_DATA_RAM_MASK},
    {"RAM_DBE_STS_A_TUSER_FIFO_RAM", RAM_DBE_STS_A_TUSER_FIFO_RAM_MASK},
    {"RAM_DBE_STS_A_QID_FIFO_RAM", RAM_DBE_STS_A_QID_FIFO_RAM_MASK},
    {"RAM_DBE_STS_A_PLD_FIFO_RAM", RAM_DBE_STS_A_PLD_FIFO_RAM_MASK},
    {"RAM_DBE_STS_A_TIMER_FIFO_RAM", RAM_DBE_STS_A_TIMER_FIFO_RAM_MASK},
    {"RAM_DBE_STS_A_PASID_CTXT_RAM", RAM_DBE_STS_A_PASID_CTXT_RAM_MASK},
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
    {"RAM_DBE_STS_A_RSVD_2", RAM_DBE_STS_A_RSVD_2_MASK},
    {"RAM_DBE_STS_A_MI_C2H0_DAT", RAM_DBE_STS_A_MI_C2H0_DAT_MASK},
    {"RAM_DBE_STS_A_RSVD_3", RAM_DBE_STS_A_RSVD_3_MASK},
    {"RAM_DBE_STS_A_MI_H2C0_DAT", RAM_DBE_STS_A_MI_H2C0_DAT_MASK},
    // MD : Debug: Track initialization of RAM DBE status A field info
    printk(KERN_DEBUG "Initialized ram_dbe_sts_a_field_info with %d entries\n", sizeof(ram_dbe_sts_a_field_info)/sizeof(ram_dbe_sts_a_field_info[0]));
};

// MD : Structure to hold register field information for global identifier
static struct regfield_info glbl2_identifier_field_info[] = {
    {"GLBL2_IDENTIFIER", GLBL2_IDENTIFIER_MASK},
    {"GLBL2_IDENTIFIER_VERSION", GLBL2_IDENTIFIER_VERSION_MASK},
    // MD : Debug: Track initialization of global identifier field info
    printk(KERN_DEBUG "Initialized glbl2_identifier_field_info with %d entries\n", sizeof(glbl2_identifier_field_info)/sizeof(glbl2_identifier_field_info[0]));
};

// MD : Structure to hold register field information for PF BARLITE internal mapping
static struct regfield_info glbl2_pf_barlite_int_field_info[] = {
    {"GLBL2_PF_BARLITE_INT_PF3_BAR_MAP", GLBL2_PF_BARLITE_INT_PF3_BAR_MAP_MASK},
    {"GLBL2_PF_BARLITE_INT_PF2_BAR_MAP", GLBL2_PF_BARLITE_INT_PF2_BAR_MAP_MASK},
    {"GLBL2_PF_BARLITE_INT_PF1_BAR_MAP", GLBL2_PF_BARLITE_INT_PF1_BAR_MAP_MASK},
    {"GLBL2_PF_BARLITE_INT_PF0_BAR_MAP", GLBL2_PF_BARLITE_INT_PF0_BAR_MAP_MASK},
    // MD : Debug: Track initialization of PF BARLITE internal field info
    printk(KERN_DEBUG "Initialized glbl2_pf_barlite_int_field_info with %d entries\n", sizeof(glbl2_pf_barlite_int_field_info)/sizeof(glbl2_pf_barlite_int_field_info[0]));
};

// MD : Structure to hold register field information for PF VF BARLITE internal mapping
static struct regfield_info glbl2_pf_vf_barlite_int_field_info[] = {
    {"GLBL2_PF_VF_BARLITE_INT_PF3_MAP", GLBL2_PF_VF_BARLITE_INT_PF3_MAP_MASK},
    {"GLBL2_PF_VF_BARLITE_INT_PF2_MAP", GLBL2_PF_VF_BARLITE_INT_PF2_MAP_MASK},
    {"GLBL2_PF_VF_BARLITE_INT_PF1_MAP", GLBL2_PF_VF_BARLITE_INT_PF1_MAP_MASK},
    {"GLBL2_PF_VF_BARLITE_INT_PF0_MAP", GLBL2_PF_VF_BARLITE_INT_PF0_MAP_MASK},
    // MD : Debug: Track initialization of PF VF BARLITE internal field info
    printk(KERN_DEBUG "Initialized glbl2_pf_vf_barlite_int_field_info with %d entries\n", sizeof(glbl2_pf_vf_barlite_int_field_info)/sizeof(glbl2_pf_vf_barlite_int_field_info[0]));
};

// MD : Structure to hold register field information for PF BARLITE external mapping
static struct regfield_info glbl2_pf_barlite_ext_field_info[] = {
    {"GLBL2_PF_BARLITE_EXT_PF3_BAR_MAP", GLBL2_PF_BARLITE_EXT_PF3_BAR_MAP_MASK},
    {"GLBL2_PF_BARLITE_EXT_PF2_BAR_MAP", GLBL2_PF_BARLITE_EXT_PF2_BAR_MAP_MASK},
    {"GLBL2_PF_BARLITE_EXT_PF1_BAR_MAP", GLBL2_PF_BARLITE_EXT_PF1_BAR_MAP_MASK},
    {"GLBL2_PF_BARLITE_EXT_PF0_BAR_MAP", GLBL2_PF_BARLITE_EXT_PF0_BAR_MAP_MASK},
    // MD : Debug: Track initialization of PF BARLITE external field info
    printk(KERN_DEBUG "Initialized glbl2_pf_barlite_ext_field_info with %d entries\n", sizeof(glbl2_pf_barlite_ext_field_info)/sizeof(glbl2_pf_barlite_ext_field_info[0]));
};

// MD : Structure to hold register field information for PF VF BARLITE external mapping
static struct regfield_info glbl2_pf_vf_barlite_ext_field_info[] = {
    {"GLBL2_PF_VF_BARLITE_EXT_PF3_MAP", GLBL2_PF_VF_BARLITE_EXT_PF3_MAP_MASK},
    {"GLBL2_PF_VF_BARLITE_EXT_PF2_MAP", GLBL2_PF_VF_BARLITE_EXT_PF2_MAP_MASK},
    {"GLBL2_PF_VF_BARLITE_EXT_PF1_MAP", GLBL2_PF_VF_BARLITE_EXT_PF1_MAP_MASK},
    {"GLBL2_PF_VF_BARLITE_EXT_PF0_MAP", GLBL2_PF_VF_BARLITE_EXT_PF0_MAP_MASK},
    // MD : Debug: Track initialization of PF VF BARLITE external field info
    printk(KERN_DEBUG "Initialized glbl2_pf_vf_barlite_ext_field_info with %d entries\n", sizeof(glbl2_pf_vf_barlite_ext_field_info)/sizeof(glbl2_pf_vf_barlite_ext_field_info[0]));
};

// MD : Structure to hold register field information for channel instance
static struct regfield_info glbl2_channel_inst_field_info[] = {
    {"GLBL2_CHANNEL_INST_RSVD_1", GLBL2_CHANNEL_INST_RSVD_1_MASK},
    {"GLBL2_CHANNEL_INST_C2H_ST", GLBL2_CHANNEL_INST_C2H_ST_MASK},
    {"GLBL2_CHANNEL_INST_H2C_ST", GLBL2_CHANNEL_INST_H2C_ST_MASK},
    {"GLBL2_CHANNEL_INST_RSVD_2", GLBL2_CHANNEL_INST_RSVD_2_MASK},
    {"GLBL2_CHANNEL_INST_C2H_ENG", GLBL2_CHANNEL_INST_C2H_ENG_MASK},
    {"GLBL2_CHANNEL_INST_RSVD_3", GLBL2_CHANNEL_INST_RSVD_3_MASK},
    {"GLBL2_CHANNEL_INST_H2C_ENG", GLBL2_CHANNEL_INST_H2C_ENG_MASK},
    // MD : Debug: Track initialization of channel instance field info
    printk(KERN_DEBUG "Initialized glbl2_channel_inst_field_info with %d entries\n", sizeof(glbl2_channel_inst_field_info)/sizeof(glbl2_channel_inst_field_info[0]));
};

// MD : Structure to hold register field information for channel MDMA
static struct regfield_info glbl2_channel_mdma_field_info[] = {
    {"GLBL2_CHANNEL_MDMA_RSVD_1", GLBL2_CHANNEL_MDMA_RSVD_1_MASK},
    {"GLBL2_CHANNEL_MDMA_C2H_ST", GLBL2_CHANNEL_MDMA_C2H_ST_MASK},
    {"GLBL2_CHANNEL_MDMA_H2C_ST", GLBL2_CHANNEL_MDMA_H2C_ST_MASK},
    {"GLBL2_CHANNEL_MDMA_RSVD_2", GLBL2_CHANNEL_MDMA_RSVD_2_MASK},
    {"GLBL2_CHANNEL_MDMA_C2H_ENG", GLBL2_CHANNEL_MDMA_C2H_ENG_MASK},
    {"GLBL2_CHANNEL_MDMA_RSVD_3", GLBL2_CHANNEL_MDMA_RSVD_3_MASK},
    {"GLBL2_CHANNEL_MDMA_H2C_ENG", GLBL2_CHANNEL_MDMA_H2C_ENG_MASK},
    // MD : Debug: Track initialization of channel MDMA field info
    printk(KERN_DEBUG "Initialized glbl2_channel_mdma_field_info with %d entries\n", sizeof(glbl2_channel_mdma_field_info)/sizeof(glbl2_channel_mdma_field_info[0]));
};

// MD : Structure to hold register field information for channel stream
static struct regfield_info glbl2_channel_strm_field_info[] = {
    {"GLBL2_CHANNEL_STRM_RSVD_1", GLBL2_CHANNEL_STRM_RSVD_1_MASK},
    {"GLBL2_CHANNEL_STRM_C2H_ST", GLBL2_CHANNEL_STRM_C2H_ST_MASK},
    {"GLBL2_CHANNEL_STRM_H2C_ST", GLBL2_CHANNEL_STRM_H2C_ST_MASK},
    {"GLBL2_CHANNEL_STRM_RSVD_2", GLBL2_CHANNEL_STRM_RSVD_2_MASK},
    {"GLBL2_CHANNEL_STRM_C2H_ENG", GLBL2_CHANNEL_STRM_C2H_ENG_MASK},
    {"GLBL2_CHANNEL_STRM_RSVD_3", GLBL2_CHANNEL_STRM_RSVD_3_MASK},
    {"GLBL2_CHANNEL_STRM_H2C_ENG", GLBL2_CHANNEL_STRM_H2C_ENG_MASK},
    // MD : Debug: Track initialization of channel stream field info
    printk(KERN_DEBUG "Initialized glbl2_channel_strm_field_info with %d entries\n", sizeof(glbl2_channel_strm_field_info)/sizeof(glbl2_channel_strm_field_info[0]));
};

// MD : Structure to hold register field information for channel capabilities
static struct regfield_info glbl2_channel_cap_field_info[] = {
    {"GLBL2_CHANNEL_CAP_RSVD_1", GLBL2_CHANNEL_CAP_RSVD_1_MASK},
    {"GLBL2_CHANNEL_CAP_MULTIQ_MAX", GLBL2_CHANNEL_CAP_MULTIQ_MAX_MASK},
    // MD : Debug: Track initialization of channel capabilities field info
    printk(KERN_DEBUG "Initialized glbl2_channel_cap_field_info with %d entries\n", sizeof(glbl2_channel_cap_field_info)/sizeof(glbl2_channel_cap_field_info[0]));
};

// MD : Structure to hold register field information for channel PASID capabilities
static struct regfield_info glbl2_channel_pasid_cap_field_info[] = {
    {"GLBL2_CHANNEL_PASID_CAP_RSVD_1", GLBL2_CHANNEL_PASID_CAP_RSVD_1_MASK},
    {"GLBL2_CHANNEL_PASID_CAP_BRIDGEOFFSET", GLBL2_CHANNEL_PASID_CAP_BRIDGEOFFSET_MASK},
    {"GLBL2_CHANNEL_PASID_CAP_RSVD_2", GLBL2_CHANNEL_PASID_CAP_RSVD_2_MASK},
    {"GLBL2_CHANNEL_PASID_CAP_BRIDGEEN", GLBL2_CHANNEL_PASID_CAP_BRIDGEEN_MASK},
    {"GLBL2_CHANNEL_PASID_CAP_DMAEN", GLBL2_CHANNEL_PASID_CAP_DMAEN_MASK},
    // MD : Debug: Track initialization of channel PASID capabilities field info
    printk(KERN_DEBUG "Initialized glbl2_channel_pasid_cap_field_info with %d entries\n", sizeof(glbl2_channel_pasid_cap_field_info)/sizeof(glbl2_channel_pasid_cap_field_info[0]));
};

// MD : Structure to hold register field information for channel function return
static struct regfield_info glbl2_channel_func_ret_field_info[] = {
    {"GLBL2_CHANNEL_FUNC_RET_RSVD_1", GLBL2_CHANNEL_FUNC_RET_RSVD_1_MASK},
    {"GLBL2_CHANNEL_FUNC_RET_FUNC", GLBL2_CHANNEL_FUNC_RET_FUNC_MASK},
    // MD : Debug: Track initialization of channel function return field info
    printk(KERN_DEBUG "Initialized glbl2_channel_func_ret_field_info with %d entries\n", sizeof(glbl2_channel_func_ret_field_info)/sizeof(glbl2_channel_func_ret_field_info[0]));
};

// MD : Structure to hold register field information for system ID
static struct regfield_info glbl2_system_id_field_info[] = {
    {"GLBL2_SYSTEM_ID_RSVD_1", GLBL2_SYSTEM_ID_RSVD_1_MASK},
    {"GLBL2_SYSTEM_ID", GLBL2_SYSTEM_ID_MASK},
    // MD : Debug: Track initialization of system ID field info
    printk(KERN_DEBUG "Initialized glbl2_system_id_field_info with %d entries\n", sizeof(glbl2_system_id_field_info)/sizeof(glbl2_system_id_field_info[0]));
};

// MD : Structure to hold register field information for miscellaneous capabilities
static struct regfield_info glbl2_misc_cap_field_info[] = {
    {"GLBL2_MISC_CAP_RSVD_1", GLBL2_MISC_CAP_RSVD_1_MASK},
    // MD : Debug: Track initialization of miscellaneous capabilities field info
    printk(KERN_DEBUG "Initialized glbl2_misc_cap_field_info with %d entries\n", sizeof(glbl2_misc_cap_field_info)/sizeof(glbl2_misc_cap_field_info[0]));
};

// MD : Structure to hold register field information for PCIe request queue 0 debug
static struct regfield_info glbl2_dbg_pcie_rq0_field_info[] = {
    {"GLBL2_PCIE_RQ0_NPH_AVL", GLBL2_PCIE_RQ0_NPH_AVL_MASK},
    {"GLBL2_PCIE_RQ0_RCB_AVL", GLBL2_PCIE_RQ0_RCB_AVL_MASK},
    {"GLBL2_PCIE_RQ0_SLV_RD_CREDS", GLBL2_PCIE_RQ0_SLV_RD_CREDS_MASK},
    {"GLBL2_PCIE_RQ0_TAG_EP", GLBL2_PCIE_RQ0_TAG_EP_MASK},
    {"GLBL2_PCIE_RQ0_TAG_FL", GLBL2_PCIE_RQ0_TAG_FL_MASK},
    // MD : Debug: Track initialization of PCIe request queue 0 debug field info
    printk(KERN_DEBUG "Initialized glbl2_dbg_pcie_rq0_field_info with %d entries\n", sizeof(glbl2_dbg_pcie_rq0_field_info)/sizeof(glbl2_dbg_pcie_rq0_field_info[0]));
};

// MD : Structure to hold register field information for PCIe request queue 1 debug
static struct regfield_info glbl2_dbg_pcie_rq1_field_info[] = {
    {"GLBL2_PCIE_RQ1_RSVD_1", GLBL2_PCIE_RQ1_RSVD_1_MASK},
    {"GLBL2_PCIE_RQ1_WTLP_REQ", GLBL2_PCIE_RQ1_WTLP_REQ_MASK},
    {"GLBL2_PCIE_RQ1_WTLP_HEADER_FIFO_FL", GLBL2_PCIE_RQ1_WTLP_HEADER_FIFO_FL_MASK},
    {"GLBL2_PCIE_RQ1_WTLP_HEADER_FIFO_EP", GLBL2_PCIE_RQ1_WTLP_HEADER_FIFO_EP_MASK},
    {"GLBL2_PCIE_RQ1_RQ_FIFO_EP", GLBL2_PCIE_RQ1_RQ_FIFO_EP_MASK},
    {"GLBL2_PCIE_RQ1_RQ_FIFO_FL", GLBL2_PCIE_RQ1_RQ_FIFO_FL_MASK},
    {"GLBL2_PCIE_RQ1_TLPSM", GLBL2_PCIE_RQ1_TLPSM_MASK},
    {"GLBL2_PCIE_RQ1_TLPSM512", GLBL2_PCIE_RQ1_TLPSM512_MASK},
    {"GLBL2_PCIE_RQ1_RREQ0_RCB_OK", GLBL2_PCIE_RQ1_RREQ0_RCB_OK_MASK},
    {"GLBL2_PCIE_RQ1_RREQ0_SLV", GLBL2_PCIE_RQ1_RREQ0_SLV_MASK},
    {"GLBL2_PCIE_RQ1_RREQ0_VLD", GLBL2_PCIE_RQ1_RREQ0_VLD_MASK},
    {"GLBL2_PCIE_RQ1_RREQ1_RCB_OK", GLBL2_PCIE_RQ1_RREQ1_RCB_OK_MASK},
    {"GLBL2_PCIE_RQ1_RREQ1_SLV", GLBL2_PCIE_RQ1_RREQ1_SLV_MASK},
    {"GLBL2_PCIE_RQ1_RREQ1_VLD", GLBL2_PCIE_RQ1_RREQ1_VLD_MASK},
    // MD : Debug: Track initialization of PCIe request queue 1 debug field info
    printk(KERN_DEBUG "Initialized glbl2_dbg_pcie_rq1_field_info with %d entries\n", sizeof(glbl2_dbg_pcie_rq1_field_info)/sizeof(glbl2_dbg_pcie_rq1_field_info[0]));
};

// MD : Structure to hold register field information for AXI MM write channel 0 debug
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
    // MD : Debug: Track initialization of AXI MM write channel 0 debug field info
    printk(KERN_DEBUG "Initialized glbl2_dbg_aximm_wr0_field_info with %d entries\n", sizeof(glbl2_dbg_aximm_wr0_field_info)/sizeof(glbl2_dbg_aximm_wr0_field_info[0]));
};

// MD : Structure to hold register field information for AXI MM write channel 1 debug
static struct regfield_info glbl2_dbg_aximm_wr1_field_info[] = {
    {"GLBL2_AXIMM_WR1_RSVD_1", GLBL2_AXIMM_WR1_RSVD_1_MASK},
    {"GLBL2_AXIMM_WR1_BRSP_CNT4", GLBL2_AXIMM_WR1_BRSP_CNT4_MASK},
    {"GLBL2_AXIMM_WR1_BRSP_CNT3", GLBL2_AXIMM_WR1_BRSP_CNT3_MASK},
    {"GLBL2_AXIMM_WR1_BRSP_CNT2", GLBL2_AXIMM_WR1_BRSP_CNT2_MASK},
    {"GLBL2_AXIMM_WR1_BRSP_CNT1", GLBL2_AXIMM_WR1_BRSP_CNT1_MASK},
    {"GLBL2_AXIMM_WR1_BRSP_CNT0", GLBL2_AXIMM_WR1_BRSP_CNT0_MASK},
    // MD : Debug: Track initialization of AXI MM write channel 1 debug field info
    printk(KERN_DEBUG "Initialized glbl2_dbg_aximm_wr1_field_info with %d entries\n", sizeof(glbl2_dbg_aximm_wr1_field_info)/sizeof(glbl2_dbg_aximm_wr1_field_info[0]));
};

// MD : Structure to hold register field information for AXI MM read channel 0 debug
static struct regfield_info glbl2_dbg_aximm_rd0_field_info[] = {
    {"GLBL2_AXIMM_RD0_RSVD_1", GLBL2_AXIMM_RD0_RSVD_1_MASK},
    {"GLBL2_AXIMM_RD0_PND_CNT", GLBL2_AXIMM_RD0_PND_CNT_MASK},
    {"GLBL2_AXIMM_RD0_RD_CHNL", GLBL2_AXIMM_RD0_RD_CHNL_MASK},
    {"GLBL2_AXIMM_RD0_RD_REQ", GLBL2_AXIMM_RD0_RD_REQ_MASK},
    {"GLBL2_AXIMM_RD0_RRSP_CLAIM_CHNL", GLBL2_AXIMM_RD0_RRSP_CLAIM_CHNL_MASK},
    {"GLBL2_AXIMM_RD0_RID", GLBL2_AXIMM_RD0_RID_MASK},
    {"GLBL2_AXIMM_RD0_RVALID", GLBL2_AXIMM_RD0_RVALID_MASK},
    {"GLBL2_AXIMM_RD0_RREADY", GLBL2_AXIMM_RD0_RREADY_MASK},
    {"GLBL2_AXIMM_RD0_ARID", GLBL2_AXIMM_RD0_ARID_MASK},
    {"GLBL2_AXIMM_RD0_ARVALID", GLBL2_AXIMM_RD0_ARVALID_MASK},
    {"GLBL2_AXIMM_RD0_ARREADY", GLBL2_AXIMM_RD0_ARREADY_MASK},
    // MD : Debug: Track initialization of AXI MM read channel 0 debug field info
    printk(KERN_DEBUG "Initialized glbl2_dbg_aximm_rd0_field_info with %d entries\n", sizeof(glbl2_dbg_aximm_rd0_field_info)/sizeof(glbl2_dbg_aximm_rd0_field_info[0]));
};

// MD : Structure to hold register field information for AXI MM read channel 1 debug
static struct regfield_info glbl2_dbg_aximm_rd1_field_info[] = {
    {"GLBL2_AXIMM_RD1_RSVD_1", GLBL2_AXIMM_RD1_RSVD_1_MASK},
    {"GLBL2_AXIMM_RD1_RRSP_CNT4", GLBL2_AXIMM_RD1_RRSP_CNT4_MASK},
    {"GLBL2_AXIMM_RD1_RRSP_CNT3", GLBL2_AXIMM_RD1_RRSP_CNT3_MASK},
    {"GLBL2_AXIMM_RD1_RRSP_CNT2", GLBL2_AXIMM_RD1_RRSP_CNT2_MASK},
    {"GLBL2_AXIMM_RD1_RRSP_CNT1", GLBL2_AXIMM_RD1_RRSP_CNT1_MASK},
    {"GLBL2_AXIMM_RD1_RRSP_CNT0", GLBL2_AXIMM_RD1_RRSP_CNT0_MASK},
    // MD : Debug: Track initialization of AXI MM read channel 1 debug field info
    printk(KERN_DEBUG "Initialized glbl2_dbg_aximm_rd1_field_info with %d entries\n", sizeof(glbl2_dbg_aximm_rd1_field_info)/sizeof(glbl2_dbg_aximm_rd1_field_info[0]));
};

// MD : Structure to hold register field information for global ring size 1
static struct regfield_info glbl_rng_sz_1_field_info[] = {
    {"GLBL_RNG_SZ_1_RSVD_1", GLBL_RNG_SZ_1_RSVD_1_MASK},
    {"GLBL_RNG_SZ_1_RING_SIZE", GLBL_RNG_SZ_1_RING_SIZE_MASK},
    // MD : Debug: Track initialization of global ring size 1 field info
    printk(KERN_DEBUG "Initialized glbl_rng_sz_1_field_info with %d entries\n", sizeof(glbl_rng_sz_1_field_info)/sizeof(glbl_rng_sz_1_field_info[0]));
};

// MD : Structure to hold register field information for global ring size 2
static struct regfield_info glbl_rng_sz_2_field_info[] = {
    {"GLBL_RNG_SZ_2_RSVD_1", GLBL_RNG_SZ_2_RSVD_1_MASK},
    {"GLBL_RNG_SZ_2_RING_SIZE", GLBL_RNG_SZ_2_RING_SIZE_MASK},
    // MD : Debug: Track initialization of global ring size 2 field info
    printk(KERN_DEBUG "Initialized glbl_rng_sz_2_field_info with %d entries\n", sizeof(glbl_rng_sz_2_field_info)/sizeof(glbl_rng_sz_2_field_info[0]));
};

// MD : Structure to hold register field information for global ring size 3
static struct regfield_info glbl_rng_sz_3_field_info[] = {
    {"GLBL_RNG_SZ_3_RSVD_1", GLBL_RNG_SZ_3_RSVD_1_MASK},
    {"GLBL_RNG_SZ_3_RING_SIZE", GLBL_RNG_SZ_3_RING_SIZE_MASK},
    // MD : Debug: Track initialization of global ring size 3 field info
    printk(KERN_DEBUG "Initialized glbl_rng_sz_3_field_info with %d entries\n", sizeof(glbl_rng_sz_3_field_info)/sizeof(glbl_rng_sz_3_field_info[0]));
};

// MD : Structure to hold register field information for global ring size 4
static struct regfield_info glbl_rng_sz_4_field_info[] = {
    {"GLBL_RNG_SZ_4_RSVD_1", GLBL_RNG_SZ_4_RSVD_1_MASK},
    {"GLBL_RNG_SZ_4_RING_SIZE", GLBL_RNG_SZ_4_RING_SIZE_MASK},
    // MD : Debug: Track initialization of global ring size 4 field info
    printk(KERN_DEBUG "Initialized glbl_rng_sz_4_field_info with %d entries\n", sizeof(glbl_rng_sz_4_field_info)/sizeof(glbl_rng_sz_4_field_info[0]));
};

// MD : Structure to hold register field information for global ring size 5
static struct regfield_info glbl_rng_sz_5_field_info[] = {
    {"GLBL_RNG_SZ_5_RSVD_1", GLBL_RNG_SZ_5_RSVD_1_MASK},
    {"GLBL_RNG_SZ_5_RING_SIZE", GLBL_RNG_SZ_5_RING_SIZE_MASK},
    // MD : Debug: Track initialization of global ring size 5 field info
    printk(KERN_DEBUG "Initialized glbl_rng_sz_5_field_info with %d entries\n", sizeof(glbl_rng_sz_5_field_info)/sizeof(glbl_rng_sz_5_field_info[0]));
};

// MD : Structure to hold register field information for global ring size 6
static struct regfield_info glbl_rng_sz_6_field_info[] = {
    {"GLBL_RNG_SZ_6_RSVD_1", GLBL_RNG_SZ_6_RSVD_1_MASK},
    {"GLBL_RNG_SZ_6_RING_SIZE", GLBL_RNG_SZ_6_RING_SIZE_MASK},
    // MD : Debug: Track initialization of global ring size 6 field info
    printk(KERN_DEBUG "Initialized glbl_rng_sz_6_field_info with %d entries\n", sizeof(glbl_rng_sz_6_field_info)/sizeof(glbl_rng_sz_6_field_info[0]));
};

// MD : Structure to hold register field information for global ring size 7
static struct regfield_info glbl_rng_sz_7_field_info[] = {
    {"GLBL_RNG_SZ_7_RSVD_1", GLBL_RNG_SZ_7_RSVD_1_MASK},
    {"GLBL_RNG_SZ_7_RING_SIZE", GLBL_RNG_SZ_7_RING_SIZE_MASK},
    // MD : Debug: Track initialization of global ring size 7 field info
    printk(KERN_DEBUG "Initialized glbl_rng_sz_7_field_info with %d entries\n", sizeof(glbl_rng_sz_7_field_info)/sizeof(glbl_rng_sz_7_field_info[0]));
};

// MD : Structure to hold register field information for global ring size 8
static struct regfield_info glbl_rng_sz_8_field_info[] = {
    {"GLBL_RNG_SZ_8_RSVD_1", GLBL_RNG_SZ_8_RSVD_1_MASK},
    {"GLBL_RNG_SZ_8_RING_SIZE", GLBL_RNG_SZ_8_RING_SIZE_MASK},
    // MD : Debug: Track initialization of global ring size 8 field info
    printk(KERN_DEBUG "Initialized glbl_rng_sz_8_field_info with %d entries\n", sizeof(glbl_rng_sz_8_field_info)/sizeof(glbl_rng_sz_8_field_info[0]));
};

// MD : Structure to hold register field information for global ring size 9
static struct regfield_info glbl_rng_sz_9_field_info[] = {
    {"GLBL_RNG_SZ_9_RSVD_1", GLBL_RNG_SZ_9_RSVD_1_MASK},
    {"GLBL_RNG_SZ_9_RING_SIZE", GLBL_RNG_SZ_9_RING_SIZE_MASK},
    // MD : Debug: Track initialization of global ring size 9 field info
    printk(KERN_DEBUG "Initialized glbl_rng_sz_9_field_info with %d entries\n", sizeof(glbl_rng_sz_9_field_info)/sizeof(glbl_rng_sz_9_field_info[0]));
};

// MD : Structure to hold register field information for global ring size A
static struct regfield_info glbl_rng_sz_a_field_info[] = {
    {"GLBL_RNG_SZ_A_RSVD_1", GLBL_RNG_SZ_A_RSVD_1_MASK},
    {"GLBL_RNG_SZ_A_RING_SIZE", GLBL_RNG_SZ_A_RING_SIZE_MASK},
    // MD : Debug: Track initialization of global ring size A field info
    printk(KERN_DEBUG "Initialized glbl_rng_sz_a_field_info with %d entries\n", sizeof(glbl_rng_sz_a_field_info)/sizeof(glbl_rng_sz_a_field_info[0]));
};

// MD : Structure to hold register field information for global ring size B
static struct regfield_info glbl_rng_sz_b_field_info[] = {
    {"GLBL_RNG_SZ_B_RSVD_1", GLBL_RNG_SZ_B_RSVD_1_MASK},
    {"GLBL_RNG_SZ_B_RING_SIZE", GLBL_RNG_SZ_B_RING_SIZE_MASK},
    // MD : Debug: Track initialization of global ring size B field info
    printk(KERN_DEBUG "Initialized glbl_rng_sz_b_field_info with %d entries\n", sizeof(glbl_rng_sz_b_field_info)/sizeof(glbl_rng_sz_b_field_info[0]));
};

// MD : Structure to hold register field information for global ring size C
static struct regfield_info glbl_rng_sz_c_field_info[] = {
    {"GLBL_RNG_SZ_C_RSVD_1", GLBL_RNG_SZ_C_RSVD_1_MASK},
    {"GLBL_RNG_SZ_C_RING_SIZE", GLBL_RNG_SZ_C_RING_SIZE_MASK},
    // MD : Debug: Track initialization of global ring size C field info
    printk(KERN_DEBUG "Initialized glbl_rng_sz_c_field_info with %d entries\n", sizeof(glbl_rng_sz_c_field_info)/sizeof(glbl_rng_sz_c_field_info[0]));
};

// MD : Structure to hold register field information for global ring size D
static struct regfield_info glbl_rng_sz_d_field_info[] = {
    {"GLBL_RNG_SZ_D_RSVD_1", GLBL_RNG_SZ_D_RSVD_1_MASK},
    {"GLBL_RNG_SZ_D_RING_SIZE", GLBL_RNG_SZ_D_RING_SIZE_MASK},
    // MD : Debug: Track initialization of global ring size D field info
    printk(KERN_DEBUG "Initialized glbl_rng_sz_d_field_info with %d entries\n", sizeof(glbl_rng_sz_d_field_info)/sizeof(glbl_rng_sz_d_field_info[0]));
};

// MD : Structure to hold register field information for global ring size E
static struct regfield_info glbl_rng_sz_e_field_info[] = {
    {"GLBL_RNG_SZ_E_RSVD_1", GLBL_RNG_SZ_E_RSVD_1_MASK},
    {"GLBL_RNG_SZ_E_RING_SIZE", GLBL_RNG_SZ_E_RING_SIZE_MASK},
    // MD : Debug: Track initialization of global ring size E field info
    printk(KERN_DEBUG "Initialized glbl_rng_sz_e_field_info with %d entries\n", sizeof(glbl_rng_sz_e_field_info)/sizeof(glbl_rng_sz_e_field_info[0]));
};

// MD : Structure to hold register field information for global ring size F
static struct regfield_info glbl_rng_sz_f_field_info[] = {
    {"GLBL_RNG_SZ_F_RSVD_1", GLBL_RNG_SZ_F_RSVD_1_MASK},
    {"GLBL_RNG_SZ_F_RING_SIZE", GLBL_RNG_SZ_F_RING_SIZE_MASK},
    // MD : Debug: Track initialization of global ring size F field info
    printk(KERN_DEBUG "Initialized glbl_rng_sz_f_field_info with %d entries\n", sizeof(glbl_rng_sz_f_field_info)/sizeof(glbl_rng_sz_f_field_info[0]));
};

// MD : Structure to hold register field information for global ring size 10
static struct regfield_info glbl_rng_sz_10_field_info[] = {
    {"GLBL_RNG_SZ_10_RSVD_1", GLBL_RNG_SZ_10_RSVD_1_MASK},
    {"GLBL_RNG_SZ_10_RING_SIZE", GLBL_RNG_SZ_10_RING_SIZE_MASK},
    // MD : Debug: Track initialization of global ring size 10 field info
    printk(KERN_DEBUG "Initialized glbl_rng_sz_10_field_info with %d entries\n", sizeof(glbl_rng_sz_10_field_info)/sizeof(glbl_rng_sz_10_field_info[0]));
};

// MD : Structure to hold register field information for global error status
static struct regfield_info glbl_err_stat_field_info[] = {
    {"GLBL_ERR_STAT_RSVD_1", GLBL_ERR_STAT_RSVD_1_MASK},
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
    // MD : Debug: Track initialization of global error status field info
    printk(KERN_DEBUG "Initialized glbl_err_stat_field_info with %d entries\n", sizeof(glbl_err_stat_field_info)/sizeof(glbl_err_stat_field_info[0]));
};

// MD : Structure to hold register field information for global error mask
static struct regfield_info glbl_err_mask_field_info[] = {
    {"GLBL_ERR_RSVD_1", GLBL_ERR_RSVD_1_MASK},
    {"GLBL_ERR", GLBL_ERR_MASK},
    // MD : Debug: Track initialization of global error mask field info
    printk(KERN_DEBUG "Initialized glbl_err_mask_field_info with %d entries\n", sizeof(glbl_err_mask_field_info)/sizeof(glbl_err_mask_field_info[0]));
};

// MD : Structure to hold register field information for global descriptor configuration
static struct regfield_info glbl_dsc_cfg_field_info[] = {
    {"GLBL_DSC_CFG_RSVD_1", GLBL_DSC_CFG_RSVD_1_MASK},
    {"GLBL_DSC_CFG_UNC_OVR_COR", GLBL_DSC_CFG_UNC_OVR_COR_MASK},
    {"GLBL_DSC_CFG_CTXT_FER_DIS", GLBL_DSC_CFG_CTXT_FER_DIS_MASK},
    {"GLBL_DSC_CFG_RSVD_2", GLBL_DSC_CFG_RSVD_2_MASK},
    {"GLBL_DSC_CFG_MAXFETCH", GLBL_DSC_CFG_MAXFETCH_MASK},
    {"GLBL_DSC_CFG_WB_ACC_INT", GLBL_DSC_CFG_WB_ACC_INT_MASK},
    // MD : Debug: Track initialization of global descriptor configuration field info
    printk(KERN_DEBUG "Initialized glbl_dsc_cfg_field_info with %d entries\n", sizeof(glbl_dsc_cfg_field_info)/sizeof(glbl_dsc_cfg_field_info[0]));
};

// MD : Structure to hold register field information for global descriptor error status
static struct regfield_info glbl_dsc_err_sts_field_info[] = {
    {"GLBL_DSC_ERR_STS_RSVD_1", GLBL_DSC_ERR_STS_RSVD_1_MASK},
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
    {"GLBL_DSC_ERR_STS_UR_CA", GLBL_DSC_ERR_STS_UR_CA_MASK},
    {"GLBL_DSC_ERR_STS_POISON", GLBL_DSC_ERR_STS_POISON_MASK},
    // MD : Debug: Track initialization of global descriptor error status field info
    printk(KERN_DEBUG "Initialized glbl_dsc_err_sts_field_info with %d entries\n", sizeof(glbl_dsc_err_sts_field_info)/sizeof(glbl_dsc_err_sts_field_info[0]));
};

// MD : Structure to hold register field information for global descriptor error mask
static struct regfield_info glbl_dsc_err_msk_field_info[] = {
    {"GLBL_DSC_ERR_MSK", GLBL_DSC_ERR_MSK_MASK},
    // MD : Debug: Track initialization of global descriptor error mask field info
    printk(KERN_DEBUG "Initialized glbl_dsc_err_msk_field_info with %d entries\n", sizeof(glbl_dsc_err_msk_field_info)/sizeof(glbl_dsc_err_msk_field_info[0]));
};

// MD : Structure to hold register field information for global descriptor error log 0
static struct regfield_info glbl_dsc_err_log0_field_info[] = {
    {"GLBL_DSC_ERR_LOG0_VALID", GLBL_DSC_ERR_LOG0_VALID_MASK},
    {"GLBL_DSC_ERR_LOG0_RSVD_1", GLBL_DSC_ERR_LOG0_RSVD_1_MASK},
    {"GLBL_DSC_ERR_LOG0_QID", GLBL_DSC_ERR_LOG0_QID_MASK},
    {"GLBL_DSC_ERR_LOG0_SEL", GLBL_DSC_ERR_LOG0_SEL_MASK},
    {"GLBL_DSC_ERR_LOG0_CIDX", GLBL_DSC_ERR_LOG0_CIDX_MASK},
    // MD : Debug: Track initialization of global descriptor error log 0 field info
    printk(KERN_DEBUG "Initialized glbl_dsc_err_log0_field_info with %d entries\n", sizeof(glbl_dsc_err_log0_field_info)/sizeof(glbl_dsc_err_log0_field_info[0]));
};

// MD : Structure to hold register field information for global descriptor error log 1
static struct regfield_info glbl_dsc_err_log1_field_info[] = {
    {"GLBL_DSC_ERR_LOG1_RSVD_1", GLBL_DSC_ERR_LOG1_RSVD_1_MASK},
    {"GLBL_DSC_ERR_LOG1_SUB_TYPE", GLBL_DSC_ERR_LOG1_SUB_TYPE_MASK},
    {"GLBL_DSC_ERR_LOG1_ERR_TYPE", GLBL_DSC_ERR_LOG1_ERR_TYPE_MASK},
    // MD : Debug: Track initialization of global descriptor error log 1 field info
    printk(KERN_DEBUG "Initialized glbl_dsc_err_log1_field_info with %d entries\n", sizeof(glbl_dsc_err_log1_field_info)/sizeof(glbl_dsc_err_log1_field_info[0]));
};

// MD : Structure to hold register field information for TRQ error status
static struct regfield_info glbl_trq_err_sts_field_info[] = {
    {"GLBL_TRQ_ERR_STS_RSVD_1", GLBL_TRQ_ERR_STS_RSVD_1_MASK},
    {"GLBL_TRQ_ERR_STS_TCP_TIMEOUT", GLBL_TRQ_ERR_STS_TCP_TIMEOUT_MASK},
    {"GLBL_TRQ_ERR_STS_VF_ACCESS_ERR", GLBL_TRQ_ERR_STS_VF_ACCESS_ERR_MASK},
    {"GLBL_TRQ_ERR_STS_QID_RANGE", GLBL_TRQ_ERR_STS_QID_RANGE_MASK},
    {"GLBL_TRQ_ERR_STS_UNMAPPED", GLBL_TRQ_ERR_STS_UNMAPPED_MASK},
    // MD : Debug: Track initialization of TRQ error status field info
    printk(KERN_DEBUG "Initialized glbl_trq_err_sts_field_info with %d entries\n", sizeof(glbl_trq_err_sts_field_info)/sizeof(glbl_trq_err_sts_field_info[0]));
};

// MD : Structure to hold register field information for TRQ error mask
static struct regfield_info glbl_trq_err_msk_field_info[] = {
    {"GLBL_TRQ_ERR_MSK", GLBL_TRQ_ERR_MSK_MASK},
    // MD : Debug: Track initialization of TRQ error mask field info
    printk(KERN_DEBUG "Initialized glbl_trq_err_msk_field_info with %d entries\n", sizeof(glbl_trq_err_msk_field_info)/sizeof(glbl_trq_err_msk_field_info[0]));
};

// MD : Structure to hold register field information for TRQ error log
static struct regfield_info glbl_trq_err_log_field_info[] = {
    {"GLBL_TRQ_ERR_LOG_RSVD_1", GLBL_TRQ_ERR_LOG_RSVD_1_MASK},
    {"GLBL_TRQ_ERR_LOG_TARGET", GLBL_TRQ_ERR_LOG_TARGET_MASK},
    {"GLBL_TRQ_ERR_LOG_FUNC", GLBL_TRQ_ERR_LOG_FUNC_MASK},
    {"GLBL_TRQ_ERR_LOG_ADDRESS", GLBL_TRQ_ERR_LOG_ADDRESS_MASK},
    // MD : Debug: Track initialization of TRQ error log field info
    printk(KERN_DEBUG "Initialized glbl_trq_err_log_field_info with %d entries\n", sizeof(glbl_trq_err_log_field_info)/sizeof(glbl_trq_err_log_field_info[0]));
};

// MD : Structure to hold register field information for DSC debug data 0
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
    // MD : Debug: Track initialization of DSC debug data 0 field info
    printk(KERN_DEBUG "Initialized glbl_dsc_dbg_dat0_field_info with %d entries\n", sizeof(glbl_dsc_dbg_dat0_field_info)/sizeof(glbl_dsc_dbg_dat0_field_info[0]));
};

// MD : Structure to hold register field information for DSC debug data 1
static struct regfield_info glbl_dsc_dbg_dat1_field_info[] = {
    {"GLBL_DSC_DAT1_RSVD_1", GLBL_DSC_DAT1_RSVD_1_MASK},
    {"GLBL_DSC_DAT1_EVT_SPC_C2H", GLBL_DSC_DAT1_EVT_SPC_C2H_MASK},
    {"GLBL_DSC_DAT1_EVT_SP_H2C", GLBL_DSC_DAT1_EVT_SP_H2C_MASK},
    {"GLBL_DSC_DAT1_DSC_SPC_C2H", GLBL_DSC_DAT1_DSC_SPC_C2H_MASK},
    {"GLBL_DSC_DAT1_DSC_SPC_H2C", GLBL_DSC_DAT1_DSC_SPC_H2C_MASK},
    // MD : Debug: Track initialization of DSC debug data 1 field info
    printk(KERN_DEBUG "Initialized glbl_dsc_dbg_dat1_field_info with %d entries\n", sizeof(glbl_dsc_dbg_dat1_field_info)/sizeof(glbl_dsc_dbg_dat1_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 0
static struct regfield_info trq_sel_fmap_0_field_info[] = {
    {"TRQ_SEL_FMAP_0_RSVD_1", TRQ_SEL_FMAP_0_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_0_QID_MAX", TRQ_SEL_FMAP_0_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_0_QID_BASE", TRQ_SEL_FMAP_0_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 0 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_0_field_info with %d entries\n", sizeof(trq_sel_fmap_0_field_info)/sizeof(trq_sel_fmap_0_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 1
static struct regfield_info trq_sel_fmap_1_field_info[] = {
    {"TRQ_SEL_FMAP_1_RSVD_1", TRQ_SEL_FMAP_1_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_1_QID_MAX", TRQ_SEL_FMAP_1_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_1_QID_BASE", TRQ_SEL_FMAP_1_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 1 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_1_field_info with %d entries\n", sizeof(trq_sel_fmap_1_field_info)/sizeof(trq_sel_fmap_1_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 2
static struct regfield_info trq_sel_fmap_2_field_info[] = {
    {"TRQ_SEL_FMAP_2_RSVD_1", TRQ_SEL_FMAP_2_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_2_QID_MAX", TRQ_SEL_FMAP_2_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_2_QID_BASE", TRQ_SEL_FMAP_2_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 2 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_2_field_info with %d entries\n", sizeof(trq_sel_fmap_2_field_info)/sizeof(trq_sel_fmap_2_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 3
static struct regfield_info trq_sel_fmap_3_field_info[] = {
    {"TRQ_SEL_FMAP_3_RSVD_1", TRQ_SEL_FMAP_3_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_3_QID_MAX", TRQ_SEL_FMAP_3_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_3_QID_BASE", TRQ_SEL_FMAP_3_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 3 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_3_field_info with %d entries\n", sizeof(trq_sel_fmap_3_field_info)/sizeof(trq_sel_fmap_3_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 4
static struct regfield_info trq_sel_fmap_4_field_info[] = {
    {"TRQ_SEL_FMAP_4_RSVD_1", TRQ_SEL_FMAP_4_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_4_QID_MAX", TRQ_SEL_FMAP_4_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_4_QID_BASE", TRQ_SEL_FMAP_4_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 4 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_4_field_info with %d entries\n", sizeof(trq_sel_fmap_4_field_info)/sizeof(trq_sel_fmap_4_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 5
static struct regfield_info trq_sel_fmap_5_field_info[] = {
    {"TRQ_SEL_FMAP_5_RSVD_1", TRQ_SEL_FMAP_5_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_5_QID_MAX", TRQ_SEL_FMAP_5_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_5_QID_BASE", TRQ_SEL_FMAP_5_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 5 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_5_field_info with %d entries\n", sizeof(trq_sel_fmap_5_field_info)/sizeof(trq_sel_fmap_5_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 6
static struct regfield_info trq_sel_fmap_6_field_info[] = {
    {"TRQ_SEL_FMAP_6_RSVD_1", TRQ_SEL_FMAP_6_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_6_QID_MAX", TRQ_SEL_FMAP_6_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_6_QID_BASE", TRQ_SEL_FMAP_6_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 6 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_6_field_info with %d entries\n", sizeof(trq_sel_fmap_6_field_info)/sizeof(trq_sel_fmap_6_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 7
static struct regfield_info trq_sel_fmap_7_field_info[] = {
    {"TRQ_SEL_FMAP_7_RSVD_1", TRQ_SEL_FMAP_7_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_7_QID_MAX", TRQ_SEL_FMAP_7_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_7_QID_BASE", TRQ_SEL_FMAP_7_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 7 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_7_field_info with %d entries\n", sizeof(trq_sel_fmap_7_field_info)/sizeof(trq_sel_fmap_7_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 8
static struct regfield_info trq_sel_fmap_8_field_info[] = {
    {"TRQ_SEL_FMAP_8_RSVD_1", TRQ_SEL_FMAP_8_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_8_QID_MAX", TRQ_SEL_FMAP_8_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_8_QID_BASE", TRQ_SEL_FMAP_8_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 8 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_8_field_info with %d entries\n", sizeof(trq_sel_fmap_8_field_info)/sizeof(trq_sel_fmap_8_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 9
static struct regfield_info trq_sel_fmap_9_field_info[] = {
    {"TRQ_SEL_FMAP_9_RSVD_1", TRQ_SEL_FMAP_9_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_9_QID_MAX", TRQ_SEL_FMAP_9_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_9_QID_BASE", TRQ_SEL_FMAP_9_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 9 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_9_field_info with %d entries\n", sizeof(trq_sel_fmap_9_field_info)/sizeof(trq_sel_fmap_9_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap A
static struct regfield_info trq_sel_fmap_a_field_info[] = {
    {"TRQ_SEL_FMAP_A_RSVD_1", TRQ_SEL_FMAP_A_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_A_QID_MAX", TRQ_SEL_FMAP_A_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_A_QID_BASE", TRQ_SEL_FMAP_A_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap A field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_a_field_info with %d entries\n", sizeof(trq_sel_fmap_a_field_info)/sizeof(trq_sel_fmap_a_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap B
static struct regfield_info trq_sel_fmap_b_field_info[] = {
    {"TRQ_SEL_FMAP_B_RSVD_1", TRQ_SEL_FMAP_B_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_B_QID_MAX", TRQ_SEL_FMAP_B_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_B_QID_BASE", TRQ_SEL_FMAP_B_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap B field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_b_field_info with %d entries\n", sizeof(trq_sel_fmap_b_field_info)/sizeof(trq_sel_fmap_b_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap D
static struct regfield_info trq_sel_fmap_d_field_info[] = {
    {"TRQ_SEL_FMAP_D_RSVD_1", TRQ_SEL_FMAP_D_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_D_QID_MAX", TRQ_SEL_FMAP_D_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_D_QID_BASE", TRQ_SEL_FMAP_D_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap D field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_d_field_info with %d entries\n", sizeof(trq_sel_fmap_d_field_info)/sizeof(trq_sel_fmap_d_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap E
static struct regfield_info trq_sel_fmap_e_field_info[] = {
    {"TRQ_SEL_FMAP_E_RSVD_1", TRQ_SEL_FMAP_E_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_E_QID_MAX", TRQ_SEL_FMAP_E_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_E_QID_BASE", TRQ_SEL_FMAP_E_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap E field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_e_field_info with %d entries\n", sizeof(trq_sel_fmap_e_field_info)/sizeof(trq_sel_fmap_e_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap F
static struct regfield_info trq_sel_fmap_f_field_info[] = {
    {"TRQ_SEL_FMAP_F_RSVD_1", TRQ_SEL_FMAP_F_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_F_QID_MAX", TRQ_SEL_FMAP_F_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_F_QID_BASE", TRQ_SEL_FMAP_F_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap F field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_f_field_info with %d entries\n", sizeof(trq_sel_fmap_f_field_info)/sizeof(trq_sel_fmap_f_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 10
static struct regfield_info trq_sel_fmap_10_field_info[] = {
    {"TRQ_SEL_FMAP_10_RSVD_1", TRQ_SEL_FMAP_10_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_10_QID_MAX", TRQ_SEL_FMAP_10_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_10_QID_BASE", TRQ_SEL_FMAP_10_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 10 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_10_field_info with %d entries\n", sizeof(trq_sel_fmap_10_field_info)/sizeof(trq_sel_fmap_10_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 11
static struct regfield_info trq_sel_fmap_11_field_info[] = {
    {"TRQ_SEL_FMAP_11_RSVD_1", TRQ_SEL_FMAP_11_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_11_QID_MAX", TRQ_SEL_FMAP_11_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_11_QID_BASE", TRQ_SEL_FMAP_11_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 11 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_11_field_info with %d entries\n", sizeof(trq_sel_fmap_11_field_info)/sizeof(trq_sel_fmap_11_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 12
static struct regfield_info trq_sel_fmap_12_field_info[] = {
    {"TRQ_SEL_FMAP_12_RSVD_1", TRQ_SEL_FMAP_12_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_12_QID_MAX", TRQ_SEL_FMAP_12_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_12_QID_BASE", TRQ_SEL_FMAP_12_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 12 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_12_field_info with %d entries\n", sizeof(trq_sel_fmap_12_field_info)/sizeof(trq_sel_fmap_12_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 13
static struct regfield_info trq_sel_fmap_13_field_info[] = {
    {"TRQ_SEL_FMAP_13_RSVD_1", TRQ_SEL_FMAP_13_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_13_QID_MAX", TRQ_SEL_FMAP_13_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_13_QID_BASE", TRQ_SEL_FMAP_13_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 13 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_13_field_info with %d entries\n", sizeof(trq_sel_fmap_13_field_info)/sizeof(trq_sel_fmap_13_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 14
static struct regfield_info trq_sel_fmap_14_field_info[] = {
    {"TRQ_SEL_FMAP_14_RSVD_1", TRQ_SEL_FMAP_14_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_14_QID_MAX", TRQ_SEL_FMAP_14_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_14_QID_BASE", TRQ_SEL_FMAP_14_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 14 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_14_field_info with %d entries\n", sizeof(trq_sel_fmap_14_field_info)/sizeof(trq_sel_fmap_14_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 15
static struct regfield_info trq_sel_fmap_15_field_info[] = {
    {"TRQ_SEL_FMAP_15_RSVD_1", TRQ_SEL_FMAP_15_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_15_QID_MAX", TRQ_SEL_FMAP_15_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_15_QID_BASE", TRQ_SEL_FMAP_15_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 15 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_15_field_info with %d entries\n", sizeof(trq_sel_fmap_15_field_info)/sizeof(trq_sel_fmap_15_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 16
static struct regfield_info trq_sel_fmap_16_field_info[] = {
    {"TRQ_SEL_FMAP_16_RSVD_1", TRQ_SEL_FMAP_16_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_16_QID_MAX", TRQ_SEL_FMAP_16_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_16_QID_BASE", TRQ_SEL_FMAP_16_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 16 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_16_field_info with %d entries\n", sizeof(trq_sel_fmap_16_field_info)/sizeof(trq_sel_fmap_16_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 17
static struct regfield_info trq_sel_fmap_17_field_info[] = {
    {"TRQ_SEL_FMAP_17_RSVD_1", TRQ_SEL_FMAP_17_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_17_QID_MAX", TRQ_SEL_FMAP_17_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_17_QID_BASE", TRQ_SEL_FMAP_17_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 17 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_17_field_info with %d entries\n", sizeof(trq_sel_fmap_17_field_info)/sizeof(trq_sel_fmap_17_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 18
static struct regfield_info trq_sel_fmap_18_field_info[] = {
    {"TRQ_SEL_FMAP_18_RSVD_1", TRQ_SEL_FMAP_18_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_18_QID_MAX", TRQ_SEL_FMAP_18_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_18_QID_BASE", TRQ_SEL_FMAP_18_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 18 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_18_field_info with %d entries\n", sizeof(trq_sel_fmap_18_field_info)/sizeof(trq_sel_fmap_18_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 19
static struct regfield_info trq_sel_fmap_19_field_info[] = {
    {"TRQ_SEL_FMAP_19_RSVD_1", TRQ_SEL_FMAP_19_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_19_QID_MAX", TRQ_SEL_FMAP_19_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_19_QID_BASE", TRQ_SEL_FMAP_19_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 19 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_19_field_info with %d entries\n", sizeof(trq_sel_fmap_19_field_info)/sizeof(trq_sel_fmap_19_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 1A
static struct regfield_info trq_sel_fmap_1a_field_info[] = {
    {"TRQ_SEL_FMAP_1A_RSVD_1", TRQ_SEL_FMAP_1A_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_1A_QID_MAX", TRQ_SEL_FMAP_1A_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_1A_QID_BASE", TRQ_SEL_FMAP_1A_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 1A field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_1a_field_info with %d entries\n", sizeof(trq_sel_fmap_1a_field_info)/sizeof(trq_sel_fmap_1a_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 1B
static struct regfield_info trq_sel_fmap_1b_field_info[] = {
    {"TRQ_SEL_FMAP_1B_RSVD_1", TRQ_SEL_FMAP_1B_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_1B_QID_MAX", TRQ_SEL_FMAP_1B_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_1B_QID_BASE", TRQ_SEL_FMAP_1B_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 1B field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_1b_field_info with %d entries\n", sizeof(trq_sel_fmap_1b_field_info)/sizeof(trq_sel_fmap_1b_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 1C
static struct regfield_info trq_sel_fmap_1c_field_info[] = {
    {"TRQ_SEL_FMAP_1C_RSVD_1", TRQ_SEL_FMAP_1C_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_1C_QID_MAX", TRQ_SEL_FMAP_1C_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_1C_QID_BASE", TRQ_SEL_FMAP_1C_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 1C field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_1c_field_info with %d entries\n", sizeof(trq_sel_fmap_1c_field_info)/sizeof(trq_sel_fmap_1c_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 1D
static struct regfield_info trq_sel_fmap_1d_field_info[] = {
    {"TRQ_SEL_FMAP_1D_RSVD_1", TRQ_SEL_FMAP_1D_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_1D_QID_MAX", TRQ_SEL_FMAP_1D_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_1D_QID_BASE", TRQ_SEL_FMAP_1D_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 1D field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_1d_field_info with %d entries\n", sizeof(trq_sel_fmap_1d_field_info)/sizeof(trq_sel_fmap_1d_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 1E
static struct regfield_info trq_sel_fmap_1e_field_info[] = {
    {"TRQ_SEL_FMAP_1E_RSVD_1", TRQ_SEL_FMAP_1E_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_1E_QID_MAX", TRQ_SEL_FMAP_1E_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_1E_QID_BASE", TRQ_SEL_FMAP_1E_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 1E field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_1e_field_info with %d entries\n", sizeof(trq_sel_fmap_1e_field_info)/sizeof(trq_sel_fmap_1e_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 1F
static struct regfield_info trq_sel_fmap_1f_field_info[] = {
    {"TRQ_SEL_FMAP_1F_RSVD_1", TRQ_SEL_FMAP_1F_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_1F_QID_MAX", TRQ_SEL_FMAP_1F_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_1F_QID_BASE", TRQ_SEL_FMAP_1F_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 1F field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_1f_field_info with %d entries\n", sizeof(trq_sel_fmap_1f_field_info)/sizeof(trq_sel_fmap_1f_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 20
static struct regfield_info trq_sel_fmap_20_field_info[] = {
    {"TRQ_SEL_FMAP_20_RSVD_1", TRQ_SEL_FMAP_20_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_20_QID_MAX", TRQ_SEL_FMAP_20_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_20_QID_BASE", TRQ_SEL_FMAP_20_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 20 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_20_field_info with %d entries\n", sizeof(trq_sel_fmap_20_field_info)/sizeof(trq_sel_fmap_20_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 21
static struct regfield_info trq_sel_fmap_21_field_info[] = {
    {"TRQ_SEL_FMAP_21_RSVD_1", TRQ_SEL_FMAP_21_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_21_QID_MAX", TRQ_SEL_FMAP_21_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_21_QID_BASE", TRQ_SEL_FMAP_21_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 21 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_21_field_info with %d entries\n", sizeof(trq_sel_fmap_21_field_info)/sizeof(trq_sel_fmap_21_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 22
static struct regfield_info trq_sel_fmap_22_field_info[] = {
    {"TRQ_SEL_FMAP_22_RSVD_1", TRQ_SEL_FMAP_22_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_22_QID_MAX", TRQ_SEL_FMAP_22_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_22_QID_BASE", TRQ_SEL_FMAP_22_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 22 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_22_field_info with %d entries\n", sizeof(trq_sel_fmap_22_field_info)/sizeof(trq_sel_fmap_22_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 23
static struct regfield_info trq_sel_fmap_23_field_info[] = {
    {"TRQ_SEL_FMAP_23_RSVD_1", TRQ_SEL_FMAP_23_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_23_QID_MAX", TRQ_SEL_FMAP_23_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_23_QID_BASE", TRQ_SEL_FMAP_23_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 23 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_23_field_info with %d entries\n", sizeof(trq_sel_fmap_23_field_info)/sizeof(trq_sel_fmap_23_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 24
static struct regfield_info trq_sel_fmap_24_field_info[] = {
    {"TRQ_SEL_FMAP_24_RSVD_1", TRQ_SEL_FMAP_24_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_24_QID_MAX", TRQ_SEL_FMAP_24_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_24_QID_BASE", TRQ_SEL_FMAP_24_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 24 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_24_field_info with %d entries\n", sizeof(trq_sel_fmap_24_field_info)/sizeof(trq_sel_fmap_24_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 25
static struct regfield_info trq_sel_fmap_25_field_info[] = {
    {"TRQ_SEL_FMAP_25_RSVD_1", TRQ_SEL_FMAP_25_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_25_QID_MAX", TRQ_SEL_FMAP_25_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_25_QID_BASE", TRQ_SEL_FMAP_25_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 25 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_25_field_info with %d entries\n", sizeof(trq_sel_fmap_25_field_info)/sizeof(trq_sel_fmap_25_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 26
static struct regfield_info trq_sel_fmap_26_field_info[] = {
    {"TRQ_SEL_FMAP_26_RSVD_1", TRQ_SEL_FMAP_26_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_26_QID_MAX", TRQ_SEL_FMAP_26_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_26_QID_BASE", TRQ_SEL_FMAP_26_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 26 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_26_field_info with %d entries\n", sizeof(trq_sel_fmap_26_field_info)/sizeof(trq_sel_fmap_26_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 27
static struct regfield_info trq_sel_fmap_27_field_info[] = {
    {"TRQ_SEL_FMAP_27_RSVD_1", TRQ_SEL_FMAP_27_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_27_QID_MAX", TRQ_SEL_FMAP_27_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_27_QID_BASE", TRQ_SEL_FMAP_27_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 27 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_27_field_info with %d entries\n", sizeof(trq_sel_fmap_27_field_info)/sizeof(trq_sel_fmap_27_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 28
static struct regfield_info trq_sel_fmap_28_field_info[] = {
    {"TRQ_SEL_FMAP_28_RSVD_1", TRQ_SEL_FMAP_28_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_28_QID_MAX", TRQ_SEL_FMAP_28_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_28_QID_BASE", TRQ_SEL_FMAP_28_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 28 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_28_field_info with %d entries\n", sizeof(trq_sel_fmap_28_field_info)/sizeof(trq_sel_fmap_28_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 29
static struct regfield_info trq_sel_fmap_29_field_info[] = {
    {"TRQ_SEL_FMAP_29_RSVD_1", TRQ_SEL_FMAP_29_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_29_QID_MAX", TRQ_SEL_FMAP_29_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_29_QID_BASE", TRQ_SEL_FMAP_29_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 29 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_29_field_info with %d entries\n", sizeof(trq_sel_fmap_29_field_info)/sizeof(trq_sel_fmap_29_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 2A
static struct regfield_info trq_sel_fmap_2a_field_info[] = {
    {"TRQ_SEL_FMAP_2A_RSVD_1", TRQ_SEL_FMAP_2A_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_2A_QID_MAX", TRQ_SEL_FMAP_2A_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_2A_QID_BASE", TRQ_SEL_FMAP_2A_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 2A field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_2a_field_info with %d entries\n", sizeof(trq_sel_fmap_2a_field_info)/sizeof(trq_sel_fmap_2a_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 2B
static struct regfield_info trq_sel_fmap_2b_field_info[] = {
    {"TRQ_SEL_FMAP_2B_RSVD_1", TRQ_SEL_FMAP_2B_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_2B_QID_MAX", TRQ_SEL_FMAP_2B_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_2B_QID_BASE", TRQ_SEL_FMAP_2B_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 2B field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_2b_field_info with %d entries\n", sizeof(trq_sel_fmap_2b_field_info)/sizeof(trq_sel_fmap_2b_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 2C
static struct regfield_info trq_sel_fmap_2c_field_info[] = {
    {"TRQ_SEL_FMAP_2C_RSVD_1", TRQ_SEL_FMAP_2C_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_2C_QID_MAX", TRQ_SEL_FMAP_2C_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_2C_QID_BASE", TRQ_SEL_FMAP_2C_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 2C field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_2c_field_info with %d entries\n", sizeof(trq_sel_fmap_2c_field_info)/sizeof(trq_sel_fmap_2c_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 2D
static struct regfield_info trq_sel_fmap_2d_field_info[] = {
    {"TRQ_SEL_FMAP_2D_RSVD_1", TRQ_SEL_FMAP_2D_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_2D_QID_MAX", TRQ_SEL_FMAP_2D_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_2D_QID_BASE", TRQ_SEL_FMAP_2D_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 2D field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_2d_field_info with %d entries\n", sizeof(trq_sel_fmap_2d_field_info)/sizeof(trq_sel_fmap_2d_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 2E
static struct regfield_info trq_sel_fmap_2e_field_info[] = {
    {"TRQ_SEL_FMAP_2E_RSVD_1", TRQ_SEL_FMAP_2E_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_2E_QID_MAX", TRQ_SEL_FMAP_2E_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_2E_QID_BASE", TRQ_SEL_FMAP_2E_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 2E field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_2e_field_info with %d entries\n", sizeof(trq_sel_fmap_2e_field_info)/sizeof(trq_sel_fmap_2e_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 2F
static struct regfield_info trq_sel_fmap_2f_field_info[] = {
    {"TRQ_SEL_FMAP_2F_RSVD_1", TRQ_SEL_FMAP_2F_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_2F_QID_MAX", TRQ_SEL_FMAP_2F_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_2F_QID_BASE", TRQ_SEL_FMAP_2F_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 2F field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_2f_field_info with %d entries\n", sizeof(trq_sel_fmap_2f_field_info)/sizeof(trq_sel_fmap_2f_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 30
static struct regfield_info trq_sel_fmap_30_field_info[] = {
    {"TRQ_SEL_FMAP_30_RSVD_1", TRQ_SEL_FMAP_30_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_30_QID_MAX", TRQ_SEL_FMAP_30_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_30_QID_BASE", TRQ_SEL_FMAP_30_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 30 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_30_field_info with %d entries\n", sizeof(trq_sel_fmap_30_field_info)/sizeof(trq_sel_fmap_30_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 31
static struct regfield_info trq_sel_fmap_31_field_info[] = {
    {"TRQ_SEL_FMAP_31_RSVD_1", TRQ_SEL_FMAP_31_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_31_QID_MAX", TRQ_SEL_FMAP_31_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_31_QID_BASE", TRQ_SEL_FMAP_31_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 31 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_31_field_info with %d entries\n", sizeof(trq_sel_fmap_31_field_info)/sizeof(trq_sel_fmap_31_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 32
static struct regfield_info trq_sel_fmap_32_field_info[] = {
    {"TRQ_SEL_FMAP_32_RSVD_1", TRQ_SEL_FMAP_32_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_32_QID_MAX", TRQ_SEL_FMAP_32_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_32_QID_BASE", TRQ_SEL_FMAP_32_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 32 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_32_field_info with %d entries\n", sizeof(trq_sel_fmap_32_field_info)/sizeof(trq_sel_fmap_32_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 33
static struct regfield_info trq_sel_fmap_33_field_info[] = {
    {"TRQ_SEL_FMAP_33_RSVD_1", TRQ_SEL_FMAP_33_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_33_QID_MAX", TRQ_SEL_FMAP_33_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_33_QID_BASE", TRQ_SEL_FMAP_33_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 33 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_33_field_info with %d entries\n", sizeof(trq_sel_fmap_33_field_info)/sizeof(trq_sel_fmap_33_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 34
static struct regfield_info trq_sel_fmap_34_field_info[] = {
    {"TRQ_SEL_FMAP_34_RSVD_1", TRQ_SEL_FMAP_34_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_34_QID_MAX", TRQ_SEL_FMAP_34_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_34_QID_BASE", TRQ_SEL_FMAP_34_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 34 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_34_field_info with %d entries\n", sizeof(trq_sel_fmap_34_field_info)/sizeof(trq_sel_fmap_34_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 35
static struct regfield_info trq_sel_fmap_35_field_info[] = {
    {"TRQ_SEL_FMAP_35_RSVD_1", TRQ_SEL_FMAP_35_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_35_QID_MAX", TRQ_SEL_FMAP_35_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_35_QID_BASE", TRQ_SEL_FMAP_35_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 35 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_35_field_info with %d entries\n", sizeof(trq_sel_fmap_35_field_info)/sizeof(trq_sel_fmap_35_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 36
static struct regfield_info trq_sel_fmap_36_field_info[] = {
    {"TRQ_SEL_FMAP_36_RSVD_1", TRQ_SEL_FMAP_36_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_36_QID_MAX", TRQ_SEL_FMAP_36_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_36_QID_BASE", TRQ_SEL_FMAP_36_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 36 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_36_field_info with %d entries\n", sizeof(trq_sel_fmap_36_field_info)/sizeof(trq_sel_fmap_36_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 37
static struct regfield_info trq_sel_fmap_37_field_info[] = {
    {"TRQ_SEL_FMAP_37_RSVD_1", TRQ_SEL_FMAP_37_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_37_QID_MAX", TRQ_SEL_FMAP_37_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_37_QID_BASE", TRQ_SEL_FMAP_37_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 37 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_37_field_info with %d entries\n", sizeof(trq_sel_fmap_37_field_info)/sizeof(trq_sel_fmap_37_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 38
static struct regfield_info trq_sel_fmap_38_field_info[] = {
    {"TRQ_SEL_FMAP_38_RSVD_1", TRQ_SEL_FMAP_38_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_38_QID_MAX", TRQ_SEL_FMAP_38_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_38_QID_BASE", TRQ_SEL_FMAP_38_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 38 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_38_field_info with %d entries\n", sizeof(trq_sel_fmap_38_field_info)/sizeof(trq_sel_fmap_38_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 39
static struct regfield_info trq_sel_fmap_39_field_info[] = {
    {"TRQ_SEL_FMAP_39_RSVD_1", TRQ_SEL_FMAP_39_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_39_QID_MAX", TRQ_SEL_FMAP_39_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_39_QID_BASE", TRQ_SEL_FMAP_39_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 39 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_39_field_info with %d entries\n", sizeof(trq_sel_fmap_39_field_info)/sizeof(trq_sel_fmap_39_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 3A
static struct regfield_info trq_sel_fmap_3a_field_info[] = {
    {"TRQ_SEL_FMAP_3A_RSVD_1", TRQ_SEL_FMAP_3A_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_3A_QID_MAX", TRQ_SEL_FMAP_3A_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_3A_QID_BASE", TRQ_SEL_FMAP_3A_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 3A field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_3a_field_info with %d entries\n", sizeof(trq_sel_fmap_3a_field_info)/sizeof(trq_sel_fmap_3a_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 3B
static struct regfield_info trq_sel_fmap_3b_field_info[] = {
    {"TRQ_SEL_FMAP_3B_RSVD_1", TRQ_SEL_FMAP_3B_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_3B_QID_MAX", TRQ_SEL_FMAP_3B_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_3B_QID_BASE", TRQ_SEL_FMAP_3B_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 3B field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_3b_field_info with %d entries\n", sizeof(trq_sel_fmap_3b_field_info)/sizeof(trq_sel_fmap_3b_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 3C
static struct regfield_info trq_sel_fmap_3c_field_info[] = {
    {"TRQ_SEL_FMAP_3C_RSVD_1", TRQ_SEL_FMAP_3C_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_3C_QID_MAX", TRQ_SEL_FMAP_3C_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_3C_QID_BASE", TRQ_SEL_FMAP_3C_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 3C field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_3c_field_info with %d entries\n", sizeof(trq_sel_fmap_3c_field_info)/sizeof(trq_sel_fmap_3c_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 3D
static struct regfield_info trq_sel_fmap_3d_field_info[] = {
    {"TRQ_SEL_FMAP_3D_RSVD_1", TRQ_SEL_FMAP_3D_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_3D_QID_MAX", TRQ_SEL_FMAP_3D_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_3D_QID_BASE", TRQ_SEL_FMAP_3D_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 3D field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_3d_field_info with %d entries\n", sizeof(trq_sel_fmap_3d_field_info)/sizeof(trq_sel_fmap_3d_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 3E
static struct regfield_info trq_sel_fmap_3e_field_info[] = {
    {"TRQ_SEL_FMAP_3E_RSVD_1", TRQ_SEL_FMAP_3E_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_3E_QID_MAX", TRQ_SEL_FMAP_3E_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_3E_QID_BASE", TRQ_SEL_FMAP_3E_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 3E field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_3e_field_info with %d entries\n", sizeof(trq_sel_fmap_3e_field_info)/sizeof(trq_sel_fmap_3e_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 3F
static struct regfield_info trq_sel_fmap_3f_field_info[] = {
    {"TRQ_SEL_FMAP_3F_RSVD_1", TRQ_SEL_FMAP_3F_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_3F_QID_MAX", TRQ_SEL_FMAP_3F_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_3F_QID_BASE", TRQ_SEL_FMAP_3F_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 3F field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_3f_field_info with %d entries\n", sizeof(trq_sel_fmap_3f_field_info)/sizeof(trq_sel_fmap_3f_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 40
static struct regfield_info trq_sel_fmap_40_field_info[] = {
    {"TRQ_SEL_FMAP_40_RSVD_1", TRQ_SEL_FMAP_40_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_40_QID_MAX", TRQ_SEL_FMAP_40_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_40_QID_BASE", TRQ_SEL_FMAP_40_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 40 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_40_field_info with %d entries\n", sizeof(trq_sel_fmap_40_field_info)/sizeof(trq_sel_fmap_40_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 41
static struct regfield_info trq_sel_fmap_41_field_info[] = {
    {"TRQ_SEL_FMAP_41_RSVD_1", TRQ_SEL_FMAP_41_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_41_QID_MAX", TRQ_SEL_FMAP_41_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_41_QID_BASE", TRQ_SEL_FMAP_41_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 41 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_41_field_info with %d entries\n", sizeof(trq_sel_fmap_41_field_info)/sizeof(trq_sel_fmap_41_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 42
static struct regfield_info trq_sel_fmap_42_field_info[] = {
    {"TRQ_SEL_FMAP_42_RSVD_1", TRQ_SEL_FMAP_42_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_42_QID_MAX", TRQ_SEL_FMAP_42_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_42_QID_BASE", TRQ_SEL_FMAP_42_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 42 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_42_field_info with %d entries\n", sizeof(trq_sel_fmap_42_field_info)/sizeof(trq_sel_fmap_42_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 43
static struct regfield_info trq_sel_fmap_43_field_info[] = {
    {"TRQ_SEL_FMAP_43_RSVD_1", TRQ_SEL_FMAP_43_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_43_QID_MAX", TRQ_SEL_FMAP_43_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_43_QID_BASE", TRQ_SEL_FMAP_43_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 43 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_43_field_info with %d entries\n", sizeof(trq_sel_fmap_43_field_info)/sizeof(trq_sel_fmap_43_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 44
static struct regfield_info trq_sel_fmap_44_field_info[] = {
    {"TRQ_SEL_FMAP_44_RSVD_1", TRQ_SEL_FMAP_44_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_44_QID_MAX", TRQ_SEL_FMAP_44_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_44_QID_BASE", TRQ_SEL_FMAP_44_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 44 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_44_field_info with %d entries\n", sizeof(trq_sel_fmap_44_field_info)/sizeof(trq_sel_fmap_44_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 45
static struct regfield_info trq_sel_fmap_45_field_info[] = {
    {"TRQ_SEL_FMAP_45_RSVD_1", TRQ_SEL_FMAP_45_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_45_QID_MAX", TRQ_SEL_FMAP_45_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_45_QID_BASE", TRQ_SEL_FMAP_45_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 45 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_45_field_info with %d entries\n", sizeof(trq_sel_fmap_45_field_info)/sizeof(trq_sel_fmap_45_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 46
static struct regfield_info trq_sel_fmap_46_field_info[] = {
    {"TRQ_SEL_FMAP_46_RSVD_1", TRQ_SEL_FMAP_46_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_46_QID_MAX", TRQ_SEL_FMAP_46_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_46_QID_BASE", TRQ_SEL_FMAP_46_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 46 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_46_field_info with %d entries\n", sizeof(trq_sel_fmap_46_field_info)/sizeof(trq_sel_fmap_46_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 47
static struct regfield_info trq_sel_fmap_47_field_info[] = {
    {"TRQ_SEL_FMAP_47_RSVD_1", TRQ_SEL_FMAP_47_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_47_QID_MAX", TRQ_SEL_FMAP_47_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_47_QID_BASE", TRQ_SEL_FMAP_47_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 47 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_47_field_info with %d entries\n", sizeof(trq_sel_fmap_47_field_info)/sizeof(trq_sel_fmap_47_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 48
static struct regfield_info trq_sel_fmap_48_field_info[] = {
    {"TRQ_SEL_FMAP_48_RSVD_1", TRQ_SEL_FMAP_48_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_48_QID_MAX", TRQ_SEL_FMAP_48_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_48_QID_BASE", TRQ_SEL_FMAP_48_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 48 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_48_field_info with %d entries\n", sizeof(trq_sel_fmap_48_field_info)/sizeof(trq_sel_fmap_48_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 49
static struct regfield_info trq_sel_fmap_49_field_info[] = {
    {"TRQ_SEL_FMAP_49_RSVD_1", TRQ_SEL_FMAP_49_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_49_QID_MAX", TRQ_SEL_FMAP_49_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_49_QID_BASE", TRQ_SEL_FMAP_49_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 49 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_49_field_info with %d entries\n", sizeof(trq_sel_fmap_49_field_info)/sizeof(trq_sel_fmap_49_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 4A
static struct regfield_info trq_sel_fmap_4a_field_info[] = {
    {"TRQ_SEL_FMAP_4A_RSVD_1", TRQ_SEL_FMAP_4A_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_4A_QID_MAX", TRQ_SEL_FMAP_4A_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_4A_QID_BASE", TRQ_SEL_FMAP_4A_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 4A field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_4a_field_info with %d entries\n", sizeof(trq_sel_fmap_4a_field_info)/sizeof(trq_sel_fmap_4a_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 4B
static struct regfield_info trq_sel_fmap_4b_field_info[] = {
    {"TRQ_SEL_FMAP_4B_RSVD_1", TRQ_SEL_FMAP_4B_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_4B_QID_MAX", TRQ_SEL_FMAP_4B_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_4B_QID_BASE", TRQ_SEL_FMAP_4B_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 4B field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_4b_field_info with %d entries\n", sizeof(trq_sel_fmap_4b_field_info)/sizeof(trq_sel_fmap_4b_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 4C
static struct regfield_info trq_sel_fmap_4c_field_info[] = {
    {"TRQ_SEL_FMAP_4C_RSVD_1", TRQ_SEL_FMAP_4C_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_4C_QID_MAX", TRQ_SEL_FMAP_4C_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_4C_QID_BASE", TRQ_SEL_FMAP_4C_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 4C field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_4c_field_info with %d entries\n", sizeof(trq_sel_fmap_4c_field_info)/sizeof(trq_sel_fmap_4c_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 4D
static struct regfield_info trq_sel_fmap_4d_field_info[] = {
    {"TRQ_SEL_FMAP_4D_RSVD_1", TRQ_SEL_FMAP_4D_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_4D_QID_MAX", TRQ_SEL_FMAP_4D_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_4D_QID_BASE", TRQ_SEL_FMAP_4D_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 4D field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_4d_field_info with %d entries\n", sizeof(trq_sel_fmap_4d_field_info)/sizeof(trq_sel_fmap_4d_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 4E
static struct regfield_info trq_sel_fmap_4e_field_info[] = {
    {"TRQ_SEL_FMAP_4E_RSVD_1", TRQ_SEL_FMAP_4E_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_4E_QID_MAX", TRQ_SEL_FMAP_4E_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_4E_QID_BASE", TRQ_SEL_FMAP_4E_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 4E field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_4e_field_info with %d entries\n", sizeof(trq_sel_fmap_4e_field_info)/sizeof(trq_sel_fmap_4e_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 4F
static struct regfield_info trq_sel_fmap_4f_field_info[] = {
    {"TRQ_SEL_FMAP_4F_RSVD_1", TRQ_SEL_FMAP_4F_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_4F_QID_MAX", TRQ_SEL_FMAP_4F_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_4F_QID_BASE", TRQ_SEL_FMAP_4F_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 4F field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_4f_field_info with %d entries\n", sizeof(trq_sel_fmap_4f_field_info)/sizeof(trq_sel_fmap_4f_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 50
static struct regfield_info trq_sel_fmap_50_field_info[] = {
    {"TRQ_SEL_FMAP_50_RSVD_1", TRQ_SEL_FMAP_50_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_50_QID_MAX", TRQ_SEL_FMAP_50_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_50_QID_BASE", TRQ_SEL_FMAP_50_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 50 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_50_field_info with %d entries\n", sizeof(trq_sel_fmap_50_field_info)/sizeof(trq_sel_fmap_50_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 51
static struct regfield_info trq_sel_fmap_51_field_info[] = {
    {"TRQ_SEL_FMAP_51_RSVD_1", TRQ_SEL_FMAP_51_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_51_QID_MAX", TRQ_SEL_FMAP_51_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_51_QID_BASE", TRQ_SEL_FMAP_51_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 51 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_51_field_info with %d entries\n", sizeof(trq_sel_fmap_51_field_info)/sizeof(trq_sel_fmap_51_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 52
static struct regfield_info trq_sel_fmap_52_field_info[] = {
    {"TRQ_SEL_FMAP_52_RSVD_1", TRQ_SEL_FMAP_52_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_52_QID_MAX", TRQ_SEL_FMAP_52_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_52_QID_BASE", TRQ_SEL_FMAP_52_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 52 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_52_field_info with %d entries\n", sizeof(trq_sel_fmap_52_field_info)/sizeof(trq_sel_fmap_52_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 53
static struct regfield_info trq_sel_fmap_53_field_info[] = {
    {"TRQ_SEL_FMAP_53_RSVD_1", TRQ_SEL_FMAP_53_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_53_QID_MAX", TRQ_SEL_FMAP_53_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_53_QID_BASE", TRQ_SEL_FMAP_53_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 53 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_53_field_info with %d entries\n", sizeof(trq_sel_fmap_53_field_info)/sizeof(trq_sel_fmap_53_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 54
static struct regfield_info trq_sel_fmap_54_field_info[] = {
    {"TRQ_SEL_FMAP_54_RSVD_1", TRQ_SEL_FMAP_54_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_54_QID_MAX", TRQ_SEL_FMAP_54_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_54_QID_BASE", TRQ_SEL_FMAP_54_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 54 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_54_field_info with %d entries\n", sizeof(trq_sel_fmap_54_field_info)/sizeof(trq_sel_fmap_54_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 55
static struct regfield_info trq_sel_fmap_55_field_info[] = {
    {"TRQ_SEL_FMAP_55_RSVD_1", TRQ_SEL_FMAP_55_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_55_QID_MAX", TRQ_SEL_FMAP_55_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_55_QID_BASE", TRQ_SEL_FMAP_55_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 55 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_55_field_info with %d entries\n", sizeof(trq_sel_fmap_55_field_info)/sizeof(trq_sel_fmap_55_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 56
static struct regfield_info trq_sel_fmap_56_field_info[] = {
    {"TRQ_SEL_FMAP_56_RSVD_1", TRQ_SEL_FMAP_56_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_56_QID_MAX", TRQ_SEL_FMAP_56_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_56_QID_BASE", TRQ_SEL_FMAP_56_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 56 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_56_field_info with %d entries\n", sizeof(trq_sel_fmap_56_field_info)/sizeof(trq_sel_fmap_56_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 57
static struct regfield_info trq_sel_fmap_57_field_info[] = {
    {"TRQ_SEL_FMAP_57_RSVD_1", TRQ_SEL_FMAP_57_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_57_QID_MAX", TRQ_SEL_FMAP_57_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_57_QID_BASE", TRQ_SEL_FMAP_57_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 57 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_57_field_info with %d entries\n", sizeof(trq_sel_fmap_57_field_info)/sizeof(trq_sel_fmap_57_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 58
static struct regfield_info trq_sel_fmap_58_field_info[] = {
    {"TRQ_SEL_FMAP_58_RSVD_1", TRQ_SEL_FMAP_58_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_58_QID_MAX", TRQ_SEL_FMAP_58_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_58_QID_BASE", TRQ_SEL_FMAP_58_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 58 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_58_field_info with %d entries\n", sizeof(trq_sel_fmap_58_field_info)/sizeof(trq_sel_fmap_58_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 59
static struct regfield_info trq_sel_fmap_59_field_info[] = {
    {"TRQ_SEL_FMAP_59_RSVD_1", TRQ_SEL_FMAP_59_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_59_QID_MAX", TRQ_SEL_FMAP_59_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_59_QID_BASE", TRQ_SEL_FMAP_59_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 59 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_59_field_info with %d entries\n", sizeof(trq_sel_fmap_59_field_info)/sizeof(trq_sel_fmap_59_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 5A
static struct regfield_info trq_sel_fmap_5a_field_info[] = {
    {"TRQ_SEL_FMAP_5A_RSVD_1", TRQ_SEL_FMAP_5A_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_5A_QID_MAX", TRQ_SEL_FMAP_5A_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_5A_QID_BASE", TRQ_SEL_FMAP_5A_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 5A field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_5a_field_info with %d entries\n", sizeof(trq_sel_fmap_5a_field_info)/sizeof(trq_sel_fmap_5a_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 5B
static struct regfield_info trq_sel_fmap_5b_field_info[] = {
    {"TRQ_SEL_FMAP_5B_RSVD_1", TRQ_SEL_FMAP_5B_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_5B_QID_MAX", TRQ_SEL_FMAP_5B_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_5B_QID_BASE", TRQ_SEL_FMAP_5B_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 5B field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_5b_field_info with %d entries\n", sizeof(trq_sel_fmap_5b_field_info)/sizeof(trq_sel_fmap_5b_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 5C
static struct regfield_info trq_sel_fmap_5c_field_info[] = {
    {"TRQ_SEL_FMAP_5C_RSVD_1", TRQ_SEL_FMAP_5C_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_5C_QID_MAX", TRQ_SEL_FMAP_5C_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_5C_QID_BASE", TRQ_SEL_FMAP_5C_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 5C field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_5c_field_info with %d entries\n", sizeof(trq_sel_fmap_5c_field_info)/sizeof(trq_sel_fmap_5c_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 5D
static struct regfield_info trq_sel_fmap_5d_field_info[] = {
    {"TRQ_SEL_FMAP_5D_RSVD_1", TRQ_SEL_FMAP_5D_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_5D_QID_MAX", TRQ_SEL_FMAP_5D_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_5D_QID_BASE", TRQ_SEL_FMAP_5D_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 5D field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_5d_field_info with %d entries\n", sizeof(trq_sel_fmap_5d_field_info)/sizeof(trq_sel_fmap_5d_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 5E
static struct regfield_info trq_sel_fmap_5e_field_info[] = {
    {"TRQ_SEL_FMAP_5E_RSVD_1", TRQ_SEL_FMAP_5E_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_5E_QID_MAX", TRQ_SEL_FMAP_5E_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_5E_QID_BASE", TRQ_SEL_FMAP_5E_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 5E field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_5e_field_info with %d entries\n", sizeof(trq_sel_fmap_5e_field_info)/sizeof(trq_sel_fmap_5e_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 5F
static struct regfield_info trq_sel_fmap_5f_field_info[] = {
    {"TRQ_SEL_FMAP_5F_RSVD_1", TRQ_SEL_FMAP_5F_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_5F_QID_MAX", TRQ_SEL_FMAP_5F_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_5F_QID_BASE", TRQ_SEL_FMAP_5F_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 5F field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_5f_field_info with %d entries\n", sizeof(trq_sel_fmap_5f_field_info)/sizeof(trq_sel_fmap_5f_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 60
static struct regfield_info trq_sel_fmap_60_field_info[] = {
    {"TRQ_SEL_FMAP_60_RSVD_1", TRQ_SEL_FMAP_60_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_60_QID_MAX", TRQ_SEL_FMAP_60_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_60_QID_BASE", TRQ_SEL_FMAP_60_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 60 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_60_field_info with %d entries\n", sizeof(trq_sel_fmap_60_field_info)/sizeof(trq_sel_fmap_60_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 61
static struct regfield_info trq_sel_fmap_61_field_info[] = {
    {"TRQ_SEL_FMAP_61_RSVD_1", TRQ_SEL_FMAP_61_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_61_QID_MAX", TRQ_SEL_FMAP_61_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_61_QID_BASE", TRQ_SEL_FMAP_61_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 61 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_61_field_info with %d entries\n", sizeof(trq_sel_fmap_61_field_info)/sizeof(trq_sel_fmap_61_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 62
static struct regfield_info trq_sel_fmap_62_field_info[] = {
    {"TRQ_SEL_FMAP_62_RSVD_1", TRQ_SEL_FMAP_62_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_62_QID_MAX", TRQ_SEL_FMAP_62_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_62_QID_BASE", TRQ_SEL_FMAP_62_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 62 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_62_field_info with %d entries\n", sizeof(trq_sel_fmap_62_field_info)/sizeof(trq_sel_fmap_62_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 63
static struct regfield_info trq_sel_fmap_63_field_info[] = {
    {"TRQ_SEL_FMAP_63_RSVD_1", TRQ_SEL_FMAP_63_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_63_QID_MAX", TRQ_SEL_FMAP_63_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_63_QID_BASE", TRQ_SEL_FMAP_63_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 63 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_63_field_info with %d entries\n", sizeof(trq_sel_fmap_63_field_info)/sizeof(trq_sel_fmap_63_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 64
static struct regfield_info trq_sel_fmap_64_field_info[] = {
    {"TRQ_SEL_FMAP_64_RSVD_1", TRQ_SEL_FMAP_64_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_64_QID_MAX", TRQ_SEL_FMAP_64_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_64_QID_BASE", TRQ_SEL_FMAP_64_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 64 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_64_field_info with %d entries\n", sizeof(trq_sel_fmap_64_field_info)/sizeof(trq_sel_fmap_64_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 65
static struct regfield_info trq_sel_fmap_65_field_info[] = {
    {"TRQ_SEL_FMAP_65_RSVD_1", TRQ_SEL_FMAP_65_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_65_QID_MAX", TRQ_SEL_FMAP_65_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_65_QID_BASE", TRQ_SEL_FMAP_65_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 65 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_65_field_info with %d entries\n", sizeof(trq_sel_fmap_65_field_info)/sizeof(trq_sel_fmap_65_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 66
static struct regfield_info trq_sel_fmap_66_field_info[] = {
    {"TRQ_SEL_FMAP_66_RSVD_1", TRQ_SEL_FMAP_66_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_66_QID_MAX", TRQ_SEL_FMAP_66_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_66_QID_BASE", TRQ_SEL_FMAP_66_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 66 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_66_field_info with %d entries\n", sizeof(trq_sel_fmap_66_field_info)/sizeof(trq_sel_fmap_66_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 67
static struct regfield_info trq_sel_fmap_67_field_info[] = {
    {"TRQ_SEL_FMAP_67_RSVD_1", TRQ_SEL_FMAP_67_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_67_QID_MAX", TRQ_SEL_FMAP_67_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_67_QID_BASE", TRQ_SEL_FMAP_67_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 67 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_67_field_info with %d entries\n", sizeof(trq_sel_fmap_67_field_info)/sizeof(trq_sel_fmap_67_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 68
static struct regfield_info trq_sel_fmap_68_field_info[] = {
    {"TRQ_SEL_FMAP_68_RSVD_1", TRQ_SEL_FMAP_68_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_68_QID_MAX", TRQ_SEL_FMAP_68_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_68_QID_BASE", TRQ_SEL_FMAP_68_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 68 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_68_field_info with %d entries\n", sizeof(trq_sel_fmap_68_field_info)/sizeof(trq_sel_fmap_68_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 69
static struct regfield_info trq_sel_fmap_69_field_info[] = {
    {"TRQ_SEL_FMAP_69_RSVD_1", TRQ_SEL_FMAP_69_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_69_QID_MAX", TRQ_SEL_FMAP_69_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_69_QID_BASE", TRQ_SEL_FMAP_69_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 69 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_69_field_info with %d entries\n", sizeof(trq_sel_fmap_69_field_info)/sizeof(trq_sel_fmap_69_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 6A
static struct regfield_info trq_sel_fmap_6a_field_info[] = {
    {"TRQ_SEL_FMAP_6A_RSVD_1", TRQ_SEL_FMAP_6A_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_6A_QID_MAX", TRQ_SEL_FMAP_6A_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_6A_QID_BASE", TRQ_SEL_FMAP_6A_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 6A field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_6a_field_info with %d entries\n", sizeof(trq_sel_fmap_6a_field_info)/sizeof(trq_sel_fmap_6a_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 6B
static struct regfield_info trq_sel_fmap_6b_field_info[] = {
    {"TRQ_SEL_FMAP_6B_RSVD_1", TRQ_SEL_FMAP_6B_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_6B_QID_MAX", TRQ_SEL_FMAP_6B_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_6B_QID_BASE", TRQ_SEL_FMAP_6B_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 6B field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_6b_field_info with %d entries\n", sizeof(trq_sel_fmap_6b_field_info)/sizeof(trq_sel_fmap_6b_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 6C
static struct regfield_info trq_sel_fmap_6c_field_info[] = {
    {"TRQ_SEL_FMAP_6C_RSVD_1", TRQ_SEL_FMAP_6C_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_6C_QID_MAX", TRQ_SEL_FMAP_6C_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_6C_QID_BASE", TRQ_SEL_FMAP_6C_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 6C field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_6c_field_info with %d entries\n", sizeof(trq_sel_fmap_6c_field_info)/sizeof(trq_sel_fmap_6c_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 6D
static struct regfield_info trq_sel_fmap_6d_field_info[] = {
    {"TRQ_SEL_FMAP_6D_RSVD_1", TRQ_SEL_FMAP_6D_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_6D_QID_MAX", TRQ_SEL_FMAP_6D_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_6D_QID_BASE", TRQ_SEL_FMAP_6D_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 6D field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_6d_field_info with %d entries\n", sizeof(trq_sel_fmap_6d_field_info)/sizeof(trq_sel_fmap_6d_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 6E
static struct regfield_info trq_sel_fmap_6e_field_info[] = {
    {"TRQ_SEL_FMAP_6E_RSVD_1", TRQ_SEL_FMAP_6E_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_6E_QID_MAX", TRQ_SEL_FMAP_6E_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_6E_QID_BASE", TRQ_SEL_FMAP_6E_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 6E field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_6e_field_info with %d entries\n", sizeof(trq_sel_fmap_6e_field_info)/sizeof(trq_sel_fmap_6e_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 6F
static struct regfield_info trq_sel_fmap_6f_field_info[] = {
    {"TRQ_SEL_FMAP_6F_RSVD_1", TRQ_SEL_FMAP_6F_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_6F_QID_MAX", TRQ_SEL_FMAP_6F_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_6F_QID_BASE", TRQ_SEL_FMAP_6F_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 6F field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_6f_field_info with %d entries\n", sizeof(trq_sel_fmap_6f_field_info)/sizeof(trq_sel_fmap_6f_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 70
static struct regfield_info trq_sel_fmap_70_field_info[] = {
    {"TRQ_SEL_FMAP_70_RSVD_1", TRQ_SEL_FMAP_70_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_70_QID_MAX", TRQ_SEL_FMAP_70_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_70_QID_BASE", TRQ_SEL_FMAP_70_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 70 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_70_field_info with %d entries\n", sizeof(trq_sel_fmap_70_field_info)/sizeof(trq_sel_fmap_70_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 71
static struct regfield_info trq_sel_fmap_71_field_info[] = {
    {"TRQ_SEL_FMAP_71_RSVD_1", TRQ_SEL_FMAP_71_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_71_QID_MAX", TRQ_SEL_FMAP_71_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_71_QID_BASE", TRQ_SEL_FMAP_71_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 71 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_71_field_info with %d entries\n", sizeof(trq_sel_fmap_71_field_info)/sizeof(trq_sel_fmap_71_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 72
static struct regfield_info trq_sel_fmap_72_field_info[] = {
    {"TRQ_SEL_FMAP_72_RSVD_1", TRQ_SEL_FMAP_72_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_72_QID_MAX", TRQ_SEL_FMAP_72_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_72_QID_BASE", TRQ_SEL_FMAP_72_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 72 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_72_field_info with %d entries\n", sizeof(trq_sel_fmap_72_field_info)/sizeof(trq_sel_fmap_72_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 73
static struct regfield_info trq_sel_fmap_73_field_info[] = {
    {"TRQ_SEL_FMAP_73_RSVD_1", TRQ_SEL_FMAP_73_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_73_QID_MAX", TRQ_SEL_FMAP_73_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_73_QID_BASE", TRQ_SEL_FMAP_73_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 73 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_73_field_info with %d entries\n", sizeof(trq_sel_fmap_73_field_info)/sizeof(trq_sel_fmap_73_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 74
static struct regfield_info trq_sel_fmap_74_field_info[] = {
    {"TRQ_SEL_FMAP_74_RSVD_1", TRQ_SEL_FMAP_74_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_74_QID_MAX", TRQ_SEL_FMAP_74_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_74_QID_BASE", TRQ_SEL_FMAP_74_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 74 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_74_field_info with %d entries\n", sizeof(trq_sel_fmap_74_field_info)/sizeof(trq_sel_fmap_74_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 75
static struct regfield_info trq_sel_fmap_75_field_info[] = {
    {"TRQ_SEL_FMAP_75_RSVD_1", TRQ_SEL_FMAP_75_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_75_QID_MAX", TRQ_SEL_FMAP_75_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_75_QID_BASE", TRQ_SEL_FMAP_75_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 75 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_75_field_info with %d entries\n", sizeof(trq_sel_fmap_75_field_info)/sizeof(trq_sel_fmap_75_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 76
static struct regfield_info trq_sel_fmap_76_field_info[] = {
    {"TRQ_SEL_FMAP_76_RSVD_1", TRQ_SEL_FMAP_76_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_76_QID_MAX", TRQ_SEL_FMAP_76_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_76_QID_BASE", TRQ_SEL_FMAP_76_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 76 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_76_field_info with %d entries\n", sizeof(trq_sel_fmap_76_field_info)/sizeof(trq_sel_fmap_76_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 77
static struct regfield_info trq_sel_fmap_77_field_info[] = {
    {"TRQ_SEL_FMAP_77_RSVD_1", TRQ_SEL_FMAP_77_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_77_QID_MAX", TRQ_SEL_FMAP_77_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_77_QID_BASE", TRQ_SEL_FMAP_77_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 77 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_77_field_info with %d entries\n", sizeof(trq_sel_fmap_77_field_info)/sizeof(trq_sel_fmap_77_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 78
static struct regfield_info trq_sel_fmap_78_field_info[] = {
    {"TRQ_SEL_FMAP_78_RSVD_1", TRQ_SEL_FMAP_78_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_78_QID_MAX", TRQ_SEL_FMAP_78_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_78_QID_BASE", TRQ_SEL_FMAP_78_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 78 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_78_field_info with %d entries\n", sizeof(trq_sel_fmap_78_field_info)/sizeof(trq_sel_fmap_78_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 79
static struct regfield_info trq_sel_fmap_79_field_info[] = {
    {"TRQ_SEL_FMAP_79_RSVD_1", TRQ_SEL_FMAP_79_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_79_QID_MAX", TRQ_SEL_FMAP_79_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_79_QID_BASE", TRQ_SEL_FMAP_79_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 79 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_79_field_info with %d entries\n", sizeof(trq_sel_fmap_79_field_info)/sizeof(trq_sel_fmap_79_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 7A
static struct regfield_info trq_sel_fmap_7a_field_info[] = {
    {"TRQ_SEL_FMAP_7A_RSVD_1", TRQ_SEL_FMAP_7A_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_7A_QID_MAX", TRQ_SEL_FMAP_7A_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_7A_QID_BASE", TRQ_SEL_FMAP_7A_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 7A field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_7a_field_info with %d entries\n", sizeof(trq_sel_fmap_7a_field_info)/sizeof(trq_sel_fmap_7a_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 7B
static struct regfield_info trq_sel_fmap_7b_field_info[] = {
    {"TRQ_SEL_FMAP_7B_RSVD_1", TRQ_SEL_FMAP_7B_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_7B_QID_MAX", TRQ_SEL_FMAP_7B_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_7B_QID_BASE", TRQ_SEL_FMAP_7B_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 7B field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_7b_field_info with %d entries\n", sizeof(trq_sel_fmap_7b_field_info)/sizeof(trq_sel_fmap_7b_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 7C
static struct regfield_info trq_sel_fmap_7c_field_info[] = {
    {"TRQ_SEL_FMAP_7C_RSVD_1", TRQ_SEL_FMAP_7C_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_7C_QID_MAX", TRQ_SEL_FMAP_7C_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_7C_QID_BASE", TRQ_SEL_FMAP_7C_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 7C field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_7c_field_info with %d entries\n", sizeof(trq_sel_fmap_7c_field_info)/sizeof(trq_sel_fmap_7c_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 7D
static struct regfield_info trq_sel_fmap_7d_field_info[] = {
    {"TRQ_SEL_FMAP_7D_RSVD_1", TRQ_SEL_FMAP_7D_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_7D_QID_MAX", TRQ_SEL_FMAP_7D_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_7D_QID_BASE", TRQ_SEL_FMAP_7D_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 7D field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_7d_field_info with %d entries\n", sizeof(trq_sel_fmap_7d_field_info)/sizeof(trq_sel_fmap_7d_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 7E
static struct regfield_info trq_sel_fmap_7e_field_info[] = {
    {"TRQ_SEL_FMAP_7E_RSVD_1", TRQ_SEL_FMAP_7E_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_7E_QID_MAX", TRQ_SEL_FMAP_7E_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_7E_QID_BASE", TRQ_SEL_FMAP_7E_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 7E field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_7e_field_info with %d entries\n", sizeof(trq_sel_fmap_7e_field_info)/sizeof(trq_sel_fmap_7e_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 7F
static struct regfield_info trq_sel_fmap_7f_field_info[] = {
    {"TRQ_SEL_FMAP_7F_RSVD_1", TRQ_SEL_FMAP_7F_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_7F_QID_MAX", TRQ_SEL_FMAP_7F_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_7F_QID_BASE", TRQ_SEL_FMAP_7F_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 7F field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_7f_field_info with %d entries\n", sizeof(trq_sel_fmap_7f_field_info)/sizeof(trq_sel_fmap_7f_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 80
static struct regfield_info trq_sel_fmap_80_field_info[] = {
    {"TRQ_SEL_FMAP_80_RSVD_1", TRQ_SEL_FMAP_80_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_80_QID_MAX", TRQ_SEL_FMAP_80_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_80_QID_BASE", TRQ_SEL_FMAP_80_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 80 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_80_field_info with %d entries\n", sizeof(trq_sel_fmap_80_field_info)/sizeof(trq_sel_fmap_80_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 81
static struct regfield_info trq_sel_fmap_81_field_info[] = {
    {"TRQ_SEL_FMAP_81_RSVD_1", TRQ_SEL_FMAP_81_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_81_QID_MAX", TRQ_SEL_FMAP_81_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_81_QID_BASE", TRQ_SEL_FMAP_81_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 81 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_81_field_info with %d entries\n", sizeof(trq_sel_fmap_81_field_info)/sizeof(trq_sel_fmap_81_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 82
static struct regfield_info trq_sel_fmap_82_field_info[] = {
    {"TRQ_SEL_FMAP_82_RSVD_1", TRQ_SEL_FMAP_82_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_82_QID_MAX", TRQ_SEL_FMAP_82_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_82_QID_BASE", TRQ_SEL_FMAP_82_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 82 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_82_field_info with %d entries\n", sizeof(trq_sel_fmap_82_field_info)/sizeof(trq_sel_fmap_82_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 83
static struct regfield_info trq_sel_fmap_83_field_info[] = {
    {"TRQ_SEL_FMAP_83_RSVD_1", TRQ_SEL_FMAP_83_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_83_QID_MAX", TRQ_SEL_FMAP_83_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_83_QID_BASE", TRQ_SEL_FMAP_83_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 83 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_83_field_info with %d entries\n", sizeof(trq_sel_fmap_83_field_info)/sizeof(trq_sel_fmap_83_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 84
static struct regfield_info trq_sel_fmap_84_field_info[] = {
    {"TRQ_SEL_FMAP_84_RSVD_1", TRQ_SEL_FMAP_84_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_84_QID_MAX", TRQ_SEL_FMAP_84_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_84_QID_BASE", TRQ_SEL_FMAP_84_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 84 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_84_field_info with %d entries\n", sizeof(trq_sel_fmap_84_field_info)/sizeof(trq_sel_fmap_84_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 85
static struct regfield_info trq_sel_fmap_85_field_info[] = {
    {"TRQ_SEL_FMAP_85_RSVD_1", TRQ_SEL_FMAP_85_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_85_QID_MAX", TRQ_SEL_FMAP_85_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_85_QID_BASE", TRQ_SEL_FMAP_85_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 85 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_85_field_info with %d entries\n", sizeof(trq_sel_fmap_85_field_info)/sizeof(trq_sel_fmap_85_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 86
static struct regfield_info trq_sel_fmap_86_field_info[] = {
    {"TRQ_SEL_FMAP_86_RSVD_1", TRQ_SEL_FMAP_86_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_86_QID_MAX", TRQ_SEL_FMAP_86_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_86_QID_BASE", TRQ_SEL_FMAP_86_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 86 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_86_field_info with %d entries\n", sizeof(trq_sel_fmap_86_field_info)/sizeof(trq_sel_fmap_86_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 87
static struct regfield_info trq_sel_fmap_87_field_info[] = {
    {"TRQ_SEL_FMAP_87_RSVD_1", TRQ_SEL_FMAP_87_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_87_QID_MAX", TRQ_SEL_FMAP_87_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_87_QID_BASE", TRQ_SEL_FMAP_87_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 87 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_87_field_info with %d entries\n", sizeof(trq_sel_fmap_87_field_info)/sizeof(trq_sel_fmap_87_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 88
static struct regfield_info trq_sel_fmap_88_field_info[] = {
    {"TRQ_SEL_FMAP_88_RSVD_1", TRQ_SEL_FMAP_88_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_88_QID_MAX", TRQ_SEL_FMAP_88_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_88_QID_BASE", TRQ_SEL_FMAP_88_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 88 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_88_field_info with %d entries\n", sizeof(trq_sel_fmap_88_field_info)/sizeof(trq_sel_fmap_88_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 89
static struct regfield_info trq_sel_fmap_89_field_info[] = {
    {"TRQ_SEL_FMAP_89_RSVD_1", TRQ_SEL_FMAP_89_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_89_QID_MAX", TRQ_SEL_FMAP_89_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_89_QID_BASE", TRQ_SEL_FMAP_89_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 89 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_89_field_info with %d entries\n", sizeof(trq_sel_fmap_89_field_info)/sizeof(trq_sel_fmap_89_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 8A
static struct regfield_info trq_sel_fmap_8a_field_info[] = {
    {"TRQ_SEL_FMAP_8A_RSVD_1", TRQ_SEL_FMAP_8A_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_8A_QID_MAX", TRQ_SEL_FMAP_8A_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_8A_QID_BASE", TRQ_SEL_FMAP_8A_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 8A field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_8a_field_info with %d entries\n", sizeof(trq_sel_fmap_8a_field_info)/sizeof(trq_sel_fmap_8a_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 8B
static struct regfield_info trq_sel_fmap_8b_field_info[] = {
    {"TRQ_SEL_FMAP_8B_RSVD_1", TRQ_SEL_FMAP_8B_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_8B_QID_MAX", TRQ_SEL_FMAP_8B_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_8B_QID_BASE", TRQ_SEL_FMAP_8B_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 8B field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_8b_field_info with %d entries\n", sizeof(trq_sel_fmap_8b_field_info)/sizeof(trq_sel_fmap_8b_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 8C
static struct regfield_info trq_sel_fmap_8c_field_info[] = {
    {"TRQ_SEL_FMAP_8C_RSVD_1", TRQ_SEL_FMAP_8C_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_8C_QID_MAX", TRQ_SEL_FMAP_8C_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_8C_QID_BASE", TRQ_SEL_FMAP_8C_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 8C field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_8c_field_info with %d entries\n", sizeof(trq_sel_fmap_8c_field_info)/sizeof(trq_sel_fmap_8c_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 8D
static struct regfield_info trq_sel_fmap_8d_field_info[] = {
    {"TRQ_SEL_FMAP_8D_RSVD_1", TRQ_SEL_FMAP_8D_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_8D_QID_MAX", TRQ_SEL_FMAP_8D_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_8D_QID_BASE", TRQ_SEL_FMAP_8D_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 8D field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_8d_field_info with %d entries\n", sizeof(trq_sel_fmap_8d_field_info)/sizeof(trq_sel_fmap_8d_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 8E
static struct regfield_info trq_sel_fmap_8e_field_info[] = {
    {"TRQ_SEL_FMAP_8E_RSVD_1", TRQ_SEL_FMAP_8E_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_8E_QID_MAX", TRQ_SEL_FMAP_8E_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_8E_QID_BASE", TRQ_SEL_FMAP_8E_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 8E field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_8e_field_info with %d entries\n", sizeof(trq_sel_fmap_8e_field_info)/sizeof(trq_sel_fmap_8e_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 8F
static struct regfield_info trq_sel_fmap_8f_field_info[] = {
    {"TRQ_SEL_FMAP_8F_RSVD_1", TRQ_SEL_FMAP_8F_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_8F_QID_MAX", TRQ_SEL_FMAP_8F_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_8F_QID_BASE", TRQ_SEL_FMAP_8F_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 8F field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_8f_field_info with %d entries\n", sizeof(trq_sel_fmap_8f_field_info)/sizeof(trq_sel_fmap_8f_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 90
static struct regfield_info trq_sel_fmap_90_field_info[] = {
    {"TRQ_SEL_FMAP_90_RSVD_1", TRQ_SEL_FMAP_90_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_90_QID_MAX", TRQ_SEL_FMAP_90_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_90_QID_BASE", TRQ_SEL_FMAP_90_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 90 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_90_field_info with %d entries\n", sizeof(trq_sel_fmap_90_field_info)/sizeof(trq_sel_fmap_90_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 91
static struct regfield_info trq_sel_fmap_91_field_info[] = {
    {"TRQ_SEL_FMAP_91_RSVD_1", TRQ_SEL_FMAP_91_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_91_QID_MAX", TRQ_SEL_FMAP_91_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_91_QID_BASE", TRQ_SEL_FMAP_91_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 91 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_91_field_info with %d entries\n", sizeof(trq_sel_fmap_91_field_info)/sizeof(trq_sel_fmap_91_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 92
static struct regfield_info trq_sel_fmap_92_field_info[] = {
    {"TRQ_SEL_FMAP_92_RSVD_1", TRQ_SEL_FMAP_92_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_92_QID_MAX", TRQ_SEL_FMAP_92_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_92_QID_BASE", TRQ_SEL_FMAP_92_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 92 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_92_field_info with %d entries\n", sizeof(trq_sel_fmap_92_field_info)/sizeof(trq_sel_fmap_92_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 93
static struct regfield_info trq_sel_fmap_93_field_info[] = {
    {"TRQ_SEL_FMAP_93_RSVD_1", TRQ_SEL_FMAP_93_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_93_QID_MAX", TRQ_SEL_FMAP_93_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_93_QID_BASE", TRQ_SEL_FMAP_93_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 93 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_93_field_info with %d entries\n", sizeof(trq_sel_fmap_93_field_info)/sizeof(trq_sel_fmap_93_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 94
static struct regfield_info trq_sel_fmap_94_field_info[] = {
    {"TRQ_SEL_FMAP_94_RSVD_1", TRQ_SEL_FMAP_94_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_94_QID_MAX", TRQ_SEL_FMAP_94_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_94_QID_BASE", TRQ_SEL_FMAP_94_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 94 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_94_field_info with %d entries\n", sizeof(trq_sel_fmap_94_field_info)/sizeof(trq_sel_fmap_94_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 95
static struct regfield_info trq_sel_fmap_95_field_info[] = {
    {"TRQ_SEL_FMAP_95_RSVD_1", TRQ_SEL_FMAP_95_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_95_QID_MAX", TRQ_SEL_FMAP_95_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_95_QID_BASE", TRQ_SEL_FMAP_95_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 95 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_95_field_info with %d entries\n", sizeof(trq_sel_fmap_95_field_info)/sizeof(trq_sel_fmap_95_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 96
static struct regfield_info trq_sel_fmap_96_field_info[] = {
    {"TRQ_SEL_FMAP_96_RSVD_1", TRQ_SEL_FMAP_96_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_96_QID_MAX", TRQ_SEL_FMAP_96_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_96_QID_BASE", TRQ_SEL_FMAP_96_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 96 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_96_field_info with %d entries\n", sizeof(trq_sel_fmap_96_field_info)/sizeof(trq_sel_fmap_96_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 97
static struct regfield_info trq_sel_fmap_97_field_info[] = {
    {"TRQ_SEL_FMAP_97_RSVD_1", TRQ_SEL_FMAP_97_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_97_QID_MAX", TRQ_SEL_FMAP_97_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_97_QID_BASE", TRQ_SEL_FMAP_97_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 97 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_97_field_info with %d entries\n", sizeof(trq_sel_fmap_97_field_info)/sizeof(trq_sel_fmap_97_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 98
static struct regfield_info trq_sel_fmap_98_field_info[] = {
    {"TRQ_SEL_FMAP_98_RSVD_1", TRQ_SEL_FMAP_98_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_98_QID_MAX", TRQ_SEL_FMAP_98_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_98_QID_BASE", TRQ_SEL_FMAP_98_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 98 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_98_field_info with %d entries\n", sizeof(trq_sel_fmap_98_field_info)/sizeof(trq_sel_fmap_98_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 99
static struct regfield_info trq_sel_fmap_99_field_info[] = {
    {"TRQ_SEL_FMAP_99_RSVD_1", TRQ_SEL_FMAP_99_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_99_QID_MAX", TRQ_SEL_FMAP_99_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_99_QID_BASE", TRQ_SEL_FMAP_99_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 99 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_99_field_info with %d entries\n", sizeof(trq_sel_fmap_99_field_info)/sizeof(trq_sel_fmap_99_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 9A
static struct regfield_info trq_sel_fmap_9a_field_info[] = {
    {"TRQ_SEL_FMAP_9A_RSVD_1", TRQ_SEL_FMAP_9A_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_9A_QID_MAX", TRQ_SEL_FMAP_9A_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_9A_QID_BASE", TRQ_SEL_FMAP_9A_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 9A field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_9a_field_info with %d entries\n", sizeof(trq_sel_fmap_9a_field_info)/sizeof(trq_sel_fmap_9a_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 9B
static struct regfield_info trq_sel_fmap_9b_field_info[] = {
    {"TRQ_SEL_FMAP_9B_RSVD_1", TRQ_SEL_FMAP_9B_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_9B_QID_MAX", TRQ_SEL_FMAP_9B_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_9B_QID_BASE", TRQ_SEL_FMAP_9B_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 9B field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_9b_field_info with %d entries\n", sizeof(trq_sel_fmap_9b_field_info)/sizeof(trq_sel_fmap_9b_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 9C
static struct regfield_info trq_sel_fmap_9c_field_info[] = {
    {"TRQ_SEL_FMAP_9C_RSVD_1", TRQ_SEL_FMAP_9C_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_9C_QID_MAX", TRQ_SEL_FMAP_9C_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_9C_QID_BASE", TRQ_SEL_FMAP_9C_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 9C field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_9c_field_info with %d entries\n", sizeof(trq_sel_fmap_9c_field_info)/sizeof(trq_sel_fmap_9c_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 9D
static struct regfield_info trq_sel_fmap_9d_field_info[] = {
    {"TRQ_SEL_FMAP_9D_RSVD_1", TRQ_SEL_FMAP_9D_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_9D_QID_MAX", TRQ_SEL_FMAP_9D_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_9D_QID_BASE", TRQ_SEL_FMAP_9D_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 9D field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_9d_field_info with %d entries\n", sizeof(trq_sel_fmap_9d_field_info)/sizeof(trq_sel_fmap_9d_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 9E
static struct regfield_info trq_sel_fmap_9e_field_info[] = {
    {"TRQ_SEL_FMAP_9E_RSVD_1", TRQ_SEL_FMAP_9E_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_9E_QID_MAX", TRQ_SEL_FMAP_9E_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_9E_QID_BASE", TRQ_SEL_FMAP_9E_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 9E field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_9e_field_info with %d entries\n", sizeof(trq_sel_fmap_9e_field_info)/sizeof(trq_sel_fmap_9e_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap 9F
static struct regfield_info trq_sel_fmap_9f_field_info[] = {
    {"TRQ_SEL_FMAP_9F_RSVD_1", TRQ_SEL_FMAP_9F_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_9F_QID_MAX", TRQ_SEL_FMAP_9F_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_9F_QID_BASE", TRQ_SEL_FMAP_9F_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap 9F field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_9f_field_info with %d entries\n", sizeof(trq_sel_fmap_9f_field_info)/sizeof(trq_sel_fmap_9f_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap A0
static struct regfield_info trq_sel_fmap_a0_field_info[] = {
    {"TRQ_SEL_FMAP_A0_RSVD_1", TRQ_SEL_FMAP_A0_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_A0_QID_MAX", TRQ_SEL_FMAP_A0_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_A0_QID_BASE", TRQ_SEL_FMAP_A0_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap A0 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_a0_field_info with %d entries\n", sizeof(trq_sel_fmap_a0_field_info)/sizeof(trq_sel_fmap_a0_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap A1
static struct regfield_info trq_sel_fmap_a1_field_info[] = {
    {"TRQ_SEL_FMAP_A1_RSVD_1", TRQ_SEL_FMAP_A1_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_A1_QID_MAX", TRQ_SEL_FMAP_A1_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_A1_QID_BASE", TRQ_SEL_FMAP_A1_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap A1 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_a1_field_info with %d entries\n", sizeof(trq_sel_fmap_a1_field_info)/sizeof(trq_sel_fmap_a1_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap A2
static struct regfield_info trq_sel_fmap_a2_field_info[] = {
    {"TRQ_SEL_FMAP_A2_RSVD_1", TRQ_SEL_FMAP_A2_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_A2_QID_MAX", TRQ_SEL_FMAP_A2_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_A2_QID_BASE", TRQ_SEL_FMAP_A2_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap A2 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_a2_field_info with %d entries\n", sizeof(trq_sel_fmap_a2_field_info)/sizeof(trq_sel_fmap_a2_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap A3
static struct regfield_info trq_sel_fmap_a3_field_info[] = {
    {"TRQ_SEL_FMAP_A3_RSVD_1", TRQ_SEL_FMAP_A3_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_A3_QID_MAX", TRQ_SEL_FMAP_A3_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_A3_QID_BASE", TRQ_SEL_FMAP_A3_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap A3 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_a3_field_info with %d entries\n", sizeof(trq_sel_fmap_a3_field_info)/sizeof(trq_sel_fmap_a3_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap A4
static struct regfield_info trq_sel_fmap_a4_field_info[] = {
    {"TRQ_SEL_FMAP_A4_RSVD_1", TRQ_SEL_FMAP_A4_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_A4_QID_MAX", TRQ_SEL_FMAP_A4_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_A4_QID_BASE", TRQ_SEL_FMAP_A4_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap A4 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_a4_field_info with %d entries\n", sizeof(trq_sel_fmap_a4_field_info)/sizeof(trq_sel_fmap_a4_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap A5
static struct regfield_info trq_sel_fmap_a5_field_info[] = {
    {"TRQ_SEL_FMAP_A5_RSVD_1", TRQ_SEL_FMAP_A5_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_A5_QID_MAX", TRQ_SEL_FMAP_A5_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_A5_QID_BASE", TRQ_SEL_FMAP_A5_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap A5 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_a5_field_info with %d entries\n", sizeof(trq_sel_fmap_a5_field_info)/sizeof(trq_sel_fmap_a5_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap A6
static struct regfield_info trq_sel_fmap_a6_field_info[] = {
    {"TRQ_SEL_FMAP_A6_RSVD_1", TRQ_SEL_FMAP_A6_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_A6_QID_MAX", TRQ_SEL_FMAP_A6_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_A6_QID_BASE", TRQ_SEL_FMAP_A6_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap A6 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_a6_field_info with %d entries\n", sizeof(trq_sel_fmap_a6_field_info)/sizeof(trq_sel_fmap_a6_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap A7
static struct regfield_info trq_sel_fmap_a7_field_info[] = {
    {"TRQ_SEL_FMAP_A7_RSVD_1", TRQ_SEL_FMAP_A7_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_A7_QID_MAX", TRQ_SEL_FMAP_A7_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_A7_QID_BASE", TRQ_SEL_FMAP_A7_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap A7 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_a7_field_info with %d entries\n", sizeof(trq_sel_fmap_a7_field_info)/sizeof(trq_sel_fmap_a7_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap A8
static struct regfield_info trq_sel_fmap_a8_field_info[] = {
    {"TRQ_SEL_FMAP_A8_RSVD_1", TRQ_SEL_FMAP_A8_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_A8_QID_MAX", TRQ_SEL_FMAP_A8_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_A8_QID_BASE", TRQ_SEL_FMAP_A8_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap A8 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_a8_field_info with %d entries\n", sizeof(trq_sel_fmap_a8_field_info)/sizeof(trq_sel_fmap_a8_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap A9
static struct regfield_info trq_sel_fmap_a9_field_info[] = {
    {"TRQ_SEL_FMAP_A9_RSVD_1", TRQ_SEL_FMAP_A9_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_A9_QID_MAX", TRQ_SEL_FMAP_A9_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_A9_QID_BASE", TRQ_SEL_FMAP_A9_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap A9 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_a9_field_info with %d entries\n", sizeof(trq_sel_fmap_a9_field_info)/sizeof(trq_sel_fmap_a9_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap AA
static struct regfield_info trq_sel_fmap_aa_field_info[] = {
    {"TRQ_SEL_FMAP_AA_RSVD_1", TRQ_SEL_FMAP_AA_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_AA_QID_MAX", TRQ_SEL_FMAP_AA_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_AA_QID_BASE", TRQ_SEL_FMAP_AA_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap AA field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_aa_field_info with %d entries\n", sizeof(trq_sel_fmap_aa_field_info)/sizeof(trq_sel_fmap_aa_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap AB
static struct regfield_info trq_sel_fmap_ab_field_info[] = {
    {"TRQ_SEL_FMAP_AB_RSVD_1", TRQ_SEL_FMAP_AB_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_AB_QID_MAX", TRQ_SEL_FMAP_AB_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_AB_QID_BASE", TRQ_SEL_FMAP_AB_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap AB field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_ab_field_info with %d entries\n", sizeof(trq_sel_fmap_ab_field_info)/sizeof(trq_sel_fmap_ab_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap AC
static struct regfield_info trq_sel_fmap_ac_field_info[] = {
    {"TRQ_SEL_FMAP_AC_RSVD_1", TRQ_SEL_FMAP_AC_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_AC_QID_MAX", TRQ_SEL_FMAP_AC_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_AC_QID_BASE", TRQ_SEL_FMAP_AC_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap AC field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_ac_field_info with %d entries\n", sizeof(trq_sel_fmap_ac_field_info)/sizeof(trq_sel_fmap_ac_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap AD
static struct regfield_info trq_sel_fmap_ad_field_info[] = {
    {"TRQ_SEL_FMAP_AD_RSVD_1", TRQ_SEL_FMAP_AD_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_AD_QID_MAX", TRQ_SEL_FMAP_AD_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_AD_QID_BASE", TRQ_SEL_FMAP_AD_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap AD field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_ad_field_info with %d entries\n", sizeof(trq_sel_fmap_ad_field_info)/sizeof(trq_sel_fmap_ad_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap AE
static struct regfield_info trq_sel_fmap_ae_field_info[] = {
    {"TRQ_SEL_FMAP_AE_RSVD_1", TRQ_SEL_FMAP_AE_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_AE_QID_MAX", TRQ_SEL_FMAP_AE_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_AE_QID_BASE", TRQ_SEL_FMAP_AE_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap AE field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_ae_field_info with %d entries\n", sizeof(trq_sel_fmap_ae_field_info)/sizeof(trq_sel_fmap_ae_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap AF
static struct regfield_info trq_sel_fmap_af_field_info[] = {
    {"TRQ_SEL_FMAP_AF_RSVD_1", TRQ_SEL_FMAP_AF_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_AF_QID_MAX", TRQ_SEL_FMAP_AF_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_AF_QID_BASE", TRQ_SEL_FMAP_AF_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap AF field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_af_field_info with %d entries\n", sizeof(trq_sel_fmap_af_field_info)/sizeof(trq_sel_fmap_af_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap B0
static struct regfield_info trq_sel_fmap_b0_field_info[] = {
    {"TRQ_SEL_FMAP_B0_RSVD_1", TRQ_SEL_FMAP_B0_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_B0_QID_MAX", TRQ_SEL_FMAP_B0_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_B0_QID_BASE", TRQ_SEL_FMAP_B0_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap B0 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_b0_field_info with %d entries\n", sizeof(trq_sel_fmap_b0_field_info)/sizeof(trq_sel_fmap_b0_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap B1
static struct regfield_info trq_sel_fmap_b1_field_info[] = {
    {"TRQ_SEL_FMAP_B1_RSVD_1", TRQ_SEL_FMAP_B1_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_B1_QID_MAX", TRQ_SEL_FMAP_B1_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_B1_QID_BASE", TRQ_SEL_FMAP_B1_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap B1 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_b1_field_info with %d entries\n", sizeof(trq_sel_fmap_b1_field_info)/sizeof(trq_sel_fmap_b1_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap B2
static struct regfield_info trq_sel_fmap_b2_field_info[] = {
    {"TRQ_SEL_FMAP_B2_RSVD_1", TRQ_SEL_FMAP_B2_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_B2_QID_MAX", TRQ_SEL_FMAP_B2_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_B2_QID_BASE", TRQ_SEL_FMAP_B2_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap B2 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_b2_field_info with %d entries\n", sizeof(trq_sel_fmap_b2_field_info)/sizeof(trq_sel_fmap_b2_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap B3
static struct regfield_info trq_sel_fmap_b3_field_info[] = {
    {"TRQ_SEL_FMAP_B3_RSVD_1", TRQ_SEL_FMAP_B3_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_B3_QID_MAX", TRQ_SEL_FMAP_B3_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_B3_QID_BASE", TRQ_SEL_FMAP_B3_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap B3 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_b3_field_info with %d entries\n", sizeof(trq_sel_fmap_b3_field_info)/sizeof(trq_sel_fmap_b3_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap B4
static struct regfield_info trq_sel_fmap_b4_field_info[] = {
    {"TRQ_SEL_FMAP_B4_RSVD_1", TRQ_SEL_FMAP_B4_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_B4_QID_MAX", TRQ_SEL_FMAP_B4_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_B4_QID_BASE", TRQ_SEL_FMAP_B4_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap B4 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_b4_field_info with %d entries\n", sizeof(trq_sel_fmap_b4_field_info)/sizeof(trq_sel_fmap_b4_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap B5
static struct regfield_info trq_sel_fmap_b5_field_info[] = {
    {"TRQ_SEL_FMAP_B5_RSVD_1", TRQ_SEL_FMAP_B5_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_B5_QID_MAX", TRQ_SEL_FMAP_B5_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_B5_QID_BASE", TRQ_SEL_FMAP_B5_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap B5 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_b5_field_info with %d entries\n", sizeof(trq_sel_fmap_b5_field_info)/sizeof(trq_sel_fmap_b5_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap B6
static struct regfield_info trq_sel_fmap_b6_field_info[] = {
    {"TRQ_SEL_FMAP_B6_RSVD_1", TRQ_SEL_FMAP_B6_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_B6_QID_MAX", TRQ_SEL_FMAP_B6_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_B6_QID_BASE", TRQ_SEL_FMAP_B6_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap B6 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_b6_field_info with %d entries\n", sizeof(trq_sel_fmap_b6_field_info)/sizeof(trq_sel_fmap_b6_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap B7
static struct regfield_info trq_sel_fmap_b7_field_info[] = {
    {"TRQ_SEL_FMAP_B7_RSVD_1", TRQ_SEL_FMAP_B7_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_B7_QID_MAX", TRQ_SEL_FMAP_B7_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_B7_QID_BASE", TRQ_SEL_FMAP_B7_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap B7 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_b7_field_info with %d entries\n", sizeof(trq_sel_fmap_b7_field_info)/sizeof(trq_sel_fmap_b7_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap B8
static struct regfield_info trq_sel_fmap_b8_field_info[] = {
    {"TRQ_SEL_FMAP_B8_RSVD_1", TRQ_SEL_FMAP_B8_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_B8_QID_MAX", TRQ_SEL_FMAP_B8_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_B8_QID_BASE", TRQ_SEL_FMAP_B8_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap B8 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_b8_field_info with %d entries\n", sizeof(trq_sel_fmap_b8_field_info)/sizeof(trq_sel_fmap_b8_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap B9
static struct regfield_info trq_sel_fmap_b9_field_info[] = {
    {"TRQ_SEL_FMAP_B9_RSVD_1", TRQ_SEL_FMAP_B9_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_B9_QID_MAX", TRQ_SEL_FMAP_B9_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_B9_QID_BASE", TRQ_SEL_FMAP_B9_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap B9 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_b9_field_info with %d entries\n", sizeof(trq_sel_fmap_b9_field_info)/sizeof(trq_sel_fmap_b9_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap BA
static struct regfield_info trq_sel_fmap_ba_field_info[] = {
    {"TRQ_SEL_FMAP_BA_RSVD_1", TRQ_SEL_FMAP_BA_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_BA_QID_MAX", TRQ_SEL_FMAP_BA_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_BA_QID_BASE", TRQ_SEL_FMAP_BA_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap BA field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_ba_field_info with %d entries\n", sizeof(trq_sel_fmap_ba_field_info)/sizeof(trq_sel_fmap_ba_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap BB
static struct regfield_info trq_sel_fmap_bb_field_info[] = {
    {"TRQ_SEL_FMAP_BB_RSVD_1", TRQ_SEL_FMAP_BB_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_BB_QID_MAX", TRQ_SEL_FMAP_BB_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_BB_QID_BASE", TRQ_SEL_FMAP_BB_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap BB field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_bb_field_info with %d entries\n", sizeof(trq_sel_fmap_bb_field_info)/sizeof(trq_sel_fmap_bb_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap BC
static struct regfield_info trq_sel_fmap_bc_field_info[] = {
    {"TRQ_SEL_FMAP_BC_RSVD_1", TRQ_SEL_FMAP_BC_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_BC_QID_MAX", TRQ_SEL_FMAP_BC_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_BC_QID_BASE", TRQ_SEL_FMAP_BC_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap BC field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_bc_field_info with %d entries\n", sizeof(trq_sel_fmap_bc_field_info)/sizeof(trq_sel_fmap_bc_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap BD
static struct regfield_info trq_sel_fmap_bd_field_info[] = {
    {"TRQ_SEL_FMAP_BD_RSVD_1", TRQ_SEL_FMAP_BD_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_BD_QID_MAX", TRQ_SEL_FMAP_BD_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_BD_QID_BASE", TRQ_SEL_FMAP_BD_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap BD field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_bd_field_info with %d entries\n", sizeof(trq_sel_fmap_bd_field_info)/sizeof(trq_sel_fmap_bd_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap BE
static struct regfield_info trq_sel_fmap_be_field_info[] = {
    {"TRQ_SEL_FMAP_BE_RSVD_1", TRQ_SEL_FMAP_BE_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_BE_QID_MAX", TRQ_SEL_FMAP_BE_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_BE_QID_BASE", TRQ_SEL_FMAP_BE_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap BE field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_be_field_info with %d entries\n", sizeof(trq_sel_fmap_be_field_info)/sizeof(trq_sel_fmap_be_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap BF
static struct regfield_info trq_sel_fmap_bf_field_info[] = {
    {"TRQ_SEL_FMAP_BF_RSVD_1", TRQ_SEL_FMAP_BF_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_BF_QID_MAX", TRQ_SEL_FMAP_BF_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_BF_QID_BASE", TRQ_SEL_FMAP_BF_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap BF field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_bf_field_info with %d entries\n", sizeof(trq_sel_fmap_bf_field_info)/sizeof(trq_sel_fmap_bf_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap C0
static struct regfield_info trq_sel_fmap_c0_field_info[] = {
    {"TRQ_SEL_FMAP_C0_RSVD_1", TRQ_SEL_FMAP_C0_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_C0_QID_MAX", TRQ_SEL_FMAP_C0_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_C0_QID_BASE", TRQ_SEL_FMAP_C0_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap C0 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_c0_field_info with %d entries\n", sizeof(trq_sel_fmap_c0_field_info)/sizeof(trq_sel_fmap_c0_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap C1
static struct regfield_info trq_sel_fmap_c1_field_info[] = {
    {"TRQ_SEL_FMAP_C1_RSVD_1", TRQ_SEL_FMAP_C1_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_C1_QID_MAX", TRQ_SEL_FMAP_C1_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_C1_QID_BASE", TRQ_SEL_FMAP_C1_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap C1 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_c1_field_info with %d entries\n", sizeof(trq_sel_fmap_c1_field_info)/sizeof(trq_sel_fmap_c1_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap C2
static struct regfield_info trq_sel_fmap_c2_field_info[] = {
    {"TRQ_SEL_FMAP_C2_RSVD_1", TRQ_SEL_FMAP_C2_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_C2_QID_MAX", TRQ_SEL_FMAP_C2_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_C2_QID_BASE", TRQ_SEL_FMAP_C2_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap C2 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_c2_field_info with %d entries\n", sizeof(trq_sel_fmap_c2_field_info)/sizeof(trq_sel_fmap_c2_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap C3
static struct regfield_info trq_sel_fmap_c3_field_info[] = {
    {"TRQ_SEL_FMAP_C3_RSVD_1", TRQ_SEL_FMAP_C3_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_C3_QID_MAX", TRQ_SEL_FMAP_C3_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_C3_QID_BASE", TRQ_SEL_FMAP_C3_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap C3 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_c3_field_info with %d entries\n", sizeof(trq_sel_fmap_c3_field_info)/sizeof(trq_sel_fmap_c3_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap C4
static struct regfield_info trq_sel_fmap_c4_field_info[] = {
    {"TRQ_SEL_FMAP_C4_RSVD_1", TRQ_SEL_FMAP_C4_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_C4_QID_MAX", TRQ_SEL_FMAP_C4_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_C4_QID_BASE", TRQ_SEL_FMAP_C4_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap C4 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_c4_field_info with %d entries\n", sizeof(trq_sel_fmap_c4_field_info)/sizeof(trq_sel_fmap_c4_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap C5
static struct regfield_info trq_sel_fmap_c5_field_info[] = {
    {"TRQ_SEL_FMAP_C5_RSVD_1", TRQ_SEL_FMAP_C5_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_C5_QID_MAX", TRQ_SEL_FMAP_C5_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_C5_QID_BASE", TRQ_SEL_FMAP_C5_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap C5 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_c5_field_info with %d entries\n", sizeof(trq_sel_fmap_c5_field_info)/sizeof(trq_sel_fmap_c5_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap C6
static struct regfield_info trq_sel_fmap_c6_field_info[] = {
    {"TRQ_SEL_FMAP_C6_RSVD_1", TRQ_SEL_FMAP_C6_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_C6_QID_MAX", TRQ_SEL_FMAP_C6_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_C6_QID_BASE", TRQ_SEL_FMAP_C6_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap C6 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_c6_field_info with %d entries\n", sizeof(trq_sel_fmap_c6_field_info)/sizeof(trq_sel_fmap_c6_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap C7
static struct regfield_info trq_sel_fmap_c7_field_info[] = {
    {"TRQ_SEL_FMAP_C7_RSVD_1", TRQ_SEL_FMAP_C7_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_C7_QID_MAX", TRQ_SEL_FMAP_C7_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_C7_QID_BASE", TRQ_SEL_FMAP_C7_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap C7 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_c7_field_info with %d entries\n", sizeof(trq_sel_fmap_c7_field_info)/sizeof(trq_sel_fmap_c7_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap C8
static struct regfield_info trq_sel_fmap_c8_field_info[] = {
    {"TRQ_SEL_FMAP_C8_RSVD_1", TRQ_SEL_FMAP_C8_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_C8_QID_MAX", TRQ_SEL_FMAP_C8_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_C8_QID_BASE", TRQ_SEL_FMAP_C8_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap C8 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_c8_field_info with %d entries\n", sizeof(trq_sel_fmap_c8_field_info)/sizeof(trq_sel_fmap_c8_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap C9
static struct regfield_info trq_sel_fmap_c9_field_info[] = {
    {"TRQ_SEL_FMAP_C9_RSVD_1", TRQ_SEL_FMAP_C9_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_C9_QID_MAX", TRQ_SEL_FMAP_C9_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_C9_QID_BASE", TRQ_SEL_FMAP_C9_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap C9 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_c9_field_info with %d entries\n", sizeof(trq_sel_fmap_c9_field_info)/sizeof(trq_sel_fmap_c9_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap CA
static struct regfield_info trq_sel_fmap_ca_field_info[] = {
    {"TRQ_SEL_FMAP_CA_RSVD_1", TRQ_SEL_FMAP_CA_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_CA_QID_MAX", TRQ_SEL_FMAP_CA_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_CA_QID_BASE", TRQ_SEL_FMAP_CA_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap CA field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_ca_field_info with %d entries\n", sizeof(trq_sel_fmap_ca_field_info)/sizeof(trq_sel_fmap_ca_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap CB
static struct regfield_info trq_sel_fmap_cb_field_info[] = {
    {"TRQ_SEL_FMAP_CB_RSVD_1", TRQ_SEL_FMAP_CB_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_CB_QID_MAX", TRQ_SEL_FMAP_CB_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_CB_QID_BASE", TRQ_SEL_FMAP_CB_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap CB field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_cb_field_info with %d entries\n", sizeof(trq_sel_fmap_cb_field_info)/sizeof(trq_sel_fmap_cb_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap CC
static struct regfield_info trq_sel_fmap_cc_field_info[] = {
    {"TRQ_SEL_FMAP_CC_RSVD_1", TRQ_SEL_FMAP_CC_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_CC_QID_MAX", TRQ_SEL_FMAP_CC_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_CC_QID_BASE", TRQ_SEL_FMAP_CC_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap CC field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_cc_field_info with %d entries\n", sizeof(trq_sel_fmap_cc_field_info)/sizeof(trq_sel_fmap_cc_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap CD
static struct regfield_info trq_sel_fmap_cd_field_info[] = {
    {"TRQ_SEL_FMAP_CD_RSVD_1", TRQ_SEL_FMAP_CD_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_CD_QID_MAX", TRQ_SEL_FMAP_CD_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_CD_QID_BASE", TRQ_SEL_FMAP_CD_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap CD field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_cd_field_info with %d entries\n", sizeof(trq_sel_fmap_cd_field_info)/sizeof(trq_sel_fmap_cd_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap CE
static struct regfield_info trq_sel_fmap_ce_field_info[] = {
    {"TRQ_SEL_FMAP_CE_RSVD_1", TRQ_SEL_FMAP_CE_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_CE_QID_MAX", TRQ_SEL_FMAP_CE_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_CE_QID_BASE", TRQ_SEL_FMAP_CE_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap CE field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_ce_field_info with %d entries\n", sizeof(trq_sel_fmap_ce_field_info)/sizeof(trq_sel_fmap_ce_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap CF
static struct regfield_info trq_sel_fmap_cf_field_info[] = {
    {"TRQ_SEL_FMAP_CF_RSVD_1", TRQ_SEL_FMAP_CF_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_CF_QID_MAX", TRQ_SEL_FMAP_CF_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_CF_QID_BASE", TRQ_SEL_FMAP_CF_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap CF field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_cf_field_info with %d entries\n", sizeof(trq_sel_fmap_cf_field_info)/sizeof(trq_sel_fmap_cf_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap D0
static struct regfield_info trq_sel_fmap_d0_field_info[] = {
    {"TRQ_SEL_FMAP_D0_RSVD_1", TRQ_SEL_FMAP_D0_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_D0_QID_MAX", TRQ_SEL_FMAP_D0_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_D0_QID_BASE", TRQ_SEL_FMAP_D0_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap D0 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_d0_field_info with %d entries\n", sizeof(trq_sel_fmap_d0_field_info)/sizeof(trq_sel_fmap_d0_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap D1
static struct regfield_info trq_sel_fmap_d1_field_info[] = {
    {"TRQ_SEL_FMAP_D1_RSVD_1", TRQ_SEL_FMAP_D1_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_D1_QID_MAX", TRQ_SEL_FMAP_D1_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_D1_QID_BASE", TRQ_SEL_FMAP_D1_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap D1 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_d1_field_info with %d entries\n", sizeof(trq_sel_fmap_d1_field_info)/sizeof(trq_sel_fmap_d1_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap D2
static struct regfield_info trq_sel_fmap_d2_field_info[] = {
    {"TRQ_SEL_FMAP_D2_RSVD_1", TRQ_SEL_FMAP_D2_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_D2_QID_MAX", TRQ_SEL_FMAP_D2_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_D2_QID_BASE", TRQ_SEL_FMAP_D2_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap D2 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_d2_field_info with %d entries\n", sizeof(trq_sel_fmap_d2_field_info)/sizeof(trq_sel_fmap_d2_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap D3
static struct regfield_info trq_sel_fmap_d3_field_info[] = {
    {"TRQ_SEL_FMAP_D3_RSVD_1", TRQ_SEL_FMAP_D3_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_D3_QID_MAX", TRQ_SEL_FMAP_D3_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_D3_QID_BASE", TRQ_SEL_FMAP_D3_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap D3 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_d3_field_info with %d entries\n", sizeof(trq_sel_fmap_d3_field_info)/sizeof(trq_sel_fmap_d3_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap D4
static struct regfield_info trq_sel_fmap_d4_field_info[] = {
    {"TRQ_SEL_FMAP_D4_RSVD_1", TRQ_SEL_FMAP_D4_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_D4_QID_MAX", TRQ_SEL_FMAP_D4_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_D4_QID_BASE", TRQ_SEL_FMAP_D4_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap D4 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_d4_field_info with %d entries\n", sizeof(trq_sel_fmap_d4_field_info)/sizeof(trq_sel_fmap_d4_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap D5
static struct regfield_info trq_sel_fmap_d5_field_info[] = {
    {"TRQ_SEL_FMAP_D5_RSVD_1", TRQ_SEL_FMAP_D5_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_D5_QID_MAX", TRQ_SEL_FMAP_D5_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_D5_QID_BASE", TRQ_SEL_FMAP_D5_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap D5 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_d5_field_info with %d entries\n", sizeof(trq_sel_fmap_d5_field_info)/sizeof(trq_sel_fmap_d5_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap D6
static struct regfield_info trq_sel_fmap_d6_field_info[] = {
    {"TRQ_SEL_FMAP_D6_RSVD_1", TRQ_SEL_FMAP_D6_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_D6_QID_MAX", TRQ_SEL_FMAP_D6_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_D6_QID_BASE", TRQ_SEL_FMAP_D6_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap D6 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_d6_field_info with %d entries\n", sizeof(trq_sel_fmap_d6_field_info)/sizeof(trq_sel_fmap_d6_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap D7
static struct regfield_info trq_sel_fmap_d7_field_info[] = {
    {"TRQ_SEL_FMAP_D7_RSVD_1", TRQ_SEL_FMAP_D7_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_D7_QID_MAX", TRQ_SEL_FMAP_D7_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_D7_QID_BASE", TRQ_SEL_FMAP_D7_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap D7 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_d7_field_info with %d entries\n", sizeof(trq_sel_fmap_d7_field_info)/sizeof(trq_sel_fmap_d7_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap D8
static struct regfield_info trq_sel_fmap_d8_field_info[] = {
    {"TRQ_SEL_FMAP_D8_RSVD_1", TRQ_SEL_FMAP_D8_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_D8_QID_MAX", TRQ_SEL_FMAP_D8_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_D8_QID_BASE", TRQ_SEL_FMAP_D8_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap D8 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_d8_field_info with %d entries\n", sizeof(trq_sel_fmap_d8_field_info)/sizeof(trq_sel_fmap_d8_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap D9
static struct regfield_info trq_sel_fmap_d9_field_info[] = {
    {"TRQ_SEL_FMAP_D9_RSVD_1", TRQ_SEL_FMAP_D9_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_D9_QID_MAX", TRQ_SEL_FMAP_D9_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_D9_QID_BASE", TRQ_SEL_FMAP_D9_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap D9 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_d9_field_info with %d entries\n", sizeof(trq_sel_fmap_d9_field_info)/sizeof(trq_sel_fmap_d9_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap DA
static struct regfield_info trq_sel_fmap_da_field_info[] = {
    {"TRQ_SEL_FMAP_DA_RSVD_1", TRQ_SEL_FMAP_DA_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_DA_QID_MAX", TRQ_SEL_FMAP_DA_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_DA_QID_BASE", TRQ_SEL_FMAP_DA_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap DA field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_da_field_info with %d entries\n", sizeof(trq_sel_fmap_da_field_info)/sizeof(trq_sel_fmap_da_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap DB
static struct regfield_info trq_sel_fmap_db_field_info[] = {
    {"TRQ_SEL_FMAP_DB_RSVD_1", TRQ_SEL_FMAP_DB_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_DB_QID_MAX", TRQ_SEL_FMAP_DB_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_DB_QID_BASE", TRQ_SEL_FMAP_DB_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap DB field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_db_field_info with %d entries\n", sizeof(trq_sel_fmap_db_field_info)/sizeof(trq_sel_fmap_db_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap DC
static struct regfield_info trq_sel_fmap_dc_field_info[] = {
    {"TRQ_SEL_FMAP_DC_RSVD_1", TRQ_SEL_FMAP_DC_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_DC_QID_MAX", TRQ_SEL_FMAP_DC_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_DC_QID_BASE", TRQ_SEL_FMAP_DC_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap DC field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_dc_field_info with %d entries\n", sizeof(trq_sel_fmap_dc_field_info)/sizeof(trq_sel_fmap_dc_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap DD
static struct regfield_info trq_sel_fmap_dd_field_info[] = {
    {"TRQ_SEL_FMAP_DD_RSVD_1", TRQ_SEL_FMAP_DD_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_DD_QID_MAX", TRQ_SEL_FMAP_DD_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_DD_QID_BASE", TRQ_SEL_FMAP_DD_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap DD field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_dd_field_info with %d entries\n", sizeof(trq_sel_fmap_dd_field_info)/sizeof(trq_sel_fmap_dd_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap DE
static struct regfield_info trq_sel_fmap_de_field_info[] = {
    {"TRQ_SEL_FMAP_DE_RSVD_1", TRQ_SEL_FMAP_DE_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_DE_QID_MAX", TRQ_SEL_FMAP_DE_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_DE_QID_BASE", TRQ_SEL_FMAP_DE_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap DE field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_de_field_info with %d entries\n", sizeof(trq_sel_fmap_de_field_info)/sizeof(trq_sel_fmap_de_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap DF
static struct regfield_info trq_sel_fmap_df_field_info[] = {
    {"TRQ_SEL_FMAP_DF_RSVD_1", TRQ_SEL_FMAP_DF_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_DF_QID_MAX", TRQ_SEL_FMAP_DF_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_DF_QID_BASE", TRQ_SEL_FMAP_DF_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap DF field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_df_field_info with %d entries\n", sizeof(trq_sel_fmap_df_field_info)/sizeof(trq_sel_fmap_df_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap E0
static struct regfield_info trq_sel_fmap_e0_field_info[] = {
    {"TRQ_SEL_FMAP_E0_RSVD_1", TRQ_SEL_FMAP_E0_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_E0_QID_MAX", TRQ_SEL_FMAP_E0_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_E0_QID_BASE", TRQ_SEL_FMAP_E0_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap E0 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_e0_field_info with %d entries\n", sizeof(trq_sel_fmap_e0_field_info)/sizeof(trq_sel_fmap_e0_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap E1
static struct regfield_info trq_sel_fmap_e1_field_info[] = {
    {"TRQ_SEL_FMAP_E1_RSVD_1", TRQ_SEL_FMAP_E1_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_E1_QID_MAX", TRQ_SEL_FMAP_E1_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_E1_QID_BASE", TRQ_SEL_FMAP_E1_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap E1 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_e1_field_info with %d entries\n", sizeof(trq_sel_fmap_e1_field_info)/sizeof(trq_sel_fmap_e1_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap E2
static struct regfield_info trq_sel_fmap_e2_field_info[] = {
    {"TRQ_SEL_FMAP_E2_RSVD_1", TRQ_SEL_FMAP_E2_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_E2_QID_MAX", TRQ_SEL_FMAP_E2_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_E2_QID_BASE", TRQ_SEL_FMAP_E2_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap E2 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_e2_field_info with %d entries\n", sizeof(trq_sel_fmap_e2_field_info)/sizeof(trq_sel_fmap_e2_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap E3
static struct regfield_info trq_sel_fmap_e3_field_info[] = {
    {"TRQ_SEL_FMAP_E3_RSVD_1", TRQ_SEL_FMAP_E3_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_E3_QID_MAX", TRQ_SEL_FMAP_E3_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_E3_QID_BASE", TRQ_SEL_FMAP_E3_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap E3 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_e3_field_info with %d entries\n", sizeof(trq_sel_fmap_e3_field_info)/sizeof(trq_sel_fmap_e3_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap E4
static struct regfield_info trq_sel_fmap_e4_field_info[] = {
    {"TRQ_SEL_FMAP_E4_RSVD_1", TRQ_SEL_FMAP_E4_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_E4_QID_MAX", TRQ_SEL_FMAP_E4_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_E4_QID_BASE", TRQ_SEL_FMAP_E4_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap E4 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_e4_field_info with %d entries\n", sizeof(trq_sel_fmap_e4_field_info)/sizeof(trq_sel_fmap_e4_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap E5
static struct regfield_info trq_sel_fmap_e5_field_info[] = {
    {"TRQ_SEL_FMAP_E5_RSVD_1", TRQ_SEL_FMAP_E5_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_E5_QID_MAX", TRQ_SEL_FMAP_E5_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_E5_QID_BASE", TRQ_SEL_FMAP_E5_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap E5 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_e5_field_info with %d entries\n", sizeof(trq_sel_fmap_e5_field_info)/sizeof(trq_sel_fmap_e5_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap E6
static struct regfield_info trq_sel_fmap_e6_field_info[] = {
    {"TRQ_SEL_FMAP_E6_RSVD_1", TRQ_SEL_FMAP_E6_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_E6_QID_MAX", TRQ_SEL_FMAP_E6_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_E6_QID_BASE", TRQ_SEL_FMAP_E6_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap E6 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_e6_field_info with %d entries\n", sizeof(trq_sel_fmap_e6_field_info)/sizeof(trq_sel_fmap_e6_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap E7
static struct regfield_info trq_sel_fmap_e7_field_info[] = {
    {"TRQ_SEL_FMAP_E7_RSVD_1", TRQ_SEL_FMAP_E7_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_E7_QID_MAX", TRQ_SEL_FMAP_E7_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_E7_QID_BASE", TRQ_SEL_FMAP_E7_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap E7 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_e7_field_info with %d entries\n", sizeof(trq_sel_fmap_e7_field_info)/sizeof(trq_sel_fmap_e7_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap E8
static struct regfield_info trq_sel_fmap_e8_field_info[] = {
    {"TRQ_SEL_FMAP_E8_RSVD_1", TRQ_SEL_FMAP_E8_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_E8_QID_MAX", TRQ_SEL_FMAP_E8_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_E8_QID_BASE", TRQ_SEL_FMAP_E8_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap E8 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_e8_field_info with %d entries\n", sizeof(trq_sel_fmap_e8_field_info)/sizeof(trq_sel_fmap_e8_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap E9
static struct regfield_info trq_sel_fmap_e9_field_info[] = {
    {"TRQ_SEL_FMAP_E9_RSVD_1", TRQ_SEL_FMAP_E9_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_E9_QID_MAX", TRQ_SEL_FMAP_E9_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_E9_QID_BASE", TRQ_SEL_FMAP_E9_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap E9 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_e9_field_info with %d entries\n", sizeof(trq_sel_fmap_e9_field_info)/sizeof(trq_sel_fmap_e9_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap EA
static struct regfield_info trq_sel_fmap_ea_field_info[] = {
    {"TRQ_SEL_FMAP_EA_RSVD_1", TRQ_SEL_FMAP_EA_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_EA_QID_MAX", TRQ_SEL_FMAP_EA_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_EA_QID_BASE", TRQ_SEL_FMAP_EA_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap EA field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_ea_field_info with %d entries\n", sizeof(trq_sel_fmap_ea_field_info)/sizeof(trq_sel_fmap_ea_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap EB
static struct regfield_info trq_sel_fmap_eb_field_info[] = {
    {"TRQ_SEL_FMAP_EB_RSVD_1", TRQ_SEL_FMAP_EB_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_EB_QID_MAX", TRQ_SEL_FMAP_EB_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_EB_QID_BASE", TRQ_SEL_FMAP_EB_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap EB field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_eb_field_info with %d entries\n", sizeof(trq_sel_fmap_eb_field_info)/sizeof(trq_sel_fmap_eb_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap EC
static struct regfield_info trq_sel_fmap_ec_field_info[] = {
    {"TRQ_SEL_FMAP_EC_RSVD_1", TRQ_SEL_FMAP_EC_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_EC_QID_MAX", TRQ_SEL_FMAP_EC_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_EC_QID_BASE", TRQ_SEL_FMAP_EC_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap EC field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_ec_field_info with %d entries\n", sizeof(trq_sel_fmap_ec_field_info)/sizeof(trq_sel_fmap_ec_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap ED
static struct regfield_info trq_sel_fmap_ed_field_info[] = {
    {"TRQ_SEL_FMAP_ED_RSVD_1", TRQ_SEL_FMAP_ED_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_ED_QID_MAX", TRQ_SEL_FMAP_ED_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_ED_QID_BASE", TRQ_SEL_FMAP_ED_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap ED field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_ed_field_info with %d entries\n", sizeof(trq_sel_fmap_ed_field_info)/sizeof(trq_sel_fmap_ed_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap EE
static struct regfield_info trq_sel_fmap_ee_field_info[] = {
    {"TRQ_SEL_FMAP_EE_RSVD_1", TRQ_SEL_FMAP_EE_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_EE_QID_MAX", TRQ_SEL_FMAP_EE_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_EE_QID_BASE", TRQ_SEL_FMAP_EE_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap EE field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_ee_field_info with %d entries\n", sizeof(trq_sel_fmap_ee_field_info)/sizeof(trq_sel_fmap_ee_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap EF
static struct regfield_info trq_sel_fmap_ef_field_info[] = {
    {"TRQ_SEL_FMAP_EF_RSVD_1", TRQ_SEL_FMAP_EF_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_EF_QID_MAX", TRQ_SEL_FMAP_EF_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_EF_QID_BASE", TRQ_SEL_FMAP_EF_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap EF field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_ef_field_info with %d entries\n", sizeof(trq_sel_fmap_ef_field_info)/sizeof(trq_sel_fmap_ef_field_info[0]));
};

// MD : Structure to hold register field information for TRQ select fmap F0
static struct regfield_info trq_sel_fmap_f0_field_info[] = {
    {"TRQ_SEL_FMAP_F0_RSVD_1", TRQ_SEL_FMAP_F0_RSVD_1_MASK},
    {"TRQ_SEL_FMAP_F0_QID_MAX", TRQ_SEL_FMAP_F0_QID_MAX_MASK},
    {"TRQ_SEL_FMAP_F0_QID_BASE", TRQ_SEL_FMAP_F0_QID_BASE_MASK},
    // MD : Debug: Track initialization of TRQ select fmap F0 field info
    printk(KERN_DEBUG "Initialized trq_sel_fmap_f0_field_info with %d entries\n", sizeof(trq_sel_fmap_f0_field_info)/sizeof(trq_sel_fmap_f0_field_info[0]));
};

// MD : Structure to hold register field information for IND CTXT data 3
static struct regfield_info ind_ctxt_data_3_field_info[] = {
    {"IND_CTXT_DATA_3_DATA", IND_CTXT_DATA_3_DATA_MASK},
    // MD : Debug: Track initialization of IND CTXT data 3 field info
    printk(KERN_DEBUG "Initialized ind_ctxt_data_3_field_info with %d entries\n", sizeof(ind_ctxt_data_3_field_info)/sizeof(ind_ctxt_data_3_field_info[0]));
};

// MD : Structure to hold register field information for IND CTXT data 2
static struct regfield_info ind_ctxt_data_2_field_info[] = {
    {"IND_CTXT_DATA_2_DATA", IND_CTXT_DATA_2_DATA_MASK},
    // MD : Debug: Track initialization of IND CTXT data 2 field info
    printk(KERN_DEBUG "Initialized ind_ctxt_data_2_field_info with %d entries\n", sizeof(ind_ctxt_data_2_field_info)/sizeof(ind_ctxt_data_2_field_info[0]));
};

// MD : Structure to hold register field information for IND CTXT data 1
static struct regfield_info ind_ctxt_data_1_field_info[] = {
    {"IND_CTXT_DATA_1_DATA", IND_CTXT_DATA_1_DATA_MASK},
    // MD : Debug: Track initialization of IND CTXT data 1 field info
    printk(KERN_DEBUG "Initialized ind_ctxt_data_1_field_info with %d entries\n", sizeof(ind_ctxt_data_1_field_info)/sizeof(ind_ctxt_data_1_field_info[0]));
};

// MD : Structure to hold register field information for IND CTXT data 0
static struct regfield_info ind_ctxt_data_0_field_info[] = {
    {"IND_CTXT_DATA_0_DATA", IND_CTXT_DATA_0_DATA_MASK},
    // MD : Debug: Track initialization of IND CTXT data 0 field info
    printk(KERN_DEBUG "Initialized ind_ctxt_data_0_field_info with %d entries\n", sizeof(ind_ctxt_data_0_field_info)/sizeof(ind_ctxt_data_0_field_info[0]));
};

// MD : Structure to hold register field information for IND CTXT 3
static struct regfield_info ind_ctxt3_field_info[] = {
    {"IND_CTXT3", IND_CTXT3_MASK},
    // MD : Debug: Track initialization of IND CTXT 3 field info
    printk(KERN_DEBUG "Initialized ind_ctxt3_field_info with %d entries\n", sizeof(ind_ctxt3_field_info)/sizeof(ind_ctxt3_field_info[0]));
};

// MD : Structure to hold register field information for IND CTXT 2
static struct regfield_info ind_ctxt2_field_info[] = {
    {"IND_CTXT2", IND_CTXT2_MASK},
    // MD : Debug: Track initialization of IND CTXT 2 field info
    printk(KERN_DEBUG "Initialized ind_ctxt2_field_info with %d entries\n", sizeof(ind_ctxt2_field_info)/sizeof(ind_ctxt2_field_info[0]));
};

// MD : Structure to hold register field information for IND CTXT 1
static struct regfield_info ind_ctxt1_field_info[] = {
    {"IND_CTXT1", IND_CTXT1_MASK},
    // MD : Debug: Track initialization of IND CTXT 1 field info
    printk(KERN_DEBUG "Initialized ind_ctxt1_field_info with %d entries\n", sizeof(ind_ctxt1_field_info)/sizeof(ind_ctxt1_field_info[0]));
};

// MD : Structure to hold register field information for IND CTXT 0
static struct regfield_info ind_ctxt0_field_info[] = {
    {"IND_CTXT0", IND_CTXT0_MASK},
    // MD : Debug: Track initialization of IND CTXT 0 field info
    printk(KERN_DEBUG "Initialized ind_ctxt0_field_info with %d entries\n", sizeof(ind_ctxt0_field_info)/sizeof(ind_ctxt0_field_info[0]));
};

// MD : Structure to hold register field information for IND CTXT CMD
static struct regfield_info ind_ctxt_cmd_field_info[] = {
    {"IND_CTXT_CMD_RSVD_1", IND_CTXT_CMD_RSVD_1_MASK},
    {"IND_CTXT_CMD_QID", IND_CTXT_CMD_QID_MASK},
    {"IND_CTXT_CMD_OP", IND_CTXT_CMD_OP_MASK},
    {"IND_CTXT_CMD_SET", IND_CTXT_CMD_SET_MASK},
    {"IND_CTXT_CMD_BUSY", IND_CTXT_CMD_BUSY_MASK},
    // MD : Debug: Track initialization of IND CTXT CMD field info
    printk(KERN_DEBUG "Initialized ind_ctxt_cmd_field_info with %d entries\n", sizeof(ind_ctxt_cmd_field_info)/sizeof(ind_ctxt_cmd_field_info[0]));
};

// MD : Structure to hold register field information for C2H Timer Count 1
static struct regfield_info c2h_timer_cnt_1_field_info[] = {
    {"C2H_TIMER_CNT_1_RSVD_1", C2H_TIMER_CNT_1_RSVD_1_MASK},
    {"C2H_TIMER_CNT_1", C2H_TIMER_CNT_1_MASK},
    // MD : Debug: Track initialization of C2H Timer Count 1 field info
    printk(KERN_DEBUG "Initialized c2h_timer_cnt_1_field_info with %d entries\n", sizeof(c2h_timer_cnt_1_field_info)/sizeof(c2h_timer_cnt_1_field_info[0]));
};

// MD : Structure to hold register field information for C2H Timer Count 2
static struct regfield_info c2h_timer_cnt_2_field_info[] = {
    {"C2H_TIMER_CNT_2_RSVD_1", C2H_TIMER_CNT_2_RSVD_1_MASK},
    {"C2H_TIMER_CNT_2", C2H_TIMER_CNT_2_MASK},
    // MD : Debug: Track initialization of C2H Timer Count 2 field info
    printk(KERN_DEBUG "Initialized c2h_timer_cnt_2_field_info with %d entries\n", sizeof(c2h_timer_cnt_2_field_info)/sizeof(c2h_timer_cnt_2_field_info[0]));
};

// MD : Structure to hold register field information for C2H Timer Count 3
static struct regfield_info c2h_timer_cnt_3_field_info[] = {
    {"C2H_TIMER_CNT_3_RSVD_1", C2H_TIMER_CNT_3_RSVD_1_MASK},
    {"C2H_TIMER_CNT_3", C2H_TIMER_CNT_3_MASK},
    // MD : Debug: Track initialization of C2H Timer Count 3 field info
    printk(KERN_DEBUG "Initialized c2h_timer_cnt_3_field_info with %d entries\n", sizeof(c2h_timer_cnt_3_field_info)/sizeof(c2h_timer_cnt_3_field_info[0]));
};

// MD : Structure to hold register field information for C2H Timer Count 4
static struct regfield_info c2h_timer_cnt_4_field_info[] = {
    {"C2H_TIMER_CNT_4_RSVD_1", C2H_TIMER_CNT_4_RSVD_1_MASK},
    {"C2H_TIMER_CNT_4", C2H_TIMER_CNT_4_MASK},
    // MD : Debug: Track initialization of C2H Timer Count 4 field info
    printk(KERN_DEBUG "Initialized c2h_timer_cnt_4_field_info with %d entries\n", sizeof(c2h_timer_cnt_4_field_info)/sizeof(c2h_timer_cnt_4_field_info[0]));
};

// MD : Structure to hold register field information for C2H Timer Count 5
static struct regfield_info c2h_timer_cnt_5_field_info[] = {
    {"C2H_TIMER_CNT_5_RSVD_1", C2H_TIMER_CNT_5_RSVD_1_MASK},
    {"C2H_TIMER_CNT_5", C2H_TIMER_CNT_5_MASK},
    // MD : Debug: Track initialization of C2H Timer Count 5 field info
    printk(KERN_DEBUG "Initialized c2h_timer_cnt_5_field_info with %d entries\n", sizeof(c2h_timer_cnt_5_field_info)/sizeof(c2h_timer_cnt_5_field_info[0]));
};

// MD : Structure to hold register field information for C2H Timer Count 6
static struct regfield_info c2h_timer_cnt_6_field_info[] = {
    {"C2H_TIMER_CNT_6_RSVD_1", C2H_TIMER_CNT_6_RSVD_1_MASK},
    {"C2H_TIMER_CNT_6", C2H_TIMER_CNT_6_MASK},
    // MD : Debug: Track initialization of C2H Timer Count 6 field info
    printk(KERN_DEBUG "Initialized c2h_timer_cnt_6_field_info with %d entries\n", sizeof(c2h_timer_cnt_6_field_info)/sizeof(c2h_timer_cnt_6_field_info[0]));
};

// MD : Structure to hold register field information for C2H Timer Count 7
static struct regfield_info c2h_timer_cnt_7_field_info[] = {
    {"C2H_TIMER_CNT_7_RSVD_1", C2H_TIMER_CNT_7_RSVD_1_MASK},
    {"C2H_TIMER_CNT_7", C2H_TIMER_CNT_7_MASK},
    // MD : Debug: Track initialization of C2H Timer Count 7 field info
    printk(KERN_DEBUG "Initialized c2h_timer_cnt_7_field_info with %d entries\n", sizeof(c2h_timer_cnt_7_field_info)/sizeof(c2h_timer_cnt_7_field_info[0]));
};

// MD : Structure to hold register field information for C2H Timer Count 8
static struct regfield_info c2h_timer_cnt_8_field_info[] = {
    {"C2H_TIMER_CNT_8_RSVD_1", C2H_TIMER_CNT_8_RSVD_1_MASK},
    {"C2H_TIMER_CNT_8", C2H_TIMER_CNT_8_MASK},
    // MD : Debug: Track initialization of C2H Timer Count 8 field info
    printk(KERN_DEBUG "Initialized c2h_timer_cnt_8_field_info with %d entries\n", sizeof(c2h_timer_cnt_8_field_info)/sizeof(c2h_timer_cnt_8_field_info[0]));
};

// MD : Structure to hold register field information for C2H Timer Count 9
static struct regfield_info c2h_timer_cnt_9_field_info[] = {
    {"C2H_TIMER_CNT_9_RSVD_1", C2H_TIMER_CNT_9_RSVD_1_MASK},
    {"C2H_TIMER_CNT_9", C2H_TIMER_CNT_9_MASK},
    // MD : Debug: Track initialization of C2H Timer Count 9 field info
    printk(KERN_DEBUG "Initialized c2h_timer_cnt_9_field_info with %d entries\n", sizeof(c2h_timer_cnt_9_field_info)/sizeof(c2h_timer_cnt_9_field_info[0]));
};

// MD : Structure to hold register field information for C2H timer count A
static struct regfield_info c2h_timer_cnt_a_field_info[] = {
    {"C2H_TIMER_CNT_A_RSVD_1", C2H_TIMER_CNT_A_RSVD_1_MASK},
    {"C2H_TIMER_CNT_A", C2H_TIMER_CNT_A_MASK},
    // MD : Debug: Track initialization of C2H timer count A field info
    printk(KERN_DEBUG "Initialized c2h_timer_cnt_a_field_info with %d entries\n", sizeof(c2h_timer_cnt_a_field_info)/sizeof(c2h_timer_cnt_a_field_info[0]));
};

// MD : Structure to hold register field information for C2H timer count B
static struct regfield_info c2h_timer_cnt_b_field_info[] = {
    {"C2H_TIMER_CNT_B_RSVD_1", C2H_TIMER_CNT_B_RSVD_1_MASK},
    {"C2H_TIMER_CNT_B", C2H_TIMER_CNT_B_MASK},
    // MD : Debug: Track initialization of C2H timer count B field info
    printk(KERN_DEBUG "Initialized c2h_timer_cnt_b_field_info with %d entries\n", sizeof(c2h_timer_cnt_b_field_info)/sizeof(c2h_timer_cnt_b_field_info[0]));
};

// MD : Structure to hold register field information for C2H timer count C
static struct regfield_info c2h_timer_cnt_c_field_info[] = {
    {"C2H_TIMER_CNT_C_RSVD_1", C2H_TIMER_CNT_C_RSVD_1_MASK},
    {"C2H_TIMER_CNT_C", C2H_TIMER_CNT_C_MASK},
    // MD : Debug: Track initialization of C2H timer count C field info
    printk(KERN_DEBUG "Initialized c2h_timer_cnt_c_field_info with %d entries\n", sizeof(c2h_timer_cnt_c_field_info)/sizeof(c2h_timer_cnt_c_field_info[0]));
};

// MD : Structure to hold register field information for C2H timer count D
static struct regfield_info c2h_timer_cnt_d_field_info[] = {
    {"C2H_TIMER_CNT_D_RSVD_1", C2H_TIMER_CNT_D_RSVD_1_MASK},
    {"C2H_TIMER_CNT_D", C2H_TIMER_CNT_D_MASK},
    // MD : Debug: Track initialization of C2H timer count D field info
    printk(KERN_DEBUG "Initialized c2h_timer_cnt_d_field_info with %d entries\n", sizeof(c2h_timer_cnt_d_field_info)/sizeof(c2h_timer_cnt_d_field_info[0]));
};

// MD : Structure to hold register field information for C2H timer count E
static struct regfield_info c2h_timer_cnt_e_field_info[] = {
    {"C2H_TIMER_CNT_E_RSVD_1", C2H_TIMER_CNT_E_RSVD_1_MASK},
    {"C2H_TIMER_CNT_E", C2H_TIMER_CNT_E_MASK},
    // MD : Debug: Track initialization of C2H timer count E field info
    printk(KERN_DEBUG "Initialized c2h_timer_cnt_e_field_info with %d entries\n", sizeof(c2h_timer_cnt_e_field_info)/sizeof(c2h_timer_cnt_e_field_info[0]));
};

// MD : Structure to hold register field information for C2H timer count F
static struct regfield_info c2h_timer_cnt_f_field_info[] = {
    {"C2H_TIMER_CNT_F_RSVD_1", C2H_TIMER_CNT_F_RSVD_1_MASK},
    {"C2H_TIMER_CNT_F", C2H_TIMER_CNT_F_MASK},
    // MD : Debug: Track initialization of C2H timer count F field info
    printk(KERN_DEBUG "Initialized c2h_timer_cnt_f_field_info with %d entries\n", sizeof(c2h_timer_cnt_f_field_info)/sizeof(c2h_timer_cnt_f_field_info[0]));
};

// MD : Structure to hold register field information for C2H timer count 10
static struct regfield_info c2h_timer_cnt_10_field_info[] = {
    {"C2H_TIMER_CNT_10_RSVD_1", C2H_TIMER_CNT_10_RSVD_1_MASK},
    {"C2H_TIMER_CNT_10", C2H_TIMER_CNT_10_MASK},
    // MD : Debug: Track initialization of C2H timer count 10 field info
    printk(KERN_DEBUG "Initialized c2h_timer_cnt_10_field_info with %d entries\n", sizeof(c2h_timer_cnt_10_field_info)/sizeof(c2h_timer_cnt_10_field_info[0]));
};

// MD : Structure to hold register field information for C2H count threshold 1
static struct regfield_info c2h_cnt_th_1_field_info[] = {
    {"C2H_CNT_TH_1_RSVD_1", C2H_CNT_TH_1_RSVD_1_MASK},
    {"C2H_CNT_TH_1_THESHOLD_CNT", C2H_CNT_TH_1_THESHOLD_CNT_MASK},
    // MD : Debug: Track initialization of C2H count threshold 1 field info
    printk(KERN_DEBUG "Initialized c2h_cnt_th_1_field_info with %d entries\n", sizeof(c2h_cnt_th_1_field_info)/sizeof(c2h_cnt_th_1_field_info[0]));
};

// MD : Structure to hold register field information for C2H count threshold 2
static struct regfield_info c2h_cnt_th_2_field_info[] = {
    {"C2H_CNT_TH_2_RSVD_1", C2H_CNT_TH_2_RSVD_1_MASK},
    {"C2H_CNT_TH_2_THESHOLD_CNT", C2H_CNT_TH_2_THESHOLD_CNT_MASK},
    // MD : Debug: Track initialization of C2H count threshold 2 field info
    printk(KERN_DEBUG "Initialized c2h_cnt_th_2_field_info with %d entries\n", sizeof(c2h_cnt_th_2_field_info)/sizeof(c2h_cnt_th_2_field_info[0]));
};

// MD : Structure to hold register field information for C2H count threshold 3
static struct regfield_info c2h_cnt_th_3_field_info[] = {
    {"C2H_CNT_TH_3_RSVD_1", C2H_CNT_TH_3_RSVD_1_MASK},
    {"C2H_CNT_TH_3_THESHOLD_CNT", C2H_CNT_TH_3_THESHOLD_CNT_MASK},
    // MD : Debug: Track initialization of C2H count threshold 3 field info
    printk(KERN_DEBUG "Initialized c2h_cnt_th_3_field_info with %d entries\n", sizeof(c2h_cnt_th_3_field_info)/sizeof(c2h_cnt_th_3_field_info[0]));
};

// MD : Structure to hold register field information for C2H count threshold 4
static struct regfield_info c2h_cnt_th_4_field_info[] = {
    {"C2H_CNT_TH_4_RSVD_1", C2H_CNT_TH_4_RSVD_1_MASK},
    {"C2H_CNT_TH_4_THESHOLD_CNT", C2H_CNT_TH_4_THESHOLD_CNT_MASK},
    // MD : Debug: Track initialization of C2H count threshold 4 field info
    printk(KERN_DEBUG "Initialized c2h_cnt_th_4_field_info with %d entries\n", sizeof(c2h_cnt_th_4_field_info)/sizeof(c2h_cnt_th_4_field_info[0]));
};

// MD : Structure to hold register field information for C2H count threshold 5
static struct regfield_info c2h_cnt_th_5_field_info[] = {
    {"C2H_CNT_TH_5_RSVD_1", C2H_CNT_TH_5_RSVD_1_MASK},
    {"C2H_CNT_TH_5_THESHOLD_CNT", C2H_CNT_TH_5_THESHOLD_CNT_MASK},
    // MD : Debug: Track initialization of C2H count threshold 5 field info
    printk(KERN_DEBUG "Initialized c2h_cnt_th_5_field_info with %d entries\n", sizeof(c2h_cnt_th_5_field_info)/sizeof(c2h_cnt_th_5_field_info[0]));
};

// MD : Structure to hold register field information for C2H count threshold 6
static struct regfield_info c2h_cnt_th_6_field_info[] = {
    {"C2H_CNT_TH_6_RSVD_1", C2H_CNT_TH_6_RSVD_1_MASK},
    {"C2H_CNT_TH_6_THESHOLD_CNT", C2H_CNT_TH_6_THESHOLD_CNT_MASK},
    // MD : Debug: Track initialization of C2H count threshold 6 field info
    printk(KERN_DEBUG "Initialized c2h_cnt_th_6_field_info with %d entries\n", sizeof(c2h_cnt_th_6_field_info)/sizeof(c2h_cnt_th_6_field_info[0]));
};

// MD : Structure to hold register field information for C2H count threshold 7
static struct regfield_info c2h_cnt_th_7_field_info[] = {
    {"C2H_CNT_TH_7_RSVD_1", C2H_CNT_TH_7_RSVD_1_MASK},
    {"C2H_CNT_TH_7_THESHOLD_CNT", C2H_CNT_TH_7_THESHOLD_CNT_MASK},
    // MD : Debug: Track initialization of C2H count threshold 7 field info
    printk(KERN_DEBUG "Initialized c2h_cnt_th_7_field_info with %d entries\n", sizeof(c2h_cnt_th_7_field_info)/sizeof(c2h_cnt_th_7_field_info[0]));
};

// MD : Structure to hold register field information for C2H count threshold 8
static struct regfield_info c2h_cnt_th_8_field_info[] = {
    {"C2H_CNT_TH_8_RSVD_1", C2H_CNT_TH_8_RSVD_1_MASK},
    {"C2H_CNT_TH_8_THESHOLD_CNT", C2H_CNT_TH_8_THESHOLD_CNT_MASK},
    // MD : Debug: Track initialization of C2H count threshold 8 field info
    printk(KERN_DEBUG "Initialized c2h_cnt_th_8_field_info with %d entries\n", sizeof(c2h_cnt_th_8_field_info)/sizeof(c2h_cnt_th_8_field_info[0]));
};

// MD : Structure to hold register field information for C2H count threshold 9
static struct regfield_info c2h_cnt_th_9_field_info[] = {
    {"C2H_CNT_TH_9_RSVD_1", C2H_CNT_TH_9_RSVD_1_MASK},
    {"C2H_CNT_TH_9_THESHOLD_CNT", C2H_CNT_TH_9_THESHOLD_CNT_MASK},
    // MD : Debug: Track initialization of C2H count threshold 9 field info
    printk(KERN_DEBUG "Initialized c2h_cnt_th_9_field_info with %d entries\n", sizeof(c2h_cnt_th_9_field_info)/sizeof(c2h_cnt_th_9_field_info[0]));
};

// MD : Structure to hold register field information for C2H count threshold A
static struct regfield_info c2h_cnt_th_a_field_info[] = {
    {"C2H_CNT_TH_A_RSVD_1", C2H_CNT_TH_A_RSVD_1_MASK},
    {"C2H_CNT_TH_A_THESHOLD_CNT", C2H_CNT_TH_A_THESHOLD_CNT_MASK},
    // MD : Debug: Track initialization of C2H count threshold A field info
    printk(KERN_DEBUG "Initialized c2h_cnt_th_a_field_info with %d entries\n", sizeof(c2h_cnt_th_a_field_info)/sizeof(c2h_cnt_th_a_field_info[0]));
};

// MD : Structure to hold register field information for C2H count threshold B
static struct regfield_info c2h_cnt_th_b_field_info[] = {
    {"C2H_CNT_TH_B_RSVD_1", C2H_CNT_TH_B_RSVD_1_MASK},
    {"C2H_CNT_TH_B_THESHOLD_CNT", C2H_CNT_TH_B_THESHOLD_CNT_MASK},
    // MD : Debug: Track initialization of C2H count threshold B field info
    printk(KERN_DEBUG "Initialized c2h_cnt_th_b_field_info with %d entries\n", sizeof(c2h_cnt_th_b_field_info)/sizeof(c2h_cnt_th_b_field_info[0]));
};

// MD : Structure to hold register field information for C2H count threshold C
static struct regfield_info c2h_cnt_th_c_field_info[] = {
    {"C2H_CNT_TH_C_RSVD_1", C2H_CNT_TH_C_RSVD_1_MASK},
    {"C2H_CNT_TH_C_THESHOLD_CNT", C2H_CNT_TH_C_THESHOLD_CNT_MASK},
    // MD : Debug: Track initialization of C2H count threshold C field info
    printk(KERN_DEBUG "Initialized c2h_cnt_th_c_field_info with %d entries\n", sizeof(c2h_cnt_th_c_field_info)/sizeof(c2h_cnt_th_c_field_info[0]));
};

// MD : Structure to hold register field information for C2H count threshold D
static struct regfield_info c2h_cnt_th_d_field_info[] = {
    {"C2H_CNT_TH_D_RSVD_1", C2H_CNT_TH_D_RSVD_1_MASK},
    {"C2H_CNT_TH_D_THESHOLD_CNT", C2H_CNT_TH_D_THESHOLD_CNT_MASK},
    // MD : Debug: Track initialization of C2H count threshold D field info
    printk(KERN_DEBUG "Initialized c2h_cnt_th_d_field_info with %d entries\n", sizeof(c2h_cnt_th_d_field_info)/sizeof(c2h_cnt_th_d_field_info[0]));
};

// MD : Structure to hold register field information for C2H count threshold E
static struct regfield_info c2h_cnt_th_e_field_info[] = {
    {"C2H_CNT_TH_E_RSVD_1", C2H_CNT_TH_E_RSVD_1_MASK},
    {"C2H_CNT_TH_E_THESHOLD_CNT", C2H_CNT_TH_E_THESHOLD_CNT_MASK},
    // MD : Debug: Track initialization of C2H count threshold E field info
    printk(KERN_DEBUG "Initialized c2h_cnt_th_e_field_info with %d entries\n", sizeof(c2h_cnt_th_e_field_info)/sizeof(c2h_cnt_th_e_field_info[0]));
};

// MD : Structure to hold register field information for C2H count threshold F
static struct regfield_info c2h_cnt_th_f_field_info[] = {
    {"C2H_CNT_TH_F_RSVD_1", C2H_CNT_TH_F_RSVD_1_MASK},
    {"C2H_CNT_TH_F_THESHOLD_CNT", C2H_CNT_TH_F_THESHOLD_CNT_MASK},
    // MD : Debug: Track initialization of C2H count threshold F field info
    printk(KERN_DEBUG "Initialized c2h_cnt_th_f_field_info with %d entries\n", sizeof(c2h_cnt_th_f_field_info)/sizeof(c2h_cnt_th_f_field_info[0]));
};

// MD : Structure to hold register field information for C2H count threshold 10
static struct regfield_info c2h_cnt_th_10_field_info[] = {
    {"C2H_CNT_TH_10_RSVD_1", C2H_CNT_TH_10_RSVD_1_MASK},
    {"C2H_CNT_TH_10_THESHOLD_CNT", C2H_CNT_TH_10_THESHOLD_CNT_MASK},
    // MD : Debug: Track initialization of C2H count threshold 10 field info
    printk(KERN_DEBUG "Initialized c2h_cnt_th_10_field_info with %d entries\n", sizeof(c2h_cnt_th_10_field_info)/sizeof(c2h_cnt_th_10_field_info[0]));
};

// MD : Structure to hold register field information for C2H QID to vector map QID
static struct regfield_info c2h_qid2vec_map_qid_field_info[] = {
    {"C2H_QID2VEC_MAP_QID_RSVD_1", C2H_QID2VEC_MAP_QID_RSVD_1_MASK},
    {"C2H_QID2VEC_MAP_QID_QID", C2H_QID2VEC_MAP_QID_QID_MASK},
    // MD : Debug: Track initialization of C2H QID to vector map QID field info
    printk(KERN_DEBUG "Initialized c2h_qid2vec_map_qid_field_info with %d entries\n", sizeof(c2h_qid2vec_map_qid_field_info)/sizeof(c2h_qid2vec_map_qid_field_info[0]));
};

// MD : Structure to hold register field information for C2H QID to vector map
static struct regfield_info c2h_qid2vec_map_field_info[] = {
    {"C2H_QID2VEC_MAP_RSVD_1", C2H_QID2VEC_MAP_RSVD_1_MASK},
    {"C2H_QID2VEC_MAP_H2C_EN_COAL", C2H_QID2VEC_MAP_H2C_EN_COAL_MASK},
    {"C2H_QID2VEC_MAP_H2C_VECTOR", C2H_QID2VEC_MAP_H2C_VECTOR_MASK},
    {"C2H_QID2VEC_MAP_C2H_EN_COAL", C2H_QID2VEC_MAP_C2H_EN_COAL_MASK},
    {"C2H_QID2VEC_MAP_C2H_VECTOR", C2H_QID2VEC_MAP_C2H_VECTOR_MASK},
    // MD : Debug: Track initialization of C2H QID to vector map field info
    printk(KERN_DEBUG "Initialized c2h_qid2vec_map_field_info with %d entries\n", sizeof(c2h_qid2vec_map_field_info)/sizeof(c2h_qid2vec_map_field_info[0]));
};

// MD : Structure to hold register field information for C2H status S_AXIS C2H accepted
static struct regfield_info c2h_stat_s_axis_c2h_accepted_field_info[] = {
    {"C2H_STAT_S_AXIS_C2H_ACCEPTED", C2H_STAT_S_AXIS_C2H_ACCEPTED_MASK},
    // MD : Debug: Track initialization of C2H status S_AXIS C2H accepted field info
    printk(KERN_DEBUG "Initialized c2h_stat_s_axis_c2h_accepted_field_info with %d entries\n", sizeof(c2h_stat_s_axis_c2h_accepted_field_info)/sizeof(c2h_stat_s_axis_c2h_accepted_field_info[0]));
};

// MD : Structure to hold register field information for C2H status S_AXIS WRB accepted
static struct regfield_info c2h_stat_s_axis_wrb_accepted_field_info[] = {
    {"C2H_STAT_S_AXIS_WRB_ACCEPTED", C2H_STAT_S_AXIS_WRB_ACCEPTED_MASK},
    // MD : Debug: Track initialization of C2H status S_AXIS WRB accepted field info
    printk(KERN_DEBUG "Initialized c2h_stat_s_axis_wrb_accepted_field_info with %d entries\n", sizeof(c2h_stat_s_axis_wrb_accepted_field_info)/sizeof(c2h_stat_s_axis_wrb_accepted_field_info[0]));
};

// MD : Structure to hold register field information for C2H status descriptor response packet accepted
static struct regfield_info c2h_stat_desc_rsp_pkt_accepted_field_info[] = {
    {"C2H_STAT_DESC_RSP_PKT_ACCEPTED_D", C2H_STAT_DESC_RSP_PKT_ACCEPTED_D_MASK},
    // MD : Debug: Track initialization of C2H status descriptor response packet accepted field info
    printk(KERN_DEBUG "Initialized c2h_stat_desc_rsp_pkt_accepted_field_info with %d entries\n", sizeof(c2h_stat_desc_rsp_pkt_accepted_field_info)/sizeof(c2h_stat_desc_rsp_pkt_accepted_field_info[0]));
};

// MD : Structure to hold register field information for C2H status AXIS package complete
static struct regfield_info c2h_stat_axis_pkg_cmp_field_info[] = {
    {"C2H_STAT_AXIS_PKG_CMP", C2H_STAT_AXIS_PKG_CMP_MASK},
    // MD : Debug: Track initialization of C2H status AXIS package complete field info
    printk(KERN_DEBUG "Initialized c2h_stat_axis_pkg_cmp_field_info with %d entries\n", sizeof(c2h_stat_axis_pkg_cmp_field_info)/sizeof(c2h_stat_axis_pkg_cmp_field_info[0]));
};

// MD : Structure to hold register field information for C2H status descriptor response accepted
static struct regfield_info c2h_stat_desc_rsp_accepted_field_info[] = {
    {"C2H_STAT_DESC_RSP_ACCEPTED_D", C2H_STAT_DESC_RSP_ACCEPTED_D_MASK},
    // MD : Debug: Track initialization of C2H status descriptor response accepted field info
    printk(KERN_DEBUG "Initialized c2h_stat_desc_rsp_accepted_field_info with %d entries\n", sizeof(c2h_stat_desc_rsp_accepted_field_info)/sizeof(c2h_stat_desc_rsp_accepted_field_info[0]));
};

// MD : Structure to hold register field information for C2H status descriptor response complete
static struct regfield_info c2h_stat_desc_rsp_cmp_field_info[] = {
    {"C2H_STAT_DESC_RSP_CMP_D", C2H_STAT_DESC_RSP_CMP_D_MASK},
    // MD : Debug: Track initialization of C2H status descriptor response complete field info
    printk(KERN_DEBUG "Initialized c2h_stat_desc_rsp_cmp_field_info with %d entries\n", sizeof(c2h_stat_desc_rsp_cmp_field_info)/sizeof(c2h_stat_desc_rsp_cmp_field_info[0]));
};

// MD : Structure to hold register field information for C2H status WRQ out
static struct regfield_info c2h_stat_wrq_out_field_info[] = {
    {"C2H_STAT_WRQ_OUT", C2H_STAT_WRQ_OUT_MASK},
    // MD : Debug: Track initialization of C2H status WRQ out field info
    printk(KERN_DEBUG "Initialized c2h_stat_wrq_out_field_info with %d entries\n", sizeof(c2h_stat_wrq_out_field_info)/sizeof(c2h_stat_wrq_out_field_info[0]));
};

// MD : Structure to hold register field information for C2H status WPL REN accepted
static struct regfield_info c2h_stat_wpl_ren_accepted_field_info[] = {
    {"C2H_STAT_WPL_REN_ACCEPTED", C2H_STAT_WPL_REN_ACCEPTED_MASK},
    // MD : Debug: Track initialization of C2H status WPL REN accepted field info
    printk(KERN_DEBUG "Initialized c2h_stat_wpl_ren_accepted_field_info with %d entries\n", sizeof(c2h_stat_wpl_ren_accepted_field_info)/sizeof(c2h_stat_wpl_ren_accepted_field_info[0]));
};

// MD : Structure to hold register field information for C2H status total WRQ length
static struct regfield_info c2h_stat_total_wrq_len_field_info[] = {
    {"C2H_STAT_TOTAL_WRQ_LEN", C2H_STAT_TOTAL_WRQ_LEN_MASK},
    // MD : Debug: Track initialization of C2H status total WRQ length field info
    printk(KERN_DEBUG "Initialized c2h_stat_total_wrq_len_field_info with %d entries\n", sizeof(c2h_stat_total_wrq_len_field_info)/sizeof(c2h_stat_total_wrq_len_field_info[0]));
};

// MD : Structure to hold register field information for C2H status total WPL length
static struct regfield_info c2h_stat_total_wpl_len_field_info[] = {
    {"C2H_STAT_TOTAL_WPL_LEN", C2H_STAT_TOTAL_WPL_LEN_MASK},
    // MD : Debug: Track initialization of C2H status total WPL length field info
    printk(KERN_DEBUG "Initialized c2h_stat_total_wpl_len_field_info with %d entries\n", sizeof(c2h_stat_total_wpl_len_field_info)/sizeof(c2h_stat_total_wpl_len_field_info[0]));
};

// MD : Structure to hold register field information for C2H buffer size 0
static struct regfield_info c2h_buf_sz_0_field_info[] = {
    {"C2H_BUF_SZ_0_SIZE", C2H_BUF_SZ_0_SIZE_MASK},
    // MD : Debug: Track initialization of C2H buffer size 0 field info
    printk(KERN_DEBUG "Initialized c2h_buf_sz_0_field_info with %d entries\n", sizeof(c2h_buf_sz_0_field_info)/sizeof(c2h_buf_sz_0_field_info[0]));
};

// MD : Structure to hold register field information for C2H buffer size 1
static struct regfield_info c2h_buf_sz_1_field_info[] = {
    {"C2H_BUF_SZ_1_SIZE", C2H_BUF_SZ_1_SIZE_MASK},
    // MD : Debug: Track initialization of C2H buffer size 1 field info
    printk(KERN_DEBUG "Initialized c2h_buf_sz_1_field_info with %d entries\n", sizeof(c2h_buf_sz_1_field_info)/sizeof(c2h_buf_sz_1_field_info[0]));
};

// MD : Structure to hold register field information for C2H buffer size 2
static struct regfield_info c2h_buf_sz_2_field_info[] = {
    {"C2H_BUF_SZ_2_SIZE", C2H_BUF_SZ_2_SIZE_MASK},
    // MD : Debug: Track initialization of C2H buffer size 2 field info
    printk(KERN_DEBUG "Initialized c2h_buf_sz_2_field_info with %d entries\n", sizeof(c2h_buf_sz_2_field_info)/sizeof(c2h_buf_sz_2_field_info[0]));
};

// MD : Structure to hold register field information for C2H buffer size 3
static struct regfield_info c2h_buf_sz_3_field_info[] = {
    {"C2H_BUF_SZ_3_SIZE", C2H_BUF_SZ_3_SIZE_MASK},
    // MD : Debug: Track initialization of C2H buffer size 3 field info
    printk(KERN_DEBUG "Initialized c2h_buf_sz_3_field_info with %d entries\n", sizeof(c2h_buf_sz_3_field_info)/sizeof(c2h_buf_sz_3_field_info[0]));
};

// MD : Structure to hold register field information for C2H buffer size 4
static struct regfield_info c2h_buf_sz_4_field_info[] = {
    {"C2H_BUF_SZ_4_SIZE", C2H_BUF_SZ_4_SIZE_MASK},
    // MD : Debug: Track initialization of C2H buffer size 4 field info
    printk(KERN_DEBUG "Initialized c2h_buf_sz_4_field_info with %d entries\n", sizeof(c2h_buf_sz_4_field_info)/sizeof(c2h_buf_sz_4_field_info[0]));
};

// MD : Structure to hold register field information for C2H buffer size 5
static struct regfield_info c2h_buf_sz_5_field_info[] = {
    {"C2H_BUF_SZ_5_SIZE", C2H_BUF_SZ_5_SIZE_MASK},
    // MD : Debug: Track initialization of C2H buffer size 5 field info
    printk(KERN_DEBUG "Initialized c2h_buf_sz_5_field_info with %d entries\n", sizeof(c2h_buf_sz_5_field_info)/sizeof(c2h_buf_sz_5_field_info[0]));
};

// MD : Structure to hold register field information for C2H buffer size 7
static struct regfield_info c2h_buf_sz_7_field_info[] = {
    {"C2H_BUF_SZ_7_SIZE", C2H_BUF_SZ_7_SIZE_MASK},
    // MD : Debug: Track initialization of C2H buffer size 7 field info
    printk(KERN_DEBUG "Initialized c2h_buf_sz_7_field_info with %d entries\n", sizeof(c2h_buf_sz_7_field_info)/sizeof(c2h_buf_sz_7_field_info[0]));
};

// MD : Structure to hold register field information for C2H buffer size 8
static struct regfield_info c2h_buf_sz_8_field_info[] = {
    {"C2H_BUF_SZ_8_SIZE", C2H_BUF_SZ_8_SIZE_MASK},
    // MD : Debug: Track initialization of C2H buffer size 8 field info
    printk(KERN_DEBUG "Initialized c2h_buf_sz_8_field_info with %d entries\n", sizeof(c2h_buf_sz_8_field_info)/sizeof(c2h_buf_sz_8_field_info[0]));
};

// MD : Structure to hold register field information for C2H buffer size 9
static struct regfield_info c2h_buf_sz_9_field_info[] = {
    {"C2H_BUF_SZ_9_SIZE", C2H_BUF_SZ_9_SIZE_MASK},
    // MD : Debug: Track initialization of C2H buffer size 9 field info
    printk(KERN_DEBUG "Initialized c2h_buf_sz_9_field_info with %d entries\n", sizeof(c2h_buf_sz_9_field_info)/sizeof(c2h_buf_sz_9_field_info[0]));
};

// MD : Structure to hold register field information for C2H buffer size 10
static struct regfield_info c2h_buf_sz_10_field_info[] = {
    {"C2H_BUF_SZ_10_SIZE", C2H_BUF_SZ_10_SIZE_MASK},
    // MD : Debug: Track initialization of C2H buffer size 10 field info
    printk(KERN_DEBUG "Initialized c2h_buf_sz_10_field_info with %d entries\n", sizeof(c2h_buf_sz_10_field_info)/sizeof(c2h_buf_sz_10_field_info[0]));
};

// MD : Structure to hold register field information for C2H buffer size 11
static struct regfield_info c2h_buf_sz_11_field_info[] = {
    {"C2H_BUF_SZ_11_SIZE", C2H_BUF_SZ_11_SIZE_MASK},
    // MD : Debug: Track initialization of C2H buffer size 11 field info
    printk(KERN_DEBUG "Initialized c2h_buf_sz_11_field_info with %d entries\n", sizeof(c2h_buf_sz_11_field_info)/sizeof(c2h_buf_sz_11_field_info[0]));
};

// MD : Structure to hold register field information for C2H buffer size 12
static struct regfield_info c2h_buf_sz_12_field_info[] = {
    {"C2H_BUF_SZ_12_SIZE", C2H_BUF_SZ_12_SIZE_MASK},
    // MD : Debug: Track initialization of C2H buffer size 12 field info
    printk(KERN_DEBUG "Initialized c2h_buf_sz_12_field_info with %d entries\n", sizeof(c2h_buf_sz_12_field_info)/sizeof(c2h_buf_sz_12_field_info[0]));
};

// MD : Structure to hold register field information for C2H buffer size 13
static struct regfield_info c2h_buf_sz_13_field_info[] = {
    {"C2H_BUF_SZ_13_SIZE", C2H_BUF_SZ_13_SIZE_MASK},
    // MD : Debug: Track initialization of C2H buffer size 13 field info
    printk(KERN_DEBUG "Initialized c2h_buf_sz_13_field_info with %d entries\n", sizeof(c2h_buf_sz_13_field_info)/sizeof(c2h_buf_sz_13_field_info[0]));
};

// MD : Structure to hold register field information for C2H buffer size 14
static struct regfield_info c2h_buf_sz_14_field_info[] = {
    {"C2H_BUF_SZ_14_SIZE", C2H_BUF_SZ_14_SIZE_MASK},
    // MD : Debug: Track initialization of C2H buffer size 14 field info
    printk(KERN_DEBUG "Initialized c2h_buf_sz_14_field_info with %d entries\n", sizeof(c2h_buf_sz_14_field_info)/sizeof(c2h_buf_sz_14_field_info[0]));
};

// MD : Structure to hold register field information for C2H buffer size 15
static struct regfield_info c2h_buf_sz_15_field_info[] = {
    {"C2H_BUF_SZ_15_SIZE", C2H_BUF_SZ_15_SIZE_MASK},
    // MD : Debug: Track initialization of C2H buffer size 15 field info
    printk(KERN_DEBUG "Initialized c2h_buf_sz_15_field_info with %d entries\n", sizeof(c2h_buf_sz_15_field_info)/sizeof(c2h_buf_sz_15_field_info[0]));
};

// MD : Structure to hold register field information for C2H error status
static struct regfield_info c2h_err_stat_field_info[] = {
    {"C2H_ERR_STAT_RSVD_1", C2H_ERR_STAT_RSVD_1_MASK},
    {"C2H_ERR_STAT_WRB_PRTY_ERR", C2H_ERR_STAT_WRB_PRTY_ERR_MASK},
    {"C2H_ERR_STAT_WRB_CIDX_ERR", C2H_ERR_STAT_WRB_CIDX_ERR_MASK},
    {"C2H_ERR_STAT_WRB_QFULL_ERR", C2H_ERR_STAT_WRB_QFULL_ERR_MASK},
    {"C2H_ERR_STAT_WRB_INV_Q_ERR", C2H_ERR_STAT_WRB_INV_Q_ERR_MASK},
    {"C2H_ERR_STAT_PORT_ID_BYP_IN_MISMATCH", C2H_ERR_STAT_PORT_ID_BYP_IN_MISMATCH_MASK},
    {"C2H_ERR_STAT_PORT_ID_CTXT_MISMATCH", C2H_ERR_STAT_PORT_ID_CTXT_MISMATCH_MASK},
    {"C2H_ERR_STAT_ERR_DESC_CNT", C2H_ERR_STAT_ERR_DESC_CNT_MASK},
    {"C2H_ERR_STAT_RSVD_2", C2H_ERR_STAT_RSVD_2_MASK},
    {"C2H_ERR_STAT_MSI_INT_FAIL", C2H_ERR_STAT_MSI_INT_FAIL_MASK},
    {"C2H_ERR_STAT_ENG_WPL_DATA_PAR_ERR", C2H_ERR_STAT_ENG_WPL_DATA_PAR_ERR_MASK},
    {"C2H_ERR_STAT_RSVD_3", C2H_ERR_STAT_RSVD_3_MASK},
    {"C2H_ERR_STAT_DESC_RSP_ERR", C2H_ERR_STAT_DESC_RSP_ERR_MASK},
    {"C2H_ERR_STAT_QID_MISMATCH", C2H_ERR_STAT_QID_MISMATCH_MASK},
    {"C2H_ERR_STAT_RSVD_4", C2H_ERR_STAT_RSVD_4_MASK},
    {"C2H_ERR_STAT_LEN_MISMATCH", C2H_ERR_STAT_LEN_MISMATCH_MASK},
    {"C2H_ERR_STAT_MTY_MISMATCH", C2H_ERR_STAT_MTY_MISMATCH_MASK},
    // MD : Debug: Track initialization of C2H error status field info
    printk(KERN_DEBUG "Initialized c2h_err_stat_field_info with %d entries\n", sizeof(c2h_err_stat_field_info)/sizeof(c2h_err_stat_field_info[0]));
};

// MD : Structure to hold register field information for C2H error mask
static struct regfield_info c2h_err_mask_field_info[] = {
    {"C2H_ERR_EN", C2H_ERR_EN_MASK},
    // MD : Debug: Track initialization of C2H error mask field info
    printk(KERN_DEBUG "Initialized c2h_err_mask_field_info with %d entries\n", sizeof(c2h_err_mask_field_info)/sizeof(c2h_err_mask_field_info[0]));
};

// MD : Structure to hold register field information for C2H fatal error status
static struct regfield_info c2h_fatal_err_stat_field_info[] = {
    {"C2H_FATAL_ERR_STAT_RSVD_1", C2H_FATAL_ERR_STAT_RSVD_1_MASK},
    {"C2H_FATAL_ERR_STAT_WPL_DATA_PAR_ERR", C2H_FATAL_ERR_STAT_WPL_DATA_PAR_ERR_MASK},
    {"C2H_FATAL_ERR_STAT_PLD_FIFO_RAM_RDBE", C2H_FATAL_ERR_STAT_PLD_FIFO_RAM_RDBE_MASK},
    {"C2H_FATAL_ERR_STAT_QID_FIFO_RAM_RDBE", C2H_FATAL_ERR_STAT_QID_FIFO_RAM_RDBE_MASK},
    {"C2H_FATAL_ERR_STAT_TUSER_FIFO_RAM_RDBE", C2H_FATAL_ERR_STAT_TUSER_FIFO_RAM_RDBE_MASK},
    {"C2H_FATAL_ERR_STAT_WRB_COAL_DATA_RAM_RDBE", C2H_FATAL_ERR_STAT_WRB_COAL_DATA_RAM_RDBE_MASK},
    {"C2H_FATAL_ERR_STAT_INT_QID2VEC_RAM_RDBE", C2H_FATAL_ERR_STAT_INT_QID2VEC_RAM_RDBE_MASK},
    {"C2H_FATAL_ERR_STAT_INT_CTXT_RAM_RDBE", C2H_FATAL_ERR_STAT_INT_CTXT_RAM_RDBE_MASK},
    {"C2H_FATAL_ERR_STAT_DESC_REQ_FIFO_RAM_RDBE", C2H_FATAL_ERR_STAT_DESC_REQ_FIFO_RAM_RDBE_MASK},
    {"C2H_FATAL_ERR_STAT_PFCH_CTXT_RAM_RDBE", C2H_FATAL_ERR_STAT_PFCH_CTXT_RAM_RDBE_MASK},
    {"C2H_FATAL_ERR_STAT_WRB_CTXT_RAM_RDBE", C2H_FATAL_ERR_STAT_WRB_CTXT_RAM_RDBE_MASK},
    {"C2H_FATAL_ERR_STAT_PFCH_LL_RAM_RDBE", C2H_FATAL_ERR_STAT_PFCH_LL_RAM_RDBE_MASK},
    {"C2H_FATAL_ERR_STAT_TIMER_FIFO_RAM_RDBE", C2H_FATAL_ERR_STAT_TIMER_FIFO_RAM_RDBE_MASK},
    {"C2H_FATAL_ERR_STAT_QID_MISMATCH", C2H_FATAL_ERR_STAT_QID_MISMATCH_MASK},
    {"C2H_FATAL_ERR_STAT_RSVD_2", C2H_FATAL_ERR_STAT_RSVD_2_MASK},
    {"C2H_FATAL_ERR_STAT_LEN_MISMATCH", C2H_FATAL_ERR_STAT_LEN_MISMATCH_MASK},
    {"C2H_FATAL_ERR_STAT_MTY_MISMATCH", C2H_FATAL_ERR_STAT_MTY_MISMATCH_MASK},
    // MD : Debug: Track initialization of C2H fatal error status field info
    printk(KERN_DEBUG "Initialized c2h_fatal_err_stat_field_info with %d entries\n", sizeof(c2h_fatal_err_stat_field_info)/sizeof(c2h_fatal_err_stat_field_info[0]));
};

// MD : Structure to hold register field information for C2H fatal error mask
static struct regfield_info c2h_fatal_err_mask_field_info[] = {
    {"C2H_FATAL_ERR_C2HEN", C2H_FATAL_ERR_C2HEN_MASK},
    // MD : Debug: Track initialization of C2H fatal error mask field info
    printk(KERN_DEBUG "Initialized c2h_fatal_err_mask_field_info with %d entries\n", sizeof(c2h_fatal_err_mask_field_info)/sizeof(c2h_fatal_err_mask_field_info[0]));
};

// MD : Structure to hold register field information for C2H fatal error enable
static struct regfield_info c2h_fatal_err_enable_field_info[] = {
    {"C2H_FATAL_ERR_ENABLE_RSVD_1", C2H_FATAL_ERR_ENABLE_RSVD_1_MASK},
    {"C2H_FATAL_ERR_ENABLE_WPL_PAR_INV", C2H_FATAL_ERR_ENABLE_WPL_PAR_INV_MASK},
    {"C2H_FATAL_ERR_ENABLE_WRQ_DIS", C2H_FATAL_ERR_ENABLE_WRQ_DIS_MASK},
    // MD : Debug: Track initialization of C2H fatal error enable field info
    printk(KERN_DEBUG "Initialized c2h_fatal_err_enable_field_info with %d entries\n", sizeof(c2h_fatal_err_enable_field_info)/sizeof(c2h_fatal_err_enable_field_info[0]));
};

// MD : Structure to hold register field information for global error interrupt
static struct regfield_info glbl_err_int_field_info[] = {
    {"GLBL_ERR_INT_RSVD_1", GLBL_ERR_INT_RSVD_1_MASK},
    {"GLBL_ERR_INT_ARM", GLBL_ERR_INT_ARM_MASK},
    {"GLBL_ERR_INT_EN_COAL", GLBL_ERR_INT_EN_COAL_MASK},
    {"GLBL_ERR_INT_VEC", GLBL_ERR_INT_VEC_MASK},
    {"GLBL_ERR_INT_FUNC", GLBL_ERR_INT_FUNC_MASK},
    // MD : Debug: Track initialization of global error interrupt field info
    printk(KERN_DEBUG "Initialized glbl_err_int_field_info with %d entries\n", sizeof(glbl_err_int_field_info)/sizeof(glbl_err_int_field_info[0]));
};

// MD : Structure to hold register field information for C2H prefetch configuration
static struct regfield_info c2h_pfch_cfg_field_info[] = {
    {"C2H_PFCH_CFG_EVT_QCNT_TH", C2H_PFCH_CFG_EVT_QCNT_TH_MASK},
    {"C2H_PFCH_CFG_QCNT", C2H_PFCH_CFG_QCNT_MASK},
    {"C2H_PFCH_CFG_NUM", C2H_PFCH_CFG_NUM_MASK},
    {"C2H_PFCH_CFG_FL_TH", C2H_PFCH_CFG_FL_TH_MASK},
    // MD : Debug: Track initialization of C2H prefetch configuration field info
    printk(KERN_DEBUG "Initialized c2h_pfch_cfg_field_info with %d entries\n", sizeof(c2h_pfch_cfg_field_info)/sizeof(c2h_pfch_cfg_field_info[0]));
};

// MD : Structure to hold register field information for C2H interrupt timer tick
static struct regfield_info c2h_int_timer_tick_field_info[] = {
    {"C2H_INT_TIMER_TICK", C2H_INT_TIMER_TICK_MASK},
    // MD : Debug: Track initialization of C2H interrupt timer tick field info
    printk(KERN_DEBUG "Initialized c2h_int_timer_tick_field_info with %d entries\n", sizeof(c2h_int_timer_tick_field_info)/sizeof(c2h_int_timer_tick_field_info[0]));
};

// MD : Structure to hold register field information for C2H interrupt C2H MM request
static struct regfield_info c2h_intr_c2h_mm_req_field_info[] = {
    {"C2H_INTR_C2H_MM_REQ_RSVD_1", C2H_INTR_C2H_MM_REQ_RSVD_1_MASK},
    {"C2H_INTR_C2H_MM_REQ_CNT", C2H_INTR_C2H_MM_REQ_CNT_MASK},
    // MD : Debug: Track initialization of C2H interrupt C2H MM request field info
    printk(KERN_DEBUG "Initialized c2h_intr_c2h_mm_req_field_info with %d entries\n", sizeof(c2h_intr_c2h_mm_req_field_info)/sizeof(c2h_intr_c2h_mm_req_field_info[0]));
};

// MD : Structure to hold register field information for C2H interrupt error interrupt request
static struct regfield_info c2h_intr_err_int_req_field_info[] = {
    {"C2H_INTR_ERR_INT_REQ_RSVD_1", C2H_INTR_ERR_INT_REQ_RSVD_1_MASK},
    {"C2H_INTR_ERR_INT_REQ_CNT", C2H_INTR_ERR_INT_REQ_CNT_MASK},
    // MD : Debug: Track initialization of C2H interrupt error interrupt request field info
    printk(KERN_DEBUG "Initialized c2h_intr_err_int_req_field_info with %d entries\n", sizeof(c2h_intr_err_int_req_field_info)/sizeof(c2h_intr_err_int_req_field_info[0]));
};

// MD : Structure to hold register field information for C2H interrupt C2H ST request
static struct regfield_info c2h_intr_c2h_st_req_field_info[] = {
    {"C2H_INTR_C2H_ST_REQ_RSVD_1", C2H_INTR_C2H_ST_REQ_RSVD_1_MASK},
    {"C2H_INTR_C2H_ST_REQ_CNT", C2H_INTR_C2H_ST_REQ_CNT_MASK},
    // MD : Debug: Track initialization of C2H interrupt C2H ST request field info
    printk(KERN_DEBUG "Initialized c2h_intr_c2h_st_req_field_info with %d entries\n", sizeof(c2h_intr_c2h_st_req_field_info)/sizeof(c2h_intr_c2h_st_req_field_info[0]));
};

// MD : Structure to hold register field information for C2H interrupt H2C error C2H MM MSIX acknowledgment
static struct regfield_info c2h_intr_h2c_err_c2h_mm_msix_ack_field_info[] = {
    {"C2H_INTR_H2C_ERR_C2H_MM_MSIX_ACK_RSVD_1", C2H_INTR_H2C_ERR_C2H_MM_MSIX_ACK_RSVD_1_MASK},
    {"C2H_INTR_H2C_ERR_C2H_MM_MSIX_ACK_CNT", C2H_INTR_H2C_ERR_C2H_MM_MSIX_ACK_CNT_MASK},
    // MD : Debug: Track initialization of C2H interrupt H2C error C2H MM MSIX acknowledgment field info
    printk(KERN_DEBUG "Initialized c2h_intr_h2c_err_c2h_mm_msix_ack_field_info with %d entries\n", sizeof(c2h_intr_h2c_err_c2h_mm_msix_ack_field_info)/sizeof(c2h_intr_h2c_err_c2h_mm_msix_ack_field_info[0]));
};

// MD : Structure to hold register field information for C2H interrupt H2C error C2H MM MSIX failure
static struct regfield_info c2h_intr_h2c_err_c2h_mm_msix_fail_field_info[] = {
    {"C2H_INTR_H2C_ERR_C2H_MM_MSIX_FAIL_RSVD_1", C2H_INTR_H2C_ERR_C2H_MM_MSIX_FAIL_RSVD_1_MASK},
    {"C2H_INTR_H2C_ERR_C2H_MM_MSIX_FAIL_CNT", C2H_INTR_H2C_ERR_C2H_MM_MSIX_FAIL_CNT_MASK},
    // MD : Debug: Track initialization of C2H interrupt H2C error C2H MM MSIX failure field info
    printk(KERN_DEBUG "Initialized c2h_intr_h2c_err_c2h_mm_msix_fail_field_info with %d entries\n", sizeof(c2h_intr_h2c_err_c2h_mm_msix_fail_field_info)/sizeof(c2h_intr_h2c_err_c2h_mm_msix_fail_field_info[0]));
};

// MD : Structure to hold register field information for C2H interrupt H2C error C2H MM MSIX no MSIX
static struct regfield_info c2h_intr_h2c_err_c2h_mm_msix_no_msix_field_info[] = {
    {"C2H_INTR_H2C_ERR_C2H_MM_MSIX_NO_MSIX_RSVD_1", C2H_INTR_H2C_ERR_C2H_MM_MSIX_NO_MSIX_RSVD_1_MASK},
    {"C2H_INTR_H2C_ERR_C2H_MM_MSIX_NO_MSIX_CNT", C2H_INTR_H2C_ERR_C2H_MM_MSIX_NO_MSIX_CNT_MASK},
    // MD : Debug: Track initialization of C2H interrupt H2C error C2H MM MSIX no MSIX field info
    printk(KERN_DEBUG "Initialized c2h_intr_h2c_err_c2h_mm_msix_no_msix_field_info with %d entries\n", sizeof(c2h_intr_h2c_err_c2h_mm_msix_no_msix_field_info)/sizeof(c2h_intr_h2c_err_c2h_mm_msix_no_msix_field_info[0]));
};

// MD : Structure to hold register field information for C2H interrupt H2C error C2H MM context invalid
static struct regfield_info c2h_intr_h2c_err_c2h_mm_ctxt_inval_field_info[] = {
    {"C2H_INTR_H2C_ERR_C2H_MM_CTXT_INVAL_RSVD_1", C2H_INTR_H2C_ERR_C2H_MM_CTXT_INVAL_RSVD_1_MASK},
    {"C2H_INTR_H2C_ERR_C2H_MM_CTXT_INVAL_CNT", C2H_INTR_H2C_ERR_C2H_MM_CTXT_INVAL_CNT_MASK},
    // MD : Debug: Track initialization of C2H interrupt H2C error C2H MM context invalid field info
    printk(KERN_DEBUG "Initialized c2h_intr_h2c_err_c2h_mm_ctxt_inval_field_info with %d entries\n", sizeof(c2h_intr_h2c_err_c2h_mm_ctxt_inval_field_info)/sizeof(c2h_intr_h2c_err_c2h_mm_ctxt_inval_field_info[0]));
};

// MD : Structure to hold register field information for C2H interrupt C2H ST MSIX acknowledgment
static struct regfield_info c2h_intr_c2h_st_msix_ack_field_info[] = {
    {"C2H_INTR_C2H_ST_MSIX_ACK_RSVD_1", C2H_INTR_C2H_ST_MSIX_ACK_RSVD_1_MASK},
    {"C2H_INTR_C2H_ST_MSIX_ACK_CNT", C2H_INTR_C2H_ST_MSIX_ACK_CNT_MASK},
    // MD : Debug: Track initialization of C2H interrupt C2H ST MSIX acknowledgment field info
    printk(KERN_DEBUG "Initialized c2h_intr_c2h_st_msix_ack_field_info with %d entries\n", sizeof(c2h_intr_c2h_st_msix_ack_field_info)/sizeof(c2h_intr_c2h_st_msix_ack_field_info[0]));
};

// MD : Structure to hold register field information for C2H interrupt C2H ST MSIX failure
static struct regfield_info c2h_intr_c2h_st_msix_fail_field_info[] = {
    {"C2H_INTR_C2H_ST_MSIX_FAIL_RSVD_1", C2H_INTR_C2H_ST_MSIX_FAIL_RSVD_1_MASK},
    {"C2H_INTR_C2H_ST_MSIX_FAIL_CNT", C2H_INTR_C2H_ST_MSIX_FAIL_CNT_MASK},
    // MD : Debug: Track initialization of C2H interrupt C2H ST MSIX failure field info
    printk(KERN_DEBUG "Initialized c2h_intr_c2h_st_msix_fail_field_info with %d entries\n", sizeof(c2h_intr_c2h_st_msix_fail_field_info)/sizeof(c2h_intr_c2h_st_msix_fail_field_info[0]));
};

// MD : Structure to hold register field information for C2H interrupt C2H ST no MSIX
static struct regfield_info c2h_intr_c2h_st_no_msix_field_info[] = {
    {"C2H_INTR_C2H_ST_NO_MSIX_RSVD_1", C2H_INTR_C2H_ST_NO_MSIX_RSVD_1_MASK},
    {"C2H_INTR_C2H_ST_NO_MSIX_CNT", C2H_INTR_C2H_ST_NO_MSIX_CNT_MASK},
    // MD : Debug: Track initialization of C2H interrupt C2H ST no MSIX field info
    printk(KERN_DEBUG "Initialized c2h_intr_c2h_st_no_msix_field_info with %d entries\n", sizeof(c2h_intr_c2h_st_no_msix_field_info)/sizeof(c2h_intr_c2h_st_no_msix_field_info[0]));
};

// MD : Structure to hold register field information for C2H interrupt C2H ST context invalid
static struct regfield_info c2h_intr_c2h_st_ctxt_inval_field_info[] = {
    {"C2H_INTR_C2H_ST_CTXT_INVAL_RSVD_1", C2H_INTR_C2H_ST_CTXT_INVAL_RSVD_1_MASK},
    {"C2H_INTR_C2H_ST_CTXT_INVAL_CNT", C2H_INTR_C2H_ST_CTXT_INVAL_CNT_MASK},
    // MD : Debug: Track initialization of C2H interrupt C2H ST context invalid field info
    printk(KERN_DEBUG "Initialized c2h_intr_c2h_st_ctxt_inval_field_info with %d entries\n", sizeof(c2h_intr_c2h_st_ctxt_inval_field_info)/sizeof(c2h_intr_c2h_st_ctxt_inval_field_info[0]));
};

// MD : Structure to hold register field information for C2H status write completion
static struct regfield_info c2h_stat_wr_cmp_field_info[] = {
    {"C2H_STAT_WR_CMP_RSVD_1", C2H_STAT_WR_CMP_RSVD_1_MASK},
    {"C2H_STAT_WR_CMP_CNT", C2H_STAT_WR_CMP_CNT_MASK},
    // MD : Debug: Track initialization of C2H status write completion field info
    printk(KERN_DEBUG "Initialized c2h_stat_wr_cmp_field_info with %d entries\n", sizeof(c2h_stat_wr_cmp_field_info)/sizeof(c2h_stat_wr_cmp_field_info[0]));
};

// MD : Structure to hold register field information for C2H status debug DMA engine 4
static struct regfield_info c2h_stat_dbg_dma_eng_4_field_info[] = {
    {"C2H_STAT_DMA_ENG_4_TUSER_FIFO_OUT_VLD", C2H_STAT_DMA_ENG_4_TUSER_FIFO_OUT_VLD_MASK},
    {"C2H_STAT_DMA_ENG_4_WRB_FIFO_IN_RDY", C2H_STAT_DMA_ENG_4_WRB_FIFO_IN_RDY_MASK},
    {"C2H_STAT_DMA_ENG_4_TUSER_FIFO_IN_CNT", C2H_STAT_DMA_ENG_4_TUSER_FIFO_IN_CNT_MASK},
    {"C2H_STAT_DMA_ENG_4_TUSER_FIFO_OUTPUT_CNT", C2H_STAT_DMA_ENG_4_TUSER_FIFO_OUTPUT_CNT_MASK},
    {"C2H_STAT_DMA_ENG_4_TUSER_FIFO_OUT_CNT", C2H_STAT_DMA_ENG_4_TUSER_FIFO_OUT_CNT_MASK},
    // MD : Debug: Track initialization of C2H status debug DMA engine 4 field info
    printk(KERN_DEBUG "Initialized c2h_stat_dbg_dma_eng_4_field_info with %d entries\n", sizeof(c2h_stat_dbg_dma_eng_4_field_info)/sizeof(c2h_stat_dbg_dma_eng_4_field_info[0]));
};

// MD : Structure to hold register field information for C2H status debug DMA engine 5
static struct regfield_info c2h_stat_dbg_dma_eng_5_field_info[] = {
    {"C2H_STAT_DMA_ENG_5_RSVD_1", C2H_STAT_DMA_ENG_5_RSVD_1_MASK},
    {"C2H_STAT_DMA_ENG_5_TUSER_COMB_OUT_VLD", C2H_STAT_DMA_ENG_5_TUSER_COMB_OUT_VLD_MASK},
    {"C2H_STAT_DMA_ENG_5_TUSER_FIFO_IN_RDY", C2H_STAT_DMA_ENG_5_TUSER_FIFO_IN_RDY_MASK},
    {"C2H_STAT_DMA_ENG_5_TUSER_COMB_IN_CNT", C2H_STAT_DMA_ENG_5_TUSER_COMB_IN_CNT_MASK},
    {"C2H_STAT_DMA_ENG_5_TUSE_COMB_OUTPUT_CNT", C2H_STAT_DMA_ENG_5_TUSE_COMB_OUTPUT_CNT_MASK},
    {"C2H_STAT_DMA_ENG_5_TUSER_COMB_CNT", C2H_STAT_DMA_ENG_5_TUSER_COMB_CNT_MASK},
    // MD : Debug: Track initialization of C2H status debug DMA engine 5 field info
    printk(KERN_DEBUG "Initialized c2h_stat_dbg_dma_eng_5_field_info with %d entries\n", sizeof(c2h_stat_dbg_dma_eng_5_field_info)/sizeof(c2h_stat_dbg_dma_eng_5_field_info[0]));
};

// MD : Structure to hold register field information for C2H debug prefetch QID
static struct regfield_info c2h_dbg_pfch_qid_field_info[] = {
    {"C2H_PFCH_QID_RSVD_1", C2H_PFCH_QID_RSVD_1_MASK},
    {"C2H_PFCH_QID_ERR_CTXT", C2H_PFCH_QID_ERR_CTXT_MASK},
    {"C2H_PFCH_QID_TARGET", C2H_PFCH_QID_TARGET_MASK},
    {"C2H_PFCH_QID_QID_OR_TAG", C2H_PFCH_QID_QID_OR_TAG_MASK},
    // MD : Debug: Track initialization of C2H debug prefetch QID field info
    printk(KERN_DEBUG "Initialized c2h_dbg_pfch_qid_field_info with %d entries\n", sizeof(c2h_dbg_pfch_qid_field_info)/sizeof(c2h_dbg_pfch_qid_field_info[0]));
};

// MD : Structure to hold register field information for C2H debug prefetch
static struct regfield_info c2h_dbg_pfch_field_info[] = {
    {"C2H_PFCH_DATA", C2H_PFCH_DATA_MASK},
    // MD : Debug: Track initialization of C2H debug prefetch field info
    printk(KERN_DEBUG "Initialized c2h_dbg_pfch_field_info with %d entries\n", sizeof(c2h_dbg_pfch_field_info)/sizeof(c2h_dbg_pfch_field_info[0]));
};

// MD : Structure to hold register field information for C2H interrupt debug
static struct regfield_info c2h_int_dbg_field_info[] = {
    {"C2H_INT_RSVD_1", C2H_INT_RSVD_1_MASK},
    {"C2H_INT_INT_COAL_SM", C2H_INT_INT_COAL_SM_MASK},
    {"C2H_INT_INT_SM", C2H_INT_INT_SM_MASK},
    // MD : Debug: Track initialization of C2H interrupt debug field info
    printk(KERN_DEBUG "Initialized c2h_int_dbg_field_info with %d entries\n", sizeof(c2h_int_dbg_field_info)/sizeof(c2h_int_dbg_field_info[0]));
};

// MD : Structure to hold register field information for C2H status immediate accepted
static struct regfield_info c2h_stat_imm_accepted_field_info[] = {
    {"C2H_STAT_IMM_ACCEPTED_RSVD_1", C2H_STAT_IMM_ACCEPTED_RSVD_1_MASK},
    {"C2H_STAT_IMM_ACCEPTED_CNT", C2H_STAT_IMM_ACCEPTED_CNT_MASK},
    // MD : Debug: Track initialization of C2H status immediate accepted field info
    printk(KERN_DEBUG "Initialized c2h_stat_imm_accepted_field_info with %d entries\n", sizeof(c2h_stat_imm_accepted_field_info)/sizeof(c2h_stat_imm_accepted_field_info[0]));
};

// MD : Structure to hold register field information for C2H marker accepted status
static struct regfield_info c2h_stat_marker_accepted_field_info[] = {
    {"C2H_STAT_MARKER_ACCEPTED_RSVD_1", C2H_STAT_MARKER_ACCEPTED_RSVD_1_MASK},
    {"C2H_STAT_MARKER_ACCEPTED_CNT", C2H_STAT_MARKER_ACCEPTED_CNT_MASK},
    // MD : Debug: Track initialization of C2H marker accepted field info
    printk(KERN_DEBUG "Initialized c2h_stat_marker_accepted_field_info with %d entries\n", sizeof(c2h_stat_marker_accepted_field_info)/sizeof(c2h_stat_marker_accepted_field_info[0]));
};

// MD : Structure to hold register field information for C2H disable completion accepted status
static struct regfield_info c2h_stat_disable_cmp_accepted_field_info[] = {
    {"C2H_STAT_DISABLE_CMP_ACCEPTED_RSVD_1", C2H_STAT_DISABLE_CMP_ACCEPTED_RSVD_1_MASK},
    {"C2H_STAT_DISABLE_CMP_ACCEPTED_CNT", C2H_STAT_DISABLE_CMP_ACCEPTED_CNT_MASK},
    // MD : Debug: Track initialization of C2H disable completion accepted field info
    printk(KERN_DEBUG "Initialized c2h_stat_disable_cmp_accepted_field_info with %d entries\n", sizeof(c2h_stat_disable_cmp_accepted_field_info)/sizeof(c2h_stat_disable_cmp_accepted_field_info[0]));
};

// MD : Structure to hold register field information for C2H payload FIFO credit count
static struct regfield_info c2h_pld_fifo_crdt_cnt_field_info[] = {
    {"C2H_PLD_FIFO_CRDT_CNT_RSVD_1", C2H_PLD_FIFO_CRDT_CNT_RSVD_1_MASK},
    {"C2H_PLD_FIFO_CRDT_CNT_CNT", C2H_PLD_FIFO_CRDT_CNT_CNT_MASK},
    // MD : Debug: Track initialization of C2H payload FIFO credit count field info
    printk(KERN_DEBUG "Initialized c2h_pld_fifo_crdt_cnt_field_info with %d entries\n", sizeof(c2h_pld_fifo_crdt_cnt_field_info)/sizeof(c2h_pld_fifo_crdt_cnt_field_info[0]));
};

// MD : Structure to hold register field information for H2C error status
static struct regfield_info h2c_err_stat_field_info[] = {
    {"H2C_ERR_STAT_RSVD_1", H2C_ERR_STAT_RSVD_1_MASK},
    {"H2C_ERR_STAT_SBE", H2C_ERR_STAT_SBE_MASK},
    {"H2C_ERR_STAT_DBE", H2C_ERR_STAT_DBE_MASK},
    {"H2C_ERR_STAT_NO_DMA_DS", H2C_ERR_STAT_NO_DMA_DS_MASK},
    {"H2C_ERR_STAT_SDI_MRKR_REQ_MOP_ERR", H2C_ERR_STAT_SDI_MRKR_REQ_MOP_ERR_MASK},
    {"H2C_ERR_STAT_ZERO_LEN_DS", H2C_ERR_STAT_ZERO_LEN_DS_MASK},
    // MD : Debug: Track initialization of H2C error status field info
    printk(KERN_DEBUG "Initialized h2c_err_stat_field_info with %d entries\n", sizeof(h2c_err_stat_field_info)/sizeof(h2c_err_stat_field_info[0]));
};

// MD : Structure to hold register field information for H2C error mask
static struct regfield_info h2c_err_mask_field_info[] = {
    {"H2C_ERR_EN", H2C_ERR_EN_MASK},
    // MD : Debug: Track initialization of H2C error mask field info
    printk(KERN_DEBUG "Initialized h2c_err_mask_field_info with %d entries\n", sizeof(h2c_err_mask_field_info)/sizeof(h2c_err_mask_field_info[0]));
};

// MD : Structure to hold register field information for H2C first error QID
static struct regfield_info h2c_first_err_qid_field_info[] = {
    {"H2C_FIRST_ERR_QID_RSVD_1", H2C_FIRST_ERR_QID_RSVD_1_MASK},
    {"H2C_FIRST_ERR_QID_ERR_TYPE", H2C_FIRST_ERR_QID_ERR_TYPE_MASK},
    {"H2C_FIRST_ERR_QID_RSVD_2", H2C_FIRST_ERR_QID_RSVD_2_MASK},
    {"H2C_FIRST_ERR_QID_QID", H2C_FIRST_ERR_QID_QID_MASK},
    // MD : Debug: Track initialization of H2C first error QID field info
    printk(KERN_DEBUG "Initialized h2c_first_err_qid_field_info with %d entries\n", sizeof(h2c_first_err_qid_field_info)/sizeof(h2c_first_err_qid_field_info[0]));
};

// MD : Structure to hold register field information for H2C debug register 0
static struct regfield_info h2c_dbg_reg0_field_info[] = {
    {"H2C_REG0_NUM_DSC_RCVD", H2C_REG0_NUM_DSC_RCVD_MASK},
    {"H2C_REG0_NUM_WRB_SENT", H2C_REG0_NUM_WRB_SENT_MASK},
    // MD : Debug: Track initialization of H2C debug register 0 field info
    printk(KERN_DEBUG "Initialized h2c_dbg_reg0_field_info with %d entries\n", sizeof(h2c_dbg_reg0_field_info)/sizeof(h2c_dbg_reg0_field_info[0]));
};

// MD : Structure to hold register field information for H2C debug register 1
static struct regfield_info h2c_dbg_reg1_field_info[] = {
    {"H2C_REG1_NUM_REQ_SENT", H2C_REG1_NUM_REQ_SENT_MASK},
    {"H2C_REG1_NUM_CMP_SENT", H2C_REG1_NUM_CMP_SENT_MASK},
    // MD : Debug: Track initialization of H2C debug register 1 field info
    printk(KERN_DEBUG "Initialized h2c_dbg_reg1_field_info with %d entries\n", sizeof(h2c_dbg_reg1_field_info)/sizeof(h2c_dbg_reg1_field_info[0]));
};

// MD : Structure to hold register field information for H2C debug register 2
static struct regfield_info h2c_dbg_reg2_field_info[] = {
    {"H2C_REG2_RSVD_1", H2C_REG2_RSVD_1_MASK},
    {"H2C_REG2_NUM_ERR_DSC_RCVD", H2C_REG2_NUM_ERR_DSC_RCVD_MASK},
    // MD : Debug: Track initialization of H2C debug register 2 field info
    printk(KERN_DEBUG "Initialized h2c_dbg_reg2_field_info with %d entries\n", sizeof(h2c_dbg_reg2_field_info)/sizeof(h2c_dbg_reg2_field_info[0]));
};

// MD : Structure to hold register field information for H2C debug register 3
static struct regfield_info h2c_dbg_reg3_field_info[] = {
    {"H2C_REG3", H2C_REG3_MASK},
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
    // MD : Debug: Track initialization of H2C debug register 3 field info
    printk(KERN_DEBUG "Initialized h2c_dbg_reg3_field_info with %d entries\n", sizeof(h2c_dbg_reg3_field_info)/sizeof(h2c_dbg_reg3_field_info[0]));
};

// MD : Structure to hold register field information for H2C debug register 4
static struct regfield_info h2c_dbg_reg4_field_info[] = {
    {"H2C_REG4_RDREQ_ADDR", H2C_REG4_RDREQ_ADDR_MASK},
    // MD : Debug: Track initialization of H2C debug register 4 field info
    printk(KERN_DEBUG "Initialized h2c_dbg_reg4_field_info with %d entries\n", sizeof(h2c_dbg_reg4_field_info)/sizeof(h2c_dbg_reg4_field_info[0]));
};

// MD : Structure to hold register field information for H2C fatal error enable
static struct regfield_info h2c_fatal_err_en_field_info[] = {
    {"H2C_FATAL_ERR_EN_RSVD_1", H2C_FATAL_ERR_EN_RSVD_1_MASK},
    {"H2C_FATAL_ERR_EN_H2C", H2C_FATAL_ERR_EN_H2C_MASK},
    // MD : Debug: Track initialization of H2C fatal error enable field info
    printk(KERN_DEBUG "Initialized h2c_fatal_err_en_field_info with %d entries\n", sizeof(h2c_fatal_err_en_field_info)/sizeof(h2c_fatal_err_en_field_info[0]));
};

// MD : Structure to hold register field information for C2H channel control
static struct regfield_info c2h_channel_ctl_field_info[] = {
    {"C2H_CHANNEL_CTL_RSVD_1", C2H_CHANNEL_CTL_RSVD_1_MASK},
    {"C2H_CHANNEL_CTL_RUN", C2H_CHANNEL_CTL_RUN_MASK},
    // MD : Debug: Track initialization of C2H channel control field info
    printk(KERN_DEBUG "Initialized c2h_channel_ctl_field_info with %d entries\n", sizeof(c2h_channel_ctl_field_info)/sizeof(c2h_channel_ctl_field_info[0]));
};

// MD : Structure to hold register field information for C2H channel control 1
static struct regfield_info c2h_channel_ctl_1_field_info[] = {
    {"C2H_CHANNEL_CTL_1_RUN", C2H_CHANNEL_CTL_1_RUN_MASK},
    {"C2H_CHANNEL_CTL_1_RUN_1", C2H_CHANNEL_CTL_1_RUN_1_MASK},
    // MD : Debug: Track initialization of C2H channel control 1 field info
    printk(KERN_DEBUG "Initialized c2h_channel_ctl_1_field_info with %d entries\n", sizeof(c2h_channel_ctl_1_field_info)/sizeof(c2h_channel_ctl_1_field_info[0]));
};

// MD : Structure to hold register field information for C2H MM status
static struct regfield_info c2h_mm_status_field_info[] = {
    {"C2H_MM_STATUS_RSVD_1", C2H_MM_STATUS_RSVD_1_MASK},
    {"C2H_MM_STATUS_RUN", C2H_MM_STATUS_RUN_MASK},
    // MD : Debug: Track initialization of C2H MM status field info
    printk(KERN_DEBUG "Initialized c2h_mm_status_field_info with %d entries\n", sizeof(c2h_mm_status_field_info)/sizeof(c2h_mm_status_field_info[0]));
};

// MD : Structure to hold register field information for C2H channel completion descriptor count
static struct regfield_info c2h_channel_cmpl_desc_cnt_field_info[] = {
    {"C2H_CHANNEL_CMPL_DESC_CNT_C2H_CO", C2H_CHANNEL_CMPL_DESC_CNT_C2H_CO_MASK},
    // MD : Debug: Track initialization of C2H channel completion descriptor count field info
    printk(KERN_DEBUG "Initialized c2h_channel_cmpl_desc_cnt_field_info with %d entries\n", sizeof(c2h_channel_cmpl_desc_cnt_field_info)/sizeof(c2h_channel_cmpl_desc_cnt_field_info[0]));
};

// MD : Structure to hold register field information for C2H MM error code enable mask
static struct regfield_info c2h_mm_err_code_enable_mask_field_info[] = {
    {"C2H_MM_ERR_CODE_ENABLE_RSVD_1", C2H_MM_ERR_CODE_ENABLE_RSVD_1_MASK},
    {"C2H_MM_ERR_CODE_ENABLE_WR_UC_RAM", C2H_MM_ERR_CODE_ENABLE_WR_UC_RAM_MASK},
    {"C2H_MM_ERR_CODE_ENABLE_WR_UR", C2H_MM_ERR_CODE_ENABLE_WR_UR_MASK},
    {"C2H_MM_ERR_CODE_ENABLE_WR_FLR", C2H_MM_ERR_CODE_ENABLE_WR_FLR_MASK},
    {"C2H_MM_ERR_CODE_ENABLE_RSVD_2", C2H_MM_ERR_CODE_ENABLE_RSVD_2_MASK},
    {"C2H_MM_ERR_CODE_ENABLE_RD_SLV_ERR", C2H_MM_ERR_CODE_ENABLE_RD_SLV_ERR_MASK},
    {"C2H_MM_ERR_CODE_ENABLE_WR_SLV_ERR", C2H_MM_ERR_CODE_ENABLE_WR_SLV_ERR_MASK},
    // MD : Debug: Track initialization of C2H MM error code enable mask field info
    printk(KERN_DEBUG "Initialized c2h_mm_err_code_enable_mask_field_info with %d entries\n", sizeof(c2h_mm_err_code_enable_mask_field_info)/sizeof(c2h_mm_err_code_enable_mask_field_info[0]));
};

// MD : Structure to hold register field information for C2H MM error code
static struct regfield_info c2h_mm_err_code_field_info[] = {
    {"C2H_MM_ERR_CODE_RSVD_1", C2H_MM_ERR_CODE_RSVD_1_MASK},
    {"C2H_MM_ERR_CODE_VALID", C2H_MM_ERR_CODE_VALID_MASK},
    {"C2H_MM_ERR_CODE_RDWR", C2H_MM_ERR_CODE_RDWR_MASK},
    {"C2H_MM_ERR_CODE", C2H_MM_ERR_CODE_MASK},
    // MD : Debug: Track initialization of C2H MM error code field info
    printk(KERN_DEBUG "Initialized c2h_mm_err_code_field_info with %d entries\n", sizeof(c2h_mm_err_code_field_info)/sizeof(c2h_mm_err_code_field_info[0]));
};

// MD : Structure to hold register field information for C2H MM error info
static struct regfield_info c2h_mm_err_info_field_info[] = {
    {"C2H_MM_ERR_INFO_RSVD_1", C2H_MM_ERR_INFO_RSVD_1_MASK},
    {"C2H_MM_ERR_INFO_QID", C2H_MM_ERR_INFO_QID_MASK},
    {"C2H_MM_ERR_INFO_DIR", C2H_MM_ERR_INFO_DIR_MASK},
    {"C2H_MM_ERR_INFO_CIDX", C2H_MM_ERR_INFO_CIDX_MASK},
    // MD : Debug: Track initialization of C2H MM error info field info
    printk(KERN_DEBUG "Initialized c2h_mm_err_info_field_info with %d entries\n", sizeof(c2h_mm_err_info_field_info)/sizeof(c2h_mm_err_info_field_info[0]));
};

// MD : Structure to hold register field information for C2H MM performance monitor control
static struct regfield_info c2h_mm_perf_mon_ctl_field_info[] = {
    {"C2H_MM_PERF_MON_CTL_RSVD_1", C2H_MM_PERF_MON_CTL_RSVD_1_MASK},
    {"C2H_MM_PERF_MON_CTL_IMM_START", C2H_MM_PERF_MON_CTL_IMM_START_MASK},
    {"C2H_MM_PERF_MON_CTL_RUN_START", C2H_MM_PERF_MON_CTL_RUN_START_MASK},
    {"C2H_MM_PERF_MON_CTL_IMM_CLEAR", C2H_MM_PERF_MON_CTL_IMM_CLEAR_MASK},
    {"C2H_MM_PERF_MON_CTL_RUN_CLEAR", C2H_MM_PERF_MON_CTL_RUN_CLEAR_MASK},
    // MD : Debug: Track initialization of C2H MM performance monitor control field info
    printk(KERN_DEBUG "Initialized c2h_mm_perf_mon_ctl_field_info with %d entries\n", sizeof(c2h_mm_perf_mon_ctl_field_info)/sizeof(c2h_mm_perf_mon_ctl_field_info[0]));
};

// MD : Structure to hold register field information for C2H MM performance monitor cycle count 0
static struct regfield_info c2h_mm_perf_mon_cycle_cnt0_field_info[] = {
    {"C2H_MM_PERF_MON_CYCLE_CNT0_CYC_CNT", C2H_MM_PERF_MON_CYCLE_CNT0_CYC_CNT_MASK},
    // MD : Debug: Track initialization of C2H MM performance monitor cycle count 0 field info
    printk(KERN_DEBUG "Initialized c2h_mm_perf_mon_cycle_cnt0_field_info with %d entries\n", sizeof(c2h_mm_perf_mon_cycle_cnt0_field_info)/sizeof(c2h_mm_perf_mon_cycle_cnt0_field_info[0]));
};

// MD : Structure to hold register field information for C2H MM performance monitor cycle count 1
static struct regfield_info c2h_mm_perf_mon_cycle_cnt1_field_info[] = {
    {"C2H_MM_PERF_MON_CYCLE_CNT1_RSVD_1", C2H_MM_PERF_MON_CYCLE_CNT1_RSVD_1_MASK},
    {"C2H_MM_PERF_MON_CYCLE_CNT1_CYC_CNT", C2H_MM_PERF_MON_CYCLE_CNT1_CYC_CNT_MASK},
    // MD : Debug: Track initialization of C2H MM performance monitor cycle count 1 field info
    printk(KERN_DEBUG "Initialized c2h_mm_perf_mon_cycle_cnt1_field_info with %d entries\n", sizeof(c2h_mm_perf_mon_cycle_cnt1_field_info)/sizeof(c2h_mm_perf_mon_cycle_cnt1_field_info[0]));
};

// MD : Structure to hold register field information for C2H MM performance monitor data count 0
static struct regfield_info c2h_mm_perf_mon_data_cnt0_field_info[] = {
    {"C2H_MM_PERF_MON_DATA_CNT0_DCNT", C2H_MM_PERF_MON_DATA_CNT0_DCNT_MASK},
    // MD : Debug: Track initialization of C2H MM performance monitor data count 0 field info
    printk(KERN_DEBUG "Initialized c2h_mm_perf_mon_data_cnt0_field_info with %d entries\n", sizeof(c2h_mm_perf_mon_data_cnt0_field_info)/sizeof(c2h_mm_perf_mon_data_cnt0_field_info[0]));
};

// MD : Structure to hold register field information for C2H MM performance monitor data count 1
static struct regfield_info c2h_mm_perf_mon_data_cnt1_field_info[] = {
    {"C2H_MM_PERF_MON_DATA_CNT1_RSVD_1", C2H_MM_PERF_MON_DATA_CNT1_RSVD_1_MASK},
    {"C2H_MM_PERF_MON_DATA_CNT1_DCNT", C2H_MM_PERF_MON_DATA_CNT1_DCNT_MASK},
    // MD : Debug: Track initialization of C2H MM performance monitor data count 1 field info
    printk(KERN_DEBUG "Initialized c2h_mm_perf_mon_data_cnt1_field_info with %d entries\n", sizeof(c2h_mm_perf_mon_data_cnt1_field_info)/sizeof(c2h_mm_perf_mon_data_cnt1_field_info[0]));
};

// MD : Structure to hold register field information for C2H MM debug
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
    // MD : Debug: Track initialization of C2H MM debug field info
    printk(KERN_DEBUG "Initialized c2h_mm_dbg_field_info with %d entries\n", sizeof(c2h_mm_dbg_field_info)/sizeof(c2h_mm_dbg_field_info[0]));
};

// MD : Structure to hold register field information for H2C channel control
static struct regfield_info h2c_channel_ctl_field_info[] = {
    {"H2C_CHANNEL_CTL_RSVD_1", H2C_CHANNEL_CTL_RSVD_1_MASK},
    {"H2C_CHANNEL_CTL_RUN", H2C_CHANNEL_CTL_RUN_MASK},
    // MD : Debug: Track initialization of H2C channel control field info
    printk(KERN_DEBUG "Initialized h2c_channel_ctl_field_info with %d entries\n", sizeof(h2c_channel_ctl_field_info)/sizeof(h2c_channel_ctl_field_info[0]));
};

// MD : Structure to hold register field information for H2C channel control 1
static struct regfield_info h2c_channel_ctl_1_field_info[] = {
    {"H2C_CHANNEL_CTL_1_RUN", H2C_CHANNEL_CTL_1_RUN_MASK},
    // MD : Debug: Track initialization of H2C channel control 1 field info
    printk(KERN_DEBUG "Initialized h2c_channel_ctl_1_field_info with %d entries\n", sizeof(h2c_channel_ctl_1_field_info)/sizeof(h2c_channel_ctl_1_field_info[0]));
};

// MD : Structure to hold register field information for H2C channel control 2
static struct regfield_info h2c_channel_ctl_2_field_info[] = {
    {"H2C_CHANNEL_CTL_2_RUN", H2C_CHANNEL_CTL_2_RUN_MASK},
    // MD : Debug: Track initialization of H2C channel control 2 field info
    printk(KERN_DEBUG "Initialized h2c_channel_ctl_2_field_info with %d entries\n", sizeof(h2c_channel_ctl_2_field_info)/sizeof(h2c_channel_ctl_2_field_info[0]));
};

// MD : Structure to hold register field information for H2C MM status
static struct regfield_info h2c_mm_status_field_info[] = {
    {"H2C_MM_STATUS_RSVD_1", H2C_MM_STATUS_RSVD_1_MASK},
    {"H2C_MM_STATUS_RUN", H2C_MM_STATUS_RUN_MASK},
    // MD : Debug: Track initialization of H2C MM status field info
    printk(KERN_DEBUG "Initialized h2c_mm_status_field_info with %d entries\n", sizeof(h2c_mm_status_field_info)/sizeof(h2c_mm_status_field_info[0]));
};

// MD : Structure to hold register field information for H2C channel complete descriptor count
static struct regfield_info h2c_channel_cmpl_desc_cnt_field_info[] = {
    {"H2C_CHANNEL_CMPL_DESC_CNT_H2C_CO", H2C_CHANNEL_CMPL_DESC_CNT_H2C_CO_MASK},
    // MD : Debug: Track initialization of H2C channel complete descriptor count field info
    printk(KERN_DEBUG "Initialized h2c_channel_cmpl_desc_cnt_field_info with %d entries\n", sizeof(h2c_channel_cmpl_desc_cnt_field_info)/sizeof(h2c_channel_cmpl_desc_cnt_field_info[0]));
};

// MD : Structure to hold register field information for H2C MM error code enable mask
static struct regfield_info h2c_mm_err_code_enable_mask_field_info[] = {
    {"H2C_MM_ERR_CODE_ENABLE_RSVD_1", H2C_MM_ERR_CODE_ENABLE_RSVD_1_MASK},
    {"H2C_MM_ERR_CODE_ENABLE_WR_SLV_ERR", H2C_MM_ERR_CODE_ENABLE_WR_SLV_ERR_MASK},
    {"H2C_MM_ERR_CODE_ENABLE_WR_DEC_ERR", H2C_MM_ERR_CODE_ENABLE_WR_DEC_ERR_MASK},
    {"H2C_MM_ERR_CODE_ENABLE_RSVD_2", H2C_MM_ERR_CODE_ENABLE_RSVD_2_MASK},
    {"H2C_MM_ERR_CODE_ENABLE_RD_RQ_DIS_ERR", H2C_MM_ERR_CODE_ENABLE_RD_RQ_DIS_ERR_MASK},
    {"H2C_MM_ERR_CODE_ENABLE_RSVD_3", H2C_MM_ERR_CODE_ENABLE_RSVD_3_MASK},
    {"H2C_MM_ERR_CODE_ENABLE_RD_DAT_POISON_ERR", H2C_MM_ERR_CODE_ENABLE_RD_DAT_POISON_ERR_MASK},
    {"H2C_MM_ERR_CODE_ENABLE_RSVD_4", H2C_MM_ERR_CODE_ENABLE_RSVD_4_MASK},
    {"H2C_MM_ERR_CODE_ENABLE_RD_FLR_ERR", H2C_MM_ERR_CODE_ENABLE_RD_FLR_ERR_MASK},
    {"H2C_MM_ERR_CODE_ENABLE_RSVD_5", H2C_MM_ERR_CODE_ENABLE_RSVD_5_MASK},
    {"H2C_MM_ERR_CODE_ENABLE_RD_HDR_ADR_ERR", H2C_MM_ERR_CODE_ENABLE_RD_HDR_ADR_ERR_MASK},
    {"H2C_MM_ERR_CODE_ENABLE_RD_HDR_PARA", H2C_MM_ERR_CODE_ENABLE_RD_HDR_PARA_MASK},
    {"H2C_MM_ERR_CODE_ENABLE_RD_HDR_BYTE_ERR", H2C_MM_ERR_CODE_ENABLE_RD_HDR_BYTE_ERR_MASK},
    {"H2C_MM_ERR_CODE_ENABLE_RD_UR_CA", H2C_MM_ERR_CODE_ENABLE_RD_UR_CA_MASK},
    {"H2C_MM_ERR_CODE_ENABLE_RD_HRD_POISON_ERR", H2C_MM_ERR_CODE_ENABLE_RD_HRD_POISON_ERR_MASK},
    {"H2C_MM_ERR_CODE_ENABLE_RSVD_6", H2C_MM_ERR_CODE_ENABLE_RSVD_6_MASK},
    // MD : Debug: Track initialization of H2C MM error code enable mask field info
    printk(KERN_DEBUG "Initialized h2c_mm_err_code_enable_mask_field_info with %d entries\n", sizeof(h2c_mm_err_code_enable_mask_field_info)/sizeof(h2c_mm_err_code_enable_mask_field_info[0]));
};

// MD : Structure to hold register field information for H2C MM error code
static struct regfield_info h2c_mm_err_code_field_info[] = {
    {"H2C_MM_ERR_CODE_RSVD_1", H2C_MM_ERR_CODE_RSVD_1_MASK},
    {"H2C_MM_ERR_CODE_VALID", H2C_MM_ERR_CODE_VALID_MASK},
    {"H2C_MM_ERR_CODE_RDWR", H2C_MM_ERR_CODE_RDWR_MASK},
    {"H2C_MM_ERR_CODE", H2C_MM_ERR_CODE_MASK},
    // MD : Debug: Track initialization of H2C MM error code field info
    printk(KERN_DEBUG "Initialized h2c_mm_err_code_field_info with %d entries\n", sizeof(h2c_mm_err_code_field_info)/sizeof(h2c_mm_err_code_field_info[0]));
};

// MD : Structure to hold register field information for H2C MM error info
static struct regfield_info h2c_mm_err_info_field_info[] = {
    {"H2C_MM_ERR_INFO_RSVD_1", H2C_MM_ERR_INFO_RSVD_1_MASK},
    {"H2C_MM_ERR_INFO_QID", H2C_MM_ERR_INFO_QID_MASK},
    {"H2C_MM_ERR_INFO_DIR", H2C_MM_ERR_INFO_DIR_MASK},
    {"H2C_MM_ERR_INFO_CIDX", H2C_MM_ERR_INFO_CIDX_MASK},
    // MD : Debug: Track initialization of H2C MM error info field info
    printk(KERN_DEBUG "Initialized h2c_mm_err_info_field_info with %d entries\n", sizeof(h2c_mm_err_info_field_info)/sizeof(h2c_mm_err_info_field_info[0]));
};

// MD : Structure to hold register field information for H2C MM performance monitor control
static struct regfield_info h2c_mm_perf_mon_ctl_field_info[] = {
    {"H2C_MM_PERF_MON_CTL_RSVD_1", H2C_MM_PERF_MON_CTL_RSVD_1_MASK},
    {"H2C_MM_PERF_MON_CTL_IMM_START", H2C_MM_PERF_MON_CTL_IMM_START_MASK},
    {"H2C_MM_PERF_MON_CTL_RUN_START", H2C_MM_PERF_MON_CTL_RUN_START_MASK},
    {"H2C_MM_PERF_MON_CTL_IMM_CLEAR", H2C_MM_PERF_MON_CTL_IMM_CLEAR_MASK},
    {"H2C_MM_PERF_MON_CTL_RUN_CLEAR", H2C_MM_PERF_MON_CTL_RUN_CLEAR_MASK},
    // MD : Debug: Track initialization of H2C MM performance monitor control field info
    printk(KERN_DEBUG "Initialized h2c_mm_perf_mon_ctl_field_info with %d entries\n", sizeof(h2c_mm_perf_mon_ctl_field_info)/sizeof(h2c_mm_perf_mon_ctl_field_info[0]));
};

// MD : Structure to hold register field information for H2C MM performance monitor cycle count 0
static struct regfield_info h2c_mm_perf_mon_cycle_cnt0_field_info[] = {
    {"H2C_MM_PERF_MON_CYCLE_CNT0_CYC_CNT", H2C_MM_PERF_MON_CYCLE_CNT0_CYC_CNT_MASK},
    // MD : Debug: Track initialization of H2C MM performance monitor cycle count 0 field info
    printk(KERN_DEBUG "Initialized h2c_mm_perf_mon_cycle_cnt0_field_info with %d entries\n", sizeof(h2c_mm_perf_mon_cycle_cnt0_field_info)/sizeof(h2c_mm_perf_mon_cycle_cnt0_field_info[0]));
};

// MD : Structure to hold register field information for H2C MM performance monitor cycle count 1
static struct regfield_info h2c_mm_perf_mon_cycle_cnt1_field_info[] = {
    {"H2C_MM_PERF_MON_CYCLE_CNT1_RSVD_1", H2C_MM_PERF_MON_CYCLE_CNT1_RSVD_1_MASK},
    {"H2C_MM_PERF_MON_CYCLE_CNT1_CYC_CNT", H2C_MM_PERF_MON_CYCLE_CNT1_CYC_CNT_MASK},
    // MD : Debug: Track initialization of H2C MM performance monitor cycle count 1 field info
    printk(KERN_DEBUG "Initialized h2c_mm_perf_mon_cycle_cnt1_field_info with %d entries\n", sizeof(h2c_mm_perf_mon_cycle_cnt1_field_info)/sizeof(h2c_mm_perf_mon_cycle_cnt1_field_info[0]));
};

// MD : Structure to hold register field information for H2C MM performance monitor data count 0
static struct regfield_info h2c_mm_perf_mon_data_cnt0_field_info[] = {
    {"H2C_MM_PERF_MON_DATA_CNT0_DCNT", H2C_MM_PERF_MON_DATA_CNT0_DCNT_MASK},
    // MD : Debug: Track initialization of H2C MM performance monitor data count 0 field info
    printk(KERN_DEBUG "Initialized h2c_mm_perf_mon_data_cnt0_field_info with %d entries\n", sizeof(h2c_mm_perf_mon_data_cnt0_field_info)/sizeof(h2c_mm_perf_mon_data_cnt0_field_info[0]));
};

// MD : Structure to hold register field information for H2C MM performance monitor data count 1
static struct regfield_info h2c_mm_perf_mon_data_cnt1_field_info[] = {
    {"H2C_MM_PERF_MON_DATA_CNT1_RSVD_1", H2C_MM_PERF_MON_DATA_CNT1_RSVD_1_MASK},
    {"H2C_MM_PERF_MON_DATA_CNT1_DCNT", H2C_MM_PERF_MON_DATA_CNT1_DCNT_MASK},
    // MD : Debug: Track initialization of H2C MM performance monitor data count 1 field info
    printk(KERN_DEBUG "Initialized h2c_mm_perf_mon_data_cnt1_field_info with %d entries\n", sizeof(h2c_mm_perf_mon_data_cnt1_field_info)/sizeof(h2c_mm_perf_mon_data_cnt1_field_info[0]));
};

// MD : Structure to hold register field information for H2C MM debug
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
    // MD : Debug: Track initialization of H2C MM debug field info
    printk(KERN_DEBUG "Initialized h2c_mm_dbg_field_info with %d entries\n", sizeof(h2c_mm_dbg_field_info)/sizeof(h2c_mm_dbg_field_info[0]));
};

// MD : Structure to hold register field information for function status register
static struct regfield_info func_status_reg_field_info[] = {
    {"FUNC_STATUS_REG_RSVD_1", FUNC_STATUS_REG_RSVD_1_MASK},
    {"FUNC_STATUS_REG_CUR_SRC_FN", FUNC_STATUS_REG_CUR_SRC_FN_MASK},
    {"FUNC_STATUS_REG_ACK", FUNC_STATUS_REG_ACK_MASK},
    {"FUNC_STATUS_REG_O_MSG", FUNC_STATUS_REG_O_MSG_MASK},
    {"FUNC_STATUS_REG_I_MSG", FUNC_STATUS_REG_I_MSG_MASK},
    // MD : Debug: Track initialization of function status register field info
    printk(KERN_DEBUG "Initialized func_status_reg_field_info with %d entries\n", sizeof(func_status_reg_field_info)/sizeof(func_status_reg_field_info[0]));
};

// MD : Structure to hold register field information for function command register
static struct regfield_info func_cmd_reg_field_info[] = {
    {"FUNC_CMD_REG_RSVD_1", FUNC_CMD_REG_RSVD_1_MASK},
    {"FUNC_CMD_REG_RSVD_2", FUNC_CMD_REG_RSVD_2_MASK},
    {"FUNC_CMD_REG_MSG_RCV", FUNC_CMD_REG_MSG_RCV_MASK},
    {"FUNC_CMD_REG_MSG_SENT", FUNC_CMD_REG_MSG_SENT_MASK},
    // MD : Debug: Track initialization of function command register field info
    printk(KERN_DEBUG "Initialized func_cmd_reg_field_info with %d entries\n", sizeof(func_cmd_reg_field_info)/sizeof(func_cmd_reg_field_info[0]));
};

// MD : Structure to hold register field information for function interrupt vector register
static struct regfield_info func_interrupt_vector_reg_field_info[] = {
    {"FUNC_INTERRUPT_VECTOR_REG_RSVD_1", FUNC_INTERRUPT_VECTOR_REG_RSVD_1_MASK},
    {"FUNC_INTERRUPT_VECTOR_REG_IN", FUNC_INTERRUPT_VECTOR_REG_IN_MASK},
    // MD : Debug: Track initialization of function interrupt vector register field info
    printk(KERN_DEBUG "Initialized func_interrupt_vector_reg_field_info with %d entries\n", sizeof(func_interrupt_vector_reg_field_info)/sizeof(func_interrupt_vector_reg_field_info[0]));
};

// MD : Structure to hold register field information for target function register
static struct regfield_info target_func_reg_field_info[] = {
    {"TARGET_FUNC_REG_RSVD_1", TARGET_FUNC_REG_RSVD_1_MASK},
    {"TARGET_FUNC_REG_N_ID", TARGET_FUNC_REG_N_ID_MASK},
    // MD : Debug: Track initialization of target function register field info
    printk(KERN_DEBUG "Initialized target_func_reg_field_info with %d entries\n", sizeof(target_func_reg_field_info)/sizeof(target_func_reg_field_info[0]));
};

// MD : Structure to hold register field information for function interrupt control register
static struct regfield_info func_interrupt_ctl_reg_field_info[] = {
    {"FUNC_INTERRUPT_CTL_REG_RSVD_1", FUNC_INTERRUPT_CTL_REG_RSVD_1_MASK},
    {"FUNC_INTERRUPT_CTL_REG_INT_EN", FUNC_INTERRUPT_CTL_REG_INT_EN_MASK},
    // MD : Debug: Track initialization of function interrupt control register field info
    printk(KERN_DEBUG "Initialized func_interrupt_ctl_reg_field_info with %d entries\n", sizeof(func_interrupt_ctl_reg_field_info)/sizeof(func_interrupt_ctl_reg_field_info[0]));
};

// MD : Structure to hold configuration register information for QDMA CPM4
static struct xreg_info qdma_cpm4_config_regs[] = {
{"CFG_BLK_IDENTIFIER", 0x00,
    1, 0, 0, 0,
    0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
    ARRAY_SIZE(cfg_blk_identifier_field_info),
    cfg_blk_identifier_field_info
},
// MD : Debug: Track initialization of CFG_BLK_IDENTIFIER register
printk(KERN_DEBUG "Initialized CFG_BLK_IDENTIFIER register with address 0x00\n"),

{"CFG_BLK_BUSDEV", 0x04,
    1, 0, 0, 0,
    0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
    ARRAY_SIZE(cfg_blk_busdev_field_info),
    cfg_blk_busdev_field_info
},
// MD : Debug: Track initialization of CFG_BLK_BUSDEV register
printk(KERN_DEBUG "Initialized CFG_BLK_BUSDEV register with address 0x04\n"),

{"CFG_BLK_PCIE_MAX_PLD_SIZE", 0x08,
    1, 0, 0, 0,
    0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
    ARRAY_SIZE(cfg_blk_pcie_max_pld_size_field_info),
    cfg_blk_pcie_max_pld_size_field_info
},
// MD : Debug: Track initialization of CFG_BLK_PCIE_MAX_PLD_SIZE register
printk(KERN_DEBUG "Initialized CFG_BLK_PCIE_MAX_PLD_SIZE register with address 0x08\n"),

{"CFG_BLK_PCIE_MAX_READ_REQ_SIZE", 0x0c,
    1, 0, 0, 0,
    0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
    ARRAY_SIZE(cfg_blk_pcie_max_read_req_size_field_info),
    cfg_blk_pcie_max_read_req_size_field_info
},
// MD : Debug: Track initialization of CFG_BLK_PCIE_MAX_READ_REQ_SIZE register
printk(KERN_DEBUG "Initialized CFG_BLK_PCIE_MAX_READ_REQ_SIZE register with address 0x0c\n"),

{"CFG_BLK_SYSTEM_ID", 0x10,
    1, 0, 0, 0,
    0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
    ARRAY_SIZE(cfg_blk_system_id_field_info),
    cfg_blk_system_id_field_info
},
// MD : Debug: Track initialization of CFG_BLK_SYSTEM_ID register
printk(KERN_DEBUG "Initialized CFG_BLK_SYSTEM_ID register with address 0x10\n"),

{"CFG_BLK_MSI_ENABLE", 0x014,
    1, 0, 0, 0,
    0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
    ARRAY_SIZE(cfg_blk_msi_enable_field_info),
    cfg_blk_msi_enable_field_info
},
// MD : Debug: Track initialization of CFG_BLK_MSI_ENABLE register
printk(KERN_DEBUG "Initialized CFG_BLK_MSI_ENABLE register with address 0x014\n"),

{"CFG_PCIE_DATA_WIDTH", 0x18,
    1, 0, 0, 0,
    0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
    ARRAY_SIZE(cfg_pcie_data_width_field_info),
    cfg_pcie_data_width_field_info
},
// MD : Debug: Track initialization of CFG_PCIE_DATA_WIDTH register
printk(KERN_DEBUG "Initialized CFG_PCIE_DATA_WIDTH register with address 0x18\n"),

{"CFG_PCIE_CTL", 0x1c,
    1, 0, 0, 0,
    0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
    ARRAY_SIZE(cfg_pcie_ctl_field_info),
    cfg_pcie_ctl_field_info
},
// MD : Debug: Track initialization of CFG_PCIE_CTL register
printk(KERN_DEBUG "Initialized CFG_PCIE_CTL register with address 0x1c\n"),

{"CFG_AXI_USER_MAX_PLD_SIZE", 0x40,
1, 0, 0, 0,
0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
ARRAY_SIZE(cfg_axi_user_max_pld_size_field_info),
cfg_axi_user_max_pld_size_field_info
},
// MD : Debug: Track initialization of AXI user max payload size field info
printk(KERN_DEBUG "Initialized cfg_axi_user_max_pld_size_field_info\n");

// MD : Structure to hold register field information for AXI user max read request size
{"CFG_AXI_USER_MAX_READ_REQ_SIZE", 0x44,
1, 0, 0, 0,
0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
ARRAY_SIZE(cfg_axi_user_max_read_req_size_field_info),
cfg_axi_user_max_read_req_size_field_info
},
// MD : Debug: Track initialization of AXI user max read request size field info
printk(KERN_DEBUG "Initialized cfg_axi_user_max_read_req_size_field_info\n");

// MD : Structure to hold register field information for miscellaneous control fields
{"CFG_BLK_MISC_CTL", 0x4c,
1, 0, 0, 0,
0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
ARRAY_SIZE(cfg_blk_misc_ctl_field_info),
cfg_blk_misc_ctl_field_info
},
// MD : Debug: Track initialization of miscellaneous control field info
printk(KERN_DEBUG "Initialized cfg_blk_misc_ctl_field_info\n");

// MD : Structure to hold register field information for scratch registers
{"CFG_BLK_SCRATCH_0", 0x80,
1, 0, 0, 0,
0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
ARRAY_SIZE(cfg_blk_scratch_0_field_info),
cfg_blk_scratch_0_field_info
},
// MD : Debug: Track initialization of scratch 0 field info
printk(KERN_DEBUG "Initialized cfg_blk_scratch_0_field_info\n");

{"CFG_BLK_SCRATCH_1", 0x84,
1, 0, 0, 0,
0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
ARRAY_SIZE(cfg_blk_scratch_1_field_info),
cfg_blk_scratch_1_field_info
},
// MD : Debug: Track initialization of scratch 1 field info
printk(KERN_DEBUG "Initialized cfg_blk_scratch_1_field_info\n");

{"CFG_BLK_SCRATCH_2", 0x88,
1, 0, 0, 0,
0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
ARRAY_SIZE(cfg_blk_scratch_2_field_info),
cfg_blk_scratch_2_field_info
},
// MD : Debug: Track initialization of scratch 2 field info
printk(KERN_DEBUG "Initialized cfg_blk_scratch_2_field_info\n");

{"CFG_BLK_SCRATCH_3", 0x8c,
1, 0, 0, 0,
0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
ARRAY_SIZE(cfg_blk_scratch_3_field_info),
cfg_blk_scratch_3_field_info
},
// MD : Debug: Track initialization of scratch 3 field info
printk(KERN_DEBUG "Initialized cfg_blk_scratch_3_field_info\n");

{"CFG_BLK_SCRATCH_4", 0x90,
1, 0, 0, 0,
0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
ARRAY_SIZE(cfg_blk_scratch_4_field_info),
cfg_blk_scratch_4_field_info
},
// MD : Debug: Track initialization of scratch 4 field info
printk(KERN_DEBUG "Initialized cfg_blk_scratch_4_field_info\n");

{"CFG_BLK_SCRATCH_5", 0x94,
1, 0, 0, 0,
0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
ARRAY_SIZE(cfg_blk_scratch_5_field_info),
cfg_blk_scratch_5_field_info
},
// MD : Debug: Track initialization of scratch 5 field info
printk(KERN_DEBUG "Initialized cfg_blk_scratch_5_field_info\n");

{"CFG_BLK_SCRATCH_6", 0x98,
1, 0, 0, 0,
0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
ARRAY_SIZE(cfg_blk_scratch_6_field_info),
cfg_blk_scratch_6_field_info
},
// MD : Debug: Track initialization of scratch 6 field info
printk(KERN_DEBUG "Initialized cfg_blk_scratch_6_field_info\n");

{"CFG_BLK_SCRATCH_7", 0x9c,
1, 0, 0, 0,
0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
ARRAY_SIZE(cfg_blk_scratch_7_field_info),
cfg_blk_scratch_7_field_info
},
// MD : Debug: Track initialization of scratch 7 field info
printk(KERN_DEBUG "Initialized cfg_blk_scratch_7_field_info\n");

// MD : Structure to hold register field information for RAM single-bit error mask A
{"RAM_SBE_MSK_A", 0xf0,
1, 0, 0, 0,
0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
ARRAY_SIZE(ram_sbe_msk_a_field_info),
ram_sbe_msk_a_field_info
},
// MD : Debug: Track initialization of RAM SBE mask A field info
printk(KERN_DEBUG "Initialized ram_sbe_msk_a_field_info\n");

// MD : Structure to hold register field information for RAM single-bit error status A
{"RAM_SBE_STS_A", 0xf4,
1, 0, 0, 0,
0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
ARRAY_SIZE(ram_sbe_sts_a_field_info),
ram_sbe_sts_a_field_info
},
// MD : Debug: Track initialization of RAM SBE status A field info
printk(KERN_DEBUG "Initialized ram_sbe_sts_a_field_info\n");

// MD : Structure to hold register field information for RAM double-bit error mask A
{"RAM_DBE_MSK_A", 0xf8,
1, 0, 0, 0,
0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
ARRAY_SIZE(ram_dbe_msk_a_field_info),
ram_dbe_msk_a_field_info
},
// MD : Debug: Track initialization of RAM DBE mask A field info
printk(KERN_DEBUG "Initialized ram_dbe_msk_a_field_info\n");

// MD : Structure to hold register field information for RAM double-bit error status A
{"RAM_DBE_STS_A", 0xfc,
1, 0, 0, 0,
0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
ARRAY_SIZE(ram_dbe_sts_a_field_info),
ram_dbe_sts_a_field_info
},
// MD : Debug: Track initialization of RAM DBE status A field info
printk(KERN_DEBUG "Initialized ram_dbe_sts_a_field_info\n");

// MD : Structure to hold register field information for global identifier
{"GLBL2_IDENTIFIER", 0x100,
1, 0, 0, 0,
0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
ARRAY_SIZE(glbl2_identifier_field_info),
glbl2_identifier_field_info
},
// MD : Debug: Track initialization of global identifier field info
printk(KERN_DEBUG "Initialized glbl2_identifier_field_info\n");

// MD : Structure to hold register field information for PF BARLITE internal mapping
{"GLBL2_PF_BARLITE_INT", 0x104,
1, 0, 0, 0,
0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
ARRAY_SIZE(glbl2_pf_barlite_int_field_info),
glbl2_pf_barlite_int_field_info
},
// MD : Debug: Track initialization of PF BARLITE internal field info
printk(KERN_DEBUG "Initialized glbl2_pf_barlite_int_field_info\n");

// MD : Structure to hold register field information for PF VF BARLITE internal mapping
{"GLBL2_PF_VF_BARLITE_INT", 0x108,
1, 0, 0, 0,
0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
ARRAY_SIZE(glbl2_pf_vf_barlite_int_field_info),
glbl2_pf_vf_barlite_int_field_info
},
// MD : Debug: Track initialization of PF VF BARLITE internal field info
printk(KERN_DEBUG "Initialized glbl2_pf_vf_barlite_int_field_info\n");

// MD : Structure to hold register field information for PF BARLITE external mapping
{"GLBL2_PF_BARLITE_EXT", 0x10c,
1, 0, 0, 0,
0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
ARRAY_SIZE(glbl2_pf_barlite_ext_field_info),
glbl2_pf_barlite_ext_field_info
},
// MD : Debug: Track initialization of PF BARLITE external field info
printk(KERN_DEBUG "Initialized glbl2_pf_barlite_ext_field_info\n");

// MD : Structure to hold register field information for PF VF BARLITE external mapping
{"GLBL2_PF_VF_BARLITE_EXT", 0x110,
1, 0, 0, 0,
0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
ARRAY_SIZE(glbl2_pf_vf_barlite_ext_field_info),
glbl2_pf_vf_barlite_ext_field_info
},
// MD : Debug: Track initialization of PF VF BARLITE external field info
printk(KERN_DEBUG "Initialized glbl2_pf_vf_barlite_ext_field_info\n");

// MD : Structure to hold register field information for channel instance
{"GLBL2_CHANNEL_INST", 0x114,
1, 0, 0, 0,
0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
ARRAY_SIZE(glbl2_channel_inst_field_info),
glbl2_channel_inst_field_info
},
// MD : Debug: Track initialization of channel instance field info
printk(KERN_DEBUG "Initialized glbl2_channel_inst_field_info\n");

// MD : Structure to hold register field information for channel MDMA
{"GLBL2_CHANNEL_MDMA", 0x118,
1, 0, 0, 0,
0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
ARRAY_SIZE(glbl2_channel_mdma_field_info),
glbl2_channel_mdma_field_info
},
// MD : Debug: Track initialization of channel MDMA field info
printk(KERN_DEBUG "Initialized glbl2_channel_mdma_field_info\n");

// MD : Structure to hold register field information for channel stream
{"GLBL2_CHANNEL_STRM", 0x11c,
1, 0, 0, 0,
0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
ARRAY_SIZE(glbl2_channel_strm_field_info),
glbl2_channel_strm_field_info
},
// MD : Debug: Track initialization of channel stream field info
printk(KERN_DEBUG "Initialized glbl2_channel_strm_field_info\n");

// MD : Structure to hold register field information for channel capabilities
{"GLBL2_CHANNEL_CAP", 0x120,
1, 0, 0, 0,
0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
ARRAY_SIZE(glbl2_channel_cap_field_info),
glbl2_channel_cap_field_info
},
// MD : Debug: Track initialization of channel capabilities field info
printk(KERN_DEBUG "Initialized glbl2_channel_cap_field_info\n");

// MD : Structure to hold register field information for channel PASID capabilities
{"GLBL2_CHANNEL_PASID_CAP", 0x128,
1, 0, 0, 0,
0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
ARRAY_SIZE(glbl2_channel_pasid_cap_field_info),
glbl2_channel_pasid_cap_field_info
},
// MD : Debug: Track initialization of channel PASID capabilities field info
printk(KERN_DEBUG "Initialized glbl2_channel_pasid_cap_field_info\n");

// MD : Structure to hold register field information for channel function return
{"GLBL2_CHANNEL_FUNC_RET", 0x12c,
1, 0, 0, 0,
0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
ARRAY_SIZE(glbl2_channel_func_ret_field_info),
glbl2_channel_func_ret_field_info
},
// MD : Debug: Track initialization of channel function return field info
printk(KERN_DEBUG "Initialized glbl2_channel_func_ret_field_info\n");

// MD : Structure to hold register field information for system ID
{"GLBL2_SYSTEM_ID", 0x130,
1, 0, 0, 0,
0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
ARRAY_SIZE(glbl2_system_id_field_info),
glbl2_system_id_field_info
},
// MD : Debug: Track initialization of system ID field info
printk(KERN_DEBUG "Initialized glbl2_system_id_field_info\n");

// MD : Structure to hold register field information for miscellaneous capabilities
{"GLBL2_MISC_CAP", 0x134,
1, 0, 0, 0,
0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
ARRAY_SIZE(glbl2_misc_cap_field_info),
glbl2_misc_cap_field_info
},
// MD : Debug: Track initialization of miscellaneous capabilities field info
printk(KERN_DEBUG "Initialized glbl2_misc_cap_field_info\n");

// MD : Structure to hold register field information for PCIe request queue 0 debug
{"GLBL2_DBG_PCIE_RQ0", 0x1b8,
1, 0, 0, 0,
0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
ARRAY_SIZE(glbl2_dbg_pcie_rq0_field_info),
glbl2_dbg_pcie_rq0_field_info
},
// MD : Debug: Track initialization of PCIe request queue 0 debug field info
printk(KERN_DEBUG "Initialized glbl2_dbg_pcie_rq0_field_info\n");

// MD : Structure to hold register field information for PCIe request queue 1 debug
{"GLBL2_DBG_PCIE_RQ1", 0x1bc,
1, 0, 0, 0,
0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
ARRAY_SIZE(glbl2_dbg_pcie_rq1_field_info),
glbl2_dbg_pcie_rq1_field_info
},
// MD : Debug: Track initialization of PCIe request queue 1 debug field info
printk(KERN_DEBUG "Initialized glbl2_dbg_pcie_rq1_field_info\n");

// MD : Structure to hold register field information for AXI MM write channel 0 debug
{"GLBL2_DBG_AXIMM_WR0", 0x1c0,
1, 0, 0, 0,
0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
ARRAY_SIZE(glbl2_dbg_aximm_wr0_field_info),
glbl2_dbg_aximm_wr0_field_info
},
// MD : Debug: Track initialization of AXI MM write channel 0 debug field info
printk(KERN_DEBUG "Initialized glbl2_dbg_aximm_wr0_field_info\n");

// MD : Structure to hold register field information for AXI MM write channel 1 debug
{"GLBL2_DBG_AXIMM_WR1", 0x1c4,
1, 0, 0, 0,
0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
ARRAY_SIZE(glbl2_dbg_aximm_wr1_field_info),
glbl2_dbg_aximm_wr1_field_info
},
// MD : Debug: Track initialization of AXI MM write channel 1 debug field info
printk(KERN_DEBUG "Initialized glbl2_dbg_aximm_wr1_field_info\n");

// MD : Structure to hold register field information for AXI MM read channel 0 debug
{"GLBL2_DBG_AXIMM_RD0", 0x1c8,
1, 0, 0, 0,
0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
ARRAY_SIZE(glbl2_dbg_aximm_rd0_field_info),
glbl2_dbg_aximm_rd0_field_info
},
// MD : Debug: Track initialization of AXI MM read channel 0 debug field info
printk(KERN_DEBUG "Initialized glbl2_dbg_aximm_rd0_field_info\n");

// MD : Register configuration for GLBL2 AXIMM Read Channel 1 Debug
{"GLBL2_DBG_AXIMM_RD1", 0x1cc,
    1, 0, 0, 0,
    0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
    ARRAY_SIZE(glbl2_dbg_aximm_rd1_field_info),
    glbl2_dbg_aximm_rd1_field_info
},

// MD : Register configuration for Global Ring Size 1
{"GLBL_RNG_SZ_1", 0x204,
    1, 0, 0, 0,
    0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
    ARRAY_SIZE(glbl_rng_sz_1_field_info),
    glbl_rng_sz_1_field_info
},

// MD : Register configuration for Global Ring Size 2
{"GLBL_RNG_SZ_2", 0x208,
    1, 0, 0, 0,
    0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
    ARRAY_SIZE(glbl_rng_sz_2_field_info),
    glbl_rng_sz_2_field_info
},

// MD : Register configuration for Global Ring Size 3
{"GLBL_RNG_SZ_3", 0x20c,
    1, 0, 0, 0,
    0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
    ARRAY_SIZE(glbl_rng_sz_3_field_info),
    glbl_rng_sz_3_field_info
},

// MD : Register configuration for Global Ring Size 4
{"GLBL_RNG_SZ_4", 0x210,
    1, 0, 0, 0,
    0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
    ARRAY_SIZE(glbl_rng_sz_4_field_info),
    glbl_rng_sz_4_field_info
},

// MD : Register configuration for Global Ring Size 5
{"GLBL_RNG_SZ_5", 0x214,
    1, 0, 0, 0,
    0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
    ARRAY_SIZE(glbl_rng_sz_5_field_info),
    glbl_rng_sz_5_field_info
},

// MD : Register configuration for Global Ring Size 6
{"GLBL_RNG_SZ_6", 0x218,
    1, 0, 0, 0,
    0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
    ARRAY_SIZE(glbl_rng_sz_6_field_info),
    glbl_rng_sz_6_field_info
},

// MD : Register configuration for Global Ring Size 7
{"GLBL_RNG_SZ_7", 0x21c,
    1, 0, 0, 0,
    0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
    ARRAY_SIZE(glbl_rng_sz_7_field_info),
    glbl_rng_sz_7_field_info
},

// MD : Register configuration for Global Ring Size 8
{"GLBL_RNG_SZ_8", 0x220,
    1, 0, 0, 0,
    0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
    ARRAY_SIZE(glbl_rng_sz_8_field_info),
    glbl_rng_sz_8_field_info
},

// MD : Register configuration for Global Ring Size 9
{"GLBL_RNG_SZ_9", 0x224,
    1, 0, 0, 0,
    0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
    ARRAY_SIZE(glbl_rng_sz_9_field_info),
    glbl_rng_sz_9_field_info
},

// MD : Register configuration for Global Ring Size A
{"GLBL_RNG_SZ_A", 0x228,
    1, 0, 0, 0,
    0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
    ARRAY_SIZE(glbl_rng_sz_a_field_info),
    glbl_rng_sz_a_field_info
},

// MD : Register configuration for Global Ring Size B
{"GLBL_RNG_SZ_B", 0x22c,
    1, 0, 0, 0,
    0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
    ARRAY_SIZE(glbl_rng_sz_b_field_info),
    glbl_rng_sz_b_field_info
},

// MD : Register configuration for Global Ring Size C
{"GLBL_RNG_SZ_C", 0x230,
    1, 0, 0, 0,
    0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
    ARRAY_SIZE(glbl_rng_sz_c_field_info),
    glbl_rng_sz_c_field_info
},

// MD : Register configuration for Global Ring Size D
{"GLBL_RNG_SZ_D", 0x234,
    1, 0, 0, 0,
    0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
    ARRAY_SIZE(glbl_rng_sz_d_field_info),
    glbl_rng_sz_d_field_info
},

// MD : Register configuration for Global Ring Size E
{"GLBL_RNG_SZ_E", 0x238,
    1, 0, 0, 0,
    0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
    ARRAY_SIZE(glbl_rng_sz_e_field_info),
    glbl_rng_sz_e_field_info
},

// MD : Register configuration for Global Ring Size F
{"GLBL_RNG_SZ_F", 0x23c,
    1, 0, 0, 0,
    0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
    ARRAY_SIZE(glbl_rng_sz_f_field_info),
    glbl_rng_sz_f_field_info
},

// MD : Register configuration for Global Ring Size 10
{"GLBL_RNG_SZ_10", 0x240,
    1, 0, 0, 0,
    0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
    ARRAY_SIZE(glbl_rng_sz_10_field_info),
    glbl_rng_sz_10_field_info
},

// MD : Register configuration for Global Error Status
{"GLBL_ERR_STAT", 0x248,
    1, 0, 0, 0,
    0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
    ARRAY_SIZE(glbl_err_stat_field_info),
    glbl_err_stat_field_info
},

// MD : Register configuration for Global Error Mask
{"GLBL_ERR_MASK", 0x24c,
    1, 0, 0, 0,
    0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
    ARRAY_SIZE(glbl_err_mask_field_info),
    glbl_err_mask_field_info
},

// MD : Register configuration for Global Descriptor Configuration
{"GLBL_DSC_CFG", 0x250,
    1, 0, 0, 0,
    0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
    ARRAY_SIZE(glbl_dsc_cfg_field_info),
    glbl_dsc_cfg_field_info
},

// MD : Register configuration for Global Descriptor Error Status
{"GLBL_DSC_ERR_STS", 0x254,
    1, 0, 0, 0,
    0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
    ARRAY_SIZE(glbl_dsc_err_sts_field_info),
    glbl_dsc_err_sts_field_info
},

// MD : Register configuration for Global Descriptor Error Mask
{"GLBL_DSC_ERR_MSK", 0x258,
    1, 0, 0, 0,
    0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
    ARRAY_SIZE(glbl_dsc_err_msk_field_info),
    glbl_dsc_err_msk_field_info
},

// MD : Register configuration for Global Descriptor Error Log 0
{"GLBL_DSC_ERR_LOG0", 0x25c,
    1, 0, 0, 0,
    0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
    ARRAY_SIZE(glbl_dsc_err_log0_field_info),
    glbl_dsc_err_log0_field_info
},

// MD : Register configuration for Global Descriptor Error Log 1
{"GLBL_DSC_ERR_LOG1", 0x260,
    1, 0, 0, 0,
    0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
    ARRAY_SIZE(glbl_dsc_err_log1_field_info),
    glbl_dsc_err_log1_field_info
},

// MD : Register configuration for Global TRQ Error Status
{"GLBL_TRQ_ERR_STS", 0x264,
    1, 0, 0, 0,
    0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
    ARRAY_SIZE(glbl_trq_err_sts_field_info),
    glbl_trq_err_sts_field_info
},

// MD : Register configuration for Global TRQ Error Mask
{"GLBL_TRQ_ERR_MSK", 0x268,
    1, 0, 0, 0,
    0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
    ARRAY_SIZE(glbl_trq_err_msk_field_info),
    glbl_trq_err_msk_field_info
},

// MD : Register configuration for Global TRQ Error Log
{"GLBL_TRQ_ERR_LOG", 0x26c,
    1, 0, 0, 0,
    0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
    ARRAY_SIZE(glbl_trq_err_log_field_info),
    glbl_trq_err_log_field_info
},

// MD : Register configuration for Global Descriptor Debug Data 0
{"GLBL_DSC_DBG_DAT0", 0x270,
    1, 0, 0, 0,
    0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
    ARRAY_SIZE(glbl_dsc_dbg_dat0_field_info),
    glbl_dsc_dbg_dat0_field_info
},

// MD : Register configuration for Global Descriptor Debug Data 1
{"GLBL_DSC_DBG_DAT1", 0x274,
    1, 0, 0, 0,
    0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_VF,
    ARRAY_SIZE(glbl_dsc_dbg_dat1_field_info),
    glbl_dsc_dbg_dat1_field_info
},

// MD : Register configuration for TRQ Select FMap 0
{"TRQ_SEL_FMAP_0", 0x400,
    1, 0, 0, 0,
    0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
    ARRAY_SIZE(trq_sel_fmap_0_field_info),
    trq_sel_fmap_0_field_info
},

// MD : Register configuration for TRQ Select FMap 1
{"TRQ_SEL_FMAP_1", 0x404,
    1, 0, 0, 0,
    0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
    ARRAY_SIZE(trq_sel_fmap_1_field_info),
    trq_sel_fmap_1_field_info
},

// MD : Register configuration for TRQ Select FMap 2
{"TRQ_SEL_FMAP_2", 0x408,
    1, 0, 0, 0,
    0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
    ARRAY_SIZE(trq_sel_fmap_2_field_info),
    trq_sel_fmap_2_field_info
},

{"TRQ_SEL_FMAP_3", 0x40c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_3_field_info),
	trq_sel_fmap_3_field_info
},
{"TRQ_SEL_FMAP_4", 0x410,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_4_field_info),
	trq_sel_fmap_4_field_info
},
{"TRQ_SEL_FMAP_5", 0x414,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_5_field_info),
	trq_sel_fmap_5_field_info
},
{"TRQ_SEL_FMAP_6", 0x418,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_6_field_info),
	trq_sel_fmap_6_field_info
},
{"TRQ_SEL_FMAP_7", 0x41c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_7_field_info),
	trq_sel_fmap_7_field_info
},
{"TRQ_SEL_FMAP_8", 0x420,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_8_field_info),
	trq_sel_fmap_8_field_info
},
{"TRQ_SEL_FMAP_9", 0x424,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_9_field_info),
	trq_sel_fmap_9_field_info
},
{"TRQ_SEL_FMAP_A", 0x428,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_a_field_info),
	trq_sel_fmap_a_field_info
},
{"TRQ_SEL_FMAP_B", 0x42c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_b_field_info),
	trq_sel_fmap_b_field_info
},
{"TRQ_SEL_FMAP_D", 0x430,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_d_field_info),
	trq_sel_fmap_d_field_info
},
{"TRQ_SEL_FMAP_E", 0x434,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_e_field_info),
	trq_sel_fmap_e_field_info
},
{"TRQ_SEL_FMAP_F", 0x438,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_f_field_info),
	trq_sel_fmap_f_field_info
},
{"TRQ_SEL_FMAP_10", 0x43c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_10_field_info),
	trq_sel_fmap_10_field_info
},
{"TRQ_SEL_FMAP_11", 0x440,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_11_field_info),
	trq_sel_fmap_11_field_info
},
{"TRQ_SEL_FMAP_12", 0x444,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_12_field_info),
	trq_sel_fmap_12_field_info
},
{"TRQ_SEL_FMAP_13", 0x448,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_13_field_info),
	trq_sel_fmap_13_field_info
},
{"TRQ_SEL_FMAP_14", 0x44c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_14_field_info),
	trq_sel_fmap_14_field_info
},
{"TRQ_SEL_FMAP_15", 0x450,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_15_field_info),
	trq_sel_fmap_15_field_info
},
{"TRQ_SEL_FMAP_16", 0x454,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_16_field_info),
	trq_sel_fmap_16_field_info
},
{"TRQ_SEL_FMAP_17", 0x458,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_17_field_info),
	trq_sel_fmap_17_field_info
},
{"TRQ_SEL_FMAP_18", 0x45c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_18_field_info),
	trq_sel_fmap_18_field_info
},
{"TRQ_SEL_FMAP_19", 0x460,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_19_field_info),
	trq_sel_fmap_19_field_info
},
{"TRQ_SEL_FMAP_1A", 0x464,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_1a_field_info),
	trq_sel_fmap_1a_field_info
},
{"TRQ_SEL_FMAP_1B", 0x468,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_1b_field_info),
	trq_sel_fmap_1b_field_info
},
{"TRQ_SEL_FMAP_1C", 0x46c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_1c_field_info),
	trq_sel_fmap_1c_field_info
},
{"TRQ_SEL_FMAP_1D", 0x470,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_1d_field_info),
	trq_sel_fmap_1d_field_info
},
{"TRQ_SEL_FMAP_1E", 0x474,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_1e_field_info),
	trq_sel_fmap_1e_field_info
},
{"TRQ_SEL_FMAP_1F", 0x478,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_1f_field_info),
	trq_sel_fmap_1f_field_info
},
{"TRQ_SEL_FMAP_20", 0x47c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_20_field_info),
	trq_sel_fmap_20_field_info
},
{"TRQ_SEL_FMAP_21", 0x480,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_21_field_info),
	trq_sel_fmap_21_field_info
},
{"TRQ_SEL_FMAP_22", 0x484,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_22_field_info),
	trq_sel_fmap_22_field_info
},
{"TRQ_SEL_FMAP_23", 0x488,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_23_field_info),
	trq_sel_fmap_23_field_info
},
{"TRQ_SEL_FMAP_24", 0x48c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_24_field_info),
	trq_sel_fmap_24_field_info
},
{"TRQ_SEL_FMAP_25", 0x490,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_25_field_info),
	trq_sel_fmap_25_field_info
},
{"TRQ_SEL_FMAP_26", 0x494,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_26_field_info),
	trq_sel_fmap_26_field_info
},
{"TRQ_SEL_FMAP_27", 0x498,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_27_field_info),
	trq_sel_fmap_27_field_info
},
{"TRQ_SEL_FMAP_28", 0x49c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_28_field_info),
	trq_sel_fmap_28_field_info
},
{"TRQ_SEL_FMAP_29", 0x4a0,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_29_field_info),
	trq_sel_fmap_29_field_info
},
{"TRQ_SEL_FMAP_2A", 0x4a4,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_2a_field_info),
	trq_sel_fmap_2a_field_info
},
{"TRQ_SEL_FMAP_2B", 0x4a8,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_2b_field_info),
	trq_sel_fmap_2b_field_info
},
{"TRQ_SEL_FMAP_2C", 0x4ac,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_2c_field_info),
	trq_sel_fmap_2c_field_info
},
{"TRQ_SEL_FMAP_2D", 0x4b0,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_2d_field_info),
	trq_sel_fmap_2d_field_info
},
{"TRQ_SEL_FMAP_2E", 0x4b4,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_2e_field_info),
	trq_sel_fmap_2e_field_info
},
{"TRQ_SEL_FMAP_2F", 0x4b8,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_2f_field_info),
	trq_sel_fmap_2f_field_info
},
{"TRQ_SEL_FMAP_30", 0x4bc,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_30_field_info),
	trq_sel_fmap_30_field_info
},
{"TRQ_SEL_FMAP_31", 0x4d0,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_31_field_info),
	trq_sel_fmap_31_field_info
},
{"TRQ_SEL_FMAP_32", 0x4d4,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_32_field_info),
	trq_sel_fmap_32_field_info
},
{"TRQ_SEL_FMAP_33", 0x4d8,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_33_field_info),
	trq_sel_fmap_33_field_info
},
{"TRQ_SEL_FMAP_34", 0x4dc,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_34_field_info),
	trq_sel_fmap_34_field_info
},
{"TRQ_SEL_FMAP_35", 0x4e0,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_35_field_info),
	trq_sel_fmap_35_field_info
},
{"TRQ_SEL_FMAP_36", 0x4e4,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_36_field_info),
	trq_sel_fmap_36_field_info
},
{"TRQ_SEL_FMAP_37", 0x4e8,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_37_field_info),
	trq_sel_fmap_37_field_info
},
{"TRQ_SEL_FMAP_38", 0x4ec,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_38_field_info),
	trq_sel_fmap_38_field_info
},
{"TRQ_SEL_FMAP_39", 0x4f0,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_39_field_info),
	trq_sel_fmap_39_field_info
},
{"TRQ_SEL_FMAP_3A", 0x4f4,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_3a_field_info),
	trq_sel_fmap_3a_field_info
},
{"TRQ_SEL_FMAP_3B", 0x4f8,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_3b_field_info),
	trq_sel_fmap_3b_field_info
},
{"TRQ_SEL_FMAP_3C", 0x4fc,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_3c_field_info),
	trq_sel_fmap_3c_field_info
},
{"TRQ_SEL_FMAP_3D", 0x500,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_3d_field_info),
	trq_sel_fmap_3d_field_info
},
{"TRQ_SEL_FMAP_3E", 0x504,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_3e_field_info),
	trq_sel_fmap_3e_field_info
},
{"TRQ_SEL_FMAP_3F", 0x508,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_3f_field_info),
	trq_sel_fmap_3f_field_info
},
{"TRQ_SEL_FMAP_40", 0x50c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_40_field_info),
	trq_sel_fmap_40_field_info
},
{"TRQ_SEL_FMAP_41", 0x510,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_41_field_info),
	trq_sel_fmap_41_field_info
},
{"TRQ_SEL_FMAP_42", 0x514,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_42_field_info),
	trq_sel_fmap_42_field_info
},
{"TRQ_SEL_FMAP_43", 0x518,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_43_field_info),
	trq_sel_fmap_43_field_info
},
{"TRQ_SEL_FMAP_44", 0x51c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_44_field_info),
	trq_sel_fmap_44_field_info
},
{"TRQ_SEL_FMAP_45", 0x520,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_45_field_info),
	trq_sel_fmap_45_field_info
},
{"TRQ_SEL_FMAP_46", 0x524,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_46_field_info),
	trq_sel_fmap_46_field_info
},
{"TRQ_SEL_FMAP_47", 0x528,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_47_field_info),
	trq_sel_fmap_47_field_info
},
{"TRQ_SEL_FMAP_48", 0x52c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_48_field_info),
	trq_sel_fmap_48_field_info
},
{"TRQ_SEL_FMAP_49", 0x530,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_49_field_info),
	trq_sel_fmap_49_field_info
},
{"TRQ_SEL_FMAP_4A", 0x534,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_4a_field_info),
	trq_sel_fmap_4a_field_info
},
{"TRQ_SEL_FMAP_4B", 0x538,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_4b_field_info),
	trq_sel_fmap_4b_field_info
},
{"TRQ_SEL_FMAP_4C", 0x53c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_4c_field_info),
	trq_sel_fmap_4c_field_info
},
{"TRQ_SEL_FMAP_4D", 0x540,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_4d_field_info),
	trq_sel_fmap_4d_field_info
},
{"TRQ_SEL_FMAP_4E", 0x544,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_4e_field_info),
	trq_sel_fmap_4e_field_info
},
{"TRQ_SEL_FMAP_4F", 0x548,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_4f_field_info),
	trq_sel_fmap_4f_field_info
},
{"TRQ_SEL_FMAP_50", 0x54c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_50_field_info),
	trq_sel_fmap_50_field_info
},
{"TRQ_SEL_FMAP_51", 0x550,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_51_field_info),
	trq_sel_fmap_51_field_info
},
{"TRQ_SEL_FMAP_52", 0x554,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_52_field_info),
	trq_sel_fmap_52_field_info
},
{"TRQ_SEL_FMAP_53", 0x558,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_53_field_info),
	trq_sel_fmap_53_field_info
},
{"TRQ_SEL_FMAP_54", 0x55c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_54_field_info),
	trq_sel_fmap_54_field_info
},
{"TRQ_SEL_FMAP_55", 0x560,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_55_field_info),
	trq_sel_fmap_55_field_info
},
{"TRQ_SEL_FMAP_56", 0x564,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_56_field_info),
	trq_sel_fmap_56_field_info
},
{"TRQ_SEL_FMAP_57", 0x568,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_57_field_info),
	trq_sel_fmap_57_field_info
},
{"TRQ_SEL_FMAP_58", 0x56c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_58_field_info),
	trq_sel_fmap_58_field_info
},
{"TRQ_SEL_FMAP_59", 0x570,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_59_field_info),
	trq_sel_fmap_59_field_info
},
{"TRQ_SEL_FMAP_5A", 0x574,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_5a_field_info),
	trq_sel_fmap_5a_field_info
},
{"TRQ_SEL_FMAP_5B", 0x578,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_5b_field_info),
	trq_sel_fmap_5b_field_info
},
{"TRQ_SEL_FMAP_5C", 0x57c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_5c_field_info),
	trq_sel_fmap_5c_field_info
},
{"TRQ_SEL_FMAP_5D", 0x580,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_5d_field_info),
	trq_sel_fmap_5d_field_info
},
{"TRQ_SEL_FMAP_5E", 0x584,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_5e_field_info),
	trq_sel_fmap_5e_field_info
},
{"TRQ_SEL_FMAP_5F", 0x588,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_5f_field_info),
	trq_sel_fmap_5f_field_info
},
{"TRQ_SEL_FMAP_60", 0x58c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_60_field_info),
	trq_sel_fmap_60_field_info
},
{"TRQ_SEL_FMAP_61", 0x590,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_61_field_info),
	trq_sel_fmap_61_field_info
},
{"TRQ_SEL_FMAP_62", 0x594,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_62_field_info),
	trq_sel_fmap_62_field_info
},
{"TRQ_SEL_FMAP_63", 0x598,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_63_field_info),
	trq_sel_fmap_63_field_info
},
{"TRQ_SEL_FMAP_64", 0x59c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_64_field_info),
	trq_sel_fmap_64_field_info
},
{"TRQ_SEL_FMAP_65", 0x5a0,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_65_field_info),
	trq_sel_fmap_65_field_info
},
{"TRQ_SEL_FMAP_66", 0x5a4,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_66_field_info),
	trq_sel_fmap_66_field_info
},
{"TRQ_SEL_FMAP_67", 0x5a8,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_67_field_info),
	trq_sel_fmap_67_field_info
},
{"TRQ_SEL_FMAP_68", 0x5ac,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_68_field_info),
	trq_sel_fmap_68_field_info
},
{"TRQ_SEL_FMAP_69", 0x5b0,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_69_field_info),
	trq_sel_fmap_69_field_info
},
{"TRQ_SEL_FMAP_6A", 0x5b4,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_6a_field_info),
	trq_sel_fmap_6a_field_info
},
{"TRQ_SEL_FMAP_6B", 0x5b8,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_6b_field_info),
	trq_sel_fmap_6b_field_info
},
{"TRQ_SEL_FMAP_6C", 0x5bc,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_6c_field_info),
	trq_sel_fmap_6c_field_info
},
{"TRQ_SEL_FMAP_6D", 0x5c0,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_6d_field_info),
	trq_sel_fmap_6d_field_info
},
{"TRQ_SEL_FMAP_6E", 0x5c4,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_6e_field_info),
	trq_sel_fmap_6e_field_info
},
{"TRQ_SEL_FMAP_6F", 0x5c8,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_6f_field_info),
	trq_sel_fmap_6f_field_info
},
{"TRQ_SEL_FMAP_70", 0x5cc,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_70_field_info),
	trq_sel_fmap_70_field_info
},
{"TRQ_SEL_FMAP_71", 0x5d0,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_71_field_info),
	trq_sel_fmap_71_field_info
},
{"TRQ_SEL_FMAP_72", 0x5d4,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_72_field_info),
	trq_sel_fmap_72_field_info
},
{"TRQ_SEL_FMAP_73", 0x5d8,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_73_field_info),
	trq_sel_fmap_73_field_info
},
{"TRQ_SEL_FMAP_74", 0x5dc,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_74_field_info),
	trq_sel_fmap_74_field_info
},
{"TRQ_SEL_FMAP_75", 0x5e0,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_75_field_info),
	trq_sel_fmap_75_field_info
},
{"TRQ_SEL_FMAP_76", 0x5e4,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_76_field_info),
	trq_sel_fmap_76_field_info
},
{"TRQ_SEL_FMAP_77", 0x5e8,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_77_field_info),
	trq_sel_fmap_77_field_info
},
{"TRQ_SEL_FMAP_78", 0x5ec,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_78_field_info),
	trq_sel_fmap_78_field_info
},
{"TRQ_SEL_FMAP_79", 0x5f0,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_79_field_info),
	trq_sel_fmap_79_field_info
},
{"TRQ_SEL_FMAP_7A", 0x5f4,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_7a_field_info),
	trq_sel_fmap_7a_field_info
},
{"TRQ_SEL_FMAP_7B", 0x5f8,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_7b_field_info),
	trq_sel_fmap_7b_field_info
},
{"TRQ_SEL_FMAP_7C", 0x5fc,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_7c_field_info),
	trq_sel_fmap_7c_field_info
},
{"TRQ_SEL_FMAP_7D", 0x600,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_7d_field_info),
	trq_sel_fmap_7d_field_info
},
{"TRQ_SEL_FMAP_7E", 0x604,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_7e_field_info),
	trq_sel_fmap_7e_field_info
},
{"TRQ_SEL_FMAP_7F", 0x608,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_7f_field_info),
	trq_sel_fmap_7f_field_info
},
{"TRQ_SEL_FMAP_80", 0x60c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_80_field_info),
	trq_sel_fmap_80_field_info
},
{"TRQ_SEL_FMAP_81", 0x610,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_81_field_info),
	trq_sel_fmap_81_field_info
},
{"TRQ_SEL_FMAP_82", 0x614,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_82_field_info),
	trq_sel_fmap_82_field_info
},
{"TRQ_SEL_FMAP_83", 0x618,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_83_field_info),
	trq_sel_fmap_83_field_info
},
{"TRQ_SEL_FMAP_84", 0x61c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_84_field_info),
	trq_sel_fmap_84_field_info
},
{"TRQ_SEL_FMAP_85", 0x620,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_85_field_info),
	trq_sel_fmap_85_field_info
},
{"TRQ_SEL_FMAP_86", 0x624,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_86_field_info),
	trq_sel_fmap_86_field_info
},
{"TRQ_SEL_FMAP_87", 0x628,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_87_field_info),
	trq_sel_fmap_87_field_info
},
{"TRQ_SEL_FMAP_88", 0x62c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_88_field_info),
	trq_sel_fmap_88_field_info
},
{"TRQ_SEL_FMAP_89", 0x630,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_89_field_info),
	trq_sel_fmap_89_field_info
},
{"TRQ_SEL_FMAP_8A", 0x634,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_8a_field_info),
	trq_sel_fmap_8a_field_info
},
{"TRQ_SEL_FMAP_8B", 0x638,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_8b_field_info),
	trq_sel_fmap_8b_field_info
},
{"TRQ_SEL_FMAP_8C", 0x63c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_8c_field_info),
	trq_sel_fmap_8c_field_info
},
{"TRQ_SEL_FMAP_8D", 0x640,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_8d_field_info),
	trq_sel_fmap_8d_field_info
},
{"TRQ_SEL_FMAP_8E", 0x644,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_8e_field_info),
	trq_sel_fmap_8e_field_info
},
{"TRQ_SEL_FMAP_8F", 0x648,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_8f_field_info),
	trq_sel_fmap_8f_field_info
},
{"TRQ_SEL_FMAP_90", 0x64c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_90_field_info),
	trq_sel_fmap_90_field_info
},
{"TRQ_SEL_FMAP_91", 0x650,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_91_field_info),
	trq_sel_fmap_91_field_info
},
{"TRQ_SEL_FMAP_92", 0x654,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_92_field_info),
	trq_sel_fmap_92_field_info
},
{"TRQ_SEL_FMAP_93", 0x658,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_93_field_info),
	trq_sel_fmap_93_field_info
},
{"TRQ_SEL_FMAP_94", 0x65c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_94_field_info),
	trq_sel_fmap_94_field_info
},
{"TRQ_SEL_FMAP_95", 0x660,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_95_field_info),
	trq_sel_fmap_95_field_info
},
{"TRQ_SEL_FMAP_96", 0x664,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_96_field_info),
	trq_sel_fmap_96_field_info
},
{"TRQ_SEL_FMAP_97", 0x668,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_97_field_info),
	trq_sel_fmap_97_field_info
},
{"TRQ_SEL_FMAP_98", 0x66c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_98_field_info),
	trq_sel_fmap_98_field_info
},
{"TRQ_SEL_FMAP_99", 0x670,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_99_field_info),
	trq_sel_fmap_99_field_info
},
{"TRQ_SEL_FMAP_9A", 0x674,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_9a_field_info),
	trq_sel_fmap_9a_field_info
},
{"TRQ_SEL_FMAP_9B", 0x678,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_9b_field_info),
	trq_sel_fmap_9b_field_info
},
{"TRQ_SEL_FMAP_9C", 0x67c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_9c_field_info),
	trq_sel_fmap_9c_field_info
},
{"TRQ_SEL_FMAP_9D", 0x680,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_9d_field_info),
	trq_sel_fmap_9d_field_info
},
{"TRQ_SEL_FMAP_9E", 0x684,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_9e_field_info),
	trq_sel_fmap_9e_field_info
},
{"TRQ_SEL_FMAP_9F", 0x688,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_9f_field_info),
	trq_sel_fmap_9f_field_info
},
{"TRQ_SEL_FMAP_A0", 0x68c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_a0_field_info),
	trq_sel_fmap_a0_field_info
},
{"TRQ_SEL_FMAP_A1", 0x690,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_a1_field_info),
	trq_sel_fmap_a1_field_info
},
{"TRQ_SEL_FMAP_A2", 0x694,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_a2_field_info),
	trq_sel_fmap_a2_field_info
},
{"TRQ_SEL_FMAP_A3", 0x698,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_a3_field_info),
	trq_sel_fmap_a3_field_info
},
{"TRQ_SEL_FMAP_A4", 0x69c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_a4_field_info),
	trq_sel_fmap_a4_field_info
},
{"TRQ_SEL_FMAP_A5", 0x6a0,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_a5_field_info),
	trq_sel_fmap_a5_field_info
},
{"TRQ_SEL_FMAP_A6", 0x6a4,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_a6_field_info),
	trq_sel_fmap_a6_field_info
},
{"TRQ_SEL_FMAP_A7", 0x6a8,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_a7_field_info),
	trq_sel_fmap_a7_field_info
},
{"TRQ_SEL_FMAP_A8", 0x6ac,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_a8_field_info),
	trq_sel_fmap_a8_field_info
},
{"TRQ_SEL_FMAP_A9", 0x6b0,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_a9_field_info),
	trq_sel_fmap_a9_field_info
},
{"TRQ_SEL_FMAP_AA", 0x6b4,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_aa_field_info),
	trq_sel_fmap_aa_field_info
},
{"TRQ_SEL_FMAP_AB", 0x6b8,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_ab_field_info),
	trq_sel_fmap_ab_field_info
},
{"TRQ_SEL_FMAP_AC", 0x6bc,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_ac_field_info),
	trq_sel_fmap_ac_field_info
},
{"TRQ_SEL_FMAP_AD", 0x6d0,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_ad_field_info),
	trq_sel_fmap_ad_field_info
},
{"TRQ_SEL_FMAP_AE", 0x6d4,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_ae_field_info),
	trq_sel_fmap_ae_field_info
},
{"TRQ_SEL_FMAP_AF", 0x6d8,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_af_field_info),
	trq_sel_fmap_af_field_info
},
{"TRQ_SEL_FMAP_B0", 0x6dc,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_b0_field_info),
	trq_sel_fmap_b0_field_info
},
{"TRQ_SEL_FMAP_B1", 0x6e0,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_b1_field_info),
	trq_sel_fmap_b1_field_info
},
{"TRQ_SEL_FMAP_B2", 0x6e4,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_b2_field_info),
	trq_sel_fmap_b2_field_info
},
{"TRQ_SEL_FMAP_B3", 0x6e8,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_b3_field_info),
	trq_sel_fmap_b3_field_info
},
{"TRQ_SEL_FMAP_B4", 0x6ec,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_b4_field_info),
	trq_sel_fmap_b4_field_info
},
{"TRQ_SEL_FMAP_B5", 0x6f0,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_b5_field_info),
	trq_sel_fmap_b5_field_info
},
{"TRQ_SEL_FMAP_B6", 0x6f4,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_b6_field_info),
	trq_sel_fmap_b6_field_info
},
{"TRQ_SEL_FMAP_B7", 0x6f8,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_b7_field_info),
	trq_sel_fmap_b7_field_info
},
{"TRQ_SEL_FMAP_B8", 0x6fc,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_b8_field_info),
	trq_sel_fmap_b8_field_info
},
{"TRQ_SEL_FMAP_B9", 0x700,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_b9_field_info),
	trq_sel_fmap_b9_field_info
},
{"TRQ_SEL_FMAP_BA", 0x704,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_ba_field_info),
	trq_sel_fmap_ba_field_info
},
{"TRQ_SEL_FMAP_BB", 0x708,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_bb_field_info),
	trq_sel_fmap_bb_field_info
},
{"TRQ_SEL_FMAP_BC", 0x70c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_bc_field_info),
	trq_sel_fmap_bc_field_info
},
{"TRQ_SEL_FMAP_BD", 0x710,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_bd_field_info),
	trq_sel_fmap_bd_field_info
},
{"TRQ_SEL_FMAP_BE", 0x714,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_be_field_info),
	trq_sel_fmap_be_field_info
},
{"TRQ_SEL_FMAP_BF", 0x718,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_bf_field_info),
	trq_sel_fmap_bf_field_info
},
{"TRQ_SEL_FMAP_C0", 0x71c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_c0_field_info),
	trq_sel_fmap_c0_field_info
},
{"TRQ_SEL_FMAP_C1", 0x720,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_c1_field_info),
	trq_sel_fmap_c1_field_info
},
{"TRQ_SEL_FMAP_C2", 0x734,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_c2_field_info),
	trq_sel_fmap_c2_field_info
},
{"TRQ_SEL_FMAP_C3", 0x748,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_c3_field_info),
	trq_sel_fmap_c3_field_info
},
{"TRQ_SEL_FMAP_C4", 0x74c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_c4_field_info),
	trq_sel_fmap_c4_field_info
},
{"TRQ_SEL_FMAP_C5", 0x750,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_c5_field_info),
	trq_sel_fmap_c5_field_info
},
{"TRQ_SEL_FMAP_C6", 0x754,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_c6_field_info),
	trq_sel_fmap_c6_field_info
},
{"TRQ_SEL_FMAP_C7", 0x758,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_c7_field_info),
	trq_sel_fmap_c7_field_info
},
{"TRQ_SEL_FMAP_C8", 0x75c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_c8_field_info),
	trq_sel_fmap_c8_field_info
},
{"TRQ_SEL_FMAP_C9", 0x760,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_c9_field_info),
	trq_sel_fmap_c9_field_info
},
{"TRQ_SEL_FMAP_CA", 0x764,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_ca_field_info),
	trq_sel_fmap_ca_field_info
},
{"TRQ_SEL_FMAP_CB", 0x768,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_cb_field_info),
	trq_sel_fmap_cb_field_info
},
{"TRQ_SEL_FMAP_CC", 0x76c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_cc_field_info),
	trq_sel_fmap_cc_field_info
},
{"TRQ_SEL_FMAP_CD", 0x770,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_cd_field_info),
	trq_sel_fmap_cd_field_info
},
{"TRQ_SEL_FMAP_CE", 0x774,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_ce_field_info),
	trq_sel_fmap_ce_field_info
},
{"TRQ_SEL_FMAP_CF", 0x778,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_cf_field_info),
	trq_sel_fmap_cf_field_info
},
{"TRQ_SEL_FMAP_D0", 0x77c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_d0_field_info),
	trq_sel_fmap_d0_field_info
},
{"TRQ_SEL_FMAP_D1", 0x780,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_d1_field_info),
	trq_sel_fmap_d1_field_info
},
{"TRQ_SEL_FMAP_D2", 0x784,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_d2_field_info),
	trq_sel_fmap_d2_field_info
},
{"TRQ_SEL_FMAP_D3", 0x788,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_d3_field_info),
	trq_sel_fmap_d3_field_info
},
{"TRQ_SEL_FMAP_D4", 0x78c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_d4_field_info),
	trq_sel_fmap_d4_field_info
},
{"TRQ_SEL_FMAP_D5", 0x790,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_d5_field_info),
	trq_sel_fmap_d5_field_info
},
{"TRQ_SEL_FMAP_D6", 0x794,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_d6_field_info),
	trq_sel_fmap_d6_field_info
},
{"TRQ_SEL_FMAP_D7", 0x798,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_d7_field_info),
	trq_sel_fmap_d7_field_info
},
{"TRQ_SEL_FMAP_D8", 0x79c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_d8_field_info),
	trq_sel_fmap_d8_field_info
},
{"TRQ_SEL_FMAP_D9", 0x7a0,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_d9_field_info),
	trq_sel_fmap_d9_field_info
},
{"TRQ_SEL_FMAP_DA", 0x7a4,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_da_field_info),
	trq_sel_fmap_da_field_info
},
{"TRQ_SEL_FMAP_DB", 0x7a8,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_db_field_info),
	trq_sel_fmap_db_field_info
},
{"TRQ_SEL_FMAP_DC", 0x7ac,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_dc_field_info),
	trq_sel_fmap_dc_field_info
},
{"TRQ_SEL_FMAP_DD", 0x7b0,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_dd_field_info),
	trq_sel_fmap_dd_field_info
},
{"TRQ_SEL_FMAP_DE", 0x7b4,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_de_field_info),
	trq_sel_fmap_de_field_info
},
{"TRQ_SEL_FMAP_DF", 0x7b8,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_df_field_info),
	trq_sel_fmap_df_field_info
},
{"TRQ_SEL_FMAP_E0", 0x7bc,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_e0_field_info),
	trq_sel_fmap_e0_field_info
},
{"TRQ_SEL_FMAP_E1", 0x7c0,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_e1_field_info),
	trq_sel_fmap_e1_field_info
},
{"TRQ_SEL_FMAP_E2", 0x7c4,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_e2_field_info),
	trq_sel_fmap_e2_field_info
},
{"TRQ_SEL_FMAP_E3", 0x7c8,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_e3_field_info),
	trq_sel_fmap_e3_field_info
},
{"TRQ_SEL_FMAP_E4", 0x7cc,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_e4_field_info),
	trq_sel_fmap_e4_field_info
},
{"TRQ_SEL_FMAP_E5", 0x7d0,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_e5_field_info),
	trq_sel_fmap_e5_field_info
},
{"TRQ_SEL_FMAP_E6", 0x7d4,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_e6_field_info),
	trq_sel_fmap_e6_field_info
},
{"TRQ_SEL_FMAP_E7", 0x7d8,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_e7_field_info),
	trq_sel_fmap_e7_field_info
},
{"TRQ_SEL_FMAP_E8", 0x7dc,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_e8_field_info),
	trq_sel_fmap_e8_field_info
},
{"TRQ_SEL_FMAP_E9", 0x7e0,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_e9_field_info),
	trq_sel_fmap_e9_field_info
},
{"TRQ_SEL_FMAP_EA", 0x7e4,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_ea_field_info),
	trq_sel_fmap_ea_field_info
},
{"TRQ_SEL_FMAP_EB", 0x7e8,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_eb_field_info),
	trq_sel_fmap_eb_field_info
},
{"TRQ_SEL_FMAP_EC", 0x7ec,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_ec_field_info),
	trq_sel_fmap_ec_field_info
},
{"TRQ_SEL_FMAP_ED", 0x7f0,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_ed_field_info),
	trq_sel_fmap_ed_field_info
},
{"TRQ_SEL_FMAP_EE", 0x7f4,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_ee_field_info),
	trq_sel_fmap_ee_field_info
},
{"TRQ_SEL_FMAP_EF", 0x7f8,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_ef_field_info),
	trq_sel_fmap_ef_field_info
},
{"TRQ_SEL_FMAP_F0", 0x7fc,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(trq_sel_fmap_f0_field_info),
	trq_sel_fmap_f0_field_info
},
{"IND_CTXT_DATA_3", 0x804,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(ind_ctxt_data_3_field_info),
	ind_ctxt_data_3_field_info
},
{"IND_CTXT_DATA_2", 0x808,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(ind_ctxt_data_2_field_info),
	ind_ctxt_data_2_field_info
},
{"IND_CTXT_DATA_1", 0x80c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(ind_ctxt_data_1_field_info),
	ind_ctxt_data_1_field_info
},
{"IND_CTXT_DATA_0", 0x810,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(ind_ctxt_data_0_field_info),
	ind_ctxt_data_0_field_info
},
{"IND_CTXT3", 0x814,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(ind_ctxt3_field_info),
	ind_ctxt3_field_info
},
{"IND_CTXT2", 0x818,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(ind_ctxt2_field_info),
	ind_ctxt2_field_info
},
{"IND_CTXT1", 0x81c,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(ind_ctxt1_field_info),
	ind_ctxt1_field_info
},
{"IND_CTXT0", 0x820,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(ind_ctxt0_field_info),
	ind_ctxt0_field_info
},
{"IND_CTXT_CMD", 0x824,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(ind_ctxt_cmd_field_info),
	ind_ctxt_cmd_field_info
},
{"C2H_TIMER_CNT_1", 0xa00,
	1, 0, 0, 0,
	0, QDMA_COMPLETION_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_timer_cnt_1_field_info),
	c2h_timer_cnt_1_field_info
},
{"C2H_TIMER_CNT_2", 0xa04,
	1, 0, 0, 0,
	0, QDMA_COMPLETION_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_timer_cnt_2_field_info),
	c2h_timer_cnt_2_field_info
},
{"C2H_TIMER_CNT_3", 0xa08,
	1, 0, 0, 0,
	0, QDMA_COMPLETION_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_timer_cnt_3_field_info),
	c2h_timer_cnt_3_field_info
},
{"C2H_TIMER_CNT_4", 0xa0c,
	1, 0, 0, 0,
	0, QDMA_COMPLETION_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_timer_cnt_4_field_info),
	c2h_timer_cnt_4_field_info
},
{"C2H_TIMER_CNT_5", 0xa10,
	1, 0, 0, 0,
	0, QDMA_COMPLETION_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_timer_cnt_5_field_info),
	c2h_timer_cnt_5_field_info
},
{"C2H_TIMER_CNT_6", 0xa14,
	1, 0, 0, 0,
	0, QDMA_COMPLETION_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_timer_cnt_6_field_info),
	c2h_timer_cnt_6_field_info
},
{"C2H_TIMER_CNT_7", 0xa18,
	1, 0, 0, 0,
	0, QDMA_COMPLETION_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_timer_cnt_7_field_info),
	c2h_timer_cnt_7_field_info
},
{"C2H_TIMER_CNT_8", 0xa1c,
	1, 0, 0, 0,
	0, QDMA_COMPLETION_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_timer_cnt_8_field_info),
	c2h_timer_cnt_8_field_info
},
{"C2H_TIMER_CNT_9", 0xa20,
	1, 0, 0, 0,
	0, QDMA_COMPLETION_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_timer_cnt_9_field_info),
	c2h_timer_cnt_9_field_info
},
{"C2H_TIMER_CNT_A", 0xa24,
	1, 0, 0, 0,
	0, QDMA_COMPLETION_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_timer_cnt_a_field_info),
	c2h_timer_cnt_a_field_info
},
{"C2H_TIMER_CNT_B", 0xa28,
	1, 0, 0, 0,
	0, QDMA_COMPLETION_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_timer_cnt_b_field_info),
	c2h_timer_cnt_b_field_info
},
{"C2H_TIMER_CNT_C", 0xa2c,
	1, 0, 0, 0,
	0, QDMA_COMPLETION_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_timer_cnt_c_field_info),
	c2h_timer_cnt_c_field_info
},
{"C2H_TIMER_CNT_D", 0xa30,
	1, 0, 0, 0,
	0, QDMA_COMPLETION_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_timer_cnt_d_field_info),
	c2h_timer_cnt_d_field_info
},
{"C2H_TIMER_CNT_E", 0xa34,
	1, 0, 0, 0,
	0, QDMA_COMPLETION_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_timer_cnt_e_field_info),
	c2h_timer_cnt_e_field_info
},
{"C2H_TIMER_CNT_F", 0xa38,
	1, 0, 0, 0,
	0, QDMA_COMPLETION_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_timer_cnt_f_field_info),
	c2h_timer_cnt_f_field_info
},
{"C2H_TIMER_CNT_10", 0xa3c,
	1, 0, 0, 0,
	0, QDMA_COMPLETION_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_timer_cnt_10_field_info),
	c2h_timer_cnt_10_field_info
},
{"C2H_CNT_TH_1", 0xa40,
	1, 0, 0, 0,
	0, QDMA_COMPLETION_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_cnt_th_1_field_info),
	c2h_cnt_th_1_field_info
},
{"C2H_CNT_TH_2", 0xa44,
	1, 0, 0, 0,
	0, QDMA_COMPLETION_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_cnt_th_2_field_info),
	c2h_cnt_th_2_field_info
},
{"C2H_CNT_TH_3", 0xa48,
	1, 0, 0, 0,
	0, QDMA_COMPLETION_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_cnt_th_3_field_info),
	c2h_cnt_th_3_field_info
},
{"C2H_CNT_TH_4", 0xa4c,
	1, 0, 0, 0,
	0, QDMA_COMPLETION_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_cnt_th_4_field_info),
	c2h_cnt_th_4_field_info
},
{"C2H_CNT_TH_5", 0xa50,
	1, 0, 0, 0,
	0, QDMA_COMPLETION_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_cnt_th_5_field_info),
	c2h_cnt_th_5_field_info
},
{"C2H_CNT_TH_6", 0xa54,
	1, 0, 0, 0,
	0, QDMA_COMPLETION_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_cnt_th_6_field_info),
	c2h_cnt_th_6_field_info
},
{"C2H_CNT_TH_7", 0xa58,
	1, 0, 0, 0,
	0, QDMA_COMPLETION_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_cnt_th_7_field_info),
	c2h_cnt_th_7_field_info
},
{"C2H_CNT_TH_8", 0xa5c,
	1, 0, 0, 0,
	0, QDMA_COMPLETION_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_cnt_th_8_field_info),
	c2h_cnt_th_8_field_info
},
{"C2H_CNT_TH_9", 0xa60,
	1, 0, 0, 0,
	0, QDMA_COMPLETION_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_cnt_th_9_field_info),
	c2h_cnt_th_9_field_info
},
{"C2H_CNT_TH_A", 0xa64,
	1, 0, 0, 0,
	0, QDMA_COMPLETION_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_cnt_th_a_field_info),
	c2h_cnt_th_a_field_info
},
{"C2H_CNT_TH_B", 0xa68,
	1, 0, 0, 0,
	0, QDMA_COMPLETION_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_cnt_th_b_field_info),
	c2h_cnt_th_b_field_info
},
{"C2H_CNT_TH_C", 0xa6c,
	1, 0, 0, 0,
	0, QDMA_COMPLETION_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_cnt_th_c_field_info),
	c2h_cnt_th_c_field_info
},
{"C2H_CNT_TH_D", 0xa70,
	1, 0, 0, 0,
	0, QDMA_COMPLETION_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_cnt_th_d_field_info),
	c2h_cnt_th_d_field_info
},
{"C2H_CNT_TH_E", 0xa74,
	1, 0, 0, 0,
	0, QDMA_COMPLETION_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_cnt_th_e_field_info),
	c2h_cnt_th_e_field_info
},
{"C2H_CNT_TH_F", 0xa78,
	1, 0, 0, 0,
	0, QDMA_COMPLETION_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_cnt_th_f_field_info),
	c2h_cnt_th_f_field_info
},
{"C2H_CNT_TH_10", 0xa7c,
	1, 0, 0, 0,
	0, QDMA_COMPLETION_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_cnt_th_10_field_info),
	c2h_cnt_th_10_field_info
},
{"C2H_QID2VEC_MAP_QID", 0xa80,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_qid2vec_map_qid_field_info),
	c2h_qid2vec_map_qid_field_info
},
{"C2H_QID2VEC_MAP", 0xa84,
	1, 0, 0, 0,
	0, QDMA_MM_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_qid2vec_map_field_info),
	c2h_qid2vec_map_field_info
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
{"C2H_BUF_SZ_0", 0xab0,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_buf_sz_0_field_info),
	c2h_buf_sz_0_field_info
},
{"C2H_BUF_SZ_1", 0xab4,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_buf_sz_1_field_info),
	c2h_buf_sz_1_field_info
},
{"C2H_BUF_SZ_2", 0xab8,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_buf_sz_2_field_info),
	c2h_buf_sz_2_field_info
},
{"C2H_BUF_SZ_3", 0xabc,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_buf_sz_3_field_info),
	c2h_buf_sz_3_field_info
},
{"C2H_BUF_SZ_4", 0xac0,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_buf_sz_4_field_info),
	c2h_buf_sz_4_field_info
},
{"C2H_BUF_SZ_5", 0xac4,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_buf_sz_5_field_info),
	c2h_buf_sz_5_field_info
},
{"C2H_BUF_SZ_7", 0xac8,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_buf_sz_7_field_info),
	c2h_buf_sz_7_field_info
},
{"C2H_BUF_SZ_8", 0xacc,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_buf_sz_8_field_info),
	c2h_buf_sz_8_field_info
},
{"C2H_BUF_SZ_9", 0xad0,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_buf_sz_9_field_info),
	c2h_buf_sz_9_field_info
},
{"C2H_BUF_SZ_10", 0xad4,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_buf_sz_10_field_info),
	c2h_buf_sz_10_field_info
},
{"C2H_BUF_SZ_11", 0xad8,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_buf_sz_11_field_info),
	c2h_buf_sz_11_field_info
},
{"C2H_BUF_SZ_12", 0xae0,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_buf_sz_12_field_info),
	c2h_buf_sz_12_field_info
},
{"C2H_BUF_SZ_13", 0xae4,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_buf_sz_13_field_info),
	c2h_buf_sz_13_field_info
},
{"C2H_BUF_SZ_14", 0xae8,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_buf_sz_14_field_info),
	c2h_buf_sz_14_field_info
},
{"C2H_BUF_SZ_15", 0xaec,
	1, 0, 0, 0,
	0, QDMA_ST_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_buf_sz_15_field_info),
	c2h_buf_sz_15_field_info
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
{"C2H_CHANNEL_CTL", 0x1004,
	1, 0, 0, 0,
	0, QDMA_MM_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_channel_ctl_field_info),
	c2h_channel_ctl_field_info
},
{"C2H_CHANNEL_CTL_1", 0x1008,
	1, 0, 0, 0,
	0, QDMA_MM_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_channel_ctl_1_field_info),
	c2h_channel_ctl_1_field_info
},
{"C2H_MM_STATUS", 0x1040,
	1, 0, 0, 0,
	0, QDMA_MM_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_mm_status_field_info),
	c2h_mm_status_field_info
},
{"C2H_CHANNEL_CMPL_DESC_CNT", 0x1048,
	1, 0, 0, 0,
	0, QDMA_MM_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(c2h_channel_cmpl_desc_cnt_field_info),
	c2h_channel_cmpl_desc_cnt_field_info
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
{"H2C_CHANNEL_CTL", 0x1204,
	1, 0, 0, 0,
	0, QDMA_MM_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(h2c_channel_ctl_field_info),
	h2c_channel_ctl_field_info
},
{"H2C_CHANNEL_CTL_1", 0x1208,
	1, 0, 0, 0,
	0, QDMA_MM_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(h2c_channel_ctl_1_field_info),
	h2c_channel_ctl_1_field_info
},
{"H2C_CHANNEL_CTL_2", 0x120c,
	1, 0, 0, 0,
	0, QDMA_MM_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(h2c_channel_ctl_2_field_info),
	h2c_channel_ctl_2_field_info
},
{"H2C_MM_STATUS", 0x1240,
	1, 0, 0, 0,
	0, QDMA_MM_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(h2c_mm_status_field_info),
	h2c_mm_status_field_info
},
{"H2C_CHANNEL_CMPL_DESC_CNT", 0x1248,
	1, 0, 0, 0,
	0, QDMA_MM_MODE, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(h2c_channel_cmpl_desc_cnt_field_info),
	h2c_channel_cmpl_desc_cnt_field_info
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
{"FUNC_STATUS_REG", 0x2400,
	1, 0, 0, 0,
	0, QDMA_MAILBOX, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(func_status_reg_field_info),
	func_status_reg_field_info
},
{"FUNC_CMD_REG", 0x2404,
	1, 0, 0, 0,
	0, QDMA_MAILBOX, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(func_cmd_reg_field_info),
	func_cmd_reg_field_info
},
{"FUNC_INTERRUPT_VECTOR_REG", 0x2408,
	1, 0, 0, 0,
	0, QDMA_MAILBOX, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(func_interrupt_vector_reg_field_info),
	func_interrupt_vector_reg_field_info
},
{"TARGET_FUNC_REG", 0x240c,
	1, 0, 0, 0,
	0, QDMA_MAILBOX, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(target_func_reg_field_info),
	target_func_reg_field_info
},
{"FUNC_INTERRUPT_CTL_REG", 0x2410,
	1, 0, 0, 0,
	0, QDMA_MAILBOX, QDMA_REG_READ_PF_ONLY,
	ARRAY_SIZE(func_interrupt_ctl_reg_field_info),
	func_interrupt_ctl_reg_field_info
},

};

// MD : Function to get the number of configuration registers
uint32_t qdma_cpm4_config_num_regs_get(void)
{
    // MD : Calculate the number of elements in the qdma_cpm4_config_regs array
    uint32_t num_regs = sizeof(qdma_cpm4_config_regs) / sizeof(qdma_cpm4_config_regs[0]);
    
    // MD : Debug: Print the number of configuration registers
    printk(KERN_DEBUG "Number of configuration registers: %u\n", num_regs);
    
    return num_regs;
}

// MD : Function to get the pointer to the configuration registers array
struct xreg_info *qdma_cpm4_config_regs_get(void)
{
    // MD : Debug: Print a message indicating the function call
    printk(KERN_DEBUG "Returning pointer to qdma_cpm4_config_regs\n");
    
    return qdma_cpm4_config_regs;
}

