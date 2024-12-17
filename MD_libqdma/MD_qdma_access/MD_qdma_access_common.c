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

#include "qdma_access_common.h"
#include "qdma_platform.h"
#include "qdma_soft_reg.h"
#include "qdma_soft_access.h"
#include "qdma_cpm4_access/qdma_cpm4_access.h"
#include "eqdma_soft_access.h"
#include "eqdma_cpm5_access/eqdma_cpm5_access.h"
#include "qdma_reg_dump.h"

#ifdef ENABLE_WPP_TRACING
#include "qdma_access_common.tmh"
#endif

/* MD: qdma version info */
#define RTL_BASE_VERSION                        2
#define RTL_PATCH_VERSION                       3

/* MD:*
 * enum qdma_ip - To hold ip type
 */
enum qdma_ip {
    QDMA_OR_VERSAL_IP,
    EQDMA_IP,
    EQDMA_CPM5_IP
};

/* MD:
 * hw_monitor_reg() - Poll a register repeatedly until
 * (the register value & mask) == val or time is up
 *
 * @dev_hndl: Device handle
 * @reg: Register to poll
 * @mask: Mask to apply to the register value
 * @val: Expected value after applying the mask
 * @interval_us: Polling interval in microseconds
 * @timeout_us: Timeout in microseconds
 *
 * Return: -QDMA_BUSY_TIMEOUT_ERR if register value didn't match, 0 otherwise
 */
int hw_monitor_reg(void *dev_hndl, uint32_t reg, uint32_t mask,
        uint32_t val, uint32_t interval_us, uint32_t timeout_us)
{
    int count;
    uint32_t v;

    // MD: Set default polling interval and timeout if not provided
    if (!interval_us)
        interval_us = QDMA_REG_POLL_DFLT_INTERVAL_US;
    if (!timeout_us)
        timeout_us = QDMA_REG_POLL_DFLT_TIMEOUT_US;

    // MD: Calculate the number of polling attempts
    count = timeout_us / interval_us;

    do {
        // MD: Read the register value
        v = qdma_reg_read(dev_hndl, reg);
        // MD: Check if the masked value matches the expected value
        if ((v & mask) == val)
            return QDMA_SUCCESS;
        // MD: Wait for the specified interval before the next attempt
        qdma_udelay(interval_us);
    } while (--count);

    // MD: Final check after the loop
    v = qdma_reg_read(dev_hndl, reg);
    if ((v & mask) == val)
        return QDMA_SUCCESS;

    // MD: Log an error if the expected value was not matched
    qdma_log_error("%s: Reg read=%u Expected=%u, err:%d\n",
                   __func__, v, val,
                   -QDMA_ERR_HWACC_BUSY_TIMEOUT);
    return -QDMA_ERR_HWACC_BUSY_TIMEOUT;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_get_rtl_version() - Function to get the rtl_version in
 * string format
 *
 * @rtl_version: Vivado release ID
 *
 * Return: string - success and NULL on failure
 *****************************************************************************/
static const char *qdma_get_rtl_version(enum qdma_rtl_version rtl_version)
{
    switch (rtl_version) {
    case QDMA_RTL_PATCH:
        return "RTL Patch";
    case QDMA_RTL_BASE:
        return "RTL Base";
    default:
        // MD: Log an error for invalid RTL version
        qdma_log_error("%s: invalid rtl_version(%d), err:%d\n",
                __func__, rtl_version, -QDMA_ERR_INV_PARAM);
        return NULL;
    }
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_get_ip_type() - Function to get the ip type in string format
 *
 * @dev_hndl:  Device handle
 * @is_vf:     Whether PF or VF
 * @ip_type:   IP Type
 *
 * Return: string - success and NULL on failure
 *****************************************************************************/
static const char *qdma_get_ip_type(void *dev_hndl, uint8_t is_vf,
        enum qdma_ip_type ip_type)
{
    uint32_t ip_version;
    int rv = QDMA_SUCCESS;

    // MD: Check for a valid device handle
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n",
                __func__, -QDMA_ERR_INV_PARAM);
        return NULL;
    }

    switch (ip_type) {
    case QDMA_VERSAL_HARD_IP:
        return "Versal Hard IP";
    case QDMA_VERSAL_SOFT_IP:
        return "Versal Soft IP";
    case QDMA_SOFT_IP:
        return "QDMA Soft IP";
    case EQDMA_SOFT_IP:
        // MD: Get the IP version for EQDMA
        rv = eqdma_get_ip_version(dev_hndl, is_vf, &ip_version);
        if (rv != QDMA_SUCCESS)
            return NULL;

        // MD: Determine the IP version string
        if (ip_version == EQDMA_IP_VERSION_4)
            return "EQDMA4.0 Soft IP";
        else if (ip_version == EQDMA_IP_VERSION_5)
            return "EQDMA5.0 Soft IP";

        // MD: Log an error for invalid EQDMA IP version
        qdma_log_error("%s: invalid eqdma ip version(%d), err:%d\n",
                __func__, ip_version, -QDMA_ERR_INV_PARAM);
        return NULL;
    default:
        // MD: Log an error for invalid IP type
        qdma_log_error("%s: invalid ip type(%d), err:%d\n",
                __func__, ip_type, -QDMA_ERR_INV_PARAM);
        return NULL;
    }
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_get_device_type() - Function to get the device type in
 * string format
 *
 * @device_type: Device Type
 *
 * Return: string - success and NULL on failure
 *****************************************************************************/
static const char *qdma_get_device_type(enum qdma_device_type device_type)
{
    // MD: Determine the string representation of the device type
    switch (device_type) {
    case QDMA_DEVICE_SOFT:
        return "Soft IP";
    case QDMA_DEVICE_VERSAL_CPM4:
        return "Versal CPM4 Hard IP";
    case QDMA_DEVICE_VERSAL_CPM5:
        return "Versal Hard CPM5";
    default:
        // MD: Log an error for invalid device type
        qdma_log_error("%s: invalid device type(%d), err:%d\n",
                __func__, device_type, -QDMA_ERR_INV_PARAM);
        return NULL;
    }
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_get_vivado_release_id() - Function to get the vivado release id in
 * string format
 *
 * @vivado_release_id: Vivado release ID
 *
 * Return: string - success and NULL on failure
 *****************************************************************************/
static const char *qdma_get_vivado_release_id(
                enum qdma_vivado_release_id vivado_release_id)
{
    // MD: Determine the string representation of the Vivado release ID
    switch (vivado_release_id) {
    case QDMA_VIVADO_2018_3:
        return "vivado 2018.3";
    case QDMA_VIVADO_2019_1:
        return "vivado 2019.1";
    case QDMA_VIVADO_2019_2:
        return "vivado 2019.2";
    case QDMA_VIVADO_2020_1:
        return "vivado 2020.1";
    case QDMA_VIVADO_2020_2:
        return "vivado 2020.2";
    case QDMA_VIVADO_2021_1:
        return "vivado 2021.1";
    case QDMA_VIVADO_2022_1:
        return "vivado 2022.1";
    default:
        // MD: Log an error for invalid Vivado release ID
        qdma_log_error("%s: invalid vivado_release_id(%d), err:%d\n",
                __func__, vivado_release_id, -QDMA_ERR_INV_PARAM);
        return NULL;
    }
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_write_csr_values() - Write values to CSR registers
 *
 * @dev_hndl: Device handle
 * @reg_offst: Register offset
 * @idx: Starting index
 * @cnt: Number of registers to write
 * @values: Array of values to write
 *****************************************************************************/
void qdma_write_csr_values(void *dev_hndl, uint32_t reg_offst,
        uint32_t idx, uint32_t cnt, const uint32_t *values)
{
    uint32_t index, reg_addr;

    // MD: Iterate over the specified range and write values to the registers
    for (index = idx; index < (idx + cnt); index++) {
        reg_addr = reg_offst + (index * sizeof(uint32_t));
        qdma_reg_write(dev_hndl, reg_addr, values[index - idx]);
    }
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_read_csr_values() - Read values from CSR registers
 *
 * @dev_hndl: Device handle
 * @reg_offst: Register offset
 * @idx: Starting index
 * @cnt: Number of registers to read
 * @values: Array to store read values
 *****************************************************************************/
void qdma_read_csr_values(void *dev_hndl, uint32_t reg_offst,
        uint32_t idx, uint32_t cnt, uint32_t *values)
{
    uint32_t index, reg_addr;

    // MD: Calculate the starting register address
    reg_addr = reg_offst + (idx * sizeof(uint32_t));
    // MD: Iterate over the specified range and read values from the registers
    for (index = 0; index < cnt; index++) {
        values[index] = qdma_reg_read(dev_hndl, reg_addr +
                                      (index * sizeof(uint32_t)));
    }
}

void qdma_fetch_version_details(void *dev_hndl, uint8_t is_vf,
    uint32_t version_reg_val, struct qdma_hw_version_info *version_info)
{
    uint32_t rtl_version, vivado_release_id, ip_type, device_type;
    const char *version_str;

    // MD: Check for a valid device handle
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n",
                __func__, -QDMA_ERR_INV_PARAM);
        return;
    }

    // MD: Extract version details based on whether it's a VF or not
    if (!is_vf) {
        rtl_version = FIELD_GET(QDMA_GLBL2_RTL_VERSION_MASK, version_reg_val);
        vivado_release_id = FIELD_GET(QDMA_GLBL2_VIVADO_RELEASE_MASK, version_reg_val);
        device_type = FIELD_GET(QDMA_GLBL2_DEVICE_ID_MASK, version_reg_val);
        ip_type = FIELD_GET(QDMA_GLBL2_VERSAL_IP_MASK, version_reg_val);
    } else {
        rtl_version = FIELD_GET(QDMA_GLBL2_VF_RTL_VERSION_MASK, version_reg_val);
        vivado_release_id = FIELD_GET(QDMA_GLBL2_VF_VIVADO_RELEASE_MASK, version_reg_val);
        device_type = FIELD_GET(QDMA_GLBL2_VF_DEVICE_ID_MASK, version_reg_val);
        ip_type = FIELD_GET(QDMA_GLBL2_VF_VERSAL_IP_MASK, version_reg_val);
    }

    // MD: Determine RTL version
    switch (rtl_version) {
    case 0:
        version_info->rtl_version = QDMA_RTL_BASE;
        break;
    case 1:
        version_info->rtl_version = QDMA_RTL_PATCH;
        break;
    default:
        version_info->rtl_version = QDMA_RTL_NONE;
        break;
    }

    // MD: Get RTL version string
    version_str = qdma_get_rtl_version(version_info->rtl_version);
    if (version_str != NULL)
        qdma_strncpy(version_info->qdma_rtl_version_str, version_str, QDMA_HW_VERSION_STRING_LEN);

    // MD: Determine device type
    switch (device_type) {
    case 0:
        version_info->device_type = QDMA_DEVICE_SOFT;
        break;
    case 1:
        version_info->device_type = QDMA_DEVICE_VERSAL_CPM4;
        break;
    case 2:
        version_info->device_type = QDMA_DEVICE_VERSAL_CPM5;
        break;
    default:
        version_info->device_type = QDMA_DEVICE_NONE;
        break;
    }

    // MD: Get device type string
    version_str = qdma_get_device_type(version_info->device_type);
    if (version_str != NULL)
        qdma_strncpy(version_info->qdma_device_type_str, version_str, QDMA_HW_VERSION_STRING_LEN);

    // MD: Determine IP type based on device type
    if (version_info->device_type == QDMA_DEVICE_SOFT) {
        switch (ip_type) {
        case 0:
            version_info->ip_type = QDMA_SOFT_IP;
            break;
        case 1:
        case 2:
            version_info->ip_type = EQDMA_SOFT_IP;
            break;
        default:
            version_info->ip_type = QDMA_NONE_IP;
        }
    } else {
        switch (ip_type) {
        case 0:
            version_info->ip_type = QDMA_VERSAL_HARD_IP;
            break;
        case 1:
            version_info->ip_type = QDMA_VERSAL_SOFT_IP;
            break;
        default:
            version_info->ip_type = QDMA_NONE_IP;
        }
    }

    // MD: Get IP type string
    version_str = qdma_get_ip_type(dev_hndl, is_vf, version_info->ip_type);
    if (version_str != NULL)
        qdma_strncpy(version_info->qdma_ip_type_str, version_str, QDMA_HW_VERSION_STRING_LEN);

    // MD: Determine Vivado release based on IP type
    if (version_info->ip_type == QDMA_SOFT_IP) {
        switch (vivado_release_id) {
        case 0:
            version_info->vivado_release = QDMA_VIVADO_2018_3;
            break;
        case 1:
            version_info->vivado_release = QDMA_VIVADO_2019_1;
            break;
        case 2:
            version_info->vivado_release = QDMA_VIVADO_2019_2;
            break;
        default:
            version_info->vivado_release = QDMA_VIVADO_NONE;
            break;
        }
    } else if (version_info->ip_type == EQDMA_SOFT_IP) {
        switch (vivado_release_id) {
        case 0:
            version_info->vivado_release = QDMA_VIVADO_2020_1;
            break;
        case 1:
            version_info->vivado_release = QDMA_VIVADO_2020_2;
            break;
        case 2:
            version_info->vivado_release = QDMA_VIVADO_2022_1;
            break;
        default:
            version_info->vivado_release = QDMA_VIVADO_NONE;
            break;
        }
    } else if (version_info->device_type == QDMA_DEVICE_VERSAL_CPM5) {
        switch (vivado_release_id) {
        case 0:
            version_info->vivado_release = QDMA_VIVADO_2021_1;
            break;
        case 1:
            version_info->vivado_release = QDMA_VIVADO_2022_1;
            break;
        default:
            version_info->vivado_release = QDMA_VIVADO_NONE;
            break;
        }
    } else { // MD: Versal case
        switch (vivado_release_id) {
        case 0:
            version_info->vivado_release = QDMA_VIVADO_2019_2;
            break;
        default:
            version_info->vivado_release = QDMA_VIVADO_NONE;
            break;
        }
    }

    // MD: Get Vivado release ID string
    version_str = qdma_get_vivado_release_id(version_info->vivado_release);
    if (version_str != NULL)
        qdma_strncpy(version_info->qdma_vivado_release_id_str, version_str, QDMA_HW_VERSION_STRING_LEN);
}

/* MD:
 * dump_reg() - Helper function to dump register value into string
 *
 * @buf: Buffer to store the formatted string
 * @buf_sz: Size of the buffer
 * @raddr: Register address
 * @rname: Register name
 * @rval: Register value
 *
 * Return: Length of the string copied into buffer
 */
int dump_reg(char *buf, int buf_sz, uint32_t raddr,
        const char *rname, uint32_t rval)
{
    // MD: Ensure the buffer size is sufficient
    if (buf_sz < DEBGFS_LINE_SZ) {
        qdma_log_error("%s: buf_sz(%d) < expected(%d): err: %d\n",
                        __func__, buf_sz, DEBGFS_LINE_SZ, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Format the register information into the buffer
    return QDMA_SNPRINTF_S(buf, buf_sz, DEBGFS_LINE_SZ,
            "[%#7x] %-47s %#-10x %u\n", raddr, rname, rval, rval);
}

/* MD:*
 * qdma_memset() - Function to set a block of memory with a specific value
 *
 * @to:   Pointer to the memory block to fill
 * @val:  Value to set
 * @size: Number of bytes to set
 */
void qdma_memset(void *to, uint8_t val, uint32_t size)
{
    uint32_t i;
    uint8_t *_to = (uint8_t *)to;

    // MD: Iterate over the memory block and set each byte to the specified value
    for (i = 0; i < size; i++)
        _to[i] = val;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_queue_cmpt_cidx_read() - Function to read the CMPT CIDX register
 *
 * @dev_hndl: Device handle
 * @is_vf:    Whether PF or VF
 * @qid:      Queue id relative to the PF/VF calling this API
 * @reg_info: Pointer to array to hold the values read
 *
 * Return: 0 - success and < 0 - failure
 *****************************************************************************/
static int qdma_queue_cmpt_cidx_read(void *dev_hndl, uint8_t is_vf,
        uint16_t qid, struct qdma_q_cmpt_cidx_reg_info *reg_info)
{
    uint32_t reg_val = 0;
    uint32_t reg_addr = (is_vf) ? QDMA_OFFSET_VF_DMAP_SEL_CMPT_CIDX :
            QDMA_OFFSET_DMAP_SEL_CMPT_CIDX;

    // MD: Check for a valid device handle
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n",
                __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }
    // MD: Check for a valid reg_info pointer
    if (!reg_info) {
        qdma_log_error("%s: reg_info is NULL, err:%d\n",
                __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Calculate the register address based on the queue ID
    reg_addr += qid * QDMA_CMPT_CIDX_STEP;

    // MD: Read the register value
    reg_val = qdma_reg_read(dev_hndl, reg_addr);

    // MD: Extract and store the register fields into reg_info
    reg_info->wrb_cidx =
        FIELD_GET(QDMA_DMAP_SEL_CMPT_WRB_CIDX_MASK, reg_val);
    reg_info->counter_idx =
        (uint8_t)(FIELD_GET(QDMA_DMAP_SEL_CMPT_CNT_THRESH_MASK, reg_val));
    reg_info->wrb_en =
        (uint8_t)(FIELD_GET(QDMA_DMAP_SEL_CMPT_STS_DESC_EN_MASK, reg_val));
    reg_info->irq_en =
        (uint8_t)(FIELD_GET(QDMA_DMAP_SEL_CMPT_IRQ_EN_MASK, reg_val));
    reg_info->timer_idx =
        (uint8_t)(FIELD_GET(QDMA_DMAP_SEL_CMPT_TMR_CNT_MASK, reg_val));
    reg_info->trig_mode =
        (uint8_t)(FIELD_GET(QDMA_DMAP_SEL_CMPT_TRG_MODE_MASK, reg_val));

    return QDMA_SUCCESS;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_initiate_flr() - Function to initiate Function Level Reset
 *
 * @dev_hndl: Device handle
 * @is_vf:    Whether PF or VF
 *
 * Return: 0 - success and < 0 - failure
 *****************************************************************************/
static int qdma_initiate_flr(void *dev_hndl, uint8_t is_vf)
{
    uint32_t reg_addr = (is_vf) ? QDMA_OFFSET_VF_REG_FLR_STATUS :
            QDMA_OFFSET_PF_REG_FLR_STATUS;

    // MD: Check for a valid device handle
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n",
                __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Write to the FLR status register to initiate reset
    qdma_reg_write(dev_hndl, reg_addr, 1);

    return QDMA_SUCCESS;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_is_flr_done() - Function to check whether the FLR is done or not
 *
 * @dev_hndl: Device handle
 * @is_vf:    Whether PF or VF
 * @done:     If FLR process completed, done is 1 else 0.
 *
 * Return: 0 - success and < 0 - failure
 *****************************************************************************/
static int qdma_is_flr_done(void *dev_hndl, uint8_t is_vf, uint8_t *done)
{
    int rv;
    uint32_t reg_addr = (is_vf) ? QDMA_OFFSET_VF_REG_FLR_STATUS :
            QDMA_OFFSET_PF_REG_FLR_STATUS;

    // MD: Check for a valid device handle
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n",
                __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }
    // MD: Check for a valid done pointer
    if (!done) {
        qdma_log_error("%s: done is NULL, err:%d\n",
                __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Wait for the FLR status to become zero
    rv = hw_monitor_reg(dev_hndl, reg_addr, QDMA_FLR_STATUS_MASK,
            0, 5 * QDMA_REG_POLL_DFLT_INTERVAL_US,
            QDMA_REG_POLL_DFLT_TIMEOUT_US);
    if (rv < 0)
        *done = 0;
    else
        *done = 1;

    return QDMA_SUCCESS;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_is_config_bar() - Function for the config bar verification
 *
 * @dev_hndl: Device handle
 * @is_vf:    Whether PF or VF
 * @ip:       Pointer to store the IP type
 *
 * Return: 0 - success and < 0 - failure
 *****************************************************************************/
static int qdma_is_config_bar(void *dev_hndl, uint8_t is_vf, enum qdma_ip *ip)
{
    uint32_t reg_val = 0;
    uint32_t reg_addr = (is_vf) ? QDMA_OFFSET_VF_VERSION :
            QDMA_OFFSET_CONFIG_BLOCK_ID;

    // MD: Check for a valid device handle
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n",
                __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Read the version or config block ID register
    reg_val = qdma_reg_read(dev_hndl, reg_addr);

    // MD: Determine the IP type based on the register value
    if (is_vf) {
        if (FIELD_GET(QDMA_GLBL2_VF_UNIQUE_ID_MASK, reg_val)
                != QDMA_MAGIC_NUMBER) {
            // MD: It's either QDMA or Versal
#ifdef EQDMA_CPM5_VF_GT_256Q_SUPPORTED
            *ip = EQDMA_CPM5_IP;
            reg_addr = EQDMA_CPM5_OFFSET_VF_VERSION;
#else
            *ip = EQDMA_IP;
            reg_addr = EQDMA_OFFSET_VF_VERSION;
#endif
            reg_val = qdma_reg_read(dev_hndl, reg_addr);
        } else {
            *ip = QDMA_OR_VERSAL_IP;
            return QDMA_SUCCESS;
        }
    }

    // MD: Validate the config block ID
    if (FIELD_GET(QDMA_CONFIG_BLOCK_ID_MASK, reg_val)
            != QDMA_MAGIC_NUMBER) {
        qdma_log_error("%s: Invalid config bar, err:%d\n",
                    __func__, -QDMA_ERR_HWACC_INV_CONFIG_BAR);
        return -QDMA_ERR_HWACC_INV_CONFIG_BAR;
    }

    return QDMA_SUCCESS;
}

/* MD:*
 * qdma_acc_reg_dump_buf_len() - Calculate the buffer length for register dump
 *
 * @dev_hndl: Device handle
 * @ip_type: IP type
 * @device_type: Device type
 * @buflen: Pointer to store the calculated buffer length
 *
 * Return: 0 on success, negative error code on failure
 */
int qdma_acc_reg_dump_buf_len(void *dev_hndl, enum qdma_ip_type ip_type,
        enum qdma_device_type device_type, int *buflen)
{
    uint32_t len = 0;
    int rv = 0;

    *buflen = 0;

    // MD: Check for a valid device handle
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n",
            __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Determine buffer length based on IP and device type
    switch (ip_type) {
    case QDMA_SOFT_IP:
        len = qdma_soft_reg_dump_buf_len();
        break;
    case QDMA_VERSAL_HARD_IP:
        if (device_type == QDMA_DEVICE_VERSAL_CPM4)
            len = qdma_cpm4_reg_dump_buf_len();
        else if (device_type == QDMA_DEVICE_VERSAL_CPM5)
            len = eqdma_cpm5_reg_dump_buf_len();
        else {
            qdma_log_error("%s: Invalid device type, err = %d",
                __func__, -QDMA_ERR_INV_PARAM);
            return -QDMA_ERR_INV_PARAM;
        }
        break;
    case EQDMA_SOFT_IP:
        len = eqdma_reg_dump_buf_len();
        break;
    default:
        qdma_log_error("%s: Invalid version number, err = %d",
            __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    *buflen = (int)len;
    return rv;
}

/* MD:*
 * qdma_acc_reg_info_len() - Calculate buffer length and number of registers
 *
 * @dev_hndl: Device handle
 * @ip_type: IP type
 * @device_type: Device type
 * @buflen: Pointer to store the calculated buffer length
 * @num_regs: Pointer to store the number of registers
 *
 * Return: 0 on success, negative error code on failure
 */
int qdma_acc_reg_info_len(void *dev_hndl, enum qdma_ip_type ip_type,
        enum qdma_device_type device_type, int *buflen, int *num_regs)
{
    uint32_t len = 0;
    int rv = 0;

    // MD: Check for valid pointers
    if (!dev_hndl || !buflen || !num_regs) {
        qdma_log_error("%s: Invalid parameter, err:%d\n",
            __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    *buflen = 0;

    // MD: Determine buffer length and number of registers based on IP and device type
    switch (ip_type) {
    case QDMA_SOFT_IP:
        len = 0;
        *num_regs = 0;
        break;
    case QDMA_VERSAL_HARD_IP:
        if (device_type == QDMA_DEVICE_VERSAL_CPM4) {
            len = qdma_cpm4_reg_dump_buf_len();
            *num_regs = (int)((len / REG_DUMP_SIZE_PER_LINE) - 1);
        } else if (device_type == QDMA_DEVICE_VERSAL_CPM5) {
            len = eqdma_cpm5_reg_dump_buf_len();
            *num_regs = (int)((len / REG_DUMP_SIZE_PER_LINE) - 1);
        } else {
            qdma_log_error("%s: Invalid device type, err = %d",
                __func__, -QDMA_ERR_INV_PARAM);
            return -QDMA_ERR_INV_PARAM;
        }
        break;
    case EQDMA_SOFT_IP:
        len = eqdma_reg_dump_buf_len();
        *num_regs = (int)((len / REG_DUMP_SIZE_PER_LINE) - 1);
        break;
    default:
        qdma_log_error("%s: Invalid version number, err = %d",
            __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    *buflen = (int)len;
    return rv;
}

/* MD:*
 * qdma_acc_context_buf_len() - Calculate context buffer length
 *
 * @dev_hndl: Device handle
 * @ip_type: IP type
 * @device_type: Device type
 * @st: Stream type
 * @q_type: Queue type
 * @buflen: Pointer to store the calculated buffer length
 *
 * Return: 0 on success, negative error code on failure
 */
int qdma_acc_context_buf_len(void *dev_hndl, enum qdma_ip_type ip_type,
        enum qdma_device_type device_type, uint8_t st,
        enum qdma_dev_q_type q_type, uint32_t *buflen)
{
    int rv = 0;

    *buflen = 0;
    // MD: Check for a valid device handle
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n",
            __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Determine context buffer length based on IP and device type
    switch (ip_type) {
    case QDMA_SOFT_IP:
        rv = qdma_soft_context_buf_len(st, q_type, buflen);
        break;
    case QDMA_VERSAL_HARD_IP:
        if (device_type == QDMA_DEVICE_VERSAL_CPM4)
            rv = qdma_cpm4_context_buf_len(st, q_type, buflen);
        else if (device_type == QDMA_DEVICE_VERSAL_CPM5)
            rv = eqdma_cpm5_context_buf_len(st, q_type, buflen);
        else {
            qdma_log_error("%s: Invalid device type, err = %d",
                __func__, -QDMA_ERR_INV_PARAM);
            return -QDMA_ERR_INV_PARAM;
        }
        break;
    case EQDMA_SOFT_IP:
        rv = eqdma_context_buf_len(st, q_type, buflen);
        break;
    default:
        qdma_log_error("%s: Invalid version number, err = %d",
            __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    return rv;
}

/* MD:*
 * qdma_acc_get_num_config_regs() - Get number of configuration registers
 *
 * @dev_hndl: Device handle
 * @ip_type: IP type
 * @device_type: Device type
 * @num_regs: Pointer to store the number of configuration registers
 *
 * Return: 0 on success, negative error code on failure
 */
int qdma_acc_get_num_config_regs(void *dev_hndl, enum qdma_ip_type ip_type,
        enum qdma_device_type device_type, uint32_t *num_regs)
{
    int rv = 0;

    *num_regs = 0;
    // MD: Check for a valid device handle
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n",
            __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Determine the number of configuration registers based on IP and device type
    switch (ip_type) {
    case QDMA_SOFT_IP:
        rv = qdma_get_config_num_regs();
        break;
    case QDMA_VERSAL_HARD_IP:
        if (device_type == QDMA_DEVICE_VERSAL_CPM4)
            rv = qdma_cpm4_get_config_num_regs();
        else if (device_type == QDMA_DEVICE_VERSAL_CPM5)
            rv = eqdma_cpm5_get_config_num_regs();
        else {
            qdma_log_error("%s: Invalid device type, err = %d",
                __func__, -QDMA_ERR_INV_PARAM);
            return -QDMA_ERR_INV_PARAM;
        }
        break;
    case EQDMA_SOFT_IP:
        rv = eqdma_get_config_num_regs();
        break;
    default:
        qdma_log_error("%s: Invalid version number, err = %d",
            __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    *num_regs = rv;
    return 0;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_acc_get_config_regs() - Function to get QDMA configuration registers.
 *
 * @dev_hndl:   Device handle
 * @is_vf:      Whether PF or VF
 * @ip_type:    QDMA IP Type
 * @device_type: QDMA DEVICE Type
 * @reg_data:   Pointer to register data to be filled
 *
 * Return: Length up-till the buffer is filled - success and < 0 - failure
 *****************************************************************************/
int qdma_acc_get_config_regs(void *dev_hndl, uint8_t is_vf,
        enum qdma_ip_type ip_type, enum qdma_device_type device_type,
        uint32_t *reg_data)
{
    struct xreg_info *reg_info;
    uint32_t count = 0;
    uint32_t num_regs;
    int rv = 0;

    // MD: Check for a valid device handle
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n",
                __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Configuration registers are not valid for VF
    if (is_vf) {
        qdma_log_error("%s: Get Config regs not valid for VF, err:%d\n",
            __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Check for a valid reg_data pointer
    if (reg_data == NULL) {
        qdma_log_error("%s: reg_data is NULL, err:%d\n",
                        __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Determine the number of registers and get register info based on IP and device type
    switch (ip_type) {
    case QDMA_SOFT_IP:
        num_regs = qdma_get_config_num_regs();
        reg_info = qdma_get_config_regs();
        break;
    case QDMA_VERSAL_HARD_IP:
        if (device_type == QDMA_DEVICE_VERSAL_CPM4) {
            num_regs = qdma_cpm4_get_config_num_regs();
            reg_info = qdma_cpm4_get_config_regs();
        } else if (device_type == QDMA_DEVICE_VERSAL_CPM5) {
            num_regs = eqdma_cpm5_get_config_num_regs();
            reg_info = eqdma_cpm5_get_config_regs();
        } else {
            qdma_log_error("%s: Invalid device type, err = %d",
                __func__, -QDMA_ERR_INV_PARAM);
            return -QDMA_ERR_INV_PARAM;
        }
        break;
    case EQDMA_SOFT_IP:
        num_regs = eqdma_get_config_num_regs();
        reg_info = eqdma_get_config_regs();
        break;
    default:
        qdma_log_error("%s: Invalid version number, err = %d",
            __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Read and store register values
    for (count = 0; count < num_regs - 1; count++) {
        reg_data[count] = qdma_reg_read(dev_hndl, reg_info[count].addr);
    }

    return rv;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_acc_dump_config_regs() - Function to get QDMA config register dump in a buffer
 *
 * @dev_hndl:   Device handle
 * @is_vf:      Whether PF or VF
 * @ip_type:    QDMA IP Type
 * @device_type: QDMA DEVICE Type
 * @buf:        Pointer to buffer to be filled
 * @buflen:     Length of the buffer
 *
 * Return: Length up-till the buffer is filled - success and < 0 - failure
 *****************************************************************************/
int qdma_acc_dump_config_regs(void *dev_hndl, uint8_t is_vf,
        enum qdma_ip_type ip_type, enum qdma_device_type device_type,
        char *buf, uint32_t buflen)
{
    int rv = 0;

    // MD: Dump configuration registers based on IP and device type
    switch (ip_type) {
    case QDMA_SOFT_IP:
        // MD: Dump config registers for QDMA Soft IP
        rv = qdma_soft_dump_config_regs(dev_hndl, is_vf, buf, buflen);
        break;
    case QDMA_VERSAL_HARD_IP:
        // MD: Dump config registers for Versal Hard IP based on device type
        if (device_type == QDMA_DEVICE_VERSAL_CPM4)
            rv = qdma_cpm4_dump_config_regs(dev_hndl, is_vf, buf, buflen);
        else if (device_type == QDMA_DEVICE_VERSAL_CPM5)
            rv = eqdma_cpm5_dump_config_regs(dev_hndl, is_vf, buf, buflen);
        else {
            // MD: Log error for invalid device type
            qdma_log_error("%s: Invalid device type, err = %d",
                __func__, -QDMA_ERR_INV_PARAM);
            return -QDMA_ERR_INV_PARAM;
        }
        break;
    case EQDMA_SOFT_IP:
        // MD: Dump config registers for EQDMA Soft IP
        rv = eqdma_dump_config_regs(dev_hndl, is_vf, buf, buflen);
        break;
    default:
        // MD: Log error for invalid IP type
        qdma_log_error("%s: Invalid version number, err = %d",
            __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    return rv;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_acc_dump_reg_info() - Function to dump fields in a specified register.
 *
 * @dev_hndl:   Device handle
 * @ip_type:    QDMA IP Type
 * @device_type: QDMA DEVICE Type
 * @reg_addr:   Register address
 * @num_regs:   Number of registers
 * @buf:        Pointer to buffer to be filled
 * @buflen:     Length of the buffer
 *
 * Return: Length up-till the buffer is filled - success and < 0 - failure
 *****************************************************************************/
int qdma_acc_dump_reg_info(void *dev_hndl, enum qdma_ip_type ip_type,
        enum qdma_device_type device_type, uint32_t reg_addr,
        uint32_t num_regs, char *buf, uint32_t buflen)
{
    int rv = 0;

    // MD: Check for a valid device handle
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n",
                __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Check for a valid buffer
    if (!buf || !buflen) {
        qdma_log_error("%s: Invalid input buffer, err = %d",
            __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Dump register information based on IP and device type
    switch (ip_type) {
    case QDMA_SOFT_IP:
        // MD: Indicate unsupported operation for QDMA Soft IP
        QDMA_SNPRINTF_S(buf, buflen, DEBGFS_LINE_SZ,
        "QDMA reg field info not supported for QDMA_SOFT_IP\n");
        break;
    case QDMA_VERSAL_HARD_IP:
        // MD: Dump register info for Versal Hard IP based on device type
        if (device_type == QDMA_DEVICE_VERSAL_CPM4)
            rv = qdma_cpm4_dump_reg_info(dev_hndl, reg_addr, num_regs, buf, buflen);
        else if (device_type == QDMA_DEVICE_VERSAL_CPM5)
            rv = eqdma_cpm5_dump_reg_info(dev_hndl, reg_addr, num_regs, buf, buflen);
        else {
            // MD: Log error for invalid device type
            qdma_log_error("%s: Invalid device type, err = %d",
                __func__, -QDMA_ERR_INV_PARAM);
            return -QDMA_ERR_INV_PARAM;
        }
        break;
    case EQDMA_SOFT_IP:
        // MD: Dump register info for EQDMA Soft IP
        rv = eqdma_dump_reg_info(dev_hndl, reg_addr, num_regs, buf, buflen);
        break;
    default:
        // MD: Log error for invalid IP type
        qdma_log_error("%s: Invalid version number, err = %d",
            __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    return rv;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_acc_dump_queue_context() - Function to get QDMA queue context dump in a
 * buffer
 *
 * @dev_hndl:   Device handle
 * @ip_type:    QDMA IP Type
 * @device_type:QDMA DEVICE Type
 * @st:         Queue Mode (ST or MM)
 * @q_type:     Queue Type
 * @ctxt_data:  Context Data
 * @buf:        Pointer to buffer to be filled
 * @buflen:     Length of the buffer
 *
 * Return:      Length up-till the buffer is filled - success and < 0 - failure
 *****************************************************************************/
int qdma_acc_dump_queue_context(void *dev_hndl,
        enum qdma_ip_type ip_type,
        enum qdma_device_type device_type,
        uint8_t st,
        enum qdma_dev_q_type q_type,
        struct qdma_descq_context *ctxt_data,
        char *buf, uint32_t buflen)
{
    int rv = 0;

    // MD: Determine the appropriate function to call based on IP and device type
    switch (ip_type) {
    case QDMA_SOFT_IP:
        // MD: Dump queue context for QDMA Soft IP
        rv = qdma_soft_dump_queue_context(dev_hndl,
                st, q_type, ctxt_data, buf, buflen);
        break;
    case QDMA_VERSAL_HARD_IP:
        // MD: Dump queue context for Versal Hard IP based on device type
        if (device_type == QDMA_DEVICE_VERSAL_CPM4)
            rv = qdma_cpm4_dump_queue_context(dev_hndl,
                st, q_type, ctxt_data, buf, buflen);
        else if (device_type == QDMA_DEVICE_VERSAL_CPM5)
            rv = eqdma_cpm5_dump_queue_context(dev_hndl,
                st, q_type, ctxt_data, buf, buflen);
        else {
            // MD: Log error for invalid device type
            qdma_log_error("%s: Invalid device type, err = %d",
                __func__, -QDMA_ERR_INV_PARAM);
            return -QDMA_ERR_INV_PARAM;
        }
        break;
    case EQDMA_SOFT_IP:
        // MD: Dump queue context for EQDMA Soft IP
        rv = eqdma_dump_queue_context(dev_hndl,
                st, q_type, ctxt_data, buf, buflen);
        break;
    default:
        // MD: Log error for invalid IP type
        qdma_log_error("%s: Invalid version number, err = %d",
            __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    return rv;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_acc_read_dump_queue_context() - Function to read and dump the queue
 * context in the user-provided buffer. This API is valid only for PF and
 * should not be used for VFs. For VFs, use qdma_dump_queue_context() API
 * after reading the context through mailbox.
 *
 * @dev_hndl:   Device handle
 * @ip_type:    QDMA IP type
 * @device_type:QDMA DEVICE Type
 * @func_id:    Function ID
 * @qid_hw:     Queue ID
 * @st:         Queue Mode (ST or MM)
 * @q_type:     Queue type (H2C/C2H/CMPT)
 * @buf:        Pointer to buffer to be filled
 * @buflen:     Length of the buffer
 *
 * Return:      Length up-till the buffer is filled - success and < 0 - failure
 *****************************************************************************/
int qdma_acc_read_dump_queue_context(void *dev_hndl,
                enum qdma_ip_type ip_type,
                enum qdma_device_type device_type,
                uint16_t func_id,
                uint16_t qid_hw,
                uint8_t st,
                enum qdma_dev_q_type q_type,
                char *buf, uint32_t buflen)
{
    int rv = QDMA_SUCCESS;

    // MD: Determine the appropriate function to call based on IP and device type
    switch (ip_type) {
    case QDMA_SOFT_IP:
        // MD: Read and dump queue context for QDMA Soft IP
        rv = qdma_soft_read_dump_queue_context(dev_hndl, func_id,
                qid_hw, st, q_type, buf, buflen);
        break;
    case QDMA_VERSAL_HARD_IP:
        // MD: Read and dump queue context for Versal Hard IP based on device type
        if (device_type == QDMA_DEVICE_VERSAL_CPM4)
            rv = qdma_cpm4_read_dump_queue_context(dev_hndl,
                func_id, qid_hw, st, q_type, buf, buflen);
        else if (device_type == QDMA_DEVICE_VERSAL_CPM5)
            rv = eqdma_cpm5_read_dump_queue_context(dev_hndl,
                func_id, qid_hw, st, q_type, buf, buflen);
        else {
            // MD: Log error for invalid device type
            qdma_log_error("%s: Invalid device type, err = %d",
            __func__, -QDMA_ERR_INV_PARAM);
            return -QDMA_ERR_INV_PARAM;
        }
        break;
    case EQDMA_SOFT_IP:
        // MD: Read and dump queue context for EQDMA Soft IP
        rv = eqdma_read_dump_queue_context(dev_hndl, func_id,
                qid_hw, st, q_type, buf, buflen);
        break;
    default:
        // MD: Log error for invalid IP type
        qdma_log_error("%s: Invalid version number, err = %d",
            __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    return rv;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_acc_dump_config_reg_list() - Dump the registers
 *
 * @dev_hndl:    Device handle
 * @ip_type:     QDMA IP type
 * @device_type: QDMA device type
 * @num_regs:    Max registers to read
 * @reg_list:    Array of register addresses and values
 * @buf:         Pointer to buffer to be filled
 * @buflen:      Length of the buffer
 *
 * Return: Returns the platform-specific error code
 *****************************************************************************/
int qdma_acc_dump_config_reg_list(void *dev_hndl,
        enum qdma_ip_type ip_type,
        enum qdma_device_type device_type,
        uint32_t num_regs,
        struct qdma_reg_data *reg_list,
        char *buf, uint32_t buflen)
{
    int rv = 0;

    // MD: Check the IP type and call the corresponding function
    switch (ip_type) {
    case QDMA_SOFT_IP:
        rv = qdma_soft_dump_config_reg_list(dev_hndl,
                num_regs, reg_list, buf, buflen);
        break;
    case QDMA_VERSAL_HARD_IP:
        if (device_type == QDMA_DEVICE_VERSAL_CPM4)
            rv = qdma_cpm4_dump_config_reg_list(dev_hndl,
                num_regs, reg_list, buf, buflen);
        else if (device_type == QDMA_DEVICE_VERSAL_CPM5)
            rv = eqdma_cpm5_dump_config_reg_list(dev_hndl,
                num_regs, reg_list, buf, buflen);
        else {
            qdma_log_error("%s: Invalid device type, err = %d\n",
                __func__, -QDMA_ERR_INV_PARAM);
            return -QDMA_ERR_INV_PARAM;
        }
        break;
    case EQDMA_SOFT_IP:
        rv = eqdma_dump_config_reg_list(dev_hndl,
                num_regs, reg_list, buf, buflen);
        break;
    default:
        qdma_log_error("%s: Invalid version number, err = %d\n",
            __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    return rv;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_get_function_number() - Function to get the function number
 *
 * @dev_hndl: Device handle
 * @func_id:  Pointer to hold the function ID
 *
 * Return: 0 on success and < 0 on failure
 *****************************************************************************/
static int qdma_get_function_number(void *dev_hndl, uint16_t *func_id)
{
    // MD: Validate input parameters
    if (!dev_hndl || !func_id) {
        qdma_log_error("%s: dev_handle or func_id is NULL, err:%d\n",
                __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Read the function ID from the register
    *func_id = (uint8_t)qdma_reg_read(dev_hndl,
            QDMA_OFFSET_GLBL2_CHANNEL_FUNC_RET);

    return QDMA_SUCCESS;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_hw_error_intr_setup() - Function to set up the QDMA error interrupt
 *
 * @dev_hndl:       Device handle
 * @func_id:        Function ID
 * @err_intr_index: Interrupt vector
 *
 * Return: 0 on success and < 0 on failure
 *****************************************************************************/
static int qdma_hw_error_intr_setup(void *dev_hndl, uint16_t func_id,
        uint8_t err_intr_index)
{
    uint32_t reg_val = 0;

    // MD: Validate the device handle
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n",
                __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Set up the error interrupt register value
    reg_val = FIELD_SET(QDMA_GLBL_ERR_FUNC_MASK, func_id) |
              FIELD_SET(QDMA_GLBL_ERR_VEC_MASK, err_intr_index);

    // MD: Write the register value to set up the interrupt
    qdma_reg_write(dev_hndl, QDMA_OFFSET_GLBL_ERR_INT, reg_val);

    return QDMA_SUCCESS;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_hw_error_intr_rearm() - Function to re-arm the error interrupt
 *
 * @dev_hndl: Device handle
 *
 * Return: 0 - success and < 0 - failure
 *****************************************************************************/
static int qdma_hw_error_intr_rearm(void *dev_hndl)
{
    uint32_t reg_val = 0;

    // MD: Check for a valid device handle
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n",
                __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Read the current value of the global error interrupt register
    reg_val = qdma_reg_read(dev_hndl, QDMA_OFFSET_GLBL_ERR_INT);
    // MD: Set the ARM bit to re-arm the error interrupt
    reg_val |= FIELD_SET(QDMA_GLBL_ERR_ARM_MASK, 1);

    // MD: Write the updated value back to the global error interrupt register
    qdma_reg_write(dev_hndl, QDMA_OFFSET_GLBL_ERR_INT, reg_val);

    // MD: Return success
    return QDMA_SUCCESS;
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_get_error_code() - Function to get the QDMA access mapped error code
 *
 * @acc_err_code: QDMA access error code
 *
 * Return: Returns the platform-specific error code
 *****************************************************************************/
int qdma_get_error_code(int acc_err_code)
{
    // MD: Retrieve the platform-specific error code based on the access error code
    return qdma_get_err_code(acc_err_code);
}

/* MD:****************************************************************************/
/* MD:*
 * qdma_hw_access_init() - Initialize QDMA hardware access structure
 *
 * @dev_hndl: Device handle
 * @is_vf: Indicates if the device is a Virtual Function (VF)
 * @hw_access: Pointer to the QDMA hardware access structure to initialize
 *
 * Return: 0 on success, negative error code on failure
 *****************************************************************************/
int qdma_hw_access_init(void *dev_hndl, uint8_t is_vf,
                        struct qdma_hw_access *hw_access)
{
    int rv = QDMA_SUCCESS;
    enum qdma_ip ip = EQDMA_IP;
    struct qdma_hw_version_info version_info;

    // MD: Check for a valid device handle
    if (!dev_hndl) {
        qdma_log_error("%s: dev_handle is NULL, err:%d\n",
                       __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }
    // MD: Check for a valid hardware access structure
    if (!hw_access) {
        qdma_log_error("%s: hw_access is NULL, err:%d\n",
                       __func__, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Verify the configuration bar and determine the IP type
    rv = qdma_is_config_bar(dev_hndl, is_vf, &ip);
    if (rv != QDMA_SUCCESS) {
        qdma_log_error("%s: config bar passed is INVALID, err:%d\n",
                       __func__, -QDMA_ERR_INV_PARAM);
        return rv;
    }

    // MD: Initialize the hardware access structure to zero
    qdma_memset(hw_access, 0, sizeof(struct qdma_hw_access));

    // MD: Set the version retrieval function based on the IP type
    if (ip == EQDMA_IP)
        hw_access->qdma_get_version = &eqdma_get_version;
    else if (ip == EQDMA_CPM5_IP)
        hw_access->qdma_get_version = &eqdma_cpm5_get_version;
    else
        hw_access->qdma_get_version = &qdma_get_version;

    // MD: Initialize function pointers for various QDMA operations
    hw_access->qdma_init_ctxt_memory = &qdma_init_ctxt_memory;
    hw_access->qdma_fmap_conf = &qdma_fmap_conf;
    hw_access->qdma_sw_ctx_conf = &qdma_sw_ctx_conf;
    hw_access->qdma_pfetch_ctx_conf = &qdma_pfetch_ctx_conf;
    hw_access->qdma_cmpt_ctx_conf = &qdma_cmpt_ctx_conf;
    hw_access->qdma_hw_ctx_conf = &qdma_hw_ctx_conf;
    hw_access->qdma_credit_ctx_conf = &qdma_credit_ctx_conf;
    hw_access->qdma_indirect_intr_ctx_conf = &qdma_indirect_intr_ctx_conf;
    hw_access->qdma_set_default_global_csr = &qdma_set_default_global_csr;
    hw_access->qdma_global_csr_conf = &qdma_global_csr_conf;
    hw_access->qdma_global_writeback_interval_conf =
        &qdma_global_writeback_interval_conf;
    hw_access->qdma_queue_pidx_update = &qdma_queue_pidx_update;
    hw_access->qdma_queue_cmpt_cidx_read = &qdma_queue_cmpt_cidx_read;
    hw_access->qdma_queue_cmpt_cidx_update = &qdma_queue_cmpt_cidx_update;
    hw_access->qdma_queue_intr_cidx_update = &qdma_queue_intr_cidx_update;
    hw_access->qdma_mm_channel_conf = &qdma_mm_channel_conf;
    hw_access->qdma_get_user_bar = &qdma_get_user_bar;
    hw_access->qdma_get_function_number = &qdma_get_function_number;
    hw_access->qdma_get_device_attributes = &qdma_get_device_attributes;
    hw_access->qdma_hw_error_intr_setup = &qdma_hw_error_intr_setup;
    hw_access->qdma_hw_error_intr_rearm = &qdma_hw_error_intr_rearm;
    hw_access->qdma_hw_error_enable = &qdma_hw_error_enable;
    hw_access->qdma_hw_get_error_name = &qdma_hw_get_error_name;
    hw_access->qdma_hw_error_process = &qdma_hw_error_process;
    hw_access->qdma_dump_config_regs = &qdma_soft_dump_config_regs;
    hw_access->qdma_dump_queue_context = &qdma_soft_dump_queue_context;
    hw_access->qdma_read_dump_queue_context = &qdma_soft_read_dump_queue_context;
    hw_access->qdma_dump_intr_context = &qdma_dump_intr_context;
    hw_access->qdma_is_legacy_intr_pend = &qdma_is_legacy_intr_pend;
    hw_access->qdma_clear_pend_legacy_intr = &qdma_clear_pend_legacy_intr;
    hw_access->qdma_legacy_intr_conf = &qdma_legacy_intr_conf;
    hw_access->qdma_initiate_flr = &qdma_initiate_flr;
    hw_access->qdma_is_flr_done = &qdma_is_flr_done;
    hw_access->qdma_get_error_code = &qdma_get_error_code;
    hw_access->qdma_read_reg_list = &qdma_read_reg_list;
    hw_access->qdma_dump_config_reg_list = &qdma_soft_dump_config_reg_list;
    hw_access->qdma_dump_reg_info = &qdma_dump_reg_info;
    hw_access->mbox_base_pf = QDMA_OFFSET_MBOX_BASE_PF;
    hw_access->mbox_base_vf = QDMA_OFFSET_MBOX_BASE_VF;
    hw_access->qdma_max_errors = QDMA_ERRS_ALL;

    // MD: Retrieve the version information
    rv = hw_access->qdma_get_version(dev_hndl, is_vf, &version_info);
    if (rv != QDMA_SUCCESS)
        return rv;

    // MD: Log the device type, IP type, and Vivado release
    qdma_log_info("Device Type: %s\n",
                  qdma_get_device_type(version_info.device_type));
    qdma_log_info("IP Type: %s\n",
                  qdma_get_ip_type(dev_hndl, is_vf, version_info.ip_type));
    qdma_log_info("Vivado Release: %s\n",
                  qdma_get_vivado_release_id(version_info.vivado_release));

    // MD: Configure function pointers for Versal CPM4 devices
    if (version_info.ip_type == QDMA_VERSAL_HARD_IP &&
        version_info.device_type == QDMA_DEVICE_VERSAL_CPM4) {
        hw_access->qdma_init_ctxt_memory = &qdma_cpm4_init_ctxt_memory;
        hw_access->qdma_qid2vec_conf = &qdma_cpm4_qid2vec_conf;
        hw_access->qdma_fmap_conf = &qdma_cpm4_fmap_conf;
        hw_access->qdma_sw_ctx_conf = &qdma_cpm4_sw_ctx_conf;
        hw_access->qdma_pfetch_ctx_conf = &qdma_cpm4_pfetch_ctx_conf;
        hw_access->qdma_cmpt_ctx_conf = &qdma_cpm4_cmpt_ctx_conf;
        hw_access->qdma_hw_ctx_conf = &qdma_cpm4_hw_ctx_conf;
        hw_access->qdma_credit_ctx_conf = &qdma_cpm4_credit_ctx_conf;
        hw_access->qdma_indirect_intr_ctx_conf = &qdma_cpm4_indirect_intr_ctx_conf;
        hw_access->qdma_set_default_global_csr = &qdma_cpm4_set_default_global_csr;
        hw_access->qdma_queue_pidx_update = &qdma_cpm4_queue_pidx_update;
        hw_access->qdma_queue_cmpt_cidx_update = &qdma_cpm4_queue_cmpt_cidx_update;
        hw_access->qdma_queue_intr_cidx_update = &qdma_cpm4_queue_intr_cidx_update;
        hw_access->qdma_get_user_bar = &qdma_cmp_get_user_bar;
        hw_access->qdma_get_device_attributes = &qdma_cpm4_get_device_attributes;
        hw_access->qdma_dump_config_regs = &qdma_cpm4_dump_config_regs;
        hw_access->qdma_dump_intr_context = &qdma_cpm4_dump_intr_context;
        hw_access->qdma_hw_error_enable = &qdma_cpm4_hw_error_enable;
        hw_access->qdma_hw_error_process = &qdma_cpm4_hw_error_process;
        hw_access->qdma_hw_get_error_name = &qdma_cpm4_hw_get_error_name;
        hw_access->qdma_legacy_intr_conf = NULL;
        hw_access->qdma_read_reg_list = &qdma_cpm4_read_reg_list;
        hw_access->qdma_dump_config_reg_list = &qdma_cpm4_dump_config_reg_list;
        hw_access->qdma_dump_queue_context = &qdma_cpm4_dump_queue_context;
        hw_access->qdma_read_dump_queue_context = &qdma_cpm4_read_dump_queue_context;
        hw_access->qdma_dump_reg_info = &qdma_cpm4_dump_reg_info;
        hw_access->qdma_max_errors = QDMA_CPM4_ERRS_ALL;
    }

    // MD: Configure function pointers for Versal CPM5 devices
    if (version_info.ip_type == QDMA_VERSAL_HARD_IP &&
        version_info.device_type == QDMA_DEVICE_VERSAL_CPM5) {
        hw_access->qdma_init_ctxt_memory = &eqdma_cpm5_init_ctxt_memory;
#ifdef TANDEM_BOOT_SUPPORTED
        hw_access->qdma_init_st_ctxt = &eqdma_cpm5_init_st_ctxt;
#endif
        hw_access->qdma_sw_ctx_conf = &eqdma_cpm5_sw_ctx_conf;
        hw_access->qdma_fmap_conf = &eqdma_cpm5_fmap_conf;
        hw_access->qdma_pfetch_ctx_conf = &eqdma_cpm5_pfetch_ctx_conf;
        hw_access->qdma_cmpt_ctx_conf = &eqdma_cpm5_cmpt_ctx_conf;
        hw_access->qdma_indirect_intr_ctx_conf = &eqdma_cpm5_indirect_intr_ctx_conf;
        hw_access->qdma_dump_config_regs = &eqdma_cpm5_dump_config_regs;
        hw_access->qdma_dump_intr_context = &eqdma_cpm5_dump_intr_context;
        hw_access->qdma_hw_error_enable = &eqdma_cpm5_hw_error_enable;
        hw_access->qdma_hw_error_process = &eqdma_cpm5_hw_error_process;
        hw_access->qdma_hw_get_error_name = &eqdma_cpm5_hw_get_error_name;
        hw_access->qdma_hw_ctx_conf = &eqdma_cpm5_hw_ctx_conf;
        hw_access->qdma_credit_ctx_conf = &eqdma_cpm5_credit_ctx_conf;
        hw_access->qdma_set_default_global_csr = &eqdma_cpm5_set_default_global_csr;
        hw_access->qdma_get_device_attributes = &eqdma_cpm5_get_device_attributes;
        hw_access->qdma_get_user_bar = &eqdma_cpm5_get_user_bar;
        hw_access->qdma_read_reg_list = &eqdma_cpm5_read_reg_list;
        hw_access->qdma_dump_config_reg_list = &eqdma_cpm5_dump_config_reg_list;
        hw_access->qdma_dump_queue_context = &eqdma_cpm5_dump_queue_context;
        hw_access->qdma_read_dump_queue_context = &eqdma_cpm5_read_dump_queue_context;
        hw_access->qdma_dump_reg_info = &eqdma_cpm5_dump_reg_info;
        // MD: Set mailbox offsets for EQDMA
        hw_access->mbox_base_pf = EQDMA_CPM5_OFFSET_MBOX_BASE_PF;
        hw_access->mbox_base_vf = EQDMA_CPM5_OFFSET_MBOX_BASE_VF;
        hw_access->qdma_max_errors = EQDMA_CPM5_ERRS_ALL;
    }

    // MD: Configure function pointers for EQDMA Soft IP
    if (version_info.ip_type == EQDMA_SOFT_IP) {
        hw_access->qdma_init_ctxt_memory = &eqdma_init_ctxt_memory;
        hw_access->qdma_sw_ctx_conf = &eqdma_sw_ctx_conf;
        hw_access->qdma_fmap_conf = &eqdma_fmap_conf;
        hw_access->qdma_pfetch_ctx_conf = &eqdma_pfetch_ctx_conf;
        hw_access->qdma_cmpt_ctx_conf = &eqdma_cmpt_ctx_conf;
        hw_access->qdma_indirect_intr_ctx_conf = &eqdma_indirect_intr_ctx_conf;
        hw_access->qdma_dump_config_regs = &eqdma_dump_config_regs;
        hw_access->qdma_dump_intr_context = &eqdma_dump_intr_context;
        hw_access->qdma_hw_error_enable = &eqdma_hw_error_enable;
        hw_access->qdma_hw_error_process = &eqdma_hw_error_process;
        hw_access->qdma_hw_get_error_name = &eqdma_hw_get_error_name;
        hw_access->qdma_hw_ctx_conf = &eqdma_hw_ctx_conf;
        hw_access->qdma_credit_ctx_conf = &eqdma_credit_ctx_conf;
        hw_access->qdma_set_default_global_csr = &eqdma_set_default_global_csr;
        hw_access->qdma_get_device_attributes = &eqdma_get_device_attributes;
        hw_access->qdma_get_user_bar = &eqdma_get_user_bar;
        hw_access->qdma_read_reg_list = &eqdma_read_reg_list;
        hw_access->qdma_dump_config_reg_list = &eqdma_dump_config_reg_list;
        hw_access->qdma_dump_queue_context = &eqdma_dump_queue_context;
        hw_access->qdma_read_dump_queue_context = &eqdma_read_dump_queue_context;
        hw_access->qdma_dump_reg_info = &eqdma_dump_reg_info;
        // MD: Set mailbox offsets for EQDMA
        hw_access->mbox_base_pf = EQDMA_OFFSET_MBOX_BASE_PF;
        hw_access->mbox_base_vf = EQDMA_OFFSET_MBOX_BASE_VF;
        hw_access->qdma_max_errors = EQDMA_ERRS_ALL;
    }

    return QDMA_SUCCESS;
}
