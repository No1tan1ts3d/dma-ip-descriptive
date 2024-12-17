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

#include "qdma_mbox_protocol.h"

/* MD:* mailbox function status */
#define MBOX_FN_STATUS			0x0
/* MD:* shift value for mailbox function status in msg */
#define		S_MBOX_FN_STATUS_IN_MSG	0
/* MD:* mask value for mailbox function status in msg*/
#define		M_MBOX_FN_STATUS_IN_MSG	0x1
/* MD:* face value for mailbox function status in msg */
#define		F_MBOX_FN_STATUS_IN_MSG	0x1

/* MD:* shift value for out msg */
#define		S_MBOX_FN_STATUS_OUT_MSG	1
/* MD:* mask value for out msg */
#define		M_MBOX_FN_STATUS_OUT_MSG	0x1
/* MD:* face value for out msg */
#define		F_MBOX_FN_STATUS_OUT_MSG	(1 << S_MBOX_FN_STATUS_OUT_MSG)
/* MD:* shift value for status ack */
#define		S_MBOX_FN_STATUS_ACK	2	/* MD: PF only, ack status */
/* MD:* mask value for status ack */
#define		M_MBOX_FN_STATUS_ACK	0x1
/* MD:* face value for status ack */
#define		F_MBOX_FN_STATUS_ACK	(1 << S_MBOX_FN_STATUS_ACK)
/* MD:* shift value for status src */
#define		S_MBOX_FN_STATUS_SRC	4	/* MD: PF only, source func.*/
/* MD:* mask value for status src */
#define		M_MBOX_FN_STATUS_SRC	0xFFF
/* MD:* face value for status src */
#define		G_MBOX_FN_STATUS_SRC(x)	\
		(((x) >> S_MBOX_FN_STATUS_SRC) & M_MBOX_FN_STATUS_SRC)
/* MD:* face value for mailbox function status */
#define MBOX_FN_STATUS_MASK \
		(F_MBOX_FN_STATUS_IN_MSG | \
		 F_MBOX_FN_STATUS_OUT_MSG | \
		 F_MBOX_FN_STATUS_ACK)

/* MD:* mailbox function commands register */
#define MBOX_FN_CMD			0x4
/* MD:* shift value for send command */
#define		S_MBOX_FN_CMD_SND	0
/* MD:* mask value for send command */
#define		M_MBOX_FN_CMD_SND	0x1
/* MD:* face value for send command */
#define		F_MBOX_FN_CMD_SND	(1 << S_MBOX_FN_CMD_SND)
/* MD:* shift value for receive command */
#define		S_MBOX_FN_CMD_RCV	1
/* MD:* mask value for receive command */
#define		M_MBOX_FN_CMD_RCV	0x1
/* MD:* face value for receive command */
#define		F_MBOX_FN_CMD_RCV	(1 << S_MBOX_FN_CMD_RCV)
/* MD:* shift value for vf reset */
#define		S_MBOX_FN_CMD_VF_RESET	3	/* MD: TBD PF only: reset VF */
/* MD:* mask value for vf reset */
#define		M_MBOX_FN_CMD_VF_RESET	0x1
/* MD:* mailbox isr vector register */
#define MBOX_ISR_VEC			0x8
/* MD:* shift value for isr vector */
#define		S_MBOX_ISR_VEC		0
/* MD:* mask value for isr vector */
#define		M_MBOX_ISR_VEC		0x1F
/* MD:* face value for isr vector */
#define		V_MBOX_ISR_VEC(x)	((x) & M_MBOX_ISR_VEC)
/* MD:* mailbox FN target register */
#define MBOX_FN_TARGET			0xC
/* MD:* shift value for FN target id */
#define		S_MBOX_FN_TARGET_ID	0
/* MD:* mask value for FN target id */
#define		M_MBOX_FN_TARGET_ID	0xFFF
/* MD:* face value for FN target id */
#define		V_MBOX_FN_TARGET_ID(x)	((x) & M_MBOX_FN_TARGET_ID)
/* MD:* mailbox isr enable register */
#define MBOX_ISR_EN			0x10
/* MD:* shift value for isr enable */
#define		S_MBOX_ISR_EN		0
/* MD:* mask value for isr enable */
#define		M_MBOX_ISR_EN		0x1
/* MD:* face value for isr enable */
#define		F_MBOX_ISR_EN		0x1
/* MD:* pf acknowledge base */
#define MBOX_PF_ACK_BASE		0x20
/* MD:* pf acknowledge step */
#define MBOX_PF_ACK_STEP		4
/* MD:* pf acknowledge count */
#define MBOX_PF_ACK_COUNT		8
/* MD:* mailbox incoming msg base */
#define MBOX_IN_MSG_BASE		0x800
/* MD:* mailbox outgoing msg base */
#define MBOX_OUT_MSG_BASE		0xc00
/* MD:* mailbox msg step */
#define MBOX_MSG_STEP			4
/* MD:* mailbox register max */
#define MBOX_MSG_REG_MAX		32

/* MD:*
 * enum mbox_msg_op - mailbox messages opcode
 */
#define MBOX_MSG_OP_RSP_OFFSET	0x80
enum mbox_msg_op {
	/* MD:* @MBOX_OP_BYE: vf offline, response not required*/
	MBOX_OP_VF_BYE,
	/* MD:* @MBOX_OP_HELLO: vf online */
	MBOX_OP_HELLO,
	/* MD:* @: FMAP programming request */
	MBOX_OP_FMAP,
	/* MD:* @MBOX_OP_CSR: global CSR registers request */
	MBOX_OP_CSR,
	/* MD:* @MBOX_OP_QREQ: request queues */
	MBOX_OP_QREQ,
	/* MD:* @MBOX_OP_QADD: notify of queue addition */
	MBOX_OP_QNOTIFY_ADD,
	/* MD:* @MBOX_OP_QNOTIFY_DEL: notify of queue deletion */
	MBOX_OP_QNOTIFY_DEL,
	/* MD:* @MBOX_OP_QACTIVE_CNT: get active q count */
	MBOX_OP_GET_QACTIVE_CNT,
	/* MD:* @MBOX_OP_QCTXT_WRT: queue context write */
	MBOX_OP_QCTXT_WRT,
	/* MD:* @MBOX_OP_QCTXT_RD: queue context read */
	MBOX_OP_QCTXT_RD,
	/* MD:* @MBOX_OP_QCTXT_CLR: queue context clear */
	MBOX_OP_QCTXT_CLR,
	/* MD:* @MBOX_OP_QCTXT_INV: queue context invalidate */
	MBOX_OP_QCTXT_INV,
	/* MD:* @MBOX_OP_INTR_CTXT_WRT: interrupt context write */
	MBOX_OP_INTR_CTXT_WRT,
	/* MD:* @MBOX_OP_INTR_CTXT_RD: interrupt context read */
	MBOX_OP_INTR_CTXT_RD,
	/* MD:* @MBOX_OP_INTR_CTXT_CLR: interrupt context clear */
	MBOX_OP_INTR_CTXT_CLR,
	/* MD:* @MBOX_OP_INTR_CTXT_INV: interrupt context invalidate */
	MBOX_OP_INTR_CTXT_INV,
	/* MD:* @MBOX_OP_RESET_PREPARE: PF to VF message for VF reset*/
	MBOX_OP_RESET_PREPARE,
	/* MD:* @MBOX_OP_RESET_DONE: PF reset done */
	MBOX_OP_RESET_DONE,
	/* MD:* @MBOX_OP_REG_LIST_READ: Read the register list */
	MBOX_OP_REG_LIST_READ,
	/* MD:* @MBOX_OP_PF_BYE: pf offline, response required */
	MBOX_OP_PF_BYE,
	/* MD:* @MBOX_OP_PF_RESET_VF_BYE: VF reset BYE, response required*/
	MBOX_OP_PF_RESET_VF_BYE,

	/* MD:* @MBOX_OP_HELLO_RESP: response to @MBOX_OP_HELLO */
	MBOX_OP_HELLO_RESP = 0x81,
	/* MD:* @MBOX_OP_FMAP_RESP: response to @MBOX_OP_FMAP */
	MBOX_OP_FMAP_RESP,
	/* MD:* @MBOX_OP_CSR_RESP: response to @MBOX_OP_CSR */
	MBOX_OP_CSR_RESP,
	/* MD:* @MBOX_OP_QREQ_RESP: response to @MBOX_OP_QREQ */
	MBOX_OP_QREQ_RESP,
	/* MD:* @MBOX_OP_QADD: notify of queue addition */
	MBOX_OP_QNOTIFY_ADD_RESP,
	/* MD:* @MBOX_OP_QNOTIFY_DEL: notify of queue deletion */
	MBOX_OP_QNOTIFY_DEL_RESP,
	/* MD:* @MBOX_OP_QACTIVE_CNT_RESP: get active q count */
	MBOX_OP_GET_QACTIVE_CNT_RESP,
	/* MD:* @MBOX_OP_QCTXT_WRT_RESP: response to @MBOX_OP_QCTXT_WRT */
	MBOX_OP_QCTXT_WRT_RESP,
	/* MD:* @MBOX_OP_QCTXT_RD_RESP: response to @MBOX_OP_QCTXT_RD */
	MBOX_OP_QCTXT_RD_RESP,
	/* MD:* @MBOX_OP_QCTXT_CLR_RESP: response to @MBOX_OP_QCTXT_CLR */
	MBOX_OP_QCTXT_CLR_RESP,
	/* MD:* @MBOX_OP_QCTXT_INV_RESP: response to @MBOX_OP_QCTXT_INV */
	MBOX_OP_QCTXT_INV_RESP,
	/* MD:* @MBOX_OP_INTR_CTXT_WRT_RESP: response to @MBOX_OP_INTR_CTXT_WRT */
	MBOX_OP_INTR_CTXT_WRT_RESP,
	/* MD:* @MBOX_OP_INTR_CTXT_RD_RESP: response to @MBOX_OP_INTR_CTXT_RD */
	MBOX_OP_INTR_CTXT_RD_RESP,
	/* MD:* @MBOX_OP_INTR_CTXT_CLR_RESP: response to @MBOX_OP_INTR_CTXT_CLR */
	MBOX_OP_INTR_CTXT_CLR_RESP,
	/* MD:* @MBOX_OP_INTR_CTXT_INV_RESP: response to @MBOX_OP_INTR_CTXT_INV */
	MBOX_OP_INTR_CTXT_INV_RESP,
	/* MD:* @MBOX_OP_RESET_PREPARE_RESP: response to @MBOX_OP_RESET_PREPARE */
	MBOX_OP_RESET_PREPARE_RESP,
	/* MD:* @MBOX_OP_RESET_DONE_RESP: response to @MBOX_OP_PF_VF_RESET */
	MBOX_OP_RESET_DONE_RESP,
	/* MD:* @MBOX_OP_REG_LIST_READ_RESP: response to @MBOX_OP_REG_LIST_READ */
	MBOX_OP_REG_LIST_READ_RESP,
	/* MD:* @MBOX_OP_PF_BYE_RESP: response to @MBOX_OP_PF_BYE */
	MBOX_OP_PF_BYE_RESP,
	/* MD:* @MBOX_OP_PF_RESET_VF_BYE_RESP:
	 * response to @MBOX_OP_PF_RESET_VF_BYE
	 */
	MBOX_OP_PF_RESET_VF_BYE_RESP,
	/* MD:* @MBOX_OP_MAX: total mbox opcodes*/
	MBOX_OP_MAX
};

/* MD:*
 * struct mbox_msg_hdr - mailbox message header
 */
struct mbox_msg_hdr {
	/* MD:* @op: opcode */
	uint8_t op;
	/* MD:* @status: execution status */
	char status;
	/* MD:* @src_func_id: src function */
	uint16_t src_func_id;
	/* MD:* @dst_func_id: dst function */
	uint16_t dst_func_id;
};

/* MD:*
 * struct mbox_msg_fmap - FMAP programming command
 */
struct mbox_msg_hello {
	/* MD:* @hdr: mailbox message header */
	struct mbox_msg_hdr hdr;
	/* MD:* @qbase: start queue number in the queue range */
	uint32_t qbase;
	/* MD:* @qmax: max queue number in the queue range(0-2k) */
	uint32_t qmax;
	/* MD:* @dev_cap: device capability */
	struct qdma_dev_attributes dev_cap;
	/* MD:* @dma_device_index: dma_device_index */
	uint32_t dma_device_index;
};

/* MD:*
 * struct mbox_msg_active_qcnt - get active queue count command
 */
struct mbox_msg_active_qcnt {
	/* MD:* @hdr: mailbox message header */
	struct mbox_msg_hdr hdr;
	/* MD:* @h2c_queues: number of h2c queues */
	uint32_t h2c_queues;
	/* MD:* @c2h_queues: number of c2h queues */
	uint32_t c2h_queues;
	/* MD:* @cmpt_queues: number of cmpt queues */
	uint32_t cmpt_queues;
};

/* MD:*
 * struct mbox_msg_fmap - FMAP programming command
 */
struct mbox_msg_fmap {
	/* MD:* @hdr: mailbox message header */
	struct mbox_msg_hdr hdr;
	/* MD:* @qbase: start queue number in the queue range */
	int qbase;
	/* MD:* @qmax: max queue number in the queue range(0-2k) */
	uint32_t qmax;
};

/* MD:*
 * struct mbox_msg_csr - mailbox csr reading message
 */
struct mbox_msg_csr {
	/* MD:* @hdr - mailbox message header */
	struct mbox_msg_hdr hdr;
	/* MD:* @csr_info: csr info data strucutre */
	struct qdma_csr_info csr_info;
};

/* MD:*
 * struct mbox_msg_q_nitfy - queue add/del notify message
 */
struct mbox_msg_q_nitfy {
	/* MD:* @hdr - mailbox message header */
	struct mbox_msg_hdr hdr;
	/* MD:* @qid_hw: queue ID */
	uint16_t qid_hw;
	/* MD:* @q_type: type of q */
	enum qdma_dev_q_type q_type;
};

/* MD:*
 * @struct - mbox_msg_qctxt
 * @brief queue context mailbox message header
 */
struct mbox_msg_qctxt {
	/* MD:* @hdr: mailbox message header*/
	struct mbox_msg_hdr hdr;
	/* MD:* @qid_hw: queue ID */
	uint16_t qid_hw;
	/* MD:* @st: streaming mode */
	uint8_t st:1;
	/* MD:* @c2h: c2h direction */
	uint8_t c2h:1;
	/* MD:* @cmpt_ctxt_type: completion context type */
	enum mbox_cmpt_ctxt_type cmpt_ctxt_type:2;
	/* MD:* @rsvd: reserved */
	uint8_t rsvd:4;
	/* MD:* union compiled_message - complete hw configuration */
	union {
		/* MD:* @descq_conf: mailbox message for queue context write*/
		struct mbox_descq_conf descq_conf;
		/* MD:* @descq_ctxt: mailbox message for queue context read*/
		struct qdma_descq_context descq_ctxt;
	};
};

/* MD:*
 * @struct - mbox_intr_ctxt
 * @brief queue context mailbox message header
 */
struct mbox_intr_ctxt {
	/* MD:* @hdr: mailbox message header*/
	struct mbox_msg_hdr hdr;
	/* MD:* interrupt context mailbox message */
	struct mbox_msg_intr_ctxt ctxt;
};

/* MD:*
 * @struct - mbox_read_reg_list
 * @brief read register mailbox message header
 */
struct mbox_read_reg_list {
	/* MD:* @hdr: mailbox message header*/
	struct mbox_msg_hdr hdr;
	/* MD:* @group_num: reg group to read */
	uint16_t group_num;
	/* MD:* @num_regs: number of registers to read */
	uint16_t num_regs;
	/* MD:* @reg_list: register list */
	struct qdma_reg_data reg_list[QDMA_MAX_REGISTER_DUMP];
};

union qdma_mbox_txrx {
		/* MD:* mailbox message header*/
		struct mbox_msg_hdr hdr;
		/* MD:* hello mailbox message */
		struct mbox_msg_hello hello;
		/* MD:* fmap mailbox message */
		struct mbox_msg_fmap fmap;
		/* MD:* interrupt context mailbox message */
		struct mbox_intr_ctxt intr_ctxt;
		/* MD:* queue context mailbox message*/
		struct mbox_msg_qctxt qctxt;
		/* MD:* global csr mailbox message */
		struct mbox_msg_csr csr;
		/* MD:* acive q count */
		struct mbox_msg_active_qcnt qcnt;
		/* MD:* q add/del notify message */
		struct mbox_msg_q_nitfy q_notify;
		/* MD:* reg list mailbox message */
		struct mbox_read_reg_list reg_read_list;
		/* MD:* buffer to hold raw data between pf and vf */
		uint32_t raw[MBOX_MSG_REG_MAX];
};

/* MD:*
 * Returns the mailbox base address based on the VF/PF context.
 *
 * @param dev_hndl:   Device handle obtained from qdma_open or qdma_create_vf.
 * @param is_vf:      Flag indicating whether to access VF or PF mailbox.
 *
 * @return Mailbox base address (uint32_t).
 */
static inline uint32_t get_mbox_offset(void *dev_hndl, uint8_t is_vf)
{
	uint32_t mbox_base;
	struct qdma_hw_access *hw = NULL;

	// MD: Get the hardware access structure from the device handle.
	qdma_get_hw_access(dev_hndl, &hw);
	printk(KERN_INFO "get_mbox_offset: Got hw_access %p\n", hw);

	mbox_base = (is_vf) ?  // MD: Use VF mailbox base if VF context
		hw->mbox_base_vf : hw->mbox_base_pf;  // MD: Otherwise use PF mailbox base

	printk(KERN_INFO "get_mbox_offset: MBox Base Address is %x\n", mbox_base);
	return mbox_base;
}

/* MD:*
 * Clears the function's ack status in the PF mailbox.
 *
 * @param dev_hndl:   Device handle obtained from qdma_open or qdma_create_vf.
 * @param func_id:    Function ID to clear the ack status for.
 */
static inline void mbox_pf_hw_clear_func_ack(void *dev_hndl, uint16_t func_id)
{
	int idx = func_id / 32; /* MD: bitmask, uint32_t reg */
	int bit = func_id % 32;
	uint32_t mbox_base = get_mbox_offset(dev_hndl, 0);

	printk(KERN_INFO "mbox_pf_hw_clear_func_ack: Got MBox Base Address %x\n", mbox_base);
	// MD: Clear the function's ack status by writing a 1 to the corresponding bit in the ack register.
	qdma_reg_write(dev_hndl,
			mbox_base + MBOX_PF_ACK_BASE + idx * MBOX_PF_ACK_STEP,
			(1 << bit));
	printk(KERN_INFO "mbox_pf_hw_clear_func_ack: Cleared function %d ack status\n", func_id);
}

/* MD:*
 * Copies data from one buffer to another.
 *
 * @param to:         Destination buffer address.
 * @param from:       Source buffer address.
 * @param size:       Number of bytes to copy.
 */
static void qdma_mbox_memcpy(void *to, void *from, uint8_t size)
{
	uint8_t i;
	uint8_t *_to = (uint8_t *)to;
	uint8_t *_from = (uint8_t *)from;

	printk(KERN_INFO "qdma_mbox_memcpy: Copying %d bytes from 0x%x to 0x%x\n",
			size, _from, _to);

	for (i = 0; i < size; i++) {
		_to[i] = _from[i];
	}
}

/* MD:*
 * Sets a buffer with a specified value.
 *
 * @param to:         Destination buffer address.
 * @param val:        Value to set in the buffer.
 * @param size:       Number of bytes to set.
 */
static void qdma_mbox_memset(void *to, uint8_t val, uint8_t size)
{
	uint8_t i;
	uint8_t *_to = (uint8_t *)to;

	printk(KERN_INFO "qdma_mbox_memset: Setting %d bytes of buffer 0x%x to value %d\n",
			size, _to, val);

	for (i = 0; i < size; i++) {
		_to[i] = val;
	}
}

/* MD:*
 * Finds the ring index corresponding to a given ring size.
 *
 * @param dev_hndl:   Device handle obtained from qdma_open or qdma_create_vf.
 * @param ring_sz:    Ring size to find the index for.
 * @param rng_idx:    Pointer to store the found ring index.
 *
 * @return 0 on success, negative error code otherwise.
 */
static int get_ring_idx(void *dev_hndl, uint16_t ring_sz, uint16_t *rng_idx)
{
	uint32_t rng_sz[QDMA_GLOBAL_CSR_ARRAY_SZ] = { 0 };
	int i, rv;
	struct qdma_hw_access *hw = NULL;

	qdma_get_hw_access(dev_hndl, &hw);
	printk(KERN_INFO "get_ring_idx: Got hw_access %p\n", hw);

	rv = hw->qdma_global_csr_conf(dev_hndl, 0,
			QDMA_GLOBAL_CSR_ARRAY_SZ, rng_sz,
			QDMA_CSR_RING_SZ, QDMA_HW_ACCESS_READ);

	if (rv)
		return rv;
	printk(KERN_INFO "get_ring_idx: Got Ring Sizes %d\n", rv);

	for (i = 0; i < QDMA_GLOBAL_CSR_ARRAY_SZ; i++) {
		if (ring_sz == (rng_sz[i] - 1)) { // MD: Check if ring size matches
			*rng_idx = i;
			printk(KERN_INFO "get_ring_idx: Found Ring Index %d for size %d\n", i, ring_sz);
			return QDMA_SUCCESS;
		}
	}

	qdma_log_error("%s: Ring size not found, err:%d\n",
				   __func__, -QDMA_ERR_MBOX_INV_RINGSZ);

	printk(KERN_INFO "get_ring_idx: Failed to find ring index for %d\n", ring_sz);
	return -QDMA_ERR_MBOX_INV_RINGSZ;
}

/* MD:*
 * Finds the buffer index corresponding to a given buffer size.
 *
 * @param dev_hndl:   Device handle obtained from qdma_open or qdma_create_vf.
 * @param buf_sz:     Buffer size to find the index for.
 * @param buf_idx:    Pointer to store the found buffer index.
 *
 * @return 0 on success, negative error code otherwise.
 */
static int get_buf_idx(void *dev_hndl, uint16_t buf_sz, uint16_t *buf_idx)
{
	uint32_t c2h_buf_sz[QDMA_GLOBAL_CSR_ARRAY_SZ] = { 0 };
	int i, rv;
	struct qdma_hw_access *hw = NULL;

	qdma_get_hw_access(dev_hndl, &hw);

	rv = hw->qdma_global_csr_conf(dev_hndl, 0,
			QDMA_GLOBAL_CSR_ARRAY_SZ, c2h_buf_sz,
			QDMA_CSR_BUF_SZ, QDMA_HW_ACCESS_READ);
	if (rv)
		return rv;
	printk(KERN_INFO "get_buf_idx: Got Buffer Sizes %d\n", rv);

	for (i = 0; i < QDMA_GLOBAL_CSR_ARRAY_SZ; i++) {
		if (c2h_buf_sz[i] == buf_sz) { // MD: Check if buffer size matches
			*buf_idx = i;
			printk(KERN_INFO "get_buf_idx: Found Buffer Index %d for size %d\n", i, buf_sz);
			return QDMA_SUCCESS;
		}
	}

	qdma_log_error("%s: Buf index not found, err:%d\n",
				   __func__, -QDMA_ERR_MBOX_INV_BUFSZ);

	printk(KERN_INFO "get_buf_idx: Failed to find buffer index for %d\n", buf_sz);
	return -QDMA_ERR_MBOX_INV_BUFSZ;
}

/* MD:****************************************************************************/
/* MD:*
 * get_cntr_idx() - Get the index of a counter threshold value
 *
 * @dev_hndl: Device handle
 * @cntr_val: Counter value to find
 * @cntr_idx: Pointer to store the index of the counter value
 *
 * Return: QDMA_SUCCESS on success, negative error code on failure
 *****************************************************************************/
static int get_cntr_idx(void *dev_hndl, uint8_t cntr_val, uint8_t *cntr_idx)
{
    uint32_t cntr_th[QDMA_GLOBAL_CSR_ARRAY_SZ] = { 0 };
    int i, rv;
    struct qdma_hw_access *hw = NULL;

    // MD: Retrieve hardware access structure
    qdma_get_hw_access(dev_hndl, &hw);

    // MD: Read global CSR configuration for counter thresholds
    rv = hw->qdma_global_csr_conf(dev_hndl, 0,
            QDMA_GLOBAL_CSR_ARRAY_SZ, cntr_th,
            QDMA_CSR_CNT_TH, QDMA_HW_ACCESS_READ);

    if (rv)
        return rv;

    // MD: Find the index of the specified counter value
    for (i = 0; i < QDMA_GLOBAL_CSR_ARRAY_SZ; i++) {
        if (cntr_th[i] == cntr_val) {
            *cntr_idx = i;
            return QDMA_SUCCESS;
        }
    }

    // MD: Log an error if the counter value is not found
    qdma_log_error("%s: Counter val not found, err:%d\n",
                   __func__, -QDMA_ERR_MBOX_INV_CNTR_TH);
    return -QDMA_ERR_MBOX_INV_CNTR_TH;
}

/* MD:****************************************************************************/
/* MD:*
 * get_tmr_idx() - Get the index of a timer threshold value
 *
 * @dev_hndl: Device handle
 * @tmr_val: Timer value to find
 * @tmr_idx: Pointer to store the index of the timer value
 *
 * Return: QDMA_SUCCESS on success, negative error code on failure
 *****************************************************************************/
static int get_tmr_idx(void *dev_hndl, uint8_t tmr_val, uint8_t *tmr_idx)
{
    uint32_t tmr_th[QDMA_GLOBAL_CSR_ARRAY_SZ] = { 0 };
    int i, rv;
    struct qdma_hw_access *hw = NULL;

    // MD: Retrieve hardware access structure
    qdma_get_hw_access(dev_hndl, &hw);

    // MD: Read global CSR configuration for timer thresholds
    rv = hw->qdma_global_csr_conf(dev_hndl, 0,
            QDMA_GLOBAL_CSR_ARRAY_SZ, tmr_th,
            QDMA_CSR_TIMER_CNT, QDMA_HW_ACCESS_READ);
    if (rv)
        return rv;

    // MD: Find the index of the specified timer value
    for (i = 0; i < QDMA_GLOBAL_CSR_ARRAY_SZ; i++) {
        if (tmr_th[i] == tmr_val) {
            *tmr_idx = i;
            return QDMA_SUCCESS;
        }
    }

    // MD: Log an error if the timer value is not found
    qdma_log_error("%s: Timer val not found, err:%d\n",
                   __func__, -QDMA_ERR_MBOX_INV_TMR_TH);
    return -QDMA_ERR_MBOX_INV_TMR_TH;
}

/* MD:****************************************************************************/
/* MD:*
 * mbox_compose_sw_context() - Compose software context for mailbox message
 *
 * @dev_hndl: Device handle
 * @qctxt: Pointer to mailbox message queue context
 * @sw_ctxt: Pointer to software context to be composed
 *
 * Return: QDMA_SUCCESS on success, negative error code on failure
 *****************************************************************************/
static int mbox_compose_sw_context(void *dev_hndl,
                                   struct mbox_msg_qctxt *qctxt,
                                   struct qdma_descq_sw_ctxt *sw_ctxt)
{
    uint16_t rng_idx = 0;
    int rv = QDMA_SUCCESS;

    // MD: Validate input parameters
    if (!qctxt || !sw_ctxt) {
        qdma_log_error("%s: qctxt=%p sw_ctxt=%p, err:%d\n",
                        __func__, qctxt, sw_ctxt, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Get the ring index based on the queue context configuration
    rv = get_ring_idx(dev_hndl, qctxt->descq_conf.ringsz, &rng_idx);
    if (rv < 0) {
        qdma_log_error("%s: failed to get ring index, err:%d\n",
                        __func__, rv);
        return rv;
    }

    // MD: Compose the software context based on the queue context configuration
    sw_ctxt->vec = qctxt->descq_conf.intr_id;
    sw_ctxt->intr_aggr = qctxt->descq_conf.intr_aggr;
    sw_ctxt->ring_bs_addr = qctxt->descq_conf.ring_bs_addr;
    sw_ctxt->wbi_chk = qctxt->descq_conf.wbi_chk;
    sw_ctxt->wbi_intvl_en = qctxt->descq_conf.wbi_intvl_en;
    sw_ctxt->rngsz_idx = rng_idx;
    sw_ctxt->bypass = qctxt->descq_conf.en_bypass;
    sw_ctxt->wbk_en = qctxt->descq_conf.wbk_en;
    sw_ctxt->irq_en = qctxt->descq_conf.irq_en;
    sw_ctxt->is_mm = ~qctxt->st;
    sw_ctxt->mm_chn = 0;
    sw_ctxt->qen = 1;
    sw_ctxt->frcd_en = qctxt->descq_conf.forced_en;
    sw_ctxt->desc_sz = qctxt->descq_conf.desc_sz;
    sw_ctxt->fnc_id = qctxt->descq_conf.func_id;
    sw_ctxt->irq_arm = qctxt->descq_conf.irq_arm;

    // MD: Adjust context settings for specific conditions
    if (qctxt->st && qctxt->c2h) {
        sw_ctxt->irq_en = 0;
        sw_ctxt->irq_arm = 0;
        sw_ctxt->wbk_en = 0;
        sw_ctxt->wbi_chk = 0;
    }

    return QDMA_SUCCESS;
}

/* MD:****************************************************************************/
/* MD:*
 * mbox_compose_prefetch_context() - Compose prefetch context for mailbox message
 *
 * @dev_hndl: Device handle
 * @qctxt: Pointer to mailbox message queue context
 * @pfetch_ctxt: Pointer to prefetch context to be composed
 *
 * Return: QDMA_SUCCESS on success, negative error code on failure
 *****************************************************************************/
static int mbox_compose_prefetch_context(void *dev_hndl,
                                         struct mbox_msg_qctxt *qctxt,
                                         struct qdma_descq_prefetch_ctxt *pfetch_ctxt)
{
    uint16_t buf_idx = 0;
    int rv = QDMA_SUCCESS;

    // MD: Validate input parameters
    if (!qctxt || !pfetch_ctxt) {
        qdma_log_error("%s: qctxt=%p pfetch_ctxt=%p, err:%d\n",
                       __func__, qctxt, pfetch_ctxt, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Get the buffer index based on the queue context configuration
    rv = get_buf_idx(dev_hndl, qctxt->descq_conf.bufsz, &buf_idx);
    if (rv < 0) {
        qdma_log_error("%s: failed to get buf index, err:%d\n",
                       __func__, -QDMA_ERR_INV_PARAM);
        return rv;
    }

    // MD: Compose the prefetch context based on the queue context configuration
    pfetch_ctxt->valid = 1;
    pfetch_ctxt->bypass = qctxt->descq_conf.en_bypass_prefetch;
    pfetch_ctxt->bufsz_idx = buf_idx;
    pfetch_ctxt->pfch_en = qctxt->descq_conf.pfch_en;

    return QDMA_SUCCESS;
}

/* MD:*
 * mbox_compose_cmpt_context() - Compose completion context for mailbox message
 *
 * @dev_hndl: Device handle
 * @qctxt: Pointer to mailbox message queue context
 * @cmpt_ctxt: Pointer to completion context to be composed
 *
 * Return: QDMA_SUCCESS on success, negative error code on failure
 */
static int mbox_compose_cmpt_context(void *dev_hndl,
                                     struct mbox_msg_qctxt *qctxt,
                                     struct qdma_descq_cmpt_ctxt *cmpt_ctxt)
{
    uint16_t rng_idx = 0;
    uint8_t cntr_idx = 0, tmr_idx = 0;
    int rv = QDMA_SUCCESS;

    // MD: Validate input parameters
    if (!qctxt || !cmpt_ctxt) {
        qdma_log_error("%s: qctxt=%p cmpt_ctxt=%p, err:%d\n",
                       __func__, qctxt, cmpt_ctxt, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Get the counter index based on the queue context configuration
    rv = get_cntr_idx(dev_hndl, qctxt->descq_conf.cnt_thres, &cntr_idx);
    if (rv < 0)
        return rv;

    // MD: Get the timer index based on the queue context configuration
    rv = get_tmr_idx(dev_hndl, qctxt->descq_conf.timer_thres, &tmr_idx);
    if (rv < 0)
        return rv;

    // MD: Get the ring index based on the queue context configuration
    rv = get_ring_idx(dev_hndl, qctxt->descq_conf.cmpt_ringsz, &rng_idx);
    if (rv < 0)
        return rv;

    // MD: Compose the completion context based on the queue context configuration
    cmpt_ctxt->bs_addr = qctxt->descq_conf.cmpt_ring_bs_addr;
    cmpt_ctxt->en_stat_desc = qctxt->descq_conf.cmpl_stat_en;
    cmpt_ctxt->en_int = qctxt->descq_conf.cmpt_int_en;
    cmpt_ctxt->trig_mode = qctxt->descq_conf.triggermode;
    cmpt_ctxt->fnc_id = qctxt->descq_conf.func_id;
    cmpt_ctxt->timer_idx = tmr_idx;
    cmpt_ctxt->counter_idx = cntr_idx;
    cmpt_ctxt->color = 1;
    cmpt_ctxt->ringsz_idx = rng_idx;
    cmpt_ctxt->desc_sz = qctxt->descq_conf.cmpt_desc_sz;
    cmpt_ctxt->valid = 1;
    cmpt_ctxt->ovf_chk_dis = qctxt->descq_conf.dis_overflow_check;
    cmpt_ctxt->vec = qctxt->descq_conf.intr_id;
    cmpt_ctxt->int_aggr = qctxt->descq_conf.intr_aggr;

    return QDMA_SUCCESS;
}

/* MD:*
 * mbox_clear_queue_contexts() - Clear queue contexts
 *
 * @dev_hndl: Device handle
 * @dma_device_index: DMA device index
 * @func_id: Function ID
 * @qid_hw: Queue ID in hardware
 * @st: Stream type
 * @c2h: C2H direction flag
 * @cmpt_ctxt_type: Completion context type
 *
 * Return: QDMA_SUCCESS on success, negative error code on failure
 */
static int mbox_clear_queue_contexts(void *dev_hndl, uint8_t dma_device_index,
                                     uint16_t func_id, uint16_t qid_hw, uint8_t st,
                                     uint8_t c2h, enum mbox_cmpt_ctxt_type cmpt_ctxt_type)
{
    int rv;
    int qbase;
    uint32_t qmax;
    enum qdma_dev_q_range q_range;
    struct qdma_hw_access *hw = NULL;

    // MD: Retrieve hardware access structure
    qdma_get_hw_access(dev_hndl, &hw);

    if (cmpt_ctxt_type == QDMA_MBOX_CMPT_CTXT_ONLY) {
        // MD: Clear only the completion context
        rv = hw->qdma_cmpt_ctx_conf(dev_hndl, qid_hw, NULL, QDMA_HW_ACCESS_CLEAR);
        if (rv < 0) {
            qdma_log_error("%s: clear cmpt ctxt, err:%d\n", __func__, rv);
            return rv;
        }
    } else {
        // MD: Get queue information
        rv = qdma_dev_qinfo_get(dma_device_index, func_id, &qbase, &qmax);
        if (rv < 0) {
            qdma_log_error("%s: failed to get qinfo, err:%d\n", __func__, rv);
            return rv;
        }

        // MD: Check if the queue is in range
        q_range = qdma_dev_is_queue_in_range(dma_device_index, func_id, qid_hw);
        if (q_range != QDMA_DEV_Q_IN_RANGE) {
            qdma_log_error("%s: q_range invalid, err:%d\n", __func__, rv);
            return rv;
        }

        // MD: Clear software context
        rv = hw->qdma_sw_ctx_conf(dev_hndl, c2h, qid_hw, NULL, QDMA_HW_ACCESS_CLEAR);
        if (rv < 0) {
            qdma_log_error("%s: clear sw_ctxt, err:%d\n", __func__, rv);
            return rv;
        }

        // MD: Clear hardware context
        rv = hw->qdma_hw_ctx_conf(dev_hndl, c2h, qid_hw, NULL, QDMA_HW_ACCESS_CLEAR);
        if (rv < 0) {
            qdma_log_error("%s: clear hw_ctxt, err:%d\n", __func__, rv);
            return rv;
        }

        // MD: Clear credit context
        rv = hw->qdma_credit_ctx_conf(dev_hndl, c2h, qid_hw, NULL, QDMA_HW_ACCESS_CLEAR);
        if (rv < 0) {
            qdma_log_error("%s: clear cr_ctxt, err:%d\n", __func__, rv);
            return rv;
        }

        // MD: Clear prefetch context if applicable
        if (st && c2h) {
            rv = hw->qdma_pfetch_ctx_conf(dev_hndl, qid_hw, NULL, QDMA_HW_ACCESS_CLEAR);
            if (rv < 0) {
                qdma_log_error("%s: clear pfetch ctxt, err:%d\n", __func__, rv);
                return rv;
            }
        }

        // MD: Clear completion context if applicable
        if ((cmpt_ctxt_type == QDMA_MBOX_CMPT_WITH_MM) ||
            (cmpt_ctxt_type == QDMA_MBOX_CMPT_WITH_ST)) {
            rv = hw->qdma_cmpt_ctx_conf(dev_hndl, qid_hw, NULL, QDMA_HW_ACCESS_CLEAR);
            if (rv < 0) {
                qdma_log_error("%s: clear cmpt ctxt, err:%d\n", __func__, rv);
                return rv;
            }
        }
    }

    return QDMA_SUCCESS;
}

/* MD:*
 * mbox_invalidate_queue_contexts() - Invalidate queue contexts
 *
 * @dev_hndl: Device handle
 * @dma_device_index: DMA device index
 * @func_id: Function ID
 * @qid_hw: Queue ID in hardware
 * @st: Stream type
 * @c2h: C2H (Completion to Host) flag
 * @cmpt_ctxt_type: Completion context type
 *
 * Return: QDMA_SUCCESS on success, negative error code on failure
 */
static int mbox_invalidate_queue_contexts(void *dev_hndl,
        uint8_t dma_device_index, uint16_t func_id,
        uint16_t qid_hw, uint8_t st,
        uint8_t c2h, enum mbox_cmpt_ctxt_type cmpt_ctxt_type)
{
    int rv;
    int qbase;
    uint32_t qmax;
    enum qdma_dev_q_range q_range;
    struct qdma_hw_access *hw = NULL;

    // MD: Get hardware access structure
    qdma_get_hw_access(dev_hndl, &hw);

    if (cmpt_ctxt_type == QDMA_MBOX_CMPT_CTXT_ONLY) {
        // MD: Invalidate completion context only
        rv = hw->qdma_cmpt_ctx_conf(dev_hndl, qid_hw, NULL,
                                    QDMA_HW_ACCESS_INVALIDATE);
        if (rv < 0) {
            qdma_log_error("%s: inv cmpt ctxt, err:%d\n", __func__, rv);
            return rv;
        }
    } else {
        // MD: Get queue information
        rv = qdma_dev_qinfo_get(dma_device_index, func_id, &qbase, &qmax);
        if (rv < 0) {
            qdma_log_error("%s: failed to get qinfo, err:%d\n", __func__, rv);
            return rv;
        }

        // MD: Check if queue is in range
        q_range = qdma_dev_is_queue_in_range(dma_device_index, func_id, qid_hw);
        if (q_range != QDMA_DEV_Q_IN_RANGE) {
            qdma_log_error("%s: Invalid qrange, err:%d\n", __func__, rv);
            return rv;
        }

        // MD: Invalidate software context
        rv = hw->qdma_sw_ctx_conf(dev_hndl, c2h, qid_hw, NULL, QDMA_HW_ACCESS_INVALIDATE);
        if (rv < 0) {
            qdma_log_error("%s: inv sw ctxt, err:%d\n", __func__, rv);
            return rv;
        }

        // MD: Invalidate hardware context
        rv = hw->qdma_hw_ctx_conf(dev_hndl, c2h, qid_hw, NULL, QDMA_HW_ACCESS_INVALIDATE);
        if (rv < 0) {
            qdma_log_error("%s: clear hw_ctxt, err:%d\n", __func__, rv);
            return rv;
        }

        // MD: Invalidate credit context
        rv = hw->qdma_credit_ctx_conf(dev_hndl, c2h, qid_hw, NULL, QDMA_HW_ACCESS_INVALIDATE);
        if (rv < 0) {
            qdma_log_error("%s: clear cr_ctxt, err:%d\n", __func__, rv);
            return rv;
        }

        // MD: Invalidate prefetch context if applicable
        if (st && c2h) {
            rv = hw->qdma_pfetch_ctx_conf(dev_hndl, qid_hw, NULL, QDMA_HW_ACCESS_INVALIDATE);
            if (rv < 0) {
                qdma_log_error("%s: inv pfetch ctxt, err:%d\n", __func__, rv);
                return rv;
            }
        }

        // MD: Invalidate completion context if applicable
        if ((cmpt_ctxt_type == QDMA_MBOX_CMPT_WITH_MM) || (cmpt_ctxt_type == QDMA_MBOX_CMPT_WITH_ST)) {
            rv = hw->qdma_cmpt_ctx_conf(dev_hndl, qid_hw, NULL, QDMA_HW_ACCESS_INVALIDATE);
            if (rv < 0) {
                qdma_log_error("%s: inv cmpt ctxt, err:%d\n", __func__, rv);
                return rv;
            }
        }
    }

    return QDMA_SUCCESS;
}

/* MD:*
 * mbox_write_queue_contexts() - Write queue contexts
 *
 * @dev_hndl: Device handle
 * @dma_device_index: DMA device index
 * @qctxt: Pointer to the mailbox message queue context
 *
 * Return: QDMA_SUCCESS on success, negative error code on failure
 */
static int mbox_write_queue_contexts(void *dev_hndl, uint8_t dma_device_index,
                                     struct mbox_msg_qctxt *qctxt)
{
    int rv;
    int qbase;
    uint32_t qmax;
    enum qdma_dev_q_range q_range;
    struct qdma_descq_context descq_ctxt;
    uint16_t qid_hw = qctxt->qid_hw;
    struct qdma_hw_access *hw = NULL;

    // MD: Get hardware access structure
    qdma_get_hw_access(dev_hndl, &hw);

    // MD: Get queue information
    rv = qdma_dev_qinfo_get(dma_device_index, qctxt->descq_conf.func_id, &qbase, &qmax);
    if (rv < 0)
        return rv;

    // MD: Check if queue is in range
    q_range = qdma_dev_is_queue_in_range(dma_device_index, qctxt->descq_conf.func_id, qctxt->qid_hw);
    if (q_range != QDMA_DEV_Q_IN_RANGE) {
        qdma_log_error("%s: Invalid qrange, err:%d\n", __func__, rv);
        return rv;
    }

    // MD: Initialize descriptor queue context
    qdma_mbox_memset(&descq_ctxt, 0, sizeof(struct qdma_descq_context));

    if (qctxt->cmpt_ctxt_type == QDMA_MBOX_CMPT_CTXT_ONLY) {
        // MD: Compose and write completion context
        rv = mbox_compose_cmpt_context(dev_hndl, qctxt, &descq_ctxt.cmpt_ctxt);
        if (rv < 0)
            return rv;

        rv = hw->qdma_cmpt_ctx_conf(dev_hndl, qid_hw, NULL, QDMA_HW_ACCESS_CLEAR);
        if (rv < 0) {
            qdma_log_error("%s: clear cmpt ctxt, err:%d\n", __func__, rv);
            return rv;
        }

        rv = hw->qdma_cmpt_ctx_conf(dev_hndl, qid_hw, &descq_ctxt.cmpt_ctxt, QDMA_HW_ACCESS_WRITE);
        if (rv < 0) {
            qdma_log_error("%s: write cmpt ctxt, err:%d\n", __func__, rv);
            return rv;
        }

    } else {
        // MD: Compose and write software context
        rv = mbox_compose_sw_context(dev_hndl, qctxt, &descq_ctxt.sw_ctxt);
        if (rv < 0)
            return rv;

        // MD: Compose and write prefetch context if applicable
        if (qctxt->st && qctxt->c2h) {
            rv = mbox_compose_prefetch_context(dev_hndl, qctxt, &descq_ctxt.pfetch_ctxt);
            if (rv < 0)
                return rv;
        }

        // MD: Compose and write completion context if applicable
        if ((qctxt->cmpt_ctxt_type == QDMA_MBOX_CMPT_WITH_MM) || (qctxt->cmpt_ctxt_type == QDMA_MBOX_CMPT_WITH_ST)) {
            rv = mbox_compose_cmpt_context(dev_hndl, qctxt, &descq_ctxt.cmpt_ctxt);
            if (rv < 0)
                return rv;
        }

        // MD: Clear existing queue contexts
        rv = mbox_clear_queue_contexts(dev_hndl, dma_device_index, qctxt->descq_conf.func_id, qctxt->qid_hw, qctxt->st, qctxt->c2h, qctxt->cmpt_ctxt_type);
        if (rv < 0)
            return rv;

        // MD: Write software context
        rv = hw->qdma_sw_ctx_conf(dev_hndl, qctxt->c2h, qid_hw, &descq_ctxt.sw_ctxt, QDMA_HW_ACCESS_WRITE);
        if (rv < 0) {
            qdma_log_error("%s: write sw ctxt, err:%d\n", __func__, rv);
            return rv;
        }

        // MD: Write prefetch context if applicable
        if (qctxt->st && qctxt->c2h) {
            rv = hw->qdma_pfetch_ctx_conf(dev_hndl, qid_hw, &descq_ctxt.pfetch_ctxt, QDMA_HW_ACCESS_WRITE);
            if (rv < 0) {
                qdma_log_error("%s:write pfetch ctxt, err:%d\n", __func__, rv);
                return rv;
            }
        }

        // MD: Write completion context if applicable
        if ((qctxt->cmpt_ctxt_type == QDMA_MBOX_CMPT_WITH_MM) || (qctxt->cmpt_ctxt_type == QDMA_MBOX_CMPT_WITH_ST)) {
            rv = hw->qdma_cmpt_ctx_conf(dev_hndl, qid_hw, &descq_ctxt.cmpt_ctxt, QDMA_HW_ACCESS_WRITE);
            if (rv < 0) {
                qdma_log_error("%s: write cmpt ctxt, err:%d\n", __func__, rv);
                return rv;
            }
        }
    }
    return QDMA_SUCCESS;
}

/* MD:*
 * mbox_read_queue_contexts() - Read queue contexts from a QDMA device
 *
 * @dev_hndl: Device handle
 * @func_id: Function ID
 * @qid_hw: Hardware Queue ID
 * @st: Stream type
 * @c2h: Direction flag (C2H)
 * @cmpt_ctxt_type: Completion context type
 * @ctxt: Pointer to the structure to store the queue contexts
 *
 * Return: QDMA_SUCCESS on success, negative error code on failure
 */
static int mbox_read_queue_contexts(void *dev_hndl, uint16_t func_id,
        uint16_t qid_hw, uint8_t st, uint8_t c2h,
        enum mbox_cmpt_ctxt_type cmpt_ctxt_type,
        struct qdma_descq_context *ctxt)
{
    int rv;
    struct qdma_hw_access *hw = NULL;

    // MD: Retrieve hardware access functions for the device
    qdma_get_hw_access(dev_hndl, &hw);

    // MD: Read the software context
    rv = hw->qdma_sw_ctx_conf(dev_hndl, c2h, qid_hw, &ctxt->sw_ctxt,
                              QDMA_HW_ACCESS_READ);
    if (rv < 0) {
        qdma_log_error("%s: read sw ctxt, err:%d\n", __func__, rv);
        return rv;
    }

    // MD: Read the hardware context
    rv = hw->qdma_hw_ctx_conf(dev_hndl, c2h, qid_hw, &ctxt->hw_ctxt,
                              QDMA_HW_ACCESS_READ);
    if (rv < 0) {
        qdma_log_error("%s: read hw ctxt, err:%d\n", __func__, rv);
        return rv;
    }

    // MD: Read the credit context
    rv = hw->qdma_credit_ctx_conf(dev_hndl, c2h, qid_hw, &ctxt->cr_ctxt,
                                  QDMA_HW_ACCESS_READ);
    if (rv < 0) {
        qdma_log_error("%s: read credit ctxt, err:%d\n", __func__, rv);
        return rv;
    }

    // MD: Read the function map context
    rv = hw->qdma_fmap_conf(dev_hndl, func_id, &ctxt->fmap,
                            QDMA_HW_ACCESS_READ);
    if (rv < 0) {
        qdma_log_error("%s: read fmap ctxt, err:%d\n", __func__, rv);
        return rv;
    }

    // MD: If stream type and direction are set, read the prefetch context
    if (st && c2h) {
        rv = hw->qdma_pfetch_ctx_conf(dev_hndl, qid_hw, &ctxt->pfetch_ctxt,
                                      QDMA_HW_ACCESS_READ);
        if (rv < 0) {
            qdma_log_error("%s: read pfetch ctxt, err:%d\n", __func__, rv);
            return rv;
        }
    }

    // MD: If completion context type is set, read the completion context
    if ((cmpt_ctxt_type == QDMA_MBOX_CMPT_WITH_MM) ||
        (cmpt_ctxt_type == QDMA_MBOX_CMPT_WITH_ST)) {
        rv = hw->qdma_cmpt_ctx_conf(dev_hndl, qid_hw, &ctxt->cmpt_ctxt,
                                    QDMA_HW_ACCESS_READ);
        if (rv < 0) {
            qdma_log_error("%s: read cmpt ctxt, err:%d\n", __func__, rv);
            return rv;
        }
    }

    return QDMA_SUCCESS;
}

/* MD:*
 * qdma_mbox_pf_rcv_msg_handler() - Handle received mailbox messages
 *
 * @dev_hndl: Device handle
 * @dma_device_index: DMA device index
 * @func_id: Function ID
 * @rcv_msg: Received message
 * @resp_msg: Response message
 *
 * Return: Status code indicating success or failure
 */
int qdma_mbox_pf_rcv_msg_handler(void *dev_hndl, uint8_t dma_device_index,
                                 uint16_t func_id, uint32_t *rcv_msg,
                                 uint32_t *resp_msg)
{
    union qdma_mbox_txrx *rcv = (union qdma_mbox_txrx *)rcv_msg;
    union qdma_mbox_txrx *resp = (union qdma_mbox_txrx *)resp_msg;
    struct mbox_msg_hdr *hdr = &rcv->hdr;
    struct qdma_hw_access *hw = NULL;
    int rv = QDMA_SUCCESS;
    int ret = 0;

    // MD: Check for a valid received message
    if (!rcv) {
        qdma_log_error("%s: rcv_msg=%p failure:%d\n",
                       __func__, rcv, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Get hardware access structure
    qdma_get_hw_access(dev_hndl, &hw);

    // MD: Handle different mailbox operations based on the operation code
    switch (rcv->hdr.op) {
    case MBOX_OP_VF_BYE:
    {
        struct qdma_fmap_cfg fmap;
        fmap.qbase = 0;
        fmap.qmax = 0;

        // MD: Configure the function map and destroy the device entry
        rv = hw->qdma_fmap_conf(dev_hndl, hdr->src_func_id, &fmap, QDMA_HW_ACCESS_WRITE);
        qdma_dev_entry_destroy(dma_device_index, hdr->src_func_id);

        ret = QDMA_MBOX_VF_OFFLINE;
    }
    break;

    case MBOX_OP_PF_RESET_VF_BYE:
    {
        struct qdma_fmap_cfg fmap;
        fmap.qbase = 0;
        fmap.qmax = 0;

        // MD: Configure the function map and destroy the device entry
        rv = hw->qdma_fmap_conf(dev_hndl, hdr->src_func_id, &fmap, QDMA_HW_ACCESS_WRITE);
        qdma_dev_entry_destroy(dma_device_index, hdr->src_func_id);

        ret = QDMA_MBOX_VF_RESET_BYE;
    }
    break;

    case MBOX_OP_HELLO:
    {
        struct mbox_msg_fmap *fmap = &rcv->fmap;
        struct qdma_fmap_cfg fmap_cfg;
        struct mbox_msg_hello *rsp_hello = &resp->hello;

        // MD: Get queue information and create device entry if necessary
        rv = qdma_dev_qinfo_get(dma_device_index, hdr->src_func_id, &fmap->qbase, &fmap->qmax);
        if (rv < 0)
            rv = qdma_dev_entry_create(dma_device_index, hdr->src_func_id);

        if (!rv) {
            rsp_hello->qbase = fmap->qbase;
            rsp_hello->qmax = fmap->qmax;
            rsp_hello->dma_device_index = dma_device_index;
            hw->qdma_get_device_attributes(dev_hndl, &rsp_hello->dev_cap);
        }

        // MD: Clear the function map configuration
        qdma_mbox_memset(&fmap_cfg, 0, sizeof(struct qdma_fmap_cfg));
        hw->qdma_fmap_conf(dev_hndl, hdr->src_func_id, &fmap_cfg, QDMA_HW_ACCESS_WRITE);

        ret = QDMA_MBOX_VF_ONLINE;
    }
    break;

    case MBOX_OP_FMAP:
    {
        struct mbox_msg_fmap *fmap = &rcv->fmap;
        struct qdma_fmap_cfg fmap_cfg;

        fmap_cfg.qbase = fmap->qbase;
        fmap_cfg.qmax = fmap->qmax;

        // MD: Write the function map configuration
        rv = hw->qdma_fmap_conf(dev_hndl, hdr->src_func_id, &fmap_cfg, QDMA_HW_ACCESS_WRITE);
        if (rv < 0) {
            qdma_log_error("%s: failed to write fmap, err:%d\n", __func__, rv);
            return rv;
        }
    }
    break;

    case MBOX_OP_CSR:
    {
        struct mbox_msg_csr *rsp_csr = &resp->csr;
        struct qdma_dev_attributes dev_cap;
        uint32_t ringsz[QDMA_GLOBAL_CSR_ARRAY_SZ] = {0};
        uint32_t bufsz[QDMA_GLOBAL_CSR_ARRAY_SZ] = {0};
        uint32_t tmr_th[QDMA_GLOBAL_CSR_ARRAY_SZ] = {0};
        uint32_t cntr_th[QDMA_GLOBAL_CSR_ARRAY_SZ] = {0};
        int i;

        // MD: Read global CSR configurations
        rv = hw->qdma_global_csr_conf(dev_hndl, 0, QDMA_GLOBAL_CSR_ARRAY_SZ, ringsz, QDMA_CSR_RING_SZ, QDMA_HW_ACCESS_READ);
        if (rv < 0)
            goto exit_func;

        hw->qdma_get_device_attributes(dev_hndl, &dev_cap);

        if (dev_cap.st_en) {
            rv = hw->qdma_global_csr_conf(dev_hndl, 0, QDMA_GLOBAL_CSR_ARRAY_SZ, bufsz, QDMA_CSR_BUF_SZ, QDMA_HW_ACCESS_READ);
            if (rv < 0 && (rv != -QDMA_ERR_HWACC_FEATURE_NOT_SUPPORTED))
                goto exit_func;
        }

        if (dev_cap.st_en || dev_cap.mm_cmpt_en) {
            rv = hw->qdma_global_csr_conf(dev_hndl, 0, QDMA_GLOBAL_CSR_ARRAY_SZ, tmr_th, QDMA_CSR_TIMER_CNT, QDMA_HW_ACCESS_READ);
            if (rv < 0 && (rv != -QDMA_ERR_HWACC_FEATURE_NOT_SUPPORTED))
                goto exit_func;

            rv = hw->qdma_global_csr_conf(dev_hndl, 0, QDMA_GLOBAL_CSR_ARRAY_SZ, cntr_th, QDMA_CSR_CNT_TH, QDMA_HW_ACCESS_READ);
            if (rv < 0 && (rv != -QDMA_ERR_HWACC_FEATURE_NOT_SUPPORTED))
                goto exit_func;
        }

        // MD: Populate response CSR information
        for (i = 0; i < QDMA_GLOBAL_CSR_ARRAY_SZ; i++) {
            rsp_csr->csr_info.ringsz[i] = ringsz[i] & 0xFFFF;
            if (!rv) {
                rsp_csr->csr_info.bufsz[i] = bufsz[i] & 0xFFFF;
                rsp_csr->csr_info.timer_cnt[i] = tmr_th[i] & 0xFF;
                rsp_csr->csr_info.cnt_thres[i] = cntr_th[i] & 0xFF;
            }
        }

        if (rv == -QDMA_ERR_HWACC_FEATURE_NOT_SUPPORTED)
            rv = QDMA_SUCCESS;
    }
    break;

    case MBOX_OP_QREQ:
    {
        struct mbox_msg_fmap *fmap = &rcv->fmap;

        // MD: Update device queue information
        rv = qdma_dev_update(dma_device_index, hdr->src_func_id, fmap->qmax, &fmap->qbase);
        if (rv == 0)
            rv = qdma_dev_qinfo_get(dma_device_index, hdr->src_func_id, &resp->fmap.qbase, &resp->fmap.qmax);
        if (rv < 0)
            rv = -QDMA_ERR_MBOX_NUM_QUEUES;
        else {
            struct qdma_fmap_cfg fmap_cfg;
            qdma_mbox_memset(&fmap_cfg, 0, sizeof(struct qdma_fmap_cfg));
            hw->qdma_fmap_conf(dev_hndl, hdr->src_func_id, &fmap_cfg, QDMA_HW_ACCESS_WRITE);
        }
    }
    break;

    case MBOX_OP_QNOTIFY_ADD:
    {
        struct mbox_msg_q_nitfy *q_notify = &rcv->q_notify;
        enum qdma_dev_q_range q_range;

        // MD: Check if the queue is in range and increment active queue count
        q_range = qdma_dev_is_queue_in_range(dma_device_index, q_notify->hdr.src_func_id, q_notify->qid_hw);
        if (q_range != QDMA_DEV_Q_IN_RANGE)
            rv = -QDMA_ERR_MBOX_INV_QID;
        else
            rv = qdma_dev_increment_active_queue(dma_device_index, q_notify->hdr.src_func_id, q_notify->q_type);
    }
    break;

    case MBOX_OP_QNOTIFY_DEL:
    {
        struct mbox_msg_q_nitfy *q_notify = &rcv->q_notify;
        enum qdma_dev_q_range q_range;

        // MD: Check if the queue is in range and decrement active queue count
        q_range = qdma_dev_is_queue_in_range(dma_device_index, q_notify->hdr.src_func_id, q_notify->qid_hw);
        if (q_range != QDMA_DEV_Q_IN_RANGE)
            rv = -QDMA_ERR_MBOX_INV_QID;
        else
            rv = qdma_dev_decrement_active_queue(dma_device_index, q_notify->hdr.src_func_id, q_notify->q_type);
    }
    break;

    case MBOX_OP_GET_QACTIVE_CNT:
    {
        // MD: Get active queue counts for different queue types
        rv = qdma_get_device_active_queue_count(dma_device_index, rcv->hdr.src_func_id, QDMA_DEV_Q_TYPE_H2C);
        resp->qcnt.h2c_queues = rv;

        rv = qdma_get_device_active_queue_count(dma_device_index, rcv->hdr.src_func_id, QDMA_DEV_Q_TYPE_C2H);
        resp->qcnt.c2h_queues = rv;

        rv = qdma_get_device_active_queue_count(dma_device_index, rcv->hdr.src_func_id, QDMA_DEV_Q_TYPE_CMPT);
        resp->qcnt.cmpt_queues = rv;
    }
    break;

    case MBOX_OP_INTR_CTXT_WRT:
    {
        struct mbox_msg_intr_ctxt *ictxt = &rcv->intr_ctxt.ctxt;
        struct qdma_indirect_intr_ctxt *ctxt;
        uint8_t i;
        uint32_t ring_index;

        // MD: Write interrupt context for each ring
        for (i = 0; i < ictxt->num_rings; i++) {
            ring_index = ictxt->ring_index_list[i];
            ctxt = &ictxt->ictxt[i];

            rv = hw->qdma_indirect_intr_ctx_conf(dev_hndl, ring_index, NULL, QDMA_HW_ACCESS_CLEAR);
            if (rv < 0)
                resp->hdr.status = rv;

            rv = hw->qdma_indirect_intr_ctx_conf(dev_hndl, ring_index, ctxt, QDMA_HW_ACCESS_WRITE);
            if (rv < 0)
                resp->hdr.status = rv;
        }
    }
    break;

    case MBOX_OP_INTR_CTXT_RD:
    {
        struct mbox_msg_intr_ctxt *rcv_ictxt = &rcv->intr_ctxt.ctxt;
        struct mbox_msg_intr_ctxt *rsp_ictxt = &resp->intr_ctxt.ctxt;
        uint8_t i;
        uint32_t ring_index;

        // MD: Read interrupt context for each ring
        for (i = 0; i < rcv_ictxt->num_rings; i++) {
            ring_index = rcv_ictxt->ring_index_list[i];
            rv = hw->qdma_indirect_intr_ctx_conf(dev_hndl, ring_index, &rsp_ictxt->ictxt[i], QDMA_HW_ACCESS_READ);
            if (rv < 0)
                resp->hdr.status = rv;
        }
    }
    break;

    case MBOX_OP_INTR_CTXT_CLR:
    {
        int i;
        struct mbox_msg_intr_ctxt *ictxt = &rcv->intr_ctxt.ctxt;

        // MD: Clear interrupt context for each ring
        for (i = 0; i < ictxt->num_rings; i++) {
            rv = hw->qdma_indirect_intr_ctx_conf(dev_hndl, ictxt->ring_index_list[i], NULL, QDMA_HW_ACCESS_CLEAR);
            if (rv < 0)
                resp->hdr.status = rv;
        }
    }
    break;

    case MBOX_OP_INTR_CTXT_INV:
    {
        struct mbox_msg_intr_ctxt *ictxt = &rcv->intr_ctxt.ctxt;
        int i;

        // MD: Invalidate interrupt context for each ring
        for (i = 0; i < ictxt->num_rings; i++) {
            rv = hw->qdma_indirect_intr_ctx_conf(dev_hndl, ictxt->ring_index_list[i], NULL, QDMA_HW_ACCESS_INVALIDATE);
            if (rv < 0)
                resp->hdr.status = rv;
        }
    }
    break;

    case MBOX_OP_QCTXT_INV:
    {
        struct mbox_msg_qctxt *qctxt = &rcv->qctxt;

        // MD: Invalidate queue contexts
        rv = mbox_invalidate_queue_contexts(dev_hndl, dma_device_index, hdr->src_func_id, qctxt->qid_hw, qctxt->st, qctxt->c2h, qctxt->cmpt_ctxt_type);
    }
    break;

    case MBOX_OP_QCTXT_CLR:
    {
        struct mbox_msg_qctxt *qctxt = &rcv->qctxt;

        // MD: Clear queue contexts
        rv = mbox_clear_queue_contexts(dev_hndl, dma_device_index, hdr->src_func_id, qctxt->qid_hw, qctxt->st, qctxt->c2h, qctxt->cmpt_ctxt_type);
    }
    break;

    case MBOX_OP_QCTXT_RD:
    {
        struct mbox_msg_qctxt *qctxt = &rcv->qctxt;

        // MD: Read queue contexts
        rv = mbox_read_queue_contexts(dev_hndl, hdr->src_func_id, qctxt->qid_hw, qctxt->st, qctxt->c2h, qctxt->cmpt_ctxt_type, &resp->qctxt.descq_ctxt);
    }
    break;

    case MBOX_OP_QCTXT_WRT:
    {
        struct mbox_msg_qctxt *qctxt = &rcv->qctxt;

        // MD: Write queue contexts
        qctxt->descq_conf.func_id = hdr->src_func_id;
        rv = mbox_write_queue_contexts(dev_hndl, dma_device_index, qctxt);
    }
    break;

    case MBOX_OP_RESET_PREPARE_RESP:
        return QDMA_MBOX_VF_RESET;

    case MBOX_OP_RESET_DONE_RESP:
        return QDMA_MBOX_PF_RESET_DONE;

    case MBOX_OP_REG_LIST_READ:
    {
        struct mbox_read_reg_list *rcv_read_reg_list = &rcv->reg_read_list;
        struct mbox_read_reg_list *rsp_read_reg_list = &resp->reg_read_list;

        // MD: Read register list
        rv = hw->qdma_read_reg_list((void *)dev_hndl, 1, rcv_read_reg_list->group_num, &rsp_read_reg_list->num_regs, rsp_read_reg_list->reg_list);

        if (rv < 0 || rsp_read_reg_list->num_regs == 0) {
            rv = -QDMA_ERR_MBOX_REG_READ_FAILED;
            goto exit_func;
        }
    }
    break;

    case MBOX_OP_PF_BYE_RESP:
        return QDMA_MBOX_PF_BYE;

    default:
        // MD: Log an error for invalid operation
        qdma_log_error("%s: op=%d invalid, err:%d\n", __func__, rcv->hdr.op, -QDMA_ERR_MBOX_INV_MSG);
        return -QDMA_ERR_MBOX_INV_MSG;
    break;
    }

exit_func:
    // MD: Prepare response message header
    resp->hdr.op = rcv->hdr.op + MBOX_MSG_OP_RSP_OFFSET;
    resp->hdr.dst_func_id = rcv->hdr.src_func_id;
    resp->hdr.src_func_id = func_id;
    resp->hdr.status = rv;

    return ret;
}

/* MD:*
 * qmda_mbox_compose_vf_online() - Compose a mailbox message for VF online
 *
 * @func_id: Function ID
 * @qmax: Maximum queue value
 * @qbase: Pointer to the base queue value
 * @raw_data: Pointer to the raw data buffer
 *
 * Return: QDMA_SUCCESS on success, negative error code on failure
 */
int qmda_mbox_compose_vf_online(uint16_t func_id,
                                uint16_t qmax, int *qbase, uint32_t *raw_data)
{
    union qdma_mbox_txrx *msg = (union qdma_mbox_txrx *)raw_data;

    // MD: Check for a valid raw_data pointer
    if (!raw_data) {
        qdma_log_error("%s: raw_data=%p, err:%d\n",
                        __func__, raw_data,
                        -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Initialize the raw data buffer to zero
    qdma_mbox_memset(raw_data, 0, sizeof(union qdma_mbox_txrx));

    // MD: Set the operation type and function ID
    msg->hdr.op = MBOX_OP_HELLO;
    msg->hdr.src_func_id = func_id;

    // MD: Set the queue base and maximum values
    msg->fmap.qbase = (uint32_t)*qbase;
    msg->fmap.qmax = qmax;

    return QDMA_SUCCESS;
}

/* MD:*
 * qdma_mbox_compose_vf_offline() - Compose a mailbox message for VF offline
 *
 * @func_id: Function ID
 * @raw_data: Pointer to the raw data buffer
 *
 * Return: QDMA_SUCCESS on success, negative error code on failure
 */
int qdma_mbox_compose_vf_offline(uint16_t func_id,
                                 uint32_t *raw_data)
{
    union qdma_mbox_txrx *msg = (union qdma_mbox_txrx *)raw_data;

    // MD: Check for a valid raw_data pointer
    if (!raw_data) {
        qdma_log_error("%s: raw_data=%p, err:%d\n",
                        __func__, raw_data,
                        -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Initialize the raw data buffer to zero
    qdma_mbox_memset(raw_data, 0, sizeof(union qdma_mbox_txrx));

    // MD: Set the operation type and function ID
    msg->hdr.op = MBOX_OP_VF_BYE;
    msg->hdr.src_func_id = func_id;

    return QDMA_SUCCESS;
}

/* MD:*
 * qdma_mbox_compose_vf_reset_offline() - Compose a mailbox message for VF reset offline
 *
 * @func_id: Function ID
 * @raw_data: Pointer to the raw data buffer
 *
 * Return: QDMA_SUCCESS on success, negative error code on failure
 */
int qdma_mbox_compose_vf_reset_offline(uint16_t func_id,
                                       uint32_t *raw_data)
{
    union qdma_mbox_txrx *msg = (union qdma_mbox_txrx *)raw_data;

    // MD: Check for a valid raw_data pointer
    if (!raw_data) {
        qdma_log_error("%s: raw_data=%p, err:%d\n",
                        __func__, raw_data,
                        -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Initialize the raw data buffer to zero
    qdma_mbox_memset(raw_data, 0, sizeof(union qdma_mbox_txrx));

    // MD: Set the operation type and function ID
    msg->hdr.op = MBOX_OP_PF_RESET_VF_BYE;
    msg->hdr.src_func_id = func_id;

    return QDMA_SUCCESS;
}

/* MD:*
 * qdma_mbox_compose_vf_qreq() - Compose a mailbox message for VF queue request
 *
 * @func_id: Function ID
 * @qmax: Maximum queue value
 * @qbase: Base queue value
 * @raw_data: Pointer to the raw data buffer
 *
 * Return: QDMA_SUCCESS on success, negative error code on failure
 */
int qdma_mbox_compose_vf_qreq(uint16_t func_id,
                              uint16_t qmax, int qbase, uint32_t *raw_data)
{
    union qdma_mbox_txrx *msg = (union qdma_mbox_txrx *)raw_data;

    // MD: Check for a valid raw_data pointer
    if (!raw_data) {
        qdma_log_error("%s: raw_data=%p, err:%d\n",
                        __func__, raw_data,
                        -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Initialize the raw data buffer to zero
    qdma_mbox_memset(raw_data, 0, sizeof(union qdma_mbox_txrx));

    // MD: Set the operation type and function ID
    msg->hdr.op = MBOX_OP_QREQ;
    msg->hdr.src_func_id = func_id;

    // MD: Set the queue base and maximum values
    msg->fmap.qbase = qbase;
    msg->fmap.qmax = qmax;

    return QDMA_SUCCESS;
}

/* MD:*
 * qdma_mbox_compose_vf_notify_qadd() - Compose a mailbox message to notify queue addition
 *
 * @func_id: Function ID
 * @qid_hw: Hardware queue ID
 * @q_type: Queue type
 * @raw_data: Pointer to the raw data buffer
 *
 * Return: QDMA_SUCCESS on success, negative error code on failure
 */
int qdma_mbox_compose_vf_notify_qadd(uint16_t func_id,
                                     uint16_t qid_hw,
                                     enum qdma_dev_q_type q_type,
                                     uint32_t *raw_data)
{
    union qdma_mbox_txrx *msg = (union qdma_mbox_txrx *)raw_data;

    // MD: Check for a valid raw_data pointer
    if (!raw_data) {
        qdma_log_error("%s: raw_data=%p, err:%d\n",
                        __func__, raw_data,
                        -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Initialize the raw data buffer to zero
    qdma_mbox_memset(raw_data, 0, sizeof(union qdma_mbox_txrx));

    // MD: Set the operation type and function ID
    msg->hdr.op = MBOX_OP_QNOTIFY_ADD;
    msg->hdr.src_func_id = func_id;

    // MD: Set the hardware queue ID and queue type
    msg->q_notify.qid_hw = qid_hw;
    msg->q_notify.q_type = q_type;

    return QDMA_SUCCESS;
}

/* MD:*
 * qdma_mbox_compose_vf_get_device_active_qcnt() - Compose a mailbox message to get active queue count
 *
 * @func_id: Function ID
 * @raw_data: Pointer to the raw data buffer
 *
 * Return: QDMA_SUCCESS on success, negative error code on failure
 */
int qdma_mbox_compose_vf_get_device_active_qcnt(uint16_t func_id,
                                                uint32_t *raw_data)
{
    union qdma_mbox_txrx *msg = (union qdma_mbox_txrx *)raw_data;

    // MD: Check for a valid raw_data pointer
    if (!raw_data) {
        qdma_log_error("%s: raw_data=%p, err:%d\n",
                        __func__, raw_data,
                        -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Initialize the raw data buffer to zero
    qdma_mbox_memset(raw_data, 0, sizeof(union qdma_mbox_txrx));

    // MD: Set the operation type and function ID
    msg->hdr.op = MBOX_OP_GET_QACTIVE_CNT;
    msg->hdr.src_func_id = func_id;

    return QDMA_SUCCESS;
}

/* MD:*
 * qdma_mbox_compose_vf_notify_qdel() - Compose a mailbox message to notify queue deletion
 *
 * @func_id: Function ID
 * @qid_hw: Hardware queue ID
 * @q_type: Queue type
 * @raw_data: Pointer to the raw data buffer
 *
 * Return: QDMA_SUCCESS on success, negative error code on failure
 */
int qdma_mbox_compose_vf_notify_qdel(uint16_t func_id,
                                     uint16_t qid_hw,
                                     enum qdma_dev_q_type q_type,
                                     uint32_t *raw_data)
{
    union qdma_mbox_txrx *msg = (union qdma_mbox_txrx *)raw_data;

    // MD: Check for a valid raw_data pointer
    if (!raw_data) {
        qdma_log_error("%s: raw_data=%p, err:%d\n",
                        __func__, raw_data,
                        -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Initialize the raw data buffer to zero
    qdma_mbox_memset(raw_data, 0, sizeof(union qdma_mbox_txrx));

    // MD: Set the operation type and function ID
    msg->hdr.op = MBOX_OP_QNOTIFY_DEL;
    msg->hdr.src_func_id = func_id;

    // MD: Set the hardware queue ID and queue type
    msg->q_notify.qid_hw = qid_hw;
    msg->q_notify.q_type = q_type;

    return QDMA_SUCCESS;
}

/* MD:*
 * qdma_mbox_compose_vf_fmap_prog() - Compose a mailbox message for function map programming
 *
 * @func_id: Function ID
 * @qmax: Maximum queue value
 * @qbase: Base queue value
 * @raw_data: Pointer to the raw data buffer
 *
 * Return: QDMA_SUCCESS on success, negative error code on failure
 */
int qdma_mbox_compose_vf_fmap_prog(uint16_t func_id,
                                   uint16_t qmax, int qbase,
                                   uint32_t *raw_data)
{
    union qdma_mbox_txrx *msg = (union qdma_mbox_txrx *)raw_data;

    // MD: Check for a valid raw_data pointer
    if (!raw_data) {
        qdma_log_error("%s: raw_data=%p, err:%d\n",
                        __func__, raw_data,
                        -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Initialize the raw data buffer to zero
    qdma_mbox_memset(raw_data, 0, sizeof(union qdma_mbox_txrx));

    // MD: Set the operation type and function ID
    msg->hdr.op = MBOX_OP_FMAP;
    msg->hdr.src_func_id = func_id;

    // MD: Set the queue base and maximum values
    msg->fmap.qbase = (uint32_t)qbase;
    msg->fmap.qmax = qmax;

    return QDMA_SUCCESS;
}

/* MD:*
 * qdma_mbox_compose_vf_qctxt_write() - Compose a mailbox message to write queue context
 *
 * @func_id: Function ID
 * @qid_hw: Queue ID in hardware
 * @st: Stream type
 * @c2h: C2H (Completion to Host) flag
 * @cmpt_ctxt_type: Completion context type
 * @descq_conf: Descriptor queue configuration
 * @raw_data: Pointer to raw data buffer
 *
 * Return: QDMA_SUCCESS on success, negative error code on failure
 */
int qdma_mbox_compose_vf_qctxt_write(uint16_t func_id,
            uint16_t qid_hw, uint8_t st, uint8_t c2h,
            enum mbox_cmpt_ctxt_type cmpt_ctxt_type,
            struct mbox_descq_conf *descq_conf,
            uint32_t *raw_data)
{
    union qdma_mbox_txrx *msg = (union qdma_mbox_txrx *)raw_data;

    // MD: Check for a valid raw data pointer
    if (!raw_data) {
        qdma_log_error("%s: raw_data=%p, err:%d\n",
                        __func__, raw_data,
                        -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Initialize the raw data buffer
    qdma_mbox_memset(raw_data, 0, sizeof(union qdma_mbox_txrx));
    msg->hdr.op = MBOX_OP_QCTXT_WRT;
    msg->hdr.src_func_id = func_id;
    msg->qctxt.qid_hw = qid_hw;
    msg->qctxt.c2h = c2h;
    msg->qctxt.st = st;
    msg->qctxt.cmpt_ctxt_type = cmpt_ctxt_type;

    // MD: Copy descriptor queue configuration
    qdma_mbox_memcpy(&msg->qctxt.descq_conf, descq_conf,
           sizeof(struct mbox_descq_conf));

    return QDMA_SUCCESS;
}

/* MD:*
 * qdma_mbox_compose_vf_qctxt_read() - Compose a mailbox message to read queue context
 *
 * @func_id: Function ID
 * @qid_hw: Queue ID in hardware
 * @st: Stream type
 * @c2h: C2H (Completion to Host) flag
 * @cmpt_ctxt_type: Completion context type
 * @raw_data: Pointer to raw data buffer
 *
 * Return: QDMA_SUCCESS on success, negative error code on failure
 */
int qdma_mbox_compose_vf_qctxt_read(uint16_t func_id,
                uint16_t qid_hw, uint8_t st, uint8_t c2h,
                enum mbox_cmpt_ctxt_type cmpt_ctxt_type,
                uint32_t *raw_data)
{
    union qdma_mbox_txrx *msg = (union qdma_mbox_txrx *)raw_data;

    // MD: Check for a valid raw data pointer
    if (!raw_data) {
        qdma_log_error("%s: raw_data=%p, err:%d\n",
                        __func__, raw_data,
                        -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Initialize the raw data buffer
    qdma_mbox_memset(raw_data, 0, sizeof(union qdma_mbox_txrx));
    msg->hdr.op = MBOX_OP_QCTXT_RD;
    msg->hdr.src_func_id = func_id;
    msg->qctxt.qid_hw = qid_hw;
    msg->qctxt.c2h = c2h;
    msg->qctxt.st = st;
    msg->qctxt.cmpt_ctxt_type = cmpt_ctxt_type;

    return QDMA_SUCCESS;
}

/* MD:*
 * qdma_mbox_compose_vf_qctxt_invalidate() - Compose a mailbox message to invalidate queue context
 *
 * @func_id: Function ID
 * @qid_hw: Queue ID in hardware
 * @st: Stream type
 * @c2h: C2H (Completion to Host) flag
 * @cmpt_ctxt_type: Completion context type
 * @raw_data: Pointer to raw data buffer
 *
 * Return: QDMA_SUCCESS on success, negative error code on failure
 */
int qdma_mbox_compose_vf_qctxt_invalidate(uint16_t func_id,
                uint16_t qid_hw, uint8_t st, uint8_t c2h,
                enum mbox_cmpt_ctxt_type cmpt_ctxt_type,
                uint32_t *raw_data)
{
    union qdma_mbox_txrx *msg = (union qdma_mbox_txrx *)raw_data;

    // MD: Check for a valid raw data pointer
    if (!raw_data) {
        qdma_log_error("%s: raw_data=%p, err:%d\n",
                        __func__, raw_data,
                        -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Initialize the raw data buffer
    qdma_mbox_memset(raw_data, 0, sizeof(union qdma_mbox_txrx));
    msg->hdr.op = MBOX_OP_QCTXT_INV;
    msg->hdr.src_func_id = func_id;
    msg->qctxt.qid_hw = qid_hw;
    msg->qctxt.c2h = c2h;
    msg->qctxt.st = st;
    msg->qctxt.cmpt_ctxt_type = cmpt_ctxt_type;

    return QDMA_SUCCESS;
}

/* MD:*
 * qdma_mbox_compose_vf_qctxt_clear() - Compose a mailbox message to clear queue context
 *
 * @func_id: Function ID
 * @qid_hw: Queue ID in hardware
 * @st: Stream type
 * @c2h: C2H (Completion to Host) flag
 * @cmpt_ctxt_type: Completion context type
 * @raw_data: Pointer to raw data buffer
 *
 * Return: QDMA_SUCCESS on success, negative error code on failure
 */
int qdma_mbox_compose_vf_qctxt_clear(uint16_t func_id,
                uint16_t qid_hw, uint8_t st, uint8_t c2h,
                enum mbox_cmpt_ctxt_type cmpt_ctxt_type,
                uint32_t *raw_data)
{
    union qdma_mbox_txrx *msg = (union qdma_mbox_txrx *)raw_data;

    // MD: Check for a valid raw data pointer
    if (!raw_data) {
        qdma_log_error("%s: raw_data=%p, err:%d\n",
                        __func__, raw_data,
                        -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Initialize the raw data buffer
    qdma_mbox_memset(raw_data, 0, sizeof(union qdma_mbox_txrx));
    msg->hdr.op = MBOX_OP_QCTXT_CLR;
    msg->hdr.src_func_id = func_id;
    msg->qctxt.qid_hw = qid_hw;
    msg->qctxt.c2h = c2h;
    msg->qctxt.st = st;
    msg->qctxt.cmpt_ctxt_type = cmpt_ctxt_type;

    return QDMA_SUCCESS;
}

/* MD:*
 * qdma_mbox_compose_csr_read() - Compose a mailbox message to read CSR
 *
 * @func_id: Function ID
 * @raw_data: Pointer to raw data buffer
 *
 * Return: QDMA_SUCCESS on success, negative error code on failure
 */
int qdma_mbox_compose_csr_read(uint16_t func_id,
                   uint32_t *raw_data)
{
    union qdma_mbox_txrx *msg = (union qdma_mbox_txrx *)raw_data;

    // MD: Check for a valid raw data pointer
    if (!raw_data) {
        qdma_log_error("%s: raw_data=%p, err:%d\n",
                        __func__, raw_data,
                        -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Initialize the raw data buffer
    qdma_mbox_memset(raw_data, 0, sizeof(union qdma_mbox_txrx));
    msg->hdr.op = MBOX_OP_CSR;
    msg->hdr.src_func_id = func_id;

    return QDMA_SUCCESS;
}

/* MD:*
 * qdma_mbox_compose_reg_read() - Compose a mailbox message to read register list
 *
 * @func_id: Function ID
 * @group_num: Group number of registers to read
 * @raw_data: Pointer to raw data buffer
 *
 * Return: QDMA_SUCCESS on success, negative error code on failure
 */
int qdma_mbox_compose_reg_read(uint16_t func_id,
                    uint16_t group_num,
                    uint32_t *raw_data)
{
    union qdma_mbox_txrx *msg = (union qdma_mbox_txrx *)raw_data;

    // MD: Check for a valid raw data pointer
    if (!raw_data) {
        qdma_log_error("%s: raw_data=%p, err:%d\n",
                        __func__, raw_data,
                        -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Initialize the raw data buffer
    qdma_mbox_memset(raw_data, 0, sizeof(union qdma_mbox_txrx));
    msg->hdr.op = MBOX_OP_REG_LIST_READ;
    msg->hdr.src_func_id = func_id;
    msg->reg_read_list.group_num = group_num;

    return QDMA_SUCCESS;
}

/* MD:*
 * qdma_mbox_compose_vf_intr_ctxt_write() - Compose a mailbox message to write
 * interrupt context for a virtual function.
 *
 * @func_id: Function ID of the VF
 * @intr_ctxt: Pointer to the interrupt context structure
 * @raw_data: Pointer to the raw data buffer to be filled
 *
 * Return: QDMA_SUCCESS on success, negative error code on failure
 */
int qdma_mbox_compose_vf_intr_ctxt_write(uint16_t func_id,
                                         struct mbox_msg_intr_ctxt *intr_ctxt,
                                         uint32_t *raw_data)
{
    union qdma_mbox_txrx *msg = (union qdma_mbox_txrx *)raw_data;

    // MD: Check for a valid raw_data pointer
    if (!raw_data) {
        qdma_log_error("%s: raw_data=%p, err:%d\n",
                       __func__, raw_data, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Initialize the raw data buffer
    qdma_mbox_memset(raw_data, 0, sizeof(union qdma_mbox_txrx));
    // MD: Set the operation and source function ID
    msg->hdr.op = MBOX_OP_INTR_CTXT_WRT;
    msg->hdr.src_func_id = func_id;
    // MD: Copy the interrupt context into the message
    qdma_mbox_memcpy(&msg->intr_ctxt.ctxt, intr_ctxt,
                     sizeof(struct mbox_msg_intr_ctxt));

    return QDMA_SUCCESS;
}

/* MD:*
 * qdma_mbox_compose_vf_intr_ctxt_read() - Compose a mailbox message to read
 * interrupt context for a virtual function.
 *
 * @func_id: Function ID of the VF
 * @intr_ctxt: Pointer to the interrupt context structure
 * @raw_data: Pointer to the raw data buffer to be filled
 *
 * Return: QDMA_SUCCESS on success, negative error code on failure
 */
int qdma_mbox_compose_vf_intr_ctxt_read(uint16_t func_id,
                                        struct mbox_msg_intr_ctxt *intr_ctxt,
                                        uint32_t *raw_data)
{
    union qdma_mbox_txrx *msg = (union qdma_mbox_txrx *)raw_data;

    // MD: Check for a valid raw_data pointer
    if (!raw_data) {
        qdma_log_error("%s: raw_data=%p, err:%d\n",
                       __func__, raw_data, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Initialize the raw data buffer
    qdma_mbox_memset(raw_data, 0, sizeof(union qdma_mbox_txrx));
    // MD: Set the operation and source function ID
    msg->hdr.op = MBOX_OP_INTR_CTXT_RD;
    msg->hdr.src_func_id = func_id;
    // MD: Copy the interrupt context into the message
    qdma_mbox_memcpy(&msg->intr_ctxt.ctxt, intr_ctxt,
                     sizeof(struct mbox_msg_intr_ctxt));

    return QDMA_SUCCESS;
}

/* MD:*
 * qdma_mbox_compose_vf_intr_ctxt_clear() - Compose a mailbox message to clear
 * interrupt context for a virtual function.
 *
 * @func_id: Function ID of the VF
 * @intr_ctxt: Pointer to the interrupt context structure
 * @raw_data: Pointer to the raw data buffer to be filled
 *
 * Return: QDMA_SUCCESS on success, negative error code on failure
 */
int qdma_mbox_compose_vf_intr_ctxt_clear(uint16_t func_id,
                                         struct mbox_msg_intr_ctxt *intr_ctxt,
                                         uint32_t *raw_data)
{
    union qdma_mbox_txrx *msg = (union qdma_mbox_txrx *)raw_data;

    // MD: Check for a valid raw_data pointer
    if (!raw_data) {
        qdma_log_error("%s: raw_data=%p, err:%d\n",
                       __func__, raw_data, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Initialize the raw data buffer
    qdma_mbox_memset(raw_data, 0, sizeof(union qdma_mbox_txrx));
    // MD: Set the operation and source function ID
    msg->hdr.op = MBOX_OP_INTR_CTXT_CLR;
    msg->hdr.src_func_id = func_id;
    // MD: Copy the interrupt context into the message
    qdma_mbox_memcpy(&msg->intr_ctxt.ctxt, intr_ctxt,
                     sizeof(struct mbox_msg_intr_ctxt));

    return QDMA_SUCCESS;
}

/* MD:*
 * qdma_mbox_compose_vf_intr_ctxt_invalidate() - Compose a mailbox message to
 * invalidate interrupt context for a virtual function.
 *
 * @func_id: Function ID of the VF
 * @intr_ctxt: Pointer to the interrupt context structure
 * @raw_data: Pointer to the raw data buffer to be filled
 *
 * Return: QDMA_SUCCESS on success, negative error code on failure
 */
int qdma_mbox_compose_vf_intr_ctxt_invalidate(uint16_t func_id,
                                              struct mbox_msg_intr_ctxt *intr_ctxt,
                                              uint32_t *raw_data)
{
    union qdma_mbox_txrx *msg = (union qdma_mbox_txrx *)raw_data;

    // MD: Check for a valid raw_data pointer
    if (!raw_data) {
        qdma_log_error("%s: raw_data=%p, err:%d\n",
                       __func__, raw_data, -QDMA_ERR_INV_PARAM);
        return -QDMA_ERR_INV_PARAM;
    }

    // MD: Initialize the raw data buffer
    qdma_mbox_memset(raw_data, 0, sizeof(union qdma_mbox_txrx));
    // MD: Set the operation and source function ID
    msg->hdr.op = MBOX_OP_INTR_CTXT_INV;
    msg->hdr.src_func_id = func_id;
    // MD: Copy the interrupt context into the message
    qdma_mbox_memcpy(&msg->intr_ctxt.ctxt, intr_ctxt,
                     sizeof(struct mbox_msg_intr_ctxt));

    return QDMA_SUCCESS;
}

/* MD:*
 * qdma_mbox_is_msg_response() - Check if the received message is a response
 * to the sent message.
 *
 * @send_data: Pointer to the sent message data
 * @rcv_data: Pointer to the received message data
 *
 * Return: 1 if the received message is a response, 0 otherwise
 */
uint8_t qdma_mbox_is_msg_response(uint32_t *send_data, uint32_t *rcv_data)
{
    union qdma_mbox_txrx *tx_msg = (union qdma_mbox_txrx *)send_data;
    union qdma_mbox_txrx *rx_msg = (union qdma_mbox_txrx *)rcv_data;

    // MD: Check if the operation code of the received message matches the expected response
    return ((tx_msg->hdr.op + MBOX_MSG_OP_RSP_OFFSET) == rx_msg->hdr.op) ? 1 : 0;
}

/* MD:*
 * qdma_mbox_vf_response_status() - Get the status from the received message
 *
 * @rcv_data: Pointer to the received message data
 *
 * Return: Status of the received message
 */
int qdma_mbox_vf_response_status(uint32_t *rcv_data)
{
    union qdma_mbox_txrx *msg = (union qdma_mbox_txrx *)rcv_data;

    // MD: Return the status field from the message header
    return msg->hdr.status;
}

/* MD:*
 * qdma_mbox_vf_func_id_get() - Get the function ID from the received message
 *
 * @rcv_data: Pointer to the received message data
 * @is_vf: Whether the message is for a VF
 *
 * Return: Function ID extracted from the message
 */
uint8_t qdma_mbox_vf_func_id_get(uint32_t *rcv_data, uint8_t is_vf)
{
    union qdma_mbox_txrx *msg = (union qdma_mbox_txrx *)rcv_data;
    uint16_t func_id;

    // MD: Determine the function ID based on whether it's a VF or not
    if (is_vf)
        func_id = msg->hdr.dst_func_id;
    else
        func_id = msg->hdr.src_func_id;

    return func_id;
}

/* MD:*
 * qdma_mbox_vf_active_queues_get() - Get the number of active queues from the
 * received message
 *
 * @rcv_data: Pointer to the received message data
 * @q_type: Queue type (H2C, C2H, CMPT)
 *
 * Return: Number of active queues of the specified type
 */
int qdma_mbox_vf_active_queues_get(uint32_t *rcv_data,
                                   enum qdma_dev_q_type q_type)
{
    union qdma_mbox_txrx *msg = (union qdma_mbox_txrx *)rcv_data;
    int queues = 0;

    // MD: Determine the number of active queues based on the queue type
    if (q_type == QDMA_DEV_Q_TYPE_H2C)
        queues = msg->qcnt.h2c_queues;

    if (q_type == QDMA_DEV_Q_TYPE_C2H)
        queues = msg->qcnt.c2h_queues;

    if (q_type == QDMA_DEV_Q_TYPE_CMPT)
        queues = msg->qcnt.cmpt_queues;

    return queues;
}

/* MD:*
 * qdma_mbox_vf_parent_func_id_get() - Get the parent function ID from received data
 *
 * @rcv_data: Pointer to the received data
 *
 * Return: Source function ID
 */
uint8_t qdma_mbox_vf_parent_func_id_get(uint32_t *rcv_data)
{
    union qdma_mbox_txrx *msg = (union qdma_mbox_txrx *)rcv_data;

    // MD: Return the source function ID from the message header
    return msg->hdr.src_func_id;
}

/* MD:*
 * qdma_mbox_vf_dev_info_get() - Get device information from received data
 *
 * @rcv_data: Pointer to the received data
 * @dev_cap: Pointer to store device capabilities
 * @dma_device_index: Pointer to store DMA device index
 *
 * Return: Status from the message header
 */
int qdma_mbox_vf_dev_info_get(uint32_t *rcv_data,
    struct qdma_dev_attributes *dev_cap, uint32_t *dma_device_index)
{
    union qdma_mbox_txrx *msg = (union qdma_mbox_txrx *)rcv_data;

    // MD: Copy device capabilities and DMA device index from the message
    *dev_cap = msg->hello.dev_cap;
    *dma_device_index = msg->hello.dma_device_index;

    // MD: Return the status from the message header
    return msg->hdr.status;
}

/* MD:*
 * qdma_mbox_vf_qinfo_get() - Get queue information from received data
 *
 * @rcv_data: Pointer to the received data
 * @qbase: Pointer to store queue base
 * @qmax: Pointer to store maximum queue size
 *
 * Return: Status from the message header
 */
int qdma_mbox_vf_qinfo_get(uint32_t *rcv_data, int *qbase, uint16_t *qmax)
{
    union qdma_mbox_txrx *msg = (union qdma_mbox_txrx *)rcv_data;

    // MD: Extract queue base and maximum queue size from the message
    *qbase = msg->fmap.qbase;
    *qmax = msg->fmap.qmax;

    // MD: Return the status from the message header
    return msg->hdr.status;
}

/* MD:*
 * qdma_mbox_vf_csr_get() - Get CSR information from received data
 *
 * @rcv_data: Pointer to the received data
 * @csr: Pointer to store CSR information
 *
 * Return: Status from the message header
 */
int qdma_mbox_vf_csr_get(uint32_t *rcv_data, struct qdma_csr_info *csr)
{
    union qdma_mbox_txrx *msg = (union qdma_mbox_txrx *)rcv_data;

    // MD: Copy CSR information from the message
    qdma_mbox_memcpy(csr, &msg->csr.csr_info, sizeof(struct qdma_csr_info));

    // MD: Return the status from the message header
    return msg->hdr.status;
}

/* MD:*
 * qdma_mbox_vf_reg_list_get() - Get register list from received data
 *
 * @rcv_data: Pointer to the received data
 * @num_regs: Pointer to store the number of registers
 * @reg_list: Pointer to store the register list
 *
 * Return: Status from the message header
 */
int qdma_mbox_vf_reg_list_get(uint32_t *rcv_data,
    uint16_t *num_regs, struct qdma_reg_data *reg_list)
{
    union qdma_mbox_txrx *msg = (union qdma_mbox_txrx *)rcv_data;

    // MD: Extract the number of registers and copy the register list from the message
    *num_regs = msg->reg_read_list.num_regs;
    qdma_mbox_memcpy(reg_list, &(msg->reg_read_list.reg_list),
        (*num_regs * sizeof(struct qdma_reg_data)));

    // MD: Return the status from the message header
    return msg->hdr.status;
}

/* MD:*
 * qdma_mbox_vf_context_get() - Get descriptor queue context from received data
 *
 * @rcv_data: Pointer to the received data
 * @ctxt: Pointer to store descriptor queue context
 *
 * Return: Status from the message header
 */
int qdma_mbox_vf_context_get(uint32_t *rcv_data,
    struct qdma_descq_context *ctxt)
{
    union qdma_mbox_txrx *msg = (union qdma_mbox_txrx *)rcv_data;

    // MD: Copy descriptor queue context from the message
    qdma_mbox_memcpy(ctxt, &msg->qctxt.descq_ctxt,
        sizeof(struct qdma_descq_context));

    // MD: Return the status from the message header
    return msg->hdr.status;
}

/* MD:*
 * qdma_mbox_vf_intr_context_get() - Get interrupt context from received data
 *
 * @rcv_data: Pointer to the received data
 * @ictxt: Pointer to store interrupt context
 *
 * Return: Status from the message header
 */
int qdma_mbox_vf_intr_context_get(uint32_t *rcv_data,
    struct mbox_msg_intr_ctxt *ictxt)
{
    union qdma_mbox_txrx *msg = (union qdma_mbox_txrx *)rcv_data;

    // MD: Copy interrupt context from the message
    qdma_mbox_memcpy(ictxt, &msg->intr_ctxt.ctxt,
        sizeof(struct mbox_msg_intr_ctxt));

    // MD: Return the status from the message header
    return msg->hdr.status;
}

/* MD:*
 * qdma_mbox_pf_hw_clear_ack() - Clear hardware acknowledgment for PF
 *
 * @dev_hndl: Device handle
 */
void qdma_mbox_pf_hw_clear_ack(void *dev_hndl)
{
    uint32_t v;
    uint32_t reg;
    int i;
    uint32_t mbox_base = get_mbox_offset(dev_hndl, 0);

    // MD: Calculate the register address for PF acknowledgment
    reg = mbox_base + MBOX_PF_ACK_BASE;

    // MD: Read the function status register
    v = qdma_reg_read(dev_hndl, mbox_base + MBOX_FN_STATUS);
    // MD: Check if acknowledgment is already cleared
    if ((v & F_MBOX_FN_STATUS_ACK) == 0)
        return;

    // MD: Iterate over the acknowledgment registers and clear them
    for (i = 0; i < MBOX_PF_ACK_COUNT; i++, reg += MBOX_PF_ACK_STEP) {
        v = qdma_reg_read(dev_hndl, reg);

        // MD: If the register value is zero, continue to the next
        if (!v)
            continue;

        // MD: Clear the acknowledgment status
        qdma_reg_write(dev_hndl, reg, v);
    }
}

/* MD:*
 * qdma_mbox_send() - Send a message via the mailbox
 *
 * @dev_hndl: Device handle
 * @is_vf:    Whether PF or VF
 * @raw_data: Pointer to the raw data to be sent
 *
 * Return: 0 on success, negative error code on failure
 */
int qdma_mbox_send(void *dev_hndl, uint8_t is_vf, uint32_t *raw_data)
{
    int i;
    uint32_t reg = MBOX_OUT_MSG_BASE;
    uint32_t v;
    union qdma_mbox_txrx *msg = (union qdma_mbox_txrx *)raw_data;
    uint16_t dst_func_id = msg->hdr.dst_func_id;
    uint32_t mbox_base = get_mbox_offset(dev_hndl, is_vf);

    // MD: Check if the mailbox is busy
    v = qdma_reg_read(dev_hndl, mbox_base + MBOX_FN_STATUS);
    if (v & F_MBOX_FN_STATUS_OUT_MSG)
        return -QDMA_ERR_MBOX_SEND_BUSY;

    // MD: Set the target function ID for PF
    if (!is_vf)
        qdma_reg_write(dev_hndl, mbox_base + MBOX_FN_TARGET,
                       V_MBOX_FN_TARGET_ID(dst_func_id));

    // MD: Write the message to the mailbox registers
    for (i = 0; i < MBOX_MSG_REG_MAX; i++, reg += MBOX_MSG_STEP)
        qdma_reg_write(dev_hndl, mbox_base + reg, raw_data[i]);

    // MD: Clear the outgoing acknowledgment for PF
    if (!is_vf)
        mbox_pf_hw_clear_func_ack(dev_hndl, dst_func_id);

    // MD: Log the message details
    qdma_log_debug("%s %s tx from_id=%d, to_id=%d, opcode=0x%x\n", __func__,
                   is_vf ? "VF" : "PF", msg->hdr.src_func_id,
                   msg->hdr.dst_func_id, msg->hdr.op);

    // MD: Send the message
    qdma_reg_write(dev_hndl, mbox_base + MBOX_FN_CMD, F_MBOX_FN_CMD_SND);

    return QDMA_SUCCESS;
}

/* MD:*
 * qdma_mbox_rcv() - Receive a message via the mailbox
 *
 * @dev_hndl: Device handle
 * @is_vf:    Whether PF or VF
 * @raw_data: Pointer to store the received raw data
 *
 * Return: 0 on success, negative error code on failure
 */
int qdma_mbox_rcv(void *dev_hndl, uint8_t is_vf, uint32_t *raw_data)
{
    uint32_t reg = MBOX_IN_MSG_BASE;
    uint32_t v = 0;
    int all_zero_msg = 1;
    int i;
    uint32_t from_id = 0;
    union qdma_mbox_txrx *msg = (union qdma_mbox_txrx *)raw_data;
    uint32_t mbox_base = get_mbox_offset(dev_hndl, is_vf);

    // MD: Check if there is a message in the mailbox
    v = qdma_reg_read(dev_hndl, mbox_base + MBOX_FN_STATUS);
    if (!(v & M_MBOX_FN_STATUS_IN_MSG))
        return -QDMA_ERR_MBOX_NO_MSG_IN;

    // MD: Set the source function ID for PF
    if (!is_vf) {
        from_id = G_MBOX_FN_STATUS_SRC(v);
        qdma_reg_write(dev_hndl, mbox_base + MBOX_FN_TARGET, from_id);
    }

    // MD: Read the message from the mailbox registers
    for (i = 0; i < MBOX_MSG_REG_MAX; i++, reg += MBOX_MSG_STEP) {
        raw_data[i] = qdma_reg_read(dev_hndl, mbox_base + reg);
        // MD: Check if the received message is all zeros
        if (raw_data[i])
            all_zero_msg = 0;
    }

    // MD: Acknowledge the sender
    qdma_reg_write(dev_hndl, mbox_base + MBOX_FN_CMD, F_MBOX_FN_CMD_RCV);
    if (all_zero_msg) {
        qdma_log_error("%s: Message recv'd is all zeros. failure:%d\n",
                       __func__, -QDMA_ERR_MBOX_ALL_ZERO_MSG);
        return -QDMA_ERR_MBOX_ALL_ZERO_MSG;
    }

    // MD: Log the message details
    qdma_log_debug("%s %s fid=%d, opcode=0x%x\n", __func__,
                   is_vf ? "VF" : "PF", msg->hdr.dst_func_id,
                   msg->hdr.op);

    // MD: Update the source function ID if necessary
    if (!is_vf && (from_id != msg->hdr.src_func_id))
        msg->hdr.src_func_id = from_id;

    return QDMA_SUCCESS;
}

/* MD:*
 * qdma_mbox_hw_init() - Initialize the mailbox hardware
 *
 * @dev_hndl: Device handle
 * @is_vf:    Whether PF or VF
 */
void qdma_mbox_hw_init(void *dev_hndl, uint8_t is_vf)
{
    uint32_t v;
    uint32_t mbox_base = get_mbox_offset(dev_hndl, is_vf);

    // MD: Initialize the mailbox for VF
    if (is_vf) {
        v = qdma_reg_read(dev_hndl, mbox_base + MBOX_FN_STATUS);
        if (v & M_MBOX_FN_STATUS_IN_MSG)
            qdma_reg_write(dev_hndl, mbox_base + MBOX_FN_CMD,
                           F_MBOX_FN_CMD_RCV);
    } else
        qdma_mbox_pf_hw_clear_ack(dev_hndl);
}

/* MD:*
 * qdma_mbox_enable_interrupts() - Enable mailbox interrupts
 *
 * @dev_hndl: Device handle
 * @is_vf:    Whether PF or VF
 */
void qdma_mbox_enable_interrupts(void *dev_hndl, uint8_t is_vf)
{
    int vector = 0x0;
    uint32_t mbox_base = get_mbox_offset(dev_hndl, is_vf);

    // MD: Enable interrupts for the mailbox
    qdma_reg_write(dev_hndl, mbox_base + MBOX_ISR_VEC, vector);
    qdma_reg_write(dev_hndl, mbox_base + MBOX_ISR_EN, 0x1);
}

/* MD:*
 * qdma_mbox_disable_interrupts() - Disable mailbox interrupts
 *
 * @dev_hndl: Device handle
 * @is_vf:    Whether PF or VF
 */
void qdma_mbox_disable_interrupts(void *dev_hndl, uint8_t is_vf)
{
    uint32_t mbox_base = get_mbox_offset(dev_hndl, is_vf);

    // MD: Disable interrupts for the mailbox
    qdma_reg_write(dev_hndl, mbox_base + MBOX_ISR_EN, 0x0);
}

/* MD:*
 * qdma_mbox_compose_vf_reset_message() - Compose a VF reset message
 *
 * @raw_data: Pointer to the raw data buffer
 * @src_funcid: Source function ID
 * @dest_funcid: Destination function ID
 *
 * Return: 0 on success, negative error code on failure
 */
int qdma_mbox_compose_vf_reset_message(uint32_t *raw_data, uint8_t src_funcid,
                uint8_t dest_funcid)
{
    union qdma_mbox_txrx *msg = (union qdma_mbox_txrx *)raw_data;

    // MD: Check for a valid raw data pointer
    if (!raw_data)
        return -QDMA_ERR_INV_PARAM;

    // MD: Initialize the raw data buffer
    qdma_mbox_memset(raw_data, 0, sizeof(union qdma_mbox_txrx));
    msg->hdr.op = MBOX_OP_RESET_PREPARE;
    msg->hdr.src_func_id = src_funcid;
    msg->hdr.dst_func_id = dest_funcid;

    return 0;
}

/* MD:*
 * qdma_mbox_compose_pf_reset_done_message() - Compose a PF reset done message
 *
 * @raw_data: Pointer to the raw data buffer
 * @src_funcid: Source function ID
 * @dest_funcid: Destination function ID
 *
 * Return: 0 on success, negative error code on failure
 */
int qdma_mbox_compose_pf_reset_done_message(uint32_t *raw_data,
                    uint8_t src_funcid, uint8_t dest_funcid)
{
    union qdma_mbox_txrx *msg = (union qdma_mbox_txrx *)raw_data;

    // MD: Check for a valid raw data pointer
    if (!raw_data)
        return -QDMA_ERR_INV_PARAM;

    // MD: Initialize the raw data buffer
    qdma_mbox_memset(raw_data, 0, sizeof(union qdma_mbox_txrx));
    msg->hdr.op = MBOX_OP_RESET_DONE;
    msg->hdr.src_func_id = src_funcid;
    msg->hdr.dst_func_id = dest_funcid;

    return 0;
}

/* MD:*
 * qdma_mbox_compose_pf_offline() - Compose a PF offline message
 *
 * @raw_data: Pointer to the raw data buffer
 * @src_funcid: Source function ID
 * @dest_funcid: Destination function ID
 *
 * Return: 0 on success, negative error code on failure
 */
int qdma_mbox_compose_pf_offline(uint32_t *raw_data, uint8_t src_funcid,
                uint8_t dest_funcid)
{
    union qdma_mbox_txrx *msg = (union qdma_mbox_txrx *)raw_data;

    // MD: Check for a valid raw data pointer
    if (!raw_data)
        return -QDMA_ERR_INV_PARAM;

    // MD: Initialize the raw data buffer
    qdma_mbox_memset(raw_data, 0, sizeof(union qdma_mbox_txrx));
    msg->hdr.op = MBOX_OP_PF_BYE;
    msg->hdr.src_func_id = src_funcid;
    msg->hdr.dst_func_id = dest_funcid;

    return 0;
}

/* MD:*
 * qdma_mbox_vf_rcv_msg_handler() - Handle received VF messages
 *
 * @rcv_msg: Pointer to the received message
 * @resp_msg: Pointer to the response message
 *
 * Return: Message handling result
 */
int qdma_mbox_vf_rcv_msg_handler(uint32_t *rcv_msg, uint32_t *resp_msg)
{
    union qdma_mbox_txrx *rcv =  (union qdma_mbox_txrx *)rcv_msg;
    union qdma_mbox_txrx *resp =  (union qdma_mbox_txrx *)resp_msg;
    int rv = 0;

    // MD: Handle the received message based on its operation code
    switch (rcv->hdr.op) {
    case MBOX_OP_RESET_PREPARE:
        resp->hdr.op = rcv->hdr.op + MBOX_MSG_OP_RSP_OFFSET;
        resp->hdr.dst_func_id = rcv->hdr.src_func_id;
        resp->hdr.src_func_id = rcv->hdr.dst_func_id;
        rv = QDMA_MBOX_VF_RESET;
        break;
    case MBOX_OP_RESET_DONE:
        resp->hdr.op = rcv->hdr.op + MBOX_MSG_OP_RSP_OFFSET;
        resp->hdr.dst_func_id = rcv->hdr.src_func_id;
        resp->hdr.src_func_id = rcv->hdr.dst_func_id;
        rv = QDMA_MBOX_PF_RESET_DONE;
        break;
    case MBOX_OP_PF_BYE:
        resp->hdr.op = rcv->hdr.op + MBOX_MSG_OP_RSP_OFFSET;
        resp->hdr.dst_func_id = rcv->hdr.src_func_id;
        resp->hdr.src_func_id = rcv->hdr.dst_func_id;
        rv = QDMA_MBOX_PF_BYE;
        break;
    default:
        break;
    }
    return rv;
}

/* MD:*
 * qdma_mbox_out_status() - Check the mailbox output status
 *
 * @dev_hndl: Device handle
 * @is_vf: Whether PF or VF
 *
 * Return: 1 if there is an outgoing message, 0 otherwise
 */
uint8_t qdma_mbox_out_status(void *dev_hndl, uint8_t is_vf)
{
    uint32_t v;
    uint32_t mbox_base = get_mbox_offset(dev_hndl, is_vf);

    // MD: Read the mailbox function status register
    v = qdma_reg_read(dev_hndl, mbox_base + MBOX_FN_STATUS);

    // MD: Check if there is an outgoing message
    if (v & F_MBOX_FN_STATUS_OUT_MSG)
        return 1;
    else
        return 0;
}
