// MD: SPDX-License-Identifier: GPL-2.0+
/*
 * PCIe host controller driver for Xilinx Versal CPM DMA Bridge
 *
 * (C) Copyright 2019 - 2020, Xilinx, Inc.
 */

#include <linux/bitfield.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_pci.h>
#include <linux/of_platform.h>

#include "../pci.h"
#include "pcie-xilinx-common.h"

/* Register definitions for the Xilinx CPM PCIe controller */
#define XILINX_CPM_PCIE_REG_IDR         0x00000E10  /* Interrupt Disable Register offset */
#define XILINX_CPM_PCIE_REG_IMR         0x00000E14  /* Interrupt Mask Register offset */
#define XILINX_CPM_PCIE_REG_PSCR        0x00000E1C  /* Phy Status/Control Register offset */
#define XILINX_CPM_PCIE_REG_RPSC        0x00000E20  /* Root Port Status/Control Register offset */
#define XILINX_CPM_PCIE_REG_RPEFR       0x00000E2C  /* Root Port Error FIFO Read Register offset */
#define XILINX_CPM_PCIE_REG_IDRN        0x00000E38  /* Interrupt Disable Register Next offset */
#define XILINX_CPM_PCIE_REG_IDRN_MASK   0x00000E3C  /* Mask for IDRN */
#define XILINX_CPM_PCIE_MISC_IR_STATUS  0x00000340  /* Miscellaneous Interrupt Status Register */
#define XILINX_CPM_PCIE_MISC_IR_ENABLE  0x00000348  /* Miscellaneous Interrupt Enable Register */
#define XILINX_CPM_PCIE0_MISC_IR_LOCAL  BIT(1)      /* Local interrupt bit for PCIe0 */
#define XILINX_CPM_PCIE1_MISC_IR_LOCAL  BIT(2)      /* Local interrupt bit for PCIe1 */

#define XILINX_CPM_PCIE0_IR_STATUS      0x000002A0  /* Interrupt Status Register for PCIe0 */
#define XILINX_CPM_PCIE1_IR_STATUS      0x000002B4  /* Interrupt Status Register for PCIe1 */
#define XILINX_CPM_PCIE0_IR_ENABLE      0x000002A8  /* Interrupt Enable Register for PCIe0 */
#define XILINX_CPM_PCIE1_IR_ENABLE      0x000002BC  /* Interrupt Enable Register for PCIe1 */
#define XILINX_CPM_PCIE_IR_LOCAL        BIT(0)      /* Local interrupt bit */

/* Macro to create a bitmask for interrupt management registers */
#define IMR(x) BIT(XILINX_PCIE_INTR_##x)

/* Aggregate all possible interrupt masks into a single mask */
#define XILINX_CPM_PCIE_IMR_ALL_MASK            \
    (                                           \
        IMR(LINK_DOWN)      |                   \
        IMR(HOT_RESET)      |                   \
        IMR(CFG_PCIE_TIMEOUT)|                  \
        IMR(CFG_TIMEOUT)    |                   \
        IMR(CORRECTABLE)    |                   \
        IMR(NONFATAL)       |                   \
        IMR(FATAL)          |                   \
        IMR(CFG_ERR_POISON) |                   \
        IMR(PME_TO_ACK_RCVD)|                   \
        IMR(INTX)           |                   \
        IMR(PM_PME_RCVD)    |                   \
        IMR(SLV_UNSUPP)     |                   \
        IMR(SLV_UNEXP)      |                   \
        IMR(SLV_COMPL)      |                   \
        IMR(SLV_ERRP)       |                   \
        IMR(SLV_CMPABT)     |                   \
        IMR(SLV_ILLBUR)     |                   \
        IMR(MST_DECERR)     |                   \
        IMR(MST_SLVERR)     |                   \
        IMR(SLV_PCIE_TIMEOUT)                   \
    )

#define XILINX_CPM_PCIE_IDR_ALL_MASK    0xFFFFFFFF  /* Mask to disable all interrupts */
#define XILINX_CPM_PCIE_IDRN_MASK       GENMASK(19, 16) /* Mask for IDRN */
#define XILINX_CPM_PCIE_IDRN_SHIFT      16          /* Shift for IDRN */

/* Root Port Error FIFO Read Register definitions */
#define XILINX_CPM_PCIE_RPEFR_ERR_VALID BIT(18)    /* Error valid bit */
#define XILINX_CPM_PCIE_RPEFR_REQ_ID    GENMASK(15, 0) /* Requester ID mask */
#define XILINX_CPM_PCIE_RPEFR_ALL_MASK  0xFFFFFFFF /* Mask for all errors */

/* Root Port Status/control Register definitions */
#define XILINX_CPM_PCIE_REG_RPSC_BEN    BIT(0)     /* Bridge enable bit */

/* Phy Status/Control Register definitions */
#define XILINX_CPM_PCIE_REG_PSCR_LNKUP  BIT(11)    /* Link-up bit */

/* Enumeration for different CPM versions */
enum xilinx_cpm_version {
    CPM,        /* CPM version */
    CPM5,       /* CPM5 version */
    CPM5_HOST1, /* CPM5 Host1 version */
};

/* MD:
 * struct xilinx_cpm_variant - CPM variant information
 * @version: CPM version
 * @ir_status: Offset for the error interrupt status register
 * @ir_enable: Offset for the CPM5 local error interrupt enable register
 * @ir_misc_value: A bitmask for the miscellaneous interrupt status
 */
struct xilinx_cpm_variant {
    enum xilinx_cpm_version version;
    u32 ir_status;
    u32 ir_enable;
    u32 ir_misc_value;
};

/*
 * struct xilinx_cpm_pcie - PCIe port information
 * @dev: Device pointer
 * @reg_base: Bridge Register Base
 * @cpm_base: CPM System Level Control and Status Register(SLCR) Base
 * @intx_domain: Legacy IRQ domain pointer
 * @cpm_domain: CPM IRQ domain pointer
 * @cfg: Holds mappings of config space window
 * @intx_irq: legacy interrupt number
 * @irq: Error interrupt number
 * @lock: lock protecting shared register access
 * @variant: CPM version check pointer
 */
struct xilinx_cpm_pcie {
    struct device           *dev;
    void __iomem            *reg_base;
    void __iomem            *cpm_base;
    struct irq_domain       *intx_domain;
    struct irq_domain       *cpm_domain;
    struct pci_config_window *cfg;
    int                     intx_irq;
    int                     irq;
    raw_spinlock_t          lock;
    const struct xilinx_cpm_variant *variant;
};

/* MD:
 * pcie_read - Read a value from a PCIe register
 * @port: Pointer to the PCIe port structure
 * @reg: Register offset to read from
 *
 * Return: The value read from the specified register
 */
static u32 pcie_read(struct xilinx_cpm_pcie *port, u32 reg)
{
    // MD: Read a 32-bit value from the specified register offset
    return readl_relaxed(port->reg_base + reg);
}

/* MD:
 * pcie_write - Write a value to a PCIe register
 * @port: Pointer to the PCIe port structure
 * @val: Value to write
 * @reg: Register offset to write to
 */
static void pcie_write(struct xilinx_cpm_pcie *port, u32 val, u32 reg)
{
    // MD: Debug print statement to log the write operation
    dev_dbg(port->dev, "Writing value 0x%x to register 0x%x\n", val, reg);
    // MD: Write a 32-bit value to the specified register offset
    writel_relaxed(val, port->reg_base + reg);
}

/* MD:
 * cpm_pcie_link_up - Check if the PCIe link is up
 * @port: Pointer to the PCIe port structure
 *
 * Return: True if the link is up, false otherwise
 */
static bool cpm_pcie_link_up(struct xilinx_cpm_pcie *port)
{
    // MD: Check the link-up status by reading the Phy Status/Control Register
    return (pcie_read(port, XILINX_CPM_PCIE_REG_PSCR) &
            XILINX_CPM_PCIE_REG_PSCR_LNKUP);
}

/* MD:
 * cpm_pcie_clear_err_interrupts - Clear error interrupts
 * @port: Pointer to the PCIe port structure
 */
static void cpm_pcie_clear_err_interrupts(struct xilinx_cpm_pcie *port)
{
    // MD: Read the Root Port Error FIFO Read Register
    unsigned long val = pcie_read(port, XILINX_CPM_PCIE_REG_RPEFR);

    // MD: Check if the error valid bit is set
    if (val & XILINX_CPM_PCIE_RPEFR_ERR_VALID) {
        // MD: Debug print statement to log the requester ID
        dev_dbg(port->dev, "Requester ID %lu\n",
                val & XILINX_CPM_PCIE_RPEFR_REQ_ID);
        // MD: Clear all errors by writing to the Root Port Error FIFO Read Register
        pcie_write(port, XILINX_CPM_PCIE_RPEFR_ALL_MASK,
                   XILINX_CPM_PCIE_REG_RPEFR);
    }
}

/* MD:
 * xilinx_cpm_mask_leg_irq - Mask a legacy IRQ
 * @data: IRQ data structure
 */
static void xilinx_cpm_mask_leg_irq(struct irq_data *data)
{
    struct xilinx_cpm_pcie *port = irq_data_get_irq_chip_data(data);
    unsigned long flags;
    u32 mask;
    u32 val;

    // MD: Calculate the mask for the IRQ
    mask = BIT(data->hwirq + XILINX_CPM_PCIE_IDRN_SHIFT);
    // MD: Acquire the lock and save the interrupt flags
    raw_spin_lock_irqsave(&port->lock, flags);
    // MD: Read the current mask value
    val = pcie_read(port, XILINX_CPM_PCIE_REG_IDRN_MASK);
    // MD: Update the mask to disable the IRQ
    pcie_write(port, (val & (~mask)), XILINX_CPM_PCIE_REG_IDRN_MASK);
    // MD: Release the lock and restore the interrupt flags
    raw_spin_unlock_irqrestore(&port->lock, flags);
}

/* MD:
 * xilinx_cpm_unmask_leg_irq - Unmask a legacy IRQ
 * @data: IRQ data structure
 */
static void xilinx_cpm_unmask_leg_irq(struct irq_data *data)
{
    struct xilinx_cpm_pcie *port = irq_data_get_irq_chip_data(data);
    unsigned long flags;
    u32 mask;
    u32 val;

    // MD: Calculate the mask for the IRQ
    mask = BIT(data->hwirq + XILINX_CPM_PCIE_IDRN_SHIFT);
    // MD: Acquire the lock and save the interrupt flags
    raw_spin_lock_irqsave(&port->lock, flags);
    // MD: Read the current mask value
    val = pcie_read(port, XILINX_CPM_PCIE_REG_IDRN_MASK);
    // MD: Update the mask to enable the IRQ
    pcie_write(port, (val | mask), XILINX_CPM_PCIE_REG_IDRN_MASK);
    // MD: Release the lock and restore the interrupt flags
    raw_spin_unlock_irqrestore(&port->lock, flags);
}

// MD: Define the IRQ chip for legacy interrupts
static struct irq_chip xilinx_cpm_leg_irq_chip = {
    .name       = "INTx",
    .irq_mask   = xilinx_cpm_mask_leg_irq,
    .irq_unmask = xilinx_cpm_unmask_leg_irq,
};

/* MD:
 * xilinx_cpm_pcie_intx_map - Set the handler for the INTx and mark IRQ as valid
 * @domain: IRQ domain
 * @irq: Virtual IRQ number
 * @hwirq: HW interrupt number
 *
 * Return: Always returns 0.
 */
static int xilinx_cpm_pcie_intx_map(struct irq_domain *domain,
                                    unsigned int irq, irq_hw_number_t hwirq)
{
    // MD: Set the IRQ chip and handler for the given IRQ
    irq_set_chip_and_handler(irq, &xilinx_cpm_leg_irq_chip,
                             handle_level_irq);
    // MD: Set the chip data for the IRQ
    irq_set_chip_data(irq, domain->host_data);
    // MD: Mark the IRQ as level-triggered
    irq_set_status_flags(irq, IRQ_LEVEL);

    return 0;
}

// MD: INTx IRQ Domain operations
static const struct irq_domain_ops intx_domain_ops = {
    .map = xilinx_cpm_pcie_intx_map,
};

/* MD:
 * xilinx_cpm_pcie_intx_flow - Handle INTx interrupts
 * @desc: IRQ descriptor
 */
static void xilinx_cpm_pcie_intx_flow(struct irq_desc *desc)
{
    struct xilinx_cpm_pcie *port = irq_desc_get_handler_data(desc);
    struct irq_chip *chip = irq_desc_get_chip(desc);
    unsigned long val;
    int i;

    // MD: Enter the chained IRQ handler
    chained_irq_enter(chip, desc);

    // MD: Read the masked interrupt status
    val = FIELD_GET(XILINX_CPM_PCIE_IDRN_MASK,
                    pcie_read(port, XILINX_CPM_PCIE_REG_IDRN));

    // MD: Handle each set interrupt bit
    for_each_set_bit(i, &val, PCI_NUM_INTX)
        generic_handle_domain_irq(port->intx_domain, i);

    // MD: Exit the chained IRQ handler
    chained_irq_exit(chip, desc);
}

/* MD:
 * xilinx_cpm_mask_event_irq - Mask an event IRQ
 * @d: IRQ data structure
 */
static void xilinx_cpm_mask_event_irq(struct irq_data *d)
{
    struct xilinx_cpm_pcie *port = irq_data_get_irq_chip_data(d);
    u32 val;

    // MD: Acquire the lock
    raw_spin_lock(&port->lock);
    // MD: Read the current interrupt mask
    val = pcie_read(port, XILINX_CPM_PCIE_REG_IMR);
    // MD: Update the mask to disable the event IRQ
    val &= ~BIT(d->hwirq);
    // MD: Write the updated mask
    pcie_write(port, val, XILINX_CPM_PCIE_REG_IMR);
    // MD: Release the lock
    raw_spin_unlock(&port->lock);
}

/* MD:
 * xilinx_cpm_unmask_event_irq - Unmask an event IRQ
 * @d: IRQ data structure
 */
static void xilinx_cpm_unmask_event_irq(struct irq_data *d)
{
    struct xilinx_cpm_pcie *port = irq_data_get_irq_chip_data(d);
    u32 val;

    // MD: Acquire the lock
    raw_spin_lock(&port->lock);
    // MD: Read the current interrupt mask
    val = pcie_read(port, XILINX_CPM_PCIE_REG_IMR);
    // MD: Update the mask to enable the event IRQ
    val |= BIT(d->hwirq);
    // MD: Write the updated mask
    pcie_write(port, val, XILINX_CPM_PCIE_REG_IMR);
    // MD: Release the lock
    raw_spin_unlock(&port->lock);
}

// MD: Define the IRQ chip for event interrupts
static struct irq_chip xilinx_cpm_event_irq_chip = {
    .name       = "RC-Event",
    .irq_mask   = xilinx_cpm_mask_event_irq,
    .irq_unmask = xilinx_cpm_unmask_event_irq,
};

/* MD:
 * xilinx_cpm_pcie_event_map - Map event IRQs
 * @domain: IRQ domain
 * @irq: Virtual IRQ number
 * @hwirq: HW interrupt number
 *
 * Return: Always returns 0.
 */
static int xilinx_cpm_pcie_event_map(struct irq_domain *domain,
                                     unsigned int irq, irq_hw_number_t hwirq)
{
    // MD: Set the IRQ chip and handler for the given IRQ
    irq_set_chip_and_handler(irq, &xilinx_cpm_event_irq_chip,
                             handle_level_irq);
    // MD: Set the chip data for the IRQ
    irq_set_chip_data(irq, domain->host_data);
    // MD: Mark the IRQ as level-triggered
    irq_set_status_flags(irq, IRQ_LEVEL);
    return 0;
}

// MD: Event IRQ Domain operations
static const struct irq_domain_ops event_domain_ops = {
    .map = xilinx_cpm_pcie_event_map,
};

/* MD:
 * xilinx_cpm_pcie_event_flow - Handle event interrupts
 * @desc: IRQ descriptor
 */
static void xilinx_cpm_pcie_event_flow(struct irq_desc *desc)
{
    struct xilinx_cpm_pcie *port = irq_desc_get_handler_data(desc);
    struct irq_chip *chip = irq_desc_get_chip(desc);
    const struct xilinx_cpm_variant *variant = port->variant;
    unsigned long val;
    int i;

    // MD: Enter the chained IRQ handler
    chained_irq_enter(chip, desc);
    // MD: Read and mask interrupt status
    val =  pcie_read(port, XILINX_CPM_PCIE_REG_IDR);
    val &= pcie_read(port, XILINX_CPM_PCIE_REG_IMR);
    // MD: Handle each set interrupt bit
    for_each_set_bit(i, &val, 32)
        generic_handle_domain_irq(port->cpm_domain, i);
    // MD: Clear handled interrupts
    pcie_write(port, val, XILINX_CPM_PCIE_REG_IDR);

    // MD: Handle variant-specific interrupt status
    if (variant->ir_status) {
        val = readl_relaxed(port->cpm_base + variant->ir_status);
        if (val)
            writel_relaxed(val, port->cpm_base + variant->ir_status);
    }

    /*
     * XILINX_CPM_PCIE_MISC_IR_STATUS register is mapped to
     * CPM SLCR block.
     */
    val = readl_relaxed(port->cpm_base + XILINX_CPM_PCIE_MISC_IR_STATUS);
    if (val)
        writel_relaxed(val,
                       port->cpm_base + XILINX_CPM_PCIE_MISC_IR_STATUS);

    // MD: Exit the chained IRQ handler
    chained_irq_exit(chip, desc);
}

#define _IC(x, s)                              \
    [XILINX_PCIE_INTR_##x] = { __stringify(x), s }

// MD: Define the interrupt cause mapping
static const struct {
    const char      *sym;  // MD: Symbolic name of the interrupt
    const char      *str;  // MD: Descriptive string of the interrupt
} intr_cause[32] = {
    _IC(LINK_DOWN,       "Link Down"),
    _IC(HOT_RESET,       "Hot reset"),
    _IC(CFG_TIMEOUT,     "ECAM access timeout"),
    _IC(CORRECTABLE,     "Correctable error message"),
    _IC(NONFATAL,        "Non fatal error message"),
    _IC(FATAL,           "Fatal error message"),
    _IC(SLV_UNSUPP,      "Slave unsupported request"),
    _IC(SLV_UNEXP,       "Slave unexpected completion"),
    _IC(SLV_COMPL,       "Slave completion timeout"),
    _IC(SLV_ERRP,        "Slave Error Poison"),
    _IC(SLV_CMPABT,      "Slave Completer Abort"),
    _IC(SLV_ILLBUR,      "Slave Illegal Burst"),
    _IC(MST_DECERR,      "Master decode error"),
    _IC(MST_SLVERR,      "Master slave error"),
    _IC(CFG_PCIE_TIMEOUT,"PCIe ECAM access timeout"),
    _IC(CFG_ERR_POISON,  "ECAM poisoned completion received"),
    _IC(PME_TO_ACK_RCVD, "PME_TO_ACK message received"),
    _IC(PM_PME_RCVD,     "PM_PME message received"),
    _IC(SLV_PCIE_TIMEOUT,"PCIe completion timeout received"),
};

/* MD:
 * xilinx_cpm_pcie_intr_handler - Handle PCIe interrupts
 * @irq: Interrupt number
 * @dev_id: Device identifier
 *
 * Return: IRQ_HANDLED after processing the interrupt
 */
static irqreturn_t xilinx_cpm_pcie_intr_handler(int irq, void *dev_id)
{
    struct xilinx_cpm_pcie *port = dev_id;
    struct device *dev = port->dev;
    struct irq_data *d;

    // MD: Get the IRQ data for the given interrupt
    d = irq_domain_get_irq_data(port->cpm_domain, irq);

    // MD: Handle specific interrupts
    switch (d->hwirq) {
    case XILINX_PCIE_INTR_CORRECTABLE:
    case XILINX_PCIE_INTR_NONFATAL:
    case XILINX_PCIE_INTR_FATAL:
        // MD: Clear error interrupts
        cpm_pcie_clear_err_interrupts(port);
        fallthrough;

    default:
        // MD: Log the interrupt cause
        if (intr_cause[d->hwirq].str)
            dev_warn(dev, "%s\n", intr_cause[d->hwirq].str);
        else
            dev_warn(dev, "Unknown IRQ %ld\n", d->hwirq);
    }

    return IRQ_HANDLED;
}

/* MD:
 * xilinx_cpm_free_irq_domains - Free IRQ domains
 * @port: PCIe port information
 */
static void xilinx_cpm_free_irq_domains(struct xilinx_cpm_pcie *port)
{
    // MD: Remove the INTx domain if it exists
    if (port->intx_domain) {
        irq_domain_remove(port->intx_domain);
        port->intx_domain = NULL;
    }

    // MD: Remove the CPM domain if it exists
    if (port->cpm_domain) {
        irq_domain_remove(port->cpm_domain);
        port->cpm_domain = NULL;
    }
}

/* MD:
 * xilinx_cpm_pcie_init_irq_domain - Initialize IRQ domain
 * @port: PCIe port information
 *
 * Return: '0' on success and error value on failure
 */
static int xilinx_cpm_pcie_init_irq_domain(struct xilinx_cpm_pcie *port)
{
    struct device *dev = port->dev;
    struct device_node *node = dev->of_node;
    struct device_node *pcie_intc_node;

    // MD: Setup INTx
    pcie_intc_node = of_get_next_child(node, NULL);
    if (!pcie_intc_node) {
        dev_err(dev, "No PCIe Intc node found\n");
        return -EINVAL;
    }

    // MD: Add a linear IRQ domain for CPM
    port->cpm_domain = irq_domain_add_linear(pcie_intc_node, 32,
                                             &event_domain_ops,
                                             port);
    if (!port->cpm_domain)
        goto out;

    // MD: Update the bus token for the CPM domain
    irq_domain_update_bus_token(port->cpm_domain, DOMAIN_BUS_NEXUS);

    // MD: Add a linear IRQ domain for INTx
    port->intx_domain = irq_domain_add_linear(pcie_intc_node, PCI_NUM_INTX,
                                              &intx_domain_ops,
                                              port);
    if (!port->intx_domain)
        goto out;

    // MD: Update the bus token for the INTx domain
    irq_domain_update_bus_token(port->intx_domain, DOMAIN_BUS_WIRED);

    of_node_put(pcie_intc_node);
    raw_spin_lock_init(&port->lock);

    return 0;
out:
    xilinx_cpm_free_irq_domains(port);
    of_node_put(pcie_intc_node);
    dev_err(dev, "Failed to allocate IRQ domains\n");

    return -ENOMEM;
}

/* MD:
 * xilinx_cpm_setup_irq - Setup IRQs for the PCIe port
 * @port: PCIe port information
 *
 * Return: '0' on success and error value on failure
 */
static int xilinx_cpm_setup_irq(struct xilinx_cpm_pcie *port)
{
    struct device *dev = port->dev;
    struct platform_device *pdev = to_platform_device(dev);
    int i, irq;

    // MD: Get the platform IRQ
    port->irq = platform_get_irq(pdev, 0);
    if (port->irq < 0)
        return port->irq;

    // MD: Iterate over each interrupt cause
    for (i = 0; i < ARRAY_SIZE(intr_cause); i++) {
        int err;

        // MD: Skip if no description is available
        if (!intr_cause[i].str)
            continue;

        // MD: Create a mapping for the interrupt
        irq = irq_create_mapping(port->cpm_domain, i);
        if (!irq) {
            dev_err(dev, "Failed to map interrupt\n");
            return -ENXIO;
        }

        // MD: Request the IRQ
        err = devm_request_irq(dev, irq, xilinx_cpm_pcie_intr_handler,
                               0, intr_cause[i].sym, port);
        if (err) {
            dev_err(dev, "Failed to request IRQ %d\n", irq);
            return err;
        }
    }

    // MD: Create a mapping for the INTx interrupt
    port->intx_irq = irq_create_mapping(port->cpm_domain,
                                        XILINX_PCIE_INTR_INTX);
    if (!port->intx_irq) {
        dev_err(dev, "Failed to map INTx interrupt\n");
        return -ENXIO;
    }

    // MD: Plug the INTx chained handler
    irq_set_chained_handler_and_data(port->intx_irq,
                                     xilinx_cpm_pcie_intx_flow, port);

    // MD: Plug the main event chained handler
    irq_set_chained_handler_and_data(port->irq,
                                     xilinx_cpm_pcie_event_flow, port);

    return 0;
}

/* MD:
 * xilinx_cpm_pcie_init_port - Initialize hardware
 * @port: PCIe port information
 */
static void xilinx_cpm_pcie_init_port(struct xilinx_cpm_pcie *port)
{
    const struct xilinx_cpm_variant *variant = port->variant;

    // MD: Check if the PCIe link is up
    if (cpm_pcie_link_up(port))
        dev_info(port->dev, "PCIe Link is UP\n");
    else
        dev_info(port->dev, "PCIe Link is DOWN\n");

    // MD: Disable all interrupts
    pcie_write(port, ~XILINX_CPM_PCIE_IDR_ALL_MASK,
               XILINX_CPM_PCIE_REG_IMR);

    // MD: Clear pending interrupts
    pcie_write(port, pcie_read(port, XILINX_CPM_PCIE_REG_IDR) &
               XILINX_CPM_PCIE_IMR_ALL_MASK,
               XILINX_CPM_PCIE_REG_IDR);

    /*
     * XILINX_CPM_PCIE_MISC_IR_ENABLE register is mapped to
     * CPM SLCR block.
     */
    writel(variant->ir_misc_value,
           port->cpm_base + XILINX_CPM_PCIE_MISC_IR_ENABLE);

    if (variant->ir_enable) {
        writel(XILINX_CPM_PCIE_IR_LOCAL,
               port->cpm_base + variant->ir_enable);
    }

    // MD: Set Bridge enable bit
    pcie_write(port, pcie_read(port, XILINX_CPM_PCIE_REG_RPSC) |
               XILINX_CPM_PCIE_REG_RPSC_BEN,
               XILINX_CPM_PCIE_REG_RPSC);
}

/* MD:
 * xilinx_cpm_pcie_parse_dt - Parse Device tree
 * @port: PCIe port information
 * @bus_range: Bus resource
 *
 * Return: '0' on success and error value on failure
 */
static int xilinx_cpm_pcie_parse_dt(struct xilinx_cpm_pcie *port,
                                    struct resource *bus_range)
{
    struct device *dev = port->dev;
    struct platform_device *pdev = to_platform_device(dev);
    struct resource *res;

    // MD: Map the CPM SLCR base address
    port->cpm_base = devm_platform_ioremap_resource_byname(pdev,
                                                           "cpm_slcr");
    if (IS_ERR(port->cpm_base))
        return PTR_ERR(port->cpm_base);

    // MD: Get the configuration space resource
    res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "cfg");
    if (!res)
        return -ENXIO;

    // MD: Create the PCI ECAM configuration window
    port->cfg = pci_ecam_create(dev, res, bus_range,
                                &pci_generic_ecam_ops);
    if (IS_ERR(port->cfg))
        return PTR_ERR(port->cfg);

    // MD: Map the CPM CSR base address for CPM5 variant
    if (port->variant->version == CPM5) {
        port->reg_base = devm_platform_ioremap_resource_byname(pdev,
                                                                "cpm_csr");
        if (IS_ERR(port->reg_base))
            return PTR_ERR(port->reg_base);
    } else {
        port->reg_base = port->cfg->win;
    }

    return 0;
}

/* MD:
 * xilinx_cpm_free_interrupts - Free interrupts for the PCIe port
 * @port: PCIe port information
 */
static void xilinx_cpm_free_interrupts(struct xilinx_cpm_pcie *port)
{
    // MD: Clear the chained handler for INTx
    irq_set_chained_handler_and_data(port->intx_irq, NULL, NULL);
    // MD: Clear the chained handler for main event
    irq_set_chained_handler_and_data(port->irq, NULL, NULL);
}

/* MD:
 * xilinx_cpm_pcie_probe - Probe function
 * @pdev: Platform device pointer
 *
 * Return: '0' on success and error value on failure
 */
static int xilinx_cpm_pcie_probe(struct platform_device *pdev)
{
    struct xilinx_cpm_pcie *port;
    struct device *dev = &pdev->dev;
    struct pci_host_bridge *bridge;
    struct resource_entry *bus;
    int err;

    // MD: Allocate memory for the PCI host bridge
    bridge = devm_pci_alloc_host_bridge(dev, sizeof(*port));
    if (!bridge)
        return -ENODEV;

    // MD: Get the private data for the PCI host bridge
    port = pci_host_bridge_priv(bridge);

    port->dev = dev;

    // MD: Initialize the IRQ domain
    err = xilinx_cpm_pcie_init_irq_domain(port);
    if (err)
        return err;

    // MD: Get the first bus resource
    bus = resource_list_first_type(&bridge->windows, IORESOURCE_BUS);
    if (!bus)
        return -ENODEV;

    // MD: Get the variant data from the device tree
    port->variant = of_device_get_match_data(dev);

    // MD: Parse the device tree
    err = xilinx_cpm_pcie_parse_dt(port, bus->res);
    if (err) {
        dev_err(dev, "Parsing DT failed\n");
        goto err_parse_dt;
    }

    // MD: Initialize the PCIe port
    xilinx_cpm_pcie_init_port(port);

    // MD: Setup the IRQs
    err = xilinx_cpm_setup_irq(port);
    if (err) {
        dev_err(dev, "Failed to set up interrupts\n");
        goto err_setup_irq;
    }

    // MD: Set the system data and operations for the PCI host bridge
    bridge->sysdata = port->cfg;
    bridge->ops = (struct pci_ops *)&pci_generic_ecam_ops.pci_ops;

    // MD: Probe the PCI host bridge
    err = pci_host_probe(bridge);
    if (err < 0)
        goto err_host_bridge;

    return 0;

err_host_bridge:
    xilinx_cpm_free_interrupts(port);
err_setup_irq:
    pci_ecam_free(port->cfg);
err_parse_dt:
    xilinx_cpm_free_irq_domains(port);
    return err;
}

// MD: Define the CPM variant for the host
static const struct xilinx_cpm_variant cpm_host = {
    .version = CPM,
    .ir_misc_value = XILINX_CPM_PCIE0_MISC_IR_LOCAL,
};

// MD: Define the CPM5 variant for the host
static const struct xilinx_cpm_variant cpm5_host = {
    .version = CPM5,
    .ir_misc_value = XILINX_CPM_PCIE0_MISC_IR_LOCAL,
    .ir_status = XILINX_CPM_PCIE0_IR_STATUS,
    .ir_enable = XILINX_CPM_PCIE0_IR_ENABLE,
};

// MD: Define the CPM5 Host1 variant
static const struct xilinx_cpm_variant cpm5_host1 = {
    .version = CPM5_HOST1,
    .ir_misc_value = XILINX_CPM_PCIE1_MISC_IR_LOCAL,
    .ir_status = XILINX_CPM_PCIE1_IR_STATUS,
    .ir_enable = XILINX_CPM_PCIE1_IR_ENABLE,
};

// MD: Device tree match table
static const struct of_device_id xilinx_cpm_pcie_of_match[] = {
    {
        .compatible = "xlnx,versal-cpm-host-1.00",
        .data = &cpm_host,
    },
    {
        .compatible = "xlnx,versal-cpm5-host",
        .data = &cpm5_host,
    },
    {
        .compatible = "xlnx,versal-cpm5-host1",
        .data = &cpm5_host1,
    },
    {}
};

// MD: Define the platform driver for the Xilinx CPM PCIe
static struct platform_driver xilinx_cpm_pcie_driver = {
    .driver = {
        .name = "xilinx-cpm-pcie",
        .of_match_table = xilinx_cpm_pcie_of_match,
        .suppress_bind_attrs = true,
    },
    .probe = xilinx_cpm_pcie_probe,
};

// MD: Register the platform driver
builtin_platform_driver(xilinx_cpm_pcie_driver);
