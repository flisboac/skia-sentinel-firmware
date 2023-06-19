#include "uart_sc16is7xx_common.h"
#include "uart_sc16is7xx_poll.h"

#include <zephyr/logging/log.h>

#ifdef CONFIG_UART_SC16IS7XX_INTERRUPT_DRIVEN
#define CONFIG_UART_SC16IS7XX_INTERRUPT_DRIVEN_ENABLED 1
#else
#define CONFIG_UART_SC16IS7XX_INTERRUPT_DRIVEN_ENABLED 0
#endif

LOG_MODULE_REGISTER(nxp_sc16is7xx_uart_controller, CONFIG_UART_LOG_LEVEL);

static void sc16is7xx_uart_handle_interrupt(  //
    const struct sc16is7xx_bridge_info* bridge_info,
    const struct sc16is7xx_interrupt_info* interrupt_info,
    uint8_t bridge_index
)
{
    uint8_t interrupt_id = SC16IS7XX_GETVALUE(interrupt_info->iir, SC16IS7XX_REGFLD_IIR_ID);

    if (interrupt_id == SC16IS7XX_REGVAL_IIR_ID_GPIO) {
        // Nothing to do
        return;
    }

    struct sc16is7xx_uart_data* const data = CONTAINER_OF(bridge_info, struct sc16is7xx_uart_data, bridge_info);
    const struct device* const dev = data->own_instance;
    const struct sc16is7xx_uart_config* const config = dev->config;
    struct sc16is7xx_bus_lock bus_lock;
    bool bus_locked = false;
    int err = 0;

    SC16IS7XX_BUS_LOCK_INIT(bus_lock);

    err = sc16is7xx_lock_bus(config->parent_dev, &bus_lock, K_FOREVER);
    if (err) {
        LOG_DBG("Device '%s': Could not lock device bus access!", dev->name);
        return;
    }

    bus_locked = true;

    // TODO Data processing

    err = sc16is7xx_unlock_bus(config->parent_dev, &bus_lock);
    if (err) { LOG_DBG("Device '%s': Could not unlock device bus access! Error code = %d", dev->name, err); }
    bus_locked = false;

    // TODO fire callbacks

end:
    if (err) { LOG_DBG("Device '%s': Failed to read interrupt sources: %d", dev->name, err); }
    if (bus_locked && !sc16is7xx_unlock_bus(config->parent_dev, &bus_lock)) {
        LOG_DBG("Device '%s': Failed to unlock device bus access!", dev->name);
    }
}

static int sc16is7xx_uart_validate_config(const struct device* dev)
{
    const struct sc16is7xx_uart_config* const config = dev->config;
    struct sc16is7xx_uart_data* const data = dev->data;
    int err = 0, ext_err = 0;

    if ((config->rx_fifo_halt_level % 4) != 0) {
        LOG_DBG(
            "Device '%s': `rx_fifo_halt_level` must be a multiple of 4; got: rx_fifo_halt_level = %d",
            dev->name,
            config->rx_fifo_halt_level
        );
        err = -EINVAL;
    }

    if ((config->rx_fifo_resume_level % 4) != 0) {
        LOG_DBG(
            "Device '%s': `rx_fifo_resume_level` must be a multiple of 4; got: rx_fifo_resume_level = %d",
            dev->name,
            config->rx_fifo_resume_level
        );
        err = -EINVAL;
    }

    if ((config->tx_fifo_trigger_level % 4) != 0) {
        LOG_DBG(
            "Device '%s': `tx_fifo_trigger` must be a multiple of 4; got: tx_fifo_trigger = %d",
            dev->name,
            config->tx_fifo_trigger_level
        );
        err = -EINVAL;
    }

    if ((config->rx_fifo_trigger_level % 4) != 0) {
        LOG_DBG(
            "Device '%s': `rx_fifo_trigger` must be a multiple of 4; got: rx_fifo_trigger = %d",
            dev->name,
            config->rx_fifo_trigger_level
        );
        err = -EINVAL;
    }

    if (config->enable_fifo && config->tx_fifo_trigger_level == 0) {
        LOG_DBG(
            "Device '%s': `tx_fifo_trigger_level` cannot be zero when `enable_fifo` is activated",
            dev->name
        );
        err = -EINVAL;
    }

    if (config->enable_fifo && config->rx_fifo_trigger_level == 0) {
        LOG_DBG(
            "Device '%s': `rx_fifo_trigger_level` cannot be zero when `enable_fifo` is activated",
            dev->name
        );
        err = -EINVAL;
    }

    if (config->rx_fifo_halt_level <= config->rx_fifo_resume_level) {
        LOG_DBG(
            "Device '%s': `rx_fifo_halt_level` must be greater than `rx_fifo_resume_level`; got: rx_fifo_halt_level = "
            "%d, rx_fifo_resume_level = %d",
            dev->name,
            config->rx_fifo_halt_level,
            config->rx_fifo_resume_level
        );
        err = -EINVAL;
    }

    ext_err = sc16is7xx_uart_validate_baud_rate(  //
        dev,
        config->initial_config.baudrate,
        config->initial_config.baudrate
    );

    if (ext_err && !err) { err = ext_err; }

    return err;
}

static int sc16is7xx_uart_init(const struct device* dev)
{
    const struct sc16is7xx_uart_config* const config = dev->config;
    struct sc16is7xx_uart_data* const data = dev->data;
    const struct device* const parent_dev = config->parent_dev;
    struct sc16is7xx_bus_lock bus_lock;
    uint8_t reg_addr;
    uint8_t reg_data;
    uint8_t efr_data;
    uint8_t mcr_data;
    uint8_t fcr_data;
    uint8_t efcr_data;
    uint8_t ier_data;
    int err;

    SC16IS7XX_BUS_LOCK_INIT(bus_lock);

    if (!parent_dev) {
        err = -ENODEV;
        LOG_ERR("Device '%s': Could not get/find parent MFD instance! Error code = %d", dev->name, err);
        return err;
    }

    if (!device_is_ready(parent_dev)) {
        err = -ENODEV;
        LOG_ERR("Device '%s': Parent MFD instance was not initialized! Error code = %d", dev->name, err);
        return err;
    }

    data->own_instance = dev;

    data->device_info = sc16is7xx_get_device_info(parent_dev);
    if (!data->device_info) {
        err = -EINVAL;
        LOG_ERR(
            "Device '%s': Could not get device info from parent MFD! Error code = %d, parent = %s",
            dev->name,
            err,
            parent_dev->name
        );
        return err;
    }

    err = sc16is7xx_register_bridge(parent_dev, &data->bridge_info);
    if (err) {
        LOG_ERR("Device '%s': Could not register bridge in parent MFD! Error code = %d", dev->name, err);
        return err;
    }

    err = sc16is7xx_uart_validate_config(dev);
    if (err) { return err; }

    err = sc16is7xx_lock_bus(config->parent_dev, &bus_lock, K_FOREVER);
    if (err) {
        LOG_ERR("Device '%s': Could not lock device bus access! Error code = %d", dev->name, err);
        return err;
    }

    // Data locking (data->lock) is not really needed, as we're in the device initialization phase.

    fcr_data = 0;
    mcr_data = 0;
    efr_data = SC16IS7XX_BITSET(0, SC16IS7XX_REGFLD_EFR_ENHANCEDBIT);
    efcr_data = 0;

    // As per datasheet, EFR, XON* and XOFF* registers are only available if `LCR == 0xBF`
    BUS_WRITE_BYTE_ELSE_GOTO(end, -EIO, err, dev, bus_lock.bus, LCR, 0xBF);

    // As per datasheet, some registers are only available (e.g. TCR, TLR) when `EFR[4] = 1`
    BUS_WRITE_BYTE_ELSE_GOTO(end, -EIO, err, dev, bus_lock.bus, EFR, efr_data);

    if (config->hw_transmit_loopback) {
        mcr_data = SC16IS7XX_BITSET(mcr_data, SC16IS7XX_REGFLD_MCR_LOOPBACK);
    }

    if (config->enable_fifo) {
        mcr_data = SC16IS7XX_BITSET(mcr_data, SC16IS7XX_REGFLD_MCR_TCRTLRENABLE);
    }

    if (config->irda_transceiver) {
        mcr_data = SC16IS7XX_BITSET(mcr_data, SC16IS7XX_REGFLD_MCR_IRDAMODE);
        switch (config->irda_pulse_width) {
        case SC16IS7XX_UART_IRDAPULSE_AUTO:
        case SC16IS7XX_UART_IRDAPULSE_3_16:
            data->irda_pulse_width = SC16IS7XX_UART_IRDAPULSE_3_16;
            efcr_data = SC16IS7XX_BITCLEAR(efcr_data, SC16IS7XX_REGFLD_EFCR_IRDAMODE);
            break;
        case SC16IS7XX_UART_IRDAPULSE_1_4:
            data->irda_pulse_width = SC16IS7XX_UART_IRDAPULSE_1_4;
            efcr_data = SC16IS7XX_BITSET(efcr_data, SC16IS7XX_REGFLD_EFCR_IRDAMODE);
            break;
        }
    }

    BUS_WRITE_BYTE_ELSE_GOTO(end, -EIO, err, dev, bus_lock.bus, MCR, mcr_data);

    if (config->enable_fifo) {
        fcr_data = SC16IS7XX_BITSET(fcr_data, SC16IS7XX_REGFLD_FCR_ENABLE);
    }

    BUS_WRITE_BYTE_ELSE_GOTO(end, -EIO, err, dev, bus_lock.bus, FCR, fcr_data);

    if (config->enable_fifo) {
        // TLR can only be written if both FIFO and enhanced functions are enabled
        BUS_WRITE_BYTE_ELSE_GOTO(end, -EIO, err, dev, bus_lock.bus, TLR, TLR_REG_VALUE(config));
    }


#if defined(CONFIG_UART_SC16IS7XX_INTERRUPT_DRIVEN) && defined(CONFIG_MFD_SC16IS7XX_INTERRUPT_TRIGGER_GPIO)
    data->ier_enabled = 0;
    data->ier_disabled = 0;

    data->ier_enabled = SC16IS7XX_BITSET(data->ier_enabled, SC16IS7XX_REGFLD_IER_MODEM);
    data->ier_enabled = SC16IS7XX_BITSET(data->ier_enabled, SC16IS7XX_REGFLD_IER_RXLINE);
    // data->ier_enabled = SC16IS7XX_BITSET(data->ier_enabled, SC16IS7XX_REGFLD_IER_THR);
    data->ier_enabled = SC16IS7XX_BITSET(data->ier_enabled, SC16IS7XX_REGFLD_IER_RHR);

    if (config->enable_sleep_mode) {
        data->ier_enabled = SC16IS7XX_BITSET(data->ier_enabled, SC16IS7XX_REGFLD_IER_SLEEP);
        data->ier_disabled = SC16IS7XX_BITSET(data->ier_disabled, SC16IS7XX_REGFLD_IER_SLEEP);
    }
#else
    ier_data = 0;

    if (config->enable_sleep_mode) {
        ier_data = SC16IS7XX_BITSET(ier_data, SC16IS7XX_REGFLD_IER_SLEEP);
    }
#endif /* CONFIG_UART_SC16IS7XX_INTERRUPT_DRIVEN */


    if (config->disable_rx) {
        efcr_data = SC16IS7XX_BITSET(efcr_data, SC16IS7XX_REGFLD_EFCR_RXDISABLE);
    }

    if (config->disable_tx) {
        efcr_data = SC16IS7XX_BITSET(efcr_data, SC16IS7XX_REGFLD_EFCR_TXDISABLE);
    }

    BUS_WRITE_BYTE_ELSE_GOTO(end, -EIO, err, dev, bus_lock.bus, EFCR, efcr_data);

    err = sc16is7xx_uart_set_zephyr_config_UNSAFE_(dev, bus_lock.bus, &config->initial_config);
    if (err) { goto end; }

#ifdef CONFIG_UART_SC16IS7XX_INTERRUPT_DRIVEN
    data->interrupts_enabled = true;
    BUS_WRITE_BYTE_ELSE_GOTO(end, -EIO, err, dev, bus_lock.bus, IER, data->ier_enabled);
#else
    BUS_WRITE_BYTE_ELSE_GOTO(end, -EIO, err, dev, bus_lock.bus, IER, ier_data);
#endif

end:
    if (sc16is7xx_unlock_bus(config->parent_dev, &bus_lock)) {
        LOG_ERR("Device '%s': Could not unlock device bus access!", dev->name);
    }

    return err;
}

static const struct uart_driver_api sc16is7xx_uart_driver_api = {
    .poll_in = sc16is7xx_uart_poll_in,
    .poll_out = sc16is7xx_uart_poll_out,
    .err_check = sc16is7xx_uart_err_check,
#ifdef CONFIG_UART_SC16IS7XX_USE_RUNTIME_CONFIGURE
    .configure = sc16is7xx_uart_configure,
    .config_get = sc16is7xx_uart_config_get,
#endif
#ifdef CONFIG_UART_SC16IS7XX_ASYNC_API
// .callback_set = sc16is7xx_uart_callback_set,
// .tx = sc16is7xx_uart_tx,
// .tx_abort = sc16is7xx_uart_tx_abort,
// .rx_enable = sc16is7xx_uart_tx_enable,
// .rx_buf_rsp = sc16is7xx_uart_rx_buf_rsp,
// .rx_disable = sc16is7xx_uart_rx_disable,
#endif
#ifdef CONFIG_UART_SC16IS7XX_INTERRUPT_DRIVEN
// .fifo_fill = sc16is7xx_fifo_fill,
// .fifo_read = sc16is7xx_fifo_read,
// .irq_tx_enable = sc16is7xx_irq_tx_enable,
// .irq_tx_disable = sc16is7xx_irq_tx_disable,
// .irq_tx_ready = sc16is7xx_irq_tx_ready,
// .irq_tx_complete = sc16is7xx_irq_tx_complete,
// .irq_rx_enable = sc16is7xx_irq_rx_enable,
// .irq_rx_disable = sc16is7xx_irq_rx_disable,
// .irq_rx_ready = sc16is7xx_irq_rx_ready,
// .irq_err_enable = sc16is7xx_irq_err_enable,
// .irq_err_disable = sc16is7xx_irq_err_disable,
// .irq_is_pending = sc16is7xx_irq_is_pending,
// .irq_update = sc16is7xx_irq_update,
// .irq_callback_set = sc16is7xx_irq_callback_set,
#endif
#ifdef CONFIG_UART_SC16IS7XX_LINE_CTRL
    .line_ctrl_get = sc16is7xx_uart_line_ctrl_get,
    .line_ctrl_set = sc16is7xx_uart_line_ctrl_set,
#endif
};

#define SC16IS7XX_CONFIG_COMMON_PROPS(inst, pn_suffix) \
    .parent_dev = DEVICE_DT_GET(DT_INST_PARENT(inst)), \
    .baud_clock_prescaler = DT_INST_PROP_OR(inst, baud_clock_prescaler, 0), \
    .device_address = DT_INST_PROP_OR(inst, device_address, -1), \
    .operation_mode = DT_INST_ENUM_IDX(inst, operation_mode), \
    .flow_control_mode = DT_INST_ENUM_IDX(inst, flow_control_mode), \
    .sw_flow_mode = DT_INST_ENUM_IDX(inst, sw_flow_mode), \
    .xon = { \
        DT_INST_PROP_LEN_OR(inst, xon, 0), \
        COND_CODE_1(DT_INST_PROP_HAS_IDX(inst, xon, 0), (DT_INST_PROP_BY_IDX(inst, xon, 0)), (0)), \
        COND_CODE_1(DT_INST_PROP_HAS_IDX(inst, xon, 1), (DT_INST_PROP_BY_IDX(inst, xon, 1)), (0)), \
    }, \
    .xoff = { \
        DT_INST_PROP_LEN_OR(inst, xoff, 0), \
        COND_CODE_1(DT_INST_PROP_HAS_IDX(inst, xoff, 0), (DT_INST_PROP_BY_IDX(inst, xoff, 0)), (0)), \
        COND_CODE_1(DT_INST_PROP_HAS_IDX(inst, xoff, 1), (DT_INST_PROP_BY_IDX(inst, xoff, 1)), (0)), \
    }, \
    .tx_fifo_trigger_level = DT_INST_PROP(inst, tx_fifo_trigger), \
    .rx_fifo_trigger_level = DT_INST_PROP(inst, rx_fifo_trigger), \
    .rx_fifo_halt_level = DT_INST_PROP(inst, rx_flow_halt_trigger), \
    .rx_fifo_resume_level = DT_INST_PROP(inst, rx_flow_resume_trigger), \
    .enable_sleep_mode = DT_INST_PROP_OR(inst, enable_sleep_mode, false), \
    .disable_tx = DT_INST_PROP_OR(inst, disable_tx, false), \
    .disable_rx = DT_INST_PROP_OR(inst, disable_rx, false), \
    .hw_flow_control = DT_INST_PROP_OR(inst, hw_flow_control, false), \
    .modem_flow_control = DT_INST_PROP_OR(inst, modem_flow_control, false), \
    .irda_transceiver = DT_INST_PROP_OR(inst, irda_transceiver, false), \
    .supports_modem_flow_control = DT_INST_PROP_OR(inst, supports_modem_flow_control, false), \
    .supports_irda_fast_speed = DT_INST_PROP_OR(inst, supports_irda_fast_speed, false), \
    .xon_any_mode = DT_INST_ENUM_IDX(inst, xon_any_mode), \
    .irda_pulse_width = DT_INST_ENUM_IDX_OR(inst, irda_pulse_width, 1), \
    .hw_transmit_loopback = DT_INST_PROP_OR(inst, hw_transmit_loopback, false), \
    .enable_xon_any = DT_INST_PROP_OR(inst, enable_xon_any, false), \
    .enable_fifo = DT_INST_PROP_OR(inst, enable_fifo, false) || DT_INST_ENUM_IDX(inst, flow_control_mode) == SC16IS7XX_UART_FLOWCTRL_AUTO, \
    .detect_special_character = (\
        DT_INST_ENUM_IDX(inst, operation_mode) == SC16IS7XX_UART_OPMODE_RS485 && \
        DT_INST_NODE_HAS_PROP(inst, device_address) \
        ) || DT_INST_PROP_OR(inst, detect_special_character, false), \
    .rs485_invert_rts = DT_INST_ENUM_IDX(inst, operation_mode) == SC16IS7XX_UART_OPMODE_RS485 && \
        DT_INST_PROP_OR(inst, rs485_invert_rts, false), \
    .initial_config.baudrate = DT_INST_PROP(inst, current_speed), \
    .initial_config.parity = DT_INST_ENUM_IDX_OR(inst, parity, 0), \
    .initial_config.stop_bits = DT_INST_ENUM_IDX_OR(inst, stop_bits, 0) + 1, \
    .initial_config.data_bits = DT_INST_PROP_OR(inst, data_bits, 8), \
    .initial_config.flow_ctrl = \
        DT_INST_ENUM_IDX(inst, flow_control_mode) == SC16IS7XX_UART_FLOWCTRL_USER ? UART_CFG_FLOW_CTRL_NONE \
        : DT_INST_ENUM_IDX(inst, operation_mode) == SC16IS7XX_UART_OPMODE_RS485   ? UART_CFG_FLOW_CTRL_RS485 \
        : DT_INST_PROP_OR(inst, modem_flow_control, false)                        ? UART_CFG_FLOW_CTRL_DTR_DSR \
        : UART_CFG_FLOW_CTRL_RTS_CTS

#define SC16IS7XX_DEFINE(inst, pn_suffix) \
    static struct sc16is7xx_uart_data sc16is##pn_suffix##_uart_data_##inst = { \
        .lock = Z_SEM_INITIALIZER(sc16is##pn_suffix##_uart_data_##inst.lock, 1, 1), \
        .irda_pulse_width = DT_INST_ENUM_IDX_OR(inst, irda_pulse_width, 1), \
        .bridge_info.kind = SC16IS7XX_BRIDGE_UART, \
        .bridge_info.channel_id = DT_INST_PROP(inst, channel), \
        .bridge_info.on_interrupt = CONFIG_UART_SC16IS7XX_INTERRUPT_DRIVEN_ENABLED ? sc16is7xx_uart_handle_interrupt : NULL \
    }; \
    static const struct sc16is7xx_uart_config sc16is##pn_suffix##_uart_config_##inst = { \
        SC16IS7XX_CONFIG_COMMON_PROPS(inst, pn_suffix) \
    }; \
    DEVICE_DT_INST_DEFINE( \
        inst, \
        sc16is7xx_uart_init, \
        NULL, \
        &sc16is##pn_suffix##_uart_data_##inst, \
        &sc16is##pn_suffix##_uart_config_##inst, \
        POST_KERNEL, \
        CONFIG_GPIO_SC16IS7XX_INIT_PRIORITY, \
        &sc16is7xx_uart_driver_api \
    );

#define SC16IS740_INIT(inst) SC16IS7XX_DEFINE(inst, 740)
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT nxp_sc16is740_uart_controller
DT_INST_FOREACH_STATUS_OKAY(SC16IS740_INIT)

#define SC16IS750_INIT(inst) SC16IS7XX_DEFINE(inst, 750)
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT nxp_sc16is750_uart_controller
DT_INST_FOREACH_STATUS_OKAY(SC16IS750_INIT)

#define SC16IS760_INIT(inst) SC16IS7XX_DEFINE(inst, 760)
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT nxp_sc16is760_uart_controller
DT_INST_FOREACH_STATUS_OKAY(SC16IS760_INIT)

#define SC16IS752_INIT(inst) SC16IS7XX_DEFINE(inst, 752)
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT nxp_sc16is752_uart_controller
DT_INST_FOREACH_STATUS_OKAY(SC16IS752_INIT)

#define SC16IS762_INIT(inst) SC16IS7XX_DEFINE(inst, 762)
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT nxp_sc16is762_uart_controller
DT_INST_FOREACH_STATUS_OKAY(SC16IS762_INIT)
