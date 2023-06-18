#define SC16IS7XX_UART_COMMON_API
#include "uart_sc16is7xx_common.h"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(nxp_sc16is7xx_uart_controller_common, CONFIG_UART_LOG_LEVEL);

int sc16is7xx_uart_validate_baud_rate(const struct device* dev, uint32_t target_baud_rate, uint32_t actual_baud_rate)
{
    const struct sc16is7xx_uart_config* const config = dev->config;
    struct sc16is7xx_uart_data* const data = dev->data;

    if (target_baud_rate != actual_baud_rate) {
        LOG_WRN(
            "Device '%s': actual baud rate = '%s' is off by '%d' units, compared to target baud rate '%s'",
            dev->name,
            actual_baud_rate,
            actual_baud_rate - target_baud_rate,
            target_baud_rate
        );
    }

    if (config->irda_transceiver) {
        switch (target_baud_rate) {
        case 9600:
        case 19200:
        case 38400:
        case 57600:
        case 115200:
            if (config->irda_pulse_width == SC16IS7XX_UART_IRDAPULSE_1_4) {
                LOG_WRN(
                    "Device '%s': `irda_pulse_width` is not properly configured for standard speeds",
                    dev->name
                );
            }
            break;
        case 576000:
        case 1152000:
            switch (config->irda_pulse_width) {
            case SC16IS7XX_UART_IRDAPULSE_AUTO:
            case SC16IS7XX_UART_IRDAPULSE_1_4: //
                break;
            case SC16IS7XX_UART_IRDAPULSE_3_16:
                LOG_WRN(
                    "Device '%s': `irda_pulse_width` is not properly configured for fast speeds",
                    dev->name
                );
                break;
            }
            break;
        default:
            LOG_WRN(
                "Device '%s': non-standard speed selected for IrDA transceiver mode; communication may become unstable",
                dev->name
            );
            break;
        }
    }

    return 0;
}

static inline void sc16is7xx_uart_calculate_baud_rate_UNSAFE_(  //
    int32_t desired_baud_rate,
    int32_t xtal_freq,
    int8_t prescaler,
    struct sc16is7xx_uart_baud_rate_settings* settings
)
{
    settings->prescaler = prescaler;
    settings->target_value = desired_baud_rate;
    settings->divider = DIV_NEAREST(DIV_NEAREST(xtal_freq, prescaler), (desired_baud_rate * 16));
    settings->actual_value = DIV_NEAREST(DIV_NEAREST(xtal_freq, (settings->divider * prescaler)), 16);
    settings->abs_error_diff = ABS(settings->actual_value - settings->target_value);
}

static inline void sc16is7xx_uart_calculate_best_baud_rate_UNSAFE_(  //
    const struct device* dev,
    int32_t desired_baud_rate,
    struct sc16is7xx_uart_baud_rate_settings* settings
)
{
    struct sc16is7xx_uart_data* data = dev->data;
    const struct sc16is7xx_uart_config* config = dev->config;
    struct sc16is7xx_uart_baud_rate_settings settings_1;
    struct sc16is7xx_uart_baud_rate_settings settings_4;

    if (config->baud_clock_prescaler) {
        sc16is7xx_uart_calculate_baud_rate_UNSAFE_(  //
            desired_baud_rate,
            data->device_info->xtal_freq,
            config->baud_clock_prescaler,
            settings
        );
        return;
    }

    sc16is7xx_uart_calculate_baud_rate_UNSAFE_(desired_baud_rate, data->device_info->xtal_freq, 1, &settings_1);
    sc16is7xx_uart_calculate_baud_rate_UNSAFE_(desired_baud_rate, data->device_info->xtal_freq, 4, &settings_4);

    if (settings_4.abs_error_diff < settings_1.abs_error_diff) {
        memcpy(settings, &settings_4, sizeof(struct sc16is7xx_uart_baud_rate_settings));
    } else {
        memcpy(settings, &settings_1, sizeof(struct sc16is7xx_uart_baud_rate_settings));
    }
}

static int sc16is7xx_uart_set_baud_rate_UNSAFE_(  //
    const struct device* dev,
    const struct sc16is7xx_bus* bus,
    uint32_t desired_baud_rate
)
{
    const struct sc16is7xx_uart_config* config = dev->config;
    struct sc16is7xx_uart_data* data = dev->data;
    struct sc16is7xx_uart_baud_rate_settings settings;
    enum sc16is7xx_uart_irda_pulse_width irda_pulse_width = data->irda_pulse_width;
    bool sleep_mode_active = config->enable_sleep_mode;
    uint8_t channel_id = data->bridge_info.channel_id;
    uint8_t reg_addr;
    uint8_t reg_data;
    uint8_t lcr_data;
    uint8_t ier_data;
    uint8_t efcr_data;
    int err = 0;

    sc16is7xx_uart_calculate_best_baud_rate_UNSAFE_(dev, desired_baud_rate, &settings);
    sc16is7xx_uart_validate_baud_rate(dev, settings.target_value, settings.actual_value);

    LOG_DBG(
        "Device '%s': Setting baud rate: target_value = %d, actual_value = %d, xtal_freq = %d, divider = %d, prescaler "
        "= %d",
        dev->name,
        settings.target_value,
        settings.actual_value,
        data->device_info->xtal_freq,
        settings.divider,
        settings.prescaler
    );

    BUS_READ_BYTE(err, dev, bus, LCR, &lcr_data) else return -EIO;
    BUS_READ_BYTE(err, dev, bus, EFCR, &efcr_data) else return -EIO;

    // EFR register is only available if `LCR == 0xBF`.
    BUS_WRITE_BYTE(err, dev, bus, LCR, 0xBF) else return -EIO;

    // EFR is necessary to enable enhanced functions mode, to then
    // access the clock prescaler and sleep mode flags.
    // EFR[4] is already set during device initialization.

    //
    // Set prescaler
    //

    BUS_READ_BYTE(err, dev, bus, MCR, &reg_data) else return -EIO;
    reg_data = settings.prescaler == 4 ? SC16IS7XX_BITSET(reg_data, SC16IS7XX_REGFLD_MCR_CLOCKDIVISOR)
                                       : SC16IS7XX_BITCLEAR(reg_data, SC16IS7XX_REGFLD_MCR_CLOCKDIVISOR);
    BUS_WRITE_BYTE(err, dev, bus, MCR, reg_data) else return -EIO;

    //
    // Disable sleep mode (if enabled).
    // This is needed before changing the clock divisor, as per Application Note AN10571.
    //

    if (sleep_mode_active) {
        BUS_READ_BYTE(err, dev, bus, IER, &ier_data) else return -EIO;

        if (SC16IS7XX_BITCHECK(ier_data, SC16IS7XX_REGFLD_IER_SLEEP)) {
            reg_data = SC16IS7XX_BITCLEAR(ier_data, SC16IS7XX_REGFLD_IER_SLEEP);
            BUS_WRITE_BYTE(err, dev, bus, IER, reg_data) else return -EIO;
        } else {
            sleep_mode_active = false;
        }
    }

    //
    // Set clock divisor
    //

    // As per the datasheet, to change the clock divisor, `LCR != 0xBF`.
    // Just assign anything, so that we can change the clock divisor.
    reg_data = SC16IS7XX_BITSET(0U, SC16IS7XX_REGFLD_LCR_DIVISORENABLE);
    BUS_WRITE_BYTE(err, dev, bus, LCR, reg_data) else return -EIO;

    reg_data = settings.divider & 0xff;
    BUS_WRITE_BYTE(err, dev, bus, DLL, reg_data) else return -EIO;

    reg_data = (settings.divider >> 8) & 0xff;
    BUS_WRITE_BYTE(err, dev, bus, DLH, reg_data) else return -EIO;

    //
    // Restore previous data (including sleep mode)
    //

    if (sleep_mode_active) {  //
        BUS_WRITE_BYTE(err, dev, bus, IER, ier_data) else return -EIO;
    }

    if (config->irda_transceiver && config->irda_pulse_width == SC16IS7XX_UART_IRDAPULSE_AUTO) {
        if (settings.actual_value > 115200 && config->supports_irda_fast_speed) {
            irda_pulse_width = SC16IS7XX_UART_IRDAPULSE_1_4;
            efcr_data = SC16IS7XX_BITSET(efcr_data, SC16IS7XX_REGFLD_EFCR_IRDAMODE);
        } else {
            irda_pulse_width = SC16IS7XX_UART_IRDAPULSE_3_16;
            efcr_data = SC16IS7XX_BITCLEAR(efcr_data, SC16IS7XX_REGFLD_EFCR_IRDAMODE);
        }
    }

    BUS_WRITE_BYTE(err, dev, bus, EFCR, efcr_data) else return -EIO;
    BUS_WRITE_BYTE(err, dev, bus, LCR, lcr_data) else return -EIO;

    //
    // Done!
    //

    data->runtime_config.baudrate = settings.actual_value;
    data->irda_pulse_width = irda_pulse_width;

    return 0;
}

static inline int sc16is7xx_uart_reg_set_parity_UNSAFE_(  //
    const struct device* dev,
    uint8_t* reg_data,
    uint8_t parity
)
{
    switch (parity) {
    case UART_CFG_PARITY_NONE: *reg_data = SC16IS7XX_BITCLEAR(*reg_data, SC16IS7XX_REGFLD_LCR_PARITYENABLEBIT); break;
    case UART_CFG_PARITY_ODD:
        *reg_data = SC16IS7XX_BITSET(*reg_data, SC16IS7XX_REGFLD_LCR_PARITYENABLEBIT);
        *reg_data = SC16IS7XX_BITCLEAR(*reg_data, SC16IS7XX_REGFLD_LCR_PARITYTYPEBIT);
        *reg_data = SC16IS7XX_BITCLEAR(*reg_data, SC16IS7XX_REGFLD_LCR_PARITYFORCEBIT);
        break;
    case UART_CFG_PARITY_EVEN:
        *reg_data = SC16IS7XX_BITSET(*reg_data, SC16IS7XX_REGFLD_LCR_PARITYENABLEBIT);
        *reg_data = SC16IS7XX_BITSET(*reg_data, SC16IS7XX_REGFLD_LCR_PARITYTYPEBIT);
        *reg_data = SC16IS7XX_BITCLEAR(*reg_data, SC16IS7XX_REGFLD_LCR_PARITYFORCEBIT);
        break;
    case UART_CFG_PARITY_MARK:
        *reg_data = SC16IS7XX_BITSET(*reg_data, SC16IS7XX_REGFLD_LCR_PARITYENABLEBIT);
        *reg_data = SC16IS7XX_BITCLEAR(*reg_data, SC16IS7XX_REGFLD_LCR_PARITYTYPEBIT);
        *reg_data = SC16IS7XX_BITSET(*reg_data, SC16IS7XX_REGFLD_LCR_PARITYFORCEBIT);
        break;
    case UART_CFG_PARITY_SPACE:
        *reg_data = SC16IS7XX_BITSET(*reg_data, SC16IS7XX_REGFLD_LCR_PARITYENABLEBIT);
        *reg_data = SC16IS7XX_BITSET(*reg_data, SC16IS7XX_REGFLD_LCR_PARITYTYPEBIT);
        *reg_data = SC16IS7XX_BITSET(*reg_data, SC16IS7XX_REGFLD_LCR_PARITYFORCEBIT);
        break;
    default:  //
        LOG_DBG("Device '%s': Invalid parity: %d", dev->name, parity);
        return -EINVAL;
    }

    return 0;
}

static int sc16is7xx_uart_set_line_control_UNSAFE_(  //
    const struct device* dev,
    const struct sc16is7xx_bus* bus,
    const struct uart_config* settings
)
{
    const struct sc16is7xx_uart_config* config = dev->config;
    struct sc16is7xx_uart_data* data = dev->data;
    uint8_t channel_id = data->bridge_info.channel_id;
    uint8_t reg_addr;
    uint8_t reg_data;
    int err = 0;

    BUS_READ_BYTE(err, dev, bus, LCR, &reg_data) else return -EIO;

    switch (settings->data_bits) {
    case UART_CFG_DATA_BITS_5:
    case UART_CFG_DATA_BITS_6:
    case UART_CFG_DATA_BITS_7:
    case UART_CFG_DATA_BITS_8:
        reg_data = SC16IS7XX_SETVALUE(reg_data, settings->data_bits, SC16IS7XX_REGFLD_LCR_WORDLENGTH);
        break;
    default:  //
        LOG_DBG("Device '%s': Invalid data_bits: %d", dev->name, settings->data_bits);
        return -EINVAL;
    }

    err = sc16is7xx_uart_reg_set_parity_UNSAFE_(dev, &reg_data, settings->parity);
    if (err) { return err; }

    switch (settings->stop_bits) {
    case UART_CFG_STOP_BITS_1:  //
        reg_data = SC16IS7XX_BITSET(reg_data, SC16IS7XX_REGFLD_LCR_STOPBITS);
        break;
    case UART_CFG_STOP_BITS_1_5:
        if (settings->data_bits != 5) {
            LOG_DBG(
                "Device '%s': Invalid stop_bits = '%d' for data_bits = '%d'; the device does not support this "
                "configuration",
                dev->name,
                settings->stop_bits,
                settings->data_bits
            );
            return -EINVAL;
        }
        // !!passthrough!!
    case UART_CFG_STOP_BITS_2:
        switch (settings->data_bits) {
        case UART_CFG_DATA_BITS_6:
        case UART_CFG_DATA_BITS_7:
        case UART_CFG_DATA_BITS_8: break;
        default:
            LOG_DBG(
                "Device '%s': Invalid stop_bits = '%d' for data_bits = '%d'; the device does not support this "
                "configuration",
                dev->name,
                settings->stop_bits,
                settings->data_bits
            );
            return -EINVAL;
        }
        reg_data = SC16IS7XX_BITCLEAR(reg_data, SC16IS7XX_REGFLD_LCR_STOPBITS);
        break;
    default:  //
        LOG_DBG("Device '%s': Invalid stop_bits: %d", dev->name, settings->stop_bits);
        return -EINVAL;
    }

    BUS_WRITE_BYTE(err, dev, bus, LCR, reg_data) else return -EIO;

    data->runtime_config.parity = settings->parity;
    data->runtime_config.stop_bits = settings->stop_bits;
    data->runtime_config.data_bits = settings->data_bits;

    return 0;
}

static int sc16is7xx_uart_set_zephyr_flow_control_UNSAFE_(  //
    const struct device* dev,
    const struct sc16is7xx_bus* bus,
    uint8_t flow_ctrl
)
{
    const struct sc16is7xx_uart_config* config = dev->config;
    struct sc16is7xx_uart_data* data = dev->data;
    struct sc16is7xx_uart_baud_rate_settings settings;
    bool writing_tcr = false;
    bool writing_address = false;
    uint8_t channel_id = data->bridge_info.channel_id;
    uint8_t reg_addr;
    uint8_t mcr_data;  // RS232/RS485, DTR
    uint8_t efr_data;  // RS232, RTS/CTS
    uint8_t efcr_data;  // RS485, RTS
    uint8_t fcr_data; // to force-enable FIFO
    uint8_t lcr_data;
    int err = 0;

    switch (flow_ctrl) {
    case UART_CFG_FLOW_CTRL_NONE:
    case UART_CFG_FLOW_CTRL_RTS_CTS:
    case UART_CFG_FLOW_CTRL_RS485:  //
        break;
    case UART_CFG_FLOW_CTRL_DTR_DSR:
        if (!config->modem_flow_control) {
            LOG_DBG(
                "Device '%s': Invalid flow control mode '%d': cannot control DTR/DSR lines, `modem-flow-control` was "
                "not activated",
                dev->name,
                flow_ctrl
            );
            return -EINVAL;
        }
        break;
    default:  //
        LOG_DBG("Device '%s': Invalid flow control mode: %d", dev->name, flow_ctrl);
        return -EINVAL;
    }

    if (flow_ctrl != UART_CFG_FLOW_CTRL_NONE && !config->enable_fifo) {
        LOG_DBG(
            "Device '%s': Invalid flow control mode '%d': FIFO was not enabled for the device, automatic control flow "
            "won't work",
            dev->name,
            flow_ctrl
        );
        return -EINVAL;
    }

    // FIFO configuration was already set on device initialization.

    BUS_READ_BYTE(err, dev, bus, LCR, &lcr_data) else return -EIO;
    BUS_READ_BYTE(err, dev, bus, MCR, &mcr_data) else return -EIO;
    BUS_READ_BYTE(err, dev, bus, FCR, &fcr_data) else return -EIO;

    // As per datasheet, EFR, XON* and XOFF* registers are only available if `LCR == 0xBF`
    BUS_WRITE_BYTE(err, dev, bus, LCR, 0xBF) else return -EIO;
    BUS_READ_BYTE(err, dev, bus, EFR, &efr_data) else return -EIO;
    BUS_READ_BYTE(err, dev, bus, EFCR, &efcr_data) else return -EIO;

    // As per datasheet, some registers are only available (e.g. TCR, TLR) when `EFR[4] = 1`.
    // EFR[4] is already set during device initialization.

    // (Hopefully) the compiler will optimize this away.
    efr_data = SC16IS7XX_BITCLEAR(efr_data, SC16IS7XX_REGFLD_EFR_CTSFLOW);
    efr_data = SC16IS7XX_BITCLEAR(efr_data, SC16IS7XX_REGFLD_EFR_RTSFLOW);
    efr_data = SC16IS7XX_BITCLEAR(efr_data, SC16IS7XX_REGFLD_EFR_SPECIALCHAR);
    efr_data = SC16IS7XX_SETVALUE(efr_data, SC16IS7XX_REGVAL_EFR_SWFLOW_NONE, SC16IS7XX_REGFLD_EFR_SWFLOWMODE);
    efcr_data = SC16IS7XX_BITCLEAR(efcr_data, SC16IS7XX_REGFLD_EFCR_RTSCONTROL);
    efcr_data = SC16IS7XX_BITCLEAR(efcr_data, SC16IS7XX_REGFLD_EFCR_RTSINVERT);
    mcr_data = SC16IS7XX_BITCLEAR(mcr_data, SC16IS7XX_REGFLD_MCR_XONANY);
    fcr_data = SC16IS7XX_BITCLEAR(efr_data, SC16IS7XX_REGFLD_FCR_ENABLE);

    if (config->detect_special_character) {
        efr_data = SC16IS7XX_BITSET(efr_data, SC16IS7XX_REGFLD_EFR_SPECIALCHAR);
    }

    if (config->enable_xon_any) {
        if (config->xon_any_mode == SC16IS7XX_UART_XONANY_ALWAYS || flow_ctrl != UART_CFG_FLOW_CTRL_NONE) {
            mcr_data = SC16IS7XX_BITSET(mcr_data, SC16IS7XX_REGFLD_MCR_XONANY);
        }
    }

    if (config->enable_fifo) {
        fcr_data = SC16IS7XX_BITSET(efr_data, SC16IS7XX_REGFLD_FCR_ENABLE);
        mcr_data = SC16IS7XX_BITSET(mcr_data, SC16IS7XX_REGFLD_MCR_TCRTLRENABLE);
        writing_tcr = true;
    }

    switch (flow_ctrl) {
    case UART_CFG_FLOW_CTRL_NONE: //
        break;
    case UART_CFG_FLOW_CTRL_RTS_CTS:
    case UART_CFG_FLOW_CTRL_DTR_DSR:
        if (config->hw_flow_control) {
            efr_data = SC16IS7XX_BITSET(efr_data, SC16IS7XX_REGFLD_EFR_CTSFLOW);
            efr_data = SC16IS7XX_BITSET(efr_data, SC16IS7XX_REGFLD_EFR_RTSFLOW);
        } else {
            efr_data = SC16IS7XX_SETVALUE(efr_data, config->sw_flow_mode, SC16IS7XX_REGFLD_EFR_SWFLOWMODE);
        }
        fcr_data = SC16IS7XX_BITSET(efr_data, SC16IS7XX_REGFLD_FCR_ENABLE);
        mcr_data = SC16IS7XX_BITSET(mcr_data, SC16IS7XX_REGFLD_MCR_TCRTLRENABLE);
        writing_tcr = true;
        if (flow_ctrl == UART_CFG_FLOW_CTRL_DTR_DSR && config->modem_flow_control) {
            mcr_data = SC16IS7XX_BITSET(mcr_data, SC16IS7XX_REGFLD_MCR_DTR);
        }
        break;
    case UART_CFG_FLOW_CTRL_RS485:
        efcr_data = SC16IS7XX_BITSET(efcr_data, SC16IS7XX_REGFLD_EFCR_RXDISABLE);
        err = sc16is7xx_uart_reg_set_parity_UNSAFE_(dev, &lcr_data, UART_CFG_PARITY_SPACE);
        if (err) { return err; }
        if (config->device_address >= 0) {
            efr_data = SC16IS7XX_BITSET(efr_data, SC16IS7XX_REGFLD_EFR_SPECIALCHAR);
            writing_address = true;
        }
        if (config->hw_flow_control) {
            efcr_data = SC16IS7XX_BITSET(efcr_data, SC16IS7XX_REGFLD_EFCR_RTSCONTROL);
        } else {
            efr_data = SC16IS7XX_SETVALUE(efr_data, config->sw_flow_mode, SC16IS7XX_REGFLD_EFR_SWFLOWMODE);
        }
        if (config->rs485_invert_rts) {  //
            efcr_data = SC16IS7XX_BITSET(efcr_data, SC16IS7XX_REGFLD_EFCR_RTSINVERT);
        }
        fcr_data = SC16IS7XX_BITSET(efr_data, SC16IS7XX_REGFLD_FCR_ENABLE);
        mcr_data = SC16IS7XX_BITSET(mcr_data, SC16IS7XX_REGFLD_MCR_TCRTLRENABLE);
        writing_tcr = true;
        break;
    default:
        // Validation was already done previously
        break;
    }

    if (config->xon[0]) {
        BUS_WRITE_BYTE(err, dev, bus, XON1, config->xon[1]) else return -EIO;
        BUS_WRITE_BYTE(err, dev, bus, XON2, config->xon[2]) else return -EIO;
    }

    if (config->xoff[0]) {
        BUS_WRITE_BYTE(err, dev, bus, XOFF1, config->xoff[1]) else return -EIO;
        BUS_WRITE_BYTE(err, dev, bus, XOFF2, config->xoff[2]) else return -EIO;
    }

    if (writing_address) {  //
        BUS_WRITE_BYTE(err, dev, bus, XOFF2, config->device_address) else return -EIO;
    }

    BUS_WRITE_BYTE(err, dev, bus, FCR, fcr_data) else return -EIO;
    BUS_WRITE_BYTE(err, dev, bus, MCR, mcr_data) else return -EIO;

    // TCR and TLR are only available when `EFR[4] == 1 && MCR[2] == 1`, hence the conditional.
    if (writing_tcr) {
        BUS_WRITE_BYTE(err, dev, bus, TCR, TCR_REG_VALUE(config)) else return -EIO;
        // Writing TLR here is necessary because there can be times when TLR is not set
        // during device initialization (e.g. `enable_fifo == false`). FIFO is always
        // necessary for automatic flow control.
        BUS_WRITE_BYTE(err, dev, bus, TLR, TLR_REG_VALUE(config)) else return -EIO;
    }

    BUS_WRITE_BYTE(err, dev, bus, EFCR, efcr_data) else return -EIO;
    BUS_WRITE_BYTE(err, dev, bus, EFR, efr_data) else return -EIO;
    BUS_WRITE_BYTE(err, dev, bus, LCR, lcr_data) else return -EIO;

    data->runtime_config.flow_ctrl = flow_ctrl;

    return 0;
}

int sc16is7xx_uart_set_zephyr_config_UNSAFE_(  //
    const struct device* dev,
    const struct sc16is7xx_bus* bus,
    const struct uart_config* settings
)
{
    int err;

    err = sc16is7xx_uart_set_baud_rate_UNSAFE_(dev, bus, settings->baudrate);
    if (err) { return err; }

    err = sc16is7xx_uart_set_line_control_UNSAFE_(dev, bus, settings);
    if (err) { return err; }

    err = sc16is7xx_uart_set_zephyr_flow_control_UNSAFE_(dev, bus, settings->flow_ctrl);
    if (err) { return err; }

    return 0;
}

int sc16is7xx_uart_err_check(const struct device* dev)
{
    const struct sc16is7xx_uart_config* config = dev->config;
    const struct device* parent_dev = config->parent_dev;
    struct sc16is7xx_uart_data* data = dev->data;
    struct sc16is7xx_bus_lock bus_lock;
    uint8_t channel_id = data->bridge_info.channel_id;
    uint8_t reg_addr;
    uint8_t reg_data;
    int err = 0;

    SC16IS7XX_BUS_LOCK_INIT(bus_lock);

    err = sc16is7xx_lock_bus(parent_dev, &bus_lock, K_FOREVER);
    if (err) {
        LOG_DBG("Device '%s': Could not lock bus! Error code = %d", dev->name, err);
        return err;
    }

    BUS_READ_BYTE(err, dev, bus_lock.bus, LSR, &reg_data) else
    {
        err = -EIO;
        goto end;
    }

    err = (SC16IS7XX_BITCHECK(reg_data, SC16IS7XX_REGFLD_LSR_BREAK) ? UART_BREAK : 0)
        | (SC16IS7XX_BITCHECK(reg_data, SC16IS7XX_REGFLD_LSR_FRAMING) ? UART_ERROR_FRAMING : 0)
        | (SC16IS7XX_BITCHECK(reg_data, SC16IS7XX_REGFLD_LSR_PARITY) ? UART_ERROR_PARITY : 0)
        | (SC16IS7XX_BITCHECK(reg_data, SC16IS7XX_REGFLD_LSR_OVERRUN) ? UART_ERROR_OVERRUN : 0);

end:
    if (sc16is7xx_unlock_bus(parent_dev, &bus_lock)) {  //
        LOG_DBG("Device '%s': Could not unlock bus!", dev->name);
    }

    return err;
}

#ifdef CONFIG_UART_SC16IS7XX_USE_RUNTIME_CONFIGURE

int sc16is7xx_uart_configure(const struct device* dev, const struct uart_config* cfg)
{
    const struct sc16is7xx_uart_config* config = dev->config;
    const struct device* parent_dev = config->parent_dev;
    struct sc16is7xx_uart_data* data = dev->data;
    struct sc16is7xx_bus_lock bus_lock;
    bool data_locked = false;
    int err = 0;

    err = sc16is7xx_lock_bus(parent_dev, &bus_lock, K_FOREVER);
    if (err) {
        LOG_DBG("Device '%s': Could not lock bus! Error code = %d", dev->name, err);
        return err;
    }

    err = k_sem_take(&data->lock, K_FOREVER);
    if (err) {
        LOG_DBG("Device '%s': Could not lock data access! Error code = %d", dev->name, err);
        goto end;
    }
    data_locked = true;

    err = sc16is7xx_uart_set_zephyr_config_UNSAFE_(dev, bus_lock.bus, cfg);

end:
    if (data_locked) { k_sem_give(&data->lock); }
    if (sc16is7xx_unlock_bus(parent_dev, &bus_lock)) {  //
        LOG_DBG("Device '%s': Could not unlock bus!", dev->name);
    }

    return err;
}

int sc16is7xx_uart_config_get(const struct device* dev, struct uart_config* cfg)
{
    const struct sc16is7xx_uart_config* config = dev->config;
    const struct device* parent_dev = config->parent_dev;
    struct sc16is7xx_uart_data* data = dev->data;
    struct sc16is7xx_bus_lock bus_lock;
    int err = 0;

    err = k_sem_take(&data->lock, K_FOREVER);
    if (err) {
        LOG_DBG("Device '%s': Could not lock data access! Error code = %d", dev->name, err);
        return err;
    }

    memcpy(cfg, &data->runtime_config, sizeof(struct uart_config));

    k_sem_give(&data->lock);

    return 0;
}

#endif /* CONFIG_UART_SC16IS7XX_USE_RUNTIME_CONFIGURE */

#ifdef CONFIG_UART_SC16IS7XX_LINE_CTRL

int sc16is7xx_uart_line_ctrl_set(  //
    const struct device* dev,
    uint32_t ctrl,
    uint32_t val
)
{
    const struct sc16is7xx_uart_config* config = dev->config;
    const struct device* parent_dev = config->parent_dev;
    struct sc16is7xx_uart_data* data = dev->data;
    struct sc16is7xx_bus_lock bus_lock;
    bool data_locked = false;
    uint8_t channel_id = data->bridge_info.channel_id;
    uint8_t reg_addr;
    uint8_t reg_data;
    int err = 0;

    uint32_t allowed_flags =
        UART_LINE_CTRL_BAUD_RATE | UART_LINE_CTRL_RTS | (config->modem_flow_control ? UART_LINE_CTRL_DTR : 0);

    if (ctrl & ~allowed_flags) {
        LOG_DBG("Device '%s': Invalid line control flags/signals: '%d'", dev->name, ctrl);
        return -EINVAL;
    }

    err = sc16is7xx_lock_bus(parent_dev, &bus_lock, K_FOREVER);
    if (err) {
        LOG_DBG("Device '%s': Could not lock bus! Error code = %d", dev->name, err);
        return err;
    }

    BUS_READ_BYTE(err, dev, bus_lock.bus, MCR, &reg_data) else
    {
        err = -EIO;
        goto end;
    }

    if (ctrl & UART_LINE_CTRL_RTS) {
        if (val) {
            reg_data = SC16IS7XX_BITSET(reg_data, SC16IS7XX_REGFLD_MCR_RTS);
        } else {
            reg_data = SC16IS7XX_BITCLEAR(reg_data, SC16IS7XX_REGFLD_MCR_RTS);
        }
    }

    if (ctrl & UART_LINE_CTRL_DTR) {
        if (val) {
            reg_data = SC16IS7XX_BITSET(reg_data, SC16IS7XX_REGFLD_MCR_DTR);
        } else {
            reg_data = SC16IS7XX_BITCLEAR(reg_data, SC16IS7XX_REGFLD_MCR_DTR);
        }
    }

    BUS_WRITE_BYTE(err, dev, bus_lock.bus, MCR, reg_data) else
    {
        err = -EIO;
        goto end;
    }

    if (ctrl & UART_LINE_CTRL_BAUD_RATE) {
        err = k_sem_take(&data->lock, K_FOREVER);
        if (err) {
            LOG_DBG("Device '%s': Could not lock data access! Error code = %d", dev->name, err);
            goto end;
        }
        data_locked = true;
        sc16is7xx_uart_set_baud_rate_UNSAFE_(dev, bus_lock.bus, val);
    }

end:
    if (data_locked) { k_sem_give(&data->lock); }
    if (sc16is7xx_unlock_bus(parent_dev, &bus_lock)) {  //
        LOG_DBG("Device '%s': Could not unlock bus!", dev->name);
    }

    return err;
}

int sc16is7xx_uart_line_ctrl_get(  //
    const struct device* dev,
    uint32_t ctrl,
    uint32_t* val
)
{
    const struct sc16is7xx_uart_config* config = dev->config;
    const struct device* parent_dev = config->parent_dev;
    struct sc16is7xx_uart_data* data = dev->data;
    struct sc16is7xx_bus_lock bus_lock;
    bool data_locked = false;
    uint8_t channel_id = data->bridge_info.channel_id;
    uint8_t reg_addr;
    uint8_t reg_data;
    int err = 0;

    if (!val) {
        LOG_DBG("Device '%s': Missing value destination pointer `val`", dev->name);
        return -EINVAL;
    }

    if (ctrl & (UART_LINE_CTRL_DCD | UART_LINE_CTRL_DSR) && !config->modem_flow_control) {
        LOG_DBG(
            "Device '%s': Invalid line control flags/signals in '%d': `modem-control-flow` must be enabled",
            dev->name,
            ctrl
        );
        return -EINVAL;
    }

    switch (ctrl) {
    case UART_LINE_CTRL_BAUD_RATE:
        err = k_sem_take(&data->lock, K_FOREVER);
        if (err) {
            LOG_DBG("Device '%s': Could not lock data access! Error code = %d", dev->name, err);
            goto end;
        }
        data_locked = true;
        *val = data->runtime_config.baudrate;
        break;
    case UART_LINE_CTRL_RTS:
        BUS_READ_BYTE(err, dev, bus_lock.bus, MCR, &reg_data) else
        {
            err = -EIO;
            goto end;
        }
        *val = SC16IS7XX_BITCHECK(reg_data, SC16IS7XX_REGFLD_MCR_RTS);
        break;
    case UART_LINE_CTRL_DTR:
        BUS_READ_BYTE(err, dev, bus_lock.bus, MCR, &reg_data) else
        {
            err = -EIO;
            goto end;
        }
        *val = SC16IS7XX_BITCHECK(reg_data, SC16IS7XX_REGFLD_MCR_DTR);
        break;
    case UART_LINE_CTRL_DCD:
        BUS_READ_BYTE(err, dev, bus_lock.bus, MSR, &reg_data) else
        {
            err = -EIO;
            goto end;
        }
        *val = SC16IS7XX_BITCHECK(reg_data, SC16IS7XX_REGFLD_MSR_CD);
        break;
    case UART_LINE_CTRL_DSR:
        BUS_READ_BYTE(err, dev, bus_lock.bus, MSR, &reg_data) else
        {
            err = -EIO;
            goto end;
        }
        *val = SC16IS7XX_BITCHECK(reg_data, SC16IS7XX_REGFLD_MSR_DSR);
        break;
    default:  //
        LOG_DBG("Device '%s': Invalid line control flags/signal '%d'", dev->name, ctrl);
        return -EINVAL;
    }

end:
    if (data_locked) { k_sem_give(&data->lock); }
    if (sc16is7xx_unlock_bus(parent_dev, &bus_lock)) {  //
        LOG_DBG("Device '%s': Could not unlock bus!", dev->name);
    }

    return err;
}

#endif /* CONFIG_UART_SC16IS7XX_LINE_CTRL */
