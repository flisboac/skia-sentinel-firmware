#define SC16IS7XX_UART_COMMON_API
#include "uart_sc16is7xx_common.h"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(nxp_sc16is7xx_uart_controller_common, CONFIG_UART_LOG_LEVEL);

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
    uint32_t desired_baud_rate,
)
{
    const struct sc16is7xx_uart_config* config = dev->config;
    struct sc16is7xx_uart_data* data = dev->data;
    struct sc16is7xx_uart_baud_rate_settings settings;
    bool sleep_mode_active = config->enable_sleep_mode;
    uint8_t channel_id = data->bridge_info.channel_id;
    uint8_t reg_addr;
    uint8_t reg_data;
    uint8_t lcr_data;
    uint8_t ier_data;
    uint8_t efr_data;
    int err = 0;

    sc16is7xx_uart_calculate_best_baud_rate_UNSAFE_(dev, desired_baud_rate, &settings);

    BUS_READ_BYTE(err, dev, bus, LCR, &lcr_data) else return -EIO;

    // EFR register is only available if `LCR == 0xBF`.
    BUS_WRITE_BYTE(err, dev, bus, LCR, 0xBF) else return -EIO;
    BUS_READ_BYTE(err, dev, bus, EFR, &efr_data) else return -EIO;

    // EFR is necessary to enable enhanced functions mode, to then
    // access the clock prescaler and sleep mode flags.
    reg_data = SC16IS7XX_BITSET(efr_data, SC16IS7XX_REGFLD_EFR_ENHANCEDBIT);
    BUS_WRITE_BYTE(err, dev, bus, EFR, reg_data) else return -EIO;

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
    BUS_WRITE_BYTE(err, dev, bus, DHL, reg_data) else return -EIO;

    //
    // Restore previous data (including sleep mode)
    //

    if (sleep_mode_active) {  //
        BUS_WRITE_BYTE(err, dev, bus, IER, ier_data) else return -EIO;
    }

    // EFR register is only available if `LCR == 0xBF`.
    BUS_WRITE_BYTE(err, dev, bus, LCR, 0xBF) else return -EIO;
    BUS_WRITE_BYTE(err, dev, bus, EFR, efr_data) else return -EIO;

    BUS_WRITE_BYTE(err, dev, bus, LCR, lcr_data) else return -EIO;

    //
    // Done!
    //

    data->runtime_config.baudrate = settings.actual_value;
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
    struct sc16is7xx_uart_baud_rate_settings settings;
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

    switch (settings->parity) {
    case UART_CFG_PARITY_NONE: reg_data = SC16IS7XX_BITCLEAR(reg_data, SC16IS7XX_REGFLD_LCR_PARITYENABLEBIT); break;
    case UART_CFG_PARITY_ODD:
        reg_data = SC16IS7XX_BITSET(reg_data, SC16IS7XX_REGFLD_LCR_PARITYENABLEBIT);
        reg_data = SC16IS7XX_BITCLEAR(reg_data, SC16IS7XX_REGFLD_LCR_PARITYTYPEBIT);
        reg_data = SC16IS7XX_BITCLEAR(reg_data, SC16IS7XX_REGFLD_LCR_PARITYFORCEBIT);
        break;
    case UART_CFG_PARITY_EVEN:
        reg_data = SC16IS7XX_BITSET(reg_data, SC16IS7XX_REGFLD_LCR_PARITYENABLEBIT);
        reg_data = SC16IS7XX_BITSET(reg_data, SC16IS7XX_REGFLD_LCR_PARITYTYPEBIT);
        reg_data = SC16IS7XX_BITCLEAR(reg_data, SC16IS7XX_REGFLD_LCR_PARITYFORCEBIT);
        break;
    case UART_CFG_PARITY_MARK:
        reg_data = SC16IS7XX_BITSET(reg_data, SC16IS7XX_REGFLD_LCR_PARITYENABLEBIT);
        reg_data = SC16IS7XX_BITCLEAR(reg_data, SC16IS7XX_REGFLD_LCR_PARITYTYPEBIT);
        reg_data = SC16IS7XX_BITSET(reg_data, SC16IS7XX_REGFLD_LCR_PARITYFORCEBIT);
        break;
    case UART_CFG_PARITY_SPACE:
        reg_data = SC16IS7XX_BITSET(reg_data, SC16IS7XX_REGFLD_LCR_PARITYENABLEBIT);
        reg_data = SC16IS7XX_BITSET(reg_data, SC16IS7XX_REGFLD_LCR_PARITYTYPEBIT);
        reg_data = SC16IS7XX_BITSET(reg_data, SC16IS7XX_REGFLD_LCR_PARITYFORCEBIT);
        break;
    default:  //
        LOG_DBG("Device '%s': Invalid parity: %d", dev->name, settings->parity);
        return -EINVAL;
    }

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

    data->settings.parity = settings->parity;
    data->settings.stop_bits = settings->stop_bits;
    data->settings.data_bits = settings->data_bits;

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
    uint8_t channel_id = data->bridge_info.channel_id;
    uint8_t reg_addr;
    uint8_t mcr_data;  // RS232/RS485, DTR
    uint8_t efr_data;  // RS232, RTS/CTS
    uint8_t efcr_data;  // RS485, RTS
    uint8_t lcr_data;
    int err = 0;

    BUS_READ_BYTE(err, dev, bus, LCR, &lcr_data) else return -EIO;
    BUS_READ_BYTE(err, dev, bus, MCR, &mcr_data) else return -EIO;

    // As per datasheet, the EFR register is only available if `LCR == 0xBF`
    BUS_WRITE_BYTE(err, dev, bus, LCR, 0xBF) else return -EIO;
    BUS_READ_BYTE(err, dev, bus, EFR, &efr_data) else return -EIO;
    BUS_READ_BYTE(err, dev, bus, EFCR, &efcr_data) else return -EIO;

    efr_data = SC16IS7XX_BITCLEAR(efr_data, SC16IS7XX_REGFLD_EFR_CTSFLOW);
    efr_data = SC16IS7XX_BITCLEAR(efr_data, SC16IS7XX_REGFLD_EFR_RTSFLOW);
    efcr_data = SC16IS7XX_BITCLEAR(efcr_data, SC16IS7XX_REGFLD_EFCR_RTSCONTROL);
    mcr_data = SC16IS7XX_BITCLEAR(efcr_data, SC16IS7XX_REGFLD_MCR_TCRTLRENABLE);

    switch (flow_ctrl) {
    case UART_CFG_FLOW_CTRL_NONE: break;
    case UART_CFG_FLOW_CTRL_RTS_CTS:
    case UART_CFG_FLOW_CTRL_DTR_DSR:
        if (config->operation_mode != SC16IS7XX_UART_OPMODE_RS232) {
            LOG_DBG(
                "Device '%s': Invalid flow control mode '%d' for current operation mode '%d'",
                dev->name,
                flow_ctrl,
                config->operation_mode
            );
            return -EINVAL;
        }
        efr_data = SC16IS7XX_BITSET(efr_data, SC16IS7XX_REGFLD_EFR_CTSFLOW);
        efr_data = SC16IS7XX_BITSET(efr_data, SC16IS7XX_REGFLD_EFR_RTSFLOW);
        mcr_data = SC16IS7XX_BITSET(efcr_data, SC16IS7XX_REGFLD_MCR_TCRTLRENABLE);
        writing_tcr = true;

        if (flow_ctrl == UART_CFG_FLOW_CTRL_DTR_DSR) {
            if (!config->modem_flow_control) {
                LOG_DBG(
                    "Device '%s': Invalid flow control mode '%d': `modem-flow-control` must be enabled",
                    dev->name,
                    flow_ctrl
                );
                return -EINVAL;
            }
            mcr_data = SC16IS7XX_BITSET(efcr_data, SC16IS7XX_REGFLD_MCR_DTR);
        }
        break;
    case UART_CFG_FLOW_CTRL_RS485:
        if (config->operation_mode != SC16IS7XX_UART_OPMODE_RS485) {
            LOG_DBG(
                "Device '%s': Invalid flow control mode '%d' for current operation mode '%d'",
                dev->name,
                flow_ctrl,
                config->operation_mode
            );
            return -EINVAL;
        }
        efcr_data = SC16IS7XX_BITSET(efcr_data, SC16IS7XX_REGFLD_EFCR_RTSCONTROL);
        mcr_data = SC16IS7XX_BITSET(efcr_data, SC16IS7XX_REGFLD_MCR_TCRTLRENABLE);
        writing_tcr = true;
        break;
    default:  //
        LOG_DBG("Device '%s': Invalid flow control mode: %d", dev->name, flow_ctrl);
        return -EINVAL;
    }

    BUS_WRITE_BYTE(err, dev, bus, MCR, mcr_data) else return -EIO;

    if (writing_tcr) {
        BUS_WRITE_BYTE(err, dev, bus, TCR, TCR_REG_VALUE(config)) else return -EIO;
        BUS_WRITE_BYTE(err, dev, bus, TLR, TLR_REG_VALUE(config)) else return -EIO;
    }

    BUS_WRITE_BYTE(err, dev, bus, EFCR, efcr_data) else return -EIO;
    BUS_WRITE_BYTE(err, dev, bus, EFR, efr_data) else return -EIO;
    BUS_WRITE_BYTE(err, dev, bus, LCR, lcr_data) else return -EIO;

    data->settings.flow_ctrl = flow_ctrl;

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

    err = (SC16IS7XX_BITCHECK(reg_value, SC16IS7XX_REGFLD_LSR_BREAK) ? UART_BREAK : 0)
        | (SC16IS7XX_BITCHECK(reg_value, SC16IS7XX_REGFLD_LSR_FRAMING) ? UART_ERROR_FRAMING : 0)
        | (SC16IS7XX_BITCHECK(reg_value, SC16IS7XX_REGFLD_LSR_PARITY) ? UART_ERROR_PARITY : 0)
        | (SC16IS7XX_BITCHECK(reg_value, SC16IS7XX_REGFLD_LSR_OVERRUN) ? UART_ERROR_OVERRUN : 0);

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
        LOG_DBG("Device '%s': Invalid line control flags/signals in '%d'", dev->name, ctrl);
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
            SC16IS7XX_BITSET(reg_data, SC16IS7XX_REGFLD_MCR_RTS);
        } else {
            SC16IS7XX_BITCLEAR(reg_data, SC16IS7XX_REGFLD_MCR_RTS);
        }
    }

    if (ctrl & UART_LINE_CTRL_DTR) {
        if (val) {
            SC16IS7XX_BITSET(reg_data, SC16IS7XX_REGFLD_MCR_DTR);
        } else {
            SC16IS7XX_BITCLEAR(reg_data, SC16IS7XX_REGFLD_MCR_DTR);
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
