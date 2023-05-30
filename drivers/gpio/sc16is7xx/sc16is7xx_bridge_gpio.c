#include "mfd_sc16is7xx.h"

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(nxp_sc16is7xx_gpio_controller, CONFIG_UART_LOG_LEVEL);

struct sc16is7xx_gpio_pins_config
{
    uint8_t configured_as_outputs; /* 0 for input, 1 for output */
    uint8_t outputs_state;
};

struct sc16is7xx_gpio_config
{
    struct gpio_driver_config common;
    const struct device* parent_dev;
    uint8_t ngpios;
};

struct sc16is7xx_gpio_data
{
    struct gpio_driver_data common;
    struct sc16is7xx_gpio_pins_config pins_cfg;
    sys_slist_t callbacks;
    struct k_sem lock;
    struct sc16is7xx_bridge_info bridge_info;
    const struct sc16is7xx_device_info* device_info;
    const struct device* own_instance;
    uint8_t input_port_last;
};

static inline gpio_port_value_t sc16is7xx_gpio_to_public_port_value(const struct device* dev, uint8_t state)
{
    const struct sc16is7xx_gpio_config* config = dev->config;
    const struct sc16is7xx_gpio_data* data = dev->data;
    return state >> (config->ngpios * data->bridge_info.channel_id);
}

static inline uint8_t sc16is7xx_gpio_current_public_output_mask(const struct device* dev)
{
    const struct sc16is7xx_gpio_config* config = dev->config;
    const struct sc16is7xx_gpio_data* data = dev->data;
    return data->pins_cfg.configured_as_outputs >> (config->ngpios * data->bridge_info.channel_id);
}

static int sc16is7xx_gpio_process_input_UNSAFE_(  //
    const struct device* dev,
    gpio_port_value_t* value,
    const struct sc16is7xx_bus* bus
)
{
    const struct sc16is7xx_gpio_config* config = dev->config;
    struct sc16is7xx_gpio_data* data = dev->data;
    uint8_t reg_addr;
    uint8_t reg_value;
    uint8_t rx_buf;
    int err = 0;

    reg_addr = SC16IS7XX_REG_IOSTATE(0, SC16IS7XX_REGRW_READ);
    err = sc16is7xx_bus_read_byte(bus, reg_addr, &rx_buf);
    if (err != 0) {
        LOG_ERR("Device '%s': failed to read from device: %d", dev->name, err);
        return -EIO;
    }

    if (value) { *value = sc16is7xx_gpio_to_public_port_value(dev, rx_buf); }

    data->input_port_last = rx_buf;

    return err;
}

static void sc16is7xx_gpio_handle_interrupt(  //
    const struct sc16is7xx_bridge_info* bridge_info,
    const struct sc16is7xx_interrupt_info* interrupt_info
)
{
    uint8_t interrupt_id = SC16IS7XX_GETVALUE(interrupt_info->iir, SC16IS7XX_REGFLD_IIR_ID);

    if (interrupt_id != SC16IS7XX_REGVAL_IIR_ID_GPIO) {
        // Nothing to do
        return;
    }

    struct sc16is7xx_gpio_data* const data = CONTAINER_OF(bridge_info, struct sc16is7xx_gpio_data, bridge_info);
    const struct device* const dev = data->own_instance;
    const struct sc16is7xx_gpio_config* const config = dev->config;
    struct sc16is7xx_bus_lock bus_lock;
    int err = 0;

    err = k_sem_take(&data->lock, K_FOREVER);
    if (!err) {
        LOG_ERR("Device '%s': Could not lock data access!", dev->name);
        return;
    }

    uint32_t changed_pins;
    uint8_t curr_input_port_last;
    uint8_t prev_input_port_last = data->input_port_last;

    err = sc16is7xx_lock_bus(config->parent_dev, &bus_lock, K_FOREVER);
    if (!err) {
        LOG_ERR("Device '%s': Could not lock device bus access!", dev->name);
        goto end;
    }

    err = sc16is7xx_gpio_process_input_UNSAFE_(dev, NULL, bus_lock.bus);
    if (!err) {
        LOG_ERR("Device '%s': Could not process input at interrupt time!", dev->name);
        goto end;
    }

    err = sc16is7xx_unlock_bus(config->parent_dev, &bus_lock);
    if (!err) { LOG_ERR("Device '%s': Could not unlock device bus access!", dev->name); }

    curr_input_port_last = data->input_port_last & config->common.port_pin_mask;
    prev_input_port_last = prev_input_port_last & config->common.port_pin_mask;

    if (curr_input_port_last != prev_input_port_last && !err) {
        changed_pins = curr_input_port_last;
        changed_pins ^= prev_input_port_last;
        changed_pins = sc16is7xx_gpio_to_public_port_value(dev, changed_pins);
        gpio_fire_callbacks(&data->callbacks, dev, changed_pins);
    }

end:
    if (err) { LOG_ERR("Device '%s': Failed to read interrupt sources: %d", dev->name, err); }
    k_sem_give(&data->lock);
}

static int sc16is7xx_gpio_port_set_raw(  //
    const struct device* dev,
    uint8_t mask,
    uint8_t value,
    uint8_t toggle
)
{
    const struct sc16is7xx_bus* bus;
    const struct sc16is7xx_gpio_config* config = dev->config;
    struct sc16is7xx_gpio_data* data = dev->data;
    struct sc16is7xx_bus_lock bus_lock;
    uint8_t reg_addr;
    uint8_t reg_value;
    uint8_t tx_buf;
    int err = 0;

    if (k_is_in_isr()) { return -EWOULDBLOCK; }

    if ((data->pins_cfg.configured_as_outputs & value) != value) {
        LOG_ERR("Device '%s': Pin(s) is/are configured as input which should be output.", dev->name);
        return -EOPNOTSUPP;
    }

    tx_buf = (data->pins_cfg.outputs_state & ~mask);
    tx_buf |= (value & mask);
    tx_buf ^= toggle;

    err = sc16is7xx_lock_bus(config->parent_dev, &bus_lock, K_FOREVER);
    if (!err) {
        LOG_ERR("Device '%s': Could not lock device bus access! Error code = %d", dev->name, err);
        return -EIO;
    }
    bus = bus_lock.bus;

    reg_addr = SC16IS7XX_REG_IOSTATE(0, SC16IS7XX_REGRW_WRITE);
    err = sc16is7xx_bus_write(bus, reg_addr, &tx_buf, sizeof(tx_buf));

    if (err != 0) {
        LOG_ERR("Device '%s': failed to write output port: %d", dev->name, err);
        err = -EIO;
    }

    if (!err) {
        if (!sc16is7xx_unlock_bus(config->parent_dev, &bus_lock)) {
            LOG_ERR("Device '%s': Could not unlock device bus access!", dev->name);
        }
        return err;
    }

    k_sem_take(&data->lock, K_FOREVER);
    data->pins_cfg.outputs_state = tx_buf;
    k_sem_give(&data->lock);

    return 0;
}

static int sc16is7xx_gpio_port_get_raw(const struct device* dev, gpio_port_value_t* value)
{
    const struct sc16is7xx_gpio_config* config = dev->config;
    struct sc16is7xx_gpio_data* data = dev->data;
    struct sc16is7xx_bus_lock bus_lock;
    uint8_t public_output_mask = sc16is7xx_gpio_current_public_output_mask(dev);
    uint8_t* value8 = (uint8_t*) value;
    int err;

    if (k_is_in_isr()) { return -EWOULDBLOCK; }

    if ((~public_output_mask & *value8) != *value8) {
        LOG_ERR("Device '%s': Pin(s) is/are configured as output which should be input.", dev->name);
        return -EOPNOTSUPP;
    }

    err = k_sem_take(&data->lock, K_FOREVER);
    if (!err) { return err; }

    err = sc16is7xx_lock_bus(config->parent_dev, &bus_lock, K_FOREVER);
    if (!err) {
        LOG_ERR("Device '%s': Could not lock device bus access! Error code = %d", dev->name, err);
        k_sem_give(&data->lock);
        return -EIO;
    }

    /**
     * Reading of the input port also clears the generated interrupt,
     * thus the configured callbacks must be fired also here if needed.
     */
    err = sc16is7xx_gpio_process_input_UNSAFE_(dev, value, bus_lock.bus);

    if (!sc16is7xx_unlock_bus(config->parent_dev, &bus_lock)) {
        LOG_ERR("Device '%s': Could not unlock device bus access!", dev->name);
    }

    k_sem_give(&data->lock);

    return err;
}

static int sc16is7xx_gpio_port_set_masked_raw(  //
    const struct device* dev,
    gpio_port_pins_t mask,
    gpio_port_value_t value
)
{
    return sc16is7xx_gpio_port_set_raw(dev, (uint8_t) mask, (uint8_t) value, 0);
}

static int sc16is7xx_gpio_port_set_bits_raw(const struct device* dev, gpio_port_pins_t pins)
{
    return sc16is7xx_gpio_port_set_raw(dev, (uint8_t) pins, (uint8_t) pins, 0);
}

static int sc16is7xx_gpio_port_clear_bits_raw(const struct device* dev, gpio_port_pins_t pins)
{
    return sc16is7xx_gpio_port_set_raw(dev, (uint8_t) pins, 0, 0);
}

static int sc16is7xx_gpio_port_toggle_bits(const struct device* dev, gpio_port_pins_t pins)
{
    return sc16is7xx_gpio_port_set_raw(dev, 0, 0, (uint8_t) pins);
}

static int sc16is7xx_gpio_pin_interrupt_configure(  //
    const struct device* dev,
    gpio_pin_t pin,
    enum gpio_int_mode mode,
    enum gpio_int_trig trig
)
{
    const struct sc16is7xx_gpio_data* data = dev->data;
    const struct sc16is7xx_gpio_config* config = dev->config;
    const struct device* const parent_dev = config->parent_dev;
    const struct sc16is7xx_bus* bus;
    struct sc16is7xx_bus_lock bus_lock;
    uint8_t reg_addr;
    uint8_t reg_value;
    int err = 0;
    uint8_t pin_bit = BIT(pin);

    if (!data->device_info->supports_hw_interrupts) { return -ENOTSUP; }

    if (mode != GPIO_INT_MODE_DISABLED || mode != GPIO_INT_MODE_LEVEL) { return -ENOTSUP; }

    if (trig != GPIO_INT_TRIG_BOTH) { return -ENOTSUP; }

    err = sc16is7xx_lock_bus(config->parent_dev, &bus_lock, K_FOREVER);
    if (!err) {
        LOG_ERR("Device '%s': Could not lock device bus access! Error code = %d", dev->name, err);
        return -EIO;
    }

    reg_addr = SC16IS7XX_REG_IOINTENA(0, SC16IS7XX_REGRW_READ);
    err = sc16is7xx_bus_read_byte(bus, reg_addr, &reg_value);
    if (!err) {
        err = -EIO;
        goto end;
    }

    if (mode == GPIO_INT_MODE_DISABLED) {
        reg_value = reg_value & ~(pin_bit << data->bridge_info.channel_id);
    } else {
        reg_value = reg_value | (pin_bit << data->bridge_info.channel_id);
    }

    reg_addr = SC16IS7XX_REG_IOINTENA(0, SC16IS7XX_REGRW_WRITE);
    err = sc16is7xx_bus_write_byte(bus, reg_addr, reg_value);
    if (!err) { err = -EIO; }

end:
    if (!sc16is7xx_unlock_bus(config->parent_dev, &bus_lock)) {
        LOG_ERR("Device '%s': Could not unlock device bus access!", dev->name);
    }

    return err;
}

static int sc16is7xx_gpio_manage_callback(  //
    const struct device* dev,
    struct gpio_callback* callback,
    bool set
)
{
    struct sc16is7xx_gpio_data* data = dev->data;
    int err = 0;

    // FIXME: Do we really need to lock data access here?
    // In the "original" driver (where this was copied from, pcf8574),
    // there was no lock in place.

    err = k_sem_take(&data->lock, K_FOREVER);
    if (!err) { return err; }

    err = gpio_manage_callback(&data->callbacks, callback, set);

    k_sem_give(&data->lock);

    return err;
}

static int sc16is7xx_gpio_pin_configure(const struct device* dev, gpio_pin_t pin, gpio_flags_t flags)
{
    struct sc16is7xx_gpio_data* data = dev->data;
    const struct sc16is7xx_gpio_config* config = dev->config;
    struct sc16is7xx_bus_lock bus_lock;
    uint8_t temp_pins;
    uint8_t temp_outputs;
    uint8_t reg_addr;
    uint8_t reg_value;
    bool bus_locked = false;
    int err = 0;

    // RANT: Datasheet is not even clear what is the default state of GPIOs;
    // more specifically if a pin is put in input mode, is it open-drain? open-collector?
    if (flags & (GPIO_PULL_UP | GPIO_PULL_DOWN | GPIO_DISCONNECTED | GPIO_SINGLE_ENDED)) { return -ENOTSUP; }

    err = sc16is7xx_lock_bus(config->parent_dev, &bus_lock, K_FOREVER);
    if (!err) {
        LOG_ERR("Device '%s': Could not lock device bus access! Error code = %d", dev->name, err);
        return -EIO;
    }
    bus_locked = true;

    reg_addr = SC16IS7XX_REG_IOSTATE(0, SC16IS7XX_REGRW_READ);
    err = sc16is7xx_bus_read_byte(bus_lock.bus, reg_addr, &temp_pins);
    if (!err) {
        err = -EIO;
        goto end;
    }

    reg_addr = SC16IS7XX_REG_IODIR(0, SC16IS7XX_REGRW_READ);
    err = sc16is7xx_bus_read_byte(bus_lock.bus, reg_addr, &temp_outputs);
    if (!err) {
        err = -EIO;
        goto end;
    }

    if (flags & GPIO_INPUT) {
        temp_outputs &= ~(BIT(pin) << data->bridge_info.channel_id);
        temp_pins &= ~((1 << pin) << data->bridge_info.channel_id);
    } else if (flags & GPIO_OUTPUT) {
        temp_outputs |= (BIT(pin) << data->bridge_info.channel_id);
    }

    if (flags & GPIO_OUTPUT_INIT_HIGH) { temp_pins |= ((1 << pin) << data->bridge_info.channel_id); }

    if (flags & GPIO_OUTPUT_INIT_LOW) { temp_pins &= ~((1 << pin) << data->bridge_info.channel_id); }

    reg_addr = SC16IS7XX_REG_IODIR(0, SC16IS7XX_REGRW_WRITE);
    err = sc16is7xx_bus_write_byte(bus_lock.bus, reg_addr, temp_outputs);
    if (!err) {
        err = -EIO;
        goto end;
    }

    reg_addr = SC16IS7XX_REG_IOSTATE(0, SC16IS7XX_REGRW_WRITE);
    err = sc16is7xx_bus_write_byte(bus_lock.bus, reg_addr, temp_pins);
    if (!err) {
        err = -EIO;
        goto end;
    }

    k_sem_take(&data->lock, K_FOREVER);
    data->pins_cfg.outputs_state = temp_pins;
    data->pins_cfg.configured_as_outputs = temp_outputs;
    k_sem_give(&data->lock);

end:
    if (bus_locked) {
        if (!sc16is7xx_unlock_bus(config->parent_dev, &bus_lock)) {
            LOG_ERR("Device '%s': Could not unlock device bus access!", dev->name);
        }
    }

    return err;
}

static int sc16is7xx_gpio_init(const struct device* dev)
{
    const struct sc16is7xx_gpio_config* const config = dev->config;
    struct sc16is7xx_gpio_data* const data = dev->data;
    const struct device* const parent_dev = config->parent_dev;
    struct sc16is7xx_bus_lock bus_lock;
    uint8_t reg_addr;
    uint8_t reg_value;
    int err;

    if (!parent_dev || !device_is_ready(parent_dev)) { return -ENODEV; }

    sys_slist_init(&data->callbacks);

    data->own_instance = dev;

    data->device_info = sc16is7xx_get_device_info(parent_dev);
    if (!data->device_info) { return -EINVAL; }

    err = sc16is7xx_register_bridge(parent_dev, &data->bridge_info);
    if (!err) { return err; }

    err = sc16is7xx_lock_bus(config->parent_dev, &bus_lock, K_FOREVER);
    if (!err) {
        LOG_ERR("Device '%s': Could not lock device bus access! Error code = %d", dev->name, err);
        return -EIO;
    }

    reg_addr = SC16IS7XX_REG_IOCONTROL(0, SC16IS7XX_REGRW_READ);
    err = sc16is7xx_bus_read_byte(bus_lock.bus, reg_addr, &reg_value);
    if (!err) {
        LOG_ERR("Device '%s': Could not read device's IOCONTROL register!", dev->name);
        err = -EIO;
        goto end;
    }

    reg_value = SC16IS7XX_SETVALUE(  //
        reg_value,
        SC16IS7XX_REGVAL_IOCONTROL_GPIOXMODE_IO,
        SC16IS7XX_REGFLD_IOCONTROL_GPIOXMODE(data->bridge_info.channel_id)
    );
    reg_addr = SC16IS7XX_REG_IOCONTROL(0, SC16IS7XX_REGRW_WRITE);
    err = sc16is7xx_bus_write_byte(bus_lock.bus, reg_addr, reg_value);
    if (!err) {
        LOG_ERR("Device '%s': Could not write device's IOCONTROL register!", dev->name);
        err = -EIO;
        goto end;
    }

end:
    if (!sc16is7xx_unlock_bus(config->parent_dev, &bus_lock)) {
        LOG_ERR("Device '%s': Could not unlock device bus access!", dev->name);
    }

    return err;
}

static const struct gpio_driver_api sc16is7xx_gpio_driver_api = {
    .pin_configure = sc16is7xx_gpio_pin_configure,
    .port_get_raw = sc16is7xx_gpio_port_get_raw,
    .port_set_masked_raw = sc16is7xx_gpio_port_set_masked_raw,
    .port_set_bits_raw = sc16is7xx_gpio_port_set_bits_raw,
    .port_clear_bits_raw = sc16is7xx_gpio_port_clear_bits_raw,
    .port_toggle_bits = sc16is7xx_gpio_port_toggle_bits,
    .pin_interrupt_configure = sc16is7xx_gpio_pin_interrupt_configure,
    .manage_callback = sc16is7xx_gpio_manage_callback,
};

#define SC16IS7XX_CONFIG_COMMON_PROPS(inst, pn_suffix) \
    .common = { .port_pin_mask = \
                    (GPIO_PORT_PIN_MASK_FROM_NGPIOS(DT_INST_PROP(inst, ngpios)) << DT_INST_PROP(inst, channel)) }, \
    .parent_dev = DEVICE_DT_GET(DT_INST_PARENT(inst)), .ngpios = DT_INST_PROP(inst, ngpios)

#define SC16IS7XX_DEFINE(inst, pn_suffix) \
    static struct sc16is7xx_gpio_data sc16is##pn_suffix##_gpio_data_##inst = { \
        .lock = Z_SEM_INITIALIZER(sc16is##pn_suffix##_gpio_data_##inst.lock, 1, 1), \
        .bridge_info.kind = SC16IS7XX_BRIDGE_GPIO, \
        .bridge_info.channel_id = DT_INST_PROP(inst, channel), \
        .bridge_info.on_interrupt = sc16is7xx_gpio_handle_interrupt, \
    }; \
    static const struct sc16is7xx_gpio_config sc16is##pn_suffix##_gpio_config_##inst = { \
        SC16IS7XX_CONFIG_COMMON_PROPS(inst, pn_suffix) \
    }; \
    DEVICE_DT_INST_DEFINE( \
        inst, \
        sc16is7xx_gpio_init, \
        NULL, \
        &sc16is##pn_suffix##_gpio_data_##inst, \
        &sc16is##pn_suffix##_gpio_config_##inst, \
        POST_KERNEL, \
        CONFIG_GPIO_SC16IS7XX_INIT_PRIORITY, \
        &sc16is7xx_gpio_driver_api \
    );

#define SC16IS750_INIT(inst) SC16IS7XX_DEFINE(inst, 750)
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT nxp_sc16is750_gpio_controller
DT_INST_FOREACH_STATUS_OKAY(SC16IS750_INIT)

#define SC16IS760_INIT(inst) SC16IS7XX_DEFINE(inst, 760)
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT nxp_sc16is760_gpio_controller
DT_INST_FOREACH_STATUS_OKAY(SC16IS760_INIT)

#define SC16IS752_INIT(inst) SC16IS7XX_DEFINE(inst, 752)
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT nxp_sc16is752_gpio_controller
DT_INST_FOREACH_STATUS_OKAY(SC16IS752_INIT)

#define SC16IS762_INIT(inst) SC16IS7XX_DEFINE(inst, 762)
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT nxp_sc16is762_gpio_controller
DT_INST_FOREACH_STATUS_OKAY(SC16IS762_INIT)
