// REMOVE-ME
#define MFD_SC16IS7XX_INTERRUPT
#define MFD_SC16IS7XX_INTERRUPT_OWN_THREAD

#define SC16IS7XX_IAPI
#include "mfd_sc16is7xx.h"

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>
#include <zephyr/logging/log.h>

#ifdef CONFIG_MFD_SC16IS7XX_PM
    #include <zephyr/pm/device.h>
#endif

#ifdef MFD_SC16IS7XX_INTERRUPT
    #define MFD_SC16IS7XX_INTERRUPT_ENABLED 1
#else
    #define MFD_SC16IS7XX_INTERRUPT_ENABLED 0
#endif

#ifdef MFD_SC16IS7XX_INTERRUPT_OWN_THREAD
    #define MFD_SC16IS7XX_INTERRUPT_OWN_THREAD_ENABLED 1
#else
    #define MFD_SC16IS7XX_INTERRUPT_OWN_THREAD_ENABLED 0
#endif

LOG_MODULE_REGISTER(nxp_sc16is7xx_mfd, CONFIG_MFD_LOG_LEVEL);

/**
 * @brief Has information on all registered bridges, for the strict
 * purposes of MFD device (ie. global) validation and initialization.
 */
struct sc16is7xx_bridge_dt_info
{
    enum sc16is7xx_bridge_kind kind;
    uint8_t channel_id;
    bool modem_flow_control_enabled;
};

struct sc16is7xx_interrupt_config
{
    union
    {
        struct gpio_dt_spec gpio;
    };

    int (*on_setup)(const struct device* dev);
};

struct sc16is7xx_bridge_registration
{
    const struct sc16is7xx_bridge_info* bridge_info;
};

struct sc16is7xx_device_config
{
    size_t bridges_cap;
    struct sc16is7xx_bus bus;
    struct sc16is7xx_device_info device_info;
#ifdef MFD_SC16IS7XX_INTERRUPT
    struct sc16is7xx_interrupt_config interrupt;
    #ifdef MFD_SC16IS7XX_INTERRUPT_OWN_THREAD
    size_t interrupt_queue_stack_len;
    void* interrupt_queue_stack_area;
    #endif /* MFD_SC16IS7XX_INTERRUPT_OWN_THREAD */
#endif /* MFD_SC16IS7XX_INTERRUPT */
    const struct sc16is7xx_bridge_dt_info* bridges_dt_info;
    struct sc16is7xx_bridge_registration* bridges_ptr;
};

struct sc16is7xx_device_data
{
    bool ready;
    size_t bridges_len;
    struct k_mutex bus_lock;
#ifdef MFD_SC16IS7XX_INTERRUPT
    struct k_spinlock interrupt_lock;
    struct k_work interrupt_work;
    #ifdef MFD_SC16IS7XX_INTERRUPT_OWN_THREAD
    struct k_work_q interrupt_queue;
    #endif /* MFD_SC16IS7XX_INTERRUPT_OWN_THREAD */
    struct gpio_callback gpio_interrupt_callback;
    const struct device* own_instance;
#endif /* MFD_SC16IS7XX_INTERRUPT */
};

static int sc16is7xx_mfd_init_bridges_UNSAFE_(const struct device* mfd_dev);

const struct sc16is7xx_device_info* sc16is7xx_get_device_info( //
    const struct device* mfd_dev
)
{
    const struct sc16is7xx_device_config* config;

    if (!mfd_dev) { return NULL; }

    return &config->device_info;
}

int sc16is7xx_register_bridge(  //
    const struct device* mfd_dev,
    const struct sc16is7xx_bridge_info* bridge_info
)
{
    if (!mfd_dev || !bridge_info) { return -EINVAL; }

    size_t i;
    k_spinlock_key_t lock_handle;
    const struct sc16is7xx_device_config* config = mfd_dev->config;
    struct sc16is7xx_device_data* data = mfd_dev->data;
    struct sc16is7xx_bridge_registration* sub;
    int err = 0;

    lock_handle = k_spin_lock(&data->interrupt_lock);

    if (data->bridges_len >= config->bridges_cap) {
        err = -ENOMEM;
        goto end;
    }

    for (i = 0; i < data->bridges_len; i++) {
        sub = config->bridges_ptr + data->bridges_len;

        if (sub->bridge_info->kind == bridge_info->kind && sub->bridge_info->channel_id == bridge_info->channel_id) {
            LOG_ERR(  //
                "Device '%s': Trying to register sub-device twice, check driver implementation or devicetree for "
                "duplicates! kind = %d, channel_id = %d",
                mfd_dev->name,
                bridge_info->kind,
                bridge_info->channel_id
            );
            err = -EINVAL;
            goto end;
        }
    }

    sub = config->bridges_ptr + data->bridges_len;
    data->bridges_len++;

    sub->bridge_info = bridge_info;

    if (data->bridges_len == config->bridges_cap) {
        LOG_DBG(  //
            "Device '%s': %d sub-device(s) registered, %d remaining",
            mfd_dev->name,
            data->bridges_len,
            config->bridges_cap - data->bridges_len
        );
        goto end;
    }

    err = sc16is7xx_mfd_init_bridges_UNSAFE_(mfd_dev);

end:
    k_spin_unlock(&data->interrupt_lock, lock_handle);
    return err;
}

int sc16is7xx_enqueue_interrupt_work(  //
    const struct device* mfd_dev,
    struct k_work* work
)
{
    struct sc16is7xx_device_data* data;

    if (!mfd_dev || !work) { return -EINVAL; }

#ifndef MFD_SC16IS7XX_INTERRUPT
    return -ENOTSUP;
#endif

#ifdef MFD_SC16IS7XX_INTERRUPT_OWN_THREAD
    data = mfd_dev->data;
    return k_work_submit_to_queue(&data->interrupt_queue, work);
#else
    return k_work_submit(work);
#endif
}

int sc16is7xx_lock_bus( //
    const struct device* mfd_dev,
    struct sc16is7xx_bus_lock* lock,
    k_timeout_t timeout
)
{
    int err;

    if (!mfd_dev || !lock) { return -EINVAL; }

    const struct sc16is7xx_device_config* config = mfd_dev->config;
    struct sc16is7xx_device_data* data = mfd_dev->data;

    err = k_mutex_lock(&data->bus_lock, timeout);
    if (err) { return err; }

    lock->bus = &config->bus;
    return 0;
}

int sc16is7xx_unlock_bus(  //
    const struct device* mfd_dev,
    struct sc16is7xx_bus_lock* lock
)
{
    int err;

    if (!mfd_dev || !lock) { return -EINVAL; }

    struct sc16is7xx_device_data* data = mfd_dev->data;

    return k_mutex_unlock(&data->bus_lock);
}

static int sc16is7xx_mfd_init_bridges_UNSAFE_(const struct device* mfd_dev)
{
    struct sc16is7xx_device_data* const data = mfd_dev->data;
    data->ready = true;
    return 0;
}

#ifdef MFD_SC16IS7XX_INTERRUPT
static void sc16is7xx_mfd_handle_interrupt(struct k_work* work_item)
{
    size_t i;
    uint8_t channel_id;
    uint8_t reg_addr;
    uint8_t reg_value;
    struct sc16is7xx_bridge_registration* sub;

    struct sc16is7xx_device_data* const data = CONTAINER_OF(work_item, struct sc16is7xx_device_data, interrupt_work);
    const struct device* const dev = data->own_instance;
    const struct sc16is7xx_device_config* const config = dev->config;
    const struct sc16is7xx_bus* const bus = &config->bus;
    struct sc16is7xx_interrupt_info interrupt_info;
    k_spinlock_key_t lock_handle;
    int err = 0;
    bool has_interrupts, ready;

    // Maybe it's okay to NOT lock data access before dereferencing `data->ready`.
    lock_handle = k_spin_lock(&data->interrupt_lock);
    ready = data->ready;
    k_spin_unlock(&data->interrupt_lock, lock_handle);

    if (!data->ready) { return; }

    for (channel_id = 0; !err && channel_id < config->device_info.total_uart_channels; channel_id++) {
        has_interrupts = true;

        while (!err && has_interrupts) {
            // Maybe it's okay to NOT lock the bus here, because:
            // 1. This register is read-only; and
            // 2. It's only read by the MFD driver.
            reg_addr = SC16IS7XX_REG_IIR(channel_id, SC16IS7XX_REGRW_READ);
            err = sc16is7xx_bus_read_byte(bus, reg_addr, &reg_value);
            if (err) {
                LOG_ERR("Device '%s': Could not read interrupt information!", dev->name);
                break;
            }

            has_interrupts = SC16IS7XX_BITCHECK(reg_value, SC16IS7XX_REGFLD_IIR_STATUS);
            if (!has_interrupts) { break; }

            interrupt_info.iir = reg_value;

            for (i = 0; i < data->bridges_len; ++i) {
                sub = config->bridges_ptr + i;
                sub->bridge_info->on_interrupt(sub->bridge_info, &interrupt_info);
            }
        }
    }
}

static void sc16is7xx_mfd_on_gpio_callback(  //
    const struct device* port,
    struct gpio_callback* cb,
    gpio_port_pins_t pins
)
{
    struct sc16is7xx_device_data* const data = CONTAINER_OF(cb, struct sc16is7xx_device_data, gpio_interrupt_callback);
    const struct device* const dev = data->own_instance;
    const struct sc16is7xx_device_config* const config = dev->config;
    const struct sc16is7xx_bus* const bus = &config->bus;

    ARG_UNUSED(port);
    ARG_UNUSED(pins);

    #ifdef MFD_SC16IS7XX_INTERRUPT_OWN_THREAD
    k_work_submit_to_queue(&data->interrupt_queue, &data->interrupt_work);
    #else
    k_work_submit(&data->interrupt_work);
    #endif
}

SC16IS7XX_MAYBE_UNUSED static int sc16is7xx_mfd_on_setup_gpio(const struct device* dev)
{
    const struct sc16is7xx_device_config* config = dev->config;
    struct sc16is7xx_device_data* data = dev->data;
    int ret = 0;

    if (!device_is_ready(config->interrupt.gpio.port)) {
        LOG_ERR("%s: interrupt GPIO not ready", dev->name);
        return -ENODEV;
    }

    ret = gpio_pin_configure_dt(&config->interrupt.gpio, GPIO_INPUT);
    if (ret != 0) {
        LOG_ERR(  //
            "%s: failed to configure interrupt pin %d (%d)",
            dev->name,
            config->interrupt.gpio.pin,
            ret
        );
        return ret;
    }

    gpio_init_callback(  //
        &data->gpio_interrupt_callback,
        sc16is7xx_mfd_on_gpio_callback,
        BIT(config->interrupt.gpio.pin)
    );

    gpio_add_callback(  //
        config->interrupt.gpio.port,
        &data->gpio_interrupt_callback
    );

    return ret;
}

#endif

static int sc16is7xx_device_validate_dt_info(const struct device* dev)
{
    const struct sc16is7xx_device_config* config = dev->config;
    const struct sc16is7xx_bridge_dt_info* lhs_bridge_dt;
    const struct sc16is7xx_bridge_dt_info* rhs_bridge_dt;
    size_t i, j;
    bool is_shared_gpio_channel, is_modem_flow_enabled, is_same_kind, is_same_channel;

    // Max channel index validation is already performed at DTS-level.
    // Here, we only perform the semantic validations, for aspects
    // of the implementation that cannot be described or restricted at
    // Devicetree level.
    //

    for (i = 0; i < config->bridges_cap; i++) {
        lhs_bridge_dt = config->bridges_dt_info + i;
        is_shared_gpio_channel = false;

        if (lhs_bridge_dt->kind == SC16IS7XX_BRIDGE_GPIO) {
            for (j = 0; !is_shared_gpio_channel && j < config->device_info.shared_gpio_channels_len; j++) {
                is_shared_gpio_channel = lhs_bridge_dt->channel_id == config->device_info.shared_gpio_channels[j];
            }
        }

        for (j = 0; j < config->bridges_cap; j++) {
            rhs_bridge_dt = config->bridges_dt_info + j;
            is_same_kind = lhs_bridge_dt->kind == rhs_bridge_dt->kind;
            is_same_channel = lhs_bridge_dt->channel_id == rhs_bridge_dt->channel_id;

            if (is_same_kind && is_same_channel) {
                LOG_ERR(
                    "Device '%s': Duplicate bridge devices detected in the devicetree!"
                    " Child_indexes = [%d, %d], kind = %d, channel = %d",
                    dev->name,
                    i, j,
                    lhs_bridge_dt->kind,
                    lhs_bridge_dt->channel_id
                );
                return -EINVAL;
            }


            if (is_shared_gpio_channel) {
                is_modem_flow_enabled = rhs_bridge_dt->kind == SC16IS7XX_BRIDGE_UART && rhs_bridge_dt->modem_flow_control_enabled;

                if (is_modem_flow_enabled && is_same_channel) {
                    LOG_ERR(
                        "Device '%s': A GPIO channel is sharing its port with an UART channel with modem-flow control active!"
                        " Channel = %d, GPIO child index = %d, UART child index = %d",
                        dev->name,
                        lhs_bridge_dt->channel_id,
                        i, j
                    );
                    return -EINVAL;
                }
            }
        }
    }

    return 0;
}

static int sc16is7xx_device_softreset_UNSAFE_(const struct device* dev)
{
    const struct sc16is7xx_device_config* config = dev->config;
    const struct sc16is7xx_bus* bus = &config->bus;
    struct sc16is7xx_device_data* data = dev->data;
    uint8_t reg_addr;
    uint8_t reg_data;
    int err;

    reg_addr = SC16IS7XX_REG_IOCONTROL(0, SC16IS7XX_REGRW_WRITE);
    err = sc16is7xx_bus_read_byte(bus, reg_addr, &reg_data);
    if (err) { return err; }

    reg_data = SC16IS7XX_BITSET(reg_data, SC16IS7XX_REGFLD_IOCONTROL_SOFTRESET);
    err = sc16is7xx_bus_write_byte(bus, reg_addr, reg_data);
    if (err) { return err; }

    return err;
}

static int sc16is7xx_device_init(const struct device* dev)
{
    const struct sc16is7xx_device_config* config = dev->config;
    const struct sc16is7xx_bus* bus = &config->bus;
    struct sc16is7xx_device_data* data = dev->data;
    int err;

    err = sc16is7xx_bus_check(bus);
    if (err < 0) {
        LOG_ERR("Device '%s': Bus not ready!", dev->name);
        return err;
    }

    data->bridges_len = 0;
    data->own_instance = dev;

    err = k_mutex_init(&data->bus_lock);
    if (err) { return err; }

    err = sc16is7xx_device_validate_dt_info(dev);
    if (err) { return err; }

#ifdef MFD_SC16IS7XX_INTERRUPT
#ifdef MFD_SC16IS7XX_INTERRUPT_OWN_THREAD
    k_work_queue_start(  //
        &data->interrupt_queue,
        config->interrupt_queue_stack_area,
        config->interrupt_queue_stack_len,
        CONFIG_MFD_SC16IS7XX_INTERRUPT_THREAD_PRIORITY,
        NULL
    );
#endif

    k_work_init(&data->interrupt_work, sc16is7xx_mfd_handle_interrupt);

    if (!config->interrupt.on_setup) { return -EINVAL; }
    err = config->interrupt.on_setup(dev);
    if (err != 0) { return err; }
#endif

    err = sc16is7xx_device_softreset_UNSAFE_(dev);
    if (err) { return err; }

    data->ready = false;

    return err;
    return 0;
}

#define DT_INST_SC16IS7XX_CHILDREN_LEN_1_(inst) (1)
#define DT_INST_SC16IS7XX_CHILDREN_LEN(inst) \
    (DT_INST_FOREACH_CHILD_STATUS_OKAY_SEP(inst, DT_INST_SC16IS7XX_CHILDREN_LEN_1_, (+)))

#define DT_INST_SC16IS7XX(inst, pn_suffix) \
    DT_INST(inst, COND_CODE_1(DT_INST_ON_BUS(inst, spi), (nxp_sc16is##pn_suffix##_spi), (nxp_sc16is##pn_suffix##_i2c)))

#define SC16IS7XX_CONFIG_COMMON_PROPS(inst, pn_suffix) \
    COND_CODE_1( \
        MFD_SC16IS7XX_INTERRUPT_ENABLED, \
        (COND_CODE_1( \
            DT_INST_NODE_HAS_PROP(inst, interrupt_gpios), \
            (.interrupt.gpio = GPIO_DT_SPEC_INST_GET(inst, interrupt_gpios), ), \
            () \
        )), \
        () \
    ) \
        .interrupt.on_setup = COND_CODE_1( \
        MFD_SC16IS7XX_INTERRUPT_ENABLED, \
        COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, interrupt_gpios), (sc16is7xx_device_on_setup_gpio), (NULL)), \
        (NULL) \
    ), \
     COND_CODE_1( \
         MFD_SC16IS7XX_INTERRUPT_OWN_THREAD_ENABLED, \
         (.interrupt_queue_stack_len = K_THREAD_STACK_SIZEOF(sc16is##pn_suffix##_device_interrupt_stack_area_##inst), \
          .interrupt_queue_stack_area = &sc16is##pn_suffix##_device_interrupt_stack_area_##inst, ), \
         () \
     ) \
         .device_info.total_uart_channels = DT_INST_PROP(inst, total_uart_channels), \
     .device_info.total_gpio_channels = DT_INST_PROP(inst, total_gpio_channels), \
     .device_info.shared_gpio_channels_len = DT_INST_PROP_LEN(inst, shared_gpio_channels), \
     .device_info.supports_hw_interrupts = MFD_SC16IS7XX_INTERRUPT_ENABLED && (DT_INST_NODE_HAS_PROP(inst, interrupt_gpios)), \
     .device_info.supports_modem_flow_control = DT_INST_PROP(inst, supports_modem_flow_control), \
     .device_info.part_id = DT_INST_ENUM_IDX(inst, part_number), \
     .device_info.xtal_freq = DT_INST_PROP(inst, xtal_freq), \
     .device_info.shared_gpio_channels = sc16is##pn_suffix##_device_shared_gpio_channels_##inst, \
     .bridges_dt_info = sc16is##pn_suffix##_device_bridges_dt_info_##inst, \
     .bridges_cap = DT_INST_SC16IS7XX_CHILDREN_LEN(inst), .bridges_ptr = sc16is##pn_suffix##_device_subs_##inst

#define SC16IS7XX_CONFIG_SPI(inst, pn_suffix) \
    { \
        .bus.dev.spi = SPI_DT_SPEC_INST_GET(inst, SC16IS7XX_SPI_OPERATION, 0), .bus.api = &sc16is7xx_bus_api_spi, \
        SC16IS7XX_CONFIG_COMMON_PROPS(inst, pn_suffix) \
    }

#define SC16IS7XX_CONFIG_I2C(inst, pn_suffix) \
    { \
        .bus.dev.i2c = I2C_DT_SPEC_INST_GET(inst), .bus.api = &sc16is7xx_bus_api_i2c, \
        SC16IS7XX_CONFIG_COMMON_PROPS(inst, pn_suffix) \
    }

#define SC16IS7XX_DEFINE_BRIDGE_DT(resolved_inst) \
    { \
        .kind = DT_PROP_OR(resolved_inst, gpio_controller, false) ? SC16IS7XX_BRIDGE_GPIO : SC16IS7XX_BRIDGE_UART, \
        .channel_id = DT_PROP(resolved_inst, channel), \
        .modem_flow_control_enabled = DT_PROP_OR(resolved_inst, modem_flow_control, false) \
    }

#define SC16IS7XX_DEFINE(inst, pn_suffix) \
    static const struct sc16is7xx_bridge_dt_info \
        sc16is##pn_suffix##_device_bridges_dt_info_##inst[DT_INST_SC16IS7XX_CHILDREN_LEN(inst)] = { \
            DT_INST_FOREACH_CHILD_STATUS_OKAY_SEP(inst, SC16IS7XX_DEFINE_BRIDGE_DT, (, )) \
        }; \
    COND_CODE_1( \
        MFD_SC16IS7XX_INTERRUPT_OWN_THREAD_ENABLED, \
        (K_THREAD_STACK_DEFINE( \
             sc16is##pn_suffix##_device_interrupt_stack_area_##inst, CONFIG_MFD_SC16IS7XX_INTERRUPT_THREAD_STACK_SIZE \
        );), \
        () \
    ) \
    static const uint8_t \
        sc16is##pn_suffix##_device_shared_gpio_channels_##inst[DT_INST_PROP_LEN(inst, shared_gpio_channels)] = \
            DT_INST_PROP(inst, shared_gpio_channels); \
    static struct sc16is7xx_bridge_registration \
        sc16is##pn_suffix##_device_subs_##inst[DT_INST_SC16IS7XX_CHILDREN_LEN(inst)]; \
    static struct sc16is7xx_device_data sc16is##pn_suffix##_device_data_##inst = {0}; \
    static const struct sc16is7xx_device_config sc16is##pn_suffix##_device_config_##inst = COND_CODE_1( \
        DT_INST_ON_BUS(inst, spi), (SC16IS7XX_CONFIG_SPI(inst, pn_suffix)), (SC16IS7XX_CONFIG_I2C(inst, pn_suffix)) \
    ); \
    DEVICE_DT_DEFINE( \
        DT_INST_SC16IS7XX(inst, pn_suffix), \
        sc16is7xx_device_init, \
        NULL, \
        &sc16is##pn_suffix##_device_data_##inst, \
        &sc16is##pn_suffix##_device_config_##inst, \
        POST_KERNEL, \
        CONFIG_MFD_SC16IS7XX_DEVICE_INIT_PRIORITY, \
        NULL \
    );

#define SC16IS740_INIT(inst) SC16IS7XX_DEFINE(inst, 740)
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT nxp_sc16is740_i2c
DT_INST_FOREACH_STATUS_OKAY(SC16IS740_INIT)

#define SC16IS740_INIT(inst) SC16IS7XX_DEFINE(inst, 740)
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT nxp_sc16is740_spi
DT_INST_FOREACH_STATUS_OKAY(SC16IS740_INIT)

#define SC16IS750_INIT(inst) SC16IS7XX_DEFINE(inst, 750)
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT nxp_sc16is750_i2c
DT_INST_FOREACH_STATUS_OKAY(SC16IS750_INIT)

#define SC16IS750_INIT(inst) SC16IS7XX_DEFINE(inst, 750)
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT nxp_sc16is750_spi
DT_INST_FOREACH_STATUS_OKAY(SC16IS750_INIT)

#define SC16IS760_INIT(inst) SC16IS7XX_DEFINE(inst, 760)
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT nxp_sc16is760_i2c
DT_INST_FOREACH_STATUS_OKAY(SC16IS760_INIT)

#define SC16IS760_INIT(inst) SC16IS7XX_DEFINE(inst, 760)
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT nxp_sc16is760_spi
DT_INST_FOREACH_STATUS_OKAY(SC16IS760_INIT)

#define SC16IS752_INIT(inst) SC16IS7XX_DEFINE(inst, 752)
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT nxp_sc16is752_i2c
DT_INST_FOREACH_STATUS_OKAY(SC16IS752_INIT)

#define SC16IS752_INIT(inst) SC16IS7XX_DEFINE(inst, 752)
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT nxp_sc16is752_spi
DT_INST_FOREACH_STATUS_OKAY(SC16IS752_INIT)

#define SC16IS762_INIT(inst) SC16IS7XX_DEFINE(inst, 762)
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT nxp_sc16is762_i2c
DT_INST_FOREACH_STATUS_OKAY(SC16IS762_INIT)

#define SC16IS762_INIT(inst) SC16IS7XX_DEFINE(inst, 762)
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT nxp_sc16is762_spi
DT_INST_FOREACH_STATUS_OKAY(SC16IS762_INIT)
