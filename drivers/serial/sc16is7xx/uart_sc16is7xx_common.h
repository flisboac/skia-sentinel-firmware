#ifndef CF2C4623_3757_49C7_8F95_0DF2AE415932
#define CF2C4623_3757_49C7_8F95_0DF2AE415932

#include "mfd_sc16is7xx.h"

#include <zephyr/drivers/uart.h>

#ifndef SC16IS7XX_UART_COMMON_API
    #define SC16IS7XX_UART_COMMON_API extern
#endif /* SC16IS7XX_UART_COMMON_API */

#define MAX_TX_FIFO_SIZE (64)
#define MAX_RX_FIFO_SIZE (64)

#define DIV_NEAREST(n, d) (((n) < 0) != ((d) < 0) ? ((n) - ((d) / 2)) / (d) : ((n) + ((d) / 2)) / (d))
#define ABS(v) ((v) > 0 ? v : -v)

#define TCR_REG_VALUE(dev_config) \
    ((((dev_config)->rx_fifo_halt_level / 4) & 0x0f) | ((((dev_config)->rx_fifo_resume_level / 4) & 0x0f) << 4))

#define TLR_REG_VALUE(dev_config) \
    ((((dev_config)->tx_fifo_trigger_level / 4) & 0x0f) | ((((dev_config)->rx_fifo_trigger_level / 4) & 0x0f) << 4))

#define BUS_READ_BYTE(err, dev, bus, channel_id, reg_name, value_addr) \
    if (!(err = sc16is7xx_bus_read_byte( \
              bus, \
              SC16IS7XX_REG_##reg_name( \
                  ((struct sc16is7xx_uart_data*) ((dev)->data))->bridge_info.channel_id, SC16IS7XX_REGRW_READ \
              ), \
              value_addr \
          ), \
          err)) { \
        LOG_DBG( \
            "Device '%s': Could not read register " #reg_name "@%d! Error code = %d", \
            (dev)->name, \
            ((struct sc16is7xx_uart_data*) ((dev)->data))->bridge_info.channel_id, \
            err \
        ); \
    } \
    if (!err) { }

#define BUS_WRITE_BYTE(err, dev, bus, channel_id, reg_name, value) \
    if (!(err = sc16is7xx_bus_write_byte( \
              bus, \
              SC16IS7XX_REG_##reg_name( \
                  ((struct sc16is7xx_uart_data*) ((dev)->data))->bridge_info.channel_id, SC16IS7XX_REGRW_WRITE \
              ), \
              value \
          ), \
          err)) { \
        LOG_DBG( \
            "Device '%s': Could not write register " #reg_name "@%d = 0x%02x! Error code = %d", \
            (dev)->name, \
            ((struct sc16is7xx_uart_data*) ((dev)->data))->bridge_info.channel_id, \
            value, \
            err \
        ); \
    } \
    if (!err) { }

enum sc16is7xx_uart_operation_mode
{
    SC16IS7XX_UART_OPMODE_RS232,
    SC16IS7XX_UART_OPMODE_RS485
};

enum sc16is7xx_uart_hw_flow_mode
{
    SC16IS7XX_UART_HWFLOW_USER,
    SC16IS7XX_UART_HWFLOW_AUTO
};

enum sc16is7xx_uart_sw_flow_mode
{
    SC16IS7XX_UART_SWFLOW_NONE,
    SC16IS7XX_UART_SWFLOW_USER = SC16IS7XX_UART_SWFLOW_NONE,
    SC16IS7XX_UART_SWFLOW_TX0_RX2,
    SC16IS7XX_UART_SWFLOW_TX0_RX1,
    SC16IS7XX_UART_SWFLOW_TX0_RX1AND2,
    SC16IS7XX_UART_SWFLOW_TX1_RX0,
    SC16IS7XX_UART_SWFLOW_TX1_RX2,
    SC16IS7XX_UART_SWFLOW_TX1_RX1,
    SC16IS7XX_UART_SWFLOW_TX1_RX1OR2,
    SC16IS7XX_UART_SWFLOW_TX2_RX0,
    SC16IS7XX_UART_SWFLOW_TX2_RX2,
    SC16IS7XX_UART_SWFLOW_TX2_RX1,
    SC16IS7XX_UART_SWFLOW_TX2_RX1OR2,
    SC16IS7XX_UART_SWFLOW_TX1AND2_RX0,
    SC16IS7XX_UART_SWFLOW_TX1AND2_RX2,
    SC16IS7XX_UART_SWFLOW_TX1AND2_RX1,
    SC16IS7XX_UART_SWFLOW_TX1AND2_RX1AND2
};

struct sc16is7xx_uart_callback
{
    uart_irq_callback_user_data_t fn;
    void* ctx;
}

struct sc16is7xx_uart_baud_rate_settings

{
    int32_t target_value;
    int32_t actual_value;
    int32_t abs_error_diff;  // Difference between target and actual.
    int16_t divider;  // UART divider to be set on device.
    int8_t prescaler;  // UART prescaler to be set on the device. Can be either 1 or 4.
};

struct sc16is7xx_uart_config
{
    const struct device* parent_dev;
    int32_t device_address;  // Negative value if no address was set.
    struct uart_config initial_config;

    union
    {
        uint32_t flags;

        struct
        {
            uint32_t enable_sleep_mode : 1;
            uint32_t disable_tx : 1;
            uint32_t disable_rx : 1;
            uint32_t hw_flow_control : 1;
            uint32_t modem_flow_control : 1;
            uint32_t hw_transmit_loopback : 1;
            uint32_t enable_xon_any : 1;
            uint32_t enable_fifo : 1;
            uint32_t enable_tcr_tlr : 1;
            uint32_t irda_transceiver : 1;
            uint32_t supports_modem_flow_control : 1;
            uint32_t supports_irda_fast_speed : 1;
            uint32_t allow_irda_fast_speed : 1;
            uint32_t detect_special_character : 1;
            uint32_t rs485_invert_rts : 1;
        };
    };

    struct sc16is7xx_uart_operation_mode operation_mode;
    enum sc16is7xx_uart_hw_flow_mode hw_flow_mode;
    enum sc16is7xx_uart_sw_flow_mode sw_flow_mode;
    uint8_t xon[3];
    uint8_t xoff[3];
    uint8_t baud_clock_prescaler;
    uint8_t tx_fifo_trigger_level;
    uint8_t rx_fifo_trigger_level;
    uint8_t rx_fifo_halt_level;
    uint8_t rx_fifo_resume_level;
    uint8_t ier_defaults;
};

struct sc16is7xx_uart_data
{
    struct uart_config runtime_config;
    // struct sc16is7xx_bus bus;
    struct sc16is7xx_uart_callback callback;
    struct k_sem lock;
    struct sc16is7xx_bridge_info bridge_info;
    const struct sc16is7xx_device_info* device_info;
    const struct device* own_instance;
};

static inline void sc16is7xx_uart_command_wait(const struct device* dev)
{
    const struct sc16is7xx_uart_data* data = dev->data;
    k_sleep(data->device_info->cmd_period);
}

static inline void sc16is7xx_uart_xmit_wait(const struct device* dev)
{
    const struct sc16is7xx_uart_data* data = dev->data;
    k_sleep(data->device_info->xmit_period);
}

SC16IS7XX_UART_COMMON_API sc16is7xx_uart_set_zephyr_config_UNSAFE_(  //
    const struct device* dev,
    const struct sc16is7xx_bus* bus,
    const struct uart_config* settings
);

SC16IS7XX_UART_COMMON_API int sc16is7xx_uart_err_check(const struct device* dev);

#ifdef CONFIG_UART_SC16IS7XX_USE_RUNTIME_CONFIGURE
SC16IS7XX_UART_COMMON_API int sc16is7xx_uart_config_get(const struct device* dev, struct uart_config* cfg);
SC16IS7XX_UART_COMMON_API int sc16is7xx_uart_configure(const struct device* dev, const struct uart_config* cfg);
#endif /* CONFIG_UART_SC16IS7XX_USE_RUNTIME_CONFIGURE */

#ifdef CONFIG_UART_SC16IS7XX_LINE_CTRL
SC16IS7XX_UART_COMMON_API int sc16is7xx_uart_line_ctrl_set(const struct device* dev, uint32_t ctrl, uint32_t val);
SC16IS7XX_UART_COMMON_API int sc16is7xx_uart_line_ctrl_get(const struct device* dev, uint32_t ctrl, uint32_t* val);
#endif /* CONFIG_UART_LINE_CTRL */


#endif /* CF2C4623_3757_49C7_8F95_0DF2AE415932 */
