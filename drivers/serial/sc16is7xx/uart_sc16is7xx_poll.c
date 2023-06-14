#define SC16IS7XX_UART_POLL_API
#include "uart_sc16is7xx_poll.h"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(nxp_sc16is7xx_uart_controller_poll, CONFIG_UART_LOG_LEVEL);

static int sc16is7xx_uart_poll_in(const struct device* dev, unsigned char* p_char)
{
    const struct sc16is7xx_uart_config* config = dev->config;
    const struct device* parent_dev = config->parent_dev;
    struct sc16is7xx_uart_data* data = dev->data;
    struct sc16is7xx_bus_lock bus_lock;
    uint8_t reg_addr;
    uint8_t reg_data;
    int err = 0;

    SC16IS7XX_BUS_LOCK_INIT(bus_lock);

    err = sc16is7xx_lock_bus(parent_dev, &bus_lock, K_FOREVER);
    if (err) {
        LOG_DBG("Device '%s': Could not lock bus! Error code = %d", dev->name, err);
        return err;
    }

    BUS_READ_BYTE(err, dev, bus_lock.bus, RXLVL, &reg_data) else
    {
        err = -EIO;
        goto end;
    }

    if (reg_data > 0) {
        BUS_READ_BYTE(err, dev, bus_lock.bus, RHR, &reg_data) else
        {
            err = -EIO;
            goto end;
        }

        *p_char = (unsigned char) reg_data;
    } else {
        err = -1;
    }

end:
    if (sc16is7xx_unlock_bus(parent_dev, &bus_lock)) {  //
        LOG_DBG("Device '%s': Could not unlock bus!", dev->name);
    }

    return err;
}

static int sc16is7xx_uart_poll_out(const struct device* dev, unsigned char p_char)
{
    const struct sc16is7xx_uart_config* config = dev->config;
    const struct device* parent_dev = config->parent_dev;
    struct sc16is7xx_uart_data* data = dev->data;
    struct sc16is7xx_bus_lock bus_lock;
    bool waiting = false;
    uint8_t reg_addr;
    uint8_t txlvl_data;
    bool sending = true;
    int err = 0;

    SC16IS7XX_BUS_LOCK_INIT(bus_lock);

    err = sc16is7xx_lock_bus(parent_dev, &bus_lock, K_FOREVER);
    if (err) {
        LOG_DBG("Device '%s': Could not lock bus! Error code = %d", dev->name, err);
        return err;
    }

    // Ensure we have at least one byte available in the FIFO
    if (config->enable_fifo) {
        waiting = false;
        do {
            if (waiting) sc16is7xx_uart_xmit_wait(dev);
            BUS_READ_BYTE(err, dev, bus_lock.bus, TXLVL, &txlvl_data) else
            {
                err = -EIO;
                goto end;
            }
            waiting = txlvl_data >= MAX_TX_FIFO_SIZE;
        }
        while (waiting);
    } else {
        waiting = false;
        do {
            if (waiting) sc16is7xx_uart_xmit_wait(dev);
            BUS_READ_BYTE(err, dev, bus_lock.bus, LSR, &txlvl_data) else
            {
                err = -EIO;
                goto end;
            }
            waiting = !SC16IS7XX_BITCHECK(txlvl_data, SC16IS7XX_REGFLD_LSR_THRTSR);
        }
        while (waiting);
    }

    BUS_WRITE_BYTE(err, dev, bus_lock.bus, THR, p_char) else
    {
        err = -EIO;
        goto end;
    }

    if (config->enable_fifo) {
        waiting = false;
        do {
            if (waiting) sc16is7xx_uart_xmit_wait(dev);
            BUS_READ_BYTE(err, dev, bus_lock.bus, TXLVL, &txlvl_data) else
            {
                err = -EIO;
                goto end;
            }
            waiting = txlvl_data < MAX_TX_FIFO_SIZE;  // Could wait less here
        }
        while (waiting);
    } else {
        do {
            BUS_READ_BYTE(err, dev, bus_lock.bus, LSR, &txlvl_data) else
            {
                err = -EIO;
                goto end;
            }
            sc16is7xx_uart_xmit_wait(dev);
        }
        while (SC16IS7XX_BITCHECK(txlvl_data, SC16IS7XX_REGFLD_LSR_THRTSR));
    }

end:
    if (sc16is7xx_unlock_bus(parent_dev, &bus_lock)) {  //
        LOG_DBG("Device '%s': Could not unlock bus!", dev->name);
    }

    return 0;
}
