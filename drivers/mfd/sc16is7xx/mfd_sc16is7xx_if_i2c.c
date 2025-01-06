/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifdef CONFIG_I2C

#include "mfd_sc16is7xx_if.h"
#include <zephyr/logging/log.h>
#include <zephyr/drivers/i2c.h>

LOG_MODULE_DECLARE(nxp_sc16is7xx_i2c, CONFIG_MFD_LOG_LEVEL);

static int sc16is7xx_bus_check_i2c(const union sc16is7xx_bus_dev* bus)
{
    return device_is_ready(bus->i2c.bus) ? 0 : -ENODEV;
}

static int sc16is7xx_bus_read_i2c(const union sc16is7xx_bus_dev* dev, sc16is7xx_regaddr_t start, uint8_t* buf, size_t size)
{
    return i2c_burst_read_dt(&dev->i2c, start, buf, size);
}

static int sc16is7xx_bus_write_i2c(const union sc16is7xx_bus_dev* dev, sc16is7xx_regaddr_t start, const uint8_t* buf, size_t size)
{
    return i2c_burst_write_dt(&dev->i2c, start, buf, size);
}

const struct sc16is7xx_bus_api sc16is7xx_bus_api_i2c = {
    .check = sc16is7xx_bus_check_i2c,
    .read = sc16is7xx_bus_read_i2c,
    .write = sc16is7xx_bus_write_i2c,
};

#endif
