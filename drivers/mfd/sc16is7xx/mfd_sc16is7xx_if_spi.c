/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifdef CONFIG_SPI

#include "mfd_sc16is7xx_if.h"
#include <zephyr/logging/log.h>
#include <zephyr/drivers/spi.h>

// TODO Implementation.

LOG_MODULE_DECLARE(nxp_sc16is7xx_spi, CONFIG_MFD_LOG_LEVEL);

static int sc16is7xx_bus_check_spi(const union sc16is7xx_bus_dev* dev)
{
    return spi_is_ready_dt(&dev->spi) ? 0 : -ENOTSUP;
}

static int sc16is7xx_bus_write_spi(const union sc16is7xx_bus_dev* dev, sc16is7xx_regaddr_t start, const uint8_t* buf, size_t size)
{
    return -ENOTSUP;
}

static int sc16is7xx_bus_read_spi(const union sc16is7xx_bus_dev* dev, sc16is7xx_regaddr_t start, uint8_t* buf, size_t size)
{
    return -ENOTSUP;
}

const struct sc16is7xx_bus_api sc16is7xx_bus_api_spi = {
    .check = sc16is7xx_bus_check_spi,
    .read = sc16is7xx_bus_read_spi,
    .write = sc16is7xx_bus_write_spi,
};

#endif
