/*
 * Copyright (c) 2016, 2017 Intel Corporation
 * Copyright (c) 2017 IpTronix S.r.l.
 * Copyright (c) 2021 Nordic Semiconductor ASA
 * Copyright (c) 2022, Leonard Pollak
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Bus-specific functionality for ENS160s accessed via I2C.
 */

#include "ens160.h"

#if ENS160_BUS_I2C
LOG_MODULE_DECLARE(skia_ens160_i2c, CONFIG_SENSOR_LOG_LEVEL);

static int ens160_bus_check_i2c(const union ens160_bus* bus)
{
    return device_is_ready(bus->i2c.bus) ? 0 : -ENODEV;
}

static int ens160_reg_read_i2c(const struct device* dev, uint8_t start, uint8_t* buf, int size)
{
    const struct ens160_config* config = dev->config;

    return i2c_burst_read_dt(&config->bus.i2c, start, buf, size);
}

static int ens160_reg_write_i2c(const struct device* dev, uint8_t reg, uint8_t val)
{
    const struct ens160_config* config = dev->config;

    return i2c_reg_write_byte_dt(&config->bus.i2c, reg, val);
}

static int ens160_reg_write_buf_i2c(const struct device* dev, uint8_t reg, const uint8_t* buf, int size)
{
    const struct ens160_config* config = dev->config;

    return i2c_burst_write_dt(&config->bus.i2c, reg, buf, size);
}

const struct ens160_bus_io ens160_bus_io_i2c = {
    .check = ens160_bus_check_i2c,
    .read = ens160_reg_read_i2c,
    .write = ens160_reg_write_i2c,
    .write_buf = ens160_reg_write_buf_i2c,
};
#endif /* ENS160_BUS_I2C */
