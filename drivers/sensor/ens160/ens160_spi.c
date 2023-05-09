/*
 * Copyright (c) 2016, 2017 Intel Corporation
 * Copyright (c) 2017 IpTronix S.r.l.
 * Copyright (c) 2021 Nordic Semiconductor ASA
 * Copyright (c) 2022, Leonard Pollak
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Bus-specific functionality for ENS160s accessed via SPI.
 */

#include <zephyr/logging/log.h>
#include "ens160.h"

// TODO Adjust and test all of this.
// Beware, this SPI implementation is not tested.
// As per the datasheet, this implementation is not correct.
// Some changes are necessary, especially regarding how data is sent.

#if ENS160_BUS_SPI

LOG_MODULE_DECLARE(ens160, CONFIG_SENSOR_LOG_LEVEL);

static int ens160_bus_check_spi(const union ens160_bus *bus)
{
	return spi_is_ready_dt(&bus->spi) ? 0 : -ENODEV;
}

static inline int ens160_set_mem_page(const struct device *dev, uint8_t addr)
{
	const struct ens160_config *config = dev->config;
	struct ens160_data *data = dev->data;
	uint8_t page = (addr > 0x7f) ? 0U : 1U;
	int err = 0;

	if (data->mem_page != page) {
		uint8_t buf[2];

		struct spi_buf tx_buf = {
			.buf = &buf[0],
			.len = 1,
		};
		const struct spi_buf_set tx = {
			.buffers = &tx_buf,
			.count = 1,
		};

		const struct spi_buf rx_buf[] = {
			{ .buf = NULL, .len = 1 },
			{ .buf = &buf[1], .len = 1 },
		};
		const struct spi_buf_set rx = {
			.buffers = rx_buf,
			.count = ARRAY_SIZE(rx_buf),
		};

		buf[0] = ENS160_REG_STATUS | ENS160_SPI_READ_BIT;
		err = spi_transceive_dt(&config->bus.spi, &tx, &rx);
		if (err < 0) {
			return err;
		}

		if (data->mem_page == 1U) {
			buf[1] &= ~ENS160_SPI_MEM_PAGE_MSK;
		} else {
			buf[1] |= ENS160_SPI_MEM_PAGE_MSK;
		}

		buf[0] = ENS160_REG_STATUS & ENS160_SPI_WRITE_MSK;
		tx_buf.len = 2;
		err = spi_write_dt(&config->bus.spi, &tx);
		if (err < 0) {
			return err;
		}

		data->mem_page = page;
	}

	return err;
}

static int ens160_reg_write_spi(const struct device *dev,
				uint8_t reg, uint8_t val)
{
	const struct ens160_config *config = dev->config;
	int err;
	uint8_t cmd[] = { reg & ENS160_SPI_WRITE_MSK, val };
	const struct spi_buf tx_buf = {
		.buf = cmd,
		.len = sizeof(cmd)
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1
	};

	err = ens160_set_mem_page(dev, reg);
	if (err) {
		return err;
	}

	err = spi_write_dt(&config->bus.spi, &tx);

	return err;
}

static int ens160_reg_write_buf_spi(const struct device* dev, uint8_t reg, const uint8_t* buf, int size)
{
	int err = 0;
	int i;
	for (i = 0; !err && i < size; ++i) {
		err = ens160_reg_write_spi(dev, reg + i, *(buf + i));
	}
	return err;
}

static int ens160_reg_read_spi(const struct device *dev,
			       uint8_t start, uint8_t *buf, int size)
{
	const struct ens160_config *config = dev->config;
	int err;
	uint8_t addr;
	const struct spi_buf tx_buf = {
		.buf = &addr,
		.len = 1
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1
	};
	struct spi_buf rx_buf[2];
	const struct spi_buf_set rx = {
		.buffers = rx_buf,
		.count = ARRAY_SIZE(rx_buf)
	};

	rx_buf[0].buf = NULL;
	rx_buf[0].len = 1;

	addr = start | ENS160_SPI_READ_BIT;
	rx_buf[1].buf = buf;
	rx_buf[1].len = size;

	err = ens160_set_mem_page(dev, start);
	if (err) {
		return err;
	}

	err = spi_transceive_dt(&config->bus.spi, &tx, &rx);

	return err;
}

const struct ens160_bus_io ens160_bus_io_spi = {
	.check = ens160_bus_check_spi,
	.read = ens160_reg_read_spi,
	.write = ens160_reg_write_spi,
	.write_buf = ens160_reg_write_buf_spi,
};
#endif /* ENS160_BUS_SPI */
