/*
 * Copyright (c) 2018 Bosch Sensortec GmbH
 * Copyright (c) 2022, Leonard Pollak
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __ZEPHYR_DRIVERS_SENSOR_ENS160_H__
#define __ZEPHYR_DRIVERS_SENSOR_ENS160_H__

#define DT_DRV_COMPAT sciosense_ens160

#include <zephyr/types.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#if ENS160_BUS_I2C
#include <zephyr/drivers/i2c.h>
#endif
#if ENS160_BUS_SPI
#include <zephyr/drivers/spi.h>
#endif
#include <zephyr/drivers/sensor/skia_ens160.h>

// #if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
// #error "no sciosense_ens160 DT available"
// #endif

#define ENS160_BUS_SPI DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
#define ENS160_BUS_I2C DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)

union ens160_bus
{
#if ENS160_BUS_SPI
    struct spi_dt_spec spi;
#endif
#if ENS160_BUS_I2C
    struct i2c_dt_spec i2c;
#endif
};

typedef int (*ens160_bus_check_fn)(const union ens160_bus* bus);
typedef int (*ens160_reg_read_fn)(const struct device* dev, uint8_t start, uint8_t* buf, int size);
typedef int (*ens160_reg_write_fn)(const struct device* dev, uint8_t reg, uint8_t val);
typedef int (*ens160_reg_write_buf_fn)(const struct device* dev, uint8_t reg, const uint8_t* buf, int size);

struct ens160_bus_io
{
    ens160_bus_check_fn check;
    ens160_reg_read_fn read;
    ens160_reg_write_fn write;
    ens160_reg_write_buf_fn write_buf;
};

#if ENS160_BUS_SPI
    #define ENS160_SPI_OPERATION \
        (SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_OP_MODE_MASTER)
extern const struct ens160_bus_io ens160_bus_io_spi;
#endif

#if ENS160_BUS_I2C
extern const struct ens160_bus_io ens160_bus_io_i2c;
#endif

struct ens160_config
{
    union ens160_bus bus;
    const struct ens160_bus_io* bus_io;
};

// Chip constants
#define ENS160_PARTID 0x0160
#define ENS161_PARTID 0x0161
#define ENS160_BOOTING 10

// 7-bit I2C slave address of the ENS160
#define ENS160_I2CADDR_0 0x52  // ADDR low
#define ENS160_I2CADDR_1 0x53  // ADDR high

// ENS160 registers for version V0
#define ENS160_REG_PART_ID 0x00  // 2 byte register
#define ENS160_REG_OPMODE 0x10
#define ENS160_REG_CONFIG 0x11
#define ENS160_REG_COMMAND 0x12
#define ENS160_REG_TEMP_IN 0x13
#define ENS160_REG_RH_IN 0x15
#define ENS160_REG_DATA_STATUS 0x20
#define ENS160_REG_DATA_AQI 0x21
#define ENS160_REG_DATA_TVOC 0x22
#define ENS160_REG_DATA_ECO2 0x24
#define ENS160_REG_DATA_BL 0x28
#define ENS160_REG_DATA_T 0x30
#define ENS160_REG_DATA_RH 0x32
#define ENS160_REG_DATA_MISR 0x38
#define ENS160_REG_GPR_WRITE_0 0x40
#define ENS160_REG_GPR_WRITE_1 (ENS160_REG_GPR_WRITE_0 + 1)
#define ENS160_REG_GPR_WRITE_2 (ENS160_REG_GPR_WRITE_0 + 2)
#define ENS160_REG_GPR_WRITE_3 (ENS160_REG_GPR_WRITE_0 + 3)
#define ENS160_REG_GPR_WRITE_4 (ENS160_REG_GPR_WRITE_0 + 4)
#define ENS160_REG_GPR_WRITE_5 (ENS160_REG_GPR_WRITE_0 + 5)
#define ENS160_REG_GPR_WRITE_6 (ENS160_REG_GPR_WRITE_0 + 6)
#define ENS160_REG_GPR_WRITE_7 (ENS160_REG_GPR_WRITE_0 + 7)
#define ENS160_REG_GPR_READ_0 0x48
#define ENS160_REG_GPR_READ_4 (ENS160_REG_GPR_READ_0 + 4)
#define ENS160_REG_GPR_READ_6 (ENS160_REG_GPR_READ_0 + 6)
#define ENS160_REG_GPR_READ_7 (ENS160_REG_GPR_READ_0 + 7)

// ENS160 data register fields
#define ENS160_COMMAND_NOP 0x00
#define ENS160_COMMAND_CLRGPR 0xCC
#define ENS160_COMMAND_GET_APPVER 0x0E
#define ENS160_COMMAND_SETTH 0x02
#define ENS160_COMMAND_SETSEQ 0xC2

#define ENS160_BL_CMD_START 0x02
#define ENS160_BL_CMD_ERASE_APP 0x04
#define ENS160_BL_CMD_ERASE_BLINE 0x06
#define ENS160_BL_CMD_WRITE 0x08
#define ENS160_BL_CMD_VERIFY 0x0A
#define ENS160_BL_CMD_GET_BLVER 0x0C
#define ENS160_BL_CMD_GET_APPVER 0x0E
#define ENS160_BL_CMD_EXITBL 0x12

#define ENS160_SEQ_ACK_NOTCOMPLETE 0x80
#define ENS160_SEQ_ACK_COMPLETE 0xC0

#define IS_ENS160_SEQ_ACK_NOT_COMPLETE(x) (ENS160_SEQ_ACK_NOTCOMPLETE == (ENS160_SEQ_ACK_NOTCOMPLETE & (x)))
#define IS_ENS160_SEQ_ACK_COMPLETE(x) (ENS160_SEQ_ACK_COMPLETE == (ENS160_SEQ_ACK_COMPLETE & (x)))

#ifndef ENS160_POW
    #ifdef CONFIG_MINIMAL_LIBC
        #define ENS160_POW(b, e) upow32(b, e)
    #else
        #include <math.h>
        #define ENS160_POW(b, e) pow(b, e)
    #endif
#endif

#define CONVERT_RS_RAW2OHMS_I(x) (1 << ((x) >> 11))
#define CONVERT_RS_RAW2OHMS_F(x) (ENS160_POW(2, (float) (x) / 2048))


#define ENS160_CONCAT_BYTES(msb, lsb) (((uint16_t) msb << 8) | (uint16_t) lsb)

enum sensor_value_ens160_stage
{
    SENSOR_ENS160_STAGE_UNKNOWN,
    SENSOR_ENS160_STAGE_RUNNING,
    SENSOR_ENS160_STAGE_CUSTOM_CONFIGURING,
};

struct ens160_step_config
{
    uint8_t time;

    union
    {
        uint8_t measuring;

        struct
        {
            uint8_t measuring_hp0 : 1;
            uint8_t measuring_hp1 : 1;
            uint8_t measuring_hp2 : 1;
            uint8_t measuring_hp3 : 1;
        };
    };

    union
    {
        uint16_t temperatures[4];

        struct
        {
            uint16_t temperature_hp0;
            uint16_t temperature_hp1;
            uint16_t temperature_hp2;
            uint16_t temperature_hp3;
        };
    };
};

struct ens160_data
{
    union
    {
        uint8_t bool_flags;

        struct
        {
            uint8_t available : 1;
            uint8_t waiting_for_new_data : 1;
        };
    };

    uint8_t rev_ens16x;
    uint8_t fw_ver_major;
    uint8_t fw_ver_minor;
    uint8_t fw_ver_build;
    uint8_t step_count;
    uint8_t aqi_uba;
    uint8_t sensor_status;
    uint16_t max_wait_cycles;
    uint16_t part_id;
    uint16_t aqi_epa;
    uint16_t tvoc;
    uint16_t eco2;
    enum sensor_value_ens160_stage stage;
    enum sensor_value_ens160_opmode opmode;
    enum sensor_value_ens160_opmode desired_opmode;
    struct ens160_step_config step_config;
    struct sensor_value temp_in;
    struct sensor_value rh_in;
    k_timeout_t wait_cycle_time;

    union
    {
        uint32_t hotplates_rs[4];

        struct
        {
            uint32_t hotplate0_rs;
            uint32_t hotplate1_rs;
            uint32_t hotplate2_rs;
            uint32_t hotplate3_rs;
        };
    };

    union
    {
        uint32_t hotplates_bl[4];

        struct
        {
            uint32_t hotplate0_bl;
            uint32_t hotplate1_bl;
            uint32_t hotplate2_bl;
            uint32_t hotplate3_bl;
        };
    };

#if ENS160_BUS_SPI
    uint8_t mem_page;
#endif
};

static inline uint32_t upow32(uint32_t base, uint32_t exp)
{
    int result = 1;
    while (exp) {
        if (exp & 1) result *= base;
        exp >>= 1;
        base *= base;
    }
    return result;
}

#endif /* __ZEPHYR_DRIVERS_SENSOR_ENS160_H__ */
