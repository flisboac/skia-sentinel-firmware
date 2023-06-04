#ifndef B3944738_DED4_4B67_80FF_315FD24D2C7C
#define B3944738_DED4_4B67_80FF_315FD24D2C7C

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/types.h>

#ifdef CONFIG_I2C
#include <zephyr/drivers/i2c.h>
#endif

#ifdef CONFIG_SPI
#include <zephyr/drivers/spi.h>
#endif

// NOTE Cannot conditionally compile away neither SPI nor I2C fields
// per-device, like in other drivers, because:
//
// 1. Multiple models from the same IC family are implemented by
//    this driver;
// 2. The necessary macros to achieve this (ie. conditional compilation)
//    depend explicitly on a DT_DRV_COMPAT being defined before being
//    referenced; and
// 3. Defining DT_DRV_COMPAT for all variants is impossible at a
//    header-level.
//
// Remember that the SC16IS7XX is a multi-function device, having
// both UART and GPIO available thorugh I2C or SPI.
//
// E.g. those are incorrect assumptions:
// #define SC16IS7XX_BUS_SPI DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
// #define SC16IS7XX_BUS_I2C DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
//

typedef uint8_t sc16is7xx_regaddr_t;

#ifdef CONFIG_SPI
#define SC16IS7XX_SPI_OPERATION \
    (SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_OP_MODE_MASTER)
#endif

struct sc16is7xx_bus_regs
{
    uint8_t iodir; /* 0 for input, 1 for output */
    uint8_t iostate;
};

union sc16is7xx_bus_dev
{
#ifdef CONFIG_SPI
    struct spi_dt_spec spi;
#endif
#ifdef CONFIG_I2C
    struct i2c_dt_spec i2c;
#endif
};

typedef int (*sc16is7xx_bus_check_fn)(const union sc16is7xx_bus_dev* bus);
typedef int (*sc16is7xx_bus_read_fn)(  //
    const union sc16is7xx_bus_dev* dev,
    sc16is7xx_regaddr_t start,
    uint8_t* buf,
    size_t size
);
typedef int (*sc16is7xx_bus_write_fn)(  //
    const union sc16is7xx_bus_dev* dev,
    sc16is7xx_regaddr_t start,
    const uint8_t* buf,
    size_t size
);

struct sc16is7xx_bus_api
{
    sc16is7xx_bus_check_fn check;
    sc16is7xx_bus_read_fn read;
    sc16is7xx_bus_write_fn write;
};

struct sc16is7xx_bus
{
    const struct sc16is7xx_bus_api* const api;
    union sc16is7xx_bus_dev const dev;
    struct sc16is7xx_bus_regs regs;
    struct sc16is7xx_bus_regs prev_regs;
    struct sc16is7xx_bus_regs curr_regs;
};


#ifdef CONFIG_I2C
extern const struct sc16is7xx_bus_api sc16is7xx_bus_api_i2c;
#endif

#ifdef CONFIG_SPI
extern const struct sc16is7xx_bus_api sc16is7xx_bus_api_spi;
#endif

static inline int sc16is7xx_bus_check(const struct sc16is7xx_bus* bus)
{
    return bus->api->check(&bus->dev);
}

static inline int sc16is7xx_bus_read(  //
    const struct sc16is7xx_bus* bus,
    sc16is7xx_regaddr_t start,
    uint8_t* buf,
    size_t size
)
{
    return bus->api->read(&bus->dev, start, buf, size);
}

static inline int sc16is7xx_bus_write(  //
    const struct sc16is7xx_bus* bus,
    sc16is7xx_regaddr_t start,
    const uint8_t* buf,
    size_t size
)
{
    return bus->api->write(&bus->dev, start, buf, size);
}

static inline int sc16is7xx_bus_read_byte(  //
    const struct sc16is7xx_bus* bus,
    sc16is7xx_regaddr_t start,
    uint8_t* buf
)
{
    return sc16is7xx_bus_read(bus, start, buf, 1);
}

static inline int sc16is7xx_bus_write_byte(  //
    const struct sc16is7xx_bus* bus,
    sc16is7xx_regaddr_t start,
    uint8_t val
)
{
    return sc16is7xx_bus_write(bus, start, &val, 1);
}

#endif /* B3944738_DED4_4B67_80FF_315FD24D2C7C */
