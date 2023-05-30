# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(
    mfd_sc16is7xx.c
    mfd_sc16is7xx_if_i2c.c
    mfd_sc16is7xx_if_spi.c
)
