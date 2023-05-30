# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_include_directories(../../mfd/sc16is7xx)

zephyr_library_sources(
    sc16is7xx_bridge_uart.c
)
