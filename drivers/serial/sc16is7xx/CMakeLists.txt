# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_include_directories(../../mfd/sc16is7xx)

zephyr_library_sources(
    uart_sc16is7xx.c
    uart_sc16is7xx_common.c
    uart_sc16is7xx_poll.c
)

# zephyr_library_sources_ifdef(CONFIG_UART_SC16IS7XX_ASYNC_API uart_sc16is7xx_async.c)
# zephyr_library_sources_ifdef(CONFIG_UART_SC16IS7XX_INTERRUPT_DRIVEN uart_sc16is7xx_int.c)
