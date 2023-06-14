#ifndef F72E9808_CF10_43A6_9A4A_4FD39E855C40
#define F72E9808_CF10_43A6_9A4A_4FD39E855C40

#include "uart_sc16is7xx_common.h"

#ifndef SC16IS7XX_UART_POLL_API
    #define SC16IS7XX_UART_POLL_API extern
#endif /* SC16IS7XX_UART_POLL_API */

SC16IS7XX_UART_POLL_API int sc16is7xx_uart_poll_in(const struct device* dev, unsigned char* p_char);
SC16IS7XX_UART_POLL_API int sc16is7xx_uart_poll_out(const struct device* dev, unsigned char p_char);

#endif /* F72E9808_CF10_43A6_9A4A_4FD39E855C40 */
