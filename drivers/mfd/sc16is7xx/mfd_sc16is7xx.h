#ifndef A7093F74_6F3B_4A05_8FF2_D8FCCE709EF1
#define A7093F74_6F3B_4A05_8FF2_D8FCCE709EF1

#include <stdint.h>

#include "mfd_sc16is7xx_if.h"

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>

#ifndef SC16IS7XX_IAPI
    #define SC16IS7XX_IAPI extern
#endif

#define SC16IS7XX_MAYBE_UNUSED __attribute__((unused))

// REGI: Register ID
// REGF: Register field (mask/first-bit preproc tuple)
// REGV: Register field value

#define SC16IS7XX_MAX_UART_CHANNELS (2)
#define SC16IS7XX_MAX_GPIO_CHANNELS (2)
#define SC16IS7XX_MAX_CHANNELS MAX(SC16IS7XX_MAX_UART_CHANNELS, SC16IS7XX_MAX_GPIO_CHANNELS)

#define SC16IS7XX_EXPLODE__(...) __VA_ARGS__
#define SC16IS7XX_APPLY__(FN, ...) FN __VA_ARGS__

#define SC16IS7XX_BITCHECK_1__(reg_value, field_mask, field_fbit) ((reg_value) & (field_mask))
#define SC16IS7XX_BITCHECK(reg_value, field) \
    SC16IS7XX_APPLY__(SC16IS7XX_BITCHECK_1__, (reg_value, SC16IS7XX_EXPLODE__ field))

#define SC16IS7XX_BITSET_1__(reg_value, field_mask, field_fbit) ((reg_value) | (field_mask))
#define SC16IS7XX_BITSET(reg_value, field) \
    SC16IS7XX_APPLY__(SC16IS7XX_BITSET_1__, (reg_value, SC16IS7XX_EXPLODE__ field))

#define SC16IS7XX_BITCLEAR_1__(reg_value, field_mask, field_fbit) ((reg_value) & ~(field_mask))
#define SC16IS7XX_BITCLEAR(reg_value, field) \
    SC16IS7XX_APPLY__(SC16IS7XX_BITCLEAR_1__, (reg_value, SC16IS7XX_EXPLODE__ field))

#define SC16IS7XX_GETVALUE_1__(reg_value, field_mask, field_fbit) (((reg_value) & (field_mask)) >> field_fbit)
#define SC16IS7XX_GETVALUE(reg_value, field) \
    SC16IS7XX_APPLY__(SC16IS7XX_GETVALUE_1__, (reg_value, SC16IS7XX_EXPLODE__ field))

#define SC16IS7XX_SETVALUE_1__(reg_value, value, field_mask, field_fbit) \
    (SC16IS7XX_BITCLEAR_1__(reg_value, field_mask, field_fbit) | (((value) << (field_fbit)) & field_mask))
#define SC16IS7XX_SETVALUE(reg_value, field_value, field) \
    SC16IS7XX_APPLY__(SC16IS7XX_SETVALUE_1__, (reg_value, field_value, SC16IS7XX_EXPLODE__ field))

#define SC16IS7XX_REGRW_READ (1U)
#define SC16IS7XX_REGRW_WRITE (0U)
#define SC16IS7XX_REGRW(val) ((val) ? SC16IS7XX_REGRW_READ : SC16IS7XX_REGRW_WRITE)

#define SC16IS7XX_REG(reg_id, channel_idx, rw_flag) \
    ((SC16IS7XX_REGRW(rw_flag) << 7) | (((reg_id) & (0x0f)) << 3) | (((channel_idx) & (0x03)) << 1))

//
// [[ GENERAL REGISTER IDS ]]
//

#define SC16IS7XX_REG_RHR(channel_idx, rw_flag) SC16IS7XX_REG(SC16IS7XX_REGIDX_RHR, channel_idx, rw_flag)
#define SC16IS7XX_REG_THR(channel_idx, rw_flag) SC16IS7XX_REG(SC16IS7XX_REGIDX_THR, channel_idx, rw_flag)
#define SC16IS7XX_REG_IER(channel_idx, rw_flag) SC16IS7XX_REG(SC16IS7XX_REGIDX_IER, channel_idx, rw_flag)
#define SC16IS7XX_REG_FCR(channel_idx, rw_flag) SC16IS7XX_REG(SC16IS7XX_REGIDX_FCR, channel_idx, rw_flag)
#define SC16IS7XX_REG_IIR(channel_idx, rw_flag) SC16IS7XX_REG(SC16IS7XX_REGIDX_IIR, channel_idx, rw_flag)
#define SC16IS7XX_REG_LCR(channel_idx, rw_flag) SC16IS7XX_REG(SC16IS7XX_REGIDX_LCR, channel_idx, rw_flag)
#define SC16IS7XX_REG_MCR(channel_idx, rw_flag) SC16IS7XX_REG(SC16IS7XX_REGIDX_MCR, channel_idx, rw_flag)
#define SC16IS7XX_REG_LSR(channel_idx, rw_flag) SC16IS7XX_REG(SC16IS7XX_REGIDX_LSR, channel_idx, rw_flag)
#define SC16IS7XX_REG_MSR(channel_idx, rw_flag) SC16IS7XX_REG(SC16IS7XX_REGIDX_MSR, channel_idx, rw_flag)
#define SC16IS7XX_REG_SPR(channel_idx, rw_flag) SC16IS7XX_REG(SC16IS7XX_REGIDX_SPR, channel_idx, rw_flag)
#define SC16IS7XX_REG_TCR(channel_idx, rw_flag) SC16IS7XX_REG(SC16IS7XX_REGIDX_TCR, channel_idx, rw_flag)
#define SC16IS7XX_REG_TLR(channel_idx, rw_flag) SC16IS7XX_REG(SC16IS7XX_REGIDX_TLR, channel_idx, rw_flag)
#define SC16IS7XX_REG_TXLVL(channel_idx, rw_flag) SC16IS7XX_REG(SC16IS7XX_REGIDX_TXLVL, channel_idx, rw_flag)
#define SC16IS7XX_REG_RXLVL(channel_idx, rw_flag) SC16IS7XX_REG(SC16IS7XX_REGIDX_RXLVL, channel_idx, rw_flag)
#define SC16IS7XX_REG_IODIR(channel_idx, rw_flag) SC16IS7XX_REG(SC16IS7XX_REGIDX_IODIR, channel_idx, rw_flag)
#define SC16IS7XX_REG_IOSTATE(channel_idx, rw_flag) SC16IS7XX_REG(SC16IS7XX_REGIDX_IOSTATE, channel_idx, rw_flag)
#define SC16IS7XX_REG_IOINTENA(channel_idx, rw_flag) SC16IS7XX_REG(SC16IS7XX_REGIDX_IOINTENA, channel_idx, rw_flag)
#define SC16IS7XX_REG_IOCONTROL(channel_idx, rw_flag) SC16IS7XX_REG(SC16IS7XX_REGIDX_IOCONTROL, channel_idx, rw_flag)
#define SC16IS7XX_REG_EFCR(channel_idx, rw_flag) SC16IS7XX_REG(SC16IS7XX_REGIDX_EFCR, channel_idx, rw_flag)
#define SC16IS7XX_REG_DLL(channel_idx, rw_flag) SC16IS7XX_REG(SC16IS7XX_REGIDX_DLL, channel_idx, rw_flag)
#define SC16IS7XX_REG_DHL(channel_idx, rw_flag) SC16IS7XX_REG(SC16IS7XX_REGIDX_DHL, channel_idx, rw_flag)

#define SC16IS7XX_REG_EFR(channel_idx, rw_flag) SC16IS7XX_REG(SC16IS7XX_REGIDX_EFR, channel_idx, rw_flag)
#define SC16IS7XX_REG_XON1(channel_idx, rw_flag) SC16IS7XX_REG(SC16IS7XX_REGIDX_XON1, channel_idx, rw_flag)
#define SC16IS7XX_REG_XON2(channel_idx, rw_flag) SC16IS7XX_REG(SC16IS7XX_REGIDX_XON2, channel_idx, rw_flag)
#define SC16IS7XX_REG_XOFF1(channel_idx, rw_flag) SC16IS7XX_REG(SC16IS7XX_REGIDX_XOFF1, channel_idx, rw_flag)
#define SC16IS7XX_REG_XOFF2(channel_idx, rw_flag) SC16IS7XX_REG(SC16IS7XX_REGIDX_XOFF2, channel_idx, rw_flag)

#define SC16IS7XX_REGIDX_RHR (0x00U)
#define SC16IS7XX_REGIDX_THR (0x00U)
#define SC16IS7XX_REGIDX_IER (0x01U)
#define SC16IS7XX_REGIDX_FCR (0x02U)
#define SC16IS7XX_REGIDX_IIR (0x02U)
#define SC16IS7XX_REGIDX_LCR (0x03U)
#define SC16IS7XX_REGIDX_MCR (0x04U)
#define SC16IS7XX_REGIDX_LSR (0x05U)
#define SC16IS7XX_REGIDX_MSR (0x06U)
#define SC16IS7XX_REGIDX_SPR (0x07U)
#define SC16IS7XX_REGIDX_TCR (0x06U)
#define SC16IS7XX_REGIDX_TLR (0x07U)
#define SC16IS7XX_REGIDX_TXLVL (0x08U)
#define SC16IS7XX_REGIDX_RXLVL (0x09U)
#define SC16IS7XX_REGIDX_IODIR (0x0AU)
#define SC16IS7XX_REGIDX_IOSTATE (0x0BU)
#define SC16IS7XX_REGIDX_IOINTENA (0x0CU)
#define SC16IS7XX_REGIDX_IOCONTROL (0x0EU)
#define SC16IS7XX_REGIDX_EFCR (0x0FU)
#define SC16IS7XX_REGIDX_DLL (0x00U)
#define SC16IS7XX_REGIDX_DLH (0x01U)

#define SC16IS7XX_REGIDX_EFR (0x02U)
#define SC16IS7XX_REGIDX_XON1 (0x04U)
#define SC16IS7XX_REGIDX_XON2 (0x05U)
#define SC16IS7XX_REGIDX_XOFF1 (0x06U)
#define SC16IS7XX_REGIDX_XOFF2 (0x07U)

#define SC16IS7XX_REGFLD_IER_CTS ((1U << 7U), (7U))
#define SC16IS7XX_REGFLD_IER_RTS ((1U << 6U), (6U))
#define SC16IS7XX_REGFLD_IER_XOFF ((1U << 5U), (5U))
#define SC16IS7XX_REGFLD_IER_SLEEP ((1U << 4U), (4U))
#define SC16IS7XX_REGFLD_IER_MODEM ((1U << 3U), (3U))
#define SC16IS7XX_REGFLD_IER_RXLINE ((1U << 2U), (2U))
#define SC16IS7XX_REGFLD_IER_THR ((1U << 1U), (1U))
#define SC16IS7XX_REGFLD_IER_RHR ((1U << 0U), (0U))

#define SC16IS7XX_REGFLD_FCR_RXTRIGGER ((0x03U << 6U), (6U))
#define SC16IS7XX_REGFLD_FCR_TXTRIGGER ((0x03U << 4U), (4U))
#define SC16IS7XX_REGFLD_FCR_TXRESET ((1U << 2U), (2U))
#define SC16IS7XX_REGFLD_FCR_RXRESET ((1U << 1U), (1U))
#define SC16IS7XX_REGFLD_FCR_ENABLE ((1U << 0U), (0U))

#define SC16IS7XX_REGVAL_FCR_RXTRIGGER_8 (0x00U)
#define SC16IS7XX_REGVAL_FCR_RXTRIGGER_16 (0x01U)
#define SC16IS7XX_REGVAL_FCR_RXTRIGGER_56 (0x02U)
#define SC16IS7XX_REGVAL_FCR_RXTRIGGER_60 (0x03U)

#define SC16IS7XX_REGVAL_FCR_TXTRIGGER_8 (0x00U)
#define SC16IS7XX_REGVAL_FCR_TXTRIGGER_16 (0x01U)
#define SC16IS7XX_REGVAL_FCR_TXTRIGGER_32 (0x02U)
#define SC16IS7XX_REGVAL_FCR_TXTRIGGER_56 (0x03U)

#define SC16IS7XX_REGFLD_IIR_ID ((0x1FU << 1U), (0U))
#define SC16IS7XX_REGFLD_IIR_STATUS ((1U << 0U), (0U))

#define SC16IS7XX_REGVAL_IIR_ID_RXLINEERROR (0x06U)
#define SC16IS7XX_REGVAL_IIR_ID_RXTIMEOUTERROR (0x0CU)
#define SC16IS7XX_REGVAL_IIR_ID_RHR (0x04U)
#define SC16IS7XX_REGVAL_IIR_ID_THR (0x02U)
#define SC16IS7XX_REGVAL_IIR_ID_MODEM (0x00U)
#define SC16IS7XX_REGVAL_IIR_ID_GPIO (0x30U)
#define SC16IS7XX_REGVAL_IIR_ID_XOFF (0x10U)
#define SC16IS7XX_REGVAL_IIR_ID_HWFLOW (0x20U)

#define SC16IS7XX_REGFLD_LCR_DIVISORENABLE ((1U << 7U), (7U))
#define SC16IS7XX_REGFLD_LCR_BREAKCONTROL ((1U << 6U), (6U))
#define SC16IS7XX_REGFLD_LCR_PARITYFORCEBIT ((1U << 5U), (5U))
#define SC16IS7XX_REGFLD_LCR_PARITYTYPEBIT ((1U << 4U), (4U))
#define SC16IS7XX_REGFLD_LCR_PARITYENABLEBIT ((1U << 3U), (3U))
#define SC16IS7XX_REGFLD_LCR_STOPBITS ((1U << 2U), (2U))
#define SC16IS7XX_REGFLD_LCR_WORDLENGTH ((0x03U << 0U), (0U))

#define SC16IS7XX_REGVAL_LCR_PARITY_NONE (0x00U)
#define SC16IS7XX_REGVAL_LCR_PARITY_ODD (0x01U)
#define SC16IS7XX_REGVAL_LCR_PARITY_EVEN (0x03U)
#define SC16IS7XX_REGVAL_LCR_PARITY_MARK (0x05U)
#define SC16IS7XX_REGVAL_LCR_PARITY_SPACE (0x07U)

#define SC16IS7XX_REGVAL_LCR_STOPBITS_1 (0x0U)
#define SC16IS7XX_REGVAL_LCR_STOPBITS_1_5 (0x1U)
#define SC16IS7XX_REGVAL_LCR_STOPBITS_2 (0x1U)

#define SC16IS7XX_REGVAL_LCR_WORDLENGTH_5 (0U)
#define SC16IS7XX_REGVAL_LCR_WORDLENGTH_6 (1U)
#define SC16IS7XX_REGVAL_LCR_WORDLENGTH_7 (2U)
#define SC16IS7XX_REGVAL_LCR_WORDLENGTH_8 (3U)

#define SC16IS7XX_REGFLD_MCR_CLOCKDIVISOR ((1U << 7U), (7U))
#define SC16IS7XX_REGFLD_MCR_IRDAMODE ((1U << 6U), (6U))
#define SC16IS7XX_REGFLD_MCR_XONANY ((1U << 5U), (5U))
#define SC16IS7XX_REGFLD_MCR_LOOPBACK ((1U << 4U), (4U))
#define SC16IS7XX_REGFLD_MCR_TCRTLRENABLE ((1U << 2U), (2U))
#define SC16IS7XX_REGFLD_MCR_RTS ((1U << 1U), (1U))
#define SC16IS7XX_REGFLD_MCR_DTR ((1U << 0U), (0U))

#define SC16IS7XX_REGFLD_LSR_FIFO ((1U << 7U), (7U))
#define SC16IS7XX_REGFLD_LSR_THRTSR ((1U << 6U), (6U))
#define SC16IS7XX_REGFLD_LSR_THR ((1U << 5U), (5U))
#define SC16IS7XX_REGFLD_LSR_BREAK ((1U << 4U), (4U))
#define SC16IS7XX_REGFLD_LSR_FRAMING ((1U << 3U), (3U))
#define SC16IS7XX_REGFLD_LSR_PARITY ((1U << 2U), (2U))
#define SC16IS7XX_REGFLD_LSR_OVERRUN ((1U << 1U), (1U))
#define SC16IS7XX_REGFLD_LSR_RXDATA ((1U << 0U), (0U))

#define SC16IS7XX_REGFLD_MSR_CD ((1U << 7U), (7U))
#define SC16IS7XX_REGFLD_MSR_RI ((1U << 6U), (6U))
#define SC16IS7XX_REGFLD_MSR_DSR ((1U << 5U), (5U))
#define SC16IS7XX_REGFLD_MSR_CTS ((1U << 4U), (4U))
#define SC16IS7XX_REGFLD_MSR_XCD ((1U << 3U), (3U))
#define SC16IS7XX_REGFLD_MSR_XRI ((1U << 2U), (2U))
#define SC16IS7XX_REGFLD_MSR_XDSR ((1U << 1U), (1U))
#define SC16IS7XX_REGFLD_MSR_XCTS ((1U << 0U), (0U))

#define SC16IS7XX_REGFLD_TCR_RXRESUME ((0xfU << 4U), (4U))
#define SC16IS7XX_REGFLD_TCR_RXHALT ((0xfU << 0U), (0U))

#define SC16IS7XX_REGFLD_TLR_RX ((0xfU << 4U), (4U))
#define SC16IS7XX_REGFLD_TLR_TX ((0xfU << 0U), (0U))

#define SC16IS7XX_REGFLD_IOCONTROL_SOFTRESET ((1U << 3U), (3U))
#define SC16IS7XX_REGFLD_IOCONTROL_GPIO0MODE ((1U << 2U), (2U))
#define SC16IS7XX_REGFLD_IOCONTROL_GPIO1MODE ((1U << 1U), (1U))
#define SC16IS7XX_REGFLD_IOCONTROL_GPIOXMODE(channel_idx) \
    ((1U << ((1 - (channel_idx)) + 1U)), ((1 - (channel_idx)) + 1U))
#define SC16IS7XX_REGFLD_IOCONTROL_IOLATCH ((1U << 0U), (0U))

#define SC16IS7XX_REGVAL_IOCONTROL_GPIOXMODE_IO (0)
#define SC16IS7XX_REGVAL_IOCONTROL_GPIOXMODE_MODEM (0)

#define SC16IS7XX_REGFLD_EFCR_IRDAMODE ((1U << 7U), (7U))
#define SC16IS7XX_REGFLD_EFCR_RTSINVERT ((1U << 5U), (5U))
#define SC16IS7XX_REGFLD_EFCR_RTSCONTROL ((1U << 4U), (4U))
#define SC16IS7XX_REGFLD_EFCR_TXDISABLE ((1U << 2U), (2U))
#define SC16IS7XX_REGFLD_EFCR_RXDISABLE ((1U << 1U), (1U))
#define SC16IS7XX_REGFLD_EFCR_RS485MODE ((1U << 0U), (0U))

#define SC16IS7XX_REGFLD_EFR_CTSFLOW ((1U << 7U), (7U))
#define SC16IS7XX_REGFLD_EFR_RTSFLOW ((1U << 6U), (6U))
#define SC16IS7XX_REGFLD_EFR_SPECIALCHAR ((1U << 5U), (5U))
#define SC16IS7XX_REGFLD_EFR_ENHANCEDBIT ((1U << 4U), (4U))
#define SC16IS7XX_REGFLD_EFR_SWFLOWMODE ((0xfU << 0U), (0U))

#define SC16IS7XX_REGVAL_EFR_SWFLOW_NONE (0x0U)
#define SC16IS7XX_REGVAL_EFR_SWFLOW_TX0_RX1 (0x1U)
#define SC16IS7XX_REGVAL_EFR_SWFLOW_TX0_RX2 (0x2U)
#define SC16IS7XX_REGVAL_EFR_SWFLOW_TX0_RX1AND2 (0x3U)
#define SC16IS7XX_REGVAL_EFR_SWFLOW_TX1_RX0 (0x4U)
#define SC16IS7XX_REGVAL_EFR_SWFLOW_TX1_RX2 (0x5U)
#define SC16IS7XX_REGVAL_EFR_SWFLOW_TX1_RX1 (0x6U)
#define SC16IS7XX_REGVAL_EFR_SWFLOW_TX1_RX1OR2 (0x7U)
#define SC16IS7XX_REGVAL_EFR_SWFLOW_TX2_RX0 (0x8U)
#define SC16IS7XX_REGVAL_EFR_SWFLOW_TX2_RX2 (0x9U)
#define SC16IS7XX_REGVAL_EFR_SWFLOW_TX2_RX1 (0xAU)
#define SC16IS7XX_REGVAL_EFR_SWFLOW_TX2_RX1OR2 (0xBU)
#define SC16IS7XX_REGVAL_EFR_SWFLOW_TX1AND2_RX0 (0xCU)
#define SC16IS7XX_REGVAL_EFR_SWFLOW_TX1AND2_RX2 (0xDU)
#define SC16IS7XX_REGVAL_EFR_SWFLOW_TX1AND2_RX1 (0xEU)
#define SC16IS7XX_REGVAL_EFR_SWFLOW_TX1AND2_RX1AND2 (0xFU)

//
// [[ SPECIAL REGISTER IDS ]]
//

#define SC16IS7XX_REGIDX_DLL (0x00U)
#define SC16IS7XX_REGIDX_DLH (0x01U)

#define SC16IS7XX_REG_DLL(channel_idx, rw_flag) SC16IS7XX_REG(SC16IS7XX_REGIDX_DLL, channel_idx, rw_flag)
#define SC16IS7XX_REG_DLH(channel_idx, rw_flag) SC16IS7XX_REG(SC16IS7XX_REGIDX_DLH, channel_idx, rw_flag)

// -----

#define SC16IS7XX_BUS_LOCK_INIT(lock) \
    do { \
        lock.bus = NULL; \
        lock.locked = false; \
    } \
    while (false)

//
// [[ STRUCTS/DEFINES ]]
//


enum sc16is7xx_part_id
{
    SC16IS7XX_PARTID_740,
    SC16IS7XX_PARTID_750,
    SC16IS7XX_PARTID_760,
    SC16IS7XX_PARTID_752,
    SC16IS7XX_PARTID_762
};

enum sc16is7xx_bridge_kind
{
    SC16IS7XX_BRIDGE_UART = 1,
    SC16IS7XX_BRIDGE_GPIO,
};

struct sc16is7xx_interrupt_info
{
    uint8_t iir;
};

struct sc16is7xx_bus_lock
{
    struct sc16is7xx_bus* bus;
    bool locked;
};

struct sc16is7xx_device_info
{
    uint8_t total_uart_channels;
    uint8_t total_gpio_channels;
    uint8_t shared_gpio_channels_len;
    bool supports_modem_flow_control;
    bool supports_interrupts;
    enum sc16is7xx_part_id part_id;
    uint32_t xtal_freq;
    int32_t interrupt_polling_period;
    k_timeout_t xtal_period;
    k_timeout_t xmit_period;
    k_timeout_t cmd_period;
    const uint8_t* shared_gpio_channels;
};

struct sc16is7xx_bridge_info
{
    /**
     * @brief Indicates the bridge's device type.
     *
     * Although all APIs are internal, some validation will
     * be performed by the MFD device. This field helps the MFD
     * ensure proper resource allocation. For example,
     * limitations will apply (ie. resource allocations will fail)
     * when registering more than the number of available devices
     * per type.
     */
    enum sc16is7xx_bridge_kind kind;

    /**
     * @brief The channel (for UART) or port (for GPIO)
     * that the child/bridge device is controlling.
     */
    uint8_t channel_id;

    /**
     * @brief A callback function, called when the device receives
     * an interrupt.
     */
    void (*on_interrupt)(  //
        const struct sc16is7xx_bridge_info* bridge_info,
        const struct sc16is7xx_interrupt_info* interrupt_info,
        uint8_t bridge_index
    );
};

SC16IS7XX_IAPI const struct sc16is7xx_device_info* sc16is7xx_get_device_info(  //
    const struct device* mfd_dev
);

SC16IS7XX_IAPI int sc16is7xx_register_bridge(  //
    const struct device* mfd_dev,
    const struct sc16is7xx_bridge_info* bridge_info
);

SC16IS7XX_IAPI int sc16is7xx_enqueue_interrupt_work(  //
    const struct device* mfd_dev,
    struct k_work* work
);

SC16IS7XX_IAPI int sc16is7xx_lock_bus(  //
    const struct device* mfd_dev,
    struct sc16is7xx_bus_lock* lock,
    k_timeout_t timeout
);

SC16IS7XX_IAPI int sc16is7xx_unlock_bus(  //
    const struct device* mfd_dev,
    struct sc16is7xx_bus_lock* lock
);

#endif /* A7093F74_6F3B_4A05_8FF2_D8FCCE709EF1 */
