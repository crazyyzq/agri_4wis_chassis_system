/* HPM6750 hardware backend for compile-controlled periodic diagnostics. */
#include <string.h>

#include "board.h"
#include "hpm_can_drv.h"
#include "hpm_clock_drv.h"
#include "hpm_mchtmr_drv.h"
#include "hpm_uart_drv.h"
#include "periodic_tx.h"

#ifndef ECU_PERIODIC_CAN_TX
#define ECU_PERIODIC_CAN_TX 0
#endif
#ifndef ECU_PERIODIC_RS485_TX
#define ECU_PERIODIC_RS485_TX 0
#endif
#ifndef ECU_PERIODIC_RS232_TX
#define ECU_PERIODIC_RS232_TX 0
#endif

static CAN_Type *const s_can_ports[PERIODIC_TX_CAN_COUNT] = {
    BOARD_CAN1_BASE, BOARD_CAN2_BASE, BOARD_CAN3_BASE, BOARD_CAN4_BASE
};
static UART_Type *const s_rs485_ports[PERIODIC_TX_RS485_COUNT] = {
    BOARD_RS485_1_UART_BASE, BOARD_RS485_2_UART_BASE, BOARD_RS485_3_UART_BASE
};
static UART_Type *const s_rs232_ports[PERIODIC_TX_RS232_COUNT] = {
    BOARD_RS232_1_UART_BASE, BOARD_RS232_2_UART_BASE,
    BOARD_RS232_3_UART_BASE, BOARD_RS232_4_UART_BASE
};

/**
 * @brief Convert the free-running MCHTMR count to wrapping milliseconds.
 * @return Low 32 bits of elapsed milliseconds, or zero without a clock.
 */
static uint32_t default_now_ms(void)
{
    uint32_t frequency = clock_get_frequency(clock_mchtmr0);
    if (frequency == 0U) {
        return 0U;
    }
    return (uint32_t)((mchtmr_get_count(HPM_MCHTMR) * 1000ULL) / frequency);
}

/** @brief Configure one one-based external CAN channel for Classic CAN. */
static bool default_can_init(uint8_t channel)
{
    if (channel == 0U || channel > PERIODIC_TX_CAN_COUNT) {
        return false;
    }
    CAN_Type *base = s_can_ports[channel - 1U];
    can_config_t config;
    can_get_default_config(&config);
    config.baudrate = 500000U;
    config.mode = can_mode_normal;
    config.enable_canfd = false;
    board_init_can(base);
    uint32_t clock = board_init_can_clock(base);
    return clock != 0U && can_init(base, &config, clock) == status_success;
}

/** @brief Attempt one non-blocking Classic CAN data-frame transmission. */
static bool default_can_send(uint8_t channel, uint16_t id,
                             const uint8_t data[8])
{
    if (channel == 0U || channel > PERIODIC_TX_CAN_COUNT || data == NULL) {
        return false;
    }
    can_transmit_buf_t message = { 0 };
    message.id = id;
    message.extend_id = false;
    message.dlc = 8U;
    memcpy(message.data, data, 8U);
    return can_send_message_nonblocking(s_can_ports[channel - 1U], &message) ==
           status_success;
}

/** @brief Resolve one serial group/channel to its board UART instance. */
static UART_Type *serial_base(periodic_tx_serial_kind_t kind, uint8_t channel)
{
    if (kind == PERIODIC_TX_SERIAL_RS485) {
        return channel >= 1U && channel <= PERIODIC_TX_RS485_COUNT
            ? s_rs485_ports[channel - 1U] : NULL;
    }
    if (kind == PERIODIC_TX_SERIAL_RS232) {
        return channel >= 1U && channel <= PERIODIC_TX_RS232_COUNT
            ? s_rs232_ports[channel - 1U] : NULL;
    }
    return NULL;
}

/** @brief Configure one serial channel for 115200 8N1 and optional hardware DE. */
static bool default_serial_init(periodic_tx_serial_kind_t kind, uint8_t channel)
{
    UART_Type *uart = serial_base(kind, channel);
    if (uart == NULL) {
        return false;
    }
    board_init_uart(uart);
    uart_config_t config;
    uart_default_config(uart, &config);
    config.src_freq_in_hz = board_init_uart_clock(uart);
    config.baudrate = 115200U;
    config.word_length = word_length_8_bits;
    config.parity = parity_none;
    config.num_of_stop_bits = stop_bits_1;
    config.fifo_enable = true;
    config.modem_config.auto_flow_ctrl_en = kind == PERIODIC_TX_SERIAL_RS485;
    return config.src_freq_in_hz != 0U && uart_init(uart, &config) == status_success;
}

/** @brief Send one complete diagnostic line without waiting for receive data. */
static bool default_serial_write(periodic_tx_serial_kind_t kind, uint8_t channel,
                                 const uint8_t *data, size_t length)
{
    UART_Type *uart = serial_base(kind, channel);
    if (uart == NULL || data == NULL) {
        return false;
    }
    for (size_t i = 0U; i < length; ++i) {
        if (uart_send_byte(uart, data[i]) != status_success) {
            return false;
        }
    }
    return true;
}

void periodic_tx_init_default(void)
{
    static const periodic_tx_hw_ops_t ops = {
        default_now_ms, default_can_init, default_can_send,
        default_serial_init, default_serial_write
    };
    const periodic_tx_config_t config = {
        ECU_PERIODIC_CAN_TX != 0,
        ECU_PERIODIC_RS485_TX != 0,
        ECU_PERIODIC_RS232_TX != 0
    };
    periodic_tx_init(&ops, &config);
}
