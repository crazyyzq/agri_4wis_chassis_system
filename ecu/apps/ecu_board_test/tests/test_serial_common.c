/* Shared UART setup and framed echo engine for RS232/RS485 test cases. */
#include <string.h>
#include "board.h"
#include "comm_pattern.h"
#include "test_serial_common.h"
bool test_uart_configure(UART_Type *uart, uint32_t baudrate, parity_setting_t parity,
                         num_of_stop_bits_t stop_bits, bool automatic_de)
{
    if (uart == NULL || baudrate == 0U) return false;
    board_init_uart(uart);
    uart_config_t config;
    uart_default_config(uart, &config);
    config.src_freq_in_hz = board_init_uart_clock(uart);
    config.baudrate = baudrate;
    config.word_length = word_length_8_bits;
    config.parity = parity;
    config.num_of_stop_bits = stop_bits;
    config.fifo_enable = true;
    config.modem_config.auto_flow_ctrl_en = automatic_de;
    return uart_init(uart, &config) == status_success;
}
static bool receive_byte_timeout(UART_Type *uart, uint8_t *byte)
{
    if (uart == NULL || byte == NULL) return false;
    for (uint32_t wait = 0U; wait < 2000U; ++wait) {
        if (uart_try_receive_byte(uart, byte) == status_success) return true;
        board_delay_ms(1U);
    }
    return false;
}
bool test_uart_echo_frames(UART_Type *uart, uint8_t channel, uint32_t frame_count,
                           test_context_t *context)
{
    if (uart == NULL || context == NULL || frame_count == 0U) return false;
    uint8_t tx[64], rx[64];
    for (uint32_t sequence = 0U; sequence < frame_count; ++sequence) {
        if (test_runner_poll_abort(context)) return false;
        comm_frame_t source, decoded;
        comm_pattern_make(channel, sequence, &source);
        size_t length = comm_frame_encode(&source, tx, sizeof(tx));
        for (size_t i = 0U; i < length; ++i) if (uart_send_byte(uart, tx[i]) != status_success) return false;
        for (size_t i = 0U; i < length; ++i) if (!receive_byte_timeout(uart, &rx[i])) return false;
        if (comm_frame_decode(rx, length, &decoded) != COMM_OK ||
            decoded.channel != source.channel || decoded.sequence != source.sequence ||
            decoded.length != source.length || memcmp(decoded.payload, source.payload, source.length) != 0) return false;
    }
    return true;
}
