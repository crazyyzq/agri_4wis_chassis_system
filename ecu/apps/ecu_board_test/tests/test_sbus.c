/* Receive/decode 100 SBUS frames and verify the transmitter can go silent. */
#include <stdio.h>
#include "board.h"
#include "operator_io.h"
#include "sbus_decoder.h"
#include "test_cases.h"
#include "test_serial_common.h"

/**
 * @brief Verify that no SBUS byte arrives during a continuous observation window.
 * @param context    Active context polled for abort once per millisecond.
 * @param observe_ms Required silent interval in milliseconds.
 * @return true only when the complete interval is silent; false on data or abort.
 */
static bool sbus_stream_is_silent(test_context_t *context, uint32_t observe_ms)
{
    for (uint32_t elapsed = 0U; elapsed < observe_ms; ++elapsed) {
        if (test_runner_poll_abort(context)) return false;
        uint8_t byte;
        if (uart_try_receive_byte(BOARD_SBUS_UART_BASE, &byte) == status_success) return false;
        board_delay_ms(1U);
    }
    return true;
}

test_status_t test_sbus(test_context_t *context)
{
    if (!operator_confirm("SBUS receiver is connected and transmitting")) return TEST_BLOCKED;
    if (!test_uart_configure(BOARD_SBUS_UART_BASE, BOARD_SBUS_BAUDRATE, parity_even, stop_bits_2, false)) return TEST_FAIL;
    uint8_t frame_data[25];
    uint32_t valid = 0U;
    uint8_t position = 0U;
    for (uint32_t wait = 0U; wait < 200000U && valid < 100U; ++wait) {
        if (test_runner_poll_abort(context)) return TEST_BLOCKED;
        uint8_t byte;
        if (uart_try_receive_byte(BOARD_SBUS_UART_BASE, &byte) != status_success) { board_delay_us(100U); continue; }
        if (position == 0U && byte != 0x0FU) continue;
        frame_data[position++] = byte;
        if (position == sizeof(frame_data)) {
            sbus_frame_t decoded;
            if (sbus_decode(frame_data, sizeof(frame_data), &decoded) == SBUS_OK) {
                ++valid;
                if (valid == 1U || valid == 100U) printf("SBUS frame=%lu ch1=%u lost=%u failsafe=%u\n",
                    (unsigned long)valid, decoded.channels[0], decoded.frame_lost, decoded.failsafe);
            }
            position = 0U;
        }
    }
    if (valid < 100U) return TEST_FAIL;
    if (!operator_confirm("Stop the SBUS transmitter, then confirm")) return TEST_BLOCKED;
    return sbus_stream_is_silent(context, 50U) ? TEST_PASS :
        (context->abort_requested ? TEST_BLOCKED : TEST_FAIL);
}
