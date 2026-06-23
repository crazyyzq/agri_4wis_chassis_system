#include <stdio.h>
#include "board.h"
#include "operator_io.h"
#include "sbus_decoder.h"
#include "test_cases.h"
#include "test_serial_common.h"
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
    return operator_confirm("Stop transmitter now; stream timeout observed") ? TEST_PASS : TEST_FAIL;
}
