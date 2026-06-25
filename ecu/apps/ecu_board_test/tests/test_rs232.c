/* Four-port RS232 deterministic echo and physical polarity confirmation. */
#include <stdio.h>
#include "board.h"
#include "operator_io.h"
#include "test_cases.h"
#include "test_limits.h"
#include "test_serial_common.h"
static UART_Type *const rs232_ports[] = { BOARD_RS232_1_UART_BASE, BOARD_RS232_2_UART_BASE, BOARD_RS232_3_UART_BASE, BOARD_RS232_4_UART_BASE };
test_status_t test_rs232(test_context_t *context)
{
    if (!operator_confirm("RS232 echo fixture is connected, 115200 8N1")) return TEST_BLOCKED;
    for (uint8_t port = 0U; port < 4U; ++port) {
        if (!test_uart_configure(rs232_ports[port], 115200U, parity_none, stop_bits_1, false) ||
            !test_uart_echo_frames(rs232_ports[port], port + 1U, TEST_COMM_FRAME_COUNT, context))
            return context->abort_requested ? TEST_BLOCKED : TEST_FAIL;
        char prompt[80];
        snprintf(prompt, sizeof(prompt), "RS232.%u TX idle level is negative and active level is positive", port + 1U);
        if (!operator_confirm(prompt)) return TEST_FAIL;
    }
    return TEST_PASS;
}
