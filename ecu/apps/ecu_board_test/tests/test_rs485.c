#include <stdio.h>
#include "board.h"
#include "operator_io.h"
#include "safety_manager.h"
#include "test_cases.h"
#include "test_limits.h"
#include "test_serial_common.h"
static UART_Type *const rs485_ports[] = { BOARD_RS485_1_UART_BASE, BOARD_RS485_2_UART_BASE, BOARD_RS485_3_UART_BASE };
void test_rs485_cleanup(test_context_t *context)
{
    (void)context;
    for (uint8_t i = 1U; i <= 3U; ++i) safety_rs485_receive(i);
}
test_status_t test_rs485(test_context_t *context)
{
    if (!operator_confirm("RS485 echo fixture is connected, 115200 8N1")) return TEST_BLOCKED;
    for (uint8_t port = 0U; port < 3U; ++port) {
        if (!test_uart_configure(rs485_ports[port], 115200U, parity_none, stop_bits_1, true)) return TEST_FAIL;
        safety_rs485_transmit(port + 1U);
        bool ok = test_uart_echo_frames(rs485_ports[port], port + 1U, TEST_COMM_FRAME_COUNT, context);
        safety_rs485_receive(port + 1U);
        printf("RS485.%u frames=%u result=%s\n", port + 1U, TEST_COMM_FRAME_COUNT, ok ? "PASS" : "FAIL");
        if (!ok) return context->abort_requested ? TEST_BLOCKED : TEST_FAIL;
    }
    return TEST_PASS;
}
