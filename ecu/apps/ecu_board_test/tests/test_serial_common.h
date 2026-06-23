#ifndef ECU_TEST_SERIAL_COMMON_H
#define ECU_TEST_SERIAL_COMMON_H
#include <stdbool.h>
#include <stdint.h>
#include "hpm_uart_drv.h"
#include "test_runner.h"
bool test_uart_configure(UART_Type *uart, uint32_t baudrate, parity_setting_t parity,
                         num_of_stop_bits_t stop_bits, bool automatic_de);
bool test_uart_echo_frames(UART_Type *uart, uint8_t channel, uint32_t frame_count,
                           test_context_t *context);
#endif
