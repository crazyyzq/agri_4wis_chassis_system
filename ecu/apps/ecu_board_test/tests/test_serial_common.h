/* Internal UART test helpers; not part of the application-wide public API. */
#ifndef ECU_TEST_SERIAL_COMMON_H
#define ECU_TEST_SERIAL_COMMON_H
#include <stdbool.h>
#include <stdint.h>
#include "hpm_uart_drv.h"
#include "test_runner.h"
/**
 * @brief Initialize one board UART for an 8-bit polled fixture protocol.
 * @param uart         Supported UART instance to initialize.
 * @param baudrate     Nonzero line rate in bits per second.
 * @param parity       SDK parity selection.
 * @param stop_bits    SDK stop-bit selection.
 * @param automatic_de true to route UART hardware-DE for RS485 direction.
 * @return true when arguments and UART initialization succeed; otherwise false.
 */
bool test_uart_configure(UART_Type *uart, uint32_t baudrate, parity_setting_t parity,
                         num_of_stop_bits_t stop_bits, bool automatic_de);
/**
 * @brief Send and verify deterministic framed echoes through one UART.
 * @param uart        Configured UART connected to an echo fixture.
 * @param channel     One-based logical channel encoded into each frame.
 * @param frame_count Nonzero number of sequential frames to exchange.
 * @param context     Active context used for foreground abort polling.
 * @return true only when every byte and decoded payload matches; false for
 *         invalid arguments, abort, UART error, 2 s byte timeout, or mismatch.
 */
bool test_uart_echo_frames(UART_Type *uart, uint8_t channel, uint32_t frame_count,
                           test_context_t *context);
#endif
