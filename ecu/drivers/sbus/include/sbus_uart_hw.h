/* HPM6750 UART binding for the CPU0 SBUS receiver.
 *
 * The generic SBUS service only knows how to consume bytes and expose a
 * decoded snapshot. This module owns the board-specific UART1 setup and the
 * interrupt handler that feeds those bytes into the service.
 */
#ifndef SBUS_UART_HW_H
#define SBUS_UART_HW_H

#include <stdbool.h>

#include "sbus_service.h"

/* Initialize BOARD_SBUS_UART_BASE as the SBUS receiver.
 *
 * Protocol: 100000 baud, 8 data bits, even parity, 2 stop bits.
 * Interrupts: RX data/timeout, line status, and RX line idle.
 * Returns false only for a local configuration/programming error. Link loss is
 * reported later through sbus_service_snapshot_t.connected.
 */
bool sbus_uart_hw_init(sbus_service_t *service);

/* Declared for the SDK ISR table macro and for source-level contract tests.
 * Application code should not call this function directly.
 */
void sbus_uart_hw_isr(void);

#endif /* SBUS_UART_HW_H */
