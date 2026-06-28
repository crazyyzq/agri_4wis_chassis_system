#ifndef UART_RS485_HW_H
#define UART_RS485_HW_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define UART_RS485_HW_RX_BUFFER_SIZE (256U)

typedef struct {
    uint32_t rx_count;
    uint32_t tx_count;
    uint32_t line_error_count;
    uint32_t rx_overflow_count;
    uint32_t baudrate;
    uint8_t rx_buffer[UART_RS485_HW_RX_BUFFER_SIZE];
    size_t rx_size;
    size_t rx_head;
    size_t rx_tail;
    bool initialized;
} uart_rs485_hw_t;

bool uart_rs485_1_hw_init(uart_rs485_hw_t *service, uint32_t baudrate);
bool uart_rs485_1_hw_send(uart_rs485_hw_t *service,
                          const uint8_t *data,
                          size_t size);
size_t uart_rs485_1_hw_read(uart_rs485_hw_t *service,
                            uint8_t *out,
                            size_t max_size);
void uart_rs485_1_hw_clear_rx(uart_rs485_hw_t *service);
void uart_rs485_1_hw_isr(void);

#endif /* UART_RS485_HW_H */
