#ifndef UART_COMM_SERVICE_H
#define UART_COMM_SERVICE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define UART_COMM_SERVICE_MAX_FRAME_BYTES (256U)

typedef struct {
    uint32_t baudrate;
    bool online;
    uint8_t last_tx[UART_COMM_SERVICE_MAX_FRAME_BYTES];
    size_t last_tx_size;
    uint32_t tx_count;
} uart_comm_service_t;

/* Initialize one UART communication service.
 *
 * Owner: CPU0/CPU1 task that owns the physical UART.
 * ISR: not safe.
 */
void uart_comm_service_init(uart_comm_service_t *service, uint32_t baudrate);
void uart_comm_service_set_online(uart_comm_service_t *service, bool online);

/* Send one foreground frame through the configured UART backend. */
bool uart_comm_service_send(uart_comm_service_t *service,
                            const uint8_t *data,
                            size_t size);

#endif /* UART_COMM_SERVICE_H */
