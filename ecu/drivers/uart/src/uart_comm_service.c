#include <string.h>

#include "uart_comm_service.h"

void uart_comm_service_init(uart_comm_service_t *service, uint32_t baudrate)
{
    if (service == 0) {
        return;
    }
    memset(service, 0, sizeof(*service));
    service->baudrate = baudrate;
    service->online = true;
}

void uart_comm_service_set_online(uart_comm_service_t *service, bool online)
{
    if (service != 0) {
        service->online = online;
    }
}

bool uart_comm_service_send(uart_comm_service_t *service,
                            const uint8_t *data,
                            size_t size)
{
    if (service == 0 || data == 0 || size > UART_COMM_SERVICE_MAX_FRAME_BYTES ||
        !service->online) {
        return false;
    }
    memcpy(service->last_tx, data, size);
    service->last_tx_size = size;
    service->tx_count++;
    return true;
}
