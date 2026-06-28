#include <string.h>

#include "FreeRTOS.h"
#include "can_bus_service.h"
#include "task.h"

void can_bus_service_init(can_bus_service_t *service, uint32_t bitrate)
{
    if (service == 0) {
        return;
    }
    memset(service, 0, sizeof(*service));
    service->bitrate = bitrate;
    service->online = true;
    service->diagnostic = DIAG_OK;
}

void can_bus_service_set_online(can_bus_service_t *service, bool online)
{
    if (service != 0) {
        service->online = online;
    }
}

bool can_bus_service_send_canopen(can_bus_service_t *service,
                                  const canopen_frame_t *frame)
{
    if (service == 0 || frame == 0 || !service->online) {
        return false;
    }
    service->last_tx = *frame;
    service->tx_count++;
    service->diagnostic = DIAG_OK;
    return true;
}

void can_bus_service_note_rx_from_isr(can_bus_service_t *service,
                                      uint32_t id,
                                      bool extended,
                                      bool remote,
                                      const uint8_t *data,
                                      uint8_t size)
{
    if (service == 0) {
        return;
    }

    if (size > CANOPEN_FRAME_MAX_DATA_BYTES) {
        size = CANOPEN_FRAME_MAX_DATA_BYTES;
    }

    service->last_rx_id = id;
    service->last_rx_extended = extended;
    service->last_rx_remote = remote;
    service->last_rx_size = size;
    memset(service->last_rx_data, 0, sizeof(service->last_rx_data));
    if (data != 0 && size > 0U) {
        memcpy(service->last_rx_data, data, size);
    }
    service->rx_count++;
    service->diagnostic = DIAG_OK;
}

void can_bus_service_note_error_from_isr(can_bus_service_t *service)
{
    if (service == 0) {
        return;
    }

    service->error_count++;
    service->diagnostic = DIAG_UNKNOWN;
}

void can_bus_service_note_controller_status(can_bus_service_t *service,
                                            uint8_t rx_buffer_status,
                                            uint8_t tx_rx_flags,
                                            uint8_t error_flags,
                                            uint8_t receive_error_count,
                                            uint8_t transmit_error_count,
                                            uint8_t last_error_kind)
{
    if (service == 0) {
        return;
    }

    service->rx_buffer_status = rx_buffer_status;
    service->tx_rx_flags = tx_rx_flags;
    service->error_flags = error_flags;
    service->receive_error_count = receive_error_count;
    service->transmit_error_count = transmit_error_count;
    service->last_error_kind = last_error_kind;
}

void can_bus_service_get_snapshot(const can_bus_service_t *service,
                                  can_bus_service_t *out)
{
    if (service == 0 || out == 0) {
        return;
    }

    /* CAN RX and error accounting may be updated from the controller ISR while
     * the diagnostic task is formatting a monitor line. Keep the critical
     * section limited to the structure copy; printf stays outside.
     */
    taskENTER_CRITICAL();
    *out = *service;
    taskEXIT_CRITICAL();
}
