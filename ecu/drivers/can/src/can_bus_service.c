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

void can_bus_service_set_tx_backend(can_bus_service_t *service,
                                    can_bus_service_tx_backend_t backend)
{
    if (service != 0) {
        service->tx_backend = backend;
    }
}

bool can_bus_service_send_frame(can_bus_service_t *service,
                                const ecu_can_frame_t *frame)
{
    if (service == 0 || frame == 0 || !service->online) {
        return false;
    }
    if (frame->size > CAN_BUS_FRAME_MAX_DATA_BYTES) {
        return false;
    }

    if (service->tx_backend != 0 && !service->tx_backend(frame)) {
        service->error_count++;
        service->diagnostic = DIAG_UNKNOWN;
        return false;
    }

    service->last_tx_frame = *frame;
    service->tx_count++;
    service->diagnostic = DIAG_OK;
    return true;
}

bool can_bus_service_send_canopen(can_bus_service_t *service,
                                  const canopen_frame_t *frame)
{
    if (service == 0 || frame == 0 || frame->size > CANOPEN_FRAME_MAX_DATA_BYTES) {
        return false;
    }

    ecu_can_frame_t service_frame;
    memset(&service_frame, 0, sizeof(service_frame));
    service_frame.id = frame->cob_id;
    service_frame.size = frame->size;
    service_frame.extended = false;
    service_frame.remote = false;
    memcpy(service_frame.data, frame->data, frame->size);

    if (!can_bus_service_send_frame(service, &service_frame)) {
        return false;
    }

    service->last_tx = *frame;
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

    if (size > CAN_BUS_FRAME_MAX_DATA_BYTES) {
        size = CAN_BUS_FRAME_MAX_DATA_BYTES;
    }

    service->last_rx_id = id;
    service->last_rx_extended = extended;
    service->last_rx_remote = remote;
    service->last_rx_size = size;
    memset(service->last_rx_data, 0, sizeof(service->last_rx_data));
    memset(&service->last_rx_frame, 0, sizeof(service->last_rx_frame));
    service->last_rx_frame.id = id;
    service->last_rx_frame.extended = extended;
    service->last_rx_frame.remote = remote;
    service->last_rx_frame.size = size;
    if (data != 0 && size > 0U) {
        memcpy(service->last_rx_data, data, size);
        memcpy(service->last_rx_frame.data, data, size);
    }
    if (service->rx_queue_count < CAN_BUS_RX_QUEUE_CAPACITY) {
        service->rx_queue[service->rx_queue_head] = service->last_rx_frame;
        service->rx_queue_head =
            (uint8_t)((service->rx_queue_head + 1U) % CAN_BUS_RX_QUEUE_CAPACITY);
        service->rx_queue_count++;
    } else {
        service->rx_queue_dropped_count++;
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

bool can_bus_service_pop_rx_frame(can_bus_service_t *service,
                                  ecu_can_frame_t *out)
{
    bool has_frame = false;

    if (service == 0 || out == 0) {
        return false;
    }

    taskENTER_CRITICAL();
    if (service->rx_queue_count > 0U) {
        *out = service->rx_queue[service->rx_queue_tail];
        service->rx_queue_tail =
            (uint8_t)((service->rx_queue_tail + 1U) % CAN_BUS_RX_QUEUE_CAPACITY);
        service->rx_queue_count--;
        has_frame = true;
    }
    taskEXIT_CRITICAL();
    return has_frame;
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
