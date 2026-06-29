#include "modbus_master_service.h"

#include <string.h>

static bool time_elapsed(uint32_t now_ms, uint32_t reference_ms, uint32_t period_ms)
{
    return (now_ms - reference_ms) >= period_ms;
}

static bool time_reached(uint32_t now_ms, uint32_t deadline_ms)
{
    return (int32_t)(now_ms - deadline_ms) >= 0;
}

void modbus_master_service_init(modbus_master_service_t *service,
                                uint32_t request_period_ms,
                                uint32_t response_timeout_ms)
{
    if (service == 0) {
        return;
    }

    memset(service, 0, sizeof(*service));
    service->request_period_ms = request_period_ms;
    service->response_timeout_ms = response_timeout_ms;
    service->snapshot.initialized = true;
    service->snapshot.state = MODBUS_MASTER_STATE_IDLE;
}

static void modbus_master_start_request(modbus_master_service_t *service,
                                        uart_rs485_hw_t *uart,
                                        uint32_t now_ms,
                                        const modbus_rtu_frame_t *request,
                                        size_t expected_response_size)
{
    if (expected_response_size == 0U ||
        expected_response_size > sizeof(service->rx_buffer) ||
        request == 0 || request->size == 0U) {
        service->snapshot.error_count++;
        return;
    }

    uart_rs485_hw_clear_rx(uart);
    memset(service->rx_buffer, 0, sizeof(service->rx_buffer));
    service->rx_size = 0U;
    service->expected_response_size = expected_response_size;

    if (!uart_rs485_hw_send(uart, request->data, request->size)) {
        service->snapshot.error_count++;
        return;
    }

    service->snapshot.tx_count++;
    service->snapshot.last_request_ms = now_ms;
    service->snapshot.state = MODBUS_MASTER_STATE_WAIT_RESPONSE;
    service->response_deadline_ms = now_ms + service->response_timeout_ms;
}

static void modbus_master_finish_response(modbus_master_service_t *service,
                                          uint32_t now_ms,
                                          modbus_master_response_handler_t handler,
                                          void *handler_context)
{
    bool ok = false;

    if (handler != 0) {
        ok = handler(handler_context, service->rx_buffer, service->rx_size);
    }

    if (ok) {
        service->snapshot.rx_count++;
        service->snapshot.last_response_ms = now_ms;
        service->snapshot.last_response_size = service->rx_size;
    } else {
        service->snapshot.error_count++;
    }

    service->snapshot.state = MODBUS_MASTER_STATE_IDLE;
    service->rx_size = 0U;
    service->expected_response_size = 0U;
}

void modbus_master_service_process(modbus_master_service_t *service,
                                   uart_rs485_hw_t *uart,
                                   uint32_t now_ms,
                                   const modbus_rtu_frame_t *request,
                                   size_t expected_response_size,
                                   modbus_master_response_handler_t handler,
                                   void *handler_context)
{
    if (service == 0 || uart == 0 || !service->snapshot.initialized ||
        !uart->initialized) {
        return;
    }

    if (service->snapshot.state == MODBUS_MASTER_STATE_IDLE) {
        if (service->snapshot.tx_count == 0U ||
            time_elapsed(now_ms,
                         service->snapshot.last_request_ms,
                         service->request_period_ms)) {
            modbus_master_start_request(service,
                                        uart,
                                        now_ms,
                                        request,
                                        expected_response_size);
        }
        return;
    }

    if (service->snapshot.state == MODBUS_MASTER_STATE_WAIT_RESPONSE) {
        size_t target_size = service->expected_response_size;
        if (target_size == 0U || target_size > sizeof(service->rx_buffer)) {
            target_size = sizeof(service->rx_buffer);
        }

        if (service->rx_size < target_size) {
            service->rx_size += uart_rs485_hw_read(uart,
                                                   &service->rx_buffer[service->rx_size],
                                                   target_size - service->rx_size);
        }

        if (service->expected_response_size > 0U &&
            service->rx_size >= service->expected_response_size) {
            modbus_master_finish_response(service,
                                          now_ms,
                                          handler,
                                          handler_context);
            return;
        }

        if (time_reached(now_ms, service->response_deadline_ms)) {
            service->snapshot.timeout_count++;
            service->snapshot.error_count++;
            service->snapshot.state = MODBUS_MASTER_STATE_IDLE;
            service->rx_size = 0U;
            service->expected_response_size = 0U;
        }
    }
}

void modbus_master_service_get_snapshot(const modbus_master_service_t *service,
                                        modbus_master_snapshot_t *out)
{
    if (service == 0 || out == 0) {
        return;
    }

    *out = service->snapshot;
}
