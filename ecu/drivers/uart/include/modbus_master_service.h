#ifndef MODBUS_MASTER_SERVICE_H
#define MODBUS_MASTER_SERVICE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "agile_modbus.h"
#include "agile_modbus_rtu.h"
#include "uart_rs485_hw.h"

#define MODBUS_MASTER_MAX_ADU_BYTES (AGILE_MODBUS_RTU_MAX_ADU_LENGTH)

typedef enum {
    MODBUS_MASTER_STATE_IDLE = 0,
    MODBUS_MASTER_STATE_WAIT_RESPONSE
} modbus_master_state_t;

typedef bool (*modbus_master_response_handler_t)(void *context,
                                                 const uint8_t *adu,
                                                 size_t adu_size);

typedef struct {
    uint8_t data[MODBUS_MASTER_MAX_ADU_BYTES];
    size_t size;
} modbus_master_request_t;

typedef struct {
    bool initialized;
    modbus_master_state_t state;
    uint32_t tx_count;
    uint32_t rx_count;
    uint32_t timeout_count;
    uint32_t error_count;
    uint32_t last_request_ms;
    uint32_t last_response_ms;
    size_t last_response_size;
} modbus_master_snapshot_t;

typedef struct {
    modbus_master_snapshot_t snapshot;
    uint32_t request_period_ms;
    uint32_t response_timeout_ms;
    uint32_t response_deadline_ms;
    uint8_t rx_buffer[MODBUS_MASTER_MAX_ADU_BYTES];
    size_t rx_size;
    size_t expected_response_size;
} modbus_master_service_t;

void modbus_master_service_init(modbus_master_service_t *service,
                                uint32_t request_period_ms,
                                uint32_t response_timeout_ms);

void modbus_master_service_process(modbus_master_service_t *service,
                                   uart_rs485_hw_t *uart,
                                   uint32_t now_ms,
                                   const modbus_master_request_t *request,
                                   size_t expected_response_size,
                                   modbus_master_response_handler_t handler,
                                   void *handler_context);

void modbus_master_service_get_snapshot(const modbus_master_service_t *service,
                                        modbus_master_snapshot_t *out);

#endif /* MODBUS_MASTER_SERVICE_H */
