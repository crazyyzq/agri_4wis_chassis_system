#ifndef WARNING_LIGHT_DEVICE_H
#define WARNING_LIGHT_DEVICE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "ecu_config.h"
#include "ecu_types.h"
#include "modbus_master_service.h"
#include "modbus_rtu.h"
#include "uart_rs485_hw.h"
#include "vehicle_types.h"

typedef struct {
    uint32_t apply_count;
    uint32_t response_count;
    uint32_t error_count;
    uint32_t last_response_ms;
    indicator_mode_t last_indicator_mode;
    uint16_t last_register_value;
    uint8_t expected_response[MODBUS_RTU_MAX_ADU_BYTES];
    size_t expected_response_size;
    bool online;
    ecu_device_apply_result_t last_result;
} warning_light_device_state_t;

/* Initialize the CPU0-owned warning-light adapter.
 *
 * Owner: task_io_service / vehicle executor path on CPU0.
 * ISR: not safe.
 */
void warning_light_device_init(warning_light_device_state_t *state);

/* Send final indicator mode to the RS485 warning-light device.
 *
 * Units: indicator_mode is the final logical lamp state.
 * Dependencies: RS485_2 UART hardware, CPU0 Modbus master state, and project
 * Modbus slave/register settings from config.
 * Failure behavior: returns backend-offline when RS485_2 is unavailable. The
 * foreground call advances one Modbus transaction step and leaves retry timing
 * to modbus_master_service_t.
 */
ecu_device_apply_result_t warning_light_device_apply(warning_light_device_state_t *state,
                                                     modbus_master_service_t *master,
                                                     uart_rs485_hw_t *uart,
                                                     const ecu_hardware_config_t *config,
                                                     indicator_mode_t indicator_mode,
                                                     uint32_t now_ms);

#endif /* WARNING_LIGHT_DEVICE_H */
