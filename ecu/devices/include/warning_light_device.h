#ifndef WARNING_LIGHT_DEVICE_H
#define WARNING_LIGHT_DEVICE_H

#include "ecu_config.h"
#include "ecu_types.h"
#include "uart_comm_service.h"
#include "vehicle_types.h"

typedef struct {
    uint32_t apply_count;
    indicator_mode_t last_indicator_mode;
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
 * Dependencies: RS485 UART service and project Modbus slave ID from config.
 * Failure behavior: returns backend-offline when RS485 is unavailable; no
 * blocking retry is performed in this foreground call.
 */
ecu_device_apply_result_t warning_light_device_apply(warning_light_device_state_t *state,
                                                     uart_comm_service_t *rs485,
                                                     const ecu_hardware_config_t *config,
                                                     indicator_mode_t indicator_mode);

#endif /* WARNING_LIGHT_DEVICE_H */
