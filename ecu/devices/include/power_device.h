#ifndef POWER_DEVICE_H
#define POWER_DEVICE_H

#include <stdbool.h>
#include <stdint.h>

#include "can_bus_service.h"
#include "ecu_config.h"
#include "ecu_types.h"

typedef struct {
    bool high_voltage_requested;
    uint32_t apply_count;
    ecu_device_apply_result_t last_result;
} power_device_state_t;

/* Initialize the CPU0-owned power device adapter.
 *
 * Owner: task_can1_power / vehicle executor path on CPU0.
 * ISR: not safe.
 * Failure behavior: null state is ignored.
 */
void power_device_init(power_device_state_t *state);

/* Apply high-voltage intent to CAN1 power devices.
 *
 * Units: high_voltage_enable is a logical request after safety clamping.
 * Dependencies: CAN1 service and guessed CANopen node mappings from config.
 * Failure behavior: returns a device result and records it in state; no retry or
 * delayed execution is hidden inside this function.
 */
ecu_device_apply_result_t power_device_apply(power_device_state_t *state,
                                             can_bus_service_t *can1,
                                             const ecu_hardware_config_t *config,
                                             bool high_voltage_enable);

#endif /* POWER_DEVICE_H */
