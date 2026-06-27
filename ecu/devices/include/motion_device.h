#ifndef MOTION_DEVICE_H
#define MOTION_DEVICE_H

#include <stdint.h>

#include "can_bus_service.h"
#include "ecu_config.h"
#include "ecu_types.h"
#include "vehicle_types.h"

typedef struct {
    uint32_t apply_count;
    ecu_device_apply_result_t last_result;
    vehicle_actuator_command_t last_motion_command;
} motion_device_state_t;

/* Initialize the CPU0-owned motion device adapter.
 *
 * Owner: task_can2_motion / vehicle executor path on CPU0.
 * ISR: not safe.
 */
void motion_device_init(motion_device_state_t *state);

/* Apply final drive, steering and brake intent to CAN2 motion nodes.
 *
 * Units: speed is kph, steering is degrees, brake_release is logical.
 * Dependencies: CAN2 service and guessed drive/steer CANopen mappings.
 * Failure behavior: returns one aggregate result after attempting configured
 * wheel commands; safety decisions are not made here.
 */
ecu_device_apply_result_t motion_device_apply(motion_device_state_t *state,
                                              can_bus_service_t *can2,
                                              const ecu_hardware_config_t *config,
                                              const vehicle_actuator_command_t *command);

#endif /* MOTION_DEVICE_H */
