#ifndef MOTION_DEVICE_H
#define MOTION_DEVICE_H

#include <stdbool.h>
#include <stdint.h>

#include "canopen_master_service.h"
#include "ecu_config.h"
#include "ecu_types.h"
#include "vehicle_types.h"

typedef struct {
    uint32_t apply_count;
    uint32_t skipped_count;
    uint32_t last_motion_command_queue_ms;
    bool last_motion_command_valid;
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
 * Dependencies: CAN2 service and project drive/steer CANopen mappings.
 * Timing: unchanged commands are periodically re-queued because a successful
 * local SDO enqueue does not guarantee the remote drive accepted the transfer.
 * Failure behavior: returns one aggregate result after attempting configured
 * wheel commands; safety decisions are not made here.
 */
ecu_device_apply_result_t motion_device_apply(motion_device_state_t *state,
                                              canopen_master_service_t *canopen,
                                              const ecu_hardware_config_t *config,
                                              const vehicle_actuator_command_t *command,
                                              uint32_t now_ms);

#endif /* MOTION_DEVICE_H */
