#ifndef LIFT_HYDRAULIC_DEVICE_H
#define LIFT_HYDRAULIC_DEVICE_H

#include <stdbool.h>

#include "canopen_master_service.h"
#include "dio_service.h"
#include "ecu_config.h"
#include "ecu_types.h"
#include "vehicle_types.h"

typedef struct {
    uint32_t apply_count;
    uint32_t skipped_lift_canopen_count;
    uint32_t last_valve_mask;
    uint32_t last_lift_command_queue_ms;
    bool last_lift_command_valid;
    vehicle_actuator_command_t last_lift_command;
    ecu_device_apply_result_t last_result;
} lift_hydraulic_device_state_t;

/* Initialize the CPU0-owned lift and hydraulic adapter.
 *
 * Owner: task_can3_lift_hydraulic / vehicle executor path on CPU0.
 * ISR: not safe.
 */
void lift_hydraulic_device_init(lift_hydraulic_device_state_t *state);

/* Apply final lift and track-width hydraulic intent.
 *
 * Units: height is mm, height rate is mm/s, track rate is mm/s.
 * Dependencies: CAN3 lift nodes, DIO hydraulic enable/valve masks and config.
 * Timing: unchanged CANopen commands are periodically re-queued because local
 * SDO enqueue success is not the same as remote SDO completion.
 * Failure behavior: returns an aggregate result; abort/exit ordering is decided
 * by the vehicle and control layers before this function is called.
 */
ecu_device_apply_result_t lift_hydraulic_device_apply(lift_hydraulic_device_state_t *state,
                                                      canopen_master_service_t *canopen,
                                                      dio_service_t *dio,
                                                      const ecu_hardware_config_t *config,
                                                      const vehicle_actuator_command_t *command,
                                                      uint32_t now_ms);

#endif /* LIFT_HYDRAULIC_DEVICE_H */
