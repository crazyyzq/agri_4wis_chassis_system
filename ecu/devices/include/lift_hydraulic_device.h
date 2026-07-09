#ifndef LIFT_HYDRAULIC_DEVICE_H
#define LIFT_HYDRAULIC_DEVICE_H

#include <stdbool.h>
#include <stdint.h>

#include "canopen_master_service.h"
#include "dio_service.h"
#include "ecu_config.h"
#include "ecu_types.h"
#include "vehicle_types.h"

typedef enum {
    HYDRAULIC_PUMP_STATE_STOPPED = 0,
    HYDRAULIC_PUMP_STATE_STARTING,
    HYDRAULIC_PUMP_STATE_VALVE_READY,
    HYDRAULIC_PUMP_STATE_START_TIMEOUT
} hydraulic_pump_state_t;

typedef struct {
    uint32_t apply_count;
    uint32_t skipped_lift_canopen_count;
    uint32_t valve_interlock_reject_count;
    uint32_t lift_setup_request_mask;
    uint32_t lift_feedback_fresh_mask;
    uint32_t lift_interpolation_group_sequence;
    uint32_t lift_interpolation_queued_count;
    uint32_t lift_interpolation_reject_count;
    uint32_t lift_interpolation_failure_count;
    uint32_t lift_hold_count;
    uint32_t pump_group_sequence;
    uint32_t pump_velocity_queued_count;
    uint32_t pump_velocity_reject_count;
    uint32_t pump_positive_clamp_count;
    uint32_t pump_feedback_reject_count;
    uint32_t pump_start_timeout_count;
    uint32_t pump_start_request_ms;
    uint32_t valve_change_hold_until_ms;
    uint32_t last_requested_valve_mask;
    uint32_t pending_valve_mask;
    uint32_t last_valve_mask;
    uint32_t last_interlocked_valve_mask;
    uint32_t last_lift_command_queue_ms;
    uint32_t last_lift_setup_request_ms;
    uint32_t last_lift_interpolation_ms;
    uint32_t last_pump_velocity_ms;
    int32_t lift_actual_position_counts[ECU_WHEEL_COUNT];
    int32_t lift_target_position_counts[ECU_WHEEL_COUNT];
    int32_t last_pump_velocity_units;
    int32_t pump_actual_velocity_units;
    uint8_t pump_speed_ready_samples;
    hydraulic_pump_state_t pump_state;
    bool pump_feedback_valid;
    bool last_lift_command_valid;
    bool lift_targets_initialized;
    vehicle_actuator_command_t last_lift_command;
    ecu_device_apply_result_t last_result;
} lift_hydraulic_device_state_t;

/* Initialize the CPU0-owned lift and hydraulic adapter.
 *
 * Owner: task_can3_lift_hydraulic / vehicle executor path on CPU0.
 * ISR: not safe.
 */
void lift_hydraulic_device_init(lift_hydraulic_device_state_t *state);

/* Convert a logical pump request into a motor-side velocity command.
 *
 * Zero means stop.  A positive logical request means "run hydraulic pump" and
 * is clamped to the configured 1000..2400 rpm working range.  Field
 * commissioning confirms the physical pump motor must run only in reverse; if
 * a future code path would emit positive motor velocity, this helper clamps it
 * to zero.
 */
int32_t hydraulic_pump_safe_velocity_units(int32_t requested_velocity_units);

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
