#ifndef LIFT_HYDRAULIC_DEVICE_H
#define LIFT_HYDRAULIC_DEVICE_H

#include <stdbool.h>
#include <stdint.h>

#include "canopen_master_service.h"
#include "ecu_config.h"
#include "ecu_types.h"
#include "vehicle_types.h"

typedef enum {
    HYDRAULIC_PUMP_STATE_STOPPED = 0,
    HYDRAULIC_PUMP_STATE_CONFIGURING,
    HYDRAULIC_PUMP_STATE_STARTING,
    HYDRAULIC_PUMP_STATE_VALVE_READY,
    HYDRAULIC_PUMP_STATE_START_TIMEOUT
} hydraulic_pump_state_t;

typedef enum {
    LIFT_INTERPOLATION_STATE_STOPPED = 0,
    LIFT_INTERPOLATION_STATE_CONFIGURING = 1,
    /* Values 2 and 3 belonged to an obsolete PDO enable sequence.  Keep the
     * remaining diagnostic values stable while setup/enable is now owned by
     * the verified SDO transaction. */
    LIFT_INTERPOLATION_STATE_SETTLING = 4,
    LIFT_INTERPOLATION_STATE_PRELOADING,
    LIFT_INTERPOLATION_STATE_TRIGGERING,
    LIFT_INTERPOLATION_STATE_RUNNING,
    LIFT_INTERPOLATION_STATE_STOPPING,
    LIFT_INTERPOLATION_STATE_FAULT
} lift_interpolation_state_t;

typedef struct {
    uint32_t apply_count;
    uint32_t skipped_lift_canopen_count;
    uint32_t valve_interlock_reject_count;
    uint32_t lift_setup_request_mask;
    uint32_t lift_feedback_fresh_mask;
    uint32_t lift_axis_fault_mask;
    uint32_t lift_interpolation_group_sequence;
    uint32_t lift_interpolation_queued_count;
    uint32_t lift_interpolation_reject_count;
    uint32_t lift_interpolation_failure_count;
    uint32_t lift_interpolation_recovery_count;
    uint32_t lift_hold_count;
    uint32_t pump_group_sequence;
    uint32_t pump_active_group_sequence;
    uint32_t pump_group_complete_count;
    uint32_t pump_group_failure_count;
    uint32_t pump_velocity_queued_count;
    uint32_t pump_velocity_reject_count;
    uint32_t pump_positive_clamp_count;
    uint32_t pump_feedback_reject_count;
    uint32_t pump_start_timeout_count;
    uint32_t pump_setup_reject_count;
    uint32_t pump_setup_failure_count;
    uint32_t pump_start_request_ms;
    uint32_t pump_setup_deadline_ms;
    uint32_t pump_setup_expected_download_count;
    uint32_t pump_setup_abort_count_baseline;
    uint32_t pump_last_speed_ready_ms;
    uint32_t valve_change_hold_until_ms;
    uint32_t last_requested_valve_mask;
    uint32_t pending_valve_mask;
    uint32_t last_valve_mask;
    uint32_t last_interlocked_valve_mask;
    uint32_t last_lift_command_queue_ms;
    uint32_t last_lift_setup_request_ms;
    uint32_t last_lift_interpolation_ms;
    uint32_t lift_feedback_missing_since_ms;
    uint32_t lift_setup_deadline_ms;
    uint32_t lift_setup_nmt_sent_mask;
    uint32_t lift_setup_expected_download_count;
    uint32_t lift_setup_abort_count_baseline;
    uint32_t lift_active_group_sequence;
    uint32_t lift_recovery_not_before_ms;
    uint32_t lift_remote_neutral_since_ms;
    uint32_t lift_enable_settle_until_ms;
    uint32_t lift_settle_sample_ms;
    uint32_t lift_settle_stable_since_ms;
    uint32_t lift_last_stream_step_ms;
    uint32_t last_pump_velocity_ms;
    uint32_t lift_progress_timestamp_ms[ECU_WHEEL_COUNT];
    int32_t lift_actual_position_counts[ECU_WHEEL_COUNT];
    int32_t lift_target_position_counts[ECU_WHEEL_COUNT];
    int32_t lift_stream_origin_position_counts[ECU_WHEEL_COUNT];
    int32_t lift_progress_position_counts[ECU_WHEEL_COUNT];
    int32_t lift_settle_reference_position_counts[ECU_WHEEL_COUNT];
    int32_t lift_command_target_position_counts;
    int32_t lift_stream_planned_delta_counts;
    int32_t lift_stream_total_distance_counts;
    int32_t lift_stream_velocity_counts_per_sec;
    int32_t last_pump_velocity_units;
    int32_t pump_active_velocity_units;
    int32_t pump_actual_velocity_units;
    uint16_t last_pump_controlword;
    uint16_t pump_active_controlword;
    uint8_t pump_speed_ready_samples;
    uint8_t lift_preload_points_completed;
    uint8_t lift_target_stable_samples;
    int8_t lift_requested_direction;
    int8_t lift_active_direction;
    hydraulic_pump_state_t pump_state;
    lift_interpolation_state_t lift_interpolation_state;
    bool pump_feedback_valid;
    bool pump_pressure_ready;
    bool pump_velocity_setup_ready;
    bool pump_setup_in_flight;
    bool pump_group_in_flight;
    bool last_lift_command_valid;
    bool last_pump_velocity_command_valid;
    bool lift_targets_initialized;
    bool lift_progress_initialized;
    bool lift_group_in_flight;
    bool lift_transport_recovery_required;
    bool lift_at_target_disabled;
    bool lift_setup_sdo_queued;
    bool lift_setup_enable_operation;
    bool lift_preload_group_pending;
    bool lift_settle_initialized;
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
 * is clamped to ECU_HYDRAULIC_PUMP_WORK_RPM through
 * ECU_HYDRAULIC_PUMP_MAX_REVERSE_RPM. Field
 * commissioning confirms the physical pump motor must run only in reverse; if
 * a future code path would emit positive motor velocity, this helper clamps it
 * to zero.
 */
int32_t hydraulic_pump_safe_velocity_units(int32_t requested_velocity_units);

/* Apply final lift and track-width hydraulic intent.
 *
 * Units: height is mm, height rate is mm/s, track rate is mm/s.
 * Dependencies: CAN3 lift nodes and hardware configuration.  Valve intent is
 * published to the single CPU0 IO owner by the vehicle executor.
 * Timing: the lift position stream is periodic.  The synchronous pump PDO is
 * sent only when its command changes (or after a confirmed speed-loss retry),
 * so it cannot inject unrelated SYNC frames into the lift interpolation stream.
 * Failure behavior: returns an aggregate result; abort/exit ordering is decided
 * by the vehicle and control layers before this function is called.
 */
ecu_device_apply_result_t lift_hydraulic_device_apply(lift_hydraulic_device_state_t *state,
                                                      canopen_master_service_t *canopen,
                                                      const ecu_hardware_config_t *config,
                                                      const vehicle_actuator_command_t *command,
                                                      uint32_t now_ms);

#endif /* LIFT_HYDRAULIC_DEVICE_H */
