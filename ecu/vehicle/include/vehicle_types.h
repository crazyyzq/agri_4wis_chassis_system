#ifndef VEHICLE_TYPES_H
#define VEHICLE_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#include "diag_codes.h"
#include "ecu_types.h"
#include "remote_types.h"

typedef struct {
    bool a_class_fault;
    bool estop_latched;
    bool sbus_failsafe;
    bool controlled_stop_active;
    bool shutdown_protect_active;
    bool brake_release_allowed;
    bool zero_speed_confirmed;
    diag_code_t primary_diag;
} vehicle_safety_snapshot_t;

typedef struct {
    bool valid;
    uint32_t sequence;
    uint32_t timestamp_ms;
    ecu_motion_mode_t requested_mode;
    float target_speed_mps;
    float target_steer_deg;
    bool request_control;
} auto_control_request_t;

/* Ground-clearance intent must distinguish an operator's neutral request from
 * a safety stop.  Both have zero height rate, but only operator neutral may
 * keep the four lift axes enabled long enough to level before applying the
 * brakes. */
typedef enum {
    VEHICLE_LIFT_REQUEST_SAFE_STOP = 0,
    VEHICLE_LIFT_REQUEST_NEUTRAL_STOP,
    VEHICLE_LIFT_REQUEST_EXTEND,
    VEHICLE_LIFT_REQUEST_RETRACT
} vehicle_lift_request_t;

typedef struct {
    ecu_command_source_t source;
    ecu_motion_mode_t motion_mode;
    ecu_gear_request_t active_gear;
    float target_speed_mps;
    float target_wheel_speed_mps[ECU_WHEEL_COUNT];
    float target_steer_deg[ECU_WHEEL_COUNT];
    float target_height_mm;
    float height_rate_mm_s;
    vehicle_lift_request_t lift_request;
    float track_rate_mm_s;
    /* High-level permission to request a servo drive into a motion-capable
     * CiA-402 state.  This is not a PCB output level, not a 0x2194/OUT bit,
     * and not proof that a mechanical brake has physically released.
     */
    bool brake_release;
    bool steer_commission_interlock_ok;
    bool steer_commission_steering_neutral;
    bool steer_zero_calibration_request;
    /* True only while HOME remains in the adjustment domain.  The CAN2 owner
     * uses its falling edge to abort an in-progress calibration safely. */
    bool steer_zero_calibration_domain_active;
    bool high_voltage_enable;
    bool high_voltage_disable_request;
    /* True only after the power task has observed validated BMS contactor
     * feedback that says the high-voltage bus is actually present.  Motion
     * devices use this as the servo-enable prerequisite; it is deliberately
     * separate from high_voltage_enable, which is only the operator/safety
     * request that latches the MOS6 battery-key output.
     */
    bool high_voltage_feedback_ready;
    bool hydraulic_enable;
    uint32_t hydraulic_valve_mask;
    bool track_assist_requested;
    bool track_assist_active;
    int16_t track_assist_current_10ma[ECU_WHEEL_COUNT];
    indicator_mode_t indicator_mode;
    bool horn_on;
    bool headlight_on;
    diag_code_t diagnostic;
} vehicle_actuator_command_t;

typedef struct {
    uint32_t applied_sequence;
    vehicle_actuator_command_t last_command;
    ecu_device_apply_result_t power_result;
    ecu_device_apply_result_t motion_result;
    ecu_device_apply_result_t lift_hydraulic_result;
    ecu_device_apply_result_t local_io_result;
    ecu_device_apply_result_t warning_light_result;
    bool high_voltage_relay_latched;
    uint32_t hydraulic_requested_valve_mask;
    uint32_t hydraulic_applied_valve_mask;
    uint32_t hydraulic_interlocked_valve_mask;
    uint32_t hydraulic_valve_interlock_reject_count;
    uint8_t hydraulic_pump_state;
    bool hydraulic_pump_feedback_valid;
    int32_t hydraulic_pump_actual_velocity_units;
    uint32_t hydraulic_pump_start_timeout_count;
    uint8_t lift_interpolation_state;
    int8_t lift_requested_direction;
    int8_t lift_active_direction;
    uint8_t lift_feedback_fresh_mask;
    uint8_t lift_axis_fault_mask;
    uint8_t lift_preload_points_completed;
    uint32_t lift_interpolation_queued_count;
    uint32_t lift_interpolation_reject_count;
    uint32_t lift_interpolation_failure_count;
    uint32_t lift_interpolation_recovery_count;
    uint32_t lift_running_spread_warning_count;
    uint32_t lift_leveling_entry_count;
    uint32_t lift_leveling_complete_count;
    uint32_t lift_range_direction_reject_count;
    uint32_t lift_sync_enable_count;
    int32_t lift_stream_planned_delta_counts;
    int32_t lift_running_spread_counts;
    int32_t lift_max_running_spread_counts;
    int32_t lift_level_target_position_counts;
    uint8_t lift_level_stable_samples;
    uint8_t lift_below_safe_range_mask;
    uint8_t lift_above_safe_range_mask;
    uint8_t lift_mechanical_range_invalid_mask;
    int32_t lift_actual_position_counts[ECU_WHEEL_COUNT];
    int32_t lift_target_position_counts[ECU_WHEEL_COUNT];
    bool steer_normal_pdo_allowed;
    bool steer_safety_inhibited;
    uint8_t steer_inhibit_reason;
    uint32_t steer_safety_inhibit_count;
    uint32_t steer_last_allowed_to_inhibited_ms;
    bool steer_safe_stop_pending;
    uint8_t steer_profile_setup_state;
    uint8_t steer_profile_setup_axis;
    uint8_t steer_profile_setup_object;
    uint8_t steer_profile_verified_mask;
    uint32_t steer_profile_setup_failure_count;
    uint8_t steer_commission_state;
    uint8_t steer_commission_axis_mask;
    uint8_t steer_commission_nmt_sent_mask;
    uint32_t steer_commission_authorization_clear_count;
    bool steer_commission_post_command_tpdo_pending;
    uint8_t steer_commission_post_command_axis_mask;
    uint8_t steer_commission_post_command_missing_mask;
    uint32_t steer_commission_post_command_timeout_count;
    uint32_t can2_realtime_transient_recovery_count;
    uint32_t can2_realtime_consecutive_failure_count;
    uint32_t can2_realtime_last_recovery_ms;
    uint8_t can2_node_recovery_pending_mask;
    uint8_t can2_stale_feedback_mask;
    bool can2_partial_group_recovery_active;
    bool can2_recovery_steer_sync_pending;
    uint32_t can2_partial_group_recovery_count;
    uint32_t can2_command_stale_count;
    uint32_t can2_last_command_stale_ms;
    bool presteer_drive_hold_active;
    bool presteer_target_reached;
    bool track_assist_steer_approximately_ready;
    uint8_t track_assist_overspeed_mask;
    uint8_t track_assist_feedback_invalid_mask;
    uint8_t drive_last_command_kind[ECU_WHEEL_COUNT];
    bool drive_last_enable_requested[ECU_WHEEL_COUNT];
    int16_t drive_last_current_10ma[ECU_WHEEL_COUNT];
    uint32_t drive_group_complete_count;
    uint32_t drive_group_failure_count;
    uint8_t presteer_mode;
    uint8_t presteer_missing_axis_mask;
    uint32_t presteer_timeout_count;
    uint8_t steer_zero_calibration_state;
    uint8_t steer_zero_calibration_done_mask;
    uint8_t steer_zero_calibration_fault_mask;
    uint32_t steer_zero_calibration_request_count;
    int32_t steer_zero_calibration_midpoint_counts[ECU_WHEEL_COUNT];
    int16_t steer_zero_calibration_peak_current_10ma[ECU_WHEEL_COUNT];
    bool steer_transition_active;
    bool steer_transition_completed;
    bool steer_transition_rejected_stale_feedback;
    uint32_t steer_transition_id;
    uint8_t steer_transition_feedback_fresh_mask;
    uint8_t steer_transition_moving_axis_mask;
    int32_t steer_transition_max_distance_counts;
    int32_t steer_transition_actual_counts[ECU_WHEEL_COUNT];
    int32_t steer_transition_output_counts[ECU_WHEEL_COUNT];
    int32_t steer_transition_error_counts[ECU_WHEEL_COUNT];
} vehicle_executor_state_t;

#endif /* VEHICLE_TYPES_H */
