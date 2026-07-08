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

typedef struct {
    ecu_command_source_t source;
    ecu_motion_mode_t motion_mode;
    ecu_gear_request_t active_gear;
    float target_speed_mps;
    float target_wheel_speed_mps[ECU_WHEEL_COUNT];
    float target_steer_deg[ECU_WHEEL_COUNT];
    float target_height_mm;
    float height_rate_mm_s;
    float track_rate_mm_s;
    /* High-level permission to request a servo drive into a motion-capable
     * CiA-402 state.  This is not a PCB output level, not a 0x2194/OUT bit,
     * and not proof that a mechanical brake has physically released.
     */
    bool brake_release;
    bool steer_commission_interlock_ok;
    bool steer_commission_steering_neutral;
    bool high_voltage_enable;
    bool high_voltage_disable_request;
    /* True only after the power task has observed validated BMS contactor
     * feedback that says the high-voltage bus is actually present.  Motion
     * devices use this as the servo-enable prerequisite; it is deliberately
     * separate from high_voltage_enable, which is only the operator/safety
     * request that latches MOS8.
     */
    bool high_voltage_feedback_ready;
    bool hydraulic_enable;
    uint32_t hydraulic_valve_mask;
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
    bool steer_normal_pdo_allowed;
    bool steer_safety_inhibited;
    uint8_t steer_inhibit_reason;
    uint32_t steer_safety_inhibit_count;
    uint32_t steer_last_allowed_to_inhibited_ms;
    bool steer_safe_stop_pending;
    uint8_t steer_commission_state;
    uint8_t steer_commission_axis_mask;
    uint8_t steer_commission_nmt_sent_mask;
    uint32_t steer_commission_authorization_clear_count;
    bool steer_commission_post_command_tpdo_pending;
    uint8_t steer_commission_post_command_axis_mask;
    uint8_t steer_commission_post_command_missing_mask;
    uint32_t steer_commission_post_command_timeout_count;
    bool presteer_drive_hold_active;
    bool presteer_target_reached;
    uint8_t presteer_mode;
    uint8_t presteer_missing_axis_mask;
    uint32_t presteer_timeout_count;
} vehicle_executor_state_t;

#endif /* VEHICLE_TYPES_H */
