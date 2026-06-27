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
    float target_speed_kph;
    float target_steer_deg;
    bool request_control;
} auto_control_request_t;

typedef struct {
    ecu_command_source_t source;
    ecu_motion_mode_t motion_mode;
    ecu_gear_request_t active_gear;
    float target_speed_kph;
    float target_steer_deg[4];
    float target_height_mm;
    float height_rate_mm_s;
    float track_rate_mm_s;
    bool brake_release;
    bool high_voltage_enable;
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
} vehicle_executor_state_t;

#endif /* VEHICLE_TYPES_H */
