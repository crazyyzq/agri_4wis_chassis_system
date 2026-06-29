/* Remote-control data model and FSM state enums. */
#ifndef REMOTE_TYPES_H
#define REMOTE_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#include "diag_codes.h"
#include "ecu_types.h"

typedef enum {
    REMOTE_POS_LOW = 0,
    REMOTE_POS_CENTER,
    REMOTE_POS_HIGH,
    REMOTE_POS_INVALID
} remote_position_t;

typedef enum {
    REMOTE_LINK_OFFLINE = 0,
    REMOTE_LINK_QUALIFYING,
    REMOTE_LINK_ONLINE,
    REMOTE_LINK_FAILSAFE
} remote_link_state_t;

typedef enum {
    REMOTE_ARM_DISARMED = 0,
    REMOTE_ARM_WAIT_NEUTRAL,
    REMOTE_ARM_READY
} remote_arm_state_t;

typedef enum {
    REMOTE_ESTOP_CLEAR = 0,
    REMOTE_ESTOP_LATCHED,
    REMOTE_ESTOP_RESET_REQUESTED,
    REMOTE_ESTOP_CLEAR_WAIT_NORMAL
} remote_estop_state_t;

typedef enum {
    GEAR_STATE_PARKED_BRAKED = 0,
    GEAR_STATE_ARM_D,
    GEAR_STATE_ARM_R,
    GEAR_STATE_DRIVE_D,
    GEAR_STATE_DRIVE_R,
    GEAR_STATE_STOPPING,
    GEAR_STATE_TRACK_COMPLIANT,
    GEAR_STATE_SHIFT_REJECTED
} remote_gear_state_t;

typedef enum {
    ADJUST_STATE_IDLE = 0,
    ADJUST_STATE_CLEARANCE_ACTIVE,
    ADJUST_STATE_TRACK_PREPARE,
    ADJUST_STATE_TRACK_ACTIVE,
    ADJUST_STATE_TRACK_EXITING,
    ADJUST_STATE_ABORTED
} remote_adjust_state_t;

typedef enum {
    REMOTE_ADJUST_OWNER_NONE = 0,
    REMOTE_ADJUST_OWNER_CLEARANCE,
    REMOTE_ADJUST_OWNER_TRACK
} remote_adjust_owner_t;

typedef enum {
    INDICATOR_OFF = 0,
    INDICATOR_LEFT,
    INDICATOR_RIGHT,
    INDICATOR_HAZARD_USER,
    INDICATOR_HAZARD_SAFETY
} indicator_mode_t;

typedef enum {
    REMOTE_POWER_OFF = 0,
    REMOTE_POWER_ON_REQUESTED,
    REMOTE_POWER_ON,
    REMOTE_POWER_DOWN_REQUESTED,
    REMOTE_POWER_SHUTDOWN_PROTECT,
    REMOTE_POWER_REJECTED
} remote_power_state_t;

typedef enum {
    REMOTE_AUTHORITY_MANUAL = 0,
    REMOTE_AUTHORITY_AUTO_REQUESTED,
    REMOTE_AUTHORITY_AUTO_ACTIVE,
    REMOTE_AUTHORITY_TAKEOVER_WAIT_NEUTRAL,
    REMOTE_AUTHORITY_REJECTED
} remote_authority_state_t;

typedef enum {
    REMOTE_EVENT_MODE_REQUEST = 0,
    REMOTE_EVENT_POWER_REQUEST,
    REMOTE_EVENT_ESTOP_RESET,
    REMOTE_EVENT_LIGHT_REQUEST
} remote_event_type_t;

typedef struct {
    remote_position_t candidate_position;
    remote_position_t stable_position;
    remote_position_t previous_stable_position;
    uint32_t candidate_since_ms;
    uint32_t stable_since_ms;
    bool changed;
} remote_discrete_channel_t;

typedef struct {
    bool link_online;
    bool arm_ready;
    bool estop_latched;
    bool a_class_fault;
    bool zero_speed;
    bool brake_applied;
    bool brake_release_confirmed;
    bool throttle_low;
    bool steering_neutral;
    bool adjustment_active;
    bool active_transition;
    bool external_auto_request_valid;
    bool power_ready;
    bool low_voltage_ok;
    bool can1_power_online;
    bool hydraulic_stopped;
    ecu_gear_request_t requested_gear;
    ecu_gear_request_t active_gear;
} remote_preconditions_t;

typedef struct {
    uint32_t now_ms;
    bool sbus_valid;
    bool sbus_failsafe;
    bool sbus_timeout;
    bool decode_error_limit;
    bool credibility_error;
    remote_position_t ch13_estop;
    remote_position_t gear;
    remote_position_t home;
    remote_position_t clearance;
    remote_position_t throttle;
    remote_position_t steering;
    remote_position_t power;
    remote_position_t authority;
    remote_position_t track;
    remote_position_t r1;
    remote_position_t r2;
    int16_t steer_per_mille;
    int16_t throttle_per_mille;
    int16_t clearance_per_mille;
    int16_t track_per_mille;
    bool r1_changed;
    bool r2_changed;
    remote_position_t left_indicator;
    remote_position_t right_indicator;
    remote_position_t hazard;
    remote_position_t horn;
    remote_position_t headlight;
} remote_input_snapshot_t;

typedef struct {
    remote_link_state_t link_state;
    remote_arm_state_t arm_state;
    remote_estop_state_t estop_state;
    ecu_estop_source_t estop_source;
    remote_gear_state_t gear_state;
    ecu_gear_request_t requested_gear;
    ecu_gear_request_t active_gear;
    ecu_home_domain_t requested_domain;
    ecu_home_domain_t active_domain;
    ecu_motion_mode_t requested_motion_mode;
    ecu_motion_mode_t active_motion_mode;
    remote_adjust_state_t adjust_state;
    remote_adjust_owner_t adjust_owner;
    remote_power_state_t power_state;
    remote_authority_state_t authority_state;
    bool high_voltage_enable_request;
    bool orderly_shutdown_request;
    bool auto_control_allowed;
    int16_t steer_per_mille;
    int16_t throttle_per_mille;
    int16_t clearance_per_mille;
    int16_t track_per_mille;
    indicator_mode_t indicator_mode;
    bool horn_on;
    bool headlight_on;
    bool request_rejected;
    diag_code_t diagnostic;
} remote_control_request_t;

#endif /* REMOTE_TYPES_H */
