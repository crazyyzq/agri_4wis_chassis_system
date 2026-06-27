#include "ecu_config.h"

static const ecu_config_t s_default_config = {
    .discrete_debounce_ms = REMOTE_DISCRETE_DEBOUNCE_MS,
    .link_qualify_ms = REMOTE_LINK_QUALIFY_MS,
    .neutral_qualify_ms = REMOTE_NEUTRAL_QUALIFY_MS,
    .failsafe_timeout_ms = REMOTE_FAILSAFE_TIMEOUT_MS,
    .domain_event_guard_ms = REMOTE_DOMAIN_EVENT_GUARD_MS,
    .power_long_press_ms = REMOTE_POWER_LONG_PRESS_MS,
    .mode_request_ttl_ms = REMOTE_EVENT_MODE_REQUEST_TTL_MS,
    .power_request_ttl_ms = REMOTE_EVENT_POWER_REQUEST_TTL_MS,
    .estop_reset_ttl_ms = REMOTE_EVENT_ESTOP_RESET_TTL_MS,
    .light_request_ttl_ms = REMOTE_EVENT_LIGHT_REQUEST_TTL_MS,
    .sbus_thresholds = {
        .low_max = ECU_SBUS_RAW_LOW_MAX,
        .center_min = ECU_SBUS_RAW_CENTER_MIN,
        .center_max = ECU_SBUS_RAW_CENTER_MAX,
        .high_min = ECU_SBUS_RAW_HIGH_MIN,
        .neutral = ECU_SBUS_RAW_NEUTRAL,
        .throttle_start = ECU_SBUS_RAW_THROTTLE_START
    },
    .cpu0_task_period_ms = {
        ECU_CPU0_SAFETY_PERIOD_MS,
        ECU_CPU0_CAN2_MOTION_PERIOD_MS,
        ECU_CPU0_REMOTE_PERIOD_MS,
        ECU_CPU0_CONTROL_PERIOD_MS,
        ECU_CPU0_POWER_PERIOD_MS,
        ECU_CPU0_LIFT_HYD_PERIOD_MS,
        ECU_CPU0_IO_PERIOD_MS,
        ECU_CPU0_DIAG_PERIOD_MS
    },
    .cpu1_service_period_ms = ECU_CPU1_SERVICE_PERIOD_MS
};

static const track_adjust_config_t s_track_adjust_config = {
    .steer_target_deg = { 90.0f, -90.0f, -90.0f, 90.0f },
    .assist_torque_sign = { 1.0f, -1.0f, -1.0f, 1.0f },
    .assist_torque_limit_nm = { 8.0f, 8.0f, 8.0f, 8.0f },
    .assist_wheel_speed_limit_rpm = { 12.0f, 12.0f, 12.0f, 12.0f }
};

const ecu_config_t *ecu_config_default(void)
{
    return &s_default_config;
}

const track_adjust_config_t *ecu_track_adjust_config_default(void)
{
    return &s_track_adjust_config;
}
