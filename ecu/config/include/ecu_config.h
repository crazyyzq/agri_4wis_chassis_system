/* Central calibration and timing configuration for the ECU main application. */
#ifndef ECU_CONFIG_H
#define ECU_CONFIG_H

#include <stdint.h>

#include "ecu_types.h"

#define REMOTE_DISCRETE_DEBOUNCE_MS       (80U)
#define REMOTE_LINK_QUALIFY_MS          (1000U)
#define REMOTE_NEUTRAL_QUALIFY_MS        (300U)
#define REMOTE_FAILSAFE_TIMEOUT_MS       (100U)
#define REMOTE_DOMAIN_EVENT_GUARD_MS     (150U)
#define REMOTE_POWER_LONG_PRESS_MS      (1000U)
#define REMOTE_EVENT_MODE_REQUEST_TTL_MS (250U)
#define REMOTE_EVENT_POWER_REQUEST_TTL_MS (500U)
#define REMOTE_EVENT_ESTOP_RESET_TTL_MS (1000U)
#define REMOTE_EVENT_LIGHT_REQUEST_TTL_MS (1000U)

#define ECU_SBUS_RAW_LOW_MAX             (1050U)
#define ECU_SBUS_RAW_CENTER_MIN          (1400U)
#define ECU_SBUS_RAW_CENTER_MAX          (1600U)
#define ECU_SBUS_RAW_HIGH_MIN            (1950U)
#define ECU_SBUS_RAW_NEUTRAL             (1500U)
#define ECU_SBUS_RAW_THROTTLE_START      (1900U)

#define ECU_CPU0_SAFETY_PERIOD_MS        (1U)
#define ECU_CPU0_CAN2_MOTION_PERIOD_MS   (2U)
#define ECU_CPU0_REMOTE_PERIOD_MS        (5U)
#define ECU_CPU0_CONTROL_PERIOD_MS       (5U)
#define ECU_CPU0_POWER_PERIOD_MS         (10U)
#define ECU_CPU0_LIFT_HYD_PERIOD_MS      (10U)
#define ECU_CPU0_IO_PERIOD_MS            (10U)
#define ECU_CPU0_DIAG_PERIOD_MS          (100U)
#define ECU_CPU1_SERVICE_PERIOD_MS       (20U)

typedef enum {
    ECU_SBUS_CH_STEER = 0,
    ECU_SBUS_CH_CLEARANCE,
    ECU_SBUS_CH_THROTTLE,
    ECU_SBUS_CH_POWER,
    ECU_SBUS_CH_GEAR,
    ECU_SBUS_CH_HOME,
    ECU_SBUS_CH_AUTHORITY,
    ECU_SBUS_CH_LEFT_INDICATOR,
    ECU_SBUS_CH_HAZARD,
    ECU_SBUS_CH_HORN,
    ECU_SBUS_CH_HEADLIGHT,
    ECU_SBUS_CH_RIGHT_INDICATOR,
    ECU_SBUS_CH_ESTOP,
    ECU_SBUS_CH_TRACK,
    ECU_SBUS_CH_R1,
    ECU_SBUS_CH_R2,
    ECU_SBUS_CHANNEL_COUNT
} ecu_sbus_channel_role_t;

typedef struct {
    uint16_t low_max;
    uint16_t center_min;
    uint16_t center_max;
    uint16_t high_min;
    uint16_t neutral;
    uint16_t throttle_start;
} ecu_sbus_thresholds_t;

typedef struct {
    uint32_t discrete_debounce_ms;
    uint32_t link_qualify_ms;
    uint32_t neutral_qualify_ms;
    uint32_t failsafe_timeout_ms;
    uint32_t domain_event_guard_ms;
    uint32_t power_long_press_ms;
    uint32_t mode_request_ttl_ms;
    uint32_t power_request_ttl_ms;
    uint32_t estop_reset_ttl_ms;
    uint32_t light_request_ttl_ms;
    ecu_sbus_thresholds_t sbus_thresholds;
    uint32_t cpu0_task_period_ms[8];
    uint32_t cpu1_service_period_ms;
} ecu_config_t;

typedef struct {
    float steer_target_deg[ECU_WHEEL_COUNT];
    float assist_torque_sign[ECU_WHEEL_COUNT];
    float assist_torque_limit_nm[ECU_WHEEL_COUNT];
    float assist_wheel_speed_limit_rpm[ECU_WHEEL_COUNT];
} track_adjust_config_t;

const ecu_config_t *ecu_config_default(void);
const track_adjust_config_t *ecu_track_adjust_config_default(void);

#endif /* ECU_CONFIG_H */
