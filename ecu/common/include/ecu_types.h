/* Shared ECU domain types. No hardware register or protocol dependency here. */
#ifndef ECU_TYPES_H
#define ECU_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#define ECU_WHEEL_COUNT (4U)

typedef enum {
    ECU_STATUS_OK = 0,
    ECU_STATUS_INVALID_ARGUMENT,
    ECU_STATUS_REJECTED,
    ECU_STATUS_TIMEOUT,
    ECU_STATUS_FAULT
} ecu_status_t;

typedef enum {
    ECU_HOME_DOMAIN_ACKERMANN = 0,
    ECU_HOME_DOMAIN_ADJUST,
    ECU_HOME_DOMAIN_SPECIAL
} ecu_home_domain_t;

typedef enum {
    ECU_MOTION_MODE_POSITIVE_ACKERMANN = 0,
    ECU_MOTION_MODE_REVERSE_ACKERMANN,
    ECU_MOTION_MODE_SPIN,
    ECU_MOTION_MODE_CRAB
} ecu_motion_mode_t;

typedef enum {
    ECU_GEAR_REQUEST_P = 0,
    ECU_GEAR_REQUEST_D,
    ECU_GEAR_REQUEST_R
} ecu_gear_request_t;

typedef enum {
    COMMAND_SOURCE_NONE = 0,
    COMMAND_SOURCE_SAFETY,
    COMMAND_SOURCE_REMOTE,
    COMMAND_SOURCE_AUTO,
    COMMAND_SOURCE_MAINTENANCE,
    COMMAND_SOURCE_CPU1
} ecu_command_source_t;

typedef enum {
    ECU_BRAKE_APPLY = 0,
    ECU_BRAKE_RELEASE
} ecu_brake_command_t;

typedef enum {
    ECU_ESTOP_SOURCE_NONE = 0,
    ECU_ESTOP_SOURCE_CH13,
    ECU_ESTOP_SOURCE_SBUS_TIMEOUT,
    ECU_ESTOP_SOURCE_SBUS_FAILSAFE,
    ECU_ESTOP_SOURCE_DECODE_ERRORS,
    ECU_ESTOP_SOURCE_CREDIBILITY
} ecu_estop_source_t;

typedef enum {
    ECU_DEVICE_APPLY_OK = 0,
    ECU_DEVICE_APPLY_INVALID_ARGUMENT,
    ECU_DEVICE_APPLY_BACKEND_OFFLINE,
    ECU_DEVICE_APPLY_REJECTED
} ecu_device_apply_result_t;

typedef struct {
    uint32_t now_ms;
} ecu_time_snapshot_t;

#endif /* ECU_TYPES_H */
