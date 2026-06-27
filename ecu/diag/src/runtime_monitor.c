#include <stdio.h>

#include "ecu_config.h"
#include "runtime_monitor.h"

static const char *bool_text(bool value)
{
    return value ? "1" : "0";
}

static const char *link_state_text(remote_link_state_t state)
{
    switch (state) {
    case REMOTE_LINK_OFFLINE: return "offline";
    case REMOTE_LINK_QUALIFYING: return "qualifying";
    case REMOTE_LINK_ONLINE: return "online";
    case REMOTE_LINK_FAILSAFE: return "failsafe";
    default: return "unknown";
    }
}

static const char *estop_state_text(remote_estop_state_t state)
{
    switch (state) {
    case REMOTE_ESTOP_CLEAR: return "clear";
    case REMOTE_ESTOP_LATCHED: return "latched";
    case REMOTE_ESTOP_RESET_REQUESTED: return "reset_req";
    case REMOTE_ESTOP_CLEAR_WAIT_NORMAL: return "wait_normal";
    default: return "unknown";
    }
}

static const char *source_text(ecu_command_source_t source)
{
    switch (source) {
    case COMMAND_SOURCE_NONE: return "none";
    case COMMAND_SOURCE_SAFETY: return "safety";
    case COMMAND_SOURCE_REMOTE: return "remote";
    case COMMAND_SOURCE_AUTO: return "auto";
    case COMMAND_SOURCE_MAINTENANCE: return "maint";
    case COMMAND_SOURCE_CPU1: return "cpu1";
    default: return "unknown";
    }
}

static const char *motion_mode_text(ecu_motion_mode_t mode)
{
    switch (mode) {
    case ECU_MOTION_MODE_POSITIVE_ACKERMANN: return "ackermann";
    case ECU_MOTION_MODE_REVERSE_ACKERMANN: return "reverse_ack";
    case ECU_MOTION_MODE_SPIN: return "spin";
    case ECU_MOTION_MODE_CRAB: return "crab";
    default: return "unknown";
    }
}

static const char *gear_text(ecu_gear_request_t gear)
{
    switch (gear) {
    case ECU_GEAR_REQUEST_P: return "P";
    case ECU_GEAR_REQUEST_D: return "D";
    case ECU_GEAR_REQUEST_R: return "R";
    default: return "?";
    }
}

static const char *device_result_text(ecu_device_apply_result_t result)
{
    switch (result) {
    case ECU_DEVICE_APPLY_OK: return "ok";
    case ECU_DEVICE_APPLY_INVALID_ARGUMENT: return "bad_arg";
    case ECU_DEVICE_APPLY_BACKEND_OFFLINE: return "offline";
    case ECU_DEVICE_APPLY_REJECTED: return "rejected";
    default: return "unknown";
    }
}

static void print_centi_value(int32_t value)
{
    uint32_t magnitude;
    if (value < 0) {
        printf("-");
        magnitude = (uint32_t)(-value);
    } else {
        magnitude = (uint32_t)value;
    }
    printf("%lu.%02lu",
           (unsigned long)(magnitude / 100U),
           (unsigned long)(magnitude % 100U));
}

void runtime_monitor_print_cpu0(const runtime_monitor_snapshot_t *snapshot)
{
    if (snapshot == 0) {
        return;
    }

    printf("[ECU MON] t=%lu seq=%lu sbus_valid=%s sbus_conn=%s fs=%s "
           "frames=%lu dec_err=%lu link=%s estop=%s diag=%s\r\n",
           (unsigned long)snapshot->now_ms,
           (unsigned long)snapshot->executor_sequence,
           bool_text(snapshot->sbus_valid),
           bool_text(snapshot->sbus_connected),
           bool_text(snapshot->sbus_failsafe),
           (unsigned long)snapshot->sbus_frame_count,
           (unsigned long)snapshot->sbus_decode_error_count,
           link_state_text(snapshot->link_state),
           estop_state_text(snapshot->estop_state),
           diag_code_name(snapshot->diagnostic));

#if ECU_DEBUG_MONITOR_VERBOSE
    printf("[ECU CMD] src=%s mode=%s gear=%s speed=",
           source_text(snapshot->source),
           motion_mode_text(snapshot->motion_mode),
           gear_text(snapshot->active_gear));
    print_centi_value(snapshot->target_speed_centi_kph);
    printf("kph steer=[");
    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        if (wheel > 0U) {
            printf(",");
        }
        print_centi_value(snapshot->target_steer_centi_deg[wheel]);
    }
    printf("]deg brake=%s hv=%s hyd=%s valve=0x%08lx "
           "res[pwr=%s mot=%s lift=%s io=%s warn=%s]\r\n",
           bool_text(snapshot->brake_release),
           bool_text(snapshot->high_voltage_enable),
           bool_text(snapshot->hydraulic_enable),
           (unsigned long)snapshot->hydraulic_valve_mask,
           device_result_text(snapshot->power_result),
           device_result_text(snapshot->motion_result),
           device_result_text(snapshot->lift_hydraulic_result),
           device_result_text(snapshot->local_io_result),
           device_result_text(snapshot->warning_light_result));
#endif
}
