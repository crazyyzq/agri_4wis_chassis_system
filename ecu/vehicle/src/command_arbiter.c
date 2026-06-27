#include "command_arbiter.h"

void vehicle_actuator_command_safe_default(vehicle_actuator_command_t *out)
{
    if (out == 0) {
        return;
    }
    out->source = COMMAND_SOURCE_NONE;
    out->motion_mode = ECU_MOTION_MODE_POSITIVE_ACKERMANN;
    out->active_gear = ECU_GEAR_REQUEST_P;
    out->target_speed_kph = 0.0f;
    for (uint8_t i = 0U; i < 4U; ++i) {
        out->target_steer_deg[i] = 0.0f;
    }
    out->target_height_mm = 0.0f;
    out->height_rate_mm_s = 0.0f;
    out->track_rate_mm_s = 0.0f;
    out->brake_release = false;
    out->high_voltage_enable = false;
    out->hydraulic_enable = false;
    out->hydraulic_valve_mask = 0U;
    out->indicator_mode = INDICATOR_OFF;
    out->horn_on = false;
    out->headlight_on = false;
    out->diagnostic = DIAG_OK;
}

static bool remote_has_priority(const remote_control_request_t *remote)
{
    return remote != 0 &&
           remote->link_state == REMOTE_LINK_ONLINE &&
           remote->arm_state == REMOTE_ARM_READY &&
           remote->estop_state == REMOTE_ESTOP_CLEAR &&
           !remote->auto_control_allowed;
}

void command_arbiter_update(const remote_control_request_t *remote,
                            const auto_control_request_t *auto_request,
                            const vehicle_safety_snapshot_t *safety,
                            uint32_t now_ms,
                            vehicle_actuator_command_t *out)
{
    (void)now_ms;
    if (out == 0) {
        return;
    }

    /* complete_rebuild_each_cycle: never patch the previous actuator command. */
    vehicle_actuator_command_safe_default(out);

    if (safety != 0 &&
        (safety->a_class_fault || safety->estop_latched ||
         safety->sbus_failsafe || safety->controlled_stop_active)) {
        out->source = COMMAND_SOURCE_SAFETY;
        out->diagnostic = safety->primary_diag;
        return;
    }

    /* priority: hardware safety > estop/failsafe > controlled stop > remote > auto. */
    if (remote_has_priority(remote)) {
        out->source = COMMAND_SOURCE_REMOTE;
        out->motion_mode = remote->active_motion_mode;
        out->active_gear = remote->active_gear;
        out->brake_release = remote->active_gear != ECU_GEAR_REQUEST_P;
        out->high_voltage_enable = remote->high_voltage_enable_request;
        out->indicator_mode = remote->indicator_mode;
        out->horn_on = remote->horn_on;
        out->headlight_on = remote->headlight_on;
        out->diagnostic = remote->diagnostic;
        return;
    }

    if (remote != 0 && remote->auto_control_allowed &&
        auto_request != 0 && auto_request->valid && auto_request->request_control) {
        out->source = COMMAND_SOURCE_AUTO;
        out->motion_mode = auto_request->requested_mode;
        out->target_speed_kph = auto_request->target_speed_kph;
        out->target_steer_deg[0] = auto_request->target_steer_deg;
        out->target_steer_deg[1] = auto_request->target_steer_deg;
        out->target_steer_deg[2] = auto_request->target_steer_deg;
        out->target_steer_deg[3] = auto_request->target_steer_deg;
        out->high_voltage_enable = remote->high_voltage_enable_request;
    }
}
