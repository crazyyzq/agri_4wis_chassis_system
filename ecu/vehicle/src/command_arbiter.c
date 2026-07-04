#include "command_arbiter.h"

#include "ecu_config.h"
#include "motion_control.h"

void vehicle_actuator_command_safe_default(vehicle_actuator_command_t *out)
{
    if (out == 0) {
        return;
    }
    out->source = COMMAND_SOURCE_NONE;
    out->motion_mode = ECU_MOTION_MODE_POSITIVE_ACKERMANN;
    out->active_gear = ECU_GEAR_REQUEST_P;
    out->target_speed_kph = 0.0f;
    for (uint8_t i = 0U; i < ECU_WHEEL_COUNT; ++i) {
        out->target_wheel_speed_kph[i] = 0.0f;
        out->target_steer_deg[i] = 0.0f;
    }
    out->target_height_mm = 0.0f;
    out->height_rate_mm_s = 0.0f;
    out->track_rate_mm_s = 0.0f;
    out->brake_release = false;
    out->steer_commission_interlock_ok = false;
    out->steer_commission_steering_neutral = false;
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

static float clamp_per_mille(int16_t value)
{
    if (value > 1000) {
        return 1000.0f;
    }
    if (value < -1000) {
        return -1000.0f;
    }
    return (float)value;
}

static float scale_signed_per_mille(int16_t per_mille, float full_scale)
{
    return (clamp_per_mille(per_mille) * full_scale) / 1000.0f;
}

static float scale_positive_per_mille(int16_t per_mille, float full_scale)
{
    float clamped = clamp_per_mille(per_mille);
    if (clamped < 0.0f) {
        clamped = 0.0f;
    }
    return (clamped * full_scale) / 1000.0f;
}

static bool motion_mode_reverses_driving_direction(ecu_motion_mode_t mode)
{
    return mode == ECU_MOTION_MODE_REVERSE_ACKERMANN;
}

static float apply_driving_direction_to_speed(ecu_motion_mode_t mode, float speed_kph)
{
    /* Gear and throttle are operator-frame concepts.  In reverse Ackermann the
     * operator drives from the rear-facing frame: D means travel toward the
     * original vehicle rear, so the fixed vehicle-frame speed sign is inverted.
     */
    return motion_mode_reverses_driving_direction(mode) ? -speed_kph : speed_kph;
}

static float remote_speed_command_kph(const remote_control_request_t *remote, ecu_motion_mode_t mode)
{
    if (remote->active_gear == ECU_GEAR_REQUEST_P) {
        return 0.0f;
    }

    float speed = scale_positive_per_mille(remote->throttle_per_mille,
                                           ECU_REMOTE_MAX_SPEED_KPH);
    if (remote->active_gear == ECU_GEAR_REQUEST_R) {
        speed = -speed;
    }
    return apply_driving_direction_to_speed(mode, speed);
}

static bool remote_requests_brake_release(const remote_control_request_t *remote)
{
    if (remote == 0) {
        return false;
    }

    /* D/R arming is a two-step sequence.  The gear FSM first enters ARM_D or
     * ARM_R while the active gear is still P and speed remains zero.  During
     * this arming state the command layer may request drive motion enable, but
     * this request is not a physical brake-release confirmation.  Current
     * firmware keeps the confirmation unavailable until independent feedback is
     * integrated.
     */
    if (remote->gear_state == GEAR_STATE_ARM_D ||
        remote->gear_state == GEAR_STATE_ARM_R) {
        return true;
    }

    /* Track-width adjustment has the same handshake shape.  Requesting motion
     * enable during prepare is only intent; the FSM must not treat it as
     * measured brake feedback.
     */
    if (remote->adjust_state == ADJUST_STATE_TRACK_PREPARE ||
        remote->adjust_state == ADJUST_STATE_TRACK_ACTIVE) {
        return true;
    }

    return remote->active_gear != ECU_GEAR_REQUEST_P;
}

#if ECU_CANOPEN_COMMISSIONING_POLICY == ECU_CANOPEN_COMMISSIONING_POLICY_STEER4_REMOTE_COMMISSIONING
static bool remote_steer_commissioning_interlock_ok(const remote_control_request_t *remote)
{
    return remote != 0 &&
           remote->link_state == REMOTE_LINK_ONLINE &&
           remote->arm_state == REMOTE_ARM_READY &&
           remote->estop_state == REMOTE_ESTOP_CLEAR &&
           remote->active_gear == ECU_GEAR_REQUEST_P &&
           remote->throttle_per_mille == 0;
}

static bool remote_steer_commissioning_steering_neutral(const remote_control_request_t *remote)
{
    return remote != 0 && remote->steer_per_mille == 0;
}
#endif

static void apply_commissioning_steer_only_direct_targets(vehicle_actuator_command_t *out,
                                                         const remote_control_request_t *remote,
                                                         float steer_deg)
{
#if ECU_COMMISSIONING_STEER_ONLY_MODE
    if (out == 0) {
        return;
    }
    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        out->target_wheel_speed_kph[wheel] = 0.0f;
        out->target_steer_deg[wheel] = steer_deg;
    }
    out->target_speed_kph = 0.0f;
#if ECU_CANOPEN_COMMISSIONING_POLICY == ECU_CANOPEN_COMMISSIONING_POLICY_STEER4_REMOTE_COMMISSIONING
    out->active_gear = ECU_GEAR_REQUEST_P;
    out->brake_release = false;
    out->steer_commission_interlock_ok =
        remote_steer_commissioning_interlock_ok(remote);
    out->steer_commission_steering_neutral =
        remote_steer_commissioning_steering_neutral(remote);
    out->hydraulic_enable = false;
    out->hydraulic_valve_mask = 0U;
    out->height_rate_mm_s = 0.0f;
    out->track_rate_mm_s = 0.0f;
#else
    out->brake_release = false;
#endif
#else
    (void)out;
    (void)remote;
    (void)steer_deg;
#endif
}

static ecu_gear_request_t auto_gear_from_speed(float target_speed_kph)
{
    if (target_speed_kph > 0.0f) {
        return ECU_GEAR_REQUEST_D;
    }
    if (target_speed_kph < 0.0f) {
        return ECU_GEAR_REQUEST_R;
    }
    return ECU_GEAR_REQUEST_P;
}

static bool auto_requests_brake_release(const auto_control_request_t *auto_request)
{
    return auto_request != 0 && auto_request->target_speed_kph != 0.0f;
}

static float auto_speed_command_kph(const auto_control_request_t *auto_request)
{
    if (auto_request == 0) {
        return 0.0f;
    }
    /* Automatic requests use the same driver-frame convention as remote
     * requests: positive target speed means "forward in the selected motion
     * mode", not always fixed vehicle +X.
     */
    return apply_driving_direction_to_speed(auto_request->requested_mode,
                                           auto_request->target_speed_kph);
}

static void apply_remote_adjust_command(const remote_control_request_t *remote,
                                        vehicle_actuator_command_t *out)
{
    float height_rate = 0.0f;
    float track_rate = 0.0f;

    if (remote->adjust_state == ADJUST_STATE_CLEARANCE_ACTIVE) {
        height_rate = scale_signed_per_mille(remote->clearance_per_mille,
                                             ECU_REMOTE_MAX_HEIGHT_RATE_MM_S);
        if (height_rate > 0.0f) {
            out->target_height_mm = ECU_REMOTE_MAX_HEIGHT_TARGET_MM;
        } else if (height_rate < 0.0f) {
            out->target_height_mm = ECU_REMOTE_MIN_HEIGHT_TARGET_MM;
        }
    } else if (remote->adjust_state == ADJUST_STATE_TRACK_ACTIVE) {
        track_rate = scale_signed_per_mille(remote->track_per_mille,
                                            ECU_REMOTE_MAX_TRACK_RATE_MM_S);
    }

    out->height_rate_mm_s = height_rate;
    out->track_rate_mm_s = track_rate;
    out->hydraulic_enable = height_rate != 0.0f || track_rate != 0.0f;
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
        motion_control_limits_t limits = {
            .max_speed_kph = ECU_REMOTE_MAX_SPEED_KPH,
            .max_steer_deg = ECU_REMOTE_MAX_STEER_DEG,
            .wheelbase_mm = ECU_VEHICLE_WHEELBASE_MM,
            .track_width_mm = ECU_VEHICLE_TRACK_WIDTH_DEFAULT_MM,
        };
        out->source = COMMAND_SOURCE_REMOTE;
        out->motion_mode = remote->active_motion_mode;
        out->active_gear = remote->active_gear;
        out->brake_release = remote_requests_brake_release(remote);
        out->high_voltage_enable = remote->high_voltage_enable_request;
        out->indicator_mode = remote->indicator_mode;
        out->horn_on = remote->horn_on;
        out->headlight_on = remote->headlight_on;
        out->diagnostic = remote->diagnostic;
        float steer_deg = scale_signed_per_mille(remote->steer_per_mille,
                                                 ECU_REMOTE_MAX_STEER_DEG);
        motion_control_build_candidate(remote->active_motion_mode,
                                       remote_speed_command_kph(remote, remote->active_motion_mode),
                                       steer_deg,
                                       &limits,
                                       out);
        apply_commissioning_steer_only_direct_targets(out, remote, steer_deg);
        apply_remote_adjust_command(remote, out);
#if ECU_CANOPEN_COMMISSIONING_POLICY == ECU_CANOPEN_COMMISSIONING_POLICY_STEER4_REMOTE_COMMISSIONING
        out->target_speed_kph = 0.0f;
        for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
            out->target_wheel_speed_kph[wheel] = 0.0f;
        }
        out->hydraulic_enable = false;
        out->hydraulic_valve_mask = 0U;
        out->height_rate_mm_s = 0.0f;
        out->track_rate_mm_s = 0.0f;
#endif
        return;
    }

    if (remote != 0 && remote->auto_control_allowed &&
        auto_request != 0 && auto_request->valid && auto_request->request_control) {
        out->source = COMMAND_SOURCE_AUTO;
        out->motion_mode = auto_request->requested_mode;
        out->active_gear = auto_gear_from_speed(auto_request->target_speed_kph);
        out->brake_release = auto_requests_brake_release(auto_request);
        motion_control_limits_t limits = {
            .max_speed_kph = ECU_REMOTE_MAX_SPEED_KPH,
            .max_steer_deg = ECU_REMOTE_MAX_STEER_DEG,
            .wheelbase_mm = ECU_VEHICLE_WHEELBASE_MM,
            .track_width_mm = ECU_VEHICLE_TRACK_WIDTH_DEFAULT_MM,
        };
        motion_control_build_candidate(auto_request->requested_mode,
                                       auto_speed_command_kph(auto_request),
                                       auto_request->target_steer_deg,
                                       &limits,
                                       out);
        out->high_voltage_enable = remote->high_voltage_enable_request;
    }
}
