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
    out->target_speed_mps = 0.0f;
    for (uint8_t i = 0U; i < ECU_WHEEL_COUNT; ++i) {
        out->target_wheel_speed_mps[i] = 0.0f;
        out->target_steer_deg[i] = 0.0f;
        out->track_assist_current_10ma[i] = 0;
    }
    out->target_height_mm = 0.0f;
    out->height_rate_mm_s = 0.0f;
    out->lift_request = VEHICLE_LIFT_REQUEST_SAFE_STOP;
    out->track_rate_mm_s = 0.0f;
    out->brake_release = false;
    out->steer_commission_interlock_ok = false;
    out->steer_commission_steering_neutral = false;
    out->steer_zero_calibration_request = false;
    out->steer_zero_calibration_domain_active = false;
    out->high_voltage_enable = false;
    out->high_voltage_disable_request = false;
    out->high_voltage_feedback_ready = false;
    out->hydraulic_enable = false;
    out->hydraulic_valve_mask = 0U;
    out->track_assist_requested = false;
    out->track_assist_active = false;
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

static float apply_driving_direction_to_speed(ecu_motion_mode_t mode, float speed_mps)
{
    /* Gear and throttle are operator-frame concepts.  In reverse Ackermann the
     * operator drives from the rear-facing frame: D means travel toward the
     * original vehicle rear, so the fixed vehicle-frame speed sign is inverted.
     */
    return motion_mode_reverses_driving_direction(mode) ? -speed_mps : speed_mps;
}

static float remote_speed_command_mps(const remote_control_request_t *remote, ecu_motion_mode_t mode)
{
    if (remote->active_gear == ECU_GEAR_REQUEST_P) {
        return 0.0f;
    }

    float speed = scale_positive_per_mille(remote->throttle_per_mille,
                                           ECU_REMOTE_MAX_SPEED_MPS);
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
        out->target_wheel_speed_mps[wheel] = 0.0f;
        out->target_steer_deg[wheel] = steer_deg;
    }
    out->target_speed_mps = 0.0f;
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

static ecu_gear_request_t auto_gear_from_speed(float target_speed_mps)
{
    if (target_speed_mps > 0.0f) {
        return ECU_GEAR_REQUEST_D;
    }
    if (target_speed_mps < 0.0f) {
        return ECU_GEAR_REQUEST_R;
    }
    return ECU_GEAR_REQUEST_P;
}

static bool auto_requests_brake_release(const auto_control_request_t *auto_request)
{
    return auto_request != 0 && auto_request->target_speed_mps != 0.0f;
}

static float auto_speed_command_mps(const auto_control_request_t *auto_request)
{
    if (auto_request == 0) {
        return 0.0f;
    }
    /* Automatic requests use the same driver-frame convention as remote
     * requests: positive target speed means "forward in the selected motion
     * mode", not always fixed vehicle +X.
     */
    return apply_driving_direction_to_speed(auto_request->requested_mode,
                                           auto_request->target_speed_mps);
}

static bool remote_in_adjust_domain(const remote_control_request_t *remote)
{
    return remote != 0 &&
           remote->adjust_state != ADJUST_STATE_IDLE;
}

static uint32_t track_width_valve_mask_from_remote(const remote_control_request_t *remote)
{
    if (remote->track_per_mille >= ECU_REMOTE_TRACK_EXTEND_PER_MILLE_MIN) {
        return ECU_HYD_VALVE_TRACK_EXTEND_MASK;
    }
    if (remote->track_per_mille <= ECU_REMOTE_TRACK_RETRACT_PER_MILLE_MAX) {
        return ECU_HYD_VALVE_TRACK_RETRACT_MASK;
    }
    return 0U;
}

static uint32_t suspension_valve_mask_from_remote(const remote_control_request_t *remote)
{
    if (remote == 0 ||
        remote->adjust_state != ADJUST_STATE_HYDRAULIC_ACTIVE ||
        remote->adjust_owner != REMOTE_ADJUST_OWNER_HYDRAULIC) {
        return 0U;
    }

    /* In HOME-center adjustment the physical D/P/R switch is reused only as a
     * suspension direction selector.  P is a hard hold command: no front/rear
     * suspension valve may remain energized merely because the adjustment FSM
     * was previously hydraulic-active.
     */
    if (remote->requested_gear == ECU_GEAR_REQUEST_P) {
        return 0U;
    }
    if (remote->requested_gear == ECU_GEAR_REQUEST_D) {
        return remote->hydraulic_suspension_target == REMOTE_HYDRAULIC_SUSPENSION_FRONT ?
               ECU_HYD_VALVE_FRONT_SUSPENSION_RETRACT_MASK :
               ECU_HYD_VALVE_REAR_SUSPENSION_RETRACT_MASK;
    }
    if (remote->requested_gear == ECU_GEAR_REQUEST_R) {
        return remote->hydraulic_suspension_target == REMOTE_HYDRAULIC_SUSPENSION_FRONT ?
               ECU_HYD_VALVE_FRONT_SUSPENSION_EXTEND_MASK :
               ECU_HYD_VALVE_REAR_SUSPENSION_EXTEND_MASK;
    }
    return 0U;
}

static bool remote_adjust_state_allows_clearance(remote_adjust_state_t state)
{
    return state == ADJUST_STATE_CLEARANCE_ACTIVE;
}

static bool remote_adjust_state_allows_track(remote_adjust_state_t state)
{
    return state == ADJUST_STATE_TRACK_PREPARE ||
           state == ADJUST_STATE_TRACK_ACTIVE;
}

static bool remote_adjust_state_keeps_track_posture(remote_adjust_state_t state)
{
    return remote_adjust_state_allows_track(state) ||
           state == ADJUST_STATE_TRACK_EXITING;
}

static bool remote_adjust_state_allows_suspension(remote_adjust_state_t state)
{
    return state == ADJUST_STATE_HYDRAULIC_ACTIVE;
}

static void apply_track_adjust_steering_posture(vehicle_actuator_command_t *out)
{
    const track_adjust_config_t *track_config = ecu_track_adjust_config_default();

    if (track_config == 0) {
        return;
    }

    /* Track-width adjustment is a CAN2/CAN3 coordinated mode:
     * - steering first moves into the sideways track posture;
     * - CAN3 may open the valve only after the CAN2 presteer gate reports ready;
     * - drive current is enabled later by the executor only after the valve is
     *   actually open.  This function only builds the coherent steering intent.
     *
     * The TRACK_EXITING state also calls this helper so the wheels remain
     * visibly sideways for ECU_REMOTE_TRACK_ASSIST_CENTER_EXIT_MS after CH14
     * returns to center; it does not request valves or assist current.
     */
    out->motion_mode = ECU_MOTION_MODE_CRAB;
    out->active_gear = ECU_GEAR_REQUEST_P;
    out->track_assist_requested = true;
    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        out->target_steer_deg[wheel] = track_config->steer_target_deg[wheel];
        out->target_wheel_speed_mps[wheel] = 0.0f;
    }
}

static void apply_track_adjust_drive_assist(vehicle_actuator_command_t *out,
                                            uint32_t hydraulic_valve_mask)
{
    const track_adjust_config_t *track_config = ecu_track_adjust_config_default();
    const bool extend =
        (hydraulic_valve_mask & ECU_HYD_VALVE_TRACK_EXTEND_MASK) != 0U;
    const bool retract =
        (hydraulic_valve_mask & ECU_HYD_VALVE_TRACK_RETRACT_MASK) != 0U;

    if (track_config == 0 || (!extend && !retract)) {
        return;
    }

    /* This helper only expresses the configured assist-current intent.  The
     * executor clears these currents until CAN3 reports that one of the
     * track-width valves is actually energized, preventing a wheel from
     * pushing a closed hydraulic circuit.
     */
    out->brake_release = true;
    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        float outward_sign = track_config->assist_torque_sign[wheel];
        int16_t configured_current_10ma = track_config->assist_current_10ma[wheel];
        if (configured_current_10ma < 0) {
            configured_current_10ma = (int16_t)-configured_current_10ma;
        }
        int16_t current_10ma =
            outward_sign >= 0.0f ? configured_current_10ma :
                                   (int16_t)-configured_current_10ma;
        if (retract) {
            current_10ma = (int16_t)-current_10ma;
        }
        out->track_assist_current_10ma[wheel] = current_10ma;
    }
}

static void apply_remote_adjust_command(const remote_control_request_t *remote,
                                        vehicle_actuator_command_t *out)
{
    float height_rate = 0.0f;
    float track_rate = 0.0f;

    if (!remote_in_adjust_domain(remote) ||
        remote->adjust_state == ADJUST_STATE_ABORTED) {
        return;
    }

    /* HOME-center neutral is an intentional level-and-hold request.  Safety
     * overrides and leaving the adjustment domain retain SAFE_STOP from the
     * complete command default instead. */
    out->lift_request = VEHICLE_LIFT_REQUEST_NEUTRAL_LEVEL;

    if (remote_adjust_state_allows_clearance(remote->adjust_state) &&
        remote->clearance_per_mille <= ECU_REMOTE_CLEARANCE_DOWN_PER_MILLE_MAX) {
        /* Ground-clearance adjustment is an electric four-leg CAN3 servo
         * function, not a hydraulic-valve function.
         *
         * Field verification on the installed receiver shows the right-stick
         * vertical channel is inverted after SBUS-to-per-mille normalization:
         * physical stick up is negative, and physical stick down is positive.
         * The operator-facing rule is therefore:
         *
         *   right stick up   -> extend legs toward 490 mm
         *   right stick down -> retract legs toward 10 mm
         *   middle band      -> decelerate, level all four legs, then brake
         */
        height_rate = ECU_REMOTE_MAX_HEIGHT_RATE_MM_S;
        out->target_height_mm = ECU_REMOTE_MAX_HEIGHT_TARGET_MM;
        out->lift_request = VEHICLE_LIFT_REQUEST_EXTEND;
    } else if (remote_adjust_state_allows_clearance(remote->adjust_state) &&
               remote->clearance_per_mille >= ECU_REMOTE_CLEARANCE_UP_PER_MILLE_MIN) {
        height_rate = -ECU_REMOTE_MAX_HEIGHT_RATE_MM_S;
        out->target_height_mm = ECU_REMOTE_MIN_HEIGHT_TARGET_MM;
        out->lift_request = VEHICLE_LIFT_REQUEST_RETRACT;
    }

    uint32_t hydraulic_valve_mask =
        remote_adjust_state_allows_track(remote->adjust_state) ?
        track_width_valve_mask_from_remote(remote) : 0U;
    if (remote_adjust_state_keeps_track_posture(remote->adjust_state)) {
        apply_track_adjust_steering_posture(out);
    }
    if (hydraulic_valve_mask != 0U) {
        track_rate = scale_signed_per_mille(remote->track_per_mille,
                                            ECU_REMOTE_MAX_TRACK_RATE_MM_S);
        apply_track_adjust_drive_assist(out, hydraulic_valve_mask);
    }
    if (remote_adjust_state_allows_suspension(remote->adjust_state)) {
        hydraulic_valve_mask |= suspension_valve_mask_from_remote(remote);
    }

    out->height_rate_mm_s = height_rate;
    out->track_rate_mm_s = track_rate;
    out->hydraulic_valve_mask |= hydraulic_valve_mask;
    /* HOME-center adjustment owns both electric lift and hydraulic functions,
     * but the pump is required only when a valve is actually requested.
     * Ground-clearance lift is electric CAN3 servo motion; starting Node13
     * during lift-only motion consumes the same CAN3 setup/realtime lane and
     * can delay the four-axis interpolation stream.  Track-width and
     * suspension requests still set a nonzero valve mask, which starts the pump
     * before the valve-open interlock can pass.
     */
    out->hydraulic_enable = hydraulic_valve_mask != 0U;
}

/* HOME-center adjustment is an actuator domain boundary.
 *
 * The remote may operate CAN3 lift/hydraulic functions in this domain, but it
 * must not retain authority over CAN2 drive or steering.  Build the complete
 * CAN2-safe intent here before returning from the arbiter so CH1, CH3, CH5 and
 * a previously selected steering mode cannot leak into wheel commands.
 */
static void inhibit_can2_motion_in_adjust_domain(vehicle_actuator_command_t *out)
{
    out->motion_mode = ECU_MOTION_MODE_POSITIVE_ACKERMANN;
    out->active_gear = ECU_GEAR_REQUEST_P;
    out->target_speed_mps = 0.0f;
    out->brake_release = false;
    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        out->target_wheel_speed_mps[wheel] = 0.0f;
        out->target_steer_deg[wheel] = 0.0f;
        out->track_assist_current_10ma[wheel] = 0;
    }
    out->track_assist_requested = false;
    out->track_assist_active = false;
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
            .max_speed_mps = ECU_REMOTE_MAX_SPEED_MPS,
            .max_steer_deg = ECU_REMOTE_MAX_STEER_DEG,
            .wheelbase_mm = ECU_VEHICLE_WHEELBASE_MM,
            .track_width_mm = ECU_VEHICLE_TRACK_WIDTH_DEFAULT_MM,
        };
        out->source = COMMAND_SOURCE_REMOTE;
        out->motion_mode = remote->active_motion_mode;
        out->active_gear = remote->active_gear;
        out->brake_release = remote_requests_brake_release(remote);
        out->high_voltage_enable = remote->high_voltage_enable_request;
        out->high_voltage_disable_request = remote->high_voltage_disable_request;
        out->steer_zero_calibration_request =
            remote->steer_zero_calibration_request &&
            remote->active_gear == ECU_GEAR_REQUEST_P &&
            remote->throttle_per_mille == 0;
        out->steer_zero_calibration_domain_active =
            remote_in_adjust_domain(remote);
        out->indicator_mode = remote->indicator_mode;
        out->horn_on = remote->horn_on;
        out->headlight_on = remote->headlight_on;
        out->diagnostic = remote->diagnostic;
        if (remote_in_adjust_domain(remote)) {
            inhibit_can2_motion_in_adjust_domain(out);
            apply_remote_adjust_command(remote, out);
            return;
        }
        /* CH1 field convention: pushing the right stick left must steer the
         * vehicle left.  Positive steering in the vehicle/servo layers already
         * means left, so keep the operator sign correction centralized here
         * instead of scattering per-mode sign flips through Ackermann, crab and
         * spin kinematics.
         */
        float steer_deg = scale_signed_per_mille(
            (int16_t)(remote->steer_per_mille * ECU_REMOTE_STEER_INPUT_SIGN),
            ECU_REMOTE_MAX_STEER_DEG);
        motion_control_build_candidate(remote->active_motion_mode,
                                       remote_speed_command_mps(remote, remote->active_motion_mode),
                                       steer_deg,
                                       &limits,
                                       out);
        apply_commissioning_steer_only_direct_targets(out, remote, steer_deg);
        apply_remote_adjust_command(remote, out);
#if ECU_CANOPEN_COMMISSIONING_POLICY == ECU_CANOPEN_COMMISSIONING_POLICY_STEER4_REMOTE_COMMISSIONING
        out->target_speed_mps = 0.0f;
        for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
            out->target_wheel_speed_mps[wheel] = 0.0f;
        }
        out->hydraulic_enable = false;
        out->hydraulic_valve_mask = 0U;
        out->height_rate_mm_s = 0.0f;
        out->lift_request = VEHICLE_LIFT_REQUEST_SAFE_STOP;
        out->track_rate_mm_s = 0.0f;
#endif
        return;
    }

    if (remote != 0 && remote->auto_control_allowed &&
        auto_request != 0 && auto_request->valid && auto_request->request_control) {
        out->source = COMMAND_SOURCE_AUTO;
        out->motion_mode = auto_request->requested_mode;
        out->active_gear = auto_gear_from_speed(auto_request->target_speed_mps);
        out->brake_release = auto_requests_brake_release(auto_request);
        motion_control_limits_t limits = {
            .max_speed_mps = ECU_REMOTE_MAX_SPEED_MPS,
            .max_steer_deg = ECU_REMOTE_MAX_STEER_DEG,
            .wheelbase_mm = ECU_VEHICLE_WHEELBASE_MM,
            .track_width_mm = ECU_VEHICLE_TRACK_WIDTH_DEFAULT_MM,
        };
        motion_control_build_candidate(auto_request->requested_mode,
                                       auto_speed_command_mps(auto_request),
                                       auto_request->target_steer_deg,
                                       &limits,
                                       out);
        out->high_voltage_enable = remote->high_voltage_enable_request;
        out->high_voltage_disable_request = remote->high_voltage_disable_request;
    }
}
