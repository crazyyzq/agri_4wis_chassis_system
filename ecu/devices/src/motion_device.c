#include <stdint.h>
#include <limits.h>
#include <string.h>

#include "motion_device.h"
#include "canopen_pdo_profile.h"
#include "hpm_common.h"
#include "motion_setpoint_shaper.h"
#include "servo_drive_canopen.h"
#include "steering_transition_planner.h"

#define ECU_STEER_GROUP_PDO_FRAME_COUNT (ECU_WHEEL_COUNT * 2U)
#define ECU_NODE5_STEER_PDO_FRAME_COUNT (2U)
#define ECU_DRIVE_GROUP_PDO_FRAME_COUNT (ECU_WHEEL_COUNT)
#define ECU_DRIVE_GROUP_SEQUENCE_BASE   (0x40000000UL)
#define ECU_STEER_PROFILE_OBJECT_COUNT  (3U)
#define CIA402_STATUS_STATE_MASK         ((uint16_t)0x006FU)
#define CIA402_STATUS_OPERATION_ENABLED  ((uint16_t)0x0027U)
#define CIA402_STATUS_FAULT_BIT          ((uint16_t)(1U << 3))

ATTR_PLACE_AT_NONCACHEABLE_BSS
volatile ecu_steer_commissioning_control_t g_ecu_steer_commissioning_control;
ATTR_PLACE_AT_NONCACHEABLE_BSS
volatile ecu_steer_calibration_override_t g_ecu_steer_calibration_override;

bool ecu_commissioning_node5_pdo_runtime_authorized(void)
{
    /* This runtime authorization is intentionally false in production images.
     * A Node5-only commissioning image may replace this hook with a physical
     * safety-confirmed authorization path before any real RPDO is emitted.
     */
    return false;
}

static void clear_steer_commissioning_authorization(motion_device_state_t *state)
{
#if !ECU_BUILD_PROFILE_STEER4_REMOTE_90
    g_ecu_steer_commissioning_control.steer_remote_commission_enable = false;
    g_ecu_steer_commissioning_control.enabled_axis_mask = 0U;
#endif
    if (state != NULL) {
        state->steer_commission_authorization_clear_count++;
        state->selected_axis_mask = 0U;
        state->steer_commission_nmt_sent_mask = 0U;
        state->steer_commission_neutral_since_ms = 0U;
        state->steer_commission_last_sync_ms = 0U;
        state->steer_commission_sync_wait_start_ms = 0U;
        state->steer_commission_sync_complete_ms = 0U;
        state->steer_commission_sync_complete_count_before = 0U;
        state->steer_commission_centered = false;
        state->steer_commission_post_command_is_centering = false;
        state->steer_commission_post_command_tpdo_pending = false;
        state->steer_commission_post_command_axis_mask = 0U;
        state->steer_commission_post_command_missing_mask = 0U;
        state->steer_commission_post_command_start_ms = 0U;
        for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
            state->steer_commission_ramped_target_valid[wheel] = false;
            state->steer_commission_ramped_target_counts[wheel] = 0;
        }
        state->steer_commission_ramp_last_ms = 0U;
        state->steer_next_group_valid = false;
        state->steer_commission_state = STEER_REMOTE_COMMISSION_WAIT_AUTH;
    }
}

static bool commissioning_policy_allows_node5_steer_pdo(void)
{
    return ECU_CANOPEN_COMMISSIONING_POLICY ==
           ECU_CANOPEN_COMMISSIONING_POLICY_NODE5_STEER_PDO_ONLY;
}

static bool commissioning_policy_allows_full_steer_pdo(void)
{
    return ECU_CANOPEN_COMMISSIONING_POLICY ==
           ECU_CANOPEN_COMMISSIONING_POLICY_PDO_OUTPUT_ENABLED;
}

static bool commissioning_policy_allows_steer4_remote(void)
{
    return ECU_CANOPEN_COMMISSIONING_POLICY ==
           ECU_CANOPEN_COMMISSIONING_POLICY_STEER4_REMOTE_COMMISSIONING;
}

static bool commissioning_policy_allows_drive_rpdo(void)
{
    return ECU_CANOPEN_COMMISSIONING_POLICY ==
           ECU_CANOPEN_COMMISSIONING_POLICY_PDO_OUTPUT_ENABLED;
}

static bool commissioning_policy_allows_can3_rpdo(void)
{
    /* CAN3 lift/hydraulic RPDO output is owned by the CAN3 task in a later
     * phase.  Keep it closed while steering commissioning is being corrected.
     */
    return false;
}

static int32_t scaled_float_to_i32(float value, float scale)
{
    float scaled = value * scale;
    if (scaled > 2147483647.0f) {
        return 2147483647;
    }
    if (scaled < -2147483648.0f) {
        return INT32_MIN;
    }
    return (int32_t)scaled;
}

static int32_t clamp_i32(int32_t value, int32_t min_value, int32_t max_value)
{
    if (value > max_value) {
        return max_value;
    }
    if (value < min_value) {
        return min_value;
    }
    return value;
}

static float clamp_f32(float value, float min_value, float max_value)
{
    if (value > max_value) {
        return max_value;
    }
    if (value < min_value) {
        return min_value;
    }
    return value;
}

static bool drive_direction_sign_is_valid(int8_t sign)
{
    return sign == 1 || sign == -1;
}

static bool can2_feedback_operation_enabled(
    uint8_t node_id,
    const canopen_node_feedback_t *feedback)
{
    if (feedback == NULL || !feedback->feedback_fresh ||
        feedback->fault_latched != 0U) {
        return false;
    }
    if (node_id == ECU_CANOPEN_STEER_RR_NODE_ID &&
        ECU_CANOPEN_NODE8_TPDO1_ACCEPTANCE_WORKAROUND != 0U &&
        feedback->tpdo1_rx_count == 0U) {
        return true;
    }
    return feedback->tpdo1_fresh &&
           (feedback->statusword & CIA402_STATUS_STATE_MASK) ==
               CIA402_STATUS_OPERATION_ENABLED;
}

static bool can2_steer_boot_heartbeat_evidence_ready(
    const canopen_node_feedback_t *feedback,
    uint32_t now_ms)
{
#if ECU_CANOPEN_STEER_BOOT_HEARTBEAT_EVIDENCE_ENABLED
    if (feedback == NULL || !feedback->heartbeat_valid ||
        !feedback->heartbeat_operational_seen ||
        feedback->fault_latched != 0U) {
        return false;
    }
    return (uint32_t)(now_ms - feedback->last_heartbeat_ms) <=
           ECU_CANOPEN_HEARTBEAT_TIMEOUT_MS;
#else
    (void)feedback;
    (void)now_ms;
    return false;
#endif
}

static void mark_steer_axis_ready_from_startup_evidence(
    motion_device_state_t *state,
    uint32_t wheel,
    bool has_position_feedback,
    int32_t position_counts)
{
    state->steer_axis_remote_verified[wheel] = true;
    state->steer_axis_config_state[wheel] = MOTION_STEER_AXIS_READY;
    state->steer_pdo_configured[wheel] = true;
    state->steer_position_mode_ready[wheel] = true;
    if (has_position_feedback) {
        state->steer_last_position_valid[wheel] = true;
        state->steer_last_position_counts[wheel] = position_counts;
    }
}

static bool steer_axis_calibration_is_valid(const steer_axis_calibration_t *axis)
{
    if (axis == NULL || !axis->valid) {
        return false;
    }
    if (axis->direction_sign != 1 && axis->direction_sign != -1) {
        return false;
    }
    if (axis->minimum_position_counts > axis->maximum_position_counts) {
        return false;
    }
    if (axis->straight_zero_offset_counts < axis->minimum_position_counts ||
        axis->straight_zero_offset_counts > axis->maximum_position_counts) {
        return false;
    }
    if (axis->minimum_position_counts < -ECU_CANOPEN_STEER_MAX_POSITION_COUNTS ||
        axis->maximum_position_counts > ECU_CANOPEN_STEER_MAX_POSITION_COUNTS) {
        return false;
    }
    if (axis->commissioning_max_abs_deg <= 0.0f ||
        axis->commissioning_max_abs_deg > ECU_STEER_REMOTE_COMMISSION_MAX_DEG) {
        return false;
    }
    return true;
}

static void copy_default_steer_calibration(
    const ecu_hardware_config_t *config,
    steer_axis_calibration_t out_calibration[ECU_WHEEL_COUNT])
{
    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        out_calibration[wheel] = config->steer_axis_calibration[wheel];
    }
}

bool motion_device_get_effective_steer_calibration(
    const ecu_hardware_config_t *config,
    uint8_t enabled_axis_mask,
    steer_axis_calibration_t out_calibration[ECU_WHEEL_COUNT],
    bool *using_ram_override)
{
    if (config == NULL || out_calibration == NULL) {
        return false;
    }

    if (using_ram_override != NULL) {
        *using_ram_override = false;
    }
    copy_default_steer_calibration(config, out_calibration);

    uint8_t selected_mask =
        enabled_axis_mask & ECU_STEER_REMOTE_COMMISSION_AXIS_MASK_ALL;
    if (selected_mask == 0U) {
        return true;
    }

    if (g_ecu_steer_calibration_override.magic !=
            ECU_STEER_CALIBRATION_OVERRIDE_MAGIC ||
        !g_ecu_steer_calibration_override.enable) {
        for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
            if ((selected_mask & (uint8_t)(1U << wheel)) == 0U) {
                continue;
            }
            if (!steer_axis_calibration_is_valid(&out_calibration[wheel])) {
                return false;
            }
        }
        return true;
    }

    steer_axis_calibration_t override_axis[ECU_WHEEL_COUNT];
    uint32_t sequence_before =
        __atomic_load_n(&g_ecu_steer_calibration_override.sequence,
                        __ATOMIC_ACQUIRE);
    if ((sequence_before & 1U) != 0U) {
        return false;
    }
    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        override_axis[wheel] = g_ecu_steer_calibration_override.axis[wheel];
    }
    uint32_t sequence_after =
        __atomic_load_n(&g_ecu_steer_calibration_override.sequence,
                        __ATOMIC_ACQUIRE);

    /* J-Link RAM calibration updates use an odd/even sequence protocol:
     * odd means "writer is editing", even means "stable".  Reject unstable or
     * torn reads so a half-written zero/min/max never reaches the actuator path.
     */
    if (sequence_before != sequence_after || (sequence_after & 1U) != 0U) {
        return false;
    }

    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        if ((selected_mask & (uint8_t)(1U << wheel)) == 0U) {
            continue;
        }
        if (!steer_axis_calibration_is_valid(&override_axis[wheel])) {
            return false;
        }
    }

    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        out_calibration[wheel] = override_axis[wheel];
    }
    if (using_ram_override != NULL) {
        *using_ram_override = true;
    }
    return true;
}

bool steer_commissioning_build_targets(
    const steer_axis_calibration_t calibration[ECU_WHEEL_COUNT],
    uint8_t enabled_axis_mask,
    float remote_steer_deg,
    int32_t out_target_counts[ECU_WHEEL_COUNT])
{
    if (calibration == NULL || out_target_counts == NULL ||
        (enabled_axis_mask & ECU_STEER_REMOTE_COMMISSION_AXIS_MASK_ALL) == 0U) {
        return false;
    }

    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        out_target_counts[wheel] = 0;
    }

    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        uint8_t axis_bit = (uint8_t)(1U << wheel);
        if ((enabled_axis_mask & axis_bit) == 0U) {
            continue;
        }

        const steer_axis_calibration_t *axis = &calibration[wheel];
        if (!axis->valid ||
            (axis->direction_sign != 1 && axis->direction_sign != -1) ||
            axis->minimum_position_counts > axis->maximum_position_counts ||
            axis->commissioning_max_abs_deg <= 0.0f) {
            return false;
        }

        float axis_limit = clamp_f32(axis->commissioning_max_abs_deg,
                                     0.0f,
                                     ECU_STEER_REMOTE_COMMISSION_MAX_DEG);
        float limited_deg = clamp_f32(remote_steer_deg, -axis_limit, axis_limit);
        float delta_f = limited_deg * ECU_STEER_DEG_TO_COUNTS *
                        (float)axis->direction_sign;
        if (delta_f > 2147483647.0f || delta_f < -2147483648.0f) {
            return false;
        }

        int32_t delta_counts = (int32_t)delta_f;
        if ((delta_counts > 0 &&
             axis->straight_zero_offset_counts > INT32_MAX - delta_counts) ||
            (delta_counts < 0 &&
             axis->straight_zero_offset_counts < INT32_MIN - delta_counts)) {
            return false;
        }

        int32_t target = axis->straight_zero_offset_counts + delta_counts;
        out_target_counts[wheel] = clamp_i32(target,
                                             axis->minimum_position_counts,
                                             axis->maximum_position_counts);
    }

    return true;
}

static int32_t abs_i32_delta(int32_t a, int32_t b)
{
    int32_t delta = a - b;
    return delta < 0 ? -delta : delta;
}

static bool i32_changed_beyond_deadband(int32_t previous,
                                        int32_t current,
                                        int32_t deadband)
{
    return abs_i32_delta(previous, current) >= deadband;
}

static bool drive_output_allowed(const vehicle_actuator_command_t *command)
{
    if (command == NULL || !commissioning_policy_allows_drive_rpdo()) {
        return false;
    }
#if ECU_COMMISSIONING_STEER_ONLY_MODE
    return false;
#else
    return command->brake_release;
#endif
}

static bool motion_mode_requires_presteer(ecu_motion_mode_t mode)
{
    return mode == ECU_MOTION_MODE_SPIN ||
           mode == ECU_MOTION_MODE_CRAB;
}

static bool command_requests_drive_motion(const vehicle_actuator_command_t *command)
{
    if (command == NULL) {
        return false;
    }

    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        float speed_mps = command->target_wheel_speed_mps[wheel];
        if (speed_mps > 0.001f || speed_mps < -0.001f) {
            return true;
        }
    }
    return false;
}

static bool steering_feedback_is_at_targets(
    motion_device_state_t *state,
    const canopen_master_service_t *canopen,
    const ecu_hardware_config_t *config,
    const int32_t target_counts[ECU_WHEEL_COUNT],
    int32_t tolerance_counts,
    uint8_t *missing_axis_mask)
{
    uint8_t missing_mask = 0U;

    if (state == NULL || canopen == NULL || config == NULL ||
        target_counts == NULL) {
        if (missing_axis_mask != NULL) {
            *missing_axis_mask = ECU_CANOPEN_PRESTEER_REQUIRED_AXIS_MASK;
        }
        return false;
    }

    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        uint8_t axis_bit = (uint8_t)(1U << wheel);
        uint8_t node_id = config->steer_nodes[wheel].node_id;
        canopen_node_feedback_t feedback;

        if ((ECU_CANOPEN_PRESTEER_REQUIRED_AXIS_MASK & axis_bit) == 0U) {
            continue;
        }

        bool feedback_ready =
            canopen_master_service_get_node_feedback(canopen,
                                                     node_id,
                                                     &feedback) &&
            feedback.feedback_fresh &&
            feedback.fault_latched == 0U;
        if (!feedback_ready) {
            missing_mask |= axis_bit;
            continue;
        }

        state->steer_last_position_valid[wheel] = true;
        state->steer_last_position_counts[wheel] =
            feedback.actual_position_counts;
        if (abs_i32_delta(feedback.actual_position_counts,
                          target_counts[wheel]) >
            tolerance_counts) {
            missing_mask |= axis_bit;
        }
    }

    if (missing_axis_mask != NULL) {
        *missing_axis_mask = missing_mask;
    }
    return missing_mask == 0U;
}

static bool presteer_gate_allows_drive(
    motion_device_state_t *state,
    const canopen_master_service_t *canopen,
    const ecu_hardware_config_t *config,
    const vehicle_actuator_command_t *command,
    const int32_t steer_target_counts[ECU_WHEEL_COUNT],
    bool drive_allowed_by_safety,
    uint32_t now_ms)
{
    bool requires_presteer =
        command != NULL && motion_mode_requires_presteer(command->motion_mode);
    bool drive_motion_requested = command_requests_drive_motion(command);
    bool track_assist_requested =
        command != NULL && command->track_assist_requested;
    bool presteer_check_needed =
        requires_presteer &&
        ((drive_allowed_by_safety && drive_motion_requested) ||
         track_assist_requested);

    if (state == NULL || !presteer_check_needed) {
        if (state != NULL) {
            state->presteer_drive_hold_active = false;
            state->presteer_target_reached = false;
            state->track_assist_steer_approximately_ready = false;
            state->presteer_missing_axis_mask = 0U;
            state->track_assist_missing_axis_mask = 0U;
            state->presteer_hold_start_ms = 0U;
            state->track_assist_steer_ready_since_ms = 0U;
            state->track_assist_steer_ready_eval_ms = 0U;
            state->presteer_mode =
                command != NULL ? command->motion_mode : ECU_MOTION_MODE_POSITIVE_ACKERMANN;
        }
        return true;
    }

    state->presteer_mode = command->motion_mode;
    uint8_t missing_mask = 0U;
    bool target_reached = steering_feedback_is_at_targets(state,
                                                          canopen,
                                                          config,
                                                          steer_target_counts,
                                                          ECU_CANOPEN_PRESTEER_POSITION_TOLERANCE_COUNTS,
                                                          &missing_mask);
    uint8_t track_assist_missing_mask = 0U;
    bool track_assist_ready_raw =
        track_assist_requested &&
        steering_feedback_is_at_targets(state,
                                        canopen,
                                        config,
                                        steer_target_counts,
                                        ECU_TRACK_ASSIST_STEER_APPROX_TOLERANCE_COUNTS,
                                        &track_assist_missing_mask);
    if (track_assist_requested) {
        /* CAN3 valve gating is task-decoupled from CAN2 steering.  Record the
         * time at which CAN2 evaluated the current track-width steering target
         * so the CAN3 task can reject a stale "ready" bit left over from a
         * previous track-width session.
         */
        state->track_assist_steer_ready_eval_ms = now_ms;
    }
    state->track_assist_missing_axis_mask = track_assist_missing_mask;
    if (!track_assist_ready_raw) {
        state->track_assist_steer_approximately_ready = false;
        state->track_assist_steer_ready_since_ms = 0U;
    } else {
        if (state->track_assist_steer_ready_since_ms == 0U) {
            state->track_assist_steer_ready_since_ms = now_ms;
        }
        state->track_assist_steer_approximately_ready =
            (uint32_t)(now_ms - state->track_assist_steer_ready_since_ms) >=
            ECU_TRACK_ASSIST_STEER_READY_STABLE_MS;
    }
    state->presteer_target_reached = target_reached;
    state->presteer_missing_axis_mask = missing_mask;

    if (target_reached) {
        state->presteer_drive_hold_active = false;
        state->presteer_hold_start_ms = 0U;
        return true;
    }

    if (!state->presteer_drive_hold_active) {
        state->presteer_hold_start_ms = now_ms;
    }
    state->presteer_drive_hold_active = true;

    if (state->presteer_hold_start_ms != 0U &&
        (uint32_t)(now_ms - state->presteer_hold_start_ms) >=
            ECU_CANOPEN_PRESTEER_TIMEOUT_MS) {
        state->presteer_timeout_count++;
        state->presteer_last_timeout_ms = now_ms;
        state->presteer_hold_start_ms = now_ms;
    }

    /* While held, steering targets continue through the normal PDO path but
     * drive RPDOs are cached as disabled/zero.  This prevents spin/crab from
     * rolling sideways before the steering feedback window is reached.
     */
    return false;
}

static uint8_t collect_steering_feedback_for_planner(
    motion_device_state_t *state,
    const canopen_master_service_t *canopen,
    const ecu_hardware_config_t *config,
    int32_t actual_position_counts[ECU_WHEEL_COUNT])
{
    uint8_t feedback_fresh_mask = 0U;

    if (state == NULL || canopen == NULL || config == NULL ||
        actual_position_counts == NULL) {
        return 0U;
    }

    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        uint8_t node_id = config->steer_nodes[wheel].node_id;
        canopen_node_feedback_t feedback;
        bool fresh =
            canopen_master_service_get_node_feedback(canopen,
                                                     node_id,
                                                     &feedback) &&
            feedback.feedback_fresh &&
            feedback.fault_latched == 0U;
        if (fresh) {
            state->steer_last_position_valid[wheel] = true;
            state->steer_last_position_counts[wheel] =
                feedback.actual_position_counts;
            actual_position_counts[wheel] = feedback.actual_position_counts;
            feedback_fresh_mask |= (uint8_t)(1U << wheel);
        } else if (state->steer_last_position_valid[wheel]) {
            actual_position_counts[wheel] = state->steer_last_position_counts[wheel];
        } else if (state->steer_last_commanded_position_valid[wheel]) {
            actual_position_counts[wheel] =
                state->steer_last_commanded_position_counts[wheel];
        } else {
            actual_position_counts[wheel] = 0;
        }
    }
    return feedback_fresh_mask;
}

static bool steer_limit_blocks_target(canopen_master_service_t *canopen,
                                      const ecu_canopen_node_config_t *node,
                                      motion_device_state_t *state,
                                      uint32_t wheel,
                                      float steer_deg,
                                      uint32_t now_ms)
{
#if ECU_COMMISSIONING_STEER_ONLY_MODE || !ECU_CANOPEN_STEER_LIMIT_INPUT_GATING_ENABLED
    /* During steering/whole-vehicle commissioning the drive input bits may be
     * unconfigured, inverted, or not yet wired.  Still allow the diagnostic
     * readback path elsewhere to observe 0x2190, but do not turn an unverified
     * IN2/IN3 bit into a quick-stop that blocks one steering direction.  Enable
     * ECU_CANOPEN_STEER_LIMIT_INPUT_GATING_ENABLED only after the polarity and
     * wiring are hardware-verified.
     */
    (void)canopen;
    (void)node;
    (void)state;
    (void)wheel;
    (void)steer_deg;
    (void)now_ms;
    return false;
#else
    uint16_t input_states = 0U;
    bool read_due = (uint32_t)(now_ms - state->steer_last_limit_read_ms[wheel]) >=
                    ECU_CANOPEN_MOTION_COMMAND_REFRESH_MS;
    if (read_due &&
        servo_drive_canopen_read_input_states(canopen, node, &input_states)) {
        state->steer_positive_limit[wheel] =
            (input_states & SERVO_DRIVE_INPUT_IN2_MASK) != 0U;
        state->steer_negative_limit[wheel] =
            (input_states & SERVO_DRIVE_INPUT_IN3_MASK) != 0U;
        state->steer_last_limit_read_ms[wheel] = now_ms;
    }

    return (steer_deg > 0.0f && state->steer_positive_limit[wheel]) ||
           (steer_deg < 0.0f && state->steer_negative_limit[wheel]);
#endif
}

static int32_t normalize_steer_target_counts(int32_t position_counts)
{
    position_counts = clamp_i32(position_counts,
                                -ECU_CANOPEN_STEER_MAX_POSITION_COUNTS,
                                ECU_CANOPEN_STEER_MAX_POSITION_COUNTS);
    if (position_counts > -ECU_CANOPEN_STEER_POSITION_NEUTRAL_DEADBAND_COUNTS &&
        position_counts < ECU_CANOPEN_STEER_POSITION_NEUTRAL_DEADBAND_COUNTS) {
        return 0;
    }
    return position_counts;
}

static bool cache_latest_steer_target(motion_device_state_t *state,
                                      uint32_t wheel,
                                      int32_t position_counts)
{
    position_counts = normalize_steer_target_counts(position_counts);
    bool changed = !state->steer_latest_target_valid[wheel] ||
                   i32_changed_beyond_deadband(
                       state->steer_latest_target_counts[wheel],
                       position_counts,
                       ECU_CANOPEN_STEER_POSITION_TRIGGER_THRESHOLD_COUNTS);
    state->steer_latest_target_counts[wheel] = position_counts;
    state->steer_latest_target_valid[wheel] = true;
    if (changed) {
        state->steer_pending_target[wheel] = true;
    }
    return true;
}

static uint16_t steer_profile_object_index(uint8_t object)
{
    static const uint16_t indexes[ECU_STEER_PROFILE_OBJECT_COUNT] = {
        ECU_CANOPEN_OBJ_PROFILE_VELOCITY,
        ECU_CANOPEN_OBJ_PROFILE_ACCELERATION,
        ECU_CANOPEN_OBJ_PROFILE_DECELERATION
    };
    return object < ECU_STEER_PROFILE_OBJECT_COUNT ? indexes[object] : 0U;
}

static int32_t steer_profile_object_value(uint8_t object)
{
    static const int32_t values[ECU_STEER_PROFILE_OBJECT_COUNT] = {
        ECU_STEER_PROFILE_VELOCITY_UNITS,
        ECU_STEER_PROFILE_ACCEL_COUNTS_PER_SEC2,
        ECU_STEER_PROFILE_DECEL_COUNTS_PER_SEC2
    };
    return object < ECU_STEER_PROFILE_OBJECT_COUNT ? values[object] : 0;
}

static void steer_profile_setup_reset(motion_device_state_t *state)
{
    if (state == NULL) {
        return;
    }
    state->steer_profile_setup_state = MOTION_STEER_PROFILE_WRITE_REQUEST;
    state->steer_profile_setup_axis = 0U;
    state->steer_profile_setup_object = 0U;
    state->steer_profile_verified_mask = 0U;
    state->steer_profile_setup_start_ms = 0U;
    memset(state->steer_profile_readback, 0, sizeof(state->steer_profile_readback));
}

static void steer_profile_setup_retry(motion_device_state_t *state,
                                      uint32_t now_ms)
{
    uint8_t axis = state->steer_profile_setup_axis;
    state->steer_profile_setup_failure_count++;
    if (axis < ECU_WHEEL_COUNT && state->steer_profile_retry_count[axis] < UINT8_MAX) {
        state->steer_profile_retry_count[axis]++;
    }
    state->steer_profile_setup_start_ms = now_ms;
    state->steer_profile_setup_state = MOTION_STEER_PROFILE_RETRY_BACKOFF;
}

static void steer_profile_setup_advance(motion_device_state_t *state)
{
    state->steer_profile_setup_object++;
    if (state->steer_profile_setup_object < ECU_STEER_PROFILE_OBJECT_COUNT) {
        state->steer_profile_setup_state = MOTION_STEER_PROFILE_WRITE_REQUEST;
        return;
    }

    state->steer_profile_verified_mask |=
        (uint8_t)(1U << state->steer_profile_setup_axis);
    state->steer_profile_setup_axis++;
    state->steer_profile_setup_object = 0U;
    state->steer_profile_setup_state =
        state->steer_profile_setup_axis >= ECU_WHEEL_COUNT ?
        MOTION_STEER_PROFILE_COMPLETE : MOTION_STEER_PROFILE_WRITE_REQUEST;
}

/* Configure and read back the volatile profile-position limits without ever
 * writing PDO mapping or drive NVM.  This state machine is called only by the
 * CAN2 owner and queues one asynchronous SDO at a time.  Repeated failures
 * remain fail-closed for motion but retry automatically after a bounded
 * backoff, so reconnecting one drive does not require an ECU reboot.
 */
static bool steer_profile_setup_step(motion_device_state_t *state,
                                     canopen_master_service_t *canopen,
                                     const ecu_hardware_config_t *config,
                                     uint32_t now_ms)
{
    if (state == NULL || canopen == NULL || config == NULL) {
        return false;
    }
#if ECU_CAN2_BENCH_PDO_CAPTURE_MODE
    state->steer_profile_verified_mask = ECU_STEER_REMOTE_COMMISSION_AXIS_MASK_ALL;
    state->steer_profile_setup_state = MOTION_STEER_PROFILE_COMPLETE;
    return true;
#else
    if (state->steer_profile_setup_state == MOTION_STEER_PROFILE_COMPLETE) {
        return state->steer_profile_verified_mask ==
               ECU_STEER_REMOTE_COMMISSION_AXIS_MASK_ALL;
    }
    if (state->steer_profile_setup_axis >= ECU_WHEEL_COUNT ||
        state->steer_profile_setup_object >= ECU_STEER_PROFILE_OBJECT_COUNT) {
        steer_profile_setup_reset(state);
    }

    uint8_t axis = state->steer_profile_setup_axis;
    uint8_t node_id = config->steer_nodes[axis].node_id;
    uint16_t index = steer_profile_object_index(state->steer_profile_setup_object);
    int32_t expected = steer_profile_object_value(state->steer_profile_setup_object);
    canopen_master_snapshot_t snapshot;
    canopen_master_service_get_snapshot(canopen, &snapshot);

    switch (state->steer_profile_setup_state) {
    case MOTION_STEER_PROFILE_WRITE_REQUEST:
        if (!canopen_master_service_realtime_pdo_idle(canopen) ||
            !canopen_master_service_sdo_download_idle(canopen)) {
            return false;
        }
        state->steer_profile_setup_success_before = snapshot.sdo_download_count;
        state->steer_profile_setup_abort_before = snapshot.sdo_download_abort_count;
        if (!canopen_master_service_request_sdo_write(canopen,
                                                       node_id,
                                                       index,
                                                       0U,
                                                       4U,
                                                       expected)) {
            steer_profile_setup_retry(state, now_ms);
            return false;
        }
        state->steer_profile_setup_start_ms = now_ms;
        state->steer_profile_setup_state = MOTION_STEER_PROFILE_WRITE_WAIT;
        return false;

    case MOTION_STEER_PROFILE_WRITE_WAIT:
        if (snapshot.sdo_download_abort_count !=
                state->steer_profile_setup_abort_before &&
            snapshot.last_download_index == index &&
            snapshot.last_download_subindex == 0U) {
            steer_profile_setup_retry(state, now_ms);
            return false;
        }
        if (snapshot.sdo_download_count != state->steer_profile_setup_success_before &&
            snapshot.last_download_index == index &&
            snapshot.last_download_subindex == 0U &&
            snapshot.last_download_value == expected) {
            state->steer_profile_setup_state = MOTION_STEER_PROFILE_READ_REQUEST;
            return false;
        }
        if ((uint32_t)(now_ms - state->steer_profile_setup_start_ms) >=
            ECU_STEER_PROFILE_SETUP_TIMEOUT_MS) {
            steer_profile_setup_retry(state, now_ms);
        }
        return false;

    case MOTION_STEER_PROFILE_READ_REQUEST:
        state->steer_profile_setup_success_before = snapshot.sdo_upload_count;
        state->steer_profile_setup_abort_before = snapshot.sdo_abort_count;
        if (!canopen_master_service_request_sdo_read(canopen, node_id, index, 0U)) {
            steer_profile_setup_retry(state, now_ms);
            return false;
        }
        state->steer_profile_setup_start_ms = now_ms;
        state->steer_profile_setup_state = MOTION_STEER_PROFILE_READ_WAIT;
        return false;

    case MOTION_STEER_PROFILE_READ_WAIT:
        if (snapshot.sdo_abort_count != state->steer_profile_setup_abort_before &&
            snapshot.last_sdo_node_id == node_id &&
            snapshot.last_sdo_index == index &&
            snapshot.last_sdo_subindex == 0U) {
            steer_profile_setup_retry(state, now_ms);
            return false;
        }
        if (snapshot.sdo_upload_count != state->steer_profile_setup_success_before &&
            snapshot.last_sdo_node_id == node_id &&
            snapshot.last_sdo_index == index &&
            snapshot.last_sdo_subindex == 0U &&
            snapshot.last_sdo_abort_code == 0U) {
            state->steer_profile_readback[axis][state->steer_profile_setup_object] =
                snapshot.last_sdo_value;
            if (snapshot.last_sdo_value == (uint32_t)expected) {
                steer_profile_setup_advance(state);
            } else {
                steer_profile_setup_retry(state, now_ms);
            }
            return state->steer_profile_setup_state == MOTION_STEER_PROFILE_COMPLETE;
        }
        if ((uint32_t)(now_ms - state->steer_profile_setup_start_ms) >=
            ECU_STEER_PROFILE_SETUP_TIMEOUT_MS) {
            steer_profile_setup_retry(state, now_ms);
        }
        return false;

    case MOTION_STEER_PROFILE_RETRY_BACKOFF:
        if ((uint32_t)(now_ms - state->steer_profile_setup_start_ms) >=
            ECU_STEER_PROFILE_SETUP_RETRY_BACKOFF_MS) {
            state->steer_profile_setup_state = MOTION_STEER_PROFILE_WRITE_REQUEST;
        }
        return false;

    case MOTION_STEER_PROFILE_COMPLETE:
    default:
        return state->steer_profile_verified_mask ==
               ECU_STEER_REMOTE_COMMISSION_AXIS_MASK_ALL;
    }
#endif
}

static bool prepare_steer_axis_once(canopen_master_service_t *canopen,
                                    const ecu_canopen_node_config_t *node,
                                    motion_device_state_t *state,
                                    uint32_t wheel)
{
    (void)canopen;
    (void)node;

    if (state->steer_axis_config_state[wheel] == MOTION_STEER_AXIS_READY) {
        return true;
    }

    if (state->steer_axis_config_state[wheel] == MOTION_STEER_AXIS_FAULT ||
        state->steer_axis_config_state[wheel] == MOTION_STEER_AXIS_SDO_TIMEOUT ||
        state->steer_axis_config_state[wheel] == MOTION_STEER_AXIS_SDO_ABORT) {
        return false;
    }

    if (state->steer_axis_config_state[wheel] == MOTION_STEER_AXIS_UNSEEN) {
        /* Phase-A commissioning is read-only.  The CAN analyzer / maintenance
         * tool owns PDO mapping configuration and flash save.  Normal ECU boot
         * must not rewrite 0x1400/0x1600 or send automatic mode/enable SDOs;
         * later phases should advance this state only after readback evidence.
         */
        state->steer_axis_config_state[wheel] =
            MOTION_STEER_AXIS_CONFIG_UNVERIFIED;
        state->steer_pdo_configured[wheel] = false;
        state->steer_axis_remote_verified[wheel] = false;
        state->steer_position_mode_ready[wheel] = false;
        state->steer_last_position_valid[wheel] = false;
        state->steer_last_commanded_position_valid[wheel] = false;
        state->steer_realtime_enabled[wheel] = false;
        state->steer_setup_queued_ms[wheel] = 0U;
    }

    return state->steer_axis_config_state[wheel] == MOTION_STEER_AXIS_READY;
}

static bool send_steer_command(canopen_master_service_t *canopen,
                               const ecu_canopen_node_config_t *node,
                               motion_device_state_t *state,
                               uint32_t wheel,
                               float steer_deg,
                               float steer_scale,
                               uint32_t now_ms)
{
    if (steer_limit_blocks_target(canopen, node, state, wheel, steer_deg, now_ms)) {
        state->steer_pending_target[wheel] = false;
        return servo_drive_canopen_send_control_word(canopen,
                                                     node,
                                                     SERVO_DRIVE_CONTROL_QUICK_STOP);
    }

    int32_t position_counts = scaled_float_to_i32(steer_deg, steer_scale);
    bool ok = cache_latest_steer_target(state, wheel, position_counts);
    if (ok) {
        ok = prepare_steer_axis_once(canopen, node, state, wheel);
    }
    return ok;
}

static bool command_source_allows_motion_output(ecu_command_source_t source)
{
    return source == COMMAND_SOURCE_REMOTE ||
           source == COMMAND_SOURCE_AUTO ||
           source == COMMAND_SOURCE_MAINTENANCE ||
           source == COMMAND_SOURCE_CPU1;
}

static bool can2_zero_speed_operation_enable_permitted(
    const vehicle_actuator_command_t *command)
{
    return command != NULL &&
           command->high_voltage_enable &&
           command->high_voltage_feedback_ready &&
           !command->high_voltage_disable_request &&
           command_source_allows_motion_output(command->source);
}

static bool node5_steer_contract_allows(const ecu_hardware_config_t *config,
                                        const canopen_master_pdo_request_t *request)
{
    if (config == 0 || request == 0) {
        return false;
    }

    return config->steer_nodes[ECU_WHEEL_LEG1_FRONT_RIGHT].node_id ==
               ECU_CANOPEN_STEER_FR_NODE_ID &&
           request->node_id == ECU_CANOPEN_STEER_FR_NODE_ID &&
           request->cob_id == (uint16_t)(ECU_CANOPEN_RPDO2_BASE +
                                         ECU_CANOPEN_STEER_FR_NODE_ID) &&
           request->size == 7U &&
           request->data[2] == (uint8_t)SERVO_DRIVE_MODE_PROFILE_POSITION;
}

static bool steer_all_axes_have_remote_evidence(motion_device_state_t *state,
                                                const canopen_master_service_t *canopen,
                                                const ecu_hardware_config_t *config,
                                                uint32_t now_ms)
{
    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        uint8_t node_id = config->steer_nodes[wheel].node_id;
        canopen_node_feedback_t feedback;
        bool feedback_read =
            canopen_master_service_get_node_feedback(canopen, node_id, &feedback);
        bool feedback_ready =
            feedback_read && can2_feedback_operation_enabled(node_id, &feedback);
        if (feedback_ready) {
            mark_steer_axis_ready_from_startup_evidence(
                state,
                wheel,
                true,
                feedback.actual_position_counts);
        } else if (feedback_read &&
                   can2_steer_boot_heartbeat_evidence_ready(&feedback, now_ms)) {
            /* This is a startup-only escape from the TPDO-before-RPDO
             * deadlock.  It proves the node is alive and operational on CAN2,
             * but it does not claim the next RPDO has been accepted.  Actual
             * position remains invalid until TPDO0 arrives.
             */
            mark_steer_axis_ready_from_startup_evidence(
                state,
                wheel,
                false,
                0);
        }
        bool has_evidence =
            feedback_ready ||
            (feedback_read &&
             can2_steer_boot_heartbeat_evidence_ready(&feedback, now_ms)) ||
            state->steer_axis_remote_verified[wheel] ||
            canopen_master_service_has_node_evidence(canopen, node_id);
        if (!has_evidence ||
            state->steer_axis_config_state[wheel] != MOTION_STEER_AXIS_READY) {
            return false;
        }
    }
    return true;
}

static bool steer_commissioning_authorization_valid(uint32_t now_ms,
                                                    uint8_t *axis_mask)
{
#if ECU_BUILD_PROFILE_STEER4_REMOTE_90
    /* V10 full-range steering commissioning does not use J-Link as a motion
     * enable switch.  J-Link is only allowed to load RAM calibration.  Runtime
     * motion still requires the normal remote/safety interlocks before any PDO
     * can be queued.
     */
    (void)now_ms;
    if (axis_mask != NULL) {
        *axis_mask = ECU_STEER_REMOTE_COMMISSION_AXIS_MASK_ALL;
    }
    return true;
#else
    uint8_t mask = g_ecu_steer_commissioning_control.enabled_axis_mask &
                   ECU_STEER_REMOTE_COMMISSION_AXIS_MASK_ALL;
    bool valid =
        g_ecu_steer_commissioning_control.magic == ECU_STEER_REMOTE_COMMISSION_AUTH_MAGIC &&
        g_ecu_steer_commissioning_control.steer_remote_commission_enable &&
        mask != 0U &&
        now_ms <= g_ecu_steer_commissioning_control.expiry_ms;
    if (axis_mask != NULL) {
        *axis_mask = valid ? mask : 0U;
    }
    return valid;
#endif
}

#if ECU_CANOPEN_COMMISSIONING_POLICY == ECU_CANOPEN_COMMISSIONING_POLICY_STEER4_REMOTE_COMMISSIONING
static bool steer_commissioning_axis_feedback_ready(
    const canopen_master_service_t *canopen,
    const ecu_hardware_config_t *config,
    uint8_t axis_mask)
{
    if (canopen == NULL || config == NULL) {
        return false;
    }
    if (!canopen_master_service_steer_tpdo_observers_ready(canopen)) {
        return false;
    }

    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        uint8_t axis_bit = (uint8_t)(1U << wheel);
        if ((axis_mask & axis_bit) == 0U) {
            continue;
        }

        canopen_node_feedback_t feedback;
        uint8_t node_id = config->steer_nodes[wheel].node_id;
        if (!canopen_master_service_get_node_feedback(canopen, node_id, &feedback) ||
            !feedback.feedback_fresh ||
            feedback.fault_latched != 0U) {
            return false;
        }
    }
    return true;
}

static bool steer_commissioning_axis_calibration_ready(
    const ecu_hardware_config_t *config,
    uint8_t axis_mask)
{
    if (config == NULL) {
        return false;
    }

    steer_axis_calibration_t calibration[ECU_WHEEL_COUNT];
    int32_t targets[ECU_WHEEL_COUNT] = {0};
    return motion_device_get_effective_steer_calibration(config,
                                                         axis_mask,
                                                         calibration,
                                                         NULL) &&
           steer_commissioning_build_targets(calibration,
                                              axis_mask,
                                              0.0f,
                                              targets);
}
#endif

static bool steer_commissioning_remote_conditions_ok(
    const vehicle_actuator_command_t *command)
{
    return command != NULL &&
           command->source == COMMAND_SOURCE_REMOTE &&
           command->steer_commission_interlock_ok &&
           command->target_speed_mps == 0.0f;
}

static motion_steer_inhibit_reason_t evaluate_steer_inhibit_reason(
    motion_device_state_t *state,
    const canopen_master_service_t *canopen,
    const ecu_hardware_config_t *config,
    const vehicle_actuator_command_t *command,
    uint32_t now_ms)
{
    if (command->diagnostic == DIAG_REMOTE_ESTOP_CH13 ||
        command->diagnostic == DIAG_A_CLASS_FAULT ||
        command->diagnostic == DIAG_CONTROLLED_STOP_ACTIVE) {
        return MOTION_STEER_INHIBIT_ESTOP_LATCHED;
    }
    if (command->diagnostic == DIAG_REMOTE_ESTOP_SBUS_TIMEOUT ||
        command->diagnostic == DIAG_REMOTE_ESTOP_FAILSAFE ||
        command->diagnostic == DIAG_REMOTE_ESTOP_DECODE_ERRORS ||
        command->diagnostic == DIAG_REMOTE_ESTOP_CREDIBILITY) {
        return MOTION_STEER_INHIBIT_SBUS_OFFLINE;
    }
    if (commissioning_policy_allows_can3_rpdo()) {
        return MOTION_STEER_INHIBIT_BENCH_MODE_DISABLED;
    }
    if (!commissioning_policy_allows_full_steer_pdo() &&
        !commissioning_policy_allows_steer4_remote() &&
        !commissioning_policy_allows_node5_steer_pdo()) {
        (void)state;
        (void)canopen;
        (void)config;
        return MOTION_STEER_INHIBIT_BENCH_MODE_DISABLED;
    }
    if (commissioning_policy_allows_node5_steer_pdo() &&
        !ecu_commissioning_node5_pdo_runtime_authorized()) {
        (void)state;
        (void)canopen;
        (void)config;
        return MOTION_STEER_INHIBIT_BENCH_MODE_DISABLED;
    }
    if (commissioning_policy_allows_steer4_remote()) {
        uint8_t axis_mask = 0U;
        if (!steer_commissioning_authorization_valid(now_ms, &axis_mask)) {
            return MOTION_STEER_INHIBIT_AXIS_NOT_READY;
        }
        if (command->source != COMMAND_SOURCE_REMOTE) {
            return MOTION_STEER_INHIBIT_COMMAND_SOURCE_NOT_AUTHORIZED;
        }
        if (!steer_commissioning_remote_conditions_ok(command)) {
            return MOTION_STEER_INHIBIT_REMOTE_DISARMED;
        }
        if (state->steer_group_degraded) {
            return MOTION_STEER_INHIBIT_GROUP_DEGRADED;
        }
        state->selected_axis_mask = axis_mask;
        return MOTION_STEER_INHIBIT_NONE;
    }
    if (!command_source_allows_motion_output(command->source)) {
        return MOTION_STEER_INHIBIT_COMMAND_SOURCE_NOT_AUTHORIZED;
    }
    if (!command->high_voltage_enable ||
        !command->high_voltage_feedback_ready) {
        return MOTION_STEER_INHIBIT_REMOTE_DISARMED;
    }
    /* P gear inhibits drive velocity only.  Steering must remain visible in P
     * after high voltage is confirmed, so the operator can select
     * Ackermann/spin/crab and verify the four wheel angles before allowing
     * drive motion.
     */
    if (command->active_gear != ECU_GEAR_REQUEST_P &&
        !command->brake_release) {
        return MOTION_STEER_INHIBIT_REMOTE_DISARMED;
    }
    if (state->can2_node_recovery_pending_mask != 0U) {
        return MOTION_STEER_INHIBIT_AXIS_NOT_READY;
    }
    if (state->steer_group_degraded) {
        return MOTION_STEER_INHIBIT_GROUP_DEGRADED;
    }
#if ECU_CAN2_BENCH_PDO_CAPTURE_MODE
    (void)canopen;
    (void)config;
#else
    if (!commissioning_policy_allows_node5_steer_pdo() &&
        !steer_all_axes_have_remote_evidence(state, canopen, config, now_ms)) {
        return MOTION_STEER_INHIBIT_AXIS_NOT_READY;
    }
#endif
    return MOTION_STEER_INHIBIT_NONE;
}

static bool motion_device_update_steer_safety_gate(
    motion_device_state_t *state,
    canopen_master_service_t *canopen,
    const ecu_hardware_config_t *config,
    const vehicle_actuator_command_t *command,
    uint32_t now_ms)
{
    motion_steer_inhibit_reason_t reason =
        evaluate_steer_inhibit_reason(state, canopen, config, command, now_ms);
    bool allowed = reason == MOTION_STEER_INHIBIT_NONE;
    bool was_allowed = state->steer_normal_pdo_allowed;
    motion_steer_inhibit_reason_t old_reason = state->steer_inhibit_reason;

    state->steer_normal_pdo_allowed = allowed;
    state->steer_safety_inhibited = !allowed;
    state->steer_inhibit_reason = reason;

    if (!allowed) {
        if (commissioning_policy_allows_steer4_remote() &&
            (state->steer_commission_state == STEER_REMOTE_COMMISSION_ACTIVE ||
             state->steer_commission_state == STEER_REMOTE_COMMISSION_CENTERING ||
             state->steer_commission_state == STEER_REMOTE_COMMISSION_WAIT_SYNC_TX_COMPLETE ||
             state->steer_commission_state == STEER_REMOTE_COMMISSION_WAIT_CENTER_SETTLE ||
             state->steer_commission_state == STEER_REMOTE_COMMISSION_WAIT_POST_COMMAND_TPDO)) {
            state->steer_commission_state = STEER_REMOTE_COMMISSION_FAULT;
            clear_steer_commissioning_authorization(state);
        }
        state->steer_next_group_valid = false;
        state->steer_safe_stop_pending = true;
        if (was_allowed) {
            state->steer_last_allowed_to_inhibited_ms = now_ms;
        }
        if (was_allowed || old_reason != reason) {
            state->steer_safety_inhibit_count++;
            canopen_master_service_note_pdo_safety_inhibit(canopen);
        }
        if (state->steer_active_group_sequence != 0U) {
            (void)canopen_master_service_cancel_pdo_group(
                canopen,
                state->steer_active_group_sequence);
        }
    } else {
        state->steer_safe_stop_pending = false;
    }

    return allowed;
}

static bool command_changed(const motion_device_state_t *state,
                            const vehicle_actuator_command_t *command)
{
    if (!state->last_motion_command_valid) {
        return true;
    }

    if (state->last_motion_command.source != command->source ||
        state->last_motion_command.motion_mode != command->motion_mode ||
        state->last_motion_command.active_gear != command->active_gear ||
        state->last_motion_command.target_speed_mps != command->target_speed_mps ||
        state->last_motion_command.brake_release != command->brake_release ||
        state->last_motion_command.high_voltage_enable !=
            command->high_voltage_enable ||
        state->last_motion_command.high_voltage_disable_request !=
            command->high_voltage_disable_request ||
        state->last_motion_command.high_voltage_feedback_ready !=
            command->high_voltage_feedback_ready ||
        state->last_motion_command.steer_commission_interlock_ok !=
            command->steer_commission_interlock_ok ||
        state->last_motion_command.steer_commission_steering_neutral !=
            command->steer_commission_steering_neutral ||
        state->last_motion_command.track_assist_requested !=
            command->track_assist_requested ||
        state->last_motion_command.track_assist_active !=
            command->track_assist_active) {
        return true;
    }

    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        if (state->last_motion_command.target_wheel_speed_mps[wheel] !=
            command->target_wheel_speed_mps[wheel]) {
            return true;
        }
        if (state->last_motion_command.target_steer_deg[wheel] !=
            command->target_steer_deg[wheel]) {
            return true;
        }
        if (state->last_motion_command.track_assist_current_10ma[wheel] !=
            command->track_assist_current_10ma[wheel]) {
            return true;
        }
    }

    return false;
}

/* Decide when an unchanged motion command must refresh slow setup state.
 *
 * Drive-mode, brake-output and steering setup SDOs are transactional and slow.
 * They are only refreshed periodically.  Fast steering target changes are not
 * sent here; they are cached and flushed by motion_device_flush_realtime().
 */
static bool motion_command_refresh_due(const motion_device_state_t *state,
                                       uint32_t now_ms)
{
    return !state->last_motion_command_valid ||
           (uint32_t)(now_ms - state->last_motion_command_queue_ms) >=
               ECU_CANOPEN_MOTION_COMMAND_REFRESH_MS;
}

static bool build_steer_rpdo_request(canopen_master_pdo_request_t *request,
                                     const ecu_canopen_node_config_t *node,
                                     uint16_t control_word,
                                     int32_t target_position_counts,
                                     uint32_t group_sequence,
                                     canopen_master_pdo_phase_t phase)
{
    canopen_node_pdo_profile_t profile;

    if (request == 0 || node == 0 ||
        !canopen_pdo_profile_init(node->node_id,
                                  CANOPEN_AXIS_ROLE_STEER_POSITION,
                                  &profile)) {
        return false;
    }

    return canopen_pdo_build_position_rpdo1(&profile,
                                            control_word,
                                            target_position_counts,
                                            request,
                                            group_sequence,
                                            phase);
}

static bool build_drive_velocity_rpdo_request(canopen_master_pdo_request_t *request,
                                               const ecu_canopen_node_config_t *node,
                                               uint16_t control_word,
                                               int32_t target_velocity_units,
                                               uint32_t group_sequence)
{
    canopen_node_pdo_profile_t profile;

    if (request == NULL || node == NULL ||
        !canopen_pdo_profile_init(node->node_id,
                                  CANOPEN_AXIS_ROLE_DRIVE_VELOCITY,
                                  &profile)) {
        return false;
    }

    return canopen_pdo_build_velocity_rpdo0(&profile,
                                            control_word,
                                            target_velocity_units,
                                            request,
                                            group_sequence,
                                            CANOPEN_MASTER_PDO_PHASE_DRIVE_VELOCITY);
}

static bool build_drive_current_rpdo_request(canopen_master_pdo_request_t *request,
                                             const ecu_canopen_node_config_t *node,
                                             uint16_t control_word,
                                             int16_t target_current_10ma,
                                             uint32_t group_sequence)
{
    canopen_node_pdo_profile_t profile;

    if (request == NULL || node == NULL ||
        !canopen_pdo_profile_init(node->node_id,
                                  CANOPEN_AXIS_ROLE_DRIVE_VELOCITY,
                                  &profile)) {
        return false;
    }

    return canopen_pdo_build_current_rpdo3(&profile,
                                           control_word,
                                           target_current_10ma,
                                           request,
                                           group_sequence,
                                            CANOPEN_MASTER_PDO_PHASE_DRIVE_CURRENT);
}

static bool build_steer_zero_velocity_rpdo_request(
    canopen_master_pdo_request_t *request,
    const ecu_canopen_node_config_t *node,
    uint16_t control_word,
    int32_t target_velocity_units,
    uint32_t group_sequence)
{
    if (request == NULL || node == NULL || node->node_id == 0U) {
        return false;
    }

    canopen_node_pdo_profile_t profile = {
        .node_id = node->node_id,
        .role = CANOPEN_AXIS_ROLE_STEER_POSITION,
        .rpdo0_cob_id = (uint16_t)(CANOPEN_PDO_STANDARD_RPDO0_BASE + node->node_id),
        .rpdo1_cob_id = (uint16_t)(CANOPEN_PDO_STANDARD_RPDO1_BASE + node->node_id),
        .rpdo2_cob_id = (uint16_t)(CANOPEN_PDO_STANDARD_RPDO2_BASE + node->node_id),
        .rpdo3_cob_id = (uint16_t)(CANOPEN_PDO_STANDARD_RPDO3_BASE + node->node_id),
        .tpdo0_cob_id = (uint16_t)(CANOPEN_PDO_STANDARD_TPDO0_BASE + node->node_id),
        .tpdo1_cob_id = (uint16_t)(CANOPEN_PDO_STANDARD_TPDO1_BASE + node->node_id),
        .required_mode = CANOPEN_PDO_MODE_PROFILE_VELOCITY
    };

    return canopen_pdo_build_velocity_rpdo0(
        &profile,
        control_word,
        target_velocity_units,
        request,
        group_sequence,
        CANOPEN_MASTER_PDO_PHASE_DRIVE_VELOCITY);
}

static bool build_node5_steer_rpdo_request(canopen_master_pdo_request_t *request,
                                           const ecu_hardware_config_t *config,
                                           uint16_t control_word,
                                           int32_t target_position_counts,
                                           uint32_t group_sequence,
                                           canopen_master_pdo_phase_t phase)
{
    if (config == 0) {
        return false;
    }
    if (!build_steer_rpdo_request(request,
                                  &config->steer_nodes[ECU_WHEEL_LEG1_FRONT_RIGHT],
                                  control_word,
                                  target_position_counts,
                                  group_sequence,
                                  phase)) {
        return false;
    }
    return node5_steer_contract_allows(config, request);
}

static int32_t i32_abs_saturating(int32_t value)
{
    if (value == INT32_MIN) {
        return INT32_MAX;
    }
    return value < 0 ? -value : value;
}

static bool cache_latest_drive_velocity(motion_device_state_t *state,
                                         uint32_t wheel,
                                         int32_t velocity_units,
                                         bool enable_requested)
{
    if (state == NULL || wheel >= ECU_WHEEL_COUNT) {
        return false;
    }

    if (velocity_units > -ECU_CANOPEN_DRIVE_COMMAND_ZERO_DEADBAND_UNITS &&
        velocity_units < ECU_CANOPEN_DRIVE_COMMAND_ZERO_DEADBAND_UNITS) {
        velocity_units = 0;
    }

    if (!enable_requested) {
        velocity_units = 0;
    }

    if (!state->drive_latest_velocity_valid[wheel] ||
        state->drive_latest_command_kind[wheel] != MOTION_DRIVE_COMMAND_VELOCITY ||
        i32_changed_beyond_deadband(state->drive_latest_velocity_units[wheel],
                                    velocity_units,
                                    ECU_CANOPEN_DRIVE_PDO_CHANGE_THRESHOLD_UNITS) ||
        state->drive_latest_enable_requested[wheel] != enable_requested) {
        state->drive_latest_velocity_units[wheel] = velocity_units;
        state->drive_latest_current_10ma[wheel] = 0;
        state->drive_latest_command_kind[wheel] = MOTION_DRIVE_COMMAND_VELOCITY;
        state->drive_latest_enable_requested[wheel] = enable_requested;
        state->drive_latest_velocity_valid[wheel] = true;
        state->drive_pending_velocity[wheel] = true;
    }
    return true;
}

static bool cache_latest_drive_current(motion_device_state_t *state,
                                       uint32_t wheel,
                                       int16_t current_10ma,
                                       bool enable_requested)
{
    if (state == NULL || wheel >= ECU_WHEEL_COUNT) {
        return false;
    }

    if (!enable_requested) {
        current_10ma = 0;
    }

    if (!state->drive_latest_velocity_valid[wheel] ||
        state->drive_latest_command_kind[wheel] != MOTION_DRIVE_COMMAND_CURRENT ||
        state->drive_latest_current_10ma[wheel] != current_10ma ||
        state->drive_latest_enable_requested[wheel] != enable_requested) {
        state->drive_latest_velocity_units[wheel] = 0;
        state->drive_latest_current_10ma[wheel] = current_10ma;
        state->drive_latest_command_kind[wheel] = MOTION_DRIVE_COMMAND_CURRENT;
        state->drive_latest_enable_requested[wheel] = enable_requested;
        state->drive_latest_velocity_valid[wheel] = true;
        state->drive_pending_velocity[wheel] = true;
    }
    return true;
}

static bool steer_axis_realtime_ready(motion_device_state_t *state,
                                      const canopen_master_service_t *canopen,
                                      uint8_t node_id,
                                      uint32_t wheel,
                                      uint32_t now_ms)
{
    if (state->steer_setup_queued_ms[wheel] == 0U) {
        state->steer_setup_queued_ms[wheel] = now_ms;
        return false;
    }
    (void)now_ms;
#if ECU_CAN2_BENCH_PDO_CAPTURE_MODE
    state->steer_axis_remote_verified[wheel] = true;
    state->steer_axis_config_state[wheel] = MOTION_STEER_AXIS_READY;
    state->steer_pdo_configured[wheel] = true;
    state->steer_position_mode_ready[wheel] = true;
#else
    canopen_node_feedback_t feedback;
    bool feedback_read =
        canopen_master_service_get_node_feedback(canopen, node_id, &feedback);
    bool feedback_valid = feedback_read &&
        can2_feedback_operation_enabled(node_id, &feedback);
    if (feedback_valid) {
        /* PDO mapping is configured out-of-band with the analyzer and saved to
         * the drive flash.  The ECU marks realtime-ready only after it sees
         * fresh TPDO0/TPDO1 feedback from this exact node, which proves the
         * remote device is alive on this bus and not fault-latched.  This does
         * not claim that a later RPDO was accepted; it only opens the realtime
         * PDO path.
         *
         * Volatile profile-parameter verification is deliberately not part of
         * this node-feedback predicate.  Post-fault recovery must be able to
         * transmit its coherent four-axis resynchronization group before the
         * separate setup state machine runs.  Normal motion remains blocked by
         * the explicit all-axis profile gate in motion_device_flush_realtime().
         */
        mark_steer_axis_ready_from_startup_evidence(
            state,
            wheel,
            true,
            feedback.actual_position_counts);
    } else if (feedback_read &&
               can2_steer_boot_heartbeat_evidence_ready(&feedback, now_ms)) {
        /* Permit the first realtime steering group after a fresh operational
         * heartbeat in the whole-vehicle commissioning image.  TPDO feedback
         * still becomes the normal measured source once the drive starts
         * producing it.
         */
        mark_steer_axis_ready_from_startup_evidence(state, wheel, false, 0);
    }
    if (!feedback_valid &&
        !can2_steer_boot_heartbeat_evidence_ready(
            feedback_read ? &feedback : NULL,
            now_ms)) {
        state->steer_realtime_enabled[wheel] = false;
        return false;
    }
    if (state->steer_axis_config_state[wheel] != MOTION_STEER_AXIS_READY) {
        return false;
    }
    if (!state->steer_axis_remote_verified[wheel]) {
        return false;
    }
#endif
    return state->steer_axis_config_state[wheel] == MOTION_STEER_AXIS_READY;
}

static bool all_steer_axes_realtime_ready(motion_device_state_t *state,
                                          const canopen_master_service_t *canopen,
                                          const ecu_hardware_config_t *config,
                                          uint32_t now_ms)
{
    if (commissioning_policy_allows_node5_steer_pdo()) {
        return steer_axis_realtime_ready(
            state,
            canopen,
            config->steer_nodes[ECU_WHEEL_LEG1_FRONT_RIGHT].node_id,
            ECU_WHEEL_LEG1_FRONT_RIGHT,
            now_ms);
    }

    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        if (!steer_axis_realtime_ready(state,
                                       canopen,
                                       config->steer_nodes[wheel].node_id,
                                       wheel,
                                       now_ms)) {
            return false;
        }
    }
    return true;
}

static bool build_steer_group_targets(motion_device_state_t *state,
                                      uint32_t elapsed_ms,
                                      int32_t out_targets[ECU_WHEEL_COUNT])
{
    bool group_changed = false;
    bool fixed_posture_path =
        state->last_motion_command_valid &&
        steering_transition_planner_mode_is_fixed_posture(
            state->last_motion_command.motion_mode);

    int32_t current_targets[ECU_WHEEL_COUNT];
    int32_t requested_targets[ECU_WHEEL_COUNT];

    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        if (!state->steer_latest_target_valid[wheel]) {
            return false;
        }

        requested_targets[wheel] = state->steer_latest_target_counts[wheel];
        if (!state->steer_commanded_target_valid[wheel]) {
            if (state->steer_last_position_valid[wheel]) {
                state->steer_commanded_target_counts[wheel] =
                    state->steer_last_position_counts[wheel];
            } else if (state->steer_last_commanded_position_valid[wheel]) {
                state->steer_commanded_target_counts[wheel] =
                    state->steer_last_commanded_position_counts[wheel];
            } else {
                /* No measured or transmitted origin exists yet. Starting at
                 * the requested position avoids inventing an unsafe zero-based
                 * trajectory; startup readiness still requires node evidence.
                 */
                state->steer_commanded_target_counts[wheel] =
                    requested_targets[wheel];
            }
            state->steer_commanded_target_valid[wheel] = true;
            state->steer_commanded_velocity_counts_per_sec[wheel] = 0;
        }

        current_targets[wheel] = state->steer_commanded_target_counts[wheel];
    }

    if (fixed_posture_path) {
        memcpy(out_targets, requested_targets, sizeof(requested_targets));
        state->steer_group_commanded_speed_counts_per_sec = 0;
        state->steer_follow_band = MOTION_STEER_FOLLOW_BAND_HOLD;
    } else if (!motion_setpoint_shape_steering_group(
                   current_targets,
                   requested_targets,
                   state->steer_group_commanded_speed_counts_per_sec,
                   elapsed_ms,
                   out_targets,
                   &state->steer_group_commanded_speed_counts_per_sec,
                   &state->steer_follow_band)) {
        return false;
    }

    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        int32_t previous = state->steer_commanded_target_counts[wheel];
        state->steer_commanded_target_counts[wheel] = out_targets[wheel];
        if (elapsed_ms != 0U) {
            int64_t velocity =
                ((int64_t)out_targets[wheel] - (int64_t)previous) * 1000LL /
                (int64_t)elapsed_ms;
            if (velocity > INT32_MAX) {
                velocity = INT32_MAX;
            } else if (velocity < INT32_MIN) {
                velocity = INT32_MIN;
            }
            state->steer_commanded_velocity_counts_per_sec[wheel] =
                (int32_t)velocity;
        } else {
            state->steer_commanded_velocity_counts_per_sec[wheel] = 0;
        }

        if (!state->steer_last_commanded_position_valid[wheel] ||
            state->steer_pending_target[wheel] ||
            i32_changed_beyond_deadband(state->steer_last_commanded_position_counts[wheel],
                                        out_targets[wheel],
                                        ECU_CANOPEN_STEER_POSITION_TRIGGER_THRESHOLD_COUNTS)) {
            group_changed = true;
        }
    }

    return group_changed;
}

static uint32_t next_steer_group_sequence(motion_device_state_t *state)
{
    state->steer_group_sequence_counter++;
    if (state->steer_group_sequence_counter == 0U) {
        state->steer_group_sequence_counter = 1U;
    }
    return state->steer_group_sequence_counter;
}

static bool queue_node5_steer_group(canopen_master_service_t *canopen,
                                    const ecu_hardware_config_t *config,
                                    motion_device_state_t *state,
                                    const int32_t targets[ECU_WHEEL_COUNT],
                                    uint32_t now_ms)
{
    if (!ecu_commissioning_node5_pdo_runtime_authorized() ||
        canopen_master_service_pdo_queue_available(canopen) <
            ECU_NODE5_STEER_PDO_FRAME_COUNT) {
        return false;
    }

    canopen_master_pdo_request_t requests[ECU_NODE5_STEER_PDO_FRAME_COUNT];
    uint32_t group_sequence = next_steer_group_sequence(state);
    int32_t target = targets[ECU_WHEEL_LEG1_FRONT_RIGHT];

    if (!build_node5_steer_rpdo_request(
            &requests[0],
            config,
            SERVO_DRIVE_CONTROL_ENABLE_OPERATION,
            target,
            group_sequence,
            CANOPEN_MASTER_PDO_PHASE_NODE5_POSITION_ARM) ||
        !build_node5_steer_rpdo_request(
            &requests[1],
            config,
            SERVO_DRIVE_CONTROL_TRIGGER_ABSOLUTE_POSITION,
            target,
            group_sequence,
            CANOPEN_MASTER_PDO_PHASE_NODE5_POSITION_TRIGGER)) {
        return false;
    }

    canopen_master_pdo_group_descriptor_t descriptor = {
        .expected_frames = ECU_NODE5_STEER_PDO_FRAME_COUNT,
        .arm_frame_count = 1U,
        .trigger_frame_count = 1U,
        .axis_mask = 0x01U,
        .position_group = true,
        .sync_after_arm = true,
        .sync_after_trigger = true
    };

    if (!canopen_master_service_queue_pdo_batch_with_descriptor(
            canopen,
            requests,
            ECU_NODE5_STEER_PDO_FRAME_COUNT,
            &descriptor)) {
        state->steer_pdo_tx_error_count[ECU_WHEEL_LEG1_FRONT_RIGHT]++;
        state->steer_group_failure_count++;
        return false;
    }

    state->steer_active_group_sequence = group_sequence;
    state->steer_active_group_submit_ms = now_ms;
    memcpy(state->steer_active_group_target_counts,
           targets,
           sizeof(state->steer_active_group_target_counts));
    state->steer_group_active = true;
    state->steer_active_group_node5_only = true;
    state->steer_active_group_axis_mask = 0x01U;
    state->steer_group_degraded = false;
    return true;
}

static uint8_t count_selected_axes(uint8_t axis_mask)
{
    uint8_t count = 0U;
    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        if ((axis_mask & (uint8_t)(1U << wheel)) != 0U) {
            count++;
        }
    }
    return count;
}

static bool queue_steer4_remote_group(canopen_master_service_t *canopen,
                                      const ecu_hardware_config_t *config,
                                      motion_device_state_t *state,
                                      const int32_t targets[ECU_WHEEL_COUNT],
                                      uint8_t axis_mask,
                                      uint32_t now_ms)
{
    uint8_t selected_axes = count_selected_axes(axis_mask);
    uint8_t frame_count = (uint8_t)(selected_axes * 2U);
    if (selected_axes == 0U ||
        canopen_master_service_pdo_queue_available(canopen) < frame_count) {
        return false;
    }

    canopen_master_pdo_request_t requests[ECU_STEER_GROUP_PDO_FRAME_COUNT];
    uint32_t group_sequence = next_steer_group_sequence(state);
    uint8_t out_index = 0U;

    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        if ((axis_mask & (uint8_t)(1U << wheel)) == 0U) {
            continue;
        }
        if (!build_steer_rpdo_request(&requests[out_index],
                                      &config->steer_nodes[wheel],
                                      SERVO_DRIVE_CONTROL_ENABLE_OPERATION,
                                      targets[wheel],
                                      group_sequence,
                                      CANOPEN_MASTER_PDO_PHASE_STEER_ARM)) {
            return false;
        }
        out_index++;
    }

    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        if ((axis_mask & (uint8_t)(1U << wheel)) == 0U) {
            continue;
        }
        if (!build_steer_rpdo_request(&requests[out_index],
                                      &config->steer_nodes[wheel],
                                      SERVO_DRIVE_CONTROL_TRIGGER_ABSOLUTE_POSITION,
                                      targets[wheel],
                                      group_sequence,
                                      CANOPEN_MASTER_PDO_PHASE_STEER_TRIGGER)) {
            return false;
        }
        out_index++;
    }

    canopen_master_pdo_group_descriptor_t descriptor = {
        .expected_frames = frame_count,
        .arm_frame_count = selected_axes,
        .trigger_frame_count = selected_axes,
        .axis_mask = axis_mask,
        .position_group = true,
        .sync_after_arm = true,
        .sync_after_trigger = true
    };

    if (!canopen_master_service_queue_pdo_batch_with_descriptor(canopen,
                                                                requests,
                                                                frame_count,
                                                                &descriptor)) {
        for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
            if ((axis_mask & (uint8_t)(1U << wheel)) != 0U) {
                state->steer_pdo_tx_error_count[wheel]++;
            }
        }
        state->steer_group_failure_count++;
        return false;
    }

    state->steer_active_group_sequence = group_sequence;
    state->steer_active_group_submit_ms = now_ms;
    memcpy(state->steer_active_group_target_counts,
           targets,
           sizeof(state->steer_active_group_target_counts));
    state->steer_group_active = true;
    state->steer_active_group_node5_only = false;
    state->steer_active_group_axis_mask = axis_mask;
    state->steer_group_degraded = false;
    return true;
}

static bool queue_steer_group(canopen_master_service_t *canopen,
                              const ecu_hardware_config_t *config,
                              motion_device_state_t *state,
                              const int32_t targets[ECU_WHEEL_COUNT],
                              uint32_t now_ms)
{
    bool queued = false;

    if (commissioning_policy_allows_node5_steer_pdo()) {
        queued = queue_node5_steer_group(canopen, config, state, targets, now_ms);
        goto done;
    } else if (commissioning_policy_allows_steer4_remote()) {
        queued = queue_steer4_remote_group(canopen,
                                           config,
                                           state,
                                           targets,
                                           state->selected_axis_mask,
                                           now_ms);
        goto done;
    }

    if (!commissioning_policy_allows_full_steer_pdo()) {
        goto done;
    }

    if (canopen_master_service_pdo_queue_available(canopen) < ECU_STEER_GROUP_PDO_FRAME_COUNT) {
        goto done;
    }

    canopen_master_pdo_request_t requests[ECU_STEER_GROUP_PDO_FRAME_COUNT];
    uint32_t group_sequence = next_steer_group_sequence(state);
    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        const ecu_canopen_node_config_t *node = &config->steer_nodes[wheel];
        if (!build_steer_rpdo_request(&requests[wheel],
                                      node,
                                      SERVO_DRIVE_CONTROL_ABSOLUTE_UPDATE_ARM,
                                      targets[wheel],
                                      group_sequence,
                                      CANOPEN_MASTER_PDO_PHASE_STEER_ARM)) {
            goto done;
        }
    }

    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        const ecu_canopen_node_config_t *node = &config->steer_nodes[wheel];
        if (!build_steer_rpdo_request(&requests[ECU_WHEEL_COUNT + wheel],
                                      node,
                                      SERVO_DRIVE_CONTROL_ABSOLUTE_UPDATE_TRIGGER,
                                      targets[wheel],
                                      group_sequence,
                                      CANOPEN_MASTER_PDO_PHASE_STEER_TRIGGER)) {
            goto done;
        }
    }

    canopen_master_pdo_group_descriptor_t descriptor = {
        .expected_frames = ECU_STEER_GROUP_PDO_FRAME_COUNT,
        .arm_frame_count = ECU_WHEEL_COUNT,
        .trigger_frame_count = ECU_WHEEL_COUNT,
        .axis_mask = ECU_STEER_REMOTE_COMMISSION_AXIS_MASK_ALL,
        .position_group = true,
        .sync_after_arm = true,
        .sync_after_trigger = true
    };

    if (!canopen_master_service_queue_pdo_batch_with_descriptor(
            canopen,
            requests,
            ECU_STEER_GROUP_PDO_FRAME_COUNT,
            &descriptor)) {
        for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
            state->steer_pdo_tx_error_count[wheel]++;
        }
        state->steer_group_failure_count++;
        goto done;
    }

    state->steer_active_group_sequence = group_sequence;
    state->steer_active_group_submit_ms = now_ms;
    memcpy(state->steer_active_group_target_counts,
           targets,
           sizeof(state->steer_active_group_target_counts));
    state->steer_group_active = true;
    state->steer_active_group_node5_only = false;
    state->steer_active_group_axis_mask = ECU_STEER_REMOTE_COMMISSION_AXIS_MASK_ALL;
    state->steer_group_degraded = false;
    queued = true;

done:
    if (queued && state->can2_recovery_steer_sync_pending) {
        /* Remember the exact coherent four-axis group that is allowed to
         * release the post-recovery drive interlock.  An older group finishing
         * after recovery started must never reopen traction output. */
        state->can2_recovery_steer_group_sequence =
            state->steer_active_group_sequence;
    }
    return queued;
}

static void steer_zero_calibration_enter_state(
    motion_device_state_t *state,
    motion_steer_zero_calibration_state_t next_state,
    uint32_t now_ms)
{
    if (state == NULL) {
        return;
    }
    state->steer_zero_calibration_state = next_state;
    state->steer_zero_calibration_state_enter_ms = now_ms;
    state->steer_zero_calibration_done_mask = 0U;
    if (next_state == MOTION_STEER_ZERO_CAL_SEARCH_LEFT ||
        next_state == MOTION_STEER_ZERO_CAL_SEARCH_RIGHT) {
        memset(state->steer_zero_calibration_peak_current_10ma,
               0,
               sizeof(state->steer_zero_calibration_peak_current_10ma));
    }
    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        state->steer_zero_calibration_zero_speed_since_ms[wheel] = 0U;
    }
}

static void steer_zero_calibration_fault(motion_device_state_t *state,
                                         uint8_t fault_mask,
                                         uint32_t now_ms)
{
    if (state == NULL) {
        return;
    }
    state->steer_zero_calibration_fault_mask |= fault_mask != 0U ?
        fault_mask : ECU_STEER_REMOTE_COMMISSION_AXIS_MASK_ALL;
    steer_zero_calibration_enter_state(state,
                                       MOTION_STEER_ZERO_CAL_FAULT,
                                       now_ms);
    state->steer_zero_calibration_requested = false;
    state->steer_zero_calibration_sdo_active = false;
    state->steer_zero_calibration_group_active = false;
}

static bool steer_zero_feedback(canopen_master_service_t *canopen,
                                const ecu_hardware_config_t *config,
                                uint32_t wheel,
                                canopen_node_feedback_t *feedback)
{
    if (canopen == NULL || config == NULL || feedback == NULL ||
        wheel >= ECU_WHEEL_COUNT) {
        return false;
    }
    uint8_t node_id = config->steer_nodes[wheel].node_id;
    return canopen_master_service_get_node_feedback(canopen,
                                                   node_id,
                                                   feedback) &&
           feedback->feedback_fresh;
}

static bool steer_zero_queue_sdo(motion_device_state_t *state,
                                 canopen_master_service_t *canopen,
                                 uint8_t node_id,
                                 uint16_t index,
                                 uint8_t subindex,
                                 uint8_t size,
                                 int32_t value,
                                 uint32_t now_ms)
{
    if (state == NULL || canopen == NULL || state->steer_zero_calibration_sdo_active) {
        return false;
    }
    if (!canopen_master_service_sdo_download_idle(canopen)) {
        return false;
    }

    canopen_master_snapshot_t snapshot;
    memset(&snapshot, 0, sizeof(snapshot));
    canopen_master_service_get_snapshot(canopen, &snapshot);

    bool calibration_position_write = index == ECU_CANOPEN_OBJ_ACTUAL_POSITION &&
                                      subindex == 0U && size == 4U;
    bool queued = calibration_position_write ?
        (value == 0 ?
            canopen_master_service_request_calibration_position_zero(canopen,
                                                                     node_id) :
            canopen_master_service_request_calibration_position_restore(canopen,
                                                                        node_id,
                                                                        value)) :
        canopen_master_service_request_sdo_write(canopen,
                                                 node_id,
                                                 index,
                                                 subindex,
                                                 size,
                                                 value);
    if (!queued) {
        /* sdo_download_idle() proved that neither a queued nor active runtime
         * download occupies the service.  A failure here is therefore a local
         * policy/argument rejection, not ordinary back-pressure.  Fail the
         * calibration immediately instead of retrying silently until the
         * outer 30 s setup timeout expires. */
        steer_zero_calibration_fault(state, 0U, now_ms);
        return false;
    }

    state->steer_zero_calibration_sdo_active = true;
    state->steer_zero_calibration_sdo_node_id = node_id;
    state->steer_zero_calibration_sdo_index = index;
    state->steer_zero_calibration_sdo_subindex = subindex;
    state->steer_zero_calibration_sdo_size = size;
    state->steer_zero_calibration_sdo_value = value;
    state->steer_zero_calibration_sdo_download_count_before =
        snapshot.sdo_download_count;
    state->steer_zero_calibration_sdo_abort_count_before =
        snapshot.sdo_download_abort_count;
    state->steer_zero_calibration_sdo_start_ms = now_ms;
    return true;
}

static bool steer_zero_sdo_complete(motion_device_state_t *state,
                                    canopen_master_service_t *canopen,
                                    uint32_t now_ms,
                                    bool *faulted)
{
    if (faulted != NULL) {
        *faulted = false;
    }
    if (state == NULL || canopen == NULL) {
        if (faulted != NULL) {
            *faulted = true;
        }
        return false;
    }
    if (!state->steer_zero_calibration_sdo_active) {
        return true;
    }

    canopen_master_snapshot_t snapshot;
    memset(&snapshot, 0, sizeof(snapshot));
    canopen_master_service_get_snapshot(canopen, &snapshot);

    if (snapshot.sdo_download_abort_count >
        state->steer_zero_calibration_sdo_abort_count_before) {
        if (faulted != NULL) {
            *faulted = true;
        }
        return false;
    }

    if (snapshot.sdo_download_count >
            state->steer_zero_calibration_sdo_download_count_before &&
        snapshot.last_download_index == state->steer_zero_calibration_sdo_index &&
        snapshot.last_download_subindex ==
            state->steer_zero_calibration_sdo_subindex &&
        snapshot.last_download_size == state->steer_zero_calibration_sdo_size &&
        snapshot.last_download_value == state->steer_zero_calibration_sdo_value &&
        snapshot.last_download_abort_code == 0U) {
        state->steer_zero_calibration_sdo_active = false;
        return true;
    }

    if ((uint32_t)(now_ms - state->steer_zero_calibration_sdo_start_ms) >
        ECU_STEER_ZERO_SDO_TIMEOUT_MS) {
        if (faulted != NULL) {
            *faulted = true;
        }
    }
    return false;
}

static bool steer_zero_setup_step(motion_device_state_t *state,
                                  canopen_master_service_t *canopen,
                                  const ecu_hardware_config_t *config,
                                  uint32_t now_ms)
{
    if (state->steer_zero_calibration_sdo_active) {
        canopen_master_snapshot_t snapshot;
        memset(&snapshot, 0, sizeof(snapshot));
        canopen_master_service_get_snapshot(canopen, &snapshot);

        bool download_aborted =
            snapshot.sdo_download_abort_count >
            state->steer_zero_calibration_sdo_abort_count_before;
        bool download_timed_out =
            (uint32_t)(now_ms - state->steer_zero_calibration_sdo_start_ms) >
            ECU_STEER_ZERO_SDO_TIMEOUT_MS;
        if (download_aborted || download_timed_out) {
            state->steer_zero_calibration_sdo_active = false;
            if (state->steer_zero_calibration_setup_sdo_retry_count <
                ECU_STEER_ZERO_SETUP_SDO_MAX_RETRIES) {
                /* Retry this exact setup object.  setup_step advances only
                 * after a matching successful completion, so a transient SDO
                 * timeout/abort can never skip a CiA-402 state transition. */
                state->steer_zero_calibration_setup_sdo_retry_count++;
                return false;
            }
            steer_zero_calibration_fault(state, 0U, now_ms);
            return false;
        }

        bool download_completed =
            snapshot.sdo_download_count >
                state->steer_zero_calibration_sdo_download_count_before &&
            snapshot.last_download_index ==
                state->steer_zero_calibration_sdo_index &&
            snapshot.last_download_subindex ==
                state->steer_zero_calibration_sdo_subindex &&
            snapshot.last_download_size ==
                state->steer_zero_calibration_sdo_size &&
            snapshot.last_download_value ==
                state->steer_zero_calibration_sdo_value &&
            snapshot.last_download_abort_code == 0U;
        if (!download_completed) {
            return false;
        }

        state->steer_zero_calibration_sdo_active = false;
        state->steer_zero_calibration_setup_sdo_retry_count = 0U;
        state->steer_zero_calibration_setup_step++;
        if (state->steer_zero_calibration_setup_step > 10U) {
            state->steer_zero_calibration_setup_step = 0U;
            state->steer_zero_calibration_setup_node_index++;
            return false;
        }
    }

    if (state->steer_zero_calibration_setup_node_index >= ECU_WHEEL_COUNT) {
        return true;
    }

    uint32_t wheel = state->steer_zero_calibration_setup_node_index;
    uint8_t node_id = config->steer_nodes[wheel].node_id;
    uint16_t index = 0U;
    uint8_t subindex = 0U;
    uint8_t size = 0U;
    int32_t value = 0;

    switch (state->steer_zero_calibration_setup_step) {
    case 0U:
        {
            canopen_node_feedback_t feedback;
            if (!steer_zero_feedback(canopen, config, wheel, &feedback)) {
                return false;
            }
            if (feedback.fault_latched != 0U) {
                index = ECU_CANOPEN_OBJ_FAULT_LATCHED;
                size = 4U;
                value = (int32_t)feedback.fault_latched;
                break;
            }
            state->steer_zero_calibration_setup_step++;
            state->steer_zero_calibration_setup_sdo_retry_count = 0U;
            return false;
        }
    case 1U:
        index = 0x6040U;
        size = 2U;
        value = SERVO_DRIVE_CONTROL_FAULT_RESET;
        break;
    case 2U:
        index = ECU_CANOPEN_OBJ_BC_INTERPOLATION_OPTION;
        size = 2U;
        value = 0x001E;
        break;
    case 3U:
        index = 0x6060U;
        size = 1U;
        value = SERVO_DRIVE_MODE_PROFILE_VELOCITY;
        break;
    case 4U:
        index = 0x6081U;
        size = 4U;
        value = ECU_STEER_ZERO_SEARCH_PROFILE_VELOCITY_COUNTS_PER_SEC;
        break;
    case 5U:
        index = 0x6083U;
        size = 4U;
        value = ECU_STEER_ZERO_SEARCH_PROFILE_ACCEL_COUNTS_PER_SEC2;
        break;
    case 6U:
        index = 0x6084U;
        size = 4U;
        value = ECU_STEER_ZERO_SEARCH_PROFILE_ACCEL_COUNTS_PER_SEC2;
        break;
    case 7U:
        index = ECU_CANOPEN_OBJ_COMMAND_CURRENT_RAMP;
        size = 4U;
        value = 1000;
        break;
    case 8U:
        index = 0x6040U;
        size = 2U;
        value = SERVO_DRIVE_CONTROL_SHUTDOWN;
        break;
    case 9U:
        index = 0x6040U;
        size = 2U;
        value = SERVO_DRIVE_CONTROL_SWITCH_ON;
        break;
    case 10U:
        index = 0x6040U;
        size = 2U;
        value = SERVO_DRIVE_CONTROL_ENABLE_OPERATION;
        break;
    default:
        state->steer_zero_calibration_setup_step = 0U;
        state->steer_zero_calibration_setup_sdo_retry_count = 0U;
        state->steer_zero_calibration_setup_node_index++;
        return false;
    }

    (void)steer_zero_queue_sdo(state,
                               canopen,
                               node_id,
                               index,
                               subindex,
                               size,
                               value,
                               now_ms);
    return false;
}

static bool steer_zero_queue_velocity_group(
    motion_device_state_t *state,
    canopen_master_service_t *canopen,
    const ecu_hardware_config_t *config,
    const int32_t velocity_units[ECU_WHEEL_COUNT],
    uint32_t now_ms)
{
    if (state == NULL || canopen == NULL || config == NULL ||
        canopen_master_service_pdo_queue_available(canopen) < ECU_WHEEL_COUNT) {
        return false;
    }

    canopen_master_pdo_request_t requests[ECU_WHEEL_COUNT];
    uint32_t group_sequence = next_steer_group_sequence(state);
    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        if (!build_steer_zero_velocity_rpdo_request(
                &requests[wheel],
                &config->steer_nodes[wheel],
                SERVO_DRIVE_CONTROL_ENABLE_OPERATION,
                velocity_units[wheel],
                group_sequence)) {
            return false;
        }
    }

    canopen_master_pdo_group_descriptor_t descriptor = {
        .expected_frames = ECU_WHEEL_COUNT,
        .arm_frame_count = ECU_WHEEL_COUNT,
        .trigger_frame_count = 0U,
        .axis_mask = ECU_STEER_REMOTE_COMMISSION_AXIS_MASK_ALL,
        .position_group = false,
        .sync_after_arm = true,
        .sync_after_trigger = false
    };

    if (!canopen_master_service_queue_pdo_batch_with_descriptor(canopen,
                                                                requests,
                                                                ECU_WHEEL_COUNT,
                                                                &descriptor)) {
        return false;
    }

    state->steer_zero_calibration_active_group_sequence = group_sequence;
    state->steer_zero_calibration_last_pdo_ms = now_ms;
    state->steer_zero_calibration_group_active = true;
    return true;
}

static bool steer_zero_velocity_group_ready(motion_device_state_t *state,
                                            canopen_master_service_t *canopen,
                                            uint32_t now_ms)
{
    if (state == NULL || canopen == NULL) {
        return false;
    }
    if (!state->steer_zero_calibration_group_active) {
        return true;
    }
    uint32_t group = state->steer_zero_calibration_active_group_sequence;
    if (canopen_master_service_pdo_group_failed(canopen, group) ||
        canopen_master_service_pdo_group_cancelled(canopen, group)) {
        steer_zero_calibration_fault(state, 0U, now_ms);
        return false;
    }
    if (canopen_master_service_pdo_group_pending(canopen, group)) {
        return false;
    }
    state->steer_zero_calibration_group_active = false;
    return true;
}

static int32_t steer_zero_left_direction_sign(uint32_t wheel)
{
    switch (wheel) {
    case ECU_WHEEL_LEG1_FRONT_RIGHT:
        return ECU_STEER_ZERO_LEG1_LEFT_SIGN;
    case ECU_WHEEL_LEG2_FRONT_LEFT:
        return ECU_STEER_ZERO_LEG2_LEFT_SIGN;
    case ECU_WHEEL_LEG3_REAR_LEFT:
        return ECU_STEER_ZERO_LEG3_LEFT_SIGN;
    case ECU_WHEEL_LEG4_REAR_RIGHT:
        return ECU_STEER_ZERO_LEG4_LEFT_SIGN;
    default:
        return 1;
    }
}

static int32_t steer_zero_direction_sign_for_axis(uint32_t wheel,
                                                  bool left_search)
{
    int32_t left_sign = steer_zero_left_direction_sign(wheel);
    return left_search ? left_sign : -left_sign;
}

static int32_t steer_zero_search_velocity_for_axis(
    motion_device_state_t *state,
    uint32_t wheel,
    int32_t position_counts,
    int32_t direction_sign)
{
    int32_t travel =
        abs_i32_delta(position_counts,
                      state->steer_zero_calibration_direction_start_counts[wheel]);
    int32_t speed = travel >= ECU_STEER_ZERO_SEARCH_SLOWDOWN_ABS_COUNTS ?
        ECU_STEER_ZERO_SEARCH_SLOW_VELOCITY_UNITS :
        ECU_STEER_ZERO_SEARCH_FAST_VELOCITY_UNITS;
    return direction_sign >= 0 ? speed : -speed;
}

static bool steer_zero_axis_at_end_stop(motion_device_state_t *state,
                                        uint32_t wheel,
                                        const canopen_node_feedback_t *feedback,
                                        uint32_t now_ms)
{
    int32_t current = feedback->actual_current_raw;
    if (current < 0) {
        current = -current;
    }
    if (current > state->steer_zero_calibration_peak_current_10ma[wheel]) {
        state->steer_zero_calibration_peak_current_10ma[wheel] = (int16_t)current;
    }

    /* Ignore startup/brake-release current while the commanded velocity is
     * first taking effect.  Without this arming delay, a heavily loaded wheel
     * can be mistaken for an end stop before it has moved away from its start
     * position, producing a wrong midpoint and therefore a dangerous zero. */
    if ((uint32_t)(now_ms - state->steer_zero_calibration_state_enter_ms) <
        ECU_STEER_ZERO_STALL_ARM_DELAY_MS) {
        state->steer_zero_calibration_zero_speed_since_ms[wheel] = 0U;
        return false;
    }
    if (current >= ECU_STEER_ZERO_PROTECTION_CURRENT_10MA) {
        return true;
    }

    if (current >= ECU_STEER_ZERO_STALL_CURRENT_10MA) {
        if (state->steer_zero_calibration_zero_speed_since_ms[wheel] == 0U) {
            state->steer_zero_calibration_zero_speed_since_ms[wheel] = now_ms;
        }
        return (uint32_t)(now_ms -
               state->steer_zero_calibration_zero_speed_since_ms[wheel]) >=
               ECU_STEER_ZERO_STALL_DWELL_MS;
    }

    state->steer_zero_calibration_zero_speed_since_ms[wheel] = 0U;
    return false;
}

static bool steer_zero_run_search(motion_device_state_t *state,
                                  canopen_master_service_t *canopen,
                                  const ecu_hardware_config_t *config,
                                  uint32_t now_ms,
                                  bool left_search)
{
    if ((uint32_t)(now_ms - state->steer_zero_calibration_state_enter_ms) >
        ECU_STEER_ZERO_SEARCH_TIMEOUT_MS) {
        steer_zero_calibration_fault(state, 0U, now_ms);
        return false;
    }
    if (!steer_zero_velocity_group_ready(state, canopen, now_ms)) {
        return false;
    }

    int32_t velocities[ECU_WHEEL_COUNT] = {0};
    bool axis_stopped_now = false;
    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        uint8_t axis_bit = (uint8_t)(1U << wheel);
        canopen_node_feedback_t feedback;
        if ((state->steer_zero_calibration_done_mask & axis_bit) != 0U) {
            velocities[wheel] = 0;
            continue;
        }
        if (!steer_zero_feedback(canopen, config, wheel, &feedback)) {
            steer_zero_calibration_fault(state, axis_bit, now_ms);
            return false;
        }
        int32_t travel =
            abs_i32_delta(feedback.actual_position_counts,
                          state->steer_zero_calibration_direction_start_counts[wheel]);
        if (travel > ECU_STEER_ZERO_MAX_TRAVEL_COUNTS) {
            steer_zero_calibration_fault(state, axis_bit, now_ms);
            return false;
        }
        if (steer_zero_axis_at_end_stop(state, wheel, &feedback, now_ms)) {
            if (left_search) {
                state->steer_zero_calibration_left_hit_counts[wheel] =
                    feedback.actual_position_counts;
            } else {
                state->steer_zero_calibration_right_hit_counts[wheel] =
                    feedback.actual_position_counts;
            }
            state->steer_zero_calibration_done_mask |= axis_bit;
            velocities[wheel] = 0;
            axis_stopped_now = true;
            continue;
        }
        if (feedback.fault_latched != 0U) {
            if (state->steer_zero_calibration_peak_current_10ma[wheel] >=
                ECU_STEER_ZERO_STALL_CURRENT_10MA) {
                if (left_search) {
                    state->steer_zero_calibration_left_hit_counts[wheel] =
                        feedback.actual_position_counts;
                } else {
                    state->steer_zero_calibration_right_hit_counts[wheel] =
                        feedback.actual_position_counts;
                }
                state->steer_zero_calibration_done_mask |= axis_bit;
                velocities[wheel] = 0;
                axis_stopped_now = true;
                continue;
            } else {
                steer_zero_calibration_fault(state, axis_bit, now_ms);
                return false;
            }
        }
        int32_t direction_sign =
            steer_zero_direction_sign_for_axis(wheel, left_search);
        velocities[wheel] = steer_zero_search_velocity_for_axis(
            state,
            wheel,
            feedback.actual_position_counts,
            direction_sign);
    }

    if (state->steer_zero_calibration_done_mask ==
        ECU_STEER_REMOTE_COMMISSION_AXIS_MASK_ALL) {
        int32_t zeros[ECU_WHEEL_COUNT] = {0};
        if (!steer_zero_queue_velocity_group(
                state, canopen, config, zeros, now_ms)) {
            steer_zero_calibration_fault(state, 0U, now_ms);
            return false;
        }
        return true;
    }

    if (axis_stopped_now ||
        (uint32_t)(now_ms - state->steer_zero_calibration_last_pdo_ms) >=
        ECU_STEER_ZERO_VELOCITY_PDO_PERIOD_MS) {
        if (!steer_zero_queue_velocity_group(state, canopen, config, velocities, now_ms)) {
            steer_zero_calibration_fault(state, 0U, now_ms);
        }
    }
    return false;
}

static void steer_zero_begin_velocity_prepare(motion_device_state_t *state)
{
    if (state == NULL) {
        return;
    }
    state->steer_zero_calibration_setup_node_index = 0U;
    state->steer_zero_calibration_setup_step = 0U;
    state->steer_zero_calibration_setup_sdo_retry_count = 0U;
    state->steer_zero_calibration_sdo_active = false;
    state->steer_zero_calibration_prepare_zero_queued = false;
    state->steer_zero_calibration_prepare_settle_start_ms = 0U;
}

static bool steer_zero_prepare_velocity_phase(motion_device_state_t *state,
                                              canopen_master_service_t *canopen,
                                              const ecu_hardware_config_t *config,
                                              uint32_t now_ms)
{
    if ((uint32_t)(now_ms - state->steer_zero_calibration_state_enter_ms) >
        ECU_STEER_ZERO_SETUP_TIMEOUT_MS) {
        steer_zero_calibration_fault(state, 0U, now_ms);
        return false;
    }
    if (!steer_zero_velocity_group_ready(state, canopen, now_ms)) {
        return false;
    }
    if (!steer_zero_setup_step(state, canopen, config, now_ms)) {
        return false;
    }

    /* Match the analyzer-proven workflow: after every SDO mode/fault/enable
     * preparation, commit one coherent four-axis zero-velocity PDO group and
     * allow it to settle before issuing a limit-search or return command.  This
     * prevents a retained target-velocity value from moving an axis during
     * mode changes. */
    if (!state->steer_zero_calibration_prepare_zero_queued) {
        int32_t zeros[ECU_WHEEL_COUNT] = {0};
        if (!steer_zero_queue_velocity_group(state, canopen, config, zeros, now_ms)) {
            return false;
        }
        state->steer_zero_calibration_prepare_zero_queued = true;
        state->steer_zero_calibration_prepare_settle_start_ms = 0U;
        return false;
    }

    if (state->steer_zero_calibration_prepare_settle_start_ms == 0U) {
        /* Start the settling interval only after the CAN2 group completion was
         * observed, not merely after local queue submission. */
        state->steer_zero_calibration_prepare_settle_start_ms = now_ms;
        return false;
    }

    return (uint32_t)(now_ms -
                      state->steer_zero_calibration_prepare_settle_start_ms) >=
           ECU_STEER_ZERO_PHASE_SETTLE_MS;
}

static int32_t steer_zero_return_velocity(int32_t error_counts)
{
    int32_t abs_error = i32_abs_saturating(error_counts);
    int32_t speed = ECU_STEER_ZERO_MID_RETURN_FAST_VELOCITY_UNITS;
    if (abs_error <= ECU_STEER_ZERO_MID_RETURN_SLOW_ERROR_COUNTS) {
        speed = ECU_STEER_ZERO_MID_RETURN_SLOW_VELOCITY_UNITS;
    } else if (abs_error <= ECU_STEER_ZERO_MID_RETURN_MEDIUM_ERROR_COUNTS) {
        speed = ECU_STEER_ZERO_MID_RETURN_MEDIUM_VELOCITY_UNITS;
    }
    return error_counts >= 0 ? speed : -speed;
}

static bool steer_zero_run_return_mid(motion_device_state_t *state,
                                      canopen_master_service_t *canopen,
                                      const ecu_hardware_config_t *config,
                                      uint32_t now_ms)
{
    if ((uint32_t)(now_ms - state->steer_zero_calibration_state_enter_ms) >
        ECU_STEER_ZERO_RETURN_TIMEOUT_MS) {
        /* Never redefine an arbitrary timeout position as mechanical center.
         * A wrong zero survives into every later steering command and is more
         * dangerous than an explicit calibration fault. */
        steer_zero_calibration_fault(state, 0U, now_ms);
        return false;
    }
    if (!steer_zero_velocity_group_ready(state, canopen, now_ms)) {
        return false;
    }

    int32_t velocities[ECU_WHEEL_COUNT] = {0};
    uint8_t ready_mask = 0U;
    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        uint8_t axis_bit = (uint8_t)(1U << wheel);
        canopen_node_feedback_t feedback;
        if (!steer_zero_feedback(canopen, config, wheel, &feedback)) {
            steer_zero_calibration_fault(state, axis_bit, now_ms);
            return false;
        }
        int32_t error =
            state->steer_zero_calibration_midpoint_counts[wheel] -
            feedback.actual_position_counts;
        bool crossed_target =
            (state->steer_zero_calibration_return_last_error_counts[wheel] > 0 &&
             error < 0) ||
            (state->steer_zero_calibration_return_last_error_counts[wheel] < 0 &&
             error > 0);
        bool close_enough =
            i32_abs_saturating(error) <= ECU_STEER_ZERO_MIDPOINT_TOLERANCE_COUNTS;
        if (feedback.fault_latched != 0U && !close_enough && !crossed_target) {
            steer_zero_calibration_fault(state, axis_bit, now_ms);
            return false;
        }
        if (close_enough || crossed_target) {
            if (state->steer_zero_calibration_midpoint_stable_samples[wheel] <
                UINT8_MAX) {
                state->steer_zero_calibration_midpoint_stable_samples[wheel]++;
            }
        } else {
            state->steer_zero_calibration_midpoint_stable_samples[wheel] = 0U;
        }
        state->steer_zero_calibration_return_last_error_counts[wheel] = error;
        if (state->steer_zero_calibration_midpoint_stable_samples[wheel] >=
            ECU_STEER_ZERO_MIDPOINT_STABLE_SAMPLES) {
            ready_mask |= axis_bit;
            velocities[wheel] = 0;
        } else {
            velocities[wheel] = steer_zero_return_velocity(error);
        }
    }

    if (ready_mask == ECU_STEER_REMOTE_COMMISSION_AXIS_MASK_ALL) {
        int32_t zeros[ECU_WHEEL_COUNT] = {0};
        if (!steer_zero_queue_velocity_group(
                state, canopen, config, zeros, now_ms)) {
            steer_zero_calibration_fault(state, 0U, now_ms);
            return false;
        }
        return true;
    }

    if ((uint32_t)(now_ms - state->steer_zero_calibration_last_pdo_ms) >=
        ECU_STEER_ZERO_VELOCITY_PDO_PERIOD_MS) {
        if (!steer_zero_queue_velocity_group(state, canopen, config, velocities, now_ms)) {
            steer_zero_calibration_fault(state, 0U, now_ms);
        }
    }
    return false;
}

static bool steer_zero_write_zero_step(motion_device_state_t *state,
                                       canopen_master_service_t *canopen,
                                       const ecu_hardware_config_t *config,
                                       uint32_t now_ms)
{
    bool faulted = false;
    bool write_was_active = state->steer_zero_calibration_sdo_active;
    uint8_t completed_node_id = state->steer_zero_calibration_sdo_node_id;
    if (!steer_zero_sdo_complete(state, canopen, now_ms, &faulted)) {
        if (faulted) {
            steer_zero_calibration_fault(state, 0U, now_ms);
        }
        return false;
    }
    if (faulted) {
        steer_zero_calibration_fault(state, 0U, now_ms);
        return false;
    }
    if (write_was_active) {
        for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
            if (config->steer_nodes[wheel].node_id == completed_node_id) {
                state->steer_zero_calibration_zero_written_mask |=
                    (uint8_t)(1U << wheel);
                break;
            }
        }
    }
    if (state->steer_zero_calibration_zero_write_index >= ECU_WHEEL_COUNT) {
        return true;
    }
    uint32_t wheel = state->steer_zero_calibration_zero_write_index;
    if (steer_zero_queue_sdo(state,
                             canopen,
                             config->steer_nodes[wheel].node_id,
                             ECU_CANOPEN_OBJ_ACTUAL_POSITION,
                             0x00U,
                             4U,
                             0,
                             now_ms)) {
        state->steer_zero_calibration_zero_write_index++;
    }
    return false;
}

/* Accept the calibration only after every drive reports a new TPDO0 sample
 * showing that the actual-position zero write took effect.  SDO download
 * completion proves only that the object write was acknowledged, not that the
 * complete four-axis zero is coherent and observable by the control loop. */
static bool steer_zero_verify_zero(motion_device_state_t *state,
                                   canopen_master_service_t *canopen,
                                   const ecu_hardware_config_t *config,
                                   uint32_t now_ms)
{
    uint8_t verified_mask = 0U;

    if ((uint32_t)(now_ms - state->steer_zero_calibration_state_enter_ms) >
        ECU_STEER_ZERO_VERIFY_TIMEOUT_MS) {
        steer_zero_calibration_fault(state, 0U, now_ms);
        return false;
    }

    /* TPDO0 is synchronous type 1.  A successful SDO write alone cannot make
     * a new position sample appear, so explicitly send one zero-velocity group
     * and its SYNC before evaluating the post-write feedback timestamps. */
    if (!steer_zero_velocity_group_ready(state, canopen, now_ms)) {
        return false;
    }
    if (!state->steer_zero_calibration_verify_sync_sent) {
        int32_t zeros[ECU_WHEEL_COUNT] = {0};
        if (!steer_zero_queue_velocity_group(state, canopen, config, zeros, now_ms)) {
            return false;
        }
        state->steer_zero_calibration_verify_sync_sent = true;
        return false;
    }

    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        canopen_node_feedback_t feedback;
        if (!steer_zero_feedback(canopen, config, wheel, &feedback) ||
            feedback.last_tpdo0_ms <
                state->steer_zero_calibration_state_enter_ms ||
            feedback.fault_latched != 0U ||
            i32_abs_saturating(feedback.actual_position_counts) >
                ECU_STEER_ZERO_MIDPOINT_TOLERANCE_COUNTS) {
            continue;
        }
        verified_mask |= (uint8_t)(1U << wheel);
    }

    return verified_mask == ECU_STEER_REMOTE_COMMISSION_AXIS_MASK_ALL;
}

static void steer_zero_begin_abort(motion_device_state_t *state,
                                   uint32_t now_ms)
{
    if (state == NULL) {
        return;
    }
    state->steer_zero_calibration_requested = false;
    state->steer_zero_calibration_abort_restore_index = 0U;
    state->steer_zero_calibration_abort_zero_queued = false;
    steer_zero_calibration_enter_state(state,
                                       MOTION_STEER_ZERO_CAL_ABORTING,
                                       now_ms);
}

static void steer_zero_note_aborted_sdo_completion(
    motion_device_state_t *state,
    const ecu_hardware_config_t *config,
    uint8_t node_id,
    uint16_t index,
    int32_t value)
{
    if (state == NULL || config == NULL ||
        index != ECU_CANOPEN_OBJ_ACTUAL_POSITION) {
        return;
    }
    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        if (config->steer_nodes[wheel].node_id != node_id) {
            continue;
        }
        uint8_t axis_bit = (uint8_t)(1U << wheel);
        if (value == 0 &&
            state->steer_zero_calibration_abort_restore_index == 0U) {
            /* A zero write that was already in flight when HOME left the
             * adjustment domain must be rolled back. */
            state->steer_zero_calibration_zero_written_mask |= axis_bit;
        } else {
            /* Any position write issued by the abort phase is a rollback to
             * the pre-calibration coordinate at the measured midpoint. */
            state->steer_zero_calibration_zero_written_mask &=
                (uint8_t)~axis_bit;
        }
        break;
    }
}

static ecu_device_apply_result_t steer_zero_abort_step(
    motion_device_state_t *state,
    canopen_master_service_t *canopen,
    const ecu_hardware_config_t *config,
    uint32_t now_ms)
{
    if ((uint32_t)(now_ms - state->steer_zero_calibration_state_enter_ms) >
        ECU_STEER_ZERO_SETUP_TIMEOUT_MS) {
        steer_zero_calibration_fault(state, 0U, now_ms);
        return ECU_DEVICE_APPLY_REJECTED;
    }

    if (state->steer_zero_calibration_sdo_active) {
        bool faulted = false;
        uint8_t node_id = state->steer_zero_calibration_sdo_node_id;
        uint16_t index = state->steer_zero_calibration_sdo_index;
        int32_t value = state->steer_zero_calibration_sdo_value;
        if (!steer_zero_sdo_complete(state, canopen, now_ms, &faulted)) {
            if (!faulted) {
                return ECU_DEVICE_APPLY_OK;
            }
            state->steer_zero_calibration_sdo_active = false;
            state->steer_zero_calibration_fault_mask |=
                ECU_STEER_REMOTE_COMMISSION_AXIS_MASK_ALL;
        } else {
            steer_zero_note_aborted_sdo_completion(state,
                                                   config,
                                                   node_id,
                                                   index,
                                                   value);
        }
    }

    if (state->steer_zero_calibration_group_active) {
        uint32_t group = state->steer_zero_calibration_active_group_sequence;
        if (canopen_master_service_pdo_group_pending(canopen, group)) {
            return ECU_DEVICE_APPLY_OK;
        }
        if (canopen_master_service_pdo_group_failed(canopen, group) ||
            canopen_master_service_pdo_group_cancelled(canopen, group)) {
            state->steer_zero_calibration_fault_mask |=
                ECU_STEER_REMOTE_COMMISSION_AXIS_MASK_ALL;
        }
        state->steer_zero_calibration_group_active = false;
    }

    if (!state->steer_zero_calibration_abort_zero_queued) {
        int32_t zeros[ECU_WHEEL_COUNT] = {0};
        if (steer_zero_queue_velocity_group(state,
                                            canopen,
                                            config,
                                            zeros,
                                            now_ms)) {
            state->steer_zero_calibration_abort_zero_queued = true;
        }
        return ECU_DEVICE_APPLY_OK;
    }

    while (state->steer_zero_calibration_abort_restore_index <
           ECU_WHEEL_COUNT) {
        uint32_t wheel = state->steer_zero_calibration_abort_restore_index;
        uint8_t axis_bit = (uint8_t)(1U << wheel);
        if ((state->steer_zero_calibration_zero_written_mask & axis_bit) == 0U) {
            state->steer_zero_calibration_abort_restore_index++;
            continue;
        }
        if (steer_zero_queue_sdo(
                state,
                canopen,
                config->steer_nodes[wheel].node_id,
                ECU_CANOPEN_OBJ_ACTUAL_POSITION,
                0U,
                4U,
                state->steer_zero_calibration_midpoint_counts[wheel],
                now_ms)) {
            state->steer_zero_calibration_abort_restore_index++;
        }
        return ECU_DEVICE_APPLY_OK;
    }

    if (state->steer_zero_calibration_zero_written_mask != 0U) {
        /* A rollback SDO failed.  Do not resume normal steering with mixed
         * coordinate systems; stay fail-closed and expose the fault. */
        steer_zero_calibration_fault(
            state,
            state->steer_zero_calibration_zero_written_mask,
            now_ms);
        return ECU_DEVICE_APPLY_REJECTED;
    }

    state->steer_zero_calibration_group_active = false;
    state->steer_zero_calibration_sdo_active = false;
    steer_profile_setup_reset(state);
    steer_zero_calibration_enter_state(state,
                                       MOTION_STEER_ZERO_CAL_IDLE,
                                       now_ms);
    return ECU_DEVICE_APPLY_OK;
}

static ecu_device_apply_result_t steer_zero_calibration_step(
    motion_device_state_t *state,
    canopen_master_service_t *canopen,
    const ecu_hardware_config_t *config,
    uint32_t now_ms)
{
    if (state == NULL || canopen == NULL || config == NULL) {
        return ECU_DEVICE_APPLY_INVALID_ARGUMENT;
    }

    state->steer_normal_pdo_allowed = false;
    state->steer_next_group_valid = false;

    if (state->steer_zero_calibration_state == MOTION_STEER_ZERO_CAL_IDLE &&
        state->steer_zero_calibration_requested &&
        !state->steer_zero_calibration_domain_active) {
        state->steer_zero_calibration_requested = false;
        return ECU_DEVICE_APPLY_OK;
    }

    if (!state->steer_zero_calibration_domain_active &&
        state->steer_zero_calibration_state != MOTION_STEER_ZERO_CAL_IDLE &&
        state->steer_zero_calibration_state != MOTION_STEER_ZERO_CAL_ABORTING) {
        if (state->steer_zero_calibration_state ==
            MOTION_STEER_ZERO_CAL_COMPLETE) {
            state->steer_zero_calibration_requested = false;
            steer_profile_setup_reset(state);
            steer_zero_calibration_enter_state(state,
                                               MOTION_STEER_ZERO_CAL_IDLE,
                                               now_ms);
            return ECU_DEVICE_APPLY_OK;
        }
        steer_zero_begin_abort(state, now_ms);
    }

    if ((state->steer_zero_calibration_state == MOTION_STEER_ZERO_CAL_COMPLETE ||
         state->steer_zero_calibration_state == MOTION_STEER_ZERO_CAL_FAULT) &&
        state->steer_zero_calibration_requested) {
        /* A second HOME-center/B1 triple click is the explicit retry gesture.
         * It clears the maintenance FSM state only after the normal vehicle
         * command path has already inhibited ordinary CAN2 motion.
         */
        state->steer_zero_calibration_state = MOTION_STEER_ZERO_CAL_IDLE;
    }

    if (state->steer_zero_calibration_state == MOTION_STEER_ZERO_CAL_COMPLETE &&
        !state->steer_zero_calibration_requested &&
        (uint32_t)(now_ms - state->steer_zero_calibration_state_enter_ms) >= 1000U) {
        /* Homing uses calibration-specific velocity/acceleration values. Force
         * the normal 3000-rpm profile to be written and read back again before
         * ordinary steering resumes.
         */
        steer_profile_setup_reset(state);
        steer_zero_calibration_enter_state(state, MOTION_STEER_ZERO_CAL_IDLE, now_ms);
        return ECU_DEVICE_APPLY_OK;
    }

    if (state->steer_zero_calibration_state == MOTION_STEER_ZERO_CAL_IDLE &&
        state->steer_zero_calibration_requested) {
        memset(state->steer_zero_calibration_direction_start_counts,
               0,
               sizeof(state->steer_zero_calibration_direction_start_counts));
        memset(state->steer_zero_calibration_left_hit_counts,
               0,
               sizeof(state->steer_zero_calibration_left_hit_counts));
        memset(state->steer_zero_calibration_right_hit_counts,
               0,
               sizeof(state->steer_zero_calibration_right_hit_counts));
        memset(state->steer_zero_calibration_midpoint_counts,
               0,
               sizeof(state->steer_zero_calibration_midpoint_counts));
        memset(state->steer_zero_calibration_peak_current_10ma,
               0,
               sizeof(state->steer_zero_calibration_peak_current_10ma));
        memset(state->steer_zero_calibration_return_last_error_counts,
               0,
               sizeof(state->steer_zero_calibration_return_last_error_counts));
        memset(state->steer_zero_calibration_midpoint_stable_samples,
               0,
               sizeof(state->steer_zero_calibration_midpoint_stable_samples));
        state->steer_zero_calibration_fault_mask = 0U;
        state->steer_zero_calibration_setup_node_index = 0U;
        state->steer_zero_calibration_setup_step = 0U;
        state->steer_zero_calibration_setup_sdo_retry_count = 0U;
        state->steer_zero_calibration_zero_write_index = 0U;
        state->steer_zero_calibration_zero_written_mask = 0U;
        state->steer_zero_calibration_abort_restore_index = 0U;
        state->steer_zero_calibration_abort_zero_queued = false;
        state->steer_zero_calibration_sdo_active = false;
        state->steer_zero_calibration_group_active = false;
        state->steer_zero_calibration_prepare_zero_queued = false;
        state->steer_zero_calibration_verify_sync_sent = false;
        state->steer_zero_calibration_prepare_settle_start_ms = 0U;
        steer_zero_calibration_enter_state(state,
                                           MOTION_STEER_ZERO_CAL_SETUP,
                                           now_ms);
    }

    switch (state->steer_zero_calibration_state) {
    case MOTION_STEER_ZERO_CAL_ABORTING:
        return steer_zero_abort_step(state, canopen, config, now_ms);
    case MOTION_STEER_ZERO_CAL_SETUP:
        if ((uint32_t)(now_ms - state->steer_zero_calibration_state_enter_ms) >
            ECU_STEER_ZERO_SETUP_TIMEOUT_MS) {
            steer_zero_calibration_fault(state, 0U, now_ms);
            break;
        }
        if (steer_zero_prepare_velocity_phase(state, canopen, config, now_ms)) {
            for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
                canopen_node_feedback_t feedback;
                if (!steer_zero_feedback(canopen, config, wheel, &feedback)) {
                    steer_zero_calibration_fault(state,
                                                 (uint8_t)(1U << wheel),
                                                 now_ms);
                    break;
                }
                state->steer_zero_calibration_direction_start_counts[wheel] =
                    feedback.actual_position_counts;
            }
            if (state->steer_zero_calibration_state != MOTION_STEER_ZERO_CAL_FAULT) {
                steer_zero_calibration_enter_state(
                    state,
                    MOTION_STEER_ZERO_CAL_SEARCH_LEFT,
                    now_ms);
            }
        }
        break;
    case MOTION_STEER_ZERO_CAL_SEARCH_LEFT:
        if (steer_zero_run_search(state, canopen, config, now_ms, true)) {
            steer_zero_begin_velocity_prepare(state);
            steer_zero_calibration_enter_state(state,
                                               MOTION_STEER_ZERO_CAL_RETREAT_LEFT,
                                               now_ms);
        }
        break;
    case MOTION_STEER_ZERO_CAL_RETREAT_LEFT:
        if (steer_zero_prepare_velocity_phase(state, canopen, config, now_ms)) {
            for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
                canopen_node_feedback_t feedback;
                if (!steer_zero_feedback(canopen, config, wheel, &feedback)) {
                    steer_zero_calibration_fault(state,
                                                 (uint8_t)(1U << wheel),
                                                 now_ms);
                    break;
                }
                state->steer_zero_calibration_direction_start_counts[wheel] =
                    feedback.actual_position_counts;
            }
            if (state->steer_zero_calibration_state != MOTION_STEER_ZERO_CAL_FAULT) {
                steer_zero_calibration_enter_state(
                    state,
                    MOTION_STEER_ZERO_CAL_SEARCH_RIGHT,
                    now_ms);
            }
        }
        break;
    case MOTION_STEER_ZERO_CAL_SEARCH_RIGHT:
        if (steer_zero_run_search(state, canopen, config, now_ms, false)) {
            for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
                int64_t sum =
                    (int64_t)state->steer_zero_calibration_left_hit_counts[wheel] +
                    (int64_t)state->steer_zero_calibration_right_hit_counts[wheel];
                state->steer_zero_calibration_midpoint_counts[wheel] =
                    (int32_t)(sum / 2);
            }
            steer_zero_begin_velocity_prepare(state);
            steer_zero_calibration_enter_state(state,
                                               MOTION_STEER_ZERO_CAL_RETREAT_RIGHT,
                                               now_ms);
        }
        break;
    case MOTION_STEER_ZERO_CAL_RETREAT_RIGHT:
        if (steer_zero_prepare_velocity_phase(state, canopen, config, now_ms)) {
            memset(state->steer_zero_calibration_return_last_error_counts,
                   0,
                   sizeof(state->steer_zero_calibration_return_last_error_counts));
            memset(state->steer_zero_calibration_midpoint_stable_samples,
                   0,
                   sizeof(state->steer_zero_calibration_midpoint_stable_samples));
            steer_zero_calibration_enter_state(state,
                                               MOTION_STEER_ZERO_CAL_RETURN_MID,
                                               now_ms);
        }
        break;
    case MOTION_STEER_ZERO_CAL_RETURN_MID:
        if (steer_zero_run_return_mid(state, canopen, config, now_ms)) {
            state->steer_zero_calibration_zero_write_index = 0U;
            state->steer_zero_calibration_setup_step = 0U;
            steer_zero_calibration_enter_state(state,
                                               MOTION_STEER_ZERO_CAL_WRITE_ZERO,
                                               now_ms);
        }
        break;
    case MOTION_STEER_ZERO_CAL_WRITE_ZERO:
        if (steer_zero_write_zero_step(state, canopen, config, now_ms)) {
            state->steer_zero_calibration_verify_sync_sent = false;
            steer_zero_calibration_enter_state(state,
                                               MOTION_STEER_ZERO_CAL_VERIFY_ZERO,
                                               now_ms);
        }
        break;
    case MOTION_STEER_ZERO_CAL_VERIFY_ZERO:
        if (steer_zero_verify_zero(state, canopen, config, now_ms)) {
            state->steer_zero_calibration_requested = false;
            steer_zero_calibration_enter_state(state,
                                               MOTION_STEER_ZERO_CAL_COMPLETE,
                                               now_ms);
        }
        break;
    case MOTION_STEER_ZERO_CAL_COMPLETE:
        break;
    case MOTION_STEER_ZERO_CAL_FAULT:
        {
            int32_t zeros[ECU_WHEEL_COUNT] = {0};
            if (steer_zero_velocity_group_ready(state, canopen, now_ms) &&
                (uint32_t)(now_ms - state->steer_zero_calibration_last_pdo_ms) >=
                    ECU_STEER_ZERO_VELOCITY_PDO_PERIOD_MS) {
                (void)steer_zero_queue_velocity_group(state,
                                                      canopen,
                                                      config,
                                                      zeros,
                                                      now_ms);
            }
        }
        return ECU_DEVICE_APPLY_REJECTED;
    case MOTION_STEER_ZERO_CAL_IDLE:
    default:
        break;
    }

    return state->steer_zero_calibration_state == MOTION_STEER_ZERO_CAL_FAULT ?
        ECU_DEVICE_APPLY_REJECTED : ECU_DEVICE_APPLY_OK;
}

static uint32_t next_drive_group_sequence(motion_device_state_t *state)
{
    state->drive_group_sequence_counter++;
    if (state->drive_group_sequence_counter == 0U ||
        state->drive_group_sequence_counter >= ECU_DRIVE_GROUP_SEQUENCE_BASE) {
        state->drive_group_sequence_counter = 1U;
    }
    return ECU_DRIVE_GROUP_SEQUENCE_BASE | state->drive_group_sequence_counter;
}

static bool drive_axis_realtime_ready(motion_device_state_t *state,
                                      const canopen_master_service_t *canopen,
                                      uint8_t node_id,
                                      uint32_t wheel)
{
    canopen_node_feedback_t feedback;

    if (state == NULL || canopen == NULL || wheel >= ECU_WHEEL_COUNT ||
        node_id == 0U) {
        return false;
    }

    if (!canopen_master_service_get_node_feedback(canopen, node_id, &feedback) ||
        !can2_feedback_operation_enabled(node_id, &feedback)) {
        state->drive_realtime_enabled[wheel] = false;
        return false;
    }

    if (!state->drive_realtime_enabled[wheel]) {
        state->drive_pending_velocity[wheel] = true;
    }
    state->drive_realtime_enabled[wheel] = true;
    return true;
}

static bool all_drive_axes_realtime_ready(motion_device_state_t *state,
                                          const canopen_master_service_t *canopen,
                                          const ecu_hardware_config_t *config)
{
    if (!commissioning_policy_allows_drive_rpdo()) {
        return false;
    }

    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        if (!drive_axis_realtime_ready(state,
                                       canopen,
                                       config->drive_nodes[wheel].node_id,
                                       wheel)) {
            return false;
        }
    }
    return true;
}

static bool build_drive_group_targets(motion_device_state_t *state,
                                      uint32_t elapsed_ms,
                                      uint32_t now_ms,
                                      int32_t out_velocity_units[ECU_WHEEL_COUNT],
                                      int16_t out_current_10ma[ECU_WHEEL_COUNT],
                                      motion_drive_command_kind_t *out_kind,
                                      bool out_enable_requested[ECU_WHEEL_COUNT])
{
    bool group_changed = false;
    motion_drive_command_kind_t group_kind = MOTION_DRIVE_COMMAND_VELOCITY;
    int32_t requested_velocity_units[ECU_WHEEL_COUNT] = {0};
    int32_t current_velocity_units[ECU_WHEEL_COUNT] = {0};
    int16_t requested_current_10ma[ECU_WHEEL_COUNT] = {0};
    int16_t current_current_10ma[ECU_WHEEL_COUNT] = {0};
    bool all_velocity_axes_enabled = true;

    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        if (!state->drive_latest_velocity_valid[wheel]) {
            return false;
        }

        motion_drive_command_kind_t requested_kind =
            state->drive_latest_command_kind[wheel];
        if (wheel == 0U) {
            group_kind = requested_kind;
        } else if (requested_kind != group_kind) {
            return false;
        }

        requested_velocity_units[wheel] =
            state->drive_latest_velocity_units[wheel];
        requested_current_10ma[wheel] =
            state->drive_latest_current_10ma[wheel];
        if (state->drive_last_velocity_valid[wheel] &&
            state->drive_last_command_kind[wheel] ==
                MOTION_DRIVE_COMMAND_VELOCITY) {
            current_velocity_units[wheel] =
                state->drive_last_velocity_units[wheel];
        } else if (state->drive_last_velocity_valid[wheel] &&
                   state->drive_last_command_kind[wheel] ==
                       MOTION_DRIVE_COMMAND_CURRENT) {
            current_current_10ma[wheel] =
                state->drive_last_current_10ma[wheel];
        }
        if (!state->drive_latest_enable_requested[wheel]) {
            all_velocity_axes_enabled = false;
        }
    }

    int32_t shaped_velocity_units[ECU_WHEEL_COUNT] = {0};
    int16_t shaped_current_10ma[ECU_WHEEL_COUNT] = {0};
    if (group_kind == MOTION_DRIVE_COMMAND_VELOCITY &&
        all_velocity_axes_enabled) {
        if (!motion_setpoint_shape_drive_group(
                current_velocity_units,
                requested_velocity_units,
                elapsed_ms,
                shaped_velocity_units,
                &state->drive_reversal_through_zero)) {
            return false;
        }
    } else if (group_kind == MOTION_DRIVE_COMMAND_CURRENT &&
               all_velocity_axes_enabled) {
        if (!motion_setpoint_shape_drive_current_group(
                current_current_10ma,
                requested_current_10ma,
                elapsed_ms,
                shaped_current_10ma)) {
            return false;
        }
        state->drive_reversal_through_zero = false;
    } else {
        /* Disabled groups bypass comfort shaping so safe zero/disable intent
         * cannot be delayed by a previous motion command. */
        state->drive_reversal_through_zero = false;
    }

    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        motion_drive_command_kind_t requested_kind =
            state->drive_latest_command_kind[wheel];

        bool enable_requested = state->drive_latest_enable_requested[wheel];
        int32_t limited = shaped_velocity_units[wheel];
        int16_t current_10ma = shaped_current_10ma[wheel];

        if (requested_kind == MOTION_DRIVE_COMMAND_CURRENT) {
            limited = 0;
            if (!enable_requested) {
                current_10ma = 0;
            }
        }
        if (!enable_requested) {
            limited = 0;
        }

        out_velocity_units[wheel] = limited;
        out_current_10ma[wheel] = current_10ma;
        out_enable_requested[wheel] = enable_requested;
        bool nonzero_velocity_refresh_due =
            requested_kind == MOTION_DRIVE_COMMAND_VELOCITY &&
            enable_requested &&
            (limited != 0 || state->drive_last_velocity_units[wheel] != 0) &&
            (uint32_t)(now_ms - state->drive_last_target_update_ms[wheel]) >=
                ECU_CANOPEN_DRIVE_VELOCITY_REFRESH_MS;
        bool current_refresh_due =
            requested_kind == MOTION_DRIVE_COMMAND_CURRENT &&
            enable_requested &&
            current_10ma != 0 &&
            (uint32_t)(now_ms - state->drive_last_target_update_ms[wheel]) >=
                ECU_CANOPEN_DRIVE_VELOCITY_REFRESH_MS;
        if (!state->drive_last_velocity_valid[wheel] ||
            state->drive_pending_velocity[wheel] ||
            nonzero_velocity_refresh_due ||
            current_refresh_due ||
            state->drive_last_command_kind[wheel] != requested_kind ||
            state->drive_last_enable_requested[wheel] != enable_requested ||
            (requested_kind == MOTION_DRIVE_COMMAND_VELOCITY &&
             i32_changed_beyond_deadband(state->drive_last_velocity_units[wheel],
                                         limited,
                                         ECU_CANOPEN_DRIVE_PDO_CHANGE_THRESHOLD_UNITS)) ||
            (requested_kind == MOTION_DRIVE_COMMAND_CURRENT &&
             state->drive_last_current_10ma[wheel] != current_10ma)) {
            group_changed = true;
        }
    }

    if (out_kind != NULL) {
        *out_kind = group_kind;
    }
    return group_changed;
}

static bool queue_drive_group(canopen_master_service_t *canopen,
                              const ecu_hardware_config_t *config,
                              motion_device_state_t *state,
                              const int32_t velocity_units[ECU_WHEEL_COUNT],
                              const int16_t current_10ma[ECU_WHEEL_COUNT],
                              motion_drive_command_kind_t group_kind,
                              const bool enable_requested[ECU_WHEEL_COUNT],
                              uint32_t now_ms)
{
    if (!commissioning_policy_allows_drive_rpdo() ||
        canopen_master_service_pdo_queue_available(canopen) < ECU_DRIVE_GROUP_PDO_FRAME_COUNT) {
        return false;
    }

    canopen_master_pdo_request_t requests[ECU_DRIVE_GROUP_PDO_FRAME_COUNT];
    uint32_t group_sequence = next_drive_group_sequence(state);
    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        uint16_t control_word = enable_requested[wheel] ?
            SERVO_DRIVE_CONTROL_ENABLE_OPERATION :
            SERVO_DRIVE_CONTROL_DISABLE_VOLTAGE;
        if (group_kind == MOTION_DRIVE_COMMAND_CURRENT) {
            if (!build_drive_current_rpdo_request(&requests[wheel],
                                                  &config->drive_nodes[wheel],
                                                  control_word,
                                                  current_10ma[wheel],
                                                  group_sequence)) {
                return false;
            }
        } else {
            if (!build_drive_velocity_rpdo_request(&requests[wheel],
                                                   &config->drive_nodes[wheel],
                                                   control_word,
                                                   velocity_units[wheel],
                                                   group_sequence)) {
                return false;
            }
        }
    }

    canopen_master_pdo_group_descriptor_t descriptor = {
        .expected_frames = ECU_DRIVE_GROUP_PDO_FRAME_COUNT,
        .arm_frame_count = ECU_DRIVE_GROUP_PDO_FRAME_COUNT,
        .trigger_frame_count = 0U,
        .axis_mask = ECU_STEER_REMOTE_COMMISSION_AXIS_MASK_ALL,
        .position_group = false,
        .sync_after_arm = true,
        .sync_after_trigger = false
    };

    if (!canopen_master_service_queue_pdo_batch_with_descriptor(
            canopen,
            requests,
            ECU_DRIVE_GROUP_PDO_FRAME_COUNT,
            &descriptor)) {
        for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
            state->drive_pdo_tx_error_count[wheel]++;
        }
        state->drive_group_failure_count++;
        return false;
    }

    state->drive_active_group_sequence = group_sequence;
    state->drive_active_group_submit_ms = now_ms;
    memcpy(state->drive_active_group_velocity_units,
           velocity_units,
           sizeof(state->drive_active_group_velocity_units));
    memcpy(state->drive_active_group_current_10ma,
           current_10ma,
           sizeof(state->drive_active_group_current_10ma));
    state->drive_active_group_kind = group_kind;
    memcpy(state->drive_active_group_enable_requested,
           enable_requested,
           sizeof(state->drive_active_group_enable_requested));
    state->drive_group_active = true;
    return true;
}

static void finish_completed_drive_group(motion_device_state_t *state,
                                         uint32_t now_ms)
{
    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        int32_t velocity = state->drive_active_group_velocity_units[wheel];
        int16_t current_10ma = state->drive_active_group_current_10ma[wheel];
        bool enable_requested =
            state->drive_active_group_enable_requested[wheel];
        state->drive_last_velocity_valid[wheel] = true;
        state->drive_last_velocity_units[wheel] = velocity;
        state->drive_last_current_10ma[wheel] = current_10ma;
        state->drive_last_command_kind[wheel] = state->drive_active_group_kind;
        state->drive_last_enable_requested[wheel] = enable_requested;
        state->drive_last_target_update_ms[wheel] = now_ms;
        state->drive_pending_velocity[wheel] =
            state->drive_active_group_kind != state->drive_latest_command_kind[wheel] ||
            velocity != state->drive_latest_velocity_units[wheel] ||
            current_10ma != state->drive_latest_current_10ma[wheel] ||
            enable_requested != state->drive_latest_enable_requested[wheel];
    }

    state->last_target_update_ms = now_ms;
    state->drive_group_complete_count++;
    state->can2_realtime_consecutive_failure_count = 0U;
    state->drive_group_active = false;
    state->drive_active_group_sequence = 0U;
    state->drive_active_group_kind = MOTION_DRIVE_COMMAND_VELOCITY;
}

static bool drive_safe_stop_required(const motion_device_state_t *state)
{
    if (state == NULL) {
        return false;
    }
    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        if (!state->drive_last_velocity_valid[wheel] ||
            state->drive_last_enable_requested[wheel] ||
            state->drive_last_velocity_units[wheel] != 0 ||
            state->drive_last_current_10ma[wheel] != 0) {
            return true;
        }
    }
    return false;
}

static bool drive_zero_intent_completed(const motion_device_state_t *state,
                                        bool enable_requested)
{
    if (state == NULL) {
        return false;
    }
    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        if (!state->drive_last_velocity_valid[wheel] ||
            state->drive_last_velocity_units[wheel] != 0 ||
            state->drive_last_current_10ma[wheel] != 0 ||
            state->drive_last_enable_requested[wheel] != enable_requested) {
            return false;
        }
    }
    return true;
}

static bool queue_drive_safe_stop(motion_device_state_t *state,
                                  canopen_master_service_t *canopen,
                                  const ecu_hardware_config_t *config,
                                  uint32_t now_ms)
{
    int32_t zero_velocity[ECU_WHEEL_COUNT] = {0};
    int16_t zero_current[ECU_WHEEL_COUNT] = {0};
    bool disable[ECU_WHEEL_COUNT] = {false};

    if (state == NULL || canopen == NULL || config == NULL ||
        state->drive_group_active ||
        !canopen_master_service_realtime_pdo_idle(canopen)) {
        return true;
    }
    if (!queue_drive_group(canopen,
                           config,
                           state,
                           zero_velocity,
                           zero_current,
                           MOTION_DRIVE_COMMAND_VELOCITY,
                           disable,
                           now_ms)) {
        return false;
    }
    state->drive_safe_stop_pending = false;
    state->drive_safe_stop_count++;
    return true;
}

static void force_can2_drive_zero_intent(motion_device_state_t *state,
                                         bool enable_requested)
{
    if (state == NULL) {
        return;
    }

    /* Always discard the cached nonzero command.  Recovery may keep the servo
     * Operation Enabled at zero speed so its own TPDO gate can become true;
     * actual safety shutdown still requests disabled output. */
    bool new_zero_intent_required =
        !drive_zero_intent_completed(state, enable_requested);
    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        new_zero_intent_required =
            new_zero_intent_required ||
            state->drive_latest_velocity_units[wheel] != 0 ||
            state->drive_latest_current_10ma[wheel] != 0 ||
            state->drive_latest_enable_requested[wheel] != enable_requested;
    }
    if (enable_requested) {
        state->drive_safe_stop_pending = false;
    } else {
        state->drive_safe_stop_pending =
            state->drive_safe_stop_pending || new_zero_intent_required;
    }
    state->drive_next_group_valid = false;
    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        state->drive_latest_velocity_units[wheel] = 0;
        state->drive_latest_current_10ma[wheel] = 0;
        state->drive_latest_command_kind[wheel] =
            MOTION_DRIVE_COMMAND_VELOCITY;
        state->drive_latest_enable_requested[wheel] = enable_requested;
        state->drive_latest_velocity_valid[wheel] = true;
        if (new_zero_intent_required) {
            state->drive_pending_velocity[wheel] = true;
        }
    }
}

static void force_can2_drive_safe_stop_intent(motion_device_state_t *state)
{
    force_can2_drive_zero_intent(state, false);
}

static bool can2_recovery_zero_enable_permitted(
    const motion_device_state_t *state)
{
    return state != NULL && state->last_motion_command_valid &&
           can2_zero_speed_operation_enable_permitted(
               &state->last_motion_command);
}

static void force_can2_drive_recovery_zero_intent(
    motion_device_state_t *state)
{
    force_can2_drive_zero_intent(
        state,
        can2_recovery_zero_enable_permitted(state));
}

static bool recover_or_latch_can2_transient_failure(motion_device_state_t *state,
                                                    canopen_master_service_t *canopen,
                                                    uint32_t now_ms,
                                                    bool force_latch)
{
    if (state == NULL) {
        return true;
    }

    state->can2_realtime_consecutive_failure_count++;
    /* Ordinary submission/timeout failures are recoverable transport events.
     * A permanent steering inhibit is reserved for a partial trigger group,
     * where different wheels may genuinely hold different targets.  Repeated
     * local failures remain visible in diagnostics but must not create the old
     * self-locking degraded state that only an operator disable could clear.
     */
    bool latch_required = force_latch;

    if (canopen != NULL) {
        canopen_master_service_cancel_realtime_pdo(canopen);
        (void)canopen_master_service_recover_transport(canopen);
    }
    state->can2_realtime_transient_recovery_count++;
    state->can2_realtime_last_recovery_ms = now_ms;
    state->steer_group_active = false;
    state->steer_active_group_node5_only = false;
    state->steer_active_group_axis_mask = 0U;
    state->steer_active_group_sequence = 0U;
    state->steer_next_group_valid = false;
    state->drive_group_active = false;
    state->drive_active_group_sequence = 0U;
    state->drive_next_group_valid = false;
    state->drive_reversal_through_zero = false;
    state->steer_group_commanded_speed_counts_per_sec = 0;
    state->steer_follow_band = MOTION_STEER_FOLLOW_BAND_HOLD;
    state->steer_safe_stop_pending = latch_required;
    state->can2_recovery_steer_sync_pending = true;
    state->can2_recovery_steer_group_sequence = 0U;
    if (latch_required) {
        state->can2_partial_group_recovery_active = true;
        state->can2_partial_group_recovery_start_ms = now_ms;
        state->can2_partial_group_ready_since_ms = 0U;
    }

    force_can2_drive_safe_stop_intent(state);

    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        if (state->steer_latest_target_valid[wheel]) {
            state->steer_pending_target[wheel] = true;
        }
    }
    state->steer_group_degraded = latch_required;
    return latch_required;
}

static bool fail_active_drive_group(motion_device_state_t *state,
                                    canopen_master_service_t *canopen,
                                    uint32_t now_ms)
{
    state->drive_group_failure_count++;
    return recover_or_latch_can2_transient_failure(state,
                                                   canopen,
                                                   now_ms,
                                                   false);
}

static bool canopen_pdo_lane_busy_for_other_group(
    const canopen_master_service_t *canopen,
    uint32_t allowed_group_sequence)
{
    if (canopen == NULL) {
        return false;
    }
    if (canopen->pdo_in_flight &&
        canopen->pdo_in_flight_request.group_sequence != allowed_group_sequence) {
        return true;
    }
    if (canopen->sync_in_flight) {
        return true;
    }
    if (canopen->active_pdo_group_sequence != 0U &&
        canopen->active_pdo_group_sequence != allowed_group_sequence &&
        (canopen->active_pdo_group_state == CANOPEN_MASTER_PDO_GROUP_STATE_QUEUED ||
         canopen->active_pdo_group_state == CANOPEN_MASTER_PDO_GROUP_STATE_ARM_IN_FLIGHT ||
         canopen->active_pdo_group_state == CANOPEN_MASTER_PDO_GROUP_STATE_TRIGGER_IN_FLIGHT)) {
        return true;
    }
    return false;
}

static void send_can2_feedback_sync_if_due(motion_device_state_t *state,
                                            canopen_master_service_t *canopen,
                                            uint32_t now_ms)
{
    if (state == NULL || canopen == NULL ||
        (uint32_t)(now_ms - state->can2_feedback_last_sync_ms) <
            ECU_CANOPEN_STEER_PDO_PERIOD_MS) {
        return;
    }

    if (canopen_master_service_send_feedback_sync(canopen, now_ms)) {
        state->can2_feedback_last_sync_ms = now_ms;
    }
}

static uint8_t can2_motion_node_mask(uint8_t node_id)
{
    if (node_id < ECU_CANOPEN_DRIVE_FR_NODE_ID ||
        node_id > ECU_CANOPEN_STEER_RR_NODE_ID) {
        return 0U;
    }
    return (uint8_t)(1U << (node_id - ECU_CANOPEN_DRIVE_FR_NODE_ID));
}

static bool can2_motion_node_is_steer(uint8_t node_id)
{
    return node_id >= ECU_CANOPEN_STEER_FR_NODE_ID &&
           node_id <= ECU_CANOPEN_STEER_RR_NODE_ID;
}

static void reset_can2_motion_operational_request(motion_device_state_t *state)
{
    if (state == NULL) {
        return;
    }
    state->can2_motion_operational_nmt_sent_mask = 0U;
    state->can2_motion_operational_nmt_last_ms = 0U;
}

static uint8_t can2_node_index(uint8_t node_id)
{
    return (node_id >= ECU_CANOPEN_DRIVE_FR_NODE_ID &&
            node_id <= ECU_CANOPEN_STEER_RR_NODE_ID) ?
           (uint8_t)(node_id - ECU_CANOPEN_DRIVE_FR_NODE_ID) : 0xFFU;
}

static bool can2_motion_high_voltage_ready(const motion_device_state_t *state);

static void observe_can2_node_recovery_state(motion_device_state_t *state,
                                             const canopen_master_service_t *canopen,
                                             uint32_t now_ms)
{
    if (state == NULL || canopen == NULL) {
        return;
    }

    uint8_t previous_pending_mask = state->can2_node_recovery_pending_mask;
    uint8_t stale_feedback_mask = 0U;
    for (uint8_t node_id = ECU_CANOPEN_DRIVE_FR_NODE_ID;
         node_id <= ECU_CANOPEN_STEER_RR_NODE_ID;
         ++node_id) {
        uint8_t index = can2_node_index(node_id);
        uint8_t bit = can2_motion_node_mask(node_id);
        canopen_node_feedback_t feedback;
        if (index >= ECU_CANOPEN_CAN2_MOTION_NODE_COUNT || bit == 0U ||
            !canopen_master_service_get_node_feedback(canopen,
                                                      node_id,
                                                      &feedback)) {
            continue;
        }

        if (feedback.bootup_count != state->can2_node_bootup_seen[index]) {
            state->can2_node_bootup_seen[index] = feedback.bootup_count;
            state->can2_node_recovery_pending_mask |= bit;
            state->can2_node_recovery_entry_mask |= bit;
            state->can2_node_recovery_attempts[index] = 0U;
            state->can2_motion_operational_nmt_sent_mask &= (uint8_t)~bit;
        }

        bool recent_non_operational_heartbeat =
            feedback.heartbeat_valid && feedback.nmt_state != 5U &&
            (uint32_t)(now_ms - feedback.last_heartbeat_ms) <=
                ECU_CANOPEN_HEARTBEAT_TIMEOUT_MS;
        bool lost_confirmed_heartbeat =
            feedback.heartbeat_operational_seen && !feedback.feedback_fresh &&
            (uint32_t)(now_ms - feedback.last_heartbeat_ms) >
                ECU_CANOPEN_HEARTBEAT_TIMEOUT_MS;
        bool operational_heartbeat_recent =
            feedback.heartbeat_valid && feedback.nmt_state == 5U &&
            (uint32_t)(now_ms - feedback.last_heartbeat_ms) <=
                ECU_CANOPEN_HEARTBEAT_TIMEOUT_MS;
        bool tpdo1_required =
            !(node_id == ECU_CANOPEN_STEER_RR_NODE_ID &&
              ECU_CANOPEN_NODE8_TPDO1_ACCEPTANCE_WORKAROUND != 0U &&
              feedback.tpdo1_rx_count == 0U);
        bool stale_motion_feedback =
            can2_motion_high_voltage_ready(state) &&
            operational_heartbeat_recent &&
            ((!feedback.tpdo0_valid ||
              (uint32_t)(now_ms - feedback.last_tpdo0_ms) >=
                  ECU_CANOPEN_MOTION_FEEDBACK_STALE_RECOVERY_MS) ||
             (tpdo1_required &&
              (!feedback.tpdo1_valid ||
               (uint32_t)(now_ms - feedback.last_tpdo1_ms) >=
                   ECU_CANOPEN_MOTION_FEEDBACK_STALE_RECOVERY_MS)));
        bool cia402_recovery_needed =
            feedback.tpdo1_fresh &&
            (feedback.fault_latched != 0U ||
             (feedback.statusword & CIA402_STATUS_STATE_MASK) !=
                 CIA402_STATUS_OPERATION_ENABLED);
        bool steer_startup_enable_needed =
            can2_motion_node_is_steer(node_id) &&
            can2_motion_high_voltage_ready(state) &&
            !can2_feedback_operation_enabled(node_id, &feedback);

        if (recent_non_operational_heartbeat || lost_confirmed_heartbeat ||
            stale_motion_feedback || cia402_recovery_needed ||
            steer_startup_enable_needed) {
            state->can2_node_recovery_pending_mask |= bit;
            state->can2_motion_operational_nmt_sent_mask &= (uint8_t)~bit;
            if (stale_motion_feedback) {
                stale_feedback_mask |= bit;
            }
        } else if (can2_feedback_operation_enabled(node_id, &feedback)) {
            state->can2_node_recovery_pending_mask &= (uint8_t)~bit;
            state->can2_node_recovery_attempts[index] = 0U;
        }
    }
    state->can2_stale_feedback_mask = stale_feedback_mask;
    state->can2_node_recovery_entry_mask |=
        (uint8_t)(state->can2_node_recovery_pending_mask &
                  (uint8_t)~previous_pending_mask);

    if (previous_pending_mask != 0U &&
        state->can2_node_recovery_pending_mask == 0U) {
        /* Node feedback has proved that every recovered axis is operational.
         * Re-send one coherent four-steering-axis target before traction may
         * resume.  This prevents a recovered drive axis from moving against a
         * steering axis that retained a different pre-fault target. */
        state->can2_recovery_steer_sync_pending = true;
        state->can2_recovery_steer_group_sequence = 0U;
        for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
            if (state->steer_latest_target_valid[wheel]) {
                state->steer_pending_target[wheel] = true;
            }
        }
        state->steer_realtime_last_flush_ms = 0U;
    }
}

static bool can2_all_motion_nodes_operation_enabled_fresh(
    const canopen_master_service_t *canopen)
{
    if (canopen == NULL) {
        return false;
    }

    for (uint8_t node_id = ECU_CANOPEN_DRIVE_FR_NODE_ID;
         node_id <= ECU_CANOPEN_STEER_RR_NODE_ID;
         ++node_id) {
        canopen_node_feedback_t feedback;
        if (!canopen_master_service_get_node_feedback(canopen,
                                                      node_id,
                                                      &feedback) ||
            !can2_feedback_operation_enabled(node_id, &feedback)) {
            return false;
        }
    }
    return true;
}

static bool can2_node_recovery_due(const motion_device_state_t *state,
                                   uint8_t index,
                                   uint32_t now_ms)
{
    uint32_t interval =
        state->can2_node_recovery_attempts[index] <
            ECU_CANOPEN_NODE_RECOVERY_FAST_ATTEMPTS ?
        ECU_CANOPEN_NODE_RECOVERY_RETRY_MS :
        ECU_CANOPEN_NODE_RECOVERY_COOLDOWN_MS;
    return state->can2_node_recovery_last_ms[index] == 0U ||
           (uint32_t)(now_ms - state->can2_node_recovery_last_ms[index]) >=
               interval;
}

static void service_can2_node_recovery(motion_device_state_t *state,
                                       canopen_master_service_t *canopen,
                                       uint32_t now_ms)
{
    if (state == NULL || canopen == NULL ||
        state->can2_node_recovery_pending_mask == 0U ||
        !canopen_master_service_realtime_pdo_idle(canopen) ||
        !canopen_master_service_sdo_download_idle(canopen)) {
        return;
    }

    for (uint8_t node_id = ECU_CANOPEN_DRIVE_FR_NODE_ID;
         node_id <= ECU_CANOPEN_STEER_RR_NODE_ID;
         ++node_id) {
        uint8_t index = can2_node_index(node_id);
        uint8_t bit = can2_motion_node_mask(node_id);
        if (index >= ECU_CANOPEN_CAN2_MOTION_NODE_COUNT ||
            (state->can2_node_recovery_pending_mask & bit) == 0U ||
            !can2_node_recovery_due(state, index, now_ms)) {
            continue;
        }

        canopen_node_feedback_t feedback;
        bool feedback_read = canopen_master_service_get_node_feedback(
            canopen, node_id, &feedback);
        bool vendor_fault_latched = feedback_read && feedback.tpdo1_valid &&
            feedback.fault_latched != 0U;
        bool fault_present = feedback_read && feedback.tpdo1_valid &&
            (feedback.fault_latched != 0U ||
             (feedback.statusword & CIA402_STATUS_FAULT_BIT) != 0U);
        bool steer_node = can2_motion_node_is_steer(node_id);
        bool operation_enabled =
            feedback_read && can2_feedback_operation_enabled(node_id, &feedback);

        if (!canopen_master_service_request_nmt(
                canopen,
                node_id,
                CANOPEN_MASTER_DEBUG_COMMAND_NMT_OPERATIONAL)) {
            return;
        }

        bool queued = true;
        if (vendor_fault_latched) {
            queued = canopen_master_service_request_sdo_write(
                canopen,
                node_id,
                ECU_CANOPEN_OBJ_FAULT_LATCHED,
                0U,
                4U,
                (int32_t)feedback.fault_latched);
        }
        if (fault_present) {
            queued = queued && canopen_master_service_request_sdo_write(
                canopen, node_id, ECU_CANOPEN_OBJ_CONTROLWORD,
                0U, 2U, SERVO_DRIVE_CONTROL_FAULT_RESET);
        }
        if (queued && steer_node && !operation_enabled) {
            /* Field test with the CAN analyzer showed Node5..8 power up with
             * no selected operation mode and not operation-enabled, then only
             * produce the expected synchronous TPDO0/TPDO1 stream after
             * profile-position mode and CiA-402 enable-operation are written.
             * Do not wait for TPDO readiness before sending this startup
             * sequence; that creates a deadlock where steering never enables
             * and drive remains held by the pre-steer safety gate.
             *
             * This sequence does not reset the node, reset communication, write
             * the actual-position object, or trigger a target position.  It
             * only selects position mode and enables the drive so the realtime
             * RPDO path can then own all steering targets.
             */
            queued =
                canopen_master_service_request_sdo_write(
                    canopen, node_id, ECU_CANOPEN_OBJ_MODES_OF_OPERATION,
                    0U, 1U, SERVO_DRIVE_MODE_PROFILE_POSITION) &&
                canopen_master_service_request_sdo_write(
                    canopen, node_id, ECU_CANOPEN_OBJ_CONTROLWORD,
                    0U, 2U, SERVO_DRIVE_CONTROL_SHUTDOWN) &&
                canopen_master_service_request_sdo_write(
                    canopen, node_id, ECU_CANOPEN_OBJ_CONTROLWORD,
                    0U, 2U, SERVO_DRIVE_CONTROL_SWITCH_ON) &&
                canopen_master_service_request_sdo_write(
                    canopen, node_id, ECU_CANOPEN_OBJ_CONTROLWORD,
                    0U, 2U, SERVO_DRIVE_CONTROL_ENABLE_OPERATION);
        } else if (queued && !steer_node && !operation_enabled) {
            queued =
                canopen_master_service_request_sdo_write(
                    canopen, node_id, ECU_CANOPEN_OBJ_MODES_OF_OPERATION,
                    0U, 1U, SERVO_DRIVE_MODE_PROFILE_VELOCITY) &&
                canopen_master_service_request_sdo_write(
                    canopen, node_id, ECU_CANOPEN_OBJ_CONTROLWORD,
                    0U, 2U, SERVO_DRIVE_CONTROL_SHUTDOWN) &&
                canopen_master_service_request_sdo_write(
                    canopen, node_id, ECU_CANOPEN_OBJ_CONTROLWORD,
                    0U, 2U, SERVO_DRIVE_CONTROL_SWITCH_ON) &&
                canopen_master_service_request_sdo_write(
                    canopen, node_id, ECU_CANOPEN_OBJ_CONTROLWORD,
                    0U, 2U, SERVO_DRIVE_CONTROL_ENABLE_OPERATION);
        }
        if (!queued) {
            return;
        }

        state->can2_motion_operational_nmt_sent_mask |= bit;
        state->can2_motion_operational_nmt_last_ms = now_ms;
        state->can2_node_recovery_last_ms[index] = now_ms;
        if (state->can2_node_recovery_attempts[index] < UINT8_MAX) {
            state->can2_node_recovery_attempts[index]++;
        }
        state->can2_node_recovery_count[index]++;
        return;
    }
}

static void service_can2_partial_group_recovery(
    motion_device_state_t *state,
    const canopen_master_service_t *canopen,
    uint32_t now_ms)
{
    if (state == NULL || canopen == NULL ||
        !state->can2_partial_group_recovery_active) {
        return;
    }

    /* A partial arm/trigger group can leave steering nodes at different
     * targets.  Recovery is automatic, but only after CAN TX is quiescent,
     * traction zero has completed locally, no node-level repair remains, and
     * all eight nodes have continuously reported operation-enabled feedback.
     */
    bool recovery_enable_requested =
        can2_recovery_zero_enable_permitted(state);
    bool recovery_ready =
        state->can2_node_recovery_pending_mask == 0U &&
        !state->drive_group_active &&
        !state->steer_group_active &&
        drive_zero_intent_completed(state, recovery_enable_requested) &&
        canopen_master_service_realtime_pdo_idle(canopen) &&
        can2_all_motion_nodes_operation_enabled_fresh(canopen);
    if (!recovery_ready) {
        state->can2_partial_group_ready_since_ms = 0U;
        return;
    }

    if (state->can2_partial_group_ready_since_ms == 0U) {
        state->can2_partial_group_ready_since_ms = now_ms;
        return;
    }
    if ((uint32_t)(now_ms - state->can2_partial_group_ready_since_ms) <
        ECU_CANOPEN_PARTIAL_GROUP_RECOVERY_STABLE_MS) {
        return;
    }

    state->can2_partial_group_recovery_active = false;
    state->can2_partial_group_ready_since_ms = 0U;
    state->steer_group_degraded = false;
    state->steer_safe_stop_pending = false;
    state->can2_recovery_steer_sync_pending = true;
    state->can2_recovery_steer_group_sequence = 0U;
    state->steer_realtime_last_flush_ms = 0U;
    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        if (state->steer_latest_target_valid[wheel]) {
            state->steer_pending_target[wheel] = true;
        }
    }
    state->can2_partial_group_recovery_count++;
}

static void reset_can2_realtime_motion_state(motion_device_state_t *state,
                                             canopen_master_service_t *canopen,
                                             uint32_t now_ms)
{
    if (state == NULL) {
        return;
    }
    (void)now_ms;

    if (canopen != NULL) {
        canopen_master_service_cancel_realtime_pdo(canopen);
    }

    /* This recovery is deliberately ECU-local.  Do not send NMT reset-node,
     * reset-communication, or re-run mode/PDO setup here: the steering drives
     * keep their calibrated zero/reference, and a remote disable/enable cycle
     * must not risk losing that reference.  Only cancel stale realtime PDO
     * queue/group state and overwrite actuator intent with safe zero targets.
     */
    state->steer_realtime_last_flush_ms = 0U;
    state->drive_realtime_last_flush_ms = 0U;

    state->steer_group_active = false;
    state->steer_active_group_sequence = 0U;
    state->steer_next_group_valid = false;
    state->steer_group_degraded = false;
    state->steer_safe_stop_pending = false;
    state->steer_group_trigger_partial_failure = false;
    state->steer_group_clean_cancelled = false;
    state->can2_partial_group_recovery_active = false;
    state->can2_recovery_steer_sync_pending = false;
    state->can2_recovery_steer_group_sequence = 0U;
    state->can2_node_recovery_entry_mask = 0U;
    state->can2_partial_group_ready_since_ms = 0U;

    state->drive_group_active = false;
    state->drive_active_group_sequence = 0U;
    state->drive_next_group_valid = false;

    state->presteer_drive_hold_active = false;
    state->presteer_target_reached = false;
    state->track_assist_steer_approximately_ready = false;
    state->presteer_missing_axis_mask = 0U;
    state->track_assist_missing_axis_mask = 0U;
    state->presteer_hold_start_ms = 0U;
    state->track_assist_steer_ready_since_ms = 0U;
    state->track_assist_steer_ready_eval_ms = 0U;

    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        state->drive_realtime_enabled[wheel] = false;
        state->drive_pending_velocity[wheel] = true;
        state->drive_latest_velocity_units[wheel] = 0;
        state->drive_latest_enable_requested[wheel] = false;
        state->drive_latest_velocity_valid[wheel] = true;
        state->drive_last_velocity_valid[wheel] = false;
        state->drive_last_velocity_units[wheel] = 0;
        state->drive_last_enable_requested[wheel] = false;

        state->steer_realtime_enabled[wheel] = false;
        state->steer_pending_target[wheel] = false;
        state->steer_commanded_target_valid[wheel] = false;
        state->steer_commanded_target_counts[wheel] = 0;
        state->steer_commanded_velocity_counts_per_sec[wheel] = 0;
        state->steer_last_commanded_position_valid[wheel] = false;
        state->steer_last_commanded_position_counts[wheel] = 0;
    }
}

static bool can2_motion_high_voltage_ready(const motion_device_state_t *state)
{
    return state != NULL &&
           state->last_motion_command_valid &&
           state->last_motion_command.high_voltage_enable &&
           state->last_motion_command.high_voltage_feedback_ready &&
           !state->last_motion_command.high_voltage_disable_request;
}

static bool can2_realtime_motion_state_needs_recovery(
    const motion_device_state_t *state)
{
    if (state == NULL) {
        return false;
    }
    return state->steer_group_active ||
           state->steer_next_group_valid ||
           state->steer_group_degraded ||
           state->steer_safe_stop_pending ||
           state->drive_group_active ||
           state->drive_next_group_valid ||
           (state->last_motion_command_valid &&
            state->last_motion_command.high_voltage_enable);
}

static void request_can2_motion_nodes_operational(motion_device_state_t *state,
                                                  canopen_master_service_t *canopen,
                                                  const ecu_hardware_config_t *config,
                                                  uint32_t now_ms)
{
    if (state == NULL || canopen == NULL || config == NULL) {
        return;
    }

    if (!can2_motion_high_voltage_ready(state)) {
        reset_can2_motion_operational_request(state);
        return;
    }

    /* NMT is sent through the same CANopen service as PDO traffic.  Do not emit
     * it while a realtime PDO/SYNC is in-flight, otherwise a controller TX
     * completion belonging to NMT/SYNC could be mistaken for the queued PDO
     * group's completion.
     */
    if (!canopen_master_service_realtime_pdo_idle(canopen) ||
        (uint32_t)(now_ms - state->can2_motion_operational_nmt_last_ms) <
            ECU_CANOPEN_DRIVE_PDO_PERIOD_MS) {
        return;
    }

    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        uint8_t node_id = config->drive_nodes[wheel].node_id;
        uint8_t node_mask = can2_motion_node_mask(node_id);
        if (node_mask != 0U &&
            (state->can2_motion_operational_nmt_sent_mask & node_mask) == 0U &&
            canopen_master_service_request_nmt(
                canopen,
                node_id,
                CANOPEN_MASTER_DEBUG_COMMAND_NMT_OPERATIONAL)) {
            state->can2_motion_operational_nmt_sent_mask |= node_mask;
            state->can2_motion_operational_nmt_last_ms = now_ms;
            return;
        }
    }

    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        uint8_t node_id = config->steer_nodes[wheel].node_id;
        uint8_t node_mask = can2_motion_node_mask(node_id);
        if (node_mask != 0U &&
            (state->can2_motion_operational_nmt_sent_mask & node_mask) == 0U &&
            canopen_master_service_request_nmt(
                canopen,
                node_id,
                CANOPEN_MASTER_DEBUG_COMMAND_NMT_OPERATIONAL)) {
            state->can2_motion_operational_nmt_sent_mask |= node_mask;
            state->can2_motion_operational_nmt_last_ms = now_ms;
            return;
        }
    }
}

static ecu_device_apply_result_t flush_drive_velocity_realtime(
    motion_device_state_t *state,
    canopen_master_service_t *canopen,
    const ecu_hardware_config_t *config,
    uint32_t now_ms)
{
    if (!commissioning_policy_allows_drive_rpdo()) {
        state->drive_next_group_valid = false;
        return ECU_DEVICE_APPLY_OK;
    }

    if (state->drive_group_active) {
        if (canopen_master_service_pdo_group_failed(canopen,
                                                    state->drive_active_group_sequence)) {
            bool latched = fail_active_drive_group(state, canopen, now_ms);
            return latched ? ECU_DEVICE_APPLY_REJECTED : ECU_DEVICE_APPLY_OK;
        }
        if (canopen_master_service_pdo_group_pending(canopen,
                                                     state->drive_active_group_sequence)) {
            return ECU_DEVICE_APPLY_OK;
        }
        finish_completed_drive_group(state, now_ms);
    }

    if (state->drive_next_group_valid) {
        if (canopen_pdo_lane_busy_for_other_group(canopen,
                                                  state->drive_active_group_sequence)) {
            return ECU_DEVICE_APPLY_OK;
        }
        bool queued = queue_drive_group(canopen,
                                        config,
                                        state,
                                        state->drive_next_group_velocity_units,
                                        state->drive_next_group_current_10ma,
                                        state->drive_next_group_kind,
                                        state->drive_next_group_enable_requested,
                                        now_ms);
        if (queued) {
            state->drive_next_group_valid = false;
            return ECU_DEVICE_APPLY_OK;
        }
        return ECU_DEVICE_APPLY_REJECTED;
    }

    if ((uint32_t)(now_ms - state->drive_realtime_last_flush_ms) <
        ECU_CANOPEN_DRIVE_PDO_PERIOD_MS) {
        return ECU_DEVICE_APPLY_OK;
    }

    uint32_t elapsed_ms = state->drive_realtime_last_flush_ms == 0U ?
                          ECU_CANOPEN_DRIVE_PDO_PERIOD_MS :
                          (uint32_t)(now_ms - state->drive_realtime_last_flush_ms);
    state->drive_realtime_last_flush_ms = now_ms;

    bool recovery_zero_intent_active =
        state->can2_node_recovery_pending_mask != 0U ||
        state->can2_partial_group_recovery_active ||
        state->can2_recovery_steer_sync_pending;
    if (recovery_zero_intent_active) {
        bool enable_requested_value =
            can2_recovery_zero_enable_permitted(state);
        if (drive_zero_intent_completed(state, enable_requested_value)) {
            return ECU_DEVICE_APPLY_OK;
        }
        if (canopen_pdo_lane_busy_for_other_group(canopen,
                                                  state->drive_active_group_sequence)) {
            return ECU_DEVICE_APPLY_OK;
        }

        /* Recovery zero is an immediate safety target, not an ordinary motion
         * trajectory.  Bypass the normal velocity rate limiter so no fragment
         * of the pre-fault nonzero command can be replayed while nodes recover. */
        int32_t zero_velocity[ECU_WHEEL_COUNT] = {0};
        int16_t zero_current[ECU_WHEEL_COUNT] = {0};
        bool enable_requested[ECU_WHEEL_COUNT] = {
            enable_requested_value,
            enable_requested_value,
            enable_requested_value,
            enable_requested_value,
        };
        if (!queue_drive_group(canopen,
                               config,
                               state,
                               zero_velocity,
                               zero_current,
                               MOTION_DRIVE_COMMAND_VELOCITY,
                               enable_requested,
                               now_ms)) {
            return ECU_DEVICE_APPLY_REJECTED;
        }
        return ECU_DEVICE_APPLY_OK;
    }

    if (!all_drive_axes_realtime_ready(state, canopen, config)) {
        state->drive_safe_stop_pending =
            state->drive_safe_stop_pending || drive_safe_stop_required(state);
        if (state->drive_safe_stop_pending &&
            !queue_drive_safe_stop(state, canopen, config, now_ms)) {
            return ECU_DEVICE_APPLY_REJECTED;
        }
        send_can2_feedback_sync_if_due(state, canopen, now_ms);
        return ECU_DEVICE_APPLY_OK;
    }

    int32_t velocity_units[ECU_WHEEL_COUNT] = {0};
    int16_t current_10ma[ECU_WHEEL_COUNT] = {0};
    bool enable_requested[ECU_WHEEL_COUNT] = {false};
    motion_drive_command_kind_t group_kind = MOTION_DRIVE_COMMAND_VELOCITY;
    if (!build_drive_group_targets(state,
                                   elapsed_ms,
                                   now_ms,
                                   velocity_units,
                                   current_10ma,
                                   &group_kind,
                                   enable_requested)) {
        canopen_master_service_note_pdo_same_target_coalesced(canopen);
        return ECU_DEVICE_APPLY_OK;
    }

    if (canopen_pdo_lane_busy_for_other_group(canopen,
                                              state->drive_active_group_sequence)) {
        memcpy(state->drive_next_group_velocity_units,
               velocity_units,
               sizeof(state->drive_next_group_velocity_units));
        memcpy(state->drive_next_group_current_10ma,
               current_10ma,
               sizeof(state->drive_next_group_current_10ma));
        state->drive_next_group_kind = group_kind;
        memcpy(state->drive_next_group_enable_requested,
               enable_requested,
               sizeof(state->drive_next_group_enable_requested));
        state->drive_next_group_valid = true;
        return ECU_DEVICE_APPLY_OK;
    }

    if (!queue_drive_group(canopen,
                           config,
                           state,
                           velocity_units,
                           current_10ma,
                           group_kind,
                           enable_requested,
                           now_ms)) {
        memcpy(state->drive_next_group_velocity_units,
               velocity_units,
               sizeof(state->drive_next_group_velocity_units));
        memcpy(state->drive_next_group_current_10ma,
               current_10ma,
               sizeof(state->drive_next_group_current_10ma));
        state->drive_next_group_kind = group_kind;
        memcpy(state->drive_next_group_enable_requested,
               enable_requested,
               sizeof(state->drive_next_group_enable_requested));
        state->drive_next_group_valid = true;
        return ECU_DEVICE_APPLY_REJECTED;
    }

    return ECU_DEVICE_APPLY_OK;
}

static void finish_completed_steer_group(motion_device_state_t *state,
                                         uint32_t now_ms)
{
    uint32_t completed_group_sequence = state->steer_active_group_sequence;
    uint8_t axis_mask = state->steer_active_group_axis_mask != 0U ?
                        state->steer_active_group_axis_mask :
                        (state->steer_active_group_node5_only ? 0x01U :
                         ECU_STEER_REMOTE_COMMISSION_AXIS_MASK_ALL);
    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        if ((axis_mask & (uint8_t)(1U << wheel)) == 0U) {
            continue;
        }
        int32_t target = state->steer_active_group_target_counts[wheel];
        state->steer_last_commanded_position_valid[wheel] = true;
        state->steer_last_commanded_position_counts[wheel] = target;
        state->steer_commanded_target_valid[wheel] = true;
        state->steer_commanded_target_counts[wheel] = target;
        state->steer_last_target_update_ms[wheel] = now_ms;
        state->steer_pending_target[wheel] =
            target != state->steer_latest_target_counts[wheel];
    }

    state->last_target_update_ms = now_ms;
    state->steer_group_complete_count++;
    state->can2_realtime_consecutive_failure_count = 0U;
    state->steer_group_active = false;
    state->steer_active_group_node5_only = false;
    state->steer_active_group_axis_mask = 0U;
    state->steer_active_group_sequence = 0U;

    if (state->can2_recovery_steer_sync_pending &&
        state->can2_recovery_steer_group_sequence != 0U &&
        completed_group_sequence ==
            state->can2_recovery_steer_group_sequence) {
        /* Only the specifically recorded post-recovery four-axis group may
         * reopen traction.  The vehicle task will publish a fresh drive
         * snapshot on its next cycle; no pre-fault velocity is retained. */
        state->can2_recovery_steer_sync_pending = false;
        state->can2_recovery_steer_group_sequence = 0U;
        state->drive_realtime_last_flush_ms = 0U;
    }
}

static void clean_cancel_active_steer_group(motion_device_state_t *state,
                                            uint32_t now_ms)
{
    state->steer_group_clean_cancelled = true;
    state->steer_last_clean_cancel_ms = now_ms;
    state->steer_group_active = false;
    state->steer_active_group_node5_only = false;
    state->steer_active_group_axis_mask = 0U;
    state->steer_active_group_sequence = 0U;
}

static bool fail_active_steer_group(motion_device_state_t *state,
                                    canopen_master_service_t *canopen,
                                    uint32_t now_ms)
{
    canopen_master_snapshot_t snapshot;
    bool trigger_or_partial_failure;

    memset(&snapshot, 0, sizeof(snapshot));
    canopen_master_service_get_snapshot(canopen, &snapshot);
    state->steer_group_failure_count++;
    trigger_or_partial_failure =
        snapshot.pdo_trigger_complete_frames > 0U ||
        snapshot.last_pdo_failed_phase ==
            (uint8_t)CANOPEN_MASTER_PDO_PHASE_STEER_TRIGGER ||
        snapshot.last_pdo_failed_phase ==
            (uint8_t)CANOPEN_MASTER_PDO_PHASE_NODE5_POSITION_TRIGGER;
    if (trigger_or_partial_failure) {
        state->steer_group_trigger_partial_failure = true;
        state->steer_last_partial_failure_ms = now_ms;
    }
    return recover_or_latch_can2_transient_failure(state,
                                                   canopen,
                                                   now_ms,
                                                   trigger_or_partial_failure);
}

#if ECU_CANOPEN_COMMISSIONING_POLICY == ECU_CANOPEN_COMMISSIONING_POLICY_STEER4_REMOTE_COMMISSIONING
static bool steering_command_is_neutral(const vehicle_actuator_command_t *command)
{
    if (command == NULL) {
        return false;
    }
    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        if (command->target_steer_deg[wheel] > 0.5f ||
            command->target_steer_deg[wheel] < -0.5f) {
            return false;
        }
    }
    return true;
}

static void request_selected_steer_nodes_operational(canopen_master_service_t *canopen,
                                                     const ecu_hardware_config_t *config,
                                                     motion_device_state_t *state,
                                                     uint8_t axis_mask)
{
    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        uint8_t axis_bit = (uint8_t)(1U << wheel);
        if ((axis_mask & axis_bit) == 0U ||
            (state->steer_commission_nmt_sent_mask & axis_bit) != 0U) {
            continue;
        }
        if (canopen_master_service_request_nmt(
                canopen,
                config->steer_nodes[wheel].node_id,
                CANOPEN_MASTER_DEBUG_COMMAND_NMT_OPERATIONAL)) {
            state->steer_commission_nmt_sent_mask |= axis_bit;
        }
    }
}

static void update_steer_remote_commissioning_state(
    motion_device_state_t *state,
    canopen_master_service_t *canopen,
    const ecu_hardware_config_t *config,
    const vehicle_actuator_command_t *command,
    uint32_t now_ms)
{
    uint8_t axis_mask = 0U;
    bool authorization_valid = steer_commissioning_authorization_valid(now_ms, &axis_mask);

    if (!commissioning_policy_allows_steer4_remote()) {
        state->steer_commission_state = STEER_REMOTE_COMMISSION_DISABLED;
        return;
    }
    if (!authorization_valid) {
        state->steer_commission_state = STEER_REMOTE_COMMISSION_WAIT_AUTH;
        state->selected_axis_mask = 0U;
        return;
    }
    state->selected_axis_mask = axis_mask;
    if (!steer_commissioning_remote_conditions_ok(command)) {
        state->steer_commission_state = STEER_REMOTE_COMMISSION_WAIT_AUTH;
        return;
    }

    if (!state->steer_commission_centered &&
        state->steer_commission_state != STEER_REMOTE_COMMISSION_CENTERING &&
        state->steer_commission_state != STEER_REMOTE_COMMISSION_WAIT_SYNC_TX_COMPLETE &&
        state->steer_commission_state != STEER_REMOTE_COMMISSION_WAIT_CENTER_SETTLE &&
        state->steer_commission_state != STEER_REMOTE_COMMISSION_WAIT_POST_COMMAND_TPDO) {
        if (!command->steer_commission_steering_neutral ||
            !steering_command_is_neutral(command)) {
            state->steer_commission_neutral_since_ms = 0U;
            state->steer_commission_state = STEER_REMOTE_COMMISSION_WAIT_NEUTRAL;
            return;
        }
        if (state->steer_commission_neutral_since_ms == 0U) {
            state->steer_commission_neutral_since_ms = now_ms;
            state->steer_commission_state = STEER_REMOTE_COMMISSION_WAIT_NEUTRAL;
            return;
        }
        if ((uint32_t)(now_ms - state->steer_commission_neutral_since_ms) <
            ECU_STEER_REMOTE_COMMISSION_NEUTRAL_MS) {
            state->steer_commission_state = STEER_REMOTE_COMMISSION_WAIT_NEUTRAL;
            return;
        }
    }

    request_selected_steer_nodes_operational(canopen, config, state, axis_mask);
    if ((state->steer_commission_nmt_sent_mask & axis_mask) != axis_mask) {
        state->steer_commission_state = STEER_REMOTE_COMMISSION_TPDO_MONITOR;
        return;
    }
    if (!steer_commissioning_axis_feedback_ready(canopen, config, axis_mask)) {
        state->steer_commission_state = STEER_REMOTE_COMMISSION_TPDO_MONITOR;
        return;
    }
    if (!steer_commissioning_axis_calibration_ready(config, axis_mask)) {
        state->steer_commission_state = STEER_REMOTE_COMMISSION_WAIT_CALIBRATION;
        return;
    }
    if (state->steer_commission_state == STEER_REMOTE_COMMISSION_CENTERING ||
        state->steer_commission_state == STEER_REMOTE_COMMISSION_WAIT_SYNC_TX_COMPLETE ||
        state->steer_commission_state == STEER_REMOTE_COMMISSION_WAIT_CENTER_SETTLE ||
        state->steer_commission_state == STEER_REMOTE_COMMISSION_WAIT_POST_COMMAND_TPDO) {
        return;
    }
    if (!state->steer_commission_centered) {
        state->steer_commission_state = STEER_REMOTE_COMMISSION_CENTERING;
    } else if (state->steer_commission_state != STEER_REMOTE_COMMISSION_ACTIVE) {
        state->steer_commission_state = STEER_REMOTE_COMMISSION_AXIS_READY;
    }
}

static void send_commissioning_sync_if_due(canopen_master_service_t *canopen,
                                           motion_device_state_t *state,
                                           uint32_t now_ms)
{
    if ((uint32_t)(now_ms - state->steer_commission_last_sync_ms) >=
        ECU_STEER_REMOTE_COMMISSION_PERIOD_MS) {
        if (canopen_master_service_send_sync(canopen, now_ms)) {
            state->steer_commission_last_sync_ms = now_ms;
        }
    }
}

static bool selected_targets_changed(const motion_device_state_t *state,
                                     const int32_t targets[ECU_WHEEL_COUNT],
                                     uint8_t axis_mask)
{
    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        if ((axis_mask & (uint8_t)(1U << wheel)) == 0U) {
            continue;
        }
        if (!state->steer_last_commanded_position_valid[wheel] ||
            i32_changed_beyond_deadband(
                state->steer_last_commanded_position_counts[wheel],
                targets[wheel],
                ECU_STEER_REMOTE_COMMISSION_TRIGGER_THRESHOLD_COUNTS)) {
            return true;
        }
    }
    return false;
}

static bool center_feedback_within_tolerance(
    const motion_device_state_t *state,
    const canopen_master_service_t *canopen,
    const ecu_hardware_config_t *config,
    const steer_axis_calibration_t calibration[ECU_WHEEL_COUNT],
    uint8_t axis_mask)
{
    (void)state;
    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        uint8_t axis_bit = (uint8_t)(1U << wheel);
        if ((axis_mask & axis_bit) == 0U) {
            continue;
        }

        canopen_node_feedback_t feedback;
        uint8_t node_id = config->steer_nodes[wheel].node_id;
        if (!canopen_master_service_get_node_feedback(canopen, node_id, &feedback) ||
            !feedback.feedback_fresh ||
            feedback.fault_latched != 0U ||
            abs_i32_delta(feedback.actual_position_counts,
                          calibration[wheel].straight_zero_offset_counts) >
                ECU_STEER_REMOTE_COMMISSION_CENTER_TOLERANCE_COUNTS) {
            return false;
        }
    }
    return true;
}

static void initialize_commissioning_ramp_from_feedback(
    motion_device_state_t *state,
    const canopen_master_service_t *canopen,
    const ecu_hardware_config_t *config,
    const steer_axis_calibration_t calibration[ECU_WHEEL_COUNT],
    uint8_t axis_mask,
    uint32_t now_ms)
{
    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        uint8_t axis_bit = (uint8_t)(1U << wheel);
        if ((axis_mask & axis_bit) == 0U) {
            state->steer_commission_ramped_target_valid[wheel] = false;
            state->steer_commission_ramped_target_counts[wheel] = 0;
            continue;
        }

        canopen_node_feedback_t feedback;
        uint8_t node_id = config->steer_nodes[wheel].node_id;
        if (canopen_master_service_get_node_feedback(canopen, node_id, &feedback) &&
            feedback.feedback_fresh &&
            feedback.fault_latched == 0U) {
            state->steer_commission_ramped_target_counts[wheel] =
                feedback.actual_position_counts;
        } else {
            state->steer_commission_ramped_target_counts[wheel] =
                calibration[wheel].straight_zero_offset_counts;
        }
        state->steer_commission_ramped_target_valid[wheel] = true;
    }
    state->steer_commission_ramp_last_ms = now_ms;
}

static void ramp_commissioning_targets(
    motion_device_state_t *state,
    const steer_axis_calibration_t calibration[ECU_WHEEL_COUNT],
    const int32_t desired_targets[ECU_WHEEL_COUNT],
    uint8_t axis_mask,
    uint32_t now_ms,
    int32_t out_targets[ECU_WHEEL_COUNT])
{
    uint32_t elapsed_ms = state->steer_commission_ramp_last_ms == 0U ?
                          ECU_STEER_REMOTE_COMMISSION_PERIOD_MS :
                          (uint32_t)(now_ms - state->steer_commission_ramp_last_ms);
    int64_t max_step = ((int64_t)ECU_STEER_REMOTE_COMMISSION_RAMP_COUNTS_PER_SEC *
                        (int64_t)elapsed_ms) / 1000;
    if (max_step < 1) {
        max_step = 1;
    }

    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        uint8_t axis_bit = (uint8_t)(1U << wheel);
        if ((axis_mask & axis_bit) == 0U) {
            out_targets[wheel] = 0;
            continue;
        }

        int32_t current = state->steer_commission_ramped_target_valid[wheel] ?
                          state->steer_commission_ramped_target_counts[wheel] :
                          calibration[wheel].straight_zero_offset_counts;
        int64_t delta = (int64_t)desired_targets[wheel] - (int64_t)current;
        int32_t next = desired_targets[wheel];
        if (delta > max_step) {
            next = current + (int32_t)max_step;
        } else if (delta < -max_step) {
            next = current - (int32_t)max_step;
        }
        out_targets[wheel] = clamp_i32(next,
                                       calibration[wheel].minimum_position_counts,
                                       calibration[wheel].maximum_position_counts);
        state->steer_commission_ramped_target_counts[wheel] = out_targets[wheel];
        state->steer_commission_ramped_target_valid[wheel] = true;
    }

    state->steer_commission_ramp_last_ms = now_ms;
}

static bool start_post_command_tpdo_window(motion_device_state_t *state,
                                           canopen_master_service_t *canopen,
                                           const ecu_hardware_config_t *config,
                                           uint8_t axis_mask,
                                           bool is_centering,
                                           uint32_t now_ms)
{
    uint8_t selected_mask = axis_mask & ECU_STEER_REMOTE_COMMISSION_AXIS_MASK_ALL;
    if (selected_mask == 0U) {
        return false;
    }

    state->steer_commission_post_command_tpdo_pending = true;
    state->steer_commission_post_command_axis_mask = selected_mask;
    state->steer_commission_post_command_missing_mask = selected_mask;
    state->steer_commission_post_command_start_ms = 0U;
    state->steer_commission_post_command_is_centering = is_centering;

    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        state->steer_commission_tpdo0_count_before[wheel] = 0U;
        state->steer_commission_tpdo1_count_before[wheel] = 0U;
        if ((selected_mask & (uint8_t)(1U << wheel)) == 0U) {
            continue;
        }

        canopen_node_feedback_t feedback;
        uint8_t node_id = config->steer_nodes[wheel].node_id;
        if (canopen_master_service_get_node_feedback(canopen, node_id, &feedback)) {
            state->steer_commission_tpdo0_count_before[wheel] = feedback.tpdo0_rx_count;
            state->steer_commission_tpdo1_count_before[wheel] = feedback.tpdo1_rx_count;
        }
    }

    canopen_master_snapshot_t snapshot;
    memset(&snapshot, 0, sizeof(snapshot));
    canopen_master_service_get_snapshot(canopen, &snapshot);
    state->steer_commission_sync_complete_count_before =
        snapshot.sync_tx_complete_count;
    state->steer_commission_sync_wait_start_ms = now_ms;
    state->steer_commission_sync_complete_ms = 0U;

    if (canopen_master_service_send_sync(canopen, now_ms)) {
        state->steer_commission_last_sync_ms = now_ms;
        state->steer_commission_state = STEER_REMOTE_COMMISSION_WAIT_SYNC_TX_COMPLETE;
        return true;
    }
    state->steer_commission_post_command_tpdo_pending = false;
    state->steer_commission_post_command_axis_mask = 0U;
    state->steer_commission_post_command_missing_mask = 0U;
    state->steer_commission_post_command_is_centering = false;
    state->steer_commission_state = STEER_REMOTE_COMMISSION_FAULT;
    return false;
}

static bool post_command_sync_complete(motion_device_state_t *state,
                                       const canopen_master_service_t *canopen,
                                       uint32_t now_ms)
{
    canopen_master_snapshot_t snapshot;
    memset(&snapshot, 0, sizeof(snapshot));
    canopen_master_service_get_snapshot(canopen, &snapshot);
    if (snapshot.sync_tx_complete_count >
            state->steer_commission_sync_complete_count_before &&
        snapshot.last_sync_tx_complete_ms >=
            state->steer_commission_sync_wait_start_ms) {
        state->steer_commission_sync_complete_ms =
            snapshot.last_sync_tx_complete_ms;
        state->steer_commission_post_command_start_ms = now_ms;
        state->steer_commission_state =
            STEER_REMOTE_COMMISSION_WAIT_POST_COMMAND_TPDO;
        return true;
    }
    if ((uint32_t)(now_ms - state->steer_commission_sync_wait_start_ms) >=
        ECU_STEER_REMOTE_COMMISSION_SYNC_TIMEOUT_MS) {
        state->steer_commission_post_command_timeout_count++;
        state->steer_commission_last_post_command_timeout_ms = now_ms;
        state->steer_commission_state = STEER_REMOTE_COMMISSION_FAULT;
    }
    return false;
}

static bool post_command_tpdo_window_complete(motion_device_state_t *state,
                                              const canopen_master_service_t *canopen,
                                              const ecu_hardware_config_t *config)
{
    if (!state->steer_commission_post_command_tpdo_pending) {
        return true;
    }

    uint8_t missing_mask = 0U;
    uint8_t axis_mask = state->steer_commission_post_command_axis_mask &
                        ECU_STEER_REMOTE_COMMISSION_AXIS_MASK_ALL;
    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        uint8_t axis_bit = (uint8_t)(1U << wheel);
        if ((axis_mask & axis_bit) == 0U) {
            continue;
        }

        canopen_node_feedback_t feedback;
        uint8_t node_id = config->steer_nodes[wheel].node_id;
        if (!canopen_master_service_get_node_feedback(canopen, node_id, &feedback) ||
            feedback.tpdo0_rx_count <= state->steer_commission_tpdo0_count_before[wheel] ||
            !feedback.feedback_fresh ||
            feedback.fault_latched != 0U) {
            missing_mask |= axis_bit;
        }
    }

    state->steer_commission_post_command_missing_mask = missing_mask;
    if (missing_mask != 0U) {
        return false;
    }

    state->steer_commission_post_command_tpdo_pending = false;
    state->steer_commission_post_command_axis_mask = 0U;
    state->steer_commission_post_command_missing_mask = 0U;
    state->steer_commission_post_command_start_ms = 0U;
    state->steer_commission_state =
        state->steer_commission_post_command_is_centering ?
        STEER_REMOTE_COMMISSION_WAIT_CENTER_SETTLE :
        STEER_REMOTE_COMMISSION_ACTIVE;
    state->steer_commission_post_command_is_centering = false;
    return true;
}

static ecu_device_apply_result_t flush_steer4_remote_commissioning(
    motion_device_state_t *state,
    canopen_master_service_t *canopen,
    const ecu_hardware_config_t *config,
    uint32_t now_ms)
{
    if (!state->last_motion_command_valid) {
        state->steer_commission_state = STEER_REMOTE_COMMISSION_WAIT_AUTH;
        return ECU_DEVICE_APPLY_OK;
    }

    update_steer_remote_commissioning_state(state,
                                            canopen,
                                            config,
                                            &state->last_motion_command,
                                            now_ms);
    if (state->steer_commission_state == STEER_REMOTE_COMMISSION_TPDO_MONITOR) {
        send_commissioning_sync_if_due(canopen, state, now_ms);
    }

    if (state->steer_group_active) {
        if (canopen_master_service_pdo_group_failed(canopen, state->steer_active_group_sequence)) {
            bool latched = fail_active_steer_group(state, canopen, now_ms);
            if (latched) {
                clear_steer_commissioning_authorization(state);
                state->steer_commission_state = STEER_REMOTE_COMMISSION_FAULT;
                return ECU_DEVICE_APPLY_REJECTED;
            }
            return ECU_DEVICE_APPLY_OK;
        }
        if (canopen_master_service_pdo_group_cancelled(canopen,
                                                       state->steer_active_group_sequence)) {
            clean_cancel_active_steer_group(state, now_ms);
            return ECU_DEVICE_APPLY_OK;
        }
        if (canopen_master_service_pdo_group_pending(canopen, state->steer_active_group_sequence)) {
            return ECU_DEVICE_APPLY_OK;
        }
        uint8_t completed_axis_mask = state->steer_active_group_axis_mask;
        bool was_centering =
            state->steer_commission_state == STEER_REMOTE_COMMISSION_CENTERING;
        if (!start_post_command_tpdo_window(state,
                                            canopen,
                                            config,
                                            completed_axis_mask,
                                            was_centering,
                                            now_ms)) {
            (void)fail_active_steer_group(state, canopen, now_ms);
            clear_steer_commissioning_authorization(state);
            state->steer_commission_state = STEER_REMOTE_COMMISSION_FAULT;
            return ECU_DEVICE_APPLY_REJECTED;
        }
        finish_completed_steer_group(state, now_ms);
    }

    if (state->steer_commission_state ==
        STEER_REMOTE_COMMISSION_WAIT_SYNC_TX_COMPLETE) {
        if (!post_command_sync_complete(state, canopen, now_ms)) {
            if (state->steer_commission_state == STEER_REMOTE_COMMISSION_FAULT) {
                clear_steer_commissioning_authorization(state);
                return ECU_DEVICE_APPLY_REJECTED;
            }
            return ECU_DEVICE_APPLY_OK;
        }
    }

    if (state->steer_commission_post_command_tpdo_pending) {
        if (post_command_tpdo_window_complete(state, canopen, config)) {
            return ECU_DEVICE_APPLY_OK;
        }
        if ((uint32_t)(now_ms - state->steer_commission_post_command_start_ms) >=
            ECU_STEER_REMOTE_COMMISSION_POST_COMMAND_TPDO_TIMEOUT_MS) {
            state->steer_commission_post_command_timeout_count++;
            state->steer_commission_last_post_command_timeout_ms = now_ms;
            clear_steer_commissioning_authorization(state);
            state->steer_commission_state = STEER_REMOTE_COMMISSION_FAULT;
            return ECU_DEVICE_APPLY_REJECTED;
        }
        return ECU_DEVICE_APPLY_OK;
    }

    if (state->steer_commission_state == STEER_REMOTE_COMMISSION_AXIS_READY) {
        state->steer_commission_state = STEER_REMOTE_COMMISSION_ACTIVE;
    }
    if (state->steer_commission_state == STEER_REMOTE_COMMISSION_WAIT_CENTER_SETTLE) {
        uint8_t axis_mask = state->selected_axis_mask &
                            ECU_STEER_REMOTE_COMMISSION_AXIS_MASK_ALL;
        steer_axis_calibration_t calibration[ECU_WHEEL_COUNT];
        if (!motion_device_get_effective_steer_calibration(config,
                                                           axis_mask,
                                                           calibration,
                                                           NULL)) {
            clear_steer_commissioning_authorization(state);
            state->steer_commission_state = STEER_REMOTE_COMMISSION_FAULT;
            return ECU_DEVICE_APPLY_REJECTED;
        }
        if (center_feedback_within_tolerance(state,
                                             canopen,
                                             config,
                                             calibration,
                                             axis_mask)) {
            state->steer_commission_centered = true;
            initialize_commissioning_ramp_from_feedback(state,
                                                        canopen,
                                                        config,
                                                        calibration,
                                                        axis_mask,
                                                        now_ms);
            state->steer_commission_state = STEER_REMOTE_COMMISSION_ACTIVE;
            return ECU_DEVICE_APPLY_OK;
        }
        send_commissioning_sync_if_due(canopen, state, now_ms);
        if ((uint32_t)(now_ms - state->steer_commission_sync_complete_ms) >=
            ECU_STEER_REMOTE_COMMISSION_POST_COMMAND_TPDO_TIMEOUT_MS) {
            state->steer_commission_post_command_timeout_count++;
            state->steer_commission_last_post_command_timeout_ms = now_ms;
            clear_steer_commissioning_authorization(state);
            state->steer_commission_state = STEER_REMOTE_COMMISSION_FAULT;
            return ECU_DEVICE_APPLY_REJECTED;
        }
        return ECU_DEVICE_APPLY_OK;
    }

    if (state->steer_commission_state == STEER_REMOTE_COMMISSION_CENTERING) {
        uint8_t axis_mask = state->selected_axis_mask &
                            ECU_STEER_REMOTE_COMMISSION_AXIS_MASK_ALL;
        steer_axis_calibration_t calibration[ECU_WHEEL_COUNT];
        int32_t center_targets[ECU_WHEEL_COUNT] = {0};
        if (!motion_device_get_effective_steer_calibration(config,
                                                           axis_mask,
                                                           calibration,
                                                           NULL) ||
            !steer_commissioning_build_targets(calibration,
                                               axis_mask,
                                               0.0f,
                                               center_targets)) {
            clear_steer_commissioning_authorization(state);
            state->steer_commission_state = STEER_REMOTE_COMMISSION_FAULT;
            return ECU_DEVICE_APPLY_REJECTED;
        }
        if ((uint32_t)(now_ms - state->steer_realtime_last_flush_ms) <
            ECU_STEER_REMOTE_COMMISSION_PERIOD_MS) {
            return ECU_DEVICE_APPLY_OK;
        }
        state->steer_realtime_last_flush_ms = now_ms;
        if (!queue_steer4_remote_group(canopen,
                                       config,
                                       state,
                                       center_targets,
                                       axis_mask,
                                       now_ms)) {
            return ECU_DEVICE_APPLY_REJECTED;
        }
        return ECU_DEVICE_APPLY_OK;
    }

    if (state->steer_commission_state != STEER_REMOTE_COMMISSION_ACTIVE) {
        return ECU_DEVICE_APPLY_OK;
    }
    if ((uint32_t)(now_ms - state->steer_realtime_last_flush_ms) <
        ECU_STEER_REMOTE_COMMISSION_PERIOD_MS) {
        return ECU_DEVICE_APPLY_OK;
    }
    state->steer_realtime_last_flush_ms = now_ms;

    uint8_t axis_mask = state->selected_axis_mask &
                        ECU_STEER_REMOTE_COMMISSION_AXIS_MASK_ALL;
    steer_axis_calibration_t calibration[ECU_WHEEL_COUNT];
    int32_t desired_targets[ECU_WHEEL_COUNT] = {0};
    int32_t ramped_targets[ECU_WHEEL_COUNT] = {0};
    if (!motion_device_get_effective_steer_calibration(config,
                                                       axis_mask,
                                                       calibration,
                                                       NULL) ||
        !steer_commissioning_build_targets(calibration,
                                           axis_mask,
                                           state->last_motion_command.target_steer_deg[0],
                                           desired_targets)) {
        clear_steer_commissioning_authorization(state);
        state->steer_commission_state = STEER_REMOTE_COMMISSION_FAULT;
        return ECU_DEVICE_APPLY_REJECTED;
    }
    ramp_commissioning_targets(state,
                               calibration,
                               desired_targets,
                               axis_mask,
                               now_ms,
                               ramped_targets);
    if (!selected_targets_changed(state, ramped_targets, axis_mask)) {
        canopen_master_service_note_pdo_same_target_coalesced(canopen);
        return ECU_DEVICE_APPLY_OK;
    }
    if (!queue_steer4_remote_group(canopen, config, state, ramped_targets, axis_mask, now_ms)) {
        return ECU_DEVICE_APPLY_REJECTED;
    }
    return ECU_DEVICE_APPLY_OK;
}
#endif

void motion_device_init(motion_device_state_t *state)
{
    if (state != 0) {
        memset(state, 0, sizeof(*state));
        steer_profile_setup_reset(state);
        state->last_result = ECU_DEVICE_APPLY_OK;
        state->steer_inhibit_reason = MOTION_STEER_INHIBIT_BENCH_MODE_DISABLED;
        state->steer_safety_inhibited = true;
        state->steer_commission_state = STEER_REMOTE_COMMISSION_DISABLED;
        state->selected_axis_mask = 0U;
        steering_transition_planner_init(&state->steer_transition_planner);
        for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
            state->steer_axis_config_state[wheel] = MOTION_STEER_AXIS_UNSEEN;
        }
    }
}

/* Apply a per-axis speed envelope to open-loop current assist.  Current mode
 * controls torque, not speed; without this feedback gate an unloaded wheel can
 * accelerate while the other wheels are still overcoming static load. */
static int16_t limit_track_assist_current_by_feedback(
    motion_device_state_t *state,
    const canopen_master_service_t *canopen,
    const ecu_canopen_node_config_t *node,
    uint32_t wheel,
    int16_t requested_current_10ma)
{
    if (state == NULL || canopen == NULL || node == NULL ||
        wheel >= ECU_WHEEL_COUNT) {
        return 0;
    }

    uint8_t bit = (uint8_t)(1U << wheel);
    canopen_node_feedback_t feedback;
    if (!canopen_master_service_get_node_feedback(canopen,
                                                   node->node_id,
                                                   &feedback) ||
        !can2_feedback_operation_enabled(node->node_id, &feedback)) {
        state->track_assist_feedback_invalid_mask |= bit;
        return 0;
    }
    state->track_assist_feedback_invalid_mask &= (uint8_t)~bit;

    int32_t absolute_velocity =
        i32_abs_saturating(feedback.actual_velocity_units);
    if (absolute_velocity >= ECU_TRACK_ASSIST_OVERSPEED_VELOCITY_UNITS) {
        state->track_assist_overspeed_mask |= bit;
    } else if ((state->track_assist_overspeed_mask & bit) != 0U &&
               absolute_velocity <=
                   ECU_TRACK_ASSIST_OVERSPEED_RESUME_VELOCITY_UNITS) {
        state->track_assist_overspeed_mask &= (uint8_t)~bit;
    }

    if ((state->track_assist_overspeed_mask & bit) != 0U) {
        return 0;
    }
    if (absolute_velocity >= ECU_TRACK_ASSIST_SLOWDOWN_VELOCITY_UNITS) {
        return (int16_t)(requested_current_10ma / 2);
    }
    return requested_current_10ma;
}

ecu_device_apply_result_t motion_device_apply(motion_device_state_t *state,
                                              canopen_master_service_t *canopen,
                                              const ecu_hardware_config_t *config,
                                              const vehicle_actuator_command_t *command,
                                              uint32_t command_sequence,
                                              uint32_t now_ms)
{
    if (state == 0 || canopen == 0 || config == 0 || command == 0) {
        return ECU_DEVICE_APPLY_INVALID_ARGUMENT;
    }
    if (!canopen->snapshot.initialized || !canopen->snapshot.can_normal) {
        state->last_result = ECU_DEVICE_APPLY_BACKEND_OFFLINE;
        return state->last_result;
    }

    bool realtime_disable_requested =
        command->high_voltage_disable_request ||
        !command->high_voltage_enable ||
        command->source == COMMAND_SOURCE_SAFETY;
    if (realtime_disable_requested &&
        can2_realtime_motion_state_needs_recovery(state)) {
        reset_can2_realtime_motion_state(state, canopen, now_ms);
    }

    bool changed = command_changed(state, command);
    bool refresh_due = motion_command_refresh_due(state, now_ms);

    /* These safety gates must track the newest coherent vehicle command even
     * when a later device/setup operation in this apply pass is rejected.
     * Keeping an older successful high-voltage permission would let the CAN2
     * task continue creating motion PDO groups after the arbiter has removed
     * high voltage.
     */
    state->last_motion_command.high_voltage_enable =
        command->high_voltage_enable;
    state->last_motion_command.high_voltage_disable_request =
        command->high_voltage_disable_request;
    state->last_motion_command.high_voltage_feedback_ready =
        command->high_voltage_feedback_ready;
    state->last_motion_command.source = command->source;
    state->last_motion_command.diagnostic = command->diagnostic;

    bool steer_allowed = motion_device_update_steer_safety_gate(state,
                                                                canopen,
                                                                config,
                                                                command,
                                                                now_ms);
    state->steer_zero_calibration_domain_active =
        command->steer_zero_calibration_domain_active;
    if (command->steer_zero_calibration_request) {
        /* Steering zero calibration is an explicit maintenance operation.  It
         * must not be mixed with normal remote/Ackermann steering groups or
         * drive velocity groups.  The CAN2-owned calibration state machine is
         * allowed to consume this latched request; until then, hold all normal
         * CAN2 actuator intent at zero so the operator gesture cannot leak into
         * ordinary motion output.
         */
        if (!state->steer_zero_calibration_requested) {
            state->steer_zero_calibration_requested = true;
            state->steer_zero_calibration_request_count++;
            state->steer_zero_calibration_last_request_ms = now_ms;
        }
        steer_allowed = false;
    }

    int32_t steer_position_counts[ECU_WHEEL_COUNT] = {0};
    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        steer_position_counts[wheel] =
            scaled_float_to_i32(command->target_steer_deg[wheel],
                                config->steer_deg_to_counts);
    }

    int32_t steer_actual_position_counts[ECU_WHEEL_COUNT] = {0};
    uint8_t steer_feedback_fresh_mask =
        collect_steering_feedback_for_planner(state,
                                              canopen,
                                              config,
                                              steer_actual_position_counts);
    if (steering_transition_planner_mode_is_fixed_posture(command->motion_mode)) {
        int32_t planned_steer_counts[ECU_WHEEL_COUNT] = {0};
        if (steering_transition_planner_update(&state->steer_transition_planner,
                                               command->motion_mode,
                                               now_ms,
                                               steer_feedback_fresh_mask,
                                               steer_actual_position_counts,
                                               steer_position_counts,
                                               planned_steer_counts)) {
            memcpy(steer_position_counts,
                   planned_steer_counts,
                   sizeof(steer_position_counts));
        } else {
            /* A fixed-posture transition without fresh TPDO feedback can
             * create a large unexpected jump.  Hold the best known safe target
             * and let the existing presteer/safety path block drive motion.
             */
            state->steer_inhibit_reason = MOTION_STEER_INHIBIT_AXIS_NOT_READY;
            state->steer_safety_inhibited = true;
            state->steer_safe_stop_pending = true;
            steer_allowed = false;
            for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
                if ((steer_feedback_fresh_mask & (uint8_t)(1U << wheel)) != 0U ||
                    state->steer_last_position_valid[wheel]) {
                    steer_position_counts[wheel] =
                        steer_actual_position_counts[wheel];
                } else if (state->steer_last_commanded_position_valid[wheel]) {
                    steer_position_counts[wheel] =
                        state->steer_last_commanded_position_counts[wheel];
                } else {
                    steer_position_counts[wheel] = 0;
                }
            }
        }
    } else {
        steering_transition_planner_reset(&state->steer_transition_planner);
    }

    bool can2_recovery_allows_drive =
        state->can2_node_recovery_pending_mask == 0U &&
        !state->can2_partial_group_recovery_active &&
        !state->can2_recovery_steer_sync_pending;
    bool drive_allowed_by_safety =
        drive_output_allowed(command) && steer_allowed &&
        can2_recovery_allows_drive;
    bool drive_allowed =
        drive_allowed_by_safety &&
        presteer_gate_allows_drive(state,
                                   canopen,
                                   config,
                                   command,
                                   steer_position_counts,
                                    drive_allowed_by_safety,
                                    now_ms);
    bool drive_enable_requested =
        can2_zero_speed_operation_enable_permitted(command);
    bool track_assist_current_allowed =
        command->track_assist_requested &&
        command->track_assist_active &&
        command->high_voltage_feedback_ready &&
        !command->high_voltage_disable_request &&
        steer_allowed &&
        can2_recovery_allows_drive &&
        state->track_assist_steer_approximately_ready;
    if (!track_assist_current_allowed) {
        state->track_assist_overspeed_mask = 0U;
        state->track_assist_feedback_invalid_mask = 0U;
    }
    bool ok = true;
    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        int32_t velocity_units = 0;
        if (drive_allowed) {
            int8_t drive_sign = config->drive_direction_sign[wheel];
            if (!drive_direction_sign_is_valid(drive_sign)) {
                ok = false;
            } else {
                velocity_units = scaled_float_to_i32(
                    command->target_wheel_speed_mps[wheel],
                    config->drive_speed_mps_to_counts_per_sec *
                        (float)drive_sign);
            }
        }

        /* This is only a RAM mailbox update; CAN submission remains owned by
         * motion_device_flush_realtime().  Update it every control pass so a
         * safety inhibit or brake-release transition immediately overwrites a
         * previous nonzero velocity instead of waiting for the 500 ms refresh.
         */
        if (track_assist_current_allowed) {
            int16_t limited_current_10ma =
                limit_track_assist_current_by_feedback(
                    state,
                    canopen,
                    &config->drive_nodes[wheel],
                    wheel,
                    command->track_assist_current_10ma[wheel]);
            ok = cache_latest_drive_current(state,
                                            wheel,
                                            limited_current_10ma,
                                            drive_enable_requested) && ok;
        } else {
            ok = cache_latest_drive_velocity(state,
                                             wheel,
                                             velocity_units,
                                             drive_enable_requested) && ok;
        }
        if (!commissioning_policy_allows_drive_rpdo()) {
            state->drive_last_velocity_valid[wheel] = true;
            state->drive_last_velocity_units[wheel] = 0;
            state->drive_last_current_10ma[wheel] = 0;
            state->drive_last_command_kind[wheel] = MOTION_DRIVE_COMMAND_VELOCITY;
            state->drive_last_enable_requested[wheel] = false;
            state->drive_velocity_mode_ready[wheel] = false;
        }

        ok = cache_latest_steer_target(state,
                                       wheel,
                                       command->steer_zero_calibration_request ? 0 :
                                       steer_position_counts[wheel]) && ok;
        if (!commissioning_policy_allows_steer4_remote() &&
            steer_allowed && (changed || refresh_due)) {
            ok = send_steer_command(canopen,
                                    &config->steer_nodes[wheel],
                                    state,
                                    wheel,
                                    command->target_steer_deg[wheel],
                                    config->steer_deg_to_counts,
                                    now_ms) && ok;
        }
    }

    if (ok) {
        state->last_motion_command = *command;
        state->last_motion_command_valid = true;
        state->last_motion_command_queue_ms = now_ms;
        state->last_motion_command_sequence = command_sequence;
    }
    state->apply_count++;
    state->last_result = ok ? ECU_DEVICE_APPLY_OK : ECU_DEVICE_APPLY_REJECTED;
    return state->last_result;
}

ecu_device_apply_result_t motion_device_flush_realtime(motion_device_state_t *state,
                                                       canopen_master_service_t *canopen,
                                                       const ecu_hardware_config_t *config,
                                                       uint32_t now_ms)
{
    if (state == 0 || canopen == 0 || config == 0) {
        return ECU_DEVICE_APPLY_INVALID_ARGUMENT;
    }
    if (!canopen->snapshot.initialized || !canopen->snapshot.can_normal) {
        return ECU_DEVICE_APPLY_BACKEND_OFFLINE;
    }

#if ECU_CANOPEN_COMMISSIONING_POLICY == ECU_CANOPEN_COMMISSIONING_POLICY_STEER4_REMOTE_COMMISSIONING
    return flush_steer4_remote_commissioning(state, canopen, config, now_ms);
#elif ECU_CANOPEN_COMMISSIONING_POLICY != ECU_CANOPEN_COMMISSIONING_POLICY_PDO_OUTPUT_ENABLED && \
    ECU_CANOPEN_COMMISSIONING_POLICY != ECU_CANOPEN_COMMISSIONING_POLICY_NODE5_STEER_PDO_ONLY
    state->steer_normal_pdo_allowed = false;
    state->steer_safety_inhibited = true;
    state->steer_inhibit_reason = MOTION_STEER_INHIBIT_BENCH_MODE_DISABLED;
    state->steer_next_group_valid = false;
    return ECU_DEVICE_APPLY_OK;
#endif

    observe_can2_node_recovery_state(state, canopen, now_ms);
    if (state->can2_node_recovery_pending_mask == 0U) {
        request_can2_motion_nodes_operational(state, canopen, config, now_ms);
    }
    if (!can2_motion_high_voltage_ready(state)) {
        /* Do not create zero-velocity or steering groups against unpowered
         * drives.  The orderly shutdown path sends explicit zero while high
         * voltage feedback is still valid; after feedback is gone, repeatedly
         * queueing then cancelling zero groups only floods CAN2 and can wedge
         * the HPM nonblocking primary TX lane.
         */
        return ECU_DEVICE_APPLY_OK;
    }
    send_can2_feedback_sync_if_due(state, canopen, now_ms);

    if (state->can2_node_recovery_pending_mask != 0U) {
        /* Stop ordinary PDO production while a node is being repaired.  If a
         * group was in flight when feedback went stale, cancel its complete
         * software group and reset only the ECU CAN transport; never NMT-reset
        * a steering drive because that can destroy its calibrated reference. */
        if (state->can2_node_recovery_entry_mask != 0U) {
            if (state->steer_group_active) {
                /* Preserve partial-trigger classification before cancelling
                 * the group.  A feedback timeout during a trigger phase has
                 * the same split-target hazard as an explicit TX failure. */
                (void)fail_active_steer_group(state, canopen, now_ms);
            } else if (state->drive_group_active) {
                (void)fail_active_drive_group(state, canopen, now_ms);
            } else {
                (void)recover_or_latch_can2_transient_failure(state,
                                                              canopen,
                                                              now_ms,
                                                              false);
            }
            state->can2_node_recovery_entry_mask = 0U;
        }
        force_can2_drive_recovery_zero_intent(state);
        bool recovery_enable_requested =
            can2_recovery_zero_enable_permitted(state);
        ecu_device_apply_result_t stop_result =
            flush_drive_velocity_realtime(state, canopen, config, now_ms);
        if (stop_result == ECU_DEVICE_APPLY_REJECTED) {
            return stop_result;
        }
        if (!state->drive_group_active &&
            drive_zero_intent_completed(state, recovery_enable_requested) &&
            canopen_master_service_realtime_pdo_idle(canopen)) {
            service_can2_node_recovery(state, canopen, now_ms);
        }
        return ECU_DEVICE_APPLY_OK;
    }

    if (state->can2_partial_group_recovery_active) {
        force_can2_drive_recovery_zero_intent(state);
        ecu_device_apply_result_t stop_result =
            flush_drive_velocity_realtime(state, canopen, config, now_ms);
        if (stop_result == ECU_DEVICE_APPLY_REJECTED) {
            return stop_result;
        }
        service_can2_partial_group_recovery(state, canopen, now_ms);
        return ECU_DEVICE_APPLY_OK;
    }

    bool recovery_enable_requested =
        can2_recovery_zero_enable_permitted(state);
    if (state->can2_recovery_steer_sync_pending &&
        (state->drive_group_active ||
         !drive_zero_intent_completed(state, recovery_enable_requested))) {
        /* Complete the explicit traction-zero group before using the CAN2
         * realtime lane for the steering resynchronization group. */
        return flush_drive_velocity_realtime(state, canopen, config, now_ms);
    }

    if (state->steer_zero_calibration_requested ||
        state->steer_zero_calibration_state != MOTION_STEER_ZERO_CAL_IDLE) {
        /* Calibration owns CAN2 until it completes, faults, or is explicitly
         * retried.  This prevents mixed ordinary steering/drive PDO groups
         * while axes are intentionally seeking mechanical end stops.
         */
        return steer_zero_calibration_step(state, canopen, config, now_ms);
    }

    bool steer_profile_setup_incomplete =
        state->steer_profile_setup_state != MOTION_STEER_PROFILE_COMPLETE ||
        state->steer_profile_verified_mask !=
            ECU_STEER_REMOTE_COMMISSION_AXIS_MASK_ALL;
    if (steer_profile_setup_incomplete &&
        !state->can2_recovery_steer_sync_pending) {
        /* No nonzero CAN2 motion is allowed until every steering node has
         * acknowledged and read back the same runtime profile.  A pending
         * post-fault steering resynchronization is allowed to finish first;
         * otherwise the recovery group and this setup gate would wait on each
         * other indefinitely.
         *
         * Keep the drive axes Operation Enabled at explicit zero during this
         * setup.  Disabling them here would make their TPDO state leave
         * Operation Enabled, retrigger node recovery, and prevent the setup
         * state machine from ever reaching its SDO work.
         */
        bool setup_zero_enable_requested =
            can2_recovery_zero_enable_permitted(state);
        force_can2_drive_zero_intent(state, setup_zero_enable_requested);
        ecu_device_apply_result_t stop_result =
            flush_drive_velocity_realtime(state, canopen, config, now_ms);
        if (stop_result == ECU_DEVICE_APPLY_REJECTED) {
            return stop_result;
        }
        if (state->drive_group_active ||
            !drive_zero_intent_completed(state, setup_zero_enable_requested) ||
            !canopen_master_service_realtime_pdo_idle(canopen)) {
            return ECU_DEVICE_APPLY_OK;
        }
        (void)steer_profile_setup_step(state, canopen, config, now_ms);
        return ECU_DEVICE_APPLY_OK;
    }

    if (!state->steer_normal_pdo_allowed) {
        if (state->steer_next_group_valid) {
            canopen_master_service_note_pdo_safety_inhibit(canopen);
        }
        state->steer_next_group_valid = false;
        return flush_drive_velocity_realtime(state, canopen, config, now_ms);
    }

    if (state->steer_group_active) {
        if (canopen_master_service_pdo_group_failed(canopen, state->steer_active_group_sequence)) {
            bool latched = fail_active_steer_group(state, canopen, now_ms);
            if (latched) {
                return ECU_DEVICE_APPLY_REJECTED;
            }
            return flush_drive_velocity_realtime(state, canopen, config, now_ms);
        }
        if (canopen_master_service_pdo_group_cancelled(canopen,
                                                       state->steer_active_group_sequence)) {
            clean_cancel_active_steer_group(state, now_ms);
            return ECU_DEVICE_APPLY_OK;
        }
        if (canopen_master_service_pdo_group_pending(canopen, state->steer_active_group_sequence)) {
            if ((uint32_t)(now_ms - state->steer_realtime_last_flush_ms) >=
                ECU_CANOPEN_STEER_PDO_PERIOD_MS &&
                all_steer_axes_realtime_ready(state, canopen, config, now_ms)) {
                int32_t next_targets[ECU_WHEEL_COUNT] = {0};
                uint32_t elapsed_ms = state->steer_realtime_last_flush_ms == 0U ?
                                      ECU_CANOPEN_STEER_PDO_PERIOD_MS :
                                      (uint32_t)(now_ms -
                                                 state->steer_realtime_last_flush_ms);
                state->steer_realtime_last_flush_ms = now_ms;
                if (build_steer_group_targets(state, elapsed_ms, next_targets)) {
                    memcpy(state->steer_next_group_target_counts,
                           next_targets,
                           sizeof(state->steer_next_group_target_counts));
                    state->steer_next_group_valid = state->steer_normal_pdo_allowed;
                } else {
                    canopen_master_service_note_pdo_same_target_coalesced(canopen);
                }
            }
            /* Steering position groups are serialized as arm/SYNC/trigger/SYNC.
             * While one steering group is active, still run the drive builder
             * so the latest velocity or zero-speed intent overwrites stale
             * targets.  flush_drive_velocity_realtime() detects that the CAN2
             * PDO lane is busy with another group and keeps only the newest
             * pending drive snapshot instead of attempting a conflicting queue.
             */
            return flush_drive_velocity_realtime(state, canopen, config, now_ms);
        }
        finish_completed_steer_group(state, now_ms);

        /* A steering update is an ordered arm/SYNC/trigger/SYNC group.  When
         * the remote joystick is moving continuously the next steering target
         * can already be pending as soon as the previous group completes.  If
         * we always start that next steering group first, the drive velocity
         * group may never get an empty CAN2 PDO lane and the wheels appear to
         * start/stop in bursts.  Give the drive path one bounded opportunity
         * after each completed steering group; it either queues one coherent
         * velocity/zero-speed group or just updates its latest pending
         * snapshot.  A newly queued drive group owns the lane until complete,
         * so the next steering group waits instead of interleaving frames.
         */
        ecu_device_apply_result_t drive_result =
            flush_drive_velocity_realtime(state, canopen, config, now_ms);
        if (drive_result == ECU_DEVICE_APPLY_REJECTED) {
            return drive_result;
        }
        if (canopen_pdo_lane_busy_for_other_group(canopen, 0U)) {
            return ECU_DEVICE_APPLY_OK;
        }
    }

    if (state->steer_next_group_valid) {
        bool queued = queue_steer_group(canopen,
                                        config,
                                        state,
                                        state->steer_next_group_target_counts,
                                        now_ms);
        if (queued) {
            state->steer_next_group_valid = false;
            return ECU_DEVICE_APPLY_OK;
        }
        (void)flush_drive_velocity_realtime(state, canopen, config, now_ms);
        return ECU_DEVICE_APPLY_REJECTED;
    }

    if ((uint32_t)(now_ms - state->steer_realtime_last_flush_ms) <
        ECU_CANOPEN_STEER_PDO_PERIOD_MS) {
        return flush_drive_velocity_realtime(state, canopen, config, now_ms);
    }

    uint32_t elapsed_ms = state->steer_realtime_last_flush_ms == 0U ?
                          ECU_CANOPEN_STEER_PDO_PERIOD_MS :
                          (uint32_t)(now_ms - state->steer_realtime_last_flush_ms);
    state->steer_realtime_last_flush_ms = now_ms;

    int32_t target_counts[ECU_WHEEL_COUNT] = {0};
    if (!all_steer_axes_realtime_ready(state, canopen, config, now_ms)) {
        send_can2_feedback_sync_if_due(state, canopen, now_ms);
        return flush_drive_velocity_realtime(state, canopen, config, now_ms);
    }
    if (!build_steer_group_targets(state, elapsed_ms, target_counts)) {
        canopen_master_service_note_pdo_same_target_coalesced(canopen);
        return flush_drive_velocity_realtime(state, canopen, config, now_ms);
    }

    if (!queue_steer_group(canopen, config, state, target_counts, now_ms)) {
        memcpy(state->steer_next_group_target_counts,
               target_counts,
               sizeof(state->steer_next_group_target_counts));
        state->steer_next_group_valid = true;
        (void)flush_drive_velocity_realtime(state, canopen, config, now_ms);
        return ECU_DEVICE_APPLY_REJECTED;
    }

    return ECU_DEVICE_APPLY_OK;
}
