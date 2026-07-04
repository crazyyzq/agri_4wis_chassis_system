#include <stdint.h>
#include <string.h>

#include "motion_device.h"
#include "canopen_pdo_profile.h"
#include "servo_drive_canopen.h"

#define ECU_STEER_GROUP_PDO_FRAME_COUNT (ECU_WHEEL_COUNT * 2U)
#define ECU_NODE5_STEER_PDO_FRAME_COUNT (2U)

bool ecu_commissioning_node5_pdo_runtime_authorized(void)
{
    /* This runtime authorization is intentionally false in production images.
     * A Node5-only commissioning image may replace this hook with a physical
     * safety-confirmed authorization path before any real RPDO is emitted.
     */
    return false;
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

static bool commissioning_policy_allows_drive_rpdo(void)
{
    /* Drive-wheel RPDO output is a later phase.  It must define velocity units,
     * zero-speed watchdog behavior and brake/enable interlocks before this can
     * return true.
     */
    return false;
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
    (void)command;
    if (!commissioning_policy_allows_drive_rpdo()) {
        return false;
    }
#if ECU_COMMISSIONING_STEER_ONLY_MODE
    return false;
#else
    return command->brake_release;
#endif
}

#if ECU_CANOPEN_COMMISSIONING_POLICY == ECU_CANOPEN_COMMISSIONING_POLICY_PDO_OUTPUT_ENABLED
static bool target_update_interval_elapsed(uint32_t last_update_ms,
                                           uint32_t now_ms)
{
    return (uint32_t)(now_ms - last_update_ms) >=
           ECU_CANOPEN_MOTION_TARGET_MIN_INTERVAL_MS;
}

static bool send_drive_target_update(canopen_master_service_t *canopen,
                                     const ecu_canopen_node_config_t *node,
                                     motion_device_state_t *state,
                                     uint32_t wheel,
                                     int32_t velocity_units,
                                     bool immediate_stop,
                                     uint32_t now_ms)
{
    bool target_changed = !state->drive_last_velocity_valid[wheel] ||
                          i32_changed_beyond_deadband(
                              state->drive_last_velocity_units[wheel],
                              velocity_units,
                              ECU_CANOPEN_DRIVE_VELOCITY_DEADBAND_UNITS) ||
                          immediate_stop;
    if (!target_changed) {
        return true;
    }
    if (!immediate_stop &&
        !target_update_interval_elapsed(state->drive_last_target_update_ms[wheel],
                                        now_ms)) {
        return true;
    }

    if (!servo_drive_canopen_update_target_velocity(canopen,
                                                   node,
                                                   velocity_units)) {
        return false;
    }

    state->drive_last_velocity_valid[wheel] = true;
    state->drive_last_velocity_units[wheel] = velocity_units;
    state->drive_last_target_update_ms[wheel] = now_ms;
    state->last_target_update_ms = now_ms;
    return true;
}

static bool send_drive_command(canopen_master_service_t *canopen,
                               const ecu_canopen_node_config_t *node,
                               motion_device_state_t *state,
                               uint32_t wheel,
                               int32_t velocity_units,
                               bool brake_release,
                               uint32_t now_ms)
{
    if (!brake_release) {
        bool ok = true;
        bool needs_stop = !state->drive_last_velocity_valid[wheel] ||
                          state->drive_last_velocity_units[wheel] != 0 ||
                          state->drive_velocity_mode_ready[wheel];
        if (needs_stop) {
            ok = servo_drive_canopen_stop_velocity_mode(canopen, node);
        }
        if (ok) {
            state->drive_velocity_mode_ready[wheel] = false;
            state->drive_last_velocity_valid[wheel] = true;
            state->drive_last_velocity_units[wheel] = 0;
        }
        return ok;
    }

    bool ok = true;

    if (ok && !state->drive_velocity_mode_ready[wheel]) {
        ok = servo_drive_canopen_prepare_velocity_mode(canopen, node);
        if (ok) {
            state->drive_velocity_mode_ready[wheel] = true;
            state->drive_last_velocity_valid[wheel] = false;
        }
    }

    if (ok) {
        ok = send_drive_target_update(canopen,
                                      node,
                                      state,
                                      wheel,
                                      velocity_units,
                                      false,
                                      now_ms);
    }
    return ok;
}
#endif

static bool steer_limit_blocks_target(canopen_master_service_t *canopen,
                                      const ecu_canopen_node_config_t *node,
                                      motion_device_state_t *state,
                                      uint32_t wheel,
                                      float steer_deg,
                                      uint32_t now_ms)
{
#if ECU_COMMISSIONING_STEER_ONLY_MODE
    /* During steering bring-up the limit inputs may be unconfigured, inverted,
     * or not yet wired.  Do not let a diagnostic read of object 0x2190 inject
     * quick-stop commands while verifying that position PDOs move the steering
     * motors.  Production mode keeps the limit protection below.
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
    if (!state->steer_latest_target_valid[wheel] ||
        i32_changed_beyond_deadband(state->steer_latest_target_counts[wheel],
                                    position_counts,
                                    ECU_CANOPEN_STEER_POSITION_TRIGGER_THRESHOLD_COUNTS)) {
        state->steer_latest_target_counts[wheel] = position_counts;
        state->steer_latest_target_valid[wheel] = true;
        state->steer_pending_target[wheel] = true;
    }
    return true;
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
                                                const ecu_hardware_config_t *config)
{
    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        uint8_t node_id = config->steer_nodes[wheel].node_id;
        bool has_evidence =
            state->steer_axis_remote_verified[wheel] ||
            canopen_master_service_has_node_evidence(canopen, node_id);
        if (!has_evidence ||
            state->steer_axis_config_state[wheel] != MOTION_STEER_AXIS_READY) {
            return false;
        }
    }
    return true;
}

static motion_steer_inhibit_reason_t evaluate_steer_inhibit_reason(
    motion_device_state_t *state,
    const canopen_master_service_t *canopen,
    const ecu_hardware_config_t *config,
    const vehicle_actuator_command_t *command)
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
    if (commissioning_policy_allows_can3_rpdo() ||
        commissioning_policy_allows_drive_rpdo()) {
        return MOTION_STEER_INHIBIT_BENCH_MODE_DISABLED;
    }
    if (!commissioning_policy_allows_full_steer_pdo() &&
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
    if (!command_source_allows_motion_output(command->source)) {
        return MOTION_STEER_INHIBIT_COMMAND_SOURCE_NOT_AUTHORIZED;
    }
    if (command->active_gear == ECU_GEAR_REQUEST_P) {
        return MOTION_STEER_INHIBIT_GEAR_PARK;
    }
    if (!command->brake_release) {
        return MOTION_STEER_INHIBIT_REMOTE_DISARMED;
    }
    if (state->steer_group_degraded) {
        return MOTION_STEER_INHIBIT_GROUP_DEGRADED;
    }
#if ECU_CAN2_BENCH_PDO_CAPTURE_MODE
    (void)canopen;
    (void)config;
#else
    if (!commissioning_policy_allows_node5_steer_pdo() &&
        !steer_all_axes_have_remote_evidence(state, canopen, config)) {
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
        evaluate_steer_inhibit_reason(state, canopen, config, command);
    bool allowed = reason == MOTION_STEER_INHIBIT_NONE;
    bool was_allowed = state->steer_normal_pdo_allowed;
    motion_steer_inhibit_reason_t old_reason = state->steer_inhibit_reason;

    state->steer_normal_pdo_allowed = allowed;
    state->steer_safety_inhibited = !allowed;
    state->steer_inhibit_reason = reason;

    if (!allowed) {
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
        state->last_motion_command.target_speed_kph != command->target_speed_kph ||
        state->last_motion_command.brake_release != command->brake_release) {
        return true;
    }

    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        if (state->last_motion_command.target_wheel_speed_kph[wheel] !=
            command->target_wheel_speed_kph[wheel]) {
            return true;
        }
        if (state->last_motion_command.target_steer_deg[wheel] !=
            command->target_steer_deg[wheel]) {
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

static int32_t rate_limit_target_counts(int32_t current,
                                        int32_t requested,
                                        uint32_t elapsed_ms)
{
    int32_t max_delta =
        (int32_t)((ECU_CANOPEN_STEER_TARGET_RATE_LIMIT_COUNTS_PER_SEC *
                   elapsed_ms) / 1000U);
    if (max_delta < ECU_CANOPEN_STEER_POSITION_TRIGGER_THRESHOLD_COUNTS) {
        max_delta = ECU_CANOPEN_STEER_POSITION_TRIGGER_THRESHOLD_COUNTS;
    }

    int32_t delta = requested - current;
    if (delta > max_delta) {
        return current + max_delta;
    }
    if (delta < -max_delta) {
        return current - max_delta;
    }
    return requested;
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
    if (state->steer_axis_config_state[wheel] != MOTION_STEER_AXIS_READY) {
        return false;
    }
    if (!state->steer_axis_remote_verified[wheel]) {
        return false;
    }
    if (!canopen_master_service_has_node_evidence(
            canopen,
            node_id)) {
        return false;
    }
#endif
    if (!state->steer_realtime_enabled[wheel]) {
        state->steer_pending_target[wheel] = true;
    }
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

    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        if (!state->steer_latest_target_valid[wheel]) {
            return false;
        }

        int32_t requested = state->steer_latest_target_counts[wheel];
        int32_t limited = requested;
        if (state->steer_last_commanded_position_valid[wheel]) {
            /* This is command-side smoothing only.  Until TPDO actual-position
             * decoding is wired in, the ECU must not treat the last submitted
             * target as measured wheel position.
             */
            limited = rate_limit_target_counts(state->steer_last_commanded_position_counts[wheel],
                                               requested,
                                               elapsed_ms);
        }
        out_targets[wheel] = limited;

        if (!state->steer_last_position_valid[wheel] ||
            state->steer_pending_target[wheel] ||
            i32_changed_beyond_deadband(state->steer_last_position_counts[wheel],
                                        limited,
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

    if (!canopen_master_service_queue_pdo_batch(
            canopen,
            requests,
            ECU_NODE5_STEER_PDO_FRAME_COUNT)) {
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
    state->steer_group_degraded = false;
    return true;
}

static bool queue_steer_group(canopen_master_service_t *canopen,
                              const ecu_hardware_config_t *config,
                              motion_device_state_t *state,
                              const int32_t targets[ECU_WHEEL_COUNT],
                              uint32_t now_ms)
{
    if (commissioning_policy_allows_node5_steer_pdo()) {
        return queue_node5_steer_group(canopen, config, state, targets, now_ms);
    }

    if (!commissioning_policy_allows_full_steer_pdo()) {
        return false;
    }

    if (canopen_master_service_pdo_queue_available(canopen) < ECU_STEER_GROUP_PDO_FRAME_COUNT) {
        return false;
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
            return false;
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
            return false;
        }
    }

    if (!canopen_master_service_queue_pdo_batch(canopen,
                                                requests,
                                                ECU_STEER_GROUP_PDO_FRAME_COUNT)) {
        for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
            state->steer_pdo_tx_error_count[wheel]++;
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
    state->steer_group_degraded = false;
    return true;
}

static void finish_completed_steer_group(motion_device_state_t *state,
                                         uint32_t now_ms)
{
    uint32_t wheels_to_update = state->steer_active_group_node5_only ?
                                1U : ECU_WHEEL_COUNT;
    for (uint32_t wheel = 0U; wheel < wheels_to_update; ++wheel) {
        int32_t target = state->steer_active_group_target_counts[wheel];
        state->steer_last_position_valid[wheel] = true;
        state->steer_last_position_counts[wheel] = target;
        state->steer_last_commanded_position_valid[wheel] = true;
        state->steer_last_commanded_position_counts[wheel] = target;
        state->steer_last_target_update_ms[wheel] = now_ms;
        state->steer_pending_target[wheel] =
            target != state->steer_latest_target_counts[wheel];
    }

    state->last_target_update_ms = now_ms;
    state->steer_group_complete_count++;
    state->steer_group_active = false;
    state->steer_active_group_node5_only = false;
    state->steer_active_group_sequence = 0U;
}

static void clean_cancel_active_steer_group(motion_device_state_t *state,
                                            uint32_t now_ms)
{
    state->steer_group_clean_cancelled = true;
    state->steer_last_clean_cancel_ms = now_ms;
    state->steer_group_active = false;
    state->steer_active_group_node5_only = false;
    state->steer_active_group_sequence = 0U;
}

static void fail_active_steer_group(motion_device_state_t *state,
                                    const canopen_master_service_t *canopen,
                                    uint32_t now_ms)
{
    canopen_master_snapshot_t snapshot;

    memset(&snapshot, 0, sizeof(snapshot));
    canopen_master_service_get_snapshot(canopen, &snapshot);
    state->steer_group_failure_count++;
    if (snapshot.pdo_trigger_complete_frames > 0U ||
        snapshot.last_pdo_failed_phase ==
            (uint8_t)CANOPEN_MASTER_PDO_PHASE_STEER_TRIGGER ||
        snapshot.last_pdo_failed_phase ==
            (uint8_t)CANOPEN_MASTER_PDO_PHASE_NODE5_POSITION_TRIGGER) {
        state->steer_group_trigger_partial_failure = true;
        state->steer_last_partial_failure_ms = now_ms;
    }
    state->steer_group_degraded = true;
    state->steer_group_active = false;
    state->steer_active_group_node5_only = false;
    state->steer_active_group_sequence = 0U;
}

void motion_device_init(motion_device_state_t *state)
{
    if (state != 0) {
        memset(state, 0, sizeof(*state));
        state->last_result = ECU_DEVICE_APPLY_OK;
        state->steer_inhibit_reason = MOTION_STEER_INHIBIT_BENCH_MODE_DISABLED;
        state->steer_safety_inhibited = true;
        for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
            state->steer_axis_config_state[wheel] = MOTION_STEER_AXIS_UNSEEN;
        }
    }
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

    bool changed = command_changed(state, command);
    bool refresh_due = motion_command_refresh_due(state, now_ms);
    bool steer_allowed = motion_device_update_steer_safety_gate(state,
                                                                canopen,
                                                                config,
                                                                command,
                                                                now_ms);

    bool drive_allowed = drive_output_allowed(command);
    bool ok = true;
    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        int32_t velocity_units = drive_allowed ?
            scaled_float_to_i32(command->target_wheel_speed_kph[wheel],
                                config->drive_speed_kph_to_counts_per_sec) :
            0;

#if ECU_CANOPEN_COMMISSIONING_POLICY == ECU_CANOPEN_COMMISSIONING_POLICY_PDO_OUTPUT_ENABLED
        if (changed || refresh_due) {
            ok = send_drive_command(canopen,
                                    &config->drive_nodes[wheel],
                                    state,
                                    wheel,
                                    velocity_units,
                                    drive_allowed,
                                    now_ms) && ok;
        }
#else
        state->drive_last_velocity_valid[wheel] = true;
        state->drive_last_velocity_units[wheel] = 0;
        state->drive_velocity_mode_ready[wheel] = false;
        (void)velocity_units;
        (void)drive_allowed;
#endif

        int32_t steer_position_counts =
            scaled_float_to_i32(command->target_steer_deg[wheel],
                                config->steer_deg_to_counts);
        ok = cache_latest_steer_target(state, wheel, steer_position_counts) && ok;
        if (steer_allowed && (changed || refresh_due)) {
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

#if ECU_CANOPEN_COMMISSIONING_POLICY != ECU_CANOPEN_COMMISSIONING_POLICY_PDO_OUTPUT_ENABLED && \
    ECU_CANOPEN_COMMISSIONING_POLICY != ECU_CANOPEN_COMMISSIONING_POLICY_NODE5_STEER_PDO_ONLY
    state->steer_normal_pdo_allowed = false;
    state->steer_safety_inhibited = true;
    state->steer_inhibit_reason = MOTION_STEER_INHIBIT_BENCH_MODE_DISABLED;
    state->steer_next_group_valid = false;
    return ECU_DEVICE_APPLY_OK;
#endif

    if (!state->steer_normal_pdo_allowed) {
        if (state->steer_next_group_valid) {
            canopen_master_service_note_pdo_safety_inhibit(canopen);
        }
        state->steer_next_group_valid = false;
        return ECU_DEVICE_APPLY_OK;
    }

    if (state->steer_group_active) {
        if (canopen_master_service_pdo_group_failed(canopen, state->steer_active_group_sequence)) {
            fail_active_steer_group(state, canopen, now_ms);
            return ECU_DEVICE_APPLY_REJECTED;
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
            return ECU_DEVICE_APPLY_OK;
        }
        finish_completed_steer_group(state, now_ms);
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
        return ECU_DEVICE_APPLY_REJECTED;
    }

    if ((uint32_t)(now_ms - state->steer_realtime_last_flush_ms) <
        ECU_CANOPEN_STEER_PDO_PERIOD_MS) {
        return ECU_DEVICE_APPLY_OK;
    }

    uint32_t elapsed_ms = state->steer_realtime_last_flush_ms == 0U ?
                          ECU_CANOPEN_STEER_PDO_PERIOD_MS :
                          (uint32_t)(now_ms - state->steer_realtime_last_flush_ms);
    state->steer_realtime_last_flush_ms = now_ms;

    int32_t target_counts[ECU_WHEEL_COUNT] = {0};
    if (!all_steer_axes_realtime_ready(state, canopen, config, now_ms)) {
        return ECU_DEVICE_APPLY_OK;
    }
    if (!build_steer_group_targets(state, elapsed_ms, target_counts)) {
        canopen_master_service_note_pdo_same_target_coalesced(canopen);
        return ECU_DEVICE_APPLY_OK;
    }

    if (!queue_steer_group(canopen, config, state, target_counts, now_ms)) {
        memcpy(state->steer_next_group_target_counts,
               target_counts,
               sizeof(state->steer_next_group_target_counts));
        state->steer_next_group_valid = true;
        return ECU_DEVICE_APPLY_REJECTED;
    }

    return ECU_DEVICE_APPLY_OK;
}
