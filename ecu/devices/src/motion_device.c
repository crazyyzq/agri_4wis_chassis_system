#include <stdint.h>
#include <string.h>

#include "motion_device.h"
#include "servo_drive_canopen.h"

#define ECU_STEER_GROUP_PDO_FRAME_COUNT (ECU_WHEEL_COUNT * 2U)

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

static bool target_update_interval_elapsed(uint32_t last_update_ms,
                                           uint32_t now_ms)
{
    return (uint32_t)(now_ms - last_update_ms) >=
           ECU_CANOPEN_MOTION_TARGET_MIN_INTERVAL_MS;
}

static uint16_t drive_brake_output_value(bool brake_release)
{
    bool active_bit = brake_release ?
                      (ECU_SERVO_BRAKE_RELEASE_CANOPEN_ACTIVE_BIT != 0U) :
                      (ECU_SERVO_BRAKE_RELEASE_CANOPEN_ACTIVE_BIT == 0U);
    return active_bit ? SERVO_DRIVE_OUTPUT_OUT1_MASK : 0U;
}

static bool drive_output_allowed(const vehicle_actuator_command_t *command)
{
#if ECU_COMMISSIONING_STEER_ONLY_MODE
    (void)command;
    return false;
#else
    return command->brake_release;
#endif
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
                          state->drive_velocity_mode_ready[wheel] ||
                          state->drive_brake_release_active[wheel];
        if (needs_stop) {
            ok = servo_drive_canopen_stop_velocity_mode(canopen, node);
        }
        if (state->drive_brake_release_active[wheel] || needs_stop) {
            ok = servo_drive_canopen_set_output_state(
                     canopen,
                     node,
                     SERVO_DRIVE_OUTPUT_OUT1_MASK,
                     drive_brake_output_value(false)) && ok;
        }
        if (ok) {
            state->drive_velocity_mode_ready[wheel] = false;
            state->drive_brake_release_active[wheel] = false;
            state->drive_last_velocity_valid[wheel] = true;
            state->drive_last_velocity_units[wheel] = 0;
        }
        return ok;
    }

    bool ok = true;
    if (!state->drive_brake_release_active[wheel]) {
        ok = servo_drive_canopen_set_output_state(
                 canopen,
                 node,
                 SERVO_DRIVE_OUTPUT_OUT1_MASK,
                 drive_brake_output_value(true));
        if (ok) {
            state->drive_brake_release_active[wheel] = true;
        }
    }

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
    bool ok = true;

    if (!state->steer_pdo_configured[wheel]) {
        ok = servo_drive_canopen_configure_steer_rpdo(canopen, node);
        if (ok) {
            state->steer_pdo_configured[wheel] = true;
            state->steer_setup_queued_ms[wheel] = 0U;
        }
    }

    if (ok && !state->steer_position_mode_ready[wheel]) {
        ok = servo_drive_canopen_prepare_position_mode(canopen,
                                                      node,
                                                      ECU_STEER_POSITION_SPEED_UNITS);
        if (ok) {
            state->steer_position_mode_ready[wheel] = true;
            state->steer_last_position_valid[wheel] = false;
            state->steer_realtime_position_valid[wheel] = false;
            state->steer_realtime_enabled[wheel] = false;
            state->steer_setup_queued_ms[wheel] = 0U;
        }
    }

    return ok;
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
           source == COMMAND_SOURCE_CPU1 ||
           source == COMMAND_SOURCE_SAFETY;
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

static void write_le_u16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8U) & 0xFFU);
}

static void write_le_i32(uint8_t *data, int32_t value)
{
    uint32_t raw = (uint32_t)value;
    data[0] = (uint8_t)(raw & 0xFFU);
    data[1] = (uint8_t)((raw >> 8U) & 0xFFU);
    data[2] = (uint8_t)((raw >> 16U) & 0xFFU);
    data[3] = (uint8_t)((raw >> 24U) & 0xFFU);
}

static void build_steer_rpdo_request(canopen_master_pdo_request_t *request,
                                     const ecu_canopen_node_config_t *node,
                                     uint16_t control_word,
                                     int32_t target_position_counts,
                                     uint32_t group_sequence,
                                     canopen_master_pdo_phase_t phase)
{
    memset(request, 0, sizeof(*request));
    request->cob_id = (uint16_t)node->rpdo1_cob_id;
    request->size = 6U;
    request->node_id = node->node_id;
    request->group_sequence = group_sequence;
    request->phase = phase;
    write_le_u16(&request->data[0], control_word);
    write_le_i32(&request->data[2], target_position_counts);
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
                                      uint32_t wheel,
                                      uint32_t now_ms)
{
    if (!state->steer_pdo_configured[wheel] ||
        !state->steer_position_mode_ready[wheel]) {
        return false;
    }
    if (state->steer_realtime_enabled[wheel]) {
        return true;
    }

    if (state->steer_setup_queued_ms[wheel] == 0U) {
        state->steer_setup_queued_ms[wheel] = now_ms;
        return false;
    }

    bool setup_queue_drained =
        canopen->command_queue_count == 0U && !canopen->sdo_download_active;
    bool settle_elapsed =
        (uint32_t)(now_ms - state->steer_setup_queued_ms[wheel]) >=
        ECU_CANOPEN_STEER_SETUP_SETTLE_MS;
    if (setup_queue_drained && settle_elapsed) {
        state->steer_realtime_enabled[wheel] = true;
        state->steer_pending_target[wheel] = true;
        return true;
    }
    return false;
}

static bool all_steer_axes_realtime_ready(motion_device_state_t *state,
                                          const canopen_master_service_t *canopen,
                                          uint32_t now_ms)
{
    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        if (!steer_axis_realtime_ready(state, canopen, wheel, now_ms)) {
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
        if (state->steer_realtime_position_valid[wheel]) {
            limited = rate_limit_target_counts(state->steer_realtime_position_counts[wheel],
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

static bool queue_steer_group(canopen_master_service_t *canopen,
                              const ecu_hardware_config_t *config,
                              motion_device_state_t *state,
                              const int32_t targets[ECU_WHEEL_COUNT],
                              uint32_t now_ms)
{
    if (canopen_master_service_pdo_queue_available(canopen) < ECU_STEER_GROUP_PDO_FRAME_COUNT) {
        return false;
    }

    canopen_master_pdo_request_t requests[ECU_STEER_GROUP_PDO_FRAME_COUNT];
    uint32_t group_sequence = next_steer_group_sequence(state);
    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        const ecu_canopen_node_config_t *node = &config->steer_nodes[wheel];
        build_steer_rpdo_request(&requests[wheel],
                                 node,
                                 SERVO_DRIVE_CONTROL_ABSOLUTE_UPDATE_ARM,
                                 targets[wheel],
                                 group_sequence,
                                 CANOPEN_MASTER_PDO_PHASE_STEER_ARM);
    }

    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        const ecu_canopen_node_config_t *node = &config->steer_nodes[wheel];
        build_steer_rpdo_request(&requests[ECU_WHEEL_COUNT + wheel],
                                 node,
                                 SERVO_DRIVE_CONTROL_ABSOLUTE_UPDATE_TRIGGER,
                                 targets[wheel],
                                 group_sequence,
                                 CANOPEN_MASTER_PDO_PHASE_STEER_TRIGGER);
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
    state->steer_group_degraded = false;
    return true;
}

static void finish_completed_steer_group(motion_device_state_t *state,
                                         uint32_t now_ms)
{
    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        int32_t target = state->steer_active_group_target_counts[wheel];
        state->steer_last_position_valid[wheel] = true;
        state->steer_last_position_counts[wheel] = target;
        state->steer_realtime_position_valid[wheel] = true;
        state->steer_realtime_position_counts[wheel] = target;
        state->steer_last_target_update_ms[wheel] = now_ms;
        state->steer_pending_target[wheel] =
            target != state->steer_latest_target_counts[wheel];
    }

    state->last_target_update_ms = now_ms;
    state->steer_group_complete_count++;
    state->steer_group_active = false;
    state->steer_active_group_sequence = 0U;
}

static void fail_active_steer_group(motion_device_state_t *state)
{
    state->steer_group_failure_count++;
    state->steer_group_degraded = true;
    state->steer_group_active = false;
    state->steer_active_group_sequence = 0U;
}

void motion_device_init(motion_device_state_t *state)
{
    if (state != 0) {
        memset(state, 0, sizeof(*state));
        state->last_result = ECU_DEVICE_APPLY_OK;
    }
}

ecu_device_apply_result_t motion_device_apply(motion_device_state_t *state,
                                              canopen_master_service_t *canopen,
                                              const ecu_hardware_config_t *config,
                                              const vehicle_actuator_command_t *command,
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
    if (!command_source_allows_motion_output(command->source)) {
        state->skipped_count++;
        state->last_result = ECU_DEVICE_APPLY_OK;
        return state->last_result;
    }

    bool drive_allowed = drive_output_allowed(command);
    bool ok = true;
    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        int32_t velocity_units = drive_allowed ?
            scaled_float_to_i32(command->target_wheel_speed_kph[wheel],
                                config->drive_speed_kph_to_counts_per_sec) :
            0;

        if (changed || refresh_due) {
            ok = send_drive_command(canopen,
                                    &config->drive_nodes[wheel],
                                    state,
                                    wheel,
                                    velocity_units,
                                    drive_allowed,
                                    now_ms) && ok;
        }

        ok = send_steer_command(canopen,
                                &config->steer_nodes[wheel],
                                state,
                                wheel,
                                command->target_steer_deg[wheel],
                                config->steer_deg_to_counts,
                                now_ms) && ok;
    }

    if (ok) {
        state->last_motion_command = *command;
        state->last_motion_command_valid = true;
        state->last_motion_command_queue_ms = now_ms;
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

    if (state->steer_group_active) {
        if (canopen_master_service_pdo_group_failed(canopen, state->steer_active_group_sequence)) {
            fail_active_steer_group(state);
            return ECU_DEVICE_APPLY_REJECTED;
        }
        if (canopen_master_service_pdo_group_pending(canopen, state->steer_active_group_sequence)) {
            if ((uint32_t)(now_ms - state->steer_realtime_last_flush_ms) >=
                ECU_CANOPEN_STEER_PDO_PERIOD_MS &&
                all_steer_axes_realtime_ready(state, canopen, now_ms)) {
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
                    state->steer_next_group_valid = true;
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
    if (!all_steer_axes_realtime_ready(state, canopen, now_ms)) {
        return ECU_DEVICE_APPLY_OK;
    }
    if (!build_steer_group_targets(state, elapsed_ms, target_counts)) {
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
