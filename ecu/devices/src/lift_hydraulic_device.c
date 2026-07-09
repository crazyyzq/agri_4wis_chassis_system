#include <string.h>
#include <stdint.h>

#include "canopen_pdo_profile.h"
#include "lift_hydraulic_device.h"

#define SERVO_DRIVE_CONTROL_DISABLE_VOLTAGE ((uint16_t)0x0000U)
#define SERVO_DRIVE_CONTROL_ENABLE_OPERATION ((uint16_t)0x000FU)

typedef enum {
    LIFT_INTERP_DIRECTION_HOLD = 0,
    LIFT_INTERP_DIRECTION_EXTEND = -1,
    LIFT_INTERP_DIRECTION_RETRACT = 1
} lift_interpolation_direction_t;

static int32_t clamp_i32(int32_t value, int32_t low, int32_t high)
{
    if (value < low) {
        return low;
    }
    if (value > high) {
        return high;
    }
    return value;
}

static int32_t i32_abs(int32_t value)
{
    return value < 0 ? -value : value;
}

static bool time_reached(uint32_t now_ms, uint32_t deadline_ms)
{
    return (int32_t)(now_ms - deadline_ms) >= 0;
}

static int32_t lift_position_lower_limit(void)
{
    return ECU_LIFT_LONGEST_POSITION_COUNTS < ECU_LIFT_SHORTEST_POSITION_COUNTS ?
           ECU_LIFT_LONGEST_POSITION_COUNTS : ECU_LIFT_SHORTEST_POSITION_COUNTS;
}

static int32_t lift_position_upper_limit(void)
{
    return ECU_LIFT_LONGEST_POSITION_COUNTS > ECU_LIFT_SHORTEST_POSITION_COUNTS ?
           ECU_LIFT_LONGEST_POSITION_COUNTS : ECU_LIFT_SHORTEST_POSITION_COUNTS;
}

static int32_t clamp_lift_position_counts(int32_t position_counts)
{
    return clamp_i32(position_counts,
                     lift_position_lower_limit(),
                     lift_position_upper_limit());
}

static int32_t lift_interpolation_step_counts(void)
{
    const int32_t step =
        (int32_t)((ECU_LIFT_INTERPOLATION_SPEED_COUNTS_PER_SEC *
                   ECU_CANOPEN_LIFT_INTERPOLATION_PERIOD_MS) / 1000);
    return step > 0 ? step : 1;
}

static uint32_t next_lift_group_sequence(lift_hydraulic_device_state_t *state)
{
    state->lift_interpolation_group_sequence += 2U;
    if (state->lift_interpolation_group_sequence == 0U ||
        (state->lift_interpolation_group_sequence & 1U) != 0U) {
        state->lift_interpolation_group_sequence = 2U;
    }
    return state->lift_interpolation_group_sequence;
}

static uint32_t next_pump_group_sequence(lift_hydraulic_device_state_t *state)
{
    state->pump_group_sequence += 2U;
    if (state->pump_group_sequence == 0U ||
        (state->pump_group_sequence & 1U) == 0U) {
        state->pump_group_sequence = 1U;
    }
    return state->pump_group_sequence;
}

static uint32_t clear_conflicting_valve_pair(uint32_t valve_mask,
                                             uint32_t pair_mask,
                                             uint32_t *interlocked_mask)
{
    if (pair_mask == 0U) {
        return valve_mask;
    }
    if ((valve_mask & pair_mask) == pair_mask) {
        if (interlocked_mask != 0) {
            *interlocked_mask |= pair_mask;
        }
        valve_mask &= ~pair_mask;
    }
    return valve_mask;
}

/* Enforce relay-box v1.5 hydraulic valve exclusivity.
 *
 * Valve pairs 1/2, 3/4 and 5/6 are opposite hydraulic functions.  If any
 * upstream command accidentally requests both valves in a pair, this adapter
 * clears the whole pair instead of choosing a direction.  That fail-closed
 * behavior prevents a wiring or arbitration bug from energizing both sides of
 * one hydraulic circuit at the same time.
 */
static uint32_t sanitize_hydraulic_valve_mask(const ecu_hardware_config_t *config,
                                              uint32_t requested_mask,
                                              uint32_t *interlocked_mask)
{
    uint32_t valve_mask = requested_mask & config->hydraulic_managed_valve_mask;
    if (interlocked_mask != 0) {
        *interlocked_mask = 0U;
    }
    valve_mask = clear_conflicting_valve_pair(valve_mask,
                                              config->hydraulic_valve_interlock_pair12_mask,
                                              interlocked_mask);
    valve_mask = clear_conflicting_valve_pair(valve_mask,
                                              config->hydraulic_valve_interlock_pair34_mask,
                                              interlocked_mask);
    valve_mask = clear_conflicting_valve_pair(valve_mask,
                                              config->hydraulic_valve_interlock_pair56_mask,
                                              interlocked_mask);
    return valve_mask;
}

/* Convert signed track-width rate into the PCB hydraulic valve output mask.
 * The final DIO write is still gated by the hydraulic-enable command. */
static uint32_t valve_mask_from_track_rate(const ecu_hardware_config_t *config,
                                           float track_rate_mm_s)
{
    if (track_rate_mm_s > 0.0f) {
        return config->hydraulic_track_extend_mask;
    }
    if (track_rate_mm_s < 0.0f) {
        return config->hydraulic_track_retract_mask;
    }
    return 0U;
}

int32_t hydraulic_pump_safe_velocity_units(int32_t requested_velocity_units)
{
    if (requested_velocity_units <= 0) {
        return 0;
    }
    if (requested_velocity_units < ECU_HYDRAULIC_PUMP_MIN_WORK_VELOCITY_UNITS) {
        requested_velocity_units = ECU_HYDRAULIC_PUMP_MIN_WORK_VELOCITY_UNITS;
    }

    int64_t commanded = (int64_t)requested_velocity_units *
                        (int64_t)ECU_HYDRAULIC_PUMP_DIRECTION_SIGN;

    if (commanded < -(int64_t)ECU_HYDRAULIC_PUMP_MAX_REVERSE_VELOCITY_UNITS) {
        commanded = -(int64_t)ECU_HYDRAULIC_PUMP_MAX_REVERSE_VELOCITY_UNITS;
    }
    if (commanded > (int64_t)ECU_HYDRAULIC_PUMP_MAX_REVERSE_VELOCITY_UNITS) {
        commanded = (int64_t)ECU_HYDRAULIC_PUMP_MAX_REVERSE_VELOCITY_UNITS;
    }

#if !ECU_HYDRAULIC_PUMP_ALLOW_POSITIVE_VELOCITY
    if (commanded > 0) {
        return 0;
    }
#endif

    return (int32_t)commanded;
}

/* Avoid resending identical CANopen SDO sequences every scheduler tick.
 *
 * CAN3 servo outputs are disabled in the V7 static-contract commit.  The cache
 * still tracks high-level lift and hydraulic intent so later standard-PDO CAN3
 * work can re-enable command publishing without reintroducing SDO control.
 */
static bool can3_actuator_command_changed(const lift_hydraulic_device_state_t *state,
                                          const vehicle_actuator_command_t *command)
{
    return !state->last_lift_command_valid ||
           state->last_lift_command.target_height_mm != command->target_height_mm ||
           state->last_lift_command.height_rate_mm_s != command->height_rate_mm_s ||
           state->last_lift_command.track_rate_mm_s != command->track_rate_mm_s ||
           state->last_lift_command.hydraulic_enable != command->hydraulic_enable ||
           state->last_lift_command.hydraulic_valve_mask != command->hydraulic_valve_mask ||
           state->last_lift_command.source != command->source;
}

static lift_interpolation_direction_t lift_direction_from_height_rate(float height_rate_mm_s)
{
    if (height_rate_mm_s > 0.0f) {
        /* Extending the legs is confirmed as reverse motor rotation, which
         * moves absolute position toward the negative long-leg limit.
         */
        return LIFT_INTERP_DIRECTION_EXTEND;
    }
    if (height_rate_mm_s < 0.0f) {
        return LIFT_INTERP_DIRECTION_RETRACT;
    }
    return LIFT_INTERP_DIRECTION_HOLD;
}

static bool read_lift_axis_feedback(const canopen_master_service_t *canopen,
                                    const ecu_canopen_node_config_t *node,
                                    uint32_t now_ms,
                                    int32_t *position_counts)
{
    canopen_node_feedback_t feedback;

    if (canopen == 0 || node == 0 || position_counts == 0 ||
        !canopen_master_service_get_node_feedback(canopen, node->node_id, &feedback) ||
        !feedback.tpdo0_valid || !feedback.feedback_fresh ||
        (uint32_t)(now_ms - feedback.last_tpdo0_ms) > ECU_CANOPEN_LIFT_FEEDBACK_TIMEOUT_MS) {
        return false;
    }

    *position_counts = feedback.actual_position_counts;
    return true;
}

static bool refresh_lift_feedback(lift_hydraulic_device_state_t *state,
                                  const canopen_master_service_t *canopen,
                                  const ecu_hardware_config_t *config,
                                  uint32_t now_ms)
{
    uint32_t fresh_mask = 0U;

    for (uint32_t leg = 0U; leg < ECU_WHEEL_COUNT; ++leg) {
        int32_t position_counts = 0;
        if (read_lift_axis_feedback(canopen,
                                    &config->lift_nodes[leg],
                                    now_ms,
                                    &position_counts)) {
            state->lift_actual_position_counts[leg] = position_counts;
            fresh_mask |= (1UL << leg);
        }
    }

    state->lift_feedback_fresh_mask = fresh_mask;
    return fresh_mask == ((1UL << ECU_WHEEL_COUNT) - 1UL);
}

static bool queue_lift_interpolation_setup(lift_hydraulic_device_state_t *state,
                                           canopen_master_service_t *canopen,
                                           const ecu_hardware_config_t *config,
                                           bool high_voltage_feedback_ready,
                                           uint32_t now_ms)
{
    bool ok = true;
    const uint16_t controlword = high_voltage_feedback_ready ?
        SERVO_DRIVE_CONTROL_ENABLE_OPERATION :
        SERVO_DRIVE_CONTROL_DISABLE_VOLTAGE;

    if ((uint32_t)(now_ms - state->last_lift_setup_request_ms) <
        ECU_CANOPEN_LIFT_SETUP_REFRESH_MS) {
        return true;
    }

    for (uint32_t leg = 0U; leg < ECU_WHEEL_COUNT; ++leg) {
        const ecu_canopen_node_config_t *node = &config->lift_nodes[leg];
        bool node_ok = true;

        node_ok = canopen_master_service_request_nmt(
                      canopen,
                      node->node_id,
                      CANOPEN_MASTER_DEBUG_COMMAND_NMT_OPERATIONAL) && node_ok;
        node_ok = canopen_master_service_request_sdo_write(
                      canopen,
                      node->node_id,
                      ECU_CANOPEN_OBJ_MODES_OF_OPERATION,
                      0U,
                      1U,
                      CANOPEN_PDO_MODE_INTERPOLATED_POSITION) && node_ok;
        node_ok = canopen_master_service_request_sdo_write(
                      canopen,
                      node->node_id,
                      ECU_CANOPEN_OBJ_INTERPOLATION_TIME_PERIOD,
                      1U,
                      1U,
                      ECU_CANOPEN_LIFT_INTERPOLATION_PERIOD_MS) && node_ok;
        node_ok = canopen_master_service_request_sdo_write(
                      canopen,
                      node->node_id,
                      ECU_CANOPEN_OBJ_INTERPOLATION_MODE,
                      0U,
                      1U,
                      0) && node_ok;
        node_ok = canopen_master_service_request_sdo_write(
                      canopen,
                      node->node_id,
                      ECU_CANOPEN_OBJ_CONTROLWORD,
                      0U,
                      2U,
                      controlword) && node_ok;

        if (node_ok) {
            state->lift_setup_request_mask |= (1UL << leg);
        } else {
            ok = false;
        }
    }

    state->last_lift_setup_request_ms = now_ms;
    return ok;
}

static int32_t corrected_lift_delta_counts(lift_interpolation_direction_t direction,
                                           int32_t actual_position_counts,
                                           int32_t average_position_counts)
{
    int32_t step_counts = lift_interpolation_step_counts();
    int32_t error_counts = actual_position_counts - average_position_counts;
    int32_t extra_counts = 0;

    if (direction == LIFT_INTERP_DIRECTION_EXTEND) {
        if (error_counts > ECU_LIFT_SYNC_RECOVERY_TOLERANCE_COUNTS) {
            extra_counts = i32_abs(error_counts) / 4;
        } else if (error_counts < -ECU_LIFT_SYNC_RECOVERY_TOLERANCE_COUNTS) {
            extra_counts = -(i32_abs(error_counts) / 4);
        }
    } else if (direction == LIFT_INTERP_DIRECTION_RETRACT) {
        if (error_counts < -ECU_LIFT_SYNC_RECOVERY_TOLERANCE_COUNTS) {
            extra_counts = i32_abs(error_counts) / 4;
        } else if (error_counts > ECU_LIFT_SYNC_RECOVERY_TOLERANCE_COUNTS) {
            extra_counts = -(i32_abs(error_counts) / 4);
        }
    }

    extra_counts = clamp_i32(extra_counts,
                             -(step_counts / 2),
                             ECU_LIFT_SYNC_RECOVERY_MAX_EXTRA_COUNTS);
    step_counts += extra_counts;
    if (step_counts < 0) {
        step_counts = 0;
    }

    return (int32_t)direction * step_counts;
}

static bool build_lift_interpolation_request(canopen_master_pdo_request_t *request,
                                             const ecu_canopen_node_config_t *node,
                                             int32_t target_position_counts,
                                             uint32_t group_sequence)
{
    canopen_node_pdo_profile_t profile;

    if (request == 0 || node == 0 ||
        !canopen_pdo_profile_init(node->node_id,
                                  CANOPEN_AXIS_ROLE_LIFT_POSITION,
                                  &profile)) {
        return false;
    }

    return canopen_pdo_build_interpolated_position_rpdo2(
        &profile,
        target_position_counts,
        request,
        group_sequence,
        CANOPEN_MASTER_PDO_PHASE_LIFT_INTERPOLATION_POINT);
}

static bool queue_lift_interpolation_group(lift_hydraulic_device_state_t *state,
                                           canopen_master_service_t *canopen,
                                           const ecu_hardware_config_t *config,
                                           lift_interpolation_direction_t direction,
                                           uint32_t now_ms)
{
    canopen_master_pdo_request_t requests[ECU_WHEEL_COUNT];
    int64_t position_sum = 0;
    int32_t average_position_counts;
    uint32_t group_sequence;

    if (direction == LIFT_INTERP_DIRECTION_HOLD) {
        state->lift_targets_initialized = false;
        state->lift_hold_count++;
        return true;
    }

    if ((uint32_t)(now_ms - state->last_lift_interpolation_ms) <
        ECU_CANOPEN_LIFT_INTERPOLATION_REFRESH_MS) {
        return true;
    }

    if (!refresh_lift_feedback(state, canopen, config, now_ms)) {
        state->lift_interpolation_reject_count++;
        state->lift_targets_initialized = false;
        return false;
    }

    for (uint32_t leg = 0U; leg < ECU_WHEEL_COUNT; ++leg) {
        position_sum += state->lift_actual_position_counts[leg];
    }
    average_position_counts = (int32_t)(position_sum / (int64_t)ECU_WHEEL_COUNT);
    group_sequence = next_lift_group_sequence(state);

    for (uint32_t leg = 0U; leg < ECU_WHEEL_COUNT; ++leg) {
        int32_t delta_counts =
            corrected_lift_delta_counts(direction,
                                        state->lift_actual_position_counts[leg],
                                        average_position_counts);
        int32_t target_counts =
            clamp_lift_position_counts(state->lift_actual_position_counts[leg] +
                                       delta_counts);
        if (!build_lift_interpolation_request(&requests[leg],
                                              &config->lift_nodes[leg],
                                              target_counts,
                                              group_sequence)) {
            state->lift_interpolation_reject_count++;
            return false;
        }
        state->lift_target_position_counts[leg] = target_counts;
    }

    canopen_master_pdo_group_descriptor_t descriptor = {
        .expected_frames = ECU_WHEEL_COUNT,
        .arm_frame_count = ECU_WHEEL_COUNT,
        .trigger_frame_count = 0U,
        .axis_mask = (uint8_t)((1U << ECU_WHEEL_COUNT) - 1U),
        .position_group = true,
        .sync_after_arm = true,
        .sync_after_trigger = false
    };

    if (!canopen_master_service_queue_pdo_batch_with_descriptor(
            canopen,
            requests,
            ECU_WHEEL_COUNT,
            &descriptor)) {
        state->lift_interpolation_failure_count++;
        return false;
    }

    state->lift_targets_initialized = true;
    state->lift_interpolation_queued_count++;
    state->last_lift_interpolation_ms = now_ms;
    return true;
}

static bool build_pump_velocity_request(canopen_master_pdo_request_t *request,
                                        const ecu_canopen_node_config_t *node,
                                        uint16_t controlword,
                                        int32_t target_velocity_units,
                                        uint32_t group_sequence)
{
    canopen_node_pdo_profile_t profile;

    if (request == 0 || node == 0 ||
        !canopen_pdo_profile_init(node->node_id,
                                  CANOPEN_AXIS_ROLE_HYDRAULIC_VELOCITY,
                                  &profile)) {
        return false;
    }

    return canopen_pdo_build_velocity_rpdo0(
        &profile,
        controlword,
        target_velocity_units,
        request,
        group_sequence,
        CANOPEN_MASTER_PDO_PHASE_HYDRAULIC_PUMP_VELOCITY);
}

static bool queue_hydraulic_pump_velocity(lift_hydraulic_device_state_t *state,
                                          canopen_master_service_t *canopen,
                                          const ecu_hardware_config_t *config,
                                          bool pump_enable,
                                          uint32_t now_ms)
{
    canopen_master_pdo_request_t request;
    const bool pump_should_run = pump_enable;
    const int32_t requested_units =
        pump_should_run ? ECU_HYDRAULIC_PUMP_ENABLE_VELOCITY_UNITS : 0;
    const int32_t target_velocity_units =
        hydraulic_pump_safe_velocity_units(requested_units);
    const uint16_t controlword =
        pump_should_run ? SERVO_DRIVE_CONTROL_ENABLE_OPERATION :
                          SERVO_DRIVE_CONTROL_DISABLE_VOLTAGE;

    if (pump_should_run && target_velocity_units >= 0) {
        state->pump_positive_clamp_count++;
    }

    if (state->last_pump_velocity_units == target_velocity_units &&
        (uint32_t)(now_ms - state->last_pump_velocity_ms) <
            ECU_CANOPEN_PUMP_VELOCITY_REFRESH_MS) {
        return true;
    }

    uint32_t group_sequence = next_pump_group_sequence(state);
    if (!build_pump_velocity_request(&request,
                                     &config->hydraulic_pump_node,
                                     controlword,
                                     target_velocity_units,
                                     group_sequence)) {
        state->pump_velocity_reject_count++;
        return false;
    }

    canopen_master_pdo_group_descriptor_t descriptor = {
        .expected_frames = 1U,
        .arm_frame_count = 1U,
        .trigger_frame_count = 0U,
        .axis_mask = 0U,
        .position_group = false,
        .sync_after_arm = true,
        .sync_after_trigger = false
    };

    if (!canopen_master_service_queue_pdo_batch_with_descriptor(
            canopen,
            &request,
            1U,
            &descriptor)) {
        state->pump_velocity_reject_count++;
        return false;
    }

    state->last_pump_velocity_units = target_velocity_units;
    state->last_pump_velocity_ms = now_ms;
    state->pump_velocity_queued_count++;
    return true;
}

/* Validate Node13 TPDO feedback before it is used as the valve-open interlock.
 *
 * The pump is mechanically reverse-only.  A positive measured speed therefore
 * never satisfies the interlock even if its magnitude exceeds the threshold.
 * Stale TPDO data, a CiA-402 fault bit, or the vendor fault latch all fail
 * closed.
 */
static bool read_hydraulic_pump_feedback(lift_hydraulic_device_state_t *state,
                                         const canopen_master_service_t *canopen,
                                         const ecu_hardware_config_t *config,
                                         uint32_t now_ms)
{
    canopen_node_feedback_t feedback;

    state->pump_feedback_valid = false;
    if (!canopen_master_service_get_node_feedback(
            canopen, config->hydraulic_pump_node.node_id, &feedback) ||
        !feedback.tpdo0_valid ||
        !feedback.feedback_fresh ||
        (uint32_t)(now_ms - feedback.last_tpdo0_ms) >
            ECU_CANOPEN_LIFT_FEEDBACK_TIMEOUT_MS ||
        (feedback.statusword & (uint16_t)(1U << 3)) != 0U ||
        feedback.fault_latched != 0U) {
        state->pump_feedback_reject_count++;
        return false;
    }

    state->pump_actual_velocity_units = feedback.actual_velocity_units;
    state->pump_feedback_valid = true;
    return true;
}

static void close_hydraulic_valves(lift_hydraulic_device_state_t *state,
                                   dio_service_t *dio,
                                   const ecu_hardware_config_t *config)
{
    dio_service_write_masked(dio, config->dio_hydraulic_enable_mask, false);
    dio_service_write_masked(dio, config->hydraulic_managed_valve_mask, false);
    state->last_valve_mask = 0U;
}

/* Apply the pump-first hydraulic sequence.
 *
 *  1. Close every valve before starting or changing direction.
 *  2. Run Node13 in the mechanically safe reverse direction.
 *  3. Require three consecutive fresh TPDO samples above 800 rpm.
 *  4. Open only the sanitized requested valve mask.
 *
 * A start timeout remains latched until the operator releases the hydraulic
 * request.  This prevents an unavailable pump from cycling indefinitely while
 * a valve command remains held.
 */
static bool apply_hydraulic_pump_and_valves(lift_hydraulic_device_state_t *state,
                                            canopen_master_service_t *canopen,
                                            dio_service_t *dio,
                                            const ecu_hardware_config_t *config,
                                            uint32_t valve_mask,
                                            bool hydraulic_request,
                                            uint32_t now_ms)
{
    const bool request_active = hydraulic_request && valve_mask != 0U;

    if (!request_active) {
        close_hydraulic_valves(state, dio, config);
        state->pending_valve_mask = 0U;
        state->pump_speed_ready_samples = 0U;
        state->pump_feedback_valid = false;
        state->pump_state = HYDRAULIC_PUMP_STATE_STOPPED;
        state->pump_start_request_ms = 0U;
        return queue_hydraulic_pump_velocity(state, canopen, config, false, now_ms);
    }

    if (state->pump_state == HYDRAULIC_PUMP_STATE_START_TIMEOUT) {
        close_hydraulic_valves(state, dio, config);
        return queue_hydraulic_pump_velocity(state, canopen, config, false, now_ms);
    }

    if (state->pending_valve_mask != valve_mask) {
        close_hydraulic_valves(state, dio, config);
        state->pending_valve_mask = valve_mask;
        state->valve_change_hold_until_ms =
            now_ms + ECU_HYDRAULIC_VALVE_CHANGE_DEADTIME_MS;
    }

    if (state->pump_state == HYDRAULIC_PUMP_STATE_STOPPED) {
        state->pump_state = HYDRAULIC_PUMP_STATE_STARTING;
        state->pump_start_request_ms = now_ms;
        state->pump_speed_ready_samples = 0U;
    }

    const bool was_valve_ready =
        state->pump_state == HYDRAULIC_PUMP_STATE_VALVE_READY;
    const bool feedback_ok =
        read_hydraulic_pump_feedback(state, canopen, config, now_ms);
    const bool reverse_speed_ready =
        feedback_ok &&
        state->pump_actual_velocity_units <
            -ECU_HYDRAULIC_PUMP_VALVE_OPEN_MIN_VELOCITY_UNITS;

    if (reverse_speed_ready) {
        if (state->pump_speed_ready_samples < ECU_HYDRAULIC_PUMP_SPEED_READY_SAMPLES) {
            state->pump_speed_ready_samples++;
        }
    } else {
        state->pump_speed_ready_samples = 0U;
        state->pump_state = HYDRAULIC_PUMP_STATE_STARTING;
        if (was_valve_ready) {
            /* A running pump gets a fresh recovery window after a speed or
             * feedback dropout.  The valve still closes immediately.
             */
            state->pump_start_request_ms = now_ms;
        }
        close_hydraulic_valves(state, dio, config);
    }

    if (state->pump_speed_ready_samples <
            ECU_HYDRAULIC_PUMP_SPEED_READY_SAMPLES &&
        (uint32_t)(now_ms - state->pump_start_request_ms) >
            ECU_HYDRAULIC_PUMP_START_TIMEOUT_MS) {
        close_hydraulic_valves(state, dio, config);
        state->pump_state = HYDRAULIC_PUMP_STATE_START_TIMEOUT;
        state->pump_start_timeout_count++;
        return queue_hydraulic_pump_velocity(state, canopen, config, false, now_ms);
    }

    const bool pump_command_ok =
        queue_hydraulic_pump_velocity(state, canopen, config, true, now_ms);
    if (!pump_command_ok ||
        state->pump_speed_ready_samples < ECU_HYDRAULIC_PUMP_SPEED_READY_SAMPLES ||
        !time_reached(now_ms, state->valve_change_hold_until_ms)) {
        close_hydraulic_valves(state, dio, config);
        return pump_command_ok;
    }

    state->pump_state = HYDRAULIC_PUMP_STATE_VALVE_READY;
    dio_service_write_masked(dio, config->dio_hydraulic_enable_mask, true);
    dio_service_write_masked(dio, config->hydraulic_managed_valve_mask, false);
    dio_service_write_masked(dio, valve_mask, true);
    state->last_valve_mask = valve_mask;
    return true;
}

/* Reset counters and cached command state.  The device adapter owns no hardware
 * resources directly; CANopen and DIO services are injected at apply time. */
void lift_hydraulic_device_init(lift_hydraulic_device_state_t *state)
{
    if (state != 0) {
        memset(state, 0, sizeof(*state));
        state->last_result = ECU_DEVICE_APPLY_OK;
    }
}

/* Apply the vehicle lift/hydraulic command to CAN3 servos and local valves.
 *
 * Ground-clearance motion is owned by four CAN3 lift servos.  The local relay
 * box is limited to hydraulic-station enable and hydraulic valve coils; it is
 * not used to synthesize lift-servo brake release or to move the electric lift
 * axes.  Realtime lift points and pump velocity commands are queued through
 * the CAN3 CANopen service so bus ordering remains under the CAN3 task owner.
 */
ecu_device_apply_result_t lift_hydraulic_device_apply(lift_hydraulic_device_state_t *state,
                                                      canopen_master_service_t *canopen,
                                                      dio_service_t *dio,
                                                      const ecu_hardware_config_t *config,
                                                      const vehicle_actuator_command_t *command,
                                                      uint32_t now_ms)
{
    if (state == 0 || canopen == 0 || dio == 0 || config == 0 || command == 0) {
        return ECU_DEVICE_APPLY_INVALID_ARGUMENT;
    }
    if (!canopen->snapshot.initialized || !canopen->snapshot.can_normal) {
        close_hydraulic_valves(state, dio, config);
        state->pump_feedback_valid = false;
        state->pump_speed_ready_samples = 0U;
        state->pump_state = HYDRAULIC_PUMP_STATE_STOPPED;
        state->last_result = ECU_DEVICE_APPLY_BACKEND_OFFLINE;
        return state->last_result;
    }

    bool ok = true;
    lift_interpolation_direction_t lift_direction =
        lift_direction_from_height_rate(command->height_rate_mm_s);
    if (!command->high_voltage_feedback_ready) {
        lift_direction = LIFT_INTERP_DIRECTION_HOLD;
    }

    if (can3_actuator_command_changed(state, command)) {
        state->last_lift_command = *command;
        state->last_lift_command_valid = true;
        state->last_lift_command_queue_ms = now_ms;
    } else {
        state->skipped_lift_canopen_count++;
    }

    ok = queue_lift_interpolation_setup(state,
                                        canopen,
                                        config,
                                        command->high_voltage_feedback_ready,
                                        now_ms) && ok;
    ok = queue_lift_interpolation_group(state,
                                        canopen,
                                        config,
                                        lift_direction,
                                        now_ms) && ok;
    uint32_t requested_valve_mask = command->hydraulic_valve_mask |
                                    valve_mask_from_track_rate(config, command->track_rate_mm_s);
    uint32_t interlocked_valve_mask = 0U;
    uint32_t valve_mask = sanitize_hydraulic_valve_mask(config,
                                                        requested_valve_mask,
                                                        &interlocked_valve_mask);
    const bool hydraulic_request =
        command->hydraulic_enable &&
        command->high_voltage_feedback_ready &&
        valve_mask != 0U;
    ok = apply_hydraulic_pump_and_valves(state,
                                         canopen,
                                         dio,
                                         config,
                                         valve_mask,
                                         hydraulic_request,
                                         now_ms) && ok;
    state->last_requested_valve_mask = requested_valve_mask & config->hydraulic_managed_valve_mask;
    state->last_interlocked_valve_mask = interlocked_valve_mask;
    if (interlocked_valve_mask != 0U) {
        state->valve_interlock_reject_count++;
    }
    state->apply_count++;
    state->last_result = ok ? ECU_DEVICE_APPLY_OK : ECU_DEVICE_APPLY_REJECTED;
    return state->last_result;
}
