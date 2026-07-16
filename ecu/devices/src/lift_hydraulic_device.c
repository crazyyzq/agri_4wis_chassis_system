#include <string.h>
#include <stdint.h>
#include <limits.h>

#include "canopen_pdo_profile.h"
#include "lift_hydraulic_device.h"

#define SERVO_DRIVE_CONTROL_DISABLE_VOLTAGE ((uint16_t)0x0000U)
#define SERVO_DRIVE_CONTROL_SHUTDOWN ((uint16_t)0x0006U)
#define SERVO_DRIVE_CONTROL_SWITCH_ON ((uint16_t)0x0007U)
#define SERVO_DRIVE_CONTROL_ENABLE_OPERATION ((uint16_t)0x000FU)
#define SERVO_DRIVE_CONTROL_START_INTERPOLATION ((uint16_t)0x003FU)
#define SERVO_DRIVE_CONTROL_FAULT_RESET ((uint16_t)0x0080U)
#define LIFT_SETUP_SDO_WRITES_ENABLED_PER_NODE (12U)
#define LIFT_SETUP_SDO_WRITES_DISABLED_PER_NODE (9U)
#define PUMP_SETUP_SDO_WRITES (6U)

typedef enum {
    LIFT_INTERP_DIRECTION_HOLD = 0,
    LIFT_INTERP_DIRECTION_EXTEND = -1,
    LIFT_INTERP_DIRECTION_RETRACT = 1
} lift_interpolation_direction_t;

typedef enum {
    CAN3_PDO_SUBMIT_OK = 0,
    CAN3_PDO_SUBMIT_DEFERRED,
    CAN3_PDO_SUBMIT_TRANSPORT_FAULT,
    CAN3_PDO_SUBMIT_INVALID
} can3_pdo_submit_result_t;

static void lift_stream_reset_trajectory(lift_hydraulic_device_state_t *state,
                                         uint32_t now_ms);

static void lift_settle_reset(lift_hydraulic_device_state_t *state)
{
    if (state == 0) {
        return;
    }
    state->lift_settle_initialized = false;
    state->lift_settle_sample_ms = 0U;
    state->lift_settle_stable_since_ms = 0U;
}

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

static int32_t lift_interpolation_step_counts(lift_hydraulic_device_state_t *state,
                                              lift_interpolation_direction_t direction,
                                              int32_t remaining_counts,
                                              int32_t speed_limit_counts_per_sec,
                                              uint32_t now_ms)
{
    const int64_t acceleration =
        ECU_LIFT_INTERPOLATION_ACCEL_COUNTS_PER_SEC2;
    const int64_t period_ms = ECU_CANOPEN_LIFT_INTERPOLATION_PERIOD_MS;
    int64_t requested_velocity;
    int64_t next_velocity;
    int64_t step;

    if (state == 0 || direction == LIFT_INTERP_DIRECTION_HOLD) {
        if (state != 0) {
            state->lift_stream_velocity_counts_per_sec = 0;
            state->lift_last_stream_step_ms = now_ms;
        }
        return 0;
    }

    state->lift_last_stream_step_ms = now_ms;

    /* Match the analyzer script's fixed-sample trapezoid.  The BC2 drive
     * consumes each buffered interpolation point as one interpolation-time
     * segment
     * (20 ms in the current setup), so the planned point-to-point distance must
     * be based on the configured interpolation period, not on FreeRTOS/CAN
     * scheduling jitter.  If one ECU producer cycle is late, the scheduler
     * catches up by preserving the planned 20 ms phase instead of increasing
     * the next segment distance.
     */
    const int64_t velocity = state->lift_stream_velocity_counts_per_sec;
    const int64_t braking_distance = acceleration > 0 ?
        (velocity * velocity) / (2 * acceleration) : 0;
    const int64_t velocity_step = (acceleration * period_ms) / 1000;

    speed_limit_counts_per_sec = clamp_i32(
        speed_limit_counts_per_sec,
        ECU_LIFT_FEEDBACK_GOVERNOR_MIN_SPEED_COUNTS_PER_SEC,
        ECU_LIFT_INTERPOLATION_SPEED_COUNTS_PER_SEC);

    /* The feedback governor may lower the allowed velocity while the target is
     * still far from its final position.  Approach that lower limit with the
     * same physical acceleration step instead of clamping velocity in one
     * cycle; an instantaneous clamp was visible as a loaded-chassis jerk. */
    requested_velocity = speed_limit_counts_per_sec;
    if ((int64_t)remaining_counts <= braking_distance) {
        requested_velocity = velocity > velocity_step ?
            velocity - velocity_step : 0;
    }
    if (velocity < requested_velocity) {
        next_velocity = velocity + velocity_step;
        if (next_velocity > requested_velocity) {
            next_velocity = requested_velocity;
        }
    } else {
        next_velocity = velocity > velocity_step ?
            velocity - velocity_step : 0;
        if (next_velocity < requested_velocity) {
            next_velocity = requested_velocity;
        }
    }
    state->lift_stream_velocity_counts_per_sec = (int32_t)next_velocity;

    step = (next_velocity * period_ms) / 1000;
    if (step <= 0 && remaining_counts > 0) {
        step = 1;
    }
    return step > 0 ? (int32_t)step : 0;
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

/* Track whether the high-level CAN3 command snapshot changed.
 *
 * The cache is only a diagnostic/retry aid; it must never be used as proof that
 * a remote drive or valve accepted the command.  Lift interpolation and pump
 * velocity are still submitted through the CAN3-owned CANopen service below.
 */
static bool can3_actuator_command_changed(const lift_hydraulic_device_state_t *state,
                                          const vehicle_actuator_command_t *command)
{
    return !state->last_lift_command_valid ||
           state->last_lift_command.target_height_mm != command->target_height_mm ||
           state->last_lift_command.height_rate_mm_s != command->height_rate_mm_s ||
           state->last_lift_command.lift_request != command->lift_request ||
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

static int32_t lift_target_counts_from_mm(float target_height_mm)
{
    if (target_height_mm < ECU_REMOTE_MIN_HEIGHT_TARGET_MM) {
        target_height_mm = ECU_REMOTE_MIN_HEIGHT_TARGET_MM;
    }
    if (target_height_mm > ECU_REMOTE_MAX_HEIGHT_TARGET_MM) {
        target_height_mm = ECU_REMOTE_MAX_HEIGHT_TARGET_MM;
    }

    return clamp_lift_position_counts(
        (int32_t)(-(target_height_mm * ECU_LIFT_MM_TO_COUNTS)));
}

static int32_t lift_unclamped_counts_from_mm(float height_mm)
{
    return (int32_t)(-(height_mm * ECU_LIFT_MM_TO_COUNTS));
}

/* Classify measured lift positions without changing the drive's established
 * mechanical zero.  The 10..490 mm band is the normal software range; the
 * wider 0..500 mm band plus a small measurement margin is only a plausibility
 * guard. */
static void update_lift_position_range_masks(lift_hydraulic_device_state_t *state)
{
    const int32_t safe_retracted =
        lift_unclamped_counts_from_mm(ECU_REMOTE_MIN_HEIGHT_TARGET_MM);
    const int32_t safe_extended =
        lift_unclamped_counts_from_mm(ECU_REMOTE_MAX_HEIGHT_TARGET_MM);
    const int32_t mechanical_retracted =
        lift_unclamped_counts_from_mm(ECU_LIFT_MECHANICAL_MIN_HEIGHT_MM);
    const int32_t mechanical_extended =
        lift_unclamped_counts_from_mm(ECU_LIFT_MECHANICAL_MAX_HEIGHT_MM);
    const int32_t plausibility_margin =
        (int32_t)(ECU_LIFT_MECHANICAL_PLAUSIBILITY_MARGIN_MM *
                  ECU_LIFT_MM_TO_COUNTS);
    uint8_t below_mask = 0U;
    uint8_t above_mask = 0U;
    uint8_t invalid_mask = 0U;

    if (state == 0) {
        return;
    }
    for (uint32_t leg = 0U; leg < ECU_WHEEL_COUNT; ++leg) {
        const uint8_t leg_mask = (uint8_t)(1U << leg);
        const int32_t position = state->lift_actual_position_counts[leg];
        if (position > safe_retracted) {
            below_mask |= leg_mask;
        }
        if (position < safe_extended) {
            above_mask |= leg_mask;
        }
        if ((int64_t)position >
                (int64_t)mechanical_retracted + plausibility_margin ||
            (int64_t)position <
                (int64_t)mechanical_extended - plausibility_margin) {
            invalid_mask |= leg_mask;
        }
    }
    state->lift_below_safe_range_mask = below_mask;
    state->lift_above_safe_range_mask = above_mask;
    state->lift_mechanical_range_invalid_mask = invalid_mask;
}

/* A single common travel direction must move every out-of-range leg toward
 * the normal band.  Mixed low/high violations cannot be corrected safely by
 * one ordinary extend/retract command and are therefore rejected. */
static bool lift_direction_allowed_by_range(
    const lift_hydraulic_device_state_t *state,
    lift_interpolation_direction_t direction)
{
    if (state == 0 || state->lift_mechanical_range_invalid_mask != 0U) {
        return false;
    }
    if (state->lift_below_safe_range_mask != 0U &&
        state->lift_above_safe_range_mask != 0U) {
        return false;
    }
    if (state->lift_below_safe_range_mask != 0U) {
        return direction == LIFT_INTERP_DIRECTION_EXTEND;
    }
    if (state->lift_above_safe_range_mask != 0U) {
        return direction == LIFT_INTERP_DIRECTION_RETRACT;
    }
    return direction != LIFT_INTERP_DIRECTION_HOLD;
}

static int32_t lift_level_target_from_feedback(
    const lift_hydraulic_device_state_t *state)
{
    int32_t sorted[ECU_WHEEL_COUNT];

    for (uint32_t leg = 0U; leg < ECU_WHEEL_COUNT; ++leg) {
        sorted[leg] = state->lift_actual_position_counts[leg];
    }
    for (uint32_t i = 1U; i < ECU_WHEEL_COUNT; ++i) {
        int32_t value = sorted[i];
        uint32_t j = i;
        while (j > 0U && sorted[j - 1U] > value) {
            sorted[j] = sorted[j - 1U];
            --j;
        }
        sorted[j] = value;
    }

    /* Midpoint of the two central legs rejects one high and one low outlier
     * and minimizes the largest correction compared with choosing an end. */
    int32_t target = (int32_t)(((int64_t)sorted[1] + sorted[2]) / 2);
    const int32_t safe_extended =
        lift_unclamped_counts_from_mm(ECU_REMOTE_MAX_HEIGHT_TARGET_MM);
    const int32_t safe_retracted =
        lift_unclamped_counts_from_mm(ECU_REMOTE_MIN_HEIGHT_TARGET_MM);
    return clamp_i32(target, safe_extended, safe_retracted);
}

static bool read_lift_axis_feedback(const canopen_master_service_t *canopen,
                                    const ecu_canopen_node_config_t *node,
                                    uint32_t now_ms,
                                    int32_t *position_counts,
                                    int32_t *velocity_units,
                                    bool *axis_fault)
{
    canopen_node_feedback_t feedback;

    if (axis_fault != 0) {
        *axis_fault = false;
    }
    if (canopen == 0 || node == 0 || position_counts == 0 ||
        velocity_units == 0 ||
        axis_fault == 0 ||
        !canopen_master_service_get_node_feedback(canopen, node->node_id, &feedback)) {
        return false;
    }

    /* BC2's statusword is not CiA-402-bit compatible in interpolation mode
     * (a healthy movable axis can report 0x162F).  The analyzer-proven hard
     * fault source is the vendor 0x2183 word mapped in TPDO1. */
    if (feedback.tpdo1_valid && feedback.fault_latched != 0U) {
        *axis_fault = true;
        return false;
    }
    if (!feedback.tpdo0_valid || !feedback.feedback_fresh ||
        (uint32_t)(now_ms - feedback.last_tpdo0_ms) > ECU_CANOPEN_LIFT_FEEDBACK_TIMEOUT_MS) {
        return false;
    }

    *position_counts = feedback.actual_position_counts;
    *velocity_units = feedback.actual_velocity_units;
    return true;
}

static bool refresh_lift_feedback(lift_hydraulic_device_state_t *state,
                                  const canopen_master_service_t *canopen,
                                  const ecu_hardware_config_t *config,
                                  uint32_t now_ms)
{
    uint32_t fresh_mask = 0U;
    uint32_t fault_mask = 0U;

    for (uint32_t leg = 0U; leg < ECU_WHEEL_COUNT; ++leg) {
        int32_t position_counts = 0;
        int32_t velocity_units = 0;
        bool axis_fault = false;
        if (read_lift_axis_feedback(canopen,
                                    &config->lift_nodes[leg],
                                    now_ms,
                                    &position_counts,
                                    &velocity_units,
                                    &axis_fault)) {
            state->lift_actual_position_counts[leg] = position_counts;
            state->lift_actual_velocity_units[leg] = velocity_units;
            fresh_mask |= (1UL << leg);
        } else if (axis_fault) {
            fault_mask |= (1UL << leg);
        }
    }

    state->lift_feedback_fresh_mask = fresh_mask;
    state->lift_axis_fault_mask = fault_mask;
    const bool all_fresh = fresh_mask == ((1UL << ECU_WHEEL_COUNT) - 1UL);
    if (all_fresh) {
        state->lift_feedback_missing_since_ms = 0U;
    }
    return all_fresh;
}

/* Judge setup readiness without turning sparse TPDO1 traffic into a hard
 * startup deadlock.
 *
 * The commissioning PDO profile may make TPDO1 synchronous or low-rate.  The
 * drive has already acknowledged the CiA-402 setup sequence through SDO before
 * this helper is queried, so lack of a fresh TPDO1 is not by itself a reason
 * to block motion forever.  BC/BC2 field evidence also shows that a healthy,
 * enabled and movable interpolation axis can report statusword 0x162F.  The
 * statusword is therefore diagnostic-only here; a nonzero vendor 0x2183 latch
 * remains a hard setup gate.  TPDO0 is still mandatory before actual motion
 * because interpolation points must start from measured position.
 */
static bool servo_setup_feedback_allows_ready(
    const canopen_master_service_t *canopen,
    uint8_t node_id)
{
    canopen_node_feedback_t feedback;

    /* Accept the field-observed healthy 0x162F statusword; only 0x2183 is a
     * reliable hard-fault gate for this BC/BC2 interpolation setup. */
    if (!canopen_master_service_get_node_feedback(
            canopen, node_id, &feedback)) {
        return true;
    }
    if (feedback.tpdo1_valid && feedback.fault_latched != 0U) {
        return false;
    }
    return true;
}

static bool lift_axes_setup_feedback_ready(
    const canopen_master_service_t *canopen,
    const ecu_hardware_config_t *config)
{
    for (uint32_t leg = 0U; leg < ECU_WHEEL_COUNT; ++leg) {
        if (!servo_setup_feedback_allows_ready(
                canopen, config->lift_nodes[leg].node_id)) {
            return false;
        }
    }
    return true;
}

/* Start one nonblocking setup transaction for all lift axes.
 *
 * Bench evidence on the installed BC2 firmware established that the configured
 * interpolation-buffer-clear object does not reliably discard stale points
 * while the axis remains Operation Enabled.  The mandatory order is therefore:
 *
 *   shutdown -> mode/time/submode -> clear buffer
 *   -> switch on -> operation enabled -> profile limits
 *
 * 0x0006 is a CiA-402 state transition; it does not reset the drive or its
 * absolute position reference.  Field testing with the CAN analyzer showed
 * that BC2 lift interpolation runs continuously only when the drive is already
 * Operation Enabled before the first RPDO2 buffer points are queued.  The SDO
 * stage therefore configures all four axes while disabled.  The following
 * 0x0007 and 0x000F transitions are separate four-axis RPDO1 groups with one
 * SYNC per group, so brake release and Operation Enabled do not walk one node
 * at a time.  The realtime trajectory then uses RPDO2 + SYNC, with the later
 * RPDO1 0x003F group as the interpolation start edge after all four axes have
 * received the same number of preload points.
 */
static bool queue_lift_setup_sdos(lift_hydraulic_device_state_t *state,
                                  canopen_master_service_t *canopen,
                                  const ecu_hardware_config_t *config,
                                  uint32_t now_ms)
{
    const uint32_t writes_per_node = state->lift_setup_enable_operation ?
        LIFT_SETUP_SDO_WRITES_ENABLED_PER_NODE :
        LIFT_SETUP_SDO_WRITES_DISABLED_PER_NODE;
    uint32_t expected_writes = ECU_WHEEL_COUNT * writes_per_node;

    if (!canopen_master_service_sdo_download_idle(canopen)) {
        state->lift_interpolation_reject_count++;
        return false;
    }

    for (uint32_t leg = 0U; leg < ECU_WHEEL_COUNT; ++leg) {
        const ecu_canopen_node_config_t *node = &config->lift_nodes[leg];
        canopen_node_feedback_t feedback;
        bool node_ok = true;
        if (canopen_master_service_get_node_feedback(
                canopen, node->node_id, &feedback) &&
            feedback.tpdo1_valid && feedback.fault_latched != 0U) {
            node_ok = canopen_master_service_request_sdo_write(
                canopen,
                node->node_id,
                ECU_CANOPEN_OBJ_FAULT_LATCHED,
                0U,
                4U,
                (int32_t)feedback.fault_latched);
            if (node_ok) {
                expected_writes++;
            }
        }
        node_ok = node_ok &&
            /* CiA-402 fault reset is edge-sensitive.  Drive the reset bit low
             * first so a previous interrupted setup cannot leave bit 7 high
             * and make the following 0x0080 write a no-op. */
            canopen_master_service_request_sdo_write(
                canopen, node->node_id, ECU_CANOPEN_OBJ_CONTROLWORD,
                0U, 2U, SERVO_DRIVE_CONTROL_DISABLE_VOLTAGE) &&
            canopen_master_service_request_sdo_write(
                canopen, node->node_id, ECU_CANOPEN_OBJ_CONTROLWORD,
                0U, 2U, SERVO_DRIVE_CONTROL_FAULT_RESET) &&
            canopen_master_service_request_sdo_write(
                canopen, node->node_id, ECU_CANOPEN_OBJ_CONTROLWORD,
                0U, 2U, SERVO_DRIVE_CONTROL_SHUTDOWN) &&
            canopen_master_service_request_sdo_write(
                canopen, node->node_id, ECU_CANOPEN_OBJ_BC_INTERPOLATION_OPTION,
                0U, 2U, ECU_CANOPEN_BC_INTERPOLATION_OPTION_VALUE) &&
            canopen_master_service_request_sdo_write(
                canopen, node->node_id, ECU_CANOPEN_OBJ_MODES_OF_OPERATION,
                0U, 1U, CANOPEN_PDO_MODE_INTERPOLATED_POSITION) &&
            canopen_master_service_request_sdo_write(
                canopen, node->node_id, ECU_CANOPEN_OBJ_INTERPOLATION_MODE,
                0U, 2U, ECU_CANOPEN_LIFT_INTERPOLATION_MODE) &&
            canopen_master_service_request_sdo_write(
                canopen, node->node_id, ECU_CANOPEN_OBJ_INTERPOLATION_TIME_PERIOD,
                1U, 1U, ECU_CANOPEN_LIFT_INTERPOLATION_PERIOD_MS) &&
            canopen_master_service_request_sdo_write(
                canopen, node->node_id, ECU_CANOPEN_OBJ_INTERPOLATION_BUFFER_CLEAR,
                6U, 1U, 0);

        if (node_ok && state->lift_setup_enable_operation) {
            /* Configure every axis while it remains disabled.  The four final
             * 0x0007 and 0x000F transitions are emitted later as synchronous
             * RPDO1 groups so brake release cannot walk Node9 -> Node12. */
            node_ok =
                canopen_master_service_request_sdo_write(
                    canopen, node->node_id, ECU_CANOPEN_OBJ_PROFILE_VELOCITY,
                    0U, 4U, ECU_LIFT_PROFILE_VELOCITY_UNITS) &&
                canopen_master_service_request_sdo_write(
                    canopen, node->node_id, ECU_CANOPEN_OBJ_PROFILE_ACCELERATION,
                    0U, 4U, ECU_LIFT_PROFILE_ACCEL_UNITS) &&
                canopen_master_service_request_sdo_write(
                    canopen, node->node_id, ECU_CANOPEN_OBJ_PROFILE_DECELERATION,
                    0U, 4U, ECU_LIFT_PROFILE_ACCEL_UNITS) &&
                canopen_master_service_request_sdo_write(
                    canopen, node->node_id, ECU_CANOPEN_OBJ_FOLLOWING_ERROR_WINDOW,
                    0U, 4U, ECU_LIFT_FOLLOWING_ERROR_WINDOW_COUNTS);
        } else if (node_ok) {
            node_ok = canopen_master_service_request_sdo_write(
                canopen, node->node_id, ECU_CANOPEN_OBJ_CONTROLWORD,
                0U, 2U, SERVO_DRIVE_CONTROL_DISABLE_VOLTAGE);
        }

        if (!node_ok) {
            state->lift_interpolation_failure_count++;
            state->lift_interpolation_state = LIFT_INTERPOLATION_STATE_FAULT;
            return false;
        }
    }

    state->lift_setup_sdo_queued = true;
    state->lift_setup_expected_download_count =
        canopen->snapshot.sdo_download_count + expected_writes;
    state->lift_setup_abort_count_baseline =
        canopen->snapshot.sdo_download_abort_count;
    (void)now_ms;
    return true;
}

static bool begin_lift_interpolation_setup(lift_hydraulic_device_state_t *state,
                                           canopen_master_service_t *canopen,
                                           const ecu_hardware_config_t *config,
                                           lift_interpolation_direction_t direction,
                                           bool enable_operation,
                                           uint32_t now_ms)
{
    (void)canopen;
    (void)config;

    if (state->lift_group_in_flight) {
        return false;
    }

    state->lift_setup_request_mask = 0U;
    state->lift_setup_nmt_sent_mask = 0U;
    state->lift_setup_expected_download_count = 0U;
    state->lift_setup_abort_count_baseline = 0U;
    state->lift_setup_sdo_queued = false;
    state->lift_setup_enable_operation = enable_operation;
    state->lift_prepared_disabled = false;
    state->lift_transport_recovery_required = false;
    state->lift_enable_settle_until_ms = 0U;
    lift_settle_reset(state);
    state->lift_setup_deadline_ms =
        now_ms + ECU_CANOPEN_LIFT_SETUP_TIMEOUT_MS;
    state->last_lift_setup_request_ms = now_ms;
    state->lift_requested_direction = (int8_t)direction;
    state->lift_active_direction = (int8_t)LIFT_INTERP_DIRECTION_HOLD;
    state->lift_running_spread_counts = 0;
    state->lift_max_running_spread_counts = 0;
    state->lift_preload_points_completed = 0U;
    state->lift_preload_group_pending = false;
    state->lift_starvation_candidate_samples = 0U;
    state->lift_starvation_recovery_attempts = 0U;
    state->lift_progress_initialized = false;
    state->lift_interpolation_state = LIFT_INTERPOLATION_STATE_CONFIGURING;
    lift_stream_reset_trajectory(state, now_ms);
    return true;
}

/* Fast path after HOME-center background preparation.  No SDO is issued here:
 * the buffer is already clear and every node is disabled in interpolation
 * mode.  The next state emits the synchronous 0x0007/0x000F RPDO1 groups. */
static bool begin_prepared_lift_enable(
    lift_hydraulic_device_state_t *state,
    lift_interpolation_direction_t direction,
    uint32_t now_ms)
{
    if (state == 0 || state->lift_group_in_flight ||
        !state->lift_prepared_disabled) {
        return false;
    }
    state->lift_prepared_disabled = false;
    state->lift_setup_enable_operation = true;
    state->lift_requested_direction = (int8_t)direction;
    state->lift_active_direction = (int8_t)LIFT_INTERP_DIRECTION_HOLD;
    state->lift_running_spread_counts = 0;
    state->lift_max_running_spread_counts = 0;
    state->lift_preload_points_completed = 0U;
    state->lift_preload_group_pending = false;
    state->lift_starvation_candidate_samples = 0U;
    state->lift_starvation_recovery_attempts = 0U;
    state->lift_start_with_leveling = false;
    state->lift_progress_initialized = false;
    lift_settle_reset(state);
    lift_stream_reset_trajectory(state, now_ms);
    state->lift_setup_deadline_ms =
        now_ms + ECU_CANOPEN_LIFT_SYNC_ENABLE_TIMEOUT_MS;
    state->lift_interpolation_state =
        LIFT_INTERPOLATION_STATE_READY_TO_SWITCH_ON;
    return true;
}

static bool lift_setup_completed(lift_hydraulic_device_state_t *state,
                                 canopen_master_service_t *canopen,
                                 const ecu_hardware_config_t *config,
                                 uint32_t now_ms)
{
    const uint32_t all_lift_axes_mask = (1UL << ECU_WHEEL_COUNT) - 1UL;

    if (state->lift_setup_nmt_sent_mask != all_lift_axes_mask) {
        for (uint32_t leg = 0U; leg < ECU_WHEEL_COUNT; ++leg) {
            const uint32_t leg_mask = 1UL << leg;
            if ((state->lift_setup_nmt_sent_mask & leg_mask) != 0U) {
                continue;
            }
            if (canopen_master_service_request_nmt(
                    canopen,
                    config->lift_nodes[leg].node_id,
                    CANOPEN_MASTER_DEBUG_COMMAND_NMT_OPERATIONAL)) {
                state->lift_setup_nmt_sent_mask |= leg_mask;
                state->lift_setup_request_mask =
                    state->lift_setup_nmt_sent_mask;
            }
            break;
        }
        if (time_reached(now_ms, state->lift_setup_deadline_ms)) {
            state->lift_interpolation_failure_count++;
            state->lift_interpolation_state = LIFT_INTERPOLATION_STATE_FAULT;
        }
        return false;
    }

    if (!state->lift_setup_sdo_queued) {
        if (!queue_lift_setup_sdos(state, canopen, config, now_ms) &&
            state->lift_interpolation_state == LIFT_INTERPOLATION_STATE_FAULT) {
            return false;
        }
        return false;
    }

    if (canopen->snapshot.sdo_download_abort_count !=
            state->lift_setup_abort_count_baseline) {
        state->lift_interpolation_failure_count++;
        state->lift_interpolation_state = LIFT_INTERPOLATION_STATE_FAULT;
        return false;
    }
    if (canopen->snapshot.sdo_download_count >=
            state->lift_setup_expected_download_count) {
        if (!state->lift_setup_enable_operation) {
            state->lift_prepared_disabled = true;
            state->lift_interpolation_state = LIFT_INTERPOLATION_STATE_STOPPED;
            return true;
        }
        state->lift_interpolation_state =
            LIFT_INTERPOLATION_STATE_READY_TO_SWITCH_ON;
        state->lift_setup_deadline_ms =
            now_ms + ECU_CANOPEN_LIFT_SYNC_ENABLE_TIMEOUT_MS;
        return true;
    }
    if (time_reached(now_ms, state->lift_setup_deadline_ms)) {
        state->lift_interpolation_failure_count++;
        state->lift_interpolation_state = LIFT_INTERPOLATION_STATE_FAULT;
        return false;
    }
    if (canopen->snapshot.sdo_download_count <
            state->lift_setup_expected_download_count) {
        return false;
    }
    return false;
}

/* Generate the same coherent four-axis trajectory used by the analyzer script.
 *
 * Field evidence from the 2026-07-11 full-stroke run:
 *   1. If the four axes are not level, first move them to one same-direction
 *      absolute target.
 *   2. Then stream one 20 ms absolute-position point per axis through RPDO2
 *      and one common SYNC.
 *   3. Keep a bounded lead over measured feedback and disable the drives after
 *      the final 10 mm or 490 mm target is reached.
 *
 * Do not compute a different trajectory step for each leg inside one CAN3
 * cycle.  The analyzer script samples the trajectory once per cycle and sends
 * the resulting coherent four-axis group immediately.
 */
static int32_t lift_interpolation_delta_counts(lift_hydraulic_device_state_t *state,
                                               lift_interpolation_direction_t direction,
                                               int32_t remaining_counts,
                                               int32_t speed_limit_counts_per_sec,
                                               uint32_t now_ms)
{
    int32_t step_counts =
        lift_interpolation_step_counts(state,
                                       direction,
                                       remaining_counts,
                                       speed_limit_counts_per_sec,
                                       now_ms);
    return step_counts;
}

/* Record four-axis synchronization quality without interrupting a healthy
 * interpolation stream.
 *
 * The running 10 mm threshold is deliberately diagnostic-only. Field tests
 * showed that disabling/reconfiguring the drives for a transient spread turns
 * a recoverable tracking difference into visible stop/restart jerking. The
 * normal per-axis bounded correction continues to pull the four axes together.
 * Final brake engagement remains guarded by the stricter 3 mm target/spread
 * check in lift_positions_at_target().
 */
static void update_lift_running_spread_diagnostics(
    lift_hydraulic_device_state_t *state)
{
    int32_t low;
    int32_t high;
    int32_t spread;

    if (state == 0) {
        return;
    }

    low = state->lift_actual_position_counts[0];
    high = low;
    for (uint32_t leg = 1U; leg < ECU_WHEEL_COUNT; ++leg) {
        const int32_t position = state->lift_actual_position_counts[leg];
        if (position < low) {
            low = position;
        }
        if (position > high) {
            high = position;
        }
    }

    spread = high - low;
    state->lift_running_spread_counts = spread;
    if (spread > state->lift_max_running_spread_counts) {
        state->lift_max_running_spread_counts = spread;
    }
    if (spread > ECU_LIFT_RUNNING_SPREAD_WARNING_COUNTS) {
        state->lift_running_spread_warning_count++;
    }
}

static int32_t lift_direction_sign(lift_interpolation_direction_t direction)
{
    return direction == LIFT_INTERP_DIRECTION_EXTEND ? -1 :
           direction == LIFT_INTERP_DIRECTION_RETRACT ? 1 : 0;
}

/* Choose the measured axis already furthest along the requested direction.
 *
 * The analyzer-proven stream first converges all axes to this position.  No
 * axis is asked to reverse merely to establish the common start, and every
 * later RPDO2 in the normal trajectory carries one identical absolute target.
 */
static int32_t lift_direction_leading_position(
    const lift_hydraulic_device_state_t *state,
    lift_interpolation_direction_t direction)
{
    int32_t target = state->lift_actual_position_counts[0];

    for (uint32_t leg = 1U; leg < ECU_WHEEL_COUNT; ++leg) {
        const int32_t position = state->lift_actual_position_counts[leg];
        if ((direction == LIFT_INTERP_DIRECTION_EXTEND && position < target) ||
            (direction == LIFT_INTERP_DIRECTION_RETRACT && position > target)) {
            target = position;
        }
    }
    return target;
}

static int32_t lift_common_following_error_counts(
    const lift_hydraulic_device_state_t *state,
    lift_interpolation_direction_t direction)
{
    const int32_t direction_sign = lift_direction_sign(direction);
    int64_t slowest_progress;
    int64_t target_progress;

    if (state == 0 || direction_sign == 0) {
        return 0;
    }

    slowest_progress =
        (int64_t)direction_sign * state->lift_actual_position_counts[0];
    for (uint32_t leg = 1U; leg < ECU_WHEEL_COUNT; ++leg) {
        const int64_t progress =
            (int64_t)direction_sign * state->lift_actual_position_counts[leg];
        if (progress < slowest_progress) {
            slowest_progress = progress;
        }
    }
    target_progress =
        (int64_t)direction_sign * state->lift_common_target_position_counts;
    if (target_progress <= slowest_progress) {
        return 0;
    }
    const int64_t error = target_progress - slowest_progress;
    return error > INT32_MAX ? INT32_MAX : (int32_t)error;
}

/* Reduce one common trajectory velocity as its lead approaches the configured
 * running following-error window.  This never changes relative leg targets:
 * all four axes still receive the same absolute point and therefore remain
 * synchronized by construction.
 */
static int32_t lift_feedback_governed_speed_counts_per_sec(
    const lift_hydraulic_device_state_t *state,
    lift_interpolation_direction_t direction)
{
    const int32_t following_error =
        lift_common_following_error_counts(state, direction);
    const int32_t normal_lead = ECU_LIFT_INTERPOLATION_TARGET_LEAD_COUNTS;
    const int32_t guard = ECU_LIFT_FOLLOWING_ERROR_WINDOW_COUNTS;

    if (following_error <= normal_lead || guard <= normal_lead) {
        return ECU_LIFT_INTERPOLATION_SPEED_COUNTS_PER_SEC;
    }
    if (following_error >= guard) {
        return ECU_LIFT_FEEDBACK_GOVERNOR_MIN_SPEED_COUNTS_PER_SEC;
    }

    const int64_t available_span = guard - normal_lead;
    const int64_t remaining_span = guard - following_error;
    const int64_t speed_span =
        ECU_LIFT_INTERPOLATION_SPEED_COUNTS_PER_SEC -
        ECU_LIFT_FEEDBACK_GOVERNOR_MIN_SPEED_COUNTS_PER_SEC;
    return ECU_LIFT_FEEDBACK_GOVERNOR_MIN_SPEED_COUNTS_PER_SEC +
           (int32_t)((speed_span * remaining_span) / available_span);
}

static bool lift_axes_near_zero_for_starvation(
    const lift_hydraulic_device_state_t *state)
{
    for (uint32_t leg = 0U; leg < ECU_WHEEL_COUNT; ++leg) {
        if (i32_abs(state->lift_actual_velocity_units[leg]) >
            ECU_LIFT_STARVATION_MAX_SPEED_VELOCITY_UNITS) {
            return false;
        }
    }
    return true;
}

static void lift_progress_watchdog_rebaseline(
    lift_hydraulic_device_state_t *state,
    uint32_t now_ms)
{
    for (uint32_t leg = 0U; leg < ECU_WHEEL_COUNT; ++leg) {
        state->lift_progress_position_counts[leg] =
            state->lift_actual_position_counts[leg];
        state->lift_progress_timestamp_ms[leg] = now_ms;
    }
    state->lift_progress_initialized = true;
}

static void lift_stream_capture_origin(lift_hydraulic_device_state_t *state)
{
    if (state == 0) {
        return;
    }

    for (uint32_t leg = 0U; leg < ECU_WHEEL_COUNT; ++leg) {
        state->lift_stream_origin_position_counts[leg] =
            state->lift_actual_position_counts[leg];
        state->lift_target_position_counts[leg] =
            state->lift_actual_position_counts[leg];
    }
    state->lift_common_target_position_counts =
        state->lift_actual_position_counts[0];
    state->lift_stream_planned_delta_counts = 0;
    state->lift_stream_total_distance_counts = 0;
}

/* Match the analyzer's post-enable stability gate before capturing the
 * interpolation origin.  Position drift is sampled at 100 ms intervals so a
 * brake-release transient or mechanical rebound cannot become the first
 * trajectory segment.  The function is nonblocking and advances only from the
 * CAN3 task.
 */
static bool lift_axes_settled(lift_hydraulic_device_state_t *state,
                              uint32_t now_ms)
{
    if (state == 0) {
        return false;
    }

    if (!state->lift_settle_initialized) {
        for (uint32_t leg = 0U; leg < ECU_WHEEL_COUNT; ++leg) {
            state->lift_settle_reference_position_counts[leg] =
                state->lift_actual_position_counts[leg];
        }
        state->lift_settle_sample_ms = now_ms;
        state->lift_settle_stable_since_ms = now_ms;
        state->lift_settle_initialized = true;
        return false;
    }

    if ((uint32_t)(now_ms - state->lift_settle_sample_ms) <
        ECU_CANOPEN_LIFT_SETTLE_SAMPLE_MS) {
        return false;
    }

    bool stable_sample = true;
    for (uint32_t leg = 0U; leg < ECU_WHEEL_COUNT; ++leg) {
        const int32_t position = state->lift_actual_position_counts[leg];
        const int32_t drift = position -
            state->lift_settle_reference_position_counts[leg];
        if (i32_abs(drift) > ECU_CANOPEN_LIFT_SETTLE_MAX_DRIFT_COUNTS) {
            stable_sample = false;
        }
        state->lift_settle_reference_position_counts[leg] = position;
    }
    state->lift_settle_sample_ms = now_ms;
    if (!stable_sample) {
        state->lift_settle_stable_since_ms = now_ms;
        return false;
    }

    return (uint32_t)(now_ms - state->lift_settle_stable_since_ms) >=
           ECU_CANOPEN_LIFT_SETTLE_STABLE_MS;
}

static void lift_stream_reset_trajectory(lift_hydraulic_device_state_t *state,
                                         uint32_t now_ms)
{
    if (state == 0) {
        return;
    }

    state->lift_targets_initialized = false;
    state->lift_stream_velocity_counts_per_sec = 0;
    state->lift_stream_planned_delta_counts = 0;
    state->lift_stream_total_distance_counts = 0;
    state->lift_target_stable_samples = 0U;
    state->lift_last_stream_step_ms = now_ms;
}

static int32_t lift_stream_total_distance(
    const lift_hydraulic_device_state_t *state,
    int32_t command_target_position_counts)
{
    int32_t distance = 0;

    if (state == 0) {
        return 0;
    }

    for (uint32_t leg = 0U; leg < ECU_WHEEL_COUNT; ++leg) {
        const int32_t leg_distance = i32_abs(
            command_target_position_counts -
            state->lift_stream_origin_position_counts[leg]);
        if (leg_distance > distance) {
            distance = leg_distance;
        }
    }
    return distance;
}

static void lift_stream_advance_send_deadline(lift_hydraulic_device_state_t *state,
                                              uint32_t now_ms)
{
    uint32_t scheduled_ms;

    if (state == 0) {
        return;
    }

    scheduled_ms = state->last_lift_interpolation_ms +
                   ECU_CANOPEN_LIFT_INTERPOLATION_REFRESH_MS;
    /* Preserve a 20 ms producer phase so short FreeRTOS/CAN jitter does not
     * permanently reduce the producer rate below the drive's 20 ms
     * interpolation-consumption rate.  If the ECU was blocked for more than one
     * extra period, re-baseline to now rather than sending a burst of stale
     * catch-up points.
     */
    if ((uint32_t)(now_ms - scheduled_ms) >
        ECU_CANOPEN_LIFT_INTERPOLATION_REFRESH_MS) {
        state->last_lift_interpolation_ms = now_ms;
    } else {
        state->last_lift_interpolation_ms = scheduled_ms;
    }
}

static bool lift_positions_at_target(const lift_hydraulic_device_state_t *state,
                                     int32_t target_position_counts)
{
    int32_t low = state->lift_actual_position_counts[0];
    int32_t high = state->lift_actual_position_counts[0];

    for (uint32_t leg = 0U; leg < ECU_WHEEL_COUNT; ++leg) {
        const int32_t position = state->lift_actual_position_counts[leg];
        const int32_t error = position - target_position_counts;
        if (i32_abs(error) > ECU_LIFT_TARGET_REACHED_TOLERANCE_COUNTS) {
            return false;
        }
        if (position < low) {
            low = position;
        }
        if (position > high) {
            high = position;
        }
    }

    return (high - low) <= ECU_LIFT_FINAL_SPREAD_TOLERANCE_COUNTS;
}

static bool lift_progress_stalled(lift_hydraulic_device_state_t *state,
                                  int32_t target_position_counts,
                                  uint32_t now_ms)
{
    if (!state->lift_progress_initialized) {
        for (uint32_t leg = 0U; leg < ECU_WHEEL_COUNT; ++leg) {
            state->lift_progress_position_counts[leg] =
                state->lift_actual_position_counts[leg];
            state->lift_progress_timestamp_ms[leg] = now_ms;
        }
        state->lift_progress_initialized = true;
        return false;
    }

    for (uint32_t leg = 0U; leg < ECU_WHEEL_COUNT; ++leg) {
        const int32_t position = state->lift_actual_position_counts[leg];
        const int32_t target_error = position - target_position_counts;
        const int32_t progress =
            position - state->lift_progress_position_counts[leg];

        if (i32_abs(target_error) <= ECU_LIFT_TARGET_REACHED_TOLERANCE_COUNTS) {
            state->lift_progress_position_counts[leg] = position;
            state->lift_progress_timestamp_ms[leg] = now_ms;
            continue;
        }
        if (i32_abs(progress) >= ECU_LIFT_STALL_PROGRESS_COUNTS) {
            state->lift_progress_position_counts[leg] = position;
            state->lift_progress_timestamp_ms[leg] = now_ms;
            continue;
        }
        if ((uint32_t)(now_ms - state->lift_progress_timestamp_ms[leg]) >=
            ECU_LIFT_STALL_TIMEOUT_MS) {
            return true;
        }
    }

    return false;
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

static bool build_lift_interpolation_trigger(
    canopen_master_pdo_request_t *request,
    const ecu_canopen_node_config_t *node,
    int32_t current_position_counts,
    uint16_t controlword,
    uint32_t group_sequence)
{
    canopen_node_pdo_profile_t profile;

    if (request == 0 || node == 0 ||
        !canopen_pdo_profile_init(node->node_id,
                                  CANOPEN_AXIS_ROLE_LIFT_POSITION,
                                  &profile)) {
        return false;
    }

    memset(request, 0, sizeof(*request));
    request->cob_id = profile.rpdo1_cob_id;
    request->size = 7U;
    request->node_id = node->node_id;
    request->group_sequence = group_sequence;
    request->phase = CANOPEN_MASTER_PDO_PHASE_LIFT_INTERPOLATION_TRIGGER;
    request->data[0] = (uint8_t)(controlword & 0xFFU);
    request->data[1] =
        (uint8_t)((controlword >> 8) & 0xFFU);
    request->data[2] = (uint8_t)CANOPEN_PDO_MODE_INTERPOLATED_POSITION;
    request->data[3] = (uint8_t)((uint32_t)current_position_counts & 0xFFU);
    request->data[4] =
        (uint8_t)(((uint32_t)current_position_counts >> 8) & 0xFFU);
    request->data[5] =
        (uint8_t)(((uint32_t)current_position_counts >> 16) & 0xFFU);
    request->data[6] =
        (uint8_t)(((uint32_t)current_position_counts >> 24) & 0xFFU);
    return true;
}

static bool lift_group_completed(lift_hydraulic_device_state_t *state,
                                 canopen_master_service_t *canopen)
{
    if (!state->lift_group_in_flight) {
        return true;
    }
    canopen_master_pdo_group_result_t result =
        canopen_master_service_pdo_group_result(
            canopen, state->lift_active_group_sequence);
    if (result == CANOPEN_MASTER_PDO_GROUP_RESULT_PENDING) {
        return false;
    }
    state->lift_group_in_flight = false;
    if (result != CANOPEN_MASTER_PDO_GROUP_RESULT_COMPLETE) {
        state->lift_interpolation_failure_count++;
        state->lift_transport_recovery_required = true;
        state->lift_interpolation_state = LIFT_INTERPOLATION_STATE_FAULT;
        return false;
    }
    return true;
}

static bool queue_lift_interpolation_trigger(
    lift_hydraulic_device_state_t *state,
    canopen_master_service_t *canopen,
    const ecu_hardware_config_t *config,
    uint16_t controlword,
    lift_interpolation_state_t queued_state);

static void reset_lift_motion_state(lift_hydraulic_device_state_t *state,
                                    uint32_t now_ms)
{
    state->lift_group_in_flight = false;
    state->lift_active_group_sequence = 0U;
    state->lift_setup_sdo_queued = false;
    state->lift_setup_nmt_sent_mask = 0U;
    state->lift_setup_expected_download_count = 0U;
    state->lift_setup_abort_count_baseline = 0U;
    state->lift_progress_initialized = false;
    state->lift_feedback_missing_since_ms = 0U;
    state->lift_active_direction = (int8_t)LIFT_INTERP_DIRECTION_HOLD;
    state->lift_running_spread_counts = 0;
    state->lift_common_target_position_counts = 0;
    state->lift_preload_points_completed = 0U;
    state->lift_preload_group_pending = false;
    state->lift_starvation_candidate_samples = 0U;
    state->lift_starvation_recovery_attempts = 0U;
    state->lift_at_target_disabled = false;
    state->lift_transport_recovery_required = false;
    state->lift_level_target_valid = false;
    state->lift_start_with_leveling = false;
    state->lift_level_stable_samples = 0U;
    state->lift_level_alignment_sample_count = 0U;
    state->lift_level_alignment_sample_index = 0U;
    state->lift_progress_initialized = false;
    state->lift_level_resume_direction =
        (int8_t)LIFT_INTERP_DIRECTION_HOLD;
    state->lift_prepared_disabled = false;
    for (uint32_t leg = 0U; leg < ECU_WHEEL_COUNT; ++leg) {
        state->lift_level_velocity_counts_per_sec[leg] = 0;
    }
    lift_settle_reset(state);
    state->last_lift_interpolation_ms = now_ms;
    lift_stream_reset_trajectory(state, now_ms);
    state->lift_interpolation_state = LIFT_INTERPOLATION_STATE_STOPPED;
}

/* A drive-local SDO/setup failure does not justify reinitializing the CAN
 * controller.  Let any already queued SDOs drain, then retry the setup from a
 * fresh measured origin after a bounded backoff. */
static void recover_lift_control_state(lift_hydraulic_device_state_t *state,
                                       uint32_t now_ms)
{
    if (state == 0) {
        return;
    }
    reset_lift_motion_state(state, now_ms);
    state->lift_recovery_not_before_ms =
        now_ms + ECU_LIFT_RECOVERY_BACKOFF_MS;
    state->lift_interpolation_recovery_count++;
}

static void recover_lift_transport_state(lift_hydraulic_device_state_t *state,
                                         canopen_master_service_t *canopen,
                                         uint32_t now_ms)
{
    bool recovery_attempted = false;

    if (state == 0) {
        return;
    }

    if (canopen != 0 &&
        (canopen->snapshot.can_normal ||
         time_reached(now_ms, canopen->next_transport_recovery_ms))) {
        canopen_master_service_cancel_realtime_pdo(canopen);
        (void)canopen_master_service_recover_transport(canopen);
        recovery_attempted = true;
    }

    state->pump_group_in_flight = false;
    state->pump_active_group_sequence = 0U;
    state->last_pump_velocity_command_valid = false;
    state->pump_velocity_setup_ready = false;
    state->pump_setup_in_flight = false;
    state->pump_feedback_valid = false;
    state->pump_pressure_ready = false;
    state->pump_speed_ready_samples = 0U;
    state->last_valve_mask = 0U;
    reset_lift_motion_state(state, now_ms);
    if (recovery_attempted) {
        state->lift_recovery_not_before_ms =
            now_ms + ECU_CANOPEN_TRANSPORT_RECOVERY_BACKOFF_MS;
        state->lift_interpolation_recovery_count++;
    }
}

static bool submit_lift_target_group(lift_hydraulic_device_state_t *state,
                                     canopen_master_service_t *canopen,
                                     const ecu_hardware_config_t *config,
                                     const int32_t target_positions[ECU_WHEEL_COUNT],
                                     uint32_t now_ms)
{
    canopen_master_pdo_request_t requests[ECU_WHEEL_COUNT];
    const uint32_t group_sequence = next_lift_group_sequence(state);

    for (uint32_t leg = 0U; leg < ECU_WHEEL_COUNT; ++leg) {
        if (!build_lift_interpolation_request(&requests[leg],
                                              &config->lift_nodes[leg],
                                              target_positions[leg],
                                              group_sequence)) {
            state->lift_interpolation_reject_count++;
            state->lift_interpolation_state = LIFT_INTERPOLATION_STATE_FAULT;
            return false;
        }
    }

    const canopen_master_pdo_group_descriptor_t descriptor = {
        .expected_frames = ECU_WHEEL_COUNT,
        .arm_frame_count = ECU_WHEEL_COUNT,
        .trigger_frame_count = 0U,
        .axis_mask = (uint8_t)((1U << ECU_WHEEL_COUNT) - 1U),
        .position_group = true,
        .sync_after_arm = true,
        .sync_after_trigger = false
    };
    if (!canopen_master_service_queue_pdo_batch_with_descriptor(
            canopen, requests, ECU_WHEEL_COUNT, &descriptor)) {
        state->lift_interpolation_failure_count++;
        state->lift_interpolation_state = LIFT_INTERPOLATION_STATE_FAULT;
        return false;
    }

    for (uint32_t leg = 0U; leg < ECU_WHEEL_COUNT; ++leg) {
        state->lift_target_position_counts[leg] = target_positions[leg];
    }
    state->lift_targets_initialized = true;
    state->lift_active_group_sequence = group_sequence;
    state->lift_group_in_flight = true;
    state->lift_interpolation_queued_count++;
    lift_stream_advance_send_deadline(state, now_ms);
    return true;
}

static bool queue_lift_interpolation_group(lift_hydraulic_device_state_t *state,
                                           canopen_master_service_t *canopen,
                                           const ecu_hardware_config_t *config,
                                           lift_interpolation_direction_t direction,
                                           int32_t command_target_position_counts,
                                           uint32_t now_ms)
{
    int32_t target_positions[ECU_WHEEL_COUNT];

    if ((uint32_t)(now_ms - state->last_lift_interpolation_ms) <
        ECU_CANOPEN_LIFT_INTERPOLATION_REFRESH_MS) {
        return true;
    }

    if (!lift_group_completed(state, canopen)) {
        return state->lift_interpolation_state != LIFT_INTERPOLATION_STATE_FAULT;
    }
    if (!canopen_master_service_realtime_pdo_idle(canopen)) {
        return true;
    }

    if (!refresh_lift_feedback(state, canopen, config, now_ms)) {
        state->lift_interpolation_reject_count++;
        state->lift_at_target_disabled = false;
        if (state->lift_axis_fault_mask != 0U) {
            /* A vendor-latched axis fault pauses the coherent stream and uses
             * the normal STOPPING -> setup path.  Do not reset the CAN
             * transport for a drive-local fault: after the disable group the
             * setup SDOs clear the vendor latch/controlword fault and re-enable
             * all four axes from newly measured positions. */
            state->lift_target_stable_samples = 0U;
            state->lift_progress_initialized = false;
            state->lift_feedback_missing_since_ms = 0U;
            state->lift_interpolation_recovery_count++;
            return queue_lift_interpolation_trigger(
                state,
                canopen,
                config,
                SERVO_DRIVE_CONTROL_DISABLE_VOLTAGE,
                LIFT_INTERPOLATION_STATE_STOPPING);
        }
        if (state->lift_feedback_missing_since_ms == 0U) {
            state->lift_feedback_missing_since_ms = now_ms;
        }
        if ((uint32_t)(now_ms - state->lift_feedback_missing_since_ms) >=
            ECU_CANOPEN_LIFT_SETUP_TIMEOUT_MS) {
            state->lift_interpolation_failure_count++;
            recover_lift_transport_state(state, canopen, now_ms);
            state->lift_recovery_not_before_ms =
                now_ms + ECU_LIFT_RECOVERY_BACKOFF_MS;
        }
        return true;
    }

    update_lift_position_range_masks(state);
    /* PRELOADING deliberately queues stationary points at each axis's freshly
     * measured position, so its direction is HOLD.  Those points cannot move a
     * leg farther out of range and must be allowed to fill the BC2
     * interpolation buffers before the common trigger edge.  The previous
     * unconditional direction gate rejected every HOLD preload, immediately
     * disabled all four axes, and restarted setup forever.  Keep the range gate
     * mandatory for every real extend/retract trajectory point.
     */
    if (direction != LIFT_INTERP_DIRECTION_HOLD &&
        !lift_direction_allowed_by_range(state, direction)) {
        state->lift_range_direction_reject_count++;
        return queue_lift_interpolation_trigger(
            state,
            canopen,
            config,
            SERVO_DRIVE_CONTROL_DISABLE_VOLTAGE,
            LIFT_INTERPOLATION_STATE_STOPPING);
    }
    update_lift_running_spread_diagnostics(state);

    if (lift_positions_at_target(state, command_target_position_counts)) {
        /* Keep streaming the final absolute point until measured feedback has
         * remained inside the 3 mm position/spread window for several complete
         * interpolation periods.  A one-sample decision can disable the axes
         * while BC2 still holds earlier buffered points, which produces the
         * short move / brake / restart symptom seen on the vehicle. */
        if (state->lift_target_stable_samples <
                ECU_CANOPEN_LIFT_TARGET_STABLE_SAMPLES) {
            state->lift_target_stable_samples++;
        }
        if (state->lift_target_stable_samples >=
                ECU_CANOPEN_LIFT_TARGET_STABLE_SAMPLES) {
            lift_stream_reset_trajectory(state, now_ms);
            state->lift_progress_initialized = false;
            state->lift_at_target_disabled = true;
            return queue_lift_interpolation_trigger(
                state,
                canopen,
                config,
                SERVO_DRIVE_CONTROL_DISABLE_VOLTAGE,
                LIFT_INTERPOLATION_STATE_STOPPING);
        }
    } else {
        state->lift_target_stable_samples = 0U;
    }
    if (lift_progress_stalled(state, command_target_position_counts, now_ms)) {
        /* A short lift-progress stall is not a drive fault by itself.
         *
         * Field tests showed that treating this condition as STOPPING causes a
         * visible jerk: all four lift axes are disabled, the interpolation
         * setup is rebuilt, and then motion restarts.  The analyzer-proven
         * method keeps all four axes on the same absolute-position stream and
         * only re-baselines the lead from measured feedback.
         *
         * Therefore a progress stall only re-baselines the progress watchdog.
         * The trajectory origin and last queued target must stay unchanged:
         * replacing them while the BC2 FIFO still contains older points creates
         * a discontinuity and was one direct cause of the observed jerking.
         * Hard recovery remains reserved for real transport loss, missing
         * feedback beyond timeout, or CANopen group failure.
         */
        state->lift_interpolation_reject_count++;
        lift_progress_watchdog_rebaseline(state, now_ms);
        state->lift_at_target_disabled = false;
    }

    if (!state->lift_targets_initialized) {
        lift_stream_capture_origin(state);
        state->lift_stream_total_distance_counts =
            lift_stream_total_distance(state,
                                       command_target_position_counts);
    }

    if (direction != LIFT_INTERP_DIRECTION_HOLD) {
        const int32_t following_error =
            lift_common_following_error_counts(state, direction);
        if (following_error > ECU_LIFT_INTERPOLATION_TARGET_LEAD_COUNTS &&
            lift_axes_near_zero_for_starvation(state)) {
            if (state->lift_starvation_candidate_samples <
                ECU_LIFT_STARVATION_CONFIRM_SAMPLES) {
                state->lift_starvation_candidate_samples++;
            }
        } else {
            state->lift_starvation_candidate_samples = 0U;
            if (following_error <=
                    ECU_LIFT_INTERPOLATION_TARGET_LEAD_COUNTS ||
                !lift_axes_near_zero_for_starvation(state)) {
                state->lift_starvation_recovery_attempts = 0U;
            }
        }
        if (state->lift_starvation_candidate_samples >=
                ECU_LIFT_STARVATION_CONFIRM_SAMPLES &&
            time_reached(now_ms, state->lift_recovery_not_before_ms)) {
            if (state->lift_starvation_recovery_attempts >=
                    ECU_LIFT_STARVATION_RECOVERY_LIMIT) {
                /* Repeated starvation is still recoverable.  Rate-limit the
                 * next re-prime instead of converting it into an axis disable
                 * or transport reset. */
                state->lift_starvation_recovery_attempts = 0U;
                state->lift_recovery_not_before_ms =
                    now_ms + ECU_LIFT_RECOVERY_BACKOFF_MS;
                state->lift_starvation_candidate_samples = 0U;
                return true;
            }
            state->lift_starvation_recovery_attempts++;
            state->lift_starvation_recovery_count++;
            state->lift_interpolation_recovery_count++;
            state->lift_starvation_candidate_samples = 0U;
            state->lift_level_resume_direction = (int8_t)direction;
            state->lift_start_with_leveling = true;
            return queue_lift_interpolation_trigger(
                state,
                canopen,
                config,
                SERVO_DRIVE_CONTROL_ENABLE_OPERATION,
                LIFT_INTERPOLATION_STATE_STARVATION_CLEARING);
        }

        const int32_t remaining_counts =
            state->lift_stream_total_distance_counts -
            state->lift_stream_planned_delta_counts;
        const int32_t speed_limit_counts_per_sec =
            lift_feedback_governed_speed_counts_per_sec(state, direction);
        const int32_t delta_counts =
            lift_interpolation_delta_counts(state,
                                            direction,
                                            remaining_counts > 0 ?
                                                remaining_counts : 0,
                                            speed_limit_counts_per_sec,
                                            now_ms);
        int64_t planned_delta =
            (int64_t)state->lift_stream_planned_delta_counts +
            (int64_t)delta_counts;
        if (planned_delta < 0) {
            planned_delta = 0;
        }
        if (planned_delta > state->lift_stream_total_distance_counts) {
            planned_delta = state->lift_stream_total_distance_counts;
        }
        state->lift_stream_planned_delta_counts = (int32_t)planned_delta;
    } else {
        state->lift_stream_velocity_counts_per_sec = 0;
        state->lift_last_stream_step_ms = now_ms;
        state->lift_stream_planned_delta_counts = 0;
        /* Preload/re-prime points must hold each axis at its own fresh measured
         * absolute position.  The common position is introduced only after the
         * shared 0x003F trigger, through the smooth LEVELING stage. */
        for (uint32_t leg = 0U; leg < ECU_WHEEL_COUNT; ++leg) {
            target_positions[leg] =
                state->lift_stream_origin_position_counts[leg];
        }
        return submit_lift_target_group(state,
                                        canopen,
                                        config,
                                        target_positions,
                                        now_ms);
    }

    const int32_t direction_sign = lift_direction_sign(direction);
    const int32_t common_origin =
        state->lift_stream_origin_position_counts[0];
    int64_t common_target =
        (int64_t)common_origin +
        (int64_t)direction_sign *
            state->lift_stream_planned_delta_counts;
    if (direction == LIFT_INTERP_DIRECTION_EXTEND &&
        common_target < command_target_position_counts) {
        common_target = command_target_position_counts;
    } else if (direction == LIFT_INTERP_DIRECTION_RETRACT &&
               common_target > command_target_position_counts) {
        common_target = command_target_position_counts;
    }
    state->lift_common_target_position_counts =
        clamp_lift_position_counts((int32_t)common_target);
    for (uint32_t leg = 0U; leg < ECU_WHEEL_COUNT; ++leg) {
        target_positions[leg] =
            state->lift_common_target_position_counts;
    }
    return submit_lift_target_group(state,
                                    canopen,
                                    config,
                                    target_positions,
                                    now_ms);
}

static void begin_lift_leveling(lift_hydraulic_device_state_t *state,
                                lift_interpolation_direction_t resume_direction)
{
    int32_t maximum_alignment_distance = 0;

    if (state == 0) {
        return;
    }
    state->lift_level_target_position_counts =
        resume_direction == LIFT_INTERP_DIRECTION_HOLD ?
            lift_level_target_from_feedback(state) :
            lift_direction_leading_position(state, resume_direction);
    state->lift_level_target_valid = true;
    state->lift_level_stable_samples = 0U;
    state->lift_level_resume_direction = (int8_t)resume_direction;
    state->lift_level_alignment_sample_count = 0U;
    state->lift_level_alignment_sample_index = 0U;
    state->lift_leveling_entry_count++;

    const int32_t signed_stream_velocity =
        state->lift_active_direction == (int8_t)LIFT_INTERP_DIRECTION_EXTEND ?
            -state->lift_stream_velocity_counts_per_sec :
        state->lift_active_direction == (int8_t)LIFT_INTERP_DIRECTION_RETRACT ?
            state->lift_stream_velocity_counts_per_sec : 0;
    for (uint32_t leg = 0U; leg < ECU_WHEEL_COUNT; ++leg) {
        const int32_t distance = i32_abs(
            state->lift_level_target_position_counts -
            state->lift_actual_position_counts[leg]);
        state->lift_level_origin_position_counts[leg] =
            state->lift_actual_position_counts[leg];
        state->lift_level_velocity_counts_per_sec[leg] =
            signed_stream_velocity;
        if (distance > maximum_alignment_distance) {
            maximum_alignment_distance = distance;
        }
        if (!state->lift_targets_initialized) {
            state->lift_target_position_counts[leg] =
                state->lift_actual_position_counts[leg];
        }
    }
    if (resume_direction != LIFT_INTERP_DIRECTION_HOLD &&
        maximum_alignment_distance > 0) {
        /* Match the analyzer script's cubic pre-alignment.  Cubic smoothstep
         * has a peak normalized slope of 1.5, so size the fixed sample count
         * from that peak and the configured 2 mm/s leveling speed. */
        const int64_t numerator =
            3LL * maximum_alignment_distance * 1000LL;
        const int64_t denominator =
            2LL * ECU_LIFT_LEVELING_SPEED_COUNTS_PER_SEC *
            ECU_CANOPEN_LIFT_INTERPOLATION_REFRESH_MS;
        uint32_t samples = denominator > 0 ?
            (uint32_t)((numerator + denominator - 1LL) / denominator) : 0U;
        if (samples < ECU_CANOPEN_LIFT_PRELOAD_POINTS) {
            samples = ECU_CANOPEN_LIFT_PRELOAD_POINTS;
        }
        state->lift_level_alignment_sample_count = samples;
    }
    state->lift_targets_initialized = true;
    state->lift_interpolation_state = LIFT_INTERPOLATION_STATE_LEVELING;
}

/* Return one analyzer-equivalent cubic smoothstep pre-alignment point.
 *
 * The origins and common endpoint are fixed when leveling begins.  No measured
 * feedback is fed back into the point generator, so a responsive axis cannot
 * be commanded alternately to opposite sides of the endpoint while another
 * axis catches up.
 */
static int32_t lift_directional_alignment_target(
    const lift_hydraulic_device_state_t *state,
    uint32_t leg)
{
    const uint32_t count = state->lift_level_alignment_sample_count;
    const uint32_t index = state->lift_level_alignment_sample_index + 1U;
    const int32_t origin = state->lift_level_origin_position_counts[leg];
    const int32_t distance = state->lift_level_target_position_counts - origin;

    if (count == 0U || index >= count) {
        return state->lift_level_target_position_counts;
    }

    /* smoothstep(i/n) = i^2 * (3n - 2i) / n^3 */
    const int64_t numerator =
        (int64_t)index * index * (3LL * count - 2LL * index);
    const int64_t denominator = (int64_t)count * count * count;
    const int64_t offset =
        ((int64_t)distance * numerator) / denominator;
    return origin + (int32_t)offset;
}

static int32_t approach_zero(int32_t value, int32_t step)
{
    if (value > step) {
        return value - step;
    }
    if (value < -step) {
        return value + step;
    }
    return 0;
}

/* Plan one continuous per-axis leveling point.  Starting from the last queued
 * target, rather than measured position, preserves continuity with the BC2
 * interpolation buffer.  An axis that must reverse first ramps its planned
 * velocity through zero, so neutral cannot create a target-position jump. */
static int32_t lift_level_next_target(lift_hydraulic_device_state_t *state,
                                      uint32_t leg)
{
    const int32_t period_ms = ECU_CANOPEN_LIFT_INTERPOLATION_REFRESH_MS;
    const int32_t acceleration_step = (int32_t)(
        ((int64_t)ECU_LIFT_LEVELING_ACCEL_COUNTS_PER_SEC2 * period_ms) /
        1000);
    const int32_t previous = state->lift_target_position_counts[leg];
    const int32_t error = state->lift_level_target_position_counts - previous;
    int32_t velocity = state->lift_level_velocity_counts_per_sec[leg];
    const int32_t error_sign = error > 0 ? 1 : (error < 0 ? -1 : 0);
    const int32_t velocity_sign = velocity > 0 ? 1 : (velocity < 0 ? -1 : 0);
    const int64_t braking_distance =
        acceleration_step > 0 ?
            ((int64_t)velocity * velocity) /
                (2LL * ECU_LIFT_LEVELING_ACCEL_COUNTS_PER_SEC2) : 0;

    if (error_sign == 0 ||
        (velocity_sign != 0 && velocity_sign != error_sign) ||
        braking_distance >= i32_abs(error)) {
        velocity = approach_zero(velocity, acceleration_step);
    } else {
        int64_t accelerated = (int64_t)velocity +
                              (int64_t)error_sign * acceleration_step;
        accelerated = clamp_i32((int32_t)accelerated,
                                -ECU_LIFT_LEVELING_SPEED_COUNTS_PER_SEC,
                                ECU_LIFT_LEVELING_SPEED_COUNTS_PER_SEC);
        velocity = (int32_t)accelerated;
    }

    int64_t next = (int64_t)previous +
                   ((int64_t)velocity * period_ms) / 1000;
    if ((error > 0 && next >= state->lift_level_target_position_counts) ||
        (error < 0 && next <= state->lift_level_target_position_counts)) {
        next = state->lift_level_target_position_counts;
        velocity = 0;
    }
    next = clamp_i32((int32_t)next,
                     lift_unclamped_counts_from_mm(
                         ECU_REMOTE_MAX_HEIGHT_TARGET_MM),
                     lift_unclamped_counts_from_mm(
                         ECU_REMOTE_MIN_HEIGHT_TARGET_MM));
    if ((int32_t)next == previous && error != 0) {
        velocity = 0;
    }
    state->lift_level_velocity_counts_per_sec[leg] = velocity;
    return (int32_t)next;
}

static bool lift_level_feedback_stable(
    const lift_hydraulic_device_state_t *state)
{
    int32_t low = state->lift_actual_position_counts[0];
    int32_t high = low;

    for (uint32_t leg = 0U; leg < ECU_WHEEL_COUNT; ++leg) {
        const int32_t position = state->lift_actual_position_counts[leg];
        if (i32_abs(position - state->lift_level_target_position_counts) >
            ECU_LIFT_TARGET_REACHED_TOLERANCE_COUNTS) {
            return false;
        }
        if (i32_abs(state->lift_actual_velocity_units[leg]) >
            ECU_LIFT_ZERO_SPEED_VELOCITY_UNITS) {
            return false;
        }
        if (position < low) {
            low = position;
        }
        if (position > high) {
            high = position;
        }
    }
    return (high - low) <= ECU_LIFT_FINAL_SPREAD_TOLERANCE_COUNTS;
}

static void resume_lift_stream_after_leveling(
    lift_hydraulic_device_state_t *state,
    int32_t command_target_position_counts,
    lift_interpolation_direction_t direction,
    uint32_t now_ms)
{
    for (uint32_t leg = 0U; leg < ECU_WHEEL_COUNT; ++leg) {
        state->lift_stream_origin_position_counts[leg] =
            state->lift_level_target_position_counts;
        state->lift_target_position_counts[leg] =
            state->lift_level_target_position_counts;
    }
    state->lift_common_target_position_counts =
        state->lift_level_target_position_counts;
    state->lift_stream_planned_delta_counts = 0;
    state->lift_stream_velocity_counts_per_sec = 0;
    state->lift_stream_total_distance_counts =
        lift_stream_total_distance(state, command_target_position_counts);
    state->lift_last_stream_step_ms = now_ms;
    state->lift_level_target_valid = false;
    state->lift_level_stable_samples = 0U;
    state->lift_progress_initialized = false;
    state->lift_target_stable_samples = 0U;
    state->lift_starvation_candidate_samples = 0U;
    state->lift_level_resume_direction = (int8_t)LIFT_INTERP_DIRECTION_HOLD;
    state->lift_requested_direction = (int8_t)direction;
    state->lift_active_direction = (int8_t)direction;
    state->lift_command_target_position_counts = command_target_position_counts;
    state->lift_interpolation_state = LIFT_INTERPOLATION_STATE_RUNNING;
}

static bool queue_lift_leveling_group(lift_hydraulic_device_state_t *state,
                                      canopen_master_service_t *canopen,
                                      const ecu_hardware_config_t *config,
                                      int32_t command_target_position_counts,
                                      uint32_t now_ms)
{
    int32_t targets[ECU_WHEEL_COUNT];

    if ((uint32_t)(now_ms - state->last_lift_interpolation_ms) <
        ECU_CANOPEN_LIFT_INTERPOLATION_REFRESH_MS) {
        return true;
    }
    if (!lift_group_completed(state, canopen)) {
        return state->lift_interpolation_state != LIFT_INTERPOLATION_STATE_FAULT;
    }
    const lift_interpolation_direction_t resume_direction =
        (lift_interpolation_direction_t)state->lift_level_resume_direction;
    if (resume_direction != LIFT_INTERP_DIRECTION_HOLD &&
        state->lift_level_alignment_sample_index >=
            state->lift_level_alignment_sample_count) {
        resume_lift_stream_after_leveling(state,
                                          command_target_position_counts,
                                          resume_direction,
                                          now_ms);
        return true;
    }
    if (!canopen_master_service_realtime_pdo_idle(canopen)) {
        return true;
    }
    if (!refresh_lift_feedback(state, canopen, config, now_ms)) {
        state->lift_interpolation_reject_count++;
        return true;
    }
    update_lift_position_range_masks(state);
    if (state->lift_mechanical_range_invalid_mask != 0U) {
        state->lift_interpolation_failure_count++;
        return queue_lift_interpolation_trigger(
            state,
            canopen,
            config,
            SERVO_DRIVE_CONTROL_DISABLE_VOLTAGE,
            LIFT_INTERPOLATION_STATE_STOPPING);
    }
    if (resume_direction != LIFT_INTERP_DIRECTION_HOLD) {
        for (uint32_t leg = 0U; leg < ECU_WHEEL_COUNT; ++leg) {
            targets[leg] = lift_directional_alignment_target(state, leg);
        }
        if (!submit_lift_target_group(state,
                                      canopen,
                                      config,
                                      targets,
                                      now_ms)) {
            return false;
        }
        if (state->lift_group_in_flight) {
            state->lift_level_alignment_sample_index++;
        }
        return true;
    }
    if (lift_progress_stalled(state,
                              state->lift_level_target_position_counts,
                              now_ms)) {
        /* A loaded axis may pause briefly while the other axes converge.
         * Preserve the continuous common interpolation window and rebaseline
         * only the progress observer; disabling here would lock in an uneven
         * height and make the next operator request start from a worse state. */
        state->lift_interpolation_reject_count++;
        lift_progress_watchdog_rebaseline(state, now_ms);
    }

    if (lift_level_feedback_stable(state)) {
        if (state->lift_level_stable_samples <
                ECU_LIFT_LEVELING_STABLE_SAMPLES) {
            state->lift_level_stable_samples++;
        }
    } else {
        state->lift_level_stable_samples = 0U;
    }
    if (state->lift_level_stable_samples >=
            ECU_LIFT_LEVELING_STABLE_SAMPLES) {
        state->lift_leveling_complete_count++;
        if (resume_direction != LIFT_INTERP_DIRECTION_HOLD &&
            lift_direction_allowed_by_range(state, resume_direction)) {
            resume_lift_stream_after_leveling(state,
                                              command_target_position_counts,
                                              resume_direction,
                                              now_ms);
            return true;
        }
        state->lift_at_target_disabled = true;
        return queue_lift_interpolation_trigger(
            state,
            canopen,
            config,
            SERVO_DRIVE_CONTROL_DISABLE_VOLTAGE,
            LIFT_INTERPOLATION_STATE_STOPPING);
    }

    for (uint32_t leg = 0U; leg < ECU_WHEEL_COUNT; ++leg) {
        targets[leg] = lift_level_next_target(state, leg);
    }
    return submit_lift_target_group(state, canopen, config, targets, now_ms);
}

static bool queue_lift_interpolation_trigger(
    lift_hydraulic_device_state_t *state,
    canopen_master_service_t *canopen,
    const ecu_hardware_config_t *config,
    uint16_t controlword,
    lift_interpolation_state_t queued_state)
{
    canopen_master_pdo_request_t requests[ECU_WHEEL_COUNT];
    uint32_t group_sequence;

    if (!lift_group_completed(state, canopen)) {
        return state->lift_interpolation_state != LIFT_INTERPOLATION_STATE_FAULT;
    }
    if (!canopen_master_service_realtime_pdo_idle(canopen)) {
        return true;
    }
    group_sequence = next_lift_group_sequence(state);
    for (uint32_t leg = 0U; leg < ECU_WHEEL_COUNT; ++leg) {
        if (!build_lift_interpolation_trigger(
                &requests[leg],
                &config->lift_nodes[leg],
                state->lift_actual_position_counts[leg],
                controlword,
                group_sequence)) {
            state->lift_interpolation_reject_count++;
            return false;
        }
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
            canopen, requests, ECU_WHEEL_COUNT, &descriptor)) {
        state->lift_interpolation_failure_count++;
        return false;
    }

    state->lift_active_group_sequence = group_sequence;
    state->lift_group_in_flight = true;
    state->lift_interpolation_state = queued_state;
    return true;
}

static bool process_lift_interpolation(
    lift_hydraulic_device_state_t *state,
    canopen_master_service_t *canopen,
    const ecu_hardware_config_t *config,
    lift_interpolation_direction_t requested_direction,
    float target_height_mm,
    vehicle_lift_request_t lift_request,
    bool high_voltage_feedback_ready,
    uint32_t now_ms)
{
    const bool safe_stop_requested =
        !high_voltage_feedback_ready ||
        lift_request == VEHICLE_LIFT_REQUEST_SAFE_STOP;
    const bool operator_neutral_requested =
        high_voltage_feedback_ready &&
        lift_request == VEHICLE_LIFT_REQUEST_NEUTRAL_STOP;
    const bool direction_matches_request =
        (lift_request == VEHICLE_LIFT_REQUEST_EXTEND &&
         requested_direction == LIFT_INTERP_DIRECTION_EXTEND) ||
        (lift_request == VEHICLE_LIFT_REQUEST_RETRACT &&
         requested_direction == LIFT_INTERP_DIRECTION_RETRACT);
    const lift_interpolation_direction_t desired_direction =
        !safe_stop_requested && direction_matches_request ?
            requested_direction : LIFT_INTERP_DIRECTION_HOLD;
    const int32_t command_target_position_counts =
        lift_target_counts_from_mm(target_height_mm);

    if (state->lift_interpolation_state ==
            LIFT_INTERPOLATION_STATE_STOPPING) {
        state->lift_requested_direction = (int8_t)desired_direction;
        if (!lift_group_completed(state, canopen)) {
            return state->lift_interpolation_state !=
                   LIFT_INTERPOLATION_STATE_FAULT;
        }
        lift_stream_reset_trajectory(state, now_ms);
        state->lift_progress_initialized = false;
        state->lift_active_direction =
            (int8_t)LIFT_INTERP_DIRECTION_HOLD;
        state->lift_interpolation_state = LIFT_INTERPOLATION_STATE_STOPPED;
        /* A normal synchronous disable does not invalidate the saved PDO
         * mapping or interpolation setup. Keep the drives in the prepared,
         * brake-held state so the next operator request can use the fast
         * four-axis RPDO enable path. Faulted or de-energized axes must take
         * the full setup path instead. */
        state->lift_prepared_disabled =
            high_voltage_feedback_ready &&
            state->lift_axis_fault_mask == 0U &&
            !state->lift_transport_recovery_required;
        state->lift_level_target_valid = false;
        state->lift_start_with_leveling = false;
        state->lift_level_resume_direction =
            (int8_t)LIFT_INTERP_DIRECTION_HOLD;
        state->lift_level_alignment_sample_count = 0U;
        state->lift_level_alignment_sample_index = 0U;
        bool same_target = state->lift_command_target_position_counts ==
                           command_target_position_counts;
        state->lift_command_target_position_counts =
            command_target_position_counts;
        if (desired_direction == LIFT_INTERP_DIRECTION_HOLD ||
            (state->lift_at_target_disabled && same_target) ||
            !time_reached(now_ms, state->lift_recovery_not_before_ms)) {
            return true;
        }
        state->lift_at_target_disabled = false;
        return begin_lift_interpolation_setup(
            state,
            canopen,
            config,
            desired_direction,
            high_voltage_feedback_ready,
            now_ms);
    }

    if (state->lift_interpolation_state ==
            LIFT_INTERPOLATION_STATE_STARVATION_CLEARING) {
        if (!lift_group_completed(state, canopen)) {
            return state->lift_interpolation_state !=
                   LIFT_INTERPOLATION_STATE_FAULT;
        }
        if (safe_stop_requested) {
            return queue_lift_interpolation_trigger(
                state,
                canopen,
                config,
                SERVO_DRIVE_CONTROL_DISABLE_VOLTAGE,
                LIFT_INTERPOLATION_STATE_STOPPING);
        }
        if (!refresh_lift_feedback(state, canopen, config, now_ms)) {
            return true;
        }

        /* Re-prime from fresh measured hold points.  No drive disable, NMT
         * reset, SDO reconfiguration or cached nonzero point is replayed. */
        lift_stream_reset_trajectory(state, now_ms);
        lift_stream_capture_origin(state);
        state->lift_preload_points_completed = 0U;
        state->lift_preload_group_pending = false;
        state->lift_active_direction =
            (int8_t)LIFT_INTERP_DIRECTION_HOLD;
        state->lift_requested_direction = (int8_t)desired_direction;
        state->lift_level_resume_direction = (int8_t)desired_direction;
        state->lift_start_with_leveling = true;
        state->lift_interpolation_state =
            LIFT_INTERPOLATION_STATE_PRELOADING;
        return true;
    }

    if (state->lift_interpolation_state ==
            LIFT_INTERPOLATION_STATE_LEVELING) {
        const lift_interpolation_direction_t alignment_direction =
            (lift_interpolation_direction_t)
                state->lift_level_resume_direction;
        /* Pre-alignment has a fixed origin, endpoint and direction. Never
         * rewrite it from live joystick samples: doing so made one axis chase
         * a moving endpoint and caused the observed random reversal/vibration.
         * Neutral, safety stop or a direction change cancels the complete
         * four-axis group and applies the brakes. A later request restarts from
         * fresh TPDO positions. */
        if (safe_stop_requested ||
            desired_direction == LIFT_INTERP_DIRECTION_HOLD ||
            desired_direction != alignment_direction) {
            state->lift_level_resume_direction =
                (int8_t)LIFT_INTERP_DIRECTION_HOLD;
            state->lift_at_target_disabled = false;
            if (!lift_group_completed(state, canopen)) {
                return state->lift_interpolation_state !=
                       LIFT_INTERPOLATION_STATE_FAULT;
            }
            return queue_lift_interpolation_trigger(
                state,
                canopen,
                config,
                SERVO_DRIVE_CONTROL_DISABLE_VOLTAGE,
                LIFT_INTERPOLATION_STATE_STOPPING);
        }
        return queue_lift_leveling_group(state,
                                         canopen,
                                         config,
                                         command_target_position_counts,
                                         now_ms);
    }

    if (safe_stop_requested || operator_neutral_requested ||
        desired_direction == LIFT_INTERP_DIRECTION_HOLD) {
        state->lift_hold_count++;
        state->lift_requested_direction = (int8_t)LIFT_INTERP_DIRECTION_HOLD;
        state->lift_at_target_disabled = false;
        if (state->lift_interpolation_state ==
                LIFT_INTERPOLATION_STATE_CONFIGURING) {
            const bool setup_was_enabling =
                state->lift_setup_enable_operation;
            if (lift_setup_completed(state, canopen, config, now_ms)) {
                lift_stream_reset_trajectory(state, now_ms);
                state->lift_progress_initialized = false;
                state->lift_active_direction =
                    (int8_t)LIFT_INTERP_DIRECTION_HOLD;
                if (setup_was_enabling) {
                    /* The operator returned the stick to neutral while the
                     * enable/configuration SDOs were still in flight.  Do not
                     * leave the four lift drives Operation Enabled in a
                     * nominal STOPPED state; immediately run the same
                     * non-reset disable setup used after normal target reach,
                     * so the drive brakes can hold the legs and motor heat is
                     * minimized.
                     */
                    state->lift_interpolation_state =
                        LIFT_INTERPOLATION_STATE_STOPPED;
                    return begin_lift_interpolation_setup(
                        state,
                        canopen,
                        config,
                        LIFT_INTERP_DIRECTION_HOLD,
                        false,
                        now_ms);
                }
                state->lift_interpolation_state =
                    LIFT_INTERPOLATION_STATE_STOPPED;
            }
            return state->lift_interpolation_state !=
                   LIFT_INTERPOLATION_STATE_FAULT;
        }
        if (state->lift_interpolation_state ==
                LIFT_INTERPOLATION_STATE_STOPPED) {
            return true;
        }
        if (!lift_group_completed(state, canopen)) {
            return state->lift_interpolation_state !=
                   LIFT_INTERPOLATION_STATE_FAULT;
        }
        /* Operator neutral and all safe-stop sources synchronously disable the
         * four axes. Do not continue the old endpoint for a debounce interval,
         * and do not enter feedback-chasing leveling. */
        if (state->lift_interpolation_state ==
                LIFT_INTERPOLATION_STATE_FAULT) {
            state->lift_interpolation_state =
                LIFT_INTERPOLATION_STATE_STOPPED;
            return begin_lift_interpolation_setup(
                state,
                canopen,
                config,
                LIFT_INTERP_DIRECTION_HOLD,
                false,
                now_ms);
        }
        return queue_lift_interpolation_trigger(
            state,
            canopen,
            config,
            SERVO_DRIVE_CONTROL_DISABLE_VOLTAGE,
            LIFT_INTERPOLATION_STATE_STOPPING);
    }

    if (state->lift_interpolation_state == LIFT_INTERPOLATION_STATE_FAULT) {
        if (state->lift_transport_recovery_required) {
            recover_lift_transport_state(state, canopen, now_ms);
        } else {
            recover_lift_control_state(state, now_ms);
        }
        state->lift_command_target_position_counts =
            command_target_position_counts;
        return true;
    }

    if (state->lift_interpolation_state ==
            LIFT_INTERPOLATION_STATE_CONFIGURING) {
        (void)lift_setup_completed(state, canopen, config, now_ms);
        return state->lift_interpolation_state !=
               LIFT_INTERPOLATION_STATE_FAULT;
    }

    if (state->lift_interpolation_state ==
            LIFT_INTERPOLATION_STATE_READY_TO_SWITCH_ON) {
        if (!refresh_lift_feedback(state, canopen, config, now_ms)) {
            if (time_reached(now_ms, state->lift_setup_deadline_ms)) {
                state->lift_interpolation_failure_count++;
                state->lift_interpolation_state =
                    LIFT_INTERPOLATION_STATE_FAULT;
                return false;
            }
            return true;
        }
        update_lift_position_range_masks(state);
        if (state->lift_mechanical_range_invalid_mask != 0U) {
            state->lift_range_direction_reject_count++;
            return queue_lift_interpolation_trigger(
                state,
                canopen,
                config,
                SERVO_DRIVE_CONTROL_DISABLE_VOLTAGE,
                LIFT_INTERPOLATION_STATE_STOPPING);
        }
        return queue_lift_interpolation_trigger(
            state,
            canopen,
            config,
            SERVO_DRIVE_CONTROL_SWITCH_ON,
            LIFT_INTERPOLATION_STATE_SWITCHING_ON);
    }

    if (state->lift_interpolation_state ==
            LIFT_INTERPOLATION_STATE_SWITCHING_ON) {
        if (!lift_group_completed(state, canopen)) {
            return state->lift_interpolation_state !=
                   LIFT_INTERPOLATION_STATE_FAULT;
        }
        return queue_lift_interpolation_trigger(
            state,
            canopen,
            config,
            SERVO_DRIVE_CONTROL_ENABLE_OPERATION,
            LIFT_INTERPOLATION_STATE_ENABLING_OPERATION);
    }

    if (state->lift_interpolation_state ==
            LIFT_INTERPOLATION_STATE_ENABLING_OPERATION) {
        if (!lift_group_completed(state, canopen)) {
            return state->lift_interpolation_state !=
                   LIFT_INTERPOLATION_STATE_FAULT;
        }
        state->lift_sync_enable_count++;
        state->lift_enable_settle_until_ms =
            now_ms + ECU_CANOPEN_LIFT_ENABLE_SETTLE_MS;
        state->lift_setup_deadline_ms =
            state->lift_enable_settle_until_ms +
            ECU_CANOPEN_LIFT_SETTLE_TIMEOUT_MS;
        lift_settle_reset(state);
        state->lift_interpolation_state = LIFT_INTERPOLATION_STATE_SETTLING;
        return true;
    }

    if (state->lift_interpolation_state ==
            LIFT_INTERPOLATION_STATE_SETTLING) {
        const lift_interpolation_direction_t setup_direction =
            (lift_interpolation_direction_t)state->lift_requested_direction;
        const bool feedback_fresh =
            refresh_lift_feedback(state, canopen, config, now_ms);
        if (!time_reached(now_ms, state->lift_enable_settle_until_ms)) {
            return true;
        }
        if (!lift_axes_setup_feedback_ready(canopen, config)) {
            state->lift_interpolation_reject_count++;
            if (time_reached(now_ms, state->lift_setup_deadline_ms)) {
                state->lift_interpolation_failure_count++;
                state->lift_interpolation_state =
                    LIFT_INTERPOLATION_STATE_FAULT;
                return false;
            }
            return true;
        }
        if (!feedback_fresh) {
            if (time_reached(now_ms, state->lift_setup_deadline_ms)) {
                state->lift_interpolation_failure_count++;
                state->lift_interpolation_state =
                    LIFT_INTERPOLATION_STATE_FAULT;
                return false;
            }
            return true;
        }
        if (!lift_axes_settled(state, now_ms)) {
            if (time_reached(now_ms, state->lift_setup_deadline_ms)) {
                state->lift_interpolation_failure_count++;
                state->lift_interpolation_state =
                    LIFT_INTERPOLATION_STATE_FAULT;
                return false;
            }
            return true;
        }
        update_lift_position_range_masks(state);
        /* The setup direction is latched when the four-axis enable sequence
         * starts.  Use that coherent request for the post-enable range gate
         * instead of the latest task snapshot parameter.  Near the 10 mm
         * boundary, a one-cycle neutral/opposite snapshot previously made a
         * healthy synchronous enable group immediately disable and restart,
         * even though the operator's confirmed request and all node feedback
         * remained valid.  A real neutral request is handled by the stop branch
         * above; a real direction change is handled by the command-change
         * branch after setup completes.
         */
        if (state->lift_mechanical_range_invalid_mask != 0U ||
            !lift_direction_allowed_by_range(state, setup_direction)) {
            state->lift_range_direction_reject_count++;
            return queue_lift_interpolation_trigger(
                state,
                canopen,
                config,
                SERVO_DRIVE_CONTROL_DISABLE_VOLTAGE,
                LIFT_INTERPOLATION_STATE_STOPPING);
        }
        update_lift_running_spread_diagnostics(state);
        state->lift_start_with_leveling =
            state->lift_running_spread_counts > 0 ||
            state->lift_below_safe_range_mask != 0U ||
            state->lift_above_safe_range_mask != 0U;
        state->lift_progress_initialized = false;
        lift_stream_reset_trajectory(state, now_ms);
        state->lift_active_direction = (int8_t)LIFT_INTERP_DIRECTION_HOLD;
        /* Preload measured stationary points first, then create the common
         * 0x003F trigger edge.  Once interpolation is running, smoothly align
         * every axis to the direction-leading measured position before the
         * normal common-target trajectory.  This is the analyzer-proven order:
         * attempting to level before the trigger is a deadlock, while skipping
         * alignment would turn the initial measured spread into a step. */
        state->lift_preload_points_completed = 0U;
        state->lift_preload_group_pending = false;
        state->lift_interpolation_state =
            LIFT_INTERPOLATION_STATE_PRELOADING;
        return true;
    }

    if (state->lift_interpolation_state == LIFT_INTERPOLATION_STATE_STOPPED) {
        if (!lift_group_completed(state, canopen)) {
            return state->lift_interpolation_state !=
                   LIFT_INTERPOLATION_STATE_FAULT;
        }
        const bool feedback_fresh =
            refresh_lift_feedback(state, canopen, config, now_ms);
        if (feedback_fresh) {
            update_lift_position_range_masks(state);
        }
        if (!feedback_fresh) {
            return true;
        }
        if (!lift_direction_allowed_by_range(state, requested_direction)) {
            state->lift_range_direction_reject_count++;
            return true;
        }
        if (lift_positions_at_target(state, command_target_position_counts)) {
            state->lift_command_target_position_counts =
                command_target_position_counts;
            state->lift_at_target_disabled = true;
            return true;
        }
        if (!time_reached(now_ms, state->lift_recovery_not_before_ms)) {
            return true;
        }
        state->lift_at_target_disabled = false;
        state->lift_command_target_position_counts =
            command_target_position_counts;
        if (state->lift_prepared_disabled) {
            return begin_prepared_lift_enable(state,
                                              requested_direction,
                                              now_ms);
        }
        return begin_lift_interpolation_setup(
            state, canopen, config, requested_direction, true, now_ms);
    }

    if (state->lift_requested_direction != (int8_t)requested_direction ||
        state->lift_command_target_position_counts !=
            command_target_position_counts) {
        state->lift_requested_direction = (int8_t)requested_direction;
        state->lift_command_target_position_counts =
            command_target_position_counts;
        if (!lift_group_completed(state, canopen)) {
            return state->lift_interpolation_state !=
                   LIFT_INTERPOLATION_STATE_FAULT;
        }
        return queue_lift_interpolation_trigger(
            state,
            canopen,
            config,
            SERVO_DRIVE_CONTROL_DISABLE_VOLTAGE,
            LIFT_INTERPOLATION_STATE_STOPPING);
    }

    if (state->lift_interpolation_state ==
            LIFT_INTERPOLATION_STATE_PRELOADING) {
        if (state->lift_group_in_flight) {
            if (!lift_group_completed(state, canopen)) {
                return state->lift_interpolation_state !=
                       LIFT_INTERPOLATION_STATE_FAULT;
            }
            if (state->lift_interpolation_state ==
                    LIFT_INTERPOLATION_STATE_FAULT) {
                return false;
            }
            if (state->lift_preload_group_pending) {
                state->lift_preload_points_completed++;
                state->lift_preload_group_pending = false;
            }
            if (state->lift_preload_points_completed >=
                    ECU_CANOPEN_LIFT_PRELOAD_POINTS) {
                state->lift_interpolation_state =
                    LIFT_INTERPOLATION_STATE_TRIGGERING;
                return true;
            }
        }
        if (!queue_lift_interpolation_group(
                state,
                canopen,
                config,
                LIFT_INTERP_DIRECTION_HOLD,
                command_target_position_counts,
                now_ms)) {
            return false;
        }
        if (state->lift_group_in_flight) {
            state->lift_preload_group_pending = true;
        }
        return true;
    }

    if (state->lift_interpolation_state ==
            LIFT_INTERPOLATION_STATE_TRIGGERING) {
        if (!lift_group_completed(state, canopen)) {
            return state->lift_interpolation_state !=
                   LIFT_INTERPOLATION_STATE_FAULT;
        }
        if (state->lift_active_direction ==
                (int8_t)LIFT_INTERP_DIRECTION_HOLD) {
            if (!queue_lift_interpolation_trigger(
                    state,
                    canopen,
                    config,
                    SERVO_DRIVE_CONTROL_START_INTERPOLATION,
                    LIFT_INTERPOLATION_STATE_TRIGGERING)) {
                return false;
            }
            state->lift_active_direction = (int8_t)requested_direction;
            return true;
        }
        if (state->lift_start_with_leveling) {
            state->lift_start_with_leveling = false;
            begin_lift_leveling(state, requested_direction);
            return true;
        }
        state->lift_interpolation_state = LIFT_INTERPOLATION_STATE_RUNNING;
    }

    if (state->lift_interpolation_state ==
            LIFT_INTERPOLATION_STATE_RUNNING) {
        return queue_lift_interpolation_group(
            state,
            canopen,
            config,
            requested_direction,
            command_target_position_counts,
            now_ms);
    }
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

static can3_pdo_submit_result_t queue_hydraulic_pump_velocity(
    lift_hydraulic_device_state_t *state,
    canopen_master_service_t *canopen,
    const ecu_hardware_config_t *config,
    int32_t requested_velocity_units,
    uint32_t now_ms)
{
    canopen_master_pdo_request_t request;
    const bool pump_should_run = requested_velocity_units > 0;
    const int32_t target_velocity_units =
        hydraulic_pump_safe_velocity_units(requested_velocity_units);
    const uint16_t controlword =
        pump_should_run ? SERVO_DRIVE_CONTROL_ENABLE_OPERATION :
                          SERVO_DRIVE_CONTROL_DISABLE_VOLTAGE;

    if (pump_should_run && target_velocity_units >= 0) {
        state->pump_positive_clamp_count++;
    }

    if (state->pump_group_in_flight) {
        canopen_master_pdo_group_result_t group_result =
            canopen_master_service_pdo_group_result(
                canopen, state->pump_active_group_sequence);
        if (group_result == CANOPEN_MASTER_PDO_GROUP_RESULT_PENDING) {
            return CAN3_PDO_SUBMIT_DEFERRED;
        }
        state->pump_group_in_flight = false;
        if (group_result != CANOPEN_MASTER_PDO_GROUP_RESULT_COMPLETE) {
            state->last_pump_velocity_command_valid = false;
            state->pump_group_failure_count++;
            return CAN3_PDO_SUBMIT_TRANSPORT_FAULT;
        }
        state->last_pump_velocity_units = state->pump_active_velocity_units;
        state->last_pump_controlword = state->pump_active_controlword;
        state->last_pump_velocity_command_valid = true;
        state->last_pump_velocity_ms = now_ms;
        state->pump_group_complete_count++;
    }

    if (state->last_pump_velocity_command_valid &&
        state->last_pump_controlword == controlword &&
        state->last_pump_velocity_units == target_velocity_units) {
        return CAN3_PDO_SUBMIT_OK;
    }
    if (!canopen_master_service_realtime_pdo_idle(canopen)) {
        return CAN3_PDO_SUBMIT_DEFERRED;
    }

    uint32_t group_sequence = next_pump_group_sequence(state);
    if (!build_pump_velocity_request(&request,
                                     &config->hydraulic_pump_node,
                                     controlword,
                                     target_velocity_units,
                                     group_sequence)) {
        state->pump_velocity_reject_count++;
        return CAN3_PDO_SUBMIT_INVALID;
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
        return canopen->snapshot.last_pdo_failed_reason ==
                   (uint8_t)CANOPEN_MASTER_PDO_FAIL_GROUP_CONFLICT ||
               canopen->snapshot.last_pdo_failed_reason ==
                   (uint8_t)CANOPEN_MASTER_PDO_FAIL_QUEUE_FULL ?
               CAN3_PDO_SUBMIT_DEFERRED : CAN3_PDO_SUBMIT_INVALID;
    }

    state->pump_active_group_sequence = group_sequence;
    state->pump_active_velocity_units = target_velocity_units;
    state->pump_active_controlword = controlword;
    state->pump_group_in_flight = true;
    state->pump_velocity_queued_count++;
    return CAN3_PDO_SUBMIT_OK;
}

/* Return true only after the CAN3 scheduler has completed one explicit
 * zero-speed/disable PDO for Node13.  The stopped command is intentionally not
 * refreshed: RPDO0 is synchronous, so every refresh would add an unrelated
 * SYNC to the four-axis interpolation stream and prematurely consume one BC2
 * buffer point.  A cold boot and every transport recovery clear the cached
 * completion state, therefore one fail-safe stop is still transmitted before
 * CAN3 becomes idle again. */
static bool hydraulic_pump_stop_confirmed(
    const lift_hydraulic_device_state_t *state)
{
    return state != 0 &&
           !state->pump_group_in_flight &&
           state->last_pump_velocity_command_valid &&
           state->last_pump_controlword ==
               SERVO_DRIVE_CONTROL_DISABLE_VOLTAGE &&
           state->last_pump_velocity_units == 0;
}

/* Configure Node13 for velocity-mode PDO operation before the first pump run.
 *
 * The pump drive must not be reset here; resetting a servo can invalidate
 * position references elsewhere on the CANopen network.  This setup only sends
 * CiA-402 mode/controlword writes on the CAN3-owned SDO path:
 *
 *   NMT operational -> ECU_CANOPEN_OBJ_MODES_OF_OPERATION = velocity mode
 *   -> ECU_CANOPEN_OBJ_CONTROLWORD shutdown
 *   -> ECU_CANOPEN_OBJ_CONTROLWORD switch on
 *   -> ECU_CANOPEN_OBJ_CONTROLWORD enable operation
 *
 * Completion is judged from the CANopen service SDO download counter and abort
 * counter, so a locally queued request is not treated as remote acceptance.
 */
static bool ensure_hydraulic_pump_velocity_setup(lift_hydraulic_device_state_t *state,
                                                 canopen_master_service_t *canopen,
                                                 const ecu_hardware_config_t *config,
                                                 uint32_t now_ms)
{
    const ecu_canopen_node_config_t *node;

    if (state == 0 || canopen == 0 || config == 0) {
        return false;
    }
    if (state->pump_velocity_setup_ready) {
        return true;
    }

    node = &config->hydraulic_pump_node;

    if (state->pump_setup_in_flight) {
        if (canopen->snapshot.sdo_download_abort_count !=
                state->pump_setup_abort_count_baseline) {
            state->pump_setup_in_flight = false;
            state->pump_setup_failure_count++;
            state->pump_state = HYDRAULIC_PUMP_STATE_START_TIMEOUT;
            return false;
        }
        if (canopen->snapshot.sdo_download_count >=
                state->pump_setup_expected_download_count) {
            if (servo_setup_feedback_allows_ready(canopen, node->node_id)) {
                state->pump_setup_in_flight = false;
                state->pump_velocity_setup_ready = true;
                /* The SDO setup can take seconds.  Start the speed-feedback
                 * proof window here, not at an old/zero timestamp captured
                 * before configuration, or the first real start request can
                 * be reported as timed out immediately. */
                state->pump_state = HYDRAULIC_PUMP_STATE_STARTING;
                state->pump_start_request_ms = now_ms;
                state->pump_speed_ready_samples = 0U;
                state->pump_pressure_ready = false;
                return true;
            }
        }
        if (time_reached(now_ms, state->pump_setup_deadline_ms)) {
            state->pump_setup_in_flight = false;
            state->pump_setup_failure_count++;
            state->pump_state = HYDRAULIC_PUMP_STATE_START_TIMEOUT;
            return false;
        }
        return false;
    }

    if (!canopen_master_service_sdo_download_idle(canopen)) {
        state->pump_setup_reject_count++;
        return false;
    }

    uint32_t expected_writes = PUMP_SETUP_SDO_WRITES;
    canopen_node_feedback_t feedback;
    bool node_ok = true;
    if (canopen_master_service_get_node_feedback(
            canopen, node->node_id, &feedback) &&
        feedback.tpdo1_valid && feedback.fault_latched != 0U) {
        node_ok = canopen_master_service_request_sdo_write(
            canopen,
            node->node_id,
            ECU_CANOPEN_OBJ_FAULT_LATCHED,
            0U,
            4U,
            (int32_t)feedback.fault_latched);
        if (node_ok) {
            expected_writes++;
        }
    }
    node_ok = node_ok &&
        canopen_master_service_request_nmt(
            canopen,
            node->node_id,
            CANOPEN_MASTER_DEBUG_COMMAND_NMT_OPERATIONAL) &&
        canopen_master_service_request_sdo_write(
            canopen, node->node_id, ECU_CANOPEN_OBJ_CONTROLWORD,
            0U, 2U, SERVO_DRIVE_CONTROL_DISABLE_VOLTAGE) &&
        canopen_master_service_request_sdo_write(
            canopen, node->node_id, ECU_CANOPEN_OBJ_CONTROLWORD,
            0U, 2U, SERVO_DRIVE_CONTROL_FAULT_RESET) &&
        canopen_master_service_request_sdo_write(
            canopen, node->node_id, ECU_CANOPEN_OBJ_MODES_OF_OPERATION,
            0U, 1U, CANOPEN_PDO_MODE_PROFILE_VELOCITY) &&
        canopen_master_service_request_sdo_write(
            canopen, node->node_id, ECU_CANOPEN_OBJ_CONTROLWORD,
            0U, 2U, SERVO_DRIVE_CONTROL_SHUTDOWN) &&
        canopen_master_service_request_sdo_write(
            canopen, node->node_id, ECU_CANOPEN_OBJ_CONTROLWORD,
            0U, 2U, SERVO_DRIVE_CONTROL_SWITCH_ON) &&
        canopen_master_service_request_sdo_write(
            canopen, node->node_id, ECU_CANOPEN_OBJ_CONTROLWORD,
            0U, 2U, SERVO_DRIVE_CONTROL_ENABLE_OPERATION);

    if (!node_ok) {
        state->pump_setup_reject_count++;
        return false;
    }

    state->pump_setup_in_flight = true;
    state->pump_state = HYDRAULIC_PUMP_STATE_CONFIGURING;
    state->pump_setup_expected_download_count =
        canopen->snapshot.sdo_download_count + expected_writes;
    state->pump_setup_abort_count_baseline =
        canopen->snapshot.sdo_download_abort_count;
    state->pump_setup_deadline_ms =
        now_ms + ECU_CANOPEN_LIFT_SETUP_TIMEOUT_MS;
    return false;
}

/* Validate Node13 speed feedback before it is used as the valve-open interlock.
 *
 * The active PDO contract maps actual position and actual velocity into TPDO0.
 * Field logs on the current relay-box commissioning build show TPDO0 refreshing
 * continuously while TPDO1 may be event/sync sparse after startup.  The valve
 * gate therefore uses fresh TPDO0 velocity for the pump-speed proof and still
 * treats any received TPDO1 fault information as fail-closed.  A positive
 * measured speed never satisfies the gate because the pump is mechanically
 * reverse-only.
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
        (uint32_t)(now_ms - feedback.last_tpdo0_ms) >
            ECU_CANOPEN_LIFT_FEEDBACK_TIMEOUT_MS) {
        state->pump_feedback_reject_count++;
        return false;
    }

    state->pump_actual_velocity_units = feedback.actual_velocity_units;

    /* Use the same vendor-fault contract as the lift axes.  BC/BC2 statusword
     * bit layout is not a reliable CiA-402 fault test in every operating mode;
     * the mapped nonzero 0x2183 latch is the hard fault source. */
    if (feedback.tpdo1_valid && feedback.fault_latched != 0U) {
        state->pump_feedback_reject_count++;
        state->pump_velocity_setup_ready = false;
        return false;
    }

    state->pump_feedback_valid = true;
    return true;
}

static void close_hydraulic_valves(lift_hydraulic_device_state_t *state,
                                   const ecu_hardware_config_t *config)
{
    (void)config;
    state->last_valve_mask = 0U;
}

static bool handle_pump_submit_result(lift_hydraulic_device_state_t *state,
                                      canopen_master_service_t *canopen,
                                      can3_pdo_submit_result_t result,
                                      uint32_t now_ms)
{
    if (result == CAN3_PDO_SUBMIT_OK ||
        result == CAN3_PDO_SUBMIT_DEFERRED) {
        return true;
    }
    state->last_pump_velocity_command_valid = false;
    state->pump_velocity_setup_ready = false;
    if (result == CAN3_PDO_SUBMIT_TRANSPORT_FAULT) {
        recover_lift_transport_state(state, canopen, now_ms);
    }
    return false;
}

/* Apply the pump-first hydraulic sequence.
 *
 *  1. Close every valve before starting or changing direction.
 *  2. Run Node13 in the mechanically safe reverse direction.
 *  3. Before opening a valve for the first time, require three consecutive
 *     fresh TPDO samples above 800 rpm to prove pressure has been established.
 *  4. Open only the sanitized requested valve mask.
 *
 * The 800 rpm threshold is an initial pressure-building proof, not a continuous
 * valve safety cutoff.  After pressure_ready latches, short speed-feedback dips
 * do not chatter valve outputs.  Valves still close immediately when the
 * operator exits the adjustment domain, a valve request is removed, CAN3/high
 * voltage is unavailable, or pump feedback indicates fault/positive rotation.
 */
static bool apply_hydraulic_pump_and_valves(lift_hydraulic_device_state_t *state,
                                            canopen_master_service_t *canopen,
                                            const ecu_hardware_config_t *config,
                                            uint32_t valve_mask,
                                            bool pump_request,
                                            int32_t pump_velocity_request_units,
                                            uint32_t now_ms)
{
    if (!pump_request) {
        close_hydraulic_valves(state, config);
        state->pending_valve_mask = 0U;
        state->pump_speed_ready_samples = 0U;
        state->pump_feedback_valid = false;
        state->pump_pressure_ready = false;
        state->pump_state = HYDRAULIC_PUMP_STATE_STOPPED;
        state->pump_start_request_ms = 0U;
        state->pump_last_speed_ready_ms = 0U;
        if (hydraulic_pump_stop_confirmed(state)) {
            return true;
        }
        return handle_pump_submit_result(
            state,
            canopen,
            queue_hydraulic_pump_velocity(
                state, canopen, config, 0, now_ms),
            now_ms);
    }

    if (!ensure_hydraulic_pump_velocity_setup(state, canopen, config, now_ms)) {
        close_hydraulic_valves(state, config);
        return true;
    }

    if (state->pending_valve_mask != valve_mask) {
        close_hydraulic_valves(state, config);
        state->pending_valve_mask = valve_mask;
        state->valve_change_hold_until_ms =
            now_ms + ECU_HYDRAULIC_VALVE_CHANGE_DEADTIME_MS;
    }

    if (state->pump_state == HYDRAULIC_PUMP_STATE_STOPPED) {
        state->pump_state = HYDRAULIC_PUMP_STATE_STARTING;
        state->pump_start_request_ms = now_ms;
        state->pump_speed_ready_samples = 0U;
        state->pump_pressure_ready = false;
    }

    const bool feedback_ok =
        read_hydraulic_pump_feedback(state, canopen, config, now_ms);
    if (state->pump_pressure_ready && !feedback_ok) {
        close_hydraulic_valves(state, config);
        state->pump_pressure_ready = false;
        state->pump_speed_ready_samples = 0U;
        state->pump_state = HYDRAULIC_PUMP_STATE_STARTING;
        state->pump_start_request_ms = now_ms;
    }
    if (feedback_ok &&
        state->pump_actual_velocity_units >
            ECU_HYDRAULIC_PUMP_ZERO_SPEED_VELOCITY_UNITS) {
        close_hydraulic_valves(state, config);
        state->pump_pressure_ready = false;
        state->pump_state = HYDRAULIC_PUMP_STATE_START_TIMEOUT;
        state->pump_start_timeout_count++;
        return handle_pump_submit_result(
            state,
            canopen,
            queue_hydraulic_pump_velocity(
                state, canopen, config, 0, now_ms),
            now_ms);
    }

    const bool reverse_speed_ready =
        feedback_ok &&
        state->pump_actual_velocity_units <
            -ECU_HYDRAULIC_PUMP_VALVE_OPEN_MIN_VELOCITY_UNITS;
    if (reverse_speed_ready) {
        state->pump_last_speed_ready_ms = now_ms;
    }

    if (!state->pump_pressure_ready && reverse_speed_ready) {
        if (state->pump_speed_ready_samples < ECU_HYDRAULIC_PUMP_SPEED_READY_SAMPLES) {
            state->pump_speed_ready_samples++;
        }
        state->pump_last_speed_ready_ms = now_ms;
        if (state->pump_speed_ready_samples >=
            ECU_HYDRAULIC_PUMP_SPEED_READY_SAMPLES) {
            state->pump_pressure_ready = true;
            state->pump_state = HYDRAULIC_PUMP_STATE_VALVE_READY;
        }
    } else if (!state->pump_pressure_ready) {
        state->pump_speed_ready_samples = 0U;
        state->pump_state = HYDRAULIC_PUMP_STATE_STARTING;
        close_hydraulic_valves(state, config);
    }

    /* A brief speed dip after pressure has been established must not chatter
     * the valves, but a sustained loss of pump rotation cannot be ignored.
     * Close the valves after the confirmation window and invalidate the cached
     * PDO so the unchanged reverse-speed command is submitted once again. */
    if (state->pump_pressure_ready && feedback_ok && !reverse_speed_ready &&
        (uint32_t)(now_ms - state->pump_last_speed_ready_ms) >=
            ECU_HYDRAULIC_PUMP_SPEED_LOSS_CONFIRM_MS) {
        close_hydraulic_valves(state, config);
        state->pump_pressure_ready = false;
        state->pump_speed_ready_samples = 0U;
        state->pump_state = HYDRAULIC_PUMP_STATE_STARTING;
        state->pump_start_request_ms = now_ms;
        state->last_pump_velocity_command_valid = false;
    }

    if (!state->pump_pressure_ready &&
        state->pump_speed_ready_samples <
            ECU_HYDRAULIC_PUMP_SPEED_READY_SAMPLES &&
        (uint32_t)(now_ms - state->pump_start_request_ms) >
            ECU_HYDRAULIC_PUMP_START_TIMEOUT_MS) {
        close_hydraulic_valves(state, config);
        state->pump_state = HYDRAULIC_PUMP_STATE_STARTING;
        state->pump_start_request_ms = now_ms;
        state->pump_start_timeout_count++;
        state->last_pump_velocity_command_valid = false;
        return handle_pump_submit_result(
            state,
            canopen,
            queue_hydraulic_pump_velocity(
                state,
                canopen,
                config,
                pump_velocity_request_units,
                now_ms),
            now_ms);
    }

    const can3_pdo_submit_result_t pump_submit_result =
        queue_hydraulic_pump_velocity(state,
                                      canopen,
                                      config,
                                      pump_velocity_request_units,
                                      now_ms);
    if (!handle_pump_submit_result(state,
                                   canopen,
                                   pump_submit_result,
                                   now_ms)) {
        close_hydraulic_valves(state, config);
        return false;
    }
    if (!state->pump_pressure_ready ||
        valve_mask == 0U ||
        !time_reached(now_ms, state->valve_change_hold_until_ms)) {
        close_hydraulic_valves(state, config);
        return true;
    }

    state->pump_state = HYDRAULIC_PUMP_STATE_VALVE_READY;
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
                                                      const ecu_hardware_config_t *config,
                                                      const vehicle_actuator_command_t *command,
                                                      uint32_t now_ms)
{
    if (state == 0 || canopen == 0 || config == 0 || command == 0) {
        return ECU_DEVICE_APPLY_INVALID_ARGUMENT;
    }
    if (!canopen->snapshot.initialized) {
        close_hydraulic_valves(state, config);
        state->pump_feedback_valid = false;
        state->pump_speed_ready_samples = 0U;
        state->pump_pressure_ready = false;
        state->pump_state = HYDRAULIC_PUMP_STATE_STOPPED;
        state->lift_prepared_disabled = false;
        state->lift_interpolation_state = LIFT_INTERPOLATION_STATE_FAULT;
        lift_stream_reset_trajectory(state, now_ms);
        state->lift_active_direction =
            (int8_t)LIFT_INTERP_DIRECTION_HOLD;
        state->last_result = ECU_DEVICE_APPLY_BACKEND_OFFLINE;
        return state->last_result;
    }
    if (!canopen->snapshot.can_normal) {
        close_hydraulic_valves(state, config);
        state->pump_feedback_valid = false;
        state->pump_speed_ready_samples = 0U;
        state->pump_pressure_ready = false;
        recover_lift_transport_state(state, canopen, now_ms);
        if (!canopen->snapshot.can_normal) {
            state->last_result = ECU_DEVICE_APPLY_BACKEND_OFFLINE;
            return state->last_result;
        }
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

    /* Valves are energized only from the explicit valve mask produced by the
     * arbiter/executor.  Do not infer a valve from track_rate_mm_s here: the
     * rate is diagnostic/control intent, while hydraulic_valve_mask has already
     * passed the track presteer gate and the relay-pair interlock policy.
     */
    uint32_t requested_valve_mask = command->hydraulic_valve_mask;
    uint32_t interlocked_valve_mask = 0U;
    uint32_t valve_mask = sanitize_hydraulic_valve_mask(config,
                                                        requested_valve_mask,
                                                        &interlocked_valve_mask);
    const bool pump_request =
        command->hydraulic_enable &&
        command->high_voltage_enable &&
        !command->high_voltage_disable_request &&
        command->high_voltage_feedback_ready;
    /* Valve 3/4 track-width motion uses the configured maximum pump operating
     * point. Front/rear suspension valves keep ECU_HYDRAULIC_PUMP_WORK_RPM.
     * Select from the interlocked mask so a rejected/conflicting raw request
     * can never raise pump speed on its own. */
    const bool track_width_valve_requested =
        (valve_mask & (ECU_HYD_VALVE_TRACK_EXTEND_MASK |
                       ECU_HYD_VALVE_TRACK_RETRACT_MASK)) != 0U;
    const int32_t pump_velocity_request_units =
        pump_request ?
            (track_width_valve_requested ?
                ECU_HYDRAULIC_PUMP_TRACK_WIDTH_VELOCITY_UNITS :
                ECU_HYDRAULIC_PUMP_ENABLE_VELOCITY_UNITS) :
            0;
    ok = apply_hydraulic_pump_and_valves(state,
                                         canopen,
                                         config,
                                         valve_mask,
                                         pump_request,
                                         pump_velocity_request_units,
                                         now_ms) && ok;
    state->last_requested_valve_mask = requested_valve_mask & config->hydraulic_managed_valve_mask;
    state->last_interlocked_valve_mask = interlocked_valve_mask;
    if (interlocked_valve_mask != 0U) {
        state->valve_interlock_reject_count++;
    }

    /* Node13 and the four lift axes share one strict CAN3 realtime FIFO.  Give
     * a changed/refresh-due pump command the first submission opportunity, then
     * let the lift group use every remaining cycle.  A busy lane is a normal
     * deferral, never a transport reset.  This prevents both pump starvation
     * and the old reset loop where a lift group made the pump enqueue fail and
     * the failure cancelled the healthy lift group in return.
     */
    ok = process_lift_interpolation(state,
                                    canopen,
                                    config,
                                    lift_direction,
                                    command->target_height_mm,
                                    command->lift_request,
                                    command->high_voltage_feedback_ready,
                                    now_ms) && ok;
    state->apply_count++;
    state->last_result = ok ? ECU_DEVICE_APPLY_OK : ECU_DEVICE_APPLY_REJECTED;
    return state->last_result;
}
