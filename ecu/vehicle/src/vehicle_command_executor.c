#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "command_arbiter.h"
#include "ecu_config.h"
#include "lift_hydraulic_device.h"
#include "local_io_device.h"
#include "motion_device.h"
#include "vehicle_command_executor.h"
#include "warning_light_device.h"

typedef struct {
    bool initialized;
    motion_device_state_t motion;
    lift_hydraulic_device_state_t lift_hydraulic;
    local_io_device_state_t local_io;
    warning_light_device_state_t warning_light;
    vehicle_motion_command_mailbox_t motion_mailbox;
    vehicle_can3_command_mailbox_t can3_mailbox;
    volatile uint32_t local_output_intent_mask;
    volatile uint32_t hydraulic_output_intent_mask;
    uint32_t track_gate_requested_valve_mask;
    uint32_t track_gate_request_timestamp_ms;
} vehicle_executor_runtime_t;

static vehicle_executor_runtime_t s_runtime;

static void publish_motion_command_snapshot(vehicle_motion_command_mailbox_t *mailbox,
                                            const vehicle_actuator_command_t *command,
                                            uint32_t now_ms)
{
    if (mailbox == 0 || command == 0) {
        return;
    }

    /* Odd sequence means a writer is copying the multi-field command.  Readers
     * only accept the snapshot when the sequence is even and unchanged across
     * the copy.
     */
    uint32_t sequence = __atomic_load_n(&mailbox->publish_sequence,
                                        __ATOMIC_RELAXED) + 1U;
    if ((sequence & 1U) == 0U) {
        sequence++;
    }
    __atomic_store_n(&mailbox->publish_sequence, sequence, __ATOMIC_RELEASE);
    mailbox->command = *command;
    mailbox->timestamp_ms = now_ms;
    mailbox->valid = true;
    __atomic_store_n(&mailbox->publish_sequence,
                     sequence + 1U,
                     __ATOMIC_RELEASE);
}

static bool read_motion_command_snapshot(const vehicle_motion_command_mailbox_t *mailbox,
                                         vehicle_actuator_command_t *command,
                                         uint32_t *sequence,
                                         uint32_t *timestamp_ms)
{
    uint32_t read_sequence_before;
    uint32_t read_sequence_after;

    if (mailbox == 0 || command == 0 || sequence == 0 || timestamp_ms == 0) {
        return false;
    }

    read_sequence_before = __atomic_load_n(&mailbox->publish_sequence,
                                            __ATOMIC_ACQUIRE);
    if ((read_sequence_before & 1U) != 0U || !mailbox->valid) {
        return false;
    }
    *command = mailbox->command;
    *timestamp_ms = mailbox->timestamp_ms;
    read_sequence_after = __atomic_load_n(&mailbox->publish_sequence,
                                           __ATOMIC_ACQUIRE);

    if (read_sequence_before != read_sequence_after ||
        (read_sequence_after & 1U) != 0U) {
        return false;
    }

    *sequence = read_sequence_after;
    return true;
}

static void publish_can3_command_snapshot(vehicle_can3_command_mailbox_t *mailbox,
                                          const vehicle_actuator_command_t *command,
                                          uint32_t now_ms)
{
    publish_motion_command_snapshot(mailbox, command, now_ms);
}

static bool read_can3_command_snapshot(const vehicle_can3_command_mailbox_t *mailbox,
                                       vehicle_actuator_command_t *command,
                                       uint32_t *sequence,
                                       uint32_t *timestamp_ms)
{
    return read_motion_command_snapshot(mailbox, command, sequence, timestamp_ms);
}

static void vehicle_executor_runtime_init_once(void)
{
    if (s_runtime.initialized) {
        return;
    }
    motion_device_init(&s_runtime.motion);
    lift_hydraulic_device_init(&s_runtime.lift_hydraulic);
    local_io_device_init(&s_runtime.local_io);
    warning_light_device_init(&s_runtime.warning_light);
    s_runtime.initialized = true;
}

static void update_executor_motion_diagnostics(vehicle_executor_state_t *executor)
{
    if (executor == 0) {
        return;
    }

    executor->steer_normal_pdo_allowed =
        s_runtime.motion.steer_normal_pdo_allowed;
    executor->steer_safety_inhibited =
        s_runtime.motion.steer_safety_inhibited;
    executor->steer_inhibit_reason =
        (uint8_t)s_runtime.motion.steer_inhibit_reason;
    executor->steer_safety_inhibit_count =
        s_runtime.motion.steer_safety_inhibit_count;
    executor->steer_last_allowed_to_inhibited_ms =
        s_runtime.motion.steer_last_allowed_to_inhibited_ms;
    executor->steer_safe_stop_pending =
        s_runtime.motion.steer_safe_stop_pending;
    executor->steer_profile_setup_state =
        (uint8_t)s_runtime.motion.steer_profile_setup_state;
    executor->steer_profile_setup_axis =
        s_runtime.motion.steer_profile_setup_axis;
    executor->steer_profile_setup_object =
        s_runtime.motion.steer_profile_setup_object;
    executor->steer_profile_verified_mask =
        s_runtime.motion.steer_profile_verified_mask;
    executor->steer_profile_setup_failure_count =
        s_runtime.motion.steer_profile_setup_failure_count;
    executor->steer_commission_state =
        (uint8_t)s_runtime.motion.steer_commission_state;
    executor->steer_commission_axis_mask =
        s_runtime.motion.selected_axis_mask;
    executor->steer_commission_nmt_sent_mask =
        s_runtime.motion.steer_commission_nmt_sent_mask;
    executor->steer_commission_authorization_clear_count =
        s_runtime.motion.steer_commission_authorization_clear_count;
    executor->steer_commission_post_command_tpdo_pending =
        s_runtime.motion.steer_commission_post_command_tpdo_pending;
    executor->steer_commission_post_command_axis_mask =
        s_runtime.motion.steer_commission_post_command_axis_mask;
    executor->steer_commission_post_command_missing_mask =
        s_runtime.motion.steer_commission_post_command_missing_mask;
    executor->steer_commission_post_command_timeout_count =
        s_runtime.motion.steer_commission_post_command_timeout_count;
    executor->can2_realtime_transient_recovery_count =
        s_runtime.motion.can2_realtime_transient_recovery_count;
    executor->can2_realtime_consecutive_failure_count =
        s_runtime.motion.can2_realtime_consecutive_failure_count;
    executor->can2_realtime_last_recovery_ms =
        s_runtime.motion.can2_realtime_last_recovery_ms;
    executor->can2_node_recovery_pending_mask =
        s_runtime.motion.can2_node_recovery_pending_mask;
    executor->can2_stale_feedback_mask =
        s_runtime.motion.can2_stale_feedback_mask;
    executor->can2_partial_group_recovery_active =
        s_runtime.motion.can2_partial_group_recovery_active;
    executor->can2_recovery_steer_sync_pending =
        s_runtime.motion.can2_recovery_steer_sync_pending;
    executor->can2_partial_group_recovery_count =
        s_runtime.motion.can2_partial_group_recovery_count;
    executor->presteer_drive_hold_active =
        s_runtime.motion.presteer_drive_hold_active;
    executor->presteer_target_reached =
        s_runtime.motion.presteer_target_reached;
    executor->track_assist_steer_approximately_ready =
        s_runtime.motion.track_assist_steer_approximately_ready;
    executor->track_assist_overspeed_mask =
        s_runtime.motion.track_assist_overspeed_mask;
    executor->track_assist_feedback_invalid_mask =
        s_runtime.motion.track_assist_feedback_invalid_mask;
    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        executor->drive_last_command_kind[wheel] =
            (uint8_t)s_runtime.motion.drive_last_command_kind[wheel];
        executor->drive_last_enable_requested[wheel] =
            s_runtime.motion.drive_last_enable_requested[wheel];
        executor->drive_last_current_10ma[wheel] =
            s_runtime.motion.drive_last_current_10ma[wheel];
    }
    executor->drive_group_complete_count =
        s_runtime.motion.drive_group_complete_count;
    executor->drive_group_failure_count =
        s_runtime.motion.drive_group_failure_count;
    executor->presteer_mode =
        (uint8_t)s_runtime.motion.presteer_mode;
    executor->presteer_missing_axis_mask =
        s_runtime.motion.presteer_missing_axis_mask;
    executor->presteer_timeout_count =
        s_runtime.motion.presteer_timeout_count;
    executor->steer_zero_calibration_state =
        (uint8_t)s_runtime.motion.steer_zero_calibration_state;
    executor->steer_zero_calibration_done_mask =
        s_runtime.motion.steer_zero_calibration_done_mask;
    executor->steer_zero_calibration_fault_mask =
        s_runtime.motion.steer_zero_calibration_fault_mask;
    executor->steer_zero_calibration_request_count =
        s_runtime.motion.steer_zero_calibration_request_count;
    memcpy(executor->steer_zero_calibration_midpoint_counts,
           s_runtime.motion.steer_zero_calibration_midpoint_counts,
           sizeof(executor->steer_zero_calibration_midpoint_counts));
    memcpy(executor->steer_zero_calibration_peak_current_10ma,
           s_runtime.motion.steer_zero_calibration_peak_current_10ma,
           sizeof(executor->steer_zero_calibration_peak_current_10ma));
    executor->steer_transition_active =
        s_runtime.motion.steer_transition_planner.active;
    executor->steer_transition_completed =
        s_runtime.motion.steer_transition_planner.completed;
    executor->steer_transition_rejected_stale_feedback =
        s_runtime.motion.steer_transition_planner.rejected_stale_feedback;
    executor->steer_transition_id =
        s_runtime.motion.steer_transition_planner.transition_id;
    executor->steer_transition_feedback_fresh_mask =
        s_runtime.motion.steer_transition_planner.feedback_fresh_mask;
    executor->steer_transition_moving_axis_mask =
        s_runtime.motion.steer_transition_planner.moving_axis_mask;
    executor->steer_transition_max_distance_counts =
        s_runtime.motion.steer_transition_planner.max_distance_counts;
    memcpy(executor->steer_transition_actual_counts,
           s_runtime.motion.steer_transition_planner.actual_position_counts,
           sizeof(executor->steer_transition_actual_counts));
    memcpy(executor->steer_transition_output_counts,
           s_runtime.motion.steer_transition_planner.output_target_counts,
           sizeof(executor->steer_transition_output_counts));
    memcpy(executor->steer_transition_error_counts,
           s_runtime.motion.steer_transition_planner.error_counts,
           sizeof(executor->steer_transition_error_counts));
}

static void update_executor_lift_hydraulic_diagnostics(vehicle_executor_state_t *executor)
{
    if (executor == 0) {
        return;
    }
    executor->hydraulic_requested_valve_mask =
        s_runtime.lift_hydraulic.last_requested_valve_mask;
    executor->hydraulic_applied_valve_mask =
        s_runtime.lift_hydraulic.last_valve_mask;
    executor->hydraulic_interlocked_valve_mask =
        s_runtime.lift_hydraulic.last_interlocked_valve_mask;
    executor->hydraulic_valve_interlock_reject_count =
        s_runtime.lift_hydraulic.valve_interlock_reject_count;
    executor->hydraulic_pump_state =
        (uint8_t)s_runtime.lift_hydraulic.pump_state;
    executor->hydraulic_pump_feedback_valid =
        s_runtime.lift_hydraulic.pump_feedback_valid;
    executor->hydraulic_pump_actual_velocity_units =
        s_runtime.lift_hydraulic.pump_actual_velocity_units;
    executor->hydraulic_pump_start_timeout_count =
        s_runtime.lift_hydraulic.pump_start_timeout_count;
    executor->lift_interpolation_state =
        (uint8_t)s_runtime.lift_hydraulic.lift_interpolation_state;
    executor->lift_requested_direction =
        s_runtime.lift_hydraulic.lift_requested_direction;
    executor->lift_active_direction =
        s_runtime.lift_hydraulic.lift_active_direction;
    executor->lift_feedback_fresh_mask =
        (uint8_t)s_runtime.lift_hydraulic.lift_feedback_fresh_mask;
    executor->lift_axis_fault_mask =
        (uint8_t)s_runtime.lift_hydraulic.lift_axis_fault_mask;
    executor->lift_preload_points_completed =
        s_runtime.lift_hydraulic.lift_preload_points_completed;
    executor->lift_interpolation_queued_count =
        s_runtime.lift_hydraulic.lift_interpolation_queued_count;
    executor->lift_interpolation_reject_count =
        s_runtime.lift_hydraulic.lift_interpolation_reject_count;
    executor->lift_interpolation_failure_count =
        s_runtime.lift_hydraulic.lift_interpolation_failure_count;
    executor->lift_interpolation_recovery_count =
        s_runtime.lift_hydraulic.lift_interpolation_recovery_count;
    executor->lift_running_spread_warning_count =
        s_runtime.lift_hydraulic.lift_running_spread_warning_count;
    executor->lift_stream_planned_delta_counts =
        s_runtime.lift_hydraulic.lift_stream_planned_delta_counts;
    executor->lift_running_spread_counts =
        s_runtime.lift_hydraulic.lift_running_spread_counts;
    executor->lift_max_running_spread_counts =
        s_runtime.lift_hydraulic.lift_max_running_spread_counts;
    memcpy(executor->lift_actual_position_counts,
           s_runtime.lift_hydraulic.lift_actual_position_counts,
           sizeof(executor->lift_actual_position_counts));
    memcpy(executor->lift_target_position_counts,
           s_runtime.lift_hydraulic.lift_target_position_counts,
           sizeof(executor->lift_target_position_counts));
}

static uint32_t track_valve_mask(void)
{
    return ECU_HYD_VALVE_TRACK_EXTEND_MASK | ECU_HYD_VALVE_TRACK_RETRACT_MASK;
}

/* CAN3 may request the track-width valve only after CAN2 steering has reached
 * the commanded sideways posture.  The remote/arbiter layer expresses intent;
 * this executor gate uses the CAN2 device feedback-derived presteer state.
 */
static bool timestamp_reached(uint32_t now_ms, uint32_t reference_ms)
{
    return (int32_t)(now_ms - reference_ms) >= 0;
}

/* A higher-priority vehicle publisher may preempt CAN2/CAN3 after the bus task
 * captured its cycle-entry now_ms.  The coherent mailbox can therefore carry
 * a timestamp a few milliseconds newer than the consumer timestamp.  Unsigned
 * subtraction turns that valid scheduling skew into a near-UINT32_MAX age and
 * falsely injects a safe-stop command (observed as an intermittent pump stop).
 */
static bool command_snapshot_timestamp_is_fresh(uint32_t now_ms,
                                                uint32_t timestamp_ms,
                                                uint32_t stale_timeout_ms)
{
    int32_t signed_age_ms = (int32_t)(now_ms - timestamp_ms);
    if (signed_age_ms >= 0) {
        return (uint32_t)signed_age_ms <= stale_timeout_ms;
    }
    return (timestamp_ms - now_ms) <= ECU_CONTROL_SNAPSHOT_MAX_FUTURE_SKEW_MS;
}

static void update_track_valve_gate_session(const vehicle_actuator_command_t *command,
                                            uint32_t command_timestamp_ms)
{
    uint32_t requested_track_mask =
        command != 0 ? (command->hydraulic_valve_mask & track_valve_mask()) : 0U;

    if (requested_track_mask == 0U) {
        s_runtime.track_gate_requested_valve_mask = 0U;
        s_runtime.track_gate_request_timestamp_ms = 0U;
        return;
    }

    if (requested_track_mask != s_runtime.track_gate_requested_valve_mask) {
        s_runtime.track_gate_requested_valve_mask = requested_track_mask;
        s_runtime.track_gate_request_timestamp_ms = command_timestamp_ms;
    }
}

static void gate_track_valves_until_presteer_ready(vehicle_actuator_command_t *command,
                                                  uint32_t command_timestamp_ms)
{
    if (command == 0 || !command->track_assist_requested) {
        update_track_valve_gate_session(command, command_timestamp_ms);
        return;
    }

    update_track_valve_gate_session(command, command_timestamp_ms);

    bool ready_was_evaluated_after_this_request =
        s_runtime.track_gate_request_timestamp_ms == 0U ||
        timestamp_reached(s_runtime.motion.track_assist_steer_ready_eval_ms,
                          s_runtime.track_gate_request_timestamp_ms);

    if (!s_runtime.motion.track_assist_steer_approximately_ready ||
        !ready_was_evaluated_after_this_request) {
        command->hydraulic_valve_mask &= ~track_valve_mask();
        command->track_rate_mm_s = 0.0f;
    }
}

/* CAN2 drive current assist is allowed only after the CAN3 hydraulic adapter
 * has actually opened a track-width valve.  This prevents a wheel from pushing
 * against a closed hydraulic circuit while the pump is still starting.
 */
static void gate_track_assist_current_until_valve_open(vehicle_actuator_command_t *command)
{
    if (command == 0 || !command->track_assist_requested) {
        if (command != 0) {
            command->track_assist_active = false;
        }
        return;
    }

    bool valve_open =
        (s_runtime.lift_hydraulic.last_valve_mask & track_valve_mask()) != 0U;
    command->track_assist_active = valve_open;
    if (!valve_open) {
        for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
            command->track_assist_current_10ma[wheel] = 0;
        }
    }
}

static bool apply_can3_safe_default(vehicle_executor_state_t *executor,
                                    canopen_master_service_t *can3_lift_hydraulic_canopen,
                                    const ecu_hardware_config_t *config,
                                    uint32_t now_ms)
{
    vehicle_actuator_command_t safe_command;

    if (executor == 0 || can3_lift_hydraulic_canopen == 0 || config == 0) {
        return false;
    }

    vehicle_actuator_command_safe_default(&safe_command);
    safe_command.source = COMMAND_SOURCE_SAFETY;
    executor->lift_hydraulic_result =
        lift_hydraulic_device_apply(&s_runtime.lift_hydraulic,
                                    can3_lift_hydraulic_canopen,
                                    config,
                                    &safe_command,
                                    now_ms);
    update_executor_lift_hydraulic_diagnostics(executor);
    __atomic_store_n(&s_runtime.hydraulic_output_intent_mask,
                     0U,
                     __ATOMIC_RELEASE);
    return executor->lift_hydraulic_result == ECU_DEVICE_APPLY_OK;
}

void vehicle_command_executor_init(vehicle_executor_state_t *executor)
{
    if (executor == 0) {
        return;
    }
    memset(executor, 0, sizeof(*executor));
    vehicle_actuator_command_safe_default(&executor->last_command);
    executor->power_result = ECU_DEVICE_APPLY_OK;
    executor->motion_result = ECU_DEVICE_APPLY_OK;
    executor->lift_hydraulic_result = ECU_DEVICE_APPLY_OK;
    executor->local_io_result = ECU_DEVICE_APPLY_OK;
    executor->warning_light_result = ECU_DEVICE_APPLY_OK;
    memset(&s_runtime, 0, sizeof(s_runtime));
    vehicle_executor_runtime_init_once();
    update_executor_motion_diagnostics(executor);
    update_executor_lift_hydraulic_diagnostics(executor);
}

bool vehicle_command_executor_apply(vehicle_executor_state_t *executor,
                                    const vehicle_executor_io_t *io,
                                    const vehicle_actuator_command_t *command,
                                    uint32_t now_ms)
{
    const ecu_hardware_config_t *config = ecu_hardware_config_default();

    if (executor == 0 || io == 0 || command == 0 ||
        io->can3_lift_hydraulic_canopen == 0 ||
        io->dio == 0 || io->warning_light_uart == 0 ||
        io->warning_light_modbus == 0) {
        return false;
    }

    vehicle_executor_runtime_init_once();
    publish_motion_command_snapshot(&s_runtime.motion_mailbox, command, now_ms);
    publish_can3_command_snapshot(&s_runtime.can3_mailbox, command, now_ms);
    executor->motion_result = ECU_DEVICE_APPLY_OK;
    executor->lift_hydraulic_result = ECU_DEVICE_APPLY_OK;
    executor->local_io_result = local_io_device_apply(&s_runtime.local_io,
                                                      config,
                                                      command);
    __atomic_store_n(&s_runtime.local_output_intent_mask,
                     s_runtime.local_io.last_output_mask,
                     __ATOMIC_RELEASE);
    executor->high_voltage_relay_latched =
        s_runtime.local_io.high_voltage_relay_latched;
    executor->warning_light_result =
        warning_light_device_apply(&s_runtime.warning_light,
                                   io->warning_light_modbus,
                                   io->warning_light_uart,
                                   config,
                                   command->indicator_mode,
                                   now_ms);
    executor->last_command = *command;
    executor->applied_sequence++;
    return executor->power_result == ECU_DEVICE_APPLY_OK &&
           executor->motion_result == ECU_DEVICE_APPLY_OK &&
           executor->lift_hydraulic_result == ECU_DEVICE_APPLY_OK &&
           executor->local_io_result == ECU_DEVICE_APPLY_OK &&
           executor->warning_light_result == ECU_DEVICE_APPLY_OK;
}

bool vehicle_command_executor_flush_can2_motion(vehicle_executor_state_t *executor,
                                                canopen_master_service_t *can2_motion_canopen,
                                                uint32_t now_ms)
{
    const ecu_hardware_config_t *config = ecu_hardware_config_default();
    vehicle_actuator_command_t command;
    uint32_t command_sequence;
    uint32_t command_timestamp_ms;

    if (executor == 0 || can2_motion_canopen == 0) {
        return false;
    }

    vehicle_executor_runtime_init_once();
    if (!read_motion_command_snapshot(&s_runtime.motion_mailbox,
                                      &command,
                                      &command_sequence,
                                      &command_timestamp_ms)) {
        executor->motion_result = ECU_DEVICE_APPLY_OK;
        return true;
    }
    if (!command_snapshot_timestamp_is_fresh(
            now_ms,
            command_timestamp_ms,
            ECU_CAN2_COMMAND_STALE_TIMEOUT_MS)) {
        vehicle_actuator_command_t stale_safe_command;
        vehicle_actuator_command_safe_default(&stale_safe_command);
        stale_safe_command.source = COMMAND_SOURCE_SAFETY;
        /* Preserve the measured HV window only long enough for CAN2 to submit
         * an explicit zero/disable group.  The power/safety layers remain the
         * authority that subsequently removes high voltage.
         */
        stale_safe_command.high_voltage_enable = command.high_voltage_enable;
        stale_safe_command.high_voltage_feedback_ready =
            command.high_voltage_feedback_ready;
        stale_safe_command.high_voltage_disable_request = false;
        command = stale_safe_command;
        executor->can2_command_stale_count++;
        executor->can2_last_command_stale_ms = now_ms;
    }
    gate_track_assist_current_until_valve_open(&command);

    executor->motion_result = motion_device_apply(&s_runtime.motion,
                                                  can2_motion_canopen,
                                                  config,
                                                  &command,
                                                  command_sequence,
                                                  now_ms);
    update_executor_motion_diagnostics(executor);
    if (executor->motion_result != ECU_DEVICE_APPLY_OK) {
        return false;
    }
    executor->motion_result =
        motion_device_flush_realtime(&s_runtime.motion,
                                     can2_motion_canopen,
                                     config,
                                     now_ms);
    update_executor_motion_diagnostics(executor);
    return executor->motion_result == ECU_DEVICE_APPLY_OK;
}

bool vehicle_command_executor_flush_can3_lift_hydraulic(
    vehicle_executor_state_t *executor,
    canopen_master_service_t *can3_lift_hydraulic_canopen,
    uint32_t now_ms)
{
    const ecu_hardware_config_t *config = ecu_hardware_config_default();
    vehicle_actuator_command_t command;
    uint32_t command_sequence;
    uint32_t command_timestamp_ms;

    if (executor == 0 || can3_lift_hydraulic_canopen == 0) {
        return false;
    }

    vehicle_executor_runtime_init_once();
    if (!read_can3_command_snapshot(&s_runtime.can3_mailbox,
                                    &command,
                                    &command_sequence,
                                    &command_timestamp_ms)) {
        return apply_can3_safe_default(executor,
                                       can3_lift_hydraulic_canopen,
                                       config,
                                       now_ms);
    }
    if (!command_snapshot_timestamp_is_fresh(
            now_ms,
            command_timestamp_ms,
            ECU_CAN3_COMMAND_STALE_TIMEOUT_MS)) {
        return apply_can3_safe_default(executor,
                                       can3_lift_hydraulic_canopen,
                                       config,
                                       now_ms);
    }
    (void)command_sequence;
    gate_track_valves_until_presteer_ready(&command, command_timestamp_ms);

    executor->lift_hydraulic_result =
        lift_hydraulic_device_apply(&s_runtime.lift_hydraulic,
                                    can3_lift_hydraulic_canopen,
                                    config,
                                    &command,
                                    now_ms);
    update_executor_lift_hydraulic_diagnostics(executor);
    uint32_t hydraulic_mask = s_runtime.lift_hydraulic.last_valve_mask;
    if (hydraulic_mask != 0U) {
        hydraulic_mask |= config->dio_hydraulic_enable_mask;
    }
    __atomic_store_n(&s_runtime.hydraulic_output_intent_mask,
                     hydraulic_mask,
                     __ATOMIC_RELEASE);
    return executor->lift_hydraulic_result == ECU_DEVICE_APPLY_OK;
}

static uint32_t clear_conflicting_output_pair(uint32_t output_mask,
                                              uint32_t pair_mask)
{
    return pair_mask != 0U && (output_mask & pair_mask) == pair_mask ?
           (output_mask & ~pair_mask) : output_mask;
}

bool vehicle_command_executor_flush_local_io(vehicle_executor_state_t *executor,
                                             dio_service_t *dio)
{
    const ecu_hardware_config_t *config = ecu_hardware_config_default();

    if (executor == 0 || dio == 0 || config == 0) {
        return false;
    }

    uint32_t local_mask = __atomic_load_n(
        &s_runtime.local_output_intent_mask,
        __ATOMIC_ACQUIRE);
    uint32_t hydraulic_mask = __atomic_load_n(
        &s_runtime.hydraulic_output_intent_mask,
        __ATOMIC_ACQUIRE);
    uint32_t output_mask = local_mask | hydraulic_mask;

    /* Final defense at the sole hardware-output owner.  Even a corrupted or
     * torn upstream valve request cannot energize both directions of one
     * hydraulic circuit.
     */
    output_mask = clear_conflicting_output_pair(
        output_mask, config->hydraulic_valve_interlock_pair12_mask);
    output_mask = clear_conflicting_output_pair(
        output_mask, config->hydraulic_valve_interlock_pair34_mask);
    output_mask = clear_conflicting_output_pair(
        output_mask, config->hydraulic_valve_interlock_pair56_mask);
    dio_service_write_output_mask(dio, output_mask);
    executor->local_io_result = dio->last_apply_ok ?
        ECU_DEVICE_APPLY_OK : ECU_DEVICE_APPLY_REJECTED;
    return executor->local_io_result == ECU_DEVICE_APPLY_OK;
}

void vehicle_command_executor_get_state(const vehicle_executor_state_t *executor,
                                        vehicle_executor_state_t *out)
{
    if (executor != 0 && out != 0) {
        *out = *executor;
    }
}
