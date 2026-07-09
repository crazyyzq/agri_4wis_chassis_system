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
    mailbox->publish_sequence++;
    if ((mailbox->publish_sequence & 1U) == 0U) {
        mailbox->publish_sequence++;
    }
    mailbox->command = *command;
    mailbox->timestamp_ms = now_ms;
    mailbox->valid = true;
    mailbox->publish_sequence++;
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

    read_sequence_before = mailbox->publish_sequence;
    if ((read_sequence_before & 1U) != 0U || !mailbox->valid) {
        return false;
    }
    *command = mailbox->command;
    *timestamp_ms = mailbox->timestamp_ms;
    read_sequence_after = mailbox->publish_sequence;

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
    executor->presteer_drive_hold_active =
        s_runtime.motion.presteer_drive_hold_active;
    executor->presteer_target_reached =
        s_runtime.motion.presteer_target_reached;
    executor->presteer_mode =
        (uint8_t)s_runtime.motion.presteer_mode;
    executor->presteer_missing_axis_mask =
        s_runtime.motion.presteer_missing_axis_mask;
    executor->presteer_timeout_count =
        s_runtime.motion.presteer_timeout_count;
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
                                                      io->dio,
                                                      config,
                                                      command);
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
    (void)command_timestamp_ms;

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
    dio_service_t *dio,
    uint32_t now_ms)
{
    const ecu_hardware_config_t *config = ecu_hardware_config_default();
    vehicle_actuator_command_t command;
    uint32_t command_sequence;
    uint32_t command_timestamp_ms;

    if (executor == 0 || can3_lift_hydraulic_canopen == 0 || dio == 0) {
        return false;
    }

    vehicle_executor_runtime_init_once();
    if (!read_can3_command_snapshot(&s_runtime.can3_mailbox,
                                    &command,
                                    &command_sequence,
                                    &command_timestamp_ms)) {
        executor->lift_hydraulic_result = ECU_DEVICE_APPLY_OK;
        return true;
    }
    (void)command_sequence;
    (void)command_timestamp_ms;

    executor->lift_hydraulic_result =
        lift_hydraulic_device_apply(&s_runtime.lift_hydraulic,
                                    can3_lift_hydraulic_canopen,
                                    dio,
                                    config,
                                    &command,
                                    now_ms);
    update_executor_lift_hydraulic_diagnostics(executor);
    return executor->lift_hydraulic_result == ECU_DEVICE_APPLY_OK;
}

void vehicle_command_executor_get_state(const vehicle_executor_state_t *executor,
                                        vehicle_executor_state_t *out)
{
    if (executor != 0 && out != 0) {
        *out = *executor;
    }
}
