#include <stdbool.h>
#include <string.h>

#include "analog_input_service.h"
#include "command_arbiter.h"
#include "ecu_config.h"
#include "ecu_tasks.h"
#include "ipc_snapshot.h"
#include "remote_discrete.h"
#include "remote_manager.h"
#include "safety_manager.h"
#include "sbus_service.h"
#include "vehicle_command_executor.h"
#include "vehicle_state.h"

typedef struct {
    bool initialized;
    sbus_service_t sbus;
    analog_input_service_t analog_inputs;
    remote_discrete_channel_t discrete_channels[ECU_SBUS_CHANNEL_COUNT];
    remote_manager_t remote_manager;
    remote_control_request_t remote_request;
    auto_control_request_t auto_request;
    vehicle_safety_snapshot_t safety_snapshot;
    vehicle_actuator_command_t final_command;
    vehicle_executor_state_t executor;
    vehicle_state_snapshot_t vehicle_state;
    ipc_snapshot_t cpu0_snapshot;
} ecu_runtime_context_t;

static ecu_runtime_context_t s_runtime;

static const ecu_task_descriptor_t s_cpu0_tasks[ECU_TASK_CPU0_COUNT] = {
    { "safety", ECU_CPU0_SAFETY_PERIOD_MS, 31U, 512U },
    { "can2_motion", ECU_CPU0_CAN2_MOTION_PERIOD_MS, 29U, 512U },
    { "remote", ECU_CPU0_REMOTE_PERIOD_MS, 27U, 768U },
    { "vehicle", ECU_CPU0_CONTROL_PERIOD_MS, 26U, 768U },
    { "can1_power", ECU_CPU0_POWER_PERIOD_MS, 22U, 512U },
    { "can3_lift", ECU_CPU0_LIFT_HYD_PERIOD_MS, 22U, 512U },
    { "io", ECU_CPU0_IO_PERIOD_MS, 16U, 512U },
    { "diag", ECU_CPU0_DIAG_PERIOD_MS, 8U, 512U }
};

static void ecu_runtime_init_once(uint32_t now_ms)
{
    if (s_runtime.initialized) {
        return;
    }

    memset(&s_runtime, 0, sizeof(s_runtime));
    sbus_service_init(&s_runtime.sbus, REMOTE_FAILSAFE_TIMEOUT_MS);
    analog_input_service_init(&s_runtime.analog_inputs);
    for (uint32_t channel = 0U; channel < ECU_SBUS_CHANNEL_COUNT; ++channel) {
        remote_discrete_init(&s_runtime.discrete_channels[channel],
                             REMOTE_POS_CENTER,
                             now_ms);
    }
    remote_manager_init(&s_runtime.remote_manager, now_ms);
    vehicle_actuator_command_safe_default(&s_runtime.final_command);
    vehicle_command_executor_init(&s_runtime.executor);
    ipc_snapshot_init(&s_runtime.cpu0_snapshot);

    s_runtime.safety_snapshot.brake_release_allowed = false;
    s_runtime.safety_snapshot.zero_speed_confirmed = true;
    s_runtime.safety_snapshot.primary_diag = DIAG_OK;
    s_runtime.initialized = true;
}

static remote_position_t stable_position_from_channel(const sbus_service_snapshot_t *sbus,
                                                      ecu_sbus_channel_role_t channel,
                                                      const ecu_config_t *config,
                                                      uint32_t now_ms)
{
    remote_position_t candidate = REMOTE_POS_INVALID;
    if (sbus->valid && channel < ECU_SBUS_CHANNEL_COUNT) {
        candidate = remote_discrete_position_from_raw(sbus->channels[channel],
                                                      &config->sbus_thresholds);
    }
    remote_discrete_update(&s_runtime.discrete_channels[channel],
                           candidate,
                           now_ms,
                           config->discrete_debounce_ms);
    return s_runtime.discrete_channels[channel].stable_position;
}

static void build_remote_input_snapshot(const sbus_service_snapshot_t *sbus,
                                        const ecu_config_t *config,
                                        uint32_t now_ms,
                                        remote_input_snapshot_t *out)
{
    memset(out, 0, sizeof(*out));
    out->now_ms = now_ms;
    out->sbus_valid = sbus->valid && sbus->connected;
    out->sbus_failsafe = sbus->failsafe;
    out->sbus_timeout = !sbus->connected;
    out->decode_error_limit = false;
    out->credibility_error = false;

    out->steering = stable_position_from_channel(sbus, ECU_SBUS_CH_STEER, config, now_ms);
    out->clearance = stable_position_from_channel(sbus, ECU_SBUS_CH_CLEARANCE, config, now_ms);
    out->throttle = stable_position_from_channel(sbus, ECU_SBUS_CH_THROTTLE, config, now_ms);
    out->power = stable_position_from_channel(sbus, ECU_SBUS_CH_POWER, config, now_ms);
    out->gear = stable_position_from_channel(sbus, ECU_SBUS_CH_GEAR, config, now_ms);
    out->home = stable_position_from_channel(sbus, ECU_SBUS_CH_HOME, config, now_ms);
    out->authority = stable_position_from_channel(sbus, ECU_SBUS_CH_AUTHORITY, config, now_ms);
    out->left_indicator = stable_position_from_channel(sbus, ECU_SBUS_CH_LEFT_INDICATOR, config, now_ms);
    out->hazard = stable_position_from_channel(sbus, ECU_SBUS_CH_HAZARD, config, now_ms);
    out->horn = stable_position_from_channel(sbus, ECU_SBUS_CH_HORN, config, now_ms);
    out->headlight = stable_position_from_channel(sbus, ECU_SBUS_CH_HEADLIGHT, config, now_ms);
    out->right_indicator = stable_position_from_channel(sbus, ECU_SBUS_CH_RIGHT_INDICATOR, config, now_ms);
    out->track = stable_position_from_channel(sbus, ECU_SBUS_CH_TRACK, config, now_ms);
    out->r1 = stable_position_from_channel(sbus, ECU_SBUS_CH_R1, config, now_ms);
    out->r2 = stable_position_from_channel(sbus, ECU_SBUS_CH_R2, config, now_ms);

    /* CH13 e-stop intentionally bypasses normal event suppression. */
    out->ch13_estop = stable_position_from_channel(sbus, ECU_SBUS_CH_ESTOP, config, now_ms);
    if (sbus->valid &&
        remote_discrete_position_from_raw(sbus->channels[ECU_SBUS_CH_ESTOP],
                                          &config->sbus_thresholds) == REMOTE_POS_HIGH) {
        out->ch13_estop = REMOTE_POS_HIGH;
    }
}

static void build_remote_preconditions(const remote_input_snapshot_t *input,
                                       remote_preconditions_t *out)
{
    memset(out, 0, sizeof(*out));
    out->link_online = s_runtime.remote_request.link_state == REMOTE_LINK_ONLINE;
    out->arm_ready = s_runtime.remote_request.arm_state == REMOTE_ARM_READY;
    out->estop_latched = s_runtime.safety_snapshot.estop_latched;
    out->a_class_fault = s_runtime.safety_snapshot.a_class_fault;
    out->zero_speed = s_runtime.safety_snapshot.zero_speed_confirmed;
    out->brake_applied = !s_runtime.executor.last_command.brake_release;
    out->brake_release_confirmed = s_runtime.executor.last_command.brake_release;
    out->throttle_low = input->throttle == REMOTE_POS_LOW;
    out->steering_neutral = input->steering == REMOTE_POS_CENTER;
    out->adjustment_active = s_runtime.remote_request.adjust_state != ADJUST_STATE_IDLE;
    out->active_transition = false;
    out->external_auto_request_valid = s_runtime.auto_request.valid;
    out->power_ready = true;
    out->low_voltage_ok = true;
    out->can1_power_online = true;
    out->hydraulic_stopped = !s_runtime.executor.last_command.hydraulic_enable;
    out->requested_gear = s_runtime.remote_request.requested_gear;
    out->active_gear = s_runtime.remote_request.active_gear;
}

const ecu_task_descriptor_t *ecu_cpu0_task_descriptor(ecu_cpu0_task_id_t task_id)
{
    return task_id < ECU_TASK_CPU0_COUNT ? &s_cpu0_tasks[task_id] : 0;
}

void ecu_task_safety_supervisor_step(uint32_t now_ms)
{
    ecu_runtime_init_once(now_ms);
    s_runtime.safety_snapshot.estop_latched =
        s_runtime.remote_request.estop_state != REMOTE_ESTOP_CLEAR;
    s_runtime.safety_snapshot.sbus_failsafe =
        s_runtime.remote_request.link_state == REMOTE_LINK_FAILSAFE;
    s_runtime.safety_snapshot.controlled_stop_active =
        s_runtime.safety_snapshot.estop_latched ||
        s_runtime.safety_snapshot.sbus_failsafe;
    s_runtime.safety_snapshot.shutdown_protect_active =
        s_runtime.remote_request.orderly_shutdown_request;
    s_runtime.safety_snapshot.brake_release_allowed =
        !s_runtime.safety_snapshot.a_class_fault &&
        !s_runtime.safety_snapshot.estop_latched &&
        !s_runtime.safety_snapshot.sbus_failsafe;
    s_runtime.safety_snapshot.primary_diag = s_runtime.remote_request.diagnostic;
}

void ecu_task_can2_motion_step(uint32_t now_ms)
{
    ecu_runtime_init_once(now_ms);
    /* CAN2 feedback integration point: drive, steering and brake status from
     * configured CANopen nodes refresh safety and motion-state snapshots here. */
}

void ecu_task_remote_manager_step(uint32_t now_ms)
{
    ecu_runtime_init_once(now_ms);

    const ecu_config_t *config = ecu_config_default();
    sbus_service_snapshot_t sbus_snapshot;
    remote_input_snapshot_t remote_input;
    remote_preconditions_t preconditions;

    sbus_service_poll(&s_runtime.sbus, now_ms);
    sbus_service_get_snapshot(&s_runtime.sbus, &sbus_snapshot);
    build_remote_input_snapshot(&sbus_snapshot, config, now_ms, &remote_input);
    build_remote_preconditions(&remote_input, &preconditions);
    remote_manager_update(&s_runtime.remote_manager,
                          &remote_input,
                          &preconditions,
                          config);
    remote_manager_get_request(&s_runtime.remote_manager, &s_runtime.remote_request);
}

void ecu_task_vehicle_control_step(uint32_t now_ms)
{
    ecu_runtime_init_once(now_ms);

    command_arbiter_update(&s_runtime.remote_request,
                           &s_runtime.auto_request,
                           &s_runtime.safety_snapshot,
                           now_ms,
                           &s_runtime.final_command);
    safety_manager_apply(&s_runtime.safety_snapshot, &s_runtime.final_command);
    (void)vehicle_command_executor_apply(&s_runtime.executor, &s_runtime.final_command);
    vehicle_state_publish(&s_runtime.vehicle_state,
                          now_ms,
                          &s_runtime.safety_snapshot,
                          &s_runtime.final_command);
}

void ecu_task_can1_power_step(uint32_t now_ms)
{
    ecu_runtime_init_once(now_ms);
    /* CAN1 feedback integration point: BMS, DC/DC and inverter status refresh
     * power preconditions and high-voltage diagnostics here. */
}

void ecu_task_can3_lift_hydraulic_step(uint32_t now_ms)
{
    ecu_runtime_init_once(now_ms);
    /* CAN3 feedback integration point: lift axes and hydraulic station status
     * refresh adjustment limits, timeout diagnostics and hydraulic-stopped state. */
}

void ecu_task_io_service_step(uint32_t now_ms)
{
    ecu_runtime_init_once(now_ms);
    analog_input_snapshot_t analog_snapshot;
    analog_input_service_get_snapshot(&s_runtime.analog_inputs, &analog_snapshot);
    (void)analog_snapshot;
    /* Local IO integration point: DI, DO and ADC services refresh brake relay,
     * analog sensor and local fault snapshots here. ISR handlers only feed
     * lightweight buffers. */
}

void ecu_task_diag_cpu0_step(uint32_t now_ms)
{
    ecu_runtime_init_once(now_ms);
    (void)ipc_snapshot_publish(&s_runtime.cpu0_snapshot,
                               &s_runtime.vehicle_state,
                               sizeof(s_runtime.vehicle_state),
                               now_ms);
}

void ecu_task_cpu1_service_step(uint32_t now_ms)
{
    (void)now_ms;
}
