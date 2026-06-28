#include <stdbool.h>
#include <string.h>

#include "analog_modbus_device.h"
#include "analog_input_service.h"
#include "can_bus_hw.h"
#include "can_bus_service.h"
#include "canopen_master_service.h"
#include "command_arbiter.h"
#include "ecu_config.h"
#include "ecu_tasks.h"
#include "ipc_snapshot.h"
#include "modbus_master_service.h"
#include "power_device.h"
#include "remote_discrete.h"
#include "remote_manager.h"
#include "runtime_monitor.h"
#include "safety_manager.h"
#include "sbus_service.h"
#include "sbus_uart_hw.h"
#include "uart_rs485_hw.h"
#include "vehicle_command_executor.h"
#include "vehicle_state.h"

typedef struct {
    bool initialized;
    sbus_service_t sbus;
    sbus_service_snapshot_t sbus_snapshot;
    can_bus_service_t can1_power;
    power_device_state_t power_device;
    power_device_snapshot_t power_snapshot;
    can_bus_service_t can2_drive_debug;
#if ECU_ENABLE_CANOPENNODE
    canopen_master_service_t can2_canopen_diag;
#endif
    uart_rs485_hw_t rs485_1_hw;
    modbus_master_service_t adc_modbus_master;
    analog_modbus_device_state_t analog_modbus_adc;
    analog_input_service_t analog_inputs;
    analog_input_snapshot_t analog_snapshot;
    remote_discrete_channel_t discrete_channels[ECU_SBUS_CHANNEL_COUNT];
    remote_manager_t remote_manager;
    remote_control_request_t remote_request;
    auto_control_request_t auto_request;
    vehicle_safety_snapshot_t safety_snapshot;
    ecu_hardware_feedback_snapshot_t hardware_feedback;
    vehicle_actuator_command_t final_command;
    vehicle_executor_state_t executor;
    vehicle_state_snapshot_t vehicle_state;
    ipc_snapshot_t cpu0_snapshot;
    uint32_t last_debug_monitor_ms;
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

void ecu_task_runtime_init(uint32_t now_ms)
{
    if (s_runtime.initialized) {
        return;
    }

    memset(&s_runtime, 0, sizeof(s_runtime));
    sbus_service_init(&s_runtime.sbus, REMOTE_FAILSAFE_TIMEOUT_MS);
    if (!sbus_uart_hw_init(&s_runtime.sbus)) {
        s_runtime.sbus.snapshot.uart_error_count++;
    }
    const ecu_hardware_config_t *hardware_config = ecu_hardware_config_default();
    can_bus_service_init(&s_runtime.can1_power, hardware_config->can1_bitrate);
    power_device_init(&s_runtime.power_device);
    if (hardware_config->power_protocol == ECU_POWER_PROTOCOL_SUPPLIER_CAN &&
        !can_bus_hw_init_can1_power(&s_runtime.can1_power,
                                    hardware_config->can1_bitrate)) {
        can_bus_service_note_error_from_isr(&s_runtime.can1_power);
    }
#if ECU_ENABLE_CANOPENNODE
    if (!canopen_master_service_init(&s_runtime.can2_canopen_diag,
                                     hardware_config->can2_bitrate,
                                     ECU_CANOPEN_MASTER_NODE_ID,
                                     ECU_CANOPEN_BC2_DIAG_NODE_ID,
                                     now_ms)) {
        s_runtime.can2_canopen_diag.snapshot.last_error = -1;
    }
#else
    can_bus_service_init(&s_runtime.can2_drive_debug, hardware_config->can2_bitrate);
    if (!can_bus_hw_init_can2_rx_only(&s_runtime.can2_drive_debug,
                                      hardware_config->can2_bitrate)) {
        can_bus_service_note_error_from_isr(&s_runtime.can2_drive_debug);
    }
#endif
    analog_input_service_init(&s_runtime.analog_inputs);
    analog_modbus_device_init(&s_runtime.analog_modbus_adc);
    modbus_master_service_init(&s_runtime.adc_modbus_master,
                               hardware_config->modbus_adc_poll_period_ms,
                               hardware_config->modbus_adc_response_timeout_ms);
    if (!uart_rs485_1_hw_init(&s_runtime.rs485_1_hw,
                              hardware_config->modbus_adc_baudrate)) {
        s_runtime.adc_modbus_master.snapshot.error_count++;
    }
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
    s_runtime.hardware_feedback.zero_speed_confirmed = true;
    s_runtime.hardware_feedback.hydraulic_stopped = true;
    s_runtime.safety_snapshot.primary_diag = DIAG_OK;
    s_runtime.initialized = true;
}

static void ecu_runtime_init_once(uint32_t now_ms)
{
    ecu_task_runtime_init(now_ms);
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
    out->zero_speed = s_runtime.hardware_feedback.zero_speed_confirmed;
    out->brake_applied = !s_runtime.executor.last_command.brake_release;
    out->brake_release_confirmed = s_runtime.hardware_feedback.brake_release_confirmed;
    out->throttle_low = input->throttle == REMOTE_POS_LOW;
    out->steering_neutral = input->steering == REMOTE_POS_CENTER;
    out->adjustment_active = s_runtime.remote_request.adjust_state != ADJUST_STATE_IDLE;
    out->active_transition = false;
    out->external_auto_request_valid = s_runtime.auto_request.valid;
    out->power_ready = s_runtime.hardware_feedback.power_ready;
    out->low_voltage_ok = s_runtime.hardware_feedback.low_voltage_ok;
    out->can1_power_online = s_runtime.hardware_feedback.can1_power_online;
    out->hydraulic_stopped = s_runtime.hardware_feedback.hydraulic_stopped;
    out->requested_gear = s_runtime.remote_request.requested_gear;
    out->active_gear = s_runtime.remote_request.active_gear;
}

static void refresh_can2_feedback(void)
{
#if ECU_ENABLE_CANOPENNODE
    canopen_master_snapshot_t snapshot;
    canopen_master_service_get_snapshot(&s_runtime.can2_canopen_diag, &snapshot);
    s_runtime.hardware_feedback.can2_motion_online =
        snapshot.initialized && snapshot.can_normal;
#else
    can_bus_service_t snapshot;
    can_bus_service_get_snapshot(&s_runtime.can2_drive_debug, &snapshot);
    s_runtime.hardware_feedback.can2_motion_online =
        snapshot.online && snapshot.error_count == 0U;
#endif
}

static void refresh_power_feedback(void)
{
    const ecu_hardware_config_t *config = ecu_hardware_config_default();
    power_device_get_snapshot(&s_runtime.power_device, &s_runtime.power_snapshot);

    if (config->power_protocol == ECU_POWER_PROTOCOL_DISABLED) {
        s_runtime.hardware_feedback.power_ready = false;
        s_runtime.hardware_feedback.low_voltage_ok = false;
        s_runtime.hardware_feedback.can1_power_online = false;
        return;
    }

    s_runtime.hardware_feedback.power_ready = s_runtime.power_snapshot.power_ready;
    s_runtime.hardware_feedback.low_voltage_ok = s_runtime.power_snapshot.low_voltage_ok;
    s_runtime.hardware_feedback.can1_power_online =
        s_runtime.power_snapshot.can1_online &&
        (s_runtime.power_snapshot.bms_online ||
         s_runtime.power_snapshot.dcdc48_online ||
         s_runtime.power_snapshot.dcdc12_online ||
         s_runtime.power_snapshot.dcac_online);
}

static void refresh_lift_hydraulic_feedback(void)
{
    s_runtime.hardware_feedback.can3_lift_hydraulic_online = false;
    s_runtime.hardware_feedback.hydraulic_stopped =
        !s_runtime.executor.last_command.hydraulic_enable;
}

static void refresh_local_io_feedback(void)
{
    s_runtime.hardware_feedback.brake_release_confirmed =
        s_runtime.executor.last_command.brake_release;
    s_runtime.hardware_feedback.zero_speed_confirmed =
        s_runtime.final_command.target_speed_kph == 0.0f;
}

static int32_t float_to_centi(float value)
{
    float scaled = value * 100.0f;
    if (scaled > 2147483000.0f) {
        return 2147483000;
    }
    if (scaled < -2147483000.0f) {
        return -2147483000;
    }
    return (int32_t)scaled;
}

static void build_runtime_monitor_snapshot(uint32_t now_ms,
                                           runtime_monitor_snapshot_t *out)
{
    memset(out, 0, sizeof(*out));
    out->now_ms = now_ms;
    out->executor_sequence = s_runtime.executor.applied_sequence;
    out->sbus_valid = s_runtime.sbus_snapshot.valid;
    out->sbus_connected = s_runtime.sbus_snapshot.connected;
    out->sbus_failsafe = s_runtime.sbus_snapshot.failsafe;
    out->sbus_frame_lost = s_runtime.sbus_snapshot.frame_lost;
    out->sbus_channel17 = s_runtime.sbus_snapshot.channel17;
    out->sbus_channel18 = s_runtime.sbus_snapshot.channel18;
    for (uint32_t channel = 0U; channel < ECU_SBUS_CHANNEL_COUNT; ++channel) {
        out->sbus_channels[channel] = s_runtime.sbus_snapshot.channels[channel];
    }
    out->sbus_frame_count = s_runtime.sbus_snapshot.frame_count;
    out->sbus_decode_error_count = s_runtime.sbus_snapshot.decode_error_count;
    out->sbus_uart_error_count = s_runtime.sbus_snapshot.uart_error_count;
    out->sbus_last_frame_ms = s_runtime.sbus_snapshot.last_frame_ms;

#if ECU_ENABLE_CANOPENNODE
    canopen_master_service_get_snapshot(&s_runtime.can2_canopen_diag,
                                        &out->can2_canopen_snapshot);
    out->can2_canopen_initialized = out->can2_canopen_snapshot.initialized;
    out->canopen_command = out->can2_canopen_snapshot.last_command;
#else
    can_bus_service_t can2_snapshot;
    can_bus_service_get_snapshot(&s_runtime.can2_drive_debug, &can2_snapshot);
    out->can2_rx_count = can2_snapshot.rx_count;
    out->can2_error_count = can2_snapshot.error_count;
    out->can2_rx_buffer_status = can2_snapshot.rx_buffer_status;
    out->can2_tx_rx_flags = can2_snapshot.tx_rx_flags;
    out->can2_error_flags = can2_snapshot.error_flags;
    out->can2_receive_error_count = can2_snapshot.receive_error_count;
    out->can2_transmit_error_count = can2_snapshot.transmit_error_count;
    out->can2_last_error_kind = can2_snapshot.last_error_kind;
    out->can2_last_rx_id = can2_snapshot.last_rx_id;
    out->can2_last_rx_size = can2_snapshot.last_rx_size;
    out->can2_last_rx_extended = can2_snapshot.last_rx_extended;
    out->can2_last_rx_remote = can2_snapshot.last_rx_remote;
    memcpy(out->can2_last_rx_data,
           can2_snapshot.last_rx_data,
           sizeof(out->can2_last_rx_data));
#endif
    can_bus_service_t can1_snapshot;
    can_bus_service_get_snapshot(&s_runtime.can1_power, &can1_snapshot);
    out->can1_tx_count = can1_snapshot.tx_count;
    out->can1_rx_count = can1_snapshot.rx_count;
    out->can1_error_count = can1_snapshot.error_count;
    out->can1_last_tx_id = can1_snapshot.last_tx_frame.id;
    out->can1_last_tx_size = can1_snapshot.last_tx_frame.size;
    out->can1_last_tx_extended = can1_snapshot.last_tx_frame.extended;
    out->can1_last_rx_id = can1_snapshot.last_rx_frame.id;
    out->can1_last_rx_size = can1_snapshot.last_rx_frame.size;
    out->can1_last_rx_extended = can1_snapshot.last_rx_frame.extended;
    memcpy(out->can1_last_rx_data,
           can1_snapshot.last_rx_frame.data,
           sizeof(out->can1_last_rx_data));
    power_device_get_snapshot(&s_runtime.power_device, &out->power_snapshot);
    modbus_master_service_get_snapshot(&s_runtime.adc_modbus_master,
                                       &out->modbus_adc_master);
    out->analog_modbus_adc = s_runtime.analog_modbus_adc;
    out->hardware_feedback = s_runtime.hardware_feedback;
    out->link_state = s_runtime.remote_request.link_state;
    out->estop_state = s_runtime.remote_request.estop_state;
    out->diagnostic = s_runtime.final_command.diagnostic;
    out->source = s_runtime.final_command.source;
    out->motion_mode = s_runtime.final_command.motion_mode;
    out->active_gear = s_runtime.final_command.active_gear;
    out->target_speed_centi_kph = float_to_centi(s_runtime.final_command.target_speed_kph);
    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        out->target_steer_centi_deg[wheel] =
            float_to_centi(s_runtime.final_command.target_steer_deg[wheel]);
    }
    out->brake_release = s_runtime.final_command.brake_release;
    out->high_voltage_enable = s_runtime.final_command.high_voltage_enable;
    out->hydraulic_enable = s_runtime.final_command.hydraulic_enable;
    out->hydraulic_valve_mask = s_runtime.final_command.hydraulic_valve_mask;
    out->power_result = s_runtime.executor.power_result;
    out->motion_result = s_runtime.executor.motion_result;
    out->lift_hydraulic_result = s_runtime.executor.lift_hydraulic_result;
    out->local_io_result = s_runtime.executor.local_io_result;
    out->warning_light_result = s_runtime.executor.warning_light_result;
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
#if ECU_ENABLE_CANOPENNODE
    canopen_master_service_process(&s_runtime.can2_canopen_diag, now_ms);
#else
    can_bus_hw_poll_can2_rx(&s_runtime.can2_drive_debug);
#endif
    refresh_can2_feedback();
}

void ecu_task_remote_manager_step(uint32_t now_ms)
{
    ecu_runtime_init_once(now_ms);

    const ecu_config_t *config = ecu_config_default();
    remote_input_snapshot_t remote_input;
    remote_preconditions_t preconditions;

    sbus_service_poll(&s_runtime.sbus, now_ms);
    sbus_service_get_snapshot(&s_runtime.sbus, &s_runtime.sbus_snapshot);
    build_remote_input_snapshot(&s_runtime.sbus_snapshot, config, now_ms, &remote_input);
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
    const ecu_hardware_config_t *hardware_config = ecu_hardware_config_default();
    can_bus_hw_poll_can1_rx(&s_runtime.can1_power);
    power_device_process_rx(&s_runtime.power_device,
                            &s_runtime.can1_power,
                            hardware_config,
                            now_ms);
    s_runtime.executor.power_result =
        power_device_apply(&s_runtime.power_device,
                           &s_runtime.can1_power,
                           hardware_config,
                           s_runtime.final_command.high_voltage_enable,
                           now_ms);
    refresh_power_feedback();
}

void ecu_task_can3_lift_hydraulic_step(uint32_t now_ms)
{
    ecu_runtime_init_once(now_ms);
    refresh_lift_hydraulic_feedback();
}

void ecu_task_io_service_step(uint32_t now_ms)
{
    ecu_runtime_init_once(now_ms);
    const ecu_hardware_config_t *hardware_config = ecu_hardware_config_default();
    analog_modbus_device_process(&s_runtime.analog_modbus_adc,
                                 &s_runtime.adc_modbus_master,
                                 &s_runtime.rs485_1_hw,
                                 &s_runtime.analog_inputs,
                                 hardware_config,
                                 now_ms);
    analog_input_service_get_snapshot(&s_runtime.analog_inputs, &s_runtime.analog_snapshot);
    refresh_local_io_feedback();
}

void ecu_task_diag_cpu0_step(uint32_t now_ms)
{
    ecu_runtime_init_once(now_ms);
    (void)ipc_snapshot_publish(&s_runtime.cpu0_snapshot,
                               &s_runtime.vehicle_state,
                               sizeof(s_runtime.vehicle_state),
                               now_ms);
#if ECU_ENABLE_DEBUG_MONITOR
    if ((now_ms - s_runtime.last_debug_monitor_ms) >= ECU_DEBUG_MONITOR_PERIOD_MS) {
        runtime_monitor_snapshot_t monitor_snapshot;
        s_runtime.last_debug_monitor_ms = now_ms;
        build_runtime_monitor_snapshot(now_ms, &monitor_snapshot);
        runtime_monitor_print_cpu0(&monitor_snapshot);
    }
#endif
}

void ecu_task_cpu1_service_step(uint32_t now_ms)
{
    (void)now_ms;
}
