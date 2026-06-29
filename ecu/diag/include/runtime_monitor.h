/* Runtime monitor print contract for the CPU0 debug UART.
 *
 * This module is intentionally diagnostic-only. It receives a compact snapshot
 * from the task layer and prints it to the board debug console. It must not own
 * control state, decide safety policy, access hardware registers or call the
 * vehicle executor.
 */
#ifndef RUNTIME_MONITOR_H
#define RUNTIME_MONITOR_H

#include <stdbool.h>
#include <stdint.h>

#include "canopen_master_service.h"
#include "analog_modbus_device.h"
#include "ecu_config.h"
#include "diag_codes.h"
#include "ecu_types.h"
#include "modbus_master_service.h"
#include "power_device.h"
#include "remote_types.h"

typedef struct {
    uint32_t now_ms;
    uint32_t executor_sequence;

    bool sbus_valid;
    bool sbus_connected;
    bool sbus_failsafe;
    bool sbus_frame_lost;
    bool sbus_channel17;
    bool sbus_channel18;
    uint16_t sbus_channels[ECU_SBUS_CHANNEL_COUNT];
    uint32_t sbus_frame_count;
    uint32_t sbus_decode_error_count;
    uint32_t sbus_uart_error_count;
    uint32_t sbus_last_frame_ms;

    uint32_t can2_rx_count;
    uint32_t can2_error_count;
    uint8_t can2_rx_buffer_status;
    uint8_t can2_tx_rx_flags;
    uint8_t can2_error_flags;
    uint8_t can2_receive_error_count;
    uint8_t can2_transmit_error_count;
    uint8_t can2_last_error_kind;
    uint32_t can2_last_rx_id;
    uint8_t can2_last_rx_size;
    bool can2_last_rx_extended;
    bool can2_last_rx_remote;
    uint8_t can2_last_rx_data[8];
    bool can2_canopen_initialized;
    canopen_master_snapshot_t can2_canopen_snapshot;
    bool can3_canopen_initialized;
    canopen_master_snapshot_t can3_canopen_snapshot;
    canopen_master_debug_command_t canopen_command;
    uint32_t can1_tx_count;
    uint32_t can1_rx_count;
    uint32_t can1_error_count;
    uint32_t can1_last_tx_id;
    uint8_t can1_last_tx_size;
    bool can1_last_tx_extended;
    uint32_t can1_last_rx_id;
    uint8_t can1_last_rx_size;
    bool can1_last_rx_extended;
    uint8_t can1_last_rx_data[8];
    power_device_snapshot_t power_snapshot;
    modbus_master_snapshot_t modbus_adc_master;
    analog_modbus_device_state_t analog_modbus_adc;
    ecu_hardware_feedback_snapshot_t hardware_feedback;

    remote_link_state_t link_state;
    remote_estop_state_t estop_state;
    diag_code_t diagnostic;

    ecu_command_source_t source;
    ecu_motion_mode_t motion_mode;
    ecu_gear_request_t active_gear;
    int32_t target_speed_centi_kph;
    int32_t target_steer_centi_deg[ECU_WHEEL_COUNT];
    bool brake_release;
    bool high_voltage_enable;
    bool hydraulic_enable;
    uint32_t hydraulic_valve_mask;

    ecu_device_apply_result_t power_result;
    ecu_device_apply_result_t motion_result;
    ecu_device_apply_result_t lift_hydraulic_result;
    ecu_device_apply_result_t local_io_result;
    ecu_device_apply_result_t warning_light_result;
} runtime_monitor_snapshot_t;

/* Print one CPU0 runtime snapshot.
 *
 * Caller: CPU0 diagnostic task only.
 * Rate limit: handled by the task layer before this function is called.
 */
void runtime_monitor_print_cpu0(const runtime_monitor_snapshot_t *snapshot);

#endif /* RUNTIME_MONITOR_H */
