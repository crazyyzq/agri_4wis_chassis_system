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
#include "status_led_service.h"

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
    uint16_t sbus_ppm_channels[ECU_SBUS_CHANNEL_COUNT];
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
    uint32_t can4_test_tx_count;
    uint32_t can4_test_rx_count;
    uint32_t can4_test_error_count;
    uint8_t can4_test_rx_buffer_status;
    uint8_t can4_test_tx_rx_flags;
    uint8_t can4_test_error_flags;
    uint8_t can4_test_receive_error_count;
    uint8_t can4_test_transmit_error_count;
    uint8_t can4_test_last_error_kind;
    uint32_t can4_test_last_tx_id;
    uint8_t can4_test_last_tx_size;
    uint8_t can4_test_last_tx_data[8];
    power_device_snapshot_t power_snapshot;
    modbus_master_snapshot_t modbus_adc_master;
    analog_modbus_device_state_t analog_modbus_adc;
    ecu_hardware_feedback_snapshot_t hardware_feedback;

    remote_link_state_t link_state;
    remote_arm_state_t arm_state;
    remote_estop_state_t estop_state;
    remote_gear_state_t gear_state;
    remote_power_state_t power_state;
    remote_authority_state_t authority_state;
    remote_adjust_state_t adjust_state;
    int16_t remote_steer_per_mille;
    int16_t remote_throttle_per_mille;
    int16_t remote_clearance_per_mille;
    int16_t remote_track_per_mille;
    bool steer_zero_calibration_request;
    bool b1_zero_calibration_pressed_latched;
    bool b1_zero_calibration_raw_request;
    bool b1_zero_calibration_gate_blocked;
    uint8_t b1_zero_calibration_press_count;
    status_led_pattern_t status_led_pattern;
    diag_code_t diagnostic;

    ecu_command_source_t source;
    ecu_motion_mode_t motion_mode;
    ecu_gear_request_t active_gear;
    int32_t target_speed_milli_mps;
    int32_t target_steer_centi_deg[ECU_WHEEL_COUNT];
    bool brake_release;
    bool high_voltage_enable;
    bool high_voltage_feedback_ready;
    bool high_voltage_relay_latched;
    bool commissioning_power_debug_active;
    int32_t target_height_milli_mm;
    int32_t height_rate_milli_mm_s;
    bool hydraulic_enable;
    uint32_t hydraulic_valve_mask;
    uint32_t hydraulic_requested_valve_mask;
    uint32_t hydraulic_applied_valve_mask;
    uint32_t hydraulic_interlocked_valve_mask;
    uint32_t hydraulic_valve_interlock_reject_count;
    uint8_t hydraulic_pump_state;
    bool hydraulic_pump_feedback_valid;
    int32_t hydraulic_pump_actual_velocity_units;
    uint32_t hydraulic_pump_start_timeout_count;
    uint8_t lift_interpolation_state;
    int8_t lift_requested_direction;
    int8_t lift_active_direction;
    uint8_t lift_feedback_fresh_mask;
    uint8_t lift_preload_points_completed;
    uint32_t lift_interpolation_queued_count;
    uint32_t lift_interpolation_reject_count;
    uint32_t lift_interpolation_failure_count;
    uint32_t lift_interpolation_recovery_count;
    int32_t lift_stream_planned_delta_counts;
    int32_t lift_actual_position_counts[ECU_WHEEL_COUNT];
    int32_t lift_target_position_counts[ECU_WHEEL_COUNT];
    bool steer_normal_pdo_allowed;
    bool steer_safety_inhibited;
    uint8_t steer_inhibit_reason;
    uint32_t steer_safety_inhibit_count;
    uint32_t steer_last_allowed_to_inhibited_ms;
    bool steer_safe_stop_pending;
    uint8_t steer_commission_state;
    uint8_t steer_commission_axis_mask;
    uint8_t steer_commission_nmt_sent_mask;
    uint32_t steer_commission_authorization_clear_count;
    bool steer_commission_post_command_tpdo_pending;
    uint8_t steer_commission_post_command_axis_mask;
    uint8_t steer_commission_post_command_missing_mask;
    uint32_t steer_commission_post_command_timeout_count;
    uint32_t can2_realtime_transient_recovery_count;
    uint32_t can2_realtime_consecutive_failure_count;
    uint32_t can2_realtime_last_recovery_ms;
    bool presteer_drive_hold_active;
    bool presteer_target_reached;
    bool track_assist_steer_approximately_ready;
    uint8_t presteer_mode;
    uint8_t presteer_missing_axis_mask;
    uint32_t presteer_timeout_count;
    uint8_t steer_zero_calibration_state;
    uint8_t steer_zero_calibration_done_mask;
    uint8_t steer_zero_calibration_fault_mask;
    uint32_t steer_zero_calibration_request_count;
    int32_t steer_zero_calibration_midpoint_counts[ECU_WHEEL_COUNT];
    int16_t steer_zero_calibration_peak_current_10ma[ECU_WHEEL_COUNT];
    bool steer_transition_active;
    bool steer_transition_completed;
    bool steer_transition_rejected_stale_feedback;
    uint32_t steer_transition_id;
    uint8_t steer_transition_feedback_fresh_mask;
    uint8_t steer_transition_moving_axis_mask;
    int32_t steer_transition_max_distance_counts;
    int32_t steer_transition_actual_counts[ECU_WHEEL_COUNT];
    int32_t steer_transition_output_counts[ECU_WHEEL_COUNT];
    int32_t steer_transition_error_counts[ECU_WHEEL_COUNT];
    bool steer_calibration_ram_override_enabled;
    bool steer_calibration_ram_override_valid;
    uint32_t steer_calibration_ram_override_sequence;
    steer_axis_calibration_t steer_effective_calibration[ECU_WHEEL_COUNT];

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
