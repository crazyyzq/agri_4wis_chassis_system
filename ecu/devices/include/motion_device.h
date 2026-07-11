#ifndef MOTION_DEVICE_H
#define MOTION_DEVICE_H

#include <stdbool.h>
#include <stdint.h>

#include "canopen_master_service.h"
#include "ecu_config.h"
#include "ecu_types.h"
#include "steering_transition_planner.h"
#include "vehicle_types.h"

typedef enum {
    MOTION_STEER_INHIBIT_NONE = 0,
    MOTION_STEER_INHIBIT_ESTOP_LATCHED,
    MOTION_STEER_INHIBIT_SBUS_OFFLINE,
    MOTION_STEER_INHIBIT_REMOTE_DISARMED,
    MOTION_STEER_INHIBIT_GEAR_PARK,
    MOTION_STEER_INHIBIT_AXIS_NOT_READY,
    MOTION_STEER_INHIBIT_GROUP_DEGRADED,
    MOTION_STEER_INHIBIT_COMMAND_SOURCE_NOT_AUTHORIZED,
    MOTION_STEER_INHIBIT_BENCH_MODE_DISABLED
} motion_steer_inhibit_reason_t;

typedef enum {
    MOTION_STEER_AXIS_UNSEEN = 0,
    MOTION_STEER_AXIS_SDO_PENDING,
    MOTION_STEER_AXIS_SDO_TIMEOUT,
    MOTION_STEER_AXIS_SDO_ABORT,
    MOTION_STEER_AXIS_CONFIG_UNVERIFIED,
    MOTION_STEER_AXIS_READY,
    MOTION_STEER_AXIS_FAULT
} motion_steer_axis_config_state_t;

typedef enum {
    STEER_REMOTE_COMMISSION_DISABLED = 0,
    STEER_REMOTE_COMMISSION_WAIT_AUTH,
    STEER_REMOTE_COMMISSION_WAIT_CALIBRATION,
    STEER_REMOTE_COMMISSION_WAIT_NEUTRAL,
    STEER_REMOTE_COMMISSION_TPDO_MONITOR,
    STEER_REMOTE_COMMISSION_AXIS_READY,
    STEER_REMOTE_COMMISSION_CENTERING,
    STEER_REMOTE_COMMISSION_WAIT_SYNC_TX_COMPLETE,
    STEER_REMOTE_COMMISSION_WAIT_CENTER_SETTLE,
    STEER_REMOTE_COMMISSION_ACTIVE,
    STEER_REMOTE_COMMISSION_WAIT_POST_COMMAND_TPDO,
    STEER_REMOTE_COMMISSION_FAULT
} steer_remote_commission_state_t;

typedef enum {
    MOTION_DRIVE_COMMAND_VELOCITY = 0,
    MOTION_DRIVE_COMMAND_CURRENT
} motion_drive_command_kind_t;

typedef enum {
    MOTION_STEER_ZERO_CAL_IDLE = 0,
    MOTION_STEER_ZERO_CAL_SETUP,
    MOTION_STEER_ZERO_CAL_SEARCH_LEFT,
    MOTION_STEER_ZERO_CAL_RETREAT_LEFT,
    MOTION_STEER_ZERO_CAL_SEARCH_RIGHT,
    MOTION_STEER_ZERO_CAL_RETREAT_RIGHT,
    MOTION_STEER_ZERO_CAL_RETURN_MID,
    MOTION_STEER_ZERO_CAL_WRITE_ZERO,
    MOTION_STEER_ZERO_CAL_COMPLETE,
    MOTION_STEER_ZERO_CAL_FAULT
} motion_steer_zero_calibration_state_t;

typedef struct {
    uint32_t apply_count;
    uint32_t skipped_count;
    uint32_t last_motion_command_sequence;
    uint32_t last_motion_command_queue_ms;
    uint32_t last_target_update_ms;
    uint32_t drive_last_target_update_ms[ECU_WHEEL_COUNT];
    uint32_t steer_last_target_update_ms[ECU_WHEEL_COUNT];
    uint32_t steer_realtime_last_flush_ms;
    uint32_t steer_group_sequence_counter;
    uint32_t steer_active_group_sequence;
    uint32_t steer_active_group_submit_ms;
    int32_t steer_active_group_target_counts[ECU_WHEEL_COUNT];
    int32_t steer_next_group_target_counts[ECU_WHEEL_COUNT];
    uint8_t steer_active_group_axis_mask;
    bool steer_group_active;
    bool steer_active_group_node5_only;
    bool steer_next_group_valid;
    bool steer_group_degraded;
    bool steer_normal_pdo_allowed;
    bool steer_safety_inhibited;
    motion_steer_inhibit_reason_t steer_inhibit_reason;
    uint32_t steer_safety_inhibit_count;
    uint32_t steer_last_allowed_to_inhibited_ms;
    bool steer_safe_stop_pending;
    uint32_t steer_group_complete_count;
    uint32_t steer_group_failure_count;
    bool steer_zero_calibration_requested;
    uint32_t steer_zero_calibration_request_count;
    uint32_t steer_zero_calibration_last_request_ms;
    motion_steer_zero_calibration_state_t steer_zero_calibration_state;
    uint32_t steer_zero_calibration_state_enter_ms;
    uint32_t steer_zero_calibration_last_pdo_ms;
    uint32_t steer_zero_calibration_active_group_sequence;
    bool steer_zero_calibration_group_active;
    uint8_t steer_zero_calibration_done_mask;
    uint8_t steer_zero_calibration_fault_mask;
    uint8_t steer_zero_calibration_setup_node_index;
    uint8_t steer_zero_calibration_setup_step;
    uint8_t steer_zero_calibration_zero_write_index;
    bool steer_zero_calibration_sdo_active;
    uint8_t steer_zero_calibration_sdo_node_id;
    uint16_t steer_zero_calibration_sdo_index;
    uint8_t steer_zero_calibration_sdo_subindex;
    uint8_t steer_zero_calibration_sdo_size;
    int32_t steer_zero_calibration_sdo_value;
    uint32_t steer_zero_calibration_sdo_download_count_before;
    uint32_t steer_zero_calibration_sdo_abort_count_before;
    uint32_t steer_zero_calibration_sdo_start_ms;
    int32_t steer_zero_calibration_direction_start_counts[ECU_WHEEL_COUNT];
    int32_t steer_zero_calibration_left_hit_counts[ECU_WHEEL_COUNT];
    int32_t steer_zero_calibration_right_hit_counts[ECU_WHEEL_COUNT];
    int32_t steer_zero_calibration_midpoint_counts[ECU_WHEEL_COUNT];
    int16_t steer_zero_calibration_peak_current_10ma[ECU_WHEEL_COUNT];
    uint32_t steer_zero_calibration_zero_speed_since_ms[ECU_WHEEL_COUNT];
    int32_t steer_zero_calibration_return_last_error_counts[ECU_WHEEL_COUNT];
    uint8_t steer_zero_calibration_midpoint_stable_samples[ECU_WHEEL_COUNT];
    uint32_t can2_realtime_transient_recovery_count;
    uint32_t can2_realtime_consecutive_failure_count;
    uint32_t can2_realtime_last_recovery_ms;
    bool steer_group_clean_cancelled;
    bool steer_group_trigger_partial_failure;
    uint32_t steer_last_clean_cancel_ms;
    uint32_t steer_last_partial_failure_ms;
    steer_remote_commission_state_t steer_commission_state;
    uint8_t selected_axis_mask;
    uint32_t steer_commission_neutral_since_ms;
    uint32_t steer_commission_last_sync_ms;
    uint32_t steer_commission_sync_wait_start_ms;
    uint32_t steer_commission_sync_complete_ms;
    uint32_t steer_commission_sync_complete_count_before;
    uint32_t steer_commission_authorization_clear_count;
    uint8_t steer_commission_nmt_sent_mask;
    bool steer_commission_centered;
    bool steer_commission_post_command_is_centering;
    bool steer_commission_post_command_tpdo_pending;
    uint8_t steer_commission_post_command_axis_mask;
    uint8_t steer_commission_post_command_missing_mask;
    uint32_t steer_commission_post_command_start_ms;
    uint32_t steer_commission_last_post_command_timeout_ms;
    uint32_t steer_commission_post_command_timeout_count;
    uint32_t steer_commission_tpdo0_count_before[ECU_WHEEL_COUNT];
    uint32_t steer_commission_tpdo1_count_before[ECU_WHEEL_COUNT];
    bool steer_commission_ramped_target_valid[ECU_WHEEL_COUNT];
    int32_t steer_commission_ramped_target_counts[ECU_WHEEL_COUNT];
    uint32_t steer_commission_ramp_last_ms;
    bool drive_velocity_mode_ready[ECU_WHEEL_COUNT];
    bool drive_last_velocity_valid[ECU_WHEEL_COUNT];
    int32_t drive_last_velocity_units[ECU_WHEEL_COUNT];
    int16_t drive_last_current_10ma[ECU_WHEEL_COUNT];
    motion_drive_command_kind_t drive_last_command_kind[ECU_WHEEL_COUNT];
    bool drive_last_enable_requested[ECU_WHEEL_COUNT];
    uint8_t can2_motion_operational_nmt_sent_mask;
    uint32_t can2_motion_operational_nmt_last_ms;
    uint8_t can2_node_recovery_pending_mask;
    uint32_t can2_node_bootup_seen[ECU_CANOPEN_CAN2_MOTION_NODE_COUNT];
    uint32_t can2_node_recovery_last_ms[ECU_CANOPEN_CAN2_MOTION_NODE_COUNT];
    uint32_t can2_node_recovery_count[ECU_CANOPEN_CAN2_MOTION_NODE_COUNT];
    uint8_t can2_node_recovery_attempts[ECU_CANOPEN_CAN2_MOTION_NODE_COUNT];
    bool drive_safe_stop_pending;
    uint32_t drive_safe_stop_count;
    uint32_t can2_feedback_last_sync_ms;
    uint32_t drive_realtime_last_flush_ms;
    uint32_t drive_group_sequence_counter;
    uint32_t drive_active_group_sequence;
    uint32_t drive_active_group_submit_ms;
    int32_t drive_active_group_velocity_units[ECU_WHEEL_COUNT];
    int16_t drive_active_group_current_10ma[ECU_WHEEL_COUNT];
    motion_drive_command_kind_t drive_active_group_kind;
    bool drive_active_group_enable_requested[ECU_WHEEL_COUNT];
    int32_t drive_next_group_velocity_units[ECU_WHEEL_COUNT];
    int16_t drive_next_group_current_10ma[ECU_WHEEL_COUNT];
    motion_drive_command_kind_t drive_next_group_kind;
    bool drive_next_group_enable_requested[ECU_WHEEL_COUNT];
    bool drive_group_active;
    bool drive_next_group_valid;
    bool drive_latest_velocity_valid[ECU_WHEEL_COUNT];
    int32_t drive_latest_velocity_units[ECU_WHEEL_COUNT];
    int16_t drive_latest_current_10ma[ECU_WHEEL_COUNT];
    motion_drive_command_kind_t drive_latest_command_kind[ECU_WHEEL_COUNT];
    bool drive_latest_enable_requested[ECU_WHEEL_COUNT];
    bool drive_pending_velocity[ECU_WHEEL_COUNT];
    bool drive_realtime_enabled[ECU_WHEEL_COUNT];
    bool presteer_drive_hold_active;
    bool presteer_target_reached;
    bool track_assist_steer_approximately_ready;
    ecu_motion_mode_t presteer_mode;
    uint8_t presteer_missing_axis_mask;
    uint8_t track_assist_missing_axis_mask;
    uint32_t presteer_hold_start_ms;
    uint32_t track_assist_steer_ready_since_ms;
    uint32_t track_assist_steer_ready_eval_ms;
    uint32_t presteer_timeout_count;
    uint32_t presteer_last_timeout_ms;
    uint32_t drive_group_complete_count;
    uint32_t drive_group_failure_count;
    uint32_t drive_pdo_tx_error_count[ECU_WHEEL_COUNT];
    bool steer_pdo_configured[ECU_WHEEL_COUNT];
    bool steer_position_mode_ready[ECU_WHEEL_COUNT];
    motion_steer_axis_config_state_t steer_axis_config_state[ECU_WHEEL_COUNT];
    bool steer_axis_remote_verified[ECU_WHEEL_COUNT];
    uint32_t steer_setup_queued_ms[ECU_WHEEL_COUNT];
    bool steer_realtime_enabled[ECU_WHEEL_COUNT];
    bool steer_latest_target_valid[ECU_WHEEL_COUNT];
    int32_t steer_latest_target_counts[ECU_WHEEL_COUNT];
    bool steer_commanded_target_valid[ECU_WHEEL_COUNT];
    int32_t steer_commanded_target_counts[ECU_WHEEL_COUNT];
    int32_t steer_commanded_velocity_counts_per_sec[ECU_WHEEL_COUNT];
    steering_transition_planner_t steer_transition_planner;
    bool steer_pending_target[ECU_WHEEL_COUNT];
    bool steer_last_commanded_position_valid[ECU_WHEEL_COUNT];
    int32_t steer_last_commanded_position_counts[ECU_WHEEL_COUNT];
    bool steer_last_position_valid[ECU_WHEEL_COUNT];
    int32_t steer_last_position_counts[ECU_WHEEL_COUNT];
    uint32_t steer_last_limit_read_ms[ECU_WHEEL_COUNT];
    bool steer_positive_limit[ECU_WHEEL_COUNT];
    bool steer_negative_limit[ECU_WHEEL_COUNT];
    uint32_t steer_pdo_tx_error_count[ECU_WHEEL_COUNT];
    bool last_motion_command_valid;
    ecu_device_apply_result_t last_result;
    vehicle_actuator_command_t last_motion_command;
} motion_device_state_t;

/* Initialize the CPU0-owned motion device adapter.
 *
 * Owner: task_can2_motion / vehicle executor path on CPU0.
 * ISR: not safe.
 */
void motion_device_init(motion_device_state_t *state);

/* Convert one remote steering angle into per-axis absolute target counts for
 * V8 remote steering commissioning.  Selected axes require valid calibration;
 * unselected axes are left at zero by this pure helper and are not transmitted.
 */
bool steer_commissioning_build_targets(
    const steer_axis_calibration_t calibration[ECU_WHEEL_COUNT],
    uint8_t enabled_axis_mask,
    float remote_steer_deg,
    int32_t out_target_counts[ECU_WHEEL_COUNT]);

/* Read the effective steering commissioning calibration.
 *
 * Static configuration is fail-closed by default.  A field engineer may write
 * g_ecu_steer_calibration_override through J-Link RAM watch/memory access for
 * a RAM-only commissioning session.  The override is never saved to flash and
 * is accepted only when magic, enable, sequence stability and selected-axis
 * limits are valid.
 */
bool motion_device_get_effective_steer_calibration(
    const ecu_hardware_config_t *config,
    uint8_t enabled_axis_mask,
    steer_axis_calibration_t out_calibration[ECU_WHEEL_COUNT],
    bool *using_ram_override);

/* Apply final drive, steering and brake intent to CAN2 motion nodes.
 *
 * Units: speed is m/s, steering is degrees, brake_release is logical.
 * Dependencies: CAN2 service and project drive/steer CANopen mappings.
 * Timing: unchanged commands are periodically re-queued because a successful
 * local SDO enqueue does not guarantee the remote drive accepted the transfer.
 * Failure behavior: returns one aggregate result after attempting configured
 * wheel commands; safety decisions are not made here.
 */
ecu_device_apply_result_t motion_device_apply(motion_device_state_t *state,
                                              canopen_master_service_t *canopen,
                                              const ecu_hardware_config_t *config,
                                              const vehicle_actuator_command_t *command,
                                              uint32_t command_sequence,
                                              uint32_t now_ms);

/* Flush realtime steering PDOs from the latest cached motion command.
 *
 * The vehicle task calls motion_device_apply() to cache one coherent four-wheel
 * target.  The CAN2 motion task calls this function at the bus cadence; it owns
 * all steering RPDO transmissions and never waits for one axis to complete
 * before updating the other axes.
 */
ecu_device_apply_result_t motion_device_flush_realtime(motion_device_state_t *state,
                                                       canopen_master_service_t *canopen,
                                                       const ecu_hardware_config_t *config,
                                                       uint32_t now_ms);

#endif /* MOTION_DEVICE_H */
