#ifndef MOTION_DEVICE_H
#define MOTION_DEVICE_H

#include <stdbool.h>
#include <stdint.h>

#include "canopen_master_service.h"
#include "ecu_config.h"
#include "ecu_types.h"
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
    bool steer_group_clean_cancelled;
    bool steer_group_trigger_partial_failure;
    uint32_t steer_last_clean_cancel_ms;
    uint32_t steer_last_partial_failure_ms;
    bool drive_velocity_mode_ready[ECU_WHEEL_COUNT];
    bool drive_last_velocity_valid[ECU_WHEEL_COUNT];
    int32_t drive_last_velocity_units[ECU_WHEEL_COUNT];
    bool steer_pdo_configured[ECU_WHEEL_COUNT];
    bool steer_position_mode_ready[ECU_WHEEL_COUNT];
    motion_steer_axis_config_state_t steer_axis_config_state[ECU_WHEEL_COUNT];
    bool steer_axis_remote_verified[ECU_WHEEL_COUNT];
    uint32_t steer_setup_queued_ms[ECU_WHEEL_COUNT];
    bool steer_realtime_enabled[ECU_WHEEL_COUNT];
    bool steer_latest_target_valid[ECU_WHEEL_COUNT];
    int32_t steer_latest_target_counts[ECU_WHEEL_COUNT];
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

/* Apply final drive, steering and brake intent to CAN2 motion nodes.
 *
 * Units: speed is kph, steering is degrees, brake_release is logical.
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
