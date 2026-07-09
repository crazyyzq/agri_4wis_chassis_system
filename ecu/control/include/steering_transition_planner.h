#ifndef STEERING_TRANSITION_PLANNER_H
#define STEERING_TRANSITION_PLANNER_H

#include <stdbool.h>
#include <stdint.h>

#include "ecu_config.h"
#include "ecu_types.h"

/* Four-axis fixed-posture transition planner.
 *
 * This module is deliberately independent from CANopen and PDO scheduling. It
 * only converts one fixed steering target snapshot into intermediate absolute
 * position targets.  The CAN2 motion task still owns all realtime RPDO output.
 */
typedef struct {
    bool active;
    bool completed;
    bool rejected_stale_feedback;
    ecu_motion_mode_t active_mode;
    uint32_t transition_id;
    uint32_t start_ms;
    uint32_t duration_ms;
    uint8_t feedback_fresh_mask;
    uint8_t moving_axis_mask;
    int32_t max_distance_counts;
    int32_t actual_position_counts[ECU_WHEEL_COUNT];
    int32_t start_counts[ECU_WHEEL_COUNT];
    int32_t requested_target_counts[ECU_WHEEL_COUNT];
    int32_t output_target_counts[ECU_WHEEL_COUNT];
    int32_t error_counts[ECU_WHEEL_COUNT];
} steering_transition_planner_t;

void steering_transition_planner_init(steering_transition_planner_t *planner);
void steering_transition_planner_reset(steering_transition_planner_t *planner);
bool steering_transition_planner_mode_is_fixed_posture(ecu_motion_mode_t mode);

/* Update one fixed-posture trajectory.
 *
 * Parameters:
 * - feedback_fresh_mask: bit0..bit3 indicate fresh TPDO actual positions.
 * - actual_position_counts: measured steering actual positions, one per axis.
 * - requested_target_counts: final absolute steering targets.
 * - output_target_counts: planner output for this control cycle.
 *
 * Return false only when a fixed-posture transition would need fresh feedback
 * but one or more axes are stale.  The caller must then hold safe steering
 * targets and prevent drive motion through the normal safety path.
 */
bool steering_transition_planner_update(
    steering_transition_planner_t *planner,
    ecu_motion_mode_t mode,
    uint32_t now_ms,
    uint8_t feedback_fresh_mask,
    const int32_t actual_position_counts[ECU_WHEEL_COUNT],
    const int32_t requested_target_counts[ECU_WHEEL_COUNT],
    int32_t output_target_counts[ECU_WHEEL_COUNT]);

#endif /* STEERING_TRANSITION_PLANNER_H */
