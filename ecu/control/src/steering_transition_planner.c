#include <limits.h>
#include <string.h>

#include "steering_transition_planner.h"

static int32_t planner_abs_delta(int32_t a, int32_t b)
{
    int64_t delta = (int64_t)a - (int64_t)b;
    if (delta < 0) {
        delta = -delta;
    }
    return delta > INT32_MAX ? INT32_MAX : (int32_t)delta;
}

static uint32_t clamp_u32(uint32_t value, uint32_t min_value, uint32_t max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

/* Smoothstep with Q15 progress: 3t^2 - 2t^3.
 *
 * It starts and ends with zero slope, which avoids the visible start/stop kick
 * seen when fixed steering postures are commanded as independent step targets.
 */
static uint32_t smoothstep_q15(uint32_t progress_q15)
{
    if (progress_q15 >= 32768U) {
        return 32768U;
    }
    uint64_t t = progress_q15;
    uint64_t t2 = t * t;
    uint64_t three_minus_2t = (3ULL * 32768ULL) - (2ULL * t);
    return (uint32_t)((t2 * three_minus_2t) / (32768ULL * 32768ULL));
}

static int32_t interpolate_counts(int32_t start, int32_t target, uint32_t smooth_q15)
{
    int64_t delta = (int64_t)target - (int64_t)start;
    int64_t value = (int64_t)start + ((delta * (int64_t)smooth_q15) / 32768LL);
    if (value > INT32_MAX) {
        return INT32_MAX;
    }
    if (value < INT32_MIN) {
        return INT32_MIN;
    }
    return (int32_t)value;
}

static bool targets_changed(const steering_transition_planner_t *planner,
                            const int32_t requested_target_counts[ECU_WHEEL_COUNT])
{
    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        if (planner_abs_delta(planner->requested_target_counts[wheel],
                              requested_target_counts[wheel]) >=
            ECU_CANOPEN_STEER_POSITION_TRIGGER_THRESHOLD_COUNTS) {
            return true;
        }
    }
    return false;
}

static uint32_t transition_duration_ms(int32_t max_distance_counts)
{
    if (max_distance_counts <= 0) {
        return ECU_STEER_FIXED_TRANSITION_MIN_MS;
    }

    uint64_t duration =
        ((uint64_t)(uint32_t)max_distance_counts * 1000ULL) /
        (uint64_t)ECU_STEER_FIXED_TRANSITION_MAX_SPEED_COUNTS_PER_SEC;
    if (duration > UINT32_MAX) {
        duration = UINT32_MAX;
    }
    return clamp_u32((uint32_t)duration,
                     ECU_STEER_FIXED_TRANSITION_MIN_MS,
                     ECU_STEER_FIXED_TRANSITION_MAX_MS);
}

void steering_transition_planner_init(steering_transition_planner_t *planner)
{
    if (planner != 0) {
        memset(planner, 0, sizeof(*planner));
        planner->active_mode = (ecu_motion_mode_t)0;
    }
}

void steering_transition_planner_reset(steering_transition_planner_t *planner)
{
    if (planner == 0) {
        return;
    }
    uint32_t transition_id = planner->transition_id;
    steering_transition_planner_init(planner);
    planner->transition_id = transition_id;
}

bool steering_transition_planner_mode_is_fixed_posture(ecu_motion_mode_t mode)
{
    return mode == ECU_MOTION_MODE_SPIN ||
           mode == ECU_MOTION_MODE_CRAB;
}

bool steering_transition_planner_update(
    steering_transition_planner_t *planner,
    ecu_motion_mode_t mode,
    uint32_t now_ms,
    uint8_t feedback_fresh_mask,
    const int32_t actual_position_counts[ECU_WHEEL_COUNT],
    const int32_t requested_target_counts[ECU_WHEEL_COUNT],
    int32_t output_target_counts[ECU_WHEEL_COUNT])
{
    if (planner == 0 || actual_position_counts == 0 ||
        requested_target_counts == 0 || output_target_counts == 0) {
        return false;
    }

    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        output_target_counts[wheel] = requested_target_counts[wheel];
        planner->actual_position_counts[wheel] = actual_position_counts[wheel];
    }
    planner->feedback_fresh_mask = feedback_fresh_mask;
    planner->rejected_stale_feedback = false;

    if (!steering_transition_planner_mode_is_fixed_posture(mode)) {
        steering_transition_planner_reset(planner);
        return true;
    }

    bool same_target =
        planner->active_mode == mode &&
        !targets_changed(planner, requested_target_counts);
    if (!planner->active && planner->completed && same_target) {
        for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
            planner->output_target_counts[wheel] = requested_target_counts[wheel];
            planner->error_counts[wheel] =
                requested_target_counts[wheel] - actual_position_counts[wheel];
            output_target_counts[wheel] = requested_target_counts[wheel];
        }
        return true;
    }

    bool new_transition = !planner->active || !same_target;
    if (new_transition) {
        if ((feedback_fresh_mask & ECU_STEER_REMOTE_COMMISSION_AXIS_MASK_ALL) !=
            ECU_STEER_REMOTE_COMMISSION_AXIS_MASK_ALL) {
            planner->active = false;
            planner->completed = false;
            planner->rejected_stale_feedback = true;
            return false;
        }

        planner->active_mode = mode;
        planner->start_ms = now_ms;
        planner->moving_axis_mask = 0U;
        planner->max_distance_counts = 0;
        for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
            int32_t distance = planner_abs_delta(actual_position_counts[wheel],
                                                 requested_target_counts[wheel]);
            planner->start_counts[wheel] = actual_position_counts[wheel];
            planner->requested_target_counts[wheel] =
                requested_target_counts[wheel];
            planner->error_counts[wheel] =
                requested_target_counts[wheel] - actual_position_counts[wheel];
            if (distance >= ECU_CANOPEN_STEER_POSITION_TRIGGER_THRESHOLD_COUNTS) {
                planner->moving_axis_mask |= (uint8_t)(1U << wheel);
            }
            if (distance > planner->max_distance_counts) {
                planner->max_distance_counts = distance;
            }
        }

        planner->duration_ms = transition_duration_ms(planner->max_distance_counts);
        planner->transition_id++;
        if (planner->transition_id == 0U) {
            planner->transition_id = 1U;
        }
        planner->active = planner->moving_axis_mask != 0U;
        planner->completed = !planner->active;
    }

    uint32_t elapsed_ms = now_ms - planner->start_ms;
    uint32_t progress_q15 = planner->duration_ms == 0U ||
                            elapsed_ms >= planner->duration_ms ?
                            32768U :
                            (uint32_t)(((uint64_t)elapsed_ms * 32768ULL) /
                                       planner->duration_ms);
    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        uint32_t smooth_q15 = smoothstep_q15(progress_q15);
        int32_t value = progress_q15 >= 32768U ?
            planner->requested_target_counts[wheel] :
            interpolate_counts(planner->start_counts[wheel],
                               planner->requested_target_counts[wheel],
                               smooth_q15);
        planner->output_target_counts[wheel] = value;
        planner->error_counts[wheel] =
            planner->requested_target_counts[wheel] - value;
        output_target_counts[wheel] = value;
    }

    if (progress_q15 >= 32768U) {
        planner->active = false;
        planner->completed = true;
    }
    return true;
}
