#include "motion_setpoint_shaper.h"

#include <limits.h>

#include "ecu_config.h"

static int32_t abs_delta_saturating(int32_t requested, int32_t current)
{
    int64_t delta = (int64_t)requested - (int64_t)current;
    if (delta < 0) {
        delta = -delta;
    }
    return delta > INT32_MAX ? INT32_MAX : (int32_t)delta;
}

static int32_t max_abs_error(const int32_t current[ECU_WHEEL_COUNT],
                             const int32_t requested[ECU_WHEEL_COUNT])
{
    int32_t maximum = 0;
    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        int32_t error = abs_delta_saturating(requested[wheel], current[wheel]);
        if (error > maximum) {
            maximum = error;
        }
    }
    return maximum;
}

static uint32_t bounded_elapsed_ms(uint32_t elapsed_ms, uint32_t maximum_ms)
{
    if (elapsed_ms == 0U) {
        return 1U;
    }
    return elapsed_ms > maximum_ms ? maximum_ms : elapsed_ms;
}

static int32_t approach_signed(int32_t current,
                               int32_t requested,
                               int32_t max_delta)
{
    if (max_delta < 1) {
        max_delta = 1;
    }

    int64_t delta = (int64_t)requested - (int64_t)current;
    if (delta > (int64_t)max_delta) {
        return current + max_delta;
    }
    if (delta < -(int64_t)max_delta) {
        return current - max_delta;
    }
    return requested;
}

static int32_t abs_i32_saturating(int32_t value)
{
    if (value == INT32_MIN) {
        return INT32_MAX;
    }
    return value < 0 ? -value : value;
}

static int32_t clamp_steering_velocity(int32_t velocity)
{
    if (velocity > ECU_STEER_MAX_POSITION_COUNTS_PER_SEC) {
        return ECU_STEER_MAX_POSITION_COUNTS_PER_SEC;
    }
    if (velocity < -ECU_STEER_MAX_POSITION_COUNTS_PER_SEC) {
        return -ECU_STEER_MAX_POSITION_COUNTS_PER_SEC;
    }
    return velocity;
}

static bool signs_are_opposite(int32_t first, int32_t second)
{
    return (first > 0 && second < 0) || (first < 0 && second > 0);
}

static int32_t scale_axis_acceleration(int32_t group_accel,
                                       int32_t axis_speed,
                                       int32_t group_speed)
{
    if (group_accel <= 0 || axis_speed <= 0 || group_speed <= 0) {
        return group_accel > 0 ? 1 : 0;
    }

    int64_t scaled =
        ((int64_t)group_accel * (int64_t)axis_speed) /
        (int64_t)group_speed;
    if (scaled < 1) {
        return 1;
    }
    return scaled > INT32_MAX ? INT32_MAX : (int32_t)scaled;
}

/* Fixed-iteration integer square root for deterministic braking calculations.
 * The bitwise algorithm performs at most 32 iterations for a 64-bit input and
 * avoids floating point in the CAN2 realtime path.
 */
static uint32_t integer_sqrt_u64(uint64_t value)
{
    uint64_t result = 0U;
    uint64_t bit = 1ULL << 62;

    while (bit > value) {
        bit >>= 2U;
    }
    while (bit != 0U) {
        if (value >= result + bit) {
            value -= result + bit;
            result = (result >> 1U) + bit;
        } else {
            result >>= 1U;
        }
        bit >>= 2U;
    }

    return result > UINT32_MAX ? UINT32_MAX : (uint32_t)result;
}

static void copy_group(const int32_t source[ECU_WHEEL_COUNT],
                       int32_t destination[ECU_WHEEL_COUNT])
{
    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        destination[wheel] = source[wheel];
    }
}

static void advance_group_by_common_step(
    const int32_t current[ECU_WHEEL_COUNT],
    const int32_t requested[ECU_WHEEL_COUNT],
    int32_t maximum_error,
    int32_t maximum_step,
    int32_t output[ECU_WHEEL_COUNT])
{
    if (maximum_error <= 0 || maximum_step >= maximum_error) {
        copy_group(requested, output);
        return;
    }

    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        int64_t error = (int64_t)requested[wheel] - (int64_t)current[wheel];
        int64_t scaled_step = (error * (int64_t)maximum_step) /
                              (int64_t)maximum_error;
        if (scaled_step == 0 && error != 0) {
            scaled_step = error > 0 ? 1 : -1;
        }
        int64_t next = (int64_t)current[wheel] + scaled_step;
        if ((error > 0 && next > requested[wheel]) ||
            (error < 0 && next < requested[wheel])) {
            next = requested[wheel];
        }
        output[wheel] = (int32_t)next;
    }
}

static motion_steer_follow_band_t select_steering_band(int32_t maximum_error,
                                                        int32_t *maximum_speed,
                                                        int32_t *maximum_accel)
{
    if (maximum_error <= ECU_CANOPEN_STEER_TARGET_HOLD_COUNTS) {
        *maximum_speed = 0;
        *maximum_accel = ECU_CANOPEN_STEER_TARGET_ACCEL_FINE_COUNTS_PER_SEC2;
        return MOTION_STEER_FOLLOW_BAND_HOLD;
    }
    if (maximum_error <= ECU_CANOPEN_STEER_TARGET_ERROR_FINE_COUNTS) {
        *maximum_speed = ECU_CANOPEN_STEER_TARGET_RATE_FINE_COUNTS_PER_SEC;
        *maximum_accel = ECU_CANOPEN_STEER_TARGET_ACCEL_FINE_COUNTS_PER_SEC2;
        return MOTION_STEER_FOLLOW_BAND_FINE;
    }
    if (maximum_error <= ECU_CANOPEN_STEER_TARGET_ERROR_SMALL_COUNTS) {
        *maximum_speed = ECU_CANOPEN_STEER_TARGET_RATE_SMALL_COUNTS_PER_SEC;
        *maximum_accel = ECU_CANOPEN_STEER_TARGET_ACCEL_SMALL_COUNTS_PER_SEC2;
        return MOTION_STEER_FOLLOW_BAND_SMALL;
    }
    if (maximum_error <= ECU_CANOPEN_STEER_TARGET_ERROR_MEDIUM_COUNTS) {
        *maximum_speed = ECU_CANOPEN_STEER_TARGET_RATE_MEDIUM_COUNTS_PER_SEC;
        *maximum_accel = ECU_CANOPEN_STEER_TARGET_ACCEL_MEDIUM_COUNTS_PER_SEC2;
        return MOTION_STEER_FOLLOW_BAND_MEDIUM;
    }

    *maximum_speed = ECU_CANOPEN_STEER_TARGET_RATE_LARGE_COUNTS_PER_SEC;
    *maximum_accel = ECU_CANOPEN_STEER_TARGET_ACCEL_LARGE_COUNTS_PER_SEC2;
    return MOTION_STEER_FOLLOW_BAND_LARGE;
}

bool motion_setpoint_shape_steering_group(
    const int32_t current_counts[ECU_WHEEL_COUNT],
    const int32_t requested_counts[ECU_WHEEL_COUNT],
    const int32_t current_velocity_counts_per_sec[ECU_WHEEL_COUNT],
    uint32_t elapsed_ms,
    int32_t output_counts[ECU_WHEEL_COUNT],
    int32_t output_velocity_counts_per_sec[ECU_WHEEL_COUNT],
    int32_t *output_group_speed_counts_per_sec,
    motion_steer_follow_band_t *output_band)
{
    if (current_counts == 0 || requested_counts == 0 ||
        current_velocity_counts_per_sec == 0 || output_counts == 0 ||
        output_velocity_counts_per_sec == 0 ||
        output_group_speed_counts_per_sec == 0 || output_band == 0) {
        return false;
    }

    int32_t maximum_error = max_abs_error(current_counts, requested_counts);
    int32_t band_speed = 0;
    int32_t band_accel = 0;
    motion_steer_follow_band_t band =
        select_steering_band(maximum_error, &band_speed, &band_accel);
    *output_band = band;

    uint32_t dt_ms = bounded_elapsed_ms(
        elapsed_ms, ECU_CANOPEN_STEER_SHAPER_MAX_ELAPSED_MS);

    /* The discrete band sets the allowed cruise speed. The braking envelope
     * sqrt(2*a*distance) then lowers that request continuously near the target.
     * This prevents a high-speed final snap without the stop/restart cycles
     * produced by a binary "brake now" decision.
     */
    int32_t desired_group_speed = 0;
    if (band != MOTION_STEER_FOLLOW_BAND_HOLD) {
        uint64_t braking_term =
            2ULL *
            (uint64_t)ECU_CANOPEN_STEER_TARGET_DECEL_COUNTS_PER_SEC2 *
            (uint64_t)(uint32_t)maximum_error;
        uint32_t braking_speed = integer_sqrt_u64(braking_term);
        desired_group_speed = braking_speed < (uint32_t)band_speed ?
            (int32_t)braking_speed : band_speed;
    }

    int32_t current_group_speed = 0;
    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        int32_t magnitude = abs_i32_saturating(
            clamp_steering_velocity(current_velocity_counts_per_sec[wheel]));
        if (magnitude > current_group_speed) {
            current_group_speed = magnitude;
        }
    }

    int32_t group_speed = 0;
    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        int64_t error = (int64_t)requested_counts[wheel] -
                        (int64_t)current_counts[wheel];
        int32_t desired_velocity = 0;
        if (maximum_error > 0 && desired_group_speed > 0) {
            int64_t scaled =
                (error * (int64_t)desired_group_speed) /
                (int64_t)maximum_error;
            if (scaled == 0 && error != 0) {
                scaled = error > 0 ? 1 : -1;
            }
            desired_velocity = (int32_t)scaled;
        }

        int32_t current_velocity =
            clamp_steering_velocity(current_velocity_counts_per_sec[wheel]);
        int32_t current_magnitude = abs_i32_saturating(current_velocity);
        int32_t desired_magnitude = abs_i32_saturating(desired_velocity);
        bool reversing = signs_are_opposite(current_velocity, desired_velocity);
        int32_t group_accel = band_accel;
        int32_t accel_scale_speed = desired_magnitude;
        int32_t accel_scale_group_speed = desired_group_speed;
        if (reversing) {
            group_accel =
                ECU_CANOPEN_STEER_TARGET_REVERSAL_DECEL_COUNTS_PER_SEC2;
            accel_scale_speed = current_magnitude;
            accel_scale_group_speed = current_group_speed;
        } else if (desired_magnitude < current_magnitude) {
            group_accel = ECU_CANOPEN_STEER_TARGET_DECEL_COUNTS_PER_SEC2;
            accel_scale_speed = current_magnitude;
            accel_scale_group_speed = current_group_speed;
        }
        int32_t applied_accel = scale_axis_acceleration(
            group_accel, accel_scale_speed, accel_scale_group_speed);

        int32_t velocity_step =
            (int32_t)(((int64_t)applied_accel * (int64_t)dt_ms) / 1000LL);
        int32_t next_velocity = approach_signed(current_velocity,
                                                desired_velocity,
                                                velocity_step);
        if (desired_velocity == 0 &&
            abs_i32_saturating(next_velocity) <=
                ECU_CANOPEN_STEER_TARGET_VELOCITY_SETTLE_COUNTS_PER_SEC) {
            next_velocity = 0;
        }

        /* Trapezoidal integration keeps target velocity continuous even when
         * the joystick reverses inside one PDO period.  A target crossing is
         * clamped to the requested position, but the virtual velocity is kept
         * and then decays normally.  If the operator continues in the same
         * direction on the next sample, motion therefore resumes without a
         * stop/start impulse.
         */
        int64_t position_step =
            ((int64_t)current_velocity + (int64_t)next_velocity) *
            (int64_t)dt_ms / 2000LL;
        int64_t next_position = (int64_t)current_counts[wheel] + position_step;
        int64_t absolute_error = error < 0 ? -error : error;
        bool axis_settled =
            absolute_error <= ECU_CANOPEN_STEER_TARGET_HOLD_COUNTS &&
            abs_i32_saturating(next_velocity) <=
                ECU_CANOPEN_STEER_TARGET_VELOCITY_SETTLE_COUNTS_PER_SEC;

        /* Do not snap a moving axis to a newly reversed target merely because
         * that target is inside the hold band.  The signed virtual velocity
         * must first decelerate through zero; otherwise a small joystick
         * reversal still becomes an instantaneous target-direction change.
         *
         * Once both position error and velocity are small, snapping the
         * remaining sub-deadband error is safe and prevents numerical drift.
         * A normal target crossing is clamped so a stationary target does not
         * create a second corrective move after the drive's own profile stop.
         */
        if (axis_settled) {
            next_position = requested_counts[wheel];
            next_velocity = 0;
        } else if (error == 0 ||
            (error > 0 && next_position > requested_counts[wheel]) ||
            (error < 0 && next_position < requested_counts[wheel])) {
            next_position = requested_counts[wheel];
        }
        if (next_position > ECU_CANOPEN_STEER_MAX_POSITION_COUNTS) {
            next_position = ECU_CANOPEN_STEER_MAX_POSITION_COUNTS;
            if (next_velocity > 0) {
                next_velocity = 0;
            }
        } else if (next_position < -ECU_CANOPEN_STEER_MAX_POSITION_COUNTS) {
            next_position = -ECU_CANOPEN_STEER_MAX_POSITION_COUNTS;
            if (next_velocity < 0) {
                next_velocity = 0;
            }
        }

        output_counts[wheel] = (int32_t)next_position;
        output_velocity_counts_per_sec[wheel] = next_velocity;
        int32_t next_magnitude = abs_i32_saturating(next_velocity);
        if (next_magnitude > group_speed) {
            group_speed = next_magnitude;
        }
    }

    *output_group_speed_counts_per_sec = group_speed;
    return true;
}

static bool group_requests_reversal(
    const int32_t current[ECU_WHEEL_COUNT],
    const int32_t requested[ECU_WHEEL_COUNT])
{
    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        if ((current[wheel] > ECU_CANOPEN_DRIVE_COMMAND_ZERO_DEADBAND_UNITS &&
             requested[wheel] < -ECU_CANOPEN_DRIVE_COMMAND_ZERO_DEADBAND_UNITS) ||
            (current[wheel] < -ECU_CANOPEN_DRIVE_COMMAND_ZERO_DEADBAND_UNITS &&
             requested[wheel] > ECU_CANOPEN_DRIVE_COMMAND_ZERO_DEADBAND_UNITS)) {
            return true;
        }
    }
    return false;
}

static int32_t select_drive_accel(int32_t maximum_error, bool reversing)
{
    if (reversing) {
        return ECU_CANOPEN_DRIVE_ACCEL_REVERSAL_UNITS_PER_SEC;
    }
    if (maximum_error <= ECU_CANOPEN_DRIVE_ERROR_SMALL_UNITS) {
        return ECU_CANOPEN_DRIVE_ACCEL_SMALL_UNITS_PER_SEC;
    }
    if (maximum_error <= ECU_CANOPEN_DRIVE_ERROR_MEDIUM_UNITS) {
        return ECU_CANOPEN_DRIVE_ACCEL_MEDIUM_UNITS_PER_SEC;
    }
    return ECU_CANOPEN_DRIVE_ACCEL_LARGE_UNITS_PER_SEC;
}

bool motion_setpoint_shape_drive_group(
    const int32_t current_velocity_units[ECU_WHEEL_COUNT],
    const int32_t requested_velocity_units[ECU_WHEEL_COUNT],
    uint32_t elapsed_ms,
    int32_t output_velocity_units[ECU_WHEEL_COUNT],
    bool *reversal_through_zero)
{
    if (current_velocity_units == 0 || requested_velocity_units == 0 ||
        output_velocity_units == 0 || reversal_through_zero == 0) {
        return false;
    }

    int32_t effective_requested[ECU_WHEEL_COUNT];
    bool reversing = group_requests_reversal(current_velocity_units,
                                             requested_velocity_units);
    *reversal_through_zero = reversing;
    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        if (reversing) {
            effective_requested[wheel] = 0;
        } else {
            effective_requested[wheel] = requested_velocity_units[wheel];
        }
    }

    int32_t maximum_error = max_abs_error(current_velocity_units,
                                          effective_requested);
    if (maximum_error == 0) {
        copy_group(effective_requested, output_velocity_units);
        return true;
    }

    uint32_t dt_ms = bounded_elapsed_ms(elapsed_ms,
                                        ECU_CANOPEN_DRIVE_PDO_PERIOD_MS * 2U);
    int32_t accel = select_drive_accel(maximum_error, reversing);
    int32_t maximum_step =
        (int32_t)(((int64_t)accel * (int64_t)dt_ms) / 1000LL);
    if (maximum_step < ECU_CANOPEN_DRIVE_RAMP_MIN_STEP_UNITS) {
        maximum_step = ECU_CANOPEN_DRIVE_RAMP_MIN_STEP_UNITS;
    }
    advance_group_by_common_step(current_velocity_units,
                                 effective_requested,
                                 maximum_error,
                                 maximum_step,
                                 output_velocity_units);
    return true;
}

bool motion_setpoint_shape_drive_current_group(
    const int16_t current_10ma[ECU_WHEEL_COUNT],
    const int16_t requested_10ma[ECU_WHEEL_COUNT],
    uint32_t elapsed_ms,
    int16_t output_10ma[ECU_WHEEL_COUNT])
{
    if (current_10ma == 0 || requested_10ma == 0 || output_10ma == 0) {
        return false;
    }

    uint32_t dt_ms = bounded_elapsed_ms(elapsed_ms,
                                        ECU_CANOPEN_DRIVE_PDO_PERIOD_MS * 2U);
    int32_t maximum_step =
        (int32_t)(((int64_t)ECU_TRACK_ASSIST_CURRENT_SLEW_10MA_PER_SEC *
                   (int64_t)dt_ms) / 1000LL);
    if (maximum_step < 1) {
        maximum_step = 1;
    }

    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        int32_t current = current_10ma[wheel];
        int32_t requested = requested_10ma[wheel];
        int32_t current_abs = current < 0 ? -current : current;
        int32_t requested_abs = requested < 0 ? -requested : requested;
        bool same_nonzero_sign =
            (current > 0 && requested > 0) ||
            (current < 0 && requested < 0);

        /* Torque reduction, zero and reversal are safety-relevant and must not
         * wait for the comfort ramp. */
        if (!same_nonzero_sign || requested_abs <= current_abs) {
            output_10ma[wheel] = (int16_t)requested;
            continue;
        }

        int32_t delta = requested - current;
        if (delta > maximum_step) {
            delta = maximum_step;
        } else if (delta < -maximum_step) {
            delta = -maximum_step;
        }
        output_10ma[wheel] = (int16_t)(current + delta);
    }
    return true;
}
