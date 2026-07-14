/* Coherent CAN2 motion setpoint shaping.
 *
 * These helpers operate on complete four-axis vectors. They own no hardware,
 * RTOS object, CAN queue, or safety policy. The CAN2 motion task supplies the
 * latest coherent request and remains the sole owner of PDO transmission.
 */
#ifndef MOTION_SETPOINT_SHAPER_H
#define MOTION_SETPOINT_SHAPER_H

#include <stdbool.h>
#include <stdint.h>

#include "ecu_types.h"

typedef enum {
    MOTION_STEER_FOLLOW_BAND_HOLD = 0,
    MOTION_STEER_FOLLOW_BAND_FINE,
    MOTION_STEER_FOLLOW_BAND_SMALL,
    MOTION_STEER_FOLLOW_BAND_MEDIUM,
    MOTION_STEER_FOLLOW_BAND_LARGE,
} motion_steer_follow_band_t;

/* Advance one coherent four-steering-axis position vector.
 *
 * All position and velocity values use Node5..8 encoder position counts:
 *   position: count
 *   group_speed: count/s of the axis with the largest remaining error
 *
 * Every output axis advances by the same fraction of its own current error.
 * `elapsed_ms` is bounded internally so a delayed scheduler tick cannot create
 * one large target jump.
 */
bool motion_setpoint_shape_steering_group(
    const int32_t current_counts[ECU_WHEEL_COUNT],
    const int32_t requested_counts[ECU_WHEEL_COUNT],
    int32_t current_group_speed_counts_per_sec,
    uint32_t elapsed_ms,
    int32_t output_counts[ECU_WHEEL_COUNT],
    int32_t *output_group_speed_counts_per_sec,
    motion_steer_follow_band_t *output_band);

/* Advance one coherent four-drive-axis velocity vector.
 *
 * Values use the saved target-velocity contract (0.1 encoder-count/s). If any active
 * axis requests the opposite sign, `reversal_through_zero` is set and the
 * complete vector is first ramped to zero. The opposite command starts only
 * after that coherent zero vector has been reached.
 */
bool motion_setpoint_shape_drive_group(
    const int32_t current_velocity_units[ECU_WHEEL_COUNT],
    const int32_t requested_velocity_units[ECU_WHEEL_COUNT],
    uint32_t elapsed_ms,
    int32_t output_velocity_units[ECU_WHEEL_COUNT],
    bool *reversal_through_zero);

/* Slew one coherent four-drive-axis current vector.
 *
 * Units are 10 mA.  Increasing torque is rate-limited; reducing magnitude,
 * crossing zero, or removing assist is immediate so a safety/overspeed gate
 * cannot leave stale torque active.
 */
bool motion_setpoint_shape_drive_current_group(
    const int16_t current_10ma[ECU_WHEEL_COUNT],
    const int16_t requested_10ma[ECU_WHEEL_COUNT],
    uint32_t elapsed_ms,
    int16_t output_10ma[ECU_WHEEL_COUNT]);

#endif /* MOTION_SETPOINT_SHAPER_H */
