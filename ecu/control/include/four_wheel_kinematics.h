#ifndef FOUR_WHEEL_KINEMATICS_H
#define FOUR_WHEEL_KINEMATICS_H

#include "ecu_types.h"

typedef struct {
    /* Longitudinal distance between the front and rear axle centers.
     * Unit: millimeters in the fixed vehicle frame where the front is +X.
     */
    float wheelbase_mm;

    /* Lateral distance between left and right wheel centers.
     * Unit: millimeters.  This value can change at runtime when the hydraulic
     * track-width mechanism moves the legs.
     */
    float track_width_mm;

    /* Smallest allowed absolute ICR turning radius.  The implementation clamps
     * requested steering to this radius to avoid invalid geometry and excessive
     * steering targets.
     */
    float min_turn_radius_mm;
} four_wheel_kinematics_geometry_t;

typedef struct {
    /* Per-wheel steering targets in vehicle leg order:
     * leg1/front-right, leg2/front-left, leg3/rear-left, leg4/rear-right.
     * Positive angle is counter-clockwise in the fixed vehicle frame.
     */
    float target_steer_deg[ECU_WHEEL_COUNT];

    /* Per-wheel signed speed targets in the fixed vehicle frame.
     * Positive speed moves the wheel along vehicle +X; reverse Ackermann may
     * command negative speed while the operator is in D gear because the rear
     * of the machine is treated as the driving-forward direction.
     */
    float target_wheel_speed_kph[ECU_WHEEL_COUNT];
} four_wheel_kinematics_output_t;

/* Build normal four-wheel Ackermann targets.
 *
 * Operator view: the vehicle front is the driving-forward direction.  A
 * positive steering input bends the trajectory toward the vehicle-left side.
 */
void four_wheel_kinematics_build_ackermann(float speed_kph,
                                           float steer_input_deg,
                                           const four_wheel_kinematics_geometry_t *geometry,
                                           four_wheel_kinematics_output_t *out);

/* Build reverse-Ackermann targets.
 *
 * Operator view: the vehicle rear is the driving-forward direction.  The caller
 * must already have converted D/R into fixed-frame speed sign.  The steering
 * input is still operator-relative, so a positive input bends left from the
 * rear-facing driving frame, which is rightward in the fixed front-facing frame.
 */
void four_wheel_kinematics_build_reverse_ackermann(float speed_kph,
                                                   float steer_input_deg,
                                                   const four_wheel_kinematics_geometry_t *geometry,
                                                   four_wheel_kinematics_output_t *out);

/* Build fixed-wheel-angle spin targets.
 *
 * This is a deliberate commissioning-friendly spin model: steering angles and
 * wheel speed signs are explicit in the confirmed vehicle leg order.
 */
void four_wheel_kinematics_build_spin(float speed_kph,
                                      float spin_angle_deg,
                                      four_wheel_kinematics_output_t *out);

/* Build crab targets with the same steering angle and speed on every wheel. */
void four_wheel_kinematics_build_crab(float speed_kph,
                                      float steer_deg,
                                      four_wheel_kinematics_output_t *out);

#endif /* FOUR_WHEEL_KINEMATICS_H */
