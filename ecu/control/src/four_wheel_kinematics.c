#include <math.h>
#include <string.h>

#include "four_wheel_kinematics.h"
#include "ecu_config.h"

#define FOUR_WHEEL_KINEMATICS_DEG_TO_RAD (0.01745329251994329577f)
#define FOUR_WHEEL_KINEMATICS_RAD_TO_DEG (57.295779513082320876f)
#define FOUR_WHEEL_KINEMATICS_STEER_ZERO_EPS_DEG (0.01f)

typedef struct {
    /* Wheel center in the fixed vehicle frame:
     * +X points to the original vehicle front and +Y points to the original
     * vehicle left.  The ECU array order remains leg1 FR, leg2 FL, leg3 RL,
     * leg4 RR regardless of the selected driving mode.
     */
    float wheel_x_mm;
    float wheel_y_mm;
} wheel_position_t;

static float clampf(float value, float lower, float upper)
{
    if (value < lower) {
        return lower;
    }
    if (value > upper) {
        return upper;
    }
    return value;
}

static four_wheel_kinematics_geometry_t clamp_geometry(
    const four_wheel_kinematics_geometry_t *geometry)
{
    four_wheel_kinematics_geometry_t out = {
        .wheelbase_mm = ECU_VEHICLE_WHEELBASE_MM,
        .track_width_mm = ECU_VEHICLE_TRACK_WIDTH_DEFAULT_MM,
        .min_turn_radius_mm = ECU_VEHICLE_MIN_TURN_RADIUS_MM,
    };

    if (geometry != 0) {
        out = *geometry;
    }

    /* Geometry can eventually come from hydraulic/ADC feedback.  Clamp it here
     * so a missing sensor, uninitialized field or impossible calibration cannot
     * create a divide-by-zero or an extreme steering command.
     */
    out.wheelbase_mm = clampf(out.wheelbase_mm,
                              ECU_VEHICLE_WHEELBASE_MIN_MM,
                              ECU_VEHICLE_WHEELBASE_MAX_MM);
    out.track_width_mm = clampf(out.track_width_mm,
                                ECU_VEHICLE_TRACK_WIDTH_MIN_MM,
                                ECU_VEHICLE_TRACK_WIDTH_MAX_MM);
    if (out.min_turn_radius_mm < out.wheelbase_mm) {
        out.min_turn_radius_mm = out.wheelbase_mm;
    }
    return out;
}

static void clear_output(four_wheel_kinematics_output_t *out)
{
    if (out != 0) {
        memset(out, 0, sizeof(*out));
    }
}

static void set_all_wheel_speeds(four_wheel_kinematics_output_t *out,
                                 float speed_kph)
{
    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        out->target_wheel_speed_kph[wheel] = speed_kph;
    }
}

static wheel_position_t wheel_position(uint32_t wheel,
                                       const four_wheel_kinematics_geometry_t *geometry)
{
    float half_wheelbase_mm = geometry->wheelbase_mm * 0.5f;
    float half_track_width_mm = geometry->track_width_mm * 0.5f;
    wheel_position_t position = {
        .wheel_x_mm = half_wheelbase_mm,
        .wheel_y_mm = -half_track_width_mm,
    };

    switch (wheel) {
    case ECU_WHEEL_LEG1_FRONT_RIGHT:
        position.wheel_x_mm = half_wheelbase_mm;
        position.wheel_y_mm = -half_track_width_mm;
        break;
    case ECU_WHEEL_LEG2_FRONT_LEFT:
        position.wheel_x_mm = half_wheelbase_mm;
        position.wheel_y_mm = half_track_width_mm;
        break;
    case ECU_WHEEL_LEG3_REAR_LEFT:
        position.wheel_x_mm = -half_wheelbase_mm;
        position.wheel_y_mm = half_track_width_mm;
        break;
    case ECU_WHEEL_LEG4_REAR_RIGHT:
    default:
        position.wheel_x_mm = -half_wheelbase_mm;
        position.wheel_y_mm = -half_track_width_mm;
        break;
    }
    return position;
}

static float signed_speed_magnitude(float linear_velocity_x,
                                    float linear_velocity_y,
                                    float center_speed_kph)
{
    float magnitude = sqrtf((linear_velocity_x * linear_velocity_x) +
                            (linear_velocity_y * linear_velocity_y));
    return center_speed_kph < 0.0f ? -magnitude : magnitude;
}

static void build_ackermann_from_curvature(float speed_kph,
                                           float steer_input_deg,
                                           float curvature_sign,
                                           const four_wheel_kinematics_geometry_t *geometry,
                                           four_wheel_kinematics_output_t *out)
{
    if (out == 0) {
        return;
    }
    clear_output(out);
    four_wheel_kinematics_geometry_t safe_geometry = clamp_geometry(geometry);
    /* curvature_sign converts the operator steering request into the fixed
     * front-facing vehicle frame.  Positive Ackermann keeps the sign.  Reverse
     * Ackermann flips it because the operator is viewing the machine from the
     * rear as the driving-forward direction.
     */
    float signed_steer_deg = steer_input_deg * curvature_sign;

    if (fabsf(signed_steer_deg) < FOUR_WHEEL_KINEMATICS_STEER_ZERO_EPS_DEG) {
        set_all_wheel_speeds(out, speed_kph);
        return;
    }

    /* The input steering angle is interpreted as the virtual center steering
     * angle.  It defines the ICR radius at the vehicle center; each wheel then
     * points tangent to its own circular path around the same ICR.
     */
    float steer_rad = fabsf(signed_steer_deg) * FOUR_WHEEL_KINEMATICS_DEG_TO_RAD;
    float turn_radius_mm = safe_geometry.wheelbase_mm / tanf(steer_rad);
    turn_radius_mm = clampf(turn_radius_mm,
                            safe_geometry.min_turn_radius_mm,
                            10000000.0f);
    if (signed_steer_deg < 0.0f) {
        turn_radius_mm = -turn_radius_mm;
    }

    /* steering_angular_rate builds wheel angles independent of actual speed;
     * speed_angular_rate builds signed wheel speeds.  Keeping them separate
     * makes zero-speed steering targets valid while preserving D/R direction.
     */
    float steering_angular_rate = 1.0f / turn_radius_mm;
    float speed_angular_rate = speed_kph / turn_radius_mm;

    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        wheel_position_t position = wheel_position(wheel, &safe_geometry);

        /* Unit tangent vector for steering angle.  The vector is calculated
         * from geometry only, so the wheel still receives a meaningful steering
         * target when commanded speed is zero during a mode transition.
         */
        float axis_velocity_x = 1.0f - (steering_angular_rate * position.wheel_y_mm);
        float axis_velocity_y = steering_angular_rate * position.wheel_x_mm;

        /* Actual signed velocity vector.  Inner wheels get smaller magnitude,
         * outer wheels get larger magnitude, and the sign follows the fixed
         * vehicle-frame speed chosen by the command arbiter.
         */
        float linear_velocity_x = speed_kph - (speed_angular_rate * position.wheel_y_mm);
        float linear_velocity_y = speed_angular_rate * position.wheel_x_mm;

        out->target_steer_deg[wheel] =
            atan2f(axis_velocity_y, axis_velocity_x) *
            FOUR_WHEEL_KINEMATICS_RAD_TO_DEG;
        out->target_wheel_speed_kph[wheel] =
            signed_speed_magnitude(linear_velocity_x,
                                   linear_velocity_y,
                                   speed_kph);
    }
}

void four_wheel_kinematics_build_ackermann(float speed_kph,
                                           float steer_input_deg,
                                           const four_wheel_kinematics_geometry_t *geometry,
                                           four_wheel_kinematics_output_t *out)
{
    build_ackermann_from_curvature(speed_kph, steer_input_deg, 1.0f, geometry, out);
}

void four_wheel_kinematics_build_reverse_ackermann(float speed_kph,
                                                   float steer_input_deg,
                                                   const four_wheel_kinematics_geometry_t *geometry,
                                                   four_wheel_kinematics_output_t *out)
{
    /* Reverse Ackermann is not just "drive backward".  The rear becomes the
     * operator's forward direction, so the command arbiter flips D/R speed
     * before calling this function, and this function flips steering curvature
     * into the fixed front-facing vehicle frame.
     */
    build_ackermann_from_curvature(speed_kph, steer_input_deg, -1.0f, geometry, out);
}

void four_wheel_kinematics_build_spin(float speed_kph,
                                      float spin_angle,
                                      four_wheel_kinematics_output_t *out)
{
    if (out == 0) {
        return;
    }
    clear_output(out);
    out->target_steer_deg[ECU_WHEEL_LEG1_FRONT_RIGHT] = spin_angle;
    out->target_steer_deg[ECU_WHEEL_LEG2_FRONT_LEFT] = -spin_angle;
    out->target_steer_deg[ECU_WHEEL_LEG3_REAR_LEFT] = spin_angle;
    out->target_steer_deg[ECU_WHEEL_LEG4_REAR_RIGHT] = -spin_angle;

    out->target_wheel_speed_kph[ECU_WHEEL_LEG1_FRONT_RIGHT] = speed_kph;
    out->target_wheel_speed_kph[ECU_WHEEL_LEG2_FRONT_LEFT] = -speed_kph;
    out->target_wheel_speed_kph[ECU_WHEEL_LEG3_REAR_LEFT] = -speed_kph;
    out->target_wheel_speed_kph[ECU_WHEEL_LEG4_REAR_RIGHT] = speed_kph;
}

void four_wheel_kinematics_build_crab(float speed_kph,
                                      float steer_deg,
                                      four_wheel_kinematics_output_t *out)
{
    if (out == 0) {
        return;
    }
    clear_output(out);
    set_all_wheel_speeds(out, speed_kph);
    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        out->target_steer_deg[wheel] = steer_deg;
    }
}
