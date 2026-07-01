#include "motion_control.h"

#include "ecu_config.h"
#include "four_wheel_kinematics.h"

static float clampf(float value, float limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

static four_wheel_kinematics_geometry_t geometry_from_limits(
    const motion_control_limits_t *limits)
{
    four_wheel_kinematics_geometry_t geometry = {
        .wheelbase_mm = ECU_VEHICLE_WHEELBASE_MM,
        .track_width_mm = ECU_VEHICLE_TRACK_WIDTH_DEFAULT_MM,
        .min_turn_radius_mm = ECU_VEHICLE_MIN_TURN_RADIUS_MM,
    };

    if (limits != 0) {
        geometry.wheelbase_mm = limits->wheelbase_mm;
        geometry.track_width_mm = limits->track_width_mm;
    }
    return geometry;
}

static void apply_kinematics_output(const four_wheel_kinematics_output_t *input,
                                    vehicle_actuator_command_t *out)
{
    if (input == 0 || out == 0) {
        return;
    }
    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        out->target_steer_deg[wheel] = input->target_steer_deg[wheel];
        out->target_wheel_speed_kph[wheel] = input->target_wheel_speed_kph[wheel];
    }
}

static void build_positive_ackermann_targets(float speed_kph,
                                             float steer_deg,
                                             const motion_control_limits_t *limits,
                                             vehicle_actuator_command_t *out)
{
    four_wheel_kinematics_output_t kinematics;
    four_wheel_kinematics_geometry_t geometry = geometry_from_limits(limits);
    four_wheel_kinematics_build_ackermann(speed_kph, steer_deg, &geometry, &kinematics);
    apply_kinematics_output(&kinematics, out);
}

static void build_reverse_ackermann_targets(float speed_kph,
                                            float steer_deg,
                                            const motion_control_limits_t *limits,
                                            vehicle_actuator_command_t *out)
{
    /* At this layer speed_kph is already expressed in the fixed vehicle frame.
     * For reverse Ackermann, D gear has been converted to negative vehicle-X
     * speed by command_arbiter because the rear is the driving-forward end.
     */
    four_wheel_kinematics_output_t kinematics;
    four_wheel_kinematics_geometry_t geometry = geometry_from_limits(limits);
    four_wheel_kinematics_build_reverse_ackermann(speed_kph, steer_deg, &geometry, &kinematics);
    apply_kinematics_output(&kinematics, out);
}

static void build_spin_targets(float speed_kph,
                               vehicle_actuator_command_t *out)
{
    four_wheel_kinematics_output_t kinematics;
    four_wheel_kinematics_build_spin(speed_kph,
                                     ECU_MOTION_SPIN_STEER_DEG,
                                     &kinematics);
    apply_kinematics_output(&kinematics, out);
}

static void build_crab_targets(float speed_kph,
                               float steer_deg,
                               vehicle_actuator_command_t *out)
{
    four_wheel_kinematics_output_t kinematics;
    four_wheel_kinematics_build_crab(speed_kph, steer_deg, &kinematics);
    apply_kinematics_output(&kinematics, out);
}

void motion_control_build_candidate(ecu_motion_mode_t mode,
                                    float speed_kph,
                                    float steer_input_deg,
                                    const motion_control_limits_t *limits,
                                    vehicle_actuator_command_t *out)
{
    if (limits == 0 || out == 0) {
        return;
    }
    out->motion_mode = mode;
    float limited_speed_kph = clampf(speed_kph, limits->max_speed_kph);
    float steer_deg = clampf(steer_input_deg, limits->max_steer_deg);

    /* target_speed_kph is stored in the fixed vehicle frame.  It may be
     * negative while active_gear is D in reverse Ackermann, because that mode
     * defines the rear of the vehicle as the operator's forward direction.
     */
    out->target_speed_kph = limited_speed_kph;

    switch (mode) {
    case ECU_MOTION_MODE_REVERSE_ACKERMANN:
        build_reverse_ackermann_targets(limited_speed_kph, steer_deg, limits, out);
        break;
    case ECU_MOTION_MODE_SPIN:
        build_spin_targets(limited_speed_kph, out);
        break;
    case ECU_MOTION_MODE_CRAB:
        build_crab_targets(limited_speed_kph, steer_deg, out);
        break;
    case ECU_MOTION_MODE_POSITIVE_ACKERMANN:
    default:
        build_positive_ackermann_targets(limited_speed_kph, steer_deg, limits, out);
        break;
    }
}
