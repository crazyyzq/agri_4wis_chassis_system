#include "motion_control.h"

#include "ecu_config.h"

static float clampf(float value, float limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

static void set_all_wheel_speeds(vehicle_actuator_command_t *out,
                                 float speed_kph)
{
    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        out->target_wheel_speed_kph[wheel] = speed_kph;
    }
}

static void build_positive_ackermann_targets(float speed_kph,
                                             float steer_deg,
                                             vehicle_actuator_command_t *out)
{
    set_all_wheel_speeds(out, speed_kph);
    out->target_steer_deg[ECU_WHEEL_LEG1_FRONT_RIGHT] =
        steer_deg * ECU_MOTION_ACKERMANN_FRONT_SIGN;
    out->target_steer_deg[ECU_WHEEL_LEG2_FRONT_LEFT] =
        steer_deg * ECU_MOTION_ACKERMANN_FRONT_SIGN;
    out->target_steer_deg[ECU_WHEEL_LEG3_REAR_LEFT] =
        steer_deg * ECU_MOTION_ACKERMANN_REAR_SIGN;
    out->target_steer_deg[ECU_WHEEL_LEG4_REAR_RIGHT] =
        steer_deg * ECU_MOTION_ACKERMANN_REAR_SIGN;
}

static void build_reverse_ackermann_targets(float speed_kph,
                                            float steer_deg,
                                            vehicle_actuator_command_t *out)
{
    set_all_wheel_speeds(out, speed_kph);
    out->target_steer_deg[ECU_WHEEL_LEG1_FRONT_RIGHT] =
        steer_deg * ECU_MOTION_REVERSE_ACK_FRONT_SIGN;
    out->target_steer_deg[ECU_WHEEL_LEG2_FRONT_LEFT] =
        steer_deg * ECU_MOTION_REVERSE_ACK_FRONT_SIGN;
    out->target_steer_deg[ECU_WHEEL_LEG3_REAR_LEFT] =
        steer_deg * ECU_MOTION_REVERSE_ACK_REAR_SIGN;
    out->target_steer_deg[ECU_WHEEL_LEG4_REAR_RIGHT] =
        steer_deg * ECU_MOTION_REVERSE_ACK_REAR_SIGN;
}

static void build_spin_targets(float speed_kph,
                               vehicle_actuator_command_t *out)
{
    float spin_angle = ECU_MOTION_SPIN_STEER_DEG;

    out->target_steer_deg[ECU_WHEEL_LEG1_FRONT_RIGHT] = spin_angle;
    out->target_steer_deg[ECU_WHEEL_LEG2_FRONT_LEFT] = -spin_angle;
    out->target_steer_deg[ECU_WHEEL_LEG3_REAR_LEFT] = spin_angle;
    out->target_steer_deg[ECU_WHEEL_LEG4_REAR_RIGHT] = -spin_angle;

    out->target_wheel_speed_kph[ECU_WHEEL_LEG1_FRONT_RIGHT] = speed_kph;
    out->target_wheel_speed_kph[ECU_WHEEL_LEG2_FRONT_LEFT] = -speed_kph;
    out->target_wheel_speed_kph[ECU_WHEEL_LEG3_REAR_LEFT] = -speed_kph;
    out->target_wheel_speed_kph[ECU_WHEEL_LEG4_REAR_RIGHT] = speed_kph;
}

static void build_crab_targets(float speed_kph,
                               float steer_deg,
                               vehicle_actuator_command_t *out)
{
    set_all_wheel_speeds(out, speed_kph);
    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        out->target_steer_deg[wheel] = steer_deg;
    }
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
    out->target_speed_kph = limited_speed_kph;

    switch (mode) {
    case ECU_MOTION_MODE_REVERSE_ACKERMANN:
        build_reverse_ackermann_targets(limited_speed_kph, steer_deg, out);
        break;
    case ECU_MOTION_MODE_SPIN:
        build_spin_targets(limited_speed_kph, out);
        break;
    case ECU_MOTION_MODE_CRAB:
        build_crab_targets(limited_speed_kph, steer_deg, out);
        break;
    case ECU_MOTION_MODE_POSITIVE_ACKERMANN:
    default:
        build_positive_ackermann_targets(limited_speed_kph, steer_deg, out);
        break;
    }
}
