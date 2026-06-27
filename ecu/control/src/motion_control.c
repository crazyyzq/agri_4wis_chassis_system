#include "motion_control.h"

static float clampf(float value, float limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
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
    out->target_speed_kph = clampf(speed_kph, limits->max_speed_kph);
    float steer_deg = mode == ECU_MOTION_MODE_SPIN ? 0.0f : clampf(steer_input_deg, limits->max_steer_deg);
    out->target_steer_deg[0] = steer_deg;
    out->target_steer_deg[1] = steer_deg;
    out->target_steer_deg[2] = steer_deg;
    out->target_steer_deg[3] = steer_deg;
}
