#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include "vehicle_types.h"

typedef struct {
    float max_speed_kph;
    float max_steer_deg;
} motion_control_limits_t;

void motion_control_build_candidate(ecu_motion_mode_t mode,
                                    float speed_kph,
                                    float steer_input_deg,
                                    const motion_control_limits_t *limits,
                                    vehicle_actuator_command_t *out);

#endif /* MOTION_CONTROL_H */
