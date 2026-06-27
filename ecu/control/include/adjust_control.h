#ifndef ADJUST_CONTROL_H
#define ADJUST_CONTROL_H

#include <stdint.h>

#include "ecu_config.h"

typedef struct {
    float min_height_mm;
    float max_height_mm;
    float max_height_rate_mm_s;
} height_adjust_config_t;

typedef struct {
    float target_height_mm;
    float body_height_rate_cmd_mm_s;
    float track_rate_cmd_mm_s;
    track_adjust_config_t track;
} adjust_control_command_t;

void adjust_control_integrate_height(float current_target_mm,
                                     float rate_cmd_mm_s,
                                     uint32_t dt_ms,
                                     const height_adjust_config_t *config,
                                     adjust_control_command_t *out);
void adjust_control_build_track_command(float global_track_rate_mm_s,
                                        const track_adjust_config_t *config,
                                        adjust_control_command_t *out);

#endif /* ADJUST_CONTROL_H */
