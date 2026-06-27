#include "adjust_control.h"

static float clamp_range(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

void adjust_control_integrate_height(float current_target_mm,
                                     float rate_cmd_mm_s,
                                     uint32_t dt_ms,
                                     const height_adjust_config_t *config,
                                     adjust_control_command_t *out)
{
    if (config == 0 || out == 0) {
        return;
    }
    float rate = clamp_range(rate_cmd_mm_s,
                             -config->max_height_rate_mm_s,
                             config->max_height_rate_mm_s);
    float next = current_target_mm + rate * ((float)dt_ms / 1000.0f);
    out->target_height_mm = clamp_range(next, config->min_height_mm, config->max_height_mm);
    out->body_height_rate_cmd_mm_s = rate;
}

void adjust_control_build_track_command(float global_track_rate_mm_s,
                                        const track_adjust_config_t *config,
                                        adjust_control_command_t *out)
{
    if (config == 0 || out == 0) {
        return;
    }
    out->track_rate_cmd_mm_s = global_track_rate_mm_s;
    out->track = *config;
}
