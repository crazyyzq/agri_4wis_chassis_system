#include <string.h>
#include <stdint.h>

#include "lift_hydraulic_device.h"
#include "servo_drive_canopen.h"

static int32_t scaled_float_to_i32(float value, float scale)
{
    float scaled = value * scale;
    if (scaled > 2147483647.0f) {
        return 2147483647;
    }
    if (scaled < -2147483648.0f) {
        return INT32_MIN;
    }
    return (int32_t)scaled;
}

static bool send_lift_command(can_bus_service_t *can3,
                              const ecu_canopen_node_config_t *node,
                              float height_mm,
                              float height_rate_mm_s,
                              float lift_scale)
{
    uint16_t control_word = (height_rate_mm_s == 0.0f) ?
                            SERVO_DRIVE_CONTROL_QUICK_STOP :
                            SERVO_DRIVE_CONTROL_ENABLE_OPERATION;
    int32_t height_counts = scaled_float_to_i32(height_mm, lift_scale);

    return servo_drive_canopen_select_mode(can3,
                                           node,
                                           control_word,
                                           SERVO_DRIVE_MODE_PROFILE_POSITION) &&
           servo_drive_canopen_set_target_position(can3,
                                                   node,
                                                   control_word,
                                                   height_counts);
}

static uint32_t valve_mask_from_track_rate(const ecu_hardware_config_t *config,
                                           float track_rate_mm_s)
{
    if (track_rate_mm_s > 0.0f) {
        return config->hydraulic_track_extend_mask;
    }
    if (track_rate_mm_s < 0.0f) {
        return config->hydraulic_track_retract_mask;
    }
    return 0U;
}

void lift_hydraulic_device_init(lift_hydraulic_device_state_t *state)
{
    if (state != 0) {
        memset(state, 0, sizeof(*state));
        state->last_result = ECU_DEVICE_APPLY_OK;
    }
}

ecu_device_apply_result_t lift_hydraulic_device_apply(lift_hydraulic_device_state_t *state,
                                                      can_bus_service_t *can3,
                                                      dio_service_t *dio,
                                                      const ecu_hardware_config_t *config,
                                                      const vehicle_actuator_command_t *command)
{
    if (state == 0 || can3 == 0 || dio == 0 || config == 0 || command == 0) {
        return ECU_DEVICE_APPLY_INVALID_ARGUMENT;
    }
    if (!can3->online) {
        state->last_result = ECU_DEVICE_APPLY_BACKEND_OFFLINE;
        return state->last_result;
    }

    bool ok = true;
    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        ok = send_lift_command(can3,
                               &config->lift_nodes[wheel],
                               command->target_height_mm,
                               command->height_rate_mm_s,
                               config->lift_mm_to_counts) && ok;
    }

    uint32_t valve_mask = command->hydraulic_valve_mask |
                          valve_mask_from_track_rate(config, command->track_rate_mm_s);
    valve_mask &= config->hydraulic_managed_valve_mask;
    dio_service_write_masked(dio, config->dio_hydraulic_enable_mask, command->hydraulic_enable);
    dio_service_write_masked(dio, config->hydraulic_managed_valve_mask, false);
    dio_service_write_masked(dio, valve_mask, command->hydraulic_enable && valve_mask != 0U);
    state->last_valve_mask = valve_mask;
    state->apply_count++;
    state->last_result = ok ? ECU_DEVICE_APPLY_OK : ECU_DEVICE_APPLY_REJECTED;
    return state->last_result;
}
