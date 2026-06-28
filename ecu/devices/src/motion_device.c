#include <string.h>
#include <stdint.h>

#include "motion_device.h"
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

static bool send_drive_command(can_bus_service_t *can2,
                               const ecu_canopen_node_config_t *node,
                               float speed_kph,
                               float speed_scale,
                               bool brake_release)
{
    uint16_t control_word = brake_release ?
                            SERVO_DRIVE_CONTROL_ENABLE_OPERATION :
                            SERVO_DRIVE_CONTROL_QUICK_STOP;
    int32_t velocity_counts = scaled_float_to_i32(speed_kph, speed_scale);

    return servo_drive_canopen_select_mode(can2,
                                           node,
                                           control_word,
                                           SERVO_DRIVE_MODE_PROFILE_VELOCITY) &&
           servo_drive_canopen_set_target_velocity(can2,
                                                   node,
                                                   control_word,
                                                   velocity_counts);
}

static bool send_steer_command(can_bus_service_t *can2,
                               const ecu_canopen_node_config_t *node,
                               float steer_deg,
                               float steer_scale)
{
    int32_t position_counts = scaled_float_to_i32(steer_deg, steer_scale);

    return servo_drive_canopen_select_mode(can2,
                                           node,
                                           SERVO_DRIVE_CONTROL_ENABLE_OPERATION,
                                           SERVO_DRIVE_MODE_PROFILE_POSITION) &&
           servo_drive_canopen_set_target_position(can2,
                                                   node,
                                                   SERVO_DRIVE_CONTROL_ENABLE_OPERATION,
                                                   position_counts);
}

void motion_device_init(motion_device_state_t *state)
{
    if (state != 0) {
        memset(state, 0, sizeof(*state));
        state->last_result = ECU_DEVICE_APPLY_OK;
    }
}

ecu_device_apply_result_t motion_device_apply(motion_device_state_t *state,
                                              can_bus_service_t *can2,
                                              const ecu_hardware_config_t *config,
                                              const vehicle_actuator_command_t *command)
{
    if (state == 0 || can2 == 0 || config == 0 || command == 0) {
        return ECU_DEVICE_APPLY_INVALID_ARGUMENT;
    }
    if (!can2->online) {
        state->last_result = ECU_DEVICE_APPLY_BACKEND_OFFLINE;
        return state->last_result;
    }

    bool ok = true;
    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        ok = send_drive_command(can2,
                                &config->drive_nodes[wheel],
                                command->target_speed_kph,
                                config->drive_speed_kph_to_counts_per_sec,
                                command->brake_release) && ok;
        ok = send_steer_command(can2,
                                &config->steer_nodes[wheel],
                                command->target_steer_deg[wheel],
                                config->steer_deg_to_counts) && ok;
    }
    state->last_motion_command = *command;
    state->apply_count++;
    state->last_result = ok ? ECU_DEVICE_APPLY_OK : ECU_DEVICE_APPLY_REJECTED;
    return state->last_result;
}
