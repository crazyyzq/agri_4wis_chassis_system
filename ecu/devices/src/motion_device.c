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

static bool send_drive_command(canopen_master_service_t *canopen,
                               const ecu_canopen_node_config_t *node,
                               float speed_kph,
                               float speed_scale,
                               bool brake_release)
{
    uint16_t control_word = brake_release ?
                            SERVO_DRIVE_CONTROL_ENABLE_OPERATION :
                            SERVO_DRIVE_CONTROL_QUICK_STOP;
    int32_t velocity_counts = scaled_float_to_i32(speed_kph, speed_scale);
    uint16_t brake_active_mask =
        brake_release ?
        ((ECU_SERVO_BRAKE_RELEASE_CANOPEN_ACTIVE_BIT != 0U) ?
         SERVO_DRIVE_OUTPUT_OUT1_MASK : 0U) :
        ((ECU_SERVO_BRAKE_RELEASE_CANOPEN_ACTIVE_BIT != 0U) ?
         0U : SERVO_DRIVE_OUTPUT_OUT1_MASK);

    return servo_drive_canopen_set_output_state(canopen,
                                                node,
                                                SERVO_DRIVE_OUTPUT_OUT1_MASK,
                                                brake_active_mask) &&
           servo_drive_canopen_select_mode(canopen,
                                           node,
                                           control_word,
                                           SERVO_DRIVE_MODE_PROFILE_VELOCITY) &&
           servo_drive_canopen_set_target_velocity(canopen,
                                                   node,
                                                   control_word,
                                                   velocity_counts);
}

static bool send_steer_command(canopen_master_service_t *canopen,
                               const ecu_canopen_node_config_t *node,
                               float steer_deg,
                               float steer_scale)
{
    uint16_t input_states = 0U;
    if (servo_drive_canopen_read_input_states(canopen, node, &input_states)) {
        bool positive_limit = (input_states & SERVO_DRIVE_INPUT_IN2_MASK) != 0U;
        bool negative_limit = (input_states & SERVO_DRIVE_INPUT_IN3_MASK) != 0U;
        if ((steer_deg > 0.0f && positive_limit) ||
            (steer_deg < 0.0f && negative_limit)) {
            return servo_drive_canopen_send_control_word(canopen,
                                                         node,
                                                         SERVO_DRIVE_CONTROL_QUICK_STOP);
        }
    }

    int32_t position_counts = scaled_float_to_i32(steer_deg, steer_scale);

    return servo_drive_canopen_select_mode(canopen,
                                           node,
                                           SERVO_DRIVE_CONTROL_ENABLE_OPERATION,
                                           SERVO_DRIVE_MODE_PROFILE_POSITION) &&
           servo_drive_canopen_set_target_position(canopen,
                                                   node,
                                                   SERVO_DRIVE_CONTROL_ENABLE_OPERATION,
                                                   position_counts);
}

static bool command_source_allows_motion_output(ecu_command_source_t source)
{
    return source == COMMAND_SOURCE_REMOTE ||
           source == COMMAND_SOURCE_AUTO ||
           source == COMMAND_SOURCE_MAINTENANCE ||
           source == COMMAND_SOURCE_CPU1 ||
           source == COMMAND_SOURCE_SAFETY;
}

static bool command_changed(const motion_device_state_t *state,
                            const vehicle_actuator_command_t *command)
{
    return !state->last_motion_command_valid ||
           memcmp(&state->last_motion_command,
                  command,
                  sizeof(state->last_motion_command)) != 0;
}

void motion_device_init(motion_device_state_t *state)
{
    if (state != 0) {
        memset(state, 0, sizeof(*state));
        state->last_result = ECU_DEVICE_APPLY_OK;
    }
}

ecu_device_apply_result_t motion_device_apply(motion_device_state_t *state,
                                              canopen_master_service_t *canopen,
                                              const ecu_hardware_config_t *config,
                                              const vehicle_actuator_command_t *command)
{
    if (state == 0 || canopen == 0 || config == 0 || command == 0) {
        return ECU_DEVICE_APPLY_INVALID_ARGUMENT;
    }
    if (!canopen->snapshot.initialized || !canopen->snapshot.can_normal) {
        state->last_result = ECU_DEVICE_APPLY_BACKEND_OFFLINE;
        return state->last_result;
    }
    if (!command_source_allows_motion_output(command->source) ||
        !command_changed(state, command)) {
        state->skipped_count++;
        state->last_result = ECU_DEVICE_APPLY_OK;
        return state->last_result;
    }

    bool ok = true;
    for (uint32_t wheel = 0U; wheel < ECU_WHEEL_COUNT; ++wheel) {
        ok = send_drive_command(canopen,
                                &config->drive_nodes[wheel],
                                command->target_speed_kph,
                                config->drive_speed_kph_to_counts_per_sec,
                                command->brake_release) && ok;
        ok = send_steer_command(canopen,
                                &config->steer_nodes[wheel],
                                command->target_steer_deg[wheel],
                                config->steer_deg_to_counts) && ok;
    }
    state->last_motion_command = *command;
    state->last_motion_command_valid = true;
    state->apply_count++;
    state->last_result = ok ? ECU_DEVICE_APPLY_OK : ECU_DEVICE_APPLY_REJECTED;
    return state->last_result;
}
