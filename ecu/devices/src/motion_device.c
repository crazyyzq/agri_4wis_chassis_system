#include <string.h>

#include "canopen_frame.h"
#include "motion_device.h"

static void pack_i16_le(int16_t value, uint8_t *out)
{
    out[0] = (uint8_t)value;
    out[1] = (uint8_t)((uint16_t)value >> 8U);
}

static int16_t scaled_float_to_i16(float value, float scale)
{
    float scaled = value * scale;
    if (scaled > 32767.0f) {
        return 32767;
    }
    if (scaled < -32768.0f) {
        return -32768;
    }
    return (int16_t)scaled;
}

static bool send_drive_command(can_bus_service_t *can2,
                               const ecu_canopen_node_config_t *node,
                               float speed_kph,
                               bool brake_release)
{
    uint8_t payload[4] = { 0U };
    pack_i16_le(scaled_float_to_i16(speed_kph, 100.0f), &payload[0]);
    payload[2] = brake_release ? 1U : 0U;
    canopen_frame_t frame;
    return canopen_frame_build_pdo(node->rpdo1_cob_id, payload, sizeof(payload), &frame) &&
           can_bus_service_send_canopen(can2, &frame);
}

static bool send_steer_command(can_bus_service_t *can2,
                               const ecu_canopen_node_config_t *node,
                               float steer_deg)
{
    uint8_t payload[4] = { 0U };
    pack_i16_le(scaled_float_to_i16(steer_deg, 100.0f), &payload[0]);
    canopen_frame_t frame;
    return canopen_frame_build_pdo(node->rpdo1_cob_id, payload, sizeof(payload), &frame) &&
           can_bus_service_send_canopen(can2, &frame);
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
                                command->brake_release) && ok;
        ok = send_steer_command(can2,
                                &config->steer_nodes[wheel],
                                command->target_steer_deg[wheel]) && ok;
    }
    state->last_motion_command = *command;
    state->apply_count++;
    state->last_result = ok ? ECU_DEVICE_APPLY_OK : ECU_DEVICE_APPLY_REJECTED;
    return state->last_result;
}
