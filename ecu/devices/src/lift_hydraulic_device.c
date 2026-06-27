#include <string.h>

#include "canopen_frame.h"
#include "lift_hydraulic_device.h"

static bool send_lift_command(can_bus_service_t *can3,
                              const ecu_canopen_node_config_t *node,
                              float height_mm,
                              float height_rate_mm_s)
{
    int16_t height_cmd = (int16_t)height_mm;
    int16_t rate_cmd = (int16_t)height_rate_mm_s;
    uint8_t payload[4] = {
        (uint8_t)height_cmd,
        (uint8_t)((uint16_t)height_cmd >> 8U),
        (uint8_t)rate_cmd,
        (uint8_t)((uint16_t)rate_cmd >> 8U)
    };
    canopen_frame_t frame;
    return canopen_frame_build_pdo(node->rpdo1_cob_id, payload, sizeof(payload), &frame) &&
           can_bus_service_send_canopen(can3, &frame);
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
                               command->height_rate_mm_s) && ok;
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
