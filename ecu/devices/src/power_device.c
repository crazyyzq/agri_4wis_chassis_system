#include <string.h>

#include "canopen_frame.h"
#include "power_device.h"

static bool send_power_node(can_bus_service_t *can1,
                            const ecu_canopen_node_config_t *node,
                            bool high_voltage_enable)
{
    uint8_t payload[2] = {
        high_voltage_enable ? 1U : 0U,
        0U
    };
    canopen_frame_t frame;
    return canopen_frame_build_pdo(node->rpdo1_cob_id, payload, sizeof(payload), &frame) &&
           can_bus_service_send_canopen(can1, &frame);
}

void power_device_init(power_device_state_t *state)
{
    if (state != 0) {
        memset(state, 0, sizeof(*state));
        state->last_result = ECU_DEVICE_APPLY_OK;
    }
}

ecu_device_apply_result_t power_device_apply(power_device_state_t *state,
                                             can_bus_service_t *can1,
                                             const ecu_hardware_config_t *config,
                                             bool high_voltage_enable)
{
    if (state == 0 || can1 == 0 || config == 0) {
        return ECU_DEVICE_APPLY_INVALID_ARGUMENT;
    }
    if (!can1->online) {
        state->last_result = ECU_DEVICE_APPLY_BACKEND_OFFLINE;
        return state->last_result;
    }

    bool ok = send_power_node(can1, &config->bms_node, high_voltage_enable) &&
              send_power_node(can1, &config->dcdc_48v_node, high_voltage_enable) &&
              send_power_node(can1, &config->dcdc_12v_node, high_voltage_enable) &&
              send_power_node(can1, &config->inverter_node, high_voltage_enable);
    state->high_voltage_requested = high_voltage_enable;
    state->apply_count++;
    state->last_result = ok ? ECU_DEVICE_APPLY_OK : ECU_DEVICE_APPLY_REJECTED;
    return state->last_result;
}
