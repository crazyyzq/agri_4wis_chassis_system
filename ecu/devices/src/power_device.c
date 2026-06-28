#include <string.h>

#include "power_device.h"

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

    if (config->power_protocol == ECU_POWER_PROTOCOL_DISABLED) {
        state->high_voltage_requested = false;
        state->apply_count++;
        state->last_result = high_voltage_enable ?
                             ECU_DEVICE_APPLY_UNCONFIGURED :
                             ECU_DEVICE_APPLY_OK;
        return state->last_result;
    }

    state->high_voltage_requested = false;
    state->apply_count++;
    state->last_result = ECU_DEVICE_APPLY_UNCONFIGURED;
    return state->last_result;
}
