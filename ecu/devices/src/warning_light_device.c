#include <string.h>

#include "modbus_rtu.h"
#include "warning_light_device.h"

static uint16_t warning_light_value(indicator_mode_t mode)
{
    switch (mode) {
    case INDICATOR_LEFT:
        return 1U;
    case INDICATOR_RIGHT:
        return 2U;
    case INDICATOR_HAZARD_USER:
        return 3U;
    case INDICATOR_HAZARD_SAFETY:
        return 4U;
    case INDICATOR_OFF:
    default:
        return 0U;
    }
}

void warning_light_device_init(warning_light_device_state_t *state)
{
    if (state != 0) {
        memset(state, 0, sizeof(*state));
        state->last_result = ECU_DEVICE_APPLY_OK;
    }
}

ecu_device_apply_result_t warning_light_device_apply(warning_light_device_state_t *state,
                                                     uart_comm_service_t *rs485,
                                                     const ecu_hardware_config_t *config,
                                                     indicator_mode_t indicator_mode)
{
    if (state == 0 || rs485 == 0 || config == 0) {
        return ECU_DEVICE_APPLY_INVALID_ARGUMENT;
    }
    if (!rs485->online) {
        state->last_result = ECU_DEVICE_APPLY_BACKEND_OFFLINE;
        return state->last_result;
    }

    modbus_rtu_frame_t frame;
    bool ok = modbus_rtu_build_write_single_register(config->modbus_warning_light_slave_id,
                                                     0U,
                                                     warning_light_value(indicator_mode),
                                                     &frame) &&
              uart_comm_service_send(rs485, frame.data, frame.size);
    state->last_indicator_mode = indicator_mode;
    state->apply_count++;
    state->last_result = ok ? ECU_DEVICE_APPLY_OK : ECU_DEVICE_APPLY_REJECTED;
    return state->last_result;
}
