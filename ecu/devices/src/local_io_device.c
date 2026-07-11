#include <string.h>

#include "local_io_device.h"

void local_io_device_init(local_io_device_state_t *state)
{
    if (state != 0) {
        memset(state, 0, sizeof(*state));
        state->last_result = ECU_DEVICE_APPLY_OK;
    }
}

ecu_device_apply_result_t local_io_device_apply(local_io_device_state_t *state,
                                                const ecu_hardware_config_t *config,
                                                const vehicle_actuator_command_t *command)
{
    if (state == 0 || config == 0 || command == 0) {
        return ECU_DEVICE_APPLY_INVALID_ARGUMENT;
    }

    uint32_t output_mask = 0U;
    if (command->horn_on) {
        output_mask |= config->dio_horn_mask;
    }
    if (command->headlight_on) {
        output_mask |= config->dio_headlight_mask;
    }
    if (command->high_voltage_disable_request) {
        state->high_voltage_relay_latched = false;
    } else if (command->high_voltage_enable) {
        state->high_voltage_relay_latched = true;
    }
    if (state->high_voltage_relay_latched) {
        output_mask |= config->dio_high_voltage_relay_mask;
    }
    if (command->indicator_mode == INDICATOR_LEFT ||
        command->indicator_mode == INDICATOR_HAZARD_USER ||
        command->indicator_mode == INDICATOR_HAZARD_SAFETY) {
        output_mask |= config->dio_left_indicator_mask;
    }
    if (command->indicator_mode == INDICATOR_RIGHT ||
        command->indicator_mode == INDICATOR_HAZARD_USER ||
        command->indicator_mode == INDICATOR_HAZARD_SAFETY) {
        output_mask |= config->dio_right_indicator_mask;
    }

    /* Hydraulic valve bits are deliberately excluded.  CAN3 publishes those
     * separately and the IO task merges both snapshots exactly once.
     */
    state->last_output_mask =
        output_mask & (config->dio_managed_output_mask &
                       ~config->hydraulic_managed_valve_mask);
    state->apply_count++;
    state->last_result = ECU_DEVICE_APPLY_OK;
    return state->last_result;
}
