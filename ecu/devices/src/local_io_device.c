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
                                                dio_service_t *dio,
                                                const ecu_hardware_config_t *config,
                                                const vehicle_actuator_command_t *command)
{
    if (state == 0 || dio == 0 || config == 0 || command == 0) {
        return ECU_DEVICE_APPLY_INVALID_ARGUMENT;
    }

    dio_service_write_masked(dio, config->dio_brake_release_mask, command->brake_release);
    dio_service_write_masked(dio, config->dio_horn_mask, command->horn_on);
    dio_service_write_masked(dio, config->dio_headlight_mask, command->headlight_on);
    dio_service_write_masked(dio,
                             config->dio_left_indicator_mask,
                             command->indicator_mode == INDICATOR_LEFT ||
                             command->indicator_mode == INDICATOR_HAZARD_USER ||
                             command->indicator_mode == INDICATOR_HAZARD_SAFETY);
    dio_service_write_masked(dio,
                             config->dio_right_indicator_mask,
                             command->indicator_mode == INDICATOR_RIGHT ||
                             command->indicator_mode == INDICATOR_HAZARD_USER ||
                             command->indicator_mode == INDICATOR_HAZARD_SAFETY);

    state->last_output_mask = dio->output_mask;
    state->apply_count++;
    state->last_result = ECU_DEVICE_APPLY_OK;
    return state->last_result;
}
