#include "analog_modbus_device.h"

#include <string.h>

typedef struct {
    analog_modbus_device_state_t *state;
    analog_input_service_t *analog_inputs;
    const ecu_hardware_config_t *config;
    uint32_t now_ms;
} analog_modbus_response_context_t;

static uint32_t scale_raw_to_millivolt(uint16_t raw,
                                       const ecu_hardware_config_t *config)
{
    if (config == 0 || config->modbus_adc_raw_max == 0U) {
        return 0U;
    }

    return ((uint32_t)raw * config->adc_external_mv_max) /
           config->modbus_adc_raw_max;
}

static bool analog_modbus_response_handler(void *context,
                                           const uint8_t *adu,
                                           size_t adu_size)
{
    analog_modbus_response_context_t *typed =
        (analog_modbus_response_context_t *)context;
    if (typed == 0) {
        return false;
    }

    return analog_modbus_device_apply_response(typed->state,
                                               typed->analog_inputs,
                                               typed->config,
                                               adu,
                                               adu_size,
                                               typed->now_ms);
}

void analog_modbus_device_init(analog_modbus_device_state_t *state)
{
    if (state != 0) {
        memset(state, 0, sizeof(*state));
    }
}

bool analog_modbus_device_build_request(const ecu_hardware_config_t *config,
                                        modbus_rtu_frame_t *out)
{
    if (config == 0 || out == 0 ||
        config->modbus_adc_register_count == 0U ||
        config->modbus_adc_register_count > ECU_ADC_CHANNEL_COUNT) {
        return false;
    }

    return modbus_rtu_build_read_input_registers(config->modbus_adc_slave_id,
                                                 config->modbus_adc_start_register,
                                                 config->modbus_adc_register_count,
                                                 out);
}

bool analog_modbus_device_apply_response(analog_modbus_device_state_t *state,
                                         analog_input_service_t *analog_inputs,
                                         const ecu_hardware_config_t *config,
                                         const uint8_t *adu,
                                         size_t adu_size,
                                         uint32_t now_ms)
{
    modbus_rtu_register_response_t response;

    if (state == 0 || analog_inputs == 0 || config == 0) {
        return false;
    }

    if (!modbus_rtu_extract_read_input_registers(adu,
                                                adu_size,
                                                config->modbus_adc_slave_id,
                                                config->modbus_adc_register_count,
                                                &response)) {
        state->error_count++;
        state->online = false;
        return false;
    }

    for (uint8_t channel = 0U; channel < response.register_count; ++channel) {
        uint32_t millivolt = scale_raw_to_millivolt(response.registers[channel],
                                                   config);
        state->raw[channel] = response.registers[channel];
        state->millivolt[channel] = millivolt;
        (void)analog_input_service_update_millivolt(analog_inputs,
                                                    channel,
                                                    millivolt);
    }

    state->response_count++;
    state->last_response_ms = now_ms;
    state->online = true;
    return true;
}

void analog_modbus_device_process(analog_modbus_device_state_t *state,
                                  modbus_master_service_t *master,
                                  uart_rs485_hw_t *uart,
                                  analog_input_service_t *analog_inputs,
                                  const ecu_hardware_config_t *config,
                                  uint32_t now_ms)
{
    modbus_rtu_frame_t request;
    modbus_master_snapshot_t master_snapshot;
    analog_modbus_response_context_t context;

    if (state == 0 || master == 0 || uart == 0 || analog_inputs == 0 ||
        config == 0) {
        return;
    }

    if (!analog_modbus_device_build_request(config, &request)) {
        state->error_count++;
        state->online = false;
        return;
    }

    context.state = state;
    context.analog_inputs = analog_inputs;
    context.config = config;
    context.now_ms = now_ms;

    modbus_master_service_process(master,
                                  uart,
                                  now_ms,
                                  &request,
                                  (size_t)(5U + (config->modbus_adc_register_count * 2U)),
                                  analog_modbus_response_handler,
                                  &context);

    modbus_master_service_get_snapshot(master, &master_snapshot);
    state->request_count = master_snapshot.tx_count;
    if (master_snapshot.error_count > state->error_count) {
        state->error_count = master_snapshot.error_count;
    }
}
