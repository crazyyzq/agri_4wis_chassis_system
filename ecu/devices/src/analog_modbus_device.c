#include "analog_modbus_device.h"

#include <string.h>

static bool analog_modbus_init_context(agile_modbus_rtu_t *rtu,
                                       uint8_t *send_buffer,
                                       uint8_t *read_buffer,
                                       const ecu_hardware_config_t *config)
{
    if (rtu == 0 || send_buffer == 0 || read_buffer == 0 || config == 0) {
        return false;
    }

    if (agile_modbus_rtu_init(rtu,
                              send_buffer,
                              MODBUS_MASTER_MAX_ADU_BYTES,
                              read_buffer,
                              MODBUS_MASTER_MAX_ADU_BYTES) != 0) {
        return false;
    }

    return agile_modbus_set_slave(&rtu->_ctx, config->modbus_adc_slave_id) == 0;
}

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
                                        modbus_master_request_t *out)
{
    agile_modbus_rtu_t rtu;
    uint8_t read_buffer[MODBUS_MASTER_MAX_ADU_BYTES] = {0};
    int length;

    if (config == 0 || out == 0 ||
        config->modbus_adc_register_count == 0U ||
        config->modbus_adc_register_count > ECU_ADC_CHANNEL_COUNT) {
        return false;
    }

    memset(out, 0, sizeof(*out));
    if (!analog_modbus_init_context(&rtu, out->data, read_buffer, config)) {
        return false;
    }

    length = agile_modbus_serialize_read_input_registers(&rtu._ctx,
                                                         config->modbus_adc_start_register,
                                                         config->modbus_adc_register_count);
    if (length <= 0 || length > (int)sizeof(out->data)) {
        return false;
    }

    out->size = (size_t)length;
    return true;
}

bool analog_modbus_device_apply_response(analog_modbus_device_state_t *state,
                                         analog_input_service_t *analog_inputs,
                                         const ecu_hardware_config_t *config,
                                         const uint8_t *adu,
                                         size_t adu_size,
                                         uint32_t now_ms)
{
    agile_modbus_rtu_t rtu;
    uint8_t send_buffer[MODBUS_MASTER_MAX_ADU_BYTES] = {0};
    uint8_t read_buffer[MODBUS_MASTER_MAX_ADU_BYTES] = {0};
    uint16_t registers[ECU_ADC_CHANNEL_COUNT] = {0};
    int result;

    if (state == 0 || analog_inputs == 0 || config == 0) {
        return false;
    }

    if (adu == 0 || adu_size > sizeof(read_buffer) ||
        !analog_modbus_init_context(&rtu, send_buffer, read_buffer, config)) {
        state->error_count++;
        state->online = false;
        return false;
    }

    (void)agile_modbus_serialize_read_input_registers(&rtu._ctx,
                                                      config->modbus_adc_start_register,
                                                      config->modbus_adc_register_count);
    memcpy(read_buffer, adu, adu_size);
    result = agile_modbus_deserialize_read_input_registers(&rtu._ctx,
                                                           (int)adu_size,
                                                           registers);
    if (result < 0) {
        state->error_count++;
        state->online = false;
        return false;
    }

    for (uint8_t channel = 0U; channel < config->modbus_adc_register_count; ++channel) {
        uint32_t millivolt = scale_raw_to_millivolt(registers[channel], config);
        state->raw[channel] = registers[channel];
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
    modbus_master_request_t request;
    modbus_master_snapshot_t master_snapshot;
    analog_modbus_response_context_t context;
    uint32_t previous_timeout_count;

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

    modbus_master_service_get_snapshot(master, &master_snapshot);
    previous_timeout_count = master_snapshot.timeout_count;

    modbus_master_service_process(master,
                                  uart,
                                  now_ms,
                                  &request,
                                  (size_t)(5U + (config->modbus_adc_register_count * 2U)),
                                  analog_modbus_response_handler,
                                  &context);

    modbus_master_service_get_snapshot(master, &master_snapshot);
    state->request_count = master_snapshot.tx_count;
    if (master_snapshot.timeout_count > previous_timeout_count) {
        state->online = false;
    }
    if (master_snapshot.error_count > state->error_count) {
        state->error_count = master_snapshot.error_count;
    }
}
