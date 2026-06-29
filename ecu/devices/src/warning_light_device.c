#include "warning_light_device.h"

#include <string.h>

typedef struct {
    warning_light_device_state_t *state;
    const ecu_hardware_config_t *config;
    uint16_t register_value;
    uint32_t now_ms;
} warning_light_response_context_t;

static bool warning_light_build_request(const ecu_hardware_config_t *config,
                                        uint16_t register_value,
                                        modbus_master_request_t *out)
{
    agile_modbus_rtu_t rtu;
    uint8_t read_buffer[MODBUS_MASTER_MAX_ADU_BYTES] = {0};
    int length;

    if (config == 0 || out == 0) {
        return false;
    }

    memset(out, 0, sizeof(*out));
    if (agile_modbus_rtu_init(&rtu,
                              out->data,
                              MODBUS_MASTER_MAX_ADU_BYTES,
                              read_buffer,
                              MODBUS_MASTER_MAX_ADU_BYTES) != 0) {
        return false;
    }
    if (agile_modbus_set_slave(&rtu._ctx,
                               config->modbus_warning_light_slave_id) != 0) {
        return false;
    }

    length = agile_modbus_serialize_write_register(&rtu._ctx,
                                                   config->modbus_warning_light_register,
                                                   register_value);
    if (length <= 0 || length > (int)sizeof(out->data)) {
        return false;
    }

    out->size = (size_t)length;
    return true;
}

static uint16_t warning_light_value(indicator_mode_t mode)
{
    switch (mode) {
    case INDICATOR_LEFT:
    case INDICATOR_RIGHT:
    case INDICATOR_HAZARD_USER:
        return ECU_WARNING_LIGHT_VALUE_YELLOW_SLOW_FLASH;
    case INDICATOR_HAZARD_SAFETY:
        return ECU_WARNING_LIGHT_VALUE_RED_STEADY_BUZZER;
    case INDICATOR_OFF:
    default:
        return ECU_WARNING_LIGHT_VALUE_OFF;
    }
}

static bool warning_light_response_handler(void *context,
                                           const uint8_t *adu,
                                           size_t adu_size)
{
    warning_light_response_context_t *typed =
        (warning_light_response_context_t *)context;
    if (typed == 0 || typed->state == 0 || typed->config == 0 || adu == 0) {
        return false;
    }

    warning_light_device_state_t *state = typed->state;
    agile_modbus_rtu_t rtu;
    uint8_t send_buffer[MODBUS_MASTER_MAX_ADU_BYTES] = {0};
    uint8_t read_buffer[MODBUS_MASTER_MAX_ADU_BYTES] = {0};
    bool ok = false;

    if (adu_size <= sizeof(read_buffer) &&
        agile_modbus_rtu_init(&rtu,
                              send_buffer,
                              MODBUS_MASTER_MAX_ADU_BYTES,
                              read_buffer,
                              MODBUS_MASTER_MAX_ADU_BYTES) == 0 &&
        agile_modbus_set_slave(&rtu._ctx,
                               typed->config->modbus_warning_light_slave_id) == 0) {
        (void)agile_modbus_serialize_write_register(&rtu._ctx,
                                                    typed->config->modbus_warning_light_register,
                                                    typed->register_value);
        memcpy(read_buffer, adu, adu_size);
        ok = agile_modbus_deserialize_write_register(&rtu._ctx, (int)adu_size) == 0;
    }

    if (ok) {
        state->response_count++;
        state->last_response_ms = typed->now_ms;
        state->online = true;
    } else {
        state->error_count++;
        state->online = false;
    }
    return ok;
}

void warning_light_device_init(warning_light_device_state_t *state)
{
    if (state != 0) {
        memset(state, 0, sizeof(*state));
        state->last_result = ECU_DEVICE_APPLY_OK;
    }
}

ecu_device_apply_result_t warning_light_device_apply(warning_light_device_state_t *state,
                                                     modbus_master_service_t *master,
                                                     uart_rs485_hw_t *uart,
                                                     const ecu_hardware_config_t *config,
                                                     indicator_mode_t indicator_mode,
                                                     uint32_t now_ms)
{
    modbus_master_request_t request;
    modbus_master_snapshot_t before;
    modbus_master_snapshot_t after;
    warning_light_response_context_t context;
    uint16_t register_value;

    if (state == 0 || master == 0 || uart == 0 || config == 0) {
        return ECU_DEVICE_APPLY_INVALID_ARGUMENT;
    }
    if (!uart->initialized) {
        state->online = false;
        state->last_result = ECU_DEVICE_APPLY_BACKEND_OFFLINE;
        return state->last_result;
    }

    modbus_master_service_get_snapshot(master, &before);
    register_value = warning_light_value(indicator_mode);
    if (!warning_light_build_request(config, register_value, &request)) {
        state->error_count++;
        state->online = false;
        state->last_result = ECU_DEVICE_APPLY_REJECTED;
        return state->last_result;
    }

    if (before.state == MODBUS_MASTER_STATE_IDLE) {
        memset(state->expected_response, 0, sizeof(state->expected_response));
        memcpy(state->expected_response, request.data, request.size);
        state->expected_response_size = request.size;
        state->last_indicator_mode = indicator_mode;
        state->last_register_value = register_value;
    }

    context.state = state;
    context.config = config;
    context.register_value = register_value;
    context.now_ms = now_ms;

    modbus_master_service_process(master,
                                  uart,
                                  now_ms,
                                  &request,
                                  request.size,
                                  warning_light_response_handler,
                                  &context);
    modbus_master_service_get_snapshot(master, &after);

    state->apply_count++;
    if (after.timeout_count > before.timeout_count) {
        state->online = false;
    }
    if (after.error_count > before.error_count) {
        state->error_count = after.error_count;
        state->last_result = ECU_DEVICE_APPLY_REJECTED;
        return state->last_result;
    }

    state->last_result = ECU_DEVICE_APPLY_OK;
    return state->last_result;
}
