#ifndef ANALOG_MODBUS_DEVICE_H
#define ANALOG_MODBUS_DEVICE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "analog_input_service.h"
#include "ecu_config.h"
#include "modbus_master_service.h"
#include "uart_rs485_hw.h"

typedef struct {
    uint32_t request_count;
    uint32_t response_count;
    uint32_t error_count;
    uint16_t raw[ECU_ADC_CHANNEL_COUNT];
    uint32_t millivolt[ECU_ADC_CHANNEL_COUNT];
    uint32_t last_response_ms;
    bool online;
} analog_modbus_device_state_t;

void analog_modbus_device_init(analog_modbus_device_state_t *state);

bool analog_modbus_device_build_request(const ecu_hardware_config_t *config,
                                        modbus_master_request_t *out);

bool analog_modbus_device_apply_response(analog_modbus_device_state_t *state,
                                         analog_input_service_t *analog_inputs,
                                         const ecu_hardware_config_t *config,
                                         const uint8_t *adu,
                                         size_t adu_size,
                                         uint32_t now_ms);

void analog_modbus_device_process(analog_modbus_device_state_t *state,
                                  modbus_master_service_t *master,
                                  uart_rs485_hw_t *uart,
                                  analog_input_service_t *analog_inputs,
                                  const ecu_hardware_config_t *config,
                                  uint32_t now_ms);

#endif /* ANALOG_MODBUS_DEVICE_H */
