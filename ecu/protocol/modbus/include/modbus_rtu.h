#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "ecu_config.h"

#define MODBUS_RTU_MAX_ADU_BYTES (256U)

typedef struct {
    uint8_t data[MODBUS_RTU_MAX_ADU_BYTES];
    size_t size;
} modbus_rtu_frame_t;

typedef struct {
    uint16_t registers[ECU_ADC_CHANNEL_COUNT];
    uint8_t register_count;
} modbus_rtu_register_response_t;

/* Build a Modbus RTU read-input-registers request through HPM SDK
 * agile_modbus.
 *
 * Units: register address and count are Modbus register units, not bytes.
 * Owner: UART/RS485 device task code.
 * ISR: not safe.
 * Failure behavior: returns false for invalid arguments or library errors.
 *
 * This wrapper exists so device code can express register-level operations
 * without depending on the Modbus stack API directly. It must remain a thin
 * adapter; do not add local CRC, parser or protocol-state logic here.
 */
bool modbus_rtu_build_read_input_registers(uint8_t slave_id,
                                           uint16_t start_register,
                                           uint16_t register_count,
                                           modbus_rtu_frame_t *out);
bool modbus_rtu_build_write_single_register(uint8_t slave_id,
                                            uint16_t register_address,
                                            uint16_t value,
                                            modbus_rtu_frame_t *out);

bool modbus_rtu_extract_read_input_registers(const uint8_t *adu,
                                             size_t adu_size,
                                             uint8_t expected_slave_id,
                                             uint16_t expected_count,
                                             modbus_rtu_register_response_t *out);

#endif /* MODBUS_RTU_H */
