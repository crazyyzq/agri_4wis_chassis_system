#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define MODBUS_RTU_MAX_ADU_BYTES (256U)

typedef struct {
    uint8_t data[MODBUS_RTU_MAX_ADU_BYTES];
    size_t size;
} modbus_rtu_frame_t;

uint16_t modbus_rtu_crc16(const uint8_t *data, size_t size);

/* Build a Modbus RTU read-input-registers request.
 *
 * Units: register address and count are Modbus register units, not bytes.
 * Owner: UART/RS485 device task code.
 * ISR: not safe.
 * Failure behavior: returns false for invalid arguments or oversized frames.
 */
bool modbus_rtu_build_read_input_registers(uint8_t slave_id,
                                           uint16_t start_register,
                                           uint16_t register_count,
                                           modbus_rtu_frame_t *out);
bool modbus_rtu_build_write_single_register(uint8_t slave_id,
                                            uint16_t register_address,
                                            uint16_t value,
                                            modbus_rtu_frame_t *out);

#endif /* MODBUS_RTU_H */
