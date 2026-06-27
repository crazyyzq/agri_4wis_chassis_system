#include "modbus_rtu.h"

uint16_t modbus_rtu_crc16(const uint8_t *data, size_t size)
{
    uint16_t crc = 0xFFFFU;
    if (data == 0) {
        return crc;
    }
    for (size_t byte_index = 0U; byte_index < size; ++byte_index) {
        crc ^= data[byte_index];
        for (uint8_t bit_index = 0U; bit_index < 8U; ++bit_index) {
            if ((crc & 1U) != 0U) {
                crc = (uint16_t)((crc >> 1U) ^ 0xA001U);
            } else {
                crc >>= 1U;
            }
        }
    }
    return crc;
}

bool modbus_rtu_build_read_input_registers(uint8_t slave_id,
                                           uint16_t start_register,
                                           uint16_t register_count,
                                           modbus_rtu_frame_t *out)
{
    if (out == 0 || register_count == 0U) {
        return false;
    }
    out->data[0] = slave_id;
    out->data[1] = 4U;
    out->data[2] = (uint8_t)(start_register >> 8U);
    out->data[3] = (uint8_t)start_register;
    out->data[4] = (uint8_t)(register_count >> 8U);
    out->data[5] = (uint8_t)register_count;
    uint16_t crc = modbus_rtu_crc16(out->data, 6U);
    out->data[6] = (uint8_t)crc;
    out->data[7] = (uint8_t)(crc >> 8U);
    out->size = 8U;
    return true;
}

bool modbus_rtu_build_write_single_register(uint8_t slave_id,
                                            uint16_t register_address,
                                            uint16_t value,
                                            modbus_rtu_frame_t *out)
{
    if (out == 0) {
        return false;
    }
    out->data[0] = slave_id;
    out->data[1] = 6U;
    out->data[2] = (uint8_t)(register_address >> 8U);
    out->data[3] = (uint8_t)register_address;
    out->data[4] = (uint8_t)(value >> 8U);
    out->data[5] = (uint8_t)value;
    uint16_t crc = modbus_rtu_crc16(out->data, 6U);
    out->data[6] = (uint8_t)crc;
    out->data[7] = (uint8_t)(crc >> 8U);
    out->size = 8U;
    return true;
}
