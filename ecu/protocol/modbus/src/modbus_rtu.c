#include "modbus_rtu.h"

#include <string.h>

#include "agile_modbus.h"

static bool modbus_rtu_prepare_context(uint8_t slave_id,
                                       modbus_rtu_frame_t *out,
                                       agile_modbus_rtu_t *rtu,
                                       agile_modbus_t **ctx)
{
    static uint8_t unused_read_buffer[MODBUS_RTU_MAX_ADU_BYTES];

    if (out == 0 || rtu == 0 || ctx == 0) {
        return false;
    }

    memset(out, 0, sizeof(*out));
    agile_modbus_rtu_init(rtu,
                          out->data,
                          (int)sizeof(out->data),
                          unused_read_buffer,
                          (int)sizeof(unused_read_buffer));
    *ctx = &rtu->_ctx;
    return agile_modbus_set_slave(*ctx, (int)slave_id) == 0;
}

static bool modbus_rtu_finish_serialized_frame(modbus_rtu_frame_t *out, int frame_size)
{
    if (out == 0 || frame_size <= 0 || frame_size > (int)sizeof(out->data)) {
        return false;
    }

    out->size = (size_t)frame_size;
    return true;
}

bool modbus_rtu_build_read_input_registers(uint8_t slave_id,
                                           uint16_t start_register,
                                           uint16_t register_count,
                                           modbus_rtu_frame_t *out)
{
    agile_modbus_rtu_t rtu;
    agile_modbus_t *ctx = 0;

    if (out == 0 || register_count == 0U) {
        return false;
    }

    if (!modbus_rtu_prepare_context(slave_id, out, &rtu, &ctx)) {
        return false;
    }

    int frame_size = agile_modbus_serialize_read_input_registers(ctx,
                                                                 (int)start_register,
                                                                 (int)register_count);
    return modbus_rtu_finish_serialized_frame(out, frame_size);
}

bool modbus_rtu_build_write_single_register(uint8_t slave_id,
                                            uint16_t register_address,
                                            uint16_t value,
                                            modbus_rtu_frame_t *out)
{
    agile_modbus_rtu_t rtu;
    agile_modbus_t *ctx = 0;

    if (!modbus_rtu_prepare_context(slave_id, out, &rtu, &ctx)) {
        return false;
    }

    int frame_size = agile_modbus_serialize_write_register(ctx,
                                                           (int)register_address,
                                                           value);
    return modbus_rtu_finish_serialized_frame(out, frame_size);
}

bool modbus_rtu_extract_read_input_registers(const uint8_t *adu,
                                             size_t adu_size,
                                             uint8_t expected_slave_id,
                                             uint16_t expected_count,
                                             modbus_rtu_register_response_t *out)
{
    uint8_t send_buffer[MODBUS_RTU_MAX_ADU_BYTES];
    uint8_t read_buffer[MODBUS_RTU_MAX_ADU_BYTES];
    agile_modbus_rtu_t rtu;
    agile_modbus_t *ctx = 0;

    if (adu == 0 || out == 0 || expected_count == 0U ||
        expected_count > ECU_ADC_CHANNEL_COUNT ||
        adu_size == 0U || adu_size > sizeof(read_buffer)) {
        return false;
    }

    memset(out, 0, sizeof(*out));
    memset(send_buffer, 0, sizeof(send_buffer));
    memset(read_buffer, 0, sizeof(read_buffer));
    memcpy(read_buffer, adu, adu_size);

    agile_modbus_rtu_init(&rtu,
                          send_buffer,
                          (int)sizeof(send_buffer),
                          read_buffer,
                          (int)sizeof(read_buffer));
    ctx = &rtu._ctx;
    if (agile_modbus_set_slave(ctx, (int)expected_slave_id) != 0) {
        return false;
    }

    /* agile_modbus validates read responses against the last serialized
     * request.  The address is not part of the read-response value check; the
     * expected register count is the important contract for the 8AI module.
     */
    int request_size = agile_modbus_serialize_read_input_registers(ctx,
                                                                   0,
                                                                   (int)expected_count);
    if (request_size <= 0) {
        return false;
    }

    int rc = agile_modbus_deserialize_read_input_registers(ctx,
                                                           (int)adu_size,
                                                           out->registers);
    if (rc != (int)expected_count) {
        memset(out, 0, sizeof(*out));
        return false;
    }

    out->register_count = (uint8_t)rc;
    return true;
}
