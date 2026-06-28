#include "servo_drive_canopen.h"

#include "canopen_frame.h"

static bool servo_drive_cob_id_enabled(uint32_t cob_id)
{
    return (cob_id & ECU_CANOPEN_COB_ID_DISABLED) == 0U;
}

static void servo_drive_pack_u16_le(uint16_t value, uint8_t *out)
{
    out[0] = (uint8_t)value;
    out[1] = (uint8_t)(value >> 8U);
}

static void servo_drive_pack_i16_le(int16_t value, uint8_t *out)
{
    uint16_t raw = (uint16_t)value;
    out[0] = (uint8_t)raw;
    out[1] = (uint8_t)(raw >> 8U);
}

static void servo_drive_pack_i32_le(int32_t value, uint8_t *out)
{
    uint32_t raw = (uint32_t)value;
    out[0] = (uint8_t)raw;
    out[1] = (uint8_t)(raw >> 8U);
    out[2] = (uint8_t)(raw >> 16U);
    out[3] = (uint8_t)(raw >> 24U);
}

static bool servo_drive_send_pdo(can_bus_service_t *bus,
                                 uint32_t cob_id,
                                 const uint8_t *payload,
                                 uint8_t payload_size)
{
    canopen_frame_t frame;

    if (bus == 0 || payload == 0 || !servo_drive_cob_id_enabled(cob_id)) {
        return false;
    }

    return canopen_frame_build_pdo(cob_id, payload, payload_size, &frame) &&
           can_bus_service_send_canopen(bus, &frame);
}

bool servo_drive_canopen_send_control_word(can_bus_service_t *bus,
                                           const ecu_canopen_node_config_t *node,
                                           uint16_t control_word)
{
    uint8_t payload[2];

    if (node == 0) {
        return false;
    }

    servo_drive_pack_u16_le(control_word, payload);
    return servo_drive_send_pdo(bus, node->rpdo1_cob_id, payload, sizeof(payload));
}

bool servo_drive_canopen_select_mode(can_bus_service_t *bus,
                                     const ecu_canopen_node_config_t *node,
                                     uint16_t control_word,
                                     servo_drive_mode_t mode)
{
    uint8_t payload[3];

    if (node == 0) {
        return false;
    }

    servo_drive_pack_u16_le(control_word, &payload[0]);
    payload[2] = (uint8_t)mode;
    return servo_drive_send_pdo(bus, node->rpdo2_cob_id, payload, sizeof(payload));
}

bool servo_drive_canopen_set_target_position(can_bus_service_t *bus,
                                             const ecu_canopen_node_config_t *node,
                                             uint16_t control_word,
                                             int32_t target_position_counts)
{
    uint8_t payload[6];

    if (node == 0) {
        return false;
    }

    servo_drive_pack_u16_le(control_word, &payload[0]);
    servo_drive_pack_i32_le(target_position_counts, &payload[2]);
    return servo_drive_send_pdo(bus, node->rpdo3_cob_id, payload, sizeof(payload));
}

bool servo_drive_canopen_set_target_velocity(can_bus_service_t *bus,
                                             const ecu_canopen_node_config_t *node,
                                             uint16_t control_word,
                                             int32_t target_velocity_counts_per_sec)
{
    uint8_t payload[6];

    if (node == 0) {
        return false;
    }

    servo_drive_pack_u16_le(control_word, &payload[0]);
    servo_drive_pack_i32_le(target_velocity_counts_per_sec, &payload[2]);
    return servo_drive_send_pdo(bus, node->rpdo4_cob_id, payload, sizeof(payload));
}

bool servo_drive_canopen_set_target_torque(can_bus_service_t *bus,
                                           const ecu_canopen_node_config_t *node,
                                           uint16_t control_word,
                                           int16_t target_torque_raw)
{
    uint8_t payload[4];

    if (node == 0) {
        return false;
    }

    servo_drive_pack_u16_le(control_word, &payload[0]);
    servo_drive_pack_i16_le(target_torque_raw, &payload[2]);
    return servo_drive_send_pdo(bus, node->rpdo5_cob_id, payload, sizeof(payload));
}
