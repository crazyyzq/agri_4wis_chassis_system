#include "servo_drive_canopen.h"

static bool servo_drive_node_valid(const ecu_canopen_node_config_t *node)
{
    return node != 0 && node->node_id != 0U;
}

static bool servo_drive_write_sdo(canopen_master_service_t *canopen,
                                  const ecu_canopen_node_config_t *node,
                                  uint16_t index,
                                  uint8_t size,
                                  int32_t value)
{
    if (canopen == 0 || !servo_drive_node_valid(node)) {
        return false;
    }

    return canopen_master_service_request_sdo_write(canopen,
                                                    node->node_id,
                                                    index,
                                                    0U,
                                                    size,
                                                    value);
}

bool servo_drive_canopen_send_control_word(canopen_master_service_t *canopen,
                                           const ecu_canopen_node_config_t *node,
                                           uint16_t control_word)
{
    return servo_drive_write_sdo(canopen,
                                 node,
                                 ECU_CANOPEN_OBJ_CONTROLWORD,
                                 2U,
                                 (int32_t)control_word);
}

bool servo_drive_canopen_select_mode(canopen_master_service_t *canopen,
                                     const ecu_canopen_node_config_t *node,
                                     uint16_t control_word,
                                     servo_drive_mode_t mode)
{
    return servo_drive_write_sdo(canopen,
                                 node,
                                 ECU_CANOPEN_OBJ_MODES_OF_OPERATION,
                                 1U,
                                 (int32_t)mode) &&
           servo_drive_canopen_send_control_word(canopen, node, control_word);
}

bool servo_drive_canopen_set_target_position(canopen_master_service_t *canopen,
                                             const ecu_canopen_node_config_t *node,
                                             uint16_t control_word,
                                             int32_t target_position_counts)
{
    return servo_drive_write_sdo(canopen,
                                 node,
                                 ECU_CANOPEN_OBJ_TARGET_POSITION,
                                 4U,
                                 target_position_counts) &&
           servo_drive_canopen_send_control_word(canopen, node, control_word);
}

bool servo_drive_canopen_set_target_velocity(canopen_master_service_t *canopen,
                                             const ecu_canopen_node_config_t *node,
                                             uint16_t control_word,
                                             int32_t target_velocity_counts_per_sec)
{
    return servo_drive_write_sdo(canopen,
                                 node,
                                 ECU_CANOPEN_OBJ_TARGET_VELOCITY,
                                 4U,
                                 target_velocity_counts_per_sec) &&
           servo_drive_canopen_send_control_word(canopen, node, control_word);
}

bool servo_drive_canopen_set_target_torque(canopen_master_service_t *canopen,
                                           const ecu_canopen_node_config_t *node,
                                           uint16_t control_word,
                                           int16_t target_torque_raw)
{
    return servo_drive_write_sdo(canopen,
                                 node,
                                 ECU_CANOPEN_OBJ_TARGET_TORQUE,
                                 2U,
                                 (int32_t)target_torque_raw) &&
           servo_drive_canopen_send_control_word(canopen, node, control_word);
}

bool servo_drive_canopen_set_output_state(canopen_master_service_t *canopen,
                                          const ecu_canopen_node_config_t *node,
                                          uint16_t output_mask,
                                          uint16_t active_mask)
{
    if (output_mask == 0U) {
        return true;
    }

    uint16_t value = active_mask & output_mask;
    return servo_drive_write_sdo(canopen,
                                 node,
                                 ECU_CANOPEN_OBJ_OUTPUT_STATES_PROGRAM_CONTROL,
                                 2U,
                                 (int32_t)value);
}

bool servo_drive_canopen_read_input_states(canopen_master_service_t *canopen,
                                           const ecu_canopen_node_config_t *node,
                                           uint16_t *input_states)
{
    if (canopen == 0 || !servo_drive_node_valid(node) || input_states == 0) {
        return false;
    }

    canopen_master_snapshot_t snapshot;
    canopen_master_service_get_snapshot(canopen, &snapshot);

    (void)canopen_master_service_request_sdo_read(canopen,
                                                  node->node_id,
                                                  ECU_CANOPEN_OBJ_DIGITAL_INPUT_STATES,
                                                  0U);

    if (snapshot.last_sdo_node_id != node->node_id ||
        snapshot.last_sdo_index != ECU_CANOPEN_OBJ_DIGITAL_INPUT_STATES ||
        snapshot.last_sdo_subindex != 0U ||
        snapshot.last_sdo_abort_code != 0U) {
        return false;
    }

    *input_states = (uint16_t)snapshot.last_sdo_value;
    return true;
}
