#include "servo_drive_canopen.h"

static bool servo_drive_node_valid(const ecu_canopen_node_config_t *node)
{
    return node != 0 && node->node_id != 0U;
}

static bool servo_drive_write_sdo_sub(canopen_master_service_t *canopen,
                                      const ecu_canopen_node_config_t *node,
                                      uint16_t index,
                                      uint8_t subindex,
                                      uint8_t size,
                                      int32_t value)
{
    if (canopen == 0 || !servo_drive_node_valid(node)) {
        return false;
    }

    return canopen_master_service_request_sdo_write(canopen,
                                                    node->node_id,
                                                    index,
                                                    subindex,
                                                    size,
                                                    value);
}

static bool servo_drive_write_sdo(canopen_master_service_t *canopen,
                                  const ecu_canopen_node_config_t *node,
                                  uint16_t index,
                                  uint8_t size,
                                  int32_t value)
{
    return servo_drive_write_sdo_sub(canopen, node, index, 0U, size, value);
}

static bool servo_drive_enter_operational(canopen_master_service_t *canopen,
                                          const ecu_canopen_node_config_t *node)
{
    if (canopen == 0 || !servo_drive_node_valid(node)) {
        return false;
    }

    return canopen_master_service_request_nmt(
        canopen,
        node->node_id,
        CANOPEN_MASTER_DEBUG_COMMAND_NMT_OPERATIONAL);
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

bool servo_drive_canopen_prepare_velocity_mode(canopen_master_service_t *canopen,
                                               const ecu_canopen_node_config_t *node)
{
    return servo_drive_enter_operational(canopen, node) &&
           servo_drive_write_sdo(canopen,
                                 node,
                                 ECU_CANOPEN_OBJ_MODES_OF_OPERATION,
                                 1U,
                                 (int32_t)SERVO_DRIVE_MODE_PROFILE_VELOCITY) &&
           servo_drive_canopen_send_control_word(
               canopen,
               node,
               SERVO_DRIVE_CONTROL_ENABLE_OPERATION);
}

bool servo_drive_canopen_run_velocity_mode(canopen_master_service_t *canopen,
                                           const ecu_canopen_node_config_t *node,
                                           int32_t target_velocity_units)
{
    return servo_drive_canopen_prepare_velocity_mode(canopen, node) &&
           servo_drive_canopen_update_target_velocity(canopen,
                                                     node,
                                                     target_velocity_units);
}

bool servo_drive_canopen_update_target_velocity(canopen_master_service_t *canopen,
                                                const ecu_canopen_node_config_t *node,
                                                int32_t target_velocity_units)
{
    return servo_drive_write_sdo(canopen,
                                 node,
                                 ECU_CANOPEN_OBJ_TARGET_VELOCITY,
                                 4U,
                                 target_velocity_units);
}

bool servo_drive_canopen_stop_velocity_mode(canopen_master_service_t *canopen,
                                            const ecu_canopen_node_config_t *node)
{
    return servo_drive_write_sdo(canopen,
                                 node,
                                 ECU_CANOPEN_OBJ_TARGET_VELOCITY,
                                 4U,
                                 0) &&
           servo_drive_canopen_send_control_word(
               canopen,
               node,
               SERVO_DRIVE_CONTROL_DISABLE_VOLTAGE);
}

static bool servo_drive_canopen_run_position_mode(canopen_master_service_t *canopen,
                                                  const ecu_canopen_node_config_t *node,
                                                  int32_t profile_velocity_units,
                                                  int32_t target_position_counts,
                                                  uint16_t trigger_control_word)
{
    return servo_drive_canopen_prepare_position_mode(canopen,
                                                    node,
                                                    profile_velocity_units) &&
           servo_drive_write_sdo(canopen,
                                 node,
                                 ECU_CANOPEN_OBJ_TARGET_POSITION,
                                 4U,
                                 target_position_counts) &&
           servo_drive_canopen_send_control_word(
               canopen,
               node,
               SERVO_DRIVE_CONTROL_ENABLE_OPERATION) &&
           servo_drive_canopen_send_control_word(canopen,
                                                node,
                                                trigger_control_word);
}

bool servo_drive_canopen_prepare_position_mode(canopen_master_service_t *canopen,
                                               const ecu_canopen_node_config_t *node,
                                               int32_t profile_velocity_units)
{
    return servo_drive_enter_operational(canopen, node) &&
           servo_drive_write_sdo(canopen,
                                 node,
                                 ECU_CANOPEN_OBJ_MODES_OF_OPERATION,
                                 1U,
                                 (int32_t)SERVO_DRIVE_MODE_PROFILE_POSITION) &&
           servo_drive_write_sdo(canopen,
                                 node,
                                 ECU_CANOPEN_OBJ_PROFILE_VELOCITY,
                                 4U,
                                 profile_velocity_units) &&
           servo_drive_canopen_send_control_word(
               canopen,
               node,
               SERVO_DRIVE_CONTROL_ENABLE_OPERATION);
}

bool servo_drive_canopen_configure_steer_rpdo(canopen_master_service_t *canopen,
                                              const ecu_canopen_node_config_t *node)
{
    if (canopen == 0 || !servo_drive_node_valid(node) ||
        node->rpdo1_cob_id == 0UL ||
        (node->rpdo1_cob_id & ECU_CANOPEN_COB_ID_DISABLED) != 0UL) {
        return false;
    }

    uint32_t disabled_cob_id = ECU_CANOPEN_COB_ID_DISABLED | node->rpdo1_cob_id;

    return servo_drive_write_sdo_sub(canopen,
                                     node,
                                     ECU_CANOPEN_OBJ_RPDO1_COMM_PARAM,
                                     ECU_CANOPEN_OBJ_PDO_COB_ID_SUBINDEX,
                                     4U,
                                     (int32_t)disabled_cob_id) &&
           servo_drive_write_sdo_sub(canopen,
                                     node,
                                     ECU_CANOPEN_OBJ_RPDO1_MAPPING,
                                     ECU_CANOPEN_OBJ_PDO_MAPPING_COUNT_SUBINDEX,
                                     1U,
                                     0) &&
           servo_drive_write_sdo_sub(canopen,
                                     node,
                                     ECU_CANOPEN_OBJ_RPDO1_MAPPING,
                                     ECU_CANOPEN_OBJ_PDO_MAPPING_FIRST_SUBINDEX,
                                     4U,
                                     (int32_t)ECU_CANOPEN_PDO_MAP_CONTROLWORD_16) &&
           servo_drive_write_sdo_sub(canopen,
                                     node,
                                     ECU_CANOPEN_OBJ_RPDO1_MAPPING,
                                     ECU_CANOPEN_OBJ_PDO_MAPPING_SECOND_SUBINDEX,
                                     4U,
                                     (int32_t)ECU_CANOPEN_PDO_MAP_TARGET_POSITION_32) &&
           servo_drive_write_sdo_sub(canopen,
                                     node,
                                     ECU_CANOPEN_OBJ_RPDO1_MAPPING,
                                     ECU_CANOPEN_OBJ_PDO_MAPPING_COUNT_SUBINDEX,
                                     1U,
                                     2) &&
           servo_drive_write_sdo_sub(canopen,
                                     node,
                                     ECU_CANOPEN_OBJ_RPDO1_COMM_PARAM,
                                     ECU_CANOPEN_OBJ_PDO_TRANSMISSION_TYPE_SUBINDEX,
                                     1U,
                                     ECU_CANOPEN_RPDO_TRANSMISSION_ASYNC) &&
           servo_drive_write_sdo_sub(canopen,
                                     node,
                                     ECU_CANOPEN_OBJ_RPDO1_COMM_PARAM,
                                     ECU_CANOPEN_OBJ_PDO_COB_ID_SUBINDEX,
                                     4U,
                                     (int32_t)node->rpdo1_cob_id);
}

bool servo_drive_canopen_run_absolute_position_mode(canopen_master_service_t *canopen,
                                                    const ecu_canopen_node_config_t *node,
                                                    int32_t profile_velocity_units,
                                                    int32_t target_position_counts)
{
    return servo_drive_canopen_run_position_mode(
        canopen,
        node,
        profile_velocity_units,
        target_position_counts,
        SERVO_DRIVE_CONTROL_TRIGGER_ABSOLUTE_POSITION);
}

bool servo_drive_canopen_update_absolute_position(canopen_master_service_t *canopen,
                                                  const ecu_canopen_node_config_t *node,
                                                  int32_t target_position_counts)
{
    return servo_drive_write_sdo(canopen,
                                 node,
                                 ECU_CANOPEN_OBJ_TARGET_POSITION,
                                 4U,
                                 target_position_counts) &&
           servo_drive_canopen_send_control_word(
               canopen,
               node,
               SERVO_DRIVE_CONTROL_ENABLE_OPERATION) &&
           servo_drive_canopen_send_control_word(
               canopen,
               node,
               SERVO_DRIVE_CONTROL_TRIGGER_ABSOLUTE_POSITION);
}

bool servo_drive_canopen_run_current_mode(canopen_master_service_t *canopen,
                                          const ecu_canopen_node_config_t *node,
                                          int32_t current_ramp_ma_per_sec,
                                          int16_t current_10ma)
{
    return servo_drive_enter_operational(canopen, node) &&
           servo_drive_write_sdo(canopen,
                                 node,
                                 ECU_CANOPEN_OBJ_MODES_OF_OPERATION,
                                 1U,
                                 (int32_t)SERVO_DRIVE_MODE_PROFILE_TORQUE) &&
           servo_drive_write_sdo(canopen,
                                 node,
                                 ECU_CANOPEN_OBJ_COMMAND_CURRENT_RAMP,
                                 4U,
                                 current_ramp_ma_per_sec) &&
           servo_drive_canopen_send_control_word(
               canopen,
               node,
               SERVO_DRIVE_CONTROL_ENABLE_OPERATION) &&
           servo_drive_write_sdo(canopen,
                                 node,
                                 ECU_CANOPEN_OBJ_COMMAND_CURRENT,
                                 2U,
                                 (int32_t)current_10ma);
}

bool servo_drive_canopen_stop_current_mode(canopen_master_service_t *canopen,
                                           const ecu_canopen_node_config_t *node)
{
    return servo_drive_write_sdo(canopen,
                                 node,
                                 ECU_CANOPEN_OBJ_COMMAND_CURRENT,
                                 2U,
                                 0) &&
           servo_drive_canopen_send_control_word(
               canopen,
               node,
               SERVO_DRIVE_CONTROL_DISABLE_VOLTAGE);
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
                                 ECU_CANOPEN_OBJ_COMMAND_CURRENT,
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
