#include <string.h>

#include "canopen_pdo_profile.h"

#define CONTRACT_MODE_FOR_ROLE(role_) \
    (((role_) == CANOPEN_AXIS_ROLE_DRIVE_VELOCITY || \
      (role_) == CANOPEN_AXIS_ROLE_HYDRAULIC_VELOCITY) ? \
         CANOPEN_PDO_MODE_PROFILE_VELOCITY : \
     ((role_) == CANOPEN_AXIS_ROLE_LIFT_POSITION) ? \
         CANOPEN_PDO_MODE_INTERPOLATED_POSITION : CANOPEN_PDO_MODE_PROFILE_POSITION)

#define CONTRACT_ENTRY(bus_, node_, leg_, role_, name_) \
    { \
        .bus = (bus_), \
        .node_id = (node_), \
        .leg_index = (leg_), \
        .role = (role_), \
        .rpdo0_cob_id = (uint16_t)(CANOPEN_PDO_STANDARD_RPDO0_BASE + (node_)), \
        .rpdo1_cob_id = (uint16_t)(CANOPEN_PDO_STANDARD_RPDO1_BASE + (node_)), \
        .rpdo2_cob_id = (uint16_t)(CANOPEN_PDO_STANDARD_RPDO2_BASE + (node_)), \
        .rpdo3_cob_id = (uint16_t)(CANOPEN_PDO_STANDARD_RPDO3_BASE + (node_)), \
        .tpdo0_cob_id = (uint16_t)(CANOPEN_PDO_STANDARD_TPDO0_BASE + (node_)), \
        .tpdo1_cob_id = (uint16_t)(CANOPEN_PDO_STANDARD_TPDO1_BASE + (node_)), \
        .required_mode = (int8_t)CONTRACT_MODE_FOR_ROLE(role_), \
        .name = (name_) \
    }

static const canopen_node_pdo_contract_t k_contract_table[CANOPEN_PDO_CONTRACT_NODE_COUNT] = {
    CONTRACT_ENTRY(CANOPEN_MASTER_BUS_CAN2, ECU_CANOPEN_DRIVE_FR_NODE_ID, 0U,
                   CANOPEN_AXIS_ROLE_DRIVE_VELOCITY, "can2_drive_fr"),
    CONTRACT_ENTRY(CANOPEN_MASTER_BUS_CAN2, ECU_CANOPEN_DRIVE_FL_NODE_ID, 1U,
                   CANOPEN_AXIS_ROLE_DRIVE_VELOCITY, "can2_drive_fl"),
    CONTRACT_ENTRY(CANOPEN_MASTER_BUS_CAN2, ECU_CANOPEN_DRIVE_RL_NODE_ID, 2U,
                   CANOPEN_AXIS_ROLE_DRIVE_VELOCITY, "can2_drive_rl"),
    CONTRACT_ENTRY(CANOPEN_MASTER_BUS_CAN2, ECU_CANOPEN_DRIVE_RR_NODE_ID, 3U,
                   CANOPEN_AXIS_ROLE_DRIVE_VELOCITY, "can2_drive_rr"),
    CONTRACT_ENTRY(CANOPEN_MASTER_BUS_CAN2, ECU_CANOPEN_STEER_FR_NODE_ID, 0U,
                   CANOPEN_AXIS_ROLE_STEER_POSITION, "can2_steer_fr"),
    CONTRACT_ENTRY(CANOPEN_MASTER_BUS_CAN2, ECU_CANOPEN_STEER_FL_NODE_ID, 1U,
                   CANOPEN_AXIS_ROLE_STEER_POSITION, "can2_steer_fl"),
    CONTRACT_ENTRY(CANOPEN_MASTER_BUS_CAN2, ECU_CANOPEN_STEER_RL_NODE_ID, 2U,
                   CANOPEN_AXIS_ROLE_STEER_POSITION, "can2_steer_rl"),
    CONTRACT_ENTRY(CANOPEN_MASTER_BUS_CAN2, ECU_CANOPEN_STEER_RR_NODE_ID, 3U,
                   CANOPEN_AXIS_ROLE_STEER_POSITION, "can2_steer_rr"),
    CONTRACT_ENTRY(CANOPEN_MASTER_BUS_CAN3, ECU_CANOPEN_LIFT_FR_NODE_ID, 0U,
                   CANOPEN_AXIS_ROLE_LIFT_POSITION, "can3_lift_fr"),
    CONTRACT_ENTRY(CANOPEN_MASTER_BUS_CAN3, ECU_CANOPEN_LIFT_RR_NODE_ID, 3U,
                   CANOPEN_AXIS_ROLE_LIFT_POSITION, "can3_lift_rr"),
    CONTRACT_ENTRY(CANOPEN_MASTER_BUS_CAN3, ECU_CANOPEN_LIFT_FL_NODE_ID, 1U,
                   CANOPEN_AXIS_ROLE_LIFT_POSITION, "can3_lift_fl"),
    CONTRACT_ENTRY(CANOPEN_MASTER_BUS_CAN3, ECU_CANOPEN_LIFT_RL_NODE_ID, 2U,
                   CANOPEN_AXIS_ROLE_LIFT_POSITION, "can3_lift_rl"),
    CONTRACT_ENTRY(CANOPEN_MASTER_BUS_CAN3, ECU_CANOPEN_HYDRAULIC_PUMP_NODE_ID,
                   CANOPEN_PDO_CONTRACT_PUMP_LEG_INDEX,
                   CANOPEN_AXIS_ROLE_HYDRAULIC_VELOCITY, "can3_hydraulic_pump"),
};

static bool node_id_is_valid(uint8_t node_id)
{
    return node_id != 0U && node_id <= CANOPEN_PDO_STANDARD_MAX_NODE_ID;
}

static void write_le_u16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8U) & 0xFFU);
}

static void write_le_i16(uint8_t *data, int16_t value)
{
    uint16_t raw = (uint16_t)value;
    data[0] = (uint8_t)(raw & 0xFFU);
    data[1] = (uint8_t)((raw >> 8U) & 0xFFU);
}

static void write_le_i32(uint8_t *data, int32_t value)
{
    uint32_t raw = (uint32_t)value;
    data[0] = (uint8_t)(raw & 0xFFU);
    data[1] = (uint8_t)((raw >> 8U) & 0xFFU);
    data[2] = (uint8_t)((raw >> 16U) & 0xFFU);
    data[3] = (uint8_t)((raw >> 24U) & 0xFFU);
}

const canopen_node_pdo_contract_t *canopen_pdo_contract_find(
    canopen_master_bus_t bus,
    uint8_t node_id)
{
    for (uint32_t index = 0U; index < CANOPEN_PDO_CONTRACT_NODE_COUNT; ++index) {
        const canopen_node_pdo_contract_t *contract = &k_contract_table[index];
        if (contract->bus == bus && contract->node_id == node_id) {
            return contract;
        }
    }
    return 0;
}

static const canopen_node_pdo_contract_t *find_by_role_leg(canopen_master_bus_t bus,
                                                           canopen_axis_role_t role,
                                                           uint32_t leg)
{
    if (leg >= ECU_WHEEL_COUNT) {
        return 0;
    }
    for (uint32_t index = 0U; index < CANOPEN_PDO_CONTRACT_NODE_COUNT; ++index) {
        const canopen_node_pdo_contract_t *contract = &k_contract_table[index];
        if (contract->bus == bus &&
            contract->role == role &&
            contract->leg_index == (uint8_t)leg) {
            return contract;
        }
    }
    return 0;
}

const canopen_node_pdo_contract_t *canopen_pdo_contract_for_drive_leg(uint32_t leg)
{
    return find_by_role_leg(CANOPEN_MASTER_BUS_CAN2,
                            CANOPEN_AXIS_ROLE_DRIVE_VELOCITY,
                            leg);
}

const canopen_node_pdo_contract_t *canopen_pdo_contract_for_steer_leg(uint32_t leg)
{
    return find_by_role_leg(CANOPEN_MASTER_BUS_CAN2,
                            CANOPEN_AXIS_ROLE_STEER_POSITION,
                            leg);
}

const canopen_node_pdo_contract_t *canopen_pdo_contract_for_lift_leg(uint32_t leg)
{
    return find_by_role_leg(CANOPEN_MASTER_BUS_CAN3,
                            CANOPEN_AXIS_ROLE_LIFT_POSITION,
                            leg);
}

const canopen_node_pdo_contract_t *canopen_pdo_contract_for_hydraulic_pump(void)
{
    for (uint32_t index = 0U; index < CANOPEN_PDO_CONTRACT_NODE_COUNT; ++index) {
        const canopen_node_pdo_contract_t *contract = &k_contract_table[index];
        if (contract->role == CANOPEN_AXIS_ROLE_HYDRAULIC_VELOCITY) {
            return contract;
        }
    }
    return 0;
}

const canopen_node_pdo_contract_t *canopen_pdo_contract_table(uint32_t *count)
{
    if (count != 0) {
        *count = CANOPEN_PDO_CONTRACT_NODE_COUNT;
    }
    return k_contract_table;
}

bool canopen_pdo_profile_init(uint8_t node_id,
                              canopen_axis_role_t role,
                              canopen_node_pdo_profile_t *profile)
{
    const canopen_node_pdo_contract_t *contract = 0;

    if (profile == 0 || !node_id_is_valid(node_id)) {
        return false;
    }

    for (uint32_t index = 0U; index < CANOPEN_PDO_CONTRACT_NODE_COUNT; ++index) {
        if (k_contract_table[index].node_id == node_id &&
            k_contract_table[index].role == role) {
            contract = &k_contract_table[index];
            break;
        }
    }
    if (contract == 0) {
        return false;
    }

    memset(profile, 0, sizeof(*profile));
    profile->node_id = contract->node_id;
    profile->role = contract->role;
    profile->rpdo0_cob_id = contract->rpdo0_cob_id;
    profile->rpdo1_cob_id = contract->rpdo1_cob_id;
    profile->rpdo2_cob_id = contract->rpdo2_cob_id;
    profile->rpdo3_cob_id = contract->rpdo3_cob_id;
    profile->tpdo0_cob_id = contract->tpdo0_cob_id;
    profile->tpdo1_cob_id = contract->tpdo1_cob_id;
    profile->required_mode = contract->required_mode;
    return true;
}

bool canopen_pdo_build_velocity_rpdo0(
    const canopen_node_pdo_profile_t *profile,
    uint16_t controlword,
    int32_t target_velocity,
    canopen_master_pdo_request_t *request,
    uint32_t group_sequence,
    canopen_master_pdo_phase_t phase)
{
    if (profile == 0 || request == 0 ||
        profile->required_mode != CANOPEN_PDO_MODE_PROFILE_VELOCITY) {
        return false;
    }

    memset(request, 0, sizeof(*request));
    request->cob_id = profile->rpdo0_cob_id;
    request->size = 7U;
    request->node_id = profile->node_id;
    request->group_sequence = group_sequence;
    request->phase = phase;
    write_le_u16(&request->data[0], controlword);
    request->data[2] = (uint8_t)CANOPEN_PDO_MODE_PROFILE_VELOCITY;
    write_le_i32(&request->data[3], target_velocity);
    return true;
}

bool canopen_pdo_build_position_rpdo1(
    const canopen_node_pdo_profile_t *profile,
    uint16_t controlword,
    int32_t target_position,
    canopen_master_pdo_request_t *request,
    uint32_t group_sequence,
    canopen_master_pdo_phase_t phase)
{
    if (profile == 0 || request == 0 ||
        profile->required_mode != CANOPEN_PDO_MODE_PROFILE_POSITION) {
        return false;
    }

    memset(request, 0, sizeof(*request));
    request->cob_id = profile->rpdo1_cob_id;
    request->size = 7U;
    request->node_id = profile->node_id;
    request->group_sequence = group_sequence;
    request->phase = phase;
    write_le_u16(&request->data[0], controlword);
    request->data[2] = (uint8_t)CANOPEN_PDO_MODE_PROFILE_POSITION;
    write_le_i32(&request->data[3], target_position);
    return true;
}

bool canopen_pdo_build_interpolated_position_rpdo2(
    const canopen_node_pdo_profile_t *profile,
    int32_t interpolation_position,
    canopen_master_pdo_request_t *request,
    uint32_t group_sequence,
    canopen_master_pdo_phase_t phase)
{
    if (profile == 0 || request == 0 ||
        profile->required_mode != CANOPEN_PDO_MODE_INTERPOLATED_POSITION) {
        return false;
    }

    memset(request, 0, sizeof(*request));
    request->cob_id = profile->rpdo2_cob_id;
    request->size = 4U;
    request->node_id = profile->node_id;
    request->group_sequence = group_sequence;
    request->phase = phase;
    write_le_i32(&request->data[0], interpolation_position);
    return true;
}

bool canopen_pdo_build_current_rpdo3(
    const canopen_node_pdo_profile_t *profile,
    uint16_t controlword,
    int16_t command_current_10ma,
    canopen_master_pdo_request_t *request,
    uint32_t group_sequence,
    canopen_master_pdo_phase_t phase)
{
    if (profile == 0 || request == 0) {
        return false;
    }

    memset(request, 0, sizeof(*request));
    request->cob_id = profile->rpdo3_cob_id;
    request->size = 5U;
    request->node_id = profile->node_id;
    request->group_sequence = group_sequence;
    request->phase = phase;
    write_le_u16(&request->data[0], controlword);
    request->data[2] = (uint8_t)CANOPEN_PDO_MODE_CURRENT;
    write_le_i16(&request->data[3], command_current_10ma);
    return true;
}
