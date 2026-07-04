#include <string.h>

#include "canopen_pdo_profile.h"

static bool node_id_is_valid(uint8_t node_id)
{
    return node_id != 0U && node_id <= CANOPEN_PDO_STANDARD_MAX_NODE_ID;
}

static int8_t mode_for_role(canopen_axis_role_t role)
{
    switch (role) {
    case CANOPEN_AXIS_ROLE_DRIVE_VELOCITY:
    case CANOPEN_AXIS_ROLE_HYDRAULIC_VELOCITY:
        return CANOPEN_PDO_MODE_PROFILE_VELOCITY;
    case CANOPEN_AXIS_ROLE_STEER_POSITION:
    case CANOPEN_AXIS_ROLE_LIFT_POSITION:
        return CANOPEN_PDO_MODE_PROFILE_POSITION;
    default:
        return 0;
    }
}

static void write_le_u16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8U) & 0xFFU);
}

static void write_le_i32(uint8_t *data, int32_t value)
{
    uint32_t raw = (uint32_t)value;
    data[0] = (uint8_t)(raw & 0xFFU);
    data[1] = (uint8_t)((raw >> 8U) & 0xFFU);
    data[2] = (uint8_t)((raw >> 16U) & 0xFFU);
    data[3] = (uint8_t)((raw >> 24U) & 0xFFU);
}

bool canopen_pdo_profile_init(uint8_t node_id,
                              canopen_axis_role_t role,
                              canopen_node_pdo_profile_t *profile)
{
    int8_t required_mode;

    if (profile == 0 || !node_id_is_valid(node_id)) {
        return false;
    }

    required_mode = mode_for_role(role);
    if (required_mode == 0) {
        return false;
    }

    memset(profile, 0, sizeof(*profile));
    profile->node_id = node_id;
    profile->role = role;
    profile->rpdo0_cob_id = (uint16_t)(CANOPEN_PDO_STANDARD_RPDO0_BASE + node_id);
    profile->rpdo1_cob_id = (uint16_t)(CANOPEN_PDO_STANDARD_RPDO1_BASE + node_id);
    profile->tpdo0_cob_id = (uint16_t)(CANOPEN_PDO_STANDARD_TPDO0_BASE + node_id);
    profile->tpdo1_cob_id = (uint16_t)(CANOPEN_PDO_STANDARD_TPDO1_BASE + node_id);
    profile->required_mode = required_mode;
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
