#ifndef CANOPEN_PDO_PROFILE_H
#define CANOPEN_PDO_PROFILE_H

#include <stdbool.h>
#include <stdint.h>

#include "canopen_master_service.h"
#include "ecu_config.h"

#define CANOPEN_PDO_STANDARD_RPDO0_BASE ((uint16_t)ECU_CANOPEN_RPDO1_BASE)
#define CANOPEN_PDO_STANDARD_RPDO1_BASE ((uint16_t)ECU_CANOPEN_RPDO2_BASE)
#define CANOPEN_PDO_STANDARD_TPDO0_BASE ((uint16_t)ECU_CANOPEN_TPDO1_BASE)
#define CANOPEN_PDO_STANDARD_TPDO1_BASE ((uint16_t)ECU_CANOPEN_TPDO2_BASE)
#define CANOPEN_PDO_STANDARD_MAX_NODE_ID (127U)

#define CANOPEN_PDO_MODE_PROFILE_POSITION (1)
#define CANOPEN_PDO_MODE_PROFILE_VELOCITY (3)

typedef enum {
    CANOPEN_AXIS_ROLE_DRIVE_VELOCITY = 0,
    CANOPEN_AXIS_ROLE_STEER_POSITION,
    CANOPEN_AXIS_ROLE_LIFT_POSITION,
    CANOPEN_AXIS_ROLE_HYDRAULIC_VELOCITY
} canopen_axis_role_t;

typedef struct {
    uint8_t node_id;
    canopen_axis_role_t role;
    uint16_t rpdo0_cob_id; /* DS301 RPDO0 command COB-ID. */
    uint16_t rpdo1_cob_id; /* DS301 RPDO1 command COB-ID. */
    uint16_t tpdo0_cob_id; /* DS301 TPDO0 feedback COB-ID. */
    uint16_t tpdo1_cob_id; /* DS301 TPDO1 feedback COB-ID. */
    int8_t required_mode;  /* 0x03 velocity or 0x01 position. */
} canopen_node_pdo_profile_t;

typedef struct {
    canopen_master_bus_t bus;
    uint8_t node_id;
    uint8_t leg_index;
    canopen_axis_role_t role;
    uint16_t rpdo0_cob_id;
    uint16_t rpdo1_cob_id;
    uint16_t tpdo0_cob_id;
    uint16_t tpdo1_cob_id;
    int8_t required_mode;
    const char *name;
} canopen_node_pdo_contract_t;

#define CANOPEN_PDO_CONTRACT_PUMP_LEG_INDEX (0xFFU)
#define CANOPEN_PDO_CONTRACT_NODE_COUNT (13U)

/* Existing ecu_canopen_node_config_t fields keep their historical names:
 *   node->rpdo1_cob_id == DS301 RPDO0 == 0x200 + node
 *   node->rpdo2_cob_id == DS301 RPDO1 == 0x300 + node
 * PDO command and feedback code should use this frozen contract table instead
 * of relying on legacy field names at call sites.
 */
const canopen_node_pdo_contract_t *canopen_pdo_contract_find(
    canopen_master_bus_t bus,
    uint8_t node_id);

const canopen_node_pdo_contract_t *canopen_pdo_contract_for_drive_leg(uint32_t leg);
const canopen_node_pdo_contract_t *canopen_pdo_contract_for_steer_leg(uint32_t leg);
const canopen_node_pdo_contract_t *canopen_pdo_contract_for_lift_leg(uint32_t leg);
const canopen_node_pdo_contract_t *canopen_pdo_contract_for_hydraulic_pump(void);
const canopen_node_pdo_contract_t *canopen_pdo_contract_table(uint32_t *count);

bool canopen_pdo_profile_init(uint8_t node_id,
                              canopen_axis_role_t role,
                              canopen_node_pdo_profile_t *profile);

bool canopen_pdo_build_velocity_rpdo0(
    const canopen_node_pdo_profile_t *profile,
    uint16_t controlword,
    int32_t target_velocity,
    canopen_master_pdo_request_t *request,
    uint32_t group_sequence,
    canopen_master_pdo_phase_t phase);

bool canopen_pdo_build_position_rpdo1(
    const canopen_node_pdo_profile_t *profile,
    uint16_t controlword,
    int32_t target_position,
    canopen_master_pdo_request_t *request,
    uint32_t group_sequence,
    canopen_master_pdo_phase_t phase);

#endif /* CANOPEN_PDO_PROFILE_H */
