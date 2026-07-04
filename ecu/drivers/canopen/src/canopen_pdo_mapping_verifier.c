#include <string.h>

#include "canopen_pdo_mapping_verifier.h"
#include "ecu_config.h"

#define CANOPEN_PDO_MAPPING_VERIFY_REQUEST_GUARD_MS (10U)
#define CANOPEN_PDO_MAPPING_VERIFY_TIMEOUT_MS (200U)

typedef struct {
    uint16_t index;
    uint8_t subindex;
    uint32_t value;
    uint8_t size;
} mapping_template_item_t;

static const mapping_template_item_t k_mapping_template[] = {
    { ECU_CANOPEN_OBJ_RPDO1_COMM_PARAM, ECU_CANOPEN_OBJ_PDO_COB_ID_SUBINDEX, 0U, 4U },
    { ECU_CANOPEN_OBJ_RPDO1_COMM_PARAM, ECU_CANOPEN_OBJ_PDO_TRANSMISSION_TYPE_SUBINDEX,
      ECU_CANOPEN_RPDO_TRANSMISSION_ASYNC, 1U },
    { ECU_CANOPEN_OBJ_RPDO1_MAPPING, ECU_CANOPEN_OBJ_PDO_MAPPING_COUNT_SUBINDEX, 2U, 1U },
    { ECU_CANOPEN_OBJ_RPDO1_MAPPING, ECU_CANOPEN_OBJ_PDO_MAPPING_FIRST_SUBINDEX,
      ECU_CANOPEN_PDO_MAP_CONTROLWORD_16, 4U },
    { ECU_CANOPEN_OBJ_RPDO1_MAPPING, ECU_CANOPEN_OBJ_PDO_MAPPING_SECOND_SUBINDEX,
      ECU_CANOPEN_PDO_MAP_MODE_OF_OPERATION_8, 4U },
    { ECU_CANOPEN_OBJ_RPDO1_MAPPING, ECU_CANOPEN_OBJ_PDO_MAPPING_THIRD_SUBINDEX,
      ECU_CANOPEN_PDO_MAP_TARGET_VELOCITY_32, 4U },
    { ECU_CANOPEN_OBJ_RPDO2_COMM_PARAM, ECU_CANOPEN_OBJ_PDO_COB_ID_SUBINDEX, 0U, 4U },
    { ECU_CANOPEN_OBJ_RPDO2_COMM_PARAM, ECU_CANOPEN_OBJ_PDO_TRANSMISSION_TYPE_SUBINDEX,
      ECU_CANOPEN_RPDO_TRANSMISSION_ASYNC, 1U },
    { ECU_CANOPEN_OBJ_RPDO2_MAPPING, ECU_CANOPEN_OBJ_PDO_MAPPING_COUNT_SUBINDEX, 2U, 1U },
    { ECU_CANOPEN_OBJ_RPDO2_MAPPING, ECU_CANOPEN_OBJ_PDO_MAPPING_FIRST_SUBINDEX,
      ECU_CANOPEN_PDO_MAP_CONTROLWORD_16, 4U },
    { ECU_CANOPEN_OBJ_RPDO2_MAPPING, ECU_CANOPEN_OBJ_PDO_MAPPING_SECOND_SUBINDEX,
      ECU_CANOPEN_PDO_MAP_MODE_OF_OPERATION_8, 4U },
    { ECU_CANOPEN_OBJ_RPDO2_MAPPING, ECU_CANOPEN_OBJ_PDO_MAPPING_THIRD_SUBINDEX,
      ECU_CANOPEN_PDO_MAP_TARGET_POSITION_32, 4U },
    { ECU_CANOPEN_OBJ_TPDO1_COMM_PARAM, ECU_CANOPEN_OBJ_PDO_COB_ID_SUBINDEX, 0U, 4U },
    { ECU_CANOPEN_OBJ_TPDO1_COMM_PARAM, ECU_CANOPEN_OBJ_PDO_TRANSMISSION_TYPE_SUBINDEX,
      ECU_CANOPEN_TPDO_TRANSMISSION_SYNC1, 1U },
    { ECU_CANOPEN_OBJ_TPDO1_MAPPING, ECU_CANOPEN_OBJ_PDO_MAPPING_COUNT_SUBINDEX, 2U, 1U },
    { ECU_CANOPEN_OBJ_TPDO1_MAPPING, ECU_CANOPEN_OBJ_PDO_MAPPING_FIRST_SUBINDEX,
      ECU_CANOPEN_PDO_MAP_ACTUAL_POSITION_32, 4U },
    { ECU_CANOPEN_OBJ_TPDO1_MAPPING, ECU_CANOPEN_OBJ_PDO_MAPPING_SECOND_SUBINDEX,
      ECU_CANOPEN_PDO_MAP_ACTUAL_VELOCITY_32, 4U },
    { ECU_CANOPEN_OBJ_TPDO2_COMM_PARAM, ECU_CANOPEN_OBJ_PDO_COB_ID_SUBINDEX, 0U, 4U },
    { ECU_CANOPEN_OBJ_TPDO2_COMM_PARAM, ECU_CANOPEN_OBJ_PDO_TRANSMISSION_TYPE_SUBINDEX,
      ECU_CANOPEN_TPDO_TRANSMISSION_SYNC1, 1U },
    { ECU_CANOPEN_OBJ_TPDO2_MAPPING, ECU_CANOPEN_OBJ_PDO_MAPPING_COUNT_SUBINDEX, 3U, 1U },
    { ECU_CANOPEN_OBJ_TPDO2_MAPPING, ECU_CANOPEN_OBJ_PDO_MAPPING_FIRST_SUBINDEX,
      ECU_CANOPEN_PDO_MAP_FAULT_LATCHED_32, 4U },
    { ECU_CANOPEN_OBJ_TPDO2_MAPPING, ECU_CANOPEN_OBJ_PDO_MAPPING_SECOND_SUBINDEX,
      ECU_CANOPEN_PDO_MAP_STATUSWORD_16, 4U },
    { ECU_CANOPEN_OBJ_TPDO2_MAPPING, ECU_CANOPEN_OBJ_PDO_MAPPING_THIRD_SUBINDEX,
      ECU_CANOPEN_PDO_MAP_ACTUAL_CURRENT_16, 4U },
};

static uint32_t expected_value_for_profile(const canopen_node_pdo_profile_t *profile,
                                           const mapping_template_item_t *item)
{
    if (item->index == ECU_CANOPEN_OBJ_RPDO1_COMM_PARAM &&
        item->subindex == ECU_CANOPEN_OBJ_PDO_COB_ID_SUBINDEX) {
        return profile->rpdo0_cob_id;
    }
    if (item->index == ECU_CANOPEN_OBJ_RPDO2_COMM_PARAM &&
        item->subindex == ECU_CANOPEN_OBJ_PDO_COB_ID_SUBINDEX) {
        return profile->rpdo1_cob_id;
    }
    if (item->index == ECU_CANOPEN_OBJ_TPDO1_COMM_PARAM &&
        item->subindex == ECU_CANOPEN_OBJ_PDO_COB_ID_SUBINDEX) {
        return profile->tpdo0_cob_id;
    }
    if (item->index == ECU_CANOPEN_OBJ_TPDO2_COMM_PARAM &&
        item->subindex == ECU_CANOPEN_OBJ_PDO_COB_ID_SUBINDEX) {
        return profile->tpdo1_cob_id;
    }
    return item->value;
}

void canopen_pdo_mapping_verifier_init(canopen_pdo_mapping_verifier_t *verifier)
{
    if (verifier != 0) {
        memset(verifier, 0, sizeof(*verifier));
        verifier->state = CANOPEN_PDO_MAPPING_VERIFY_IDLE;
    }
}

bool canopen_pdo_mapping_verifier_start(
    canopen_pdo_mapping_verifier_t *verifier,
    const canopen_node_pdo_profile_t *profile,
    uint32_t now_ms)
{
    if (verifier == 0 || profile == 0) {
        return false;
    }

    memset(verifier, 0, sizeof(*verifier));
    verifier->profile = *profile;
    verifier->state = CANOPEN_PDO_MAPPING_VERIFY_PENDING;
    verifier->start_ms = now_ms;
    verifier->check_count = (uint8_t)(sizeof(k_mapping_template) /
                                      sizeof(k_mapping_template[0]));
    return true;
}

bool canopen_pdo_mapping_verifier_step(canopen_pdo_mapping_verifier_t *verifier,
                                       canopen_master_service_t *service,
                                       uint32_t now_ms)
{
    const mapping_template_item_t *item;
    const canopen_master_snapshot_t *snapshot;

    if (verifier == 0 || service == 0 ||
        verifier->state != CANOPEN_PDO_MAPPING_VERIFY_PENDING) {
        return false;
    }
    snapshot = &service->snapshot;
    if (verifier->request_pending) {
        bool matching_upload =
            snapshot->sdo_upload_count != verifier->observed_upload_count &&
            snapshot->last_sdo_node_id == verifier->active_check.node_id &&
            snapshot->last_sdo_index == verifier->active_check.expected_index &&
            snapshot->last_sdo_subindex ==
                verifier->active_check.expected_subindex;
        bool matching_abort =
            snapshot->sdo_abort_count != verifier->observed_abort_count &&
            snapshot->last_sdo_abort_code != 0U &&
            snapshot->last_sdo_node_id == verifier->active_check.node_id &&
            snapshot->last_sdo_index == verifier->active_check.expected_index &&
            snapshot->last_sdo_subindex ==
                verifier->active_check.expected_subindex;

        if (matching_upload) {
            if (snapshot->last_sdo_size != verifier->active_check.expected_size ||
                snapshot->last_sdo_value != verifier->active_check.expected_value) {
                verifier->mismatch_count++;
            }
            verifier->request_pending = false;
            verifier->check_index++;
        } else if (matching_abort) {
            verifier->state = CANOPEN_PDO_MAPPING_VERIFY_ABORT;
            return false;
        } else if ((uint32_t)(now_ms - verifier->last_request_ms) >=
                   CANOPEN_PDO_MAPPING_VERIFY_TIMEOUT_MS) {
            verifier->state = CANOPEN_PDO_MAPPING_VERIFY_TIMEOUT;
            return false;
        } else {
            return true;
        }
    }
    if (verifier->check_index >= verifier->check_count) {
        verifier->state = verifier->mismatch_count == 0U ?
                          CANOPEN_PDO_MAPPING_VERIFY_MATCHED :
                          CANOPEN_PDO_MAPPING_VERIFY_MISMATCH;
        return true;
    }
    if ((uint32_t)(now_ms - verifier->last_request_ms) <
        CANOPEN_PDO_MAPPING_VERIFY_REQUEST_GUARD_MS) {
        return true;
    }

    item = &k_mapping_template[verifier->check_index];
    verifier->active_check.node_id = verifier->profile.node_id;
    verifier->active_check.expected_index = item->index;
    verifier->active_check.expected_subindex = item->subindex;
    verifier->active_check.expected_value =
        expected_value_for_profile(&verifier->profile, item);
    verifier->active_check.expected_size = item->size;

    /* Phase A is read-only.  A later service update must associate this
     * request_sequence with the exact SDO upload result before advancing the
     * verifier to MATCHED/MISMATCH.
     */
    if (!canopen_master_service_request_sdo_read(service,
                                                 verifier->profile.node_id,
                                                 item->index,
                                                 item->subindex)) {
        return false;
    }

    verifier->request_sequence++;
    verifier->observed_upload_count = snapshot->sdo_upload_count;
    verifier->observed_abort_count = snapshot->sdo_abort_count;
    verifier->last_request_ms = now_ms;
    verifier->request_pending = true;
    return true;
}
