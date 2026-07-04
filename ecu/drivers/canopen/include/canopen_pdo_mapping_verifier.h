#ifndef CANOPEN_PDO_MAPPING_VERIFIER_H
#define CANOPEN_PDO_MAPPING_VERIFIER_H

#include <stdbool.h>
#include <stdint.h>

#include "canopen_master_service.h"
#include "canopen_pdo_profile.h"

typedef enum {
    CANOPEN_PDO_MAPPING_VERIFY_IDLE = 0,
    CANOPEN_PDO_MAPPING_VERIFY_PENDING,
    CANOPEN_PDO_MAPPING_VERIFY_MATCHED,
    CANOPEN_PDO_MAPPING_VERIFY_MISMATCH,
    CANOPEN_PDO_MAPPING_VERIFY_TIMEOUT,
    CANOPEN_PDO_MAPPING_VERIFY_ABORT
} canopen_pdo_mapping_verify_state_t;

typedef struct {
    uint8_t node_id;
    uint16_t expected_index;
    uint8_t expected_subindex;
    uint32_t expected_value;
    uint8_t expected_size;
} canopen_pdo_mapping_check_t;

typedef struct {
    canopen_node_pdo_profile_t profile;
    canopen_pdo_mapping_verify_state_t state;
    uint32_t request_sequence;
    uint32_t start_ms;
    uint32_t last_request_ms;
    uint32_t observed_upload_count;
    uint32_t observed_abort_count;
    uint8_t check_index;
    uint8_t check_count;
    uint8_t mismatch_count;
    bool request_pending;
    canopen_pdo_mapping_check_t active_check;
} canopen_pdo_mapping_verifier_t;

void canopen_pdo_mapping_verifier_init(canopen_pdo_mapping_verifier_t *verifier);
bool canopen_pdo_mapping_verifier_start(
    canopen_pdo_mapping_verifier_t *verifier,
    const canopen_node_pdo_profile_t *profile,
    uint32_t now_ms);
bool canopen_pdo_mapping_verifier_step(canopen_pdo_mapping_verifier_t *verifier,
                                       canopen_master_service_t *service,
                                       uint32_t now_ms);

#endif /* CANOPEN_PDO_MAPPING_VERIFIER_H */
