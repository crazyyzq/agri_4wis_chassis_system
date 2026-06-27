#include <string.h>

#include "canopen_frame.h"

bool canopen_frame_build_pdo(uint32_t cob_id,
                             const uint8_t *payload,
                             uint8_t payload_size,
                             canopen_frame_t *out)
{
    if (out == 0 || payload == 0 || payload_size > CANOPEN_FRAME_MAX_DATA_BYTES) {
        return false;
    }
    out->cob_id = cob_id;
    out->size = payload_size;
    memset(out->data, 0, sizeof(out->data));
    memcpy(out->data, payload, payload_size);
    return true;
}

bool canopen_frame_build_nmt(uint8_t command,
                             uint8_t node_id,
                             canopen_frame_t *out)
{
    if (out == 0) {
        return false;
    }
    out->cob_id = 0U;
    out->size = 2U;
    memset(out->data, 0, sizeof(out->data));
    out->data[0] = command;
    out->data[1] = node_id;
    return true;
}
