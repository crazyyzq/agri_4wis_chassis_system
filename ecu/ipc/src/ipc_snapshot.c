#include <string.h>

#include "ecu_time.h"
#include "ipc_snapshot.h"

static uint32_t crc32_lightweight(const uint8_t *data, size_t length)
{
    uint32_t crc = 0xA5A5A5A5UL;
    for (size_t i = 0U; i < length; ++i) {
        crc ^= data[i];
        crc = (crc << 5) | (crc >> 27);
    }
    return crc;
}

void ipc_snapshot_init(ipc_snapshot_t *snapshot)
{
    if (snapshot != 0) {
        memset(snapshot, 0, sizeof(*snapshot));
    }
}

bool ipc_snapshot_publish(ipc_snapshot_t *snapshot,
                          const void *payload,
                          size_t length,
                          uint32_t timestamp_ms)
{
    if (snapshot == 0 || payload == 0 || length > sizeof(snapshot->payload)) {
        return false;
    }
    memcpy(snapshot->payload, payload, length);
    snapshot->length = length;
    snapshot->timestamp_ms = timestamp_ms;
    snapshot->sequence++;
    snapshot->crc32 = crc32_lightweight(snapshot->payload, length);
    snapshot->valid = true;
    return true;
}

bool ipc_snapshot_copy_valid(const ipc_snapshot_t *snapshot,
                             void *payload,
                             size_t capacity,
                             size_t *length_out,
                             uint32_t now_ms,
                             uint32_t timeout_ms)
{
    if (snapshot == 0 || payload == 0 || length_out == 0 ||
        !snapshot->valid || snapshot->length > capacity) {
        return false;
    }
    if (ecu_time_elapsed(now_ms, snapshot->timestamp_ms, timeout_ms)) {
        return false;
    }
    if (crc32_lightweight(snapshot->payload, snapshot->length) != snapshot->crc32) {
        return false;
    }
    memcpy(payload, snapshot->payload, snapshot->length);
    *length_out = snapshot->length;
    return true;
}
