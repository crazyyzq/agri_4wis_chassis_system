#ifndef IPC_SNAPSHOT_H
#define IPC_SNAPSHOT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint32_t sequence;
    uint32_t timestamp_ms;
    uint32_t crc32;
    bool valid;
    uint8_t payload[128];
    size_t length;
} ipc_snapshot_t;

void ipc_snapshot_init(ipc_snapshot_t *snapshot);
bool ipc_snapshot_publish(ipc_snapshot_t *snapshot,
                          const void *payload,
                          size_t length,
                          uint32_t timestamp_ms);
bool ipc_snapshot_copy_valid(const ipc_snapshot_t *snapshot,
                             void *payload,
                             size_t capacity,
                             size_t *length_out,
                             uint32_t now_ms,
                             uint32_t timeout_ms);

#endif /* IPC_SNAPSHOT_H */
