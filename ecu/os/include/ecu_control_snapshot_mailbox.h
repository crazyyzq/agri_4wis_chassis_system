#ifndef ECU_CONTROL_SNAPSHOT_MAILBOX_H
#define ECU_CONTROL_SNAPSHOT_MAILBOX_H

#include <stdbool.h>
#include <stdint.h>

#include "remote_types.h"
#include "vehicle_types.h"

/* Single-writer, multi-reader double buffers for CPU0 control handoffs.
 * Writers publish a complete inactive slot with release ordering. Readers copy
 * the selected small snapshot in a bounded task critical section because the
 * 1 ms safety publisher may otherwise preempt every lower-priority retry.
 * No ISR writes these mailboxes.
 */
typedef struct {
    volatile uint32_t publish_sequence;
    uint32_t timestamp_ms[2];
    remote_control_request_t value[2];
} remote_request_mailbox_t;

typedef struct {
    volatile uint32_t publish_sequence;
    uint32_t timestamp_ms[2];
    vehicle_safety_snapshot_t value[2];
} safety_snapshot_mailbox_t;

void remote_request_mailbox_publish(remote_request_mailbox_t *mailbox,
                                    const remote_control_request_t *request,
                                    uint32_t now_ms);
bool remote_request_mailbox_read(const remote_request_mailbox_t *mailbox,
                                 remote_control_request_t *request,
                                 uint32_t *timestamp_ms);
void safety_snapshot_mailbox_publish(safety_snapshot_mailbox_t *mailbox,
                                     const vehicle_safety_snapshot_t *snapshot,
                                     uint32_t now_ms);
bool safety_snapshot_mailbox_read(const safety_snapshot_mailbox_t *mailbox,
                                  vehicle_safety_snapshot_t *snapshot,
                                  uint32_t *timestamp_ms);

#endif /* ECU_CONTROL_SNAPSHOT_MAILBOX_H */
