#ifndef REMOTE_EVENT_LIFECYCLE_H
#define REMOTE_EVENT_LIFECYCLE_H

#include <stdbool.h>
#include <stdint.h>

#include "remote_types.h"

typedef struct {
    remote_event_type_t type;
    bool active;
    uint32_t sequence;
    uint32_t started_ms;
    uint32_t expires_at_ms;
} remote_event_lifecycle_t;

/* Initialize one remote event lease.
 *
 * Unit: all time arguments are milliseconds from the ECU monotonic clock.
 * Owner: the remote-manager task owns this object.
 * ISR: not safe; call only from task context.
 * Failure behavior: null pointers are ignored and leave caller state unchanged.
 */
void remote_event_lifecycle_init(remote_event_lifecycle_t *event,
                                 remote_event_type_t type);

/* Start or replace an event lease.
 *
 * A lease represents a fresh operator action such as a mode request or power
 * request. When the lease expires, the request must be dropped and not queued
 * for later execution.
 */
void remote_event_lifecycle_start(remote_event_lifecycle_t *event,
                                  uint32_t now_ms,
                                  uint32_t ttl_ms);

/* Return true while the event lease is active and not expired. */
bool remote_event_lifecycle_is_active(const remote_event_lifecycle_t *event,
                                      uint32_t now_ms);

/* Consume a live event once. Expired or inactive events return false. */
bool remote_event_lifecycle_consume(remote_event_lifecycle_t *event,
                                    uint32_t now_ms);

/* Expire an event immediately after domain sync, reconnect, or safety reset. */
void remote_event_lifecycle_cancel(remote_event_lifecycle_t *event);

#endif /* REMOTE_EVENT_LIFECYCLE_H */
