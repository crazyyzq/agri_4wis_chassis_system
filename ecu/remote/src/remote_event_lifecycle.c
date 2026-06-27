#include "remote_event_lifecycle.h"

static bool time_reached(uint32_t now_ms, uint32_t deadline_ms)
{
    return (int32_t)(now_ms - deadline_ms) >= 0;
}

void remote_event_lifecycle_init(remote_event_lifecycle_t *event,
                                 remote_event_type_t type)
{
    if (event == 0) {
        return;
    }
    event->type = type;
    event->active = false;
    event->sequence = 0U;
    event->started_ms = 0U;
    event->expires_at_ms = 0U;
}

void remote_event_lifecycle_start(remote_event_lifecycle_t *event,
                                  uint32_t now_ms,
                                  uint32_t ttl_ms)
{
    if (event == 0) {
        return;
    }
    event->active = true;
    event->sequence++;
    event->started_ms = now_ms;
    event->expires_at_ms = now_ms + ttl_ms;
}

bool remote_event_lifecycle_is_active(const remote_event_lifecycle_t *event,
                                      uint32_t now_ms)
{
    if (event == 0 || !event->active) {
        return false;
    }
    return !time_reached(now_ms, event->expires_at_ms);
}

bool remote_event_lifecycle_consume(remote_event_lifecycle_t *event,
                                    uint32_t now_ms)
{
    if (!remote_event_lifecycle_is_active(event, now_ms)) {
        return false;
    }
    event->active = false;
    return true;
}

void remote_event_lifecycle_cancel(remote_event_lifecycle_t *event)
{
    if (event != 0) {
        event->active = false;
    }
}
