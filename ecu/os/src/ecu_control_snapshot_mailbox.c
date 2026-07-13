#include <stddef.h>

#include "ecu_control_snapshot_mailbox.h"

#define DEFINE_DOUBLE_BUFFER_MAILBOX(prefix, mailbox_type, value_type)                 \
    void prefix##_publish(mailbox_type *mailbox,                                      \
                          const value_type *value,                                    \
                          uint32_t now_ms)                                             \
    {                                                                                  \
        if (mailbox == NULL || value == NULL) {                                        \
            return;                                                                    \
        }                                                                              \
        uint32_t sequence = __atomic_load_n(&mailbox->publish_sequence,                \
                                             __ATOMIC_RELAXED) + 1U;                   \
        uint32_t slot = sequence & 1U;                                                 \
        mailbox->value[slot] = *value;                                                 \
        mailbox->timestamp_ms[slot] = now_ms;                                          \
        __atomic_store_n(&mailbox->publish_sequence, sequence, __ATOMIC_RELEASE);      \
    }                                                                                  \
                                                                                       \
    bool prefix##_read(const mailbox_type *mailbox,                                   \
                       value_type *value,                                              \
                       uint32_t *timestamp_ms)                                         \
    {                                                                                  \
        if (mailbox == NULL || value == NULL || timestamp_ms == NULL) {                \
            return false;                                                              \
        }                                                                              \
        uint32_t before = __atomic_load_n(&mailbox->publish_sequence,                  \
                                           __ATOMIC_ACQUIRE);                          \
        if (before == 0U) {                                                            \
            return false;                                                              \
        }                                                                              \
        uint32_t slot = before & 1U;                                                   \
        *value = mailbox->value[slot];                                                 \
        *timestamp_ms = mailbox->timestamp_ms[slot];                                   \
        uint32_t after = __atomic_load_n(&mailbox->publish_sequence,                   \
                                          __ATOMIC_ACQUIRE);                           \
        return before == after;                                                        \
    }

DEFINE_DOUBLE_BUFFER_MAILBOX(remote_request_mailbox,
                             remote_request_mailbox_t,
                             remote_control_request_t)
DEFINE_DOUBLE_BUFFER_MAILBOX(safety_snapshot_mailbox,
                             safety_snapshot_mailbox_t,
                             vehicle_safety_snapshot_t)
