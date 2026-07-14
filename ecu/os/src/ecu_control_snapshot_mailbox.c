#include <stddef.h>

#include "FreeRTOS.h"
#include "task.h"

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
        /* The 1 ms safety publisher has higher priority than both consumers.         \
         * A retry-only seqlock reader can therefore be preempted on every retry and  \
         * falsely report a missing safety snapshot.  Copy the small fixed-size      \
         * snapshot in one bounded critical section.  No ISR publishes these         \
         * mailboxes, and the section contains no calls, loops or blocking work.      \
         */                                                                            \
        taskENTER_CRITICAL();                                                          \
        uint32_t sequence = __atomic_load_n(&mailbox->publish_sequence,                \
                                             __ATOMIC_ACQUIRE);                        \
        if (sequence == 0U) {                                                          \
            taskEXIT_CRITICAL();                                                       \
            return false;                                                              \
        }                                                                              \
        uint32_t slot = sequence & 1U;                                                 \
        *value = mailbox->value[slot];                                                 \
        *timestamp_ms = mailbox->timestamp_ms[slot];                                   \
        taskEXIT_CRITICAL();                                                           \
        return true;                                                                   \
    }

DEFINE_DOUBLE_BUFFER_MAILBOX(remote_request_mailbox,
                             remote_request_mailbox_t,
                             remote_control_request_t)
DEFINE_DOUBLE_BUFFER_MAILBOX(safety_snapshot_mailbox,
                             safety_snapshot_mailbox_t,
                             vehicle_safety_snapshot_t)
