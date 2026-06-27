#include <stdbool.h>

#include "ecu_tasks.h"

typedef struct {
    bool initialized;
    uint32_t heartbeat_sequence;
    uint32_t last_service_ms;
} ecu_cpu1_runtime_context_t;

static ecu_cpu1_runtime_context_t s_cpu1_runtime;

static void ecu_cpu1_runtime_init_once(uint32_t now_ms)
{
    if (s_cpu1_runtime.initialized) {
        return;
    }
    s_cpu1_runtime.heartbeat_sequence = 0U;
    s_cpu1_runtime.last_service_ms = now_ms;
    s_cpu1_runtime.initialized = true;
}

void ecu_task_cpu1_service_step(uint32_t now_ms)
{
    ecu_cpu1_runtime_init_once(now_ms);
    s_cpu1_runtime.heartbeat_sequence++;
    s_cpu1_runtime.last_service_ms = now_ms;
}
