/* Monotonic time value helpers used by pure state machines. */
#ifndef ECU_TIME_H
#define ECU_TIME_H

#include <stdbool.h>
#include <stdint.h>

static inline bool ecu_time_elapsed(uint32_t now_ms, uint32_t start_ms, uint32_t duration_ms)
{
    return (uint32_t)(now_ms - start_ms) >= duration_ms;
}

#endif /* ECU_TIME_H */
