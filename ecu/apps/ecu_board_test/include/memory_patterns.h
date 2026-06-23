#ifndef ECU_MEMORY_PATTERNS_H
#define ECU_MEMORY_PATTERNS_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
typedef enum {
    MEMORY_PATTERN_ZERO, MEMORY_PATTERN_ONES, MEMORY_PATTERN_AA,
    MEMORY_PATTERN_55, MEMORY_PATTERN_ADDRESS, MEMORY_PATTERN_WALKING_ONE
} memory_pattern_t;
typedef struct { uintptr_t address; uint32_t expected; uint32_t actual; } memory_mismatch_t;
bool memory_pattern_test(volatile uint32_t *base, size_t word_count,
                         memory_pattern_t pattern, memory_mismatch_t *mismatch);
#endif
