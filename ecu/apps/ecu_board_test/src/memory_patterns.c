/* Destructive RAM pattern write/read verification used by internal RAM/SDRAM. */
#include <stddef.h>
#include "memory_patterns.h"
/**
 * @brief Derive the value written for one pattern pass and word index.
 *
 * @param pattern Selected memory pattern.
 * @param index   Zero-based word index within the tested region.
 * @param walk    Walking-one bit position; ignored by other patterns.
 * @return The 32-bit value to write and later verify.
 */
static uint32_t pattern_value(memory_pattern_t pattern, size_t index, uint32_t walk)
{
    switch (pattern) {
    case MEMORY_PATTERN_ZERO: return 0U;
    case MEMORY_PATTERN_ONES: return UINT32_MAX;
    case MEMORY_PATTERN_AA: return 0xAAAAAAAAU;
    case MEMORY_PATTERN_55: return 0x55555555U;
    case MEMORY_PATTERN_ADDRESS: return (uint32_t)index;
    case MEMORY_PATTERN_WALKING_ONE: return 1UL << walk;
    default: return 0U;
    }
}
bool memory_pattern_test(volatile uint32_t *base, size_t word_count,
                         memory_pattern_t pattern, memory_mismatch_t *mismatch)
{
    if (base == NULL || word_count == 0U || pattern > MEMORY_PATTERN_WALKING_ONE) return false;
    uint32_t passes = pattern == MEMORY_PATTERN_WALKING_ONE ? 32U : 1U;
    for (uint32_t walk = 0U; walk < passes; ++walk) {
        for (size_t i = 0; i < word_count; ++i) base[i] = pattern_value(pattern, i, walk);
        for (size_t i = 0; i < word_count; ++i) {
            uint32_t expected = pattern_value(pattern, i, walk);
            uint32_t actual = base[i];
            if (actual != expected) {
                if (mismatch != NULL) *mismatch = (memory_mismatch_t){ (uintptr_t)&base[i], expected, actual };
                return false;
            }
        }
    }
    return true;
}
