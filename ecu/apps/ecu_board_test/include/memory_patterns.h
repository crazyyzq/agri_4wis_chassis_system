/* Destructive word-oriented RAM pattern generator and verifier. */
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
/**
 * @brief Destructively write and verify a 32-bit memory region.
 *
 * @param base       First volatile 32-bit word to test.
 * @param word_count Number of consecutive words to overwrite and verify.
 * @param pattern    Fixed, address-derived, or walking-one test pattern.
 * @param mismatch   Optional caller-owned record receiving the address,
 *                   expected value, and actual value of the first mismatch.
 * @return true when every write reads back correctly; false for invalid
 *         arguments/patterns or the first verification mismatch.
 *
 * @warning Every supplied word is overwritten. A walking-one test performs 32
 *          complete write/read passes and leaves the final walking-one value.
 */
bool memory_pattern_test(volatile uint32_t *base, size_t word_count,
                         memory_pattern_t pattern, memory_mismatch_t *mismatch);
#endif
