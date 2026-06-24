/* Pure algorithm, registry and malformed-input regression coverage. */
#include <string.h>
#include "selftest.h"
#include "test_limits.h"
#include "adc_math.h"
#include "memory_patterns.h"
#include "comm_pattern.h"
#include "sbus_decoder.h"
#include "test_registry.h"

bool selftest_algorithms(void)
{
    SELFTEST_ASSERT_TRUE(test_5v_in_range(4750U));
    SELFTEST_ASSERT_TRUE(test_5v_in_range(5250U));
    SELFTEST_ASSERT_TRUE(!test_5v_in_range(4749U));
    SELFTEST_ASSERT_TRUE(test_3v3_in_range(3135U));
    SELFTEST_ASSERT_TRUE(!test_3v3_in_range(3466U));
    SELFTEST_ASSERT_EQ(3205128U, adc_pin_uv_from_external_uv(5000000U));

    uint32_t words[256];
    memory_mismatch_t mismatch;
    SELFTEST_ASSERT_TRUE(memory_pattern_test(words, 256U, MEMORY_PATTERN_ADDRESS, &mismatch));
    SELFTEST_ASSERT_TRUE(memory_pattern_test(words, 256U, MEMORY_PATTERN_WALKING_ONE, &mismatch));
    SELFTEST_ASSERT_TRUE(!memory_pattern_test(words, 256U, (memory_pattern_t)99, &mismatch));

    comm_frame_t frame;
    uint8_t encoded[64];
    comm_pattern_make(2U, 7U, &frame);
    size_t length = comm_frame_encode(&frame, encoded, sizeof(encoded));
    comm_frame_t decoded;
    SELFTEST_ASSERT_TRUE(length > 0U);
    SELFTEST_ASSERT_EQ(COMM_OK, comm_frame_decode(encoded, length, &decoded));
    encoded[length - 1U] ^= 1U;
    SELFTEST_ASSERT_EQ(COMM_BAD_CRC, comm_frame_decode(encoded, length, &decoded));
    SELFTEST_ASSERT_EQ(0U, comm_crc16(NULL, 1U));

    uint8_t sbus[25] = { 0x0FU };
    sbus[24] = 0x00U;
    sbus_frame_t sbus_out;
    SELFTEST_ASSERT_EQ(SBUS_OK, sbus_decode(sbus, sizeof(sbus), &sbus_out));
    SELFTEST_ASSERT_EQ(0U, sbus_out.channels[0]);

    size_t registry_count;
    const test_descriptor_t *registry = test_registry_all(&registry_count);
    SELFTEST_ASSERT_TRUE(test_registry_find(NULL) == NULL);
    SELFTEST_ASSERT_TRUE(registry_count >= 16U);
    SELFTEST_ASSERT_TRUE(strcmp(registry[0].id, "SAFE.BOOT") == 0);
    SELFTEST_ASSERT_TRUE(strcmp(registry[registry_count - 1U].id, "ETH.SKIP_NO_DEVICE") == 0);
    for (size_t i = 0U; i < registry_count; ++i)
        SELFTEST_ASSERT_TRUE(strstr(registry[i].id, "CANFD") == NULL);
    return true;
}
