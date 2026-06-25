/* Target-side regression tests for SBUS parser state and connection timeout. */
#include <string.h>

#include "sbus_service.h"
#include "selftest.h"

static void feed_frame(uint32_t now_ms, const uint8_t frame[SBUS_SERVICE_FRAME_LENGTH])
{
    for (uint8_t i = 0U; i < SBUS_SERVICE_FRAME_LENGTH; ++i) {
        sbus_service_test_feed_byte(now_ms, frame[i]);
    }
}

bool selftest_sbus_service(void)
{
    static const uint8_t valid_idle_frame[SBUS_SERVICE_FRAME_LENGTH] = {
        0x0FU, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
        0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
        0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U
    };
    uint8_t invalid_frame[SBUS_SERVICE_FRAME_LENGTH];
    sbus_debug_state_t snapshot;

    sbus_service_test_reset();
    feed_frame(10U, valid_idle_frame);
    sbus_service_get_snapshot(&snapshot);
    SELFTEST_ASSERT_EQ(1U, snapshot.connected);
    SELFTEST_ASSERT_EQ(1U, snapshot.frame_count);
    SELFTEST_ASSERT_EQ(0U, snapshot.decode_error_count);
    SELFTEST_ASSERT_EQ(10U, snapshot.last_frame_ms);
    SELFTEST_ASSERT_EQ(0U, snapshot.channels[0]);
    SELFTEST_ASSERT_EQ(0U, snapshot.channels[15]);

    sbus_service_test_poll_at(10U + SBUS_SERVICE_CONNECTED_TIMEOUT_MS + 1U);
    sbus_service_get_snapshot(&snapshot);
    SELFTEST_ASSERT_EQ(0U, snapshot.connected);
    SELFTEST_ASSERT_EQ(1U, snapshot.frame_count);

    memcpy(invalid_frame, valid_idle_frame, sizeof(invalid_frame));
    invalid_frame[SBUS_SERVICE_FRAME_LENGTH - 1U] = 0x55U;
    sbus_service_test_reset();
    feed_frame(20U, invalid_frame);
    sbus_service_get_snapshot(&snapshot);
    SELFTEST_ASSERT_EQ(0U, snapshot.connected);
    SELFTEST_ASSERT_EQ(0U, snapshot.frame_count);
    SELFTEST_ASSERT_EQ(1U, snapshot.decode_error_count);

    sbus_service_test_reset();
    sbus_service_test_feed_byte(30U, 0x55U);
    feed_frame(31U, valid_idle_frame);
    sbus_service_get_snapshot(&snapshot);
    SELFTEST_ASSERT_EQ(1U, snapshot.connected);
    SELFTEST_ASSERT_EQ(1U, snapshot.frame_count);

    return true;
}
