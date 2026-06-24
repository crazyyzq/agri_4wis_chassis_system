/* Target-side deterministic checks for RGB status timing and override rules. */
#include "board.h"
#include "selftest.h"
#include "status_led.h"

static uint32_t fake_now_ms;
static bool fake_levels[3];

static uint32_t fake_now(void)
{
    return fake_now_ms;
}

static void fake_write(uint8_t color, bool on)
{
    if (color <= BOARD_RGB_BLUE) {
        fake_levels[color] = on;
    }
}

static bool only_color_is_on(uint8_t color)
{
    for (uint8_t i = BOARD_RGB_RED; i <= BOARD_RGB_BLUE; ++i) {
        if (fake_levels[i] != (i == color)) {
            return false;
        }
    }
    return true;
}

bool selftest_status_led(void)
{
    const status_led_hw_ops_t fake_ops = { fake_now, fake_write };
    fake_now_ms = 0U;
    for (uint8_t i = BOARD_RGB_RED; i <= BOARD_RGB_BLUE; ++i) {
        fake_levels[i] = false;
    }

    status_led_init(&fake_ops);
    status_led_set(STATUS_LED_BOOTING);
    SELFTEST_ASSERT_TRUE(only_color_is_on(BOARD_RGB_RED));

    status_led_set(STATUS_LED_READY);
    SELFTEST_ASSERT_TRUE(only_color_is_on(BOARD_RGB_GREEN));
    fake_now_ms = 499U;
    status_led_poll();
    SELFTEST_ASSERT_TRUE(only_color_is_on(BOARD_RGB_GREEN));
    fake_now_ms = 500U;
    status_led_poll();
    SELFTEST_ASSERT_TRUE(!fake_levels[BOARD_RGB_RED]);
    SELFTEST_ASSERT_TRUE(!fake_levels[BOARD_RGB_GREEN]);
    SELFTEST_ASSERT_TRUE(!fake_levels[BOARD_RGB_BLUE]);

    status_led_set(STATUS_LED_TESTING);
    SELFTEST_ASSERT_TRUE(only_color_is_on(BOARD_RGB_BLUE));
    fake_now_ms += 124U;
    status_led_poll();
    SELFTEST_ASSERT_TRUE(only_color_is_on(BOARD_RGB_BLUE));
    fake_now_ms += 1U;
    status_led_poll();
    SELFTEST_ASSERT_TRUE(!fake_levels[BOARD_RGB_BLUE]);

    status_led_override_begin();
    SELFTEST_ASSERT_TRUE(status_led_is_overridden());
    fake_write(BOARD_RGB_RED, true);
    fake_now_ms += 500U;
    status_led_poll();
    SELFTEST_ASSERT_TRUE(fake_levels[BOARD_RGB_RED]);
    status_led_override_begin();
    status_led_override_end();
    SELFTEST_ASSERT_TRUE(!status_led_is_overridden());
    SELFTEST_ASSERT_EQ(STATUS_LED_TESTING, status_led_get());
    SELFTEST_ASSERT_TRUE(only_color_is_on(BOARD_RGB_BLUE));

    status_led_override_end();
    SELFTEST_ASSERT_TRUE(!status_led_is_overridden());
    status_led_set(STATUS_LED_FAILED);
    SELFTEST_ASSERT_TRUE(only_color_is_on(BOARD_RGB_RED));
    return true;
}
