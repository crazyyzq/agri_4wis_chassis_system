#include "board.h"
#include "operator_io.h"
#include "status_led.h"
#include "test_cases.h"

static void rgb_all_off(void)
{
    for (uint8_t color = BOARD_RGB_RED; color <= BOARD_RGB_BLUE; ++color) {
        board_rgb_write(color, 0U);
    }
}

void test_rgb_cleanup(test_context_t *context)
{
    (void)context;
    rgb_all_off();
    status_led_override_end();
}

test_status_t test_rgb(test_context_t *context)
{
    (void)context;
    static const char *names[] = { "red LED is on", "green LED is on", "blue LED is on" };
    status_led_override_begin();
    for (uint8_t color = BOARD_RGB_RED; color <= BOARD_RGB_BLUE; ++color) {
        rgb_all_off();
        board_rgb_write(color, 1U);
        if (!operator_confirm(names[color])) return TEST_FAIL;
    }
    return TEST_PASS;
}
