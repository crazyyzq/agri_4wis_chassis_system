#include "board.h"
#include "operator_io.h"
#include "test_cases.h"
void test_rgb_cleanup(test_context_t *context)
{
    (void)context;
    for (uint8_t color = BOARD_RGB_RED; color <= BOARD_RGB_BLUE; ++color) board_rgb_write(color, 0U);
}
test_status_t test_rgb(test_context_t *context)
{
    (void)context;
    static const char *names[] = { "red LED is on", "green LED is on", "blue LED is on" };
    for (uint8_t color = BOARD_RGB_RED; color <= BOARD_RGB_BLUE; ++color) {
        test_rgb_cleanup(context);
        board_rgb_write(color, 1U);
        if (!operator_confirm(names[color])) return TEST_FAIL;
    }
    return TEST_PASS;
}
