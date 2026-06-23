#include <stdio.h>
#include "board.h"
#include "operator_io.h"
#include "safety_manager.h"
#include "test_cases.h"
#include "test_limits.h"
void test_output_cleanup(test_context_t *context) { (void)context; safety_all_off(); }
test_status_t test_digital_output(test_context_t *context)
{
    (void)context;
    if (!operator_confirm("220 ohm / 2 W load fixture is connected to the selected output only")) return TEST_BLOCKED;
    for (uint8_t index = 1U; index <= BOARD_ECU_OUTPUT_COUNT; ++index) {
        uint32_t on_mv, off_mv;
        if (safety_output_on(index) != SAFETY_OK) return TEST_FAIL;
        board_delay_ms(TEST_DO_MAX_ON_MS);
        char prompt[64];
        snprintf(prompt, sizeof(prompt), "EX_OUT%u ON voltage (mV, output to GND)", index);
        if (!operator_read_u32(prompt, &on_mv)) { safety_output_off(index); return TEST_BLOCKED; }
        safety_output_off(index);
        snprintf(prompt, sizeof(prompt), "EX_OUT%u OFF voltage (mV, output to GND)", index);
        if (!operator_read_u32(prompt, &off_mv)) return TEST_BLOCKED;
        printf("DO.%u on_mV=%lu off_mV=%lu\n", index, (unsigned long)on_mv, (unsigned long)off_mv);
        if (on_mv > 500U || off_mv < 10000U) return TEST_FAIL;
    }
    return TEST_PASS;
}
