/* Current-limited 500 ms pulse and peak-hold measurement for 12 outputs. */
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
        char prompt[96];
        snprintf(prompt, sizeof(prompt),
                 "Set meter to peak-hold for EX_OUT%u, then start the 500 ms pulse", index);
        if (!operator_confirm(prompt)) return TEST_BLOCKED;
        if (safety_output_on(index) != SAFETY_OK) return TEST_FAIL;
        board_delay_ms(TEST_DO_MAX_ON_MS);
        safety_output_off(index);
        snprintf(prompt, sizeof(prompt), "EX_OUT%u peak ON voltage (mV, output to GND)", index);
        if (!operator_read_u32(prompt, &on_mv)) return TEST_BLOCKED;
        snprintf(prompt, sizeof(prompt), "EX_OUT%u OFF voltage (mV, output to GND)", index);
        if (!operator_read_u32(prompt, &off_mv)) return TEST_BLOCKED;
        printf("DO.%u on_mV=%lu off_mV=%lu\n", index, (unsigned long)on_mv, (unsigned long)off_mv);
        if (on_mv > 500U || off_mv < 10000U) return TEST_FAIL;
    }
    return TEST_PASS;
}
