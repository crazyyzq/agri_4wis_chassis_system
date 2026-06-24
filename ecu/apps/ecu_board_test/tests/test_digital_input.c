/* Operator-driven inactive/12 V stability checks for 12 isolated inputs. */
#include <stdio.h>
#include "board.h"
#include "operator_io.h"
#include "test_cases.h"
static bool input_stable(uint8_t index, uint8_t expected)
{
    for (uint8_t sample = 0U; sample < 20U; ++sample) {
        if (board_ecu_input_read(index) != expected) return false;
        board_delay_ms(2U);
    }
    return true;
}
test_status_t test_digital_input(test_context_t *context)
{
    (void)context;
    for (uint8_t index = 1U; index <= BOARD_ECU_INPUT_COUNT; ++index) {
        char prompt[72];
        snprintf(prompt, sizeof(prompt), "Set EX_IN%u inactive (0 V differential)", index);
        if (!operator_confirm(prompt)) return TEST_BLOCKED;
        if (!input_stable(index, 0U)) return TEST_FAIL;
        snprintf(prompt, sizeof(prompt), "Set EX_IN%u active (12 V differential)", index);
        if (!operator_confirm(prompt)) return TEST_BLOCKED;
        if (!input_stable(index, 1U)) return TEST_FAIL;
    }
    return TEST_PASS;
}
