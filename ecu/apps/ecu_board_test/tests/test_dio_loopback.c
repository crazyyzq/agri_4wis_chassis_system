/* Fixture loopback between each protected output and matching digital input. */
#include <stdio.h>
#include "board.h"
#include "operator_io.h"
#include "safety_manager.h"
#include "test_cases.h"
test_status_t test_dio_loopback(test_context_t *context)
{
    (void)context;
    if (!operator_confirm("DIO fixture: each DI+ to 12 V and matching DI- to EX_OUT")) return TEST_BLOCKED;
    for (uint8_t index = 1U; index <= BOARD_ECU_OUTPUT_COUNT; ++index) {
        if (safety_output_on(index) != SAFETY_OK) return TEST_FAIL;
        board_delay_ms(20U);
        bool asserted = board_ecu_input_read(index) != 0U;
        safety_output_off(index);
        board_delay_ms(20U);
        bool released = board_ecu_input_read(index) == 0U;
        printf("DIO.%u asserted=%u released=%u\n", index, asserted, released);
        if (!asserted || !released) { safety_all_off(); return TEST_FAIL; }
    }
    return TEST_PASS;
}
