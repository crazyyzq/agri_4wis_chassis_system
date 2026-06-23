#include <stdio.h>
#include "safety_manager.h"
#include "test_cases.h"
test_status_t test_safe_boot(test_context_t *context)
{
    (void)context;
    safety_snapshot_t state = safety_snapshot();
    if (state.active_output != 0U || state.can_term_mask != 0U || state.rs485_tx_mask != 0U) {
        printf("SAFE.BOOT unsafe state do=%u can=0x%02x rs485=0x%02x\n",
               state.active_output, state.can_term_mask, state.rs485_tx_mask);
        return TEST_FAIL;
    }
    return TEST_PASS;
}
