#include <stdio.h>
#include "operator_io.h"
#include "test_cases.h"
#include "test_limits.h"
test_status_t test_power(test_context_t *context)
{
    (void)context;
    uint32_t input_12v, rail_5v, rail_3v3, rail_1v8, rail_1v1;
    if (!operator_read_u32("Measured input supply (mV)", &input_12v) ||
        !operator_read_u32("Measured 5 V rail (mV)", &rail_5v) ||
        !operator_read_u32("Measured 3.3 V rail (mV)", &rail_3v3) ||
        !operator_read_u32("Measured 1.8 V rail (mV)", &rail_1v8) ||
        !operator_read_u32("Measured 1.1 V rail (mV)", &rail_1v1)) return TEST_BLOCKED;
    bool ok = input_12v >= 11400U && input_12v <= 12600U &&
              test_5v_in_range(rail_5v) && test_3v3_in_range(rail_3v3) &&
              rail_1v8 >= 1710U && rail_1v8 <= 1890U &&
              rail_1v1 >= 1045U && rail_1v1 <= 1155U;
    printf("PWR rails_mV 12=%lu 5=%lu 3v3=%lu 1v8=%lu 1v1=%lu\n",
           (unsigned long)input_12v, (unsigned long)rail_5v, (unsigned long)rail_3v3,
           (unsigned long)rail_1v8, (unsigned long)rail_1v1);
    return ok ? TEST_PASS : TEST_FAIL;
}
