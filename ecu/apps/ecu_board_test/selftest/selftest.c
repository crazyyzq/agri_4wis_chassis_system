/* Ordered target-side regression-test dispatcher and summary. */
#include <stdio.h>
#include "selftest.h"

int selftest_run_all(void)
{
    static const selftest_case_t cases[] = {
        { "result", selftest_result },
        { "safety", selftest_safety },
        { "cli_runner", selftest_cli_runner },
        { "algorithms", selftest_algorithms },
        { "status_led", selftest_status_led },
        { "periodic_tx", selftest_periodic_tx },
        { "sbus_service", selftest_sbus_service },
        { "debug_monitor", selftest_debug_monitor },
    };
    uint32_t pass = 0U;
    uint32_t fail = 0U;

    for (uint32_t i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        bool ok = cases[i].run();
        printf("SELFTEST %s %s\n", cases[i].name, ok ? "PASS" : "FAIL");
        ok ? ++pass : ++fail;
    }
    printf("SELFTEST SUMMARY pass=%lu fail=%lu\n", (unsigned long)pass,
           (unsigned long)fail);
    return fail == 0U ? 0 : -1;
}
