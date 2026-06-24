/* Lightweight target-side regression tests run before the operator shell. */
#ifndef ECU_SELFTEST_H
#define ECU_SELFTEST_H

#include <stdbool.h>
#include <stdint.h>

typedef bool (*selftest_fn_t)(void);

typedef struct {
    const char *name;
    selftest_fn_t run;
} selftest_case_t;

#define SELFTEST_ASSERT_TRUE(expr) do { if (!(expr)) { return false; } } while (0)
#define SELFTEST_ASSERT_EQ(expected, actual) \
    do { if ((uint32_t)(expected) != (uint32_t)(actual)) { return false; } } while (0)

bool selftest_result(void);
bool selftest_safety(void);
bool selftest_cli_runner(void);
bool selftest_algorithms(void);
bool selftest_status_led(void);
/* Return zero only when every registered self-test passes. */
int selftest_run_all(void);

#endif
