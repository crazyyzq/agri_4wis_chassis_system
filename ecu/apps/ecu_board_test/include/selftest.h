/* Lightweight target-side regression tests run before the operator shell. */
#ifndef ECU_SELFTEST_H
#define ECU_SELFTEST_H

#include <stdbool.h>
#include <stdint.h>

/** @brief Target self-test callback returning true only when all checks pass. */
typedef bool (*selftest_fn_t)(void);

typedef struct {
    const char *name;
    selftest_fn_t run;
} selftest_case_t;

#define SELFTEST_ASSERT_TRUE(expr) do { if (!(expr)) { return false; } } while (0)
#define SELFTEST_ASSERT_EQ(expected, actual) \
    do { if ((uint32_t)(expected) != (uint32_t)(actual)) { return false; } } while (0)

/** @brief Verify result aggregation and complete-session rules. @return true only when every assertion passes. */
bool selftest_result(void);
/** @brief Verify safe initialization and exclusive-output rules. @return true only when every assertion passes. */
bool selftest_safety(void);
/** @brief Verify CLI parsing, LED policy, abort parsing, and runner cleanup. @return true only when every assertion passes. */
bool selftest_cli_runner(void);
/** @brief Verify pure algorithms, malformed inputs, and registry policy. @return true only when every assertion passes. */
bool selftest_algorithms(void);
/** @brief Verify RGB timing, exclusive colors, and override restoration. @return true only when every assertion passes. */
bool selftest_status_led(void);
/** @brief Verify periodic communication scheduling, formatting, and ownership. @return true only when every assertion passes. */
bool selftest_periodic_tx(void);
/**
 * @brief Run every registered target-side self-test and print a summary.
 *
 * @return Zero only when every self-test passes; otherwise -1.
 */
int selftest_run_all(void);

#endif
