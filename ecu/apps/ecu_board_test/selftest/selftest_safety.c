/* Fake-backend verification of safe initialization and exclusive output rules. */
#include <string.h>
#include "selftest.h"
#include "safety_manager.h"

static bool outputs[12];
static bool terms[4];
static bool rs485_tx[3];
/* Fake callbacks mirror the manager's one-based external indexing in arrays. */
static void set_output(uint8_t i, bool on) { outputs[i - 1U] = on; }
static void set_term(uint8_t i, bool on) { terms[i - 1U] = on; }
static void set_rs485(uint8_t i, bool tx) { rs485_tx[i - 1U] = tx; }
static const safety_hw_ops_t ops = { set_output, set_term, set_rs485 };

bool selftest_safety(void)
{
    memset(outputs, 1, sizeof(outputs));
    memset(terms, 1, sizeof(terms));
    memset(rs485_tx, 1, sizeof(rs485_tx));
    safety_init(&ops);
    SELFTEST_ASSERT_TRUE(safety_backend() == &ops);
    for (uint8_t i = 0; i < 12U; ++i) SELFTEST_ASSERT_TRUE(!outputs[i]);
    for (uint8_t i = 0; i < 4U; ++i) SELFTEST_ASSERT_TRUE(!terms[i]);
    for (uint8_t i = 0; i < 3U; ++i) SELFTEST_ASSERT_TRUE(!rs485_tx[i]);
    SELFTEST_ASSERT_EQ(SAFETY_OK, safety_output_on(1));
    SELFTEST_ASSERT_EQ(SAFETY_BUSY, safety_output_on(2));
    safety_all_off();
    const safety_hw_ops_t invalid_ops = { NULL, set_term, set_rs485 };
    safety_init(&invalid_ops);
    SELFTEST_ASSERT_EQ(SAFETY_INVALID, safety_output_on(1U));
    safety_init(&ops);
    return true;
}
