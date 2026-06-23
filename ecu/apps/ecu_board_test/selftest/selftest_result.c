#include "selftest.h"
#include "test_types.h"

bool selftest_result(void)
{
    test_session_t s;
    test_session_init(&s, "ECU-SELFTEST");
    test_session_add(&s, TEST_REQUIRED, TEST_PASS);
    test_session_add(&s, TEST_OPTIONAL, TEST_SKIP);
    SELFTEST_ASSERT_EQ(TEST_BOARD_PASS, test_session_status(&s));

    test_session_init(&s, "ECU-SELFTEST");
    test_session_add(&s, TEST_REQUIRED, TEST_BLOCKED);
    SELFTEST_ASSERT_EQ(TEST_BOARD_INCOMPLETE, test_session_status(&s));
    return true;
}
