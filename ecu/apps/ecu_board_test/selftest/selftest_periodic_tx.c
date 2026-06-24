/* Deterministic regression coverage for the foreground periodic TX service. */
#include <string.h>

#include "periodic_tx.h"
#include "selftest.h"

typedef struct {
    uint16_t id;
    uint8_t data[8];
} captured_can_t;

static uint32_t fake_now_ms;
static uint32_t can_init_calls;
static uint32_t rs485_init_calls;
static uint32_t rs232_init_calls;
static uint32_t can_send_calls;
static uint32_t rs485_write_calls;
static uint32_t rs232_write_calls;
static uint8_t failing_can_channel;
static captured_can_t captured_can[PERIODIC_TX_CAN_COUNT];
static char captured_rs485[PERIODIC_TX_RS485_COUNT][32];
static char captured_rs232[PERIODIC_TX_RS232_COUNT][32];

/** @brief Reset all fake transport observations before one scenario. */
static void reset_fake(void)
{
    fake_now_ms = 0U;
    can_init_calls = 0U;
    rs485_init_calls = 0U;
    rs232_init_calls = 0U;
    can_send_calls = 0U;
    rs485_write_calls = 0U;
    rs232_write_calls = 0U;
    failing_can_channel = 0U;
    memset(captured_can, 0, sizeof(captured_can));
    memset(captured_rs485, 0, sizeof(captured_rs485));
    memset(captured_rs232, 0, sizeof(captured_rs232));
}

/** @brief Return deterministic foreground time controlled by this self-test. */
static uint32_t fake_now(void)
{
    return fake_now_ms;
}

/** @brief Count one-based CAN initialization attempts. */
static bool fake_can_init(uint8_t channel)
{
    ++can_init_calls;
    return channel >= 1U && channel <= PERIODIC_TX_CAN_COUNT;
}

/** @brief Capture one CAN frame and optionally inject a channel failure. */
static bool fake_can_send(uint8_t channel, uint16_t id, const uint8_t data[8])
{
    ++can_send_calls;
    if (channel >= 1U && channel <= PERIODIC_TX_CAN_COUNT) {
        captured_can[channel - 1U].id = id;
        memcpy(captured_can[channel - 1U].data, data, 8U);
    }
    return channel != failing_can_channel;
}

/** @brief Count serial initialization separately for RS485 and RS232. */
static bool fake_serial_init(periodic_tx_serial_kind_t kind, uint8_t channel)
{
    if (kind == PERIODIC_TX_SERIAL_RS485) {
        ++rs485_init_calls;
        return channel >= 1U && channel <= PERIODIC_TX_RS485_COUNT;
    }
    ++rs232_init_calls;
    return kind == PERIODIC_TX_SERIAL_RS232 &&
           channel >= 1U && channel <= PERIODIC_TX_RS232_COUNT;
}

/** @brief Capture one complete serial line in caller-independent storage. */
static bool fake_serial_write(periodic_tx_serial_kind_t kind, uint8_t channel,
                              const uint8_t *data, size_t length)
{
    char (*capture)[32];
    size_t count;
    if (kind == PERIODIC_TX_SERIAL_RS485) {
        ++rs485_write_calls;
        capture = captured_rs485;
        count = PERIODIC_TX_RS485_COUNT;
    } else {
        ++rs232_write_calls;
        capture = captured_rs232;
        count = PERIODIC_TX_RS232_COUNT;
    }
    if (channel == 0U || channel > count || data == NULL || length >= 32U) {
        return false;
    }
    memcpy(capture[channel - 1U], data, length);
    capture[channel - 1U][length] = '\0';
    return true;
}

bool selftest_periodic_tx(void)
{
    static const periodic_tx_hw_ops_t fake_ops = {
        fake_now, fake_can_init, fake_can_send,
        fake_serial_init, fake_serial_write
    };
    const periodic_tx_config_t config = { true, true, true };

    reset_fake();
    periodic_tx_init(&fake_ops, &config);
    SELFTEST_ASSERT_EQ(4U, can_init_calls);
    SELFTEST_ASSERT_EQ(3U, rs485_init_calls);
    SELFTEST_ASSERT_EQ(4U, rs232_init_calls);

    fake_now_ms = 499U;
    periodic_tx_poll();
    SELFTEST_ASSERT_EQ(0U, can_send_calls);

    fake_now_ms = 500U;
    periodic_tx_poll();
    SELFTEST_ASSERT_EQ(4U, can_send_calls);
    SELFTEST_ASSERT_EQ(3U, rs485_write_calls);
    SELFTEST_ASSERT_EQ(4U, rs232_write_calls);
    SELFTEST_ASSERT_EQ(0x601U, captured_can[0].id);
    SELFTEST_ASSERT_EQ(1U, captured_can[0].data[0]);
    SELFTEST_ASSERT_EQ(1U, captured_can[0].data[1]);
    SELFTEST_ASSERT_EQ(0x45U, captured_can[0].data[5]);
    SELFTEST_ASSERT_EQ(0x43U, captured_can[0].data[6]);
    SELFTEST_ASSERT_EQ(0x55U, captured_can[0].data[7]);
    SELFTEST_ASSERT_TRUE(strcmp(captured_rs485[0], "RS485-1,00000001\r\n") == 0);
    SELFTEST_ASSERT_TRUE(strcmp(captured_rs232[0], "RS232-1,00000001\r\n") == 0);

    periodic_tx_suspend();
    fake_now_ms = 1000U;
    periodic_tx_poll();
    SELFTEST_ASSERT_EQ(4U, can_send_calls);
    SELFTEST_ASSERT_TRUE(periodic_tx_is_suspended());
    periodic_tx_resume();
    SELFTEST_ASSERT_TRUE(!periodic_tx_is_suspended());
    SELFTEST_ASSERT_EQ(8U, can_init_calls);
    SELFTEST_ASSERT_EQ(6U, rs485_init_calls);
    SELFTEST_ASSERT_EQ(8U, rs232_init_calls);
    fake_now_ms = 1499U;
    periodic_tx_poll();
    SELFTEST_ASSERT_EQ(4U, can_send_calls);
    fake_now_ms = 1500U;
    periodic_tx_poll();
    SELFTEST_ASSERT_EQ(8U, can_send_calls);
    SELFTEST_ASSERT_TRUE(strcmp(captured_rs232[0], "RS232-1,00000002\r\n") == 0);

    reset_fake();
    fake_now_ms = UINT32_MAX - 249U;
    periodic_tx_init(&fake_ops, &config);
    fake_now_ms = 250U;
    periodic_tx_poll();
    SELFTEST_ASSERT_EQ(4U, can_send_calls);

    reset_fake();
    failing_can_channel = 1U;
    periodic_tx_init(&fake_ops, &config);
    fake_now_ms = 500U;
    periodic_tx_poll();
    periodic_tx_snapshot_t snapshot = periodic_tx_snapshot();
    SELFTEST_ASSERT_EQ(4U, can_send_calls);
    SELFTEST_ASSERT_EQ(3U, rs485_write_calls);
    SELFTEST_ASSERT_EQ(4U, rs232_write_calls);
    SELFTEST_ASSERT_EQ(10U, snapshot.send_success_count);
    SELFTEST_ASSERT_EQ(1U, snapshot.send_failure_count);
    SELFTEST_ASSERT_EQ(0x604U, captured_can[3].id);
    return true;
}
