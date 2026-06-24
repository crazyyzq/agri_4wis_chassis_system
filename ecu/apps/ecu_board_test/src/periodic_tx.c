/* Hardware-independent scheduling and deterministic diagnostic frame formatting. */
#include <stdio.h>
#include <string.h>

#include "periodic_tx.h"

typedef struct {
    const periodic_tx_hw_ops_t *ops;
    periodic_tx_config_t config;
    bool can_ready[PERIODIC_TX_CAN_COUNT];
    bool rs485_ready[PERIODIC_TX_RS485_COUNT];
    bool rs232_ready[PERIODIC_TX_RS232_COUNT];
    periodic_tx_snapshot_t snapshot;
    uint32_t last_ms;
} periodic_tx_state_t;

static periodic_tx_state_t s_state;

/** @brief Record exactly one physical-channel send attempt. */
static void record_send(bool success)
{
    if (success) {
        ++s_state.snapshot.send_success_count;
    } else {
        ++s_state.snapshot.send_failure_count;
    }
}

/** @brief Initialize every enabled channel and retain per-channel readiness. */
static void configure_channels(void)
{
    memset(s_state.can_ready, 0, sizeof(s_state.can_ready));
    memset(s_state.rs485_ready, 0, sizeof(s_state.rs485_ready));
    memset(s_state.rs232_ready, 0, sizeof(s_state.rs232_ready));
    if (s_state.ops == NULL) {
        return;
    }
    if (s_state.config.can_enabled && s_state.ops->can_init != NULL) {
        for (uint8_t channel = 1U; channel <= PERIODIC_TX_CAN_COUNT; ++channel) {
            s_state.can_ready[channel - 1U] = s_state.ops->can_init(channel);
        }
    }
    if (s_state.config.rs485_enabled && s_state.ops->serial_init != NULL) {
        for (uint8_t channel = 1U; channel <= PERIODIC_TX_RS485_COUNT; ++channel) {
            s_state.rs485_ready[channel - 1U] =
                s_state.ops->serial_init(PERIODIC_TX_SERIAL_RS485, channel);
        }
    }
    if (s_state.config.rs232_enabled && s_state.ops->serial_init != NULL) {
        for (uint8_t channel = 1U; channel <= PERIODIC_TX_RS232_COUNT; ++channel) {
            s_state.rs232_ready[channel - 1U] =
                s_state.ops->serial_init(PERIODIC_TX_SERIAL_RS232, channel);
        }
    }
}

/** @brief Format and attempt one Classic CAN diagnostic data frame. */
static void send_can(uint8_t channel)
{
    uint32_t sequence = ++s_state.snapshot.can_sequence[channel - 1U];
    uint8_t data[8] = {
        channel,
        (uint8_t)sequence,
        (uint8_t)(sequence >> 8U),
        (uint8_t)(sequence >> 16U),
        (uint8_t)(sequence >> 24U),
        0x45U, 0x43U, 0x55U
    };
    bool success = s_state.ops->can_send != NULL &&
                   s_state.ops->can_send(channel,
                       (uint16_t)(0x600U + channel), data);
    record_send(success);
}

/** @brief Format and attempt one readable RS485 or RS232 diagnostic line. */
static void send_serial(periodic_tx_serial_kind_t kind, uint8_t channel,
                        uint32_t *sequence)
{
    char line[32];
    ++(*sequence);
    const char *prefix = kind == PERIODIC_TX_SERIAL_RS485 ? "RS485" : "RS232";
    int length = snprintf(line, sizeof(line), "%s-%u,%08lX\r\n", prefix,
                          channel, (unsigned long)*sequence);
    bool success = length > 0 && (size_t)length < sizeof(line) &&
                   s_state.ops->serial_write != NULL &&
                   s_state.ops->serial_write(kind, channel,
                                             (const uint8_t *)line,
                                             (size_t)length);
    record_send(success);
}

void periodic_tx_init(const periodic_tx_hw_ops_t *ops,
                      const periodic_tx_config_t *config)
{
    memset(&s_state, 0, sizeof(s_state));
    s_state.ops = ops;
    if (config != NULL) {
        s_state.config = *config;
    }
    configure_channels();
    if (s_state.ops != NULL && s_state.ops->now_ms != NULL) {
        s_state.last_ms = s_state.ops->now_ms();
    }
}

void periodic_tx_poll(void)
{
    if (s_state.snapshot.suspended || s_state.ops == NULL ||
        s_state.ops->now_ms == NULL) {
        return;
    }
    uint32_t now = s_state.ops->now_ms();
    uint32_t elapsed = now - s_state.last_ms;
    if (elapsed < PERIODIC_TX_PERIOD_MS) {
        return;
    }
    s_state.last_ms += (elapsed / PERIODIC_TX_PERIOD_MS) * PERIODIC_TX_PERIOD_MS;

    for (uint8_t channel = 1U; channel <= PERIODIC_TX_CAN_COUNT; ++channel) {
        if (s_state.can_ready[channel - 1U]) {
            send_can(channel);
        }
    }
    for (uint8_t channel = 1U; channel <= PERIODIC_TX_RS485_COUNT; ++channel) {
        if (s_state.rs485_ready[channel - 1U]) {
            send_serial(PERIODIC_TX_SERIAL_RS485, channel,
                        &s_state.snapshot.rs485_sequence[channel - 1U]);
        }
    }
    for (uint8_t channel = 1U; channel <= PERIODIC_TX_RS232_COUNT; ++channel) {
        if (s_state.rs232_ready[channel - 1U]) {
            send_serial(PERIODIC_TX_SERIAL_RS232, channel,
                        &s_state.snapshot.rs232_sequence[channel - 1U]);
        }
    }
}

void periodic_tx_suspend(void)
{
    s_state.snapshot.suspended = true;
}

void periodic_tx_resume(void)
{
    if (!s_state.snapshot.suspended) {
        return;
    }
    configure_channels();
    if (s_state.ops != NULL && s_state.ops->now_ms != NULL) {
        s_state.last_ms = s_state.ops->now_ms();
    }
    s_state.snapshot.suspended = false;
}

bool periodic_tx_is_suspended(void)
{
    return s_state.snapshot.suspended;
}

periodic_tx_snapshot_t periodic_tx_snapshot(void)
{
    return s_state.snapshot;
}
