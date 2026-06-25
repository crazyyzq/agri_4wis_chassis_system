/* Foreground-polled, compile-configured periodic communication transmitter. */
#ifndef ECU_PERIODIC_TX_H
#define ECU_PERIODIC_TX_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define PERIODIC_TX_PERIOD_MS 500U
#define PERIODIC_TX_CAN_COUNT 4U
#define PERIODIC_TX_RS485_COUNT 3U
#define PERIODIC_TX_RS232_COUNT 4U

typedef enum {
    PERIODIC_TX_SERIAL_RS485,
    PERIODIC_TX_SERIAL_RS232
} periodic_tx_serial_kind_t;

typedef struct {
    bool can_enabled;
    bool rs485_enabled;
    bool rs232_enabled;
} periodic_tx_config_t;

typedef struct {
    uint32_t (*now_ms)(void);
    bool (*can_init)(uint8_t channel);
    bool (*can_send)(uint8_t channel, uint16_t id, const uint8_t data[8]);
    bool (*serial_init)(periodic_tx_serial_kind_t kind, uint8_t channel);
    bool (*serial_write)(periodic_tx_serial_kind_t kind, uint8_t channel,
                         const uint8_t *data, size_t length);
} periodic_tx_hw_ops_t;

typedef struct {
    uint32_t can_sequence[PERIODIC_TX_CAN_COUNT];
    uint32_t rs485_sequence[PERIODIC_TX_RS485_COUNT];
    uint32_t rs232_sequence[PERIODIC_TX_RS232_COUNT];
    uint32_t send_success_count;
    uint32_t send_failure_count;
    bool suspended;
} periodic_tx_snapshot_t;

/**
 * @brief Initialize the periodic scheduler with an injected hardware backend.
 *
 * @param ops    Borrowed backend pointer that must remain valid while in use.
 * @param config Enabled communication groups; null disables every group.
 *
 * @note Enabled physical channels are initialized immediately. Failed channels
 *       remain inactive while other channels continue independently.
 */
void periodic_tx_init(const periodic_tx_hw_ops_t *ops,
                      const periodic_tx_config_t *config);

/**
 * @brief Initialize the HPM6750 backend from compile-time group definitions.
 */
void periodic_tx_init_default(void);

/**
 * @brief Send one burst when the foreground 500 ms deadline has elapsed.
 *
 * @note This call never waits for receive data and emits at most one burst per
 *       poll, even when multiple periods elapsed.
 */
void periodic_tx_poll(void);

/** @brief Idempotently suppress periodic transmissions without clearing state. */
void periodic_tx_suspend(void);

/**
 * @brief Reinitialize enabled channels and restart the 500 ms deadline.
 *
 * @note Counters and accumulated statistics are preserved. Calling this while
 *       already active has no effect.
 */
void periodic_tx_resume(void);

/** @brief Query scheduler ownership. @return true while traffic is suppressed. */
bool periodic_tx_is_suspended(void);

/**
 * @brief Copy the current sequence counters and send statistics.
 * @return Value snapshot owned by the caller.
 */
periodic_tx_snapshot_t periodic_tx_snapshot(void);

#endif
