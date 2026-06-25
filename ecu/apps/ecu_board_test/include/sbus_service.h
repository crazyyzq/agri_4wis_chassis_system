/* Continuous SBUS receiver service used by tests, live monitor, and debugger watches. */
#ifndef ECU_SBUS_SERVICE_H
#define ECU_SBUS_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#define SBUS_SERVICE_CHANNEL_COUNT 16U
#define SBUS_SERVICE_FRAME_LENGTH 25U
#define SBUS_SERVICE_CONNECTED_TIMEOUT_MS 100U

/**
 * @brief Debugger-readable SBUS runtime state.
 *
 * The global instance is intentionally simple and volatile-friendly so SEGGER
 * Embedded Studio can watch it while the firmware is running. Foreground code
 * should still prefer sbus_service_get_snapshot() to avoid partially copied
 * data while the UART ISR is updating the live state.
 */
typedef struct {
    volatile uint32_t connected;
    volatile uint32_t frame_count;
    volatile uint32_t decode_error_count;
    volatile uint32_t uart_error_count;
    volatile uint32_t last_frame_ms;
    volatile uint16_t channels[SBUS_SERVICE_CHANNEL_COUNT];
    volatile uint8_t frame_lost;
    volatile uint8_t failsafe;
    volatile uint8_t channel17;
    volatile uint8_t channel18;
} sbus_debug_state_t;

extern volatile sbus_debug_state_t g_sbus_debug_state;

/** @brief Configure UART1 and start continuous interrupt-driven SBUS reception. */
void sbus_service_init(void);

/** @brief Stop UART1 SBUS interrupts so selftests can safely own service state. */
void sbus_service_stop(void);

/** @brief Refresh timeout-derived connection state from the foreground loop. */
void sbus_service_poll(void);

/** @brief Copy the latest SBUS state for safe formatting in foreground code. */
void sbus_service_get_snapshot(sbus_debug_state_t *snapshot);

/** @brief Reset parser and published state without touching UART hardware. */
void sbus_service_test_reset(void);

/** @brief Feed one byte into the parser for target-side selftests. */
void sbus_service_test_feed_byte(uint32_t now_ms, uint8_t byte);

/** @brief Advance the service clock for timeout-only target-side selftests. */
void sbus_service_test_poll_at(uint32_t now_ms);

#endif
