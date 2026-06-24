/*
 * Application-level RGB status indicator.
 *
 * The board package owns GPIO polarity; this module owns only the runtime
 * state and blink timing. Calls are non-blocking and must be serviced by
 * status_led_poll() from foreground wait loops.
 */
#ifndef ECU_STATUS_LED_H
#define ECU_STATUS_LED_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    STATUS_LED_BOOTING,
    STATUS_LED_READY,
    STATUS_LED_TESTING,
    STATUS_LED_FAILED
} status_led_state_t;

/* Injectable adapter keeps timing behavior deterministic in target self-tests. */
typedef struct {
    uint32_t (*now_ms)(void);
    void (*write)(uint8_t color, bool on);
} status_led_hw_ops_t;

/**
 * @brief Install an RGB/timing adapter and reset the state to BOOTING.
 *
 * @param ops Borrowed adapter pointer; null or incomplete adapters disable I/O.
 *
 * @note The initial BOOTING indication is rendered immediately when valid.
 */
void status_led_init(const status_led_hw_ops_t *ops);

/**
 * @brief Install the HPM6750 MCHTMR clock and board RGB GPIO adapter.
 *
 * @note Timing is derived from the foreground-polled 64-bit MCHTMR counter; no
 *       interrupt or peripheral timer is reserved.
 */
void status_led_init_default(void);

/**
 * @brief Change status, restart its animation phase, and render immediately.
 *
 * @param state BOOTING, READY, TESTING, or FAILED; invalid values fall back to
 *              STATUS_LED_FAILED.
 *
 * @note BOOTING is steady red; READY blinks green at a 500 ms half-period;
 *       TESTING blinks blue and FAILED blinks red at a 125 ms half-period.
 */
void status_led_set(status_led_state_t state);
/** @brief Read the selected logical RGB status. @return Current state value. */
status_led_state_t status_led_get(void);

/**
 * @brief Advance the current RGB animation from the adapter clock.
 *
 * @note Non-blocking and safe across uint32_t millisecond wrap. Does nothing for
 *       BOOTING, override mode, or an invalid adapter.
 */
void status_led_poll(void);

/**
 * @brief Suspend status rendering so a hardware test owns all RGB channels.
 *
 * @note Repeated calls keep override active and do not alter physical outputs.
 */
void status_led_override_begin(void);
/**
 * @brief End direct RGB ownership and immediately restore status indication.
 *
 * @note Idempotent when no override is active. Restoration restarts the blink
 *       phase in the on state.
 */
void status_led_override_end(void);
/** @brief Query direct RGB ownership. @return true while override is active. */
bool status_led_is_overridden(void);

#endif
