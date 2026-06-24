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

/* Install an adapter and reset the state machine to BOOTING. */
void status_led_init(const status_led_hw_ops_t *ops);

/* Install the HPM6750 MCHTMR and board RGB adapter. */
void status_led_init_default(void);

/* Change state, restart its animation phase and render it immediately. */
void status_led_set(status_led_state_t state);
status_led_state_t status_led_get(void);

/* Advance the active animation using the adapter clock; never blocks. */
void status_led_poll(void);

/* Temporarily give a hardware test direct ownership of all three RGB channels. */
void status_led_override_begin(void);
void status_led_override_end(void);
bool status_led_is_overridden(void);

#endif
