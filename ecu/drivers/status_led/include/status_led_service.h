#ifndef STATUS_LED_SERVICE_H
#define STATUS_LED_SERVICE_H

#include <stdint.h>

typedef enum {
    /* Firmware has started, but CPU0 has not yet published a stable runtime status. */
    STATUS_LED_PATTERN_BOOT = 0,
    /* Scheduler is alive, but SBUS/remote is not online. Actuator outputs remain safety-clamped. */
    STATUS_LED_PATTERN_NO_REMOTE,
    /* Remote and required communication paths are healthy; no active output command is present. */
    STATUS_LED_PATTERN_READY,
    /* At least one intentional output command is active, such as HV, brake release, hydraulic or motion. */
    STATUS_LED_PATTERN_ACTIVE,
    /* Non-fatal peripheral or communication warning. The ECU is alive and still enforcing safety. */
    STATUS_LED_PATTERN_WARNING,
    /* Operator-requested emergency stop, normally from SBUS CH13. */
    STATUS_LED_PATTERN_ESTOP,
    /* Fatal ECU-side fault that must be investigated before motion is allowed. */
    STATUS_LED_PATTERN_FATAL
} status_led_pattern_t;

typedef struct {
    status_led_pattern_t last_pattern;
    uint32_t last_update_ms;
    uint8_t phase;
} status_led_service_t;

void status_led_service_init(status_led_service_t *service, uint32_t now_ms);
void status_led_service_update(status_led_service_t *service,
                               status_led_pattern_t pattern,
                               uint32_t now_ms);

#endif /* STATUS_LED_SERVICE_H */
