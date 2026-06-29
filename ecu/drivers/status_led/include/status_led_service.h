#ifndef STATUS_LED_SERVICE_H
#define STATUS_LED_SERVICE_H

#include <stdint.h>

typedef enum {
    STATUS_LED_PATTERN_BOOT = 0,
    STATUS_LED_PATTERN_READY,
    STATUS_LED_PATTERN_WARNING,
    STATUS_LED_PATTERN_ESTOP
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
