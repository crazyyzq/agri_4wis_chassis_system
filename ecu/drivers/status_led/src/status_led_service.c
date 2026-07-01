#include "status_led_service.h"

#include <stdbool.h>
#include <string.h>

#include "board.h"

#define STATUS_LED_STEP_MS (250U)

/*
 * ECU front-panel RGB meaning:
 * BOOT: blue heartbeat while scheduler is starting.
 * NO_REMOTE: blue heartbeat when SBUS/remote is not online.
 * READY: solid green when remote and required buses are healthy.
 * ACTIVE: cyan heartbeat when an actuator/high-voltage command is active.
 * WARNING: yellow heartbeat for missing optional/commissioning peripherals.
 * ESTOP/FATAL: red is reserved for operator emergency stop or fatal faults.
 */

static void status_led_write(bool red, bool green, bool blue)
{
    board_rgb_write(BOARD_RGB_RED, red ? 1U : 0U);
    board_rgb_write(BOARD_RGB_GREEN, green ? 1U : 0U);
    board_rgb_write(BOARD_RGB_BLUE, blue ? 1U : 0U);
}

void status_led_service_init(status_led_service_t *service, uint32_t now_ms)
{
    if (service == 0) {
        return;
    }

    memset(service, 0, sizeof(*service));
    service->last_pattern = STATUS_LED_PATTERN_BOOT;
    service->last_update_ms = now_ms;
    status_led_write(false, false, true);
}

void status_led_service_update(status_led_service_t *service,
                               status_led_pattern_t pattern,
                               uint32_t now_ms)
{
    if (service == 0) {
        return;
    }

    if (pattern != service->last_pattern) {
        service->last_pattern = pattern;
        service->phase = 0U;
        service->last_update_ms = now_ms;
    } else if ((uint32_t)(now_ms - service->last_update_ms) >= STATUS_LED_STEP_MS) {
        service->last_update_ms = now_ms;
        service->phase ^= 1U;
    }

    switch (service->last_pattern) {
    case STATUS_LED_PATTERN_NO_REMOTE:
        status_led_write(false, false, service->phase != 0U);
        break;
    case STATUS_LED_PATTERN_READY:
        status_led_write(false, true, false);
        break;
    case STATUS_LED_PATTERN_ACTIVE:
        status_led_write(false, service->phase != 0U, service->phase != 0U);
        break;
    case STATUS_LED_PATTERN_WARNING:
        status_led_write(service->phase != 0U, service->phase != 0U, false);
        break;
    case STATUS_LED_PATTERN_FATAL:
    case STATUS_LED_PATTERN_ESTOP:
        status_led_write(true, false, false);
        break;
    case STATUS_LED_PATTERN_BOOT:
    default:
        status_led_write(false, false, service->phase != 0U);
        break;
    }
}
