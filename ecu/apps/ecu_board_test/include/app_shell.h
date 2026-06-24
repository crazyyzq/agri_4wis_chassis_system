#ifndef ECU_APP_SHELL_H
#define ECU_APP_SHELL_H

#include "status_led.h"
#include "test_types.h"

/* Map completed test/session outcomes to the persistent operator indication. */
status_led_state_t app_shell_test_led_state(test_status_t status);
status_led_state_t app_shell_board_led_state(test_board_status_t status);

void app_shell_run(void);
#endif
