/* ECU board-test entry point: safe board init, self-test, then operator shell. */
#include <stdio.h>

#include "app_shell.h"
#include "board.h"
#include "hpm_ppor_drv.h"
#include "safety_manager.h"
#include "selftest.h"
#include "status_led.h"

static void real_output_write(uint8_t index, bool on) { board_ecu_output_write(index, on ? 1U : 0U); }
static void real_can_term_write(uint8_t index, bool enable) { board_set_can_termination(index, enable); }
static void real_rs485_direction_write(uint8_t index, bool transmit)
{
    (void)index;
    (void)transmit;
    /* UART hardware DE owns the physical direction pin on this pinmux. */
}
static const safety_hw_ops_t real_safety_ops = {
    real_output_write, real_can_term_write, real_rs485_direction_write
};

int main(void)
{
    /* board_init() establishes the hardware-safe GPIO levels before the
     * operator is allowed to issue any test command. */
    board_init();
    status_led_init_default();
    status_led_set(STATUS_LED_BOOTING);
    uint32_t reset_flags = ppor_reset_get_flags(HPM_PPOR);
    printf("RESET flags=0x%08lx status=0x%08lx\n", (unsigned long)reset_flags,
           (unsigned long)ppor_reset_get_status(HPM_PPOR));
    ppor_reset_clear_flags(HPM_PPOR, reset_flags);
    safety_init(&real_safety_ops);
    int selftest_result = selftest_run_all();
    /* The safety self-test installs a fake backend; restore hardware ownership. */
    safety_init(&real_safety_ops);
    /* The status self-test also installs a fake adapter with automatic storage. */
    status_led_init_default();
    if (selftest_result == 0) {
        status_led_set(STATUS_LED_READY);
        printf("ECU_TEST READY board=%s sdk=1.11.0\n", BOARD_NAME);
    } else {
        status_led_set(STATUS_LED_FAILED);
        printf("ECU_TEST SELFTEST_FAILED board=%s sdk=1.11.0\n", BOARD_NAME);
    }
    app_shell_run();
    return 0;
}
