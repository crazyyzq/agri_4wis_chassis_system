/*
 * Foreground-driven RGB status state machine.
 *
 * No timer or interrupt is reserved. A wrapping 32-bit millisecond timestamp
 * is sufficient because every configured half-period is much shorter than
 * the unsigned wrap interval.
 */
#include <stddef.h>

#include "board.h"
#include "hpm_clock_drv.h"
#include "hpm_mchtmr_drv.h"
#include "status_led.h"

#define READY_HALF_PERIOD_MS  (500U)
#define ACTIVE_HALF_PERIOD_MS (125U)

static const status_led_hw_ops_t *s_ops;
static status_led_state_t s_state = STATUS_LED_BOOTING;
static uint32_t s_last_change_ms;
static bool s_phase_on = true;
static bool s_overridden;

/**
 * @brief Convert the free-running MCHTMR counter to wrapping milliseconds.
 * @return Low 32 bits of elapsed milliseconds, or zero if its clock is absent.
 */
static uint32_t default_now_ms(void)
{
    uint32_t frequency = clock_get_frequency(clock_mchtmr0);
    if (frequency == 0U) {
        return 0U;
    }
    return (uint32_t)((mchtmr_get_count(HPM_MCHTMR) * 1000ULL) / frequency);
}

/** @brief Adapt logical color state to the board-owned RGB polarity API. */
static void default_write(uint8_t color, bool on)
{
    board_rgb_write(color, on ? 1U : 0U);
}

static const status_led_hw_ops_t s_default_ops = {
    default_now_ms,
    default_write,
};

/** @brief Check that both required borrowed adapter callbacks are callable. */
static bool adapter_is_valid(void)
{
    return s_ops != NULL && s_ops->now_ms != NULL && s_ops->write != NULL;
}

/**
 * @brief Render at most one logical color, explicitly clearing the other two.
 *
 * @note board_rgb_write() owns the active-low electrical conversion; this layer
 *       uses logical on/off values only.
 */
static void write_exclusive(uint8_t color, bool on)
{
    s_ops->write(BOARD_RGB_RED, false);
    s_ops->write(BOARD_RGB_GREEN, false);
    s_ops->write(BOARD_RGB_BLUE, false);
    if (on) {
        s_ops->write(color, true);
    }
}

/**
 * @brief Render the current state/phase unless a hardware test owns the RGB pins.
 *
 * @note Unknown state values deliberately use the FAILED red indication.
 */
static void render(void)
{
    if (!adapter_is_valid() || s_overridden) {
        return;
    }

    switch (s_state) {
    case STATUS_LED_BOOTING:
        write_exclusive(BOARD_RGB_RED, true);
        break;
    case STATUS_LED_READY:
        write_exclusive(BOARD_RGB_GREEN, s_phase_on);
        break;
    case STATUS_LED_TESTING:
        write_exclusive(BOARD_RGB_BLUE, s_phase_on);
        break;
    case STATUS_LED_FAILED:
    default:
        write_exclusive(BOARD_RGB_RED, s_phase_on);
        break;
    }
}

void status_led_init(const status_led_hw_ops_t *ops)
{
    s_ops = ops;
    s_state = STATUS_LED_BOOTING;
    s_overridden = false;
    s_phase_on = true;
    s_last_change_ms = adapter_is_valid() ? s_ops->now_ms() : 0U;
    render();
}

void status_led_init_default(void)
{
    status_led_init(&s_default_ops);
}

void status_led_set(status_led_state_t state)
{
    if (state > STATUS_LED_FAILED) {
        state = STATUS_LED_FAILED;
    }
    s_state = state;
    s_phase_on = true;
    s_last_change_ms = adapter_is_valid() ? s_ops->now_ms() : 0U;
    render();
}

status_led_state_t status_led_get(void)
{
    return s_state;
}

void status_led_poll(void)
{
    if (!adapter_is_valid() || s_overridden || s_state == STATUS_LED_BOOTING) {
        return;
    }

    uint32_t half_period = s_state == STATUS_LED_READY
        ? READY_HALF_PERIOD_MS
        : ACTIVE_HALF_PERIOD_MS;
    uint32_t now = s_ops->now_ms();
    /* Unsigned subtraction preserves elapsed time across uint32_t wrap. */
    uint32_t elapsed = now - s_last_change_ms;
    if (elapsed < half_period) {
        return;
    }

    uint32_t periods = elapsed / half_period;
    s_last_change_ms += periods * half_period;
    if ((periods & 1U) != 0U) {
        s_phase_on = !s_phase_on;
        render();
    }
}

void status_led_override_begin(void)
{
    s_overridden = true;
}

void status_led_override_end(void)
{
    if (!s_overridden) {
        return;
    }
    s_overridden = false;
    s_phase_on = true;
    s_last_change_ms = adapter_is_valid() ? s_ops->now_ms() : 0U;
    render();
}

bool status_led_is_overridden(void)
{
    return s_overridden;
}
