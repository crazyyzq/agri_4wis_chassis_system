/* Polled debugger-controlled monitor; hardware access is isolated behind a backend. */
#include <stdio.h>
#include <string.h>

#include "debug_monitor.h"

#define ECU_DEBUG_MONITOR_DEFAULT_PERIOD_MS 200U
#define ECU_DEBUG_MONITOR_MIN_PERIOD_MS 50U
#define ECU_DEBUG_MONITOR_DO_MASK_ALL 0x00000FFFUL

volatile ecu_debug_monitor_t g_ecu_debug_monitor = {
    0U, ECU_DEBUG_VIEW_NONE, 0U, ECU_DEBUG_MONITOR_DEFAULT_PERIOD_MS, 0U, 0U
};

static uint32_t default_now_ms(void);
static bool default_read_sbus(sbus_frame_t *frame);
static bool default_read_adc_mv(uint8_t channel, uint32_t *mv);
static uint8_t default_read_di(uint8_t channel);
static void default_write_do_mask(uint32_t mask);
static void default_write_line(const char *line);

static const ecu_debug_monitor_backend_t s_default_backend = {
    default_now_ms,
    default_read_sbus,
    default_read_adc_mv,
    default_read_di,
    default_write_do_mask,
    default_write_line
};

static const ecu_debug_monitor_backend_t *s_backend;
static bool s_suspended;
static uint32_t s_last_print_ms;

static const ecu_debug_monitor_backend_t *active_backend(void)
{
    if (s_backend == NULL) {
        s_backend = &s_default_backend;
    }
    return s_backend;
}

static uint32_t normalized_period_ms(void)
{
    uint32_t period = g_ecu_debug_monitor.period_ms;
    return period < ECU_DEBUG_MONITOR_MIN_PERIOD_MS ? ECU_DEBUG_MONITOR_MIN_PERIOD_MS : period;
}

static bool selected(uint8_t index)
{
    return g_ecu_debug_monitor.channel == 0U || g_ecu_debug_monitor.channel == index;
}

static void append_text(char *line, size_t capacity, const char *text)
{
    size_t used = strlen(line);
    if (text != NULL && used + 1U < capacity) {
        (void)snprintf(&line[used], capacity - used, "%s", text);
    }
}

static void append_u32(char *line, size_t capacity, const char *prefix, uint32_t value)
{
    char fragment[32];
    (void)snprintf(fragment, sizeof(fragment), "%s%lu", prefix, (unsigned long)value);
    append_text(line, capacity, fragment);
}

static void print_adc(const ecu_debug_monitor_backend_t *backend)
{
    char line[160] = "ADC";
    for (uint8_t ch = 1U; ch <= 4U; ++ch) {
        if (!selected(ch)) continue;
        uint32_t mv = 0U;
        if (backend->read_adc_mv(ch, &mv)) {
            char prefix[16];
            (void)snprintf(prefix, sizeof(prefix), " EX%u=", ch);
            append_u32(line, sizeof(line), prefix, mv);
            append_text(line, sizeof(line), "mV");
        }
    }
    backend->write_line(line);
}

static void print_di(const ecu_debug_monitor_backend_t *backend)
{
    char line[192] = "DI";
    for (uint8_t ch = 1U; ch <= 12U; ++ch) {
        if (!selected(ch)) continue;
        char fragment[20];
        (void)snprintf(fragment, sizeof(fragment), " EX_IN%u=%u", ch,
                       backend->read_di(ch) ? 1U : 0U);
        append_text(line, sizeof(line), fragment);
    }
    backend->write_line(line);
}

static void print_sbus(const ecu_debug_monitor_backend_t *backend)
{
    sbus_frame_t frame;
    if (!backend->read_sbus(&frame)) {
        backend->write_line("SBUS no_frame");
        return;
    }
    char line[192] = "SBUS";
    for (uint8_t ch = 1U; ch <= 16U; ++ch) {
        if (!selected(ch)) continue;
        char fragment[20];
        (void)snprintf(fragment, sizeof(fragment), " ch%u=%u", ch, frame.channels[ch - 1U]);
        append_text(line, sizeof(line), fragment);
    }
    char status[32];
    (void)snprintf(status, sizeof(status), " lost=%u failsafe=%u",
                   frame.frame_lost ? 1U : 0U, frame.failsafe ? 1U : 0U);
    append_text(line, sizeof(line), status);
    backend->write_line(line);
}

static void print_do(const ecu_debug_monitor_backend_t *backend, uint32_t applied_mask)
{
    char line[192];
    (void)snprintf(line, sizeof(line), "DO mask=0x%03lX", (unsigned long)applied_mask);
    for (uint8_t ch = 1U; ch <= 12U; ++ch) {
        if (!selected(ch)) continue;
        char fragment[20];
        (void)snprintf(fragment, sizeof(fragment), " EX_OUT%u=%u", ch,
                       ((applied_mask & (1UL << (ch - 1U))) != 0UL) ? 1U : 0U);
        append_text(line, sizeof(line), fragment);
    }
    backend->write_line(line);
}

void ecu_debug_monitor_init(void)
{
    const ecu_debug_monitor_backend_t *backend = active_backend();
    s_suspended = false;
    g_ecu_debug_monitor.enable = 0U;
    g_ecu_debug_monitor.view = ECU_DEBUG_VIEW_NONE;
    g_ecu_debug_monitor.channel = 0U;
    g_ecu_debug_monitor.period_ms = ECU_DEBUG_MONITOR_DEFAULT_PERIOD_MS;
    g_ecu_debug_monitor.do_enable = 0U;
    g_ecu_debug_monitor.do_mask = 0U;
    s_last_print_ms = (uint32_t)(0U - normalized_period_ms());
    backend->write_do_mask(0U);
}

void ecu_debug_monitor_poll(void)
{
    const ecu_debug_monitor_backend_t *backend = active_backend();
    uint32_t applied_mask = g_ecu_debug_monitor.do_enable ?
                            (g_ecu_debug_monitor.do_mask & ECU_DEBUG_MONITOR_DO_MASK_ALL) : 0U;
    backend->write_do_mask(applied_mask);
    if (s_suspended || g_ecu_debug_monitor.enable == 0U ||
        g_ecu_debug_monitor.view == ECU_DEBUG_VIEW_NONE) {
        return;
    }

    uint32_t now = backend->now_ms();
    if ((uint32_t)(now - s_last_print_ms) < normalized_period_ms()) return;
    s_last_print_ms = now;

    switch (g_ecu_debug_monitor.view) {
    case ECU_DEBUG_VIEW_SBUS:
        print_sbus(backend);
        break;
    case ECU_DEBUG_VIEW_ADC:
        print_adc(backend);
        break;
    case ECU_DEBUG_VIEW_DI:
        print_di(backend);
        break;
    case ECU_DEBUG_VIEW_DO:
        print_do(backend, applied_mask);
        break;
    case ECU_DEBUG_VIEW_ALL:
        print_sbus(backend);
        print_adc(backend);
        print_di(backend);
        print_do(backend, applied_mask);
        break;
    default:
        break;
    }
}

void ecu_debug_monitor_suspend(void)
{
    const ecu_debug_monitor_backend_t *backend = active_backend();
    s_suspended = true;
    backend->write_do_mask(0U);
}

void ecu_debug_monitor_resume(void)
{
    s_suspended = false;
}

void ecu_debug_monitor_use_backend(const ecu_debug_monitor_backend_t *backend)
{
    s_backend = backend != NULL ? backend : &s_default_backend;
}

void ecu_debug_monitor_restore_default_backend(void)
{
    s_backend = &s_default_backend;
}

static uint32_t default_now_ms(void)
{
    return 0U;
}

static bool default_read_sbus(sbus_frame_t *frame)
{
    (void)frame;
    return false;
}

static bool default_read_adc_mv(uint8_t channel, uint32_t *mv)
{
    (void)channel;
    (void)mv;
    return false;
}

static uint8_t default_read_di(uint8_t channel)
{
    (void)channel;
    return 0U;
}

static void default_write_do_mask(uint32_t mask)
{
    (void)mask;
}

static void default_write_line(const char *line)
{
    if (line != NULL) printf("%s\n", line);
}
