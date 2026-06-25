# Debug Monitor Structure Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a SEGGER-debugger-controlled monitor structure that prints selected SBUS, ADC, DI, and DO data to the UART0 test console and lets `do_mask` hold selected digital outputs on.

**Architecture:** Add `debug_monitor` as a small polled service with a volatile public control structure and a backend interface. Unit tests use a fake backend to verify view selection, print rate limiting, ADC mV formatting, DI 0/1 formatting, SBUS 16-channel formatting, and DO mask mapping before HPM6750 hardware is connected.

**Tech Stack:** C99, HPMicro SDK 1.11, SEGGER Embedded Studio 8.28 project generation, GNU embedded build scripts, existing ECU board-test selftest framework, UART0 console printing.

---

## File structure

- Create `ecu/apps/ecu_board_test/include/debug_monitor.h`: public control structure, enum values, backend interface, lifecycle functions, and test-only line building helpers.
- Create `ecu/apps/ecu_board_test/src/debug_monitor.c`: hardware-independent monitor state machine, print scheduling, SBUS byte assembly, ADC/DI/DO dispatch, and fake-backend-testable output formatting.
- Create `ecu/apps/ecu_board_test/selftest/selftest_debug_monitor.c`: selftests for the monitor logic.
- Modify `ecu/apps/ecu_board_test/include/selftest.h`: add `selftest_debug_monitor()`.
- Modify `ecu/apps/ecu_board_test/selftest/selftest.c`: run the new selftest.
- Modify `ecu/apps/ecu_board_test/CMakeLists.txt`: compile the new source and selftest source.
- Modify `ecu/apps/ecu_board_test/src/main.c`: initialize the monitor after safety initialization.
- Modify `ecu/apps/ecu_board_test/src/operator_io.c`: poll the monitor in the foreground console wait loop.
- Modify `ecu/apps/ecu_board_test/src/app_shell.c`: suspend monitor and force debug-controlled DO off around registered tests.
- Modify `docs/ecu-test-progress.md`: record build/download/COM9 evidence and bench-observation status.
- Modify `docs/ecu-board-test-code-reading-guide.md`: add the monitor module and debugger workflow.

---

### Task 1: Add test-first debug monitor API and fake-backend selftest

**Files:**
- Create: `ecu/apps/ecu_board_test/include/debug_monitor.h`
- Create: `ecu/apps/ecu_board_test/src/debug_monitor.c`
- Create: `ecu/apps/ecu_board_test/selftest/selftest_debug_monitor.c`
- Modify: `ecu/apps/ecu_board_test/include/selftest.h`
- Modify: `ecu/apps/ecu_board_test/selftest/selftest.c`
- Modify: `ecu/apps/ecu_board_test/CMakeLists.txt`

- [ ] **Step 1: Write the failing selftest**

Add this declaration to `ecu/apps/ecu_board_test/include/selftest.h`:

```c
void selftest_debug_monitor(selftest_result_t *result);
```

Add this call in `selftest_run_all()` in `ecu/apps/ecu_board_test/selftest/selftest.c`:

```c
selftest_debug_monitor(&result);
```

Add `ecu/apps/ecu_board_test/selftest/selftest_debug_monitor.c` with this complete initial test file:

```c
#include <string.h>
#include "debug_monitor.h"
#include "selftest.h"

static uint32_t fake_now_ms;
static char fake_last_line[192];
static uint32_t fake_line_count;
static uint32_t fake_do_mask;
static uint8_t fake_di[12];
static uint32_t fake_adc_mv[4];
static sbus_frame_t fake_sbus;

static uint32_t fake_now(void) { return fake_now_ms; }

static bool fake_read_sbus(sbus_frame_t *frame)
{
    if (frame == NULL) return false;
    *frame = fake_sbus;
    return true;
}

static bool fake_read_adc_mv(uint8_t channel, uint32_t *mv)
{
    if (channel < 1U || channel > 4U || mv == NULL) return false;
    *mv = fake_adc_mv[channel - 1U];
    return true;
}

static uint8_t fake_read_di(uint8_t channel)
{
    if (channel < 1U || channel > 12U) return 0U;
    return fake_di[channel - 1U];
}

static void fake_write_do_mask(uint32_t mask)
{
    fake_do_mask = mask;
}

static void fake_write_line(const char *line)
{
    ++fake_line_count;
    strncpy(fake_last_line, line, sizeof(fake_last_line) - 1U);
    fake_last_line[sizeof(fake_last_line) - 1U] = '\0';
}

static void reset_fake_backend(void)
{
    fake_now_ms = 0U;
    fake_line_count = 0U;
    fake_do_mask = 0xFFFFFFFFUL;
    memset(fake_last_line, 0, sizeof(fake_last_line));
    memset(fake_di, 0, sizeof(fake_di));
    memset(fake_adc_mv, 0, sizeof(fake_adc_mv));
    memset(&fake_sbus, 0, sizeof(fake_sbus));
    for (uint8_t i = 0U; i < 16U; ++i) {
        fake_sbus.channels[i] = (uint16_t)(1000U + i);
    }
    fake_sbus.frame_lost = true;
    fake_sbus.failsafe = false;
    fake_adc_mv[0] = 1200U;
    fake_adc_mv[1] = 2500U;
    fake_adc_mv[2] = 0U;
    fake_adc_mv[3] = 4998U;
    fake_di[0] = 1U;
    fake_di[5] = 1U;
}

static const ecu_debug_monitor_backend_t fake_backend = {
    fake_now,
    fake_read_sbus,
    fake_read_adc_mv,
    fake_read_di,
    fake_write_do_mask,
    fake_write_line
};

void selftest_debug_monitor(selftest_result_t *result)
{
    reset_fake_backend();
    ecu_debug_monitor_use_backend(&fake_backend);
    ecu_debug_monitor_init();

    SELFTEST_ASSERT_EQ(0U, g_ecu_debug_monitor.enable);
    SELFTEST_ASSERT_EQ(ECU_DEBUG_VIEW_NONE, g_ecu_debug_monitor.view);
    SELFTEST_ASSERT_EQ(200U, g_ecu_debug_monitor.period_ms);
    SELFTEST_ASSERT_EQ(0U, fake_do_mask);

    g_ecu_debug_monitor.enable = 1U;
    g_ecu_debug_monitor.view = ECU_DEBUG_VIEW_ADC;
    g_ecu_debug_monitor.period_ms = 200U;
    ecu_debug_monitor_poll();
    SELFTEST_ASSERT_EQ(1U, fake_line_count);
    SELFTEST_ASSERT_TRUE(strstr(fake_last_line, "ADC") != NULL);
    SELFTEST_ASSERT_TRUE(strstr(fake_last_line, "EX1=1200mV") != NULL);
    SELFTEST_ASSERT_TRUE(strstr(fake_last_line, "EX4=4998mV") != NULL);

    ecu_debug_monitor_poll();
    SELFTEST_ASSERT_EQ(1U, fake_line_count);
    fake_now_ms = 200U;
    g_ecu_debug_monitor.view = ECU_DEBUG_VIEW_DI;
    ecu_debug_monitor_poll();
    SELFTEST_ASSERT_EQ(2U, fake_line_count);
    SELFTEST_ASSERT_TRUE(strstr(fake_last_line, "EX_IN1=1") != NULL);
    SELFTEST_ASSERT_TRUE(strstr(fake_last_line, "EX_IN6=1") != NULL);

    fake_now_ms = 400U;
    g_ecu_debug_monitor.view = ECU_DEBUG_VIEW_SBUS;
    ecu_debug_monitor_poll();
    SELFTEST_ASSERT_TRUE(strstr(fake_last_line, "ch1=1000") != NULL);
    SELFTEST_ASSERT_TRUE(strstr(fake_last_line, "ch16=1015") != NULL);
    SELFTEST_ASSERT_TRUE(strstr(fake_last_line, "lost=1") != NULL);

    fake_now_ms = 600U;
    g_ecu_debug_monitor.view = ECU_DEBUG_VIEW_DO;
    g_ecu_debug_monitor.do_enable = 1U;
    g_ecu_debug_monitor.do_mask = 0x00000005UL;
    ecu_debug_monitor_poll();
    SELFTEST_ASSERT_EQ(0x00000005UL, fake_do_mask);
    SELFTEST_ASSERT_TRUE(strstr(fake_last_line, "mask=0x005") != NULL);
    SELFTEST_ASSERT_TRUE(strstr(fake_last_line, "EX_OUT1=1") != NULL);
    SELFTEST_ASSERT_TRUE(strstr(fake_last_line, "EX_OUT2=0") != NULL);
    SELFTEST_ASSERT_TRUE(strstr(fake_last_line, "EX_OUT3=1") != NULL);

    fake_now_ms = 800U;
    g_ecu_debug_monitor.do_enable = 0U;
    g_ecu_debug_monitor.do_mask = 0x00000FFFUL;
    ecu_debug_monitor_poll();
    SELFTEST_ASSERT_EQ(0U, fake_do_mask);

    ecu_debug_monitor_restore_default_backend();
}
```

Add these source files to `ecu/apps/ecu_board_test/CMakeLists.txt`:

```cmake
    src/debug_monitor.c
    selftest/selftest_debug_monitor.c
```

- [ ] **Step 2: Run the test build and verify RED**

Run:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\ecu\scripts\build_ecu_test.ps1
```

Expected: build fails because `debug_monitor.h`, `ecu_debug_monitor_use_backend()`, or `ecu_debug_monitor_poll()` is not implemented.

- [ ] **Step 3: Implement the minimal public header**

Create `ecu/apps/ecu_board_test/include/debug_monitor.h`:

```c
#ifndef ECU_DEBUG_MONITOR_H
#define ECU_DEBUG_MONITOR_H

#include <stdbool.h>
#include <stdint.h>
#include "sbus_decoder.h"

typedef enum {
    ECU_DEBUG_VIEW_NONE = 0,
    ECU_DEBUG_VIEW_SBUS = 1,
    ECU_DEBUG_VIEW_ADC  = 2,
    ECU_DEBUG_VIEW_DI   = 3,
    ECU_DEBUG_VIEW_DO   = 4,
    ECU_DEBUG_VIEW_ALL  = 5
} ecu_debug_view_t;

typedef struct {
    volatile uint32_t enable;
    volatile uint32_t view;
    volatile uint32_t channel;
    volatile uint32_t period_ms;
    volatile uint32_t do_enable;
    volatile uint32_t do_mask;
} ecu_debug_monitor_t;

typedef struct {
    uint32_t (*now_ms)(void);
    bool (*read_sbus)(sbus_frame_t *frame);
    bool (*read_adc_mv)(uint8_t channel, uint32_t *mv);
    uint8_t (*read_di)(uint8_t channel);
    void (*write_do_mask)(uint32_t mask);
    void (*write_line)(const char *line);
} ecu_debug_monitor_backend_t;

extern volatile ecu_debug_monitor_t g_ecu_debug_monitor;

void ecu_debug_monitor_init(void);
void ecu_debug_monitor_poll(void);
void ecu_debug_monitor_suspend(void);
void ecu_debug_monitor_resume(void);
void ecu_debug_monitor_use_backend(const ecu_debug_monitor_backend_t *backend);
void ecu_debug_monitor_restore_default_backend(void);

#endif
```

- [ ] **Step 4: Implement minimal hardware-independent monitor logic**

Create `ecu/apps/ecu_board_test/src/debug_monitor.c` with:

```c
#include <stdio.h>
#include <string.h>
#include "adc_math.h"
#include "board.h"
#include "debug_monitor.h"
#include "hpm_adc16_drv.h"
#include "hpm_uart_drv.h"
#include "safety_manager.h"
#include "test_serial_common.h"

#define ECU_DEBUG_MONITOR_DEFAULT_PERIOD_MS 200U
#define ECU_DEBUG_MONITOR_MIN_PERIOD_MS 50U
#define ECU_DEBUG_MONITOR_DO_MASK_ALL 0x00000FFFUL

volatile ecu_debug_monitor_t g_ecu_debug_monitor = {
    0U, ECU_DEBUG_VIEW_NONE, 0U, ECU_DEBUG_MONITOR_DEFAULT_PERIOD_MS, 0U, 0U
};

static const ecu_debug_monitor_backend_t *s_backend;
static bool s_suspended;
static uint32_t s_last_print_ms;

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
    if (used + 1U < capacity) {
        (void)snprintf(&line[used], capacity - used, "%s", text);
    }
}

static void append_u32(char *line, size_t capacity, const char *prefix, uint32_t value)
{
    char fragment[32];
    (void)snprintf(fragment, sizeof(fragment), "%s%lu", prefix, (unsigned long)value);
    append_text(line, capacity, fragment);
}

static void print_adc(void)
{
    char line[160] = "ADC";
    for (uint8_t ch = 1U; ch <= 4U; ++ch) {
        if (!selected(ch)) continue;
        uint32_t mv = 0U;
        if (s_backend->read_adc_mv(ch, &mv)) {
            char prefix[16];
            (void)snprintf(prefix, sizeof(prefix), " EX%u=", ch);
            append_u32(line, sizeof(line), prefix, mv);
            append_text(line, sizeof(line), "mV");
        }
    }
    s_backend->write_line(line);
}

static void print_di(void)
{
    char line[192] = "DI";
    for (uint8_t ch = 1U; ch <= 12U; ++ch) {
        if (!selected(ch)) continue;
        char fragment[20];
        (void)snprintf(fragment, sizeof(fragment), " EX_IN%u=%u", ch,
                       s_backend->read_di(ch) ? 1U : 0U);
        append_text(line, sizeof(line), fragment);
    }
    s_backend->write_line(line);
}

static void print_sbus(void)
{
    sbus_frame_t frame;
    if (!s_backend->read_sbus(&frame)) {
        s_backend->write_line("SBUS no_frame");
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
    s_backend->write_line(line);
}

static void print_do(uint32_t applied_mask)
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
    s_backend->write_line(line);
}

void ecu_debug_monitor_init(void)
{
    s_backend = &s_default_backend;
    s_suspended = false;
    s_last_print_ms = 0U;
    g_ecu_debug_monitor.enable = 0U;
    g_ecu_debug_monitor.view = ECU_DEBUG_VIEW_NONE;
    g_ecu_debug_monitor.channel = 0U;
    g_ecu_debug_monitor.period_ms = ECU_DEBUG_MONITOR_DEFAULT_PERIOD_MS;
    g_ecu_debug_monitor.do_enable = 0U;
    g_ecu_debug_monitor.do_mask = 0U;
    s_backend->write_do_mask(0U);
}

void ecu_debug_monitor_use_backend(const ecu_debug_monitor_backend_t *backend)
{
    s_backend = backend != NULL ? backend : &s_default_backend;
}

void ecu_debug_monitor_restore_default_backend(void)
{
    s_backend = &s_default_backend;
}

void ecu_debug_monitor_suspend(void)
{
    s_suspended = true;
    if (s_backend != NULL && s_backend->write_do_mask != NULL) {
        s_backend->write_do_mask(0U);
    }
}

void ecu_debug_monitor_resume(void)
{
    s_suspended = false;
}

void ecu_debug_monitor_poll(void)
{
    if (s_backend == NULL) s_backend = &s_default_backend;
    uint32_t applied_mask = g_ecu_debug_monitor.do_enable ?
                            (g_ecu_debug_monitor.do_mask & ECU_DEBUG_MONITOR_DO_MASK_ALL) : 0U;
    s_backend->write_do_mask(applied_mask);
    if (s_suspended || g_ecu_debug_monitor.enable == 0U ||
        g_ecu_debug_monitor.view == ECU_DEBUG_VIEW_NONE) {
        return;
    }
    uint32_t now = s_backend->now_ms();
    if ((uint32_t)(now - s_last_print_ms) < normalized_period_ms()) return;
    s_last_print_ms = now;

    switch (g_ecu_debug_monitor.view) {
    case ECU_DEBUG_VIEW_SBUS: print_sbus(); break;
    case ECU_DEBUG_VIEW_ADC:  print_adc(); break;
    case ECU_DEBUG_VIEW_DI:   print_di(); break;
    case ECU_DEBUG_VIEW_DO:   print_do(applied_mask); break;
    case ECU_DEBUG_VIEW_ALL:
        print_sbus();
        print_adc();
        print_di();
        print_do(applied_mask);
        break;
    default:
        break;
    }
}
```

Add stub defaults at the bottom so the unit build links before Task 2:

```c
static uint32_t default_now_ms(void) { return 0U; }
static bool default_read_sbus(sbus_frame_t *frame) { (void)frame; return false; }
static bool default_read_adc_mv(uint8_t channel, uint32_t *mv) { (void)channel; (void)mv; return false; }
static uint8_t default_read_di(uint8_t channel) { (void)channel; return 0U; }
static void default_write_do_mask(uint32_t mask) { (void)mask; }
static void default_write_line(const char *line) { printf("%s\n", line); }
```

- [ ] **Step 5: Run selftest build and verify GREEN**

Run:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\ecu\scripts\build_ecu_test.ps1
```

Expected: build succeeds and the selftest source links.

- [ ] **Step 6: Commit Task 1**

Run:

```powershell
git add ecu/apps/ecu_board_test/include/debug_monitor.h ecu/apps/ecu_board_test/src/debug_monitor.c ecu/apps/ecu_board_test/selftest/selftest_debug_monitor.c ecu/apps/ecu_board_test/include/selftest.h ecu/apps/ecu_board_test/selftest/selftest.c ecu/apps/ecu_board_test/CMakeLists.txt
git commit -m "feat: add debugger monitor core"
```

---

### Task 2: Connect HPM6750 SBUS, ADC, DI, DO, and time backend

**Files:**
- Modify: `ecu/apps/ecu_board_test/src/debug_monitor.c`

- [ ] **Step 1: Add backend-focused selftest assertions before hardware code**

Extend `selftest_debug_monitor()` with these checks after the existing DO checks:

```c
fake_now_ms = 1000U;
g_ecu_debug_monitor.view = ECU_DEBUG_VIEW_ADC;
g_ecu_debug_monitor.channel = 2U;
ecu_debug_monitor_poll();
SELFTEST_ASSERT_TRUE(strstr(fake_last_line, "EX1=") == NULL);
SELFTEST_ASSERT_TRUE(strstr(fake_last_line, "EX2=2500mV") != NULL);

fake_now_ms = 1200U;
g_ecu_debug_monitor.view = ECU_DEBUG_VIEW_DO;
g_ecu_debug_monitor.channel = 3U;
g_ecu_debug_monitor.do_enable = 1U;
g_ecu_debug_monitor.do_mask = 0x00001FFFUL;
ecu_debug_monitor_poll();
SELFTEST_ASSERT_EQ(0x00000FFFUL, fake_do_mask);
SELFTEST_ASSERT_TRUE(strstr(fake_last_line, "EX_OUT2=") == NULL);
SELFTEST_ASSERT_TRUE(strstr(fake_last_line, "EX_OUT3=1") != NULL);
```

- [ ] **Step 2: Run build and verify RED or expanded coverage**

Run:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\ecu\scripts\build_ecu_test.ps1
```

Expected: if Task 1 selection already works, build succeeds; otherwise it fails at the new assertions during target selftest later. Keep the assertions because they lock channel selection and DO mask clipping.

- [ ] **Step 3: Replace default stubs with real board backend**

In `ecu/apps/ecu_board_test/src/debug_monitor.c`, replace the default backend stubs with:

```c
static uint32_t default_now_ms(void)
{
    uint64_t ticks = mchtmr_get_count(HPM_MCHTMR);
    uint32_t freq = clock_get_frequency(clock_mchtmr0);
    if (freq == 0U) return 0U;
    return (uint32_t)((ticks * 1000ULL) / freq);
}

static bool default_read_sbus(sbus_frame_t *frame)
{
    static bool configured;
    static uint8_t data[25];
    static uint8_t position;
    if (frame == NULL) return false;
    if (!configured) {
        if (!test_uart_configure(BOARD_SBUS_UART_BASE, BOARD_SBUS_BAUDRATE,
                                 parity_even, stop_bits_2, false)) {
            return false;
        }
        configured = true;
    }
    for (uint16_t attempts = 0U; attempts < 256U; ++attempts) {
        uint8_t byte;
        if (uart_try_receive_byte(BOARD_SBUS_UART_BASE, &byte) != status_success) break;
        if (position == 0U && byte != 0x0FU) continue;
        data[position++] = byte;
        if (position == sizeof(data)) {
            position = 0U;
            return sbus_decode(data, sizeof(data), frame) == SBUS_OK;
        }
    }
    return false;
}

static bool default_read_adc_mv(uint8_t channel, uint32_t *mv)
{
    static bool configured;
    static const uint8_t adc_channels[] = {
        BOARD_ANALOG_EX1_ADC_CH, BOARD_ANALOG_EX2_ADC_CH,
        BOARD_ANALOG_EX3_ADC_CH, BOARD_ANALOG_EX4_ADC_CH
    };
    if (channel < 1U || channel > 4U || mv == NULL) return false;
    if (!configured) {
        adc16_config_t config;
        board_init_adc16_pins();
        board_init_adc_clock(BOARD_APP_ADC16_BASE, true);
        adc16_get_default_config(&config);
        config.res = adc16_res_16_bits;
        config.conv_mode = adc16_conv_mode_oneshot;
        config.adc_clk_div = adc16_clock_divider_4;
        config.sel_sync_ahb = true;
        if (adc16_init(BOARD_APP_ADC16_BASE, &config) != status_success) return false;
        configured = true;
    }
    adc16_channel_config_t channel_config;
    adc16_get_channel_default_config(&channel_config);
    channel_config.ch = adc_channels[channel - 1U];
    channel_config.sample_cycle = 20U;
    if (adc16_init_channel(BOARD_APP_ADC16_BASE, &channel_config) != status_success) return false;
    uint16_t raw;
    if (adc16_get_oneshot_result(BOARD_APP_ADC16_BASE, adc_channels[channel - 1U], &raw) != status_success) {
        return false;
    }
    *mv = adc_external_uv_from_raw(raw, 3300000U) / 1000U;
    return true;
}

static uint8_t default_read_di(uint8_t channel)
{
    if (channel < 1U || channel > BOARD_ECU_INPUT_COUNT) return 0U;
    return board_ecu_input_read(channel) ? 1U : 0U;
}

static void default_write_do_mask(uint32_t mask)
{
    for (uint8_t channel = 1U; channel <= BOARD_ECU_OUTPUT_COUNT; ++channel) {
        bool on = (mask & (1UL << (channel - 1U))) != 0UL;
        if (on) {
            (void)safety_output_on(channel);
        } else {
            safety_output_off(channel);
        }
    }
}

static void default_write_line(const char *line)
{
    if (line != NULL) printf("%s\n", line);
}
```

Also add needed includes if missing:

```c
#include "hpm_clock_drv.h"
#include "hpm_mchtmr_drv.h"
```

- [ ] **Step 4: Run build and verify GREEN**

Run:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\ecu\scripts\build_ecu_test.ps1
```

Expected: build succeeds.

- [ ] **Step 5: Commit Task 2**

Run:

```powershell
git add ecu/apps/ecu_board_test/src/debug_monitor.c ecu/apps/ecu_board_test/selftest/selftest_debug_monitor.c
git commit -m "feat: connect debug monitor hardware backend"
```

---

### Task 3: Integrate monitor with startup, foreground polling, and registered tests

**Files:**
- Modify: `ecu/apps/ecu_board_test/src/main.c`
- Modify: `ecu/apps/ecu_board_test/src/operator_io.c`
- Modify: `ecu/apps/ecu_board_test/src/app_shell.c`

- [ ] **Step 1: Write failing integration selftest**

Extend `selftest_debug_monitor()` with suspend/resume behavior:

```c
fake_now_ms = 1400U;
g_ecu_debug_monitor.enable = 1U;
g_ecu_debug_monitor.view = ECU_DEBUG_VIEW_DI;
g_ecu_debug_monitor.do_enable = 1U;
g_ecu_debug_monitor.do_mask = 0x00000003UL;
ecu_debug_monitor_suspend();
ecu_debug_monitor_poll();
SELFTEST_ASSERT_EQ(0U, fake_do_mask);
uint32_t lines_before_resume = fake_line_count;
fake_now_ms = 1600U;
ecu_debug_monitor_poll();
SELFTEST_ASSERT_EQ(lines_before_resume, fake_line_count);
ecu_debug_monitor_resume();
fake_now_ms = 1800U;
ecu_debug_monitor_poll();
SELFTEST_ASSERT_TRUE(fake_line_count > lines_before_resume);
```

- [ ] **Step 2: Run build**

Run:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\ecu\scripts\build_ecu_test.ps1
```

Expected: build succeeds if suspend/resume from Task 1 already satisfies this; otherwise fix `ecu_debug_monitor_suspend()` and `ecu_debug_monitor_resume()` until it does.

- [ ] **Step 3: Initialize monitor in `main.c`**

Add include:

```c
#include "debug_monitor.h"
```

Call after `safety_init();`:

```c
ecu_debug_monitor_init();
```

- [ ] **Step 4: Poll monitor in `operator_io.c`**

Add include:

```c
#include "debug_monitor.h"
```

In `operator_read_line()`, poll beside the existing LED and periodic transmitter:

```c
status_led_poll();
periodic_tx_poll();
ecu_debug_monitor_poll();
```

- [ ] **Step 5: Suspend around registered tests in `app_shell.c`**

Add include:

```c
#include "debug_monitor.h"
```

In the registered-test execution path, call:

```c
ecu_debug_monitor_suspend();
```

before `test_runner_execute(...)`, and call:

```c
ecu_debug_monitor_resume();
```

after cleanup is complete. For `SELFTEST.ALL`, suspend before `selftest_run_all()` and then call:

```c
ecu_debug_monitor_init();
```

after selftests, because selftests install a fake debug-monitor backend.

- [ ] **Step 6: Run build and verify GREEN**

Run:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\ecu\scripts\build_ecu_test.ps1
```

Expected: build succeeds.

- [ ] **Step 7: Commit Task 3**

Run:

```powershell
git add ecu/apps/ecu_board_test/src/main.c ecu/apps/ecu_board_test/src/operator_io.c ecu/apps/ecu_board_test/src/app_shell.c ecu/apps/ecu_board_test/selftest/selftest_debug_monitor.c
git commit -m "feat: integrate debugger monitor service"
```

---

### Task 4: Build, download, COM9 verification, and documentation

**Files:**
- Modify: `docs/ecu-test-progress.md`
- Modify: `docs/ecu-board-test-code-reading-guide.md`

- [ ] **Step 1: Run diff hygiene**

Run:

```powershell
git diff --check
```

Expected: no output and exit code 0.

- [ ] **Step 2: Run default GNU build**

Run:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\ecu\scripts\build_ecu_test.ps1
```

Expected: build succeeds and writes `tmp/ecu_board_test_build/output/demo.elf`.

- [ ] **Step 3: Run SES generation/build check**

Run:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\ecu\scripts\generate_ses_ecu_test.ps1
```

If `emBuild.exe` is available, run the generated SES Debug build command used elsewhere in the project notes. Expected: exit code 0.

- [ ] **Step 4: Download with J-Link**

Run:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\ecu\scripts\flash_ecu_test.ps1
```

Expected: J-Link reports target voltage, device identity, and flash download success.

- [ ] **Step 5: Verify COM9 selftest**

Open COM9 with the existing script:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\ecu\scripts\serial_console.ps1 -Port COM9
```

Send:

```text
SELFTEST.ALL
```

Expected summary includes:

```text
SELFTEST debug_monitor PASS
SELFTEST SUMMARY
```

- [ ] **Step 6: Verify one non-destructive monitor output**

With the board halted or live in SEGGER, set:

```c
g_ecu_debug_monitor.enable = 1
g_ecu_debug_monitor.view = ECU_DEBUG_VIEW_DI
g_ecu_debug_monitor.channel = 0
g_ecu_debug_monitor.period_ms = 200
g_ecu_debug_monitor.do_enable = 0
g_ecu_debug_monitor.do_mask = 0
```

Expected COM9 output periodically contains:

```text
DI EX_IN1=
```

The exact 0/1 values depend on connected inputs.

- [ ] **Step 7: Update docs**

Add to `docs/ecu-board-test-code-reading-guide.md`:

```markdown
| `ecu/apps/ecu_board_test/include/debug_monitor.h` | SEGGER debugger-editable monitor structure | `g_ecu_debug_monitor` selects SBUS/ADC/DI/DO printing and DO mask control |
| `ecu/apps/ecu_board_test/src/debug_monitor.c` | Polled monitor service and board backend | ADC prints mV, DI prints 0/1, SBUS prints 16 channels, DO follows `do_mask` when enabled |
```

Add to `docs/ecu-test-progress.md`:

```markdown
- 2026-06-25：新增 SEGGER 调试结构体 `g_ecu_debug_monitor`，可选择打印 SBUS、ADC、DI、DO；ADC 单位为 mV，DI/DO 为 0/1，DO 由 `do_enable` 和 `do_mask` 保持控制。
```

- [ ] **Step 8: Commit Task 4**

Run:

```powershell
git add docs/ecu-test-progress.md docs/ecu-board-test-code-reading-guide.md
git commit -m "docs: record debugger monitor workflow"
```

---

## Self-review checklist

- Spec coverage:
  - Public `g_ecu_debug_monitor` structure: Task 1.
  - SBUS 16-channel output: Task 1 and Task 2.
  - ADC mV output: Task 1 and Task 2.
  - DI 0/1 output: Task 1 and Task 2.
  - DO held output from `do_mask`: Task 1 and Task 2.
  - Suspend around registered tests: Task 3.
  - Build/download/COM9 evidence and documentation: Task 4.
- Incomplete-marker scan: the plan contains no deferred markers, deferred implementation notes, or unspecified code steps.
- Type consistency: names are `ecu_debug_monitor_t`, `ecu_debug_monitor_backend_t`, `g_ecu_debug_monitor`, and `ECU_DEBUG_VIEW_*` throughout.
