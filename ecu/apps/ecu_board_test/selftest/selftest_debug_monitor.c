#include <string.h>

#include "debug_monitor.h"
#include "selftest.h"

static uint32_t fake_now_ms;
static char fake_last_line[384];
static uint32_t fake_line_count;
static uint32_t fake_do_mask;
static uint8_t fake_di[12];
static uint32_t fake_adc_mv[4];
static uint8_t fake_adc_valid[4];
static sbus_debug_state_t fake_sbus;

static uint32_t fake_now(void)
{
    return fake_now_ms;
}

static bool fake_read_sbus_state(sbus_debug_state_t *state)
{
    if (state == NULL) return false;
    *state = fake_sbus;
    return true;
}

static bool fake_read_adc_mv(uint8_t channel, uint32_t *mv)
{
    if (channel < 1U || channel > 4U || mv == NULL) return false;
    if (fake_adc_valid[channel - 1U] == 0U) return false;
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
    memset(fake_adc_valid, 1, sizeof(fake_adc_valid));
    memset(&fake_sbus, 0, sizeof(fake_sbus));
    fake_sbus.connected = 1U;
    fake_sbus.frame_count = 42U;
    fake_sbus.last_frame_ms = 1234U;
    for (uint8_t i = 0U; i < 16U; ++i) {
        fake_sbus.channels[i] = (uint16_t)(1000U + i);
    }
    fake_sbus.frame_lost = 1U;
    fake_sbus.failsafe = 0U;
    fake_adc_mv[0] = 1200U;
    fake_adc_mv[1] = 2500U;
    fake_adc_mv[2] = 0U;
    fake_adc_mv[3] = 4998U;
    fake_adc_valid[2] = 0U;
    fake_di[0] = 1U;
    fake_di[5] = 1U;
}

static const ecu_debug_monitor_backend_t fake_backend = {
    fake_now,
    fake_read_sbus_state,
    fake_read_adc_mv,
    fake_read_di,
    fake_write_do_mask,
    fake_write_line
};

bool selftest_debug_monitor(void)
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
    SELFTEST_ASSERT_TRUE(strstr(fake_last_line, "EX3=ERR") != NULL);
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
    SELFTEST_ASSERT_TRUE(strstr(fake_last_line, "connected=1") != NULL);
    SELFTEST_ASSERT_TRUE(strstr(fake_last_line, "frames=42") != NULL);
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

    ecu_debug_monitor_restore_default_backend();
    return true;
}
