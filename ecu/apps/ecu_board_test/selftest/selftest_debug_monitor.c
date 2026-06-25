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

static uint32_t fake_now(void)
{
    return fake_now_ms;
}

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
    return true;
}
