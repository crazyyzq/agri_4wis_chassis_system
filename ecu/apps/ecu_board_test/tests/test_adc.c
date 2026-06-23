#include <stdio.h>
#include "adc_math.h"
#include "board.h"
#include "hpm_adc16_drv.h"
#include "operator_io.h"
#include "test_cases.h"

static bool adc_sample_channel(uint8_t channel, uint32_t *mean, uint16_t *minimum,
                               uint16_t *maximum)
{
    adc16_channel_config_t channel_config;
    adc16_get_channel_default_config(&channel_config);
    channel_config.ch = channel;
    channel_config.sample_cycle = 20U;
    if (adc16_init_channel(BOARD_APP_ADC16_BASE, &channel_config) != status_success) return false;
    uint32_t sum = 0U;
    *minimum = UINT16_MAX;
    *maximum = 0U;
    for (uint16_t i = 0U; i < 128U; ++i) {
        uint16_t value;
        /* Blocking one-shot mode avoids treating conversion-in-progress as a fault. */
        if (adc16_get_oneshot_result(BOARD_APP_ADC16_BASE, channel, &value) != status_success) return false;
        sum += value;
        if (value < *minimum) *minimum = value;
        if (value > *maximum) *maximum = value;
    }
    *mean = sum / 128U;
    return true;
}

test_status_t test_adc(test_context_t *context)
{
    (void)context;
    static const uint8_t channels[] = {
        BOARD_ANALOG_EX1_ADC_CH, BOARD_ANALOG_EX2_ADC_CH,
        BOARD_ANALOG_EX3_ADC_CH, BOARD_ANALOG_EX4_ADC_CH
    };
    static const uint32_t points_uv[] = { 0U, 500000U, 2500000U, 5000000U };
    adc16_config_t config;
    board_init_adc16_pins();
    board_init_adc_clock(BOARD_APP_ADC16_BASE, true);
    adc16_get_default_config(&config);
    config.res = adc16_res_16_bits;
    config.conv_mode = adc16_conv_mode_oneshot;
    config.adc_clk_div = adc16_clock_divider_4;
    config.sel_sync_ahb = true;
    if (adc16_init(BOARD_APP_ADC16_BASE, &config) != status_success) return TEST_FAIL;

    for (uint8_t input = 0U; input < 4U; ++input) {
        uint32_t previous = 0U;
        for (uint8_t point = 0U; point < 4U; ++point) {
            char prompt[80];
            snprintf(prompt, sizeof(prompt), "Apply %lu uV to EX%u",
                     (unsigned long)points_uv[point], input + 1U);
            if (!operator_confirm(prompt)) return TEST_BLOCKED;
            uint32_t mean_raw;
            uint16_t min_raw, max_raw;
            if (!adc_sample_channel(channels[input], &mean_raw, &min_raw, &max_raw)) return TEST_FAIL;
            uint32_t measured_uv = adc_external_uv_from_raw((uint16_t)mean_raw, 3300000U);
            printf("ADC.EX%u expected_uV=%lu measured_uV=%lu raw_mean=%lu raw_min=%u raw_max=%u p2p=%u\n",
                   input + 1U, (unsigned long)points_uv[point], (unsigned long)measured_uv,
                   (unsigned long)mean_raw, min_raw, max_raw, max_raw - min_raw);
            if (!adc_external_in_tolerance(measured_uv, points_uv[point])) return TEST_FAIL;
            if (point > 0U && measured_uv <= previous) return TEST_FAIL;
            previous = measured_uv;
        }
    }
    return TEST_PASS;
}
