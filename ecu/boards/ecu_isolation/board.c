/*
 * Copyright (c) 2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Board support file for ECU_Isolation board
 * MCU module: ZLG MR/MB6750-WB32F8AWI-L
 */

#include "board.h"
#include "pinmux.h"

#include "hpm_adc16_drv.h"
#include "hpm_clock_drv.h"
#include "hpm_gpio_drv.h"
#include "hpm_gpiom_drv.h"
#include "hpm_gptmr_drv.h"
#include "hpm_femc_drv.h"
#include "hpm_i2c_drv.h"
#include "hpm_pcfg_drv.h"
#include "hpm_pllctl_drv.h"
#include "hpm_pmp_drv.h"
#include "hpm_sysctl_drv.h"
#include "hpm_uart_drv.h"
#include "hpm_enet_drv.h"
#include "hpm_sdk_version.h"

#if defined(INIT_EXT_RAM_FOR_DATA)
extern void init_femc_pins(void);
#endif

#if defined(FLASH_XIP) && FLASH_XIP
/* Keep the same XIP option style as the official HPM6750 EVK file.
 * If your MR6750 module uses a different Flash model, replace this with the
 * Flash option recommended by the module vendor.
 */
__attribute__((section(".nor_cfg_option"), used)) const uint32_t option[4] = {
    0xfcf90002, 0x00000007, 0x0000000E, 0x00000000
};
#endif

#if defined(FLASH_UF2) && FLASH_UF2
ATTR_PLACE_AT(".uf2_signature") __attribute__((used)) const uint32_t uf2_signature = BOARD_UF2_SIGNATURE;
#endif

typedef struct {
    uint32_t assign;
    uint32_t oe_index;
    uint32_t do_index;
    uint32_t di_index;
    uint32_t pin;
} board_gpio_desc_t;

static const board_gpio_desc_t s_ecu_outputs[BOARD_ECU_OUTPUT_COUNT] = {
    /* EX_OUT1  */ { GPIOM_ASSIGN_GPIOA, GPIO_OE_GPIOA, GPIO_DO_GPIOA, GPIO_DI_GPIOA, 13U },
    /* EX_OUT2  */ { GPIOM_ASSIGN_GPIOA, GPIO_OE_GPIOA, GPIO_DO_GPIOA, GPIO_DI_GPIOA, 4U  },
    /* EX_OUT3  */ { GPIOM_ASSIGN_GPIOA, GPIO_OE_GPIOA, GPIO_DO_GPIOA, GPIO_DI_GPIOA, 11U },
    /* EX_OUT4  */ { GPIOM_ASSIGN_GPIOA, GPIO_OE_GPIOA, GPIO_DO_GPIOA, GPIO_DI_GPIOA, 0U  },
    /* EX_OUT5  */ { GPIOM_ASSIGN_GPIOA, GPIO_OE_GPIOA, GPIO_DO_GPIOA, GPIO_DI_GPIOA, 15U },
    /* EX_OUT6  */ { GPIOM_ASSIGN_GPIOA, GPIO_OE_GPIOA, GPIO_DO_GPIOA, GPIO_DI_GPIOA, 9U  },
    /* EX_OUT7  */ { GPIOM_ASSIGN_GPIOA, GPIO_OE_GPIOA, GPIO_DO_GPIOA, GPIO_DI_GPIOA, 8U  },
    /* EX_OUT8  */ { GPIOM_ASSIGN_GPIOA, GPIO_OE_GPIOA, GPIO_DO_GPIOA, GPIO_DI_GPIOA, 3U  },
    /* EX_OUT9  */ { GPIOM_ASSIGN_GPIOA, GPIO_OE_GPIOA, GPIO_DO_GPIOA, GPIO_DI_GPIOA, 14U },
    /* EX_OUT10 */ { GPIOM_ASSIGN_GPIOA, GPIO_OE_GPIOA, GPIO_DO_GPIOA, GPIO_DI_GPIOA, 7U  },
    /* EX_OUT11 */ { GPIOM_ASSIGN_GPIOZ, GPIO_OE_GPIOZ, GPIO_DO_GPIOZ, GPIO_DI_GPIOZ, 0U  },
    /* EX_OUT12 */ { GPIOM_ASSIGN_GPIOZ, GPIO_OE_GPIOZ, GPIO_DO_GPIOZ, GPIO_DI_GPIOZ, 2U  },
};

static const board_gpio_desc_t s_ecu_inputs[BOARD_ECU_INPUT_COUNT] = {
    /* EX_IN1  */ { GPIOM_ASSIGN_GPIOA, GPIO_OE_GPIOA, GPIO_DO_GPIOA, GPIO_DI_GPIOA, 28U },
    /* EX_IN2  */ { GPIOM_ASSIGN_GPIOA, GPIO_OE_GPIOA, GPIO_DO_GPIOA, GPIO_DI_GPIOA, 25U },
    /* EX_IN3  */ { GPIOM_ASSIGN_GPIOA, GPIO_OE_GPIOA, GPIO_DO_GPIOA, GPIO_DI_GPIOA, 27U },
    /* EX_IN4  */ { GPIOM_ASSIGN_GPIOA, GPIO_OE_GPIOA, GPIO_DO_GPIOA, GPIO_DI_GPIOA, 24U },
    /* EX_IN5  */ { GPIOM_ASSIGN_GPIOB, GPIO_OE_GPIOB, GPIO_DO_GPIOB, GPIO_DI_GPIOB, 5U  },
    /* EX_IN6  */ { GPIOM_ASSIGN_GPIOB, GPIO_OE_GPIOB, GPIO_DO_GPIOB, GPIO_DI_GPIOB, 7U  },
    /* EX_IN7  */ { GPIOM_ASSIGN_GPIOA, GPIO_OE_GPIOA, GPIO_DO_GPIOA, GPIO_DI_GPIOA, 5U  },
    /* EX_IN8  */ { GPIOM_ASSIGN_GPIOB, GPIO_OE_GPIOB, GPIO_DO_GPIOB, GPIO_DI_GPIOB, 2U  },
    /* EX_IN9  */ { GPIOM_ASSIGN_GPIOA, GPIO_OE_GPIOA, GPIO_DO_GPIOA, GPIO_DI_GPIOA, 6U  },
    /* EX_IN10 */ { GPIOM_ASSIGN_GPIOA, GPIO_OE_GPIOA, GPIO_DO_GPIOA, GPIO_DI_GPIOA, 16U },
    /* EX_IN11 */ { GPIOM_ASSIGN_GPIOA, GPIO_OE_GPIOA, GPIO_DO_GPIOA, GPIO_DI_GPIOA, 10U },
    /* EX_IN12 */ { GPIOM_ASSIGN_GPIOA, GPIO_OE_GPIOA, GPIO_DO_GPIOA, GPIO_DI_GPIOA, 12U },
};

static void board_gpio_output_initial(const board_gpio_desc_t *gpio, uint8_t level)
{
    gpiom_set_pin_controller(HPM_GPIOM, gpio->assign, gpio->pin, gpiom_soc_gpio0);
    gpio_set_pin_output(HPM_GPIO0, gpio->oe_index, gpio->pin);
    gpio_write_pin(HPM_GPIO0, gpio->do_index, gpio->pin, level);
}

static void board_gpio_input(const board_gpio_desc_t *gpio)
{
    gpiom_set_pin_controller(HPM_GPIOM, gpio->assign, gpio->pin, gpiom_soc_gpio0);
    gpio_set_pin_input(HPM_GPIO0, gpio->oe_index, gpio->pin);
}

void board_delay_ms(uint32_t ms)
{
    clock_cpu_delay_ms(ms);
}

void board_delay_us(uint32_t us)
{
    clock_cpu_delay_us(us);
}

void board_print_clock_freq(void)
{
    printf("==============================\n");
    printf(" %s clock summary\n", BOARD_NAME);
    printf("==============================\n");
    printf("cpu0:\t\t %luHz\n", clock_get_frequency(clock_cpu0));
    printf("cpu1:\t\t %luHz\n", clock_get_frequency(clock_cpu1));
    printf("axi0:\t\t %luHz\n", clock_get_frequency(clock_axi0));
    printf("axi1:\t\t %luHz\n", clock_get_frequency(clock_axi1));
    printf("axi2:\t\t %luHz\n", clock_get_frequency(clock_axi2));
    printf("ahb:\t\t %luHz\n", clock_get_frequency(clock_ahb));
    printf("mchtmr0:\t %luHz\n", clock_get_frequency(clock_mchtmr0));
    printf("mchtmr1:\t %luHz\n", clock_get_frequency(clock_mchtmr1));
    printf("xpi0:\t\t %luHz\n", clock_get_frequency(clock_xpi0));
    printf("xpi1:\t\t %luHz\n", clock_get_frequency(clock_xpi1));
    printf("==============================\n");
}

void board_print_banner(void)
{
#ifdef SDK_VERSION_STRING
    printf("hpm_sdk: %s\n", SDK_VERSION_STRING);
#endif
    printf("Board: %s\n", BOARD_NAME);
}

void board_init_console(void)
{
#if !defined(CONFIG_NDEBUG_CONSOLE) || !CONFIG_NDEBUG_CONSOLE
#if BOARD_CONSOLE_TYPE == CONSOLE_TYPE_UART
    console_config_t cfg;

    init_console_uart_pins();
    clock_add_to_group(BOARD_CONSOLE_UART_CLK_NAME, BOARD_RUNNING_CORE & 0x1);

    cfg.type = BOARD_CONSOLE_TYPE;
    cfg.base = (uint32_t) BOARD_CONSOLE_UART_BASE;
    cfg.src_freq_in_hz = clock_get_frequency(BOARD_CONSOLE_UART_CLK_NAME);
    cfg.baudrate = BOARD_CONSOLE_UART_BAUDRATE;

    if (status_success != console_init(&cfg)) {
        while (1) {
        }
    }
#else
    while (1) {
    }
#endif
#endif
}

void board_init_clock(void)
{
    uint32_t cpu0_freq = clock_get_frequency(clock_cpu0);

    if (cpu0_freq == PLLCTL_SOC_PLL_REFCLK_FREQ) {
        pllctl_xtal_set_rampup_time(HPM_PLLCTL, 32UL * 1000UL * 9U);
        sysctl_clock_set_preset(HPM_SYSCTL, sysctl_preset_1);
    }

    clock_add_to_group(clock_cpu0, 0);
    clock_add_to_group(clock_mchtmr0, 0);
    clock_add_to_group(clock_axi0, 0);
    clock_add_to_group(clock_axi1, 0);
    clock_add_to_group(clock_axi2, 0);
    clock_add_to_group(clock_ahb, 0);
    clock_add_to_group(clock_xdma, 0);
    clock_add_to_group(clock_hdma, 0);
    clock_add_to_group(clock_xpi0, 0);
    clock_add_to_group(clock_xpi1, 0);
    clock_add_to_group(clock_ram0, 0);
    clock_add_to_group(clock_ram1, 0);
    clock_add_to_group(clock_lmm0, 0);
    clock_add_to_group(clock_lmm1, 0);
    clock_add_to_group(clock_gpio, 0);
    clock_add_to_group(clock_mot0, 0);
    clock_add_to_group(clock_mot1, 0);
    clock_add_to_group(clock_mot2, 0);
    clock_add_to_group(clock_mot3, 0);
    clock_add_to_group(clock_synt, 0);
    clock_add_to_group(clock_ptpc, 0);
    clock_connect_group_to_cpu(0, 0);

    clock_add_to_group(clock_cpu1, 1);
    clock_add_to_group(clock_mchtmr1, 1);
    clock_connect_group_to_cpu(1, 1);

    pcfg_dcdc_set_voltage(HPM_PCFG, 1275);
    pcfg_dcdc_switch_to_dcm_mode(HPM_PCFG);

    if (status_success != pllctl_init_int_pll_with_freq(HPM_PLLCTL, 0, BOARD_CPU_FREQ)) {
        printf("Failed to set pll0_clk0 to %ldHz\n", BOARD_CPU_FREQ);
        while (1) {
        }
    }

    clock_set_source_divider(clock_cpu0, clk_src_pll0_clk0, 1);
    clock_set_source_divider(clock_cpu1, clk_src_pll0_clk0, 1);
    clock_update_core_clock();

    clock_set_source_divider(clock_ahb, clk_src_pll1_clk1, 2); /* 200MHz */
    clock_set_source_divider(clock_mchtmr0, clk_src_osc24m, 1);
    clock_set_source_divider(clock_mchtmr1, clk_src_osc24m, 1);
}

void board_init_pmp(void)
{
    uint32_t start_addr;
    uint32_t end_addr;
    uint32_t length;
    pmp_entry_t pmp_entry[16] = {0};
    uint8_t index = 0;

    extern uint32_t __noncacheable_start__[];
    extern uint32_t __noncacheable_end__[];
    start_addr = (uint32_t) __noncacheable_start__;
    end_addr = (uint32_t) __noncacheable_end__;
    length = end_addr - start_addr;
    if (length > 0) {
        assert((length & (length - 1U)) == 0U);
        assert((start_addr & (length - 1U)) == 0U);
        pmp_entry[index].pmp_addr = PMP_NAPOT_ADDR(start_addr, length);
        pmp_entry[index].pmp_cfg.val = PMP_CFG(READ_EN, WRITE_EN, EXECUTE_EN, ADDR_MATCH_NAPOT, REG_UNLOCK);
        pmp_entry[index].pma_addr = PMA_NAPOT_ADDR(start_addr, length);
        pmp_entry[index].pma_cfg.val = PMA_CFG(ADDR_MATCH_NAPOT, MEM_TYPE_MEM_NON_CACHE_BUF, AMO_EN);
        index++;
    }

    extern uint32_t __share_mem_start__[];
    extern uint32_t __share_mem_end__[];
    start_addr = (uint32_t) __share_mem_start__;
    end_addr = (uint32_t) __share_mem_end__;
    length = end_addr - start_addr;
    if (length > 0) {
        assert((length & (length - 1U)) == 0U);
        assert((start_addr & (length - 1U)) == 0U);
        pmp_entry[index].pmp_addr = PMP_NAPOT_ADDR(start_addr, length);
        pmp_entry[index].pmp_cfg.val = PMP_CFG(READ_EN, WRITE_EN, EXECUTE_EN, ADDR_MATCH_NAPOT, REG_UNLOCK);
        pmp_entry[index].pma_addr = PMA_NAPOT_ADDR(start_addr, length);
        pmp_entry[index].pma_cfg.val = PMA_CFG(ADDR_MATCH_NAPOT, MEM_TYPE_MEM_NON_CACHE_BUF, AMO_EN);
        index++;
    }

    pmp_config(&pmp_entry[0], index);
}

void board_init_safe_gpio(void)
{
    init_ecu_output_gpio_pins();
    init_ecu_input_gpio_pins();
    init_can_term_gpio_pins();
    init_rgb_led_gpio_pins();

    /* Re-apply safe states after pinmux tool generated default GPIO writes. */
    for (uint8_t i = 0; i < BOARD_ECU_OUTPUT_COUNT; i++) {
        board_gpio_output_initial(&s_ecu_outputs[i], BOARD_ECU_OUTPUT_OFF_LEVEL);
    }

    for (uint8_t i = 0; i < BOARD_ECU_INPUT_COUNT; i++) {
        board_gpio_input(&s_ecu_inputs[i]);
    }

    /* Disable all CAN termination by default. Enable only on bus end nodes. */
    board_set_can_termination(1, false);
    board_set_can_termination(2, false);
    board_set_can_termination(3, false);
    board_set_can_termination(4, false);

    board_rgb_write(BOARD_RGB_RED, 0);
    board_rgb_write(BOARD_RGB_GREEN, 0);
    board_rgb_write(BOARD_RGB_BLUE, 0);
}

void board_init(void)
{
    board_init_clock();
    board_init_console();
    board_init_safe_gpio();
    board_init_pmp();
#if BOARD_SHOW_CLOCK
    board_print_clock_freq();
#endif
#if BOARD_SHOW_BANNER
    board_print_banner();
#endif
}

void board_init_core1(void)
{
    clock_update_core_clock();
    board_init_console();
    board_init_pmp();
}


#if defined(INIT_EXT_RAM_FOR_DATA)
void board_init_sdram_pins(void)
{
    /*
     * MR6750 module has SDRAM connected internally to HPM6750 FEMC pins.
     * You still need pinmux.c to provide init_femc_pins().
     * Generate it with HPMicro Pinmux Tool by enabling FEMC SDRAM pins.
     */
    init_femc_pins();
}

uint32_t board_init_femc_clock(void)
{
    /* Same style as official HPM6750 EVKMINI: FEMC = PLL2_CLK0 / 2, about 166 MHz. */
    clock_add_to_group(clock_femc, BOARD_RUNNING_CORE & 0x1);
    clock_set_source_divider(clock_femc, clk_src_pll2_clk0, 2U);
    return clock_get_frequency(clock_femc);
}

void _init_ext_ram(void)
{
    femc_config_t config;
    femc_sdram_config_t sdram_config;
    uint32_t femc_clk_in_hz;

    board_init_sdram_pins();
    femc_clk_in_hz = board_init_femc_clock();

    femc_default_config(HPM_FEMC, &config);
    femc_init(HPM_FEMC, &config);

    femc_get_typical_sdram_config(HPM_FEMC, &sdram_config);

    sdram_config.cs = BOARD_SDRAM_CS;
    sdram_config.base_address = BOARD_SDRAM_ADDRESS;
    sdram_config.size_in_byte = BOARD_SDRAM_SIZE;
    sdram_config.port_size = BOARD_SDRAM_PORT_SIZE;
    sdram_config.col_addr_bits = BOARD_SDRAM_COLUMN_ADDR_BITS;
    sdram_config.refresh_count = BOARD_SDRAM_REFRESH_COUNT;
    sdram_config.refresh_in_ms = BOARD_SDRAM_REFRESH_IN_MS;

    sdram_config.bank_num = FEMC_SDRAM_BANK_NUM_4;
    sdram_config.cas_latency = FEMC_SDRAM_CAS_LATENCY_3;
    sdram_config.burst_len_in_byte = 8U;
    sdram_config.prescaler = 0x3U;
    sdram_config.auto_refresh_count_in_one_burst = 1U;
    sdram_config.auto_refresh_cmd_count = 8U;

    /* Keep delay cell disabled for the first bring-up. Adjust later if SDRAM test is unstable. */
    sdram_config.delay_cell_disable = true;
    sdram_config.delay_cell_value = 0U;

    if (femc_config_sdram(HPM_FEMC, femc_clk_in_hz, &sdram_config) != status_success) {
        while (1) {
        }
    }
}
#endif

void board_init_gpio_pins(void)
{
    board_init_safe_gpio();
}

void board_init_led_pins(void)
{
    init_rgb_led_gpio_pins();
    board_rgb_write(BOARD_RGB_RED, 0);
    board_rgb_write(BOARD_RGB_GREEN, 0);
    board_rgb_write(BOARD_RGB_BLUE, 0);
}

void board_rgb_write(uint8_t color, uint8_t on)
{
    uint8_t level = on ? BOARD_LED_ON_LEVEL : BOARD_LED_OFF_LEVEL;

    switch (color) {
    case BOARD_RGB_RED:
        gpio_write_pin(BOARD_R_GPIO_CTRL, BOARD_R_GPIO_INDEX, BOARD_R_GPIO_PIN, level);
        break;
    case BOARD_RGB_GREEN:
        gpio_write_pin(BOARD_G_GPIO_CTRL, BOARD_G_GPIO_INDEX, BOARD_G_GPIO_PIN, level);
        break;
    case BOARD_RGB_BLUE:
        gpio_write_pin(BOARD_B_GPIO_CTRL, BOARD_B_GPIO_INDEX, BOARD_B_GPIO_PIN, level);
        break;
    default:
        break;
    }
}

void board_led_write(uint8_t state)
{
    gpio_write_pin(BOARD_LED_GPIO_CTRL, BOARD_LED_GPIO_INDEX, BOARD_LED_GPIO_PIN, state);
}

void board_led_toggle(void)
{
    uint8_t state = gpio_read_pin(BOARD_G_GPIO_CTRL, GPIO_DI_GPIOE, BOARD_G_GPIO_PIN);
    gpio_write_pin(BOARD_G_GPIO_CTRL, BOARD_G_GPIO_INDEX, BOARD_G_GPIO_PIN, !state);
}

void board_ecu_output_write(uint8_t index, uint8_t on)
{
    if ((index == 0U) || (index > BOARD_ECU_OUTPUT_COUNT)) {
        return;
    }
    gpio_write_pin(HPM_GPIO0, s_ecu_outputs[index - 1U].do_index, s_ecu_outputs[index - 1U].pin,
                   on ? BOARD_ECU_OUTPUT_ON_LEVEL : BOARD_ECU_OUTPUT_OFF_LEVEL);
}

uint8_t board_ecu_input_read(uint8_t index)
{
    if ((index == 0U) || (index > BOARD_ECU_INPUT_COUNT)) {
        return 0U;
    }

    uint8_t raw = gpio_read_pin(HPM_GPIO0, s_ecu_inputs[index - 1U].di_index, s_ecu_inputs[index - 1U].pin);
    return (raw == BOARD_ECU_INPUT_ACTIVE_LEVEL) ? 1U : 0U;
}

uint32_t board_init_uart_clock(UART_Type *ptr)
{
    clock_name_t clk = clock_uart0;

    if (ptr == HPM_UART0) {
        clk = clock_uart0;
    } else if (ptr == HPM_UART1) {
        clk = clock_uart1;
    } else if (ptr == HPM_UART5) {
        clk = clock_uart5;
    } else if (ptr == HPM_UART10) {
        clk = clock_uart10;
    } else if (ptr == HPM_UART11) {
        clk = clock_uart11;
    } else if (ptr == HPM_UART12) {
        clk = clock_uart12;
    } else if (ptr == HPM_UART13) {
        clk = clock_uart13;
    } else if (ptr == HPM_UART14) {
        clk = clock_uart14;
    } else if (ptr == HPM_UART15) {
        clk = clock_uart15;
    } else {
        return 0U;
    }

    clock_add_to_group(clk, BOARD_RUNNING_CORE & 0x1);
    return clock_get_frequency(clk);
}

void board_init_uart(UART_Type *ptr)
{
    if (ptr == HPM_UART0) {
        init_console_uart_pins();
    } else if (ptr == HPM_UART1) {
        init_sbus_pins();
    } else if ((ptr == HPM_UART10) || (ptr == HPM_UART11) || (ptr == HPM_UART12)) {
        init_rs485_pins();
    } else if ((ptr == HPM_UART5) || (ptr == HPM_UART13) || (ptr == HPM_UART14) || (ptr == HPM_UART15)) {
        init_rs232_pins();
    }

    board_init_uart_clock(ptr);
}

uint32_t board_init_i2c_clock(I2C_Type *ptr)
{
    uint32_t freq = 0U;

    if (ptr == HPM_I2C3) {
        clock_add_to_group(clock_i2c3, BOARD_RUNNING_CORE & 0x1);
        freq = clock_get_frequency(clock_i2c3);
    }

    return freq;
}

void board_init_i2c(I2C_Type *ptr)
{
    i2c_config_t config;
    hpm_stat_t stat;
    uint32_t freq;

    if (ptr == HPM_I2C3) {
        init_eeprom_i2c_pins();
    }

    freq = board_init_i2c_clock(ptr);
    config.i2c_mode = i2c_mode_normal;
    config.is_10bit_addressing = false;
    stat = i2c_init_master(ptr, freq, &config);
    if (stat != status_success) {
        printf("failed to initialize i2c 0x%lx\n", (uint32_t) ptr);
        while (1) {
        }
    }
}

void board_init_can(CAN_Type *ptr)
{
    (void) ptr;
    init_can_pins();
    board_init_can_clock(ptr);
}

uint32_t board_init_can_clock(CAN_Type *ptr)
{
    clock_name_t clk;

    if (ptr == HPM_CAN0) {
        clk = clock_can0;
    } else if (ptr == HPM_CAN1) {
        clk = clock_can1;
    } else if (ptr == HPM_CAN2) {
        clk = clock_can2;
    } else if (ptr == HPM_CAN3) {
        clk = clock_can3;
    } else {
        return 0U;
    }

    /* CAN clock = 80MHz, same as official HPM6750 EVK style. */
    clock_set_source_divider(clk, clk_src_pll1_clk1, 5);
    clock_add_to_group(clk, BOARD_RUNNING_CORE & 0x1);
    return clock_get_frequency(clk);
}

void board_set_can_termination(uint8_t can_index, bool enable)
{
    uint8_t level = enable ? BOARD_CAN_TERM_ENABLE_LEVEL : BOARD_CAN_TERM_DISABLE_LEVEL;

    switch (can_index) {
    case 1:
        gpio_write_pin(BOARD_CAN1_TERM_GPIO_CTRL, BOARD_CAN1_TERM_GPIO_INDEX, BOARD_CAN1_TERM_GPIO_PIN, level);
        break;
    case 2:
        gpio_write_pin(BOARD_CAN2_TERM_GPIO_CTRL, BOARD_CAN2_TERM_GPIO_INDEX, BOARD_CAN2_TERM_GPIO_PIN, level);
        break;
    case 3:
        gpio_write_pin(BOARD_CAN3_TERM_GPIO_CTRL, BOARD_CAN3_TERM_GPIO_INDEX, BOARD_CAN3_TERM_GPIO_PIN, level);
        break;
    case 4:
        gpio_write_pin(BOARD_CAN4_TERM_GPIO_CTRL, BOARD_CAN4_TERM_GPIO_INDEX, BOARD_CAN4_TERM_GPIO_PIN, level);
        break;
    default:
        break;
    }
}

uint32_t board_init_adc_clock(void *ptr, bool clk_src_bus)
{
    uint32_t freq = 0U;

    if (ptr == (void *) HPM_ADC3) {
        if (clk_src_bus) {
            clock_set_adc_source(clock_adc3, clk_adc_src_ahb0);
        } else {
            clock_set_adc_source(clock_adc3, clk_adc_src_ana2);
            clock_set_source_divider(clock_ana2, clk_src_pll1_clk1, 2U);
        }
        clock_add_to_group(clock_adc3, BOARD_RUNNING_CORE & 0x1);
        freq = clock_get_frequency(clock_adc3);
    }

    return freq;
}

void board_init_adc16_pins(void)
{
    init_adc_pins();
}

hpm_stat_t board_init_enet_pins(ENET_Type *ptr)
{
    if (ptr != HPM_ENET1) {
        return status_invalid_argument;
    }

    init_eth1_rgmii_pins();
    clock_add_to_group(clock_eth1, BOARD_RUNNING_CORE & 0x1);

    /* Keep PHY in reset here. Call board_reset_enet_phy() to release it. */
    gpio_write_pin(BOARD_ENET_PHY_RST_GPIO, BOARD_ENET_PHY_RST_GPIO_INDEX,
                   BOARD_ENET_PHY_RST_GPIO_PIN, BOARD_ENET_PHY_RESET_ACTIVE_LEVEL);

    /* Do not enable PHY INTB here. Enable it after your Ethernet driver ISR is ready. */
    gpio_disable_pin_interrupt(HPM_GPIO0, GPIO_IE_GPIOD, BOARD_ENET_PHY_INT_GPIO_PIN);

    return status_success;
}

hpm_stat_t board_reset_enet_phy(ENET_Type *ptr)
{
    if (ptr != HPM_ENET1) {
        return status_invalid_argument;
    }

    gpio_write_pin(BOARD_ENET_PHY_RST_GPIO, BOARD_ENET_PHY_RST_GPIO_INDEX,
                   BOARD_ENET_PHY_RST_GPIO_PIN, BOARD_ENET_PHY_RESET_ACTIVE_LEVEL);
    board_delay_ms(10);
    gpio_write_pin(BOARD_ENET_PHY_RST_GPIO, BOARD_ENET_PHY_RST_GPIO_INDEX,
                   BOARD_ENET_PHY_RST_GPIO_PIN, BOARD_ENET_PHY_RESET_INACTIVE_LEVEL);
    board_delay_ms(50);

    return status_success;
}

hpm_stat_t board_init_enet_ptp_clock(ENET_Type *ptr)
{
    if (ptr != HPM_ENET1) {
        return status_invalid_argument;
    }

    clock_add_to_group(clock_ptp1, BOARD_RUNNING_CORE & 0x1);
    clock_set_source_divider(clock_ptp1, clk_src_pll1_clk1, 4); /* 100MHz */
    return status_success;
}

hpm_stat_t board_enable_enet_irq(ENET_Type *ptr)
{
    if (ptr == HPM_ENET1) {
        intc_m_enable_irq(IRQn_ENET1);
        return status_success;
    }
    return status_invalid_argument;
}

hpm_stat_t board_disable_enet_irq(ENET_Type *ptr)
{
    if (ptr == HPM_ENET1) {
        intc_m_disable_irq(IRQn_ENET1);
        return status_success;
    }
    return status_invalid_argument;
}

uint8_t board_get_enet_dma_pbl(ENET_Type *ptr)
{
    (void) ptr;
    return enet_pbl_32;
}

void board_ungate_mchtmr_at_lp_mode(void)
{
    sysctl_set_cpu_lp_mode(HPM_SYSCTL, BOARD_RUNNING_CORE, cpu_lp_mode_ungate_cpu_clock);
}
