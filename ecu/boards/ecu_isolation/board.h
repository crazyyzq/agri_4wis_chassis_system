/*
 * Copyright (c) 2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Board support file for ECU_Isolation board
 * MCU module: ZLG MR/MB6750-WB32F8AWI-L
 */

#ifndef _HPM_BOARD_H
#define _HPM_BOARD_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "hpm_common.h"
#include "hpm_soc.h"
#include "hpm_soc_feature.h"
#include "hpm_clock_drv.h"
#include "hpm_femc_drv.h"
#include "pinmux.h"

#if !defined(CONFIG_NDEBUG_CONSOLE) || !CONFIG_NDEBUG_CONSOLE
#include "hpm_debug_console.h"
#endif

#if defined(CONFIG_ENET_PHY) && CONFIG_ENET_PHY
#include "hpm_enet_phy.h"
#endif

#define BOARD_NAME          "ecu_isolation_mr6750"
#define BOARD_UF2_SIGNATURE (0x0A4D5048UL)

#ifndef BOARD_RUNNING_CORE
#define BOARD_RUNNING_CORE HPM_CORE0
#endif

#ifndef BOARD_CPU_FREQ
#define BOARD_CPU_FREQ (816000000UL)
#endif

#ifndef BOARD_SHOW_CLOCK
#define BOARD_SHOW_CLOCK 1
#endif

#ifndef BOARD_SHOW_BANNER
#define BOARD_SHOW_BANNER 1
#endif

/* -------------------------------------------------------------------------- */
/* Console / debug UART                                                        */
/* DEBUG_TX: PY06 -> UART0_TXD, DEBUG_RX: PY07 -> UART0_RXD                    */
/* -------------------------------------------------------------------------- */
#ifndef BOARD_CONSOLE_TYPE
#define BOARD_CONSOLE_TYPE CONSOLE_TYPE_UART
#endif

#ifndef BOARD_CONSOLE_UART_BASE
#define BOARD_CONSOLE_UART_BASE       HPM_UART0
#define BOARD_CONSOLE_UART_CLK_NAME   clock_uart0
#define BOARD_CONSOLE_UART_IRQ        IRQn_UART0
#define BOARD_CONSOLE_UART_TX_DMA_REQ HPM_DMA_SRC_UART0_TX
#define BOARD_CONSOLE_UART_RX_DMA_REQ HPM_DMA_SRC_UART0_RX
#endif
#define BOARD_CONSOLE_UART_BAUDRATE (115200UL)

/* SDRAM section */
#define BOARD_SDRAM_ADDRESS          (0x40000000UL)
#define BOARD_SDRAM_SIZE             (32 * SIZE_1MB)
#define BOARD_SDRAM_CS               FEMC_SDRAM_CS0
#define BOARD_SDRAM_PORT_SIZE        FEMC_SDRAM_PORT_SIZE_16_BITS
#define BOARD_SDRAM_COLUMN_ADDR_BITS FEMC_SDRAM_COLUMN_ADDR_9_BITS
/* 32MB / 256Mbit SDRAM is normally 8192 refresh cycles per 64ms.
 * If your module manual says 16MB / 128Mbit SDRAM, change this back to 4096UL.
 */
#define BOARD_SDRAM_REFRESH_COUNT    (8192UL)
#define BOARD_SDRAM_REFRESH_IN_MS    (64UL)

/* Keep SDK examples compatible: use RS232_1 as generic APP UART by default */
#ifndef BOARD_APP_UART_BASE
#define BOARD_APP_UART_BASE       HPM_UART15
#define BOARD_APP_UART_IRQ        IRQn_UART15
#define BOARD_APP_UART_BAUDRATE   (115200UL)
#define BOARD_APP_UART_CLK_NAME   clock_uart15
#define BOARD_APP_UART_RX_DMA_REQ HPM_DMA_SRC_UART15_RX
#define BOARD_APP_UART_TX_DMA_REQ HPM_DMA_SRC_UART15_TX
#endif

/* SBUS: PA01 -> UART1_RXD */
#define BOARD_SBUS_UART_BASE     HPM_UART1
#define BOARD_SBUS_UART_IRQ      IRQn_UART1
#define BOARD_SBUS_UART_CLK_NAME clock_uart1
#define BOARD_SBUS_BAUDRATE      (100000UL)

/* -------------------------------------------------------------------------- */
/* EEPROM I2C                                                                  */
/* PB14 -> I2C3_SCL, PB13 -> I2C3_SDA                                          */
/* -------------------------------------------------------------------------- */
#define BOARD_EEPROM_I2C_BASE     HPM_I2C3
#define BOARD_EEPROM_I2C_IRQ      IRQn_I2C3
#define BOARD_EEPROM_I2C_CLK_NAME clock_i2c3
#define BOARD_EEPROM_I2C_DMA_SRC  HPM_DMA_SRC_I2C3
#define BOARD_EEPROM_I2C_ADDRESS  (0x50U)

#define BOARD_APP_I2C_BASE        BOARD_EEPROM_I2C_BASE
#define BOARD_APP_I2C_IRQ         BOARD_EEPROM_I2C_IRQ
#define BOARD_APP_I2C_CLK_NAME    BOARD_EEPROM_I2C_CLK_NAME
#define BOARD_APP_I2C_DMA_SRC     BOARD_EEPROM_I2C_DMA_SRC

/* -------------------------------------------------------------------------- */
/* CAN / CANFD                                                                 */
/* External CAN1..4 maps to HPM CAN0..3. Pinmux is the same for CAN/CANFD.      */
/* -------------------------------------------------------------------------- */
#define BOARD_CAN1_BASE     HPM_CAN0
#define BOARD_CAN1_IRQn     IRQn_CAN0
#define BOARD_CAN1_CLK_NAME clock_can0
#define BOARD_CAN2_BASE     HPM_CAN1
#define BOARD_CAN2_IRQn     IRQn_CAN1
#define BOARD_CAN2_CLK_NAME clock_can1
#define BOARD_CAN3_BASE     HPM_CAN2
#define BOARD_CAN3_IRQn     IRQn_CAN2
#define BOARD_CAN3_CLK_NAME clock_can2
#define BOARD_CAN4_BASE     HPM_CAN3
#define BOARD_CAN4_IRQn     IRQn_CAN3
#define BOARD_CAN4_CLK_NAME clock_can3

#define BOARD_APP_CAN_BASE  BOARD_CAN1_BASE
#define BOARD_APP_CAN_IRQn  BOARD_CAN1_IRQn

/* CAN 120R termination control pins. The schematic uses optocoupler + NMOS.
 * MCU output low  -> termination enabled
 * MCU output high -> termination disabled
 */
#define BOARD_CAN_TERM_ENABLE_LEVEL  (0U)
#define BOARD_CAN_TERM_DISABLE_LEVEL (1U)

#define BOARD_CAN1_TERM_GPIO_CTRL  HPM_GPIO0
#define BOARD_CAN1_TERM_GPIO_INDEX GPIO_DO_GPIOB
#define BOARD_CAN1_TERM_GPIO_PIN   (6U)
#define BOARD_CAN2_TERM_GPIO_CTRL  HPM_GPIO0
#define BOARD_CAN2_TERM_GPIO_INDEX GPIO_DO_GPIOB
#define BOARD_CAN2_TERM_GPIO_PIN   (3U)
#define BOARD_CAN3_TERM_GPIO_CTRL  HPM_GPIO0
#define BOARD_CAN3_TERM_GPIO_INDEX GPIO_DO_GPIOB
#define BOARD_CAN3_TERM_GPIO_PIN   (1U)
#define BOARD_CAN4_TERM_GPIO_CTRL  HPM_GPIO0
#define BOARD_CAN4_TERM_GPIO_INDEX GPIO_DO_GPIOB
#define BOARD_CAN4_TERM_GPIO_PIN   (4U)

/* -------------------------------------------------------------------------- */
/* RS485                                                                       */
/* Current pinmux uses UART hardware DE pins. If you later change DE to GPIO,  */
/* set BOARD_RS485_DE_USING_GPIO to 1 and update pinmux accordingly.           */
/* -------------------------------------------------------------------------- */
#define BOARD_RS485_1_UART_BASE     HPM_UART11
#define BOARD_RS485_1_UART_IRQ      IRQn_UART11
#define BOARD_RS485_1_UART_CLK_NAME clock_uart11
#define BOARD_RS485_2_UART_BASE     HPM_UART12
#define BOARD_RS485_2_UART_IRQ      IRQn_UART12
#define BOARD_RS485_2_UART_CLK_NAME clock_uart12
#define BOARD_RS485_3_UART_BASE     HPM_UART10
#define BOARD_RS485_3_UART_IRQ      IRQn_UART10
#define BOARD_RS485_3_UART_CLK_NAME clock_uart10

#ifndef BOARD_RS485_DE_USING_GPIO
#define BOARD_RS485_DE_USING_GPIO 0
#endif
#define BOARD_RS485_TX_ENABLE_LEVEL 1U
#define BOARD_RS485_RX_ENABLE_LEVEL 0U

/* GPIO definition only used when BOARD_RS485_DE_USING_GPIO == 1 */
#define BOARD_RS485_1_DE_GPIO_CTRL  HPM_GPIO0
#define BOARD_RS485_1_DE_GPIO_INDEX GPIO_DO_GPIOD
#define BOARD_RS485_1_DE_GPIO_PIN   (18U)
#define BOARD_RS485_2_DE_GPIO_CTRL  HPM_GPIO0
#define BOARD_RS485_2_DE_GPIO_INDEX GPIO_DO_GPIOB
#define BOARD_RS485_2_DE_GPIO_PIN   (11U)
#define BOARD_RS485_3_DE_GPIO_CTRL  HPM_GPIO0
#define BOARD_RS485_3_DE_GPIO_INDEX GPIO_DO_GPIOA
#define BOARD_RS485_3_DE_GPIO_PIN   (17U)

/* -------------------------------------------------------------------------- */
/* RS232                                                                       */
/* -------------------------------------------------------------------------- */
#define BOARD_RS232_1_UART_BASE     HPM_UART15
#define BOARD_RS232_1_UART_IRQ      IRQn_UART15
#define BOARD_RS232_1_UART_CLK_NAME clock_uart15
#define BOARD_RS232_2_UART_BASE     HPM_UART13
#define BOARD_RS232_2_UART_IRQ      IRQn_UART13
#define BOARD_RS232_2_UART_CLK_NAME clock_uart13
#define BOARD_RS232_3_UART_BASE     HPM_UART5
#define BOARD_RS232_3_UART_IRQ      IRQn_UART5
#define BOARD_RS232_3_UART_CLK_NAME clock_uart5
#define BOARD_RS232_4_UART_BASE     HPM_UART14
#define BOARD_RS232_4_UART_IRQ      IRQn_UART14
#define BOARD_RS232_4_UART_CLK_NAME clock_uart14

/* -------------------------------------------------------------------------- */
/* On-board RGB LED                                                            */
/* PE07: Red, PE05: Green, PE02: Blue, common-anode, low-active.               */
/* -------------------------------------------------------------------------- */
#define BOARD_LED_ON_LEVEL  0U
#define BOARD_LED_OFF_LEVEL 1U

#define BOARD_R_GPIO_CTRL  HPM_GPIO0
#define BOARD_R_GPIO_INDEX GPIO_DO_GPIOE
#define BOARD_R_GPIO_PIN   (7U)
#define BOARD_G_GPIO_CTRL  HPM_GPIO0
#define BOARD_G_GPIO_INDEX GPIO_DO_GPIOE
#define BOARD_G_GPIO_PIN   (5U)
#define BOARD_B_GPIO_CTRL  HPM_GPIO0
#define BOARD_B_GPIO_INDEX GPIO_DO_GPIOE
#define BOARD_B_GPIO_PIN   (2U)

#define BOARD_LED_GPIO_CTRL  BOARD_G_GPIO_CTRL
#define BOARD_LED_GPIO_INDEX BOARD_G_GPIO_INDEX
#define BOARD_LED_GPIO_PIN   BOARD_G_GPIO_PIN

#define BOARD_RGB_RED   (0U)
#define BOARD_RGB_GREEN (1U)
#define BOARD_RGB_BLUE  (2U)

/* -------------------------------------------------------------------------- */
/* ECU digital IO                                                              */
/* 12 outputs are low=off. 12 isolated inputs are low-active.                  */
/* -------------------------------------------------------------------------- */
#define BOARD_ECU_OUTPUT_COUNT (12U)
#define BOARD_ECU_INPUT_COUNT  (12U)
#define BOARD_ECU_OUTPUT_OFF_LEVEL 0U
#define BOARD_ECU_OUTPUT_ON_LEVEL  1U
#define BOARD_ECU_INPUT_ACTIVE_LEVEL 0U

/* -------------------------------------------------------------------------- */
/* ADC                                                                         */
/* Analog inputs: ADC3_INA4/5/6/7.                                             */
/* -------------------------------------------------------------------------- */
#define BOARD_APP_ADC16_NAME     "ADC3"
#define BOARD_APP_ADC16_BASE     HPM_ADC3
#define BOARD_APP_ADC16_IRQn     IRQn_ADC3
#define BOARD_APP_ADC16_CLK_NAME clock_adc3
#define BOARD_APP_ADC16_CLK_BUS  clk_adc_src_ahb0

#define BOARD_ANALOG_EX1_ADC_BASE BOARD_APP_ADC16_BASE
#define BOARD_ANALOG_EX1_ADC_CH   (6U)  /* PF09 -> ADC3_INA6 */
#define BOARD_ANALOG_EX2_ADC_BASE BOARD_APP_ADC16_BASE
#define BOARD_ANALOG_EX2_ADC_CH   (4U)  /* PF05 -> ADC3_INA4 */
#define BOARD_ANALOG_EX3_ADC_BASE BOARD_APP_ADC16_BASE
#define BOARD_ANALOG_EX3_ADC_CH   (5U)  /* PF07 -> ADC3_INA5 */
#define BOARD_ANALOG_EX4_ADC_BASE BOARD_APP_ADC16_BASE
#define BOARD_ANALOG_EX4_ADC_CH   (7U)  /* PF10 -> ADC3_INA7 */

/* XPI0 NOR parameters match the MR6750 module's 8 MiB boot flash. */
#define BOARD_APP_XPI_NOR_XPI_BASE     HPM_XPI0
#define BOARD_APP_XPI_NOR_CFG_OPT_HDR  (0xfcf90002U)
#define BOARD_APP_XPI_NOR_CFG_OPT_OPT0 (0x00000007U)
#define BOARD_APP_XPI_NOR_CFG_OPT_OPT1 (0x0000000EU)

/* -------------------------------------------------------------------------- */
/* Ethernet                                                                    */
/* RTL8211E + ETH1 RGMII. PD30=PHY reset low-active, PD31=PHY INTB low-active. */
/* -------------------------------------------------------------------------- */
#define BOARD_ENET              HPM_ENET1
#define BOARD_ENET_IRQ          IRQn_ENET1
#define BOARD_ENET_CLK_NAME     clock_eth1
#define BOARD_ENET_PTP_CLOCK    clock_ptp1
#define BOARD_ENET_PHY_RST_GPIO HPM_GPIO0
#define BOARD_ENET_PHY_RST_GPIO_INDEX GPIO_DO_GPIOD
#define BOARD_ENET_PHY_RST_GPIO_PIN   (30U)
#define BOARD_ENET_PHY_INT_GPIO HPM_GPIO0
#define BOARD_ENET_PHY_INT_GPIO_INDEX GPIO_DI_GPIOD
#define BOARD_ENET_PHY_INT_GPIO_PIN   (31U)
#define BOARD_ENET_PHY_RESET_ACTIVE_LEVEL   0U
#define BOARD_ENET_PHY_RESET_INACTIVE_LEVEL 1U

/* -------------------------------------------------------------------------- */
/* Timer                                                                       */
/* -------------------------------------------------------------------------- */
#define BOARD_DELAY_TIMER          HPM_GPTMR7
#define BOARD_DELAY_TIMER_CH       (0U)
#define BOARD_DELAY_TIMER_CLK_NAME clock_gptmr7

#if defined(__cplusplus)
extern "C" {
#endif

typedef void (*board_timer_cb)(void);

/**
 * @brief Initialize core clocks, console, safe GPIO, and PMP for core 0.
 * @note Safe GPIO levels are established before diagnostic banners are printed.
 *       Fatal clock/console initialization failures remain in an infinite loop.
 */
void board_init(void);
/** @brief Refresh core-1 clock state, console, and PMP configuration. */
void board_init_core1(void);
/**
 * @brief Configure core/bus clocks and the HPM6750 DCDC operating point.
 * @note A PLL0 configuration failure is fatal and remains in an infinite loop.
 */
void board_init_clock(void);
/**
 * @brief Initialize the 115200-baud UART0 debug console.
 * @note Unsupported console types or initialization failure are fatal loops.
 */
void board_init_console(void);
/** @brief Print selected core, bus, MCHTMR, and XPI frequencies in hertz. */
void board_print_clock_freq(void);
/** @brief Print the HPM SDK version, when available, and BOARD_NAME. */
void board_print_banner(void);
/** @brief Configure PMP/PMA entries for linker-defined noncacheable/shared RAM. */
void board_init_pmp(void);

/* External SDRAM / FEMC. Used when yaml enables on-board-ram: sdram. */
/** @brief Apply generated MR6750 FEMC SDRAM pin multiplexing. */
void board_init_sdram_pins(void);
/**
 * @brief Enable and configure the FEMC source clock.
 * @return Actual FEMC clock frequency in hertz.
 */
uint32_t board_init_femc_clock(void);
/**
 * @brief Initialize the module's 32 MiB, 16-bit, four-bank external SDRAM.
 * @note Available when INIT_EXT_RAM_FOR_DATA is enabled. Configuration failure
 *       is fatal and remains in an infinite loop.
 */
void _init_ext_ram(void);

/** @brief Busy-wait for a requested duration. @param ms Delay in milliseconds. */
void board_delay_ms(uint32_t ms);
/** @brief Busy-wait for a requested duration. @param us Delay in microseconds. */
void board_delay_us(uint32_t us);

/**
 * @brief Initialize vehicle GPIO and force outputs, CAN termination, and RGB off.
 * @note Safe logical levels are re-applied after generated pinmux initialization.
 */
void board_init_safe_gpio(void);
/** @brief Compatibility wrapper that applies board_init_safe_gpio(). */
void board_init_gpio_pins(void);
/** @brief Initialize common-anode RGB pins and turn all colors logically off. */
void board_init_led_pins(void);
/**
 * @brief Write the raw electrical level of the generic green SDK LED.
 * @param state Raw GPIO level; BOARD_LED_ON_LEVEL is active-low on this board.
 */
void board_led_write(uint8_t state);
/** @brief Toggle the generic green SDK LED's current raw GPIO level. */
void board_led_toggle(void);
/**
 * @brief Set one RGB color using logical on/off semantics.
 * @param color BOARD_RGB_RED, BOARD_RGB_GREEN, or BOARD_RGB_BLUE.
 * @param on    Nonzero for on, zero for off; active-low polarity is handled here.
 * @note Invalid color values are ignored.
 */
void board_rgb_write(uint8_t color, uint8_t on);

/**
 * @brief Set one protected external ECU output using logical on/off semantics.
 * @param index One-based output number in 1..BOARD_ECU_OUTPUT_COUNT.
 * @param on    Nonzero for on, zero for off.
 * @note Invalid indexes are ignored.
 */
void board_ecu_output_write(uint8_t index, uint8_t on);
/**
 * @brief Read one isolated external ECU input as an active-high logical value.
 * @param index One-based input number in 1..BOARD_ECU_INPUT_COUNT.
 * @return One when the active-low hardware input is asserted; zero when inactive
 *         or when index is invalid.
 */
uint8_t board_ecu_input_read(uint8_t index);

/**
 * @brief Enable the clock for a supported console/SBUS/RS232/RS485 UART.
 * @param ptr UART0, 1, 5, or 10..15 as mapped by this board.
 * @return Actual UART clock frequency in hertz, or zero when unsupported.
 */
uint32_t board_init_uart_clock(UART_Type *ptr);
/**
 * @brief Apply board pinmux and enable the clock for a supported UART.
 * @param ptr Console, SBUS, RS232, or RS485 UART instance; unsupported instances
 *            receive no pin configuration and have a zero clock result.
 */
void board_init_uart(UART_Type *ptr);

/**
 * @brief Enable the EEPROM I2C3 clock.
 * @param ptr I2C instance; only HPM_I2C3 is supported.
 * @return Actual I2C3 frequency in hertz, or zero when unsupported.
 */
uint32_t board_init_i2c_clock(I2C_Type *ptr);
/**
 * @brief Apply EEPROM pins and initialize I2C3 as a normal-mode master.
 * @param ptr Must be HPM_I2C3.
 * @note Initialization failure is fatal and remains in an infinite loop.
 */
void board_init_i2c(I2C_Type *ptr);

/**
 * @brief Enable and configure the clock for CAN0..CAN3.
 * @param ptr CAN0, CAN1, CAN2, or CAN3.
 * @return Actual 80 MHz CAN clock in hertz, or zero when unsupported.
 */
uint32_t board_init_can_clock(CAN_Type *ptr);
/**
 * @brief Apply all CAN pinmux settings and enable the selected CAN clock.
 * @param ptr CAN0..CAN3; unsupported instances receive a zero clock result.
 */
void board_init_can(CAN_Type *ptr);
/**
 * @brief Set external 120 Ohm termination for one CAN connector.
 * @param can_index One-based external CAN number in 1..4.
 * @param enable    true to enable termination, false to disable it.
 * @note Invalid indexes are ignored. This function handles the active-low GPIO.
 */
void board_set_can_termination(uint8_t can_index, bool enable);

/**
 * @brief Select, enable, and query the ADC3 clock.
 * @param ptr         ADC instance; only HPM_ADC3 is supported.
 * @param clk_src_bus true for AHB0, false for ANA2 derived from PLL1.
 * @return Actual ADC3 frequency in hertz, or zero when unsupported.
 */
uint32_t board_init_adc_clock(void *ptr, bool clk_src_bus);
/** @brief Apply generated pinmux for the four ADC3 external analog inputs. */
void board_init_adc16_pins(void);

/**
 * @brief Configure ENET1 RGMII pins/clock while keeping the PHY in reset.
 * @param ptr Must be HPM_ENET1.
 * @return status_success, or status_invalid_argument for another instance.
 * @note PHY INTB remains disabled until the driver ISR is ready.
 */
hpm_stat_t board_init_enet_pins(ENET_Type *ptr);
/**
 * @brief Pulse the active-low ENET1 PHY reset and wait for release.
 * @param ptr Must be HPM_ENET1.
 * @return status_success, or status_invalid_argument for another instance.
 */
hpm_stat_t board_reset_enet_phy(ENET_Type *ptr);
/**
 * @brief Configure the ENET1 PTP clock to 100 MHz.
 * @param ptr Must be HPM_ENET1.
 * @return status_success, or status_invalid_argument for another instance.
 */
hpm_stat_t board_init_enet_ptp_clock(ENET_Type *ptr);
/**
 * @brief Enable the machine interrupt for ENET1.
 * @param ptr Must be HPM_ENET1.
 * @return status_success, or status_invalid_argument for another instance.
 */
hpm_stat_t board_enable_enet_irq(ENET_Type *ptr);
/**
 * @brief Disable the machine interrupt for ENET1.
 * @param ptr Must be HPM_ENET1.
 * @return status_success, or status_invalid_argument for another instance.
 */
hpm_stat_t board_disable_enet_irq(ENET_Type *ptr);
/**
 * @brief Return the fixed ENET DMA programmable-burst length.
 * @param ptr ENET instance; currently ignored for SDK compatibility.
 * @return enet_pbl_32.
 */
uint8_t board_get_enet_dma_pbl(ENET_Type *ptr);

/** @brief Keep the running core clock and MCHTMR available in low-power mode. */
void board_ungate_mchtmr_at_lp_mode(void);

#if defined(__cplusplus)
}
#endif

#endif /* _HPM_BOARD_H */
