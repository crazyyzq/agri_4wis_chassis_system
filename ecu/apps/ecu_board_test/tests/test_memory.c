/* Internal RAM, SDRAM, reserved QSPI sector and restore-after-test EEPROM checks. */
#include <stdio.h>
#include <string.h>
#include "board.h"
#include "hpm_l1c_drv.h"
#include "hpm_i2c_drv.h"
#include "hpm_romapi.h"
#include "memory_patterns.h"
#include "operator_io.h"
#include "test_cases.h"

extern uint8_t __ecu_test_flash_start__[];
extern uint8_t __ecu_test_flash_end__[];
static uint32_t s_ram_scratch[1024];
static uint32_t s_flash_write[64];
static uint32_t s_flash_read[64];

static bool run_basic_patterns(volatile uint32_t *base, size_t words, test_context_t *context)
{
    memory_mismatch_t mismatch;
    static const memory_pattern_t patterns[] = {
        MEMORY_PATTERN_ZERO, MEMORY_PATTERN_ONES, MEMORY_PATTERN_AA,
        MEMORY_PATTERN_55, MEMORY_PATTERN_ADDRESS
    };
    for (size_t i = 0; i < sizeof(patterns) / sizeof(patterns[0]); ++i) {
        if (test_runner_poll_abort(context)) return false;
        if (!memory_pattern_test(base, words, patterns[i], &mismatch)) {
            printf("MEM mismatch address=0x%08lx expected=0x%08lx actual=0x%08lx\n",
                   (unsigned long)mismatch.address, (unsigned long)mismatch.expected,
                   (unsigned long)mismatch.actual);
            return false;
        }
    }
    return true;
}

test_status_t test_internal_ram(test_context_t *context)
{
    bool ok = run_basic_patterns(s_ram_scratch, sizeof(s_ram_scratch) / sizeof(s_ram_scratch[0]), context);
    return ok ? TEST_PASS : (context->abort_requested ? TEST_BLOCKED : TEST_FAIL);
}

test_status_t test_sdram(test_context_t *context)
{
    if (!operator_confirm("Run destructive test over unused 32 MiB SDRAM")) return TEST_BLOCKED;
    volatile uint32_t *base = (volatile uint32_t *)BOARD_SDRAM_ADDRESS;
    const size_t chunk_words = (1U * SIZE_1MB) / sizeof(uint32_t);
    for (size_t offset = 0U; offset < BOARD_SDRAM_SIZE / sizeof(uint32_t); offset += chunk_words) {
        if (test_runner_poll_abort(context)) return TEST_BLOCKED;
        if (!run_basic_patterns(base + offset, chunk_words, context))
            return context->abort_requested ? TEST_BLOCKED : TEST_FAIL;
        printf("SDRAM checked %lu/%lu MiB\n", (unsigned long)((offset + chunk_words) * 4U / SIZE_1MB),
               (unsigned long)(BOARD_SDRAM_SIZE / SIZE_1MB));
    }
    return TEST_PASS;
}

ATTR_RAMFUNC test_status_t test_qspi_flash(test_context_t *context)
{
    (void)context;
    uintptr_t start = (uintptr_t)__ecu_test_flash_start__;
    uintptr_t end = (uintptr_t)__ecu_test_flash_end__;
    if (start != 0x807F0000UL || end != 0x80800000UL) {
        printf("FLASH reserved range invalid: 0x%08lx..0x%08lx\n", (unsigned long)start, (unsigned long)end);
        return TEST_BLOCKED;
    }
    if (!operator_confirm("Erase/program reserved final 64 KiB QSPI region")) return TEST_BLOCKED;
    xpi_nor_config_option_t option = { 0 };
    xpi_nor_config_t config;
    option.header.U = BOARD_APP_XPI_NOR_CFG_OPT_HDR;
    option.option0.U = BOARD_APP_XPI_NOR_CFG_OPT_OPT0;
    option.option1.U = BOARD_APP_XPI_NOR_CFG_OPT_OPT1;
    if (rom_xpi_nor_auto_config(BOARD_APP_XPI_NOR_XPI_BASE, &config, &option) != status_success) return TEST_FAIL;
    uint32_t flash_size = 0U, sector_size = 0U;
    if (rom_xpi_nor_get_property(BOARD_APP_XPI_NOR_XPI_BASE, &config,
            xpi_nor_property_total_size, &flash_size) != status_success ||
        rom_xpi_nor_get_property(BOARD_APP_XPI_NOR_XPI_BASE, &config,
            xpi_nor_property_sector_size, &sector_size) != status_success) {
        return TEST_FAIL;
    }
    if (flash_size != 8U * SIZE_1MB || sector_size == 0U || (64U * 1024U) % sector_size != 0U) return TEST_BLOCKED;
    uint32_t offset = flash_size - 64U * 1024U;
    uint8_t *write_bytes = (uint8_t *)s_flash_write;
    for (uint32_t i = 0U; i < sizeof(s_flash_write); ++i) write_bytes[i] = (uint8_t)(i ^ 0xA5U);
    hpm_stat_t status = rom_xpi_nor_erase(BOARD_APP_XPI_NOR_XPI_BASE, xpi_xfer_channel_auto,
                                          &config, offset, 64U * 1024U);
    if (status == status_success) status = rom_xpi_nor_program(BOARD_APP_XPI_NOR_XPI_BASE,
        xpi_xfer_channel_auto, &config, s_flash_write, offset, sizeof(s_flash_write));
    if (status == status_success) status = rom_xpi_nor_read(BOARD_APP_XPI_NOR_XPI_BASE,
        xpi_xfer_channel_auto, &config, s_flash_read, offset, sizeof(s_flash_read));
    bool matched = status == status_success && memcmp(s_flash_write, s_flash_read, sizeof(s_flash_write)) == 0;
    /* Always leave the reserved test region erased for deterministic recovery. */
    hpm_stat_t erase_status = rom_xpi_nor_erase(BOARD_APP_XPI_NOR_XPI_BASE,
        xpi_xfer_channel_auto, &config, offset, 64U * 1024U);
    l1c_dc_invalidate_all();
    return matched && erase_status == status_success ? TEST_PASS : TEST_FAIL;
}

test_status_t test_eeprom(test_context_t *context)
{
    (void)context;
    uint8_t address = 0xF0U;
    uint8_t backup[16], pattern[16], verify[16];
    board_init_i2c(BOARD_EEPROM_I2C_BASE);
    if (i2c_master_address_read(BOARD_EEPROM_I2C_BASE, BOARD_EEPROM_I2C_ADDRESS,
        &address, 1U, backup, sizeof(backup)) != status_success) return TEST_FAIL;
    for (uint8_t i = 0U; i < sizeof(pattern); ++i) pattern[i] = (uint8_t)(0x5AU ^ i);
    hpm_stat_t status = i2c_master_address_write(BOARD_EEPROM_I2C_BASE, BOARD_EEPROM_I2C_ADDRESS,
        &address, 1U, pattern, 8U);
    board_delay_ms(10U);
    address = 0xF8U;
    if (status == status_success) status = i2c_master_address_write(BOARD_EEPROM_I2C_BASE,
        BOARD_EEPROM_I2C_ADDRESS, &address, 1U, &pattern[8], 8U);
    board_delay_ms(10U);
    address = 0xF0U;
    if (status == status_success) status = i2c_master_address_read(BOARD_EEPROM_I2C_BASE,
        BOARD_EEPROM_I2C_ADDRESS, &address, 1U, verify, sizeof(verify));
    bool matched = status == status_success && memcmp(pattern, verify, sizeof(pattern)) == 0;
    status = i2c_master_address_write(BOARD_EEPROM_I2C_BASE, BOARD_EEPROM_I2C_ADDRESS,
        &address, 1U, backup, 8U);
    board_delay_ms(10U);
    address = 0xF8U;
    if (status == status_success) status = i2c_master_address_write(BOARD_EEPROM_I2C_BASE,
        BOARD_EEPROM_I2C_ADDRESS, &address, 1U, &backup[8], 8U);
    board_delay_ms(10U);
    address = 0xF0U;
    if (status == status_success) status = i2c_master_address_read(BOARD_EEPROM_I2C_BASE,
        BOARD_EEPROM_I2C_ADDRESS, &address, 1U, verify, sizeof(verify));
    return matched && status == status_success && memcmp(backup, verify, sizeof(backup)) == 0 ? TEST_PASS : TEST_FAIL;
}
