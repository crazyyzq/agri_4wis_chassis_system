/* Public entry points registered by test_registry.c. */
#ifndef ECU_TEST_CASES_H
#define ECU_TEST_CASES_H
#include "test_runner.h"
test_status_t test_safe_boot(test_context_t *context);
test_status_t test_power(test_context_t *context);
test_status_t test_rgb(test_context_t *context);
void test_rgb_cleanup(test_context_t *context);
test_status_t test_internal_ram(test_context_t *context);
test_status_t test_sdram(test_context_t *context);
test_status_t test_qspi_flash(test_context_t *context);
test_status_t test_eeprom(test_context_t *context);
test_status_t test_adc(test_context_t *context);
test_status_t test_digital_input(test_context_t *context);
test_status_t test_digital_output(test_context_t *context);
test_status_t test_dio_loopback(test_context_t *context);
void test_output_cleanup(test_context_t *context);
test_status_t test_can(test_context_t *context);
void test_can_cleanup(test_context_t *context);
test_status_t test_rs485(test_context_t *context);
void test_rs485_cleanup(test_context_t *context);
test_status_t test_rs232(test_context_t *context);
test_status_t test_sbus(test_context_t *context);
test_status_t test_ethernet_skip(test_context_t *context);
#endif
