#include <string.h>
#include "test_cases.h"
#include "test_registry.h"
static const test_descriptor_t registry[] = {
    { "SAFE.BOOT", TEST_REQUIRED, NULL, 3000U, NULL, test_safe_boot, NULL },
    { "PWR.RAILS", TEST_REQUIRED, "SAFE.BOOT", 120000U, NULL, test_power, NULL },
    { "RGB.ALL", TEST_REQUIRED, "PWR.RAILS", 60000U, NULL, test_rgb, test_rgb_cleanup },
    { "MEM.INTERNAL", TEST_REQUIRED, "PWR.RAILS", 30000U, NULL, test_internal_ram, NULL },
    { "MEM.SDRAM", TEST_REQUIRED, "PWR.RAILS", 600000U, NULL, test_sdram, NULL },
    { "MEM.QSPI", TEST_REQUIRED, "PWR.RAILS", 120000U, NULL, test_qspi_flash, NULL },
    { "MEM.EEPROM", TEST_REQUIRED, "PWR.RAILS", 30000U, NULL, test_eeprom, NULL },
    { "ADC.ALL", TEST_REQUIRED, "PWR.RAILS", 300000U, NULL, test_adc, NULL },
    { "DI.ALL", TEST_REQUIRED, "PWR.RAILS", 300000U, NULL, test_digital_input, NULL },
    { "SBUS.RX", TEST_REQUIRED, "PWR.RAILS", 120000U, NULL, test_sbus, NULL },
    { "RS232.ALL", TEST_REQUIRED, "PWR.RAILS", 600000U, NULL, test_rs232, NULL },
    { "RS485.ALL", TEST_REQUIRED, "PWR.RAILS", 600000U, NULL, test_rs485, test_rs485_cleanup },
    { "CAN.ALL", TEST_REQUIRED, "PWR.RAILS", 600000U, NULL, test_can, test_can_cleanup },
    { "DO.ALL", TEST_REQUIRED, "PWR.RAILS", 300000U, NULL, test_digital_output, test_output_cleanup },
    { "DIO.LOOP", TEST_REQUIRED, "PWR.RAILS", 300000U, NULL, test_dio_loopback, test_output_cleanup },
    { "ETH.SKIP_NO_DEVICE", TEST_OPTIONAL, NULL, 0U, NULL, test_ethernet_skip, NULL },
};
const test_descriptor_t *test_registry_all(size_t *count)
{
    if (count != NULL) *count = sizeof(registry) / sizeof(registry[0]);
    return registry;
}
const test_descriptor_t *test_registry_find(const char *id)
{
    for (size_t i = 0U; i < sizeof(registry) / sizeof(registry[0]); ++i)
        if (strcmp(registry[i].id, id) == 0) return &registry[i];
    return NULL;
}
