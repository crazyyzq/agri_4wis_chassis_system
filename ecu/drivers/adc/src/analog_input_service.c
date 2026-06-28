#include <string.h>

#include "analog_input_service.h"

void analog_input_service_init(analog_input_service_t *service)
{
    if (service != 0) {
        memset(service, 0, sizeof(*service));
    }
}

bool analog_input_service_update_raw(analog_input_service_t *service,
                                     uint8_t channel,
                                     uint32_t raw,
                                     const ecu_hardware_config_t *config)
{
    if (service == 0 || config == 0 || channel >= config->adc_channel_count ||
        channel >= ECU_ADC_CHANNEL_COUNT || config->adc_raw_max == 0U) {
        return false;
    }
    if (raw > config->adc_raw_max) {
        raw = config->adc_raw_max;
    }
    service->snapshot.millivolt[channel] =
        (raw * config->adc_external_mv_max) / config->adc_raw_max;
    service->snapshot.valid[channel] = true;
    service->snapshot.update_sequence++;
    return true;
}

bool analog_input_service_update_millivolt(analog_input_service_t *service,
                                           uint8_t channel,
                                           uint32_t millivolt)
{
    if (service == 0 || channel >= ECU_ADC_CHANNEL_COUNT) {
        return false;
    }

    service->snapshot.millivolt[channel] = millivolt;
    service->snapshot.valid[channel] = true;
    service->snapshot.update_sequence++;
    return true;
}

void analog_input_service_get_snapshot(const analog_input_service_t *service,
                                       analog_input_snapshot_t *out)
{
    if (service != 0 && out != 0) {
        *out = service->snapshot;
    }
}
