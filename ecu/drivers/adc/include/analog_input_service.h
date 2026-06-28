#ifndef ANALOG_INPUT_SERVICE_H
#define ANALOG_INPUT_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#include "ecu_config.h"

typedef struct {
    uint32_t millivolt[ECU_ADC_CHANNEL_COUNT];
    bool valid[ECU_ADC_CHANNEL_COUNT];
    uint32_t update_sequence;
} analog_input_snapshot_t;

typedef struct {
    analog_input_snapshot_t snapshot;
} analog_input_service_t;

/* Initialize analog input conversion state.
 *
 * Owner: CPU0 IO task.
 * ISR: not safe; ISR code should only buffer raw samples.
 */
void analog_input_service_init(analog_input_service_t *service);

/* Convert one raw ADC sample into external millivolts.
 *
 * Units: raw is ADC counts, output is millivolts at the external signal input.
 * Scaling comes from hardware config so divider changes do not touch logic.
 */
bool analog_input_service_update_raw(analog_input_service_t *service,
                                     uint8_t channel,
                                     uint32_t raw,
                                     const ecu_hardware_config_t *config);

/* Store an already-scaled external analog voltage.
 *
 * Use this for external acquisition modules that report engineering units or
 * values already converted by their device adapter.  The function still owns
 * validity flags and sequence accounting so all analog sources present the same
 * snapshot contract.
 */
bool analog_input_service_update_millivolt(analog_input_service_t *service,
                                           uint8_t channel,
                                           uint32_t millivolt);
void analog_input_service_get_snapshot(const analog_input_service_t *service,
                                       analog_input_snapshot_t *out);

#endif /* ANALOG_INPUT_SERVICE_H */
