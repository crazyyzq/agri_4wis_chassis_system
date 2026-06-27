#ifndef DIO_SERVICE_H
#define DIO_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t output_mask;
    uint32_t physical_output_mask;
    uint32_t managed_output_mask;
    bool active_high;
} dio_service_t;

/* Initialize digital output state, active polarity and managed bit range.
 *
 * Owner: CPU0 local IO task and final executor path.
 * ISR: not safe.
 * Safety: active-low conversion is limited to managed_output_mask so unrelated
 * GPIO bits are never driven by a 32-bit complement operation.
 */
void dio_service_init(dio_service_t *service,
                      bool active_high,
                      uint32_t managed_output_mask);

/* Set or clear logical output bits using a mask from hardware config. */
void dio_service_write_masked(dio_service_t *service,
                              uint32_t mask,
                              bool enabled);
uint32_t dio_service_get_physical_mask(const dio_service_t *service);

#endif /* DIO_SERVICE_H */
