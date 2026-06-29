#ifndef DIO_SERVICE_H
#define DIO_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

typedef bool (*dio_service_apply_backend_t)(void *context,
                                            uint32_t physical_output_mask,
                                            uint32_t managed_output_mask);

typedef struct {
    uint32_t output_mask;
    uint32_t physical_output_mask;
    uint32_t managed_output_mask;
    void *apply_context;
    dio_service_apply_backend_t apply_backend;
    bool active_high;
    bool last_apply_ok;
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

/* Attach the physical output writer owned by the board driver layer.
 *
 * The service still owns logical state and polarity conversion.  The backend
 * receives only the already-converted physical mask for managed output bits.
 */
void dio_service_set_apply_backend(dio_service_t *service,
                                   dio_service_apply_backend_t backend,
                                   void *context);

/* Set or clear logical output bits using a mask from hardware config. */
void dio_service_write_masked(dio_service_t *service,
                              uint32_t mask,
                              bool enabled);
uint32_t dio_service_get_physical_mask(const dio_service_t *service);

#endif /* DIO_SERVICE_H */
