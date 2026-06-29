#ifndef DIO_HW_H
#define DIO_HW_H

#include <stdbool.h>

#include "dio_service.h"

/* Bind the logical DIO service to the 12 isolated ECU output pins.
 *
 * Output bit 0 maps to board output index 1, bit 1 to index 2, and so on.
 * Bits outside BOARD_ECU_OUTPUT_COUNT stay owned by software state only.
 */
void dio_hw_attach_outputs(dio_service_t *service);

bool dio_hw_apply_output_mask(void *context,
                              uint32_t physical_output_mask,
                              uint32_t managed_output_mask);

#endif /* DIO_HW_H */
