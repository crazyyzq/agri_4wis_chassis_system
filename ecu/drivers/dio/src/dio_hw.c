#include "dio_hw.h"

#include "board.h"

bool dio_hw_apply_output_mask(void *context,
                              uint32_t physical_output_mask,
                              uint32_t managed_output_mask)
{
    (void)context;

    for (uint8_t output = 0U; output < BOARD_ECU_OUTPUT_COUNT; ++output) {
        uint32_t bit = 1UL << output;
        if ((managed_output_mask & bit) != 0U) {
            board_ecu_output_write((uint8_t)(output + 1U),
                                   (physical_output_mask & bit) != 0U);
        }
    }
    return true;
}

void dio_hw_attach_outputs(dio_service_t *service)
{
    dio_service_set_apply_backend(service, dio_hw_apply_output_mask, 0);
}
