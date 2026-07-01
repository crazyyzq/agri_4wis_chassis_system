#ifndef LOCAL_IO_DEVICE_H
#define LOCAL_IO_DEVICE_H

#include "dio_service.h"
#include "ecu_config.h"
#include "ecu_types.h"
#include "vehicle_types.h"

typedef struct {
    uint32_t apply_count;
    uint32_t last_output_mask;
    ecu_device_apply_result_t last_result;
} local_io_device_state_t;

/* Initialize the CPU0-owned local IO adapter.
 *
 * Owner: task_io_service / vehicle executor path on CPU0.
 * ISR: not safe.
 */
void local_io_device_init(local_io_device_state_t *state);

/* Apply final horn, headlight and indicator requests to DIO.
 *
 * Units: all fields are logical outputs after safety clamping.
 * Dependencies: DIO masks and active polarity from hardware config.
 * Servo brake outputs are intentionally excluded; they are controlled through
 * the BC/BC2 drive terminal outputs over CANopen.
 * Failure behavior: invalid arguments return an error; this function does not
 * infer missing safety conditions.
 */
ecu_device_apply_result_t local_io_device_apply(local_io_device_state_t *state,
                                                dio_service_t *dio,
                                                const ecu_hardware_config_t *config,
                                                const vehicle_actuator_command_t *command);

#endif /* LOCAL_IO_DEVICE_H */
