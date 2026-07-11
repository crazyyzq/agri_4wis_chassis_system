#ifndef LOCAL_IO_DEVICE_H
#define LOCAL_IO_DEVICE_H

#include "ecu_config.h"
#include "ecu_types.h"
#include "vehicle_types.h"

typedef struct {
    uint32_t apply_count;
    uint32_t last_output_mask;
    bool high_voltage_relay_latched;
    ecu_device_apply_result_t last_result;
} local_io_device_state_t;

/* Initialize the CPU0-owned local IO adapter.
 *
 * Owner: task_io_service / vehicle executor path on CPU0.
 * ISR: not safe.
 */
void local_io_device_init(local_io_device_state_t *state);

/* Build the final board-level non-hydraulic output request.
 *
 * Units: all fields are logical outputs after safety clamping.
 * Dependencies: DIO masks and active polarity from hardware config.
 * Relay-box wiring v1.5 assigns MOS6 / EX_OUT6 to the battery-key
 * high-voltage request output.  The physical output is latched: a
 * high-voltage-enable command turns MOS6 on and keeps it on; an explicit
 * high-voltage-disable request or safety clamp turns it off.  The relay-box
 * control input is low-level active, but the ECU GPIO command is active high:
 * GPIO high turns the MOS output on and pulls the relay-box control terminal
 * low.  MOS8 / EX_OUT8 is reserved and must not be used for this function.
 * Servo brake outputs are intentionally excluded; they are controlled through
 * the BC/BC2 drive terminal outputs over CANopen.
 * Failure behavior: invalid arguments return an error; this function does not
 * infer missing safety conditions.
 */
ecu_device_apply_result_t local_io_device_apply(local_io_device_state_t *state,
                                                const ecu_hardware_config_t *config,
                                                const vehicle_actuator_command_t *command);

#endif /* LOCAL_IO_DEVICE_H */
