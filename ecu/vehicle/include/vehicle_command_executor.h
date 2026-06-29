#ifndef VEHICLE_COMMAND_EXECUTOR_H
#define VEHICLE_COMMAND_EXECUTOR_H

#include <stdint.h>

#include "canopen_master_service.h"
#include "dio_service.h"
#include "modbus_master_service.h"
#include "uart_rs485_hw.h"
#include "vehicle_types.h"

typedef struct {
    canopen_master_service_t *can2_motion_canopen;
    canopen_master_service_t *can3_lift_hydraulic_canopen;
    dio_service_t *dio;
    uart_rs485_hw_t *warning_light_uart;
    modbus_master_service_t *warning_light_modbus;
} vehicle_executor_io_t;

void vehicle_command_executor_init(vehicle_executor_state_t *executor);
bool vehicle_command_executor_apply(vehicle_executor_state_t *executor,
                                    const vehicle_executor_io_t *io,
                                    const vehicle_actuator_command_t *command,
                                    uint32_t now_ms);
void vehicle_command_executor_get_state(const vehicle_executor_state_t *executor,
                                        vehicle_executor_state_t *out);

#endif /* VEHICLE_COMMAND_EXECUTOR_H */
