#include <string.h>

#include "command_arbiter.h"
#include "can_bus_service.h"
#include "dio_service.h"
#include "ecu_config.h"
#include "lift_hydraulic_device.h"
#include "local_io_device.h"
#include "motion_device.h"
#include "power_device.h"
#include "uart_comm_service.h"
#include "vehicle_command_executor.h"
#include "warning_light_device.h"

typedef struct {
    bool initialized;
    can_bus_service_t can1_power;
    can_bus_service_t can2_motion;
    can_bus_service_t can3_lift_hydraulic;
    dio_service_t dio;
    uart_comm_service_t rs485;
    power_device_state_t power;
    motion_device_state_t motion;
    lift_hydraulic_device_state_t lift_hydraulic;
    local_io_device_state_t local_io;
    warning_light_device_state_t warning_light;
} vehicle_executor_runtime_t;

static vehicle_executor_runtime_t s_runtime;

static void vehicle_executor_runtime_init_once(void)
{
    if (s_runtime.initialized) {
        return;
    }
    const ecu_hardware_config_t *config = ecu_hardware_config_default();
    can_bus_service_init(&s_runtime.can1_power, config->can1_bitrate);
    can_bus_service_init(&s_runtime.can2_motion, config->can2_bitrate);
    can_bus_service_init(&s_runtime.can3_lift_hydraulic, config->can3_bitrate);
    dio_service_init(&s_runtime.dio,
                     config->dio_active_high,
                     config->dio_managed_output_mask);
    uart_comm_service_init(&s_runtime.rs485, config->rs485_baudrate);
    power_device_init(&s_runtime.power);
    motion_device_init(&s_runtime.motion);
    lift_hydraulic_device_init(&s_runtime.lift_hydraulic);
    local_io_device_init(&s_runtime.local_io);
    warning_light_device_init(&s_runtime.warning_light);
    s_runtime.initialized = true;
}

void vehicle_command_executor_init(vehicle_executor_state_t *executor)
{
    if (executor == 0) {
        return;
    }
    memset(executor, 0, sizeof(*executor));
    vehicle_actuator_command_safe_default(&executor->last_command);
    executor->power_result = ECU_DEVICE_APPLY_OK;
    executor->motion_result = ECU_DEVICE_APPLY_OK;
    executor->lift_hydraulic_result = ECU_DEVICE_APPLY_OK;
    executor->local_io_result = ECU_DEVICE_APPLY_OK;
    executor->warning_light_result = ECU_DEVICE_APPLY_OK;
    memset(&s_runtime, 0, sizeof(s_runtime));
    vehicle_executor_runtime_init_once();
}

bool vehicle_command_executor_apply(vehicle_executor_state_t *executor,
                                    const vehicle_actuator_command_t *command)
{
    if (executor == 0 || command == 0) {
        return false;
    }
    vehicle_executor_runtime_init_once();
    const ecu_hardware_config_t *config = ecu_hardware_config_default();
    executor->power_result = power_device_apply(&s_runtime.power,
                                                &s_runtime.can1_power,
                                                config,
                                                command->high_voltage_enable);
    executor->motion_result = motion_device_apply(&s_runtime.motion,
                                                  &s_runtime.can2_motion,
                                                  config,
                                                  command);
    executor->lift_hydraulic_result =
        lift_hydraulic_device_apply(&s_runtime.lift_hydraulic,
                                    &s_runtime.can3_lift_hydraulic,
                                    &s_runtime.dio,
                                    config,
                                    command);
    executor->local_io_result = local_io_device_apply(&s_runtime.local_io,
                                                      &s_runtime.dio,
                                                      config,
                                                      command);
    executor->warning_light_result =
        warning_light_device_apply(&s_runtime.warning_light,
                                   &s_runtime.rs485,
                                   config,
                                   command->indicator_mode);
    executor->last_command = *command;
    executor->applied_sequence++;
    return executor->power_result == ECU_DEVICE_APPLY_OK &&
           executor->motion_result == ECU_DEVICE_APPLY_OK &&
           executor->lift_hydraulic_result == ECU_DEVICE_APPLY_OK &&
           executor->local_io_result == ECU_DEVICE_APPLY_OK &&
           executor->warning_light_result == ECU_DEVICE_APPLY_OK;
}

void vehicle_command_executor_get_state(const vehicle_executor_state_t *executor,
                                        vehicle_executor_state_t *out)
{
    if (executor != 0 && out != 0) {
        *out = *executor;
    }
}
