#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "command_arbiter.h"
#include "ecu_config.h"
#include "lift_hydraulic_device.h"
#include "local_io_device.h"
#include "motion_device.h"
#include "vehicle_command_executor.h"
#include "warning_light_device.h"

typedef struct {
    bool initialized;
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
                                    const vehicle_executor_io_t *io,
                                    const vehicle_actuator_command_t *command,
                                    uint32_t now_ms)
{
    const ecu_hardware_config_t *config = ecu_hardware_config_default();

    if (executor == 0 || io == 0 || command == 0 ||
        io->can2_motion_canopen == 0 || io->can3_lift_hydraulic_canopen == 0 ||
        io->dio == 0 || io->warning_light_uart == 0 ||
        io->warning_light_modbus == 0) {
        return false;
    }

    vehicle_executor_runtime_init_once();
    executor->motion_result = motion_device_apply(&s_runtime.motion,
                                                  io->can2_motion_canopen,
                                                  config,
                                                  command,
                                                  now_ms);
    executor->lift_hydraulic_result =
        lift_hydraulic_device_apply(&s_runtime.lift_hydraulic,
                                    io->can3_lift_hydraulic_canopen,
                                    io->dio,
                                    config,
                                    command,
                                    now_ms);
    executor->local_io_result = local_io_device_apply(&s_runtime.local_io,
                                                      io->dio,
                                                      config,
                                                      command);
    executor->warning_light_result =
        warning_light_device_apply(&s_runtime.warning_light,
                                   io->warning_light_modbus,
                                   io->warning_light_uart,
                                   config,
                                   command->indicator_mode,
                                   now_ms);
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
