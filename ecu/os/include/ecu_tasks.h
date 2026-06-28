#ifndef ECU_TASKS_H
#define ECU_TASKS_H

#include <stdint.h>

typedef enum {
    ECU_TASK_SAFETY_SUPERVISOR = 0,
    ECU_TASK_CAN2_MOTION,
    ECU_TASK_REMOTE_MANAGER,
    ECU_TASK_VEHICLE_CONTROL,
    ECU_TASK_CAN1_POWER,
    ECU_TASK_CAN3_LIFT_HYDRAULIC,
    ECU_TASK_IO_SERVICE,
    ECU_TASK_DIAG_CPU0,
    ECU_TASK_CPU0_COUNT
} ecu_cpu0_task_id_t;

typedef struct {
    const char *name;
    uint32_t period_ms;
    uint32_t priority;
    uint32_t stack_words;
} ecu_task_descriptor_t;

const ecu_task_descriptor_t *ecu_cpu0_task_descriptor(ecu_cpu0_task_id_t task_id);

void ecu_task_runtime_init(uint32_t now_ms);
void ecu_task_safety_supervisor_step(uint32_t now_ms);
void ecu_task_can2_motion_step(uint32_t now_ms);
void ecu_task_remote_manager_step(uint32_t now_ms);
void ecu_task_vehicle_control_step(uint32_t now_ms);
void ecu_task_can1_power_step(uint32_t now_ms);
void ecu_task_can3_lift_hydraulic_step(uint32_t now_ms);
void ecu_task_io_service_step(uint32_t now_ms);
void ecu_task_diag_cpu0_step(uint32_t now_ms);
void ecu_task_cpu1_service_step(uint32_t now_ms);

#endif /* ECU_TASKS_H */
