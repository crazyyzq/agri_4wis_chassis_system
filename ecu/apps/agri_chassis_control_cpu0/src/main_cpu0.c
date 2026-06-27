#include <stdio.h>

#include "FreeRTOS.h"
#include "board.h"
#include "ecu_config.h"
#include "ecu_tasks.h"
#include "task.h"

static void run_periodic_task(void (*step)(uint32_t), uint32_t period_ms)
{
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period_ticks = pdMS_TO_TICKS(period_ms);
    for (;;) {
        step((uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS));
        vTaskDelayUntil(&last_wake, period_ticks);
    }
}

static void task_safety_supervisor(void *arg) { (void)arg; run_periodic_task(ecu_task_safety_supervisor_step, ECU_CPU0_SAFETY_PERIOD_MS); }
static void task_can2_motion(void *arg) { (void)arg; run_periodic_task(ecu_task_can2_motion_step, ECU_CPU0_CAN2_MOTION_PERIOD_MS); }
static void task_remote_manager(void *arg) { (void)arg; run_periodic_task(ecu_task_remote_manager_step, ECU_CPU0_REMOTE_PERIOD_MS); }
static void task_vehicle_control(void *arg) { (void)arg; run_periodic_task(ecu_task_vehicle_control_step, ECU_CPU0_CONTROL_PERIOD_MS); }
static void task_can1_power(void *arg) { (void)arg; run_periodic_task(ecu_task_can1_power_step, ECU_CPU0_POWER_PERIOD_MS); }
static void task_can3_lift_hydraulic(void *arg) { (void)arg; run_periodic_task(ecu_task_can3_lift_hydraulic_step, ECU_CPU0_LIFT_HYD_PERIOD_MS); }
static void task_io_service(void *arg) { (void)arg; run_periodic_task(ecu_task_io_service_step, ECU_CPU0_IO_PERIOD_MS); }
static void task_diag_cpu0(void *arg) { (void)arg; run_periodic_task(ecu_task_diag_cpu0_step, ECU_CPU0_DIAG_PERIOD_MS); }

static void create_task(void (*entry)(void *), ecu_cpu0_task_id_t id)
{
    const ecu_task_descriptor_t *desc = ecu_cpu0_task_descriptor(id);
    if (desc == 0) {
        return;
    }
    (void)xTaskCreate(entry, desc->name, (uint16_t)desc->stack_words,
                      0, (UBaseType_t)desc->priority, 0);
}

int main(void)
{
    board_init();
    printf("agri_chassis_control_cpu0 start\n");
    create_task(task_safety_supervisor, ECU_TASK_SAFETY_SUPERVISOR);
    create_task(task_can2_motion, ECU_TASK_CAN2_MOTION);
    create_task(task_remote_manager, ECU_TASK_REMOTE_MANAGER);
    create_task(task_vehicle_control, ECU_TASK_VEHICLE_CONTROL);
    create_task(task_can1_power, ECU_TASK_CAN1_POWER);
    create_task(task_can3_lift_hydraulic, ECU_TASK_CAN3_LIFT_HYDRAULIC);
    create_task(task_io_service, ECU_TASK_IO_SERVICE);
    create_task(task_diag_cpu0, ECU_TASK_DIAG_CPU0);
    vTaskStartScheduler();
    for (;;) {
    }
}

void vApplicationMallocFailedHook(void)
{
    taskDISABLE_INTERRUPTS();
    for (;;) {
    }
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *name)
{
    (void)task;
    (void)name;
    taskDISABLE_INTERRUPTS();
    for (;;) {
    }
}
