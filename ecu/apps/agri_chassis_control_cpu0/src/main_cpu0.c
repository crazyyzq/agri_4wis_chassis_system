#include <stdio.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "board.h"
#include "ecu_config.h"
#include "ecu_tasks.h"
#include "task.h"

/* Run one periodic ECU task using FreeRTOS absolute delay semantics.
 *
 * Keeping this helper small makes every task wrapper identical and prevents
 * task-specific busy waits from appearing in application startup code.
 */
static void run_periodic_task(void (*step)(uint32_t), uint32_t period_ms)
{
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period_ticks = pdMS_TO_TICKS(period_ms);
    for (;;) {
        step((uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS));
        vTaskDelayUntil(&last_wake, period_ticks);
    }
}

typedef struct {
    void (*entry)(void *);
    ecu_cpu0_task_id_t id;
} task_startup_entry_t;

static void task_safety_supervisor(void *arg)
{
    (void)arg;
    run_periodic_task(ecu_task_safety_supervisor_step, ECU_CPU0_SAFETY_PERIOD_MS);
}

static void task_can2_motion(void *arg)
{
    (void)arg;
    run_periodic_task(ecu_task_can2_motion_step, ECU_CPU0_CAN2_MOTION_PERIOD_MS);
}

static void task_remote_manager(void *arg)
{
    (void)arg;
    run_periodic_task(ecu_task_remote_manager_step, ECU_CPU0_REMOTE_PERIOD_MS);
}

static void task_vehicle_control(void *arg)
{
    (void)arg;
    run_periodic_task(ecu_task_vehicle_control_step, ECU_CPU0_CONTROL_PERIOD_MS);
}

static void task_can1_power(void *arg)
{
    (void)arg;
    run_periodic_task(ecu_task_can1_power_step, ECU_CPU0_POWER_PERIOD_MS);
}

static void task_can3_lift_hydraulic(void *arg)
{
    (void)arg;
    run_periodic_task(ecu_task_can3_lift_hydraulic_step, ECU_CPU0_LIFT_HYD_PERIOD_MS);
}

static void task_io_service(void *arg)
{
    (void)arg;
    run_periodic_task(ecu_task_io_service_step, ECU_CPU0_IO_PERIOD_MS);
}

static void task_diag_cpu0(void *arg)
{
    (void)arg;
    run_periodic_task(ecu_task_diag_cpu0_step, ECU_CPU0_DIAG_PERIOD_MS);
}

static const task_startup_entry_t s_startup_tasks[] = {
    { task_safety_supervisor, ECU_TASK_SAFETY_SUPERVISOR },
    { task_can2_motion, ECU_TASK_CAN2_MOTION },
    { task_remote_manager, ECU_TASK_REMOTE_MANAGER },
    { task_vehicle_control, ECU_TASK_VEHICLE_CONTROL },
    { task_can1_power, ECU_TASK_CAN1_POWER },
    { task_can3_lift_hydraulic, ECU_TASK_CAN3_LIFT_HYDRAULIC },
    { task_io_service, ECU_TASK_IO_SERVICE },
    { task_diag_cpu0, ECU_TASK_DIAG_CPU0 }
};

static bool create_task_or_report(void (*entry)(void *), ecu_cpu0_task_id_t id)
{
    const ecu_task_descriptor_t *desc = ecu_cpu0_task_descriptor(id);
    if (desc == 0) {
        printf("[ECU] FATAL invalid task id=%d\r\n", (int)id);
        return false;
    }
    BaseType_t result = xTaskCreate(entry, desc->name, (uint16_t)desc->stack_words,
                                    0, (UBaseType_t)desc->priority, 0);
    if (result != pdPASS) {
        printf("[ECU] FATAL create task failed: %s stack=%lu priority=%lu\r\n",
               desc->name,
               (unsigned long)desc->stack_words,
               (unsigned long)desc->priority);
        return false;
    }
    printf("[ECU] task created: %s period=%lums priority=%lu\r\n",
           desc->name,
           (unsigned long)desc->period_ms,
           (unsigned long)desc->priority);
    return true;
}

int main(void)
{
    board_init();
    printf("\r\n[ECU] ECU CPU0 boot: agri_chassis_control_cpu0\r\n");
    ecu_task_runtime_init(0U);

    bool tasks_ok = true;
    for (uint32_t index = 0U;
         index < (sizeof(s_startup_tasks) / sizeof(s_startup_tasks[0]));
         ++index) {
        tasks_ok = create_task_or_report(s_startup_tasks[index].entry,
                                         s_startup_tasks[index].id) && tasks_ok;
    }
    if (!tasks_ok) {
        printf("[ECU] FATAL scheduler not started because task creation failed\r\n");
        for (;;) {
        }
    }

    printf("[ECU] starting FreeRTOS scheduler\r\n");
    vTaskStartScheduler();
    printf("[ECU] FATAL scheduler returned unexpectedly\r\n");
    for (;;) {
    }
}

void vApplicationMallocFailedHook(void)
{
    printf("[ECU] FATAL malloc failed\r\n");
    taskDISABLE_INTERRUPTS();
    for (;;) {
    }
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *name)
{
    (void)task;
    printf("[ECU] FATAL stack overflow task=%s\r\n", name != 0 ? name : "unknown");
    taskDISABLE_INTERRUPTS();
    for (;;) {
    }
}
