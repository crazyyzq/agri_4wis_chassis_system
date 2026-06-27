#include <stdio.h>

#include "FreeRTOS.h"
#include "board.h"
#include "ecu_config.h"
#include "ecu_tasks.h"
#include "ipc_snapshot.h"
#include "task.h"

static ipc_snapshot_t s_cpu1_snapshot;

static void task_cpu1_service(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period_ticks = pdMS_TO_TICKS(ECU_CPU1_SERVICE_PERIOD_MS);
    for (;;) {
        ecu_task_cpu1_service_step((uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS));
        uint32_t heartbeat = (uint32_t)xTaskGetTickCount();
        (void)ipc_snapshot_publish(&s_cpu1_snapshot, &heartbeat, sizeof(heartbeat),
                                   (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS));
        vTaskDelayUntil(&last_wake, period_ticks);
    }
}

int main(void)
{
    board_init();
    ipc_snapshot_init(&s_cpu1_snapshot);
    printf("agri_chassis_control_cpu1 start\n");
    (void)xTaskCreate(task_cpu1_service, "cpu1_service", 512U, 0, 8U, 0);
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
