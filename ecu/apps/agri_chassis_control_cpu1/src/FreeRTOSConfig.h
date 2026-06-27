#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include "board.h"

#if (portasmHAS_MTIME == 0)
#define configMTIME_BASE_ADDRESS                (0)
#define configMTIMECMP_BASE_ADDRESS             (0)
#else
#define configMTIME_BASE_ADDRESS                (HPM_MCHTMR_BASE)
#define configMTIMECMP_BASE_ADDRESS             (HPM_MCHTMR_BASE + 8UL)
#endif

#define configUSE_PREEMPTION                    1
#define configCPU_CLOCK_HZ                      ((uint32_t)24000000)
#define configTICK_RATE_HZ                      ((TickType_t)1000)
#define configMAX_PRIORITIES                    (16)
#define configMINIMAL_STACK_SIZE                (256)
#define configMAX_TASK_NAME_LEN                 16
#define configUSE_16_BIT_TICKS                  0
#define configUSE_MUTEXES                       1
#define configSUPPORT_STATIC_ALLOCATION         0
#define configSUPPORT_DYNAMIC_ALLOCATION        1
#define configTOTAL_HEAP_SIZE                   ((size_t)(32 * 1024))
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0
#define configCHECK_FOR_STACK_OVERFLOW          1
#define configUSE_MALLOC_FAILED_HOOK            1
#define configUSE_TRACE_FACILITY                1
#define configUSE_TIMERS                        1
#define configTIMER_TASK_PRIORITY               (configMAX_PRIORITIES - 1)
#define configTIMER_QUEUE_LENGTH                8
#define configTIMER_TASK_STACK_DEPTH            (configMINIMAL_STACK_SIZE)

#define INCLUDE_vTaskPrioritySet                1
#define INCLUDE_uxTaskPriorityGet               1
#define INCLUDE_vTaskDelete                     1
#define INCLUDE_vTaskDelayUntil                 1
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_xTaskGetCurrentTaskHandle       1
#define INCLUDE_xTaskGetSchedulerState          1

#define configSETUP_TICK_INTERRUPT() vConfigureTickInterrupt()
#define configCLEAR_TICK_INTERRUPT() vClearTickInterrupt()
#define configASSERT(x) if ((x) == 0) { taskDISABLE_INTERRUPTS(); __asm volatile("ebreak"); for (;;); }

#endif /* FREERTOS_CONFIG_H */
