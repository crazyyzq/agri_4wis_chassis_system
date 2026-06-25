/* Debugger-controlled live monitor for bench bring-up visibility. */
#ifndef ECU_DEBUG_MONITOR_H
#define ECU_DEBUG_MONITOR_H

#include <stdbool.h>
#include <stdint.h>

#include "sbus_service.h"

typedef enum {
    ECU_DEBUG_VIEW_NONE = 0,
    ECU_DEBUG_VIEW_SBUS = 1,
    ECU_DEBUG_VIEW_ADC  = 2,
    ECU_DEBUG_VIEW_DI   = 3,
    ECU_DEBUG_VIEW_DO   = 4,
    ECU_DEBUG_VIEW_ALL  = 5
} ecu_debug_view_t;

typedef struct {
    volatile uint32_t enable;
    volatile uint32_t view;
    volatile uint32_t channel;
    volatile uint32_t period_ms;
    volatile uint32_t do_enable;
    volatile uint32_t do_mask;
} ecu_debug_monitor_t;

typedef struct {
    uint32_t (*now_ms)(void);
    bool (*read_sbus_state)(sbus_debug_state_t *state);
    bool (*read_adc_mv)(uint8_t channel, uint32_t *mv);
    uint8_t (*read_di)(uint8_t channel);
    void (*write_do_mask)(uint32_t mask);
    void (*write_line)(const char *line);
} ecu_debug_monitor_backend_t;

extern volatile ecu_debug_monitor_t g_ecu_debug_monitor;

void ecu_debug_monitor_init(void);
void ecu_debug_monitor_poll(void);
void ecu_debug_monitor_suspend(void);
void ecu_debug_monitor_resume(void);
void ecu_debug_monitor_use_backend(const ecu_debug_monitor_backend_t *backend);
void ecu_debug_monitor_restore_default_backend(void);

#endif
