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
    /* Legacy serial-monitor DO path. Prefer g_ecu_debug_mos for JLink live control. */
    volatile uint32_t do_enable;
    volatile uint32_t do_mask;
} ecu_debug_monitor_t;

typedef struct {
    /*
     * JLink/SEGGER live MOS output control register block.
     *
     * Watch/edit these fields directly:
     *   enable       = 1 lets this block own EX_OUT1..EX_OUT12.
     *   request_mask = bit0..bit11 request EX_OUT1..EX_OUT12 ON.
     *
     * Read these fields to confirm what firmware actually applied:
     *   applied_mask  = last sanitized mask written to board_ecu_output_write().
     *   write_count   = increments after every monitor poll write.
     *   last_write_ms = backend timestamp of the last write.
     *
     * When enable is 0, the old g_ecu_debug_monitor.do_enable/do_mask path still
     * works for compatibility with serial monitor tests.
     */
    volatile uint32_t enable;
    volatile uint32_t request_mask;
    volatile uint32_t applied_mask;
    volatile uint32_t write_count;
    volatile uint32_t last_write_ms;
} ecu_debug_mos_t;

typedef struct {
    uint32_t (*now_ms)(void);
    bool (*read_sbus_state)(sbus_debug_state_t *state);
    bool (*read_adc_mv)(uint8_t channel, uint32_t *mv);
    uint8_t (*read_di)(uint8_t channel);
    void (*write_do_mask)(uint32_t mask);
    void (*write_line)(const char *line);
} ecu_debug_monitor_backend_t;

extern volatile ecu_debug_monitor_t g_ecu_debug_monitor;
extern volatile ecu_debug_mos_t g_ecu_debug_mos;

void ecu_debug_monitor_init(void);
void ecu_debug_monitor_poll(void);
void ecu_debug_monitor_suspend(void);
void ecu_debug_monitor_resume(void);
void ecu_debug_monitor_use_backend(const ecu_debug_monitor_backend_t *backend);
void ecu_debug_monitor_restore_default_backend(void);

#endif
