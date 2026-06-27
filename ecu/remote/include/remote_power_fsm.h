#ifndef REMOTE_POWER_FSM_H
#define REMOTE_POWER_FSM_H

#include "ecu_config.h"
#include "remote_types.h"

typedef struct {
    remote_power_state_t state;
    remote_position_t hold_position;
    uint32_t hold_since_ms;
    bool high_voltage_enable_request;
    bool orderly_shutdown_request;
    bool request_rejected;
    diag_code_t diagnostic;
} remote_power_fsm_t;

/* Initialize remote high-voltage request state.
 *
 * Owner: task_remote_manager.
 * ISR: not safe.
 * Failure behavior: null pointers are ignored.
 */
void remote_power_fsm_init(remote_power_fsm_t *fsm, uint32_t now_ms);

/* Update the remote power FSM from CH4.
 *
 * CH4 high held for config->power_long_press_ms requests high voltage.
 * CH4 low held for config->power_long_press_ms requests orderly shutdown.
 * Rejected requests are reported immediately and are not queued.
 */
void remote_power_fsm_update(remote_power_fsm_t *fsm,
                             const remote_input_snapshot_t *input,
                             const remote_preconditions_t *preconditions,
                             const ecu_config_t *config);

remote_power_state_t remote_power_fsm_get_state(const remote_power_fsm_t *fsm);

#endif /* REMOTE_POWER_FSM_H */
