#ifndef REMOTE_POWER_FSM_H
#define REMOTE_POWER_FSM_H

#include "ecu_config.h"
#include "remote_types.h"

/* Bit mask reported when the operator requests high voltage but one or more
 * startup interlocks are not satisfied.  These bits are diagnostic evidence
 * only; the power FSM remains the sole owner of the actual acceptance policy.
 */
typedef enum {
    REMOTE_POWER_BLOCK_GEAR_NOT_P          = (1U << 0),
    REMOTE_POWER_BLOCK_SPEED_NOT_ZERO      = (1U << 1),
    REMOTE_POWER_BLOCK_THROTTLE_NOT_LOW    = (1U << 2),
    REMOTE_POWER_BLOCK_STEERING_NOT_CENTER = (1U << 3),
    REMOTE_POWER_BLOCK_ARM_NOT_READY       = (1U << 4),
    REMOTE_POWER_BLOCK_ESTOP_LATCHED       = (1U << 5),
    REMOTE_POWER_BLOCK_A_CLASS_FAULT       = (1U << 6),
    REMOTE_POWER_BLOCK_CAN1_POWER_OFFLINE  = (1U << 7)
} remote_power_block_t;

typedef struct {
    remote_power_state_t state;
    remote_position_t hold_position;
    uint32_t hold_since_ms;
    uint16_t power_on_block_mask;
    bool high_voltage_enable_request;
    bool high_voltage_disable_request;
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
 * CH4 low held for config->power_long_press_ms requests high-voltage release.
 * The release request is emitted even if orderly shutdown preconditions reject
 * the higher-level state transition, because the logical high-voltage request
 * must not remain latched after the operator explicitly disables high voltage.
 * The field value is 350 ms. CH4 high uses the shared
 * ECU_SBUS_PPM_HIGH_MIN..ECU_SBUS_PPM_HIGH threshold range, so the power
 * gesture remains calibration-owned rather than duplicated here.
 * Rejected requests are reported immediately and are not queued.
 */
void remote_power_fsm_update(remote_power_fsm_t *fsm,
                             const remote_input_snapshot_t *input,
                             const remote_preconditions_t *preconditions,
                             const ecu_config_t *config);

remote_power_state_t remote_power_fsm_get_state(const remote_power_fsm_t *fsm);

#endif /* REMOTE_POWER_FSM_H */
