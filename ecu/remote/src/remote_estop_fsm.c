#include "ecu_config.h"
#include "ecu_time.h"
#include "remote_estop_fsm.h"

#include <stddef.h>

static ecu_estop_source_t detect_estop_source(const remote_input_snapshot_t *input)
{
    if (input->sbus_timeout) return ECU_ESTOP_SOURCE_SBUS_TIMEOUT;
    if (input->sbus_failsafe) return ECU_ESTOP_SOURCE_SBUS_FAILSAFE;
    if (input->decode_error_limit) return ECU_ESTOP_SOURCE_DECODE_ERRORS;
    if (input->credibility_error) return ECU_ESTOP_SOURCE_CREDIBILITY;
    return ECU_ESTOP_SOURCE_NONE;
}

static bool ch13_position_requests_estop(const remote_input_snapshot_t *input)
{
    return input != NULL &&
           (input->ch13_estop == REMOTE_POS_LOW ||
            input->ch13_estop == REMOTE_POS_HIGH);
}

static void update_center_hold(remote_estop_fsm_t *fsm,
                               const remote_input_snapshot_t *input)
{
    if (fsm == NULL || input == NULL) {
        return;
    }
    if (input->ch13_estop != REMOTE_POS_CENTER) {
        fsm->center_hold_active = false;
        return;
    }
    if (!fsm->center_hold_active) {
        fsm->center_hold_active = true;
        fsm->center_hold_since_ms = input->now_ms;
    }
}

static bool estop_reset_preconditions_met(const remote_estop_fsm_t *fsm,
                                          const remote_input_snapshot_t *input,
                                          const remote_preconditions_t *preconditions)
{
    return fsm != NULL &&
           input != NULL &&
           preconditions != NULL &&
           fsm->center_hold_active &&
           ecu_time_elapsed(input->now_ms,
                            fsm->center_hold_since_ms,
                            REMOTE_ESTOP_CENTER_HOLD_MS) &&
           preconditions->zero_speed &&
           preconditions->brake_applied &&
           !preconditions->a_class_fault;
}

static diag_code_t diag_from_source(ecu_estop_source_t estop_source)
{
    switch (estop_source) {
    case ECU_ESTOP_SOURCE_CH13: return DIAG_REMOTE_ESTOP_CH13;
    case ECU_ESTOP_SOURCE_SBUS_TIMEOUT: return DIAG_REMOTE_ESTOP_SBUS_TIMEOUT;
    case ECU_ESTOP_SOURCE_SBUS_FAILSAFE: return DIAG_REMOTE_ESTOP_FAILSAFE;
    case ECU_ESTOP_SOURCE_DECODE_ERRORS: return DIAG_REMOTE_ESTOP_DECODE_ERRORS;
    case ECU_ESTOP_SOURCE_CREDIBILITY: return DIAG_REMOTE_ESTOP_CREDIBILITY;
    case ECU_ESTOP_SOURCE_NONE:
    default: return DIAG_OK;
    }
}

void remote_estop_fsm_init(remote_estop_fsm_t *fsm, uint32_t now_ms)
{
    if (fsm == 0) {
        return;
    }
    fsm->state = REMOTE_ESTOP_CLEAR;
    fsm->estop_source = ECU_ESTOP_SOURCE_NONE;
    fsm->center_hold_since_ms = now_ms;
    fsm->center_hold_active = false;
    fsm->diagnostic = DIAG_OK;
}

void remote_estop_fsm_update(remote_estop_fsm_t *fsm,
                             const remote_input_snapshot_t *input,
                             const remote_preconditions_t *preconditions)
{
    if (fsm == 0 || input == 0 || preconditions == 0) {
        return;
    }
    /* CH13 is a three-position emergency-stop control.  Either endpoint is an
     * active stop request; center is the only reset position.  Evaluate the
     * debounced stable position directly instead of tracking an extra edge
     * baseline, because a raw-high shortcut previously advanced that baseline
     * before debounce completed and could lose the actual HIGH event.
     */
    if (ch13_position_requests_estop(input)) {
        fsm->center_hold_active = false;
        fsm->state = REMOTE_ESTOP_LATCHED;
        fsm->estop_source = ECU_ESTOP_SOURCE_CH13;
        fsm->diagnostic = DIAG_REMOTE_ESTOP_CH13;
        return;
    }

    ecu_estop_source_t estop_source = detect_estop_source(input);
    if (estop_source != ECU_ESTOP_SOURCE_NONE) {
        fsm->center_hold_active = false;
        fsm->state = REMOTE_ESTOP_LATCHED;
        fsm->estop_source = estop_source;
        fsm->diagnostic = diag_from_source(estop_source);
        return;
    }
    if (fsm->state == REMOTE_ESTOP_CLEAR) {
        fsm->center_hold_active = false;
    } else {
        update_center_hold(fsm, input);
    }
    switch (fsm->state) {
    case REMOTE_ESTOP_LATCHED:
        if (estop_reset_preconditions_met(fsm, input, preconditions)) {
            fsm->state = REMOTE_ESTOP_RESET_REQUESTED;
        }
        break;
    case REMOTE_ESTOP_RESET_REQUESTED:
        if (estop_reset_preconditions_met(fsm, input, preconditions)) {
            fsm->state = REMOTE_ESTOP_CLEAR_WAIT_NORMAL;
        } else {
            fsm->state = REMOTE_ESTOP_LATCHED;
        }
        break;
    case REMOTE_ESTOP_CLEAR_WAIT_NORMAL:
        if (estop_reset_preconditions_met(fsm, input, preconditions)) {
            fsm->state = REMOTE_ESTOP_CLEAR;
            fsm->estop_source = ECU_ESTOP_SOURCE_NONE;
            fsm->center_hold_active = false;
            fsm->diagnostic = DIAG_OK;
        } else {
            fsm->state = REMOTE_ESTOP_LATCHED;
        }
        break;
    case REMOTE_ESTOP_CLEAR:
    default:
        fsm->diagnostic = DIAG_OK;
        break;
    }
}

remote_estop_state_t remote_estop_fsm_get_state(const remote_estop_fsm_t *fsm)
{
    return fsm != 0 ? fsm->state : REMOTE_ESTOP_LATCHED;
}

ecu_estop_source_t remote_estop_fsm_get_source(const remote_estop_fsm_t *fsm)
{
    return fsm != 0 ? fsm->estop_source : ECU_ESTOP_SOURCE_CREDIBILITY;
}
