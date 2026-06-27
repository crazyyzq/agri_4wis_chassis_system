#include "remote_estop_fsm.h"

static ecu_estop_source_t detect_estop_source(const remote_input_snapshot_t *input)
{
    if (input->ch13_estop == REMOTE_POS_HIGH) return ECU_ESTOP_SOURCE_CH13;
    if (input->sbus_timeout) return ECU_ESTOP_SOURCE_SBUS_TIMEOUT;
    if (input->sbus_failsafe) return ECU_ESTOP_SOURCE_SBUS_FAILSAFE;
    if (input->decode_error_limit) return ECU_ESTOP_SOURCE_DECODE_ERRORS;
    if (input->credibility_error) return ECU_ESTOP_SOURCE_CREDIBILITY;
    return ECU_ESTOP_SOURCE_NONE;
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
    fsm->reset_since_ms = now_ms;
    fsm->diagnostic = DIAG_OK;
}

void remote_estop_fsm_update(remote_estop_fsm_t *fsm,
                             const remote_input_snapshot_t *input,
                             const remote_preconditions_t *preconditions)
{
    if (fsm == 0 || input == 0 || preconditions == 0) {
        return;
    }
    ecu_estop_source_t estop_source = detect_estop_source(input);
    if (estop_source != ECU_ESTOP_SOURCE_NONE) {
        fsm->state = REMOTE_ESTOP_LATCHED;
        fsm->estop_source = estop_source;
        fsm->diagnostic = diag_from_source(estop_source);
        return;
    }
    switch (fsm->state) {
    case REMOTE_ESTOP_LATCHED:
        if (input->ch13_estop == REMOTE_POS_CENTER &&
            preconditions->zero_speed &&
            preconditions->brake_applied &&
            !preconditions->a_class_fault) {
            fsm->state = REMOTE_ESTOP_RESET_REQUESTED;
            fsm->reset_since_ms = input->now_ms;
        }
        break;
    case REMOTE_ESTOP_RESET_REQUESTED:
        if (preconditions->zero_speed && preconditions->brake_applied) {
            fsm->state = REMOTE_ESTOP_CLEAR_WAIT_NORMAL;
        }
        break;
    case REMOTE_ESTOP_CLEAR_WAIT_NORMAL:
        if (input->ch13_estop == REMOTE_POS_LOW) {
            fsm->state = REMOTE_ESTOP_CLEAR;
            fsm->estop_source = ECU_ESTOP_SOURCE_NONE;
            fsm->diagnostic = DIAG_OK;
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
