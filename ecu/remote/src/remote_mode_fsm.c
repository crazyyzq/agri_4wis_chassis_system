#include "ecu_time.h"
#include "remote_mode_fsm.h"

static ecu_home_domain_t domain_from_home(remote_position_t home)
{
    if (home == REMOTE_POS_CENTER) return ECU_HOME_DOMAIN_ADJUST;
    if (home == REMOTE_POS_HIGH) return ECU_HOME_DOMAIN_SPECIAL;
    return ECU_HOME_DOMAIN_ACKERMANN;
}

static bool mode_preconditions_ok(const remote_preconditions_t *preconditions)
{
    return preconditions->active_gear == ECU_GEAR_REQUEST_P &&
           preconditions->zero_speed &&
           preconditions->throttle_low &&
           preconditions->steering_neutral &&
           !preconditions->estop_latched &&
           !preconditions->a_class_fault &&
           !preconditions->active_transition;
}

void remote_mode_fsm_init(remote_mode_fsm_t *fsm, uint32_t now_ms)
{
    if (fsm == 0) {
        return;
    }
    fsm->requested_domain = ECU_HOME_DOMAIN_ACKERMANN;
    fsm->active_domain = ECU_HOME_DOMAIN_ACKERMANN;
    fsm->requested_motion_mode = ECU_MOTION_MODE_POSITIVE_ACKERMANN;
    fsm->active_motion_mode = ECU_MOTION_MODE_POSITIVE_ACKERMANN;
    fsm->domain_guard_until_ms = now_ms;
    fsm->request_rejected = false;
    fsm->diagnostic = DIAG_OK;
}

void remote_mode_fsm_update(remote_mode_fsm_t *fsm,
                            const remote_input_snapshot_t *input,
                            const remote_preconditions_t *preconditions,
                            uint32_t guard_ms)
{
    if (fsm == 0 || input == 0 || preconditions == 0) {
        return;
    }
    fsm->request_rejected = false;
    ecu_home_domain_t new_domain = domain_from_home(input->home);
    if (new_domain != fsm->requested_domain) {
        fsm->requested_domain = new_domain;
        fsm->domain_guard_until_ms = input->now_ms + guard_ms;
        return;
    }
    if (!ecu_time_elapsed(input->now_ms, fsm->domain_guard_until_ms, 0U)) {
        return; /* HOME domain guard suppresses old R1/R2 events. */
    }
    if (input->r1 != REMOTE_POS_HIGH && input->r2 != REMOTE_POS_HIGH) {
        return;
    }
    if (!mode_preconditions_ok(preconditions)) {
        fsm->request_rejected = true;
        fsm->diagnostic = DIAG_REJECT_MODE_PRECONDITION;
        return;
    }
    fsm->active_domain = fsm->requested_domain;
    if (fsm->requested_domain == ECU_HOME_DOMAIN_SPECIAL) {
        fsm->requested_motion_mode = input->r1 == REMOTE_POS_HIGH ?
                                     ECU_MOTION_MODE_SPIN : ECU_MOTION_MODE_CRAB;
    } else {
        fsm->requested_motion_mode = input->r1 == REMOTE_POS_HIGH ?
                                     ECU_MOTION_MODE_POSITIVE_ACKERMANN :
                                     ECU_MOTION_MODE_REVERSE_ACKERMANN;
    }
    fsm->active_motion_mode = fsm->requested_motion_mode;
    fsm->diagnostic = DIAG_OK;
}

ecu_motion_mode_t remote_mode_fsm_get_state(const remote_mode_fsm_t *fsm)
{
    return fsm != 0 ? fsm->active_motion_mode : ECU_MOTION_MODE_POSITIVE_ACKERMANN;
}
