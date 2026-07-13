#include <stdbool.h>

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

static ecu_motion_mode_t default_mode_for_domain(ecu_home_domain_t domain)
{
    if (domain == ECU_HOME_DOMAIN_SPECIAL) {
        return ECU_MOTION_MODE_SPIN;
    }
    return ECU_MOTION_MODE_POSITIVE_ACKERMANN;
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
    fsm->domain_changed_since_ms = now_ms;
    fsm->domain_default_pending = false;
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
    if (input->home == REMOTE_POS_INVALID) {
        /* An invalid HOME value must not silently select Ackermann and release
         * an adjustment-domain inhibit.  Hold the last accepted domain until
         * the debounced switch position is valid again.
         */
        fsm->request_rejected = true;
        fsm->diagnostic = DIAG_REJECT_MODE_PRECONDITION;
        return;
    }
    ecu_home_domain_t new_domain = domain_from_home(input->home);
    if (new_domain != fsm->requested_domain) {
        fsm->requested_domain = new_domain;
        fsm->domain_changed_since_ms = input->now_ms;
        fsm->domain_default_pending = true;
        return;
    }
    if (!ecu_time_elapsed(input->now_ms, fsm->domain_changed_since_ms, guard_ms)) {
        return; /* Ignore R1/R2 events until the new HOME domain is stable. */
    }
    /* R1/R2 are treated as edge events, not as held-level selectors.  Field
     * tests showed the physical button/wheel stable level can be high or low
     * depending on the transmitter state; any new stable R1 event selects the
     * R1 mode, and any new stable R2 event selects the R2 mode.  Release edges
     * are harmless because they re-select the same latched mode.
     */
    bool fresh_r1_event = input->r1_changed;
    bool fresh_r2_event = input->r2_changed;
    if (!fresh_r1_event && !fresh_r2_event && !fsm->domain_default_pending) {
        return;
    }
    if (!mode_preconditions_ok(preconditions)) {
        fsm->request_rejected = true;
        fsm->diagnostic = DIAG_REJECT_MODE_PRECONDITION;
        return;
    }
    fsm->active_domain = fsm->requested_domain;
    if (fsm->requested_domain == ECU_HOME_DOMAIN_SPECIAL) {
        fsm->requested_motion_mode = fresh_r2_event ?
                                     ECU_MOTION_MODE_CRAB :
                                     (fresh_r1_event ?
                                      ECU_MOTION_MODE_SPIN :
                                      default_mode_for_domain(fsm->requested_domain));
    } else {
        fsm->requested_motion_mode = fresh_r2_event ?
                                     ECU_MOTION_MODE_REVERSE_ACKERMANN :
                                     (fresh_r1_event ?
                                      ECU_MOTION_MODE_POSITIVE_ACKERMANN :
                                      default_mode_for_domain(fsm->requested_domain));
    }
    fsm->active_motion_mode = fsm->requested_motion_mode;
    fsm->domain_default_pending = false;
    fsm->diagnostic = DIAG_OK;
}

ecu_motion_mode_t remote_mode_fsm_get_state(const remote_mode_fsm_t *fsm)
{
    return fsm != 0 ? fsm->active_motion_mode : ECU_MOTION_MODE_POSITIVE_ACKERMANN;
}
