#include "remote_adjust_fsm.h"

#include "ecu_config.h"

static void clear_track_request(remote_adjust_fsm_t *fsm)
{
    if (fsm == 0) {
        return;
    }
    fsm->track_direction = 0;
    fsm->pending_track_direction = 0;
    fsm->track_request_since_ms = 0U;
}

static bool adjust_preconditions_ok(const remote_adjust_fsm_t *fsm,
                                    const remote_input_snapshot_t *input,
                                    const remote_preconditions_t *preconditions)
{
    const bool entering_adjust_domain =
        fsm == 0 ||
        fsm->state == ADJUST_STATE_IDLE ||
        fsm->state == ADJUST_STATE_ABORTED;

    return input->home == REMOTE_POS_CENTER &&
           preconditions->active_gear == ECU_GEAR_REQUEST_P &&
           (!entering_adjust_domain || preconditions->zero_speed) &&
           preconditions->throttle_low &&
           !preconditions->estop_latched &&
           !preconditions->a_class_fault;
}

static bool clearance_request_active(const remote_input_snapshot_t *input)
{
    return input->clearance == REMOTE_POS_LOW ||
           input->clearance == REMOTE_POS_HIGH;
}

static int8_t raw_track_direction(const remote_adjust_fsm_t *fsm,
                                  const remote_input_snapshot_t *input)
{
    if (input == 0) {
        return 0;
    }

    /* Hysteresis is directional.  Once a valid extend/retract command is
     * latched, keep it until CH14 returns well into the center band.  This
     * avoids the valve/pump command oscillating if the analog value vibrates
     * around the engage threshold.
     */
    if (fsm != 0 && fsm->track_direction > 0 &&
        input->track_per_mille >= ECU_REMOTE_TRACK_EXTEND_RELEASE_PER_MILLE_MIN) {
        return 1;
    }
    if (fsm != 0 && fsm->track_direction < 0 &&
        input->track_per_mille <= ECU_REMOTE_TRACK_RETRACT_RELEASE_PER_MILLE_MAX) {
        return -1;
    }

    if (input->track_per_mille >= ECU_REMOTE_TRACK_EXTEND_PER_MILLE_MIN) {
        return 1;
    }
    if (input->track_per_mille <= ECU_REMOTE_TRACK_RETRACT_PER_MILLE_MAX) {
        return -1;
    }
    return 0;
}

static int8_t update_stable_track_direction(remote_adjust_fsm_t *fsm,
                                            const remote_input_snapshot_t *input)
{
    int8_t raw_direction = raw_track_direction(fsm, input);

    if (raw_direction == 0) {
        clear_track_request(fsm);
        return 0;
    }

    if (fsm->track_direction == raw_direction) {
        fsm->pending_track_direction = raw_direction;
        fsm->track_request_since_ms = input->now_ms;
        return raw_direction;
    }

    if (fsm->pending_track_direction != raw_direction) {
        fsm->pending_track_direction = raw_direction;
        fsm->track_request_since_ms = input->now_ms;
        return 0;
    }

    if ((uint32_t)(input->now_ms - fsm->track_request_since_ms) >=
        ECU_REMOTE_TRACK_REQUEST_STABLE_MS) {
        fsm->track_direction = raw_direction;
        return raw_direction;
    }

    return 0;
}

static bool track_owner_center_exit_active(const remote_adjust_fsm_t *fsm,
                                           const remote_input_snapshot_t *input)
{
    return fsm != 0 &&
           input != 0 &&
           fsm->adjust_owner == REMOTE_ADJUST_OWNER_TRACK &&
           fsm->track_direction == 0;
}

static bool track_request_pending(const remote_adjust_fsm_t *fsm)
{
    return fsm != 0 && fsm->pending_track_direction != 0;
}

static bool suspension_request_active(const remote_input_snapshot_t *input)
{
    if (input == 0 || input->gear == REMOTE_POS_CENTER) {
        return false;
    }
    return input->gear == REMOTE_POS_LOW ||
           input->gear == REMOTE_POS_HIGH;
}

void remote_adjust_fsm_init(remote_adjust_fsm_t *fsm)
{
    if (fsm == 0) {
        return;
    }
    fsm->state = ADJUST_STATE_IDLE;
    fsm->adjust_owner = REMOTE_ADJUST_OWNER_NONE;
    fsm->hydraulic_suspension_target = REMOTE_HYDRAULIC_SUSPENSION_FRONT;
    fsm->track_center_since_ms = 0U;
    fsm->track_request_since_ms = 0U;
    fsm->track_direction = 0;
    fsm->pending_track_direction = 0;
    fsm->request_rejected = false;
    fsm->diagnostic = DIAG_OK;
}

void remote_adjust_fsm_update(remote_adjust_fsm_t *fsm,
                              const remote_input_snapshot_t *input,
                              const remote_preconditions_t *preconditions)
{
    if (fsm == 0 || input == 0 || preconditions == 0) {
        return;
    }
    fsm->request_rejected = false;
    if (input->home != REMOTE_POS_CENTER) {
        fsm->state = ADJUST_STATE_IDLE;
        fsm->adjust_owner = REMOTE_ADJUST_OWNER_NONE;
        fsm->track_center_since_ms = 0U;
        clear_track_request(fsm);
        fsm->diagnostic = DIAG_OK;
        return;
    }
    if (!adjust_preconditions_ok(fsm, input, preconditions)) {
        /* HOME-center remains an adjustment-domain boundary even when an
         * adjustment command is rejected.  Do not fall back to IDLE here:
         * command arbitration must still suppress normal remote drive/steer
         * commands while the operator is in the adjustment domain.  CH1
         * steering is intentionally ignored in this domain, not used as an
         * abort condition.  Zero speed is required to enter the adjustment
         * domain, but not to stay in it after track-width assist current has
         * been commanded; otherwise the assist current's own velocity feedback
         * would abort the track operation and chatter the hydraulic valve.
         */
        fsm->state = ADJUST_STATE_ABORTED;
        fsm->adjust_owner = REMOTE_ADJUST_OWNER_NONE;
        fsm->track_center_since_ms = 0U;
        clear_track_request(fsm);
        fsm->diagnostic = DIAG_TRACK_ADJUST_ABORTED;
        return;
    }

    if (input->r1_changed) {
        fsm->hydraulic_suspension_target = REMOTE_HYDRAULIC_SUSPENSION_FRONT;
    }
    if (input->r2_changed) {
        fsm->hydraulic_suspension_target = REMOTE_HYDRAULIC_SUSPENSION_REAR;
    }

    bool clearance_active = clearance_request_active(input);
    int8_t track_direction = update_stable_track_direction(fsm, input);
    bool track_active = track_direction != 0;
    bool suspension_active = suspension_request_active(input);

    if (track_active) {
        fsm->track_center_since_ms = 0U;
        if (fsm->adjust_owner != REMOTE_ADJUST_OWNER_TRACK) {
            fsm->adjust_owner = REMOTE_ADJUST_OWNER_TRACK;
            fsm->state = ADJUST_STATE_TRACK_PREPARE;
        } else {
            fsm->state = ADJUST_STATE_TRACK_ACTIVE;
        }
        fsm->diagnostic = DIAG_OK;
        return;
    }

    if (track_owner_center_exit_active(fsm, input)) {
        if (fsm->track_center_since_ms == 0U) {
            fsm->track_center_since_ms = input->now_ms;
        }
        fsm->state = ADJUST_STATE_TRACK_EXITING;
        fsm->diagnostic = DIAG_OK;
        if ((uint32_t)(input->now_ms - fsm->track_center_since_ms) >=
            ECU_REMOTE_TRACK_ASSIST_CENTER_EXIT_MS) {
            fsm->adjust_owner = REMOTE_ADJUST_OWNER_NONE;
            fsm->track_center_since_ms = 0U;
            clear_track_request(fsm);
            fsm->state = ADJUST_STATE_READY;
        }
        return;
    }

    if (!clearance_active && !track_active && !suspension_active) {
        fsm->adjust_owner = REMOTE_ADJUST_OWNER_NONE;
        fsm->state = ADJUST_STATE_READY;
        fsm->track_center_since_ms = 0U;
        /* Do not clear a pending CH14 track-width request here.  The analog
         * wheel must remain beyond the engage threshold for
         * ECU_REMOTE_TRACK_REQUEST_STABLE_MS before it is accepted; clearing
         * pending state every 5 ms makes that debounce impossible and prevents
         * the track posture from ever being commanded.
         */
        if (!track_request_pending(fsm)) {
            clear_track_request(fsm);
        }
        fsm->diagnostic = DIAG_OK;
        return;
    }

    if (suspension_active) {
        fsm->adjust_owner = REMOTE_ADJUST_OWNER_HYDRAULIC;
        fsm->state = ADJUST_STATE_HYDRAULIC_ACTIVE;
        fsm->track_center_since_ms = 0U;
        clear_track_request(fsm);
        fsm->diagnostic = DIAG_OK;
        return;
    }

    if (clearance_active) {
        fsm->adjust_owner = REMOTE_ADJUST_OWNER_CLEARANCE;
        fsm->state = ADJUST_STATE_CLEARANCE_ACTIVE;
        fsm->track_center_since_ms = 0U;
        clear_track_request(fsm);
        fsm->diagnostic = DIAG_OK;
    }
}

remote_adjust_state_t remote_adjust_fsm_get_state(const remote_adjust_fsm_t *fsm)
{
    return fsm != 0 ? fsm->state : ADJUST_STATE_ABORTED;
}

int8_t remote_adjust_fsm_get_track_direction(const remote_adjust_fsm_t *fsm)
{
    return fsm != 0 ? fsm->track_direction : 0;
}
