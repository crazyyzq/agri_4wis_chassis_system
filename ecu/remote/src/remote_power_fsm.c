#include "ecu_time.h"
#include "remote_power_fsm.h"

static bool power_on_preconditions_ok(const remote_preconditions_t *preconditions)
{
    return preconditions->active_gear == ECU_GEAR_REQUEST_P &&
           preconditions->zero_speed &&
           preconditions->throttle_low &&
           preconditions->steering_neutral &&
           preconditions->arm_ready &&
           !preconditions->estop_latched &&
           !preconditions->a_class_fault &&
           preconditions->power_ready &&
           preconditions->low_voltage_ok &&
           preconditions->can1_power_online;
}

static bool power_down_preconditions_ok(const remote_preconditions_t *preconditions)
{
    return preconditions->active_gear == ECU_GEAR_REQUEST_P &&
           preconditions->zero_speed &&
           preconditions->brake_applied &&
           preconditions->hydraulic_stopped &&
           !preconditions->adjustment_active &&
           !preconditions->active_transition &&
           !preconditions->estop_latched;
}

void remote_power_fsm_init(remote_power_fsm_t *fsm, uint32_t now_ms)
{
    if (fsm == 0) {
        return;
    }
    fsm->state = REMOTE_POWER_OFF;
    fsm->hold_position = REMOTE_POS_CENTER;
    fsm->hold_since_ms = now_ms;
    fsm->high_voltage_enable_request = false;
    fsm->orderly_shutdown_request = false;
    fsm->request_rejected = false;
    fsm->diagnostic = DIAG_OK;
}

void remote_power_fsm_update(remote_power_fsm_t *fsm,
                             const remote_input_snapshot_t *input,
                             const remote_preconditions_t *preconditions,
                             const ecu_config_t *config)
{
    if (fsm == 0 || input == 0 || preconditions == 0 || config == 0) {
        return;
    }

    fsm->request_rejected = false;
    fsm->orderly_shutdown_request = false;

    if (preconditions->estop_latched || preconditions->a_class_fault) {
        fsm->state = REMOTE_POWER_SHUTDOWN_PROTECT;
        fsm->high_voltage_enable_request = false;
        fsm->diagnostic = preconditions->a_class_fault ? DIAG_A_CLASS_FAULT : DIAG_CONTROLLED_STOP_ACTIVE;
        return;
    }

    if (input->power != REMOTE_POS_HIGH && input->power != REMOTE_POS_LOW) {
        fsm->hold_position = input->power;
        fsm->hold_since_ms = input->now_ms;
        fsm->diagnostic = DIAG_OK;
        return;
    }

    if (input->power != fsm->hold_position) {
        fsm->hold_position = input->power;
        fsm->hold_since_ms = input->now_ms;
        return;
    }

    if (!ecu_time_elapsed(input->now_ms, fsm->hold_since_ms, config->power_long_press_ms)) {
        fsm->state = input->power == REMOTE_POS_HIGH ?
                     REMOTE_POWER_ON_REQUESTED : REMOTE_POWER_DOWN_REQUESTED;
        return;
    }

    if (input->power == REMOTE_POS_HIGH) {
        if (power_on_preconditions_ok(preconditions)) {
            fsm->state = REMOTE_POWER_ON;
            fsm->high_voltage_enable_request = true;
            fsm->diagnostic = DIAG_OK;
        } else {
            fsm->state = REMOTE_POWER_REJECTED;
            fsm->high_voltage_enable_request = false;
            fsm->request_rejected = true;
            fsm->diagnostic = DIAG_REJECT_POWER_PRECONDITION;
        }
    } else {
        if (power_down_preconditions_ok(preconditions)) {
            fsm->state = REMOTE_POWER_OFF;
            fsm->high_voltage_enable_request = false;
            fsm->orderly_shutdown_request = true;
            fsm->diagnostic = DIAG_OK;
        } else {
            fsm->state = REMOTE_POWER_REJECTED;
            fsm->request_rejected = true;
            fsm->diagnostic = DIAG_REJECT_POWER_PRECONDITION;
        }
    }
}

remote_power_state_t remote_power_fsm_get_state(const remote_power_fsm_t *fsm)
{
    return fsm != 0 ? fsm->state : REMOTE_POWER_REJECTED;
}
