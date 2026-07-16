#include "ecu_time.h"
#include "remote_power_fsm.h"

static uint16_t power_on_precondition_block_mask(
    const remote_preconditions_t *preconditions)
{
    uint16_t mask = 0U;

    if (preconditions->active_gear != ECU_GEAR_REQUEST_P) {
        mask |= REMOTE_POWER_BLOCK_GEAR_NOT_P;
    }
    if (!preconditions->zero_speed) {
        mask |= REMOTE_POWER_BLOCK_SPEED_NOT_ZERO;
    }
    if (!preconditions->throttle_low) {
        mask |= REMOTE_POWER_BLOCK_THROTTLE_NOT_LOW;
    }
    if (!preconditions->steering_neutral) {
        mask |= REMOTE_POWER_BLOCK_STEERING_NOT_CENTER;
    }
    if (!preconditions->arm_ready) {
        mask |= REMOTE_POWER_BLOCK_ARM_NOT_READY;
    }
    if (preconditions->estop_latched) {
        mask |= REMOTE_POWER_BLOCK_ESTOP_LATCHED;
    }
    if (preconditions->a_class_fault) {
        mask |= REMOTE_POWER_BLOCK_A_CLASS_FAULT;
    }
    if (!preconditions->can1_power_online) {
        mask |= REMOTE_POWER_BLOCK_CAN1_POWER_OFFLINE;
    }
    return mask;
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

static void request_safe_power_down(remote_power_fsm_t *fsm,
                                    const remote_preconditions_t *preconditions)
{
    if (power_down_preconditions_ok(preconditions)) {
        fsm->high_voltage_enable_request = false;
        fsm->high_voltage_disable_request = true;
        fsm->state = REMOTE_POWER_OFF;
        fsm->orderly_shutdown_request = true;
        fsm->diagnostic = DIAG_OK;
        return;
    }

    /* Keep the battery-key relay latched while the vehicle is moving or
     * hydraulics are active. Removing high voltage before the controlled stop
     * and brake conditions are true would defeat the actuators needed to reach
     * a safe state.
     */
    fsm->state = REMOTE_POWER_REJECTED;
    fsm->request_rejected = true;
    fsm->diagnostic = DIAG_REJECT_POWER_PRECONDITION;
}

void remote_power_fsm_init(remote_power_fsm_t *fsm, uint32_t now_ms)
{
    if (fsm == 0) {
        return;
    }
    fsm->state = REMOTE_POWER_OFF;
    fsm->hold_position = REMOTE_POS_CENTER;
    fsm->hold_since_ms = now_ms;
    fsm->power_on_block_mask = 0U;
    fsm->high_voltage_enable_request = false;
    fsm->high_voltage_disable_request = false;
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
    fsm->high_voltage_disable_request = false;
    fsm->orderly_shutdown_request = false;
    fsm->power_on_block_mask =
        power_on_precondition_block_mask(preconditions);

    if (preconditions->estop_latched || preconditions->a_class_fault) {
        /* The safety/actuator layers own the controlled stop.  Preserve the
         * logical battery-key latch here so software state cannot claim OFF
         * while the physical MOS6 relay is deliberately held during braking.
         * A later operator low-hold performs the explicit safe power-down.
         */
        fsm->state = REMOTE_POWER_SHUTDOWN_PROTECT;
        fsm->high_voltage_disable_request = false;
        fsm->diagnostic = preconditions->a_class_fault ? DIAG_A_CLASS_FAULT : DIAG_CONTROLLED_STOP_ACTIVE;
        return;
    }

    if (input->power != REMOTE_POS_HIGH && input->power != REMOTE_POS_LOW) {
        fsm->hold_position = input->power;
        fsm->hold_since_ms = input->now_ms;
        fsm->state = fsm->high_voltage_enable_request ? REMOTE_POWER_ON : REMOTE_POWER_OFF;
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
        if (fsm->high_voltage_enable_request) {
            fsm->state = REMOTE_POWER_ON;
            fsm->diagnostic = DIAG_OK;
            return;
        }
        if (fsm->power_on_block_mask == 0U) {
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
        request_safe_power_down(fsm, preconditions);
    }
}

remote_power_state_t remote_power_fsm_get_state(const remote_power_fsm_t *fsm)
{
    return fsm != 0 ? fsm->state : REMOTE_POWER_REJECTED;
}
