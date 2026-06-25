/* Hardware-independent safety ownership for outputs, termination and RS485 DE. */
#include <stddef.h>
#include "safety_manager.h"

static const safety_hw_ops_t *s_ops;
static safety_snapshot_t s_state;

void safety_all_off(void)
{
    if (s_ops == NULL || s_ops->output_write == NULL ||
        s_ops->can_term_write == NULL || s_ops->rs485_direction_write == NULL) return;
    /* External hardware channels are intentionally one-based throughout. */
    for (uint8_t i = 1U; i <= SAFETY_OUTPUT_COUNT; ++i) s_ops->output_write(i, false);
    for (uint8_t i = 1U; i <= SAFETY_CAN_COUNT; ++i) s_ops->can_term_write(i, false);
    for (uint8_t i = 1U; i <= SAFETY_RS485_COUNT; ++i) s_ops->rs485_direction_write(i, false);
    s_state.active_output = 0U;
    s_state.can_term_mask = 0U;
    s_state.rs485_tx_mask = 0U;
}

void safety_init(const safety_hw_ops_t *ops)
{
    s_ops = ops != NULL && ops->output_write != NULL &&
            ops->can_term_write != NULL && ops->rs485_direction_write != NULL
        ? ops
        : NULL;
    s_state = (safety_snapshot_t){ 0 };
    safety_all_off();
}

const safety_hw_ops_t *safety_backend(void)
{
    return s_ops;
}

safety_status_t safety_output_on(uint8_t index)
{
    if (s_ops == NULL || index == 0U || index > SAFETY_OUTPUT_COUNT) return SAFETY_INVALID;
    if (s_state.active_output != 0U && s_state.active_output != index) return SAFETY_BUSY;
    s_ops->output_write(index, true);
    s_state.active_output = index;
    return SAFETY_OK;
}

void safety_output_off(uint8_t index)
{
    if (s_ops == NULL || index == 0U || index > SAFETY_OUTPUT_COUNT) return;
    s_ops->output_write(index, false);
    if (s_state.active_output == index) s_state.active_output = 0U;
}

safety_status_t safety_can_term_set(uint8_t index, bool enable)
{
    if (s_ops == NULL || index == 0U || index > SAFETY_CAN_COUNT) return SAFETY_INVALID;
    s_ops->can_term_write(index, enable);
    if (enable) s_state.can_term_mask |= (uint8_t)(1U << (index - 1U));
    else s_state.can_term_mask &= (uint8_t)~(1U << (index - 1U));
    return SAFETY_OK;
}

safety_status_t safety_rs485_transmit(uint8_t index)
{
    if (s_ops == NULL || index == 0U || index > SAFETY_RS485_COUNT) return SAFETY_INVALID;
    s_ops->rs485_direction_write(index, true);
    s_state.rs485_tx_mask |= (uint8_t)(1U << (index - 1U));
    return SAFETY_OK;
}

void safety_rs485_receive(uint8_t index)
{
    if (s_ops == NULL || index == 0U || index > SAFETY_RS485_COUNT) return;
    s_ops->rs485_direction_write(index, false);
    s_state.rs485_tx_mask &= (uint8_t)~(1U << (index - 1U));
}

safety_snapshot_t safety_snapshot(void) { return s_state; }
