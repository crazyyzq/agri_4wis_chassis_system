#include "board.h"
#include "hpm_canopen_can.h"
#include "user_config.h"

extern void convert_can_frame_to_can_frame(const struct can_frame *frame,
                                           can_transmit_buf_t *tx_buf);
extern int hpm_can_get_state(const struct device *dev,
                             enum can_state *state,
                             struct can_bus_err_cnt *err_cnt);

/* Linker wrap for the SDK symbol hpm_can_send().
 *
 * The upstream HPM CANopen CAN adapter starts a non-blocking CAN transmission
 * and then busy-waits forever for the TX interrupt. If a CANopen bus has no ACK
 * source, that wait can starve lower-priority FreeRTOS tasks. CANopenNode does
 * not require this board adapter to wait for physical TX completion, so this
 * wrapper starts the hardware transmission and returns immediately.
 */
int __wrap_hpm_can_send(const struct device *dev,
                        const struct can_frame *frame)
{
    hpm_can_config_t *cfg = dev->config;
    hpm_can_data_t *data = dev->data;
    CAN_Type *can = cfg->base;
    enum can_state state;
    hpm_stat_t status;

    if ((frame->flags & (CAN_FRAME_FDF | CAN_FRAME_BRS | CAN_FRAME_ESI)) != 0) {
        return -ENOTSUP;
    }

    if (frame->dlc > CAN_MAX_DLC) {
        return -EINVAL;
    }

    if (!data->started) {
        return -ENETDOWN;
    }

    (void)hpm_can_get_state(dev, &state, NULL);
    if (state == CAN_STATE_BUS_OFF) {
        return -ENETUNREACH;
    }

    can_transmit_buf_t tx_buf;
    convert_can_frame_to_can_frame(frame, &tx_buf);

    if (!can_is_primary_transmit_buffer_full(can)) {
        status = can_send_high_priority_message_nonblocking(can, &tx_buf);
    } else if (!can_is_secondary_transmit_buffer_full(can)) {
        status = can_send_message_nonblocking(can, &tx_buf);
    } else {
        return -EBUSY;
    }

    return status == status_success ? 0 : -EIO;
}
