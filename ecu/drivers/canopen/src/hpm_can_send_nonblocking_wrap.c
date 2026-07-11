#include <string.h>

#include "board.h"
#include "CO_driver.h"
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
 *
 * The realtime PDO scheduler depends on a strict one-frame FIFO.  Therefore
 * this wrapper only uses the primary TX buffer; a busy primary buffer is
 * reported as -EBUSY instead of spilling a later frame into another hardware
 * queue where it could overtake an older group member.
 */
int __wrap_hpm_can_send(const struct device *dev,
                        const struct can_frame *frame)
{
    hpm_can_config_t *cfg = dev->config;
    hpm_can_data_t *data = dev->data;
    CAN_Type *can = cfg->base;
    enum can_state state = CAN_STATE_ERROR_ACTIVE;
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

    if (hpm_can_get_state(dev, &state, NULL) != 0) {
        return -EIO;
    }
    if (state == CAN_STATE_BUS_OFF) {
        return -ENETUNREACH;
    }

    can_transmit_buf_t tx_buf;
    convert_can_frame_to_can_frame(frame, &tx_buf);

    if (can_is_primary_transmit_buffer_full(can)) {
        return -EBUSY;
    }

    status = can_send_high_priority_message_nonblocking(can, &tx_buf);
    return status == status_success ? 0 : -EIO;
}

/* CANopenNode's HPM adapter discards the return value from hpm_can_send().
 * That makes NMT/SDO report local success even when the primary TX buffer is
 * busy or the controller is bus-off.  Keep the vendor SDK untouched and wrap
 * CO_CANsend() in project code so the CANopenNode state machines receive the
 * real nonblocking submission result.
 */
CO_ReturnError_t __wrap_CO_CANsend(CO_CANmodule_t *CANmodule,
                                   CO_CANtx_t *buffer)
{
    struct can_frame frame;

    if (CANmodule == NULL || CANmodule->CANptr == NULL || buffer == NULL) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }
    if (buffer->bufferFull) {
        buffer->bufferFull = false;
        return CO_ERROR_TX_OVERFLOW;
    }

    memset(&frame, 0, sizeof(frame));
    frame.id = buffer->ident;
    frame.dlc = buffer->DLC;
    frame.flags = buffer->rtr ? CAN_FRAME_RTR : 0U;
    memcpy(frame.data, buffer->data, buffer->DLC);

    int result = __wrap_hpm_can_send(
        (const struct device *)CANmodule->CANptr,
        &frame);
    if (result == 0) {
        return CO_ERROR_NO;
    }
    if (result == -EBUSY) {
        return CO_ERROR_TX_BUSY;
    }
    return CO_ERROR_TX_UNCONFIGURED;
}
