/* HPM6750 CAN hardware bindings used by ECU diagnostic bring-up.
 *
 * The first binding is deliberately RX-only for CAN2 BC/BC2 drive debugging. It lets us
 * verify bitrate, pinmux, transceiver direction and incoming CANopen heartbeat
 * or EMCY traffic without sending control words to a connected drive.
 */
#ifndef CAN_BUS_HW_H
#define CAN_BUS_HW_H

#include <stdbool.h>
#include <stdint.h>

#include "can_bus_service.h"

/* Initialize external CAN2 as a 2.0B RX-only diagnostic listener.
 *
 * External CAN2 maps to HPM CAN1 on this board. The ISR records received frames
 * into the provided service. No transmit API is exposed from this module.
 */
bool can_bus_hw_init_can2_rx_only(can_bus_service_t *service, uint32_t bitrate);

/* Foreground fallback poll for bring-up.
 *
 * The ISR is still the normal receive path. Polling lets diagnostics distinguish
 * "IRQ not firing" from "CAN controller has no received frames".
 */
void can_bus_hw_poll_can2_rx(can_bus_service_t *service);

void can_bus_hw_can2_isr(void);

#endif /* CAN_BUS_HW_H */
