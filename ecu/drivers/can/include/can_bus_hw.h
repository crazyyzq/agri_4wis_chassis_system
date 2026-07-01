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

/* Initialize external CAN1 as the 250 kbit/s supplier power bus.
 *
 * External CAN1 maps to HPM CAN0 on this board. RX frames are recorded into the
 * service and TX uses the service backend registered by this function.
 */
bool can_bus_hw_init_can1_power(can_bus_service_t *service, uint32_t bitrate);

/* Initialize external CAN2 as a 2.0B RX-only diagnostic listener.
 *
 * External CAN2 maps to HPM CAN1 on this board. The ISR records received frames
 * into the provided service. This API is kept for passive analyzer bring-up.
 */
bool can_bus_hw_init_can2_rx_only(can_bus_service_t *service, uint32_t bitrate);

/* Initialize external CAN2 as the BC/BC2 motion bus with TX/RX enabled. */
bool can_bus_hw_init_can2_motion(can_bus_service_t *service, uint32_t bitrate);

/* Initialize external CAN3 as the lift/hydraulic bus with TX/RX enabled. */
bool can_bus_hw_init_can3_lift_hydraulic(can_bus_service_t *service, uint32_t bitrate);

/* Initialize external CAN4 as an auxiliary physical test / future expansion bus. */
bool can_bus_hw_init_can4_auxiliary(can_bus_service_t *service, uint32_t bitrate);

bool can_bus_hw_send_can1_frame(const ecu_can_frame_t *frame);
bool can_bus_hw_send_can2_frame(const ecu_can_frame_t *frame);
bool can_bus_hw_send_can3_frame(const ecu_can_frame_t *frame);
bool can_bus_hw_send_can4_frame(const ecu_can_frame_t *frame);

/* Foreground fallback poll for bring-up.
 *
 * The ISR is still the normal receive path. Polling lets diagnostics distinguish
 * "IRQ not firing" from "CAN controller has no received frames".
 */
void can_bus_hw_poll_can1_rx(can_bus_service_t *service);
void can_bus_hw_poll_can2_rx(can_bus_service_t *service);
void can_bus_hw_poll_can3_rx(can_bus_service_t *service);
void can_bus_hw_poll_can4_rx(can_bus_service_t *service);

void can_bus_hw_can1_isr(void);
void can_bus_hw_can2_isr(void);
void can_bus_hw_can3_isr(void);
void can_bus_hw_can4_isr(void);

#endif /* CAN_BUS_HW_H */
