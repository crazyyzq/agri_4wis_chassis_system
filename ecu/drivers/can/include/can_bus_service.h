#ifndef CAN_BUS_SERVICE_H
#define CAN_BUS_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#include "canopen_frame.h"
#include "diag_codes.h"

typedef struct {
    uint32_t bitrate;
    bool online;
    uint32_t tx_count;
    canopen_frame_t last_tx;
    diag_code_t diagnostic;
} can_bus_service_t;

/* Initialize one logical CAN bus service with a configured bitrate.
 *
 * Owner: CPU0 task that owns the bus.
 * ISR: not safe.
 */
void can_bus_service_init(can_bus_service_t *service, uint32_t bitrate);
void can_bus_service_set_online(can_bus_service_t *service, bool online);

/* Send a CANopen frame through the current backend.
 *
 * The default backend records the frame. Hardware binding can replace this
 * boundary without changing business logic.
 */
bool can_bus_service_send_canopen(can_bus_service_t *service,
                                  const canopen_frame_t *frame);

#endif /* CAN_BUS_SERVICE_H */
