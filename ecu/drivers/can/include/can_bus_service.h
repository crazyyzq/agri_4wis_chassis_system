#ifndef CAN_BUS_SERVICE_H
#define CAN_BUS_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#include "canopen_frame.h"
#include "diag_codes.h"

#define CAN_BUS_FRAME_MAX_DATA_BYTES (8U)
#define CAN_BUS_RX_QUEUE_CAPACITY    (16U)

typedef struct {
    uint32_t id;
    uint8_t size;
    bool extended;
    bool remote;
    uint8_t data[CAN_BUS_FRAME_MAX_DATA_BYTES];
} ecu_can_frame_t;

typedef bool (*can_bus_service_tx_backend_t)(const ecu_can_frame_t *frame);

typedef struct {
    uint32_t bitrate;
    bool online;
    uint32_t tx_count;
    uint32_t rx_count;
    uint32_t error_count;
    uint8_t rx_buffer_status;
    uint8_t tx_rx_flags;
    uint8_t error_flags;
    uint8_t receive_error_count;
    uint8_t transmit_error_count;
    uint8_t last_error_kind;
    canopen_frame_t last_tx;
    ecu_can_frame_t last_tx_frame;
    uint32_t last_rx_id;
    uint8_t last_rx_size;
    bool last_rx_extended;
    bool last_rx_remote;
    uint8_t last_rx_data[CAN_BUS_FRAME_MAX_DATA_BYTES];
    ecu_can_frame_t last_rx_frame;
    ecu_can_frame_t rx_queue[CAN_BUS_RX_QUEUE_CAPACITY];
    uint8_t rx_queue_head;
    uint8_t rx_queue_tail;
    uint8_t rx_queue_count;
    uint32_t rx_queue_dropped_count;
    can_bus_service_tx_backend_t tx_backend;
    diag_code_t diagnostic;
} can_bus_service_t;

/* Initialize one logical CAN bus service with a configured bitrate.
 *
 * Owner: CPU0 task that owns the bus.
 * ISR: not safe.
 */
void can_bus_service_init(can_bus_service_t *service, uint32_t bitrate);
void can_bus_service_set_online(can_bus_service_t *service, bool online);
void can_bus_service_set_tx_backend(can_bus_service_t *service,
                                    can_bus_service_tx_backend_t backend);

/* Send one classic CAN 2.0 frame through the service backend.
 *
 * The service owns validation and accounting. Hardware-specific transmission
 * must be supplied by can_bus_hw before the service can accept TX requests.
 */
bool can_bus_service_send_frame(can_bus_service_t *service,
                                const ecu_can_frame_t *frame);

/* Send a CANopen frame through the current backend.
 *
 * CANopen uses standard 11-bit identifiers, so this helper adapts the legacy
 * CANopen frame type to the generic CAN service frame.
 */
bool can_bus_service_send_canopen(can_bus_service_t *service,
                                  const canopen_frame_t *frame);

/* Record one received CAN 2.0 frame from a hardware ISR.
 *
 * This is intentionally frame-level, not CANopen-level. Protocol decoding
 * belongs above the driver boundary.
 */
void can_bus_service_note_rx_from_isr(can_bus_service_t *service,
                                      uint32_t id,
                                      bool extended,
                                      bool remote,
                                      const uint8_t *data,
                                      uint8_t size);

void can_bus_service_note_error_from_isr(can_bus_service_t *service);

/* Pop one received CAN frame from the foreground task.
 *
 * RX producers are the controller ISR and foreground fallback poll. The pop
 * path is atomic so protocol tasks can drain the queue without racing updates.
 */
bool can_bus_service_pop_rx_frame(can_bus_service_t *service,
                                  ecu_can_frame_t *out);

void can_bus_service_note_controller_status(can_bus_service_t *service,
                                            uint8_t rx_buffer_status,
                                            uint8_t tx_rx_flags,
                                            uint8_t error_flags,
                                            uint8_t receive_error_count,
                                            uint8_t transmit_error_count,
                                            uint8_t last_error_kind);

/* Copy the bus state for foreground diagnostics.
 *
 * RX/error fields can be updated by a CAN ISR, so callers should use this
 * accessor instead of reading the structure directly from another task.
 */
void can_bus_service_get_snapshot(const can_bus_service_t *service,
                                  can_bus_service_t *out);

#endif /* CAN_BUS_SERVICE_H */
