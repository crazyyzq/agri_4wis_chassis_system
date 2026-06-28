#ifndef CANOPEN_FRAME_H
#define CANOPEN_FRAME_H

#include <stdbool.h>
#include <stdint.h>

#define CANOPEN_FRAME_MAX_DATA_BYTES (8U)

typedef struct {
    uint32_t cob_id;
    uint8_t size;
    uint8_t data[CANOPEN_FRAME_MAX_DATA_BYTES];
} canopen_frame_t;

/* Raw CANopen frame helper used by device adapters that have an explicitly
 * configured PDO backend.
 *
 * This file is not a CANopen protocol stack. Do not add SDO, heartbeat, node
 * guarding, PDO mapping, NMT state handling or EMCY handling here. Those belong
 * to HPM SDK CANopenNode and the project-specific object dictionary.
 */

/* Build a raw PDO CAN frame from a configured COB-ID.
 *
 * Units: COB-ID is the 11-bit CAN identifier stored in ECU hardware config.
 * Owner: device adapters call this from CPU0 task context.
 * ISR: not safe; use only in foreground tasks.
 * Failure behavior: returns false and clears no caller state when arguments are invalid.
 */
bool canopen_frame_build_pdo(uint32_t cob_id,
                             const uint8_t *payload,
                             uint8_t payload_size,
                             canopen_frame_t *out);

/* Build a CANopen NMT command frame using configured node IDs.
 *
 * The command byte is provided by the device layer so this helper stays generic.
 */
bool canopen_frame_build_nmt(uint8_t command,
                             uint8_t node_id,
                             canopen_frame_t *out);

#endif /* CANOPEN_FRAME_H */
