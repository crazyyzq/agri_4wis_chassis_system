/* Device-level CiA 402 servo-drive control adapter.
 *
 * This module exposes motion-control intent in names used by the rest of the
 * ECU. It is not a CANopen stack. The first backend sends EDS-defined PDO
 * payloads through the raw-PDO frame helper; the API is intentionally kept
 * stable so the backend can move to HPM SDK CANopenNode without changing the
 * vehicle or control layers.
 */
#ifndef SERVO_DRIVE_CANOPEN_H
#define SERVO_DRIVE_CANOPEN_H

#include <stdbool.h>
#include <stdint.h>

#include "can_bus_service.h"
#include "ecu_config.h"

/* CiA 402 control words used by high-level device adapters.
 *
 * Keep these named at the device layer. Callers should request behavior such as
 * "enable operation" or "quick stop" instead of passing unexplained literals.
 */
#define SERVO_DRIVE_CONTROL_DISABLE_VOLTAGE   (0x0000U)
#define SERVO_DRIVE_CONTROL_SHUTDOWN          (0x0006U)
#define SERVO_DRIVE_CONTROL_SWITCH_ON         (0x0007U)
#define SERVO_DRIVE_CONTROL_ENABLE_OPERATION  (0x000FU)
#define SERVO_DRIVE_CONTROL_QUICK_STOP        (0x0002U)

typedef enum {
    SERVO_DRIVE_MODE_PROFILE_POSITION = 1,
    SERVO_DRIVE_MODE_PROFILE_VELOCITY = 3,
    SERVO_DRIVE_MODE_PROFILE_TORQUE = 4
} servo_drive_mode_t;

/* Send RPDO1: control word only. */
bool servo_drive_canopen_send_control_word(can_bus_service_t *bus,
                                           const ecu_canopen_node_config_t *node,
                                           uint16_t control_word);

/* Send RPDO2: control word + operation mode. */
bool servo_drive_canopen_select_mode(can_bus_service_t *bus,
                                     const ecu_canopen_node_config_t *node,
                                     uint16_t control_word,
                                     servo_drive_mode_t mode);

/* Send RPDO3: control word + target position.
 *
 * Units: target_position_counts is in the drive's configured position units.
 */
bool servo_drive_canopen_set_target_position(can_bus_service_t *bus,
                                             const ecu_canopen_node_config_t *node,
                                             uint16_t control_word,
                                             int32_t target_position_counts);

/* Send RPDO4: control word + target velocity.
 *
 * Units: target_velocity_counts_per_sec is in the drive's configured velocity
 * units per second.
 */
bool servo_drive_canopen_set_target_velocity(can_bus_service_t *bus,
                                             const ecu_canopen_node_config_t *node,
                                             uint16_t control_word,
                                             int32_t target_velocity_counts_per_sec);

/* Send RPDO5: control word + target torque.
 *
 * The provided EDS marks RPDO5 disabled by default. This function returns false
 * until configuration provides an enabled RPDO5 COB-ID.
 */
bool servo_drive_canopen_set_target_torque(can_bus_service_t *bus,
                                           const ecu_canopen_node_config_t *node,
                                           uint16_t control_word,
                                           int16_t target_torque_raw);

#endif /* SERVO_DRIVE_CANOPEN_H */
