/* Device-level CiA 402 servo-drive control adapter.
 *
 * This module exposes motion-control intent in names used by the rest of the
 * ECU. It is not a CANopen stack; all CANopen communication is delegated to the
 * HPM SDK CANopenNode service.
 */
#ifndef SERVO_DRIVE_CANOPEN_H
#define SERVO_DRIVE_CANOPEN_H

#include <stdbool.h>
#include <stdint.h>

#include "canopen_master_service.h"
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

#define SERVO_DRIVE_OUTPUT_OUT1_MASK          (1U << 0)
#define SERVO_DRIVE_OUTPUT_OUT2_MASK          (1U << 1)
#define SERVO_DRIVE_OUTPUT_OUT3_MASK          (1U << 2)
#define SERVO_DRIVE_OUTPUT_OUT4_MASK          (1U << 3)
#define SERVO_DRIVE_OUTPUT_OUT5_MASK          (1U << 4)
#define SERVO_DRIVE_OUTPUT_OUT6_MASK          (1U << 5)

#define SERVO_DRIVE_INPUT_IN1_MASK            (1U << 0)
#define SERVO_DRIVE_INPUT_IN2_MASK            (1U << 1)
#define SERVO_DRIVE_INPUT_IN3_MASK            (1U << 2)
#define SERVO_DRIVE_INPUT_IN4_MASK            (1U << 3)
#define SERVO_DRIVE_INPUT_IN5_MASK            (1U << 4)
#define SERVO_DRIVE_INPUT_IN6_MASK            (1U << 5)
#define SERVO_DRIVE_INPUT_IN7_MASK            (1U << 6)
#define SERVO_DRIVE_INPUT_IN8_MASK            (1U << 7)

typedef enum {
    SERVO_DRIVE_MODE_PROFILE_POSITION = 1,
    SERVO_DRIVE_MODE_PROFILE_VELOCITY = 3,
    SERVO_DRIVE_MODE_PROFILE_TORQUE = 4
} servo_drive_mode_t;

/* Queue control-word SDO write through CANopenNode. */
bool servo_drive_canopen_send_control_word(canopen_master_service_t *canopen,
                                           const ecu_canopen_node_config_t *node,
                                           uint16_t control_word);

/* Queue operation-mode and control-word SDO writes through CANopenNode. */
bool servo_drive_canopen_select_mode(canopen_master_service_t *canopen,
                                     const ecu_canopen_node_config_t *node,
                                     uint16_t control_word,
                                     servo_drive_mode_t mode);

/* Queue target-position SDO writes through CANopenNode.
 *
 * Units: target_position_counts is in the drive's configured position units.
 */
bool servo_drive_canopen_set_target_position(canopen_master_service_t *canopen,
                                             const ecu_canopen_node_config_t *node,
                                             uint16_t control_word,
                                             int32_t target_position_counts);

/* Queue target-velocity SDO writes through CANopenNode.
 *
 * Units: target_velocity_counts_per_sec is in the drive's configured velocity
 * units per second.
 */
bool servo_drive_canopen_set_target_velocity(canopen_master_service_t *canopen,
                                             const ecu_canopen_node_config_t *node,
                                             uint16_t control_word,
                                             int32_t target_velocity_counts_per_sec);

/* Queue target-torque SDO writes through CANopenNode. */
bool servo_drive_canopen_set_target_torque(canopen_master_service_t *canopen,
                                           const ecu_canopen_node_config_t *node,
                                           uint16_t control_word,
                                           int16_t target_torque_raw);

/* Queue a program-controlled output-state write to object 0x2194.
 *
 * output_mask selects the OUT bits that are owned by this command.
 * active_mask is the 0x2194 bit value to write for those outputs.
 */
bool servo_drive_canopen_set_output_state(canopen_master_service_t *canopen,
                                          const ecu_canopen_node_config_t *node,
                                          uint16_t output_mask,
                                          uint16_t active_mask);

/* Request object 0x2190 and return the latest completed value when available.
 *
 * The call always refreshes the queued read request. It returns true only when
 * the service snapshot already contains a valid 0x2190 value for the node.
 */
bool servo_drive_canopen_read_input_states(canopen_master_service_t *canopen,
                                           const ecu_canopen_node_config_t *node,
                                           uint16_t *input_states);

#endif /* SERVO_DRIVE_CANOPEN_H */
