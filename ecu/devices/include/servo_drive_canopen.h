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
#define SERVO_DRIVE_CONTROL_FAULT_RESET        (0x0080U)
#define SERVO_DRIVE_CONTROL_TRIGGER_ABSOLUTE_POSITION (0x001FU)
/* Continuous absolute-position following uses the BC/BC2 immediate-update
 * sequence verified with the CAN analyzer: keep bit4 low with 0x002F, then
 * raise bit4 with 0x003F after all axes have received the latest target.
 */
#define SERVO_DRIVE_CONTROL_ABSOLUTE_UPDATE_ARM     (0x002FU)
#define SERVO_DRIVE_CONTROL_ABSOLUTE_UPDATE_TRIGGER (0x003FU)
#define SERVO_DRIVE_CONTROL_QUICK_STOP        (0x0002U)

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

/* Queue one-time velocity-mode preparation.  Call this when a node first enters
 * drive mode or after a drive reset; steady joystick changes should then update
 * only the target-velocity object through
 * servo_drive_canopen_update_target_velocity().
 */
bool servo_drive_canopen_prepare_velocity_mode(canopen_master_service_t *canopen,
                                               const ecu_canopen_node_config_t *node);

/* Queue the BC/BC2 velocity-mode sequence verified during commissioning:
 * NMT operational, mode=velocity, enable operation, then target velocity.
 *
 * Units: target_velocity_units uses the drive manual's 0.1 count/s unit.
 */
bool servo_drive_canopen_run_velocity_mode(canopen_master_service_t *canopen,
                                           const ecu_canopen_node_config_t *node,
                                           int32_t target_velocity_units);

/* Queue only the velocity target object.  This is the responsive path for
 * joystick motion after velocity mode and enable-operation have been prepared.
 */
bool servo_drive_canopen_update_target_velocity(canopen_master_service_t *canopen,
                                                const ecu_canopen_node_config_t *node,
                                                int32_t target_velocity_units);

/* Queue a normal BC/BC2 velocity stop: target velocity zero, then output off. */
bool servo_drive_canopen_stop_velocity_mode(canopen_master_service_t *canopen,
                                            const ecu_canopen_node_config_t *node);

/* Queue one-time position-mode preparation.  Steady steering commands should
 * then update only target position plus the 0x000F -> 0x001F trigger edge.
 */
bool servo_drive_canopen_prepare_position_mode(canopen_master_service_t *canopen,
                                               const ecu_canopen_node_config_t *node,
                                               int32_t profile_velocity_units);

/* Queue the BC/BC2 absolute-position trigger sequence:
 * NMT operational, mode=position, profile speed, target position, enable
 * operation, then the absolute-position trigger control word.
 */
bool servo_drive_canopen_run_absolute_position_mode(canopen_master_service_t *canopen,
                                                    const ecu_canopen_node_config_t *node,
                                                    int32_t profile_velocity_units,
                                                    int32_t target_position_counts);

/* Queue one absolute steering target after position mode is ready.  The BC/BC2
 * drive requires a fresh bit4 edge for every new position command, so this
 * function writes the target-position object, then the enable-operation
 * control word, then the absolute-position trigger control word.
 */
bool servo_drive_canopen_update_absolute_position(canopen_master_service_t *canopen,
                                                  const ecu_canopen_node_config_t *node,
                                                  int32_t target_position_counts);

/* Queue BC/BC2 current mode.  The drive manual names it torque mode, but the
 * command is the configured run-current object in 10 mA units.
 */
bool servo_drive_canopen_run_current_mode(canopen_master_service_t *canopen,
                                          const ecu_canopen_node_config_t *node,
                                          int32_t current_ramp_ma_per_sec,
                                          int16_t current_10ma);

/* Queue current zero and disable the drive output. */
bool servo_drive_canopen_stop_current_mode(canopen_master_service_t *canopen,
                                           const ecu_canopen_node_config_t *node);

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

/* Request object 0x2190 and return the latest completed value when available.
 *
 * The call always refreshes the queued read request. It returns true only when
 * the service snapshot already contains a valid 0x2190 value for the node.
 */
bool servo_drive_canopen_read_input_states(canopen_master_service_t *canopen,
                                           const ecu_canopen_node_config_t *node,
                                           uint16_t *input_states);

#endif /* SERVO_DRIVE_CANOPEN_H */
