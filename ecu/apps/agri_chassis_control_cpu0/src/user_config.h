/* Application configuration consumed by HPM SDK middleware ports.
 *
 * CANopenNode's HPM port includes this file for project-owned sizing. Keep it
 * small and mechanical; device object dictionaries and control semantics belong
 * under ecu/devices and ecu/config.
 */
#ifndef USER_CONFIG_H
#define USER_CONFIG_H

/* The current CANopenNode diagnostic build owns only CAN2. Additional CANopen
 * networks must be added deliberately, with separate runtime state and IRQ
 * ownership.
 */
#define MAX_CANOPEN_DEVICE (1U)

#endif /* USER_CONFIG_H */
