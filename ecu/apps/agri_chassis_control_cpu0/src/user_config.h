/* Application configuration consumed by HPM SDK middleware ports.
 *
 * CANopenNode's HPM port includes this file for project-owned sizing. Keep it
 * small and mechanical; device object dictionaries and control semantics belong
 * under ecu/devices and ecu/config.
 */
#ifndef USER_CONFIG_H
#define USER_CONFIG_H

/* CPU0 owns two independent CANopenNode networks:
 * - CAN2: drive and steering BC/BC2 servo drives.
 * - CAN3: lift servo drives and hydraulic-related CANopen devices.
 */
#define MAX_CANOPEN_DEVICE (2U)

#endif /* USER_CONFIG_H */
