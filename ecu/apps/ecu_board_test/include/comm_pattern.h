/* Framing and CRC used by deterministic RS232/RS485 loopback tests. */
#ifndef ECU_COMM_PATTERN_H
#define ECU_COMM_PATTERN_H
#include <stddef.h>
#include <stdint.h>
#define COMM_PAYLOAD_MAX 32U
typedef enum { COMM_OK, COMM_INVALID, COMM_TRUNCATED, COMM_BAD_CRC } comm_status_t;
typedef struct {
    uint8_t channel;
    uint8_t direction;
    uint32_t sequence;
    uint8_t length;
    uint8_t payload[COMM_PAYLOAD_MAX];
} comm_frame_t;
/**
 * @brief Calculate the CRC-16/CCITT-FALSE checksum used by loopback frames.
 *
 * @param data   Byte sequence to checksum; may be null only when length is zero.
 * @param length Number of bytes in data.
 * @return The CRC value, or zero when data is null and length is nonzero.
 *
 * @note The calculation uses initial value 0xFFFF and polynomial 0x1021, with
 *       no input/output reflection and no final XOR.
 */
uint16_t comm_crc16(const uint8_t *data, size_t length);
/**
 * @brief Initialize a deterministic eight-byte serial-loopback frame.
 *
 * @param channel  Logical serial channel identifier stored in the frame.
 * @param sequence Sequence number used as the first payload byte.
 * @param frame    Caller-owned destination; no operation is performed if null.
 *
 * @note The structure is cleared first. Direction remains zero and payload byte
 *       n is the low eight bits of sequence + n.
 */
void comm_pattern_make(uint8_t channel, uint32_t sequence, comm_frame_t *frame);
/**
 * @brief Encode one loopback frame into caller-provided storage.
 *
 * @param frame    Frame to encode; its payload length must not exceed
 *                 COMM_PAYLOAD_MAX.
 * @param output   Caller-owned byte buffer receiving the encoded frame.
 * @param capacity Available bytes in output.
 * @return Exact encoded byte count, or zero for null arguments, an invalid
 *         payload length, or insufficient capacity.
 *
 * @note Multi-byte sequence and CRC fields are encoded least-significant byte
 *       first. No bytes are written when validation fails.
 */
size_t comm_frame_encode(const comm_frame_t *frame, uint8_t *output, size_t capacity);
/**
 * @brief Validate and decode one complete loopback frame.
 *
 * @param data   Encoded input frame.
 * @param length Exact number of encoded bytes; trailing bytes are rejected.
 * @param frame  Caller-owned destination populated only after structural and
 *               CRC validation succeeds.
 * @return COMM_OK on success, COMM_INVALID for null arguments or invalid
 *         markers/length fields, COMM_TRUNCATED for a non-exact byte count, or
 *         COMM_BAD_CRC for a checksum mismatch.
 */
comm_status_t comm_frame_decode(const uint8_t *data, size_t length, comm_frame_t *frame);
#endif
