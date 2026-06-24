/* Deterministic binary loopback frame format with CRC-16/CCITT-FALSE. */
#include <string.h>
#include "comm_pattern.h"
#define COMM_MAGIC0 0xEC
#define COMM_MAGIC1 0x55
uint16_t comm_crc16(const uint8_t *data, size_t length)
{
    if (data == NULL && length != 0U) return 0U;
    uint16_t crc = 0xFFFFU;
    for (size_t i = 0; i < length; ++i) {
        crc ^= (uint16_t)data[i] << 8U;
        for (uint8_t bit = 0; bit < 8U; ++bit) crc = (crc & 0x8000U) ? (uint16_t)((crc << 1U) ^ 0x1021U) : (uint16_t)(crc << 1U);
    }
    return crc;
}
void comm_pattern_make(uint8_t channel, uint32_t sequence, comm_frame_t *frame)
{
    if (frame == NULL) return;
    memset(frame, 0, sizeof(*frame));
    frame->channel = channel;
    frame->sequence = sequence;
    frame->length = 8U;
    for (uint8_t i = 0; i < frame->length; ++i) frame->payload[i] = (uint8_t)(sequence + i);
}
size_t comm_frame_encode(const comm_frame_t *frame, uint8_t *output, size_t capacity)
{
    if (frame == NULL || output == NULL || frame->length > COMM_PAYLOAD_MAX) return 0U;
    size_t needed = 11U + frame->length;
    if (capacity < needed) return 0U;
    output[0] = COMM_MAGIC0; output[1] = COMM_MAGIC1;
    output[2] = frame->channel; output[3] = frame->direction;
    output[4] = (uint8_t)frame->sequence; output[5] = (uint8_t)(frame->sequence >> 8U);
    output[6] = (uint8_t)(frame->sequence >> 16U); output[7] = (uint8_t)(frame->sequence >> 24U);
    output[8] = frame->length;
    memcpy(&output[9], frame->payload, frame->length);
    uint16_t crc = comm_crc16(output, 9U + frame->length);
    output[9U + frame->length] = (uint8_t)crc;
    output[10U + frame->length] = (uint8_t)(crc >> 8U);
    return needed;
}
comm_status_t comm_frame_decode(const uint8_t *data, size_t length, comm_frame_t *frame)
{
    if (data == NULL || frame == NULL) return COMM_INVALID;
    if (length < 11U) return COMM_TRUNCATED;
    if (data[0] != COMM_MAGIC0 || data[1] != COMM_MAGIC1 || data[8] > COMM_PAYLOAD_MAX) return COMM_INVALID;
    size_t expected_length = 11U + data[8];
    if (length != expected_length) return COMM_TRUNCATED;
    uint16_t expected_crc = (uint16_t)data[length - 2U] | ((uint16_t)data[length - 1U] << 8U);
    if (comm_crc16(data, length - 2U) != expected_crc) return COMM_BAD_CRC;
    frame->channel = data[2]; frame->direction = data[3];
    frame->sequence = (uint32_t)data[4] | ((uint32_t)data[5] << 8U) |
                      ((uint32_t)data[6] << 16U) | ((uint32_t)data[7] << 24U);
    frame->length = data[8];
    memcpy(frame->payload, &data[9], frame->length);
    return COMM_OK;
}
