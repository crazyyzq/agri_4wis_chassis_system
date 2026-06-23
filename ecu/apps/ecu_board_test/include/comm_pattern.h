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
uint16_t comm_crc16(const uint8_t *data, size_t length);
void comm_pattern_make(uint8_t channel, uint32_t sequence, comm_frame_t *frame);
size_t comm_frame_encode(const comm_frame_t *frame, uint8_t *output, size_t capacity);
comm_status_t comm_frame_decode(const uint8_t *data, size_t length, comm_frame_t *frame);
#endif
