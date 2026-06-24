/* Decoder for one standard 25-byte, 16-channel SBUS frame. */
#ifndef ECU_SBUS_DECODER_H
#define ECU_SBUS_DECODER_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
typedef enum { SBUS_OK, SBUS_INVALID_LENGTH, SBUS_INVALID_MARKER } sbus_status_t;
typedef struct {
    uint16_t channels[16];
    bool channel17;
    bool channel18;
    bool frame_lost;
    bool failsafe;
} sbus_frame_t;
/* Expand 16 packed 11-bit channels plus digital/lost/failsafe flags. */
sbus_status_t sbus_decode(const uint8_t *data, size_t length, sbus_frame_t *frame);
#endif
