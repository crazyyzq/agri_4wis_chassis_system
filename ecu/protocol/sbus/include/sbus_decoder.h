/* SBUS frame decoder. Pure protocol logic; no UART, ISR or board dependency. */
#ifndef SBUS_DECODER_H
#define SBUS_DECODER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define SBUS_FRAME_LENGTH (25U)
#define SBUS_CHANNEL_COUNT (16U)

typedef enum {
    SBUS_DECODE_OK = 0,
    SBUS_DECODE_INVALID_ARGUMENT,
    SBUS_DECODE_INVALID_LENGTH,
    SBUS_DECODE_INVALID_MARKER
} sbus_decode_status_t;

typedef struct {
    uint16_t channels[SBUS_CHANNEL_COUNT];
    bool channel17;
    bool channel18;
    bool frame_lost;
    bool failsafe;
} sbus_decoded_frame_t;

sbus_decode_status_t sbus_decode_frame(const uint8_t *frame,
                                       size_t length,
                                       sbus_decoded_frame_t *out);

#endif /* SBUS_DECODER_H */
