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
/**
 * @brief Decode one standard SBUS frame into channel values and status flags.
 *
 * @param data   Encoded SBUS frame containing exactly 25 bytes.
 * @param length Input length, which must equal 25.
 * @param frame  Caller-owned destination cleared and populated on success.
 * @return SBUS_OK on success, SBUS_INVALID_LENGTH for null arguments or any
 *         non-25-byte input, or SBUS_INVALID_MARKER for invalid start/end bytes.
 *
 * @note Expands sixteen little-endian packed 11-bit analog channels and the
 *       channel-17, channel-18, frame-lost, and failsafe flags.
 */
sbus_status_t sbus_decode(const uint8_t *data, size_t length, sbus_frame_t *frame);
#endif
