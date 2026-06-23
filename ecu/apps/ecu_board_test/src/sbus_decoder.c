#include <string.h>
#include "sbus_decoder.h"
sbus_status_t sbus_decode(const uint8_t *data, size_t length, sbus_frame_t *frame)
{
    if (data == NULL || frame == NULL || length != 25U) return SBUS_INVALID_LENGTH;
    if (data[0] != 0x0FU || data[24] != 0x00U) return SBUS_INVALID_MARKER;
    memset(frame, 0, sizeof(*frame));
    uint32_t accumulator = 0U;
    uint8_t bits = 0U;
    uint8_t input = 1U;
    for (uint8_t channel = 0U; channel < 16U; ++channel) {
        while (bits < 11U) {
            accumulator |= (uint32_t)data[input++] << bits;
            bits = (uint8_t)(bits + 8U);
        }
        frame->channels[channel] = (uint16_t)(accumulator & 0x7FFU);
        accumulator >>= 11U;
        bits = (uint8_t)(bits - 11U);
    }
    frame->channel17 = (data[23] & 0x01U) != 0U;
    frame->channel18 = (data[23] & 0x02U) != 0U;
    frame->frame_lost = (data[23] & 0x04U) != 0U;
    frame->failsafe = (data[23] & 0x08U) != 0U;
    return SBUS_OK;
}
