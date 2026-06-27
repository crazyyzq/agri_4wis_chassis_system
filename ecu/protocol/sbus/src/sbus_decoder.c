#include "sbus_decoder.h"

sbus_decode_status_t sbus_decode_frame(const uint8_t *frame,
                                       size_t length,
                                       sbus_decoded_frame_t *out)
{
    if (frame == 0 || out == 0) {
        return SBUS_DECODE_INVALID_ARGUMENT;
    }
    if (length != SBUS_FRAME_LENGTH) {
        return SBUS_DECODE_INVALID_LENGTH;
    }
    if (frame[0] != 0x0FU || frame[24] != 0x00U) {
        return SBUS_DECODE_INVALID_MARKER;
    }

    out->channels[0]  = (uint16_t)((frame[1]       | frame[2]  << 8) & 0x07FFU);
    out->channels[1]  = (uint16_t)((frame[2]  >> 3 | frame[3]  << 5) & 0x07FFU);
    out->channels[2]  = (uint16_t)((frame[3]  >> 6 | frame[4]  << 2 | frame[5] << 10) & 0x07FFU);
    out->channels[3]  = (uint16_t)((frame[5]  >> 1 | frame[6]  << 7) & 0x07FFU);
    out->channels[4]  = (uint16_t)((frame[6]  >> 4 | frame[7]  << 4) & 0x07FFU);
    out->channels[5]  = (uint16_t)((frame[7]  >> 7 | frame[8]  << 1 | frame[9] << 9) & 0x07FFU);
    out->channels[6]  = (uint16_t)((frame[9]  >> 2 | frame[10] << 6) & 0x07FFU);
    out->channels[7]  = (uint16_t)((frame[10] >> 5 | frame[11] << 3) & 0x07FFU);
    out->channels[8]  = (uint16_t)((frame[12]      | frame[13] << 8) & 0x07FFU);
    out->channels[9]  = (uint16_t)((frame[13] >> 3 | frame[14] << 5) & 0x07FFU);
    out->channels[10] = (uint16_t)((frame[14] >> 6 | frame[15] << 2 | frame[16] << 10) & 0x07FFU);
    out->channels[11] = (uint16_t)((frame[16] >> 1 | frame[17] << 7) & 0x07FFU);
    out->channels[12] = (uint16_t)((frame[17] >> 4 | frame[18] << 4) & 0x07FFU);
    out->channels[13] = (uint16_t)((frame[18] >> 7 | frame[19] << 1 | frame[20] << 9) & 0x07FFU);
    out->channels[14] = (uint16_t)((frame[20] >> 2 | frame[21] << 6) & 0x07FFU);
    out->channels[15] = (uint16_t)((frame[21] >> 5 | frame[22] << 3) & 0x07FFU);

    out->channel17 = (frame[23] & 0x01U) != 0U;
    out->channel18 = (frame[23] & 0x02U) != 0U;
    out->frame_lost = (frame[23] & 0x04U) != 0U;
    out->failsafe = (frame[23] & 0x08U) != 0U;
    return SBUS_DECODE_OK;
}
