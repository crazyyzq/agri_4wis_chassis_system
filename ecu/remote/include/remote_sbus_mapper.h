/* SBUS-to-remote input mapper.
 *
 * This module owns receiver calibration, PPM-equivalent conversion, discrete
 * switch debounce and analog per-mille conversion.  CPU0 tasks should only pass
 * a fresh SBUS sample in and consume the resulting remote_input_snapshot_t.
 */
#ifndef REMOTE_SBUS_MAPPER_H
#define REMOTE_SBUS_MAPPER_H

#include <stdbool.h>
#include <stdint.h>

#include "ecu_config.h"
#include "remote_types.h"

typedef struct {
    uint16_t channels[ECU_SBUS_CHANNEL_COUNT];
    bool valid;
    bool connected;
    bool failsafe;
    bool frame_lost;
    bool channel17;
    bool channel18;
    uint32_t frame_count;
    uint32_t decode_error_count;
    uint32_t uart_error_count;
    uint32_t last_frame_ms;
} remote_sbus_sample_t;

typedef struct {
    remote_discrete_channel_t discrete_channels[ECU_SBUS_CHANNEL_COUNT];
} remote_sbus_mapper_t;

void remote_sbus_mapper_init(remote_sbus_mapper_t *mapper, uint32_t now_ms);

void remote_sbus_mapper_build_ppm_sample(const remote_sbus_sample_t *raw,
                                         remote_sbus_sample_t *ppm);

void remote_sbus_mapper_build_input(remote_sbus_mapper_t *mapper,
                                    const remote_sbus_sample_t *ppm_sbus,
                                    const ecu_config_t *config,
                                    uint32_t now_ms,
                                    remote_input_snapshot_t *out);

#endif /* REMOTE_SBUS_MAPPER_H */
