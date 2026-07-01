#include "remote_sbus_mapper.h"

#include <string.h>

#include "remote_discrete.h"

static int16_t clamp_per_mille_i16(int32_t value)
{
    if (value > 1000) {
        return 1000;
    }
    if (value < -1000) {
        return -1000;
    }
    return (int16_t)value;
}

static uint16_t interpolate_u16(uint16_t input,
                                uint16_t input_low,
                                uint16_t input_high,
                                uint16_t output_low,
                                uint16_t output_high)
{
    if (input_high <= input_low) {
        return output_low;
    }

    uint32_t input_span = (uint32_t)input_high - (uint32_t)input_low;
    uint32_t output_span = (uint32_t)output_high - (uint32_t)output_low;
    uint32_t offset = (uint32_t)input - (uint32_t)input_low;
    return (uint16_t)((uint32_t)output_low +
                      ((offset * output_span + (input_span / 2U)) / input_span));
}

static uint16_t sbus_protocol_raw_to_ppm_equivalent(uint16_t raw)
{
    if (raw <= ECU_SBUS_PROTOCOL_RAW_LOW) {
        return ECU_SBUS_PPM_LOW;
    }
    if (raw < ECU_SBUS_PROTOCOL_RAW_CENTER) {
        return interpolate_u16(raw,
                               ECU_SBUS_PROTOCOL_RAW_LOW,
                               ECU_SBUS_PROTOCOL_RAW_CENTER,
                               ECU_SBUS_PPM_LOW,
                               ECU_SBUS_PPM_CENTER);
    }
    if (raw == ECU_SBUS_PROTOCOL_RAW_CENTER) {
        return ECU_SBUS_PPM_CENTER;
    }
    if (raw < ECU_SBUS_PROTOCOL_RAW_HIGH) {
        return interpolate_u16(raw,
                               ECU_SBUS_PROTOCOL_RAW_CENTER,
                               ECU_SBUS_PROTOCOL_RAW_HIGH,
                               ECU_SBUS_PPM_CENTER,
                               ECU_SBUS_PPM_HIGH);
    }
    return ECU_SBUS_PPM_HIGH;
}

static remote_position_t stable_position_from_channel(remote_sbus_mapper_t *mapper,
                                                      const remote_sbus_sample_t *sbus,
                                                      ecu_sbus_channel_role_t channel,
                                                      const ecu_config_t *config,
                                                      uint32_t now_ms)
{
    remote_position_t candidate = REMOTE_POS_INVALID;
    if (sbus->valid && channel < ECU_SBUS_CHANNEL_COUNT) {
        candidate = remote_discrete_position_from_raw(sbus->channels[channel],
                                                      &config->sbus_thresholds);
    }
    remote_discrete_update(&mapper->discrete_channels[channel],
                           candidate,
                           now_ms,
                           config->discrete_debounce_ms);
    return mapper->discrete_channels[channel].stable_position;
}

static int16_t sbus_per_mille_from_ppm(uint16_t ppm,
                                       const ecu_sbus_thresholds_t *thresholds)
{
    if (thresholds == 0) {
        return 0;
    }

    int32_t neutral = (int32_t)thresholds->stick_neutral;
    int32_t ppm_value = (int32_t)ppm;
    if (ppm_value >= neutral) {
        int32_t span = (int32_t)thresholds->stick_max - neutral;
        if (span <= 0) {
            return 0;
        }
        return clamp_per_mille_i16(((ppm_value - neutral) * 1000) / span);
    }

    int32_t span = neutral - (int32_t)thresholds->stick_min;
    if (span <= 0) {
        return 0;
    }
    return clamp_per_mille_i16(-(((neutral - ppm_value) * 1000) / span));
}

static int16_t sbus_throttle_per_mille_from_ppm(uint16_t ppm,
                                                const ecu_sbus_thresholds_t *thresholds)
{
    if (thresholds == 0 || ppm <= thresholds->throttle_min) {
        return 0;
    }

    uint32_t span = (uint32_t)thresholds->throttle_max -
                    (uint32_t)thresholds->throttle_min;
    if (span == 0U) {
        return 0;
    }

    if (ppm >= thresholds->throttle_max) {
        return 1000;
    }

    return clamp_per_mille_i16((int32_t)(((uint32_t)(ppm - thresholds->throttle_min) *
                                          1000U) / span));
}

static bool sbus_ppm_channels_are_credible(const remote_sbus_sample_t *sbus)
{
    if (sbus == 0 || !sbus->valid || !sbus->connected) {
        return true;
    }

    for (uint32_t channel = 0U; channel < ECU_SBUS_CHANNEL_COUNT; ++channel) {
        if (sbus->channels[channel] < ECU_SBUS_PPM_CREDIBLE_MIN ||
            sbus->channels[channel] > ECU_SBUS_PPM_CREDIBLE_MAX) {
            return false;
        }
    }
    return true;
}

void remote_sbus_mapper_init(remote_sbus_mapper_t *mapper, uint32_t now_ms)
{
    if (mapper == 0) {
        return;
    }

    for (uint32_t channel = 0U; channel < ECU_SBUS_CHANNEL_COUNT; ++channel) {
        remote_discrete_init(&mapper->discrete_channels[channel],
                             REMOTE_POS_CENTER,
                             now_ms);
    }
}

void remote_sbus_mapper_build_ppm_sample(const remote_sbus_sample_t *raw,
                                         remote_sbus_sample_t *ppm)
{
    if (raw == 0 || ppm == 0) {
        return;
    }

    *ppm = *raw;
    if (!ppm->valid) {
        return;
    }

    for (uint32_t channel = 0U; channel < ECU_SBUS_CHANNEL_COUNT; ++channel) {
        ppm->channels[channel] =
            sbus_protocol_raw_to_ppm_equivalent(raw->channels[channel]);
    }
}

void remote_sbus_mapper_build_input(remote_sbus_mapper_t *mapper,
                                    const remote_sbus_sample_t *ppm_sbus,
                                    const ecu_config_t *config,
                                    uint32_t now_ms,
                                    remote_input_snapshot_t *out)
{
    if (out == 0) {
        return;
    }
    memset(out, 0, sizeof(*out));
    out->now_ms = now_ms;

    if (mapper == 0 || ppm_sbus == 0 || config == 0) {
        out->sbus_timeout = true;
        return;
    }

    out->sbus_valid = ppm_sbus->valid && ppm_sbus->connected;
    out->sbus_failsafe = ppm_sbus->failsafe;
    out->sbus_timeout = !ppm_sbus->connected;
    out->decode_error_limit =
        ppm_sbus->decode_error_count >= ECU_SBUS_DECODE_ERROR_LIMIT;
    out->credibility_error = !sbus_ppm_channels_are_credible(ppm_sbus);

    out->steering = stable_position_from_channel(mapper, ppm_sbus, ECU_SBUS_CH_STEER, config, now_ms);
    out->clearance = stable_position_from_channel(mapper, ppm_sbus, ECU_SBUS_CH_CLEARANCE, config, now_ms);
    out->throttle = stable_position_from_channel(mapper, ppm_sbus, ECU_SBUS_CH_THROTTLE, config, now_ms);
    out->power = stable_position_from_channel(mapper, ppm_sbus, ECU_SBUS_CH_POWER, config, now_ms);
    out->gear = stable_position_from_channel(mapper, ppm_sbus, ECU_SBUS_CH_GEAR, config, now_ms);
    out->home = stable_position_from_channel(mapper, ppm_sbus, ECU_SBUS_CH_HOME, config, now_ms);
    out->authority = stable_position_from_channel(mapper, ppm_sbus, ECU_SBUS_CH_AUTHORITY, config, now_ms);
    out->left_indicator = stable_position_from_channel(mapper, ppm_sbus, ECU_SBUS_CH_LEFT_INDICATOR, config, now_ms);
    out->hazard = stable_position_from_channel(mapper, ppm_sbus, ECU_SBUS_CH_HAZARD, config, now_ms);
    out->horn = stable_position_from_channel(mapper, ppm_sbus, ECU_SBUS_CH_HORN, config, now_ms);
    out->headlight = stable_position_from_channel(mapper, ppm_sbus, ECU_SBUS_CH_HEADLIGHT, config, now_ms);
    out->right_indicator = stable_position_from_channel(mapper, ppm_sbus, ECU_SBUS_CH_RIGHT_INDICATOR, config, now_ms);
    out->track = stable_position_from_channel(mapper, ppm_sbus, ECU_SBUS_CH_TRACK, config, now_ms);
    out->r1 = stable_position_from_channel(mapper, ppm_sbus, ECU_SBUS_CH_R1, config, now_ms);
    out->r2 = stable_position_from_channel(mapper, ppm_sbus, ECU_SBUS_CH_R2, config, now_ms);
    out->r1_changed = mapper->discrete_channels[ECU_SBUS_CH_R1].changed;
    out->r2_changed = mapper->discrete_channels[ECU_SBUS_CH_R2].changed;

    if (out->sbus_valid) {
        out->steer_per_mille =
            sbus_per_mille_from_ppm(ppm_sbus->channels[ECU_SBUS_CH_STEER],
                                    &config->sbus_thresholds);
        out->throttle_per_mille =
            sbus_throttle_per_mille_from_ppm(ppm_sbus->channels[ECU_SBUS_CH_THROTTLE],
                                             &config->sbus_thresholds);
        out->clearance_per_mille =
            sbus_per_mille_from_ppm(ppm_sbus->channels[ECU_SBUS_CH_CLEARANCE],
                                    &config->sbus_thresholds);
        out->track_per_mille =
            sbus_per_mille_from_ppm(ppm_sbus->channels[ECU_SBUS_CH_TRACK],
                                    &config->sbus_thresholds);
    }

    out->ch13_estop = stable_position_from_channel(mapper, ppm_sbus, ECU_SBUS_CH_ESTOP, config, now_ms);
    if (ppm_sbus->valid &&
        remote_discrete_position_from_raw(ppm_sbus->channels[ECU_SBUS_CH_ESTOP],
                                          &config->sbus_thresholds) == REMOTE_POS_HIGH) {
        out->ch13_estop = REMOTE_POS_HIGH;
    }
}
