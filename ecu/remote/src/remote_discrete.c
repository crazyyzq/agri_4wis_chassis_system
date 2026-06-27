#include "ecu_time.h"
#include "remote_discrete.h"

void remote_discrete_init(remote_discrete_channel_t *channel, remote_position_t baseline, uint32_t now_ms)
{
    if (channel == 0) {
        return;
    }
    channel->candidate_position = baseline;
    channel->stable_position = baseline;
    channel->previous_stable_position = baseline;
    channel->candidate_since_ms = now_ms;
    channel->stable_since_ms = now_ms;
    channel->changed = false;
}

void remote_discrete_sync_baseline(remote_discrete_channel_t *channel, remote_position_t baseline, uint32_t now_ms)
{
    remote_discrete_init(channel, baseline, now_ms);
}

void remote_discrete_update(remote_discrete_channel_t *channel,
                            remote_position_t candidate,
                            uint32_t now_ms,
                            uint32_t debounce_ms)
{
    if (channel == 0) {
        return;
    }
    channel->changed = false;
    if (candidate != channel->candidate_position) {
        channel->candidate_position = candidate;
        channel->candidate_since_ms = now_ms;
        return;
    }
    if (candidate != channel->stable_position &&
        ecu_time_elapsed(now_ms, channel->candidate_since_ms, debounce_ms)) {
        channel->previous_stable_position = channel->stable_position;
        channel->stable_position = candidate;
        channel->stable_since_ms = now_ms;
        channel->changed = true;
    }
}

remote_position_t remote_discrete_position_from_raw(uint16_t raw, const ecu_sbus_thresholds_t *thresholds)
{
    if (thresholds == 0) {
        return REMOTE_POS_INVALID;
    }
    if (raw <= thresholds->low_max) {
        return REMOTE_POS_LOW;
    }
    if (raw >= thresholds->center_min && raw <= thresholds->center_max) {
        return REMOTE_POS_CENTER;
    }
    if (raw >= thresholds->high_min) {
        return REMOTE_POS_HIGH;
    }
    return REMOTE_POS_INVALID;
}
