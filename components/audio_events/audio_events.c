#include "audio_events.h"

ESP_EVENT_DEFINE_BASE(AUDIO_EVENT);

const char *audio_source_name(audio_source_t src)
{
    switch (src) {
    case AUDIO_SRC_I2S_IN: return "I2S In";
    case AUDIO_SRC_BT:     return "Bluetooth";
    case AUDIO_SRC_NONE:   /* fall through */
    default:               return "Idle";
    }
}

const char *audio_route_mode_name(audio_route_mode_t mode)
{
    switch (mode) {
    case AUDIO_ROUTE_FORCE_I2S: return "I2S In";
    case AUDIO_ROUTE_FORCE_BT:  return "Bluetooth";
    case AUDIO_ROUTE_AUTO:      /* fall through */
    default:                    return "Auto";
    }
}
