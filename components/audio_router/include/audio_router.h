/*
 * Decides who gets the DAC.
 *
 * The router is the only place that knows both producers exist. It listens on
 * the event bus, applies the user's mode, and performs the switch: quiesce the
 * outgoing producer, re-clock the output, admit the incoming one.
 */
#pragma once

#include "esp_err.h"
#include "audio_events.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Restore the saved mode from NVS, subscribe to the event bus, start the task. */
esp_err_t audio_router_init(void);

/** Change and persist the routing mode. Takes effect after the usual debounce. */
esp_err_t audio_router_set_mode(audio_route_mode_t mode);

audio_route_mode_t audio_router_get_mode(void);

/** Which producer is on air right now. */
audio_source_t audio_router_get_source(void);

#ifdef __cplusplus
}
#endif
