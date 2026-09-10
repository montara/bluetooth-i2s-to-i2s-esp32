/*
 * Bluetooth Classic A2DP audio sink.
 *
 * Only the original ESP32 can do this at all -- A2DP rides on Bluetooth Classic
 * (BR/EDR), which the S3/C3/C6 do not have. If this firmware is ever retargeted,
 * this component is the part that cannot come along.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Bring up the controller, Bluedroid, A2DP sink and AVRCP. Discoverable on return. */
esp_err_t bt_audio_init(const char *device_name);

/** True between A2DP connect and disconnect, regardless of whether audio flows. */
bool bt_audio_is_connected(void);

/** True while the peer is actually streaming (A2DP audio state STARTED). */
bool bt_audio_is_streaming(void);

/** Negotiated stream rate in Hz, or 0 if not yet known. */
uint32_t bt_audio_get_rate(void);

/** Peer's friendly name if GAP has resolved it, else its address as text. */
const char *bt_audio_peer_name(void);

/** Forward incoming PCM into audio_out, or drop it on the floor. */
void bt_audio_set_capture(bool enable);

/** Absolute volume last set by the peer, 0..127. */
uint8_t bt_audio_get_volume(void);

esp_err_t bt_audio_disconnect(void);

#ifdef __cplusplus
}
#endif
