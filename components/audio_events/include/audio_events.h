/*
 * The shared vocabulary of the firmware.
 *
 * This component holds types and an esp_event base and nothing else. Every
 * other component depends on it and it depends on none of them, which is what
 * keeps the audio path, the router and the UI free of dependency cycles: the
 * producers post events, the router and the UI subscribe, and nobody reaches
 * sideways into anybody else's state.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_event.h"

#ifdef __cplusplus
extern "C" {
#endif

ESP_EVENT_DECLARE_BASE(AUDIO_EVENT);

/** Which producer is currently feeding the DAC. */
typedef enum {
    AUDIO_SRC_NONE = 0,   /*!< nothing is playing; the DAC is fed silence */
    AUDIO_SRC_I2S_IN,     /*!< the wired I2S input                        */
    AUDIO_SRC_BT,         /*!< the Bluetooth A2DP stream                  */
} audio_source_t;

/** What the user has asked the router to do. Persisted in NVS. */
typedef enum {
    AUDIO_ROUTE_AUTO = 0,   /*!< Bluetooth wins while it is streaming, else I2S */
    AUDIO_ROUTE_FORCE_I2S,  /*!< always the wired input                         */
    AUDIO_ROUTE_FORCE_BT,   /*!< always Bluetooth                               */
} audio_route_mode_t;

typedef enum {
    AUDIO_EVENT_SOURCE_CHANGED,    /*!< audio_evt_source_t   */
    AUDIO_EVENT_MODE_CHANGED,      /*!< audio_evt_mode_t     */
    AUDIO_EVENT_I2S_IN_PRESENT,    /*!< audio_evt_stream_t   */
    AUDIO_EVENT_I2S_IN_ABSENT,     /*!< no payload           */
    AUDIO_EVENT_BT_CONNECTED,      /*!< audio_evt_bt_t       */
    AUDIO_EVENT_BT_DISCONNECTED,   /*!< no payload           */
    AUDIO_EVENT_BT_STREAM_START,   /*!< audio_evt_stream_t   */
    AUDIO_EVENT_BT_STREAM_STOP,    /*!< no payload           */
    AUDIO_EVENT_UNDERRUN,          /*!< audio_evt_underrun_t */
} audio_event_id_t;

typedef struct {
    audio_source_t source;
    uint32_t sample_rate_hz;
} audio_evt_source_t;

typedef struct {
    audio_route_mode_t mode;
} audio_evt_mode_t;

typedef struct {
    uint32_t sample_rate_hz;
} audio_evt_stream_t;

#define AUDIO_BT_NAME_MAX 32

typedef struct {
    char name[AUDIO_BT_NAME_MAX];
    uint8_t bda[6];
} audio_evt_bt_t;

typedef struct {
    uint32_t underruns;   /*!< running total since boot */
} audio_evt_underrun_t;

/** Short human-readable names, for the UI and the console. */
const char *audio_source_name(audio_source_t src);
const char *audio_route_mode_name(audio_route_mode_t mode);

#ifdef __cplusplus
}
#endif
