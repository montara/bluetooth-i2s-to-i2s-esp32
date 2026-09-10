/*
 * The one and only path to the DAC.
 *
 * audio_out owns the I2S1 master TX channel and a single elastic ring buffer.
 * Producers (the wired I2S input, the Bluetooth stream) push frames in;
 * a pump task drains them to I2S. Which producer is allowed to push is the
 * router's business, not ours.
 *
 * Everything here speaks in *frames*: one frame is one stereo pair of int16_t,
 * i.e. 4 bytes.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t sample_rate_hz;  /*!< rate the TX channel is currently clocked at */
    uint32_t fill_frames;     /*!< frames currently queued in the elastic buffer */
    uint32_t capacity_frames;
    int      fill_pct;
    int32_t  mclk_delta_hz;   /*!< how far the servo has pulled MCLK from nominal */
    uint32_t mclk_hz;
    uint32_t underruns;       /*!< pump found the buffer empty while streaming */
    uint32_t overflows;       /*!< a producer pushed into a full buffer */
    uint32_t resyncs;         /*!< buffer left the safe band and was re-centred */
    bool     streaming;
    bool     servo_enabled;
} audio_out_stats_t;

/** Create the I2S TX channel and the elastic buffer. Does not start audio. */
esp_err_t audio_out_init(void);

/** Start the pump and servo tasks. The DAC is fed silence until a producer writes. */
esp_err_t audio_out_start(void);

/**
 * Re-clock the TX channel.
 *
 * Unlike the servo's fine tuning this is a real reconfiguration: it stops the
 * channel, changes the clock, empties and re-centres the buffer, and starts
 * again. Only call it on an actual rate change -- it is audible.
 */
esp_err_t audio_out_set_rate(uint32_t sample_rate_hz);

uint32_t audio_out_get_rate(void);

/**
 * Push stereo frames. Never blocks.
 *
 * @return the number of frames actually accepted; anything less means the
 *         buffer was full and the shortfall was dropped (and counted).
 */
size_t audio_out_write(const int16_t *frames, size_t frame_count);

/** Drop everything queued and re-centre the buffer with silence. */
void audio_out_flush(void);

/**
 * Tell the output whether a producer is expected to be feeding it.
 *
 * This gates underrun accounting and the drift servo: while idle, an empty
 * buffer is the normal state and neither should be complaining about it.
 */
void audio_out_set_streaming(bool streaming);

/** Turn the drift servo off, e.g. to measure raw drift during bring-up. */
void audio_out_servo_enable(bool enable);

/**
 * Nudge MCLK by hand. Bring-up aid: proves tune_rate reaches the hardware on a
 * *running* channel, which is the assumption the servo rests on.
 */
esp_err_t audio_out_tune_manual(int32_t delta_hz);

void audio_out_get_stats(audio_out_stats_t *out);

#ifdef __cplusplus
}
#endif
