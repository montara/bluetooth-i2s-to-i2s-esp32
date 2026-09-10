/*
 * The wired I2S input.
 *
 * The ESP32 is the *slave* here: the external source drives BCLK and WS, so the
 * chip has no idea what sample rate is arriving and no way to ask. We work it
 * out by counting WS edges -- the WS frequency *is* the sample rate -- using a
 * PCNT unit fed from the same GPIO the I2S peripheral is listening on. That
 * also gives signal-presence detection for free: no edges means no source.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Create the RX channel and the rate detector. Capture starts disabled. */
esp_err_t i2s_in_init(void);

/** Start the detector and the capture task. */
esp_err_t i2s_in_start(void);

/**
 * Enable or disable forwarding into audio_out.
 *
 * Disabling also stops the I2S RX channel, which matters: while Bluetooth owns
 * the DAC there is no reason to keep a DMA channel running and dropping frames
 * on the floor. The rate detector keeps running either way, because the router
 * needs to know whether the wired source is still there to fall back to.
 */
esp_err_t i2s_in_set_capture(bool enable);

/** Last stable detected rate in Hz, or 0 if no source is present. */
uint32_t i2s_in_get_rate(void);

/** True while WS edges are arriving. */
bool i2s_in_present(void);

#ifdef __cplusplus
}
#endif
