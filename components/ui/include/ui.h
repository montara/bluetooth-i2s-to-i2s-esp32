/*
 * The 2.4" TFT user interface.
 *
 * Deliberately a read-only view over the event bus plus one action (choose a
 * routing mode). The UI never inspects the audio path directly and never calls
 * into it except through audio_router -- so a change to how audio is routed
 * cannot break the display, and vice versa.
 */
#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Bring up the panel, LVGL, the encoder input device and the screens.
 *
 * Call this BEFORE bt_audio_init(). LVGL's draw buffers must be DMA-capable
 * and contiguous, and Bluedroid fragments internal RAM badly once it starts --
 * whoever asks first gets the good memory.
 */
esp_err_t ui_init(void);

#ifdef __cplusplus
}
#endif
