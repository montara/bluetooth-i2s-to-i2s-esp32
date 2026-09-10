/*
 * EC11 rotary encoder + select + back, as an LVGL input device.
 *
 * Written directly against PCNT and GPIO rather than pulling in
 * espressif/knob + espressif/button. Two reasons:
 *
 *   - LVGL's encoder input model has no concept of a *back* button. It knows
 *     rotate and press. A back button that pops a screen stack has to be
 *     handled outside LVGL's indev regardless, so half of this layer would be
 *     custom either way.
 *   - Those components carry their own version constraints against
 *     esp_lvgl_port, and matching them up is more moving parts than the ~150
 *     lines they would save.
 */
#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Called when the back button is pressed.
 *
 * Runs in the esp_timer task, NOT the LVGL task. It must not touch LVGL
 * directly and must not block -- defer the screen change to the UI task.
 */
typedef void (*ui_input_back_cb_t)(void *ctx);

/** Set up PCNT quadrature decoding and the button debouncer. */
esp_err_t ui_input_init(void);

/** Create the LVGL encoder input device and bind it to a group. */
lv_indev_t *ui_input_create_indev(lv_group_t *group);

void ui_input_set_back_cb(ui_input_back_cb_t cb, void *ctx);

#ifdef __cplusplus
}
#endif
