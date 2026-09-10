#include <string.h>
#include <stdint.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "esp_event.h"

#include "i2s_in.h"
#include "rate_table.h"
#include "audio_out.h"
#include "audio_events.h"
#include "app_pins.h"

static const char *TAG = "i2s_in";

/* ------------------------------------------------------------------ */
/* Rate detection                                                      */
/* ------------------------------------------------------------------ */

/*
 * 100 ms gate window, chosen for a hardware reason rather than taste: the ESP32
 * PCNT counter is 16-bit *signed*, so it saturates at 32767. A one-second window
 * at 48 kHz would count 48000 edges and blow straight past that. 100 ms gives
 * 4800 counts at 48 kHz and 19200 at 192 kHz -- comfortably inside the counter,
 * with 10 Hz resolution, which is far finer than we need to tell 44.1 k from
 * 48 k apart.
 */
#define DETECT_WINDOW_MS    100

/* Consecutive agreeing windows before we act on a change. Guards against
 * reporting a rate mid-transition while a source is starting up. */
#define DETECT_STABLE_COUNT 3

/* PCNT limits: the driver requires low < 0 < high. High is set well above the
 * worst-case count per window so a watch point is never hit. */
#define PCNT_HIGH_LIMIT     30000
#define PCNT_LOW_LIMIT      -1

/* ------------------------------------------------------------------ */
/* State                                                               */
/* ------------------------------------------------------------------ */

#define CAPTURE_CHUNK_FRAMES 240

static i2s_chan_handle_t s_rx;
static pcnt_unit_handle_t s_pcnt;
static pcnt_channel_handle_t s_pcnt_chan;

static volatile uint32_t s_rate;          /* 0 == absent */
static volatile bool     s_present;
static volatile bool     s_capture;
static bool              s_rx_enabled;

static uint32_t s_candidate_rate;
static int      s_candidate_count;
static int      s_silent_windows;

static int16_t s_chunk[CAPTURE_CHUNK_FRAMES * 2];

/* ------------------------------------------------------------------ */

static void apply_rate(uint32_t rate)
{
    if (rate == s_rate) {
        return;
    }
    uint32_t old = s_rate;
    s_rate = rate;

    if (rate == 0) {
        s_present = false;
        ESP_LOGI(TAG, "input lost");
        esp_event_post(AUDIO_EVENT, AUDIO_EVENT_I2S_IN_ABSENT, NULL, 0, 0);
        return;
    }

    /* The RX channel's clock config is what the driver uses to size its own
     * internal expectations even in slave mode, so keep it honest across a rate
     * change. This needs the channel stopped, but it only happens when the
     * source actually changes rate. */
    bool was_enabled = s_rx_enabled;
    if (was_enabled) {
        i2s_channel_disable(s_rx);
        s_rx_enabled = false;
    }
    i2s_std_clk_config_t clk = I2S_STD_CLK_DEFAULT_CONFIG(rate);
    esp_err_t err = i2s_channel_reconfig_std_clock(s_rx, &clk);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "RX re-clock to %" PRIu32 " failed: %s", rate, esp_err_to_name(err));
    }
    if (was_enabled) {
        if (i2s_channel_enable(s_rx) == ESP_OK) {
            s_rx_enabled = true;
        }
    }

    s_present = true;
    ESP_LOGI(TAG, "input %s at %" PRIu32 " Hz", old ? "changed to" : "detected", rate);
    audio_evt_stream_t evt = { .sample_rate_hz = rate };
    esp_event_post(AUDIO_EVENT, AUDIO_EVENT_I2S_IN_PRESENT, &evt, sizeof(evt), 0);
}

static void detect_window(void)
{
    /*
     * Measure the gate width instead of trusting the delay.
     *
     * rate = edges / elapsed. Taking `elapsed` from the timer rather than
     * assuming exactly DETECT_WINDOW_MS means scheduler jitter cannot bias the
     * result -- which matters because a 1 ms error on a 100 ms window is a 1 %
     * error on the measured rate, and we are trying to resolve rates that sit
     * 8 % apart.
     */
    static int64_t s_last_us;

    int count = 0;
    int64_t now = esp_timer_get_time();
    pcnt_unit_get_count(s_pcnt, &count);
    pcnt_unit_clear_count(s_pcnt);

    int64_t elapsed_us = now - s_last_us;
    bool first_window = (s_last_us == 0);
    s_last_us = now;
    if (first_window || elapsed_us <= 0) {
        /* The first call has no previous timestamp to measure against -- using
         * it would divide by the whole uptime and report a nonsense rate. The
         * counter has already been cleared, so the next window is valid. */
        return;
    }

    if (count <= 0) {
        s_candidate_rate = 0;
        s_candidate_count = 0;
        if (s_present && ++s_silent_windows >= DETECT_STABLE_COUNT) {
            apply_rate(0);
        }
        return;
    }
    s_silent_windows = 0;

    uint32_t measured = (uint32_t)(((int64_t)count * 1000000) / elapsed_us);
    uint32_t snapped = rate_table_snap(measured);
    if (snapped == 0) {
        /* Present but unintelligible -- log once per transition, do not thrash. */
        if (s_candidate_rate != UINT32_MAX) {
            ESP_LOGW(TAG, "unrecognised input rate ~%" PRIu32 " Hz", measured);
            s_candidate_rate = UINT32_MAX;
        }
        s_candidate_count = 0;
        return;
    }

    if (snapped == s_candidate_rate) {
        if (s_candidate_count < DETECT_STABLE_COUNT) {
            s_candidate_count++;
        }
    } else {
        s_candidate_rate = snapped;
        s_candidate_count = 1;
    }

    if (s_candidate_count >= DETECT_STABLE_COUNT) {
        apply_rate(snapped);
    }
}

/*
 * The detector runs as its own task rather than an esp_timer callback, because
 * apply_rate() may stop and restart the I2S RX channel -- taking driver mutexes
 * and blocking briefly. That is not something to do on the shared esp_timer
 * task, where blocking delays every other timer in the system.
 */
static void i2s_in_detect_task(void *arg)
{
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        xTaskDelayUntil(&last, pdMS_TO_TICKS(DETECT_WINDOW_MS));
        detect_window();
    }
}

/* ------------------------------------------------------------------ */
/* Capture                                                             */
/* ------------------------------------------------------------------ */

static void i2s_in_task(void *arg)
{
    (void)arg;
    for (;;) {
        if (!s_capture || !s_rx_enabled) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        size_t read = 0;
        esp_err_t err = i2s_channel_read(s_rx, s_chunk, sizeof(s_chunk), &read,
                                         pdMS_TO_TICKS(100));
        if (err == ESP_ERR_TIMEOUT) {
            /* Slave mode with no incoming clock: expected while the source is
             * absent. The detector, not this task, decides what that means. */
            continue;
        }
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "read failed: %s", esp_err_to_name(err));
            continue;
        }
        if (s_capture && read >= 4) {
            audio_out_write(s_chunk, read / 4);
        }
    }
}

/* ------------------------------------------------------------------ */
/* Setup                                                               */
/* ------------------------------------------------------------------ */

static esp_err_t pcnt_setup(void)
{
    pcnt_unit_config_t unit_cfg = {
        .high_limit = PCNT_HIGH_LIMIT,
        .low_limit  = PCNT_LOW_LIMIT,
    };
    ESP_RETURN_ON_ERROR(pcnt_new_unit(&unit_cfg, &s_pcnt), TAG, "pcnt_new_unit");

    /*
     * The same GPIO already feeds the I2S peripheral's WS input. That is not a
     * conflict: the ESP32 GPIO matrix fans one input pin out to as many
     * peripheral inputs as want it, so no extra wiring and no arbitration.
     *
     * No glitch filter on this unit. WS at 192 kHz has a 2.6 us half-period, and
     * a filter wide enough to be useful for debouncing would swallow the signal
     * outright. (The encoder's PCNT unit does use one -- separate units,
     * separate filters.)
     */
    pcnt_chan_config_t chan_cfg = {
        .edge_gpio_num  = PIN_I2S_IN_WS,
        .level_gpio_num = -1,
    };
    ESP_RETURN_ON_ERROR(pcnt_new_channel(s_pcnt, &chan_cfg, &s_pcnt_chan), TAG, "pcnt_new_channel");

    /* Count rising edges only: one per WS period, so count == sample count. */
    ESP_RETURN_ON_ERROR(pcnt_channel_set_edge_action(s_pcnt_chan,
                                                     PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                                     PCNT_CHANNEL_EDGE_ACTION_HOLD),
                        TAG, "edge_action");
    ESP_RETURN_ON_ERROR(pcnt_channel_set_level_action(s_pcnt_chan,
                                                      PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                                      PCNT_CHANNEL_LEVEL_ACTION_KEEP),
                        TAG, "level_action");

    ESP_RETURN_ON_ERROR(pcnt_unit_enable(s_pcnt), TAG, "pcnt_unit_enable");
    ESP_RETURN_ON_ERROR(pcnt_unit_clear_count(s_pcnt), TAG, "pcnt_clear");
    ESP_RETURN_ON_ERROR(pcnt_unit_start(s_pcnt), TAG, "pcnt_start");
    return ESP_OK;
}

esp_err_t i2s_in_init(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_SLAVE);
    chan_cfg.dma_desc_num = 6;
    chan_cfg.dma_frame_num = CAPTURE_CHUNK_FRAMES;

    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg, NULL, &s_rx), TAG, "i2s_new_channel");

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(48000),
        /*
         * Philips framing to match the output side.
         *
         * Bring-up warning: ESP32 I2S *slave* mode is fussy about first-bit
         * alignment and the received data can land shifted by one BCLK relative
         * to what the master sent. If a known tone comes through recognisable
         * but distorted, or the channels are swapped, this slot config is the
         * first thing to suspect -- try toggling slot_cfg.bit_shift, and check
         * left_align, before going looking for bugs anywhere else.
         */
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                        I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = PIN_I2S_IN_BCLK,
            .ws   = PIN_I2S_IN_WS,
            .dout = I2S_GPIO_UNUSED,
            .din  = PIN_I2S_IN_DATA,
            .invert_flags = { false, false, false },
        },
    };
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(s_rx, &std_cfg), TAG, "init_std_mode");

    ESP_RETURN_ON_ERROR(pcnt_setup(), TAG, "pcnt_setup");

    ESP_LOGI(TAG, "slave RX ready (BCLK=%d WS=%d DIN=%d), detector on WS",
             PIN_I2S_IN_BCLK, PIN_I2S_IN_WS, PIN_I2S_IN_DATA);
    return ESP_OK;
}

esp_err_t i2s_in_start(void)
{
    BaseType_t ok = xTaskCreatePinnedToCore(i2s_in_task, "i2s_in", 4096, NULL, 22, NULL, 1);
    ESP_RETURN_ON_FALSE(ok == pdPASS, ESP_ERR_NO_MEM, TAG, "capture task");

    ok = xTaskCreatePinnedToCore(i2s_in_detect_task, "i2s_in_det", 3072, NULL, 6, NULL, 1);
    ESP_RETURN_ON_FALSE(ok == pdPASS, ESP_ERR_NO_MEM, TAG, "detect task");
    return ESP_OK;
}

esp_err_t i2s_in_set_capture(bool enable)
{
    if (enable == s_capture) {
        return ESP_OK;
    }

    if (enable) {
        if (!s_rx_enabled) {
            ESP_RETURN_ON_ERROR(i2s_channel_enable(s_rx), TAG, "enable");
            s_rx_enabled = true;
        }
        s_capture = true;
    } else {
        s_capture = false;
        if (s_rx_enabled) {
            i2s_channel_disable(s_rx);
            s_rx_enabled = false;
        }
    }
    ESP_LOGI(TAG, "capture %s", enable ? "on" : "off");
    return ESP_OK;
}

uint32_t i2s_in_get_rate(void)
{
    return s_rate;
}

bool i2s_in_present(void)
{
    return s_present;
}
