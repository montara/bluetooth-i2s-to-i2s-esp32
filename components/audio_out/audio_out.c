#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2s_std.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_event.h"
#include "esp_timer.h"

#include "audio_out.h"
#include "frame_ring.h"
#include "audio_events.h"
#include "app_pins.h"

static const char *TAG = "audio_out";

/* ------------------------------------------------------------------ */
/* Tunables                                                            */
/* ------------------------------------------------------------------ */

/* Power of two, as frame_ring requires. 4096 frames is ~85 ms at 48 kHz and
 * costs 16 KB -- a deliberate compromise on a WROOM-32 where Bluedroid wants
 * every byte it can get. Deeper tolerates a burstier producer; shallower means
 * less latency and less RAM. */
#define RING_FRAMES         4096u

/* How much the pump moves per I2S write. 240 frames is 5 ms at 48 kHz. */
#define CHUNK_FRAMES        240u

#define TARGET_FILL_PCT     50
#define RESYNC_LO_PCT       10
#define RESYNC_HI_PCT       90

#define SERVO_PERIOD_MS     100

/*
 * Servo gains.
 *
 * The actuator is i2s_channel_tune_rate() in ADDSUB mode, which *accumulates*:
 * each tick adds `u` to the current MCLK. So MCLK is already an integrator of
 * our output, and buffer fill is an integrator of the rate error. Two
 * integrators in series is a pure oscillator -- a proportional-only law here
 * rings forever instead of settling. The damping has to come from a term on the
 * *change* in error, which is why this is P+D on fill (equivalently PI on
 * frequency) rather than the PI-on-fill you might reach for first.
 *
 * Sizing, with e in frames and one tick = 0.1 s:
 *   d2e/dt2 + b*de/dt + a*e = 0,  a = Kp/(256*T),  b = Kd/(256*T)
 * Targeting wn = 0.5 rad/s (settles in ~15 s, slow enough to be inaudible) and
 * zeta = 0.8 (no overshoot worth hearing):
 *   a = wn^2 = 0.25       -> Kp = 0.25 * 256 * 0.1 = 6.4 Hz per frame
 *   b = 2*zeta*wn = 0.8   -> Kd = 0.8  * 256 * 0.1 = 20.5 Hz per (frame/s)
 * Kd multiplies (e - e_prev), which is de/dt * T, so it scales by 1/T:
 *   Kd_discrete = 20.5 / 0.1 = 205 Hz per frame of delta
 *
 * Derived on paper; expect to trim on the bench. Watch `stats`: a healthy loop
 * parks near 50 % with a steady mclk offset. Hunting around 50 % means too much
 * gain; a slow monotonic drift to 0 or 100 % means too little (or a sign error).
 */
#define SERVO_KP_X10        64      /* 6.4   */
#define SERVO_KD_X10        2048    /* 204.8 */

/* Ceiling on how far the servo may pull MCLK from nominal. +/-20 kHz on a
 * 12.288 MHz MCLK is +/-0.16 %, well inside what a PCM5102A-class DAC will
 * swallow and far more than the tens of ppm a real crystal mismatch needs. */
#define SERVO_MAX_DELTA_HZ  20000

/* Per-tick slew limit, so a transient cannot slam the clock in one step. */
#define SERVO_MAX_STEP_HZ   2000

#define UNDERRUN_POST_MIN_INTERVAL_US   (5 * 1000 * 1000)

#define DEFAULT_SAMPLE_RATE 48000u

/* ------------------------------------------------------------------ */
/* State                                                               */
/* ------------------------------------------------------------------ */

static frame_ring_t s_ring;
static i2s_chan_handle_t s_tx;
static uint32_t s_rate = DEFAULT_SAMPLE_RATE;
static volatile bool s_streaming;
static volatile bool s_servo_enabled = true;
static SemaphoreHandle_t s_rate_lock;

static uint32_t s_underruns;
static uint32_t s_underruns_reported;
static int64_t  s_underrun_last_post_us;
static uint32_t s_resyncs;
static int32_t  s_servo_prev_err;
static int32_t  s_mclk_delta;
static uint32_t s_mclk_hz;

static int16_t s_chunk[CHUNK_FRAMES * 2];

/* ------------------------------------------------------------------ */

static i2s_std_config_t make_std_cfg(uint32_t rate)
{
    i2s_std_config_t cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(rate),
        /* Philips framing (data delayed one BCLK after the WS edge) is what
         * PCM5102A-class DACs expect. Note the IDF a2dp_sink example uses MSB
         * framing instead -- that is for its internal-DAC path, not for us. */
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                        I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,   /* the DAC derives everything from BCLK */
            .bclk = PIN_I2S_OUT_BCLK,
            .ws   = PIN_I2S_OUT_WS,
            .dout = PIN_I2S_OUT_DATA,
            .din  = I2S_GPIO_UNUSED,
            .invert_flags = { false, false, false },
        },
    };
    /* APLL, not the 160 MHz PLL: the servo needs a clock that can be trimmed in
     * fine steps, and APLL is the only source on ESP32 with that resolution.
     * MCLK is not routed to a pin but still sets BCLK. */
    cfg.clk_cfg.clk_src = I2S_CLK_SRC_APLL;
    cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;
    return cfg;
}

esp_err_t audio_out_init(void)
{
    s_rate_lock = xSemaphoreCreateMutex();
    ESP_RETURN_ON_FALSE(s_rate_lock, ESP_ERR_NO_MEM, TAG, "no mem for lock");

    int16_t *storage = heap_caps_malloc(RING_FRAMES * 2 * sizeof(int16_t),
                                        MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    ESP_RETURN_ON_FALSE(storage, ESP_ERR_NO_MEM, TAG,
                        "no mem for ring (%u frames)", (unsigned)RING_FRAMES);
    ESP_RETURN_ON_FALSE(frame_ring_init(&s_ring, storage, RING_FRAMES),
                        ESP_ERR_INVALID_ARG, TAG, "ring init");
    frame_ring_recentre(&s_ring, TARGET_FILL_PCT);

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = 6;
    chan_cfg.dma_frame_num = CHUNK_FRAMES;
    /* auto_clear makes the DMA emit zeros rather than replay the stale tail of
     * the previous buffer if we ever fail to keep up. Cheap insurance against a
     * loud repeated fragment. */
    chan_cfg.auto_clear = true;

    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg, &s_tx, NULL), TAG, "i2s_new_channel");

    i2s_std_config_t std_cfg = make_std_cfg(s_rate);
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(s_tx, &std_cfg), TAG, "init_std_mode");

    ESP_LOGI(TAG, "TX ready: %" PRIu32 " Hz, ring %u frames (%u ms, %u KB)",
             s_rate, (unsigned)RING_FRAMES,
             (unsigned)(RING_FRAMES * 1000 / s_rate),
             (unsigned)(RING_FRAMES * 4 / 1024));
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/* Pump                                                                */
/* ------------------------------------------------------------------ */

static void audio_out_pump_task(void *arg)
{
    (void)arg;
    for (;;) {
        size_t got = frame_ring_read(&s_ring, s_chunk, CHUNK_FRAMES);
        if (got < CHUNK_FRAMES) {
            /* Pad with silence rather than short-writing. A short write lets
             * the DMA run dry and produces a discontinuity at the buffer
             * boundary; padding keeps the clock and frame alignment intact and
             * costs nothing but the audio that was not there anyway. */
            memset(&s_chunk[got * 2], 0, (CHUNK_FRAMES - got) * 2 * sizeof(int16_t));
            if (s_streaming) {
                s_underruns++;
            }
        }
        size_t written = 0;
        /* This blocking write paces the loop: it returns at exactly the rate
         * the DAC consumes, so no explicit timing is needed anywhere. */
        i2s_channel_write(s_tx, s_chunk, sizeof(s_chunk), &written, portMAX_DELAY);
        (void)written;
    }
}

/* ------------------------------------------------------------------ */
/* Drift servo                                                         */
/* ------------------------------------------------------------------ */

static void servo_apply(int32_t delta_hz)
{
    i2s_tuning_config_t tune = {
        .tune_mode      = I2S_TUNING_MODE_ADDSUB,
        .tune_mclk_val  = delta_hz,
        .max_delta_mclk =  SERVO_MAX_DELTA_HZ,
        .min_delta_mclk = -SERVO_MAX_DELTA_HZ,
    };
    i2s_tuning_info_t info = { 0 };
    if (i2s_channel_tune_rate(s_tx, &tune, &info) == ESP_OK) {
        s_mclk_hz = (uint32_t)info.curr_mclk_hz;
        s_mclk_delta = info.delta_mclk_hz;
    }
}

static void servo_reset(void)
{
    i2s_tuning_config_t tune = {
        .tune_mode      = I2S_TUNING_MODE_RESET,
        .max_delta_mclk =  SERVO_MAX_DELTA_HZ,
        .min_delta_mclk = -SERVO_MAX_DELTA_HZ,
    };
    i2s_tuning_info_t info = { 0 };
    if (i2s_channel_tune_rate(s_tx, &tune, &info) == ESP_OK) {
        s_mclk_hz = (uint32_t)info.curr_mclk_hz;
        s_mclk_delta = info.delta_mclk_hz;
    }
    s_servo_prev_err = 0;
}

static void maybe_report_underruns(void)
{
    /* Rate-limited on purpose: a broken source can underrun on every chunk, and
     * a flood of events would do more damage than the glitch it reports. */
    if (s_underruns == s_underruns_reported) {
        return;
    }
    int64_t now = esp_timer_get_time();
    if (now - s_underrun_last_post_us < UNDERRUN_POST_MIN_INTERVAL_US) {
        return;
    }
    s_underrun_last_post_us = now;
    s_underruns_reported = s_underruns;

    audio_evt_underrun_t evt = { .underruns = s_underruns };
    esp_event_post(AUDIO_EVENT, AUDIO_EVENT_UNDERRUN, &evt, sizeof(evt), 0);
}

static void audio_out_servo_task(void *arg)
{
    (void)arg;
    const int32_t target = (int32_t)((RING_FRAMES * TARGET_FILL_PCT) / 100);

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(SERVO_PERIOD_MS));

        maybe_report_underruns();

        if (!s_streaming || !s_servo_enabled) {
            s_servo_prev_err = 0;
            continue;
        }

        int32_t fill = (int32_t)frame_ring_fill(&s_ring);
        int pct = (int)((fill * 100) / (int32_t)RING_FRAMES);

        /* Outside the safe band the buffer is not drifting -- something broke:
         * the source changed rate, a cable was pulled, the stream stalled. Fine
         * tuning cannot dig out of that, so re-centre and start over rather
         * than winding the clock to its limit trying. */
        if (pct < RESYNC_LO_PCT || pct > RESYNC_HI_PCT) {
            ESP_LOGW(TAG, "buffer at %d%%, resyncing", pct);
            frame_ring_recentre(&s_ring, TARGET_FILL_PCT);
            servo_reset();
            s_resyncs++;
            continue;
        }

        int32_t err = fill - target;
        int32_t derr = err - s_servo_prev_err;
        s_servo_prev_err = err;

        int32_t u = (SERVO_KP_X10 * err + SERVO_KD_X10 * derr) / 10;
        if (u >  SERVO_MAX_STEP_HZ) u =  SERVO_MAX_STEP_HZ;
        if (u < -SERVO_MAX_STEP_HZ) u = -SERVO_MAX_STEP_HZ;

        servo_apply(u);
    }
}

esp_err_t audio_out_start(void)
{
    ESP_RETURN_ON_ERROR(i2s_channel_enable(s_tx), TAG, "i2s_channel_enable");
    servo_reset();

    /* Core 1, high priority: the audio path gets a core largely to itself while
     * Bluedroid and LVGL live on core 0. */
    BaseType_t ok = xTaskCreatePinnedToCore(audio_out_pump_task, "aout_pump", 4096, NULL, 23, NULL, 1);
    ESP_RETURN_ON_FALSE(ok == pdPASS, ESP_ERR_NO_MEM, TAG, "pump task");

    ok = xTaskCreatePinnedToCore(audio_out_servo_task, "aout_servo", 3072, NULL, 5, NULL, 1);
    ESP_RETURN_ON_FALSE(ok == pdPASS, ESP_ERR_NO_MEM, TAG, "servo task");

    ESP_LOGI(TAG, "output started");
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

esp_err_t audio_out_set_rate(uint32_t sample_rate_hz)
{
    if (sample_rate_hz == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_rate_lock, portMAX_DELAY);
    esp_err_t err = ESP_OK;

    if (sample_rate_hz != s_rate) {
        ESP_LOGI(TAG, "re-clocking %" PRIu32 " -> %" PRIu32 " Hz", s_rate, sample_rate_hz);

        /* reconfig_std_clock genuinely does require a stopped channel (unlike
         * tune_rate, whose "READY only" doc note the implementation does not
         * enforce). That is why this is reserved for real rate changes. */
        err = i2s_channel_disable(s_tx);
        if (err == ESP_OK) {
            i2s_std_clk_config_t clk = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate_hz);
            clk.clk_src = I2S_CLK_SRC_APLL;
            clk.mclk_multiple = I2S_MCLK_MULTIPLE_256;
            err = i2s_channel_reconfig_std_clock(s_tx, &clk);
        }
        if (err == ESP_OK) {
            s_rate = sample_rate_hz;
            frame_ring_recentre(&s_ring, TARGET_FILL_PCT);
            err = i2s_channel_enable(s_tx);
        }
        if (err == ESP_OK) {
            /* The reconfiguration reset the driver's notion of nominal MCLK, so
             * the servo's accumulated correction is meaningless now. */
            servo_reset();
        } else {
            ESP_LOGE(TAG, "re-clock failed: %s", esp_err_to_name(err));
        }
    }

    xSemaphoreGive(s_rate_lock);
    return err;
}

uint32_t audio_out_get_rate(void)
{
    return s_rate;
}

size_t audio_out_write(const int16_t *frames, size_t frame_count)
{
    return frame_ring_write(&s_ring, frames, frame_count);
}

void audio_out_flush(void)
{
    frame_ring_recentre(&s_ring, TARGET_FILL_PCT);
    s_servo_prev_err = 0;
}

void audio_out_set_streaming(bool streaming)
{
    if (s_streaming != streaming) {
        s_streaming = streaming;
        s_servo_prev_err = 0;
    }
}

void audio_out_servo_enable(bool enable)
{
    s_servo_enabled = enable;
    if (!enable) {
        servo_reset();
    }
}

esp_err_t audio_out_tune_manual(int32_t delta_hz)
{
    servo_apply(delta_hz);
    return ESP_OK;
}

void audio_out_get_stats(audio_out_stats_t *out)
{
    if (!out) {
        return;
    }
    uint32_t fill = frame_ring_fill(&s_ring);

    out->sample_rate_hz  = s_rate;
    out->fill_frames     = fill;
    out->capacity_frames = RING_FRAMES;
    out->fill_pct        = (int)((fill * 100) / RING_FRAMES);
    out->mclk_delta_hz   = s_mclk_delta;
    out->mclk_hz         = s_mclk_hz;
    out->underruns       = s_underruns;
    out->overflows       = s_ring.dropped;
    out->resyncs         = s_resyncs;
    out->streaming       = s_streaming;
    out->servo_enabled   = s_servo_enabled;
}
