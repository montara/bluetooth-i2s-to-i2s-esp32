#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_event.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "audio_router.h"
#include "audio_out.h"
#include "i2s_in.h"
#include "bt_audio.h"
#include "audio_events.h"

static const char *TAG = "router";

#define NVS_NAMESPACE   "audio"
#define NVS_KEY_MODE    "route_mode"

/*
 * How long the inputs must hold still before we act on them.
 *
 * A2DP streams stutter: a phone pausing between tracks, a moment of Wi-Fi
 * contention, or a brief RF dropout all produce STOP/START pairs milliseconds
 * apart. Without this, every one of them would tear down the route, re-clock
 * the DAC and re-centre the buffer -- turning a glitch we would not have heard
 * into an audible one, repeatedly.
 */
#define DEBOUNCE_MS     250

/* Fallback when a source is selected but has not reported a rate yet. */
#define FALLBACK_RATE   48000

static volatile audio_route_mode_t s_mode = AUDIO_ROUTE_AUTO;
static volatile audio_source_t s_source = AUDIO_SRC_NONE;
static TaskHandle_t s_task;

/* ------------------------------------------------------------------ */
/* Persistence                                                         */
/* ------------------------------------------------------------------ */

static void mode_load(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        return;   /* never written yet; the AUTO default stands */
    }
    uint8_t stored = 0;
    if (nvs_get_u8(h, NVS_KEY_MODE, &stored) == ESP_OK && stored <= AUDIO_ROUTE_FORCE_BT) {
        s_mode = (audio_route_mode_t)stored;
    }
    nvs_close(h);
}

static void mode_store(audio_route_mode_t mode)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGW(TAG, "cannot open NVS to persist mode");
        return;
    }
    nvs_set_u8(h, NVS_KEY_MODE, (uint8_t)mode);
    nvs_commit(h);
    nvs_close(h);
}

/* ------------------------------------------------------------------ */
/* Switching                                                           */
/* ------------------------------------------------------------------ */

static audio_source_t decide(void)
{
    bool bt = bt_audio_is_streaming();
    bool i2s = i2s_in_present();

    switch (s_mode) {
    case AUDIO_ROUTE_FORCE_I2S:
        return i2s ? AUDIO_SRC_I2S_IN : AUDIO_SRC_NONE;
    case AUDIO_ROUTE_FORCE_BT:
        return bt ? AUDIO_SRC_BT : AUDIO_SRC_NONE;
    case AUDIO_ROUTE_AUTO:
    default:
        /* Bluetooth wins while it is actually streaming -- a phone merely being
         * connected is not a reason to mute the wired input. */
        if (bt) {
            return AUDIO_SRC_BT;
        }
        return i2s ? AUDIO_SRC_I2S_IN : AUDIO_SRC_NONE;
    }
}

static uint32_t rate_for(audio_source_t src)
{
    uint32_t hz = 0;
    switch (src) {
    case AUDIO_SRC_I2S_IN: hz = i2s_in_get_rate();   break;
    case AUDIO_SRC_BT:     hz = bt_audio_get_rate(); break;
    default: break;
    }
    return hz ? hz : FALLBACK_RATE;
}

static void apply(audio_source_t want)
{
    if (want == s_source) {
        return;
    }

    ESP_LOGI(TAG, "%s -> %s", audio_source_name(s_source), audio_source_name(want));

    /* Order matters. Stop the old producer writing *before* touching the buffer,
     * or its in-flight frames land in the ring we just emptied and the new
     * source starts out mixed with the tail of the old one. */
    i2s_in_set_capture(false);
    bt_audio_set_capture(false);
    audio_out_set_streaming(false);

    audio_out_set_rate(rate_for(want));

    switch (want) {
    case AUDIO_SRC_I2S_IN: i2s_in_set_capture(true);   break;
    case AUDIO_SRC_BT:     bt_audio_set_capture(true); break;
    default: break;
    }
    audio_out_set_streaming(want != AUDIO_SRC_NONE);

    s_source = want;

    audio_evt_source_t evt = { .source = want, .sample_rate_hz = audio_out_get_rate() };
    esp_event_post(AUDIO_EVENT, AUDIO_EVENT_SOURCE_CHANGED, &evt, sizeof(evt), 0);
}

/* ------------------------------------------------------------------ */
/* Task                                                                */
/* ------------------------------------------------------------------ */

static void router_task(void *arg)
{
    (void)arg;

    /* Settle once at boot so a source already present is picked up without
     * waiting for an event that may never come. */
    apply(decide());

    for (;;) {
        /*
         * Block until something pokes us, then wait for DEBOUNCE_MS of quiet
         * before acting. Each further poke inside the window restarts the wait,
         * so a burst of A2DP start/stop chatter collapses into one decision
         * made after the dust settles.
         */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        while (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(DEBOUNCE_MS)) > 0) {
            /* still churning; keep waiting */
        }
        apply(decide());
    }
}

static void on_audio_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg; (void)base; (void)data;

    switch (id) {
    case AUDIO_EVENT_I2S_IN_PRESENT:
    case AUDIO_EVENT_I2S_IN_ABSENT:
    case AUDIO_EVENT_BT_STREAM_START:
    case AUDIO_EVENT_BT_STREAM_STOP:
    case AUDIO_EVENT_BT_DISCONNECTED:
    case AUDIO_EVENT_MODE_CHANGED:
        if (s_task) {
            xTaskNotifyGive(s_task);
        }
        break;
    default:
        /* SOURCE_CHANGED is ours; reacting to it would be a loop. */
        break;
    }
}

/* ------------------------------------------------------------------ */

esp_err_t audio_router_init(void)
{
    mode_load();
    ESP_LOGI(TAG, "mode: %s", audio_route_mode_name(s_mode));

    ESP_RETURN_ON_ERROR(esp_event_handler_register(AUDIO_EVENT, ESP_EVENT_ANY_ID,
                                                   on_audio_event, NULL),
                        TAG, "event_register");

    BaseType_t ok = xTaskCreatePinnedToCore(router_task, "router", 3072, NULL, 8, &s_task, 0);
    ESP_RETURN_ON_FALSE(ok == pdPASS, ESP_ERR_NO_MEM, TAG, "router task");
    return ESP_OK;
}

esp_err_t audio_router_set_mode(audio_route_mode_t mode)
{
    if (mode > AUDIO_ROUTE_FORCE_BT) {
        return ESP_ERR_INVALID_ARG;
    }
    if (mode == s_mode) {
        return ESP_OK;
    }

    s_mode = mode;
    mode_store(mode);
    ESP_LOGI(TAG, "mode -> %s", audio_route_mode_name(mode));

    audio_evt_mode_t evt = { .mode = mode };
    esp_event_post(AUDIO_EVENT, AUDIO_EVENT_MODE_CHANGED, &evt, sizeof(evt), 0);
    return ESP_OK;
}

audio_route_mode_t audio_router_get_mode(void)
{
    return s_mode;
}

audio_source_t audio_router_get_source(void)
{
    return s_source;
}
