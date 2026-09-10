/*
 * I2S passthrough DAC with Bluetooth audio and a TFT front panel.
 *
 * Wired I2S in -> DAC is the default path. A phone that connects over A2DP and
 * starts streaming takes over automatically; when it stops, the wired input
 * comes back. The encoder and screen let you pin it to one or the other.
 */
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "nvs_flash.h"

#include "audio_out.h"
#include "i2s_in.h"
#include "bt_audio.h"
#include "audio_router.h"
#include "ui.h"
#include "ui_input.h"
#include "console_cmds.h"

static const char *TAG = "main";

static void nvs_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS needs erasing (%s)", esp_err_to_name(err));
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

void app_main(void)
{
    ESP_LOGI(TAG, "starting, %u KB internal heap free",
             (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024));

    nvs_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /*
     * Init order is not arbitrary.
     *
     * 1. audio_out first: everything downstream writes into its buffer, so it
     *    has to exist before anything can produce.
     * 2. i2s_in next; it only feeds audio_out.
     * 3. The UI *before* Bluetooth. LVGL needs two contiguous DMA-capable draw
     *    buffers, and Bluedroid fragments internal RAM badly once it comes up.
     *    Asking for the big allocations first is the difference between a
     *    display that works and one that fails to allocate on a WROOM-32.
     * 4. Bluetooth, then the router -- which needs both producers to already
     *    exist, because its first act is to survey them and pick one.
     * 5. Console last, so its banner does not land in the middle of init logs.
     */
    ESP_ERROR_CHECK(audio_out_init());
    ESP_ERROR_CHECK(audio_out_start());

    ESP_ERROR_CHECK(i2s_in_init());
    ESP_ERROR_CHECK(i2s_in_start());

    ESP_ERROR_CHECK(ui_input_init());
    ESP_ERROR_CHECK(ui_init());

    ESP_ERROR_CHECK(bt_audio_init(CONFIG_BT_AUDIO_DEVICE_NAME));

    ESP_ERROR_CHECK(audio_router_init());

    ESP_ERROR_CHECK(console_cmds_init());

    ESP_LOGI(TAG, "ready, %u KB internal heap free",
             (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024));
}
