#include <string.h>
#include <stdio.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_event.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"

#include "bt_audio.h"
#include "audio_out.h"
#include "audio_events.h"

static const char *TAG = "bt_audio";

/*
 * A note on which A2DP data path this uses, because it is not obvious and it is
 * load-bearing.
 *
 * ESP-IDF offers two ways to receive A2DP audio:
 *
 *   1. esp_a2d_sink_register_data_callback() -- Bluedroid's *internal* SBC
 *      decoder hands us ready-made 16-bit stereo PCM. This is what we use.
 *   2. esp_a2d_sink_register_audio_data_callback() -- raw, still-encoded SBC
 *      frames, which the application is expected to decode itself.
 *
 * As of v5.5 option 1 has been moved out of esp_a2dp_api.h into
 * esp_a2dp_legacy_api.h (which esp_a2dp_api.h still includes, so the call
 * compiles unchanged), and the Kconfig help for BT_A2DP_USE_EXTERNAL_CODEC says
 * the internal codec "will be removed in the future". So this is a legacy path
 * with a known sunset.
 *
 * We take it anyway: it is the difference between this component being ~300
 * lines and it needing a full SBC decoder. If a future IDF drops it, the fix is
 * to set BT_A2DP_USE_EXTERNAL_CODEC=y, register the endpoint and the audio-data
 * callback, and decode into audio_out_write() -- everything else here, and
 * everything downstream of here, is unaffected.
 */

#define VOLUME_DEFAULT      96      /* ~75%, until the peer tells us otherwise */
#define VOLUME_MAX          127
#define SCALE_CHUNK_FRAMES  256

static volatile bool     s_connected;
static volatile bool     s_streaming;
static volatile bool     s_capture;
static volatile uint32_t s_rate;
static volatile uint8_t  s_volume = VOLUME_DEFAULT;

static esp_bd_addr_t s_peer_bda;
static char s_peer_name[AUDIO_BT_NAME_MAX];

/* Touched only from the Bluedroid A2DP task, so a single static is safe. */
static int16_t s_scale_buf[SCALE_CHUNK_FRAMES * 2];

/* ------------------------------------------------------------------ */

static void set_peer_name_from_bda(const esp_bd_addr_t bda)
{
    snprintf(s_peer_name, sizeof(s_peer_name), "%02x:%02x:%02x:%02x:%02x:%02x",
             bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
}

static uint32_t sbc_samp_freq_to_hz(uint8_t samp_freq)
{
    if (samp_freq & ESP_A2D_SBC_CIE_SF_16K) return 16000;
    if (samp_freq & ESP_A2D_SBC_CIE_SF_32K) return 32000;
    if (samp_freq & ESP_A2D_SBC_CIE_SF_44K) return 44100;
    if (samp_freq & ESP_A2D_SBC_CIE_SF_48K) return 48000;
    return 0;
}

/* ------------------------------------------------------------------ */
/* Audio data                                                          */
/* ------------------------------------------------------------------ */

static void bt_a2d_data_cb(const uint8_t *data, uint32_t len)
{
    if (!s_capture || len < 4) {
        return;
    }

    const int16_t *pcm = (const int16_t *)data;
    size_t frames = len / 4;   /* 2 channels * int16 */

    uint8_t vol = s_volume;
    if (vol >= VOLUME_MAX) {
        /* Unity gain: hand the decoder's own buffer straight through with no
         * copy and no arithmetic. */
        audio_out_write(pcm, frames);
        return;
    }

    /*
     * Attenuate in software.
     *
     * This is not optional politeness: we advertise AVRCP absolute-volume
     * support below, which tells the phone "do not attenuate, the sink will".
     * If we then ignored the volume it sent, the phone's volume slider would do
     * nothing and everything would play at full scale.
     *
     * Done in chunks through a small scratch buffer rather than one big one --
     * the callback's length varies with the peer's MTU and we would rather not
     * size a worst-case buffer we mostly do not use.
     */
    int32_t gain_q15 = ((int32_t)vol * 32767) / VOLUME_MAX;
    size_t done = 0;
    while (done < frames) {
        size_t n = frames - done;
        if (n > SCALE_CHUNK_FRAMES) {
            n = SCALE_CHUNK_FRAMES;
        }
        for (size_t i = 0; i < n * 2; i++) {
            s_scale_buf[i] = (int16_t)(((int32_t)pcm[(done * 2) + i] * gain_q15) >> 15);
        }
        audio_out_write(s_scale_buf, n);
        done += n;
    }
}

/* ------------------------------------------------------------------ */
/* A2DP events                                                         */
/* ------------------------------------------------------------------ */

static void bt_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param)
{
    switch (event) {
    case ESP_A2D_CONNECTION_STATE_EVT: {
        esp_a2d_connection_state_t st = param->conn_stat.state;
        if (st == ESP_A2D_CONNECTION_STATE_CONNECTED) {
            memcpy(s_peer_bda, param->conn_stat.remote_bda, sizeof(esp_bd_addr_t));
            if (s_peer_name[0] == '\0') {
                set_peer_name_from_bda(s_peer_bda);
            }
            s_connected = true;
            ESP_LOGI(TAG, "connected to %s", s_peer_name);

            audio_evt_bt_t evt = { 0 };
            memcpy(evt.bda, s_peer_bda, sizeof(evt.bda));
            strlcpy(evt.name, s_peer_name, sizeof(evt.name));
            esp_event_post(AUDIO_EVENT, AUDIO_EVENT_BT_CONNECTED, &evt, sizeof(evt), 0);

            /* Ask GAP for the friendly name; the reply upgrades the display
             * from a bare MAC address to something a person recognises. */
            esp_bt_gap_read_remote_name(s_peer_bda);
        } else if (st == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
            s_connected = false;
            s_streaming = false;
            s_rate = 0;
            s_peer_name[0] = '\0';
            ESP_LOGI(TAG, "disconnected");
            esp_event_post(AUDIO_EVENT, AUDIO_EVENT_BT_DISCONNECTED, NULL, 0, 0);
            /* Become findable again so the phone can simply reconnect. */
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        }
        break;
    }

    case ESP_A2D_AUDIO_STATE_EVT: {
        bool started = (param->audio_stat.state == ESP_A2D_AUDIO_STATE_STARTED);
        if (started != s_streaming) {
            s_streaming = started;
            ESP_LOGI(TAG, "stream %s", started ? "started" : "stopped");
            if (started) {
                audio_evt_stream_t evt = { .sample_rate_hz = s_rate };
                esp_event_post(AUDIO_EVENT, AUDIO_EVENT_BT_STREAM_START, &evt, sizeof(evt), 0);
            } else {
                esp_event_post(AUDIO_EVENT, AUDIO_EVENT_BT_STREAM_STOP, NULL, 0, 0);
            }
        }
        break;
    }

    case ESP_A2D_AUDIO_CFG_EVT: {
        if (param->audio_cfg.mcc.type != ESP_A2D_MCT_SBC) {
            ESP_LOGW(TAG, "unexpected codec type %d", param->audio_cfg.mcc.type);
            break;
        }
        uint32_t hz = sbc_samp_freq_to_hz(param->audio_cfg.mcc.cie.sbc_info.samp_freq);
        if (hz && hz != s_rate) {
            s_rate = hz;
            ESP_LOGI(TAG, "stream configured at %" PRIu32 " Hz", hz);
        }
        break;
    }

    default:
        break;
    }
}

/* ------------------------------------------------------------------ */
/* AVRCP                                                               */
/* ------------------------------------------------------------------ */

static void bt_avrc_ct_cb(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param)
{
    /*
     * Controller role is registered but deliberately near-empty. It costs
     * almost nothing to have running and it is the hook that track metadata and
     * transport controls will attach to later; without it registered at connect
     * time, adding those means renegotiating with the peer.
     */
    switch (event) {
    case ESP_AVRC_CT_CONNECTION_STATE_EVT:
        ESP_LOGI(TAG, "AVRCP CT %s",
                 param->conn_stat.connected ? "connected" : "disconnected");
        break;
    default:
        break;
    }
}

static void bt_avrc_tg_cb(esp_avrc_tg_cb_event_t event, esp_avrc_tg_cb_param_t *param)
{
    switch (event) {
    case ESP_AVRC_TG_SET_ABSOLUTE_VOLUME_CMD_EVT:
        s_volume = param->set_abs_vol.volume;
        ESP_LOGI(TAG, "volume set by peer to %u%%",
                 (unsigned)((uint32_t)s_volume * 100 / VOLUME_MAX));
        break;

    case ESP_AVRC_TG_REGISTER_NOTIFICATION_EVT:
        if (param->reg_ntf.event_id == ESP_AVRC_RN_VOLUME_CHANGE) {
            /* Interim response: "registered, current value is this". We only
             * ever change volume on the peer's instruction, so there is never a
             * CHANGED response to follow up with. */
            esp_avrc_rn_param_t rn = { .volume = s_volume };
            esp_avrc_tg_send_rn_rsp(ESP_AVRC_RN_VOLUME_CHANGE, ESP_AVRC_RN_RSP_INTERIM, &rn);
        }
        break;

    default:
        break;
    }
}

/* ------------------------------------------------------------------ */
/* GAP                                                                 */
/* ------------------------------------------------------------------ */

static void bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_BT_GAP_AUTH_CMPL_EVT:
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            strlcpy(s_peer_name, (const char *)param->auth_cmpl.device_name, sizeof(s_peer_name));
            ESP_LOGI(TAG, "authenticated: %s", s_peer_name);
        } else {
            ESP_LOGE(TAG, "authentication failed, status %d", param->auth_cmpl.stat);
        }
        break;

    case ESP_BT_GAP_READ_REMOTE_NAME_EVT:
        if (param->read_rmt_name.stat == ESP_BT_STATUS_SUCCESS) {
            strlcpy(s_peer_name, (const char *)param->read_rmt_name.rmt_name, sizeof(s_peer_name));
            ESP_LOGI(TAG, "peer name: %s", s_peer_name);
            /* Re-post so the UI can replace the MAC address it is showing. */
            audio_evt_bt_t evt = { 0 };
            memcpy(evt.bda, s_peer_bda, sizeof(evt.bda));
            strlcpy(evt.name, s_peer_name, sizeof(evt.name));
            esp_event_post(AUDIO_EVENT, AUDIO_EVENT_BT_CONNECTED, &evt, sizeof(evt), 0);
        }
        break;

    case ESP_BT_GAP_CFM_REQ_EVT:
        /* Just-works pairing: no keypad, nothing to compare against. */
        ESP_LOGI(TAG, "pairing confirm request (%" PRIu32 "), accepting",
                 param->cfm_req.num_val);
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;

    case ESP_BT_GAP_KEY_NOTIF_EVT:
        ESP_LOGI(TAG, "pairing passkey: %" PRIu32, param->key_notif.passkey);
        break;

    default:
        break;
    }
}

/* ------------------------------------------------------------------ */
/* Init                                                                */
/* ------------------------------------------------------------------ */

esp_err_t bt_audio_init(const char *device_name)
{
    /* Hand the BLE half of the controller's RAM back before it is claimed.
     * On a WROOM-32 with no PSRAM this is tens of KB we would otherwise lose to
     * a radio mode this design never uses -- and it must happen before
     * controller init, or the memory is already gone. */
    ESP_RETURN_ON_ERROR(esp_bt_controller_mem_release(ESP_BT_MODE_BLE), TAG, "mem_release");

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    bt_cfg.mode = ESP_BT_MODE_CLASSIC_BT;
    ESP_RETURN_ON_ERROR(esp_bt_controller_init(&bt_cfg), TAG, "controller_init");
    ESP_RETURN_ON_ERROR(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT), TAG, "controller_enable");

    ESP_RETURN_ON_ERROR(esp_bluedroid_init(), TAG, "bluedroid_init");
    ESP_RETURN_ON_ERROR(esp_bluedroid_enable(), TAG, "bluedroid_enable");

    ESP_RETURN_ON_ERROR(esp_bt_gap_register_callback(bt_gap_cb), TAG, "gap_register");

    /* Secure Simple Pairing, no IO: the box has a screen but no way to type, so
     * "just works" is the only honest capability to advertise. */
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_NONE;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));

    ESP_RETURN_ON_ERROR(esp_bt_gap_set_device_name(device_name), TAG, "set_device_name");

    /* AVRCP first, then A2DP: the peer inspects our SDP records when it
     * connects, and they should all be in place before we are discoverable. */
    ESP_RETURN_ON_ERROR(esp_avrc_ct_register_callback(bt_avrc_ct_cb), TAG, "avrc_ct_register");
    ESP_RETURN_ON_ERROR(esp_avrc_ct_init(), TAG, "avrc_ct_init");

    ESP_RETURN_ON_ERROR(esp_avrc_tg_register_callback(bt_avrc_tg_cb), TAG, "avrc_tg_register");
    ESP_RETURN_ON_ERROR(esp_avrc_tg_init(), TAG, "avrc_tg_init");

    esp_avrc_rn_evt_cap_mask_t evt_set = { 0 };
    esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &evt_set, ESP_AVRC_RN_VOLUME_CHANGE);
    ESP_RETURN_ON_ERROR(esp_avrc_tg_set_rn_evt_cap(&evt_set), TAG, "avrc_tg_rn_cap");

    ESP_RETURN_ON_ERROR(esp_a2d_register_callback(bt_a2d_cb), TAG, "a2d_register");
    ESP_RETURN_ON_ERROR(esp_a2d_sink_register_data_callback(bt_a2d_data_cb), TAG, "a2d_data_cb");
    ESP_RETURN_ON_ERROR(esp_a2d_sink_init(), TAG, "a2d_sink_init");

    ESP_RETURN_ON_ERROR(esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE),
                        TAG, "set_scan_mode");

    ESP_LOGI(TAG, "A2DP sink up, discoverable as \"%s\"", device_name);
    return ESP_OK;
}

/* ------------------------------------------------------------------ */

bool bt_audio_is_connected(void)  { return s_connected; }
bool bt_audio_is_streaming(void)  { return s_streaming; }
uint32_t bt_audio_get_rate(void)  { return s_rate; }
uint8_t bt_audio_get_volume(void) { return s_volume; }

const char *bt_audio_peer_name(void)
{
    return s_peer_name[0] ? s_peer_name : "";
}

void bt_audio_set_capture(bool enable)
{
    s_capture = enable;
}

esp_err_t bt_audio_disconnect(void)
{
    if (!s_connected) {
        return ESP_ERR_INVALID_STATE;
    }
    return esp_a2d_sink_disconnect(s_peer_bda);
}
