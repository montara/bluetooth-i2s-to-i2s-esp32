#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"

#include "ui_input.h"
#include "app_pins.h"

static const char *TAG = "ui_input";

/* Most EC11s produce a full quadrature cycle -- four counted transitions --
 * between mechanical detents. Users think in clicks, so divide. */
#define COUNTS_PER_DETENT   4

/* PCNT's glitch filter runs off APB (80 MHz) and its width field tops out
 * around 12.7 us, so this is essentially "as much filtering as the hardware
 * will give". It takes the edge off contact bounce but is not a substitute for
 * an RC filter on the board: if detents feel jumpy, add 10k/100nF on A and B
 * before reaching for a software workaround. */
#define ENCODER_GLITCH_NS   12000

#define DEBOUNCE_POLL_MS    10
#define DEBOUNCE_STABLE     2       /* consecutive agreeing polls */

static pcnt_unit_handle_t s_pcnt;
static pcnt_channel_handle_t s_chan_a;
static pcnt_channel_handle_t s_chan_b;
static esp_timer_handle_t s_debounce_timer;

static int s_count_remainder;       /* sub-detent counts carried between reads */

/* Produced by the debounce timer, consumed by the LVGL task. Both sides do a
 * read-modify-write on it, so it needs more than `volatile`. */
static int s_pending_detents;
static portMUX_TYPE s_detent_mux = portMUX_INITIALIZER_UNLOCKED;

static bool s_select_down;
static bool s_back_down;
static int  s_select_stable;
static int  s_back_stable;

static ui_input_back_cb_t s_back_cb;
static void *s_back_ctx;

/* ------------------------------------------------------------------ */
/* Buttons                                                             */
/* ------------------------------------------------------------------ */

/* Both buttons are wired to ground with the internal pull-up engaged, so a
 * pressed button reads low. */
static inline bool raw_pressed(gpio_num_t pin)
{
    return gpio_get_level(pin) == 0;
}

static void debounce_poll(void *arg)
{
    (void)arg;

    bool sel = raw_pressed(PIN_ENC_PUSH);
    if (sel == s_select_down) {
        s_select_stable = 0;
    } else if (++s_select_stable >= DEBOUNCE_STABLE) {
        s_select_down = sel;
        s_select_stable = 0;
        /* LVGL picks the new state up on its next read; nothing to post. */
    }

    bool back = raw_pressed(PIN_BTN_BACK);
    if (back == s_back_down) {
        s_back_stable = 0;
    } else if (++s_back_stable >= DEBOUNCE_STABLE) {
        s_back_down = back;
        s_back_stable = 0;
        if (back && s_back_cb) {
            /* Fires on press, not release -- back should feel immediate. */
            s_back_cb(s_back_ctx);
        }
    }

    /*
     * Drain the encoder here rather than inside the LVGL read callback.
     *
     * Reading and clearing PCNT at a steady 10 ms keeps the window between the
     * two tiny and predictable. Doing it from LVGL's read callback would mean
     * draining at whatever rate LVGL happens to poll, which varies with how
     * busy the UI is -- exactly when a fast spin is most likely to be dropped.
     */
    int count = 0;
    pcnt_unit_get_count(s_pcnt, &count);
    pcnt_unit_clear_count(s_pcnt);

    if (count != 0) {
        s_count_remainder += count;
        int detents = s_count_remainder / COUNTS_PER_DETENT;
        if (detents != 0) {
            /* Truncation toward zero, so the leftover keeps its sign and a
             * slow turn accumulates instead of being rounded away. */
            s_count_remainder -= detents * COUNTS_PER_DETENT;
            portENTER_CRITICAL(&s_detent_mux);
            s_pending_detents += detents;
            portEXIT_CRITICAL(&s_detent_mux);
        }
    }
}

/* ------------------------------------------------------------------ */
/* LVGL glue                                                           */
/* ------------------------------------------------------------------ */

static void encoder_read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
    (void)indev;

    portENTER_CRITICAL(&s_detent_mux);
    int detents = s_pending_detents;
    s_pending_detents = 0;
    portEXIT_CRITICAL(&s_detent_mux);

    data->enc_diff = (int16_t)detents;
    data->state = s_select_down ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
}

lv_indev_t *ui_input_create_indev(lv_group_t *group)
{
    lv_indev_t *indev = lv_indev_create();
    if (!indev) {
        return NULL;
    }
    lv_indev_set_type(indev, LV_INDEV_TYPE_ENCODER);
    lv_indev_set_read_cb(indev, encoder_read_cb);
    if (group) {
        lv_indev_set_group(indev, group);
    }
    return indev;
}

void ui_input_set_back_cb(ui_input_back_cb_t cb, void *ctx)
{
    s_back_ctx = ctx;
    s_back_cb = cb;
}

/* ------------------------------------------------------------------ */
/* Setup                                                               */
/* ------------------------------------------------------------------ */

static esp_err_t encoder_setup(void)
{
    /* Limits only need to bound one 10 ms drain. Even a violent spin is a few
     * hundred counts, so there is no risk of hitting them. */
    pcnt_unit_config_t unit_cfg = {
        .high_limit = 10000,
        .low_limit  = -10000,
    };
    ESP_RETURN_ON_ERROR(pcnt_new_unit(&unit_cfg, &s_pcnt), TAG, "pcnt_new_unit");

    pcnt_glitch_filter_config_t filter = { .max_glitch_ns = ENCODER_GLITCH_NS };
    ESP_RETURN_ON_ERROR(pcnt_unit_set_glitch_filter(s_pcnt, &filter), TAG, "glitch_filter");

    /*
     * Standard 4x quadrature decode: each channel counts edges on one phase and
     * uses the other phase's level to decide the direction. Two channels
     * cross-wired this way turn A/B into a signed count, in hardware, with no
     * interrupts and no state machine in software.
     */
    pcnt_chan_config_t chan_a_cfg = { .edge_gpio_num = PIN_ENC_A, .level_gpio_num = PIN_ENC_B };
    ESP_RETURN_ON_ERROR(pcnt_new_channel(s_pcnt, &chan_a_cfg, &s_chan_a), TAG, "chan_a");
    pcnt_chan_config_t chan_b_cfg = { .edge_gpio_num = PIN_ENC_B, .level_gpio_num = PIN_ENC_A };
    ESP_RETURN_ON_ERROR(pcnt_new_channel(s_pcnt, &chan_b_cfg, &s_chan_b), TAG, "chan_b");

    ESP_RETURN_ON_ERROR(pcnt_channel_set_edge_action(s_chan_a,
                                                     PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                                                     PCNT_CHANNEL_EDGE_ACTION_INCREASE),
                        TAG, "a edge");
    ESP_RETURN_ON_ERROR(pcnt_channel_set_level_action(s_chan_a,
                                                      PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                                      PCNT_CHANNEL_LEVEL_ACTION_INVERSE),
                        TAG, "a level");
    ESP_RETURN_ON_ERROR(pcnt_channel_set_edge_action(s_chan_b,
                                                     PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                                     PCNT_CHANNEL_EDGE_ACTION_DECREASE),
                        TAG, "b edge");
    ESP_RETURN_ON_ERROR(pcnt_channel_set_level_action(s_chan_b,
                                                      PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                                      PCNT_CHANNEL_LEVEL_ACTION_INVERSE),
                        TAG, "b level");

    ESP_RETURN_ON_ERROR(pcnt_unit_enable(s_pcnt), TAG, "pcnt_enable");
    ESP_RETURN_ON_ERROR(pcnt_unit_clear_count(s_pcnt), TAG, "pcnt_clear");
    ESP_RETURN_ON_ERROR(pcnt_unit_start(s_pcnt), TAG, "pcnt_start");
    return ESP_OK;
}

esp_err_t ui_input_init(void)
{
    gpio_config_t btn_cfg = {
        .pin_bit_mask = (1ULL << PIN_ENC_PUSH) | (1ULL << PIN_BTN_BACK),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&btn_cfg), TAG, "button gpio");

    ESP_RETURN_ON_ERROR(encoder_setup(), TAG, "encoder_setup");

    const esp_timer_create_args_t timer_args = {
        .callback = debounce_poll,
        .name = "ui_input",
    };
    ESP_RETURN_ON_ERROR(esp_timer_create(&timer_args, &s_debounce_timer), TAG, "timer_create");
    ESP_RETURN_ON_ERROR(esp_timer_start_periodic(s_debounce_timer, DEBOUNCE_POLL_MS * 1000),
                        TAG, "timer_start");

    ESP_LOGI(TAG, "encoder on GPIO%d/%d, select GPIO%d, back GPIO%d",
             PIN_ENC_A, PIN_ENC_B, PIN_ENC_PUSH, PIN_BTN_BACK);
    return ESP_OK;
}
