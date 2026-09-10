#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_heap_caps.h"

#include "esp_lvgl_port.h"
#include "lvgl.h"

#if CONFIG_UI_PANEL_ILI9341
#include "esp_lcd_ili9341.h"
#endif

#include "ui.h"
#include "ui_input.h"
#include "audio_router.h"
#include "audio_events.h"
#include "bt_audio.h"
#include "audio_out.h"
#include "app_pins.h"

static const char *TAG = "ui";

#define LCD_H_RES           240
#define LCD_V_RES           320
#define LCD_SPI_HOST        SPI3_HOST
#define LCD_PCLK_HZ         (40 * 1000 * 1000)
#define LCD_CMD_BITS        8
#define LCD_PARAM_BITS      8

#define UI_STACK_MAX        4

/* A Kconfig bool that is 'n' leaves the symbol *undefined*, not 0, so it cannot
 * be used directly as a C expression. */
#ifdef CONFIG_UI_PANEL_INVERT_COLOR
#define UI_INVERT_COLOR     true
#else
#define UI_INVERT_COLOR     false
#endif

/* Colours */
#define COL_BG              lv_color_hex(0x101014)
#define COL_TEXT            lv_color_hex(0xf0f0f0)
#define COL_DIM             lv_color_hex(0x8a8a95)
#define COL_ACCENT          lv_color_hex(0x4aa3ff)

/* ------------------------------------------------------------------ */
/* State                                                               */
/* ------------------------------------------------------------------ */

static lv_display_t *s_disp;
static lv_indev_t   *s_indev;

/* Each screen owns its own focus group; swapping the indev's group on
 * transition is simpler and less error-prone than adding and removing objects
 * from one shared group. */
static lv_obj_t   *s_stack[UI_STACK_MAX];
static lv_group_t *s_stack_group[UI_STACK_MAX];
static int         s_depth;

/* Now Playing widgets we update in place. */
static lv_obj_t *s_lbl_source;
static lv_obj_t *s_lbl_rate;
static lv_obj_t *s_lbl_mode;
static lv_obj_t *s_lbl_bt;

static TaskHandle_t s_ui_task;

/* Deferred requests from contexts that must not touch LVGL. */
#define UI_REQ_BACK     (1u << 0)
#define UI_REQ_REFRESH  (1u << 1)

/* ------------------------------------------------------------------ */
/* Screen stack                                                        */
/* ------------------------------------------------------------------ */

static void ui_stack_push(lv_obj_t *scr, lv_group_t *group)
{
    if (s_depth >= UI_STACK_MAX) {
        ESP_LOGW(TAG, "screen stack full, refusing push");
        lv_obj_delete(scr);
        if (group) {
            lv_group_delete(group);
        }
        return;
    }
    s_stack[s_depth] = scr;
    s_stack_group[s_depth] = group;
    s_depth++;

    lv_indev_set_group(s_indev, group);
    lv_screen_load(scr);
}

static void ui_stack_pop(void)
{
    if (s_depth <= 1) {
        return;   /* already at the root; back does nothing */
    }
    lv_obj_t   *leaving = s_stack[s_depth - 1];
    lv_group_t *leaving_group = s_stack_group[s_depth - 1];
    s_depth--;

    /* Load the screen underneath *before* deleting the one we are leaving --
     * deleting the active screen out from under LVGL is a use-after-free. */
    lv_indev_set_group(s_indev, s_stack_group[s_depth - 1]);
    lv_screen_load(s_stack[s_depth - 1]);

    lv_obj_delete(leaving);
    if (leaving_group) {
        lv_group_delete(leaving_group);
    }
}

/* ------------------------------------------------------------------ */
/* Now Playing                                                         */
/* ------------------------------------------------------------------ */

static void now_playing_refresh(void)
{
    audio_source_t src = audio_router_get_source();

    lv_label_set_text(s_lbl_source, audio_source_name(src));
    lv_obj_set_style_text_color(s_lbl_source,
                               src == AUDIO_SRC_NONE ? COL_DIM : COL_ACCENT, 0);

    if (src == AUDIO_SRC_NONE) {
        lv_label_set_text(s_lbl_rate, "no signal");
    } else {
        uint32_t hz = audio_out_get_rate();
        lv_label_set_text_fmt(s_lbl_rate, "%" PRIu32 ".%" PRIu32 " kHz",
                              hz / 1000, (hz % 1000) / 100);
    }

    lv_label_set_text_fmt(s_lbl_mode, "Mode: %s",
                          audio_route_mode_name(audio_router_get_mode()));

    if (bt_audio_is_connected()) {
        const char *name = bt_audio_peer_name();
        lv_label_set_text_fmt(s_lbl_bt, LV_SYMBOL_BLUETOOTH " %s",
                              name[0] ? name : "connected");
        lv_obj_set_style_text_color(s_lbl_bt, COL_TEXT, 0);
    } else {
        lv_label_set_text(s_lbl_bt, LV_SYMBOL_BLUETOOTH " not connected");
        lv_obj_set_style_text_color(s_lbl_bt, COL_DIM, 0);
    }
}

static void source_menu_open(void);

static void now_playing_clicked(lv_event_t *e)
{
    (void)e;
    source_menu_open();
}

static lv_obj_t *now_playing_create(lv_group_t *group)
{
    lv_obj_t *scr = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr, COL_BG, 0);
    lv_obj_set_style_pad_all(scr, 12, 0);
    lv_obj_remove_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *heading = lv_label_create(scr);
    lv_label_set_text(heading, "ACTIVE INPUT");
    lv_obj_set_style_text_color(heading, COL_DIM, 0);
    lv_obj_set_style_text_font(heading, &lv_font_montserrat_14, 0);
    lv_obj_align(heading, LV_ALIGN_TOP_MID, 0, 8);

    s_lbl_source = lv_label_create(scr);
    lv_obj_set_style_text_font(s_lbl_source, &lv_font_montserrat_28, 0);
    lv_obj_align(s_lbl_source, LV_ALIGN_TOP_MID, 0, 40);

    s_lbl_rate = lv_label_create(scr);
    lv_obj_set_style_text_color(s_lbl_rate, COL_TEXT, 0);
    lv_obj_set_style_text_font(s_lbl_rate, &lv_font_montserrat_20, 0);
    lv_obj_align(s_lbl_rate, LV_ALIGN_TOP_MID, 0, 84);

    s_lbl_mode = lv_label_create(scr);
    lv_obj_set_style_text_color(s_lbl_mode, COL_DIM, 0);
    lv_obj_align(s_lbl_mode, LV_ALIGN_TOP_MID, 0, 120);

    s_lbl_bt = lv_label_create(scr);
    lv_label_set_long_mode(s_lbl_bt, LV_LABEL_LONG_MODE_DOTS);
    lv_obj_set_width(s_lbl_bt, LCD_H_RES - 32);
    lv_obj_set_style_text_align(s_lbl_bt, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(s_lbl_bt, LV_ALIGN_TOP_MID, 0, 152);

    /*
     * The home screen's one focusable object. LVGL's encoder input needs
     * something in the group to deliver its click to, and making that an actual
     * labelled button rather than an invisible hit area means the screen also
     * tells you what pressing the knob will do.
     */
    lv_obj_t *hit = lv_button_create(scr);
    lv_obj_set_size(hit, LCD_H_RES - 24, 44);
    lv_obj_align(hit, LV_ALIGN_BOTTOM_MID, 0, -8);
    lv_obj_set_style_bg_color(hit, lv_color_hex(0x1e1e26), 0);
    lv_obj_set_style_border_color(hit, COL_ACCENT, LV_STATE_FOCUSED);
    lv_obj_set_style_border_width(hit, 2, LV_STATE_FOCUSED);
    lv_obj_add_event_cb(hit, now_playing_clicked, LV_EVENT_CLICKED, NULL);

    lv_obj_t *hint = lv_label_create(hit);
    lv_label_set_text(hint, "Change source");
    lv_obj_set_style_text_color(hint, COL_TEXT, 0);
    lv_obj_center(hint);

    lv_group_add_obj(group, hit);

    now_playing_refresh();
    return scr;
}

/* ------------------------------------------------------------------ */
/* Source menu                                                         */
/* ------------------------------------------------------------------ */

static void ui_request(uint32_t bits);

static void source_chosen(lv_event_t *e)
{
    audio_route_mode_t mode = (audio_route_mode_t)(intptr_t)lv_event_get_user_data(e);
    audio_router_set_mode(mode);

    /* Pop via the UI task rather than inline. We are currently inside an event
     * callback belonging to a button on the screen that popping would delete --
     * tearing it down here would free the object LVGL is still dispatching on.
     * Deferring lets this callback return first. */
    ui_request(UI_REQ_BACK | UI_REQ_REFRESH);
}

static void source_menu_open(void)
{
    lv_group_t *group = lv_group_create();

    lv_obj_t *scr = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr, COL_BG, 0);
    lv_obj_set_style_pad_all(scr, 12, 0);
    lv_obj_remove_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *heading = lv_label_create(scr);
    lv_label_set_text(heading, "AUDIO SOURCE");
    lv_obj_set_style_text_color(heading, COL_DIM, 0);
    lv_obj_align(heading, LV_ALIGN_TOP_MID, 0, 8);

    lv_obj_t *list = lv_list_create(scr);
    lv_obj_set_size(list, LCD_H_RES - 24, 180);
    lv_obj_align(list, LV_ALIGN_TOP_MID, 0, 36);
    lv_obj_set_style_bg_color(list, lv_color_hex(0x1a1a22), 0);
    lv_obj_set_style_border_width(list, 0, 0);

    static const struct {
        audio_route_mode_t mode;
        const char *label;
    } items[] = {
        { AUDIO_ROUTE_AUTO,       "Auto"      },
        { AUDIO_ROUTE_FORCE_I2S,  "I2S In"    },
        { AUDIO_ROUTE_FORCE_BT,   "Bluetooth" },
    };

    audio_route_mode_t current = audio_router_get_mode();

    for (size_t i = 0; i < sizeof(items) / sizeof(items[0]); i++) {
        bool selected = (items[i].mode == current);
        lv_obj_t *btn = lv_list_add_button(list, selected ? LV_SYMBOL_OK : NULL,
                                           items[i].label);
        lv_obj_set_style_text_color(btn, selected ? COL_ACCENT : COL_TEXT, 0);
        lv_obj_set_style_bg_color(btn, COL_ACCENT, LV_STATE_FOCUSED);
        lv_obj_set_style_text_color(btn, lv_color_black(), LV_STATE_FOCUSED);
        lv_obj_add_event_cb(btn, source_chosen, LV_EVENT_CLICKED,
                            (void *)(intptr_t)items[i].mode);
        lv_group_add_obj(group, btn);
        if (selected) {
            lv_group_focus_obj(btn);
        }
    }

    lv_obj_t *hint = lv_label_create(scr);
    lv_label_set_text(hint, "Back button to cancel");
    lv_obj_set_style_text_color(hint, COL_DIM, 0);
    lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -8);

    ui_stack_push(scr, group);
}

/* ------------------------------------------------------------------ */
/* Event plumbing                                                      */
/* ------------------------------------------------------------------ */

/*
 * The UI task exists purely to own LVGL access.
 *
 * Both of its wake-up sources -- the esp_event loop task and the esp_timer task
 * that debounces buttons -- are forbidden from calling LVGL: it is not
 * thread-safe, and blocking on lvgl_port_lock() from a shared system task
 * stalls everything else that task serves. So they set a notification bit and
 * this task does the work with the lock held.
 */
static void ui_task(void *arg)
{
    (void)arg;
    for (;;) {
        uint32_t bits = 0;
        xTaskNotifyWait(0, UINT32_MAX, &bits, portMAX_DELAY);

        if (!lvgl_port_lock(200)) {
            ESP_LOGW(TAG, "could not take LVGL lock, dropping UI update");
            continue;
        }
        if (bits & UI_REQ_BACK) {
            ui_stack_pop();
            now_playing_refresh();
        }
        if (bits & UI_REQ_REFRESH) {
            now_playing_refresh();
        }
        lvgl_port_unlock();
    }
}

static void ui_request(uint32_t bits)
{
    if (s_ui_task) {
        xTaskNotify(s_ui_task, bits, eSetBits);
    }
}

static void on_back_pressed(void *ctx)
{
    (void)ctx;
    ui_request(UI_REQ_BACK);
}

static void on_audio_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg; (void)base; (void)id; (void)data;
    /* Every audio event the UI cares about results in the same thing: redraw
     * Now Playing from current state. No point discriminating. */
    ui_request(UI_REQ_REFRESH);
}

/* ------------------------------------------------------------------ */
/* Panel bring-up                                                      */
/* ------------------------------------------------------------------ */

static esp_err_t panel_init(esp_lcd_panel_io_handle_t *out_io,
                            esp_lcd_panel_handle_t *out_panel)
{
    gpio_config_t bl_cfg = {
        .pin_bit_mask = 1ULL << PIN_TFT_BACKLIGHT,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&bl_cfg), TAG, "backlight gpio");
    /* Plain on/off for now. The pin is LEDC-capable, so dimming is a drop-in
     * change later without moving any wires. */
    gpio_set_level(PIN_TFT_BACKLIGHT, 0);

    spi_bus_config_t bus_cfg = {
        .sclk_io_num = PIN_TFT_SCLK,
        .mosi_io_num = PIN_TFT_MOSI,
        .miso_io_num = GPIO_NUM_NC,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        /* One whole draw buffer must fit in a single transfer. */
        .max_transfer_sz = LCD_H_RES * CONFIG_UI_LVGL_BUFFER_LINES * sizeof(uint16_t),
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(LCD_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO),
                        TAG, "spi_bus_initialize");

    esp_lcd_panel_io_spi_config_t io_cfg = {
        .dc_gpio_num = PIN_TFT_DC,
        .cs_gpio_num = PIN_TFT_CS,
        .pclk_hz = LCD_PCLK_HZ,
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
        /* on_color_trans_done is deliberately left unset: esp_lvgl_port installs
         * its own to signal flush-ready, and setting one here would replace it. */
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_SPI_HOST,
                                                 &io_cfg, out_io),
                        TAG, "new_panel_io_spi");

    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = PIN_TFT_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,   /* ILI9341 modules are BGR */
        .bits_per_pixel = 16,
    };

#if CONFIG_UI_PANEL_ILI9341
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_ili9341(*out_io, &panel_cfg, out_panel),
                        TAG, "new_panel_ili9341");
#else
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_st7789(*out_io, &panel_cfg, out_panel),
                        TAG, "new_panel_st7789");
#endif

    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(*out_panel), TAG, "panel_reset");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(*out_panel), TAG, "panel_init");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_invert_color(*out_panel, UI_INVERT_COLOR),
                        TAG, "invert_color");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(*out_panel, true), TAG, "disp_on");

    gpio_set_level(PIN_TFT_BACKLIGHT, 1);
    return ESP_OK;
}

esp_err_t ui_init(void)
{
    esp_lcd_panel_io_handle_t io = NULL;
    esp_lcd_panel_handle_t panel = NULL;
    ESP_RETURN_ON_ERROR(panel_init(&io, &panel), TAG, "panel_init");

    /* LVGL lives on core 0 at low priority: it must never get in the way of the
     * audio pump on core 1, and a late redraw is invisible where a late audio
     * buffer is not. */
    lvgl_port_cfg_t port_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    port_cfg.task_priority = 2;
    port_cfg.task_affinity = 0;
    port_cfg.task_stack = 6144;
    port_cfg.timer_period_ms = 5;
    ESP_RETURN_ON_ERROR(lvgl_port_init(&port_cfg), TAG, "lvgl_port_init");

    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io,
        .panel_handle = panel,
        .buffer_size = LCD_H_RES * CONFIG_UI_LVGL_BUFFER_LINES,
        .double_buffer = true,
        .hres = LCD_H_RES,
        .vres = LCD_V_RES,
        .monochrome = false,
        .color_format = LV_COLOR_FORMAT_RGB565,
        .flags = {
            .buff_dma = true,
            .buff_spiram = false,   /* no PSRAM on a WROOM-32 */
            /* The panel wants big-endian RGB565; let the port byte-swap rather
             * than configuring LVGL globally for it. */
            .swap_bytes = true,
        },
    };
    s_disp = lvgl_port_add_disp(&disp_cfg);
    ESP_RETURN_ON_FALSE(s_disp, ESP_FAIL, TAG, "lvgl_port_add_disp");

    ESP_LOGI(TAG, "display up, %d KB free internal (largest block %d KB)",
             (int)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024),
             (int)(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL) / 1024));

    BaseType_t ok = xTaskCreatePinnedToCore(ui_task, "ui", 4096, NULL, 3, &s_ui_task, 0);
    ESP_RETURN_ON_FALSE(ok == pdPASS, ESP_ERR_NO_MEM, TAG, "ui task");

    ESP_RETURN_ON_FALSE(lvgl_port_lock(1000), ESP_ERR_TIMEOUT, TAG, "LVGL lock");
    lv_group_t *root_group = lv_group_create();
    s_indev = ui_input_create_indev(root_group);
    if (s_indev) {
        lv_obj_t *root = now_playing_create(root_group);
        ui_stack_push(root, root_group);
    }
    lvgl_port_unlock();
    ESP_RETURN_ON_FALSE(s_indev, ESP_FAIL, TAG, "encoder indev");

    /* Only now is it safe for the back button to reach the screen stack. */
    ui_input_set_back_cb(on_back_pressed, NULL);
    ESP_RETURN_ON_ERROR(esp_event_handler_register(AUDIO_EVENT, ESP_EVENT_ANY_ID,
                                                   on_audio_event, NULL),
                        TAG, "event_register");
    return ESP_OK;
}
