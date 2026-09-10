#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>

#include "esp_console.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_heap_caps.h"

#include "console_cmds.h"
#include "audio_out.h"
#include "audio_router.h"
#include "i2s_in.h"
#include "bt_audio.h"
#include "audio_events.h"

static const char *TAG = "console";

/* ------------------------------------------------------------------ */

static int cmd_src(int argc, char **argv)
{
    if (argc == 1) {
        printf("mode:   %s\n", audio_route_mode_name(audio_router_get_mode()));
        printf("active: %s\n", audio_source_name(audio_router_get_source()));
        return 0;
    }
    if (argc != 2) {
        printf("usage: src [auto|i2s|bt]\n");
        return 1;
    }

    audio_route_mode_t mode;
    if (strcmp(argv[1], "auto") == 0) {
        mode = AUDIO_ROUTE_AUTO;
    } else if (strcmp(argv[1], "i2s") == 0) {
        mode = AUDIO_ROUTE_FORCE_I2S;
    } else if (strcmp(argv[1], "bt") == 0) {
        mode = AUDIO_ROUTE_FORCE_BT;
    } else {
        printf("unknown source '%s' (expected auto, i2s or bt)\n", argv[1]);
        return 1;
    }

    audio_router_set_mode(mode);
    printf("mode -> %s\n", audio_route_mode_name(mode));
    return 0;
}

static int cmd_stats(int argc, char **argv)
{
    (void)argc; (void)argv;

    audio_out_stats_t st;
    audio_out_get_stats(&st);

    printf("output   : %" PRIu32 " Hz, %s, servo %s\n",
           st.sample_rate_hz,
           st.streaming ? "streaming" : "idle",
           st.servo_enabled ? "on" : "off");
    printf("buffer   : %" PRIu32 "/%" PRIu32 " frames (%d%%, target 50%%)\n",
           st.fill_frames, st.capacity_frames, st.fill_pct);
    printf("mclk     : %" PRIu32 " Hz (%+" PRId32 " Hz from nominal)\n",
           st.mclk_hz, st.mclk_delta_hz);
    printf("glitches : %" PRIu32 " underrun, %" PRIu32 " overflow, %" PRIu32 " resync\n",
           st.underruns, st.overflows, st.resyncs);
    printf("i2s in   : %s", i2s_in_present() ? "present" : "absent");
    if (i2s_in_present()) {
        printf(", %" PRIu32 " Hz", i2s_in_get_rate());
    }
    printf("\n");
    printf("bluetooth: %s", bt_audio_is_connected() ? "connected" : "disconnected");
    if (bt_audio_is_connected()) {
        printf(" to %s, %s, %" PRIu32 " Hz, volume %u%%",
               bt_audio_peer_name(),
               bt_audio_is_streaming() ? "streaming" : "idle",
               bt_audio_get_rate(),
               (unsigned)((uint32_t)bt_audio_get_volume() * 100 / 127));
    }
    printf("\n");
    return 0;
}

static int cmd_servo(int argc, char **argv)
{
    if (argc == 2 && strcmp(argv[1], "on") == 0) {
        audio_out_servo_enable(true);
        printf("servo on\n");
        return 0;
    }
    if (argc == 2 && strcmp(argv[1], "off") == 0) {
        audio_out_servo_enable(false);
        printf("servo off (clock reset to nominal)\n");
        return 0;
    }
    if (argc == 3 && strcmp(argv[1], "nudge") == 0) {
        int32_t hz = (int32_t)strtol(argv[2], NULL, 10);
        audio_out_tune_manual(hz);
        printf("nudged MCLK by %+" PRId32 " Hz\n", hz);
        return 0;
    }
    printf("usage: servo [on|off|nudge <hz>]\n");
    printf("  `servo off` then `servo nudge 5000` on a running stream is the\n");
    printf("  check that tune_rate reaches the hardware -- the pitch should\n");
    printf("  shift audibly and `stats` should show the mclk offset.\n");
    return 1;
}

static int cmd_bt(int argc, char **argv)
{
    if (argc == 2 && strcmp(argv[1], "disconnect") == 0) {
        esp_err_t err = bt_audio_disconnect();
        printf("%s\n", err == ESP_OK ? "disconnecting" : esp_err_to_name(err));
        return err == ESP_OK ? 0 : 1;
    }
    printf("usage: bt disconnect\n");
    return 1;
}

static int cmd_heap(int argc, char **argv)
{
    (void)argc; (void)argv;
    printf("internal free   : %u bytes\n",
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    printf("largest block   : %u bytes\n",
           (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
    printf("minimum ever    : %u bytes\n",
           (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL));
    return 0;
}

/* ------------------------------------------------------------------ */

esp_err_t console_cmds_init(void)
{
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_cfg = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_cfg.prompt = "dac>";
    repl_cfg.max_cmdline_length = 128;

    esp_console_dev_uart_config_t uart_cfg = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_console_new_repl_uart(&uart_cfg, &repl_cfg, &repl),
                        TAG, "new_repl_uart");

    const esp_console_cmd_t cmds[] = {
        { .command = "src",   .help = "Show or force the audio source: src [auto|i2s|bt]", .func = cmd_src },
        { .command = "stats", .help = "Audio pipeline state: buffer fill, clock, glitch counters", .func = cmd_stats },
        { .command = "servo", .help = "Drift servo control: servo [on|off|nudge <hz>]", .func = cmd_servo },
        { .command = "bt",    .help = "Bluetooth control: bt disconnect", .func = cmd_bt },
        { .command = "heap",  .help = "Internal heap usage", .func = cmd_heap },
    };
    for (size_t i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++) {
        ESP_RETURN_ON_ERROR(esp_console_cmd_register(&cmds[i]), TAG, "register %s", cmds[i].command);
    }

    ESP_RETURN_ON_ERROR(esp_console_start_repl(repl), TAG, "start_repl");
    return ESP_OK;
}
