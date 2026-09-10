idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash esp_event
             audio_events audio_out i2s_in bt_audio audio_router ui ui_input console_cmds
)
