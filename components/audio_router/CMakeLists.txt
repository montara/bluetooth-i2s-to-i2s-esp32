idf_component_register(
    SRCS "audio_router.c"
    INCLUDE_DIRS "include"
    REQUIRES audio_events esp_event
    PRIV_REQUIRES audio_out i2s_in bt_audio nvs_flash
)
