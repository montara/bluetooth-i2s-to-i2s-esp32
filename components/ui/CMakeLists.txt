idf_component_register(
    SRCS "ui.c"
    INCLUDE_DIRS "include"
    REQUIRES audio_events esp_lcd esp_driver_spi esp_driver_gpio esp_event
    PRIV_REQUIRES board ui_input audio_router audio_out bt_audio
)
