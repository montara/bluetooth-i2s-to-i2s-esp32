idf_component_register(
    SRCS "audio_out.c" "frame_ring.c"
    INCLUDE_DIRS "include"
    REQUIRES audio_events esp_driver_i2s esp_event esp_timer
    PRIV_REQUIRES board
)
