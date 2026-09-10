idf_component_register(
    SRCS "bt_audio.c"
    INCLUDE_DIRS "include"
    REQUIRES audio_events bt esp_event
    PRIV_REQUIRES audio_out
)
