idf_component_register(
    SRCS "i2s_in.c" "rate_table.c"
    INCLUDE_DIRS "include"
    REQUIRES audio_events esp_driver_i2s esp_driver_pcnt esp_event esp_timer
    PRIV_REQUIRES board audio_out
)
