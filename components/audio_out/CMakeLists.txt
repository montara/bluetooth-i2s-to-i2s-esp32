idf_component_register(
    SRCS "audio_out.c" "frame_ring.c"
    INCLUDE_DIRS "include"
    REQUIRES audio_events esp_driver_i2s esp_event esp_timer
    # esp_hw_support: periph_rtc_apll_acquire(), the APLL-refcount workaround
    # in audio_out_init() -- see the comment there. Private: only audio_out.c
    # calls it, nothing in audio_out.h's public API depends on it.
    PRIV_REQUIRES board esp_hw_support
)
