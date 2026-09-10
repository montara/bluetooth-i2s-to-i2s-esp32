idf_component_register(
    SRCS "console_cmds.c"
    INCLUDE_DIRS "include"
    REQUIRES console
    PRIV_REQUIRES audio_events audio_out audio_router i2s_in bt_audio
)
