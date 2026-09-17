# lvgl must be a PUBLIC requirement (REQUIRES, not PRIV_REQUIRES), even though
# only ui_input.c uses it directly. ui_input.h itself includes lvgl.h and
# exposes lv_indev_t/lv_group_t in its API, so any component that merely
# includes ui_input.h -- main.c, for the plain ui_input_init() call -- needs
# lvgl's include path too. A component manager dependency listed only in
# idf_component.yml is auto-injected as a PRIVATE requirement, which is not
# enough here: it satisfies this component's own compilation but does not
# propagate to consumers of its public header.
idf_component_register(
    SRCS "ui_input.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_pcnt esp_driver_gpio esp_timer lvgl__lvgl
    PRIV_REQUIRES board
)
